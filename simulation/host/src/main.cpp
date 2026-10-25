//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// SIMULATED HOST FIRMWARE (ESP32 DevKit / ESP32-D0WD)
//*************************************************************************************************************************************************
/*
  Simulates two hosts on one ESP32, each on its own UART, that can be wired to the Host UART of a TVV node.
  Each host repeats a packet from hostTxMsgs every second (TX_INTVL_MS), switching to its next packet of a shared shuffled
  cycle at the end of each hold (HOLD_MS, or random with HOLD_RANDOM; every packet once per cycle per host). Host B sends
  half a second after host A, runs half a cycle behind it and switches halfway through each of host A's holds, so the two
  hosts never send the same packet at once.
  Each host learns the ID of its node from the node's own packet ("$id,..."), so a DEST_OTHER packet can be addressed to
  the node on the other UART host.
  The bursts received on one or both of the UARTs (DISPLAY_UART) are printed to the Serial Monitor, the node and its
  peers together, only when they hold a new packet (the node resends the same packets every HOST_TX_INTVL). Each new Tx
  packet is printed once (not its repeats).

  Flash:    pio run -e host -t upload
  Monitor:  pio device monitor -e host

  Wiring (cross over, common GND):
    Host A (UART1): RX = GPIO16 <- node HOST_TX,  TX = GPIO17 -> node HOST_RX
    Host B (UART2): RX = GPIO18 <- node HOST_TX,  TX = GPIO19 -> node HOST_RX

  Host -> node: "$status,tasks,battery,dest,message\n"
  Node -> host: "$id,date,time,lat,lon,alt,speed,course,status,tasks,battery,message,seq,srcRssi,pktRssi\n" for the node,
                then the same with '#' instead of '$' per cluster peer, every HOST_TX_INTVL on the node
*/
//*************************************************************************************************************************************************

#include <Arduino.h>

//*************************************************************************************************************************************************
// MACROS
//*************************************************************************************************************************************************

// Settings
#define DISPLAY_UART  3       // UART(s) whose Rx is printed: 1 = Host A (GPIO16/17), 2 = Host B (GPIO18/19), 3 = both
#define SHOW_TX       true    // also print each new packet sent by the hosts (not the repeats)

#define SERIAL_BAUD   115200

// Host A (UART1)
#define HOST_A_RX     16      // Pin 16
#define HOST_A_TX     17      // Pin 17

// Host B (UART2)
#define HOST_B_RX     18      // Pin 18
#define HOST_B_TX     19      // Pin 19

// Must match the node (system.h)
#define HOST_BAUD     115200
#define HOST_START    '$'     // first character of every host packet (both directions), starts a burst from the node
#define HOST_PEER     '#'     // first character of each peer packet from the node
#define HOST_MSG      4       // message code set by the host (must match the node's HOST_MSG)
#define MSG_SIZE      160     // longest Rx packet kept (node packets are ~100 characters)

#define MAX_BURST     8       // packets kept per burst: the node and its peers (node MAX_PEERS = 3)
#define BURST_GAP_MS  20      // ms without Rx after a packet that ends a burst (the node sends a burst in one go)

// Each host repeats one packet every TX_INTVL_MS (keep below the node's HOST_TIMEOUT, 10 s), host B half an interval after
// host A, for a hold before switching to another packet: HOLD_MS, or with HOLD_RANDOM a random HOLD_MIN_MS..HOLD_MAX_MS
// per hold. Holds are rounded to whole Tx intervals (at least one)
#define TX_INTVL_MS   1000
#define HOLD_MS       30000
#define HOLD_RANDOM   true
#define HOLD_MIN_MS   1000
#define HOLD_MAX_MS   15000

static_assert(HOLD_MS >= TX_INTVL_MS, "HOLD_MS shorter than TX_INTVL_MS");
static_assert((HOLD_MIN_MS >= TX_INTVL_MS) && (HOLD_MAX_MS >= HOLD_MIN_MS), "HOLD_MIN_MS/HOLD_MAX_MS invalid");

#define DEST_OTHER    -1      // hostTxMsgs dest: the ID of the node on the other UART host

//*************************************************************************************************************************************************
// HOST TX MESSAGES
//*************************************************************************************************************************************************

/*
  Sent as "$status,tasks,battery,dest,message\n"
  status:   0 = Parked, 1 = Charging, 2 = Delivering, 3 = Taxi
  battery:  % (decimals allowed, the node rounds)
  dest:     node ID, 0 = broadcast, DEST_OTHER = the node on the other UART host
  message:  0 = none, HOST_MSG (4) = host message
*/
struct HostTxMsg {
  int status;
  int tasks;
  float battery;
  int dest;
  int message;
};

/*
  Every dest (broadcast, the other node, absent nodes 3 and 7) with and without a host message, covering every status,
  no/some tasks, and battery full, above, at (low) and below LOW_BATTERY_THRESHOLD (50 %), rounding up and down
  Nodes 3 and 7 are absent: their packets test that nodes drop/do not forward packets for a node outside their cluster
*/
const HostTxMsg hostTxMsgs[] = {
  {0, 0, 100,  0,          0},           // parked, full, broadcast
  {2, 8, 92.8, 0,          HOST_MSG},    // delivering, 8 tasks, broadcast, host message (rounds up)
  {1, 0, 23.5, DEST_OTHER, 0},           // charging, low battery (rounds up), to the other node
  {3, 6, 33,   DEST_OTHER, HOST_MSG},    // taxi, 6 tasks, low battery, to the other node, host message
  {3, 1, 71,   3,          0},           // taxi, 1 task, to node 3
  {2, 5, 64.2, 3,          HOST_MSG},    // delivering, 5 tasks, to node 3, host message (rounds down)
  {0, 9, 5,    7,          0},           // parked, 9 tasks, almost flat, to node 7
  {1, 4, 50,   7,          HOST_MSG},    // charging, 4 tasks, at LOW_BATTERY_THRESHOLD (low), to node 7, host message
};
const uint8_t HOST_TX_COUNT = sizeof(hostTxMsgs) / sizeof(hostTxMsgs[0]);

const char *const statusNames[] = {"Parked", "Charging", "Delivering", "Taxi"};

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

// One packet of a burst from the node (GPS, date, time, speed and course are not kept)
struct RxPkt {
  bool peer;                    // a cluster peer ('#'), otherwise the node itself ('$')
  int id;
  int status;
  int tasks;
  int battery;
  int message;
  int seq;                      // the node's last beacon, or the peer's last packet received by the node
  int rssi;                     // the node's srcRssi, or the peer's pktRssi (dBm)
};

// One simulated host on its own UART
struct SimHost {

  const char *name;
  HardwareSerial uart;
  int8_t rxPin;
  int8_t txPin;

  SimHost *other = nullptr;     // the other UART host (for DEST_OTHER)
  int nodeId = -1;              // ID of the node on this UART, from its own packet (-1 = not heard yet)

  // Tx
  uint32_t nextTx = 0;          // ms
  int16_t holdLeft = 0;         // Tx intervals before switching to another packet (the follower's is set by the leader)
  int16_t lastMsg = -1;         // index of the packet being sent
  bool leader = false;          // this host picks the holds and advances the shared cycle (txStep), otherwise a follower
  uint8_t slotOffset = 0;       // slots this host runs behind the leader in the shared cycle
  HostTxMsg txShown = {-1, -1, -1, -1, -1};   // last shown Tx packet (dest resolved)

  // Rx
  char rxBuffer[MSG_SIZE];
  uint8_t rxLen = 0;
  bool inPacket = false;
  bool peerPacket = false;      // the packet being collected started with HOST_PEER
  uint32_t lastRxMs = 0;        // ms the last packet was received
  RxPkt burst[MAX_BURST];       // burst being collected
  uint8_t burstLen = 0;
  RxPkt shown[MAX_BURST];       // last burst shown
  uint8_t shownLen = 0;

  SimHost(const char *name, uint8_t num, int8_t rxPin, int8_t txPin) : name(name), uart(num), rxPin(rxPin), txPin(txPin) {}
};

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

SimHost hostA("A", 1, HOST_A_RX, HOST_A_TX);
SimHost hostB("B", 2, HOST_B_RX, HOST_B_TX);

// Shared Tx cycle: every hostTxMsgs index once, in a random order
uint8_t txOrder[HOST_TX_COUNT];
uint8_t txStep = 0;             // the leader's slot in txOrder
bool txStarted = false;         // the leader has sent its first packet

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

/*
  Returns true if hostTxMsgs[i] can be sent now: a DEST_OTHER packet needs the other UART host's node to have been heard
  (its ID known), and a packet is never addressed to the host's own node (it would be forwarded back to it)
*/
bool txAllowed(const SimHost &h, uint8_t i) {

  if (hostTxMsgs[i].dest == DEST_OTHER) {
    return h.other->nodeId >= 0;
  }
  return !((h.nodeId >= 0) && (hostTxMsgs[i].dest == h.nodeId));
}

/*
  Shuffles txOrder into a new cycle (Fisher-Yates)
  first: the first cycle (no previous one), otherwise reshuffled until, across the change of cycle:
    - the leader does not repeat its last packet (new slot 0 vs old last slot),
    - the leader's new first packet differs from the follower's packet still being sent (old slot HALF - 1),
    - the follower does not repeat its packet when it moves to the new cycle (new slot HALF)
*/
void shuffleTxOrder(bool first) {

  const uint8_t HALF = HOST_TX_COUNT / 2;
  uint8_t oldLast = txOrder[HOST_TX_COUNT - 1];
  uint8_t oldFollower = txOrder[HALF - 1];

  for (uint8_t attempt = 0; attempt < 100; attempt++) {
    for (uint8_t i = 0; i < HOST_TX_COUNT; i++) {
      txOrder[i] = i;
    }
    for (uint8_t i = HOST_TX_COUNT - 1; i > 0; i--) {
      uint8_t j = random(i + 1);
      uint8_t t = txOrder[i];
      txOrder[i] = txOrder[j];
      txOrder[j] = t;
    }
    if (first || ((txOrder[0] != oldLast) && (txOrder[0] != oldFollower) && (txOrder[HALF] != oldFollower))) {
      return;
    }
  }
}

/*
  Returns the host's next packet from the shared cycle
  The leader moves to the next slot (reshuffling at the end of the cycle); the follower takes the slot slotOffset ahead of
  the leader's, wrapping within the cycle, so the two hosts always send different packets and each sends every packet
  once per cycle
  A packet that is not allowed (txAllowed) is replaced by a broadcast packet that differs from the other host's
*/
uint8_t nextTxMsg(SimHost &h) {

  uint8_t slot;
  if (h.leader) {
    if (txStarted && (++txStep >= HOST_TX_COUNT)) {
      shuffleTxOrder(false);
      txStep = 0;
    }
    txStarted = true;
    slot = txStep;
  } else {
    slot = (txStep + h.slotOffset) % HOST_TX_COUNT;
  }

  uint8_t i = txOrder[slot];
  if (txAllowed(h, i)) {
    return i;
  }

  uint8_t fallback = i;
  for (uint8_t j = 0; j < HOST_TX_COUNT; j++) {
    if ((hostTxMsgs[j].dest == 0) && (j != h.other->lastMsg)) {
      fallback = j;
      if (j != h.lastMsg) {
        break;
      }
    }
  }
  return fallback;
}

/*
  Returns the length of the leader's next hold in Tx intervals (at least one): HOLD_MS, or a random HOLD_MIN_MS..HOLD_MAX_MS
  with HOLD_RANDOM, rounded to the nearest interval
*/
int16_t holdTicks(void) {
  uint32_t ms = HOLD_RANDOM ? random(HOLD_MIN_MS, HOLD_MAX_MS + 1) : HOLD_MS;
  return max<int16_t>(1, (ms + TX_INTVL_MS / 2) / TX_INTVL_MS);
}

/*
  Sends the host's packet every TX_INTVL_MS, switching to its next packet of the shared shuffled cycle of hostTxMsgs at the
  end of each hold, so every packet is sent once per cycle (HOST_TX_COUNT holds) and never by both hosts at once
  The follower sends half an interval after the leader. The leader picks each hold (holdTicks = N intervals) and sets the
  follower to switch on its (N / 2 + 1)th send after it, i.e. N / 2 + 0.5 intervals into the hold (halfway, rounded up
  to the follower's next send, and always before the leader's next switch): the follower switches exactly once per leader
  hold, also with random holds of a single interval. The follower sends nothing before its first switch
  The host message is only sent with the first packet of a hold (message = 0 on the repeats), so the node broadcasts it once
*/
void hostTx(SimHost &h) {

  if ((int32_t)(millis() - h.nextTx) < 0) {
    return;
  }
  h.nextTx += TX_INTVL_MS;    // fixed phase: the follower stays half an interval after the leader

  // Switch packet at the end of the hold
  bool first;
  if (h.leader) {
    first = (h.lastMsg < 0) || (--h.holdLeft <= 0);
  } else {
    first = (h.holdLeft > 0) && (--h.holdLeft == 0);
    if (!first && (h.lastMsg < 0)) {
      return;   // waiting for the first switch set by the leader
    }
  }
  if (first) {
    h.lastMsg = nextTxMsg(h);
    if (h.leader) {
      int16_t hold = holdTicks();
      h.holdLeft = hold;
      h.other->holdLeft = hold / 2 + 1;
    }
  }

  const HostTxMsg &m = hostTxMsgs[h.lastMsg];
  int dest = (m.dest == DEST_OTHER) ? h.other->nodeId : m.dest;
  int message = first ? m.message : 0;    // host message once per hold

  char packet[48];
  snprintf(packet, sizeof(packet), "%c%d,%d,%g,%d,%d\n", HOST_START, m.status, m.tasks, m.battery, dest, message);
  h.uart.print(packet);

  // Only show a new packet that differs from the last one shown for this host
  bool changed = (m.status != h.txShown.status) || (m.tasks != h.txShown.tasks) || (m.battery != h.txShown.battery) ||
                 (dest != h.txShown.dest) || (message != h.txShown.message);

  if (SHOW_TX && first && changed) {
    h.txShown = {m.status, m.tasks, m.battery, dest, message};
    char batt[8];
    snprintf(batt, sizeof(batt), "%g%%", m.battery);
    // Same columns as printRx (the ME/Pn and id columns left blank)
    Serial.printf("%s TX %-4s %2s %-10s t=%-2d b=%-6s m=%d to=%d%s\n",
      h.name, "", "", (m.status >= 0 && m.status < 4) ? statusNames[m.status] : "?", m.tasks, batt, message,
      dest, (dest == 0) ? " (all)" : (m.dest == DEST_OTHER) ? " (other)" : "");
  }
}

/*
  Returns true if the burst holds a packet not in the last burst shown: the node has sent a new beacon (its seq changed),
  a peer packet is new (new peer or new seq), or a peer has left (fewer packets)
  Bursts that only repeat the packets already shown (the node resends them every HOST_TX_INTVL) are not new
*/
bool burstIsNew(const SimHost &h) {

  if (h.burstLen != h.shownLen) {
    return true;
  }
  for (uint8_t i = 0; i < h.burstLen; i++) {
    bool found = false;
    for (uint8_t j = 0; j < h.shownLen; j++) {
      if ((h.burst[i].peer == h.shown[j].peer) && (h.burst[i].id == h.shown[j].id) && (h.burst[i].seq == h.shown[j].seq)) {
        found = true;
        break;
      }
    }
    if (!found) {
      return true;
    }
  }
  return false;
}

/*
  Ends the burst being collected: prints all its packets together (the node, then its peers) if it is new (burstIsNew)
  Line: "<host> RX <ME|Pn> <id> <status> t=<tasks> b=<battery>% m=<message> seq=<seq> <rssi> dBm", in fixed-width columns
  shared with the TX lines (rssi: the node's own srcRssi, or how the node heard the peer, pktRssi)
*/
void flushBurst(SimHost &h, bool show) {

  if (show && burstIsNew(h)) {
    for (uint8_t i = 0; i < h.burstLen; i++) {
      const RxPkt &p = h.burst[i];
      char role[6], batt[8];
      if (p.peer) {
        snprintf(role, sizeof(role), "P%u", i);
      } else {
        snprintf(role, sizeof(role), "ME");
      }
      snprintf(batt, sizeof(batt), "%d%%", p.battery);
      Serial.printf("%s RX %-4s %2d %-10s t=%-2d b=%-6s m=%d seq=%-5d %4d dBm\n",
        h.name, role, p.id, (p.status >= 0 && p.status < 4) ? statusNames[p.status] : "?", p.tasks, batt, p.message,
        p.seq, p.rssi);
    }
    memcpy(h.shown, h.burst, sizeof(h.burst));
    h.shownLen = h.burstLen;
  }
  h.burstLen = 0;
}

/*
  Parses one packet from the node (in rxBuffer) into the burst being collected
  The node's own packet also gives its ID (nodeId)
*/
void parseRx(SimHost &h) {

  uint32_t date, time;
  float lat, lon, alt, speed, course, pktRssi;
  int id, status, tasks, battery, message, seq, srcRssi;

  int got = sscanf(h.rxBuffer,
  "%d,"                   // node ID
  "%" SCNu32 ","          // date (DDMMYY)
  "%" SCNu32 ","          // time (HHMMSSCC)
  "%f,%f,%f,"             // lat, lon, alt
  "%f,%f,"                // speed, course
  "%d,%d,%d,%d,"          // status, tasks, battery, message
  "%d,%d,%f",             // seq, srcRssi, pktRssi
  &id, &date, &time, &lat, &lon, &alt, &speed, &course,
  &status, &tasks, &battery, &message, &seq, &srcRssi, &pktRssi);

  if (got != 15) {
    Serial.printf("%s RX unparsed: %s\n", h.name, h.rxBuffer);
    return;
  }

  // The node's own packet: learn (or update) its ID
  if (!h.peerPacket && (id != h.nodeId)) {
    h.nodeId = id;
    Serial.printf("%s node ID = %d\n", h.name, id);
  }

  if (h.burstLen < MAX_BURST) {
    h.burst[h.burstLen++] = {h.peerPacket, id, status, tasks, battery, message, seq, h.peerPacket ? (int)pktRssi : srcRssi};
  }
}

/*
  Collects bursts from the node, printing each new one if show is true
  Burst: "$...\n" for the node itself, then "#...\n" per peer; it ends at the next '$' or after BURST_GAP_MS without Rx
*/
void hostRx(SimHost &h, bool show) {

  while (h.uart.available()) {
    char c = h.uart.read();

    // Start (or restart) a packet: '$' = the node (new burst), '#' = a peer
    if ((c == HOST_START) || (c == HOST_PEER)) {
      if ((c == HOST_START) && (h.burstLen > 0)) {
        flushBurst(h, show);
      }
      h.inPacket = true;
      h.peerPacket = (c == HOST_PEER);
      h.rxLen = 0;
      continue;
    }

    if (!h.inPacket) {
      continue;
    }

    if (c == '\n') {
      h.inPacket = false;
      h.rxBuffer[h.rxLen] = '\0';
      h.rxLen = 0;
      h.lastRxMs = millis();
      parseRx(h);
      continue;
    }

    if (c != '\r') {
      h.rxBuffer[h.rxLen++] = c;
    }

    // Drop oversized/garbled packets (keeps room for the terminator)
    if (h.rxLen >= MSG_SIZE - 1) {
      h.inPacket = false;
      h.rxLen = 0;
    }
  }

  // The node has finished sending the burst
  if ((h.burstLen > 0) && !h.inPacket && (millis() - h.lastRxMs >= BURST_GAP_MS)) {
    flushBurst(h, show);
  }
}

//*************************************************************************************************************************************************
// MAIN
//*************************************************************************************************************************************************

void setup() {

  Serial.begin(SERIAL_BAUD);
  delay(500);

  randomSeed(esp_random());

  hostA.other = &hostB;
  hostB.other = &hostA;

  hostA.uart.begin(HOST_BAUD, SERIAL_8N1, hostA.rxPin, hostA.txPin);
  hostB.uart.begin(HOST_BAUD, SERIAL_8N1, hostB.rxPin, hostB.txPin);

  // Host B sends half an interval after host A
  hostA.nextTx = millis();
  hostB.nextTx = hostA.nextTx + TX_INTVL_MS / 2;

  // Shared cycle: host A leads, host B runs half a cycle behind and switches halfway through each of host A's holds
  shuffleTxOrder(true);
  hostA.leader = true;
  hostB.slotOffset = HOST_TX_COUNT / 2;

  Serial.println("\nTVV simulated host");
  Serial.printf("Host A: UART1 RX=%d TX=%d | Host B: UART2 RX=%d TX=%d | %u Tx messages\n",
    HOST_A_RX, HOST_A_TX, HOST_B_RX, HOST_B_TX, HOST_TX_COUNT);
  if (HOLD_RANDOM) {
    Serial.printf("Hold: random %u-%u s\n", HOLD_MIN_MS / 1000, HOLD_MAX_MS / 1000);
  } else {
    Serial.printf("Hold: %u s\n", HOLD_MS / 1000);
  }
  Serial.printf("Displaying Rx of %s\n\n", (DISPLAY_UART == 3) ? "A and B" : (DISPLAY_UART == 1) ? hostA.name : hostB.name);
}

void loop() {

  hostTx(hostA);
  hostTx(hostB);

  hostRx(hostA, DISPLAY_UART & 1);
  hostRx(hostB, DISPLAY_UART & 2);

  delay(1);
}
