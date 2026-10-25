//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - VANET
//*************************************************************************************************************************************************

#ifndef TVV_VANET_H
#define TVV_VANET_H

#include "system.h"

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

/*
  LoRa packet: 25 bytes of binary, to one node (dest = its ID) or broadcast (dest = 0)
  Tx: pack() from a node | Rx: unpack() into a node

  | dest | src | seq | ttl | message | date | time | lat | lon | status | tasks | battery |
     1      1     2     1      1        4      4     4     4      1       1        1     bytes

  seq: sender's beacon count (gaps = lost packets)
  ttl: hops left, TTL_INIT when sent, decremented by each node that forwards it (not sent at 0)

  Decimals are sent as whole numbers: lat/lon x 10^7
  packed: no gaps, so the bytes on air match this layout
*/
struct __attribute__((packed)) Packet {

  // Header
  uint8_t dest;         // destination ID, 0 = broadcast (must stay first: readPacket() checks it before reading the rest)
  uint8_t src;          // sender ID
  uint16_t seq;         // sequence number, increments on every Tx (wraps at 65535)
  uint8_t ttl;          // hops left (TTL_INIT when sent, decremented by each forward)
  uint8_t message;      // message code

  // Payload
  uint32_t date;        // DDMMYY
  uint32_t time;        // HHMMSSCC
  int32_t lat;          // deg x 10^7
  int32_t lon;          // deg x 10^7
  uint8_t status;       // status code
  uint8_t tasks;        // pending tasks
  uint8_t battery;      // %

  /*
    Tx: fills the packet from a node
  */
  void pack(const Data &node);

  /*
    Rx: copies the packet back into a node
  */
  void unpack(Data &node);
};

// Table of peers within the cluster
struct Cluster {

  Data peerTable[MAX_PEERS];       // peer table with defined maximum size
  uint8_t count = 0;               // number of actual peers stored

  // Timestamps (ms)
  uint32_t lastHeard[MAX_PEERS];   // when each peer's last packet arrived (local time)
  uint32_t lastCheck = 0;          // previous check for stale peers

  /*
    Returns the index of the peer with the given ID, or -1 if it is not a peer
  */
  int find(byte id);

  /*
    Adds a new peer or refreshes an existing peer's information
    Returns false if the peer is new and the cluster is full
  */
  bool update(const Data &node);

  /*
    Removes peers not heard for more than PEER_TIMEOUT (ms), i.e. out of range
    Returns the number removed
  */
  uint8_t removeStale(void);

  /*
    Returns the average SNR (dB) of the last packet from each peer, 0 if there are no peers
  */
  float averageSnr(void);
};

/*
  LoRa radio, MAC and cluster

  MAC (CSMA/CA):
    1. DIO0 interrupt sets rxFlag when a packet is received
    2. The network loop runs rx(), forwardDue() and beaconDue() every loop, independently
    3. rx() reads the packet, and starts a random back-off (the channel was busy)
    4. forwardDue() and beaconDue() only let a packet out when a transmission is due (txDue()): BEACON_INTVL since my
       last beacon or forward, the back-off has ended and no packet is on air (receiving)
    5. Beacons and forwards share that interval, so every transmission is at least BEACON_INTVL apart (duty cycle).
       Forwards have priority: while one is queued it takes the slot and my beacon waits (forwards are addressed,
       situation-specific packets, e.g. arranging for two Tukzies to meet and swap parcels)

  Every packet received for me (dest = my ID or 0) straight from its sender (ttl = TTL_INIT) adds or refreshes the
  sender in the cluster. A packet forwarded to me (lower ttl) is read, but its sender is not a peer: it may be out of
  my range. A repeat of a packet already read (same src and seq) is dropped

  Forwarding (against the hidden terminal problem: a peer of mine may not hear the sender):
    A packet addressed to any other node (dest = not my ID, not 0) is queued and sent on exactly as received (same src, seq
    and payload), with its ttl decremented first. At ttl 0 it is not sent. It goes out like a beacon (in a transmission
    slot, after the back-off, with the channel free), so several forwarders do not all send at once. It is not read into
    rxBuffer (not mine)

  All radio SPI runs in the network task, not in interrupts (see the TVV PATCH note in lib/LoRa/src/LoRa.h)
*/
struct Vanet {

  SPIClass spi{HSPI};

  // Radio interrupt flag
  volatile bool rxFlag = false;     // DIO0: a packet has been received
  volatile int64_t dio0Us = 0;      // DIO0: local µs of the edge (RxDone of the packet, for latency)
  portMUX_TYPE dio0Mux = portMUX_INITIALIZER_UNLOCKED;   // dio0Us is 64 bits, not atomic
  bool rxValid = false;             // last packet was a valid TVV packet

  // Latest message from a peer (for the display)
  uint8_t peerMsg = 0;          // message code of the latest received packet, 0 = none
  uint8_t peerMsgId = 0;        // ID of the node that sent it
  uint32_t peerMsgAt = 0;       // ms when it was received

  // Message in my last beacon sent (for the host)
  uint8_t lastMsg = 0;

  // Link quality
  uint16_t txSeq = 0;           // sequence number of my last beacon sent
  int8_t txRssi = 0;            // my LoRa.rssi() (dBm) sent in my last beacon

  // Timestamps (ms)
  uint32_t lastTx = 0;          // previous VANET transmission (my beacon or a forward)

  // Back-off (ms)
  uint32_t backoffStart = 0;    // when it started
  uint32_t backoffMs = 0;       // how long it lasts (random)

  // Last received packet
  Data rxBuffer;

  // Packets for other nodes waiting to be forwarded (oldest first, ttl already decremented)
  Packet fwdQueue[FWD_QUEUE_SIZE];
  uint8_t fwdCount = 0;
  uint32_t fwdDropped = 0;      // packets not forwarded because the queue was full

  // Packets recently read (src, seq), to drop repeats (e.g. the forwarded copy of a packet already received direct)
  struct Seen {
    bool used = false;
    uint8_t src = 0;
    uint16_t seq = 0;
    uint32_t at = 0;            // ms when it was read
  };
  Seen seen[DUP_HISTORY];
  uint8_t seenNext = 0;         // slot for the next packet (oldest replaced first)
  uint32_t rxDuplicates = 0;    // repeats dropped

  // Peers
  Cluster cluster;

  /*
    Setup LoRa, interrupts and continuous Rx
    Returns true if successful
  */
  bool init(void);

  /*
    Reads a received packet (if rxFlag is set) and starts the back-off
    Returns true if a valid TVV packet was read into rxBuffer
  */
  bool rx(void);

  /*
    Reads a packet for me into rxBuffer and adds or refreshes the sender in the cluster
    Queues a packet addressed to any other node to be forwarded
    Ignores packets of the wrong size, and repeats of a packet already read
    (Do not use LoRa.parsePacket() here: the library has already read the packet)
  */
  void readPacket(int size);

  /*
    Returns true if this packet (src, seq) was already read within DUP_WINDOW_MS, otherwise remembers it
  */
  bool seenBefore(uint8_t src, uint16_t seq);

  /*
    Queues a packet for another node, unchanged except for its ttl, which is decremented first
    Not queued if the ttl reaches 0, or if the queue is full (counted in fwdDropped)
  */
  void queueForward(Packet pkt);

  /*
    Returns true when a packet is waiting to be forwarded and a transmission is due
    (forwards go before my beacon)
  */
  bool forwardDue(void);

  /*
    Sends the oldest queued packet exactly as queued, then returns to Rx
    node: filled with the packet as sent (for the log), with utc, airUs and sync of this transmission
    Returns true if it was sent (false if the radio was busy: it stays queued)
  */
  bool forward(Data &node);

  /*
    Resets rxBuffer to default values (all fields 0)
  */
  void clearBuffer(void);

  /*
    Starts a new random back-off (BACKOFF_MIN_MS to BACKOFF_MAX_MS)
  */
  void startBackoff(void);

  /*
    Returns true while the back-off is running
  */
  bool backoffActive(void);

  /*
    Returns true if a packet is being received right now
    (rxFlag only shows packets that have finished, the modem status also shows one still on air)
  */
  bool receiving(void);

  /*
    Returns true when a transmission is due (BEACON_INTVL since my last beacon or forward), the back-off has ended
    and the channel is free
  */
  bool txDue(void);

  /*
    Returns true when a transmission is due and no forward is queued
    (forwards have priority: they carry situation-specific packets addressed to another node)
  */
  bool beaconDue(void);

  /*
    Broadcasts a node as a binary packet, then returns to Rx
    node: the node to send (a copy of myself taken under myselfMutex), seq, ttl, srcRssi, utc, airUs and sync are filled in here
    Returns true if it was sent (false if the radio was busy)
  */
  bool tx(Data &node);
};

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern Vanet vanet;

//*************************************************************************************************************************************************
// INTERRUPTS
//*************************************************************************************************************************************************

/*
  LoRa DIO0: sets vanet.rxFlag and records the local µs of the edge (RxDone) when a packet is received
  Replaces the library's own DIO0 interrupt, which uses SPI
*/
void IRAM_ATTR loraDio0ISR();

//*************************************************************************************************************************************************
// CALLBACKS
//*************************************************************************************************************************************************

/*
  Called by the LoRa library (from rx()) for each valid packet received
  size: packet length in bytes
*/
void onLoraReceive(int size);

#endif // TVV_VANET_H
