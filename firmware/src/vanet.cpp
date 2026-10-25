//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - VANET
//*************************************************************************************************************************************************

#include "system.h"
#include "vanet.h"
#include "interpret.h"
#include "sensing.h"      // utcClock, for latency timestamps

// Every node relies on this layout, so stop the build if the packet size changes
static_assert(sizeof(Packet) == 25, "Packet must be 25 bytes: update both ends and the layout comment in vanet.h");


//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

Vanet vanet;

//*************************************************************************************************************************************************
// INTERRUPTS
//*************************************************************************************************************************************************

/*
  LoRa DIO0: sets vanet.rxFlag and records the local µs of the edge (RxDone) when a packet is received
  Replaces the library's own DIO0 interrupt, which uses SPI
*/
void IRAM_ATTR loraDio0ISR() {
  int64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&vanet.dio0Mux);
  vanet.dio0Us = now;
  portEXIT_CRITICAL_ISR(&vanet.dio0Mux);
  vanet.rxFlag = true;
}

//*************************************************************************************************************************************************
// CALLBACKS
//*************************************************************************************************************************************************

/*
  Called by the LoRa library (from rx()) for each valid packet received
  size: packet length in bytes
*/
void onLoraReceive(int size) {
  vanet.readPacket(size);
}

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Packet
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Scales a decimal to a whole number and keeps it within [min, max]
  e.g. toFixed(-26.2041, 1e7, ...) => -262041000
*/
static int32_t toFixed(double value, double scale, int32_t min, int32_t max) {
  double scaled = round(value * scale);
  if (scaled < min) {
    return min;
  }
  if (scaled > max) {
    return max;
  }
  return (int32_t)scaled;
}

/*
  Tx: fills the packet from a node
*/
void Packet::pack(const Data &node) {

  // Header
  dest = node.destId;
  src = node.id;
  seq = node.seq;
  ttl = node.ttl;
  message = node.message;

  // Payload (decimals scaled to whole numbers)
  date = node.date;                                           // DDMMYY
  time = node.time;                                           // HHMMSSCC
  lat = toFixed(node.lat, 1e7, -900000000, 900000000);        // +-90 deg
  lon = toFixed(node.lon, 1e7, -1800000000, 1800000000);      // +-180 deg
  status = node.status;
  tasks = node.tasks;
  battery = node.battery;                                     // 0 to 100 %
}

/*
  Rx: copies the packet back into a node
*/
void Packet::unpack(Data &node) {

  // Header
  node.destId = dest;
  node.id = src;
  node.seq = seq;
  node.ttl = ttl;
  node.message = message;

  // Payload (scaled back to decimals)
  node.date = date;
  node.time = time;
  node.lat = (float)(lat / 1e7);    // divided as double to keep precision
  node.lon = (float)(lon / 1e7);
  node.status = status;
  node.tasks = tasks;
  node.battery = battery;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Cluster
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Returns the index of the peer with the given ID, or -1 if it is not a peer
*/
int Cluster::find(byte id) {
  for (uint8_t i = 0; i < count; i++) {
    if (peerTable[i].id == id) {
      return i;
    }
  }
  return -1;
}

/*
  Adds a new peer or refreshes an existing peer's information
  Returns false if the peer is new and the cluster is full
*/
bool Cluster::update(const Data &node) {

  int i = find(node.id);

  if (i < 0) {
    if (count >= MAX_PEERS) {
      return false;
    }
    i = count;
    count++; // New peer
  }

  peerTable[i] = node;
  lastHeard[i] = millis();

  return true;
}

/*
  Removes peers not heard for more than PEER_TIMEOUT (ms), i.e. out of range
  Returns the number removed
*/
uint8_t Cluster::removeStale(void) {

  uint8_t removed = 0;
  uint8_t i = 0;

  while (i < count) {

    if (millis() - lastHeard[i] > PEER_TIMEOUT) {
      // Shift later peers down (i now holds the next peer)
      for (uint8_t j = i; j + 1 < count; j++) {
        peerTable[j] = peerTable[j + 1];
        lastHeard[j] = lastHeard[j + 1];
      }
      count--;
      peerTable[count] = Data();    // clear the freed slot
      removed++;
    } else {
      i++;
    }
  }

  return removed;
}

/*
  Returns the average SNR (dB) of the last packet from each peer, 0 if there are no peers
*/
float Cluster::averageSnr(void) {

  if (count == 0) {
    return 0.0;
  }

  float sum = 0.0;
  for (uint8_t i = 0; i < count; i++) {
    sum += peerTable[i].snr;
  }
  return sum / count;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Vanet
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Setup LoRa, interrupts and continuous Rx
  Returns true if successful
*/
bool Vanet::init(void) {

  spi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  LoRa.setSPI(spi);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(LORA_SPI_FREQ);

  if (LoRa.begin(LORA_FREQ)) {
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setSyncWord(LORA_SYNC);
    LoRa.setTxPower(LORA_TXPOWER, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.enableCrc();

    randomSeed(esp_random());
    lastTx = millis() - random(0, BEACON_INTVL); // random first beacon, so nodes don't sync

    // Rx callback (also attaches the library's DIO0 interrupt, replaced below)
    LoRa.onReceive(onLoraReceive);

    // Replace it with one that only sets a flag (the library's uses SPI)
    pinMode(LORA_DIO0, INPUT);
    detachInterrupt(digitalPinToInterrupt(LORA_DIO0));
    attachInterrupt(digitalPinToInterrupt(LORA_DIO0), loraDio0ISR, RISING);

    // Listen continuously
    rxFlag = false;
    clearBuffer();
    LoRa.receive();

    return true;

  } else {
    reportError("LoRa: radio not detected\n");
    return false;
  }
}

/*
  Reads a received packet (if rxFlag is set) and starts the back-off
  Returns true if a valid TVV packet was read into rxBuffer
*/
bool Vanet::rx(void) {

  // Nothing received
  if (!rxFlag) {
    return false;
  }
  rxFlag = false;

  // RxDone time, taken before the radio is read
  portENTER_CRITICAL(&dio0Mux);
  int64_t rxDoneUs = dio0Us;
  portEXIT_CRITICAL(&dio0Mux);

  // Channel was busy: back off before the next Tx
  startBackoff();

  // Library reads the radio and calls onLoraReceive for a valid packet (stays in Rx)
  rxValid = false;
  LoRa.handleDio0Rise();

  // Latency: RxDone in UTC (rxBuffer is cleared and filled by onLoraReceive)
  if (rxValid) {
    rxBuffer.utc = utcClock.toUtc(rxDoneUs, rxBuffer.sync);
  }

  return rxValid;
}

/*
  Reads a packet for me into rxBuffer and adds or refreshes the sender in the cluster (only if heard straight from it)
  Queues a packet addressed to any other node to be forwarded
  Ignores packets of the wrong size, and repeats of a packet already read
  (Do not use LoRa.parsePacket() here: the library has already read the packet)
*/
void Vanet::readPacket(int size) {

  // TVV packets are always exactly this size
  if (size != (int)sizeof(Packet)) {
    return;
  }

  // For me: dest == my ID, or a broadcast (dest == 0). Any other dest: forwarded
  Packet pkt;
  uint8_t *bytes = (uint8_t *)&pkt;
  for (uint8_t i = 0; i < sizeof(Packet); i++) {
    bytes[i] = LoRa.read();
  }

  // A repeat of a packet already read (e.g. the forwarded copy of one received direct): dropped
  if (seenBefore(pkt.src, pkt.seq)) {
    rxDuplicates++;
    return;
  }

  // For another node: forward it as it is (not mine, so not read into rxBuffer)
  bool forMe = (pkt.dest == myself.id) || (pkt.dest == 0);
  if (!forMe) {
    queueForward(pkt);
    return;
  }

  // Binary to a clean rxBuffer
  clearBuffer();
  pkt.unpack(rxBuffer);

  rxBuffer.pktRssi = LoRa.packetRssi();
  rxBuffer.snr =  LoRa.packetSnr();
  rxBuffer.freqErr = LoRa.packetFrequencyError();

  // A sender heard directly is a peer (a forwarded packet has a lower ttl: its sender may be out of my range)
  if (rxBuffer.ttl >= TTL_INIT) {
    cluster.update(rxBuffer);
  }

  // Message of the latest received packet, for the display (0 = no message)
  peerMsg = rxBuffer.message;
  peerMsgId = rxBuffer.id;
  peerMsgAt = millis();

  rxValid = true;
}

/*
  Returns true if this packet (src, seq) was already read within DUP_WINDOW_MS, otherwise remembers it
*/
bool Vanet::seenBefore(uint8_t src, uint16_t seq) {

  uint32_t now = millis();

  for (uint8_t i = 0; i < DUP_HISTORY; i++) {
    if (seen[i].used && (seen[i].src == src) && (seen[i].seq == seq) && (now - seen[i].at < DUP_WINDOW_MS)) {
      return true;
    }
  }

  // New: remember it in place of the oldest
  seen[seenNext].used = true;
  seen[seenNext].src = src;
  seen[seenNext].seq = seq;
  seen[seenNext].at = now;
  seenNext = (seenNext + 1) % DUP_HISTORY;
  return false;
}

/*
  Resets rxBuffer to default values (all fields 0)
*/
void Vanet::clearBuffer(void) {
  rxBuffer = Data();    // a fresh Data: every field 0
}

/*
  Starts a new random back-off (BACKOFF_MIN_MS to BACKOFF_MAX_MS)
*/
void Vanet::startBackoff(void) {
  backoffStart = millis();
  backoffMs = random(BACKOFF_MIN_MS, BACKOFF_MAX_MS + 1);   // + 1 so max is included
}

/*
  Returns true while the back-off is running
*/
bool Vanet::backoffActive(void) {
  return (millis() - backoffStart) < backoffMs;
}

/*
  Returns true if a packet is being received right now
  (rxFlag only shows packets that have finished, the modem status also shows one still on air)
*/
bool Vanet::receiving(void) {
  uint8_t modemStat = LoRa.readRegister(LORA_REG_MODEM_STAT);
  return (modemStat & MODEM_STAT_RX_BUSY) != 0;
}

/*
  Returns true when a transmission is due (BEACON_INTVL since my last beacon or forward), the back-off has ended
  and the channel is free
*/
bool Vanet::txDue(void) {

  // Transmission not due yet (beacons and forwards share the interval, keeping the duty cycle)
  if (millis() - lastTx < BEACON_INTVL) {
    return false;
  }

  // Still backing off after a received packet
  if (backoffActive()) {
    return false;
  }

  // Another node is transmitting right now
  if (receiving()) {
    return false;
  }

  return true;
}

/*
  Returns true when a transmission is due and no forward is queued
  (forwards have priority: they carry situation-specific packets addressed to another node)
*/
bool Vanet::beaconDue(void) {

  // A queued forward takes the slot, my beacon waits for the next one
  if (fwdCount > 0) {
    return false;
  }

  return txDue();
}

/*
  Sends a packet's 25 bytes, then returns to Rx
  node: its utc (TX start), airUs and sync are filled in, for latency
  Returns true if it was sent (false if the radio was busy)
*/
static bool sendPacket(const Packet &pkt, Data &node) {

  if (!LoRa.beginPacket()) {
    return false; // radio busy
  }

  LoRa.write((const uint8_t *)&pkt, sizeof(Packet));    // send the packet's 25 bytes

  // Latency: TX start (endPacket's first SPI write puts the radio in Tx) and TX done (endPacket returns on TxDone)
  int64_t txStartUs = esp_timer_get_time();
  LoRa.endPacket();                                     // waits until sent (~62 ms at SF7)
  int64_t txDoneUs = esp_timer_get_time();

  node.utc = utcClock.toUtc(txStartUs, node.sync);
  node.airUs = (uint32_t)(txDoneUs - txStartUs);

  // Back to Rx
  LoRa.receive();

  return true;
}

/*
  Broadcasts a node as a binary packet, then returns to Rx
  node: the node to send (a copy of myself taken under myselfMutex), seq, ttl, srcRssi, utc, airUs and sync are filled in here
  Returns true if it was sent (false if the radio was busy)
*/
bool Vanet::tx(Data &node) {

  // Link quality: this beacon's sequence number and my channel RSSI (read in Rx, before beginPacket() leaves it)
  node.seq = txSeq + 1;     // wraps to 0 after 65535
  node.ttl = TTL_INIT;
  node.srcRssi = (int8_t)toFixed(LoRa.rssi(), 1.0, -128, 127);

  // Node to binary
  Packet pkt;
  pkt.pack(node);

  if (!sendPacket(pkt, node)) {
    return false;
  }

  lastTx = millis();
  txSeq = node.seq;                                     // only counted once sent
  txRssi = node.srcRssi;                                // my channel RSSI before this beacon (for the host)
  lastMsg = node.message;

  return true;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Forwarding
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Queues a packet for another node, unchanged except for its ttl, which is decremented first
  Not queued if the ttl reaches 0, or if the queue is full (counted in fwdDropped)
*/
void Vanet::queueForward(Packet pkt) {

  // Decremented to 0 (or already 0): not sent
  if (pkt.ttl <= 1) {
    return;
  }

  if (fwdCount >= FWD_QUEUE_SIZE) {
    fwdDropped++;
    return;
  }

  pkt.ttl--;
  fwdQueue[fwdCount++] = pkt;
}

/*
  Returns true when a packet is waiting to be forwarded and a transmission is due
  (forwards go before my beacon)
*/
bool Vanet::forwardDue(void) {
  return (fwdCount > 0) && txDue();
}

/*
  Sends the oldest queued packet exactly as queued, then returns to Rx
  node: filled with the packet as sent (for the log), with utc, airUs and sync of this transmission
  Returns true if it was sent (false if the radio was busy: it stays queued)
*/
bool Vanet::forward(Data &node) {

  if (fwdCount == 0) {
    return false;
  }

  Packet pkt = fwdQueue[0];

  node = Data();
  pkt.unpack(node);

  if (!sendPacket(pkt, node)) {
    return false;
  }

  // Uses the transmission slot: the next beacon or forward waits BEACON_INTVL from now
  lastTx = millis();

  // Remove it from the queue
  for (uint8_t i = 1; i < fwdCount; i++) {
    fwdQueue[i - 1] = fwdQueue[i];
  }
  fwdCount--;

  return true;
}
