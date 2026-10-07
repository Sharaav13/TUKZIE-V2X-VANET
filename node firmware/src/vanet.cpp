//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - VANET
//*************************************************************************************************************************************************

#include "system.h"
#include "vanet.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

Vanet vanet;

//*************************************************************************************************************************************************
// FUNCTIONS - CLUSTER
//*************************************************************************************************************************************************

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
bool Cluster::updatePeer(const Data &buffer) {

  int i = find(buffer.id);

  if (i < 0) { 
    if (count >= MAX_PEERS) {
      return false;
    }
    i = count;
    count++; // New peer
  }

  Data &peer = peerTable[i];    // reference, so the stored peer is changed
  peer.id = buffer.id;
  peer.fleetId = buffer.fleetId;
  peer.lat = buffer.lat;
  peer.lon = buffer.lon;
  peer.alt = buffer.alt;
  peer.speed = buffer.speed;
  peer.course = buffer.course;
  peer.tasks = buffer.tasks;
  peer.status = buffer.status;
  peer.message = buffer.message;
  peer.battery = buffer.battery;
  peer.date = buffer.date;
  peer.time = buffer.time;
  peer.rssi = buffer.rssi;
  peer.snr = buffer.snr;
  peer.freqErr = buffer.freqErr;
  // peer.distance = myself.distanceTo(buffer);

  return true;
}

//*************************************************************************************************************************************************
// INTERRUPTS
//*************************************************************************************************************************************************
/*
  The radio signals events on its DIO pins (mapping set by REG_DIO_MAPPING_1, 0x00 by default and in LoRa.receive):
    DIO0: RxDone in Rx mode (CadDone if remapped for CAD)
    DIO1: RxTimeout in single Rx mode, CadDetected in CAD mode (channel busy, for listen-before-talk MAC)
  The ISRs only set flags: SPI must not be used inside an interrupt, so the radio is read later in rx()
*/

/*
  LoRa DIO0 (RxDone): sets vanet.rxFlag
*/
void IRAM_ATTR loraDio0ISR() {
  vanet.rxFlag = true;
}

/*
  LoRa DIO1 (RxTimeout / CadDetected): sets vanet.dio1Flag
*/
void IRAM_ATTR loraDio1ISR() {
  vanet.dio1Flag = true;
}

//*************************************************************************************************************************************************
// FUNCTIONS - VANET
//*************************************************************************************************************************************************

/*
  Setup LoRa, attaches the DIO0/DIO1 interrupts and starts continuous Rx
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
    // LoRa.setPreambleLength(LORA_PREAMBLE);
    LoRa.enableCrc();

    randomSeed(esp_random());
    lastTx = millis() - random(0, BEACON_INTVL); // Prevents sychronous initial transmission

    // Radio event interrupts (LoRa.onReceive is not used, so DIO0 is free for our own ISR)
    pinMode(LORA_DIO0, INPUT);
    pinMode(LORA_DIO1, INPUT);
    attachInterrupt(digitalPinToInterrupt(LORA_DIO0), loraDio0ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(LORA_DIO1), loraDio1ISR, RISING);

    // Listen continuously, DIO0 rises when a packet arrives
    rxFlag = false;
    dio1Flag = false;
    LoRa.receive();

    return true;

  } else {
    reportError("LoRa: radio not detected\n");
    return false;
  }
}

/*
  Procedure for receiving LoRa packet into buffer
  Only reads the radio when the DIO0 interrupt has set rxFlag, then returns to continuous Rx
  Returns the message, or -1 otherwise
*/
int Vanet::rx(void) {

  // Nothing received, so no need to talk to the radio
  if (!rxFlag) {
    return -1;
  }
  rxFlag = false;

  int msg = readPacket();

  // Reading a packet leaves the radio in standby, so listen again
  LoRa.receive();

  return msg;
}

/*
  Reads and parses the packet waiting in the radio FIFO into buffer
  Returns the message, or -1 if there was no valid packet
*/
int Vanet::readPacket(void) {

  if (LoRa.parsePacket()) {

    resetRxFlags();   // fresh flags for each packet

    byte dest = LoRa.read();
    byte src = LoRa.read();

    handleRx(dest, src);

    if (!extract) { // Drop the packet before futher processing
      return -1;
    }
    buffer.dest = dest;
    buffer.id = src;
    buffer.fleetId = LoRa.read();
    buffer.message = LoRa.read();

    uint8_t incomingLen = LoRa.read();
    char incoming[MSG_SIZE];
    uint8_t len = 0;
    while (LoRa.available()) {
      char c = (char)LoRa.read();
      if (len < MSG_SIZE - 1) {   // keep room for the terminator, drain any excess
        incoming[len++] = c;
      }
    }
    incoming[len] = '\0';
    if (incomingLen != len) {
      return -1;  // Error in payload length not matching
    }

    if (pass) {
      strlcpy(buffer.payload, incoming, sizeof(buffer.payload)); // Stop further processing if the packet must be passed.
      return -1;
    }

    // sscanf %d needs int, so status and tasks are parsed into ints then narrowed
    int status, tasks;
    int got = sscanf(incoming, // Parse incoming payload
    "%" SCNu32 ","          // date (DDMMYY)
    "%" SCNu32 ","          // time (HHMMSSCC)
    "%f,"                   // lat (deg)
    "%f,"                   // lon (deg)
    "%f,"                   // alt (m)
    "%f,"                   // speed (kmph)
    "%f,"                   // heading/course (deg bearing)
    "%d,"                   // status
    "%d,"                   // number of pending tasks
    "%f",                   // battery percentage
    &buffer.date,
    &buffer.time,
    &buffer.lat, &buffer.lon, &buffer.alt,
    &buffer.speed, &buffer.course,
    &status,
    &tasks,
    &buffer.battery);
    if (got != 10) {
      return -1; // Not all fields of the payload were received.
    }
    buffer.status = (uint8_t)status;
    buffer.tasks = (uint8_t)tasks;

    buffer.rssi = LoRa.packetRssi();
    buffer.snr =  LoRa.packetSnr();
    buffer.freqErr = LoRa.packetFrequencyError();

    if (add || update) {
      cluster.updatePeer(buffer);
    }
    return (int)buffer.message;
  } else {
    return -1;
  }
}

/*
  Procedure for transmitting LoRa packet, then returns to continuous Rx
  node: the node information to send (pass a copy of myself taken under myselfMutex)
*/
void Vanet::tx(byte destId, const Data &node) {
  if (!LoRa.beginPacket()) {
    return; // radio busy
  } else {
    LoRa.write(destId);            // add destination address
    LoRa.write(node.id);           // add sender address
    LoRa.write(node.fleetId);      // add fleet ID
    LoRa.write(node.message);      // add message

    
    char payload[128];      // Create payload
    int n = snprintf(payload, sizeof(payload),
    "%06" PRIu32 ","        // date (DDMMYY)
    "%08" PRIu32 ","        // time (HHMMSSCC)
    "%.4f,"                 // lat (deg)
    "%.4f,"                 // lon (deg)
    "%.1f,"                 // alt (m)
    "%.1f,"                 // speed (kmph)
    "%.1f,"                 // heading/course (deg bearing)
    "%d,"                   // status
    "%d,"                   // number of pending tasks
    "%.2f",                 // battery percentage
    node.date,
    node.time,
    node.lat, node.lon, node.alt,
    node.speed, node.course,
    node.status,
    node.tasks,
    node.battery);
    size_t len = (n < (int)sizeof(payload)) ? n : sizeof(payload) - 1;   // length stored

    LoRa.write((uint8_t)len);                     // add payload length
    LoRa.write((const uint8_t *)payload, len);    // add payload
    LoRa.endPacket();                             // waits until the packet is sent (~30-100 ms at SF7)
  }

  // Transmitting leaves the radio in standby, so listen again
  LoRa.receive();
}

/*
  Checks the ID against myself and the cluster
  Returns 1 if it is me or broadcast, 2 if it is a peer, 0 otherwise
*/
uint8_t Vanet::analyseId(byte id) {
  if ((id == myself.id) || (id == 0)) {
    return 1;
  } else if (cluster.find(id) >= 0) {
    return 2;
  }
  return 0;
}

/*
  Sets the Rx packet handling flags from the destination and source IDs
*/
void Vanet::handleRx(byte destId, byte srcId) {

  uint8_t d = analyseId(destId);
  uint8_t s = analyseId(srcId);

  switch (d) {
    case 0:
      if (s != 0) {
        pass = true;
      } else {
        extract = false;
      }
      break;
    case 1:
      if (s == 0) {
        add = true;
      }
      break;
    case 2:
      pass = true;
      break;
  }

  if (s == 2) {
    update = true;
  }
}

/*
  Resets all the flags
*/
void Vanet::resetRxFlags(void) {
  extract = true;
  pass = false;
  update = false;
  add = false;
}
