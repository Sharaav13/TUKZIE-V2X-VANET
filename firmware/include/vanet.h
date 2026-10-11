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

  | dest | src | seq | rssi | message | date | time | lat | lon | status | tasks | battery |
     1      1     2     1       1        4      4     4     4      1       1        1     bytes

  seq: sender's beacon count (gaps = lost packets) | rssi: sender's LoRa.rssi() (dBm) just before it sent

  Decimals are sent as whole numbers: lat/lon x 10^7
  packed: no gaps, so the bytes on air match this layout
*/
struct __attribute__((packed)) Packet {

  // Header
  uint8_t dest;         // destination ID, 0 = broadcast (must stay first: readPacket() checks it before reading the rest)
  uint8_t src;          // sender ID
  uint16_t seq;         // sequence number, increments on every Tx (wraps at 65535)
  int8_t rssi;          // sender's LoRa.rssi() (dBm) just before it sent
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
    2. The network loop runs rx() then beaconDue() every loop, independently
    3. rx() reads the packet, and starts a random back-off (the channel was busy)
    4. beaconDue() only lets a beacon out once the back-off has ended and no packet is on air (receiving)

  Every received packet adds or refreshes its sender in the cluster

  All radio SPI runs in the network task, not in interrupts (see the TVV PATCH note in lib/LoRa/src/LoRa.h)
*/
struct Vanet {

  SPIClass spi{HSPI};

  // Radio interrupt flag
  volatile bool rxFlag = false;     // DIO0: a packet has been received
  bool rxValid = false;             // last packet was a valid TVV packet

  // Latest message from a peer (for the display)
  uint8_t peerMsg = 0;          // message code of the latest received packet, 0 = none
  uint8_t peerMsgId = 0;        // ID of the peer that sent it

  // Message in my last beacon sent (for the host)
  uint8_t lastMsg = 0;

  // Link quality
  uint16_t txSeq = 0;           // sequence number of my last beacon sent
  int8_t txRssi = 0;            // my LoRa.rssi() (dBm) sent in my last beacon

  // Timestamps (ms)
  uint32_t lastTx = 0;          // previous VANET transmission

  // Back-off (ms)
  uint32_t backoffStart = 0;    // when it started
  uint32_t backoffMs = 0;       // how long it lasts (random)

  // Last received packet
  Data rxBuffer;

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
    Reads the packet into rxBuffer and adds or refreshes the sender in the cluster
    Ignores packets of the wrong size, and packets addressed to another node
    (Do not use LoRa.parsePacket() here: the library has already read the packet)
  */
  void readPacket(int size);

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
    Returns true when a beacon is due, the back-off has ended and the channel is free
  */
  bool beaconDue(void);

  /*
    Broadcasts a node as a binary packet, then returns to Rx
    node: the node to send (a copy of myself taken under myselfMutex), seq and srcRssi are filled in here
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
  LoRa DIO0: sets vanet.rxFlag when a packet is received
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
