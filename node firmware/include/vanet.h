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

// Table of peers within the cluster
struct Cluster {

  Data peerTable[MAX_PEERS];    // peer table with defined maximum size 
  uint8_t count = 0;               // number of actual peers stored

  /*
    Returns the index of the peer with the given ID, or -1 if it is not a peer
  */
  int find(byte id);

  /*
    Adds a new peer or refreshes an existing peer's information
    Returns false if the peer is new and the cluster is full
  */
  bool updatePeer(const Data &buffer);
};

// LoRa radio, routing and Rx packet handling
struct Vanet {

  SPIClass spi{HSPI};

  // Radio interrupt flags (set by loraDio0ISR / loraDio1ISR, cleared by the code that handles them)
  volatile bool rxFlag = false;     // DIO0 RxDone: a packet is waiting in the radio FIFO
  volatile bool dio1Flag = false;   // DIO1: RxTimeout in single Rx mode, CadDetected in CAD mode (for a future MAC protocol)

  // Rx packet handling flags
  bool extract = true;
  bool pass = false;
  bool update = false;
  bool add = false;

  // Timestamps (ms)
  uint32_t lastTx = 0;          // previous VANET transmission

  // Last received packet
  Data buffer;

  // Routing
  Cluster cluster;

  /*
    Setup LoRa, attaches the DIO0/DIO1 interrupts and starts continuous Rx
    Returns true if successful
  */
  bool init(void);

  /*
    Procedure for receiving LoRa packet into buffer
    Only reads the radio when the DIO0 interrupt has set rxFlag, then returns to continuous Rx
    Returns the message, or -1 otherwise
  */
  int rx(void);

  /*
    Reads and parses the packet waiting in the radio FIFO into buffer
    Returns the message, or -1 if there was no valid packet
  */
  int readPacket(void);

  /*
    Procedure for transmitting LoRa packet, then returns to continuous Rx
    node: the node information to send (pass a copy of myself taken under myselfMutex)
  */
  void tx(byte destId, const Data &node);

  /*
    Checks the ID against myself and the cluster
    Returns 1 if it is me or broadcast, 2 if it is a peer, 0 otherwise
  */
  uint8_t analyseId(byte id);

  /*
    Sets the Rx packet handling flags from the destination and source IDs
  */
  void handleRx(byte destId, byte srcId);

  /*
    Resets all the flags
  */
  void resetRxFlags(void);
};

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern Vanet vanet;

//*************************************************************************************************************************************************
// INTERRUPTS
//*************************************************************************************************************************************************

/*
  LoRa DIO0 (RxDone): sets vanet.rxFlag
*/
void IRAM_ATTR loraDio0ISR();

/*
  LoRa DIO1 (RxTimeout / CadDetected): sets vanet.dio1Flag
*/
void IRAM_ATTR loraDio1ISR();

#endif // TVV_VANET_H
