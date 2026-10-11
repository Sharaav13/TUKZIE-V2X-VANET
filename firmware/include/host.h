//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - HOST (UART)
//*************************************************************************************************************************************************

#ifndef TVV_HOST_H
#define TVV_HOST_H

#include "system.h"

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

// Host link (UART): the system the node is attached to, e.g. the TUKZIE dashboard
struct Host {

  HardwareSerial uart{2};       // UART2

  // Timestamps (ms)
  uint32_t lastTx = 0;          // previous host transmission
  uint32_t lastRx = 0;          // previous valid host packet (0 = none yet)

  // Rx
  char rxBuffer[MSG_SIZE];      // partial packet carried over between rx calls
  uint8_t rxLen = 0;            // number of characters in rxBuffer
  bool inPacket = false;        // a HOST_START has been seen, collecting until '\n'

  /*
    Setup the host UART
    Returns true if the UART driver was installed
  */
  bool init(void);

  /*
    Transmits myself and a table of nodes to the host, one packet per node: "$<node>\n"
    me: a copy of myself
  */
  void tx(const Data &me, Data table[], uint8_t size);

  /*
    Receives and parses a packet from the host into myself
    Packet: "$status,tasks,battery,dest,message\n" (anything before the '$' is ignored)
    message: 0 = none (leaves myself.message untouched), HOST_MSG = host message
    Returns true when a complete packet has been parsed
  */
  bool rx(void);

  /*
    Returns true if a valid host packet arrived within HOST_TIMEOUT
  */
  bool present(void);
};

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern Host host;

#endif // TVV_HOST_H
