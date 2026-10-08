//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - DASHBOARD (UART)
//*************************************************************************************************************************************************

#ifndef TVV_DASHBOARD_H
#define TVV_DASHBOARD_H

#include "system.h"

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

// TUKZIE dashboard link
struct Dashboard {

  HardwareSerial uart{2};       // UART2

  // Status
  bool ok = false;

  // Timestamps (ms)
  uint32_t lastTx = 0;          // previous dashboard transmission

  // Rx
  char rxBuffer[MSG_SIZE];      // partial packet carried over between rx calls
  uint8_t rxLen = 0;            // number of characters in rxBuffer

  /*
    Setup dashboard communication and serial debugging messages
    Returns true if the dashboard UART driver was installed
  */
  bool init(void);

  /*
    Transmits myself and a table of nodes to the TUKZIE dashboard
    One node per line
  */
  void tx(Data table[], uint8_t size);

  /*
    Receives and parses a packet from the TUKZIE dashboard
    Packets are terminated by '\n'
    Returns true when a complete packet has been parsed
  */
  bool rx(void);
};

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern Dashboard dash;

#endif // TVV_DASHBOARD_H
