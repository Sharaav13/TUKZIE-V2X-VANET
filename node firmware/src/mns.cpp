//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - MULTI NETWORK SYSTEM (MNS) UART
//*************************************************************************************************************************************************

#include "System.h"
#include "MNS.h"
#include "Display.h"   // uartMNSDebug() calls disp()

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

// HardwareSerial Serial(0);   // UART0
HardwareSerial uartMNS(2);  // UART2

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Setup
bool mnsOK = false;

// Debug
uint32_t lastHudTx = 0;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup MNS communication and serial debugging messages
String uartMNSInit(void) {
  uartMNS.begin(MNS_BAUD, SERIAL_8N1, MNS_RX, MNS_TX);
  mnsOK = true;
  return "HUD UART init pass.\n";
}

// Debugs if HUD Tx and Rx ports are working
// Tx and Rx are wired together
void uartMNSDebug(void) {

  if ((millis() - lastHudTx) >= MNS_TX_INTVL) {
    uartMNS.println("HUD Tx:" + (String)millis());
    lastHudTx = millis();
  }

  while (uartMNS.available() > 0) {
    disp(String((char)uartMNS.read()), 1, false, true);
  }
}
