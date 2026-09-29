//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - DEBUG
//*************************************************************************************************************************************************

#include "System.h"
#include "Debug.h"
#include "SDCard.h"
#include "GPS.h"
#include "IMU.h"
#include "MNS.h"
#include "LoRaRadio.h"

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Runs the debug routine of the selected peripheral
void debug(int peripheral) {

  switch (peripheral) {
    case SD_CARD:
      sdDebug();
      break;
    case GPS:
      gpsDebug();
      break;
    case PPS_CLOCK:
      clockDiff();
      break;
    case IMU:
      imuDebug();
      break;
    case MNS:
      uartMNSDebug();
      break;
    case LORA_RX:
      loraDebug(0);
      break;
    case LORA_TX:
      loraDebug(1);
      break;
    case LORA:
      loraDebug(2);
      break;
  }
}
