//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - MULTI NETWORK SYSTEM (MNS) UART
//*************************************************************************************************************************************************

#include "system.h"
#include "mns.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

HardwareSerial uartMNS(2);  // UART2

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup MNS communication and serial debugging messages
// Returns true if the MNS UART driver was installed
bool mnsInit(void) {
  uartMNS.begin(MNS_BAUD, SERIAL_8N1, MNS_RX, MNS_TX);

  if (!uartMNS) {
    error = true;
    errMsg += "MNS: UART driver failed to install\n";
    return false;
  }

  return true;
}

// Transmits a packet to the MNS
void mnsTx(void) {
  char payload[128];
  snprintf(payload, sizeof(payload),
  "%06" PRIu32 ","        // date (DDMMYY)
  "%08" PRIu32 ","        // time (HHMMSSCC)
  "%.4f,"                 // lat (deg)
  "%.4f,"                 // lon (deg)
  "%.1f,"                 // alt (m)
  "%.1f,"                 // speed (kmph)
  "%.1f,"                 // heading/course (deg bearing)
  "%d,"                   // status
  "%d",                   // message code
  myself.date,
  myself.time,
  myself.lat,
  myself.lon,
  myself.alt,
  myself.speed,
  myself.course,
  myself.status,
  myself.message);
  uartMNS.println(payload);
}
