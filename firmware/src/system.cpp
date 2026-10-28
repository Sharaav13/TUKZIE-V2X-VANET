//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SYSTEM (DATA & SHARED GLOBAL VARIABLES)
//*************************************************************************************************************************************************
/*
  Version:  1.0
  Date:     27/10/2026
  Authors:  Sharaav Dhebideen
*/
//*************************************************************************************************************************************************

/*
  STRUCTURE OF MESSAGES:

  VANET network (binary Packet, 25 bytes, dest = node ID or 0 for broadcast, see vanet.h):
    TX/RX = | dest | src | seq | ttl | message | date | time | lat | lon | status | tasks | battery |
    A packet addressed to another node (not me, not broadcast) is forwarded as received, with ttl - 1 (not at 0)

  Host (text, each packet "$...\n" or "#...\n"):
    TX = $id,<node> for myself, then #id,<node> per cluster peer (<node> = Data::toStr())
    RX = $status,tasks,battery,dest,message
*/

#include "system.h"

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// This node (its ID from CONFIG_FILE, the rest from the GPS, IMU and host)
Data myself;

// System operation
Errors errors;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Error Reporting
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Sets errors.check and appends text to errors.msg (safe from both cores)
*/
void reportError(const char *text) {
    if (errors.mutex != NULL) {
        xSemaphoreTake(errors.mutex, portMAX_DELAY);
    }

    errors.check = true;
    errors.msg += text;

    if (errors.mutex != NULL) {
        xSemaphoreGive(errors.mutex);
    }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Data
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Converts the node into a single CSV string
*/
String Data::toStr(void) const {
    char result[160];
    snprintf(result, sizeof(result),
    "%06" PRIu32 ","        // date (DDMMYY)
    "%08" PRIu32 ","        // time (HHMMSSCC)
    "%.7f,"                 // lat (deg)
    "%.7f,"                 // lon (deg)
    "%.1f,"                 // alt (m)
    "%.1f,"                 // speed (kmph)
    "%.1f,"                 // course (deg)
    "%d,"                   // status code
    "%d,"                   // pending tasks
    "%u,"                   // battery (%)
    "%d,"                   // message code
    "%u,"                   // sequence number
    "%d,"                   // srcRssi (dBm, my LoRa.rssi() before my last beacon, 0 for peers: not carried in packets)
    "%.1f",                 // pktRssi (dBm, how well I hear this node)
    date,
    time,
    lat, lon, alt,
    speed, course,
    status,
    tasks,
    battery,
    message,
    seq,
    srcRssi,
    pktRssi);
    return String(result);
}
