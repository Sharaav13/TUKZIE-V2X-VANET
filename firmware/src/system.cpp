//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SYSTEM (DATA & SHARED GLOBAL VARIABLES)
//*************************************************************************************************************************************************

/*
STRUCTURE OF MESSAGES:

VANET network (binary Packet, 31 bytes, broadcast only, see vanet.h):
    TX/RX = | src | fleet ID | message | date | time | lat | lon | alt | speed | course | status | tasks | battery |

Host (text, each packet "$...\n"):
    TX = $<node> per node, myself first then the cluster peers (<node> = Data::toStr())
    RX = $status,tasks,battery
*/

#include "system.h"

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Node configuration
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
    char result[128];
    snprintf(result, sizeof(result),
    "%06" PRIu32 ","        // date (DDMMYY)
    "%08" PRIu32 ","        // time (HHMMSSCC)
    "%.7f,"                 // lat (deg)
    "%.7f,"                 // lon (deg)
    "%.1f,"                 // alt (m)
    "%.1f,"                 // speed (kmph)
    "%.1f,"                 // heading/course (deg bearing)
    "%d,"                   // status
    "%d,"                   // pending tasks
    "%.2f,"                 // battery percentage
    "%d",                   // message code
    date,
    time,
    lat, lon, alt,
    speed, course,
    status,
    tasks,
    battery,
    message);
    return String(result);
}
