//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SYSTEM (DATA & SHARED GLOBAL VARIABLES)
//*************************************************************************************************************************************************

// or evaluate on time?
// have a table checker to check for stale node entries and make them expire
// Check for how to await for response: have variables and check packet headers
// Error handling
// No acknowledgements needed for dest of 0
// Signal bar
// Display message

/*
STRUCTURE OF MESSAGES:

VANET network:
    TX/RX = | dest | src | timestamp | location | speed | course | status | deliveries | battery | message |

Dashboard:
    TX = | timestamp | location | speed | course | message |
    RX = | status | deliveries | battery | message |
*/

#include "system.h"
#include "sensing.h"
#include "dashboard.h"
#include "vanet.h"

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Node congifuration
Data myself;
Data buffer;

uint8_t config[CFG_FIELDS];  // {node ID, use, fleet number}

// System operation
bool error = false;
String errMsg = "Errors in:\n";
SemaphoreHandle_t errMutex = NULL;  // protects error and errMsg, which both cores write

// Coded messages (array index is the message code)
const char *const codedMessages[] = {
    "",                                                             // 0
    "Request delivery assistance",                                  // 1
    "Assistance has been organised",                                // 2
    "Met with accident",                                            // 3
    "Outside of geofenced area",                                    // 4
    "Rough terrain is present",                                     // 5
    "Potholes are present",                                         // 6
    "Acknowledgement of received message",                          // 7
    "Please re transmit message ~ bad rssi/snr or garbled data",    // 8
    "Carried over message"                                          // 9
    "Low on battery"                                                // 10
};

// Coded status (array index is the status code)
const char *const codedStatus[] = {
    "Stationary",   // 0
    "Charging",     // 1
    "Delivering",   // 2
    "Taxi"          // 3
};

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

/*
  Flags an error and appends its message to errMsg
  Safe to call from both cores once errMutex has been created at the start of setup
*/
void reportError(const char *msg) {
    if (errMutex != NULL) {
        xSemaphoreTake(errMutex, portMAX_DELAY);
    }

    error = true;
    errMsg += msg;

    if (errMutex != NULL) {
        xSemaphoreGive(errMutex);
    }
}

/*
  Converts the node into a single CSV string
*/
String Data::toStr(void) {
    char result[128];
    snprintf(result, sizeof(result),
    "%06" PRIu32 ","        // date (DDMMYY)
    "%08" PRIu32 ","        // time (HHMMSSCC)
    "%.4f,"                 // lat (deg)
    "%.4f,"                 // lon (deg)
    "%.1f,"                 // alt (m)
    "%.1f,"                 // speed (kmph)
    "%.1f,"                 // heading/course (deg bearing)
    "%d,"                   // status
    "%d,"                   // pending deliveries
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

/*
  Calculates the 3D straight line distance (km) to another node
  lat/lon in degrees, alt in metres above the ellipsoid
  Converts both positions to Earth-Centred, Earth-Fixed (ECEF) coordinates on the WGS84 ellipsoid
*/
float Data::distanceTo(const Data &other) {

    double x[2], y[2], z[2];
    double latRad[2] = {lat * DEG_TO_RAD, other.lat * DEG_TO_RAD};
    double lonRad[2] = {lon * DEG_TO_RAD, other.lon * DEG_TO_RAD};
    double altM[2] = {alt, other.alt};

    for (int i = 0; i < 2; i++) {
        double sinLat = sin(latRad[i]);
        double cosLat = cos(latRad[i]);
        double n = WGS84_A / sqrt(1.0 - WGS84_E2 * sinLat * sinLat);   // prime vertical radius of curvature

        x[i] = (n + altM[i]) * cosLat * cos(lonRad[i]);
        y[i] = (n + altM[i]) * cosLat * sin(lonRad[i]);
        z[i] = (n * (1.0 - WGS84_E2) + altM[i]) * sinLat;
    }

    double dx = x[1] - x[0];
    double dy = y[1] - y[0];
    double dz = z[1] - z[0];

    double result = sqrt(dx * dx + dy * dy + dz * dz) / 1000.0;

    return (float)result;
}
