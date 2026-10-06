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

MNS:
    TX = | timestamp | location | speed | course | message |
*/

#include "system.h"
#include "gps.h"
#include "imu.h"
#include "dashboard.h"
#include "vanet.h"

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Node congifuration
Data myself;
int config[CFG_FIELDS];

// System operation
bool error = false;
String errMsg = "Errors in:\n";

// RGB LED
Adafruit_NeoPixel rgb_led = Adafruit_NeoPixel(NUM_RGB_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// Coded messages (array index is the message code)
const char *codedMessages[] = {
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
const char *codedStatus[] = {
    "Stationary",   // 0
    "Charging",     // 1
    "Delivering",   // 2
    "Taxi"          // 3
};

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Converts a node into a single CSV string
String node2Str(Data node) {
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
    node.date,
    node.time,
    node.lat, node.lon, node.alt,
    node.speed, node.course,
    node.status,
    node.tasks,
    node.battery,
    node.message);
    return String(result);
}

// Setup RGB LED
// Returns true (the NeoPixel library does not report failures)
bool rgbInit(void) {
    rgb_led.begin();
    rgb_led.setPixelColor(0, rgb_led.Color(0, 0, 0)); // Initialize all pixels to 'off'
    rgb_led.show(); // Initialize all pixels to 'off'
    return true;
}

void rgbToggle (int colour) {
    switch (colour) {
        default:
            rgb_led.setPixelColor(0, rgb_led.Color(0, 0, 0)); // Off
            break;

        case RED:
            rgb_led.setPixelColor(0, rgb_led.Color(255, 0, 0));
            break;

        case ORANGE:
            rgb_led.setPixelColor(0, rgb_led.Color(255, 128, 0));
            break;

        case YELLOW:
            rgb_led.setPixelColor(0, rgb_led.Color(255, 255, 0));
            break;

        case GREEN:
            rgb_led.setPixelColor(0, rgb_led.Color(0, 255, 0));
            break;

        case BLUE:
            rgb_led.setPixelColor(0, rgb_led.Color(0, 0, 255));
            break;

        case PURPLE:
            rgb_led.setPixelColor(0, rgb_led.Color(128, 0, 128));
            break;
    }
}
