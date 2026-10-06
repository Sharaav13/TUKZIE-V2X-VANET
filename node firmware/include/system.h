//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SYSTEM (LIBRARIES, MACROS, DATA & SHARED GLOBAL VARIABLES)
//*************************************************************************************************************************************************
/*
  Version:  0
  Date:     25/09/2026
  Authors:  Sharaav Dhebideen
*/
//*************************************************************************************************************************************************

#ifndef TVV_SYSTEM_H
#define TVV_SYSTEM_H

//*************************************************************************************************************************************************
// LIBRARIES
//*************************************************************************************************************************************************

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>                 // SD Card
#include <Adafruit_SSD1306.h>   // Display
#include <Adafruit_GFX.h>       // Display
#include <TinyGPS++.h>          // GPS
#include "LSM6.h"               // IMU
#include <LoRa.h>               // LoRa
#include <BLEDevice.h>          // TUKZIE dashboard
#include <BLEServer.h>          // TUKZIE dashboard
#include <BLE2902.h>            // TUKZIE dashboard
#include <Adafruit_NeoPixel.h>  // RGB (debugging)

//*************************************************************************************************************************************************
// MACROS
//*************************************************************************************************************************************************

// ------------------------------------------------------------------------------------------------------------------------------------------------
// MAIN (main.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define SERIAL_BAUD   115200

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SYSTEM (System.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

// RGB LED
#define RGB_LED_PIN   48    // Pin 48
#define NUM_RGB_LEDS  1
#define BRIGHTNESS    100   // (0-255)
#define RED           1
#define ORANGE        2
#define YELLOW        3
#define GREEN         4
#define BLUE          5
#define PURPLE        6

// Messages
#define MSG_SIZE      128
// #define TTL 10 // depends on node sparsity

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SD CARD (SDCard.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define SD_CS         10    // Pin 10
#define SD_MOSI       11    // Pin 11
#define SD_SCK        12    // Pin 12
#define SD_MISO       13    // Pin 13
#define CFG_FIELDS    3
#define CFG_PATH      "/config.txt"
// #define LOG_PATH  "/tvv_log.txt"

// ------------------------------------------------------------------------------------------------------------------------------------------------
// IMU (IMU.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define IMU_I2C_SDA   8     // Pin 8
#define IMU_I2C_SCL   9     // Pin 9

// Conversions and thresholds
#define ACC_MG_PER_LSB          0.061f
#define GYRO_MDPS_PER_LSB       4.375f    // Check datasheet
#define CALIBRATION_SAMPLES     1000.0    // Calibrating IMU
#define VIBR_G_THRESHOLD        1.8
#define VIBR_INTERVAL_THRESHOLD 500

// Reading indices (first index of imuVals/offsets, reading argument of imuGet)
#define ACC           0
#define GYRO          1

// Axis indices (second index of imuVals/offsets, axis argument of imuGet)
#define X_AXIS        0
#define Y_AXIS        1
#define Z_AXIS        2

// ------------------------------------------------------------------------------------------------------------------------------------------------
// GPS & PPS (GPS.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define GPS_RX        17    // Pin 17, to TX of GPS
#define GPS_TX        18    // Pin 18, to RX of GPS
#define PPS           16    // Pin 16
#define GPS_BAUD      9600
#define SAST_OFFSET   2     // SAST = UTC + 2hr

// ------------------------------------------------------------------------------------------------------------------------------------------------
// OLED DISPLAY (Display.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define OLED_I2C_SDA  4     // Pin 4
#define OLED_I2C_SCL  5     // Pin 5
#define OLED_RESET    -1    // Pin -1, no reset pin
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

// ------------------------------------------------------------------------------------------------------------------------------------------------
// MNS (MNS.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define MNS_RX        1     // Pin 1
#define MNS_TX        2     // Pin 2
#define MNS_BAUD      115200

// ------------------------------------------------------------------------------------------------------------------------------------------------
// VANET (Vanet.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define LORA_SCK      39    // Pin 39
#define LORA_MISO     40    // Pin 40
#define LORA_MOSI     41    // Pin 41
#define LORA_NSS      42    // Pin 42
#define LORA_RST      38    // Pin 38
#define LORA_DIO0     21    // Pin 21

// LoRa radio
#define LORA_SPI_FREQ 8E6       // 1E6 for breadboarding otherwise 8E6
#define LORA_FREQ     433.6E6   // Hz (433 MHz band)
#define LORA_BW       125E3     // Hz
#define LORA_SF       7         // 7 = low latency, 12 = max range
#define LORA_CR       5         // denominator of 4/x
#define LORA_SYNC     0x12      // private network
#define LORA_TXPOWER  10        // dBm via PA_BOOST
#define LORA_PREAMBLE 8         // symbols
#define BEACON_INTVL  5000      // ms

// Routing
#define MAX_PEERS     3
// #define TTL 5
// #define CLSUTER_BOUNDS 10 // km and is editable

// WGS84 ellipsoid (used by GPS)
#define WGS84_A       6378137.0           // semi-major axis (m)
#define WGS84_E2      6.69437999014e-3    // first eccentricity squared

// ------------------------------------------------------------------------------------------------------------------------------------------------
// DASHBOARD (Dashboard.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define DASH_NAME     "TUKZIE"

// Nordic UART Service (NUS) - standard BLE serial service
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // dashboard writes to node
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // node notifies dashboard

#define BLE_CHUNK     20    // bytes per notification (default BLE MTU of 23 minus 3 byte header)

// ------------------------------------------------------------------------------------------------------------------------------------------------
// DEBUG (Debug.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Peripherals to debug (argument of debug)
#define SD_CARD       0     // SD Card write and read
#define GPS           1     // GPS data
#define PPS_CLOCK     2     // PPS clock synchronisation error
#define IMU           3     // IMU data
#define MNS           4     // MNS UART Tx and Rx
#define LORA_RX       5     // LoRa receive only
#define LORA_TX       6     // LoRa transmit only
#define LORA          7     // LoRa transmit and receive

// Sampling and intervals (milliseconds)
#define GPS_SAMPLING    1000
#define IMU_SAMPLING    1000
#define SD_READ_INTVL   2000
#define SD_WRITE_INTVL  1000
#define MNS_TX_INTVL    1000
#define MNS_RX_INTVL    1000

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

struct Data {

    // VANET routing
    byte dest;
    byte src;
    byte ttl;
    float distance;
    String payload;

    // Network health
    float rssi;
    float snr;
    float freqErr;

    // Node identification
    byte id = 0;
    byte fleetId;

    // Location
    float lat;      // deg (+N/-S)
    float lon;      // deg (+E/-W)
    float alt;      // m

    // GPS fix quality
    byte sats;      // satellites in view
    float hdop;     // horizontal dilution of precision

    // Motion
    float speed;
    float course;

    // Fleet management
    int tasks;
    int status;
    int message;    // number coded

    // Vehicle health
    float battery;

    // Last updated time
    uint32_t date;
    uint32_t time;
};

// To make the parsing easier, each node is stored as an array

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

extern Data myself;
extern Data rxBuffer;

// Node congifuration
extern int config[CFG_FIELDS];

// System operation
extern bool error;
extern String errMsg;

// RGB LED
extern Adafruit_NeoPixel rgb_led;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Converts a node into a single CSV string
String node2Str(Data node);

// Setup RGB LED
// Returns true (the NeoPixel library does not report failures)
bool rgbInit(void);

// Sets the RGB LED colour
void rgbToggle(int colour);

// Compiles Tx packet content for  VANET
// For dest: 0 => Broadcast; # => Destination node ID
String makeVanetPkt(int dest, int msg);

// Compile Tx message for TUKZIE dashboard
// One node per line
String makeDashPkt(void);

// Compile Tx packet content for MNS
String makeMnsPkt(int msg);

// Parses a Rx VANET packet content
// Returns the destination (0 = broadcast, # = node ID), or -1 if parsing failed
int prsVanetPkt(String msg, Data &node);

// Parses a Rx dashboard packet content into myself
// Returns true if all fields were parsed
bool prsDashPkt(String msg);

#endif // TVV_SYSTEM_H
