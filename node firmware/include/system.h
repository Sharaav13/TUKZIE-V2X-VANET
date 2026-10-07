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

// FreeRTOS tasks
#define SENSING_CORE  0     // GPS, IMU, dashboard Rx
#define NETWORK_CORE  1     // VANET Rx/Tx, SD Card logging
#define TASK_STACK    8192  // bytes per task
#define TASK_PRIORITY 1

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SYSTEM (System.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Messages
#define MSG_SIZE      128
// #define TTL 10 // depends on node sparsity

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SD CARD (Storage.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define SD_CS         10    // Pin 10
#define SD_MOSI       11    // Pin 11
#define SD_SCK        12    // Pin 12
#define SD_MISO       13    // Pin 13
#define CFG_FIELDS    3
#define CFG_PATH      "/config.txt"
#define LOG_FILE      "tvv_log"       // logged to /tvv_log.txt
#define LOG_INTVL     1000            // ms between logs of myself
#define SD_PATH_SIZE  32              // max length of "/<file>.txt"

// ------------------------------------------------------------------------------------------------------------------------------------------------
// IMU (Sensing.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define IMU_I2C_SDA   8     // Pin 8
#define IMU_I2C_SCL   9     // Pin 9

// Conversions and thresholds
#define ACC_MG_PER_LSB          0.061f
#define GYRO_MDPS_PER_LSB       4.375f    // Check datasheet
#define CALIBRATION_SAMPLES     1000.0    // Calibrating IMU
#define VIBR_G_THRESHOLD        1.8
#define VIBR_INTERVAL_THRESHOLD 500

// Reading indices (reading argument of imu.get)
#define ACC           0
#define GYRO          1

// Axis indices (index of imu.acc/imu.gyro/imu.accBias/imu.gyroBias, axis argument of imu.get)
#define X_AXIS        0
#define Y_AXIS        1
#define Z_AXIS        2

// ------------------------------------------------------------------------------------------------------------------------------------------------
// GPS & PPS (Sensing.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define GPS_RX        17    // Pin 17, to TX of GPS
#define GPS_TX        18    // Pin 18, to RX of GPS
#define PPS           16    // Pin 16
#define GPS_BAUD      9600
#define SAST_OFFSET   2     // SAST = UTC + 2hr
#define GPS_TIMEOUT   3000  // ms without UART data before the GPS is considered missing
#define GPS_MAX_AGE   2000  // ms before a GPS fix/speed is considered stale

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SPEED FUSION - KALMAN FILTER (Sensing.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define GRAVITY             9.80665f  // m/s^2
#define FUSION_PERIOD_US    10000     // IMU prediction period (100 Hz)
#define IMU_MAX_FAILS       100       // consecutive failed IMU reads before the IMU is flagged as lost

// Attitude filter
#define ATTITUDE_GYRO_WEIGHT  0.997f

// Forward acceleration
#define FWD_ACCEL_ALPHA     0.12f     // low pass filter weight
#define ACCEL_DEADBAND      0.06f     // m/s^2
#define MAX_USED_ACCEL      4.0f      // m/s^2
#define TURN_SUPPRESS_DPS   30.0f     // no IMU prediction during fast turns
#define MAX_DEAD_RECKON_MS  3000      // max IMU only prediction after the last GPS correction

// Stationary detector
#define STOP_SPEED_MPS      0.40f
#define STILL_GYRO_DPS      1.5f
#define STILL_ACCEL_TOL_G   0.06f
#define STILL_HOLD_MS       700
#define ZUPT_INTVL          100       // ms between zero velocity updates

// ------------------------------------------------------------------------------------------------------------------------------------------------
// OLED DISPLAY (Display.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define OLED_I2C_SDA  4     // Pin 4
#define OLED_I2C_SCL  5     // Pin 5
#define OLED_RESET    -1    // Pin -1, no reset pin
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SYSTEM LIGHTS - RGB LED (Display.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define RGB_LED_PIN   48    // Pin 48
#define NUM_RGB_LEDS  1
#define BRIGHTNESS    100   // (0-255)

// Colours (argument of lights.toggle)
#define RED           1
#define ORANGE        2
#define YELLOW        3
#define GREEN         4
#define BLUE          5
#define PURPLE        6

// ------------------------------------------------------------------------------------------------------------------------------------------------
// VANET (Vanet.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define LORA_SCK      39    // Pin 39
#define LORA_MISO     40    // Pin 40
#define LORA_MOSI     41    // Pin 41
#define LORA_NSS      42    // Pin 42
#define LORA_RST      38    // Pin 38
#define LORA_DIO0     21    // Pin 21, RxDone interrupt
#define LORA_DIO1     47    // Pin 47, RxTimeout / CadDetected interrupt (PLACEHOLDER: set to the wired pin)

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

#define DASH_RX       1     // Pin 1
#define DASH_TX       2     // Pin 2
#define DASH_BAUD     115200

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
#define DBG_SD_CARD   0     // SD Card write and read
#define DBG_GPS       1     // GPS data
#define DBG_PPS_CLOCK 2     // PPS clock synchronisation error
#define DBG_IMU       3     // IMU data
#define DBG_DASH      4     // Dashboard UART Tx and Rx
#define DBG_LORA_RX   5     // LoRa receive only
#define DBG_LORA_TX   6     // LoRa transmit only
#define DBG_LORA      7     // LoRa transmit and receive

// Sampling and intervals (milliseconds)
#define GPS_SAMPLING    1000
#define IMU_SAMPLING    1000
#define SD_READ_INTVL   2000
#define SD_WRITE_INTVL  1000
#define DASH_TX_INTVL   1000
#define DASH_RX_INTVL   1000

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

struct Data {

    // VANET routing
    byte dest;
    byte src;
    byte ttl;
    float distance;
    char payload[MSG_SIZE] = "";    // fixed size so Data copies never touch the heap

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
    uint8_t tasks;      // number of pending tasks
    uint8_t status;     // number coded (codedStatus)
    uint8_t message;    // number coded (codedMessages)

    // Vehicle health
    float battery;

    // Last updated time
    uint32_t date;
    uint32_t time;

    /*
      Converts the node into a single CSV string
    */
    String toStr(void);

    /*
      Calculates the 3D straight line distance (km) to another node
      lat/lon in degrees, alt in metres above the ellipsoid
    */
    float distanceTo(const Data &other);
};

// To make the parsing easier, each node is stored as an array

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

extern Data myself;

// Node congifuration
extern uint8_t config[CFG_FIELDS];  // {node ID, use, fleet number}

// System operation
extern bool error;
extern String errMsg;
extern SemaphoreHandle_t errMutex;  // protects error and errMsg, which both cores write

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

/*
  Flags an error and appends its message to errMsg
  Safe to call from both cores once errMutex has been created at the start of setup
*/
void reportError(const char *msg);

#endif // TVV_SYSTEM_H
