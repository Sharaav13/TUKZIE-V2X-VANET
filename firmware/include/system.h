//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SYSTEM (LIBRARIES, MACROS, DATA & SHARED GLOBAL VARIABLES)
//*************************************************************************************************************************************************
/*
  Version:  0
  Date:     09/10/2026
  Authors:  Sharaav Dhebideen
*/
//*************************************************************************************************************************************************

#ifndef TVV_SYSTEM_H
#define TVV_SYSTEM_H

//*************************************************************************************************************************************************
// LIBRARIES
//*************************************************************************************************************************************************

#include <Arduino.h>
#include <inttypes.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>                 // SD Card
#include <Adafruit_SSD1306.h>   // Display
#include <TinyGPS++.h>          // GPS
#include "LSM6.h"               // IMU
#include <LoRa.h>               // LoRa
#include <Adafruit_NeoPixel.h>  // RGB (debugging)

//*************************************************************************************************************************************************
// MACROS
//*************************************************************************************************************************************************

// ------------------------------------------------------------------------------------------------------------------------------------------------
// MAIN (main.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define SERIAL_BAUD   115200

// FreeRTOS tasks
#define V2S_CORE      0     // GPS, IMU, host Rx
#define V2V_CORE      1     // VANET Rx/Tx, host Tx, OLED, RGB, SD Card logging
#define TASK_STACK    8192  // bytes per task
#define TASK_PRIORITY 1

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SYSTEM (system.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Messages
#define MSG_SIZE      128

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SD CARD (storage.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define SD_CS         10    // Pin 10
#define SD_MOSI       11    // Pin 11
#define SD_SCK        12    // Pin 12
#define SD_MISO       13    // Pin 13
#define NODE_LOG_INTVL    1000            // ms between logs of myself (storage.myselfFile)
#define SENSE_LOG_INTVL   1000            // ms between logs of sensing (storage.sensorFile)

// SD Card files
#define CONFIG_FILE       "/config.txt"         // node ID, e.g. "7"
#define DEBUG_FILE        "/debug.txt"          // SD Card testing

// SD Card logs, named with my node ID: prefix + ID + ".txt", e.g. "/vanet_log7.txt" (Storage::nameLogs)
#define VANET_LOG         "/vanet_log"          // VANET Rx, Tx and forwards
#define SENSOR_LOG        "/sensor_log"         // my GPS and IMU data
#define MYSELF_LOG        "/myself_log"         // all my data

// ------------------------------------------------------------------------------------------------------------------------------------------------
// IMU (sensing.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define IMU_I2C_SDA   8     // Pin 8
#define IMU_I2C_SCL   9     // Pin 9

// Conversions and thresholds
#define ACC_MG_PER_LSB          0.244f    // +-8 g full scale (set in IMU::init, enableDefault() would leave +-2 g)
#define GYRO_MDPS_PER_LSB       8.75f     // +-245 dps full scale (LSM6 enableDefault(): FS_G = 245 dps)
#define CALIBRATION_SAMPLES     1000.0   // Calibrating IMU

// imu.get() readings
#define ACC           0
#define GYRO          1

// Axis indices (imu arrays and imu.get())
#define X_AXIS        0
#define Y_AXIS        1
#define Z_AXIS        2

// ------------------------------------------------------------------------------------------------------------------------------------------------
// GPS (sensing.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define GPS_RX        17    // Pin 17, to TX of GPS
#define GPS_TX        18    // Pin 18, to RX of GPS
#define GPS_BAUD      9600
#define SAST_OFFSET   2     // SAST = UTC + 2hr
#define GPS_TIMEOUT   3000  // ms without data => GPS missing
#define GPS_MAX_AGE   2000  // ms before a fix/speed is stale

// PPS (UTC time base for packet latency, see UtcClock in sensing.h)
#define GPS_PPS             16          // Pin 16, from 1PPS of GPS (rising edge = start of a UTC second)
#define PPS_NMEA_MIN_US     20000       // µs, earliest a second's NMEA time arrives after its PPS edge
#define PPS_NMEA_MAX_US     900000      // µs, latest (outside this window the pairing is skipped)
#define PPS_LOCK_US         1500000     // µs since the last paired PPS edge => still locked (else holdover)
#define PPS_MAX_RATE_ERR    200e-6      // largest accepted ESP32 clock error (200 ppm)
#define DAY_US              86400000000LL   // µs in a day

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SPEED FUSION - KALMAN FILTER (sensing.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define GRAVITY             9.80665f  // m/s^2
#define FUSION_PERIOD_US    10000     // IMU prediction period (100 Hz)
#define IMU_MAX_FAILS       100       // failed reads in a row => IMU lost

// Attitude filter
#define ATTITUDE_GYRO_WEIGHT  0.997f

// Forward acceleration
#define FWD_ACCEL_ALPHA     0.12f     // low pass filter weight
#define ACCEL_DEADBAND      0.06f     // m/s^2
#define MAX_USED_ACCEL      4.0f      // m/s^2
#define TURN_SUPPRESS_DPS   30.0f     // no IMU prediction during fast turns
#define MAX_DEAD_RECKON_MS  3000      // ms of IMU-only prediction after a GPS correction

// Stationary detector
#define STOP_SPEED_MPS      0.40f
#define STILL_GYRO_DPS      1.5f
#define STILL_ACCEL_TOL_G   0.06f
#define STILL_HOLD_MS       700
#define ZUPT_INTVL          100       // ms between zero velocity updates

// ------------------------------------------------------------------------------------------------------------------------------------------------
// INTERPRET (interpret.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Coded messages
#define MSG_COUNT               5         // entries in codedMessages and msgFlags
#define HOST_MSG                4         // message code set by the host, sent before every other code
#define STATUS_COUNT            4         // entries in codedStatus

// Accident
#define TOPPLE_DURATION         10000     // ms on its side or upside down => accident
#define ACCIDENT_G              7.0       // g, |a| (all axes, gravity included) at or above this => impact => accident
#define UPSIDE_DOWN_G           -0.8      // g, raw Z at or below this => upside down (upright ~+1, upside down ~-1)
#define SIDE_G                  0.8       // g, raw |X| or |Y| at or above this => on its side (upright ~0, on its side ~1)

// Terrain
#define TERRAIN_G               0.8       // g, |vertical acceleration - 1 g| at or above this => bad terrain (bump, pothole or rough surface)
#define ROAD_MIN_SPEED          5.0       // kmph, bad terrain only while moving this fast (no false triggers when parked)

// Battery
#define LOW_BATTERY_THRESHOLD   50        // percent

// ------------------------------------------------------------------------------------------------------------------------------------------------
// OLED DISPLAY (ui.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define OLED_I2C_SDA  4     // Pin 4
#define OLED_I2C_SCL  5     // Pin 5
#define OLED_RESET    -1    // Pin -1, no reset pin
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define DISP_INTVL    1000  // ms between node view updates

// Signal bars: average SNR of my peers (dB) at or above each level, below SNR_2_BARS => 1 bar
#define SNR_5_BARS    5.0
#define SNR_4_BARS    2.0
#define SNR_3_BARS    -1.0
#define SNR_2_BARS    -4.0

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SYSTEM LIGHTS - RGB LED (ui.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define RGB_LED_PIN   48    // Pin 48
#define NUM_RGB_LEDS  1
#define BRIGHTNESS    100   // (0-255)
#define FLASH_MS      500   // ms a Tx flash stays on
#define LIGHT_HOLD_MS 200   // ms a condition light (accident, low battery, bad terrain) stays on after it was last detected

// Colours (lights.toggle)
#define OFF           0
#define RED           1
#define ORANGE        2
#define YELLOW        3
#define GREEN         4
#define BLUE          5
#define PURPLE        6
#define BRIGHTWHITE   7

#define NO_FIX_COLOUR YELLOW // steady while there is no GPS fix (between Tx flashes), OFF to disable

// ------------------------------------------------------------------------------------------------------------------------------------------------
// VANET (vanet.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define LORA_SCK      39    // Pin 39
#define LORA_MISO     40    // Pin 40
#define LORA_MOSI     41    // Pin 41
#define LORA_NSS      42    // Pin 42
#define LORA_RST      38    // Pin 38
#define LORA_DIO0     21    // Pin 21, RxDone interrupt
// LoRa radio
#define LORA_SPI_FREQ 8E6       // 1E6 for breadboarding otherwise 8E6
#define LORA_FREQ     433.2125E6  // Hz (433 MHz band)
#define LORA_BW       125E3     // Hz
#define LORA_SF       12        // 7 = low latency, 12 = max range
#define LORA_CR       5         // denominator of 4/x
#define LORA_SYNC     0x12      // private network
#define LORA_TXPOWER  10        // dBm via PA_BOOST
#define BEACON_INTVL  15000     // ms between transmissions, shared by my beacons and forwards (keeps the duty cycle at 9.9 %)

// MAC: random wait before Tx after an Rx (~4 x 1483 ms airtime at SF12)
#define BACKOFF_MIN_MS  0       // ms
#define BACKOFF_MAX_MS  6000    // ms

// MAC: radio status, checked before Tx for a packet on air
#define LORA_REG_MODEM_STAT  0x18    // RegModemStat address
#define MODEM_STAT_RX_BUSY   0x0B    // bits 0, 1, 3: signal detected, synchronised, header valid (bit 2 "RX on-going" is always set in continuous Rx)

// Cluster
#define MAX_PEERS         3
#define PEER_TIMEOUT      30000   // ms without hearing a peer => removed (out of range)
#define PEER_CHECK_INTVL  1000    // ms between checks for stale peers

// Forwarding: a packet addressed to another node (not me, not broadcast) is sent on as received (against the hidden terminal problem)
#define TTL_INIT          2       // hops a packet may make: each forward decrements it, at 0 it is not sent
#define FWD_QUEUE_SIZE    2       // packets waiting to be forwarded (a full queue drops new ones)

// Repeats: a packet (src, seq) already read is dropped, e.g. the forwarded copy of one received direct
#define DUP_HISTORY       48      // recent packets remembered (covers DUP_WINDOW_MS of a full cluster's traffic)
#define DUP_WINDOW_MS     90000   // ms a packet is remembered (longer than a forward can take to arrive: a queued
                                  // forward may wait up to FWD_QUEUE_SIZE transmission slots of BEACON_INTVL, plus
                                  // back-offs and channel-busy deferrals)

// ------------------------------------------------------------------------------------------------------------------------------------------------
// HOST (host.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

#define HOST_RX       1     // Pin 1
#define HOST_TX       2     // Pin 2
#define HOST_BAUD     115200
#define HOST_START    '$'   // first character of every host packet (both directions), starts a burst in host Tx
#define HOST_PEER     '#'   // first character of each peer packet in host Tx
#define HOST_TX_INTVL 1000  // ms between sends of myself and the cluster
#define HOST_TIMEOUT  10000 // ms without a valid host packet => host absent

// ------------------------------------------------------------------------------------------------------------------------------------------------
// DEBUGGING (debugging.cpp)
// ------------------------------------------------------------------------------------------------------------------------------------------------

// debug() peripherals
#define DBG_SD_CARD   0     // SD Card write and read
#define DBG_GPS       1     // GPS data
#define DBG_IMU       2     // IMU data
#define DBG_HOST      3     // Host UART Tx and Rx
#define DBG_LORA_RX   4     // LoRa receive only
#define DBG_LORA_TX   5     // LoRa transmit only
#define DBG_LORA      6     // LoRa transmit and receive
#define DBG_UI        7     // OLED, RGB LED and Serial Monitor output

// Sampling and intervals (milliseconds)
#define GPS_SAMPLING    1000
#define IMU_SAMPLING    1000
#define SD_READ_INTVL   2000
#define SD_WRITE_INTVL  1000
#define UI_TEST_INTVL   1000  // ms between UI test messages, each with the next RGB colour

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

struct Data {
    
    // VANET
    byte destId = 0;
    byte srcId = 0;
    uint16_t seq = 0;       // sequence number of the beacon (gaps = lost packets)
    uint8_t ttl = 0;        // hops left: TTL_INIT when sent, one less after each forward
    int8_t srcRssi = 0;     // my LoRa.rssi() (dBm) just before I sent (my beacons only, not carried in packets: 0 for peers)

    // Network health
    float pktRssi = 0.0;    // dBm of the last packet received from this node (measured by the receiver)
    float snr = 0.0;
    float freqErr = 0.0;

    // Packet timing (PPS UTC time base, for latency)
    int64_t utc = 0;        // µs since UTC midnight: TX start (my beacon) or RxDone (a received packet)
    uint32_t airUs = 0;     // µs from TX start to TX done (my beacon only)
    uint8_t sync = 0;       // utc quality: 0 = not synced, 1 = PPS locked, 2 = holdover (PPS lost)

    // Node identification
    byte id = 0;

    // Location
    float lat = 0.0;      // deg (+N/-S)
    float lon = 0.0;      // deg (+E/-W)
    float alt = 0.0;     // m

    // GPS fix quality
    byte sats = 0;      // satellites in view
    float hdop = 0.0;     // horizontal dilution of precision

    // Motion
    float speed = 0.0;
    float course = 0.0;

    uint8_t tasks = 0;      // number of pending tasks
    uint8_t status = 0;     // number coded (codedStatus)
    uint8_t message = 0;   // number coded (codedMessages)

    // Vehicle health
    byte battery = 100;     // % (whole number, from the host)

    // Last updated time
    uint32_t date = 0;
    uint32_t time = 0;

    /*
      Converts the node into a single CSV string
    */
    String toStr(void) const;
};

// System errors (both cores, protected by mutex)
struct Errors {
    bool check = false;                 // true once an error is reported
    String msg = "Errors in:\n";        // all error messages
    SemaphoreHandle_t mutex = NULL;     // created at the start of setup
};

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

extern Data myself;

// System operation
extern Errors errors;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

/*
  Sets errors.check and appends text to errors.msg (safe from both cores)
*/
void reportError(const char *text);

#endif // TVV_SYSTEM_H
