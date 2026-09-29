//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SYSTEM (LIBRARIES, MACROS & SHARED GLOBAL VARIABLES)
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
#include "BluetoothSerial.h"    // TUKZIE dashboard
#include <Adafruit_NeoPixel.h>  // RGB (debugging)

//*************************************************************************************************************************************************
// MACROS
//*************************************************************************************************************************************************

// Serial Debugging
#define SERIAL_BAUD 115200

// SD Card
#define SD_CS     10      // Pin 10
#define SD_MOSI   11      // Pin 11
#define SD_SCK    12      // Pin 12
#define SD_MISO   13      // Pin 13
#define CFG_FIELDS  3
#define CFG_PATH  "/config.txt"
// #define LOG_PATH  "/tvv_log.txt"

// IMU
#define IMU_I2C_SDA   8   // Pin 8
#define IMU_I2C_SCL   9   // Pin 9

// GPS
#define GPS_RX 17         // Pin 17, To TX of GPS
#define GPS_TX 18         // Pin 18, to RX of GPS
#define PPS 16            // Pin 16
#define GPS_BAUD 9600

// OLED Display
#define OLED_I2C_SDA  4  // Pin 4
#define OLED_I2C_SCL  5  // Pin 5
#define OLED_RESET    -1 // No reset pin
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

// MNS
#define MNS_RX 1 // Pin 1
#define MNS_TX 2 // Pin 2
#define MNS_BAUD 115200

// LoRa
#define LORA_SCK   39 // Pin 39
#define LORA_MISO  40 // Pin 40
#define LORA_MOSI  41 // Pin 41
#define LORA_NSS   42 // Pin 42
#define LORA_RST   38 // Pin 38
#define LORA_DIO0  21 // Pin 21
#define LORA_FREQ  433.6E6   // 433 MHz band
#define LORA_BW    125E3     // Hz
#define LORA_SF    7         // 7 = low latency, 12 = max range
#define LORA_CR    5         // denominator of 4/x
// #define LORA_SYNC       0x12      // private network
#define LORA_TXPOWER  10        // dBm via PA_BOOST
// #define LORA_PREAMBLE  8

// Sampling and Intervals (milliseconds)
#define GPS_SAMPLING    1000
#define IMU_SAMPLING    1000
#define SD_READ_INTVL   2000
#define SD_WRITE_INTVL  1000
#define BEACON_INTVL    5000
#define MNS_TX_INTVL    1000
#define MNS_RX_INTVL    1000

// RGB
#define RGB_LED_PIN   48 // Pin 48
#define NUM_RGB_LEDS  1

// Information
#define PROTO_VERSION  1
#define FRAME_CSV_MAX  96

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Node congifuration
extern int config[CFG_FIELDS];

// System operation
extern bool error;
extern String errorMessage;

// Payload variables
extern String message; // Make them volatile for interrupts.
// extern char messageTx[FRAME_CSV_MAX];
// extern char messageRx[FRAME_CSV_MAX];

#endif // TVV_SYSTEM_H
