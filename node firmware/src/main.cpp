//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV) 
// NODE FIRMWARE
//*************************************************************************************************************************************************
/*
  Version:  0
  Date:     22/09/2026
  Authors:  Sharaav Dhebideen

*/
//*************************************************************************************************************************************************
// LIBRARIES
//*************************************************************************************************************************************************

// General
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

// Neo M8M GPS Module
#include <TinyGPS++.h>

// LSM6DS3 IMU Module
#include "LSM6.h"
// #include "SparkFunLSM6DS3.h"

// OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// SD Card Module
#include <SD.h>

// RA-02 LoRa Module
#include <LoRa.h>

//*************************************************************************************************************************************************
// MACROS & INSTANCES
//*************************************************************************************************************************************************

// Tukzie MNS
#define MNS_SERIAL_BAUD_RATE 115200 // Serial 0, Also used for debugging
HardwareSerial MNSSerial(0);   // UART0

// Tukzie HUD
#define HUD_Serial_BAUD_RATE 115200 // Serial 2
#define HUD_RX 17
#define HUD_TX 18
HardwareSerial HUDSerial(2);   // UART2

// Neo M8M GPS Module
#define GPS_SERIAL_BAUD_RATE 9600 // Serial 1
#define GPS_RX 1
#define GPS_TX 2
// #define PPS 21
#define NORTH "N"
#define SOUTH "S"
#define EAST  "E"
#define WEST  "W"
// #define GPS_SAMPLING <> // Hz
HardwareSerial GPSSerial(1); // UART1
TinyGPSPlus gps;

// LSD6MDS3 IMU Module
#define IMU_I2C_SDA   6 
#define IMU_I2C_SCL   7 
#define IMU_SAMPLING  1 // Hz
#define ACC_MG_PER_LSB    0.061f 
#define GYRO_MDPS_PER_LSB 4.375f  
LSM6 imu;
// LSM6DS3 imu( I2C_MODE, 0x6B );
// TwoWire Wire1 = TwoWire(1); // Second instance of I2C

// OLED Display
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_I2C_SDA  8 
#define OLED_I2C_SCL  9 
#define OLED_RESET    -1 // No reset pin
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// SD Card Module
#define SD_CS     10
#define SD_MOSI   11
#define SD_SCK    12
#define SD_MISO   13
#define SD_SAMPLING 1  // Hz
SPIClass spiSD(FSPI); 

// RGB LED
#define RGB_LED_PIN 48 // Test lora with RGB
#define NUM_RGB_LEDS 1
Adafruit_NeoPixel rgb_led = Adafruit_NeoPixel(NUM_RGB_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// RA-02 LoRa Module
#define LORA_SCK   39
#define LORA_MISO  40
#define LORA_MOSI  41
#define LORA_NSS   42
#define LORA_RST   38 
#define LORA_DIO0  21

#define LORA_FREQ       433E6   // 433 MHz
#define LORA_BW         125E3   // Hz
#define LORA_SF         9       // 7 = low latency, 12 = max range
#define LORA_CR         5       // denominator of 4/x
#define LORA_SYNC       0x12    // private network
#define LORA_TXPOWER    17      // dBm via PA_BOOST. Keep <= 17.
#define LORA_PREAMBLE   8

#define NODE_ID         1
#define BEACON_MS       2000
SPIClass spiLoRa(HSPI); 

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// General
bool error = false;
String errorMessage = "";

// GPS data
int datetime[] = {0, 0, 0, 0, 0, 0}; // {day, month, year, hour, min, sec} 
String direction[] = {"", ""}; // {N/S, E/W}
float location[] = {0, 0, 0}; // {lat, lon, alt}
float motion[] = {0, 0}; // {kmph, course}
float fix[] = {0, 0}; // {sats, hdop}
// uint32_t lastGPS = 0; // Timestamp of previous GPS sample

// IMU data
float acc[] = {0, 0, 0}; // {x, y, z}
float gyro[] = {0, 0, 0}; // {x, y, z}
uint32_t lastImu = 0; // Timestamp of previous IMU sample

// SD Card Log
uint32_t lastLog = 0; // Timestamp of previous log
uint32_t lastRead = 0; // Timestamp of previous read
/*
// Debugging HUDSerial
int counter = 0;
int increm = 1;
uint32_t lastTx = 0;
String   rxBuf;
*/

// LoRa Module
volatile bool rxFlag = false;
volatile int rxLen = 0;
volatile bool txDone = true;
volatile bool cadBusy = false;
volatile bool cadPending = false;
uint32_t lastBeacon = 0;
uint32_t seq = 0;


//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Helper function to center text on OLED
void centerText(String text) {
  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH - w) / 2;
  int16_t y = (SCREEN_HEIGHT - h) / 2;

  display.setCursor(x, y);
  display.print(text);
}

// Simpler OLED display() function
void push2Screen(String text, int size, bool center, bool append) {
  
  if(!append) { // Appended to previous text or not
    display.clearDisplay(); // Clear the display
    display.setTextSize(size); // 1 = default size
    display.setCursor(0, 0); // Cursor to origin
  }
    
  if (center) {
    centerText(text); // Centered text
  } else {
    display.print(text);
  }

  display.display(); // Push to OLED display
}

// Retrieve GPS data
void getGpsData(void) {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  // Date and time
  if (gps.time.isValid() && gps.date.isValid()) {
    int year_temp = gps.date.year();
    int month_temp = gps.date.month();
    int day_temp = gps.date.day();
    int hour_temp = gps.time.hour();
    int minute_temp = gps.time.minute();
    int sec_temp = gps.time.second();
       
    datetime[2] = year_temp;
    datetime[1] = month_temp;
    datetime[0] = day_temp;
    
    datetime[4] = minute_temp;
    datetime[5] = sec_temp;
    int hour_corrected = hour_temp + 2; // SAST = UTC + 2hr
    datetime[3] = (hour_corrected < 24) ? hour_corrected : (hour_corrected - 24); 
  }

  // Location
  if (gps.location.isUpdated()) {
    //float lat = ceil(gps.location.lat() * 1000000.0) / 1000000.0; 
    //float lon = ceil(gps.location.lng() * 1000000.0) / 1000000.0;
    float lat_temp = gps.location.lat(); 
    float lon_temp = gps.location.lng(); 
    location[0] = abs(lat_temp);
    location[1] = abs(lon_temp);
    direction[0] = (lat_temp > 0) ? NORTH : SOUTH;
    direction[1] = (lon_temp > 0) ? EAST : WEST;
  }

  // Altitude
  if (gps.altitude.isValid()) {
    float alt_temp = gps.altitude.meters();
    location[2] = alt_temp;
  }

  // Speed
  if (gps.speed.isUpdated()) {
    float speed_gps_temp = gps.speed.kmph();
    motion[0] = speed_gps_temp;
  }

  // Course
  if (gps.course.isValid()) {
    float course_temp = gps.course.deg();
    motion[1] = course_temp;
  }

  // Fix quality
  if (gps.satellites.isValid() && gps.hdop.isValid()) {
    int sats_temp = gps.satellites.value();
    float hdop_temp = gps.hdop.hdop();
    fix[0] = sats_temp;
    fix[1] = hdop_temp;
  }
}

// All current GPS data into a single string
size_t gpsToString(char *out, size_t cap) {
    return snprintf(out, cap,
        "%02d-%02d-%04d  %02d:%02d:%02d\n"
        "lat: %.4f deg %s\n"
        "lon: %.4f deg %s\n"
        "alt: %.1f m\n"
        "speed: %.1f kmph\n"
        "course: %.1f deg\n"
        "sats: %d   hdop: %.1f",
        datetime[0], datetime[1], datetime[2], datetime[3], datetime[4], datetime[5],
        location[0], direction[0],
        location[1], direction[1],
        location[2],
        motion[0], motion[1],
        (int)fix[0], fix[1]);
}

// Retrieve GPS data
void getImuData(void) {
  imu.read();

  float acc_x = imu.a.x * ACC_MG_PER_LSB / 1000.0; // imu.readFloatAccelX(); 
  float acc_y = imu.a.y * ACC_MG_PER_LSB / 1000.0; // imu.readFloatAccelY(); 
  float acc_z = imu.a.z * ACC_MG_PER_LSB / 1000.0; // imu.readFloatAccelZ();
  acc[0] = acc_x;
  acc[1] = acc_y;
  acc[2] = acc_z;

  float gyro_x = imu.g.x * GYRO_MDPS_PER_LSB / 1000.0; // imu.readFloatGyroX();
  float gyro_y = imu.g.y * GYRO_MDPS_PER_LSB / 1000.0; // imu.readFloatGyroY();
  float gyro_z = imu.g.z * GYRO_MDPS_PER_LSB / 1000.0; // imu.readFloatGyroZ();
  gyro[0] = gyro_x;
  gyro[1] = gyro_y;
  gyro[2] = gyro_z;
}

// All current GPS data into a single string
size_t imuToString(char *out, size_t cap) {
  return snprintf(out, cap, 
        "A: %2f %2f %2f\n"
        "G: %2f %2f %2f",
        acc[0], acc[1], acc[2],
        gyro[0], gyro[1], gyro[2]);
}

// All recently ollected data into a single string
size_t dataToString(char *out, size_t cap) {
  return snprintf(out, cap, 
        "%02d-%02d-%04d  "
        "%02d:%02d:%02d  "    
        "GPS[lat: %.4f deg %s, lon: %.4f deg %s, alt: %.1f m, spd: %.1f kmph, crs: %.1f deg, sats: %d, hdop: %.1f]  "
        "ACC[%2f, %2f, %2f]  "
        "GYR[%2f, %2f, %2f]  ",
        datetime[0], datetime[1], datetime[2], datetime[3], datetime[4], datetime[5],
        location[0], direction[0],
        location[1], direction[1],
        location[2],
        motion[0], motion[1],
        (int)fix[0], fix[1],
        acc[0], acc[1], acc[2],
        gyro[0], gyro[1], gyro[2]);
}

// Procedure for Rx packet
void onReceiveISR(int packetSize) { 
  rxLen = packetSize; 
  rxFlag = true; 
}

// Flag packet Tx is done
void onTxDoneISR() { 
  txDone = true; 
}

void sendBeacon() {
  if (!LoRa.beginPacket()) return;   // radio busy
  LoRa.print("Hello there!");
  txDone = false;
  LoRa.endPacket(true);              // true = async, completion via DIO0
  // Serial.printf("%lu,TX,%d,%lu,,,\n", millis(), NODE_ID, seq++);
}

// LoRa Rx data into string
size_t loraToString(char *out, size_t cap, String content) {
  return snprintf(out, cap, 
        "%lu,RX,%s,%d,%.2f,%ld\n",
        millis(), content.c_str(),
        LoRa.packetRssi(), LoRa.packetSnr(),
        LoRa.packetFrequencyError());
  }
//*************************************************************************************************************************************************
// SETUP
//*************************************************************************************************************************************************

void setup() {
    
  // Initialise OLED display
  bool status1 = Wire.begin(OLED_I2C_SDA, OLED_I2C_SCL); 
  if (status1 && (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))) { // 0x3C is common I2C address
    error = true;
    errorMessage += "OLED init failed.\n";
  } else {
    display.setTextColor(SSD1306_WHITE); // White text
    push2Screen("TUKZIE V2X", 2, true, false);
    delay(5000);
    push2Screen("STARTING SETUP", 1, true, false);
    delay(5000);
  }

  // Initialize Serial
  MNSSerial.begin(MNS_SERIAL_BAUD_RATE); // Tukzie MNS Serial / Serial Monitor
  delay(1000); push2Screen("MNS initialized.\n", 1, false, false);
  HUDSerial.begin(HUD_Serial_BAUD_RATE, SERIAL_8N1, HUD_RX, HUD_TX); // Tukzie HUD Serial 
  delay(1000); push2Screen("HUD initialized.\n", 1, false, true);
  GPSSerial.begin(GPS_SERIAL_BAUD_RATE, SERIAL_8N1, GPS_RX, GPS_TX); // Neo M8M GPS Module
  delay(1000); push2Screen("GPS initialized.\n", 1, false, true);
  delay(1000);

  // Initialize IMU
  // Wire1.setPins(6, 7);
  bool status2 = Wire1.begin(IMU_I2C_SDA, IMU_I2C_SCL);
  imu.setBus(&Wire1); 
  if (status2 && !imu.init()) {
  // if (status1 ||(imu.begin() != 0)) {
    error = true;
    errorMessage += "IMU init failed.\n";
  } else {
    imu.enableDefault();
    push2Screen("IMU initialized.\n", 1, false, true);
    delay(1000);
  }

  // Initialize SD Card
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    error = true;
    errorMessage += "SD init failed.\n";
  } else {
    push2Screen("SD initialized.\n", 1, false, true);
    delay(1000);
  }

  // Initialise LoRa
  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setSPI(spiLoRa);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(8E6);        // 1E6 for breadboarding
  
  if (!LoRa.begin(LORA_FREQ)) {
    error = true;
    errorMessage += "LoRa init failed.";
  } else {
    LoRa.setSpreadingFactor(LORA_SF); // 12
    LoRa.setSignalBandwidth(LORA_BW); //125E3
    LoRa.setCodingRate4(LORA_CR); // 8
    // LoRa.setPreambleLength(LORA_PREAMBLE);
    // LoRa.setSyncWord(LORA_SYNC);
    LoRa.setTxPower(LORA_TXPOWER);    // 20 PA_BOOST output on the RA-02
    // LoRa.enableCrc();

    LoRa.onReceive(onReceiveISR);
    LoRa.onTxDone(onTxDoneISR);
    // LoRa.onCadDone(onCadDoneISR); // Double check
    LoRa.receive();
    push2Screen("LoRa initialized.\n", 1, false, true);
    delay(1000);
  }

  // Finish setup
  display.clearDisplay();
  push2Screen("FINISHED SETUP", 1, true, false);
  delay(5000);
  display.clearDisplay();
}

//*************************************************************************************************************************************************
// MAIN CODE/LOOP
//*************************************************************************************************************************************************

void loop() {
  
  if (error) {
    push2Screen(errorMessage, 1, true, false); // Show errors
    MNSSerial.println(errorMessage); // Debugging
    while(true){}; // Must be a more elegant way to do this
  }

  // Show GPS data
  getGpsData();
  // char gps_str[192];
  // gpsToString(gps_str, sizeof(gps_str));
  // push2Screen(gps_str, 1, false, false); 
  // MNSSerial.println(gps_str); // Debugging
  
  // Show IMU data
  if ((millis() - lastImu) >= (1000 / IMU_SAMPLING)) {
    getImuData();
    // char imu_str[80];
    // imuToString(imu_str, sizeof(imu_str));
    // push2Screen(imu_str, 1, false, false);
    lastImu = millis();
    // MNSSerial.println(imu_str); // Debugging
  }

  /*
  // Log to SD Card
  if ((millis() - lastLog) >= (1000 / SD_SAMPLING)) {
    File logFile = SD.open("/tvv_log.txt", FILE_APPEND);
    if (logFile) {
      // char log_str[192];
      // dataToString(log_str, sizeof(log_str));
      String log_str = "1";
      logFile.print(log_str);
      // logFile.println(log_str);
      logFile.close();
      // push2Screen("Logged to SD", 1, false, false);
    } else {
      push2Screen("Failed to open log file", 1, false, false);
    }
    lastLog = millis();
  }

  // Read from SD Card
  if ((millis() - lastRead) >= (2000 / SD_SAMPLING)) {
    File logFile = SD.open("/tvv_log.txt");
    if (logFile) {
      
      // Read number of lines
      // size_t len = logFile.size();
      // push2Screen((String)len, 1, false, false);
      
      // Read character by character
      String contents;
      while (logFile.available()) {
        contents += (char)logFile.read();
      }
      logFile.close();
      push2Screen(contents, 1, false, false);
    } else {
      push2Screen("Failed to open log file", 1, false, false);
    }
    lastRead = millis();
  }
  */

  // Check if recieved LoRa packet
  if (rxFlag) {
    rxFlag = false;
    String payload = "";
    while (LoRa.available()) {
      payload += (char)LoRa.read();
    }
    char rx_str[192];
    loraToString(rx_str, sizeof(rx_str), payload);
    push2Screen(rx_str, 1, false, true);
    LoRa.receive();                  // re-arm
  }

  // LoRa Tx
  if (txDone &&  (millis() - lastBeacon > BEACON_MS)) {
    lastBeacon = millis();
    sendBeacon();
  }

  /*
  if (millis() - lastTx >= 1000) {
    HUDSerial.println("Hello world");
  }
  
  if (millis() - lastTx >= 1000) {
    lastTx = millis();
    counter += increm;
    HUDSerial.println(counter);
  }

  while (HUDSerial.available() > 0) {
    char c = (char)HUDSerial.read();
    if (c != '\n') {
      rxBuf += c;
    }
  }
  text = (String) rxBuf;
  display.display();
  */
}