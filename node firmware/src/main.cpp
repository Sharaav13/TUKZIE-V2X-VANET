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
/*
  DESCRIPTION:
  1. Each TVV node has an uniquie ID, stored in teh configuration data on the inserted SD card.
  2. The node retrieves and interprets GPS and IMU information itself. 
  3. Other information is exchanged with it via the onboard Multi Network System (MNS) and Heads Up Display (HUD) in Tukzie.
  4. Nodes communicate using LoRa and implement a MAC and Routing protocols.
  5. The TVV mobile app connects via Bluetooth to a node to display messages and network topology of all nodes.
  6. MNS port can provides serial meassages for debugging.
*/
//*************************************************************************************************************************************************
// LIBRARIES
//*************************************************************************************************************************************************

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>  // RGB (debugging)
#include <SD.h>                 // SD Card 
#include <Adafruit_SSD1306.h>   // Display
#include <Adafruit_GFX.h>       // Display
#include <TinyGPS++.h>          // GPS
#include "LSM6.h"               // IMU
#include <LoRa.h>               // LoRa 

//*************************************************************************************************************************************************
// MACROS
//*************************************************************************************************************************************************

// RGB (debugging)
#define RGB_LED_PIN   48 
#define NUM_RGB_LEDS  1

// SD Card
#define SD_CS     10
#define SD_MOSI   11
#define SD_SCK    12
#define SD_MISO   13
#define CFG_FIELDS  3
// #define CFG_PATH  "/tvv_config.txt"
// #define LOG_PATH  "/tvv_log.txt"

// MNS
#define MNS_BAUD 115200

// HUD
#define HUD_BAUD 115200 
#define HUD_RX 17
#define HUD_TX 18

// Display
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_I2C_SDA  8 
#define OLED_I2C_SCL  9 
#define OLED_RESET    -1 // No reset pin

// GPS
#define GPS_BAUD 9600 
#define GPS_RX 1
#define GPS_TX 2
// #define PPS 21
#define NORTH "N"
#define SOUTH "S"
#define EAST  "E"
#define WEST  "W"

// IMU
#define IMU_I2C_SDA   6 
#define IMU_I2C_SCL   7 
#define ACC_MG_PER_LSB    0.061f 
#define GYRO_MDPS_PER_LSB 4.375f  

// LoRa
#define LORA_SCK   39
#define LORA_MISO  40
#define LORA_MOSI  41
#define LORA_NSS   42
#define LORA_RST   38 
#define LORA_DIO0  21
#define LORA_FREQ       433.6E6   // 433 MHz
#define LORA_BW         125E3     // Hz
#define LORA_SF         7         // 7 = low latency, 12 = max range
#define LORA_CR         5         // denominator of 4/x
// #define LORA_SYNC       0x12      // private network
#define LORA_TXPOWER    10        // dBm via PA_BOOST
// #define LORA_PREAMBLE   8


// Sampling and Intervals (milliseconds)
#define GPS_SAMPLING    1000 
#define IMU_SAMPLING    1000 
#define SD_READ_INTVL   2000
#define SD_WRITE_INTVL  1000   
#define BEACON_INTVL    5000
#define MNS_TX_INTVL    1000 
#define HUD_TX_INTVL    1000 
#define MNS_RX_INTVL    1000 
#define HUD_RX_INTVL    1000       

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

Adafruit_NeoPixel rgb = Adafruit_NeoPixel(NUM_RGB_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800); // (debugging)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

HardwareSerial MNSSerial(0);   // UART0
HardwareSerial GPSSerial(1);   // UART1
HardwareSerial HUDSerial(2);   // UART2

SPIClass spiSD(FSPI);
SPIClass spiLoRa(HSPI);

TinyGPSPlus gps;  
LSM6 imu;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Node congifuration
int config[CFG_FIELDS];

// System operation
bool error = false;
String setupMessage;
String errorMessage = "Errors in:\n";

// Display
bool oledOK = false;

// GPS
int datetime[6];      // {day, month, year, hour, min, sec} 
String direction[2];  // {N/S, E/W}
float location[3];    // {lat, lon, alt}
float motion[2];      // {kmph, course}
float fix[2];         // {sats, hdop}

// IMU
float acc[3];   // {x, y, z}
float gyro[3];  // {x, y, z}

// LoRa
String message;
volatile int rssi;
volatile float snr;
volatile long freqErr;
// volatile bool cadBusy = false;
// volatile bool cadPending = false;
// uint32_t seq = 0;

// Previous data transaction times
uint32_t lastSdWrite = 0; // Timestamp of previous SD Card write
uint32_t lastSdRead = 0;  // Timestamp of previous SD Card read
uint32_t lastGps = 0;     // Timestamp of previous GPS sample
uint32_t lastImu = 0;     // Timestamp of previous IMU sample
uint32_t lastHudTx = 0;   
uint32_t lastMnsTx = 0;   
uint32_t lastHudRx = 0;   
uint32_t lastMnsRx = 0;   
uint32_t lastBeacon = 0;
uint32_t lastDbg = 0;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SETTING UP 
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Commence the initialisation of teh node and its peripheral parts.

// Setup OLED display
String oledInit(void) {
  
  bool status1 = Wire.begin(OLED_I2C_SDA, OLED_I2C_SCL); // Determine if I2C is ready
  
  if (status1 && (display.begin(SSD1306_SWITCHCAPVCC, 0x3C))) { // 0x3C is common I2C address
    oledOK = true;
    display.setTextColor(SSD1306_WHITE); // White text
    display.clearDisplay();
    display.display();
    return "OLED init success.\n";
  } else {
    error = true;
    return "OLED init failed.\n";
  }
}

// Setup GPS
String gpsInit(void) { 
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  return "GPS init success.\n";
}

// Setup SD Card module
String sdInit(void) {
  
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  
  if (SD.begin(SD_CS, spiSD)) {
    return "SD init success.\n";
  } else {
    error = true;
    return "SD init failed.\n";
  }
}

// Setup IMU
String imuInit(void) {
  
  bool status2 = Wire1.begin(IMU_I2C_SDA, IMU_I2C_SCL);
  imu.setBus(&Wire1); 
  
  if (status2 && imu.init()) {
    imu.enableDefault();
    return "IMU init success.\n";
  } else {
    error = true;
    return "IMU init failed.\n";
  }
}

// Setup HUD communication
String hudSerialInit(void) {
  HUDSerial.begin(HUD_BAUD, SERIAL_8N1, HUD_RX, HUD_TX);
  return "HUD UART init success.\n";
}

// Setup MNS communication and serial debugging messages
String mnsSerialInit(void) {
  MNSSerial.begin(MNS_BAUD);
  return "MNS UART init success.\n";
} 

// Setup LoRa
String loraInit(void) {
  
  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setSPI(spiLoRa);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(1E6);        // 1E6 for breadboarding otherwise 8E6
  
  if (LoRa.begin(LORA_FREQ)) {

    LoRa.setSpreadingFactor(LORA_SF); // 12
    LoRa.setSignalBandwidth(LORA_BW); //125E3
    LoRa.setCodingRate4(LORA_CR);     // 8
    // LoRa.setPreambleLength(LORA_PREAMBLE);
    // LoRa.setSyncWord(LORA_SYNC);
    LoRa.setTxPower(LORA_TXPOWER, PA_OUTPUT_PA_BOOST_PIN);    // PA_BOOST output on the RA-02
    // LoRa.enableCrc();
    // LoRa.onCadDone(onCadDoneISR); 
    LoRa.receive();
    
    randomSeed(esp_random());
    lastBeacon = millis() - random(0, BEACON_INTVL);
    
    return "LoRa init success.\n";
  } else {
    error = true;
    return "LoRa init failed.\n";
  }
}

// Read configuration data from SD Card
String configNode(void) {

  File f = SD.open("/tvv_config.txt");
  if (!f) {
    return "Config open failed.\n";
  }

  String line = f.readStringUntil('\n');
  f.close();
  line.trim();                      // drops spaces and any trailing '\r'

  for (int i = 0; i < CFG_FIELDS; i++) {
    int comma = line.indexOf(',');

    if (comma < 0) {                // last field, no comma after it
      if (i != CFG_FIELDS - 1) {
        return "Config parse failed.\n";
      }
      config[i] = line.toInt();
    } else {
      config[i] = line.substring(0, comma).toInt();
      line = line.substring(comma + 1);
    }
  }

  String configData = "Config success.\nNode ID: "  + (String)config[0] + "\nUse: ";
  String use; 
  switch (config[1]) {
    case 0:
      configData = configData + "Tukzie" + "\nFleet ID: " + (String)config[1];
      break;

    case 1:
      configData = configData + "Fleet" + "\nFleet ID: " + (String)config[1];
      break;

    case 2:
      configData = configData + "Private";
      break;
  }

  return configData;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// INFORMATION RETREIVAL & TRANSFER 
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Consist of get and set functions.

// Used for SD Card module testing and node debugging
// Returns teh content as a String
String sdRead(String readFile) {
  
  File f = SD.open("/" + readFile + ".txt");
  if (f) {
    
    // Read number of lines
    // size_t len = logFile.size();
    // disp((String)len, 1, false, false);
    
    // Read character by character
    String contents;
    while (f.available()) {
      contents += (char)f.read();
    }
    f.close();
    return contents;
  } else {
    return "SD Card read failed.";
  }
}

// Write/Log contents to SD Card
// Used for node debugging and testing
void sdWrite(String content, String logFile) {

  File f = SD.open("/" + logFile + ".txt", FILE_APPEND);
  
  if (f) {
    f.print(content);
    f.close();
    // disp("Logged to SD", 1, false, false); // debugging
  } else {
    errorMessage += "Failed to open log file\n";
  }
}

// Center text on display
void centerText(String text) {
  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH - w) / 2;
  int16_t y = (SCREEN_HEIGHT - h) / 2;

  display.setCursor(x, y);
  display.print(text);
}

// Display the formatted text onto the OLED display or Serial if OLED fails
// Used for OLED display module testing
void disp(String text, int size, bool center, bool append) {
  
  if (!oledOK) { // Check if OLED is working
    MNSSerial.print(text); 
    return;
  }

  // Appended to previous text or not and start afresh if text ran off screen
  if(!append || display.getCursorY() >= display.height()) { 
    display.clearDisplay();     // Clear display buffer
    display.setCursor(0, 0);    // Cursor to origin
  }
    
  display.setTextSize(size);  // 1 = default size

  if (center) {
    centerText(text); // Centre the text
  } else {
    display.print(text);
  }
  
  display.display(); // Push to OLED display
}

// Refreshes OLED display
void refreshDisp(void) {
  display.clearDisplay();
  display.display();
}

// Retreives GPS data
// Used for GPS module testing
void gpsGet(void) {
  
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

// Retreives IMU data
// Used for IMU module testing
void imuGet(void) {
  
  imu.read();

  // Acceleration
  float acc_x = imu.a.x * ACC_MG_PER_LSB / 1000.0; // imu.readFloatAccelX(); 
  float acc_y = imu.a.y * ACC_MG_PER_LSB / 1000.0; // imu.readFloatAccelY(); 
  float acc_z = imu.a.z * ACC_MG_PER_LSB / 1000.0; // imu.readFloatAccelZ();
  acc[0] = acc_x;
  acc[1] = acc_y;
  acc[2] = acc_z;

  // Gyroscope
  float gyro_x = imu.g.x * GYRO_MDPS_PER_LSB / 1000.0; // imu.readFloatGyroX();
  float gyro_y = imu.g.y * GYRO_MDPS_PER_LSB / 1000.0; // imu.readFloatGyroY();
  float gyro_z = imu.g.z * GYRO_MDPS_PER_LSB / 1000.0; // imu.readFloatGyroZ();
  gyro[0] = gyro_x;
  gyro[1] = gyro_y;
  gyro[2] = gyro_z;
}

// Updates date and time data
void datetimeGet(void) {
  
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
}

// Updates location data
void locationGet(void) {
  
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  // Location
  if (gps.location.isUpdated()) {
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
}

// Updates motion data
void motionGet(void) {
  
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
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

  imuGet();
}

// Updates satellite fix data
void gpsFixGet(void) {
  
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  // Fix quality
  if (gps.satellites.isValid() && gps.hdop.isValid()) {
    int sats_temp = gps.satellites.value();
    float hdop_temp = gps.hdop.hdop();
    fix[0] = sats_temp;
    fix[1] = hdop_temp;
  }
}

// Procedure for receiving LoRa packet
String loraRx() { 
  
  String msg = "";
  
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    
    while (LoRa.available()) {
      msg += String((char)LoRa.read());
    }
    rssi = LoRa.packetRssi();
    snr =  LoRa.packetSnr();
  }

  LoRa.receive();  // re-arm
  return msg;
}

// Procedure for transmitting LoRa packet
void loraTx(String payload) {

  if (!LoRa.beginPacket()) {
    return; // radio busy
  } else {
    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket(true); // true = async, completion via DIO0
    delay(200);
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// TO STRINGS
// ------------------------------------------------------------------------------------------------------------------------------------------------

// These functions convert selected data into formatted strings.

// Converts raw collected data into a single string
size_t rawInfoStr(char *out, size_t cap) {
  return snprintf(out, cap, 
        "%02d-%02d-%04d  "
        "%02d:%02d:%02d  "    
        "GPS[lat: %.4f deg %s, lon: %.4f deg %s, alt: %.1f m, spd: %.1f kmph, crs: %.1f deg, sats: %d, hdop: %.1f]  "
        "ACC[%2f, %2f, %2f]  "
        "GYR[%2f, %2f, %2f]  ",
        datetime[0], datetime[1], datetime[2], 
        datetime[3], datetime[4], datetime[5],
        location[0], direction[0],
        location[1], direction[1],
        location[2],
        motion[0], motion[1],
        (int)fix[0], fix[1],
        acc[0], acc[1], acc[2],
        gyro[0], gyro[1], gyro[2]);
}

// Converts currently stored GPS data into a single string
// Used for GPS module testing
size_t gpsStr(char *out, size_t cap) {
    
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

// Converts currently stored IMU data into a single string
// Used for IMU module testing
size_t imuStr(char *out, size_t cap) {
  
  return  snprintf(out, cap, 
        "A: %2f %2f %2f\n"
        "G: %2f %2f %2f",
        acc[0], acc[1], acc[2],
        gyro[0], gyro[1], gyro[2]);
}

// Converts LoRa Rx information into a single string
size_t loraRxStr(char *out, size_t cap, String content) {
  return snprintf(out, cap, 
        "%02d-%02d-%04d,"       // date
        "%02d:%02d:%02d,%lu,"   // time 
        "%d,%.2f,%ld",          // RX diagnostics
        "%s\n",                 // received payload
        datetime[0], datetime[1], datetime[2], 
        datetime[3], datetime[4], datetime[5], millis(), 
        rssi, snr, freqErr,
        message);
}

// Converts LoRa Tx information into a single string
size_t loraTxStr(char *out, size_t cap, String content) {
  return snprintf(out, cap,
        "%02d-%02d-%04d,"       // date
        "%02d:%02d:%02d,%lu,"   // time
        "%s\n",                 // transmitted payload
        datetime[0], datetime[1], datetime[2], 
        datetime[3], datetime[4], datetime[5], millis(),
        content); 
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// DEBUGGING FUNCTIONS 
// ------------------------------------------------------------------------------------------------------------------------------------------------

// These functions test the peripherals and send messages on operation.

// Writes and reads to the SD card
void sdDebug(void) {
  
  String debugFile = "sd_debug";

  if ((millis() - lastSdWrite) >= SD_WRITE_INTVL) {
    String write_content = (String)millis() + "SD debugging\n"; 
    sdWrite(write_content, debugFile);
    lastSdRead = millis();
  }

  if ((millis() - lastSdRead) >= (SD_READ_INTVL)) {
    String read_content = sdRead(debugFile);
    disp(read_content, 1, false, false); 
    lastSdRead = millis();
  }
}

// Displays all GPS data periodically
void gpsDebug(void) {
  if ((millis() - lastGps) >= GPS_SAMPLING) {
    char gps_str[192];
    gpsStr(gps_str, sizeof(gps_str));
    disp(gps_str, 1, false, false);
    lastGps = millis();
  }
}

// Displays all IMU data periodically
void imuDebug(void) {
  if ((millis() - lastImu) >= IMU_SAMPLING) {
    char imu_str[192];
    imuStr(imu_str, sizeof(imu_str));
    disp(imu_str, 1, false, false);
    lastGps = millis();
  }
}

// Debugs if HUD Tx and Rx ports are working 
// Tx and Rx are wired together
void HudSerialDebug() {

  if ((millis() - lastHudTx) >= HUD_TX_INTVL) {
    HUDSerial.println("HUD Tx:" + (String)millis());
    lastHudTx = millis();
  }

  while (HUDSerial.available() > 0) {
    disp(String((char)HUDSerial.read()), 1, false, true);
  }
}

// Shows LoRa Tx and Rx
void loraDebug(void) {

  if ((millis() - lastBeacon) >= BEACON_INTVL) {
    String tx_msg = "Hello there!";
    loraTx(tx_msg);
    char tx_str[192];
    loraTxStr(tx_str, sizeof(tx_str), tx_msg);
    //disp(tx_str, 1, false, true);
  }

  message = loraRx();
  if (message != "") {
    char rx_str[192];
    loraTxStr(rx_str, sizeof(rx_str), message);
    disp(rx_str, 1, false, false);
  }

}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// MODES 
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Dictate how the node should operate

/*
// Node operations in normal mode
void normal(void) {

}

// Node operates in debugging mode
void debugging(void) {

}

// Node operates in testing mode
void testing(void) {

}
*/
//*************************************************************************************************************************************************
// SETUP
//*************************************************************************************************************************************************

void setup() {

  // Initialise interfaces
  String mnsMessage = mnsSerialInit(); // Serial debugging initialised
  String hudMessage = hudSerialInit();

  // Initialise OLED
  String oledMessage = oledInit();
  
  disp("TUKZIE V2X\n", 2, true, false);
  delay(2000);
  refreshDisp();
  disp("Booting...\n", 1, true, false);
  delay(2000);
  refreshDisp();

  // Show existing init messages
  disp(mnsMessage, 1, false, false);
  delay(1000);
  disp(hudMessage, 1, false, true);
  delay(1000);
  disp(oledMessage, 1, false, true);
  delay(1000);

  // Initialise other peripherals
  disp(sdInit(), 1, false, true);
  delay(1000);
  disp(gpsInit(), 1, false, true);
  delay(1000);
  disp(imuInit(), 1, false, true);
  delay(1000);
  disp(loraInit(), 1, false, true);
  delay(3000);
  
  // Start configurating node
  refreshDisp();
  disp(configNode(), 1, false, false);
  delay(3000);

  // Add necessary setup error message
  if (error) {
    errorMessage += "Setting up\n";
  }

  // Indicate setting up finished
  refreshDisp();
  disp("Finished boot.", 1, false, false);
  delay(3000);

}

//*************************************************************************************************************************************************
// MAIN CODE/LOOP
//*************************************************************************************************************************************************

void loop() {
  
  // Modify for debugging operation => wantthe errors to show without halting function
  // Show error in setup or operation
  if (error) {
    disp(errorMessage, 1, false, false);
  }

  //imuDebug();
  //gpsDebug();
  //sdDebug();
  //HudSerialDebug();
  loraDebug();
}