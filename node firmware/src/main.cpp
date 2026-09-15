//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV) 
// NODE FIRMWARE
//*************************************************************************************************************************************************
/*
  Version:  0
  Date:     18/09/2026
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

// Neo M8M GPS Module
#include <TinyGPS++.h>

// OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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
#define EAST "E"
#define WEST "W"
HardwareSerial GPSSerial(1); // UART1
TinyGPSPlus gps;

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_I2C_SDA 8 
#define OLED_I2C_SCL 9 
#define OLED_RESET -1 // No reset pin
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

/*
// Debugging HUDSerial
int counter = 0;
int increm = 1;
uint32_t lastTx = 0;
String   rxBuf;
*/

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
size_t gpsToString(char *out, size_t cap)
{
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
//*************************************************************************************************************************************************
// SETUP
//*************************************************************************************************************************************************

void setup() {
    
  // Initialise OLED display
  Wire.begin(OLED_I2C_SDA, OLED_I2C_SCL); 
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 0x3C is common I2C address
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
  
  display.clearDisplay();
  push2Screen("FINISHED SETUP", 1, true, false);
    delay(5000);
}

//*************************************************************************************************************************************************
// MAIN CODE/LOOP
//*************************************************************************************************************************************************

void loop() {
  
  if (error) {
    push2Screen(errorMessage, 2, true, false); // Show errors
    MNSSerial.println(errorMessage); // Debugging
    while(true){}; // Must be a more elegant way to do this
  }

  getGpsData();
  char gps_str[192];
  gpsToString(gps_str, sizeof(gps_str));
  push2Screen(gps_str, 1, false, false); 
  // MNSSerial.println(gps_str); // Debugging
  
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