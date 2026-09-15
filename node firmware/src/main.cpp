//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV) 
// NODE FIRMWARE
//*************************************************************************************************************************************************
/*
  Version:  0
  Date:     14/09/2026
  Authors:  Sharaav Dhebideen

*/
//*************************************************************************************************************************************************
// LIBRARIES
//*************************************************************************************************************************************************

// General
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
#define SERIAL0_BAUD_RATE 115200 // Serial 0, Also used for debugging
HardwareSerial MySerial(0);   // UART0

// Tukzie HUD
#define SERIAL2_BAUD_RATE 115200 // Serial 2
#define RX2 17
#define TX2 18
HardwareSerial MySerial(2);   // UART2

// Neo M8M GPS Module
#define SERIAL1_BAUD_RATE 9600 // Serial 1
#define RX1 2
#define TX1 1
// #define PPS 21
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

bool error = false;
String errorMessage = "";
String text = "";

// Debugging Serial2
int counter = 0;
int increm = 1;
uint32_t lastTx = 0;
String   rxBuf;


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
  
  if(!append) { // Checks if new text should be appended to previous text
    display.clearDisplay(); // Clear the display
    display.setTextSize(size); // 1 = default size
  }
    
  if (center) {
    centerText(text); // Centered text
  } else {
    display.print(text);
  }

  display.display(); // Push to OLED display
}


//*************************************************************************************************************************************************
// SETUP
//*************************************************************************************************************************************************

void setup() {
    
  // Initialise OLED display
  Wire.begin(OLED_I2C_SDA, OLED_I2C_SCL); 
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 0x3C is common I2C address
    Serial.println(F("OLED init failed."));
    error = true;
    errorMessage += "OLED init failed.\n";
  } else {
    push2Screen("SETTING UP...", 1, true, false);
    delay(5000);
  }

  // Initialize Serial
  Serial.begin(SERIAL0_BAUD_RATE); // Tukzie MNS Serial / Serial Monitor
  delay(1000); push2Screen("MNS Serial initialized.\n", 1, false, false);
  Serial2.begin(SERIAL2_BAUD_RATE, SERIAL_8N1, RX2, TX2); // Tukzie HUD Serial 
  delay(1000); push2Screen("HUD Serial initialized.\n", 1, false, true);
  GPSSerial.begin(SERIAL1_BAUD_RATE, SERIAL_8N1, RX1, TX1); // Neo M8M GPS Module
  delay(1000); push2Screen("GPS Serial initialized.\n", 1, false, true);

  delay(1000);
  display.clearDisplay();
}

//*************************************************************************************************************************************************
// MAIN CODE/LOOP
//*************************************************************************************************************************************************

void loop() {
  
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }
 
  if (gps.location.isUpdated()) {
  
    float lat = ceil(gps.location.lat() * 1000000.0) / 1000000.0; 
    float lon = ceil(gps.location.lng() * 1000000.0) / 1000000.0; 

    String gpsString = "lat:" + (String)lat + "\n" + "lon:" + (String)lon;

    push2Screen(gpsString, 1, false, false);
  }

  /*
  if (millis() - lastTx >= 1000) {
    lastTx = millis();
    counter += increm;
    Serial2.println(counter);
  }

  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    if (c != '\n') {
      rxBuf += c;
    }
  }
  text = (String) rxBuf;
  display.display();
  */
}