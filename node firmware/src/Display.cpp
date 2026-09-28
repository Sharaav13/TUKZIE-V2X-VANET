//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - OLED DISPLAY
//*************************************************************************************************************************************************

#include "Display.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Adafruit_NeoPixel rgb = Adafruit_NeoPixel(NUM_RGB_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800); // (debugging)

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// OLED Display
bool oledOK = false;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup OLED display
String oledInit(void) {

  bool status1 = Wire.begin(OLED_I2C_SDA, OLED_I2C_SCL); // Determine if I2C is ready

  if (status1 && (display.begin(SSD1306_SWITCHCAPVCC, 0x3C))) { // 0x3C is common I2C address
    oledOK = true;
    display.setTextColor(SSD1306_WHITE); // White text
    display.clearDisplay();
    display.display();
    return "OLED init pass.\n";
  } else {
    error = true;
    return "OLED init failed.\n";
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
    Serial.print(text);
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
