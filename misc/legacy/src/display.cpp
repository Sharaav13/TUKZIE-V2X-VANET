//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - OLED DISPLAY
//*************************************************************************************************************************************************

#include "system.h"
#include "display.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

OLED oled;
SystemLights lights;

//*************************************************************************************************************************************************
// FUNCTIONS - OLED
//*************************************************************************************************************************************************

/*
  Setup OLED display
  Returns true if successful
*/
bool OLED::init(void) {

  if (!Wire.begin(OLED_I2C_SDA, OLED_I2C_SCL)) { // Determine if I2C is ready
    reportError("OLED: I2C bus failed to start\n");
    return false;
  }

  if (!dev.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 0x3C is common I2C address
    reportError("OLED: display not found at 0x3C\n");
    return false;
  }

  ok = true;
  dev.setTextColor(SSD1306_WHITE); // White text
  dev.clearDisplay();
  dev.display();
  return true;
}

/*
  Center text on OLED display
*/
void OLED::centerText(const String &text) {
  int16_t x1, y1;
  uint16_t w, h;

  dev.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH - w) / 2;
  int16_t y = (SCREEN_HEIGHT - h) / 2;

  dev.setCursor(x, y);
  dev.print(text);
}

/*
  Display the formatted text onto the OLED display or Serial if OLED fails
  Also used for OLED display module testing
*/
void OLED::disp(const String &text, uint8_t size, bool center, bool append) {

  if (!ok) { // Check if OLED is working
    Serial.print(text);
    return;
  }

  // Appended to previous text or not and start afresh if text ran off screen
  if(!append || dev.getCursorY() >= dev.height()) {
    dev.clearDisplay();     // Clear display buffer
    dev.setCursor(0, 0);    // Cursor to origin
  }

  dev.setTextSize(size);  // 1 = default size

  if (center) {
    centerText(text); // Centre the text
  } else {
    dev.print(text);
  }

  dev.display(); // Push to OLED display
}

/*
  Refreshes OLED display
*/
void OLED::refresh(void) {
  dev.clearDisplay();
  dev.display();
}

//*************************************************************************************************************************************************
// FUNCTIONS - SYSTEM LIGHTS
//*************************************************************************************************************************************************

/*
  Setup RGB LED
  Returns true (the NeoPixel library does not report failures)
*/
bool SystemLights::init(void) {
  dev.begin();
  dev.setBrightness(BRIGHTNESS);            // (0-255)
  dev.setPixelColor(0, dev.Color(0, 0, 0)); // Initialize all pixels to 'off'
  dev.show();                               // Initialize all pixels to 'off'
  return true;
}

/*
  Sets the RGB LED colour
  colour: RED, ORANGE, YELLOW, GREEN, BLUE or PURPLE (anything else turns it off)
*/
void SystemLights::toggle(uint8_t colour) {
  switch (colour) {
    default:
      dev.setPixelColor(0, dev.Color(0, 0, 0)); // Off
      break;

    case RED:
      dev.setPixelColor(0, dev.Color(255, 0, 0));
      break;

    case ORANGE:
      dev.setPixelColor(0, dev.Color(255, 128, 0));
      break;

    case YELLOW:
      dev.setPixelColor(0, dev.Color(255, 255, 0));
      break;

    case GREEN:
      dev.setPixelColor(0, dev.Color(0, 255, 0));
      break;

    case BLUE:
      dev.setPixelColor(0, dev.Color(0, 0, 255));
      break;

    case PURPLE:
      dev.setPixelColor(0, dev.Color(128, 0, 128));
      break;
  }

  dev.show(); // Push the colour to the LED
}
