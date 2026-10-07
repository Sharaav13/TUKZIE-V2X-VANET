//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - OLED DISPLAY
//*************************************************************************************************************************************************

#ifndef TVV_DISPLAY_H
#define TVV_DISPLAY_H

#include "system.h"

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

// OLED display
struct OLED {

  Adafruit_SSD1306 dev{SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET};

  // Status
  bool ok = false;

  /*
    Setup OLED display
    Returns true if successful
  */
  bool init(void);

  /*
    Center text on display
  */
  void centerText(const String &text);

  /*
    Display the formatted text onto the OLED display or Serial if OLED fails
    Used for OLED display module testing
  */
  void disp(const String &text, uint8_t size, bool center, bool append);

  /*
    Refreshes OLED display
  */
  void refresh(void);
};

// RGB LED
struct SystemLights {

  Adafruit_NeoPixel dev{NUM_RGB_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800};

  /*
    Setup RGB LED
    Returns true (the NeoPixel library does not report failures)
  */
  bool init(void);

  /*
    Sets the RGB LED colour
    colour: RED, ORANGE, YELLOW, GREEN, BLUE or PURPLE (anything else turns it off)
  */
  void toggle(uint8_t colour);
};

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern OLED oled;
extern SystemLights lights;

#endif // TVV_DISPLAY_H
