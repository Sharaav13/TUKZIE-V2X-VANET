//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - UI (OLED DISPLAY & RGB LED)
//*************************************************************************************************************************************************

#ifndef TVV_UI_H
#define TVV_UI_H

#include "system.h"

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

// OLED display
struct OLED {

  Adafruit_SSD1306 dev{SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET};

  // Status
  bool ok = false;

  // Timestamps (ms)
  uint32_t lastShow = 0;        // previous node view update

  /*
    Setup OLED display
    Returns true if successful
  */
  bool init(void);

  /*
    Center text on OLED display
  */
  void centerText(const String &text);

  /*
    Shows text on the OLED (Serial if the OLED failed)
  */
  void disp(const String &text, uint8_t size, bool center, bool append);

  /*
    Refreshes OLED display
  */
  void refresh(void);

  /*
    Node view: signal bars, my speed, battery, status and location, my peers' IDs and the latest peer message
    me: a copy of myself
  */
  void showNode(const Data &me);
};

// RGB LED
struct SystemLights {

  Adafruit_NeoPixel dev{NUM_RGB_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800};

  // Status
  uint8_t colour = OFF;         // colour currently shown
  uint8_t background = OFF;     // colour shown between flashes (NO_FIX_COLOUR without a GPS fix, else OFF)
  bool flashing = false;        // a flash is waiting to be turned off

  // Timestamps (ms)
  uint32_t flashStart = 0;      // when the current flash started

  /*
    Setup RGB LED
    Returns true (the NeoPixel library does not report failures)
  */
  bool init(void);

  /*
    Sets the RGB LED colour
    colour: RED, ORANGE, YELLOW, GREEN, BLUE, PURPLE or BRIGHTWHITE (anything else turns it off)
  */
  void toggle(uint8_t colour);

  /*
    Shows a colour for FLASH_MS without blocking (update() then returns to the background colour)
  */
  void flash(uint8_t colour);

  /*
    Sets the colour shown between flashes, straight away unless a flash is on
  */
  void setBackground(uint8_t colour);

  /*
    Call every loop: ends a flash after FLASH_MS, back to the background colour
  */
  void update(void);
};

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern OLED oled;
extern SystemLights lights;

#endif // TVV_UI_H
