//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - OLED DISPLAY
//*************************************************************************************************************************************************

#include "system.h"
#include "display.h"
#include "vanet.h"
#include "host.h"
#include "interpret.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

OLED oled;
SystemLights lights;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// ------------------------------------------------------------------------------------------------------------------------------------------------
// OLED
// ------------------------------------------------------------------------------------------------------------------------------------------------

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
  Shows text on the OLED (Serial if the OLED failed)
*/
void OLED::disp(const String &text, uint8_t size, bool center, bool append) {

  if (!ok) { // Check if OLED is working
    Serial.print(text);
    return;
  }

  // New screen unless appending, or if the text ran off the bottom
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

/*
  Signal bars for an average SNR (dB), from "||" (poor) to "|| || || || ||" (good)
*/
static String signalBars(float snr) {

  uint8_t bars = 1;
  if (snr >= SNR_5_BARS) {
    bars = 5;
  } else if (snr >= SNR_4_BARS) {
    bars = 4;
  } else if (snr >= SNR_3_BARS) {
    bars = 3;
  } else if (snr >= SNR_2_BARS) {
    bars = 2;
  }

  String text = "||";
  for (uint8_t i = 1; i < bars; i++) {
    text += " ||";
  }
  return text;
}

/*
  Node view: signal bars, my speed, battery, status and location, my peers' IDs and the latest peer message
  me: a copy of myself
*/
void OLED::showNode(const Data &me) {

  // Signal environment: average SNR of my peers as bars (none without peers)
  String text = "Net: ";
  text += (vanet.cluster.count == 0) ? String("No signal") : signalBars(vanet.cluster.averageSnr());
  text += "\n";

  // Speed, battery and status (21 characters per line), e.g. "45kmph 85% Delivering"
  text += String(me.speed, 0) + "kmph ";
  text += host.present() ? String(me.battery) + "% " : String("--% ");   // battery is unknown without a host
  text += (me.status < STATUS_COUNT) ? codedStatus[me.status] : "Unknown";   // codes outside codedStatus show as Unknown
  text += "\n";
  text += "Lat: " + String(me.lat, 7) + "\n";
  text += "Lon: " + String(me.lon, 7) + "\n";

  // Cluster peers
  text += "Peers:";
  if (vanet.cluster.count == 0) {
    text += " none";
  }
  for (uint8_t i = 0; i < vanet.cluster.count; i++) {
    text += " " + String(vanet.cluster.peerTable[i].id);
  }
  text += "\n";

  // Latest peer message (unknown codes ignored)
  if ((vanet.peerMsg != 0) && (vanet.peerMsg < MSG_COUNT)) {
    text += "Msg from " + String(vanet.peerMsgId) + ":\n" + codedMessages[vanet.peerMsg];
  } else {
    text += "No messages";
  }

  disp(text, 1, false, false);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SystemLights
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Setup RGB LED
  Returns true (the NeoPixel library does not report failures)
*/
bool SystemLights::init(void) {
  dev.begin();
  dev.setBrightness(BRIGHTNESS);            // (0-255)
  dev.setPixelColor(0, dev.Color(0, 0, 0)); // off
  dev.show();
  return true;
}

/*
  Sets the RGB LED colour
  colour: RED, ORANGE, YELLOW, GREEN, BLUE or PURPLE (anything else turns it off)
*/
void SystemLights::toggle(uint8_t colour) {
  this->colour = colour;

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

/*
  Shows a colour for FLASH_MS without blocking (update() turns it off)
*/
void SystemLights::flash(uint8_t colour) {
  toggle(colour);
  flashing = true;
  flashStart = millis();
}

/*
  Call every loop: ends a flash after FLASH_MS
*/
void SystemLights::update(void) {
  if (flashing && (millis() - flashStart >= FLASH_MS)) {
    flashing = false;
    toggle(OFF);
  }
}
