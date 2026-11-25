//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - UI (OLED DISPLAY & RGB LED)
//*************************************************************************************************************************************************
/*
  Version:  1.0
  Date:     27/10/2026
  Authors:  Sharaav Dhebideen
*/
//*************************************************************************************************************************************************

#include "system.h"
#include "ui.h"
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

  if (!Wire.begin(OLED_I2C_SDA, OLED_I2C_SCL)) {
    reportError("OLED: I2C bus failed to start\n");
    return false;
  }

  // Probe the address first: dev.begin() only fails if its buffer can't be allocated, not if no display answers
  Wire.beginTransmission(0x3C);                    // 0x3C: the usual I2C address of SSD1306 modules
  if (Wire.endTransmission() != 0) {
    reportError("OLED: display not found at 0x3C\n");
    return false;
  }

  if (!dev.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    reportError("OLED: display buffer could not be allocated\n");
    return false;
  }

  ok = true;
  dev.setTextColor(SSD1306_WHITE);
  dev.clearDisplay();
  dev.display();
  return true;
}

/*
  Centres text on the OLED display
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
  Shows text on the OLED (if present) and on Serial
*/
void OLED::disp(const String &text, uint8_t size, bool center, bool append) {

  // Always mirror the text to Serial
  Serial.print(text);
  if (!append && !text.endsWith("\n")) {
    Serial.print("\n");   // keep each new screen on its own lines
  }

  if (!ok) {
    return;               // no display buffer if init failed
  }

  // New screen unless appending, or if the text ran off the bottom
  if(!append || dev.getCursorY() >= dev.height()) {
    dev.clearDisplay();
    dev.setCursor(0, 0);
  }

  dev.setTextSize(size);    // 1 = 21 characters per line

  if (center) {
    centerText(text);
  } else {
    dev.print(text);
  }

  dev.display();            // push the buffer to the OLED
}

/*
  Clears the OLED display
*/
void OLED::refresh(void) {
  if (!ok) {
    return;               // no display buffer if init failed
  }
  dev.clearDisplay();
  dev.display();
}

/*
  Signal bars for an average SNR (dB), from "||" (poor) to "|| || || || ||" (excellent)
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

  // Link quality: average SNR of my peers as bars ("No signal" without peers)
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
  colour: RED, ORANGE, YELLOW, GREEN, BLUE, PURPLE or BRIGHTWHITE (anything else turns it off)
*/
void SystemLights::toggle(uint8_t colour) {
  this->colour = colour;

  switch (colour) {
    default:
      dev.setPixelColor(0, dev.Color(0, 0, 0));   // off
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

    case BRIGHTWHITE:
      dev.setPixelColor(0, dev.Color(255, 255, 255));
      break;
  }

  dev.show();   // push the colour to the LED
}

/*
  Shows a colour for FLASH_MS without blocking (update() then returns to the background colour)
*/
void SystemLights::flash(uint8_t colour) {
  toggle(colour);
  flashing = true;
  flashStart = millis();
}

/*
  Sets the colour shown between flashes, straight away unless a flash is on
*/
void SystemLights::setBackground(uint8_t colour) {
  background = colour;
  if (!flashing && (this->colour != colour)) {
    toggle(colour);
  }
}

/*
  Call every loop: ends a flash after FLASH_MS, back to the background colour
*/
void SystemLights::update(void) {
  if (flashing && (millis() - flashStart >= FLASH_MS)) {
    flashing = false;
    toggle(background);
  }
}
