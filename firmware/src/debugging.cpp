//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - DEBUGGING
//*************************************************************************************************************************************************

#include "system.h"
#include "debugging.h"
#include "ui.h"
#include "storage.h"
#include "sensing.h"
#include "host.h"

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Debug Selection
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Runs the debug routine of the selected peripheral
  peripheral: DBG_UI, DBG_SD_CARD, DBG_GPS, DBG_IMU, DBG_HOST, DBG_LORA_RX, DBG_LORA_TX or DBG_LORA
*/
void debug(uint8_t peripheral) {

  switch (peripheral) {
    case DBG_UI:
      uiDebug();
      break;
    case DBG_SD_CARD:
      storageDebug();
      break;
    case DBG_GPS:
      gpsDebug();
      break;
    case DBG_IMU:
      imuDebug();
      break;
    case DBG_HOST:
      hostDebug();
      break;
    case DBG_LORA_RX:
      loraDebug(0);
      break;
    case DBG_LORA_TX:
      loraDebug(1);
      break;
    case DBG_LORA:
      loraDebug(2);
      break;
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// User Interface
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Every UI_TEST_INTVL: sets the RGB LED to its next colour (cycling through all of them) and shows the same test
  message, with that colour's name, on the OLED and the Serial Monitor
*/
void uiDebug(void) {

  // Name of each RGB colour (index = colour code, OFF to BRIGHTWHITE)
  static const char *const colourNames[] = {"Off", "Red", "Orange", "Yellow", "Green", "Blue", "Purple", "White"};
  static_assert(sizeof(colourNames) / sizeof(colourNames[0]) == BRIGHTWHITE + 1, "colourNames must have an entry for every colour");

  static uint32_t lastTest = 0;     // previous UI test message (kept between calls)
  static uint8_t colour = RED;      // next colour shown (kept between calls)

  if ((millis() - lastTest) >= UI_TEST_INTVL) {
    lights.toggle(colour);
    String msg = "UI test: " + String(millis()) + " ms\nRGB: " + colourNames[colour] + "\n";
    oled.disp(msg, 1, false, false);
    Serial.print(msg);

    colour = (colour >= BRIGHTWHITE) ? RED : (colour + 1);   // RED to BRIGHTWHITE, then round again
    lastTest = millis();
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Storage
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Writes a test line to DEBUG_FILE (shown on the OLED) and reads the file back to Serial
*/
void storageDebug(void) {

  if ((millis() - storage.lastWrite) >= SD_WRITE_INTVL) {
    String written = "SD test: " + String(millis()) + " ms\n";
    storage.write(written, DEBUG_FILE);
    oled.disp(written, 1, false, false);    
    storage.lastWrite = millis();
  }

  if ((millis() - storage.lastRead) >= (SD_READ_INTVL)) {
    String content = storage.read(DEBUG_FILE);
    Serial.println(content + "\n");
    storage.lastRead = millis();
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// GPS
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Displays all GPS data periodically
*/
void gpsDebug(void) {
  if ((millis() - gps.lastSample) >= GPS_SAMPLING) {
    gps.get();
    char gps_str[192];
    gps.toStr(gps_str, sizeof(gps_str));
    Serial.println("GPS test: " + String(millis()) + "\n" + gps_str + "\n");
    gps.lastSample = millis();
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// IMU
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Displays all IMU data periodically
*/
void imuDebug(void) {
  if ((millis() - imu.lastSample) >= IMU_SAMPLING) {
    imu.getAll();
    char imu_str[192];
    imu.toStr(imu_str, sizeof(imu_str));
    Serial.println("IMU test: " + String(millis()) + "\n" + imu_str  + "\n");
    imu.lastSample = millis();
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Host
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Host UART loopback test (Tx wired to Rx)
*/
void hostDebug(void) {

  if ((millis() - host.lastTx) >= HOST_TX_INTVL) {
    String txLine = "Host Tx: " + (String)millis();
    host.uart.println(txLine);
    Serial.println(txLine);
    host.lastTx = millis();
  }

  // Print each complete received line
  static String rxLine = "";      // received so far (kept between calls)

  while (host.uart.available() > 0) {
    char c = (char)host.uart.read();
    if (c == '\n') {
      Serial.println("Host Rx: got[" + rxLine + "]" + " @ " + (String)millis());
      rxLine = "";
    } else if (c != '\r') {
      rxLine += c;
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// LoRa
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  LoRa test: Tx on the OLED, Rx on Serial
  select: 0 => Rx, 1 => Tx, 2 => both
  Tx packets carry the GPS date, time and a count
*/
void loraDebug(uint8_t select) {

  static uint32_t lastBeacon = 0;   // previous test Tx
  static uint32_t txCount = 0;      // test packets sent

  // Keep date and time current (no tasks in debug mode)
  gps.get();

  // Tx: send a test packet every BEACON_INTVL
  if ((select == 1) || (select == 2)) {
    if ((millis() - lastBeacon) >= BEACON_INTVL) {
      char txPkt[64];
      snprintf(txPkt, sizeof(txPkt),
        "Hello! I am %u,"
        "%06" PRIu32 ","          // date (DDMMYY)
        "%08" PRIu32 ","          // time (HHMMSSCC)
        "%" PRIu32,               // count, including this packet
        myself.id, myself.date, myself.time, txCount + 1);

      if (LoRa.beginPacket()) {     // false if the radio is busy
        LoRa.print(txPkt);
        LoRa.endPacket();           // waits until the packet is sent (~30-100 ms at SF7)
        txCount++;                  // only counted once sent

        char txStr[96];
        snprintf(txStr, sizeof(txStr),
          "TX: "
          "%s\n",                   // transmitted payload
          txPkt);
        oled.disp(txStr, 1, false, false);    // fresh screen each Tx
      }
      lastBeacon = millis();
    }
  }

  // Rx: read a received packet and its link quality
  if ((select == 0) || (select == 2)) {
    String rxPkt = "";

    if (LoRa.parsePacket()) {
      while (LoRa.available()) {
        rxPkt += String((char)LoRa.read());
      }
      myself.pktRssi = LoRa.packetRssi();
      myself.snr = LoRa.packetSnr();
      myself.freqErr = LoRa.packetFrequencyError();
    }

    if (rxPkt != "") {
      char rxStr[192];
      snprintf(rxStr, sizeof(rxStr),
        "RX:"
        "%06" PRIu32 ","          // date (DDMMYY)
        "%08" PRIu32 ",%lu,"      // time (HHMMSSCC), ms
        "%.1f,%.2f,%.0f,"         // RX link quality (rssi, snr, freqErr)
        "%s\n",                   // received payload
        myself.date,
        myself.time, millis(),
        myself.pktRssi, myself.snr, myself.freqErr,
        rxPkt.c_str());
      Serial.print(rxStr);
    }
  }
}
