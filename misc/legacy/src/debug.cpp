//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - DEBUG
//*************************************************************************************************************************************************

#include "system.h"
#include "debug.h"
#include "display.h"
#include "storage.h"
#include "sensing.h"
#include "dashboard.h"
#include "vanet.h"

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

/*
  Runs the debug routine of the selected peripheral
*/
void debug(uint8_t peripheral) {

  switch (peripheral) {
    case DBG_SD_CARD:
      sdDebug();
      break;
    case DBG_GPS:
      gpsDebug();
      break;
    case DBG_PPS_CLOCK:
      clockDiff();
      break;
    case DBG_IMU:
      imuDebug();
      break;
    case DBG_DASH:
      dashDebug();
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

/*
  Writes and reads to the SD card
*/
void sdDebug(void) {

  const char *debugFile = "sd_debug";

  if ((millis() - storage.lastWrite) >= SD_WRITE_INTVL) {
    String write_content = (String)millis() + "SD debugging\n";
    storage.write(write_content, debugFile);
    storage.lastWrite = millis();
  }

  if ((millis() - storage.lastRead) >= (SD_READ_INTVL)) {
    String read_content = storage.read(debugFile);
    oled.disp(read_content, 1, false, false);
    storage.lastRead = millis();
  }
}

/*
  Calculates the error in clock synchronisation with PPS
*/
void clockDiff(void) {
  String clock_diff = "PPS Count = " + String(countPps) + "\n" + "Clock erro = " + String(gapMs);
  oled.disp(clock_diff, 1, false, false);
}

/*
  Converts currently stored GPS data into a single string
  Used for GPS module testing
*/
size_t gpsStr(char *out, size_t cap) {

  return snprintf(out, cap,
        "%06" PRIu32 "  %08" PRIu32 " SAST\n"
        "lat: %.4f deg\n"
        "lon: %.4f deg\n"
        "alt: %.1f m\n"
        "speed: %.1f kmph\n"
        "course: %.1f deg\n"
        "sats: %d   hdop: %.1f",
        myself.date, myself.time,
        myself.lat,
        myself.lon,
        myself.alt,
        myself.speed, myself.course,
        myself.sats, myself.hdop);
}

/*
  Displays all GPS data periodically
*/
void gpsDebug(void) {
  if ((millis() - gps.lastSample) >= GPS_SAMPLING) {
    gps.get();
    char gps_str[192];
    gpsStr(gps_str, sizeof(gps_str));
    oled.disp(gps_str, 1, false, false);
    gps.lastSample = millis();
  }
}

/*
  Converts currently stored IMU data into a single string
  Used for IMU module testing
*/
size_t imuStr(char *out, size_t cap) {

  return  snprintf(out, cap,
        "A: %.2f %.2f %.2f\n"
        "G: %.2f %.2f %.2f",
        imu.acc[X_AXIS], imu.acc[Y_AXIS], imu.acc[Z_AXIS],
        imu.gyro[X_AXIS], imu.gyro[Y_AXIS], imu.gyro[Z_AXIS]);
}

/*
  Displays all IMU data periodically
*/
void imuDebug(void) {
  if ((millis() - imu.lastSample) >= IMU_SAMPLING) {
    imu.getAll();
    char imu_str[192];
    imuStr(imu_str, sizeof(imu_str));
    oled.disp(imu_str, 1, false, false);
    imu.lastSample = millis();
  }
}

/*
  Debugs if dashboard Tx and Rx ports are working
  Tx and Rx are wired together
*/
void dashDebug(void) {

  if ((millis() - dash.lastTx) >= DASH_TX_INTVL) {
    dash.uart.println("Dash Tx:" + (String)millis());
    dash.lastTx = millis();
  }

  while (dash.uart.available() > 0) {
    oled.disp(String((char)dash.uart.read()), 1, false, true);
  }
}

/*
  Procedure for receiving LoRa packet
*/
String loraRx(void) {

  String msg = "";

  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    while (LoRa.available()) {
      msg += String((char)LoRa.read());
    }
    myself.rssi = LoRa.packetRssi();
    myself.snr =  LoRa.packetSnr();
    myself.freqErr = LoRa.packetFrequencyError();
  }
  return msg;
}

/*
  Procedure for transmitting LoRa packet
*/
void loraTx(String payload) {

  if (!LoRa.beginPacket()) {
    return; // radio busy
  } else {
    LoRa.print(payload);
    LoRa.endPacket();     // waits until the packet is sent (~30-100 ms at SF7)
  }
}

/*
  Converts LoRa Rx information into a single string
*/
size_t loraRxStr(char *out, size_t cap, String pkt) {
  return snprintf(out, cap,
        "RX:"
        "%06" PRIu32 ","        // date (DDMMYY)
        "%08" PRIu32 ",%lu,"    // time (HHMMSSCC)
        "%.1f,%.2f,%.0f,"       // RX diagnostics (rssi, snr, freqErr)
        "%s\n",                 // received payload
        myself.date,
        myself.time, millis(),
        myself.rssi, myself.snr, myself.freqErr,
        pkt.c_str());
}

/*
  Converts LoRa Tx information into a single string
*/
size_t loraTxStr(char *out, size_t cap, String pkt) {
  return snprintf(out, cap,
        "TX: "
        "%s\n",                 // transmitted payload
        pkt.c_str());
}

/*
  Shows LoRa Tx and Rx
*/
void loraDebug(uint8_t select) {

  static uint32_t lastBeacon = 0;   // previous LoRa debug transmission (kept between calls)

  if ((select == 1) || (select == 2)){
    if ((millis() - lastBeacon) >= BEACON_INTVL) {
      String txPkt = "Hello there!";
      loraTx(txPkt);
      char txStr[192];
      if (select != 2) {
        loraTxStr(txStr, sizeof(txStr), txPkt);
        oled.disp(txStr, 1, false, false);
      }
      lastBeacon = millis();
    }
  }

  if ((select == 0) || (select == 2)){
    String rxPkt = loraRx();
    if (rxPkt != "") {
      char rxStr[192];
      loraRxStr(rxStr, sizeof(rxStr), rxPkt);
      oled.disp(rxStr, 1, false, false);
    }
  }
}
