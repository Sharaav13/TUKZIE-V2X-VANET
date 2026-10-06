//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - DEBUG
//*************************************************************************************************************************************************

#include "system.h"
#include "debug.h"
#include "display.h"
#include "sdcard.h"
#include "gps.h"
#include "imu.h"
#include "mns.h"
#include "vanet.h"

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Timestamps of previous debug actions
uint32_t lastSdWrite = 0; // Timestamp of previous SD Card write
uint32_t lastSdRead = 0;  // Timestamp of previous SD Card read
uint32_t lastGps = 0;     // Timestamp of previous GPS sample
uint32_t lastImu = 0;     // Timestamp of previous IMU sample
uint32_t lastHudTx = 0;   // Timestamp of previous MNS transmission
uint32_t lastBeacon = 0;  // Timestamp of previous LoRa transmission

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Runs the debug routine of the selected peripheral
void debug(int peripheral) {

  switch (peripheral) {
    case SD_CARD:
      sdDebug();
      break;
    case GPS:
      gpsDebug();
      break;
    case PPS_CLOCK:
      clockDiff();
      break;
    case IMU:
      imuDebug();
      break;
    case MNS:
      uartMNSDebug();
      break;
    case LORA_RX:
      loraDebug(0);
      break;
    case LORA_TX:
      loraDebug(1);
      break;
    case LORA:
      loraDebug(2);
      break;
  }
}

// Writes and reads to the SD card
void sdDebug(void) {

  String debugFile = "sd_debug";

  if ((millis() - lastSdWrite) >= SD_WRITE_INTVL) {
    String write_content = (String)millis() + "SD debugging\n";
    sdWrite(write_content, debugFile);
    lastSdWrite = millis();
  }

  if ((millis() - lastSdRead) >= (SD_READ_INTVL)) {
    String read_content = sdRead(debugFile);
    disp(read_content, 1, false, false);
    lastSdRead = millis();
  }
}

// Calculates the error in clock synchronisation with PPS
void clockDiff(void) {
  String clock_diff = "PPS Count = " + String(countPps) + "\n" + "Clock erro = " + String(gapMs);
  disp(clock_diff, 1, false, false);
}

// Converts currently stored GPS data into a single string
// Used for GPS module testing
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

// Displays all GPS data periodically
void gpsDebug(void) {
  if ((millis() - lastGps) >= GPS_SAMPLING) {
    gpsGet();
    char gps_str[192];
    gpsStr(gps_str, sizeof(gps_str));
    disp(gps_str, 1, false, false);
    lastGps = millis();
  }
}

// Converts currently stored IMU data into a single string
// Used for IMU module testing
size_t imuStr(char *out, size_t cap) {

  return  snprintf(out, cap,
        "A: %.2f %.2f %.2f\n"
        "G: %.2f %.2f %.2f",
        imuVals[ACC][X_AXIS], imuVals[ACC][Y_AXIS], imuVals[ACC][Z_AXIS],
        imuVals[GYRO][X_AXIS], imuVals[GYRO][Y_AXIS], imuVals[GYRO][Z_AXIS]);
}

// Displays all IMU data periodically
void imuDebug(void) {
  if ((millis() - lastImu) >= IMU_SAMPLING) {
    imuGetAll();
    char imu_str[192];
    imuStr(imu_str, sizeof(imu_str));
    disp(imu_str, 1, false, false);
    lastImu = millis();
  }
}

// Debugs if HUD Tx and Rx ports are working
// Tx and Rx are wired together
void uartMNSDebug(void) {

  if ((millis() - lastHudTx) >= MNS_TX_INTVL) {
    uartMNS.println("HUD Tx:" + (String)millis());
    lastHudTx = millis();
  }

  while (uartMNS.available() > 0) {
    disp(String((char)uartMNS.read()), 1, false, true);
  }
}

// Procedure for receiving LoRa packet
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

// Procedure for transmitting LoRa packet
void loraTx(String payload) {

  if (!LoRa.beginPacket()) {
    return; // radio busy
  } else {
    LoRa.print(payload);
    LoRa.endPacket();     // waits until the packet is sent (~30-100 ms at SF7)
  }
}

// Converts LoRa Rx information into a single string
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

// Converts LoRa Tx information into a single string
size_t loraTxStr(char *out, size_t cap, String pkt) {
  return snprintf(out, cap,
        "TX: "
        "%s\n",                 // transmitted payload
        pkt.c_str());
}

// Shows LoRa Tx and Rx
void loraDebug(int select) {
  if ((select == 1) || (select == 2)){
    if ((millis() - lastBeacon) >= BEACON_INTVL) {
      String txPkt = "Hello there!";
      loraTx(txPkt);
      char txStr[192];
      if (select != 2) {
        loraTxStr(txStr, sizeof(txStr), txPkt);
        disp(txStr, 1, false, false);
      }
      lastBeacon = millis();
    }
  }

  if ((select == 0) || (select == 2)){
    String rxPkt = loraRx();
    if (rxPkt != "") {
      char rxStr[192];
      loraRxStr(rxStr, sizeof(rxStr), rxPkt);
      disp(rxStr, 1, false, false);
    }
  }
}
