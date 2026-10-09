//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - DIAGNOSTICS
//*************************************************************************************************************************************************

#include "system.h"
#include "diagnostics.h"
#include "display.h"
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
  peripheral: DBG_OLED, DBG_SD_CARD, DBG_GPS, DBG_IMU, DBG_HOST, DBG_LORA_RX, DBG_LORA_TX or DBG_LORA
*/
void debug(uint8_t peripheral) {

  switch (peripheral) {
    case DBG_OLED:
      oledDebug();
      break;
    case DBG_SD_CARD:
      sdDebug();
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
// OLED
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Shows the same test message on the OLED and the Serial Monitor every OLED_TEST_INTVL
*/
void oledDebug(void) {

  static uint32_t lastTest = 0;     // previous OLED test message (kept between calls)

  if ((millis() - lastTest) >= OLED_TEST_INTVL) {
    String msg = "OLED test: " + String(millis()) + " ms\n";
    oled.disp(msg, 1, false, false);
    Serial.print(msg);
    lastTest = millis();
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// SD Card
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Writes a test line to DEBUG_FILE (shown on the OLED) and reads the file back to Serial
*/
void sdDebug(void) {

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
    host.uart.println("Host Tx: " + (String)millis());
    host.lastTx = millis();
  }

  // Print each complete received line
  static String rxLine = "";      // received so far (kept between calls)

  while (host.uart.available() > 0) {
    char c = (char)host.uart.read();
    if (c == '\n') {
      Serial.println("Host Rx: " + rxLine);
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
      myself.rssi = LoRa.packetRssi();
      myself.snr = LoRa.packetSnr();
      myself.freqErr = LoRa.packetFrequencyError();
    }

    if (rxPkt != "") {
      char rxStr[192];
      snprintf(rxStr, sizeof(rxStr),
        "RX:"
        "%06" PRIu32 ","          // date (DDMMYY)
        "%08" PRIu32 ",%lu,"      // time (HHMMSSCC), ms
        "%.1f,%.2f,%.0f,"         // RX diagnostics (rssi, snr, freqErr)
        "%s\n",                   // received payload
        myself.date,
        myself.time, millis(),
        myself.rssi, myself.snr, myself.freqErr,
        rxPkt.c_str());
      Serial.print(rxStr);
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Log Formatting
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Formats a sensing (GPS and IMU) log line for SENSE_FILE
  node: a copy of myself (IMU values come from imu)
  Line: ms,date,time,lat,lon,alt,speed,course,sats,hdop,ax,ay,az,gx,gy,gz
*/
String formatSensing(const Data &node) {
  char line[192];
  snprintf(line, sizeof(line),
    "%lu,"                          // ms
    "%06" PRIu32 ",%08" PRIu32 ","  // date (DDMMYY), time (HHMMSSCC)
    "%.7f,%.7f,%.1f,"               // lat, lon (deg), alt (m)
    "%.1f,%.1f,"                    // speed (kmph), course (deg)
    "%u,%.1f,"                      // sats, hdop
    "%.3f,%.3f,%.3f,"               // acc x, y, z (g)
    "%.2f,%.2f,%.2f\n",             // gyro x, y, z (dps)
    (unsigned long)millis(),
    node.date, node.time,
    node.lat, node.lon, node.alt,
    node.speed, node.course,
    node.sats, node.hdop,
    imu.acc[X_AXIS], imu.acc[Y_AXIS], imu.acc[Z_AXIS],
    imu.gyro[X_AXIS], imu.gyro[Y_AXIS], imu.gyro[Z_AXIS]);
  return String(line);
}

/*
  Formats a VANET log line for LOG_FILE
  event: "RX" or "TX"
  Line: ms,event,src,fleetId,<node>,rssi,snr
  <node> is Data::toStr(): date,time,lat,lon,alt,speed,course,status,tasks,battery,message
*/
String formatVanet(const char *event, const Data &node) {

  // When, what and who: ms,event,src,fleetId,
  char head[48];
  snprintf(head, sizeof(head), "%lu,%s,%u,%u,", (unsigned long)millis(), event, node.id, node.fleetId);

  // Link quality (0 for TX): ,rssi,snr
  char tail[32];
  snprintf(tail, sizeof(tail), ",%.1f,%.2f\n", node.rssi, node.snr);

  return String(head) + node.toStr() + tail;
}

/*
  Formats all of a node's data for NODE_FILE or DEBUG_FILE
  Line: ms,id,fleetId,<node>
*/
String formatMyself(const Data &node) {
  char head[32];
  snprintf(head, sizeof(head), "%lu,%u,%u,", (unsigned long)millis(), node.id, node.fleetId);
  return String(head) + node.toStr() + "\n";
}
