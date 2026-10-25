//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - STORAGE (SD CARD)
//*************************************************************************************************************************************************

#include "system.h"
#include "storage.h"
#include "sensing.h"    // imu, for formatSensing

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

Storage storage;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Storage
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Setup SD Card module
  Returns true if successful
*/
bool Storage::init(void) {

  spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (SD.begin(SD_CS, spi)) {
    ok = true;
    return true;
  } else {
    reportError("SD: card not detected or failed to mount\n");
    return false;
  }
}

/*
  Names the log files with my node ID (call after configNode()), e.g. "/vanet_log7.txt",
  and deletes them for fresh logs every boot
*/
void Storage::nameLogs(void) {

  snprintf(vanetFile, sizeof(vanetFile), "%s%u.txt", VANET_LOG, myself.id);
  snprintf(sensorFile, sizeof(sensorFile), "%s%u.txt", SENSOR_LOG, myself.id);
  snprintf(myselfFile, sizeof(myselfFile), "%s%u.txt", MYSELF_LOG, myself.id);

  if (ok) {
    deleteLogs();     // fresh logs every boot
  }
}

/*
  Deletes the log files and DEBUG_FILE (CONFIG_FILE is kept), write() creates them again when first used
*/
void Storage::deleteLogs(void) {

  const char *logs[] = {vanetFile, sensorFile, myselfFile, DEBUG_FILE};

  for (const char *file : logs) {
    if (SD.exists(file) && !SD.remove(file)) {
      reportError("SD: failed to delete a log file\n");
    }
  }
}

/*
  Reads the node ID from CONFIG_FILE into myself (skipped without an SD Card)
  Returns a summary, or an error message
*/
String Storage::configNode(void) {

  // No SD Card (already reported by init), so no configuration
  if (!ok) {
    return "SD: no card, config skipped.\n";
  }

  File f = SD.open(CONFIG_FILE);
  if (!f) {
    reportError("SD: failed to open config file.\n");
    return "SD: failed to open config file.\n";
  }

  String line = f.readStringUntil('\n');
  f.close();
  line.trim();                      // drops spaces and any trailing '\r'

  // File format: node ID, e.g. "7"
  int id;                           // sscanf %d needs int, the Data field is a byte
  if (sscanf(line.c_str(), "%d", &id) != 1) {
    reportError("SD: config parse failed.\n");
    return "SD: config parse failed.\n";
  }

  myself.id = id;

  return "Config success.\nNode ID: " + String(myself.id);
}

/*
  Returns a file's content (for testing), or an error message
*/
String Storage::read(const char *file) {

  File f = SD.open(file);
  if (f) {

    // Read character by character
    String contents;
    while (f.available()) {
      contents += (char)f.read();
    }
    f.close();
    return contents;
  } else {
    reportError("SD: read failed\n");
    return "SD: read failed\n";
  }
}

/*
  Appends content to a file on the SD Card
  file: vanetFile, sensorFile, myselfFile or DEBUG_FILE
*/
void Storage::write(const String &content, const char *file) {

  // No SD Card (also avoids repeated errors)
  if (!ok) {
    return;
  }

  File f = SD.open(file, FILE_APPEND);
  if (f) {
    f.print(content);
    f.close();
  } else {
    reportError("SD: failed to open log file\n");
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Log Formatting
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Formats a sensing (GPS and IMU) log line for the sensor log (storage.sensorFile)
  node: a copy of myself (IMU values come from imu), with message = the code in my last beacon (vanet.lastMsg)
  Line: ms,date,time,lat,lon,alt,speed,course,sats,hdop,ax,ay,az,gx,gy,gz,message
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
    "%.2f,%.2f,%.2f,"               // gyro x, y, z (dps)
    "%u\n",                         // message code
    (unsigned long)millis(),
    node.date, node.time,
    node.lat, node.lon, node.alt,
    node.speed, node.course,
    node.sats, node.hdop,
    imu.acc[X_AXIS], imu.acc[Y_AXIS], imu.acc[Z_AXIS],
    imu.gyro[X_AXIS], imu.gyro[Y_AXIS], imu.gyro[Z_AXIS],
    node.message);
  return String(line);
}

/*
  Formats a VANET log line for the VANET log (storage.vanetFile)
  event: "RX" (received for me), "TX" (my beacon) or "FW" (forwarded for another node: src and seq are the sender's)
  Line: ms,event,src,<node>,snr,freqErr,utc,airUs,sync,ttl
  <node> is Data::toStr(): date,time,lat,lon,alt,speed,course,status,tasks,battery,message,seq,srcRssi,pktRssi
  srcRssi: my channel RSSI before I sent (TX), 0 otherwise (not carried in packets)
  utc: µs since UTC midnight (PPS time base) of the TX start (TX, FW) or RxDone (RX) | airUs: TX start to TX done (0 for RX)
  sync: 0 = not synced (utc 0), 1 = PPS locked, 2 = holdover
  ttl: hops left as sent (TX, FW) or received (RX): TTL_INIT = direct, less = forwarded
*/
String formatVanet(const char *event, const Data &node) {

  // When, what and who: ms,event,src,
  char head[48];
  snprintf(head, sizeof(head), "%lu,%s,%u,", (unsigned long)millis(), event, node.id);

  // Link quality (pktRssi, snr and freqErr are 0 for TX), seq and srcRssi are in <node>: ,snr,freqErr
  // Packet timing and hops left: ,utc,airUs,sync,ttl
  char tail[80];
  snprintf(tail, sizeof(tail), ",%.2f,%.0f,%lld,%lu,%u,%u\n", node.snr, node.freqErr,
    (long long)node.utc, (unsigned long)node.airUs, node.sync, node.ttl);

  return String(head) + node.toStr() + tail;
}

/*
  Formats all of a node's data for the myself log (storage.myselfFile) or DEBUG_FILE
  Line: ms,id,<node>
*/
String formatMyself(const Data &node) {
  char head[32];
  snprintf(head, sizeof(head), "%lu,%u,", (unsigned long)millis(), node.id);
  return String(head) + node.toStr() + "\n";
}
