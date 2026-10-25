//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - STORAGE (SD CARD)
//*************************************************************************************************************************************************

#ifndef TVV_STORAGE_H
#define TVV_STORAGE_H

#include "system.h"

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

// SD Card module
struct Storage {

  SPIClass spi{FSPI};

  // Status
  bool ok = false;

  // Timestamps (ms)
  uint32_t lastWrite = 0;       // previous SD Card write
  uint32_t lastRead = 0;        // previous SD Card read
  uint32_t lastNodeLog = 0;     // previous log of myself
  uint32_t lastSenseLog = 0;    // previous log of sensing

  // Log files, named with my node ID by nameLogs() (ID 0 until then)
  char vanetFile[24] = VANET_LOG "0.txt";     // VANET Rx, Tx and forwards
  char sensorFile[24] = SENSOR_LOG "0.txt";   // my GPS and IMU data
  char myselfFile[24] = MYSELF_LOG "0.txt";   // all my data

  /*
    Setup SD Card module
    Returns true if successful
  */
  bool init(void);

  /*
    Names the log files with my node ID (call after configNode()), e.g. "/vanet_log7.txt",
    and deletes them for fresh logs every boot
  */
  void nameLogs(void);

  /*
    Deletes the log files and DEBUG_FILE (CONFIG_FILE is kept), write() creates them again when first used
  */
  void deleteLogs(void);

  /*
    Reads the node ID from CONFIG_FILE into myself (skipped without an SD Card)
    Returns a summary, or an error message
  */
  String configNode(void);

  /*
    Returns a file's content (for testing), or an error message
  */
  String read(const char *file);

  /*
    Appends content to a file on the SD Card
    file: vanetFile, sensorFile, myselfFile or DEBUG_FILE
  */
  void write(const String &content, const char *file);
};

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Log Formatting
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Formats a sensing (GPS and IMU) log line for the sensor log (storage.sensorFile)
  node: a copy of myself (IMU values come from imu), with message = the code in my last beacon (vanet.lastMsg)
  Line: ms,date,time,lat,lon,alt,speed,course,sats,hdop,ax,ay,az,gx,gy,gz,message
*/
String formatSensing(const Data &node);

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
String formatVanet(const char *event, const Data &node);

/*
  Formats all of a node's data for the myself log (storage.myselfFile) or DEBUG_FILE
  Line: ms,id,<node>
*/
String formatMyself(const Data &node);

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern Storage storage;

#endif // TVV_STORAGE_H
