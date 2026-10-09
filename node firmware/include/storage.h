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

  /*
    Setup SD Card module
    Returns true if successful
  */
  bool init(void);

  /*
    Reads "id,fleetId" from CONFIG_FILE into myself (skipped without an SD Card)
    Returns a summary, or an error message
  */
  String configNode(void);

  /*
    Returns a file's content (for testing), or an error message
  */
  String read(const char *file);

  /*
    Appends content to a file on the SD Card
    file: LOG_FILE, NODE_FILE, SENSE_FILE or DEBUG_FILE
  */
  void write(const String &content, const char *file);
};

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern Storage storage;

#endif // TVV_STORAGE_H
