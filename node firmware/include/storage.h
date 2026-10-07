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

  /*
    Setup SD Card module
    Returns true if successful
  */
  bool init(void);

  /*
    Read configuration data from SD Card
    Saves the node ID to myself.id and the fleet ID to myself.fleetId
    Returns a summary of the configuration, or an error message if it failed
  */
  String configNode(void);

  /*
    Used for SD Card module testing and node debugging
    Returns the content as a String, or an error message if the read failed
  */
  String read(const char *readFile);

  /*
    Write/Log contents to SD Card
    Used for node debugging and testing
  */
  void write(const String &content, const char *logFile);
};

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern Storage storage;

#endif // TVV_STORAGE_H
