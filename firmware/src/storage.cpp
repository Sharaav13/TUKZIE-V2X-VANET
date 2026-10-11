//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - STORAGE (SD CARD)
//*************************************************************************************************************************************************

#include "system.h"
#include "storage.h"

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
  Setup SD Card module and delete the log files (CONFIG_FILE is kept)
  Returns true if successful
*/
bool Storage::init(void) {

  spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (SD.begin(SD_CS, spi)) {
    ok = true;
    deleteLogs();     // fresh logs every boot
    return true;
  } else {
    reportError("SD: card not detected or failed to mount\n");
    return false;
  }
}

/*
  Deletes the log files (CONFIG_FILE is kept), write() creates them again when first used
*/
void Storage::deleteLogs(void) {

  const char *logs[] = {SENSE_FILE, NODE_FILE, LOG_FILE, DEBUG_FILE};

  for (const char *file : logs) {
    if (SD.exists(file) && !SD.remove(file)) {
      reportError("SD: failed to delete a log file\n");
    }
  }
}

/*
  Reads "id,fleetId" from CONFIG_FILE into myself (skipped without an SD Card)
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

  // File format: "id,fleetId" e.g. "7,3"
  int id, fleetId;                  // sscanf %d needs int, the Data fields are bytes
  if (sscanf(line.c_str(), "%d,%d", &id, &fleetId) != 2) {
    reportError("SD: config parse failed.\n");
    return "SD: config parse failed.\n";
  }

  myself.id = id;
  myself.fleetId = fleetId;

  return "Config success.\nNode ID: " + String(myself.id) + "\nFleet ID: " + String(myself.fleetId);
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
  file: LOG_FILE, NODE_FILE, SENSE_FILE or DEBUG_FILE
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
