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
  Read configuration data from SD Card
*/
String Storage::configNode(void) {

  File f = SD.open(CFG_PATH);
  if (!f) {
    reportError("SD: failed to open config file.\n");
    return "SD: failed to open config file.\n";
  }

  String line = f.readStringUntil('\n');
  f.close();
  line.trim();                      // drops spaces and any trailing '\r'

  for (int i = 0; i < CFG_FIELDS; i++) {
    int comma = line.indexOf(',');

    if (comma < 0) {                // last field, no comma after it
      if (i != CFG_FIELDS - 1) {
        reportError("SD: config parse failed.\n");
        return "SD: config parse failed.\n";
      }
      config[i] = line.toInt();
    } else {
      config[i] = line.substring(0, comma).toInt();
      line = line.substring(comma + 1);
    }
  }

  myself.id = config[0];
  myself.fleetId = config[1];

  String configData = "Config success.\nNode ID: "  + (String)config[0] + "\nUse: ";
  switch (config[1]) {
    case 0:
      configData = configData + "Tukzie Fleet " + (String)config[2];
      break;

    case 1:
      configData = configData + "Fleet " + (String)config[2];
      break;

    case 2:
      configData = configData + "Private";
      break;
  }

  return configData;
}

/*
  Used for SD Card module testing and node debugging
  Returns the content as a String
*/
String Storage::read(const char *readFile) {

  char path[SD_PATH_SIZE];
  snprintf(path, sizeof(path), "/%s.txt", readFile);

  File f = SD.open(path);
  if (f) {

    // Read number of lines
    // size_t len = logFile.size();
    // oled.disp((String)len, 1, false, false);

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
  Write/Log contents to SD Card
  Used for node debugging and testing
*/
void Storage::write(const String &content, const char *logFile) {

  char path[SD_PATH_SIZE];
  snprintf(path, sizeof(path), "/%s.txt", logFile);

  File f = SD.open(path, FILE_APPEND);

  if (f) {
    f.print(content);
    f.close();
    // oled.disp("Logged to SD", 1, false, false); // debugging
  } else {
    reportError("SD: failed to open log file\n");
  }
}
