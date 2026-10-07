//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SD CARD
//*************************************************************************************************************************************************

#include "system.h"
#include "sdcard.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

SPIClass spiSD(FSPI);

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup SD Card module
// Returns true if successful
bool sdInit(void) {

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (SD.begin(SD_CS, spiSD)) {
    sdOK = true;
    return true;
  } else {
    error = true;
    errMsg += "SD: card not detected or failed to mount\n";
    return false;
  }
}

// Read configuration data from SD Card
String configNode(void) {

  File f = SD.open(CFG_PATH);
  if (!f) {
    error = true;
    return "Config open failed.\n";
  }

  String line = f.readStringUntil('\n');
  f.close();
  line.trim();                      // drops spaces and any trailing '\r'

  for (int i = 0; i < CFG_FIELDS; i++) {
    int comma = line.indexOf(',');

    if (comma < 0) {                // last field, no comma after it
      if (i != CFG_FIELDS - 1) {
        error = true;
        return "Config parse failed.\n";
      }
      config[i] = line.toInt();
    } else {
      config[i] = line.substring(0, comma).toInt();
      line = line.substring(comma + 1);
    }
  }

  myself.id = config[0];

  String configData = "Config success.\nNode ID: "  + (String)config[0] + "\nUse: ";
  switch (config[1]) {
    case 0:
      configData = configData + "Tukzie Fleet " + (String)config[1];
      break;

    case 1:
      configData = configData + "Fleet " + (String)config[1];
      break;

    case 2:
      configData = configData + "Private";
      break;
  }

  return configData;
}

// Used for SD Card module testing and node debugging
// Returns teh content as a String
String sdRead(String readFile) {

  File f = SD.open("/" + readFile + ".txt");
  if (f) {

    // Read number of lines
    // size_t len = logFile.size();
    // disp((String)len, 1, false, false);

    // Read character by character
    String contents;
    while (f.available()) {
      contents += (char)f.read();
    }
    f.close();
    return contents;
  } else {
    return "SD Card read failed.";
  }
}

// Write/Log contents to SD Card
// Used for node debugging and testing
void sdWrite(String content, String logFile) {

  File f = SD.open("/" + logFile + ".txt", FILE_APPEND);

  if (f) {
    f.print(content);
    f.close();
    // disp("Logged to SD", 1, false, false); // debugging
  } else {
    errMsg += "SD: failed to open log file\n";
  }
}
