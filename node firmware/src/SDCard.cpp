//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SD CARD
//*************************************************************************************************************************************************

#include "SDCard.h"
#include "Display.h"   // sdDebug() calls disp()

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

SPIClass spiSD(FSPI);

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Debug
uint32_t lastSdWrite = 0; // Timestamp of previous SD Card write
uint32_t lastSdRead = 0;  // Timestamp of previous SD Card read

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup SD Card module
String sdInit(void) {

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (SD.begin(SD_CS, spiSD)) {
    return "SD init pass.\n";
  } else {
    error = true;
    return "SD init failed.\n";
  }
}

// Read configuration data from SD Card
String configNode(void) {

  File f = SD.open("/tvv_config.txt");
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
    errorMessage += "Failed to open log file\n";
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
