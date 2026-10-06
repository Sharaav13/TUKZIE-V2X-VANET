//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE
//*************************************************************************************************************************************************
/*
  Version:  0
  Date:     30/09/2026
  Authors:  Sharaav Dhebideen
*/
//*************************************************************************************************************************************************
/*
  DESCRIPTION:
  1. Each TVV node has an uniquie ID, stored in teh configuration data on the inserted SD card.
  2. The node retrieves and interprets GPS and IMU information itself.
  3. Other information is exchanged with it via the onboard Multi Network System (MNS) and Heads Up Display (HUD) in Tukzie.
  4. Nodes communicate using LoRa and implement a MAC and Routing protocols.
  5. The TVV mobile app connects via Bluetooth to a node to display messages and network topology of all nodes.
  6. MNS port can provides serial meassages for debugging.
*/
//*************************************************************************************************************************************************
// LIBRARIES
//*************************************************************************************************************************************************

#include <system.h>         // External libraries, macros, Data struct & shared global variables
#include <display.h>        // OLED display
#include <sdcard.h>         // SD Card
#include <gps.h>            // GPS & PPS
#include <imu.h>            // IMU
#include <mns.h>            // MNS UART
#include <dashboard.h>      // TUKZIE dashboard
#include <vanet.h>          // LoRa
#include <debug.h>          // Peripheral debugging

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Initialisation results (true = successful)
bool mnsReady = false;
bool oledReady = false;
bool rgbReady = false;
bool sdReady = false;
bool gpsReady = false;
bool imuReady = false;
bool vanetReady = false;
bool dashReady = false;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Creates an init status message and flags an error if the init failed
String initStatus(String name, bool ready) {
  if (!ready) {
    error = true;
  }
  return name + (ready ? " init pass.\n" : " init failed.\n");
}

//*************************************************************************************************************************************************
// SETUP
//*************************************************************************************************************************************************

void setup() {

  // Serial debugging initialised
  Serial.begin(SERIAL_BAUD);

  // Initialise Tukzie interfaces
  mnsReady = mnsInit();

  // Initialise OLED
  oledReady = oledInit();

  disp("TUKZIE V2X", 2, true, false);
  delay(2000);
  refreshDisp();
  disp("Booting...", 1, true, false);
  delay(2000);
  refreshDisp();

  // Show existing init messages
  disp(initStatus("MNS", mnsReady), 1, false, false);
  delay(1000);
  disp(initStatus("OLED", oledReady), 1, false, true);
  delay(1000);

  // Initialise other peripherals
  rgbReady = rgbInit();
  disp(initStatus("RGB", rgbReady), 1, false, true);
  delay(1000);
  sdReady = sdInit();
  disp(initStatus("SD", sdReady), 1, false, true);
  delay(1000);
  gpsReady = gpsInit();
  disp(initStatus("GPS", gpsReady), 1, false, true);
  delay(1000);
  imuReady = imuInit();
  disp(initStatus("IMU", imuReady), 1, false, true);
  delay(1000);
  vanetReady = vanetInit();
  disp(initStatus("LoRa", vanetReady), 1, false, true);
  delay(1000);
  dashReady = dashInit();
  disp(initStatus("BLE", dashReady), 1, false, true);
  delay(3000);

  // Start configurating node
  refreshDisp();
  disp(configNode(), 1, false, false);
  delay(3000);

  // Add necessary setup error message
  if (error) {
    errMsg += "Setup: errors occurred during boot\n";
  }

  // Indicate setting up finished
  refreshDisp();
  disp("Finished boot.", 1, true, false);
  delay(3000);

  if (error) {
    disp(errMsg, 1, false, false);
    delay(3000);
  }

  refreshDisp();
}

//*************************************************************************************************************************************************
// MAIN CODE/LOOP
//*************************************************************************************************************************************************

void loop() {

  // Modify for debugging operation => wantthe errors to show without halting function
  // Show error in setup or operation
  //if (error) {
    //disp(errMsg, 1, false, false);
    //while(true) {delay(1000);}
  //}

  //debug(IMU);
  //debug(GPS);
  //debug(SD_CARD);
  //debug(MNS);
  //debug(LORA);

  // Collect information
  gpsGet();
  roadSense();

  // Intepret information

  // Transfer information
}
