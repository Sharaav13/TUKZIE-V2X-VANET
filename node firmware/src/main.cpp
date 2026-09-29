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

#include <System.h>         // External libraries, macros & shared global variables (config, error)
#include <Display.h>        // OLED display
#include <SDCard.h>         // SD Card
#include <GPS.h>            // GPS & PPS
#include <IMU.h>            // IMU
#include <MNS.h>            // MNS UART
#include <Bluetooth.h>      // TUKZIE dashboard
#include <LoRaRadio.h>      // LoRa
#include <Protocol.h>       // Frame structure & payload
#include <Debug.h>          // Peripheral debugging

//*************************************************************************************************************************************************
// SETUP
//*************************************************************************************************************************************************

void setup() {

  // Serial debugging initialised
  Serial.begin(SERIAL_BAUD);

  // Initialise Tukzie interfaces
  String mnsMessage = uartMNSInit();

  // Initialise OLED
  String oledMessage = oledInit();

  disp("TUKZIE V2X", 2, true, false);
  delay(2000);
  refreshDisp();
  disp("Booting...", 1, true, false);
  delay(2000);
  refreshDisp();

  // Show existing init messages
  disp(mnsMessage, 1, false, false);
  delay(1000);
  disp(oledMessage, 1, false, true);
  delay(1000);

  // Initialise other peripherals
  disp(sdInit(), 1, false, true);
  delay(1000);
  disp(gpsInit(), 1, false, true);
  delay(1000);
  disp(imuInit(), 1, false, true);
  delay(1000);
  disp(loraInit(), 1, false, true);
  delay(3000);

  // Start configurating node
  refreshDisp();
  disp(configNode(), 1, false, false);
  delay(3000);

  // Add necessary setup error message
  if (error) {
    errorMessage += "Setting up\n";
  }

  // Indicate setting up finished
  refreshDisp();
  disp("Finished boot.", 1, true, false);
  delay(3000);

  if (error) {
    disp(errorMessage, 1, false, false);
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
    //disp(errorMessage, 1, false, false);
    //while(true) {delay(1000);}
  //}

  //debug(IMU);
  //debug(GPS);
  //debug(SD_CARD);
  //debug(MNS);
  //debug(LORA);

  // Collect information
  gpsGetAll();
  roadSense();

  // Intepret information

  // Transfer information
}
