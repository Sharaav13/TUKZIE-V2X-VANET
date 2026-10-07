//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE
//*************************************************************************************************************************************************
/*
  Version:  0
  Date:     09/10/2026
  Authors:  Sharaav Dhebideen
*/
//*************************************************************************************************************************************************
/*
  DESCRIPTION:
  1. Each TVV node has an uniquie ID, stored in teh configuration data on the inserted SD card.
  2. The node retrieves and interprets GPS and IMU information itself.
  3. Other information is exchanged with it via the onboard TUKZIE dashboard.
  4. Nodes communicate using LoRa and implement a MAC and Routing protocols.
  5. The TVV mobile app connects via Bluetooth to a node to display messages and network topology of all nodes.
  6. Dashboard port can provide serial messages for debugging.
*/
//*************************************************************************************************************************************************
// LIBRARIES
//*************************************************************************************************************************************************

#include <system.h>         // External libraries, macros, Data struct & shared global variables
#include <display.h>        // OLED display
#include <storage.h>        // SD Card
#include <sensing.h>        // GPS & PPS, IMU
#include <dashboard.h>      // TUKZIE dashboard
#include <vanet.h>          // LoRa
#include <debug.h>          // Peripheral debugging

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Initialisation results (true = successful)
bool dashReady = false;
bool oledReady = false;
bool rgbReady = false;
bool sdReady = false;
bool gpsReady = false;
bool imuReady = false;
bool vanetReady = false;

// Tasks
TaskHandle_t sensingTask = NULL;    // core 0
TaskHandle_t networkTask = NULL;    // core 1

// Protects myself, which is written on core 0 and read on core 1
SemaphoreHandle_t myselfMutex = NULL;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

/*
  Creates an init status message and flags an error if the init failed
*/
String initStatus(const char *name, bool ready) {
  if (!ready) {
    error = true;
  }
  return String(name) + (ready ? " init pass.\n" : " init failed.\n");
}

/*
  Returns a copy of myself taken under myselfMutex
  Used on core 1 so myself is never read while core 0 is updating it
*/
Data copyMyself(void) {
  Data copy;
  xSemaphoreTake(myselfMutex, portMAX_DELAY);
  copy = myself;
  xSemaphoreGive(myselfMutex);
  return copy;
}

//*************************************************************************************************************************************************
// TASKS
//*************************************************************************************************************************************************

/*
  Core 0: collects and interprets GPS and IMU data, receives dashboard packets and stores them in myself
*/
void sensingLoop(void *param) {
  for (;;) {

    // Interpret IMU data (Kalman filter predict step at 100 Hz, does not touch myself)
    // Skipped if the IMU failed to start or calibrate: the filter then only receives GPS corrections, so it follows the GPS speed
    if (imuReady) {
      speedFilter.update();
    }

    // Collect, interpret and store into myself
    xSemaphoreTake(myselfMutex, portMAX_DELAY);
    gps.get();                              // GPS data, also runs the Kalman filter correct step
    myself.speed = speedFilter.getSpeed();  // fused GPS + IMU speed (km/h)
    if (imuReady) {
      senseTerrain();                       // road awareness from the IMU
    }
    dash.rx();                              // status, tasks, battery and message from the dashboard
    xSemaphoreGive(myselfMutex);

    vTaskDelay(1);                  // yield (1 tick = 1 ms) so the idle task can feed the watchdog
  }
}

/*
  Core 1: handles VANET Rx and Tx and logs data to the SD Card
*/
void networkLoop(void *param) {
  for (;;) {

    if (vanetReady) {

      // Rx, logging each packet received
      if ((vanet.rx() >= 0) && storage.ok) {
        storage.write("RX," + String(vanet.buffer.id) + "," + vanet.buffer.toStr() + "\n", LOG_FILE);
      }

      // Tx, broadcasting myself
      if (millis() - vanet.lastTx >= BEACON_INTVL) {
        vanet.tx(0, copyMyself());
        vanet.lastTx = millis();
      }
    }

    // Log myself
    if (storage.ok && (millis() - storage.lastWrite >= LOG_INTVL)) {
      Data me = copyMyself();
      storage.write("ME," + String(me.id) + "," + me.toStr() + "\n", LOG_FILE);
      storage.lastWrite = millis();
    }

    vTaskDelay(1);                  // yield (1 tick = 1 ms) so the idle task can feed the watchdog
  }
}

//*************************************************************************************************************************************************
// SETUP
//*************************************************************************************************************************************************

void setup() {

  // Serial debugging initialised
  Serial.begin(SERIAL_BAUD);

  // Protects error and errMsg once both cores are running (checked before the tasks start)
  errMutex = xSemaphoreCreateMutex();

  // Initialise TUKZIE dashboard
  dashReady = dash.init();

  // Initialise OLED
  oledReady = oled.init();

  oled.disp("TUKZIE V2X", 2, true, false);
  delay(2000);
  oled.refresh();
  oled.disp("Booting...", 1, true, false);
  delay(2000);
  oled.refresh();

  // Show existing init messages
  oled.disp(initStatus("Dashboard", dashReady), 1, false, false);
  delay(1000);
  oled.disp(initStatus("OLED", oledReady), 1, false, true);
  delay(1000);

  // Initialise other peripherals
  rgbReady = lights.init();
  oled.disp(initStatus("RGB", rgbReady), 1, false, true);
  delay(1000);
  sdReady = storage.init();
  oled.disp(initStatus("SD", sdReady), 1, false, true);
  delay(1000);
  gpsReady = gps.init();
  oled.disp(initStatus("GPS", gpsReady), 1, false, true);
  delay(1000);
  imuReady = imu.init();
  oled.disp(initStatus("IMU", imuReady), 1, false, true);
  delay(1000);
  vanetReady = vanet.init();
  oled.disp(initStatus("LoRa", vanetReady), 1, false, true);
  delay(3000);

  // Start configurating node
  oled.refresh();
  oled.disp(storage.configNode(), 1, false, false);
  delay(3000);

  // Add necessary setup error message
  if (error) {
    reportError("Setup: errors occurred during boot\n");
  }

  // Indicate setting up finished
  oled.refresh();
  oled.disp("Finished boot.", 1, true, false);
  delay(3000);

  if (error) {
    oled.disp(errMsg, 1, false, false);
    delay(3000);
  }

  oled.refresh();

  // Start a task on each core (only if both mutexes exist, otherwise shared data would be unprotected)
  myselfMutex = xSemaphoreCreateMutex();
  if ((myselfMutex == NULL) || (errMutex == NULL)) {
    reportError("Setup: mutex could not be created\n");
    oled.disp(errMsg, 1, false, false);
    return;
  }

  xTaskCreatePinnedToCore(sensingLoop, "sensing", TASK_STACK, NULL, TASK_PRIORITY, &sensingTask, SENSING_CORE);
  xTaskCreatePinnedToCore(networkLoop, "network", TASK_STACK, NULL, TASK_PRIORITY, &networkTask, NETWORK_CORE);
}

//*************************************************************************************************************************************************
// MAIN CODE/LOOP
//*************************************************************************************************************************************************

void loop() {

  // Modify for debugging operation => wantthe errors to show without halting function
  // Show error in setup or operation
  //if (error) {
    //oled.disp(errMsg, 1, false, false);
    //while(true) {delay(1000);}
  //}

  // Peripheral debugging: stop the tasks being created at the end of setup, then uncomment one debug call
  //debug(DBG_IMU);
  //debug(DBG_GPS);
  //debug(DBG_SD_CARD);
  //debug(DBG_DASH);
  //debug(DBG_LORA);

  // All work runs in sensingLoop (core 0) and networkLoop (core 1)
  vTaskDelete(NULL);    // removes the Arduino loop task so it does not compete with networkLoop on core 1
}
