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
  1. Each TVV node has a unique ID, stored in the configuration file on the inserted SD card.
  2. The node retrieves and interprets GPS and IMU information itself.
  3. Status, tasks and battery come from the onboard host, which receives the node and its cluster in return.
  4. Nodes broadcast over LoRa with a MAC protocol, and keep a cluster of the peers they hear.
  5. The OLED shows the node view and the RGB LED flashes the message sent with each beacon.
  6. The USB serial port provides debugging messages.
*/
//*************************************************************************************************************************************************
// LIBRARIES
//*************************************************************************************************************************************************

#include <system.h>         // External libraries, macros, Data struct & shared global variables
#include <display.h>        // OLED display
#include <storage.h>        // SD Card
#include <sensing.h>        // GPS, IMU
#include <interpret.h>      // Coded messages & status, road awareness
#include <host.h>           // Host link (UART), e.g. the TUKZIE dashboard
#include <vanet.h>          // LoRa
#include <diagnostics.h>    // Peripheral debugging, testing and log formatting

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// true: loop() runs the debug calls, no tasks
bool debugMode = false;

// Initialisation results (true = successful)
bool hostReady = false;
bool sdReady = false;
bool gpsReady = false;
bool imuReady = false;
bool vanetReady = false;

// Tasks
TaskHandle_t sensingTask = NULL;    // core 0
TaskHandle_t networkTask = NULL;    // core 1

// Protects myself (written on core 0, read on core 1)
SemaphoreHandle_t myselfMutex = NULL;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

/*
  Creates an init status message and flags an error if the init failed
*/
String initStatus(const char *name, bool ready) {
  if (!ready) {
    errors.check = true;
  }
  return String(name) + (ready ? " init pass.\n" : " init failed.\n");
}

/*
  Shows an init result on the OLED and blinks the RGB LED (green = pass, red = fail) until the next OLED change
  holdMs: time until the next OLED change
*/
void showInit(const char *name, bool ready, bool append, uint32_t holdMs) {
  oled.disp(initStatus(name, ready), 1, false, append);
  lights.toggle(ready ? GREEN : RED);
  delay(holdMs);
  lights.toggle(OFF);
}

/*
  Returns a copy of myself, taken under myselfMutex
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
  Core 0: GPS, IMU and host Rx into myself
*/
void sensingLoop(void *param) {
  for (;;) {

    // Kalman predict step at 100 Hz (without the IMU, speed follows the GPS)
    if (imuReady) {
      speedFilter.update();
    }

    // Collect, interpret and store into myself
    xSemaphoreTake(myselfMutex, portMAX_DELAY);
    gps.get();                              // GPS data and Kalman correct step
    myself.speed = speedFilter.getSpeed();  // fused GPS + IMU speed (kmph)
    if (imuReady) {
      checkAccident();                      // accident or toppled over
      checkTerrain();                       // rough terrain or potholes
    }
    if (hostReady) {
      host.rx();                            // status, tasks and battery from the host
    }
    if (host.present()) {
      checkBattery();                       // only with a host, which is the only source of the battery
    }
    xSemaphoreGive(myselfMutex);

    vTaskDelay(1);                  // yield 1 ms so the idle task feeds the watchdog
  }
}

/*
  Core 1: VANET Rx/Tx, cluster, OLED node view and SD Card logs
*/
void networkLoop(void *param) {
  for (;;) {

    bool received = false;    // a valid packet was read this loop

    if (vanetReady) {

      // Rx and Tx run independently (see the MAC note in vanet.h)

      // Rx: also starts the back-off and updates the cluster
      // Log now: rxBuffer changes on the next rx()
      received = vanet.rx();
      if (received) {
        storage.write(formatVanet("RX", vanet.rxBuffer), LOG_FILE);
      }

      // Tx: broadcast myself with the host message if set, otherwise the top pending message
      if (vanet.beaconDue()) {
        Data me = copyMyself();
        bool hostMsg = (me.message == HOST_MSG);
        if (!hostMsg) {
          me.message = sortMsg();   // other messages stay pending while the host message goes first
        }

        bool sent = vanet.tx(me);

        if (sent) {
          lights.flash(msgColours[me.message]);   // colour of the message sent
          storage.write(formatVanet("TX", me), LOG_FILE);
          if (hostMsg) {
            xSemaphoreTake(myselfMutex, portMAX_DELAY);
            myself.message = 0;             // host message sent once, cleared until the host sends it again
            xSemaphoreGive(myselfMutex);
          }
        } else if ((me.message != 0) && !hostMsg) {
          setMsgFlag(me.message);           // radio busy: retry next beacon (a host message is still in myself)
        }
      }

      // Remove peers not heard from for PEER_TIMEOUT
      if (millis() - vanet.cluster.lastCheck >= PEER_CHECK_INTVL) {
        // The peer whose message is shown has left the cluster: clear its message
        if ((vanet.cluster.removeStale() > 0) && (vanet.cluster.find(vanet.peerMsgId) < 0)) {
          vanet.peerMsg = 0;
        }
        vanet.cluster.lastCheck = millis();
      }
    }

    // Host: myself (with the message in my last beacon) and my cluster peers
    if (hostReady && (millis() - host.lastTx >= HOST_TX_INTVL)) {
      Data me = copyMyself();
      me.message = vanet.lastMsg;
      me.seq = vanet.txSeq;         // sequence number of my last beacon
      me.srcRssi = vanet.txRssi;    // my LoRa.rssi() sent in my last beacon
      host.tx(me, vanet.cluster.peerTable, vanet.cluster.count);
      host.lastTx = millis();
    }

    // OLED node view: every DISP_INTVL, and straight away for a new packet
    if (received || (millis() - oled.lastShow >= DISP_INTVL)) {
      oled.showNode(copyMyself());
      oled.lastShow = millis();
    }

    // RGB: end Tx flashes
    lights.update();

    // Log myself
    if (millis() - storage.lastNodeLog >= NODE_LOG_INTVL) {
      storage.write(formatMyself(copyMyself()), NODE_FILE);
      storage.lastNodeLog = millis();
    }

    // Log sensing
    if (millis() - storage.lastSenseLog >= SENSE_LOG_INTVL) {
      storage.write(formatSensing(copyMyself()), SENSE_FILE);
      storage.lastSenseLog = millis();
    }

    vTaskDelay(1);                  // yield 1 ms so the idle task feeds the watchdog
  }
}

//*************************************************************************************************************************************************
// SETUP
//*************************************************************************************************************************************************

void setup() {

  // Serial debugging initialised
  Serial.begin(SERIAL_BAUD);

  // Protects errors (checked before the tasks start)
  errors.mutex = xSemaphoreCreateMutex();

  // Protects the message flags (checked before the tasks start)
  msgFlagsMutex = xSemaphoreCreateMutex();

  // Initialise RGB LED first, so every init result can blink
  lights.init();

  // Initialise host
  hostReady = host.init();

  // Initialise OLED (a failure is still reported in the error list)
  oled.init();

  oled.disp("TUKZIE V2X", 2, true, false);
  delay(2000);
  oled.refresh();
  oled.disp("Booting...", 1, true, false);
  delay(2000);
  oled.refresh();

  // Show existing init messages (each blinks green or red)
  showInit("Host", hostReady, false, 1000);

  // Initialise other peripherals
  sdReady = storage.init();
  showInit("SD", sdReady, true, 1000);
  gpsReady = gps.init();
  showInit("GPS", gpsReady, true, 1000);
  imuReady = imu.init();
  showInit("IMU", imuReady, true, 1000);
  vanetReady = vanet.init();
  showInit("LoRa", vanetReady, true, 3000);

  // Start configurating node
  oled.refresh();
  oled.disp(storage.configNode(), 1, false, false);
  delay(3000);

  // Add necessary setup error message
  if (errors.check) {
    reportError("Setup: errors occurred during boot\n");
  }

  // Indicate setting up finished
  oled.refresh();
  oled.disp("Finished boot.", 1, true, false);
  delay(3000);

  if (errors.check) {
    oled.disp(errors.msg, 1, false, false);
    lights.toggle(RED);         // red while the errors show
    delay(3000);
    lights.toggle(OFF);
  }

  oled.refresh();

  // Debug mode: no tasks, loop() runs the debug calls
  if (debugMode) {
    return;
  }

  // Start a task on each core (only if all mutexes exist)
  myselfMutex = xSemaphoreCreateMutex();
  if ((myselfMutex == NULL) || (errors.mutex == NULL) || (msgFlagsMutex == NULL)) {
    reportError("Setup: mutex could not be created\n");
    oled.disp(errors.msg, 1, false, false);   // falls back to Serial if the OLED failed
    lights.toggle(RED);                         // red while the errors show (they stay on screen)
    return;
  }

  xTaskCreatePinnedToCore(sensingLoop, "sensing", TASK_STACK, NULL, TASK_PRIORITY, &sensingTask, SENSING_CORE);
  xTaskCreatePinnedToCore(networkLoop, "network", TASK_STACK, NULL, TASK_PRIORITY, &networkTask, NETWORK_CORE);
}

//*************************************************************************************************************************************************
// MAIN CODE/LOOP
//*************************************************************************************************************************************************

void loop() {

  // Peripheral debugging: set debugMode to true, then uncomment one debug call
  //debug(DBG_OLED);
  //debug(DBG_IMU);
  //debug(DBG_GPS);
  //debug(DBG_SD_CARD);
  //debug(DBG_HOST);
  //debug(DBG_LORA);
  //debug(DBG_SENSE);

  // Debug mode: keep looping
  if (debugMode) {
    return;
  }

  // All work runs in sensingLoop (core 0) and networkLoop (core 1)
  vTaskDelete(NULL);    // end the Arduino loop task (frees core 1)
}
