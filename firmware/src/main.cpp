//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE
//*************************************************************************************************************************************************
/*
  Version:  1.0
  Date:     27/10/2026
  Authors:  Sharaav Dhebideen
*/
//*************************************************************************************************************************************************
/*
  DESCRIPTION:
  1. Each TVV node has a unique ID, stored in the configuration file on the inserted SD card.
  2. The node retrieves and interprets GPS and IMU information itself.
  3. Status, tasks and battery come from the onboard host, which receives the node and its cluster in return.
  4. Nodes send beacons over LoRa with a CSMA/CA-style MAC, keep a cluster of the peers they hear, and forward packets
     addressed to other nodes once.
  5. The OLED shows the node view. The RGB LED flashes on each beacon sent and shows the conditions detected in between.
  6. Every boot starts fresh SD Card logs, and the USB serial port provides debugging messages.
*/
//*************************************************************************************************************************************************
// LIBRARIES
//*************************************************************************************************************************************************

#include <system.h>         // External libraries, macros, Data struct & shared global variables
#include <ui.h>             // OLED display, RGB LED
#include <storage.h>        // SD Card
#include <sensing.h>        // GPS, IMU
#include <interpret.h>      // Coded messages & status, road awareness
#include <host.h>           // Host link (UART), e.g. the TUKZIE dashboard
#include <vanet.h>          // LoRa
#include <debugging.h>      // Peripheral debugging and testing

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
TaskHandle_t v2sTask = NULL;    // core 0
TaskHandle_t v2vTask = NULL;    // core 1

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
  Core 0: GPS, IMU (speed fusion, road awareness) and host Rx into myself
*/
void v2sLoop(void *param) {
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
      checkTerrain();                       // bad terrain (bump, pothole or rough surface)
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
  Core 1: VANET Rx/Tx and forwards, cluster, host Tx, OLED node view, RGB LED and SD Card logs
*/
void v2vLoop(void *param) {
  for (;;) {

    bool received = false;    // a valid packet was read this loop

    if (vanetReady) {

      // Rx and Tx run independently (see the MAC note in vanet.h)

      // Rx: also starts the back-off and updates the cluster
      // Log now: rxBuffer changes on the next rx()
      received = vanet.rx();
      if (received) {
        storage.write(formatVanet("RX", vanet.rxBuffer), storage.vanetFile);
      }

      // Forward: a packet addressed to another node, sent on as received (in a transmission slot shared with my
      // beacons, before them, after the back-off, with the channel free)
      if (vanet.forwardDue()) {
        Data fwd;
        if (vanet.forward(fwd)) {
          storage.write(formatVanet("FW", fwd), storage.vanetFile);
        }
      }

      // Tx: broadcast myself with the host message if set, otherwise the top pending message
      if (vanet.beaconDue()) {

        // The host message is taken with the copy (sent once): one the host sends while this beacon is on air
        // (~1.5 s at SF12) is kept in myself for the next beacon instead of being cleared after it
        xSemaphoreTake(myselfMutex, portMAX_DELAY);
        Data me = myself;
        bool hostMsg = (me.message == HOST_MSG);
        if (hostMsg) {
          myself.message = 0;
        }
        xSemaphoreGive(myselfMutex);

        if (!hostMsg) {
          me.message = sortMsg();   // other messages stay pending while the host message goes first
        }

        bool sent = vanet.tx(me);

        if (sent) {
          lights.flash(msgColours[hostMsg ? HOST_MSG : 0]);   // green for a beacon, blue for one carrying the host message
          storage.write(formatVanet("TX", me), storage.vanetFile);
        } else if (hostMsg) {
          xSemaphoreTake(myselfMutex, portMAX_DELAY);
          myself.message = HOST_MSG;        // radio busy: put the host message back for the next beacon
          xSemaphoreGive(myselfMutex);
        } else if (me.message != 0) {
          setMsgFlag(me.message);           // radio busy: retry next beacon
        }
      }

      // Remove peers not heard from for PEER_TIMEOUT
      if (millis() - vanet.cluster.lastCheck >= PEER_CHECK_INTVL) {
        vanet.cluster.removeStale();

        // The message shown is from a node not in my cluster (it left, or it was only forwarded to me)
        // and older than PEER_TIMEOUT: clear it
        if ((vanet.peerMsg != 0) && (vanet.cluster.find(vanet.peerMsgId) < 0) && (millis() - vanet.peerMsgAt > PEER_TIMEOUT)) {
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

    // RGB: steady colour of the most important condition detected (held LIGHT_HOLD_MS after it was last seen),
    // Tx flashes on top of it. In message priority (accident red > low battery purple > bad terrain orange),
    // then no GPS fix (yellow). Without a host the battery is not checked, so its light goes off
    uint8_t background = gps.fix ? OFF : NO_FIX_COLOUR;
    if (detected(3)) {
      background = msgColours[3];     // bad terrain
    }
    if (detected(2)) {
      background = msgColours[2];     // low battery
    }
    if (detected(1)) {
      background = msgColours[1];     // accident
    }
    lights.setBackground(background);
    lights.update();

    // Log myself
    if (millis() - storage.lastNodeLog >= NODE_LOG_INTVL) {
      storage.write(formatMyself(copyMyself()), storage.myselfFile);
      storage.lastNodeLog = millis();
    }

    // Log sensing (with the message in my last beacon)
    if (millis() - storage.lastSenseLog >= SENSE_LOG_INTVL) {
      Data me = copyMyself();
      me.message = vanet.lastMsg;
      storage.write(formatSensing(me), storage.sensorFile);
      storage.lastSenseLog = millis();
    }

    vTaskDelay(1);                  // yield 1 ms so the idle task feeds the watchdog
  }
}

//*************************************************************************************************************************************************
// SETUP
//*************************************************************************************************************************************************

void setup() {

  // Serial Monitor (debug messages)
  Serial.begin(SERIAL_BAUD);

  // Protects errors (checked before the tasks start)
  errors.mutex = xSemaphoreCreateMutex();

  // Protects the message flags (checked before the tasks start)
  msgFlagsMutex = xSemaphoreCreateMutex();

  // Initialise RGB LED first, so every init result can blink
  lights.init();
  lights.toggle(BRIGHTWHITE);         // powered on: white until the first init result (Host) blinks green or red

  // Initialise host
  hostReady = host.init();

  // Initialise OLED (a failure is still reported in the error list)
  oled.init();

  oled.disp("TUKZIE V2X", 2, true, false);
  delay(1000);
  oled.refresh();
  oled.disp("BOOTING", 1, true, false);
  delay(500);
  oled.refresh();

  // Show existing init messages (each blinks green or red)
  showInit("Host", hostReady, false, 500);

  // Initialise other peripherals
  sdReady = storage.init();
  showInit("SD", sdReady, true, 500);
  gpsReady = gps.init();
  showInit("GPS", gpsReady, true, 500);
  imuReady = imu.init();
  showInit("IMU", imuReady, true, 500);
  vanetReady = vanet.init();
  showInit("LoRa", vanetReady, true, 500);

  // Node ID from the SD Card
  oled.refresh();
  oled.disp(storage.configNode(), 1, false, false);
  storage.nameLogs();             // log files named with my node ID (fresh every boot)
  delay(1000);

  // Summarise the failed inits in the error list
  if (errors.check) {
    reportError("Setup: errors occurred during boot\n");
  }

  // Boot finished
  oled.refresh();
  oled.disp("FINISHED BOOT", 1, true, false);
  delay(500);

  if (errors.check) {
    oled.disp(errors.msg, 1, false, false);
    lights.toggle(RED);         // red while the errors show
    delay(1000);
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

  xTaskCreatePinnedToCore(v2sLoop, "v2s", TASK_STACK, NULL, TASK_PRIORITY, &v2sTask, V2S_CORE);
  xTaskCreatePinnedToCore(v2vLoop, "v2v", TASK_STACK, NULL, TASK_PRIORITY, &v2vTask, V2V_CORE);
}

//*************************************************************************************************************************************************
// LOOP
//*************************************************************************************************************************************************

void loop() {

  // Peripheral debugging: set debugMode to true, then uncomment one debug call
  //debug(DBG_UI);
  //debug(DBG_IMU);
  //debug(DBG_GPS);
  //debug(DBG_SD_CARD);
  //debug(DBG_HOST);
  //debug(DBG_LORA_RX);
  //debug(DBG_LORA_TX);
  //debug(DBG_LORA);

  // Debug mode: keep looping
  if (debugMode) {
    return;
  }

  // All work runs in v2sLoop (core 0) and v2vLoop (core 1)
  vTaskDelete(NULL);    // end the Arduino loop task (frees core 1)
}
