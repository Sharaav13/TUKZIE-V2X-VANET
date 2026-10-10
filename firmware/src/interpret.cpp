//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - INTERPRET (CODED MESSAGES & STATUS, ROAD AWARENESS)
//*************************************************************************************************************************************************

#include "system.h"
#include "sensing.h"
#include "interpret.h"

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Coded messages (index = code, lower = higher priority)
const char *const codedMessages[] = {
    "",                                                             // 0
    "Met with accident",                                            // 1
    "Low on battery",                                               // 2
    "Rough terrain is present",                                     // 3
    "Potholes are present",                                         // 4
};
static_assert(sizeof(codedMessages) / sizeof(codedMessages[0]) == MSG_COUNT, "codedMessages must have MSG_COUNT entries");

// RGB colour flashed for each message on Tx (index = code)
const uint8_t msgColours[] = {
    BLUE,       // 0 none
    RED,        // 1 accident
    PURPLE,     // 2 low battery
    ORANGE,     // 3 rough terrain
    YELLOW      // 4 potholes
};
static_assert(sizeof(msgColours) / sizeof(msgColours[0]) == MSG_COUNT, "msgColours must have MSG_COUNT entries");

// Coded status (index = code, set by the host)
const char *const codedStatus[] = {
    "Parked",       // 0
    "Charging",     // 1
    "Delivering",   // 2
    "Taxi"          // 3
};
static_assert(sizeof(codedStatus) / sizeof(codedStatus[0]) == STATUS_COUNT, "codedStatus must have STATUS_COUNT entries");

// Pending messages (index = code), only changed under msgFlagsMutex
static bool msgFlags[MSG_COUNT] = {false};

// Protects msgFlags (set on both cores)
SemaphoreHandle_t msgFlagsMutex = NULL;

// Accident
static bool force = false;
static uint32_t forceStart;
static bool spinning = false;
static uint32_t spinStart;

// Terrain
static bool vibration = false;     // a vibration window is open
static bool aboveVibr = false;     // acc_z is currently above the vibration threshold
static byte vibrationCount = 0;
static uint32_t vibrationStart;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Road Awareness
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Flags an accident from prolonged large force or spinning
  Uses the latest imu.getAll() readings (no extra I2C reads)
*/
void checkAccident(void) {

  // On its side or upside down (g, raw: gravity included, z + = up), timed from when it starts
  if ((abs(imu.acc[X_AXIS]) >= SIDE_G) ||
      (abs(imu.acc[Y_AXIS]) >= SIDE_G) ||
      (imu.acc[Z_AXIS] <= UPSIDE_DOWN_G)) {
    if (!force) {
      forceStart = millis();
      force = true;   // Experienced a large force
    }
  } else {
    force = false;
  }

  // Determine if prolonged force is an accident
  if (force) {
    uint32_t forceDuration = millis() - forceStart;
    if (forceDuration >= FORCE_DURATION) {
      setMsgFlag(1);   // Met with accident
    }
  }

  // Spinning (dps), timed from when it starts
  if ((abs(imu.gyro[X_AXIS]) >= 90) || (abs(imu.gyro[Y_AXIS]) >= 90) || (abs(imu.gyro[Z_AXIS]) >= 90)) {
    if (!spinning) {
      spinStart = millis();
      spinning = true;   // Vehicle is spinning
    }
  } else {
    spinning = false;
  }

  // Determine if prolonged spinning is an accident
  if (spinning) {
    uint32_t spinDuration = millis() - spinStart;
    if (spinDuration >= SPIN_DURATION) {
      setMsgFlag(1);   // Met with accident
    }
  }
}

/*
  Flags rough terrain or potholes from vertical vibrations
  VIBR_CNT_THRESHOLD or more in the window => rough terrain, fewer => pothole
*/
void checkTerrain(void) {

  // Vertical acceleration (g, gravity removed, + = up)
  float acc_z = imu.acc[Z_AXIS] - imu.accBias[Z_AXIS];

  // Count each crossing of the threshold once
  if (!aboveVibr && (acc_z >= VIBR_G_THRESHOLD)) {
    aboveVibr = true;
    if (!vibration) {               // first vibration opens the window
      vibrationStart = millis();
      vibration = true;
    }
    vibrationCount++;
  } else if (aboveVibr && (acc_z < VIBR_G_RELEASE)) {
    aboveVibr = false;              // below again: ready for the next one
  }

  // Determine rough/bumpy terrain or potholes
  if (vibration) {

    uint32_t vibrationDuration = millis() - vibrationStart;

    if (vibrationDuration >= VIBR_INTERVAL_THRESHOLD) {

      if (vibrationCount >= VIBR_CNT_THRESHOLD) {
        setMsgFlag(3);   // Rough terrain is present
      } else {
        setMsgFlag(4);   // Potholes are present
      }

      vibration = false;
      vibrationCount = 0;
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Battery
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Flags a low battery (at or below LOW_BATTERY_THRESHOLD)
*/
void checkBattery(void) {
  if (myself.battery <= LOW_BATTERY_THRESHOLD) {
    setMsgFlag(2);   // Low on battery
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Messages
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Marks a message as pending (safe from both cores)
  code: index of codedMessages, ignored if out of range
*/
void setMsgFlag(uint8_t code) {
  if (code >= MSG_COUNT) {
    return;
  }
  xSemaphoreTake(msgFlagsMutex, portMAX_DELAY);
  msgFlags[code] = true;
  xSemaphoreGive(msgFlagsMutex);
}

/*
  Returns the highest priority pending message (lowest code, 0 = none) and clears its flag
*/
uint8_t sortMsg(void) {
  uint8_t priorityMsg = MSG_COUNT;
  xSemaphoreTake(msgFlagsMutex, portMAX_DELAY);
  for (uint8_t i = 0; i < MSG_COUNT; i++) {
    if ((msgFlags[i] == true) && (i < priorityMsg)) {
      priorityMsg = i;
      msgFlags[i] = false; // Clear the flag
    }
  }
  xSemaphoreGive(msgFlagsMutex);
  return (priorityMsg == MSG_COUNT) ? 0 : priorityMsg;
}
