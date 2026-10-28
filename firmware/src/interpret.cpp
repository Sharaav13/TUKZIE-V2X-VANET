//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - INTERPRET (CODED MESSAGES & STATUS, ROAD AWARENESS)
//*************************************************************************************************************************************************
/*
  Version:  1.0
  Date:     27/10/2026
  Authors:  Sharaav Dhebideen
*/
//*************************************************************************************************************************************************

#include "system.h"
#include "sensing.h"
#include "interpret.h"

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Coded messages (index = code, lower = higher priority, except HOST_MSG which is always sent first)
const char *const codedMessages[] = {
    "",                                                             // 0
    "Met with accident",                                            // 1
    "Low battery",                                                  // 2
    "Bad terrain",                                                  // 3
    "Custom host message",                                          // 4
};
static_assert(sizeof(codedMessages) / sizeof(codedMessages[0]) == MSG_COUNT, "codedMessages must have MSG_COUNT entries");

// RGB colour of each message (index = code): Tx flash green (0) or blue (host), steady red/purple/orange while an accident/low battery/bad terrain is detected
const uint8_t msgColours[] = {
    GREEN,      // 0 none
    RED,        // 1 accident
    PURPLE,     // 2 low battery
    ORANGE,     // 3 bad terrain
    BLUE        // 4 host
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

// When each condition was last detected (ms, index = message code, 0 = never): set on core 0, read on core 1
static volatile uint32_t detectedAt[MSG_COUNT] = {0};

// Accident
static bool toppled = false;        // on its side or upside down
static uint32_t toppleStart = 0;    // ms when it toppled

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Road Awareness
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Flags an accident from a sharp impact (ACCIDENT_G or more), or from lying on its side or upside down
  for TOPPLE_DURATION
  Uses the latest imu.getAll() readings (no extra I2C reads)
*/
void checkAccident(void) {

  // Impact: a sharp acceleration of ACCIDENT_G or more in any direction (g, raw: gravity included)
  float magnitude = sqrtf(imu.acc[X_AXIS] * imu.acc[X_AXIS] +
                          imu.acc[Y_AXIS] * imu.acc[Y_AXIS] +
                          imu.acc[Z_AXIS] * imu.acc[Z_AXIS]);
  bool impact = (magnitude >= ACCIDENT_G);

  // On its side or upside down (g, raw: gravity included, +Z = up), timed from when it starts
  if ((abs(imu.acc[X_AXIS]) >= SIDE_G) ||
      (abs(imu.acc[Y_AXIS]) >= SIDE_G) ||
      (imu.acc[Z_AXIS] <= UPSIDE_DOWN_G)) {
    if (!toppled) {
      toppleStart = millis();
      toppled = true;
    }
  } else {
    toppled = false;
  }

  // Lying toppled for long enough is an accident
  bool toppledTooLong = toppled && ((millis() - toppleStart) >= TOPPLE_DURATION);

  if (impact || toppledTooLong) {
    detectedAt[1] = millis();
    setMsgFlag(1);   // Met with accident
  }
}

/*
  Flags bad terrain (a bump, pothole or rough surface): the vertical acceleration, gravity removed, reaches TERRAIN_G
  either way, only while moving at ROAD_MIN_SPEED or faster (no false triggers when parked: loading, doors, people
  getting in)
*/
void checkTerrain(void) {

  // Vertical acceleration (g, gravity removed: 0 when level and still, + = up)
  float acc_z = imu.acc[Z_AXIS] - imu.accBias[Z_AXIS];

  if ((fabsf(acc_z) >= TERRAIN_G) && (myself.speed >= ROAD_MIN_SPEED)) {
    detectedAt[3] = millis();
    setMsgFlag(3);   // Bad terrain
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
    detectedAt[2] = millis();
    setMsgFlag(2);   // Low battery
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Messages
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Returns true while a condition is detected, and for LIGHT_HOLD_MS after it was last detected (for the RGB light)
  code: message code of the condition (1 accident, 2 low battery, 3 bad terrain)
*/
bool detected(uint8_t code) {
  if (code >= MSG_COUNT) {
    return false;
  }
  uint32_t at = detectedAt[code];     // read before millis(), so a detection on core 0 in between cannot wrap
  return (at != 0) && (millis() - at < LIGHT_HOLD_MS);
}

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
      msgFlags[i] = false;    // clear the flag
    }
  }
  xSemaphoreGive(msgFlagsMutex);
  return (priorityMsg == MSG_COUNT) ? 0 : priorityMsg;
}
