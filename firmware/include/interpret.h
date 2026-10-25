//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - INTERPRET (CODED MESSAGES & STATUS, ROAD AWARENESS)
//*************************************************************************************************************************************************

#ifndef TVV_INTERPRET_H
#define TVV_INTERPRET_H

#include "system.h"

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Coded messages and status (index = code)
extern const char *const codedMessages[];   // MSG_COUNT entries
extern const char *const codedStatus[];    // STATUS_COUNT entries
extern const uint8_t msgColours[];         // RGB colour of each message (MSG_COUNT entries): Tx flashes and steady condition lights

// Protects the message flags (lock order: myselfMutex first)
extern SemaphoreHandle_t msgFlagsMutex;


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
void checkAccident(void);

/*
  Flags bad terrain (a bump, pothole or rough surface): the vertical acceleration, gravity removed, reaches TERRAIN_G
  either way, only while moving at ROAD_MIN_SPEED or faster
*/
void checkTerrain(void);

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Battery
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Flags a low battery (at or below LOW_BATTERY_THRESHOLD)
*/
void checkBattery(void);

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Messages
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Returns true while a condition is detected, and for LIGHT_HOLD_MS after it was last detected (for the RGB light)
  code: message code of the condition (1 accident, 2 low battery, 3 bad terrain)
*/
bool detected(uint8_t code);

/*
  Marks a message as pending (safe from both cores)
  code: index of codedMessages, ignored if out of range
*/
void setMsgFlag(uint8_t code);

/*
  Returns the highest priority pending message (lowest code, 0 = none) and clears its flag
*/
uint8_t sortMsg(void);

#endif // TVV_INTERPRET_H
