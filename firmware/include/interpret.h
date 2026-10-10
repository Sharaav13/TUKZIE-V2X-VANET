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
extern const uint8_t msgColours[];         // RGB colour flashed for each message on Tx (MSG_COUNT entries)

// Protects the message flags (lock order: myselfMutex first)
extern SemaphoreHandle_t msgFlagsMutex;

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
void checkAccident(void);

/*
  Flags rough terrain or potholes from vertical vibrations
  VIBR_CNT_THRESHOLD or more in the window => rough terrain, fewer => pothole
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
  Marks a message as pending (safe from both cores)
  code: index of codedMessages, ignored if out of range
*/
void setMsgFlag(uint8_t code);

/*
  Returns the highest priority pending message (lowest code, 0 = none) and clears its flag
*/
uint8_t sortMsg(void);

#endif // TVV_INTERPRET_H
