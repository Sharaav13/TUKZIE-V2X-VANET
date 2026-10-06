//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - IMU
//*************************************************************************************************************************************************

#ifndef TVV_IMU_H
#define TVV_IMU_H

#include "system.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern LSM6 imu;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Setup
extern bool imuOK;
extern bool calibrated;

// Current values
extern float imuVals[2][3];  // {acc => {x, y, z}, gyro => {x, y, z}}

// Offsets
extern float offsets[2][3];  // {acc => {x, y, z}, gyro => {x, y, z}}

// Road awareness
extern bool jolt;
extern bool accident;  // 1
extern bool terrain;   // 2
extern bool pothole;   // 3
extern uint32_t joltStart;
extern uint32_t joltDuration;
extern int roadAwarenessNum;  // 0 = none, 1 = accident, 2 = terrain, 3 = pothole

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup IMU and calibrate
// Returns true if successful
bool imuInit(void);

// Calibrates IMU
void imuCalibrate(void);

// Gets a specific reading from the IMU on demand
// reading: ACC or GYRO | axis: X_AXIS, Y_AXIS or Z_AXIS
float imuGet(int reading, int axis);

// Gets all IMU values
void imuGetAll(void);

// Define threshold for jolts
// Checks terrain
// Fix according to speed !!
void roadSense(void);

#endif // TVV_IMU_H
