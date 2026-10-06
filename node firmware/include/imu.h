//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - IMU
//*************************************************************************************************************************************************

#ifndef TVV_IMU_H
#define TVV_IMU_H

#include "System.h"

//*************************************************************************************************************************************************
// MACROS
//*************************************************************************************************************************************************

// Reading indices (first index of imuVals/offsets, reading argument of imuGet)
#define ACC   0
#define GYRO  1

// Axis indices (second index of imuVals/offsets, axis argument of imuGet)
#define X_AXIS  0
#define Y_AXIS  1
#define Z_AXIS  2

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
extern bool accident;
extern bool terrain;
extern bool pothole;
extern uint32_t joltStart;
extern uint32_t joltDuration;

// Debug
extern uint32_t lastImu;     // Timestamp of previous IMU sample

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup IMU and calibrate
String imuInit(void);

// Calibrates IMU
void imuCalibrate(void);

// Gets a specific reading from the IMU
// reading: ACC or GYRO | axis: X_AXIS, Y_AXIS or Z_AXIS
float imuGet(int reading, int axis);

// Gets all IMU values
void imuGetAll(void);

// Define threshold for jolts
// Checks terrain
void roadSense(void);

// Converts currently stored IMU data into a single string
// Used for IMU module testing
size_t imuStr(char *out, size_t cap);

// Displays all IMU data periodically
void imuDebug(void);

#endif // TVV_IMU_H
