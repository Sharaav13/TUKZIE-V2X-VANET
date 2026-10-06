//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - IMU
//*************************************************************************************************************************************************

#include "system.h"
#include "imu.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

LSM6 imu;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

bool calibrated = false;

// Current values
float imuVals[2][3];// {acc => {x, y, z}, gyro => {x, y, z}}

// Offsets
float offsets [2][3]; // {acc => {x, y, z}, gyro => {x, y, z}}

// Road awareness
bool vibration = false;
bool accident = false;  // 1
bool terrain = false;   // 2
bool pothole = false;   // 3
uint32_t vibrationStart;
uint32_t vibrationDuration;
int roadAwarenessNum = 0;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup IMU
// Returns true if successful
bool imuInit(void) {

  if (!Wire1.begin(IMU_I2C_SDA, IMU_I2C_SCL)) {
    error = true;
    errMsg += "IMU: I2C bus failed to start\n";
    return false;
  }
  imu.setBus(&Wire1);

  if (!imu.init()) {
    error = true;
    errMsg += "IMU: LSM6 not detected\n";
    return false;
  }

  imu.enableDefault();
  imuCalibrate();
  imuOK= true;
  return true;
}

// Calibrates IMU
void imuCalibrate(void) {
  
    int counter = 0;
    float sum[2][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

    while (counter != CALIBRATION_SAMPLES) {
        
        // Acceleration
        sum[ACC][X_AXIS] += imuGet(ACC, X_AXIS);
        sum[ACC][Y_AXIS] += imuGet(ACC, Y_AXIS);
        sum[ACC][Z_AXIS] += imuGet(ACC, Z_AXIS);

        // Gyroscope
        sum[GYRO][X_AXIS] += imuGet(GYRO, X_AXIS);
        sum[GYRO][Y_AXIS] += imuGet(GYRO, Y_AXIS);
        sum[GYRO][Z_AXIS] += imuGet(GYRO, Z_AXIS);

        counter ++;
    }

    // Acceleration
    offsets[ACC][X_AXIS] = sum[ACC][X_AXIS] / CALIBRATION_SAMPLES;
    offsets[ACC][Y_AXIS] = sum[ACC][Y_AXIS] / CALIBRATION_SAMPLES;
    offsets[ACC][Z_AXIS] = sum[ACC][Z_AXIS] / CALIBRATION_SAMPLES;

    // Gyroscope
    offsets[GYRO][X_AXIS] = sum[GYRO][X_AXIS] / CALIBRATION_SAMPLES;
    offsets[GYRO][Y_AXIS] = sum[GYRO][Y_AXIS] / CALIBRATION_SAMPLES;
    offsets[GYRO][Z_AXIS] = sum[GYRO][Z_AXIS] / CALIBRATION_SAMPLES;

    calibrated = true;
}

// Gets a specific reading from the IMU on demand
float imuGet(int reading, int axis) {
  
  float result;
  imu.read();

    if (reading == ACC) { // Acceleration
    switch (axis) {
      case X_AXIS:
        result = imu.a.x;
        break;
      case Y_AXIS:
        result = imu.a.y;
        break;
      case Z_AXIS:
        result = imu.a.z;
        break;
    }
  } else if (reading == GYRO) { // Gyroscope
      switch (axis) {
      case X_AXIS:
        result = imu.g.x;
        break;
      case Y_AXIS:
        result = imu.g.y;
        break;
      case Z_AXIS:
        result = imu.g.z;
        break;
    }
  }

  // Apply conversions
  (reading == ACC) ? (result = result * ACC_MG_PER_LSB / 1000.0): (result = result * GYRO_MDPS_PER_LSB / 1000.0);

  // Use offset if calibrated
  if (calibrated) {
    return result - offsets[reading][axis]; 
  } else {
    return result;
  }
}

// Gets all IMU values
void imuGetAll (void) {
  
  // Acceleration
  imuVals[ACC][X_AXIS] = imuGet(ACC, X_AXIS);
  imuVals[ACC][Y_AXIS] = imuGet(ACC, Y_AXIS);
  imuVals[ACC][Z_AXIS] = imuGet(ACC, Z_AXIS);

  // Gyroscope
  imuVals[GYRO][X_AXIS] = imuGet(GYRO, X_AXIS);
  imuVals[GYRO][Y_AXIS] = imuGet(GYRO, Y_AXIS);
  imuVals[GYRO][Z_AXIS] = imuGet(GYRO, Z_AXIS);
}

// Define threshold for VIBRs
// Checks terrain
// Fix according to speed !!
void roadSense(void) {


  pothole = false;
  terrain = false;

  float acc_x = abs(imuGet(ACC, X_AXIS));
  float acc_y = abs(imuGet(ACC, Y_AXIS));
  float acc_z = imuGet(ACC, Z_AXIS);

  // Sense VIBRs
  if ((acc_z >= VIBR_G_THRESHOLD) && (vibration == false)) {
    vibrationStart = millis();
    vibration = true;
  }
  
  // Determine rough/bumpy terrain or potholes
  if (vibration) {
    vibrationDuration = millis() - vibrationStart;
    if (vibrationDuration >= VIBR_INTERVAL_THRESHOLD) {
      terrain = true;
      roadAwarenessNum = 2;
      myself.message = 5;   // Rough terrain is present
    } else if ((vibrationDuration < VIBR_INTERVAL_THRESHOLD) && (acc_z < VIBR_G_THRESHOLD)) {
      pothole = true;
      roadAwarenessNum = 3;
      myself.message = 6;   // Potholes are present
    }
  }

  // Checks if there was an accident or toppled over
  if ((acc_x >= 1) || (acc_y >= 1) || (acc_z >= 1)) {
    accident = true;
    roadAwarenessNum = 1;
    myself.message = 3;     // Met with accident
  }

}
