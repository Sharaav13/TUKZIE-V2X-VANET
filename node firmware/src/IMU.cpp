//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - IMU
//*************************************************************************************************************************************************

#include "IMU.h"
#include "Display.h"   // imuDebug() calls disp()

#define ACC_MG_PER_LSB    0.061f 
#define GYRO_MDPS_PER_LSB 4.375f // Check datasheet 
#define CALIBRATION_SAMPLES 1000.0 // Calibrating IMU 
#define JOLT_G_THRESHOLD 1.8 
#define JOLT_INTERVAL_THRESHOLD 500

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

LSM6 imu;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Setup
bool imuOK = false;
bool calibrated = false;

// Current values
float imuVals[2][3];// {acc => {x, y, z}, gyro => {x, y, z}}

// Offsets
float offsets [2][3]; // {acc => {x, y, z}, gyro => {x, y, z}}

// Road awareness
bool jolt = false;
bool accident = false;
bool terrain = false;
bool pothole = false;
uint32_t joltStart;
uint32_t joltDuration;

// Debug
uint32_t lastImu;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************


// Setup IMU
String imuInit(void) {

  bool status2 = Wire1.begin(IMU_I2C_SDA, IMU_I2C_SCL);
  imu.setBus(&Wire1); 
  
  if (status2 && imu.init()) {
    imu.enableDefault();
    imuCalibrate();
    imuOK= true;
    return "IMU init pass.\n";
  } else {
    error = true;
    return "IMU init failed.\n";
  }
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

// Gets a specific reading from the IMU
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

// Define threshold for jolts
// Checks terrain
void senseRoad(void) {

  pothole = false;
  terrain = false;

  float acc_x = abs(imuGet(ACC, X_AXIS));
  float acc_y = abs(imuGet(ACC, Y_AXIS));
  float acc_z = imuGet(ACC, Z_AXIS);

  // Sense jolts
  if ((acc_z >= JOLT_G_THRESHOLD) && (jolt == false)) {
    joltStart = millis();
    jolt = true;
  }
  
  // Determine rough/bumpy terrain or potholes
  if (jolt) {
    joltDuration = millis() - joltStart;
    if (joltDuration >= JOLT_INTERVAL_THRESHOLD) {
      terrain = true;
    } else if ((joltDuration < JOLT_INTERVAL_THRESHOLD) && (acc_z < JOLT_G_THRESHOLD)) {
      pothole = true;
    }
  }

  // Checks if there was an accident or toppled over
  if ((acc_x >= 1) || (acc_y >= 1) || (acc_z >= 1)) {
    accident = true;
  }
}

// Converts currently stored IMU data into a single string
// Used for IMU module testing
size_t imuStr(char *out, size_t cap) {

  return  snprintf(out, cap,
        "A: %.2f %.2f %.2f\n"
        "G: %.2f %.2f %.2f",
        imuVals[ACC][X_AXIS], imuVals[ACC][Y_AXIS], imuVals[ACC][Z_AXIS],
        imuVals[GYRO][X_AXIS], imuVals[GYRO][Y_AXIS], imuVals[GYRO][Z_AXIS]);
}

// Displays all IMU data periodically
void imuDebug(void) {
  if ((millis() - lastImu) >= IMU_SAMPLING) {
    imuGetAll();
    char imu_str[192];
    imuStr(imu_str, sizeof(imu_str));
    disp(imu_str, 1, false, false);
    lastImu = millis();
  }
}
