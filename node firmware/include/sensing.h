//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SENSING (GPS & PPS, IMU)
//*************************************************************************************************************************************************

#ifndef TVV_SENSING_H
#define TVV_SENSING_H

#include "system.h"

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

// GPS module
struct GPS {

  HardwareSerial uart{1};       // UART1
  TinyGPSPlus tinyGps;          // NMEA parser

  // Status
  bool detected = false;        // UART data received within GPS_TIMEOUT
  bool fix = false;             // valid and fresh location and speed
  float lastSpeed = 0.0f;       // m/s

  // Timestamps (ms)
  uint32_t lastSample = 0;      // previous GPS sample
  uint32_t lastByte = 0;        // previous GPS UART byte

  /*
    Setup GPS and PPS
    Returns true if the GPS UART driver was installed
  */
  bool init(void);

  /*
    Retreives GPS data and saves it to myself
    Fix quality is saved to myself.sats and myself.hdop
    New GPS speeds correct the Kalman filter
  */
  void get(void);

  /*
    Updates detected and fix
  */
  void status(void);

  /*
    GPS speed measurement variance (m/s)^2
  */
  float speedVariance(void);
};

// IMU module
struct IMU {

  LSM6 lsm6;                    // accelerometer and gyroscope

  // Status
  uint16_t fails = 0;           // consecutive failed reads

  // Calibration (average readings while still, in g and dps)
  float accBias[3] = {0.0f, 0.0f, 0.0f};    // includes gravity, so it also gives the resting attitude
  float gyroBias[3] = {0.0f, 0.0f, 0.0f};   // refined by the speed filter while stationary

  // Latest readings from getAll
  float acc[3] = {0.0f, 0.0f, 0.0f};        // g, gravity included (no bias removed)
  float gyro[3] = {0.0f, 0.0f, 0.0f};       // dps, gyroBias removed

  // Timestamps (ms)
  uint32_t lastSample = 0;      // previous IMU sample

  /*
    Setup IMU and calibrate
    Returns true if successful (false if the IMU is not found or calibration fails)
  */
  bool init(void);

  /*
    Calibrates IMU (keep the node still)
    Saves the average readings to accBias and gyroBias
    Returns false if too many reads failed (biases are left at 0)
  */
  bool calibrate(void);

  /*
    Gets a specific reading from the IMU on demand
    reading: ACC or GYRO | axis: X_AXIS, Y_AXIS or Z_AXIS
    calibrated: 0 => raw, 1 => bias removed (acc then reads ~0 g at rest)
    scaled: 0 => sensor counts, 1 => g or dps
  */
  float get(uint8_t reading, uint8_t axis, uint8_t calibrated, uint8_t scaled);

  /*
    Reads all IMU values in one transaction into acc (g, gravity included) and gyro (dps, gyroBias removed)
    Returns false if the I2C read failed
  */
  bool getAll(void);
};

/*
  KALMAN FILTER - 2-state Kalman filter for speed

  State (x) and covariance (P):
    x = [ speed (m/s), forward accelerometer bias (m/s^2) ]
    P = [ P00 P01 ]  uncertainty of speed, bias and how they relate
        [ P10 P11 ]

  Only does the maths. SpeedFilter decides when to predict and correct, and with what.
    predict():  move the state forward using the measured acceleration (uncertainty grows)
    correct():  blend in a measured speed using the Kalman gain (uncertainty shrinks)
*/
struct KalmanFilter {

  // State
  float speed = 0.0f;           // m/s
  float accelBias = 0.0f;       // m/s^2, slowly varying error of the forward accelerometer

  // Covariance (how uncertain the state is)
  float P00 = 1.0f;             // speed variance
  float P01 = 0.0f;             // speed/bias covariance
  float P10 = 0.0f;             // bias/speed covariance
  float P11 = 0.10f;            // bias variance

  /*
    Resets the filter to a known speed (m/s) with a fresh covariance
  */
  void reset(float initialSpeed);

  /*
    PREDICT: integrates the forward acceleration (m/s^2) over dt (s) into speed
    and grows the uncertainty, since a prediction is less certain than a measurement
  */
  void predict(float acceleration, float dt);

  /*
    CORRECT: blends a measured speed (m/s) into the prediction using the Kalman gain
    measurementVariance is how much the measurement is trusted (smaller = more trust)
    allowHardReset re-anchors the filter to the measurement if the prediction drifted badly
  */
  void correct(float measuredSpeed, float measurementVariance, bool allowHardReset);
};

/*
  SPEED FILTER - GPS + IMU speed fusion, using a KalmanFilter (kalman)

  Why: GPS speed is accurate over time but only arrives once per second and lags.
  The IMU reacts instantly but drifts if integrated on its own. Fusing both gives a
  smooth, responsive speed that stays anchored to GPS.

  Workflow:
    1. init()        Once, after imu.calibrate(): starting roll/pitch from gravity in imu.accBias.
    2. update()      Every loop: runs imuUpdate() at FUSION_PERIOD_US (100 Hz).
    3. imuUpdate()   Each IMU step:
                       a. imu.getAll(): imu.acc (with gravity) and imu.gyro (imu.gyroBias removed)
                       b. stationaryUpdate(): is the node confidently stopped?
                       c. track roll/pitch with an adaptive complementary filter (gyro + gravity)
                       d. remove gravity from the forward axis to get forward acceleration
                       e. PREDICT: kalman.predict() integrates that acceleration into speed
                       f. while stopped: refine imu.gyroBias and the accelerometer bias,
                          and apply a zero velocity update (ZUPT)
    4. gpsUpdate()   Each new GPS speed (called from gps.get()):
                       CORRECT: kalman.correct() pulls the predicted speed towards the GPS speed,
                       weighted by how uncertain each one is (the Kalman gain)
    5. getSpeed()    The fused speed in km/h, stored in myself.speed
*/
struct SpeedFilter {

  // Speed and accelerometer bias estimate (kalman.speed, kalman.accelBias)
  KalmanFilter kalman;

  // Fusion state
  bool initialized = false;     // true once the first GPS speed has seeded the filter
  bool stationary = false;      // true when GPS and IMU agree the node is stopped
  uint32_t stationaryStart = 0; // ms, 0 = not a stationary candidate
  float roll = 0.0f;            // rad
  float pitch = 0.0f;           // rad
  float fwdAccelFiltered = 0.0f;           // m/s^2, low pass filtered forward acceleration
  // The gyro bias is imu.gyroBias (set by imu.calibrate, refined here while stationary)

  // Timestamps
  uint32_t lastGpsCorrection = 0;   // ms, previous GPS correction
  uint32_t lastZupt = 0;            // ms, previous zero velocity update
  uint32_t lastFusion = 0;          // us, previous IMU prediction

  /*
    Initialises the speed fusion from the IMU calibration
    Call after imu.calibrate, with the node held still
  */
  void init(void);

  /*
    Corrects the filter with a new GPS speed (m/s)
    The first valid GPS speed seeds the filter
  */
  void gpsUpdate(float gpsSpeed);

  /*
    Detects if the node is stopped using GPS speed and IMU stillness
  */
  void stationaryUpdate(float accMagnitude, float gyroMagnitude);

  /*
    Updates attitude and predicts speed from the IMU over dt (s)
    Mounting convention: +X = forward, +Y = left, +Z = up
  */
  void imuUpdate(float dt);

  /*
    Runs the IMU prediction at FUSION_PERIOD_US
    Call every loop
  */
  void update(void);

  /*
    Returns the fused speed (km/h)
    Falls back to raw GPS speed until the filter is initialised
  */
  float getSpeed(void);
};

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern GPS gps;
extern IMU imu;
extern SpeedFilter speedFilter;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// PPS
extern volatile uint32_t countPps; // in seconds
extern volatile uint32_t gapMs;
extern volatile uint32_t lastPps;   // previous PPS pulse (set by ppsISR)

// Road awareness
extern bool vibration;
extern bool accident;  // 1
extern bool terrain;   // 2
extern bool pothole;   // 3
extern uint32_t vibrationStart;
extern uint32_t vibrationDuration;
extern uint8_t roadAwarenessNum;  // 0 = none, 1 = accident, 2 = terrain, 3 = pothole

//*************************************************************************************************************************************************
// INTERRUPTS
//*************************************************************************************************************************************************

/*
  Records a second indicated by PPS
*/
void IRAM_ATTR ppsISR();

//*************************************************************************************************************************************************
// FUNCTIONS - ROAD AWARENESS
//*************************************************************************************************************************************************

/*
  Define threshold for vibrations
  Checks terrain
  Fix according to speed !!
*/
void senseTerrain(void);

#endif // TVV_SENSING_H
