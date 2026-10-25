//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SENSING (GPS, IMU)
//*************************************************************************************************************************************************

#ifndef TVV_SENSING_H
#define TVV_SENSING_H

#include "system.h"

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

/*
  UTC time base from the GPS PPS, for packet latency (both nodes share it, unlike their millis())

  ppsISR() records the local µs (esp_timer) of each PPS edge, GPS::get() pairs it with the UTC second of the NMEA
  time that follows it (an anchor), and toUtc() converts any local µs to µs since UTC midnight:
    UTC(t) = anchorUtc + (t - anchorUs) / rate
  rate (local µs per UTC µs) is measured between anchors, so the ESP32 clock error is corrected
  Without PPS (no fix) the last anchor and rate keep running (holdover)
*/
struct UtcClock {

  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;   // ppsUs (ISR) and the anchor (core 0) are read on core 1

  volatile int64_t ppsUs = 0;   // local µs of the latest PPS edge (0 = none yet)
  int64_t anchorUs = 0;         // local µs of the PPS edge paired with a UTC second (0 = not synced)
  int64_t anchorUtc = 0;        // that UTC second, µs since UTC midnight
  double rate = 1.0;            // local µs per UTC µs

  /*
    Pairs the latest PPS edge with the UTC second of a newly parsed NMEA time
    utcSecond: seconds since UTC midnight
  */
  void pair(int32_t utcSecond);

  /*
    Converts a local µs (esp_timer_get_time()) to µs since UTC midnight
    sync: set to 0 = not synced (returns 0), 1 = PPS locked, 2 = holdover
  */
  int64_t toUtc(int64_t localUs, uint8_t &sync);
};

// GPS module
struct GPS {

  HardwareSerial uart{1};       // UART1
  TinyGPSPlus tinyGps;          // NMEA parser
  int32_t lastUtcSecond = -1;   // UTC second of the last NMEA time paired with PPS

  // Status
  bool detected = false;        // UART data received within GPS_TIMEOUT
  bool fix = false;             // valid and fresh location and speed
  float lastSpeed = 0.0f;       // m/s

  // Timestamps (ms)
  uint32_t lastSample = 0;      // previous GPS sample
  uint32_t lastByte = 0;        // previous GPS UART byte

  /*
    Setup GPS
    Returns true if the GPS UART driver was installed
  */
  bool init(void);

  /*
    Reads the GPS into myself (incl. sats and hdop)
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

  /*
    GPS data in myself as text (for testing)
  */
  size_t toStr(char *out, size_t cap);
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
    Setup and calibrate the IMU
    Returns true if successful
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
    Reads acc (g, with gravity) and gyro (dps, bias removed) in one I2C read
    Returns false if the read failed
  */
  bool getAll(void);

  /*
    IMU readings as text (for testing)
  */
  size_t toStr(char *out, size_t cap);
};

/*
  KALMAN FILTER - 2-state speed filter
    x = [ speed (m/s), forward accelerometer bias (m/s^2) ]
    P = [ P00 P01 ; P10 P11 ]  uncertainty of x
  Only the maths: SpeedFilter decides when to predict() and correct()
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
    PREDICT: integrates forward acceleration (m/s^2) over dt (s) into speed, uncertainty grows
  */
  void predict(float acceleration, float dt);

  /*
    CORRECT: blends a measured speed (m/s) in using the Kalman gain
    measurementVariance: smaller = more trust
    allowHardReset: jump to the measurement if the prediction drifted badly
  */
  void correct(float measuredSpeed, float measurementVariance, bool allowHardReset);
};

/*
  SPEED FILTER - GPS + IMU speed fusion (KalmanFilter kalman)
  GPS speed is accurate but slow (1 Hz) and lags, the IMU is instant but drifts: fused, speed is smooth and anchored to GPS

  1. init()        once after imu.calibrate(): starting roll/pitch from gravity
  2. update()      every loop: runs imuUpdate() at 100 Hz
  3. imuUpdate()   read IMU, detect stops, track roll/pitch, remove gravity, PREDICT,
                   and while stopped refine the biases and apply a zero velocity update (ZUPT)
  4. gpsUpdate()   each GPS speed (from gps.get()): CORRECT
  5. getSpeed()    fused speed in kmph (myself.speed)
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
  // Gyro bias: imu.gyroBias (refined here while stationary)

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
    Returns the fused speed in kmph (never negative)
    Uses the GPS speed until the filter has started
  */
  float getSpeed(void);
};

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern UtcClock utcClock;
extern GPS gps;
extern IMU imu;
extern SpeedFilter speedFilter;

//*************************************************************************************************************************************************
// INTERRUPTS
//*************************************************************************************************************************************************

/*
  GPS PPS: records the local µs of the edge in utcClock.ppsUs
*/
void IRAM_ATTR ppsISR();

#endif // TVV_SENSING_H
