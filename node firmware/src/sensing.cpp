//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SENSING (GPS & PPS, IMU)
//*************************************************************************************************************************************************

#include "system.h"
#include "sensing.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

GPS gps;
IMU imu;
SpeedFilter speedFilter;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// PPS
volatile uint32_t countPps =  0; // seconds
volatile uint32_t gapMs  =    0;
volatile uint32_t lastPps = 0;      // previous PPS pulse (set by ppsISR)

// Road awareness
bool vibration = false;
bool accident = false;  // 1
bool terrain = false;   // 2
bool pothole = false;   // 3
uint32_t vibrationStart;
uint32_t vibrationDuration;
uint8_t roadAwarenessNum = 0;

//*************************************************************************************************************************************************
// INTERRUPTS
//*************************************************************************************************************************************************

/*
  Records a second indicated by PPS
*/
void IRAM_ATTR ppsISR() {
  uint32_t now = millis();
  gapMs  = now - lastPps;
  lastPps = now;
  countPps++;
}

//*************************************************************************************************************************************************
// FUNCTIONS - GPS & PPS
//*************************************************************************************************************************************************

/*
  Setup GPS and PPS
  Returns true if the GPS UART driver was installed
*/
bool GPS::init(void) {
  uart.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  pinMode(PPS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS), ppsISR, RISING);

  if (!uart) {
    reportError("GPS: UART driver failed to install\n");
    return false;
  }
  return true;
}

/*
  Retreives GPS data and saves it to myself
  Fix quality is saved to myself.sats and myself.hdop
  New GPS speeds correct the Kalman filter
*/
void GPS::get(void) {

  while (uart.available()) {
    tinyGps.encode(uart.read());
    lastByte = millis();
  }

  status();

  // Date and time (converted from UTC to SAST)
  if (tinyGps.time.isValid() && tinyGps.date.isValid()) {
    int day   = tinyGps.date.day();
    int month = tinyGps.date.month();
    int year  = tinyGps.date.year();
    int hour  = tinyGps.time.hour() + SAST_OFFSET;

    // Roll over into the next day
    if (hour >= 24) {
      hour -= 24;
      day++;

      int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      bool leapYear = ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
      if (leapYear) {
        daysInMonth[1] = 29;
      }

      // Roll over into the next month
      if (day > daysInMonth[month - 1]) {
        day = 1;
        month++;

        // Roll over into the next year
        if (month > 12) {
          month = 1;
          year++;
        }
      }
    }

    myself.date = (uint32_t)day * 10000 + month * 100 + (year % 100);   // DDMMYY (SAST)
    myself.time = (uint32_t)hour * 1000000 + tinyGps.time.minute() * 10000
                + tinyGps.time.second() * 100 + tinyGps.time.centisecond();     // HHMMSSCC (SAST)
  }

  // Location
  if (tinyGps.location.isUpdated()) {
    myself.lat = tinyGps.location.lat();
    myself.lon = tinyGps.location.lng();
  }

  // Altitude
  if (tinyGps.altitude.isValid()) {
    myself.alt = tinyGps.altitude.meters();
  }

  // Speed
  if (tinyGps.speed.isUpdated()) {
    myself.speed  = tinyGps.speed.kmph();
    speedFilter.gpsUpdate(tinyGps.speed.mps());   // Kalman correct step
  }

  // Course
  if (tinyGps.course.isValid()) {
    myself.course = tinyGps.course.deg();
  }

  // Fix quality
  if (tinyGps.satellites.isValid() && tinyGps.hdop.isValid()) {
    myself.sats = tinyGps.satellites.value();
    myself.hdop = tinyGps.hdop.hdop();
  }
}

/*
  Updates detected and fix
*/
void GPS::status(void) {

  detected = (lastByte != 0) && (millis() - lastByte < GPS_TIMEOUT);

  fix = detected
     && tinyGps.location.isValid()
     && (tinyGps.location.age() < GPS_MAX_AGE)
     && tinyGps.speed.isValid();
}

/*
  GPS speed measurement variance (m/s)^2
*/
float GPS::speedVariance(void) {

  float sigma = 0.25f;

  // Slightly more conservative at walking speed
  if (lastSpeed < 2.0f) { // Evaluate for speeds upto 70 kmph
    sigma = 0.32f;
  }

  // Less trust with few satellites
  if (tinyGps.satellites.isValid() && (tinyGps.satellites.value() < 5)) {
    sigma *= 1.4f;
  }

  return sigma * sigma;
}

//*************************************************************************************************************************************************
// FUNCTIONS - IMU
//*************************************************************************************************************************************************

/*
  Setup IMU
  Returns true if successful
*/
bool IMU::init(void) {

  if (!Wire1.begin(IMU_I2C_SDA, IMU_I2C_SCL)) {
    reportError("IMU: I2C bus failed to start\n");
    return false;
  }
  lsm6.setBus(&Wire1);

  if (!lsm6.init()) {
    reportError("IMU: LSM6 not detected\n");
    return false;
  }

  lsm6.enableDefault();

  if (!calibrate()) {
    return false;       // calibrate has already reported the error
  }

  speedFilter.init();   // starting attitude from the calibration
  return true;
}

/*
  Calibrates IMU (keep the node still)
  Saves the average readings to accBias and gyroBias
  Returns false if too many reads failed (biases are left at 0)
*/
bool IMU::calibrate(void) {

  uint16_t counter = 0;
  float accSum[3] = {0.0f, 0.0f, 0.0f};
  float gyroSum[3] = {0.0f, 0.0f, 0.0f};

  // Start from zero bias, so getAll returns the plain readings being averaged
  for (uint8_t i = 0; i < 3; i++) {
    accBias[i] = 0.0f;
    gyroBias[i] = 0.0f;
  }

  uint16_t attempts = 0;

  while (counter != CALIBRATION_SAMPLES) {

    // Give up rather than hang the boot if the IMU keeps failing
    if (++attempts > 2 * CALIBRATION_SAMPLES) {
      reportError("IMU: calibration failed\n");
      return false;
    }

    if (!getAll()) {
      continue;   // skip failed reads
    }

    for (uint8_t i = 0; i < 3; i++) {
      accSum[i] += acc[i];
      gyroSum[i] += gyro[i];
    }

    counter ++;
  }

  for (uint8_t i = 0; i < 3; i++) {
    accBias[i] = accSum[i] / CALIBRATION_SAMPLES;    // g, includes gravity
    gyroBias[i] = gyroSum[i] / CALIBRATION_SAMPLES;  // dps
  }

  return true;
}

/*
  Gets a specific reading from the IMU on demand
  reading: ACC or GYRO | axis: X_AXIS, Y_AXIS or Z_AXIS
  calibrated: 0 => raw, 1 => bias removed (acc then reads ~0 g at rest)
  scaled: 0 => sensor counts, 1 => g or dps
*/
float IMU::get(uint8_t reading, uint8_t axis, uint8_t calibrated, uint8_t scaled) {

  float result = 0.0f;
  lsm6.read();

    if (reading == ACC) { // Acceleration
    switch (axis) {
      case X_AXIS:
        result = lsm6.a.x;
        break;
      case Y_AXIS:
        result = lsm6.a.y;
        break;
      case Z_AXIS:
        result = lsm6.a.z;
        break;
    }
  } else if (reading == GYRO) { // Gyroscope
      switch (axis) {
      case X_AXIS:
        result = lsm6.g.x;
        break;
      case Y_AXIS:
        result = lsm6.g.y;
        break;
      case Z_AXIS:
        result = lsm6.g.z;
        break;
    }
  }

  // Counts to g or dps (the biases are stored in g and dps)
  float factor = (reading == ACC) ? (ACC_MG_PER_LSB / 1000.0f) : (GYRO_MDPS_PER_LSB / 1000.0f);
  float bias = (reading == ACC) ? accBias[axis] : gyroBias[axis];

  if (scaled) {
    result *= factor;
    if (calibrated) {
      result -= bias;
    }
  } else if (calibrated) {
    result -= bias / factor;    // bias converted back to counts
  }

  return result;
}

/*
  Reads all IMU values in one transaction into acc (g, gravity included) and gyro (dps, gyroBias removed)
  Returns false if the I2C read failed
*/
bool IMU::getAll(void) {

  lsm6.read();

  // A failed I2C read returns 0xFF for every byte (-1 on every axis)
  if ((lsm6.a.x == -1) && (lsm6.a.y == -1) && (lsm6.a.z == -1) &&
      (lsm6.g.x == -1) && (lsm6.g.y == -1) && (lsm6.g.z == -1)) {
    return false;
  }

  const float accFactor = ACC_MG_PER_LSB / 1000.0f;     // counts to g
  const float gyroFactor = GYRO_MDPS_PER_LSB / 1000.0f; // counts to dps

  acc[X_AXIS] = lsm6.a.x * accFactor;
  acc[Y_AXIS] = lsm6.a.y * accFactor;
  acc[Z_AXIS] = lsm6.a.z * accFactor;

  gyro[X_AXIS] = lsm6.g.x * gyroFactor - gyroBias[X_AXIS];
  gyro[Y_AXIS] = lsm6.g.y * gyroFactor - gyroBias[Y_AXIS];
  gyro[Z_AXIS] = lsm6.g.z * gyroFactor - gyroBias[Z_AXIS];

  return true;
}

//*************************************************************************************************************************************************
// FUNCTIONS - ROAD AWARENESS
//*************************************************************************************************************************************************

/*
  Define threshold for vibrartions
  Checks terrain
  Fix according to speed !!
*/
void senseTerrain(void) {

  pothole = false;
  terrain = false;

  // Calibrated and scaled, so gravity is removed (~0 g at rest)
  float acc_x = abs(imu.get(ACC, X_AXIS, 1, 1));
  float acc_y = abs(imu.get(ACC, Y_AXIS, 1, 1));
  float acc_z = imu.get(ACC, Z_AXIS, 1, 1);

  // Sense vibrations
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

//*************************************************************************************************************************************************
// FUNCTIONS - KALMAN FILTER
//*************************************************************************************************************************************************
/*
  The Kalman filter alternates two steps:
    PREDICT (predict, ~100 Hz from the IMU):  move the state forward using the measured acceleration.
                                              Uncertainty P grows, because integrating acceleration drifts.
    CORRECT (correct, ~1 Hz from the GPS):    compare the prediction with a measurement.
                                              The Kalman gain K decides how far to move towards the measurement:
                                              K is large when the prediction is uncertain (P big) and small when
                                              the measurement is noisy (variance big). Uncertainty P then shrinks.
*/

/*
  Resets the filter to a known speed (m/s) with a fresh covariance
*/
void KalmanFilter::reset(float initialSpeed) {
  speed = max(0.0f, initialSpeed);
  accelBias = 0.0f;

  P00 = 0.25f;    // fairly confident in the starting speed
  P01 = 0.0f;
  P10 = 0.0f;
  P11 = 0.10f;    // unsure about the accelerometer bias
}

/*
  PREDICT: integrates the forward acceleration (m/s^2) over dt (s) into speed
  and grows the uncertainty, since a prediction is less certain than a measurement
*/
void KalmanFilter::predict(float acceleration, float dt) {

  // State prediction: speed += (acceleration - estimated bias) * dt
  float correctedAccel = acceleration - accelBias;

  speed += correctedAccel * dt;
  if (speed < 0.0f) {
    speed = 0.0f;   // speed is a magnitude, it cannot be negative
  }

  // Process noise Q: how much new uncertainty each prediction adds
  const float accelNoiseStd = 0.35f;    // m/s^2, accelerometer noise
  const float biasWalkStd = 0.015f;     // m/s^2 per sqrt(s), how fast the bias can wander

  float qVelocity = accelNoiseStd * accelNoiseStd * dt * dt;
  float qBias = biasWalkStd * biasWalkStd * dt;

  // Covariance prediction: P = F P F^T + Q, with F = [1 -dt; 0 1] (speed depends on bias)
  float oldP00 = P00;
  float oldP01 = P01;
  float oldP10 = P10;
  float oldP11 = P11;

  P00 = oldP00 - dt * oldP01 - dt * oldP10 + dt * dt * oldP11 + qVelocity;
  P01 = oldP01 - dt * oldP11;
  P10 = oldP10 - dt * oldP11;
  P11 = oldP11 + qBias;
}

/*
  CORRECT: blends a measured speed (m/s) into the prediction using the Kalman gain
  measurementVariance is how much the measurement is trusted (smaller = more trust)
  allowHardReset re-anchors the filter to the measurement if the prediction drifted badly
*/
void KalmanFilter::correct(float measuredSpeed, float measurementVariance, bool allowHardReset) {

  measuredSpeed = max(0.0f, measuredSpeed);

  // Innovation: how far the measurement is from the prediction
  float innovation = measuredSpeed - speed;

  // GPS is the absolute speed reference, re-anchor if the inertial prediction drifted badly
  if (allowHardReset && (fabsf(innovation) > 2.5f)) {
    speed = measuredSpeed;
    accelBias = 0.0f;

    P00 = measurementVariance;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 0.10f;
    return;
  }

  // Innovation variance S: total uncertainty of the prediction plus the measurement
  float S = P00 + measurementVariance;
  if (S < 0.000001f) {
    return;
  }

  // Kalman gain K: how much of the innovation to apply to each state
  float K0 = P00 / S;   // speed gain
  float K1 = P10 / S;   // bias gain (the bias is learnt through its correlation with speed)

  float oldP00 = P00;
  float oldP01 = P01;
  float oldP10 = P10;
  float oldP11 = P11;

  // State correction: x += K * innovation
  speed += K0 * innovation;
  accelBias += K1 * innovation;
  accelBias = constrain(accelBias, -0.40f, 0.40f);

  if (speed < 0.0f) {
    speed = 0.0f;
  }

  // Covariance correction (Joseph form, numerically stable): P = (I - K H) P (I - K H)^T + K R K^T
  float A = 1.0f - K0;

  P00 = A * A * oldP00 + K0 * K0 * measurementVariance;
  P01 = A * (oldP01 - K1 * oldP00) + K0 * K1 * measurementVariance;
  P10 = A * (oldP10 - K1 * oldP00) + K0 * K1 * measurementVariance;
  P11 = oldP11 - K1 * oldP01 - K1 * oldP10 + K1 * K1 * oldP00 + K1 * K1 * measurementVariance;

  // Keep the covariance symmetric
  float cross = 0.5f * (P01 + P10);
  P01 = cross;
  P10 = cross;
}

//*************************************************************************************************************************************************
// FUNCTIONS - SPEED FILTER (GPS + IMU FUSION)
//*************************************************************************************************************************************************

/*
  Initialises the speed fusion from the IMU calibration
  Call after imu.calibrate, with the node held still
*/
void SpeedFilter::init(void) {

  // The gyro bias is already in imu.gyroBias (imu.getAll removes it from imu.gyro)

  // Initial attitude from the gravity direction (the only acceleration while still)
  // imu.accBias is the average acceleration during calibration, so it points along gravity
  float ax = imu.accBias[X_AXIS];
  float ay = imu.accBias[Y_AXIS];
  float az = imu.accBias[Z_AXIS];
  roll = atan2f(ay, az);
  pitch = atan2f(ax, sqrtf(ay * ay + az * az));

  // The Kalman filter waits for the first GPS speed before it starts (see gpsUpdate)
  kalman.reset(0.0f);
  initialized = false;
  stationary = false;
  stationaryStart = 0;
  fwdAccelFiltered = 0.0f;
  lastFusion = micros();
}

/*
  Corrects the filter with a new GPS speed (m/s)
  The first valid GPS speed seeds the filter
*/
void SpeedFilter::gpsUpdate(float gpsSpeed) {

  gps.lastSpeed = gpsSpeed;

  // Only trust GPS speed with a valid, fresh fix
  if (!gps.fix) {
    return;
  }

  if (!initialized) {
    kalman.reset(gpsSpeed);    // first fix: start the filter from the GPS speed
    initialized = true;
  } else {
    kalman.correct(gpsSpeed, gps.speedVariance(), true);   // CORRECT step
  }

  lastGpsCorrection = millis();
}

/*
  Detects if the node is stopped using GPS speed and IMU stillness
*/
void SpeedFilter::stationaryUpdate(float accMagnitude, float gyroMagnitude) {

  // Both sensors must agree: GPS says slow, IMU says not rotating and only feeling gravity (1 g)
  bool freshGpsSpeed = gps.fix && gps.tinyGps.speed.isValid() && (gps.tinyGps.speed.age() < GPS_MAX_AGE);
  bool gpsStill = freshGpsSpeed && (gps.lastSpeed < STOP_SPEED_MPS);
  bool imuStill = (gyroMagnitude < STILL_GYRO_DPS) && (fabsf(accMagnitude - 1.0f) < STILL_ACCEL_TOL_G);

  // Must stay still for STILL_HOLD_MS before it counts, so brief pauses are ignored
  if (gpsStill && imuStill) {
    if (stationaryStart == 0) {
      stationaryStart = millis();
    }
    if (millis() - stationaryStart >= STILL_HOLD_MS) {
      stationary = true;
    }
  } else {
    stationaryStart = 0;
    stationary = false;
  }
}

/*
  Updates attitude and predicts speed from the IMU over dt (s)
  Mounting convention: +X = forward, +Y = left, +Z = up
  Speed is scalar and does not depend on yaw, so turns cannot directly create a large speed increase
*/
void SpeedFilter::imuUpdate(float dt) {

  // a. Read the IMU (imu.gyro already has imu.gyroBias removed) ------------------------------------------------------------------------------

  if (!imu.getAll()) {
    // Report the IMU as lost once, then keep retrying in case it recovers
    if (imu.fails < UINT16_MAX) {
      imu.fails++;
    }
    if (imu.fails == IMU_MAX_FAILS + 1) {
      reportError("IMU: connection lost\n");
    }
    return;
  }
  imu.fails = 0;

  float ax = imu.acc[X_AXIS];
  float ay = imu.acc[Y_AXIS];
  float az = imu.acc[Z_AXIS];

  float gx = imu.gyro[X_AXIS];
  float gy = imu.gyro[Y_AXIS];
  float gz = imu.gyro[Z_AXIS];

  float accMagnitude = sqrtf(ax * ax + ay * ay + az * az);
  float gyroMagnitude = sqrtf(gx * gx + gy * gy + gz * gz);

  // b. Stationary detection -----------------------------------------------------------------------------------------------------------------

  stationaryUpdate(accMagnitude, gyroMagnitude);

  // Slowly refine imu.gyroBias only while confidently stopped (any rotation reading must then be bias)
  // imu.gyro is the reading minus the current bias, so it is exactly the remaining bias error
  if (stationary) {
    const float alpha = 0.0015f;
    imu.gyroBias[X_AXIS] += alpha * gx;
    imu.gyroBias[Y_AXIS] += alpha * gy;
    imu.gyroBias[Z_AXIS] += alpha * gz;
  }

  // c. Attitude (roll/pitch) ----------------------------------------------------------------------------------------------------------------

  // Attitude from gravity: accurate long term, but noisy and fooled by acceleration
  float rollAcc = atan2f(ay, az);
  float pitchAcc = atan2f(ax, sqrtf(ay * ay + az * az));

  // Attitude from the gyro: smooth short term, but drifts
  // Convert to the body convention used by the rate equations
  float p = gx * DEG_TO_RAD;
  float q = -gy * DEG_TO_RAD;
  float r = -gz * DEG_TO_RAD;

  float sr = sinf(roll);
  float cr = cosf(roll);
  float cp = cosf(pitch);

  if (fabsf(cp) < 0.1f) {
    cp = (cp >= 0) ? 0.1f : -0.1f;    // avoid dividing by ~0 near +-90 deg pitch
  }

  float tanPitch = sinf(pitch) / cp;

  // Euler angle rates from the body rotation rates
  float rollRate = p + q * sr * tanPitch + r * cr * tanPitch;
  float pitchRate = q * cr - r * sr;

  float predictedRoll = roll + rollRate * dt;
  float predictedPitch = pitch + pitchRate * dt;

  // Adaptive complementary filter: mostly gyro, nudged towards gravity
  // The accelerometer is trusted less when it measures more than gravity (|acc| far from 1 g)
  float gravityError = fabsf(accMagnitude - 1.0f);
  float accTrust = 1.0f - constrain(gravityError / 0.10f, 0.0f, 1.0f);
  float gyroWeight = 1.0f - (1.0f - ATTITUDE_GYRO_WEIGHT) * accTrust;

  roll = gyroWeight * predictedRoll + (1.0f - gyroWeight) * rollAcc;
  pitch = gyroWeight * predictedPitch + (1.0f - gyroWeight) * pitchAcc;

  // d. Forward acceleration -----------------------------------------------------------------------------------------------------------------

  // The forward axis also feels gravity when pitched, so remove that component
  float fwdAccel = ax * GRAVITY - GRAVITY * sinf(pitch);

  // Low pass filter to smooth vibration
  fwdAccelFiltered += FWD_ACCEL_ALPHA * (fwdAccel - fwdAccelFiltered);

  // Ignore tiny accelerations (noise) and clamp unrealistic ones
  float usedAccel = fwdAccelFiltered;
  if (fabsf(usedAccel) < ACCEL_DEADBAND) {
    usedAccel = 0.0f;
  }
  usedAccel = constrain(usedAccel, -MAX_USED_ACCEL, MAX_USED_ACCEL);

  // e. PREDICT step -------------------------------------------------------------------------------------------------------------------------

  // Only predict when it is safe to:
  //  - fast rotations generate large transient accelerations
  //  - without a recent GPS correction, pure IMU integration drifts too far
  bool fastRotation = gyroMagnitude > TURN_SUPPRESS_DPS;
  bool gpsRecentlyCorrected = (lastGpsCorrection != 0) && (millis() - lastGpsCorrection < MAX_DEAD_RECKON_MS);

  if (initialized && gpsRecentlyCorrected && !stationary && !fastRotation) {
    kalman.predict(usedAccel, dt);
  }

  // f. While stopped: learn biases and apply a zero velocity update -------------------------------------------------------------------------

  // Any forward acceleration measured while stopped must be accelerometer bias
  if (stationary && initialized) {
    const float biasAlpha = 0.01f;
    kalman.accelBias += biasAlpha * (fwdAccelFiltered - kalman.accelBias);
  }

  // Zero velocity update (ZUPT): a CORRECT step with a very trusted measurement of 0 m/s
  if (stationary && initialized && (millis() - lastZupt >= ZUPT_INTVL)) {
    lastZupt = millis();

    kalman.correct(0.0f, 0.0025f, false);
    if (kalman.speed < 0.03f) {
      kalman.speed = 0.0f;
    }

    fwdAccelFiltered *= 0.8f;
  }
}

/*
  Runs the IMU prediction at FUSION_PERIOD_US
  Call every loop
*/
void SpeedFilter::update(void) {

  // Wait until FUSION_PERIOD_US has passed since the last prediction
  uint32_t now = micros();
  if ((uint32_t)(now - lastFusion) < FUSION_PERIOD_US) {
    return;
  }

  float dt = (now - lastFusion) / 1000000.0f;   // s
  lastFusion = now;

  // Skip unrealistic intervals (e.g. after blocking delays)
  if ((dt > 0.001f) && (dt < 0.10f)) {
    imuUpdate(dt);
  }
}

/*
  Returns the fused speed (km/h)
  Falls back to raw GPS speed until the filter is initialised
*/
float SpeedFilter::getSpeed(void) {

  if (initialized) {
    return kalman.speed * 3.6f;    // m/s to km/h
  }

  if (gps.fix && gps.tinyGps.speed.isValid()) {
    return gps.lastSpeed * 3.6f;
  }

  return 0.0f;
}
