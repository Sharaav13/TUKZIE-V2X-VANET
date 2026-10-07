#include <Wire.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
 
// ============================================================
// HARDWARE
// ============================================================
 
#define I2C_SDA 21
#define I2C_SCL 22
 
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600
 
#define MPU_ADDR  0x68
#define OLED_ADDR 0x3C
 
// ============================================================
// OLED
// ============================================================
 
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
 
Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
);
 
bool oledOK = false;
 
// ============================================================
//  GPS
// ============================================================
 
HardwareSerial GPSSerial(2);
TinyGPSPlus  gps;
 
bool gpsFix = false;
bool gpsDetected = false;
 
float lastGpsSpeed = 0.0f;     // m/s
float lastGpsCourse = 0.0f;    // degrees
 
uint32_t lastGpsByteTime = 0;
uint32_t lastGpsCorrectionTime = 0;
 
// ============================================================
// IMU STATUS
// ============================================================
 
bool imuOK = false;
uint16_t imuFailCount = 0;
 
// ============================================================
// CONSTANTS
// ============================================================
 
const float G = 9.80665f;
const float DEG_TO_RAD_F = PI / 180.0f;
const float RAD_TO_DEG_F = 180.0f / PI;
 
// MPU = 100 Hz
const uint32_t IMU_PERIOD_US = 10000;
 
// OLED = 5 Hz
const uint32_t OLED_PERIOD_MS = 200;
 
// ============================================================
// ATTITUDE FILTER
// ============================================================
 
const float ATTITUDE_GYRO_WEIGHT = 0.997f;
 
float roll = 0.0f;
float pitch = 0.0f;
 
bool attitudeInitialized = false;
 
// ============================================================
// STARTUP GYRO CALIBRATION
// ============================================================
 
const int CALIBRATION_SAMPLES = 1500;
 
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
 
// ============================================================
// FORWARD ACCELERATION
// ============================================================
 
float forwardAccelFiltered = 0.0f;
 
const float FORWARD_ACCEL_ALPHA = 0.12f;
const float ACCEL_DEADBAND = 0.06f;
const float MAX_USED_ACCEL = 4.0f;
 
// During a fast turn, do not integrate MPU acceleration.
const float TURN_SUPPRESS_DPS = 30.0f;
 
// Only allow pure IMU prediction for a short interval
// after the last valid GPS speed correction.
const uint32_t MAX_DEAD_RECKON_MS = 3000;
 
// ============================================================
// STATIONARY DETECTOR
// ============================================================
 
const float STOP_SPEED_MPS = 0.40f;
const float STILL_GYRO_DPS = 1.5f;
const float STILL_ACCEL_TOL_G = 0.06f;
const uint32_t STILL_HOLD_MS = 700;
 
bool stationary = false;
uint32_t stationaryCandidateTime = 0;
uint32_t lastZUPTTime = 0;
 
// ============================================================
// TIMERS
// ============================================================
 
uint32_t lastImuTime = 0;
uint32_t lastOLEDTime = 0;
uint32_t lastSerialTime = 0;
uint32_t systemStartTime = 0;
 
// ============================================================
// 2-STATE SPEED KALMAN FILTER
//
// State:
// x0 = speed
// x1 = forward  accelerometer bias
//
// GPS speed is the absolute reference.
// MPU6050 supplies short-term acceleration prediction.
// ============================================================
 
struct SpeedKalman
{
  float speed = 0.0f;
  float accelBias = 0.0f;
 
  float P00 = 1.0f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 0.10f;
 
  void reset(float initialSpeed)
  {
    speed = max(0.0f, initialSpeed);
    accelBias = 0.0f;
 
    P00 = 0.25f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 0.10f;
  }
 
  void predict(float acceleration, float dt)
  {
    float correctedAccel = acceleration - accelBias;
 
    speed += correctedAccel * dt;
 
    if (speed < 0.0f)
      speed = 0.0f;
 
    const float accelNoiseStd = 0.35f;
    const float biasWalkStd = 0.015f;
 
    float qVelocity = accelNoiseStd * accelNoiseStd * dt * dt;
    float qBias = biasWalkStd * biasWalkStd * dt;
 
    float oldP00 = P00;
    float oldP01 = P01;
    float oldP10 = P10;
    float oldP11 = P11;
 
    P00 =
      oldP00
      - dt * oldP01
      - dt * oldP10
      + dt * dt * oldP11
      + qVelocity;
 
    P01 = oldP01 - dt * oldP11;
    P10 = oldP10 - dt * oldP11;
    P11 = oldP11 + qBias;
  }
 
  void update(
    float measuredSpeed,
    float measurementVariance,
    bool allowHardReset
  )
  {
    measuredSpeed = max(0.0f, measuredSpeed);
 
    float innovation = measuredSpeed - speed;
 
    // GPS is the absolute speed reference. If the inertial
    // prediction ever drifts badly, immediately re-anchor it.
    if (
      allowHardReset &&
      fabsf(innovation) > 2.5f
    )
    {
      speed = measuredSpeed;
      accelBias = 0.0f;
 
      P00 = measurementVariance;
      P01 = 0.0f;
      P10 = 0.0f;
      P11 = 0.10f;
 
      return;
    }
 
    float S = P00 + measurementVariance;
 
    if (S < 0.000001f)
      return;
 
    float K0 = P00 / S;
    float K1 = P10 / S;
 
    float oldP00 = P00;
    float oldP01 = P01;
    float oldP10 = P10;
    float oldP11 = P11;
 
    speed += K0 * innovation;
    accelBias += K1 * innovation;
 
    accelBias = constrain(
      accelBias,
      -0.40f,
      0.40f
    );
 
    if (speed < 0.0f)
      speed = 0.0f;
 
    float A = 1.0f - K0;
 
    P00 =
      A * A * oldP00 +
      K0 * K0 * measurementVariance;
 
    P01 =
      A * (oldP01 - K1 * oldP00) +
      K0 * K1 * measurementVariance;
 
    P10 =
      A * (oldP10 - K1 * oldP00) +
      K0 * K1 * measurementVariance;
 
    P11 =
      oldP11
      - K1 * oldP01
      - K1 * oldP10
      + K1 * K1 * oldP00
      + K1 * K1 * measurementVariance;
 
    float cross = 0.5f * (P01 + P10);
    P01 = cross;
    P10 = cross;
  }
};
 
SpeedKalman speedFilter;
bool speedInitialized = false;
 
// ============================================================
// MPU REGISTER FUNCTIONS
// ============================================================
 
void writeMPU(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
 
uint8_t readMPURegister(uint8_t reg)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
 
  if (Wire.endTransmission(false) != 0)
    return 0xFF;
 
  Wire.requestFrom(
    (uint8_t)MPU_ADDR,
    (uint8_t)1
  );
 
  if (Wire.available())
    return Wire.read();
 
  return 0xFF;
}
 
// ============================================================
// DETECT MPU6050
// ============================================================
 
bool detectMPU()
{
  for (int i = 0; i < 3; i++)
  {
    uint8_t id = readMPURegister(0x75);
 
    if (id == 0x68)
      return true;
 
    delay(20);
  }
 
  return false;
}
 
// ============================================================
// INITIALIZE MPU6050
// ============================================================
 
void initMPU()
{
  writeMPU(0x6B, 0x01); // Wake, gyro PLL
  delay(100);
 
  writeMPU(0x19, 0x04); // Sample divider
  writeMPU(0x1A, 0x03); // DLPF
  writeMPU(0x1B, 0x00); // Gyro +/-250 dps
  writeMPU(0x1C, 0x00); // Accel +/-2g
 
  delay(100);
}
 
// ============================================================
// READ MPU6050
// ============================================================
 
bool readMPU(
  float &ax,
  float &ay,
  float &az,
  float &gx,
  float &gy,
  float &gz
)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
 
  if (Wire.endTransmission(false) != 0)
    return false;
 
  uint8_t received = Wire.requestFrom(
    (uint8_t)MPU_ADDR,
    (uint8_t)14
  );
 
  if (received != 14)
    return false;
 
  int16_t rawAX =
    (int16_t)((Wire.read() << 8) | Wire.read());
 
  int16_t rawAY =
    (int16_t)((Wire.read() << 8) | Wire.read());
 
  int16_t rawAZ =
    (int16_t)((Wire.read() << 8) | Wire.read());
 
  // Skip temperature
  Wire.read();
  Wire.read();
 
  int16_t rawGX =
    (int16_t)((Wire.read() << 8) | Wire.read());
 
  int16_t rawGY =
    (int16_t)((Wire.read() << 8) | Wire.read());
 
  int16_t rawGZ =
    (int16_t)((Wire.read() << 8) | Wire.read());
 
  ax = rawAX / 16384.0f;
  ay = rawAY / 16384.0f;
  az = rawAZ / 16384.0f;
 
  gx = rawGX / 131.0f;
  gy = rawGY / 131.0f;
  gz = rawGZ / 131.0f;
 
  return true;
}
 
// ============================================================
//  GPS SERVICE
// ============================================================
 
void serviceGPS()
{
  while (GPSSerial.available())
  {
    char c = GPSSerial.read();
 
    lastGpsByteTime = millis();
     gps.encode(c);
  }
 
  if (
    lastGpsByteTime != 0 &&
    millis() - lastGpsByteTime < 3000
  )
  {
    gpsDetected = true;
  }
  else
  {
    gpsDetected = false;
  }
}
 
// ============================================================
// OLED MESSAGE
// ============================================================
 
void oledMessage(
  const char *line1,
  const char *line2
)
{
  if (!oledOK)
    return;
 
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextWrap(false);
 
  display.setCursor(0, 20);
  display.println(line1);
 
  display.setCursor(0, 38);
  display.println(line2);
 
  display.display();
}
 
// ============================================================
// CALIBRATION SCREEN
// ============================================================
 
void drawCalibration(int percent)
{
  if (!oledOK)
    return;
 
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
  display.setTextSize(1);
 
  display.setCursor(28, 4);
  display.print("AUTO CALIBRATE");
 
  display.drawFastHLine(
    0,
    16,
    128,
    SSD1306_WHITE
  );
 
  display.setCursor(11, 25);
  display.print("KEEP DEVICE STILL");
 
  display.setCursor(52, 40);
  display.print(percent);
  display.print("%");
 
  display.drawRect(
    5,
    52,
    118,
    9,
    SSD1306_WHITE
  );
 
  int fill = map(
    percent,
    0,
    100,
    0,
    114
  );
 
  display.fillRect(
    7,
    54,
    fill,
    5,
    SSD1306_WHITE
  );
 
  display.display();
}
 
// ============================================================
// AUTOMATIC STARTUP CALIBRATION
//
// The device may be in any orientation. It only needs to stay
// still. The routine estimates gyro zero bias and initializes
// roll/pitch from the gravity direction.
// ============================================================
 
void autoCalibrateMPU()
{
  if (!imuOK)
    return;
 
  const int samples = CALIBRATION_SAMPLES;
 
  double sumAX = 0;
  double sumAY = 0;
  double sumAZ = 0;
 
  double sumGX = 0;
  double sumGY = 0;
  double sumGZ = 0;
 
  int good = 0;
 
  drawCalibration(0);
  delay(500);
 
  while (good < samples)
  {
    // Keep reading GPS even while the IMU is calibrating.
    serviceGPS();
 
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
 
    if (!readMPU(ax, ay, az, gx, gy, gz))
      continue;
 
    float aMag = sqrtf(
      ax * ax +
      ay * ay +
      az * az
    );
 
    float gMag = sqrtf(
      gx * gx +
      gy * gy +
      gz * gz
    );
 
    bool still =
      aMag > 0.75f &&
      aMag < 1.25f &&
      gMag < 5.0f;
 
    if (!still)
    {
      good = 0;
 
      sumAX = 0;
      sumAY = 0;
      sumAZ = 0;
 
      sumGX = 0;
      sumGY = 0;
      sumGZ = 0;
 
      drawCalibration(0);
      delay(100);
      continue;
    }
 
    sumAX += ax;
    sumAY += ay;
    sumAZ += az;
 
    sumGX += gx;
    sumGY += gy;
    sumGZ += gz;
 
    good++;
 
    if (good % 30 == 0)
    {
      int percent = good * 100 / samples;
      drawCalibration(percent);
    }
 
    delay(2);
  }
 
  float avgAX = sumAX / samples;
  float avgAY = sumAY / samples;
  float avgAZ = sumAZ / samples;
 
  gyroBiasX = sumGX / samples;
  gyroBiasY = sumGY / samples;
  gyroBiasZ = sumGZ / samples;
 
  roll = atan2f(avgAY, avgAZ);
 
  pitch = atan2f(
    avgAX,
    sqrtf(avgAY * avgAY + avgAZ * avgAZ)
  );
 
  attitudeInitialized = true;
 
  drawCalibration(100);
  delay(600);
 
  Serial.println();
  Serial.println("MPU automatic calibration complete");
 
  Serial.print("Gyro bias X: ");
  Serial.println(gyroBiasX, 5);
 
  Serial.print("Gyro bias Y: ");
  Serial.println(gyroBiasY, 5);
 
  Serial.print("Gyro bias Z: ");
  Serial.println(gyroBiasZ, 5);
}
 
// ============================================================
//  GPS STATUS
// ============================================================
 
void updateGPSStatus()
{
  gpsFix =
    gpsDetected &&
     gps.location.isValid() &&
    gps.location.age() < 2000 &&
    gps.speed.isValid();
}
 
// ============================================================
// GPS SPEED MEASUREMENT QUALITY
// ============================================================
 
float gpsSpeedVariance()
{
  float sigma = 0.25f;
 
  // Slightly more conservative at walking speed.
  if (lastGpsSpeed < 2.0f)
    sigma = 0.32f;
 
  if (
    gps.satellites.isValid() &&
    gps.satellites.value() < 5
  )
  {
    sigma *= 1.4f;
  }
 
  return sigma * sigma;
}
 
// ============================================================
// GPS SPEED FUSION
//
// GPS speed directly corrects the main speed estimate.
// Heading/yaw is not required for the primary speed output.
// ============================================================
 
void updateGPSFusion()
{
  if (!gps.speed.isUpdated())
    return;
 
  lastGpsSpeed = gps.speed.mps();
 
  if (!gpsFix)
    return;
 
  if (gps.course.isValid())
    lastGpsCourse = gps.course.deg();
 
  if (!speedInitialized)
  {
    speedFilter.reset(lastGpsSpeed);
    speedInitialized = true;
  }
  else
  {
    speedFilter.update(
      lastGpsSpeed,
      gpsSpeedVariance(),
      true
    );
  }
 
  lastGpsCorrectionTime = millis();
}
 
// ============================================================
// STATIONARY DETECTOR
// ============================================================
 
void updateStationary(
  float accelMagnitude,
  float gyroMagnitude
)
{
  bool freshGpsSpeed =
    gpsFix &&
    gps.speed.isValid() &&
    gps.speed.age() < 2000;
 
  bool gpsStill =
    freshGpsSpeed &&
    lastGpsSpeed < STOP_SPEED_MPS;
 
  bool imuStill =
    gyroMagnitude < STILL_GYRO_DPS &&
    fabsf(accelMagnitude - 1.0f) < STILL_ACCEL_TOL_G;
 
  if (gpsStill && imuStill)
  {
    if (stationaryCandidateTime == 0)
      stationaryCandidateTime = millis();
 
    if (
      millis() - stationaryCandidateTime >=
      STILL_HOLD_MS
    )
    {
      stationary = true;
    }
  }
  else
  {
    stationaryCandidateTime = 0;
    stationary = false;
  }
}
 
// ============================================================
// UPDATE MPU6050
//
// Mounting convention:
// +X = direction of travel / forward
// +Y = left
// +Z = up
//
// The main speed is scalar and does not depend on yaw. A turn
// therefore cannot directly create a large speed increase.
// ============================================================
 
void updateIMU(float dt)
{
  if (!imuOK)
    return;
 
  float ax;
  float ay;
  float az;
 
  float rawGX;
  float rawGY;
  float rawGZ;
 
  if (
    !readMPU(
      ax,
      ay,
      az,
      rawGX,
      rawGY,
      rawGZ
    )
  )
  {
    imuFailCount++;
 
    if (imuFailCount > 100)
      imuOK = false;
 
    return;
  }
 
  imuFailCount = 0;
 
  float gx = rawGX - gyroBiasX;
  float gy = rawGY - gyroBiasY;
  float gz = rawGZ - gyroBiasZ;
 
  float accelMagnitude = sqrtf(
    ax * ax +
    ay * ay +
    az * az
  );
 
  float gyroMagnitude = sqrtf(
    gx * gx +
    gy * gy +
    gz * gz
  );
 
  updateStationary(
    accelMagnitude,
    gyroMagnitude
  );
 
  // Slowly update gyro bias only while confidently stopped.
  if (stationary)
  {
    const float alpha = 0.0015f;
 
    gyroBiasX += alpha * (rawGX - gyroBiasX);
    gyroBiasY += alpha * (rawGY - gyroBiasY);
    gyroBiasZ += alpha * (rawGZ - gyroBiasZ);
  }
 
  // ==========================================================
  // ATTITUDE
  // ==========================================================
 
  float rollAccel = atan2f(ay, az);
 
  float pitchAccel = atan2f(
    ax,
    sqrtf(ay * ay + az * az)
  );
 
  if (!attitudeInitialized)
  {
    roll = rollAccel;
    pitch = pitchAccel;
    attitudeInitialized = true;
  }
 
  // Convert to the body convention used by the rate equations.
  float p = gx * DEG_TO_RAD_F;
  float q = -gy * DEG_TO_RAD_F;
  float r = -gz * DEG_TO_RAD_F;
 
  float sr = sinf(roll);
  float cr = cosf(roll);
  float cp = cosf(pitch);
 
  if (fabsf(cp) < 0.1f)
    cp = (cp >= 0) ? 0.1f : -0.1f;
 
  float tanPitch = sinf(pitch) / cp;
 
  float rollRate =
    p +
    q * sr * tanPitch +
    r * cr * tanPitch;
 
  float pitchRate =
    q * cr -
    r * sr;
 
  float predictedRoll =
    roll + rollRate * dt;
 
  float predictedPitch =
    pitch + pitchRate * dt;
 
  // ==========================================================
  // ADAPTIVE COMPLEMENTARY FILTER
  // ==========================================================
 
  float gravityError =
    fabsf(accelMagnitude - 1.0f);
 
  float accelTrust =
    1.0f -
    constrain(
      gravityError / 0.10f,
      0.0f,
      1.0f
    );
 
  float gyroWeight =
    1.0f -
    (1.0f - ATTITUDE_GYRO_WEIGHT) *
    accelTrust;
 
  roll =
    gyroWeight * predictedRoll +
    (1.0f - gyroWeight) * rollAccel;
 
  pitch =
    gyroWeight * predictedPitch +
    (1.0f - gyroWeight) * pitchAccel;
 
  // ==========================================================
  // FORWARD LINEAR ACCELERATION
  //
  //  Accelerometer X contains a gravity component when pitched.
  // Remove that gravity component before integrating speed.
  // ==========================================================
 
  float forwardAccel =
    ax * G -
    G * sinf(pitch);
 
  forwardAccelFiltered +=
    FORWARD_ACCEL_ALPHA *
    (forwardAccel - forwardAccelFiltered);
 
  float usedAccel = forwardAccelFiltered;
 
  if (fabsf(usedAccel) < ACCEL_DEADBAND)
    usedAccel = 0.0f;
 
  usedAccel = constrain(
    usedAccel,
    -MAX_USED_ACCEL,
    MAX_USED_ACCEL
  );
 
  // Fast hand/device rotations can generate large transient
  // accelerations. Suppress inertial prediction during them.
  bool fastRotation =
    gyroMagnitude > TURN_SUPPRESS_DPS;
 
  bool gpsRecentlyCorrected =
    lastGpsCorrectionTime != 0 &&
    millis() - lastGpsCorrectionTime < MAX_DEAD_RECKON_MS;
 
  if (
    speedInitialized &&
    gpsRecentlyCorrected &&
    !stationary &&
    !fastRotation
  )
  {
    speedFilter.predict(
      usedAccel,
      dt
    );
  }
 
  // Learn residual forward-acceleration bias while stopped.
  if (
    stationary &&
    speedInitialized
  )
  {
    const float biasAlpha = 0.01f;
 
    speedFilter.accelBias +=
      biasAlpha *
      (
        forwardAccelFiltered -
        speedFilter.accelBias
      );
  }
 
  // ZUPT at 10 Hz when confidently stationary.
  if (
    stationary &&
    speedInitialized &&
    millis() - lastZUPTTime >= 100
  )
  {
    lastZUPTTime = millis();
 
    speedFilter.update(
      0.0f,
      0.0025f,
      false
    );
 
    if (speedFilter.speed < 0.03f)
      speedFilter.speed = 0.0f;
 
    forwardAccelFiltered *= 0.8f;
  }
}
 
// ============================================================
// MAIN FUSED SPEED
// ============================================================
 
float getFusedSpeed()
{
  if (speedInitialized)
    return speedFilter.speed;
 
  if (
    gpsFix &&
     gps.speed.isValid()
  )
  {
    return lastGpsSpeed;
  }
 
  return 0.0f;
}
 
// ============================================================
// SENSOR ERROR SCREEN
// ============================================================
 
void drawSensorError()
{
  if (!oledOK)
    return;
 
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
  display.setTextSize(1);
 
  display.setCursor(20, 4);
  display.print("SENSOR ERROR");
 
  display.drawFastHLine(
    0,
    16,
    128,
    SSD1306_WHITE
  );
 
  if (!imuOK && !gpsDetected)
  {
    display.setCursor(20, 26);
    display.print(" GPS NOT DETECTED");
 
    display.setCursor(20, 42);
    display.print("IMU NOT DETECTED");
  }
  else if (!imuOK)
  {
    display.setCursor(13, 28);
    display.print("IMU NOT DETECTED");
 
    display.setCursor(12, 45);
    display.print("CHECK SDA / SCL");
  }
  else if (!gpsDetected)
  {
    display.setCursor(12, 28);
    display.print("GPS NOT DETECTED");
 
    display.setCursor(7, 45);
    display.print("CHECK TX -> GPIO16");
  }
 
  display.display();
}
 
// ============================================================
// MAIN OLED SCREEN
// ============================================================
 
void updateOLED()
{
  if (!oledOK)
    return;
 
  if (
    millis() - lastOLEDTime <
    OLED_PERIOD_MS
  )
  {
    return;
  }
 
  lastOLEDTime = millis();
 
  // Allow the GPS several seconds to begin sending UART data
  // before declaring it missing.
  bool gpsMissing =
    millis() - systemStartTime > 4000 &&
    !gpsDetected;
 
  if (!imuOK || gpsMissing)
  {
    drawSensorError();
    return;
  }
 
  float fusedSpeedMPS = getFusedSpeed();
  float fusedSpeedKMH = fusedSpeedMPS * 3.6f;
  float gpsSpeedKMH = lastGpsSpeed * 3.6f;
 
  bool goodCourse =
    gpsFix &&
    gps.course.isValid() &&
    gps.course.age() < 2000 &&
    lastGpsSpeed > 0.70f;
 
  float heading = lastGpsCourse;
 
  float velocityN = 0.0f;
  float velocityE = 0.0f;
 
  // VN / VE are display values derived from the fused scalar
  // speed and the current GPS course. They are not used to
  // calculate the main speed.
  if (goodCourse)
  {
    float courseRad =
      heading * DEG_TO_RAD_F;
 
    velocityN =
      fusedSpeedMPS *
      cosf(courseRad);
 
    velocityE =
      fusedSpeedMPS *
      sinf(courseRad);
  }
 
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
 
  // ==========================================================
  // TOP YELLOW AREA - LARGE FUSED SPEED
  // ==========================================================
 
  char speedString[12];
 
  snprintf(
    speedString,
    sizeof(speedString),
    "%.1f",
    fusedSpeedKMH
  );
 
  int16_t x1;
  int16_t y1;
  uint16_t w;
  uint16_t h;
 
  display.setTextSize(2);
 
  display.getTextBounds(
    speedString,
    0,
    0,
    &x1,
    &y1,
    &w,
    &h
  );
 
  int totalWidth =
    w + 4 + 24;
 
  int startX =
    (128 - totalWidth) / 2;
 
  if (startX < 0)
    startX = 0;
 
  display.setCursor(startX, 0);
  display.print(speedString);
 
  display.setTextSize(1);
 
  display.setCursor(
    startX + w + 4,
    7
  );
 
  display.print("km/h");
 
  display.drawFastHLine(
    0,
    16,
    128,
    SSD1306_WHITE
  );
 
  // ==========================================================
  // GPS FIX + SATELLITES
  // ==========================================================
 
  display.setCursor(0, 19);
  display.print("GPS FIX:");
 
  if (gpsFix)
    display.print("YES");
  else
    display.print("NO");
 
  display.setCursor(88, 19);
  display.print("S:");
 
  if ( gps.satellites.isValid())
  {
    uint32_t sats =
      gps.satellites.value();
 
    if (sats < 10)
      display.print("0");
 
    display.print(sats);
  }
  else
  {
    display.print("--");
  }
 
  display.drawFastHLine(
    0,
    28,
    128,
    SSD1306_WHITE
  );
 
  // ==========================================================
  // RAW  GPS SPEED
  // ==========================================================
 
  display.setCursor(0, 31);
  display.print("GPS ");
 
  if (gpsFix)
  {
    display.print(
      gpsSpeedKMH,
      1
    );
  }
  else
  {
    display.print("--.-");
  }
 
  display.print(" km/h");
 
  // ==========================================================
  // HEADING / HDOP
  // ==========================================================
 
  display.setCursor(0, 42);
  display.print("HDG ");
 
  if (goodCourse)
  {
    int headingInt =
      (int)roundf(heading);
 
    if (headingInt >= 360)
      headingInt = 0;
 
    if (headingInt < 100)
      display.print("0");
 
    if (headingInt < 10)
      display.print("0");
 
    display.print(headingInt);
  }
  else
  {
    display.print("---");
  }
 
  display.print("   HDOP ");
 
  if (gps.hdop.isValid())
  {
    display.print(
      gps.hdop.hdop(),
      1
    );
  }
  else
  {
    display.print("--");
  }
 
  // ==========================================================
  // NORTH / EAST VELOCITY DISPLAY
  // ==========================================================
 
  display.setCursor(0, 53);
  display.print("VN ");
 
  if (goodCourse)
  {
    if (velocityN >= 0)
      display.print("+");
 
    display.print(
      velocityN,
      1
    );
  }
  else
  {
    display.print("--");
  }
 
  display.print("  VE ");
 
  if (goodCourse)
  {
    if (velocityE >= 0)
      display.print("+");
 
    display.print(
      velocityE,
      1
    );
  }
  else
  {
    display.print("--");
  }
 
  display.display();
}
 
// ============================================================
// SERIAL DEBUG
// ============================================================
 
void printSerial()
{
  Serial.println();
  Serial.println("========================================");
 
  Serial.print("GPS detected:     ");
  Serial.println(gpsDetected ? "YES" : "NO");
 
  Serial.print("IMU detected:     ");
  Serial.println(imuOK ? "YES" : "NO");
 
  Serial.print("GPS fix:          ");
  Serial.println(gpsFix ? "YES" : "NO");
 
  Serial.print("GPS speed:        ");
  Serial.print(lastGpsSpeed * 3.6f, 2);
  Serial.println(" km/h");
 
  Serial.print("FUSED speed:      ");
  Serial.print(getFusedSpeed() * 3.6f, 2);
  Serial.println(" km/h");
 
  Serial.print("Forward accel:    ");
  Serial.print(forwardAccelFiltered, 4);
  Serial.println(" m/s2");
 
  Serial.print("Accel bias est:   ");
  Serial.print(speedFilter.accelBias, 5);
  Serial.println(" m/s2");
 
  Serial.print("Stationary:       ");
  Serial.println(stationary ? "YES" : "NO");
 
  Serial.print("Roll:             ");
  Serial.print(roll * RAD_TO_DEG_F, 2);
  Serial.println(" deg");
 
  Serial.print("Pitch:            ");
  Serial.print(pitch * RAD_TO_DEG_F, 2);
  Serial.println(" deg");
 
  if ( gps.course.isValid())
  {
    Serial.print("GPS course:       ");
    Serial.print(gps.course.deg(), 1);
    Serial.println(" deg");
  }
}
 
// ============================================================
// SETUP
// ============================================================
 
void setup()
{
  Serial.begin(115200);
  delay(500);
 
  systemStartTime = millis();
 
  // Start  GPS early so it can acquire satellites while the
  // IMU startup calibration is running.
  GPSSerial.begin(
    GPS_BAUD,
    SERIAL_8N1,
    GPS_RX,
    GPS_TX
  );
 
  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );
 
  Wire.setClock(400000);
 
  oledOK = display.begin(
    SSD1306_SWITCHCAPVCC,
    OLED_ADDR,
    true,
    false
  );
 
  if (oledOK)
  {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
 
    display.setCursor(17, 20);
    display.print("GPS + MPU6050");
 
    display.setCursor(16, 37);
    display.print("Velocity Fusion");
 
    display.display();
    delay(700);
  }
  else
  {
    Serial.println("OLED NOT DETECTED");
  }
 
  imuOK = detectMPU();
 
  if (imuOK)
  {
    Serial.println("MPU6050 detected");
 
    initMPU();
    autoCalibrateMPU();
  }
  else
  {
    Serial.println("MPU6050 NOT DETECTED");
 
    oledMessage(
      "IMU NOT DETECTED",
      "CHECK SDA / SCL"
    );
 
    delay(2000);
  }
 
  speedFilter.reset(0.0f);
  speedInitialized = false;
 
  lastImuTime = micros();
}
 
// ============================================================
// LOOP
// ============================================================
 
void loop()
{
  serviceGPS();
 
  updateGPSStatus();
  updateGPSFusion();
 
  uint32_t now = micros();
 
  if (
    (uint32_t)(now - lastImuTime) >=
    IMU_PERIOD_US
  )
  {
    float dt =
      (now - lastImuTime) /
      1000000.0f;
 
    lastImuTime = now;
 
    if (
      dt > 0.001f &&
      dt < 0.10f
    )
    {
      updateIMU(dt);
    }
  }
 
  updateOLED();
 
  if (
    millis() - lastSerialTime >=
    500
  )
  {
    lastSerialTime = millis();
    printSerial();
  }
}

