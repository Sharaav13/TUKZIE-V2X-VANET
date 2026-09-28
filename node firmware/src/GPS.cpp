//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - GPS & PPS
//*************************************************************************************************************************************************

#include "GPS.h"
#include "IMU.h"       // motionGet() calls imuGet()
#include "Display.h"   // clockDiff() calls disp()

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

HardwareSerial uartGPS(1);  // UART1
TinyGPSPlus gps;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// GPS
int datetime[6];      // {day, month, year, hour, min, sec}
float location[3];    // {lat, lon, alt} (signed: +N/-S, +E/-W)
float motion[2];      // {kmph, course}
float fix[2];         // {sats, hdop}
volatile uint32_t countPps = 0; // in seconds
volatile uint32_t lastPps = 0;
volatile uint32_t gapMs  = 0;

// Debug
uint32_t lastGps = 0;     // Timestamp of previous GPS sample

//*************************************************************************************************************************************************
// INTERRUPTS
//*************************************************************************************************************************************************

// Records a second indicated by PPS
void IRAM_ATTR ppsISR() {
  uint32_t now = millis();
  gapMs  = now - lastPps;
  lastPps = now;
  countPps++;
}

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup GPS and PPS
String gpsInit(void) {
  uartGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  //pinMode(PPS, INPUT);
  //attachInterrupt(digitalPinToInterrupt(PPS), ppsISR, RISING);
  return "GPS init pass.\n";
}

// Retreives GPS data
// Used for GPS module testing
void gpsGetAll(void) {

  while (uartGPS.available()) {
    gps.encode(uartGPS.read());
  }

  datetimeGet();
  locationGet();
  motionGet();
  fixGet();

}

// Updates date and time data
void datetimeGet(void) {

  while (uartGPS.available()) {
    gps.encode(uartGPS.read());
  }

  // Date and time
  if (gps.time.isValid() && gps.date.isValid()) {
    int year_temp = gps.date.year();
    int month_temp = gps.date.month();
    int day_temp = gps.date.day();
    int hour_temp = gps.time.hour();
    int minute_temp = gps.time.minute();
    int sec_temp = gps.time.second();

    datetime[YEAR] = year_temp;
    datetime[MONTH] = month_temp;
    datetime[DAY] = day_temp;

    datetime[MINUTE] = minute_temp;
    datetime[SECOND] = sec_temp;
    int hour_corrected = hour_temp + 2; // SAST = UTC + 2hr
    datetime[HOUR] =(hour_corrected < 24) ? hour_corrected : (hour_corrected - 24);
  }
}

// Updates location data
void locationGet(void) {

  while (uartGPS.available()) {
    gps.encode(uartGPS.read());
  }

  // Location
  if (gps.location.isUpdated()) {
    float lat_temp = gps.location.lat();
    float lon_temp = gps.location.lng();
    location[LAT] = lat_temp;
    location[LON] = lon_temp;
  }

  // Altitude
  if (gps.altitude.isValid()) {
    float alt_temp = gps.altitude.meters();
    location[ALT] = alt_temp;
  }
}

// Updates motion data
void motionGet(void) {

  while (uartGPS.available()) {
    gps.encode(uartGPS.read());
  }

  // Speed
  if (gps.speed.isUpdated()) {
    float speed_gps_temp = gps.speed.kmph();
    motion[SPEED] = speed_gps_temp;
  }

  // Course
  if (gps.course.isValid()) {
    float course_temp = gps.course.deg();
    motion[COURSE] = course_temp;
  }
}

// Updates satellite fix data
void fixGet(void) {

  while (uartGPS.available()) {
    gps.encode(uartGPS.read());
  }

  // Fix quality
  if (gps.satellites.isValid() && gps.hdop.isValid()) {
    int sats_temp = gps.satellites.value();
    float hdop_temp = gps.hdop.hdop();
    fix[SATS] = sats_temp;
    fix[HDOP] = hdop_temp;
  }
}

// Calculates the error in clock synchronisation with PPS
void clockDiff(void) {
  String clock_diff = "PPS Count = " + String(countPps) + "\n" + "Clock erro = " + String(gapMs);
  disp(clock_diff, 1, false, false);
}

// Converts currently stored GPS data into a single string
// Used for GPS module testing
size_t gpsStr(char *out, size_t cap) {

  return snprintf(out, cap,
        "%02d-%02d-%04d  %02d:%02d:%02d\n"
        "lat: %.4f deg\n"
        "lon: %.4f deg\n"
        "alt: %.1f m\n"
        "speed: %.1f kmph\n"
        "course: %.1f deg\n"
        "sats: %d   hdop: %.1f",
        datetime[DAY], datetime[MONTH], datetime[YEAR],
        datetime[HOUR], datetime[MINUTE], datetime[SECOND],
        location[LAT],
        location[LON],
        location[ALT],
        motion[SPEED], motion[COURSE],
        (int)fix[SATS], fix[HDOP]);
}

// Displays all GPS data periodically
void gpsDebug(void) {
  if ((millis() - lastGps) >= GPS_SAMPLING) {
    gpsGetAll();
    char gps_str[192];
    gpsStr(gps_str, sizeof(gps_str));
    disp(gps_str, 1, false, false);
    lastGps = millis();
  }
}
