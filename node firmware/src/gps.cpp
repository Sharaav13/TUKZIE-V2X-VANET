//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - GPS & PPS
//*************************************************************************************************************************************************

#include "system.h"
#include "gps.h"
#include "imu.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

HardwareSerial uartGPS(1);  // UART1 // Put into main or system
TinyGPSPlus gps;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

volatile uint32_t countPps =  0; // seconds
volatile uint32_t lastPps =   0;
volatile uint32_t gapMs  =    0;

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
// Returns true if the GPS UART driver was installed
bool gpsInit(void) {
  uartGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  //pinMode(PPS, INPUT);
  //attachInterrupt(digitalPinToInterrupt(PPS), ppsISR, RISING);

  if (!uartGPS) {
    error = true;
    errMsg += "GPS: UART driver failed to install\n";
    return false;
  }

  gpsOK = true;
  return true;
}

// Retreives GPS data and saves it to myself
// Fix quality is saved to myself.sats and myself.hdop
void gpsGet(void) {

  while (uartGPS.available()) {
    gps.encode(uartGPS.read());
  }

  // Date and time (converted from UTC to SAST)
  if (gps.time.isValid() && gps.date.isValid()) {
    int day   = gps.date.day();
    int month = gps.date.month();
    int year  = gps.date.year();
    int hour  = gps.time.hour() + SAST_OFFSET;

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
    myself.time = (uint32_t)hour * 1000000 + gps.time.minute() * 10000
                + gps.time.second() * 100 + gps.time.centisecond();     // HHMMSSCC (SAST)
  }

  // Location
  if (gps.location.isUpdated()) {
    myself.lat = gps.location.lat();
    myself.lon = gps.location.lng();
  }

  // Altitude
  if (gps.altitude.isValid()) {
    myself.alt = gps.altitude.meters();
  }

  // Speed
  if (gps.speed.isUpdated()) {
    myself.speed  = gps.speed.kmph();
  }

  // Course
  if (gps.course.isValid()) {
    myself.course = gps.course.deg();
  }

  // Fix quality
  if (gps.satellites.isValid() && gps.hdop.isValid()) {
    myself.sats = gps.satellites.value();
    myself.hdop = gps.hdop.hdop();
  }
}
