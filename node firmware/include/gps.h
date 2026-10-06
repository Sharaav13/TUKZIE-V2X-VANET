//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - GPS & PPS
//*************************************************************************************************************************************************

#ifndef TVV_GPS_H
#define TVV_GPS_H

#include "System.h"

//*************************************************************************************************************************************************
// MACROS
//*************************************************************************************************************************************************

// Date and time indices (datetime)
#define DAY     0
#define MONTH   1
#define YEAR    2
#define HOUR    3
#define MINUTE  4
#define SECOND  5
#define CENTISECOND  6

// Location indices (location)
#define LAT  0
#define LON  1
#define ALT  2

// Motion indices (motion)
#define SPEED   0
#define COURSE  1

// Satellite fix indices (fix)
#define SATS  0
#define HDOP  1

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern HardwareSerial uartGPS;  // UART1
extern TinyGPSPlus gps;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Setup
extern bool gpsOK;

// GPS
extern int datetime[7];      // {day, month, year, hour, min, sec, centisec}
extern float location[3];    // {lat, lon, alt} (signed: +N/-S, +E/-W)
extern float motion[2];      // {kmph, course}
extern float fix[2];         // {sats, hdop}
extern volatile uint32_t countPps; // in seconds
extern volatile uint32_t lastPps;
extern volatile uint32_t gapMs;

// Debug
extern uint32_t lastGps;     // Timestamp of previous GPS sample

//*************************************************************************************************************************************************
// INTERRUPTS
//*************************************************************************************************************************************************

// Records a second indicated by PPS
void IRAM_ATTR ppsISR();

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup GPS and PPS
String gpsInit(void);

// Retreives GPS data
// Used for GPS module testing
void gpsGetAll(void);

// Updates date and time data
void datetimeGet(void);

// Updates location data
void locationGet(void);

// Updates motion data
void motionGet(void);

// Updates satellite fix data
void fixGet(void);

// Calculates the error in clock synchronisation with PPS
void clockDiff(void);

// Converts currently stored GPS data into a single string
// Used for GPS module testing
size_t gpsStr(char *out, size_t cap);

// Displays all GPS data periodically
void gpsDebug(void);

#endif // TVV_GPS_H
