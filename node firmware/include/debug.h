//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - DEBUG
//*************************************************************************************************************************************************

#ifndef TVV_DEBUG_H
#define TVV_DEBUG_H

#include "system.h"

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Timestamps of previous debug actions
extern uint32_t lastSdWrite; // Timestamp of previous SD Card write
extern uint32_t lastSdRead;  // Timestamp of previous SD Card read
extern uint32_t lastGps;     // Timestamp of previous GPS sample
extern uint32_t lastImu;     // Timestamp of previous IMU sample
extern uint32_t lastHudTx;   // Timestamp of previous MNS transmission
extern uint32_t lastBeacon;  // Timestamp of previous LoRa transmission

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Runs the debug routine of the selected peripheral
// peripheral: SD_CARD, GPS, PPS_CLOCK, IMU, MNS, LORA_RX, LORA_TX or LORA
void debug(int peripheral);

// Writes and reads to the SD card
void sdDebug(void);

// Calculates the error in clock synchronisation with PPS
void clockDiff(void);

// Converts currently stored GPS data into a single string
// Used for GPS module testing
size_t gpsStr(char *out, size_t cap);

// Displays all GPS data periodically
void gpsDebug(void);

// Converts currently stored IMU data into a single string
// Used for IMU module testing
size_t imuStr(char *out, size_t cap);

// Displays all IMU data periodically
void imuDebug(void);

// Debugs if HUD Tx and Rx ports are working
// Tx and Rx are wired together
void uartMNSDebug(void);

// Procedure for receiving LoRa packet
String loraRx(void);

// Procedure for transmitting LoRa packet
void loraTx(String payload);

// Converts LoRa Rx information into a single string
size_t loraRxStr(char *out, size_t cap, String pkt);

// Converts LoRa Tx information into a single string
size_t loraTxStr(char *out, size_t cap, String pkt);

// Shows LoRa Tx and Rx
void loraDebug(int select);

#endif // TVV_DEBUG_H
