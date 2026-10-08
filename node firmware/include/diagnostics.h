//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - DEBUG
//*************************************************************************************************************************************************

#ifndef TVV_DEBUG_H
#define TVV_DEBUG_H

#include "system.h"

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

/*
  Runs the debug routine of the selected peripheral
  peripheral: DBG_SD_CARD, DBG_GPS, DBG_PPS_CLOCK, DBG_IMU, DBG_DASH, DBG_LORA_RX, DBG_LORA_TX or DBG_LORA
*/
void debug(uint8_t peripheral);

/*
  Writes and reads to the SD card
*/
void sdDebug(void);

/*
  Calculates the error in clock synchronisation with PPS
*/
void clockDiff(void);

/*
  Converts currently stored GPS data into a single string
  Used for GPS module testing
*/
size_t gpsStr(char *out, size_t cap);

/*
  Displays all GPS data periodically
*/
void gpsDebug(void);

/*
  Converts currently stored IMU data into a single string
  Used for IMU module testing
*/
size_t imuStr(char *out, size_t cap);

/*
  Displays all IMU data periodically
*/
void imuDebug(void);

/*
  Debugs if dashboard Tx and Rx ports are working
  Tx and Rx are wired together
*/
void dashDebug(void);

/*
  Procedure for receiving LoRa packet
*/
String loraRx(void);

/*
  Procedure for transmitting LoRa packet
*/
void loraTx(String payload);

/*
  Converts LoRa Rx information into a single string
*/
size_t loraRxStr(char *out, size_t cap, String pkt);

/*
  Converts LoRa Tx information into a single string
*/
size_t loraTxStr(char *out, size_t cap, String pkt);

/*
  Shows LoRa Tx and Rx
*/
void loraDebug(uint8_t select);

#endif // TVV_DEBUG_H
