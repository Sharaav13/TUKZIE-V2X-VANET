//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - LoRa RADIO
//*************************************************************************************************************************************************

#ifndef TVV_LORA_H
#define TVV_LORA_H

#include "System.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern SPIClass spiLoRa;

extern uint32_t lastBeacon;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Setup
extern bool loraOK;

// LoRa
extern int rssi;
extern float snr;
extern long freqErr;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup LoRa
String loraInit(void);

// Procedure for receiving LoRa packet
String loraRx(void);

// Procedure for transmitting LoRa packet
void loraTx(String payload);

// Converts LoRa Rx information into a single string
size_t loraRxStr(char *out, size_t cap, String content);

// Converts LoRa Tx information into a single string
size_t loraTxStr(char *out, size_t cap, String content);

// Shows LoRa Tx and Rx
void loraDebug(int select);

#endif // TVV_LORA_H
