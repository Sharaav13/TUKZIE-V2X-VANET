//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SD CARD
//*************************************************************************************************************************************************

#ifndef TVV_SD_H
#define TVV_SD_H

#include "System.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern SPIClass spiSD;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Setup
extern bool sdOK;

// Debug
extern uint32_t lastSdWrite; // Timestamp of previous SD Card write
extern uint32_t lastSdRead;  // Timestamp of previous SD Card read

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup SD Card module
String sdInit(void);

// Read configuration data from SD Card
String configNode(void);

// Used for SD Card module testing and node debugging
// Returns teh content as a String
String sdRead(String readFile);

// Write/Log contents to SD Card
// Used for node debugging and testing
void sdWrite(String content, String logFile);

// Writes and reads to the SD card
void sdDebug(void);

#endif // TVV_SD_H
