//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - SYSTEM (SHARED GLOBAL VARIABLES)
//*************************************************************************************************************************************************

#include "System.h"

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Node congifuration
int config[CFG_FIELDS];

// System operation
bool error = false;
String errorMessage = "Errors in:\n";

// Payload variables
String message; // Make them volatile for interrupts.
// char messageTx[FRAME_CSV_MAX];
// char messageRx[FRAME_CSV_MAX];
