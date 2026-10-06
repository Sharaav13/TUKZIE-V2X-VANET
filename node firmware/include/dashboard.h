//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - DASHBOARD (BLUETOOTH LOW ENERGY)
//*************************************************************************************************************************************************

#ifndef TVV_DASHBOARD_H
#define TVV_DASHBOARD_H

#include "system.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

// Bluetooth Low Energy
extern BLEServer *bleServer;
extern BLECharacteristic *bleTx;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Setup
extern bool dashboardOK;
extern bool dashConnected;
extern int pendingDeliveries;
extern float battery;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup Bluetooth Low Energy communication to TUKZIE dashboard
// Returns true if successful
bool dashInit(void);

// Transmits a table of nodes to the TUKZIE dashboard
// One node per line
// Returns false if no dashboard is connected
bool dashTx(Data table[], int size);

// Receives and parses a packet from the TUKZIE dashboard
// Packets are terminated by '\n'
// Returns true when a complete packet has been parsed
bool dashRx(void);

#endif // TVV_DASHBOARD_H
