//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - VANET (LoRa RADIO & ROUTING)
//*************************************************************************************************************************************************

#ifndef TVV_VANET_H
#define TVV_VANET_H

#include "system.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

extern SPIClass spiLoRa;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Rx packet handling flags
extern bool extract;
extern bool pass;
extern bool update;
extern bool add;

// LoRa
extern uint32_t lastLora;

// Routing
extern byte peers;
extern Data cluster[MAX_PEERS];

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup LoRa
// Returns true if successful
bool vanetInit(void);

// Procedure for receiving LoRa packet
// Returns the message, or -1 otherwise
int vanetRx(Data &buffer);

// Checks the ID
int analyseId(byte id, byte myId);

// Updates the peer information within a cluster
void updateCluster(Data buffer);

// Procedure for transmitting LoRa packet
void vanetTx(byte dest);

// Calculates the 3D straight line distance (km) between two GPS positions
// lat/lon in degrees, alt in metres above the ellipsoid
float calcDistance(Data node1, Data node2);

// Sets the Rx packet handling flags from the destination and source IDs
void handleRx(byte destId, byte srcId, byte myId);

// Resets all the flags
void resetRxFlags(void);

#endif // TVV_VANET_H
