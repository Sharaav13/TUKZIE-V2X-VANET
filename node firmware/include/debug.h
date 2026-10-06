//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - DEBUG
//*************************************************************************************************************************************************

#ifndef TVV_DEBUG_H
#define TVV_DEBUG_H

#include "System.h"

//*************************************************************************************************************************************************
// MACROS
//*************************************************************************************************************************************************

// Peripherals to debug (argument of debug)
#define SD_CARD    0  // SD Card write and read
#define GPS        1  // GPS data
#define PPS_CLOCK  2  // PPS clock synchronisation error
#define IMU        3  // IMU data
#define MNS        4  // MNS UART Tx and Rx
#define LORA_RX    5  // LoRa receive only
#define LORA_TX    6  // LoRa transmit only
#define LORA       7  // LoRa transmit and receive

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Runs the debug routine of the selected peripheral
// peripheral: SD_CARD, GPS, PPS_CLOCK, IMU, MNS, LORA_RX, LORA_TX or LORA
void debug(int peripheral);

#endif // TVV_DEBUG_H
