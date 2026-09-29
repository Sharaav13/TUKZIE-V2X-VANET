//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - PROTOCOL (FRAME STRUCTURE & PAYLOAD)
//*************************************************************************************************************************************************

#ifndef TVV_PROTOCOL_H
#define TVV_PROTOCOL_H

#include "System.h"

//*************************************************************************************************************************************************
// STRUCTURES
//*************************************************************************************************************************************************

// Contents of a frame
struct Frame {
  uint8_t  ver;
  char     type;
  uint8_t  node;
  uint16_t seq;
  float    lat;        // signed: negative = southern hemisphere
  float    lon;        // signed: negative = western hemisphere
  float    speed;      // km/h
  float    heading;    // degrees from north
  float    battery;    // TUKZIE battery percentage
  float    message;    // messages to send
};

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Creates the data payload in a CSV format
size_t createFrame(const Frame &f, char *out, size_t cap);

#endif // TVV_PROTOCOL_H
