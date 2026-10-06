//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - VANET (LoRa RADIO & ROUTING)
//*************************************************************************************************************************************************

#include "system.h"
#include "vanet.h"

// Discard or retransmit if low snr and high freq error

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

SPIClass spiLoRa(HSPI);

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Routing
// migration of all glbal varibles into a module called data
// Next hop problem discuss in report ~ I get a pkt that was forawrded by a peer but dest!=me and src!=peers
// Dash board send checks if teh node fields are empty or not before sending
// need to have ttl?

// Can only be on CAD, TX and RX at a time, cannot do simultaneously
// have a dedicated RX and TX lora module

// destination ID
// ttl
// date
// time
// src ID
// primary src ID
// core message

/*
SENDING A PACKET TO A SPECIFIC NODE
1. Send paket with the above fields

RECIEVING A PACKET FOR A SPECIFIC NODE
if (dest != me but dest is within my routing table) {
    make recived packet payload and decrease its ttl;
    send new packet with this payload;
}

ignore ttl of 1

if (dest == me) {
    parse packet
    interpret packet
    send info to appropriate places
}

need some sort of buffer for these messages


*/

// Converts the Link state table into a string

// Converts the entire routing table into a string

// Implelment ttl
// Change ints to bytes where appropriate

bool extract = true;
bool pass = false;
bool update = false;
bool add = false;

uint32_t lastLora = 0;

int peers = 0;
Data cluster[MAX_PEERS];

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup LoRa
// Returns true if successful
bool vanetInit(void) {

  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  LoRa.setSPI(spiLoRa);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(LORA_SPI_FREQ);        // 1E6 for bextractboarding otherwise 8E6
  
  if (LoRa.begin(LORA_FREQ)) {
    LoRa.setSpreadingFactor(LORA_SF); // 12
    LoRa.setSignalBandwidth(LORA_BW); //125E3
    LoRa.setCodingRate4(LORA_CR);     // 8
    LoRa.setSyncWord(LORA_SYNC);      // 0x12 for private network
    LoRa.setTxPower(LORA_TXPOWER, PA_OUTPUT_PA_BOOST_PIN);    // PA_BOOST output on the RA-02
    // LoRa.setPreambleLength(LORA_PREAMBLE);
    LoRa.enableCrc();
    
    randomSeed(esp_random());
    lastLora = millis() - random(0, BEACON_INTVL); // Prevents sychronous initail transmission
    
    return true;

  } else {
    error = true;
    errMsg += "LoRa: radio not detected\n";
    return false;
  }
}

// Procedure for receiving LoRa packet
// Returns the message, or -1 otherwise
int vanetRx(Data &buffer) {
  
  if (LoRa.parsePacket()) {
    
    byte dest = LoRa.read();
    byte src = LoRa.read();

    handleRx(dest, src, myself.id);

    if (!extract) { // Drop the packet before futher processing
      return -1;
    } 
    buffer.fleetId = LoRa.read();
    buffer.message = LoRa.read();
    
    byte incomingLen = LoRa.read();
    String incoming = "";
    while (LoRa.available()) {
      incoming += String((char)LoRa.read());
    }
    if (incomingLen != incoming.length()) {   
      return -1;  // Error in payload length not matching                           
    }

    if (pass) {
      buffer.payload = incoming; // Save further processing if teh packet must be passed.
      return -1;
    }
    int got = sscanf(incoming.c_str(), // Parse incoming payload
    "%d,"                   // destination ID
    "%d,"                   // source ID
    "%" SCNu32 ","          // date (DDMMYY)
    "%" SCNu32 ","          // time (HHMMSSCC)
    "%f,"                   // lat (deg)
    "%f,"                   // lon (deg)
    "%f,"                   // alt (m)
    "%f,"                   // speed (kmph)
    "%f,"                   // heading/course (deg bearing)
    "%d,"                   // status
    "%d,"                   // pending deliveries
    "%f,",                   // battery percentage
    &buffer.dest,
    &buffer.id,
    &buffer.date,
    &buffer.time,
    &buffer.lat, &buffer.lon, &buffer.alt,
    &buffer.speed, &buffer.course,
    &buffer.status,
    &buffer.tasks,
    &buffer.battery);
    if (got == 13) {
      return -1; // Not all fields of the payload were received.
    }

    buffer.rssi = LoRa.packetRssi();
    buffer.snr =  LoRa.packetSnr();
    buffer.freqErr = LoRa.packetFrequencyError();

    updateCluster(buffer);
    return (int)buffer.message;  
  } else {
    return -1;
  }
  resetRxFlags(); 
}

// Checks the ID
int analyseId(byte id, byte myId) {
  if ((id == myId) || (id == 0)) {
    return 1;
  } else {
    for (int i = 0; i < MAX_PEERS; i++) {
      Data peer = cluster[i];
      if (peer.id == id) {
        return 2;
      }
    }
  }
  return 0;
}

// Updates the peer information within a cluster
void updateCluster(Data buffer) {
  
  int j = peers + 1;
  for (int i = 0; i < MAX_PEERS; i++) {
    Data peer = cluster[i];
    if (peer.id == buffer.id) {
      j = i;
    }
  }
  if (j = peers + 1) {
    peers++;
  }
  if (j = MAX_PEERS + 1) {
    return;
  }
  
 if (add || update) {
    Data peer = cluster[j];
    peer.id = buffer.id;
    peer.fleetId = buffer.fleetId;
    peer.lat = buffer.lat;
    peer.lon = buffer.lat;
    peer.alt = buffer.lat;
    peer.speed = buffer.lat;
    peer.course = buffer.lat;
    peer.tasks = buffer.lat;
    peer.status = buffer.lat;
    peer.message = buffer.lat;
    peer.battery = buffer.lat;
    peer.date = buffer.lat;
    peer.time = buffer.lat;
    peer.rssi = buffer.lat;
    peer.snr = buffer.lat;
    peer.freqErr = buffer.lat;
    // peer.distance = calcDistance(peer, buffer);
  }
}

// Procedure for transmitting LoRa packet
// Returns true if the packet was sent
void vanetTx(byte dest) {
  if (!LoRa.beginPacket()) {
    return; // radio busy
  } else {
    LoRa.write(dest);              // add destination address
    LoRa.write(myself.id);               // add sender address
    LoRa.write(myself.fleetId);             // add fleet ID
    LoRa.write(myself.message);             // add message

    // Create payload
    char payload[128];
    int n = snprintf(payload, sizeof(payload),
    "%06" PRIu32 ","        // date (DDMMYY)
    "%08" PRIu32 ","        // time (HHMMSSCC)
    "%.4f,"                 // lat (deg)
    "%.4f,"                 // lon (deg)
    "%.1f,"                 // alt (m)
    "%.1f,"                 // speed (kmph)
    "%.1f,"                 // heading/course (deg bearing)
    "%d,"                   // status
    "%d,"                   // pending drop offs
    "%.2f,",                // battery percentage
    myself.date,
    myself.time,
    myself.lat, myself.lon, myself.alt,
    myself.speed, myself.course,
    myself.status,
    myself.tasks,
    myself.battery);
    size_t len = (n < (int)sizeof(payload)) ? n : sizeof(payload) - 1;   // length actually stored

    LoRa.write((uint8_t)len);                     // add payload length
    LoRa.write((const uint8_t *)payload, len);    // add payload
    LoRa.endPacket();                             // waits until the packet is sent (~30-100 ms at SF7)


  }
}

// Calculates the 3D straight line distance (m) between two GPS positions
// lat/lon in degrees, alt in metres above the ellipsoid
// Converts both positions to Earth-Centred, Earth-Fixed (ECEF) coordinates on the WGS84 ellipsoid
float calcDistance(Data node1, Data node2) {

  double x[2], y[2], z[2];
  double lat[2] = {node1.lat * DEG_TO_RAD, node2.lat * DEG_TO_RAD};
  double lon[2] = {node1.lon * DEG_TO_RAD, node2.lon * DEG_TO_RAD};
  double alt[2] = {node1.alt, node2.alt};

  for (int i = 0; i < 2; i++) {
    double sinLat = sin(lat[i]);
    double cosLat = cos(lat[i]);
    double n = WGS84_A / sqrt(1.0 - WGS84_E2 * sinLat * sinLat);   // prime vertical radius of curvature

    x[i] = (n + alt[i]) * cosLat * cos(lon[i]);
    y[i] = (n + alt[i]) * cosLat * sin(lon[i]);
    z[i] = (n * (1.0 - WGS84_E2) + alt[i]) * sinLat;
  }

  double dx = x[1] - x[0];
  double dy = y[1] - y[0];
  double dz = z[1] - z[0];

  double result = sqrt(dx * dx + dy * dy + dz * dz) / 1000.0;

  return (float)result;
}

void handleRx(byte destId, byte srcId, byte myId) {
  
  int d = analyseId(destId, myId);
  int s = analyseId(srcId, myId);
  
  switch (d) {
    case 0:
      if (s != 0) {
        pass = true;
      } else {
        extract = false;
      }
      break;
    case 1:
      if (s == 0) {
        add = true;
      }
      break;
    case 2:
      pass = true;
      break;
  }

  if (s == 2) {
    update = true;
  } 
}

// Resets all the flags
void resetRxFlags(void) {
  extract = true;
  pass = false;
  update = false;
  add = false;
}

// Counts the number of cluster peers
int countPeers(void) {
  return peers;
}