import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;

namespace lorav2v;

//
// TVV VANET packet (firmware: struct Packet in include/vanet.h), after the dest byte.
//
// On air the firmware sends 25 bytes:
//   | dest | src | seq | rssi | message | date | time | lat | lon | status | tasks | battery |
//      1      1     2     1       1        4      4     4     4      1       1        1
//
// FLoRa's radio needs its own MAC header (flora::LoRaMacFrame) at the front of every frame. TvvMac makes it
// 1 byte long to stand for the dest byte, so this chunk is the other 24 bytes (TvvRouting.packetLength) and the
// frame is 25 bytes on air, as on the node. dest is still kept here, since TvvRouting filters on it.
//
// Field values are scaled as in the firmware (Packet::pack): lat/lon x 10^7.
// The chunk length is set by TvvRouting and does not depend on these fields.
//
class TvvPacket extends inet::FieldsChunk
{
    // Header
    uint8_t dest;           // destination ID, 0 = broadcast
    uint8_t src;            // sender ID
    uint16_t seq;           // sender's beacon count, increments on every Tx (wraps at 65535)
    int8_t rssi;            // sender's LoRa.rssi() (dBm) just before it sent
    uint8_t messageCode;    // message code (firmware field "message", a reserved word in .msg files)

    // Payload
    uint32_t date;          // DDMMYY
    uint32_t time;          // HHMMSSCC
    int32_t lat;            // deg x 10^7
    int32_t lon;            // deg x 10^7
    uint8_t status;         // status code
    uint8_t tasks;          // pending tasks
    uint8_t battery;        // %

    // Simulation only (not part of the 25 bytes): delay and distance statistics
    omnetpp::simtime_t creationTime;
    double positionX;       // m, OMNeT++ playground coordinates
    double positionY;
}
