import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;

namespace lorav2v;

//
// Broadcast beacon payload. The chunk length is set by the app
// (payloadLength), independent of these fields.
//
class V2VBeacon extends inet::FieldsChunk
{
    int senderId;                  // OMNeT++ module id of the sending vehicle
    uint32_t sequenceNumber;
    omnetpp::simtime_t creationTime;
    double positionX;
    double positionY;
    double speed;
}
