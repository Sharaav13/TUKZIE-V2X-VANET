#ifndef __LORAV2V_ITVVBEACONSOURCE_H
#define __LORAV2V_ITVVBEACONSOURCE_H

#include "inet/common/packet/Packet.h"

namespace lorav2v {

/**
 * Builds the node's beacon at the moment TvvMac gets the channel.
 *
 * The firmware has no transmit queue: networkLoop() asks vanet.beaconDue(), and only then
 * copies myself and calls vanet.tx(), so every beacon carries the node's latest state.
 * TvvMac calls createBeacon() at that same point.
 */
class ITvvBeaconSource
{
  public:
    virtual ~ITvvBeaconSource() {}

    /**
     * Returns a new beacon (TvvPacket) ready for the MAC, not owned by any module.
     */
    virtual inet::Packet *createBeacon() = 0;
};

} // namespace lorav2v

#endif
