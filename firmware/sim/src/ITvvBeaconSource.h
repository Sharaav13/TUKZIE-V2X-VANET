#ifndef __LORAV2V_ITVVBEACONSOURCE_H
#define __LORAV2V_ITVVBEACONSOURCE_H

#include "inet/common/packet/Packet.h"

namespace lorav2v {

/**
 * Builds the node's beacon at the moment TvvMac gets the channel, and hands over packets queued for forwarding.
 *
 * The firmware has no beacon queue: v2vLoop() asks vanet.beaconDue(), and only then
 * copies myself and calls vanet.tx(), so every beacon carries the node's latest state.
 * TvvMac calls createBeacon() at that same point.
 *
 * Forwarding: vanet.readPacket() queues packets addressed to other nodes, and v2vLoop() sends the oldest
 * (vanet.forwardDue(), vanet.forward()) before checking the beacon, in the same transmission slot. TvvMac calls
 * takeForward() instead of createBeacon() when a slot comes up with a forward queued.
 */
class ITvvBeaconSource
{
  public:
    virtual ~ITvvBeaconSource() {}

    /**
     * Returns a new beacon (TvvPacket) ready for the MAC, not owned by any module.
     */
    virtual inet::Packet *createBeacon() = 0;

    /**
     * Returns true if a packet is queued for forwarding.
     */
    virtual bool hasForward() const = 0;

    /**
     * Removes the oldest queued packet (TvvPacket, ttl already decremented) and returns it, not owned by any module.
     */
    virtual inet::Packet *takeForward() = 0;
};

/**
 * Told by the routing when a packet is queued for forwarding. TvvMac sends it in its next transmission slot.
 */
class ITvvForwardListener
{
  public:
    virtual ~ITvvForwardListener() {}

    virtual void forwardQueued() = 0;
};

} // namespace lorav2v

#endif
