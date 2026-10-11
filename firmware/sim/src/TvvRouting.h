#ifndef __LORAV2V_TVVROUTING_H
#define __LORAV2V_TVVROUTING_H

#include <map>
#include <vector>

#include "inet/common/INETDefs.h"
#include "inet/common/packet/Packet.h"
#include "inet/mobility/contract/IMobility.h"

#include "ITvvBeaconSource.h"

namespace lorav2v {

using namespace inet;

/**
 * Broadcast routing of the TVV node firmware (vanet.cpp).
 *
 * Single hop, no forwarding: every node broadcasts its own beacon and keeps a table of the peers it hears.
 *  - Tx (Packet::pack, Vanet::tx): one TvvPacket per beacon, seq = previous seq + 1, rssi = LoRa.rssi().
 *  - Rx (Vanet::readPacket):
 *      1. frames that are not exactly the TVV packet size are ignored,
 *      2. provision for destination based routing: only packets for me (dest == my ID) or broadcasts
 *         (dest == 0) are read,
 *      3. every sender is a peer: added to, or refreshed in, the cluster (Cluster::update); a new peer
 *         is not added when the cluster already holds maxPeers,
 *      4. the message of the latest packet is kept (peerMsg / peerMsgId).
 *  - Cluster::removeStale every peerCheckInterval drops peers not heard for more than peerTimeout.
 *
 * Statistics also count lost beacons from gaps in each sender's seq, as tests/rf_profiling.m does on the node logs.
 */
class TvvRouting : public cSimpleModule, public ITvvBeaconSource
{
  protected:
    struct Peer {
        int id = 0;
        simtime_t lastHeard;
        uint16_t seq = 0;
        double pktRssi = 0;     // dBm
        double snr = 0;         // dB
        int srcRssi = 0;        // dBm
        int message = 0;
    };

    // parameters
    int nodeId = 0;
    int destId = 0;
    int message = 0;
    int channelRssi = 0;
    B packetLength;
    int maxPeers = 0;
    simtime_t peerTimeout;
    simtime_t peerCheckInterval;

    IMobility *mobility = nullptr;
    cMessage *peerCheckTimer = nullptr;

    // firmware state
    uint16_t txSeq = 0;                 // sequence number of my last beacon
    std::vector<Peer> cluster;          // Cluster::peerTable
    int peerMsg = 0;                    // message of the latest received packet
    int peerMsgId = 0;                  // ID of the peer that sent it

    // statistics
    std::map<int, uint16_t> lastSeq;    // last seq heard from each sender
    long numSent = 0;
    long numReceived = 0;               // packets read (for me or broadcast)
    long numWrongSize = 0;
    long numNotForMe = 0;
    long numClusterFull = 0;            // packets from new peers that did not fit in the cluster
    long numSeqLost = 0;                // beacons missing between two received seqs
    long numDuplicates = 0;

    static simsignal_t beaconSentSignal;
    static simsignal_t beaconReceivedSignal;
    static simsignal_t rxDistanceSignal;
    static simsignal_t txNeighborDistanceSignal;
    static simsignal_t rxPowerSignal;
    static simsignal_t rxSnrSignal;
    static simsignal_t clusterSizeSignal;

  public:
    virtual ~TvvRouting();

    // ITvvBeaconSource
    virtual Packet *createBeacon() override;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual void refreshDisplay() const override;

    virtual void readPacket(Packet *packet);
    virtual bool updateCluster(const Peer& peer);    // Cluster::update
    virtual int removeStale();                       // Cluster::removeStale
    virtual void emitNeighborDistances(const Coord& position);
};

} // namespace lorav2v

#endif
