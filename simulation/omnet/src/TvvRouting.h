#ifndef __LORAV2V_TVVROUTING_H
#define __LORAV2V_TVVROUTING_H

#include <deque>
#include <map>
#include <set>
#include <vector>

#include "inet/common/INETDefs.h"
#include "inet/common/packet/Packet.h"
#include "inet/mobility/contract/IMobility.h"

#include "ITvvBeaconSource.h"

namespace lorav2v {

using namespace inet;

/**
 * Routing of the TVV node firmware (vanet.cpp): broadcast beacons, a cluster of peers, and one-hop forwarding.
 *
 *  - Tx (Packet::pack, Vanet::tx): one TvvPacket per beacon, seq = previous seq + 1, ttl = ttlInit.
 *  - Rx (Vanet::readPacket):
 *      1. frames that are not exactly the TVV packet size are ignored,
 *      2. a repeat of a packet (src, seq) already read within dupWindow (last dupHistory packets) is dropped,
 *      3. packets for me (dest == my ID) and broadcasts (dest == 0) are read; a packet addressed to any other node
 *         is queued to be forwarded exactly as received, with ttl - 1
 *         (not queued at ttl 0, or when the queue holds fwdQueueSize); TvvMac sends it (ITvvBeaconSource),
 *      4. a packet for me heard straight from its sender (ttl == ttlInit) adds or refreshes the sender in the
 *         cluster (Cluster::update; a new peer is not added when the cluster already holds maxPeers); a packet
 *         forwarded to me (lower ttl) is read, but its sender is not a peer (it may be out of my range),
 *      5. the message of the latest packet is kept (peerMsg / peerMsgId).
 *  - Every peerCheckInterval: Cluster::removeStale drops peers not heard for more than peerTimeout, and the
 *    message shown is cleared once its sender is not in the cluster and it is older than peerTimeout.
 *
 * Statistics also count lost beacons from gaps in each sender's seq (every frame heard directly, any dest), as
 * tests/perf_analysis.m does on the node logs.
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
        int message = 0;
    };

    struct Seen {               // a packet read, for dropping repeats
        int src = 0;
        uint16_t seq = 0;
        simtime_t at;
    };

    // parameters
    int nodeId = 0;
    int destId = 0;
    int message = 0;
    B packetLength;
    int maxPeers = 0;
    simtime_t peerTimeout;
    simtime_t peerCheckInterval;
    int ttlInit = 0;
    int fwdQueueSize = 0;
    int dupHistory = 0;
    simtime_t dupWindow;
    double addressedFraction = 0;       // share of beacons addressed to a random other node (simulation traffic mix)
    simtime_t rxProcessingDelay;        // added to the delay statistics: node time from RxDone to the timestamp

    IMobility *mobility = nullptr;
    ITvvForwardListener *forwardListener = nullptr;     // TvvMac
    cMessage *peerCheckTimer = nullptr;

    // firmware state
    uint16_t txSeq = 0;                 // sequence number of my last beacon
    std::vector<Peer> cluster;          // Cluster::peerTable
    int peerMsg = 0;                    // message of the latest received packet
    int peerMsgId = 0;                  // ID of the node that sent it
    simtime_t peerMsgAt;                // when it was received
    std::deque<Packet *> fwdQueue;      // Vanet::fwdQueue, oldest first
    std::vector<Seen> seen;             // Vanet::seen, at most dupHistory
    int seenNext = 0;                   // slot for the next packet once full (oldest replaced first)

    // statistics
    std::map<int, uint16_t> lastSeq;    // newest seq heard from each sender
    long numSent = 0;
    long numReceived = 0;               // packets read for me (broadcast or my ID), direct or forwarded
    long numReceivedForwarded = 0;      // of them, received through a forwarder (ttl < ttlInit)
    long numWrongSize = 0;
    long numClusterFull = 0;            // packets from new peers that did not fit in the cluster
    long numHeardDirect = 0;            // frames heard straight from their sender (ttl == ttlInit), any dest
    long numSeqLost = 0;                // of those senders' frames, missing between two heard seqs
    long numAddressedSent = 0;          // my beacons addressed to one node (dest != 0)
    long numAddressedReceived = 0;      // packets addressed to me read (direct or forwarded)
    long numAddressedUnique = 0;        // of them, the first copy of each (src, seq)
    std::set<std::pair<int, uint16_t>> addressedSeen;   // simulation only: (src, seq) addressed to me already read
    long numPeersAdded = 0;             // peers added to the cluster
    long numPeersRemoved = 0;           // peers dropped by removeStale
    long numDuplicates = 0;             // repeats dropped
    long numForwardQueued = 0;          // packets for another node queued to be forwarded
    long numTtlExpired = 0;             // packets for another node not forwarded: ttl would reach 0
    long numFwdQueueFull = 0;           // packets for another node not forwarded: queue full

    static simsignal_t beaconSentSignal;
    static simsignal_t beaconReceivedSignal;
    static simsignal_t rxDistanceSignal;
    static simsignal_t txNeighborDistanceSignal;
    static simsignal_t rxPowerSignal;
    static simsignal_t rxSnrSignal;
    static simsignal_t clusterSizeSignal;
    static simsignal_t beaconReceivedDirectSignal;
    static simsignal_t beaconReceivedForwardedSignal;
    static simsignal_t heardDirectSignal;
    static simsignal_t seqLostSignal;
    static simsignal_t addressedSentSignal;
    static simsignal_t addressedReceivedSignal;
    static simsignal_t addressedUniqueSignal;
    static simsignal_t forwardedCopySignal;
    static simsignal_t clusterFullSignal;
    static simsignal_t peerAddedSignal;
    static simsignal_t peerRemovedSignal;

  public:
    virtual ~TvvRouting();

    // ITvvBeaconSource
    virtual Packet *createBeacon() override;
    virtual bool hasForward() const override;
    virtual Packet *takeForward() override;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual void refreshDisplay() const override;

    virtual void readPacket(Packet *packet);
    virtual bool seenBefore(int src, uint16_t seq);     // Vanet::seenBefore
    virtual void queueForward(Packet *packet);         // Vanet::queueForward
    virtual bool isPeer(int id) const;                  // Cluster::find >= 0
    virtual bool updateCluster(const Peer& peer);       // Cluster::update
    virtual int removeStale();                          // Cluster::removeStale
    virtual void emitNeighborDistances(const Coord& position);
    virtual int randomOtherNodeId();
    virtual void countSeq(int src, uint16_t seq);
};

} // namespace lorav2v

#endif
