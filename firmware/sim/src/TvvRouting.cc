#include "TvvRouting.h"

#include <algorithm>
#include <cmath>

#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/SignalTag_m.h"
#include "TvvPacket_m.h"

namespace lorav2v {

Define_Module(TvvRouting);

simsignal_t TvvRouting::beaconSentSignal = registerSignal("beaconSent");
simsignal_t TvvRouting::beaconReceivedSignal = registerSignal("beaconReceived");
simsignal_t TvvRouting::rxDistanceSignal = registerSignal("rxDistance");
simsignal_t TvvRouting::txNeighborDistanceSignal = registerSignal("txNeighborDistance");
simsignal_t TvvRouting::rxPowerSignal = registerSignal("rxPower");
simsignal_t TvvRouting::rxSnrSignal = registerSignal("rxSnr");
simsignal_t TvvRouting::clusterSizeSignal = registerSignal("clusterSize");
simsignal_t TvvRouting::beaconReceivedDirectSignal = registerSignal("beaconReceivedDirect");
simsignal_t TvvRouting::beaconReceivedForwardedSignal = registerSignal("beaconReceivedForwarded");
simsignal_t TvvRouting::heardDirectSignal = registerSignal("heardDirect");
simsignal_t TvvRouting::seqLostSignal = registerSignal("seqLost");
simsignal_t TvvRouting::addressedSentSignal = registerSignal("addressedSent");
simsignal_t TvvRouting::addressedReceivedSignal = registerSignal("addressedReceived");
simsignal_t TvvRouting::addressedUniqueSignal = registerSignal("addressedUnique");
simsignal_t TvvRouting::forwardedCopySignal = registerSignal("forwardedCopy");
simsignal_t TvvRouting::clusterFullSignal = registerSignal("clusterFull");
simsignal_t TvvRouting::peerAddedSignal = registerSignal("peerAdded");
simsignal_t TvvRouting::peerRemovedSignal = registerSignal("peerRemoved");

TvvRouting::~TvvRouting()
{
    cancelAndDelete(peerCheckTimer);
    for (auto packet : fwdQueue)
        delete packet;
}

void TvvRouting::initialize(int stage)
{
    cSimpleModule::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        nodeId = par("nodeId");
        if (nodeId < 0)
            nodeId = getContainingNode(this)->getIndex() + 1;     // 0 is the broadcast address
        if (nodeId < 1 || nodeId > 255)
            throw cRuntimeError("nodeId must be in 1..255 (got %d): 0 is broadcast and the ID is 1 byte", nodeId);
        destId = par("destId");
        message = par("message");
        packetLength = B(par("packetLength"));
        maxPeers = par("maxPeers");
        peerTimeout = par("peerTimeout");
        peerCheckInterval = par("peerCheckInterval");
        ttlInit = par("ttlInit");
        fwdQueueSize = par("fwdQueueSize");
        dupHistory = par("dupHistory");
        dupWindow = par("dupWindow");
        addressedFraction = par("addressedFraction");
        rxProcessingDelay = par("rxProcessingDelay");
        if (ttlInit < 1 || ttlInit > 255)
            throw cRuntimeError("ttlInit must be in 1..255 (1 byte, at least one hop)");
        if (fwdQueueSize < 0 || dupHistory < 0)
            throw cRuntimeError("fwdQueueSize and dupHistory must be >= 0");
        if (addressedFraction < 0 || addressedFraction > 1)
            throw cRuntimeError("addressedFraction must be in 0..1");

        peerCheckTimer = new cMessage("peerCheck");

        WATCH(nodeId);
        WATCH(txSeq);
        WATCH(peerMsg);
        WATCH(peerMsgId);
        WATCH(numSent);
        WATCH(numReceived);
        WATCH(numReceivedForwarded);
        WATCH(numSeqLost);
        WATCH(numDuplicates);
        WATCH(numForwardQueued);
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        mobility = check_and_cast<IMobility *>(getContainingNode(this)->getSubmodule("mobility"));
        forwardListener = check_and_cast<ITvvForwardListener *>(getModuleByPath(par("macModule").stringValue()));
        scheduleAfter(peerCheckInterval, peerCheckTimer);
    }
}

void TvvRouting::handleMessage(cMessage *msg)
{
    if (msg == peerCheckTimer) {
        int removed = removeStale();
        numPeersRemoved += removed;
        for (int i = 0; i < removed; i++)
            emit(peerRemovedSignal, 1);

        // The message shown is from a node not in my cluster (it left, or it was only forwarded to me)
        // and older than peerTimeout: clear it
        if (peerMsg != 0 && !isPeer(peerMsgId) && simTime() - peerMsgAt > peerTimeout)
            peerMsg = 0;

        emit(clusterSizeSignal, (intval_t)cluster.size());
        scheduleAfter(peerCheckInterval, peerCheckTimer);
    }
    else
        readPacket(check_and_cast<Packet *>(msg));
}

// ------------------------------------------------------------------------------------------------------------------------
// Tx
// ------------------------------------------------------------------------------------------------------------------------

Packet *TvvRouting::createBeacon()
{
    Enter_Method("createBeacon");

    Coord position = mobility->getCurrentPosition();

    // Vanet::tx: this beacon's sequence number, and the hops it may make
    uint16_t seq = txSeq + 1;       // wraps to 0 after 65535
    txSeq = seq;                    // TvvMac sends every beacon it asks for (beginPacket() cannot fail here)

    // Packet::pack from a copy of myself
    // Simulation traffic mix: a share of beacons addressed to one other node (as a host would set dest)
    int dest = destId;
    if (addressedFraction > 0 && uniform(0, 1) < addressedFraction) {
        int other = randomOtherNodeId();
        if (other > 0)
            dest = other;
    }
    if (dest != 0) {
        numAddressedSent++;
        emit(addressedSentSignal, 1);
    }

    auto pkt = makeShared<TvvPacket>();
    pkt->setDest(dest);
    pkt->setSrc(nodeId);
    pkt->setSeq(seq);
    pkt->setTtl(ttlInit);
    pkt->setMessageCode(message);
    pkt->setBattery(100);           // %
    pkt->setCreationTime(simTime());
    pkt->setPositionX(position.x);
    pkt->setPositionY(position.y);
    pkt->setChunkLength(packetLength);

    emitNeighborDistances(position);
    emit(beaconSentSignal, (intval_t)seq);
    numSent++;

    std::string name = "tvv-" + std::to_string(nodeId) + "-" + std::to_string(seq);
    auto packet = new Packet(name.c_str(), pkt);
    drop(packet);   // handed to TvvMac
    return packet;
}

bool TvvRouting::hasForward() const
{
    return !fwdQueue.empty();
}

Packet *TvvRouting::takeForward()
{
    Enter_Method("takeForward");

    if (fwdQueue.empty())
        throw cRuntimeError("takeForward() with nothing queued");
    Packet *packet = fwdQueue.front();
    fwdQueue.pop_front();
    drop(packet);   // handed to TvvMac
    return packet;
}

void TvvRouting::emitNeighborDistances(const Coord& position)
{
    // Distance to every other vehicle now: the denominator of PDR vs. distance (simulations/pdr.py)
    cModule *host = getContainingNode(this);
    cModule *network = host->getParentModule();
    int numNodes = network->getSubmoduleVectorSize(host->getName());
    for (int i = 0; i < numNodes; i++) {
        cModule *other = network->getSubmodule(host->getName(), i);
        if (other == nullptr || other == host)
            continue;
        auto otherMobility = dynamic_cast<IMobility *>(other->getSubmodule("mobility"));
        if (otherMobility != nullptr)
            emit(txNeighborDistanceSignal, position.distance(otherMobility->getCurrentPosition()));
    }
}

int TvvRouting::randomOtherNodeId()
{
    // Uniformly one of the other nodes of the same vector (their routing.nodeId)
    cModule *host = getContainingNode(this);
    cModule *network = host->getParentModule();
    int numNodes = network->getSubmoduleVectorSize(host->getName());
    if (numNodes < 2)
        return 0;
    int i = intuniform(0, numNodes - 2);
    if (i >= host->getIndex())
        i++;
    cModule *other = network->getSubmodule(host->getName(), i);
    auto otherRouting = other ? dynamic_cast<TvvRouting *>(other->getSubmodule("routing")) : nullptr;
    return otherRouting ? otherRouting->nodeId : 0;
}

// ------------------------------------------------------------------------------------------------------------------------
// Rx
// ------------------------------------------------------------------------------------------------------------------------

void TvvRouting::readPacket(Packet *packet)
{
    // TVV packets are always exactly this size
    if (packet->getDataLength() != packetLength) {     // after TvvMac removed its header
        numWrongSize++;
        delete packet;
        return;
    }
    const auto& pkt = packet->peekAtFront<TvvPacket>();

    // PDR statistics: every frame heard straight from its sender, whatever its dest
    if (pkt->getTtl() >= ttlInit)
        countSeq(pkt->getSrc(), pkt->getSeq());

    // Delay of every forwarded copy that reaches its destination, before the repeat check drops it
    if (pkt->getTtl() < ttlInit && pkt->getDest() == nodeId)
        emit(forwardedCopySignal, simTime() - pkt->getCreationTime() + rxProcessingDelay);

    // A repeat of a packet already read (e.g. the forwarded copy of one received direct): dropped
    if (seenBefore(pkt->getSrc(), pkt->getSeq())) {
        numDuplicates++;
        delete packet;
        return;
    }

    // For me: dest == my ID, or a broadcast (dest == 0). Any other dest: forward it as it is (not mine, so not read)
    bool forMe = pkt->getDest() == nodeId || pkt->getDest() == 0;
    if (!forMe) {
        queueForward(packet);
        return;
    }

    Peer peer;
    peer.id = pkt->getSrc();
    peer.lastHeard = simTime();
    peer.seq = pkt->getSeq();
    peer.message = pkt->getMessageCode();
    if (auto signalPowerInd = packet->findTag<SignalPowerInd>())
        peer.pktRssi = math::mW2dBmW(mW(signalPowerInd->getPower()).get());
    if (auto snirInd = packet->findTag<SnirInd>())
        peer.snr = math::fraction2dB(snirInd->getMinimumSnir());

    if (pkt->getDest() == nodeId) {
        numAddressedReceived++;
        emit(addressedReceivedSignal, 1);
        // A copy read again after it left the repeat check (dupWindow / dupHistory) is not a new delivery
        if (addressedSeen.insert({pkt->getSrc(), pkt->getSeq()}).second) {
            numAddressedUnique++;
            emit(addressedUniqueSignal, 1);
        }
    }

    // A sender heard directly is a peer (a forwarded packet has a lower ttl: its sender may be out of my range)
    bool direct = pkt->getTtl() >= ttlInit;
    if (direct) {
        bool known = isPeer(peer.id);
        if (!updateCluster(peer)) {
            numClusterFull++;
            emit(clusterFullSignal, 1);
        }
        else if (!known) {
            numPeersAdded++;
            emit(peerAddedSignal, 1);
        }
    }
    else
        numReceivedForwarded++;

    // Message of the latest received packet, for the display (0 = no message)
    peerMsg = peer.message;
    peerMsgId = peer.id;
    peerMsgAt = simTime();

    // Delay and distance from the original sender (end to end, also through a forwarder)
    Coord senderPosition(pkt->getPositionX(), pkt->getPositionY());
    simtime_t delay = simTime() - pkt->getCreationTime() + rxProcessingDelay;
    emit(beaconReceivedSignal, delay);
    emit(direct ? beaconReceivedDirectSignal : beaconReceivedForwardedSignal, delay);
    emit(rxDistanceSignal, mobility->getCurrentPosition().distance(senderPosition));
    emit(rxPowerSignal, peer.pktRssi);
    emit(rxSnrSignal, peer.snr);
    numReceived++;
    delete packet;
}

void TvvRouting::countSeq(int src, uint16_t seq)
{
    // Lost frames from gaps in the sender's seq (as tests/perf_analysis.m on the node logs). Counted on every
    // frame heard directly, also those addressed to another node, so addressed traffic does not show up as gaps.
    // A forwarded copy can arrive after a newer beacon, but forwarded copies are not counted here.
    numHeardDirect++;
    emit(heardDirectSignal, 1);
    auto last = lastSeq.find(src);
    if (last == lastSeq.end()) {
        lastSeq[src] = seq;
        return;
    }
    uint16_t gap = seq - last->second;      // uint16_t: correct across the 65535 wrap
    if (gap != 0 && gap < 0x8000) {
        numSeqLost += gap - 1;
        if (gap > 1)
            emit(seqLostSignal, (intval_t)(gap - 1));
        last->second = seq;
    }
}

bool TvvRouting::seenBefore(int src, uint16_t seq)
{
    simtime_t now = simTime();
    for (const auto& s : seen)
        if (s.src == src && s.seq == seq && now - s.at < dupWindow)
            return true;

    // New: remember it in place of the oldest
    if (dupHistory == 0)
        return false;
    Seen entry{src, seq, now};
    if ((int)seen.size() < dupHistory)
        seen.push_back(entry);
    else {
        seen[seenNext] = entry;
        seenNext = (seenNext + 1) % dupHistory;
    }
    return false;
}

void TvvRouting::queueForward(Packet *packet)
{
    const auto& pkt = packet->peekAtFront<TvvPacket>();

    // Decremented to 0 (or already 0): not sent
    if (pkt->getTtl() <= 1) {
        numTtlExpired++;
        delete packet;
        return;
    }
    if ((int)fwdQueue.size() >= fwdQueueSize) {
        numFwdQueueFull++;
        delete packet;
        return;
    }

    // Sent on exactly as received (same src, seq and payload), with ttl - 1
    auto copy = makeShared<TvvPacket>(*pkt);
    copy->setTtl(pkt->getTtl() - 1);
    std::string name = "tvv-" + std::to_string(pkt->getSrc()) + "-" + std::to_string(pkt->getSeq()) + "-fw" + std::to_string(nodeId);
    fwdQueue.push_back(new Packet(name.c_str(), copy));
    numForwardQueued++;
    delete packet;

    forwardListener->forwardQueued();
}

bool TvvRouting::isPeer(int id) const
{
    return std::any_of(cluster.begin(), cluster.end(), [&](const Peer& p) { return p.id == id; });
}

bool TvvRouting::updateCluster(const Peer& peer)
{
    auto it = std::find_if(cluster.begin(), cluster.end(), [&](const Peer& p) { return p.id == peer.id; });
    if (it != cluster.end()) {
        *it = peer;     // refresh an existing peer
        return true;
    }
    if ((int)cluster.size() >= maxPeers)
        return false;   // new peer, cluster full
    cluster.push_back(peer);
    return true;
}

int TvvRouting::removeStale()
{
    // Peers not heard for more than peerTimeout, i.e. out of range
    simtime_t now = simTime();
    auto stale = std::remove_if(cluster.begin(), cluster.end(), [&](const Peer& p) { return now - p.lastHeard > peerTimeout; });
    int removed = cluster.end() - stale;
    cluster.erase(stale, cluster.end());
    return removed;
}

void TvvRouting::refreshDisplay() const
{
    std::string text = "id " + std::to_string(nodeId) + ", peers:";
    if (cluster.empty())
        text += " none";
    for (const auto& peer : cluster)
        text += " " + std::to_string(peer.id);
    if (!fwdQueue.empty())
        text += ", fwd " + std::to_string(fwdQueue.size());
    getDisplayString().setTagArg("t", 0, text.c_str());
}

void TvvRouting::finish()
{
    recordScalar("nodeId", nodeId);
    recordScalar("beaconsSent", numSent);
    recordScalar("beaconsReceived", numReceived);
    recordScalar("beaconsReceivedForwarded", numReceivedForwarded);
    recordScalar("numWrongSize", numWrongSize);
    recordScalar("numClusterFull", numClusterFull);
    recordScalar("numHeardDirect", numHeardDirect);
    recordScalar("numSeqLost", numSeqLost);
    recordScalar("numAddressedSent", numAddressedSent);
    recordScalar("numAddressedReceived", numAddressedReceived);
    recordScalar("numAddressedUnique", numAddressedUnique);
    recordScalar("numPeersAdded", numPeersAdded);
    recordScalar("numPeersRemoved", numPeersRemoved);
    recordScalar("numDuplicates", numDuplicates);
    recordScalar("numForwardQueued", numForwardQueued);
    recordScalar("numTtlExpired", numTtlExpired);
    recordScalar("numFwdQueueFull", numFwdQueueFull);
}

} // namespace lorav2v
