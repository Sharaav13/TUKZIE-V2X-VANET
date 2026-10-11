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

TvvRouting::~TvvRouting()
{
    cancelAndDelete(peerCheckTimer);
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
        channelRssi = (int)std::round(par("channelRssi").doubleValue());
        packetLength = B(par("packetLength"));
        maxPeers = par("maxPeers");
        peerTimeout = par("peerTimeout");
        peerCheckInterval = par("peerCheckInterval");

        peerCheckTimer = new cMessage("peerCheck");

        WATCH(nodeId);
        WATCH(txSeq);
        WATCH(peerMsg);
        WATCH(peerMsgId);
        WATCH(numSent);
        WATCH(numReceived);
        WATCH(numNotForMe);
        WATCH(numSeqLost);
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        mobility = check_and_cast<IMobility *>(getContainingNode(this)->getSubmodule("mobility"));
        scheduleAfter(peerCheckInterval, peerCheckTimer);
    }
}

void TvvRouting::handleMessage(cMessage *msg)
{
    if (msg == peerCheckTimer) {
        // The peer whose message is shown has left the cluster: clear its message
        if (removeStale() > 0 && std::none_of(cluster.begin(), cluster.end(), [&](const Peer& p) { return p.id == peerMsgId; }))
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

    // Vanet::tx: this beacon's sequence number and my channel RSSI
    uint16_t seq = txSeq + 1;       // wraps to 0 after 65535
    txSeq = seq;                    // TvvMac sends every beacon it asks for (beginPacket() cannot fail here)

    // Packet::pack from a copy of myself
    auto pkt = makeShared<TvvPacket>();
    pkt->setDest(destId);
    pkt->setSrc(nodeId);
    pkt->setSeq(seq);
    pkt->setRssi((int8_t)std::max(-128, std::min(127, channelRssi)));
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

    // Provision for destination based routing has been made:
    // only packets for me (dest == my ID) or broadcasts (dest == 0) are read
    if (pkt->getDest() != nodeId && pkt->getDest() != 0) {
        numNotForMe++;
        delete packet;
        return;
    }

    Peer peer;
    peer.id = pkt->getSrc();
    peer.lastHeard = simTime();
    peer.seq = pkt->getSeq();
    peer.srcRssi = pkt->getRssi();
    peer.message = pkt->getMessageCode();
    if (auto signalPowerInd = packet->findTag<SignalPowerInd>())
        peer.pktRssi = math::mW2dBmW(mW(signalPowerInd->getPower()).get());
    if (auto snirInd = packet->findTag<SnirInd>())
        peer.snr = math::fraction2dB(snirInd->getMinimumSnir());

    // Lost beacons from gaps in the sender's seq (as tests/rf_profiling.m on the node logs)
    auto last = lastSeq.find(peer.id);
    if (last != lastSeq.end()) {
        uint16_t gap = peer.seq - last->second;     // uint16_t: correct across the 65535 wrap
        if (gap == 0)
            numDuplicates++;
        else
            numSeqLost += gap - 1;
    }
    lastSeq[peer.id] = peer.seq;

    // Every sender is a peer
    if (!updateCluster(peer))
        numClusterFull++;

    // Message of the latest received packet, for the display (0 = no message)
    peerMsg = peer.message;
    peerMsgId = peer.id;

    Coord senderPosition(pkt->getPositionX(), pkt->getPositionY());
    emit(beaconReceivedSignal, simTime() - pkt->getCreationTime());
    emit(rxDistanceSignal, mobility->getCurrentPosition().distance(senderPosition));
    emit(rxPowerSignal, peer.pktRssi);
    emit(rxSnrSignal, peer.snr);
    numReceived++;
    delete packet;
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
    getDisplayString().setTagArg("t", 0, text.c_str());
}

void TvvRouting::finish()
{
    recordScalar("nodeId", nodeId);
    recordScalar("beaconsSent", numSent);
    recordScalar("beaconsReceived", numReceived);
    recordScalar("numWrongSize", numWrongSize);
    recordScalar("numNotForMe", numNotForMe);
    recordScalar("numClusterFull", numClusterFull);
    recordScalar("numSeqLost", numSeqLost);
    recordScalar("numDuplicates", numDuplicates);
}

} // namespace lorav2v
