#include "LoRaBeaconApp.h"

#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/SignalTag_m.h"
#include "V2VBeacon_m.h"

namespace lorav2v {

Define_Module(LoRaBeaconApp);

simsignal_t LoRaBeaconApp::beaconSentSignal = registerSignal("beaconSent");
simsignal_t LoRaBeaconApp::beaconReceivedSignal = registerSignal("beaconReceived");
simsignal_t LoRaBeaconApp::rxDistanceSignal = registerSignal("rxDistance");
simsignal_t LoRaBeaconApp::txNeighborDistanceSignal = registerSignal("txNeighborDistance");
simsignal_t LoRaBeaconApp::rxPowerSignal = registerSignal("rxPower");

LoRaBeaconApp::~LoRaBeaconApp()
{
    cancelAndDelete(sendTimer);
}

void LoRaBeaconApp::initialize(int stage)
{
    cSimpleModule::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        payloadLength = B(par("payloadLength"));
        sendTimer = new cMessage("sendBeacon");
        WATCH(numSent);
        WATCH(numReceived);
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        mobility = check_and_cast<IMobility *>(getContainingNode(this)->getSubmodule("mobility"));
        scheduleAfter(par("startTime"), sendTimer);
    }
}

void LoRaBeaconApp::handleMessage(cMessage *msg)
{
    if (msg == sendTimer) {
        sendBeacon();
        scheduleAfter(par("beaconInterval"), sendTimer);
    }
    else
        processBeacon(check_and_cast<Packet *>(msg));
}

void LoRaBeaconApp::sendBeacon()
{
    cModule *host = getContainingNode(this);
    Coord position = mobility->getCurrentPosition();

    auto beacon = makeShared<V2VBeacon>();
    beacon->setSenderId(host->getId());
    beacon->setSequenceNumber(sequenceNumber);
    beacon->setCreationTime(simTime());
    beacon->setPositionX(position.x);
    beacon->setPositionY(position.y);
    beacon->setSpeed(mobility->getCurrentVelocity().length());
    beacon->setChunkLength(payloadLength);

    std::string name = "beacon-" + std::to_string(host->getIndex()) + "-" + std::to_string(sequenceNumber);
    auto packet = new Packet(name.c_str(), beacon);

    // distance to every other vehicle now: denominator for PDR vs. distance
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

    emit(beaconSentSignal, (intval_t)sequenceNumber);
    sequenceNumber++;
    numSent++;
    send(packet, "socketOut");
}

void LoRaBeaconApp::processBeacon(Packet *packet)
{
    const auto& beacon = packet->peekAtFront<V2VBeacon>();
    Coord senderPosition(beacon->getPositionX(), beacon->getPositionY());
    emit(beaconReceivedSignal, simTime() - beacon->getCreationTime());
    emit(rxDistanceSignal, mobility->getCurrentPosition().distance(senderPosition));
    if (auto signalPowerInd = packet->findTag<SignalPowerInd>())
        emit(rxPowerSignal, math::mW2dBmW(mW(signalPowerInd->getPower()).get()));
    numReceived++;
    delete packet;
}

void LoRaBeaconApp::finish()
{
    recordScalar("beaconsSent", numSent);
    recordScalar("beaconsReceived", numReceived);
}

} // namespace lorav2v
