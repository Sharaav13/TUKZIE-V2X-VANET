#include "LoRaCsmaMac.h"

#include "inet/common/ProtocolTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "LoRa/LoRaMacFrame_m.h"
#include "LoRa/LoRaRadio.h"
#include "LoRa/LoRaTagInfo_m.h"

namespace lorav2v {

using namespace inet::physicallayer;

Define_Module(LoRaCsmaMac);

simsignal_t LoRaCsmaMac::channelAccessDelaySignal = registerSignal("channelAccessDelay");
simsignal_t LoRaCsmaMac::backoffSlotsSignal = registerSignal("backoffSlots");
simsignal_t LoRaCsmaMac::radioDroppedPacketSignal = registerSignal("droppedPacket"); // emitted by flora::LoRaRadio

LoRaCsmaMac::~LoRaCsmaMac()
{
    cancelAndDelete(endDifs);
    cancelAndDelete(endBackoff);
}

void LoRaCsmaMac::initialize(int stage)
{
    MacProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        headerLength = B(par("headerLength"));
        slotTime = par("slotTime");
        difsTime = par("difsTime");
        cw = par("cw");
        centerFrequency = Hz(par("centerFrequency").doubleValue());
        bandwidth = Hz(par("bandwidth").doubleValue());
        spreadingFactor = par("spreadingFactor");
        codingRate = par("codingRate");
        txPowerDbm = par("txPower").doubleValue();
        // FLoRa's collision/capture table covers SF7..SF12
        if (spreadingFactor < 7 || spreadingFactor > 12)
            throw cRuntimeError("spreadingFactor must be in 7..12");
        if (codingRate < 1 || codingRate > 4)
            throw cRuntimeError("codingRate must be in 1..4 (4/5..4/8)");
        if (cw < 0)
            throw cRuntimeError("cw must be >= 0");

        endDifs = new cMessage("DIFS");
        endBackoff = new cMessage("Backoff");
        txQueue = getQueue(gate(upperLayerInGateId));

        WATCH(state);
        WATCH(backoffPeriod);
        WATCH(numSent);
        WATCH(numReceived);
        WATCH(numDeferred);
        WATCH(numBackoffFrozen);
        WATCH(numRxLost);
    }
    else if (stage == INITSTAGE_LINK_LAYER) {
        radio.reference(this, "radioModule", true);
        // FLoRa's receiver builds its listening (CF/BW/SF) from these fields
        auto loRaRadio = check_and_cast<flora::LoRaRadio *>(radio.get());
        loRaRadio->loRaCF = centerFrequency;
        loRaRadio->loRaBW = bandwidth;
        loRaRadio->loRaSF = spreadingFactor;
        loRaRadio->loRaCR = codingRate;
        loRaRadio->loRaTP = txPowerDbm;
        loRaRadio->loRaUseHeader = true;

        cModule *radioModule = check_and_cast<cModule *>(radio.get());
        radioModule->subscribe(IRadio::receptionStateChangedSignal, this);
        radioModule->subscribe(IRadio::transmissionStateChangedSignal, this);
        radioModule->subscribe(radioDroppedPacketSignal, this);
        radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
    }
}

void LoRaCsmaMac::finish()
{
    recordScalar("numSent", numSent);
    recordScalar("numReceived", numReceived);
    recordScalar("numDeferred", numDeferred);
    recordScalar("numBackoffFrozen", numBackoffFrozen);
    recordScalar("numRxLost", numRxLost);
}

void LoRaCsmaMac::configureNetworkInterface()
{
    MacAddress address = parseMacAddressParameter(par("address"));
    // nominal LoRa bit rate: SF * BW / 2^SF * 4 / (4 + CR)
    double bitrate = spreadingFactor * bandwidth.get() / (1 << spreadingFactor) * 4.0 / (4 + codingRate);
    networkInterface->setDatarate(bitrate);
    networkInterface->setMacAddress(address);
    networkInterface->setMtu(255); // max LoRa PHY payload
    networkInterface->setMulticast(true);
    networkInterface->setBroadcast(true);
    networkInterface->setPointToPoint(false);
}

void LoRaCsmaMac::handleSelfMessage(cMessage *msg)
{
    if (msg == endDifs)
        startBackoff();
    else if (msg == endBackoff)
        transmit();
    else
        throw cRuntimeError("Unknown self message: %s", msg->getName());
}

void LoRaCsmaMac::processUpperPacket()
{
    handleUpperPacket(dequeuePacket());
}

void LoRaCsmaMac::handleUpperPacket(Packet *packet)
{
    if (currentTxFrame != nullptr)
        throw cRuntimeError("Model error: frame received from upper layer while another is pending");
    encapsulate(packet);
    currentTxFrame = packet;
    accessStartTime = simTime();
    startContention();
}

void LoRaCsmaMac::encapsulate(Packet *packet)
{
    auto header = makeShared<flora::LoRaMacFrame>();
    header->setChunkLength(headerLength);
    header->setTransmitterAddress(networkInterface->getMacAddress());
    header->setReceiverAddress(MacAddress::BROADCAST_ADDRESS);
    header->setSequenceNumber(sequenceNumber++);
    header->setLoRaTP(txPowerDbm);
    header->setLoRaCF(centerFrequency);
    header->setLoRaSF(spreadingFactor);
    header->setLoRaBW(bandwidth);
    header->setLoRaCR(codingRate);
    header->setLoRaUseHeader(true);
    packet->insertAtFront(header);
    // INET radios copy this tag to received frames; same value as flora::LoRaMac
    packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::apskPhy);

    // flora::LoRaRadio turns this tag into the PHY preamble
    auto tag = packet->addTagIfAbsent<flora::LoRaTag>();
    tag->setCenterFrequency(centerFrequency);
    tag->setBandwidth(bandwidth);
    tag->setSpreadFactor(spreadingFactor);
    tag->setCodeRendundance(codingRate);
    tag->setPower(mW(math::dBmW2mW(txPowerDbm)));
    tag->setUseHeader(true);
}

void LoRaCsmaMac::startContention()
{
    if (isMediumFree())
        startDifs();
    else {
        numDeferred++;
        state = DEFER;
    }
}

void LoRaCsmaMac::startDifs()
{
    state = WAITDIFS;
    scheduleAfter(difsTime, endDifs);
}

void LoRaCsmaMac::startBackoff()
{
    state = BACKOFF;
    if (backoffPeriod < 0) {
        int slots = intrand(cw + 1);
        emit(backoffSlotsSignal, slots);
        backoffPeriod = slots * slotTime;
    }
    scheduleAfter(backoffPeriod, endBackoff);
}

void LoRaCsmaMac::freezeBackoff()
{
    // only fully elapsed idle slots count
    simtime_t elapsed = simTime() - endBackoff->getSendingTime();
    cancelEvent(endBackoff);
    backoffPeriod -= ((int)(elapsed / slotTime)) * slotTime;
    ASSERT(backoffPeriod >= 0);
    numBackoffFrozen++;
    state = DEFER;
}

void LoRaCsmaMac::transmit()
{
    state = TRANSMIT;
    backoffPeriod = -1;
    emit(channelAccessDelaySignal, simTime() - accessStartTime);
    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);
    Packet *frame = currentTxFrame;
    currentTxFrame = nullptr;
    numSent++;
    sendDown(frame);
}

void LoRaCsmaMac::handleMediumStateChange()
{
    bool free = isMediumFree();
    switch (state) {
        case DEFER:
            if (free)
                startDifs();
            break;
        case WAITDIFS:
            if (!free) {
                cancelEvent(endDifs);
                state = DEFER;
            }
            break;
        case BACKOFF:
            if (!free)
                freezeBackoff();
            break;
        default:
            break;
    }
}

bool LoRaCsmaMac::isMediumFree() const
{
    return radio->getReceptionState() == IRadio::RECEPTION_STATE_IDLE;
}

void LoRaCsmaMac::handleLowerPacket(Packet *packet)
{
    // flora::LoRaRadio already discards collided frames; anything here was decoded
    auto header = packet->popAtFront<flora::LoRaMacFrame>();
    auto addressInd = packet->addTagIfAbsent<MacAddressInd>();
    addressInd->setSrcAddress(header->getTransmitterAddress());
    addressInd->setDestAddress(header->getReceiverAddress());
    numReceived++;
    sendUp(packet);
}

void LoRaCsmaMac::receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details)
{
    Enter_Method("%s", cComponent::getSignalName(signalID));

    if (signalID == IRadio::receptionStateChangedSignal)
        handleMediumStateChange();
    else if (signalID == IRadio::transmissionStateChangedSignal) {
        auto newState = static_cast<IRadio::TransmissionState>(value);
        bool endOfTransmission = transmissionState == IRadio::TRANSMISSION_STATE_TRANSMITTING && newState == IRadio::TRANSMISSION_STATE_IDLE;
        transmissionState = newState;
        if (endOfTransmission) {
            state = IDLE;
            radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
            if (state == IDLE && currentTxFrame == nullptr && canDequeuePacket())
                processUpperPacket();
        }
    }
    else if (signalID == radioDroppedPacketSignal)
        numRxLost++;
}

queueing::IPassivePacketSource *LoRaCsmaMac::getProvider(cGate *gate)
{
    return (gate->getId() == upperLayerInGateId) ? txQueue.get() : nullptr;
}

void LoRaCsmaMac::handleCanPullPacketChanged(cGate *gate)
{
    Enter_Method("handleCanPullPacketChanged");
    if (state == IDLE && currentTxFrame == nullptr && canDequeuePacket())
        processUpperPacket();
}

void LoRaCsmaMac::handlePullPacketProcessed(Packet *packet, cGate *gate, bool successful)
{
    Enter_Method("handlePullPacketProcessed");
    throw cRuntimeError("Not supported callback");
}

} // namespace lorav2v
