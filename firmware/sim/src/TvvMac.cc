#include "TvvMac.h"

#include "inet/common/ProtocolTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "LoRa/LoRaMacFrame_m.h"
#include "LoRa/LoRaRadio.h"
#include "LoRa/LoRaTagInfo_m.h"

namespace lorav2v {

using namespace inet::physicallayer;

Define_Module(TvvMac);

simsignal_t TvvMac::channelAccessDelaySignal = registerSignal("channelAccessDelay");
simsignal_t TvvMac::backoffDurationSignal = registerSignal("backoffDuration");
simsignal_t TvvMac::txIntervalSignal = registerSignal("txInterval");
simsignal_t TvvMac::radioDroppedPacketSignal = registerSignal("droppedPacket"); // emitted by flora::LoRaRadio
simsignal_t TvvMac::rxLostSignal = registerSignal("rxLost");
simsignal_t TvvMac::rxOkSignal = registerSignal("rxOk");
simsignal_t TvvMac::frameSentSignal = registerSignal("frameSent");

TvvMac::~TvvMac()
{
    cancelAndDelete(beaconTimer);
}

void TvvMac::initialize(int stage)
{
    MacProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        headerLength = B(par("headerLength"));
        centerFrequency = Hz(par("centerFrequency").doubleValue());
        bandwidth = Hz(par("bandwidth").doubleValue());
        spreadingFactor = par("spreadingFactor");
        codingRate = par("codingRate");
        txPowerDbm = par("txPower").doubleValue();
        beaconInterval = par("beaconInterval");
        backoffMinMs = (int)std::round(par("backoffMin").doubleValue() * 1000);
        backoffMaxMs = (int)std::round(par("backoffMax").doubleValue() * 1000);
        pollInterval = par("pollInterval");
        csDetectionTime = par("csDetectionTime");

        std::string carrierSense = par("carrierSense").stdstringValue();
        if (carrierSense == "preamble")
            senseEnergy = false;
        else if (carrierSense == "energy")
            senseEnergy = true;
        else
            throw cRuntimeError("carrierSense must be \"preamble\" or \"energy\"");

        // FLoRa's collision/capture table covers SF7..SF12
        if (spreadingFactor < 7 || spreadingFactor > 12)
            throw cRuntimeError("spreadingFactor must be in 7..12");
        if (codingRate < 1 || codingRate > 4)
            throw cRuntimeError("codingRate must be in 1..4 (4/5..4/8)");
        if (backoffMinMs < 0 || backoffMaxMs < backoffMinMs)
            throw cRuntimeError("Need 0 <= backoffMin <= backoffMax");
        if (beaconInterval < SimTime(1, SIMTIME_MS) || pollInterval <= SIMTIME_ZERO)
            throw cRuntimeError("beaconInterval must be >= 1 ms and pollInterval > 0");

        beaconTimer = new cMessage("txDue");

        WATCH(state);
        WATCH(lastTx);
        WATCH(backoffEnd);
        WATCH(numSent);
        WATCH(numForwarded);
        WATCH(transmitting);
        WATCH(numReceived);
        WATCH(numRxLost);
        WATCH(numBackoffs);
        WATCH(numBlockedByBackoff);
        WATCH(numBlockedByReceiving);
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

        // Before subscribing: the radio's state signals (also from setRadioMode below) ask it for queued forwards
        beaconSource = check_and_cast<ITvvBeaconSource *>(getModuleByPath(par("beaconSourceModule").stringValue()));

        cModule *radioModule = check_and_cast<cModule *>(radio.get());
        radioModule->subscribe(IRadio::receptionStateChangedSignal, this);
        radioModule->subscribe(IRadio::transmissionStateChangedSignal, this);
        radioModule->subscribe(radioDroppedPacketSignal, this);
        radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);   // LoRa.receive(): continuous Rx

        // init(): lastTx = millis() - random(0, BEACON_INTVL), random first beacon so nodes don't sync
        int intervalMs = (int)std::round(beaconInterval.dbl() * 1000);
        lastTx = simTime() - SimTime(intuniform(0, intervalMs - 1), SIMTIME_MS);
        backoffEnd = simTime();
        scheduleNextBeacon();
    }
}

void TvvMac::finish()
{
    recordScalar("numSent", numSent);
    recordScalar("numForwarded", numForwarded);
    recordScalar("numReceived", numReceived);
    recordScalar("numRxLost", numRxLost);
    recordScalar("numBackoffs", numBackoffs);
    recordScalar("numBlockedByBackoff", numBlockedByBackoff);
    recordScalar("numBlockedByReceiving", numBlockedByReceiving);
}

void TvvMac::configureNetworkInterface()
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

void TvvMac::refreshDisplay() const
{
    static const char *stateNames[] = {"waiting", "channel?", "Tx"};
    getDisplayString().setTagArg("t", 0, transmitting && !transmittingBeacon ? "forwarding" : stateNames[state]);
}

// ------------------------------------------------------------------------------------------------------------------------
// Tx
// ------------------------------------------------------------------------------------------------------------------------

void TvvMac::scheduleNextBeacon()
{
    state = WAIT_INTERVAL;
    rescheduleAt(lastTx + beaconInterval, beaconTimer);
}

void TvvMac::handleSelfMessage(cMessage *msg)
{
    if (msg != beaconTimer)
        throw cRuntimeError("Unknown self message: %s", msg->getName());

    // BEACON_INTVL has passed: a transmission (beacon or forward) is due from now on
    if (state == WAIT_INTERVAL) {
        state = WAIT_CHANNEL;
        dueTime = simTime();
        blockedByBackoff = false;
        blockedByReceiving = false;
    }
    tryTransmit();
}

void TvvMac::tryTransmit()
{
    // A beacon or forward is on air: asked again when it ends
    if (transmitting)
        return;

    // Still backing off after a received packet: ask again when it ends
    if (backoffActive()) {
        if (!blockedByBackoff) {
            numBlockedByBackoff++;
            blockedByBackoff = true;
        }
        rescheduleAt(backoffEnd, beaconTimer);
        return;
    }

    // Another node is transmitting right now: asked again when the reception state changes
    if (receiving()) {
        if (!blockedByReceiving) {
            numBlockedByReceiving++;
            blockedByReceiving = true;
        }
        return;
    }

    // The slot is used: v2vLoop() checks forwardDue() before beaconDue(), so a queued forward takes it and the
    // beacon waits for the next slot
    state = TRANSMIT;
    emit(channelAccessDelaySignal, simTime() - dueTime);
    if (beaconSource->hasForward())
        transmitForward();
    else
        transmit();
}

void TvvMac::transmit()
{
    // v2vLoop(): copyMyself(), then vanet.tx() packs and sends it
    Packet *packet = beaconSource->createBeacon();
    take(packet);
    numSent++;

    transmittingBeacon = true;
    sendFrame(packet);
}

void TvvMac::forwardQueued()
{
    Enter_Method("forwardQueued");

    // Nothing to do: the forward waits for the next transmission slot (forwardDue() needs txDue())
}

void TvvMac::transmitForward()
{
    // vanet.forward(): the oldest queued packet, exactly as queued (ttl already decremented)
    Packet *packet = beaconSource->takeForward();
    take(packet);
    numForwarded++;

    transmittingBeacon = false;
    sendFrame(packet);
}

void TvvMac::sendFrame(Packet *packet)
{
    if (lastTxStart >= SIMTIME_ZERO)
        emit(txIntervalSignal, simTime() - lastTxStart);
    lastTxStart = simTime();
    emit(frameSentSignal, transmittingBeacon ? 0 : 1);

    encapsulate(packet);
    transmitting = true;
    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);
    sendDown(packet);
}

void TvvMac::encapsulate(Packet *packet)
{
    // FLoRa's radio and medium need a LoRaMacFrame at the front: 1 B, standing for the dest byte.
    // Its receiver address is always broadcast: like the SX127x, every radio receives every frame,
    // and TvvRouting applies the firmware's dest filter.
    auto header = makeShared<flora::LoRaMacFrame>();
    header->setChunkLength(headerLength);
    header->setTransmitterAddress(networkInterface->getMacAddress());
    header->setReceiverAddress(MacAddress::BROADCAST_ADDRESS);
    header->setLoRaTP(txPowerDbm);
    header->setLoRaCF(centerFrequency);
    header->setLoRaSF(spreadingFactor);
    header->setLoRaBW(bandwidth);
    header->setLoRaCR(codingRate);
    header->setLoRaUseHeader(true);
    packet->insertAtFront(header);
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

void TvvMac::handleUpperPacket(Packet *packet)
{
    // The firmware has no transmit queue: beacons are built by ITvvBeaconSource when the channel is free
    throw cRuntimeError("TvvMac builds its own beacons, nothing should arrive from the upper layer (%s)", packet->getName());
}

// ------------------------------------------------------------------------------------------------------------------------
// Rx and channel state
// ------------------------------------------------------------------------------------------------------------------------

void TvvMac::startBackoff()
{
    // random(BACKOFF_MIN_MS, BACKOFF_MAX_MS + 1): whole ms, max included; replaces a running back-off
    simtime_t duration = SimTime(intuniform(backoffMinMs, backoffMaxMs), SIMTIME_MS);
    backoffEnd = simTime() + duration;
    numBackoffs++;
    emit(backoffDurationSignal, duration);
}

bool TvvMac::backoffActive() const
{
    return simTime() < backoffEnd;
}

bool TvvMac::receiving() const
{
    // RECEIVING: the radio is decoding a frame above sensitivity (LoRaV2VReceiver), like RegModemStat
    // "signal detected / synchronised / header valid". BUSY: energy above energyDetection only.
    // With csDetectionTime > 0 the frame is only seen once that much of its preamble has arrived, as the
    // SX127x needs a few preamble symbols before it sets "signal detected"
    auto receptionState = radio->getReceptionState();
    if (receptionState == IRadio::RECEPTION_STATE_RECEIVING)
        return simTime() - receptionStart >= csDetectionTime;
    return senseEnergy && receptionState == IRadio::RECEPTION_STATE_BUSY;
}

void TvvMac::handleLowerPacket(Packet *packet)
{
    // RxDone (CRC valid): rx() starts the back-off before reading the packet
    numReceived++;
    emit(rxOkSignal, 1);
    startBackoff();

    auto header = packet->popAtFront<flora::LoRaMacFrame>();
    auto addressInd = packet->addTagIfAbsent<MacAddressInd>();
    addressInd->setSrcAddress(header->getTransmitterAddress());
    addressInd->setDestAddress(header->getReceiverAddress());
    sendUp(packet);
}

void TvvMac::receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details)
{
    Enter_Method("%s", cComponent::getSignalName(signalID));

    if (signalID == IRadio::receptionStateChangedSignal) {
        bool nowReceiving = static_cast<IRadio::ReceptionState>(value) == IRadio::RECEPTION_STATE_RECEIVING;
        if (nowReceiving && !wasReceiving)
            receptionStart = simTime();
        wasReceiving = nowReceiving;

        // A due transmission is checked again on the next v2vLoop() pass
        if (!transmitting && !receiving() && state == WAIT_CHANNEL && !beaconTimer->isScheduled())
            scheduleAfter(pollInterval, beaconTimer);
    }
    else if (signalID == IRadio::transmissionStateChangedSignal) {
        auto newState = static_cast<IRadio::TransmissionState>(value);
        bool endOfTransmission = transmissionState == IRadio::TRANSMISSION_STATE_TRANSMITTING && newState == IRadio::TRANSMISSION_STATE_IDLE;
        transmissionState = newState;
        if (endOfTransmission) {
            // endPacket() returned: back to continuous Rx
            radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
            transmitting = false;

            // tx() and forward() both set lastTx = millis(): the next beacon or forward waits BEACON_INTVL
            lastTx = simTime();
            scheduleNextBeacon();
        }
    }
    else if (signalID == radioDroppedPacketSignal) {
        // RxDone with a CRC error (collision): DIO0 still fires, so the back-off still starts
        numRxLost++;
        emit(rxLostSignal, 1);
        startBackoff();
    }
}

} // namespace lorav2v
