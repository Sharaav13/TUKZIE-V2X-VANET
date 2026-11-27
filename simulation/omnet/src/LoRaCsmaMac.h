#ifndef __LORAV2V_LORACSMAMAC_H
#define __LORAV2V_LORACSMAMAC_H

#include "inet/common/ModuleRefByPar.h"
#include "inet/common/packet/Packet.h"
#include "inet/linklayer/base/MacProtocolBase.h"
#include "inet/linklayer/contract/IMacProtocol.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadio.h"
#include "inet/queueing/contract/IActivePacketSink.h"
#include "inet/queueing/contract/IPacketQueue.h"

namespace lorav2v {

using namespace inet;

/**
 * Broadcast-only CSMA/CA MAC on top of the FLoRa LoRa radio.
 *
 * Every frame is broadcast. Before each transmission the node waits until the
 * channel has been idle for DIFS, then counts down a random backoff of
 * uniform{0..cw} slots. The countdown freezes while the channel is busy and
 * resumes, after another DIFS, once it is idle again. Broadcasts get no
 * feedback, so there are no ACKs, retries or contention-window growth.
 * Carrier sense is the radio's reception state (energy detection).
 */
class LoRaCsmaMac : public MacProtocolBase, public IMacProtocol, public queueing::IActivePacketSink
{
  protected:
    enum State { IDLE, DEFER, WAITDIFS, BACKOFF, TRANSMIT };

    // parameters
    B headerLength;
    simtime_t slotTime;
    simtime_t difsTime;
    int cw = 0;
    Hz centerFrequency;
    Hz bandwidth;
    int spreadingFactor = 0;
    int codingRate = 0;
    double txPowerDbm = 0;

    ModuleRefByPar<physicallayer::IRadio> radio;
    physicallayer::IRadio::TransmissionState transmissionState = physicallayer::IRadio::TRANSMISSION_STATE_UNDEFINED;

    State state = IDLE;
    simtime_t backoffPeriod = -1; // remaining backoff; negative = not drawn yet
    simtime_t accessStartTime;
    int sequenceNumber = 0;

    cMessage *endDifs = nullptr;
    cMessage *endBackoff = nullptr;

    // statistics
    long numSent = 0;
    long numReceived = 0;
    long numDeferred = 0;      // channel busy when a frame became ready
    long numBackoffFrozen = 0; // countdowns interrupted by a busy channel
    long numRxLost = 0;        // receptions started but lost (collision)
    static simsignal_t channelAccessDelaySignal;
    static simsignal_t backoffSlotsSignal;
    static simsignal_t radioDroppedPacketSignal;

  public:
    virtual ~LoRaCsmaMac();

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void finish() override;
    virtual void configureNetworkInterface() override;

    virtual void handleSelfMessage(cMessage *msg) override;
    virtual void handleUpperPacket(Packet *packet) override;
    virtual void handleLowerPacket(Packet *packet) override;
    using MacProtocolBase::receiveSignal;
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details) override;

    // IActivePacketSink
    virtual queueing::IPassivePacketSource *getProvider(cGate *gate) override;
    virtual void handleCanPullPacketChanged(cGate *gate) override;
    virtual void handlePullPacketProcessed(Packet *packet, cGate *gate, bool successful) override;

    virtual void processUpperPacket();
    virtual void encapsulate(Packet *packet);
    virtual void startContention();
    virtual void startDifs();
    virtual void startBackoff();
    virtual void freezeBackoff();
    virtual void transmit();
    virtual void handleMediumStateChange();
    virtual bool isMediumFree() const;
};

} // namespace lorav2v

#endif
