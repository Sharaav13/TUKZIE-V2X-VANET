#ifndef __LORAV2V_TVVMAC_H
#define __LORAV2V_TVVMAC_H

#include "inet/common/ModuleRefByPar.h"
#include "inet/common/packet/Packet.h"
#include "inet/linklayer/base/MacProtocolBase.h"
#include "inet/linklayer/contract/IMacProtocol.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadio.h"

#include "ITvvBeaconSource.h"

namespace lorav2v {

using namespace inet;

/**
 * MAC of the TVV node firmware (struct Vanet in vanet.h / vanet.cpp).
 *
 * Firmware rules, modelled one to one:
 *  - First beacon: init() sets lastTx = now - random(0, BEACON_INTVL), so nodes start out of step.
 *  - beaconDue() lets a beacon out only when all three hold:
 *      1. BEACON_INTVL has passed since lastTx (the end of the previous Tx),
 *      2. the back-off has ended,
 *      3. no LoRa frame is being received right now (receiving(): RegModemStat).
 *    Otherwise networkLoop() asks again on its next pass (pollInterval).
 *  - Every RxDone (DIO0) starts a new back-off of random(BACKOFF_MIN_MS, BACKOFF_MAX_MS + 1) ms,
 *    replacing any running one. RxDone also fires for frames with a CRC error, so frames lost to a
 *    collision start the back-off too, as do frames later dropped by TvvRouting (dest or size).
 *  - tx(): the beacon is built at that moment (ITvvBeaconSource), sent as one broadcast frame,
 *    then the radio returns to continuous Rx and lastTx is set. No queue, ACKs or retries.
 *  - Half duplex: the radio cannot receive while transmitting.
 */
class TvvMac : public MacProtocolBase, public IMacProtocol
{
  protected:
    enum State { WAIT_INTERVAL, WAIT_CHANNEL, TRANSMIT };

    // parameters
    B headerLength;
    Hz centerFrequency;
    Hz bandwidth;
    int spreadingFactor = 0;
    int codingRate = 0;
    double txPowerDbm = 0;
    simtime_t beaconInterval;
    int backoffMinMs = 0;
    int backoffMaxMs = 0;
    simtime_t pollInterval;
    bool senseEnergy = false;

    ModuleRefByPar<physicallayer::IRadio> radio;
    ITvvBeaconSource *beaconSource = nullptr;
    physicallayer::IRadio::TransmissionState transmissionState = physicallayer::IRadio::TRANSMISSION_STATE_UNDEFINED;

    // firmware state (Vanet)
    State state = WAIT_INTERVAL;
    simtime_t lastTx;               // end of the previous Tx
    simtime_t backoffEnd;           // backoffStart + backoffMs
    simtime_t dueTime;              // when the current beacon became due
    simtime_t lastTxStart = -1;
    bool blockedByBackoff = false;  // the current beacon has waited for the back-off
    bool blockedByReceiving = false;// the current beacon has waited for a reception

    cMessage *beaconTimer = nullptr;    // beacon due, or the next networkLoop() pass while waiting

    // statistics
    long numSent = 0;
    long numReceived = 0;           // frames decoded and passed up
    long numRxLost = 0;             // receptions that ended with a CRC error (collision)
    long numBackoffs = 0;           // back-offs started (RxDone)
    long numBlockedByBackoff = 0;   // beacons delayed by the back-off
    long numBlockedByReceiving = 0; // beacons delayed by a frame on air
    static simsignal_t channelAccessDelaySignal;
    static simsignal_t backoffDurationSignal;
    static simsignal_t txIntervalSignal;
    static simsignal_t radioDroppedPacketSignal;

  public:
    virtual ~TvvMac();

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void finish() override;
    virtual void configureNetworkInterface() override;
    virtual void refreshDisplay() const override;

    virtual void handleSelfMessage(cMessage *msg) override;
    virtual void handleUpperPacket(Packet *packet) override;
    virtual void handleLowerPacket(Packet *packet) override;
    using MacProtocolBase::receiveSignal;
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details) override;

    // Vanet
    virtual void tryTransmit();         // beaconDue(), then tx()
    virtual void transmit();            // tx()
    virtual void startBackoff();        // startBackoff()
    virtual bool backoffActive() const; // backoffActive()
    virtual bool receiving() const;     // receiving()
    virtual void scheduleNextBeacon();
    virtual void encapsulate(Packet *packet);
};

} // namespace lorav2v

#endif
