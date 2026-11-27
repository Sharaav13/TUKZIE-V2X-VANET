#ifndef __LORAV2V_LORABEACONAPP_H
#define __LORAV2V_LORABEACONAPP_H

#include "inet/common/INETDefs.h"
#include "inet/common/packet/Packet.h"
#include "inet/mobility/contract/IMobility.h"

namespace lorav2v {

using namespace inet;

/**
 * Periodically broadcasts a V2VBeacon and records what it receives.
 *
 * For packet delivery ratio vs. distance, every transmission also records
 * the distance to every other vehicle at that moment (txNeighborDistance,
 * the denominator); every reception records the sender distance
 * (rxDistance, the numerator).
 */
class LoRaBeaconApp : public cSimpleModule
{
  protected:
    cMessage *sendTimer = nullptr;
    B payloadLength;
    uint32_t sequenceNumber = 0;
    IMobility *mobility = nullptr;

    long numSent = 0;
    long numReceived = 0;

    static simsignal_t beaconSentSignal;
    static simsignal_t beaconReceivedSignal;
    static simsignal_t rxDistanceSignal;
    static simsignal_t txNeighborDistanceSignal;
    static simsignal_t rxPowerSignal;

  public:
    virtual ~LoRaBeaconApp();

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;

    virtual void sendBeacon();
    virtual void processBeacon(Packet *packet);
};

} // namespace lorav2v

#endif
