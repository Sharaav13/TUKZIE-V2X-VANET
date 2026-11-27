#ifndef __LORAV2V_LORAV2VTRANSMITTER_H
#define __LORAV2V_LORAV2VTRANSMITTER_H

#include "LoRaPhy/LoRaTransmitter.h"

namespace lorav2v {

using namespace inet;
using namespace inet::physicallayer;

/**
 * FLoRa LoRaTransmitter with the standard Semtech time-on-air formula
 * (SX127x datasheet / AN1200.13) evaluated on the actual frame length.
 * FLoRa's own transmitter assumes a fixed 20-byte payload.
 */
class LoRaV2VTransmitter : public flora::LoRaTransmitter
{
  protected:
    int preambleSymbols = 8;

  public:
    virtual void initialize(int stage) override;
    virtual const ITransmission *createTransmission(const IRadio *radio, const Packet *packet, const simtime_t startTime) const override;
};

} // namespace lorav2v

#endif
