#ifndef __LORAV2V_LORAV2VRECEIVER_H
#define __LORAV2V_LORAV2VRECEIVER_H

#include "LoRaPhy/LoRaReceiver.h"

namespace lorav2v {

using namespace inet;
using namespace inet::physicallayer;

/**
 * FLoRa LoRaReceiver usable under any MAC. The original collision path
 * casts the MAC to flora::LoRaMac, and it attempts reception of frames
 * below sensitivity; both are fixed here. The collision/capture model
 * itself (isPacketCollided) is unchanged.
 */
class LoRaV2VReceiver : public flora::LoRaReceiver
{
  public:
    virtual bool computeIsReceptionAttempted(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference) const override;
};

} // namespace lorav2v

#endif
