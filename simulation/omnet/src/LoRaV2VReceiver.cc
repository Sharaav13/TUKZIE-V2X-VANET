#include "LoRaV2VReceiver.h"

namespace lorav2v {

Define_Module(LoRaV2VReceiver);

bool LoRaV2VReceiver::computeIsReceptionAttempted(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference) const
{
    // INET's RadioMedium::isReceptionAttempted() calls this directly to decide
    // whether the radio starts receiving (RECEIVING state). FLoRa only checks
    // collisions here, so frames below sensitivity would lock the receiver and
    // make carrier sense see every transmission on the map. Require sensitivity too.
    if (!computeIsReceptionPossible(listening, reception, part))
        return false;
    return !isPacketCollided(reception, part, interference);
}

} // namespace lorav2v
