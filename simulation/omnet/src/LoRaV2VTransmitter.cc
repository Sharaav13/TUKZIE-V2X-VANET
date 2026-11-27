#include "LoRaV2VTransmitter.h"

#include "LoRaPhy/LoRaPhyPreamble_m.h"
#include "LoRaPhy/LoRaTransmission.h"

namespace lorav2v {

Define_Module(LoRaV2VTransmitter);

void LoRaV2VTransmitter::initialize(int stage)
{
    flora::LoRaTransmitter::initialize(stage);
    if (stage == INITSTAGE_LOCAL)
        preambleSymbols = par("preambleSymbols");
}

const ITransmission *LoRaV2VTransmitter::createTransmission(const IRadio *radio, const Packet *packet, const simtime_t startTime) const
{
    const auto& preamble = packet->peekAtFront<flora::LoRaPhyPreamble>();
    int sf = preamble->getSpreadFactor();
    double bw = preamble->getBandwidth().get();
    int cr = preamble->getCodeRendundance(); // 1..4 = 4/5..4/8

    // PHY payload = MAC header + data. The LoRaPhyPreamble chunk is FLoRa's
    // simulation-side carrier for the PHY settings, not part of the payload.
    int payloadBytes = (int)std::ceil(b(packet->getTotalLength() - preamble->getChunkLength()).get() / 8.0);

    double tSym = std::pow(2.0, sf) / bw;
    int de = tSym > 16e-3 ? 1 : 0;              // low data rate optimisation
    int ih = preamble->getUseHeader() ? 0 : 1;  // implicit header
    int crc = 1;
    double num = 8.0 * payloadBytes - 4.0 * sf + 28 + 16 * crc - 20 * ih;
    int payloadSymbols = 8 + std::max((int)std::ceil(num / (4.0 * (sf - 2 * de))) * (cr + 4), 0);

    simtime_t preambleDuration = (preambleSymbols + 4.25) * tSym;
    simtime_t headerDuration = 8 * tSym; // first 8 payload symbols carry the explicit header
    simtime_t dataDuration = (payloadSymbols - 8) * tSym;
    simtime_t endTime = startTime + preambleDuration + headerDuration + dataDuration;

    IMobility *mobility = radio->getAntenna()->getMobility();
    const Coord position = mobility->getCurrentPosition();
    const Quaternion orientation = mobility->getCurrentAngularPosition();

    return new flora::LoRaTransmission(radio, packet, startTime, endTime,
            preambleDuration, headerDuration, dataDuration,
            position, position, orientation, orientation,
            preamble->getPower(), preamble->getCenterFrequency(), sf, preamble->getBandwidth(), cr);
}

} // namespace lorav2v
