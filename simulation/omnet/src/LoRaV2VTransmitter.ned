package lorav2v;

import flora.LoRaPhy.LoRaTransmitter;

module LoRaV2VTransmitter extends LoRaTransmitter
{
    parameters:
        int preambleSymbols = default(8);
        headerLength = default(0B); // INET base parameter; airtime comes from the LoRa formula
        @class(lorav2v::LoRaV2VTransmitter);
}
