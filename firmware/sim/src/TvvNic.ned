package lorav2v;

import inet.linklayer.common.WirelessInterface;

//
// LoRa network interface of a TVV node: FLoRa radio + the firmware's MAC (TvvMac).
// No transmit queue: like the firmware, TvvMac builds each beacon when it gets the channel.
//
module TvvNic extends WirelessInterface
{
    parameters:
        @display("i=block/ifcard");
        *.interfaceTableModule = default(absPath(this.interfaceTableModule));
        radio.typename = default("flora.LoRa.LoRaRadio");
        radio.antenna.mobilityModule = default("^.^.^.mobility");
        radio.transmitter.typename = default("lorav2v.LoRaV2VTransmitter");   // time on air from the real 25 B frame
        radio.receiver.typename = default("lorav2v.LoRaV2VReceiver");         // only attempts frames above sensitivity
        mac.typename = default("lorav2v.TvvMac");
}
