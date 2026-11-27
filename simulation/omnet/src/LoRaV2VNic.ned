package lorav2v;

import inet.linklayer.common.WirelessInterface;

//
// LoRa network interface: FLoRa radio + broadcast CSMA/CA MAC.
//
module LoRaV2VNic extends WirelessInterface
{
    parameters:
        @display("i=block/ifcard");
        *.interfaceTableModule = default(absPath(this.interfaceTableModule));
        radio.typename = default("flora.LoRa.LoRaRadio");
        radio.antenna.mobilityModule = default("^.^.^.mobility");
        radio.transmitter.typename = default("lorav2v.LoRaV2VTransmitter");
        radio.receiver.typename = default("lorav2v.LoRaV2VReceiver");
        mac.typename = default("lorav2v.LoRaCsmaMac");
        queue.typename = default("DropTailQueue");
        queue.packetCapacity = default(5);
}
