package lorav2v;

import inet.networklayer.common.InterfaceTable;
import org.car2x.veins.subprojects.veins_inet.VeinsInetMobility;

//
// Vehicle carrying a TVV node: LoRa radio, the firmware's MAC (TvvMac) and broadcast routing (TvvRouting).
// Created and deleted by VeinsInetManager as SUMO vehicles enter and leave.
//
module TvvCar
{
    parameters:
        @networkNode();
        *.interfaceTableModule = default(absPath(".interfaceTable"));
        @display("i=block/wrxtx;is=vs");
    submodules:
        interfaceTable: InterfaceTable {
            @display("p=50,50");
        }
        mobility: VeinsInetMobility {
            @display("p=50,150");
        }
        // must be named LoRaNic: FLoRa's receiver looks up "LoRaNic.radio"
        LoRaNic: TvvNic {
            @display("p=200,250");
        }
        routing: TvvRouting {
            @display("p=200,80");
        }
    connections allowunconnected:   // radioIn is fed directly by the radio medium
        routing.lowerLayerOut --> LoRaNic.upperLayerIn;
        LoRaNic.upperLayerOut --> routing.lowerLayerIn;
}
