package lorav2v;

import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;

//
// TVV node at a fixed position (no SUMO): same LoRa interface, MAC and routing as TvvCar,
// with an INET mobility module (StationaryMobility by default). Used by TvvStatic.
//
module TvvNode
{
    parameters:
        @networkNode();
        *.interfaceTableModule = default(absPath(".interfaceTable"));
        @display("i=block/wrxtx;is=vs");
    submodules:
        interfaceTable: InterfaceTable {
            @display("p=50,50");
        }
        mobility: <default("StationaryMobility")> like IMobility {
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
