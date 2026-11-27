package lorav2v;

import inet.networklayer.common.InterfaceTable;
import org.car2x.veins.subprojects.veins_inet.VeinsInetMobility;

//
// Vehicle with a LoRa radio broadcasting V2V beacons. Created and deleted
// by VeinsInetManager as SUMO vehicles enter and leave.
//
module LoRaV2VCar
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
        LoRaNic: LoRaV2VNic {
            @display("p=200,250");
        }
        app: LoRaBeaconApp {
            @display("p=200,80");
        }
    connections allowunconnected:   // radioIn is fed directly by the radio medium
        app.socketOut --> LoRaNic.upperLayerIn;
        LoRaNic.upperLayerOut --> app.socketIn;
}
