package lorav2v;

import org.car2x.veins.subprojects.veins_inet.VeinsInetMobility;

//
// Ordinary (non-LoRa) vehicle: position only, no radio. Lets the
// background traffic be shown in the OMNeT++ window; it takes no part in
// the LoRa communication or its statistics.
//
module PlainCar
{
    parameters:
        @display("i=veins/node/car,blue;is=l");
    submodules:
        mobility: VeinsInetMobility {
            @display("p=50,50");
        }
}
