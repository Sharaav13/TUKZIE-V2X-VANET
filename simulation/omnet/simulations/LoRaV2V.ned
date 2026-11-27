package lorav2v.simulations;

import flora.LoRaPhy.LoRaMedium;
import lorav2v.LoRaV2VCar;
import lorav2v.PlainCar;
import org.car2x.veins.subprojects.veins_inet.VeinsInetManager;
import org.car2x.veins.visualizer.roads.RoadsCanvasVisualizer;
import inet.environment.common.PhysicalEnvironment;
import inet.visualizer.canvas.environment.PhysicalEnvironmentCanvasVisualizer;

//
// Car-to-car LoRa broadcast: SUMO vehicles (via Veins) broadcast beacons
// to each other over a shared 433.6 MHz LoRa channel using CSMA/CA.
// No gateways, no network server.
//
network LoRaV2V
{
    parameters:
        @display("bgb=2700,3100,#4b3b63");   // dark lilac background
    submodules:
        manager: VeinsInetManager {
            @display("p=100,100");
        }
        roadsCanvasVisualizer: RoadsCanvasVisualizer {
            @display("p=100,250");
        }
        LoRaMedium: LoRaMedium {
            @display("p=100,400");
        }
        // buildings, for drawing only (no obstacle loss is configured)
        physicalEnvironment: PhysicalEnvironment {
            @display("p=100,550");
        }
        environmentVisualizer: PhysicalEnvironmentCanvasVisualizer {
            @display("p=100,700");
        }
        node[0]: LoRaV2VCar;   // LoRa vehicles, added dynamically by the manager
        car[0]: PlainCar;      // ordinary vehicles (no radio), added dynamically
}
