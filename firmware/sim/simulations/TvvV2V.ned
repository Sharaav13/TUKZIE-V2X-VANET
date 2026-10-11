package lorav2v.simulations;

import flora.LoRaPhy.LoRaMedium;
import lorav2v.TvvCar;
import lorav2v.PlainCar;
import org.car2x.veins.subprojects.veins_inet.VeinsInetManager;
import org.car2x.veins.visualizer.roads.RoadsCanvasVisualizer;
import inet.environment.common.PhysicalEnvironment;
import inet.visualizer.canvas.environment.PhysicalEnvironmentCanvasVisualizer;

//
// The LoRaV2V scenario with TVV nodes: SUMO vehicles of type loraCar (via Veins) run the
// TVV firmware's MAC and broadcast routing over a shared 433.2125 MHz LoRa channel.
//
network TvvV2V
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
        node[0]: TvvCar;       // TVV vehicles, added dynamically by the manager
        car[0]: PlainCar;      // ordinary vehicles (no radio), added dynamically
}
