package lorav2v.simulations;

import flora.LoRaPhy.LoRaMedium;
import lorav2v.TvvNode;

//
// numNodes TVV nodes at fixed positions on a shared 433.2125 MHz LoRa channel, no SUMO.
// Positions are set in tvv.ini (TvvStatic configs): a two-node distance sweep, or a disc where
// every node hears every other.
//
network TvvStatic
{
    parameters:
        int numNodes = default(2);
        @display("bgb=2700,3100,#4b3b63");
    submodules:
        LoRaMedium: LoRaMedium {
            @display("p=100,100");
        }
        node[numNodes]: TvvNode;
}
