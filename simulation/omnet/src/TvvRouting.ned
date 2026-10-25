package lorav2v;

//
// Routing of the TVV node firmware (vanet.cpp: Packet, readPacket(), Cluster, forwarding).
// See TvvRouting.h. Builds each beacon, and hands over each queued forward, when TvvMac gets the channel (ITvvBeaconSource).
//
simple TvvRouting
{
    parameters:
        int nodeId = default(-1);           // config.txt node ID (1..255), -1 = vehicle index + 1 (0 is the broadcast address)
        int destId = default(0);            // destination set by the host, 0 = broadcast
        int message = default(0);           // message code carried in every beacon (0 = none)
        int packetLength @unit(B) = default(24B);           // TvvPacket after the dest byte: with TvvMac's 1 B header, 25 B on air
        int maxPeers = default(3);                          // MAX_PEERS
        double peerTimeout @unit(s) = default(30s);         // PEER_TIMEOUT
        double peerCheckInterval @unit(s) = default(1s);    // PEER_CHECK_INTVL
        int ttlInit = default(2);                           // TTL_INIT: hops a packet may make (a forward decrements it, not sent at 0)
        int fwdQueueSize = default(2);                      // FWD_QUEUE_SIZE
        int dupHistory = default(48);                       // DUP_HISTORY: recent packets (src, seq) remembered to drop repeats
        double dupWindow @unit(s) = default(90s);           // DUP_WINDOW_MS
        double addressedFraction = default(0);              // simulation traffic mix: share of beacons addressed to a random other node
        double rxProcessingDelay @unit(s) = default(0s);    // added to the delay statistics (node processing from RxDone to the timestamp)
        string macModule = default("^.LoRaNic.mac");        // TvvMac, told when a packet is queued for forwarding

        @class(lorav2v::TvvRouting);
        @display("i=block/routing");

        // Names shared with LoRaBeaconApp, so simulations/pdr.py and plot.py work on TvvRouting results
        @signal[beaconSent](type=long);
        @signal[beaconReceived](type=simtime_t);
        @signal[rxDistance](type=double);
        @signal[txNeighborDistance](type=double);
        @signal[rxPower](type=double);
        @signal[rxSnr](type=double);
        @signal[clusterSize](type=long);
        @statistic[beaconSent](record=count);
        @statistic[beaconDelay](source=beaconReceived; record=count,mean,max,vector; unit=s);
        @statistic[rxDistance](record=mean,max,vector; unit=m);
        @statistic[txNeighborDistance](record=vector; unit=m);
        @statistic[rxPower](title="pktRssi"; record=mean,vector; unit=dBm);
        @statistic[rxSnr](title="snr"; record=mean,vector; unit=dB);
        @statistic[clusterSize](title="peers in the cluster"; record=mean,max,vector);

        // Counts and delays that honour warmup-period (the stationary sweeps)
        @signal[beaconReceivedDirect](type=simtime_t);
        @signal[beaconReceivedForwarded](type=simtime_t);
        @signal[heardDirect](type=long);
        @signal[seqLost](type=long);
        @signal[addressedSent](type=long);
        @signal[addressedReceived](type=long);
        @signal[addressedUnique](type=long);
        @signal[forwardedCopy](type=simtime_t);
        @signal[clusterFull](type=long);
        @signal[peerAdded](type=long);
        @signal[peerRemoved](type=long);
        @statistic[beaconDelayDirect](source=beaconReceivedDirect; record=count,mean,max; unit=s);
        @statistic[beaconDelayForwarded](source=beaconReceivedForwarded; record=count,mean,max; unit=s);
        @statistic[heardDirect](record=sum);        // frames heard straight from their sender, any dest
        @statistic[seqLost](record=sum);            // frames missing from those senders' seq
        @statistic[addressedSent](record=sum);      // my beacons addressed to one node
        @statistic[addressedReceived](record=sum);  // packets addressed to me read, direct or forwarded
        @statistic[addressedUnique](record=sum);    // of them, the first copy of each packet
        @statistic[forwardedCopyDelay](source=forwardedCopy; record=count,mean,max; unit=s);   // every forwarded copy reaching its dest
        @statistic[clusterFull](record=sum);        // packets from new peers that did not fit in the cluster
        @statistic[peerAdded](record=sum);
        @statistic[peerRemoved](record=sum);
    gates:
        input lowerLayerIn;
        output lowerLayerOut;   // unused: TvvMac pulls beacons through ITvvBeaconSource
}
