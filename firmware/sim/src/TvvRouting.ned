package lorav2v;

//
// Broadcast routing of the TVV node firmware (vanet.cpp: Packet, readPacket(), Cluster).
// See TvvRouting.h. Builds each beacon when TvvMac gets the channel (ITvvBeaconSource).
//
simple TvvRouting
{
    parameters:
        int nodeId = default(-1);           // config.txt node ID (1..255), -1 = vehicle index + 1 (0 is the broadcast address)
        int destId = default(0);            // destination set by the host, 0 = broadcast
        int message = default(0);           // message code carried in every beacon (0 = none)
        double channelRssi @unit(dBm) = default(-120dBm);   // LoRa.rssi() sent in each beacon: not modelled, a fixed channel level (calibrate from srcRssi in your logs)
        int packetLength @unit(B) = default(24B);           // TvvPacket after the dest byte: with TvvMac's 1 B header, 25 B on air
        int maxPeers = default(3);                          // MAX_PEERS
        double peerTimeout @unit(s) = default(10s);         // PEER_TIMEOUT
        double peerCheckInterval @unit(s) = default(1s);    // PEER_CHECK_INTVL

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
    gates:
        input lowerLayerIn;
        output lowerLayerOut;   // unused: TvvMac pulls beacons through ITvvBeaconSource
}
