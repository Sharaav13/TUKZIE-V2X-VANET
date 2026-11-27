package lorav2v;

import inet.applications.contract.IApp;

//
// Periodic V2V broadcast beacon (see LoRaBeaconApp.h).
//
simple LoRaBeaconApp like IApp
{
    parameters:
        volatile double startTime @unit(s) = default(uniform(0s, 5s));
        volatile double beaconInterval @unit(s) = default(5s);
        int payloadLength @unit(B) = default(20B);
        @class(lorav2v::LoRaBeaconApp);
        @display("i=block/app");
        @signal[beaconSent](type=long);
        @signal[beaconReceived](type=simtime_t);
        @signal[rxDistance](type=double);
        @signal[txNeighborDistance](type=double);
        @signal[rxPower](type=double);
        @statistic[beaconSent](record=count);
        @statistic[beaconDelay](source=beaconReceived; record=count,mean,max,vector; unit=s);
        @statistic[rxDistance](record=mean,max,vector; unit=m);
        @statistic[txNeighborDistance](record=vector; unit=m);
        @statistic[rxPower](record=mean,vector; unit=dBm);
    gates:
        input socketIn;
        output socketOut;
}
