package lorav2v;

import inet.linklayer.base.MacProtocolBase;
import inet.linklayer.contract.IMacProtocol;

//
// MAC of the TVV node firmware (struct Vanet in vanet.h / vanet.cpp), on the FLoRa LoRa radio.
// See TvvMac.h for the access rules. Also configures the LoRa PHY (system.h LORA_*).
//
simple TvvMac extends MacProtocolBase like IMacProtocol
{
    parameters:
        string radioModule = default("^.radio");
        string beaconSourceModule = default("^.^.routing");    // builds each beacon (ITvvBeaconSource)
        string address @mutable = default("auto");
        int headerLength @unit(B) = default(1B);    // flora::LoRaMacFrame, stands for the dest byte (see TvvPacket.msg)

        // LoRa PHY (system.h)
        double centerFrequency @unit(Hz) = default(433.2125MHz);   // LORA_FREQ
        double bandwidth @unit(Hz) = default(125kHz);          // LORA_BW
        int spreadingFactor = default(7);                       // LORA_SF (7..12)
        int codingRate = default(1);                            // LORA_CR 5 = 4/5 (1..4 = 4/5..4/8)
        double txPower @unit(dBm) = default(10dBm);            // LORA_TXPOWER

        // MAC (system.h)
        double beaconInterval @unit(s) = default(15000ms);      // BEACON_INTVL, from the end of the previous Tx
        double backoffMin @unit(s) = default(0ms);              // BACKOFF_MIN_MS
        double backoffMax @unit(s) = default(300ms);            // BACKOFF_MAX_MS (included)
        double pollInterval @unit(s) = default(1ms);            // networkLoop() period (vTaskDelay(1) + loop work)
        // "preamble": busy while a LoRa frame is being received (receiving(): RegModemStat signal detected/synced/header valid)
        // "energy":   also busy while energy is above the receiver's energyDetection (for comparison, not on the node)
        string carrierSense = default("preamble");

        @class(lorav2v::TvvMac);
        @display("i=block/rxtx");

        @signal[channelAccessDelay](type=simtime_t);
        @signal[backoffDuration](type=simtime_t);
        @signal[txInterval](type=simtime_t);
        @statistic[channelAccessDelay](title="channel access delay (beacon due to Tx start)"; record=mean,max,vector; unit=s);
        @statistic[backoffDuration](title="back-off duration"; record=mean,histogram; unit=s);
        @statistic[txInterval](title="time between beacons"; record=mean,max,vector; unit=s);
}
