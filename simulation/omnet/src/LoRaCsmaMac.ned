package lorav2v;

import inet.linklayer.base.MacProtocolBase;
import inet.linklayer.contract.IMacProtocol;

//
// Broadcast-only CSMA/CA MAC for LoRa (see LoRaCsmaMac.h).
// Also configures the LoRa PHY settings (CF, BW, SF, CR, TX power).
//
simple LoRaCsmaMac extends MacProtocolBase like IMacProtocol
{
    parameters:
        string radioModule = default("^.radio");
        string address @mutable = default("auto");
        int headerLength @unit(B) = default(8B);
        // LoRa PHY
        double centerFrequency @unit(Hz) = default(433.6MHz);
        double bandwidth @unit(Hz) = default(125kHz);
        int spreadingFactor = default(7);       // 7..12
        int codingRate = default(1);            // 1..4 = 4/5..4/8
        double txPower @unit(dBm) = default(10dBm); // EU 433.05-434.79 MHz: 10 mW e.r.p.
        // CSMA/CA
        double slotTime @unit(s) = default(2.048ms); // ~2 symbols at SF7/125 kHz (CAD + turnaround)
        double difsTime @unit(s) = default(5ms);
        int cw = default(63);                   // backoff drawn from 0..cw slots
        @class(lorav2v::LoRaCsmaMac);
        @display("i=block/rxtx");
        @signal[channelAccessDelay](type=simtime_t);
        @signal[backoffSlots](type=long);
        @statistic[channelAccessDelay](title="channel access delay"; record=mean,max,vector; unit=s);
        @statistic[backoffSlots](title="backoff slots"; record=mean,histogram);
}
