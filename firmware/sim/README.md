# TVV firmware simulation

OMNeT++ model of the TVV node firmware's **MAC** and **broadcast routing**, for the `lora-v2v` project
(OMNeT++ 6.1, INET 4.5, FLoRa 1.2.0, Veins 5.3.1 + SUMO, in `~/TVV`). It reuses the project's LoRa PHY
(`LoRaV2VTransmitter`, `LoRaV2VReceiver`), channel, SUMO scenario and analysis scripts.

## Files

| File | Firmware | Role |
|---|---|---|
| `src/TvvMac.*` | `Vanet` (vanet.h / vanet.cpp) | MAC: beacon timing, back-off, carrier sense, Tx/Rx; also sets the LoRa PHY |
| `src/TvvRouting.*` | `Packet`, `readPacket()`, `Cluster` | Broadcast routing: builds beacons, dest filter, cluster of peers |
| `src/TvvPacket.msg` | `struct Packet` | The 25-byte packet (24 B here + TvvMac's 1 B header, see below) |
| `src/ITvvBeaconSource.h` | `copyMyself()` + `vanet.tx()` | Lets TvvMac build the beacon when it gets the channel |
| `src/TvvNic.ned`, `src/TvvCar.ned` | | LoRa interface and vehicle |
| `simulations/TvvV2V.ned` | | Network: the LoRaV2V scenario with TvvCar nodes |
| `simulations/tvv.ini` | `system.h` | Configs `TvvFirmware`, `TvvFirmwareEnergySense`, `TvvFirmwareBigCluster` |

## Install and run (WSL)

```bash
# from Windows' copy of the repo into the project
cp /mnt/c/Users/shara/Dev/TUKZIE-V2X-VANET/firmware/sim/src/*         ~/TVV/lora-v2v/src/
cp /mnt/c/Users/shara/Dev/TUKZIE-V2X-VANET/firmware/sim/simulations/* ~/TVV/lora-v2v/simulations/
echo "include tvv.ini" >> ~/TVV/lora-v2v/simulations/omnetpp.ini     # once

source ~/TVV/tvv-env.sh
cd ~/TVV/lora-v2v
make makefiles          # new .cc/.msg files: regenerate src/Makefile
make MODE=release

tvv-run lora-v2v --config TvvFirmware             # GUI
tvv-run lora-v2v --config TvvFirmware --cmdenv    # headless

cd ~/TVV/lora-v2v/simulations
python3 pdr.py  "results/TvvFirmware-#0.vec"      # PDR vs distance + totals
python3 plot.py "results/TvvFirmware-#0.vec"
```

The existing `General` config (LoRaCsmaMac + LoRaBeaconApp) is unchanged, so both can be compared.

## MAC (TvvMac)

As in `vanet.cpp`, with the `system.h` values as defaults:

1. **First beacon:** `lastTx = now - random(0, BEACON_INTVL)`, so nodes start out of step.
2. **A beacon goes out only when all three hold** (`beaconDue()`):
   - `BEACON_INTVL` (15 s) has passed since `lastTx`, the **end** of the previous Tx,
   - the back-off has ended,
   - no LoRa frame is being received (`receiving()`, RegModemStat: signal detected, synchronised, header valid).

   Otherwise the beacon is checked again on the next `networkLoop()` pass (`pollInterval`, 1 ms), or when the back-off ends.
3. **Back-off:** every RxDone starts a new back-off of `random(0, 301)` ms, replacing any running one.
   RxDone also fires for frames with a CRC error, so collided frames start it too, as do frames the routing
   layer then drops (dest or size).
4. **Tx:** the beacon is built at that moment from the node's current state, sent as one broadcast frame, then
   the radio returns to continuous Rx. No queue, ACKs or retries. The radio is half duplex.

`carrierSense = "energy"` (config `TvvFirmwareEnergySense`) also treats energy above the receiver's
`energyDetection` as busy. The node does not do this; it is there for comparison.

## Broadcast routing (TvvRouting)

Single hop, no forwarding. Every node broadcasts its own beacon and keeps the peers it hears:

- **Tx:** `seq` = previous + 1 (wraps at 65535), `rssi` = `channelRssi`, `dest` = `destId` (0 = broadcast).
- **Rx:** wrong-size frames are ignored; only packets with `dest == nodeId` or `dest == 0` are read
  (provision for destination-based routing); the sender is added to or refreshed in the cluster; the
  latest message is kept.
- **Cluster:** at most `maxPeers` (3). A new peer is not added while it is full. Peers not heard for
  `peerTimeout` (10 s) are removed every `peerCheckInterval` (1 s).

Node IDs default to vehicle index + 1 (0 is the broadcast address). Set `routing.nodeId` per node to choose them.

## Results

`simulations/results/<config>-#0.{sca,vec}`, per node:

| Module | Statistics |
|---|---|
| `node[*].routing` | `beaconsSent`, `beaconsReceived`, `beaconDelay`, `rxDistance`, `txNeighborDistance`, `rxPower` (pktRssi), `rxSnr`, `clusterSize`, `numSeqLost` (gaps in each sender's seq, as `tests/rf_profiling.m`), `numNotForMe`, `numClusterFull`, `numWrongSize`, `numDuplicates`, `nodeId` |
| `node[*].LoRaNic.mac` | `numSent`, `numReceived`, `numRxLost` (CRC errors), `numBackoffs`, `numBlockedByBackoff`, `numBlockedByReceiving`, `channelAccessDelay` (due to Tx start), `txInterval`, `backoffDuration` |

The names match `LoRaBeaconApp`, so `pdr.py` and `plot.py` work unchanged.

A 120 s test run with the earlier 2 s beacon interval (10 LoRa cars, Erlangen) delivered 3871 of 5067 possible receptions (76 %), with beacons
2.13 s apart on average. With 10 cars in range, each node's 3-peer cluster was full (`numClusterFull`);
`TvvFirmwareBigCluster` shows about 8 peers per node.

## Modelling notes

- **Packet size:** FLoRa's radio needs its own MAC header (`flora::LoRaMacFrame`) at the front of every frame.
  TvvMac makes it 1 byte long to stand for the `dest` byte, so with the 24-byte `TvvPacket` the frame is
  25 bytes on air and the time on air matches the node (about 62 ms at SF7). The header's receiver address
  is always broadcast, so every radio receives every frame (as the SX127x does) and TvvRouting applies the
  firmware's dest filter.
- **`message` field:** called `messageCode` in `TvvPacket.msg`, since `message` is a reserved word there.
- **`rssi` / srcRssi:** `LoRa.rssi()` (channel noise at the sender) is not modelled. Each beacon carries the
  fixed `channelRssi`. Set it from the srcRssi column of your node logs.
- **Not modelled:** frequency error and the GPS/host payload (the payload fields are 0, battery 100 %),
  and loop delays from SD writes (`pollInterval` covers the 1 ms loop only).
- **SNR:** FLoRa's SNIR is not capped. A real SX127x reports at most about +10 dB, so the simulated SNR
  reads higher than the node's for strong links.
- **Carrier sense:** a frame counts as being received as soon as it arrives above sensitivity; a real
  SX127x needs a few preamble symbols to detect it.
- **Collisions:** FLoRa's model, unchanged from `lora-v2v`.
