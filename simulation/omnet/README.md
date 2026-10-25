# TVV firmware simulation

OMNeT++ model of the TVV node firmware's **MAC** and **routing** (beacons, cluster, one-hop forwarding), for the `lora-v2v` project
(OMNeT++ 6.1, INET 4.5, FLoRa 1.2.0, Veins 5.3.1 + SUMO, in `~/TVV`). It reuses the project's LoRa PHY
(`LoRaV2VTransmitter`, `LoRaV2VReceiver`), channel, SUMO scenario and analysis scripts.

## Files

| File | Firmware | Role |
|---|---|---|
| `src/TvvMac.*` | `Vanet` (vanet.h / vanet.cpp) | MAC: beacon timing, back-off, carrier sense, Tx/Rx of beacons and forwards; also sets the LoRa PHY |
| `src/TvvRouting.*` | `Packet`, `readPacket()`, `Cluster`, forwarding | Routing: builds beacons, dest filter, repeat check, forward queue, cluster of peers |
| `src/TvvPacket.msg` | `struct Packet` | The 25-byte packet (24 B here + TvvMac's 1 B header, see below) |
| `src/ITvvBeaconSource.h` | `copyMyself()` + `vanet.tx()`, `vanet.forward()` | Lets TvvMac build the beacon, or take the next forward, when it gets the channel |
| `src/TvvNic.ned`, `src/TvvCar.ned` | | LoRa interface and vehicle |
| `src/TvvNode.ned` | | TVV node at a fixed position (StationaryMobility), no SUMO |
| `simulations/TvvV2V.ned` | | Network: the LoRaV2V scenario with TvvCar nodes |
| `simulations/TvvStatic.ned` | | Network: `numNodes` TvvNode at positions set in `tvv.ini` |
| `simulations/tvv.ini` | `system.h` | Configs `TvvFirmware`, `TvvFirmwareEnergySense`, `TvvFirmwareBigCluster`, `TvvFirmwareForwarding`; stationary `TvvValidation`, `TvvValidationUncalibrated`, `SMO1`, `SMO1BigCluster`, `SMO2`, `SMO1CsDetect`, `SMO2CsDetect` |
| `simulations/tvvstats.py` | | Tables (mean ± 95 % CI over seeds) for the stationary configs |

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
tvv-run lora-v2v --config TvvFirmwareForwarding --cmdenv    # addressed beacons, so peers forward them

cd ~/TVV/lora-v2v/simulations
python3 pdr.py  "results/TvvFirmware-#0.vec"      # PDR vs distance + totals
python3 plot.py "results/TvvFirmware-#0.vec"
```

The existing `General` config (LoRaCsmaMac + LoRaBeaconApp) is unchanged, so both can be compared.

### Stationary configs (no SUMO)

`TvvStatic` (network `TvvStatic.ned`) puts the nodes at fixed positions, so these run without SUMO and without
`tvv-run`:

```bash
source ~/TVV/tvv-env.sh
cd ~/TVV/lora-v2v/simulations
opp_runall -j16 ./run -u Cmdenv -c TvvValidation               # two nodes D = 10..700 m apart, 5 seeds x 1 h
opp_runall -j16 ./run -u Cmdenv -c TvvValidationUncalibrated   # the same on the old channel, D = 100..9000 m
opp_runall -j16 ./run -u Cmdenv -c SMO1                        # SM/O1: N = 2, 5, 10, 20, 40 nodes, 10 seeds x 1 h
opp_runall -j16 ./run -u Cmdenv -c SMO1BigCluster              # SM/O1 with an unlimited cluster
opp_runall -j16 ./run -u Cmdenv -c SMO2                        # SM/O2: 10 nodes, back-off max 0, 1.5, 3, 6, 12 s
opp_runall -j16 ./run -u Cmdenv -c SMO1CsDetect                # sensitivity: carrier sense needs 4 preamble symbols
opp_runall -j16 ./run -u Cmdenv -c SMO2CsDetect

python3 tvvstats.py validation results/TvvValidation-D*.sca    # PDR vs distance
python3 tvvstats.py sweep N results/SMO1-*.sca                 # metric tables, mean ± 95 % CI over seeds
python3 tvvstats.py sweep B results/SMO2-*.sca
```

- **Cluster layout** (`TvvCluster`, base of `SMO1`/`SMO2`): N nodes in a disc of radius 25 m (largest separation
  50 m, calibrated PDR about 99.8 %), on a sunflower pattern (node i at r = 25 m·sqrt((i + 0.5)/N), angle i·137.5°),
  so every node hears every other.
- **Traffic:** `routing.addressedFraction = 0.1`: each beacon is addressed to a random other node with
  probability 0.1 (the report's 90 % broadcast / 10 % addressed assumption), so the other nodes forward it once.
- **Warm-up:** 1 h runs, the first 300 s discarded (`warmup-period`). The counts used for the tables are signal
  statistics (`heardDirect:sum`, `seqLost:sum`, `rxLost:sum`, ...), so they honour the warm-up; the `recordScalar`
  counters cover the whole run.
- Results of the calibration and the sweeps: `simulation/SMO1_results.txt` and `simulation/SMO2_results.txt` in the repo.

## MAC (TvvMac)

As in `vanet.cpp`, with the `system.h` values as defaults:

1. **First beacon:** `lastTx = now - random(0, BEACON_INTVL)`, so nodes start out of step.
2. **A beacon or forward goes out only when all three hold** (`txDue()`):
   - `BEACON_INTVL` (15 s) has passed since `lastTx`, the **end** of the previous beacon or forward,
   - the back-off has ended,
   - no LoRa frame is being received (`receiving()`, RegModemStat: signal detected, synchronised, header valid).

   Otherwise the beacon is checked again on the next `v2vLoop()` pass (`pollInterval`, 1 ms), or when the back-off ends.
3. **Back-off:** every RxDone starts a new back-off of `random(BACKOFF_MIN_MS, BACKOFF_MAX_MS + 1)` ms (0 to 6000 ms), replacing any running one.
   RxDone also fires for frames with a CRC error, so collided frames start it too, as do frames the routing
   layer then drops (dest or size).
4. **Tx:** the beacon is built at that moment from the node's current state, sent as one broadcast frame, then
   the radio returns to continuous Rx. No beacon queue, ACKs or retries. The radio is half duplex.
5. **Forwards:** a packet queued by the routing shares the beacon's transmission slot (`forwardDue()` needs
   `txDue()`). As `v2vLoop()` checks `forwardDue()` before `beaconDue()`, a queued forward takes the slot and the
   beacon waits for the next one. A forward sets `lastTx` like a beacon, so transmissions are always at least
   `BEACON_INTVL` apart and the duty cycle stays at 9.9 % whatever the forwarding load.

`csDetectionTime` (default 0) makes carrier sense see a frame only once it has been arriving that long, as the
SX127x needs a few preamble symbols before RegModemStat shows "signal detected"; `SMO1CsDetect`/`SMO2CsDetect` use
131 ms (4 symbols at SF12/125 kHz).

`carrierSense = "energy"` (config `TvvFirmwareEnergySense`) also treats energy above the receiver's
`energyDetection` as busy. The node does not do this; it is there for comparison.

## Routing (TvvRouting)

Every node broadcasts its own beacon, keeps the peers it hears, and forwards packets addressed to its peers once:

- **Tx:** `seq` = previous + 1 (wraps at 65535), `ttl` = `ttlInit` (2), `dest` = `destId` (0 = broadcast).
- **Tx, simulation only:** with `addressedFraction` > 0, that share of beacons gets `dest` = a random other
  node's ID instead of `destId`.
- **Rx:**
  1. wrong-size frames are ignored;
  2. a repeat of a packet (`src`, `seq`) read within `dupWindow` (90 s, last `dupHistory` = 48 packets) is dropped;
  3. packets with `dest == nodeId` or `dest == 0` are read;
  4. a packet addressed to any other node is queued, exactly as received with `ttl` − 1 (not queued at 0, or when
     `fwdQueueSize` = 2 are waiting), and TvvMac sends it;
  5. a packet for me heard straight from its sender (`ttl == ttlInit`) adds or refreshes the sender in the
     cluster; a packet forwarded to me (lower `ttl`) is read, but its sender is not a peer;
  6. the latest message is kept.
- **Cluster:** at most `maxPeers` (3). A new peer is not added while it is full. Peers not heard for
  `peerTimeout` (30 s) are removed every `peerCheckInterval` (1 s). The message shown is cleared once its sender
  is not in the cluster and it is older than `peerTimeout`.
- **Forwarding needs addressed packets:** broadcasts are never forwarded. `TvvFirmwareForwarding` has even vehicles
  address the next vehicle (vehicle 0 addresses ID 2, vehicle 2 addresses ID 4, ...) while odd vehicles broadcast;
  every other vehicle that hears an addressed beacon forwards it once.

Node IDs default to vehicle index + 1 (0 is the broadcast address). Set `routing.nodeId` per node to choose them.

## Results

`simulations/results/<config>-#0.{sca,vec}`, per node:

| Module | Statistics |
|---|---|
| `node[*].routing` | `beaconsSent`, `beaconsReceived` (for me, direct or forwarded), `beaconsReceivedForwarded`, `beaconDelay` (+ `beaconDelayDirect`, `beaconDelayForwarded`), `rxDistance`, `txNeighborDistance`, `rxPower` (pktRssi), `rxSnr`, `clusterSize`, `numHeardDirect` / `heardDirect` (frames heard straight from their sender, any dest), `numSeqLost` / `seqLost` (gaps in those senders' seq, as `tests/perf_analysis.m`), `numClusterFull` / `clusterFull`, `numPeersAdded` / `peerAdded`, `numPeersRemoved` / `peerRemoved`, `numAddressedSent` / `addressedSent`, `numAddressedReceived` / `addressedReceived` (every copy read), `numAddressedUnique` / `addressedUnique` (first copy of each packet), `forwardedCopyDelay` (every forwarded copy reaching its dest, before the repeat check), `numWrongSize`, `numDuplicates` (repeats dropped), `numForwardQueued`, `numTtlExpired`, `numFwdQueueFull`, `nodeId` |
| `node[*].LoRaNic.mac` | `numSent` (beacons), `numForwarded`, `numReceived` / `rxOk`, `numRxLost` / `rxLost` (CRC errors), `frameSent` (count = frames, sum = forwards), `numBackoffs`, `numBlockedByBackoff`, `numBlockedByReceiving`, `channelAccessDelay` (due to Tx start), `txInterval`, `backoffDuration` |

The names in pairs are the whole-run counter (`recordScalar`) and the signal statistic, which honours `warmup-period`.
PDR per receiver = `heardDirect:sum` / (`heardDirect:sum` + `seqLost:sum`). `numSeqLost` now counts gaps over every
frame heard directly (also those addressed to another node), so addressed traffic no longer reads as lost beacons.

`beaconDelay` and `rxDistance` are end to end: from the original sender's Tx and position, also for a packet
received through a forwarder. `beaconDelay` includes `routing.rxProcessingDelay` (25.5 ms in `TvvFirmware`: the
measured latency minus time on air of the PPS-timed field packets, i.e. node processing from RxDone to the timestamp).

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
- **Not modelled:** frequency error and the GPS/host payload (the payload fields are 0, battery 100 %),
  and loop delays from SD writes (`pollInterval` covers the 1 ms loop only).
- **Channel (calibrated):** `TvvFirmware` sets `LogNormalShadowing` alpha 2.77, sigma 5.8 dB, systemLoss 58.3 dB,
  fitted to the Bob -> Alice packets of the UCT rugby field test (see `simulation/SMO1_results.txt`): PDR about 99.8 %
  to 50 m, 90 % at 100 m, 50 % at about 200 m, 20 % at 300 m. `[General]` keeps alpha 3.5, sigma 6 dB, free space at
  1 m (50 % PDR at about 3 km). The shadowing is drawn independently for every frame (no spatial correlation).
- **Sensitivity and noise:** FLoRa decodes a frame above its sensitivity table (-137 dBm at SF12/125 kHz), which the
  field fit reproduces (-137.1 dBm, an SNR of -19.8 dB over the measured -117.3 dBm noise floor). FLoRa's noise in the
  SNIR is that same -137 dBm, not the real -117 dBm floor, so `rxSnr` reads about 20 dB higher than the node's SNR
  (node SNR ≈ `rxPower` + 117.3 dB).
- **Time on air:** 1482.75 ms for the 25 B frame (SF12, BW125, CR4/5, 8-symbol preamble, explicit header, CRC, low
  data rate optimisation), as the theoretical value; the nodes measured 1482.98-1483.00 ms (TxDone interrupt latency).
- **SNR:** FLoRa's SNIR is not capped. A real SX127x reports at most about +10 dB, so the simulated SNR
  reads higher than the node's for strong links.
- **Carrier sense:** by default a frame counts as being received as soon as it arrives above sensitivity; a real
  SX127x needs a few preamble symbols to detect it (`csDetectionTime`).
- **Collisions:** FLoRa's model, unchanged from `lora-v2v`.
