# TVV firmware simulation

OMNeT++ model of the TVV node firmware's **MAC** and **routing** (beacons, cluster, one-hop forwarding), built on the
`lora-v2v` car-to-car LoRa project: SUMO vehicles (Veins) broadcast LoRa beacons over a FLoRa/INET radio channel.

This folder is the complete `lora-v2v` OMNeT++ project. It holds both the base LoRa V2V model (`LoRaCsmaMac` +
`LoRaBeaconApp`, config `General`, described in [`LORA_V2V.md`](LORA_V2V.md)) and the TVV firmware model (`TvvMac` +
`TvvRouting`, described below). It also contains:

- [`docs/TVV_OMNeT_Simulation_Guide.docx`](docs/TVV_OMNeT_Simulation_Guide.docx): a step-by-step guide
- `calibration/`: the channel-calibration scripts `extract_rugby.py` and `fit_channel.py`, run from that folder with
  Python 3, numpy and scipy
- `results/`: summaries of the calibration and sweep results
- `images/`: screenshots of the Erlangen scenario
- `scripts/`: the helper scripts `tvv-env.sh` (environment setup) and `tvv-run` (runs a simulation, with SUMO
  started automatically)

Build outputs (`src/out`, `*.so`, the generated `*_m.cc/h` files, `src/Makefile`) and raw results
(`simulations/results/`) are not in the repo. They are created when you build and run the project.

## Requirements

| Software | Version | Source |
|---|---|---|
| Ubuntu (native, or WSL2 on Windows 10/11) | 24.04 | Tested on 24.04 LTS. Other recent Linux distributions should work with the equivalent packages. |
| OMNeT++ | 6.1.0 | [omnetpp-6.1.0-linux-x86_64.tgz](https://github.com/omnetpp/omnetpp/releases/tag/omnetpp-6.1.0) |
| INET | 4.5.4 | [inet-4.5.4-src.tgz](https://github.com/inet-framework/inet/releases/tag/v4.5.4) |
| FLoRa | 1.2.0 | [florasim/flora](https://github.com/florasim/flora), tag `v1.2.0` |
| Veins + veins_inet | 5.3.1 | [sommer/veins](https://github.com/sommer/veins), tag `veins-5.3.1` |
| SUMO | 1.18 (Ubuntu package) | `apt install sumo sumo-tools` |

Plan for about 4 GB of disk space and an hour or more of build time, most of it for OMNeT++ and INET.
These are the same versions as the original setup, and nothing in them is modified.

### Folder layout

`Makefile` and `simulations/run` find INET, Veins and FLoRa by relative paths, so every component must sit side by
side in one folder. This guide uses `~/TVV`:

```
~/TVV/
├── omnetpp-6.1/
├── inet4.5/
├── flora-1.2.0/
├── veins-5.3.1/        (with subprojects/veins_inet)
├── lora-v2v/           this folder (simulation/omnet of the repo)
├── tvv-env.sh          from scripts/
└── tvv-run             from scripts/
```

To use a folder other than `~/TVV`, run `export TVV=/path/to/folder` before sourcing `tvv-env.sh`. `tvv-run` finds
the folder it is in by itself.

## Setup

Run every command below in a Linux (WSL) terminal, not in Windows PowerShell.

### 0. Windows only: install WSL2

In PowerShell, as administrator:

```powershell
wsl --install -d Ubuntu-24.04
```

Restart if asked, then open "Ubuntu 24.04" from the Start menu and create a user. On Windows 11, WSLg shows the
OMNeT++ and SUMO GUIs with no extra setup. Keep all the files inside the Linux file system (`~/TVV`), not under
`/mnt/c`: building there is far slower.

### 1. System packages

```bash
sudo apt-get update
sudo apt-get install -y build-essential clang lld gdb bison flex perl pkg-config \
    python3 python3-pip python3-venv libpython3-dev \
    qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libqt5opengl5-dev \
    libxml2-dev zlib1g-dev doxygen graphviz libwebkit2gtk-4.1-0 xdg-utils libdw-dev \
    sumo sumo-tools git wget
```

### 2. OMNeT++ 6.1

```bash
mkdir -p ~/TVV && cd ~/TVV
wget https://github.com/omnetpp/omnetpp/releases/download/omnetpp-6.1.0/omnetpp-6.1.0-linux-x86_64.tgz
tar xzf omnetpp-6.1.0-linux-x86_64.tgz          # creates omnetpp-6.1/
cd omnetpp-6.1
python3 -m venv .venv                            # Python packages for OMNeT++ and the analysis scripts
source .venv/bin/activate
python3 -m pip install -r python/requirements.txt
source setenv
./configure WITH_OSG=no WITH_OSGEARTH=no
make -j$(nproc)
```

### 3. INET 4.5.4

```bash
cd ~/TVV
wget https://github.com/inet-framework/inet/releases/download/v4.5.4/inet-4.5.4-src.tgz
tar xzf inet-4.5.4-src.tgz                       # creates inet4.5/
cd inet4.5
source setenv
make makefiles
make -j$(nproc) MODE=release
```

### 4. FLoRa 1.2.0

```bash
cd ~/TVV
git clone --branch v1.2.0 --depth 1 https://github.com/florasim/flora.git flora-1.2.0
cd flora-1.2.0
make makefiles INET_DIR=../inet4.5
make -j$(nproc) MODE=release
```

### 5. Veins 5.3.1 and veins_inet

```bash
cd ~/TVV
git clone --branch veins-5.3.1 --depth 1 https://github.com/sommer/veins.git veins-5.3.1
cd veins-5.3.1
./configure
make -j$(nproc) MODE=release
cd subprojects/veins_inet
./configure --with-inet=../../../inet4.5
make -j$(nproc) MODE=release
```

### 6. This project

Clone the repo inside WSL/Linux, then copy this folder and the helper scripts into `~/TVV`:

```bash
cd ~
git clone https://github.com/Sharaav13/TUKZIE-V2X-VANET.git
cp -r ~/TUKZIE-V2X-VANET/simulation/omnet ~/TVV/lora-v2v
cp ~/TVV/lora-v2v/scripts/tvv-env.sh ~/TVV/lora-v2v/scripts/tvv-run ~/TVV/
chmod +x ~/TVV/tvv-run ~/TVV/lora-v2v/simulations/run ~/TVV/lora-v2v/simulations/tools/*.sh

source ~/TVV/tvv-env.sh
cd ~/TVV/lora-v2v
make makefiles
make -j$(nproc) MODE=release                     # builds src/liblorav2v.so
```

Run `source ~/TVV/tvv-env.sh` in every new terminal before building or running. If you change `src/`, run `make` again,
and also `make makefiles` first if you added a `.cc` or `.msg` file.

To check the toolchain, `tvv-run flora --cmdenv` runs FLoRa's own example and `tvv-run veins --cmdenv` runs Veins'
example.

## Running the simulations

### Mobile configs (SUMO, Erlangen map)

`tvv-run` starts SUMO (through `veins_launchd`) on a free port, runs the simulation, and stops SUMO afterwards.

```bash
source ~/TVV/tvv-env.sh

tvv-run lora-v2v                                            # base model (General), GUI: click the window, press F6 to run
tvv-run lora-v2v --cmdenv                                   # base model, headless (about 1 min)
tvv-run lora-v2v --config TvvFirmware                       # TVV firmware model, GUI
tvv-run lora-v2v --config TvvFirmware --cmdenv              # TVV firmware model, headless
tvv-run lora-v2v --config TvvFirmware --sumo-gui            # also show SUMO's traffic window
tvv-run lora-v2v --config TvvFirmwareForwarding --cmdenv    # addressed beacons, so peers forward them

cd ~/TVV/lora-v2v/simulations
python3 pdr.py  "results/TvvFirmware-#0.vec"                # PDR vs distance + totals
python3 plot.py "results/TvvFirmware-#0.vec"                # writes results/plots.png
```

Other mobile configs: `TvvFirmwareEnergySense` and `TvvFirmwareBigCluster` (see `simulations/tvv.ini`). Results go to
`~/TVV/lora-v2v/simulations/results/`. With no argument, `pdr.py` and `plot.py` read `results/General-#0.vec`.

### Stationary configs (no SUMO)

`TvvStatic` (network `TvvStatic.ned`) puts the nodes at fixed positions, so these configs run without SUMO and without
`tvv-run`. `opp_runall` runs all the runs of a config (every parameter value and seed) in parallel:

```bash
source ~/TVV/tvv-env.sh
cd ~/TVV/lora-v2v/simulations
opp_runall -j$(nproc) ./run -u Cmdenv -c TvvValidation               # two nodes D = 10..700 m apart, 5 seeds x 1 h
opp_runall -j$(nproc) ./run -u Cmdenv -c TvvValidationUncalibrated   # the same on the old channel, D = 100..9000 m
opp_runall -j$(nproc) ./run -u Cmdenv -c SMO1                        # SM/O1: N = 2, 5, 10, 20, 40 nodes, 10 seeds x 1 h
opp_runall -j$(nproc) ./run -u Cmdenv -c SMO1BigCluster              # SM/O1 with an unlimited cluster
opp_runall -j$(nproc) ./run -u Cmdenv -c SMO2                        # SM/O2: 10 nodes, back-off max 0, 1.5, 3, 6, 12 s
opp_runall -j$(nproc) ./run -u Cmdenv -c SMO1CsDetect                # sensitivity: carrier sense needs 4 preamble symbols
opp_runall -j$(nproc) ./run -u Cmdenv -c SMO2CsDetect

python3 tvvstats.py validation results/TvvValidation-D*.sca    # PDR vs distance
python3 tvvstats.py sweep N results/SMO1-*.sca                 # metric tables, mean ± 95 % CI over seeds
python3 tvvstats.py sweep B results/SMO2-*.sca
```

A single run: `./run -u Cmdenv -c SMO1 -r 2` (run 2 of `SMO1`). `./run -c SMO1 -q runs` lists the runs of a config.

### Troubleshooting

- **`opp_run: command not found`, or missing libraries:** run `source ~/TVV/tvv-env.sh` in this terminal.
- **`src/Makefile does not exist`:** run `make makefiles` in `~/TVV/lora-v2v` first.
- **Cannot load `libINET.so`/`libflora.so`/`libveins_inet.so`:** a component is missing or sits at the wrong path.
  Check the folder layout above and that each component's `make` finished.
- **`/bin/sh^M` or `bad interpreter`:** the scripts got Windows line endings (cloned on Windows). Clone inside WSL, or
  run `sed -i 's/\r$//' ~/TVV/tvv-run ~/TVV/tvv-env.sh ~/TVV/lora-v2v/simulations/run`.
- **SUMO connection errors:** see `simulations/veins_launchd.log`. `tvv-run` picks a free port from 9999 upwards, so
  several runs can share one machine.
- **No GUI window under WSL:** update WSL (`wsl --update` in PowerShell), or use `--cmdenv`.

## Files

| File | Firmware | Role |
|---|---|---|
| `src/TvvMac.*` | `Vanet` (vanet.h / vanet.cpp) | MAC: beacon timing, back-off, carrier sense, Tx/Rx of beacons and forwards; also sets the LoRa PHY |
| `src/TvvRouting.*` | `Packet`, `readPacket()`, `Cluster`, forwarding | Routing: builds beacons, dest filter, repeat check, forward queue, cluster of peers |
| `src/TvvPacket.msg` | `struct Packet` | The 25-byte packet (24 B here + TvvMac's 1 B header, see below) |
| `src/ITvvBeaconSource.h` | `copyMyself()` + `vanet.tx()`, `vanet.forward()` | Lets TvvMac build the beacon, or take the next forward, when it gets the channel |
| `src/TvvNic.ned`, `src/TvvCar.ned` | | LoRa interface and vehicle |
| `src/TvvNode.ned` | | TVV node at a fixed position (StationaryMobility), no SUMO |
| `src/LoRaV2VTransmitter.*`, `src/LoRaV2VReceiver.*` | | LoRa PHY shared by both models: time on air, receiver without FLoRa's LoRaMac dependency |
| `src/LoRaCsmaMac.*`, `src/LoRaBeaconApp.*`, `src/V2VBeacon.msg` | | Base model: broadcast CSMA/CA MAC and periodic beacon app |
| `src/LoRaV2VNic.ned`, `src/LoRaV2VCar.ned`, `src/PlainCar.ned` | | Base model's interface and vehicle; ordinary cars with no radio |
| `simulations/LoRaV2V.ned` | | Network: Veins manager, LoRa medium, Erlangen map (base model) |
| `simulations/TvvV2V.ned` | | Network: the LoRaV2V scenario with TvvCar nodes |
| `simulations/TvvStatic.ned` | | Network: `numNodes` TvvNode at positions set in `tvv.ini` |
| `simulations/omnetpp.ini` | | Base config `General`; includes `tvv.ini` |
| `simulations/tvv.ini` | `system.h` | Configs `TvvFirmware`, `TvvFirmwareEnergySense`, `TvvFirmwareBigCluster`, `TvvFirmwareForwarding`; stationary `TvvValidation`, `TvvValidationUncalibrated`, `SMO1`, `SMO1BigCluster`, `SMO2`, `SMO1CsDetect`, `SMO2CsDetect` |
| `simulations/erlangen.*` | | SUMO scenario (Veins' Erlangen example, with LoRa cars and random routes) |
| `simulations/tools/` | | `make_random_routes.sh` (traffic), `make_buildings.py` (building colours) |
| `simulations/run` | | Runs `opp_run` with the INET, Veins and FLoRa libraries and NED paths |
| `simulations/pdr.py`, `plot.py` | | PDR vs distance, totals and plots for one run |
| `simulations/tvvstats.py` | | Tables (mean ± 95 % CI over seeds) for the stationary configs |
| `Makefile` | | `make makefiles` / `make`: builds `src/liblorav2v.so` against INET and FLoRa |

## TVV configs

- **Cluster layout** (`TvvCluster`, base of `SMO1`/`SMO2`): N nodes in a disc of radius 25 m (largest separation
  50 m, calibrated PDR about 99.8 %), on a sunflower pattern (node i at r = 25 m·sqrt((i + 0.5)/N), angle i·137.5°),
  so every node hears every other.
- **Traffic:** `routing.addressedFraction = 0.1`: each beacon is addressed to a random other node with
  probability 0.1 (the report's 90 % broadcast / 10 % addressed assumption), so the other nodes forward it once.
- **Warm-up:** 1 h runs, the first 300 s discarded (`warmup-period`). The counts used for the tables are signal
  statistics (`heardDirect:sum`, `seqLost:sum`, `rxLost:sum`, ...), so they honour the warm-up; the `recordScalar`
  counters cover the whole run.
- Results of the calibration and the sweeps: `results/SMO1_results.txt` and `results/SMO2_results.txt` in this folder.

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
| `node[*].routing` | `beaconsSent`, `beaconsReceived` (for me, direct or forwarded), `beaconsReceivedForwarded`, `beaconDelay` (+ `beaconDelayDirect`, `beaconDelayForwarded`), `rxDistance`, `txNeighborDistance`, `rxPower` (pktRssi), `rxSnr`, `clusterSize`, `numHeardDirect` / `heardDirect` (frames heard straight from their sender, any dest), `numSeqLost` / `seqLost` (gaps in those senders' seq, as `tests/test_analysis.m`), `numClusterFull` / `clusterFull`, `numPeersAdded` / `peerAdded`, `numPeersRemoved` / `peerRemoved`, `numAddressedSent` / `addressedSent`, `numAddressedReceived` / `addressedReceived` (every copy read), `numAddressedUnique` / `addressedUnique` (first copy of each packet), `forwardedCopyDelay` (every forwarded copy reaching its dest, before the repeat check), `numWrongSize`, `numDuplicates` (repeats dropped), `numForwardQueued`, `numTtlExpired`, `numFwdQueueFull`, `nodeId` |
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
  fitted to the Bob -> Alice packets of the UCT rugby field test (see `results/SMO1_results.txt`): PDR about 99.8 %
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
