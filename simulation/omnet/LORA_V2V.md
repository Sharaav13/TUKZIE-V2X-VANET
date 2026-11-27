# lora-v2v

Car-to-car LoRa broadcast. SUMO vehicles (Veins 5.3.1 + veins_inet) each
broadcast periodic beacons over a shared 433.6 MHz LoRa channel using a
broadcast-only CSMA/CA MAC with random backoff. No gateways, no network
server. The LoRa PHY (radio, medium, collision/capture model) is FLoRa 1.2.0
on INET 4.5.4. Map: Veins' Erlangen example network.

Traffic (simulations/tools/make_random_routes.sh [seed] [N] [PERIOD] [END], seed 42, N 10, PERIOD 2, END 1000):
- Ordinary cars (vtype0, blue): one new car every 2 s (1.5x the Veins example's
  1 car / 3 s) until t=1000 s, ~500 cars, ~160-175 on the road from t=600 s,
  each on its own random trip, so cars turn differently at intersections. In OMNeT++ they
  are PlainCar modules (position only, no radio) so they are visible; they
  take no part in the LoRa communication or statistics.
- LoRa cars (loraCar, yellow): 10 cars, all departing at t=0 on long random
  routes (>= 1300 s free-flow), so all 10 broadcast for the whole run. They do
  not reroute (keeps their long routes), use impatience=1 and ignore stopped
  foes at junctions (jmIgnoreFoeSpeed 1 m/s), so they don't get stuck in junction
  deadlocks. If a LoRa car is stuck for 300 s anyway (a real jam), SUMO teleports
  it and Veins re-creates its node under a new index: that shows up as 11+ nodes
  in the results. Default run (Veins seed 0): no LoRa teleports, 10 nodes x 200
  beacons. Other seeds: occasionally one.
- Display, both windows: dark lilac background, black roads, green buildings,
  blue ordinary cars, yellow LoRa cars (large icons in OMNeT++).
  OMNeT++: background in LoRaV2V.ned (bgb), road/car colours in omnetpp.ini,
  buildings drawn from erlangen.buildings.xml (INET PhysicalEnvironment, drawing
  only: no obstacle loss, so they do not affect the radio).
  SUMO: simulations/erlangen.view.xml (background, roads, vehicle size 8x);
  buildings green in erlangen.poly.xml. Both building files come from
  tools/make_buildings.py (original polygons: erlangen.poly.xml.orig).
- SUMO settings for long, dense runs (erlangen.sumo.cfg): ordinary cars re-plan
  their route every 60 s around congestion; cars stop waiting on a junction
  blocker after 20 s; minor collisions are logged, not teleported. A few
  ordinary cars (~13 per 1000 s) are still teleported at awkward junctions
  of the Erlangen network.
- 3x traffic (PERIOD 1) is fine for 300 s runs but jams the network over
  1000 s (about half the cars stopped by the end).
- The Veins example's single fixed route is kept as
  simulations/erlangen.rou.xml.orig-single-route.

## Build (once, or after changing src/)
    source ~/TVV/tvv-env.sh
    cd ~/TVV/lora-v2v
    make makefiles
    make MODE=release

## Run
    source ~/TVV/tvv-env.sh
    tvv-run lora-v2v              # GUI (click the OMNeT++ window, press F6)
    tvv-run lora-v2v --sumo-gui   # also show SUMO's traffic window
    tvv-run lora-v2v --cmdenv     # headless, ~1 min
    cd ~/TVV/lora-v2v/simulations
    python3 pdr.py                # PDR vs distance + MAC totals
    python3 plot.py               # results/plots.png

## MAC: LoRaCsmaMac (src/LoRaCsmaMac.*)
For every frame:
 1. channel busy -> wait (DEFER)
 2. channel idle for difsTime -> draw backoff uniform{0..cw} slots
 3. count down slotTime per idle slot; if the channel turns busy the
    countdown freezes and resumes after the next DIFS
 4. counter reaches 0 -> broadcast
No ACKs, no retries, fixed cw (broadcasts get no feedback).
Carrier sense = radio energy detection (receiver.energyDetection).

## Settings (simulations/omnetpp.ini)
| setting                | value            | where                                  |
|------------------------|------------------|----------------------------------------|
| carrier frequency      | 433.6 MHz        | LoRaNic.mac.centerFrequency            |
| SF / BW / CR           | 7 / 125 kHz / 4/5| LoRaNic.mac.spreadingFactor/bandwidth/codingRate |
| TX power               | 10 dBm (EU 433)  | LoRaNic.mac.txPower                    |
| slot / DIFS / cw       | 2.048 ms / 5 ms / 63 | LoRaNic.mac.slotTime/difsTime/cw   |
| carrier-sense threshold| -120 dBm         | LoRaNic.radio.receiver.energyDetection |
| beacon interval/payload| 5 s / 20 B (+8 B MAC header) | app.beaconInterval/payloadLength |
| path loss              | log-distance, alpha 3.5, sigma 6 dB, free-space ref @1 m | LoRaMedium.pathLoss.* |
| time on air            | 66.8 ms (28 B, SF7, CR 4/5) | computed by LoRaV2VTransmitter |
| simulation length      | 1000 s           | sim-time-limit (SUMO end: 1100 s)      |

## Results
simulations/results/General-#0.{sca,vec}
  node[*].app          beaconsSent, beaconsReceived, beaconDelay, rxDistance, rxPower
  node[*].LoRaNic.mac  numSent, numReceived, numDeferred, numBackoffFrozen,
                       numRxLost (receptions lost to collisions), channelAccessDelay

## Files
- src/LoRaCsmaMac.*          broadcast CSMA/CA MAC; also sets the LoRa PHY parameters
- src/LoRaV2VTransmitter.*   Semtech time-on-air from the real frame length
- src/LoRaV2VReceiver.*      FLoRa receiver without its LoRaMac dependency; only
                             attempts frames above sensitivity (else carrier sense
                             would see every transmission on the map)
- src/LoRaBeaconApp.*, V2VBeacon.msg  periodic beacon app + payload
- src/LoRaV2VNic.ned, LoRaV2VCar.ned  interface and vehicle modules
- simulations/LoRaV2V.ned    network (Veins manager + LoRa medium)
- simulations/erlangen.*     SUMO scenario, copied unchanged from Veins
- simulations/pdr.py, plot.py  analysis

## Modelling notes
- Carrier sense is instantaneous: a node senses a transmission as soon as it
  arrives. Real LoRa CAD needs ~1-2 symbols, so simultaneous starts are
  slightly rarer here than on hardware.
- Collisions use FLoRa's model (same-frequency overlap; survives if >= the
  capture threshold above the interferer, or if the overlap misses the
  last 6 preamble symbols).
- Path-loss exponent/shadowing are generic urban V2V values, not 433 MHz
  measurements. Tune alpha/sigma for your environment.
