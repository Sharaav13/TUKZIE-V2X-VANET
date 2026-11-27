#!/bin/bash
# Random traffic for the Erlangen network (replaces the Veins example's single
# fixed route), written to two SUMO route files:
#  - erlangen.rou.xml:      ordinary cars (vtype0, blue), each with its own random
#                           trip; one new car every PERIOD s (default 2 s = 1.5x the
#                           Veins example's 1 car / 3 s; 1 s = 3x jams the network
#                           over 1000 s), departing until t=END s.
#  - erlangen.lora.rou.xml: N LoRa cars (loraCar, yellow), all departing at t=0 on
#                           long random routes (>= END + 300 s free-flow driving)
#                           so they stay on the road for the whole simulation.
#   tools/make_random_routes.sh [seed] [N] [PERIOD] [END]     (run from simulations/)
#   END should match sim-time-limit in omnetpp.ini (default 1000 s).
set -e
cd "$(dirname "$0")/.."
export SUMO_HOME=${SUMO_HOME:-/usr/share/sumo}
SEED=${1:-42}
N=${2:-10}
PERIOD=${3:-2}
END=${4:-1000}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# vehicle types: the original Veins vtype0, recoloured blue, plus loraCar (yellow)
python3 -I - erlangen.rou.xml.orig-single-route "$TMP" <<'PY'
import sys, xml.etree.ElementTree as ET
vt = ET.parse(sys.argv[1]).getroot().find("vType")
vt.set("color", "0,0,1")
out = ET.Element("routes"); out.append(vt)
ET.ElementTree(out).write(sys.argv[2] + "/vtype.xml", encoding="utf-8", xml_declaration=True)
lora = ET.Element("vType", dict(vt.attrib)); lora.set("id", "loraCar"); lora.set("color", "1,1,0")
# Keep LoRa cars from getting stuck at junctions (after 300 s SUMO teleports a stuck
# car; Veins then deletes and re-creates its node): impatience=1 accepts smaller gaps,
# and foes slower than 1 m/s (i.e. stopped/deadlocked) are ignored. Moving traffic is
# still yielded to.
lora.set("impatience", "1")
lora.set("jmIgnoreFoeProb", "1")
lora.set("jmIgnoreFoeSpeed", "1")
ET.SubElement(lora, "param", {"key": "has.rerouting.device", "value": "false"})  # keep long routes
out = ET.Element("routes"); out.append(lora)
ET.ElementTree(out).write(sys.argv[2] + "/loratype.xml", encoding="utf-8", xml_declaration=True)
PY

python3 -I $SUMO_HOME/tools/randomTrips.py -n erlangen.net.xml -o "$TMP/trips.xml" -r "$TMP/routes.xml" \
    -b 0 -e "$END" --period "$PERIOD" --fringe-factor 5 --min-distance 500 --seed "$SEED" --validate \
    --prefix veh --trip-attributes 'type="vtype0" departLane="best" departSpeed="max"' \
    --additional-files "$TMP/vtype.xml" >/dev/null
cp "$TMP/routes.xml" erlangen.rou.xml

# LoRa cars: 60 long candidates, keep the first N that drive >= END + 300 s (free flow)
python3 -I $SUMO_HOME/tools/randomTrips.py -n erlangen.net.xml -o "$TMP/ltrips.xml" -r "$TMP/lroutes.xml" \
    -b 0 -e 60 --period 1 --intermediate 30 --min-distance 800 --seed "$((SEED + 1))" --validate \
    --prefix lora --trip-attributes 'type="loraCar" departLane="best" departSpeed="max"' \
    --additional-files "$TMP/loratype.xml" >/dev/null
/usr/bin/python3 -I - "$TMP/lroutes.xml" erlangen.lora.rou.xml "$N" "$((END + 300))" <<'PY'
import sys, xml.etree.ElementTree as ET
sys.path.append("/usr/share/sumo/tools")
import sumolib
net = sumolib.net.readNet("erlangen.net.xml")
tree = ET.parse(sys.argv[1]); root = tree.getroot(); n = int(sys.argv[3]); min_dur = float(sys.argv[4])
vmax = float(root.find("vType").get("maxSpeed"))
def duration(edges):
    return sum(net.getEdge(e).getLength() / min(vmax, net.getEdge(e).getSpeed()) for e in edges)
kept = 0
for v in root.findall("vehicle"):
    if kept < n and duration(v.find("route").get("edges").split()) >= min_dur:
        v.set("id", f"lora{kept}"); v.set("depart", "0.00"); kept += 1
    else:
        root.remove(v)
if kept < n:
    sys.exit(f"only {kept} LoRa routes of >= {min_dur:.0f} s found, need {n}")
ET.indent(root)
tree.write(sys.argv[2], encoding="utf-8", xml_declaration=True)
PY
echo "wrote erlangen.rou.xml ($(grep -c '<vehicle ' erlangen.rou.xml) ordinary cars) and" \
     "erlangen.lora.rou.xml ($(grep -c '<vehicle ' erlangen.lora.rou.xml) LoRa cars), seed $SEED, period $PERIOD s, until $END s"
