# Source this file: source ~/TVV/tvv-env.sh
# Sets up OMNeT++, INET, Veins, FLoRa and SUMO for the lora-v2v project.
# TVV is the folder that holds omnetpp-6.1, inet4.5, veins-5.3.1, flora-1.2.0 and lora-v2v
# (default ~/TVV; set TVV before sourcing to use another folder).
TVV=${TVV:-$HOME/TVV}
[ -f "$TVV/omnetpp-6.1/.venv/bin/activate" ] && source "$TVV/omnetpp-6.1/.venv/bin/activate"
source "$TVV/omnetpp-6.1/setenv" -f >/dev/null
pushd "$TVV/inet4.5" >/dev/null && source ./setenv -f >/dev/null; popd >/dev/null
export INET_ROOT="$TVV/inet4.5"
export VEINS_ROOT="$TVV/veins-5.3.1"
export FLORA_ROOT="$TVV/flora-1.2.0"
export SUMO_HOME=${SUMO_HOME:-/usr/share/sumo}
export PATH="$TVV:$VEINS_ROOT/bin:$PATH"   # tvv-run, veins_launchd
