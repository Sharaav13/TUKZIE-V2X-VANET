#!/usr/bin/env python3
"""Packet delivery ratio vs. distance for the LoRa V2V simulation.

Usage:  python3 pdr.py [results/General-#0.vec] [bin size in m, default 100]

PDR(bin) = beacons received from senders at that distance
           / (beacons sent x other vehicles at that distance)
"""
import csv, os, subprocess, sys, tempfile

vec = sys.argv[1] if len(sys.argv) > 1 else "results/General-#0.vec"
binsize = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
sca = vec[:-4] + ".sca"

with tempfile.TemporaryDirectory() as tmp:
    out = os.path.join(tmp, "v.csv")
    subprocess.run(["opp_scavetool", "export", "-F", "CSV-R", "-o", out,
                    "-f", 'name =~ "rxDistance:vector" OR name =~ "txNeighborDistance:vector"', vec],
                   check=True, stdout=subprocess.DEVNULL)
    csv.field_size_limit(sys.maxsize)
    rx, pot = {}, {}
    with open(out, newline="") as f:
        for row in csv.DictReader(f):
            if row["type"] != "vector":
                continue
            target = rx if row["name"].startswith("rxDistance") else pot
            for v in row["vecvalue"].split():
                b = int(float(v) // binsize)
                target[b] = target.get(b, 0) + 1

print(f"{'distance [m]':>15} {'potential':>10} {'received':>10} {'PDR':>7}")
for b in sorted(pot):
    lo, hi = b * binsize, (b + 1) * binsize
    n, r = pot[b], rx.get(b, 0)
    print(f"{lo:7.0f}-{hi:<7.0f} {n:10d} {r:10d} {r / n:7.3f}")

# MAC/app totals from the scalar file
totals = {}
with tempfile.TemporaryDirectory() as tmp:
    out = os.path.join(tmp, "s.csv")
    subprocess.run(["opp_scavetool", "export", "-F", "CSV-R", "-o", out,
                    "-f", 'name =~ "num*" OR name =~ "beacons*" OR name =~ "beaconDelay:mean" OR name =~ "channelAccessDelay:mean"', sca],
                   check=True, stdout=subprocess.DEVNULL)
    with open(out, newline="") as f:
        for row in csv.DictReader(f):
            if row["type"] != "scalar" or row["value"] == "":
                continue
            v = float(row["value"])
            if v == v:  # skip NaN (e.g. vehicles that never transmitted)
                totals.setdefault(row["name"], []).append(v)
print()
for k in ["beaconsSent", "beaconsReceived", "numSent", "numReceived", "numDeferred", "numBackoffFrozen", "numRxLost"]:
    if k in totals:
        print(f"{k:>22}: {sum(totals[k]):.0f}")
for k in ["channelAccessDelay:mean", "beaconDelay:mean"]:
    if k in totals:
        v = totals[k]
        print(f"{k:>22}: {1000 * sum(v) / len(v):.1f} ms (mean over vehicles)")
