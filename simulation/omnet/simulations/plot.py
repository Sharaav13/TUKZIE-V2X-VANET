#!/usr/bin/env python3
"""Plots for the LoRa V2V simulation.

Usage:  python3 plot.py [results/General-#0.vec]
Writes results/plots.png (PDR vs distance, RSSI vs distance,
beacon delay, channel access delay).
"""
import csv, os, subprocess, sys, tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

vec = sys.argv[1] if len(sys.argv) > 1 else "results/General-#0.vec"
names = ["rxDistance", "txNeighborDistance", "rxPower", "beaconDelay", "channelAccessDelay"]

data = {n: [] for n in names}
rssi_dist = []  # (distance, rssi) pairs per receiver
with tempfile.TemporaryDirectory() as tmp:
    out = os.path.join(tmp, "v.csv")
    flt = " OR ".join(f'name =~ "{n}:vector"' for n in names)
    subprocess.run(["opp_scavetool", "export", "-F", "CSV-R", "-o", out, "-f", flt, vec],
                   check=True, stdout=subprocess.DEVNULL)
    csv.field_size_limit(sys.maxsize)
    per_module = {}
    with open(out, newline="") as f:
        for row in csv.DictReader(f):
            if row["type"] != "vector":
                continue
            name = row["name"].split(":")[0]
            values = np.array(row["vecvalue"].split(), dtype=float)
            data[name].append(values)
            if name in ("rxDistance", "rxPower"):
                per_module.setdefault(row["module"].rsplit(".", 1)[0], {})[name] = values
    for m in per_module.values():
        if "rxDistance" in m and "rxPower" in m and len(m["rxDistance"]) == len(m["rxPower"]):
            rssi_dist.append(np.column_stack([m["rxDistance"], m["rxPower"]]))

cat = {n: (np.concatenate(v) if v else np.array([])) for n, v in data.items()}

fig, ax = plt.subplots(2, 2, figsize=(12, 9))
fig.suptitle("LoRa V2V broadcast (433.6 MHz, SF7, CSMA/CA)")

binsize = 100
edges = np.arange(0, max(cat["txNeighborDistance"].max(), cat["rxDistance"].max()) + binsize, binsize)
pot, _ = np.histogram(cat["txNeighborDistance"], edges)
rcv, _ = np.histogram(cat["rxDistance"], edges)
mask = pot > 0
centers = (edges[:-1] + edges[1:]) / 2
ax[0, 0].plot(centers[mask], rcv[mask] / pot[mask], "o-")
ax[0, 0].set(title="Packet delivery ratio vs distance", xlabel="sender-receiver distance [m]",
             ylabel="PDR", ylim=(0, 1.05))
ax[0, 0].grid(True)

if rssi_dist:
    rd = np.concatenate(rssi_dist)
    ax[0, 1].scatter(rd[:, 0], rd[:, 1], s=2, alpha=0.2)
ax[0, 1].axhline(-124, color="red", ls="--", label="SF7 sensitivity (-124 dBm)")
ax[0, 1].set(title="Received power vs distance", xlabel="distance [m]", ylabel="RSSI [dBm]")
ax[0, 1].legend()
ax[0, 1].grid(True)

for a, n, title in [(ax[1, 0], "beaconDelay", "Beacon delay (creation to reception)"),
                    (ax[1, 1], "channelAccessDelay", "MAC channel access delay")]:
    v = np.sort(cat[n]) * 1000
    if len(v):
        a.plot(v, np.arange(1, len(v) + 1) / len(v))
    a.set(title=title, xlabel="delay [ms]", ylabel="CDF", ylim=(0, 1.02))
    a.grid(True)

fig.tight_layout()
png = os.path.join(os.path.dirname(vec) or ".", "plots.png")
fig.savefig(png, dpi=120)
print("wrote", png)
