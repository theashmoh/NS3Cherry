#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
# Author: Jesse Chiu (Kuan-Po) <kchiu5@uw.edu>, University of Washington, Seattle
"""Beam edge (README example 4): DL SINR against elevation for UEs at 0, 25 and 50 km."""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

OUT = Path(sys.argv[1] if len(sys.argv) > 1 else "results/ntn-beam-edge")
RADII_KM = [0, 25, 50]  # --ueRadiiKm of run_ntn_beam_edge.sh; UE i attaches as RNTI i + 1

rows = []
for d in sorted(OUT.glob("e*/"), key=lambda p: int(p.name[1:])):
    t = pd.read_csv(d / "RxPacketTrace.txt", sep="\t")
    t.columns = [c.strip() for c in t.columns]
    t = t[t["direction"] == "DL"]
    if t["cellId"].nunique() != 1:
        sys.exit(f"{d}: the UEs attached to different satellites; RNTIs no longer map to radii")
    for rnti, sinr in t.groupby("rnti")["SINR(dB)"]:
        if rnti > len(RADII_KM):
            # A UE that loses the link re-attaches under a new RNTI; leave that tail out.
            print(f"{d.name}: RNTI {rnti} is a re-attachment, {len(sinr)} TBs left out")
            continue
        rows.append(dict(elev=int(d.name[1:]), radius_km=RADII_KM[rnti - 1],
                         p10=sinr.quantile(0.1), p50=sinr.median(), p90=sinr.quantile(0.9)))
df = pd.DataFrame(rows)

fig, ax = plt.subplots(figsize=(6.4, 4))
for radius, g in df.groupby("radius_km"):
    ax.plot(g["elev"], g["p50"], "o-", ms=4, label=f"{radius} km")
    ax.fill_between(g["elev"], g["p10"], g["p90"], alpha=0.2)
ax.set(xlabel="elevation (deg)", ylabel="DL SINR (dB): median, 10-90 %")
ax.legend(title="from beam centre")
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / "beam-edge-sinr.png", dpi=120)
df.to_csv(OUT / "beam-edge-sinr.csv", index=False)
print(df.pivot(index="elev", columns="radius_km", values="p50").round(1).to_string())
print(f"-> {OUT}/beam-edge-sinr.png")
