#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
# Author: Jesse Chiu (Kuan-Po) <kchiu5@uw.edu>, University of Washington, Seattle
"""Scheduler comparison under CSI aging: cell throughput, fairness, per-UE vs beam taper."""
import glob
import os
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

OUT = Path(sys.argv[1] if len(sys.argv) > 1 else "results/ntn-sched10")
DUR = 60.0 - 0.4  # traffic runs from 0.4 s, when leo-link starts its applications
# Each UE's spot-beam taper, as the example prints it: "UE i footprint: ..., gainTaper=X dB"
TAPER = [float(t) for t in re.findall(r"gainTaper=([-0-9.e]+) dB",
                                      next(OUT.glob("*/stdout.txt")).read_text())]
SCHEDS = ["RR", "PF", "MR"]


def per_ue(d):
    t = pd.read_csv(d / "RxPacketTrace.txt", sep="\t")
    t.columns = [c.strip() for c in t.columns]
    t = t[t["direction"] == "DL"]
    ue = t[t["corrupt"] == 0].groupby("rnti").tbSize.sum() * 8 / DUR / 1e6
    # MR can starve a UE outright; it then never appears in the trace but counts as 0.
    return ue.reindex(range(1, len(TAPER) + 1), fill_value=0.0)


rows = []
for d in sorted(glob.glob(str(OUT / "*/"))):
    m = re.match(r"s(\d)-(rr|pf|mr)-csi(\d+)$", os.path.basename(d.rstrip("/")))
    if not m:
        continue
    ue = per_ue(Path(d))
    rows.append(dict(seed=int(m.group(1)), sched=m.group(2).upper(), csi=int(m.group(3)),
                     cell=ue.sum(), jain=ue.sum() ** 2 / (len(ue) * (ue ** 2).sum()),
                     worst=ue.min(), ue=ue.values))
df = pd.DataFrame(rows)

fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
w = 0.35
for i, (csi, off, lab) in enumerate([(10, -w / 2, "CSI 10 slots (fresh)"),
                                     (160, w / 2, "CSI 160 slots (stale)")]):
    s = df[df["csi"] == csi].groupby("sched")
    for col, key, ylab in [(0, "cell", "cell throughput (Mbps)"), (1, "jain", "Jain fairness")]:
        m, e = s[key].mean().reindex(SCHEDS), s[key].std().reindex(SCHEDS)
        ax[col].bar([SCHEDS.index(k) + off for k in SCHEDS], m, w, yerr=e, capsize=3, label=lab)
        ax[col].set(xticks=range(3), ylabel=ylab)
        ax[col].set_xticklabels(SCHEDS)
ax[0].legend(fontsize=8)
ax[1].set_ylim(0, 1.05)

# Per-UE against beam taper, fresh CSI, averaged over seeds: who pays for MR's cell throughput.
for sched in SCHEDS:
    s = df[(df["sched"] == sched) & (df["csi"] == 10)]
    ax[2].plot(TAPER, np.mean(np.stack(s["ue"].values), axis=0), "o-", label=sched, ms=4)
ax[2].set(xlabel="spot-beam taper (dB)", ylabel="per-UE throughput (Mbps)")
ax[2].legend()
for a in ax:
    a.grid(alpha=.3)
fig.suptitle("10 UEs in one beam, saturated, 90 deg, 3 seeds", fontsize=11)
fig.tight_layout()
fig.savefig(OUT / "sched.png", dpi=120)
print(f"-> {OUT}/sched.png")
