#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
# Author: Jesse Chiu (Kuan-Po) <kchiu5@uw.edu>, University of Washington, Seattle
"""v2 teaching figure: what ARQ buys and what it costs, vs elevation. Reads summary.csv."""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

OUT = Path(sys.argv[1] if len(sys.argv) > 1 else "results/ntn-demo")
df = pd.read_csv(OUT / "summary.csv")
df = df[(df["load"] == 10) & (df["arm"] == "OLLA")]   # headroom on the link: ARQ, not congestion
elevs = sorted(df["elev"].unique())
w = 0.35

fig, ax = plt.subplots(1, 2, figsize=(9, 3.6))
for col, (key, lab) in enumerate([("pdcp_loss", "PDCP loss (fraction)"),
                                  ("pdcp_delay_ms", "PDCP mean delay (ms)")]):
    for mode, off in [("UM", -w / 2), ("AM", w / 2)]:
        s = df[df["rlc"] == mode].set_index("elev").reindex(elevs)
        bars = ax[col].bar([elevs.index(e) + off for e in elevs], s[key], w, label=mode)
        ax[col].bar_label(bars, fmt="%.3g", fontsize=7, padding=1)
    ax[col].set(xticks=range(len(elevs)), xlabel="elevation (deg)", ylabel=lab)
    ax[col].set_xticklabels(elevs)
    ax[col].grid(alpha=.3, axis="y")
ax[0].legend()
fig.suptitle("RLC ARQ over an NTN loop, 10 Mbps offered", fontsize=11)
fig.tight_layout()
fig.savefig(OUT / "v2-arq.png", dpi=120)
print(f"-> {OUT}/v2-arq.png")
