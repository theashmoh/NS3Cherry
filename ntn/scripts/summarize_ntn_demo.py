#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
# Author: Jesse Chiu (Kuan-Po) <kchiu5@uw.edu>, University of Washington, Seattle
"""Summarize run_ntn_demo.sh: ILLA vs OLLA, then RLC UM vs AM. CSV + one figure."""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

OUT = Path(sys.argv[1] if len(sys.argv) > 1 else "results/ntn-demo")
ELEVS = [30, 60, 90]
DUR = 20.0


def phy(d):
    """DL first transmissions (rv==0) carry the link-adaptation decision; rv>0 are HARQ retx."""
    t = pd.read_csv(d / "RxPacketTrace.txt", sep="\t")
    t.columns = [c.strip() for c in t.columns]
    t = t[t["direction"] == "DL"]
    first = t[t["rv"] == 0]
    return dict(
        mcs=first["mcs"].mean(),
        bler1=first["corrupt"].mean(),          # first-transmission BLER, what OLLA steers
        retx=len(t[t["rv"] > 0]) / max(len(first), 1),
        sinr=first["SINR(dB)"].mean(),
        # decoded TB bytes: PHY-layer, so it counts HARQ retransmissions and padding,
        # not application goodput. Compare it to the offered load, not to it directly.
        phy_mbps=t.loc[t["corrupt"] == 0, "tbSize"].sum() * 8 / DUR / 1e6,
        t=first["Time"], m=first["mcs"],
    )


def bearer(d, layer):
    """E2E RLC/PDCP stats, one row per data bearer, delay in seconds.
    Judge ARQ at PDCP: an AM retransmission counts as a new RLC Tx, so RLC-level
    loss looks the same for UM and AM. PDCP is what the application actually sees."""
    r = pd.read_csv(d / f"NrDl{layer}StatsE2E.txt", sep="\t", comment="%", header=None,
                    names="start end cell imsi rnti lcid nTx txB nRx rxB delay sd min max".split(),
                    usecols=range(14))
    r = r[r["lcid"] > 2]  # data bearers only, skip SRBs
    return dict(loss=1 - r["nRx"].sum() / max(r["nTx"].sum(), 1),
                delay=(r["delay"] * r["nRx"]).sum() / max(r["nRx"].sum(), 1) * 1e3,
                dmax=r["max"].max() * 1e3)


rows = []
for e in ELEVS:
    for tag, arm, mode, load in [(f"illa-e{e}", "ILLA", "UM", 20), (f"olla-e{e}", "OLLA", "UM", 20),
                                 (f"am-e{e}", "OLLA", "AM", 20),
                                 (f"lo-um-e{e}", "OLLA", "UM", 10),
                                 (f"lo-am-e{e}", "OLLA", "AM", 10)]:
        d = OUT / tag
        if not all((d / f).exists() for f in ("RxPacketTrace.txt", "NrDlPdcpStatsE2E.txt")):
            print(f"skip {tag}: incomplete"); continue
        p, r, c = phy(d), bearer(d, "Rlc"), bearer(d, "Pdcp")
        rows.append(dict(elev=e, arm=arm, rlc=mode, load=load, mcs=round(p["mcs"], 2),
                         sinr=round(p["sinr"], 2), bler1=round(p["bler1"], 4),
                         retx=round(p["retx"], 3), phy_mbps=round(p["phy_mbps"], 2),
                         pdcp_loss=round(c["loss"], 5), pdcp_delay_ms=round(c["delay"], 1),
                         pdcp_max_ms=round(c["dmax"], 1), rlc_delay_ms=round(r["delay"], 1)))

df = pd.DataFrame(rows)
df.to_csv(OUT / "summary.csv", index=False)
print(df.to_string(index=False))

fig, ax = plt.subplots(1, 2, figsize=(11, 4))
for e in ELEVS:  # running first-tx BLER: OLLA walks to the target, ILLA sits wherever CQI puts it
    for tag, ls in [(f"illa-e{e}", "--"), (f"olla-e{e}", "-")]:
        d = OUT / tag
        if not (d / "RxPacketTrace.txt").exists():
            continue
        t = pd.read_csv(d / "RxPacketTrace.txt", sep="\t")
        t.columns = [c.strip() for c in t.columns]
        t = t[(t["direction"] == "DL") & (t["rv"] == 0)]
        ax[0].plot(t["Time"].values, t["corrupt"].expanding().mean().values, ls, label=tag)
ax[0].axhline(0.1, color="k", lw=.8, ls=":")
ax[0].set(xlabel="time (s)", ylabel="running first-tx BLER", ylim=(0, .35),
          title="ILLA vs OLLA (dotted = 0.10 target)")
ax[0].legend(fontsize=7); ax[0].grid(alpha=.3)

if not df.empty:
    w = 0.35
    for i, (mode, off) in enumerate([("UM", -w / 2), ("AM", w / 2)]):
        s = df[(df["rlc"] == mode) & (df["arm"] == "OLLA") & (df["load"] == 10)]
        ax[1].bar([ELEVS.index(e) + off for e in s["elev"]], s["pdcp_delay_ms"], w, label=mode)
    ax[1].set(xticks=range(len(ELEVS)), xlabel="elevation (deg)", ylabel="PDCP delay (ms)",
              title="ARQ cost: UM vs AM @10 Mbps")
    ax[1].set_xticklabels(ELEVS); ax[1].legend(); ax[1].grid(alpha=.3)
fig.tight_layout(); fig.savefig(OUT / "ntn-demo.png", dpi=120)
print(f"\n-> {OUT}/summary.csv  {OUT}/ntn-demo.png")
