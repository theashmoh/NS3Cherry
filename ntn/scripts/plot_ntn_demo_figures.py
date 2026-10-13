#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
# Author: Jesse Chiu (Kuan-Po) <kchiu5@uw.edu>, University of Washington, Seattle
"""The two README figures made from the run_ntn_demo.sh results.

    python3 ntn/scripts/plot_ntn_demo_figures.py results/ntn-demo
        -> results/ntn-demo/v1-tbler.png      (example 6, the OLLA knobs over time)
        -> results/ntn-demo/v2-delay-e30.png  (example 7, the PDCP delay CDF at 30 deg)

Lifted unchanged from the slide-deck builder, which does not ship, so the kit can regenerate the
figures without it. Needs pandas, numpy and matplotlib.
"""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

OUT = Path(sys.argv[1] if len(sys.argv) > 1 else "results/ntn-demo")


def _all_dl(tag):
    """Every DL transmission including HARQ retransmissions -- needed for delivered bits."""
    x = pd.read_csv(OUT / tag / "RxPacketTrace.txt", sep="\t")
    x.columns = [c.strip() for c in x.columns]
    return x[x["direction"] == "DL"]


# --- example 7: where the delay goes once ARQ has to step in.
f3, a3 = plt.subplots(figsize=(6.2, 4.4))
for tag, lab, c in [("lo-um-e30", "UM", "tab:blue"), ("lo-am-e30", "AM", "tab:orange")]:
    d = pd.read_csv(OUT / tag / "NrDlPdcpRxStats.txt", sep="\t")["delay(s)"] * 1000
    a3.plot(np.sort(d), np.linspace(0, 1, len(d)), color=c, lw=1.6, label=lab)
a3.axvline(11.2, color="k", lw=.9, ls=":")
a3.text(11.9, 0.30, "one loop RTT\n(11.2 ms at 30°)", fontsize=8)
a3.set(xscale="log", xlabel="PDCP delay (ms)", ylabel="CDF", xlim=(2, 1200),
       title="30°, 10 Mbps: what ARQ recovery costs the tail")
a3.legend(fontsize=9)
a3.grid(alpha=.3, which="both")
f3.tight_layout()
f3.savefig(OUT / "v2-delay-e30.png", dpi=130)

# --- example 6: what MCS and BLER do under each knob setting at 30 deg, over the run.
# 4 s of steady state. Per-slot trace kept faint so the staircase texture stays visible, with a
# rolling mean over it for the level. Both windows are TRAILING (pandas default): a point at t
# uses only slots <= t, the same information the gNB has, and costs a real lag of half the window.
#
# The two panels need different widths because they plot different kinds of quantity. MCS is a
# VALUE -- each slot already carries it, so 81 slots is only cosmetic smoothing and the lag stays
# at 40 ms, short against the 42 ms coherence time. BLER is a RATE -- no slot carries it, it only
# exists as a fraction over many, with estimator sd sqrt(p(1-p)/w). At w=81 that sd is 0.033
# against a t=0.05 target, so the curve is pure noise; it needs w=400 (sd 0.015) to show a level
# at all. The price is a 200 ms lag, which is fine here because this panel compares the LEVEL each
# knob setting settles at, not the timing of any one fade. Read fade timing off the MCS panel.
_Z0, _Z1, _MW, _BW = 9.0, 13.0, 81, 400


def _series(tag):
    x = _all_dl(tag).reset_index(drop=True)
    x = x[(x["Time"] >= _Z0 - _BW / 1000) & (x["Time"] <= _Z1)].copy()
    x["mcs_s"] = x["mcs"].rolling(_MW, min_periods=20).mean()
    x["bler"] = x["corrupt"].rolling(_BW, min_periods=20).mean()
    # Delivered PHY bits: decoded TBs including HARQ retransmissions, over the same window.
    x["ok_bits"] = np.where(x["corrupt"] == 0, x["tbSize"] * 8, 0.0)
    x["thr"] = x["ok_bits"].rolling(_BW, min_periods=20).mean() * 1000 / 1e6
    return x[x["Time"] >= _Z0]


f5, a5 = plt.subplots(3, 2, figsize=(10.6, 5.6), sharex=True)
_TGT = [("t005-e30", "t = 0.05", .05, "tab:blue"), ("ollaslow-e30", "t = 0.10", .10, "tab:green"),
        ("t025-e30", "t = 0.25", .25, "tab:red")]
_STP = [("ollaslow-e30", "s = 0.001", "tab:green"), ("olla-e30", "s = 0.01", "tab:orange"),
        ("s05-e30", "s = 0.05", "tab:red")]
for col, runs in ((0, _TGT), (1, _STP)):
    for row in runs:
        tag, lab, c = (row[0], row[1], row[3]) if len(row) == 4 else row
        d = _series(tag)
        if col == 0:                      # targets: faint per-slot under a rolling mean
            a5[0][col].plot(d["Time"], d["mcs"], color=c, lw=.4, alpha=.30)
            a5[0][col].plot(d["Time"], d["mcs_s"], color=c, lw=1.8, label=lab)
        else:                             # step size: raw per-slot only, no smoothing
            a5[0][col].plot(d["Time"], d["mcs"], color=c, lw=.6, label=lab)
        a5[1][col].plot(d["Time"], d["bler"], color=c, lw=1.5, label=lab)
        a5[2][col].plot(d["Time"], d["thr"], color=c, lw=1.5, label=lab)
for _tag, _lab, _tgt, _c in _TGT:
    a5[1][0].axhline(_tgt, color=_c, lw=.8, ls=":")
a5[1][1].axhline(.10, color="k", lw=.8, ls=":")
a5[0][0].set(ylabel="MCS (faint: per slot)", ylim=(12, 30), title="target t, at step s = 0.001")
a5[0][1].set(ylim=(12, 30), title="step s, at target t = 0.10 (raw, per slot)")
a5[1][0].set(ylabel="BLER, 400 ms trailing", ylim=(0, .45), xlim=(_Z0, _Z1))
a5[1][1].set(ylim=(0, .45), xlim=(_Z0, _Z1))
a5[2][0].set(xlabel="time (s)", ylabel="PHY delivered (Mbps)", ylim=(0, 32), xlim=(_Z0, _Z1))
a5[2][1].set(xlabel="time (s)", ylim=(0, 32), xlim=(_Z0, _Z1))
a5[0][0].legend(fontsize=7.5, ncol=3, loc="lower left")
a5[0][1].legend(fontsize=7.5, ncol=3, loc="lower left")
for _r in a5:
    for _ax in _r:
        _ax.grid(alpha=.3)
f5.tight_layout()
f5.savefig(OUT / "v1-tbler.png", dpi=130)
print(f"wrote {OUT / 'v1-tbler.png'} and {OUT / 'v2-delay-e30.png'}")
