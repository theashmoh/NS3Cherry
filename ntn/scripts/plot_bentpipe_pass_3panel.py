#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
"""Three-panel bent-pipe pass figure: geometry, per-link SNR, HARQ loop RTT.

Reads the geometry log a bent-pipe run writes (`log.txt`), which carries both hop
elevations, both hop SNRs, the cascaded end-to-end SNR and the one-way radio delay. The
point of the figure is the contrast the single-hop case cannot show: the end-to-end link is
the cascade of two hops, and the resulting HARQ loop is longer than the process pipeline can
cover.

  python3 ntn/scripts/plot_bentpipe_pass_3panel.py results/<bent-pipe-run> \
      --output ntn/figures/bentpipe-pass.png
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

C_KM_MS = 299.792458  # km per ms


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("result_dir", type=Path, help="run directory containing log.txt")
    ap.add_argument("--output", type=Path, default=Path("ntn/figures/bentpipe-pass.png"))
    ap.add_argument("--harq-procs", type=int, default=32,
                    help="process count; at mu=0 the pipeline it covers is this many ms")
    ap.add_argument("--min-elev", type=float, default=0.0,
                    help="drop samples below this service elevation (below-horizon rows)")
    ap.add_argument("--dpi", type=int, default=200)
    a = ap.parse_args()

    g = pd.read_csv(a.result_dir / "log.txt", sep="\t")
    g = g[g["sat2_UE_elevation_deg"] >= a.min_elev].reset_index(drop=True)
    if g.empty:
        raise SystemExit(f"no samples above {a.min_elev} deg in {a.result_dir}")
    t = g["Time"].to_numpy(dtype=float)

    fig, axes = plt.subplots(3, 1, figsize=(10.5, 9.0), sharex=True)

    # (a) geometry -- service-link elevation and both slant ranges
    ax = axes[0]
    ax.plot(t, g["sat2_UE_elevation_deg"], color="tab:blue", lw=2.2, label="sat2→UE elev")
    ax.plot(t, g["GT_sat1_elevation_deg"], color="navy", lw=1.6, ls="--", label="GT→sat1 elev")
    ax.set_ylabel("Elevation (deg)", color="tab:blue", fontweight="bold")
    ax.tick_params(axis="y", labelcolor="tab:blue")
    ax2 = ax.twinx()
    ax2.plot(t, g["sat2_UE_distance_m"] / 1e3, color="tab:red", lw=2.2, label="sat2→UE dist")
    ax2.plot(t, g["GT_sat1_distance_m"] / 1e3, color="darkred", lw=1.6, ls="--",
             label="GT→sat1 dist")
    ax2.set_ylabel("Slant distance (km)", color="tab:red", fontweight="bold")
    ax2.tick_params(axis="y", labelcolor="tab:red")
    ax.set_title("(a) Pass geometry", fontweight="bold")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=8, ncol=2, loc="upper right")
    ax.grid(alpha=.3)

    # (b) per-link SNR -- the end-to-end value is the cascade, not either hop
    ax = axes[1]
    # SNR_sat2_db is the feeder SNR after the relay loss, not the service hop: that is gamma_2_db
    ax.plot(t, g["SNR_sat2_db"], color="tab:red", lw=1.2, ls="--", label="feeder GT→sat")
    ax.plot(t, g["gamma_2_db"], color="tab:orange", lw=1.2, ls="--", label="downlink sat→UE")
    ax.plot(t, g["SNR_UE_db"], color="tab:green", lw=1.4, label="end-to-end (UE)")
    ax.set_ylabel("SNR (dB)", fontweight="bold")
    ax.set_title("(b) Per-link SNR: end-to-end is the cascade of both hops", fontweight="bold")
    ax.legend(fontsize=9, loc="upper right")
    ax.grid(alpha=.3)

    # (c) HARQ loop RTT -- bent-pipe against the same geometry regenerated on board
    ax = axes[2]
    rtt_bp = 2.0 * g["radio_one_way_delay_ms"].to_numpy(dtype=float)
    rtt_1hop = 2.0 * (g["sat2_UE_distance_m"].to_numpy(dtype=float) / 1e3) / C_KM_MS
    ax.plot(t, rtt_bp, color="tab:red", lw=2.2, label="bent-pipe loop (GT→sat1→sat2→UE)")
    ax.plot(t, rtt_1hop, color="tab:blue", lw=2.0, ls="--", label="single hop (regen-equiv.)")
    ax.axhline(a.harq_procs, color="0.4", ls=":", lw=1.4)
    ax.annotate(f"{a.harq_procs}-process pipeline (~{a.harq_procs} ms)",
                (t[len(t) // 2], a.harq_procs), textcoords="offset points", xytext=(0, 6),
                fontsize=9, color="0.35", fontweight="bold")
    ax.set_ylabel("HARQ loop RTT (ms)", fontweight="bold")
    ax.set_xlabel("Time (s)", fontweight="bold")
    ax.set_title("(c) HARQ loop round-trip time", fontweight="bold")
    ax.legend(fontsize=9, loc="upper left")
    ax.grid(alpha=.3)

    fig.tight_layout()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.output, dpi=a.dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"-> {a.output}")
    print(f"   {len(g)} samples, elevation {g['sat2_UE_elevation_deg'].min():.0f}"
          f"–{g['sat2_UE_elevation_deg'].max():.0f} deg, "
          f"bent-pipe RTT {rtt_bp.min():.0f}–{rtt_bp.max():.0f} ms vs "
          f"single hop {rtt_1hop.min():.0f}–{rtt_1hop.max():.0f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
