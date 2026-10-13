#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
"""Per-UE centre-vs-edge footprint analysis (reviewer item 2/4).

For a footprint run (footprintRadiusKm>0) with UE0=centre(r=0), UE1=r~2km,
UE2/UE3=edge(r=R, -3 dB half-power taper), bin each UE's downlink RxPacketTrace
by satellite elevation and compare centre vs edge on SINR / MCS / throughput /
BLER / HARQ-retx. Elevation(t) and the bin convention are reused from
run_leo_link_analysis.py so numbers match the aggregate elevation_stats.

Usage:
  python3 analyze_footprint_per_ue.py <result_dir> [--label NAME] [--out DIR]
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import run_leo_link_analysis as leo

# RxPacketTrace.txt columns (whitespace-delimited):
# Time direction frame subF slot 1stSym nSymbol cellId bwpId rnti tbSize mcs rank rv SINR(dB) CQI corrupt TBler
RX_COLS = ["time_s", "dir", "frame", "subF", "slot", "sym0", "nSym", "cellId",
           "bwpId", "rnti", "tbSize", "mcs", "rank", "rv", "sinr_db", "cqi",
           "corrupt", "tbler"]


def load_rx(result_dir: Path) -> pd.DataFrame:
    path = result_dir / "RxPacketTrace.txt"
    df = pd.read_csv(path, sep=r"\s+", engine="c", header=0, names=RX_COLS,
                     skiprows=1, on_bad_lines="skip")
    df = df[df["dir"] == "DL"].copy()
    for c in ["time_s", "tbSize", "mcs", "rv", "sinr_db", "corrupt", "tbler"]:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    return df.dropna(subset=["time_s", "rnti", "sinr_db"])


def geometry_elevation(result_dir: Path, arch: str, freq_hz: float) -> pd.DataFrame:
    """Return time_s + elevation_deg. Regen: log.txt colon trace. Bent-pipe:
    bin by SERVICE (sat2->UE) elevation — feeder is common to all UEs, so it
    does not confound the centre/edge footprint comparison."""
    if arch == "bentpipe":
        import run_leo_link_bent_pipe_analysis as bp
        g = bp.load_bent_pipe_geometry(result_dir / "log.txt", freq_hz)
        if g.empty or "sat2_ue_elevation_deg" not in g.columns:
            raise RuntimeError("bent-pipe geometry empty/missing")
        geom = g[["time_s", "sat2_ue_elevation_deg"]].rename(
            columns={"sat2_ue_elevation_deg": "elevation_deg"})
    else:
        geom = leo.parse_geometry_trace(result_dir / leo.GEOMETRY_LOG_NAME)
        if geom.empty:
            raise RuntimeError("log.txt geometry empty/missing")
    geom = geom.dropna(subset=["elevation_deg"]).sort_values("time_s").reset_index(drop=True)
    geom["elevation_deg"] = leo.fold_pass_angle_to_elevation(
        geom["elevation_deg"].to_numpy(dtype=float))
    return geom


def detect_arch(result_dir: Path) -> str:
    txt = (result_dir / "run.log")
    if txt.exists() and "bent-pipe" in txt.read_text(errors="ignore").lower():
        return "bentpipe"
    return "regen"


def time_in_elevation_bins(geom: pd.DataFrame, centers, half_bin) -> np.ndarray:
    """Wall-time the pass spends in each elevation bin (s), common to all UEs."""
    t = geom["time_s"].to_numpy(dtype=float)
    e = geom["elevation_deg"].to_numpy(dtype=float)
    dt = np.median(np.diff(t)) if t.size > 1 else 0.0
    out = np.zeros(len(centers))
    for i, c in enumerate(centers):
        lo, hi = c - half_bin, c + half_bin
        if c == centers[-1]:
            m = (e >= lo) & (e <= hi)
        else:
            m = (e >= lo) & (e < hi)
        out[i] = m.sum() * dt
    return out


def per_ue_binned(rx: pd.DataFrame, geom: pd.DataFrame):
    centers = np.arange(0.0, 90.0 + leo.ELEVATION_BIN_DEG, leo.ELEVATION_BIN_DEG)
    half = leo.ELEVATION_BIN_DEG / 2.0
    rx = rx.copy()
    rx["elev"] = leo.interpolate_elevation(geom, rx["time_s"].to_numpy(dtype=float))
    tib = time_in_elevation_bins(geom, centers, half)
    rows = []
    for rnti, g in rx.groupby("rnti"):
        for i, c in enumerate(centers):
            lo, hi = c - half, c + half
            if c == centers[-1]:
                m = (g["elev"] >= lo) & (g["elev"] <= hi)
            else:
                m = (g["elev"] >= lo) & (g["elev"] < hi)
            sub = g[m]
            if sub.empty:
                continue
            new = sub[sub["rv"] == 0]
            delivered_bits = float(sub.loc[sub["corrupt"] == 0, "tbSize"].sum()) * 8.0
            tput = delivered_bits / tib[i] / 1e6 if tib[i] > 0 else np.nan
            rows.append({
                "rnti": int(rnti),
                "elev_center": float(c),
                "n_tb": int(len(sub)),
                "sinr_db_mean": float(sub["sinr_db"].mean()),
                "mcs_mean": float(new["mcs"].mean()) if not new.empty else np.nan,
                "tput_mbps": tput,
                "bler_realized": float(sub["corrupt"].mean()),
                "tbler_model": float(sub["tbler"].mean()),
                "retx_frac": float((sub["rv"] > 0).mean()),
            })
    return pd.DataFrame(rows), centers


def parse_ue_radii(result_dir: Path):
    """Read each UE's radial distance (km) from the run log
    'UE i footprint: radial=X km' lines. rnti = install_index + 1."""
    radii = {}
    for name in ("run.log", "stdout.log"):
        p = result_dir / name
        if not p.exists():
            continue
        for ln in p.read_text(errors="ignore").splitlines():
            m = re.search(r"UE\s+(\d+)\s+footprint:\s+radial=([\d.]+)\s*km", ln)
            if m:
                radii[int(m.group(1)) + 1] = float(m.group(2))
        if radii:
            break
    return radii


def classify_ues(df: pd.DataFrame, result_dir: Path):
    """Group RNTIs into footprint tiers by radial distance (from run.log). Returns
    (roles, tiers, rntis); tiers is ordered by radius with a plot style each."""
    rntis = sorted(int(r) for r in df["rnti"].unique())
    radii = parse_ue_radii(result_dir)
    by_r = {}
    for r in rntis:
        rad = radii.get(r)
        by_r.setdefault(None if rad is None else round(rad), []).append(r)
    styles = [("o-", "C0"), ("s--", "C3"), ("^:", "C1"), ("d-.", "C2")]
    roles, tiers = {}, []
    for k, (rad, members) in enumerate(
            sorted(by_r.items(), key=lambda kv: (kv[0] is None, kv[0]))):
        if rad is None:
            lab = "unknown"
        elif rad < 1:
            lab = "centre (r=0)"
        else:
            lab = f"r={rad:.0f} km" + (" deep-edge" if rad >= 45 else " edge")
        st, col = styles[k % len(styles)]
        tiers.append({"radius": rad, "rntis": members, "label": lab,
                      "style": st, "color": col})
        for r in members:
            roles[r] = lab
    return roles, tiers, rntis


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("result_dir", type=Path)
    ap.add_argument("--label", default=None)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--arch", choices=["regen", "bentpipe", "auto"], default="auto")
    ap.add_argument("--freq-hz", type=float, default=2e9,
                    help="carrier freq for bent-pipe geometry slant calc")
    a = ap.parse_args(argv)
    rdir = a.result_dir
    label = a.label or rdir.name
    out = a.out or rdir
    arch = detect_arch(rdir) if a.arch == "auto" else a.arch

    rx = load_rx(rdir)
    geom = geometry_elevation(rdir, arch, a.freq_hz)
    df, centers = per_ue_binned(rx, geom)
    roles, tiers, rntis = classify_ues(df, rdir)

    csv_path = out / "footprint_per_ue_stats.csv"
    df_out = df.copy()
    df_out["role"] = df_out["rnti"].map(roles)
    df_out.sort_values(["rnti", "elev_center"]).to_csv(csv_path, index=False)

    def series(rntis_sel, col):
        # mean over the tier's UEs, per elevation bin
        s = df[df["rnti"].isin(rntis_sel)].groupby("elev_center")[col].mean()
        return s.reindex(centers)

    metrics = [
        ("sinr_db_mean", "Effective SINR (dB)"),
        ("mcs_mean", "MCS index"),
        ("tput_mbps", "Per-UE throughput (Mb/s)"),
        ("retx_frac", "HARQ retx fraction"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for ax, (col, ylab) in zip(axes.flat, metrics):
        for t in tiers:
            n = len(t["rntis"])
            lab = t["label"] + (f" (x{n} mean)" if n > 1 else "")
            ax.plot(centers, series(t["rntis"], col), t["style"],
                    color=t["color"], label=lab, markersize=4)
        ax.set_xlabel("Satellite elevation (deg)")
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle(f"Per-UE footprint tiers vs elevation — {label}")
    fig.tight_layout()
    fig_path = out / "fig_footprint_per_ue.png"
    fig.savefig(fig_path, dpi=130)
    plt.close(fig)

    # per-tier summary: served-window (elev>=5) time-weighted mean per-UE throughput,
    # consistent with the figure (per-bin delivered bits / time-in-bin).
    tib_main = time_in_elevation_bins(geom, centers, leo.ELEVATION_BIN_DEG / 2.0)
    served_w = np.where(centers >= 5.0, tib_main, 0.0)

    def tput_passavg(rntis_sel):
        g = (df[df["rnti"].isin(rntis_sel)].groupby("elev_center")["tput_mbps"]
             .mean().reindex(centers).to_numpy())
        ok = ~np.isnan(g) & (served_w > 0)
        return float(np.sum(g[ok] * served_w[ok]) / np.sum(served_w[ok])) \
            if served_w[ok].sum() > 0 else float("nan")

    hi = df[df["elev_center"] >= 75.0]
    role_map = ", ".join(f"{r}:{roles[r]}" for r in rntis)
    print(f"[{label}] RNTI->tier: {role_map}")
    print(f"[{label}] per-tier  full-pass tput | elev>=75deg SINR / MCS / tput:")
    for t in tiers:
        rs = t["rntis"]
        pa = tput_passavg(rs)
        hs = hi[hi["rnti"].isin(rs)]
        print(f"  {t['label']:18s}: {pa:6.2f} Mb/s | "
              f"SINR {hs['sinr_db_mean'].mean():6.2f}  "
              f"MCS {hs['mcs_mean'].mean():5.2f}  "
              f"tput {hs['tput_mbps'].mean():6.2f} Mb/s")
    print(f"wrote {csv_path}")
    print(f"wrote {fig_path}")


if __name__ == "__main__":
    sys.exit(main())
