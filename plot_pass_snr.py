#!/usr/bin/env python3
"""
plot_pass_snr.py -- merge the SNR tables of many full-pass runs into CSV files and plot them.

    python3 plot_pass_snr.py RESULTS_DIR [--ue 0] [--dark]

RESULTS_DIR is the OUT folder of run_pass_snr.sh: it reads epochs/run_NNNN_snr.csv (and
epochs/run_NNNN_dlpower.csv, the SNR of the downlink data) of every run, for the UE given by --ue
(default 0), and writes in RESULTS_DIR:

  snr_pass_all.csv       one line per run and elevation step: the SNR of the UE in that step of
                         the pass, as the simulator reports it (samples, min, mean, max)
  snr_pass_summary.csv   over the runs, per series and step: runs, mean, sd, 5/25/50/75/95 %,
                         lowest and highest run, and the mean of the runs' lowest and highest
                         single SNR in the step
  snr_pass.png           DL and UL SNR through the pass: mean over the runs, band 5-95 % of runs
  snr_pass_detail.png    one panel per series with the curve of every run

The simulator reports the SNR of a call in steps of 10 deg of elevation. Along the pass the steps
are, in order: 10, 20 ... 80 deg while the satellite rises (labelled -10 ... -80 in the CSV), 90 deg
over the UE, then 80 ... 10 deg while it sets. A step's value for one run is the mean of the dB
values of the receptions in it (a few hundred, fading included); this script averages those over
the runs. Needs python3 with numpy and matplotlib.
"""

import argparse
import csv
import glob
import math
import os
import re
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# The 17 steps of a pass, in order (index 0 ... 16) and what they are called
STEP_ELEV = [10 * (i + 1) for i in range(8)] + [90] + [80 - 10 * i for i in range(8)]
STEP_PHASE = ["rising"] * 8 + ["overhead"] + ["setting"] * 8

# The series: key, legend text, categorical slot colour (light, dark), CSV column of each statistic
SERIES = [
    ("dl_data", "DL data (on the scheduled RBs)", ("#2a78d6", "#3987e5")),
    ("ul_1rb", "UL, as if sent on 1 RB", ("#eb6834", "#d95926")),
    ("dl_ctrl", "DL control symbols", ("#1baf7a", "#199e70")),
    ("ul_rx", "UL as received (1 or 2 RB)", ("#eb6834", "#d95926")),
]
MAIN = ["dl_data", "ul_1rb", "dl_ctrl"]  # snr_pass.png (the fourth is in the detail figure)

INK = {
    "light": dict(surface="#fcfcfb", primary="#0b0b0b", secondary="#52514e", muted="#898781",
                  grid="#e1e0d9", axis="#c3c2b7"),
    "dark": dict(surface="#1a1a19", primary="#f0efec", secondary="#c3c2b7", muted="#898781",
                 grid="#2c2c2a", axis="#383835"),
}


def num(text):
    try:
        v = float(text)
    except (TypeError, ValueError):
        return math.nan
    return v if math.isfinite(v) else math.nan


def step_index(label):
    """The CSV label of a step (-30: rising at 30 deg, 90: overhead, 30: setting at 30) -> 0..16"""
    value = int(float(label))
    elev = abs(value)
    if elev >= 90:
        return 8
    if value < 0:
        return elev // 10 - 1
    return 9 + (80 - elev) // 10


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def load_runs(results, ue):
    """-> list of dicts, one per (run, step) of the UE"""
    records = []
    files = sorted(glob.glob(os.path.join(results, "epochs", "run_*_snr.csv")))
    if not files:
        sys.exit(f"no epochs/run_*_snr.csv in {results}: run run_pass_snr.sh first")
    for path in files:
        run = int(re.search(r"run_(\d+)_snr\.csv$", path).group(1))
        power = {}
        ppath = path.replace("_snr.csv", "_dlpower.csv")
        if os.path.exists(ppath):
            for r in read_rows(ppath):
                if int(r["ue"]) == ue:
                    power[r["elevDeg"]] = r
        for r in read_rows(path):
            if int(r["ue"]) != ue:
                continue
            p = power.get(r["elevDeg"], {})
            idx = step_index(r["elevDeg"])
            records.append({
                "run": run,
                "rngRun": r.get("rngRun", ""),
                "ue": ue,
                "step": r["elevDeg"],
                "elevDeg": STEP_ELEV[idx],
                "phase": STEP_PHASE[idx],
                "passIdx": idx,
                "dlCtrlN": num(r.get("dlN")), "dlCtrlMinDb": num(r.get("dlMinDb")),
                "dlCtrlMeanDb": num(r.get("dlMeanDb")), "dlCtrlMaxDb": num(r.get("dlMaxDb")),
                "dlDataN": num(p.get("nDataSnr")), "dlDataMinDb": num(p.get("minDataSnrDb")),
                "dlDataMeanDb": num(p.get("meanDataSnrDb")), "dlDataMaxDb": num(p.get("maxDataSnrDb")),
                "dlRbPowerMeanDbm": num(p.get("meanDbm")),
                "ulN": num(r.get("ulN")), "ul1RbMinDb": num(r.get("ulMinDb")),
                "ul1RbMeanDb": num(r.get("ulMeanDb")), "ul1RbMaxDb": num(r.get("ulMaxDb")),
                "ulRxMinDb": num(r.get("ulRxMinDb")), "ulRxMeanDb": num(r.get("ulRxMeanDb")),
                "ulRxMaxDb": num(r.get("ulRxMaxDb")),
            })
    if not records:
        sys.exit(f"no lines for UE {ue} in the files of {results}/epochs")
    return records


COLUMNS = ["run", "rngRun", "ue", "step", "elevDeg", "phase", "passIdx",
           "dlCtrlN", "dlCtrlMinDb", "dlCtrlMeanDb", "dlCtrlMaxDb",
           "dlDataN", "dlDataMinDb", "dlDataMeanDb", "dlDataMaxDb", "dlRbPowerMeanDbm",
           "ulN", "ul1RbMinDb", "ul1RbMeanDb", "ul1RbMaxDb",
           "ulRxMinDb", "ulRxMeanDb", "ulRxMaxDb"]

# series key -> the (min, mean, max) columns of snr_pass_all.csv
STAT_COLS = {
    "dl_ctrl": ("dlCtrlMinDb", "dlCtrlMeanDb", "dlCtrlMaxDb"),
    "dl_data": ("dlDataMinDb", "dlDataMeanDb", "dlDataMaxDb"),
    "ul_1rb": ("ul1RbMinDb", "ul1RbMeanDb", "ul1RbMaxDb"),
    "ul_rx": ("ulRxMinDb", "ulRxMeanDb", "ulRxMaxDb"),
}


def fmt(v):
    return "" if v is None or (isinstance(v, float) and not math.isfinite(v)) else f"{v:.6g}"


def write_all(results, records):
    path = os.path.join(results, "snr_pass_all.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for r in sorted(records, key=lambda r: (r["run"], r["passIdx"])):
            w.writerow([fmt(r[c]) if not isinstance(r[c], str) else r[c] for c in COLUMNS])
    return path


def build_arrays(records):
    """series -> {'min','mean','max'} arrays of shape (runs, 17), NaN where a run has no value"""
    runs = sorted({r["run"] for r in records})
    row = {run: i for i, run in enumerate(runs)}
    arrays = {}
    for key, (cmin, cmean, cmax) in STAT_COLS.items():
        a = {s: np.full((len(runs), len(STEP_ELEV)), np.nan) for s in ("min", "mean", "max")}
        for r in records:
            a["min"][row[r["run"]], r["passIdx"]] = r[cmin]
            a["mean"][row[r["run"]], r["passIdx"]] = r[cmean]
            a["max"][row[r["run"]], r["passIdx"]] = r[cmax]
        if np.isfinite(a["mean"]).any():
            arrays[key] = a
    return runs, arrays


def stat(col, fn, *args):
    v = col[np.isfinite(col)]
    return fn(v, *args) if v.size else math.nan


def write_summary(results, runs, arrays):
    path = os.path.join(results, "snr_pass_summary.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["series", "passIdx", "elevDeg", "phase", "runs", "meanDb", "sdDb", "p5Db",
                    "p25Db", "p50Db", "p75Db", "p95Db", "lowestRunDb", "highestRunDb",
                    "meanOfRunMinDb", "meanOfRunMaxDb"])
        for key, label, _ in SERIES:
            if key not in arrays:
                continue
            a = arrays[key]
            for i in range(len(STEP_ELEV)):
                col = a["mean"][:, i]
                n = int(np.isfinite(col).sum())
                if n == 0:
                    continue
                v = col[np.isfinite(col)]
                w.writerow([key, i, STEP_ELEV[i], STEP_PHASE[i], n, fmt(v.mean()),
                            fmt(v.std(ddof=1) if n > 1 else math.nan),
                            *[fmt(np.percentile(v, p)) for p in (5, 25, 50, 75, 95)],
                            fmt(v.min()), fmt(v.max()),
                            fmt(stat(a["min"][:, i], np.mean)), fmt(stat(a["max"][:, i], np.mean))])
    return path


def style_axes(ax, ink, idx_lo, idx_hi):
    ax.set_facecolor(ink["surface"])
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(ink["axis"])
    ax.grid(axis="y", color=ink["grid"], linewidth=1, linestyle="-")
    ax.set_axisbelow(True)
    ax.tick_params(axis="both", colors=ink["muted"], labelsize=9, length=0)
    ax.set_xticks(range(idx_lo, idx_hi + 1))
    ax.set_xticklabels([str(STEP_ELEV[i]) for i in range(idx_lo, idx_hi + 1)])
    ax.set_xlim(idx_lo - 0.4, idx_hi + 0.4)
    if idx_lo <= 8 <= idx_hi:
        ax.axvline(8, color=ink["axis"], linewidth=1, zorder=0)


def pass_labels(ax, ink, idx_lo, idx_hi):
    """'satellite rising' / 'setting' above the plot"""
    if idx_lo < 8:
        ax.text(idx_lo, 1.02, "satellite rising", transform=ax.get_xaxis_transform(),
                color=ink["muted"], fontsize=9, ha="left", va="bottom")
    if idx_hi > 8:
        ax.text(idx_hi, 1.02, "satellite setting", transform=ax.get_xaxis_transform(),
                color=ink["muted"], fontsize=9, ha="right", va="bottom")
    if idx_lo <= 8 <= idx_hi:
        ax.text(8, 1.02, "overhead", transform=ax.get_xaxis_transform(), color=ink["muted"],
                fontsize=9, ha="center", va="bottom")


def title_block(fig, ink, title, sub):
    fig.text(0.06, 0.965, title, color=ink["primary"], fontsize=14, fontweight="bold",
             ha="left", va="top")
    fig.text(0.06, 0.925, sub, color=ink["secondary"], fontsize=9.5, ha="left", va="top")


def draw_series(ax, key, color, ink, arrays, xs, label=None, lines=False, extremes=False,
                z=4):
    a = arrays[key]
    mean = np.array([stat(a["mean"][:, i], np.mean) for i in xs])
    p5 = np.array([stat(a["mean"][:, i], np.percentile, 5) for i in xs])
    p95 = np.array([stat(a["mean"][:, i], np.percentile, 95) for i in xs])
    if lines:  # the curve of every run, very faint
        alpha = min(0.4, max(0.04, 8.0 / max(1, a["mean"].shape[0])))
        for run_row in a["mean"]:
            ax.plot(xs, run_row[list(xs)], color=color, alpha=alpha, linewidth=0.8, zorder=1)
    ax.fill_between(xs, p5, p95, color=color, alpha=0.14, linewidth=0, zorder=z - 2)
    if extremes:  # what the single receptions reach in a step, on average over the runs
        lo = np.array([stat(a["min"][:, i], np.mean) for i in xs])
        hi = np.array([stat(a["max"][:, i], np.mean) for i in xs])
        ax.plot(xs, lo, color=color, linewidth=1, linestyle=":", zorder=z - 1)
        ax.plot(xs, hi, color=color, linewidth=1, linestyle=":", zorder=z - 1)
    ax.plot(xs, mean, color=color, linewidth=2, solid_capstyle="round", solid_joinstyle="round",
            marker="o", markersize=6, markeredgecolor=ink["surface"], markeredgewidth=1.5,
            label=label, zorder=z)
    return mean


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("results", help="the OUT folder of run_pass_snr.sh")
    ap.add_argument("--ue", type=int, default=0, help="which UE (default 0)")
    ap.add_argument("--dark", action="store_true", help="dark background")
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()

    mode = "dark" if args.dark else "light"
    ink = INK[mode]
    color = {key: cols[1 if args.dark else 0] for key, _, cols in SERIES}
    legend_text = {key: text for key, text, _ in SERIES}

    records = load_runs(args.results, args.ue)
    all_path = write_all(args.results, records)
    runs, arrays = build_arrays(records)
    sum_path = write_summary(args.results, runs, arrays)
    seeds = sorted({r["rngRun"] for r in records if r["rngRun"] != ""}, key=lambda s: int(s))
    seed_text = f", RngRun {seeds[0]} to {seeds[-1]}" if seeds else ""
    n_runs = len(runs)

    present = [i for i in range(len(STEP_ELEV))
               if any(np.isfinite(arrays[k]["mean"][:, i]).any() for k in arrays)]
    lo, hi = min(present), max(present)
    xs = list(range(lo, hi + 1))

    # the first of these that is installed (no warning about the others)
    from matplotlib import font_manager
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in ("Segoe UI", "Helvetica Neue", "Arial", "DejaVu Sans"):
        if name in installed:
            plt.rcParams["font.family"] = name
            break

    # --- Figure 1: DL and UL SNR through the pass ------------------------------------------------
    fig, ax = plt.subplots(figsize=(11, 6.2), dpi=args.dpi)
    fig.patch.set_facecolor(ink["surface"])
    fig.subplots_adjust(left=0.07, right=0.86, top=0.82, bottom=0.12)
    style_axes(ax, ink, lo, hi)
    pass_labels(ax, ink, lo, hi)
    ends = []
    layer = {"ul_1rb": 6, "dl_data": 5, "dl_ctrl": 4}  # the UL line over the DL control line
    for key in MAIN:
        if key not in arrays:
            continue
        mean = draw_series(ax, key, color[key], ink, arrays, xs, label=legend_text[key],
                           z=layer[key])
        ends.append((mean[-1], key))
    ax.set_ylabel("SNR of UE %d (dB)" % args.ue, color=ink["secondary"], fontsize=10)
    ax.set_xlabel("elevation of the satellite over the UE (degrees)", color=ink["secondary"],
                  fontsize=10)
    ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True, nbins=9))
    # a label at the end of each line, when the ends are far enough apart; the legend has them all
    ends.sort()
    for j, (y, key) in enumerate(ends):
        near = [abs(y - o) for k2, (o, _) in enumerate(ends) if k2 != j]
        if not near or min(near) > 1.6:
            ax.text(hi + 0.25, y, legend_text[key].split(" (")[0], color=ink["secondary"],
                    fontsize=9, va="center", ha="left", clip_on=False)
    leg = ax.legend(loc="upper center", ncol=len(ends), frameon=False, fontsize=9,
                    bbox_to_anchor=(0.5, -0.1), handlelength=1.8)
    for t in leg.get_texts():
        t.set_color(ink["secondary"])
    title_block(fig, ink, "SNR of UE %d through a full satellite pass" % args.ue,
                f"{n_runs} runs{seed_text}. Line: mean over the runs of the SNR in each 10° step "
                "of elevation. Band: 5 to 95 % of the runs.")
    p1 = os.path.join(args.results, "snr_pass.png")
    fig.savefig(p1, facecolor=fig.get_facecolor())
    plt.close(fig)

    # --- Figure 2: one panel per series, every run's curve --------------------------------------
    keys = [k for k, _, _ in SERIES if k in arrays]
    cols = 2 if len(keys) > 1 else 1
    rows = int(math.ceil(len(keys) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(6.2 * cols, 3.6 * rows + 0.9), dpi=args.dpi,
                             sharex=True, sharey=True, squeeze=False)
    fig.patch.set_facecolor(ink["surface"])
    fig.subplots_adjust(left=0.07, right=0.98, top=0.80 if rows == 2 else 0.72, bottom=0.09,
                        hspace=0.45, wspace=0.08)
    for n, key in enumerate(keys):
        ax = axes[n // cols][n % cols]
        style_axes(ax, ink, lo, hi)
        draw_series(ax, key, color[key], ink, arrays, xs, lines=True, extremes=True)
        ax.set_title(legend_text[key], color=ink["primary"], fontsize=10.5, loc="left", pad=26)
        pass_labels(ax, ink, lo, hi)
        ax.tick_params(labelbottom=True)
        if n % cols == 0:
            ax.set_ylabel("SNR (dB)", color=ink["secondary"], fontsize=9)
        if n // cols == rows - 1:
            ax.set_xlabel("elevation (degrees)", color=ink["secondary"], fontsize=9)
    for n in range(len(keys), rows * cols):
        axes[n // cols][n % cols].axis("off")
    title_block(fig, ink, "SNR of UE %d through the pass: every run" % args.ue,
                f"{n_runs} runs{seed_text}. Thick line: mean of the runs. Faint lines: each run. "
                "Band: 5 to 95 % of the runs.\nDotted: mean over the runs of the lowest and of "
                "the highest single reception in a step.")
    p2 = os.path.join(args.results, "snr_pass_detail.png")
    fig.savefig(p2, facecolor=fig.get_facecolor())
    plt.close(fig)

    # --- A short table on the screen -------------------------------------------------------------
    print(f"\n{n_runs} runs read. Mean over the runs of the SNR of UE {args.ue} (dB), by step:")
    head = "  elev  " + "".join(f"{legend_text[k].split(' (')[0][:22]:>24}" for k in keys)
    print(head)
    for i in xs:
        cells = []
        for k in keys:
            v = stat(arrays[k]["mean"][:, i], np.mean)
            cells.append(f"{v:24.2f}" if math.isfinite(v) else f"{'-':>24}")
        tag = {"rising": "up", "overhead": "  ", "setting": "dn"}[STEP_PHASE[i]]
        print(f"  {STEP_ELEV[i]:3d}{tag}" + "".join(cells))
    print("\nWritten:")
    for p in (all_path, sum_path, p1, p2):
        print("  " + p)


if __name__ == "__main__":
    main()
