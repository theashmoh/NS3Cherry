#!/usr/bin/env python3
"""plot_sweep.py - plots and tables of a run_sweep.sh folder (v10.1).

    python3 plot_sweep.py <sweep folder> [--pdb 100] [--target 95]

Reads <folder>/runs/<label>/n<users>/s<seed>_r<run>/out_*.csv of the runs that finished
(exit_code 0). One replica = one seed x run. Writes into <folder>/plots/:

  summary.csv               per configuration and user count: replicas and every metric (mean, 95 % CI)
  capacity.csv              per configuration: the most users with >= --target % of calls satisfied
  1_pdcch_utilization.png   CCEs used of the CCEs possible (mean over the cells with UEs; dashed: busiest cell)
  2_directions_qos.png      % of call directions meeting 5QI 1 (98 % within PDB, PER <= 1 %): all, UL, DL
  3_satisfied_calls.png     % of calls with both directions meeting it, --target line; % released / blocked calls
  4a_delay_p98.png          98th percentile of all packets' UE<->UPF delay (UL, DL), PDB line
  4b_delay_call_p98.png     per configuration: median and 5-95 % of the calls' own p98, UL and DL
  4c_delay_cdf_ul.png, 4c_delay_cdf_dl.png   per configuration: delay CDF, one curve per user count
  5_per_loss.png            UL and DL: PER (lost / on bearer, 1 % line), over PDB (late + discard, 2 % line),
                            % of call directions with PER <= 1 % - all calls of a run pooled

Calls a run blocked (admission) count as not meeting the QoS. Delays: the satellite UE <-> UPF
stretch (the 5QI delay) of delivered packets, from out_delays.csv (1 ms bins).
"""
import argparse
import csv
import math
import os
import re
import sys
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import MaxNLocator  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("folder")
ap.add_argument("--pdb", type=float, default=100.0, help="PDB line (ms)")
ap.add_argument("--target", type=float, default=95.0, help="%% of satisfied calls for the capacity")
args = ap.parse_args()

root = os.path.join(args.folder, "runs")
if not os.path.isdir(root):
    sys.exit(f"plot_sweep: no runs in {args.folder}")
plots = os.path.join(args.folder, "plots")
os.makedirs(plots, exist_ok=True)


def rows(path):
    if not os.path.isfile(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def pct_of_hist(hist, p):
    """p-th percentile (0-100) of a {delayMs: packets} histogram"""
    total = sum(hist.values())
    if total == 0:
        return None
    need = total * p / 100.0
    acc = 0
    for d in sorted(hist):
        acc += hist[d]
        if acc >= need:
            return d
    return max(hist)


def label_key(s):
    return (len(s) > 1, s)


# --- read every finished run -------------------------------------------------------------------
runs = []  # dicts: label, n, rep, metrics, hist {dir: {ms: packets}}, callp98 {dir: [..]}
failed = 0
for label in sorted(os.listdir(root), key=label_key):
    for nd in os.listdir(os.path.join(root, label)):
        m = re.fullmatch(r"n(\d+)", nd)
        if not m:
            continue
        n = int(m.group(1))
        for rep in sorted(os.listdir(os.path.join(root, label, nd))):
            d = os.path.join(root, label, nd, rep)
            try:
                ok = open(os.path.join(d, "exit_code")).read().strip() == "0"
            except OSError:
                ok = False
            if not ok:
                failed += 1
                continue
            calls = [r for r in rows(os.path.join(d, "out_calls.csv")) if r.get("bearer", "0") == "0"]
            if not calls:
                failed += 1
                continue
            met = {}
            # 1) PDCCH: CCEs used of possible, per cell (cells with UEs), and the busiest cell
            cells = [r for r in rows(os.path.join(d, "out_cells.csv")) if (num(r.get("ues")) or 0) > 0]
            cce = [num(r.get("ccePct")) for r in cells if num(r.get("ccePct")) is not None]
            if cce:
                met["pdcchUtilPct"] = sum(cce) / len(cce)
                met["pdcchUtilMaxPct"] = max(cce)
            pd = rows(os.path.join(d, "out_pdcch.csv"))
            if pd and num(pd[0].get("blockedPct")) is not None:
                met["pdcchBlockedPct"] = num(pd[0]["blockedPct"])
            # 2), 3) directions and calls meeting 5QI 1
            okDir = {"all": [], "ul": [], "dl": []}
            byCall = defaultdict(list)
            callp98 = {"ul": [], "dl": []}
            blocked = 0
            for r in calls:
                good = r.get("satisfied") == "1" and r.get("callStatus", "admitted") != "blocked"
                blocked += r.get("callStatus") == "blocked"
                okDir["all"].append(good)
                okDir.setdefault(r["dir"], []).append(good)
                byCall[r["ue"]].append(good)
                p98 = num(r.get("delayP98Ms"))
                if p98 is not None and r["dir"] in callp98:
                    callp98[r["dir"]].append(p98)
            for k, v in okDir.items():
                if v:
                    met[f"dirOkPct_{k}"] = 100.0 * sum(v) / len(v)
            met["callsOkPct"] = 100.0 * sum(all(v) for v in byCall.values()) / len(byCall)
            met["blockedDirPct"] = 100.0 * blocked / len(calls)
            # calls the gNB released (--dropCalls) or blocked (--admission), of all calls
            status = {}
            for r in calls:
                status[r["ue"]] = r.get("callStatus", "admitted")
            met["releasedCallsPct"] = 100.0 * sum(v == "released" for v in status.values()) / len(status)
            met["blockedCallsPct"] = 100.0 * sum(v == "blocked" for v in status.values()) / len(status)
            # 5) loss: PER = lost / on bearer, over PDB = late + discard, pooled over the calls
            for k in ("ul", "dl"):
                mine = [r for r in calls if r["dir"] == k]
                onb = sum(num(r.get("onBearer")) or 0 for r in mine)
                if onb > 0:
                    met[f"perPct_{k}"] = 100.0 * sum(num(r.get("lost")) or 0 for r in mine) / onb
                    met[f"overPdbCallsPct_{k}"] = 100.0 * sum(
                        (num(r.get("late")) or 0) + (num(r.get("discard")) or 0) for r in mine) / onb
                pers = [num(r.get("perPct")) for r in mine if num(r.get("perPct")) is not None]
                if pers:
                    met[f"dirPerOkPct_{k}"] = 100.0 * sum(x <= 1.0 for x in pers) / len(pers)
            # 4) delays: pooled histogram per direction
            hist = {"ul": defaultdict(int), "dl": defaultdict(int)}
            for r in rows(os.path.join(d, "out_delays.csv")):
                if r.get("dir") in hist and num(r.get("delayMs")) is not None:
                    hist[r["dir"]][num(r["delayMs"])] += int(float(r["packets"]))
            for k in ("ul", "dl"):
                tot = sum(hist[k].values())
                if tot:
                    met[f"delayMeanMs_{k}"] = sum(ms * c for ms, c in hist[k].items()) / tot
                    for p in (50, 95, 98):
                        met[f"delayP{p}Ms_{k}"] = pct_of_hist(hist[k], p)
                    met[f"overPdbPct_{k}"] = 100.0 * sum(c for ms, c in hist[k].items() if ms > args.pdb) / tot
            runs.append(dict(label=label, n=n, rep=rep, met=met, hist=hist, callp98=callp98))

if not runs:
    sys.exit("plot_sweep: no finished runs with CSVs")
labels = sorted({r["label"] for r in runs}, key=label_key)
colors = {l: plt.get_cmap("tab10")(i % 10) for i, l in enumerate(labels)}
styles = {l: ["-", "--", ":"][(i // 10) % 3] for i, l in enumerate(labels)}

# --- aggregate over replicas --------------------------------------------------------------------
group = defaultdict(list)
for r in runs:
    group[(r["label"], r["n"])].append(r)
metrics = sorted({k for r in runs for k in r["met"]})


def mean_ci(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None, None
    m = sum(vals) / len(vals)
    if len(vals) < 2:
        return m, 0.0
    sd = math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1))
    return m, 1.96 * sd / math.sqrt(len(vals))


agg = {}
with open(os.path.join(plots, "summary.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["label", "users", "replicas"] + [x for k in metrics for x in (k, k + "_ci95")])
    for (label, n) in sorted(group, key=lambda k: (label_key(k[0]), k[1])):
        g = group[(label, n)]
        line = [label, n, len(g)]
        for k in metrics:
            m, ci = mean_ci([r["met"].get(k) for r in g])
            agg[(label, n, k)] = (m, ci)
            line += ["" if m is None else round(m, 4), "" if ci is None else round(ci, 4)]
        w.writerow(line)


def series(label, key):
    ns = sorted(n for (l, n) in group if l == label)
    pts = [(n, *agg.get((label, n, key), (None, None))) for n in ns]
    return [(n, m, ci) for n, m, ci in pts if m is not None]


def plot_metric(ax, key, ylabel, dashed_key=None):
    for l in labels:
        s = series(l, key)
        if not s:
            continue
        ax.errorbar([p[0] for p in s], [p[1] for p in s], yerr=[p[2] for p in s], color=colors[l],
                    linestyle=styles[l], marker="o", ms=3, capsize=2, label=l)
        if dashed_key:
            s2 = series(l, dashed_key)
            ax.plot([p[0] for p in s2], [p[1] for p in s2], color=colors[l], linestyle=":", lw=1)
    ax.set_xlabel("users (calls)")
    ax.set_ylabel(ylabel)
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.grid(alpha=0.3)


reps = sorted({len(g) for g in group.values()})
note = f"mean of {reps[0]}" + (f"-{reps[-1]}" if len(reps) > 1 else "") + " replicas, bars: 95 % CI"

# 1) PDCCH utilisation
fig, ax = plt.subplots(figsize=(7, 4.5))
plot_metric(ax, "pdcchUtilPct", "PDCCH CCEs used [% of possible]", "pdcchUtilMaxPct")
ax.set_title("PDCCH utilisation vs users\n(dotted: busiest cell; " + note + ")", fontsize=10)
ax.legend(ncol=3, fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(plots, "1_pdcch_utilization.png"), dpi=150)
plt.close(fig)

# 2) directions meeting QoS
fig, axs = plt.subplots(1, 3, figsize=(15, 4.5), sharey=True)
for ax, k, t in zip(axs, ("all", "ul", "dl"), ("all directions", "uplink", "downlink")):
    plot_metric(ax, f"dirOkPct_{k}", "call directions meeting 5QI 1 [%]")
    ax.set_title(t, fontsize=10)
    ax.set_ylim(-2, 102)
axs[0].legend(ncol=3, fontsize=8)
fig.suptitle(f"98 % of packets within {args.pdb:g} ms (UE-UPF) and PER <= 1 %   ({note})", fontsize=10)
fig.tight_layout()
fig.savefig(os.path.join(plots, "2_directions_qos.png"), dpi=150)
plt.close(fig)

# 3) satisfied calls, capacity; released and blocked calls
fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13, 4.5))
plot_metric(ax, "callsOkPct", "satisfied calls [%] (both directions meet 5QI 1)")
ax.axhline(args.target, color="k", lw=0.8, ls="--")
ax.set_ylim(-2, 102)
ax.set_title(f"Satisfied calls vs users, capacity at {args.target:g} %\n({note})", fontsize=10)
ax.legend(ncol=3, fontsize=8)
plot_metric(ax2, "releasedCallsPct", "calls released [%] (solid); blocked (dotted)", "blockedCallsPct")
ax2.set_title("Calls the gNB released for missing the PDB (--dropCalls)\nand blocked at admission (--admission), dotted", fontsize=10)
fig.tight_layout()
fig.savefig(os.path.join(plots, "3_satisfied_calls.png"), dpi=150)
plt.close(fig)

with open(os.path.join(plots, "capacity.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["label", f"capacity_users_at_{args.target:g}pct", "how"])
    for l in labels:
        s = series(l, "callsOkPct")
        cap, how = None, ""
        for i, (n, m, _) in enumerate(s):
            if m >= args.target:
                cap, how = n, "last tested count meeting the target"
                if i + 1 < len(s) and s[i + 1][1] < args.target:
                    n2, m2 = s[i + 1][0], s[i + 1][1]
                    cap = n + (n2 - n) * (m - args.target) / (m - m2)
                    how = "interpolated between the counts around the crossing"
            else:
                break
        if cap is None:
            how = "below the target already at the smallest count"
        elif s and s[-1][1] >= args.target and how.startswith("last"):
            how = "meets the target at the largest count tested: test more users"
        w.writerow([l, "" if cap is None else round(cap, 1), how])

# 5) PER and over-PDB, UL and DL
fig, axs = plt.subplots(3, 2, figsize=(12, 11), sharex=True)
for col, (k, t) in enumerate((("ul", "uplink"), ("dl", "downlink"))):
    plot_metric(axs[0][col], f"perPct_{k}", "PER [%] (lost / on bearer)")
    axs[0][col].axhline(1.0, color="k", lw=0.8, ls="--")
    axs[0][col].set_title(f"{t}: packet error rate, all calls pooled", fontsize=10)
    plot_metric(axs[1][col], f"overPdbCallsPct_{k}", "over PDB [%] (late + discarded)")
    axs[1][col].axhline(2.0, color="k", lw=0.8, ls="--")
    axs[1][col].set_title(f"{t}: packets over the PDB, all calls pooled", fontsize=10)
    plot_metric(axs[2][col], f"dirPerOkPct_{k}", "call directions with PER <= 1 % [%]")
    axs[2][col].set_ylim(-2, 102)
    axs[2][col].set_title(f"{t}: call directions meeting the PER part", fontsize=10)
axs[0][0].legend(ncol=3, fontsize=8)
fig.suptitle(f"Loss vs users: PER <= 1 % and 98 % within {args.pdb:g} ms  ({note})", fontsize=10)
fig.tight_layout()
fig.savefig(os.path.join(plots, "5_per_loss.png"), dpi=150)
plt.close(fig)

# 4a) system p98 of the delay
fig, axs = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)
for ax, k, t in zip(axs, ("ul", "dl"), ("uplink", "downlink")):
    plot_metric(ax, f"delayP98Ms_{k}", "98th percentile of packet delay [ms]")
    ax.axhline(args.pdb, color="k", lw=0.8, ls="--")
    ax.set_title(f"{t}: all packets of all calls (UE-UPF)", fontsize=10)
axs[0].legend(ncol=3, fontsize=8)
fig.suptitle(f"Delay vs users: p98 of the pooled packets, PDB {args.pdb:g} ms dashed  ({note})", fontsize=10)
fig.tight_layout()
fig.savefig(os.path.join(plots, "4a_delay_p98.png"), dpi=150)
plt.close(fig)

# 4b) spread of the calls' own p98
cols = min(3, len(labels))
rws = math.ceil(len(labels) / cols)
fig, axs = plt.subplots(rws, cols, figsize=(5 * cols, 3.6 * rws), squeeze=False, sharey=True)
for i, l in enumerate(labels):
    ax = axs[i // cols][i % cols]
    for k, c in (("ul", "tab:red"), ("dl", "tab:blue")):
        xs, med, lo, hi = [], [], [], []
        for n in sorted(n for (ll, n) in group if ll == l):
            v = sorted(x for r in group[(l, n)] for x in r["callp98"][k])
            if not v:
                continue
            q = lambda p: v[min(len(v) - 1, max(0, int(round(p / 100 * (len(v) - 1)))))]  # noqa: E731
            xs.append(n); med.append(q(50)); lo.append(q(5)); hi.append(q(95))
        if xs:
            ax.plot(xs, med, color=c, marker="o", ms=3, label=f"{k.upper()} median")
            ax.fill_between(xs, lo, hi, color=c, alpha=0.2, label=f"{k.upper()} 5-95 %")
    ax.axhline(args.pdb, color="k", lw=0.8, ls="--")
    ax.set_title(l, fontsize=10)
    ax.grid(alpha=0.3)
    ax.set_xlabel("users (calls)")
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    if i % cols == 0:
        ax.set_ylabel("a call direction's p98 delay [ms]")
for j in range(len(labels), rws * cols):
    axs[j // cols][j % cols].axis("off")
axs[0][0].legend(fontsize=7)
fig.suptitle("Delay vs users: each call direction's 98th percentile, spread over the calls (all replicas)", fontsize=10)
fig.tight_layout()
fig.savefig(os.path.join(plots, "4b_delay_call_p98.png"), dpi=150)
plt.close(fig)

# 4c) CDFs, one curve per user count
for k, t in (("ul", "uplink"), ("dl", "downlink")):
    fig, axs = plt.subplots(rws, cols, figsize=(5 * cols, 3.6 * rws), squeeze=False, sharey=True)
    for i, l in enumerate(labels):
        ax = axs[i // cols][i % cols]
        ns = sorted(n for (ll, n) in group if ll == l)
        cmap = plt.get_cmap("viridis")
        for j, n in enumerate(ns):
            h = defaultdict(int)
            for r in group[(l, n)]:
                for ms, c in r["hist"][k].items():
                    h[ms] += c
            tot = sum(h.values())
            if not tot:
                continue
            xs, ys, acc = [], [], 0
            for ms in sorted(h):
                acc += h[ms]
                xs.append(ms)
                ys.append(acc / tot)
            ax.step(xs, ys, where="post", color=cmap(j / max(1, len(ns) - 1)), lw=1.2, label=f"{n}")
        ax.axvline(args.pdb, color="k", lw=0.8, ls="--")
        ax.axhline(0.98, color="grey", lw=0.6, ls=":")
        ax.set_xlim(0, 3 * args.pdb)
        ax.set_title(l, fontsize=10)
        ax.grid(alpha=0.3)
        ax.set_xlabel("delay UE-UPF [ms]")
        if i % cols == 0:
            ax.set_ylabel("CDF of packets")
        ax.legend(title="users", fontsize=7, title_fontsize=7, ncol=2, loc="lower right")
    for j in range(len(labels), rws * cols):
        axs[j // cols][j % cols].axis("off")
    fig.suptitle(f"{t}: delay CDF of all packets, one curve per user count (PDB dashed, 98 % dotted)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(plots, f"4c_delay_cdf_{k}.png"), dpi=150)
    plt.close(fig)

print(f"plot_sweep: {len(runs)} runs ({failed} failed or unfinished), {len(labels)} configurations -> {plots}")
with open(os.path.join(plots, "capacity.csv")) as f:
    print(f.read().strip())
