#!/usr/bin/env python3
"""Plots of a sweep made with sweep_nues.sh (the joined CSV files of ntnlab_nr --csv).

usage: python3 plot_sweep.py SWEEP_DIR [--scenario NAME] [--nues 10,50,100] [--bearers]
                              [--dir dl|ul] [--out DIR]

  --scenario  a sweep of several scenarios (sweep_nues.sh -S): the detailed plots of one of
              them, e.g. rohc1-sps1 (without it: the scenarios compared)
  --nues      the numbers of UEs to draw as separate curves in the CDFs (default: up to five,
              spread over those in the sweep)
  --bearers   per-user CDFs over bearers (one per connection) instead of calls
  --dir       two-way calls (--direction=both): the plots of one direction, dl (the far end
              talks) or ul (the UE talks); without it both, in OUT/dl and OUT/ul
  --out       where the PNG files go (default SWEEP_DIR/plots, or SWEEP_DIR/plots-NAME)

A sweep of several scenarios (ROHC on/off x dynamic, sps1, sps2) gets them compared, one line
each, against the number of UEs:

  scenarios_1_users_met_5qi1        users that met 5QI 1: 98 % within the PDB and PER <= 1 %
  scenarios_2_pdcch_utilization     CCEs used, % of those the PDCCH had
  scenarios_3_packets_within_pdb    packets within the PDB, % of all: the 98 % line, and in the
                                    legend the most UEs each scenario keeps at or above it
  scenarios_summary.csv             the three numbers per scenario and number of UEs

A sweep of one scenario gets:

Needs pandas and matplotlib (pip install pandas matplotlib). Every point of a curve over the
number of UEs pools all runs (RngRun) with that number of UEs; with several runs the shaded band
is the range of the runs.

  01_5qi1_met_vs_nues          calls that met 5QI 1: both rules, the delay rule, the loss rule
  02_lost_by_cause_vs_nues     lost packets by cause, % of those sent
  03_p98_delay_cdf             the 98 % delay of each call (CDF), against the PDB
  04_packet_delay_cdf          the UE-UPF delay of every delivered packet (CDF)
  05_per_and_over_pdb_ccdf     PER and packets over the PDB of each call (CCDF: most are 0)
  06_pdcch_vs_nues             CCE utilization, blocked DCIs, missed DCIs, CCEs per DCI
  07_pdcch_dcis_by_purpose     DCIs per second by purpose
  08_pdcch_levels              aggregation levels of the DCIs to UEs
  09_missed_dci_ccdf           missed DCIs of each call (CCDF)
  10_rohc_vs_nues              ROHC header size, bytes saved, decompressor drops, feedback
  11_rohc_context_loss_ccdf    time without a ROHC context per connection (CCDF)
  summary.csv                  the main numbers per number of UEs
"""
import argparse
import os
import sys
import textwrap

import matplotlib

matplotlib.use('Agg')
import matplotlib.ticker  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

# Colours: categorical slots in fixed order (identity), one blue ramp light -> dark (magnitude:
# the number of UEs, the aggregation level), chart ink and surfaces
CAT = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#4a3aa7', '#e34948']
RAMP = ['#86b6ef', '#5598e7', '#2a78d6', '#1c5cab', '#104281', '#0d366b']
SURFACE, INK, INK2, MUTED = '#fcfcfb', '#0b0b0b', '#52514e', '#898781'
GRID, AXIS = '#e1e0d9', '#c3c2b7'

plt.rcParams.update({
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
    'axes.edgecolor': AXIS, 'axes.labelcolor': INK2, 'axes.titlecolor': INK,
    'axes.titlesize': 11, 'axes.titleweight': 'bold', 'axes.labelsize': 9.5,
    'axes.spines.top': False, 'axes.spines.right': False,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
    'xtick.color': MUTED, 'ytick.color': MUTED, 'xtick.labelsize': 8.5, 'ytick.labelsize': 8.5,
    'legend.frameon': False, 'legend.fontsize': 8.5, 'lines.linewidth': 1.6,
    'font.family': 'sans-serif',
})


def ramp(n):
    """n steps of the blue ramp, light to dark"""
    if n <= 1:
        return [RAMP[2]]
    idx = np.linspace(0, len(RAMP) - 1, n).round().astype(int)
    return [RAMP[i] for i in idx]


def ecdf(values):
    x = np.sort(np.asarray(values, dtype=float))
    x = x[~np.isnan(x)]
    return x, np.arange(1, len(x) + 1) / max(len(x), 1)


def ccdf(values):
    """x and the fraction of the values above x, for a step plot (where='post') on a log y axis:
    from x = 0 to the largest value, which ends the curve at the fraction of one value"""
    v = np.sort(np.asarray(values, dtype=float))
    v = v[~np.isnan(v)]
    if not len(v):
        return np.array([]), np.array([])
    x = np.unique(np.concatenate([[0.0], v]))
    y = 1 - np.searchsorted(v, x, side='right') / len(v)
    keep = y > 0
    if not keep.any():
        return np.array([]), np.array([])
    return np.append(x[keep], x[-1]), np.append(y[keep], y[keep][-1])


def ccdf_axes(ax, linthresh, who, most):
    """symmetric-log x (0 shown, then decades), log y down to one value in a group of `most`"""
    ax.set_xscale('symlog', linthresh=linthresh, linscale=0.4)
    ax.set_yscale('log')
    ax.set_xlim(left=0)
    ax.set_ylim(0.5 / max(most, 1), 1.3)
    plain = matplotlib.ticker.FuncFormatter(lambda v, _: f'{v:g}')
    ax.xaxis.set_major_formatter(plain)
    ax.yaxis.set_major_formatter(plain)
    ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
    ax.set_ylabel(f'fraction of {who}s above x')


def save(fig, out, name, note=None):
    bottom = 0
    if note:
        # (each line wrapped to the width of the figure: about 4.2 pt a character at 7.5 pt)
        width = int(fig.get_figwidth() * 72 / 4.2)
        note = '\n'.join(textwrap.fill(line, width) for line in note.split('\n'))
        fig.text(0.01, 0.005, note, fontsize=7.5, color=MUTED, ha='left', va='bottom')
        bottom = (note.count('\n') + 1) * 11 / 72 / fig.get_figheight() + 0.01
    fig.tight_layout(rect=(0, bottom, 1, 1))
    path = os.path.join(out, name)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print('  ' + path)


def end_labels(ax, items, gap_pt=10):
    """direct labels at the ends of lines, in text ink, pushed apart vertically so that they do
    not overlap; items: (x, y, text) of each line's last point. Call it after the limits are set."""
    items = [i for i in items if i[0] is not None]
    if not items:
        return
    fig = ax.figure
    fig.canvas.draw()
    pt = 72 / fig.dpi
    ys = sorted(((ax.transData.transform((x, y))[1] * pt, x, y, t) for x, y, t in items),
                key=lambda i: i[0])
    placed = []
    for y0, *_ in ys:
        placed.append(max(y0, placed[-1] + gap_pt) if placed else y0)
    top = ax.transAxes.transform((0, 1))[1] * pt
    shift = max(0.0, placed[-1] - top)  # keep the highest label inside the axes height
    for (y0, x, y, t), y1 in zip(ys, placed):
        ax.annotate(t, (x, y), xytext=(4, y1 - shift - y0), textcoords='offset points',
                    fontsize=8, color=INK2, va='center')


def last(d):
    """(x, y) of the last point of a curve over the number of UEs"""
    return (d.nUes.values[-1], d.v.values[-1]) if len(d) else (None, None)


def by_nues(df, fn):
    """fn(rows of one run) -> number; per number of UEs: mean, min, max over its runs, and the
    value of all its runs pooled when fn is given the pooled rows"""
    rows = []
    for n, g in df.groupby('nUes'):
        per_run = [fn(r) for _, r in g.groupby('rngRun')]
        rows.append((n, fn(g), min(per_run), max(per_run)))
    return pd.DataFrame(rows, columns=['nUes', 'v', 'lo', 'hi']).sort_values('nUes')


def line_vs_nues(ax, d, color, label, band=True, z=2, lw=None):
    ax.plot(d.nUes, d.v, color=color, label=label, zorder=z, linewidth=lw)
    if band and (d.lo != d.hi).any():
        ax.fill_between(d.nUes, d.lo, d.hi, color=color, alpha=0.13, linewidth=0, zorder=z - 1)


def pick_nues(available, wanted):
    if wanted:
        got = [n for n in wanted if n in available]
        missing = [n for n in wanted if n not in available]
        if missing:
            print(f'  note: no runs with nUes = {missing}')
        return got
    if len(available) <= 5:
        return list(available)
    idx = np.linspace(0, len(available) - 1, 5).round().astype(int)
    return sorted(set(available[i] for i in idx))


def scenario_name(row):
    """rohc1-dynamic, rohc0-sps2, ...: the folder name of a run of sweep_nues.sh -S"""
    return f"rohc{int(row['rohc'])}-{row['scheduling']}"


# The scenarios: the scheduling by colour, ROHC by the line (solid on, dashed off). Each is
# drawn thinner than the one before, so that lines that coincide all stay in sight.
SCENARIOS = [(f'rohc{r}-{s}', f"ROHC {'on' if r else 'off'}, {s}", CAT[i], '-' if r else '--')
             for r in (1, 0) for i, s in enumerate(('dynamic', 'sps1', 'sps2'))]
WIDTHS = [3.6, 3.0, 2.5, 2.0, 1.6, 1.2]


def link_note(first, calls):
    """The link settings of the runs for a footer (columns of newer ntnlab_nr versions only)"""
    note = ''
    if 'ueAntennaGainDbi' in calls:
        note += f", UE antenna {first.ueAntennaGainDbi:g} dBi"
    if 'ntnScenario' in calls:  # the four losses of the service link
        note += f", {first.ntnScenario}, shadowing " + (
            f"{first.shadowingDb:g} dB" if first.shadowing == 'fixed' else 'by table')
        note += ", atmosphere " + (
            'by table' if first.atmosphere == 'table' else f"at latitude {first.latitudeDeg:g} deg")
        note += ", fast fading " + (
            f"{first.fastFadingDb:g} dB" if first.fastFading == 'fixed' else '3GPP model')
    else:  # versions with --largeScale and --ionosphere
        if 'largeScale' in calls and first.largeScale != 'fixed':
            note += (', every UE in LOS' if first.largeScale == 'los'
                     else f', {first.largeScale} LOS')
        if 'ionosphere' in calls:
            note += f", ionosphere {first.ionosphere}"
    if 'linkMarginDb' in calls and first.linkMarginDb:
        note += f", margin {first.linkMarginDb:g} dB"
    if 'ulGrantRbs' in calls and first.ulGrantRbs:
        note += f", UL grant {first.ulGrantRbs:g} RB"
    if 'errorModel' in calls and first.errorModel != 'NrLteMiErrorModel':
        note += f", {first.errorModel}"
    if 'farEnd' in calls and first.farEnd == 'ue':
        note += f", far end: UEs on {first.farEndBand}"
    return note


def direction_label(first, calls):
    """DL, UL, or the direction of the two-way calls the rows are of"""
    if 'dir' in calls and first.direction == 'both':
        return f"{first.dir.upper()} of two-way calls"
    return first.direction.upper()


def compare_scenarios(calls, pdcch, out):
    """The three plots that compare the scenarios of a sweep_nues.sh -S sweep"""
    calls = calls.copy()
    calls['scenario'] = calls.apply(scenario_name, axis=1)
    first = calls.iloc[0]
    ueGain = link_note(first, calls)
    setup = (f"{direction_label(first, calls)}, {first.scheduler}, {first.bandwidthMHz:g} MHz, "
             f"PDCCH{'' if first.idealControl else ' with errors'}"
             f"{', whole pass' if first.fullPass else ''}{ueGain}; runs: "
             f"{calls.groupby(['scenario', 'nUes']).rngRun.nunique().max()} per point, pooled")
    have = [sc for sc in SCENARIOS if sc[0] in set(calls.scenario)]
    available = sorted(int(n) for n in calls.nUes.unique())
    print(f'{len(have)} scenarios, {len(available)} numbers of UEs '
          f'({available[0]} to {available[-1]}), {len(calls)} calls')

    def per_nues(df, fn):
        rows = [(int(n), fn(g)) for n, g in df.groupby('nUes')]
        return pd.DataFrame(rows, columns=['nUes', 'v']).sort_values('nUes')

    def axes(ax, ylabel, title):
        ax.set(xlabel='UEs in the beam', ylabel=ylabel, title=title)
        ax.xaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
        ax.set_xlim(available[0] - 0.5, available[-1] + 0.5)

    # A user is a call; one that never got a bearer did not meet 5QI 1
    met = lambda g: 100.0 * g.satisfied.fillna(0).sum() / max(len(g), 1)
    # The delay rule of 5QI 1: late and discarded packets are over the PDB (lost ones count in
    # the PER rule)
    within = lambda g: 100.0 * (1 - (g.late.sum() + g.discard.sum()) / max(g.onBearer.sum(), 1))
    util = lambda g: 100.0 * g.cceUsed.sum() / max(g.ccePossible.sum(), 1)
    summary = []

    # 1. Users that met 5QI 1
    fig, ax = plt.subplots(figsize=(8.0, 4.6))
    for (name, label, color, dash), lw in zip(have, WIDTHS):
        d = per_nues(calls[calls.scenario == name], met)
        ax.plot(d.nUes, d.v, color=color, linestyle=dash, linewidth=lw, label=label)
        summary.append(d.assign(scenario=name, metric='usersMetPct'))
    ax.axhline(95, color=AXIS, linewidth=1, linestyle=':')
    axes(ax, 'users that met 5QI 1 [%]',
         'Users that met 5QI 1 (98 % within the PDB and PER <= 1 %)')
    ax.set_ylim(top=101)
    ax.legend(loc='lower left', ncol=2)
    save(fig, out, 'scenarios_1_users_met_5qi1.png',
         setup + '\nA user that never got a bearer did not meet it. Dotted: 95 %. Lines that '
         'coincide are drawn one on the other, the later thinner.')

    # 2. PDCCH utilization
    if pdcch is not None and len(pdcch):
        pdcch = pdcch.copy()
        pdcch['scenario'] = pdcch.apply(scenario_name, axis=1)
        fig, ax = plt.subplots(figsize=(8.0, 4.6))
        for (name, label, color, dash), lw in zip(have, WIDTHS):
            d = per_nues(pdcch[pdcch.scenario == name], util)
            ax.plot(d.nUes, d.v, color=color, linestyle=dash, linewidth=lw, label=label)
            summary.append(d.assign(scenario=name, metric='cceUtilizationPct'))
        axes(ax, 'CCEs used, % of the CCEs the PDCCH had', 'PDCCH utilization')
        ax.set_ylim(bottom=0)
        ax.legend(loc='upper left', ncol=2)
        save(fig, out, 'scenarios_2_pdcch_utilization.png', setup)

    # 3. Packets within the PDB, and the most UEs at or above 98 %
    fig, ax = plt.subplots(figsize=(8.0, 4.6))
    lowest = 100.0
    for (name, label, color, dash), lw in zip(have, WIDTHS):
        d = per_nues(calls[calls.scenario == name], within)
        good = d.v.values >= 98.0
        if good.all():
            most = f'98 % at all {d.nUes.values[-1]} UEs'
        elif not good[0]:
            most = f'below 98 % from {d.nUes.values[0]} UE{"s" if d.nUes.values[0] > 1 else ""}'
        else:
            most = f'98 % up to {d.nUes.values[np.argmin(good) - 1]} UEs'
        ax.plot(d.nUes, d.v, color=color, linestyle=dash, linewidth=lw, label=f'{label}: {most}')
        lowest = min(lowest, d.v.min())
        summary.append(d.assign(scenario=name, metric='packetsWithinPdbPct'))
        print(f'  {label}: {most}')
    ax.axhline(98, color=INK2, linewidth=1, linestyle=':')
    ax.annotate('98 %', (available[0] - 0.5, 98), xytext=(3, 3), textcoords='offset points',
                fontsize=8, color=INK2)
    axes(ax, 'packets within the PDB, % of all',
         'Packets within the PDB (5QI 1: 98 %), all users pooled')
    ax.set_ylim(min(lowest, 96.0) - 0.5, 100.2)
    ax.legend(loc='lower left', fontsize=8)
    save(fig, out, 'scenarios_3_packets_within_pdb.png',
         setup + '\nOver the PDB: delivered late, or discarded because they waited too long. '
         'Lost packets count in the PER rule.')

    t = pd.concat(summary).pivot_table(index=['scenario', 'nUes'], columns='metric',
                                       values='v').reset_index()
    path = os.path.join(out, 'scenarios_summary.csv')
    t.to_csv(path, index=False, float_format='%.3f')
    print('  ' + path)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sweep')
    ap.add_argument('--nues', default='')
    ap.add_argument('--bearers', action='store_true')
    ap.add_argument('--out', default='')
    ap.add_argument('--scenario', default='')
    ap.add_argument('--dir', default='', choices=['', 'dl', 'ul'])
    a = ap.parse_args()
    out = a.out or os.path.join(a.sweep, 'plots' + (f'-{a.scenario}' if a.scenario else ''))
    path = os.path.join(a.sweep, 'calls.csv')
    if not os.path.exists(path):
        sys.exit(f'{path} is missing: run sweep_nues.sh first')
    calls = pd.read_csv(path)
    dirs = sorted(calls.dir.unique()) if 'dir' in calls else []
    if a.dir and a.dir not in dirs:
        sys.exit(f'{a.sweep} has no {a.dir} flows')
    if not a.dir and len(dirs) > 1:
        for d in dirs:  # two-way calls: each direction on its own
            print(f'--- {d}')
            a.dir = d
            plot(a, os.path.join(out, d))
        return
    plot(a, out)


def plot(a, out):
    """The plots of the sweep (of one direction, a.dir, if the calls are two-way) into out"""
    os.makedirs(out, exist_ok=True)

    def read(kind):
        path = os.path.join(a.sweep, kind + '.csv')
        if not os.path.exists(path):
            return None
        df = pd.read_csv(path)
        if a.scenario:
            df = df[df.apply(scenario_name, axis=1) == a.scenario]
        if a.dir and 'dir' in df:
            df = df[df.dir.str.lower() == a.dir]  # (the ROHC files have DL and UL)
        if kind.startswith('rohc') and 'farEnd' in df:
            # the connections of the satellite UEs only: 7.0.0.2 to 7.0.0.(nUes + 1). The
            # terrestrial UEs of --farEnd=ue are 8.0.0.2 onwards (in runs of earlier versions
            # 7.0.0.(nUes + 2) onwards, which the number after 7.0.0.1 leaves out too)
            ip = df.address.str.split('.', expand=True).astype(int)
            df = df[(ip[0] == 7) & (ip[1] * 65536 + ip[2] * 256 + ip[3] - 2 < df.nUes)]
        return df

    calls_all = read('calls')
    if calls_all is None:
        sys.exit(f'{a.sweep}/calls.csv is missing: run sweep_nues.sh first')
    if a.scenario and calls_all.empty:
        sys.exit(f'{a.sweep} has no scenario {a.scenario}')
    if not a.scenario and calls_all.apply(scenario_name, axis=1).nunique() > 1:
        compare_scenarios(calls_all[calls_all.bearer == 0], read('pdcch'), out)
        return
    calls = calls_all[calls_all.bearer == 0]
    users = calls_all[calls_all.bearer > 0] if a.bearers else calls
    who = 'bearer' if a.bearers else 'call'
    available = sorted(int(n) for n in calls.nUes.unique())
    sel = pick_nues(available, [int(x) for x in a.nues.split(',') if x])
    colors = dict(zip(sel, ramp(len(sel))))
    first = calls.iloc[0]
    ueGain = link_note(first, calls)
    setup = (f"{direction_label(first, calls)}, {first.scheduling}, {first.scheduler}, "
             f"{first.bandwidthMHz:g} MHz, PDCCH {'on' if first.pdcch else 'off'}"
             f"{'' if first.idealControl else ' with errors'}, ROHC "
             f"{first.rohcModel if first.rohc else 'off'}"
             f"{', whole pass' if first.fullPass else ''}"
             f"{ueGain}"
             f"; runs: "
             f"{calls.rngRun.nunique()} per number of UEs")
    pdb = 100.0
    print(f'{a.sweep}: {len(available)} numbers of UEs ({available[0]} to {available[-1]}), '
          f'{len(calls)} calls; CDF curves for nUes = {sel}')

    # 1. Calls that met 5QI 1, against the number of UEs
    met = users[users.onBearer > 0]
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    series = [('both rules (PDB and PER)', lambda g: 100 * g.satisfied.mean()),
              ('delay rule: over PDB <= 2 %', lambda g: 100 * (1 - g.missedPdb.mean())),
              ('loss rule: PER <= 1 %', lambda g: 100 * (1 - g.missedPer.mean()))]
    labels = []
    # "both rules" is the wide line under the others (with the band of the runs): where a single
    # rule decides it, that rule's line runs inside it
    for k, ((label, fn), c) in reversed(list(enumerate(zip(series, CAT)))):
        d = by_nues(met, fn)
        line_vs_nues(ax, d, c, label, band=(k == 0), z=2 + k, lw=3.2 if k == 0 else 1.4)
        labels.append((*last(d), label.split(' (')[0].split(':')[0]))
    ax.axhline(95, color=AXIS, linewidth=1, linestyle='--')
    ax.annotate('95 %', (available[0], 95), xytext=(2, 3), textcoords='offset points',
                fontsize=8, color=MUTED)
    ax.set(xlabel='UEs in the beam', ylabel=f'{who}s that met the rule [%]',
           title='Voice calls that met 5QI 1 (TS 23.501: 98 % within the PDB, PER <= 1 %)')
    ax.set_ylim(top=101)
    h, l = ax.get_legend_handles_labels()
    ax.legend(h[::-1], l[::-1], loc='lower left')
    end_labels(ax, labels)
    save(fig, out, '01_5qi1_met_vs_nues.png', setup)

    # 2. Why packets were lost, against the number of UEs (all calls of a point pooled)
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    causes = [('no connection', 'noConn'), ('discarded late', 'discard'),
              ('lost on the radio', 'radio'), ('ROHC could not restore', 'rohcLost')]
    for (label, col), c in zip(causes, CAT):
        d = by_nues(calls, lambda g, col=col: 100 * g[col].sum() / max(g.tx.sum(), 1))
        line_vs_nues(ax, d, c, label)
    ax.set(xlabel='UEs in the beam', ylabel='% of the packets sent',
           title='Lost voice packets by cause')
    ax.legend(loc='upper left')
    save(fig, out, '02_lost_by_cause_vs_nues.png', setup)

    # 3. CDF of the 98 % delay of each call
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    for n in sel:
        x, y = ecdf(users[users.nUes == n].delayP98Ms)
        ax.step(x, y, where='post', color=colors[n], label=f'{n} UEs')
    ax.axvline(pdb, color=AXIS, linestyle='--', linewidth=1)
    ax.annotate('PDB 100 ms', (pdb, 0.02), xytext=(3, 0), textcoords='offset points',
                fontsize=8, color=MUTED)
    ax.set(xlabel=f'delay that 98 % of the packets of a {who} stayed within, UE-UPF [ms]',
           ylabel=f'fraction of {who}s', title=f'98 % delay of each {who} (CDF)')
    ax.set_ylim(0, 1.01)
    ax.legend(loc='lower right', title='')
    save(fig, out, '03_p98_delay_cdf.png', setup)

    # 4. CDF of the delay of every delivered packet, from the histograms
    delays = read('delays')
    if delays is not None and len(delays):
        fig, ax = plt.subplots(figsize=(7.2, 4.2))
        for n in sel:
            h = delays[delays.nUes == n].groupby('delayMs').packets.sum().sort_index()
            if h.empty:
                continue
            ax.step(h.index.values + 1, h.cumsum().values / h.sum(), where='post', color=colors[n],
                    label=f'{n} UEs')
        ax.axvline(pdb, color=AXIS, linestyle='--', linewidth=1)
        ax.set(xlabel='UE-UPF delay of a delivered packet [ms] (1 ms steps)',
               ylabel='fraction of the packets', title='Packet delay (CDF, all calls pooled)')
        ax.set_ylim(0, 1.01)
        ax.legend(loc='lower right')
        save(fig, out, '04_packet_delay_cdf.png', setup)

    # 5. PER and over-PDB per call: most calls have 0, so the tail (CCDF, log scales)
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 4.2))
    for ax, col, limit, title in ((axs[0], 'perPct', 1, 'PER of each ' + who),
                                  (axs[1], 'overPdbPct', 2, 'Over the PDB, each ' + who)):
        for n in sel:
            x, y = ccdf(met[met.nUes == n][col])
            ax.step(x, y, where='post', color=colors[n], label=f'{n} UEs')
        ax.axvline(limit, color=AXIS, linestyle='--', linewidth=1)
        ax.annotate(f'{limit} %', (limit, 1), xytext=(3, -10), textcoords='offset points',
                    fontsize=8, color=MUTED)
        ccdf_axes(ax, 0.01, who, max(int((met.nUes == n).sum()) for n in sel))
        ax.set(xlabel='% of the packets on the bearer', title=title + ' (CCDF)')
    axs[1].legend(loc='upper right')
    save(fig, out, '05_per_and_over_pdb_ccdf.png',
         setup + f'\nAt the dashed line: the {who}s that missed the rule. '
         f'A curve that is missing: no {who} above 0.')

    # 6. The PDCCH against the number of UEs
    pd_ = read('pdcch')
    if pd_ is not None and len(pd_):
        pd_ = pd_.copy()
        received = [c for c in pd_.columns if c.startswith('al') and c.endswith('Received')]
        pd_['ueDcis'] = pd_[received].sum(axis=1)
        fig, axs = plt.subplots(2, 2, figsize=(9.6, 6.4))
        panels = [('CCE utilization', '% of the CCEs possible',
                   lambda g: 100 * g.cceUsed.sum() / g.ccePossible.sum()),
                  ('Blocked DCIs', '% of the DCIs wanted',
                   lambda g: 100 * g.blocked.sum() / max((g.dcis + g.blocked).sum(), 1)),
                  ('Missed DCIs (not decoded)', '% of the DCIs to UEs',
                   lambda g: 100 * g.missed.sum() / max(g.received.sum(), 1)),
                  ('CCEs per DCI', 'CCEs', lambda g: g.cceUsed.sum() / max(g.dcis.sum(), 1))]
        for ax, (title, ylabel, fn) in zip(axs.flat, panels):
            line_vs_nues(ax, by_nues(pd_, fn), CAT[0], 'all DCIs')
            ax.set(title=title, ylabel=ylabel, xlabel='UEs in the beam')
        # CCEs per DCI: all of them is a mix of the common DCIs (SIB1, random access) at their
        # fixed high level and the DCIs to UEs at the levels of their SINR; apart, as well
        ax = axs.flat[3]
        common = ['sib1', 'rar']
        toUes = ['dlNew', 'dlRetx', 'dlSpsActivation', 'dlSpsRelease', 'ulNew', 'ulRetx',
                 'ulCgActivation', 'ulCgRelease']
        for (label, keys), c in zip((('DCIs to UEs', toUes), ('SIB1 + random access', common)),
                                    CAT[1:]):
            line_vs_nues(ax, by_nues(pd_, lambda g, keys=keys:
                                     sum(g[k + 'Cces'].sum() for k in keys) /
                                     max(sum(g[k + 'Dcis'].sum() for k in keys), 1)), c, label)
        ax.legend(loc='best')
        save(fig, out, '06_pdcch_vs_nues.png', setup)

        # 7. DCIs per second by purpose
        purposes = [('SIB1', 'sib1'), ('random access', 'rar'), ('DL new data', 'dlNew'),
                    ('DL retransmission', 'dlRetx'), ('DL SPS activation', 'dlSpsActivation'),
                    ('DL SPS release', 'dlSpsRelease'), ('UL new data', 'ulNew'),
                    ('UL retransmission', 'ulRetx'), ('UL CG activation', 'ulCgActivation'),
                    ('UL CG release', 'ulCgRelease')]
        used = [p for p in purposes if pd_[p[1] + 'Dcis'].sum() > 0]
        if len(used) > 8:  # fold the smallest into "other"
            used.sort(key=lambda p: -pd_[p[1] + 'Dcis'].sum())
            used = used[:7] + [('other', [p[1] for p in used[7:]])]
        fig, ax = plt.subplots(figsize=(7.2, 4.4))
        secs = lambda g: (g.windowEndS - g.windowStartS).sum()
        for (label, key), c in zip(used, CAT):
            keys = key if isinstance(key, list) else [key]
            d = by_nues(pd_, lambda g, keys=keys: sum(g[k + 'Dcis'].sum() for k in keys) / secs(g))
            line_vs_nues(ax, d, c, label)
        ax.set(xlabel='UEs in the beam', ylabel='DCIs per second', title='DCIs sent, by purpose')
        ax.legend(loc='upper left', ncol=2)
        save(fig, out, '07_pdcch_dcis_by_purpose.png', setup)

        # 8. Aggregation levels of the DCIs to UEs
        levels = [l for l in (1, 2, 4, 8, 16) if pd_[f'al{l}Received'].sum() > 0]
        if levels:
            fig, ax = plt.subplots(figsize=(7.2, 4.2))
            labels = []
            for l, c in zip(levels, ramp(len(levels))):
                d = by_nues(pd_, lambda g, l=l:
                            100 * g[f'al{l}Received'].sum() / max(g.ueDcis.sum(), 1))
                line_vs_nues(ax, d, c, f'level {l}')
                labels.append((*last(d), f'{l}'))
            ax.set(xlabel='UEs in the beam', ylabel='% of the DCIs to UEs',
                   title='Aggregation level of the DCIs to UEs')
            ax.set_ylim(-2, 105)
            ax.legend(loc='best')
            end_labels(ax, labels)
            save(fig, out, '08_pdcch_levels.png', setup)

        # 9. Missed DCIs per UE (CCDF: most UEs miss none)
        if calls.dcisMissed.fillna(0).sum() > 0:
            fig, ax = plt.subplots(figsize=(7.2, 4.2))
            src = users[users.dcis > 0]
            for n in sel:
                x, y = ccdf(src[src.nUes == n].dciMissedPct)
                ax.step(x, y, where='post', color=colors[n], label=f'{n} UEs')
            ccdf_axes(ax, 0.01, who, max(int((src.nUes == n).sum()) for n in sel))
            ax.set(xlabel='missed DCIs, % of the DCIs to the UE',
                   title=f'Missed DCIs of each {who} (CCDF)')
            ax.legend(loc='upper right')
            save(fig, out, '09_missed_dci_ccdf.png',
                 setup + f'\nA curve that is missing: no {who} missed a DCI.')

    # 10. ROHC against the number of UEs
    rohc = read('rohc')
    if rohc is not None and len(rohc):
        r = rohc.copy()
        fig, axs = plt.subplots(2, 2, figsize=(9.6, 6.4))
        wmean = lambda g, col: (g[col] * g.packets).sum() / max(g.packets.sum(), 1)
        panels = [('Mean ROHC header', 'bytes per packet', lambda g: wmean(g, 'headerOutB')),
                  ('Bytes saved by ROHC', '% of the bytes of the packets',
                   lambda g: wmean(g, 'savedPct')),
                  ('Dropped by the decompressor', '% of the packets that arrived',
                   lambda g: 100 * g.failed.sum() / max(g.arrived.sum(), 1))]
        for ax, (title, ylabel, fn) in zip(axs.flat, panels):
            line_vs_nues(ax, by_nues(r, fn), CAT[0], title)
            ax.set(title=title, ylabel=ylabel, xlabel='UEs in the beam')
        ax = axs.flat[3]
        for (label, col), c in zip((('ACK', 'fbSentAck'), ('NACK', 'fbSentNack'),
                                    ('STATIC-NACK', 'fbSentStaticNack')), CAT):
            line_vs_nues(ax, by_nues(r, lambda g, col=col: g[col].sum() / max(len(g), 1)), c, label)
        ax.set(title='Feedback sent by the decompressor', ylabel='per connection',
               xlabel='UEs in the beam')
        ax.legend(loc='best')
        save(fig, out, '10_rohc_vs_nues.png', setup)

        # 11. Context losses per connection (CCDF of the time without a context: most have none)
        fig, ax = plt.subplots(figsize=(7.2, 4.2))
        for n in sel:
            x, y = ccdf(r[r.nUes == n].contextLostS)
            ax.step(x, y, where='post', color=colors[n], label=f'{n} UEs')
        ccdf_axes(ax, 0.01, 'connection', max(int((r.nUes == n).sum()) for n in sel))
        ax.set(xlabel='time the decompressor had no usable context [s]',
               title='ROHC context loss per connection (CCDF)')
        ax.legend(loc='upper right')
        save(fig, out, '11_rohc_context_loss_ccdf.png',
             setup + '\nA connection is one RNTI of a bearer. A curve that is missing: no context '
             'loss.')

    # A table of the main numbers
    cols = [('met5qi1Pct', met, lambda g: 100 * g.satisfied.mean()),
            ('p98DelayMedianMs', users, lambda g: g.delayP98Ms.median()),
            ('perMeanPct', met, lambda g: g.perPct.mean()),
            ('overPdbMeanPct', met, lambda g: g.overPdbPct.mean())]
    if pd_ is not None and len(pd_):
        cols += [('cceUtilPct', pd_, lambda g: 100 * g.cceUsed.sum() / g.ccePossible.sum()),
                 ('blockedPct', pd_,
                  lambda g: 100 * g.blocked.sum() / max((g.dcis + g.blocked).sum(), 1)),
                 ('missedDciPct', pd_, lambda g: 100 * g.missed.sum() / max(g.received.sum(), 1))]
    t = pd.DataFrame({'nUes': available})
    for name, df, fn in cols:
        t = t.merge(by_nues(df, fn)[['nUes', 'v']].rename(columns={'v': name}), on='nUes',
                    how='left')
    path = os.path.join(out, 'summary.csv')
    t.to_csv(path, index=False, float_format='%.3f')
    print('  ' + path)
    print(t.to_string(index=False, float_format=lambda v: f'{v:.2f}'))


if __name__ == '__main__':
    main()
