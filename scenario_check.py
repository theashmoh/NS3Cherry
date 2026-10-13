"""Why the scenario plots of a sweep_nues.sh -S sweep look as they do.

usage: python3 scenario_check.py SWEEP_DIR [--from N] [--to N] [--dir dl|ul]

  1. the users that missed 5QI 1: how many, and how much of their call they had line of sight
  2. the PDCCH by purpose for every scenario and number of UEs: DCIs per 1000 packets on the
     bearers, CCEs per DCI, CCEs as % of the CCEs possible (as in scenarios_2), blocked DCIs
  3. --from A --to B (default: the two largest numbers of UEs): load or the new users? The
     drops are nested (UE k is at the same place, with the same obstruction, in every run of
     one RngRun), so UEs 0..A-1 can be compared at A UEs and at B UEs. If they cost as many
     CCEs per packet and fail as often at B as at A, what changed between A and B came with
     the new UEs B-A..B-1, not with the load.
Two-way calls (--direction=both): 1 and 3 for each direction (dl: the far end talks, ul: the UE
talks), or for --dir only; a user is then one direction of a call.
"""
import argparse
import os

import pandas as pd

KEYS = ['rohc', 'scheduling', 'nUes']


def name(df):
    return 'rohc' + df.rohc.astype(int).astype(str) + '-' + df.scheduling


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sweep')
    ap.add_argument('--from', dest='a', type=int, default=0)
    ap.add_argument('--to', dest='b', type=int, default=0)
    ap.add_argument('--dir', default='', choices=['', 'dl', 'ul'])
    a = ap.parse_args()
    calls = pd.read_csv(os.path.join(a.sweep, 'calls.csv'))
    calls = calls[calls.bearer == 0].copy()
    calls['scenario'] = name(calls)
    calls['met'] = calls.satisfied.fillna(0).astype(int)
    dirs = [a.dir] if a.dir else (sorted(calls.dir.unique()) if 'dir' in calls else [''])
    for d in dirs:
        if len(dirs) > 1 or a.dir:
            print(f'===== {d}: ' + ('the far end talks' if d == 'dl' else 'the UE talks') + '\n')
        users(calls[calls.dir == d] if d else calls, a)
        if d != dirs[-1]:
            print()
    pdcch(calls, a)


def users(calls, a):
    """1 and 3: the users that missed 5QI 1, and load or new users"""

    # 1. Users that missed 5QI 1
    miss = calls[calls.met == 0]
    print(f'1. USERS THAT MISSED 5QI 1: {len(miss)} of {len(calls)} calls')
    t = calls.groupby(['scenario', 'nUes']).agg(calls=('met', 'size'),
                                                missed=('met', lambda m: int((m == 0).sum())))
    print(t.unstack('nUes')['missed'].to_string(line_width=130))
    print('\nby the share of the call the UE had line of sight (LOS[%]):')
    bins = [-0.1, 10, 25, 50, 75, 99.9, 100]
    labels = ['0-10', '10-25', '25-50', '50-75', '75-99.9', '100']
    los = pd.DataFrame({'calls': pd.cut(calls.losPct, bins, labels=labels).value_counts(),
                        'missed': pd.cut(miss.losPct, bins, labels=labels).value_counts()})
    los = los.reindex(labels).fillna(0).astype(int)
    los['missed[%]'] = (100 * los.missed / los.calls.where(los.calls > 0)).round(1)
    los.index.name = 'LOS[%]'
    print(los.astype(object).T.to_string(line_width=130))
    print('\nmissed: PDB only', int(((miss.missedPdb == 1) & (miss.missedPer != 1)).sum()),
          ', PER only', int(((miss.missedPer == 1) & (miss.missedPdb != 1)).sum()),
          ', both', int(((miss.missedPer == 1) & (miss.missedPdb == 1)).sum()),
          ', no bearer', int(miss.missedPer.isna().sum()))
    cols = ['scenario', 'nUes', 'rngRun', 'ue', 'losPct', 'rlf', 'outS', 'dlSinrDb', 'onBearer',
            'perPct', 'overPdbPct']
    print('\nat the largest number of UEs:')
    print(miss[miss.nUes == miss.nUes.max()][cols].round(2).to_string(index=False, line_width=130))

    users_ab(calls, a)


def pdcch(calls, a):
    """2: the PDCCH by purpose (the DCIs of both directions, against the packets of both)"""
    path = os.path.join(a.sweep, 'pdcch.csv')
    if os.path.exists(path):
        p = pd.read_csv(path)
        parts = ['sib1', 'rar', 'dlNew', 'dlRetx', 'dlSpsActivation', 'dlSpsRelease', 'ulNew',
                 'ulRetx', 'ulCgActivation', 'ulCgRelease']
        parts = [x for x in parts if x + 'Dcis' in p]
        s = p.groupby(KEYS)[[x + y for x in parts for y in ('Dcis', 'Cces')] +
                            ['ccePossible', 'cceUsed', 'dcis', 'blocked']].sum()
        s['onBearer'] = calls.groupby(KEYS).onBearer.sum()
        r = pd.DataFrame(index=s.index)
        r['util%'] = 100 * s.cceUsed / s.ccePossible
        r['sib1%'] = 100 * s.sib1Cces / s.ccePossible
        for x, short in (('dlNew', 'new'), ('dlRetx', 'retx'), ('dlSpsActivation', 'act')):
            if x + 'Dcis' in s:
                r[short + '/1k'] = 1000 * s[x + 'Dcis'] / s.onBearer
                r[short + 'Cce'] = s[x + 'Cces'] / s[x + 'Dcis'].where(s[x + 'Dcis'] > 0)
                r[short + '%'] = 100 * s[x + 'Cces'] / s.ccePossible
        r['blocked%'] = 100 * s.blocked / s.dcis.where(s.dcis > 0)
        print('\n2. PDCCH BY PURPOSE (/1k: DCIs per 1000 packets on the bearers; Cce: CCEs per '
              'DCI; %: of the CCEs possible)')
        print(r.round(2).to_string(line_width=130))


def users_ab(calls, a):
    """3: load or new users, between two numbers of UEs"""
    ns = sorted(calls.nUes.unique())
    na = a.a or (ns[-2] if len(ns) > 1 else ns[0])
    nb = a.b or ns[-1]
    print(f'\n3. FROM {na} TO {nb} UEs: UEs 0..{na - 1} at both, UEs {na}..{nb - 1} new at {nb}')
    rows = []
    for sc, g in calls.groupby('scenario'):
        old_a = g[(g.nUes == na) & (g.ue < na)]
        old_b = g[(g.nUes == nb) & (g.ue < na)]
        new_b = g[(g.nUes == nb) & (g.ue >= na)]
        both = old_a.merge(old_b, on=['rngRun', 'ue'], suffixes=('_a', '_b'))
        per = lambda d: d.dciCces.sum() / max(d.onBearer.sum(), 1) * 1000
        rows.append({'scenario': sc,
                     f'old CCE/1k at {na}': per(old_a), f'old CCE/1k at {nb}': per(old_b),
                     f'new CCE/1k at {nb}': per(new_b),
                     'old met->missed': int(((both.met_a == 1) & (both.met_b == 0)).sum()),
                     'old missed->met': int(((both.met_a == 0) & (both.met_b == 1)).sum()),
                     'new missed': f'{int((new_b.met == 0).sum())} of {len(new_b)}'})
    print(pd.DataFrame(rows).set_index('scenario').round(1).to_string(line_width=130))
    print('CCE/1k: CCEs of the DCIs to these UEs per 1000 of their packets on the bearer '
          '(SIB1 not in it).')


if __name__ == '__main__':
    main()
