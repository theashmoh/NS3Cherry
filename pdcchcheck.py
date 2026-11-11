#!/usr/bin/env python3
"""Computes again, from TS 38.211/38.213 and the rules stated in nr-pdcch.h, everything the
log of scratch/pdcchtest.cc says NrPdcch decided, and compares. Written from the rules, not
from nr-pdcch.cc.   ./ns3 run pdcchtest > pdcchtest.log ; python3 pdcchcheck.py pdcchtest.log"""
import math
import sys

TABLE = [(-4.0, 0.9303), (-2.0, 0.4356), (-1.0, 0.1453), (0.0, 0.0246), (0.5, 0.00797), (1.0, 0.00221)]
LEVELS = [1, 2, 4, 8, 16]
COMMON_CANDIDATES = {4: 4, 8: 2, 16: 1}  # TS 38.213 Table 10.1-1
(SIB1, OTHER_SI, PAGING, RAR, TPC, RRC_DL, RRC_UL, DL_NEW, DL_RETX, DL_ACT, DL_REL, UL_NEW, UL_RETX,
 UL_ACT, UL_REL) = range(15)
N_PURPOSES = 15
UPLINK = {UL_NEW, UL_RETX, UL_ACT, UL_REL, RRC_UL}
COMMON = {SIB1, OTHER_SI, PAGING, RAR, TPC}
MASK = (1 << 64) - 1


def slot_random(slot, salt):
    """splitmix64 of the slot and the salt, in [0, 1): the rule of NrPdcch::SlotRandom"""
    z = (slot * 0x9e3779b97f4a7c15 + salt * 0xbf58476d1ce4e5b9 + 1) & MASK
    z = ((z ^ (z >> 30)) * 0xbf58476d1ce4e5b9) & MASK
    z = ((z ^ (z >> 27)) * 0x94d049bb133111eb) & MASK
    z = z ^ (z >> 31)
    return (z >> 11) / float(1 << 53)
A_P = {0: 39827, 1: 39829, 2: 39839}  # TS 38.213 10.1
D = 65537


def bler(level, sinr_db, dci_bits, boost_db=0.0):
    payload = dci_bits + 24
    if payload > 108 * level:  # 54 QPSK symbols per CCE
        return 1.0
    x = sinr_db + boost_db + 10 * math.log10(level) - 10 * math.log10(payload / 58.0)
    seg = len(TABLE) - 2
    for i in range(len(TABLE) - 1):
        if x <= TABLE[i + 1][0]:
            seg = i
            break
    (x0, b0), (x1, b1) = TABLE[seg], TABLE[seg + 1]
    y = math.log10(b0) + (math.log10(b1) - math.log10(b0)) * (x - x0) / (x1 - x0)
    return min(1.0, 10 ** y)


def sinr_for(level, target, dci_bits):
    lo, hi = -100.0, 100.0
    for _ in range(200):
        mid = (lo + hi) / 2
        if bler(level, mid, dci_bits) > target:
            lo = mid
        else:
            hi = mid
    return hi


class Model:
    def __init__(self, cfg):
        self.cfg = cfg
        self.credit = 0.0
        self.dl_demand = 0
        self.prev_dl_demand = 0
        self.sinr = {}
        self.stats = dict(slots=0, by_symbols=[0, 0, 0, 0], avail=0, used=0, above=0, raised=0,
                          noreport=0, unused=0, capped=0, dci=[0] * N_PURPOSES,
                          cce=[0] * N_PURPOSES, blocked=[0] * N_PURPOSES, dci_al=[0] * 5,
                          cce_al=[0] * 5)
        # the cell's own DCIs: waiting (rnti, purpose, due slot), UEs and when they connected
        self.pending = []
        self.connected = {}
        self.configured = (0, 0)
        self.cell = None

    # --- the cell's own DCIs, from the rules in nr-pdcch.h
    def queue_rrc(self, rnti, base, dl, ul):
        step = self.cell['step']
        due = base
        for i in range(max(dl, ul)):
            for purpose, n in ((RRC_DL, dl), (RRC_UL, ul)):
                if i < n:
                    due += step
                    self.pending.append((rnti, purpose, due))

    def add_ue(self, rnti, slot):
        if rnti in self.connected:
            return
        self.connected[rnti] = slot
        self.queue_rrc(rnti, slot, *self.cell['setup'])

    def call_starts(self, rnti, slot):
        if rnti in self.connected:
            self.queue_rrc(rnti, slot, *self.cell['call'])

    def cell_dcis(self, slot):
        c = self.cell
        frame, in_frame = divmod(slot, 10)
        for k, period in enumerate(c['si']):  # SI message k: frame k + 1 of its period
            pf = period // 10
            s = (1 + 2 * k) % 10
            if s == 5:
                s = 6
            if frame % pf == (k + 1) % pf and in_frame == s:
                self.pending.append((0, OTHER_SI, slot))
        if c['paging_rate'] > 0 and in_frame == 9:  # paging occasion: last slot of a paging frame
            frames = c['cycle'] // 10
            n = c['paging_frames'] or frames
            if frame % (frames // n) == 0:
                per_occasion = c['paging_rate'] * c['cycle'] / 1000.0 / n
                if slot_random(slot, 1) < 1.0 - math.exp(-per_occasion):
                    self.pending.append((0, PAGING, slot))
        if c['tpc'] > 0 and slot % c['tpc'] == 0:  # DCI 2_2
            per = c['tpc_ues']
            n = (self.configured[0] + per - 1) // per + (self.configured[1] + per - 1) // per
            self.pending += [(0, TPC, slot)] * n
        if c['meas'] > 0:
            for rnti in sorted(self.connected):
                since = self.connected[rnti]
                if slot > since and (slot - since) % c['meas'] == 0:
                    self.pending.append((rnti, RRC_UL, slot))
        keep = []
        for rnti, purpose, due in self.pending:
            if due > slot:
                keep.append((rnti, purpose, due))
                continue
            e = self.request(rnti, purpose)
            if e is None:
                keep.append((rnti, purpose, due))
            else:
                e['committed'] = True
        self.pending = keep

    # --- static rules
    def n_cce(self, symbols):
        return self.cfg['coreset'] * symbols // 6

    def max_symbols(self):
        return 3 if self.cfg['symbols'] == 0 else self.cfg['symbols']

    def candidates(self, level, common):
        if common:
            return COMMON_CANDIDATES.get(level, 0)
        return self.cfg['cand'][LEVELS.index(level)]

    def common_level(self):
        best = 0
        for l in (4, 8, 16):
            if l <= 8 and l <= self.n_cce(self.max_symbols()):
                best = l
        return best

    def choose(self, sinr):
        """lowest level with BLER <= 1 %; (level, meets target, no report)"""
        highest = 0
        for l in LEVELS:
            if self.candidates(l, False) == 0 or l > self.n_cce(self.max_symbols()):
                continue
            highest = l
            if sinr is None:
                continue
            if bler(l, sinr, self.cfg['dci']) <= 0.01:
                return l, True, False
        return highest, sinr is None, sinr is None

    def candidate_cce(self, rnti, common, level, m, symbols, slot):
        """TS 38.213 10.1: first CCE of candidate m"""
        n = self.n_cce(symbols)
        y = 0
        if not common:
            p = symbols if self.cfg['symbols'] == 0 else 1
            y = rnti
            for _ in range(slot % 10 + 1):  # Y_{p,-1} = n_RNTI, then one step per slot up to n
                y = (A_P[p % 3] * y) % D
        big_m = self.candidates(level, common)
        return level * ((y + (m * n) // (level * big_m)) % (n // level))

    def find_free(self, owner, rnti, common, level, symbols, slot):
        if level > self.n_cce(symbols):
            return None
        for m in range(self.candidates(level, common)):
            first = self.candidate_cce(rnti, common, level, m, symbols, slot)
            if all(owner[first + i] is None for i in range(level)):
                return first
        return None

    # --- one slot
    def start_slot(self, slot, twice=False):
        self.slot = slot
        self.symbols = self.max_symbols()
        n = self.n_cce(self.symbols)
        self.prev_dl_demand = self.dl_demand
        self.dl_demand = 0
        self.ul_cce = 0
        self.entries = []
        self.owner = [None] * n
        share = self.cfg['share'] * n
        cap = max(share, min(16, n))
        for _ in range(2 if twice else 1):
            self.credit = min(self.credit + share, cap) if share > 0 else 0.0
        frame, in_frame = divmod(slot, 10)
        if not twice and frame % 2 == 0 and in_frame == 5:  # SIB1 every 20 ms
            e = self.request(0, SIB1)
            if e is not None:
                e['committed'] = True
        self.cell_dcis(slot)

    def request(self, rnti, purpose):
        common = purpose in COMMON
        uplink = purpose in UPLINK
        n = self.n_cce(self.symbols)
        if common:
            wanted, meets, noreport = self.common_level(), True, False
        else:
            wanted, meets, noreport = self.choose(self.sinr.get(rnti))
        if not uplink:
            self.dl_demand += wanted
        capped = False
        for l in LEVELS:
            if l < wanted or l > n:
                continue
            if l > wanted and (common or not self.cfg['raise']):
                continue
            charge = 0
            if uplink:
                idle = n - min(n, self.prev_dl_demand)
                if self.ul_cce + l > idle and self.credit + 1e-9 < l:
                    # held back by the share only if a candidate was free
                    if self.find_free(self.owner, rnti, common, l, self.symbols,
                                      self.slot) is not None:
                        capped = True
                    continue
                charge = min(l, math.floor(self.credit + 1e-9))
            first = self.find_free(self.owner, rnti, common, l, self.symbols, self.slot)
            if first is None:
                continue
            for i in range(l):
                self.owner[first + i] = True
            if uplink:
                self.ul_cce += l
                self.credit -= charge
            e = dict(rnti=rnti, purpose=purpose, al=l, cce=first, charge=charge, meets=meets,
                     noreport=noreport, raised=l > wanted, committed=False)
            self.entries.append(e)
            return e
        self.stats['blocked'][purpose] += 1
        self.stats['capped'] += capped
        return None

    def release(self, e):
        for i in range(e['al']):
            self.owner[e['cce'] + i] = None
        if e['purpose'] in UPLINK:
            self.ul_cce -= e['al']
            self.credit += e['charge']
        self.entries.remove(e)
        self.stats['unused'] += 1

    def fix(self):
        if self.cfg['symbols'] != 0:
            return self.symbols
        for d in (1, 2):  # the fewest symbols whose CORESET holds every DCI, placed in order
            owner = [None] * self.n_cce(d)
            placed = []
            for e in self.entries:
                first = self.find_free(owner, e['rnti'], e['purpose'] in COMMON, e['al'], d, self.slot)
                if first is None:
                    break
                for i in range(e['al']):
                    owner[first + i] = True
                placed.append(first)
            else:
                for e, first in zip(self.entries, placed):
                    e['cce'] = first
                self.symbols = d
                break
        return self.symbols

    def finish(self):
        st = self.stats
        used = 0
        for e in self.entries:
            if not e['committed']:
                st['unused'] += 1
                continue
            i = LEVELS.index(e['al'])
            used += e['al']
            st['dci'][e['purpose']] += 1
            st['cce'][e['purpose']] += e['al']
            st['dci_al'][i] += 1
            st['cce_al'][i] += e['al']
            st['above'] += not e['meets']
            st['raised'] += e['raised']
            st['noreport'] += e['noreport']
        st['slots'] += 1
        st['by_symbols'][self.symbols if used else 0] += 1
        st['avail'] += self.n_cce(self.symbols)
        st['used'] += used


def main(path):
    models, errors, checked = {}, [], 0
    asks = {}
    order = []
    configured = {}
    calls = {}
    for line in open(path):
        f = line.split()
        if not f:
            continue
        if f[0] == 'CONFIG':
            cfg = dict(name=f[1], rbs=int(f[2]), symbols=int(f[3]), dci=int(f[4]),
                       coreset=int(f[5]) or int(f[2]) // 6 * 6, share=float(f[6]),
                       cand=[int(x) for x in f[8].split(',')])
            cfg['raise'] = f[7] == '1'
            m = models[f[1]] = Model(cfg)
            got = [int(f[9]), int(f[10]), int(f[11]), int(f[12]), int(f[13])]
            want = [cfg['coreset'], m.n_cce(1), m.n_cce(2), m.n_cce(3), m.common_level()]
            checked += 1
            if got != want:
                errors.append(f'CONFIG {f[1]}: {got} != {want}')
        elif f[0] == 'LEVEL':
            m, l = models[f[1]], int(f[2])
            want = [m.candidates(l, False), sinr_for(l, 0.01, m.cfg['dci']), bler(l, -3.3, m.cfg['dci']),
                    bler(l, 0.7, m.cfg['dci']), bler(l, -9.0, m.cfg['dci'], 4.0)]
            got = [int(f[3])] + [float(x) for x in f[4:8]]
            checked += 1
            if got[0] != want[0] or any(abs(a - b) > 1e-7 * max(1.0, abs(b)) for a, b in zip(got[1:], want[1:])):
                errors.append(f'LEVEL {f[1]} {l}: {got} != {want}')
        elif f[0] == 'UE':
            models[f[1]].sinr[int(f[2])] = None if f[3] == 'nan' else float(f[3])
        elif f[0] == 'CELL':
            m = models[f[1]]
            m.cell = dict(si=[] if f[2] == '-' else [int(x) for x in f[2].split(',')],
                          cycle=int(f[3]), paging_frames=int(f[4]), paging_rate=float(f[5]),
                          tpc=int(f[6]), tpc_ues=int(f[7]),
                          setup=[int(x) for x in f[8].split(':')],
                          call=[int(x) for x in f[9].split(':')], step=int(f[10]),
                          meas=int(f[11]))
            m.ues_to_add = True
        elif f[0] == 'CONFIGURED':
            configured[(f[1], int(f[2]))] = (int(f[3]), int(f[4]))
        elif f[0] == 'CALL':
            calls.setdefault((f[1], int(f[2])), []).append(int(f[3]))
        elif f[0] == 'SLOT':
            key = (f[1], int(f[2]))
            order.append((key, int(f[3])))
            asks[key] = []
        elif f[0] == 'ASK':
            asks[(f[1], int(f[2]))].append([int(x) for x in f[3:]])
    # replay the slots in order
    first_slot = {}
    for (name, slot), symbols in order:
        m = models[name]
        if name in first_slot:
            m.finish()
        twice = name not in first_slot
        if twice:
            # the UEs connected before the first slot, when NrPdcch's slot was still 0
            for rnti in sorted(m.sinr):
                m.add_ue(rnti, 0)
        first_slot.setdefault(name, slot)
        m.configured = configured.get((name, slot), (0, 0))
        m.start_slot(slot, twice)
        for rnti in calls.get((name, slot), []):
            m.call_starts(rnti, slot)
        made = []
        for rnti, purpose, al, released, cce, sym, dci_al in asks[(name, slot)]:
            e = m.request(rnti, purpose)
            made.append(e)
            checked += 1
            if (e['al'] if e else 0) != al:
                errors.append(f'{name} slot {slot} RNTI {rnti} purpose {purpose}: level {al}, '
                              f'expected {e["al"] if e else 0}')
        for e, (rnti, purpose, al, released, cce, sym, dci_al) in zip(made, asks[(name, slot)]):
            if e is not None and released:
                m.release(e)
        for e, (rnti, purpose, al, released, cce, sym, dci_al) in zip(made, asks[(name, slot)]):
            if e is not None and not released:
                e['committed'] = True
        want_symbols = m.fix()
        checked += 1
        if want_symbols != symbols:
            errors.append(f'{name} slot {slot}: {symbols} symbols, expected {want_symbols}')
        for e, (rnti, purpose, al, released, cce, sym, dci_al) in zip(made, asks[(name, slot)]):
            if e is None or released:
                continue
            checked += 1
            if (cce, sym, dci_al) != (e['cce'], want_symbols, e['al']):
                errors.append(f'{name} slot {slot} RNTI {rnti}: CCE {cce} symbols {sym} level '
                              f'{dci_al}, expected {e["cce"]} {want_symbols} {e["al"]}')
        # no two DCIs on one CCE
        seen = set()
        for e in m.entries:
            for i in range(e['al']):
                if e['cce'] + i in seen:
                    errors.append(f'{name} slot {slot}: CCE {e["cce"] + i} used twice')
                seen.add(e['cce'] + i)
    return models, errors, checked


def run(path):
    lines = open(path).read().splitlines()
    body = [l for l in lines if not l.startswith('STATS')]
    stats = [l for l in lines if l.startswith('STATS')]
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.log', delete=False) as tmp:
        tmp.write('\n'.join(body) + '\n')
    models, errors, checked = main(tmp.name)
    for line in stats:
        f = line.split()
        m = models[f[1]]
        m.finish()
        st = m.stats
        want = [st['slots']] + st['by_symbols'] + [st['avail'], st['used'], st['above'], st['raised'],
                                                   st['noreport'], st['unused'], st['capped']]
        for i in range(N_PURPOSES):
            want += [st['dci'][i], st['cce'][i], st['blocked'][i]]
        for i in range(5):
            want += [st['dci_al'][i], st['cce_al'][i]]
        got = [int(x) for x in f[2:]]
        checked += len(want)
        if got != want:
            errors.append(f'STATS {f[1]}:\n got  {got}\n want {want}')
        print(f"config {f[1]}: {st['slots']} slots, {sum(st['dci'])} DCIs, {st['used']} of "
              f"{st['avail']} CCEs, {sum(st['blocked'])} blocked, slots by symbols {st['by_symbols']}, "
              f"levels {st['dci_al']}")
    print(f'{checked} values checked, {len(errors)} differences')
    for e in errors[:40]:
        print('  ' + e)
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(run(sys.argv[1]))
