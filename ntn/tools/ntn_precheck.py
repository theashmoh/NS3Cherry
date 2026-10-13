#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
# Author: Jesse Chiu (Kuan-Po) <kchiu5@uw.edu>, University of Washington, Seattle
"""Sanity-check an NTN link configuration before spending hours simulating it.

Closes the link budget, works out the propagation delays, and gives a first-order
throughput ceiling. It is an arithmetic check, not a simulation: no fading, no scheduler,
no protocol overhead. Use it to catch configurations that cannot possibly work -- an EIRP
45 dB too low, a HARQ loop longer than the pipeline -- before ns-3 spends an afternoon
telling you the same thing.

  python3 ntn/tools/ntn_precheck.py --preset regen
  python3 ntn/tools/ntn_precheck.py --eirp-dbw-mhz 15 --bw-mhz 100 --elev 30 --rx-gain 30
  python3 ntn/tools/ntn_precheck.py --selftest
"""
import argparse
import math

C = 299_792_458.0          # m/s
K_DBW = -228.6             # Boltzmann, dBW/K/Hz
RE_KM = 6371.0             # mean Earth radius

# TS 38.214 table 5.1.3.1-2 endpoints: MCS 0 and MCS 28 spectral efficiency, bit/s/Hz.
SE_MIN, SE_MAX = 0.2344, 7.4063
# ~1.5 dB from Shannon is typical for a well-implemented NR link.
IMPL_LOSS_DB = 1.5


def slant_range_km(alt_km: float, elev_deg: float) -> float:
    """Slant range to a satellite at `alt_km` seen at `elev_deg` above the horizon."""
    e = math.radians(elev_deg)
    return math.sqrt((RE_KM + alt_km) ** 2 - (RE_KM * math.cos(e)) ** 2) - RE_KM * math.sin(e)


def fspl_db(range_km: float, freq_hz: float) -> float:
    return 20 * math.log10(4 * math.pi * range_km * 1e3 * freq_hz / C)


def link_budget(eirp_dbw_mhz, bw_mhz, freq_hz, alt_km, elev_deg, rx_gain_dbi,
                noise_fig_db, atm_loss_db, temp_k=290.0):
    """Return the pieces of a one-hop downlink budget, all in dB unless named otherwise."""
    rng = slant_range_km(alt_km, elev_deg)
    eirp_dbw = eirp_dbw_mhz + 10 * math.log10(bw_mhz)          # total over the carrier
    path = fspl_db(rng, freq_hz)
    noise_dbw = K_DBW + 10 * math.log10(temp_k) + 10 * math.log10(bw_mhz * 1e6) + noise_fig_db
    rx_dbw = eirp_dbw - path - atm_loss_db + rx_gain_dbi
    return {
        "slant_km": rng,
        "eirp_dbw": eirp_dbw,
        "fspl_db": path,
        "atm_db": atm_loss_db,
        "rx_gain_dbi": rx_gain_dbi,
        "rx_dbw": rx_dbw,
        "noise_dbw": noise_dbw,
        "snr_db": rx_dbw - noise_dbw,
    }


def throughput_mbps(snr_db: float, bw_mhz: float) -> tuple[float, float, bool]:
    """Shannon with an implementation margin, clamped to the MCS table. (Mbps, SE, capped)."""
    se = math.log2(1 + 10 ** ((snr_db - IMPL_LOSS_DB) / 10))
    capped = se > SE_MAX
    se = min(max(se, 0.0), SE_MAX)
    return se * bw_mhz, se, capped


def delays_ms(alt_km: float, elev_deg: float, hops: int, k1_slots: int, slot_ms: float,
              proc_ms: float):
    """One-way, round-trip and HARQ loop delay. hops=1 regenerative, 2 bent-pipe."""
    one_way = slant_range_km(alt_km, elev_deg) * 1e3 / C * 1e3 * hops
    rtt = 2 * one_way
    return {"one_way_ms": one_way, "rtt_ms": rtt,
            "harq_loop_ms": rtt + k1_slots * slot_ms + proc_ms}


PRESETS = {
    # Matches examples 6 and 7 in ntn/README.md.
    "regen":    dict(eirp_dbw_mhz=11.4, bw_mhz=100.0, freq_hz=12e9, alt_km=600.0,
                     rx_gain_dbi=30.0, noise_fig_db=1.2, atm_loss_db=0.5, hops=1),
    "bentpipe": dict(eirp_dbw_mhz=20.0, bw_mhz=100.0, freq_hz=2e9, alt_km=600.0,
                     rx_gain_dbi=0.0, noise_fig_db=7.0, atm_loss_db=0.5, hops=2),
}


def report(cfg, elevs, k1_slots, slot_ms, proc_ms, harq_procs):
    print(f"  carrier {cfg['freq_hz']/1e9:g} GHz   bandwidth {cfg['bw_mhz']:g} MHz   "
          f"altitude {cfg['alt_km']:g} km   {cfg['hops']} hop(s)")
    print(f"  EIRP {cfg['eirp_dbw_mhz']:g} dBW/MHz   Rx gain {cfg['rx_gain_dbi']:g} dBi   "
          f"NF {cfg['noise_fig_db']:g} dB\n")
    print(f"  {'elev':>5} {'slant':>8} {'FSPL':>8} {'SNR':>7} {'SE':>6} {'rate':>9} "
          f"{'RTT':>8} {'HARQ':>8}")
    print(f"  {'deg':>5} {'km':>8} {'dB':>8} {'dB':>7} {'b/s/Hz':>6} {'Mbps':>9} "
          f"{'ms':>8} {'loop ms':>8}")
    pipeline_ms = harq_procs * slot_ms
    warn = []
    for e in elevs:
        b = link_budget(cfg["eirp_dbw_mhz"], cfg["bw_mhz"], cfg["freq_hz"], cfg["alt_km"],
                        e, cfg["rx_gain_dbi"], cfg["noise_fig_db"], cfg["atm_loss_db"])
        d = delays_ms(cfg["alt_km"], e, cfg["hops"], k1_slots, slot_ms, proc_ms)
        mbps, se, capped = throughput_mbps(b["snr_db"], cfg["bw_mhz"])
        flag = ""
        if b["snr_db"] < -6.7:                      # below MCS 0 -- no link at all
            flag = "  << below MCS0"; warn.append(f"{e:g} deg: SNR {b['snr_db']:.1f} dB is below the MCS 0 threshold")
        elif capped:
            flag = "  << MCS28 capped"
        if d["harq_loop_ms"] > pipeline_ms:
            flag += "  << loop > pipeline"
            warn.append(f"{e:g} deg: HARQ loop {d['harq_loop_ms']:.1f} ms exceeds the "
                        f"{harq_procs}-process pipeline ({pipeline_ms:.0f} ms)")
        print(f"  {e:5g} {b['slant_km']:8.0f} {b['fspl_db']:8.1f} {b['snr_db']:7.1f} "
              f"{se:6.2f} {mbps:9.1f} {d['rtt_ms']:8.1f} {d['harq_loop_ms']:8.1f}{flag}")
    print()
    if warn:
        print("  WARNINGS")
        for w in warn:
            print(f"    - {w}")
    else:
        print("  No warnings: link closes and the HARQ loop fits the pipeline at every elevation.")
    print("\n  First-order only: Shannon less 1.5 dB, clamped to MCS 0..28. No fading, no\n"
          "  scheduler, no protocol overhead. Expect the simulator to come in below this.")


def selftest():
    # Geometry: straight up is the altitude; at the horizon it is the tangent length.
    assert abs(slant_range_km(600, 90) - 600) < 1e-6
    horizon = math.sqrt((RE_KM + 600) ** 2 - RE_KM ** 2)
    assert abs(slant_range_km(600, 0) - horizon) < 1e-6
    assert slant_range_km(600, 30) > slant_range_km(600, 60) > slant_range_km(600, 90)
    # FSPL: doubling the range adds 6 dB; doubling the frequency adds 6 dB.
    assert abs(fspl_db(2000, 12e9) - fspl_db(1000, 12e9) - 6.02) < 0.01
    assert abs(fspl_db(1000, 24e9) - fspl_db(1000, 12e9) - 6.02) < 0.01
    # Throughput clamps at both ends of the MCS table.
    assert throughput_mbps(60, 100)[2] is True and throughput_mbps(60, 100)[1] == SE_MAX
    assert throughput_mbps(-30, 100)[1] < 0.01
    # Delay: a two-hop path is twice a one-hop path; RTT is twice one-way.
    d1 = delays_ms(600, 90, 1, 2, 1.0, 0.0)
    d2 = delays_ms(600, 90, 2, 2, 1.0, 0.0)
    assert abs(d2["one_way_ms"] - 2 * d1["one_way_ms"]) < 1e-9
    assert abs(d1["rtt_ms"] - 2 * d1["one_way_ms"]) < 1e-9
    # 600 km straight up is about 2 ms one way, 4 ms there and back.
    assert 1.9 < d1["one_way_ms"] < 2.1 and 3.9 < d1["rtt_ms"] < 4.1
    # The regen preset must close the link at 30 deg; that is the demo's worst case.
    b = link_budget(**{k: v for k, v in PRESETS["regen"].items() if k != "hops"}, elev_deg=30)
    assert b["snr_db"] > 0, b
    print("selftest OK")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--preset", choices=sorted(PRESETS), help="start from a known setup")
    ap.add_argument("--eirp-dbw-mhz", type=float)
    ap.add_argument("--bw-mhz", type=float)
    ap.add_argument("--freq-ghz", type=float)
    ap.add_argument("--alt-km", type=float)
    ap.add_argument("--rx-gain", type=float, dest="rx_gain_dbi")
    ap.add_argument("--noise-fig", type=float, dest="noise_fig_db")
    ap.add_argument("--atm-loss", type=float, dest="atm_loss_db")
    ap.add_argument("--hops", type=int, choices=(1, 2))
    ap.add_argument("--elev", type=float, nargs="+", default=[10, 30, 60, 90])
    ap.add_argument("--k1", type=int, default=2, help="HARQ feedback delay, slots")
    ap.add_argument("--slot-ms", type=float, default=1.0, help="slot length (1.0 at mu=0)")
    ap.add_argument("--proc-ms", type=float, default=0.0, help="extra processing delay")
    ap.add_argument("--harq-procs", type=int, default=32)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        selftest(); return

    cfg = dict(PRESETS[a.preset]) if a.preset else dict(
        eirp_dbw_mhz=11.4, bw_mhz=100.0, freq_hz=12e9, alt_km=600.0,
        rx_gain_dbi=30.0, noise_fig_db=1.2, atm_loss_db=0.5, hops=1)
    for name, val in (("eirp_dbw_mhz", a.eirp_dbw_mhz), ("bw_mhz", a.bw_mhz),
                      ("alt_km", a.alt_km), ("rx_gain_dbi", a.rx_gain_dbi),
                      ("noise_fig_db", a.noise_fig_db), ("atm_loss_db", a.atm_loss_db),
                      ("hops", a.hops)):
        if val is not None:
            cfg[name] = val
    if a.freq_ghz is not None:
        cfg["freq_hz"] = a.freq_ghz * 1e9

    print(f"\nNTN pre-check{f'  [preset: {a.preset}]' if a.preset else ''}\n")
    report(cfg, a.elev, a.k1, a.slot_ms, a.proc_ms, a.harq_procs)


if __name__ == "__main__":
    main()
