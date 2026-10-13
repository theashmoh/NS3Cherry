#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
"""Hybrid NTN channel: orbit geometry from ns-3, local scattering and blockage from Sionna RT.

Sionna RT has no far-field source, so a satellite 500 km away cannot be placed in a scene that
spans a kilometre. This walks a proxy transmitter along the true line of sight instead: for each
sample of the orbit it sits at a fixed radius from the terminal, in the direction the satellite
actually is, carrying the satellite's true velocity so the Doppler is right. Sionna then traces
the last few kilometres over real building geometry, and the large-scale terms it cannot know are
put back afterwards.

    Sionna supplies   angular structure, blockage, local multipath, Doppler
    the orbit supplies   absolute path loss, absolute delay

Delays come back normalized (Sionna's own option), so the true slant-range delay is added rather
than corrected. Gains come back for the proxy distance, so each path is rescaled by the ratio of
the true free-space loss to the proxy's, which leaves the relative structure alone.

    ./ntn-sionna-cir.py --geom geom.csv --run-log run.log --out hybrid

Reads the geometry trace an ns-3 LEO example writes with --traceFile. See ntn/README.md, example 2.
"""

import argparse
import json
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

C = 299792458.0


def import_render_helpers():
    """Reuse the trace parsing and east-north-up maths from the pass renderer."""
    import importlib.util

    path = pathlib.Path(__file__).resolve().parent / "ntn-rt-render.py"
    spec = importlib.util.spec_from_file_location("ntn_rt_render", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def free_space_db(distance_m, frequency_hz):
    """Free-space path loss in dB. Used only as a ratio, so the constant cancels."""
    return 20.0 * math.log10(max(distance_m, 1.0)) + 20.0 * math.log10(frequency_hz) - 147.55


def path_geometry(paths, a, keep):
    """The strongest `keep` paths: interaction points from the transmitter side to the receiver
    side, in scene coordinates, with Sionna's interaction codes (1 specular, 2 diffuse,
    4 refraction, 8 diffraction) and relative power. A direct path has no points."""
    import numpy as np

    valid = np.asarray(paths.valid).reshape(-1)
    if not valid.any():
        return []  # nothing reaches the terminal: an outage sample
    codes = np.asarray(paths.interactions)
    codes = codes.reshape(codes.shape[0], -1)
    points = np.asarray(paths.vertices)
    points = points.reshape(points.shape[0], -1, 3)
    power = np.abs(np.asarray(a)) ** 2
    power = power.reshape(-1, power.shape[-2], power.shape[-1]).sum(axis=(0, 2))
    out = []
    for k in np.flatnonzero(valid):
        hops = [(int(c), [round(float(x), 2) for x in p]) for c, p in zip(codes[:, k], points[:, k])
                if c != 0]
        out.append({"codes": [c for c, _ in hops], "points": [p for _, p in hops],
                    "power": float(power[k]) if k < len(power) else 0.0})
    out.sort(key=lambda r: -r["power"])
    top = out[0]["power"] if out and out[0]["power"] > 0 else 1.0
    for r in out:
        r["rel_db"] = round(10.0 * math.log10(r["power"] / top), 1) if r["power"] > 0 else -300.0
        del r["power"]
    return out[:keep]

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--geom", required=True, help="geometry trace from --traceFile")
    ap.add_argument("--run-log", help="example stdout, for the terminal ECEF position")
    ap.add_argument("--observer", help="terminal ECEF 'x:y:z', overrides --run-log")
    ap.add_argument("--scene", default="munich",
                    help="Sionna scene name (munich, san_francisco, etoile, florence, ...) "
                         "or a path to a Mitsuba XML scene")
    ap.add_argument("--terminal", default="0:-150:1.5",
                    help="terminal position inside the scene, 'x:y:z' in metres. It must stand on "
                         "open ground: the scene origin of munich is inside a building, and a "
                         "terminal there sees no sky at any elevation. See ntn/SIONNA.md for how to find "
                         "a spot.")
    ap.add_argument("--proxy-km", type=float, default=10.0,
                    help="proxy transmitter radius; larger is more plane-wave-like")
    ap.add_argument("--frequency", type=float, default=12e9)
    ap.add_argument("--max-depth", type=int, default=3)
    ap.add_argument("--samples-per-src", type=int, default=1000000)
    ap.add_argument("--stride", type=int, default=1, help="use every Nth geometry sample")
    ap.add_argument("--limit", type=int, default=0, help="stop after N samples, 0 = all")
    ap.add_argument("--out", default="hybrid", help="output directory")
    ap.add_argument("--render", action="store_true", help="also write a scene image per sample")
    ap.add_argument("--save-paths", type=int, default=0, metavar="N",
                    help="store the N strongest paths of each sample in hybrid-cir.json: their interaction points in scene coordinates and interaction types, for ntn-rt-render.py --paths")
    ap.add_argument("--los-only", action="store_true",
                    help="only ask whether the direct ray reaches the terminal: reflections and "
                         "diffraction off, depth 0. Fast, and the answer is geometry rather than "
                         "link budget, so it is meaningful even where the link is down. Also "
                         "writes los.csv for ntn-rt-render.py --los.")
    args = ap.parse_args()

    helpers = import_render_helpers()
    geometry = helpers.read_geometry(args.geom)
    if not geometry:
        sys.exit(f"no geometry samples in {args.geom}")

    if args.observer:
        observer = tuple(float(v) for v in args.observer.split(":"))
    elif args.run_log:
        observer = helpers.read_observer(args.run_log)
        if observer is None:
            sys.exit(f"terminal position not found in {args.run_log}")
    else:
        sys.exit("need --observer or --run-log")

    geometry = geometry[:: max(1, args.stride)]
    if args.limit:
        geometry = geometry[: args.limit]

    import sionna.rt as rt

    scene_arg = args.scene
    if not scene_arg.endswith(".xml"):
        if not hasattr(rt.scene, scene_arg):
            sys.exit(f"unknown scene {scene_arg!r}")
        scene_arg = getattr(rt.scene, scene_arg)
    scene = rt.load_scene(scene_arg)
    scene.frequency = args.frequency
    # One isotropic element each end: the point here is the propagation, not the antenna.
    scene.tx_array = rt.PlanarArray(num_rows=1, num_cols=1, pattern="iso", polarization="V")
    scene.rx_array = rt.PlanarArray(num_rows=1, num_cols=1, pattern="iso", polarization="V")

    terminal = [float(v) for v in args.terminal.split(":")]
    scene.add(rt.Receiver(name="rx", position=terminal))

    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    solver = rt.PathSolver()
    radius = args.proxy_km * 1000.0

    # The satellite's velocity, differenced from consecutive ECEF samples and rotated into the
    # scene's local frame. Sionna applies it as the transmitter's Doppler, which is the one
    # large-scale term the proxy can carry itself: position is wrong, motion is right.
    east, north, up = helpers.enu_basis(observer)

    def to_enu(vec):
        return (
            sum(a * b for a, b in zip(vec, east)),
            sum(a * b for a, b in zip(vec, north)),
            sum(a * b for a, b in zip(vec, up)),
        )

    records = []
    camera = rt.Camera(position=[-400, -400, 400], look_at=terminal)

    for i, (t, sat_ecef, elev_trace, slant) in enumerate(geometry):
        direction, azimuth, elevation = helpers.look_direction(observer, sat_ecef)
        proxy = [terminal[k] + radius * direction[k] for k in range(3)]

        if i + 1 < len(geometry):
            nxt = geometry[i + 1]
            dt = nxt[0] - t
            vel_ecef = [(nxt[1][k] - sat_ecef[k]) / dt for k in range(3)] if dt > 0 else [0.0] * 3
        else:
            vel_ecef = [0.0] * 3
        velocity = list(to_enu(vel_ecef))

        if "tx" in scene.transmitters:
            scene.remove("tx")
        scene.add(rt.Transmitter(name="tx", position=proxy, velocity=velocity))

        paths = solver(
            scene,
            max_depth=0 if args.los_only else args.max_depth,
            samples_per_src=args.samples_per_src,
            los=True,
            specular_reflection=not args.los_only,
            diffraction=not args.los_only,
            # Sionna traces transmission through walls by default, so without this an obstructed
            # direct ray survives, 40 dB down, and every sample reads as line of sight.
            refraction=not args.los_only,
        )
        a, tau = paths.cir(normalize_delays=True, out_type="numpy")

        # Rescale from the proxy distance to the true slant range. The ratio is a single scalar,
        # so the relative structure Sionna computed, which is the whole point, is untouched.
        correction_db = free_space_db(radius, args.frequency) - free_space_db(slant, args.frequency)
        amplitude = 10.0 ** (correction_db / 20.0)
        a = a * amplitude
        true_delay = slant / C

        mags = abs(a).flatten()
        valid = [(m, float(d)) for m, d in zip(mags, tau.flatten()) if m > 0 and d >= 0]
        total = sum(m * m for m, _ in valid)
        if valid and total > 0:
            mean_tau = sum(m * m * d for m, d in valid) / total
            spread = math.sqrt(max(0.0, sum(m * m * (d - mean_tau) ** 2 for m, d in valid) / total))
        else:
            spread = 0.0

        record = {
            "t": t,
            "elevation_deg": elevation,
            "azimuth_deg": azimuth,
            "slant_m": slant,
            "num_paths": len(valid),
            "gain_db": 10.0 * math.log10(total) if total > 0 else float("-inf"),
            "delay_spread_s": spread,
            "absolute_delay_s": true_delay,
            "blocked": len(valid) == 0,
            "los": len(valid) > 0,
        }
        if args.save_paths and not args.los_only:
            record["paths"] = path_geometry(paths, a, args.save_paths)
        records.append(record)
        print(
            f"  t={t:7.2f}s elev={elevation:5.1f} az={azimuth:6.1f} "
            f"paths={record['num_paths']:3d} gain={record['gain_db']:8.2f} dB "
            f"spread={spread * 1e9:7.1f} ns" + ("  BLOCKED" if record["blocked"] else ""),
            flush=True,
        )

        if args.render:
            scene.render_to_file(
                camera=camera,
                filename=str(out / f"frame-{i}.png"),
                paths=paths,
                resolution=[900, 700],
            )

    (out / "hybrid-cir.json").write_text(json.dumps(records, indent=1))
    if args.los_only:
        with (out / "los.csv").open("w") as f:
            f.write("t_s,elevation_deg,los\n")
            for r in records:
                f.write(f"{r['t']:.3f},{r['elevation_deg']:.4f},{int(r['los'])}\n")
        print(f"Wrote {out / 'los.csv'}.")
    blocked = sum(1 for r in records if r["blocked"])
    print(
        f"\n{len(records)} samples over {records[-1]['t'] - records[0]['t']:.1f} s, "
        f"elevation {min(r['elevation_deg'] for r in records):.1f} to "
        f"{max(r['elevation_deg'] for r in records):.1f} deg, "
        f"{blocked} blocked ({100 * blocked / len(records):.0f}%). "
        f"Wrote {out / 'hybrid-cir.json'}."
    )


if __name__ == "__main__":
    main()
