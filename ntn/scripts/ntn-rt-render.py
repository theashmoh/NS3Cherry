#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
"""Animate a LEO pass: the line-of-sight ray sweeping over the terminal, against the link.

Reads the geometry trace the LEO examples write with --traceFile and the NR packet trace they
write alongside it, and produces one video: on the left the ground and the line-of-sight ray
swinging across the sky as the satellite rises and sets, on the right SNR, modulation and
throughput with a cursor on the current instant.

The satellite itself is never drawn. At 500 km it is a distant point far outside any sensible
ground-scale view, so the ray, its elevation and the slant range carry the geometry honestly and
a marker would only invite a wrong reading of scale.

    ./ntn-rt-render.py --dir out --out pass.mp4

See ntn/README.md, example 2, for the run that produces the inputs.
"""

import argparse
import math
import pathlib
import re
import subprocess
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.image import imread  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

TIME_RE = re.compile(r"^\+?([0-9.eE+-]+)ns$")
OBSERVER_RE = re.compile(r"Ground Node GetPosition\(\):\s*([-0-9.eE+]+):([-0-9.eE+]+):([-0-9.eE+]+)")


def parse_ns_time(token):
    """'+1e+08ns' -> 0.1 (seconds). The trace uses ns-3's Time stream format."""
    m = TIME_RE.match(token.strip())
    if not m:
        raise ValueError(f"unparsable time {token!r}")
    return float(m.group(1)) / 1e9


def read_observer(path):
    """Pull the ground terminal's ECEF position out of the example's stdout log."""
    for line in pathlib.Path(path).read_text(errors="replace").splitlines():
        m = OBSERVER_RE.search(line)
        if m:
            return tuple(float(g) for g in m.groups())
    return None


def frame_paths(directory, prefix):
    """Numbered image files, ordered by the counter in the name rather than lexically."""
    pattern = re.compile(re.escape(prefix) + r"(\d+)\.png$")
    found = []
    for p in pathlib.Path(directory).iterdir():
        m = pattern.match(p.name)
        if m:
            found.append((int(m.group(1)), p))
    return [p for _, p in sorted(found)]


def read_geometry(path):
    """[(t, satellite ECEF, elevation_deg, slant_m)] from the --traceFile trace."""
    rows = []
    for line in pathlib.Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("Time:"):
            continue  # header
        f = line.split(":")
        if len(f) < 10:
            continue
        try:
            rows.append(
                (
                    parse_ns_time(f[0]),
                    (float(f[2]), float(f[3]), float(f[4])),
                    float(f[8]),
                    float(f[9]),
                )
            )
        except ValueError:
            continue
    rows.sort(key=lambda r: r[0])
    return rows


def enu_basis(observer):
    """Local east/north/up unit vectors at an ECEF point on a spherical Earth."""
    ox, oy, oz = observer
    r = math.sqrt(ox * ox + oy * oy + oz * oz)
    if r == 0.0:
        raise ValueError("observer at the centre of the Earth")
    up = (ox / r, oy / r, oz / r)
    # East is perpendicular to both the polar axis and up. Degenerate exactly at the poles.
    ex, ey = -oy, ox
    en = math.hypot(ex, ey)
    east = (ex / en, ey / en, 0.0) if en > 0 else (1.0, 0.0, 0.0)
    north = (
        up[1] * east[2] - up[2] * east[1],
        up[2] * east[0] - up[0] * east[2],
        up[0] * east[1] - up[1] * east[0],
    )
    return east, north, up


def look_direction(observer, satellite):
    """Unit vector to the satellite in local east-north-up, plus azimuth and elevation."""
    east, north, up = enu_basis(observer)
    d = [s - o for s, o in zip(satellite, observer)]
    e = sum(a * b for a, b in zip(d, east))
    n = sum(a * b for a, b in zip(d, north))
    u = sum(a * b for a, b in zip(d, up))
    norm = math.sqrt(e * e + n * n + u * u)
    if norm == 0.0:
        return (0.0, 0.0, 1.0), 0.0, 90.0
    e, n, u = e / norm, n / norm, u / norm
    azimuth = (math.degrees(math.atan2(e, n))) % 360.0
    elevation = math.degrees(math.asin(max(-1.0, min(1.0, u))))
    return (e, n, u), azimuth, elevation


def read_packet_trace(path):
    """[(t, tbSize_bytes, mcs, sinr_db, corrupt)] for the downlink rows."""
    rows = []
    p = pathlib.Path(path)
    if not p.exists():
        return rows
    with p.open() as fh:
        header = fh.readline().split()
        try:
            col = {name: header.index(name) for name in ("Time", "tbSize", "mcs", "SINR(dB)")}
        except ValueError:
            sys.exit(f"{path}: unexpected columns {header}")
        corrupt_col = header.index("corrupt") if "corrupt" in header else None
        dir_col = header.index("direction") if "direction" in header else None
        for line in fh:
            f = line.split()
            if len(f) <= max(col.values()):
                continue
            if dir_col is not None and f[dir_col] != "DL":
                continue
            try:
                rows.append(
                    (
                        float(f[col["Time"]]),
                        float(f[col["tbSize"]]),
                        float(f[col["mcs"]]),
                        float(f[col["SINR(dB)"]]),
                        int(float(f[corrupt_col])) if corrupt_col is not None else 0,
                    )
                )
            except ValueError:
                continue
    rows.sort(key=lambda r: r[0])
    return rows


def read_los(path):
    """(time, is_line_of_sight) pairs from ntn-sionna-cir.py --los-only, CSV or JSON."""
    path = pathlib.Path(path)
    if path.suffix == ".json":
        import json
        return [(float(r["t"]), bool(r["los"])) for r in json.loads(path.read_text())]
    rows = []
    for n, line in enumerate(path.read_text().splitlines()):
        if n == 0 or not line.strip():
            continue
        f = line.split(",")
        rows.append((float(f[0]), f[2].strip() == "1"))
    return rows


def nearest_flags(frame_times, los_rows):
    """Carry the line-of-sight flag onto the frame grid; the two are sampled independently."""
    flags, j = [], 0
    for t in frame_times:
        while j + 1 < len(los_rows) and abs(los_rows[j + 1][0] - t) <= abs(los_rows[j][0] - t):
            j += 1
        flags.append(los_rows[j][1] if los_rows else True)
    return flags


def break_gaps(ts, vs, max_gap):
    """Insert a NaN across every jump, so no line is drawn over a stretch that has no data.

    Without this the per-block overlay joins the last block before a dropout to the first one
    after it, and a two-minute outage reads as a straight diagonal of real measurements.
    """
    ot, ov = [], []
    for i, (t, v) in enumerate(zip(ts, vs)):
        if i and t - ts[i - 1] > max_gap:
            ot.append(ts[i - 1] + max_gap / 2)
            ov.append(math.nan)
        ot.append(t)
        ov.append(v)
    return ot, ov


def bin_metrics(packets, frame_times, window):
    """Per frame: mean SNR, mean modulation, and throughput over the preceding window.

    Throughput counts only transport blocks that decoded, so it is delivered rate rather than
    attempted rate. SNR and modulation average every block, including the failures, because a
    frame where everything failed still has a meaningful SNR and a meaningful chosen modulation.
    """
    snr, mcs, mbps = [], [], []
    i = 0
    for t in frame_times:
        lo = t - window
        while i < len(packets) and packets[i][0] < lo:
            i += 1
        j, s, m, good_bytes, n = i, 0.0, 0.0, 0.0, 0
        while j < len(packets) and packets[j][0] <= t:
            _, tb, mc, si, corrupt = packets[j]
            s += si
            m += mc
            n += 1
            if not corrupt:
                good_bytes += tb
            j += 1
        snr.append(s / n if n else math.nan)
        mcs.append(m / n if n else math.nan)
        # NaN, not zero, when nothing arrived at all: a solid zero line over a dropout reads as
        # a measured rate and hides that the other two panels have no data there. Zero is kept
        # for the case that matters, blocks that arrived and none of them decoded.
        mbps.append(good_bytes * 8.0 / window / 1e6 if (n and window > 0) else math.nan)
    return snr, mcs, mbps


def load_buildings(path, observer, extent_m):
    """Building prisms from a GeoJSON of footprints, in metres east/north/up of the terminal.

    Footprints are (lon, lat) rings with a height_m property. The terminal's latitude and
    longitude come from its ECEF position on the same spherical Earth the examples use, and the
    few hundred metres drawn here are small enough for a flat east/north projection. Footprints
    are clipped to the square of half-width extent_m. Returns (roofs, walls, attribution).
    """
    import json

    gj = json.loads(pathlib.Path(path).read_text())
    ox, oy, oz = observer
    r = math.sqrt(ox * ox + oy * oy + oz * oz)
    lat0, lon0 = math.asin(oz / r), math.atan2(oy, ox)
    clip = lambda v: max(-extent_m, min(extent_m, v))  # noqa: E731
    roofs, walls = [], []
    for feat in gj.get("features", []):
        geom = feat.get("geometry") or {}
        if geom.get("type") != "Polygon" or not geom.get("coordinates"):
            continue
        ring = [
            ((math.radians(lon) - lon0) * r * math.cos(lat0), (math.radians(lat) - lat0) * r)
            for lon, lat in geom["coordinates"][0]
        ]
        if max(abs(v) for p in ring for v in p) > 1.5 * extent_m:
            continue  # wholly or mostly outside the window
        ring = [(clip(x), clip(y)) for x, y in ring]
        h = float(feat.get("properties", {}).get("height_m", 10.0))
        roofs.append([(x, y, h) for x, y in ring])
        walls.extend(
            [(x1, y1, 0.0), (x2, y2, 0.0), (x2, y2, h), (x1, y1, h)]
            for (x1, y1), (x2, y2) in zip(ring, ring[1:])
        )
    return roofs, walls, gj.get("attribution", "")


PATH_STYLE = (  # Sionna interaction codes -> how a traced path is drawn, strongest mechanism first
    (8, "#8172b2", "diffracted"),
    (1, "#dd8452", "reflected"),
    (4, "#8c8c8c", "through a wall"),
    (2, "#937860", "scattered"),
)


def draw_paths(ax, traced, sat_dir, ray_len, proxy_m):
    """The traced multipath of one frame: from the satellite's direction through each path's
    interaction points to the terminal. Colour by mechanism, fainter for weaker paths; a direct
    path has no points and is the red line of sight instead."""
    drawn = set()
    proxy = [c * proxy_m for c in sat_dir]
    for p in traced or []:
        pts = p.get("points") or []
        if not pts or p.get("rel_db", 0.0) < -40.0:
            continue
        colour, name = next(((c, n) for code, c, n in PATH_STYLE if code in p["codes"]),
                            ("#8c8c8c", "other"))
        alpha = max(0.25, min(1.0, 1.0 + p.get("rel_db", 0.0) / 40.0))
        # incoming leg: from the first interaction back toward the satellite, drawn to scene scale
        x0 = pts[0]
        d = [a - b for a, b in zip(proxy, x0)]
        n = math.sqrt(sum(c * c for c in d)) or 1.0
        far = [x0[k] + 0.6 * ray_len * d[k] / n for k in range(3)]
        chain = [far] + pts + [[0.0, 0.0, 0.0]]
        ax.plot([q[0] for q in chain], [q[1] for q in chain], [q[2] for q in chain],
                color=colour, linewidth=1.3, alpha=alpha, zorder=9,
                label=None if name in drawn else name)
        ax.scatter([q[0] for q in pts], [q[1] for q in pts], [q[2] for q in pts],
                   color=colour, s=10, alpha=alpha, depthshade=False, zorder=9)
        drawn.add(name)
    return drawn


def draw_scene(ax, directions, idx, ray_len, view_elev, view_azim, buildings=None, label="terminal",
               los=True, traced=None, proxy_m=10000.0):
    """Ground plane, terminal, the line-of-sight ray (only while it is unblocked), and the arc
    the satellite has swept."""
    ax.clear()
    # mplot3d depth-sorts filled polygons but paints lines in insertion order, so the ground
    # and the buildings were drawn over the ray and it looked buried. Layer explicitly: the
    # ray is an indicator of direction and must stay on top.
    ax.computed_zorder = False
    g = ray_len * 0.75
    ax.add_collection3d(
        Poly3DCollection(
            [[(-g, -g, 0), (g, -g, 0), (g, g, 0), (-g, g, 0)]],
            facecolor="#d9d9d9",
            edgecolor="#9a9a9a",
            alpha=0.55,
            zorder=1,
        )
    )
    if buildings:
        roofs, walls = buildings
        ax.add_collection3d(Poly3DCollection(walls, facecolor="#b9a98f", edgecolor="#8c7b62",
                                             linewidth=0.25, alpha=0.95, zorder=2))
        ax.add_collection3d(Poly3DCollection(roofs, facecolor="#e6dccb", edgecolor="#8c7b62",
                                             linewidth=0.35, alpha=0.95, zorder=3))
    # The track the ray has swept so far, so a still frame still shows where the pass came from.
    if idx > 0:
        ax.plot(
            [d[0] * ray_len for d in directions[: idx + 1]],
            [d[1] * ray_len for d in directions[: idx + 1]],
            [d[2] * ray_len for d in directions[: idx + 1]],
            color="#b0b0b0",
            linewidth=1.0,
            linestyle=":",
            zorder=4,
        )
    drawn = draw_paths(ax, traced, directions[idx], ray_len, proxy_m) if traced is not None else set()
    # The red ray is the line of sight, so it is drawn only while there is one: behind a building
    # it disappears and only the dotted track of where the satellite has been remains.
    if los:
        e, n, u = directions[idx]
        ax.plot(
            [0, e * ray_len], [0, n * ray_len], [0, u * ray_len],
            color="#c44e52", linewidth=2.4, solid_capstyle="round", zorder=10, label="direct",
        )
        drawn.add("direct")
    if traced is not None and drawn:
        ax.legend(loc="upper left", fontsize=7, frameon=False)
    ax.scatter([0], [0], [0], color="#2f2f2f", s=45, depthshade=False, zorder=11)
    if not buildings:
        ax.text(0, 0, -0.08 * ray_len, label, color="#2f2f2f", fontsize=8, ha="center")

    ax.set_xlim(-g, g)
    ax.set_ylim(-g, g)
    ax.set_zlim(0, ray_len * (0.6 if buildings else 1.0))
    ax.set_xlabel("east")
    ax.set_ylabel("north")
    ax.set_zticks([])
    ax.set_xticks([])
    ax.set_yticks([])
    ax.view_init(elev=view_elev, azim=view_azim)
    ax.set_box_aspect((1, 1, 0.45 if buildings else 0.75))
    # The default panes draw grey walls around the sky, which reads as a room rather than outdoors.
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.pane.fill = False
        pane.pane.set_edgecolor((1.0, 1.0, 1.0, 0.0))
    ax.grid(False)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dir", default=".", help="directory holding the traces")
    ap.add_argument("--geom", default="geom.csv", help="--traceFile geometry trace")
    ap.add_argument("--trace", default="RxPacketTrace.txt", help="NR packet trace")
    ap.add_argument("--run-log", default="run.log", help="example stdout, for the terminal position")
    ap.add_argument("--observer", help="terminal ECEF 'x:y:z', overrides --run-log")
    ap.add_argument("--stride", type=int, default=1, help="use every Nth geometry sample")
    ap.add_argument("--out", default="ntn-pass.mp4")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--view-elev", type=float, default=14.0, help="camera height, low looks ground-level")
    ap.add_argument("--view-azim", type=float, default=-62.0)
    ap.add_argument("--scene-dir",
                    help="directory of Sionna scene images (from --sionnaRender). When given, the "
                         "left panel shows the ray-traced city instead of the bare ground plane.")
    ap.add_argument("--scene-prefix", default="scene-")
    ap.add_argument("--buildings",
                    help="GeoJSON of building footprints with a height_m property, drawn at true "
                         "scale around the terminal (e.g. ntn/data/uw-campus-buildings.geojson). "
                         "Drawn only: the channel is whatever the run simulated.")
    ap.add_argument("--extent-m", type=float, default=250.0,
                    help="half-width of the ground square drawn with --buildings, metres")
    ap.add_argument("--place", default="terminal", help="label for the terminal")
    ap.add_argument("--paths",
                    help="hybrid-cir.json from ntn-sionna-cir.py --save-paths: draw each frame's traced "
                         "reflected, diffracted and refracted paths in the scene")
    ap.add_argument("--proxy-km", type=float, default=10.0,
                    help="proxy radius the paths were traced with, for the incoming legs")
    ap.add_argument("--los",
                    help="line-of-sight table from ntn-sionna-cir.py --los-only (los.csv or "
                         "hybrid-cir.json). Adds an elevation panel with an x on every sample "
                         "whose direct ray the scene blocks.")
    args = ap.parse_args()

    d = pathlib.Path(args.dir)
    geometry = read_geometry(d / args.geom)
    if not geometry:
        sys.exit(f"no geometry samples in {d / args.geom}; was the run given --traceFile?")

    if args.observer:
        observer = tuple(float(v) for v in args.observer.split(":"))
    else:
        observer = read_observer(d / args.run_log) if (d / args.run_log).exists() else None
        if observer is None:
            sys.exit(
                f"terminal position not found in {d / args.run_log}; pass --observer x:y:z"
            )

    geometry = geometry[:: max(1, args.stride)]
    times = [g[0] for g in geometry]
    directions, azimuths, elevations = [], [], []
    for _, sat, elev_trace, _slant in geometry:
        vec, az, el = look_direction(observer, sat)
        directions.append(vec)
        azimuths.append(az)
        elevations.append(el)
        if abs(el - elev_trace) > 1.0:
            print(
                f"note: computed elevation {el:.2f} deg differs from the trace's "
                f"{elev_trace:.2f} deg; check --observer",
                file=sys.stderr,
            )
            break

    window = (times[1] - times[0]) if len(times) > 1 else 1.0
    packets = read_packet_trace(d / args.trace)
    if not packets:
        print(f"note: no downlink rows in {d / args.trace}, metric panels will be empty",
              file=sys.stderr)
    snr, mcs, mbps = bin_metrics(packets, times, window)
    # The per-block series underneath the bins. Averaging to the frame rate hides the fast fading
    # entirely: at Ku band with a LEO Doppler the slot-to-slot swing is tens of dB, and a plot of
    # the means alone reads as a far smoother link than the one actually simulated.
    raw_t = [p[0] for p in packets]
    raw_snr = [p[3] for p in packets]
    raw_mcs = [p[2] for p in packets]

    print(
        f"{len(times)} frames, t = {times[0]:.1f} to {times[-1]:.1f} s "
        f"({times[-1] - times[0]:.1f} s of contact), elevation {min(elevations):.1f} to "
        f"{max(elevations):.1f} deg, slant {min(g[3] for g in geometry) / 1e3:.0f} to "
        f"{max(g[3] for g in geometry) / 1e3:.0f} km"
    )

    scene_images = []
    if args.scene_dir:
        scene_images = frame_paths(pathlib.Path(args.scene_dir), args.scene_prefix)
        if not scene_images:
            sys.exit(f"no {args.scene_prefix}*.png in {args.scene_dir}")
        # The channel is computed once per bandwidth part, so there are usually several images
        # per geometry sample. Spread them evenly over the frames rather than assuming a ratio.
        print(f"{len(scene_images)} scene images for {len(times)} frames")

    frames_dir = d / "_frames"
    frames_dir.mkdir(exist_ok=True)
    gap = max(2.0 * window, 1.0)
    raw_t_snr, raw_snr = break_gaps(raw_t, raw_snr, gap)
    raw_t_mcs, raw_mcs = break_gaps(raw_t, raw_mcs, gap)
    # Elevation on top: it is the cause, and the three link panels below read against it. With
    # --los it also carries an x on every frame whose direct ray is blocked.
    flags = [True] * len(times)
    nlos = None
    if args.los:
        los_rows = read_los(d / args.los)
        flags = nearest_flags(times, los_rows)
        nlos = [(times[j], elevations[j]) for j, ok in enumerate(flags) if not ok]
        print(f"line of sight blocked on {len(nlos)}/{len(flags)} frames "
              f"({100 * len(nlos) / max(1, len(flags)):.0f}%)")
    series = [
        (times, elevations, None, "elevation (deg)", "#8172b2", nlos),
        (times, snr, (raw_t_snr, raw_snr), "SNR (dB)", "#4c72b0", None),
        (times, mcs, (raw_t_mcs, raw_mcs), "modulation index", "#55a868", None),
        (times, mbps, None, "throughput (Mbit/s)", "#c44e52", None),
    ]

    buildings, attribution = None, ""
    if args.buildings:
        roofs, walls, attribution = load_buildings(args.buildings, observer, args.extent_m)
        buildings = (roofs, walls)
        print(f"{len(roofs)} buildings within {args.extent_m:g} m of the terminal")
    ray_len = args.extent_m / 0.75 if buildings else 1.0
    traced_by_frame = None
    if args.paths:
        import json
        recs = json.loads((d / args.paths).read_text())
        traced_by_frame = [min(recs, key=lambda r: abs(r["t"] - t)).get("paths", []) for t in times]

    for i in range(len(times)):
        fig = plt.figure(figsize=(14, 1.85 * len(series)))
        grid = fig.add_gridspec(len(series), 2, width_ratios=[1.35, 1])
        if scene_images:
            ax_scene = fig.add_subplot(grid[:, 0])
            si = min(len(scene_images) - 1, i * len(scene_images) // max(1, len(times)))
            ax_scene.imshow(imread(scene_images[si]))
            ax_scene.axis("off")
            # The image keeps its own aspect, so it cannot fill a taller axes. Anchor it to the
            # bottom edge rather than letting it float in the middle; stretching it to fit would
            # misrepresent the geometry.
            ax_scene.set_anchor("S")
        else:
            ax_scene = fig.add_subplot(grid[:, 0], projection="3d")
            draw_scene(ax_scene, directions, i, ray_len, args.view_elev, args.view_azim,
                       buildings, args.place, los=flags[i],
                       traced=traced_by_frame[i] if traced_by_frame else None,
                       proxy_m=args.proxy_km * 1000.0)
            if attribution:
                fig.text(0.01, 0.01, "Buildings \N{COPYRIGHT SIGN} OpenStreetMap contributors, ODbL",
                         fontsize=7, color="#6f6f6f")
            ax_scene.set_anchor("S")
        where = f"{args.place}    " if args.place != "terminal" else ""
        ax_scene.set_title(
            f"{where}t = {times[i]:.1f} s    elevation {elevations[i]:.1f}"
            f"\N{DEGREE SIGN}    azimuth {azimuths[i]:.0f}"
            f"\N{DEGREE SIGN}    slant {geometry[i][3] / 1e3:.0f} km"
        )

        share = None
        for k, (xs, values, raw, label, colour, marks) in enumerate(series):
            ax = fig.add_subplot(grid[k, 1], sharex=share)
            share = share or ax
            if raw:
                # Every transport block, so the fading is visible, with the frame mean over it.
                ax.plot(raw[0], raw[1], linewidth=0.4, color=colour, alpha=0.25)
            ax.plot(xs, values, linewidth=1.4, color=colour)
            if marks:
                ax.plot([m[0] for m in marks], [m[1] for m in marks], "x", color="#c44e52",
                        markersize=5, linestyle="none", label="NLoS")
                ax.legend(loc="upper right", fontsize=7)
            if not math.isnan(values[i]):
                ax.plot(xs[i], values[i], "o", color=colour, markersize=7)
            ax.axvline(times[i], color="#888888", linewidth=0.8, alpha=0.6)
            ax.set_ylabel(label, fontsize=9)
            ax.grid(alpha=0.3)
            ax.tick_params(labelsize=8)
            if k < len(series) - 1:
                ax.tick_params(labelbottom=False)
            else:
                ax.set_xlabel("time (s)", fontsize=9)
            ax.set_xlim(times[0], times[-1])
            pool = [v for v in (raw[1] if raw else values) if not math.isnan(v)]
            if pool:
                lo, hi = min(pool), max(pool)
                pad = max(0.5, 0.06 * (hi - lo))
                ax.set_ylim(lo - pad, hi + pad)

        fig.tight_layout()
        fig.savefig(frames_dir / f"f-{i:05d}.png", dpi=85)
        plt.close(fig)

    out = pathlib.Path(args.out)
    if not out.is_absolute():
        out = d / out
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps),
           "-i", str(frames_dir / "f-%05d.png")]
    if out.suffix == ".gif":
        # One palette built from the whole clip, and frames scaled to a README-friendly width:
        # ffmpeg's default GIF palette bands the plots and full-width frames make the file huge.
        cmd += ["-vf", "scale=900:-1:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];"
                       "[b][p]paletteuse=dither=bayer:bayer_scale=3", str(out)]
    else:
        cmd += ["-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-pix_fmt", "yuv420p", str(out)]
    subprocess.run(cmd, check=True)
    print(f"wrote {out} ({len(times)} frames at {args.fps} fps)")


if __name__ == "__main__":
    main()
