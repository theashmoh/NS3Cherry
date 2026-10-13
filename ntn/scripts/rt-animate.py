#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
"""Combine Sionna RT scene renders with an ns-3 metric trace into one animation.

The Sionna RT channel model writes one PNG per channel update, numbered by a counter
(frame-0.png, frame-1.png, ...), and the example writes one metric per update to a trace
file. Neither carries a timestamp, so they are matched by index and turned into seconds
here, using the update period the run was given.

    ./rt-animate.py --dir out --update-ms 100 --out link.mp4

The inputs come from leo-link-regen-RT --sionnaRender; see ntn/SIONNA.md.
"""

import argparse
import pathlib
import re
import subprocess
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.image import imread  # noqa: E402


def read_trace(path):
    """Read a whitespace-separated trace, using its last numeric column as the metric."""
    values = []
    for line in pathlib.Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith(("#", "%")):
            continue
        parts = line.split()
        try:
            values.append(float(parts[-1]))
        except ValueError:
            continue  # a header line
    return values


def frame_paths(directory, prefix):
    """Every rendered frame, ordered by the counter in its name rather than lexically."""
    pattern = re.compile(re.escape(prefix) + r"(\d+)\.png$")
    found = []
    for p in pathlib.Path(directory).iterdir():
        m = pattern.match(p.name)
        if m:
            found.append((int(m.group(1)), p))
    return [p for _, p in sorted(found)]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dir", default=".", help="directory holding the frames and the trace")
    ap.add_argument("--prefix", default="frame-", help="frame file name prefix")
    ap.add_argument("--trace", default="snr-trace.txt", help="metric trace, relative to --dir")
    ap.add_argument("--update-ms", type=float, required=True, help="channel update period in ms")
    ap.add_argument("--label", default="SNR (dB)", help="y-axis label for the metric")
    ap.add_argument("--out", default="link.mp4", help="output video (.mp4 or .gif)")
    ap.add_argument("--fps", type=int, default=6)
    args = ap.parse_args()

    d = pathlib.Path(args.dir)
    frames = frame_paths(d, args.prefix)
    if not frames:
        sys.exit(f"no frames matching {args.prefix}*.png in {d}")
    values = read_trace(d / args.trace)
    if not values:
        sys.exit(f"no numeric samples in {d / args.trace}")

    n = min(len(frames), len(values))
    if len(frames) != len(values):
        print(
            f"note: {len(frames)} frames and {len(values)} samples, using the first {n}. "
            "They diverge when rendering and metric sampling run at different periods.",
            file=sys.stderr,
        )
    frames, values = frames[:n], values[:n]
    times = [i * args.update_ms / 1000.0 for i in range(n)]

    composed = d / "_composed"
    composed.mkdir(exist_ok=True)
    lo, hi = min(values), max(values)
    margin = max(0.5, 0.1 * (hi - lo))

    for i in range(n):
        fig, (ax_scene, ax_plot) = plt.subplots(
            1, 2, figsize=(14, 5), gridspec_kw={"width_ratios": [1.3, 1]}
        )
        ax_scene.imshow(imread(frames[i]))
        ax_scene.axis("off")
        ax_scene.set_title(f"Sionna RT scene and paths, t = {times[i]:.2f} s")

        ax_plot.plot(times, values, linewidth=1.2, color="#4c72b0")
        ax_plot.plot(times[i], values[i], "o", color="#c44e52", markersize=9)
        ax_plot.axvline(times[i], color="#c44e52", linewidth=0.8, alpha=0.5)
        ax_plot.set_xlim(times[0], times[-1] if times[-1] > times[0] else times[0] + 1)
        ax_plot.set_ylim(lo - margin, hi + margin)
        ax_plot.set_xlabel("time (s)")
        ax_plot.set_ylabel(args.label)
        ax_plot.set_title(f"{args.label} = {values[i]:.2f}")
        ax_plot.grid(alpha=0.3)

        fig.tight_layout()
        fig.savefig(composed / f"c-{i:05d}.png", dpi=90)
        plt.close(fig)

    out = d / args.out if not pathlib.Path(args.out).is_absolute() else pathlib.Path(args.out)
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps),
           "-i", str(composed / "c-%05d.png")]
    if out.suffix == ".gif":
        cmd += [str(out)]
    else:
        # yuv420p needs even dimensions, and is what players outside ffmpeg expect.
        cmd += ["-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-pix_fmt", "yuv420p", str(out)]
    subprocess.run(cmd, check=True)
    print(f"wrote {out} ({n} frames at {args.fps} fps)")


if __name__ == "__main__":
    main()
