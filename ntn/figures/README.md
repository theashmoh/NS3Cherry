# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only

# Figure provenance

Every image used by the documentation, and the exact command that produces it. Run all
commands from the ns-3-dev root.

| figure | produced by | lands as |
|---|---|---|
| `ntn-demo.png` | `summarize_ntn_demo.py results/ntn-demo` | `results/ntn-demo/ntn-demo.png`, copied here |
| `v1-tbler.png` | `plot_ntn_demo_figures.py` | `results/ntn-demo/v1-tbler.png`, copied here |
| `v2-delay-e30.png` | `plot_ntn_demo_figures.py` | `results/ntn-demo/v2-delay-e30.png`, copied here |
| `v2-pdcp.png` | `plot_v2_arq.py results/ntn-demo` | `results/ntn-demo/v2-arq.png`, copied and renamed |
| `bentpipe-pass.png` | `plot_bentpipe_pass_3panel.py results/<bent-pipe-run> --output ntn/figures/bentpipe-pass.png --min-elev 10` | written directly |
| `uw-pass.gif` | `ntn-rt-render.py`, command in README Example 2 | as given by its `--out` |
| `full-pass-4ue.png` | `analyze_footprint_per_ue.py results/<run> --arch regen`, on the run of README Example 3 | `fig_footprint_per_ue.png` in the output folder, copied and renamed |
| `beam-edge-sinr.png` | `plot_ntn_beam_edge.py results/ntn-beam-edge` | `results/ntn-beam-edge/beam-edge-sinr.png`, copied here |
| `sched.png` | `plot_ntn_sched.py results/ntn-sched10` | `results/ntn-sched10/sched.png`, copied here |

The renames exist because the generating scripts name their output after the study, while
the documentation names figures after what they show. The copy is the only step not in the
script.

## Notes

`bentpipe-pass.png` is drawn from the geometry log (`log.txt`) that any bent-pipe run writes:
both hop elevations and distances, both hop SNRs, the cascaded end-to-end SNR, and the
one-way radio delay. `--min-elev 5` drops the below-horizon tail, where the 3GPP path-loss
elevation terms diverge and the cascaded SNR falls off a cliff.

The original figure was produced by a script that was never committed; this generator was
written against the same data and reproduces its numbers (bent-pipe loop 55-84 ms against
4-19 ms single-hop, versus 55-83 and 5-18 in the original).
