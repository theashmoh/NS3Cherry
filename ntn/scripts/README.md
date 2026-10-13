# ntn/scripts

*Author: Jesse Chiu (Kuan-Po) <kchiu5@uw.edu>, University of Washington, Seattle*

The shell scripts locate the **ns-3 root** through `common.sh`, so they can be invoked from any
working directory. These are the scripts the documented examples use.

---

## Building the examples

The harnesses call binaries under `build/contrib/nr/examples/` and resolve the versioned
filenames themselves. Build them with:

```bash
./ns3 build leo-link
```

If `./ns3` fails to configure because an incompatible `cmake` shim is first on `PATH`, drive
CMake directly — it writes build-system state to `cmake-cache/`, not `build/`:

```bash
CMAKE=${CMAKE:-/usr/bin/cmake}
"$CMAKE" -S . -B cmake-cache
"$CMAKE" --build cmake-cache -j16 \
    --target leo-link
```

---

## Harnesses

| harness | what it does | used by |
|---|---|---|
| `run_leo_link_bent_pipe_analysis.py` | runs the transparent example from a parameter file | Example 1 |
| `run_leo_link_analysis.py` | runs the regenerative example from a parameter file; the bent-pipe harness imports it | Example 3 |
| the binary directly | resolved by `common.sh` | `run_ntn_demo.sh` |

Both harnesses take a parameter file:

```bash
python3 ntn/scripts/run_leo_link_analysis.py --param-file ntn/params/<file>.txt
```

Use `--help` on either for the current flag names.

## Run scripts and shared helper

| script | role |
|---|---|
| `common.sh` | safe output cleanup, binary discovery, timing and optional jemalloc setup |
| `run_ntn_demo.sh` | runs the 19 ILLA, OLLA and RLC teaching cases (Examples 6 and 7) |
| `run_ntn_beam_edge.sh` | the beam-edge sweep, 3 UEs at 8 elevations (Example 4) |
| `run_ntn_sched10.sh` | the scheduler comparison, 10 UEs, RR / PF / MR x fresh / stale CSI x 3 seeds (Example 5) |

`run_ntn_beam_edge.sh` and `run_ntn_sched10.sh` run `NTN_JOBS` simulations at a time (default 1).

## Analysis and plotting

| script | input | output |
|---|---|---|
| `summarize_ntn_demo.py` | `results/ntn-demo` | `summary.csv`, `ntn-demo.png` |
| `plot_v2_arq.py` | `results/ntn-demo` | `v2-arq.png` |
| `plot_ntn_demo_figures.py` | `results/ntn-demo` | `v1-tbler.png`, `v2-delay-e30.png` |
| `plot_bentpipe_pass_3panel.py` | a bent-pipe run dir | `bentpipe-pass.png` — geometry, per-link SNR, HARQ loop RTT |
| `analyze_footprint_per_ue.py` | a multi-UE run dir of either harness | per-UE SINR, MCS, throughput, BLER and HARQ against elevation, grouped by distance from the beam centre |
| `plot_ntn_beam_edge.py` | `results/ntn-beam-edge` | `beam-edge-sinr.png` |
| `plot_ntn_sched.py` | `results/ntn-sched10` | `sched.png` — cell throughput, fairness, per-UE throughput against beam taper |

## Ray tracing (Example 2)

| script | what it does |
|---|---|
| `ntn-rt-render.py` | the pass animation: the line of sight sweeping over the terminal, against SNR, modulation and throughput. Reads the `--traceFile` geometry trace and the NR packet trace. |
| `ntn-sionna-cir.py` | the hybrid channel: walks a proxy transmitter along the true line of sight, since Sionna RT has no far-field source |
| `rt-animate.py` | matches Sionna scene PNGs to a metric trace **by index**, not timestamp, so `--update-ms` must equal the run's channel update period |

---

## Conventions

- Most run scripts take an output directory as `$1` and default to something under `results/`.
- `results/` is gitignored. The shared cleanup helper refuses to remove directories outside
  either the top-level `results/` or `ntn/results/` tree.
- `common.sh` finds the built example across ns-3 build profiles; rebuild before running
  (`./ns3 build leo-link`). Set `NTN_BIN` to select an explicit binary.
- Optional environment overrides are `NS3_BUILD_DIR`, `NTN_BIN`, `PYTHON`, `TIME_BIN`,
  `CMAKE`, and `JEMALLOC_LIB`. Jemalloc is used only when `JEMALLOC_LIB` is set.
