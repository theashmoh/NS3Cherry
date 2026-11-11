#!/usr/bin/env bash
# run_pass_snr.sh -- run one full satellite pass of a single UE (or any scenario) many times, each
# time with its own random seed (--RngRun), keep what each run writes, merge it, and plot it.
# Optionally for several values of a parameter (a sweep): one line per value in the plots.
#
#   bash run_pass_snr.sh [RUNS] [FIRST_SEED]
#
#   RUNS        how many passes to simulate (per value, in a sweep)     (default 500)
#   FIRST_SEED  --RngRun of the first run; run k uses FIRST_SEED+k-1   (default 1)
#               In a sweep every case has seeds of its own, see SEEDS below.
#
# Run it from the ns-3 folder (the one with ./ns3), or set NS3_DIR. Settings through the
# environment:
#
#   SWEEP       the parameter(s) to vary, and the values:   SWEEP="puschRepetitions=0,4,8,16"
#               any option of the program, with or without the leading --, values separated by
#               commas. Several parameters (space or ; between them) make every combination:
#               SWEEP="puschRepetitions=0,9 satSet=1,2" is 4 cases. The sweep wins over BASE_ARGS
#               and EXTRA_ARGS. To add a value later, put it at the end of the list (then the
#               runs already done keep their seeds).                              (default: none)
#   SEEDS       distinct = every case of a sweep has seeds of its own: case 1 FIRST_SEED ...
#               FIRST_SEED+RUNS-1, case 2 from FIRST_SEED+SEED_STRIDE, case 3 from
#               FIRST_SEED+2*SEED_STRIDE ... (independent samples in every line)
#               shared   = every case runs the same seeds, so the same UE positions and talk
#               patterns in every line: the lines then differ by the parameter and not by the
#               dice (the best way to see a small difference), but they are not independent
#                                                                          (default: distinct)
#   SEED_STRIDE the seed distance between cases with SEEDS=distinct; RUNS can grow up to it
#               without the seeds of two cases meeting                     (default 1000)
#   BASE_ARGS   the program's options for every run, instead of the command written below
#               (without --RngRun and --csv, which are set per run)       (default: the command below)
#   EXTRA_ARGS  more program options, after the command (they win over it)      (default none)
#   PLOT_ARGS   what plot_sweep.py draws after a sweep, e.g.
#               PLOT_ARGS="--y delayP98Ms perPct --x ue"   (default: the four SNR series against
#               the elevation). Draw anything else later, without running again, with
#               python3 plot_sweep.py $OUT --y ... (plot_sweep.py --help)
#   NS3_DIR     the ns-3 folder                                   (default: the current folder)
#   PROG        the program, as ./ns3 run takes it        (default: scratch/nrvoice-v11.cc)
#   OUT         where the results go                       (default: results/pass_snr)
#   JOBS        runs at the same time                  (default: half the cores, at least 1)
#   SKIP_BUILD  1 = do not build first (build it yourself: ./ns3 build $PROG)   (default 0)
#   SKIP_SMOKE  1 = do not try each case for 3 simulated seconds before the long runs (default 0)
#   KEEP_LOGS   1 = keep the text output of every run             (default 0; kept if a run fails)
#   KEEP_TABLES the CSV tables of each run to keep ("all" = every one)
#               (default: snr dlpower calls tries control loop ulgrants pdcch rohc rohc_types)
#   NO_PLOT     1 = only run and merge, no plot                   (default 0)
#   CASE_PLOTS  1 = also the detail plots of plot_pass_snr.py for every case of a sweep (default 0)
#   ELEVATION   the pass starts and ends at this elevation, deg   (default 10)
#
# What it writes in $OUT:
#   without SWEEP:
#     epochs/run_0001_snr.csv, _calls.csv ...    what each run wrote (all the tables)
#     epochs/run_0001.done                       marks a run that finished: an interrupted
#                                                script, started again, skips those and goes on
#     snr_pass_all.csv, snr_pass_summary.csv, snr_pass.png, snr_pass_detail.png   (plot_pass_snr.py)
#   with SWEEP: one folder per case, e.g. $OUT/puschRepetitions=4/, with the same epochs/ and a
#   case.txt (the parameter values and the full command), and in $OUT what plot_sweep.py makes:
#     sweep_snr_by_puschRepetitions.png/.csv     SNR against the elevation, a line per value
#     sweep_snr_all.csv                          the SNR of every case, run and step
#
# Runs go in the order run 1 of every case, run 2 of every case ..., so a sweep stopped half way
# has the same number of runs in every case. Start the script again with the same settings to
# continue it; add a value to SWEEP and only the new case runs. A finished run is marked with its
# seed (run_0001.done holds it): a run whose seed is not the one it would get now (another
# FIRST_SEED, SEEDS or order of the values) is simulated again, never mixed in.
#
# The pass: --fullPass=1 --elevation=10: the satellite rises over UE 0 from 10 deg, passes over it
# and sets down to 10 deg. The script puts the SNR of the call in elevation steps of 10 deg; a
# step's label is negative while the satellite comes closer (-10 ... -80), 90 over the UE, and
# positive again while it moves away (80 ... 10). Change ELEVATION for another pass. A scenario
# without a pass (--fullPass=0, many UEs) works too: put it in BASE_ARGS; the SNR plot then has
# one step, and the delay/PER plots (--x ue, --x cdf) are the ones to draw.

set -u

RUNS=${1:-500}
SEED0=${2:-1}
NS3_DIR=${NS3_DIR:-.}
PROG=${PROG:-scratch/nrvoice-v11.cc}
OUT=${OUT:-results/sweep_snr}
KEEP_LOGS=${KEEP_LOGS:-0}
SKIP_BUILD=${SKIP_BUILD:-0}
SKIP_SMOKE=${SKIP_SMOKE:-0}
NO_PLOT=${NO_PLOT:-0}
CASE_PLOTS=${CASE_PLOTS:-0}
SEEDS=${SEEDS:-distinct}
SEED_STRIDE=${SEED_STRIDE:-1000}
case "$SEEDS" in
    distinct|shared) ;;
    *) echo "error: SEEDS is distinct or shared, not '$SEEDS'" >&2; exit 1 ;;
esac
case "$RUNS$SEED0$SEED_STRIDE" in
    ''|*[!0-9]*) echo "error: RUNS, FIRST_SEED and SEED_STRIDE are whole numbers" >&2; exit 1 ;;
esac
if [ "$SEEDS" = "distinct" ] && [ "$RUNS" -gt "$SEED_STRIDE" ]; then
    echo "error: $RUNS runs per case do not fit in SEED_STRIDE=$SEED_STRIDE seeds: raise SEED_STRIDE" >&2
    exit 1
fi
# The seed of run k of case idx (idx = 1, 2 ... in the order of the cases; 1 without a sweep)
seed_of() {
    if [ "$SEEDS" = "distinct" ]; then
        echo $((SEED0 + ($1 - 1) * SEED_STRIDE + $2 - 1))
    else
        echo $((SEED0 + $2 - 1))
    fi
}
ELEVATION=${ELEVATION:-10}
EXTRA_ARGS=${EXTRA_ARGS:-}
SWEEP=${SWEEP:-}
PLOT_ARGS=${PLOT_ARGS:-}
KEEP_TABLES=${KEEP_TABLES:-"snr dlpower calls tries control loop ulgrants pdcch rohc rohc_types"}
if [ -z "${JOBS:-}" ]; then
    JOBS=$(( $(nproc 2>/dev/null || echo 2) ))
    [ "$JOBS" -ge 1 ] || JOBS=1
fi

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
NS3_DIR=$(cd "$NS3_DIR" 2>/dev/null && pwd) || { echo "error: no folder $NS3_DIR" >&2; exit 1; }
[ -x "$NS3_DIR/ns3" ] || { echo "error: $NS3_DIR/ns3 not found: run this in the ns-3 folder, or set NS3_DIR" >&2; exit 1; }
case "$OUT" in /*) ;; *) OUT="$PWD/$OUT" ;; esac
mkdir -p "$OUT" || exit 1

# The command (BASE_ARGS replaces it), with one UE (--nUes=1) and without --RngRun and --csv,
# which are set per run. The CSV tables are written by --csv (a prefix; the program adds
# _snr.csv, _calls.csv ...).
DEFAULT_ARGS="--satSet=1 --fullPass=1 --pdcch=1 --pdcchSymbols=0 --scheduling=dynamic --idealControl=0 \
--nUes=1 --bandwidthMHz=5 --dlFrequencyGHz=2.175 --altitudeKm=600 --scenario=NTN-LOS \
--scheduler=ns3::NrMacSchedulerOfdmaRR --farEnd=ue --direction=both --errorModel=NrEesmIrT1 \
--ueAntennaGainDbi=-5.5 --puschRepetitions=16 --ulGrantRbs=1 --fastFading=model \
--ns3::NrPdcp::RohcIrRefresh=250 --ns3::NrPdcp::RohcFoRefresh=100 --elevation=$ELEVATION \
--ueBufferAgeS=2 --ulWideSinrDb=3 --ulWideRbs=2 --dlBoostMaxDb=6 \
--dlPfdLimit=-90 --vafMean=0.65 --retry=1 --dlClosedLoop=1 --ulClosedLoop=1 --fadingHoldMs=200"
ARGS=${BASE_ARGS:-$DEFAULT_ARGS}
case "$ARGS" in
    *--fullPass=1*) ;;
    *) echo "note: the command has no --fullPass=1: there is no pass, the SNR tables have one step" ;;
esac

# --- The cases: every combination of the values of SWEEP --------------------------------------------
# case_label  "puschRepetitions=4,satSet=2"   case_args  " --puschRepetitions=4 --satSet=2"
# case_params the lines of case.txt
case_label=("")
case_args=("")
case_params=("")
SWEEP=${SWEEP//;/ }
sweep_names=""
for spec in $SWEEP; do
    case "$spec" in
        *=*) ;;
        *) echo "error: SWEEP entry '$spec' is not name=value1,value2,..." >&2; exit 1 ;;
    esac
    name=${spec%%=*}
    vals=${spec#*=}
    name=${name#--}
    case "$name" in
        ''|*[!A-Za-z0-9_:.]*) echo "error: SWEEP parameter name '$name' has characters that are not allowed" >&2; exit 1 ;;
    esac
    case " $sweep_names " in
        *" $name "*) echo "error: SWEEP names $name twice" >&2; exit 1 ;;
    esac
    sweep_names="$sweep_names $name"
    case "$vals" in
        ''|,*|*,|*,,*) echo "error: SWEEP $name has an empty value in '$vals'" >&2; exit 1 ;;
    esac
    short=${name##*::}
    IFS=',' read -r -a vlist <<< "$vals"
    new_label=(); new_args=(); new_params=()
    for i in "${!case_label[@]}"; do
        seen=","
        for v in "${vlist[@]}"; do
            case "$seen" in *",$v,"*) continue ;; esac   # a value twice: once
            seen="$seen$v,"
            new_label+=("${case_label[$i]:+${case_label[$i]},}$short=$v")
            new_args+=("${case_args[$i]} --$name=$v")
            new_params+=("${case_params[$i]}param.$short=$v"$'\n')
        done
    done
    case_label=("${new_label[@]}")
    case_args=("${new_args[@]}")
    case_params=("${new_params[@]}")
    case " $EXTRA_ARGS " in
        *"--$name="*) echo "note: EXTRA_ARGS has --$name too; the SWEEP value wins" ;;
    esac
done
n_cases=${#case_label[@]}

PLAN="$OUT/.plan.tsv"
: > "$PLAN"
dirs_seen=""
for ((i = 0; i < n_cases; i++)); do
    if [ -z "$SWEEP" ]; then
        dir="$OUT"; label="-"
    else
        label=${case_label[$i]}
        dir="$OUT/$(printf '%s' "$label" | tr -c 'A-Za-z0-9._=,-' '_')"
        case "$dirs_seen" in
            *"|$dir|"*) echo "error: two cases would share the folder $dir" >&2; exit 1 ;;
        esac
        dirs_seen="$dirs_seen|$dir|"
    fi
    mkdir -p "$dir/epochs" || exit 1
    printf '%d\t%s\t%s\t%s\n' $((i + 1)) "$dir" "$label" "${case_args[$i]}" >> "$PLAN"
    {
        echo "label=${label/#-/all runs}"
        printf '%s' "${case_params[$i]}"
        echo "seeds=$(seed_of $((i + 1)) 1) to $(seed_of $((i + 1)) "$RUNS") ($SEEDS)"
        echo "args=$ARGS $EXTRA_ARGS${case_args[$i]}"
    } > "$dir/case.txt"
done

if [ "$SKIP_BUILD" != "1" ]; then
    # Only this program, not a bare "./ns3 build": that builds every target of the tree (all
    # examples, every scratch program, the whole nr module again if a header changed) and can take
    # hours. Its output is shown as it comes. Up to date: it returns at once.
    echo "Building $PROG only. Up to date: a few seconds. If the nr module changed since your last"
    echo "build (an installer run), it is compiled once now, which takes a while; progress below."
    tb=$SECONDS
    (cd "$NS3_DIR" && ./ns3 build "$PROG") 2>&1 | tee "$OUT/build.log"
    if [ "${PIPESTATUS[0]}" -ne 0 ]; then
        echo "error: building $PROG failed (see above, and $OUT/build.log)" >&2
        exit 1
    fi
    echo "Build done in $((SECONDS - tb)) s"
fi

# The program must start (a second, not a 5-minute run to find out)
if ! (cd "$NS3_DIR" && ./ns3 run --no-build "$PROG --PrintHelp") > "$OUT/smoke.log" 2>&1; then
    echo "error: $PROG does not start (exit status not 0); the last lines of $OUT/smoke.log:" >&2
    tail -15 "$OUT/smoke.log" >&2
    exit 1
fi

# Every case is tried for 3 simulated seconds first (a few seconds of work): a misspelt option or
# a combination the program refuses stops the script here, not after the first pass of the sweep
if [ "$SKIP_SMOKE" != "1" ]; then
    mkdir -p "$OUT/.smoke"
    while IFS=$'\t' read -r idx dir label cargs; do
        if ! (cd "$NS3_DIR" && ./ns3 run --no-build \
              "$PROG $ARGS $EXTRA_ARGS$cargs --fullPass=0 --duration=3s --RngRun=1 --csv=$OUT/.smoke/s") \
              > "$OUT/.smoke/case$idx.log" 2>&1; then
            echo "error: the program refuses case '${label/#-/the command}' (a 3 s test run); its first lines:" >&2
            head -3 "$OUT/.smoke/case$idx.log" >&2
            echo "(SKIP_SMOKE=1 skips this test; the whole output is in $OUT/.smoke/case$idx.log)" >&2
            exit 1
        fi
    done < "$PLAN"
    rm -rf "$OUT/.smoke"
fi

# One run: the k-th of case number idx. Writes <case folder>/epochs/run_KKKK_*.csv; run_KKKK.done
# only if it went well.
run_one() {
    local idx=$1 k=$2
    local dir label cargs _i
    IFS=$'\t' read -r _i dir label cargs < <(sed -n "${idx}p" "$PLAN")
    local seed
    seed=$(seed_of "$idx" "$k")
    local tag
    tag=$(printf 'run_%04d' "$k")
    local prefix="$dir/epochs/$tag"
    local what="run $k/$RUNS"
    [ "$label" = "-" ] || what="$label, run $k/$RUNS"
    if [ -f "$prefix.done" ]; then
        # run_KKKK.done holds the seed the run had; an empty one is from an older version of this
        # script, which gave run k the seed FIRST_SEED+k-1
        local done_seed
        done_seed=$(cat "$prefix.done" 2>/dev/null)
        [ -n "$done_seed" ] || done_seed=$((SEED0 + k - 1))
        if [ "$done_seed" = "$seed" ]; then
            echo "[$what] seed $seed: done before, skipped"
            return 0
        fi
        echo "[$what] done before with seed $done_seed, now seed $seed: simulated again"
        rm -f "$prefix.done"
    fi
    rm -f "$prefix"_*.csv "$prefix.log"
    echo "[$what] seed $seed: started (a full pass takes minutes)"
    local t0=$SECONDS
    ( cd "$NS3_DIR" && ./ns3 run --no-build "$PROG $ARGS $EXTRA_ARGS$cargs --RngRun=$seed --csv=$prefix" ) \
        > "$prefix.log" 2>&1
    local rc=$?
    if [ $rc -eq 0 ] && [ -s "${prefix}_snr.csv" ] && [ "$(wc -l < "${prefix}_snr.csv")" -ge 2 ]; then
        # keep the tables asked for (KEEP_TABLES), drop the rest
        if [ "$KEEP_TABLES" != "all" ]; then
            local f t
            for f in "$dir/epochs/${tag}_"*.csv; do
                [ -e "$f" ] || continue
                t=${f##*/${tag}_}
                t=${t%.csv}
                case " $KEEP_TABLES " in
                    *" $t "*) ;;
                    *) rm -f "$f" ;;
                esac
            done
        fi
        [ "$KEEP_LOGS" = "1" ] || rm -f "$prefix.log"
        echo "$seed" > "$prefix.done"
        echo "[$what] seed $seed: ok ($((SECONDS - t0)) s)"
    else
        echo "[$what] seed $seed: FAILED (exit $rc), see $prefix.log" >&2
        return 1
    fi
}
export -f run_one seed_of
export OUT NS3_DIR PROG ARGS EXTRA_ARGS SEED0 SEEDS SEED_STRIDE RUNS KEEP_LOGS KEEP_TABLES PLAN

if [ -n "$SWEEP" ]; then
    echo "Sweep:$sweep_names; $n_cases cases x $RUNS runs = $((n_cases * RUNS)) passes, $JOBS at a time,"
    echo "results in $OUT (run 1 of every case first, then run 2 ...)"
    if [ "$SEEDS" = "distinct" ]; then
        echo "seeds: a block of its own for every case, $(seed_of 1 1) to $(seed_of 1 "$RUNS") for the first, then +$SEED_STRIDE for each next (SEEDS=shared: the same in every case)"
    else
        echo "seeds: $SEED0 to $((SEED0 + RUNS - 1)) in every case (SEEDS=shared): the same UE positions and talk patterns in every line"
    fi
else
    echo "$RUNS runs, seeds $SEED0 to $((SEED0 + RUNS - 1)), $JOBS at a time, results in $OUT"
fi
start=$SECONDS
for ((k = 1; k <= RUNS; k++)); do
    for ((i = 1; i <= n_cases; i++)); do
        echo "$i $k"
    done
done | xargs -P "$JOBS" -L 1 bash -c 'run_one "$@"' _

total_failed=0
while IFS=$'\t' read -r idx dir label cargs; do
    done_n=$(ls "$dir/epochs"/run_*.done 2>/dev/null | wc -l)
    [ "$label" = "-" ] || echo "  ${label}: $done_n of $RUNS runs done"
    total_failed=$((total_failed + RUNS - done_n))
done < "$PLAN"
echo "Finished in $((SECONDS - start)) s; runs not done: $total_failed (start the script again to repeat them)"

if [ "$NO_PLOT" != "1" ]; then
    plot_fail() {
        if python3 -c "import numpy, matplotlib" 2>/dev/null; then
            echo "The plot stopped (the message above); the runs are kept. Draw again, without running, with" >&2
            echo "  python3 $here/plot_sweep.py $OUT [options]   (--help, --list)" >&2
        else
            echo "The plot needs python3 with numpy and matplotlib: pip install numpy matplotlib" >&2
        fi
        exit 1
    }
    if [ -n "$SWEEP" ]; then
        # shellcheck disable=SC2086
        python3 "$here/plot_sweep.py" "$OUT" $PLOT_ARGS || plot_fail
        if [ "$CASE_PLOTS" = "1" ]; then
            while IFS=$'\t' read -r idx dir label cargs; do
                echo "Detail plots of $label:"
                python3 "$here/plot_pass_snr.py" "$dir" || plot_fail
            done < "$PLAN"
        fi
    else
        python3 "$here/plot_pass_snr.py" "$OUT" || plot_fail
        if [ -n "$PLOT_ARGS" ]; then
            # shellcheck disable=SC2086
            python3 "$here/plot_sweep.py" "$OUT" $PLOT_ARGS || plot_fail
        fi
    fi
    if [ -n "$SWEEP" ]; then
        echo
        echo "More plots from the same runs, no new simulation (python3 $here/plot_sweep.py --help):"
        echo "  python3 $here/plot_sweep.py $OUT --y dl_data ul_1rb"
        echo "  python3 $here/plot_sweep.py $OUT --y delayP98Ms perPct --x ue"
        echo "  python3 $here/plot_sweep.py $OUT --y delayP98Ms perPct --x cdf"
    fi
fi
