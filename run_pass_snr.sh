#!/usr/bin/env bash
# run_pass_snr.sh -- run one full satellite pass of a single UE many times, each time with its own
# random seed (--RngRun), keep the SNR of UE 0 per elevation step of the pass, merge it all into
# CSV files and plot it.
#
#   bash run_pass_snr.sh [RUNS] [FIRST_SEED]
#
#   RUNS        how many passes to simulate                       (default 500)
#   FIRST_SEED  --RngRun of the first run; run k uses FIRST_SEED+k-1   (default 1)
#
# Run it from the ns-3 folder (the one with ./ns3), or set NS3_DIR. Settings through the
# environment:
#
#   NS3_DIR     the ns-3 folder                                   (default: the current folder)
#   PROG        the program, as ./ns3 run takes it        (default: scratch/nrvoice-v11.cc)
#   OUT         where the results go                       (default: results/pass_snr)
#   JOBS        runs at the same time                  (default: half the cores, at least 1)
#   SKIP_BUILD  1 = do not run ./ns3 build first                  (default 0)
#   KEEP_LOGS   1 = keep the text output of every run             (default 0; kept if a run fails)
#   NO_PLOT     1 = only run and merge, no plot                   (default 0)
#   ELEVATION   the pass starts and ends at this elevation, deg   (default 10)
#   EXTRA_ARGS  more program options, put last (they win)        (default none)
#
# What it writes in $OUT:
#   epochs/run_0001_snr.csv, _dlpower.csv ...   what each run wrote (the SNR tables by elevation)
#   epochs/run_0001.done                        marks a run that finished: an interrupted
#                                               script, started again, skips those and goes on
#   snr_pass_all.csv        one line per run and elevation step of UE 0 (all runs together)
#   snr_pass_summary.csv    over the runs, per series and step: mean, sd, 5/25/50/75/95 %, ...
#   snr_pass.png            DL and UL SNR of UE 0 during the pass: mean over the runs, band 5-95 %
#   snr_pass_detail.png     the same, one panel per column, with every run's curve
#
# The pass: --fullPass=1 --elevation=10: the satellite rises over UE 0 from 10 deg, passes over it
# and sets down to 10 deg. The script puts the SNR of the call in elevation steps of 10 deg; a
# step's label is negative while the satellite comes closer (-10 ... -80), 90 over the UE, and
# positive again while it moves away (80 ... 10). Change --elevation below for another pass.

set -u

RUNS=${1:-500}
SEED0=${2:-1}
NS3_DIR=${NS3_DIR:-.}
PROG=${PROG:-scratch/nrvoice-v11.cc}
OUT=${OUT:-results/pass_rohconrepetmax}
KEEP_LOGS=${KEEP_LOGS:-0}
SKIP_BUILD=${SKIP_BUILD:-0}
NO_PLOT=${NO_PLOT:-0}
ELEVATION=${ELEVATION:-10}
EXTRA_ARGS=${EXTRA_ARGS:-}
if [ -z "${JOBS:-}" ]; then
    JOBS=$(( $(nproc 2>/dev/null || echo 2) ))
    [ "$JOBS" -ge 1 ] || JOBS=1
fi

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
NS3_DIR=$(cd "$NS3_DIR" 2>/dev/null && pwd) || { echo "error: no folder $NS3_DIR" >&2; exit 1; }
[ -x "$NS3_DIR/ns3" ] || { echo "error: $NS3_DIR/ns3 not found: run this in the ns-3 folder, or set NS3_DIR" >&2; exit 1; }
case "$OUT" in /*) ;; *) OUT="$PWD/$OUT" ;; esac
mkdir -p "$OUT/epochs" || exit 1

# Your command, with one UE (--nUes=1) and without --RngRun and --csv, which are set per run.
# The CSV that has the SNR by elevation is written by --csv (a prefix; the program adds _snr.csv).
ARGS="--satSet=2 --fullPass=1 --pdcch=1 --pdcchSymbols=0 --scheduling=dynamic --idealControl=0 \
--nUes=1 --bandwidthMHz=5 --dlFrequencyGHz=2.175 --altitudeKm=600 --scenario=NTN-LOS \
--scheduler=ns3::NrMacSchedulerOfdmaRR --farEnd=ue --direction=both --errorModel=NrEesmIrT1 \
--ueAntennaGainDbi=-5.5 --puschRepetitions=0 --ulGrantRbs=1 --fastFading=fixed \
--ns3::NrPdcp::RohcIrRefresh=250 --ns3::NrPdcp::RohcFoRefresh=100 --elevation=$ELEVATION \
--ueBufferAgeS=2 --ulWideSinrDb=3 --ulWideRbs=2 --dlBoostMaxDb=6 \
--dlPfdLimit=-90 --vafMean=0.65 --retry=1 --dlClosedLoop=1 --ulClosedLoop=1 --rohc=1"

echo "Building $PROG is demolished!"

# One run: the k-th. Writes epochs/run_KKKK_*.csv; run_KKKK.done only if it went well.
run_one() {
    local k=$1
    local seed=$((SEED0 + k - 1))
    local tag
    tag=$(printf 'run_%04d' "$k")
    local prefix="$OUT/epochs/$tag"
    if [ -f "$prefix.done" ]; then
        echo "[$k/$RUNS] seed $seed: done before, skipped"
        return 0
    fi
    rm -f "$prefix"_*.csv "$prefix.log"
    local t0=$SECONDS
    ( cd "$NS3_DIR" && ./ns3 run --no-build "$PROG $ARGS --RngRun=$seed --csv=$prefix $EXTRA_ARGS" ) \
        > "$prefix.log" 2>&1
    local rc=$?
    if [ $rc -eq 0 ] && [ -s "${prefix}_snr.csv" ] && [ "$(wc -l < "${prefix}_snr.csv")" -ge 2 ]; then
        # keep what the plot needs, drop the rest
        find "$OUT/epochs" -maxdepth 1 -name "${tag}_*.csv" ! -name "${tag}_snr.csv" \
            ! -name "${tag}_dlpower.csv" -delete
        [ "$KEEP_LOGS" = "1" ] || rm -f "$prefix.log"
        : > "$prefix.done"
        echo "[$k/$RUNS] seed $seed: ok ($((SECONDS - t0)) s)"
    else
        echo "[$k/$RUNS] seed $seed: FAILED (exit $rc), see $prefix.log" >&2
        return 1
    fi
}
export -f run_one
export OUT NS3_DIR PROG ARGS SEED0 RUNS KEEP_LOGS EXTRA_ARGS

echo "$RUNS runs, seeds $SEED0 to $((SEED0 + RUNS - 1)), $JOBS at a time, results in $OUT"
start=$SECONDS
seq 1 "$RUNS" | xargs -P "$JOBS" -I{} bash -c 'run_one {}'
failed=$(( RUNS - $(ls "$OUT/epochs"/*.done 2>/dev/null | wc -l) ))
echo "Finished in $((SECONDS - start)) s; runs not done: $failed (start the script again to repeat them)"

if [ "$NO_PLOT" != "1" ]; then
    python3 "$here/plot_pass_snr.py" "$OUT" || {
        echo "The plot needs python3 with numpy and matplotlib: pip install numpy matplotlib" >&2
        exit 1
    }
fi
