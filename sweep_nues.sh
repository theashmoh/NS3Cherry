#!/usr/bin/env bash
# Run ntnlab_nr (or a copy of it under another name, -x) for a range of numbers of UEs and
# collect its --csv files in one place.
#
# Run it from the ns-3 directory (where ./ns3 is):
#
#   bash sweep_nues.sh [options] [--] <ntnlab_nr arguments>
#
#   -f N      first number of UEs              (default 1)
#   -l N      last number of UEs               (default 100)
#   -s N      step                             (default 1)
#   -r "R.."  RngRun values, one run of each   (default "1"; e.g. "1 2 3" for three drops)
#   -j N      runs at the same time            (default: the number of CPU cores)
#   -o DIR    output folder                    (default sweep_<date>_<time>)
#   -p        also write <run>_packets.csv, one line per delivered packet (large files)
#   -n        do not build first
#   -x NAME   the program, scratch/NAME.cc     (default ntnlab_nr; e.g. -x ntnplot)
#   -S        the six scenarios: ROHC on and off x --scheduling=dynamic, sps1, sps2, each for
#             every number of UEs and RngRun (the sweep sets --rohc and --scheduling; --pdcch=1)
#
# Example: the whole pass, 5 MHz, PDCCH with errors, SPS scenario 1, 1 to 100 UEs:
#
#   bash sweep_nues.sh -o sweep_sps1 --fullPass=1 --bandwidthMHz=5 --pdcch=1 \
#       --scheduling=sps1 --idealControl=0
#
# The six scenarios compared, 1 to 20 UEs, three drops each:
#
#   bash sweep_nues.sh -S -f 1 -l 20 -r "1 2 3" -o sweep_six --fullPass=1 --bandwidthMHz=5 \
#       --idealControl=0
#
# Each run goes to DIR/runs/n<UEs>_r<RngRun>/ (with -S: DIR/runs/rohc1-sps1-n<UEs>_r<RngRun>/
# and so on): out.txt (what ntnlab_nr printed), err.txt, and its CSV files. At the end the CSV
# files of all runs are joined, one file per kind (the columns rohc and scheduling tell the
# scenarios apart):
#
#   DIR/calls.csv       a line per flow of a call (dir: dl, ul; two of them for two-way calls)
#                       and per bearer of it (bearer = 0: the call, 1, 2, ...: its bearers):
#                       losses, 5QI 1 (on bearer, late, PER, delay percentiles, missed, satisfied),
#                       the DCIs of the UE, the far end's stretch, end to end, and whether
#                       the call was admitted, blocked or released
#   DIR/control.csv     a line per run: calls offered, admitted, blocked (by resource),
#                       released, and what the admitted calls reserved
#   DIR/delays.csv      per flow: delivered packets per 1 ms of UE-UPF delay (for delay CDFs)
#   DIR/pdcch.csv       a line per run: CCEs, DCIs and blocking by purpose, aggregation levels,
#                       missed DCIs, SPS / configured grant counters
#   DIR/rohc.csv        a line per connection (RNTI) of a bearer: compressor, decompressor, mode
#   DIR/rohc_types.csv  ROHC packet types, % of the packets of each connection
#   DIR/packets.csv     with -p: a line per delivered voice packet
#
# Every line starts with the columns that identify its run (nUes, direction, scheduling, ...,
# rngRun). A run that finished is not run again: start the same command again to finish an
# interrupted sweep (Ctrl-C stops all runs). Another ntnlab_nr command needs another folder.
# Runs are started from the largest number of UEs down, so that the long ones do not come last.
#
# Plots: python3 plot_sweep.py DIR (needs pandas and matplotlib); a sweep with -S gets the
# scenarios compared, two-way calls (ntnlab_nr's default) a set of plots per direction.
#
# Run time grows faster than the number of UEs, and a satellite that flies takes about 2.7 times
# as long as one that stands still (two-way calls carry twice the packets of the one-way calls
# measured here, --farEnd=ue adds the terrestrial cells). Measured, one core, 5 MHz, --pdcch=1
# --scheduling=sps1 --direction=dl, 10 s
# of calls, satellite still: 1 UE 2 s, 20 UEs 30 s, 50 UEs 131 s, 99 UEs 360 s (flying, 20 UEs:
# 80 s). A whole pass (--fullPass=1) is 509 s of calls, about 50 times that: 100 UEs take many
# hours, and 1 to 100 in steps of 1 hundreds of CPU hours. For a first look use fewer points
# (-s 10) or shorter calls (--freezeGeometry=1 --duration=60s); memory is small (about 50 MB for
# 20 UEs), so -j can be the number of cores. An optimized build runs faster (no asserts, no
# logging, -O3): ./ns3 configure --build-profile=optimized (with the options of your configure),
# ./ns3 build. The program built last is the one the sweep runs (sweep.txt names it); a folder
# should hold the runs of one program only.

set -u

first=1
last=100
step=1
seeds="1"
jobs=$(nproc 2>/dev/null || echo 2)
outdir="sweep_$(date +%Y%m%d_%H%M%S)"
packets=0
build=1
prog=ntnlab_nr
six=0

# The options of the sweep come first; the program's (--name=value) start at the first argument
# that begins with "--" (or after a "--" of its own)
own=()
while [ $# -gt 0 ]; do
    case "$1" in
    --)
        shift
        break
        ;;
    --?*) break ;;
    *)
        own+=("$1")
        shift
        ;;
    esac
done
progArgs=("$@")
set -- ${own[@]+"${own[@]}"}
while getopts "f:l:s:r:j:o:x:pnSh" opt; do
    case "$opt" in
    f) first=$OPTARG ;;
    l) last=$OPTARG ;;
    s) step=$OPTARG ;;
    r) seeds=$OPTARG ;;
    j) jobs=$OPTARG ;;
    o) outdir=$OPTARG ;;
    x) prog=$OPTARG ;;
    p) packets=1 ;;
    n) build=0 ;;
    S) six=1 ;;
    h | *)
        sed -n '2,63p' "$0"
        exit 1
        ;;
    esac
done
shift $((OPTIND - 1))
if [ $# -gt 0 ]; then
    echo "not an option of the sweep: $*  (the program's arguments are --name=value)" >&2
    exit 1
fi

if [ ! -x ./ns3 ]; then
    echo "Run this from the ns-3 directory (where ./ns3 is)." >&2
    exit 1
fi

# The arguments of the program, without what the sweep sets itself
args=()
for a in ${progArgs[@]+"${progArgs[@]}"}; do
    case "$a" in
    --nUes=* | --csv=* | --csvPackets=* | --RngRun=*)
        echo "note: $a is set by the sweep, left out" >&2
        ;;
    --rohc=* | --scheduling=*)
        if [ "$six" = 1 ]; then
            echo "note: $a is set by the sweep (-S), left out" >&2
        else
            args+=("$a")
        fi
        ;;
    *) args+=("$a") ;;
    esac
done

# The scenarios: a name (part of the folder of a run) and the arguments it adds
names=("")
declare -A extraArgs=()
if [ "$six" = 1 ]; then
    case " ${args[*]-} " in
    *" --pdcch=0 "* | *" --pdcch=false "*)
        echo "-S needs --pdcch=1: the SPS scenarios are about the PDCCH" >&2
        exit 1
        ;;
    *" --pdcch="*) ;;
    *)
        echo "note: --pdcch=1 added (-S)" >&2
        args+=("--pdcch=1")
        ;;
    esac
    names=()
    for r in 1 0; do
        for s in dynamic sps1 sps2; do
            names+=("rohc$r-$s")
            extraArgs["rohc$r-$s"]="--rohc=$r --scheduling=$s"
        done
    done
fi
argsLine="${args[*]-}"
if [ "$six" = 1 ]; then
    argsLine="$argsLine (+ the six scenarios, -S)"
fi

prog=${prog#scratch/}
prog=${prog%.cc}
if [ ! -f "scratch/$prog.cc" ]; then
    echo "there is no scratch/$prog.cc (-x NAME: the program is scratch/NAME.cc)" >&2
    exit 1
fi
if ! grep -q '"csv"' "scratch/$prog.cc"; then
    echo "scratch/$prog.cc has no --csv option (ntnlab_nr.cc of 8 Oct 2026 and later have)" >&2
    exit 1
fi
if [ "$build" = 1 ]; then
    ./ns3 build "$prog" || exit 1
fi

# The program itself, so that many runs do not each start ./ns3: the one built last (with
# another build profile, e.g. optimized, build/scratch has one of each)
# (ns3.48-NAME-default, -optimized, -debug; NAME alone for the release profile)
bin=$(ls -t build/scratch/ns3*-"$prog"-* build/scratch/ns3*-"$prog" 2>/dev/null | head -n 1)
if [ -z "$bin" ] || [ ! -x "$bin" ]; then
    echo "build/scratch has no $prog program: build it first (./ns3 build $prog)" >&2
    exit 1
fi
bin=$(cd "$(dirname "$bin")" && pwd)/$(basename "$bin")
echo "program: $bin"
sum=$(md5sum "$bin" | cut -d' ' -f1)

mkdir -p "$outdir/runs" || exit 1
outdir=$(cd "$outdir" && pwd)
# The runs of a folder must all come from the same program arguments
if [ -f "$outdir/sweep.txt" ] &&
    ! grep -qxF -- "arguments: $argsLine" "$outdir/sweep.txt"; then
    echo "$outdir has runs of other program arguments:" >&2
    grep '^arguments:' "$outdir/sweep.txt" >&2
    echo "use another folder (-o)" >&2
    exit 1
fi
if [ -f "$outdir/sweep.txt" ] && ! grep -qxF -- "binary:    $sum" "$outdir/sweep.txt"; then
    echo "note: the program is not the one the runs already in $outdir were made with" >&2
fi
{
    echo "date:      $(date)"
    echo "program:   $bin"
    echo "binary:    $sum"
    echo "arguments: $argsLine"
    echo "nUes:      $first to $last, step $step"
    echo "RngRun:    $seeds"
} >>"$outdir/sweep.txt"
echo "sweep: $outdir"
# Ctrl-C: stop the runs too
trap 'trap - INT TERM; echo " stopped" >&2; kill 0' INT TERM

# The runs, largest number of UEs first; an item is "nUes RngRun scenario"
list=()
for n in $(seq "$first" "$step" "$last" | sort -rn); do
    for v in "${names[@]}"; do
        for r in $seeds; do
            list+=("$n $r $v")
        done
    done
done
total=${#list[@]}

# The folder of a run
run_name() {
    echo "${3:+$3-}n$1_r$2"
}

run_one() {
    local n=$1 r=$2 v=${3-}
    local dir
    dir="$outdir/runs/$(run_name "$n" "$r" "$v")"
    local what="nUes=$n RngRun=$r${v:+ $v}"
    if [ -f "$dir/done" ]; then
        echo "  $what: already there"
        return 0
    fi
    mkdir -p "$dir"
    rm -f "$dir"/run_*.csv
    local extra=()
    if [ "$packets" = 1 ]; then
        extra+=("--csvPackets=1")
    fi
    local scenario=()
    if [ -n "$v" ]; then
        read -r -a scenario <<<"${extraArgs[$v]}"
    fi
    local t0=$SECONDS
    if "$bin" ${args[@]+"${args[@]}"} ${scenario[@]+"${scenario[@]}"} ${extra[@]+"${extra[@]}"} \
        --nUes="$n" --RngRun="$r" --csv="$dir/run" >"$dir/out.txt" 2>"$dir/err.txt"; then
        touch "$dir/done"
        echo "  $what: done in $((SECONDS - t0)) s"
    else
        echo "  $what: FAILED (see $dir/err.txt)"
        rm -f "$dir"/run_*.csv
        return 1
    fi
}

running=0
k=0
for item in "${list[@]}"; do
    k=$((k + 1))
    if [ "$running" -ge "$jobs" ]; then
        wait -n
        running=$((running - 1))
    fi
    set -- $item
    echo "[$k/$total] nUes=$1 RngRun=$2${3:+ $3}"
    run_one "$1" "$2" "${3-}" &
    running=$((running + 1))
done
wait

# Join the files of the runs that finished: one header, then the lines of every run
for kind in calls control delays packets pdcch rohc rohc_types; do
    files=()
    for d in "$outdir"/runs/*/; do
        if [ -f "$d/done" ] && [ -f "$d/run_$kind.csv" ]; then
            files+=("$d/run_$kind.csv")
        fi
    done
    if [ ${#files[@]} = 0 ]; then
        continue
    fi
    head -n 1 "${files[0]}" >"$outdir/$kind.csv"
    for f in "${files[@]}"; do
        tail -n +2 "$f" >>"$outdir/$kind.csv"
    done
    echo "$outdir/$kind.csv: $(($(wc -l <"$outdir/$kind.csv") - 1)) lines from ${#files[@]} runs"
done

# Runs of this sweep without results, and unfinished runs of an earlier sweep into this folder
declare -A mine
failed=""
for item in "${list[@]}"; do
    set -- $item
    name=$(run_name "$1" "$2" "${3-}")
    mine["$name"]=1
    [ -f "$outdir/runs/$name/done" ] || failed+=" $name"
done
earlier=""
for d in "$outdir"/runs/*/; do
    name=$(basename "$d")
    [ -f "$d/done" ] || [ -n "${mine[$name]-}" ] || earlier+=" $name"
done
if [ -n "$earlier" ]; then
    echo "not finished, from an earlier sweep into this folder (stopped; not in the CSV files):"
    echo "   $earlier"
    echo "  to finish them, run that sweep again (its -f, -l, -s); or delete their folders"
fi
if [ -n "$failed" ]; then
    echo "runs of this sweep without results (see their err.txt):$failed"
    exit 1
fi
