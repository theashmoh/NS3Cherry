#!/usr/bin/env bash
# run_sweep.sh - v10.3: run the voice scenarios for a list of user counts, seeds and runs,
# then plot them (plot_sweep.py, next to this file).
#
#   bash run_sweep.sh [options] [-- extra script options]
#
#   --ns3dir DIR      ns-3 tree (default ~/ns-3-dev-NTN-cherry)
#   --bin FILE        the built script (default: newest build/scratch/ns3*-msatnrvoice-v7[-profile])
#   --out DIR         results folder (default ./sweep_<date>); run again with the same --out to resume
#   --scenarios "A B" scenario presets (--voiceScenario), default "A B C D E F G H I"
#   --configs FILE    instead of --scenarios: one line per configuration, "label  options ..."
#                     (e.g.  I_jit  --voiceScenario=I --hostJitter=exp --hostJitterMs=3); # = comment
#   --users "LIST"    user counts (--nUes): "1 5 10 20" or ranges "5:50:5", mixed; commas allowed
#   --seeds "LIST"    --RngSeed values (default 1)
#   --runs "LIST"     --RngRun values (default "1 2 3"); every seed x run is one replica
#   --jobs N          runs in parallel (default: CPUs - 1)
#   --extra "OPTS"    options added to every run (anything the script takes); also after "--"
#   --force           run again what has finished (default: skip runs with exit code 0)
#   --dry-run         print the runs, run nothing
#   --no-plot         do not plot;  --plot-only: only plot what --out holds
#   --pdb MS          PDB line of the plots (default 100)
#
# Each run: <out>/runs/<label>/n<users>/s<seed>_r<run>/ with cmd.txt, stdout.txt, stderr.txt,
# exit_code and the CSVs (--csv=out). <out>/sweep.txt records the settings and the binary.
# Not allowed in --extra / configs (the sweep sets them): --nUes --RngSeed --RngRun --csv.

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3DIR="$HOME/ns-3-dev-NTN-cherry"
BIN=""
OUT="./sweep_$(date +%Y%m%d_%H%M)"
SCENARIOS="A B C D E F G H I"
CONFIGS=""
USERS="1 5 10 20 30 40"
SEEDS="1"
RUNS="1 2 3"
JOBS=$(( $(nproc 2>/dev/null || echo 2) - 1 )); [ "$JOBS" -lt 1 ] && JOBS=1
EXTRA=""
FORCE=0; DRY=0; PLOT=1; RUN=1; PDB=100

die() { echo "run_sweep: $*" >&2; exit 1; }
while [ $# -gt 0 ]; do
    case "$1" in
        --ns3dir) NS3DIR="$2"; shift 2;;
        --bin) BIN="$2"; shift 2;;
        --out) OUT="$2"; shift 2;;
        --scenarios) SCENARIOS="$2"; shift 2;;
        --configs) CONFIGS="$2"; shift 2;;
        --users) USERS="$2"; shift 2;;
        --seeds) SEEDS="$2"; shift 2;;
        --runs) RUNS="$2"; shift 2;;
        --jobs) JOBS="$2"; shift 2;;
        --extra) EXTRA="$EXTRA $2"; shift 2;;
        --force) FORCE=1; shift;;
        --dry-run) DRY=1; shift;;
        --no-plot) PLOT=0; shift;;
        --plot-only) RUN=0; shift;;
        --pdb) PDB="$2"; shift 2;;
        --) shift; EXTRA="$EXTRA $*"; break;;
        -h|--help) sed -n '2,27p' "$0"; exit 0;;
        *) die "unknown option $1 (--help)";;
    esac
done

# "1 5 10:30:10,40" -> "1 5 10 20 30 40"
expand() {
    local tok out=""
    for tok in $(echo "$1" | tr ',' ' '); do
        if [[ "$tok" =~ ^([0-9]+):([0-9]+):([0-9]+)$ ]]; then
            out="$out $(seq "${BASH_REMATCH[1]}" "${BASH_REMATCH[3]}" "${BASH_REMATCH[2]}" | tr '\n' ' ')"
        elif [[ "$tok" =~ ^[0-9]+$ ]]; then
            out="$out $tok"
        else
            die "not a number or start:end:step: $tok"
        fi
    done
    echo $out
}
USERS=$(expand "$USERS"); SEEDS=$(expand "$SEEDS"); RUNS=$(expand "$RUNS")
[ -n "$USERS" ] && [ -n "$SEEDS" ] && [ -n "$RUNS" ] || die "--users, --seeds and --runs need values"

mkdir -p "$OUT/runs" || die "cannot make $OUT"
OUT="$(cd "$OUT" && pwd)"

# the configurations: label + options
declare -a LABELS OPTS
if [ -n "$CONFIGS" ]; then
    [ -r "$CONFIGS" ] || die "cannot read $CONFIGS"
    while IFS= read -r line || [ -n "$line" ]; do
        line="${line%%#*}"; line="$(echo "$line" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
        [ -z "$line" ] && continue
        LABELS+=("${line%%[[:space:]]*}")
        rest="${line#"${LABELS[-1]}"}"; OPTS+=("$(echo "$rest" | sed 's/^[[:space:]]*//')")
    done < "$CONFIGS"
else
    for s in $SCENARIOS; do
        [[ "$s" =~ ^[A-I]$ ]] || die "scenario must be A..I: $s"
        LABELS+=("$s"); OPTS+=("--voiceScenario=$s")
    done
fi
[ ${#LABELS[@]} -gt 0 ] || die "no configuration"
for o in "$EXTRA" "${OPTS[@]}"; do
    for bad in --nUes --RngSeed --RngRun --csv; do
        [[ " $o" == *" $bad="* || " $o" == *" $bad "* ]] && die "$bad is set by the sweep, leave it out of the options"
    done
done

if [ "$RUN" = 1 ]; then
    if [ -z "$BIN" ]; then
        # ns3.49-msatnrvoice-v7, or with the build profile: ...-default, -optimized, -debug (newest)
        BIN=$(ls -t "$NS3DIR"/build/scratch/ns3*-msatnrvoice-v7 "$NS3DIR"/build/scratch/ns3*-msatnrvoice-v7-* 2>/dev/null | head -1)
        [ -z "$BIN" ] && BIN=$(ls -t "$NS3DIR"/build/scratch/ns3*-ntnlab_nr "$NS3DIR"/build/scratch/ns3*-ntnlab_nr-* 2>/dev/null | head -1)
    fi
    [ -n "$BIN" ] && [ -x "$BIN" ] || die "no built script in $NS3DIR/build/scratch (ls $NS3DIR/build/scratch | grep msatnr; then --bin FILE)"
    LIBDIR="$(dirname "$(dirname "$(dirname "$BIN")")")/build/lib"
    [ -d "$LIBDIR" ] || LIBDIR="$NS3DIR/build/lib"
    # v10.1 or later: it knows --hostJitter
    env LD_LIBRARY_PATH="$LIBDIR" "$BIN" --PrintHelp 2>/dev/null | grep -q -- '--hostJitter' ||
        die "$BIN is not v10.1 (no --hostJitter): install v10.1 and build first"

    {
        echo "sweep $(date)"
        echo "binary $BIN  md5 $(md5sum "$BIN" | cut -c1-32)"
        [ -f "$NS3DIR/scratch/msatnrvoice-v7.cc" ] && echo "script md5 $(md5sum "$NS3DIR/scratch/msatnrvoice-v7.cc" | cut -c1-32)"
        echo "users: $USERS"; echo "seeds: $SEEDS"; echo "runs: $RUNS"; echo "extra: $EXTRA"
        for i in "${!LABELS[@]}"; do echo "config ${LABELS[$i]}: ${OPTS[$i]}"; done
    } > "$OUT/sweep.txt"

    # one script per run, then all of them through xargs -P
    JOBLIST="$OUT/jobs.txt"; : > "$JOBLIST"
    total=0; skipped=0
    for i in "${!LABELS[@]}"; do
        for n in $USERS; do
            for seed in $SEEDS; do
                for run in $RUNS; do
                    d="$OUT/runs/${LABELS[$i]}/n$n/s${seed}_r$run"
                    if [ "$FORCE" = 0 ] && [ -f "$d/exit_code" ] && [ "$(cat "$d/exit_code")" = 0 ]; then
                        skipped=$((skipped + 1)); continue
                    fi
                    mkdir -p "$d"
                    cmd="\"$BIN\" ${OPTS[$i]} $EXTRA --nUes=$n --RngSeed=$seed --RngRun=$run --csv=out"
                    echo "$cmd" > "$d/cmd.txt"
                    cat > "$d/run.sh" <<EOF
#!/usr/bin/env bash
cd "$d" || exit 1
rm -f exit_code out_*.csv
start=\$(date +%s)
env LD_LIBRARY_PATH="$LIBDIR" $cmd > stdout.txt 2> stderr.txt
rc=\$?
echo \$rc > exit_code
echo "\$(date +%H:%M:%S) ${LABELS[$i]} n=$n seed=$seed run=$run exit \$rc (\$(( \$(date +%s) - start )) s)"
EOF
                    echo "$d/run.sh" >> "$JOBLIST"
                    total=$((total + 1))
                done
            done
        done
    done
    echo "run_sweep: ${#LABELS[@]} configurations x $(echo $USERS | wc -w) user counts x $(echo $SEEDS | wc -w) seeds x $(echo $RUNS | wc -w) runs: $total to run, $skipped already done, $JOBS at a time"
    echo "run_sweep: results in $OUT"
    if [ "$DRY" = 1 ]; then
        while read -r f; do cat "$(dirname "$f")/cmd.txt"; done < "$JOBLIST"
        exit 0
    fi
    if [ "$total" -gt 0 ]; then
        xargs -a "$JOBLIST" -P "$JOBS" -I{} bash {}
    fi
    failed=$(grep -L '^0$' "$OUT"/runs/*/n*/s*_r*/exit_code 2>/dev/null | wc -l)
    [ "$failed" -gt 0 ] && echo "run_sweep: $failed runs failed (see their stderr.txt):" &&
        grep -L '^0$' "$OUT"/runs/*/n*/s*_r*/exit_code | sed 's|/exit_code||'
fi

if [ "$PLOT" = 1 ]; then
    command -v python3 >/dev/null || die "python3 not found: plot later with  python3 plot_sweep.py $OUT"
    python3 -c 'import matplotlib' 2>/dev/null ||
        die "matplotlib missing (pip install matplotlib, or sudo apt install python3-matplotlib), then: python3 $SCRIPT_DIR/plot_sweep.py $OUT"
    python3 "$SCRIPT_DIR/plot_sweep.py" "$OUT" --pdb "$PDB"
fi
