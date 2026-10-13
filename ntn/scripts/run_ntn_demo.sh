#!/usr/bin/env bash
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
# Author: Jesse Chiu (Kuan-Po) <kchiu5@uw.edu>, University of Washington, Seattle
# Minimal NTN teaching demo on leo-link (transparent payload).
#   Part A  ILLA (CQI inner loop only) vs OLLA (outer loop on HARQ ACK/NACK), elev 30/60/90
#   Part B  RLC UM vs AM at the same elevations -> what ARQ does over an NTN RTT
#   Part C  the two OLLA knobs (target BLER t, step size s) at 30 deg
# 19 runs of 20 s: 7 for part A (ILLA/OLLA per elevation, plus ollaslow-e30), 9 for part B
# (20 Mbps AM + a 10 Mbps UM/AM pair per elevation), 3 for part C.
source "$(dirname -- "$0")/common.sh"

BIN="$(realpath "$(ntn_find_binary leo-link)")"
OUT=${1:-results/ntn-demo}
JOBS=${NTN_JOBS:-1}   # runs at a time; each is one single-threaded process
STEP=${OLLA_STEP:-0.01}   # ollaStepUp; 0.001 is the realistic value, 0.01 converges inside 20 s

# The default link budget, TR 38.821 study case 9 (the sc9handset preset: S-band, 30 MHz,
# handheld). Frozen geometry so elevation is the only variable. A single UE carries 18 Mbps at
# 30 deg, 64 at 60 deg and 68 at 90 deg, so 20 Mbps saturates the 30 deg link and 10 Mbps is half.
# EESM-IR on BOTH NrAmc and NrSpectrumPhy - the LTE-MI default caps at 64QAM and has no IR buffer.
COMMON_ARGS=(--enableHarqReTx=1 --numHarqProcess=32
  --useFixedMcs=0 --precision=100 --freezeGeometry=1
  --numUes=1 --ueRadiiKm=0 --udpRateMbps=20 --duration=20 --RngRun=1
  --scheduler=ns3::NrMacSchedulerOfdmaRR
  --ns3::NrMacSchedulerNs3::HarqProcessTimeout=100
  --ns3::NrAmc::ErrorModelType=ns3::NrEesmIrT1
  --ns3::NrSpectrumPhy::ErrorModelType=ns3::NrEesmIrT1)

run() {  # run <tag> <extra flags...>
  local d="$OUT/$1"; shift
  mkdir -p "$d"
  echo "== $d"
  # Run inside the tag directory so every trace the example writes lands there.
  (cd "$d" && "$BIN" "${COMMON_ARGS[@]}" "$@" --linkBudgetTraceFile=log.txt \
      --channelConditionTraceFile=cc.txt >stdout.txt 2>&1) &
  while (($(jobs -rp | wc -l) >= JOBS)); do wait -n; done
}

# UE 0 where the satellite stands at e degrees: overhead at 90, else where it rises through e.
elev() { [ "$1" = 90 ] && echo --observerMode=overhead ||
    echo "--observerMode=full-pass --observerElevationBoundaryDeg=$1"; }

for e in 30 60 90; do
  # Part A. ollaStepUp=0 is ILLA: MCS from CQI alone, no correction for a biased report.
  run "illa-e$e" $(elev "$e") --ollaStepUp=0
  run "olla-e$e" $(elev "$e") --ollaStepUp="$STEP" --ollaTargetBler=0.1
  # Same loop with the realistic step. 0.001 reaches the same BLER with a far smoother offset;
  # 0.01 sawtooths +-3 MCS within a second. Only at 30 deg, where the CQI bias is largest.
  [ "$e" = 30 ] && run "ollaslow-e$e" $(elev "$e") --ollaStepUp=0.001 \
      --ollaTargetBler=0.1
  # Part B. AM on top of the OLLA run. perfectUplink because the handheld uplink is too weak for
  # its STATUS PDUs to decode here - AM would then stall for a reason that is not ARQ.
    run "am-e$e" $(elev "$e") --ollaStepUp="$STEP" --ollaTargetBler=0.1 \
      --rlcMode=am --perfectUplink=1
  # Same pair at half load. At 20 Mbps the 30 deg link has no headroom, so AM's overhead turns
  # into a growing queue and you measure congestion, not ARQ. 10 Mbps separates the two.
  run "lo-um-e$e" $(elev "$e") --ollaStepUp="$STEP" --ollaTargetBler=0.1 \
      --rlcMode=um --udpRateMbps=10
  run "lo-am-e$e" $(elev "$e") --ollaStepUp="$STEP" --ollaTargetBler=0.1 \
      --rlcMode=am --perfectUplink=1 --udpRateMbps=10
  # Part C. The two OLLA knobs, at 30 deg only -- that is where the CQI bias is large enough for
  # the setting to matter. t is the target the loop converges to; s is how far one ACK/NACK moves
  # the offset. Paired against ollaslow-e30 (t=0.10, s=0.001), which is the reference for both.
  if [ "$e" = 30 ]; then
    run "t005-e$e" $(elev "$e") --ollaStepUp=0.001 --ollaTargetBler=0.05
    run "t025-e$e" $(elev "$e") --ollaStepUp=0.001 --ollaTargetBler=0.25
    run "s05-e$e"  $(elev "$e") --ollaStepUp=0.05  --ollaTargetBler=0.1
  fi
done

wait
echo "done -> $OUT   (now: python3 summarize_ntn_demo.py $OUT)"
