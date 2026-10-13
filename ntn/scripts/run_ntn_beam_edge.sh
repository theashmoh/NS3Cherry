#!/usr/bin/env bash
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
# Beam edge (README example 4): UEs at 0, 25 and 50 km from the centre of an Airy spot beam
# (TR 38.811 6.4.1), regenerative, on the default link budget (TR 38.821 study case 9), one frozen
# run per elevation. The beam is the study case's: Set-1 S-band, 4.4127 deg HPBW, 50 km across
# at nadir. At zenith 25 km is its -3.6 dB edge and 50 km sits at -21.7 dB near the first null,
# outside any operating region, to make the beam pattern visible.
source "$(dirname -- "$0")/common.sh"
BIN="$(realpath "$(ntn_find_binary leo-link)")"
OUT=${1:-results/ntn-beam-edge}
ELEVS=${ELEVS:-"20 30 40 50 60 70 80 90"}
JOBS=${NTN_JOBS:-1}   # runs at a time; each is one single-threaded process

COMMON_ARGS=(--payload=regenerative --numUes=3 --ueRadiiKm=0,25,50
  --enableHarqReTx=1 --numHarqProcess=32 --useFixedMcs=0 --freezeGeometry=1
  --duration=3 --RngRun=1)

for e in $ELEVS; do
  d="$OUT/e$e"
  mkdir -p "$d"
  echo "== $d"
  # UE 0 where the satellite stands at e degrees: overhead at 90, else where it rises through e.
  if [ "$e" = 90 ]; then place=(--observerMode=overhead); else
    place=(--observerMode=full-pass --observerElevationBoundaryDeg="$e"); fi
  (cd "$d" && "$BIN" "${COMMON_ARGS[@]}" "${place[@]}" >stdout.txt 2>&1) &
  while (($(jobs -rp | wc -l) >= JOBS)); do wait -n; done
done
wait
echo "done -> $OUT   (now: python3 plot_ntn_beam_edge.py $OUT)"
