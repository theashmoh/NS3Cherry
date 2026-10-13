#!/usr/bin/env bash
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
# Scheduler comparison under CSI aging (README example 5): 10 UEs in one beam, saturated, 90 deg,
# RR / PF / MR x fresh (10-slot) / stale (160-slot) CSI x 3 seeds = 18 runs of 60 s.
# The default link budget, TR 38.821 study case 9, with its Set-1 S-band beam (4.4127 deg HPBW).
# UEs uniform in area over the cell out to the -4 dB contour, 26.4 km at 600 km. UE 0 at r=0 is
# the beam centre. CsiFeedbackFlags=6 (CSI_RS|CSI_IM) is required, or CsiRsPeriodicity is inert.
# The gNB fits only a few msg3 grants per slot, and with the ideal random access (--idealRa,
# default) a UE whose preamble goes unanswered never retries: 30 MHz fits all ten, 10 MHz six.
source "$(dirname -- "$0")/common.sh"
BIN="$(realpath "$(ntn_find_binary leo-link)")"
OUT=${1:-results/ntn-sched10}
JOBS=${NTN_JOBS:-1}   # runs at a time; each is one single-threaded process

COMMON_ARGS=(--payload=regenerative
  --enableHarqReTx=1 --numHarqProcess=32 --useFixedMcs=0 --precision=100
  --freezeGeometry=1 --observerMode=overhead
  --numUes=10 "--ueRadiiKm=0.0,8.8,12.4,15.2,17.6,19.7,21.6,23.3,24.9,26.4"
  "--ueAzimuthsDeg=0,36,72,108,144,180,216,252,288,324"
  --udpRateMbps=12 --duration=60 --ollaStepUp=0.001 --ollaTargetBler=0.1
  --ns3::NrHelper::CsiFeedbackFlags=6
  --ns3::NrMacSchedulerNs3::HarqProcessTimeout=100
  --ns3::NrAmc::ErrorModelType=ns3::NrEesmIrT1
  --ns3::NrSpectrumPhy::ErrorModelType=ns3::NrEesmIrT1)

for seed in 1 2 3; do
  for sched in RR PF MR; do
    for csi in 10 160; do
      d="$OUT/s$seed-${sched,,}-csi$csi"
      mkdir -p "$d"
      echo "== $d"
      # Run inside the run directory so every trace the example writes lands there.
      (cd "$d" && "$BIN" "${COMMON_ARGS[@]}" --RngRun="$seed" \
          --scheduler="ns3::NrMacSchedulerOfdma$sched" \
          --ns3::NrGnbPhy::CsiRsPeriodicity="$csi" >stdout.txt 2>&1) &
      while (($(jobs -rp | wc -l) >= JOBS)); do wait -n; done
    done
  done
done
wait
echo "done -> $OUT   (now: python3 plot_ntn_sched.py $OUT)"
