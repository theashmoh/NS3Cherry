> **Historical experiment, rerun pending.** The results quoted below were captured on the
> v4.1 fork with the old MAC RTT approximation (`--enableDynamicRtt`), which no longer exists.
> The commands have been rewritten for the ns-3.48 + 5G-LENA v5.1 `leo-link` example, which
> always applies physical propagation, runs FDD and sizes K_offset, the timing advance and the
> RRC timers from the geometry. They have not been rerun with complete physical timing. See
> [the implementation and test scope](../doc/leo-physical-timing.md) and
> `PLAN-next-release.md` at the repository root.

# LEO NTN HARQ / RTT / Scheduler test tutorial

How to reproduce the HARQ-pipeline, round-trip-time (RTT) and scheduler experiments with the
`leo-link` example in this directory.

## 1. The example

Every satellite of the orbit carries a gNB and each UE attaches to the closest one.
`--payload` picks the architecture:

| `--payload` | Architecture | HARQ loop path |
|---|---|---|
| `transparent` (default) | Amplify-and-forward relay: the waveform also crosses the feeder link, whose delay and forwarded noise `NrNtnPropagationDelayModel` and `NrNtnFeederNoiseLossModel` add | GW -> sat -> UE |
| `regenerative` | The gNB on the satellite terminates NR (stock 3GPP NTN channel) | sat -> UE |

It replaces `leo-link-regen` (now `--payload=regenerative`) and `leo-link-1hop-bent-pipe` (now
the default). A feeder through a second satellite and an ISL is not modelled in this release.

The cell is FDD, as every 3GPP NR NTN band is (n255: UL 1626.5-1660.5 / DL 1525-1559 MHz;
n256: UL 1980-2010 / DL 2170-2200 MHz; n254: UL 1610-1626.5 / DL 2483.5-2500 MHz). BWP 0 is
the downlink carrier at `--dlFrequencyHz`, BWP 1 the uplink carrier at `--ulFrequencyHz`, each
`--bandwidthHz` wide. The uplink defaults to max(190 MHz, `--bandwidthHz`) below the downlink.

## 2. Building

If `./ns3` cannot configure because an incompatible CMake shim is first on `PATH`, select a
working CMake executable explicitly. CMake writes build-system state to `cmake-cache/`, not
`build/`:

```bash
cd <ns-3-dev>
CMAKE=${CMAKE:-cmake}
"$CMAKE" -S . -B cmake-cache
"$CMAKE" --build cmake-cache --target leo-link -j16
```

The example needs Eigen (`ENABLE_EIGEN`): its default MIMO rank selection is not available
without it. Optimized binaries appear under `build/contrib/nr/examples/`; the filename includes
the ns-3 version and build profile. Run them with the library path exported, in the directory
the traces should go to:

```bash
export LD_LIBRARY_PATH=<ns-3-dev>/build/lib
BIN=$(realpath "$(find build/contrib/nr/examples -name 'ns3.*-leo-link-optimized' -print -quit)")
(cd results/run1 && $BIN <flags> > run.log 2>&1)
```

## 3. Key flags

| Flag | Meaning |
|---|---|
| `--numHarqProcess=N` | HARQ pipeline depth. With `--ntnTiming=1` (default) a process is kept for N + K_offset slots, so N sizes the number of blocks in flight, not the RTT; with `--ntnTiming=0` a process is reclaimed after N slots (N ms at mu=0) and soft-combining fails once the loop RTT exceeds N. |
| `--ntnTiming=0\|1` | Derive `NrGnbPhy::CellSpecificKoffset` (= ceil(worst-case loop RTT / 1 ms) over the simulated window), the per-UE ideal `NrUePhy::TimingAdvance` (one-way delay to the serving gNB, re-read every slot) and `NrGnbRrc::ConnectionRequestTimeoutDuration` from the geometry. `0` leaves all three at their defaults. The derived values are printed at startup and written to `NtnTimingConfig.txt` in the working directory. |
| `--idealRa=0\|1` | `NrUeMac::IdealRandomAccess`: never arm the RA response timeout, so attach cannot fail where the RAR window does not fit the round trip; also pins the connection-request timer to its 2000 ms ceiling. Default on. Msg1/Msg2 still cross the channel with real delay. |
| `--payload=transparent\|regenerative` | Radio path and link budget: feeder + service with forwarded relay noise, or the service hop alone. Everything else, geometry included, is identical, so the pair isolates the payload. |
| `--freezeGeometry=1` | Hold every satellite at its initial position: constant elevation, slant range and loop delay (the OLLA, HARQ and RLC demo rows). K_offset is then the true RTT rounded up to the millisecond. |
| `--observerMode=overhead\|full-pass` | Where UE 0 goes when `--observerLatDeg`/`--observerLonDeg` are unset: under the first satellite, or where it rises through `--observerElevationBoundaryDeg`. The run stops when the satellite descends through the boundary. `ntn/tools/aim_pass.py` prints `--observerLatDeg --observerLonDeg --satLongitudeDeg --satArgOfLatitudeDeg --duration` for a pass over a real place. |
| `--satEIRP=` | Satellite EIRP density in dBW/MHz -> sets the SINR. Lower it to push the link toward the error floor where combining matters. |
| `--gatewayLatDeg=` / `--gatewayLonDeg=` | Transparent feeder-link gateway; UE 0's position by default, which makes the feeder mirror the service link. |
| `--dlFrequencyHz=` / `--ulFrequencyHz=` / `--bandwidthHz=` | FDD carrier pair: DL centre (BWP 0), UL centre (BWP 1, `0` derives it) and the bandwidth of each carrier. |
| `--orbitFile=FILE` | Constellation in the ns-3.48 CSV format: a header line, then `altitudeKm,inclinationDegrees,numberOfPlanes,numberOfSatellitesPerPlane[,phasing,raanSpanDeg]` per row. Every satellite gets a gNB. |
| `--precision=MS` | Mobility resolution: the orbit model's CourseChange period, the trace cadence and the satellite antenna re-pointing. The satellite position itself is continuous in time. |
| `--useFixedMcs=1 --fixedMcs=M` | Disable link adaptation (fix the MCS) to isolate soft-combining from AMC compensation. Adaptive MCS with OLLA is the default. |
| `--ueRadiiKm=0,1,2,3` | UE radial positions (km from UE 0) for the spot-beam gain taper. |
| `--scheduler=ns3::NrMacSchedulerOfdma{RR\|PF}` | Round-robin or proportional-fair. |
| `--n1Delay` / `--n2Delay` | K1 (DL data -> HARQ feedback) and K2 (UL grant -> UL data) in slots, default 2, allowed 0-4. Not a propagation substitute. |

Removed with the merge into `leo-link`: `--regenPhy` (now `--payload`), `--twoSatellites`,
`--gtMaxElevationDeg`, `--gtMaxElevationTimeOffsetS`, `--ueInitialElevationDeg` and
`--uePlacementBearingDeg` (placement is by pass now), `--sat2TxEirpDbm` (now `--satEIRP` in
dBW/MHz), `--sat2RxAntennaGainDb` (now `--satAntennaGainDb`), `--gnbTxPowerDbm` and
`--sat1RxAntennaGainDb` (now `--gatewayTxPowerDbm` and `--satFeederRxGainDb`),
`--vsatAntennaGainDb` (now `--ueAntennaElementGainDbi`), `--groundTxPower` (now
`--ueTxPowerDbm`) and `--packetSize` (now `--udpPacketSize`). `--traceFile` is the
colon-separated satellite trace; the tab-separated link budget moved to
`--linkBudgetTraceFile`. Removed since the v4.1 fork: `--enableDynamicRtt`, `--regenRtt`,
`--usePropagationDelay`, `--frequencyHz`, `--pattern`, `--rlcAmPollPdu`, `--rlcAmPollByte`,
`--rlcAmWindow` and the two `--ulCtrl*` flags. The example aborts on an unknown argument.

## 4. A base command

Set a base, then override per experiment (3.5 GHz / 100 MHz transparent D2C; the uplink
carrier defaults to 3.31 GHz):

```bash
BIN=$(realpath "$(find build/contrib/nr/examples -name 'ns3.*-leo-link-optimized' -print -quit)")
BASE="--dlFrequencyHz=3500000000 --bandwidthHz=100000000 --duration=320 \
  --scenario=NTN-Rural --satEIRP=30 --relayLossDb=3.0 --receiverNfDb=0.0 \
  --enableHarqReTx=1 --numHarqProcess=32 \
  --enableMimoFeedback=1 --rankLimit=2 --fullSearchCb=ns3::NrCbTwoPort \
  --ueAntennaColumns=2 --ueNumHorizontalPorts=2 \
  --ueRadiiKm=0,1,2,3 --beamHpbwDeg=3.0 \
  --scheduler=ns3::NrMacSchedulerOfdmaPF --observerMode=overhead"
run(){ d=results/$1; shift; mkdir -p "$d"; (cd "$d" && $BIN $BASE "$@" \
       --linkBudgetTraceFile=log.txt --channelConditionTraceFile=cc.txt >run.log 2>&1); }
```

`--satEIRP=30` at 100 MHz is 80 dBm. `--ntnTiming=1 --idealRa=1` are the defaults and are part
of the base. Add `--ntnTiming=0 --idealRa=0 --ns3::NrGnbMac::RaResponseWindowSize=10` for the
arm without NTN timing.

## 5. Reproducing each experiment

### (a) HARQ pipeline-depth sweep
```bash
for H in 8 16 24 32; do run harq_h$H --numHarqProcess=$H; done
```
With `--ntnTiming=0`, soft-combining succeeds where the loop RTT (ms) is below the depth; deeper
pipelines combine down to lower elevation. With `--ntnTiming=1` the process lives H + K_offset
slots and H sets the concurrency ceiling `H * TBS / (K1 + K_offset + RTT)` instead; run both
arms to separate the two effects, and `--ns3::NrGnbPhy::CellSpecificKoffset=<n>` with
`--ntnTiming=0` to A/B K_offset alone.

### (b) Transparent vs regenerative RTT, same trajectory + matched SINR (the RTT-isolation test)
```bash
run bentpipe --numHarqProcess=32 --payload=transparent    # GW->sat->UE (2 hops)
run regen    --numHarqProcess=32 --payload=regenerative   # sat->UE (1 hop)
```
Identical geometry; only the radio path differs (transparent loop = feeder + service both ways,
regenerative = service hop both ways, so roughly half with the default gateway). The
regenerative link budget drops the feeder cascade, so match the SINR with `--satEIRP` before
comparing. Run at low EIRP (next item) for the reliability/latency effect. The printed
`Loop RTT at t=0` and K_offset show the halving directly.

### (c) Low-EIRP (push SINR to the floor so soft-combining matters)
```bash
run lowEIRP_h32 --numHarqProcess=32 --satEIRP=18
```

### (d) Fixed MCS (isolate combining from link adaptation)
```bash
run fixedmcs_h8 --numHarqProcess=8 --useFixedMcs=1 --fixedMcs=20
```
With the AMC off, the combining gain shows directly (deeper pipeline -> lower residual loss /
higher goodput).

### (e) Asymmetric feeder (gateway away from the UE)
```bash
run offtrack --observerMode=full-pass --observerElevationBoundaryDeg=10 \
   --gatewayLatDeg=<lat> --gatewayLonDeg=<lon>
```
A gateway away from UE 0 makes the feeder (GT->sat) and service (sat->UE) genuinely different
passes. Keep it where it sees the satellite for the whole window: the 3GPP NTN models reject
elevations below 10 deg. K_offset is sized for the worst-case loop RTT of the whole window (the
lowest elevation the pass visits), which is Rel-17's cell-specific behaviour; the
frozen-geometry rows do not pay that penalty.

### (f) Regenerative VSAT, scheduler RR vs PF (12 GHz / 240 MHz)
```bash
COMMON="--payload=regenerative --dlFrequencyHz=12000000000 --ulFrequencyHz=14000000000 \
  --bandwidthHz=240000000 --observerMode=full-pass --duration=280 --numHarqProcess=32 \
  --ueRadiiKm=0,30,30,50 --beamHpbwDeg=3.0"
$BIN $COMMON --satEIRP=25 --scheduler=ns3::NrMacSchedulerOfdmaRR ... > rr.log 2>&1
$BIN $COMMON --satEIRP=25 --scheduler=ns3::NrMacSchedulerOfdmaPF ... > pf.log 2>&1
```
Ku-band VSAT pairs a 14 GHz uplink with a 12 GHz downlink. Sweep `--satEIRP` (e.g.
15/20/25/35) for the power study. Note: a clustered UE layout (e.g. `0,10,10,50`) can trip a PF
lock-up at low EIRP; a spread layout (`0,30,30,50`) keeps PF fair.

## 6. Configuration reference: the NTN timing values

The example prints these at startup and writes them to `NtnTimingConfig.txt` in the working
directory (tab-separated `key value` lines):

| Key | Meaning |
|---|---|
| `architecture` | `transparent` or `regenerative` |
| `initialLoopRttMs` | Loop RTT at t = 0 from the first satellite: twice `NrNtnPropagationDelayModel::GetOneWayDelay()` (feeder + service both ways plus the relay processing, or the service hop only) |
| `sweepWindowEndS`, `minElevationDeg`, `maxLoopRttMs`, `maxLoopRttAtS` | The pass swept ahead of the run up to `--duration` or the elevation boundary, whichever comes first; the minimum elevation actually visited and the worst-case loop RTT with its time |
| `kOffsetSlots` | `NrGnbPhy::CellSpecificKoffset` = `ceil(maxLoopRttMs)` in 15 kHz reference slots (1 ms), 0 when `--ntnTiming=0`. Cell-specific and static (TS 38.213 cl. 4.2): sized once for the worst case of the window, not updated as the satellite moves; lena-1 scales it by 2^mu for the configured numerology |
| `connectionRequestTimeoutMs` | `NrGnbRrc::ConnectionRequestTimeoutDuration`: `max(15, maxLoopRttMs + 20)` with `--ntnTiming=1`, 2000 (the ceiling) with `--idealRa=1`, 15 (the default) otherwise |
| `ue <i> timingAdvanceUs` | `NrUePhy::TimingAdvance` of UE i at t = 0: its one-way delay to its serving gNB, 0 when `--ntnTiming=0`; re-read by the UE before every slot over a pass |

Expected values, 600 km orbit, one satellite: regenerative ~4 ms at zenith to ~13 ms at 10 deg;
transparent about twice that with the default gateway at UE 0 (the loop is about
`4 * slantRange / c`).

## 7. Notes / gotchas

- **Physical propagation is always on.** Control and data cross the channel with the geometric
  delay in both directions on both carriers; there is no MAC-side RTT injection any more.
- **Soft-combine rule (`--ntnTiming=0` only):** combining works iff loop RTT (ms) <
  `numHarqProcess` (at mu=0). The transparent RTT is ~2x the regenerative one (the extra feeder
  hop). With `--ntnTiming=1` the pool lives H + K_offset slots and no process is reaped by the
  RTT.
- **Under adaptive MCS, HARQ depth / RTT is roughly throughput-neutral** (the AMC picks a more
  robust MCS when combining is lost). The cost surfaces in residual block loss and latency, not
  throughput. To see the combining gain on throughput, use fixed MCS, item (d).
- **Matched-SINR comparisons:** calibrate `--satEIRP` so the end-to-end SINR matches, then vary
  only the quantity under study.
- **RLC AM with a weak uplink needs `--perfectUplink=1`:** STATUS PDUs ride the uplink, and when
  none decode the AM window stalls.
- **Steady-state runs must show an empty `NrUeRrc` RadioLinkFailure trace:** a low-elevation
  run at high BLER can drop and re-attach, and is then a re-attach measurement.
- `--linkBudgetTraceFile` carries geometry and per-link SNR columns of UE 0 through the first
  satellite (`SNR_sat1_db` feeder, `gamma_2_db` downlink, `SNR_UE_db`/`gamma_UE_cascade_db`
  end-to-end, `radio_one_way_delay_ms`, tab-separated with a header); `--traceFile` writes the
  GSoC-style colon-separated satellite trace. `RxPacketTrace.txt` carries per-TB
  `rv`/`corrupt`/`mcs`/`sinr_db` used for retx, TBLER, soft-combine and throughput.
