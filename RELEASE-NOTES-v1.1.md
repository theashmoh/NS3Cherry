# ns-3-NTN v1.1 — release notes

Platform migration plus a link budget that is finally tied to a 3GPP reference.

## Platform

ns-3.45 + 5G-LENA v4.1  →  **ns-3.48 + 5G-LENA v5.1**.

Ships as a patch kit: clone the two upstream tags and run one script.

    git clone --branch ns-3.48 https://gitlab.com/nsnam/ns-3-dev.git ns-3
    git clone --branch v5.1 https://gitlab.com/cttc-lena/nr.git ns-3/contrib/nr
    ./ntn-patch.sh apply ns-3
    ./ntn-patch.sh test  ns-3

### Release gate

Verified 2026-09-23 against a fresh clone of stock `ns-3.48` and `v5.1`, not the
development worktrees, with the kit cut at `ns-3.48_NTN_dev` 2550b14102 and
`lena-5.1_NTN_dev` 84aaf7c7:

| step | result |
|---|---|
| `ntn-patch.sh check` | both patches apply cleanly |
| `ntn-patch.sh apply` | clean, no conflict markers, `ntn/` copied into the tree |
| `ntn-patch.sh test` | 26 of 26 pass with asserts on, none skipped: the 5 NTN suites at `--fullness=EXTENSIVE`, 11 other nr suites the kit adds or changes, and 10 stock ns-3 suites of the patched modules, `wifi-phy-ofdma`, `multi-model-spectrum-channel`, `leo-mobility` and `sionna-rt-channel-model` included. `test` finds a Sionna virtual environment (`SIONNA_PYTHON`, the active one, `./venv` or `~/sionna-venv`) and configures against it as in `ntn/SIONNA.md`; without one it skips the Sionna suite |
| `leo-link` builds | links against the patched tree |
| `leo-link` runs | default `sc9handset` preset, `--duration=2`: both `--payload=transparent` and `--payload=regenerative` deliver the full 10 Mbps offered load (1.75 MB), and every trace lands in the working directory |

## New features and changes

### Duplex: TDD becomes FDD

Both examples now run a paired FDD carrier: two bandwidth parts, `DL|DL` and
`UL|UL` patterns, a static bandwidth-part manager, and one channel per part. Every 3GPP NR NTN band is FDD (n255 L-band, n256 S-band, n254, Ka
n510-n512); TDD exists only for NB-IoT NTN. 

### Examples

Six examples collapse to two, both FDD following 3GPP TR38.821
(n255 L-band, n256 S-band, n254, Ka n510-n512; TDD exists only for NB-IoT NTN).

| example | what it is |
|---|---|
| `leo-link` | one satellite pass; `--payload=transparent` (default: gateway → satellite → UE, amplify-and-forward relay) or `--payload=regenerative` (gNB on board, service hop only) |
| `leo-link-regen-RT` | regenerative payload over a Sionna ray-traced channel |

Both payloads share one code path: the same link budget, placement, traces and
flags, so switching `--payload` changes the payload and nothing else. Every
satellite carries a gNB and each UE attaches to the closest one. UE 0 is placed
by `--observerLatDeg/--observerLonDeg`, or where the pass crosses
`--observerElevationBoundaryDeg` (`--observerMode=full-pass`), or under the
satellite (`overhead`); the gateway defaults to UE 0's position
(`--gatewayLatDeg/--gatewayLonDeg`).

Dropped: `leo-link-1hop-bent-pipe` and `leo-link-regen` (merged into
`leo-link`), the old `leo-link`, `leo-link-bent-pipe`, `leo-link-transparent`
and the LEO mobility copies, the last superseded by upstream ns-3.48. The
two-satellite path with an inter-satellite link is dropped for now and planned
for a later release; the feeder is always gateway → serving satellite.

The three v1.0.1 worked examples that need several UEs, blocked until multi-UE worked, are back
in `ntn/README.md`: Example 3, a full regenerative pass with four UEs (the harness gains
`ue_radii_km`, `beam_hpbw_deg` and `ue_noise_figure_db`); Example 4, the beam edge, UEs at 0, 25
and 50 km under an Airy beam (`run_ntn_beam_edge.sh`, new, since v1.0.1 shipped no driver); and
Example 5, RR, PF and MR with fresh and stale CSI for ten UEs (`run_ntn_sched10.sh`). All seven
README examples now run on the default link budget, TR 38.821 study case 9, with the study
case's Set-1 S-band beam (4.4127 deg) where a beam taper matters.

### Link budget follows TR 38.821

`--ntnPreset` selects a calibration study case, and **study case 9
(`sc9handset`) is the default**: LEO-600 seen at 30 degrees, S-band, 30 MHz, satellite EIRP density
34 dBW/MHz, and the Table 6.1.1.1-3 handheld: one element position with two
+/-45 deg cross-polarised ports (0 dBi, 7 dB noise figure), whose two receive
branches combine with no depolarisation loss. The gNB is one element position,
since the satellite's beam gain is already in its EIRP, and rank is 1. Table
6.1.3.3-1 gives a downlink CNR of 6.6 dB including a 3 dB shadowing margin that
a simulation does not subtract; the simulated median downlink SINR at 30 degrees
is 9.3 +/- 1.2 dB (regenerative) and 9.4 +/- 1.5 dB (transparent), 6 seeds each,
against 9.6 dB.

Every preset also sets the Set-1 satellite of Table 6.1.1.1-1: its antenna gain (30 dBi
S-band, 38.5 dBi Ka), its G/T as the gNB noise figure (1.1 dB/K = 4.28 dB, 13 dB/K = 0.88 dB)
and its 3 dB beamwidth for the spot-beam taper (4.4127°, 1.7647°), and the terminal's
transmit power (23 dBm handheld; the VSAT's 76.2 dBm EIRP). The satellite antenna had been
0 dBi, so the uplink lacked 30 dB of receive gain. The gNB now spreads its power over the whole
carrier: LENA's default puts it on the RBs in use, so a partly loaded OFDMA carrier transmitted
above the EIRP density (22.7 dB SINR at 90 degrees in the demo, against a 14.8 dB budget).

`--ntnPreset=sc14handset` and `sc15handset` are the same handheld under LEO-1200
(40 dBW/MHz, 30 or 10 MHz, DL CNR 7.2 dB); they also move the default orbit to
1200 km. `--ntnPreset=sc6vsat` selects the Ka-band VSAT reference. There is no preset-free
mode: the previous ad-hoc 3.5 GHz / 100 MHz settings left the control channel
below the out-of-sync threshold. Any preset value can still be overridden with an
explicit flag.

Note on the study cases: their 30 degree elevation is not printed in
Table 6.1.1.2. It is recovered from the FSPL column, since 159.1 dB at 2 GHz is
a 1075 km slant, which is 600 km seen at 30 degrees. Study case 14 checks out
the same way. The WNS3 2023 paper that reproduces these cases states 90 degrees,
which does not match its own FSPL figures.

### NTN timing

- K_offset wired into both examples, derived from the geometry each run visits.
- Ideal, geometry-derived timing advance (`--ntnTiming`).
- Random access survives physical propagation; `--idealRa` (default on) removes
  RA failure from experiments that are not about RA.
- RRC timers sized from the loop RTT rather than terrestrial defaults.

### Defaults that changed, and why

Old defaults made throughput numbers meaningless. All measured:

| default | was | now | effect |
|---|---|---|---|
| `useFixedMcs` | 1 (MCS 4) | 0 | link adaptation actually runs |
| `enableHarqReTx` | 0 | 1 | 48 % of TBs were lost and never retransmitted |
| `ollaStepUp` | 0 | 0.01 | corrects the ~17 ms stale-CQI bias |
| `ollaTargetBler` | 0.1 | 0.15 | 0.10 gives up more MCS than HARQ costs |
| `ollaMaxOffset` | 4 | 12 | 4 pinned the loop against its floor |
| `ueIsDualPolarized` | 0 | 1 | rank 1 put half the capacity out of reach; the handheld and `sc6vsat` presets also use a dual-polarised terminal, rank 1 |

Single UE, 100 MHz, 29.4 dB: **838.8 Mbps MAC, the rank-2 MCS-28 ceiling.**


### Radio channel

- Both payloads run on the standard `MultiModelSpectrumChannel`, so MIMO
  fading, the phased-array model and every stock channel fix apply to both.
  Three small NTN models plug into it:
  - `NrNtnPropagationDelayModel`: speed-of-light delay, plus the feeder link and
    the relay processing delay for a transparent payload. Replaces the v1.0.1
    `--regenRtt`, which only shortened the modelled delay without moving the
    radio path.
  - `NrNtnFeederNoiseLossModel` wraps the service-link loss for a transparent
    payload: downlink SNR is the amplify-and-forward cascade with the noise the
    relay forwards, following the live feeder geometry. On the uplink the feeder
    adds only delay, so the satellite's receiver sets the SNR, as TR 38.821
    treats the feeder's other impairments as negligible.
  - `NrNtnUeToUeFilter` drops UE → UE signals on FDD carriers, where no UE
    listens on the uplink and the 3GPP NTN models reject a ground-to-ground link.
- `NrBentPipeSpectrumChannel` is removed; the models above replace it.
- The satellite antennas track UE 0 in both payloads. The transparent payload
  did not before, so its SINR differs from earlier builds (see the link-budget
  numbers above).
- **Multi-UE works.** LENA's uplink receiver needs overlapping receptions to
  start at the same nanosecond. Over a moving satellite the UEs miss that by
  nanoseconds (timing-advance rounding and a path that changes within a slot),
  and a second UE aborted in `NrSpectrumPhy::StartRxUlCtrl`. A real gNB takes
  all UEs through one FFT window per symbol and its cyclic prefix absorbs such
  offsets, so the delay model lines each uplink arrival up on the gNB's symbol
  grid when it lands within one cyclic prefix of it; one further off keeps its
  physical delay. The UE's timing advance is also refreshed every slot from the
  current geometry (`NrUePhy::SetTimingAdvanceCallback`) instead of on a timer.
- `MultiModelSpectrumChannel::StartRx` drops a precoder that no longer fits the
  receiver's grid, as `StartTx` already did. A UE that lost the link re-grids
  for cell search while precoded frames are still in flight, which aborted the
  run with MIMO feedback on.

### Scheduler and link adaptation

- Downlink outer-loop link adaptation (OLLA), with `OllaStepUp`,
  `OllaTargetBler`, `OllaMaxOffset` and a `DlOllaUpdate` trace.
- Rel-17 `downlinkHARQ-FeedbackDisabled`.
- The HARQ process timeout gets its own attribute instead of riding on the
  process count.
- Cell-specific NTN scheduling offsets (K_offset) and delayed HARQ feedback.
- The HARQ reshape search is bounded, and empty reshape allocations guarded.

### HARQ soft buffer

- HARQ incremental redundancy can be bounded to a finite soft buffer,
  `NrEesmIr::SoftBufferBitsPerTb`, following NR's limited-buffer rate matching
  (TS 38.212 5.4.2.1): the buffer belongs to one transport block and has no
  HARQ-process term, unlike LTE's N_soft / H. `leo-link --lbrm` sizes it at
  1.5 x TBS_LBRM, the largest block the carrier supports; off, it is unlimited.

### RLC and PDCP

- `NrRlcAm` no longer counts PDUs awaiting acknowledgement as retransmission
  demand, which had inflated the buffer status report and made the scheduler
  over-grant and pad.
- `NrRlcAm` re-segments a retransmission that no longer fits its grant (TS
  36.322 5.2.1), and the receiver rebuilds the PDU from the segments, also when
  two retransmissions cut it at different offsets. A PDU is sized by the grant it
  was first sent in, so once the MCS dropped the retransmission either
  overflowed the grant and the MAC asserted (`--rlcAmRetxAlwaysFits=1`, the old
  default), or waited for a big enough grant that never came and the bearer
  stopped for good (the NTN demo at 30 degrees delivered nothing after 2 s).
  `--rlcAmRetxAlwaysFits` now defaults to off. Simplifications: each segment
  repeats the whole PDU's length indicators, a STATUS PDU NACKs a partly
  received PDU whole, and the segment offset's 15 bits limit re-segmentation to
  PDUs of 32 KB.

### Tests and tooling

- Five NTN suites: `nr-ntn-link-models`, `nr-ntn-propagation-timing`,
  `nr-ntn-scheduler` (ported), plus `nr-ntn-random-access` and
  `nr-ntn-timing-advance` (new). `nr-ntn-timing-advance` runs two moving UEs
  through the gNB and fails without the uplink line-up. `nr-test-koffset` covers the offsets.
- `ntn-patch.sh test` configures with asserts and logging on when it drives CMake directly (the
  fallback when `./ns3` fails, and always with Sionna): an explicit build type skipped ns-3's
  default, so the NS_ASSERT checks never ran in a fresh-clone gate before.
- MAC trace output takes a results folder and a simulation tag, so parallel runs
  stop overwriting each other.
- The analysis harness ships in the kit as `ntn/` and is copied into the ns-3
  tree by `ntn-patch.sh apply`. Its README carries a generated configuration
  reference, taken from `--PrintHelp` on the built binary so the defaults are
  the ones a run will actually use, and the harness scripts run on the default
  preset and override it with their own parameters.
- `leo-link-regen-RT` runs the regenerative example over a Sionna RT ray-traced
  channel, with `--sionnaRender` writing one scene image per channel update. It
  now runs end to end; it had never delivered data. Ships with a ray-tracing scene
  of the University of Washington campus (`ntn/data/uw-campus/`, 178 buildings from
  OpenStreetMap), built by the new `ntn/tools/osm_to_sionna.py`, which turns any
  set of OSM footprints into a Sionna scene.
- `leo-link` and `leo-link-regen-RT` take `--observerLatDeg/--observerLonDeg`
  and `--satLongitudeDeg/--satArgOfLatitudeDeg`, and `ntn/tools/aim_pass.py` solves
  the latter for a pass directly overhead any latitude and longitude, mirroring the
  ns-3 orbit model exactly. Previously a run's terminal was placed from the orbit,
  so no run was ever a pass over a chosen place.
  `ntn/scripts/rt-animate.py` matches those images to a metric trace by index
  and turns the pair into an animation of the pass.

- `nr-gnb-phy` dropped the cell's first random-access response on an uplink-only
  carrier whenever K_offset was large.
- A signal was matched to the receiver's frequency grid when it was transmitted.
  Over an NTN flight time the receiving PHY finishes configuring in between, so
  the signal arrived cut for a grid the receiver no longer had. This looked like
  an elevation limit below about 41 degrees; it is really propagation delay
  against PHY setup time. The standard channel re-matches on arrival.
- The channel computes a signal's delay once, when it is sent, so a closing
  satellite delivered the next transmission a few ns before the previous one
  ended, and any moving run aborted ("Cannot receive DATA while receiving
  CTRL"). The UE's PHY now compresses each downlink signal by how much the delay
  shrinks while it is sent, as an approaching transmitter really does (Doppler),
  from `NrNtnPropagationDelayModel::GetOneWayDelayRate` with the feeder link
  included. It rounds toward shorter, so the 1-2 ns gap the NR PHYs leave between
  back-to-back signals survives and no two ever tie at the same nanosecond. The
  uplink is lined up on the gNB's symbol grid instead. Only runs with
  `NrNtnPropagationDelayModel` take this path; ns-3 itself is unchanged.
- OLLA was applied on top of a fixed MCS, so `--fixedMcs=28` transmitted at a
  constant MCS 16 and looked like a hard modulation cap. `leo-link`
  now turns OLLA off when a fixed MCS is requested; the scheduler itself still
  applies both if a program enables them together.
- Two scheduler paths dropped HARQ feedback instead of carrying it to the next
  slot: the uplink loop stopped at a full slot, and the downlink returned early
  on a slot with no data symbols. With the timeout no longer reaping processes
  that received feedback, a dropped process leaked for good, and after
  NumHarqProcess of them the UE was never scheduled again in that direction.
- `NrRlcAm` did not report the buffer status when a STATUS PDU NACKed PDUs into
  the retransmission buffer, so every ARQ recovery waited up to 20 ms for the
  next report timer tick.
- The transparent example's `--receiverNfDb` set only the channel's noise level,
  not the PHY's, so the PHY SINR was off from the channel's by the difference
  (5 dB in the NTN demo). It now applies at every node without its own noise
  figure. Its `--rngRun` overrode the standard `--RngRun`, so a seed sweep over
  `--RngRun` gave identical runs; it is removed, use `--RngRun`.
- `leo-link-regen-RT` built whenever Eigen was found, so a full build failed on a
  machine without the Sionna Python packages; it now builds only with them.
- `subbandSize` was hard-coded at 32, which aborts on any carrier narrower than
  about 30 MHz (TS 38.214 Table 5.2.1.4-2).
- ns-3.48's `SionnaRtChannelModel` could not load a scene file: it called
  `rt.Scene(filename=...)`, which Sionna RT 1.x and later reject. Built-in scenes
  took a different path and hid it. Now uses `load_scene` for both. This one is in
  ns-3 itself, `src/spectrum/`, and worth sending upstream.
- `leo-link-regen-RT` fed Sionna two Earth-centred quantities as if they were in the
  scene's east/north/up frame: the satellite velocity, giving the wrong Doppler, and
  the satellite array orientation, which skewed the two halves of a pass by 6-11 dB.
  Both now agree with the scene; rising and setting match within 2 dB in clear sky.
- `ntn-sionna-cir.py --los-only` counted rays that pass through walls as line of
  sight, because Sionna traces refraction by default.
- ns-3.48's Sionna RT channel built a wrong channel matrix from Sionna's paths whenever
  either end had more than one antenna element. `CalculateCirFromPaths` read the
  coefficient array with the path index outermost, where Sionna has it innermost, and
  placed each coefficient on the wrong element: ns-3 numbers a `UniformPlanarArray`
  row-major from the bottom row, Sionna column-major from the top. Separately,
  `SionnaRtSpectrumPropagationLossModel` multiplied Sionna's Doppler, already in Hz,
  by f/c. Together these produced fast fading on a static terminal (SINR swinging
  5-8 dB from slot to slot) and made results depend on unrelated weak paths: turning
  on diffraction, which added paths 45 dB down, moved line-of-sight SINR by up to
  16 dB. Also fixed in the same two files: the carrier phase was applied twice (once
  by Sionna's `cir()`, again per sub-band at the absolute frequency); Doppler was
  rotated from simulation start rather than from when the channel was traced; a
  `±45°` dual-polarised array had its two polarisations swapped; and the path-count
  retry lowered the trace depth for every later link. All of these are in stock
  ns-3.48 `src/spectrum/`; worth sending upstream. The Sionna test now checks each
  coefficient's phase against its path's direction and ns-3's element positions; the
  stock code fails it.
- ns-3.48's `MultiModelSpectrumChannel` computes the phased-array gain when a signal
  arrives, with the transmitter's beam at that moment. Over a satellite's milliseconds of
  delay the gNB has moved on; 5G-LENA switches it to quasi-omni at every control symbol,
  so a data burst arriving during one was scored 22 dB low. The channel now keeps the
  beam the signal left on. It produced 20 dB dips of a few seconds in clear sky,
  coming and going as the delay drifted.
- `leo-link-regen-RT` used a 2x2 satellite array, whose 6 dB array gain came on top of an
  EIRP that already includes the antenna gain. It now uses one element per polarisation,
  so received power follows `--satEIRP`; its clear-sky SINR is 6 dB lower than before.
- `leo-link-regen-RT` moved its 10 km proxy at the satellite's full speed, which
  inflates the Doppler difference between paths by range/10 km. The proxy velocity
  is now scaled by 10 km/range. The terminal default is now co-polarised with the
  gNB (`--uePolSlantAngleDeg=0`): at 90° it was crossed, and a ray-traced direct path
  couples no power between crossed polarisations.
- `leo-link-regen-RT` gains `--sionnaDiffraction` (default on) and
  `--sionnaSamplesPerSource` (default 10^8), as `SionnaRtChannelModel` attributes
  `Diffraction` and `SamplesPerSource`. At Sionna's default 10^6 rays, diffraction
  off a nearby roof edge is found at some samples and missed at others.

## Known limitations

- **A multi-element gNB sends PDCCH on a quasi-omni beam.** 5G-LENA switches the gNB to a
  quasi-omni weighting for every control symbol, which no satellite does. With radio-link
  monitoring on control SINR, a 2x2 gNB fails the link while data still gets through.
  `leo-link-regen-RT` therefore uses one element per polarisation.
- **Ideal random access leaves UEs unconnected when many start together.** The gNB fits only a
  few Msg3 grants per slot (six at 10 MHz) and drops the rest; with `--idealRa` (the default)
  a UE whose preamble goes unanswered never retries. At the default 30 MHz ten UEs connect.
  `--idealRa=0` retries.
- **The uplink SINR drops by about 40 dB from t = 1 s to 2 s.** Both payloads, at the default
  `--precision=1000`; the downlink is unaffected and the uplink recovers at 2 s. The cause is not
  found yet. Leave the first 2 s out of uplink results.
- **The handheld uplink is spread over the whole carrier.** The scheduler grants the UE all
  30 MHz, so its 23 dBm arrives 19 dB weaker per hertz than in TR 38.821's uplink budget,
  which assumes 360 kHz (CNR 2.8 dB): about -11 dB SINR at 30 degrees. RLC AM runs keep
  `--perfectUplink`.
- **No inter-satellite link.** The feeder always runs from the gateway to the
  serving satellite; a feeder through another satellite over an ISL is planned
  for a later release.

