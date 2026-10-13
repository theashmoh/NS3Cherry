# PDCCH, SPS and configured grant in NR: what 3GPP says and what the model does

Read from the Release 18 specifications on 3gpp.org: TS 38.211 V18.10.0, 38.212 V18.8.0,
38.213 V18.9.0, 38.214 V18.11.0, 38.321 V18.10.0, 38.331 V18.11.0, 38.300 V18.11.0 and
38.133 V18.15.0. "Model" is the code that comes with this file: class `NrPdcch`, the scheduler
base class `NrMacSchedulerNs3`, `NrUePhy`, `NrUeMac` and the example `ntnlab_nr.cc`.

## 1. What a PDCCH is made of

| Item | 3GPP | Model |
|---|---|---|
| CORESET | A set of resource blocks, given as a bitmap of groups of 6 consecutive RBs, over 1, 2 or 3 symbols (38.211 7.3.2.2, 38.331 `ControlResourceSet`). 3 symbols only with `dmrs-TypeA-Position = pos3`. Up to 3 CORESETs per BWP (38.213 10.1). | One CORESET: every whole group of 6 RBs of the downlink carrier (`CoresetRbs` to make it smaller), over `Symbols` = 1, 2 or 3. |
| REG, CCE | REG = 1 RB in 1 symbol. CCE = 6 REGs (38.211 7.3.2.2). | N_CCE = RBs x symbols / 6. 5 MHz (24 of 26 RBs): 4, 8, 12 CCEs. 30 MHz (156 of 160 RBs): 26, 52, 78. |
| Aggregation level | 1, 2, 4, 8 or 16 CCEs (38.211 Table 7.3.2.1-1). | The same five. |
| Modulation, coding | QPSK, polar code, 24-bit CRC scrambled with the RNTI (38.211 7.3.2.4, 38.212 7.3.2 to 7.3.4). 3 of the 12 REs of a REG are DM-RS, so a CCE has 54 QPSK symbols = 108 coded bits. | Code rate = (DCI bits + 24) / (108 x level). A DCI that does not fit a level at rate 1 cannot use it. No bit-level coding: a block error rate from a table (section 4). |
| DCI size | Set by the configuration, not chosen per DCI. Format 1_0 with C-RNTI is 28 bits plus the frequency allocation field: 37 bits for 24 or 25 RBs, 42 for 160 RBs (38.212 7.3.1.2.1). Formats 0_0 and 1_0 are padded to one size (7.3.1.0); a UE monitors at most 4 sizes, 3 with C-RNTI (38.213 10.1). At least 12 bits. | One size for every DCI: `DciBits`, default 34 as asked. (34 bits is what format 1_0 has for up to 10 RBs.) |
| CCE-to-REG mapping | Interleaved or not, REG bundles (38.211 7.3.2.2). | Not modelled: it moves a CCE in frequency, it does not change how many there are. |

## 2. Where a UE looks: search spaces

- A search space set gives the number of candidates per aggregation level, 0 to 8 each
  (38.331 `SearchSpace.nrofCandidates`), in a CORESET, in common (CSS) or UE-specific (USS) use.
- The CCEs of candidate m at level L in slot n (38.213 10.1):
  `L x { (Y + floor(m x N_CCE / (L x M)) + n_CI) mod floor(N_CCE / L) } + i`, i = 0 .. L-1.
  CSS: Y = 0. USS: Y_n = (A x Y_(n-1)) mod 65537, Y_(-1) = C-RNTI, A = 39827, 39829 or 39839 for
  CORESET index mod 3 = 0, 1, 2. A DCI with CS-RNTI (SPS, configured grant) uses the same
  candidates: "the RNTI value used for n_RNTI is the C-RNTI".
- Limits per slot at 15 kHz: 44 monitored candidates, 56 non-overlapped CCEs (Tables 10.1-2
  and 10.1-3).
- The common search space of SIB1 has 4, 2, 1 candidates at level 4, 8, 16 (Table 10.1-1).

Model: this hash, exactly, with the slot number of the frame. Candidates 6/6/4/2/1 at level
1/2/4/8/16 by default (`Candidates`; 19, and 26 with the common ones, under the limit of 44).
A DCI takes the first candidate of its level whose CCEs are all free; if there is none, the
next higher level (`RaiseAl`); if none at all it is **blocked**: not sent, its UE not scheduled
in that slot.
Common DCIs use Y = 0 at level 8, or the highest that fits the CORESET (`CommonAl`).

## 3. What shares the PDCCH

38.300 5.2.3 lists what a PDCCH carries: downlink assignments and uplink grants, and activation
and deactivation of SPS and configured grants; slot format; pre-emption; power control commands
for PUCCH, PUSCH and SRS; BWP switching; a random access order; wake-up; and a few more. The
DCIs that schedule something on a shared channel are these:

| DCI | RNTI, search space | In the model |
|---|---|---|
| Downlink assignment, new data and retransmission (1_0, 1_1) | C-RNTI, USS | yes, one per transport block |
| Uplink grant, new data and retransmission (0_0, 0_1) | C-RNTI, USS | yes, one per transport block; in the same PDCCH slot as the downlink assignments |
| SPS / configured grant activation and release, and their retransmissions | CS-RNTI, USS | yes (section 5) |
| SIB1 (1_0) | SI-RNTI, Type0 CSS | yes: one DCI every 20 ms, the default repetition of SIB1 (38.331 5.2.1) |
| Random access response, Msg3 retransmission, Msg4 | RA-RNTI, TC-RNTI, Type1 CSS | response and Msg3 retransmission yes; Msg4 goes as a normal downlink assignment |
| Paging, other system information | P-RNTI, SI-RNTI, CSS | no: the example has neither |
| Group-common: slot format, pre-emption, power control (2_x) | CSS | no |

There is no rule in 3GPP for how a gNB divides the CCEs between uplink and downlink: both
kinds of DCI are in the same search space, and the scheduler decides. The model's order in a
slot: SIB1, random access, uplink grants, downlink assignments; in each direction
retransmissions, then SPS commands, then new data in the order of the scheduler's policy
(round robin, and between UEs a policy ranks the same: the UE served longest ago first). So that
uplink grants cannot starve the downlink, they take the CCEs the downlink did not ask for in
the slot before, and beyond these on average at most `UlMaxShare` (half) of the CORESET per slot.

## 4. Aggregation level, block error rate, radio link monitoring

3GPP leaves link adaptation of the PDCCH to the gNB. The model does what was asked: the UE
reports the SINR of its control symbols (every 10 ms, 0.5 dB steps from -23 to 40 dB, the L1-SINR
format of 38.214 5.2.1.4), and a DCI gets the lowest level with a block error rate of at most
`TargetBler` (1 %) at that SINR.

The table given (CNR -4, -2, -1, 0, 0.5, 1 dB: BLER 0.9303, 0.4356, 0.1453, 0.0246, 0.00797,
0.00221) is taken as level 1 with a 34-bit DCI. For level L the curve moves 10 log10(L) dB
down (L times the energy per information bit; the small extra coding gain of the longer polar
code is left out, which is on the safe side). For another DCI size it moves
10 log10((bits + 24) / (34 + 24)) dB. Between and beyond the points the logarithm of the BLER is
linear in dB. This gives 1 % at 0.40, -2.61, -5.62, -8.63 and -11.64 dB for level 1 to 16.
`SinrOffsetDb` shifts the simulator's SINR onto the CNR axis of the table if they differ.

With control errors on (`--idealControl=0`) a DCI is lost with the BLER of its level at the SINR
it arrives with. Radio link monitoring uses the two hypothetical PDCCHs of 38.133 8.1.2
(Tables 8.1.2.1-1 and -2): out of sync when a level-8 PDCCH with 4 dB more power would lose
over 10 %, in sync when a level-4 PDCCH would lose under 2 %. From the table these are -13.8 dB
and -5.9 dB.

## 5. How many symbols the PDCCH takes

In NR the duration of a CORESET is an RRC parameter; there is no per-slot indicator as in LTE.
What can change per slot is where the PDSCH starts (the time allocation in each DCI, 38.214
5.1.2.1, start symbol 0 to 3), and a PDSCH may use the CORESET resources that its own PDCCH
does not occupy (38.214 5.1.4). `Symbols` = 1, 2 or 3 is the 3GPP case. `Symbols` = 0 ("dynamic")
is an abstraction of a gNB that has CORESETs of 1, 2 and 3 symbols and uses the smallest that
holds the DCIs of the slot: the DCIs are placed in the 1-symbol CORESET, if they do not all fit
in the 2-symbol one, then the 3-symbol one, and the data starts after it. A UE then monitors
three times the candidates, 78, which is above the limit of 44: with the default candidates
this mode is optimistic about blocking.

## 6. Semi-persistent scheduling (downlink)

- RRC gives the periodicity (10, 20, 32, 40, 64, 80, 128, 160, 320, 640 ms), up to 8
  configurations per BWP; a DCI with CS-RNTI activates or releases one (38.321 5.8.1, 38.331
  `SPS-Config`).
- The activation DCI carries the allocation and MCS, schedules the first PDSCH, and the
  assignment recurs from there every period (38.321 5.3.1, 5.8.1). A second activation of an
  active configuration re-initialises it: new MCS or resources cost **one** DCI.
- The release DCI carries no allocation (MCS and frequency allocation all ones), and the UE
  acknowledges it with HARQ-ACK (38.213 10.2). The periodicity belongs to the configuration, so
  a change of periodicity is a release of one configuration and an activation of another:
  **two** DCIs. With several configurations the HARQ process number field of the DCI names the
  configuration (`sps-ConfigIndex`, 38.213 10.2). This is scenario 2 as described.
- SPS covers first transmissions only; a retransmission has a DCI (CS-RNTI, NDI = 1). A dynamic
  assignment that overlaps a configured one overrides it (38.321 5.3.1).

Model, both scenarios: the block holds one voice packet in the steady state of ROHC
(`SpsTbBytes`). An occasion with nothing to send is empty. A UE that missed the activation DCI
does not know the blocks; the gNB sees that no HARQ feedback comes and activates again. A
missed release is sent again. A configuration whose MCS no longer fits the link by more than
`SpsMcsTolerance` steps is activated again (one DCI). If the link is worse and this cannot be
done where the configuration is (the larger block has no room in its slots, or the PDCCH of the
slot has no free candidate), its blocks would fail: it is left unused, and a later packet
activates it again in a slot that has both. An activation re-initialises a configuration
(38.321 5.8.1), so this takes no release.

| Transition | Scenario 1 (one configuration, kept) | Scenario 2 (talk 20 ms, silence 160 ms) |
|---|---|---|
| Talk to silence: first packet is UOR-2-EXT-3 + SID, larger than the block | no DCI: sent in pieces over two occasions; then 7 of 8 occasions are empty | 2 DCIs: release, activation of the silence configuration with a block that holds the packet |
| Silence to talk: first packet is UOR-2-EXT-3 + voice, larger than the block | 1 DCI: the packet gets a dynamic assignment, the occasion is skipped | 2 DCIs: release, activation of the talk configuration with a block that holds the packet |

In scenario 2 the block of a configuration keeps the size it was activated with, so after a
change it is up to `SpsFitExtraBytes` (10) larger than a steady packet needs.

The run counts every change of every speaker in the table VOICE <-> SILENCE: how the first
packet after it was sent (whole in a block; in pieces, with the number of RLC PDUs; with a DCI
of its own), whether the configuration was changed, and the DCIs this took. In the default
downlink run (10 calls, 20 s) it reads: scenario 1, 45 changes to silence, all in pieces (90 RLC
PDUs, no DCI), 40 changes to voice, all with a DCI of their own (40 DCIs); scenario 2, the same
85 changes, each with a release and an activation (170 DCIs).

## 7. Configured grant Type 2 (uplink)

The same mechanism (38.321 5.8.2, 5.4.1; up to 12 configurations per BWP; periodicities include
20 and 160 ms), with these differences:

- The UE confirms every activation and release with a MAC CE in its next new transmission.
  With **one** configuration this is the Configured Grant Confirmation MAC CE: a subheader
  only, **1 byte** (38.321 6.1.3.7, "fixed size of zero bits"). With configurations given by
  `configuredGrantConfigToAddModList`, which two periodicities need, it is the Multiple Entry
  Configured Grant Confirmation MAC CE: 2-byte subheader and 4 bytes, **6 bytes** (6.1.3.31).
  One such CE reports all configurations, so it confirms a release and the activation that came
  with it. The model uses 1 byte in scenario 1 and 6 bytes in scenario 2.
- A UE with nothing to send does not transmit in an occasion (38.321 5.4.3.1.3).
- The gNB does not see the UE's packets. In the model it learns of a talk spurt by the
  scheduling request, answers it with a DCI (the first packet), and starts the configuration so
  that its blocks come 2 ms after each following packet is due; it tells talk from silence by
  the size of the packets it has received; it learns that a block was too small from the buffer
  status report in it. A silence-to-talk change in scenario 2 therefore costs 3 DCIs in uplink.
- Scenario 1 in uplink. Talk to silence: the UE sends a first piece of the SID in its block
  with a buffer status report; the gNB takes what is left for the rest of a SID and leaves it to
  the next block: two RLC PDUs, no DCI. Silence to talk: the UE sends a first piece of the voice
  frame in the block of the occasion; the gNB sees data left while the speaker was silent and
  gives one grant with a DCI for the rest ("a dynamic grant in addition to the configured one").
- Scenario 2 in uplink. Talk to silence: the first SID fits the talk block, which is larger; one
  silence period later, when the next SID is due, a release and an activation put the silence
  configuration in place (none if the speaker talks again before that). Silence to talk: the
  grant for the scheduling request carries the first frame, a release and an activation follow.
- The block is larger than in downlink by the buffer status report (5 bytes in this nr module;
  2 in 3GPP) and the confirmation CE.
- A UE has one transmit power for all its resource blocks, so the SINR per resource block that
  the gNB measures depends on how wide the grant was. With TDMA, or OFDMA with
  `--ulGrantRbs=0`, a block takes the whole band, and its MCS is the one that power gives over
  the whole band. With OFDMA and `--ulGrantRbs` (2 RB = 360 kHz, the default) a block is that
  many resource blocks over all data symbols of the slot, with the lowest MCS (up to that of the
  link) that holds its bytes, and the new data and retransmissions of the other UEs share those
  symbols in the resource blocks it leaves; a UE with a block in a slot retransmits in a later
  slot (one PUSCH per UE and slot). Either way the MCS is worked out from the last transmission
  of the UE whatever its width.
- A configuration that has to move to other slots (see section 6) cannot be left unused, the UE
  would go on sending in it: it is activated again in the first later slot that has room and a
  PDCCH candidate, and released first if it has no room where it is.

## 8. What the model leaves out

- Bit-level DCI contents, DCI formats and their size alignment; CCE-to-REG interleaving;
  PDCCH DM-RS; power boosting of the PDCCH; beam (TCI) of the CORESET.
- Paging, other system information, group-common DCIs, PDCCH orders.
- The limits on blind decodes and non-overlapped CCEs are printed, not enforced.
- The fixed HARQ process numbers of SPS and configured grants (38.321 5.3.1, 5.4.1): the model
  takes any free process. `configuredGrantTimer` and repetitions.
- The resources of a block are placed anew in every slot (after the PDCCH symbols in
  downlink, before the PUCCH symbol in uplink) rather than kept at one place; their amount is
  kept. A TDMA scheduler gives a block whole symbols, as it does with every transport block, and
  pads it; so does the uplink of OFDMA with `--ulGrantRbs=0`. With `--ulGrantRbs` (the default)
  an OFDMA uplink block is that many resource blocks over the data symbols, shared with the
  other UEs (see section 7).
- A UE may get a block and a retransmission in the same slot. More than one PDSCH or PUSCH per
  slot is an optional UE capability in 3GPP.
- Uplink control (PUCCH) stays ideal, as in the nr module.
- PUSCH repetitions (38.214 6.1.2.1; Rel-17 NTN coverage enhancements) and TBoMS: a UE whose
  link cannot carry a voice frame at MCS 0 loses most of its uplink. With 2 RB (`--ulGrantRbs`)
  and the -5 dBi handheld that is every UE, as the nr module grants no uplink block under 12 B.
  Uplink power control is open loop only (`--ulPowerRefDeg`), no closed loop (TPC).

## 9. What to say when the results are shown

These are the assumptions the numbers stand on. Each can be changed, most by an option. The
counts of DCIs in this table were measured with the handheld antenna at 0 dBi, ns-3's
ionospheric scintillation (6.22 / f^1.5 dB: 2.2 dB at 2 GHz, 1.94 dB at 2.175 GHz), a shadow
fading draw per UE, NTN-Suburban and 26 RB on 5 MHz. The defaults are now -5 dBi
(`--ueAntennaGainDbi`), TR 38.821's link budget (3 dB shadowing, 2.2 dB scintillation, 0.1 dB
gases, no UE blocked, no fast fading) and 25 RB: the SINR of a UE in line of sight is about
8.4 dB lower than in those runs (-5 dB antenna, -3 dB shadowing, -0.1 dB gases at 30 deg,
-0.2 to -0.3 dB EIRP density; at 2.175 GHz another -0.26 dB of scintillation), and DCIs need
higher aggregation levels than the figures here show.

| Assumption | 3GPP | What it does to the results |
|---|---|---|
| One beam, no other cell: the SINR is a signal-to-noise ratio. The header prints the CNR of UE 0 in line of sight; with the defaults (2 GHz, -5 dBi, TR 38.821's margins): 6.67 / 1.60 / -3.49 dB at 90 / 30 / 10 deg (downlink), 3.69 / -1.38 / -6.47 dB on 2 RB (uplink, 1.81 GHz). With `--ueAntennaGainDbi=0`: 6.60 dB at 30 deg | TR 38.821 Table 6.1.1.2-1, LEO-600 S-band handheld (0 dBi): downlink geometry SINR -3.1 / -1.1 / 1.0 dB (5 / 50 / 95 %) with frequency reuse 1, 7.3 / 8.2 / 8.5 dB with reuse 3. Table 6.1.3.3-1, study case 9: CNR 6.6 dB at 30 deg (0 dBi, 2 GHz, with margins: next row) | With reuse 1 most DCIs would need level 2 to 8 instead of 1: two to eight times the CCEs. `--ueAntennaGainDbi` and `--satEirpDbwPerMHz` move the SINR. |
| The four losses of the service link (TR 38.811 6.6, system level), each either TR 38.821's link budget (the default) or a model. Free space: always. Blocking: `--scenario=NTN-LOS` (no UE ever blocked) or NTN-Rural, -Suburban, -Urban, -DenseUrban (blocked by the LOS probability of 6.6.1, then 16.3 to 19.5 dB of clutter loss in rural; nearest 10 deg row). Shadowing: `--shadowing=fixed` (`--shadowingDb`, 3 dB, every UE) or `table` (a draw per UE: sigma 0.72 to 1.79 dB in rural line of sight, 8.8 to 11.5 dB blocked). Atmosphere: `--atmosphere=table` (0.1 + 2.2 dB) or `location` (next rows). Fast fading: `--fastFading=fixed` (`--fastFadingDb`, 0 dB) or `model` (TR 38.901 7.5 with TR 38.811's tables) | TR 38.821's link budget (Table 6.1.3.3-1, S-band) has margins: 3 dB shadowing, 2.2 dB scintillation, 0.1 dB gases, 0 dB polarization, 0 dB additional losses, and no fast fading. Its system-level calibration (Table 6.1.1.1-5) takes TR 38.811's model, line of sight for every UE, gases considered, no ionospheric scintillation (note 2: 20 to 60 deg latitude) | With the defaults no UE is blocked and none fades: UEs differ by free space and beam gain only. The 3 dB margin is a pessimistic draw: with `--shadowing=table`, more than 3 dB of shadow loss happens to 4.7 % of line-of-sight UEs at 10 deg, 0.4 % at 20-30 deg, under 0.002 % at 80-90 deg. `--fastFading=model` adds the many paths (rural line of sight: K-factor 24.7 dB at 10 deg, 12.3 dB at 20 deg, 3.6 dB overhead). `--linkMarginDb` is the budget's additional losses. |
| `--atmosphere=location`: ionospheric scintillation by the UEs' magnetic latitude `--latitudeDeg` (TR 38.811 6.6.6.1.4, system-level): A_IS = 2.2 dB at 2 GHz x (f / 2 GHz)^-1.5 within 20 deg of the magnetic equator, none from 20 to 60 deg (default 45 deg); above 60 deg the run stops | TR 38.821 Table 6.1.3.2-1 note 1: 2.2 dB, the P3 curve at 1 % of the time; TR 38.811: "not observed at mid-latitudes except for strong geomagnetic storms", and above 60 deg its model "is not applicable" | ns-3 takes the 2.2 dB everywhere, always; the run takes it back out and puts in what the options say. At 45 deg latitude every SINR at 30 deg is 2.23 dB higher than with `--atmosphere=table` (2.175 GHz: 2.2 + 0.1 dB against 0.07 dB of gases). |
| `--atmosphere=location`: atmospheric gases at every elevation, A_zenith(f) / sin(elevation) with ns-3's A_zenith (0.035 dB at 2 GHz): 0.035 / 0.07 / 0.2 dB at 90 / 30 / 10 deg | TR 38.811 6.6.4: below 10 GHz "it may normally be neglected", (6.6-8) for system-level; TR 38.821 note 2: "shall be considered" (its budget: 0.1 dB) | ns-3 has them below 10 deg only. Under 0.2 dB; `--atmosphere=table` takes 0.1 dB at every elevation. |
| 5 MHz has 25 RB (4.5 MHz) and the satellite's EIRP is 34 dBW/MHz over them, 70.53 dBm | TS 38.101-1/-5 Table 5.3.2-1: 25 RB for 5 MHz at 15 kHz | The nr module by itself makes 26 RB (96 % of the bandwidth) and the run spread 5 MHz worth of EIRP (70.99 dBm) over 4.68 MHz: 0.29 dB too much SINR and 4 % too many resource blocks. 30 MHz had 160 RB already. |
| Feeder link of the transparent payload in Ka band, as a real LEO system has it, taken as reliable: free space and delay only. Gateway -> satellite 28.75 GHz from OneWeb's Santa Paula gateway (3.5 m, 58.0 dBi, -19.0 dBW/MHz into it: EIRP 39 dBW/MHz) to OneWeb's satellite gateway beam (G/T 11.4 dB/K): C/N 41.8 / 36.7 / 31.7 dB at 90 / 30 / 10 deg above the gateway. Satellite -> gateway (the uplink): delay only. 2100 MHz hold 420 carriers of 5 MHz (70 beams of 30 MHz) at that density with 26 W. No call while the gateway sees the satellite below 10 deg; the gateway is random within `--gatewayDiameterKm` of UE 0 | TR 38.821: the feeder link is a "3GPP or non-3GPP defined Radio interface" (Table 4.2-2), with no band, gain or bandwidth given; minimum elevation 10 deg for service and feeder link (Tables 4.2-2, 7.1-1); impairments other than delay "can be considered to be negligible" (Table 6.1.1.1-5 note 1), additional loss 0 dB (Table 6.1.3.2-1); dimensioned for availability up to 99.999 % (8.4.1.1). TR 38.811 Table 4.5-1 has 5 deg for gateways | The feeder takes 0.01 dB or less off the downlink SNR (the S-band feeder before: up to 0.06 dB). No rain fade, no outage, no feeder link switch. With the gateway within 100 km of UE 0, a whole pass loses 0 to 15 s at one end; with the satellite standing still the gateway is placed where it sees the satellite at 10 deg or more. `--feederFrequencyGHz`, `--gatewayGainDbi`, `--gatewayDbwPerMHz`, `--satFeederGtDbPerK`, `--feederBandwidthMHz`, `--gatewayDiameterKm`. |
| With a blocking scenario (`--scenario=NTN-Suburban` in the runs measured) a UE without line of sight stays in the cell (clutter loss) and, with ideal control, is scheduled at level 16 with almost three DCIs per packet | TR 38.821 6.1.1.1 calibrates with line of sight (`--scenario=NTN-LOS`, the default now) | In the drop measured one such UE took about 60 % of the CCEs used. Show the table with and without it, or run `--idealControl=0`, another drop (`--RngRun`) or NTN-LOS. |
| First-transmission error rate: the outer loop aims at 15 %; a semi-persistent block keeps the MCS of its activation | not specified | Retransmissions have a DCI each. Dynamic scheduling and scenario 2 follow the target, scenario 1 does not (about 2 % in the default run): 931 against 1380 retransmission DCIs for the same traffic. With `--targetBler=0.01` the three have about the same (857, 843, 862), and what is left is the difference between the schemes. |
| Scenario 2 changes the configuration with a release and an activation, 2 DCIs | 38.321 5.8.1 and 5.8.2 let several configurations be active in one BWP at the same time (Release 16, optional UE capability) | A gNB that keeps the 160 ms configuration active and switches only the 20 ms one needs 1 DCI per change, one that keeps both active none, at the price of unused occasions. Not modelled. |
| Voice frame 12 bytes: the 95 bits of AMR 4.75 alone | RFC 4867: 10 bits of payload header more, 14 bytes | Blocks 2 bytes (10 %) too small. `--voicePayload=14`. |
| One SID at once, then one every 160 ms | TS 26.093 5.1.2.1 (AMR): SID_FIRST, a SID_UPDATE three frames later, then every 8 frames | One SID per silence missing. In scenario 2 that SID would not fall on an occasion of the 160 ms configuration. |
| Context update of ROHC, 13 bytes (UOR-2 with extension 3), on the first packet after every change | RFC 3095: the size varies with what changed | Decides whether a first packet fits its block. `--ns3::NrPdcp::RohcTableUpdateSize`. |
| Layer-2 headers 1 + 1 + 3 bytes, the count of TR 26.940 5.2.3 (NB-IoT NTN) | NR: PDCP 2 (12-bit SN) + RLC UM 1 (no SN) + MAC 2 = the same 5 bytes; 2 bytes more on every segment after the first | None on block sizes. A SID in two pieces has 10 bytes of headers instead of 9 and still takes two occasions. |
| Buffer status report 5 bytes in every uplink MAC PDU (the nr module) | 38.321 6.1.3.1: Short BSR, 2 bytes with its subheader, and only when one is triggered | Uplink blocks 3 bytes too large. No DCI more or less. |
| Delay budget of the radio side = 100 ms less the delay UPF to gateway, drawn per run (0 to 10 ms) | TS 23.501 5.7.3.4 and Table 5.7.4-1: for 5QI 1 a fixed 20 ms is taken off, 80 ms | Up to 20 ms more room than the standard gives. `--upfGwDelayMinMs=20 --upfGwDelayMaxMs=20`. |
| DCI of 34 bits; error rates of level 1 from the table that was given, 3 dB less SINR per doubling of the level | 38.212 7.3.1.2.1: format 1_0 has 37 bits on 25 RB, 42 on 160 RB | Thresholds 0.2 to 0.6 dB optimistic. `--dciBits`. |
| One CORESET for SIB1 and the UEs; SIB1 every 20 ms at level 8; no paging, no other system information (SIB19) | 38.213 Table 13-1: CORESET 0 is 24, 48 or 96 RB, and 24 RB needs 2 or 3 symbols | A 5 MHz CORESET of 1 symbol (4 CCEs) cannot carry SIB1 in 3GPP: use `--pdcchSymbols=2`, `3` or `0` there. |
| A dynamic assignment takes the place of the SPS block in the slot of the occasion | 38.214 5.1: the UE takes it only if its PDCCH ended 14 symbols before the SPS PDSCH | The DCI would go one slot earlier. Same DCIs and CCEs, 1 ms. |
| Release and activation in the same slot | two unicast downlink DCIs in one slot are an optional UE capability | The second DCI would go a slot later. Same DCIs. |
| Uplink link adaptation (`--ulLinkAdaptation=1`): the MCS of a grant is that of the SINR measured on the UE's last transmission, scaled to the resource blocks of the grant (the UE spreads its 23 dBm over them), an outer loop on the decoding keeps first transmissions at `--targetBler`, and a UE gets no more resource blocks than still carry MCS 0 once its block holds a voice frame. A configured-grant block takes the whole band (TDMA, or `--ulGrantRbs=0`) or `--ulGrantRbs` resource blocks (OFDMA), and keeps the MCS of its activation | Not specified; a gNB knows the UE's power headroom (38.321 5.4.6, 6.1.3.8) | With `--ulLinkAdaptation=0` (the nr module's: the measured MCS whatever the size of the grant, no outer loop) first transmissions fail often, about half of them when a grant has more resource blocks than the measurement, and each retransmission has a DCI. 20 uplink calls on 5 MHz, OFDMA RR: first transmissions 22 % -> 19 % failed (the UEs with a usable link at the 15 % target), retransmission grants -35 %, lost packets 2.6 % -> 0.5 % (two UEs without a usable link), 98 % delay 99 -> 56 ms. |
| OFDMA: a HARQ retransmission gets resource block groups over the data symbols of its slot, which new data of other UEs shares; as many as hold the resource elements it had, its own first | 38.214 5.1.2 / 6.1.2: the time and frequency allocation of a retransmission is the gNB's choice (it keeps the transport block size) | In the nr module (`--ns3::NrMacSchedulerOfdma::HarqRetxInFrequency=0`) an OFDMA retransmission takes all the data symbols of its slot, in both directions: no new data in that slot, and a second retransmission due in it waits a slot. |
| A scheduling request gets a grant for a voice frame with its headers and a buffer status report (`--srGrantBytes`, 25 B) | The size is the gNB's choice (38.321 5.4.4) | With the nr module's 17 B the frame needs a second grant, a round trip later. |
| Aggregation level from one slot's control SINR, reported every 10 ms in 0.5 dB steps, no margin: the lowest level with 1 % BLER at the reported SINR, the highest level of the CORESET if none has | Not specified: the gNB chooses the level (L1-SINR reporting: TS 38.214 5.2.1.4) | At decode the report is 9 to 80 ms old (median 24 ms, 5 MHz full pass). With line of sight the control SINR moves about 2 dB in that time and a UE misses 0.7 % of its DCIs; without, 5 dB, and it misses about 30 %, more so when its SINR is below what the highest level needs. Those UEs also get most of the retransmission DCIs, so they dominate missed[%]. `--ns3::NrPdcch::SinrMarginDb`, `--pdcchSymbols`. |
| A radio link failure: the gNB deletes the UE context at once (no RRC re-establishment), the core keeps the 5QI 1 flow and the UE its address, the next connection sets the flow up on a new bearer, and the call goes on | TS 38.331 5.3.7: re-establishment within T311 (at most 30 s) keeps the bearers. Failing that, TS 23.502 4.2.6 keeps a GBR QoS flow over an AN release only for user inactivity, UE redirection or a new NAS signalling connection; otherwise the SMF triggers a PDU Session Modification for it (TS 23.401 5.3.5, LTE: the GBR bearers are deactivated) | A call a network would most likely have ended goes on; its packets without a bearer are "no conn.". The 5QI 1 table has a line per bearer (n.1, n.2, with the RNTI), each held against 5QI 1 on its own; the line of the call adds them up. |
| Carrier 2 GHz, 30 MHz, as TR 38.821 study case 9 | TS 38.101-5: band n256 is 1980-2010 / 2170-2200 MHz, channels up to 20 MHz | `--dlFrequencyGHz=2.185 --ulFrequencyGHz=1.995 --bandwidthMHz=20` (or 5, 10, 15). |

## 10. How the model was checked

- The PDCCH rules: `pdcchtest` drives `NrPdcch` through random slots and `pdcchcheck.py`, an
  independent implementation of TS 38.213 10.1 in Python, recomputes every CCE, level, blocked
  DCI and counter.
- Every identity between the tables of an output (packets sent, received and lost by cause;
  DCIs and CCEs by purpose and by level; CCEs used against CCEs possible; activations and
  releases against the PDCCH table; changes against how their first packets were sent): 5653
  checks over 41 runs (downlink and uplink; dynamic, scenario 1 and 2; TDMA and OFDMA; 5 and
  30 MHz; with and without HARQ, ROHC, control errors; satellite standing still and flying;
  five drops), none failed.
- Against the version before: 11 of 12 downlink runs give the same packets, delays, DCIs and
  CCEs with the module of this version (the twelfth has control errors and differs by the
  order of the DCIs only, see below). The one change to the downlink is the noise of the
  satellite's feeder receiver, which now has the satellite's noise figure (4.28 dB) instead of
  the UE's (7 dB): 0.1 dB more SNR.
- Five drops (`--RngRun=1` to `5`; 10 downlink calls, 20 s, `--targetBler=0.01`): DCIs for
  new data, activations and releases together are 5187 on average with dynamic scheduling
  (4498 to 5576), 70 in scenario 1 (62 to 77) and 228 in scenario 2 (211 to 259).
  Retransmission DCIs are 59 to 857, 29 to 843 and 61 to 862: they follow the number of UEs
  of the drop that have no line of sight, not the scheme.
- The same run twice gives the same output, byte for byte; so does a build from a clean copy of
  the release with the patch. The order of the DCIs of a slot no longer depends on where the
  messages lie in memory (it did in the nr module, which made runs with control errors differ
  from one build to the next).
- Geometry and link budget recomputed by hand from the printed settings: slant range, delays,
  beam footprint, beam gains, K_offset. The CNR the header prints for UE 0 against the mean SINR
  of its control symbols (5 MHz, 2.175 GHz, -5 dBi, satellite standing still,
  `--fastFading=fixed`): with the defaults 5.94 / 0.87 / -4.22 dB printed, 5.93 / 0.87 /
  -4.22 dB simulated at 90 / 30 / 10 deg. At 30 deg, printed and simulated:
  `--atmosphere=location` 3.10 and 3.10 dB (latitude 45 deg; at 10 deg elevation -2.12 and
  -2.13 dB), 1.16 and 1.16 dB (`--latitudeDeg=10`); `--shadowingDb=0` 3.87 and 3.86 dB;
  `--fastFadingDb=2` -1.13 and -1.13 dB; `--linkMarginDb=1.5` -0.63 and -0.63 dB. The Ka-band
  feeder link takes off 0.01 dB or less. With the satellite flying (30 to 31.5 deg in 6 s) the
  mean is 1.04 dB with `--fastFading=fixed` and 0.98 dB with `model`. Standing still, `model`
  gives one draw per run: -0.6 to +2.5 dB from the CNR (`--RngRun=1` to `8`).
  With `--ueAntennaGainDbi=0` at 2 GHz the header prints 6.60 dB at 30 deg: TR 38.821's 6.6 dB
  for the same case (Table 6.1.3.3-1, study case 9).
- The loss options: the five scenarios, both modes of shadowing, atmosphere (latitudes 45, 10
  and -15 deg) and fast fading, downlink and uplink, transparent and regenerative, a whole
  pass; the identities between the tables and the CSV files against the printed tables: 2481
  checks over 9 runs, none failed. A latitude above 60 deg and every misspelt mode stop the
  run with a message.
- 3GPP: the procedures and sizes in sections 1 to 7 were read in the Release 18 specifications
  and again in Release 16 (38.321 V16.4.0, 38.213, 38.214, 38.212, 38.322, 38.323, 38.331).
