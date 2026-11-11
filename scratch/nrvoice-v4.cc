/*
 * ntnlab_nr.cc  --  ntnlab_n.cc with the LTE cell replaced by an NR-NTN cell (first, simple
 * version)
 *
 *   UE i ~~ service link ~~ LEO satellite ~~ feeder link ~~ gNB at the gateway == S1-U == UPF
 *   7.0.0.(i+2)             600 km, transparent                               (SGW/PGW)  ||
 *   the other end of call i, --farEnd:                                                    ||
 *     host (default)  remote host 1.0.0.2, 20 ms from the gateway == 10 Gbps =============||
 *     ue              terrestrial UE i ~~ 5G cell, n78 or n3, 20 MHz ~~ gNB == S1-U, 5 ms =
 *                     8.0.0.(i+2)
 *
 * Needs ns-3-NTN v1.1.0: ns-3.48 with the NTN changes, and nr-ntn-dev v1.1.0 in contrib/nr with
 * the files of nr-ntnlab-module.tgz unpacked over it (ROHC, short layer-2 headers, the 20 ms
 * voice generator). Put this file in scratch/.
 *
 * Kept from ntnlab_n.cc
 *   - one remote host, N calls, one NGMN VoIP generator per call, per-call voice activity factor
 *   - a dedicated voice flow (5QI 1) for every UE, selected by the UDP port, 40 kbit/s GBR
 *   - FlowMonitor gives one line per call; the UE PHY gives the DL SINR
 *
 * What replaced the LTE cell
 *   - the radio settings are those of contrib/nr/examples/leo-link.cc with its default preset
 *     (TR 38.821 study case 9: LEO-600, S-band 2 GHz, FDD, 34 dBW/MHz, handheld UE), including
 *     K_offset, the timing advance and the feeder link of a transparent payload (here a Ka-band
 *     one, see "The gateway")
 *   - --satSet=2 swaps in the Set-2 satellite of TR 38.821, --altitudeKm=1200 the LEO-1200 one
 *     (any other altitude, 300 to 2000 km, takes the nearer of the two columns of the set);
 *     altitude, carriers, EIRP density, antenna gain, G/T and aperture are flags of their own
 *   - the beam is the pattern of TR 38.811 6.4.1, 4 |J1(ka sin(t)) / (ka sin(t))|^2 with
 *     k = 2 pi f / c and a = half the aperture: the same antenna has a narrower beam on a higher
 *     carrier, so downlink and uplink (FDD) each have their own beamwidth. The set's 3 dB
 *     beamwidth holds at the set's 2 GHz. --beamHpbwDeg fixes the width on both carriers instead.
 *     EIRP density, gain and G/T do NOT follow the carrier: they are what their flags say.
 *   - the handheld: 23 dBm, noise figure 7 dB, antenna gain --ueAntennaGainDbi = -5 dBi, on
 *     both carriers (it receives and transmits through the same antenna; TR 38.821: 0 dBi).
 *   - the carrier has the resource blocks of TS 38.101-1/-5 Table 5.3.2-1 for 15 kHz (5 MHz:
 *     25 RB = 4.5 MHz; the nr module by itself makes 26), and the satellite's EIRP is its EIRP
 *     density over these: 34 dBW/MHz x 4.5 MHz = 70.53 dBm.
 *   - the service link has four losses, those of TR 38.811 6.6 for a system-level study. For
 *     each the run takes either TR 38.821's link budget (Table 6.1.3.3-1, S-band: the defaults)
 *     or a model:
 *     1 free space (6.6-2), always.
 *     2 blocking and shadowing. --scenario=NTN-LOS (default): every UE is in line of sight all
 *       the time, the propagation condition of TR 38.821's calibration (Table 6.1.1.1-5);
 *       NTN-Rural, -Suburban, -Urban, -DenseUrban: a UE can be blocked, by the LOS probability
 *       of 6.6.1, and has the clutter loss of 6.6.2 while it is (see "Line of sight and
 *       shadowing"). --shadowing=fixed (default): --shadowingDb (3) for every UE all the time,
 *       TR 38.821's shadowing margin; table: each UE's own shadow fading, with the sigma of
 *       6.6.2 for its scenario, state and elevation (NTN-LOS: rural line of sight, 0.72 to
 *       1.79 dB).
 *     3 atmosphere. --atmosphere=table (default): TR 38.821's 0.1 dB of gases and 2.2 dB of
 *       scintillation, for every UE at every elevation; location: gases A_zenith(f) /
 *       sin(elevation) (6.6-8; ns-3 has them below 10 deg only, here they are at every
 *       elevation, as TR 38.821 Table 6.1.1.1-5 note 2 asks) and the ionospheric scintillation
 *       of 6.6.6.1.4 at --latitudeDeg (45), the UEs' magnetic latitude: within 20 deg of the
 *       magnetic equator 2.2 dB at 2 GHz, x (f / 2 GHz)^-1.5; 20 to 60 deg none; above 60 deg
 *       TR 38.811 has no model, and the run stops.
 *     4 fast fading. --fastFading=fixed (default): --fastFadingDb (0) on every resource block,
 *       the direct path alone (TR 38.821's link budget has no fast fading); model: the 3GPP
 *       channel of TR 38.901 7.5 with TR 38.811's tables for the scenario (NTN-LOS: rural), per
 *       UE, resource block and slot, with the Doppler of the satellite (see "The satellite").
 *     --linkMarginDb adds a loss on every link (TR 38.821's "additional losses": 0).
 *     The header prints the CNR of UE 0 in line of sight (--shadowing=table: with its shadowing
 *     at 0). With every default (2 GHz, -5 dBi): 6.67 / 1.60 / -3.49 dB at 90 / 30 / 10 deg;
 *     with --ueAntennaGainDbi=0 it is TR 38.821's 6.6 dB at 30 deg (study case 9).
 *   - "distance" became "elevation", the elevation of the satellite seen from UE 0
 *   - every UE loses the beam gain of the place it is at, seen from the beam centre
 *
 * Where the UEs are, --ueLayout
 *   - UE 0 is always on the beam centre: the reference user, with the full beam gain and the
 *     elevation that was asked for. With --nUes=1 it is the only one.
 *   - random (default): each other UE is dropped somewhere in the beam area, every place of the
 *     area as likely as any other, the UEs independent of each other: a uniform drop, which is
 *     what a Poisson point process gives once its number of points is known. The beam area
 *     is the ground inside the 3 dB contour of the downlink beam, the footprint of the beam. It
 *     is an ellipse stretched along the direction to the satellite, the more the lower the
 *     satellite stands. When the satellite flies the footprint changes, and the area is what
 *     the footprints of all moments of the calls have in common, so every UE stays within 3 dB
 *     of the beam centre for the whole call. A pass over the top leaves the circle the beam
 *     covers when the satellite is overhead. Places from where the satellite is ever below
 *     5 deg are left out. (UE 0 keeps the 10 deg at which the 3GPP NTN tables start. A dropped
 *     UE may see the satellite a little lower, at the start or the end of a pass or in the far
 *     half of the footprint at a low --elevation; the channel models then take their 10 deg
 *     row, as they do for everything below 15 deg.) --RngRun=2, 3 ... give other drops. The
 *     header says how large the area is, the call table where each UE is, and the second table
 *     how low each UE saw the satellite.
 *   - line: UE i is i x --spacingKm east of the beam centre
 *
 * The gateway (transparent payload, the default): the gNB is there, so the radio path is
 * gateway -> satellite -> UE. The feeder link has the delay of its length, which K_offset and
 * the timing advance cover. It is a Ka-band link, as real LEO systems have, wide enough for many
 * beams and taken as reliable (TR 38.821 8.4.1.1: dimensioned "to provide availability up to
 * 99.999%"): free space path loss and delay, nothing else (no gases, rain, scintillation or
 * shadowing).
 *   - gateway -> satellite, which carries the downlink: OneWeb's gateway at Santa Paula (FCC
 *     SES-LIC-20190422-00538: 27.5-29.1 and 29.5-30.0 GHz, a 3.5 m antenna of 58.0 dBi at
 *     28.75 GHz fed with -19.0 dBW/MHz): --feederFrequencyGHz=28.75, --gatewayGainDbi=58,
 *     --gatewayDbwPerMHz=-19, an EIRP density of 39 dBW/MHz. The satellite receives it with
 *     the G/T of OneWeb's gateway receive beam (FCC SAT-LOI-20160428-00041 Table A.13-1):
 *     --satFeederGtDbPerK=11.4. It forwards the signal with the noise of that receiver: C/N
 *     41.8 / 36.7 / 31.7 dB at 90 / 30 / 10 deg, which takes 0.01 dB or less off the downlink
 *     (TR 38.821 Table 6.1.1.1-5 note 1: negligible). --feederBandwidthMHz (2100, the two
 *     bands) is only for the header, which counts the carriers of --bandwidthMHz the link takes
 *     at that density: 420 of 5 MHz, 70 of TR 38.821's 30 MHz per beam, with 26 W into the
 *     antenna (the filing has 114 W).
 *   - satellite -> gateway, which carries the uplink: delay only (OneWeb: 17.8-19.3 GHz).
 *   - the feeder link needs the satellite at 10 deg or more above the gateway (TR 38.821 Table
 *     4.2-2, the same as for the UEs), and no call is on without it. When the satellite flies,
 *     UE 0's call starts when the gateway sees the satellite at 10 deg, if that is later than it
 *     would start, and ends when the gateway sees it sink below 10 deg, if that is earlier; the
 *     other calls keep their place after UE 0's. In a whole pass a gateway 100 km from UE 0 along
 *     the track costs up to 15 s at one end. The header says when the gateway allows calls.
 *   - --gatewayDiameterKm=200 (default): the gateway is at a random place of the circle of that
 *     diameter around UE 0 (within 100 km of it), every place of the circle as likely as any
 *     other. It is drawn on its own, whatever the UEs' places are; --RngRun gives another one.
 *     Places from where the satellite never reaches 10 deg during UE 0's call are left out (with
 *     the satellite standing still: places from where it is below 10 deg).
 *   - --gatewayDiameterKm=0: the gateway is where UE 0 is, so the feeder link is as long as
 *     UE 0's service link, and the gateway sees the satellite as high as UE 0 does.
 * With --transparent=0 the gNB is on the satellite and no gateway is in the radio path.
 *
 * The satellite
 *   - --freezeGeometry=1 (default): it stands still at --elevation but reports its orbital
 *     velocity, so the fading keeps moving. Elevation, range and delay are constant.
 *   - --freezeGeometry=0: it flies ns-3.48's circular orbit, as in leo-link. It comes from the
 *     north, is at --elevation when the calls start, passes exactly over UE 0 and goes down in
 *     the south. Elevation, range and delay change during the call, K_offset covers the longest
 *     round trip of the run, the timing advance follows the delay, and the beam keeps pointing
 *     at UE 0. UE 0's call must end before the satellite is down at 10 deg again; the header
 *     says how long that is. A LEO-600 satellite takes about 255 s from 10 deg to overhead, so a
 *     short call sees little of the pass: --elevation=10 --duration=20s goes from 10 to 12 deg.
 *   - --fullPass=1: the whole pass over UE 0. The satellite flies, UE 0's call starts when the
 *     satellite has risen to 10 deg above it and ends when it has sunk to 10 deg again (and the
 *     gateway has to see it at 10 deg too, see above), and the run lasts as long as that call.
 *     --freezeGeometry, --elevation and --duration are not used.
 *   - The calls: UE 0's call lasts --duration from t = 1 s, less what the gateway does not allow
 *     (see above). No call starts before it and none goes on after it: call i starts
 *     i x --startOffsetMs after it and ends with it.
 *     When the satellite flies, a call is also on only while the satellite is 10 deg or more
 *     above its own UE: it starts when the satellite has risen to 10 deg above that UE, if that
 *     is later, and ends when the satellite has sunk to 10 deg above it again, if that is
 *     before UE 0's call ends. A UE that never sees the satellite at 10 deg during UE 0's call
 *     has no call. When the satellite stands still nothing rises or sinks, and every call runs
 *     to the end of UE 0's wherever its UE is in the beam.
 *   - Doppler. The path delay changes while the satellite flies: the timing advance follows it,
 *     and a downlink signal arrives compressed by the delay drift over its duration. The carrier
 *     frequency offset itself (UE 0: up to 45 kHz at 10 deg on the 2 GHz downlink) is not
 *     modelled: the UE takes it off, from its GNSS position and the satellite's ephemeris (TS
 *     38.300 16.14.2), and the 0.1 ppm it may leave (TS 38.101-5), 1.2 % of a subcarrier, leaks
 *     33 dB below the signal into the other subcarriers. The header gives UE 0's Doppler.
 *   - --fastFading=model: ns-3.48's channel model draws a new realisation (K-factor included)
 *     whenever the satellite has moved 1 m, here every 1 to 2 ms when it flies. When it stands
 *     still, the realisation is drawn once per line-of-sight state, and the Doppler term moves
 *     it (at 30 deg, --RngRun=1: 0.1 to 3.6 dB) around a mean of its own: the mean SINR of the
 *     run is that of one draw (--RngRun=1 to 8: -0.6 to +2.5 dB from the CNR). Let the
 *     satellite fly for a fading that averages out.
 *
 * Line of sight and shadowing, --scenario and --shadowing
 *   - what stands around a UE does not change during its call. Every UE has an obstruction
 *     level (uniform, 0 to 1) and a shadowing level (standard normal), drawn once. --RngRun
 *     gives other levels; --scenario and --shadowing do not change them, so one drop can be
 *     compared across them.
 *   - NTN-Rural, -Suburban, -Urban, -DenseUrban: a UE is in line of sight whenever the LOS
 *     probability at its elevation (TR 38.811 Table 6.6.1-1) is above its obstruction level.
 *     Under a satellite that stands still a UE is in line of sight for the whole call or not at
 *     all; when the satellite flies, a blocked UE clears once the satellite is high enough and
 *     is blocked again when it has sunk below that elevation. The same holds on both carriers.
 *     A blocked UE also has ns-3's clutter loss (18 dB suburban, 29 dB urban at 30 deg), so it
 *     is far below the others. NTN-LOS: no UE is ever blocked.
 *   - --shadowing=table: a UE's shadow fading is its shadowing level times the sigma of its
 *     state and elevation (Tables 6.6.2-1 to 6.6.2-3). fixed: --shadowingDb for every UE,
 *     whatever its state.

 * Calls, --direction and --farEnd
 *   - --direction=both (default): every call is two-way. Each end has a speaker of its own: a
 *     voice generator with a voice activity factor (VAF) drawn for that speaker, and talk and
 *     silence of its own, so the two talk independently (both at once, or both silent,
 *     happens). A call is two flows: dl, the far end talks (over the satellite's downlink),
 *     and ul, the UE talks. --direction=dl or ul: one-way calls, that flow only.
 *   - --farEnd=host (default): the other end of every call is the remote host.
 *   - --farEnd=ue: the other end of call i is terrestrial UE i, in a typical 5G cell in ideal
 *     conditions on a band of its own, so nothing it sends meets the satellite's carriers. There
 *     are as many terrestrial UEs as calls (--nUes), on a network of their own: UE i is
 *     8.0.0.(i+2) of 8.0.0.0/8, while satellite UE i is 7.0.0.(i+2) of 7.0.0.0/8. The UPF is the
 *     gateway of both networks (7.0.0.1 and 8.0.0.1) and routes the calls between them. (No
 *     address ends in .0 or .255: after x.0.0.254 come x.0.1.1, x.0.1.2 and so on.) The
 *     header's FAR END block lists all their settings. Each has an option (defaults in brackets):
 *       --farEndBand          n78 [default]: TDD, 30 kHz; n3: FDD, 15 kHz, uplink 95 MHz below
 *       --farEndFrequencyGHz  downlink carrier, in the band [0: 3.5 GHz for n78, 1.8425 for n3]
 *       --farEndBandwidthMHz  one of the band's channel bandwidths [20]: n78 10 to 100 MHz, n3 5
 *                             to 30 MHz; the RBs are those of TS 38.101-1 Table 5.3.2-1 (20 MHz:
 *                             51 RB at 30 kHz, 106 RB at 15 kHz)
 *       --farEndTddPattern    n78: the slots, D/S/U/F [DDSUU: as many D as U slots, 2.5 ms]. In
 *                             the nr module S carries downlink data too (with the uplink
 *                             control), so DDSUU has downlink data in 3 slots, uplink in 2
 *       --farEndUesPerGnb     calls per gNB (cell) [10]; the gNBs are 100 km apart and each cell
 *                             has a channel of its own (no interference between them)
 *       --farEndGnbDbm        gNB power per 20 MHz, scaled to the bandwidth [44: ITU-R M.2412,
 *                             Dense Urban-eMBB macro]
 *       --farEndMinDistM, --farEndMaxDistM   the UEs are dropped uniformly over the 120 deg
 *                             sector of their cell, this far from the gNB [35 to 250 m]
 *       --farEndScenario      TR 38.901 UMa, UMi (street canyon) or RMa, with fast fading;
 *                             "-LOS": always in line of sight, else by the scenario's LOS
 *                             probability [UMa-LOS]. The gNB is 25 / 10 / 35 m high (Table 7.2-1).
 *                             n3: the uplink has the downlink's line of sight state and shadowing
 *                             (one link, as in TR 38.901); its fast fading is its own
 *       --farEndErrorModel    [the satellite cell's, --errorModel]
 *       --farEndUpfDelayMs    gNB - UPF, one way [5]; all on the core of the satellite cell
 *     Fixed: the gNB's 4 x 8 panel of TR 38.901 elements (8 dBi), noise figure 5 dB; the UE 1.5 m
 *     high, 23 dBm, 0 dBi, noise figure 7 dB. Its voice bearer is 5QI 1 as well. The run's
 *     switches act on the satellite cell only: the terrestrial cells are plain NR, with the nr
 *     module's own defaults (no ROHC, PDCP and RLC sequence numbers of 12 and 10 bits, HARQ with
 *     16 processes, the module's link adaptation, no uplink grant limit, no PDCCH limit, no SPS,
 *     no PDCCH errors). Only the 3-byte MAC subheader of --l2Headers=1+1+3 is the same on both.
 *     Voice goes UE to UE through the UPF; the satellite UE's call sets when both directions
 *     start and end.
 *   - The delay is measured on every packet over each stretch: satellite UE <-> UPF (the 5QI
 *     table), far end <-> UPF (--farEnd=ue: FAR END AGAINST 5QI 1) and the whole way, sender's
 *     IP to receiver's IP (END TO END).
 *
 * The tables, one line per flow (dl, ul) of a call: the call itself (CALLS), why its packets
 * were lost, the flow against the 5QI of its bearer (with --farEnd=ue also the far end's stretch
 * against it), and the whole way, end to end. Then what each UE saw of the satellite during its
 * call: elevation (- before the satellite is overhead, + after), slant range, and the
 * propagation delay of the whole path, far end to UE. CALLS starts with the two ends of the
 * call: the satellite UE's address, an arrow the way the voice goes (dl <-, ul ->), and the far
 * end's, the terrestrial UE (8.0.0.(i+2)) or the remote host (1.0.0.2); the packets a flow
 * lost (tx - rx) are in the second table, by cause. N[km] and E[km]
 * say where the UE is, north and east of the beam centre. beam[dB] and SF[dB] are the beam gain
 * towards the UE and its shadow fading, both as gains (negative: a weaker signal) and as the
 * mean over the call. LOS[%] is the share of the call with line of sight to the satellite, RLF
 * the number of radio link failures of the UE, out[s] the time of the call it had no connection
 * after such failures. DL SINR is the mean over the control symbols the UE received; it receives
 * none while it has no cell, so the time out of coverage after a failure is not in it.
 *
 * Lost packets by cause, the second table. Every lost packet (tx - rx) is in exactly one column:
 *   no conn.  the application sent it, but it never got a radio bearer: the UE had no
 *             connection, as after a radio link failure until it is attached again
 *   discard   it got a bearer, but the sender did not take it: the oldest packet in its queue
 *             had waited longer than the discard timer (see "5QI 1" below)
 *   radio     it was sent on its bearer and did not arrive at the other end: every HARQ
 *             transmission failed, or it was still waiting when the connection broke
 *   ROHC      it arrived, but the decompressor had lost its context and could not restore the
 *             header, so it dropped the packet
 *   far end   it was lost on the far end's stretch: the terrestrial UE's radio and core
 *             (--farEnd=ue), or the link to the remote host
 * The first four are on the satellite UE's stretch.
 * link[%] = no conn. + discard + radio is lost with or without header compression. ROHC[%] is
 * what the compression cost: those packets had arrived. A context is lost because packets
 * were lost on the air before (more than --rohcDynamicWindow in a row, or the IR packet that
 * should have set it up), so ROHC adds to a radio loss, it does not start one; and after a
 * radio link failure the new connection starts with a new context, so nothing is added there.
 * NrPdcp counts the packets (NrPdcp::GetPacketCounts), with or without ROHC.
 *
 * 5QI 1, the third table. Every UE has a dedicated GBR flow with 5QI 1 for its voice (and its
 * default flow, unused). TS 23.501 Table 5.7.4-1 gives 5QI 1 a packet delay budget (PDB) of
 * 100 ms, a packet error rate (PER) of 1e-2 and an averaging window of 2000 ms. The table
 * holds each call against it, on the packets its bearer was given ("on bearer"; a packet sent
 * while the UE had no connection met no bearer and is in the second table only):
 *   delay     5.7.3.4: the PDB is the bound for the time between the UE and the N6 termination
 *             point at the UPF, the same in both directions. Here that is UE to PGW: the way
 *             between gateway and UPF is in it, the N6 link to the remote host is not (see
 *             "Delays on the ground"). It is measured on every packet with a time stamp put on
 *             at one end and read at the other.
 *   98 %      5.7.3.4: of a GBR flow that keeps to its GFBR, "98 percent of the packets shall
 *             not experience a delay exceeding the 5QI's PDB". over PDB[%] must be at most 2:
 *             the packets delivered late, plus those the sender discarded because it could
 *             not send them in time. 98%[ms] is the delay 98 % of the delivered ones stayed
 *             within.
 *   PER       5.7.3.5: the packets the sending link layer processed and the receiving one did
 *             not deliver: radio + ROHC. Table 5.7.4-1 NOTE 1: "A packet which is delayed more
 *             than PDB is not counted as lost, thus not included in the PER": late and
 *             discarded packets are not in PER[%], which must be at most 1.
 *   GFBR      --gfbrKbps (40). kbit/s is the highest bit rate of the call's IP packets over
 *             any 2 s; AMR 4.75 with RTP talks at 20.8 kbit/s.
 *   missed    which of the two rules the call missed.
 *   bearers   a radio link failure ends the connection and its voice bearer; the next
 *             connection sets up a new one. A call with more than one connection has a line
 *             per bearer under its own: n.1, n.2, ... with the RNTI of the connection (as in the
 *             ROHC tables), each held against 5QI 1 on its own. The line of the call adds them
 *             up. (That the call and its 5QI 1 flow go on after a failure is the nr module's;
 *             see "Control channels and radio link failure".)
 * What the sender does with a packet it cannot send in time is the nr module's: its RLC does
 * not take a new packet while the oldest one in its queue has waited longer than the discard
 * timer. At the gNB the timer is what is left of the PDB of the bearer (100 ms for 5QI 1) when
 * the packet arrives there: the PDB less the delay between UPF and gateway, in whole ms. A UE
 * has none, so a gNB discards and a UE delivers late. --ns3::NrRlcUm::DiscardTimerMs=100 gives
 * both that timer, --ns3::NrRlcUm::EnablePdcpDiscarding=0 none to either. (TS 23.501 Table
 * 5.7.4-1 NOTE 13 takes a fixed 20 ms off the PDB for the core network, whatever the core
 * needs; here it is the delay the run has between UPF and gateway.)
 *
 * Delays on the ground, one way. The remote host is --hostGwDelayMs (20) from the gateway, in
 * every run. Where on that way the UPF is differs from run to run:
 *   UPF - gateway       one value per run, drawn uniformly between --upfGwDelayMinMs (0) and
 *                       --upfGwDelayMaxMs (10) from a random stream of its own: --RngRun gives
 *                       another one, min = max that value. It is part of the 5QI delay, and
 *                       what it takes of the PDB is not there for the radio side: with 6 ms
 *                       here, gateway to UE has 94 ms.
 *   remote host - UPF   the rest of the 20 ms (14 ms with 6 ms above). Not in the 5QI delay,
 *                       but in "delay[ms]" of the first table and in the delay of the last.
 * With --transparent=0 the gNB is on the satellite; the delay stands for the way from the UPF
 * to the gateway all the same, and the feeder link, which then carries N3, adds none.
 * With --farEnd=ue the remote host carries no call: the terrestrial gNBs are --farEndUpfDelayMs
 * (5) from the UPF, and the 5QI 1 budget of the far end's stretch (100 ms) has that taken off
 * for its radio side, as the satellite's has the UPF - gateway delay.
 *
 * Control channels and radio link failure, --idealControl
 *   - 1 (default): the nr module as it is. No control message is ever lost: a UE receives every
 *     scheduling message whatever its SINR. After a radio link failure the UE searches for the
 *     cell, and takes it, 0.1 to 0.3 s later, if the RSRP it measures is above -140 dBm (the
 *     lowest q-RxLevMin of TS 38.331, which the cell selection criterion of TS 38.304 5.2.3.2
 *     uses, and the floor of the nr module's cell search); below that it stays without service
 *     for the rest of its call, and its out[s] shows it. (At the start of the run a UE is
 *     attached without that test, wherever it is.) Its radio link monitoring counts control
 *     symbols, not time: 1200 of them below Qout (-5 dB) and then T310 (1 s) make a failure. A
 *     blocked UE at -9 or -12 dB therefore still gets nearly all its packets, each after three or
 *     four HARQ transmissions, and fails and attaches again every 3 to 5 s. Its RLF column counts
 *     these rounds, and each costs a handful of packets.
 *     Random access, and so a random access response on the PDCCH, happens at these new
 *     attachments only: the first attachment of every UE is over before the calls start, when
 *     the PDCCH report begins.
 *   - 0: the PDCCH has a block error rate (NrUePhy::PdcchErrorModel), taken from the two levels
 *     that define the radio link monitoring: 10 % at Qout (-5 dB) and 2 % at Qin (-3.9 dB), a
 *     normal curve in dB through the two. Half of the messages are lost at -6.8 dB, 99 % at
 *     -10.1 dB. Three things follow from this one curve:
 *       scheduling   a downlink assignment or an uplink grant that is lost costs that
 *                    transmission: nothing is received and HARQ has nothing to combine
 *       monitoring   as in TS 38.133, once per 10 ms: out of sync when the mean error rate of
 *                    the control symbols of the last 200 ms is above 10 %. After N310 (6) such
 *                    frames in a row T310 (1 s) runs, and the link has failed when it expires,
 *                    unless the mean of the last 100 ms was below 2 % in N311 (2) frames in a row
 *       cell search  after a failure the UE attaches again when it can receive the cell again:
 *                    the mean error rate of a 200 ms measurement period is below 2 %. Until
 *                    then it has no service and its packets are lost.
 *     A failure is then the end of a link that had become unusable and the start of an outage.
 *     Qout, Qin, N310, N311 and T310 are attributes of the nr module:
 *     --ns3::NrUePhy::Qout=-8 --ns3::NrUePhy::Qin=-6 --ns3::NrUeRrc::T310=2s and so on.
 *     Still ideal: MIB, SIB1, random access, the RRC messages and the uplink control channel
 *     (HARQ feedback, scheduling requests, CQI).
 *   - The QoS flows at a radio link failure, in both settings: the gNB deletes the UE context at
 *     once (the nr module has no RRC re-establishment, TS 38.331 5.3.7), the core keeps the
 *     UE's QoS flows and its address, and the next connection sets the same flows up again on
 *     new radio bearers: the call goes on. TS 23.502 4.2.6 keeps a GBR QoS flow over an AN
 *     release only for user inactivity, UE redirection or a new NAS signalling connection; in
 *     any other case the SMF triggers a PDU Session Modification for it (in LTE, TS 23.401
 *     5.3.5 deactivates the GBR bearers after "Radio Connection With UE Lost"). A network
 *     would most likely have ended such a call; the 5QI 1 table has a line per bearer.
 *   - ROHC at a radio link failure: the bearers are released, and the new connection has new
 *     PDCP entities, so the compressor starts again with an IR packet and in U-mode. That is
 *     what TS 38.323 5.1.2 asks for when PDCP is re-established (without drb-ContinueROHC). In
 *     the ROHC tables it is a new RNTI line with its own IR packet, not a "failed" packet:
 *     "failed" counts packets that arrived and could not be restored.
 *
 * ROHC: needs the nr module with the ROHC changes (NrPdcp::EnableRohc). It is on when the module
 * was built with the ROHC library, --rohc=0 turns it off, --rohcReport=full prints the details.
 * The run ends with the ROHC tables, one line per connection (RNTI) of each UE and a line "all"
 * for a UE that had several:
 *   compressor     what the sender made of the packets: states, header size, bytes saved, and
 *                  the feedback that reached it
 *   decompressor   what the receiver could do with them: how many arrived, how many headers it
 *                  restored, how many packets it dropped, how often and for how long its
 *                  context was lost, and "net saved": the bytes that arrived against the bytes
 *                  of the packets it could restore from them
 *   mode           when each compressor was in U-mode and when in O-mode, and which feedback
 *                  started O-mode
 *   packet types   the share of IR, UOR-2 and UO-0 packets
 * --rohcModel=table (default): the header sizes of the reference model, no compressor runs.
 *   UO-0, 3 bytes, in steady state; UOR-2 with extension 3, 13 bytes, a context update: on the
 *   first --rohcUpdatePackets (4) SIDs after the hangover and voice packets after a silence; IR,
 *   39 bytes, on the first --rohcIrPackets packets (4) of the call and of every new connection
 *   after a radio link failure. The repetitions are RFC 3095's optimistic approach (5.3.1.1.1):
 *   a compressor that gets no acknowledgement sends new context information several times in a
 *   row, so that one lost packet does not cost the context (the ROHC library sends 4).
 *   The context of the decompressor is counted in packets missing since the last packet it
 *   restored, lost on the air or dropped by itself:
 *     up to --rohcDynamicWindow (13)   nothing happens, the next packet is restored
 *     more                             the dynamic context is damaged: UO-0 packets are dropped
 *                                      until a context update or an IR arrives
 *     more than --rohcStaticWindow (61)  the static context is gone too: context updates are
 *                                      dropped as well, only an IR repairs it
 *   13 and 61 are what the 4 sequence number bits of UO-0 and the 6 of UOR-2 give the RTP
 *   profile of RFC 3095 (15 and 31 in its UDP profile). Huge values: no context is ever lost.
 *   --rohcMode=U: no feedback. The damaged context waits for the next context update the voice
 *     source causes, or for a refresh: the compressor sends IR again (--rohcIrPackets times)
 *     1700 packets after the last IR and context updates 700 packets after the last one
 *     (--ns3::NrPdcp::RohcIrRefresh, RohcFoRefresh; RohcIrRefreshTime for a period in time).
 *   --rohcMode=O (default): feedback in PDCP control PDUs on the reverse link, where it takes its
 *     time and can be lost. The decompressor answers every IR with an ACK. At the first packet
 *     that it cannot restore (--rohcNackAfter=1; n: after n such packets in a row) it sends a
 *     NACK, or a STATIC-NACK if the static context is gone, and again every 32 such packets
 *     (--rohcNackEvery). The compressor answers a NACK with --rohcUpdatePackets context updates
 *     and a STATIC-NACK with --rohcIrPackets IR packets, of which the ACK of the first that
 *     arrives ends the rest. No periodic refresh once feedback has arrived.
 *     Why not always at the first: RFC 3095 5.3.2.2.3 lets a decompressor take its context
 *     for damaged only when k of the last n packets failed their CRC, because one failure can
 *     also be a damaged packet or one it can still repair by itself; the ROHC library waits
 *     for 30 % of the last 32, which is --rohcNackAfter=10. In the size table a packet fails
 *     only when the context is lost, so waiting gains nothing and 1 is the default.
 *     This is not the NACK of HARQ. HARQ works on transport blocks in the MAC: the receiver
 *     answers every block, and a NACK brings the same block again some 20 ms later, up to
 *     three times. ROHC only comes into play when that was not enough: more packets in a row
 *     than the context survives were lost for good (--rohcDynamicWindow), packets arrive
 *     again, and the decompressor cannot restore their headers. Its NACK asks the compressor
 *     for one larger header.
 *   The mode of a compressor (RFC 3095 5.6.2): it starts in U-mode, whatever --rohcMode says,
 *     and is in O-mode from the moment any feedback of the decompressor reaches it (each is a
 *     FEEDBACK-2 with the CRC option, 5 bytes, and asks for O-mode). That is normally the ACK
 *     of the first IR, a few tens of ms into the call. If that ACK is lost on the reverse link
 *     the compressor stays in U-mode, with its periodic refreshes, until a NACK or STATIC-NACK
 *     or the ACK of its next IR refresh arrives. It never goes back to U-mode by itself: U-mode
 *     comes back with the new connection after a radio link failure.
 * --rohcModel=library: the ROHC library compresses the real headers. --rohcMode as above. The
 *   size-table options do not reach it: its number of IR packets is
 *   --ns3::NrPdcp::RohcRepetitions (4, which is also how often it repeats every change), and
 *   what its context survives follows from the packet formats it picks. Its other settings are
 *   NrPdcp attributes too: --ns3::NrPdcp::RohcCrcRepair=1 and so on. It counts failures in
 *   the last 32 packets, not in a row: --rohcNackAfter=n is n of the last 32, and without it
 *   it sends its NACK as it does by default, at 10 of the last 32. After a burst of lost
 *   packets the library sometimes puts back a wrong header (a wrong sequence number that its
 *   3-bit CRC lets pass). The receiver's UDP checksum would reject such a packet; NrPdcp drops
 *   it and counts it as failed ("header restored wrongly" in --rohcReport=full).
 *
 * Layer-2 headers, --l2Headers: "1+1+3" (default) is PDCP 1 + RLC 1 + MAC 3 = 5 bytes, the
 * overhead of TR 26.940 5.2.3 (7-bit PDCP sequence number, 5-bit RLC UM sequence number, MAC
 * subheader with the 16-bit length field); "2+2+2" is what the nr module sends by itself. The
 * 5-bit RLC sequence number has a reordering window of 16 PDUs: after 16 to 31 PDUs lost in a
 * row the receiver takes the next ones for old ones and discards them (TS 36.322 5.1.2.2.2).
 * These are the header formats of E-UTRA (TR 26.940 is about NB-IoT NTN). NR has the same
 * 5 bytes on a whole packet, as PDCP 2 (12-bit sequence number, TS 38.323) + RLC UM 1 (no
 * sequence number, TS 38.322) + MAC 2 (8-bit length, TS 38.321); its RLC UM has no such
 * window, and every segment after the first has 2 bytes more (the segment offset).
 *
 * Voice packets (--rtp=1, default): every call is an RTP stream, so each packet has 40 bytes of
 * IP/UDP/RTP header before ROHC and ROHC uses its RTP profile. The codec is AMR 4.75 kbit/s:
 * 95 bits per 20 ms frame = 12 bytes (--codecKbps, or --voicePayload in bytes), SID frames are
 * 56 bits = 7 bytes (--sidPayload). The 12 bytes are the bits of the codec alone: the RTP
 * payload of RFC 4867 has 10 bits more in front (CMR and table of contents), which makes 14
 * bytes (--voicePayload=14); the 7 bytes of the SID (39 bits) have room for them. AMR itself
 * sends SID_FIRST, a SID_UPDATE three frames later and then one every 8 frames (TS 26.093
 * 5.1.2.1); here it is one SID at once and one every 160 ms.
 * DTX as in TR 26.940 6.4.3: while the voice activity detector
 * says "talk", one voice frame per 20 ms; when the talk stops, 7 more voice frames (hangover,
 * --hangoverFrames); then DTX: one SID at once and every 160 ms (--sidIntervalMs) until the talk
 * comes back. The voice activity factor of a call is that of its detector, so with the hangover
 * a call sends voice frames a little more often than its VAF says.
 * --rtp=0 is the NGMN generator of the nr module instead: no RTP header (28 bytes of IP/UDP), no
 * hangover. With ngmn-voip-20ms.patch in contrib/nr it sends one packet per 20 ms frame while
 * talking; without the patch it keeps 12.2 kbit/s, so 12-byte packets leave every 7.87 ms. The
 * run prints which of the two it was.
 *
 * The PDCCH as a resource, --pdcch=1 (class NrPdcch of the nr module that comes with this file;
 * the 3GPP rules it follows are in pdcch-sps-3gpp-review.md). Without it a slot carries as many
 * DCIs as the scheduler likes; with it:
 *   - CORESET: all whole groups of 6 resource blocks of the downlink carrier, over
 *     --pdcchSymbols = 1, 2 or 3 symbols. A CCE is 6 resource blocks of one symbol, so there
 *     are RBs x symbols / 6 of them (TS 38.211 7.3.2.2). --pdcchSymbols=0: every slot takes
 *     the fewest symbols that hold its DCIs, and the data starts right after them.
 *   - DCI: --dciBits (34) plus a 24-bit CRC. A CCE has 108 coded bits (54 QPSK symbols).
 *   - aggregation level: 1, 2, 4, 8 or 16 CCEs. A UE reports the SINR of its control symbols
 *     every 10 ms in steps of 0.5 dB (as L1-SINR, TS 38.214 5.2.1.4), and a DCI for it gets
 *     the lowest level whose block error rate at that SINR is 1 % or less. The rate comes from
 *     --pdcchBlerTable, "SINR:BLER ..." in dB for level 1 and a 34-bit DCI; level L is the
 *     same curve 10 log10(L) dB lower, a DCI of other size 10 log10((bits+24)/58) dB higher.
 *   - search space: a UE has 6/6/4/2/1 candidates at level 1/2/4/8/16, at the CCEs the hash of
 *     TS 38.213 10.1 gives its RNTI in that slot. A DCI needs a candidate with all CCEs free;
 *     if the level has none, the next higher is tried. If none is free the DCI is "blocked":
 *     it is not sent, and its UE is not scheduled in that slot.
 *   - the cell's own DCIs (NrPdcch makes them; they take CCEs, not resource blocks):
 *       SIB1             every 20 ms (Sib1PeriodMs), common search space
 *       other SI         an SI message every period of SiPeriodsMs (160,160,320: SIB2-4,
 *                        SIB19 of the NTN cell, the rest), common search space
 *       paging           the paging occasion of a frame (PagingCycleMs 1280, PagingFrames:
 *                        every frame) has a paging message if any of its UEs is paged; with
 *                        PagingRate (5) pages a second in the beam, at random
 *       DCI 2_2          every TpcPeriodMs (40) one per TpcUesPerDci (13) UEs with an SPS or
 *                        configured grant configuration: their power control commands (a UE
 *                        scheduled by DCIs gets its commands in them)
 *       RRC              RrcSetupDcis (4:5, downlink:uplink) when a UE connects (RRCSetup,
 *                        security, capabilities, reconfiguration, with the answers),
 *                        RrcCallDcis (1:1) when its call starts (the reconfiguration of the
 *                        QoS flow), one every RrcStepMs (20); a measurement report every
 *                        MeasReportPeriodMs (0: none)
 *     One that finds no free candidate waits for the next slot.
 *   - who comes first in a slot: the cell's DCIs (common search space, level 8 or the highest
 *     the CORESET holds; the RRC ones at the UE's level), the random access response, the
 *     uplink grants, the downlink
 *     assignments; in each direction retransmissions first, then new data in the order of the
 *     scheduler's policy (round robin: the UE served longest ago first).
 *     Uplink grants may take the CCEs the downlink did not ask for in the slot before, or on
 *     average half of the CORESET (NrPdcch::UlMaxShare), whichever is more.
 *   - --idealControl=0: a DCI is lost with the block error rate of its level at the SINR it
 *     arrives with, and the radio link monitoring uses the same table for its two hypothetical
 *     PDCCHs (TS 38.133 8.1.2: level 8 with 4 dB more power at 10 %, level 4 at 2 %).
 *   - the PDCCH report: the CCEs used out of all the CCEs possible (every slot of the calls
 *     with all the PDCCH symbols it may have: --pdcchSymbols, or 3 with --pdcchSymbols=0), and
 *     every share of CCEs as a percentage of that same total; DCIs and CCEs by purpose with
 *     the blocked ones, and by aggregation level with the DCIs missed.
 *   Other settings are attributes: --ns3::NrPdcch::CoresetRbs, Candidates, TargetBler,
 *   SinrMarginDb, SinrOffsetDb, TableDciBits, CommonAl, RaiseAl, UlMaxShare and those of the
 *   cell's DCIs above (--ns3::NrPdcch::PagingRate=20 and so on).
 *
 * Voice scheduling, --scheduling (needs --pdcch=1; every scheduler of the nr module has it)
 *   - dynamic (default): every transport block has a DCI.
 *   - sps1: semi-persistent, scenario 1. One configuration: a block every 20 ms that holds one
 *     voice packet in the steady state of ROHC, activated by one DCI and then sent without
 *     any. It stays while the speaker is silent, so most blocks are then empty. A packet
 *     larger than the block (the first of a silence or of a talk spurt has the ROHC context
 *     update): a silence descriptor goes in pieces over this block and the next; a voice frame
 *     gets a DCI of its own, because the next frame is already due at the next block.
 *   - sps2: scenario 2. Two configurations, talk (20 ms) and silence (--sidIntervalMs). When
 *     the speaker starts or stops, one DCI releases the active configuration and a second
 *     activates the other, with a block large enough for the first packet.
 *   Downlink is SPS (TS 38.321 5.8.1), uplink is configured grant Type 2 (5.8.2): the same, and
 *   the UE confirms every activation and release with a MAC CE (1 byte with one configuration,
 *   6 bytes with two: TS 38.321 6.1.3.7 and 6.1.3.31), leaves a block unused when it has
 *   nothing to send, and fills a block one slot before it sends it. The gNB tells talk from
 *   silence by the size of the packets it sends or has received. In uplink it learns of a new
 *   talk spurt by the scheduling request, answers that with a DCI, and starts the
 *   configuration 2 ms after the next packet is due. Retransmissions always have a DCI.
 *   A block has the size of the fewest resource blocks that hold it. An OFDMA scheduler gives
 *   it just these; a TDMA scheduler gives every block whole symbols, and so does the uplink,
 *   with the MCS the transmit power of the UE gives over the whole band. A configuration is
 *   activated again (one DCI) when the MCS of the UE has moved from its own by more than
 *   SpsMcsTolerance. If the link is worse and this cannot be done where the configuration is
 *   (no room for the larger block in its slots, or no PDCCH candidate in this one), it is
 *   activated again in a later slot that has both; in downlink it is left unused until then.
 *   Attributes: --ns3::NrMacSchedulerNs3::SpsTbBytes, SpsSilenceTbBytes, SpsFitExtraBytes,
 *   SpsMcsTolerance. The report after the PDCCH tables counts the blocks that needed no DCI,
 *   and, in the table VOICE <-> SILENCE, every time a speaker started or stopped talking: how
 *   the first packet after it was sent (whole in a block; in pieces, with the RLC PDUs; with a
 *   dynamic grant) and whether the configuration was changed, with the DCIs this took.
 *
 * Scheduler, --scheduler. ns3::NrMacSchedulerOfdmaRR is the default: frequency division (a UE
 * gets resource block groups, over all data symbols of the slot) and round robin. The nr module
 * has ten: NrMacSchedulerOfdma or NrMacSchedulerTdma (a UE gets whole symbols of a slot, over
 * the whole band) with RR, PF (proportional fair), MR (highest MCS first), Qos (PF weighted by
 * the priority of the 5QI) or Random. In a slot the retransmissions come first,
 * then new data: the policy shares the symbols or RBGs out one at a time among the UEs with
 * data, to the UE it ranks first, and no UE gets more than it has data for. Between UEs it
 * ranks the same, the one served longest ago is first.
 * A scheduling request is answered with a grant for --srGrantBytes: by default a voice frame
 * with its headers and the buffer status report the UE sends with it, so that the packet that
 * made the UE ask goes in that grant. (17 B, the nr module's, holds the report and little else:
 * the packet then waits a round trip for the grant the report brings.)
 * A UE whose scheduling requests go unanswered keeps asking. After sr-TransMax of them TS 38.321
 * 5.4.4 has it start a random access, in which a connected UE keeps its C-RNTI (5.1.5); the nr
 * module gives it a new RNTI instead, and its downlink stops. So there is no fallback, unless
 * the command line sets --ns3::NrUeMac::SrTransMax.
 *
 * Error model, --errorModel: how the SINRs of a block's RBs give its error rate, and with it the
 * MCS table; one model for every link, satellite and terrestrial.
 *   NrLteMiErrorModel (default): LTE's. MCS 0-28 (QPSK, 16QAM, 64QAM; code rates from 0.08),
 *       turbo-code curves, the mutual information of the RBs (MIESM).
 *   NrEesmIrT1, NrEesmCcT1: NR's. MCS Table 1 of TS 38.214 (5.1.3.1-1: QPSK to 64QAM, code
 *       rates from 0.12), LDPC with NR's base graphs and code blocks (TS 38.212), curves from
 *       CTTC's NR link-level simulator, EESM. ...T2: MCS Table 2 (5.1.3.1-2), up to 256QAM.
 *   On the narrow uplink grant the NR models take a lower MCS at the same SINR: UE 0 at 30 deg,
 *   0 dBi, 2 RB (3.62 dB): MCS 9 and 33 B with NrLteMiErrorModel, MCS 4 and 18 B with
 *   NrEesmIrT1 (the first choice of the AMC; the outer loop then moves it).
 *
 * HARQ, --harq. A transport block is sent up to 4 times, and the receiver combines all
 * transmissions it has: incremental redundancy (NrLteMiErrorModel adds their mutual information
 * and code bits, NrEesmIr* their effective SINR and code bits) or chase combining (NrEesmCc*).
 * Every retransmission has a DCI; it keeps MCS and size. --harqProcesses (16) per UE and
 * direction (Rel-17 allows an NTN UE up to 32); a UE whose processes all wait for feedback gets
 * no new data until one is free. With an OFDMA scheduler a retransmission gets resource block
 * groups over the symbols the new data of the slot has (as many as hold the resource elements
 * it had, its own first if they are free), and the other UEs get new data in the rest of the
 * band: a retransmission does not keep them out of the slot.
 * (--ns3::NrMacSchedulerOfdma::HarqRetxInFrequency=0: symbols of its own, as in the release.)
 *   downlink   the UE answers a block K1 = 2 + K_offset slots after it (an ideal PUCCH: the
 *              answer is never lost), and the gNB can retransmit from the next slot on
 *   uplink     the gNB decodes and grants the retransmission itself; a grant is for the slot
 *              K2 = 2 + K_offset slots ahead
 * K_offset (TS 38.213 4.2, Rel-17) is the longest round trip of the run between a UE and the
 * gNB in whole slots, so a retransmission comes 20 ms or more after the transmission before
 * it, and a block that needs all four takes 60 ms and more of the 100 ms of 5QI 1. A block
 * that fails four times is dropped; RLC UM does not send it again. The MCS of the first
 * transmission aims at 15 % block errors (--targetBler; in downlink an outer loop on the HARQ
 * feedback keeps it there).
 * --harq=0: every block is sent once, and a block that fails is lost. In downlink the process
 * is free again at once (downlinkHARQ-FeedbackDisabled of Rel-17, for all processes). The
 * first transmission then aims at 1 % block errors, the error rate of 5QI 1, and the RLC
 * receiver does not hold packets back for one that is missing (with HARQ it waits up to
 * 100 ms, the time the retransmissions of the missing one can take).
 *
 * Uplink grant, --ulGrantRbs (OFDMA). A UE's uplink grant has at most 2 RB = 360 kHz (the
 * default; the uplink of TR 38.821's link budgets), with all the UE's power in them; 0: as many
 * as the scheduler gives. A UE the gNB has not measured yet gets a grant as wide as its data
 * needs (the gNB has nothing to choose the MCS of a narrow one from), afterwards --ulGrantRbs at
 * most. The nr module grants no uplink block under 12 B: with the -5 dBi handheld, UE 0's 2 RB
 * at 30 deg carry MCS 4 = 10 B in one slot, so without repetition almost no voice goes up (a
 * NOTE in the header says so); with --ueAntennaGainDbi=0 (TR 38.821's handheld) they carry
 * MCS 9 = 33 B. 3GPP's way out is PUSCH repetition, next (TR 38.830 also has TBoMS, which this
 * model does not have).
 *
 * PUSCH repetition, --puschRepetitions (type A, TS 38.214 6.1.2.1; Rel-17: up to 32). A UE sends
 * an uplink block in K slots in a row, the K - 1 repetitions without a DCI, and the gNB combines
 * them as it combines HARQ transmissions (incremental redundancy) and decodes after the last; a
 * HARQ retransmission is sent K times too. 0 (the default): K per UE and grant, the fewest of
 * 1, 2, 3, 4, 7, 8, 12, 16, 20, 24, 28, 32 that bring K times the SINR of the grant's resource
 * blocks to --puschRepSinrDb (2 dB); 1: no repetition; 2 to 32: that K for every block. The MCS
 * is the highest the error model decodes with 10 % block errors after the K transmissions (then
 * the outer loop). A repetition takes the resource blocks of the first transmission if they are
 * free, others if not. UE 0 at 30 deg with the -5 dBi handheld: K = 3, MCS 9 = 33 B. A
 * configured grant (--scheduling=sps1, sps2) repeats its blocks too, repK times (TS 38.214
 * 6.1.2.3; ConfiguredGrantConfig of TS 38.331 allows 1, 2, 4 or 8): the fewest of these that
 * bring repK times the SINR to --puschRepSinrDb (with a fixed K, the first that is at least K),
 * or more if the block does not fit into its RBs at the MCS they give; set when the
 * configuration is activated, which is done anew when the link needs more, or with 1 dB to
 * spare fewer. Without them a block of 26 B would not fit into 2 RB at the MCS of one
 * transmission, and no UE of the -5 dBi handheld would get a configured grant.
 * NrLteMiErrorModel has curves down to the code rate of 4 transmissions of a block; below it, a
 * block takes the lowest curve of its modulation with the mutual information scaled by that
 * curve's code rate over its own.
 * A TDMA scheduler grants a UE all resource blocks of one or two symbols: the 23 dBm of a
 * handheld spread over 30 MHz arrive at -11 to -21 dB SINR, below what the lowest MCS needs, and
 * what gets through does so after one to three HARQ retransmissions. Use --ulGrantRbs=0 with it.
 *
 * Power.
 *   Downlink: the satellite's power is shared by the resource blocks of the carrier (48.59 dBm
 *   into the antenna for 160 RB: 26.55 dBm each). An RB in use gets its share, an idle RB sends
 *   nothing; the control symbols take the whole band. --dlBoostMaxDb (OFDMA): an RB in use also
 *   takes the power the idle RBs of its symbol leave, up to that many dB over its share (0: none,
 *   the default). --dlPowerRefDeg: power control, the satellite sends full power when it is at
 *   that elevation above the beam centre and lowers it by what the free-space loss and gases to
 *   the centre are below their values there; never more than full power (0: off, the default).
 *   Uplink: the UE sends 23 dBm, spread over the resource blocks of its grant. --ulPowerRefDeg:
 *   open-loop power control of TS 38.213 7.1.1 with alpha = 1: P0 + 10 log10(M) + PL, PL
 *   measured on the downlink (without fast fading), P0 such that a UE on the beam centre with
 *   the satellite at that elevation sends all 23 dBm; never more (0: off, the default).
 * Uplink link adaptation, --ulLinkAdaptation=1 (default). The UE spreads its power over the
 * resource blocks of its grant: the MCS of a grant is that of the
 * SINR the gNB measured on the UE's last transmission, scaled by the resource blocks then over
 * those now, and an outer loop on the decoding results (the same steps as in downlink; blocks
 * whose grant the UE missed do not count) keeps the first transmissions at --targetBler.
 * --ulLinkAdaptation=0, the nr module's: the MCS of that SINR as it was, whatever the size of
 * the grant, and no outer loop: a UE that gets more resource blocks than it was measured on
 * loses about half its first transmissions.
 *
 * Call control: what a network does when the cell has more calls than it can carry.
 *   - admission, --admission=1 (default): when a call is to start, the gNB estimates what it
 *     will take while its speakers talk, per second and in each direction it carries: 50 voice
 *     packets (one per 20 ms frame) with their HARQ retransmissions (1 / (1 - --targetBler)),
 *     each packet the resource blocks its UE's MCS needs for it (downlink: from the SINR the UE
 *     has measured so far; uplink: that less the difference of the two CNRs of the header, on
 *     the grant's RBs, and K slots of PUSCH repetition), and with --pdcch=1 a DCI at the UE's
 *     aggregation level for each (semi-persistent scheduling: for the retransmissions only).
 *     It admits the call if the calls it carries, with this one, reserve at most
 *     --admissionRbPct (100) % of the downlink RBs and of the uplink RBs, and at most
 *     --admissionCcePct (60) % of the PDCCH's CCEs; else the call is blocked: it never starts.
 *     A call gives its reservation back when it ends or is released.
 *     The RBs: 100 % is what admission of GBR bearers does, the GFBR of every call reserved
 *     as if all speakers talked at once. Voice activity leaves room: a conversation keeps each
 *     direction busy about half the time, so the cell carries more than this reservation says,
 *     and --admissionRbPct above 100 lets it take more calls (and run full more often).
 *     The CCEs: a PDCCH runs out long before all its CCEs are in use. A DCI needs a free
 *     candidate of its own UE's search space, the uplink gets at most half the CCEs while the
 *     downlink wants them too, and the DCIs of a slot come in bursts; a DCI that finds no room
 *     waits a slot, and the waits add up. With the 1-symbol CORESET of 30 MHz (26 CCEs) and
 *     dynamic scheduling, in runs of 6 to 8 s: 60 calls reserve 45 % of the CCEs and all
 *     meet 5QI 1; 82 calls, 60 %: 96 % of their directions meet it; 100 calls, 73 %: 76 %;
 *     150 calls, 110 %: 54 %. Hence the 60.
 *   - release, --dropCalls=1 (default): every 100 ms the gNB looks at the packets of the last
 *     averaging window of 5QI 1 (2 s, TS 23.501 Table 5.7.4-1) on each call's satellite stretch;
 *     when fewer than --dropOnTimePct (80) % of them, in either direction, arrived within the
 *     PDB, the GFBR can no longer be guaranteed (TS 23.501 5.7.2.4, notification control) and
 *     the call is released: its voice ends and the gNB releases the UE's RRC connection (the UE
 *     connects again, without a call). Counted are the packets that arrived (within the PDB or
 *     late) and those the sender's RLC discarded for waiting too long: a full cell delays
 *     packets. A packet lost on the radio does not count: a bad link is not a full cell, and a
 *     link that fails is a radio link failure. Not while the UE is out of coverage after one.
 *     The estimate of the admission has no room for what the cell sends of its own (SIB1, DCI
 *     2_2 ...) and for the DCIs that semi-persistent scheduling still needs; where these are
 *     many against a small PDCCH, the release takes care of the rest: 60 sps1 calls on 5 MHz
 *     (4 CCEs), --admissionRbPct=80: 41 admitted, 7 of them released.
 *   The CALL CONTROL table lists the calls blocked and released.
 *
 * CSV files, --csv=<prefix>: the results also as CSV files, for plots. Every line starts with
 * the columns that identify the run (nUes, direction, scheduling, ..., farEnd, farEndBand,
 * ulGrantRbs, puschRepetitions, puschRepSinrDb, dlBoostMaxDb, dlPowerRefDeg, ulPowerRefDeg,
 * errorModel, rngSeed, rngRun), so the files of many runs can be joined (sweep_nues.sh does
 * that for a range of --nUes; plot_sweep.py draws them).
 *   <prefix>_calls.csv       a line per flow (dir: dl, ul) of a call (bearer 0) and per bearer
 *                            of it (1, 2, ...: a new one after each radio link failure): the
 *                            CALLS, LOST and 5QI tables, the delay percentiles, with --pdcch=1
 *                            the DCIs to the UE (two-way calls: those of the flow's direction),
 *                            the far end's stretch (far...) and the whole way (e2e...); the
 *                            call admitted, blocked or released (callStatus, releasedAtS)
 *   <prefix>_control.csv     the CALL CONTROL table: calls offered, admitted, blocked (and for
 *                            which resource), released; reservations, one line
 *   <prefix>_delays.csv      per flow: the delivered packets per 1 ms of UE-UPF delay
 *   <prefix>_packets.csv     --csvPackets=1: a line per delivered packet
 *   <prefix>_pdcch.csv       --pdcch=1: the PDCCH report, one line
 *   <prefix>_rohc.csv        ROHC on, --rohcReport=summary: a line per connection (RNTI) and
 *                            direction of a bearer
 *   <prefix>_rohc_types.csv  ROHC on: the ROHC packet types of each connection
 *
 * Run:
 *   ./ns3 run "ntnlab_nr"                              10 two-way calls with the remote host,
 *                                                      30 deg, UE 0 on the beam centre and 9 UEs
 *                                                      dropped at random
 *   ./ns3 run "ntnlab_nr --ueAntennaGainDbi=0"         TR 38.821's handheld: 2 RB carry the voice
 *   ./ns3 run "ntnlab_nr --farEnd=ue"                  the other end: terrestrial UEs on n78
 *   ./ns3 run "ntnlab_nr --farEnd=ue --farEndBand=n3"  ... on n3 (FDD)
 *   ./ns3 run "ntnlab_nr --farEnd=ue --farEndBandwidthMHz=100 --farEndScenario=UMi"
 *                                                      100 MHz cells, UMi with LOS probability
 *   ./ns3 run "ntnlab_nr --direction=dl"               one-way calls: the far end talks
 *   ./ns3 run "ntnlab_nr --ulGrantRbs=0"               uplink grants as wide as the scheduler gives
 *   ./ns3 run "ntnlab_nr --dlBoostMaxDb=3"             RBs in use take up to 3 dB from idle ones
 *   ./ns3 run "ntnlab_nr --freezeGeometry=0 --duration=100s --dlPowerRefDeg=30 --ulPowerRefDeg=30"
 *                                                      power control as the satellite flies
 *   ./ns3 run "ntnlab_nr --errorModel=NrEesmIrT1"      NR's MCS table and LDPC curves
 *   ./ns3 run "ntnlab_nr --RngRun=2"                   another drop (and other calls)
 *   ./ns3 run "ntnlab_nr --ueLayout=line"              UEs in a line, 2 km apart
 *   ./ns3 run "ntnlab_nr --ueLayout=line --elevation=60 --nUes=50 --spacingKm=0.4"
 *   ./ns3 run "ntnlab_nr --freezeGeometry=0 --duration=100s"   the satellite flies: 30 -> 72 deg
 *   ./ns3 run "ntnlab_nr --freezeGeometry=0 --elevation=10 --duration=60s"   it rises from 10 deg
 *   ./ns3 run "ntnlab_nr --fullPass=1 --nUes=2"        the whole pass over UE 0, 10 deg to 10 deg
 *   ./ns3 run "ntnlab_nr --satSet=2"                   Set-2 satellite: 28 dBW/MHz, 24 dBi
 *   ./ns3 run "ntnlab_nr --altitudeKm=1200"            LEO-1200 of the set: 40 dBW/MHz
 *   ./ns3 run "ntnlab_nr --satEirpDbwPerMHz=31"        the set, with 3 dB less power
 *   ./ns3 run "ntnlab_nr --dlFrequencyGHz=4.175"       same 2 m antenna: the beam is 2.11 deg wide
 *   ./ns3 run "ntnlab_nr --dlFrequencyGHz=4.175 --beamHpbwDeg=4.4127"   the beam kept as at 2 GHz
 *   ./ns3 run "ntnlab_nr --direction=ul"               one-way calls: the UE talks
 *   ./ns3 run "ntnlab_nr --gatewayDiameterKm=1000"     the gateway within 500 km of UE 0
 *   ./ns3 run "ntnlab_nr --gatewayDiameterKm=0"        the gateway where UE 0 is
 *   ./ns3 run "ntnlab_nr --gatewayGainDbi=55"          a 2.4 m gateway antenna (OneWeb's typical)
 *   ./ns3 run "ntnlab_nr --transparent=0"              gNB on the satellite, no feeder link
 *   ./ns3 run "ntnlab_nr --scenario=NTN-Rural"         with blocking; or NTN-Urban, -DenseUrban
 *   ./ns3 run "ntnlab_nr --scenario=NTN-Urban --freezeGeometry=0 --duration=100s"
 *                                                      blocked UEs clear as the satellite rises
 *   ./ns3 run "ntnlab_nr --shadowing=table"            each UE's own shadow fading, not 3 dB
 *   ./ns3 run "ntnlab_nr --atmosphere=location"        gases by elevation, latitude 45 deg
 *   ./ns3 run "ntnlab_nr --atmosphere=location --latitudeDeg=10"   near the magnetic equator
 *   ./ns3 run "ntnlab_nr --fastFading=model"           3GPP fast fading per UE, RB and slot
 *   ./ns3 run "ntnlab_nr --fastFadingDb=2"             a flat 2 dB on every RB instead
 *   ./ns3 run "ntnlab_nr --linkMarginDb=3"             3 dB more loss on every link
 *   ./ns3 run "ntnlab_nr --idealControl=0"             scheduling messages are lost at low SINR
 *   ./ns3 run "ntnlab_nr --ns3::NrRlcUm::DiscardTimerMs=100"   UEs discard late packets too
 *   ./ns3 run "ntnlab_nr --rohc=0"                     without header compression
 *   ./ns3 run "ntnlab_nr --nUes=1 --rohcReport=full"   every ROHC detail of the call
 *   ./ns3 run "ntnlab_nr --rohcIrPackets=3 --rohcDynamicWindow=5 --rohcStaticWindow=30"
 *   ./ns3 run "ntnlab_nr --rohcMode=U --ns3::NrPdcp::RohcIrRefresh=250"   no feedback, IR every
 *                                                      5 s of talk
 *   ./ns3 run "ntnlab_nr --rohcDynamicWindow=1000000 --rohcStaticWindow=1000000"   a context
 *                                                      that never fails
 *   ./ns3 run "ntnlab_nr --rohcModel=library --l2Headers=2+2+2"   real ROHC, the module's headers
 *   ./ns3 run "ntnlab_nr --rohcModel=library --rohcMode=U"        real ROHC without feedback
 *   ./ns3 run "ntnlab_nr --codecKbps=12.2"             AMR 12.2: 244 bits = 31 bytes per frame
 *   ./ns3 run "ntnlab_nr --hangoverFrames=0"           DTX without hangover
 *   ./ns3 run "ntnlab_nr --rtp=0"                      NGMN generator, no RTP header
 *   ./ns3 run "ntnlab_nr --pdcch=1"                    PDCCH: CCEs, aggregation levels, blocking
 *   ./ns3 run "ntnlab_nr --pdcch=1 --bandwidthMHz=5 --nUes=40"   a small CORESET that fills up
 *   ./ns3 run "ntnlab_nr --pdcch=1 --pdcchSymbols=0"   as many PDCCH symbols as each slot needs
 *   ./ns3 run "ntnlab_nr --pdcch=1 --idealControl=0"   DCIs lost at the rate of their level
 *   ./ns3 run "ntnlab_nr --pdcch=1 --scheduling=sps1"  semi-persistent, one configuration
 *   ./ns3 run "ntnlab_nr --pdcch=1 --scheduling=sps2 --direction=ul"   configured grant, two
 *   ./ns3 run "ntnlab_nr --pdcch=1 --pdcchBlerTable='-4:0.93 -2:0.44 0:0.025 1:0.0022'"
 *   ./ns3 run "ntnlab_nr --scheduler=ns3::NrMacSchedulerOfdmaPF"   OFDMA, proportional fair
 *   ./ns3 run "ntnlab_nr --scheduler=ns3::NrMacSchedulerTdmaRR --ulGrantRbs=0"   TDMA
 *   ./ns3 run "ntnlab_nr --harq=0"                     no HARQ retransmissions
 *   ./ns3 run "ntnlab_nr --upfGwDelayMinMs=5 --upfGwDelayMaxMs=5"   UPF to gateway: 5 ms of the 20
 *   ./ns3 run "ntnlab_nr --hostGwDelayMs=40 --upfGwDelayMaxMs=20"   a remote host further away
 *   ./ns3 run "ntnlab_nr --rohcNackAfter=10"           the ROHC NACK as the ROHC library times it
 *   ./ns3 run "ntnlab_nr --nrTraces=1" --cwd=results/t1   also write the NR trace files there
 *   ./ns3 run "ntnlab_nr --pdcch=1 --csv=results/r1"   also results/r1_calls.csv etc.
 */

#include "ns3/angles.h"
#include "ns3/applications-module.h"
#include "ns3/channel-condition-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/geocentric-ecef-mobility-model.h"
#include "ns3/geographic-positions.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/leo-circular-orbit-mobility-model.h"
#include "ns3/matrix-based-channel-model.h"
#include "ns3/network-module.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-ntn-link-models.h"
#include "ns3/point-to-point-module.h"
#include "ns3/three-gpp-antenna-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/traffic-generator-helper.h"
#include "ns3/traffic-generator-ngmn-voip.h"
#include "ns3/uniform-planar-array.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <tuple>
#include <vector>

using namespace ns3;

namespace
{

std::vector<double> g_dlSinrSum; // one entry per UE, linear
std::vector<uint32_t> g_dlSinrN;

// The SNR of a satellite UE during its call (SNR table): lowest, highest, mean of the dB values
struct SnrStats
{
    double minDb{INFINITY};
    double maxDb{-INFINITY};
    double sumDb{0.0};
    uint64_t n{0};

    void Add(double db)
    {
        minDb = std::min(minDb, db);
        maxDb = std::max(maxDb, db);
        sumDb += db;
        ++n;
    }

    double MeanDb() const
    {
        return n > 0 ? sumDb / n : NAN;
    }
};

std::vector<SnrStats> g_dlSnr;   // per UE: of the control symbols it receives
std::vector<SnrStats> g_ulSnr;   // per UE: of its PUSCH transmissions, scaled to g_ulSnrRbs RBs
std::vector<SnrStats> g_ulRxSnr; // the same as they came
std::vector<std::pair<Time, Time>> g_snrWindow; // per UE: its call
std::map<uint16_t, uint32_t> g_rntiUe;          // the satellite cell's RNTI of each UE now
double g_ulSnrRbs{2.0};

bool
InSnrWindow(uint32_t ue)
{
    const Time now = Simulator::Now();
    return now >= g_snrWindow[ue].first && now < g_snrWindow[ue].second;
}

// "DlCtrlSinr" of UE i's downlink PHY: the SINR of every control symbol the UE receives, so it
// is there for uplink calls too. i is bound when connecting.
void
DlSinr(uint32_t ue, uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
{
    g_dlSinrSum[ue] += sinr;
    ++g_dlSinrN[ue];
    if (sinr > 0.0 && InSnrWindow(ue))
    {
        g_dlSnr[ue].Add(10.0 * std::log10(sinr));
    }
}

// "RxPacketTraceGnb" of the satellite gNB: every PUSCH transmission it receives (a repetition is
// one), with the SINR over its resource blocks. One the UE did not send (it missed the DCI)
// has no signal and is left out.
void
UlSinr(RxPacketTraceParams p)
{
    const auto it = g_rntiUe.find(p.m_rnti);
    if (it == g_rntiUe.end() || p.m_pdcchMissed || p.m_sinr <= 0.0 || p.m_rbAssignedNum == 0 ||
        !InSnrWindow(it->second))
    {
        return;
    }
    // The UE's power is spread over the RBs of the grant: on g_ulSnrRbs it has that many times less
    const double db = 10.0 * std::log10(p.m_sinr);
    g_ulRxSnr[it->second].Add(db);
    g_ulSnr[it->second].Add(db + 10.0 * std::log10(p.m_rbAssignedNum / g_ulSnrRbs));
}

// The same for the terrestrial UE k (--farEnd=ue)
std::vector<double> g_farSinrSum;
std::vector<uint32_t> g_farSinrN;

void
FarDlSinr(uint32_t ue, uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
{
    g_farSinrSum[ue] += sinr;
    ++g_farSinrN[ue];
}

// --rtp=0, "Tx" of call i's NGMN generator: the shortest time between two packets of a call. A
// talking generator has to send one packet per 20 ms frame; that needs ngmn-voip-20ms.patch in
// contrib/nr. Without it the generator keeps 12.2 kbit/s while talking, so its packets leave
// every VoicePayload x 8 / 12200 s (12 bytes: 7.87 ms). In silence the gap is 160 ms.
std::vector<Time> g_lastVoiceTx;
Time g_shortestVoiceGap = Time::Max();

void
VoiceSent(uint32_t call, Ptr<const Packet> packet)
{
    const Time now = Simulator::Now();
    if (!g_lastVoiceTx[call].IsStrictlyNegative())
    {
        g_shortestVoiceGap = std::min(g_shortestVoiceGap, now - g_lastVoiceTx[call]);
    }
    g_lastVoiceTx[call] = now;
}

// "RadioLinkFailure" of UE i's RRC: the UE gave the link up (out of sync for too long) and
// has to attach again. A call with failures is not a steady call any more.
std::vector<uint32_t> g_rlf;

// The times UE i had no connection: from each radio link failure until its RRC connection is
// there again ("ConnectionEstablished" of its RRC), Time::Max() if that never happened.
std::vector<std::vector<std::pair<Time, Time>>> g_outages;

// The two directions of a call, as the satellite UE sees them: DOWN = the far end talks to the
// UE (the satellite's downlink carries it), UP = the UE talks to the far end
enum Dir : uint8_t
{
    DOWN = 0,
    UP = 1,
};

// The connections of UE i, one per "ConnectionEstablished" of its RRC: when it was set up, its
// RNTI, and what NrPdcp had counted for the UE by then (its earlier connections), in each
// direction. A radio link failure ends a connection and its voice bearer; the next connection
// sets up a new bearer, and the counts between two connections are those of the bearer of the
// first.
struct Connection
{
    Time start;
    uint16_t rnti{0};
    std::array<NrPdcp::PacketCounts, 2> before; // DOWN, UP
};

std::vector<std::vector<Connection>> g_connections;
std::vector<Ipv4Address> g_ueAddress; // of UE i, for NrPdcp::GetPacketCounts()

void
RadioLinkFailure(uint32_t ue, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    ++g_rlf[ue];
    g_outages[ue].emplace_back(Simulator::Now(), Time::Max());
}

void
ConnectionEstablished(uint32_t ue, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    if (!g_outages[ue].empty() && g_outages[ue].back().second == Time::Max())
    {
        g_outages[ue].back().second = Simulator::Now();
    }
    g_connections[ue].push_back({Simulator::Now(),
                                 rnti,
                                 {NrPdcp::GetPacketCounts(g_ueAddress[ue], true),
                                  NrPdcp::GetPacketCounts(g_ueAddress[ue], false)}});
    g_rntiUe[rnti] = ue;
}

// The connection UE i has now (index into g_connections[i]), -1 while it has none
int32_t
CurrentConnection(uint32_t ue)
{
    const bool out = !g_outages[ue].empty() && g_outages[ue].back().second == Time::Max();
    return out ? -1 : static_cast<int32_t>(g_connections[ue].size()) - 1;
}

// --csv: the DCIs each UE received, by RNTI (a connection), over the window of the PDCCH report
// (the calls): those addressed to it on the PDCCH (not the blocks of an SPS or configured-grant
// configuration, which have none), their CCEs, and how many it did not decode
struct UeDciCount
{
    uint64_t dcis{0};
    uint64_t cces{0};
    uint64_t missed{0};
};

std::map<uint16_t, std::array<UeDciCount, 2>> g_dciByRnti; // RNTI: DL DCIs, UL DCIs
bool g_dciCounting = false;

void
UeDciReceived(SfnSf sfn,
              uint16_t cellId,
              uint16_t rnti,
              uint8_t bwp,
              Ptr<const NrControlMessage> msg)
{
    if (!g_dciCounting || rnti == 0)
    {
        return;
    }
    std::shared_ptr<DciInfoElementTdma> dci;
    if (msg->GetMessageType() == NrControlMessage::DL_DCI)
    {
        dci = DynamicCast<NrDlDciMessage>(ConstCast<NrControlMessage>(msg))->GetDciInfoElement();
    }
    else if (msg->GetMessageType() == NrControlMessage::UL_DCI)
    {
        dci = DynamicCast<NrUlDciMessage>(ConstCast<NrControlMessage>(msg))->GetDciInfoElement();
    }
    if (!dci || dci->m_rnti != rnti || dci->m_type != DciInfoElementTdma::DATA ||
        dci->m_sps == DciInfoElementTdma::SPS_OCCASION || dci->m_pdcchAl == 0)
    {
        return;
    }
    UeDciCount& c = g_dciByRnti[rnti][msg->GetMessageType() == NrControlMessage::DL_DCI ? 0 : 1];
    ++c.dcis;
    c.cces += dci->m_pdcchAl;
    c.missed += dci->m_pdcchMissed ? 1 : 0;
}

// A number for a CSV file: as precise as a double is, empty for NaN
std::string
CsvNum(double v)
{
    if (std::isnan(v))
    {
        return "";
    }
    std::ostringstream o;
    o << std::setprecision(10) << v;
    return o.str();
}

// Write a CSV file: a header line and the lines
void
WriteCsvFile(const std::string& path, const std::string& header, const std::string& lines)
{
    std::ofstream f(path);
    NS_ABORT_MSG_UNLESS(f, "--csv: cannot write " << path);
    f << header << "\n" << lines;
}

// --csv: the ROHC summary of the nr module as <prefix>_rohc.csv, a line per connection (RNTI) and
// direction of a bearer, and <prefix>_rohc_types.csv, the share of each ROHC packet type. The
// module prints its tables when the simulator is destroyed; they are read back from that text
// (--rohcReport=summary; with full only the packet types table is there). Their numbers have
// the precision of the tables: header bytes and percentages to 0.1, times to 1 ms.
void
WriteRohcCsv(const std::string& printed,
             const std::string& prefix,
             const std::string& runHeader,
             const std::string& run)
{
    auto split = [](const std::string& line) {
        std::istringstream in(line);
        std::vector<std::string> t;
        std::string w;
        while (in >> w)
        {
            t.push_back(w);
        }
        return t;
    };
    auto value = [](const std::string& v) {
        std::string s = v == "-" ? "" : v;
        if (!s.empty() && s.back() == '%')
        {
            s.pop_back();
        }
        return s;
    };
    // "1/0/2" -> "1,0,2"
    auto counts = [](const std::string& v) {
        std::string s = v;
        std::replace(s.begin(), s.end(), '/', ',');
        return s;
    };

    struct Bearer
    {
        std::string compressor = ",,,,,,,,,,";   // mode ... feedback rcvd: 11 fields
        std::string decompressor = ",,,,,,,,,,"; // sent ... feedback sent: 11 fields
        std::string mode = ",,,,,";              // U from ... started by: 6 fields
    };

    std::map<std::string, Bearer> bearers; // "address,rnti,lcid,dir"
    std::vector<std::string> order;
    std::ostringstream types;
    std::vector<std::string> typeNames;
    std::string section;
    bool inTable = false;
    std::istringstream text(printed);
    std::string line;
    while (std::getline(text, line))
    {
        if (line.rfind("| ROHC ", 0) == 0)
        {
            section = line.substr(2, line.find("  ", 2) - 2);
            inTable = false;
            continue;
        }
        if (section.empty())
        {
            continue;
        }
        const std::vector<std::string> t = split(line);
        if (line.rfind("       UE address", 0) == 0)
        {
            inTable = true;
            if (section == "ROHC PACKET TYPES")
            {
                // the columns after "UE address RNTI LCID dir mode packets"
                typeNames.assign(t.begin() + 7, t.end());
            }
            continue;
        }
        if (!inTable)
        {
            continue;
        }
        if (section == "ROHC PACKET TYPES" && t.size() == typeNames.size() + 3 && t[0] == "all")
        {
            // "all bearers <packets> <%> ...": the share over all bearers
            for (std::size_t c = 0; c < typeNames.size(); ++c)
            {
                types << run << ",all,all,,," << typeNames[c] << "," << t[3 + c] << ",\n";
            }
            continue;
        }
        if (section == "ROHC PACKET TYPES" && t.size() == typeNames.size() + 4 && t[0] == "mean")
        {
            // "mean ROHC header [B] <bytes> ...": the mean header of each type
            for (std::size_t c = 0; c < typeNames.size(); ++c)
            {
                types << run << ",all,mean,,," << typeNames[c] << ",," << t[4 + c] << "\n";
            }
            continue;
        }
        if (t.size() < 6 || t[1] == "all" ||
            t[1].find_first_not_of("0123456789") != std::string::npos)
        {
            if (t.empty() || t[0].find('.') == std::string::npos)
            {
                inTable = false; // the notes under the table
            }
            continue; // a line "all" of a UE with several RNTIs: the sum of its lines
        }
        const std::string key = t[0] + "," + t[1] + "," + t[2] + "," + t[3];
        if (!bearers.contains(key))
        {
            order.push_back(key);
        }
        Bearer& b = bearers[key];
        std::ostringstream o;
        if (section == "ROHC COMPRESSOR (SENDER)" && t.size() >= 9)
        {
            // mode packets IR/FO/SO (in -> out saved% | -) feedback
            const bool sizes = t[7] != "-";
            o << value(t[4]) << "," << t[5] << "," << counts(t[6]) << ",";
            if (sizes && t.size() >= 12)
            {
                o << t[7] << "," << t[9] << "," << value(t[10]) << "," << counts(t[11]);
            }
            else
            {
                o << ",,," << counts(t[8]);
            }
            b.compressor = o.str();
        }
        else if (section == "ROHC DECOMPRESSOR (RECEIVER)" && t.size() >= 13)
        {
            // sent arrived restored failed restored% lost for[s] net-saved% feedback
            o << t[4] << "," << t[5] << "," << t[6] << "," << t[7] << "," << value(t[8]) << ","
              << t[9] << "," << t[10] << "," << value(t[11]) << "," << counts(t[12]);
            b.decompressor = o.str();
        }
        else if (section == "ROHC MODE OF THE COMPRESSOR" && t.size() >= 9)
        {
            // U from, packets, O from, packets, last packet, and why O started (words)
            std::string why;
            for (std::size_t c = 9; c < t.size(); ++c)
            {
                why += (why.empty() ? "" : " ") + t[c];
            }
            std::replace(why.begin(), why.end(), ',', ';');
            o << value(t[4]) << "," << value(t[5]) << "," << value(t[6]) << "," << value(t[7])
              << "," << value(t[8]) << "," << why;
            b.mode = o.str();
        }
        else if (section == "ROHC PACKET TYPES" && t.size() == typeNames.size() + 6)
        {
            for (std::size_t c = 0; c < typeNames.size(); ++c)
            {
                types << run << "," << key << "," << typeNames[c] << "," << t[6 + c] << ",\n";
            }
        }
    }
    std::ostringstream lines;
    for (const auto& key : order)
    {
        const Bearer& b = bearers[key];
        lines << run << "," << key << "," << b.compressor << "," << b.decompressor << "," << b.mode
              << "\n";
    }
    WriteCsvFile(prefix + "_rohc.csv",
                 runHeader + ",address,rnti,lcid,dir,mode,packets,irPackets,foPackets,soPackets,"
                             "headerInB,headerOutB,savedPct,fbRcvdAck,fbRcvdNack,fbRcvdStaticNack,"
                             "sent,arrived,restored,failed,restoredPct,contextLosses,contextLostS,"
                             "netSavedPct,fbSentAck,fbSentNack,fbSentStaticNack,uFromS,uPackets,"
                             "oFromS,oPackets,lastPacketS,oModeStartedBy",
                 lines.str());
    WriteCsvFile(prefix + "_rohc_types.csv",
                 runHeader + ",address,rnti,lcid,dir,type,packetsPct,meanHeaderB",
                 types.str());
}

// The value an attribute has by default right now: after Config::SetDefault() and after the
// command line (--ns3::NrUePhy::Qout=-6 and the like)
template <typename V>
auto
AttributeDefault(const std::string& typeId, const std::string& name)
{
    TypeId::AttributeInformation info;
    const bool found = TypeId::LookupByName(typeId).LookupAttributeByName(name, &info);
    NS_ABORT_MSG_UNLESS(found, "No attribute " << name << " in " << typeId);
    return DynamicCast<const V>(info.initialValue)->Get();
}

// --- How the output looks: titles in a frame as wide as the widest table, short lines below ---
const std::size_t g_frameWidth = 130;

// A title in a frame; `note` goes to the right end of the same line
void
Frame(const std::string& title, const std::string& note = "")
{
    const std::string bar = "+" + std::string(g_frameWidth - 2, '-') + "+";
    const std::size_t inner = g_frameWidth - 4;
    std::string line = title;
    if (!note.empty())
    {
        const std::size_t taken = title.size() + note.size();
        line += std::string(taken + 2 <= inner ? inner - taken : 2, ' ') + note;
    }
    std::cout << "\n"
              << bar << "\n| " << std::left << std::setw(inner) << line << std::right << " |\n"
              << bar << "\n";
}

// A number with a fixed number of decimals
std::string
Num(double value, int decimals)
{
    std::ostringstream o;
    o << std::fixed << std::setprecision(decimals) << value;
    return o.str();
}

// A number as it was given: 30, 1.4, 4.75
std::string
Plain(double value)
{
    std::ostringstream o;
    o << value;
    return o.str();
}

// One line of the settings the output starts with: what, and how it is set. No `what`: more of
// the line above.
void
Setting(const std::string& what, const std::string& value)
{
    std::cout << "  " << std::left << std::setw(16) << what << std::right << value << "\n";
}

// Elevation of the higher of two nodes seen from the lower one, in degrees (negative below the
// horizon). The formula of ns-3.48's GeocentricConstantPositionMobilityModel::DoGetElevationAngle(),
// which however takes the node with the smaller ECEF z as the one on the ground. That is right
// while the satellite is north of the UE and wrong once it has passed to the south: 30 deg is
// then reported as 37.7 deg. The 3GPP channel models read the elevation through that function,
// so every node of this example overrides it with this one.
double
ElevationDeg(const Vector& p, const Vector& q)
{
    const bool pIsLower = p.GetLength() < q.GetLength();
    const Vector& a = pIsLower ? p : q; // the node on the ground
    const Vector bMinusA = (pIsLower ? q : p) - a;
    const double x = (a * bMinusA) / (a.GetLength() * bMinusA.GetLength());
    return (180.0 * M_1_PI) * std::asin(std::max(-1.0, std::min(1.0, x)));
}

// --farEnd=ue on n3 (FDD): the uplink's path loss is the downlink's, mirrored. The large-scale
// parameters of TR 38.901 belong to the link, not to a direction: the line of sight state and
// the shadowing are the same both ways, and the path loss differs by its frequency term only
// (20 log10 f in Table 7.4.1-1, the same to within 0.03 dB for n3). ns-3 gives each carrier a
// model of its own with draws of its own, so the two directions could disagree (a UE in line of
// sight on the downlink and not on the uplink); this model asks the downlink's instead. The fast
// fading stays each carrier's own, as it is for carriers 95 MHz apart.
class MirroredLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabMirroredLossModel")
                                .SetParent<PropagationLossModel>()
                                .SetGroupName("Propagation")
                                .AddConstructor<MirroredLossModel>();
        return tid;
    }

    // the downlink's model, and how much less the loss is on the uplink carrier (dB)
    void Mirror(Ptr<PropagationLossModel> downlink, double lessDb)
    {
        m_downlink = downlink;
        m_lessDb = lessDb;
    }

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override
    {
        return txPowerDbm + m_downlink->CalcRxPower(0.0, b, a) + m_lessDb;
    }

    int64_t DoAssignStreams(int64_t) override
    {
        return 0;
    }

    Ptr<PropagationLossModel> m_downlink;
    double m_lessDb{0.0};
};

// A node at a fixed place: the UEs, the gateway, and the satellite under --freezeGeometry=1. The
// frozen satellite reports its orbital velocity (what leo-link does under --freezeGeometry):
// elevation, range and delay stay constant, the channel's Doppler term does not.
class FixedMobilityModel : public GeocentricEcefMobilityModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabFixedMobilityModel")
                                .SetParent<GeocentricEcefMobilityModel>()
                                .SetGroupName("Mobility")
                                .AddConstructor<FixedMobilityModel>();
        return tid;
    }

    void SetReportedVelocity(const Vector& velocityEcef)
    {
        m_velocity = velocityEcef;
    }

  private:
    Vector DoGetVelocity() const override
    {
        return m_velocity;
    }

    double DoGetElevationAngle(Ptr<const GeocentricConstantPositionMobilityModel> other) override
    {
        return std::abs(ElevationDeg(GetGeocentricPosition(), other->GetGeocentricPosition()));
    }

    Vector m_velocity{0.0, 0.0, 0.0};
};

// The satellite under --freezeGeometry=0: ns-3.48's circular orbit, the model leo-link flies
class OrbitingMobilityModel : public LeoCircularOrbitMobilityModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabOrbitingMobilityModel")
                                .SetParent<LeoCircularOrbitMobilityModel>()
                                .SetGroupName("Mobility")
                                .AddConstructor<OrbitingMobilityModel>();
        return tid;
    }

  private:
    double DoGetElevationAngle(Ptr<const GeocentricConstantPositionMobilityModel> other) override
    {
        return std::abs(ElevationDeg(GetGeocentricPosition(), other->GetGeocentricPosition()));
    }
};

// Where the satellite will be at time t, in ECEF. The orbit model only tells where it is now,
// and K_offset has to be known before the run. Same computation as the model's CalcPosition()
// and as leo-link's SatellitePredictor: a circle in the plane of the position and velocity at
// t = 0, turned back by the Earth's rotation. main() checks it against the model after the run.
struct OrbitPredictor
{
    Vector pos0;
    Vector vel0;             // (0, 0, 0): the satellite stands still
    double omegaRadPerS{0.0}; // angular velocity along the orbit

    Vector At(Time t) const
    {
        if (vel0.GetLength() <= 0.0)
        {
            return pos0;
        }
        Vector normal = CrossProduct(pos0, vel0);
        normal = normal * (1.0 / normal.GetLength());
        const double theta = omegaRadPerS * t.GetSeconds();
        const Vector orbital = pos0 * std::cos(theta) + CrossProduct(normal, pos0) * std::sin(theta);
        const double phi = 2.0 * M_PI * t.GetSeconds() / Days(1).GetSeconds();
        return Vector(orbital.x * std::cos(phi) + orbital.y * std::sin(phi),
                      -orbital.x * std::sin(phi) + orbital.y * std::cos(phi),
                      orbital.z);
    }
};

// The satellite beam is the pattern of TR 38.811 6.4.1: 4 |J1(ka sin(theta)) / (ka sin(theta))|^2,
// theta from the beam centre, a = the radius of the aperture, k = 2 pi f / c. It is 1/2 where
// ka sin(theta) = 1.616339, so a beam of 3 dB width HPBW has ka = 1.616339 / sin(HPBW / 2).
const double g_halfPowerX = 1.616339;

double
BeamKaOfWidth(double hpbwDeg)
{
    return g_halfPowerX / std::sin(hpbwDeg / 2.0 * M_PI / 180.0);
}

double
BeamWidthDegOfKa(double ka)
{
    return 2.0 * std::asin(g_halfPowerX / ka) * 180.0 / M_PI;
}

// Gain of the satellite beam towards a UE, relative to the beam centre, in dB (<= 0).
// Same function as leo-link's SpotBeamTaperDb(), with ka given instead of worked out from a
// beamwidth: ka differs between the carriers.
double
BeamTaperDb(const Vector& satEcef, const Vector& centreEcef, const Vector& ueEcef, double ka)
{
    const Vector vc = centreEcef - satEcef;
    const Vector vu = ueEcef - satEcef;
    double cosT = (vc.x * vu.x + vc.y * vu.y + vc.z * vu.z) / (vc.GetLength() * vu.GetLength());
    cosT = std::max(-1.0, std::min(1.0, cosT));
    const double sinTheta = std::sin(std::acos(cosT));
    if (sinTheta < 1e-12)
    {
        return 0.0;
    }
    const double x = ka * sinTheta;
    const double f = j1(x) / x;
    return std::max(10.0 * std::log10(std::max(4.0 * f * f, 1e-12)), -30.0);
}

// Blocking and shadowing: what stands around a UE does not change while it makes its call.
//
// (ns-3's own NTN models draw the line-of-sight state of a link again every second, each time
// with the probability of the table, whatever the state was before, and they draw the shadow
// fading again with every new state. A UE that stands still would go in and out of line of
// sight during one call. They are not used.)
//
// Here every UE has two numbers of its own, drawn once and kept for the whole run:
//   its obstruction level, uniform between 0 and 1. The UE is in line of sight whenever the LOS
//     probability at its elevation (TR 38.811 Table 6.6.1-1) is above its level. A UE that
//     stands under a satellite that stands still keeps its state. When the satellite flies, a
//     blocked UE clears once the satellite has climbed far enough, stays clear, and is blocked
//     again when the satellite has sunk below that elevation. --scenario=NTN-LOS: the level is
//     0, so the UE is never blocked.
//   its shadowing level, standard normal. --shadowing=table: its shadow fading in dB is that
//     level times the sigma of its state and elevation (TR 38.811 Tables 6.6.2-1 to 6.6.2-3),
//     so it changes only when the row of the table or the state changes. --shadowing=fixed:
//     the level is not used; every UE has --shadowingDb all the time.
// Both hold for the downlink and the uplink carrier alike: the obstacles are the same.
// The tables have one row per 10 deg of elevation and the row nearest to the elevation counts
// (10 deg for everything below), exactly as in ns-3's models: same numbers, same rounding.
// The clutter loss of a blocked UE is ns-3's (TR 38.811 Tables 6.6.2-1 to 6.6.2-3).
struct LargeScaleTable
{
    std::array<double, 9> losProbability; // at 10, 20 ... 90 deg
    std::array<double, 9> sigmaLos;       // shadow fading, dB, S-band
    std::array<double, 9> sigmaNlos;
};
const std::map<std::string, LargeScaleTable> g_largeScaleTables = {
    {"NTN-DenseUrban",
     {{0.282, 0.331, 0.398, 0.468, 0.537, 0.612, 0.738, 0.820, 0.981},
      {3.5, 3.4, 2.9, 3.0, 3.1, 2.7, 2.5, 2.3, 1.2},
      {15.5, 13.9, 12.4, 11.7, 10.6, 10.5, 10.1, 9.2, 9.2}}},
    {"NTN-Urban",
     {{0.246, 0.386, 0.493, 0.613, 0.726, 0.805, 0.919, 0.968, 0.992},
      {4, 4, 4, 4, 4, 4, 4, 4, 4},
      {6, 6, 6, 6, 6, 6, 6, 6, 6}}},
    {"NTN-Suburban",
     {{0.782, 0.869, 0.919, 0.929, 0.935, 0.940, 0.949, 0.952, 0.998},
      {1.79, 1.14, 1.14, 0.92, 1.42, 1.56, 0.85, 0.72, 0.72},
      {8.93, 9.08, 8.78, 10.25, 10.56, 10.74, 10.17, 11.52, 11.52}}},
    {"NTN-Rural",
     {{0.782, 0.869, 0.919, 0.929, 0.935, 0.940, 0.949, 0.952, 0.998},
      {1.79, 1.14, 1.14, 0.92, 1.42, 1.56, 0.85, 0.72, 0.72},
      {8.93, 9.08, 8.78, 10.25, 10.56, 10.74, 10.17, 11.52, 11.52}}},
};

// The line-of-sight state of every UE from its obstruction level, and its shadow fading from
// its shadowing level. One object serves both carriers and both ns-3 models that ask for the
// state (the path loss and the fast fading).
class FixedLevelChannelConditionModel : public ChannelConditionModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabFixedLevelChannelConditionModel")
                                .SetParent<ChannelConditionModel>()
                                .SetGroupName("Propagation")
                                .AddConstructor<FixedLevelChannelConditionModel>();
        return tid;
    }

    void SetTable(const LargeScaleTable& table)
    {
        m_table = table;
    }

    // --shadowing=fixed: every UE has this shadow fading (dB, as a loss) instead of its level's
    void SetFixedShadowing(double lossDb)
    {
        m_fixedShadowingDb = lossDb;
    }

    void AddUe(Ptr<const MobilityModel> ue, double obstructionLevel, double shadowingLevel)
    {
        m_levels[PeekPointer(ue)] = {obstructionLevel, shadowingLevel};
    }

    // The UE of a link, or null if neither end is one
    const MobilityModel* UeOf(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const
    {
        for (const MobilityModel* end : {PeekPointer(a), PeekPointer(b)})
        {
            if (m_levels.count(end))
            {
                return end;
            }
        }
        return nullptr;
    }

    // The row of the tables for a link: 0 for 10 deg ... 8 for 90 deg, as ns-3 rounds it
    static std::size_t Row(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b)
    {
        const auto [elevation, quantized] =
            ThreeGppChannelConditionModel::GetQuantizedElevationAngle(a, b);
        return static_cast<std::size_t>(std::lround(quantized / 10.0)) - 1;
    }

    bool IsLos(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const
    {
        const MobilityModel* ue = UeOf(a, b);
        NS_ABORT_MSG_UNLESS(ue, "line of sight was asked for a link without a UE");
        return m_levels.at(ue).obstruction < m_table.losProbability[Row(a, b)];
    }

    // The shadow fading of a link as a gain in dB: negative makes the signal weaker
    double ShadowGainDb(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const
    {
        const MobilityModel* ue = UeOf(a, b);
        NS_ABORT_MSG_UNLESS(ue, "shadow fading was asked for a link without a UE");
        if (std::isfinite(m_fixedShadowingDb))
        {
            return -m_fixedShadowingDb;
        }
        const std::size_t row = Row(a, b);
        const double sigma = IsLos(a, b) ? m_table.sigmaLos[row] : m_table.sigmaNlos[row];
        return -m_levels.at(ue).shadowing * sigma;
    }

    Ptr<ChannelCondition> GetChannelCondition(Ptr<const MobilityModel> a,
                                              Ptr<const MobilityModel> b) const override
    {
        return IsLos(a, b) ? m_los : m_nlos;
    }

    int64_t AssignStreams(int64_t /* stream */) override
    {
        return 0; // nothing is drawn here: the levels come from outside
    }

  private:
    struct Levels
    {
        double obstruction;
        double shadowing;
    };
    std::map<const MobilityModel*, Levels> m_levels;
    LargeScaleTable m_table{};
    double m_fixedShadowingDb{NAN}; // NaN: --shadowing=table
    // outdoor UEs, as ns-3's models make them
    Ptr<ChannelCondition> m_los{
        CreateObject<ChannelCondition>(ChannelCondition::LosConditionValue::LOS)};
    Ptr<ChannelCondition> m_nlos{
        CreateObject<ChannelCondition>(ChannelCondition::LosConditionValue::NLOS)};
};

// The shadow fading of the model above as a propagation loss, to put behind ns-3's path loss
// (whose own shadowing is then turned off)
class FixedLevelShadowingLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabFixedLevelShadowingLossModel")
                                .SetParent<PropagationLossModel>()
                                .SetGroupName("Propagation")
                                .AddConstructor<FixedLevelShadowingLossModel>();
        return tid;
    }

    void SetLevels(Ptr<const FixedLevelChannelConditionModel> levels)
    {
        m_model = levels;
    }

  private:
    double DoCalcRxPower(double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override
    {
        return m_model->UeOf(a, b) ? txPowerDbm + m_model->ShadowGainDb(a, b) : txPowerDbm;
    }

    int64_t DoAssignStreams(int64_t /* stream */) override
    {
        return 0;
    }

    Ptr<const FixedLevelChannelConditionModel> m_model;
};

// The atmosphere of the service link (satellite - UE, both carriers) and --linkMarginDb, put
// behind ns-3's path loss (and the shadowing above). ns-3's NTN path loss has TR 38.811's free
// space path loss and clutter loss, and for the atmosphere:
//   gases                      A_zenith(f) / sin(elevation), TR 38.811 (6.6-8), below 10 deg
//                              only, with A_zenith from ITU-R P.676 Fig. 6: 0.035 dB at 2 GHz.
//   ionospheric scintillation  6.22 / f[GHz]^1.5 dB at every place and time: 2.2 dB at 2 GHz,
//                              TR 38.821's link budget value (Table 6.1.3.2-1 note 1: P3 curve,
//                              1 % of the time, of the equatorial region).
// This model takes them back out and puts in what --atmosphere asks for:
//   table      TR 38.821 Table 6.1.3.3-1, S-band LEO: 0.1 dB of gases and 2.2 dB of
//              scintillation, for every UE at every elevation, on both carriers.
//   location   gases A_zenith(f) / sin(elevation) at every elevation (TR 38.821 Table 6.1.1.1-5
//              note 2: "The atmospheric absorptions loss shall be considered"); 0.07 dB at 30
//              deg. Scintillation by the magnetic latitude --latitudeDeg (TR 38.811 6.6.6.1.4):
//              6.22 / f^1.5 dB within 20 deg of the magnetic equator, none from 20 to 60 deg.
//   --linkMarginDb             more loss on every link: TR 38.821's additional losses (0).
class ServiceLinkLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabServiceLinkLossModel")
                                .SetParent<PropagationLossModel>()
                                .SetGroupName("Propagation")
                                .AddConstructor<ServiceLinkLossModel>();
        return tid;
    }

    // TR 38.821 Table 6.1.3.3-1, S-band LEO rows: atmospheric loss and scintillation loss
    static constexpr double TABLE_GASES_DB = 0.1;
    static constexpr double TABLE_SCINTILLATION_DB = 2.2;

    void Set(double frequencyHz, bool table, bool ionosphericScintillation, double marginDb)
    {
        m_frequencyHz = frequencyHz;
        m_table = table;
        m_ionosphericScintillation = ionosphericScintillation;
        m_marginDb = marginDb;
    }

    // ns-3's zenith attenuation by atmospheric gases (ThreeGppPropagationLossModel, from ITU-R
    // P.676 Figure 6), at the carrier rounded to whole GHz as ns-3 rounds it: 0.6 to 6 GHz here
    static double ZenithAttenuationDb(double frequencyHz)
    {
        static const std::array<double, 7> zenith =
            {0, 0.0300, 0.0350, 0.0380, 0.0390, 0.0410, 0.0420};
        const auto ghz = static_cast<std::size_t>(std::lround(frequencyHz / 1e9));
        NS_ABORT_MSG_UNLESS(ghz < zenith.size(), "the carrier must be below 6.5 GHz");
        return zenith[ghz];
    }

    // ns-3's ionospheric scintillation loss below 6 GHz (TR 38.811 (6.6-13), Pfluc 1.1 dB at 4 GHz)
    static double IonosphericScintillationDb(double frequencyHz)
    {
        return 6.22 / std::pow(frequencyHz / 1e9, 1.5);
    }

    // Gases at an elevation (deg), TR 38.811 (6.6-8)
    static double GasesDb(double frequencyHz, double elevationDeg)
    {
        return ZenithAttenuationDb(frequencyHz) / std::sin(elevationDeg * M_PI / 180.0);
    }

    // The gases the run takes at an elevation (deg)
    double AtmosphericDb(double elevationDeg) const
    {
        return m_table ? TABLE_GASES_DB : GasesDb(m_frequencyHz, elevationDeg);
    }

    double Frequency() const
    {
        return m_frequencyHz;
    }

    // The scintillation the run takes
    double ScintillationDb() const
    {
        if (m_table)
        {
            return TABLE_SCINTILLATION_DB;
        }
        return m_ionosphericScintillation ? IonosphericScintillationDb(m_frequencyHz) : 0.0;
    }

    // What this model adds to ns-3's path loss at an elevation (deg): what the run takes less
    // what ns-3 took
    double ExtraLossDb(double elevationDeg) const
    {
        const double ns3Gases = elevationDeg < 10.0 ? GasesDb(m_frequencyHz, elevationDeg) : 0.0;
        return m_marginDb + AtmosphericDb(elevationDeg) + ScintillationDb() - ns3Gases -
               IonosphericScintillationDb(m_frequencyHz);
    }

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override
    {
        const auto [elevation, quantized] =
            ThreeGppChannelConditionModel::GetQuantizedElevationAngle(a, b);
        return txPowerDbm - ExtraLossDb(elevation);
    }

    int64_t DoAssignStreams(int64_t /* stream */) override
    {
        return 0;
    }

    double m_frequencyHz{2e9};
    bool m_table{true};
    bool m_ionosphericScintillation{false};
    double m_marginDb{0.0};
};

// The path loss of the feeder link, gateway -> satellite, at its own Ka-band carrier: free space
// only, TR 38.811 (6.6-2). The link is taken as reliable: the gateway is sited in line of sight
// and the link is dimensioned for its availability (TR 38.821 8.4.1.1: "up to 99.999%"), so no
// clutter loss, shadowing, scintillation, gases or rain. NrNtnFeederNoiseLossModel asks it for
// the feeder loss of the downlink; on the uplink the feeder link adds delay only.
class FeederLinkLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabFeederLinkLossModel")
                                .SetParent<PropagationLossModel>()
                                .SetGroupName("Propagation")
                                .AddConstructor<FeederLinkLossModel>();
        return tid;
    }

    void SetFrequency(double frequencyHz)
    {
        m_frequencyHz = frequencyHz;
    }

    // TR 38.811 (6.6-2): carrier in Hz, distance in m
    static double FreeSpaceDb(double frequencyHz, double distanceM)
    {
        return 32.45 + 20.0 * std::log10(frequencyHz / 1e9) + 20.0 * std::log10(distanceM);
    }

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override
    {
        // the distance the delay model takes too (positions on the Earth-centred axes)
        return txPowerDbm - FreeSpaceDb(m_frequencyHz, a->GetDistanceFrom(b));
    }

    int64_t DoAssignStreams(int64_t /* stream */) override
    {
        return 0;
    }

    double m_frequencyHz{28.75e9};
};

// --fastFading=fixed: the channel between satellite and UE is the direct path alone, the
// line-of-sight ray of TR 38.901 (7.5-29) with an infinite K-factor: no scattered clusters, so
// nothing that changes from one resource block or one slot to the next. It keeps what ns-3's
// 3GPP channel puts into that ray, so the gains stay where they are: both antennas' element
// patterns and polarisation (the UE's two +/-45 deg ports, combined, recover the satellite's
// signal, TR 38.821 6.1.1.1), the beams, the beam taper, and the carrier phase of the distance.
// --fastFadingDb is put on top as a loss of its own on every resource block. It takes the place
// of ns-3's ThreeGppChannelModel in the carrier's ThreeGppSpectrumPropagationLossModel, which
// still turns the channel into the received signal.
class LosOnlyChannelModel : public MatrixBasedChannelModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::NtnLabLosOnlyChannelModel")
                .SetParent<Object>()
                .SetGroupName("Spectrum")
                .AddConstructor<LosOnlyChannelModel>()
                .AddAttribute("Frequency",
                              "The carrier (Hz)",
                              DoubleValue(2e9),
                              MakeDoubleAccessor(&LosOnlyChannelModel::m_frequencyHz),
                              MakeDoubleChecker<double>(1e8, 100e9));
        return tid;
    }

    void SetLossDb(double lossDb)
    {
        m_amplitude = std::pow(10.0, -lossDb / 20.0);
    }

    Ptr<const ChannelMatrix> GetChannel(Ptr<const MobilityModel> aMob,
                                        Ptr<const MobilityModel> bMob,
                                        Ptr<const PhasedArrayModel> aAntenna,
                                        Ptr<const PhasedArrayModel> bAntenna) override
    {
        const uint32_t aId = aMob->GetObject<Node>()->GetId();
        const uint32_t bId = bMob->GetObject<Node>()->GetId();
        Link& link = m_links[GetKey(aId, bId)];
        if (!link.matrix)
        {
            // s and u of the link are a and b of its first call, for good: a later call the
            // other way round finds the matrix reversed (ChannelMatrix::IsReverse)
            link.sId = aId;
        }
        // one matrix per link and moment: the geometry and the gains move between moments
        if (link.matrix && link.matrix->m_generatedTime == Simulator::Now())
        {
            return link.matrix;
        }
        const bool aIsS = (aId == link.sId);
        const Ptr<const MobilityModel> sMob = aIsS ? aMob : bMob;
        const Ptr<const MobilityModel> uMob = aIsS ? bMob : aMob;
        const Ptr<const PhasedArrayModel> sAntenna = aIsS ? aAntenna : bAntenna;
        const Ptr<const PhasedArrayModel> uAntenna = aIsS ? bAntenna : aAntenna;
        const uint32_t sId = sMob->GetObject<Node>()->GetId();
        const uint32_t uId = uMob->GetObject<Node>()->GetId();

        // Both ends on the Earth-centred axes, as ns-3's 3GPP channel takes them
        const Vector sPos = EcefOf(sMob);
        const Vector uPos = EcefOf(uMob);
        const Angles sAngle(uPos, sPos); // departure, at s
        const Angles uAngle(sPos, uPos); // arrival, at u

        // The parameters: one cluster, at the direct path, no delay, no moving scatterers
        Ptr<ChannelParams> params = Create<ChannelParams>();
        params->m_generatedTime = Simulator::Now();
        params->m_nodeIds = {sId, uId};
        params->m_delay = {0.0};
        params->m_angle = {{RadiansToDegrees(uAngle.GetAzimuth())},
                           {RadiansToDegrees(uAngle.GetInclination())},
                           {RadiansToDegrees(sAngle.GetAzimuth())},
                           {RadiansToDegrees(sAngle.GetInclination())}};
        params->m_cachedAngleSincos.resize(params->m_angle.size());
        for (std::size_t i = 0; i < params->m_angle.size(); ++i)
        {
            const double rad = DegreesToRadians(params->m_angle[i][0]);
            params->m_cachedAngleSincos[i] = {{std::sin(rad), std::cos(rad)}};
        }
        params->m_alpha = {0.0};
        params->m_D = {0.0};

        // The ray between every pair of elements, as ThreeGppChannelModel::GetNewChannel()
        // computes the line-of-sight ray
        Ptr<ChannelMatrix> matrix = Create<ChannelMatrix>();
        matrix->m_generatedTime = Simulator::Now();
        matrix->m_nodeIds = {sId, uId};
        matrix->m_antennaPair = {sAntenna->GetId(), uAntenna->GetId()};
        const std::size_t uSize = uAntenna->GetNumElems();
        const std::size_t sSize = sAntenna->GetNumElems();
        matrix->m_channel = Complex3DVector(uSize, sSize, 1);
        const double lambda = 3.0e8 / m_frequencyHz;
        const double distance = (sPos - uPos).GetLength();
        const std::complex<double> distancePhase = std::polar(1.0, -2.0 * M_PI * distance / lambda);
        auto towards = [](const Angles& angle, const Vector& loc) {
            return 2.0 * M_PI *
                   (std::sin(angle.GetInclination()) * std::cos(angle.GetAzimuth()) * loc.x +
                    std::sin(angle.GetInclination()) * std::sin(angle.GetAzimuth()) * loc.y +
                    std::cos(angle.GetInclination()) * loc.z);
        };
        for (std::size_t u = 0; u < uSize; ++u)
        {
            const double rxPhase = towards(uAngle, uAntenna->GetElementLocation(u));
            const auto [rxPhi, rxTheta] =
                uAntenna->GetElementFieldPattern(uAngle, uAntenna->GetElemPol(u));
            for (std::size_t s = 0; s < sSize; ++s)
            {
                const double txPhase = towards(sAngle, sAntenna->GetElementLocation(s));
                const auto [txPhi, txTheta] =
                    sAntenna->GetElementFieldPattern(sAngle, sAntenna->GetElemPol(s));
                matrix->m_channel(u, s, 0) = m_amplitude * (rxTheta * txTheta - rxPhi * txPhi) *
                                             distancePhase * std::polar(1.0, rxPhase + txPhase);
            }
        }
        link.matrix = matrix;
        link.params = params;
        return matrix;
    }

    Ptr<const ChannelParams> GetParams(Ptr<const MobilityModel> aMob,
                                       Ptr<const MobilityModel> bMob) const override
    {
        const auto it = m_links.find(
            GetKey(aMob->GetObject<Node>()->GetId(), bMob->GetObject<Node>()->GetId()));
        NS_ABORT_MSG_IF(it == m_links.end() || !it->second.params,
                        "the parameters of a link were asked for before its channel");
        return it->second.params;
    }

  private:
    static Vector EcefOf(Ptr<const MobilityModel> mob)
    {
        auto geo = DynamicCast<const GeocentricConstantPositionMobilityModel>(mob);
        return geo ? geo->GetGeocentricPosition() : mob->GetPosition();
    }

    struct Link
    {
        uint32_t sId{0};
        Ptr<ChannelMatrix> matrix;
        Ptr<ChannelParams> params;
    };

    double m_frequencyHz{2e9};
    double m_amplitude{1.0};
    std::map<uint64_t, Link> m_links;
};

// The resource blocks of a carrier of 15 kHz subcarriers: TS 38.101-1 Table 5.3.2-1 (the NTN
// bands of TS 38.101-5 Table 5.3.2-1 have the same for their 5 to 20 MHz). 0: not a channel
// bandwidth of the table. (The nr module's own count, 96 % of the bandwidth, has 26 for 5 MHz.)
uint32_t
CarrierRbs(double bandwidthMHz)
{
    static const std::map<int, uint32_t> rbs = {{5, 25},
                                                {10, 52},
                                                {15, 79},
                                                {20, 106},
                                                {25, 133},
                                                {30, 160},
                                                {35, 188},
                                                {40, 216},
                                                {45, 242},
                                                {50, 270}};
    const long mhz = std::lround(bandwidthMHz);
    const auto it = rbs.find(static_cast<int>(mhz));
    return it != rbs.end() && std::abs(bandwidthMHz - static_cast<double>(mhz)) < 1e-9 ? it->second
                                                                                       : 0;
}

// What the functions below work on while the simulation runs
struct Scene
{
    Ptr<GeocentricConstantPositionMobilityModel> satellite;
    Vector centre; // the beam centre, ECEF: the place on the ground where UE 0 is
    std::vector<Ptr<FixedMobilityModel>> ues;
    Ptr<NrNtnPropagationDelayModel> delay;
    Ptr<ChannelConditionModel> condition;           // carrier of the voice direction
    Ptr<FixedLevelChannelConditionModel> levels;    // blocking and shadowing per UE
    std::vector<Ptr<UniformPlanarArray>> satArrays; // one per BWP
    std::vector<std::vector<Ptr<IsotropicAntennaModel>>> ueElements; // per UE, one per BWP
    std::vector<std::vector<Ptr<PhasedArrayModel>>> ueArrays;        // per UE, one per BWP
    double ueAntennaGainDbi{0.0};
    std::vector<double> beamKa; // ka of the beam pattern per BWP: 0 = downlink, 1 = uplink carrier
    bool flying{false};      // --freezeGeometry=0
    OrbitPredictor orbit;    // where the satellite will be
    double groundDelayMs{0}; // far end to gateway: remote host or far gNB, UPF, gateway
    // Power control (--dlPowerRefDeg, --ulPowerRefDeg; 0 = off). The path loss of the reference,
    // a UE on the beam centre with the satellite at that elevation, and what the power is
    // without control
    double dlPowerRefDeg{0.0};
    double dlRefLossDb{0.0}; // free space and gases at dlPowerRefDeg, downlink carrier
    double dlNominalDbm{0.0};
    Ptr<NrGnbPhy> dlPhy;
    Ptr<ServiceLinkLossModel> dlExtra; // the gases of the downlink carrier
    double dlFrequencyHz{2e9};
    double ulPowerRefDeg{0.0};
    double ulRefLossDb{0.0}; // the path loss a UE on the beam centre at ulPowerRefDeg measures
    double ueMaxDbm{23.0};
    Ptr<PropagationLossModel> dlServiceLoss;       // the path loss chain of the downlink carrier
    std::vector<std::vector<Ptr<NrUePhy>>> uePhys; // per UE, one per BWP
    std::vector<double> ueLossDb;                  // the path loss each UE measured last
};
Scene g_scene;

// First, last, smallest and largest value of something sampled during a call
struct Span
{
    double first{NAN};
    double last{NAN};
    double min{NAN};
    double max{NAN};
    double sum{0.0};
    uint32_t n{0};

    void Add(double v)
    {
        if (n++ == 0)
        {
            first = min = max = v;
        }
        last = v;
        min = std::min(min, v);
        max = std::max(max, v);
        sum += v;
    }
};

// What a UE saw of the satellite during its call
struct CallView
{
    Time callStart;
    Time callEnd;
    Span elevationDeg;
    double sideAtStart{0.0}; // -1: the satellite is still coming closer, +1: it is moving away,
    double sideAtEnd{0.0};   //  0: it stands still
    Span rangeKm;            // satellite to UE
    Span delayMs;            // one way, far end -> UE: ground + feeder (transparent) + service
    std::array<Span, 2> taperDb; // beam gain towards the UE, relative to the beam centre: DL, UL
    Span shadowDb;           // shadow fading of the UE as a gain (negative: weaker)
    uint32_t losYes{0};      // line-of-sight samples of the call: how many said yes ...
    uint32_t losAll{0};      // ... out of how many
    Span ulPowerDbm;         // --ulPowerRefDeg: the transmit power of the UE
    Span ulLossDb;           // ... and the path loss it measured
    Span dlCutDb;            // --dlPowerRefDeg: what the satellite took off its power
};
std::vector<CallView> g_view;

// Where the satellite is for UE i, now. The elevation is the one the channel model uses. The
// side says whether the satellite is still coming closer to this UE (before it is overhead) or
// moving away again (after).
void
RecordView(uint32_t i)
{
    const Vector sat = g_scene.satellite->GetPosition();
    const Vector ue = g_scene.ues[i]->GetPosition();
    CallView& view = g_view[i];
    if (g_scene.flying)
    {
        const Time now = Simulator::Now();
        const bool closer = (g_scene.orbit.At(now + MilliSeconds(1)) - ue).GetLength() <
                            (g_scene.orbit.At(now) - ue).GetLength();
        view.sideAtEnd = closer ? -1.0 : 1.0;
        if (view.elevationDeg.n == 0)
        {
            view.sideAtStart = view.sideAtEnd;
        }
    }
    view.elevationDeg.Add(g_scene.satellite->GetElevationAngle(g_scene.ues[i]));
    view.rangeKm.Add((sat - ue).GetLength() / 1000.0);
    view.delayMs.Add(
        g_scene.groundDelayMs +
        g_scene.delay->GetOneWayDelay(g_scene.satellite, g_scene.ues[i]).GetSeconds() * 1000.0);
    for (std::size_t c = 0; c < view.taperDb.size(); ++c)
    {
        view.taperDb[c].Add(BeamTaperDb(sat, g_scene.centre, ue, g_scene.beamKa[c]));
    }
    if (g_scene.levels)
    {
        view.shadowDb.Add(g_scene.levels->ShadowGainDb(g_scene.satellite, g_scene.ues[i]));
    }
    if (g_scene.ulPowerRefDeg > 0.0)
    {
        view.ulPowerDbm.Add(g_scene.uePhys[i][1]->GetTxPower());
        view.ulLossDb.Add(g_scene.ueLossDb[i]);
    }
    if (g_scene.dlPowerRefDeg > 0.0)
    {
        view.dlCutDb.Add(g_scene.dlNominalDbm - g_scene.dlPhy->GetTxPower());
    }
}

// Every 100 ms of UE i's call, its first and last instant included
void
SampleCall(uint32_t i)
{
    RecordView(i);
    const CallView& view = g_view[i];
    const Time left = view.callEnd - Simulator::Now();
    if (left.IsStrictlyPositive())
    {
        Simulator::Schedule(std::min(left, Time(MilliSeconds(100))), &SampleCall, i);
    }
}

// Whether each UE has line of sight to the satellite, asked every 100 ms on the carrier that
// carries the voice and counted while the UE's call is on. The state follows from the UE's
// obstruction level and the elevation (FixedLevelChannelConditionModel): asking draws nothing.
void
SampleLos()
{
    for (std::size_t i = 0; i < g_scene.ues.size(); ++i)
    {
        const bool los =
            g_scene.condition->GetChannelCondition(g_scene.satellite, g_scene.ues[i])->IsLos();
        CallView& view = g_view[i];
        if (Simulator::Now() >= view.callStart && Simulator::Now() <= view.callEnd)
        {
            ++view.losAll;
            view.losYes += los;
        }
    }
    Simulator::Schedule(MilliSeconds(100), &SampleLos);
}

// The satellite turns its arrays (one per BWP) towards the beam centre. The channel model works
// in ECEF, so the two angles are ECEF angles.
void
PointBeam()
{
    const Angles toCentre(g_scene.centre, g_scene.satellite->GetPosition());
    for (auto& array : g_scene.satArrays)
    {
        array->SetAlpha(toCentre.GetAzimuth());
        array->SetBeta(toCentre.GetInclination());
        array->SetBeamformingVector(array->GetBeamformingVector());
    }
}

// --dlPowerRefDeg: the satellite lowers the power of the beam by what the free-space loss and the
// gases to the beam centre are below their values with the satellite at dlPowerRefDeg above it
// (a higher satellite is nearer). It never sends more than without the control.
void
DlPowerControl()
{
    const Vector sat = g_scene.satellite->GetPosition();
    const double lossDb = FeederLinkLossModel::FreeSpaceDb(g_scene.dlFrequencyHz,
                                                           (sat - g_scene.centre).GetLength()) +
                          g_scene.dlExtra->AtmosphericDb(ElevationDeg(sat, g_scene.centre));
    g_scene.dlPhy->SetTxPower(g_scene.dlNominalDbm - std::max(0.0, g_scene.dlRefLossDb - lossDb));
}

// --ulPowerRefDeg: open-loop power control of TS 38.213 7.1.1 with alpha = 1. A UE measures its
// path loss on the downlink: the loss of the downlink carrier's chain and its beam gain, not the
// fast fading, which the filtered RSRP averages out. It sends P0 + 10 log10(M) + PL, with P0 such
// that the reference UE (on the beam centre, the satellite at ulPowerRefDeg) sends all its
// power: so a UE sends what its path loss is below the reference's less, never more than all.
void
UlPowerControl()
{
    const Vector sat = g_scene.satellite->GetPosition();
    for (std::size_t i = 0; i < g_scene.ues.size(); ++i)
    {
        const double lossDb =
            -g_scene.dlServiceLoss->CalcRxPower(0.0, g_scene.satellite, g_scene.ues[i]) -
            BeamTaperDb(sat, g_scene.centre, g_scene.ues[i]->GetPosition(), g_scene.beamKa[0]);
        g_scene.ueLossDb[i] = lossDb;
        const double powerDbm =
            std::min(g_scene.ueMaxDbm, g_scene.ueMaxDbm - (g_scene.ulRefLossDb - lossDb));
        for (const auto& phy : g_scene.uePhys[i])
        {
            phy->SetTxPower(powerDbm);
        }
    }
}

// --freezeGeometry=0 only, every 100 ms: the beam stays on its centre and every UE's antenna gain
// gets the beam taper of the new geometry, as leo-link's UpdateAntennaOrientation() and
// UpdateSpotBeamTaper() do, here for each carrier with its own beam pattern. Setting the
// beamforming vector again drops the cached channel. The power control follows the geometry.
void
FollowBeam()
{
    PointBeam();
    const Vector sat = g_scene.satellite->GetPosition();
    const Vector centre = g_scene.centre;
    for (std::size_t i = 0; i < g_scene.ues.size(); ++i)
    {
        for (std::size_t bwp = 0; bwp < g_scene.ueArrays[i].size(); ++bwp)
        {
            const double taperDb =
                BeamTaperDb(sat, centre, g_scene.ues[i]->GetPosition(), g_scene.beamKa[bwp]);
            g_scene.ueElements[i][bwp]->SetAttribute(
                "Gain",
                DoubleValue(g_scene.ueAntennaGainDbi + taperDb));
        }
        for (auto& array : g_scene.ueArrays[i])
        {
            array->SetBeamformingVector(array->GetBeamformingVector());
        }
    }
    if (g_scene.dlPowerRefDeg > 0.0)
    {
        DlPowerControl();
    }
    if (g_scene.ulPowerRefDeg > 0.0)
    {
        UlPowerControl();
    }
    Simulator::Schedule(MilliSeconds(100), &FollowBeam);
}

// --- The voice flow against its 5QI (TS 23.501 5.7.3 and Table 5.7.4-1) -------------------------
// The packet delay budget is the bound for the time a packet may take "between the UE and the
// N6 termination point at the UPF". The UPF of this model is the PGW node; N6 is its link to the
// remote host, and a packet between two UEs turns around in its IP layer. Every voice packet is
// stamped where it enters such a stretch and read where it leaves it:
//   UE -> UPF  stamped when the IP layer of the sending UE sends it, read when the IP layer of
//              the PGW receives it out of the tunnel
//   UPF -> UE  stamped when the IP layer of the PGW receives it (from the remote host, or out of
//              the tunnel of the other UE), read when the IP layer of the UE delivers it to UDP
//   end to end stamped when the IP layer of the sender sends it, read when the IP layer of the
//              receiver delivers it: the delay of the call, without codec and jitter buffer
// A stamp is a byte tag: it travels with the packet through the tunnels, PDCP (ROHC keeps the
// tags) and RLC, and changes no size. Each stretch has its own stamp on the packet.
enum VoiceLeg : uint8_t
{
    LEG_UE_UPF = 0,
    LEG_UPF_UE = 1,
    LEG_E2E = 2,
};

class VoiceStampTag : public Tag
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("VoiceStampTag").SetParent<Tag>().AddConstructor<VoiceStampTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    uint32_t GetSerializedSize() const override
    {
        return 9;
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU64(static_cast<uint64_t>(m_ns));
        i.WriteU8(m_leg);
    }

    void Deserialize(TagBuffer i) override
    {
        m_ns = static_cast<int64_t>(i.ReadU64());
        m_leg = i.ReadU8();
    }

    void Print(std::ostream& os) const override
    {
        os << "entered stretch " << +m_leg << " at " << m_ns << " ns";
    }

    int64_t m_ns{0};  ///< when the packet entered the stretch
    uint8_t m_leg{0}; ///< which stretch (VoiceLeg)
};

// The stamp of a stretch on a packet: when the packet entered it, -1 if it has none
int64_t
StampOf(Ptr<const Packet> packet, uint8_t leg)
{
    ByteTagIterator it = packet->GetByteTagIterator();
    while (it.HasNext())
    {
        const ByteTagIterator::Item item = it.Next();
        if (item.GetTypeId() == VoiceStampTag::GetTypeId())
        {
            VoiceStampTag stamp;
            item.GetTag(stamp);
            if (stamp.m_leg == leg)
            {
                return stamp.m_ns;
            }
        }
    }
    return -1;
}

// What is measured of one stretch of one direction of a call
struct VoiceQos
{
    std::vector<int64_t> delayNs; // of every packet that arrived
    uint64_t unstamped{0};        // arrived without a stamp: must stay 0
    // The bit rate of the flow over the averaging window of the 5QI, in IP packets, as the UPF
    // counts it: the packets of the last window (time, bits) and the fullest window so far.
    // Times in whole microseconds: the few nanoseconds by which a longer packet arrives later
    // over the N6 link must not push a 101st packet into a window of 100 voice frames.
    std::deque<std::pair<int64_t, uint32_t>> window;
    uint64_t windowBits{0};
    uint64_t peakWindowBits{0};
    // The satellite UE's stretches: the same per connection (g_connections): the connection each
    // delay was measured on, and the fullest window of the packets that entered while the UE had
    // that connection
    std::vector<int32_t> delayConnection;
    int32_t windowConnection{-1};

    // The satellite UE's stretch, for the release of calls that miss their budget: the packets
    // that entered in each 100 ms, and how many of them left within the PDB, left after it, or
    // were discarded by the sender's RLC for waiting too long (the last 4 s)
    struct OnTimeBin
    {
        int64_t startNs{0};
        uint32_t sent{0};
        uint32_t onTime{0};
        uint32_t late{0};
        uint32_t dropped{0};
    };

    std::deque<OnTimeBin> bins;
    std::deque<std::pair<int64_t, uint32_t>> connectionWindow;
    uint64_t connectionWindowBits{0};
    std::vector<uint64_t> connectionPeakBits;
};

// Every call, in both directions (Dir): the satellite UE <-> UPF stretch, the far end's stretch
// (a terrestrial UE <-> UPF; with the remote host as the far end it is the N6 link and not
// measured), and end to end
struct VoiceQosScene
{
    std::vector<std::array<VoiceQos, 2>> sat;
    std::vector<std::array<VoiceQos, 2>> far;
    std::vector<std::array<VoiceQos, 2>> e2e;
    uint16_t port{0};       // UDP port of the voice flows
    uint32_t remoteHost{0}; // its IPv4 address
    // IPv4 address of a UE -> its call, and whether it is the far end of it (a terrestrial UE)
    std::map<uint32_t, std::pair<uint32_t, bool>> endpoint;
    int64_t windowUs{0}; // the averaging window, microseconds
    int64_t pdbNs{0};    // the packet delay budget of the satellite UE's stretch
};
VoiceQosScene g_voiceQos;

// A voice packet of `ipBytes` bytes (IP header included) enters a stretch. `call`: the call of a
// satellite UE's stretch, whose connections are followed; -1 for the other stretches.
void
VoiceEnters(VoiceQos& q, int64_t call, uint8_t leg, Ptr<const Packet> packet, uint32_t ipBytes)
{
    VoiceStampTag stamp;
    stamp.m_ns = Simulator::Now().GetNanoSeconds();
    stamp.m_leg = leg;
    packet->AddByteTag(stamp);
    const int64_t now = Simulator::Now().GetMicroSeconds();
    q.window.emplace_back(now, ipBytes * 8);
    q.windowBits += ipBytes * 8;
    while (q.window.front().first <= now - g_voiceQos.windowUs)
    {
        q.windowBits -= q.window.front().second;
        q.window.pop_front();
    }
    q.peakWindowBits = std::max(q.peakWindowBits, q.windowBits);
    if (call < 0)
    {
        return;
    }
    const int64_t ns = Simulator::Now().GetNanoSeconds();
    const int64_t bin = ns - ns % 100000000;
    if (q.bins.empty() || q.bins.back().startNs != bin)
    {
        q.bins.push_back(VoiceQos::OnTimeBin{bin});
        if (q.bins.size() > 40)
        {
            q.bins.pop_front();
        }
    }
    ++q.bins.back().sent;
    const int32_t connection = CurrentConnection(static_cast<uint32_t>(call));
    if (connection != q.windowConnection)
    {
        q.connectionWindow.clear();
        q.connectionWindowBits = 0;
        q.windowConnection = connection;
    }
    if (connection >= 0)
    {
        q.connectionWindow.emplace_back(now, ipBytes * 8);
        q.connectionWindowBits += ipBytes * 8;
        while (q.connectionWindow.front().first <= now - g_voiceQos.windowUs)
        {
            q.connectionWindowBits -= q.connectionWindow.front().second;
            q.connectionWindow.pop_front();
        }
        if (q.connectionPeakBits.size() <= static_cast<std::size_t>(connection))
        {
            q.connectionPeakBits.resize(connection + 1, 0);
        }
        q.connectionPeakBits[connection] =
            std::max(q.connectionPeakBits[connection], q.connectionWindowBits);
    }
}

// A voice packet leaves a stretch. `call` as above.
void
VoiceLeaves(VoiceQos& q, int64_t call, uint8_t leg, Ptr<const Packet> packet)
{
    const int64_t entered = StampOf(packet, leg);
    if (entered < 0)
    {
        ++q.unstamped;
        return;
    }
    q.delayNs.push_back(Simulator::Now().GetNanoSeconds() - entered);
    if (call >= 0)
    {
        const int64_t bin = entered - entered % 100000000;
        for (auto it = q.bins.rbegin(); it != q.bins.rend(); ++it)
        {
            if (it->startNs == bin)
            {
                ++(q.delayNs.back() <= g_voiceQos.pdbNs ? it->onTime : it->late);
                break;
            }
        }
    }
    if (call >= 0)
    {
        // It came over the last connection the UE set up: the one it has, or, for an uplink
        // packet still on its way to the UPF after a radio link failure, the one that failed
        q.delayConnection.push_back(static_cast<int32_t>(g_connections[call].size()) - 1);
    }
}

// "TxDrop" of an RLC of satellite UE `ue`'s bearers (on the gNB: downlink, on the UE: uplink):
// the RLC did not take the packet, its queue had waited too long; for the release of calls
void
VoiceDropped(uint32_t ue, Dir dir, Ptr<const Packet> packet)
{
    const int64_t entered = StampOf(packet, dir == DOWN ? LEG_UPF_UE : LEG_UE_UPF);
    if (entered < 0 || ue >= g_voiceQos.sat.size())
    {
        return; // not a voice packet
    }
    const int64_t bin = entered - entered % 100000000;
    for (auto& b : g_voiceQos.sat[ue][dir].bins)
    {
        if (b.startNs == bin)
        {
            ++b.dropped;
            break;
        }
    }
}

// The RLCs of the bearers under `path` (a config path ending in .../DataRadioBearerMap/*) that
// are not yet hooked to VoiceDropped()
std::set<const Object*> g_droppedHooked;

void
HookVoiceDrops(const std::string& path, uint32_t ue, Dir dir)
{
    const Config::MatchContainer rlcs = Config::LookupMatches(path + "/NrRlc");
    for (std::size_t k = 0; k < rlcs.GetN(); ++k)
    {
        const Ptr<Object> rlc = rlcs.Get(k);
        if (rlc && g_droppedHooked.insert(PeekPointer(rlc)).second)
        {
            rlc->TraceConnectWithoutContext("TxDrop", MakeBoundCallback(&VoiceDropped, ue, dir));
        }
    }
}

// "ConnectionReconfiguration" of the satellite gNB: the bearers of a UE are set up
void
SatGnbReconfigured(std::string ueMap,
                   std::map<uint64_t, uint32_t>* ueOfImsi,
                   uint64_t imsi,
                   uint16_t cellId,
                   uint16_t rnti)
{
    const auto it = ueOfImsi->find(imsi);
    if (it != ueOfImsi->end())
    {
        HookVoiceDrops(ueMap + "/" + std::to_string(rnti) + "/DataRadioBearerMap/*",
                       it->second,
                       DOWN);
    }
}

// "DrbCreated" of satellite UE `ue`'s RRC
void
SatUeDrbCreated(std::string drbs,
                uint32_t ue,
                uint64_t imsi,
                uint16_t cellId,
                uint16_t rnti,
                uint8_t lcid)
{
    HookVoiceDrops(drbs, ue, UP);
}

// The call of a UE address and whether it is the far end; false if it is no UE of a call
bool
EndpointOf(uint32_t address, uint32_t* call, bool* far)
{
    const auto it = g_voiceQos.endpoint.find(address);
    if (it == g_voiceQos.endpoint.end())
    {
        return false;
    }
    *call = it->second.first;
    *far = it->second.second;
    return true;
}

// "Rx" of the IPv4 of the PGW: every packet its IP layer receives, IP header included. A voice
// packet from a UE has come up through the tunnel and leaves the UE's uplink stretch here; one
// for a UE enters its downlink stretch here, whether it came from the remote host or up from the
// other UE of the call (then both happen to it). (The tunnelled packets themselves are for UDP
// port 2152.)
void
PgwIpReceives(Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
    uint8_t b[24];
    if (packet->GetSize() < 28 || packet->CopyData(b, 24) != 24 || b[0] != 0x45 || b[9] != 17 ||
        ((b[22] << 8) | b[23]) != g_voiceQos.port)
    {
        return; // not IPv4 without options, not UDP, or not for the voice port
    }
    const uint32_t source = (uint32_t(b[12]) << 24) | (b[13] << 16) | (b[14] << 8) | b[15];
    const uint32_t destination = (uint32_t(b[16]) << 24) | (b[17] << 16) | (b[18] << 8) | b[19];
    uint32_t call = 0;
    bool far = false;
    if (EndpointOf(source, &call, &far))
    {
        // the satellite UE talks (UP), or the terrestrial UE does (DOWN)
        if (far)
        {
            VoiceLeaves(g_voiceQos.far[call][DOWN], -1, LEG_UE_UPF, packet);
        }
        else
        {
            VoiceLeaves(g_voiceQos.sat[call][UP], call, LEG_UE_UPF, packet);
        }
    }
    if (EndpointOf(destination, &call, &far))
    {
        if (far)
        {
            VoiceEnters(g_voiceQos.far[call][UP], -1, LEG_UPF_UE, packet, packet->GetSize());
        }
        else
        {
            VoiceEnters(g_voiceQos.sat[call][DOWN], call, LEG_UPF_UE, packet, packet->GetSize());
        }
    }
}

// Whether a packet without its IP header is a UDP packet for the voice port
bool
IsVoice(const Ipv4Header& header, Ptr<const Packet> packet)
{
    uint8_t udp[4];
    return header.GetProtocol() == 17 && packet->CopyData(udp, 4) == 4 &&
           ((udp[2] << 8) | udp[3]) == g_voiceQos.port;
}

// "SendOutgoing" of the IPv4 of a UE of call i: a packet it sends, before the IP header is put
// on. far: the terrestrial UE of the call, which talks DOWN; else the satellite UE, UP.
void
UeIpSends(uint32_t call, bool far, const Ipv4Header& header, Ptr<const Packet> packet, uint32_t)
{
    if (IsVoice(header, packet))
    {
        const uint32_t ipBytes = packet->GetSize() + header.GetSerializedSize();
        const Dir dir = far ? DOWN : UP;
        VoiceEnters(g_voiceQos.e2e[call][dir], -1, LEG_E2E, packet, ipBytes);
        VoiceEnters(far ? g_voiceQos.far[call][dir] : g_voiceQos.sat[call][dir],
                    far ? int64_t{-1} : int64_t{call},
                    LEG_UE_UPF,
                    packet,
                    ipBytes);
    }
}

// "LocalDeliver" of the IPv4 of a UE of call i: a packet for this UE, its IP header taken off
void
UeIpDelivers(uint32_t call, bool far, const Ipv4Header& header, Ptr<const Packet> packet, uint32_t)
{
    if (IsVoice(header, packet))
    {
        const Dir dir = far ? UP : DOWN;
        VoiceLeaves(far ? g_voiceQos.far[call][dir] : g_voiceQos.sat[call][dir],
                    far ? int64_t{-1} : int64_t{call},
                    LEG_UPF_UE,
                    packet);
        VoiceLeaves(g_voiceQos.e2e[call][dir], -1, LEG_E2E, packet);
    }
}

// The same two on the remote host, the far end of every call without terrestrial UEs: it talks
// DOWN to the UE it sends to, and hears the UE that sent the packet
void
HostIpSends(const Ipv4Header& header, Ptr<const Packet> packet, uint32_t)
{
    uint32_t call = 0;
    bool far = false;
    if (IsVoice(header, packet) && EndpointOf(header.GetDestination().Get(), &call, &far))
    {
        VoiceEnters(g_voiceQos.e2e[call][DOWN],
                    -1,
                    LEG_E2E,
                    packet,
                    packet->GetSize() + header.GetSerializedSize());
    }
}

void
HostIpDelivers(const Ipv4Header& header, Ptr<const Packet> packet, uint32_t)
{
    uint32_t call = 0;
    bool far = false;
    if (IsVoice(header, packet) && EndpointOf(header.GetSource().Get(), &call, &far))
    {
        VoiceLeaves(g_voiceQos.e2e[call][UP], -1, LEG_E2E, packet);
    }
}

// --farEnd=ue: the bearers of a terrestrial cell are plain NR, whatever the run sets for the
// satellite cell: no ROHC, the nr module's PDCP and RLC sequence numbers (12 and 10 bits) and
// RLC reordering timer (100 ms). drbs is the config path of the bearers of a terrestrial gNB or
// UE; on the gNB (spentMs >= 0) every RLC also takes the gNB's own way from the UPF off the delay
// budget. Set on each new bearer before it carries anything, at both ends alike.
void
PlainFarBearers(const std::string& drbs, double spentMs)
{
    Config::SetFailSafe(drbs + "/NrPdcp/EnableRohc", BooleanValue(false));
    Config::SetFailSafe(drbs + "/NrPdcp/SnSize", UintegerValue(12));
    Config::SetFailSafe(drbs + "/NrRlc/$ns3::NrRlcUm/SnFieldLength", UintegerValue(10));
    Config::SetFailSafe(drbs + "/NrRlc/$ns3::NrRlcUm/ReorderingTimer",
                        TimeValue(MilliSeconds(100)));
    if (spentMs >= 0.0)
    {
        Config::SetFailSafe(drbs + "/NrRlc/$ns3::NrRlcUm/DelayBudgetSpentMs", DoubleValue(spentMs));
    }
}

// --farEnd=ue, "ConnectionReconfiguration" of a terrestrial gNB: it has just set bearers up
void
FarGnbReconfigured(std::string drbs, double spentMs, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    PlainFarBearers(drbs, spentMs);
}

// --farEnd=ue, "DrbCreated" of a terrestrial UE
void
FarUeDrbCreated(std::string drbs, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint8_t lcid)
{
    PlainFarBearers(drbs, -1.0);
}

// --farEnd=ue, "StateTransition" of a terrestrial UE's RRC: the helper gives the precoding
// search of every UE it attaches the subband size of the satellite carrier, set when the search
// is made at the attachment; once that is done, the UE's own carrier gets its own
// (TS 38.214 Table 5.2.1.4-2)
void
SetFarSubband(std::vector<Ptr<NrUePhy>> phys,
              uint32_t subbandRbs,
              uint64_t imsi,
              uint16_t cellId,
              uint16_t rnti,
              NrUeRrc::State from,
              NrUeRrc::State to)
{
    Simulator::ScheduleNow([phys, subbandRbs]() {
        for (const auto& phy : phys)
        {
            if (phy->GetPmSearch())
            {
                phy->GetPmSearch()->SetAttribute("SubbandSize", UintegerValue(subbandRbs));
            }
        }
    });
}

// The address of UE k of the network net.0.0.0/8, from net.0.0.2 on (net.0.0.1 is the UPF's).
// No address ends in .0 or .255, which read as a network and a broadcast address: after
// net.0.0.254 come net.0.1.1, net.0.1.2 and so on.
Ipv4Address
UeAddress(uint8_t net, uint32_t k)
{
    const uint32_t p = k + 1; // the place in the network's sequence of .1 to .254 addresses
    NS_ABORT_MSG_UNLESS(p / 254 < (1U << 16), "more UEs than the network has addresses");
    return Ipv4Address((static_cast<uint32_t>(net) << 24) | ((p / 254) << 8) | (p % 254 + 1));
}

// Give UE k of devs UeAddress(net, k) in net.0.0.0/8, with a default route to the gateway
Ipv4InterfaceContainer
AssignUeAddresses(const NetDeviceContainer& devs, uint8_t net, Ipv4Address gateway)
{
    Ipv4InterfaceContainer ifaces;
    Ipv4StaticRoutingHelper routing;
    const Ipv4Address network(static_cast<uint32_t>(net) << 24);
    for (uint32_t k = 0; k < devs.GetN(); ++k)
    {
        Ipv4AddressHelper one(network,
                              Ipv4Mask("255.0.0.0"),
                              Ipv4Address(UeAddress(net, k).Get() & 0x00ffffff));
        ifaces.Add(one.Assign(NetDeviceContainer(devs.Get(k))));
        Ptr<Ipv4> ip = devs.Get(k)->GetNode()->GetObject<Ipv4>();
        routing.GetStaticRouting(ip)->SetDefaultRoute(gateway,
                                                      ip->GetInterfaceForDevice(devs.Get(k)));
    }
    return ifaces;
}

// Connect a trace and stop with a clear message if the name is wrong.
void
Connect(Ptr<Object> obj, const std::string& name, const CallbackBase& cb)
{
    bool ok = obj->TraceConnectWithoutContext(name, cb);
    NS_ABORT_MSG_UNLESS(ok,
                        "No trace source called \"" << name << "\" on "
                                                    << obj->GetInstanceTypeId().GetName());
}

// One channel per FDD carrier, built as leo-link's CreateNtnCarrier() builds it: the 3GPP
// TR 38.811 model of the environment between satellite and UE, the propagation delay of the
// real path, and for a transparent payload the feeder link on top (here with a loss model of its
// own, FeederLinkLossModel, shared by both carriers). The path loss is a chain: ns-3's NTN path
// loss (free space, clutter loss while blocked), FixedLevelShadowingLossModel (shadowing),
// ServiceLinkLossModel (atmosphere, --linkMarginDb). The fast fading is ns-3's 3GPP channel, or
// LosOnlyChannelModel with --fastFading=fixed.
struct NtnCarrier
{
    Ptr<SpectrumChannel> channel;
    Ptr<PropagationLossModel> serviceLoss;
    Ptr<ServiceLinkLossModel> serviceExtra; // the last of serviceLoss's chain
    Ptr<NrNtnFeederNoiseLossModel> feeder;
};

NtnCarrier
CreateNtnCarrier(double frequencyHz,
                 const std::string& environment,
                 Ptr<MobilityModel> gateway,
                 Ptr<FeederLinkLossModel> feederLoss,
                 Ptr<NrNtnPropagationDelayModel> delayModel,
                 bool transparent,
                 Ptr<FixedLevelChannelConditionModel> levels,
                 bool atmosphereTable,
                 bool ionosphericScintillation,
                 double linkMarginDb,
                 bool fixedFastFading,
                 double fastFadingDb)
{
    NtnCarrier carrier;
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories(environment, "Default", "ThreeGpp");
    carrier.channel = channelHelper->CreateChannel(NrChannelHelper::INIT_PROPAGATION |
                                                   NrChannelHelper::INIT_FADING);
    carrier.serviceLoss = carrier.channel->GetPropagationLossModel();
    carrier.serviceLoss->SetAttributeFailSafe("Frequency", DoubleValue(frequencyHz));
    if (Ptr<PhasedArraySpectrumPropagationLossModel> fading =
            carrier.channel->GetPhasedArraySpectrumPropagationLossModel())
    {
        PointerValue channelModel;
        if (fading->GetAttributeFailSafe("ChannelModel", channelModel))
        {
            channelModel.Get<Object>()->SetAttributeFailSafe("Frequency", DoubleValue(frequencyHz));
            // the fast fading asks the same model for the state of a UE
            channelModel.Get<Object>()->SetAttribute("ChannelConditionModel", PointerValue(levels));
        }
        if (fixedFastFading)
        {
            // --fastFading=fixed: the direct path alone, and --fastFadingDb
            Ptr<ThreeGppSpectrumPropagationLossModel> threeGpp =
                DynamicCast<ThreeGppSpectrumPropagationLossModel>(fading);
            NS_ABORT_MSG_UNLESS(threeGpp, "unexpected fast fading model");
            Ptr<LosOnlyChannelModel> los = CreateObject<LosOnlyChannelModel>();
            los->SetAttribute("Frequency", DoubleValue(frequencyHz));
            los->SetLossDb(fastFadingDb);
            threeGpp->SetChannelModel(los);
        }
    }
    // The state of a UE comes from its own level, and so does its shadow fading, which replaces
    // the one ns-3's path loss model would draw
    Ptr<ThreeGppPropagationLossModel> pathLoss =
        DynamicCast<ThreeGppPropagationLossModel>(carrier.serviceLoss);
    NS_ABORT_MSG_UNLESS(pathLoss && !pathLoss->GetNext(), "unexpected path loss model");
    pathLoss->SetChannelConditionModel(levels);
    pathLoss->SetAttribute("ShadowingEnabled", BooleanValue(false));
    Ptr<FixedLevelShadowingLossModel> shadowing = CreateObject<FixedLevelShadowingLossModel>();
    shadowing->SetLevels(levels);
    pathLoss->SetNext(shadowing);
    // The atmosphere and --linkMarginDb
    carrier.serviceExtra = CreateObject<ServiceLinkLossModel>();
    carrier.serviceExtra->Set(frequencyHz, atmosphereTable, ionosphericScintillation, linkMarginDb);
    Ptr<PropagationLossModel> last = carrier.serviceLoss;
    while (last->GetNext())
    {
        last = last->GetNext();
    }
    last->SetNext(carrier.serviceExtra);
    carrier.channel->SetPropagationDelayModel(delayModel);
    carrier.channel->AddSpectrumTransmitFilter(CreateObject<NrNtnUeToUeFilter>());

    carrier.feeder = CreateObject<NrNtnFeederNoiseLossModel>();
    carrier.feeder->SetServiceLoss(carrier.serviceLoss);
    carrier.feeder->SetFeeder(gateway, feederLoss);
    if (transparent)
    {
        carrier.channel->SetAttribute("PropagationLossModel", PointerValue(carrier.feeder));
    }
    return carrier;
}

// --rtp=1: the voice source of a real VoIP call, in place of the NGMN generator.
//
// Voice activity: the same two-state chain as TrafficGeneratorNgmnVoip, stepped once per 20 ms
// speech frame: talk -> silence with probability 0.01 (2 s mean talk spurt), silence -> talk so
// that the long-run share of talk is the voice activity factor. This is the detector (VAD).
//
// DTX, the state machine of TR 26.940 6.4.3 with the hangover of AMR (TS 26.093 5.1.2.1):
//   active    VAD says talk: one voice frame per 20 ms
//   hangover  the talk has stopped: the encoder still sends `hangoverFrames` voice frames (7),
//             and returns to active without a break if the talk comes back meanwhile
//   DTX       after the hangover: one SID frame at once, then one every `sidEveryFrames` frames
//             (8 = 160 ms), nothing in between, until the VAD says talk again
// The encoder starts as after a reset, i.e. with a hangover before its first DTX (TS 26.093:
// "the first 7 frames after the reset shall always be marked SPEECH_GOOD").
// Not modelled, both from TS 26.093: the second SID of AMR 60 ms after the first one, and the
// hangover that AMR skips after a talk spurt shorter than 24 frames since the last SID update.
//
// RTP (RFC 3550, 12 bytes, so IP/UDP/RTP = 40 bytes and ROHC can use its RTP profile): version 2,
// payload type 96 for voice and SID alike (as AMR, RFC 4867), sequence number +1 per packet,
// timestamp on the 8 kHz clock = +160 per frame, also through the frames without a packet, the
// marker bit on the first voice packet after a DTX period (RFC 4867 4.1), one SSRC per call,
// sequence number and timestamp starting at random values (RFC 3550 5.1). The bits after the
// header are zeros.
class RtpVoiceSource : public Application
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabRtpVoiceSource")
                                .SetParent<Application>()
                                .SetGroupName("Applications")
                                .AddConstructor<RtpVoiceSource>();
        return tid;
    }

    // streams: the first of the two random streams of the source (talk and silence, then where
    // sequence number and timestamp start), plus the call; ssrc: its RTP SSRC
    void Setup(Address remote,
               double voiceActivityFactor,
               uint32_t voiceBytes,
               uint32_t sidBytes,
               uint32_t hangoverFrames,
               uint32_t sidEveryFrames,
               uint32_t call,
               int64_t streams,
               uint32_t ssrc)
    {
        m_remote = remote;
        m_voiceBytes = voiceBytes;
        m_sidBytes = sidBytes;
        m_hangoverFrames = hangoverFrames;
        m_hangoverLeft = hangoverFrames; // as after an encoder reset
        m_sidEveryFrames = sidEveryFrames;
        m_toSilence = 20.0 / 2000.0;
        m_toTalk = m_toSilence * voiceActivityFactor / (1.0 - voiceActivityFactor);
        m_ssrc = ssrc;
        m_coin = CreateObject<UniformRandomVariable>();
        m_coin->SetStream(streams + call); // its own stream: the calls do not disturb each other
        // Where the RTP sequence number and timestamp start: a stream of its own again, so the
        // talk and silence of the call are the same with any other start
        Ptr<UniformRandomVariable> start = CreateObject<UniformRandomVariable>();
        start->SetStream(streams + 100000 + call);
        m_sequence = start->GetInteger(0, 0xffff);
        m_timestampStart = start->GetInteger(0, 0xffff) << 16;
        m_timestampStart |= start->GetInteger(0, 0xffff);
    }

    // The call ends early: not admitted (before it starts) or released by the network
    void End()
    {
        m_ended = true;
        StopApplication();
    }

  private:
    void StartApplication() override
    {
        if (m_ended)
        {
            return; // the call was not admitted
        }
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_socket->Bind();
        m_socket->Connect(m_remote);
        m_socket->ShutdownRecv();
        Frame();
    }

    void StopApplication() override
    {
        m_frameEvent.Cancel();
        if (m_socket)
        {
            m_socket->Close();
            m_socket = nullptr;
        }
    }

    // One 20 ms speech frame
    void Frame()
    {
        m_talking = m_coin->GetValue() < (m_talking ? 1.0 - m_toSilence : m_toTalk);
        if (m_talking)
        {
            Send(m_voiceBytes, true);
            m_hangoverLeft = m_hangoverFrames;
            m_inDtx = false;
        }
        else if (m_hangoverLeft > 0)
        {
            Send(m_voiceBytes, true); // hangover: still a voice frame
            --m_hangoverLeft;
        }
        else
        {
            if (!m_inDtx)
            {
                m_inDtx = true;
                m_framesInDtx = 0;
            }
            if (m_framesInDtx++ % m_sidEveryFrames == 0)
            {
                Send(m_sidBytes, false);
            }
        }
        ++m_frame;
        m_frameEvent = Simulator::Schedule(MilliSeconds(20), &RtpVoiceSource::Frame, this);
    }

    void Send(uint32_t payloadBytes, bool voice)
    {
        // The marker bit: the first voice packet after packets were left out, i.e. after a SID
        // or as the first packet of the call
        const bool marker = voice && !m_lastWasVoice;
        m_lastWasVoice = voice;
        std::vector<uint8_t> rtp(12 + payloadBytes, 0);
        const uint32_t timestamp = m_timestampStart + 160 * m_frame; // 8 kHz x 20 ms
        rtp[0] = 0x80;                            // version 2, no padding, no extension, no CSRC
        rtp[1] = (marker ? 0x80 : 0x00) | 96;     // marker, payload type 96 (dynamic)
        rtp[2] = m_sequence >> 8;
        rtp[3] = m_sequence & 0xff;
        rtp[4] = timestamp >> 24;
        rtp[5] = (timestamp >> 16) & 0xff;
        rtp[6] = (timestamp >> 8) & 0xff;
        rtp[7] = timestamp & 0xff;
        rtp[8] = m_ssrc >> 24;
        rtp[9] = (m_ssrc >> 16) & 0xff;
        rtp[10] = (m_ssrc >> 8) & 0xff;
        rtp[11] = m_ssrc & 0xff;
        ++m_sequence;
        m_socket->Send(Create<Packet>(rtp.data(), rtp.size()));
    }

    Address m_remote;
    Ptr<Socket> m_socket;
    Ptr<UniformRandomVariable> m_coin;
    EventId m_frameEvent;
    bool m_ended{false};
    uint32_t m_voiceBytes{0};
    uint32_t m_sidBytes{0};
    uint32_t m_hangoverFrames{0};
    uint32_t m_sidEveryFrames{8};
    double m_toSilence{0.0};
    double m_toTalk{0.0};
    bool m_talking{false};        // the voice activity detector
    uint32_t m_hangoverLeft{0};   // voice frames still to send after the talk has stopped
    bool m_inDtx{false};
    uint32_t m_framesInDtx{0};
    bool m_lastWasVoice{false};   // the packet sent last was a voice frame
    uint32_t m_frame{0};
    uint16_t m_sequence{0};
    uint32_t m_timestampStart{0};
    uint32_t m_ssrc{0};
};

// --- Call control: admission at call setup, release of calls that miss their budget ----------
// Admission (--admission): when a call is to start, the gNB estimates what it will reserve
// while its speakers talk, per resource (downlink resource blocks, uplink resource blocks,
// PDCCH CCEs; each a number per second), and admits it if, with the calls it carries, that
// stays within --admissionRbPct of the RBs and --admissionCcePct of the CCEs.
// Release (--dropCalls): a call whose packets, in either direction, arrived within the PDB less
// often than --dropOnTimePct over the averaging window of 5QI 1 (TS 23.501 5.7.2.4: the GFBR can
// no longer be guaranteed) is released: its voice stops and the gNB releases the UE's RRC
// connection.
struct CallControl
{
    enum Status : uint8_t
    {
        WAITING = 0, // not started yet
        ADMITTED,
        BLOCKED,
        RELEASED,
        ENDED
    };

    bool admission{false};
    std::array<double, 3> load{1.0, 1.0, 0.6}; // share of each resource calls may reserve
    bool drop{false};
    double dropOnTime{0.8}; // share
    Time window{MilliSeconds(2000)};
    Time pdb{MilliSeconds(100)};
    std::array<double, 3> capacity{}; // per second; 0 = no limit
    std::array<double, 3> reserved{};
    std::function<std::array<double, 3>(uint32_t)> need; // what call i reserves
    std::function<void(uint32_t)> releaseUe;             // the gNB releases UE i
    std::function<void(uint32_t)> callStarts;            // the RRC messages of call i
    std::vector<Status> status;
    std::vector<std::array<double, 3>> held;
    std::vector<Time> at;          // when it was blocked or released
    std::vector<int> limitedBy;    // blocked: the resource that had no room
    std::vector<double> onTimePct; // released: the share of its packets within the PDB
    std::vector<int> releasedDir;  // released: the direction that missed (Dir)
    std::vector<std::array<Ptr<RtpVoiceSource>, 2>> sources;
    std::vector<Time> end;
    uint32_t peakAdmitted{0};
    std::array<double, 3> peakShare{}; // the most the admitted calls reserved, of the capacity
};

CallControl g_cc;
double g_ulBelowDlDb{3.0}; // the uplink CNR on the CNR's RBs below the downlink one (header)
double g_ue0DlCnrDb{0.0};  // UE 0's downlink CNR at --elevation (header)

const char* const g_resourceName[3] = {"downlink RBs", "uplink RBs", "PDCCH CCEs"};

// Call i is to start
void
AdmitCall(uint32_t i)
{
    CallControl& c = g_cc;
    const std::array<double, 3> need = c.need ? c.need(i) : std::array<double, 3>{};
    if (c.admission)
    {
        for (int r = 0; r < 3; ++r)
        {
            if (c.capacity[r] > 0.0 && c.reserved[r] + need[r] > c.load[r] * c.capacity[r])
            {
                c.status[i] = CallControl::BLOCKED;
                c.at[i] = Simulator::Now();
                c.limitedBy[i] = r;
                for (const auto& source : c.sources[i])
                {
                    if (source)
                    {
                        source->End();
                    }
                }
                return;
            }
        }
    }
    c.status[i] = CallControl::ADMITTED;
    c.at[i] = Simulator::Now();
    c.held[i] = need;
    for (int r = 0; r < 3; ++r)
    {
        c.reserved[r] += need[r];
        if (c.capacity[r] > 0.0)
        {
            c.peakShare[r] = std::max(c.peakShare[r], c.reserved[r] / c.capacity[r]);
        }
    }
    c.peakAdmitted =
        std::max<uint32_t>(c.peakAdmitted,
                           std::count(c.status.begin(), c.status.end(), CallControl::ADMITTED));
    if (c.callStarts)
    {
        c.callStarts(i);
    }
}

// Call i ends as planned
void
EndCall(uint32_t i)
{
    CallControl& c = g_cc;
    if (c.status[i] == CallControl::ADMITTED)
    {
        c.status[i] = CallControl::ENDED;
        for (int r = 0; r < 3; ++r)
        {
            c.reserved[r] -= c.held[i][r];
        }
    }
}

// Every 100 ms: the calls whose packets missed the budget too often over the last window
void
CheckCalls()
{
    CallControl& c = g_cc;
    const int64_t now = Simulator::Now().GetNanoSeconds();
    // A bin counts once all its packets have had the PDB to arrive in
    const int64_t last = now - c.pdb.GetNanoSeconds() - 100000000;
    const int64_t first = last - c.window.GetNanoSeconds();
    for (uint32_t i = 0; i < c.status.size(); ++i)
    {
        if (c.status[i] != CallControl::ADMITTED || CurrentConnection(i) < 0 ||
            Simulator::Now() < c.at[i] + c.window + c.pdb + MilliSeconds(100))
        {
            continue; // (out of coverage after a radio link failure: that counts as such)
        }
        for (const Dir dir : {DOWN, UP})
        {
            // The packets that left within the PDB, of those that left at all or were
            // discarded for waiting too long: a packet lost on the radio is no sign of a full
            // cell (a radio link failure takes care of a link that fails)
            uint64_t judged = 0;
            uint64_t onTime = 0;
            for (const auto& b : g_voiceQos.sat[i][dir].bins)
            {
                if (b.startNs > first && b.startNs <= last)
                {
                    judged += b.onTime + b.late + b.dropped;
                    onTime += b.onTime;
                }
            }
            if (judged >= 10 && static_cast<double>(onTime) < c.dropOnTime * judged)
            {
                c.status[i] = CallControl::RELEASED;
                c.at[i] = Simulator::Now();
                c.onTimePct[i] = 100.0 * onTime / judged;
                c.releasedDir[i] = dir;
                for (int r = 0; r < 3; ++r)
                {
                    c.reserved[r] -= c.held[i][r];
                }
                for (const auto& source : c.sources[i])
                {
                    if (source)
                    {
                        source->End();
                    }
                }
                if (c.releaseUe)
                {
                    c.releaseUe(i);
                }
                break;
            }
        }
    }
    Simulator::Schedule(MilliSeconds(100), &CheckCalls);
}

// The LEO satellites of TR 38.821 Table 6.1.1.1-1 (Set-1) and Table 6.1.1.1-2 (Set-2), S-band
// rows. Set-2 is the smaller antenna (1 m instead of 2 m): 6 dB less gain and EIRP, a beam twice
// as wide. The tables also have a GEO column and Ka-band rows; this example is set up for
// neither (GEO needs other HARQ and RLC settings, Ka-band the VSAT terminal).
// The S-band rows are "i.e. 2 GHz", for downlink and uplink: that is the carrier the beamwidth
// of the table belongs to.
struct SatelliteColumn
{
    const char* orbit;
    double altitudeKm;
    double eirpDbwPerMHz; // satellite EIRP density
    double gainDbi;       // satellite Tx max gain = Rx max gain
    double hpbwDeg;       // 3 dB beamwidth
    double gtDbPerK;      // G/T
    double apertureM;     // equivalent satellite antenna aperture (the diameter)
};
const SatelliteColumn g_satelliteSets[2][2] = {
    {{"LEO-600", 600.0, 34.0, 30.0, 4.4127, 1.1, 2.0},
     {"LEO-1200", 1200.0, 40.0, 30.0, 4.4127, 1.1, 2.0}},
    {{"LEO-600", 600.0, 28.0, 24.0, 8.8320, -4.9, 1.0},
     {"LEO-1200", 1200.0, 34.0, 24.0, 8.8320, -4.9, 1.0}},
};
const double g_satelliteSetCarrierHz = 2e9;

} // namespace

int
main(int argc, char* argv[])
{
    uint32_t nUes = 10;
    double elevation = 30.0; // of the satellite, seen from UE 0 (deg): always, or when the calls start
    bool freezeGeometry = true; // false: the satellite flies its orbit
    bool fullPass = false; // true: it flies the whole pass over UE 0, from 10 deg up to 10 deg down
    std::string ueLayout = "random"; // UE 0 on the beam centre; the others: random or in a line
    double ueAntennaGainDbi = -5.0;  // handheld antenna, both carriers (TR 38.821's budget: 0)
    // The losses of the service link besides free space (see the top of the file); the defaults
    // are those of TR 38.821's link budget (Table 6.1.3.3-1, S-band) with --scenario=NTN-LOS
    std::string shadowing = "fixed";  // fixed: --shadowingDb for every UE; table: TR 38.811
    double shadowingDb = 3.0;         // --shadowing=fixed: TR 38.821's shadowing margin
    std::string atmosphere = "table"; // table: TR 38.821's 0.1 + 2.2 dB; location: by elevation
    double latitudeDeg = 45.0;        // --atmosphere=location: magnetic latitude of the UEs
    std::string fastFading = "fixed"; // fixed: --fastFadingDb on every RB; model: 3GPP channel
    double fastFadingDb = 0.0;        // --fastFading=fixed: its loss
    double linkMarginDb = 0.0;        // more loss on every link: TR 38.821's additional losses
    double spacingKm = 2.0;           // --ueLayout=line: UE i is i x spacingKm from the beam centre
    Time duration = Seconds(20);
    std::string direction = "both"; // both: two-way calls; dl: the far end talks; ul: the UE talks
    std::string farEnd = "host";    // host: the remote host; ue: a terrestrial UE per call
    std::string farEndBand = "n78"; // --farEnd=ue: the band of the terrestrial cells, n78 or n3
    // --farEnd=ue, the terrestrial cells: bandwidth, carrier (0: the band's usual one), calls per
    // gNB, gNB power per 20 MHz, where the UEs are, the channel, the TDD slots, the error model
    double farEndBandwidthMHz = 20.0;
    double farEndFrequencyGHz = 0.0;
    uint32_t farEndUesPerGnb = 10;
    double farEndGnbDbm = 44.0;
    double farEndMinDistM = 35.0;
    double farEndMaxDistM = 250.0;
    std::string farEndScenario = "UMa-LOS";
    std::string farEndTddPattern = "DDSUU";
    std::string farEndErrorModel = ""; // "": the satellite cell's, --errorModel
    double farEndUpfDelayMs = 5.0;  // --farEnd=ue: terrestrial gNB <-> UPF, one way
    bool idealControl = true; // false: the PDCCH has a block error rate (NrUePhy::PdcchErrorModel)
    std::string scenario = "NTN-LOS"; // NTN-LOS: never blocked, rural tables otherwise
    bool transparent = true; // false: regenerative, the gNB is on the satellite
    std::string scheduler = "ns3::NrMacSchedulerOfdmaRR"; // OFDMA: a UE gets a few RBs
    std::string errorModel = "NrLteMiErrorModel"; // and with it the MCS table, both directions
    uint32_t ulGrantRbs = 2;     // most RBs of an uplink grant: 2 = 360 kHz (TR 38.821); 0 = all
    double dlBoostMaxDb = 0.0;   // OFDMA: the most an RB in use may get from the idle ones (dB)
    double dlPowerRefDeg = 0.0;  // downlink power control: the reference elevation, 0 = off
    double ulPowerRefDeg = 0.0;  // uplink power control (open loop): the same, 0 = off
    bool harq = true;            // false: no retransmissions, every transport block is sent once
    uint32_t harqProcesses = 16; // per UE and direction (Rel-17 NTN: up to 32)
    bool ulLinkAdaptation = true; // UL MCS for the RBs of each grant, and an outer loop
    uint32_t puschRepetitions = 0; // slots a UE sends each uplink block in; 0 = as many as needed
    double puschRepSinrDb = 2.0;   // --puschRepetitions=0: K brings K x the SINR up to this (dB)
    uint32_t srGrantBytes = 0;    // grant for a scheduling request; 0 = a voice frame and a BSR
    double targetBler = 0.0;     // of first transmissions; 0 = 0.15 with HARQ, 0.01 without
    bool nrTraces = false; // true: NrHelper::EnableTraces(), about 25 .txt files in the run folder
    bool rohc = NrPdcp::IsRohcAvailable(); // on whenever the nr module was built with the ROHC library
    std::string rohcModel = "table";       // table: reference header sizes, library: real ROHC
    std::string rohcMode = "O";            // U = no feedback, O = feedback PDUs on the reverse link
    uint32_t rohcIrPackets = 4;            // size table: IR packets in a row (optimistic approach)
    uint32_t rohcUpdatePackets = 4;        // size table: context updates in a row for a change
    uint32_t rohcDynamicWindow = 13;       // size table: packets missing in a row that UO-0 survives
    uint32_t rohcStaticWindow = 61;        // size table: ... that a context update still repairs
    uint32_t rohcNackAfter = 0;  // O-mode: failed packets (library: of the last 32) before the
                                 // NACK; 0 = 1 (library: 10)
    uint32_t rohcNackEvery = 32; // O-mode: ... and before it is sent again
    std::string l2Headers = "1+1+3";       // PDCP + RLC + MAC header bytes, or 2+2+2
    std::string rohcReport = "summary";    // none, summary (one line per call) or full
    bool rtp = true;             // RTP voice source: IP/UDP/RTP = 40 bytes; false: NGMN generator
    double codecKbps = 4.75;     // AMR 4.75: 95 bits per 20 ms frame
    uint32_t voicePayload = 0;   // bytes of one voice frame, 0 = what the codec bit rate gives
    uint32_t sidPayload = 7;     // bytes of one silence descriptor (SID) frame: 56 bits
    uint32_t hangoverFrames = 7; // voice frames still sent after the talk has stopped
    uint32_t sidIntervalMs = 160; // in DTX: a SID at once and then every this many ms
    double bandwidthMHz = 30.0;
    // The satellite. --satSet and --altitudeKm pick a column of the TR 38.821 table above; the
    // four values left at NaN are then taken from that column.
    uint32_t satSet = 1;
    double altitudeKm = 600.0;
    double dlFrequencyGHz = 2.0;
    double ulFrequencyGHz = 0.0; // 0 = 190 MHz below the downlink
    double satEirpDbwPerMHz = NAN;
    double satAntennaGainDbi = NAN;
    double satGtDbPerK = NAN;
    double satApertureM = NAN;
    double beamHpbwDeg = NAN; // not given: the beam of the aperture on each carrier
    double gatewayDiameterKm = 200.0; // transparent only: the gateway is somewhere in a circle of
                                      // this diameter around UE 0; 0 = where UE 0 is
    // The feeder link, transparent only: gateway -> satellite in Ka band (see the top of the file)
    double feederFrequencyGHz = 28.75;  // OneWeb's gateway uplink, 27.5-30 GHz
    double gatewayGainDbi = 58.0;       // ... its 3.5 m antenna at 28.75 GHz
    double gatewayDbwPerMHz = -19.0;    // ... fed with this: EIRP density 39 dBW/MHz
    double satFeederGtDbPerK = 11.4;    // OneWeb's satellite, gateway receive beam
    double feederBandwidthMHz = 2100.0; // 27.5-29.1 and 29.5-30.0 GHz: for the carrier count
    double hostGwDelayMs = 20.0;   // remote host <-> gateway, one way, the UPF somewhere between
    double upfGwDelayMinMs = 0.0;  // UPF <-> gateway, one way: the part of it that is drawn for
    double upfGwDelayMaxMs = 10.0; // the run, uniformly between these two
    uint16_t port = 4000;
    double gfbrKbps = 40.0; // guaranteed (and maximum) bit rate of the 5QI 1 voice flow
    uint32_t startOffsetMs = 1; // call i starts i x startOffsetMs after UE 0's (at 1 s)
    double vafMean = 0.45; // voice activity factor of each call ~ Gaussian(vafMean, vafStd)
    double vafStd = 0.15;  // 0 = every call gets vafMean
    bool pdcch = false;         // true: the PDCCH is a resource (NrPdcch): CCEs, aggregation levels
    uint32_t pdcchSymbols = 1;  // PDCCH symbols per slot: 1, 2 or 3; 0 = what each slot needs
    uint32_t dciBits = 34;      // size of a DCI without its CRC
    std::string pdcchBlerTable; // SINR:BLER pairs for aggregation level 1; empty = NrPdcch's own
    std::string scheduling = "dynamic"; // dynamic, sps1 or sps2 (voice only)
    std::string csv;         // the results also as CSV files <csv>_calls.csv, ...; "" = none
    bool csvPackets = false; // --csv: also one line per delivered voice packet
    bool admission = true;   // admit a call only if the cell has room for it
    double admissionCcePct = 60; // % of the PDCCH's CCEs the admitted calls may reserve
    double admissionRbPct = 100; // % of the RBs (each way) the admitted calls may reserve
    bool dropCalls = true;       // release a call that misses its delay budget too often
    double dropOnTimePct = 80;   // ... below this share of packets within the PDB

    CommandLine cmd(__FILE__);
    // In alphabetical order, as --PrintHelp lists them: one line each there (100 columns); the
    // comment at the top of the file has the rest
    cmd.AddValue("admission",
                 "1 = start a call only if its needs fit (--admissionCcePct, RbPct)",
                 admission);
    cmd.AddValue("admissionCcePct",
                 "--pdcch=1: % of the CCEs that admitted calls may reserve, talking",
                 admissionCcePct);
    cmd.AddValue("admissionRbPct",
                 "% of the RBs (each way) that admitted calls may reserve, talking",
                 admissionRbPct);
    cmd.AddValue("altitudeKm",
                 "Altitude (km), 300 to 2000; set column: the nearer of 600 and 1200",
                 altitudeKm);
    cmd.AddValue("atmosphere",
                 "Gases + scintillation: table = 0.1 + 2.2 dB; location = TR 38.811",
                 atmosphere);
    cmd.AddValue("bandwidthMHz",
                 "Channel bandwidth of each FDD carrier (MHz): 5, 10, 15, 20 ... 50",
                 bandwidthMHz);
    cmd.AddValue("beamHpbwDeg",
                 "3 dB beam width (deg) on both carriers, instead of the aperture's",
                 beamHpbwDeg);
    cmd.AddValue("codecKbps",
                 "Codec bit rate (kbit/s): voice frame = 20 ms of it, in whole bytes",
                 codecKbps);
    cmd.AddValue("csv",
                 "Also write the results as CSV files <csv>_calls.csv etc.; \"\" = none",
                 csv);
    cmd.AddValue("csvPackets",
                 "--csv: also <csv>_packets.csv, a line per delivered voice packet",
                 csvPackets);
    cmd.AddValue("dciBits", "--pdcch=1: size of a DCI in bits, without its 24-bit CRC", dciBits);
    cmd.AddValue("direction",
                 "both = two-way calls; dl: far end -> UE only; ul: UE -> far end",
                 direction);
    cmd.AddValue("dlBoostMaxDb",
                 "OFDMA: the most dB an RB in use takes from the idle RBs of its symbol",
                 dlBoostMaxDb);
    cmd.AddValue("dlFrequencyGHz", "Downlink carrier (GHz), 0.6 to 6", dlFrequencyGHz);
    cmd.AddValue("dlPowerRefDeg",
                 "DL power control: full power at this elevation, less above; 0 = off",
                 dlPowerRefDeg);
    cmd.AddValue("dropCalls",
                 "1 = release a call that misses its PDB too often (over 2 s)",
                 dropCalls);
    cmd.AddValue("dropOnTimePct",
                 "--dropCalls=1: release below this % of packets within the PDB",
                 dropOnTimePct);
    cmd.AddValue("duration",
                 "UE 0's call length (the others end with it); not with --fullPass=1",
                 duration);
    cmd.AddValue("elevation",
                 "Elevation at UE 0 at call start (deg, 10-90); --fullPass=1: 10",
                 elevation);
    cmd.AddValue("errorModel",
                 "NrLteMiErrorModel (LTE), NrEesm + Ir|Cc + T1|T2 (NR)",
                 errorModel);
    cmd.AddValue("farEnd",
                 "Far end of the calls: host = the remote host, ue = terrestrial UEs",
                 farEnd);
    cmd.AddValue("farEndBand",
                 "--farEnd=ue: band of the terrestrial cells: n78 (TDD) or n3 (FDD)",
                 farEndBand);
    cmd.AddValue("farEndBandwidthMHz",
                 "--farEnd=ue: cell bandwidth (MHz), one of the band's (TS 38.101-1)",
                 farEndBandwidthMHz);
    cmd.AddValue("farEndErrorModel",
                 "--farEnd=ue: error model of the cells; \"\" = --errorModel's",
                 farEndErrorModel);
    cmd.AddValue("farEndFrequencyGHz",
                 "--farEnd=ue: DL carrier (GHz) in the band; 0 = 3.5 / 1.8425",
                 farEndFrequencyGHz);
    cmd.AddValue("farEndGnbDbm",
                 "--farEnd=ue: gNB power (dBm) per 20 MHz, scaled to the bandwidth",
                 farEndGnbDbm);
    cmd.AddValue("farEndMaxDistM",
                 "--farEnd=ue: UEs at most this far from the gNB (m)",
                 farEndMaxDistM);
    cmd.AddValue("farEndMinDistM",
                 "--farEnd=ue: UEs at least this far from the gNB (m)",
                 farEndMinDistM);
    cmd.AddValue("farEndScenario",
                 "--farEnd=ue: TR 38.901 UMa, UMi or RMa; with -LOS: always LOS",
                 farEndScenario);
    cmd.AddValue("farEndTddPattern",
                 "--farEnd=ue, n78: the slots, D/S/U/F (DDSUU: as many D as U)",
                 farEndTddPattern);
    cmd.AddValue("farEndUesPerGnb",
                 "--farEnd=ue: calls (terrestrial UEs) per gNB; one UE per call",
                 farEndUesPerGnb);
    cmd.AddValue("farEndUpfDelayMs",
                 "--farEnd=ue: terrestrial gNB - UPF delay, one way (ms)",
                 farEndUpfDelayMs);
    cmd.AddValue("fastFading",
                 "fixed = --fastFadingDb, flat; model = TR 38.901/38.811 channel",
                 fastFading);
    cmd.AddValue("fastFadingDb", "--fastFading=fixed: its loss (dB) on every RB", fastFadingDb);
    cmd.AddValue("feederBandwidthMHz",
                 "Feeder link: gateway uplink spectrum (MHz), to count carriers",
                 feederBandwidthMHz);
    cmd.AddValue("feederFrequencyGHz",
                 "Feeder link: gateway -> satellite carrier (GHz), 10 to 100",
                 feederFrequencyGHz);
    cmd.AddValue("freezeGeometry",
                 "1 = satellite still at --elevation, 0 = it flies; --fullPass=1: 0",
                 freezeGeometry);
    cmd.AddValue("fullPass",
                 "1 = the whole pass over UE 0, 10 deg up to 10 deg down (it flies)",
                 fullPass);
    cmd.AddValue("gatewayDbwPerMHz",
                 "Feeder link: power density into the gateway antenna (dBW/MHz)",
                 gatewayDbwPerMHz);
    cmd.AddValue("gatewayDiameterKm",
                 "Gateway: random within this diameter around UE 0 (km); 0 = at UE 0",
                 gatewayDiameterKm);
    cmd.AddValue("gatewayGainDbi", "Feeder link: gateway antenna gain (dBi)", gatewayGainDbi);
    cmd.AddValue("gfbrKbps", "Guaranteed bit rate of the 5QI 1 voice flow (kbit/s)", gfbrKbps);
    cmd.AddValue("hangoverFrames",
                 "--rtp=1: voice frames still sent after the talk stops, before DTX",
                 hangoverFrames);
    cmd.AddValue("harq", "1 = HARQ, up to 3 retransmissions; 0 = every block is sent once", harq);
    cmd.AddValue("harqProcesses", "HARQ processes per UE and direction, 1 to 32", harqProcesses);
    cmd.AddValue("hostGwDelayMs",
                 "Delay remote host - gateway, one way (ms); the UPF is on that way",
                 hostGwDelayMs);
    cmd.AddValue("idealControl",
                 "1 = no control message is ever lost; 0 = lost at low SINR",
                 idealControl);
    cmd.AddValue("l2Headers",
                 "PDCP+RLC+MAC header bytes: 1+1+3 (TR 26.940) or 2+2+2 (nr module)",
                 l2Headers);
    cmd.AddValue("latitudeDeg",
                 "--atmosphere=location: magnetic latitude of the UEs (deg), -60 to 60",
                 latitudeDeg);
    cmd.AddValue("linkMarginDb",
                 "Additional losses of TR 38.821 (dB), on each satellite-UE link",
                 linkMarginDb);
    cmd.AddValue("nrTraces", "Write the NR module's PHY/MAC/RLC/PDCP trace files", nrTraces);
    cmd.AddValue("nUes", "Number of UEs, one call each", nUes);
    cmd.AddValue("pdcch", "1 = the PDCCH is a limited resource (CCEs); 0 = unlimited", pdcch);
    cmd.AddValue("pdcchBlerTable",
                 "--pdcch=1: level-1 PDCCH BLER, \"SINR:BLER ...\"; empty: the nr module's",
                 pdcchBlerTable);
    cmd.AddValue("pdcchSymbols",
                 "PDCCH symbols per slot: 1, 2 or 3; 0 = as many as each slot needs",
                 pdcchSymbols);
    cmd.AddValue("puschRepetitions",
                 "UL slots per block (PUSCH repetition), 1-32; 0 = as needed",
                 puschRepetitions);
    cmd.AddValue("puschRepSinrDb",
                 "--puschRepetitions=0: K brings K x a UE's SINR to this (dB)",
                 puschRepSinrDb);
    cmd.AddValue("rohc", "Compress the IP/UDP headers in PDCP with ROHC", rohc);
    cmd.AddValue("rohcDynamicWindow",
                 "Size table: packets missing in a row that UO-0 survives",
                 rohcDynamicWindow);
    cmd.AddValue("rohcIrPackets",
                 "Size table: IR packets in a row (start, refresh, STATIC-NACK)",
                 rohcIrPackets);
    cmd.AddValue("rohcMode",
                 "U = no feedback, repair by refresh; O = NACKs on the reverse link",
                 rohcMode);
    cmd.AddValue("rohcModel",
                 "table = header sizes from a table, library = the ROHC library",
                 rohcModel);
    cmd.AddValue("rohcNackAfter",
                 "O-mode NACK: n failed in a row (library: n in last 32); 0 = 1 / 10",
                 rohcNackAfter);
    cmd.AddValue("rohcNackEvery",
                 "O-mode: the NACK is sent again every this many failed packets",
                 rohcNackEvery);
    cmd.AddValue("rohcReport", "ROHC report at the end: none, summary or full", rohcReport);
    cmd.AddValue("rohcStaticWindow",
                 "Size table: packets missing in a row that a context update survives",
                 rohcStaticWindow);
    cmd.AddValue("rohcUpdatePackets",
                 "Size table: context updates in a row for each change of the header",
                 rohcUpdatePackets);
    cmd.AddValue("rtp", "1 = RTP source: 40 B headers, hangover; 0 = NGMN generator: 28 B", rtp);
    cmd.AddValue("satAntennaGainDbi",
                 "Satellite Tx and Rx antenna gain (dBi), default: the set",
                 satAntennaGainDbi);
    cmd.AddValue("satApertureM",
                 "Antenna aperture (m), default: the set; it sets the beam width",
                 satApertureM);
    cmd.AddValue("satEirpDbwPerMHz",
                 "Satellite EIRP density (dBW/MHz), default: the set",
                 satEirpDbwPerMHz);
    cmd.AddValue("satFeederGtDbPerK",
                 "Feeder link: satellite G/T towards the gateway (dB/K)",
                 satFeederGtDbPerK);
    cmd.AddValue("satGtDbPerK", "Satellite G/T (dB/K), default: the set", satGtDbPerK);
    cmd.AddValue("satSet", "TR 38.821 Table 6.1.1.1-1/-2, S-band: 1 = Set-1, 2 = Set-2", satSet);
    cmd.AddValue("scenario",
                 "NTN-LOS (unblocked), NTN-Rural, -Suburban, -Urban, -DenseUrban",
                 scenario);
    cmd.AddValue("scheduler", "TypeId: Tdma|Ofdma + RR|PF|MR|Qos|Random", scheduler);
    cmd.AddValue("scheduling",
                 "Voice: dynamic, or semi-persistent: sps1, sps2 (scenario 1, 2)",
                 scheduling);
    cmd.AddValue("shadowing",
                 "fixed = --shadowingDb, all UEs; table = random per UE (TR 38.811)",
                 shadowing);
    cmd.AddValue("shadowingDb", "--shadowing=fixed: its loss (dB)", shadowingDb);
    cmd.AddValue("sidIntervalMs",
                 "DTX: a SID frame at once, then every this many ms (multiple of 20)",
                 sidIntervalMs);
    cmd.AddValue("sidPayload", "Bytes of one silence descriptor (SID) frame", sidPayload);
    cmd.AddValue("spacingKm",
                 "--ueLayout=line: UE i is i x spacingKm east of the beam centre (km)",
                 spacingKm);
    cmd.AddValue("srGrantBytes",
                 "Grant for a scheduling request (B): 0 = a voice frame + BSR",
                 srGrantBytes);
    cmd.AddValue("startOffsetMs",
                 "Call i starts i x this many ms after UE 0's, which starts at 1 s",
                 startOffsetMs);
    cmd.AddValue("targetBler",
                 "First-transmission block error target; 0 = 0.15 (--harq=0: 0.01)",
                 targetBler);
    cmd.AddValue("transparent",
                 "1 = gNB at the gateway (feeder link), 0 = gNB on the satellite",
                 transparent);
    cmd.AddValue("ueAntennaGainDbi",
                 "Handheld antenna gain (dBi), downlink and uplink (TR 38.821: 0)",
                 ueAntennaGainDbi);
    cmd.AddValue("ueLayout", "UE 0 is on the beam centre; the others: random or line", ueLayout);
    cmd.AddValue("ulFrequencyGHz",
                 "Uplink carrier (GHz), 0 = 190 MHz below the downlink",
                 ulFrequencyGHz);
    cmd.AddValue("ulGrantRbs",
                 "OFDMA: most RBs of an uplink grant; 2 = 360 kHz (TR 38.821), 0 = all",
                 ulGrantRbs);
    cmd.AddValue("ulLinkAdaptation",
                 "1 = UL MCS for the RBs of a grant, outer loop; 0 = nr module's",
                 ulLinkAdaptation);
    cmd.AddValue("ulPowerRefDeg",
                 "UL power control: 23 dBm at this elevation, less above; 0 = off",
                 ulPowerRefDeg);
    cmd.AddValue("upfGwDelayMaxMs",
                 "UPF - gateway, the random part of --hostGwDelayMs: at most this (ms)",
                 upfGwDelayMaxMs);
    cmd.AddValue("upfGwDelayMinMs",
                 "UPF - gateway, the random part of --hostGwDelayMs: at least this (ms)",
                 upfGwDelayMinMs);
    cmd.AddValue("vafMean", "Mean of the per-call voice activity factor", vafMean);
    cmd.AddValue("vafStd", "Standard deviation of the per-call voice activity factor", vafStd);
    cmd.AddValue("voicePayload", "Bytes of one voice frame, 0 = from --codecKbps", voicePayload);
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
    const double elevationDefault = elevation;
    const Time durationDefault = duration;
    cmd.Parse(argc, argv);
    // --fullPass=1 sets these itself
    const bool passIgnored = fullPass && (elevation != elevationDefault || duration != durationDefault);
    NS_ABORT_MSG_UNLESS(direction == "both" || direction == "ul" || direction == "dl",
                        "--direction must be both, dl or ul");
    NS_ABORT_MSG_UNLESS(farEnd == "host" || farEnd == "ue", "--farEnd must be host or ue");
    if (errorModel.compare(0, 5, "ns3::") == 0)
    {
        errorModel.erase(0, 5);
    }
    NS_ABORT_MSG_UNLESS(errorModel == "NrLteMiErrorModel" || errorModel == "NrEesmIrT1" ||
                            errorModel == "NrEesmCcT1" || errorModel == "NrEesmIrT2" ||
                            errorModel == "NrEesmCcT2",
                        "--errorModel must be NrLteMiErrorModel, NrEesmIrT1, NrEesmCcT1, "
                        "NrEesmIrT2 or NrEesmCcT2");
    const bool chaseCombining = errorModel.find("Cc") != std::string::npos;
    NS_ABORT_MSG_UNLESS(farEndBand == "n78" || farEndBand == "n3",
                        "--farEndBand must be n78 or n3");
    NS_ABORT_MSG_UNLESS(farEndUpfDelayMs >= 0.0 && farEndUpfDelayMs <= 50.0,
                        "--farEndUpfDelayMs must be 0 to 50");
    // The terrestrial cells. RBs of a channel: TS 38.101-1 Table 5.3.2-1, 30 kHz for n78 and
    // 15 kHz for n3; the channel bandwidths of each band: Table 5.3.5-1
    const bool farTdd = (farEndBand == "n78");
    const std::map<int, uint32_t> farRbTable =
        farTdd
            ? std::map<int, uint32_t>{{10, 24},
                                      {15, 38},
                                      {20, 51},
                                      {25, 65},
                                      {30, 78},
                                      {40, 106},
                                      {50, 133},
                                      {60, 162},
                                      {70, 189},
                                      {80, 217},
                                      {90, 245},
                                      {100, 273}}
            : std::map<int, uint32_t>{{5, 25}, {10, 52}, {15, 79}, {20, 106}, {25, 133}, {30, 160}};
    const auto farRbIt = farRbTable.find(static_cast<int>(std::lround(farEndBandwidthMHz)));
    NS_ABORT_MSG_UNLESS(farRbIt != farRbTable.end() &&
                            std::abs(farEndBandwidthMHz - farRbIt->first) < 1e-9,
                        "--farEndBandwidthMHz: " << farEndBand << " has "
                                                 << (farTdd ? "10, 15, 20, 25, 30, 40, 50, 60, 70, "
                                                              "80, 90 and 100 MHz"
                                                            : "5, 10, 15, 20, 25 and 30 MHz"));
    const uint32_t farRbs = farRbIt->second;
    // the carrier: n78 3300-3800 MHz (TDD); n3 downlink 1805-1880 MHz, uplink 95 MHz below
    const double farDlHz = farEndFrequencyGHz > 0.0 ? farEndFrequencyGHz * 1e9
                           : farTdd                 ? 3.5e9
                                                    : 1842.5e6;
    const double farUlHz = farTdd ? farDlHz : farDlHz - 95e6;
    const double farBandLowHz = farTdd ? 3300e6 : 1805e6;
    const double farBandHighHz = farTdd ? 3800e6 : 1880e6;
    NS_ABORT_MSG_UNLESS(farDlHz - farEndBandwidthMHz * 0.5e6 >= farBandLowHz - 1.0 &&
                            farDlHz + farEndBandwidthMHz * 0.5e6 <= farBandHighHz + 1.0,
                        "--farEndFrequencyGHz: the "
                            << farEndBandwidthMHz << " MHz channel must lie in " << farEndBand
                            << ", " << farBandLowHz / 1e6 << " to " << farBandHighHz / 1e6
                            << " MHz (downlink)");
    NS_ABORT_MSG_UNLESS(farEndUesPerGnb >= 1, "--farEndUesPerGnb must be 1 or more");
    NS_ABORT_MSG_UNLESS(farEndGnbDbm >= 0.0 && farEndGnbDbm <= 60.0,
                        "--farEndGnbDbm must be 0 to 60");
    NS_ABORT_MSG_UNLESS(farEndMinDistM >= 10.0 && farEndMinDistM < farEndMaxDistM &&
                            farEndMaxDistM <= 5000.0,
                        "--farEndMinDistM and --farEndMaxDistM: 10 <= min < max <= 5000 m");
    NS_ABORT_MSG_UNLESS(farEndScenario == "UMa-LOS" || farEndScenario == "UMa" ||
                            farEndScenario == "UMi-LOS" || farEndScenario == "UMi" ||
                            farEndScenario == "RMa-LOS" || farEndScenario == "RMa",
                        "--farEndScenario must be UMa-LOS, UMa, UMi-LOS, UMi, RMa-LOS or RMa");
    NS_ABORT_MSG_UNLESS(!farEndTddPattern.empty() && farEndTddPattern.size() <= 20 &&
                            farEndTddPattern.find_first_not_of("DSUF") == std::string::npos &&
                            farEndTddPattern.find('D') != std::string::npos &&
                            farEndTddPattern.find('U') != std::string::npos,
                        "--farEndTddPattern: 1 to 20 slots of D, S, U or F, with a D and a U");
    if (farEndErrorModel.compare(0, 5, "ns3::") == 0)
    {
        farEndErrorModel.erase(0, 5);
    }
    if (farEndErrorModel.empty())
    {
        farEndErrorModel = errorModel;
    }
    NS_ABORT_MSG_UNLESS(farEndErrorModel == "NrLteMiErrorModel" ||
                            farEndErrorModel == "NrEesmIrT1" || farEndErrorModel == "NrEesmCcT1" ||
                            farEndErrorModel == "NrEesmIrT2" || farEndErrorModel == "NrEesmCcT2",
                        "--farEndErrorModel must be NrLteMiErrorModel, NrEesmIrT1, NrEesmCcT1, "
                        "NrEesmIrT2 or NrEesmCcT2");
    NS_ABORT_MSG_UNLESS(dlBoostMaxDb >= 0.0 && dlBoostMaxDb <= 30.0,
                        "--dlBoostMaxDb must be 0 to 30");
    NS_ABORT_MSG_UNLESS(dlPowerRefDeg == 0.0 || (dlPowerRefDeg >= 10.0 && dlPowerRefDeg <= 90.0),
                        "--dlPowerRefDeg must be 0 (off) or 10 to 90");
    NS_ABORT_MSG_UNLESS(ulPowerRefDeg == 0.0 || (ulPowerRefDeg >= 10.0 && ulPowerRefDeg <= 90.0),
                        "--ulPowerRefDeg must be 0 (off) or 10 to 90");
    const bool tdma = scheduler.find("Tdma") != std::string::npos;
    NS_ABORT_MSG_IF(tdma && ulGrantRbs > 0,
                    "--ulGrantRbs needs an OFDMA scheduler: a TDMA scheduler gives a UE whole "
                    "symbols of the band. Add --ulGrantRbs=0 --puschRepetitions=1");
    NS_ABORT_MSG_UNLESS(pdcchSymbols <= 3, "--pdcchSymbols must be 0, 1, 2 or 3");
    NS_ABORT_MSG_UNLESS(puschRepetitions <= 32, "--puschRepetitions must be 0 to 32");
    NS_ABORT_MSG_UNLESS(admissionRbPct > 0.0 && admissionRbPct <= 1000.0 && admissionCcePct > 0.0 &&
                            admissionCcePct <= 1000.0,
                        "--admissionRbPct and --admissionCcePct must be above 0, at most 1000");
    NS_ABORT_MSG_UNLESS(dropOnTimePct >= 0.0 && dropOnTimePct <= 100.0,
                        "--dropOnTimePct must be 0 to 100");
    // Admission and release end a call's voice source: the RTP source can, the NGMN generator not
    const bool callControlOff = !rtp && (admission || dropCalls);
    if (!rtp)
    {
        admission = false;
        dropCalls = false;
    }
    NS_ABORT_MSG_UNLESS(std::isfinite(puschRepSinrDb) && std::abs(puschRepSinrDb) <= 30.0,
                        "--puschRepSinrDb must be -30 to 30");
    NS_ABORT_MSG_IF(tdma && puschRepetitions != 1,
                    "PUSCH repetition needs an OFDMA scheduler. Add --puschRepetitions=1");
    NS_ABORT_MSG_IF(puschRepetitions == 0 && !ulLinkAdaptation,
                    "--puschRepetitions=0 (as many as needed) needs --ulLinkAdaptation=1: it "
                    "chooses K by the SINR of the grant's resource blocks");
    NS_ABORT_MSG_IF(pdcchSymbols == 0 && !pdcch,
                    "--pdcchSymbols=0 (as many as each slot needs) needs --pdcch=1");
    NS_ABORT_MSG_UNLESS(scheduling == "dynamic" || scheduling == "sps1" || scheduling == "sps2",
                        "--scheduling must be dynamic, sps1 or sps2");
    NS_ABORT_MSG_IF(scheduling != "dynamic" && !pdcch,
                    "--scheduling=sps1 and sps2 need --pdcch=1: what they save is PDCCH");
    NS_ABORT_MSG_UNLESS(ueLayout == "random" || ueLayout == "line",
                        "--ueLayout must be random or line");
    NS_ABORT_MSG_IF(csvPackets && csv.empty(), "--csvPackets=1 needs --csv");
    if (!csv.empty())
    {
        // --csv: the folder of the files, and whether they can be written, before the run
        const std::filesystem::path csvDir = std::filesystem::path(csv).parent_path();
        if (!csvDir.empty())
        {
            SystemPath::MakeDirectories(csvDir.string());
        }
        std::ofstream f(csv + "_calls.csv");
        NS_ABORT_MSG_UNLESS(f, "--csv: cannot write " << csv << "_calls.csv");
    }
    NS_ABORT_MSG_UNLESS(scenario == "NTN-LOS" || scenario == "NTN-Rural" ||
                            scenario == "NTN-Suburban" || scenario == "NTN-Urban" ||
                            scenario == "NTN-DenseUrban",
                        "--scenario must be NTN-LOS, NTN-Rural, NTN-Suburban, NTN-Urban or "
                        "NTN-DenseUrban");
    // NTN-LOS: the rural environment with every UE in line of sight all the time
    const bool alwaysLos = (scenario == "NTN-LOS");
    const std::string environment = alwaysLos ? std::string("NTN-Rural") : scenario;
    NS_ABORT_MSG_UNLESS(shadowing == "fixed" || shadowing == "table",
                        "--shadowing must be fixed or table");
    NS_ABORT_MSG_UNLESS(atmosphere == "table" || atmosphere == "location",
                        "--atmosphere must be table or location");
    NS_ABORT_MSG_UNLESS(fastFading == "fixed" || fastFading == "model",
                        "--fastFading must be fixed or model");
    NS_ABORT_MSG_UNLESS(std::isfinite(shadowingDb) && std::isfinite(fastFadingDb) &&
                            std::isfinite(linkMarginDb),
                        "--shadowingDb, --fastFadingDb and --linkMarginDb must be numbers");
    // TR 38.811 6.6.6.1.4: the model holds within about 20 deg of the magnetic equator, the
    // scintillation can be neglected up to 60 deg, and above that the model does not apply
    NS_ABORT_MSG_UNLESS(atmosphere != "location" || std::abs(latitudeDeg) <= 60.0,
                        "--latitudeDeg must be -60 to 60: TR 38.811 has no scintillation model "
                        "for higher latitudes");
    const bool fixedShadowing = (shadowing == "fixed");
    const bool atmosphereTable = (atmosphere == "table");
    const bool equatorial = !atmosphereTable && std::abs(latitudeDeg) < 20.0;
    const bool fixedFastFading = (fastFading == "fixed");
    NS_ABORT_MSG_UNLESS(gatewayDiameterKm >= 0.0 && gatewayDiameterKm <= 10000.0,
                        "--gatewayDiameterKm must be 0 to 10000");
    NS_ABORT_MSG_UNLESS(feederFrequencyGHz >= 10.0 && feederFrequencyGHz <= 100.0,
                        "--feederFrequencyGHz must be 10 to 100");
    NS_ABORT_MSG_UNLESS(feederBandwidthMHz >= bandwidthMHz,
                        "--feederBandwidthMHz must hold one carrier: at least --bandwidthMHz");
    NS_ABORT_MSG_UNLESS(std::isfinite(gatewayGainDbi) && std::isfinite(gatewayDbwPerMHz) &&
                            std::isfinite(satFeederGtDbPerK),
                        "--gatewayGainDbi, --gatewayDbwPerMHz and --satFeederGtDbPerK "
                        "must be numbers");
    const bool randomLayout = (ueLayout == "random");
    if (fullPass)
    {
        // The whole pass over UE 0: the satellite flies, UE 0's call starts when the satellite
        // has risen to 10 deg and ends when it has sunk to 10 deg again. --freezeGeometry,
        // --elevation and --duration are not used; the duration is set below, from the orbit.
        freezeGeometry = false;
        elevation = 10.0;
    }
    NS_ABORT_MSG_UNLESS(elevation >= 10.0 && elevation <= 90.0,
                        "--elevation must be 10 to 90 (the 3GPP NTN channel stops at 10)");
    NS_ABORT_MSG_UNLESS(nUes >= 1, "--nUes must be at least 1");
    NS_ABORT_MSG_UNLESS(upfGwDelayMinMs >= 0.0 && upfGwDelayMaxMs >= upfGwDelayMinMs &&
                            hostGwDelayMs >= upfGwDelayMaxMs,
                        "The UPF is between gateway and remote host: 0 <= --upfGwDelayMinMs <= "
                        "--upfGwDelayMaxMs <= --hostGwDelayMs");
    NS_ABORT_MSG_UNLESS(harqProcesses >= 1 && harqProcesses <= 32,
                        "--harqProcesses must be 1 to 32 (16, or up to 32 in Rel-17 NTN)");
    NS_ABORT_MSG_UNLESS(targetBler >= 0.0 && targetBler < 1.0, "--targetBler must be 0 to 1");
    NS_ABORT_MSG_IF(srGrantBytes != 0 && srGrantBytes < 12,
                    "--srGrantBytes must be 0 or 12 or more");
    if (targetBler == 0.0)
    {
        // With HARQ a first transmission may fail fairly often, the retransmission repairs it;
        // without, a block that fails is a lost packet: the error rate of 5QI 1
        targetBler = harq ? 0.15 : 0.01;
    }
    // The directions the calls have: DOWN, the far end talks to the UE; UP, the UE talks
    const bool sendDown = (direction != "ul");
    const bool sendUp = (direction != "dl");
    const bool farUe = (farEnd == "ue");
    const std::string transportProtocol = "ns3::UdpSocketFactory";

    // One VAF per speaker, drawn from a Gaussian truncated to the generator's range [0, 0.99]
    // (values outside are drawn again). Change --RngRun to get a different set. The two speakers
    // of a call talk independently of each other; each direction has its own random stream, the
    // first one in use stream 500 (so a one-way call has the VAF it had before two-way calls).
    std::array<std::vector<double>, 2> vaf;
    for (const Dir dir : {DOWN, UP})
    {
        vaf[dir].assign(nUes, NAN);
        if (dir == DOWN ? !sendDown : !sendUp)
        {
            continue;
        }
        // (the first is made as it always was, which takes the next automatic stream before it
        // gets its own; the second is given its own at construction and takes none)
        const bool first = (dir == DOWN || !sendDown);
        Ptr<NormalRandomVariable> gauss =
            first ? CreateObject<NormalRandomVariable>()
                  : CreateObjectWithAttributes<NormalRandomVariable>("Stream", IntegerValue(501));
        gauss->SetAttribute("Mean", DoubleValue(vafMean));
        gauss->SetAttribute("Variance", DoubleValue(vafStd * vafStd));
        if (first)
        {
            gauss->SetStream(500);
        }
        for (uint32_t i = 0; i < nUes; ++i)
        {
            do
            {
                vaf[dir][i] = gauss->GetValue();
            } while (vaf[dir][i] < 0.0 || vaf[dir][i] > 0.99);
        }
    }

    // The remote host is hostGwDelayMs from the gateway. Where the UPF is on that way is drawn
    // for the run: its delay to the gateway is one value from a random stream of its own
    // (--RngRun gives another one, and it moves no other random number of the run), in steps of
    // 0.01 ms so that the run has exactly the value its header prints. That is the delay of the
    // S1-U link, in both directions; the link to the remote host has the rest.
    const double upfGwDelayMs =
        std::round(CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(304))
                       ->GetValue(upfGwDelayMinMs, upfGwDelayMaxMs) *
                   100.0) /
        100.0;
    const double remoteHostDelayMs = hostGwDelayMs - upfGwDelayMs; // remote host <-> UPF
    const Time upfGwDelay = MicroSeconds(std::llround(upfGwDelayMs * 1000.0));
    const Time remoteHostDelay = MicroSeconds(std::llround(remoteHostDelayMs * 1000.0));
    Config::SetDefault("ns3::NrPointToPointEpcHelper::S1uLinkDelay", TimeValue(upfGwDelay));
    // A downlink packet has used that much of its delay budget when it reaches the gNB: the
    // RLC of the gNB takes it off the budget, which is its discard timer
    Config::SetDefault("ns3::NrRlcUm::DelayBudgetSpentMs", DoubleValue(upfGwDelayMs));

    // ROHC in every PDCP entity of a data bearer, gNB and UE. Its other settings (profiles,
    // MAX_CID, repetitions, refreshes ...) are attributes of NrPdcp and can be given on the
    // command line as --ns3::NrPdcp::RohcIrRefresh=250 and so on.
    NS_ABORT_MSG_UNLESS(rohcMode == "U" || rohcMode == "O", "--rohcMode must be U or O");
    NS_ABORT_MSG_UNLESS(rohcModel == "table" || rohcModel == "library",
                        "--rohcModel must be table or library");
    NS_ABORT_MSG_UNLESS(l2Headers == "1+1+3" || l2Headers == "2+2+2",
                        "--l2Headers must be 1+1+3 or 2+2+2");
    // The size table and the short headers are in the nr module files that came with this
    // example: say so if the module does not have them
    {
        struct TypeId::AttributeInformation info;
        BooleanValue macValue;
        const bool hasTable =
            TypeId::LookupByName("ns3::NrPdcp").LookupAttributeByName("RohcModel", &info) &&
            TypeId::LookupByName("ns3::NrPdcp").LookupAttributeByName("RohcTableStaticWindow", &info);
        const bool hasShort =
            TypeId::LookupByName("ns3::NrPdcp").LookupAttributeByName("SnSize", &info) &&
            TypeId::LookupByName("ns3::NrRlcUm").LookupAttributeByName("SnFieldLength", &info) &&
            GlobalValue::GetValueByNameFailSafe("NrMacSubheaderAlways3Bytes", macValue);
        NS_ABORT_MSG_IF((rohc && rohcModel == "table" && !hasTable) ||
                            (l2Headers == "1+1+3" && !hasShort),
                        "This nr module has no ROHC size table (or an older one), or no short "
                        "layer-2 headers: unpack nr-ntnlab-module.tgz over contrib/nr and build "
                        "again, or run with --rohcModel=library --l2Headers=2+2+2");
    }
    NS_ABORT_MSG_IF(rohcIrPackets < 1 || rohcIrPackets > 255, "--rohcIrPackets must be 1 to 255");
    NS_ABORT_MSG_IF(rohcUpdatePackets < 1 || rohcUpdatePackets > 255,
                    "--rohcUpdatePackets must be 1 to 255");
    NS_ABORT_MSG_IF(rohcStaticWindow < rohcDynamicWindow,
                    "--rohcStaticWindow must not be smaller than --rohcDynamicWindow");
    NS_ABORT_MSG_IF(rohcNackAfter > 32 || rohcNackEvery < 1 || rohcNackEvery > 32,
                    "--rohcNackAfter must be 0 to 32 and --rohcNackEvery 1 to 32");
    if (rohcNackAfter == 0)
    {
        // The size table knows at once that a context is lost. The ROHC library sees CRC
        // failures, which can have other causes, and by default wants 10 of the last 32.
        rohcNackAfter = (rohcModel == "table") ? 1 : 10;
    }
    Config::SetDefault("ns3::NrPdcp::EnableRohc", BooleanValue(rohc));
    if (rohc)
    {
        Config::SetDefault("ns3::NrPdcp::RohcMode", StringValue(rohcMode));
        Config::SetDefault("ns3::NrPdcp::RohcNackFailures", UintegerValue(rohcNackAfter));
        Config::SetDefault("ns3::NrPdcp::RohcFeedbackSpacing", UintegerValue(rohcNackEvery));
    }
    if (rohc && rohcModel == "table")
    {
        Config::SetDefault("ns3::NrPdcp::RohcModel", StringValue("Table"));
        Config::SetDefault("ns3::NrPdcp::RohcTableIrPackets", UintegerValue(rohcIrPackets));
        Config::SetDefault("ns3::NrPdcp::RohcTableUpdatePackets", UintegerValue(rohcUpdatePackets));
        Config::SetDefault("ns3::NrPdcp::RohcTableDynamicWindow", UintegerValue(rohcDynamicWindow));
        Config::SetDefault("ns3::NrPdcp::RohcTableStaticWindow", UintegerValue(rohcStaticWindow));
    }
    if (l2Headers == "1+1+3")
    {
        Config::SetDefault("ns3::NrPdcp::SnSize", UintegerValue(7));
        Config::SetDefault("ns3::NrRlcUm::SnFieldLength", UintegerValue(5));
        GlobalValue::Bind("NrMacSubheaderAlways3Bytes", BooleanValue(true));
    }
    GlobalValue::Bind("NrRohcReport", StringValue(rohcReport));

    // VoIP settings for every NGMN generator, as in ntnlab_n.cc. With ngmn-voip-20ms.patch the
    // generator sends one packet per 20 ms frame while talking, whatever the payload; without it,
    // it keeps 12.2 kbit/s, so 12-byte packets leave every 12 x 8 / 12200 s = 7.87 ms. The run
    // measures which of the two it was. --rtp=1 has its own source instead, one packet per frame.
    // One voice frame: the bits the codec produces in 20 ms, in whole bytes (AMR 4.75: 95 bits,
    // 12 bytes), unless --voicePayload gives the bytes.
    NS_ABORT_MSG_UNLESS(codecKbps > 0.0, "--codecKbps must be positive");
    const uint32_t voiceFrameBits = static_cast<uint32_t>(std::ceil(codecKbps * 20.0 - 1e-9));
    const bool voiceFromCodec = (voicePayload == 0);
    if (voiceFromCodec)
    {
        voicePayload = (voiceFrameBits + 7) / 8;
    }
    NS_ABORT_MSG_UNLESS(voicePayload >= 1 && sidPayload >= 1,
                        "A voice frame and a SID frame must have at least 1 byte");
    NS_ABORT_MSG_UNLESS(sidIntervalMs >= 20 && sidIntervalMs % 20 == 0,
                        "--sidIntervalMs must be a multiple of 20, the frame of the codec");
    Config::SetDefault("ns3::TrafficGeneratorNgmnVoip::VoicePayload", UintegerValue(voicePayload));
    Config::SetDefault("ns3::TrafficGeneratorNgmnVoip::SIDPayload", UintegerValue(sidPayload));
    Config::SetDefault("ns3::TrafficGeneratorNgmnVoip::SIDPeriodicity", UintegerValue(sidIntervalMs));

    // --- The link budget ------------------------------------------------------------------------
    // The satellite comes from TR 38.821: Set-1 or Set-2, the LEO-600 or the LEO-1200 column,
    // whichever altitude is nearer. The defaults (Set-1, 600 km, 2 GHz, 30 MHz) are study case 9,
    // leo-link's default preset. A flag given on the command line replaces the table value.
    NS_ABORT_MSG_UNLESS(satSet == 1 || satSet == 2, "--satSet must be 1 or 2");
    NS_ABORT_MSG_UNLESS(altitudeKm >= 300.0 && altitudeKm <= 2000.0,
                        "--altitudeKm must be 300 to 2000: this example is set up for LEO");
    const SatelliteColumn& satColumn = g_satelliteSets[satSet - 1][altitudeKm < 900.0 ? 0 : 1];
    const bool satAsInTable = std::isnan(satEirpDbwPerMHz) && std::isnan(satAntennaGainDbi) &&
                              std::isnan(satGtDbPerK) && std::isnan(beamHpbwDeg) &&
                              std::isnan(satApertureM) && altitudeKm == satColumn.altitudeKm;
    satEirpDbwPerMHz = std::isnan(satEirpDbwPerMHz) ? satColumn.eirpDbwPerMHz : satEirpDbwPerMHz;
    satAntennaGainDbi = std::isnan(satAntennaGainDbi) ? satColumn.gainDbi : satAntennaGainDbi;
    satGtDbPerK = std::isnan(satGtDbPerK) ? satColumn.gtDbPerK : satGtDbPerK;
    const bool beamWidthGiven = !std::isnan(beamHpbwDeg);
    NS_ABORT_MSG_IF(beamWidthGiven && !std::isnan(satApertureM),
                    "--beamHpbwDeg fixes the beam and --satApertureM lets it follow the carrier: "
                    "give one of the two");
    NS_ABORT_MSG_IF(beamWidthGiven && !(beamHpbwDeg > 0.0 && beamHpbwDeg < 180.0),
                    "--beamHpbwDeg must be between 0 and 180");
    satApertureM = std::isnan(satApertureM) ? satColumn.apertureM : satApertureM;
    NS_ABORT_MSG_UNLESS(satApertureM > 0.0, "--satApertureM must be positive");
    const double altitudeM = altitudeKm * 1e3;
    const double dlFrequencyHz = dlFrequencyGHz * 1e9;
    // FDD: the uplink has its own carrier, 190 MHz below the downlink unless given
    const double ulFrequencyHz = ulFrequencyGHz > 0.0 ? ulFrequencyGHz * 1e9 : dlFrequencyHz - 190e6;
    const double bandwidthHz = bandwidthMHz * 1e6;
    // The resource blocks 3GPP gives the channel bandwidth (5 MHz: 25), not the nr module's 96 %
    // of it (26). The EIRP density is spread over them; the rest of the channel is guard band.
    const uint32_t carrierRbs = CarrierRbs(bandwidthMHz);
    NS_ABORT_MSG_UNLESS(carrierRbs > 0,
                        "--bandwidthMHz must be a channel bandwidth of TS 38.101-1 Table 5.3.2-1 "
                        "for 15 kHz: 5, 10, 15, 20, 25, 30, 35, 40, 45 or 50");
    const double occupiedHz = carrierRbs * 12 * 15e3;
    for (double carrierHz : {dlFrequencyHz, ulFrequencyHz})
    {
        NS_ABORT_MSG_UNLESS(carrierHz >= 0.6e9 && carrierHz < 6e9,
                            "Both carriers must be 0.6 to 6 GHz: the UE is the handheld of the "
                            "S-band study cases (Ka-band needs the VSAT terminal)");
    }
    NS_ABORT_MSG_UNLESS(std::abs(dlFrequencyHz - ulFrequencyHz) >= bandwidthHz,
                        "The downlink and uplink carriers overlap: FDD needs them at least "
                        "--bandwidthMHz apart");
    // The beam on each carrier. TR 38.821 gives the aperture and the 3 dB beamwidth of the set at
    // 2 GHz; ka = 2 pi f / c x aperture / 2 grows with carrier and aperture from there, so the
    // set's beamwidth is met exactly at 2 GHz. (The bare formula gives a 2 m aperture 4.4198 deg
    // at 2 GHz, the table says 4.4127 deg.) --beamHpbwDeg: that width on both carriers.
    g_scene.beamKa.clear();
    for (double carrierHz : {dlFrequencyHz, ulFrequencyHz})
    {
        g_scene.beamKa.push_back(beamWidthGiven
                                     ? BeamKaOfWidth(beamHpbwDeg)
                                     : BeamKaOfWidth(satColumn.hpbwDeg) *
                                           (satApertureM / satColumn.apertureM) *
                                           (carrierHz / g_satelliteSetCarrierHz));
        NS_ABORT_MSG_UNLESS(g_scene.beamKa.back() > g_halfPowerX,
                            "A " << satApertureM << " m aperture at " << carrierHz / 1e9
                                 << " GHz is too small to form a beam: its gain never falls "
                                    "by 3 dB");
    }
    const double satAntennaGainDb = satAntennaGainDbi; // the EIRP already contains it
    // G/T is the antenna gain over the system noise temperature T. The gNB PHY wants a noise
    // figure on 290 K instead: T / 290 K, in dB, to 0.01 dB as leo-link gives it (Set-1 and
    // Set-2 both make 776 K = 4.28 dB).
    const double satNoiseFigureDb =
        std::round((satAntennaGainDbi - satGtDbPerK - 10.0 * std::log10(290.0)) * 100.0) / 100.0;
    const double ueTxPowerDbm = 23.0;
    const double ueNoiseFigureDb = 7.0;
    const double satEirpDbm = satEirpDbwPerMHz + 30.0 + 10.0 * std::log10(occupiedHz / 1e6);
    const double noiseFloorDbm = -174.0 + 10.0 * std::log10(occupiedHz) + ueNoiseFigureDb;
    // The feeder link on the downlink carrier's resource blocks: the gateway's EIRP there, and
    // the noise of the satellite's feeder receiver, which it relays with the signal. Only their
    // ratio counts, so the receiver is taken as a gain of G/T over a noise of k x 1 K x B.
    const double feederFrequencyHz = feederFrequencyGHz * 1e9;
    const double gatewayEirpDbwPerMHz = gatewayDbwPerMHz + gatewayGainDbi;
    const double gatewayEirpDbm = gatewayEirpDbwPerMHz + 30.0 + 10.0 * std::log10(occupiedHz / 1e6);
    const double boltzmannDbwPerKHz = -228.6;
    const double relayNoiseFloorDbm = boltzmannDbwPerKHz + 30.0 + 10.0 * std::log10(occupiedHz);
    // The distance from the ground to the satellite at an elevation (deg), TR 38.811 (6.6-3)
    auto slantM = [&](double elevationDeg) {
        const double e = elevationDeg * M_PI / 180.0;
        const double r = GeographicPositions::EARTH_SPHERE_RADIUS;
        return std::sqrt(std::pow(r * std::sin(e), 2) + altitudeM * altitudeM +
                         2.0 * altitudeM * r) -
               r * std::sin(e);
    };
    // The feeder link's C/N with the satellite at an elevation (deg) above the gateway
    auto feederCnrDb = [&](double elevationDeg) {
        return gatewayEirpDbwPerMHz -
               FeederLinkLossModel::FreeSpaceDb(feederFrequencyHz, slantM(elevationDeg)) +
               satFeederGtDbPerK - boltzmannDbwPerKHz - 60.0;
    };

    Config::SetDefault("ns3::NrGnbPhy::PowerAllocationType", StringValue("UniformPowerAllocBw"));
    // ns-3's 3GPP channel (--fastFading=model). With a period that is not zero it draws a new
    // realisation whenever an end of the link has moved more than 1 m since the last draw (a
    // flying satellite: at every look), and evolves it by TR 38.901 Procedure A once the period
    // is over if it has moved less (never here). Zero: one realisation per line-of-sight state.
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(1000)));
    Config::SetDefault("ns3::NrMacSchedulerHarqRr::ConsolidateHarqRetx", BooleanValue(true));
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(10 * 1024 * 1024));
    // The RLC receiver holds a packet back while one before it is missing, for as long as a
    // HARQ retransmission can still bring that one: 100 ms cover the three there can be. Without
    // HARQ nothing overtakes anything and a missing packet is lost: nothing is held back.
    Config::SetDefault("ns3::NrRlcUm::ReorderingTimer", TimeValue(MilliSeconds(harq ? 100 : 0)));
    // Control channels and radio link failure. The thresholds and timers are attributes of the nr
    // module (--ns3::NrUePhy::Qout=..., --ns3::NrUeRrc::T310=... change them); --idealControl=0
    // turns on the three things the header comment describes.
    const double qOutDb = AttributeDefault<DoubleValue>("ns3::NrUePhy", "Qout");
    const double qInDb = AttributeDefault<DoubleValue>("ns3::NrUePhy", "Qin");
    const uint64_t qOutEvalSf = AttributeDefault<UintegerValue>("ns3::NrUePhy", "NumQoutEvalSf");
    const uint64_t n310 = AttributeDefault<UintegerValue>("ns3::NrUeRrc", "N310");
    const Time t310 = AttributeDefault<TimeValue>("ns3::NrUeRrc", "T310");
    if (!idealControl)
    {
        Config::SetDefault("ns3::NrUePhy::PdcchErrorModel", BooleanValue(true));
    }
    // The PDCCH as a resource. Everything else of it is an attribute of NrPdcch
    // (--ns3::NrPdcch::Candidates=..., CoresetRbs, TargetBler, SinrMarginDb, UlMaxShare, ...).
    if (pdcch)
    {
        Config::SetDefault("ns3::NrPdcch::Enabled", BooleanValue(true));
        Config::SetDefault("ns3::NrPdcch::Symbols", UintegerValue(pdcchSymbols));
        Config::SetDefault("ns3::NrPdcch::DciBits", UintegerValue(dciBits));
        if (!pdcchBlerTable.empty())
        {
            Config::SetDefault("ns3::NrPdcch::BlerTable", StringValue(pdcchBlerTable));
        }
    }
    // Semi-persistent scheduling of the voice bearer: SPS in downlink, configured grant Type 2
    // in uplink (NrMacSchedulerNs3::SpsScenario). The block of a configuration holds one packet
    // in the steady state of ROHC: the frame, the compressed header, the PDCP and RLC headers
    // and the 3 B the scheduler keeps for a MAC subheader. A size given on the command line
    // (--ns3::NrMacSchedulerNs3::SpsTbBytes=..., SpsSilenceTbBytes, SpsFitExtraBytes) stays.
    const uint32_t spsScenario = scheduling == "sps1" ? 1 : (scheduling == "sps2" ? 2 : 0);
    uint32_t spsTbBytes = 0;
    uint32_t spsSilenceTbBytes = 0;
    uint32_t spsFitExtraBytes = 0;
    if (spsScenario != 0)
    {
        NS_ABORT_MSG_UNLESS(sidPayload < voicePayload,
                            "--scheduling=sps1 and sps2 tell talk from silence by the size of the "
                            "packets: --sidPayload must be smaller than the voice frame");
        const uint32_t ipHeaders = rtp ? 40 : 28;
        const uint32_t uo0 = AttributeDefault<UintegerValue>("ns3::NrPdcp", "RohcTableUo0Size");
        const uint32_t update = AttributeDefault<UintegerValue>("ns3::NrPdcp", "RohcTableUpdateSize");
        const uint32_t header = rohc ? uo0 : ipHeaders;
        const uint32_t below = (l2Headers == "1+1+3") ? 1 + 1 + 3 : 2 + 2 + 3;
        const char* sched = "ns3::NrMacSchedulerNs3";
        spsTbBytes = AttributeDefault<UintegerValue>(sched, "SpsTbBytes");
        spsSilenceTbBytes = AttributeDefault<UintegerValue>(sched, "SpsSilenceTbBytes");
        spsFitExtraBytes = AttributeDefault<UintegerValue>(sched, "SpsFitExtraBytes");
        spsTbBytes = spsTbBytes ? spsTbBytes : voicePayload + header + below;
        spsSilenceTbBytes = spsSilenceTbBytes ? spsSilenceTbBytes : sidPayload + header + below;
        spsFitExtraBytes = spsFitExtraBytes ? spsFitExtraBytes : (rohc ? update - uo0 : 0);
        Config::SetDefault("ns3::NrMacSchedulerNs3::SpsScenario", UintegerValue(spsScenario));
        Config::SetDefault("ns3::NrMacSchedulerNs3::SpsFiveQi", UintegerValue(1));
        Config::SetDefault("ns3::NrMacSchedulerNs3::SpsPeriodMs", UintegerValue(20));
        Config::SetDefault("ns3::NrMacSchedulerNs3::SpsSilencePeriodMs",
                           UintegerValue(sidIntervalMs));
        Config::SetDefault("ns3::NrMacSchedulerNs3::SpsTbBytes", UintegerValue(spsTbBytes));
        Config::SetDefault("ns3::NrMacSchedulerNs3::SpsSilenceTbBytes",
                           UintegerValue(spsSilenceTbBytes));
        Config::SetDefault("ns3::NrMacSchedulerNs3::SpsFitExtraBytes",
                           UintegerValue(spsFitExtraBytes));
        // The gNB tells a silence descriptor from a voice frame by the size of the IP packet
        Config::SetDefault("ns3::NrPdcp::SilenceSduMaxBytes", UintegerValue(ipHeaders + sidPayload));
    }
    // Ideal random access, as leo-link's default: no RA timeout, 2 s to finish the connection
    Config::SetDefault("ns3::NrGnbRrc::ConnectionRequestTimeoutDuration",
                       TimeValue(MilliSeconds(2000)));
    // The UEs attach one after another over the first 0.5 s. All at once, the gNB answers only
    // about a dozen of them and the rest never connect (ideal random access does not retry).
    Config::SetDefault("ns3::NrHelper::AttachWindow", TimeValue(MilliSeconds(500)));
    // A UE whose scheduling requests go unanswered asks again and again. After sr-TransMax of them
    // TS 38.321 5.4.4 has it start a random access, in which a connected UE keeps its C-RNTI
    // (5.1.5: Msg3 carries it); this nr module gives it a new RNTI instead, and its downlink stops
    // with the old one. So the fallback is left out, unless the command line sets SrTransMax.
    const bool srFallback = AttributeDefault<UintegerValue>("ns3::NrUeMac", "SrTransMax") != 64;
    if (!srFallback)
    {
        Config::SetDefault("ns3::NrUeMac::SrTransMax", UintegerValue(1000000000));
    }

    // --- Where everything is -----------------------------------------------------------
    // The beam centre is on the equator at longitude 0, the UEs around it (see below). The
    // satellite comes from the north on a circular orbit, heading south, towards the beam centre.
    // Standing still, it is on the centre's meridian at the central angle that puts it at
    // --elevation. Flying, it passes exactly over the centre, and it is at --elevation at
    // passStart, when UE 0's call starts unless it has to wait for the gateway (see below).
    const Time passStart = Seconds(1);
    Time overheadAt; // flying only: when the satellite is over the beam centre ...
    Time downAt;     // ... and when it is back down at 10 deg on the other side
    const double earthRadiusM = GeographicPositions::EARTH_SPHERE_RADIUS;
    const double orbitRadiusM = earthRadiusM + altitudeM;
    const double elevationRad = elevation * M_PI / 180.0;
    const double centralAngleRad =
        std::acos(earthRadiusM / orbitRadiusM * std::cos(elevationRad)) - elevationRad;
    const double satLatDeg = centralAngleRad * 180.0 / M_PI;
    const double orbitSpeedMps = std::sqrt(3.986004418e14 / orbitRadiusM);

    Ptr<Node> satellite = CreateObject<Node>();
    Ptr<GeocentricConstantPositionMobilityModel> satMobility;
    OrbitPredictor orbit;
    if (freezeGeometry)
    {
        Ptr<FixedMobilityModel> frozen = CreateObject<FixedMobilityModel>();
        frozen->SetGeographicPosition(Vector(satLatDeg, 0.0, altitudeM));
        frozen->SetReportedVelocity(Vector(orbitSpeedMps * std::sin(centralAngleRad),
                                           0.0,
                                           -orbitSpeedMps * std::cos(centralAngleRad)));
        satMobility = frozen;
        orbit.pos0 = frozen->GetPosition();
    }
    else
    {
        // A polar orbit that passes exactly over the beam centre. The Earth turns under the orbit,
        // so s seconds before it is overhead the satellite is omega x s of latitude to the north and
        // omegaEarth x s of longitude to the east of the centre, which is a central angle c with
        // cos c = cos(omega s) cos(omegaEarth s). An elevation asks for a central angle, and so for s.
        Ptr<OrbitingMobilityModel> orbiting = CreateObject<OrbitingMobilityModel>();
        orbiting->SetAttribute("Altitude", DoubleValue(altitudeM / 1000.0)); // km
        orbiting->SetAttribute("Inclination", DoubleValue(90.0));
        orbiting->SetAttribute("Resolution", TimeValue(Seconds(1))); // CourseChange period, as leo-link
        const double omega = orbiting->GetAngularVelocity();
        const double omegaEarth = 2.0 * M_PI / Days(1).GetSeconds(); // as the orbit model turns the Earth
        auto secondsFromOverhead = [&](double elevationDeg) {
            const double e = elevationDeg * M_PI / 180.0;
            const double cosCentral = std::cos(std::acos(earthRadiusM / orbitRadiusM * std::cos(e)) - e);
            double low = 0.0;
            double high = std::acos(cosCentral) / omega;
            for (int i = 0; i < 100; ++i)
            {
                const double mid = 0.5 * (low + high);
                if (std::cos(omega * mid) * std::cos(omegaEarth * mid) > cosCentral)
                {
                    low = mid; // still closer than asked
                }
                else
                {
                    high = mid;
                }
            }
            return 0.5 * (low + high);
        };
        overheadAt = passStart + Seconds(secondsFromOverhead(elevation));
        downAt = overheadAt + Seconds(secondsFromOverhead(10.0));
        if (fullPass)
        {
            duration = downAt - passStart; // UE 0's call: from 10 deg to 10 deg
        }
        // The model takes the longitude of the orbit plane and the angle along the orbit from its
        // northernmost point, here the North Pole; the satellite is 90 deg east of the plane's
        // longitude. At t = 0 it is overheadAt seconds before the beam centre.
        const double toOverhead = overheadAt.GetSeconds();
        orbiting->SetPosition(Vector(omegaEarth * toOverhead * 180.0 / M_PI - 90.0,
                                     90.0 - omega * toOverhead * 180.0 / M_PI,
                                     0.0));
        satMobility = orbiting;
        orbit.pos0 = orbiting->GetPosition();
        orbit.vel0 = orbiting->GetVelocity();
        orbit.omegaRadPerS = orbiting->GetAngularVelocity();
        NS_ABORT_MSG_UNLESS(orbit.vel0.z < 0.0 && orbit.omegaRadPerS > 0.0,
                            "the orbit model does not fly the satellite south");
    }
    satellite->AggregateObject(satMobility);

    // --- The UEs ---------------------------------------------------------------------------
    // UE 0 is on the beam centre in both layouts: the reference user.
    // --ueLayout=line: UE i is i x spacingKm east of the beam centre.
    // --ueLayout=random: every other UE is somewhere in the beam area, every place of the area as
    // likely as any other and the UEs independent of each other. That is a uniform drop: what a
    // Poisson point process gives once its number of points is known. The beam area is the
    // ground that is inside the 3 dB contour of the downlink beam for as long as the calls last,
    // and from where the satellite is 5 deg or more above the horizon all that time (why 5: see
    // minPlaceElevationDeg). With the satellite standing still that is the 3 dB footprint of the beam.
    // With the satellite flying the footprint changes its size and shape, and the area is what
    // the footprints of all moments of the calls have in common.
    Ptr<FixedMobilityModel> centreMobility = CreateObject<FixedMobilityModel>();
    centreMobility->SetGeographicPosition(Vector(0.0, 0.0, 0.0));
    const Vector centrePosition = centreMobility->GetPosition();
    // UE 0's call lasts --duration from passStart (--fullPass=1: the whole pass, the duration
    // was set above), as long as the gateway lets it (see below). No call starts before it and
    // every call is over when it ends.
    const Time passEnd = passStart + duration;

    // A place on the ground, `arc` radians of great circle from the beam centre in the direction
    // `bearing` (0 = north, pi/2 = east)
    auto groundPoint = [&](double arc, double bearing) {
        return Vector(earthRadiusM * std::cos(arc),
                      earthRadiusM * std::sin(arc) * std::sin(bearing),
                      earthRadiusM * std::sin(arc) * std::cos(bearing));
    };
    // Where the satellite is from `from` to `to`: one place, or one per second of the pass with
    // the first and the last moment and the moment it is overhead
    auto satBetween = [&](Time from, Time to) {
        std::vector<Vector> sats;
        if (freezeGeometry)
        {
            sats.push_back(orbit.pos0);
            return sats;
        }
        for (Time t = from; t < to; t += Seconds(1))
        {
            sats.push_back(orbit.At(t));
        }
        sats.push_back(orbit.At(to));
        if (overheadAt > from && overheadAt < to)
        {
            sats.push_back(orbit.At(overheadAt));
        }
        return sats;
    };
    // UE 0's call has to fit into the pass: UE 0 must see the satellite at 10 deg or more for as
    // long as it lasts. Said here, before any place is drawn: after the pass no place is left.
    {
        const std::vector<Vector> satDuringPass = satBetween(passStart, passEnd);
        NS_ABORT_MSG_IF(!freezeGeometry &&
                            std::any_of(satDuringPass.begin(),
                                        satDuringPass.end(),
                                        [&](const Vector& sat) {
                                            return ElevationDeg(sat, centrePosition) < 9.99;
                                        }),
                        "The satellite is down at 10 deg elevation again at t = "
                            << downAt.GetSeconds() << " s, before UE 0's call ends. Use --duration="
                            << std::floor((downAt - passStart).GetSeconds())
                            << "s or less, or --fullPass=1 for the whole pass.");
    }

    // A place on the ground, drawn uniformly over the cap of the sphere within `capArc` radians
    // of the beam centre (cos(arc) is uniform over a cap) and drawn again until `fits` says yes
    struct Place
    {
        Vector ecef;
        double northKm{0.0}; // seen from the beam centre
        double eastKm{0.0};
    };

    auto drawPlace = [&](Ptr<UniformRandomVariable> rng,
                         double capArc,
                         const std::function<bool(const Vector&)>& fits,
                         const std::string& what) {
        Place place;
        double arc = 0.0;
        double bearing = 0.0;
        uint32_t tries = 0;
        do
        {
            NS_ABORT_MSG_IF(++tries > 1000000, "no place found for " << what);
            arc = std::acos(1.0 - rng->GetValue() * (1.0 - std::cos(capArc)));
            bearing = 2.0 * M_PI * rng->GetValue();
            place.ecef = groundPoint(arc, bearing);
        } while (!fits(place.ecef));
        place.northKm = earthRadiusM * 1e-3 * arc * std::cos(bearing);
        place.eastKm = earthRadiusM * 1e-3 * arc * std::sin(bearing);
        return place;
    };
    auto putThere =
        [&](Ptr<FixedMobilityModel> mobility, const Place& place, const std::string& what) {
            mobility->SetGeographicPosition(
                Vector(std::asin(place.ecef.z / earthRadiusM) * 180.0 / M_PI,
                       std::atan2(place.ecef.y, place.ecef.x) * 180.0 / M_PI,
                       0.0));
            NS_ABORT_MSG_IF((mobility->GetPosition() - place.ecef).GetLength() > 0.01,
                            what << " is not where it was drawn");
        };

    // When the satellite flies: the time from `from` to `to` that it is `minDeg` or more above a
    // place. Seen from a place on the ground the satellite rises once and sinks once during a
    // pass, so that is one stretch. Its two ends are looked for in steps of 100 ms and then by
    // halving.
    struct Stretch
    {
        bool any{false}; // the satellite is high enough at some moment
        Time rise;       // from (`from` if it is high enough then) ...
        Time set;        // ... to (`to` if it still is then)
    };

    auto stretchAbove =
        [&](const Vector& place, double minDeg, Time from, Time to, const std::string& what) {
            auto highEnough = [&](Time t) { return ElevationDeg(orbit.At(t), place) >= minDeg; };
            std::vector<Time> steps;
            for (Time t = from; t < to; t += MilliSeconds(100))
            {
                steps.push_back(t);
            }
            steps.push_back(to);
            std::size_t first = steps.size(); // the first and the last step with the satellite
            std::size_t last = 0;             // high enough, and how many there are
            std::size_t count = 0;
            for (std::size_t k = 0; k < steps.size(); ++k)
            {
                if (highEnough(steps[k]))
                {
                    first = std::min(first, k);
                    last = k;
                    ++count;
                }
            }
            Stretch stretch;
            if (count == 0)
            {
                return stretch;
            }
            NS_ABORT_MSG_UNLESS(count == last - first + 1,
                                "the satellite is at " << minDeg << " deg above " << what
                                                       << " more than twice");
            // The moment between a step at which the satellite is too low and one at which it is
            // high enough: the last of the two, after halving the time between them 40 times
            auto crossing = [&](Time tooLow, Time high) {
                for (int step = 0; step < 40; ++step)
                {
                    const Time mid = tooLow + (high - tooLow) / 2;
                    (highEnough(mid) ? high : tooLow) = mid;
                }
                return high;
            };
            stretch.any = true;
            stretch.rise = first == 0 ? from : crossing(steps[first - 1], steps[first]);
            stretch.set = last + 1 == steps.size() ? to : crossing(steps[last + 1], steps[last]);
            return stretch;
        };

    // The gateway (transparent payload: where the gNB is). It carries no device; the channel
    // only needs its position, for the delay and the loss of the feeder link.
    // --gatewayDiameterKm=0: where UE 0 is, as in leo-link; the feeder link then has the same
    // length as UE 0's service link.
    // --gatewayDiameterKm=D: a random place of the circle of diameter D around UE 0, every place
    // of the circle as likely as any other. It has nothing to do with where the UEs are: its
    // place comes from a random stream of its own. The feeder link needs the satellite at 10 deg
    // or more above the gateway (TR 38.821 Table 4.2-2, as for the UEs), and no call is on while
    // it is lower. Left out are the places from where it never is during UE 0's call (with the
    // satellite standing still: the places from where it is below 10 deg).
    const bool gatewayAway = transparent && gatewayDiameterKm > 0.0;
    const double minFeederElevationDeg = 10.0;
    Ptr<Node> gateway = CreateObject<Node>();
    Ptr<FixedMobilityModel> gatewayMobility = CreateObject<FixedMobilityModel>();
    gatewayMobility->SetGeographicPosition(Vector(0.0, 0.0, 0.0));
    Place gatewayPlace;
    gatewayPlace.ecef = centrePosition;
    // Flying: when the gateway sees the satellite at 10 deg or more, from passStart until the
    // satellite is down at 10 deg above UE 0 (the call may not use all of that)
    Stretch gatewaySees;
    gatewaySees.any = true;
    gatewaySees.rise = passStart;
    gatewaySees.set = freezeGeometry ? passEnd : downAt;
    if (gatewayAway)
    {
        auto seesSatellite = [&](const Vector& place) {
            if (freezeGeometry)
            {
                return ElevationDeg(orbit.pos0, place) >= minFeederElevationDeg;
            }
            const Stretch sees =
                stretchAbove(place, minFeederElevationDeg, passStart, downAt, "the gateway");
            return sees.any && sees.rise < passEnd;
        };
        Ptr<UniformRandomVariable> gatewayDrop =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(301));
        gatewayPlace = drawPlace(gatewayDrop,
                                 gatewayDiameterKm * 500.0 / earthRadiusM, // the radius, in radians
                                 seesSatellite,
                                 "the gateway");
        putThere(gatewayMobility, gatewayPlace, "the gateway");
        if (!freezeGeometry)
        {
            gatewaySees = stretchAbove(gatewayPlace.ecef,
                                       minFeederElevationDeg,
                                       passStart,
                                       downAt,
                                       "the gateway");
        }
    }
    gateway->AggregateObject(gatewayMobility);
    // The calls: from passStart, or from when the gateway sees the satellite at 10 deg if that
    // is later, to the end of UE 0's call, or to when the gateway sees it sink below 10 deg if
    // that is earlier
    const Time callsStart = Max(passStart, gatewaySees.rise);
    const Time callsEnd = Min(passEnd, gatewaySees.set);
    NS_ABORT_MSG_IF(MilliSeconds((nUes - 1) * startOffsetMs) >= callsEnd - callsStart,
                    "Call " << nUes - 1
                            << " would start when UE 0's call is over: (nUes - 1) x "
                               "--startOffsetMs must be less than UE 0's call lasts ("
                            << (callsEnd - callsStart).GetSeconds() << " s)");
    // Where the satellite is while the calls are on
    const std::vector<Vector> satDuringCalls = satBetween(callsStart, callsEnd);
    // The lowest elevation at which a UE that was dropped at random may see the satellite during
    // the calls. UE 0 keeps 10 deg, where the 3GPP NTN tables start; the tables
    // have one row per 10 deg and the models take the 10 deg row for everything below it, so
    // 5 deg is half a row further down. With 10 deg here, a pass that uses all the time UE 0 has
    // (from 10 deg up and down to 10 deg again) would leave only the places exactly east and
    // west of UE 0: every other place is below 10 deg at the start or at the end.
    const double minPlaceElevationDeg = 5.0;
    // cos(HPBW / 2) of the downlink beam: the beam pattern is at -3 dB where ka sin(theta) = 1.616339
    const double cosEdge = std::sqrt(1.0 - std::pow(g_halfPowerX / g_scene.beamKa[0], 2));
    auto inBeamArea = [&](const Vector& place) {
        for (const Vector& sat : satDuringCalls)
        {
            const Vector toCentre = centrePosition - sat;
            const Vector toPlace = place - sat;
            if (toCentre * toPlace < cosEdge * toCentre.GetLength() * toPlace.GetLength() ||
                ElevationDeg(sat, place) < minPlaceElevationDeg)
            {
                return false;
            }
        }
        return true;
    };
    // How far the beam area reaches from the centre in 720 directions. The area is convex and the
    // centre is in it, so each direction has one edge. Twice the arc from the point below the
    // satellite to its horizon is outside for sure.
    const uint32_t areaDirections = 720;
    std::vector<double> areaEdgeArc(areaDirections);
    double areaMaxArc = 0.0;
    double beamAreaKm2 = 0.0;
    for (uint32_t k = 0; k < areaDirections; ++k)
    {
        const double bearing = 2.0 * M_PI * k / areaDirections;
        double inside = 0.0;
        double outside = 2.0 * std::acos(earthRadiusM / orbitRadiusM);
        for (int step = 0; step < 40; ++step)
        {
            const double mid = 0.5 * (inside + outside);
            (inBeamArea(groundPoint(mid, bearing)) ? inside : outside) = mid;
        }
        areaEdgeArc[k] = inside;
        areaMaxArc = std::max(areaMaxArc, inside);
        // the area of a sector of a sphere: R^2 (1 - cos(arc)) per radian of direction
        beamAreaKm2 += earthRadiusM * earthRadiusM * 1e-6 * (1.0 - std::cos(inside)) * 2.0 * M_PI /
                       areaDirections;
    }
    const double areaNorthSouthKm =
        earthRadiusM * 1e-3 * (areaEdgeArc[0] + areaEdgeArc[areaDirections / 2]);
    const double areaEastWestKm =
        earthRadiusM * 1e-3 * (areaEdgeArc[areaDirections / 4] + areaEdgeArc[3 * areaDirections / 4]);

    NodeContainer ueNodes;
    ueNodes.Create(nUes);
    std::vector<Ptr<FixedMobilityModel>> ueMobility(nUes);
    std::vector<double> ueNorthKm(nUes, 0.0); // where UE i is, seen from the beam centre
    std::vector<double> ueEastKm(nUes, 0.0);
    const double kmPerDegree = earthRadiusM * M_PI / 180.0 / 1000.0;
    // The places are drawn from a stream of their own: --RngRun gives another drop, and the drop
    // does not move any other random number of the run. (The stream is given at construction: a
    // random variable made without one takes the next automatic stream, and every later one of
    // the run, the channel's among them, would move up by one.)
    Ptr<UniformRandomVariable> drop =
        CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(300));
    for (uint32_t i = 0; i < nUes; ++i)
    {
        ueMobility[i] = CreateObject<FixedMobilityModel>();
        if (!randomLayout || i == 0)
        {
            // in a line, or UE 0: the reference user on the beam centre (i = 0)
            const double eastKm = randomLayout ? 0.0 : i * spacingKm;
            ueMobility[i]->SetGeographicPosition(Vector(0.0, eastKm / kmPerDegree, 0.0));
            ueEastKm[i] = eastKm;
        }
        else
        {
            // Over the cap that holds the beam area, until the place is in the beam area
            const std::string what = "UE " + std::to_string(i);
            const Place place = drawPlace(drop, 1.01 * areaMaxArc, inBeamArea, what);
            putThere(ueMobility[i], place, what);
            ueNorthKm[i] = place.northKm;
            ueEastKm[i] = place.eastKm;
        }
        ueNodes.Get(i)->AggregateObject(ueMobility[i]);
    }

    // When each call starts and ends. UE 0's call runs from callsStart to callsEnd; call i
    // starts i x startOffsetMs after it and ends with it.
    // When the satellite flies, a call is also on only while the satellite is 10 deg or more
    // above its own UE: it starts when the satellite has risen to 10 deg above that UE, if that
    // is later, and ends when the satellite has sunk to 10 deg above it again, if that is
    // earlier (stretchAbove() over UE 0's call). A UE whose stretch does not reach into its call
    // has no call. (UE 0 itself: the pass was laid out for its call.)
    // When the satellite stands still nothing rises or sinks, and the rule is not applied.
    const double minCallElevationDeg = 10.0;
    std::vector<Time> ueCallStart(nUes);
    std::vector<Time> ueCallEnd(nUes);
    std::vector<bool> ueHasCall(nUes, true);
    for (uint32_t i = 0; i < nUes; ++i)
    {
        ueCallStart[i] = callsStart + MilliSeconds(i * startOffsetMs); // not phase-aligned
        ueCallEnd[i] = callsEnd;
        if (freezeGeometry || i == 0)
        {
            continue;
        }
        const Stretch sees = stretchAbove(ueMobility[i]->GetPosition(),
                                          minCallElevationDeg,
                                          callsStart,
                                          callsEnd,
                                          "UE " + std::to_string(i));
        if (!sees.any)
        {
            ueHasCall[i] = false;
            continue;
        }
        ueCallStart[i] = Max(ueCallStart[i], sees.rise);
        ueCallEnd[i] = Min(ueCallEnd[i], sees.set);
        ueHasCall[i] = ueCallStart[i] < ueCallEnd[i];
    }

    // --- Channels, delay and NTN timing ---------------------------------------------------
    Ptr<NrNtnPropagationDelayModel> ntnDelay = CreateObject<NrNtnPropagationDelayModel>();
    if (transparent)
    {
        ntnDelay->SetFeeder(gatewayMobility);
    }
    // Every UE gets its obstruction level and its shadowing level, each from a random stream of
    // its own, so that they depend on nothing else in the run. Both carriers use them. (The
    // tables are those of the S-band, which is where the carriers are.)
    Ptr<FixedLevelChannelConditionModel> levels = CreateObject<FixedLevelChannelConditionModel>();
    levels->SetTable(g_largeScaleTables.at(environment));
    if (fixedShadowing)
    {
        levels->SetFixedShadowing(shadowingDb);
    }
    {
        Ptr<UniformRandomVariable> obstruction =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(302));
        Ptr<NormalRandomVariable> shadowingLevel =
            CreateObjectWithAttributes<NormalRandomVariable>("Stream", IntegerValue(303));
        for (uint32_t i = 0; i < nUes; ++i)
        {
            // (NTN-LOS: drawn all the same, so that the shadowing levels are those of the same
            // drop with NTN-Rural)
            const double obstructionLevel = obstruction->GetValue();
            levels->AddUe(ueMobility[i],
                          alwaysLos ? 0.0 : obstructionLevel,
                          shadowingLevel->GetValue());
        }
    }
    g_scene.levels = levels;
    // The feeder link: free space at its own carrier, the same for both (only the downlink uses it)
    Ptr<FeederLinkLossModel> feederLoss = CreateObject<FeederLinkLossModel>();
    feederLoss->SetFrequency(feederFrequencyHz);
    NtnCarrier dlCarrier = CreateNtnCarrier(dlFrequencyHz,
                                            environment,
                                            gatewayMobility,
                                            feederLoss,
                                            ntnDelay,
                                            transparent,
                                            levels,
                                            atmosphereTable,
                                            equatorial,
                                            linkMarginDb,
                                            fixedFastFading,
                                            fastFadingDb);
    NtnCarrier ulCarrier = CreateNtnCarrier(ulFrequencyHz,
                                            environment,
                                            gatewayMobility,
                                            feederLoss,
                                            ntnDelay,
                                            transparent,
                                            levels,
                                            atmosphereTable,
                                            equatorial,
                                            linkMarginDb,
                                            fixedFastFading,
                                            fastFadingDb);
    // (GatewayTxPowerDbm is what reaches the satellite before the path loss: the gateway's EIRP)
    for (const auto& carrier : {dlCarrier, ulCarrier})
    {
        carrier.feeder->SetAttribute("GatewayTxPowerDbm", DoubleValue(gatewayEirpDbm));
        carrier.feeder->SetAttribute("SatRxAntennaGainDb", DoubleValue(satFeederGtDbPerK));
        carrier.feeder->SetAttribute("RelayEirpDbm", DoubleValue(satEirpDbm));
        carrier.feeder->SetAttribute("RelayLossDb", DoubleValue(0.0));
        carrier.feeder->SetAttribute("RelayInputNoisePowerDbm", DoubleValue(relayNoiseFloorDbm));
        carrier.feeder->SetAttribute("ReceiverNoisePowerDbm", DoubleValue(noiseFloorDbm));
    }

    // What the run will see of the satellite, every 100 ms from t = 0 to the end (one step when
    // it stands still): the longest round trip, which K_offset has to cover, and the elevation at
    // the beam centre, which must be 10 deg or more while the calls are on (the 3GPP NTN channel
    // is defined from 10 deg up; in the second before the calls and the two after them it may be
    // a little lower, and the channel model then takes its 10 deg values).
    const Time simEnd = callsEnd + Seconds(2);
    if (!freezeGeometry)
    {
        // The pass is what was asked for: --elevation at passStart, the centre right below later
        NS_ABORT_MSG_UNLESS(
            std::abs(ElevationDeg(orbit.At(passStart), centrePosition) - elevation) < 1e-6 &&
                std::abs((orbit.At(overheadAt) - centrePosition).GetLength() - altitudeM) < 1.0,
            "the orbit does not pass over the beam centre as planned");
    }
    Ptr<ConstantPositionMobilityModel> satProbe = CreateObject<ConstantPositionMobilityModel>();
    Time maxLoopRtt;
    Time maxLoopRttAt;
    for (Time t = Seconds(0); t <= simEnd; t += MilliSeconds(100))
    {
        const Vector sat = orbit.At(t);
        satProbe->SetPosition(sat);
        for (uint32_t i = 0; i < nUes; ++i)
        {
            const Time loopRtt = 2 * ntnDelay->GetOneWayDelay(satProbe, ueMobility[i]);
            if (loopRtt > maxLoopRtt)
            {
                maxLoopRtt = loopRtt;
                maxLoopRttAt = t;
            }
        }
        NS_ABORT_MSG_IF(
            t >= callsStart && t <= callsEnd && ElevationDeg(sat, centrePosition) < 9.99,
            "The satellite is down at 10 deg elevation again at t = "
                << downAt.GetSeconds() << " s, before UE 0's call ends. Use "
                << "--duration=" << std::floor((downAt - passStart).GetSeconds()) << "s or less.");
        if (freezeGeometry)
        {
            break;
        }
    }
    // K_offset covers the longest round trip in the cell, in 1 ms slots (TS 38.213 4.2). It is
    // cell-specific and does not change during the run.
    const uint32_t kOffsetSlots = static_cast<uint32_t>(
        std::clamp<double>(std::ceil(maxLoopRtt.GetSeconds() * 1000.0), 1.0, 1023.0));

    // --- NR radio + core, configured as leo-link configures them ---------------------------
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf dlBandConf(dlFrequencyHz, bandwidthHz, 1);
    CcBwpCreator::SimpleOperationBandConf ulBandConf(ulFrequencyHz, bandwidthHz, 1);
    OperationBandInfo dlBand = ccBwpCreator.CreateOperationBandContiguousCc(dlBandConf);
    OperationBandInfo ulBand = ccBwpCreator.CreateOperationBandContiguousCc(ulBandConf);
    dlBand.GetBwpAt(0, 0)->SetChannel(dlCarrier.channel);
    ulBand.GetBwpAt(0, 0)->SetChannel(ulCarrier.channel);
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({dlBand, ulBand});

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);
    beamformingHelper->SetAttribute("BeamformingMethod",
                                    TypeIdValue(DirectPathBeamforming::GetTypeId()));
    beamformingHelper->SetAttribute("BeamformingPeriodicity", TimeValue(MilliSeconds(100)));

    nrHelper->SetGnbPhyAttribute("CellSpecificKoffset", UintegerValue(kOffsetSlots));
    nrHelper->SetUeMacAttribute("IdealRandomAccess", BooleanValue(true));
    // Every flow uses BWP 0, the downlink carrier; the gNB tells the UE that BWP 1 is its uplink
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(0));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(0));

    // The nr module counts the resource blocks of the bandwidth less this share: set so that it
    // counts 3GPP's (half a block of room against rounding)
    nrHelper->SetAttribute("RbOverhead",
                           DoubleValue(1.0 - (carrierRbs + 0.5) * 12 * 15e3 / bandwidthHz));
    // Rank 1 over two cross-polarised ports at each end, CQI/PMI feedback on
    NrHelper::MimoPmiParams mimoPmiParams;
    mimoPmiParams.pmSearchMethod = "ns3::NrPmSearchFull";
    mimoPmiParams.fullSearchCb = "ns3::NrCbTwoPort";
    mimoPmiParams.rankLimit = 1;
    mimoPmiParams.rankThreshold = 0.0;
    mimoPmiParams.rankTechnique = "Sasaoka";
    mimoPmiParams.subbandSize = carrierRbs <= 72 ? 8 : (carrierRbs <= 144 ? 16 : 32);
    mimoPmiParams.downsamplingTechnique = "FirstPRB";
    nrHelper->SetupMimoPmi(mimoPmiParams);

    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(scheduler));
    // The error model, and with it the MCS table and the HARQ combining: --errorModel, the same
    // for every link (the gNB's AMC and decoder for the uplink, the UE's decoder and the gNB's
    // AMC for the downlink; the terrestrial cells of --farEnd=ue too)
    nrHelper->SetDlErrorModel("ns3::" + errorModel);
    nrHelper->SetUlErrorModel("ns3::" + errorModel);
    // HARQ. --harq=0: no retransmission in either direction, and in downlink the process of a
    // block is free again as soon as the block is sent (Rel-17 downlinkHARQ-FeedbackDisabled),
    // not when the answer of the UE is back.
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(harq));
    nrHelper->SetSchedulerAttribute("DlHarqFeedbackDisabled", BooleanValue(!harq));
    // The symbols before the data: --pdcchSymbols, and with 0 the fewest a slot can have
    nrHelper->SetSchedulerAttribute("DlCtrlSymbols",
                                    UintegerValue(pdcchSymbols == 0 ? 1 : pdcchSymbols));
    nrHelper->SetGnbMacAttribute("NumHarqProcess", UintegerValue(harqProcesses));
    nrHelper->SetUeMacAttribute("NumHarqProcess", UintegerValue(harqProcesses));
    nrHelper->SetGnbPhyAttribute("N1Delay", UintegerValue(2));
    nrHelper->SetGnbPhyAttribute("N2Delay", UintegerValue(2));
    nrHelper->SetSchedulerAttribute("OllaStepUp", DoubleValue(0.01));
    nrHelper->SetSchedulerAttribute("OllaTargetBler", DoubleValue(targetBler));
    nrHelper->SetSchedulerAttribute("OllaMaxOffset", DoubleValue(12.0));
    // Uplink link adaptation, --ulLinkAdaptation: the UE spreads its 23 dBm over the resource
    // blocks of its grant, so the MCS of a grant is that of the SINR measured on its last
    // transmission scaled to these resource blocks; and an outer loop on the decoding results
    // keeps its first transmissions at --targetBler, as in downlink
    nrHelper->SetSchedulerAttribute("UlMcsPerRbs", BooleanValue(ulLinkAdaptation));
    nrHelper->SetSchedulerAttribute("UlOlla", BooleanValue(ulLinkAdaptation));
    // The grant for a scheduling request: one voice frame with its headers and the short buffer
    // status report the UE sends with it (5 B as the UE counts it), so that a packet that starts
    // a talk spurt, or any after a pause, goes in one grant and not in two, a round trip apart
    const uint32_t voiceSrBytes =
        voicePayload +
        (rohc ? AttributeDefault<UintegerValue>("ns3::NrPdcp", "RohcTableUo0Size")
              : (rtp ? 40 : 28)) +
        ((l2Headers == "1+1+3") ? 1 + 1 + 3 : 2 + 2 + 3) + 5;
    if (srGrantBytes == 0)
    {
        srGrantBytes = voiceSrBytes;
    }
    nrHelper->SetSchedulerAttribute("SrGrantBytes", UintegerValue(srGrantBytes));
    // The uplink grant of a UE: --ulGrantRbs resource blocks at most (OFDMA), into which it puts
    // its 23 dBm (TR 38.821 Table 6.1.3.2-1: 360 kHz). Its configured-grant blocks too.
    nrHelper->SetSchedulerAttribute("UlGrantRbs", UintegerValue(ulGrantRbs));
    // PUSCH repetition type A (TS 38.214 6.1.2.1): a UE sends each uplink block, and each HARQ
    // retransmission of it, in K slots in a row, and the gNB combines them before it decodes
    nrHelper->SetSchedulerAttribute("PuschRepetitions", UintegerValue(puschRepetitions));
    nrHelper->SetSchedulerAttribute("PuschRepetitionSinrDb", DoubleValue(puschRepSinrDb));
    // The downlink power: TxPower over the RBs of the carrier on each RB in use, nothing on an
    // idle one. --dlBoostMaxDb: an RB in use also gets the power of the idle RBs of its symbol,
    // up to that many dB more (NrGnbPhy::MaxRbBoostDb; UniformPowerAllocUsed splits TxPower
    // over the RBs in use, which the cap limits)
    if (dlBoostMaxDb > 0.0)
    {
        nrHelper->SetGnbPhyAttribute("PowerAllocationType", StringValue("UniformPowerAllocUsed"));
        nrHelper->SetGnbPhyAttribute("MaxRbBoostDb", DoubleValue(dlBoostMaxDb));
    }

    // Satellite: one element, two polarisations, 30 dBi. The gNB transmits the EIRP minus that gain.
    nrHelper->SetGnbAntennaTypeId("ns3::UniformPlanarArray");
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumVerticalPorts", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumHorizontalPorts", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("IsDualPolarized", BooleanValue(true));
    nrHelper->SetGnbAntennaAttribute("PolSlantAngle", DoubleValue(0.0));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
                                         "Gain",
                                         DoubleValue(satAntennaGainDb))));
    // Handheld: one element, two +/-45 deg ports, --ueAntennaGainDbi (with the beam taper, below)
    nrHelper->SetUeAntennaTypeId("ns3::UniformPlanarArray");
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumVerticalPorts", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumHorizontalPorts", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("IsDualPolarized", BooleanValue(true));
    nrHelper->SetUeAntennaAttribute("PolSlantAngle", DoubleValue(45.0 * M_PI / 180.0));

    // --- One gNB (on the satellite node, where the radio is) and nUes UEs -------------------
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(NodeContainer(satellite), allBwps);
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)
        ->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 1)
        ->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));

    // Each UE is installed on its own because its antenna gain carries the beam taper of its
    // position, and that once per carrier (BWP 0 = downlink, BWP 1 = uplink): the beam is not
    // equally wide on the two. The helper gives both BWPs the element of the downlink; the uplink
    // BWP gets its own below, where the arrays are collected.
    NetDeviceContainer ueDevs;
    for (uint32_t i = 0; i < nUes; ++i)
    {
        g_scene.ueElements.emplace_back();
        for (double ka : g_scene.beamKa)
        {
            const double taperDb = BeamTaperDb(satMobility->GetPosition(),
                                               centrePosition,
                                               ueMobility[i]->GetPosition(),
                                               ka);
            g_scene.ueElements[i].push_back(CreateObjectWithAttributes<IsotropicAntennaModel>(
                "Gain",
                DoubleValue(ueAntennaGainDbi + taperDb)));
        }
        nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(g_scene.ueElements[i][0]));
        ueDevs.Add(nrHelper->InstallUeDevice(NodeContainer(ueNodes.Get(i)), allBwps));
    }

    for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
    {
        NrHelper::GetGnbPhy(gnbDevs.Get(0), bwp)->SetTxPower(satEirpDbm - satAntennaGainDb);
        NrHelper::GetGnbPhy(gnbDevs.Get(0), bwp)
            ->SetAttribute("NoiseFigure", DoubleValue(satNoiseFigureDb));
        for (uint32_t i = 0; i < nUes; ++i)
        {
            NrHelper::GetUePhy(ueDevs.Get(i), bwp)->SetTxPower(ueTxPowerDbm);
            NrHelper::GetUePhy(ueDevs.Get(i), bwp)
                ->SetAttribute("NoiseFigure", DoubleValue(ueNoiseFigureDb));
        }
        // (the PHY counts its resource blocks once the simulation has started)
        Simulator::Schedule(MilliSeconds(1),
                            [phy = NrHelper::GetGnbPhy(gnbDevs.Get(0), bwp), carrierRbs]() {
                                NS_ABORT_MSG_UNLESS(phy->GetRbNum() == carrierRbs,
                                                    "the nr module made "
                                                        << phy->GetRbNum()
                                                        << " resource blocks of the carrier, not "
                                                        << carrierRbs);
                            });
    }

    // --- The far end, --farEnd=ue: a terrestrial NR UE for every call ---------------------------
    // Typical 5G cells in ideal conditions, on a band of their own, so nothing they send meets the
    // satellite's carriers. One gNB (one cell) per --farEndUesPerGnb calls; gNBs 100 km apart,
    // each cell on a spectrum channel of its own (no interference between them); all on the core
    // of the satellite cell (same UPF). UE k of the calls is paired with satellite UE k and sits in
    // cell k / --farEndUesPerGnb. Defaults in brackets (the band, its bandwidth and carrier are
    // checked above):
    //   band       --farEndBand [n78]: TDD, 30 kHz, slots --farEndTddPattern [DDSUU]; n3: FDD,
    //              15 kHz; --farEndBandwidthMHz [20], --farEndFrequencyGHz [3.5 / 1.8425]
    //   gNB        --farEndGnbDbm [44] per 20 MHz, a 4 x 8 panel of cross-polarised elements of
    //              TR 38.901 Table 7.3-1 (8 dBi each), noise figure 5 dB: ITU-R M.2412's
    //              Dense Urban-eMBB macro. The panel faces its sector; the gNB is one cell, i.e.
    //              one sector of a site. Its height is that of the scenario (TR 38.901 Table 7.2-1)
    //   UE         1.5 m high, 23 dBm, one cross-polarised pair of 0 dBi, noise figure 7 dB
    //   where      uniformly over the sector of the cell, 120 deg wide and --farEndMinDistM [35]
    //              to --farEndMaxDistM [250] m from the gNB (35 m: the shortest of TR 38.901's UMa)
    //   channel    TR 38.901 --farEndScenario [UMa-LOS: UMa in line of sight], with fast fading
    //   core       gNB - UPF --farEndUpfDelayMs [5] one way
    //   layer 2    plain NR: the nr module's defaults, whatever the run sets for the satellite
    //              cell (no ROHC, no PDCCH limit, no SPS, ...; see PlainFarBearers)
    const double farBandwidthHz = farEndBandwidthMHz * 1e6;
    const uint16_t farNumerology = farTdd ? 1 : 0;
    const double farGnbDbm = farEndGnbDbm + 10.0 * std::log10(farEndBandwidthMHz / 20.0);
    // TR 38.901 Table 7.2-1: the gNB is 25 m high in UMa, 10 m in UMi, 35 m in RMa
    const std::string farChannel = farEndScenario.substr(0, 3);
    const bool farAlwaysLos = farEndScenario.size() > 3;
    const double farGnbNoiseFigureDb = 5.0;
    const double farGnbHeightM = farChannel == "UMi" ? 10.0 : (farChannel == "RMa" ? 35.0 : 25.0);
    const double farUeHeightM = 1.5;
    const double farMinM = farEndMinDistM;
    const double farMaxM = farEndMaxDistM;
    const double farSectorDeg = 120.0;
    const double farSpacingM = 100e3;
    const uint32_t farUesPerCell = farEndUesPerGnb;
    const uint32_t farCells = farUe ? (nUes + farUesPerCell - 1) / farUesPerCell : 0;
    const Time farUpfDelay = MicroSeconds(std::llround(farEndUpfDelayMs * 1000.0));
    NodeContainer farGnbNodes;
    NodeContainer farUeNodes;
    NetDeviceContainer farGnbDevs;
    NetDeviceContainer farUeDevs;
    std::deque<OperationBandInfo> farBands;       // they hold the BWPs the devices were made with
    std::vector<double> farDistanceM(nUes, NAN);  // where far UE k is: from its gNB ...
    std::vector<double> farAzimuthDeg(nUes, NAN); // ... and off the boresight of the panel
    if (farUe)
    {
        farGnbNodes.Create(farCells);
        farUeNodes.Create(nUes);
        nrHelper->SetDlErrorModel("ns3::" + farEndErrorModel);
        nrHelper->SetUlErrorModel("ns3::" + farEndErrorModel);
        nrHelper->SetGnbPhyAttribute("CellSpecificKoffset", UintegerValue(0));
        nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(farNumerology));
        nrHelper->SetGnbPhyAttribute("PowerAllocationType", StringValue("UniformPowerAllocBw"));
        nrHelper->SetSchedulerAttribute("UlGrantRbs", UintegerValue(0));
        nrHelper->SetSchedulerAttribute("SpsScenario", UintegerValue(0));
        nrHelper->SetSchedulerAttribute("DlCtrlSymbols", UintegerValue(1));
        nrHelper->SetUePhyAttribute("PdcchErrorModel", BooleanValue(false));
        // Plain NR: what the run sets for the satellite cell goes back to the nr module's default
        auto moduleDefault = [](const std::string& tid, const std::string& name) {
            TypeId::AttributeInformation info;
            NS_ABORT_UNLESS(TypeId::LookupByName(tid).LookupAttributeByName(name, &info));
            return info.originalInitialValue;
        };
        for (const char* name : {"EnableHarqReTx",
                                 "DlHarqFeedbackDisabled",
                                 "OllaStepUp",
                                 "OllaTargetBler",
                                 "OllaMaxOffset",
                                 "UlMcsPerRbs",
                                 "UlOlla",
                                 "SrGrantBytes"})
        {
            nrHelper->SetSchedulerAttribute(name, *moduleDefault("ns3::NrMacSchedulerNs3", name));
        }
        nrHelper->SetGnbMacAttribute("NumHarqProcess",
                                     *moduleDefault("ns3::NrGnbMac", "NumHarqProcess"));
        nrHelper->SetUeMacAttribute("NumHarqProcess",
                                    *moduleDefault("ns3::NrUeMac", "NumHarqProcess"));
        nrHelper->SetGnbPhyAttribute("MaxRbBoostDb",
                                     *moduleDefault("ns3::NrGnbPhy", "MaxRbBoostDb"));
        nrHelper->SetAttribute(
            "RbOverhead",
            DoubleValue(1.0 - (farRbs + 0.5) * 12 * 15e3 * (1 << farNumerology) / farBandwidthHz));
        nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
        nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
        nrHelper->SetGnbAntennaAttribute("PolSlantAngle", DoubleValue(45.0 * M_PI / 180.0));
        nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                         PointerValue(CreateObject<ThreeGppAntennaModel>()));
        nrHelper->SetUeAntennaAttribute(
            "AntennaElement",
            PointerValue(
                CreateObjectWithAttributes<IsotropicAntennaModel>("Gain", DoubleValue(0.0))));
        // (the helper connects every gNB it installs to the core with this delay)
        epcHelper->SetAttribute("S1uLinkDelay", TimeValue(farUpfDelay));
        Ptr<UniformRandomVariable> farDrop =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(305));
        for (uint32_t c = 0; c < farCells; ++c)
        {
            // The cell's band and its own channel(s), TR 38.901 --farEndScenario ("Default": the
            // line of sight probability of the scenario)
            CcBwpCreator creator;
            Ptr<NrChannelHelper> channels = CreateObject<NrChannelHelper>();
            channels->ConfigureFactories(farChannel, farAlwaysLos ? "LOS" : "Default", "ThreeGpp");
            const auto flags = NrChannelHelper::INIT_PROPAGATION | NrChannelHelper::INIT_FADING;
            BandwidthPartInfoPtrVector bwps;
            farBands.push_back(creator.CreateOperationBandContiguousCc(
                CcBwpCreator::SimpleOperationBandConf(farDlHz, farBandwidthHz, 1)));
            if (farTdd)
            {
                channels->AssignChannelsToBands({farBands.back()}, flags);
                bwps = CcBwpCreator::GetAllBwps({farBands.back()});
            }
            else
            {
                farBands.push_back(creator.CreateOperationBandContiguousCc(
                    CcBwpCreator::SimpleOperationBandConf(farUlHz, farBandwidthHz, 1)));
                OperationBandInfo& dl = farBands[farBands.size() - 2];
                OperationBandInfo& ul = farBands.back();
                // the uplink gets its fast fading, and the downlink's line of sight state and path
                // loss (MirroredLossModel)
                channels->AssignChannelsToBands({dl}, flags);
                channels->AssignChannelsToBands({ul}, NrChannelHelper::INIT_FADING);
                bwps = CcBwpCreator::GetAllBwps({dl, ul});
                Ptr<SpectrumChannel> dlChannel = bwps[0].get()->GetChannel();
                Ptr<SpectrumChannel> ulChannel = bwps[1].get()->GetChannel();
                auto dlLoss =
                    DynamicCast<ThreeGppPropagationLossModel>(dlChannel->GetPropagationLossModel());
                auto ulFading = DynamicCast<ThreeGppSpectrumPropagationLossModel>(
                    ulChannel->GetPhasedArraySpectrumPropagationLossModel());
                NS_ABORT_MSG_UNLESS(dlLoss && ulFading && !ulChannel->GetPropagationLossModel(),
                                    "the far cell's channels are not the 3GPP ones");
                ulFading->GetChannelModel()->SetAttribute(
                    "ChannelConditionModel",
                    PointerValue(dlLoss->GetChannelConditionModel()));
                Ptr<MirroredLossModel> mirror = CreateObject<MirroredLossModel>();
                mirror->Mirror(dlLoss, 20.0 * std::log10(farDlHz / farUlHz));
                ulChannel->AddPropagationLossModel(mirror);
            }
            for (auto& bwp : bwps)
            {
                bwp.get()->GetChannel()->AddSpectrumTransmitFilter(
                    CreateObject<NrNtnUeToUeFilter>());
            }
            Ptr<ConstantPositionMobilityModel> gnbMobility =
                CreateObject<ConstantPositionMobilityModel>();
            gnbMobility->SetPosition(Vector(c * farSpacingM, 0.0, farGnbHeightM));
            farGnbNodes.Get(c)->AggregateObject(gnbMobility);
            NetDeviceContainer gnb =
                nrHelper->InstallGnbDevice(NodeContainer(farGnbNodes.Get(c)), bwps);
            farGnbDevs.Add(gnb);
            if (farTdd)
            {
                std::string pattern;
                for (const char slot : farEndTddPattern)
                {
                    pattern +=
                        slot == 'D' ? "DL|" : (slot == 'U' ? "UL|" : std::string(1, slot) + "|");
                }
                NrHelper::GetGnbPhy(gnb.Get(0), 0)->SetAttribute("Pattern", StringValue(pattern));
            }
            else
            {
                NrHelper::GetGnbPhy(gnb.Get(0), 0)
                    ->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
                NrHelper::GetGnbPhy(gnb.Get(0), 1)
                    ->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));
            }
            // Its UEs, uniformly over its sector: the panel looks along +x
            for (uint32_t k = c * farUesPerCell; k < std::min(nUes, (c + 1) * farUesPerCell); ++k)
            {
                const double r =
                    std::sqrt(farMinM * farMinM +
                              farDrop->GetValue() * (farMaxM * farMaxM - farMinM * farMinM));
                const double azimuthDeg = (farDrop->GetValue() - 0.5) * farSectorDeg;
                farDistanceM[k] = r;
                farAzimuthDeg[k] = azimuthDeg;
                Ptr<ConstantPositionMobilityModel> ueMobility =
                    CreateObject<ConstantPositionMobilityModel>();
                ueMobility->SetPosition(
                    Vector(c * farSpacingM + r * std::cos(azimuthDeg * M_PI / 180.0),
                           r * std::sin(azimuthDeg * M_PI / 180.0),
                           farUeHeightM));
                farUeNodes.Get(k)->AggregateObject(ueMobility);
                farUeDevs.Add(nrHelper->InstallUeDevice(NodeContainer(farUeNodes.Get(k)), bwps));
            }
            for (uint32_t bwp = 0; bwp < bwps.size(); ++bwp)
            {
                NrHelper::GetGnbPhy(gnb.Get(0), bwp)->SetTxPower(farGnbDbm);
                NrHelper::GetGnbPhy(gnb.Get(0), bwp)
                    ->SetAttribute("NoiseFigure", DoubleValue(farGnbNoiseFigureDb));
                Simulator::Schedule(MilliSeconds(1),
                                    [phy = NrHelper::GetGnbPhy(gnb.Get(0), bwp), farRbs]() {
                                        NS_ABORT_MSG_UNLESS(phy->GetRbNum() == farRbs,
                                                            "the nr module made "
                                                                << phy->GetRbNum()
                                                                << " resource blocks of the far "
                                                                   "cell's carrier, not "
                                                                << farRbs);
                                    });
            }
        }
        for (uint32_t k = 0; k < nUes; ++k)
        {
            for (uint32_t bwp = 0; bwp < (farTdd ? 1U : 2U); ++bwp)
            {
                NrHelper::GetUePhy(farUeDevs.Get(k), bwp)->SetTxPower(ueTxPowerDbm);
                NrHelper::GetUePhy(farUeDevs.Get(k), bwp)
                    ->SetAttribute("NoiseFigure", DoubleValue(ueNoiseFigureDb));
            }
        }
        // The PDCCH of the terrestrial cells is unlimited, as in the nr module without NrPdcch
        // (the cell's NrPdcch is made here, before its scheduler asks for it)
        for (uint32_t c = 0; c < farCells; ++c)
        {
            NrPdcch::Get(farGnbDevs.Get(c)->GetObject<NrGnbNetDevice>()->GetCellId())
                ->SetAttribute("Enabled", BooleanValue(false));
        }
    }

    // --- The remote host: the far end of every call unless --farEnd=ue --------------------------
    auto [remoteHost, remoteHostAddr] = epcHelper->SetupRemoteHost("10Gb/s", 1500, remoteHostDelay);

    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIfaces =
        AssignUeAddresses(ueDevs, 7, epcHelper->GetUeDefaultGatewayAddress());
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);
    Ipv4InterfaceContainer farIfaces;
    if (farUe)
    {
        internet.Install(farUeNodes);
        // The terrestrial UEs have a network of their own, 8.0.0.0/8: UE k is UeAddress(8, k),
        // 8.0.0.(k+2) for the first 253. The satellite UEs have the core's, 7.0.0.0/8. The UPF is
        // the gateway of both: its tunnel device, 7.0.0.1, is also 8.0.0.1, so it routes the calls
        // between the two networks.
        Ptr<Ipv4> upfIp = epcHelper->GetPgwNode()->GetObject<Ipv4>();
        upfIp->AddAddress(upfIp->GetInterfaceForAddress(epcHelper->GetUeDefaultGatewayAddress()),
                          Ipv4InterfaceAddress("8.0.0.1", "255.0.0.0"));
        farIfaces = AssignUeAddresses(farUeDevs, 8, Ipv4Address("8.0.0.1"));
        for (uint32_t k = 0; k < nUes; ++k)
        {
            nrHelper->AttachToGnb(farUeDevs.Get(k), farGnbDevs.Get(k / farUesPerCell));
        }
    }

    // Timing advance: each UE sends early by its one-way delay to the gNB (feeder link included
    // when transparent), on both of its BWP PHYs. The UE PHY asks again before every slot, so the
    // advance follows a moving satellite.
    for (uint32_t i = 0; i < nUes; ++i)
    {
        for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
        {
            NrHelper::GetUePhy(ueDevs.Get(i), bwp)
                ->SetTimingAdvanceCallback(
                    Callback<Time>([ntnDelay, satMobility, ue = ueMobility[i]]() {
                        return ntnDelay->GetOneWayDelay(satMobility, ue);
                    }));
        }
    }

    // The satellite points its arrays at the beam centre: once when it stands still, every
    // 100 ms when it flies, together with the beam gain towards every UE
    g_scene.satellite = satMobility;
    g_scene.centre = centrePosition;
    g_scene.ues = ueMobility;
    g_scene.delay = ntnDelay;
    g_scene.ueAntennaGainDbi = ueAntennaGainDbi;
    g_scene.flying = !freezeGeometry;
    g_scene.orbit = orbit;
    // The ground part of the way to the gateway: from the remote host, or from the far UE's gNB
    // (whose radio adds next to nothing)
    g_scene.groundDelayMs = farUe ? farEndUpfDelayMs + upfGwDelayMs : hostGwDelayMs;
    for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
    {
        g_scene.satArrays.push_back(NrHelper::GetGnbPhy(gnbDevs.Get(0), bwp)
                                        ->GetSpectrumPhy()
                                        ->GetAntenna()
                                        ->GetObject<UniformPlanarArray>());
    }
    for (uint32_t i = 0; i < nUes; ++i)
    {
        g_scene.ueArrays.emplace_back();
        g_scene.uePhys.emplace_back();
        for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
        {
            g_scene.ueArrays[i].push_back(DynamicCast<PhasedArrayModel>(
                NrHelper::GetUePhy(ueDevs.Get(i), bwp)->GetSpectrumPhy()->GetAntenna()));
            g_scene.uePhys[i].push_back(NrHelper::GetUePhy(ueDevs.Get(i), bwp));
            if (bwp > 0)
            {
                g_scene.ueArrays[i][bwp]->SetAntennaElement(g_scene.ueElements[i][bwp]);
            }
        }
    }
    // Power control: from the start, and every 100 ms with the beam when the satellite flies
    g_scene.dlPowerRefDeg = dlPowerRefDeg;
    g_scene.dlFrequencyHz = dlFrequencyHz;
    g_scene.dlExtra = dlCarrier.serviceExtra;
    g_scene.dlPhy = NrHelper::GetGnbPhy(gnbDevs.Get(0), 0);
    g_scene.dlNominalDbm = satEirpDbm - satAntennaGainDb;
    g_scene.ulPowerRefDeg = ulPowerRefDeg;
    g_scene.ueMaxDbm = ueTxPowerDbm;
    g_scene.dlServiceLoss = dlCarrier.serviceLoss;
    g_scene.ueLossDb.assign(nUes, NAN);
    if (dlPowerRefDeg > 0.0)
    {
        g_scene.dlRefLossDb =
            FeederLinkLossModel::FreeSpaceDb(dlFrequencyHz, slantM(dlPowerRefDeg)) +
            dlCarrier.serviceExtra->AtmosphericDb(dlPowerRefDeg);
        DlPowerControl();
    }
    if (ulPowerRefDeg > 0.0)
    {
        // what the reference UE measures: free space, the shadowing all UEs have (--shadowing=
        // fixed; with the table its median, 0), gases, scintillation, --linkMarginDb
        g_scene.ulRefLossDb =
            FeederLinkLossModel::FreeSpaceDb(dlFrequencyHz, slantM(ulPowerRefDeg)) +
            (fixedShadowing ? shadowingDb : 0.0) + linkMarginDb +
            dlCarrier.serviceExtra->AtmosphericDb(ulPowerRefDeg) +
            dlCarrier.serviceExtra->ScintillationDb();
        UlPowerControl();
    }
    PointBeam();
    if (!freezeGeometry)
    {
        Simulator::Schedule(MilliSeconds(100), &FollowBeam);
    }

    // The voice bearer: for every UE a dedicated GBR flow with 5QI 1 (TS 23.501 Table 5.7.4-1,
    // conversational voice), selected by the UDP port: what the UE receives on it (local port) and
    // what it sends to it (remote port). One call of the helper and one rule per UE:
    // NrHelper::ActivateDedicatedQosFlow() for a whole container of devices sets up the first
    // device only (it returns inside its loop), and the core network writes the flow identifier
    // into the rule it is given. Every UE also keeps its default flow, unused.
    NrGbrQosInformation qos;
    qos.gbrUl = qos.gbrDl = qos.mbrUl = qos.mbrDl = static_cast<uint64_t>(gfbrKbps * 1000.0);
    const NrQosFlow voiceFlow(NrQosFlow::GBR_CONV_VOICE, qos);
    auto voiceRule = [port](bool receives, bool sends) {
        Ptr<NrQosRule> rule = Create<NrQosRule>();
        if (receives)
        {
            NrQosRule::PacketFilter filter;
            filter.direction = NrQosRule::DOWNLINK;
            filter.localPortStart = filter.localPortEnd = port;
            rule->Add(filter);
        }
        if (sends)
        {
            NrQosRule::PacketFilter filter;
            filter.direction = NrQosRule::UPLINK;
            filter.remotePortStart = filter.remotePortEnd = port;
            rule->Add(filter);
        }
        return rule;
    };
    for (uint32_t i = 0; i < nUes; ++i)
    {
        nrHelper->ActivateDedicatedQosFlow(ueDevs.Get(i), voiceFlow, voiceRule(sendDown, sendUp));
    }
    for (uint32_t k = 0; farUe && k < nUes; ++k)
    {
        nrHelper->ActivateDedicatedQosFlow(farUeDevs.Get(k),
                                           voiceFlow,
                                           voiceRule(sendUp, sendDown));
        std::vector<Ptr<NrUePhy>> phys;
        for (uint32_t bwp = 0; bwp < (farTdd ? 1U : 2U); ++bwp)
        {
            phys.push_back(NrHelper::GetUePhy(farUeDevs.Get(k), bwp));
        }
        Connect(DynamicCast<NrUeNetDevice>(farUeDevs.Get(k))->GetRrc(),
                "StateTransition",
                MakeBoundCallback(&SetFarSubband, phys, farRbs <= 72 ? 8U : 16U));
    }
    // What TS 23.501 asks of that flow, as the nr module has it; the averaging window of 5QI 1
    // is 2000 ms (Table 5.7.4-1)
    const Time voicePdb = MilliSeconds(voiceFlow.GetPacketDelayBudgetMs());
    const double voicePer = voiceFlow.GetPacketErrorLossRate();
    const Time voiceAveragingWindow = MilliSeconds(2000);
    NS_ABORT_MSG_UNLESS(voiceFlow.GetResourceType() == 1 && voicePdb == MilliSeconds(100) &&
                            voicePer == 1e-2,
                        "5QI 1 of the nr module is not the one of TS 23.501 Table 5.7.4-1");
    // The bearers of the terrestrial cells are plain NR at both ends (PlainFarBearers), and the
    // RLC of a terrestrial gNB takes its own way from the UPF off the budget, not the satellite's
    // (the defaults of every PDCP and RLC are the satellite cell's: they are set again on each
    // bearer of a terrestrial gNB or UE when it is set up)
    for (uint32_t c = 0; c < farCells; ++c)
    {
        const std::string drbs = "/NodeList/" + std::to_string(farGnbNodes.Get(c)->GetId()) +
                                 "/DeviceList/*/$ns3::NrGnbNetDevice/NrGnbRrc/UeMap/*/"
                                 "DataRadioBearerMap/*";
        Connect(farGnbDevs.Get(c)->GetObject<NrGnbNetDevice>()->GetRrc(),
                "ConnectionReconfiguration",
                MakeBoundCallback(&FarGnbReconfigured, drbs, farEndUpfDelayMs));
    }
    for (uint32_t k = 0; farUe && k < nUes; ++k)
    {
        const std::string drbs = "/NodeList/" + std::to_string(farUeNodes.Get(k)->GetId()) +
                                 "/DeviceList/*/$ns3::NrUeNetDevice/NrUeRrc/DataRadioBearerMap/*";
        Connect(DynamicCast<NrUeNetDevice>(farUeDevs.Get(k))->GetRrc(),
                "DrbCreated",
                MakeBoundCallback(&FarUeDrbCreated, drbs));
    }

    // The delays and the bit rate of every voice flow, see VoiceStampTag
    g_voiceQos = VoiceQosScene();
    g_voiceQos.sat.assign(nUes, {});
    g_voiceQos.far.assign(nUes, {});
    g_voiceQos.e2e.assign(nUes, {});
    g_voiceQos.port = port;
    g_voiceQos.remoteHost = remoteHostAddr.Get();
    g_voiceQos.windowUs = voiceAveragingWindow.GetMicroSeconds();
    g_voiceQos.pdbNs = voicePdb.GetNanoSeconds();
    Connect(epcHelper->GetPgwNode()->GetObject<Ipv4L3Protocol>(), "Rx", MakeCallback(&PgwIpReceives));
    for (uint32_t i = 0; i < nUes; ++i)
    {
        g_voiceQos.endpoint[ueIfaces.GetAddress(i).Get()] = {i, false};
        Ptr<Ipv4L3Protocol> ip = ueNodes.Get(i)->GetObject<Ipv4L3Protocol>();
        Connect(ip, "SendOutgoing", MakeBoundCallback(&UeIpSends, i, false));
        Connect(ip, "LocalDeliver", MakeBoundCallback(&UeIpDelivers, i, false));
    }
    for (uint32_t k = 0; farUe && k < nUes; ++k)
    {
        g_voiceQos.endpoint[farIfaces.GetAddress(k).Get()] = {k, true};
        Ptr<Ipv4L3Protocol> ip = farUeNodes.Get(k)->GetObject<Ipv4L3Protocol>();
        Connect(ip, "SendOutgoing", MakeBoundCallback(&UeIpSends, k, true));
        Connect(ip, "LocalDeliver", MakeBoundCallback(&UeIpDelivers, k, true));
    }
    if (!farUe)
    {
        Ptr<Ipv4L3Protocol> ip = remoteHost->GetObject<Ipv4L3Protocol>();
        Connect(ip, "SendOutgoing", MakeCallback(&HostIpSends));
        Connect(ip, "LocalDeliver", MakeCallback(&HostIpDelivers));
    }

    // Sinks: on every satellite UE that hears (DOWN), and on the far end that hears (UP): every
    // terrestrial UE, or the remote host for all calls
    PacketSinkHelper sinkHelper(transportProtocol, InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApps;
    if (sendDown)
    {
        sinkApps.Add(sinkHelper.Install(ueNodes));
    }
    if (sendUp)
    {
        sinkApps.Add(sinkHelper.Install(farUe ? farUeNodes : NodeContainer(remoteHost)));
    }
    sinkApps.Start(Seconds(0));
    sinkApps.Stop(callsEnd + Seconds(2));

    // The loop holds only what differs per call
    g_dlSinrSum.assign(nUes, 0);
    g_dlSinrN.assign(nUes, 0);
    g_dlSnr.assign(nUes, SnrStats());
    g_ulSnr.assign(nUes, SnrStats());
    g_ulRxSnr.assign(nUes, SnrStats());
    g_snrWindow.assign(nUes, {Seconds(0), Seconds(0)});
    g_rntiUe.clear();
    g_ulSnrRbs = ulGrantRbs > 0 ? ulGrantRbs : 2; // as the CNR in the header
    for (uint32_t bwp = 0; bwp < 2; ++bwp)
    {
        NrHelper::GetGnbPhy(gnbDevs.Get(0), bwp)
            ->GetSpectrumPhy()
            ->TraceConnectWithoutContext("RxPacketTraceGnb", MakeCallback(&UlSinr));
    }
    g_rlf.assign(nUes, 0);
    g_outages.assign(nUes, {});
    g_connections.assign(nUes, {});
    g_ueAddress.clear();
    for (uint32_t i = 0; i < nUes; ++i)
    {
        g_ueAddress.push_back(ueIfaces.GetAddress(i));
    }
    g_view.assign(nUes, CallView());
    g_lastVoiceTx.assign(2 * nUes, Seconds(-1));
    g_cc.status.assign(nUes, CallControl::WAITING);
    g_cc.held.assign(nUes, {});
    g_cc.at.assign(nUes, Seconds(0));
    g_cc.limitedBy.assign(nUes, -1);
    g_cc.onTimePct.assign(nUes, NAN);
    g_cc.releasedDir.assign(nUes, -1);
    g_cc.sources.assign(nUes, {});
    g_cc.end.assign(nUes, Seconds(0));
    g_farSinrSum.assign(nUes, 0);
    g_farSinrN.assign(nUes, 0);
    for (uint32_t k = 0; farUe && k < nUes; ++k)
    {
        Connect(NrHelper::GetUePhy(farUeDevs.Get(k), 0),
                "DlCtrlSinr",
                MakeBoundCallback(&FarDlSinr, k));
    }
    // Line of sight: the same on both carriers (one obstruction level per UE); read on the downlink
    g_scene.condition = DynamicCast<ThreeGppPropagationLossModel>(dlCarrier.serviceLoss)
                            ->GetChannelConditionModel();
    // Where each direction of call i is sent from and to; the random streams of its talk and
    // silence (the first direction in use has those of a one-way call before two-way calls); the
    // RTP SSRC
    auto farNode = [&](uint32_t i) { return farUe ? farUeNodes.Get(i) : remoteHost; };
    auto farAddress = [&](uint32_t i) { return farUe ? farIfaces.GetAddress(i) : remoteHostAddr; };
    ApplicationContainer voipApps;
    // The voice packets an RLC of a satellite UE's bearer discards for waiting too long, for
    // the release of calls (CheckCalls())
    static std::map<uint64_t, uint32_t> ueOfImsi;
    for (uint32_t i = 0; i < nUes; ++i)
    {
        ueOfImsi[DynamicCast<NrUeNetDevice>(ueDevs.Get(i))->GetImsi()] = i;
        const std::string drbs = "/NodeList/" + std::to_string(ueDevs.Get(i)->GetNode()->GetId()) +
                                 "/DeviceList/*/$ns3::NrUeNetDevice/NrUeRrc/DataRadioBearerMap/*";
        Connect(DynamicCast<NrUeNetDevice>(ueDevs.Get(i))->GetRrc(),
                "DrbCreated",
                MakeBoundCallback(&SatUeDrbCreated, drbs, i));
    }
    Connect(gnbDevs.Get(0)->GetObject<NrGnbNetDevice>()->GetRrc(),
            "ConnectionReconfiguration",
            MakeBoundCallback(&SatGnbReconfigured,
                              "/NodeList/" + std::to_string(gnbDevs.Get(0)->GetNode()->GetId()) +
                                  "/DeviceList/*/$ns3::NrGnbNetDevice/NrGnbRrc/UeMap",
                              &ueOfImsi));
    for (uint32_t i = 0; i < nUes; ++i)
    {
        Connect(NrHelper::GetUePhy(ueDevs.Get(i), 0), "DlCtrlSinr", MakeBoundCallback(&DlSinr, i));
        Connect(DynamicCast<NrUeNetDevice>(ueDevs.Get(i))->GetRrc(),
                "RadioLinkFailure",
                MakeBoundCallback(&RadioLinkFailure, i));
        Connect(DynamicCast<NrUeNetDevice>(ueDevs.Get(i))->GetRrc(),
                "ConnectionEstablished",
                MakeBoundCallback(&ConnectionEstablished, i));
        if (!ueHasCall[i])
        {
            continue; // attached, but the satellite is never high enough for it during UE 0's call
        }
        // The satellite side controls the call: both directions start and end with its window
        const Time callStart = ueCallStart[i];
        g_snrWindow[i] = {ueCallStart[i], ueCallEnd[i]};
        for (const Dir dir : {DOWN, UP})
        {
            if (dir == DOWN ? !sendDown : !sendUp)
            {
                continue;
            }
            const bool first = (dir == DOWN || !sendDown);
            Ptr<Node> from = dir == DOWN ? farNode(i) : ueNodes.Get(i);
            const Ipv4Address to = dir == DOWN ? ueIfaces.GetAddress(i) : farAddress(i);
            ApplicationContainer voip;
            if (rtp)
            {
                Ptr<RtpVoiceSource> source = CreateObject<RtpVoiceSource>();
                source->Setup(InetSocketAddress(to, port),
                              vaf[dir][i],
                              voicePayload,
                              sidPayload,
                              hangoverFrames,
                              sidIntervalMs / 20,
                              i,
                              first ? 600 : 1000600,
                              (first ? 0x4e544e00 : 0x4e545500) + i); // "NTN"/"NTU" + the call
                from->AddApplication(source);
                voip.Add(source);
                g_cc.sources[i][dir] = source;
            }
            else
            {
                TrafficGeneratorHelper voipHelper(transportProtocol,
                                                  InetSocketAddress(to, port),
                                                  TrafficGeneratorNgmnVoip::GetTypeId());
                voipHelper.SetAttribute("VoiceActivityFactor", DoubleValue(vaf[dir][i]));
                voip = voipHelper.Install(from);
                voip.Get(0)->TraceConnectWithoutContext("Tx",
                                                        MakeBoundCallback(&VoiceSent, 2 * i + dir));
            }
            voip.Start(callStart);
            voip.Stop(ueCallEnd[i]);
            voipApps.Add(voip);
        }
        g_view[i].callStart = callStart;
        g_view[i].callEnd = ueCallEnd[i];
        // The gNB admits the call (or not) just before it starts; its reservation ends with it
        Simulator::Schedule(callStart - MicroSeconds(1), &AdmitCall, i);
        Simulator::Schedule(ueCallEnd[i], &EndCall, i);
        g_cc.end[i] = ueCallEnd[i];
        Simulator::Schedule(callStart, &SampleCall, i);
        if (!freezeGeometry && overheadAt > callStart && overheadAt < ueCallEnd[i])
        {
            // one more look when the satellite is over the centre, which the 100 ms steps can miss by 50 ms
            Simulator::Schedule(overheadAt, &RecordView, i);
        }
    }

    // FlowMonitor on every UE and the remote host gives one flow per direction of a call
    FlowMonitorHelper flowmonHelper;
    NodeContainer monitored(ueNodes, farUeNodes);
    monitored.Add(remoteHost);
    Ptr<FlowMonitor> monitor = flowmonHelper.Install(monitored);

    Simulator::Schedule(Seconds(1), &SampleLos);

    if (nrTraces)
    {
        nrHelper->EnableTraces();
    }

    // --- What was built --------------------------------------------------------------------
    // The delay of the whole path, one way and propagation only: remote host (or the far UE's
    // gNB) to UPF, UPF to gateway, then the feeder link when the gNB is at the gateway, then the
    // service link. With the gNB on the satellite the feeder link carries N3, which has no delay
    // in this model.
    const std::string path = std::string(farUe ? "far gNB" : "remote host") + " -> UPF -> " +
                             (transparent ? "gateway -> " : "") + "satellite -> UE";
    struct Ue0View
    {
        double elevationDeg;
        double rangeKm;
        double serviceMs;
        double feederMs;
    };
    auto ue0At = [&](Time t) {
        const Vector sat = orbit.At(t);
        satProbe->SetPosition(sat);
        Ue0View v;
        v.elevationDeg = ElevationDeg(sat, centrePosition);
        v.rangeKm = (sat - centrePosition).GetLength() / 1000.0;
        v.serviceMs = v.rangeKm * 1e6 / 299792458.0;
        v.feederMs =
            ntnDelay->GetOneWayDelay(satProbe, centreMobility).GetSeconds() * 1000.0 - v.serviceMs;
        return v;
    };
    auto delayParts = [&](const Ue0View& v) {
        std::string parts = Num(g_scene.groundDelayMs + v.feederMs + v.serviceMs, 2) + " ms = " +
                            (farUe ? Num(farEndUpfDelayMs, 2) + " far gNB-UPF + "
                                   : Num(remoteHostDelayMs, 2) + " host-UPF + ") +
                            Num(upfGwDelayMs, 2) + " UPF-gateway";
        if (transparent)
        {
            parts += " + " + Num(v.feederMs, 2) + " feeder";
        }
        return parts + " + " + Num(v.serviceMs, 2) + " service";
    };
    bool ulBlockTooSmall = false; // the uplink grant cannot carry a block (see CELL)
    uint32_t ulBlockBytes = 0;
    std::cout << std::fixed << std::setprecision(2);
    Frame("NR-NTN VOICE CELL",
          std::string(transparent ? "transparent" : "regenerative") + ", " + scenario + ", " +
              Plain(bandwidthMHz) + " MHz FDD, DL " + Num(dlFrequencyGHz, 3) + " / UL " +
              Num(ulFrequencyHz / 1e9, 3) + " GHz, " + std::to_string(nUes) +
              (sendDown && sendUp ? " two-way" : (sendUp ? " uplink" : " downlink")) +
              (nUes == 1 ? " call" : " calls") +
              (farUe ? " to terrestrial UEs (" + farEndBand + ")" : " to the remote host"));

    // ----- the voice and its bearer
    std::cout << "\nVOICE\n";
    const std::string voiceB = std::to_string(voicePayload);
    const std::string sidB = std::to_string(sidPayload);
    Setting("calls",
            sendDown && sendUp
                ? "two-way: the UE and the far end each have a speaker, with a VAF, talk and "
                  "silence of its own"
                : (sendDown ? "one-way: the far end talks to the UE (--direction=dl)"
                            : "one-way: the UE talks to the far end (--direction=ul)"));
    if (sendDown && sendUp)
    {
        Setting("", "(the two talk independently: both at once, or both silent, happens)");
    }
    Setting("far end",
            farUe ? "terrestrial UE i for satellite UE i, in a 5G cell of its own (FAR END below); "
                    "the satellite UE's call"
                  : "the remote host, " + Num(hostGwDelayMs, 2) +
                        " ms from the gateway, for every call");
    if (farUe)
    {
        Setting("", "sets when both directions start and end");
    }
    Setting("source",
            std::string(rtp ? "RTP, " : "NGMN generator, ") +
                (voiceFromCodec ? "codec " + Num(codecKbps, 2) + " kbit/s = " +
                                      std::to_string(voiceFrameBits) + " bits per 20 ms frame: "
                                : std::string()) +
                voiceB + " B voice frames");
    if (rtp && voiceFromCodec)
    {
        Setting("",
                "(the bits of the frame alone; with the 10-bit payload header of RFC 4867 it is " +
                    std::to_string((voiceFrameBits + 10 + 7) / 8) + " B: --voicePayload=" +
                    std::to_string((voiceFrameBits + 10 + 7) / 8) + ")");
    }
    if (rtp)
    {
        Setting("silence (DTX)",
                std::to_string(hangoverFrames) + " more voice frames (hangover), then a " + sidB +
                    " B SID at once and every " + std::to_string(sidIntervalMs) + " ms");
        Setting("headers", "IP 20 + UDP 8 + RTP 12 = 40 B on every packet");
    }
    else
    {
        Setting("silence (DTX)",
                "no hangover, a " + sidB + " B SID every " + std::to_string(sidIntervalMs) + " ms");
        Setting("headers", "IP 20 + UDP 8 = 28 B on every packet, no RTP header");
    }
    const uint32_t l2Bytes = (l2Headers == "1+1+3") ? 5 : 6;
    std::string belowMac; // size table only: what a packet is below the MAC
    if (!rohc)
    {
        Setting("ROHC", "off");
    }
    else if (rohcModel == "table")
    {
        Ptr<NrPdcp> probe = CreateObject<NrPdcp>(); // has the values the command line may have set
        auto valueOf = [&probe](const std::string& attribute) {
            UintegerValue v;
            probe->GetAttribute(attribute, v);
            return static_cast<uint32_t>(v.Get());
        };
        const uint32_t uo0 = valueOf("RohcTableUo0Size");
        const uint32_t update = valueOf("RohcTableUpdateSize");
        const uint32_t nackFailures = valueOf("RohcNackFailures");
        // One line each: the sizes, what the context survives, how it is repaired
        Setting("ROHC",
                "size table: UO-0 " + std::to_string(uo0) + " B, UOR-2-EXT-3 " +
                    std::to_string(update) + " B, IR " + std::to_string((rtp ? 40 : 28) - 1) +
                    " B");
        Setting("",
                "RFC 3095 5.3.1.1.1: IR " + std::to_string(rohcIrPackets) +
                    " times in a row (an ACK ends them), a context update " +
                    std::to_string(rohcUpdatePackets) + " times for each change");
        Setting("ROHC context",
                "over " + std::to_string(rohcDynamicWindow) +
                    " packets missing in a row: needs UOR-2-EXT-3 or IR; over " +
                    std::to_string(rohcStaticWindow) + ": needs IR");
        if (rohcDynamicWindow == 13 && rohcStaticWindow == 61)
        {
            Setting("",
                    "(UO-0 has 4 sequence number bits, UOR-2 6; with p = 1, RFC 3095 5.7, that is "
                    "2^4 - 3 and 2^6 - 3)");
        }
        if (rohcMode == "O")
        {
            Setting("ROHC repair",
                    "O-mode: NACK or STATIC-NACK on the reverse link " +
                        (nackFailures == 1 ? std::string("at the first packet it cannot restore")
                                           : "after " + std::to_string(nackFailures) +
                                                 " packets in a row it cannot restore") +
                        ", again every " + std::to_string(valueOf("RohcFeedbackSpacing")));
            Setting("",
                    "such packets (none for lost packets the window covers; RFC 3095 5.3.2.2.3, k_1 of "
                    "the last n_1 = " +
                        std::to_string(nackFailures) + " of " + std::to_string(nackFailures) + ")");
        }
        else
        {
            Setting("ROHC repair",
                    "U-mode: no feedback; IR again after " + std::to_string(valueOf("RohcIrRefresh")) +
                        " packets, UOR-2-EXT-3 after " + std::to_string(valueOf("RohcFoRefresh")));
        }
        auto sum = [&](uint32_t payload) {
            return std::to_string(payload) + " + " + std::to_string(uo0) + " + " +
                   std::to_string(l2Bytes) + " = " + std::to_string(payload + uo0 + l2Bytes) + " B";
        };
        belowMac = "voice " + sum(voicePayload) + ", SID " + sum(sidPayload) + ", " +
                   std::to_string(update - uo0) + " B more on a context update";
    }
    else
    {
        Setting("ROHC",
                "ROHC library, " + rohcMode + "-mode" +
                    (rohcMode == "O" ? " (feedback in PDCP control PDUs)" : " (no feedback)"));
    }
    Setting("layer 2",
            l2Headers == "1+1+3"
                ? "PDCP 1 + RLC 1 + MAC 3 = 5 B, as TR 26.940 5.2.3 counts them; NR has the same "
                  "5 B as PDCP 2 + RLC 1 + MAC 2"
                : "PDCP 2 + RLC 2 + MAC 2 = 6 B (the nr module's own)");
    if (!belowMac.empty())
    {
        Setting("below the MAC", belowMac);
    }
    // The voice bearer and what the sender does with a packet it cannot send in time: the RLC
    // (NrRlcUm) does not take a new packet while the oldest one in its queue has waited longer
    // than its discard timer. That timer is DiscardTimerMs or, if that is 0, what is left of
    // the delay budget of the bearer after the way from the UPF (DelayBudgetSpentMs); the nr
    // module tells the budget to the RLC of the gNB only.
    const bool rlcDiscards = AttributeDefault<BooleanValue>("ns3::NrRlcUm", "EnablePdcpDiscarding");
    const uint64_t rlcDiscardMs = AttributeDefault<UintegerValue>("ns3::NrRlcUm", "DiscardTimerMs");
    const double voicePdbMs = voicePdb.GetSeconds() * 1000.0;
    const double voiceTalkKbps = (voicePayload + (rtp ? 40 : 28)) * 8 * 50 / 1000.0;
    Setting("bearer",
            std::string(farUe ? "5QI 1 for every UE, satellite and terrestrial"
                              : "5QI 1 for every UE") +
                ": GBR, GFBR " + Num(gfbrKbps, 2) + " kbit/s each way (talking: " +
                Num(voiceTalkKbps, 2) + "), " + Num(voicePer * 100.0, 0) + " % error rate");
    Setting("delay budget",
            Num(voicePdbMs, 0) + " ms UE-UPF, of which UPF-gateway takes " + Num(upfGwDelayMs, 2) +
                " ms in this run: " + Num(voicePdbMs - upfGwDelayMs, 2) +
                " ms are left for the radio side, gateway-UE");
    Setting("",
            "(TS 23.501 5.7.3.4 takes a fixed 20 ms off for 5QI 1: --upfGwDelayMinMs=20 "
            "--upfGwDelayMaxMs=20)");
    if (farUe)
    {
        Setting("",
                "far end: the same 100 ms terrestrial UE-UPF, of which far gNB-UPF takes " +
                    Num(farEndUpfDelayMs, 2) + " ms (--farEndUpfDelayMs)");
    }
    if (!rlcDiscards)
    {
        Setting("late packets", "no sender discards a packet for waiting too long");
    }
    else if (rlcDiscardMs > 0)
    {
        Setting("late packets",
                "gNB and UE discard a new packet while the oldest in the queue has waited over " +
                    std::to_string(rlcDiscardMs) + " ms");
    }
    else
    {
        Setting("late packets",
                "the gNB discards a new packet while its oldest has waited over " +
                    Num(std::floor(std::max(0.0, voicePdbMs - upfGwDelayMs)), 0) +
                    " ms (that budget, in whole ms); a UE never does" +
                    (farUe ? "\n" + std::string(18, ' ') + "far gNB: the same, over " +
                                 Num(std::floor(std::max(0.0, voicePdbMs - farEndUpfDelayMs)), 0) +
                                 " ms"
                           : std::string()));
    }
    Setting("admission",
            admission
                ? "a call starts only if all calls, with it, reserve at most " +
                      Num(admissionRbPct, 0) + " % of the RBs each way" +
                      (pdcch ? ", " + Num(admissionCcePct, 0) + " % of the CCEs" : std::string()) +
                      " (talking)"
                : std::string("none: every call starts (--admission)"));
    Setting("release",
            dropCalls ? "a call is released when under " + Num(dropOnTimePct, 0) +
                            " % of its packets in a direction make the PDB over 2 s (radio losses "
                            "aside)"
                      : std::string("none: no call is released for its delays (--dropCalls)"));

    // ----- the cell
    std::cout << "\nCELL\n";
    Setting("satellite",
            Num(altitudeKm, 0) + " km, " + (satAsInTable ? "" : "changed from ") + "Set-" +
                std::to_string(satSet) + " " + satColumn.orbit + " of TR 38.821: EIRP " +
                Num(satEirpDbwPerMHz, 2) + " dBW/MHz = " + Num(satEirpDbm, 2) + " dBm, antenna " +
                Num(satAntennaGainDbi, 2) + " dBi,");
    Setting("",
            "G/T " + Num(satGtDbPerK, 2) + " dB/K (noise figure " + Num(satNoiseFigureDb, 2) +
                " dB)");
    if (beamWidthGiven)
    {
        Setting("beam", Num(beamHpbwDeg, 4) + " deg wide at 3 dB on both carriers (--beamHpbwDeg)");
    }
    else
    {
        Setting("beam",
                Num(satApertureM, 2) + " m aperture: " + Num(BeamWidthDegOfKa(g_scene.beamKa[0]), 4) +
                    " deg wide at 3 dB on the downlink, " +
                    Num(BeamWidthDegOfKa(g_scene.beamKa[1]), 4) + " deg on the uplink carrier");
    }
    {
        // The power of the beam: on the RBs in use only, and how much more an RB may get
        const double satTxDbm = satEirpDbm - satAntennaGainDb;
        const double perRbDb = 10.0 * std::log10(carrierRbs);
        Setting("DL power",
                Num(satTxDbm, 2) + " dBm into the antenna for " + std::to_string(carrierRbs) +
                    " RB: " + Num(satTxDbm - perRbDb, 2) + " dBm (EIRP " +
                    Num(satEirpDbm - perRbDb, 2) +
                    " dBm) on each RB in use; an idle RB sends nothing");
        if (dlBoostMaxDb > 0.0)
        {
            Setting(
                "",
                "an RB in use also takes the power of the idle RBs of its symbol, up to +" +
                    Num(dlBoostMaxDb, 2) + " dB = " + Num(satTxDbm - perRbDb + dlBoostMaxDb, 2) +
                    " dBm (--dlBoostMaxDb)" + (tdma ? "; TDMA: no RB of a symbol is idle" : ""));
        }
        if (dlPowerRefDeg > 0.0)
        {
            // what the control takes off with the satellite at an elevation above the beam centre
            auto cutDb = [&](double elevationDeg) {
                const double lossDb =
                    FeederLinkLossModel::FreeSpaceDb(dlFrequencyHz, slantM(elevationDeg)) +
                    dlCarrier.serviceExtra->AtmosphericDb(elevationDeg);
                return std::max(0.0, g_scene.dlRefLossDb - lossDb);
            };
            Setting("DL control",
                    "all of it with the satellite at " + Num(dlPowerRefDeg, 1) +
                        " deg over the beam centre; higher, less by what free space and gases");
            Setting("",
                    "are less (--dlPowerRefDeg): -" + Num(cutDb(90.0), 2) + " / -" +
                        Num(cutDb(std::max(30.0, dlPowerRefDeg)), 2) + " dB at 90 / " +
                        Num(std::max(30.0, dlPowerRefDeg), 0) + " deg; every 100 ms when it flies");
        }
    }
    Setting("handheld",
            Num(ueTxPowerDbm, 0) + " dBm, antenna " + Num(ueAntennaGainDbi, 2) +
                " dBi on both carriers (--ueAntennaGainDbi), noise figure " +
                Num(ueNoiseFigureDb, 0) + " dB");
    Setting("UL grant",
            ulGrantRbs > 0 ? std::to_string(ulGrantRbs) + " RB = " + Num(ulGrantRbs * 180.0, 0) +
                                 " kHz at most per UE and slot, with all its power in them "
                                 "(TR 38.821 Table 6.1.3.2-1: 360 kHz)"
                           : std::string("no limit (--ulGrantRbs=0): a UE spreads its power "
                                         "over the RBs it gets"));
    if (puschRepetitions == 1)
    {
        Setting("repetition",
                "none: a UE sends each uplink block in one slot (--puschRepetitions)");
    }
    else
    {
        const std::string k = puschRepetitions == 0 ? "K" : std::to_string(puschRepetitions);
        Setting("repetition",
                "PUSCH repetition type A (TS 38.214 6.1.2.1): every uplink block and HARQ "
                "retransmission in " +
                    k + " slots in a row,");
        if (puschRepetitions == 0)
        {
            Setting("",
                    "combined by the gNB; K per UE and grant: the fewest of 1, 2, 3, 4, 7, 8, 12, "
                    "16, ..., 32 that bring K x its SINR");
            Setting("",
                    "to " + Num(puschRepSinrDb, 2) +
                        " dB (--puschRepSinrDb); the MCS: what the error model decodes after the "
                        "K transmissions");
        }
        else
        {
            Setting("",
                    "combined by the gNB before it decodes" +
                        std::string(ulLinkAdaptation
                                        ? "; the MCS: what the error model decodes after them"
                                        : " (the MCS is that of one transmission)"));
        }
        if (scheduling != "dynamic")
        {
            Setting("",
                    "--scheduling=" + scheduling +
                        ": a configured grant repeats each block too, repK = 1, 2, 4 or 8 times "
                        "(TS 38.214 6.1.2.3)");
        }
    }
    if (ulPowerRefDeg > 0.0)
    {
        Setting("UL control",
                "open loop, TS 38.213 7.1.1 with alpha = 1: " + Num(ueTxPowerDbm, 0) +
                    " dBm for a UE whose path loss is that on the beam centre");
        Setting("",
                "at " + Num(ulPowerRefDeg, 1) + " deg (" + Num(g_scene.ulRefLossDb, 2) +
                    " dB), less by what its own is lower, never more (--ulPowerRefDeg)");
    }
    if (randomLayout)
    {
        Setting("UEs", "UE 0 on the beam centre, the others at random places of the beam area");
        Setting("beam area",
                Num(areaNorthSouthKm, 1) + " km north-south x " + Num(areaEastWestKm, 1) +
                    " km east-west, " + Num(beamAreaKm2, 0) + " km2");
    }
    else
    {
        Setting("UEs", "in a line, UE i is i x " + Num(spacingKm, 2) + " km east of the beam centre");
    }
    {
        // The four losses of the service link (see the top of the file), and what they give UE 0
        // in line of sight (the feeder link's noise, which takes off 0.01 dB or less, left out)
        auto freeSpaceDb = [&](bool downlink, double elevationDeg) {
            const NtnCarrier& c = downlink ? dlCarrier : ulCarrier;
            return 32.45 + 20.0 * std::log10(c.serviceExtra->Frequency() / 1e9) +
                   20.0 * std::log10(slantM(elevationDeg));
        };
        const double shadowLossDb = fixedShadowing ? shadowingDb : 0.0;
        const double fadingLossDb = fixedFastFading ? fastFadingDb : 0.0;
        const uint32_t cnrRbs = ulGrantRbs > 0 ? ulGrantRbs : 2; // TR 38.821: 360 kHz
        auto losCnrDb = [&](bool downlink, double elevationDeg) {
            const NtnCarrier& c = downlink ? dlCarrier : ulCarrier;
            const double pathLossDb = freeSpaceDb(downlink, elevationDeg) + shadowLossDb +
                                      c.serviceExtra->AtmosphericDb(elevationDeg) +
                                      c.serviceExtra->ScintillationDb() + linkMarginDb +
                                      fadingLossDb;
            return downlink
                       ? satEirpDbwPerMHz - 30.0 + ueAntennaGainDbi - pathLossDb -
                             (-174.0 + ueNoiseFigureDb)
                       : ueTxPowerDbm - 10.0 * std::log10(cnrRbs * 12 * 15e3) + ueAntennaGainDbi +
                             satAntennaGainDbi - pathLossDb - (-174.0 + satNoiseFigureDb);
        };
        // (for the admission of calls)
        g_ulBelowDlDb = losCnrDb(true, elevation) - losCnrDb(false, elevation);
        g_ue0DlCnrDb = losCnrDb(true, elevation);
        auto three = [&](const std::function<double(double)>& f) {
            return Num(f(90), 2) + " / " + Num(f(30), 2) + " / " + Num(f(10), 2) + " dB";
        };
        const std::string freeSpace = three([&](double e) { return freeSpaceDb(true, e); });
        Setting("1 free space",
                "TR 38.811 (6.6-2): downlink " + freeSpace + " at 90 / 30 / 10 deg");
        Setting("2 blocking",
                alwaysLos ? "NTN-LOS: no UE is ever blocked (the TR 38.821 link budget and its "
                            "calibration)"
                          : "a UE is blocked while its obstruction level is above the LOS "
                            "probability (TR 38.811 Table 6.6.1-1)");
        if (!alwaysLos)
        {
            Setting("", "and then has the clutter loss of TR 38.811 Tables 6.6.2-1 to -3");
        }
        Setting("  shadowing",
                fixedShadowing ? Num(shadowingDb, 2) +
                                     " dB for every UE all the time (--shadowingDb; TR 38.821's "
                                     "margin: 3 dB)"
                               : "each UE's level x the sigma of its state and elevation (TR "
                                 "38.811 Tables 6.6.2-1 to -3)");
        const double scintillationDl = dlCarrier.serviceExtra->ScintillationDb();
        const double scintillationUl = ulCarrier.serviceExtra->ScintillationDb();
        const std::string scintillationIs =
            Num(scintillationDl, 2) +
            (scintillationUl != scintillationDl ? " (UL " + Num(scintillationUl, 2) + ")" : "") +
            " dB";
        if (atmosphereTable)
        {
            Setting("3 atmosphere",
                    "gases " + Num(ServiceLinkLossModel::TABLE_GASES_DB, 2) +
                        " dB, scintillation " + scintillationIs +
                        ", every UE at every elevation (TR 38.821 Table 6.1.3.3-1)");
        }
        else
        {
            const double zenithDl = ServiceLinkLossModel::ZenithAttenuationDb(dlFrequencyHz);
            const double zenithUl = ServiceLinkLossModel::ZenithAttenuationDb(ulFrequencyHz);
            Setting("3 atmosphere",
                    "gases " + Num(zenithDl, 3) +
                        (zenithUl != zenithDl ? " (UL " + Num(zenithUl, 3) + ")" : "") +
                        " dB / sin(elevation) (TR 38.811 (6.6-8)); scintillation " +
                        scintillationIs + " at " + Num(latitudeDeg, 1) + " deg");
            Setting("",
                    "magnetic latitude (TR 38.811 6.6.6.1.4: within 20 deg of the magnetic "
                    "equator only)");
        }
        Setting("4 fast fading",
                fixedFastFading
                    ? Num(fastFadingDb, 2) + " dB on every RB: the direct path alone, no scattered "
                                             "paths (--fastFadingDb)"
                    : "3GPP channel per RB and slot: TR 38.901 7.5 with TR 38.811's " +
                          environment + " tables");
        if (linkMarginDb != 0.0)
        {
            Setting(
                "  margin",
                Num(linkMarginDb, 2) +
                    " dB more on every link (--linkMarginDb; TR 38.821's additional losses: 0)");
        }
        Setting("CNR",
                std::string("UE 0 in line of sight") +
                    (fixedShadowing ? "" : ", shadowing level 0") + ": downlink " +
                    three([&](double e) { return losCnrDb(true, e); }) + " at 90 / 30 / 10 deg,");
        Setting("",
                "uplink with " + Num(ueTxPowerDbm, 0) + " dBm on " + std::to_string(cnrRbs) +
                    " RB " + three([&](double e) { return losCnrDb(false, e); }) +
                    (fixedFastFading
                         ? " (the feeder link's noise not in it)"
                         : " (fast fading averaged; the feeder link's noise not in it)"));
        if (sendUp && ulGrantRbs > 0)
        {
            // What the narrow uplink grant carries for the best UE, UE 0, when its call starts:
            // the MCS the gNB's AMC gives that SINR (as the scheduler, before its outer loop),
            // and the transport block of the grant over the 13 data symbols of a slot
            Ptr<NrAmc> amc = CreateObjectWithAttributes<NrAmc>(
                "ErrorModelType",
                TypeIdValue(TypeId::LookupByName("ns3::" + errorModel)));
            SpectrumValue sinr(
                NrSpectrumValueHelper::GetSpectrumModel(carrierRbs, ulFrequencyHz, 15e3));
            const double ue0Db = losCnrDb(false, elevation);
            for (uint32_t rb = 0; rb < ulGrantRbs && rb < carrierRbs; ++rb)
            {
                sinr[rb] = std::pow(10.0, ue0Db / 10.0);
            }
            // (PUSCH repetition: K as the scheduler takes it, and the MCS after K transmissions)
            uint32_t k = std::max<uint32_t>(puschRepetitions, 1);
            if (puschRepetitions == 0)
            {
                k = 32;
                for (const uint32_t n : {1, 2, 3, 4, 7, 8, 12, 16, 20, 24, 28, 32})
                {
                    if (ue0Db + 10.0 * std::log10(n) >= puschRepSinrDb)
                    {
                        k = n;
                        break;
                    }
                }
            }
            uint8_t mcs = 0;
            amc->CreateCqiFeedbackSiso(sinr, mcs, ulLinkAdaptation ? k : 1);
            const uint32_t tbBytes = amc->CalculateTbSize(mcs, 1, ulGrantRbs * 13);
            Setting(
                "",
                "UE 0 at " + Num(elevation, 1) + " deg: " + Num(ue0Db, 2) + " dB on its " +
                    std::to_string(ulGrantRbs) + " RB give " +
                    (k > 1 ? "K = " + std::to_string(k) + ", " : std::string()) + "MCS " +
                    std::to_string(mcs) + " and a block of " + std::to_string(tbBytes) + " B " +
                    (k > 1 ? "every " + std::to_string(k) + " slots" : std::string("per slot")) +
                    (tbBytes < 12 ? " (too small: see the NOTE below)" : ""));
            ulBlockTooSmall = tbBytes < 12;
            ulBlockBytes = tbBytes;
        }
    }
    if (transparent)
    {
        const uint32_t feederCarriers =
            static_cast<uint32_t>(std::floor(feederBandwidthMHz / bandwidthMHz + 1e-9));
        const double fillW =
            std::pow(10.0, (gatewayDbwPerMHz + 10.0 * std::log10(feederBandwidthMHz)) / 10.0);
        Setting("feeder link",
                "gateway -> satellite " + Num(feederFrequencyGHz, 2) + " GHz: antenna " +
                    Num(gatewayGainDbi, 2) + " dBi, " + Num(gatewayDbwPerMHz, 2) +
                    " dBW/MHz into it = EIRP " + Num(gatewayEirpDbwPerMHz, 2) + " dBW/MHz;");
        Setting("",
                "satellite G/T " + Num(satFeederGtDbPerK, 2) + " dB/K; free space only: C/N " +
                    Num(feederCnrDb(90), 2) + " / " + Num(feederCnrDb(30), 2) + " / " +
                    Num(feederCnrDb(10), 2) + " dB at 90 / 30 / 10 deg above the gateway");
        Setting("",
                Num(feederBandwidthMHz, 0) + " MHz: " + std::to_string(feederCarriers) +
                    " carriers of " + Num(bandwidthMHz, 0) + " MHz, all at that density with " +
                    Num(fillW, 1) + " W into the antenna; uplink: delay only");
    }
    // The scheduler by its TypeId: ns3::NrMacScheduler + Tdma or Ofdma + the policy
    std::string schedulerIs = scheduler;
    {
        static const std::map<std::string, std::string> policies = {
            {"RR", "round robin"},
            {"PF", "proportional fair"},
            {"MR", "highest MCS first"},
            {"Qos", "proportional fair weighted by the priority of the 5QI"},
            {"Random", "random order"}};
        const std::string prefix = "ns3::NrMacScheduler";
        for (const std::string access : {"Tdma", "Ofdma"})
        {
            if (scheduler.rfind(prefix + access, 0) == 0)
            {
                const auto policy = policies.find(scheduler.substr(prefix.size() + access.size()));
                schedulerIs = (access == "Tdma" ? "TDMA (a UE gets whole symbols), "
                                                : "OFDMA (a UE gets resource block groups), ") +
                              (policy == policies.end() ? scheduler : policy->second);
            }
        }
    }
    Setting("interference", "none: one beam, no other cell; every SINR here is a signal-to-noise ratio");
    Setting("scheduler", schedulerIs);
    const bool table2 = errorModel.back() == '2';
    Setting("error model",
            errorModel + (errorModel == "NrLteMiErrorModel"
                              ? ": LTE's MCS table (QPSK to 64QAM), turbo code, mutual "
                                "information over the RBs (MIESM)"
                              : std::string(": NR MCS Table ") + (table2 ? "2" : "1") +
                                    " (TS 38.214 5.1.3.1-" + (table2 ? "2" : "1") +
                                    (table2 ? ": QPSK to 256QAM" : ": QPSK to 64QAM") +
                                    "), LDPC, EESM over the RBs"));
    Setting("HARQ",
            harq ? "up to 4 transmissions of a block, combined by the receiver (" +
                       std::string(chaseCombining ? "chase combining" : "incremental redundancy") +
                       "); " + std::to_string(harqProcesses) + " processes per UE"
                 : std::string("off: every block is sent once, a block that fails is lost"));
    if (sendDown)
    {
        Setting("link adaptation",
                "downlink: a first transmission aims at " + Plain(targetBler * 100.0) +
                    " % block errors (outer loop on the HARQ feedback)");
    }
    if (sendUp && ulLinkAdaptation)
    {
        Setting(sendDown ? "" : "link adaptation",
                "uplink: the SINR measured on the UE's last transmission, scaled to the resource "
                "blocks of the grant");
        Setting("",
                "(the UE spreads its power over them); a first transmission aims at " +
                    Plain(targetBler * 100.0) + " % block errors (outer loop on decoding)");
    }
    else if (sendUp)
    {
        Setting(sendDown ? "" : "link adaptation",
                "uplink: the MCS follows the SINR the gNB measured on the UE's last transmission; "
                "no outer loop, no error target");
    }
    if (sendUp)
    {
        // (the grant the gNB sends for a scheduling request)
        Setting(
            "SR grant",
            std::to_string(srGrantBytes) + " B" +
                (srGrantBytes >= voiceSrBytes
                     ? std::string(": a voice frame with its headers and a buffer status report")
                     : ", less than a voice frame with its headers and a buffer status report (" +
                           std::to_string(voiceSrBytes) + " B): the frame needs a second grant"));
    }
    if (scheduler.find("Ofdma") != std::string::npos &&
        AttributeDefault<BooleanValue>("ns3::NrMacSchedulerOfdma", "HarqRetxInFrequency"))
    {
        Setting("",
                "OFDMA: a HARQ retransmission takes resource block groups of the symbols new data "
                "has, which other UEs share");
    }
    if (scheduling != "dynamic")
    {
        Setting("",
                "a semi-persistent block keeps the MCS of its activation, so its error rate is "
                "not that target's;");
        Setting("",
                "to compare the schemes without that, give all of them a low one: "
                "--targetBler=0.01");
    }
    // The PDCCH as a resource: the settings of the cell, in an NrPdcch of the same attributes
    const uint32_t dlCarrierRbs = carrierRbs; // (the PHY knows its own only once it has started)
    const uint16_t cellId = gnbDevs.Get(0)->GetObject<NrGnbNetDevice>()->GetCellId();
    const std::array<uint8_t, 5> pdcchLevels = {1, 2, 4, 8, 16};
    Ptr<NrPdcch> pdcchView;
    const bool rlmFromTable =
        pdcch && AttributeDefault<BooleanValue>("ns3::NrUePhy", "RlmFromPdcchTable");
    if (pdcch)
    {
        pdcchView = CreateObject<NrPdcch>();
        pdcchView->SetCarrier(dlCarrierRbs, 0);
    }
    if (!srFallback)
    {
        Setting("SR",
                "a UE asks for a grant until it gets one: no random access after sr-TransMax "
                "(this module's would give it a new");
        Setting("", "RNTI and stop its downlink; TS 38.321 5.1.5 keeps the C-RNTI)");
    }
    if (idealControl)
    {
        Setting("control", "ideal: no scheduling message is ever lost");
        Setting("link failure",
                "after " + std::to_string(n310 * qOutEvalSf) + " control symbols below " +
                    Num(qOutDb, 2) + " dB and " + Num(t310.GetSeconds(), 2) +
                    " s more; the UE attaches again if its RSRP is over -140 dBm");
    }
    else if (rlmFromTable)
    {
        // TS 38.133 8.1.2: the two hypothetical PDCCHs of the radio link monitoring
        Setting("control",
                "a DCI is lost at the block error rate of its aggregation level at the SINR it "
                "arrives with");
        Setting("link failure",
                "when a level-8 PDCCH with 4 dB more power would lose over 10 % (below " +
                    Num(pdcchView->GetSinrForBler(8, 0.10) - 4.0, 2) + " dB), over the last " +
                    std::to_string(qOutEvalSf) + " ms,");
        Setting("",
                "and a level-4 PDCCH not under 2 % again (above " +
                    Num(pdcchView->GetSinrForBler(4, 0.02), 2) + " dB) within " +
                    Num(t310.GetSeconds(), 2) + " s");
        Setting("after a failure",
                "no service until the level-4 PDCCH loses under 2 % again, then the UE attaches again");
    }
    else
    {
        Setting("control",
                "scheduling messages are lost at low SINR: 10 % at " + Num(qOutDb, 2) +
                    " dB (Qout), 2 % at " + Num(qInDb, 2) + " dB (Qin)");
        Setting("link failure",
                "when that rate, over the last " + std::to_string(qOutEvalSf) +
                    " ms, is above 10 % and not back below 2 % within " +
                    Num(t310.GetSeconds(), 2) + " s");
        Setting("after a failure",
                "no service until the rate is below 2 % again, then the UE attaches again");
    }

    // ----- the PDCCH as a resource
    if (pdcch)
    {
        const double target = AttributeDefault<DoubleValue>("ns3::NrPdcch", "TargetBler");
        const double margin = AttributeDefault<DoubleValue>("ns3::NrPdcch", "SinrMarginDb");
        const double ulShare = AttributeDefault<DoubleValue>("ns3::NrPdcch", "UlMaxShare");
        const uint32_t sib1Ms = AttributeDefault<UintegerValue>("ns3::NrPdcch", "Sib1PeriodMs");
        std::cout << "\nPDCCH\n";
        const std::string coresetRbs = std::to_string(pdcchView->GetCoresetRbs()) + " RB x ";
        auto cces = [&](uint32_t symbols) { return std::to_string(pdcchView->GetNumCce(symbols)); };
        Setting("CORESET",
                (pdcchSymbols > 0
                     ? coresetRbs + std::to_string(pdcchSymbols) + " symbol" +
                           (pdcchSymbols > 1 ? "s" : "") + " = " + cces(pdcchSymbols) + " CCEs"
                     : coresetRbs + "1, 2 or 3 symbols = " + cces(1) + ", " + cces(2) + " or " +
                           cces(3) + " CCEs as the slot needs") +
                    "; a DCI is " + std::to_string(dciBits) + " + 24 CRC bits");
        {
            // DCI format 1_0 with C-RNTI: 28 bits and the frequency allocation field
            // (TS 38.212 7.3.1.2.1)
            const uint32_t n = dlCarrierRbs;
            const auto fdra = static_cast<uint32_t>(std::ceil(std::log2(n * (n + 1) / 2.0)));
            if (dciBits != 28 + fdra)
            {
                Setting("",
                        "(format 1_0 on " + std::to_string(n) + " RB has " +
                            std::to_string(28 + fdra) + " bits, TS 38.212 7.3.1.2.1: --dciBits=" +
                            std::to_string(28 + fdra) + ")");
            }
            if (pdcchSymbols > 0 && pdcchView->GetNumCce(pdcchSymbols) < 8)
            {
                Setting("",
                        "NOTE: 3GPP has no CORESET 0 of 24 RB x 1 symbol (TS 38.213 Table 13-1: 2 "
                        "or 3); SIB1 is at level 4 here");
            }
        }
        std::string from;
        std::string candidates;
        std::string levels;
        for (const uint8_t al : pdcchLevels)
        {
            if (pdcchView->GetCandidates(al) == 0 ||
                al > pdcchView->GetNumCce(pdcchView->GetMaxSymbols()))
            {
                continue;
            }
            const bool first = from.empty();
            from += (first ? "level " : ", ") + std::to_string(al) + " from " +
                    Num(pdcchView->GetSinrForBler(al, target) + margin, 2) + " dB";
            candidates += (first ? "" : "/") + std::to_string(pdcchView->GetCandidates(al));
            levels += (first ? "" : "/") + std::to_string(al);
        }
        Setting("level",
                "the lowest that loses up to " + Num(100.0 * target, 2) +
                    " % of the DCIs at the SINR the UE reported" +
                    (margin != 0.0 ? " less " + Num(margin, 2) + " dB" : std::string()) + ":");
        Setting("", from);
        StringValue blerTable;
        pdcchView->GetAttribute("BlerTable", blerTable);
        Setting("", "level-1 BLER, dB:BLER (--pdcchBlerTable): " + blerTable.Get());
        const uint32_t monitored = pdcchView->GetMonitoredCandidates() * (pdcchSymbols > 0 ? 1 : 3);
        Setting("candidates",
                candidates + " per UE at level " + levels + "; common search space at level " +
                    std::to_string(pdcchView->GetCommonAl()) + "; a UE monitors " +
                    std::to_string(monitored) + " (limit " +
                    std::to_string(NrPdcch::GetMaxMonitoredCandidates(0)) + ")");
        // The DCIs the cell sends of its own: system information, paging, DCI 2_2, RRC
        {
            auto text = [&](const std::string& attribute) {
                StringValue v;
                pdcchView->GetAttribute(attribute, v);
                return v.Get();
            };
            auto number = [&](const std::string& attribute) {
                UintegerValue v;
                pdcchView->GetAttribute(attribute, v);
                return static_cast<uint32_t>(v.Get());
            };
            DoubleValue pagingRate;
            pdcchView->GetAttribute("PagingRate", pagingRate);
            const std::string si = text("SiPeriodsMs");
            const uint32_t pagingFrames = number("PagingFrames");
            const uint32_t tpcMs = number("TpcPeriodMs");
            Setting("cell's DCIs",
                    "SIB1 every " + std::to_string(sib1Ms) + " ms; SI messages every " +
                        (si.empty() ? std::string("- (none)") : si) + " ms; paging " +
                        Plain(pagingRate.Get()) + " pages/s, an occasion every " +
                        std::to_string(pagingFrames == 0 ? 10
                                                         : number("PagingCycleMs") / pagingFrames) +
                        " ms");
            Setting("",
                    tpcMs > 0 ? "DCI 2_2 (power control) every " + std::to_string(tpcMs) +
                                    " ms, one per " + std::to_string(number("TpcUesPerDci")) +
                                    " UEs with SPS or configured grants" +
                                    (spsScenario == 0 ? std::string(": none here, all dynamic")
                                                      : std::string())
                              : std::string("DCI 2_2 (power control): none"));
            const uint32_t measMs = number("MeasReportPeriodMs");
            Setting("",
                    "RRC: " + text("RrcSetupDcis") +
                        " DCIs (downlink:uplink) when a UE connects, " + text("RrcCallDcis") +
                        " when a call starts, one every " + std::to_string(number("RrcStepMs")) +
                        " ms");
            Setting("",
                    (measMs > 0 ? "a measurement report every " + std::to_string(measMs) + " ms; "
                                : std::string()) +
                        "an RRC message takes CCEs here, not RBs");
        }
        Setting("order in a slot",
                "the cell's DCIs (those that found no room before first), random access, uplink "
                "grants");
        Setting("",
                "(at most " + Num(100.0 * ulShare, 0) +
                    " % of the CCEs while the downlink wants them too), downlink assignments;");
        Setting("", "retransmissions before new data, new data as the scheduler ranks the UEs");
        // The voice bearers: a DCI for every block, or semi-persistent scheduling
        const uint32_t confirmBytes = spsScenario == 2 ? 6 : 1;
        const uint32_t ulMore = (spsScenario == 2 ? spsFitExtraBytes : 0) + 5 + confirmBytes;
        const std::string talkB = std::to_string(spsTbBytes);
        const std::string silenceB = std::to_string(spsSilenceTbBytes);
        const std::string extraB = std::to_string(spsFitExtraBytes);
        const std::string firstTalkB = std::to_string(spsTbBytes + spsFitExtraBytes);
        const std::string firstSidB = std::to_string(spsSilenceTbBytes + spsFitExtraBytes);
        if (spsScenario == 0)
        {
            Setting("voice", "dynamic scheduling: every block has a DCI");
        }
        else if (spsScenario == 1)
        {
            Setting("voice",
                    "semi-persistent, scenario 1: one configuration, a block every 20 ms, kept "
                    "through silence (blocks stay empty)");
            Setting("", "downlink (SPS): " + talkB + " B blocks");
            Setting("",
                    "uplink (configured grant Type 2): " + std::to_string(spsTbBytes + ulMore) +
                        " B blocks = " + talkB + " B + 5 B BSR (nr module; 3GPP: 2 B) + 1 B confirmation CE");
            if (spsFitExtraBytes > 0)
            {
                Setting("to silence",
                        "the first SID has a context update, " + firstSidB +
                            " B: the RLC cuts it, in this block and the next (20 ms later); "
                            "no DCI");
                Setting("to voice",
                        "so has the first voice frame, " + firstTalkB +
                            " B: it gets a DCI of its own; downlink in place of the block,");
                Setting("", "uplink for what the block left of it");
            }
            else
            {
                Setting("to silence", "the first SID is no larger than a block: nothing to do");
                Setting("to voice", "nor is the first voice frame");
            }
        }
        else
        {
            Setting("voice",
                    "semi-persistent, scenario 2: talk: a block every 20 ms; silence: every " +
                        std::to_string(sidIntervalMs) + " ms; a change is 2 DCIs");
            Setting("",
                    "downlink (SPS): " + talkB + " B and " + silenceB + " B blocks, up to " + extraB +
                        " B larger for the first packet after a change");
            Setting("",
                    "uplink (configured grant Type 2): " + std::to_string(spsTbBytes + ulMore) +
                        " B and " + std::to_string(spsSilenceTbBytes + ulMore) + " B blocks = " +
                        talkB + " B and " + silenceB + " B + " + extraB + " B for a first packet");
            Setting("", "+ 5 B BSR (nr module; 3GPP: 2 B) + 6 B confirmation MAC CE");
            Setting("to silence",
                    "downlink: at the first SID (" + firstSidB +
                        " B) 1 DCI releases the talk configuration, 1 activates the silence one "
                        "with");
            Setting("",
                    "the SID in its first block. Uplink: the first SID comes in a talk block, the "
                    "2 DCIs one silence period later");
            Setting("", "(none if the speaker talks again before that)");
            Setting("to voice",
                    "downlink: the same at the first voice frame (" + firstTalkB +
                        " B). Uplink: its scheduling request gets a dynamic grant");
            Setting("", "(1 DCI), then the 2 DCIs: the talk configuration starts with the next frame");
        }
    }

    // ----- the far end's cells, --farEnd=ue
    if (farUe)
    {
        std::cout << "\nFAR END (--farEnd=ue)\n";
        Setting("UEs",
                std::to_string(nUes) +
                    " terrestrial UEs, one per call: UE i talks with satellite " + "UE i (--nUes)");
        Setting("cells",
                std::to_string(farCells) + (farCells == 1 ? " gNB" : " gNBs, 100 km apart") +
                    ", one per " + std::to_string(farUesPerCell) +
                    " calls (--farEndUesPerGnb); each on a channel of its own:");
        Setting("", "no interference between them, none with the satellite's carriers");
        Setting("band",
                farEndBand +
                    (farTdd ? ", TDD " + Num(farDlHz / 1e9, 4) + " GHz"
                            : ", FDD DL " + Num(farDlHz / 1e9, 4) + " / UL " +
                                  Num(farUlHz / 1e9, 4) + " GHz") +
                    " (--farEndBand, --farEndFrequencyGHz)");
        Setting("",
                Plain(farEndBandwidthMHz) + " MHz = " + std::to_string(farRbs) + " RB of " +
                    (farTdd ? "30" : "15") +
                    " kHz (--farEndBandwidthMHz; TS 38.101-1 Table 5.3.2-1)" +
                    (farTdd ? "; slots " + farEndTddPattern + " (--farEndTddPattern)" : ""));
        Setting("gNB",
                Num(farGnbHeightM, 0) + " m high, " + Num(farGnbDbm, 2) +
                    " dBm (--farEndGnbDbm: " + Plain(farEndGnbDbm) +
                    " per 20 MHz), a 4 x 8 panel of TR 38.901 elements (8 dBi),");
        Setting("",
                "noise figure 5 dB, one 120 deg sector (power and panel: ITU-R M.2412 Dense "
                "Urban-eMBB)");
        Setting("UE",
                "1.5 m high, " + Num(ueTxPowerDbm, 0) + " dBm, 0 dBi, noise figure " +
                    Num(ueNoiseFigureDb, 0) + " dB; uniformly over the sector, " + Plain(farMinM) +
                    " to " + Plain(farMaxM) + " m from the gNB");
        Setting("", "(--farEndMinDistM, --farEndMaxDistM)");
        Setting("channel",
                "TR 38.901 " + std::string(farChannel == "UMi" ? "UMi street canyon" : farChannel) +
                    (farAlwaysLos ? ", always in line of sight"
                                  : ", line of sight by its probability") +
                    ", with fast fading (--farEndScenario)");
        if (!farTdd)
        {
            Setting(
                "",
                "FDD: the uplink has the downlink's line of sight state and shadowing, and fast "
                "fading of its own");
        }
        Setting("error model",
                farEndErrorModel +
                    (farEndErrorModel == errorModel ? ", as the satellite cell" : "") +
                    " (--farEndErrorModel)");
        Setting("core",
                "gNB - UPF " + Num(farEndUpfDelayMs, 2) +
                    " ms one way (--farEndUpfDelayMs), to the satellite cell's UPF");
        Setting("scheduler", "the satellite cell's (--scheduler)");
        Setting("layer 2",
                "plain NR, whatever the run sets for the satellite cell: no ROHC, PDCP SN 12 and "
                "RLC SN 10 bits,");
        Setting("",
                "HARQ with 16 processes, the nr module's link adaptation, no uplink grant limit, "
                "no PDCCH limit, no SPS");
    }

    // ----- where everything is, and how long the signal takes
    std::cout << "\nGEOMETRY AND DELAY\n";
    // The satellite seen from the gateway at time t: elevation and distance
    auto gatewayAt = [&](Time t) {
        const Vector sat = orbit.At(t);
        const Vector gw = gatewayMobility->GetPosition();
        const bool closer = (orbit.At(t + MilliSeconds(1)) - gw).GetLength() < (sat - gw).GetLength();
        return std::make_tuple(ElevationDeg(sat, gw), (sat - gw).GetLength() / 1000.0, closer);
    };
    const std::string gatewayPlaceIs =
        gatewayAway ? "N " + Num(gatewayPlace.northKm, 1) + " km, E " + Num(gatewayPlace.eastKm, 1) +
                          " km from UE 0 (random in a circle of " + Num(gatewayDiameterKm, 0) +
                          " km diameter)"
                    : "where UE 0 is";
    const std::string otherCalls =
        nUes == 1 ? std::string()
                  : "call i starts i x " + std::to_string(startOffsetMs) + " ms after UE 0's";
    const std::string roundTrip = Num(maxLoopRtt.GetMicroSeconds() / 1000.0, 2) + " ms on the radio";
    const std::string kOffsetIs = "K_offset " + std::to_string(kOffsetSlots) + " slots";
    auto time = [](Time t) { return "t = " + Num(t.GetSeconds(), 2) + " s"; };
    if (freezeGeometry)
    {
        const Ue0View now = ue0At(Seconds(0));
        Setting("satellite", "stands still for the whole run (--freezeGeometry=0: it flies)");
        Setting("calls",
                time(callsStart) + " to " + Num(callsEnd.GetSeconds(), 2) + " s" +
                    (otherCalls.empty() ? "" : "; " + otherCalls + ", all end with it"));
        Setting("UE 0",
                "elevation " + Num(satMobility->GetElevationAngle(centreMobility), 2) +
                    " deg, slant range " + Num(now.rangeKm, 2) + " km");
        if (transparent)
        {
            Setting("gateway", gatewayPlaceIs);
            if (gatewayAway)
            {
                const auto [gwElevationDeg, gwRangeKm, gwCloser] = gatewayAt(Seconds(0));
                Setting("",
                        "elevation " + Num(gwElevationDeg, 2) + " deg, slant range " +
                            Num(gwRangeKm, 2) + " km (10 deg or more for the feeder link)");
            }
        }
        Setting("delay to UE 0", delayParts(now) + ", one way");
        Setting("round trip", roundTrip + " at most  =>  " + kOffsetIs);
    }
    else
    {
        // UE 0, on the beam centre, when its call starts and when it ends
        const Ue0View first = ue0At(callsStart);
        const Ue0View last = ue0At(callsEnd);
        auto signedDeg = [](bool before, double deg) { return (before ? "-" : "+") + Num(deg, 2); };
        Setting("satellite",
                "flies south at " + Num(orbit.vel0.GetLength() / 1000.0, 2) + " km/s, is over UE 0 at " +
                    time(overheadAt) + " and down at 10 deg again at " + time(downAt));
        const bool gatewayCuts = callsStart > passStart || callsEnd < passEnd;
        const Time canLast = Min(downAt, gatewaySees.set) - callsStart;
        Setting(
            "calls",
            "UE 0: " + time(callsStart) + " to " + Num(callsEnd.GetSeconds(), 2) + " s" +
                (fullPass
                     ? (gatewayCuts ? ", the pass while the gateway sees it" : ", the whole pass")
                     : " (it can last " + Num(std::floor(canLast.GetSeconds()), 0) + " s)") +
                (otherCalls.empty() ? "" : "; " + otherCalls + ","));
        if (!otherCalls.empty())
        {
            Setting("",
                    "is on only while the satellite is 10 deg or more above its own UE, and "
                    "ends with UE 0's at the latest");
        }
        if (passIgnored)
        {
            Setting("", "NOTE: --elevation and --duration are not used with --fullPass=1");
        }
        Setting("UE 0",
                "elevation " + signedDeg(callsStart < overheadAt, first.elevationDeg) + " -> " +
                    signedDeg(callsEnd < overheadAt, last.elevationDeg) +
                    " deg (- before it is overhead, + after), slant range " +
                    Num(first.rangeKm, 2) + " -> " + Num(last.rangeKm, 2) + " km");
        if (transparent)
        {
            Setting("gateway", gatewayPlaceIs);
            if (gatewayAway)
            {
                const auto [elevation0, range0, closer0] = gatewayAt(callsStart);
                const auto [elevation1, range1, closer1] = gatewayAt(callsEnd);
                Setting("",
                        "elevation " + signedDeg(closer0, elevation0) + " -> " +
                            signedDeg(closer1, elevation1) + " deg, slant range " + Num(range0, 2) +
                            " -> " + Num(range1, 2) + " km");
                // the feeder link needs the satellite at 10 deg or more above the gateway
                const std::string rule = " (TR 38.821: 10 deg for the feeder link)";
                const bool late = callsStart > passStart;
                const bool early = callsEnd < passEnd;
                Setting("",
                        late && early
                            ? "at 10 deg or more only from " + time(callsStart) + " to " +
                                  Num(callsEnd.GetSeconds(), 2) + " s: no call outside that" + rule
                        : late ? "below 10 deg until " + time(callsStart) +
                                     ": the calls wait for it" + rule
                        : early
                            ? "below 10 deg from " + time(callsEnd) + ": the calls end there" + rule
                            : "at 10 deg or more all through UE 0's call" + rule);
            }
        }
        Setting("delay to UE 0", delayParts(first) + ", one way, when its call starts");
        Setting("", delayParts(last) + " when it ends");
        Setting("round trip",
                roundTrip + " at most, at " + time(maxLoopRttAt) + "  =>  " + kOffsetIs +
                    " for the whole run");
    }
    {
        // The Doppler shift of UE 0's service link, f v_radial / c, and what the UE leaves of it
        const double c = 299792458.0;
        auto rangeRate = [&](Time t) { // m/s, + while the satellite moves away
            const Time d = MilliSeconds(50);
            return ((orbit.At(t + d) - centrePosition).GetLength() -
                    (orbit.At(t - d) - centrePosition).GetLength()) /
                   (2.0 * d.GetSeconds());
        };
        if (freezeGeometry)
        {
            // Flying over UE 0: the speed times the cosine of the angle between orbit and path
            const double earthM = orbitRadiusM - altitudeM;
            const double radial =
                orbitSpeedMps * earthM * std::cos(elevation * M_PI / 180.0) / orbitRadiusM;
            Setting("Doppler",
                    "none: the satellite stands still (flying past " + Num(elevation, 1) +
                        " deg, it would shift DL by " + Num(radial / c * dlFrequencyHz / 1e3, 1) +
                        " kHz, UL by " + Num(radial / c * ulFrequencyHz / 1e3, 1) + " kHz)");
        }
        else
        {
            double maxRateHzPerS = 0.0;
            for (Time t = callsStart; t <= callsEnd; t += MilliSeconds(100))
            {
                const double change =
                    (rangeRate(t + MilliSeconds(50)) - rangeRate(t - MilliSeconds(50))) / 0.1;
                maxRateHzPerS = std::max(maxRateHzPerS, std::abs(change) / c * dlFrequencyHz);
            }
            Setting("Doppler",
                    "UE 0's service link: downlink " +
                        Num(-rangeRate(callsStart) / c * dlFrequencyHz / 1e3, 1) + " -> " +
                        Num(-rangeRate(callsEnd) / c * dlFrequencyHz / 1e3, 1) +
                        " kHz over its call, changing by up to " + Num(maxRateHzPerS, 0) + " Hz/s");
        }
        // TS 38.101-5: 0.1 ppm left after pre-compensation; a frequency error of a fraction e of
        // the subcarrier spacing leaks (pi e)^2 / 3 of the power into the other subcarriers
        const double residualHz = 0.1e-6 * ulFrequencyHz;
        const double share = residualHz / 15e3;
        const double leakDb = -10.0 * std::log10(std::pow(M_PI * share, 2) / 3.0);
        Setting("",
                "the UE takes it off, from its GNSS position and the satellite's ephemeris (TS "
                "38.300 16.14.2);");
        Setting("",
                "left: 0.1 ppm (TS 38.101-5) = " + Num(residualHz, 0) + " Hz in uplink, " +
                    Num(share * 100.0, 1) + " % of a subcarrier, leaking " + Num(leakDb, 0) +
                    " dB below the signal:");
        Setting("",
                "no frequency error in this model" +
                    std::string(fixedFastFading
                                    ? ""
                                    : "; the 3GPP fading has the Doppler spread of the motion"));
    }
    if (ulBlockTooSmall)
    {
        std::cout
            << "\n  NOTE: with " << ulGrantRbs << " RB (--ulGrantRbs) even UE 0's uplink "
            << "block is " << ulBlockBytes
            << " B, and the nr module grants no uplink block "
               "under 12 B.\n"
            << "        A UE gets its first grants (the gNB has not measured it yet), then none: "
               "almost no voice goes up.\n"
            << "        --puschRepetitions=0 repeats each block in as many slots as the UE needs "
               "(PUSCH repetition); --ulGrantRbs=0 lets\n"
            << "        a UE spread its power over more RBs.\n";
    }
    if (sendUp && scheduler.find("Tdma") != std::string::npos)
    {
        std::cout << "\n  NOTE: a TDMA scheduler grants a UE the whole band, which the 23 dBm of a "
                     "handheld cannot fill: every first transmission\n"
                     "        fails and HARQ has to repair it. Expect loss and long delays, or use "
                     "an OFDMA scheduler for uplink calls\n"
                     "        (--scheduler=ns3::NrMacSchedulerOfdmaRR): it grants a few resource "
                     "blocks.\n";
    }
    if (sendUp && !harq)
    {
        std::cout << "\n  NOTE: --harq=0 in uplink: "
                  << (ulLinkAdaptation
                          ? "the outer loop keeps the first transmissions at "
                            "--targetBler, but each of them that fails is lost.\n"
                          : "the uplink has no outer loop that keeps the block errors "
                            "at a target; many first transmissions fail,\n"
                            "        and without HARQ each of them is a lost packet.\n");
    }

    if (pdcch)
    {
        // The PDCCH report covers the calls: from the start of the first to the end of the last
        Simulator::Schedule(callsStart, [cellId]() { NrPdcch::Get(cellId)->ResetStats(); });
        Simulator::Schedule(callsEnd, [cellId]() { NrPdcch::Get(cellId)->SetCounting(false); });
    }

    if (pdcch && !csv.empty())
    {
        // --csv: the DCIs of each UE, over the same window as the PDCCH report
        for (uint32_t i = 0; i < nUes; ++i)
        {
            for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
            {
                Connect(NrHelper::GetUePhy(ueDevs.Get(i), bwp),
                        "UePhyRxedCtrlMsgsTrace",
                        MakeCallback(&UeDciReceived));
            }
        }
        Simulator::Schedule(callsStart, []() { g_dciCounting = true; });
        Simulator::Schedule(callsEnd, []() { g_dciCounting = false; });
    }

    // --- Call control: what a call reserves, and what the gNB does to release one ---------
    {
        const double slotsPerSecond = 1000.0; // 15 kHz
        const uint32_t ulRbs = ulGrantRbs > 0 ? ulGrantRbs : 2;
        g_cc.admission = admission;
        g_cc.load = {admissionRbPct / 100.0, admissionRbPct / 100.0, admissionCcePct / 100.0};
        g_cc.drop = dropCalls;
        g_cc.dropOnTime = dropOnTimePct / 100.0;
        g_cc.window = voiceAveragingWindow;
        g_cc.pdb = voicePdb;
        g_cc.capacity = {carrierRbs * slotsPerSecond,
                         carrierRbs * slotsPerSecond,
                         pdcch ? pdcchView->GetNumCce(pdcchView->GetMaxSymbols()) * slotsPerSecond
                               : 0.0};
        // A voice packet below the MAC: frame, compressed or full headers, layer 2
        const uint32_t headerBytes =
            rohc ? (rohcModel == "table"
                        ? static_cast<uint32_t>(
                              AttributeDefault<UintegerValue>("ns3::NrPdcp", "RohcTableUo0Size"))
                        : 4)
                 : (rtp ? 40 : 28);
        const uint32_t packetBytes = voicePayload + headerBytes + l2Bytes;
        const double transmissions = harq ? 1.0 / (1.0 - targetBler) : 1.0;
        Ptr<NrAmc> amc = CreateObjectWithAttributes<NrAmc>(
            "ErrorModelType",
            TypeIdValue(TypeId::LookupByName("ns3::" + errorModel)));
        Ptr<const SpectrumModel> model =
            NrSpectrumValueHelper::GetSpectrumModel(carrierRbs, ulFrequencyHz, 15e3);
        const std::string schedulingMode = scheduling;
        const bool down = sendDown;
        const bool up = sendUp;
        const bool modelPdcch = pdcch;
        const uint16_t cell = cellId;
        const uint32_t rbs = carrierRbs;
        const uint32_t repetitions = puschRepetitions;
        const double repSinrDb = puschRepSinrDb;
        // While its speakers talk, a call sends a packet every 20 ms in each direction: per
        // second 50 blocks of the RBs the UE's MCS needs (uplink: K slots of its grant), with
        // their retransmissions, and with dynamic scheduling a DCI for each of them
        g_cc.need = [=](uint32_t i) {
            std::array<double, 3> need{};
            const double dlDb =
                g_dlSinrN[i] > 0 ? 10.0 * std::log10(g_dlSinrSum[i] / g_dlSinrN[i]) : g_ue0DlCnrDb;
            const double perSecond = 50.0 * transmissions;
            if (down)
            {
                SpectrumValue sinr(model);
                sinr = std::pow(10.0, dlDb / 10.0);
                uint8_t mcs = 0;
                amc->CreateCqiFeedbackSiso(sinr, mcs);
                uint32_t n = 1;
                while (n < rbs && amc->CalculateTbSize(mcs, 1, n * 13) < packetBytes)
                {
                    ++n;
                }
                need[0] = perSecond * n;
            }
            if (up)
            {
                const double ulDb = dlDb - g_ulBelowDlDb;
                uint32_t k = std::max<uint32_t>(repetitions, 1);
                if (repetitions == 0)
                {
                    k = 32;
                    for (const uint32_t n : {1, 2, 3, 4, 7, 8, 12, 16, 20, 24, 28, 32})
                    {
                        if (ulDb + 10.0 * std::log10(n) >= repSinrDb)
                        {
                            k = n;
                            break;
                        }
                    }
                }
                need[1] = perSecond * ulRbs * k;
            }
            if (modelPdcch)
            {
                const uint8_t al = NrPdcch::Get(cell)->ChooseAl(dlDb);
                // (semi-persistent: DCIs for retransmissions and the odd activation only)
                const double dcis =
                    schedulingMode == "dynamic" ? perSecond : perSecond - 50.0 + 1.0;
                need[2] = al * dcis * ((down ? 1 : 0) + (up ? 1 : 0));
            }
            return need;
        };
        Ptr<NrGnbRrc> gnbRrc = gnbDevs.Get(0)->GetObject<NrGnbNetDevice>()->GetRrc();
        g_cc.releaseUe = [gnbRrc](uint32_t i) {
            const int32_t c = CurrentConnection(i);
            if (c < 0)
            {
                return;
            }
            const uint16_t rnti = g_connections[i][c].rnti;
            if (gnbRrc->HasUeManager(rnti))
            {
                gnbRrc->GetUeManager(rnti)->SendRrcConnectionRelease();
            }
        };
        if (pdcch)
        {
            g_cc.callStarts = [cell](uint32_t i) {
                const int32_t c = CurrentConnection(i);
                if (c >= 0)
                {
                    NrPdcch::Get(cell)->CallStarts(g_connections[i][c].rnti);
                }
            };
        }
        if (dropCalls)
        {
            Simulator::Schedule(callsStart + MilliSeconds(100), &CheckCalls);
        }
    }

    Simulator::Stop(simEnd);
    Simulator::Run();

    // The orbit prediction that sized K_offset must be where the model really flew the satellite
    NS_ABORT_MSG_IF((orbit.At(Simulator::Now()) - satMobility->GetPosition()).GetLength() > 1.0,
                    "the orbit prediction and the orbit model disagree");

    // --rtp=0: how often the NGMN generators really sent while talking (the SID interval: nobody
    // talked)
    if (!rtp && g_shortestVoiceGap < MilliSeconds(sidIntervalMs))
    {
        const bool perFrame = g_shortestVoiceGap == MilliSeconds(20);
        std::cout << "Voice packets left every " << std::fixed << std::setprecision(2)
                  << g_shortestVoiceGap.GetSeconds() * 1000.0 << " ms while talking"
                  << (perFrame ? " (one per 20 ms frame)"
                               : ": TrafficGeneratorNgmnVoip keeps 12.2 kbit/s, it is not patched "
                                 "with ngmn-voip-20ms.patch")
                  << "\n\n";
    }

    // --- One line per direction of a call ------------------------------------------------------
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());

    // Every voice flow of FlowMonitor is one direction of one call: DOWN when it goes to a
    // satellite UE, UP when it comes from one. The tables list them by UE, DOWN first.
    struct VoiceFlow
    {
        uint32_t flowId{0};
        uint32_t ue{0};
        Dir dir{DOWN};
        uint64_t tx{0};
        uint64_t rx{0};
        double delayMs{NAN};  // end to end, FlowMonitor's mean
        double jitterMs{NAN}; // its mean jitter
    };

    std::vector<VoiceFlow> flows;
    for (const auto& [flowId, st] : monitor->GetFlowStats())
    {
        const Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        if (t.destinationPort != port)
        {
            continue; // not a voice flow
        }
        VoiceFlow f;
        f.flowId = flowId;
        bool found = false;
        for (uint32_t i = 0; i < nUes && !found; ++i)
        {
            for (const Dir dir : {DOWN, UP})
            {
                if ((dir == DOWN ? t.destinationAddress : t.sourceAddress) ==
                    ueIfaces.GetAddress(i))
                {
                    f.ue = i;
                    f.dir = dir;
                    found = true;
                    break;
                }
            }
        }
        NS_ABORT_MSG_UNLESS(found,
                            "a voice flow of no satellite UE: " << t.sourceAddress << " -> "
                                                                << t.destinationAddress);
        f.tx = st.txPackets;
        f.rx = st.rxPackets;
        f.delayMs = st.rxPackets ? st.delaySum.GetSeconds() * 1000.0 / st.rxPackets : NAN;
        f.jitterMs =
            st.rxPackets > 1 ? st.jitterSum.GetSeconds() * 1000.0 / (st.rxPackets - 1) : NAN;
        flows.push_back(f);
    }
    std::sort(flows.begin(), flows.end(), [](const VoiceFlow& a, const VoiceFlow& b) {
        return std::make_pair(a.ue, a.dir) < std::make_pair(b.ue, b.dir);
    });
    // The first columns of every table: the UE (its call), the direction, the address
    auto who = [&](const std::string& ue, const std::string& dir, const std::string& address) {
        std::ostringstream o;
        o << std::setw(4) << ue << "  " << std::left << std::setw(3) << dir << std::right
          << std::setw(12) << address;
        return o.str();
    };
    auto dirName = [](Dir dir) { return std::string(dir == DOWN ? "dl" : "ul"); };
    auto addressOf = [](Ipv4Address a) {
        std::ostringstream o;
        o << a;
        return o.str();
    };
    auto whoFlow = [&](const VoiceFlow& f) {
        return who(std::to_string(f.ue), dirName(f.dir), addressOf(ueIfaces.GetAddress(f.ue)));
    };

    // N[km] and E[km]: where the UE is, north and east of the beam centre.
    // SF[dB]: the shadow fading of the UE as a gain (negative: a weaker signal), the mean over
    // the call (it changes only when the satellite flies); "-" when ns-3 draws it.
    // lost = tx - rx: every generator stopped 2 s before the end, so nothing is still on its way.
    // beam[dB] is the mean over the call of the carrier of the direction (it changes only when the
    // satellite flies).

    // --- Call control: which calls were admitted, blocked, released --------------------------
    std::ostringstream csvControl; // <csv>_control.csv: one line for the run
    {
        const CallControl& c = g_cc;
        uint32_t offered = 0;
        uint32_t blocked = 0;
        uint32_t released = 0;
        std::array<uint32_t, 3> blockedBy{};
        std::array<double, 3> needSum{};
        uint32_t needN = 0;
        std::vector<uint32_t> blockedUes;
        for (uint32_t i = 0; i < nUes; ++i)
        {
            if (c.status[i] == CallControl::WAITING)
            {
                continue; // no call: the satellite is never 10 deg above the UE during UE 0's
            }
            ++offered;
            if (c.status[i] == CallControl::BLOCKED)
            {
                ++blocked;
                ++blockedBy[c.limitedBy[i]];
                blockedUes.push_back(i);
                continue;
            }
            released += c.status[i] == CallControl::RELEASED;
            for (int r = 0; r < 3; ++r)
            {
                needSum[r] += c.held[i][r];
            }
            ++needN;
        }
        Frame("CALL CONTROL",
              admission
                  ? "admission: the calls, talking, reserve at most " + Num(admissionRbPct, 0) +
                        " % of the RBs" +
                        (pdcch ? ", " + Num(admissionCcePct, 0) + " % of the CCEs" : std::string())
                  : std::string("no admission control (--admission=0)"));
        std::ostringstream by;
        for (int r = 0; r < 3; ++r)
        {
            if (blockedBy[r] > 0)
            {
                by << (by.str().empty() ? " (" : ", ") << blockedBy[r] << " for "
                   << g_resourceName[r];
            }
        }
        if (!by.str().empty())
        {
            by << ")";
        }
        std::cout << "  calls " << offered << ": admitted " << offered - blocked << ", blocked "
                  << blocked << by.str() << "; released " << released << " of the admitted"
                  << (dropCalls ? "" : " (--dropCalls=0: none is)") << "; at most "
                  << c.peakAdmitted << " on at once\n";
        std::cout << "  a call reserves, on average, per second while talking:";
        for (int r = 0; r < 3; ++r)
        {
            if (r == 2 && !pdcch)
            {
                continue;
            }
            std::cout << (r ? ", " : " ") << std::fixed << std::setprecision(0)
                      << (needN ? needSum[r] / needN : 0.0) << " " << g_resourceName[r];
        }
        std::cout << "\n  of the cell's";
        for (int r = 0; r < 3; ++r)
        {
            if (r == 2 && !pdcch)
            {
                continue;
            }
            std::cout << (r ? ", " : " ") << c.capacity[r] << " " << g_resourceName[r];
        }
        std::cout << " a second;\n  the admitted calls reserved at most";
        for (int r = 0; r < 3; ++r)
        {
            if (r == 2 && !pdcch)
            {
                continue;
            }
            std::cout << (r ? ", " : " ") << std::setprecision(1) << 100.0 * c.peakShare[r] << " %";
        }
        std::cout << "\n";
        if (!blockedUes.empty())
        {
            // UE numbers, runs of consecutive ones as a range
            std::ostringstream list;
            for (std::size_t k = 0; k < blockedUes.size(); ++k)
            {
                std::size_t e = k;
                while (e + 1 < blockedUes.size() && blockedUes[e + 1] == blockedUes[e] + 1)
                {
                    ++e;
                }
                list << (k ? ", " : "") << blockedUes[k];
                if (e > k)
                {
                    list << "-" << blockedUes[e];
                }
                k = e;
            }
            std::cout << "  blocked: UE " << list.str() << "\n";
        }
        if (released > 0)
        {
            std::cout << "  released    UE      address  at[s]  dir  on time[%]\n";
            for (uint32_t i = 0; i < nUes; ++i)
            {
                if (c.status[i] != CallControl::RELEASED)
                {
                    continue;
                }
                std::cout << std::string(12, ' ') << std::setw(4) << i << std::setw(13)
                          << addressOf(ueIfaces.GetAddress(i)) << std::setprecision(2)
                          << std::setw(7) << c.at[i].GetSeconds() << std::setw(5)
                          << dirName(static_cast<Dir>(c.releasedDir[i])) << std::setprecision(1)
                          << std::setw(12) << c.onTimePct[i] << "\n";
            }
        }
        std::cout << "reserve: what the gNB estimates a call needs while its speakers talk, from "
                     "its UE's downlink SINR (uplink: that less the\n"
                  << "difference of the CNRs of the header), MCS, repetitions and HARQ.\n"
                  << "released: fewer than " << Num(dropOnTimePct, 0)
                  << " % of its packets in a direction within the PDB over 2 s, of those that "
                     "arrived or were discarded as late\n"
                  << "(not those lost on the radio); its voice ends, and the gNB releases the "
                     "UE's RRC connection. on time: that share.\n";
        if (callControlOff)
        {
            std::cout << "  NOTE: --rtp=0: no admission control and no release (the NGMN "
                         "generator cannot be stopped).\n";
        }
        csvControl << admission << "," << CsvNum(admissionRbPct) << "," << CsvNum(admissionCcePct)
                   << "," << dropCalls << "," << CsvNum(dropOnTimePct) << "," << offered << ","
                   << offered - blocked << "," << blocked << "," << blockedBy[0] << ","
                   << blockedBy[1] << "," << blockedBy[2] << "," << released << ","
                   << c.peakAdmitted;
        for (int r = 0; r < 3; ++r)
        {
            csvControl << "," << CsvNum(needN ? needSum[r] / needN : NAN);
        }
        for (int r = 0; r < 3; ++r)
        {
            csvControl << "," << CsvNum(c.capacity[r] > 0.0 ? c.capacity[r] : NAN);
        }
        for (int r = 0; r < 3; ++r)
        {
            csvControl << "," << CsvNum(c.capacity[r] > 0.0 ? 100.0 * c.peakShare[r] : NAN);
        }
        csvControl << "\n";
    }

    // --csv: what this table says of each flow, for <csv>_calls.csv
    struct CallCsv
    {
        std::string address;
        double northKm{NAN};
        double eastKm{NAN};
        double beamDb{NAN};
        double sfDb{NAN};
        double vaf{NAN};
        double delayMs{NAN};
        double jitterMs{NAN};
        double losPct{NAN};
        uint32_t rlf{0};
        double outS{NAN};
        double dlSinrDb{NAN};
    };

    std::vector<CallCsv> callCsv;
    // out[s]: how long of its call the UE had no connection, after a radio link failure.
    // DL SINR: the mean over the control symbols the UE received; it receives none while it
    // has no cell, so the time out of coverage after a failure is not in it.
    // The two ends of each call: the satellite UE's address, an arrow the way the voice goes,
    // the far end's address; as wide as the longest of the run
    int wSat = 7;
    int wFar = 7;
    for (const VoiceFlow& f : flows)
    {
        wSat = std::max(wSat, static_cast<int>(addressOf(ueIfaces.GetAddress(f.ue)).size()));
        wFar = std::max(wFar, static_cast<int>(addressOf(farAddress(f.ue)).size()));
    }
    Frame("CALLS", "one line per direction of a call; delay: end to end, sender to receiver");
    std::cout << "  UE  dir " << std::setw(wSat) << "address" << "    " << std::left
              << std::setw(wFar) << "far end" << std::right
              << "  N[km]  E[km] beam[dB] SF[dB]  VAF    tx    rx delay[ms] jitter[ms] LOS[%] RLF "
                 "out[s] DL SINR[dB]\n";
    for (const VoiceFlow& f : flows)
    {
        const uint32_t i = f.ue;
        const CallView& view = g_view[i];
        const Span& taper = view.taperDb[f.dir == DOWN ? 0 : 1];
        Time out{0};
        for (const auto& [from, to] : g_outages[i])
        {
            out += Max(Time(0), Min(to, view.callEnd) - Max(from, view.callStart));
        }
        const double sinrDb = g_dlSinrN[i] ? 10 * std::log10(g_dlSinrSum[i] / g_dlSinrN[i]) : NAN;
        callCsv.push_back({addressOf(ueIfaces.GetAddress(i)),
                           ueNorthKm[i],
                           ueEastKm[i],
                           taper.sum / taper.n,
                           view.shadowDb.n > 0 ? view.shadowDb.sum / view.shadowDb.n : NAN,
                           vaf[f.dir][i],
                           f.delayMs,
                           f.jitterMs,
                           100.0 * view.losYes / view.losAll,
                           g_rlf[i],
                           out.GetSeconds(),
                           sinrDb});
        std::cout << std::fixed << std::setw(4) << i << "  " << dirName(f.dir) << "  "
                  << std::setw(wSat) << addressOf(ueIfaces.GetAddress(i))
                  << (f.dir == DOWN ? " <- " : " -> ") << std::left << std::setw(wFar)
                  << addressOf(farAddress(i)) << std::right << std::setprecision(1) << std::setw(7)
                  << ueNorthKm[i] << std::setw(7) << ueEastKm[i] << std::setw(9)
                  << std::setprecision(2) << taper.sum / taper.n << std::setw(7);
        if (view.shadowDb.n > 0)
        {
            std::cout << view.shadowDb.sum / view.shadowDb.n;
        }
        else
        {
            std::cout << "-";
        }
        std::cout << std::setw(5) << vaf[f.dir][i] << std::setw(6) << f.tx << std::setw(6) << f.rx
                  << std::setprecision(3) << std::setw(10);
        if (f.rx)
        {
            std::cout << f.delayMs;
        }
        else
        {
            std::cout << "-";
        }
        std::cout << std::setw(11);
        if (f.rx > 1)
        {
            std::cout << f.jitterMs;
        }
        else
        {
            std::cout << "-";
        }
        std::cout << std::setprecision(0) << std::setw(7) << 100.0 * view.losYes / view.losAll
                  << std::setw(4) << g_rlf[i] << std::setprecision(2) << std::setw(7)
                  << out.GetSeconds() << std::setprecision(1) << std::setw(12) << sinrDb << "\n";
    }
    std::cout
        << "dir: dl = the far end talks to the UE, over the satellite's downlink; ul = the UE "
           "talks. VAF: of the one who talks.\n"
        << "address, far end: the two ends of the call, the satellite UE and the "
        << (farUe ? "terrestrial UE" : "remote host")
        << "; the arrow points the way the voice goes.\n"
        << "The packets lost (tx - rx) are in the next table, by cause.\n"
        << "N, E: where the UE is, north and east of the beam centre. beam, SF: beam gain (on "
           "the carrier of the direction) and\n"
        << "shadow fading there (negative: weaker). RLF: radio link failures. out: time of the "
           "call without a connection after them.\n"
        << "DL SINR: of the control symbols received.\n";
    for (uint32_t i = 0; i < nUes; ++i)
    {
        if (!ueHasCall[i])
        {
            std::cout << "  NOTE: UE " << i << " (" << ueIfaces.GetAddress(i) << ", N "
                      << std::setprecision(1) << ueNorthKm[i] << " km, E " << ueEastKm[i]
                      << " km) had no call: the satellite was not at 10 deg or more above it during "
                         "UE 0's call\n";
        }
    }

    // --- Why the packets were lost ---------------------------------------------------------------
    // NrPdcp counts, for every UE and direction, the packets its sending PDCP entities were
    // given, those that reached the receiving PDCP entities and those passed on from there.
    // On the satellite UE's stretch:
    //   no conn.  never given to a PDCP entity: the UE had no connection (after a radio link
    //             failure), or no bearer yet
    //   discard   given to PDCP, but the RLC of the sender did not take it: the oldest packet
    //             in its queue had waited longer than the discard timer (see "late packets"
    //             in the header of the output)
    //   radio     sent, and never arrived at the other end: lost on the air (every HARQ
    //             transmission failed), or still waiting in the RLC when the connection broke.
    //             Rarely one arrives damaged, which NrPdcp drops (PacketCounts::damaged): the
    //             RLC cannot always tell, after a long burst of losses, that two pieces are
    //             not of the same packet. A line under the table says so.
    //   ROHC      arrived, but the decompressor had lost its context and could not restore the
    //             header; it dropped the packet
    // far end: lost on the far end's stretch, for any of these reasons (a terrestrial UE), or on
    // the N6 link (the remote host: none). The first three are lost with or without header
    // compression ("link"); the fourth is what ROHC cost. A packet lost anywhere else would make
    // the five not add up to "lost", and is reported in a line of its own.
    Frame("LOST PACKETS BY CAUSE", "in packets, and in % of the packets sent");
    std::cout << "  UE  dir     address      tx      rx    lost  no conn.  discard  radio  ROHC  "
                 "far end  link[%]  ROHC[%]\n";

    // The same counts, kept for the tables after this one. Without far: the satellite UE's
    // stretch; far: the far end's.
    struct Causes
    {
        uint64_t tx{0};
        uint64_t rx{0};
        uint64_t noConnection{0};
        uint64_t onBearer{0}; // given to the PDCP entity of a bearer
        uint64_t discarded{0};
        uint64_t radio{0};
        uint64_t rohc{0};
        uint64_t far{0}; // lost on the far end's stretch
    };

    std::vector<Causes> causes;    // per flow
    std::vector<Causes> farCauses; // per flow, --farEnd=ue: the terrestrial UE's stretch
    // The causes of one stretch from the counts of its PDCP, given what entered it
    auto stretchCauses = [](uint64_t entered, const NrPdcp::PacketCounts& n) {
        Causes c;
        c.noConnection = entered - n.sdus;
        c.onBearer = n.sdus;
        c.discarded = n.discarded;
        c.radio = n.sent - n.discarded - n.arrived;
        c.rohc = (n.sdus - n.sent) + (n.arrived - n.delivered); // compressor, decompressor
        return c;
    };
    {
        auto line = [&](const std::string& first, const Causes& c) {
            std::cout << first << std::setw(8) << c.tx << std::setw(8) << c.rx << std::setw(8)
                      << c.tx - c.rx << std::setw(10) << c.noConnection << std::setw(9)
                      << c.discarded << std::setw(7) << c.radio << std::setw(6)
                      << (rohc ? std::to_string(c.rohc) : "-") << std::setw(9) << c.far
                      << std::setprecision(2) << std::setw(9);
            if (c.tx == 0)
            {
                std::cout << "-" << std::setw(9) << "-";
            }
            else
            {
                std::cout << 100.0 * (c.noConnection + c.discarded + c.radio) / c.tx << std::setw(9);
                if (rohc)
                {
                    std::cout << 100.0 * c.rohc / c.tx;
                }
                else
                {
                    std::cout << "-";
                }
            }
            std::cout << "\n";
        };
        Causes all;
        std::ostringstream notes;
        for (const VoiceFlow& f : flows)
        {
            const uint32_t i = f.ue;
            const NrPdcp::PacketCounts n =
                NrPdcp::GetPacketCounts(ueIfaces.GetAddress(i), f.dir == DOWN);
            NrPdcp::PacketCounts nf;
            if (farUe)
            {
                nf = NrPdcp::GetPacketCounts(farIfaces.GetAddress(i), f.dir == UP);
            }
            Causes c;
            Causes fc;
            if (f.dir == DOWN)
            {
                // far end first: what left its stretch entered the satellite UE's
                const uint64_t atUpf = farUe ? nf.delivered : f.tx;
                if (farUe)
                {
                    fc = stretchCauses(f.tx, nf);
                }
                c = stretchCauses(atUpf, n);
                c.far = f.tx - atUpf;
                if (n.delivered != f.rx)
                {
                    notes << "  NOTE: UE " << i << " dl: PDCP passed on " << n.delivered
                          << " packets and the UE received " << f.rx
                          << ": lost outside the radio bearers\n";
                }
            }
            else
            {
                c = stretchCauses(f.tx, n);
                if (farUe)
                {
                    fc = stretchCauses(n.delivered, nf);
                }
                c.far = n.delivered >= f.rx ? n.delivered - f.rx : 0;
                if (n.delivered < f.rx || (farUe && nf.delivered != f.rx))
                {
                    notes << "  NOTE: UE " << i << " ul: PDCP passed on " << n.delivered
                          << " packets, the far end received " << f.rx
                          << ": lost outside the radio bearers\n";
                }
            }
            c.tx = f.tx;
            c.rx = f.rx;
            if (n.sdus > f.tx + (f.dir == DOWN ? 0 : 0) && f.dir == UP)
            {
                notes << "  NOTE: UE " << i << " ul: PDCP was given " << n.sdus
                      << " packets, more than were sent\n";
            }
            if (n.damaged > 0)
            {
                notes << "  NOTE: UE " << i << " " << dirName(f.dir) << ": " << n.damaged
                      << " of the radio losses arrived, but damaged: the RLC had joined pieces "
                         "of two packets (NrPdcp drops these)\n";
            }
            if (farUe && c.far > 0)
            {
                notes << "  NOTE: UE " << i << " " << dirName(f.dir) << ": far end lost " << c.far
                      << ": " << fc.noConnection << " no conn., " << fc.discarded << " discard, "
                      << fc.radio << " radio, " << fc.rohc << " ROHC\n";
            }
            else if (!farUe && c.far > 0)
            {
                notes << "  NOTE: UE " << i << " ul: " << c.far
                      << " packets lost between the UPF and the remote host\n";
            }
            line(whoFlow(f), c);
            causes.push_back(c);
            farCauses.push_back(fc);
            all.tx += c.tx;
            all.rx += c.rx;
            all.noConnection += c.noConnection;
            all.discarded += c.discarded;
            all.radio += c.radio;
            all.rohc += c.rohc;
            all.far += c.far;
        }
        if (flows.size() > 1)
        {
            line(who("all", "", ""), all);
        }
        std::cout << "no conn.: the UE had no connection. discard: the sender did not take it, the "
                     "oldest packet in its queue had waited too long.\n"
                  << "radio: sent, did not arrive. ROHC: arrived, but the header could not be "
                     "restored. All four on the satellite UE's stretch.\n"
                  << "far end: lost on the far end's stretch ("
                  << (farUe ? "the terrestrial UE's radio and core"
                            : "the N6 link to the remote host")
                  << "). link = no conn. + discard + radio.\n"
                  << notes.str();
    }

    // --csv: every line of every file starts with what identifies the run
    const std::string csvRunHeader =
        "nUes,direction,scheduling,scheduler,bandwidthMHz,pdcch,pdcchSymbols,idealControl,harq,"
        "targetBler,rohc,rohcModel,rohcMode,fullPass,freezeGeometry,elevationDeg,durationS,"
        "ueAntennaGainDbi,ntnScenario,shadowing,shadowingDb,atmosphere,latitudeDeg,fastFading,"
        "fastFadingDb,linkMarginDb,carrierRbs,farEnd,farEndBand,farEndBandwidthMHz,"
        "farEndFrequencyGHz,farEndUesPerGnb,farEndGnbDbm,farEndMinDistM,farEndMaxDistM,"
        "farEndScenario,farEndTddPattern,farEndErrorModel,ulGrantRbs,puschRepetitions,"
        "puschRepSinrDb,dlBoostMaxDb,dlPowerRefDeg,ulPowerRefDeg,errorModel,rngSeed,rngRun";
    std::string csvRun;
    {
        std::string schedulerName = scheduler;
        const std::string prefix = "ns3::NrMacScheduler";
        if (schedulerName.compare(0, prefix.size(), prefix) == 0)
        {
            schedulerName.erase(0, prefix.size());
        }
        std::ostringstream o;
        o << nUes << "," << direction << "," << scheduling << "," << schedulerName << ","
          << CsvNum(bandwidthMHz) << "," << pdcch << "," << pdcchSymbols << "," << idealControl
          << "," << harq << "," << CsvNum(targetBler) << "," << rohc << ","
          << (rohc ? rohcModel : "-") << "," << (rohc ? rohcMode : "-") << "," << fullPass << ","
          << freezeGeometry << "," << CsvNum(elevation) << "," << CsvNum(duration.GetSeconds())
          << "," << CsvNum(ueAntennaGainDbi) << "," << scenario << "," << shadowing << ","
          << CsvNum(shadowingDb) << "," << atmosphere << "," << CsvNum(latitudeDeg) << ","
          << fastFading << "," << CsvNum(fastFadingDb) << "," << CsvNum(linkMarginDb) << ","
          << carrierRbs << "," << farEnd << "," << (farUe ? farEndBand : "-") << ",";
        if (farUe)
        {
            o << CsvNum(farEndBandwidthMHz) << "," << CsvNum(farDlHz / 1e9) << ","
              << farEndUesPerGnb << "," << CsvNum(farEndGnbDbm) << "," << CsvNum(farEndMinDistM)
              << "," << CsvNum(farEndMaxDistM) << "," << farEndScenario << ","
              << (farTdd ? farEndTddPattern : "-") << "," << farEndErrorModel;
        }
        else
        {
            o << "-,-,-,-,-,-,-,-,-";
        }
        o << "," << ulGrantRbs << "," << puschRepetitions << ","
          << (puschRepetitions == 0 ? CsvNum(puschRepSinrDb) : std::string("-")) << ","
          << CsvNum(dlBoostMaxDb) << "," << CsvNum(dlPowerRefDeg) << "," << CsvNum(ulPowerRefDeg)
          << "," << errorModel << "," << RngSeedManager::GetSeed() << ","
          << RngSeedManager::GetRun();
        csvRun = o.str();
    }
    std::ostringstream csvCalls;       // <csv>_calls.csv: a line per flow and per bearer of it
    std::ostringstream csvDelays;      // <csv>_delays.csv: per flow, packets per 1 ms of delay
    std::ostringstream csvPacketLines; // <csv>_packets.csv: a line per delivered packet

    // --- The voice flows against 5QI 1 ---------------------------------------------------------
    // TS 23.501 measures a QoS flow on the packets its bearer was given ("on bearer"; a packet
    // sent while the UE had no connection met no bearer and is in the table above only):
    //   5.7.3.4  "98 percent of the packets shall not experience a delay exceeding the 5QI's
    //            PDB", for a GBR flow that keeps to its GFBR. The delay is the one between UE
    //            and UPF. A packet that took longer is late, not lost (Table 5.7.4-1 NOTE 1).
    //            A packet the sender discarded because its queue was too old did not make the
    //            budget either, so "over PDB" is late + discard.
    //   5.7.3.5  the packet error rate is the share of packets the sending link layer
    //            processed and the receiving one did not deliver: radio + ROHC.
    // The delays are those of the packets that arrived; 98% is the delay that 98 % of them
    // stayed within. kbit/s is the highest bit rate of the IP packets over the averaging window.
    // Both directions of a call are held against it, each on the satellite UE's stretch; the far
    // end's stretch has a table of its own.
    struct Row
    {
        Causes c;
        uint64_t inPdb{0};
        std::vector<int64_t> delayNs; // sorted
        double kbps{0};
    };

    const int64_t pdbNs = voicePdb.GetNanoSeconds();
    // In whole numbers, so that exactly 2 % and exactly the PER are not "above"
    const uint64_t perPerMillion = std::llround(voicePer * 1e6);
    auto missedBy = [&](const Row& r) {
        const uint64_t late = r.delayNs.size() - r.inPdb;
        const bool pdb = 100 * (late + r.c.discarded) > 2 * r.c.onBearer;
        const bool per = 1000000 * (r.c.radio + r.c.rohc) > perPerMillion * r.c.onBearer;
        return std::string(pdb ? "PDB" : "") + (pdb && per ? " " : "") + (per ? "PER" : "");
    };
    // The smallest delay that at least p % of the packets did not exceed (sorted delays)
    auto delayPct = [](const std::vector<int64_t>& sorted, uint64_t p) {
        return sorted.empty() ? NAN : sorted[(sorted.size() * p + 99) / 100 - 1] / 1e6;
    };
    auto meanMs = [](const std::vector<int64_t>& delays) {
        int64_t sum = 0;
        for (const int64_t d : delays)
        {
            sum += d;
        }
        return delays.empty() ? NAN : sum / 1e6 / delays.size();
    };
    auto qosLine = [&](const std::string& first, const Row& r, const std::string& missed) {
        const uint64_t delivered = r.delayNs.size();
        const uint64_t late = delivered - r.inPdb;
        const uint64_t lost = r.c.radio + r.c.rohc;
        std::cout << first << std::setw(11) << r.c.onBearer << std::setw(8) << r.inPdb
                  << std::setw(6) << late << std::setw(9) << r.c.discarded << std::setw(6) << lost
                  << std::setprecision(2) << std::setw(13);
        if (r.c.onBearer == 0)
        {
            std::cout << "-" << std::setw(8) << "-";
        }
        else
        {
            std::cout << 100.0 * (late + r.c.discarded) / r.c.onBearer << std::setw(8)
                      << 100.0 * lost / r.c.onBearer;
        }
        if (delivered == 0)
        {
            std::cout << std::setw(10) << "-" << std::setw(10) << "-" << std::setw(10) << "-";
        }
        else
        {
            // (wide enough for the 10 s and more an overloaded uplink can take)
            std::cout << std::setprecision(3) << std::setw(10) << meanMs(r.delayNs) << std::setw(10)
                      << delayPct(r.delayNs, 98) << std::setw(10) << r.delayNs.back() / 1e6;
        }
        std::cout << std::setprecision(1) << std::setw(8) << r.kbps << "  "
                  << (missed.empty() ? "-" : missed) << "\n";
    };
    const std::string qosHeader =
        "  UE  dir     address  on bearer  in PDB  late  discard  lost  "
        "over PDB[%]  PER[%]  mean[ms]   98%[ms]   max[ms]  kbit/s  missed\n";
    auto qosNotes = [&](const std::string& where) {
        std::cout
            << "on bearer: packets the voice bearer was given. in PDB, late: delivered within, "
               "after the budget; late is not lost.\n"
            << "lost = radio + ROHC. over PDB = late + discard, PER = lost, both in % of on "
               "bearer.\n"
            << "mean, 98%, max: delay " << where << ". kbit/s: the highest mean over "
            << std::setprecision(0) << voiceAveragingWindow.GetSeconds() << " s.\n"
            << "missed: PDB if over PDB is above 2 %, PER if PER is above " << voicePer * 100.0
            << " %.\n";
    };
    std::vector<Row> satRows(flows.size()); // per flow, for the end-to-end table
    std::vector<Row> farRows(flows.size());
    Frame("VOICE AGAINST 5QI 1 (TS 23.501 Table 5.7.4-1)",
          "satellite UE <-> UPF: at most " + Num(voicePdbMs, 0) +
              " ms for 98 % of the packets, at most " + Num(voicePer * 100.0, 0) + " % lost");
    std::cout << qosHeader;
    {
        // --csv: a line of <csv>_calls.csv. call: the line of the flow (bearer 0), with what the
        // CALLS and LOST tables say of it; nullptr for a line of one of its bearers. far, e2e:
        // the far end's stretch and the whole way, for the line of the flow
        auto csvLine = [&](const VoiceFlow& f,
                           uint32_t bearer,
                           const std::string& rntis,
                           const CallCsv& view,
                           const Causes* call,
                           const Row& r,
                           const UeDciCount* dcis,
                           const Row* far,
                           const std::vector<int64_t>* e2e) {
            const uint64_t delivered = r.delayNs.size();
            const uint64_t late = delivered - r.inPdb;
            const uint64_t lost = r.c.radio + r.c.rohc;
            const bool pdb = 100 * (late + r.c.discarded) > 2 * r.c.onBearer;
            const bool per = 1000000 * lost > perPerMillion * r.c.onBearer;
            auto pct = [](uint64_t n, uint64_t of) { return of ? 100.0 * n / of : NAN; };
            std::ostringstream o;
            o << csvRun << "," << f.flowId << "," << f.ue << "," << dirName(f.dir) << ","
              << view.address << "," << bearer << "," << rntis << "," << CsvNum(view.northKm) << ","
              << CsvNum(view.eastKm) << "," << CsvNum(view.beamDb) << "," << CsvNum(view.sfDb)
              << "," << CsvNum(view.vaf) << "," << CsvNum(view.losPct) << "," << view.rlf << ","
              << CsvNum(view.outS) << "," << CsvNum(view.dlSinrDb) << ",";
            for (const SnrStats* st : {&g_dlSnr[f.ue], &g_ulSnr[f.ue]})
            {
                o << CsvNum(st->n > 0 ? st->minDb : NAN) << "," << CsvNum(st->MeanDb()) << ","
                  << CsvNum(st->n > 0 ? st->maxDb : NAN) << ",";
            }
            if (call)
            {
                o << call->tx << "," << call->rx << "," << call->tx - call->rx << ","
                  << call->noConnection << ",";
            }
            else
            {
                o << ",,,,";
            }
            o << r.c.discarded << "," << r.c.radio << "," << r.c.rohc << ",";
            if (call)
            {
                o << CsvNum(pct(call->noConnection + call->discarded + call->radio, call->tx))
                  << "," << CsvNum(pct(call->rohc, call->tx)) << "," << CsvNum(view.delayMs) << ","
                  << CsvNum(view.jitterMs) << ",";
            }
            else
            {
                o << ",,,,";
            }
            o << r.c.onBearer << "," << r.inPdb << "," << late << ","
              << CsvNum(pct(late + r.c.discarded, r.c.onBearer)) << ","
              << CsvNum(pct(lost, r.c.onBearer)) << "," << CsvNum(meanMs(r.delayNs));
            for (uint64_t p : {50, 90, 95, 98, 99})
            {
                o << "," << CsvNum(delayPct(r.delayNs, p));
            }
            o << "," << CsvNum(delivered ? r.delayNs.back() / 1e6 : NAN) << "," << CsvNum(r.kbps)
              << ",";
            if (r.c.onBearer > 0)
            {
                o << pdb << "," << per << "," << (!pdb && !per);
            }
            else
            {
                o << ",,";
            }
            if (dcis)
            {
                o << "," << dcis->dcis << "," << dcis->cces << "," << dcis->missed << ","
                  << CsvNum(pct(dcis->missed, dcis->dcis));
            }
            else
            {
                o << ",,,,";
            }
            // the far end's stretch: lost on it, and (a terrestrial UE) its counts and delays
            if (call)
            {
                o << "," << call->far;
            }
            else
            {
                o << ",";
            }
            if (far)
            {
                o << "," << far->c.onBearer << "," << far->c.discarded << "," << far->c.radio << ","
                  << far->c.rohc << "," << CsvNum(meanMs(far->delayNs)) << ","
                  << CsvNum(delayPct(far->delayNs, 98));
            }
            else
            {
                o << ",,,,,,";
            }
            if (e2e)
            {
                const uint64_t over =
                    e2e->end() -
                    std::upper_bound(e2e->begin(), e2e->end(), MilliSeconds(150).GetNanoSeconds());
                o << "," << CsvNum(meanMs(*e2e)) << "," << CsvNum(delayPct(*e2e, 50)) << ","
                  << CsvNum(delayPct(*e2e, 98)) << ","
                  << CsvNum(e2e->empty() ? NAN : e2e->back() / 1e6) << ","
                  << CsvNum(pct(over, e2e->size()));
            }
            else
            {
                o << ",,,,,";
            }
            // call control: what became of the call
            const auto st = g_cc.status[f.ue];
            o << ","
              << (st == CallControl::RELEASED  ? "released"
                  : st == CallControl::BLOCKED ? "blocked"
                                               : "admitted")
              << "," << (st == CallControl::RELEASED ? CsvNum(g_cc.at[f.ue].GetSeconds()) : "");
            csvCalls << o.str() << "\n";
        };
        Row all;
        uint32_t flowsThatMissed = 0;
        bool aboveGfbr = false;
        bool bearerLines = false;
        std::ostringstream notes;
        for (std::size_t k = 0; k < flows.size(); ++k)
        {
            const VoiceFlow& f = flows[k];
            const uint32_t i = f.ue;
            VoiceQos& q = g_voiceQos.sat[i][f.dir];
            // The voice bearer of each connection the UE had during its call: from the counts
            // of NrPdcp between two connections, the delays measured on it, the bit rate of the
            // packets that entered while it was there
            const auto& connections = g_connections[i];
            const NrPdcp::PacketCounts total =
                NrPdcp::GetPacketCounts(ueIfaces.GetAddress(i), f.dir == DOWN);
            std::vector<std::pair<uint16_t, Row>> bearers; // RNTI, the bearer held against 5QI 1
            std::vector<uint32_t> bearerOf(connections.size(), 0); // connection -> bearer, from 1
            for (std::size_t j = 0; j < connections.size(); ++j)
            {
                // it ended at the first radio link failure after it was set up
                Time end = Time::Max();
                for (const auto& [from, to] : g_outages[i])
                {
                    if (from >= connections[j].start)
                    {
                        end = from;
                        break;
                    }
                }
                const NrPdcp::PacketCounts& a = connections[j].before[f.dir];
                const NrPdcp::PacketCounts& b =
                    j + 1 < connections.size() ? connections[j + 1].before[f.dir] : total;
                Row br;
                br.c.onBearer = b.sdus - a.sdus;
                br.c.discarded = b.discarded - a.discarded;
                br.c.radio = (b.sent - a.sent) - (b.discarded - a.discarded) - (b.arrived - a.arrived);
                br.c.rohc = ((b.sdus - a.sdus) - (b.sent - a.sent)) +
                            ((b.arrived - a.arrived) - (b.delivered - a.delivered));
                for (std::size_t n = 0; n < q.delayNs.size(); ++n)
                {
                    if (q.delayConnection[n] == static_cast<int32_t>(j))
                    {
                        br.delayNs.push_back(q.delayNs[n]);
                    }
                }
                const bool duringCall =
                    connections[j].start < ueCallEnd[i] && end > ueCallStart[i];
                if (!duringCall && br.c.onBearer == 0 && br.delayNs.empty())
                {
                    continue; // before or after the call, and nothing on its bearer
                }
                std::sort(br.delayNs.begin(), br.delayNs.end());
                br.inPdb = std::upper_bound(br.delayNs.begin(), br.delayNs.end(), pdbNs) -
                           br.delayNs.begin();
                br.kbps = j < q.connectionPeakBits.size()
                              ? q.connectionPeakBits[j] / voiceAveragingWindow.GetSeconds() / 1000.0
                              : 0.0;
                bearers.emplace_back(connections[j].rnti, std::move(br));
                bearerOf[j] = bearers.size();
            }
            if (!csv.empty() && csvPackets)
            {
                for (std::size_t n = 0; n < q.delayNs.size(); ++n)
                {
                    const int32_t j = q.delayConnection[n];
                    csvPacketLines << csvRun << "," << f.flowId << "," << i << "," << dirName(f.dir)
                                   << "," << (j >= 0 ? bearerOf[j] : 0) << ","
                                   << CsvNum(q.delayNs[n] / 1e6) << "\n";
                }
            }
            Row r;
            r.c = causes[k];
            r.delayNs = q.delayNs;
            std::sort(r.delayNs.begin(), r.delayNs.end());
            r.inPdb = std::upper_bound(r.delayNs.begin(), r.delayNs.end(), pdbNs) - r.delayNs.begin();
            r.kbps = q.peakWindowBits / voiceAveragingWindow.GetSeconds() / 1000.0;
            aboveGfbr = aboveGfbr || r.kbps > gfbrKbps;
            const std::string missed = missedBy(r);
            flowsThatMissed += !missed.empty();
            qosLine(whoFlow(f), r, missed);
            // The far end's stretch: a terrestrial UE's bearer, measured as this one
            if (farUe)
            {
                VoiceQos& fq = g_voiceQos.far[i][f.dir];
                Row& fr = farRows[k];
                fr.c = farCauses[k];
                fr.delayNs = fq.delayNs;
                std::sort(fr.delayNs.begin(), fr.delayNs.end());
                fr.inPdb = std::upper_bound(fr.delayNs.begin(), fr.delayNs.end(), pdbNs) -
                           fr.delayNs.begin();
                fr.kbps = fq.peakWindowBits / voiceAveragingWindow.GetSeconds() / 1000.0;
                if (fq.unstamped > 0)
                {
                    notes << "  NOTE: UE " << i << " " << dirName(f.dir) << ": " << fq.unstamped
                          << " packets reached the far end's stretch without its stamp\n";
                }
            }
            std::vector<int64_t> e2e = g_voiceQos.e2e[i][f.dir].delayNs;
            std::sort(e2e.begin(), e2e.end());
            if (!csv.empty())
            {
                // The DCIs of a connection (RNTI) for this flow: of its direction (DL assignments
                // for dl, UL grants for ul) when the calls are two-way, all of them when one-way
                auto dcisOf = [&](uint16_t rnti) {
                    UeDciCount c;
                    if (const auto it = g_dciByRnti.find(rnti); it != g_dciByRnti.end())
                    {
                        for (int d = 0; d < 2; ++d)
                        {
                            if (!(sendDown && sendUp) || d == (f.dir == DOWN ? 0 : 1))
                            {
                                c.dcis += it->second[d].dcis;
                                c.cces += it->second[d].cces;
                                c.missed += it->second[d].missed;
                            }
                        }
                    }
                    return c;
                };
                UeDciCount ueDcis;
                std::string rntis;
                for (const auto& [rnti, br] : bearers)
                {
                    rntis += (rntis.empty() ? "" : ";") + std::to_string(rnti);
                    const UeDciCount c = dcisOf(rnti);
                    ueDcis.dcis += c.dcis;
                    ueDcis.cces += c.cces;
                    ueDcis.missed += c.missed;
                }
                csvLine(f,
                        0,
                        rntis,
                        callCsv[k],
                        &causes[k],
                        r,
                        pdcch ? &ueDcis : nullptr,
                        farUe ? &farRows[k] : nullptr,
                        &e2e);
                for (std::size_t j = 0; j < bearers.size(); ++j)
                {
                    const UeDciCount c = dcisOf(bearers[j].first);
                    csvLine(f,
                            j + 1,
                            std::to_string(bearers[j].first),
                            callCsv[k],
                            nullptr,
                            bearers[j].second,
                            pdcch ? &c : nullptr,
                            nullptr,
                            nullptr);
                }
                // the delays of the flow on the satellite UE's stretch, in 1 ms steps
                std::map<int64_t, uint64_t> histogram;
                for (const int64_t d : r.delayNs)
                {
                    ++histogram[d / 1000000];
                }
                for (const auto& [ms, n] : histogram)
                {
                    csvDelays << csvRun << "," << f.flowId << "," << i << "," << dirName(f.dir)
                              << "," << ms << "," << n << "\n";
                }
            }
            if (bearers.size() > 1)
            {
                bearerLines = true;
                Causes sum;
                std::size_t delays = 0;
                for (std::size_t j = 0; j < bearers.size(); ++j)
                {
                    const Row& br = bearers[j].second;
                    qosLine(who(std::to_string(i) + "." + std::to_string(j + 1),
                                dirName(f.dir),
                                "RNTI " + std::to_string(bearers[j].first)),
                            br,
                            missedBy(br));
                    sum.onBearer += br.c.onBearer;
                    sum.discarded += br.c.discarded;
                    sum.radio += br.c.radio;
                    sum.rohc += br.c.rohc;
                    delays += br.delayNs.size();
                }
                if (sum.onBearer != r.c.onBearer || sum.discarded != r.c.discarded ||
                    sum.radio != r.c.radio || sum.rohc != r.c.rohc || delays != r.delayNs.size())
                {
                    notes << "  NOTE: UE " << i << " " << dirName(f.dir)
                          << ": its bearers do not add up to the flow; packets outside them\n";
                }
            }
            const uint64_t reached =
                f.dir == DOWN ? f.rx : r.c.onBearer - r.c.discarded - r.c.radio - r.c.rohc;
            if (r.delayNs.size() != reached || q.unstamped > 0)
            {
                notes << "  NOTE: UE " << i << " " << dirName(f.dir) << ": the delay of "
                      << r.delayNs.size() << " of " << reached
                      << " delivered packets was measured\n";
            }
            satRows[k] = r;
            all.c.onBearer += r.c.onBearer;
            all.c.discarded += r.c.discarded;
            all.c.radio += r.c.radio;
            all.c.rohc += r.c.rohc;
            all.inPdb += r.inPdb;
            all.delayNs.insert(all.delayNs.end(), r.delayNs.begin(), r.delayNs.end());
            all.kbps = std::max(all.kbps, r.kbps);
        }
        if (flows.size() > 1)
        {
            std::sort(all.delayNs.begin(), all.delayNs.end());
            qosLine(who("all", "", ""),
                    all,
                    std::to_string(flowsThatMissed) + (flowsThatMissed == 1 ? " flow" : " flows"));
        }
        if (aboveGfbr)
        {
            notes << "  NOTE: a flow sent more than its GFBR of " << gfbrKbps
                  << " kbit/s; the 98 % rule is for a flow that keeps to it (--gfbrKbps)\n";
        }
        qosNotes("satellite UE <-> UPF (dl: UPF to UE, ul: UE to UPF)");
        if (bearerLines)
        {
            std::cout << "i.1, i.2: UE i had more than one connection: one line for the voice "
                         "bearer of each, held against 5QI 1 on its own. A radio\nlink failure "
                         "ends a connection, the next one sets up a new bearer (RNTI as in the "
                         "ROHC tables). The line of the flow adds them up.\n";
        }
        std::cout << notes.str();
    }

    // --- The far end's stretch, --farEnd=ue: each terrestrial UE's 5QI 1 bearer ----------------
    if (farUe)
    {
        Frame("FAR END AGAINST 5QI 1",
              "terrestrial UE <-> UPF: dl = far UE to UPF, ul = UPF to far UE");
        std::cout << qosHeader;
        Row all;
        uint32_t flowsThatMissed = 0;
        for (std::size_t k = 0; k < flows.size(); ++k)
        {
            const VoiceFlow& f = flows[k];
            const Row& r = farRows[k];
            const std::string missed = missedBy(r);
            flowsThatMissed += !missed.empty();
            qosLine(
                who(std::to_string(f.ue), dirName(f.dir), addressOf(farIfaces.GetAddress(f.ue))),
                r,
                missed);
            all.c.onBearer += r.c.onBearer;
            all.c.discarded += r.c.discarded;
            all.c.radio += r.c.radio;
            all.c.rohc += r.c.rohc;
            all.inPdb += r.inPdb;
            all.delayNs.insert(all.delayNs.end(), r.delayNs.begin(), r.delayNs.end());
            all.kbps = std::max(all.kbps, r.kbps);
        }
        if (flows.size() > 1)
        {
            std::sort(all.delayNs.begin(), all.delayNs.end());
            qosLine(who("all", "", ""),
                    all,
                    std::to_string(flowsThatMissed) + (flowsThatMissed == 1 ? " flow" : " flows"));
        }
        qosNotes("terrestrial UE <-> UPF");
        std::cout
            << "address: of the terrestrial UE paired with UE i. Cell, place and SINR of each "
               "are in FAR END UEs below.\n";
    }

    // --- End to end: the whole way, and against ITU-T G.114 ------------------------------------
    {
        Frame("END TO END",
              "sender's IP to receiver's IP; ITU-T G.114: up to 150 ms one way suits most uses");
        std::cout << "  UE  dir     address    sent      rx  lost[%]  sat 98%[ms]  far 98%[ms]  "
                     "mean[ms]   98%[ms]   max[ms]  over 150 ms[%]\n";
        std::vector<int64_t> allDelays;
        uint64_t allTx = 0;
        uint64_t allRx = 0;
        std::ostringstream notes;
        auto e2eLine = [&](const std::string& first,
                           uint64_t tx,
                           uint64_t rx,
                           double sat98,
                           double far98,
                           const std::vector<int64_t>& d) {
            const uint64_t over =
                d.end() - std::upper_bound(d.begin(), d.end(), MilliSeconds(150).GetNanoSeconds());
            std::cout << first << std::setw(8) << tx << std::setw(8) << rx << std::setprecision(2)
                      << std::setw(9);
            if (tx == 0)
            {
                std::cout << "-";
            }
            else
            {
                std::cout << 100.0 * (tx - rx) / tx;
            }
            std::cout << std::setprecision(3);
            for (const double v : {sat98, far98})
            {
                std::cout << std::setw(13);
                if (std::isnan(v))
                {
                    std::cout << "-";
                }
                else
                {
                    std::cout << v;
                }
            }
            if (d.empty())
            {
                std::cout << std::setw(10) << "-" << std::setw(10) << "-" << std::setw(10) << "-"
                          << std::setw(16) << "-" << "\n";
                return;
            }
            std::cout << std::setw(10) << meanMs(d) << std::setw(10) << delayPct(d, 98)
                      << std::setw(10) << d.back() / 1e6 << std::setprecision(2) << std::setw(16)
                      << 100.0 * over / d.size() << "\n";
        };
        for (std::size_t k = 0; k < flows.size(); ++k)
        {
            const VoiceFlow& f = flows[k];
            std::vector<int64_t> d = g_voiceQos.e2e[f.ue][f.dir].delayNs;
            std::sort(d.begin(), d.end());
            const double far98 = farUe ? delayPct(farRows[k].delayNs, 98) : remoteHostDelayMs;
            e2eLine(whoFlow(f), f.tx, f.rx, delayPct(satRows[k].delayNs, 98), far98, d);
            if (d.size() != f.rx || g_voiceQos.e2e[f.ue][f.dir].unstamped > 0)
            {
                notes << "  NOTE: UE " << f.ue << " " << dirName(f.dir) << ": the delay of "
                      << d.size() << " of " << f.rx << " received packets was measured\n";
            }
            allDelays.insert(allDelays.end(), d.begin(), d.end());
            allTx += f.tx;
            allRx += f.rx;
        }
        if (flows.size() > 1)
        {
            std::sort(allDelays.begin(), allDelays.end());
            e2eLine(who("all", "", ""), allTx, allRx, NAN, NAN, allDelays);
        }
        std::cout
            << "sent, rx: packets of the flow; lost: all of them, any cause. sat 98%: of the "
               "satellite UE's stretch (5QI table);\n"
            << "far 98%: of the far end's, "
            << (farUe ? "the terrestrial UE's (FAR END table)"
                      : "the N6 link host - UPF (its propagation delay)")
            << ". mean, 98%, max, over 150 ms: of the whole way.\n"
            << "G.114's 150 ms are mouth to ear: the codec, the jitter buffer and the playout "
               "come on top of what is here.\n"
            << notes.str();
    }

    // --csv: <csv>_pdcch.csv, one line for the run
    std::ostringstream csvPdcchHeader;
    std::ostringstream csvPdcch;

    // --- The PDCCH -------------------------------------------------------------------------------
    if (pdcch)
    {
        const NrPdcch::Stats stats = NrPdcch::Get(cellId)->GetStats();
        NS_ABORT_MSG_UNLESS(NrPdcch::Get(cellId)->GetCoresetRbs() == pdcchView->GetCoresetRbs(),
                            "the CORESET of the cell is not the one printed above");
        auto percent = [](uint64_t part, uint64_t whole) {
            return whole == 0 ? 0.0 : 100.0 * static_cast<double>(part) / static_cast<double>(whole);
        };
        const uint64_t slotsWithPdcch = stats.slots - stats.slotsBySymbols[0];
        uint64_t dcis = 0;
        uint64_t blocked = 0;
        for (std::size_t i = 0; i < NrPdcch::N_PURPOSES; ++i)
        {
            dcis += stats.dci[i];
            blocked += stats.blocked[i];
        }
        Frame("PDCCH",
              "t = " + Num(callsStart.GetSeconds(), 2) + " s to " + Num(callsEnd.GetSeconds(), 2) +
                  " s: " + std::to_string(stats.slots) + " slots, " +
                  std::to_string(slotsWithPdcch) + " (" + Num(percent(slotsWithPdcch, stats.slots), 2) +
                  " %) with a DCI");
        // The CCEs the PDCCH could have carried in these slots: every slot with the most PDCCH
        // symbols it may have (--pdcchSymbols, or 3 when each slot takes what it needs). Every
        // percentage of CCEs below is of this number, so the lines of a table add up to the
        // utilization.
        const uint32_t ccePerSlot = pdcchView->GetNumCce(pdcchView->GetMaxSymbols());
        const uint64_t ccePossible = stats.slots * ccePerSlot;
        NS_ABORT_MSG_IF(pdcchSymbols > 0 && ccePossible != stats.cceAvailable,
                        "the cell counted " << stats.cceAvailable << " CCEs in its slots, not "
                                            << ccePossible);
        std::cout << std::fixed << std::setprecision(2) << "CCEs: " << stats.cceUsed << " used of "
                  << ccePossible << " possible (" << stats.slots << " slots x " << ccePerSlot
                  << " CCEs) = utilization " << percent(stats.cceUsed, ccePossible) << " %; "
                  << dcis << " DCIs, " << (dcis ? static_cast<double>(stats.cceUsed) / dcis : 0.0)
                  << " CCEs each.\n";
        if (pdcchSymbols == 0)
        {
            uint64_t symbols = 0;
            std::cout << "PDCCH symbols:";
            for (uint32_t d = 1; d <= 3; ++d)
            {
                symbols += d * stats.slotsBySymbols[d];
                std::cout << (d > 1 ? "," : "") << " " << d << " in " << stats.slotsBySymbols[d]
                          << " slots (" << percent(stats.slotsBySymbols[d], slotsWithPdcch) << " %)";
            }
            std::cout << "; "
                      << (slotsWithPdcch ? static_cast<double>(symbols) / slotsWithPdcch : 0.0)
                      << " on average.\n"
                      << "The symbols the slots took (1 where there was no DCI) held "
                      << stats.cceAvailable << " CCEs; "
                      << percent(stats.cceUsed, stats.cceAvailable) << " % of these were used.\n";
        }
        std::cout << "\npurpose                     DCIs    CCEs  CCEs[%]  blocked  blocked[%]\n";
        auto purposeLine = [&](const std::string& name, uint64_t n, uint64_t cces, uint64_t stopped) {
            std::cout << std::left << std::setw(25) << name << std::right << std::setw(7) << n
                      << std::setw(8) << cces << std::setw(9) << percent(cces, ccePossible)
                      << std::setw(9) << stopped << std::setw(12) << percent(stopped, n + stopped)
                      << "\n";
        };
        for (std::size_t i = 0; i < NrPdcch::N_PURPOSES; ++i)
        {
            const auto purpose = static_cast<NrPdcch::Purpose>(i);
            const bool semiPersistent =
                purpose == NrPdcch::DL_SPS_ACTIVATION || purpose == NrPdcch::DL_SPS_RELEASE ||
                purpose == NrPdcch::UL_CG_ACTIVATION || purpose == NrPdcch::UL_CG_RELEASE;
            if (semiPersistent && scheduling == "dynamic")
            {
                continue;
            }
            purposeLine(NrPdcch::GetPurposeName(purpose), stats.dci[i], stats.cce[i], stats.blocked[i]);
        }
        purposeLine("all", dcis, stats.cceUsed, blocked);
        std::cout << "CCEs[%]: of the " << ccePossible << " CCEs possible. blocked: not sent, no "
                     "candidate had all its CCEs free; its UE was not\n"
                  << "scheduled in that slot.\n";

        const double target = AttributeDefault<DoubleValue>("ns3::NrPdcch", "TargetBler");
        const double margin = AttributeDefault<DoubleValue>("ns3::NrPdcch", "SinrMarginDb");
        std::cout << "\nlevel    DCIs  DCIs[%]    CCEs  CCEs[%]  from[dB]  missed  missed[%]\n";
        uint64_t received = 0;
        uint64_t missed = 0;
        for (std::size_t i = 0; i < pdcchLevels.size(); ++i)
        {
            const uint8_t al = pdcchLevels[i];
            if (pdcchView->GetCandidates(al) == 0 ||
                al > pdcchView->GetNumCce(pdcchView->GetMaxSymbols()))
            {
                continue;
            }
            received += stats.rxByAl[i];
            missed += stats.missedByAl[i];
            std::cout << std::setw(5) << +al << std::setw(8) << stats.dciByAl[i] << std::setw(9)
                      << percent(stats.dciByAl[i], dcis) << std::setw(8) << stats.cceByAl[i]
                      << std::setw(9) << percent(stats.cceByAl[i], ccePossible) << std::setw(10)
                      << pdcchView->GetSinrForBler(al, target) + margin << std::setw(8)
                      << stats.missedByAl[i] << std::setw(11)
                      << percent(stats.missedByAl[i], stats.rxByAl[i]) << "\n";
        }
        std::cout << "  all" << std::setw(8) << dcis << std::setw(9) << 100.0 << std::setw(8)
                  << stats.cceUsed << std::setw(9) << percent(stats.cceUsed, ccePossible)
                  << std::setw(10) << "" << std::setw(8) << missed << std::setw(11)
                  << percent(missed, received) << "\n"
                  << "DCIs[%]: of the DCIs. CCEs[%]: of the CCEs possible, as above. from: the level "
                     "is used from this reported SINR on.\n"
                  << "missed: DCIs their UE did not decode"
                  << (idealControl ? " (--idealControl=1: none can be)" : "") << ".\n";
        if (stats.aboveTarget > 0)
        {
            std::cout << "  NOTE: " << stats.aboveTarget << " DCIs went to a UE whose reported SINR "
                      << "was below what the highest level needs for " << 100.0 * target << " %\n";
        }
        if (stats.raised > 0)
        {
            std::cout << "  NOTE: " << stats.raised << " DCIs were sent at a higher level than the "
                      << "SINR asked for: that one had no free candidate\n";
        }
        if (stats.noReport > 0)
        {
            std::cout << "  NOTE: " << stats.noReport << " DCIs were sent at the highest level: "
                      << "their UE had not reported a SINR yet\n";
        }
        if (stats.ulCapped > 0)
        {
            std::cout << "  NOTE: " << stats.ulCapped << " of the blocked uplink grants had a free "
                      << "candidate but were held back to leave CCEs to the\n"
                      << "        downlink (NrPdcch::UlMaxShare)\n";
        }
        if (stats.unusedReservations > 0)
        {
            std::cout << "  NOTE: " << stats.unusedReservations << " PDCCH candidates were kept "
                      << "for a UE that then got no data resources in the slot\n";
        }

        if (!csv.empty())
        {
            // Every counter of the report, under a name of its own
            std::vector<std::pair<std::string, std::string>> c;
            auto add = [&c](const std::string& name, const std::string& value) {
                c.emplace_back(name, value);
            };
            auto addN = [&add](const std::string& name, uint64_t value) {
                add(name, std::to_string(value));
            };
            add("windowStartS", CsvNum(callsStart.GetSeconds()));
            add("windowEndS", CsvNum(callsEnd.GetSeconds()));
            addN("slots", stats.slots);
            addN("slotsWithDci", slotsWithPdcch);
            for (uint32_t d = 1; d <= 3; ++d)
            {
                addN("slotsWith" + std::to_string(d) + "Symbols", stats.slotsBySymbols[d]);
            }
            addN("ccePerSlot", ccePerSlot);
            addN("ccePossible", ccePossible);
            addN("cceInSymbolsTaken", stats.cceAvailable);
            addN("cceUsed", stats.cceUsed);
            add("utilizationPct", CsvNum(percent(stats.cceUsed, ccePossible)));
            addN("dcis", dcis);
            add("ccePerDci", CsvNum(dcis ? static_cast<double>(stats.cceUsed) / dcis : NAN));
            addN("blocked", blocked);
            add("blockedPct", CsvNum(percent(blocked, dcis + blocked)));
            const char* purposeName[NrPdcch::N_PURPOSES] = {"sib1",
                                                            "otherSi",
                                                            "paging",
                                                            "rar",
                                                            "tpcGroup",
                                                            "rrcDl",
                                                            "rrcUl",
                                                            "dlNew",
                                                            "dlRetx",
                                                            "dlSpsActivation",
                                                            "dlSpsRelease",
                                                            "ulNew",
                                                            "ulRetx",
                                                            "ulCgActivation",
                                                            "ulCgRelease"};
            for (std::size_t i = 0; i < NrPdcch::N_PURPOSES; ++i)
            {
                addN(std::string(purposeName[i]) + "Dcis", stats.dci[i]);
                addN(std::string(purposeName[i]) + "Cces", stats.cce[i]);
                addN(std::string(purposeName[i]) + "Blocked", stats.blocked[i]);
            }
            for (std::size_t i = 0; i < pdcchLevels.size(); ++i)
            {
                const std::string al = "al" + std::to_string(pdcchLevels[i]);
                addN(al + "Dcis", stats.dciByAl[i]);
                addN(al + "Cces", stats.cceByAl[i]);
                addN(al + "Received", stats.rxByAl[i]);
                addN(al + "Missed", stats.missedByAl[i]);
                const bool usable =
                    pdcchView->GetCandidates(pdcchLevels[i]) > 0 &&
                    pdcchLevels[i] <= pdcchView->GetNumCce(pdcchView->GetMaxSymbols());
                add(al + "FromDb",
                    usable ? CsvNum(pdcchView->GetSinrForBler(pdcchLevels[i], target) + margin)
                           : "");
            }
            addN("received", received);
            addN("missed", missed);
            add("missedPct", CsvNum(percent(missed, received)));
            addN("belowHighestLevel", stats.aboveTarget);
            addN("raisedLevel", stats.raised);
            addN("noReport", stats.noReport);
            addN("ulCapped", stats.ulCapped);
            addN("unusedReservations", stats.unusedReservations);
            const char* eventName[NrPdcch::VOICE_CHANGES] = {"dlSpsUsed",
                                                             "dlSpsEmpty",
                                                             "dlSpsInPieces",
                                                             "dlSpsOwnDci",
                                                             "dlSpsLost",
                                                             "ulCgUsed",
                                                             "ulCgEmpty",
                                                             "ulCgInPieces",
                                                             "ulCgOwnDci",
                                                             "ulCgLost",
                                                             "ulCgConfirmations",
                                                             "ulCgConfirmationBytes"};
            for (std::size_t i = 0; i < NrPdcch::VOICE_CHANGES; ++i)
            {
                addN(eventName[i], stats.events[i]);
            }
            const char* changeName[NrPdcch::N_CHANGE] =
                {"Seen", "InBlock", "InPieces", "PiecePdus", "OwnDci", "NewConfiguration"};
            for (const bool isDl : {true, false})
            {
                for (const bool toVoice : {false, true})
                {
                    for (std::size_t w = 0; w < NrPdcch::N_CHANGE; ++w)
                    {
                        addN(std::string(isDl ? "dl" : "ul") + (toVoice ? "ToVoice" : "ToSilence") +
                                 changeName[w],
                             stats.events[NrPdcch::ChangeEvent(isDl,
                                                               toVoice,
                                                               static_cast<NrPdcch::Change>(w))]);
                    }
                }
            }
            csvPdcchHeader << csvRunHeader;
            csvPdcch << csvRun;
            for (const auto& [name, value] : c)
            {
                csvPdcchHeader << "," << name;
                csvPdcch << "," << value;
            }
            csvPdcchHeader << "\n";
            csvPdcch << "\n";
        }

        if (spsScenario != 0)
        {
            // Semi-persistent scheduling: the blocks that needed no DCI
            Frame("SPS AND CONFIGURED GRANT, SCENARIO " + std::to_string(spsScenario),
                  "the blocks of the voice bearers that needed no DCI, and the DCIs that set them up");
            std::cout << "direction     activations  releases    used   empty  empty[%]  in pieces  "
                         "own DCI    lost\n";
            auto spsLine = [&](const char* name,
                               NrPdcch::Purpose activation,
                               NrPdcch::Purpose release,
                               NrPdcch::Event used,
                               NrPdcch::Event empty,
                               NrPdcch::Event segment,
                               NrPdcch::Event large,
                               NrPdcch::Event lost) {
                const uint64_t nUsed = stats.events[used];
                const uint64_t nEmpty = stats.events[empty];
                std::cout << std::left << std::setw(12) << name << std::right << std::setw(13)
                          << stats.dci[activation] << std::setw(10) << stats.dci[release]
                          << std::setw(8) << nUsed << std::setw(8) << nEmpty << std::setw(10)
                          << percent(nEmpty, nUsed + nEmpty) << std::setw(11)
                          << stats.events[segment] << std::setw(9) << stats.events[large]
                          << std::setw(8) << stats.events[lost] << "\n";
            };
            spsLine("downlink SPS",
                    NrPdcch::DL_SPS_ACTIVATION,
                    NrPdcch::DL_SPS_RELEASE,
                    NrPdcch::DL_SPS_BLOCK,
                    NrPdcch::DL_SPS_EMPTY,
                    NrPdcch::DL_SPS_SEGMENT,
                    NrPdcch::DL_SPS_LARGE,
                    NrPdcch::DL_SPS_LOST_CONFIG);
            spsLine("uplink CG",
                    NrPdcch::UL_CG_ACTIVATION,
                    NrPdcch::UL_CG_RELEASE,
                    NrPdcch::UL_CG_BLOCK,
                    NrPdcch::UL_CG_EMPTY,
                    NrPdcch::UL_CG_SEGMENT,
                    NrPdcch::UL_CG_LARGE,
                    NrPdcch::UL_CG_LOST_CONFIG);
            std::cout << "CG: configured grant Type 2. used: occasions that carried data (the "
                         "first block of a configuration comes with its\n"
                      << "activation DCI and is not among them). empty: an occasion with nothing to "
                         "send. in pieces: a packet larger than the\n"
                      << "block, sent over this occasion and the next. own DCI: a larger packet "
                         "that was sent with a DCI instead. lost: blocks\n"
                      << "the UE did not know of, it had missed the DCI that activated the "
                         "configuration. in pieces and own DCI count every\n"
                      << "such packet; the next table has those that follow a change of the "
                         "speaker.\n";
            const uint64_t confirmations = stats.events[NrPdcch::UL_CG_CONFIRMATION];
            if (confirmations > 0)
            {
                std::cout << "uplink: " << confirmations << " MAC CEs of "
                          << (spsScenario == 2 ? 6 : 1) << " B confirmed these DCIs, "
                          << stats.events[NrPdcch::UL_CG_CONFIRMATION_BYTES] << " B in all"
                          << (spsScenario == 2
                                  ? "; one CE confirms a release and the activation that came with it"
                                  : "")
                          << ".\n";
            }

            // The speakers start and stop talking: what the gNB did with the first packet after
            // each change, and what that took
            Frame("VOICE <-> SILENCE, SCENARIO " + std::to_string(spsScenario),
                  "how the first packet after the speaker started or stopped talking was sent");
            std::cout << "direction  change            count  in a block  in pieces  RLC PDUs  "
                         "own DCI  new configuration  DCIs\n";
            uint64_t unsent = 0;
            auto changeLine = [&](const char* name, bool isDl, bool toVoice) {
                auto n = [&](NrPdcch::Change what) {
                    return stats.events[NrPdcch::ChangeEvent(isDl, toVoice, what)];
                };
                const uint64_t sent = n(NrPdcch::CHANGE_BLOCK) + n(NrPdcch::CHANGE_PIECES) +
                                      n(NrPdcch::CHANGE_DCI);
                unsent += n(NrPdcch::CHANGE_SEEN) > sent ? n(NrPdcch::CHANGE_SEEN) - sent : 0;
                std::cout << std::left << std::setw(11) << name << std::setw(16)
                          << (toVoice ? "silence -> voice" : "voice -> silence") << std::right
                          << std::setw(7) << n(NrPdcch::CHANGE_SEEN) << std::setw(12)
                          << n(NrPdcch::CHANGE_BLOCK) << std::setw(11) << n(NrPdcch::CHANGE_PIECES)
                          << std::setw(10) << n(NrPdcch::CHANGE_PIECE_PDUS) << std::setw(9)
                          << n(NrPdcch::CHANGE_DCI) << std::setw(19) << n(NrPdcch::CHANGE_SWITCH)
                          << std::setw(6)
                          << n(NrPdcch::CHANGE_DCI) + 2 * n(NrPdcch::CHANGE_SWITCH) << "\n";
            };
            changeLine("downlink", true, false);
            changeLine("downlink", true, true);
            changeLine("uplink", false, false);
            changeLine("uplink", false, true);
            std::cout << "count: changes the gNB saw, by the size of the first packet after each (a "
                         "voice frame or a SID). in a block, in pieces,\n"
                      << "own DCI: how that packet was sent. in a block: whole, in a block of the "
                         "configuration. in pieces: larger than the block,\n"
                      << "cut by the RLC and sent in this block and the next, 20 ms apart, as so "
                         "many RLC PDUs. own DCI: with a dynamic assignment\n"
                      << "or grant, 1 DCI (uplink: for what a block left of it, or for its "
                         "scheduling request; so comes every packet while no\n"
                      << "configuration is active). new configuration: the gNB released the "
                         "configuration and activated the other one, 2 DCIs.\n"
                      << "DCIs: own DCI + 2 x new configuration. Uplink: as the gNB sees it (the "
                         "RLC PDUs are those it received; a late SID that it\n"
                      << "takes for the start of a talk spurt makes it change the configuration for "
                         "nothing, and back).\n";
            if (unsent > 0)
            {
                std::cout << "  NOTE: " << unsent << " of these first packets are in none of the "
                          << "three columns: discarded as late before they were sent, or the\n"
                          << "        call ended first\n";
            }
        }
    }

    // --- What each UE saw of the satellite during its call, sampled every 100 ms -----------------
    // The delay is the propagation delay of the whole path, one way: remote host (or far gNB) to
    // gateway, feeder link, service link.
    if (freezeGeometry)
    {
        Frame("SATELLITE SEEN FROM EACH UE", "it stands still: the same for the whole call");
        std::cout << "  UE      address  elevation[deg]  range[km]  delay[ms]\n";
    }
    else
    {
        // " ---- label ----", exactly as wide as the four columns below it
        auto group = [](const std::string& label, std::size_t width) {
            const std::size_t dashes = width - label.size() - 3;
            return " " + std::string(dashes / 2, '-') + " " + label + " " +
                   std::string(dashes - dashes / 2, '-');
        };
        Frame("SATELLITE SEEN FROM EACH UE DURING ITS CALL");
        std::cout << std::string(33, ' ') << group("elevation [deg]", 28)
                  << group("slant range [km]", 36) << group("delay [ms]", 28) << "\n"
                  << "  UE      address from[s]   to[s]";
        for (const char* column : {"start", "end", "low", "high"})
        {
            std::cout << std::setw(7) << column;
        }
        for (int width : {9, 7})
        {
            for (const char* column : {"start", "end", "min", "max"})
            {
                std::cout << std::setw(width) << column;
            }
        }
        std::cout << "\n";
    }
    // (the UEs with a call)
    std::vector<uint32_t> callers;
    for (const VoiceFlow& f : flows)
    {
        if (callers.empty() || callers.back() != f.ue)
        {
            callers.push_back(f.ue);
        }
    }
    for (const uint32_t i : callers)
    {
        const CallView& view = g_view[i];
        std::cout << std::fixed << std::setw(4) << i << "  " << std::setw(11)
                  << addressOf(ueIfaces.GetAddress(i));
        if (freezeGeometry)
        {
            std::cout << std::setprecision(2) << std::setw(16) << view.elevationDeg.first
                      << std::setw(11) << view.rangeKm.first << std::setprecision(3)
                      << std::setw(11) << view.delayMs.first << "\n";
            continue;
        }
        std::cout << std::setprecision(2) << std::setw(8) << view.callStart.GetSeconds()
                  << std::setw(8) << view.callEnd.GetSeconds() << std::showpos << std::setw(7)
                  << view.sideAtStart * view.elevationDeg.first << std::setw(7)
                  << view.sideAtEnd * view.elevationDeg.last << std::noshowpos << std::setw(7)
                  << view.elevationDeg.min << std::setw(7) << view.elevationDeg.max;
        for (double v : {view.rangeKm.first, view.rangeKm.last, view.rangeKm.min, view.rangeKm.max})
        {
            std::cout << std::setw(9) << v;
        }
        std::cout << std::setprecision(3);
        for (double v : {view.delayMs.first, view.delayMs.last, view.delayMs.min, view.delayMs.max})
        {
            std::cout << std::setw(7) << v;
        }
        std::cout << "\n";
    }
    if (!freezeGeometry)
    {
        std::cout << "from, to: when the call started and ended. elevation: - before the satellite "
                     "is overhead, + after; low, high: of the call.\n";
    }
    std::cout << "delay: one way, " << path << ", propagation only.\n";

    // --- The SNR of each UE's two links during its call: lowest, mean, highest ------------------
    {
        const std::string ulRbs = std::to_string(static_cast<uint32_t>(g_ulSnrRbs));
        auto group = [](const std::string& label, std::size_t width) {
            const std::size_t dashes = width - label.size() - 3;
            return " " + std::string(dashes / 2, '-') + " " + label + " " +
                   std::string(dashes - dashes / 2, '-');
        };
        Frame("SNR OF EACH UE DURING ITS CALL", "one beam, no interference: SINR = SNR; in dB");
        std::cout << std::string(17, ' ') << group("downlink", 24)
                  << group("uplink on " + ulRbs + " RB", 24) << group("uplink as received", 24)
                  << "\n"
                  << "  UE      address";
        for (int g = 0; g < 3; ++g)
        {
            for (const char* column : {"min", "mean", "max"})
            {
                std::cout << std::setw(8) << column;
            }
        }
        std::cout << "  received  PUSCH\n";
        for (const uint32_t i : callers)
        {
            std::cout << std::fixed << std::setprecision(2) << std::setw(4) << i << "  "
                      << std::setw(11) << addressOf(ueIfaces.GetAddress(i));
            for (const SnrStats* st : {&g_dlSnr[i], &g_ulSnr[i], &g_ulRxSnr[i]})
            {
                for (double v : {st->minDb, st->MeanDb(), st->maxDb})
                {
                    if (st->n > 0)
                    {
                        std::cout << std::setw(8) << v;
                    }
                    else
                    {
                        std::cout << std::setw(8) << "-";
                    }
                }
            }
            std::cout << std::setw(10) << g_dlSnr[i].n << std::setw(7) << g_ulSnr[i].n << "\n";
        }
        std::cout << "downlink: the UE's, on the control symbols of each slot that has some "
                     "(they span all RBs of the carrier).\n"
                  << "uplink: the gNB's, on each PUSCH transmission of the UE (a repetition is "
                     "one), over its RBs. on "
                  << ulRbs << " RB: scaled to the UE's\n"
                  << "power in " << ulRbs
                  << " RB, as the CNR in the header; as received: in the RBs of the grant, which "
                     "share that power. mean: of the dB values.\n"
                  << "received: control symbols measured. PUSCH: transmissions measured.\n";
    }

    // --- Power control: what each UE measured and sent, and what the satellite took off ---------
    if (dlPowerRefDeg > 0.0 || ulPowerRefDeg > 0.0)
    {
        Frame(
            "POWER CONTROL",
            "sampled every 100 ms of the call: at its start, its end, and the lowest and highest");
        std::cout << "  UE      address   path loss[dB] start / end   UL power[dBm] start / end / "
                     "min / max   satellite cut[dB] start / end\n";
        auto pair = [](const Span& s, int width) {
            std::ostringstream o;
            o << std::fixed << std::setprecision(2);
            if (s.n == 0)
            {
                o << std::setw(width) << "-" << std::setw(width) << "-";
            }
            else
            {
                o << std::setw(width) << s.first << std::setw(width) << s.last;
            }
            return o.str();
        };
        for (const uint32_t i : callers)
        {
            const CallView& view = g_view[i];
            std::cout << std::setw(4) << i << "  " << std::setw(11)
                      << addressOf(ueIfaces.GetAddress(i)) << "  " << pair(view.ulLossDb, 13)
                      << "  " << pair(view.ulPowerDbm, 10);
            if (view.ulPowerDbm.n == 0)
            {
                std::cout << std::setw(8) << "-" << std::setw(8) << "-";
            }
            else
            {
                std::cout << std::setprecision(2) << std::setw(8) << view.ulPowerDbm.min
                          << std::setw(8) << view.ulPowerDbm.max;
            }
            std::cout << "  " << pair(view.dlCutDb, 12) << "\n";
        }
        std::cout << "path loss: what the UE measured on the downlink, its beam gain in it "
                     "(--ulPowerRefDeg; reference "
                  << Num(g_scene.ulRefLossDb, 2) << " dB).\n"
                  << "UL power: what it sent with. "
                  << "satellite cut: what the satellite took off its power for the whole beam "
                     "(--dlPowerRefDeg). -: that control is off.\n";
    }

    // --- The far end, --farEnd=ue: where each terrestrial UE was, and what it received ---------
    if (farUe)
    {
        Frame("FAR END UEs",
              std::to_string(farCells) + (farCells == 1 ? " cell, " : " cells, ") + farEndBand +
                  ", " + Plain(farEndBandwidthMHz) + " MHz; distance and direction from the gNB");
        std::cout << "  UE  far address  cell  distance[m]  azimuth[deg]  DL SINR[dB]\n";
        for (uint32_t k = 0; k < nUes; ++k)
        {
            std::cout << std::setw(4) << k << std::setw(13) << addressOf(farIfaces.GetAddress(k))
                      << std::setw(6) << k / farUesPerCell << std::setprecision(1) << std::setw(13)
                      << farDistanceM[k] << std::setw(14) << farAzimuthDeg[k] << std::setw(13)
                      << (g_farSinrN[k] ? 10 * std::log10(g_farSinrSum[k] / g_farSinrN[k]) : NAN)
                      << "\n";
        }
        std::cout
            << "UE: the satellite UE the terrestrial UE talks with. azimuth: from the boresight "
               "of the gNB's panel. DL SINR: of the control\n"
            << "symbols received.\n";
    }

    g_scene = Scene(); // let go of the nodes before the simulator is destroyed
    // --csv: the nr module prints the ROHC tables when the simulator is destroyed; they are read
    // from a copy of what it prints
    std::ostringstream destroyed;
    std::streambuf* coutBuffer = csv.empty() ? nullptr : std::cout.rdbuf(destroyed.rdbuf());
    Simulator::Destroy();
    if (!csv.empty())
    {
        std::cout.rdbuf(coutBuffer);
        std::cout << destroyed.str();
        WriteCsvFile(csv + "_calls.csv",
                     csvRunHeader +
                         ",flow,ue,dir,address,bearer,rnti,northKm,eastKm,beamDb,sfDb,vaf,losPct,"
                         "rlf,outS,dlSinrDb,dlSnrMinDb,dlSnrMeanDb,dlSnrMaxDb,ulSnrMinDb,"
                         "ulSnrMeanDb,ulSnrMaxDb,tx,rx,lost,noConn,discard,radio,rohcLost,"
                         "linkLossPct,rohcLossPct,e2eDelayMs,jitterMs,onBearer,inPdb,late,"
                         "overPdbPct,perPct,delayMeanMs,delayP50Ms,delayP90Ms,delayP95Ms,"
                         "delayP98Ms,delayP99Ms,delayMaxMs,kbps,missedPdb,missedPer,satisfied,dcis,"
                         "dciCces,dcisMissed,"
                         "dciMissedPct,farLost,farOnBearer,farDiscard,farRadio,farRohc,"
                         "farDelayMeanMs,farDelayP98Ms,e2eMeanMs,e2eP50Ms,e2eP98Ms,e2eMaxMs,"
                         "e2eOver150Pct,callStatus,releasedAtS",
                     csvCalls.str());
        WriteCsvFile(csv + "_control.csv",
                     csvRunHeader +
                         ",admission,admissionRbPct,admissionCcePct,dropCalls,dropOnTimePct,"
                         "offered,admitted,"
                         "blocked,blockedDlRbs,blockedUlRbs,blockedCces,released,peakAdmitted,"
                         "meanDlRbs,meanUlRbs,meanCces,capacityDlRbs,capacityUlRbs,capacityCces,"
                         "peakDlPct,peakUlPct,peakCcePct",
                     csvRun + "," + csvControl.str());
        WriteCsvFile(csv + "_delays.csv",
                     csvRunHeader + ",flow,ue,dir,delayMs,packets",
                     csvDelays.str());
        std::string written = csv + "_calls.csv, _control.csv, _delays.csv";
        if (csvPackets)
        {
            WriteCsvFile(csv + "_packets.csv",
                         csvRunHeader + ",flow,ue,dir,bearer,delayMs",
                         csvPacketLines.str());
            written += ", _packets.csv";
        }
        if (pdcch)
        {
            const std::string header = csvPdcchHeader.str();
            WriteCsvFile(csv + "_pdcch.csv",
                         header.empty() ? csvRunHeader : header.substr(0, header.size() - 1),
                         csvPdcch.str());
            written += ", _pdcch.csv";
        }
        if (rohc)
        {
            WriteRohcCsv(destroyed.str(), csv, csvRunHeader, csvRun);
            written += ", _rohc.csv, _rohc_types.csv";
        }
        std::cout << "\nCSV: " << written << "\n";
    }
    return 0;
}
