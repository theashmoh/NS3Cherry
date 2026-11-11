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
 *     --ulPolarizationLossDb adds a loss on the uplink only: TR 38.821 6.1.1.1 asks 3 dB of
 *       depolarisation for a handheld when the satellite receives circular polarisation with
 *       polarisation re-use, and 0 dB with dual polarisation per beam (default 0). R1-2205058's
 *       "additional loss" of 8 dB covers the antenna gain, the polarisation mismatch and other
 *       losses together; for its Table 2 use --ueAntennaGainDbi=-8, which applies it on both links.
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
 *     satellite has risen to --elevation above it (10 deg unless --elevation is given) and ends
 *     when it has sunk to that again (the gateway has to see it at 10 deg too, see above), and
 *     the run lasts as long as that call. --elevation=30 is a shorter pass: from 30 deg up, over
 *     the top and down to 30 deg. Call i is on only while the satellite is that high above UE 0
 *     and above UE i both. --freezeGeometry and --duration are not used.
 *   - The calls: UE 0's call lasts --duration from t = 1 s, less what the gateway does not allow
 *     (see above). No call starts before it and none goes on after it: call i starts
 *     i x --startOffsetMs after it and ends with it.
 *     When the satellite flies, a call is also on only while the satellite is 10 deg or more
 *     above its own UE (--fullPass=1: --elevation or more): it starts when the satellite has
 *     risen to that above that UE, if that is later, and ends when the satellite has sunk to it
 *     above it again, if that is before UE 0's call ends. A UE that never sees the satellite
 *     that high during UE 0's call has no call. When the satellite stands still nothing rises
 *     or sinks, and every call runs to the end of UE 0's wherever its UE is in the beam.
 *   - Doppler. The path delay changes while the satellite flies: the timing advance follows it,
 *     and a downlink signal arrives compressed by the delay drift over its duration. The carrier
 *     frequency offset itself (UE 0: up to 45 kHz at 10 deg on the 2 GHz downlink) is not
 *     modelled: the UE takes it off, from its GNSS position and the satellite's ephemeris (TS
 *     38.300 16.14.2), and the 0.1 ppm it may leave (TS 38.101-5), 218 Hz at 2.175 GHz = 1.5 %
 *     of a subcarrier, leaks about 32 dB below the signal into the other subcarriers (ICI of a
 *     frequency offset e in subcarriers: (pi e)^2 / 3). The header gives UE 0's Doppler.
 *   - --fastFading=model: ns-3.48's channel model draws a new realisation (K-factor included)
 *     whenever the satellite has moved 1 m, here every 1 to 2 ms when it flies. When it stands
 *     still, the realisation is drawn once per line-of-sight state, and the Doppler term moves
 *     it (at 30 deg, --RngRun=1: 0.1 to 3.6 dB) around a mean of its own: the mean SINR of the
 *     run is that of one draw (--RngRun=1 to 8: -0.6 to +2.5 dB from the CNR). Let the
 *     satellite fly for a fading that averages out.
 *   - --fadingHoldMs=T (--fastFading=model; 0 = off, the default): each UE's fading stays for T
 *     ms, then a new one is drawn, for every UE at the same instants. Why: a fading that is new
 *     every 1 to 2 ms is a channel nobody can follow (CQI and its outer loop are 26 slots plus a
 *     round trip old when they are used), and it is not what TR 38.811 has. TR 38.811 6.9.2
 *     puts the Doppler of the satellite on all taps alike, one shift of the whole link (the
 *     carrier offset the UE takes off); what fades is the Doppler of the UE's own motion
 *     (TR 38.901 7.7.2), whose coherence time is about 0.423 / f_D: 76 ms at 3 km/h and 2 GHz.
 *     TR 38.811 7.2 expects a fading of 100 ms coherence at most, for HAPS. The fading here is
 *     a new draw of ns-3's channel every T ms, and the satellite's Doppler is the same for all
 *     of its clusters, so that it is one phase and does not fade inside the period. The UEs
 *     stand still: nothing moves a fading within T. Try 100.
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
 *             had waited longer than the discard timer (see "5QI 1" below); or, with
 *             --ueBufferAgeS, the UE took it off its queue for having waited too long
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
 * --ueBufferAgeS=T (0 = never, the default) is a rule for the satellite UEs' queues only: a
 * voice packet that has waited T seconds for the radio is taken off the queue and lost (a
 * "discard"), so that when the link comes back it is not the old speech that goes first, the
 * speech nobody would hear any more. It acts on the oldest packets, as a real voice client
 * does not keep a recording; the discard timer above refuses the new one. Try 2 to 5.
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
 *     With --pdcch=1 these two levels are not used: NrUePhy::RlmFromPdcchTable=1 (the default)
 *     takes the two hypothetical PDCCHs of TS 38.133 8.1.2 from the PDCCH's own error table.
 *     Out of sync where a level-8 PDCCH with 4 dB more power would lose over 10 % (about
 *     -13.8 dB), in sync again where a level-4 PDCCH loses under 2 % (about -5.9 dB); the
 *     header prints both. --ns3::NrUePhy::RlmFromPdcchTable=0 gives Qout and Qin back.
 *     N310, N311 and T310 count the same way either way.
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
 *   row, so that one lost packet does not cost the context (the ROHC library sends 4). The
 *   answer to a STATIC-NACK is different: the decompressor has asked, one IR packet restores its
 *   context (--rohcRepairIrPackets=1). More of them are no use unless that one is lost, and each
 *   is 56 bytes where a voice packet is 3 + the voice.
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
 *     and a STATIC-NACK with --rohcRepairIrPackets IR packets (1). No periodic refresh once
 *     feedback has arrived.
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
 *     or the ACK of its next IR refresh arrives. It does not go back to U-mode by itself: U-mode
 *     comes back with the new connection after a radio link failure, or with --rohcFallbackAfter=n.
 *   --rohcFallbackAfter=n (0 = off, the default; O-mode and the size table): a decompressor that
 *     has not restored n packets in a row gives up on the feedback loop and asks the compressor
 *     for U-mode (RFC 3095 5.6: the decompressor decides the mode, by the Mode field of its
 *     feedback). Its feedback says so, with the NACK or STATIC-NACK it sends, until an IR or a
 *     context update of the compressor tells it that U-mode has begun. From then on the
 *     compressor repairs by itself, with its refreshes (--ns3::NrPdcp::RohcIrRefresh,
 *     RohcFoRefresh, RohcIrRefreshTime), and the decompressor sends no feedback at all. The pair
 *     stays in U-mode. It starts the refresh counts anew and sends the context updates a NACK
 *     asks for. The ROHC library does not have the transition from O-mode to U-mode, so this is
 *     for the size table. The mode table shows it ("back in U-mode").
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
 *   - DCI: --dciBits (34) plus a 24-bit CRC for those of data. The cell's own (SIB1, other SI,
 *     paging, RAR, DCI 2_2, RRC) have the size of TS 38.212 7.3.1: format 1_0 has 28 bits and a
 *     frequency allocation field (9: 37 bits on 24 or 25 RB). The reserved bits of the SI,
 *     paging and random access DCIs, and the zeros that pad formats 0_0 and 2_2, bring them
 *     all to that size (NrPdcch::CellDciBits to change). A CCE has 108 coded bits (54 QPSK
 *     symbols).
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
 *                        scheduled by DCIs gets its commands in them). It is the group TPC
 *                        DCI of TS 38.212 7.3.1.3.3: 2 bits for each UE in the payload of a
 *                        format 1_0, so 18 UEs fit (13 is a choice of this model)
 *       RRC              RrcSetupDcis (4:5, downlink:uplink) when a UE connects (RRCSetup,
 *                        security, capabilities, reconfiguration, with the answers),
 *                        RrcCallDcis (1:1) when its call starts (the reconfiguration of the
 *                        QoS flow), one every RrcStepMs (20); a measurement report every
 *                        MeasReportPeriodMs (0: none)
 *     One that finds no free candidate waits for the next slot.
 *   - who comes first in a slot: the cell's DCIs (common search space, level 8 (CommonAl, or
 *     Sib1Al, OtherSiAl, PagingAl, RarAl, TpcAl for one of them: 4, 8 or 16) or the highest
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
 *   - where a CCE goes in the CORESET is not a setting. TS 38.211 7.3.2.2 numbers the REGs time
 *     first, so a CCE of 6 REGs covers 6 RBs of one symbol, 3 RBs of two or 2 RBs of three;
 *     with interleaving (cce-REG-MappingType) the REG bundles of a DCI are spread over the whole
 *     CORESET. Which CCEs a DCI takes (the candidate hash of TS 38.213 10.1) is modelled;
 *     where their REGs are is not, because it changes nothing here: the CORESET (24 RBs, 4.3 MHz)
 *     is narrower than the coherence bandwidth of an NTN channel (delay spread of tens of ns),
 *     and the fading does not move within a slot, so there is no frequency or time diversity
 *     to gain, and every CCE of a CORESET spans the same symbols. What changes the PDCCH
 *     is the level, the CORESET (--pdcchSymbols, CoresetRbs) and the power:
 *   - PDCCH power boost, --ns3::NrPdcch::BoostMaxDb=X (0 = off): the CCEs a slot does not use
 *     give their energy to those it uses, 10 log10(CCEs of the CORESET / CCEs used) dB at most X
 *     (and what --dlPfdLimit allows an RB). The UE needs no signalling: it assumes the same
 *     energy for the DM-RS and the data of a PDCCH. The level is chosen without the boost;
 *     --ns3::NrPdcch::SinrMarginDb=-Y counts on Y dB of it when choosing.
 *   Other settings are attributes: --ns3::NrPdcch::CoresetRbs, Candidates, TargetBler,
 *   SinrMarginDb, SinrOffsetDb, TableDciBits, CommonAl, Sib1Al, OtherSiAl, PagingAl, RarAl,
 *   TpcAl, CellDciBits, RaiseAl, UlMaxShare, BoostMaxDb and those of the cell's DCIs above
 *   (--ns3::NrPdcch::PagingRate=20 and so on).
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
 * 5.4.4 has it start a random access, in which a connected UE keeps its C-RNTI (Msg3 carries the
 * C-RNTI MAC CE, 5.1.4 and 6.1.3.2; the contention is resolved on it, 5.1.5); the nr
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
 * blocks to --puschRepSinrDb (2 dB), or more if the block of a slot at the MCS they give would be
 * under the least block a UE sends data in (15 B: 10 for the MAC subheader and the RLC entity, 5
 * for the buffer status report that goes in every block; 12 B with --ns3::NrUeMac::CompactBsr=1,
 * the 2-byte short BSR of TS 38.321 6.1.3.1) or under --puschMinBlockBytes (a voice frame needs
 * about 26 B, 23 with the compact BSR); 1: no repetition; 2 to 32: that K for every
 * block (only the numbers above are accepted, those of TS 38.331 numberOfRepetitionsExt-r17; with
 * --ulWideRbs a fixed K is divided by the widening and goes up to the next of them: 9 / 2 = 4.5
 * is 7). The MCS is the highest the error model decodes with 10 % block errors after the K
 * transmissions (then the outer loop). A repetition takes the resource blocks of the first
 * transmission if they are free, others if not. UE 0 at 30 deg with the -5 dBi handheld: K = 3, MCS
 * 9 = 33 B. A configured grant (--scheduling=sps1, sps2) repeats its blocks too, repK times (TS
 * 38.214 6.1.2.3: K is numberOfRepetitions of the time domain allocation row if it has one, else
 * repK, 1, 2, 4 or 8; here the numbers above, but not more than the slots of the period, 6.1.2.3.1):
 * the fewest of these that bring K times the SINR to --puschRepSinrDb (with a fixed K, the first
 * that is at least K), or
 * more if the block does not fit into its RBs at the MCS they give; set when the configuration is
 * activated, which is done anew when the link needs more, or with 1 dB to spare fewer. Without them
 * a block of 26 B would not fit into 2 RB at the MCS of one transmission, and no UE of the -5 dBi
 * handheld would get a configured grant. The error models have curves down to the lowest code rate
 * of a modulation (NrLteMiErrorModel: of 4 transmissions of a block); below it, which a block at
 * MCS 0 reaches with its first retransmission, a block takes the lowest curve of its modulation
 * with its mutual information (NrLteMiErrorModel) or its effective SINR (NrEesmIrT1, T2) scaled by
 * that curve's code rate over its own.
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
 *   --dlPfdLimit, the power flux density on the ground that the network operator allows (S band,
 *   coexistence with terrestrial networks), in dBW/m2/MHz (0: off, the default): an RB never
 *   sends more than that PFD at the beam centre, whose distance is the satellite's slant range:
 *   EIRP density = PFD + 10 log10(4 pi d^2) + --dlPfdMarginDb per MHz, an RB (180 kHz) of it,
 *   less the antenna gain. The margin, 5.25 dB, is what TR 38.821's Set 1 loses in shadowing,
 *   gases and scintillation: -90 dBW/m2/MHz is then 41.80 dBW/MHz at 90 deg (600 km), 34.35 dBm
 *   on an RB; the limit rises 20 log10(d / 600 km), with the satellite's slant range d. It
 *   turns --dlBoostMaxDb's mechanism on: an RB in use takes the power of the idle RBs of its
 *   symbol up to that limit (and --dlBoostMaxDb, if given), never more than the satellite's
 *   TxPower in all; the limit is followed every 100 ms when the satellite flies. The control
 *   symbols obey it too. --dlRbPowerMinDbm (dBm, 0: off): the lowest power of an RB in use, under
 *   which --dlPowerRefDeg takes no RB; without that control the least an RB gets is its share.
 *   Closed loops on the power, for the SNR of a UE. They sit on top of the settings above and
 *   change none of them; with both off a run is the same as without them.
 *   --dlClosedLoop, downlink: a cap on how far over its share the satellite sends the RBs of
 *   each UE, so that no power goes to a UE that already receives well. 1: the UE should have an
 *   SNR of at least --dlSnrMinDb on its data; 2: an MCS of at least --dlMcsMin (the SNR at which
 *   the scheduler's AMC gives that MCS on the RBs the UE gets); 3: both. The boost is still the
 *   power of the idle RBs of the symbol and still bound by --dlBoostMaxDb and the PFD limit (one
 *   of them must be given): the loop only keeps it from going higher than the UE needs. Once per
 *   --closedLoopMs the UE's mean SNR over the period, less the boost its RBs had, is its SNR at
 *   its share; filtered over the periods (about three), against the target it is the boost the
 *   UE needs. The UE's SNR is its target plus what the cap has over that need: it is in a band
 *   one step wide (--closedLoopStepDb) above the target. Short of the target, the cap goes up at
 *   once, to the upper side of the band; with a step to spare it goes down by a step at most, to
 *   the lower side (20 % of the step from the edge); in the band nothing changes. It never goes
 *   under 0 or over --dlBoostMaxDb (30 dB when only the PFD limit is given). (Going to the
 *   middle of the band, as an earlier version did, holds the SNR in the upper half of its band
 *   while the satellite rises and in the lower half while it sets: the rising half of a pass
 *   then reads a band's width higher than the setting half. What is left of that difference is
 *   the landing point times the band's width, --closedLoopLanding (0.2: 0.2 dB for the 1 dB band;
 *   0 would be the edge itself), plus twice the lag of the estimate on a channel that drifts,
 *   avg x period x rate: --closedLoopAvg periods (3) at 1 s and the 0.05 dB/s of the middle of a
 *   pass at 600 km are 0.15 dB each way. Shorter periods and fewer averaged make it smaller, and
 *   the loop noisier with fast fading.) A new connection has the whole
 *   boost until its first decision, which goes to the need plus half a step at once. The cap follows an estimate
 *   of the need and not the sum of the errors, so it does not wind up when the satellite has no
 *   power to give. The SNR of one reception scatters with the fast fading (3 dB or so): the loop
 *   keeps the mean of a period at the target, not each reception, and the report says how many
 *   receptions were under it. A UE that cannot reach its target with the whole boost is
 *   counted in the "top" column. NrGnbPhy::SetUeMaxRbBoostDb is the cap's hook in the nr module.
 *   --ulClosedLoop, uplink: the closed loop of TS 38.213 7.1.1 with the accumulated command f of
 *   Table 7.1.1-1. The UE lowers its power while the SNR the gNB measures on its PUSCH is above
 *   --ulSnrHighDb, but not so far that the SNR falls under --ulSnrLowDb; if it is under that, the
 *   power goes up again. The power is never above what the UE sends without the loop (23 dBm, or
 *   the open-loop power of --ulPowerRefDeg) and never under --ulPowerMinDbm (-40 dBm, TS 38.101-1
 *   Table 6.3.1-1 for 5 MHz). Once per --closedLoopMs: the UE's SNR at the power it would have
 *   without the loop (filtered over the periods) says what f brings the SNR to the far side of
 *   the band (20 % of its width in from the edge it did not cross: the same mean SNR whether the
 *   channel gets better or worse); f moves toward it by one step down at most (command -1 dB) or
 *   three up (+3 dB), and not at all while the SNR is in the band. The SNR is that of the uplink table as if the UE had
 *   sent on --ulGrantRbs RBs (2 with --ulGrantRbs=0), whatever the grant it got. A UE spreads its
 *   power over the RBs of its grant; the loop changes the total. --ulWideSinrDb works on the
 *   same SNR: a band at or under it takes the wide grants away from the UEs the loop has
 *   lowered. The PUCCH and the other uplink control have no error model in the nr module: they
 *   are not affected by a UE's lower power. Both loops start from scratch with every new
 *   connection (after a radio link failure, a release and --retry).
 * Uplink link adaptation, --ulLinkAdaptation=1 (default). The UE spreads its power over the
 * resource blocks of its grant: the MCS of a grant is that of the
 * SINR the gNB measured on the UE's last transmission, scaled by the resource blocks then over
 * those now, and an outer loop on the decoding results (the same steps as in downlink; blocks
 * whose grant the UE missed do not count) keeps the first transmissions at --targetBler. The
 * outer loop lowers the MCS no further than the least block (15 B, or --puschMinBlockBytes): a
 * smaller block would not hold a voice frame and every frame would take two blocks.
 * --ulSinrAvgMs: a first-order average of the SINR reports (default 0: the last reception).
 * --ulLinkAdaptation=0, the nr module's: the MCS of that SINR as it was, whatever the size of
 * the grant, and no outer loop: a UE that gets more resource blocks than it was measured on
 * loses about half its first transmissions.
 *
 * Wide uplink grants, --ulWideRbs=W (OFDMA, --ulLinkAdaptation=1, W above --ulGrantRbs; 0: off,
 * the default): a UE sends in W resource blocks instead of --ulGrantRbs when its uplink allows,
 * and the block is sent in fewer slots: twice the RBs carry the bytes in about half the slots
 * but have 3 dB less SINR per RB (a fixed --puschRepetitions is divided by the widening; with
 * 0 the repetitions follow from the SINR of the wide grant). The rule, one grant at a time:
 * the UE gets its W if (1) its SINR on ONE RB, from its last PUSCH, is at least --ulWideSinrDb
 * (5: with W = 2 one transmission is then 2 dB, --puschRepSinrDb, on each RB, no repetition
 * needed) and (2) the slot has room: every UE waiting has its first RB and the uplink RBs in
 * use in the slot (repetitions and retransmissions of earlier blocks, the grants so far, this
 * one) are at most --ulWideLoadPct (50) % of the band. Otherwise it gets --ulGrantRbs with its
 * K repetitions: a power-limited UE sends more bits per resource block slot on few RBs, and
 * when UEs wait for RBs the narrow grant carries more calls. A UE changes back and forth from
 * grant to grant, there is no state. With a fixed --puschRepetitions above 1 a UE that passes
 * (1) and (2) gets W RBs and K / W repetitions even if its first RB would carry all it has;
 * with 0 it gets the second RB only while its first does not hold what it has to send. A
 * block has one DCI, whatever its repetitions: those in the slots after it have none (TS
 * 38.214 6.1.2.1). The WIDE UPLINK GRANTS table counts the PUSCH slots, repetitions included,
 * not DCIs; the DCIs are in the PDCCH report (--pdcch=1).
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
 *     the call is released: its voice ends and the gNB releases the UE's RRC connection; the UE
 *     goes to RRC_IDLE and stays there (TS 38.331 5.3.8), and connects again only to call
 *     again (--retry). Counted are the packets that arrived (within the PDB or
 *     late) and those the sender's RLC discarded for waiting too long: a full cell delays
 *     packets. A packet lost on the radio does not count: a bad link is not a full cell, and a
 *     link that fails is a radio link failure. Not while the UE is out of coverage after one,
 *     and only while the cell is loaded: the calls on reserve (as the admission counts) half
 *     of its downlink RBs, of its uplink RBs or of its CCEs or more. In a cell with room, late
 *     packets come from a bad link (HARQ retransmissions, repetitions), not from the load.
 *     The estimate of the admission has no room for what the cell sends of its own (SIB1, DCI
 *     2_2 ...) and for the DCIs that semi-persistent scheduling still needs; where these are
 *     many against a small PDCCH, the release takes care of the rest: 60 sps1 calls on 5 MHz
 *     (4 CCEs), --admissionRbPct=80: 41 admitted, 7 of them released.
 *   - retry, --retry=1 (default 0): a UE whose call was released, or blocked at admission,
 *     calls again. A call the gNB refuses counts as a release: the gNB releases the UE's RRC
 *     connection (without --retry the UE of a blocked call stays connected, with a bearer it
 *     does not use). --retrySeconds (3) after that the UE asks for a connection: it looks for
 *     the cell, does the random access and gets a new RNTI; its IP address stays (the core
 *     keeps it). When its bearers are up the gNB treats the new call as any other: admission
 *     may block it again, else the voice starts again on a new bearer, and the call is
 *     released again under the same rule. A UE makes at most --maxRetry (5) tries, the first
 *     call included (5: the call of the plan and up to 4 retries), whatever happened to the
 *     earlier ones. A radio link failure brings no retry: the call goes on by itself. No
 *     retry is made that would start after the call's window is over.
 *     With --rtp=0 nothing is released or blocked, so --retry has no effect.
 *     VOICE OF EACH CALL then lists every call of every UE, a line per direction, each against
 *     5QI 1 on its own bearer, and the mean of each UE's calls and of all calls (each call
 *     counts once); the VOICE AGAINST 5QI 1 table adds a UE's calls up, one line per bearer.
 *   The CALL CONTROL table lists the calls blocked and released, and with --retry every try.
 *
 * CSV files, --csv=<prefix>: the results also as CSV files, for plots. Every line starts with
 * the columns that identify the run (nUes, direction, scheduling, ..., farEnd, farEndBand,
 * ulGrantRbs, puschRepetitions, puschRepSinrDb, dlBoostMaxDb, dlPfdLimit, dlPowerRefDeg,
 * ulPowerRefDeg, ulWideRbs, dlClosedLoop, dlSnrMinDb, dlMcsMin, ulClosedLoop, ulSnrHighDb,
 * ulSnrLowDb, ulPowerMinDbm, closedLoopMs, closedLoopStepDb, errorModel, rngSeed, rngRun), so
 * the files of many runs can be
 * joined (sweep_nues.sh does that for a range of --nUes; plot_sweep.py draws them).
 *   <prefix>_calls.csv       a line per flow (dir: dl, ul) of a call (bearer 0) and per bearer
 *                            of it (1, 2, ...: a new one after each radio link failure, and
 *                            with --retry after each release): the
 *                            CALLS, LOST and 5QI tables, the delay percentiles, with --pdcch=1
 *                            the DCIs to the UE (two-way calls: those of the flow's direction),
 *                            the far end's stretch (far...) and the whole way (e2e...); the
 *                            call admitted, blocked or released (callStatus, releasedAtS)
 *   <prefix>_control.csv     the CALL CONTROL table: calls offered, admitted, blocked (and for
 *                            which resource), released (the calls of the plan); reservations;
 *                            --retry: the UEs that called again and their retries; one line
 *   <prefix>_tries.csv       --retry=1: a line per call (try) of a UE and direction (row=try):
 *                            when it was asked for, admitted and ended (askedS, fromS, toS),
 *                            its RNTIs, what became of it (outcome, blockedBy, onTimePct), and
 *                            the 5QI 1 columns of the packets on its bearers; then the mean of
 *                            each UE's calls (row=ue_mean), of all (all_mean), of the calls of
 *                            the plan (first_mean) and of the retries (retry_mean)
 *   <prefix>_delays.csv      per flow: the delivered packets per 1 ms of UE-UPF delay
 *   <prefix>_snr.csv         the SNR tables: a line per UE and elevation step (elevDeg: to the
 *                            nearest 10; negative while the satellite comes closer): samples,
 *                            min, mean, max of the downlink, the uplink on ulGrantRbs RBs and
 *                            as received
 *   <prefix>_dlpower.csv     --dlPfdLimit, --dlBoostMaxDb or --dlPowerRefDeg: the power of an RB
 *                            of the downlink allocations (dBm) and the SNR of the downlink data
 *                            (dB): a line per UE and elevation step, samples, min, mean, max of
 *                            each
 *   <prefix>_ulgrants.csv    --ulWideRbs: a line per UE: PUSCH slots, with the narrow grant's
 *                            RBs, the wide one's, others, and their RBs added up
 *   <prefix>_loop.csv        --dlClosedLoop, --ulClosedLoop: a line per UE: the downlink's
 *                            receptions, SNR, MCS, boost with and without the loop, the power
 *                            saved, the decisions up, down, hold, top; the uplink's PUSCH
 *                            transmissions, SNR, shares under / in / over the band, power with
 *                            and without the loop, the power saved, the decisions
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
 *   ./ns3 run "ntnlab_nr --bandwidthMHz=5 --dlPfdLimit=-90"
 *                                                      ... up to the PFD of -90 dBW/m2/MHz
 *   ./ns3 run "ntnlab_nr --ulGrantRbs=1 --ulWideRbs=2" 1 RB, 2 RB when the UE and the slot allow
 *   ./ns3 run "ntnlab_nr --freezeGeometry=0 --duration=100s --dlPowerRefDeg=30 --ulPowerRefDeg=30"
 *                                                      power control as the satellite flies
 *   ./ns3 run "ntnlab_nr --errorModel=NrEesmIrT1"      NR's MCS table and LDPC curves
 *   ./ns3 run "ntnlab_nr --RngRun=2"                   another drop (and other calls)
 *   ./ns3 run "ntnlab_nr --ueLayout=line"              UEs in a line, 2 km apart
 *   ./ns3 run "ntnlab_nr --ueLayout=line --elevation=60 --nUes=50 --spacingKm=0.4"
 *   ./ns3 run "ntnlab_nr --freezeGeometry=0 --duration=100s"   the satellite flies: 30 -> 72 deg
 *   ./ns3 run "ntnlab_nr --freezeGeometry=0 --elevation=10 --duration=60s"   it rises from 10 deg
 *   ./ns3 run "ntnlab_nr --fullPass=1 --nUes=2"        the whole pass over UE 0, 10 deg to 10 deg
 *   ./ns3 run "ntnlab_nr --fullPass=1 --elevation=30"  a shorter pass: 30 deg, over, 30 deg
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
 *   ./ns3 run "ntnlab_nr --ueBufferAgeS=3"             UEs drop voice that waited over 3 s
 *   ./ns3 run "ntnlab_nr --fastFading=model --fadingHoldMs=100"   a fading that holds 100 ms
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
 *   ./ns3 run "ntnlab_nr --retry=1 --retrySeconds=3 --maxRetry=5"   a released UE calls again
 *   ./ns3 run "ntnlab_nr --dlBoostMaxDb=6 --dlClosedLoop=1 --dlSnrMinDb=4"
 *                                                      boost (up to 6 dB) only while a UE's SNR
 *                                                      is under 4 dB
 *   ./ns3 run "ntnlab_nr --dlBoostMaxDb=6 --dlClosedLoop=2 --dlMcsMin=5"   ... while its MCS is under 5
 *   ./ns3 run "ntnlab_nr --dlBoostMaxDb=6 --dlClosedLoop=3 --dlSnrMinDb=4 --dlMcsMin=5"   both
 *   ./ns3 run "ntnlab_nr --ulClosedLoop=1 --ulSnrHighDb=3 --ulSnrLowDb=2"
 *                                                      a UE lowers its power while its uplink SNR
 *                                                      is over 3 dB, never to under 2 dB
 *   ./ns3 run "ntnlab_nr --dlPfdLimit=-90 --dlClosedLoop=1 --ulClosedLoop=1 --closedLoopMs=500"
 *                                                      both loops, a decision every 500 ms
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
#include "ns3/propagation-loss-model.h"
#include "ns3/spectrum-transmit-filter.h"
#include "ns3/three-gpp-antenna-model.h"
#include "ns3/three-gpp-channel-model.h"
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
#include <memory>
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
uint32_t g_puschMinBlockBytes{0}; // --puschMinBlockBytes
// The least uplink block of a slot that the repetitions have to give: what --puschMinBlockBytes
// says, never less than what a UE sends data in (NrMacShortBsrCe::MinUlGrantBytes(): 15 B, 12 B
// with --ns3::NrUeMac::CompactBsr=1), as in NrMacSchedulerUeInfo::UlLinkFor().
uint32_t
MinUlBlockBytes()
{
    return std::max<uint32_t>(g_puschMinBlockBytes, NrMacShortBsrCe::MinUlGrantBytes());
}
std::vector<SnrStats> g_ulRxSnr; // the same as they came
std::vector<SnrStats> g_dlRbPower; // per UE: the power of an RB of its downlink allocations, dBm
std::vector<SnrStats> g_dlDataSnr; // per UE: the SNR of its downlink data, on the RBs scheduled
std::ofstream g_sinrTraceDl; // --sinrTrace: time, UE, SINR (dB) of every control symbol received
std::ofstream g_sinrTraceUl; // --sinrTrace: time, UE, SINR (dB) of every PUSCH, its RBs

// per UE: its PUSCH slots by the RBs they have (--ulWideRbs): the narrow grant's, the wide
// one's, others
struct UlGrantStats
{
    uint64_t narrow{0};
    uint64_t wide{0};
    uint64_t other{0};
    uint64_t rbSlots{0};
};

std::vector<UlGrantStats> g_ulGrants;
uint32_t g_ulNarrowRbs{0};
uint32_t g_ulWideRbs{0};
std::vector<std::pair<Time, Time>> g_snrWindow; // per UE: its call
// The UE of a connection, by (cell id << 16 | RNTI): an RNTI is only unique within its cell
std::map<uint32_t, uint32_t> g_rntiUe;
uint32_t
RntiKey(uint32_t cellId, uint16_t rnti)
{
    return (cellId << 16) | rnti;
}
double g_ulSnrRbs{2.0};

// The same by the elevation of the satellite above the UE, in steps of 10 deg (SNR tables). A
// step is a number that sorts as the pass goes: 10 ... 80 while the satellite comes closer to the
// UE, 90 over it, 100 ... 170 (180 - the elevation) as it moves away. Without a flying satellite
// there is no before and after: the elevation, to the nearest 10 deg.
struct SnrStep
{
    SnrStats dl;
    SnrStats ul;
    SnrStats ulRx;
    SnrStats power;  // the power of an RB of the downlink allocations, dBm (--dlPfdLimit ...)
    SnrStats dlData; // the SNR of the UE's downlink data, on the RBs scheduled for it
};

std::vector<std::map<int, SnrStep>> g_snrSteps; // per UE
std::vector<int> g_snrStep;                     // per UE: its step now (0: not known yet)

int
ElevationStep(double elevationDeg, double side)
{
    const int deg = std::clamp(static_cast<int>(std::lround(elevationDeg / 10.0)) * 10, 10, 90);
    return side > 0.0 && deg < 90 ? 180 - deg : deg;
}

// How a step is written: -30 while the satellite comes closer (flying), 30 once it moves away
std::string
StepLabel(int step, bool flying)
{
    if (step > 90)
    {
        return std::to_string(180 - step);
    }
    return (flying && step < 90 ? "-" : "") + std::to_string(step);
}

bool
InSnrWindow(uint32_t ue)
{
    const Time now = Simulator::Now();
    return now >= g_snrWindow[ue].first && now < g_snrWindow[ue].second;
}

// --- Closed loop on the power: --dlClosedLoop, --ulClosedLoop ----------------------------------
// The state of the two loops for each UE, what they did (for the report), and the measurements
// they go by. ClosedLoopTick() decides, once per --closedLoopMs.
//
// Downlink: the UE measures the SNR of its data (the figure that goes into its CQI) and the loop
// sets a cap on how far over its share the satellite sends the RBs of that UE
// (NrGnbPhy::SetUeMaxRbBoostDb; --dlBoostMaxDb and the PFD limit still bind). The cap is the
// whole boost at first, then goes to what the UE needs for its target (--dlSnrMinDb, or the SNR at
// which the scheduler gives --dlMcsMin): up when it is short, down when it has a step to spare.
// Uplink: the gNB measures the SNR of the UE's PUSCH transmissions; the loop lowers the UE's
// power while that SNR is above --ulSnrHighDb, never so far that it is under --ulSnrLowDb, and
// raises it again if it falls under --ulSnrLowDb: the accumulated TPC command of TS 38.213 7.1.1.
struct DlLoopUe
{
    double capDb{0.0};   // the cap on the boost of the UE's RBs now (dB over their share)
    uint16_t rnti{0};    // the RNTI the cap is set for (0: the UE has no connection)
    bool settled{false}; // the first decision of the connection has been made
    double needBoostDb{0.0}; // the boost the UE needs for its target, from its SNR at the share
                             // (filtered over the periods)
    // since the last decision
    uint32_t n{0};         // receptions of its data
    double snrSumDb{0.0};  // their SNR
    double needSumDb{0.0}; // the SNR its target needs on the RBs of each
    uint32_t usedN{0};     // allocations the satellite sent
    double usedSumDb{0.0}; // how far over their share the RBs of these were
};

struct UlLoopUe
{
    double offsetDb{0.0}; // f of TS 38.213 7.1.1: what the loop took off the power (0 or less)
    bool settled{false};  // the first decision of the connection has been made
    double snrFullDb{0.0}; // the UE's SNR at the power it has without the loop (filtered)
    // since the last decision
    uint32_t n{0};        // PUSCH transmissions
    double snrSumDb{0.0}; // their SNR
    double offSumDb{0.0}; // the loop's offset at each (f; the power less the one without the loop)
};

// What the loops did while the UE's call was on
struct LoopTally
{
    // downlink
    uint64_t dlRx{0};       // receptions of its data
    uint64_t dlSnrBelow{0}; // ... under --dlSnrMinDb
    uint64_t dlMcsBelow{0}; // ... at an MCS under --dlMcsMin
    SnrStats dlSnr;         // their SNR, dB
    SnrStats dlMcs;         // their MCS
    SnrStats dlBoost;       // how far over their share the RBs of an allocation were sent, dB
    SnrStats dlBoostFree;   // ... and how far they would have been without the loop
    SnrStats dlCap;         // the cap, after each decision, dB
    double dlMwUsed{0.0};   // what was sent: the RBs of each allocation x their mW
    double dlMwFree{0.0};   // ... and what it would have been without the loop
    uint32_t dlWin{0};      // decisions (periods with enough receptions to decide on)
    uint32_t dlWinBelow{0}; // ... of which the period's mean SNR was under what the UE needs
    uint32_t dlUp{0};       // decisions that raised the cap, lowered it, kept it
    uint32_t dlDown{0};
    uint32_t dlHold{0};
    uint32_t dlAtMax{0};    // ... in which the boost the UE needs was over the top: out of reach
    // uplink
    uint64_t ulRx{0};      // PUSCH transmissions
    uint64_t ulBelow{0};   // ... with an SNR under --ulSnrLowDb
    uint64_t ulInBand{0};  // ... from there to --ulSnrHighDb
    uint64_t ulAbove{0};   // ... above it
    SnrStats ulSnr;        // their SNR, as if the UE sent on --ulGrantRbs RBs, dB
    SnrStats ulPower;      // the UE's power at each, dBm
    double ulMw{0.0};      // the sum of the power of each, mW
    double ulMwFree{0.0};  // ... and of what it would have sent without the loop
    uint32_t ulWin{0};     // decisions (periods with enough PUSCH transmissions to decide on)
    uint32_t ulWinInBand{0}; // ... of which the period's mean SNR was in the band
    uint32_t ulUp{0};      // decisions that raised the power, lowered it, kept it
    uint32_t ulDown{0};
    uint32_t ulHold{0};
    uint32_t ulAtMin{0};   // ... above the band with the power at its lowest
};

struct ClosedLoop
{
    uint32_t dl{0};      // --dlClosedLoop: 0 off, 1 SNR, 2 MCS, 3 both
    bool ul{false};      // --ulClosedLoop
    double dlSnrMinDb{4.0};
    uint32_t dlMcsMin{10};
    double dlMaxDb{0.0}; // the highest cap: --dlBoostMaxDb, or 30 dB if only the PFD limit binds
    double ulHighDb{3.0};
    double ulLowDb{2.0};
    double ulMinDbm{-40.0};
    double stepDb{1.0};
    double landing{0.2};                          // --closedLoopLanding, see below
    double filter{1.0 - std::exp(-1.0 / 3.0)};    // --closedLoopAvg: the estimate's share of a new period
    Time period{Seconds(1)};
    // Fewer measurements than this: no decision, the period goes on (the next one adds to it). The
    // SNR of a reception scatters with the fading by 3 dB or so, and the receptions of one HARQ
    // process or of the repetitions of one block share it: a decision on a few of them follows a
    // fade of the moment, not the UE
    uint32_t dlMinSamples{20};
    uint32_t ulMinSamples{200};
    Ptr<NrAmc> amc;         // the scheduler's: CQI -> MCS
    Ptr<const SpectrumModel> model;
    std::map<std::pair<uint32_t, uint32_t>, double> mcsSnr; // (MCS, RBs) -> dB, see DlSnrForMcs()
    std::function<bool(uint32_t)> connected; // UE i's RRC is connected
};

ClosedLoop g_loop;
std::vector<DlLoopUe> g_dlLoop;
std::vector<UlLoopUe> g_ulLoop;
std::vector<LoopTally> g_loopTally;

// (defined with the scene, below)
double DlShareDbm(uint32_t ue);
double UlTxPowerDbm(uint32_t ue);
double UlBasePowerDbm(uint32_t ue);
void ApplyUlPower(uint32_t ue);
void LoopOnConnected(uint32_t ue, uint16_t rnti);
void LoopReset(uint32_t ue);

// The lowest SNR, in dB, at which the scheduler gives a block on `rbs` RBs an MCS of `mcs` or
// more: the UE turns the SNR of its RBs into a CQI (the highest MCS whose block error rate is
// under 10 %, as a CQI), and the scheduler takes the MCS of that CQI (NrAmc::GetMcsFromCqi).
// The same AMC, error model and block sizes, asked for a flat SNR over the RBs, found by halving.
// 50 if even that does not give it.
double
DlSnrForMcs(uint32_t mcs, uint32_t rbs)
{
    const auto key = std::make_pair(mcs, rbs);
    const auto cached = g_loop.mcsSnr.find(key);
    if (cached != g_loop.mcsSnr.end())
    {
        return cached->second;
    }
    auto reached = [&](double db) {
        SpectrumValue sinr(g_loop.model);
        const double linear = std::pow(10.0, db / 10.0);
        for (uint32_t k = 0; k < rbs; ++k)
        {
            sinr[k] = linear;
        }
        uint8_t unused = 0;
        const uint8_t cqi = g_loop.amc->CreateCqiFeedbackSiso(sinr, unused);
        return g_loop.amc->GetMcsFromCqi(cqi) >= mcs;
    };
    double lo = -30.0;
    double hi = 50.0;
    double result = hi;
    if (!reached(hi))
    {
        result = hi;
    }
    else if (reached(lo))
    {
        result = lo;
    }
    else
    {
        while (hi - lo > 0.05)
        {
            const double mid = 0.5 * (lo + hi);
            (reached(mid) ? hi : lo) = mid;
        }
        result = hi;
    }
    g_loop.mcsSnr[key] = result;
    return result;
}

// A downlink reception of UE i (RxPacketTraceUe, a PDSCH or its retransmission): the SNR of the
// UE's RBs and the MCS of the block, for the loop and for the report
void
LoopDlSample(uint32_t ue, const RxPacketTraceParams& p)
{
    const double snrDb = 10.0 * std::log10(p.m_sinr);
    DlLoopUe& s = g_dlLoop[ue];
    ++s.n;
    s.snrSumDb += snrDb;
    if ((g_loop.dl & 2) != 0)
    {
        s.needSumDb += DlSnrForMcs(g_loop.dlMcsMin, std::max<uint32_t>(p.m_rbAssignedNum, 1));
    }
    if (InSnrWindow(ue))
    {
        LoopTally& t = g_loopTally[ue];
        ++t.dlRx;
        t.dlSnr.Add(snrDb);
        t.dlMcs.Add(p.m_mcs);
        t.dlSnrBelow += snrDb < g_loop.dlSnrMinDb;
        t.dlMcsBelow += p.m_mcs < g_loop.dlMcsMin;
    }
}

// The satellite sent an allocation of UE i ("DlDataPower"): how far over their share its RBs were
void
LoopDlPower(uint32_t ue, uint32_t rbs, double powerDbm)
{
    const double boostDb = powerDbm - DlShareDbm(ue);
    DlLoopUe& s = g_dlLoop[ue];
    ++s.usedN;
    s.usedSumDb += boostDb;
    if (InSnrWindow(ue))
    {
        LoopTally& t = g_loopTally[ue];
        t.dlBoost.Add(boostDb);
        t.dlMwUsed += rbs * std::pow(10.0, powerDbm / 10.0);
    }
}

// ... and what it would have sent without the cap of the UE ("DlDataPowerFree")
void
DlRbPowerFree(uint32_t cellId, uint16_t rnti, uint32_t rbs, double powerDbm)
{
    const auto it = g_rntiUe.find(RntiKey(cellId, rnti));
    if (it == g_rntiUe.end() || !std::isfinite(powerDbm) || !InSnrWindow(it->second))
    {
        return;
    }
    LoopTally& t = g_loopTally[it->second];
    t.dlMwFree += rbs * std::pow(10.0, powerDbm / 10.0);
    t.dlBoostFree.Add(powerDbm - DlShareDbm(it->second));
}

// A PUSCH transmission of UE i, as the gNB received it: its SNR as if the UE had sent on
// --ulGrantRbs RBs (the figure of the uplink SNR tables for a narrow grant, whatever the grant)
void
LoopUlSample(uint32_t ue, double scaledDb)
{
    UlLoopUe& s = g_ulLoop[ue];
    ++s.n;
    s.snrSumDb += scaledDb;
    s.offSumDb += UlTxPowerDbm(ue) - UlBasePowerDbm(ue);
    if (InSnrWindow(ue))
    {
        LoopTally& t = g_loopTally[ue];
        ++t.ulRx;
        t.ulSnr.Add(scaledDb);
        const double dbm = UlTxPowerDbm(ue);
        t.ulPower.Add(dbm);
        t.ulMw += std::pow(10.0, dbm / 10.0);
        t.ulMwFree += std::pow(10.0, UlBasePowerDbm(ue) / 10.0);
        t.ulBelow += scaledDb < g_loop.ulLowDb;
        t.ulAbove += scaledDb > g_loop.ulHighDb;
        t.ulInBand += scaledDb >= g_loop.ulLowDb && scaledDb <= g_loop.ulHighDb;
    }
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
        const double db = 10.0 * std::log10(sinr);
        g_dlSnr[ue].Add(db);
        if (g_sinrTraceDl.is_open())
        {
            g_sinrTraceDl << Simulator::Now().GetSeconds() << ',' << ue << ',' << db << '\n';
        }
        if (g_snrStep[ue] > 0)
        {
            g_snrSteps[ue][g_snrStep[ue]].dl.Add(db);
        }
    }
}

// "RxPacketTraceGnb" of the satellite gNB: every PUSCH transmission it receives (a repetition is
// one), with the SINR over its resource blocks. One the UE did not send (it missed the DCI)
// has no signal and is left out.
void
UlSinr(RxPacketTraceParams p)
{
    const auto it = g_rntiUe.find(RntiKey(p.m_cellId, p.m_rnti));
    if (it == g_rntiUe.end() || p.m_pdcchMissed || p.m_sinr <= 0.0 || p.m_rbAssignedNum == 0)
    {
        return;
    }
    // The UE's power is spread over the RBs of the grant: on g_ulSnrRbs it has that many times less
    const double db = 10.0 * std::log10(p.m_sinr);
    const double scaledDb = db + 10.0 * std::log10(p.m_rbAssignedNum / g_ulSnrRbs);
    if (g_loop.ul)
    {
        LoopUlSample(it->second, scaledDb);
    }
    if (!InSnrWindow(it->second))
    {
        return;
    }
    g_ulRxSnr[it->second].Add(db);
    g_ulSnr[it->second].Add(scaledDb);
    if (g_sinrTraceUl.is_open())
    {
        g_sinrTraceUl << Simulator::Now().GetSeconds() << ',' << it->second << ',' << db << ','
                      << static_cast<int>(p.m_rbAssignedNum) << ',' << static_cast<int>(p.m_mcs)
                      << ',' << p.m_tbSize << ',' << static_cast<int>(p.m_rv) << ','
                      << (p.m_corrupt ? 1 : 0) << ',' << p.m_tbler << '\n';
    }
    UlGrantStats& grants = g_ulGrants[it->second];
    ++(p.m_rbAssignedNum == g_ulNarrowRbs ? grants.narrow
       : p.m_rbAssignedNum == g_ulWideRbs ? grants.wide
                                          : grants.other);
    grants.rbSlots += p.m_rbAssignedNum;
    if (g_snrStep[it->second] > 0)
    {
        SnrStep& step = g_snrSteps[it->second][g_snrStep[it->second]];
        step.ulRx.Add(db);
        step.ul.Add(scaledDb);
    }
}

// "RxPacketTraceUe" of UE i's downlink spectrum PHY: every downlink data reception (a PDSCH, a
// retransmission is one) with its SINR over the RBs scheduled. i is bound when connecting.
void
DlDataSinr(uint32_t ue, RxPacketTraceParams p)
{
    if (g_loop.dl != 0 && p.m_sinr > 0.0 && !p.m_pdcchMissed)
    {
        LoopDlSample(ue, p);
    }
    if (p.m_sinr > 0.0 && InSnrWindow(ue))
    {
        const double db = 10.0 * std::log10(p.m_sinr);
        g_dlDataSnr[ue].Add(db);
        if (g_snrStep[ue] > 0)
        {
            g_snrSteps[ue][g_snrStep[ue]].dlData.Add(db);
        }
    }
}

// "DlDataPower" of the satellite gNB: the power of each RB of a downlink allocation, in dBm
void
DlRbPower(uint32_t cellId, uint16_t rnti, uint32_t rbs, double powerDbm)
{
    const auto it = g_rntiUe.find(RntiKey(cellId, rnti));
    if (it == g_rntiUe.end() || !std::isfinite(powerDbm))
    {
        return;
    }
    if (g_loop.dl != 0)
    {
        LoopDlPower(it->second, rbs, powerDbm);
    }
    if (!InSnrWindow(it->second))
    {
        return;
    }
    g_dlRbPower[it->second].Add(powerDbm);
    if (g_snrStep[it->second] > 0)
    {
        g_snrSteps[it->second][g_snrStep[it->second]].power.Add(powerDbm);
    }
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
    uint16_t cellId{0};
    std::array<NrPdcp::PacketCounts, 2> before; // DOWN, UP
};

std::vector<std::vector<Connection>> g_connections;
std::vector<Ipv4Address> g_ueAddress; // of UE i, for NrPdcp::GetPacketCounts()

void
RadioLinkFailure(uint32_t ue, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    ++g_rlf[ue];
    g_outages[ue].emplace_back(Simulator::Now(), Time::Max());
    if (g_loop.dl != 0 || g_loop.ul)
    {
        LoopReset(ue); // the UE attaches anew, with all its power (random access)
    }
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
                                 cellId,
                                 {NrPdcp::GetPacketCounts(g_ueAddress[ue], true),
                                  NrPdcp::GetPacketCounts(g_ueAddress[ue], false)}});
    g_rntiUe[RntiKey(cellId, rnti)] = ue;
    if (g_loop.dl != 0 || g_loop.ul)
    {
        LoopOnConnected(ue, rnti);
    }
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

std::map<uint32_t, std::array<UeDciCount, 2>> g_dciByRnti; // (cell id, RNTI): DL DCIs, UL DCIs
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
    UeDciCount& c = g_dciByRnti[RntiKey(cellId, rnti)][msg->GetMessageType() == NrControlMessage::DL_DCI ? 0 : 1];
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

// The BSR in the uplink blocks, as the settings say it
std::string
BsrText()
{
    return NrMacShortBsrCe::IsCompact() ? std::string("2 B BSR (TS 38.321 short BSR)")
                                        : std::string("5 B BSR (nr module; 3GPP: 2 B)");
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
BeamTaperDb(const Vector& satEcef,
            const Vector& centreEcef,
            const Vector& ueEcef,
            double ka,
            double floorDb = -30.0)
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
    return std::max(10.0 * std::log10(std::max(4.0 * f * f, 1e-12)), floorDb);
}

// --- Several beams and satellites (--beams, --beamReuse, --satellites) ---------------------------
// TR 38.821 6.1.1.1: the boresights of the beams of a satellite are the points of a hexagonal grid
// of the UV plane of the satellite (u, v: the direction cosines of a beam towards the along-track
// axis and the cross-track axis; (0, 0) is nadir), adjacent beams ABS = sqrt(3) sin(HPBW / 2)
// apart, so that the corners of the hexagon of a beam are on its 3 dB contour. The centre beam is
// the beam of the single-beam run (at the elevation of --elevation); the rings around it have 6,
// 12, 18 ... beams: 7, 19, 37, 61 beams in all.

// The beams of a hexagonal layout of `tiers` rings around the centre beam, ring by ring: the
// axial coordinates (q, r) of each and its ring
struct HexBeam
{
    int q;
    int r;
    int tier;
};

std::vector<HexBeam>
HexBeams(int tiers)
{
    std::vector<HexBeam> beams{{0, 0, 0}};
    const int sides[6][2] = {{-1, 1}, {-1, 0}, {0, -1}, {1, -1}, {1, 0}, {0, 1}};
    for (int tier = 1; tier <= tiers; ++tier)
    {
        int q = tier;
        int r = 0;
        for (const auto& side : sides)
        {
            for (int k = 0; k < tier; ++k)
            {
                beams.push_back({q, r, tier});
                q += side[0];
                r += side[1];
            }
        }
    }
    return beams;
}

// The rings of a layout with this many beams (1 + 3 k (k + 1)), -1 if no layout has that many
int
HexTiersOf(uint32_t beams)
{
    for (int k = 0; k < 20; ++k)
    {
        if (1 + 3 * k * (k + 1) == static_cast<int>(beams))
        {
            return k;
        }
    }
    return -1;
}

// The frequency colour of a beam for a reuse factor of 1, 3, 4 or 7: the 3-colouring (q - r) mod 3
// of the hexagonal grid and its relatives; beams of one colour are sqrt(reuse) beam spacings apart
int
BeamColour(int q, int r, uint32_t reuse)
{
    auto mod = [](int a, int m) { return ((a % m) + m) % m; };
    switch (reuse)
    {
    case 3:
        return mod(q - r, 3);
    case 4:
        return mod(q, 2) + 2 * mod(r, 2);
    case 7:
        return mod(q + 5 * r, 7);
    default:
        return 0;
    }
}

// Where on the ground a beam of the satellite at `sat` is pointed (ECEF): the boresight with the
// direction cosines (u, v) in the satellite's frame (u along `along`, v = up x along, nadir =
// -up), where it meets the sphere of the Earth. False if it misses the Earth.
bool
BeamGroundPoint(const Vector& sat,
                const Vector& along,
                double u,
                double v,
                double earthRadiusM,
                Vector& ground)
{
    const double s2 = u * u + v * v;
    if (s2 >= 1.0)
    {
        return false;
    }
    const double w = std::sqrt(1.0 - s2);
    const double satR = sat.GetLength();
    const Vector up(sat.x / satR, sat.y / satR, sat.z / satR);
    const Vector y(up.y * along.z - up.z * along.y,
                   up.z * along.x - up.x * along.z,
                   up.x * along.y - up.y * along.x);
    const Vector d(u * along.x + v * y.x - w * up.x,
                   u * along.y + v * y.y - w * up.y,
                   u * along.z + v * y.z - w * up.z);
    const double b = sat.x * d.x + sat.y * d.y + sat.z * d.z;
    const double disc = b * b - (satR * satR - earthRadiusM * earthRadiusM);
    if (disc < 0.0)
    {
        return false;
    }
    const double t = -b - std::sqrt(disc);
    ground = Vector(sat.x + t * d.x, sat.y + t * d.y, sat.z + t * d.z);
    return true;
}

// The along-track direction of the satellite at `sat` that points to the south: the satellites of
// this example fly south along the meridian of the beam centre
Vector
SouthOf(const Vector& sat)
{
    const double lat = std::asin(sat.z / sat.GetLength());
    const double lon = std::atan2(sat.y, sat.x);
    return Vector(std::sin(lat) * std::cos(lon), std::sin(lat) * std::sin(lon), -std::cos(lat));
}

struct BeamPlan
{
    uint32_t sat{0};
    uint32_t beam{0}; // its place in the layout of its satellite (0: the centre beam)
    int tier{0};
    int q{0};
    int r{0};
    int colour{0};
    Vector centre; // ECEF, where it points
    double u{0.0}; // the direction cosines of its boresight
    double v{0.0};
};

// The beams of every satellite (sats: where each is, in ECEF, at the moment the layout is for;
// u0: the direction cosine of the centre beam towards the along-track axis, sin of its angle from
// nadir; absUv: the spacing of the beams in the UV plane; shifts: how many beam spacings along
// the track the lattice of each satellite is from the first's, for the colours). A beam that
// misses the Earth, or whose centre is less than `minElevationDeg` above the horizon, is not
// planned: the plan is shorter than `beams` per satellite then.
std::vector<BeamPlan>
PlanBeams(const std::vector<Vector>& sats,
          int tiers,
          uint32_t reuse,
          double u0,
          double absUv,
          const std::vector<int>& shifts,
          double earthRadiusM,
          double minElevationDeg)
{
    std::vector<BeamPlan> plan;
    for (uint32_t s = 0; s < sats.size(); ++s)
    {
        const Vector along = SouthOf(sats[s]);
        uint32_t index = 0;
        for (const HexBeam& hex : HexBeams(tiers))
        {
            BeamPlan beam;
            beam.sat = s;
            beam.beam = index++;
            beam.tier = hex.tier;
            beam.q = hex.q;
            beam.r = hex.r;
            beam.u = u0 + absUv * (hex.q + 0.5 * hex.r);
            beam.v = absUv * std::sqrt(3.0) / 2.0 * hex.r;
            beam.colour = BeamColour(hex.q + shifts[s], hex.r, reuse);
            if (BeamGroundPoint(sats[s], along, beam.u, beam.v, earthRadiusM, beam.centre) &&
                (hex.tier == 0 || ElevationDeg(sats[s], beam.centre) >= minElevationDeg))
            {
                plan.push_back(beam);
            }
        }
    }
    return plan;
}

// One beam, for the interference worked out from the geometry alone
struct CiBeam
{
    Vector sat;    // where its satellite is
    Vector centre; // where it points
    int colour{0}; // its carrier
};

// The carrier-to-interference ratio (dB) of a UE in the cell `serving` at full load: every other
// beam of the same colour sends on every RB at the power of the beam it is served by, so that what
// the UE hears of it is the pattern of that beam towards the UE (BeamTaperDb()) less the
// free-space loss of the path from its satellite. No noise, no fading, no load. NaN if the colour
// has no other beam.
double
FullLoadCiDb(const std::vector<CiBeam>& beams,
             std::size_t serving,
             const Vector& ue,
             double ka,
             double floorDb)
{
    auto rxDb = [&](const CiBeam& b) {
        return BeamTaperDb(b.sat, b.centre, ue, ka, floorDb) -
               20.0 * std::log10((b.sat - ue).GetLength());
    };
    double interferenceMw = 0.0;
    for (std::size_t d = 0; d < beams.size(); ++d)
    {
        if (d != serving && beams[d].colour == beams[serving].colour)
        {
            interferenceMw += std::pow(10.0, rxDb(beams[d]) / 10.0);
        }
    }
    return interferenceMw > 0.0 ? rxDb(beams[serving]) - 10.0 * std::log10(interferenceMw) : NAN;
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

// BEGIN NTN-TDL
// --fastFading=ntn-tdl-a, -b, -c, -d: the tapped delay lines of TR 38.811 6.9.2 (Tables 6.9.2-1
// to 6.9.2-4): A and B for a link without line of sight (Rayleigh taps), C and D with it (a
// Ricean first tap, K = 10.224 dB in C, 11.707 dB in D). The same profile at every elevation, the
// approach 3GPP agreed for NTN simulations: the elevation is in the link budget (path loss, gases,
// antenna gain), not in the fading, and no K-factor or LOS probability depends on it. The
// normalised delays are
// scaled to the delay spread (TR 38.901 7.7.3) and the powers to a total of 1 (what the table's
// "mean power of 0 dB" for its first tap comes to; C is 1.005 before, D 1.12), so that the fading
// takes nothing from the link budget on average.
//   Doppler. TR 38.811 6.9.2 puts the Doppler shift of the satellite on all taps alike, one shift
//   of the whole link, which the UE takes off (it knows the satellite's ephemeris and its own
//   position, as the header of the run says); what fades is the Doppler of the UE's own motion,
//   TR 38.901 7.7.2: the Rayleigh taps have the Jakes spectrum with f_D = v f_c / c (5.6 Hz for
//   3 km/h at 2 GHz: a coherence time of 0.423 / f_D = 76 ms), and the Ricean tap has, besides
//   its Jakes part, a peak at the shift f_S = 0.7 f_D (TR 38.901 7.7.2: "the Doppler spectrum
//   additionally contains a peak at the Doppler shift f_S = 0.7 f_D"; TR 38.811 6.9.2 takes the
//   Doppler spectrum of each tap from there). The beat of that peak with the Rayleigh part makes
//   the dB fading of NTN-TDL-C decorrelate faster than the Rayleigh taps do: its autocorrelation
//   is J0(2 pi f_D t) cos(2 pi 0.7 f_D t) and falls to 0.5 at about 0.18 / f_D (30 ms at
//   3 km/h and 2 GHz; measured in the simulator: 0.72 at 20 ms, 0.23 at 40 ms). A Rayleigh tap is the sum
//   of 48 sinusoids: arrival angles evenly spaced round the circle from a random start, random
//   phases. The fading of a link is a function of the time, so it changes slowly from slot to
//   slot, as it does in a real channel, and a repetition 1 ms later sees the fading of the first.
class NtnTdlProcess
{
  public:
    struct Tap
    {
        double delay;   // normalised
        double powerDb; // before the normalisation
        bool los;       // the line-of-sight ray of a Ricean tap (the Rayleigh part is the next row)
    };

    // The rows of Tables 6.9.2-1 to 6.9.2-4 (the Ricean tap of C and D is two rows: its line of
    // sight and its scattered part, at the same delay)
    static const std::vector<Tap>& Taps(char profile)
    {
        static const std::vector<Tap> a = {{0.0, 0.0, false},
                                           {1.0811, -4.675, false},
                                           {2.8416, -6.482, false}};
        static const std::vector<Tap> b = {{0.0, 0.0, false},
                                           {0.7249, -1.973, false},
                                           {0.7410, -4.332, false},
                                           {5.7392, -11.914, false}};
        static const std::vector<Tap> c = {{0.0, -0.394, true},
                                           {0.0, -10.618, false},
                                           {14.8124, -23.373, false}};
        static const std::vector<Tap> d = {{0.0, -0.284, true},
                                           {0.0, -11.991, false},
                                           {0.5596, -9.887, false},
                                           {7.3340, -16.771, false}};
        switch (profile)
        {
        case 'a':
            return a;
        case 'b':
            return b;
        case 'c':
            return c;
        default:
            return d;
        }
    }

    // draw: a uniform number on [0, 1), for the random start of every tap
    NtnTdlProcess(char profile,
                  double delaySpreadS,
                  double dopplerHz,
                  const std::function<double()>& draw)
    {
        const std::vector<Tap>& taps = Taps(profile);
        double total = 0.0;
        for (const Tap& tap : taps)
        {
            total += std::pow(10.0, tap.powerDb / 10.0);
        }
        for (const Tap& tap : taps)
        {
            // the cluster of this delay (the rows of one Ricean tap share it)
            std::size_t cluster = m_delayS.size();
            for (std::size_t i = 0; i < m_delayS.size(); ++i)
            {
                if (m_delayS[i] == tap.delay * delaySpreadS)
                {
                    cluster = i;
                }
            }
            if (cluster == m_delayS.size())
            {
                m_delayS.push_back(tap.delay * delaySpreadS);
            }
            Component component;
            component.cluster = cluster;
            component.amplitude = std::sqrt(std::pow(10.0, tap.powerDb / 10.0) / total);
            if (tap.los)
            {
                component.omega = {2.0 * M_PI * 0.7 * dopplerHz};
                component.phase = {2.0 * M_PI * draw()};
            }
            else
            {
                const double start = draw();
                for (uint32_t m = 0; m < kSinusoids; ++m)
                {
                    const double arrival = 2.0 * M_PI * (m + start) / kSinusoids;
                    component.omega.push_back(2.0 * M_PI * dopplerHz * std::cos(arrival));
                    component.phase.push_back(2.0 * M_PI * draw());
                }
                component.amplitude /= std::sqrt(static_cast<double>(kSinusoids));
            }
            m_components.push_back(std::move(component));
        }
    }

    std::size_t Clusters() const
    {
        return m_delayS.size();
    }

    // the delay of cluster c (s)
    double DelayS(std::size_t c) const
    {
        return m_delayS[c];
    }

    // the complex gain of every cluster at the time t (s), to g[0 .. Clusters() - 1]
    void Gains(double t, std::complex<double>* g) const
    {
        for (std::size_t c = 0; c < m_delayS.size(); ++c)
        {
            g[c] = 0.0;
        }
        for (const Component& component : m_components)
        {
            std::complex<double> sum(0.0, 0.0);
            for (std::size_t i = 0; i < component.omega.size(); ++i)
            {
                sum += std::polar(1.0, component.omega[i] * t + component.phase[i]);
            }
            g[component.cluster] += component.amplitude * sum;
        }
    }

  private:
    static constexpr uint32_t kSinusoids = 48;

    struct Component
    {
        std::size_t cluster{0};
        double amplitude{0.0}; // of one sinusoid
        std::vector<double> omega; // rad/s
        std::vector<double> phase; // rad
    };

    std::vector<double> m_delayS;
    std::vector<Component> m_components;
};
// END NTN-TDL

// --fastFading=fixed: the channel between satellite and UE is the direct path alone, the
// line-of-sight ray of TR 38.901 (7.5-29) with an infinite K-factor: no scattered clusters, so
// nothing that changes from one resource block or one slot to the next. With
// --fastFading=ntn-tdl-a to -d the same ray carries the taps of an NtnTdlProcess: every tap has
// the ray's gains and phases (beams, patterns, polarisation, the carrier phase of the distance),
// times its own complex gain, which fades; the delays of the taps give the resource blocks their
// frequency selectivity. It keeps what ns-3's
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

    // --fastFading=ntn-tdl-*: the profile ('a' to 'd'), its delay spread (s), the Doppler of the
    // UE's motion (Hz, the largest), and the first of the random streams of the links' taps
    void SetTdl(char profile, double delaySpreadS, double dopplerHz, int64_t firstStream)
    {
        m_tdlProfile = profile;
        m_tdlDelaySpreadS = delaySpreadS;
        m_tdlDopplerHz = dopplerHz;
        m_tdlFirstStream = firstStream;
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

        // The taps of an NTN-TDL profile, drawn once for the link
        if (m_tdlProfile != 0 && !link.tdl)
        {
            Ptr<UniformRandomVariable> random = CreateObject<UniformRandomVariable>();
            random->SetStream(m_tdlFirstStream + static_cast<int64_t>(sId) * 65536 + uId);
            link.tdl = std::make_shared<NtnTdlProcess>(m_tdlProfile,
                                                       m_tdlDelaySpreadS,
                                                       m_tdlDopplerHz,
                                                       [random]() {
                                                           return random->GetValue(0.0, 1.0);
                                                       });
        }
        const std::size_t clusters = link.tdl ? link.tdl->Clusters() : 1;
        std::vector<std::complex<double>> tapGain(clusters, std::complex<double>(1.0, 0.0));
        if (link.tdl)
        {
            link.tdl->Gains(Simulator::Now().GetSeconds(), tapGain.data());
        }

        // The parameters: the clusters (one, or a tap of the profile each) at the direct path,
        // with the delays of the taps, no moving scatterers. (The satellite's Doppler, which the
        // channel computes from the angles and the velocities, is one phase for all of them.)
        Ptr<ChannelParams> params = Create<ChannelParams>();
        params->m_generatedTime = Simulator::Now();
        params->m_nodeIds = {sId, uId};
        params->m_delay.assign(clusters, 0.0);
        for (std::size_t c = 0; link.tdl && c < clusters; ++c)
        {
            params->m_delay[c] = link.tdl->DelayS(c);
        }
        params->m_angle = {DoubleVector(clusters, RadiansToDegrees(uAngle.GetAzimuth())),
                           DoubleVector(clusters, RadiansToDegrees(uAngle.GetInclination())),
                           DoubleVector(clusters, RadiansToDegrees(sAngle.GetAzimuth())),
                           DoubleVector(clusters, RadiansToDegrees(sAngle.GetInclination()))};
        params->m_cachedAngleSincos.resize(params->m_angle.size());
        for (std::size_t i = 0; i < params->m_angle.size(); ++i)
        {
            const double rad = DegreesToRadians(params->m_angle[i][0]);
            params->m_cachedAngleSincos[i].assign(clusters,
                                                  std::make_pair(std::sin(rad), std::cos(rad)));
        }
        params->m_alpha.assign(clusters, 0.0);
        params->m_D.assign(clusters, 0.0);

        // The ray between every pair of elements, as ThreeGppChannelModel::GetNewChannel()
        // computes the line-of-sight ray
        Ptr<ChannelMatrix> matrix = Create<ChannelMatrix>();
        matrix->m_generatedTime = Simulator::Now();
        matrix->m_nodeIds = {sId, uId};
        matrix->m_antennaPair = {sAntenna->GetId(), uAntenna->GetId()};
        const std::size_t uSize = uAntenna->GetNumElems();
        const std::size_t sSize = sAntenna->GetNumElems();
        matrix->m_channel = Complex3DVector(uSize, sSize, clusters);
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
                const std::complex<double> ray = m_amplitude *
                                                 (rxTheta * txTheta - rxPhi * txPhi) *
                                                 distancePhase *
                                                 std::polar(1.0, rxPhase + txPhase);
                for (std::size_t c = 0; c < clusters; ++c)
                {
                    matrix->m_channel(u, s, c) = ray * tapGain[c];
                }
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
        std::shared_ptr<NtnTdlProcess> tdl; // --fastFading=ntn-tdl-*
    };

    double m_frequencyHz{2e9};
    double m_amplitude{1.0};
    char m_tdlProfile{0};
    double m_tdlDelaySpreadS{10e-9};
    double m_tdlDopplerHz{0.0};
    int64_t m_tdlFirstStream{0};
    std::map<uint64_t, Link> m_links;
};

// PhasedArrayModel::InvalidateChannels() (protected) through a class derived from it, which is
// how HeldChannelModel tells ns-3's channel that the antennas' channels are to be made again
struct AntennaAccess : public PhasedArrayModel
{
    static void Invalidate(const PhasedArrayModel& antenna)
    {
        (antenna.*(&AntennaAccess::InvalidateChannels))();
    }
};

// --fadingHoldMs=T (--fastFading=model): the fading of a link stays what it is for T ms, then a
// new one is drawn. ns-3's 3GPP channel does not do that for a link to a satellite: it throws its
// realisation away whenever an end has moved 1 m (every 1 to 2 ms when the satellite flies), and,
// with the satellite standing still, the Doppler of its reported velocity turns the clusters
// against each other within a few ms. TR 38.811 6.9.2 has the Doppler of the satellite as one
// shift common to the whole link, which the receiver takes off: it does not fade. What fades is
// the Doppler of the UE's own motion, with a coherence time of 0.423 / f_D (76 ms at 3 km/h and
// 2 GHz) and no more than 100 ms in the HAPS case. This model wraps ns-3's channel so that:
//  - the realisation of every link is drawn once per period: a fresh ThreeGppChannelModel with the
//    attributes of the first one and random streams of its own, each period (block fading);
//  - within the period the matrix is the one drawn, for every call;
//  - the satellite's end of every cluster has the angle of the strongest cluster, so that the
//    satellite's Doppler is the same phase for all clusters, and does not fade. (The UE's end is
//    as ns-3 has it: a UE that stands still does not fade within the period either.)
class HeldChannelModel : public MatrixBasedChannelModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabHeldChannelModel")
                                .SetParent<Object>()
                                .SetGroupName("Spectrum")
                                .AddConstructor<HeldChannelModel>()
                                .AddAttribute("Frequency",
                                              "The carrier (Hz)",
                                              DoubleValue(2e9),
                                              MakeDoubleAccessor(&HeldChannelModel::m_frequencyHz),
                                              MakeDoubleChecker<double>(1e8, 100e9));
        return tid;
    }

    // model: the channel that is copied; hold: the period; satelliteIds: the nodes of the
    // satellites (of the beams of them); firstStream: the random stream of the first period (four
    // per period)
    void Set(Ptr<ThreeGppChannelModel> model,
             Time hold,
             const std::set<uint32_t>& satelliteIds,
             int64_t firstStream)
    {
        m_model = model;
        m_hold = hold;
        m_satelliteIds = satelliteIds;
        m_firstStream = firstStream;
    }

    Ptr<const ChannelMatrix> GetChannel(Ptr<const MobilityModel> aMob,
                                        Ptr<const MobilityModel> bMob,
                                        Ptr<const PhasedArrayModel> aAntenna,
                                        Ptr<const PhasedArrayModel> bAntenna) override
    {
        const int64_t period = Simulator::Now().GetNanoSeconds() / m_hold.GetNanoSeconds();
        if (!m_current || period != m_period)
        {
            m_period = period;
            m_current = NewModel(period);
            m_matrices.clear();
            m_params.clear();
        }
        const uint64_t antennaKey = GetKey(aAntenna->GetId(), bAntenna->GetId());
        const auto held = m_matrices.find(antennaKey);
        if (held != m_matrices.end() && Fits(held->second, aAntenna, bAntenna))
        {
            return held->second;
        }
        // The model of this period has not seen this pair of antennas: it takes the call for the
        // first creation of the channel only if the antennas say that it is out of date
        AntennaAccess::Invalidate(*aAntenna);
        AntennaAccess::Invalidate(*bAntenna);
        Ptr<const ChannelMatrix> matrix = m_current->GetChannel(aMob, bMob, aAntenna, bAntenna);
        const uint64_t nodeKey =
            GetKey(aMob->GetObject<Node>()->GetId(), bMob->GetObject<Node>()->GetId());
        if (!m_params.contains(nodeKey))
        {
            m_params[nodeKey] = SatelliteDopplerCommon(m_current->GetParams(aMob, bMob));
        }
        m_matrices[antennaKey] = matrix;
        return matrix;
    }

    Ptr<const ChannelParams> GetParams(Ptr<const MobilityModel> aMob,
                                       Ptr<const MobilityModel> bMob) const override
    {
        const auto it = m_params.find(
            GetKey(aMob->GetObject<Node>()->GetId(), bMob->GetObject<Node>()->GetId()));
        NS_ABORT_MSG_IF(it == m_params.end(),
                        "the parameters of a link were asked for before its channel");
        return it->second;
    }

  private:
    // the ThreeGppChannelModel of one period: the attributes of the first, streams of its own
    Ptr<ThreeGppChannelModel> NewModel(int64_t period) const
    {
        ObjectFactory factory;
        factory.SetTypeId(m_model->GetInstanceTypeId());
        Ptr<ThreeGppChannelModel> model = factory.Create<ThreeGppChannelModel>();
        for (TypeId tid = m_model->GetInstanceTypeId(); tid != Object::GetTypeId();
             tid = tid.GetParent())
        {
            for (std::size_t i = 0; i < tid.GetAttributeN(); ++i)
            {
                const TypeId::AttributeInformation info = tid.GetAttribute(i);
                if ((info.flags & TypeId::ATTR_GET) && (info.flags & TypeId::ATTR_SET))
                {
                    Ptr<AttributeValue> value = info.checker->Create();
                    if (m_model->GetAttributeFailSafe(info.name, *value))
                    {
                        model->SetAttributeFailSafe(info.name, *value);
                    }
                }
            }
        }
        model->AssignStreams(m_firstStream + 4 * period);
        return model;
    }

    // a matrix of the antennas' sizes, as it was made for them (ThreeGppChannelModel's test)
    static bool Fits(Ptr<const ChannelMatrix> matrix,
                     Ptr<const PhasedArrayModel> aAntenna,
                     Ptr<const PhasedArrayModel> bAntenna)
    {
        const std::size_t a = aAntenna->GetNumElems();
        const std::size_t b = bAntenna->GetNumElems();
        const std::size_t rows = matrix->m_channel.GetNumRows();
        const std::size_t cols = matrix->m_channel.GetNumCols();
        return (rows == b && cols == a) || (rows == a && cols == b);
    }

    // a copy of the parameters in which the satellite's end of every cluster has the angle of the
    // strongest one: the Doppler of the satellite's velocity is then one phase for the whole link
    Ptr<const ChannelParams> SatelliteDopplerCommon(Ptr<const ChannelParams> original) const
    {
        auto threeGpp = DynamicCast<const ThreeGppChannelModel::ThreeGppChannelParams>(original);
        NS_ABORT_MSG_UNLESS(threeGpp, "unexpected channel parameters");
        auto copy = Create<ThreeGppChannelModel::ThreeGppChannelParams>(*threeGpp);
        // m_nodeIds.first is the departure end of the angles
        const bool satelliteDeparts = m_satelliteIds.contains(copy->m_nodeIds.first);
        const uint8_t indices[2] = {satelliteDeparts ? AOD_INDEX : AOA_INDEX,
                                    satelliteDeparts ? ZOD_INDEX : ZOA_INDEX};
        for (const uint8_t index : indices)
        {
            auto& angles = copy->m_cachedAngleSincos[index];
            NS_ABORT_IF(copy->m_cluster1st >= angles.size());
            const auto strongest = angles[copy->m_cluster1st];
            std::fill(angles.begin(), angles.end(), strongest);
        }
        return copy;
    }

    double m_frequencyHz{2e9};
    Ptr<ThreeGppChannelModel> m_model;   // the channel that is copied
    Ptr<ThreeGppChannelModel> m_current; // the channel of this period
    Time m_hold;
    std::set<uint32_t> m_satelliteIds;
    int64_t m_firstStream{0};
    int64_t m_period{-1};
    std::map<uint64_t, Ptr<const ChannelMatrix>> m_matrices; // per pair of antennas
    std::map<uint64_t, Ptr<const ChannelParams>> m_params;   // per pair of nodes
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
// --dlPfdLimit: the most power of an RB (dBm, into the antenna) that keeps the power flux density
// at `distM` along the boresight of the beam under `pfd` (dBW/m2/MHz): the EIRP density that the
// PFD allows over 4 pi d^2, with the margin (shadowing, gases, scintillation), of one RB's
// bandwidth, less the antenna gain
double
PfdRbLimitDbm(double pfd, double marginDb, double distM, double antennaGainDb, double rbHz)
{
    const double eirpDbwPerMHz = pfd + 10.0 * std::log10(4.0 * M_PI * distM * distM) + marginDb;
    return eirpDbwPerMHz + 30.0 + 10.0 * std::log10(rbHz / 1e6) - antennaGainDb;
}

// The DCIs the cell sends of its own, in the common search space, as the report names them
struct CommonDci
{
    NrPdcch::Purpose purpose;
    const char* name;
};

const CommonDci kCommonDcis[] = {{NrPdcch::SIB1, "SIB1"},
                                 {NrPdcch::OTHER_SI, "other SI"},
                                 {NrPdcch::PAGING, "paging"},
                                 {NrPdcch::RAR, "RAR"},
                                 {NrPdcch::TPC_GROUP, "DCI 2_2"}};

// Their levels: one number if they are all at the same level, else each with its name
std::string
CommonLevels(const Ptr<NrPdcch>& pdcch)
{
    bool same = true;
    std::string each;
    for (const auto& c : kCommonDcis)
    {
        const uint8_t al = pdcch->GetCommonAl(c.purpose);
        same = same && al == pdcch->GetCommonAl(kCommonDcis[0].purpose);
        each += (each.empty() ? "" : ", ") + std::string(c.name) + " " + std::to_string(+al);
    }
    return same ? std::to_string(+pdcch->GetCommonAl(kCommonDcis[0].purpose)) : each;
}

// The names that share a value, in the order the values first come: "37: SIB1, paging; 40: RRC"
std::string
GroupedByValue(const std::vector<std::pair<std::string, std::string>>& valueAndName)
{
    std::vector<std::pair<std::string, std::string>> groups;
    for (const auto& [value, name] : valueAndName)
    {
        auto it = std::find_if(groups.begin(), groups.end(), [&](const auto& g) {
            return g.first == value;
        });
        if (it == groups.end())
        {
            groups.emplace_back(value, name);
        }
        else
        {
            it->second += ", " + name;
        }
    }
    std::string text;
    for (const auto& [value, names] : groups)
    {
        text += (text.empty() ? "" : "; ") + value + ": " + names;
    }
    return text;
}

// One cell: one beam of one satellite and its gNB. Cell 0 is the beam on UE 0, the only one of a
// run without --beams or --satellites.
struct Cell
{
    uint32_t sat{0};   // the satellite it is on
    uint32_t beam{0};  // its place in the satellite's layout (0 = the centre beam)
    int tier{0};       // the ring of the layout it is in (0 = the centre beam)
    int colour{0};     // its frequency colour (--beamReuse)
    Ptr<GeocentricConstantPositionMobilityModel> mobility; // the gNB's place: its satellite's
    Vector centre;     // the place on the ground it points at, ECEF
    double northKm{0.0}; // where that is, seen from cell 0's centre
    double eastKm{0.0};
    uint16_t cellId{0};
    Ptr<NrGnbNetDevice> gnb;
    std::vector<Ptr<UniformPlanarArray>> satArrays; // one per BWP
    Ptr<NrGnbPhy> dlPhy;
    Ptr<PropagationLossModel> dlServiceLoss; // the path loss chain of its downlink carrier
    Ptr<ServiceLinkLossModel> dlExtra;       // the last of that chain: the gases
    double dlRbMaxDbm{NAN};                  // what the PFD limit allows an RB now
};

struct Scene
{
    std::vector<Cell> cells;      // see Cell
    std::vector<uint32_t> cellOf; // the cell each UE is attached to
    std::unordered_map<const MobilityModel*, uint32_t> cellOfMobility; // the cell of a gNB
    std::unordered_map<const MobilityModel*, uint32_t> ueOfMobility;   // the UE of a UE's mobility
    std::map<uint16_t, uint32_t> cellOfId;                             // the cell of a cell id
    double pruneDb{40.0};         // --beamPruneDb
    // The beam pattern of every cell towards every UE, dB below the peak of the beam, for each
    // carrier (0 = downlink, 1 = uplink): [carrier][ue * cells + cell], worked out again every
    // 100 ms with the geometry (UpdateTaper()), as the UEs' antenna gain is with one beam
    std::array<std::vector<double>, 2> taperDb;
    // ... and whether the link of that pair is left out (--beamPruneDb)
    std::array<std::vector<uint8_t>, 2> pruned;
    std::vector<OrbitPredictor> orbits; // one per satellite
    // True with more than one cell: the beam pattern is a loss of the link between a beam and
    // a UE (BeamTaperLossModel), not the gain of the UE's antenna, which has only one value for
    // all the beams it hears
    bool taperInLoss{false};
    double beamFloorDb{-30.0};    // the lowest the beam pattern goes (dB below the peak)
    std::vector<Ptr<FixedMobilityModel>> ues;
    Ptr<NrNtnPropagationDelayModel> delay;
    Ptr<ChannelConditionModel> condition;           // carrier of the voice direction
    Ptr<FixedLevelChannelConditionModel> levels;    // blocking and shadowing per UE
    std::vector<std::vector<Ptr<IsotropicAntennaModel>>> ueElements; // per UE, one per BWP
    std::vector<std::vector<Ptr<PhasedArrayModel>>> ueArrays;        // per UE, one per BWP
    double ueAntennaGainDbi{0.0};
    std::vector<double> beamKa; // ka of the beam pattern per BWP: 0 = downlink, 1 = uplink carrier
    bool flying{false};      // --freezeGeometry=0
    double groundDelayMs{0}; // far end to gateway: remote host or far gNB, UPF, gateway
    // Power control (--dlPowerRefDeg, --ulPowerRefDeg; 0 = off). The path loss of the reference,
    // a UE on the beam centre with the satellite at that elevation, and what the power is
    // without control
    double dlPowerRefDeg{0.0};
    double dlRefLossDb{0.0}; // free space and gases at dlPowerRefDeg, downlink carrier
    double dlNominalDbm{0.0};
    double dlFrequencyHz{2e9};
    double ulPowerRefDeg{0.0};
    double ulRefLossDb{0.0}; // the path loss a UE on the beam centre at ulPowerRefDeg measures
    double ueMaxDbm{23.0};
    // --dlPfdLimit (0 = off): the most an RB sends, so that the power flux density at the beam
    // centre stays under the limit; --dlRbPowerMinDbm: the floor of --dlPowerRefDeg's cut
    double dlPfdLimit{0.0};
    double pdcchBoostCapDb{0.0}; // --ns3::NrPdcch::BoostMaxDb: the PFD limit bounds it further
    double dlPfdMarginDb{0.0};
    double dlRbMinDbm{0.0};
    double dlBoostMaxDb{0.0};
    double dlAntennaGainDb{0.0};
    double dlRbHz{180e3};
    uint32_t carrierRbs{1};
    std::vector<std::vector<Ptr<NrUePhy>>> uePhys; // per UE, one per BWP
    std::vector<double> ueLossDb;                  // the path loss each UE measured last
};
Scene g_scene;

// The beam pattern as a loss of the link between a beam and a UE, when there is more than one
// cell: the gain of the cell's beam towards the UE, relative to the beam's peak (<= 0 dB,
// BeamTaperDb()). With one cell the pattern is the gain of the UE's antenna, which is the same
// for all the links of the UE; with several a UE hears several beams, each at another angle, and
// the gain is a property of the pair. Put in the chain of every carrier; a link without a beam
// is passed on as it is.
class BeamTaperLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabBeamTaperLossModel")
                                .SetParent<PropagationLossModel>()
                                .SetGroupName("Propagation")
                                .AddConstructor<BeamTaperLossModel>();
        return tid;
    }

    // Which of the beam patterns of the scene it applies: 0 = downlink carrier, 1 = uplink
    void SetCarrier(std::size_t carrier)
    {
        m_carrier = carrier;
    }

  private:
    double DoCalcRxPower(double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override
    {
        auto cell = g_scene.cellOfMobility.find(PeekPointer(a));
        auto ue = g_scene.ueOfMobility.find(PeekPointer(b));
        if (cell == g_scene.cellOfMobility.end() || ue == g_scene.ueOfMobility.end())
        {
            cell = g_scene.cellOfMobility.find(PeekPointer(b));
            ue = g_scene.ueOfMobility.find(PeekPointer(a));
            if (cell == g_scene.cellOfMobility.end() || ue == g_scene.ueOfMobility.end())
            {
                return txPowerDbm;
            }
        }
        return txPowerDbm +
               g_scene.taperDb[m_carrier][ue->second * g_scene.cells.size() + cell->second];
    }

    int64_t DoAssignStreams(int64_t /* stream */) override
    {
        return 0;
    }

    std::size_t m_carrier{0};
};

// With more than one cell the channel would compute every gNB - UE pair of a carrier, and with
// the fading of the 3GPP model that is the cost of the run. A beam whose pattern is
// --beamPruneDb under the serving beam's towards the UE is 40 dB or more under its signal and
// adds nothing but computing time: its link is not computed. The serving link and every beam
// within that distance are.
class BeamPairFilter : public SpectrumTransmitFilter
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabBeamPairFilter")
                                .SetParent<SpectrumTransmitFilter>()
                                .SetGroupName("Propagation")
                                .AddConstructor<BeamPairFilter>();
        return tid;
    }

    void SetCarrier(std::size_t carrier)
    {
        m_carrier = carrier;
    }

    bool DoFilter(Ptr<const SpectrumSignalParameters> params,
                  Ptr<const SpectrumPhy> receiverPhy) override
    {
        auto tx = DynamicCast<const NrSpectrumPhy>(params->txPhy);
        auto rx = DynamicCast<const NrSpectrumPhy>(receiverPhy);
        if (!tx || !rx)
        {
            return false;
        }
        if (tx->IsGnb() == rx->IsGnb())
        {
            // two gNBs hear each other on a downlink carrier and nothing uses it (the UE-UE
            // pairs are NrNtnUeToUeFilter's)
            return tx->IsGnb();
        }
        const auto& gnb = tx->IsGnb() ? tx : rx;
        const auto& ue = tx->IsGnb() ? rx : tx;
        const auto cell = g_scene.cellOfMobility.find(PeekPointer(gnb->GetMobility()));
        const auto who = g_scene.ueOfMobility.find(PeekPointer(ue->GetMobility()));
        if (cell == g_scene.cellOfMobility.end() || who == g_scene.ueOfMobility.end())
        {
            return false;
        }
        return g_scene.pruned[m_carrier][who->second * g_scene.cells.size() + cell->second] != 0;
    }

  protected:
    int64_t DoAssignStreams(int64_t /* stream */) override
    {
        return 0;
    }

  private:
    std::size_t m_carrier{0};
};

// The beam pattern of every cell towards every UE, for the geometry of now (Scene::taperDb), and
// which links are left out (Scene::pruned)
void
UpdateTaper()
{
    const std::size_t cells = g_scene.cells.size();
    const double floorDb = g_scene.beamFloorDb;
    for (std::size_t carrier = 0; carrier < 2; ++carrier)
    {
        g_scene.taperDb[carrier].resize(g_scene.ues.size() * cells);
        g_scene.pruned[carrier].resize(g_scene.ues.size() * cells);
    }
    std::vector<Vector> satAt(cells);
    for (std::size_t c = 0; c < cells; ++c)
    {
        satAt[c] = g_scene.cells[c].mobility->GetPosition();
    }
    for (std::size_t i = 0; i < g_scene.ues.size(); ++i)
    {
        const Vector ue = g_scene.ues[i]->GetPosition();
        for (std::size_t carrier = 0; carrier < 2; ++carrier)
        {
            double* row = &g_scene.taperDb[carrier][i * cells];
            for (std::size_t c = 0; c < cells; ++c)
            {
                row[c] = BeamTaperDb(satAt[c],
                                     g_scene.cells[c].centre,
                                     ue,
                                     g_scene.beamKa[carrier],
                                     floorDb);
            }
            const double servingDb = row[g_scene.cellOf[i]];
            uint8_t* gone = &g_scene.pruned[carrier][i * cells];
            for (std::size_t c = 0; c < cells; ++c)
            {
                gone[c] = (c != g_scene.cellOf[i] && row[c] < servingDb - g_scene.pruneDb) ? 1 : 0;
            }
        }
    }
}

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
    const Cell& cell = g_scene.cells[g_scene.cellOf[i]];
    const Vector sat = cell.mobility->GetPosition();
    const Vector ue = g_scene.ues[i]->GetPosition();
    CallView& view = g_view[i];
    if (g_scene.flying)
    {
        const Time now = Simulator::Now();
        const OrbitPredictor& orbit = g_scene.orbits[cell.sat];
        const bool closer = (orbit.At(now + MilliSeconds(1)) - ue).GetLength() <
                            (orbit.At(now) - ue).GetLength();
        view.sideAtEnd = closer ? -1.0 : 1.0;
        if (view.elevationDeg.n == 0)
        {
            view.sideAtStart = view.sideAtEnd;
        }
    }
    view.elevationDeg.Add(cell.mobility->GetElevationAngle(g_scene.ues[i]));
    g_snrStep[i] = ElevationStep(view.elevationDeg.last, g_scene.flying ? view.sideAtEnd : 0.0);
    view.rangeKm.Add((sat - ue).GetLength() / 1000.0);
    view.delayMs.Add(
        g_scene.groundDelayMs +
        g_scene.delay->GetOneWayDelay(cell.mobility, g_scene.ues[i]).GetSeconds() * 1000.0);
    for (std::size_t c = 0; c < view.taperDb.size(); ++c)
    {
        view.taperDb[c].Add(
            BeamTaperDb(sat, cell.centre, ue, g_scene.beamKa[c], g_scene.beamFloorDb));
    }
    if (g_scene.levels)
    {
        view.shadowDb.Add(g_scene.levels->ShadowGainDb(cell.mobility, g_scene.ues[i]));
    }
    if (g_scene.ulPowerRefDeg > 0.0)
    {
        view.ulPowerDbm.Add(g_scene.uePhys[i][1]->GetTxPower());
        view.ulLossDb.Add(g_scene.ueLossDb[i]);
    }
    if (g_scene.dlPowerRefDeg > 0.0)
    {
        view.dlCutDb.Add(g_scene.dlNominalDbm - cell.dlPhy->GetTxPower());
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
            g_scene.condition
                ->GetChannelCondition(g_scene.cells[g_scene.cellOf[i]].mobility, g_scene.ues[i])
                ->IsLos();
        CallView& view = g_view[i];
        if (Simulator::Now() >= view.callStart && Simulator::Now() <= view.callEnd)
        {
            ++view.losAll;
            view.losYes += los;
        }
    }
    Simulator::Schedule(MilliSeconds(100), &SampleLos);
}

// The satellite turns the arrays of each beam (one per BWP) towards the beam's centre. The
// channel model works in ECEF, so the two angles are ECEF angles.
void
PointBeam()
{
    for (Cell& cell : g_scene.cells)
    {
        const Angles toCentre(cell.centre, cell.mobility->GetPosition());
        for (auto& array : cell.satArrays)
        {
            array->SetAlpha(toCentre.GetAzimuth());
            array->SetBeta(toCentre.GetInclination());
            array->SetBeamformingVector(array->GetBeamformingVector());
        }
    }
}

// --dlPowerRefDeg: the satellite lowers the power of the beam by what the free-space loss and the
// gases to the beam centre are below their values with the satellite at dlPowerRefDeg above it
// (a higher satellite is nearer). It never sends more than without the control.
void
DlPowerControl(Cell& cell)
{
    const Vector sat = cell.mobility->GetPosition();
    const double lossDb = FeederLinkLossModel::FreeSpaceDb(g_scene.dlFrequencyHz,
                                                           (sat - cell.centre).GetLength()) +
                          cell.dlExtra->AtmosphericDb(ElevationDeg(sat, cell.centre));
    double powerDbm = g_scene.dlNominalDbm - std::max(0.0, g_scene.dlRefLossDb - lossDb);
    if (g_scene.dlRbMinDbm > 0.0)
    {
        // an RB never under --dlRbPowerMinDbm (nor the satellite over full power)
        powerDbm = std::min(
            g_scene.dlNominalDbm,
            std::max(powerDbm, g_scene.dlRbMinDbm + 10.0 * std::log10(g_scene.carrierRbs)));
    }
    cell.dlPhy->SetTxPower(powerDbm);
}

// --dlPfdLimit: the most an RB may send now, from the distance to the beam centre, as the extra
// power an RB in use may take over its share of TxPower (NrGnbPhy MaxRbBoostDb: negative if the
// limit is under the share). --dlBoostMaxDb is a second cap on it.
void
DlPfdLimit(Cell& cell)
{
    const double distM = (cell.mobility->GetPosition() - cell.centre).GetLength();
    cell.dlRbMaxDbm = PfdRbLimitDbm(g_scene.dlPfdLimit,
                                    g_scene.dlPfdMarginDb,
                                    distM,
                                    g_scene.dlAntennaGainDb,
                                    g_scene.dlRbHz);
    const double shareDbm = cell.dlPhy->GetTxPower() - 10.0 * std::log10(g_scene.carrierRbs);
    double boostDb = cell.dlRbMaxDbm - shareDbm;
    if (g_scene.dlBoostMaxDb > 0.0)
    {
        boostDb = std::min(boostDb, g_scene.dlBoostMaxDb);
    }
    cell.dlPhy->SetAttribute("MaxRbBoostDb", DoubleValue(std::clamp(boostDb, -100.0, 100.0)));
    if (g_scene.pdcchBoostCapDb > 0.0)
    {
        // The PDCCH is subject to the same limit: a boosted resource element of it is an RB of
        // the PDSCH as far as the power flux density goes
        NrPdcch::Get(cell.dlPhy->GetCellId())
            ->SetAttribute("BoostMaxDb",
                           DoubleValue(std::clamp(std::min(g_scene.pdcchBoostCapDb, boostDb),
                                                  0.0,
                                                  30.0)));
    }
}

// --ulPowerRefDeg: open-loop power control of TS 38.213 7.1.1 with alpha = 1. A UE measures its
// path loss on the downlink: the loss of the downlink carrier's chain and its beam gain, not the
// fast fading, which the filtered RSRP averages out. It sends P0 + 10 log10(M) + PL, with P0 such
// that the reference UE (on the beam centre, the satellite at ulPowerRefDeg) sends all its
// power: so a UE sends what its path loss is below the reference's less, never more than all.
void
UlPowerControl()
{
    for (std::size_t i = 0; i < g_scene.ues.size(); ++i)
    {
        const Cell& cell = g_scene.cells[g_scene.cellOf[i]];
        const Vector sat = cell.mobility->GetPosition();
        // (with the beam pattern in the loss chain, the chain has the beam gain already)
        const double lossDb =
            -cell.dlServiceLoss->CalcRxPower(0.0, cell.mobility, g_scene.ues[i]) -
            (g_scene.taperInLoss ? 0.0
                                 : BeamTaperDb(sat,
                                               cell.centre,
                                               g_scene.ues[i]->GetPosition(),
                                               g_scene.beamKa[0]));
        g_scene.ueLossDb[i] = lossDb;
        ApplyUlPower(i);
    }
}

// --- The closed loops (--dlClosedLoop, --ulClosedLoop): the parts that need the scene -----------

// What a UE sends without the closed loop: all its power, or the open-loop power
// (--ulPowerRefDeg)
double
UlBasePowerDbm(uint32_t ue)
{
    return g_scene.ulPowerRefDeg > 0.0
               ? std::min(g_scene.ueMaxDbm,
                          g_scene.ueMaxDbm - (g_scene.ulRefLossDb - g_scene.ueLossDb[ue]))
               : g_scene.ueMaxDbm;
}

// UE i's power on its uplink carrier now
double
UlTxPowerDbm(uint32_t ue)
{
    return g_scene.uePhys[ue][1]->GetTxPower();
}

// What the UE sends: min{P_CMAX, P_open-loop + f} of TS 38.213 7.1.1, with f the closed loop's
// offset (0 or less), and not under the least a UE can send (TS 38.101-1 Table 6.3.1-1)
void
ApplyUlPower(uint32_t ue)
{
    double powerDbm = UlBasePowerDbm(ue);
    if (g_loop.ul)
    {
        powerDbm = std::max(std::min(g_loop.ulMinDbm, powerDbm), powerDbm + g_ulLoop[ue].offsetDb);
    }
    for (const auto& phy : g_scene.uePhys[ue])
    {
        phy->SetTxPower(powerDbm);
    }
}

// The power of an RB with no boost: its share of the satellite's TxPower
double
DlShareDbm(uint32_t ue)
{
    return g_scene.cells[g_scene.cellOf[ue]].dlPhy->GetTxPower() -
           10.0 * std::log10(g_scene.carrierRbs);
}

// UE i has a connection, with this RNTI (ConnectionEstablished): the loops start from scratch. A
// new connection starts with all the UE's power (random access) and with the whole boost the
// satellite may give its RBs: nothing is known of the UE until its first period, whose decision
// then goes to what it needs at once.
void
LoopOnConnected(uint32_t ue, uint16_t rnti)
{
    if (g_loop.dl != 0)
    {
        DlLoopUe& s = g_dlLoop[ue];
        if (s.rnti != 0 && s.rnti != rnti &&
            g_rntiUe[RntiKey(g_scene.cells[g_scene.cellOf[ue]].cellId, s.rnti)] == ue)
        {
            g_scene.cells[g_scene.cellOf[ue]].dlPhy->RemoveUeMaxRbBoostDb(s.rnti);
        }
        s = DlLoopUe();
        s.rnti = rnti;
        s.capDb = g_loop.dlMaxDb;
        g_scene.cells[g_scene.cellOf[ue]].dlPhy->SetUeMaxRbBoostDb(rnti, s.capDb);
    }
    if (g_loop.ul)
    {
        g_ulLoop[ue] = UlLoopUe();
        ApplyUlPower(ue);
    }
}

// UE i has no connection: no cap on its RBs, all its power
void
LoopReset(uint32_t ue)
{
    if (g_loop.dl != 0)
    {
        DlLoopUe& s = g_dlLoop[ue];
        if (s.rnti != 0 && g_rntiUe[RntiKey(g_scene.cells[g_scene.cellOf[ue]].cellId, s.rnti)] == ue)
        {
            g_scene.cells[g_scene.cellOf[ue]].dlPhy->RemoveUeMaxRbBoostDb(s.rnti);
        }
        s = DlLoopUe();
    }
    if (g_loop.ul)
    {
        g_ulLoop[ue] = UlLoopUe();
        ApplyUlPower(ue);
    }
}

// The share of the way an estimate moves to a new period's value, g_loop.filter: the estimates
// average about --closedLoopAvg periods (3). Both loops are first-order filters of the SNR, so on
// a channel that drifts at r dB per second they read the SNR late by about avg x period x r: low
// while it rises, high while it falls (the satellite rises, then sets).

// Where a loop puts the SNR when it acts (g_loop.landing, --closedLoopLanding, default 0.2): that
// share of the band's width in from the FAR edge.
// A loop that acts only outside its band and puts the SNR back to the middle of it (what the loops
// did before) holds the SNR in the half of the band the SNR comes from: in its upper half while
// the channel gets better (the satellite rises), in its lower half while it gets worse (it sets),
// so the SNR of the rising half of a pass reads the width of the band higher than the setting
// half. Landing near the other edge instead makes the SNR cross the whole band between two
// commands, in either direction: the same mean, whatever way the channel goes. Not on the edge
// itself: with the scatter of the estimate the loop would act again at once. What is left of the
// difference between the two halves of a pass is landing x band width (0.2 dB for the 1 dB band
// of the downlink loop) plus twice the lag of the filter (avg x period x r, about 0.3 dB at 1 s
// and 0.05 dB/s, the rate between 30 and 60 degrees at 600 km).

// The downlink decision for UE i. The UE's SNR over a period, less the boost its RBs had, is its
// SNR at the share; against what it needs (--dlSnrMinDb; the SNR of --dlMcsMin on the RBs it gets;
// the higher of the two for 3) that is the boost it needs. The estimate is filtered over the
// periods (the SNR of a period scatters with the fading, by 2 dB or so). The SNR the cap gives is
// the target plus (cap - need). When that is under the target, or a step or more over it, the cap
// goes to the other side of that band (g_loop.landing): up at once, down by a step at most; in the
// band, nothing changes. The first decision of a connection goes to the need plus half a step at
// once. The cap never goes under 0 (no boost) or over the top (--dlBoostMaxDb).
// It is not an integral of the SNR, so it does not wind up when the satellite has no power to
// give: the SNR is the one of the share, whatever the RBs got.
void
DlLoopDecide(uint32_t ue)
{
    DlLoopUe& s = g_dlLoop[ue];
    if (s.rnti == 0 || s.n < g_loop.dlMinSamples || s.usedN == 0)
    {
        return;
    }
    LoopTally& t = g_loopTally[ue];
    const double snrDb = s.snrSumDb / s.n;
    double needDb = -INFINITY;
    if ((g_loop.dl & 1) != 0)
    {
        needDb = g_loop.dlSnrMinDb;
    }
    if ((g_loop.dl & 2) != 0)
    {
        needDb = std::max(needDb, s.needSumDb / s.n);
    }
    const double usedDb = s.usedSumDb / s.usedN;
    const double needBoostDb = needDb - (snrDb - usedDb);
    const double step = g_loop.stepDb;
    const bool call = InSnrWindow(ue);
    double capDb = s.capDb;
    if (!s.settled)
    {
        s.needBoostDb = needBoostDb;
        capDb = std::clamp(s.needBoostDb + 0.5 * step, 0.0, g_loop.dlMaxDb);
        s.settled = true;
    }
    else
    {
        s.needBoostDb += g_loop.filter * (needBoostDb - s.needBoostDb);
        if (s.capDb < s.needBoostDb || s.capDb >= s.needBoostDb + step)
        {
            // The UE's SNR is its target plus (cap - need): in the band from the need to a step
            // above it. Short of it: to the upper side of the band; with a step to spare: to the
            // lower side (g_loop.landing)
            const double landDb = s.capDb < s.needBoostDb
                                      ? s.needBoostDb + (1.0 - g_loop.landing) * step
                                      : s.needBoostDb + g_loop.landing * step;
            const double toDb = std::clamp(landDb, 0.0, g_loop.dlMaxDb);
            capDb = toDb > s.capDb ? toDb : std::max(toDb, s.capDb - step);
        }
    }
    if (call)
    {
        ++t.dlWin;
        t.dlWinBelow += snrDb < needDb;
        t.dlAtMax += s.needBoostDb > g_loop.dlMaxDb;
        ++(capDb > s.capDb + 1e-9 ? t.dlUp : capDb < s.capDb - 1e-9 ? t.dlDown : t.dlHold);
        t.dlCap.Add(capDb);
    }
    if (capDb != s.capDb)
    {
        s.capDb = capDb;
        g_scene.cells[g_scene.cellOf[ue]].dlPhy->SetUeMaxRbBoostDb(s.rnti, capDb);
    }
    s.n = 0;
    s.snrSumDb = 0.0;
    s.needSumDb = 0.0;
    s.usedN = 0;
    s.usedSumDb = 0.0;
}

// The uplink decision for UE i, a TPC command of the gNB (TS 38.213 7.1.1, accumulated: f grows
// by the command). The UE's SNR over a period, less the offset f its PUSCH had, is the SNR it has
// at the power it has without the loop; filtered over the periods, plus the f it has now, that is
// the SNR it has. Above --ulSnrHighDb: lower f, to the far side of the band (g_loop.landing), by one
// step at most (the command -1 dB); under --ulSnrLowDb: raise it, to the far side of the band, by
// three steps at most (+3 dB); in the band: no command. f is never over 0
// (the UE does not send more than without the loop) nor under what the least power allows; a
// command that points over a limit changes nothing, as in 7.1.1.
void
UlLoopDecide(uint32_t ue)
{
    UlLoopUe& s = g_ulLoop[ue];
    if (s.n < g_loop.ulMinSamples)
    {
        return;
    }
    LoopTally& t = g_loopTally[ue];
    const double snrDb = s.snrSumDb / s.n;
    const double snrFullDb = snrDb - s.offSumDb / s.n;
    s.snrFullDb = s.settled ? s.snrFullDb + g_loop.filter * (snrFullDb - s.snrFullDb) : snrFullDb;
    s.settled = true;
    const double step = g_loop.stepDb;
    const double lowestDb = std::min(0.0, g_loop.ulMinDbm - UlBasePowerDbm(ue));
    const double nowDb = s.snrFullDb + s.offsetDb;
    double commandDb = 0.0;
    if (nowDb > g_loop.ulHighDb || nowDb < g_loop.ulLowDb)
    {
        // to the far side of the band (g_loop.landing of its width from the far edge)
        const double widthDb = g_loop.ulHighDb - g_loop.ulLowDb;
        const double landDb = nowDb > g_loop.ulHighDb ? g_loop.ulLowDb + g_loop.landing * widthDb
                                                      : g_loop.ulHighDb - g_loop.landing * widthDb;
        const double toDb = std::clamp(landDb - s.snrFullDb, lowestDb, 0.0);
        commandDb = std::clamp(toDb - s.offsetDb, -step, 3.0 * step);
    }
    const double offsetDb = std::clamp(s.offsetDb + commandDb, lowestDb, 0.0);
    if (InSnrWindow(ue))
    {
        ++t.ulWin;
        t.ulWinInBand += snrDb >= g_loop.ulLowDb && snrDb <= g_loop.ulHighDb;
        ++(offsetDb > s.offsetDb + 1e-9   ? t.ulUp
           : offsetDb < s.offsetDb - 1e-9 ? t.ulDown
                                          : t.ulHold);
        t.ulAtMin += nowDb > g_loop.ulHighDb && offsetDb <= lowestDb + 1e-9;
    }
    if (offsetDb != s.offsetDb)
    {
        s.offsetDb = offsetDb;
        ApplyUlPower(ue);
    }
    s.n = 0;
    s.snrSumDb = 0.0;
    s.offSumDb = 0.0;
}

// Once per --closedLoopMs, for every UE: a UE without a connection starts from scratch
void
ClosedLoopTick()
{
    for (uint32_t i = 0; i < g_scene.ues.size(); ++i)
    {
        if (!g_loop.connected(i))
        {
            LoopReset(i);
            continue;
        }
        if (g_loop.dl != 0)
        {
            DlLoopDecide(i);
        }
        if (g_loop.ul)
        {
            UlLoopDecide(i);
        }
    }
    Simulator::Schedule(g_loop.period, &ClosedLoopTick);
}

// --freezeGeometry=0 only, every 100 ms: the beam stays on its centre and every UE's antenna gain
// gets the beam taper of the new geometry, as leo-link's UpdateAntennaOrientation() and
// UpdateSpotBeamTaper() do, here for each carrier with its own beam pattern. Setting the
// beamforming vector again drops the cached channel. The power control follows the geometry.
void
FollowBeam()
{
    if (g_scene.taperInLoss)
    {
        UpdateTaper();
    }
    PointBeam();
    for (std::size_t i = 0; i < g_scene.ues.size(); ++i)
    {
        const Cell& cell = g_scene.cells[g_scene.cellOf[i]];
        // (with the beam pattern in the loss chain the gain of the antenna stays: the chain
        // follows the geometry by itself)
        for (std::size_t bwp = 0; !g_scene.taperInLoss && bwp < g_scene.ueArrays[i].size(); ++bwp)
        {
            const double taperDb = BeamTaperDb(cell.mobility->GetPosition(),
                                               cell.centre,
                                               g_scene.ues[i]->GetPosition(),
                                               g_scene.beamKa[bwp]);
            g_scene.ueElements[i][bwp]->SetAttribute(
                "Gain",
                DoubleValue(g_scene.ueAntennaGainDbi + taperDb));
        }
        for (auto& array : g_scene.ueArrays[i])
        {
            array->SetBeamformingVector(array->GetBeamformingVector());
        }
    }
    for (Cell& cell : g_scene.cells)
    {
        if (g_scene.dlPowerRefDeg > 0.0)
        {
            DlPowerControl(cell);
        }
        if (g_scene.dlPfdLimit != 0.0)
        {
            DlPfdLimit(cell);
        }
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

void RetryBearersUp(uint32_t ue); // (call control, below)

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
        RetryBearersUp(it->second); // a UE that called again has its call from now on
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
                 double fastFadingDb,
                 double fadingHoldMs,
                 const std::set<uint32_t>& satelliteIds,
                 int64_t firstStream,
                 char tdlProfile = 0,
                 double tdlDelaySpreadS = 10e-9,
                 double tdlDopplerHz = 0.0,
                 int beamCarrier = -1)
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
        if (fixedFastFading || tdlProfile != 0)
        {
            // --fastFading=fixed: the direct path alone, and --fastFadingDb; ntn-tdl-*: the
            // same with the taps of the profile
            Ptr<ThreeGppSpectrumPropagationLossModel> threeGpp =
                DynamicCast<ThreeGppSpectrumPropagationLossModel>(fading);
            NS_ABORT_MSG_UNLESS(threeGpp, "unexpected fast fading model");
            Ptr<LosOnlyChannelModel> los = CreateObject<LosOnlyChannelModel>();
            los->SetAttribute("Frequency", DoubleValue(frequencyHz));
            los->SetLossDb(fastFadingDb);
            if (tdlProfile != 0)
            {
                los->SetTdl(tdlProfile, tdlDelaySpreadS, tdlDopplerHz, firstStream);
            }
            threeGpp->SetChannelModel(los);
        }
        else if (fadingHoldMs > 0.0)
        {
            // --fadingHoldMs: ns-3's channel, but each realisation stays for that long
            Ptr<ThreeGppSpectrumPropagationLossModel> threeGpp =
                DynamicCast<ThreeGppSpectrumPropagationLossModel>(fading);
            NS_ABORT_MSG_UNLESS(threeGpp, "unexpected fast fading model");
            Ptr<ThreeGppChannelModel> model =
                DynamicCast<ThreeGppChannelModel>(threeGpp->GetChannelModel());
            NS_ABORT_MSG_UNLESS(model, "unexpected channel model");
            Ptr<HeldChannelModel> held = CreateObject<HeldChannelModel>();
            held->SetAttribute("Frequency", DoubleValue(frequencyHz));
            held->Set(model,
                      NanoSeconds(std::llround(fadingHoldMs * 1e6)),
                      satelliteIds,
                      firstStream);
            threeGpp->SetChannelModel(held);
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
    if (beamCarrier >= 0)
    {
        // more than one cell: the beam pattern of each beam towards each UE is a loss of the link
        Ptr<BeamTaperLossModel> taper = CreateObject<BeamTaperLossModel>();
        taper->SetCarrier(beamCarrier);
        last->SetNext(taper);
        last = taper;
    }
    last->SetNext(carrier.serviceExtra);
    carrier.channel->SetPropagationDelayModel(delayModel);
    carrier.channel->AddSpectrumTransmitFilter(CreateObject<NrNtnUeToUeFilter>());
    if (beamCarrier >= 0)
    {
        Ptr<BeamPairFilter> prune = CreateObject<BeamPairFilter>();
        prune->SetCarrier(beamCarrier);
        carrier.channel->AddSpectrumTransmitFilter(prune);
    }

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

    // The call ends early: not admitted (before it starts) or released by the network. The
    // socket stays open until the application stops (the end of the call's window): a call that
    // starts again (Restart) then sends from the same address and port, and the flow monitor
    // sees one flow, not one for each try.
    void End()
    {
        m_ended = true;
        m_frameEvent.Cancel();
    }

    // The call starts again after a release (--retry): the speech detector as after a reset,
    // a new RTP session (another SSRC), the same socket. (The new bearer is up by now.)
    void Restart()
    {
        m_frameEvent.Cancel();
        m_ended = false;
        m_talking = false;
        m_hangoverLeft = m_hangoverFrames;
        m_inDtx = false;
        m_framesInDtx = 0;
        m_lastWasVoice = false;
        m_ssrc += 0x100;
        if (!m_socket)
        {
            Open();
        }
        Frame();
    }

  private:
    void Open()
    {
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_socket->Bind();
        m_socket->Connect(m_remote);
        m_socket->ShutdownRecv();
    }

    void StartApplication() override
    {
        if (m_ended)
        {
            return; // the call was not admitted
        }
        Open();
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
    std::array<double, 3> capacity{}; // per second; 0 = no limit (the same for every cell)
    std::vector<std::array<double, 3>> reserved; // per cell
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
    std::vector<uint32_t> peakAdmitted; // per cell
    // the most the admitted calls reserved, of the capacity, per cell
    std::vector<std::array<double, 3>> peakShare;

    // ... and the highest of the cells
    uint32_t PeakAdmitted() const
    {
        return peakAdmitted.empty() ? 0 : *std::max_element(peakAdmitted.begin(), peakAdmitted.end());
    }

    double PeakShare(int r) const
    {
        double most = 0.0;
        for (const auto& cell : peakShare)
        {
            most = std::max(most, cell[r]);
        }
        return most;
    }

    // --retry: a UE whose call was released, or blocked (the gNB then releases its RRC
    // connection), asks for a connection again after retryDelay (a new RNTI, the same IP
    // address), and when its bearers are there the gNB admits a new call
    // for it and its voice starts again, until it has made maxTries attempts. status, at, ...
    // above are those of the UE's latest attempt; attempts has all of them.
    struct Attempt
    {
        uint32_t number{1};      // 1 = the call the plan starts, then the retries
        Time began;              // the call was due (1), or the UE asked for a connection
        Time at{Time::Max()};    // admitted or blocked; Time::Max(): no bearers before the end
        Time end{Time::Max()};   // released, ended as planned, or (blocked) refused
        Status outcome{WAITING}; // WAITING: no connection yet; then ADMITTED/BLOCKED/RELEASED/ENDED
        int limitedBy{-1};       // blocked: the resource that had no room
        double onTimePct{NAN};   // released: the share of its packets within the PDB
        int releasedDir{-1};     // released: the direction that missed (Dir)
        int firstConnection{0};  // the first of its connections (index into g_connections[ue])
    };
    std::vector<std::vector<Attempt>> attempts;
    bool retry{false};
    Time retryDelay{Seconds(3)};
    uint32_t maxTries{1};
    std::vector<bool> retrying; // UE i asked for a connection to call again, bearers not yet up
    std::function<void(uint32_t)> reconnectUe; // the NAS of UE i asks for a connection
    std::function<bool(uint32_t)> ueConnected; // UE i's RRC is still connected (to be released)
};

CallControl g_cc;
double g_ulBelowDlDb{3.0}; // the uplink CNR on the CNR's RBs below the downlink one (header)
double g_ue0DlCnrDb{0.0};  // UE 0's downlink CNR at --elevation (header)

const char* const g_resourceName[3] = {"downlink RBs", "uplink RBs", "PDCCH CCEs"};

void RetryCall(uint32_t i); // (below)

// --retry: UE i's call ended without service (released, or blocked): it calls again after
// retryDelay if it has tries left and the window of its call is not over by then
void
ScheduleRetry(uint32_t i)
{
    CallControl& c = g_cc;
    if (c.retry && c.attempts[i].size() < c.maxTries &&
        Simulator::Now() + c.retryDelay < c.end[i])
    {
        Simulator::Schedule(c.retryDelay, &RetryCall, i);
    }
}

// Call i is to start: the call of the plan, or (--retry) the retry of a UE that has its bearers
void
AdmitCall(uint32_t i)
{
    CallControl& c = g_cc;
    const uint32_t cell = g_scene.cellOf[i];
    const std::array<double, 3> need = c.need ? c.need(i) : std::array<double, 3>{};
    // A retry has its attempt since the UE asked for a connection (RetryCall); the call of the
    // plan gets its attempt now
    const bool retried = !c.attempts[i].empty();
    if (!retried)
    {
        CallControl::Attempt first;
        first.began = Simulator::Now();
        c.attempts[i].push_back(first);
    }
    CallControl::Attempt& attempt = c.attempts[i].back();
    attempt.at = Simulator::Now();
    // (the connection it is on: with a retry the new one, which is there before the bearers)
    attempt.firstConnection = std::max<int>(0, static_cast<int>(g_connections[i].size()) - 1);
    if (c.admission)
    {
        for (int r = 0; r < 3; ++r)
        {
            if (c.capacity[r] > 0.0 && c.reserved[cell][r] + need[r] > c.load[r] * c.capacity[r])
            {
                c.status[i] = CallControl::BLOCKED;
                c.at[i] = Simulator::Now();
                c.limitedBy[i] = r;
                attempt.outcome = CallControl::BLOCKED;
                attempt.end = Simulator::Now();
                attempt.limitedBy = r;
                for (const auto& source : c.sources[i])
                {
                    if (source)
                    {
                        source->End();
                    }
                }
                // --retry: a refused call is as good as a released one: the gNB releases the
                // UE's RRC connection and the UE tries again, with a new connection
                if (c.retry && c.releaseUe)
                {
                    c.releaseUe(i);
                    ScheduleRetry(i);
                }
                return;
            }
        }
    }
    c.status[i] = CallControl::ADMITTED;
    attempt.outcome = CallControl::ADMITTED;
    c.at[i] = Simulator::Now();
    c.held[i] = need;
    for (int r = 0; r < 3; ++r)
    {
        c.reserved[cell][r] += need[r];
        if (c.capacity[r] > 0.0)
        {
            c.peakShare[cell][r] = std::max(c.peakShare[cell][r], c.reserved[cell][r] / c.capacity[r]);
        }
    }
    uint32_t admittedHere = 0;
    for (uint32_t u = 0; u < c.status.size(); ++u)
    {
        admittedHere += c.status[u] == CallControl::ADMITTED && g_scene.cellOf[u] == cell;
    }
    c.peakAdmitted[cell] = std::max<uint32_t>(c.peakAdmitted[cell], admittedHere);
    if (c.callStarts)
    {
        c.callStarts(i);
    }
    if (retried)
    {
        // The voice of the call of the plan starts by itself (the applications' start time); a
        // retry starts it now, on the new bearer
        for (const auto& source : c.sources[i])
        {
            if (source)
            {
                source->Restart();
            }
        }
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
        c.attempts[i].back().outcome = CallControl::ENDED;
        c.attempts[i].back().end = Simulator::Now();
        for (int r = 0; r < 3; ++r)
        {
            c.reserved[g_scene.cellOf[i]][r] -= c.held[i][r];
        }
    }
}

// --retry: UE i, released retryDelay ago, wants its call again. It asks for a connection: it
// looks for the cell (it is camped in RRC_IDLE since the release), does the random access,
// gets a new RNTI and, from the core, its QoS flows on new bearers (the address is the same).
// Its call starts when the bearers are there (RetryBearersUp).
void
RetryCall(uint32_t i)
{
    CallControl& c = g_cc;
    if (Simulator::Now() >= c.end[i] || !c.reconnectUe)
    {
        return; // the window of its call is over
    }
    if (c.ueConnected && c.ueConnected(i))
    {
        // (--retrySeconds=0: the release has not reached the UE yet)
        Simulator::Schedule(MilliSeconds(10), &RetryCall, i);
        return;
    }
    CallControl::Attempt attempt;
    attempt.number = static_cast<uint32_t>(c.attempts[i].size()) + 1;
    attempt.began = Simulator::Now();
    c.attempts[i].push_back(attempt);
    c.retrying[i] = true;
    c.reconnectUe(i);
}

// The bearers of UE i are up (the gNB's "ConnectionReconfiguration"): if it is retrying, the gNB
// admits its call now
void
RetryBearersUp(uint32_t i)
{
    CallControl& c = g_cc;
    if (i >= c.retrying.size() || !c.retrying[i])
    {
        return;
    }
    c.retrying[i] = false;
    if (Simulator::Now() < c.end[i])
    {
        AdmitCall(i);
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
    // Only a loaded cell releases calls: the calls on reserve half of one of its resources or
    // more. In a cell with room, late packets come from a bad link (HARQ retransmissions,
    // repetitions), and releasing the call frees nothing.
    std::vector<bool> loaded(c.reserved.size(), false);
    for (std::size_t cell = 0; cell < loaded.size(); ++cell)
    {
        for (int r = 0; r < 3; ++r)
        {
            loaded[cell] = loaded[cell] ||
                           (c.capacity[r] > 0.0 && c.reserved[cell][r] >= 0.5 * c.capacity[r]);
        }
    }
    for (uint32_t i = 0; i < c.status.size(); ++i)
    {
        if (!loaded[g_scene.cellOf[i]] || c.status[i] != CallControl::ADMITTED || CurrentConnection(i) < 0 ||
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
                CallControl::Attempt& attempt = c.attempts[i].back();
                attempt.outcome = CallControl::RELEASED;
                attempt.end = Simulator::Now();
                attempt.onTimePct = c.onTimePct[i];
                attempt.releasedDir = dir;
                for (int r = 0; r < 3; ++r)
                {
                    c.reserved[g_scene.cellOf[i]][r] -= c.held[i][r];
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
                // --retry: the UE calls again, if it has tries left and its window is not over
                ScheduleRetry(i);
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
    double fadingHoldMs = 0.0; // --fastFading=model: the fading stays for this long; 0 = ns-3
    double ntnTdlDelayNs = 10.0;  // --fastFading=ntn-tdl-*: the delay spread the profile is scaled to (TR 38.821: mean DS, suburban LOS, 30 deg)
    double ntnTdlSpeedKmph = 3.0; // ... and the speed whose Doppler its Rayleigh taps have
    double linkMarginDb = 0.0;        // more loss on every link: TR 38.821's additional losses
    double ulPolarizationLossDb = 0.0; // more loss on the uplink only: TR 38.821 6.1.1.1 depolarisation
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
    uint32_t dlClosedLoop = 0;   // closed loop on the DL boost: 0 off, 1 SNR, 2 MCS, 3 both
    double dlSnrMinDb = 4.0;     // --dlClosedLoop: the SNR a UE should have at least (dB)
    uint32_t dlMcsMin = 10;      // --dlClosedLoop: the MCS a UE should have at least
    bool ulClosedLoop = false;   // closed loop on the UE's power: lower it while the SNR is high
    double ulSnrHighDb = 3.0;    // --ulClosedLoop: lower the power while the SNR is above (dB)
    double ulSnrLowDb = 2.0;     // ... but not so far that it falls under this (dB)
    double ulPowerMinDbm = -40.0; // --ulClosedLoop: the least a UE sends (TS 38.101-1 Table 6.3.1-1)
    double closedLoopMs = 1000.0; // period of the closed loops: they decide once per period
    double closedLoopStepDb = 1.0; // ... by this many dB (TS 38.213 Table 7.1.1-1: 1 dB)
    double closedLoopLanding = 0.2; // where a loop puts the SNR in its band when it acts: 0 = far edge
    double closedLoopAvg = 3.0;     // periods the loops' estimates average (a first-order filter)
    double dlPowerRefDeg = 0.0;  // downlink power control: the reference elevation, 0 = off
    double ulPowerRefDeg = 0.0;  // uplink power control (open loop): the same, 0 = off
    double dlPfdLimit = 0.0;     // PFD on the ground (dBW/m2/MHz): an RB's power stays under it
    double dlPfdMarginDb = 5.25; // EIRP over PFD x 4 pi d^2: shadowing, gases, scintillation
    double dlRbPowerMinDbm = 0.0; // the lowest power of an RB in use (dBm), 0 = no floor
    uint32_t ulWideRbs = 0;       // RBs of a wide uplink grant, 0 = none
    double ulWideSinrDb = 5.0;    // SINR on one RB a UE needs for it (dB)
    double ulWideLoadPct = 50.0;  // most load of the slot's uplink RBs at which it is given (%)
    bool harq = true;            // false: no retransmissions, every transport block is sent once
    uint32_t harqProcesses = 16; // per UE and direction (Rel-17 NTN: up to 32)
    bool ulLinkAdaptation = true; // UL MCS for the RBs of each grant, and an outer loop
    bool ulPfMarginal = true;     // OFDMA PF, uplink: rank by what the next RBG adds to the block
    uint32_t puschRepetitions = 0; // slots a UE sends each uplink block in; 0 = as many as needed
    double puschRepSinrDb = 2.0;   // --puschRepetitions=0: K brings K x the SINR up to this (dB)
    uint32_t puschMinBlockBytes = 0; // --puschRepetitions=0: K until the block is this big (B)
    double ulSinrAvgMs = 0.0;        // uplink link adaptation: time constant of the SINR mean (ms)
    uint32_t srGrantBytes = 0;    // grant for a scheduling request; 0 = a voice frame and a BSR
    double targetBler = 0.0;     // of first transmissions; 0 = 0.15 with HARQ, 0.01 without
    bool nrTraces = false; // true: NrHelper::EnableTraces(), about 25 .txt files in the run folder
    bool rohc = NrPdcp::IsRohcAvailable(); // on whenever the nr module was built with the ROHC library
    std::string rohcModel = "table";       // table: reference header sizes, library: real ROHC
    std::string rohcMode = "O";            // U = no feedback, O = feedback PDUs on the reverse link
    uint32_t rohcIrPackets = 4;            // size table: IR packets in a row (optimistic approach)
    uint32_t rohcUpdatePackets = 4;        // size table: context updates in a row for a change
    uint32_t rohcRepairIrPackets = 1;      // size table: IR packets in a row for a STATIC-NACK
    uint32_t rohcFallbackAfter = 0;        // size table, O-mode: failed packets in a row before
                                           // the decompressor asks for U-mode; 0 = never
    uint32_t rohcDynamicWindow = 13;       // size table: packets missing in a row that UO-0 survives
    uint32_t rohcStaticWindow = 61;        // size table: ... that a context update still repairs
    uint32_t rohcNackAfter = 0;  // O-mode: failed packets (library: of the last 32) before the
                                 // NACK; 0 = 1 (library: 10)
    uint32_t rohcNackEvery = 32; // O-mode: ... and before it is sent again
    std::string l2Headers = "1+1+3";       // PDCP + RLC + MAC header bytes, or 2+2+2
    std::string rohcReport = "summary";    // none, summary (one line per call) or full
    double ueBufferAgeS = 0.0;   // a satellite UE's RLC drops voice older than this (s); 0 = never
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
    // More than one beam or satellite: one cell (a gNB and its UEs) for each beam of each satellite
    uint32_t beams = 1;          // beams of a satellite: 1, 7, 19, 37, 61 (TR 38.821 6.1.1.1)
    uint32_t beamReuse = 1;      // frequency reuse factor of the beams: 1, 3, 4, 7
    double beamSpacing = 1.0;    // spacing of the beam centres, in TR 38.821's (1 = ABS)
    double beamPruneDb = 40.0;   // a beam this far under the serving beam at a UE is not computed
    double beamFloorDb = NAN;    // the lowest the beam pattern goes (dB): -30 (1 beam), -80 (more)
    uint32_t satellites = 1;     // satellites, one behind the other along the track
    double satSeparationDeg = 0.0; // their distance along the orbit (deg of arc); 0 = auto
    std::string bsr = "module";   // buffer status reports: module or standard
    std::string beamArea = "best"; // where a UE may be dropped in a cell: best or contour
    double beamEdgeDb = 10.0;    // --beamArea=best: the lowest gain (dB under the peak) of a UE's beam
    bool geometryOnly = false;   // plan the cells, drop the UEs, print the geometric C/I and stop
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
    std::string sinrTrace;   // <sinrTrace>_dl.csv, _ul.csv: the SINR of every control symbol / PUSCH
    bool admission = true;   // admit a call only if the cell has room for it
    double admissionCcePct = 60; // % of the PDCCH's CCEs the admitted calls may reserve
    double admissionRbPct = 100; // % of the RBs (each way) the admitted calls may reserve
    bool dropCalls = true;       // release a call that misses its delay budget too often
    double dropOnTimePct = 80;   // ... below this share of packets within the PDB
    bool retry = false;          // a UE whose call was released connects and calls again
    double retrySeconds = 3;     // ... this long after the release
    uint32_t maxRetry = 5;       // ... while its tries (the first call too) are fewer than this

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
    cmd.AddValue("beamArea",
                 "--beams>1: where a UE is dropped in its cell: best = where its beam is the "
                 "strongest of all and not more than --beamEdgeDb under its peak, contour = within "
                 "3 dB of the peak",
                 beamArea);
    cmd.AddValue("beamEdgeDb",
                 "--beamArea=best: the edge of the service area, dB under the peak of the cell's "
                 "beam (the outermost ring of beams: always 3 dB). The corners of a cell are "
                 "3 dB down at nadir and 7 to 9 dB down at 30 to 10 deg elevation; the side "
                 "lobes of the pattern, where no beam is near, are 17.6 dB down and more",
                 beamEdgeDb);
    cmd.AddValue("beamFloorDb",
                 "--beams>1: the lowest the beam pattern goes (dB below the peak); default -80 "
                 "(one beam: -30)",
                 beamFloorDb);
    cmd.AddValue("beamHpbwDeg",
                 "3 dB beam width (deg) on both carriers, instead of the aperture's",
                 beamHpbwDeg);
    cmd.AddValue("beamPruneDb",
                 "--beams>1: a beam this far (dB) under the serving beam's at a UE is not "
                 "computed at all",
                 beamPruneDb);
    cmd.AddValue("beamReuse",
                 "--beams>1: frequency reuse of the beams, 1, 3, 4 or 7 (each colour is a "
                 "carrier of --bandwidthMHz)",
                 beamReuse);
    cmd.AddValue("beams",
                 "Beams of each satellite, 1 + 3k(k+1) for k rings: 1, 7, 19, 37, 61, 91, 127 ... "
                 "(TR 38.821 6.1.1.1 layout; those that miss the Earth or point under 5 deg are "
                 "left out): a cell each; --nUes is then per beam",
                 beams);
    cmd.AddValue("beamSpacing",
                 "--beams>1: the spacing of the beam centres, 1 = TR 38.821's (beams cross at "
                 "-3 dB)",
                 beamSpacing);
    cmd.AddValue("bsr",
                 "Buffer status reports of the UEs: module = as the nr module has them (a 5 B BSR in "
                 "every block that carries new data, a request for a grant whenever the UE has "
                 "data); standard = as TS 38.321 5.4.5 and 6.1.3.1: a 2 B short BSR, sent when a "
                 "trigger fires (data arrives in an empty buffer, retxBSR-Timer, "
                 "periodicBSR-Timer; the padding BSR and the trigger for data of a higher "
                 "priority are not modelled). In both, the size a UE reports counts the MAC "
                 "subheaders and the report itself (TS 38.321 6.1.3.1 leaves them out), so that "
                 "the grant it brings has room for them; a long BSR for several logical channel "
                 "groups is not modelled. It sets ns3::NrUeMac::CompactBsr and BsrOnDemand, "
                 "which can also be set one by one on the command line",
                 bsr);
    cmd.AddValue("codecKbps",
                 "Codec bit rate (kbit/s): voice frame = 20 ms of it, in whole bytes",
                 codecKbps);
    cmd.AddValue("csv",
                 "Also write the results as CSV files <csv>_calls.csv etc.; \"\" = none",
                 csv);
    cmd.AddValue("csvPackets",
                 "--csv: also <csv>_packets.csv, a line per delivered voice packet",
                 csvPackets);
    cmd.AddValue("sinrTrace",
                 "Write the SINR (dB) of every control symbol UE receives to <sinrTrace>_dl.csv "
                 "(time, UE, dB) and of every PUSCH the gNB receives to <sinrTrace>_ul.csv "
                 "(time, UE, dB as received, RBs, MCS, block bytes, redundancy version, 1 = not decoded, "
                 "block error probability), for fading and link studies; \"\" = none",
                 sinrTrace);
    cmd.AddValue("dciBits", "--pdcch=1: size of a DCI in bits, without its 24-bit CRC", dciBits);
    cmd.AddValue("direction",
                 "both = two-way calls; dl: far end -> UE only; ul: UE -> far end",
                 direction);
    cmd.AddValue("dlBoostMaxDb",
                 "OFDMA: the most dB an RB in use takes from the idle RBs of its symbol",
                 dlBoostMaxDb);
    cmd.AddValue("dlClosedLoop",
                 "DL closed loop on the boost: 0 off, 1 SNR >= --dlSnrMinDb, 2 MCS >= "
                 "--dlMcsMin, 3 both",
                 dlClosedLoop);
    cmd.AddValue("dlSnrMinDb", "--dlClosedLoop: the SNR a UE should have at least (dB)", dlSnrMinDb);
    cmd.AddValue("dlMcsMin", "--dlClosedLoop: the MCS a UE should have at least", dlMcsMin);
    cmd.AddValue("ulClosedLoop",
                 "UL closed loop: lower the UE's power while its SNR is above --ulSnrHighDb, "
                 "never under --ulSnrLowDb",
                 ulClosedLoop);
    cmd.AddValue("ulSnrHighDb", "--ulClosedLoop: lower the power while the SNR is above (dB)",
                 ulSnrHighDb);
    cmd.AddValue("ulSnrLowDb",
                 "--ulClosedLoop: ... but not so far that the SNR falls under this (dB)",
                 ulSnrLowDb);
    cmd.AddValue("ulPowerMinDbm", "--ulClosedLoop: the least a UE sends (dBm)", ulPowerMinDbm);
    cmd.AddValue("closedLoopMs", "closed loops: decide once per this many ms", closedLoopMs);
    cmd.AddValue("closedLoopStepDb", "closed loops: change the power by this many dB", closedLoopStepDb);
    cmd.AddValue("closedLoopLanding",
                 "closed loops: acting, put the SNR this share of the band in from its far edge "
                 "(0 to 0.5; 0.2)",
                 closedLoopLanding);
    cmd.AddValue("closedLoopAvg",
                 "closed loops: periods the estimate of the SNR averages (1 to 10; 3)",
                 closedLoopAvg);
    cmd.AddValue("dlFrequencyGHz", "Downlink carrier (GHz), 0.6 to 6", dlFrequencyGHz);
    cmd.AddValue("dlPfdLimit",
                 "PFD limit (dBW/m2/MHz): RBs take idle RBs' power up to it; 0 = off",
                 dlPfdLimit);
    cmd.AddValue("dlPfdMarginDb",
                 "--dlPfdLimit: dB over PFD x 4 pi d^2 (shadowing, gases, scint.)",
                 dlPfdMarginDb);
    cmd.AddValue("dlPowerRefDeg",
                 "DL power control: full power at this elevation, less above; 0 = off",
                 dlPowerRefDeg);
    cmd.AddValue("dlRbPowerMinDbm",
                 "--dlPowerRefDeg: RBs in use keep at least this dBm; 0 = no floor",
                 dlRbPowerMinDbm);
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
                 "UE 0 elevation at start (10-90); --fullPass=1: pass from/to it (10)",
                 elevation);
    cmd.AddValue("errorModel",
                 "NrLteMiErrorModel (LTE), NrEesm + Ir|Cc + T1|T2 (NR)",
                 errorModel);
    cmd.AddValue("ntnTdlDelayNs",
                 "--fastFading=ntn-tdl-*: the rms delay spread (ns) the profile's delays are scaled to (TR 38.821 6.1.2: the mean DS of suburban LOS at 30 deg = 10 ns; its median is 1.9 ns)",
                 ntnTdlDelayNs);
    cmd.AddValue("ntnTdlSpeedKmph",
                 "--fastFading=ntn-tdl-*: the UE speed (km/h) whose Doppler its Rayleigh taps have",
                 ntnTdlSpeedKmph);
    cmd.AddValue("fadingHoldMs",
                 "--fastFading=model: each fading stays for this long (ms); 0 = ns-3's",
                 fadingHoldMs);
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
                 "fixed = --fastFadingDb, flat; model = TR 38.901/38.811 channel; ntn-tdl-a, "
                 "-b, -c, -d = the tapped delay line of TR 38.811 Table 6.9.2-1 to -4 at every "
                 "elevation",
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
                 "1 = whole pass over UE 0: --elevation up, over, down (it flies)",
                 fullPass);
    cmd.AddValue("gatewayDbwPerMHz",
                 "Feeder link: power density into the gateway antenna (dBW/MHz)",
                 gatewayDbwPerMHz);
    cmd.AddValue("gatewayDiameterKm",
                 "Gateway: random within this diameter around UE 0 (km); 0 = at UE 0",
                 gatewayDiameterKm);
    cmd.AddValue("gatewayGainDbi", "Feeder link: gateway antenna gain (dBi)", gatewayGainDbi);
    cmd.AddValue("geometryOnly",
                 "Plan the cells and drop the UEs, print each cell's geometric full-load C/I "
                 "(BEAMS) and stop: no simulation, so --nUes can be in the thousands",
                 geometryOnly);
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
    cmd.AddValue("ulPolarizationLossDb",
                 "Uplink only: depolarisation loss (dB), TR 38.821 6.1.1.1: 3 with circular "
                 "satellite reception and polarisation re-use, 0 with dual polarisation",
                 ulPolarizationLossDb);
    cmd.AddValue("maxRetry",
                 "--retry=1: most tries of a UE, its first call included (5: first + 4 retries)",
                 maxRetry);
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
                 "UL slots per block (PUSCH repetition): 1, 2, 3, 4, 7, 8, 12, 16, 20, 24, 28 or "
                 "32 (TS 38.331 numberOfRepetitionsExt-r17); 0 = as needed",
                 puschRepetitions);
    cmd.AddValue("puschRepSinrDb",
                 "--puschRepetitions=0: K brings K x a UE's SINR to this (dB)",
                 puschRepSinrDb);
    cmd.AddValue("puschMinBlockBytes",
                 "--puschRepetitions=0: K goes up until the block of a slot is this big (B); "
                 "0 = the least a UE sends data in (15, with --ns3::NrUeMac::CompactBsr=1 12)",
                 puschMinBlockBytes);
    cmd.AddValue("ulSinrAvgMs",
                 "Uplink link adaptation: time constant (ms) of the mean of a UE's SINR reports; "
                 "0 = the MCS and K follow the SINR of the last reception alone, fading included "
                 "(the nr module's own behaviour). A few coherence times (70 ms for a pedestrian) "
                 "average the fading out. Needs --ulLinkAdaptation=1.",
                 ulSinrAvgMs);
    cmd.AddValue("retry",
                 "1 = a UE whose call was released or blocked connects and calls again (--retrySeconds)",
                 retry);
    cmd.AddValue("retrySeconds",
                 "--retry=1: the UE calls again this long (s) after the release or the refusal",
                 retrySeconds);
    cmd.AddValue("rohc", "Compress the IP/UDP headers in PDCP with ROHC", rohc);
    cmd.AddValue("rohcDynamicWindow",
                 "Size table: packets missing in a row that UO-0 survives",
                 rohcDynamicWindow);
    cmd.AddValue("rohcFallbackAfter",
                 "O-mode: ask for U-mode after n failed in a row; 0 = never",
                 rohcFallbackAfter);
    cmd.AddValue("rohcIrPackets",
                 "Size table: IR packets in a row (start, refresh)",
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
    cmd.AddValue("rohcRepairIrPackets",
                 "Size table: IR packets in a row that answer a STATIC-NACK",
                 rohcRepairIrPackets);
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
    cmd.AddValue("satellites",
                 "Satellites, one behind the other along the meridian (--freezeGeometry=1)",
                 satellites);
    cmd.AddValue("satFeederGtDbPerK",
                 "Feeder link: satellite G/T towards the gateway (dB/K)",
                 satFeederGtDbPerK);
    cmd.AddValue("satGtDbPerK", "Satellite G/T (dB/K), default: the set", satGtDbPerK);
    cmd.AddValue("satSeparationDeg",
                 "--satellites>1: their distance along the orbit (deg of arc); 0 = the clusters "
                 "of beams touch",
                 satSeparationDeg);
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
    cmd.AddValue("ueBufferAgeS",
                 "Satellite UEs drop voice that waited this long (s); 0 = never",
                 ueBufferAgeS);
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
    cmd.AddValue("ulPfMarginal",
                 "--scheduler=ns3::NrMacSchedulerOfdmaPF, uplink, with --ulLinkAdaptation=1: rank "
                 "the UEs by what the next resource block group adds to the block of each (the "
                 "worth of a group, not of a fixed reference grant), and give a UE no group that "
                 "adds nothing. 0 = the nr module's ranking, which hands the whole band to the UE "
                 "with the best rate when the grants are not limited (--ulGrantRbs=0). Attribute "
                 "NrMacSchedulerOfdmaPF::UlMarginalRate",
                 ulPfMarginal);
    cmd.AddValue("ulPowerRefDeg",
                 "UL power control: 23 dBm at this elevation, less above; 0 = off",
                 ulPowerRefDeg);
    cmd.AddValue("ulWideLoadPct",
                 "--ulWideRbs: wide grants while the slot's RBs in use are at most this %",
                 ulWideLoadPct);
    cmd.AddValue("ulWideRbs",
                 "OFDMA: RBs of a wide uplink grant (UE with SINR, slot with room); 0 = none",
                 ulWideRbs);
    cmd.AddValue("ulWideSinrDb",
                 "--ulWideRbs: the SINR on ONE RB (dB) a UE needs for a wide grant",
                 ulWideSinrDb);
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
    const Time durationDefault = duration;
    // Whether --elevation was given, even if it is the default: with --fullPass=1 it is where the
    // pass starts and ends, and without it that is 10 deg
    bool elevationGiven = false;
    for (int k = 1; k < argc; ++k)
    {
        const std::string argument = argv[k];
        elevationGiven = elevationGiven || argument.compare(0, 12, "--elevation=") == 0 ||
                         argument.compare(0, 11, "-elevation=") == 0;
    }
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_UNLESS(bsr == "module" || bsr == "standard", "--bsr must be module or standard");
    if (bsr == "standard")
    {
        Config::SetDefault("ns3::NrUeMac::CompactBsr", BooleanValue(true));
        Config::SetDefault("ns3::NrUeMac::BsrOnDemand", BooleanValue(true));
    }
    // (the size of the BSR is a global of the MAC; the UEs set it when they are made, and the
    // sizes of the blocks worked out before that need it now)
    NrMacShortBsrCe::SetCompact(AttributeDefault<BooleanValue>("ns3::NrUeMac", "CompactBsr"));
    const bool bsrOnDemand = AttributeDefault<BooleanValue>("ns3::NrUeMac", "BsrOnDemand");
    // --fullPass=1 sets the duration itself
    const bool passIgnored = fullPass && duration != durationDefault;
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
    NS_ABORT_MSG_UNLESS(dlClosedLoop <= 3, "--dlClosedLoop must be 0 (off), 1, 2 or 3");
    NS_ABORT_MSG_UNLESS(std::isfinite(dlSnrMinDb) && std::abs(dlSnrMinDb) <= 40.0,
                        "--dlSnrMinDb must be -40 to 40");
    NS_ABORT_MSG_UNLESS(dlMcsMin <= 28, "--dlMcsMin must be 0 to 28");
    NS_ABORT_MSG_IF(dlClosedLoop != 0 && dlBoostMaxDb == 0.0 && dlPfdLimit == 0.0,
                    "--dlClosedLoop moves the boost of an RB, which is off: give --dlBoostMaxDb "
                    "(the most it may be) or --dlPfdLimit");
    NS_ABORT_MSG_IF(dlClosedLoop != 0 && scheduler.find("Tdma") != std::string::npos,
                    "--dlClosedLoop needs an OFDMA scheduler: with TDMA no RB of a symbol is idle, "
                    "there is no power to boost with");
    NS_ABORT_MSG_UNLESS(std::isfinite(ulSnrHighDb) && std::abs(ulSnrHighDb) <= 40.0 &&
                            std::isfinite(ulSnrLowDb) && std::abs(ulSnrLowDb) <= 40.0,
                        "--ulSnrHighDb and --ulSnrLowDb must be -40 to 40");
    NS_ABORT_MSG_IF(ulClosedLoop && ulSnrLowDb > ulSnrHighDb,
                    "--ulSnrLowDb must not be above --ulSnrHighDb");
    NS_ABORT_MSG_UNLESS(std::isfinite(ulPowerMinDbm) && ulPowerMinDbm >= -60.0 &&
                            ulPowerMinDbm <= 23.0,
                        "--ulPowerMinDbm must be -60 to 23 (TS 38.101-1: -40 at 5 MHz)");
    NS_ABORT_MSG_UNLESS(std::isfinite(closedLoopMs) && closedLoopMs >= 100.0 &&
                            closedLoopMs <= 60000.0,
                        "--closedLoopMs must be 100 to 60000");
    NS_ABORT_MSG_UNLESS(std::isfinite(closedLoopLanding) && closedLoopLanding >= 0.0 &&
                            closedLoopLanding <= 0.5,
                        "--closedLoopLanding must be 0 to 0.5");
    NS_ABORT_MSG_UNLESS(std::isfinite(closedLoopAvg) && closedLoopAvg >= 1.0 &&
                            closedLoopAvg <= 10.0,
                        "--closedLoopAvg must be 1 to 10");
    NS_ABORT_MSG_UNLESS(std::isfinite(closedLoopStepDb) && closedLoopStepDb >= 0.1 &&
                            closedLoopStepDb <= 3.0,
                        "--closedLoopStepDb must be 0.1 to 3");
    NS_ABORT_MSG_UNLESS(dlPowerRefDeg == 0.0 || (dlPowerRefDeg >= 10.0 && dlPowerRefDeg <= 90.0),
                        "--dlPowerRefDeg must be 0 (off) or 10 to 90");
    NS_ABORT_MSG_UNLESS(ulPowerRefDeg == 0.0 || (ulPowerRefDeg >= 10.0 && ulPowerRefDeg <= 90.0),
                        "--ulPowerRefDeg must be 0 (off) or 10 to 90");
    NS_ABORT_MSG_UNLESS(std::isfinite(dlPfdLimit) && dlPfdLimit >= -150.0 && dlPfdLimit <= 0.0,
                        "--dlPfdLimit must be -150 to 0 dBW/m2/MHz (0: off)");
    NS_ABORT_MSG_UNLESS(std::isfinite(dlPfdMarginDb) && std::abs(dlPfdMarginDb) <= 30.0,
                        "--dlPfdMarginDb must be -30 to 30");
    NS_ABORT_MSG_UNLESS(std::isfinite(dlRbPowerMinDbm) && dlRbPowerMinDbm >= 0.0 &&
                            dlRbPowerMinDbm <= 60.0,
                        "--dlRbPowerMinDbm must be 0 (off) to 60");
    NS_ABORT_MSG_IF(dlRbPowerMinDbm > 0.0 && dlPowerRefDeg == 0.0,
                    "--dlRbPowerMinDbm is the floor of what --dlPowerRefDeg takes off the power "
                    "of an RB: add --dlPowerRefDeg");
    NS_ABORT_MSG_UNLESS(ulWideRbs <= 100, "--ulWideRbs must be 0 (off) to 100");
    NS_ABORT_MSG_UNLESS(std::isfinite(ulWideSinrDb) && std::abs(ulWideSinrDb) <= 30.0,
                        "--ulWideSinrDb must be -30 to 30");
    NS_ABORT_MSG_UNLESS(ulWideLoadPct >= 0.0 && ulWideLoadPct <= 100.0,
                        "--ulWideLoadPct must be 0 to 100");
    const bool tdma = scheduler.find("Tdma") != std::string::npos;
    NS_ABORT_MSG_IF(ulWideRbs > 0 && (tdma || ulGrantRbs == 0 || ulWideRbs <= ulGrantRbs),
                    "--ulWideRbs needs an OFDMA scheduler and --ulGrantRbs above 0 and below it "
                    "(--ulGrantRbs=1 --ulWideRbs=2)");
    NS_ABORT_MSG_IF(ulWideRbs > 0 && !ulLinkAdaptation,
                    "--ulWideRbs needs --ulLinkAdaptation=1: the grant's MCS is for its RBs");
    NS_ABORT_MSG_IF(tdma && ulGrantRbs > 0,
                    "--ulGrantRbs needs an OFDMA scheduler: a TDMA scheduler gives a UE whole "
                    "symbols of the band. Add --ulGrantRbs=0 --puschRepetitions=1");
    NS_ABORT_MSG_UNLESS(pdcchSymbols <= 3, "--pdcchSymbols must be 0, 1, 2 or 3");
    NS_ABORT_MSG_UNLESS(puschRepetitions == 0 ||
                            NrMacSchedulerUeInfo::IsPuschRepetitionValue(puschRepetitions),
                        "--puschRepetitions must be 0 (as many as needed) or a number of "
                        "repetitions of TS 38.331 (numberOfRepetitionsExt-r17): 1, 2, 3, 4, 7, 8, "
                        "12, 16, 20, 24, 28 or 32");
    NS_ABORT_MSG_UNLESS(admissionRbPct > 0.0 && admissionRbPct <= 1000.0 && admissionCcePct > 0.0 &&
                            admissionCcePct <= 1000.0,
                        "--admissionRbPct and --admissionCcePct must be above 0, at most 1000");
    NS_ABORT_MSG_UNLESS(dropOnTimePct >= 0.0 && dropOnTimePct <= 100.0,
                        "--dropOnTimePct must be 0 to 100");
    NS_ABORT_MSG_UNLESS(std::isfinite(retrySeconds) && retrySeconds >= 0.0 &&
                            retrySeconds <= 3600.0,
                        "--retrySeconds must be 0 to 3600");
    NS_ABORT_MSG_UNLESS(maxRetry >= 1 && maxRetry <= 1000, "--maxRetry must be 1 to 1000");
    // Admission and release end a call's voice source: the RTP source can, the NGMN generator not
    const bool callControlOff = !rtp && (admission || dropCalls);
    const bool retryOff = !rtp && retry; // (--retry needs a release)
    if (!rtp)
    {
        admission = false;
        dropCalls = false;
        retry = false; // (nothing is released, so there is nothing to retry)
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
    NS_ABORT_MSG_UNLESS(fastFading == "fixed" || fastFading == "model" ||
                            fastFading == "ntn-tdl-a" || fastFading == "ntn-tdl-b" ||
                            fastFading == "ntn-tdl-c" || fastFading == "ntn-tdl-d",
                        "--fastFading must be fixed, model, ntn-tdl-a, ntn-tdl-b, ntn-tdl-c or "
                        "ntn-tdl-d");
    NS_ABORT_MSG_UNLESS(std::isfinite(shadowingDb) && std::isfinite(fastFadingDb) &&
                            std::isfinite(linkMarginDb) && std::isfinite(ulPolarizationLossDb),
                        "--shadowingDb, --fastFadingDb, --linkMarginDb and --ulPolarizationLossDb "
                        "must be numbers");
    NS_ABORT_MSG_UNLESS(std::isfinite(ulSinrAvgMs) && ulSinrAvgMs >= 0.0 && ulSinrAvgMs <= 10000.0,
                        "--ulSinrAvgMs must be 0 to 10000");
    NS_ABORT_MSG_IF(ulSinrAvgMs > 0.0 && !ulLinkAdaptation,
                    "--ulSinrAvgMs needs --ulLinkAdaptation=1: it smooths the SINR that the "
                    "grant's MCS and K are computed from");
    // TR 38.811 6.6.6.1.4: the model holds within about 20 deg of the magnetic equator, the
    // scintillation can be neglected up to 60 deg, and above that the model does not apply
    NS_ABORT_MSG_UNLESS(atmosphere != "location" || std::abs(latitudeDeg) <= 60.0,
                        "--latitudeDeg must be -60 to 60: TR 38.811 has no scintillation model "
                        "for higher latitudes");
    const bool fixedShadowing = (shadowing == "fixed");
    const bool atmosphereTable = (atmosphere == "table");
    const bool equatorial = !atmosphereTable && std::abs(latitudeDeg) < 20.0;
    const bool fixedFastFading = (fastFading == "fixed");
    // --fastFading=ntn-tdl-*: the profile's letter (0: not that), its Doppler
    const char tdlProfile = fastFading.rfind("ntn-tdl-", 0) == 0 ? fastFading.back() : 0;
    NS_ABORT_MSG_UNLESS(std::isfinite(ntnTdlDelayNs) && ntnTdlDelayNs >= 0.0 &&
                            ntnTdlDelayNs <= 10000.0 && std::isfinite(ntnTdlSpeedKmph) &&
                            ntnTdlSpeedKmph >= 0.0 && ntnTdlSpeedKmph <= 1000.0,
                        "--ntnTdlDelayNs must be 0 to 10000 and --ntnTdlSpeedKmph 0 to 1000");
    NS_ABORT_MSG_UNLESS(std::isfinite(fadingHoldMs) && fadingHoldMs >= 0.0 &&
                            fadingHoldMs <= 10000.0,
                        "--fadingHoldMs must be 0 to 10000");
    NS_ABORT_MSG_UNLESS(fadingHoldMs == 0.0 || fadingHoldMs >= 1.0,
                        "--fadingHoldMs must be 0 (ns-3's own) or at least 1 ms");
    NS_ABORT_MSG_IF(fadingHoldMs > 0.0 && (fixedFastFading || tdlProfile != 0),
                    "--fadingHoldMs needs --fastFading=model: with fixed there is nothing to hold, "
                    "and an ntn-tdl fading has its own time structure (--ntnTdlSpeedKmph)");
    NS_ABORT_MSG_UNLESS(std::isfinite(ueBufferAgeS) && ueBufferAgeS >= 0.0 && ueBufferAgeS <= 600.0,
                        "--ueBufferAgeS must be 0 to 600");
    NS_ABORT_MSG_UNLESS(ueBufferAgeS == 0.0 || ueBufferAgeS >= 0.001,
                        "--ueBufferAgeS must be 0 (never) or at least 0.001 s");
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
        // has risen to --elevation (10 deg unless given) and ends when it has sunk to it again.
        // --freezeGeometry and --duration are not used; the duration is set below, from the orbit.
        freezeGeometry = false;
        if (!elevationGiven)
        {
            elevation = 10.0;
        }
    }
    NS_ABORT_MSG_UNLESS(elevation >= 10.0 && elevation <= 90.0,
                        "--elevation must be 10 to 90 (the 3GPP NTN channel stops at 10)");
    NS_ABORT_MSG_IF(fullPass && elevation >= 90.0,
                    "--fullPass=1: --elevation must be below 90, the pass is from it to it again");
    // The elevation at which the pass is over for UE 0, and below which the other UEs have no call
    // (the whole pass: where it starts; otherwise 10 deg)
    const double setElevationDeg = fullPass ? elevation : 10.0;
    NS_ABORT_MSG_UNLESS(nUes >= 1, "--nUes must be at least 1");
    // Several beams or satellites: a cell for each beam of each satellite, --nUes UEs in each
    const int beamTiers = HexTiersOf(beams);
    NS_ABORT_MSG_IF(beamTiers < 0, "--beams must be 1 + 3 k (k + 1) for k rings: 1, 7, 19, 37, 61, 91, 127 ...");
    NS_ABORT_MSG_UNLESS(beamReuse == 1 || beamReuse == 3 || beamReuse == 4 || beamReuse == 7,
                        "--beamReuse must be 1, 3, 4 or 7");
    NS_ABORT_MSG_UNLESS(satellites >= 1 && satellites <= 9, "--satellites must be 1 to 9");
    NS_ABORT_MSG_IF(satellites > 1 && !freezeGeometry,
                    "--satellites>1 needs --freezeGeometry=1: the satellites stand still. (With "
                    "flying satellites the UEs would have to be handed over from one satellite to "
                    "the next; the nr module has the handover, this example does not set it up.)");
    NS_ABORT_MSG_UNLESS(beamArea == "best" || beamArea == "contour",
                        "--beamArea must be best or contour");
    NS_ABORT_MSG_UNLESS(std::isfinite(beamSpacing) && beamSpacing >= 0.2 && beamSpacing <= 3.0,
                        "--beamSpacing must be 0.2 to 3");
    NS_ABORT_MSG_UNLESS(std::isfinite(beamEdgeDb) && beamEdgeDb >= 3.0 && beamEdgeDb <= 16.0,
                        "--beamEdgeDb must be 3 to 16 (the first side lobe of the pattern is "
                        "17.6 dB down)");
    NS_ABORT_MSG_UNLESS(std::isfinite(beamPruneDb) && beamPruneDb >= 10.0 && beamPruneDb <= 120.0,
                        "--beamPruneDb must be 10 to 120");
    NS_ABORT_MSG_UNLESS(std::isnan(beamFloorDb) || (beamFloorDb <= -10.0 && beamFloorDb >= -120.0),
                        "--beamFloorDb must be -120 to -10");
    NS_ABORT_MSG_UNLESS(std::isfinite(satSeparationDeg) && satSeparationDeg >= 0.0 &&
                            satSeparationDeg <= 30.0,
                        "--satSeparationDeg must be 0 (auto) to 30");
    const bool multiCell = beams > 1 || satellites > 1;
    const uint32_t nUesPerCell = nUes;
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
    NS_ABORT_MSG_IF(multiCell && farUe,
                    "--beams>1 and --satellites>1 are not set up for --farEnd=ue: the far end is "
                    "one terrestrial UE per call, for the cells of UE 0's beam only");
    const std::string transportProtocol = "ns3::UdpSocketFactory";

    // One VAF per speaker, drawn from a Gaussian truncated to the generator's range [0, 0.99]
    // (values outside are drawn again). Change --RngRun to get a different set. The two speakers
    // of a call talk independently of each other; each direction has its own random stream, the
    // first one in use stream 500 (so a one-way call has the VAF it had before two-way calls).
    // (a run with several cells draws for every UE it could have)
    const uint32_t maxUes = nUes * beams * satellites;
    std::array<std::vector<double>, 2> vaf;
    for (const Dir dir : {DOWN, UP})
    {
        vaf[dir].assign(maxUes, NAN);
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
        for (uint32_t i = 0; i < maxUes; ++i)
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
            TypeId::LookupByName("ns3::NrPdcp")
                .LookupAttributeByName("RohcTableStaticWindow", &info) &&
            TypeId::LookupByName("ns3::NrPdcp")
                .LookupAttributeByName("RohcFallbackFailures", &info) &&
            TypeId::LookupByName("ns3::NrPdcp")
                .LookupAttributeByName("RohcTableRepairIrPackets", &info);
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
    NS_ABORT_MSG_IF(rohcRepairIrPackets < 1 || rohcRepairIrPackets > 255,
                    "--rohcRepairIrPackets must be 1 to 255");
    NS_ABORT_MSG_IF(rohcFallbackAfter > 0 && !(rohc && rohcModel == "table" && rohcMode == "O"),
                    "--rohcFallbackAfter needs --rohcModel=table and --rohcMode=O");
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
        Config::SetDefault("ns3::NrPdcp::RohcTableRepairIrPackets",
                           UintegerValue(rohcRepairIrPackets));
        Config::SetDefault("ns3::NrPdcp::RohcFallbackFailures", UintegerValue(rohcFallbackAfter));
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
    // --ueBufferAgeS: the RLC of a satellite UE drops the voice that has waited this long
    Config::SetDefault("ns3::NrRlcUm::UeTxStaleMs",
                       UintegerValue(static_cast<uint32_t>(std::lround(ueBufferAgeS * 1000.0))));
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
    // (Msg3 carries the C-RNTI MAC CE, 5.1.4 and 6.1.3.2; the contention is resolved on it, 5.1.5);
    // this nr module gives it a new RNTI instead, and its downlink stops
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
    Vector orbitStart; // flying: where the orbit model is put (see below), for the satellites made later
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
        downAt = overheadAt + Seconds(secondsFromOverhead(setElevationDeg));
        if (fullPass)
        {
            duration = downAt - passStart; // UE 0's call: from --elevation to it again
        }
        // The model takes the longitude of the orbit plane and the angle along the orbit from its
        // northernmost point, here the North Pole; the satellite is 90 deg east of the plane's
        // longitude. At t = 0 it is overheadAt seconds before the beam centre.
        const double toOverhead = overheadAt.GetSeconds();
        orbitStart = Vector(omegaEarth * toOverhead * 180.0 / M_PI - 90.0,
                            90.0 - omega * toOverhead * 180.0 / M_PI,
                            0.0);
        orbiting->SetPosition(orbitStart);
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
    // The lowest elevation at which a UE that was dropped at random may see the satellite during
    // the calls. UE 0 keeps 10 deg, where the 3GPP NTN tables start; the tables
    // have one row per 10 deg and the models take the 10 deg row for everything below it, so
    // 5 deg is half a row further down. With 10 deg here, a pass that uses all the time UE 0 has
    // (from 10 deg up and down to 10 deg again) would leave only the places exactly east and
    // west of UE 0: every other place is below 10 deg at the start or at the end.
    const double minPlaceElevationDeg = 5.0;

    // --- The cells: a gNB for each beam of each satellite ------------------------------------------
    // One beam of one satellite (--beams=1 --satellites=1): the run of this example as it always
    // was. With more, the layout of TR 38.821 6.1.1.1 (see PlanBeams()): the beams of a satellite
    // are the points of a hexagonal grid of its UV plane, the centre beam being the beam on
    // UE 0. A satellite that stands still has its beams where they are; a satellite that flies
    // has beams fixed on the ground, those of the moment it is over the centre (a real one
    // steers its beams to keep them there). The satellites are along the meridian of the centre,
    // the first one with the centre beam on UE 0 and the others to the north and the south of it
    // (--satSeparationDeg apart along the orbit).
    auto makeStaticSat = [&](double latDeg) {
        Ptr<FixedMobilityModel> frozen = CreateObject<FixedMobilityModel>();
        frozen->SetGeographicPosition(Vector(latDeg, 0.0, altitudeM));
        const double lat = latDeg * M_PI / 180.0;
        frozen->SetReportedVelocity(
            Vector(orbitSpeedMps * std::sin(lat), 0.0, -orbitSpeedMps * std::cos(lat)));
        return Ptr<GeocentricConstantPositionMobilityModel>(frozen);
    };
    auto makeFlyingSat = [&]() {
        Ptr<OrbitingMobilityModel> orbiting = CreateObject<OrbitingMobilityModel>();
        orbiting->SetAttribute("Altitude", DoubleValue(altitudeM / 1000.0)); // km
        orbiting->SetAttribute("Inclination", DoubleValue(90.0));
        orbiting->SetAttribute("Resolution", TimeValue(Seconds(1)));
        orbiting->SetPosition(orbitStart);
        return Ptr<GeocentricConstantPositionMobilityModel>(orbiting);
    };
    // the beam spacing in the UV plane, and the centre beam's direction cosine towards the track:
    // sin of its angle from nadir (the satellite that stands still sees UE 0 at --elevation)
    const double absUv = beamSpacing * std::sqrt(3.0) * g_halfPowerX / g_scene.beamKa[0];
    const double centreNadirRad =
        freezeGeometry ? std::asin(earthRadiusM * std::cos(elevationRad) / orbitRadiusM) : 0.0;
    auto satOffsetIndex = [](uint32_t s) { // 0, +1, -1, +2, -2 ... (+: to the north)
        return s % 2 == 1 ? static_cast<int>((s + 1) / 2) : -static_cast<int>(s / 2);
    };
    std::vector<OrbitPredictor> satOrbits{orbit};
    std::vector<double> satLatsDeg{satLatDeg};
    std::vector<BeamPlan> beamPlan;
    double beamPitchDeg = 0.0;  // along the track, at the centre beam (deg of arc)
    double satSepUsedDeg = 0.0; // the distance between satellites (deg of arc)
    {
        const Vector firstSat = freezeGeometry ? orbit.pos0 : Vector(orbitRadiusM, 0.0, 0.0);
        const std::vector<BeamPlan> first = PlanBeams({firstSat},
                                                      beamTiers,
                                                      beamReuse,
                                                      std::sin(centreNadirRad),
                                                      absUv,
                                                      {0},
                                                      earthRadiusM,
                                                      minPlaceElevationDeg);
        NS_ABORT_MSG_UNLESS((first.front().centre - centrePosition).GetLength() < 1.0,
                            "the centre beam does not point at UE 0");
        // the spacing of the beams along the track: the arc from the beam before the centre one
        // to the one after it, in the middle row of the layout, per beam
        double arcRad = 0.0;
        int firstQ = 0;
        int lastQ = 0;
        const BeamPlan* before = &first.front();
        const BeamPlan* after = &first.front();
        for (const BeamPlan& b : first)
        {
            if (b.r == 0 && b.q < firstQ)
            {
                firstQ = b.q;
                before = &b;
            }
            if (b.r == 0 && b.q > lastQ)
            {
                lastQ = b.q;
                after = &b;
            }
        }
        if (lastQ > firstQ)
        {
            const double cosArc = (before->centre * after->centre) /
                                  (before->centre.GetLength() * after->centre.GetLength());
            arcRad = std::acos(std::clamp(cosArc, -1.0, 1.0)) / (lastQ - firstQ);
        }
        else
        {
            arcRad = absUv * altitudeM / earthRadiusM; // a single beam: its spacing at nadir
        }
        beamPitchDeg = arcRad * 180.0 / M_PI;
        satSepUsedDeg = satSeparationDeg > 0.0 ? satSeparationDeg
                                               : (2 * beamTiers + 1) * beamPitchDeg;
        std::vector<Vector> refSats{firstSat};
        std::vector<int> shifts{0};
        for (uint32_t s = 1; s < satellites; ++s)
        {
            const double latDeg = satLatDeg + satOffsetIndex(s) * satSepUsedDeg;
            satLatsDeg.push_back(latDeg);
            OrbitPredictor other;
            other.pos0 = makeStaticSat(latDeg)->GetPosition();
            satOrbits.push_back(other);
            refSats.push_back(other.pos0);
            // (the colours of its beams go on from the first satellite's)
            shifts.push_back(-static_cast<int>(
                std::lround(satOffsetIndex(s) * satSepUsedDeg / beamPitchDeg)));
        }
        beamPlan = PlanBeams(refSats,
                             beamTiers,
                             beamReuse,
                             std::sin(centreNadirRad),
                             absUv,
                             shifts,
                             earthRadiusM,
                             minPlaceElevationDeg);
    }
    const uint32_t nCells = beamPlan.size();
    nUes = nUesPerCell * nCells;
    NodeContainer cellNodes;
    std::vector<Ptr<GeocentricConstantPositionMobilityModel>> cellMobility(nCells);
    for (uint32_t c = 0; c < nCells; ++c)
    {
        if (c == 0)
        {
            cellNodes.Add(satellite);
            cellMobility[c] = satMobility;
            continue;
        }
        Ptr<Node> node = CreateObject<Node>();
        cellMobility[c] = freezeGeometry ? makeStaticSat(satLatsDeg[beamPlan[c].sat])
                                         : makeFlyingSat();
        node->AggregateObject(cellMobility[c]);
        cellNodes.Add(node);
    }
    std::set<uint32_t> cellNodeIds;
    for (uint32_t c = 0; c < nCells; ++c)
    {
        cellNodeIds.insert(cellNodes.Get(c)->GetId());
    }
    // the cell of UE i: --nUes UEs in each, cell by cell
    std::vector<uint32_t> ueCell(nUes);
    for (uint32_t i = 0; i < nUes; ++i)
    {
        ueCell[i] = i / nUesPerCell;
    }
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

    // (`turn`: around the centre of another cell: the rotation that takes the beam centre of the
    // first cell to it; null for the first)
    using Rotation = std::array<Vector, 3>;
    auto turned = [](const Rotation* turn, const Vector& p) {
        return turn ? Vector((*turn)[0] * p, (*turn)[1] * p, (*turn)[2] * p) : p;
    };
    // how far north and east of the first beam centre a place is (km, on a flat map)
    auto mapKm = [&](const Vector& p) {
        const double lat = std::asin(p.z / earthRadiusM);
        const double lon = std::atan2(p.y, p.x);
        return std::make_pair(earthRadiusM * 1e-3 * lat, earthRadiusM * 1e-3 * lon * std::cos(lat));
    };
    auto drawPlace = [&](Ptr<UniformRandomVariable> rng,
                         double capArc,
                         const std::function<bool(const Vector&)>& fits,
                         const std::string& what,
                         const Rotation* turn = nullptr) {
        Place place;
        double arc = 0.0;
        double bearing = 0.0;
        uint32_t tries = 0;
        do
        {
            NS_ABORT_MSG_IF(++tries > 1000000, "no place found for " << what);
            arc = std::acos(1.0 - rng->GetValue() * (1.0 - std::cos(capArc)));
            bearing = 2.0 * M_PI * rng->GetValue();
            place.ecef = turned(turn, groundPoint(arc, bearing));
        } while (!fits(place.ecef));
        place.northKm = earthRadiusM * 1e-3 * arc * std::cos(bearing);
        place.eastKm = earthRadiusM * 1e-3 * arc * std::sin(bearing);
        if (turn)
        {
            std::tie(place.northKm, place.eastKm) = mapKm(place.ecef);
        }
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
    NS_ABORT_MSG_IF(MilliSeconds((nUesPerCell - 1) * startOffsetMs) >= callsEnd - callsStart,
                    "Call " << nUesPerCell - 1
                            << " would start when UE 0's call is over: (nUes - 1) x "
                               "--startOffsetMs must be less than UE 0's call lasts ("
                            << (callsEnd - callsStart).GetSeconds() << " s)");
    // Where the satellite is while the calls are on
    const std::vector<Vector> satDuringCalls = satBetween(callsStart, callsEnd);
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

    // --- The ground of each cell (several beams or satellites) ---------------------------------
    // A UE is in the cell of the beam it hears best, and stays there (a UE that is handed over
    // from one beam to the next is not modelled). Its place is drawn from the ground where the
    // cell's beam is the strongest one of all the beams, and within its 3 dB contour, at every
    // moment of the calls: the flying satellite's beams are fixed on the ground, but the
    // pattern towards a place changes as the satellite flies, and the best beam of a place near
    // the edge of two beams changes with it.
    const double floorDb = multiCell ? (std::isnan(beamFloorDb) ? -80.0 : beamFloorDb) : -30.0;
    std::vector<std::vector<Vector>> satSamples(satOrbits.size()); // where each satellite is
    if (freezeGeometry)
    {
        for (std::size_t s = 0; s < satOrbits.size(); ++s)
        {
            satSamples[s].push_back(satOrbits[s].pos0);
        }
    }
    else
    {
        // (a sample every second or so, and the last: 50 at the most)
        const std::size_t stride = std::max<std::size_t>(1, satDuringCalls.size() / 50);
        for (std::size_t k = 0; k < satDuringCalls.size(); k += stride)
        {
            satSamples[0].push_back(satDuringCalls[k]);
        }
        satSamples[0].push_back(satDuringCalls.back());
    }
    // the rotation that takes the first beam centre to the centre of cell c
    std::vector<Rotation> cellTurn(nCells);
    for (uint32_t c = 0; c < nCells; ++c)
    {
        const Vector to = beamPlan[c].centre;
        const double len = to.GetLength();
        const Vector axis(0.0, -to.z / len, to.y / len); // x cross to, |axis| = sin(angle)
        const double cosAngle = to.x / len;
        // R = I + [axis] + [axis]^2 / (1 + cos(angle))
        const double ax = axis.x, ay = axis.y, az = axis.z;
        const double k = 1.0 / (1.0 + cosAngle);
        cellTurn[c] = {Vector(1.0 + k * (-az * az - ay * ay), -az + k * (ax * ay), ay + k * (ax * az)),
                       Vector(az + k * (ax * ay), 1.0 + k * (-az * az - ax * ax), -ax + k * (ay * az)),
                       Vector(-ay + k * (ax * az), ax + k * (ay * az), 1.0 + k * (-ay * ay - ax * ax))};
    }
    // where cell c can have a UE: its beam is the strongest (withOthers), with the satellite 5 deg
    // or more above the place, at every sampled moment, and its gain is at most --beamEdgeDb under
    // the peak: the service area ends there. (Without that edge the cell of a beam would take in
    // the places far outside the layout where its side lobe happens to be the strongest of all:
    // a 19-beam layout with reuse 3 had UEs of the inner cells there, at a C/I under 0 dB.) For
    // --beamArea=contour, and in the outermost ring (whose cells have nothing beyond them), the
    // edge is the 3 dB contour of the beam.
    const bool areaContour = beamArea == "contour";
    auto inCellArea = [&](uint32_t c, bool withOthers) {
        return [&, c, withOthers](const Vector& place) {
            const bool contourOnly = areaContour || !withOthers || beamPlan[c].tier == beamTiers;
            const uint32_t mySat = beamPlan[c].sat;
            for (std::size_t k = 0; k < satSamples[mySat].size(); ++k)
            {
                const Vector& sat = satSamples[mySat][k];
                if (ElevationDeg(sat, place) < minPlaceElevationDeg)
                {
                    return false;
                }
                const double ownDb =
                    BeamTaperDb(sat, beamPlan[c].centre, place, g_scene.beamKa[0], floorDb);
                if (ownDb < (contourOnly ? -3.0103 : -beamEdgeDb))
                {
                    return false;
                }
                for (uint32_t other = 0; withOthers && other < nCells; ++other)
                {
                    const auto& samples = satSamples[beamPlan[other].sat];
                    const Vector& otherSat = samples[std::min(k, samples.size() - 1)];
                    if (other != c && BeamTaperDb(otherSat,
                                                  beamPlan[other].centre,
                                                  place,
                                                  g_scene.beamKa[0],
                                                  floorDb) > ownDb)
                    {
                        return false;
                    }
                }
            }
            return true;
        };
    };
    // how far from its centre the 3 dB contour of cell c reaches, in 72 directions (rad of arc)
    auto cellMaxArc = [&](uint32_t c) {
        const auto inside = inCellArea(c, !areaContour);
        double maxArc = 0.0;
        for (uint32_t k = 0; k < 72; ++k)
        {
            const double bearing = 2.0 * M_PI * k / 72.0;
            double in = 0.0;
            double out = 2.0 * std::acos(earthRadiusM / orbitRadiusM);
            for (int step = 0; step < 30; ++step)
            {
                const double mid = 0.5 * (in + out);
                (inside(turned(&cellTurn[c], groundPoint(mid, bearing))) ? in : out) = mid;
            }
            maxArc = std::max(maxArc, in);
        }
        return maxArc;
    };

    NodeContainer ueNodes;
    ueNodes.Create(geometryOnly ? 0 : nUes);
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
    std::vector<double> cellCapArc(nCells, 1.01 * areaMaxArc);
    for (uint32_t c = 0; multiCell && c < nCells; ++c)
    {
        cellCapArc[c] = 1.08 * cellMaxArc(c);
    }
    for (uint32_t i = 0; i < nUes; ++i)
    {
        ueMobility[i] = CreateObject<FixedMobilityModel>();
        const uint32_t cell = ueCell[i];
        const uint32_t k = i % nUesPerCell; // its place in its cell
        if (cell == 0 && (!randomLayout || k == 0))
        {
            // in a line, or UE 0: the reference user on the beam centre (i = 0)
            const double eastKm = randomLayout ? 0.0 : i * spacingKm;
            ueMobility[i]->SetGeographicPosition(Vector(0.0, eastKm / kmPerDegree, 0.0));
            ueEastKm[i] = eastKm;
        }
        else if (!randomLayout)
        {
            // in a line east of the centre of its beam
            const std::string what = "UE " + std::to_string(i);
            Place place;
            place.ecef = turned(&cellTurn[cell], groundPoint(k * spacingKm * 1e3 / earthRadiusM, M_PI / 2));
            std::tie(place.northKm, place.eastKm) = mapKm(place.ecef);
            putThere(ueMobility[i], place, what);
            ueNorthKm[i] = place.northKm;
            ueEastKm[i] = place.eastKm;
        }
        else
        {
            // Over the cap that holds the beam area, until the place is in the beam area
            const std::string what = "UE " + std::to_string(i);
            const Place place =
                multiCell ? drawPlace(drop,
                                      cellCapArc[cell],
                                      inCellArea(cell, true),
                                      what,
                                      &cellTurn[cell])
                          : drawPlace(drop, 1.01 * areaMaxArc, inBeamArea, what);
            putThere(ueMobility[i], place, what);
            ueNorthKm[i] = place.northKm;
            ueEastKm[i] = place.eastKm;
        }
        if (!geometryOnly)
        {
            ueNodes.Get(i)->AggregateObject(ueMobility[i]);
        }
    }

    if (geometryOnly)
    {
        // The geometry of the cells and the interference at full load, nothing is simulated
        const Time tMid = callsStart + (callsEnd - callsStart) / 2;
        std::vector<CiBeam> ciBeams(nCells);
        for (uint32_t c = 0; c < nCells; ++c)
        {
            ciBeams[c] = {freezeGeometry ? satOrbits[beamPlan[c].sat].pos0 : orbit.At(tMid),
                          c == 0 ? centrePosition : beamPlan[c].centre,
                          beamPlan[c].colour};
        }
        Frame("BEAMS (--geometryOnly=1)",
              std::to_string(nCells) + " cells: " + std::to_string(beams) + " beam" +
                  (beams == 1 ? "" : "s") + " x " + std::to_string(satellites) + " satellite" +
                  (satellites == 1 ? "" : "s") + ", frequency reuse " + std::to_string(beamReuse) +
                  ", " + std::to_string(nUesPerCell) + " UEs dropped in each");
        Setting("beam",
                Num(BeamWidthDegOfKa(g_scene.beamKa[0]), 4) + " deg wide at 3 dB on the downlink; "
                    "spacing " + Num(beamSpacing, 2) + " x ABS = " +
                    Num(absUv, 4) + " (UV), " + Num(beamPitchDeg * beamSpacing, 2) +
                    " deg of arc on the ground along the track");
        Setting("pattern", "TR 38.811 6.4.1, floor " + Num(floorDb, 0) + " dB; satellite at " +
                               Num(altitudeKm, 0) + " km, " + Num(elevation, 1) +
                               " deg over UE 0 under the centre beam");
        std::cout << "  cell sat beam tier col    N[km]    E[km] elev[deg]   UEs"
                     "  C/I[dB]: mean  median     5%    95%    min\n";
        std::vector<double> allCi;
        for (uint32_t c = 0; c < nCells; ++c)
        {
            std::vector<double> ci;
            for (uint32_t i = 0; i < nUes; ++i)
            {
                if (ueCell[i] == c)
                {
                    const double v = FullLoadCiDb(ciBeams,
                                                  c,
                                                  ueMobility[i]->GetPosition(),
                                                  g_scene.beamKa[0],
                                                  floorDb);
                    if (std::isfinite(v))
                    {
                        ci.push_back(v);
                        allCi.push_back(v);
                    }
                }
            }
            std::sort(ci.begin(), ci.end());
            auto at = [&](double q) { return ci[std::min(ci.size() - 1, static_cast<std::size_t>(q * ci.size()))]; };
            const auto [north, east] = mapKm(ciBeams[c].centre);
            std::cout << std::fixed << std::setprecision(1) << std::setw(6) << c << std::setw(4)
                      << beamPlan[c].sat << std::setw(5) << beamPlan[c].beam << std::setw(5)
                      << beamPlan[c].tier << std::setw(4) << beamPlan[c].colour << std::setw(9)
                      << north << std::setw(9) << east << std::setw(10)
                      << ElevationDeg(ciBeams[c].sat, ciBeams[c].centre) << std::setw(6)
                      << nUesPerCell;
            if (ci.empty())
            {
                std::cout << "   no other beam of the colour";
            }
            else
            {
                double sum = 0.0;
                for (const double v : ci)
                {
                    sum += v;
                }
                std::cout << std::setprecision(2) << std::setw(18) << sum / ci.size()
                          << std::setw(8) << at(0.5) << std::setw(7) << at(0.05) << std::setw(7)
                          << at(0.95) << std::setw(7) << ci.front();
            }
            std::cout << "\n";
        }
        if (!allCi.empty())
        {
            std::sort(allCi.begin(), allCi.end());
            double sum = 0.0;
            for (const double v : allCi)
            {
                sum += v;
            }
            auto at = [&](double q) { return allCi[std::min(allCi.size() - 1, static_cast<std::size_t>(q * allCi.size()))]; };
            std::cout << std::setprecision(2) << "   all" << std::setw(49) << "" << std::setw(18)
                      << sum / allCi.size() << std::setw(8) << at(0.5) << std::setw(7) << at(0.05)
                      << std::setw(7) << at(0.95) << std::setw(7) << allCi.front() << "\n";
        }
        std::cout << "C/I: at full load, every other beam of the colour on every RB at full power, the beam pattern and free "
                     "space only, for the UEs dropped in the cell\n(--beamArea). Cell 0, the centre beam, "
                     "is the cell the calibration of TR 38.821 reports.\n";
        return 0;
    }

    // When each call starts and ends. UE 0's call runs from callsStart to callsEnd; call i
    // starts i x startOffsetMs after it and ends with it.
    // When the satellite flies, a call is also on only while the satellite is 10 deg (--fullPass=1:
    // --elevation) or more above its own UE: it starts when the satellite has risen to that above
    // that UE, if that is later, and ends when the satellite has sunk to it above it again, if
    // that is earlier (stretchAbove() over UE 0's call). A UE whose stretch does not reach into its
    // call has no call. (UE 0 itself: the pass was laid out for its call.) When the satellite
    // stands still nothing rises or sinks, and the rule is not applied.
    const double minCallElevationDeg = setElevationDeg;
    const std::string minCallElevationText =
        Num(minCallElevationDeg, std::fmod(minCallElevationDeg, 1.0) == 0.0 ? 0 : 1);
    std::vector<Time> ueCallStart(nUes);
    std::vector<Time> ueCallEnd(nUes);
    std::vector<bool> ueHasCall(nUes, true);
    for (uint32_t i = 0; i < nUes; ++i)
    {
        ueCallStart[i] = callsStart + MilliSeconds((i % nUesPerCell) * startOffsetMs); // not phase-aligned
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
    // One pair of carriers (a downlink and an uplink channel) for each frequency colour of the
    // beams (--beamReuse): a colour is the carrier above the one before it, --bandwidthMHz wide.
    // The beams of one colour share a channel and so interfere; the beams of two colours do
    // not meet. With one beam, or reuse 1, there is one colour: the carriers of the example.
    const uint32_t colours = multiCell ? beamReuse : 1;
    NS_ABORT_MSG_UNLESS(colours * bandwidthHz <= std::abs(dlFrequencyHz - ulFrequencyHz),
                        "--beamReuse=" << beamReuse << " needs " << colours * bandwidthMHz
                                       << " MHz of each of the downlink and uplink bands, and "
                                          "the two carriers are only "
                                       << std::abs(dlFrequencyHz - ulFrequencyHz) / 1e6
                                       << " MHz apart: use a smaller --bandwidthMHz or another "
                                          "reuse");
    std::vector<double> dlColourHz(colours);
    std::vector<double> ulColourHz(colours);
    std::vector<NtnCarrier> dlCarriers;
    std::vector<NtnCarrier> ulCarriers;
    for (uint32_t col = 0; col < colours; ++col)
    {
        dlColourHz[col] = dlFrequencyHz + col * bandwidthHz;
        ulColourHz[col] = ulFrequencyHz + col * bandwidthHz;
        // (the random streams of the fading of each colour: 10^7 of its own)
        dlCarriers.push_back(
            CreateNtnCarrier(dlColourHz[col],
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
                             fastFadingDb,
                             fadingHoldMs,
                             cellNodeIds,
                             1000000000 + static_cast<int64_t>(col) * 10000000,
                             tdlProfile,
                             ntnTdlDelayNs * 1e-9,
                             ntnTdlSpeedKmph / 3.6 * dlColourHz[col] / 299792458.0,
                             multiCell ? 0 : -1));
        ulCarriers.push_back(
            CreateNtnCarrier(ulColourHz[col],
                             environment,
                             gatewayMobility,
                             feederLoss,
                             ntnDelay,
                             transparent,
                             levels,
                             atmosphereTable,
                             equatorial,
                             linkMarginDb + ulPolarizationLossDb,
                             fixedFastFading,
                             fastFadingDb,
                             fadingHoldMs,
                             cellNodeIds,
                             2000000000 + static_cast<int64_t>(col) * 10000000,
                             tdlProfile,
                             ntnTdlDelayNs * 1e-9,
                             ntnTdlSpeedKmph / 3.6 * ulColourHz[col] / 299792458.0,
                             multiCell ? 1 : -1));
    }
    NtnCarrier dlCarrier = dlCarriers[0];
    NtnCarrier ulCarrier = ulCarriers[0];
    // (GatewayTxPowerDbm is what reaches the satellite before the path loss: the gateway's EIRP)
    for (const auto& carriers : {dlCarriers, ulCarriers})
    {
        for (const auto& carrier : carriers)
        {
            carrier.feeder->SetAttribute("GatewayTxPowerDbm", DoubleValue(gatewayEirpDbm));
            carrier.feeder->SetAttribute("SatRxAntennaGainDb", DoubleValue(satFeederGtDbPerK));
            carrier.feeder->SetAttribute("RelayEirpDbm", DoubleValue(satEirpDbm));
            carrier.feeder->SetAttribute("RelayLossDb", DoubleValue(0.0));
            carrier.feeder->SetAttribute("RelayInputNoisePowerDbm", DoubleValue(relayNoiseFloorDbm));
            carrier.feeder->SetAttribute("ReceiverNoisePowerDbm", DoubleValue(noiseFloorDbm));
        }
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
            if (beamPlan[ueCell[i]].sat != 0)
            {
                satProbe->SetPosition(satOrbits[beamPlan[ueCell[i]].sat].At(t));
            }
            const Time loopRtt = 2 * ntnDelay->GetOneWayDelay(satProbe, ueMobility[i]);
            if (beamPlan[ueCell[i]].sat != 0)
            {
                satProbe->SetPosition(sat);
            }
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
    // (the bands hold the BWPs the devices are made with)
    std::deque<OperationBandInfo> colourBands;
    std::vector<BandwidthPartInfoPtrVector> colourBwps; // the downlink and uplink BWP of a colour
    for (uint32_t col = 0; col < colours; ++col)
    {
        CcBwpCreator::SimpleOperationBandConf dlBandConf(dlColourHz[col], bandwidthHz, 1);
        CcBwpCreator::SimpleOperationBandConf ulBandConf(ulColourHz[col], bandwidthHz, 1);
        colourBands.push_back(ccBwpCreator.CreateOperationBandContiguousCc(dlBandConf));
        colourBands.push_back(ccBwpCreator.CreateOperationBandContiguousCc(ulBandConf));
        OperationBandInfo& dlBand = colourBands[colourBands.size() - 2];
        OperationBandInfo& ulBand = colourBands.back();
        dlBand.GetBwpAt(0, 0)->SetChannel(dlCarriers[col].channel);
        ulBand.GetBwpAt(0, 0)->SetChannel(ulCarriers[col].channel);
        colourBwps.push_back(CcBwpCreator::GetAllBwps({dlBand, ulBand}));
    }
    BandwidthPartInfoPtrVector allBwps = colourBwps[0];

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
    if (scheduler == "ns3::NrMacSchedulerOfdmaPF")
    {
        nrHelper->SetSchedulerAttribute("UlMarginalRate", BooleanValue(ulPfMarginal));
    }
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
    nrHelper->SetSchedulerAttribute("PuschMinBlockBytes", UintegerValue(puschMinBlockBytes));
    nrHelper->SetSchedulerAttribute("UlSinrAvgTime", TimeValue(MicroSeconds(std::llround(ulSinrAvgMs * 1000.0))));
    g_puschMinBlockBytes = puschMinBlockBytes;
    // A wide uplink grant for the UE whose SINR on one RB allows it, in a slot with room
    nrHelper->SetSchedulerAttribute("UlWideGrantRbs", UintegerValue(ulWideRbs));
    nrHelper->SetSchedulerAttribute("UlWideSinrDb", DoubleValue(ulWideSinrDb));
    nrHelper->SetSchedulerAttribute("UlWideLoadPct", DoubleValue(ulWideLoadPct));
    // The downlink power: TxPower over the RBs of the carrier on each RB in use, nothing on an
    // idle one. --dlBoostMaxDb: an RB in use also gets the power of the idle RBs of its symbol,
    // up to that many dB more (NrGnbPhy::MaxRbBoostDb; UniformPowerAllocUsed splits TxPower
    // over the RBs in use, which the cap limits). --dlPfdLimit: the cap is the PFD limit, which
    // DlPfdLimit() sets, and --dlBoostMaxDb a second one, if given
    if (dlBoostMaxDb > 0.0 || dlPfdLimit != 0.0)
    {
        nrHelper->SetGnbPhyAttribute("PowerAllocationType", StringValue("UniformPowerAllocUsed"));
        nrHelper->SetGnbPhyAttribute("MaxRbBoostDb",
                                     DoubleValue(dlBoostMaxDb > 0.0 ? dlBoostMaxDb : 100.0));
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
    NetDeviceContainer gnbDevs;
    for (uint32_t c = 0; c < nCells; ++c)
    {
        // (the gNB of a beam is on its satellite's node, a node of its own for each beam; its
        // BWPs are the downlink and uplink carriers of its colour)
        gnbDevs.Add(nrHelper->InstallGnbDevice(NodeContainer(cellNodes.Get(c)),
                                               colourBwps[beamPlan[c].colour]));
        NrHelper::GetGnbPhy(gnbDevs.Get(c), 0)
            ->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
        NrHelper::GetGnbPhy(gnbDevs.Get(c), 1)
            ->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));
    }

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
            // (several beams: the pattern is a loss of each link, see BeamTaperLossModel)
            const double taperDb = multiCell ? 0.0
                                             : BeamTaperDb(satMobility->GetPosition(),
                                                           centrePosition,
                                                           ueMobility[i]->GetPosition(),
                                                           ka);
            g_scene.ueElements[i].push_back(CreateObjectWithAttributes<IsotropicAntennaModel>(
                "Gain",
                DoubleValue(ueAntennaGainDbi + taperDb)));
        }
        nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(g_scene.ueElements[i][0]));
        ueDevs.Add(nrHelper->InstallUeDevice(NodeContainer(ueNodes.Get(i)),
                                             colourBwps[beamPlan[ueCell[i]].colour]));
    }

    for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
    {
        for (uint32_t c = 0; c < nCells; ++c)
        {
            NrHelper::GetGnbPhy(gnbDevs.Get(c), bwp)->SetTxPower(satEirpDbm - satAntennaGainDb);
            NrHelper::GetGnbPhy(gnbDevs.Get(c), bwp)
                ->SetAttribute("NoiseFigure", DoubleValue(satNoiseFigureDb));
            // (the PHY counts its resource blocks once the simulation has started)
            Simulator::Schedule(MilliSeconds(1),
                                [phy = NrHelper::GetGnbPhy(gnbDevs.Get(c), bwp), carrierRbs]() {
                                    NS_ABORT_MSG_UNLESS(phy->GetRbNum() == carrierRbs,
                                                        "the nr module made "
                                                            << phy->GetRbNum()
                                                            << " resource blocks of the carrier, "
                                                               "not "
                                                            << carrierRbs);
                                });
        }
        for (uint32_t i = 0; i < nUes; ++i)
        {
            NrHelper::GetUePhy(ueDevs.Get(i), bwp)->SetTxPower(ueTxPowerDbm);
            NrHelper::GetUePhy(ueDevs.Get(i), bwp)
                ->SetAttribute("NoiseFigure", DoubleValue(ueNoiseFigureDb));
        }
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
    if (multiCell)
    {
        // a UE is in the cell of the beam it was dropped in (the closest gNB is not that: the
        // gNBs of one satellite are all at the same place)
        for (uint32_t i = 0; i < nUes; ++i)
        {
            nrHelper->AttachToGnb(ueDevs.Get(i), gnbDevs.Get(ueCell[i]));
        }
    }
    else
    {
        nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);
    }
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
                    Callback<Time>([ntnDelay, sat = cellMobility[ueCell[i]], ue = ueMobility[i]]() {
                        return ntnDelay->GetOneWayDelay(sat, ue);
                    }));
        }
    }

    // The satellite points its arrays at the beam centre: once when it stands still, every
    // 100 ms when it flies, together with the beam gain towards every UE
    g_scene.ues = ueMobility;
    g_scene.delay = ntnDelay;
    g_scene.ueAntennaGainDbi = ueAntennaGainDbi;
    g_scene.flying = !freezeGeometry;
    g_scene.orbits = satOrbits;
    // The ground part of the way to the gateway: from the remote host, or from the far UE's gNB
    // (whose radio adds next to nothing)
    g_scene.groundDelayMs = farUe ? farEndUpfDelayMs + upfGwDelayMs : hostGwDelayMs;
    for (uint32_t c = 0; c < nCells; ++c)
    {
        // (cell 0 of a run without --beams or --satellites: the beam on UE 0)
        Cell cell;
        cell.sat = beamPlan[c].sat;
        cell.beam = beamPlan[c].beam;
        cell.tier = beamPlan[c].tier;
        cell.colour = beamPlan[c].colour;
        cell.mobility = cellMobility[c];
        cell.centre = c == 0 ? centrePosition : beamPlan[c].centre;
        std::tie(cell.northKm, cell.eastKm) = mapKm(cell.centre);
        cell.gnb = gnbDevs.Get(c)->GetObject<NrGnbNetDevice>();
        cell.cellId = cell.gnb->GetCellId();
        for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
        {
            cell.satArrays.push_back(NrHelper::GetGnbPhy(gnbDevs.Get(c), bwp)
                                         ->GetSpectrumPhy()
                                         ->GetAntenna()
                                         ->GetObject<UniformPlanarArray>());
        }
        cell.dlPhy = NrHelper::GetGnbPhy(gnbDevs.Get(c), 0);
        cell.dlExtra = dlCarriers[cell.colour].serviceExtra;
        cell.dlServiceLoss = dlCarriers[cell.colour].serviceLoss;
        g_scene.cellOfMobility[PeekPointer(cell.mobility)] = c;
        g_scene.cellOfId[cell.cellId] = c;
        g_scene.cells.push_back(cell);
    }
    g_scene.cellOf = ueCell;
    for (uint32_t i = 0; i < nUes; ++i)
    {
        g_scene.ueOfMobility[PeekPointer(ueMobility[i])] = i;
    }
    g_scene.taperInLoss = multiCell;
    g_scene.beamFloorDb = floorDb;
    g_scene.pruneDb = beamPruneDb;
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
    g_scene.dlNominalDbm = satEirpDbm - satAntennaGainDb;
    g_scene.dlPfdLimit = dlPfdLimit;
    g_scene.pdcchBoostCapDb =
        pdcch ? AttributeDefault<DoubleValue>("ns3::NrPdcch", "BoostMaxDb") : 0.0;
    g_scene.dlPfdMarginDb = dlPfdMarginDb;
    g_scene.dlRbMinDbm = dlRbPowerMinDbm;
    g_scene.dlBoostMaxDb = dlBoostMaxDb;
    g_scene.dlAntennaGainDb = satAntennaGainDb;
    g_scene.dlRbHz = occupiedHz / carrierRbs;
    g_scene.carrierRbs = carrierRbs;
    g_scene.ulPowerRefDeg = ulPowerRefDeg;
    g_scene.ueMaxDbm = ueTxPowerDbm;
    g_scene.ueLossDb.assign(nUes, NAN);
    if (dlPowerRefDeg > 0.0)
    {
        g_scene.dlRefLossDb =
            FeederLinkLossModel::FreeSpaceDb(dlFrequencyHz, slantM(dlPowerRefDeg)) +
            dlCarrier.serviceExtra->AtmosphericDb(dlPowerRefDeg);
        for (Cell& cell : g_scene.cells)
        {
            DlPowerControl(cell);
        }
    }
    if (dlPfdLimit != 0.0)
    {
        for (Cell& cell : g_scene.cells)
        {
            DlPfdLimit(cell);
        }
        // the floor and the limit: a limit under the floor, at its lowest with the satellite
        // over the beam centre, is a contradiction
        const double lowestDbm = PfdRbLimitDbm(dlPfdLimit,
                                               dlPfdMarginDb,
                                               slantM(90.0),
                                               satAntennaGainDb,
                                               g_scene.dlRbHz);
        NS_ABORT_MSG_IF(dlRbPowerMinDbm > 0.0 && lowestDbm < dlRbPowerMinDbm,
                        "--dlPfdLimit allows an RB " << lowestDbm
                                                     << " dBm at 90 deg, less than "
                                                        "--dlRbPowerMinDbm");
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
    if (multiCell)
    {
        UpdateTaper();
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
    g_dlRbPower.assign(nUes, SnrStats());
    g_dlDataSnr.assign(nUes, SnrStats());
    g_ulGrants.assign(nUes, UlGrantStats());
    g_ulNarrowRbs = ulGrantRbs;
    g_ulWideRbs = ulWideRbs;
    g_snrSteps.assign(nUes, std::map<int, SnrStep>());
    g_snrStep.assign(nUes, 0);
    g_snrWindow.assign(nUes, {Seconds(0), Seconds(0)});
    g_rntiUe.clear();
    g_ulSnrRbs = ulGrantRbs > 0 ? ulGrantRbs : 2; // as the CNR in the header
    if (!sinrTrace.empty())
    {
        g_sinrTraceDl.open(sinrTrace + "_dl.csv");
        g_sinrTraceUl.open(sinrTrace + "_ul.csv");
        NS_ABORT_MSG_UNLESS(g_sinrTraceDl.is_open() && g_sinrTraceUl.is_open(),
                            "--sinrTrace: cannot write " << sinrTrace << "_dl.csv or _ul.csv");
        g_sinrTraceDl << "t_s,ue,dl_ctrl_sinr_db\n";
        g_sinrTraceUl << "t_s,ue,ul_sinr_db,rbs,mcs,tb_bytes,rv,corrupt,tbler\n";
    }
    for (uint32_t c = 0; c < nCells; ++c)
    {
        for (uint32_t bwp = 0; bwp < 2; ++bwp)
        {
            NrHelper::GetGnbPhy(gnbDevs.Get(c), bwp)
                ->GetSpectrumPhy()
                ->TraceConnectWithoutContext("RxPacketTraceGnb", MakeCallback(&UlSinr));
        }
        if (dlPfdLimit != 0.0 || dlBoostMaxDb > 0.0 || dlPowerRefDeg > 0.0)
        {
            Connect(NrHelper::GetGnbPhy(gnbDevs.Get(c), 0),
                    "DlDataPower",
                    MakeBoundCallback(&DlRbPower, static_cast<uint32_t>(g_scene.cells[c].cellId)));
        }
    }
    // The closed loops (--dlClosedLoop, --ulClosedLoop): off, nothing below runs
    g_loop.dl = dlClosedLoop;
    g_loop.ul = ulClosedLoop;
    g_loop.dlSnrMinDb = dlSnrMinDb;
    g_loop.dlMcsMin = dlMcsMin;
    g_loop.dlMaxDb = dlBoostMaxDb > 0.0 ? dlBoostMaxDb : 30.0;
    g_loop.ulHighDb = ulSnrHighDb;
    g_loop.ulLowDb = ulSnrLowDb;
    g_loop.ulMinDbm = ulPowerMinDbm;
    g_loop.stepDb = closedLoopStepDb;
    g_loop.landing = closedLoopLanding;
    g_loop.filter = 1.0 - std::exp(-1.0 / closedLoopAvg);
    g_loop.period = MilliSeconds(static_cast<int64_t>(std::llround(closedLoopMs)));
    g_loop.connected = [ueDevs](uint32_t i) {
        return DynamicCast<NrUeNetDevice>(ueDevs.Get(i))->GetRrc()->GetState() >=
               NrUeRrc::CONNECTED_NORMALLY;
    };
    g_dlLoop.assign(nUes, DlLoopUe());
    g_ulLoop.assign(nUes, UlLoopUe());
    g_loopTally.assign(nUes, LoopTally());
    if (dlClosedLoop != 0 || ulClosedLoop)
    {
        if ((dlClosedLoop & 2) != 0)
        {
            // the scheduler's MCS for a CQI, to turn the MCS target into an SNR
            g_loop.amc = CreateObjectWithAttributes<NrAmc>(
                "ErrorModelType",
                TypeIdValue(TypeId::LookupByName("ns3::" + errorModel)));
            g_loop.model = NrSpectrumValueHelper::GetSpectrumModel(carrierRbs, dlFrequencyHz, 15e3);
            NS_ABORT_MSG_UNLESS(dlMcsMin <= g_loop.amc->GetMaxMcs(),
                                "--dlMcsMin " << dlMcsMin << " is over the highest MCS of "
                                              << errorModel << " (" << +g_loop.amc->GetMaxMcs()
                                              << ")");
        }
        if (dlClosedLoop != 0)
        {
            for (uint32_t c = 0; c < nCells; ++c)
            {
                Connect(NrHelper::GetGnbPhy(gnbDevs.Get(c), 0),
                        "DlDataPowerFree",
                        MakeBoundCallback(&DlRbPowerFree,
                                          static_cast<uint32_t>(g_scene.cells[c].cellId)));
            }
        }
        Simulator::Schedule(g_loop.period, &ClosedLoopTick);
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
    g_cc.attempts.assign(nUes, {});
    g_cc.retrying.assign(nUes, false);
    g_cc.retry = retry;
    g_cc.retryDelay = Seconds(retrySeconds);
    g_cc.maxTries = maxRetry;
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
    for (uint32_t c = 0; c < nCells; ++c)
    {
        Connect(gnbDevs.Get(c)->GetObject<NrGnbNetDevice>()->GetRrc(),
                "ConnectionReconfiguration",
                MakeBoundCallback(&SatGnbReconfigured,
                                  "/NodeList/" + std::to_string(gnbDevs.Get(c)->GetNode()->GetId()) +
                                      "/DeviceList/*/$ns3::NrGnbNetDevice/NrGnbRrc/UeMap",
                                  &ueOfImsi));
    }
    for (uint32_t i = 0; i < nUes; ++i)
    {
        Connect(NrHelper::GetUePhy(ueDevs.Get(i), 0), "DlCtrlSinr", MakeBoundCallback(&DlSinr, i));
        Connect(NrHelper::GetUePhy(ueDevs.Get(i), 0)->GetSpectrumPhy(),
                "RxPacketTraceUe",
                MakeBoundCallback(&DlDataSinr, i));
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
              (multiCell ? " in " + std::to_string(nCells) + " cells" : std::string()) +
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
        Setting("",
                "a STATIC-NACK is answered with " + std::to_string(rohcRepairIrPackets) +
                    " IR packet" + (rohcRepairIrPackets == 1 ? "" : "s"));
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
            if (rohcFallbackAfter > 0)
            {
                Setting("",
                        "after " + std::to_string(rohcFallbackAfter) +
                            " such packets in a row the decompressor asks for U-mode (RFC 3095 "
                            "5.6) and stays there");
            }
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
    if (ueBufferAgeS > 0.0)
    {
        Setting("UE buffer",
                "a satellite UE drops voice that waited over " + Num(ueBufferAgeS, 3) +
                    " s in its queue (--ueBufferAgeS): a discard");
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
                            "aside),"
                      : std::string("none: no call is released for its delays (--dropCalls)"));
    if (dropCalls)
    {
        Setting("", "while the calls on reserve half of the cell's RBs (one way) or CCEs or more;");
        Setting("", "the UE then goes to RRC_IDLE (a released UE connects only when it is to call)");
        if (!retry)
        {
            Setting("retry", "none: a released UE stays idle for the rest of its window (--retry=1)");
        }
    }
    if (retry)
    {
        Setting("retry",
                "a UE whose call was released, or refused by admission (the gNB then releases its "
                "connection), connects again " +
                    Num(retrySeconds, 1) + " s later (new RNTI, same address) and calls again:");
        Setting("",
                "a new call on a new bearer, up to " + std::to_string(maxRetry) +
                    " tries, its first call included (--retry, --retrySeconds, --maxRetry)");
    }

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
    if (multiCell)
    {
        Setting("cells",
                std::to_string(nCells) + " (" + std::to_string(beams) + " beam" +
                    (beams == 1 ? "" : "s") + " x " + std::to_string(satellites) + " satellite" +
                    (satellites == 1 ? "" : "s") +
                    (nCells < beams * satellites ? ", the beams that miss the Earth or point under 5 deg left out)"
                                                 : ")") +
                    ": a gNB and --nUes = " + std::to_string(nUesPerCell) + " UEs in each");
        Setting("",
                "layout of TR 38.821 6.1.1.1: a hexagonal grid in the satellite's UV plane, " +
                    Num(beamSpacing, 2) + " x ABS = " +
                    Num(beamSpacing * std::sqrt(3.0) * g_halfPowerX / g_scene.beamKa[0], 4) +
                    " apart (" + Num(beamPitchDeg * beamSpacing, 2) +
                    " deg of arc on the ground along the track at the centre beam)");
        Setting("",
                "the pattern is a loss of each link (TR 38.811 6.4.1, down to " + Num(floorDb, 0) +
                    " dB); a link with the pattern " + Num(beamPruneDb, 0) +
                    " dB under the UE's serving beam is not computed (--beamPruneDb)");
        Setting("reuse",
                std::to_string(beamReuse) + ": " + std::to_string(colours) + " frequency colour" +
                    (colours == 1 ? "" : "s") + ", each a carrier pair of " + Plain(bandwidthMHz) +
                    " MHz one bandwidth above the one before; the beams of a colour share a "
                    "channel and interfere, other colours do not meet");
        if (satellites > 1)
        {
            Setting("satellites",
                    std::to_string(satellites) + " standing still, " + Num(satSepUsedDeg, 2) +
                        " deg of arc apart along the track" +
                        (satSeparationDeg > 0.0 ? " (--satSeparationDeg)"
                                                : ", 2 tiers + 1 beam pitches (auto)") +
                        ", centred on one satellite at --elevation over UE 0; the colours go on "
                        "across them");
        }
        Setting("handover",
                "none: a UE belongs to the cell it was dropped in, where its beam is the "
                "strongest of all (--beamArea, --beamEdgeDb), all through the calls");
        if (!freezeGeometry)
        {
            Setting("flying",
                    "the beams stay on the ground where the layout put them when the satellite "
                    "was over the centre (a real one steers them); their pattern towards a UE "
                    "changes with the geometry, every 100 ms. The antenna keeps its width, so the "
                    "lower the satellite is, the more the beams overlap (the C/I of the table is "
                    "that of the middle of the calls): for the interference between beams at one "
                    "elevation use --freezeGeometry=1");
        }
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
        if (dlPfdLimit != 0.0)
        {
            // what the PFD limit allows an RB, with the satellite at an elevation above the centre
            auto limitDbm = [&](double elevationDeg) {
                return PfdRbLimitDbm(dlPfdLimit,
                                     dlPfdMarginDb,
                                     slantM(elevationDeg),
                                     satAntennaGainDb,
                                     g_scene.dlRbHz);
            };
            auto eirpDensity = [&](double elevationDeg) {
                return dlPfdLimit +
                       10.0 * std::log10(4.0 * M_PI * std::pow(slantM(elevationDeg), 2)) +
                       dlPfdMarginDb;
            };
            Setting("DL PFD limit",
                    Num(dlPfdLimit, 1) + " dBW/m2/MHz at the beam centre, + " +
                        Num(dlPfdMarginDb, 2) + " dB (--dlPfdMarginDb) = " +
                        Num(eirpDensity(90.0), 1) + " dBW/MHz EIRP at 90 deg");
            std::string line;
            std::string density;
            std::string fits;
            for (const double e : {90.0, 60.0, 30.0, 10.0})
            {
                line += (line.empty() ? "" : " / ") + Num(limitDbm(e), 2);
                density += (density.empty() ? "" : " / ") + Num(eirpDensity(e), 1);
                // RBs in use at which the satellite's power just reaches the limit
                const double share = satEirpDbm - satAntennaGainDb - perRbDb;
                const double most = carrierRbs * std::pow(10.0, (share - limitDbm(e)) / 10.0);
                fits +=
                    (fits.empty() ? "" : " / ") + (most < 1.0 ? "no" : Num(std::floor(most), 0));
            }
            Setting("", "an RB sends at most " + line + " dBm at 90 / 60 / 30 / 10 deg");
            Setting("", "(EIRP density " + density + " dBW/MHz at those elevations)");
            Setting("",
                    "the satellite's power (share " + Num(satTxDbm - perRbDb, 2) +
                        " dBm) reaches it with " + fits + " RBs in use or fewer");
            Setting(
                "",
                "an RB in use takes the idle RBs' power of its symbol up to that" +
                    std::string(dlBoostMaxDb > 0.0 ? " and +" + Num(dlBoostMaxDb, 2) + " dB" : "") +
                    (tdma ? " (TDMA: no RB is idle)" : "") + "; every 100 ms when it flies");
        }
        if (dlRbPowerMinDbm > 0.0)
        {
            Setting("DL RB floor",
                    Num(dlRbPowerMinDbm, 2) +
                        " dBm: --dlPowerRefDeg takes no RB under it (an RB's " + "share is " +
                        Num(satTxDbm - perRbDb, 2) + " dBm)");
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
    if (dlClosedLoop != 0)
    {
        std::string target;
        if (dlClosedLoop == 1)
        {
            target = "an SNR of " + Num(dlSnrMinDb, 1) + " dB";
        }
        else if (dlClosedLoop == 2)
        {
            target = "MCS " + std::to_string(dlMcsMin);
        }
        else
        {
            target = "an SNR of " + Num(dlSnrMinDb, 1) + " dB and MCS " + std::to_string(dlMcsMin);
        }
        Setting("DL loop",
                "closed loop (--dlClosedLoop=" + std::to_string(dlClosedLoop) +
                    "): the boost of the RBs of a UE goes only as far as");
        Setting("", "the UE needs for " + target + " at least: 0 to " +
                        Num(dlBoostMaxDb > 0.0 ? dlBoostMaxDb : 30.0, 1) +
                        " dB over the RB's share (the PFD limit binds too),");
        Setting("",
                "decided once per " + Num(closedLoopMs, 0) + " ms, in steps of " +
                    Num(closedLoopStepDb, 1) + " dB");
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
    Setting("BSR",
            std::string(NrMacShortBsrCe::IsCompact() ? "2 B short BSR (TS 38.321 6.1.3.1)"
                                                     : "5 B BSR (the nr module's; TS 38.321: 2 B)") +
                (bsrOnDemand
                     ? ", sent when a trigger fires: data into an empty buffer (it asks for a "
                       "grant too), retxBSR-Timer, periodicBSR-Timer (TS 38.321 5.4.5; "
                       "no padding BSR)"
                     : " in every block with new data, and a request for a grant whenever the UE "
                       "has data (--bsr)"));
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
                        " dB (--puschRepSinrDb), and a block of a slot of " +
                        std::to_string(MinUlBlockBytes()) +
                        " B at least (--puschMinBlockBytes); the MCS: what the error model "
                        "decodes after the K transmissions");
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
    if (ulClosedLoop)
    {
        Setting("UL loop",
                "closed loop (--ulClosedLoop): a UE lowers its power while its SNR is over " +
                    Num(ulSnrHighDb, 1) + " dB,");
        Setting("",
                "never so far that it is under " + Num(ulSnrLowDb, 1) +
                    " dB; TS 38.213 7.1.1 accumulated command f <= 0, at least " +
                    Num(ulPowerMinDbm, 0) + " dBm;");
        Setting("",
                "decided once per " + Num(closedLoopMs, 0) + " ms, in steps of " +
                    Num(closedLoopStepDb, 1) + " dB down, " + Num(3.0 * closedLoopStepDb, 1) +
                    " dB up at most");
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
    if (randomLayout && multiCell)
    {
        Setting("UEs",
                "UE 0 on the centre of beam 0, the others at random places of their cell: " +
                    std::string(beamArea == "contour"
                                    ? "within 3 dB of the peak of its beam"
                                    : "where its beam is the strongest of all and at most " +
                                          Num(beamEdgeDb, 1) +
                                          " dB under its peak (the outermost ring: 3 dB)") +
                    " (--beamArea, --beamEdgeDb)");
    }
    else if (randomLayout)
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
                                      fadingLossDb + (downlink ? 0.0 : ulPolarizationLossDb);
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
                : tdlProfile != 0
                    ? std::string("NTN-TDL-") +
                          static_cast<char>(std::toupper(static_cast<unsigned char>(tdlProfile))) +
                          " of TR 38.811 Table 6.9.2-" + std::string(1, "1234"[tdlProfile - 'a']) +
                          ", the same at every elevation (--fastFading=ntn-tdl-" +
                          std::string(1, tdlProfile) + ")"
                    : "3GPP channel per RB and slot: TR 38.901 7.5 with TR 38.811's " +
                          environment + " tables");
        if (tdlProfile != 0)
        {
            const double dopplerHz = ntnTdlSpeedKmph / 3.6 * dlFrequencyHz / 299792458.0;
            Setting("",
                    "delays scaled to " + Num(ntnTdlDelayNs, 0) + " ns (--ntnTdlDelayNs), a UE at " +
                        Num(ntnTdlSpeedKmph, 1) + " km/h (--ntnTdlSpeedKmph): Doppler " +
                        Num(dopplerHz, 1) + " Hz on the downlink,");
            Setting("",
                    dopplerHz > 0.0
                        ? "time: Rayleigh taps decorrelate in 0.423 / f_D = " + Num(423.0 / dopplerHz, 0) +
                              " ms, the fading of a Ricean tap (with its peak at 0.7 f_D) to a correlation of 0.5"
                              " in 0.18 / f_D = " +
                              Num(179.0 / dopplerHz, 0) +
                              " ms; the elevation is in the link budget only"
                        : "no motion, no fading in time; the elevation is in the link budget only");
        }
        if (!fixedFastFading && tdlProfile == 0)
        {
            Setting("",
                    fadingHoldMs > 0.0
                        ? "each fading stays for " + Num(fadingHoldMs, 0) +
                              " ms, then a new one is drawn; the satellite's Doppler does not "
                              "fade (--fadingHoldMs)"
                        : "ns-3's: a new fading whenever an end has moved 1 m (a flying satellite: "
                          "every 1 to 2 ms)");
        }
        if (linkMarginDb != 0.0)
        {
            Setting(
                "  margin",
                Num(linkMarginDb, 2) +
                    " dB more on every link (--linkMarginDb; TR 38.821's additional losses: 0)");
        }
        if (ulPolarizationLossDb != 0.0)
        {
            Setting("  polarisation",
                    Num(ulPolarizationLossDb, 2) +
                        " dB more on the uplink only (--ulPolarizationLossDb; TR 38.821 6.1.1.1: 3 "
                        "with circular satellite reception and polarisation re-use, else 0)");
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
                for (const uint32_t n : NrMacSchedulerUeInfo::kPuschRepetitionValues)
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
            // (or more, as the scheduler takes it, if the block of a slot is under the least
            // block the UE sends data in, or --puschMinBlockBytes)
            for (const uint32_t n : NrMacSchedulerUeInfo::kPuschRepetitionValues)
            {
                if (puschRepetitions != 0 || !ulLinkAdaptation || n <= k ||
                    amc->CalculateTbSize(mcs, 1, ulGrantRbs * NrAmc::NR_AMC_NUM_SYMBOLS_DEFAULT) >=
                        MinUlBlockBytes())
                {
                    continue;
                }
                k = n;
                amc->CreateCqiFeedbackSiso(sinr, mcs, k);
            }
            const uint32_t tbBytes = amc->CalculateTbSize(mcs, 1, ulGrantRbs * 13);
            Setting(
                "",
                "UE 0 at " + Num(elevation, 1) + " deg: " + Num(ue0Db, 2) + " dB on its " +
                    std::to_string(ulGrantRbs) + " RB give " +
                    (k > 1 ? "K = " + std::to_string(k) + ", " : std::string()) + "MCS " +
                    std::to_string(mcs) + " and a block of " + std::to_string(tbBytes) + " B " +
                    (k > 1 ? "every " + std::to_string(k) + " slots" : std::string("per slot")) +
                    (tbBytes < NrMacShortBsrCe::MinUlGrantBytes()
                         ? " (too small: see the NOTE below)"
                         : ""));
            ulBlockTooSmall = tbBytes < NrMacShortBsrCe::MinUlGrantBytes();
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
    if (multiCell)
    {
        Setting("interference",
                beamReuse == 1 ? "every beam sends on the same channel: a UE hears the other "
                                 "beams of its satellite (and of the others) as interference"
                               : "the beams of one colour share a channel and interfere; the "
                                 "beams of another colour are on another channel (reuse " +
                                     std::to_string(beamReuse) + ")");
    }
    else
    {
        Setting("interference", "none: one beam, no other cell; every SINR here is a signal-to-noise ratio");
    }
    Setting("scheduler", schedulerIs);
    if (scheduler == "ns3::NrMacSchedulerOfdmaPF")
    {
        Setting("",
                std::string("uplink: ") +
                    (ulPfMarginal ? "the UEs are ranked by what the next resource block group adds "
                                    "to the block of each (--ulPfMarginal)"
                                  : "the nr module's ranking, by the rate of a fixed reference "
                                    "grant (--ulPfMarginal=0)"));
    }
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
        Setting("", "RNTI and stop its downlink; TS 38.321 5.1.4 and 5.1.5 keep the C-RNTI)");
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
                    "; a DCI of data is " + std::to_string(dciBits) + " + 24 CRC bits");
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
                    CommonLevels(pdcchView) + "; a UE monitors " + std::to_string(monitored) +
                    " (limit " + std::to_string(NrPdcch::GetMaxMonitoredCandidates(0)) + ")");
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
            {
                std::vector<std::pair<std::string, std::string>> bits;
                std::vector<std::pair<std::string, std::string>> levelOf;
                for (const auto& c : kCommonDcis)
                {
                    bits.emplace_back(std::to_string(pdcchView->GetDciBits(c.purpose)), c.name);
                    levelOf.emplace_back(std::to_string(+pdcchView->GetCommonAl(c.purpose)),
                                         c.name);
                }
                bits.emplace_back(std::to_string(pdcchView->GetDciBits(NrPdcch::RRC_DL)), "RRC");
                Setting("", "size, bits without the 24 CRC bits: " + GroupedByValue(bits));
                Setting("", "(TS 38.212 7.3.1: formats 1_0, 0_0 and 2_2 are padded to one size)");
                Setting("",
                        "level (--ns3::NrPdcch::CommonAl, Sib1Al, ...): " +
                            GroupedByValue(levelOf) + "; RRC: by the UE's SINR");
            }
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
                        " B blocks = " + talkB + " B + " + BsrText() + " + 1 B confirmation CE");
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
            Setting("", "+ " + BsrText() + " + 6 B confirmation MAC CE");
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
                "flies south at " + Num(orbit.vel0.GetLength() / 1000.0, 2) +
                    " km/s, is over UE 0 at " + time(overheadAt) + " and down at " +
                    Num(setElevationDeg, 0) + " deg again at " + time(downAt));
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
                    "is on only while the satellite is " + Num(setElevationDeg, 0) +
                        " deg or more above its own UE, and ends with UE 0's at the latest");
        }
        if (passIgnored)
        {
            Setting("", "NOTE: --duration is not used with --fullPass=1");
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
               "under " << NrMacShortBsrCe::MinUlGrantBytes() << " B (a UE sends no data in one).\n"
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
        std::vector<uint16_t> cellIds;
        for (const Cell& c : g_scene.cells)
        {
            cellIds.push_back(c.cellId);
        }
        Simulator::Schedule(callsStart, [cellIds]() {
            for (const uint16_t id : cellIds)
            {
                NrPdcch::Get(id)->ResetStats();
            }
        });
        Simulator::Schedule(callsEnd, [cellIds]() {
            for (const uint16_t id : cellIds)
            {
                NrPdcch::Get(id)->SetCounting(false);
            }
        });
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
        g_cc.reserved.assign(nCells, {});
        g_cc.peakShare.assign(nCells, {});
        g_cc.peakAdmitted.assign(nCells, 0);
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
                    for (const uint32_t n : NrMacSchedulerUeInfo::kPuschRepetitionValues)
                    {
                        if (ulDb + 10.0 * std::log10(n) >= repSinrDb)
                        {
                            k = n;
                            break;
                        }
                    }
                    // (or more, as the scheduler takes it, if the block of a slot is too small)
                    SpectrumValue ulSinr(model);
                    ulSinr = 0.0;
                    for (uint32_t rb = 0; rb < ulRbs && rb < ulSinr.GetValuesN(); ++rb)
                    {
                        ulSinr[rb] = std::pow(10.0, ulDb / 10.0);
                    }
                    uint8_t ulMcs = 0;
                    amc->CreateCqiFeedbackSiso(ulSinr, ulMcs, k);
                    for (const uint32_t n : NrMacSchedulerUeInfo::kPuschRepetitionValues)
                    {
                        if (n <= k ||
                            amc->CalculateTbSize(ulMcs,
                                                 1,
                                                 ulRbs * NrAmc::NR_AMC_NUM_SYMBOLS_DEFAULT) >=
                                MinUlBlockBytes())
                        {
                            continue;
                        }
                        k = n;
                        amc->CreateCqiFeedbackSiso(ulSinr, ulMcs, k);
                    }
                }
                need[1] = perSecond * ulRbs * k;
            }
            if (modelPdcch)
            {
                const uint8_t al =
                    NrPdcch::Get(g_scene.cells[g_scene.cellOf[i]].cellId)->ChooseAl(dlDb);
                // (semi-persistent: DCIs for retransmissions and the odd activation only)
                const double dcis =
                    schedulingMode == "dynamic" ? perSecond : perSecond - 50.0 + 1.0;
                need[2] = al * dcis * ((down ? 1 : 0) + (up ? 1 : 0));
            }
            return need;
        };
        g_cc.releaseUe = [](uint32_t i) {
            const int32_t c = CurrentConnection(i);
            if (c < 0)
            {
                return;
            }
            const uint16_t rnti = g_connections[i][c].rnti;
            const Ptr<NrGnbRrc> gnbRrc = g_scene.cells[g_scene.cellOf[i]].gnb->GetRrc();
            if (gnbRrc->HasUeManager(rnti))
            {
                gnbRrc->GetUeManager(rnti)->SendRrcConnectionRelease();
            }
        };
        // --retry: the UE asks for a connection, as the helper does when it attaches a UE (once
        // for each carrier). It is camped in RRC_IDLE since the release (the nr module's
        // ReconnectAfterRelease is off), or still looking for the cell; the NAS takes it from
        // there: random access, a new RNTI, and the QoS flows of the core on new bearers.
        g_cc.ueConnected = [ueDevs](uint32_t i) {
            return DynamicCast<NrUeNetDevice>(ueDevs.Get(i))->GetRrc()->GetState() >=
                   NrUeRrc::CONNECTED_NORMALLY;
        };
        g_cc.reconnectUe = [ueDevs](uint32_t i) {
            if (g_loop.dl != 0 || g_loop.ul)
            {
                LoopReset(i); // the random access of the new call is at full power
            }
            Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(ueDevs.Get(i));
            Ptr<NrGnbNetDevice> gnb = g_scene.cells[g_scene.cellOf[i]].gnb;
            for (uint32_t b = 0; b < gnb->GetCcMapSize(); ++b)
            {
                ue->GetNas()->Connect(gnb->GetCellId(), gnb->GetBwpArfcn(b));
            }
        };
        if (pdcch)
        {
            g_cc.callStarts = [](uint32_t i) {
                const int32_t c = CurrentConnection(i);
                if (c >= 0)
                {
                    NrPdcch::Get(g_scene.cells[g_scene.cellOf[i]].cellId)
                        ->CallStarts(g_connections[i][c].rnti);
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

    // What became of UE i's call, for the tables that list every UE. A UE that never saw the
    // satellite at 10 deg or more during UE 0's call has no call at all; the highest elevation
    // it saw is looked up once.
    auto highestElevationDeg = [&](uint32_t i) {
        double highest = -90.0;
        const Vector ue = ueMobility[i]->GetPosition();
        for (Time t = callsStart; t < callsEnd; t += Seconds(1))
        {
            highest = std::max(highest, ElevationDeg(orbit.At(t), ue));
        }
        return std::max(highest, ElevationDeg(orbit.At(callsEnd), ue));
    };
    // What became of one try of a call (the call of the plan, or --retry: a call again)
    auto tryOutcome = [&](const CallControl::Attempt& a) {
        std::ostringstream o;
        o << std::fixed;
        switch (a.outcome)
        {
        case CallControl::BLOCKED:
            o << "blocked at " << std::setprecision(2) << a.at.GetSeconds() << " s: no "
              << g_resourceName[a.limitedBy] << " left";
            break;
        case CallControl::RELEASED:
            o << "released at " << std::setprecision(2) << a.end.GetSeconds() << " s, "
              << dirName(static_cast<Dir>(a.releasedDir)) << " " << std::setprecision(1)
              << a.onTimePct << " % of its packets on time";
            break;
        case CallControl::WAITING:
            o << "asked for a connection, none before the end of its window";
            break;
        default:
            o << "call to the end of its window";
        }
        return o.str();
    };
    auto whatHappened = [&](uint32_t i) {
        const CallControl& c = g_cc;
        std::ostringstream o;
        o << std::fixed;
        if (!ueHasCall[i])
        {
            o << "no call: the satellite is not " << minCallElevationText
              << " deg above it during UE 0's call (at most " << std::setprecision(1)
              << highestElevationDeg(i) << " deg)";
            return o.str();
        }
        if (c.attempts[i].size() > 1)
        {
            // --retry: it called again after a release; the last try is what it ended with
            o << c.attempts[i].size() << " tries, the last: " << tryOutcome(c.attempts[i].back());
        }
        else
        {
            switch (c.status[i])
            {
            case CallControl::BLOCKED:
                o << "blocked at " << std::setprecision(3) << c.at[i].GetSeconds() << " s: no "
                  << g_resourceName[c.limitedBy[i]] << " left";
                break;
            case CallControl::RELEASED:
                o << "released at " << std::setprecision(2) << c.at[i].GetSeconds() << " s, "
                  << dirName(static_cast<Dir>(c.releasedDir[i])) << " " << std::setprecision(1)
                  << c.onTimePct[i] << " % of its packets on time";
                break;
            case CallControl::WAITING:
                o << "call on, no call control (--rtp=0)";
                break;
            default:
                o << "call to the end of its window";
            }
        }
        if (g_rlf[i] > 0)
        {
            o << "; " << g_rlf[i] << " radio link failure" << (g_rlf[i] > 1 ? "s" : "");
        }
        return o.str();
    };
    // One character for the narrow tables: B blocked, R released, r released and then called
    // again (--retry), - no call, blank otherwise
    auto statusMark = [&](uint32_t i) {
        return !ueHasCall[i]                             ? '-'
               : g_cc.status[i] == CallControl::BLOCKED  ? 'B'
               : g_cc.status[i] == CallControl::RELEASED ? 'R'
               : g_cc.attempts[i].size() > 1             ? 'r'
                                                         : ' ';
    };

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
        // (the call of the plan is the UE's first try: what became of it is counted here, the
        // retries of --retry further down)
        for (uint32_t i = 0; i < nUes; ++i)
        {
            if (c.status[i] == CallControl::WAITING || c.attempts[i].empty())
            {
                continue; // no call: the satellite is never 10 deg above the UE during UE 0's
            }
            const CallControl::Attempt& first = c.attempts[i].front();
            ++offered;
            if (first.outcome == CallControl::BLOCKED)
            {
                ++blocked;
                ++blockedBy[first.limitedBy];
                blockedUes.push_back(i);
                continue;
            }
            released += first.outcome == CallControl::RELEASED;
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
                  << c.PeakAdmitted() << " on at once" << (multiCell ? " in a cell" : "")
                  << "\n";
        if (c.retry)
        {
            // --retry: what became of the retries (the tries after the first of each UE)
            uint32_t ues = 0;
            uint32_t retries = 0;
            uint32_t retryAdmitted = 0;
            uint32_t retryBlocked = 0;
            uint32_t retryNoBearers = 0;
            uint32_t retryReleased = 0;
            uint32_t exhausted = 0;
            for (uint32_t i = 0; i < nUes; ++i)
            {
                const auto& tries = c.attempts[i];
                if (tries.size() > 1)
                {
                    ++ues;
                    exhausted += tries.size() >= c.maxTries &&
                                 (tries.back().outcome == CallControl::RELEASED ||
                                  tries.back().outcome == CallControl::BLOCKED);
                }
                for (std::size_t n = 1; n < tries.size(); ++n)
                {
                    ++retries;
                    retryBlocked += tries[n].outcome == CallControl::BLOCKED;
                    retryNoBearers += tries[n].outcome == CallControl::WAITING;
                    retryAdmitted += tries[n].outcome != CallControl::BLOCKED &&
                                     tries[n].outcome != CallControl::WAITING;
                    retryReleased += tries[n].outcome == CallControl::RELEASED;
                }
            }
            std::cout << "  retry: " << Num(retrySeconds, 1)
                      << " s after a release or a refused call the UE connects again and calls, " << maxRetry << " tries at most (the first included): "
                      << ues << " UE" << (ues == 1 ? "" : "s") << " called again, " << retries
                      << " retries, of them admitted " << retryAdmitted << " (released again "
                      << retryReleased << "), blocked " << retryBlocked << ", no connection before "
                      << "the end " << retryNoBearers << "; " << exhausted << " UE"
                      << (exhausted == 1 ? "" : "s") << " released or blocked on the last of their "
                      << maxRetry << " tries\n";
        }
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
            std::cout << (r ? ", " : " ") << std::setprecision(1) << 100.0 * c.PeakShare(r) << " %";
        }
        std::cout << "\n";
        // Every UE, and what happened to it: no call, blocked, released, or the call to the end
        {
            std::cout
                << "  UE      address   N[km]   E[km]  call from[s]    to[s]  what happened\n";
            for (uint32_t i = 0; i < nUes; ++i)
            {
                std::cout << std::fixed << std::setw(4) << i << std::setw(13)
                          << addressOf(ueIfaces.GetAddress(i)) << std::setprecision(1)
                          << std::setw(8) << ueNorthKm[i] << std::setw(8) << ueEastKm[i];
                if (ueHasCall[i])
                {
                    std::cout << std::setprecision(2) << std::setw(14)
                              << ueCallStart[i].GetSeconds() << std::setw(9)
                              << ueCallEnd[i].GetSeconds();
                }
                else
                {
                    std::cout << std::setw(14) << "-" << std::setw(9) << "-";
                }
                std::cout << "  " << whatHappened(i) << "\n";
                // --retry: every try of a UE that made more than one
                if (c.attempts[i].size() > 1)
                {
                    for (const CallControl::Attempt& a : c.attempts[i])
                    {
                        std::cout << std::fixed << std::setprecision(2) << "        try "
                                  << a.number << ": asked " << std::setw(6) << a.began.GetSeconds()
                                  << " s";
                        if (a.at != Time::Max())
                        {
                            std::cout << ", bearers " << std::setw(6) << a.at.GetSeconds()
                                      << " s (RNTI "
                                      << g_connections[i][a.firstConnection].rnti << ")";
                        }
                        std::cout << ": " << tryOutcome(a) << "\n";
                    }
                }
            }
        }
        std::cout << "reserve: what the gNB estimates a call needs while its speakers talk, from "
                     "its UE's downlink SINR (uplink: that less the\n"
                  << "difference of the CNRs of the header), MCS, repetitions and HARQ.\n"
                  << "released: fewer than " << Num(dropOnTimePct, 0)
                  << " % of its packets in a direction within the PDB over 2 s, of those that "
                     "arrived or were discarded as late\n"
                  << "(not those lost on the radio), while the calls on reserve half of a "
                     "resource or more; its voice ends, and\n"
                  << "the gNB releases the UE's RRC connection. on time: that share.\n";
        if (callControlOff)
        {
            std::cout << "  NOTE: --rtp=0: no admission control and no release"
                      << (retryOff ? ", so no retry (--retry)" : "")
                      << " (the NGMN generator cannot be stopped).\n";
        }
        csvControl << admission << "," << CsvNum(admissionRbPct) << "," << CsvNum(admissionCcePct)
                   << "," << dropCalls << "," << CsvNum(dropOnTimePct) << "," << offered << ","
                   << offered - blocked << "," << blocked << "," << blockedBy[0] << ","
                   << blockedBy[1] << "," << blockedBy[2] << "," << released << ","
                   << c.PeakAdmitted();
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
            csvControl << "," << CsvNum(c.capacity[r] > 0.0 ? 100.0 * c.PeakShare(r) : NAN);
        }
        // --retry: the tries after the first, and the UEs that made them
        uint32_t csvRetries = 0;
        uint32_t csvRetriedUes = 0;
        for (uint32_t i = 0; i < nUes; ++i)
        {
            csvRetries += c.attempts[i].empty() ? 0 : c.attempts[i].size() - 1;
            csvRetriedUes += c.attempts[i].size() > 1;
        }
        csvControl << "," << c.retry << "," << CsvNum(retrySeconds) << "," << maxRetry << ","
                   << csvRetriedUes << "," << csvRetries;
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
    // A UE whose call never started (blocked, or no call) has no flow: one line for it, in its
    // place, with where it is and why there is no call.
    uint32_t nextUe = 0;
    auto printUesWithoutFlow = [&](uint32_t before) {
        for (; nextUe < before; ++nextUe)
        {
            const uint32_t i = nextUe;
            if (std::any_of(flows.begin(), flows.end(), [i](const VoiceFlow& f) {
                    return f.ue == i;
                }))
            {
                continue;
            }
            std::cout << std::fixed << std::setw(4) << i << "  " << std::left << std::setw(3) << "-"
                      << std::right << std::setw(wSat) << addressOf(ueIfaces.GetAddress(i))
                      << "    " << std::left << std::setw(wFar) << addressOf(farAddress(i))
                      << std::right << std::setprecision(1) << std::setw(7) << ueNorthKm[i]
                      << std::setw(7) << ueEastKm[i] << "  " << whatHappened(i) << "\n";
        }
    };
    for (const VoiceFlow& f : flows)
    {
        const uint32_t i = f.ue;
        printUesWithoutFlow(i);
        nextUe = std::max(nextUe, i + 1);
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
    printUesWithoutFlow(nUes);
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
        const std::string ueDiscard =
            ueBufferAgeS > 0.0 ? " (a UE: over " + Num(ueBufferAgeS, 3) + " s)" : std::string();
        std::cout << "no conn.: the UE had no connection. discard: the sender did not take it, the "
                     "oldest packet in its queue had waited too long"
                  << ueDiscard << ".\n"
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
        "puschRepSinrDb,dlBoostMaxDb,dlPfdLimit,dlPowerRefDeg,ulPowerRefDeg,ulWideRbs,"
        "dlClosedLoop,dlSnrMinDb,dlMcsMin,ulClosedLoop,ulSnrHighDb,ulSnrLowDb,ulPowerMinDbm,"
        "closedLoopMs,closedLoopStepDb,closedLoopLanding,closedLoopAvg,errorModel,rngSeed,rngRun,"
        "ulPolarizationLossDb,puschMinBlockBytes,compactBsr,ulSinrAvgMs,beams,beamReuse,"
        "satellites,nUesPerBeam,beamSpacing,beamPruneDb,cells,beamEdgeDb,bsrOnDemand";
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
          << CsvNum(dlBoostMaxDb) << "," << CsvNum(dlPfdLimit) << "," << CsvNum(dlPowerRefDeg)
          << "," << CsvNum(ulPowerRefDeg) << "," << ulWideRbs << "," << dlClosedLoop << ","
          << CsvNum(dlSnrMinDb) << "," << dlMcsMin << "," << ulClosedLoop << ","
          << CsvNum(ulSnrHighDb) << "," << CsvNum(ulSnrLowDb) << "," << CsvNum(ulPowerMinDbm)
          << "," << CsvNum(closedLoopMs) << "," << CsvNum(closedLoopStepDb) << ","
          << CsvNum(closedLoopLanding) << "," << CsvNum(closedLoopAvg) << "," << errorModel
          << "," << RngSeedManager::GetSeed() << "," << RngSeedManager::GetRun() << ","
          << CsvNum(ulPolarizationLossDb) << "," << puschMinBlockBytes << ","
          << (NrMacShortBsrCe::IsCompact() ? 1 : 0) << "," << CsvNum(ulSinrAvgMs) << ","
          << beams << "," << beamReuse << "," << satellites << "," << nUesPerCell << ","
          << CsvNum(beamSpacing) << "," << CsvNum(beamPruneDb) << "," << nCells << ","
          << CsvNum(beamEdgeDb) << "," << (bsrOnDemand ? 1 : 0);
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
    // --retry: a call (try) of a flow: what the UE's connections of that try were given and did
    struct TryRow
    {
        CallControl::Attempt attempt;
        std::string rntis; // the RNTIs of its connections, ';' between
        Row row;           // all of the packets on those bearers
    };
    std::vector<std::vector<TryRow>> tryRows(flows.size());
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
            {
                const Cell& cellOfUe = g_scene.cells[g_scene.cellOf[f.ue]];
                o << "," << g_scene.cellOf[f.ue] << "," << cellOfUe.sat << "," << cellOfUe.beam
                  << "," << cellOfUe.colour;
            }
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
            if (g_cc.retry && !g_cc.attempts[i].empty())
            {
                // The bearers of each call: those of the connections from the one it was
                // admitted on to the one before the next call's (a radio link failure inside a
                // call makes more than one)
                const auto& tries = g_cc.attempts[i];
                for (std::size_t n = 0; n < tries.size(); ++n)
                {
                    TryRow t;
                    t.attempt = tries[n];
                    if (tries[n].at != Time::Max())
                    {
                        const std::size_t from = n == 0 ? 0 : tries[n].firstConnection;
                        std::size_t to = connections.size();
                        for (std::size_t m = n + 1; m < tries.size(); ++m)
                        {
                            if (tries[m].at != Time::Max())
                            {
                                to = tries[m].firstConnection;
                                break;
                            }
                        }
                        for (std::size_t j = from; j < to && j < connections.size(); ++j)
                        {
                            if (bearerOf[j] == 0)
                            {
                                continue;
                            }
                            const Row& br = bearers[bearerOf[j] - 1].second;
                            t.rntis += (t.rntis.empty() ? "" : ";") + std::to_string(connections[j].rnti);
                            t.row.c.onBearer += br.c.onBearer;
                            t.row.c.discarded += br.c.discarded;
                            t.row.c.radio += br.c.radio;
                            t.row.c.rohc += br.c.rohc;
                            t.row.inPdb += br.inPdb;
                            t.row.delayNs.insert(t.row.delayNs.end(), br.delayNs.begin(), br.delayNs.end());
                            t.row.kbps = std::max(t.row.kbps, br.kbps);
                        }
                        std::sort(t.row.delayNs.begin(), t.row.delayNs.end());
                    }
                    tryRows[k].push_back(std::move(t));
                }
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
                    if (const auto it =
                            g_dciByRnti.find(RntiKey(g_scene.cells[g_scene.cellOf[f.ue]].cellId, rnti));
                        it != g_dciByRnti.end())
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
                         "ends a connection"
                      << (g_cc.retry ? ", so does a release or a refusal (--retry: the UE then calls again, "
                                       "VOICE OF EACH CALL below)"
                                     : "")
                      << ", the next one sets up a new bearer (RNTI as in the ROHC tables). "
                         "The line of the flow adds them up.\n";
        }
        std::cout << notes.str();
    }

    // --- BEAMS: one line per cell (--beams, --satellites) --------------------------------------
    // The cells of the run (one beam of one satellite each) side by side: where they are, how
    // many UEs and calls, how the voice did, and the interference they are in. The full-load C/I
    // is geometric: every beam of the cell's colour sends on every RB at full power, so that
    // what a UE hears of another beam is that beam's pattern towards it (TR 38.811 6.4.1) less the
    // free-space loss of its own path; no noise, no fading, no load.
    std::ostringstream csvCells; // <csv>_cells.csv: a line per cell
    {
        const std::size_t nc = g_scene.cells.size();
        const Time tMid = callsStart + (callsEnd - callsStart) / 2;
        auto satAt = [&](std::size_t c, Time t) {
            return freezeGeometry ? satOrbits[g_scene.cells[c].sat].pos0 : orbit.At(t);
        };
        std::vector<CiBeam> ciBeams(nc);
        for (std::size_t c = 0; c < nc; ++c)
        {
            ciBeams[c] = {satAt(c, tMid), g_scene.cells[c].centre, g_scene.cells[c].colour};
        }
        struct PerCell
        {
            uint32_t ues{0};
            uint32_t blocked{0};
            uint32_t released{0};
            double ciSum{0.0};
            double ciMin{INFINITY};
            uint32_t ciN{0};
            double sinrSum{0.0};
            uint32_t sinrN{0};
            uint64_t onBearer{0};
            uint64_t inPdb{0};
            uint64_t delivered{0};
            uint64_t discarded{0};
            uint64_t lost{0};
        };
        std::vector<PerCell> pc(nc);
        for (uint32_t i = 0; i < nUes; ++i)
        {
            const std::size_t c = g_scene.cellOf[i];
            PerCell& p = pc[c];
            ++p.ues;
            p.blocked += g_cc.status[i] == CallControl::BLOCKED;
            p.released += g_cc.status[i] == CallControl::RELEASED;
            const double ci = FullLoadCiDb(ciBeams,
                                           c,
                                           g_scene.ues[i]->GetPosition(),
                                           g_scene.beamKa[0],
                                           g_scene.beamFloorDb);
            if (std::isfinite(ci))
            {
                p.ciSum += ci;
                p.ciMin = std::min(p.ciMin, ci);
                ++p.ciN;
            }
            if (g_dlSinrN[i] > 0)
            {
                p.sinrSum += g_dlSinrSum[i] / g_dlSinrN[i];
                ++p.sinrN;
            }
        }
        for (std::size_t k = 0; k < flows.size(); ++k)
        {
            PerCell& p = pc[g_scene.cellOf[flows[k].ue]];
            p.onBearer += satRows[k].c.onBearer;
            p.inPdb += satRows[k].inPdb;
            p.delivered += satRows[k].delayNs.size();
            p.discarded += satRows[k].c.discarded;
            p.lost += satRows[k].c.radio + satRows[k].c.rohc;
        }
        auto cellPcts = [&](const PerCell& p) {
            const uint64_t late = p.delivered - p.inPdb;
            return std::make_pair(p.onBearer ? 100.0 * (late + p.discarded) / p.onBearer : NAN,
                                  p.onBearer ? 100.0 * p.lost / p.onBearer : NAN);
        };
        auto ccePct = [&](std::size_t c) -> double {
            if (!pdcch)
            {
                return NAN;
            }
            const NrPdcch::Stats st = NrPdcch::Get(g_scene.cells[c].cellId)->GetStats();
            const uint64_t possible =
                st.slots * pdcchView->GetNumCce(pdcchView->GetMaxSymbols());
            return possible ? 100.0 * st.cceUsed / possible : NAN;
        };
        if (multiCell)
        {
            Frame("BEAMS",
                  std::to_string(nc) + " cells: " + std::to_string(beams) + " beam" +
                      (beams == 1 ? "" : "s") + " x " + std::to_string(satellites) + " satellite" +
                      (satellites == 1 ? "" : "s") + ", frequency reuse " +
                      std::to_string(beamReuse) + ", " + std::to_string(nUesPerCell) +
                      " UEs in each");
            std::cout << "  cell sat beam tier col    N[km]    E[km] UEs blk rel elev[deg]"
                         "  C/I[dB] mean   min  DL SINR[dB]  on bearer  over PDB[%]  PER[%]";
            if (pdcch)
            {
                std::cout << "  CCE[%]";
            }
            std::cout << "\n";
            PerCell total;
            double totalCiSum = 0.0;
            for (std::size_t c = 0; c < nc; ++c)
            {
                const Cell& cell = g_scene.cells[c];
                const PerCell& p = pc[c];
                const auto [overPdb, per] = cellPcts(p);
                std::cout << std::fixed << std::setw(6) << c << std::setw(4) << cell.sat
                          << std::setw(5) << cell.beam << std::setw(5) << cell.tier << std::setw(4)
                          << cell.colour << std::setprecision(1) << std::setw(9) << cell.northKm
                          << std::setw(9) << cell.eastKm << std::setw(4) << p.ues << std::setw(4)
                          << p.blocked << std::setw(4) << p.released << std::setw(10)
                          << ElevationDeg(satAt(c, tMid), cell.centre);
                if (p.ciN > 0)
                {
                    std::cout << std::setw(16) << p.ciSum / p.ciN << std::setw(6) << p.ciMin;
                }
                else
                {
                    std::cout << std::setw(16) << "-" << std::setw(6) << "-";
                }
                if (p.sinrN > 0)
                {
                    std::cout << std::setw(13) << 10.0 * std::log10(p.sinrSum / p.sinrN);
                }
                else
                {
                    std::cout << std::setw(13) << "-";
                }
                std::cout << std::setw(11) << p.onBearer << std::setprecision(2) << std::setw(13)
                          << overPdb << std::setw(8) << per;
                if (pdcch)
                {
                    std::cout << std::setprecision(1) << std::setw(8) << ccePct(c);
                }
                std::cout << "\n";
                total.ues += p.ues;
                total.blocked += p.blocked;
                total.released += p.released;
                total.onBearer += p.onBearer;
                total.inPdb += p.inPdb;
                total.delivered += p.delivered;
                total.discarded += p.discarded;
                total.lost += p.lost;
                totalCiSum += p.ciSum;
                total.ciN += p.ciN;
                total.ciMin = std::min(total.ciMin, p.ciMin);
                total.sinrSum += p.sinrSum;
                total.sinrN += p.sinrN;
            }
            const auto [totalOver, totalPer] = cellPcts(total);
            std::cout << "   all" << std::setw(18) << "" << std::setw(9) << "" << std::setw(4)
                      << total.ues << std::setw(4) << total.blocked << std::setw(4)
                      << total.released << std::setprecision(1) << std::setw(10) << "";
            if (total.ciN > 0)
            {
                std::cout << std::setw(16) << totalCiSum / total.ciN << std::setw(6) << total.ciMin;
            }
            else
            {
                std::cout << std::setw(16) << "-" << std::setw(6) << "-";
            }
            std::cout << std::setw(13)
                      << (total.sinrN > 0 ? 10.0 * std::log10(total.sinrSum / total.sinrN) : NAN)
                      << std::setw(11) << total.onBearer << std::setprecision(2) << std::setw(13)
                      << totalOver << std::setw(8) << totalPer << "\n";
            std::cout << "sat, beam, tier: the satellite, the place of the beam in its layout (0: the centre beam) and its ring. col:"
                         " frequency colour (--beamReuse).\n"
                      << "N, E: where the beam points, north and east of the beam centre of cell 0. elev: the satellite above that"
                         " place, at the middle of the calls.\n"
                      << "C/I: geometric, at full load, UEs of the cell: all beams of its colour on every RB at full power, the"
                         " beam pattern and free space only;\n"
                      << "no noise, no fading, no load. DL SINR: measured, the control symbols the UEs received (noise, the beams that"
                         " were sending, the fading):\n"
                      << "a voice load uses a few RBs of each beam, so it is above the C/I when the beams are not noise-limited."
                         " blk, rel: calls blocked or released.\n";
        }
        for (std::size_t c = 0; c < nc; ++c)
        {
            const Cell& cell = g_scene.cells[c];
            const PerCell& p = pc[c];
            const auto [overPdb, per] = cellPcts(p);
            csvCells << csvRun << "," << c << "," << cell.sat << "," << cell.beam << ","
                     << cell.tier << "," << cell.colour << "," << CsvNum(cell.northKm) << ","
                     << CsvNum(cell.eastKm) << "," << p.ues << "," << p.blocked << ","
                     << p.released << "," << CsvNum(ElevationDeg(satAt(c, tMid), cell.centre)) << ","
                     << CsvNum(p.ciN ? p.ciSum / p.ciN : NAN) << ","
                     << CsvNum(p.ciN ? p.ciMin : NAN) << ","
                     << CsvNum(p.sinrN ? 10.0 * std::log10(p.sinrSum / p.sinrN) : NAN) << ","
                     << p.onBearer << "," << p.inPdb << "," << p.delivered - p.inPdb << ","
                     << p.discarded << "," << p.lost << "," << CsvNum(overPdb) << ","
                     << CsvNum(per) << "," << CsvNum(ccePct(c)) << "\n";
        }
    }

    // --- --retry: the voice of each call of each UE, the mean of a UE's calls and of all --------
    // A UE whose call was released connects again and calls again (--retry): each call has a
    // bearer of its own (a new RNTI, the same address), held against 5QI 1 on its own. Here the
    // calls one by one, then the mean of each UE's calls and of all calls: every call counts
    // once, however long it lasted. (The table above adds the packets of a UE's calls up.)
    std::ostringstream csvTries; // <csv>_tries.csv
    if (g_cc.retry)
    {
        // The mean of some calls: those with packets on their bearer (a call that was blocked, or
        // had no bearers, has none and is left out)
        struct Mean
        {
            uint32_t calls{0};
            double onBearer{0};
            double inPdb{0};
            double late{0};
            double discard{0};
            double lost{0};
            double overPdb{0};
            double per{0};
            double delayMean{0};
            double p98{0};
            double max{0};
            double kbps{0};
            uint32_t withDelay{0};
            uint32_t missedPdb{0};
            uint32_t missedPer{0};
        };
        auto addMean = [&](Mean& m, const Row& r) {
            if (r.c.onBearer == 0)
            {
                return;
            }
            const uint64_t delivered = r.delayNs.size();
            const uint64_t late = delivered - r.inPdb;
            const uint64_t lost = r.c.radio + r.c.rohc;
            ++m.calls;
            m.onBearer += r.c.onBearer;
            m.inPdb += r.inPdb;
            m.late += late;
            m.discard += r.c.discarded;
            m.lost += lost;
            m.overPdb += 100.0 * (late + r.c.discarded) / r.c.onBearer;
            m.per += 100.0 * lost / r.c.onBearer;
            if (delivered > 0)
            {
                ++m.withDelay;
                m.delayMean += meanMs(r.delayNs);
                m.p98 += delayPct(r.delayNs, 98);
                m.max += r.delayNs.back() / 1e6;
            }
            m.kbps += r.kbps;
            const std::string missed = missedBy(r);
            m.missedPdb += missed.find("PDB") != std::string::npos;
            m.missedPer += missed.find("PER") != std::string::npos;
        };
        auto outcomeWord = [](const CallControl::Attempt& a) {
            return std::string(a.outcome == CallControl::RELEASED  ? "released"
                               : a.outcome == CallControl::BLOCKED ? "blocked"
                               : a.outcome == CallControl::WAITING ? "no bearers"
                               : a.outcome == CallControl::ENDED   ? "to the end"
                                                                   : "on at end");
        };
        auto seconds = [](Time t) { return t == Time::Max() ? std::string("-") : Num(t.GetSeconds(), 2); };
        // The columns after the first ones, as in qosLine
        const std::string columns =
            "  on bearer  in PDB  late  discard  lost  over PDB[%]  PER[%]  mean[ms]   98%[ms]   "
            "max[ms]  kbit/s  missed\n";
        const std::string tryHeader = "  UE  try  dir      RNTI  asked[s]  from[s]    to[s]  ended      " + columns;
        auto meanLine = [&](const std::string& first, const Mean& m) {
            std::cout << first << std::fixed;
            if (m.calls == 0)
            {
                std::cout << std::setw(11) << "-" << std::setw(8) << "-" << std::setw(6) << "-"
                          << std::setw(9) << "-" << std::setw(6) << "-" << std::setw(13) << "-"
                          << std::setw(8) << "-" << std::setw(10) << "-" << std::setw(10) << "-"
                          << std::setw(10) << "-" << std::setw(8) << "-" << "  -\n";
                return;
            }
            const double n = m.calls;
            const double nd = std::max<uint32_t>(m.withDelay, 1);
            std::cout << std::setprecision(1) << std::setw(11) << m.onBearer / n << std::setw(8)
                      << m.inPdb / n << std::setw(6) << m.late / n << std::setw(9) << m.discard / n
                      << std::setw(6) << m.lost / n << std::setprecision(2) << std::setw(13)
                      << m.overPdb / n << std::setw(8) << m.per / n << std::setprecision(3);
            if (m.withDelay == 0)
            {
                std::cout << std::setw(10) << "-" << std::setw(10) << "-" << std::setw(10) << "-";
            }
            else
            {
                std::cout << std::setw(10) << m.delayMean / nd << std::setw(10) << m.p98 / nd
                          << std::setw(10) << m.max / nd;
            }
            std::cout << std::setprecision(1) << std::setw(8) << m.kbps / n << "  ";
            std::string missed;
            if (m.missedPdb > 0)
            {
                missed += "PDB " + std::to_string(m.missedPdb);
            }
            if (m.missedPer > 0)
            {
                missed += (missed.empty() ? "" : ", ") + std::string("PER ") + std::to_string(m.missedPer);
            }
            std::cout << (missed.empty() ? "-" : missed) << " of " << m.calls << "\n";
        };
        // <csv>_tries.csv: row = try (one call), ue_mean (the mean of a UE's calls), all_mean,
        // first_mean (the calls of the plan), retry_mean (the retries)
        auto csvTry = [&](const std::string& rowKind,
                          const std::string& ue,
                          const std::string& address,
                          const std::string& dir,
                          const CallControl::Attempt* a,
                          const std::string& rntis,
                          const Row* r,
                          const Mean* m) {
            std::ostringstream o;
            o << csvRun << "," << ue << "," << address << "," << dir << "," << rowKind << ",";
            if (a)
            {
                o << a->number << "," << rntis << "," << CsvNum(a->began.GetSeconds()) << ","
                  << (a->at == Time::Max() ? "" : CsvNum(a->at.GetSeconds())) << ","
                  << (a->end == Time::Max() ? "" : CsvNum(a->end.GetSeconds())) << ","
                  << outcomeWord(*a) << ","
                  << (a->outcome == CallControl::BLOCKED ? g_resourceName[a->limitedBy] : "") << ","
                  << (a->outcome == CallControl::RELEASED ? CsvNum(a->onTimePct) : "") << ",";
            }
            else
            {
                o << ",,,,,,,,";
            }
            if (r)
            {
                Mean one;
                addMean(one, *r);
                o << one.calls << ",";
                if (one.calls > 0)
                {
                    o << r->c.onBearer << "," << r->inPdb << "," << one.late << "," << r->c.discarded
                      << "," << one.lost << "," << CsvNum(one.overPdb) << "," << CsvNum(one.per)
                      << "," << CsvNum(one.withDelay ? one.delayMean : NAN) << ","
                      << CsvNum(one.withDelay ? one.p98 : NAN) << ","
                      << CsvNum(one.withDelay ? one.max : NAN) << "," << CsvNum(one.kbps) << ","
                      << one.missedPdb << "," << one.missedPer;
                }
                else
                {
                    o << ",,,,,,,,,,,,";
                }
            }
            else if (m)
            {
                const double n = std::max<uint32_t>(m->calls, 1);
                const double nd = std::max<uint32_t>(m->withDelay, 1);
                o << m->calls << ",";
                if (m->calls > 0)
                {
                    o << CsvNum(m->onBearer / n) << "," << CsvNum(m->inPdb / n) << ","
                      << CsvNum(m->late / n) << "," << CsvNum(m->discard / n) << ","
                      << CsvNum(m->lost / n) << "," << CsvNum(m->overPdb / n) << ","
                      << CsvNum(m->per / n) << ","
                      << CsvNum(m->withDelay ? m->delayMean / nd : NAN) << ","
                      << CsvNum(m->withDelay ? m->p98 / nd : NAN) << ","
                      << CsvNum(m->withDelay ? m->max / nd : NAN) << "," << CsvNum(m->kbps / n) << ","
                      << m->missedPdb << "," << m->missedPer;
                }
                else
                {
                    o << ",,,,,,,,,,,,";
                }
            }
            csvTries << o.str() << "\n";
        };

        Frame("VOICE OF EACH CALL (--retry=1)",
              "each call (try) of a UE on the bearer of its own, against 5QI 1, and the mean of "
              "each UE's calls and of all");
        bool any = false;
        for (uint32_t i = 0; i < nUes; ++i)
        {
            any = any || g_cc.attempts[i].size() > 1;
        }
        if (!any)
        {
            std::cout << "  no UE called again: none was released (or none had time to try "
                         "again)\n";
        }
        else
        {
            std::cout << tryHeader;
            std::array<Mean, 2> allMean{};   // per direction: all calls
            std::array<Mean, 2> firstMean{}; // the calls of the plan
            std::array<Mean, 2> retryMean{}; // the retries
            // The RNTIs of the connections of try n of a UE (for a UE with no flow: it never sent
            // a packet, as every try of it was refused)
            auto triedRntis = [&](uint32_t ue, std::size_t n) {
                const auto& tries = g_cc.attempts[ue];
                std::string rntis;
                if (tries[n].at == Time::Max())
                {
                    return rntis;
                }
                const std::size_t from = n == 0 ? 0 : tries[n].firstConnection;
                std::size_t to = g_connections[ue].size();
                for (std::size_t m = n + 1; m < tries.size(); ++m)
                {
                    if (tries[m].at != Time::Max())
                    {
                        to = tries[m].firstConnection;
                        break;
                    }
                }
                for (std::size_t j = from; j < to && j < g_connections[ue].size(); ++j)
                {
                    rntis += (rntis.empty() ? "" : ";") + std::to_string(g_connections[ue][j].rnti);
                }
                return rntis;
            };
            // The flows of one UE (dl, ul) are next to each other
            std::vector<std::pair<std::size_t, std::size_t>> flowsOf(nUes, {0, 0});
            for (std::size_t k = 0; k < flows.size();)
            {
                std::size_t end = k;
                while (end < flows.size() && flows[end].ue == flows[k].ue)
                {
                    ++end;
                }
                flowsOf[flows[k].ue] = {k, end};
                k = end;
            }
            for (uint32_t ue = 0; ue < nUes; ++ue)
            {
                const auto [k, end] = flowsOf[ue];
                if (k == end)
                {
                    // No packet of this UE reached a bearer: its tries, with nothing to count
                    for (std::size_t n = 0; n < g_cc.attempts[ue].size(); ++n)
                    {
                        const CallControl::Attempt& a = g_cc.attempts[ue][n];
                        const std::string rntis = triedRntis(ue, n);
                        std::ostringstream first;
                        first << std::setw(4) << ue << std::setw(5) << a.number << "  " << std::left
                              << std::setw(3) << "-" << std::right << std::setw(10)
                              << (rntis.empty() ? "-" : rntis) << std::setw(10)
                              << seconds(a.began) << std::setw(9) << seconds(a.at) << std::setw(9)
                              << seconds(a.end) << "  " << std::left << std::setw(11)
                              << outcomeWord(a) << std::right;
                        meanLine(first.str(), Mean());
                        const Row none;
                        csvTry("try", std::to_string(ue), addressOf(ueIfaces.GetAddress(ue)), "-",
                               &a, rntis, &none, nullptr);
                    }
                    continue;
                }
                const std::size_t nTries = tryRows[k].size();
                for (std::size_t n = 0; n < nTries; ++n)
                {
                    for (std::size_t m = k; m < end; ++m)
                    {
                        const VoiceFlow& f = flows[m];
                        const TryRow& t = tryRows[m][n];
                        const CallControl::Attempt& a = t.attempt;
                        std::ostringstream first;
                        first << std::setw(4) << ue << std::setw(5) << a.number << "  " << std::left
                              << std::setw(3) << dirName(f.dir) << std::right << std::setw(10)
                              << (t.rntis.empty() ? "-" : t.rntis) << std::setw(10)
                              << seconds(a.began) << std::setw(9) << seconds(a.at) << std::setw(9)
                              << seconds(a.end) << "  " << std::left << std::setw(11)
                              << outcomeWord(a) << std::right;
                        if (t.row.c.onBearer == 0)
                        {
                            Mean none;
                            meanLine(first.str(), none);
                        }
                        else
                        {
                            qosLine(first.str(), t.row, missedBy(t.row));
                        }
                        csvTry("try", std::to_string(ue), addressOf(ueIfaces.GetAddress(ue)),
                               dirName(f.dir), &a, t.rntis, &t.row, nullptr);
                        const int d = f.dir == DOWN ? 0 : 1;
                        addMean(allMean[d], t.row);
                        addMean(n == 0 ? firstMean[d] : retryMean[d], t.row);
                    }
                }
                if (nTries > 1)
                {
                    for (std::size_t m = k; m < end; ++m)
                    {
                        Mean ueMean;
                        for (const TryRow& t : tryRows[m])
                        {
                            addMean(ueMean, t.row);
                        }
                        std::ostringstream first;
                        first << std::setw(4) << ue << std::setw(5) << "mean" << "  " << std::left
                              << std::setw(3) << dirName(flows[m].dir) << std::right
                              << std::setw(10) << "" << std::setw(10) << "" << std::setw(9) << ""
                              << std::setw(9) << "" << "  " << std::left << std::setw(11)
                              << (std::to_string(ueMean.calls) + " calls") << std::right;
                        meanLine(first.str(), ueMean);
                        csvTry("ue_mean", std::to_string(ue), addressOf(ueIfaces.GetAddress(ue)),
                               dirName(flows[m].dir), nullptr, "", nullptr, &ueMean);
                    }
                }
            }
            // Everything the UEs did: all calls, then those of the plan and the retries apart
            std::cout << "\n";
            const char* const names[3] = {"all calls", "first calls", "retries"};
            for (int which = 0; which < 3; ++which)
            {
                for (int d = 0; d < 2; ++d)
                {
                    if (!(sendDown && sendUp) && ((d == 0) != sendDown))
                    {
                        continue;
                    }
                    const Mean& m = which == 0 ? allMean[d] : which == 1 ? firstMean[d] : retryMean[d];
                    std::ostringstream first;
                    first << std::setw(4) << "all" << std::setw(5) << "mean" << "  " << std::left
                          << std::setw(3) << (d == 0 ? "dl" : "ul") << std::right << std::setw(10)
                          << "" << std::setw(10) << "" << std::setw(9) << "" << std::setw(9) << ""
                          << "  " << std::left << std::setw(11) << names[which] << std::right;
                    meanLine(first.str(), m);
                    csvTry(which == 0 ? "all_mean" : which == 1 ? "first_mean" : "retry_mean",
                           "all", "", d == 0 ? "dl" : "ul", nullptr, "", nullptr, &m);
                }
            }
            std::cout
                << "try: 1 is the call of the plan, 2 and more are retries. asked: when the call was due, or the UE "
                   "asked for a connection;\nfrom: its bearers were there and the gNB admitted or blocked it; "
                   "to: released, ended, or refused. RNTI: of its connection (several after a radio\n"
                   "link failure inside the call). The columns are those of the table above, "
                   "for the packets on the bearer of the call.\n"
                   "mean: each call counts once, however long it lasted; counts, percentages, delays and "
                   "kbit/s are the mean of the calls' own, max the mean of\ntheir maxima. A call "
                   "with no packet on its bearer (blocked, or no bearers before the end) is left out. missed: how many of "
                   "the calls missed PDB or PER.\n";
        }
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
              std::string(multiCell ? "cell 0, the centre beam, " : "") + "t = " +
                  Num(callsStart.GetSeconds(), 2) + " s to " + Num(callsEnd.GetSeconds(), 2) +
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
        std::cout << "\npurpose                  bits level    DCIs    CCEs  CCEs[%]  blocked  "
                     "blocked[%]\n";
        auto purposeLine = [&](const std::string& name,
                               const std::string& bits,
                               const std::string& level,
                               uint64_t n,
                               uint64_t cces,
                               uint64_t stopped) {
            std::cout << std::left << std::setw(25) << name << std::right << std::setw(5) << bits
                      << std::setw(6) << level << std::setw(8) << n << std::setw(8) << cces
                      << std::setw(9) << percent(cces, ccePossible) << std::setw(9) << stopped
                      << std::setw(12) << percent(stopped, n + stopped) << "\n";
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
            const bool common = purpose == NrPdcch::SIB1 || purpose == NrPdcch::OTHER_SI ||
                                purpose == NrPdcch::PAGING || purpose == NrPdcch::RAR ||
                                purpose == NrPdcch::TPC_GROUP;
            purposeLine(NrPdcch::GetPurposeName(purpose),
                        std::to_string(pdcchView->GetDciBits(purpose)),
                        common ? std::to_string(+pdcchView->GetCommonAl(purpose)) : "-",
                        stats.dci[i],
                        stats.cce[i],
                        stats.blocked[i]);
        }
        purposeLine("all", "", "", dcis, stats.cceUsed, blocked);
        std::cout
            << "CCEs[%]: of the " << ccePossible
            << " CCEs possible. blocked: not sent, no "
               "candidate had all its CCEs free; its UE was not\n"
            << "scheduled in that slot. bits: without the 24-bit CRC. level: of the cell's DCIs "
               "of the common search space, fixed;\n"
            << "the others take the lowest the UE's SINR allows (the next table).\n";
        {
            // What a level of the common search space needs at the size of its DCI, and the
            // lowest SINR of the control symbols any UE received in these calls
            double weakest = INFINITY;
            for (const auto& u : g_dlSnr)
            {
                if (u.n > 0)
                {
                    weakest = std::min(weakest, u.minDb);
                }
            }
            if (std::isfinite(weakest))
            {
                const double needTarget =
                    AttributeDefault<DoubleValue>("ns3::NrPdcch", "TargetBler");
                const double needMargin =
                    AttributeDefault<DoubleValue>("ns3::NrPdcch", "SinrMarginDb");
                std::cout
                    << "\nThe cell's DCIs of the common search space reach a UE whose SINR is at "
                       "least what their level needs for "
                    << Num(100.0 * needTarget, 2) << " % lost.\n"
                    << "The lowest SINR of the control symbols of any UE in these calls: "
                    << Num(weakest, 2) << " dB; spare = that less what the level needs.\n"
                    << "  DCI         bits  level  needs[dB]  spare[dB]\n";
                for (const auto& c : kCommonDcis)
                {
                    if (stats.dci[c.purpose] == 0)
                    {
                        continue;
                    }
                    const uint32_t bits = pdcchView->GetDciBits(c.purpose);
                    const uint8_t al = pdcchView->GetCommonAl(c.purpose);
                    const double need =
                        pdcchView->GetSinrForBler(al, needTarget, bits) + needMargin;
                    std::cout << "  " << std::left << std::setw(10) << c.name << std::right
                              << std::setw(5) << bits << std::setw(7) << +al << std::setw(11)
                              << Num(need, 2) << std::setw(11) << Num(weakest - need, 2)
                              << (weakest < need ? "  <- the weakest UE would not decode it" : "")
                              << "\n";
                }
            }
        }

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
        if (AttributeDefault<DoubleValue>("ns3::NrPdcch", "BoostMaxDb") > 0.0)
        {
            // The energy of the CCEs a slot leaves unused goes to those it uses
            std::cout << "PDCCH power boost (--ns3::NrPdcch::BoostMaxDb="
                      << AttributeDefault<DoubleValue>("ns3::NrPdcch", "BoostMaxDb")
                      << " dB at most, and what the PFD limit allows): " << stats.boosted
                      << " of the " << dcis << " DCIs had more than 0.05 dB, "
                      << std::fixed << std::setprecision(2)
                      << (dcis > 0 ? stats.boostSumDb / static_cast<double>(dcis) : 0.0)
                      << " dB on average; the level was chosen without it\n"
                      << std::defaultfloat << std::setprecision(6);
        }
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
                const auto purpose = static_cast<NrPdcch::Purpose>(i);
                addN(std::string(purposeName[i]) + "Bits", pdcchView->GetDciBits(purpose));
                const bool common = purpose == NrPdcch::SIB1 || purpose == NrPdcch::OTHER_SI ||
                                    purpose == NrPdcch::PAGING || purpose == NrPdcch::RAR ||
                                    purpose == NrPdcch::TPC_GROUP;
                addN(std::string(purposeName[i]) + "Level",
                     common ? pdcchView->GetCommonAl(purpose) : 0);
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
            addN("boostedDcis", stats.boosted);
            add("boostMeanDb", CsvNum(dcis > 0 ? stats.boostSumDb / static_cast<double>(dcis) : 0.0));
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
        std::cout << "  UE      address  elevation[deg]  range[km]  delay[ms]  st\n";
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
        std::cout << "  st\n";
    }
    // (every UE: one whose call was blocked is seen all the same, from the moment its call was
    // to start, and one that has no call has no view at all)
    std::vector<uint32_t> callers;
    for (uint32_t i = 0; i < nUes; ++i)
    {
        callers.push_back(i);
    }
    for (const uint32_t i : callers)
    {
        const CallView& view = g_view[i];
        std::cout << std::fixed << std::setw(4) << i << "  " << std::setw(11)
                  << addressOf(ueIfaces.GetAddress(i));
        if (view.elevationDeg.n == 0)
        {
            // no call, so nothing was looked at
            std::cout << std::string(freezeGeometry ? 38 : 117, ' ') << statusMark(i) << "\n";
            continue;
        }
        if (freezeGeometry)
        {
            std::cout << std::setprecision(2) << std::setw(16) << view.elevationDeg.first
                      << std::setw(11) << view.rangeKm.first << std::setprecision(3)
                      << std::setw(11) << view.delayMs.first << "  " << statusMark(i) << "\n";
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
        std::cout << "  " << statusMark(i) << "\n";
    }
    if (!freezeGeometry)
    {
        std::cout << "from, to: when the call started and ended. elevation: - before the satellite "
                     "is overhead, + after; low, high: of the call.\n";
    }
    std::cout << "delay: one way, " << path << ", propagation only.\n"
              << "st: B = the call was blocked (the UE is seen from the moment it was to start), "
                 "R = released,\n"
              << (retry ? "r = released or blocked, then called again (--retry), " : "")
              << "- = no call: the satellite is not " << minCallElevationText
              << " deg above the UE during UE 0's call.\n";

    // --- The SNR of each UE's two links during its call, by the elevation of the satellite -------
    std::ostringstream csvSnr; // <csv>_snr.csv: one line per UE and elevation step
    std::ostringstream csvDlPower; // <csv>_dlpower.csv: the same for the power of a downlink RB
    const bool dlPowerShown = dlPfdLimit != 0.0 || dlBoostMaxDb > 0.0 || dlPowerRefDeg > 0.0;
    {
        const bool flying = !freezeGeometry;
        // the elevation steps any UE was measured at, in the order of the pass
        std::set<int> steps;
        for (const uint32_t i : callers)
        {
            for (const auto& [step, s] : g_snrSteps[i])
            {
                if (s.dl.n > 0 || s.ulRx.n > 0 || s.power.n > 0 || s.dlData.n > 0)
                {
                    steps.insert(step);
                }
            }
        }
        auto group = [](const std::string& label, std::size_t width) {
            const std::size_t dashes = width > label.size() + 3 ? width - label.size() - 3 : 0;
            return " " + std::string(dashes / 2, '-') + " " + label + " " +
                   std::string(dashes - dashes / 2, '-');
        };
        // One table: a line for each UE, the mean SNR of the call at each elevation step, then
        // the lowest, mean and highest SNR of the whole call
        // A number of a cell: no minus sign in front of a zero
        auto cell = [](double v, int decimals) {
            std::string s = Num(v, decimals);
            if (s.find_first_not_of("-0.") == std::string::npos)
            {
                s.erase(std::remove(s.begin(), s.end(), '-'), s.end());
            }
            return s;
        };
        // kind: 0 downlink SNR, 1 uplink SNR, 2 power of an RB of the downlink allocations,
        // 3 SNR of the downlink data
        auto table = [&](int kind) {
            const std::size_t minRegion = 19; // under the title "elevation [deg]"
            std::size_t addressWidth = 7;
            for (const uint32_t i : callers)
            {
                addressWidth = std::max(addressWidth, addressOf(ueIfaces.GetAddress(i)).size());
            }
            // UE, address; min, mean, max; st
            const std::size_t fixedWidth = 3 + 1 + addressWidth + 21 + 3;
            auto stats = [&](uint32_t ue, int step) -> const SnrStats* {
                const auto it = g_snrSteps[ue].find(step);
                if (it == g_snrSteps[ue].end())
                {
                    return nullptr;
                }
                return kind == 1   ? &it->second.ulRx
                       : kind == 2 ? &it->second.power
                       : kind == 3 ? &it->second.dlData
                                   : &it->second.dl;
            };
            // as many decimals as fit in the line; a column is as wide as its widest value
            int decimals = 2;
            std::map<int, std::size_t> width;
            std::size_t cellsWidth = 0;
            for (;; --decimals)
            {
                cellsWidth = 0;
                for (const int step : steps)
                {
                    std::size_t w = StepLabel(step, flying).size();
                    for (const uint32_t i : callers)
                    {
                        const SnrStats* st = stats(i, step);
                        if (st && st->n > 0)
                        {
                            w = std::max(w, cell(st->MeanDb(), decimals).size());
                        }
                    }
                    width[step] = w + 1;
                    cellsWidth += w + 1;
                }
                if (decimals == 0 || fixedWidth + std::max(cellsWidth, minRegion) <= g_frameWidth)
                {
                    break;
                }
            }
            const std::size_t region = std::max(cellsWidth, minRegion);
            const std::string pad(region - cellsWidth, ' ');

            std::cout << std::string(3 + 1 + addressWidth, ' ') << group("elevation [deg]", region)
                      << group("whole pass", 21) << "\n"
                      << std::setw(3) << "UE" << " " << std::setw(static_cast<int>(addressWidth))
                      << "address" << pad;
            for (const int step : steps)
            {
                std::cout << std::setw(static_cast<int>(width[step])) << StepLabel(step, flying);
            }
            std::cout << std::setw(7) << "min" << std::setw(7) << "mean" << std::setw(7) << "max"
                      << "  st\n";
            for (const uint32_t i : callers)
            {
                const SnrStats& total = kind == 1   ? g_ulRxSnr[i]
                                        : kind == 2 ? g_dlRbPower[i]
                                        : kind == 3 ? g_dlDataSnr[i]
                                                    : g_dlSnr[i];
                std::cout << std::setw(3) << i << " " << std::setw(static_cast<int>(addressWidth))
                          << addressOf(ueIfaces.GetAddress(i)) << pad;
                for (const int step : steps)
                {
                    const SnrStats* st = stats(i, step);
                    std::cout << std::setw(static_cast<int>(width[step]))
                              << (st && st->n > 0 ? cell(st->MeanDb(), decimals) : "-");
                }
                for (const double v : {total.minDb, total.MeanDb(), total.maxDb})
                {
                    std::cout << std::setw(7) << (total.n > 0 ? cell(v, 2) : "-");
                }
                std::cout << "  " << statusMark(i) << "\n";
            }
            if (kind == 2 && dlPfdLimit != 0.0)
            {
                // what the PFD limit allows an RB with the satellite at each elevation
                std::cout << std::setw(3) << "" << " " << std::setw(static_cast<int>(addressWidth))
                          << "limit" << pad;
                for (const int step : steps)
                {
                    std::cout << std::setw(static_cast<int>(width[step]))
                              << cell(PfdRbLimitDbm(dlPfdLimit,
                                                    dlPfdMarginDb,
                                                    slantM(step > 90 ? 180 - step : step),
                                                    satAntennaGainDb,
                                                    g_scene.dlRbHz),
                                      decimals);
                }
                std::cout << "\n";
            }
            std::cout << (flying
                              ? "elevation [deg]: of the satellite above the UE, to the nearest 10 "
                                "deg; - while it comes closer to the UE, none once it\nmoves "
                                "away; 90 is over the UE. "
                              : "elevation [deg]: of the satellite above the UE, to the nearest 10 "
                                "deg (10 also takes the lower ones).\n")
                      << "A column is the mean " << (kind == 2 ? "power of an RB" : "SNR")
                      << " of the call at that elevation (- : none).\n"
                         "whole pass: the lowest, mean and highest "
                      << (kind == 2 ? "power of an RB" : "SNR")
                      << " of the UE's call. mean: of the dB values.\n";
            if (decimals < 2)
            {
                std::cout << "The columns of the elevations have "
                          << (decimals == 1 ? "1 decimal" : "no decimals")
                          << ", so that they fit the line.\n";
            }
        };

        const std::string title = "SNR OF EACH UE DURING ITS CALL: ";
        const std::string interferenceText =
            multiCell ? "other beams: interference, SINR = signal / (noise + interference); in dB"
                      : "one beam, no interference: SINR = SNR; in dB";
        Frame(title + "DOWNLINK", interferenceText);
        table(0);
        std::cout << "downlink: the UE's, on the control symbols of each slot that has some (they "
                     "span all RBs of the carrier). st: as above.\n";
        if (dlPowerShown)
        {
            Frame(title + "DOWNLINK DATA",
                  "on the RBs scheduled for the UE; " + interferenceText);
            table(3);
            std::cout << "downlink data: the UE's, on each reception of its data (an allocation, "
                         "HARQ retransmissions included),\naveraged over its RBs. st: as above.\n";
            Frame("POWER OF AN RB OF EACH UE DURING ITS CALL: DOWNLINK",
                  "dBm into the antenna; one value per allocation of the UE's data");
            table(2);
            std::cout << "power: of each RB of a downlink allocation of the UE (data), as the "
                         "satellite sent it:\nits share of TxPower, "
                      << (dlBoostMaxDb > 0.0 || dlPfdLimit != 0.0
                              ? "more the fewer RBs are in use in the symbol, up to the limit"
                              : "less with --dlPowerRefDeg")
                      << (dlPfdLimit != 0.0 ? ".\nlimit: what --dlPfdLimit allows an RB at that "
                                              "elevation (the satellite's power may not reach it)"
                                            : "")
                      << ". st: as above.\n";
        }

        Frame(title + "UPLINK",
              "on the RBs scheduled; " + interferenceText);
        table(1);
        std::cout << "uplink: the gNB's, on each PUSCH transmission of the UE (a repetition is "
                     "one), over the RBs scheduled for it,\nas they came: not scaled to a "
                     "number of RBs. st: as above.\n";

        for (const uint32_t i : callers)
        {
            for (const auto& [step, s] : g_snrSteps[i])
            {
                if (s.dl.n == 0 && s.ul.n == 0)
                {
                    continue;
                }
                csvSnr << csvRun << "," << i << "," << addressOf(ueIfaces.GetAddress(i)) << ","
                       << StepLabel(step, flying) << "," << s.dl.n << ","
                       << CsvNum(s.dl.n > 0 ? s.dl.minDb : NAN) << "," << CsvNum(s.dl.MeanDb())
                       << "," << CsvNum(s.dl.n > 0 ? s.dl.maxDb : NAN) << "," << s.ul.n << ","
                       << CsvNum(s.ul.n > 0 ? s.ul.minDb : NAN) << "," << CsvNum(s.ul.MeanDb())
                       << "," << CsvNum(s.ul.n > 0 ? s.ul.maxDb : NAN) << ","
                       << CsvNum(s.ulRx.n > 0 ? s.ulRx.minDb : NAN) << ","
                       << CsvNum(s.ulRx.MeanDb()) << ","
                       << CsvNum(s.ulRx.n > 0 ? s.ulRx.maxDb : NAN) << "\n";
                if (dlPowerShown && (s.power.n > 0 || s.dlData.n > 0))
                {
                    csvDlPower << csvRun << "," << i << "," << addressOf(ueIfaces.GetAddress(i))
                               << "," << StepLabel(step, flying) << "," << s.power.n << ","
                               << CsvNum(s.power.n > 0 ? s.power.minDb : NAN) << ","
                               << CsvNum(s.power.MeanDb()) << ","
                               << CsvNum(s.power.n > 0 ? s.power.maxDb : NAN) << "," << s.dlData.n
                               << "," << CsvNum(s.dlData.n > 0 ? s.dlData.minDb : NAN) << ","
                               << CsvNum(s.dlData.MeanDb()) << ","
                               << CsvNum(s.dlData.n > 0 ? s.dlData.maxDb : NAN) << "\n";
                }
            }
        }
    }

    // --- Wide uplink grants: the PUSCH slots of each UE by the RBs they have ---------------------
    std::ostringstream csvUlGrants; // <csv>_ulgrants.csv: a line per UE
    if (ulWideRbs > 0)
    {
        Frame("WIDE UPLINK GRANTS",
              std::to_string(ulGrantRbs) + " RB, or " + std::to_string(ulWideRbs) +
                  " when the UE's SINR on one RB is " + Num(ulWideSinrDb, 1) +
                  " dB or more and the slot has room (<= " + Num(ulWideLoadPct, 0) + " % load)");
        std::size_t addressWidth = 7;
        for (const uint32_t i : callers)
        {
            addressWidth = std::max(addressWidth, addressOf(ueIfaces.GetAddress(i)).size());
        }
        std::cout << std::setw(3) << "UE" << " " << std::setw(static_cast<int>(addressWidth))
                  << "address" << std::setw(9) << "slots" << std::setw(9)
                  << (std::to_string(ulGrantRbs) + " RB") << std::setw(9)
                  << (std::to_string(ulWideRbs) + " RB") << std::setw(9) << "other" << std::setw(10)
                  << "RB-slots" << std::setw(9) << "wide[%]" << "  st\n";
        UlGrantStats all;
        for (const uint32_t i : callers)
        {
            const UlGrantStats& g = g_ulGrants[i];
            const uint64_t slots = g.narrow + g.wide + g.other;
            all.narrow += g.narrow;
            all.wide += g.wide;
            all.other += g.other;
            all.rbSlots += g.rbSlots;
            std::cout << std::setw(3) << i << " " << std::setw(static_cast<int>(addressWidth))
                      << addressOf(ueIfaces.GetAddress(i)) << std::setw(9) << slots << std::setw(9)
                      << g.narrow << std::setw(9) << g.wide << std::setw(9) << g.other
                      << std::setw(10) << g.rbSlots << std::setw(9)
                      << (slots > 0 ? Num(100.0 * g.wide / slots, 1) : "-") << "  " << statusMark(i)
                      << "\n";
            csvUlGrants << csvRun << "," << i << "," << addressOf(ueIfaces.GetAddress(i)) << ","
                        << slots << "," << g.narrow << "," << g.wide << "," << g.other << ","
                        << g.rbSlots << "\n";
        }
        const uint64_t allSlots = all.narrow + all.wide + all.other;
        std::cout << std::setw(3) << "all" << " " << std::setw(static_cast<int>(addressWidth)) << ""
                  << std::setw(9) << allSlots << std::setw(9) << all.narrow << std::setw(9)
                  << all.wide << std::setw(9) << all.other << std::setw(10) << all.rbSlots
                  << std::setw(9) << (allSlots > 0 ? Num(100.0 * all.wide / allSlots, 1) : "-")
                  << "\n"
                  << "slots: the PUSCH slots the gNB received from the UE during its call, "
                     "repetitions and retransmissions included.\n"
                     "RB-slots: their RBs added up. other: slots with another number of RBs, "
                     "as the first grant of a UE the gNB has not measured yet.\n";
    }

    // --- Closed loop on the power: what each UE got, and what the loops saved --------------------
    std::ostringstream csvLoop; // <csv>_loop.csv: a line per UE
    const bool loopShown = dlClosedLoop != 0 || ulClosedLoop;
    if (loopShown)
    {
        auto pct = [](uint64_t part, uint64_t all) {
            return all > 0 ? Num(100.0 * part / all, 1) : std::string("-");
        };
        auto mean1 = [](const SnrStats& st, int decimals) {
            return st.n > 0 ? Num(st.MeanDb(), decimals) : std::string("-");
        };
        auto savedPct = [](double used, double free) {
            return free > 0.0 ? Num(100.0 * (1.0 - used / free), 1) : std::string("-");
        };
        auto addStats = [](SnrStats& to, const SnrStats& from) {
            if (from.n > 0)
            {
                to.minDb = std::min(to.minDb, from.minDb);
                to.maxDb = std::max(to.maxDb, from.maxDb);
                to.sumDb += from.sumDb;
                to.n += from.n;
            }
        };
        auto mw2dbm = [](double mw, uint64_t n) {
            return n > 0 && mw > 0.0 ? Num(10.0 * std::log10(mw / n), 2) : std::string("-");
        };
        // A table: two header lines, a line for each UE (UE, address, the cells, status mark), and
        // the line of all of them. A column is as wide as its widest cell.
        auto printTable = [&](const std::vector<std::string>& head1,
                              const std::vector<std::string>& head2,
                              const std::vector<std::vector<std::string>>& rows,
                              const std::vector<std::string>& total) {
            std::size_t addressWidth = 7;
            for (const uint32_t i : callers)
            {
                addressWidth = std::max(addressWidth, addressOf(ueIfaces.GetAddress(i)).size());
            }
            std::vector<std::size_t> width(head1.size());
            for (std::size_t c = 0; c < head1.size(); ++c)
            {
                width[c] = std::max(head1[c].size(), head2[c].size());
                for (const auto& r : rows)
                {
                    width[c] = std::max(width[c], r[c].size());
                }
                width[c] = std::max(width[c], total[c].size()) + 1;
            }
            for (const auto* head : {&head1, &head2})
            {
                std::cout << std::setw(3) << (head == &head1 ? "UE" : "") << " "
                          << std::setw(static_cast<int>(addressWidth))
                          << (head == &head1 ? "address" : "");
                for (std::size_t c = 0; c < head->size(); ++c)
                {
                    std::cout << std::setw(static_cast<int>(width[c])) << (*head)[c];
                }
                std::cout << (head == &head1 ? "  st" : "") << "\n";
            }
            for (std::size_t r = 0; r < rows.size(); ++r)
            {
                const uint32_t i = callers[r];
                std::cout << std::setw(3) << i << " " << std::setw(static_cast<int>(addressWidth))
                          << addressOf(ueIfaces.GetAddress(i));
                for (std::size_t c = 0; c < rows[r].size(); ++c)
                {
                    std::cout << std::setw(static_cast<int>(width[c])) << rows[r][c];
                }
                std::cout << "  " << statusMark(i) << "\n";
            }
            std::cout << std::setw(3) << "all" << " " << std::setw(static_cast<int>(addressWidth))
                      << "";
            for (std::size_t c = 0; c < total.size(); ++c)
            {
                std::cout << std::setw(static_cast<int>(width[c])) << total[c];
            }
            std::cout << "\n";
        };
        const std::string every =
            Num(closedLoopMs, 0) + " ms, steps of " + Num(closedLoopStepDb, 1) + " dB";
        LoopTally sum; // all UEs
        for (const uint32_t i : callers)
        {
            const LoopTally& t = g_loopTally[i];
            sum.dlRx += t.dlRx;
            sum.dlSnrBelow += t.dlSnrBelow;
            sum.dlMcsBelow += t.dlMcsBelow;
            sum.dlMwUsed += t.dlMwUsed;
            sum.dlMwFree += t.dlMwFree;
            sum.dlWin += t.dlWin;
            sum.dlWinBelow += t.dlWinBelow;
            sum.dlUp += t.dlUp;
            sum.dlDown += t.dlDown;
            sum.dlHold += t.dlHold;
            sum.dlAtMax += t.dlAtMax;
            addStats(sum.dlSnr, t.dlSnr);
            addStats(sum.dlMcs, t.dlMcs);
            addStats(sum.dlBoost, t.dlBoost);
            addStats(sum.dlBoostFree, t.dlBoostFree);
            sum.ulRx += t.ulRx;
            sum.ulBelow += t.ulBelow;
            sum.ulInBand += t.ulInBand;
            sum.ulAbove += t.ulAbove;
            sum.ulMw += t.ulMw;
            sum.ulMwFree += t.ulMwFree;
            sum.ulWin += t.ulWin;
            sum.ulWinInBand += t.ulWinInBand;
            sum.ulUp += t.ulUp;
            sum.ulDown += t.ulDown;
            sum.ulHold += t.ulHold;
            sum.ulAtMin += t.ulAtMin;
            addStats(sum.ulSnr, t.ulSnr);
            addStats(sum.ulPower, t.ulPower);
        }
        if (dlClosedLoop != 0)
        {
            std::string target;
            if ((dlClosedLoop & 1) != 0)
            {
                target = "SNR >= " + Num(dlSnrMinDb, 1) + " dB";
            }
            if ((dlClosedLoop & 2) != 0)
            {
                target += std::string(target.empty() ? "" : " and ") + "MCS >= " +
                          std::to_string(dlMcsMin);
            }
            Frame("CLOSED LOOP: DOWNLINK POWER",
                  target + "; boost 0 to " + Num(g_loop.dlMaxDb, 1) + " dB over the RB's share; " +
                      every);
            const bool bySnr = (dlClosedLoop & 1) != 0;
            const bool byMcs = (dlClosedLoop & 2) != 0;
            auto row = [&](const LoopTally& t) {
                return std::vector<std::string>{std::to_string(t.dlRx),
                                                mean1(t.dlSnr, 2),
                                                bySnr ? pct(t.dlSnrBelow, t.dlRx) : "-",
                                                pct(t.dlWinBelow, t.dlWin),
                                                mean1(t.dlMcs, 1),
                                                byMcs ? pct(t.dlMcsBelow, t.dlRx) : "-",
                                                mean1(t.dlBoost, 2),
                                                t.dlBoost.n > 0 ? Num(t.dlBoost.maxDb, 2) : "-",
                                                mean1(t.dlBoostFree, 2),
                                                savedPct(t.dlMwUsed, t.dlMwFree),
                                                std::to_string(t.dlUp),
                                                std::to_string(t.dlDown),
                                                std::to_string(t.dlHold),
                                                std::to_string(t.dlAtMax)};
            };
            std::vector<std::vector<std::string>> rows;
            for (const uint32_t i : callers)
            {
                rows.push_back(row(g_loopTally[i]));
            }
            printTable({"recept.", "SNR", "SNR<", "win<", "MCS", "MCS<", "boost", "boost", "w/o loop",
                        "saved", "up", "down", "hold", "top"},
                       {"", "[dB]", "[%]", "[%]", "", "[%]", "mean[dB]", "max[dB]", "mean[dB]",
                        "[%]", "", "", "", ""},
                       rows,
                       row(sum));
            std::cout
                << "recept.: receptions of the UE's data during its call (a HARQ retransmission is "
                   "one). SNR: their mean, dB. SNR<, MCS<: the share of them under the SNR\n"
                   "target, and at an MCS under the MCS target (the SNR of one reception scatters "
                   "with the fading; the loop acts on its mean over a period). win<: the share of\n"
                   "the periods whose mean SNR was under what the UE needs. MCS: the scheduler's, "
                   "for the block. boost: how far over their share (TxPower / RBs of the carrier, "
                << Num(DlShareDbm(0), 2)
                << " dBm)\nthe satellite sent the RBs of the UE's data; w/o loop: the same "
                   "without the closed loop (what --dlBoostMaxDb or the PFD limit allow). saved: "
                   "power sent to the\nUE against that (mW, over all its RBs). up, down, hold: "
                   "periods in which the cap on the boost was raised, lowered or kept; top: "
                   "periods in which the boost the UE needs\nwas over the top of the range (the "
                   "target is out of reach; nothing more to give).\n";
        }
        if (ulClosedLoop)
        {
            Frame("CLOSED LOOP: UPLINK POWER",
                  "lower the power above " + Num(ulSnrHighDb, 1) + " dB, never under " +
                      Num(ulSnrLowDb, 1) + " dB; at least " + Num(ulPowerMinDbm, 0) + " dBm; " +
                      every);
            auto row = [&](const LoopTally& t) {
                return std::vector<std::string>{std::to_string(t.ulRx),
                                                mean1(t.ulSnr, 2),
                                                pct(t.ulBelow, t.ulRx),
                                                pct(t.ulInBand, t.ulRx),
                                                pct(t.ulAbove, t.ulRx),
                                                pct(t.ulWinInBand, t.ulWin),
                                                mw2dbm(t.ulMw, t.ulRx),
                                                t.ulPower.n > 0 ? Num(t.ulPower.minDb, 2) : "-",
                                                mw2dbm(t.ulMwFree, t.ulRx),
                                                savedPct(t.ulMw, t.ulMwFree),
                                                std::to_string(t.ulUp),
                                                std::to_string(t.ulDown),
                                                std::to_string(t.ulHold),
                                                std::to_string(t.ulAtMin)};
            };
            std::vector<std::vector<std::string>> rows;
            for (const uint32_t i : callers)
            {
                rows.push_back(row(g_loopTally[i]));
            }
            printTable({"PUSCH", "SNR", "under", "in band", "over", "win in", "power", "power",
                        "w/o loop", "saved", "up", "down", "hold", "low"},
                       {"", "[dB]", "[%]", "[%]", "[%]", "band[%]", "mean[dBm]", "min[dBm]",
                        "mean[dBm]", "[%]", "", "", "", ""},
                       rows,
                       row(sum));
            std::cout << "PUSCH: transmissions the gNB received from the UE during its call (a "
                         "repetition is one). SNR: their mean, dB, as if the UE had sent on "
                      << static_cast<uint32_t>(g_ulSnrRbs)
                      << " RB (the uplink SNR table's\nfigure for a narrow grant, whatever the "
                         "grant). under, in band, over: the share of them under "
                      << Num(ulSnrLowDb, 1) << " dB, from there to " << Num(ulSnrHighDb, 1)
                      << " dB, over it (the SNR of one transmission scatters with the\nfading; the "
                         "loop acts on its mean over a period). win in band: the share of the "
                         "periods whose mean was in the band. power: the UE's, at each "
                         "transmission\n(mean of the mW); w/o loop: what it would have sent "
                         "without the loop (all "
                      << Num(ueTxPowerDbm, 0)
                      << " dBm, or the open-loop power); saved: against that. up, down, hold: "
                         "periods in which the power was raised,\nlowered or kept; low: periods "
                         "over the band with the power already at its lowest.\n";
        }
        for (const uint32_t i : callers)
        {
            const LoopTally& t = g_loopTally[i];
            csvLoop << csvRun << "," << i << "," << addressOf(ueIfaces.GetAddress(i)) << ","
                    << t.dlRx << "," << CsvNum(t.dlSnr.MeanDb()) << ","
                    << (t.dlRx > 0 ? CsvNum(100.0 * t.dlSnrBelow / t.dlRx) : "") << ","
                    << (t.dlWin > 0 ? CsvNum(100.0 * t.dlWinBelow / t.dlWin) : "") << ","
                    << CsvNum(t.dlMcs.MeanDb()) << ","
                    << (t.dlRx > 0 ? CsvNum(100.0 * t.dlMcsBelow / t.dlRx) : "") << ","
                    << CsvNum(t.dlBoost.MeanDb()) << ","
                    << CsvNum(t.dlBoost.n > 0 ? t.dlBoost.maxDb : NAN) << ","
                    << CsvNum(t.dlBoostFree.MeanDb()) << ","
                    << (t.dlMwFree > 0.0 ? CsvNum(100.0 * (1.0 - t.dlMwUsed / t.dlMwFree)) : "")
                    << "," << t.dlUp << "," << t.dlDown << "," << t.dlHold << "," << t.dlAtMax
                    << "," << t.ulRx << "," << CsvNum(t.ulSnr.MeanDb()) << ","
                    << (t.ulRx > 0 ? CsvNum(100.0 * t.ulBelow / t.ulRx) : "") << ","
                    << (t.ulRx > 0 ? CsvNum(100.0 * t.ulInBand / t.ulRx) : "") << ","
                    << (t.ulRx > 0 ? CsvNum(100.0 * t.ulAbove / t.ulRx) : "") << ","
                    << (t.ulWin > 0 ? CsvNum(100.0 * t.ulWinInBand / t.ulWin) : "") << ","
                    << (t.ulRx > 0 ? CsvNum(10.0 * std::log10(t.ulMw / t.ulRx)) : "") << ","
                    << CsvNum(t.ulPower.n > 0 ? t.ulPower.minDb : NAN) << ","
                    << (t.ulRx > 0 ? CsvNum(10.0 * std::log10(t.ulMwFree / t.ulRx)) : "") << ","
                    << (t.ulMwFree > 0.0 ? CsvNum(100.0 * (1.0 - t.ulMw / t.ulMwFree)) : "")
                    << "," << t.ulUp << "," << t.ulDown << "," << t.ulHold << "," << t.ulAtMin
                    << "\n";
        }
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
                  << "satellite cut: what the satellite took off its power for the whole beam\n"
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
    g_sinrTraceDl.close();
    g_sinrTraceUl.close();
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
                         "e2eOver150Pct,callStatus,releasedAtS,cell,sat,beam,colour",
                     csvCalls.str());
        WriteCsvFile(csv + "_cells.csv",
                     csvRunHeader +
                         ",cell,sat,beam,tier,colour,northKm,eastKm,ues,blocked,released,elevDeg,"
                         "ciMeanDb,ciMinDb,dlSinrDb,onBearer,inPdb,late,discard,lost,overPdbPct,"
                         "perPct,ccePct",
                     csvCells.str());
        WriteCsvFile(csv + "_control.csv",
                     csvRunHeader +
                         ",admission,admissionRbPct,admissionCcePct,dropCalls,dropOnTimePct,"
                         "offered,admitted,"
                         "blocked,blockedDlRbs,blockedUlRbs,blockedCces,released,peakAdmitted,"
                         "meanDlRbs,meanUlRbs,meanCces,capacityDlRbs,capacityUlRbs,capacityCces,"
                         "peakDlPct,peakUlPct,peakCcePct,retry,retrySeconds,maxRetry,retriedUes,"
                         "retries",
                     csvRun + "," + csvControl.str());
        WriteCsvFile(csv + "_delays.csv",
                     csvRunHeader + ",flow,ue,dir,delayMs,packets",
                     csvDelays.str());
        WriteCsvFile(csv + "_snr.csv",
                     csvRunHeader + ",ue,address,elevDeg,dlN,dlMinDb,dlMeanDb,dlMaxDb,ulN,ulMinDb,"
                                    "ulMeanDb,ulMaxDb,ulRxMinDb,ulRxMeanDb,ulRxMaxDb",
                     csvSnr.str());
        std::string written = csv + "_calls.csv, _cells.csv, _control.csv, _delays.csv, _snr.csv";
        if (retry)
        {
            WriteCsvFile(csv + "_tries.csv",
                         csvRunHeader + ",ue,address,dir,row,try,rnti,askedS,fromS,toS,outcome,"
                                        "blockedBy,onTimePct,calls,onBearer,inPdb,late,discard,"
                                        "lost,overPdbPct,perPct,delayMeanMs,delayP98Ms,"
                                        "delayMaxMs,kbps,missedPdb,missedPer",
                         csvTries.str());
            written += ", _tries.csv";
        }
        if (dlPowerShown)
        {
            WriteCsvFile(csv + "_dlpower.csv",
                         csvRunHeader + ",ue,address,elevDeg,nAllocations,minDbm,meanDbm,maxDbm," +
                             "nDataSnr,minDataSnrDb,meanDataSnrDb,maxDataSnrDb",
                         csvDlPower.str());
            written += ", _dlpower.csv";
        }
        if (loopShown)
        {
            WriteCsvFile(csv + "_loop.csv",
                         csvRunHeader + ",ue,address,dlRx,dlSnrMeanDb,dlSnrBelowPct,"
                                        "dlWinBelowPct,dlMcsMean,dlMcsBelowPct,dlBoostMeanDb,"
                                        "dlBoostMaxDb,dlBoostFreeMeanDb,dlSavedPct,dlUp,dlDown,"
                                        "dlHold,dlAtMax,ulRx,ulSnrMeanDb,ulUnderPct,ulInBandPct,"
                                        "ulOverPct,ulWinInBandPct,ulPowerMeanDbm,ulPowerMinDbm,"
                                        "ulFreeMeanDbm,ulSavedPct,ulUp,ulDown,ulHold,ulAtMin",
                         csvLoop.str());
            written += ", _loop.csv";
        }
        if (ulWideRbs > 0)
        {
            WriteCsvFile(csv + "_ulgrants.csv",
                         csvRunHeader + ",ue,address,slots,narrow,wide,other,rbSlots",
                         csvUlGrants.str());
            written += ", _ulgrants.csv";
        }
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
