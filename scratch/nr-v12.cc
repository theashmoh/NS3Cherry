/*
 * ntnlab_nr.cc  --  ntnlab_n.cc with the LTE cell replaced by an NR-NTN cell (first, simple version)
 *
 *   UE i ~~ service link ~~ LEO satellite ~~ feeder link ~~ gNB at the gateway === S1-U === SGW/PGW
 *   7.0.0.(i+2)             600 km, transparent                                             ||
 *                                                                    remote host 1.0.0.2 == 10 Gbps, 10 ms
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
 *     K_offset, the timing advance and the feeder link of a transparent payload
 *   - --satSet=2 swaps in the Set-2 satellite of TR 38.821, --altitudeKm=1200 the LEO-1200 one
 *     (any other altitude, 300 to 2000 km, takes the nearer of the two columns of the set);
 *     altitude, carriers, EIRP density, antenna gain, G/T and aperture are flags of their own
 *   - the beam is the pattern of TR 38.811 6.4.1, 4 |J1(ka sin(t)) / (ka sin(t))|^2 with
 *     k = 2 pi f / c and a = half the aperture: the same antenna has a narrower beam on a higher
 *     carrier, so downlink and uplink (FDD) each have their own beamwidth. The set's 3 dB
 *     beamwidth holds at the set's 2 GHz. --beamHpbwDeg fixes the width on both carriers instead.
 *     EIRP density, gain and G/T do NOT follow the carrier: they are what their flags say.
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
 * the timing advance cover, and on the downlink the loss of its length: the satellite forwards
 * the gateway's signal together with its own receiver noise. On the uplink it adds delay only.
 *   - --gatewayDiameterKm=200 (default): the gateway is at a random place of the circle of that
 *     diameter around UE 0 (within 100 km of it), every place of the circle as likely as any
 *     other. It is drawn on its own, whatever the UEs' places are; --RngRun gives another one.
 *     Places from where the satellite is below 5 deg at any moment of the calls are left out.
 *   - --gatewayDiameterKm=0: the gateway is where UE 0 is, so the feeder link is as long as
 *     UE 0's service link.
 * With --transparent=0 the gNB is on the satellite and no gateway is in the radio path.
 *
 * The satellite
 *   - --freezeGeometry=1 (default): it stands still at --elevation but reports its orbital
 *     velocity, so the fading keeps moving. Elevation, range and delay are constant.
 *   - --freezeGeometry=0: it flies ns-3.48's circular orbit, as in leo-link. It comes from the
 *     north, is at --elevation when the calls start, passes exactly over UE 0 and goes down in
 *     the south. Elevation, range and delay change during the call, K_offset covers the longest
 *     round trip of the run, the timing advance follows the delay, and the beam keeps pointing
 *     at UE 0. The calls must end before the satellite is down at 10 deg again; the header says
 *     how long that is.
 *   - --fullPass=1: the whole pass over UE 0. The satellite flies, UE 0's call starts when the
 *     satellite has risen to 10 deg above it and ends when it has sunk to 10 deg again, and the
 *     run lasts as long as that call. Every other UE starts its call when the satellite has
 *     risen to 10 deg above itself (with UE 0, if that was earlier) and ends it with UE 0.
 *     --freezeGeometry, --elevation and --duration are not used.
 *   - When the satellite flies, the fast fading behaves differently: ns-3.48's channel model
 *     draws a new realisation (K-factor included) whenever the satellite has moved 1 m, here
 *     every 1 to 2 ms. When the satellite stands still, it is drawn once per line-of-sight
 *     state and only the Doppler term moves it.
 *
 * Line of sight and shadowing, --largeScale
 *   - fixed (default): what stands around a UE does not change during its call. Every UE has an
 *     obstruction level (uniform, 0 to 1) and a shadowing level (standard normal), drawn once.
 *     It is in line of sight whenever the LOS probability at its elevation (TR 38.811 Table
 *     6.6.1-1) is above its obstruction level, and its shadow fading is its shadowing level
 *     times the sigma of its state and elevation (Tables 6.6.2-1 to 6.6.2-3). Under a satellite
 *     that stands still a UE is in line of sight for the whole call or not at all; when the
 *     satellite flies, a blocked UE clears once the satellite is high enough and is blocked
 *     again when it has sunk below that elevation. The same holds on both carriers. A blocked
 *     UE also has ns-3's clutter loss (18 dB suburban, 29 dB urban at 30 deg), so it is far
 *     below the others. --RngRun gives other levels.
 *   - ns3: ns-3's own models. They draw the line-of-sight state of each UE again every second,
 *     whatever it was before, and the shadow fading with each new state; when the satellite
 *     flies they also let the shadowing drift every millisecond.
 *
 * Four tables, one line per call: the call itself, why its packets were lost, the call against
 * the 5QI of its bearer, and what the UE saw of the satellite during its call: elevation (-
 * before the satellite is overhead, + after),
 * slant range, and the propagation delay of the whole path, remote host to UE. N[km] and E[km]
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
 *             point at the UPF, the same in both directions. Here that is UE to PGW: the N6
 *             link to the remote host (--internetDelay) is not in it. It is measured on every
 *             packet with a time stamp put on at one end and read at the other.
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
 * What the sender does with a packet it cannot send in time is the nr module's: its RLC does
 * not take a new packet while the oldest one in its queue has waited longer than the discard
 * timer. The timer is the PDB of the bearer at the gNB (100 ms for 5QI 1) and none at a UE, so
 * a gNB discards and a UE delivers late. --ns3::NrRlcUm::DiscardTimerMs=100 gives both the
 * same timer, --ns3::NrRlcUm::EnablePdcpDiscarding=0 none to either. TS 23.501 keeps 20 ms
 * of the 100 for the core network (Table 5.7.4-1 NOTE 13); the core of this model takes no
 * time between PGW and gNB, so the whole delay is the radio side's.
 *
 * Control channels and radio link failure, --idealControl
 *   - 1 (default): the nr module as it is. No control message is ever lost: a UE receives every
 *     scheduling message whatever its SINR, and after a radio link failure it is attached again
 *     0.1 to 0.3 s later wherever it is. Its radio link monitoring counts control symbols, not
 *     time: 1200 of them below Qout (-5 dB) and then T310 (1 s) make a failure. A blocked UE at
 *     -9 or -12 dB therefore still gets nearly all its packets, each after three or four HARQ
 *     transmissions, and fails and attaches again every 3 to 5 s. Its RLF column counts these
 *     rounds, and each costs a handful of packets.
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
 *   UO-0, 3 bytes, in steady state; UOR-2 with extension 3, 13 bytes, on the first SID after the
 *   hangover and on the first voice packet after a silence (a context update); IR, 39 bytes, on
 *   the first --rohcIrPackets packets (1) of the call and of every new connection after a radio
 *   link failure.
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
 *     source causes, or for a refresh: the compressor sends one IR 1700 packets after the
 *     last IR and a context update 700 packets after the last one
 *     (--ns3::NrPdcp::RohcIrRefresh, RohcFoRefresh; RohcIrRefreshTime for a period in time).
 *   --rohcMode=O (default): feedback in PDCP control PDUs on the reverse link, where it takes its
 *     time and can be lost. The decompressor answers every IR with an ACK; after 10 packets in a
 *     row that it could not restore (--ns3::NrPdcp::RohcNackFailures) it sends a NACK, or a
 *     STATIC-NACK if the static context is gone, and again every 32 such packets
 *     (RohcFeedbackSpacing). The compressor answers a NACK with one context update and a
 *     STATIC-NACK with one IR. No periodic refresh once feedback has arrived.
 *   The mode of a compressor (RFC 3095 5.6.2): it starts in U-mode, whatever --rohcMode says,
 *     and is in O-mode from the moment any feedback of the decompressor reaches it (each is a
 *     FEEDBACK-2 with the CRC option, 5 bytes, and asks for O-mode). That is normally the ACK
 *     of the first IR, a few tens of ms into the call. If that ACK is lost on the reverse link
 *     the compressor stays in U-mode, with its periodic refreshes, until a NACK or STATIC-NACK
 *     or the ACK of its next IR refresh arrives. It never goes back to U-mode by itself: U-mode
 *     comes back with the new connection after a radio link failure.
 * --rohcModel=library: the ROHC library compresses the real headers. --rohcMode as above. The
 *   three size-table options do not reach it: its number of IR packets is
 *   --ns3::NrPdcp::RohcRepetitions (4, which is also how often it repeats every change), and
 *   what its context survives follows from the packet formats it picks. Its other settings are
 *   NrPdcp attributes too: --ns3::NrPdcp::RohcNackFailures=1 and so on. After a burst of lost
 *   packets the library sometimes puts back a wrong header (a wrong sequence number that its
 *   3-bit CRC lets pass). The receiver's UDP checksum would reject such a packet; NrPdcp drops
 *   it and counts it as failed ("header restored wrongly" in --rohcReport=full).
 *
 * Layer-2 headers, --l2Headers: "1+1+3" (default) is PDCP 1 + RLC 1 + MAC 3 = 5 bytes, the
 * overhead of TR 26.940 5.2.3 (7-bit PDCP sequence number, 5-bit RLC UM sequence number, MAC
 * subheader with the 16-bit length field); "2+2+2" is what the nr module sends by itself. The
 * 5-bit RLC sequence number has a reordering window of 16 PDUs: after 16 to 31 PDUs lost in a
 * row the receiver takes the next ones for old ones and discards them (TS 36.322 5.1.2.2.2).
 *
 * Voice packets (--rtp=1, default): every call is an RTP stream, so each packet has 40 bytes of
 * IP/UDP/RTP header before ROHC and ROHC uses its RTP profile. The codec is AMR 4.75 kbit/s:
 * 95 bits per 20 ms frame = 12 bytes (--codecKbps, or --voicePayload in bytes), SID frames are
 * 56 bits = 7 bytes (--sidPayload). DTX as in TR 26.940 6.4.3: while the voice activity detector
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
 *   - who comes first in a slot: SIB1 (every 20 ms, common search space, level 8 or the
 *     highest the CORESET holds), the random access response, the uplink grants, the downlink
 *     assignments; in each direction retransmissions first, then the UE served longest ago.
 *     Uplink grants may take the CCEs the downlink did not ask for in the slot before, or on
 *     average half of the CORESET (NrPdcch::UlMaxShare), whichever is more.
 *   - --idealControl=0: a DCI is lost with the block error rate of its level at the SINR it
 *     arrives with, and the radio link monitoring uses the same table for its two hypothetical
 *     PDCCHs (TS 38.133 8.1.2: level 8 with 4 dB more power at 10 %, level 4 at 2 %).
 *   - the PDCCH report: CCEs used and available, utilization, DCIs and CCEs by purpose with
 *     the blocked ones, and by aggregation level with the DCIs missed.
 *   Other settings are attributes: --ns3::NrPdcch::CoresetRbs, Candidates, TargetBler,
 *   SinrMarginDb, SinrOffsetDb, TableDciBits, CommonAl, RaiseAl, Sib1PeriodMs, UlMaxShare.
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
 *   SpsMcsTolerance. The report after the PDCCH tables counts the blocks that needed no DCI.
 *
 * Uplink calls run, but the handheld uplink of this release is far below its link budget.
 *
 * Run:
 *   ./ns3 run "ntnlab_nr"                              10 calls, downlink, 30 deg, UE 0 on the beam
 *                                                      centre and 9 UEs dropped at random
 *   ./ns3 run "ntnlab_nr --RngRun=2"                   another drop (and other calls)
 *   ./ns3 run "ntnlab_nr --ueLayout=line"              UEs in a line, 2 km apart
 *   ./ns3 run "ntnlab_nr --ueLayout=line --elevation=60 --nUes=50 --spacingKm=0.4"
 *   ./ns3 run "ntnlab_nr --freezeGeometry=0 --duration=100s"   the satellite flies: 30 -> 72 deg
 *   ./ns3 run "ntnlab_nr --fullPass=1 --nUes=2"        the whole pass over UE 0, 10 deg to 10 deg
 *   ./ns3 run "ntnlab_nr --satSet=2"                   Set-2 satellite: 28 dBW/MHz, 24 dBi
 *   ./ns3 run "ntnlab_nr --altitudeKm=1200"            LEO-1200 of the set: 40 dBW/MHz
 *   ./ns3 run "ntnlab_nr --satEirpDbwPerMHz=31"        the set, with 3 dB less power
 *   ./ns3 run "ntnlab_nr --dlFrequencyGHz=4.175"       same 2 m antenna: the beam is 2.11 deg wide
 *   ./ns3 run "ntnlab_nr --dlFrequencyGHz=4.175 --beamHpbwDeg=4.4127"   the beam kept as at 2 GHz
 *   ./ns3 run "ntnlab_nr --direction=ul"
 *   ./ns3 run "ntnlab_nr --gatewayDiameterKm=1000"     the gateway within 500 km of UE 0
 *   ./ns3 run "ntnlab_nr --gatewayDiameterKm=0"        the gateway where UE 0 is
 *   ./ns3 run "ntnlab_nr --transparent=0"              gNB on the satellite, no feeder link
 *   ./ns3 run "ntnlab_nr --scenario=NTN-Rural"         or NTN-Urban, NTN-DenseUrban
 *   ./ns3 run "ntnlab_nr --largeScale=ns3"             line of sight redrawn every second, as before
 *   ./ns3 run "ntnlab_nr --idealControl=0"             scheduling messages are lost at low SINR
 *   ./ns3 run "ntnlab_nr --idealControl=0 --largeScale=ns3 --ns3::NrUeRrc::T310=2s"
 *                                                      blockages of 1 s that ROHC has to survive
 *   ./ns3 run "ntnlab_nr --ns3::NrRlcUm::DiscardTimerMs=100"   UEs discard late packets too
 *   ./ns3 run "ntnlab_nr --rohc=0"                     without header compression
 *   ./ns3 run "ntnlab_nr --nUes=1 --rohcReport=full"   every ROHC detail of the call
 *   ./ns3 run "ntnlab_nr --rohcIrPackets=3 --rohcDynamicWindow=5 --rohcStaticWindow=30"
 *   ./ns3 run "ntnlab_nr --rohcMode=U --ns3::NrPdcp::RohcIrRefresh=250"   no feedback, IR every 5 s of talk
 *   ./ns3 run "ntnlab_nr --rohcDynamicWindow=1000000 --rohcStaticWindow=1000000"   a context that never fails
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
 *   ./ns3 run "ntnlab_nr --nrTraces=1" --cwd=results/t1   also write the NR trace files there
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
#include "ns3/network-module.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-ntn-link-models.h"
#include "ns3/point-to-point-module.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/traffic-generator-helper.h"
#include "ns3/traffic-generator-ngmn-voip.h"
#include "ns3/uniform-planar-array.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <tuple>
#include <vector>

using namespace ns3;

namespace
{

std::vector<double> g_dlSinrSum; // one entry per UE, linear
std::vector<uint32_t> g_dlSinrN;

// "DlCtrlSinr" of UE i's downlink PHY: the SINR of every control symbol the UE receives, so it
// is there for uplink calls too. i is bound when connecting.
void
DlSinr(uint32_t ue, uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
{
    g_dlSinrSum[ue] += sinr;
    ++g_dlSinrN[ue];
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

// --largeScale=fixed: what stands around a UE does not change while it makes its call.
//
// ns-3's own NTN models draw the line-of-sight state of a link again every second, each time
// with the probability of the table, whatever the state was before, and they draw the shadow
// fading again with every new state (and let it drift when the satellite moves). A UE that
// stands still then goes in and out of line of sight during one call.
//
// Here every UE has two numbers of its own, drawn once and kept for the whole run:
//   its obstruction level, uniform between 0 and 1. The UE is in line of sight whenever the LOS
//     probability at its elevation (TR 38.811 Table 6.6.1-1) is above its level. A UE that
//     stands under a satellite that stands still keeps its state. When the satellite flies, a
//     blocked UE clears once the satellite has climbed far enough, stays clear, and is blocked
//     again when the satellite has sunk below that elevation.
//   its shadowing level, standard normal. Its shadow fading in dB is that level times the
//     sigma of its state and elevation (TR 38.811 Tables 6.6.2-1 to 6.6.2-3), so it changes
//     only when the row of the table or the state changes.
// Both hold for the downlink and the uplink carrier alike: the obstacles are the same.
// The tables have one row per 10 deg of elevation and the row nearest to the elevation counts
// (10 deg for everything below), exactly as in ns-3's models: same numbers, same rounding.
// Everything else is still ns-3's: path loss, the clutter loss of a blocked UE, scintillation,
// and the fast fading with its K-factor (which ns-3 draws anew with each new state and, when
// the satellite flies, with every metre it moves).
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

// What the functions below work on while the simulation runs
struct Scene
{
    Ptr<GeocentricConstantPositionMobilityModel> satellite;
    Vector centre; // the beam centre, ECEF: the place on the ground where UE 0 is
    std::vector<Ptr<FixedMobilityModel>> ues;
    Ptr<NrNtnPropagationDelayModel> delay;
    Ptr<ChannelConditionModel> condition;                     // carrier of the voice direction
    Ptr<FixedLevelChannelConditionModel> levels;              // --largeScale=fixed, else null
    std::vector<Ptr<UniformPlanarArray>> satArrays;           // one per BWP
    std::vector<std::vector<Ptr<IsotropicAntennaModel>>> ueElements; // per UE, one per BWP
    std::vector<std::vector<Ptr<PhasedArrayModel>>> ueArrays;        // per UE, one per BWP
    double ueAntennaGainDbi{0.0};
    std::vector<double> beamKa; // ka of the beam pattern per BWP: 0 = downlink, 1 = uplink carrier
    uint32_t voiceBwp{0};       // the BWP that carries the voice
    bool flying{false};      // --freezeGeometry=0
    OrbitPredictor orbit;    // where the satellite will be
    double groundDelayMs{0}; // remote host to gNB: the N6 link
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
    Span rangeKm;        // satellite to UE
    Span delayMs;        // one way, remote host to UE: N6 link + feeder link (transparent) + service link
    Span taperDb;        // beam gain towards the UE, relative to the beam centre
    Span shadowDb;       // --largeScale=fixed: shadow fading of the UE as a gain (negative: weaker)
    uint32_t losYes{0};  // line-of-sight samples of the call: how many said yes ...
    uint32_t losAll{0};  // ... out of how many
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
    view.taperDb.Add(BeamTaperDb(sat, g_scene.centre, ue, g_scene.beamKa[g_scene.voiceBwp]));
    if (g_scene.levels)
    {
        view.shadowDb.Add(g_scene.levels->ShadowGainDb(g_scene.satellite, g_scene.ues[i]));
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
// carries the voice and counted while the UE's call is on. The channel redraws the state once per
// second (see below). One tick for all UEs, in UE order: the question can be the one that makes
// the channel redraw, so who asks when decides the order of the random draws.
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

// --freezeGeometry=0 only, every 100 ms: the beam stays on its centre and every UE's antenna gain gets
// the beam taper of the new geometry, as leo-link's UpdateAntennaOrientation() and
// UpdateSpotBeamTaper() do, here for each carrier with its own beam pattern. Setting the
// beamforming vector again drops the cached channel.
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
    Simulator::Schedule(MilliSeconds(100), &FollowBeam);
}

// --- The voice flow against its 5QI (TS 23.501 5.7.3 and Table 5.7.4-1) -------------------------
// The packet delay budget is the bound for the time a packet may take "between the UE and the
// N6 termination point at the UPF". The UPF of this model is the PGW node, and N6 is its link
// to the remote host. So every voice packet is stamped where it enters that stretch and read
// where it leaves it:
//   downlink  stamped when the IP layer of the PGW receives it from the remote host, read when
//             the IP layer of the UE delivers it to UDP
//   uplink    stamped when the IP layer of the UE sends it, read when the IP layer of the PGW
//             receives it out of the tunnel
// The stamp is a byte tag: it travels with the packet through the tunnels, PDCP (ROHC keeps
// the tags) and RLC, and changes no size.
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
        return 8;
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU64(static_cast<uint64_t>(m_ns));
    }

    void Deserialize(TagBuffer i) override
    {
        m_ns = static_cast<int64_t>(i.ReadU64());
    }

    void Print(std::ostream& os) const override
    {
        os << "entered the path between UE and UPF at " << m_ns << " ns";
    }

    int64_t m_ns{0}; ///< when the packet entered the path between UE and UPF
};

// What is measured of one call
struct VoiceQos
{
    std::vector<int64_t> delayNs; // UE <-> UPF, of every packet that arrived
    uint64_t unstamped{0};        // arrived without a stamp: must stay 0
    // The bit rate of the flow over the averaging window of the 5QI, in IP packets, as the UPF
    // counts it: the packets of the last window (time, bits) and the fullest window so far.
    // Times in whole microseconds: the few nanoseconds by which a longer packet arrives later
    // over the N6 link must not push a 101st packet into a window of 100 voice frames.
    std::deque<std::pair<int64_t, uint32_t>> window;
    uint64_t windowBits{0};
    uint64_t peakWindowBits{0};
};

struct VoiceQosScene
{
    std::vector<VoiceQos> calls;
    uint16_t port{0};                      // UDP port of the voice flows
    uint32_t remoteHost{0};                // its IPv4 address
    std::map<uint32_t, uint32_t> callOfUe; // IPv4 address of a UE -> its call
    int64_t windowUs{0};                   // the averaging window, microseconds
};
VoiceQosScene g_voiceQos;

// A voice packet of `ipBytes` bytes (IP header included) enters the path between UE and UPF
void
VoiceEnters(uint32_t call, Ptr<const Packet> packet, uint32_t ipBytes)
{
    VoiceStampTag stamp;
    stamp.m_ns = Simulator::Now().GetNanoSeconds();
    packet->AddByteTag(stamp);
    VoiceQos& q = g_voiceQos.calls[call];
    const int64_t now = Simulator::Now().GetMicroSeconds();
    q.window.emplace_back(now, ipBytes * 8);
    q.windowBits += ipBytes * 8;
    while (q.window.front().first <= now - g_voiceQos.windowUs)
    {
        q.windowBits -= q.window.front().second;
        q.window.pop_front();
    }
    q.peakWindowBits = std::max(q.peakWindowBits, q.windowBits);
}

// A voice packet leaves the path between UE and UPF
void
VoiceLeaves(uint32_t call, Ptr<const Packet> packet)
{
    VoiceQos& q = g_voiceQos.calls[call];
    VoiceStampTag stamp;
    if (packet->FindFirstMatchingByteTag(stamp))
    {
        q.delayNs.push_back(Simulator::Now().GetNanoSeconds() - stamp.m_ns);
    }
    else
    {
        ++q.unstamped;
    }
}

// "Rx" of the IPv4 of the PGW: every packet its IP layer receives, IP header included. A voice
// packet from the remote host is on its way down and enters here; one from a UE has come up
// through the tunnel and leaves here. (The tunnelled packets themselves are for UDP port 2152.)
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
    if (source == g_voiceQos.remoteHost)
    {
        const auto ue = g_voiceQos.callOfUe.find(destination);
        if (ue != g_voiceQos.callOfUe.end())
        {
            VoiceEnters(ue->second, packet, packet->GetSize());
        }
    }
    else if (destination == g_voiceQos.remoteHost)
    {
        const auto ue = g_voiceQos.callOfUe.find(source);
        if (ue != g_voiceQos.callOfUe.end())
        {
            VoiceLeaves(ue->second, packet);
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

// "SendOutgoing" of the IPv4 of UE i: a packet it sends, before the IP header is put on
void
UeIpSends(uint32_t call, const Ipv4Header& header, Ptr<const Packet> packet, uint32_t interface)
{
    if (IsVoice(header, packet))
    {
        VoiceEnters(call, packet, packet->GetSize() + header.GetSerializedSize());
    }
}

// "LocalDeliver" of the IPv4 of UE i: a packet for this UE, its IP header taken off
void
UeIpDelivers(uint32_t call, const Ipv4Header& header, Ptr<const Packet> packet, uint32_t interface)
{
    if (IsVoice(header, packet))
    {
        VoiceLeaves(call, packet);
    }
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
// TR 38.811 model of --scenario between satellite and UE, the propagation delay of the real
// path, and for a transparent payload the feeder link on top.
struct NtnCarrier
{
    Ptr<SpectrumChannel> channel;
    Ptr<PropagationLossModel> serviceLoss;
    Ptr<NrNtnFeederNoiseLossModel> feeder;
};

NtnCarrier
CreateNtnCarrier(double frequencyHz,
                 const std::string& scenario,
                 Ptr<MobilityModel> gateway,
                 Ptr<NrNtnPropagationDelayModel> delayModel,
                 bool transparent,
                 Ptr<FixedLevelChannelConditionModel> levels)
{
    NtnCarrier carrier;
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories(scenario, "Default", "ThreeGpp");
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
            if (levels)
            {
                // the fast fading asks the same model for the state of a UE
                channelModel.Get<Object>()->SetAttribute("ChannelConditionModel",
                                                         PointerValue(levels));
            }
        }
    }
    if (levels)
    {
        // --largeScale=fixed: the state of a UE comes from its own level, and so does its
        // shadow fading, which replaces the one ns-3's path loss model would draw
        Ptr<ThreeGppPropagationLossModel> pathLoss =
            DynamicCast<ThreeGppPropagationLossModel>(carrier.serviceLoss);
        NS_ABORT_MSG_UNLESS(pathLoss && !pathLoss->GetNext(), "unexpected path loss model");
        pathLoss->SetChannelConditionModel(levels);
        pathLoss->SetAttribute("ShadowingEnabled", BooleanValue(false));
        Ptr<FixedLevelShadowingLossModel> shadowing = CreateObject<FixedLevelShadowingLossModel>();
        shadowing->SetLevels(levels);
        pathLoss->SetNext(shadowing);
    }
    carrier.channel->SetPropagationDelayModel(delayModel);
    carrier.channel->AddSpectrumTransmitFilter(CreateObject<NrNtnUeToUeFilter>());

    // The feeder link is line of sight: the gateway is sited for it
    Ptr<NrChannelHelper> feederHelper = CreateObject<NrChannelHelper>();
    feederHelper->ConfigureFactories(scenario, "LOS", "ThreeGpp");
    Ptr<PropagationLossModel> feederLoss =
        feederHelper->CreateChannel(NrChannelHelper::INIT_PROPAGATION)->GetPropagationLossModel();
    feederLoss->SetAttributeFailSafe("Frequency", DoubleValue(frequencyHz));
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

    void Setup(Address remote,
               double voiceActivityFactor,
               uint32_t voiceBytes,
               uint32_t sidBytes,
               uint32_t hangoverFrames,
               uint32_t sidEveryFrames,
               uint32_t call)
    {
        m_remote = remote;
        m_voiceBytes = voiceBytes;
        m_sidBytes = sidBytes;
        m_hangoverFrames = hangoverFrames;
        m_hangoverLeft = hangoverFrames; // as after an encoder reset
        m_sidEveryFrames = sidEveryFrames;
        m_toSilence = 20.0 / 2000.0;
        m_toTalk = m_toSilence * voiceActivityFactor / (1.0 - voiceActivityFactor);
        m_ssrc = 0x4e544e00 + call; // "NTN" + the call
        m_coin = CreateObject<UniformRandomVariable>();
        m_coin->SetStream(600 + call); // its own stream: the calls do not disturb each other
        // Where the RTP sequence number and timestamp start: a stream of its own again, so the
        // talk and silence of the call are the same with any other start
        Ptr<UniformRandomVariable> start = CreateObject<UniformRandomVariable>();
        start->SetStream(100600 + call);
        m_sequence = start->GetInteger(0, 0xffff);
        m_timestampStart = start->GetInteger(0, 0xffff) << 16;
        m_timestampStart |= start->GetInteger(0, 0xffff);
    }

  private:
    void StartApplication() override
    {
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
    std::string largeScale = "fixed"; // fixed: each UE keeps its line-of-sight and shadowing
                                      // levels for the run; ns3: ns-3 draws them again and again
    double spacingKm = 2.0;  // --ueLayout=line: UE i is i x spacingKm from the beam centre
    Time duration = Seconds(20);
    std::string direction = "dl";
    bool idealControl = true; // false: the PDCCH has a block error rate (NrUePhy::PdcchErrorModel)
    std::string scenario = "NTN-Suburban";
    bool transparent = true; // false: regenerative, the gNB is on the satellite
    std::string scheduler = "ns3::NrMacSchedulerTdmaRR"; // leo-link's default
    bool nrTraces = false; // true: NrHelper::EnableTraces(), about 25 .txt files in the run folder
    bool rohc = NrPdcp::IsRohcAvailable(); // on whenever the nr module was built with the ROHC library
    std::string rohcModel = "table";       // table: reference header sizes, library: real ROHC
    std::string rohcMode = "O";            // U = no feedback, O = feedback PDUs on the reverse link
    uint32_t rohcIrPackets = 1;            // size table: IR packets in a row at the start of a call
    uint32_t rohcDynamicWindow = 13;       // size table: packets missing in a row that UO-0 survives
    uint32_t rohcStaticWindow = 61;        // size table: ... that a context update still repairs
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
    double gatewayTxPowerDbm = 60.0; // feeder link, transparent only (leo-link's defaults)
    double satFeederRxGainDbi = 30.0;
    Time internetDelay = MilliSeconds(10);
    uint16_t port = 4000;
    double gfbrKbps = 40.0; // guaranteed (and maximum) bit rate of the 5QI 1 voice flow
    uint32_t startOffsetMs = 1; // call i starts at 1 s + i x startOffsetMs
    double vafMean = 0.45; // voice activity factor of each call ~ Gaussian(vafMean, vafStd)
    double vafStd = 0.15;  // 0 = every call gets vafMean
    bool pdcch = false;         // true: the PDCCH is a resource (NrPdcch): CCEs, aggregation levels
    uint32_t pdcchSymbols = 1;  // PDCCH symbols per slot: 1, 2 or 3; 0 = what each slot needs
    uint32_t dciBits = 34;      // size of a DCI without its CRC
    std::string pdcchBlerTable; // SINR:BLER pairs for aggregation level 1; empty = NrPdcch's own
    std::string scheduling = "dynamic"; // dynamic, sps1 or sps2 (voice only)

    CommandLine cmd(__FILE__);
    // One line each in --PrintHelp (100 columns); the comment at the top of the file has the rest
    cmd.AddValue("nUes", "Number of UEs, one call each", nUes);
    cmd.AddValue("elevation",
                 "Satellite elevation at UE 0 when the calls start (deg), 10 to 90",
                 elevation);
    cmd.AddValue("freezeGeometry",
                 "1 = satellite stands still at --elevation, 0 = it flies over UE 0",
                 freezeGeometry);
    cmd.AddValue("fullPass",
                 "1 = the whole pass over UE 0: calls from 10 deg up to 10 deg down",
                 fullPass);
    cmd.AddValue("ueLayout", "UE 0 is on the beam centre; the others: random or line", ueLayout);
    cmd.AddValue("largeScale",
                 "LOS and shadowing per UE: fixed = kept for the run, ns3 = redrawn",
                 largeScale);
    cmd.AddValue("spacingKm",
                 "--ueLayout=line: UE i is i x spacingKm east of the beam centre (km)",
                 spacingKm);
    cmd.AddValue("duration", "How long each call lasts", duration);
    cmd.AddValue("direction",
                 "ul = UEs talk to the remote host, dl = remote host talks to UEs",
                 direction);
    cmd.AddValue("scenario", "NTN-Suburban, NTN-Rural, NTN-Urban or NTN-DenseUrban", scenario);
    cmd.AddValue("idealControl",
                 "1 = no control message is ever lost; 0 = lost at low SINR",
                 idealControl);
    cmd.AddValue("transparent",
                 "1 = gNB at the gateway (feeder link), 0 = gNB on the satellite",
                 transparent);
    cmd.AddValue("bandwidthMHz", "Bandwidth of each FDD carrier (MHz)", bandwidthMHz);
    cmd.AddValue("satSet",
                 "TR 38.821 Table 6.1.1.1-1/-2, S-band: 1 = Set-1, 2 = Set-2",
                 satSet);
    cmd.AddValue("altitudeKm",
                 "Altitude (km), 300 to 2000; set column: the nearer of 600 and 1200",
                 altitudeKm);
    cmd.AddValue("dlFrequencyGHz", "Downlink carrier (GHz), 0.6 to 6", dlFrequencyGHz);
    cmd.AddValue("ulFrequencyGHz",
                 "Uplink carrier (GHz), 0 = 190 MHz below the downlink",
                 ulFrequencyGHz);
    cmd.AddValue("satEirpDbwPerMHz",
                 "Satellite EIRP density (dBW/MHz), default: the set",
                 satEirpDbwPerMHz);
    cmd.AddValue("satAntennaGainDbi",
                 "Satellite Tx and Rx antenna gain (dBi), default: the set",
                 satAntennaGainDbi);
    cmd.AddValue("satGtDbPerK", "Satellite G/T (dB/K), default: the set", satGtDbPerK);
    cmd.AddValue("satApertureM",
                 "Antenna aperture (m), default: the set; it sets the beam width",
                 satApertureM);
    cmd.AddValue("beamHpbwDeg",
                 "3 dB beam width (deg) on both carriers, instead of the aperture's",
                 beamHpbwDeg);
    cmd.AddValue("gatewayDiameterKm",
                 "Gateway: random within this diameter around UE 0 (km); 0 = at UE 0",
                 gatewayDiameterKm);
    cmd.AddValue("gatewayTxPowerDbm",
                 "Feeder link: power the gateway radiates (dBm)",
                 gatewayTxPowerDbm);
    cmd.AddValue("satFeederRxGainDbi",
                 "Feeder link: satellite receive antenna gain (dBi)",
                 satFeederRxGainDbi);
    cmd.AddValue("scheduler", "MAC scheduler TypeId (TdmaRR, OfdmaRR, ...)", scheduler);
    cmd.AddValue("nrTraces", "Write the NR module's PHY/MAC/RLC/PDCP trace files", nrTraces);
    cmd.AddValue("rohc", "Compress the IP/UDP headers in PDCP with ROHC", rohc);
    cmd.AddValue("rohcModel",
                 "table = header sizes from a table, library = the ROHC library",
                 rohcModel);
    cmd.AddValue("rohcIrPackets",
                 "Size table: IR packets in a row at the start of a call",
                 rohcIrPackets);
    cmd.AddValue("rohcDynamicWindow",
                 "Size table: packets missing in a row that UO-0 survives",
                 rohcDynamicWindow);
    cmd.AddValue("rohcStaticWindow",
                 "Size table: packets missing in a row that a context update survives",
                 rohcStaticWindow);
    cmd.AddValue("l2Headers",
                 "PDCP+RLC+MAC header bytes: 1+1+3 (TR 26.940) or 2+2+2 (nr module)",
                 l2Headers);
    cmd.AddValue("rohcMode",
                 "U = no feedback, repair by refresh; O = NACKs on the reverse link",
                 rohcMode);
    cmd.AddValue("rohcReport", "ROHC report at the end: none, summary or full", rohcReport);
    cmd.AddValue("rtp", "1 = RTP source: 40 B headers, hangover; 0 = NGMN generator: 28 B", rtp);
    cmd.AddValue("codecKbps",
                 "Codec bit rate (kbit/s): voice frame = 20 ms of it, in whole bytes",
                 codecKbps);
    cmd.AddValue("voicePayload", "Bytes of one voice frame, 0 = from --codecKbps", voicePayload);
    cmd.AddValue("sidPayload", "Bytes of one silence descriptor (SID) frame", sidPayload);
    cmd.AddValue("hangoverFrames",
                 "--rtp=1: voice frames still sent after the talk stops, before DTX",
                 hangoverFrames);
    cmd.AddValue("sidIntervalMs",
                 "DTX: a SID frame at once, then every this many ms (multiple of 20)",
                 sidIntervalMs);
    cmd.AddValue("internetDelay", "Delay of the PGW to remote host link", internetDelay);
    cmd.AddValue("gfbrKbps", "Guaranteed bit rate of the 5QI 1 voice flow (kbit/s)", gfbrKbps);
    cmd.AddValue("startOffsetMs", "Call i starts at 1 s + i x startOffsetMs", startOffsetMs);
    cmd.AddValue("vafMean", "Mean of the per-call voice activity factor", vafMean);
    cmd.AddValue("vafStd", "Standard deviation of the per-call voice activity factor", vafStd);
    cmd.AddValue("pdcch", "1 = the PDCCH is a limited resource (CCEs); 0 = unlimited", pdcch);
    cmd.AddValue("pdcchSymbols",
                 "PDCCH symbols per slot: 1, 2 or 3; 0 = as many as each slot needs",
                 pdcchSymbols);
    cmd.AddValue("dciBits", "--pdcch=1: size of a DCI in bits, without its 24-bit CRC", dciBits);
    cmd.AddValue("pdcchBlerTable",
                 "--pdcch=1: PDCCH BLER at aggregation level 1, \"SINR:BLER SINR:BLER ...\"",
                 pdcchBlerTable);
    cmd.AddValue("scheduling",
                 "Voice: dynamic, or semi-persistent: sps1, sps2 (scenario 1, 2)",
                 scheduling);
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_UNLESS(direction == "ul" || direction == "dl", "--direction must be ul or dl");
    NS_ABORT_MSG_UNLESS(pdcchSymbols <= 3, "--pdcchSymbols must be 0, 1, 2 or 3");
    NS_ABORT_MSG_IF(pdcchSymbols == 0 && !pdcch,
                    "--pdcchSymbols=0 (as many as each slot needs) needs --pdcch=1");
    NS_ABORT_MSG_UNLESS(scheduling == "dynamic" || scheduling == "sps1" || scheduling == "sps2",
                        "--scheduling must be dynamic, sps1 or sps2");
    NS_ABORT_MSG_IF(scheduling != "dynamic" && !pdcch,
                    "--scheduling=sps1 and sps2 need --pdcch=1: what they save is PDCCH");
    NS_ABORT_MSG_UNLESS(ueLayout == "random" || ueLayout == "line",
                        "--ueLayout must be random or line");
    NS_ABORT_MSG_UNLESS(largeScale == "fixed" || largeScale == "ns3",
                        "--largeScale must be fixed or ns3");
    const bool fixedLargeScale = (largeScale == "fixed");
    NS_ABORT_MSG_UNLESS(gatewayDiameterKm >= 0.0 && gatewayDiameterKm <= 10000.0,
                        "--gatewayDiameterKm must be 0 to 10000");
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
    const bool ul = (direction == "ul");
    const std::string transportProtocol = "ns3::UdpSocketFactory";

    // One VAF per call, drawn from a Gaussian truncated to the generator's range [0, 0.99]
    // (values outside are drawn again). Change --RngRun to get a different set.
    Ptr<NormalRandomVariable> gauss = CreateObject<NormalRandomVariable>();
    gauss->SetAttribute("Mean", DoubleValue(vafMean));
    gauss->SetAttribute("Variance", DoubleValue(vafStd * vafStd));
    gauss->SetStream(500);
    std::vector<double> vaf(nUes);
    for (uint32_t i = 0; i < nUes; ++i)
    {
        do
        {
            vaf[i] = gauss->GetValue();
        } while (vaf[i] < 0.0 || vaf[i] > 0.99);
    }

    // ROHC in every PDCP entity of a data bearer, gNB and UE. Its other settings (profiles,
    // MAX_CID, repetitions, refreshes, NACK threshold ...) are attributes of NrPdcp and can be
    // given on the command line as --ns3::NrPdcp::RohcNackFailures=1 and so on.
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
    NS_ABORT_MSG_IF(rohcStaticWindow < rohcDynamicWindow,
                    "--rohcStaticWindow must not be smaller than --rohcDynamicWindow");
    Config::SetDefault("ns3::NrPdcp::EnableRohc", BooleanValue(rohc));
    if (rohc)
    {
        Config::SetDefault("ns3::NrPdcp::RohcMode", StringValue(rohcMode));
    }
    if (rohc && rohcModel == "table")
    {
        Config::SetDefault("ns3::NrPdcp::RohcModel", StringValue("Table"));
        Config::SetDefault("ns3::NrPdcp::RohcTableIrPackets", UintegerValue(rohcIrPackets));
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
    const double ueAntennaGainDbi = 0.0;
    const double satFeederRxGainDb = satFeederRxGainDbi;
    const double satEirpDbm = satEirpDbwPerMHz + 30.0 + 10.0 * std::log10(bandwidthMHz);
    const double noiseFloorDbm = -174.0 + 10.0 * std::log10(bandwidthHz) + ueNoiseFigureDb;
    const Time channelUpdatePeriod = MilliSeconds(1000); // LOS state redrawn this often

    Config::SetDefault("ns3::NrGnbPhy::PowerAllocationType", StringValue("UniformPowerAllocBw"));
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(channelUpdatePeriod));
    Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                       TimeValue(channelUpdatePeriod));
    Config::SetDefault("ns3::NrMacSchedulerHarqRr::ConsolidateHarqRetx", BooleanValue(true));
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(10 * 1024 * 1024));
    Config::SetDefault("ns3::NrRlcUm::ReorderingTimer", TimeValue(MilliSeconds(100)));
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

    // --- Where everything is -----------------------------------------------------------
    // The beam centre is on the equator at longitude 0, the UEs around it (see below). The
    // satellite comes from the north on a circular orbit, heading south, towards the beam centre.
    // Standing still, it is on the centre's meridian at the central angle that puts it at
    // --elevation. Flying, it passes exactly over the centre, and it is at --elevation at the
    // moment the calls start.
    const Time callsStart = Seconds(1);
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
        overheadAt = callsStart + Seconds(secondsFromOverhead(elevation));
        downAt = overheadAt + Seconds(secondsFromOverhead(10.0));
        if (fullPass)
        {
            duration = downAt - callsStart; // UE 0's call: from 10 deg to 10 deg
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
    const Time lastCallStart = callsStart + MilliSeconds((nUes - 1) * startOffsetMs);
    // When the last call ends. --fullPass=1: every call ends with UE 0's, when the satellite is
    // down at 10 deg above UE 0.
    const Time callsEnd = fullPass ? downAt : lastCallStart + duration;

    // A place on the ground, `arc` radians of great circle from the beam centre in the direction
    // `bearing` (0 = north, pi/2 = east)
    auto groundPoint = [&](double arc, double bearing) {
        return Vector(earthRadiusM * std::cos(arc),
                      earthRadiusM * std::sin(arc) * std::sin(bearing),
                      earthRadiusM * std::sin(arc) * std::cos(bearing));
    };
    // Where the satellite is while the calls are on: one place, or one per second of the pass
    // with the first and the last moment and the moment it is overhead
    std::vector<Vector> satDuringCalls;
    if (freezeGeometry)
    {
        satDuringCalls.push_back(orbit.pos0);
    }
    else
    {
        for (Time t = callsStart; t < callsEnd; t += Seconds(1))
        {
            satDuringCalls.push_back(orbit.At(t));
        }
        satDuringCalls.push_back(orbit.At(callsEnd));
        if (overheadAt > callsStart && overheadAt < callsEnd)
        {
            satDuringCalls.push_back(orbit.At(overheadAt));
        }
    }
    // The calls have to fit into the pass: UE 0 must see the satellite at 10 deg or more for as
    // long as they last. Said here, before any place is drawn: after the pass no place is left.
    NS_ABORT_MSG_IF(!freezeGeometry &&
                        std::any_of(satDuringCalls.begin(),
                                    satDuringCalls.end(),
                                    [&](const Vector& sat) {
                                        return ElevationDeg(sat, centrePosition) < 9.99;
                                    }),
                    "The satellite is down at 10 deg elevation again at t = "
                        << downAt.GetSeconds() << " s, before the calls end. Use --duration="
                        << std::floor((downAt - lastCallStart).GetSeconds())
                        << "s or less, or --fullPass=1 for the whole pass.");
    // The lowest elevation at which a UE that was dropped at random, or the gateway, may see the
    // satellite during the calls. UE 0 keeps 10 deg, where the 3GPP NTN tables start; the tables
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
    auto putThere = [&](Ptr<FixedMobilityModel> mobility, const Place& place, const std::string& what) {
        mobility->SetGeographicPosition(Vector(std::asin(place.ecef.z / earthRadiusM) * 180.0 / M_PI,
                                               std::atan2(place.ecef.y, place.ecef.x) * 180.0 / M_PI,
                                               0.0));
        NS_ABORT_MSG_IF((mobility->GetPosition() - place.ecef).GetLength() > 0.01,
                        what << " is not where it was drawn");
    };

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

    // When each call starts and ends. Call i starts i x startOffsetMs after the first one and
    // lasts --duration.
    // --fullPass=1: UE 0's call is the whole pass, from the moment the satellite has risen to
    // 10 deg above UE 0 to the moment it has sunk to 10 deg again. The satellite reaches 10 deg
    // above the other UEs a little earlier or later: a UE that is still below 10 deg when
    // UE 0's call starts begins its own call when the satellite has risen to 10 deg above it;
    // a UE that is already above 10 deg starts with UE 0. All calls end with UE 0's.
    std::vector<Time> ueCallStart(nUes);
    std::vector<Time> ueCallEnd(nUes);
    for (uint32_t i = 0; i < nUes; ++i)
    {
        ueCallStart[i] = callsStart + MilliSeconds(i * startOffsetMs); // not phase-aligned
        ueCallEnd[i] = ueCallStart[i] + duration;
        if (fullPass)
        {
            const Vector ue = ueMobility[i]->GetPosition();
            if (ElevationDeg(orbit.At(callsStart), ue) < 10.0)
            {
                Time below = callsStart;
                Time above = overheadAt;
                NS_ABORT_MSG_IF(ElevationDeg(orbit.At(above), ue) < 10.0,
                                "the satellite never rises to 10 deg above UE " << i);
                for (int step = 0; step < 50; ++step)
                {
                    const Time mid = below + (above - below) / 2;
                    (ElevationDeg(orbit.At(mid), ue) < 10.0 ? below : above) = mid;
                }
                ueCallStart[i] = std::max(ueCallStart[i], above);
            }
            ueCallEnd[i] = callsEnd;
        }
    }

    // The gateway (transparent payload: where the gNB is). It carries no device; the channel
    // only needs its position, for the delay and the loss of the feeder link.
    // --gatewayDiameterKm=0: where UE 0 is, as in leo-link; the feeder link then has the same
    // length as UE 0's service link.
    // --gatewayDiameterKm=D: a random place of the circle of diameter D around UE 0, every place
    // of the circle as likely as any other. It has nothing to do with where the UEs are: its
    // place comes from a random stream of its own. Left out are the places from where the
    // satellite is below 5 deg at any moment of the calls, as for the UEs.
    Ptr<Node> gateway = CreateObject<Node>();
    Ptr<FixedMobilityModel> gatewayMobility = CreateObject<FixedMobilityModel>();
    gatewayMobility->SetGeographicPosition(Vector(0.0, 0.0, 0.0));
    Place gatewayPlace;
    gatewayPlace.ecef = centrePosition;
    if (transparent && gatewayDiameterKm > 0.0)
    {
        auto seesSatellite = [&](const Vector& place) {
            return std::all_of(satDuringCalls.begin(), satDuringCalls.end(), [&](const Vector& sat) {
                return ElevationDeg(sat, place) >= minPlaceElevationDeg;
            });
        };
        Ptr<UniformRandomVariable> gatewayDrop =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(301));
        gatewayPlace = drawPlace(gatewayDrop,
                                 gatewayDiameterKm * 500.0 / earthRadiusM, // the radius, in radians
                                 seesSatellite,
                                 "the gateway");
        putThere(gatewayMobility, gatewayPlace, "the gateway");
    }
    gateway->AggregateObject(gatewayMobility);

    // --- Channels, delay and NTN timing ---------------------------------------------------
    Ptr<NrNtnPropagationDelayModel> ntnDelay = CreateObject<NrNtnPropagationDelayModel>();
    if (transparent)
    {
        ntnDelay->SetFeeder(gatewayMobility);
    }
    // --largeScale=fixed: every UE gets its obstruction level and its shadowing level, each from
    // a random stream of its own, so that they depend on nothing else in the run. Both carriers
    // use them. (The tables are those of the S-band, which is where the carriers are.)
    Ptr<FixedLevelChannelConditionModel> levels;
    if (fixedLargeScale)
    {
        NS_ABORT_MSG_UNLESS(g_largeScaleTables.count(scenario),
                            "--largeScale=fixed has no tables for --scenario=" << scenario);
        levels = CreateObject<FixedLevelChannelConditionModel>();
        levels->SetTable(g_largeScaleTables.at(scenario));
        Ptr<UniformRandomVariable> obstruction =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(302));
        Ptr<NormalRandomVariable> shadowing =
            CreateObjectWithAttributes<NormalRandomVariable>("Stream", IntegerValue(303));
        for (uint32_t i = 0; i < nUes; ++i)
        {
            const double obstructionLevel = obstruction->GetValue();
            levels->AddUe(ueMobility[i], obstructionLevel, shadowing->GetValue());
        }
    }
    g_scene.levels = levels;
    NtnCarrier dlCarrier =
        CreateNtnCarrier(dlFrequencyHz, scenario, gatewayMobility, ntnDelay, transparent, levels);
    NtnCarrier ulCarrier =
        CreateNtnCarrier(ulFrequencyHz, scenario, gatewayMobility, ntnDelay, transparent, levels);
    for (const auto& carrier : {dlCarrier, ulCarrier})
    {
        carrier.feeder->SetAttribute("GatewayTxPowerDbm", DoubleValue(gatewayTxPowerDbm));
        carrier.feeder->SetAttribute("SatRxAntennaGainDb", DoubleValue(satFeederRxGainDb));
        carrier.feeder->SetAttribute("RelayEirpDbm", DoubleValue(satEirpDbm));
        carrier.feeder->SetAttribute("RelayLossDb", DoubleValue(0.0));
        carrier.feeder->SetAttribute("RelayInputNoisePowerDbm", DoubleValue(noiseFloorDbm));
        carrier.feeder->SetAttribute("ReceiverNoisePowerDbm", DoubleValue(noiseFloorDbm));
    }

    // What the run will see of the satellite, every 100 ms from t = 0 to the end (one step when
    // it stands still): the longest round trip, which K_offset has to cover, and the elevation at
    // the beam centre, which must be 10 deg or more while the calls are on (the 3GPP NTN channel
    // is defined from 10 deg up; in the second before the calls and the two after them it may be
    // a little lower, and the channel model then takes its 10 deg values).
    const Time simEnd = callsStart + duration + Seconds(2);
    if (!freezeGeometry)
    {
        // The pass is what was asked for: --elevation when the calls start, the centre right below later
        NS_ABORT_MSG_UNLESS(
            std::abs(ElevationDeg(orbit.At(callsStart), centrePosition) - elevation) < 1e-6 &&
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
        NS_ABORT_MSG_IF(t >= callsStart && t <= callsEnd &&
                            ElevationDeg(sat, centrePosition) < 9.99,
                        "The satellite is down at 10 deg elevation again at t = "
                            << downAt.GetSeconds() << " s, before the calls end. Use --duration="
                            << std::floor((downAt - lastCallStart).GetSeconds()) << "s or less.");
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

    // Rank 1 over two cross-polarised ports at each end, CQI/PMI feedback on
    const uint32_t carrierRbs = static_cast<uint32_t>(bandwidthHz * 0.96 / (15e3 * 12));
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
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(true));
    // The symbols before the data: --pdcchSymbols, and with 0 the fewest a slot can have
    nrHelper->SetSchedulerAttribute("DlCtrlSymbols",
                                    UintegerValue(pdcchSymbols == 0 ? 1 : pdcchSymbols));
    nrHelper->SetGnbMacAttribute("NumHarqProcess", UintegerValue(16));
    nrHelper->SetUeMacAttribute("NumHarqProcess", UintegerValue(16));
    nrHelper->SetGnbPhyAttribute("N1Delay", UintegerValue(2));
    nrHelper->SetGnbPhyAttribute("N2Delay", UintegerValue(2));
    nrHelper->SetSchedulerAttribute("OllaStepUp", DoubleValue(0.01));
    nrHelper->SetSchedulerAttribute("OllaTargetBler", DoubleValue(0.15));
    nrHelper->SetSchedulerAttribute("OllaMaxOffset", DoubleValue(12.0));

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
    // Handheld: one element, two +/-45 deg ports, 0 dBi
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
    }

    // --- ONE remote host, far end of every call ---------------------------------------------
    auto [remoteHost, remoteHostAddr] = epcHelper->SetupRemoteHost("10Gb/s", 1500, internetDelay);

    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIfaces = epcHelper->AssignUeIpv4Address(ueDevs);
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

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
    g_scene.voiceBwp = ul ? 1 : 0;
    g_scene.flying = !freezeGeometry;
    g_scene.orbit = orbit;
    g_scene.groundDelayMs = internetDelay.GetSeconds() * 1000.0;
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
        for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
        {
            g_scene.ueArrays[i].push_back(DynamicCast<PhasedArrayModel>(
                NrHelper::GetUePhy(ueDevs.Get(i), bwp)->GetSpectrumPhy()->GetAntenna()));
            if (bwp > 0)
            {
                g_scene.ueArrays[i][bwp]->SetAntennaElement(g_scene.ueElements[i][bwp]);
            }
        }
    }
    PointBeam();
    if (!freezeGeometry)
    {
        Simulator::Schedule(MilliSeconds(100), &FollowBeam);
    }

    // The voice bearer: for every UE a dedicated GBR flow with 5QI 1 (TS 23.501 Table 5.7.4-1,
    // conversational voice), selected by the UDP port. One call of the helper and one rule per
    // UE: NrHelper::ActivateDedicatedQosFlow() for a whole container of devices sets up the
    // first device only (it returns inside its loop), and the core network writes the flow
    // identifier into the rule it is given. Every UE also keeps its default flow, unused.
    NrGbrQosInformation qos;
    qos.gbrUl = qos.gbrDl = qos.mbrUl = qos.mbrDl = static_cast<uint64_t>(gfbrKbps * 1000.0);
    const NrQosFlow voiceFlow(NrQosFlow::GBR_CONV_VOICE, qos);
    for (uint32_t i = 0; i < nUes; ++i)
    {
        Ptr<NrQosRule> rule = Create<NrQosRule>();
        NrQosRule::PacketFilter filter;
        filter.direction = ul ? NrQosRule::UPLINK : NrQosRule::DOWNLINK;
        if (ul)
        {
            filter.remotePortStart = filter.remotePortEnd = port;
        }
        else
        {
            filter.localPortStart = filter.localPortEnd = port;
        }
        rule->Add(filter);
        nrHelper->ActivateDedicatedQosFlow(ueDevs.Get(i), voiceFlow, rule);
    }
    // What TS 23.501 asks of that flow, as the nr module has it; the averaging window of 5QI 1
    // is 2000 ms (Table 5.7.4-1)
    const Time voicePdb = MilliSeconds(voiceFlow.GetPacketDelayBudgetMs());
    const double voicePer = voiceFlow.GetPacketErrorLossRate();
    const Time voiceAveragingWindow = MilliSeconds(2000);
    NS_ABORT_MSG_UNLESS(voiceFlow.GetResourceType() == 1 && voicePdb == MilliSeconds(100) &&
                            voicePer == 1e-2,
                        "5QI 1 of the nr module is not the one of TS 23.501 Table 5.7.4-1");

    // The delay between UE and UPF and the bit rate of every voice flow, see VoiceStampTag
    g_voiceQos = VoiceQosScene();
    g_voiceQos.calls.assign(nUes, VoiceQos());
    g_voiceQos.port = port;
    g_voiceQos.remoteHost = remoteHostAddr.Get();
    g_voiceQos.windowUs = voiceAveragingWindow.GetMicroSeconds();
    Connect(epcHelper->GetPgwNode()->GetObject<Ipv4L3Protocol>(), "Rx", MakeCallback(&PgwIpReceives));
    for (uint32_t i = 0; i < nUes; ++i)
    {
        g_voiceQos.callOfUe[ueIfaces.GetAddress(i).Get()] = i;
        Ptr<Ipv4L3Protocol> ip = ueNodes.Get(i)->GetObject<Ipv4L3Protocol>();
        Connect(ip, "SendOutgoing", MakeBoundCallback(&UeIpSends, i));
        Connect(ip, "LocalDeliver", MakeBoundCallback(&UeIpDelivers, i));
    }

    // Sinks: dl = one per UE (one Install call), ul = one on the remote host for all calls
    PacketSinkHelper sinkHelper(transportProtocol, InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApps = sinkHelper.Install(ul ? NodeContainer(remoteHost) : ueNodes);
    sinkApps.Start(Seconds(0));
    sinkApps.Stop(Seconds(3) + duration);

    // The loop holds only what differs per call
    g_dlSinrSum.assign(nUes, 0);
    g_dlSinrN.assign(nUes, 0);
    g_rlf.assign(nUes, 0);
    g_outages.assign(nUes, {});
    g_view.assign(nUes, CallView());
    g_lastVoiceTx.assign(nUes, Seconds(-1));
    // Line of sight is read on the carrier that carries the voice
    g_scene.condition =
        DynamicCast<ThreeGppPropagationLossModel>((ul ? ulCarrier : dlCarrier).serviceLoss)
            ->GetChannelConditionModel();
    ApplicationContainer voipApps;
    for (uint32_t i = 0; i < nUes; ++i)
    {
        Ipv4Address dest = ul ? remoteHostAddr : ueIfaces.GetAddress(i);
        ApplicationContainer voip;
        if (rtp)
        {
            Ptr<RtpVoiceSource> source = CreateObject<RtpVoiceSource>();
            source->Setup(InetSocketAddress(dest, port),
                          vaf[i],
                          voicePayload,
                          sidPayload,
                          hangoverFrames,
                          sidIntervalMs / 20,
                          i);
            (ul ? ueNodes.Get(i) : remoteHost)->AddApplication(source);
            voip.Add(source);
        }
        else
        {
            TrafficGeneratorHelper voipHelper(transportProtocol,
                                              InetSocketAddress(dest, port),
                                              TrafficGeneratorNgmnVoip::GetTypeId());
            voipHelper.SetAttribute("VoiceActivityFactor", DoubleValue(vaf[i])); // this call only
            voip = voipHelper.Install(ul ? ueNodes.Get(i) : remoteHost);
            voip.Get(0)->TraceConnectWithoutContext("Tx", MakeBoundCallback(&VoiceSent, i));
        }
        const Time callStart = ueCallStart[i];
        voip.Start(callStart);
        voip.Stop(ueCallEnd[i]);
        voipApps.Add(voip);
        g_view[i].callStart = callStart;
        g_view[i].callEnd = ueCallEnd[i];
        Simulator::Schedule(callStart, &SampleCall, i);
        if (!freezeGeometry && overheadAt > callStart && overheadAt < ueCallEnd[i])
        {
            // one more look when the satellite is over the centre, which the 100 ms steps can miss by 50 ms
            Simulator::Schedule(overheadAt, &RecordView, i);
        }

        Connect(NrHelper::GetUePhy(ueDevs.Get(i), 0), "DlCtrlSinr", MakeBoundCallback(&DlSinr, i));
        Connect(DynamicCast<NrUeNetDevice>(ueDevs.Get(i))->GetRrc(),
                "RadioLinkFailure",
                MakeBoundCallback(&RadioLinkFailure, i));
        Connect(DynamicCast<NrUeNetDevice>(ueDevs.Get(i))->GetRrc(),
                "ConnectionEstablished",
                MakeBoundCallback(&ConnectionEstablished, i));
    }

    // FlowMonitor on the UEs and the remote host gives one flow per call
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.Install(NodeContainer(ueNodes, remoteHost));

    Simulator::Schedule(Seconds(1), &SampleLos);

    if (nrTraces)
    {
        nrHelper->EnableTraces();
    }

    // --- What was built --------------------------------------------------------------------
    // The delay of the whole path, one way and propagation only: the N6 link, then the feeder
    // link when the gNB is at the gateway, then the service link. With the gNB on the satellite
    // the feeder link carries N3, which has no delay in this model.
    const std::string path =
        transparent ? "remote host -> gateway -> satellite -> UE" : "remote host -> satellite -> UE";
    const double n6Ms = internetDelay.GetSeconds() * 1000.0;
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
        std::ostringstream o;
        o << std::fixed << std::setprecision(2) << n6Ms + v.feederMs + v.serviceMs << " ms = " << n6Ms
          << " N6";
        if (transparent)
        {
            o << " + " << v.feederMs << " feeder";
        }
        o << " + " << v.serviceMs << " service";
        return o.str();
    };
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "NR-NTN cell: " << (transparent ? "transparent" : "regenerative") << ", " << scenario
              << ", " << bandwidthMHz << " MHz FDD, DL " << std::setprecision(3) << dlFrequencyGHz
              << " / UL " << ulFrequencyHz / 1e9 << " GHz, " << std::setprecision(2) << nUes << " "
              << direction << " call(s)\n";
    std::cout << "  voice packets          : ";
    std::ostringstream voiceFrame;
    if (voiceFromCodec)
    {
        voiceFrame << "codec " << codecKbps << " kbit/s = " << voiceFrameBits
                   << " bits per 20 ms frame -> ";
    }
    voiceFrame << voicePayload << " B voice frames";
    if (rtp)
    {
        std::cout << "RTP source, " << voiceFrame.str() << "\n"
                  << "  DTX                    : " << hangoverFrames
                  << " more voice frames after the talk stops (hangover), then a " << sidPayload
                  << " B SID at once and every " << sidIntervalMs << " ms\n"
                  << "  headers before ROHC    : IP 20 + UDP 8 + RTP 12 = 40 B on every packet\n";
    }
    else
    {
        std::cout << "NGMN generator, " << voiceFrame.str() << "\n"
                  << "  DTX                    : no hangover, a " << sidPayload << " B SID every "
                  << sidIntervalMs << " ms in silence\n"
                  << "  headers before ROHC    : IP 20 + UDP 8 = 28 B on every packet, no RTP header\n";
    }
    const uint32_t l2Bytes = (l2Headers == "1+1+3") ? 5 : 6;
    std::cout << "  header compression     : ";
    if (!rohc)
    {
        std::cout << "off\n";
    }
    else if (rohcModel == "table")
    {
        UintegerValue uo0;
        UintegerValue update;
        Ptr<NrPdcp> probe = CreateObject<NrPdcp>(); // has the values the command line may have set
        probe->GetAttribute("RohcTableUo0Size", uo0);
        probe->GetAttribute("RohcTableUpdateSize", update);
        const uint32_t chain = rtp ? 40 : 28;
        UintegerValue irRefresh;
        UintegerValue foRefresh;
        UintegerValue nackFailures;
        UintegerValue feedbackSpacing;
        probe->GetAttribute("RohcIrRefresh", irRefresh);
        probe->GetAttribute("RohcFoRefresh", foRefresh);
        probe->GetAttribute("RohcNackFailures", nackFailures);
        probe->GetAttribute("RohcFeedbackSpacing", feedbackSpacing);
        // One line each: the sizes, what the context survives, how it is repaired
        std::cout << "ROHC size table: UO-0 " << uo0.Get() << " B, UOR-2-EXT-3 " << update.Get()
                  << " B, IR " << chain - 1 << " B (" << rohcIrPackets << " at the start)\n"
                  << "  ROHC context           : over " << rohcDynamicWindow
                  << " packets missing in a row: needs UOR-2-EXT-3 or IR; over "
                  << rohcStaticWindow << ": needs IR\n"
                  << "  ROHC repair            : ";
        if (rohcMode == "O")
        {
            std::cout << "O-mode: NACK/STATIC-NACK on the reverse link after "
                      << nackFailures.Get() << " failed packets, then every "
                      << feedbackSpacing.Get() << "\n";
        }
        else
        {
            std::cout << "U-mode: no feedback; IR again after " << irRefresh.Get()
                      << " packets, UOR-2-EXT-3 after " << foRefresh.Get() << "\n";
        }
        std::cout << "  packet below the MAC   : voice " << voicePayload << " + " << uo0.Get() << " + "
                  << l2Bytes << " = " << voicePayload + uo0.Get() + l2Bytes << " B, SID " << sidPayload
                  << " + " << uo0.Get() << " + " << l2Bytes << " = " << sidPayload + uo0.Get() + l2Bytes
                  << " B, " << update.Get() - uo0.Get() << " B more on a context update\n";
    }
    else
    {
        std::cout << "ROHC library, " << rohcMode << "-mode"
                  << (rohcMode == "O" ? " (feedback in PDCP control PDUs)" : " (no feedback)") << "\n";
    }
    std::cout << "  layer-2 headers        : "
              << (l2Headers == "1+1+3"
                      ? "PDCP 1 + RLC 1 + MAC 3 = 5 B (7-bit PDCP SN, 5-bit RLC SN, 16-bit MAC length)"
                      : "PDCP 2 + RLC 2 + MAC 2 = 6 B (the nr module's own)")
              << "\n";
    // The voice bearer and what the sender does with a packet it cannot send in time: the RLC
    // (NrRlcUm) does not take a new packet while the oldest one in its queue has waited longer
    // than its discard timer. That timer is DiscardTimerMs or, if that is 0, the delay budget
    // of the bearer, which the nr module tells the RLC of the gNB only.
    const bool rlcDiscards = AttributeDefault<BooleanValue>("ns3::NrRlcUm", "EnablePdcpDiscarding");
    const uint64_t rlcDiscardMs = AttributeDefault<UintegerValue>("ns3::NrRlcUm", "DiscardTimerMs");
    const double voiceTalkKbps = (voicePayload + (rtp ? 40 : 28)) * 8 * 50 / 1000.0;
    std::cout << "  voice bearer           : 5QI 1 for every UE: GBR, GFBR " << gfbrKbps
              << " kbit/s (talking: " << voiceTalkKbps << "), " << std::setprecision(0)
              << voicePdb.GetSeconds() * 1000.0 << " ms delay budget UE-UPF, " << voicePer * 100.0
              << " % error rate\n"
              << "  packets that wait      : ";
    if (!rlcDiscards)
    {
        std::cout << "no sender discards a packet for waiting too long\n";
    }
    else if (rlcDiscardMs > 0)
    {
        std::cout << "gNB and UE discard a new packet while the oldest in the queue has waited over "
                  << rlcDiscardMs << " ms\n";
    }
    else
    {
        std::cout << "the gNB discards a new packet while the oldest in its queue has waited over "
                  << voicePdb.GetSeconds() * 1000.0 << " ms; a UE never does\n";
    }
    std::cout << std::setprecision(2);
    std::cout << "  satellite radio        : " << std::setprecision(0) << altitudeKm << " km, "
              << std::setprecision(2) << (satAsInTable ? "" : "changed from ") << "Set-" << satSet
              << " " << satColumn.orbit << " of TR 38.821\n"
              << "                           EIRP " << satEirpDbwPerMHz << " dBW/MHz = "
              << satEirpDbm << " dBm, antenna " << satAntennaGainDbi << " dBi, G/T " << satGtDbPerK
              << " dB/K (noise figure " << satNoiseFigureDb << " dB)\n";
    std::cout << "  satellite beam         : ";
    if (beamWidthGiven)
    {
        std::cout << std::setprecision(4) << beamHpbwDeg
                  << " deg wide at 3 dB on both carriers (--beamHpbwDeg)\n";
    }
    else
    {
        std::cout << satApertureM << " m aperture: " << std::setprecision(4)
                  << BeamWidthDegOfKa(g_scene.beamKa[0]) << " deg wide at 3 dB on the downlink, "
                  << BeamWidthDegOfKa(g_scene.beamKa[1]) << " deg on the uplink carrier\n";
    }
    std::cout << std::setprecision(1) << "  UE positions           : ";
    if (randomLayout)
    {
        std::cout << "UE 0 on the beam centre, the others at random places of the beam area\n"
                  << "  beam area              : " << areaNorthSouthKm << " km north-south x "
                  << areaEastWestKm << " km east-west, " << std::setprecision(0) << beamAreaKm2
                  << " km2\n";
    }
    else
    {
        std::cout << std::setprecision(2) << "in a line, UE i is i x " << spacingKm
                  << " km east of the beam centre\n";
    }
    std::cout << "  line of sight, shadow  : "
              << (fixedLargeScale
                      ? "each UE keeps its obstruction and shadowing levels for the whole run"
                      : "ns-3 draws them again: the line of sight of every UE once per second")
              << "\n";
    std::cout << std::setprecision(2);
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
        const double target = AttributeDefault<DoubleValue>("ns3::NrPdcch", "TargetBler");
        const double margin = AttributeDefault<DoubleValue>("ns3::NrPdcch", "SinrMarginDb");
        const double ulShare = AttributeDefault<DoubleValue>("ns3::NrPdcch", "UlMaxShare");
        const uint32_t sib1Ms = AttributeDefault<UintegerValue>("ns3::NrPdcch", "Sib1PeriodMs");
        std::cout << "  PDCCH                  : CORESET of " << pdcchView->GetCoresetRbs() << " RB x ";
        if (pdcchSymbols > 0)
        {
            std::cout << pdcchSymbols << " symbol" << (pdcchSymbols > 1 ? "s" : "") << " = "
                      << pdcchView->GetNumCce(pdcchSymbols) << " CCEs";
        }
        else
        {
            std::cout << "1, 2 or 3 symbols = " << pdcchView->GetNumCce(1) << ", "
                      << pdcchView->GetNumCce(2) << " or " << pdcchView->GetNumCce(3)
                      << " CCEs as the slot needs";
        }
        std::cout << "; a DCI is " << dciBits << " + 24 CRC bits\n"
                  << "  aggregation level      : the lowest that loses up to " << 100.0 * target
                  << " % of the DCIs at the SINR the UE reported";
        if (margin != 0.0)
        {
            std::cout << " less " << margin << " dB";
        }
        std::cout << ":\n" << std::string(27, ' ');
        const char* separator = "";
        std::ostringstream candidates;
        std::ostringstream levels;
        for (const uint8_t al : pdcchLevels)
        {
            if (pdcchView->GetCandidates(al) == 0 ||
                al > pdcchView->GetNumCce(pdcchView->GetMaxSymbols()))
            {
                continue;
            }
            const bool first = separator[0] == 0;
            std::cout << separator << (first ? "level " : "") << +al << " from "
                      << pdcchView->GetSinrForBler(al, target) + margin << " dB";
            candidates << (first ? "" : "/") << pdcchView->GetCandidates(al);
            levels << (first ? "" : "/") << +al;
            separator = ", ";
        }
        const uint32_t monitored = pdcchView->GetMonitoredCandidates() *
                                   (pdcchSymbols > 0 ? 1 : 3);
        std::cout << "\n  PDCCH candidates       : " << candidates.str() << " per UE at level "
                  << levels.str() << "; SIB1 and random access at level "
                  << +pdcchView->GetCommonAl() << "; a UE monitors " << monitored << " (limit "
                  << NrPdcch::GetMaxMonitoredCandidates(0) << ")\n"
                  << "  PDCCH order in a slot  : ";
        if (sib1Ms > 0)
        {
            std::cout << "SIB1 (every " << sib1Ms << " ms), ";
        }
        std::cout << "random access, uplink grants (" << 100.0 * ulShare
                  << " % of the CCEs if the downlink wants them too),\n"
                  << std::string(27, ' ')
                  << "downlink assignments; retransmissions first, then the UE served longest ago\n";
        // The voice bearers: a DCI for every block, or semi-persistent scheduling
        const uint32_t confirmBytes = spsScenario == 2 ? 6 : 1;
        const uint32_t ulMore = (spsScenario == 2 ? spsFitExtraBytes : 0) + 5 + confirmBytes;
        if (spsScenario == 0)
        {
            std::cout << "  voice scheduling       : dynamic: every block has a DCI "
                         "(--scheduling=sps1 or sps2: semi-persistent)\n";
        }
        else if (spsScenario == 1)
        {
            std::cout << "  voice scheduling       : semi-persistent, scenario 1: one configuration, "
                         "a block every 20 ms, kept through silence\n"
                      << std::string(27, ' ') << "downlink (SPS): " << spsTbBytes
                      << " B blocks; a larger SID packet goes in pieces, a larger voice packet "
                         "gets a DCI\n"
                      << std::string(27, ' ') << "uplink (configured grant Type 2): "
                      << spsTbBytes + ulMore << " B blocks = " << spsTbBytes
                      << " B + 5 B status report + 1 B confirmation MAC CE\n";
        }
        else
        {
            std::cout << "  voice scheduling       : semi-persistent, scenario 2: talk: a block every "
                         "20 ms; silence: every "
                      << sidIntervalMs << " ms; a change is 2 DCIs\n"
                      << std::string(27, ' ') << "downlink (SPS): " << spsTbBytes << " B and "
                      << spsSilenceTbBytes << " B blocks, up to " << spsFitExtraBytes
                      << " B larger for the first packet after a change\n"
                      << std::string(27, ' ') << "uplink (configured grant Type 2): "
                      << spsTbBytes + ulMore << " B and " << spsSilenceTbBytes + ulMore
                      << " B blocks = " << spsTbBytes << " B and " << spsSilenceTbBytes << " B + "
                      << spsFitExtraBytes << " B for a first packet\n"
                      << std::string(27, ' ') << "+ 5 B status report + 6 B confirmation MAC CE\n";
        }
    }
    if (idealControl)
    {
        std::cout << "  control channels       : ideal: no scheduling message is ever lost "
                     "(--idealControl=0: lost at low SINR)\n"
                  << "  radio link failure     : after " << n310 * qOutEvalSf
                  << " control symbols below " << qOutDb << " dB and " << t310.GetSeconds()
                  << " s more; the UE attaches again at once\n";
    }
    else if (rlmFromTable)
    {
        // TS 38.133 8.1.2: the two hypothetical PDCCHs of the radio link monitoring
        std::cout << "  control channels       : a DCI is lost at the block error rate of its "
                     "aggregation level at the SINR it arrives with\n"
                  << "  radio link failure     : when a level-8 PDCCH with 4 dB more power would "
                     "lose over 10 % (below "
                  << pdcchView->GetSinrForBler(8, 0.10) - 4.0 << " dB), over the last "
                  << qOutEvalSf << " ms,\n"
                  << std::string(27, ' ') << "and a level-4 PDCCH not under 2 % again (above "
                  << pdcchView->GetSinrForBler(4, 0.02) << " dB) within " << t310.GetSeconds()
                  << " s\n"
                  << "  after a failure        : no service until the level-4 PDCCH loses under 2 % "
                     "again, then the UE attaches again\n";
    }
    else
    {
        std::cout << "  control channels       : scheduling messages are lost at low SINR: 10 % at "
                  << qOutDb << " dB (Qout), 2 % at " << qInDb << " dB (Qin)\n"
                  << "  radio link failure     : when that rate, over the last " << qOutEvalSf
                  << " ms, is above 10 % and not back below 2 % within " << t310.GetSeconds()
                  << " s\n"
                  << "  after a failure        : no service until the rate is below 2 % again, then "
                     "the UE attaches again\n";
    }
    const bool gatewayAway = transparent && gatewayDiameterKm > 0.0;
    // The satellite seen from the gateway at time t: elevation and distance
    auto gatewayAt = [&](Time t) {
        const Vector sat = orbit.At(t);
        const Vector gw = gatewayMobility->GetPosition();
        const bool closer = (orbit.At(t + MilliSeconds(1)) - gw).GetLength() < (sat - gw).GetLength();
        return std::make_tuple(ElevationDeg(sat, gw), (sat - gw).GetLength() / 1000.0, closer);
    };
    if (transparent)
    {
        std::cout << "  gateway                : ";
        if (gatewayAway)
        {
            std::cout << std::setprecision(1) << "N " << gatewayPlace.northKm << " km, E "
                      << gatewayPlace.eastKm << " km from UE 0 (random in a circle of "
                      << std::setprecision(0) << gatewayDiameterKm << " km diameter)\n"
                      << std::setprecision(2);
        }
        else
        {
            std::cout << "where UE 0 is\n";
        }
        std::cout << "  feeder link            : gateway " << gatewayTxPowerDbm
                  << " dBm, satellite receive gain " << satFeederRxGainDbi << " dBi\n";
    }
    if (freezeGeometry)
    {
        const Ue0View now = ue0At(Seconds(0));
        std::cout << "  satellite              : stands still for the whole run "
                     "(--freezeGeometry=0 lets it fly)\n";
        std::cout << "  elevation at UE 0      : " << satMobility->GetElevationAngle(centreMobility)
                  << " deg (asked " << elevation << ")\n";
        std::cout << "  slant range to UE 0    : " << now.rangeKm << " km\n";
        if (gatewayAway)
        {
            const auto [gwElevationDeg, gwRangeKm, gwCloser] = gatewayAt(Seconds(0));
            std::cout << "  elevation at gateway   : " << gwElevationDeg << " deg\n"
                      << "  slant range to gateway : " << gwRangeKm << " km\n";
        }
        std::cout << "  one-way delay to UE 0  : " << delayParts(now) << "\n";
        std::cout << "  longest round trip     : " << maxLoopRtt.GetMicroSeconds() / 1000.0
                  << " ms on the radio  =>  K_offset " << kOffsetSlots << " slots\n";
    }
    else
    {
        // UE 0, on the beam centre, when its call starts and when it ends: the first line of the
        // second table
        const Time ue0CallEnd = callsStart + duration;
        const Ue0View first = ue0At(callsStart);
        const Ue0View last = ue0At(ue0CallEnd);
        std::cout << "  satellite              : flies south at " << orbit.vel0.GetLength() / 1000.0
                  << " km/s, is over UE 0 at t = " << overheadAt.GetSeconds()
                  << " s and down at 10 deg again at t = " << downAt.GetSeconds() << " s\n";
        if (fullPass)
        {
            std::cout << "                           (the whole pass: UE 0's call lasts "
                      << duration.GetSeconds() << " s; the other calls start when the satellite\n"
                      << "                           is 10 deg above their UE and end with UE 0's)."
                      << " Elevation: - before it is overhead, + after.\n";
        }
        else
        {
            std::cout << "                           (calls can last " << std::setprecision(0)
                      << std::floor((downAt - lastCallStart).GetSeconds()) << std::setprecision(2)
                      << " s). Elevation: - before it is overhead, + after.\n";
        }
        std::cout << "  UE 0's call            : starts at t = " << callsStart.GetSeconds()
                  << " s  ->  ends at t = " << ue0CallEnd.GetSeconds() << " s\n";
        std::cout << "  elevation at UE 0      : " << (callsStart < overheadAt ? "-" : "+")
                  << first.elevationDeg << " deg (asked " << elevation << ")  ->  "
                  << (ue0CallEnd < overheadAt ? "-" : "+") << last.elevationDeg << " deg\n";
        std::cout << "  slant range to UE 0    : " << first.rangeKm << " km  ->  " << last.rangeKm
                  << " km\n";
        if (gatewayAway)
        {
            const auto [elevation0, range0, closer0] = gatewayAt(callsStart);
            const auto [elevation1, range1, closer1] = gatewayAt(ue0CallEnd);
            std::cout << "  elevation at gateway   : " << (closer0 ? "-" : "+") << elevation0
                      << " deg  ->  " << (closer1 ? "-" : "+") << elevation1 << " deg\n"
                      << "  slant range to gateway : " << range0 << " km  ->  " << range1
                      << " km\n";
        }
        std::cout << "  one-way delay to UE 0  : " << delayParts(first) << "\n"
                  << "                       ->  " << delayParts(last) << "\n";
        std::cout << "  longest round trip     : " << maxLoopRtt.GetMicroSeconds() / 1000.0
                  << " ms on the radio, at t = " << maxLoopRttAt.GetSeconds()
                  << " s  =>  K_offset " << kOffsetSlots << " slots for the whole run\n";
    }
    if (ul)
    {
        std::cout << "  NOTE: the handheld uplink of this NR-NTN release is far below its link budget "
                     "(known limitation):\n"
                     "        expect loss and long delays in uplink calls.\n";
    }
    std::cout << "\n";

    if (pdcch)
    {
        // The PDCCH report covers the calls: from the start of the first to the end of the last
        Simulator::Schedule(callsStart, [cellId]() { NrPdcch::Get(cellId)->ResetStats(); });
        Simulator::Schedule(callsEnd, [cellId]() { NrPdcch::Get(cellId)->SetCounting(false); });
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

    // --- One line per call -------------------------------------------------------------------
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    // N[km] and E[km]: where the UE is, north and east of the beam centre.
    // SF[dB]: the shadow fading of the UE as a gain (negative: a weaker signal), the mean over
    // the call (it changes only when the satellite flies); "-" when ns-3 draws it.
    // lost = tx - rx: every generator stopped 2 s before the end, so nothing is still on its way.
    // beam[dB] is the mean over the call (it changes only when the satellite flies).
    std::vector<std::pair<uint32_t, uint32_t>> calls; // flow id, UE
    std::vector<std::pair<uint64_t, uint64_t>> callPackets; // of the same calls: sent, received
    // out[s]: how long of its call the UE had no connection, after a radio link failure.
    // DL SINR: the mean over the control symbols the UE received; it receives none while it
    // has no cell, so the time out of coverage after a failure is not in it.
    std::cout << "flow   UE address  N[km]  E[km]  beam[dB]  SF[dB]   VAF    tx    rx  lost  delay[ms]  "
                 "jitter[ms]  LOS[%]  RLF  out[s]  DL SINR[dB]\n";
    for (const auto& [flowId, st] : monitor->GetFlowStats())
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        if (t.destinationPort != port)
        {
            continue; // not a voice flow
        }
        Ipv4Address ueAddr = ul ? t.sourceAddress : t.destinationAddress;
        uint32_t i = 0;
        while (i < nUes && ueIfaces.GetAddress(i) != ueAddr)
        {
            ++i; // which UE is this flow?
        }
        calls.emplace_back(flowId, i);
        callPackets.emplace_back(st.txPackets, st.rxPackets);
        const CallView& view = g_view[i];
        Time out{0};
        for (const auto& [from, to] : g_outages[i])
        {
            out += Max(Time(0), Min(to, view.callEnd) - Max(from, view.callStart));
        }
        std::ostringstream address;
        address << ueAddr;
        std::cout << std::fixed << std::setw(4) << flowId << "  " << std::setw(11) << address.str()
                  << std::setprecision(1) << std::setw(7) << ueNorthKm[i] << std::setw(7)
                  << ueEastKm[i] << std::setw(10) << std::setprecision(2)
                  << view.taperDb.sum / view.taperDb.n << std::setw(8);
        if (view.shadowDb.n > 0)
        {
            std::cout << view.shadowDb.sum / view.shadowDb.n;
        }
        else
        {
            std::cout << "-";
        }
        std::cout << std::setw(6) << vaf[i] << std::setw(6) << st.txPackets << std::setw(6) << st.rxPackets
                  << std::setw(6) << st.txPackets - st.rxPackets << std::setprecision(3)
                  << std::setw(11)
                  << (st.rxPackets ? st.delaySum.GetSeconds() * 1000.0 / st.rxPackets : NAN)
                  << std::setw(12)
                  << (st.rxPackets > 1 ? st.jitterSum.GetSeconds() * 1000.0 / (st.rxPackets - 1)
                                       : NAN)
                  << std::setprecision(0) << std::setw(8)
                  << 100.0 * view.losYes / view.losAll << std::setw(5) << g_rlf[i]
                  << std::setprecision(2) << std::setw(8) << out.GetSeconds()
                  << std::setprecision(1) << std::setw(13)
                  << (g_dlSinrN[i] ? 10 * std::log10(g_dlSinrSum[i] / g_dlSinrN[i]) : NAN) << "\n";
    }

    // --- Why the packets were lost ---------------------------------------------------------------
    // NrPdcp counts, for every UE and direction, the packets its sending PDCP entities were
    // given, those that reached the receiving PDCP entities and those passed on from there:
    //   no conn.  sent by the application and never given to a PDCP entity: the UE had no
    //             connection (after a radio link failure), or no bearer yet
    //   discard   given to PDCP, but the RLC of the sender did not take it: the oldest packet
    //             in its queue had waited longer than the discard timer (see "packets that
    //             wait" in the header of the output)
    //   radio     sent, and never arrived at the other end: lost on the air (every HARQ
    //             transmission failed), or still waiting in the RLC when the connection broke.
    //             Rarely one arrives damaged, which NrPdcp drops (PacketCounts::damaged): the
    //             RLC cannot always tell, after a long burst of losses, that two pieces are
    //             not of the same packet. A line under the table says so.
    //   ROHC      arrived, but the decompressor had lost its context and could not restore the
    //             header; it dropped the packet
    // The first three are lost with or without header compression ("link"); the fourth is what
    // ROHC cost. A packet lost anywhere else would make the three not add up to "lost", and is
    // reported in a line of its own.
    std::cout << "\nLost packets by cause, in packets and in % of the packets sent. link = no conn. "
                 "+ discard + radio: lost with or without ROHC.\n"
              << "no conn.: the UE had no connection, the packet never got a radio bearer. discard: "
                 "the sender did not take it, the oldest\n"
              << "packet in its queue had waited too long. radio: sent on a bearer, did not arrive. "
                 "ROHC: arrived, but the decompressor had\n"
              << "lost its context and could not restore the header.\n"
              << "flow   UE address    tx    rx  lost  no conn.  discard  radio  ROHC  link[%]  "
                 "ROHC[%]\n";
    // The same counts, kept for the table after this one
    struct Causes
    {
        uint64_t tx{0};
        uint64_t rx{0};
        uint64_t noConnection{0};
        uint64_t onBearer{0}; // given to the PDCP entity of a bearer
        uint64_t discarded{0};
        uint64_t radio{0};
        uint64_t rohc{0};
    };
    std::vector<Causes> causes;
    {
        auto line = [&](const std::string& first, const Causes& c) {
            std::cout << first << std::setw(6) << c.tx << std::setw(6) << c.rx << std::setw(6)
                      << c.tx - c.rx << std::setw(10) << c.noConnection << std::setw(9) << c.discarded
                      << std::setw(7) << c.radio << std::setw(6)
                      << (rohc ? std::to_string(c.rohc) : "-") << std::setprecision(2) << std::setw(9);
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
        for (std::size_t k = 0; k < calls.size(); ++k)
        {
            const auto [flowId, i] = calls[k];
            const NrPdcp::PacketCounts n = NrPdcp::GetPacketCounts(ueIfaces.GetAddress(i), !ul);
            Causes c;
            c.tx = callPackets[k].first;
            c.rx = callPackets[k].second;
            c.noConnection = c.tx - n.sdus;
            c.onBearer = n.sdus;
            c.discarded = n.discarded;
            c.radio = n.sent - n.discarded - n.arrived;
            c.rohc = (n.sdus - n.sent) + (n.arrived - n.delivered); // compressor, decompressor
            std::ostringstream address;
            address << ueIfaces.GetAddress(i);
            std::ostringstream first;
            first << std::setw(4) << flowId << "  " << std::setw(11) << address.str();
            line(first.str(), c);
            if (n.delivered != c.rx || n.sdus > c.tx)
            {
                notes << "  NOTE: flow " << flowId << ": PDCP was given " << n.sdus
                      << " packets and passed on " << n.delivered
                      << ", which does not fit tx and rx: packets were lost outside the radio "
                         "bearers\n";
            }
            if (n.damaged > 0)
            {
                notes << "  NOTE: flow " << flowId << ": " << n.damaged
                      << " of the radio losses arrived, but damaged: the RLC had joined pieces "
                         "of two packets (NrPdcp drops these)\n";
            }
            causes.push_back(c);
            all.tx += c.tx;
            all.rx += c.rx;
            all.noConnection += c.noConnection;
            all.discarded += c.discarded;
            all.radio += c.radio;
            all.rohc += c.rohc;
        }
        if (calls.size() > 1)
        {
            line(" all" + std::string(13, ' '), all);
        }
        std::cout << notes.str();
    }

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
    std::cout << "\nVoice against 5QI 1 (TS 23.501 Table 5.7.4-1): at most " << std::setprecision(0)
              << voicePdb.GetSeconds() * 1000.0
              << " ms between UE and UPF (the PGW, without the N6 link) for 98 % of the\n"
              << "packets, at most " << voicePer * 100.0
              << " % of them lost. on bearer: the packets the voice bearer was given. in PDB, "
                 "late: delivered within, after\n"
              << "the budget; late is not lost. lost: radio + ROHC. over PDB[%] = late + discard, "
                 "PER[%] = lost, both in % of on bearer.\n"
              << "Delay: UE to UPF, of the packets delivered; 98%: what 98 % of them stayed "
                 "within. kbit/s: the highest mean over "
              << voiceAveragingWindow.GetSeconds() << " s.\n"
              << "missed: PDB if over PDB is above 2 %, PER if PER is above " << voicePer * 100.0
              << " %.\n"
              << "flow   UE address  on bearer  in PDB  late  discard  lost  over PDB[%]  PER[%]  "
                 "mean[ms]  98%[ms]  max[ms]  kbit/s  missed\n";
    {
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
        auto line = [&](const std::string& first, const Row& r, const std::string& missed) {
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
                std::cout << std::setw(10) << "-" << std::setw(9) << "-" << std::setw(9) << "-";
            }
            else
            {
                int64_t sum = 0;
                for (const int64_t d : r.delayNs)
                {
                    sum += d;
                }
                // the smallest delay that at least 98 % of the packets did not exceed
                const std::size_t k98 = (delivered * 98 + 99) / 100 - 1;
                std::cout << std::setprecision(3) << std::setw(10) << sum / 1e6 / delivered
                          << std::setw(9) << r.delayNs[k98] / 1e6 << std::setw(9)
                          << r.delayNs.back() / 1e6;
            }
            std::cout << std::setprecision(1) << std::setw(8) << r.kbps << "  "
                      << (missed.empty() ? "-" : missed) << "\n";
        };
        Row all;
        uint32_t callsThatMissed = 0;
        bool aboveGfbr = false;
        std::ostringstream notes;
        for (std::size_t k = 0; k < calls.size(); ++k)
        {
            const auto [flowId, i] = calls[k];
            VoiceQos& q = g_voiceQos.calls[i];
            Row r;
            r.c = causes[k];
            r.delayNs = std::move(q.delayNs);
            std::sort(r.delayNs.begin(), r.delayNs.end());
            r.inPdb = std::upper_bound(r.delayNs.begin(), r.delayNs.end(), pdbNs) - r.delayNs.begin();
            r.kbps = q.peakWindowBits / voiceAveragingWindow.GetSeconds() / 1000.0;
            aboveGfbr = aboveGfbr || r.kbps > gfbrKbps;
            std::ostringstream address;
            address << ueIfaces.GetAddress(i);
            std::ostringstream first;
            first << std::setw(4) << flowId << "  " << std::setw(11) << address.str();
            const std::string missed = missedBy(r);
            callsThatMissed += !missed.empty();
            line(first.str(), r, missed);
            if (r.delayNs.size() != r.c.rx || q.unstamped > 0)
            {
                notes << "  NOTE: flow " << flowId << ": the delay of " << r.delayNs.size() << " of "
                      << r.c.rx << " received packets was measured\n";
            }
            all.c.onBearer += r.c.onBearer;
            all.c.discarded += r.c.discarded;
            all.c.radio += r.c.radio;
            all.c.rohc += r.c.rohc;
            all.inPdb += r.inPdb;
            all.delayNs.insert(all.delayNs.end(), r.delayNs.begin(), r.delayNs.end());
            all.kbps = std::max(all.kbps, r.kbps);
        }
        if (calls.size() > 1)
        {
            std::sort(all.delayNs.begin(), all.delayNs.end());
            line(" all" + std::string(13, ' '),
                 all,
                 std::to_string(callsThatMissed) + (callsThatMissed == 1 ? " call" : " calls"));
        }
        if (aboveGfbr)
        {
            notes << "  NOTE: a call sent more than its GFBR of " << gfbrKbps
                  << " kbit/s; the 98 % rule is for a flow that keeps to it (--gfbrKbps)\n";
        }
        std::cout << notes.str();
    }

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
        std::cout << std::fixed << std::setprecision(2) << "\nPDCCH of the cell from t = "
                  << callsStart.GetSeconds() << " s to " << callsEnd.GetSeconds() << " s: "
                  << stats.slots << " slots, of which " << slotsWithPdcch << " ("
                  << percent(slotsWithPdcch, stats.slots) << " %) carried a DCI.\n";
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
                         "A slot without a DCI counts 1 symbol of CCEs.\n";
        }
        std::cout << "CCEs: " << stats.cceUsed << " used of " << stats.cceAvailable
                  << " = utilization " << percent(stats.cceUsed, stats.cceAvailable) << " %; "
                  << dcis << " DCIs, "
                  << (dcis ? static_cast<double>(stats.cceUsed) / dcis : 0.0) << " CCEs each.\n"
                  << "blocked: a DCI that was not sent, none of its candidates had all CCEs free; "
                     "its UE was not scheduled in that slot.\n"
                  << "purpose                     DCIs    CCEs  CCEs[%]  blocked  blocked[%]\n";
        auto purposeLine = [&](const std::string& name, uint64_t n, uint64_t cces, uint64_t stopped) {
            std::cout << std::left << std::setw(25) << name << std::right << std::setw(7) << n
                      << std::setw(8) << cces << std::setw(9) << percent(cces, stats.cceUsed)
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

        const double target = AttributeDefault<DoubleValue>("ns3::NrPdcch", "TargetBler");
        const double margin = AttributeDefault<DoubleValue>("ns3::NrPdcch", "SinrMarginDb");
        std::cout << "from[dB]: the level is used from this reported SINR on. missed: DCIs their UE "
                     "did not decode"
                  << (idealControl ? " (--idealControl=1: none can be)" : "") << ".\n"
                  << "level    DCIs  DCIs[%]    CCEs  CCEs[%]  from[dB]  missed  missed[%]\n";
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
                      << std::setw(9) << percent(stats.cceByAl[i], stats.cceUsed) << std::setw(10)
                      << pdcchView->GetSinrForBler(al, target) + margin << std::setw(8)
                      << stats.missedByAl[i] << std::setw(11)
                      << percent(stats.missedByAl[i], stats.rxByAl[i]) << "\n";
        }
        std::cout << "  all" << std::setw(8) << dcis << std::setw(9) << 100.0 << std::setw(8)
                  << stats.cceUsed << std::setw(9) << 100.0 << std::setw(10) << "" << std::setw(8)
                  << missed << std::setw(11) << percent(missed, received) << "\n";
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
            std::cout << "  NOTE: " << stats.ulCapped << " of the blocked uplink grants were held "
                      << "back to leave CCEs to the downlink (NrPdcch::UlMaxShare)\n";
        }
        if (stats.unusedReservations > 0)
        {
            std::cout << "  NOTE: " << stats.unusedReservations << " PDCCH candidates were kept "
                      << "for a UE that then got no data resources in the slot\n";
        }

        if (spsScenario != 0)
        {
            // Semi-persistent scheduling: the blocks that needed no DCI
            std::cout << "\nSemi-persistent scheduling, scenario " << spsScenario
                      << ": the blocks of the voice bearers that needed no DCI, and the DCIs that "
                         "set them up.\n"
                      << "used: carried data. empty: an occasion with nothing to send. in pieces: a "
                         "packet larger than the block, sent over this\n"
                      << "occasion and the next. own DCI: a packet larger than the block that was "
                         "sent with a DCI instead. lost: blocks the UE\n"
                      << "did not know of, it had missed the DCI that activated the "
                         "configuration.\n"
                      << "direction  activations  releases    used   empty  empty[%]  in pieces  "
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
                std::cout << std::left << std::setw(9) << name << std::right << std::setw(13)
                          << stats.dci[activation] << std::setw(10) << stats.dci[release]
                          << std::setw(8) << nUsed << std::setw(8) << nEmpty << std::setw(10)
                          << percent(nEmpty, nUsed + nEmpty) << std::setw(11)
                          << stats.events[segment] << std::setw(9) << stats.events[large]
                          << std::setw(8) << stats.events[lost] << "\n";
            };
            spsLine("downlink",
                    NrPdcch::DL_SPS_ACTIVATION,
                    NrPdcch::DL_SPS_RELEASE,
                    NrPdcch::DL_SPS_BLOCK,
                    NrPdcch::DL_SPS_EMPTY,
                    NrPdcch::DL_SPS_SEGMENT,
                    NrPdcch::DL_SPS_LARGE,
                    NrPdcch::DL_SPS_LOST_CONFIG);
            spsLine("uplink",
                    NrPdcch::UL_CG_ACTIVATION,
                    NrPdcch::UL_CG_RELEASE,
                    NrPdcch::UL_CG_BLOCK,
                    NrPdcch::UL_CG_EMPTY,
                    NrPdcch::UL_CG_SEGMENT,
                    NrPdcch::UL_CG_LARGE,
                    NrPdcch::UL_CG_LOST_CONFIG);
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
        }
    }

    // --- What each UE saw of the satellite during its call, sampled every 100 ms -----------------
    // The delay is the propagation delay of the whole path, one way: N6 + feeder + service.
    if (freezeGeometry)
    {
        std::cout << "\nSatellite seen from each UE: the same for the whole call, it stands still.\n"
                  << "Delay: one way, " << path << ", propagation only.\n"
                  << "flow   UE address  elevation[deg]  range[km]  delay[ms]\n";
    }
    else
    {
        // " ---- label ----", exactly as wide as the four columns below it
        auto group = [](const std::string& label, std::size_t width) {
            const std::size_t dashes = width - label.size() - 3;
            return " " + std::string(dashes / 2, '-') + " " + label + " " +
                   std::string(dashes - dashes / 2, '-');
        };
        std::cout << "\nSatellite seen from each UE during its call.\n"
                  << "Elevation: - before the satellite is overhead, + after; low and high are the "
                     "lowest and highest of the call.\n"
                  << "Delay: one way, " << path << ", propagation only. from[s]: when the call "
                     "started.\n"
                  << std::string(26, ' ') << group("elevation [deg]", 32)
                  << group("slant range [km]", 40) << group("delay [ms]", 32) << "\n"
                  << "flow   UE address  from[s]";
        for (const char* column : {"start", "end", "low", "high"})
        {
            std::cout << std::setw(8) << column;
        }
        for (int width : {10, 8})
        {
            for (const char* column : {"start", "end", "min", "max"})
            {
                std::cout << std::setw(width) << column;
            }
        }
        std::cout << "\n";
    }
    for (const auto& [flowId, i] : calls)
    {
        const CallView& view = g_view[i];
        std::ostringstream address;
        address << ueIfaces.GetAddress(i);
        std::cout << std::fixed << std::setw(4) << flowId << "  " << std::setw(11) << address.str();
        if (freezeGeometry)
        {
            std::cout << std::setprecision(2) << std::setw(16) << view.elevationDeg.first
                      << std::setw(11) << view.rangeKm.first << std::setprecision(3)
                      << std::setw(11) << view.delayMs.first << "\n";
            continue;
        }
        std::cout << std::setprecision(2) << std::setw(9) << view.callStart.GetSeconds()
                  << std::showpos << std::setw(8)
                  << view.sideAtStart * view.elevationDeg.first << std::setw(8)
                  << view.sideAtEnd * view.elevationDeg.last << std::noshowpos << std::setw(8)
                  << view.elevationDeg.min << std::setw(8) << view.elevationDeg.max;
        for (double v : {view.rangeKm.first, view.rangeKm.last, view.rangeKm.min, view.rangeKm.max})
        {
            std::cout << std::setw(10) << v;
        }
        std::cout << std::setprecision(3);
        for (double v : {view.delayMs.first, view.delayMs.last, view.delayMs.min, view.delayMs.max})
        {
            std::cout << std::setw(8) << v;
        }
        std::cout << "\n";
    }

    g_scene = Scene(); // let go of the nodes before the simulator is destroyed
    Simulator::Destroy();
    return 0;
}
