// Copyright (c) 2011-2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>

#include "nr-pdcp.h"

#include "nr-pdcp-header.h"
#include "nr-pdcp-sap.h"
#include "nr-pdcp-tag.h"
#include "nr-rlc-tag.h"

#include "ns3/abort.h"
#include "ns3/boolean.h"
#include "ns3/enum.h"
#include "ns3/global-value.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/tag.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <typeinfo>
#include <utility>
#include <vector>

#ifdef NR_ROHC
#include <rohc/rohc.h>
#include <rohc/rohc_comp.h>
#include <rohc/rohc_decomp.h>
#endif

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrPdcp");

/// NrPdcpSpecificNrRlcSapUser class
class NrPdcpSpecificNrRlcSapUser : public NrRlcSapUser
{
  public:
    /**
     * Constructor
     *
     * @param pdcp PDCP
     */
    NrPdcpSpecificNrRlcSapUser(NrPdcp* pdcp);

    // Interface provided to lower RLC entity (implemented from NrRlcSapUser)
    void ReceivePdcpPdu(Ptr<Packet> p) override;

  private:
    NrPdcpSpecificNrRlcSapUser();
    NrPdcp* m_pdcp; ///< the PDCP
};

NrPdcpSpecificNrRlcSapUser::NrPdcpSpecificNrRlcSapUser(NrPdcp* pdcp)
    : m_pdcp(pdcp)
{
}

NrPdcpSpecificNrRlcSapUser::NrPdcpSpecificNrRlcSapUser()
{
}

void
NrPdcpSpecificNrRlcSapUser::ReceivePdcpPdu(Ptr<Packet> p)
{
    m_pdcp->DoReceivePdu(p);
}

///////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(NrPdcp);

// What is printed about ROHC when the simulation is destroyed: none, summary or full. Summary:
// three tables with one line per bearer and direction (what the compressor sent; what reached
// the decompressor and what it could restore; the mode of the compressor over time). Full: one
// block per bearer and direction, with every mode change, context loss and feedback. Summary
// and full end with a table of the ROHC packet types: their share per bearer and their mean
// header size. Set it with GlobalValue::Bind("NrRohcReport", StringValue("full")) or
// --NrRohcReport=full.
static GlobalValue g_nrRohcReport(
    "NrRohcReport",
    "ROHC report at the end of the simulation: none, summary or full",
    StringValue("summary"),
    MakeStringChecker());

// ---------------------------------------------------------------------------------------------
// What became of the packets of every data radio bearer, with or without ROHC. The sending PDCP
// entity of a bearer and the receiving one write into the same record, as they do for the ROHC
// report below; NrPdcp::GetPacketCounts() adds the records of a UE up.
// ---------------------------------------------------------------------------------------------
namespace
{

struct BearerCounts
{
    uint32_t ue = 0; // IPv4 address of the UE of the bearer, 0 until a packet has told
    NrPdcp::PacketCounts n;
};

// A data radio bearer: RNTI, LCID, downlink, and the cell (two cells may give the same RNTI)
using BearerKey = std::tuple<uint16_t, uint8_t, bool, uint16_t>;

std::map<BearerKey, BearerCounts> g_bearerCounts;
bool g_bearerCountsClearScheduled = false;
// The bearer whose data PDU is being handed to the RLC at this moment, null at any other time:
// the RLC says through NrPdcp::NotePduDiscarded() if it does not take the PDU
BearerCounts* g_bearerHandingOver = nullptr;

// The SDUs the gNB had on each bearer, by kind: for NrPdcp::GetLastSduKind() and
// GetVoiceActivity()
std::map<BearerKey, NrPdcp::VoiceActivity> g_voiceActivity;

// The RLC PDUs the PDU being received came in: NrPdcp::NoteSduPdus()
uint32_t g_sduPdus = 1;

// One more SDU of a bearer: a silence descriptor or a voice frame
void
NoteSduKind(uint16_t cellId, uint16_t rnti, uint8_t lcid, bool downlink, bool silence)
{
    NrPdcp::VoiceActivity& activity = g_voiceActivity[BearerKey(rnti, lcid, downlink, cellId)];
    const NrPdcp::SduKind kind = silence ? NrPdcp::SDU_SILENCE : NrPdcp::SDU_TALK;
    if (activity.last != NrPdcp::SDU_UNKNOWN && activity.last != kind)
    {
        ++(silence ? activity.toSilence : activity.toVoice);
        if (!downlink)
        {
            (silence ? activity.toSilencePdus : activity.toVoicePdus) += g_sduPdus;
        }
    }
    activity.last = kind;
    ++activity.sdus;
}

void
ClearBearerCounts()
{
    g_bearerCounts.clear();
    g_voiceActivity.clear();
    g_bearerCountsClearScheduled = false;
    g_bearerHandingOver = nullptr;
}

BearerCounts&
GetBearerCounts(uint16_t cellId, uint16_t rnti, uint8_t lcid, bool downlink)
{
    if (!g_bearerCountsClearScheduled)
    {
        Simulator::ScheduleDestroy(&ClearBearerCounts); // the next simulation starts at zero
        g_bearerCountsClearScheduled = true;
    }
    return g_bearerCounts[BearerKey(rnti, lcid, downlink, cellId)];
}

// What the simulator knows and a real receiver does not: whether a data PDU that arrives is the
// one that was sent. RLC UM numbers its PDUs with few bits; after a burst of lost PDUs as long
// as that number space it cannot tell, and can put the first piece of one packet together with
// the last piece of another. A real PDCP entity would pass that packet on, and the UDP checksum
// of the receiver would reject it. Here the sending entity tags the body of each data PDU with
// a number, its length and a hash, and the receiving entity drops a PDU that is not, from the
// first byte to the last, one tagged body (`PacketCounts::damaged`), so that what PDCP,
// FlowMonitor and the application count as received is the same.
class NrPdcpCheckTag : public Tag
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NrPdcpCheckTag")
                                .SetParent<Tag>()
                                .SetGroupName("Nr")
                                .AddConstructor<NrPdcpCheckTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    uint32_t GetSerializedSize() const override
    {
        return 12;
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU32(m_number);
        i.WriteU32(m_bytes);
        i.WriteU32(m_hash);
    }

    void Deserialize(TagBuffer i) override
    {
        m_number = i.ReadU32();
        m_bytes = i.ReadU32();
        m_hash = i.ReadU32();
    }

    void Print(std::ostream& os) const override
    {
        os << "PDCP PDU body " << m_number << " of " << m_bytes << " bytes, hash " << m_hash;
    }

    uint32_t m_number{0}; ///< a number no other tagged PDU of the simulation has
    uint32_t m_bytes{0};  ///< length of the body of the PDU as it was sent
    uint32_t m_hash{0};   ///< FNV-1a hash of these bytes
};

NS_OBJECT_ENSURE_REGISTERED(NrPdcpCheckTag);

// The tag that fits the bytes of a packet, with a number of its own
NrPdcpCheckTag
MakePdcpCheckTag(Ptr<const Packet> p)
{
    static uint32_t number = 0;
    std::vector<uint8_t> bytes(p->GetSize());
    p->CopyData(bytes.data(), bytes.size());
    NrPdcpCheckTag tag;
    tag.m_number = ++number;
    tag.m_bytes = bytes.size();
    tag.m_hash = 2166136261u;
    for (const uint8_t byte : bytes)
    {
        tag.m_hash = (tag.m_hash ^ byte) * 16777619u;
    }
    return tag;
}

// Whether a PDU body is one that was sent. It is if it has no check tag at all (the sender did
// not count it), or if one check tag fits its bytes and lies on all of them. The RLC cuts a
// PDU into pieces and joins them again, and each piece keeps the tag on its own bytes: a body
// joined from pieces of two PDUs has two tags, each on a part. (A packet that crossed another
// bearer before may carry the tag of that one as well, which does not fit.)
bool
IsPdcpPduIntact(Ptr<const Packet> p)
{
    const NrPdcpCheckTag now = MakePdcpCheckTag(p);
    bool tagged = false;
    std::map<uint32_t, uint32_t> covered; // number of a fitting tag -> bytes it lies on
    ByteTagIterator i = p->GetByteTagIterator();
    while (i.HasNext())
    {
        ByteTagIterator::Item item = i.Next();
        if (item.GetTypeId() == NrPdcpCheckTag::GetTypeId())
        {
            NrPdcpCheckTag sent;
            item.GetTag(sent);
            tagged = true;
            if (sent.m_bytes == now.m_bytes && sent.m_hash == now.m_hash)
            {
                covered[sent.m_number] += item.GetEnd() - item.GetStart();
                if (covered[sent.m_number] == now.m_bytes)
                {
                    return true;
                }
            }
        }
    }
    return !tagged;
}

// A PDCP entity does not know whether it is in the gNB or in the UE, but its SAP user does: the
// UE RRC installs NrPdcpSpecificNrPdcpSapUser<NrUeRrc>, the gNB installs
// NrPdcpSpecificNrPdcpSapUser<NrUeManager>.
bool
IsUeSide(const NrPdcpSapUser* user)
{
    return user && std::string(typeid(*user).name()).find("NrUeRrc") != std::string::npos;
}

} // namespace

NrPdcp::PacketCounts
NrPdcp::GetPacketCounts(Ipv4Address ue, bool downlink)
{
    PacketCounts sum;
    for (const auto& [key, counts] : g_bearerCounts)
    {
        if (std::get<2>(key) == downlink && counts.ue != 0 && counts.ue == ue.Get())
        {
            sum.sdus += counts.n.sdus;
            sum.sent += counts.n.sent;
            sum.discarded += counts.n.discarded;
            sum.arrived += counts.n.arrived;
            sum.delivered += counts.n.delivered;
            sum.damaged += counts.n.damaged;
        }
    }
    return sum;
}

NrPdcp::SduKind
NrPdcp::GetLastSduKind(uint16_t cellId, uint16_t rnti, uint8_t lcid, bool downlink)
{
    return GetVoiceActivity(cellId, rnti, lcid, downlink).last;
}

NrPdcp::VoiceActivity
NrPdcp::GetVoiceActivity(uint16_t cellId, uint16_t rnti, uint8_t lcid, bool downlink)
{
    const auto it = g_voiceActivity.find(BearerKey(rnti, lcid, downlink, cellId));
    return it == g_voiceActivity.end() ? VoiceActivity() : it->second;
}

void
NrPdcp::NoteSduPdus(uint32_t pdus)
{
    g_sduPdus = pdus;
}

void
NrPdcp::NotePduDiscarded()
{
    if (g_bearerHandingOver)
    {
        g_bearerHandingOver->n.discarded++;
    }
}

#ifdef NR_ROHC
// ---------------------------------------------------------------------------------------------
// The size-table model of ROHC (`RohcModel` = Table): no compressor runs. The IPv4/UDP[/RTP]
// header chain of a packet is replaced by as many bytes as the table says, so everything below
// PDCP carries a packet of the right size, and the receiving entity puts the header chain back
// if its context allows.
//
// The table (compressor):
//   IR            the header chain minus one byte (what RFC 3095 needs for static plus dynamic
//                 chain: 39 bytes for IP/UDP/RTP, 27 for IP/UDP). `RohcTableIrPackets` of them in
//                 a row at the start of a flow; one on a periodic IR refresh, one after a
//                 STATIC-NACK.
//   UOR-2-EXT-3   a context update, `RohcTableUpdateSize` bytes (13): the first packet whose
//                 length differs from the one before, or with the RTP marker bit (for a voice
//                 source with DTX: the first SID packet after the hangover and the first voice
//                 packet after a silence); also one on a periodic FO refresh and one after a NACK.
//   UO-0          every other packet, `RohcTableUo0Size` bytes (3)
//
// The context (decompressor), as W-LSB decoding of the sequence number behaves: every packet has
// a number; "gap" is how many packets are missing between the last packet that was restored and
// the one that arrives.
//   IR            always restored; static and dynamic context are valid afterwards
//   UOR-2-EXT-3   restored if the static context is valid and gap <= `RohcTableStaticWindow`;
//                 the dynamic context is valid afterwards
//   UO-0          restored if both contexts are valid and gap <= `RohcTableDynamicWindow`
//   A gap above the dynamic window makes the dynamic context invalid (a context update or an IR
//   repairs it), a gap above the static window the static context too (only an IR repairs it).
//   Packets that are not restored are dropped; they do not move the reference, so the gap grows.
//
// The repair, by `RohcMode` (the mode of the decompressor; the compressor starts in U-mode and
// is in O-mode from the first feedback it receives, RFC 3095 5.6.1):
//   U   no feedback. The compressor sends one IR `RohcIrRefresh` packets after the last IR, and
//       a context update `RohcFoRefresh` packets after the last update (and by time, if
//       `RohcIrRefreshTime` is set); a damaged context waits for the next of these or for the
//       next regular update.
//   O   feedback: RFC 3095 FEEDBACK-2 with the CRC option, 5 bytes, in a PDCP control PDU on the
//       reverse link, where it can be late or lost. The decompressor answers every IR packet
//       with an ACK. After `RohcNackFailures` packets in a row that it could not restore, and
//       again every `RohcFeedbackSpacing` packets while that lasts, it sends a NACK (dynamic
//       context) or a STATIC-NACK (static context). The compressor answers a NACK with one
//       context update, a STATIC-NACK with one IR.
//
// The mode of the compressor (RFC 3095 5.6.2): all feedback of the decompressor says "O-mode"
// and has the CRC option, which a mode request needs. The compressor is in O-mode, and stops its
// periodic refreshes, as soon as any of it arrives: normally the ACK of the first IR packet. If
// that ACK is lost on the reverse link, the compressor stays in U-mode until other feedback
// arrives: a NACK or STATIC-NACK, or the ACK of the IR packet of its next refresh. It does not
// go back to U-mode. A new PDCP entity, as a UE gets with every new connection after a radio
// link failure, starts in U-mode again (TS 38.323 5.1.2: ROHC is reset when PDCP is
// re-established).
//
// The header chain travels in a byte tag, as ns-3 tags do: unseen by the layers below, which
// see only the size.
// ---------------------------------------------------------------------------------------------
class NrRohcSizeTableTag : public Tag
{
  public:
    /// Rows of the table
    enum Type : uint8_t
    {
        UO_0 = 0,
        UPDATE = 1,
        IR = 2
    };

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NrRohcSizeTableTag")
                                .SetParent<Tag>()
                                .SetGroupName("Nr")
                                .AddConstructor<NrRohcSizeTableTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    uint32_t GetSerializedSize() const override
    {
        return 7 + m_header.size();
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU8(m_type);
        i.WriteU32(m_number);
        i.WriteU8(m_compressedSize);
        i.WriteU8(m_header.size());
        i.Write(m_header.data(), m_header.size());
    }

    void Deserialize(TagBuffer i) override
    {
        m_type = i.ReadU8();
        m_number = i.ReadU32();
        m_compressedSize = i.ReadU8();
        m_header.resize(i.ReadU8());
        i.Read(m_header.data(), m_header.size());
    }

    void Print(std::ostream& os) const override
    {
        os << "packet " << m_number << ", header chain of " << m_header.size() << " bytes sent as "
           << +m_compressedSize;
    }

    uint8_t m_type{UO_0};          ///< the row of the table
    uint32_t m_number{0};          ///< the number of the packet in its flow
    uint8_t m_compressedSize{0};   ///< bytes that stand for the header chain in the PDU
    std::vector<uint8_t> m_header; ///< the header chain
};

NS_OBJECT_ENSURE_REGISTERED(NrRohcSizeTableTag);

// ---------------------------------------------------------------------------------------------
// What the simulator knows and a real decompressor does not: whether the ROHC library gave back
// the packet that was sent. A small ROHC packet is protected by a 3-bit CRC, so after a burst of
// lost packets the library can take a wrong sequence number for the right one (one wrong guess
// in eight passes the CRC) and return a packet with a wrong IP identification, RTP sequence
// number and timestamp. It would pass such a packet on. The receiver cannot use it: its UDP
// checksum, which travels unchanged, does not fit, and ns-3's UDP drops it. The sending entity
// therefore tags each packet with a hash of what it was, and the receiving entity drops a packet
// that comes back different and counts it as failed ("header restored wrongly").
// ---------------------------------------------------------------------------------------------
class NrRohcCheckTag : public Tag
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NrRohcCheckTag")
                                .SetParent<Tag>()
                                .SetGroupName("Nr")
                                .AddConstructor<NrRohcCheckTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    uint32_t GetSerializedSize() const override
    {
        return 4;
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU32(m_hash);
    }

    void Deserialize(TagBuffer i) override
    {
        m_hash = i.ReadU32();
    }

    void Print(std::ostream& os) const override
    {
        os << "hash of the packet before ROHC " << m_hash;
    }

    uint32_t m_hash{0}; ///< RohcPacketHash() of the packet before compression
};

NS_OBJECT_ENSURE_REGISTERED(NrRohcCheckTag);

// ---------------------------------------------------------------------------------------------
// ROHC (RFC 3095 and following) on data radio bearers, done by the ROHC library
// (https://rohc-lib.org). As TS 38.323 5.7 has it, a PDCP entity owns at most one compressor
// (for what it sends) and one decompressor (for what it receives), configured with MAX_CID and
// the allowed profiles. In O-mode the decompressor's feedback travels back to the peer
// compressor in PDCP control PDUs for interspersed ROHC feedback (TS 38.323 5.7.6, 6.2.3.2).
//
// One report per direction of each data radio bearer that ROHC actually worked on. The
// compressing PDCP entity (sender) and the decompressing one (receiver) of the same bearer write
// into the same report, so a downlink voice bearer gives exactly one report. The summary prints
// it as three tables: what the compressors sent; what reached the decompressors and what they
// could restore of it; and the mode of each compressor over time.
// ---------------------------------------------------------------------------------------------
namespace
{

struct RohcReport
{
    // Compressor (sender side). Index = compressor state: 1 IR, 2 FO, 3 SO (0 = unknown).
    uint64_t packets[4] = {0, 0, 0, 0};
    uint64_t hdrIn[4] = {0, 0, 0, 0};  // uncompressed header bytes (IP + UDP [+ RTP])
    uint64_t hdrOut[4] = {0, 0, 0, 0}; // ROHC header bytes
    uint64_t bytesIn = 0;              // whole packets before compression
    uint64_t bytesOut = 0;             // whole ROHC packets
    uint64_t compressFailed = 0;       // rohc_compress4() failed, packet dropped
    double firstTx = -1;               // s
    double lastTx = -1;                // s
    std::string profile;
    std::string mode;
    std::set<std::string> flows;
    std::string ue;        // IPv4 address of the UE of the bearer, "" until a packet has told
    uint32_t ueNumber = 0; // the same as a number, to sort by
    struct TypeCount
    {
        uint64_t packets = 0;
        uint64_t hdrOut = 0; // ROHC header bytes
    };
    std::map<std::string, TypeCount> packetTypes; // by the library's name of the packet type
    std::vector<std::pair<double, std::string>> transitions;
    // The mode of the compressor over time. A compressor starts in U-mode and changes when
    // feedback that asks for another mode reaches it (RFC 3095 5.6).
    struct ModePeriod
    {
        char mode = 'U';       // U, O or R
        double from = 0;       // s: the first packet of the bearer, or when that feedback arrived
        uint64_t packets = 0;  // compressed in this mode
        std::string startedBy; // the feedback that started it; "" for the mode the bearer began in
    };
    std::vector<ModePeriod> modes;
    uint64_t feedbackRx = 0;     // feedback control PDUs that reached the compressor
    double feedbackDelaySum = 0; // s, from the decompressor to the compressor
    uint64_t feedbackRxKind[3] = {0, 0, 0}; // their feedback: ACK, NACK, STATIC-NACK
    // Decompressor (receiver side). Packets that reached it = decompressed + decompressFailed.
    uint64_t decompressed = 0;
    uint64_t decompressFailed = 0; // packet dropped
    std::map<std::string, uint64_t> failures; // the same, by reason
    uint64_t rxBytesIn = 0;                   // ROHC packets that were decompressed
    uint64_t rxBytesOut = 0;                  // the same packets after decompression
    uint64_t rxBytesFailed = 0;               // ROHC packets that were dropped
    uint64_t feedbackTx = 0;                  // feedback control PDUs sent
    uint64_t feedbackTxBytes = 0;             // their ROHC feedback bytes
    uint64_t feedbackTxKind[3] = {0, 0, 0};   // their feedback: ACK, NACK, STATIC-NACK
    struct FeedbackSent
    {
        int64_t sentNs = 0;  // when the decompressor sent it
        uint8_t kind = 0;    // 0 ACK, 1 NACK, 2 STATIC-NACK
        double arrived = -1; // s, when it reached the compressor; -1: it did not
    };
    std::vector<FeedbackSent> feedbackLog; // every feedback the decompressor sent
    // A context loss as the decompressor lives it: packets in a row that it cannot restore
    struct ContextLoss
    {
        double from = 0;      // s, the first packet that could not be restored
        double to = 0;        // s, the packet that was restored again, or the last that was not
        uint64_t dropped = 0; // packets
        int64_t missing = -1; // packets missing before the first of them, -1: not known
        std::map<std::string, uint64_t> reasons; // the dropped packets by reason
        std::string repairedBy;                  // type of the packet that was restored again
        bool open = true;                        // not repaired (yet)
    };
    std::vector<ContextLoss> contextLosses;
};

using RohcKey = std::tuple<uint16_t, uint8_t, bool, uint16_t>; // RNTI, LCID, downlink, cell

std::map<RohcKey, RohcReport> g_rohcReports;
bool g_rohcPrintScheduled = false;

// A packet built from raw bytes (Create<Packet>(buf, len)) has no tags. This copies every byte
// tag and packet tag from the original packet onto the new one, so tags such as FlowMonitor's
// Ipv4FlowProbeTag survive compression and decompression. Not copied are the tags PDCP and RLC
// put on a PDU for their peer entity: they are about the bytes of that PDU, not of the packet.
void
CopyTags(Ptr<const Packet> from, Ptr<Packet> to)
{
    ByteTagIterator bit = from->GetByteTagIterator();
    while (bit.HasNext())
    {
        ByteTagIterator::Item item = bit.Next();
        TypeId tid = item.GetTypeId();
        if (!tid.HasConstructor() || tid == NrPdcpCheckTag::GetTypeId() ||
            tid == NrRohcSizeTableTag::GetTypeId() || tid == NrRohcCheckTag::GetTypeId() ||
            tid == NrPdcpTag::GetTypeId() || tid == NrRlcTag::GetTypeId())
        {
            continue;
        }
        Tag* tag = dynamic_cast<Tag*>(tid.GetConstructor()());
        if (!tag)
        {
            continue;
        }
        item.GetTag(*tag);
        to->AddByteTag(*tag);
        delete tag;
    }
    PacketTagIterator pit = from->GetPacketTagIterator();
    while (pit.HasNext())
    {
        PacketTagIterator::Item item = pit.Next();
        TypeId tid = item.GetTypeId();
        if (!tid.HasConstructor())
        {
            continue;
        }
        Tag* tag = dynamic_cast<Tag*>(tid.GetConstructor()());
        if (!tag)
        {
            continue;
        }
        item.GetTag(*tag);
        to->AddPacketTag(*tag);
        delete tag;
    }
}

// "1.0.0.2:49153 -> 7.0.0.2:4000 UDP", read from an uncompressed IPv4 packet
std::string
FlowOf(const uint8_t* d, uint32_t len)
{
    if (len < 20 || (d[0] >> 4) != 4)
    {
        return "non-IPv4 packets";
    }
    const uint32_t ihl = (d[0] & 0x0f) * 4;
    const uint8_t proto = d[9];
    const bool ports = (proto == 17 || proto == 6) && len >= ihl + 4;
    std::ostringstream s;
    s << +d[12] << '.' << +d[13] << '.' << +d[14] << '.' << +d[15];
    if (ports)
    {
        s << ':' << ((d[ihl] << 8) | d[ihl + 1]);
    }
    s << " -> " << +d[16] << '.' << +d[17] << '.' << +d[18] << '.' << +d[19];
    if (ports)
    {
        s << ':' << ((d[ihl + 2] << 8) | d[ihl + 3]);
    }
    s << (proto == 17 ? " UDP" : proto == 6 ? " TCP" : " IP protocol " + std::to_string(proto));
    return s.str();
}

// Which UE a bearer belongs to, read from one of its packets: the destination of a downlink
// packet, the source of an uplink one. The RNTI does not say it: a UE gets a new RNTI, and
// with it new PDCP entities and a new line in the report, every time it attaches again after a
// radio link failure.
void
NoteUe(RohcReport& r, const uint8_t* d, uint32_t len, bool downlink)
{
    if (!r.ue.empty() || len < 20 || (d[0] >> 4) != 4)
    {
        return;
    }
    const uint8_t* a = d + (downlink ? 16 : 12);
    r.ue = std::to_string(a[0]) + "." + std::to_string(a[1]) + "." + std::to_string(a[2]) + "." +
           std::to_string(a[3]);
    r.ueNumber = (uint32_t(a[0]) << 24) | (uint32_t(a[1]) << 16) | (uint32_t(a[2]) << 8) | a[3];
}

// FNV-1a hash of a packet, for NrRohcCheckTag. The checksum of an IPv4 header is left out: the
// library computes it anew, so it differs when ns-3 left it empty (checksums not enabled).
uint32_t
RohcPacketHash(const uint8_t* d, uint32_t len)
{
    const bool ipv4 = len >= 20 && (d[0] >> 4) == 4;
    uint32_t hash = 2166136261u;
    for (uint32_t i = 0; i < len; ++i)
    {
        hash = (hash ^ ((ipv4 && (i == 10 || i == 11)) ? 0 : d[i])) * 16777619u;
    }
    return hash;
}

// The compressor is in mode `mode` now. `startedBy` says which feedback made it so; "" at a
// packet, which only tells the mode the compressor is in.
void
NoteRohcMode(RohcReport& r, char mode, double now, const std::string& startedBy)
{
    if (r.modes.empty() || r.modes.back().mode != mode)
    {
        RohcReport::ModePeriod period;
        period.mode = mode;
        period.from = now;
        period.startedBy = startedBy;
        r.modes.push_back(period);
    }
}

// The decompressor could not restore a packet: a context loss starts, or goes on
void
NoteRohcFailure(RohcReport& r, double now, const std::string& reason, int64_t missing)
{
    if (r.contextLosses.empty() || !r.contextLosses.back().open)
    {
        RohcReport::ContextLoss loss;
        loss.from = now;
        loss.missing = missing;
        r.contextLosses.push_back(loss);
    }
    RohcReport::ContextLoss& loss = r.contextLosses.back();
    loss.to = now;
    loss.dropped++;
    loss.reasons[reason]++;
}

// The decompressor restored a packet of type `packetType`: a context loss, if there was one,
// is over
void
NoteRohcRestored(RohcReport& r, double now, const std::string& packetType)
{
    if (!r.contextLosses.empty() && r.contextLosses.back().open)
    {
        RohcReport::ContextLoss& loss = r.contextLosses.back();
        loss.to = now;
        loss.repairedBy = packetType;
        loss.open = false;
    }
}

const char* const g_rohcFeedbackName[3] = {"ACK", "NACK", "STATIC-NACK"};

// One feedback element of a ROHC packet (RFC 3095 5.2.2)
struct RohcFeedbackElement
{
    uint8_t kind = 0;   // 0 ACK, 1 NACK, 2 STATIC-NACK, 3 reserved
    uint8_t mode = 0;   // the mode it asks for: 0 none, 1 U, 2 O, 3 R
    bool crc = false;   // it has the CRC option, which a mode request needs (RFC 3095 5.6.1)
};

// The feedback elements in a ROHC packet with only feedback, RFC 3095 5.2.2 and 5.7.6.1: each
// starts with 11110 and its size (0: the size is the next octet); then an Add-CID octet for a
// small CID other than 0, or the one or two octets of a large CID; then FEEDBACK-1 (one octet:
// an ACK) or FEEDBACK-2 (Acktype, Mode, 12 bits of sequence number, options). For the ROHCv1
// profiles of RFC 3095, the ones NrPdcp allows by default.
std::vector<RohcFeedbackElement>
ParseRohcFeedback(const uint8_t* d, uint32_t len, bool largeCid)
{
    std::vector<RohcFeedbackElement> elements;
    uint32_t i = 0;
    while (i < len && (d[i] & 0xf8) == 0xf0)
    {
        uint32_t size = d[i++] & 0x07;
        if (size == 0 && i < len)
        {
            size = d[i++];
        }
        if (size == 0 || i + size > len)
        {
            break;
        }
        const uint8_t* f = d + i;
        i += size;
        const uint32_t cid = largeCid ? ((f[0] & 0x80) == 0 ? 1 : 2)
                                      : ((f[0] & 0xf0) == 0xe0 && size > 1 ? 1 : 0);
        if (size <= cid)
        {
            continue;
        }
        f += cid;
        size -= cid;
        RohcFeedbackElement e;
        if (size > 1)
        {
            e.kind = f[0] >> 6;
            e.mode = (f[0] >> 4) & 0x03;
            // The options: type (4 bits) and length (4 bits), then that many octets; type 1 is
            // the CRC (RFC 3095 5.7.6.2 and 5.7.6.3)
            for (uint32_t k = 2; k < size; k += 1 + (f[k] & 0x0f))
            {
                e.crc = e.crc || (f[k] >> 4) == 1;
            }
        }
        elements.push_back(e);
    }
    return elements;
}

// RFC 3095 5.9.1: the 8-bit CRC, C(x) = 1 + x + x^2 + x^8, all ones at the start
uint8_t
RohcCrc8(const uint8_t* d, uint32_t len)
{
    uint8_t crc = 0xff;
    for (uint32_t i = 0; i < len; ++i)
    {
        crc ^= d[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1) ? (crc >> 1) ^ 0xe0 : crc >> 1;
        }
    }
    return crc;
}

// The feedback of the size-table model, 5 bytes (RFC 3095 5.7.6.1): 11110 and the size, 4; then
// FEEDBACK-2 with Acktype (0 ACK, 1 NACK, 2 STATIC-NACK), Mode (2: O-mode) and 12 bits of the
// number of the packet; then the CRC option, 0x11 and the CRC over these four octets with the
// CRC octet as zero (5.7.6.3). Only feedback with the CRC option may change the mode of the
// compressor (5.6.1); the ROHC library takes exactly these bytes as a request for O-mode.
std::array<uint8_t, 5>
RohcTableFeedback(uint8_t ackType, uint32_t number)
{
    const uint8_t first = (ackType << 6) | (2 << 4) | ((number >> 8) & 0x0f);
    std::array<uint8_t, 5> f = {0xf4, first, static_cast<uint8_t>(number & 0xff), 0x11, 0x00};
    f[4] = RohcCrc8(f.data() + 1, 4);
    return f;
}

// The bearers in the order of the tables: UE by UE (by address; bearers whose UE no packet has
// told come last), and the RNTIs of a UE in the order it got them
std::vector<std::pair<RohcKey, const RohcReport*>>
OrderedRohcReports()
{
    std::vector<std::pair<RohcKey, const RohcReport*>> rows;
    for (const auto& [key, r] : g_rohcReports)
    {
        rows.emplace_back(key, &r);
    }
    auto place = [](const std::pair<RohcKey, const RohcReport*>& row) {
        return std::make_tuple(row.second->ue.empty(),
                               row.second->ueNumber,
                               std::get<1>(row.first),  // LCID
                               !std::get<2>(row.first), // downlink before uplink
                               std::get<0>(row.first)); // RNTI
    };
    std::sort(rows.begin(), rows.end(), [&place](const auto& a, const auto& b) {
        return place(a) < place(b);
    });
    return rows;
}

// Whether two bearers are the same bearer of the same UE under two RNTIs
bool
SameUeBearer(const std::pair<RohcKey, const RohcReport*>& a,
             const std::pair<RohcKey, const RohcReport*>& b)
{
    return !a.second->ue.empty() && a.second->ue == b.second->ue &&
           std::get<1>(a.first) == std::get<1>(b.first) &&
           std::get<2>(a.first) == std::get<2>(b.first);
}

// What the tables show of a bearer, added to a sum over the RNTIs of its UE. The mode of the
// sum is the modes of its parts: "U", "O" or "U+O".
void
AddToUeSum(RohcReport& sum, const RohcReport& r)
{
    for (int st = 0; st < 4; ++st)
    {
        sum.packets[st] += r.packets[st];
        sum.hdrIn[st] += r.hdrIn[st];
        sum.hdrOut[st] += r.hdrOut[st];
    }
    sum.bytesIn += r.bytesIn;
    sum.bytesOut += r.bytesOut;
    sum.decompressed += r.decompressed;
    sum.decompressFailed += r.decompressFailed;
    sum.rxBytesIn += r.rxBytesIn;
    sum.rxBytesOut += r.rxBytesOut;
    sum.rxBytesFailed += r.rxBytesFailed;
    sum.feedbackTx += r.feedbackTx;
    sum.feedbackRx += r.feedbackRx;
    for (int kind = 0; kind < 3; ++kind)
    {
        sum.feedbackTxKind[kind] += r.feedbackTxKind[kind];
        sum.feedbackRxKind[kind] += r.feedbackRxKind[kind];
    }
    sum.contextLosses.insert(sum.contextLosses.end(),
                             r.contextLosses.begin(),
                             r.contextLosses.end());
    for (const auto& [type, count] : r.packetTypes)
    {
        sum.packetTypes[type].packets += count.packets;
        sum.packetTypes[type].hdrOut += count.hdrOut;
    }
    if (!r.mode.empty() && sum.mode.find(r.mode[0]) == std::string::npos)
    {
        // The modes of the parts, always in the same order: "U", "O", "U+O" (R after them)
        const std::string had = sum.mode + r.mode[0];
        sum.mode.clear();
        for (const char letter : std::string("UOR"))
        {
            if (had.find(letter) != std::string::npos)
            {
                sum.mode += (sum.mode.empty() ? "" : "+") + std::string(1, letter);
            }
        }
    }
}

// A title in a frame, `note` at the right end of its line: as the tables of the example that
// come before these (scratch/ntnlab_nr.cc), 130 columns wide
void
PrintRohcFrame(std::ostream& o, const std::string& title, const std::string& note = "")
{
    const std::size_t width = 130;
    const std::string bar = "+" + std::string(width - 2, '-') + "+";
    std::string line = title;
    if (!note.empty())
    {
        const std::size_t taken = title.size() + note.size();
        line += std::string(taken + 2 <= width - 4 ? width - 4 - taken : 2, ' ') + note;
    }
    o << "\n"
      << bar << "\n| " << std::left << std::setw(width - 4) << line << std::right << " |\n"
      << bar << "\n";
}

// The first columns of a line of the tables: UE, RNTI ("all": the RNTIs of the UE added up),
// LCID, direction
void
PrintRohcBearerColumns(std::ostream& o,
                       const std::string& ue,
                       const std::string& rnti,
                       const RohcKey& key)
{
    o << "    " << std::setw(13) << (ue.empty() ? "-" : ue) << std::setw(6) << rnti << std::setw(5)
      << +std::get<1>(key) << "  " << (std::get<2>(key) ? "DL" : "UL");
}

// What the compressor of a bearer sent, added up over its states
struct RohcSent
{
    uint64_t packets = 0;
    uint64_t hdrIn = 0;  // header bytes before compression
    uint64_t hdrOut = 0; // ROHC header bytes
};

RohcSent
SentOf(const RohcReport& r)
{
    RohcSent sent;
    for (int st = 0; st < 4; ++st)
    {
        sent.packets += r.packets[st];
        sent.hdrIn += r.hdrIn[st];
        sent.hdrOut += r.hdrOut[st];
    }
    return sent;
}

// "1/3/2": how many ACK, NACK and STATIC-NACK
std::string
RohcFeedbackCounts(const uint64_t kinds[3])
{
    return std::to_string(kinds[0]) + "/" + std::to_string(kinds[1]) + "/" +
           std::to_string(kinds[2]);
}

// "1 ACK + 2 NACK": the same in words, "" for none
std::string
RohcFeedbackWords(const uint64_t kinds[3])
{
    std::string words;
    for (int kind = 0; kind < 3; ++kind)
    {
        if (kinds[kind] > 0)
        {
            words += (words.empty() ? "" : " + ") + std::to_string(kinds[kind]) + " " +
                     g_rohcFeedbackName[kind];
        }
    }
    return words;
}

// Calls `line` for every bearer in the order of the tables and, after the last RNTI of a UE
// that had more than one, for their sum
void
ForEachRohcLine(const std::function<void(const std::string& rnti,
                                         const RohcKey& key,
                                         const std::string& mode,
                                         const RohcReport& r)>& line)
{
    const auto rows = OrderedRohcReports();
    for (std::size_t i = 0; i < rows.size();)
    {
        RohcReport sum;
        std::size_t j = i;
        for (; j < rows.size() && (j == i || SameUeBearer(rows[i], rows[j])); ++j)
        {
            const RohcReport& r = *rows[j].second;
            line(std::to_string(std::get<0>(rows[j].first)),
                 rows[j].first,
                 r.mode.substr(0, 1),
                 r);
            AddToUeSum(sum, r);
        }
        if (j - i > 1)
        {
            sum.ue = rows[i].second->ue;
            line("all", rows[i].first, sum.mode, sum);
        }
        i = j;
    }
}

// The library's name of a packet type without what the profile already says:
// "ROHCv1/RTP/UOR-2-TS-EXT-3" -> "UOR-2-TS-EXT-3", "ROHCv1/UO-0" -> "UO-0", "IR" -> "IR"
std::string
ShortPacketType(const std::string& name)
{
    std::string s = name;
    for (const std::string prefix : {"ROHCv1/", "ROHCv2/", "RTP/", "non-RTP/"})
    {
        if (s.compare(0, prefix.size(), prefix) == 0)
        {
            s.erase(0, prefix.size());
        }
    }
    return s;
}

// Order of the packet types in the tables: from the largest header to the smallest, i.e. IR,
// IR-DYN, UOR-2 with extension 3 ... 0, UOR-2 without, UO-1, UO-0; anything else last.
int
PacketTypeRank(const std::string& shortName)
{
    auto has = [&shortName](const char* part) { return shortName.find(part) != std::string::npos; };
    if (shortName == "IR")
    {
        return 0;
    }
    if (shortName == "IR-DYN")
    {
        return 1;
    }
    if (has("UOR-2"))
    {
        return has("EXT-3") ? 2 : has("EXT-2") ? 3 : has("EXT-1") ? 4 : has("EXT-0") ? 5 : 6;
    }
    if (has("UO-1"))
    {
        return 7;
    }
    if (has("UO-0"))
    {
        return 8;
    }
    return 9;
}

// One table for all bearers: which share of its packets each bearer sent as which ROHC packet
// type, next to the mode of its compressor; then the same over all bearers and how many header
// bytes a packet of each type had on average. The lines are in the order of the first table:
// UE by UE, and a line "all" for a UE that had several RNTIs.
void
PrintRohcPacketTypes(std::ostream& o)
{
    using Column = std::pair<int, std::string>; // rank, short name
    std::map<Column, RohcReport::TypeCount> total;
    uint64_t totalPackets = 0;
    for (const auto& [key, r] : g_rohcReports)
    {
        for (const auto& [type, count] : r.packetTypes)
        {
            const std::string shortName = ShortPacketType(type);
            RohcReport::TypeCount& t = total[{PacketTypeRank(shortName), shortName}];
            t.packets += count.packets;
            t.hdrOut += count.hdrOut;
            totalPackets += count.packets;
        }
    }
    if (totalPackets == 0)
    {
        return;
    }
    auto width = [](const Column& c) { return std::max<int>(c.second.size(), 6) + 2; };
    PrintRohcFrame(o, "ROHC PACKET TYPES", "in % of the packets of the bearer");
    o << "       UE address  RNTI LCID dir mode  packets";
    for (const auto& [column, count] : total)
    {
        o << std::setw(width(column)) << column.second;
    }
    o << "\n";
    ForEachRohcLine([&](const std::string& rnti,
                        const RohcKey& key,
                        const std::string& mode,
                        const RohcReport& r) {
        std::map<std::string, uint64_t> ofBearer;
        uint64_t packets = 0;
        for (const auto& [type, count] : r.packetTypes)
        {
            ofBearer[ShortPacketType(type)] += count.packets;
            packets += count.packets;
        }
        if (packets == 0)
        {
            return;
        }
        PrintRohcBearerColumns(o, r.ue, rnti, key);
        o << std::setw(5) << (mode.empty() ? "-" : mode) << std::setw(9) << packets;
        for (const auto& [column, count] : total)
        {
            o << std::setw(width(column)) << std::setprecision(1)
              << 100.0 * ofBearer[column.second] / packets;
        }
        o << "\n";
    });
    o << "    " << std::left << std::setw(33) << "all bearers" << std::right << std::setw(9)
      << totalPackets;
    for (const auto& [column, count] : total)
    {
        o << std::setw(width(column)) << std::setprecision(1) << 100.0 * count.packets / totalPackets;
    }
    o << "\n    " << std::left << std::setw(42) << "mean ROHC header [B]" << std::right;
    for (const auto& [column, count] : total)
    {
        o << std::setw(width(column)) << std::setprecision(1) << double(count.hdrOut) / count.packets;
    }
    o << "\n";
}

// The summary, first table: what each compressor made of its packets. One line per bearer, UE
// by UE. A UE that attached again after a radio link failure has one line per RNTI it had,
// and a line "all" with their sum.
void
PrintRohcCompressorTable(std::ostream& o)
{
    PrintRohcFrame(o, "ROHC COMPRESSOR (SENDER)", "one line per connection (RNTI) of a UE");
    o << "       UE address  RNTI LCID dir mode  packets  in IR/FO/SO       header B/pkt  saved  "
         "feedback rcvd\n";
    ForEachRohcLine([&](const std::string& rnti,
                        const RohcKey& key,
                        const std::string& mode,
                        const RohcReport& r) {
        const RohcSent sent = SentOf(r);
        std::ostringstream states;
        states << r.packets[1] << "/" << r.packets[2] << "/" << r.packets[3];
        PrintRohcBearerColumns(o, r.ue, rnti, key);
        o << std::setw(5) << (mode.empty() ? "-" : mode) << std::setw(9) << sent.packets << "  "
          << std::left << std::setw(16) << states.str() << std::right;
        if (sent.packets > 0)
        {
            o << std::setprecision(1) << std::setw(6) << double(sent.hdrIn) / sent.packets << " ->"
              << std::setw(5) << double(sent.hdrOut) / sent.packets << std::setw(6)
              << 100.0 * (1.0 - double(r.bytesOut) / r.bytesIn) << "%";
        }
        else
        {
            o << std::setw(21) << "-";
        }
        o << std::setw(15) << RohcFeedbackCounts(r.feedbackRxKind) << "\n";
    });
    o << "mode: the one it ended in. in IR/FO/SO: packets it sent in each state. saved: of the "
         "bytes of the whole packets.\n"
      << "feedback rcvd: the ACK/NACK/STATIC-NACK that reached it.\n";
}

// The summary, second table: what reached each decompressor and what it could do with it
void
PrintRohcDecompressorTable(std::ostream& o)
{
    PrintRohcFrame(o, "ROHC DECOMPRESSOR (RECEIVER)", "one line per connection (RNTI) of a UE");
    o << "       UE address  RNTI LCID dir     sent  arrived  restored  failed  restored[%]  "
         "context lost  for[s]  net saved  feedback sent\n";
    ForEachRohcLine([&](const std::string& rnti,
                        const RohcKey& key,
                        const std::string& /* mode */,
                        const RohcReport& r) {
        const uint64_t arrived = r.decompressed + r.decompressFailed;
        double lostFor = 0;
        for (const auto& loss : r.contextLosses)
        {
            lostFor += loss.to - loss.from;
        }
        PrintRohcBearerColumns(o, r.ue, rnti, key);
        o << std::setw(9) << SentOf(r).packets << std::setw(9) << arrived << std::setw(10)
          << r.decompressed << std::setw(8) << r.decompressFailed << std::setw(13);
        if (arrived > 0)
        {
            o << std::setprecision(1) << 100.0 * r.decompressed / arrived;
        }
        else
        {
            o << "-";
        }
        o << std::setw(14) << r.contextLosses.size() << std::setprecision(2) << std::setw(8)
          << lostFor;
        if (r.rxBytesOut > 0)
        {
            o << std::setprecision(1) << std::setw(10)
              << 100.0 * (1.0 - double(r.rxBytesIn + r.rxBytesFailed) / r.rxBytesOut) << "%";
        }
        else
        {
            o << std::setw(11) << "-";
        }
        o << std::setw(15) << RohcFeedbackCounts(r.feedbackTxKind) << "\n";
    });
    o << "sent: by the compressor. arrived: of these, reached the decompressor. restored: header "
         "put back, also in % of arrived.\n"
      << "failed: dropped, the context could not restore the header. context lost: runs of failed "
         "packets; for[s]: their time in all.\n"
      << "net saved: 1 - bytes arrived / bytes of the packets restored from them. feedback sent: "
         "ACK/NACK/STATIC-NACK.\n";
}

// Why the compressor of a bearer never left U-mode: what the decompressor sent and what of it
// arrived
std::string
WhyNoRohcFeedbackMode(const RohcReport& r)
{
    uint64_t arrived[3] = {0, 0, 0};
    for (const auto& feedback : r.feedbackLog)
    {
        arrived[feedback.kind] += feedback.arrived >= 0;
    }
    const std::string sent = RohcFeedbackWords(r.feedbackTxKind);
    if (sent.empty())
    {
        return "never: no feedback was sent";
    }
    const std::string got = RohcFeedbackWords(arrived);
    const std::string words =
        "never: " + sent + " sent, " + (got.empty() ? "none" : got) + " arrived";
    if (words.size() <= 48)
    {
        return words;
    }
    // (too long for the table: the totals; the decompressor table has the kinds sent)
    const uint64_t sentAll = r.feedbackTxKind[0] + r.feedbackTxKind[1] + r.feedbackTxKind[2];
    const uint64_t gotAll = arrived[0] + arrived[1] + arrived[2];
    return "never: " + std::to_string(sentAll) + " feedback sent, " +
           (gotAll > 0 ? std::to_string(gotAll) : std::string("none")) + " arrived";
}

// The summary, third table: the mode of each compressor over time, one line per bearer and
// RNTI, i.e. per connection. No line "all": the lines of a UE are its history.
void
PrintRohcModeTable(std::ostream& o)
{
    PrintRohcFrame(o, "ROHC MODE OF THE COMPRESSOR", "one line per connection (RNTI) of a UE");
    o << "       UE address  RNTI LCID dir  U from[s]  packets  O from[s]  packets  last "
         "packet[s]  O-mode started by\n";
    for (const auto& [key, report] : OrderedRohcReports())
    {
        const RohcReport& r = *report;
        if (r.modes.empty())
        {
            continue;
        }
        PrintRohcBearerColumns(o, r.ue, std::to_string(std::get<0>(key)), key);
        // What the two models do: U, or U and then O. Anything else is listed as it was.
        const bool startsInU = r.modes[0].mode == 'U';
        const bool thenO = r.modes.size() == 2 && r.modes[1].mode == 'O';
        o << std::setprecision(3);
        if (startsInU && (r.modes.size() == 1 || thenO))
        {
            o << std::setw(11) << r.modes[0].from << std::setw(9) << r.modes[0].packets;
            if (thenO)
            {
                o << std::setw(11) << r.modes[1].from << std::setw(9) << r.modes[1].packets;
            }
            else
            {
                o << std::setw(11) << "-" << std::setw(9) << "-";
            }
            o << std::setw(16) << r.lastTx << "  "
              << (thenO ? r.modes[1].startedBy : WhyNoRohcFeedbackMode(r));
        }
        else
        {
            o << std::setw(11) << "-" << std::setw(9) << "-" << std::setw(11) << "-" << std::setw(9)
              << "-" << std::setw(16) << r.lastTx << "  ";
            const char* sep = "";
            for (const auto& period : r.modes)
            {
                o << sep << period.mode << " from " << period.from << " s (" << period.packets
                  << " packets)";
                sep = ", ";
            }
        }
        o << "\n";
    }
    o << "U-mode from its first packet (U from), O-mode once feedback that asks for it has "
         "arrived (O from). It does not go back to\n"
      << "U-mode; the next connection of the UE starts in U-mode again.\n";
}

void
PrintRohcReports()
{
    static const char* stateName[4] = {"??", "IR", "FO", "SO"};
    StringValue modeValue;
    g_nrRohcReport.GetValue(modeValue);
    const std::string reportMode = modeValue.Get();
    NS_ABORT_MSG_UNLESS(reportMode == "none" || reportMode == "summary" || reportMode == "full",
                        "NrRohcReport must be none, summary or full");
    std::ostringstream o;
    o << std::fixed;
    if (reportMode == "summary" && !g_rohcReports.empty())
    {
        PrintRohcCompressorTable(o);
        PrintRohcDecompressorTable(o);
        PrintRohcModeTable(o);
    }
    for (const auto& [key, report] : OrderedRohcReports())
    {
        if (reportMode != "full")
        {
            break;
        }
        const RohcReport& r = *report;
        const uint16_t rnti = std::get<0>(key);
        const uint8_t lcid = std::get<1>(key);
        const bool downlink = std::get<2>(key);
        const RohcSent sent = SentOf(r);
        const uint64_t packets = sent.packets;
        const uint64_t hdrIn = sent.hdrIn;
        const uint64_t hdrOut = sent.hdrOut;

        o << "\n==================== ROHC report ====================\n";
        o << "Bearer        : RNTI " << rnti << ", LCID " << +lcid << ", "
          << (downlink ? "downlink (compressor in gNB, decompressor in UE)"
                       : "uplink (compressor in UE, decompressor in gNB)")
          << "\n";
        o << "UE            : " << (r.ue.empty() ? "not known, no packet was sent" : r.ue) << "\n";
        for (const auto& flow : r.flows)
        {
            o << "Flow          : " << flow << "\n";
        }
        if (!r.profile.empty())
        {
            o << "Profile       : " << r.profile << (r.mode == "-" ? "" : ", " + r.mode) << "\n";
        }
        if (packets > 0)
        {
            o << "Active        : " << std::setprecision(3) << r.firstTx << " s to " << r.lastTx
              << " s\n";
        }
        if (!r.modes.empty())
        {
            // The mode of the compressor over time: a new period when feedback changed it
            o << "Mode          : ";
            const char* sep = "";
            for (const auto& period : r.modes)
            {
                o << sep << period.mode << "-mode from " << std::setprecision(3) << period.from
                  << " s (" << period.packets << " packets";
                if (!period.startedBy.empty())
                {
                    o << ", started by the " << period.startedBy;
                }
                o << ")";
                sep = ", ";
            }
            o << "\n";
        }

        o << "Compressor    : " << packets << " packets compressed";
        if (r.compressFailed > 0)
        {
            o << ", " << r.compressFailed << " dropped (compression failed)";
        }
        o << "\n";
        if (packets > 0)
        {
            o << "  state   packets   header bytes in -> out   per packet\n";
            for (int st = 0; st < 4; ++st)
            {
                if (st == 0 && r.packets[0] == 0)
                {
                    continue;
                }
                o << "  " << std::left << std::setw(5) << stateName[st] << std::right
                  << std::setw(10) << r.packets[st] << std::setw(12) << r.hdrIn[st] << " -> "
                  << std::setw(6) << r.hdrOut[st];
                if (r.packets[st] > 0)
                {
                    o << std::setprecision(1) << std::setw(14)
                      << double(r.hdrIn[st]) / r.packets[st] << " -> "
                      << double(r.hdrOut[st]) / r.packets[st] << " B";
                }
                o << "\n";
            }
            o << "  " << std::left << std::setw(5) << "all" << std::right << std::setw(10)
              << packets << std::setw(12) << hdrIn << " -> " << std::setw(6) << hdrOut
              << std::setprecision(1) << std::setw(14) << double(hdrIn) / packets << " -> "
              << double(hdrOut) / packets << " B\n";
            o << "  headers       : " << std::setprecision(1)
              << 100.0 * (1.0 - double(hdrOut) / hdrIn) << " % smaller\n";
            o << "  whole packets : " << r.bytesIn << " -> " << r.bytesOut << " B, "
              << 100.0 * (1.0 - double(r.bytesOut) / r.bytesIn) << " % smaller\n";
            o << "  ROHC packets  : ";
            const char* sep = "";
            for (const auto& [type, count] : r.packetTypes)
            {
                o << sep << type << " x" << count.packets << " (" << std::setprecision(1)
                  << 100.0 * count.packets / packets << " %, " << double(count.hdrOut) / count.packets
                  << " B)";
                sep = ", ";
            }
            o << "\n  state changes : ";
            sep = "";
            for (const auto& [t, name] : r.transitions)
            {
                o << sep << name << " at " << std::setprecision(3) << t << " s";
                sep = ", ";
            }
            o << "\n";
        }

        o << "Decompressor  : " << r.decompressed + r.decompressFailed << " packets arrived, "
          << r.decompressed << " decompressed, " << r.decompressFailed << " failed (dropped)";
        if (r.decompressed > 0)
        {
            o << "; " << r.rxBytesIn << " -> " << r.rxBytesOut << " B";
        }
        o << "\n";
        if (!r.failures.empty())
        {
            o << "  failures      : ";
            const char* sep = "";
            for (const auto& [reason, n] : r.failures)
            {
                o << sep << reason << " x" << n;
                sep = ", ";
            }
            o << "\n";
        }
        if (r.rxBytesOut > 0)
        {
            // What it took to get these packets through: the bytes of every packet that arrived
            o << "  net saving    : " << r.rxBytesIn + r.rxBytesFailed << " B arrived for "
              << r.rxBytesOut << " B of restored packets, " << std::setprecision(1)
              << 100.0 * (1.0 - double(r.rxBytesIn + r.rxBytesFailed) / r.rxBytesOut)
              << " % smaller\n";
        }
        if (!r.contextLosses.empty())
        {
            double lostFor = 0;
            for (const auto& loss : r.contextLosses)
            {
                lostFor += loss.to - loss.from;
            }
            o << "  context lost  : " << r.contextLosses.size()
              << (r.contextLosses.size() == 1 ? " time, for " : " times, for ")
              << std::setprecision(3) << lostFor << " s in all\n";
            for (const auto& loss : r.contextLosses)
            {
                o << "    " << loss.from << " s to " << loss.to << " s: " << loss.dropped
                  << " packets dropped (";
                const char* sep = "";
                for (const auto& [reason, n] : loss.reasons)
                {
                    o << sep << reason << " x" << n;
                    sep = ", ";
                }
                o << ")";
                if (loss.missing >= 0)
                {
                    o << ", " << loss.missing << " were missing before";
                }
                o << (loss.open ? ", not repaired" : ", then restored: " + loss.repairedBy) << "\n";
            }
        }
        o << "Feedback      : " << r.feedbackTx << " control PDUs sent by the decompressor ("
          << r.feedbackTxBytes << " B of ROHC feedback), " << r.feedbackRx
          << " reached the compressor";
        if (r.feedbackRx > 0)
        {
            o << " after " << std::setprecision(1) << 1000.0 * r.feedbackDelaySum / r.feedbackRx
              << " ms on average";
        }
        o << "\n";
        if (!r.feedbackLog.empty())
        {
            o << "  sent          : ";
            const char* sep = "";
            for (const auto& feedback : r.feedbackLog)
            {
                o << sep << g_rohcFeedbackName[feedback.kind] << " at " << std::setprecision(3)
                  << feedback.sentNs / 1e9 << " s (";
                if (feedback.arrived >= 0)
                {
                    o << "arrived " << feedback.arrived << " s)";
                }
                else
                {
                    o << "did not arrive)";
                }
                sep = ", ";
            }
            o << "\n";
        }
        o << "=====================================================\n";
    }
    if (reportMode != "none")
    {
        PrintRohcPacketTypes(o);
    }
    std::cout << o.str() << std::flush;
    g_rohcReports.clear();
    g_rohcPrintScheduled = false;
}

RohcReport&
GetRohcReport(uint16_t cellId, uint16_t rnti, uint8_t lcid, bool downlink)
{
    if (!g_rohcPrintScheduled)
    {
        Simulator::ScheduleDestroy(&PrintRohcReports);
        g_rohcPrintScheduled = true;
    }
    return g_rohcReports[RohcKey(rnti, lcid, downlink, cellId)];
}

// The library asks for random numbers to pick the initial sequence number of a context
int
RohcRandom(const struct rohc_comp* const /* comp */, void* const /* userContext */)
{
    return 12345;
}

// The library asks whether a UDP packet carries RTP; only then can it use the RTP profile.
// Yes for RTP version 2 with a payload type outside RTCP's range (RFC 5761: a second octet of
// 192 to 223 is RTCP). The library itself refuses RTP headers with CSRC entries.
bool
RohcIsRtp(const unsigned char* const /* ip */,
          const unsigned char* const /* udp */,
          const unsigned char* const payload,
          const unsigned int payloadSize,
          void* const /* userContext */)
{
    return payloadSize >= 12 && (payload[0] >> 6) == 2 && (payload[1] < 192 || payload[1] > 223);
}

// "0x0000,0x0001,0x0002" -> the profile identifiers
std::vector<rohc_profile_t>
RohcProfileList(const std::string& list)
{
    std::vector<rohc_profile_t> profiles;
    std::istringstream in(list);
    std::string item;
    while (std::getline(in, item, ','))
    {
        char* end = nullptr;
        const unsigned long id = std::strtoul(item.c_str(), &end, 0);
        NS_ABORT_MSG_IF(end == item.c_str() || *end != '\0',
                        "NrPdcp::RohcProfiles: \"" << item << "\" is not a profile identifier");
        profiles.push_back(static_cast<rohc_profile_t>(id));
    }
    NS_ABORT_MSG_IF(profiles.empty(), "NrPdcp::RohcProfiles is empty");
    return profiles;
}

// The simulation time, as the library's timestamp
struct rohc_ts
RohcNow()
{
    const int64_t ns = Simulator::Now().GetNanoSeconds();
    struct rohc_ts ts;
    ts.sec = static_cast<uint64_t>(ns / 1000000000);
    ts.nsec = static_cast<uint64_t>(ns % 1000000000);
    return ts;
}

} // namespace
#endif // NR_ROHC

bool
NrPdcp::IsRohcAvailable()
{
#ifdef NR_ROHC
    return true;
#else
    return false;
#endif
}

void
NrPdcp::SetEnableRohc(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    NS_ABORT_MSG_IF(enable && !IsRohcAvailable(),
                    "NrPdcp::EnableRohc needs the ROHC library (https://rohc-lib.org): install "
                    "it and configure ns-3 again, so that the nr module is built with it");
    m_enableRohc = enable;
}

bool
NrPdcp::GetEnableRohc() const
{
    return m_enableRohc;
}

void
NrPdcp::CreateRohcCompressor()
{
#ifdef NR_ROHC
    if (m_rohcCompressor)
    {
        return;
    }
    // TS 38.331 PDCP-Config: maxCID 1 to 16383. RFC 3095 5.1.1: small CIDs up to 15, large beyond.
    m_rohcCompressor =
        rohc_comp_new2(m_rohcMaxCid > ROHC_SMALL_CID_MAX ? ROHC_LARGE_CID : ROHC_SMALL_CID,
                       m_rohcMaxCid,
                       RohcRandom,
                       nullptr);
    NS_ABORT_MSG_UNLESS(m_rohcCompressor,
                        "NrPdcp: no ROHC compressor with RohcMaxCid " << m_rohcMaxCid);
    // The time-based refresh is on when RohcIrRefreshTime is set; FO then refreshes twice as
    // often unless RohcFoRefreshTime says otherwise
    const bool timed = m_rohcIrRefreshTime.IsStrictlyPositive();
    const uint64_t irMs = m_rohcIrRefreshTime.GetMilliSeconds();
    const uint64_t foMs = m_rohcFoRefreshTime.IsStrictlyPositive()
                              ? m_rohcFoRefreshTime.GetMilliSeconds()
                              : irMs / 2;
    const unsigned int features =
        ROHC_COMP_FEATURE_NO_IP_CHECKSUMS | (timed ? ROHC_COMP_FEATURE_TIME_BASED_REFRESHES : 0);
    // Each setup call returns false on failure; stop instead of running with a broken setup
    bool ok = rohc_comp_set_features(m_rohcCompressor,
                                     static_cast<rohc_comp_features_t>(features)) &&
              rohc_comp_set_optimistic_approach(m_rohcCompressor, m_rohcRepetitions) &&
              rohc_comp_set_periodic_refreshes(m_rohcCompressor, m_rohcIrRefresh, m_rohcFoRefresh) &&
              (!timed || rohc_comp_set_periodic_refreshes_time(m_rohcCompressor, irMs, foMs)) &&
              rohc_comp_set_rtp_detection_cb(m_rohcCompressor, RohcIsRtp, nullptr);
    NS_ABORT_MSG_UNLESS(ok,
                        "NrPdcp: ROHC compressor setup failed. RohcRepetitions "
                            << m_rohcRepetitions << " must be 1 to 255; RohcFoRefresh "
                            << m_rohcFoRefresh << " must be smaller than RohcIrRefresh "
                            << m_rohcIrRefresh << "; the FO refresh time " << foMs
                            << " ms must be above 0 and smaller than RohcIrRefreshTime " << irMs
                            << " ms");
    for (const rohc_profile_t profile : RohcProfileList(m_rohcProfiles))
    {
        NS_ABORT_MSG_UNLESS(rohc_comp_enable_profile(m_rohcCompressor, profile),
                            "NrPdcp::RohcProfiles: the ROHC library cannot compress profile 0x"
                                << std::hex << profile);
    }
    m_lastRohcState = ROHC_COMP_STATE_UNKNOWN;
#endif
}

void
NrPdcp::CreateRohcDecompressor()
{
#ifdef NR_ROHC
    if (m_rohcDecompressor)
    {
        return;
    }
    m_rohcDecompressor =
        rohc_decomp_new2(m_rohcMaxCid > ROHC_SMALL_CID_MAX ? ROHC_LARGE_CID : ROHC_SMALL_CID,
                         m_rohcMaxCid,
                         m_rohcMode == ROHC_OPTIMISTIC ? ROHC_O_MODE : ROHC_U_MODE);
    NS_ABORT_MSG_UNLESS(m_rohcDecompressor,
                        "NrPdcp: no ROHC decompressor with RohcMaxCid " << m_rohcMaxCid);
    // The library counts failures and feedback over the last 32 packets of a context: it gives
    // the context up (and, in O-mode, sends a NACK) at RohcNackFailures failures, and repeats a
    // feedback of one kind at most once per RohcFeedbackSpacing packets.
    bool ok =
        rohc_decomp_set_features(m_rohcDecompressor,
                                 m_rohcCrcRepair ? ROHC_DECOMP_FEATURE_CRC_REPAIR
                                                 : ROHC_DECOMP_FEATURE_NONE) &&
        rohc_decomp_set_rate_limits(m_rohcDecompressor,
                                    1,
                                    m_rohcFeedbackSpacing,
                                    m_rohcNackFailures,
                                    32,
                                    m_rohcNackFailures,
                                    32);
    NS_ABORT_MSG_UNLESS(ok, "NrPdcp: ROHC decompressor setup failed");
    for (const rohc_profile_t profile : RohcProfileList(m_rohcProfiles))
    {
        NS_ABORT_MSG_UNLESS(rohc_decomp_enable_profile(m_rohcDecompressor, profile),
                            "NrPdcp::RohcProfiles: the ROHC library cannot decompress profile 0x"
                                << std::hex << profile);
    }
#endif
}

void
NrPdcp::FreeRohc()
{
    m_rohcFeedbackEvent.Cancel();
    m_rohcFeedback.clear();
#ifdef NR_ROHC
    if (m_rohcCompressor)
    {
        rohc_comp_free(m_rohcCompressor);
        m_rohcCompressor = nullptr;
    }
    if (m_rohcDecompressor)
    {
        rohc_decomp_free(m_rohcDecompressor);
        m_rohcDecompressor = nullptr;
    }
#endif
}

void
NrPdcp::QueueRohcFeedback(const uint8_t* data, uint32_t size)
{
    // Not sent from inside the receive path of the RLC: the control PDU leaves in its own event,
    // at the same simulation time
    m_rohcFeedback.insert(m_rohcFeedback.end(), data, data + size);
    if (!m_rohcFeedbackEvent.IsPending())
    {
        m_rohcFeedbackEvent = Simulator::ScheduleNow(&NrPdcp::SendRohcFeedback, this);
    }
}

void
NrPdcp::SendRohcFeedback()
{
#ifdef NR_ROHC
    if (m_rohcFeedback.empty() || !m_rlcSapProvider)
    {
        return;
    }
    // TS 38.323 6.2.3.2: octet 1 = D/C (0, control PDU), PDU type (001, interspersed ROHC
    // feedback), four reserved bits; then one ROHC packet with only feedback. No PDCP SN.
    std::vector<uint8_t> pdu;
    pdu.reserve(1 + m_rohcFeedback.size());
    pdu.push_back(0x10);
    pdu.insert(pdu.end(), m_rohcFeedback.begin(), m_rohcFeedback.end());
    RohcReport& r = GetRohcReport(m_cellId, m_rnti, m_lcid, IsUeSide(m_pdcpSapUser));
    r.feedbackTx++;
    r.feedbackTxBytes += m_rohcFeedback.size();
    for (const RohcFeedbackElement& element :
         ParseRohcFeedback(m_rohcFeedback.data(), m_rohcFeedback.size(), m_rohcMaxCid > 15))
    {
        if (element.kind < 3)
        {
            r.feedbackTxKind[element.kind]++;
            RohcReport::FeedbackSent sent;
            sent.sentNs = Simulator::Now().GetNanoSeconds();
            sent.kind = element.kind;
            r.feedbackLog.push_back(sent);
        }
    }
    m_rohcFeedback.clear();

    Ptr<Packet> p = Create<Packet>(pdu.data(), pdu.size());
    NrPdcpTag pdcpTag(Simulator::Now()); // sender timestamp, as on data PDUs
    p->AddByteTag(pdcpTag, 0, 1);
    m_txPdu(m_rnti, m_lcid, p->GetSize());

    NrRlcSapProvider::TransmitPdcpPduParameters txParams;
    txParams.rnti = m_rnti;
    txParams.lcid = m_lcid;
    txParams.pdcpPdu = p;
    NS_LOG_INFO("Transmitting PDCP control PDU with " << pdu.size() - 1
                                                      << " bytes of ROHC feedback");
    m_rlcSapProvider->TransmitPdcpPdu(txParams);
#endif
}

void
NrPdcp::ReceiveControlPdu(Ptr<Packet> p)
{
#ifdef NR_ROHC
    std::vector<uint8_t> pdu(p->GetSize());
    p->CopyData(pdu.data(), pdu.size());
    const uint8_t pduType = (pdu[0] >> 4) & 0x07;
    if (pduType != 1 || pdu.size() < 2)
    {
        NS_LOG_WARN("PDCP control PDU of type " << +pduType << " dropped");
        return;
    }
    // The compressor this feedback is for sends in the other direction than the feedback came
    RohcReport& r = GetRohcReport(m_cellId, m_rnti, m_lcid, !IsUeSide(m_pdcpSapUser));
    r.feedbackRx++;
    const double now = Simulator::Now().GetSeconds();
    NrPdcpTag pdcpTag;
    const bool stamped = p->FindFirstMatchingByteTag(pdcpTag);
    const Time sentAt = stamped ? pdcpTag.GetSenderTimestamp() : Simulator::Now();
    if (stamped)
    {
        r.feedbackDelaySum += (Simulator::Now() - sentAt).GetSeconds();
    }
    // For the report: what kind of feedback this is, and that it arrived. The decompressor
    // entity wrote down when it sent each one; the time stamp of the PDU tells which these are.
    const std::vector<RohcFeedbackElement> elements =
        ParseRohcFeedback(pdu.data() + 1, pdu.size() - 1, m_rohcMaxCid > 15);
    std::string kinds;
    for (const RohcFeedbackElement& element : elements)
    {
        if (element.kind >= 3)
        {
            continue;
        }
        r.feedbackRxKind[element.kind]++;
        kinds += (kinds.empty() ? "" : " + ") + std::string(g_rohcFeedbackName[element.kind]);
        for (RohcReport::FeedbackSent& sent : r.feedbackLog)
        {
            if (sent.arrived < 0 && sent.kind == element.kind &&
                sent.sentNs == sentAt.GetNanoSeconds())
            {
                sent.arrived = now;
                break;
            }
        }
    }
    std::ostringstream startedBy;
    startedBy << std::fixed << std::setprecision(3) << (kinds.empty() ? "feedback" : kinds)
              << " sent at " << sentAt.GetSeconds() << " s";
    if (m_rohcModel == ROHC_MODEL_TABLE)
    {
        // RFC 3095 5.7.6.1, FEEDBACK-2: Acktype, Mode and the sequence number. ACK (0): nothing
        // to do but the mode. NACK (1): the dynamic context is damaged, a context update
        // repairs it. STATIC-NACK (2): the static context is gone, an IR packet is due.
        for (const RohcFeedbackElement& element : elements)
        {
            if (element.mode == 2 && element.crc && !m_tableOMode)
            {
                // RFC 3095 5.6.2: the compressor is in O-mode as soon as any feedback with the
                // mode parameter O and a CRC reaches it
                m_tableOMode = true;
                r.mode = "O-mode";
                NoteRohcMode(r, 'O', now, startedBy.str());
            }
            if (element.kind == 1)
            {
                m_tableUpdateDue = true;
            }
            else if (element.kind == 2)
            {
                m_tableIrLeft = std::max<uint32_t>(m_tableIrLeft, 1);
            }
        }
        return;
    }
    if (!m_rohcCompressor)
    {
        return; // nothing sent yet, so nothing the feedback could be about
    }
    struct rohc_buf feedback;
    feedback.time = RohcNow();
    feedback.data = pdu.data() + 1;
    feedback.max_len = pdu.size() - 1;
    feedback.offset = 0;
    feedback.len = pdu.size() - 1;
    if (!rohc_comp_deliver_feedback2(m_rohcCompressor, feedback))
    {
        NS_LOG_INFO("The ROHC compressor could not use " << feedback.len << " bytes of feedback");
    }
    // The mode the compressor is in after this feedback (the context of its last packet)
    rohc_comp_last_packet_info2_t info;
    info.version_major = 0;
    info.version_minor = 0;
    if (rohc_comp_get_last_packet_info2(m_rohcCompressor, &info))
    {
        r.mode = rohc_get_mode_descr(info.context_mode);
        NoteRohcMode(r, r.mode[0], now, startedBy.str());
    }
#endif
}

uint8_t
NrPdcp::SnSizeBits() const
{
    NS_ABORT_MSG_UNLESS(m_snSize == 7 || m_snSize == 12, "NrPdcp::SnSize must be 7 or 12");
    return m_lcid >= 3 ? m_snSize : 12;
}

Ptr<Packet>
NrPdcp::CompressWithTable(Ptr<Packet> p)
{
#ifdef NR_ROHC
    const uint32_t sduSize = p->GetSize();
    std::vector<uint8_t> sdu(sduSize);
    p->CopyData(sdu.data(), sduSize);

    // The header chain: IPv4, UDP and, if the UDP payload starts like one, an RTP header without
    // CSRC entries (the test the library model uses as well)
    if (sduSize < 28 || (sdu[0] >> 4) != 4 || sdu[9] != 17)
    {
        return nullptr; // not IPv4/UDP: sent as it is
    }
    const uint32_t ipHeader = (sdu[0] & 0x0f) * 4;
    if (ipHeader < 20 || sduSize < ipHeader + 8)
    {
        return nullptr;
    }
    const uint8_t* udpPayload = sdu.data() + ipHeader + 8;
    const uint32_t udpPayloadSize = sduSize - ipHeader - 8;
    const bool rtp = RohcIsRtp(sdu.data(), sdu.data() + ipHeader, udpPayload, udpPayloadSize, nullptr) &&
                     (udpPayload[0] & 0x0f) == 0;
    const bool marker = rtp && (udpPayload[1] & 0x80) != 0;
    const uint32_t chain = ipHeader + 8 + (rtp ? 12 : 0);
    const Time now = Simulator::Now();

    // What is due: the RohcTableIrPackets IR packets of a new flow; in U-mode (as long as no
    // feedback asked for O-mode) the periodic refreshes, one IR or one context update, counted
    // in packets since the last one and, if RohcIrRefreshTime is set, also in time
    const std::string flow = FlowOf(sdu.data(), sduSize);
    if (flow != m_tableFlow)
    {
        m_tableFlow = flow;
        m_tableIrLeft = m_rohcTableIrPackets;
    }
    else if (!m_tableOMode)
    {
        const bool byTime = m_rohcIrRefreshTime.IsStrictlyPositive();
        const Time foTime =
            m_rohcFoRefreshTime.IsStrictlyPositive() ? m_rohcFoRefreshTime : m_rohcIrRefreshTime / 2;
        if (m_tableSinceIr >= m_rohcIrRefresh || (byTime && now - m_tableLastIr >= m_rohcIrRefreshTime))
        {
            m_tableIrLeft = std::max<uint32_t>(m_tableIrLeft, 1);
        }
        else if (m_tableSinceUpdate >= m_rohcFoRefresh || (byTime && now - m_tableLastUpdate >= foTime))
        {
            m_tableUpdateDue = true;
        }
    }

    // The row of the table, and the compressor state the report counts the packet in: IR for an
    // IR packet, FO for a context update that a NACK or a refresh asked for, SO for the rest
    const char* type;
    uint32_t compressed;
    uint8_t firstOctet;
    uint8_t row;
    int st;
    if (m_tableIrLeft > 0)
    {
        --m_tableIrLeft;
        type = "IR";
        row = NrRohcSizeTableTag::IR;
        compressed = chain - 1;
        firstOctet = 0xfd; // RFC 3095 5.7.7.1: 1111110D
        st = 1;
        m_tableUpdateDue = false;
        m_tableSinceIr = 0;
        m_tableSinceUpdate = 0;
        m_tableLastIr = now;
        m_tableLastUpdate = now;
    }
    else if (m_tableUpdateDue || marker || sduSize != m_tableLastSize)
    {
        type = "UOR-2-EXT-3";
        row = NrRohcSizeTableTag::UPDATE;
        compressed = m_rohcTableUpdateSize;
        firstOctet = 0xc0; // RFC 3095 5.7.4: 110 ...
        st = m_tableUpdateDue ? 2 : 3;
        m_tableUpdateDue = false;
        ++m_tableSinceIr;
        m_tableSinceUpdate = 0;
        m_tableLastUpdate = now;
    }
    else
    {
        type = "UO-0";
        row = NrRohcSizeTableTag::UO_0;
        compressed = m_rohcTableUo0Size;
        firstOctet = 0x00; // RFC 3095 5.7.1: 0 ...
        st = 3;
        ++m_tableSinceIr;
        ++m_tableSinceUpdate;
    }
    compressed = std::min(compressed, chain - 1);
    m_tableLastSize = sduSize;

    std::vector<uint8_t> body(compressed + sduSize - chain, 0);
    body[0] = firstOctet;
    std::copy(sdu.begin() + chain, sdu.end(), body.begin() + compressed);
    Ptr<Packet> out = Create<Packet>(body.data(), body.size());
    CopyTags(p, out);
    NrRohcSizeTableTag tag;
    tag.m_type = row;
    tag.m_number = m_tableNumber++;
    tag.m_compressedSize = compressed;
    tag.m_header.assign(sdu.begin(), sdu.begin() + chain);
    out->AddByteTag(tag);

    // The compressor is in the gNB for downlink, in the UE for uplink
    RohcReport& r = GetRohcReport(m_cellId, m_rnti, m_lcid, !IsUeSide(m_pdcpSapUser));
    if (st != m_lastRohcState)
    {
        r.transitions.emplace_back(now.GetSeconds(), st == 1 ? "IR" : st == 2 ? "FO" : "SO");
        m_lastRohcState = st;
    }
    r.packets[st]++;
    r.hdrIn[st] += chain;
    r.hdrOut[st] += compressed;
    r.bytesIn += sduSize;
    r.bytesOut += body.size();
    if (r.firstTx < 0)
    {
        r.firstTx = now.GetSeconds();
    }
    r.lastTx = now.GetSeconds();
    r.profile = rtp ? "size table, IP/UDP/RTP" : "size table, IP/UDP";
    r.mode = m_tableOMode ? "O-mode" : "U-mode";
    NoteRohcMode(r, r.mode[0], now.GetSeconds(), "");
    r.modes.back().packets++;
    RohcReport::TypeCount& ofType = r.packetTypes[type];
    ofType.packets++;
    ofType.hdrOut += compressed;
    r.flows.insert(flow);
    NoteUe(r, sdu.data(), sduSize, !IsUeSide(m_pdcpSapUser));
    NS_LOG_LOGIC("ROHC size table: " << type << ", " << sduSize << " -> " << body.size() << " bytes");
    return out;
#else
    return nullptr;
#endif
}

Ptr<Packet>
NrPdcp::DecompressWithTable(Ptr<Packet> p)
{
#ifdef NR_ROHC
    NrRohcSizeTableTag tag;
    if (!p->FindFirstMatchingByteTag(tag))
    {
        return p; // the sender left the packet as it was: not IPv4/UDP
    }
    NS_ABORT_MSG_IF(m_rohcTableStaticWindow < m_rohcTableDynamicWindow,
                    "NrPdcp::RohcTableStaticWindow must not be smaller than RohcTableDynamicWindow");
    // The decompressor is in the UE for downlink, in the gNB for uplink
    RohcReport& r = GetRohcReport(m_cellId, m_rnti, m_lcid, IsUeSide(m_pdcpSapUser));
    const uint32_t pduSize = p->GetSize();

    // Can the context restore this packet? "gap" = packets missing since the last restored one
    const char* failure = nullptr;
    if (tag.m_type == NrRohcSizeTableTag::IR)
    {
        m_tableRxStatic = true;
        m_tableRxDynamic = true;
    }
    else if (!m_tableRxStatic)
    {
        failure = "needs IR";
    }
    else if (tag.m_number > m_tableRxNumber)
    {
        const uint32_t gap = tag.m_number - m_tableRxNumber - 1;
        if (gap > m_rohcTableStaticWindow)
        {
            m_tableRxStatic = false;
            m_tableRxDynamic = false;
            failure = "needs IR";
        }
        else if (tag.m_type == NrRohcSizeTableTag::UPDATE)
        {
            m_tableRxDynamic = true;
        }
        else if (!m_tableRxDynamic || gap > m_rohcTableDynamicWindow)
        {
            m_tableRxDynamic = false;
            failure = "needs context update";
        }
    }
    else if (!m_tableRxDynamic)
    {
        failure = "needs context update"; // an older packet, and no context to place it
    }

    if (failure || pduSize < tag.m_compressedSize)
    {
        r.decompressFailed++;
        r.rxBytesFailed += pduSize;
        r.failures[failure ? failure : "other"]++;
        NoteRohcFailure(r,
                        Simulator::Now().GetSeconds(),
                        failure ? failure : "other",
                        tag.m_number > m_tableRxNumber ? tag.m_number - m_tableRxNumber - 1 : 0);
        ++m_tableRxFailed;
        ++m_tableRxSinceFeedback;
        // O-mode: ask the compressor for what the context needs now, a NACK (1) for a context
        // update or a STATIC-NACK (2) for an IR packet: after RohcNackFailures packets in a
        // row that failed, and the same kind again every RohcFeedbackSpacing packets
        const uint8_t ackType = m_tableRxStatic ? 1 : 2;
        if (m_rohcMode == ROHC_OPTIMISTIC && m_tableRxFailed >= m_rohcNackFailures &&
            (ackType != m_tableRxLastFeedback || m_tableRxSinceFeedback >= m_rohcFeedbackSpacing))
        {
            const std::array<uint8_t, 5> feedback = RohcTableFeedback(ackType, m_tableRxNumber);
            QueueRohcFeedback(feedback.data(), feedback.size());
            m_tableRxLastFeedback = ackType;
            m_tableRxSinceFeedback = 0;
        }
        return nullptr;
    }
    m_tableRxFailed = 0;
    m_tableRxLastFeedback = 0;
    if (tag.m_number > m_tableRxNumber || tag.m_type == NrRohcSizeTableTag::IR)
    {
        m_tableRxNumber = tag.m_number;
    }
    if (tag.m_type == NrRohcSizeTableTag::IR && m_rohcMode == ROHC_OPTIMISTIC)
    {
        // RFC 3095 5.4.2.2: an IR packet that was restored is answered with an ACK(O). It is
        // also what moves the compressor from U-mode to O-mode (5.6.2), if it arrives.
        const std::array<uint8_t, 5> feedback = RohcTableFeedback(0, tag.m_number);
        QueueRohcFeedback(feedback.data(), feedback.size());
    }

    std::vector<uint8_t> body(pduSize);
    p->CopyData(body.data(), pduSize);
    std::vector<uint8_t> sdu(tag.m_header);
    sdu.insert(sdu.end(), body.begin() + tag.m_compressedSize, body.end());
    Ptr<Packet> out = Create<Packet>(sdu.data(), sdu.size());
    CopyTags(p, out);
    r.decompressed++;
    r.rxBytesIn += pduSize;
    r.rxBytesOut += sdu.size();
    NoteRohcRestored(r,
                     Simulator::Now().GetSeconds(),
                     tag.m_type == NrRohcSizeTableTag::IR       ? "IR"
                     : tag.m_type == NrRohcSizeTableTag::UPDATE ? "UOR-2-EXT-3"
                                                                : "UO-0");
    return out;
#else
    return p;
#endif
}

NrPdcp::NrPdcp()
    : m_pdcpSapUser(nullptr),
      m_rlcSapProvider(nullptr),
      m_rnti(0),
      m_lcid(0),
      m_txSequenceNumber(0),
      m_rxSequenceNumber(0)
{
    NS_LOG_FUNCTION(this);
    m_pdcpSapProvider = new NrPdcpSpecificNrPdcpSapProvider<NrPdcp>(this);
    m_rlcSapUser = new NrPdcpSpecificNrRlcSapUser(this);
}

NrPdcp::~NrPdcp()
{
    NS_LOG_FUNCTION(this);
}

// The attribute descriptions are short, so that --PrintAttributes=ns3::NrPdcp shows one line
// each. What they leave out:
//   SnSize                  7 is the data PDU format of TS 36.323 6.2.4 for bearers on RLC UM.
//                           Signalling bearers keep 12. Both ends of a bearer need the same value.
//   RohcModel               Table: no compressor runs, see "The size-table model of ROHC" at the
//                           top of this file. RohcMode, the refreshes, RohcNackFailures and
//                           RohcFeedbackSpacing work for the table as for the library.
//   RohcTableUo0Size        3 = one octet plus the UDP checksum.
//   RohcTableDynamicWindow  The W-LSB window of the dynamic context. 13 is what the 4 sequence
//                           number bits of UO-0 give the RTP profile (RFC 3095 5.7: the window
//                           reaches from v_ref - 1 to v_ref + 14); 15 for the UDP profile.
//   RohcTableStaticWindow   The W-LSB window of the static context. 61 is what the 6 sequence
//                           number bits of UOR-2 give the RTP profile (v_ref - 1 to v_ref + 62);
//                           31 for the UDP profile. Not smaller than RohcTableDynamicWindow.
//   RohcMode                The mode the decompressor works in. U, unidirectional: only the
//                           periodic refreshes repair a damaged context. O, bidirectional
//                           optimistic: the feedback travels in PDCP control PDUs for interspersed
//                           ROHC feedback, and the compressor repairs the context when asked. The
//                           library has no R-mode.
//   RohcMaxCid              maxCID of TS 38.331 PDCP-Config. Up to 15 the small CID format is
//                           used, above it the large one.
//   RohcProfiles            profiles of TS 38.331 PDCP-Config: 0x0000 uncompressed, 0x0001
//                           RTP/UDP/IP, 0x0002 UDP/IP, 0x0003 ESP/IP, 0x0004 IP, 0x0006 TCP/IP,
//                           0x0101 to 0x0104 their ROHCv2 versions. The compressor takes the best
//                           allowed profile for each flow. Library 2.4.0 does not compress with
//                           0x0101 (ROHCv2 RTP): it sends IR packets only.
//   RohcRepetitions         The optimistic approach of RFC 3095 5.3.1.1.1: how many packets in a
//                           row carry new context information before the compressor takes it as
//                           received. It is the number of IR packets at the start, of packets
//                           after each change or refresh, and the width of the W-LSB window.
//   RohcTableIrPackets      Only the start of a flow: an IR refresh (U-mode) and the answer to a
//                           STATIC-NACK (O-mode) are one IR packet each.
//   RohcIrRefresh           Library: back to the IR state (full headers) every this many packets.
//                           Table: one IR goes out this many packets after the last IR. Without
//                           feedback this is what repairs a lost static context.
//   RohcFoRefresh           Library: back to the FO state every this many packets. Table: one
//                           context update this many packets after the last update or IR.
//   RohcCrcRepair           RFC 3095 5.3.2.2.4; lets a context survive a somewhat longer burst of
//                           lost packets.
//   RohcNackFailures        Library: how many of the last 32 packets of a context must fail (RFC
//                           3095 5.3.2.2.3, k out of n); 10 is the library's default. Table:
//                           packets in a row that could not be restored before the NACK or
//                           STATIC-NACK goes out. 1 asks for a repair at the first failure.
//   RohcFeedbackSpacing     32 is the library's default. Table: a NACK or STATIC-NACK is repeated
//                           every this many packets that could not be restored.
TypeId
NrPdcp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrPdcp")
                            .SetParent<Object>()
                            .SetGroupName("Nr")
                            .AddAttribute("EnableRohc",
                                          "Compress the IP headers on data radio bearers with "
                                          "ROHC (needs the ROHC library)",
                                          BooleanValue(false),
                                          MakeBooleanAccessor(&NrPdcp::SetEnableRohc,
                                                              &NrPdcp::GetEnableRohc),
                                          MakeBooleanChecker())
                            .AddAttribute("SnSize",
                                          "PDCP SN bits on data radio bearers: 12 (2-octet "
                                          "header) or 7 (1 octet); both ends alike",
                                          UintegerValue(12),
                                          MakeUintegerAccessor(&NrPdcp::m_snSize),
                                          MakeUintegerChecker<uint8_t>(7, 12))
                            .AddAttribute("RohcModel",
                                          "Library = the ROHC library on the real headers; Table "
                                          "= header sizes from a table",
                                          EnumValue(ROHC_MODEL_LIBRARY),
                                          MakeEnumAccessor<RohcModel>(&NrPdcp::m_rohcModel),
                                          MakeEnumChecker(ROHC_MODEL_LIBRARY,
                                                          "Library",
                                                          ROHC_MODEL_TABLE,
                                                          "Table"))
                            .AddAttribute("RohcTableUo0Size",
                                          "Table: bytes of the compressed header in steady state "
                                          "(UO-0)",
                                          UintegerValue(3),
                                          MakeUintegerAccessor(&NrPdcp::m_rohcTableUo0Size),
                                          MakeUintegerChecker<uint32_t>(1, 255))
                            .AddAttribute("RohcTableUpdateSize",
                                          "Table: bytes of the compressed header of a context "
                                          "update (UOR-2 with extension 3)",
                                          UintegerValue(13),
                                          MakeUintegerAccessor(&NrPdcp::m_rohcTableUpdateSize),
                                          MakeUintegerChecker<uint32_t>(1, 255))
                            .AddAttribute("RohcTableIrPackets",
                                          "Table: IR packets in a row at the start of a flow",
                                          UintegerValue(1),
                                          MakeUintegerAccessor(&NrPdcp::m_rohcTableIrPackets),
                                          MakeUintegerChecker<uint32_t>(1, 255))
                            .AddAttribute("RohcTableDynamicWindow",
                                          "Table: packets missing in a row that UO-0 survives; "
                                          "more need a context update or IR",
                                          UintegerValue(13),
                                          MakeUintegerAccessor(&NrPdcp::m_rohcTableDynamicWindow),
                                          MakeUintegerChecker<uint32_t>())
                            .AddAttribute("RohcTableStaticWindow",
                                          "Table: packets missing in a row that a context update "
                                          "survives; more need an IR",
                                          UintegerValue(61),
                                          MakeUintegerAccessor(&NrPdcp::m_rohcTableStaticWindow),
                                          MakeUintegerChecker<uint32_t>())
                            .AddAttribute("RohcMode",
                                          "U: no feedback, periodic refresh. O: ACK, NACK, "
                                          "STATIC-NACK in PDCP control PDUs",
                                          EnumValue(ROHC_UNIDIRECTIONAL),
                                          MakeEnumAccessor<RohcMode>(&NrPdcp::m_rohcMode),
                                          MakeEnumChecker(ROHC_UNIDIRECTIONAL,
                                                          "U",
                                                          ROHC_OPTIMISTIC,
                                                          "O"))
                            .AddAttribute("RohcMaxCid",
                                          "ROHC MAX_CID (TS 38.331): MAX_CID + 1 flows per "
                                          "bearer; above 15 the large CID format",
                                          UintegerValue(15),
                                          MakeUintegerAccessor(&NrPdcp::m_rohcMaxCid),
                                          MakeUintegerChecker<uint32_t>(0, 16383))
                            .AddAttribute("RohcProfiles",
                                          "Allowed ROHC profiles (TS 38.331), comma separated: "
                                          "0x0001 RTP, 0x0002 UDP, 0x0004 IP ...",
                                          StringValue("0x0000,0x0001,0x0002,0x0004"),
                                          MakeStringAccessor(&NrPdcp::m_rohcProfiles),
                                          MakeStringChecker())
                            .AddAttribute("RohcRepetitions",
                                          "Library: packets in a row with new context information "
                                          "(IR at start, each change)",
                                          UintegerValue(4),
                                          MakeUintegerAccessor(&NrPdcp::m_rohcRepetitions),
                                          MakeUintegerChecker<uint32_t>(1, 255))
                            .AddAttribute("RohcIrRefresh",
                                          "U-mode: packets after the last IR until the compressor "
                                          "sends IR again",
                                          UintegerValue(1700),
                                          MakeUintegerAccessor(&NrPdcp::m_rohcIrRefresh),
                                          MakeUintegerChecker<uint32_t>(2))
                            .AddAttribute("RohcFoRefresh",
                                          "U-mode: packets until the next FO refresh (Table: "
                                          "context update); less than RohcIrRefresh",
                                          UintegerValue(700),
                                          MakeUintegerAccessor(&NrPdcp::m_rohcFoRefresh),
                                          MakeUintegerChecker<uint32_t>(1))
                            .AddAttribute("RohcIrRefreshTime",
                                          "U-mode: also send IR again after this much time; 0 = "
                                          "by packet count only",
                                          TimeValue(Seconds(0)),
                                          MakeTimeAccessor(&NrPdcp::m_rohcIrRefreshTime),
                                          MakeTimeChecker())
                            .AddAttribute("RohcFoRefreshTime",
                                          "U-mode, with RohcIrRefreshTime: the same for FO; 0 = "
                                          "half of RohcIrRefreshTime",
                                          TimeValue(Seconds(0)),
                                          MakeTimeAccessor(&NrPdcp::m_rohcFoRefreshTime),
                                          MakeTimeChecker())
                            .AddAttribute("RohcCrcRepair",
                                          "Library: after a CRC failure, retry with the SN "
                                          "wrapped around (RFC 3095 5.3.2.2.4)",
                                          BooleanValue(false),
                                          MakeBooleanAccessor(&NrPdcp::m_rohcCrcRepair),
                                          MakeBooleanChecker())
                            .AddAttribute("RohcNackFailures",
                                          "Failed packets before the decompressor gives the "
                                          "context up and, in O-mode, sends a NACK",
                                          UintegerValue(10),
                                          MakeUintegerAccessor(&NrPdcp::m_rohcNackFailures),
                                          MakeUintegerChecker<uint32_t>(1, 32))
                            .AddAttribute("RohcFeedbackSpacing",
                                          "O-mode: packets between two feedbacks of the same kind "
                                          "(1 = no limit)",
                                          UintegerValue(32),
                                          MakeUintegerAccessor(&NrPdcp::m_rohcFeedbackSpacing),
                                          MakeUintegerChecker<uint32_t>(1, 32))
                            .AddAttribute("SilenceSduMaxBytes",
                                          "An SDU up to this size is a silence descriptor, a "
                                          "larger one a voice frame; 0 = not told apart",
                                          UintegerValue(0),
                                          MakeUintegerAccessor(&NrPdcp::m_silenceSduMaxBytes),
                                          MakeUintegerChecker<uint32_t>())
                            .AddTraceSource("TxPDU",
                                            "PDU transmission notified to the RLC.",
                                            MakeTraceSourceAccessor(&NrPdcp::m_txPdu),
                                            "ns3::NrPdcp::PduTxTracedCallback")
                            .AddTraceSource("RxPDU",
                                            "PDU received.",
                                            MakeTraceSourceAccessor(&NrPdcp::m_rxPdu),
                                            "ns3::NrPdcp::PduRxTracedCallback");
    return tid;
}

void
NrPdcp::DoDispose()
{
    NS_LOG_FUNCTION(this);
    FreeRohc();
    m_rlcSapProvider = nullptr;
    delete (m_pdcpSapProvider);
    delete (m_rlcSapUser);
}

void
NrPdcp::SetRnti(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << (uint32_t)rnti);
    m_rnti = rnti;
}

void
NrPdcp::SetCellId(uint16_t cellId)
{
    m_cellId = cellId;
}

void
NrPdcp::SetLcId(uint8_t lcId)
{
    NS_LOG_FUNCTION(this << (uint32_t)lcId);
    m_lcid = lcId;
}

void
NrPdcp::SetNrPdcpSapUser(NrPdcpSapUser* s)
{
    NS_LOG_FUNCTION(this << s);
    m_pdcpSapUser = s;
}

NrPdcpSapProvider*
NrPdcp::GetNrPdcpSapProvider()
{
    NS_LOG_FUNCTION(this);
    return m_pdcpSapProvider;
}

void
NrPdcp::SetNrRlcSapProvider(NrRlcSapProvider* s)
{
    NS_LOG_FUNCTION(this << s);
    m_rlcSapProvider = s;
}

NrRlcSapUser*
NrPdcp::GetNrRlcSapUser()
{
    NS_LOG_FUNCTION(this);
    return m_rlcSapUser;
}

NrPdcp::Status
NrPdcp::GetStatus() const
{
    Status s;
    s.txSn = m_txSequenceNumber;
    s.rxSn = m_rxSequenceNumber;
    return s;
}

void
NrPdcp::SetStatus(Status s)
{
    m_txSequenceNumber = s.txSn;
    m_rxSequenceNumber = s.rxSn;
}

////////////////////////////////////////

void
NrPdcp::DoTransmitPdcpSdu(NrPdcpSapProvider::TransmitPdcpSduParameters params)
{
    NS_LOG_FUNCTION(this << m_rnti << static_cast<uint16_t>(m_lcid) << params.pdcpSdu->GetSize());
    Ptr<Packet> p = params.pdcpSdu;

    // Count the packets of data radio bearers (LCID >= 3) for NrPdcp::GetPacketCounts(). This
    // entity sends: downlink if it is in the gNB. The first packet tells whose bearer this is.
    const bool countPackets = m_lcid >= 3;
    if (countPackets)
    {
        const bool downlink = !IsUeSide(m_pdcpSapUser);
        BearerCounts& counts = GetBearerCounts(m_cellId, m_rnti, m_lcid, downlink);
        uint8_t ip[20];
        if (counts.ue == 0 && p->GetSize() >= sizeof(ip) &&
            p->CopyData(ip, sizeof(ip)) == sizeof(ip) && (ip[0] >> 4) == 4)
        {
            const uint8_t* address = ip + (downlink ? 16 : 12);
            counts.ue = (uint32_t(address[0]) << 24) | (uint32_t(address[1]) << 16) |
                        (uint32_t(address[2]) << 8) | address[3];
        }
        counts.n.sdus++;
        if (downlink && m_silenceSduMaxBytes > 0)
        {
            // The gNB sends this packet: what its size says about the speaker (GetLastSduKind())
            NoteSduKind(m_cellId, m_rnti, m_lcid, true, p->GetSize() <= m_silenceSduMaxBytes);
        }
    }

#ifdef NR_ROHC
    // ROHC on data radio bearers only (LCID >= 3), never on the signalling bearers
    if (m_enableRohc && m_lcid >= 3 && m_rohcModel == ROHC_MODEL_TABLE)
    {
        Ptr<Packet> body = CompressWithTable(p);
        if (body)
        {
            p = body;
        }
    }
    else if (m_enableRohc && m_lcid >= 3)
    {
        CreateRohcCompressor();
        const uint32_t sduSize = p->GetSize();
        std::vector<uint8_t> sdu(sduSize);
        p->CopyData(sdu.data(), sduSize);
        std::vector<uint8_t> rohcPacket(std::max<uint32_t>(2048, sduSize + 256));

        struct rohc_buf in;
        in.time = RohcNow();
        in.data = sdu.data();
        in.max_len = sduSize;
        in.offset = 0;
        in.len = sduSize;
        struct rohc_buf out;
        out.time = in.time;
        out.data = rohcPacket.data();
        out.max_len = rohcPacket.size();
        out.offset = 0;
        out.len = 0;

        // The compressor is in the gNB for downlink, in the UE for uplink
        RohcReport& r = GetRohcReport(m_cellId, m_rnti, m_lcid, !IsUeSide(m_pdcpSapUser));
        if (rohc_compress4(m_rohcCompressor, in, &out) != ROHC_STATUS_OK)
        {
            // Sent as it is, the receiving entity could not tell it from a ROHC packet
            NS_LOG_WARN("ROHC compression failed, SDU of " << sduSize << " bytes dropped");
            r.compressFailed++;
            return;
        }
        rohc_comp_last_packet_info2_t info;
        info.version_major = 0;
        info.version_minor = 0;
        if (rohc_comp_get_last_packet_info2(m_rohcCompressor, &info))
        {
            const double now = Simulator::Now().GetSeconds();
            if (static_cast<int>(info.context_state) != m_lastRohcState)
            {
                r.transitions.emplace_back(now, rohc_comp_get_state_descr(info.context_state));
                m_lastRohcState = static_cast<int>(info.context_state);
            }
            const int st = (info.context_state >= ROHC_COMP_STATE_IR &&
                            info.context_state <= ROHC_COMP_STATE_SO)
                               ? static_cast<int>(info.context_state)
                               : 0;
            r.packets[st]++;
            r.hdrIn[st] += info.header_last_uncomp_size;
            r.hdrOut[st] += info.header_last_comp_size;
            r.bytesIn += sduSize;
            r.bytesOut += out.len;
            if (r.firstTx < 0)
            {
                r.firstTx = now;
            }
            r.lastTx = now;
            r.profile = rohc_get_profile_descr(static_cast<rohc_profile_t>(info.profile_id));
            r.mode = rohc_get_mode_descr(info.context_mode);
            NoteRohcMode(r, r.mode[0], now, "");
            r.modes.back().packets++;
            RohcReport::TypeCount& ofType = r.packetTypes[rohc_get_packet_descr(info.packet_type)];
            ofType.packets++;
            ofType.hdrOut += info.header_last_comp_size;
            r.flows.insert(FlowOf(sdu.data(), sduSize));
            NoteUe(r, sdu.data(), sduSize, !IsUeSide(m_pdcpSapUser));
        }
        Ptr<Packet> compressed = Create<Packet>(out.data + out.offset, out.len);
        CopyTags(p, compressed);
        NrRohcCheckTag check; // for the receiving entity: what this packet was
        check.m_hash = RohcPacketHash(sdu.data(), sduSize);
        compressed->AddByteTag(check);
        NS_LOG_LOGIC("ROHC compression: " << sduSize << " -> " << out.len << " bytes");
        p = compressed;
    }
#endif

    if (countPackets)
    {
        p->AddByteTag(MakePdcpCheckTag(p)); // for the receiving entity: the body as it is sent
    }

    // Sender timestamp
    NrPdcpTag pdcpTag(Simulator::Now());

    NrPdcpHeader pdcpHeader;
    pdcpHeader.SetSnSizeBits(SnSizeBits());
    pdcpHeader.SetSequenceNumber(m_txSequenceNumber);

    m_txSequenceNumber++;
    if (m_txSequenceNumber >= (1U << SnSizeBits()))
    {
        m_txSequenceNumber = 0;
    }

    pdcpHeader.SetDcBit(NrPdcpHeader::DATA_PDU);
    p->AddHeader(pdcpHeader);
    p->AddByteTag(pdcpTag, 0, pdcpHeader.GetSerializedSize()); // on the header bytes, see NrRlcUm

    m_txPdu(m_rnti, m_lcid, p->GetSize());

    NrRlcSapProvider::TransmitPdcpPduParameters txParams;
    txParams.rnti = m_rnti;
    txParams.lcid = m_lcid;
    txParams.pdcpPdu = p;

    if (countPackets)
    {
        BearerCounts& counts = GetBearerCounts(m_cellId, m_rnti, m_lcid, !IsUeSide(m_pdcpSapUser));
        counts.n.sent++;
        g_bearerHandingOver = &counts; // the RLC may refuse the PDU: NotePduDiscarded()
    }
    NS_LOG_INFO("Transmitting PDCP PDU with header: " << pdcpHeader);
    m_rlcSapProvider->TransmitPdcpPdu(txParams);
    g_bearerHandingOver = nullptr;
}

void
NrPdcp::DoReceivePdu(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid << p->GetSize());

    // Receiver timestamp
    NrPdcpTag pdcpTag;
    Time delay;
    p->FindFirstMatchingByteTag(pdcpTag);
    delay = Simulator::Now() - pdcpTag.GetSenderTimestamp();
    m_rxPdu(m_rnti, m_lcid, p->GetSize(), delay.GetNanoSeconds());

#ifdef NR_ROHC
    // A PDCP control PDU (D/C bit 0)? With ROHC on, the peer entity sends its feedback in them.
    if (m_enableRohc && m_lcid >= 3 && p->GetSize() >= 1)
    {
        uint8_t firstOctet = 0;
        p->CopyData(&firstOctet, 1);
        if ((firstOctet & 0x80) == 0)
        {
            ReceiveControlPdu(p);
            return;
        }
    }
#endif

    NrPdcpHeader pdcpHeader;
    pdcpHeader.SetSnSizeBits(SnSizeBits());
    p->RemoveHeader(pdcpHeader);
    NS_LOG_LOGIC("PDCP header: " << pdcpHeader);

    // A data PDU of a data radio bearer arrived (see DoTransmitPdcpSdu). This entity receives:
    // downlink if it is in the UE. A PDU that is not the bytes that were sent is dropped here,
    // see NrPdcpCheckTag.
    if (m_lcid >= 3)
    {
        BearerCounts& counts = GetBearerCounts(m_cellId, m_rnti, m_lcid, IsUeSide(m_pdcpSapUser));
        if (!IsPdcpPduIntact(p))
        {
            NS_LOG_INFO("PDCP PDU of " << p->GetSize() << " bytes is not one that was sent, dropped");
            counts.n.damaged++;
            return;
        }
        counts.n.arrived++;
    }

    m_rxSequenceNumber = pdcpHeader.GetSequenceNumber() + 1;
    if (m_rxSequenceNumber >= (1U << SnSizeBits()))
    {
        m_rxSequenceNumber = 0;
    }

#ifdef NR_ROHC
    // ROHC on data radio bearers only (LCID >= 3), never on the signalling bearers
    if (m_enableRohc && m_lcid >= 3 && m_rohcModel == ROHC_MODEL_TABLE)
    {
        p = DecompressWithTable(p);
        if (!p)
        {
            return;
        }
    }
    else if (m_enableRohc && m_lcid >= 3)
    {
        CreateRohcDecompressor();
        const uint32_t pduSize = p->GetSize();
        std::vector<uint8_t> rohcPacket(pduSize);
        p->CopyData(rohcPacket.data(), pduSize);
        std::vector<uint8_t> sdu(std::max<uint32_t>(2048, pduSize + 256));

        struct rohc_buf in;
        in.time = RohcNow();
        in.data = rohcPacket.data();
        in.max_len = pduSize;
        in.offset = 0;
        in.len = pduSize;
        struct rohc_buf out;
        out.time = in.time;
        out.data = sdu.data();
        out.max_len = sdu.size();
        out.offset = 0;
        out.len = 0;

        // In O-mode the decompressor may answer a packet, decompressed or not, with feedback
        // for the peer compressor. Feedback never arrives inside a data PDU here.
        uint8_t feedbackBytes[256];
        struct rohc_buf feedback;
        feedback.time = in.time;
        feedback.data = feedbackBytes;
        feedback.max_len = sizeof(feedbackBytes);
        feedback.offset = 0;
        feedback.len = 0;

        // The decompressor is in the UE for downlink, in the gNB for uplink
        RohcReport& r = GetRohcReport(m_cellId, m_rnti, m_lcid, IsUeSide(m_pdcpSapUser));
        const rohc_status_t status =
            rohc_decompress3(m_rohcDecompressor,
                             in,
                             &out,
                             nullptr,
                             m_rohcMode == ROHC_OPTIMISTIC ? &feedback : nullptr);
        if (feedback.len > 0)
        {
            QueueRohcFeedback(feedback.data + feedback.offset, feedback.len);
        }
        // Did the library give back the packet that was sent? See NrRohcCheckTag.
        NrRohcCheckTag check;
        const bool wrong = status == ROHC_STATUS_OK && p->FindFirstMatchingByteTag(check) &&
                           check.m_hash != RohcPacketHash(out.data + out.offset, out.len);
        if (status != ROHC_STATUS_OK || wrong)
        {
            NS_LOG_INFO("ROHC decompression failed, PDU of " << pduSize << " bytes dropped");
            const char* reason = wrong                              ? "header restored wrongly"
                                 : status == ROHC_STATUS_BAD_CRC    ? "CRC failure"
                                 : status == ROHC_STATUS_NO_CONTEXT ? "no context"
                                 : status == ROHC_STATUS_MALFORMED  ? "rejected in this state"
                                                                    : "other";
            r.decompressFailed++;
            r.rxBytesFailed += pduSize;
            r.failures[reason]++;
            NoteRohcFailure(r, Simulator::Now().GetSeconds(), reason, -1);
            return;
        }
        Ptr<Packet> decompressed = Create<Packet>(out.data + out.offset, out.len);
        CopyTags(p, decompressed);
        NS_LOG_LOGIC("ROHC decompression: " << pduSize << " -> " << out.len << " bytes");
        r.decompressed++;
        r.rxBytesIn += pduSize;
        r.rxBytesOut += out.len;
        if (!r.contextLosses.empty() && r.contextLosses.back().open)
        {
            // The first packet restored after a context loss: its type is what repaired it
            rohc_decomp_last_packet_info_t last;
            last.version_major = 0;
            last.version_minor = 1;
            NoteRohcRestored(r,
                             Simulator::Now().GetSeconds(),
                             rohc_decomp_get_last_packet_info(m_rohcDecompressor, &last)
                                 ? ShortPacketType(rohc_get_packet_descr(last.packet_type))
                                 : "?");
        }
        p = decompressed;
    }
#endif

    if (m_lcid >= 3)
    {
        GetBearerCounts(m_cellId, m_rnti, m_lcid, IsUeSide(m_pdcpSapUser)).n.delivered++;
        if (!IsUeSide(m_pdcpSapUser) && m_silenceSduMaxBytes > 0)
        {
            // The gNB received this packet from the UE
            NoteSduKind(m_cellId, m_rnti, m_lcid, false, p->GetSize() <= m_silenceSduMaxBytes);
        }
    }

    NrPdcpSapUser::ReceivePdcpSduParameters params;
    params.pdcpSdu = p;
    params.rnti = m_rnti;
    params.lcid = m_lcid;
    m_pdcpSapUser->ReceivePdcpSdu(params);
}

} // namespace ns3
