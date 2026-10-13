// Copyright (c) 2026
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-pdcch.h"

#include "ns3/abort.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrPdcch");
NS_OBJECT_ENSURE_REGISTERED(NrPdcch);

namespace
{

const std::array<uint8_t, 5> g_levels = {1, 2, 4, 8, 16};

/// The PDCCH of each cell. Emptied when the simulator is destroyed.
std::map<uint16_t, Ptr<NrPdcch>> g_pdcch;
bool g_pdcchCleanupScheduled = false;

void
ClearPdcch()
{
    g_pdcch.clear();
    g_pdcchCleanupScheduled = false;
}

/// Candidates of a common search space set at each level (TS 38.213 Table 10.1-1)
uint32_t
CommonCandidates(uint8_t al)
{
    switch (al)
    {
    case 4:
        return 4;
    case 8:
        return 2;
    case 16:
        return 1;
    default:
        return 0;
    }
}

} // namespace

TypeId
NrPdcch::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrPdcch")
            .SetParent<Object>()
            .AddConstructor<NrPdcch>()
            .AddAttribute("Enabled",
                          "Model the PDCCH: CCEs, aggregation levels, blocking.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrPdcch::m_enabled),
                          MakeBooleanChecker())
            .AddAttribute("Symbols",
                          "PDCCH symbols per slot: 1, 2 or 3; 0 = as many as each slot needs.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NrPdcch::m_symbols),
                          MakeUintegerChecker<uint8_t>(0, 3))
            .AddAttribute("DciBits",
                          "Size of a DCI in bits, without its 24-bit CRC.",
                          UintegerValue(34),
                          MakeUintegerAccessor(&NrPdcch::m_dciBits),
                          MakeUintegerChecker<uint32_t>(12, 140))
            .AddAttribute("CoresetRbs",
                          "Resource blocks of the CORESET, a multiple of 6; 0 = all the carrier has.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrPdcch::m_coresetRbsAttr),
                          MakeUintegerChecker<uint32_t>(0, 270))
            .AddAttribute("Candidates",
                          "PDCCH candidates of a UE at aggregation level 1,2,4,8,16.",
                          StringValue("6,6,4,2,1"),
                          MakeStringAccessor(&NrPdcch::SetCandidates,
                                             &NrPdcch::GetCandidatesString),
                          MakeStringChecker())
            .AddAttribute("BlerTable",
                          "PDCCH block error rate at aggregation level 1: SINR(dB):BLER pairs.",
                          StringValue("-4:0.9303 -2:0.4356 -1:0.1453 0:0.0246 0.5:0.00797 "
                                      "1:0.00221"),
                          MakeStringAccessor(&NrPdcch::SetBlerTable, &NrPdcch::GetBlerTable),
                          MakeStringChecker())
            .AddAttribute("TableDciBits",
                          "DCI size the BlerTable was made for (bits, without CRC).",
                          UintegerValue(34),
                          MakeUintegerAccessor(&NrPdcch::m_tableDciBits),
                          MakeUintegerChecker<uint32_t>(12, 140))
            .AddAttribute("TargetBler",
                          "The aggregation level is the lowest with a block error rate up to this.",
                          DoubleValue(0.01),
                          MakeDoubleAccessor(&NrPdcch::m_targetBler),
                          MakeDoubleChecker<double>(1e-6, 1.0))
            .AddAttribute("SinrMarginDb",
                          "Taken off the reported SINR before the aggregation level is chosen (dB).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrPdcch::m_sinrMarginDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("SinrOffsetDb",
                          "Added to every SINR before the BlerTable is read (dB).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrPdcch::m_sinrOffsetDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("CommonAl",
                          "Aggregation level of SIB1 and random access DCIs: up to 4, 8 or 16.",
                          UintegerValue(8),
                          MakeUintegerAccessor(&NrPdcch::m_commonAlAttr),
                          MakeUintegerChecker<uint8_t>(4, 16))
            .AddAttribute("InitialAl",
                          "Aggregation level before a UE has reported its SINR; 0 = the highest.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrPdcch::m_initialAl),
                          MakeUintegerChecker<uint8_t>(0, 16))
            .AddAttribute("RaiseAl",
                          "If the chosen aggregation level has no free candidate, try higher ones.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrPdcch::m_raiseAl),
                          MakeBooleanChecker())
            .AddAttribute("Sib1PeriodMs",
                          "A SIB1 DCI is sent every this many ms (a multiple of 10); 0 = none.",
                          UintegerValue(20),
                          MakeUintegerAccessor(&NrPdcch::m_sib1PeriodMs),
                          MakeUintegerChecker<uint32_t>(0, 160))
            .AddAttribute("UlMaxShare",
                          "Share of the CCEs uplink grants may take from downlink assignments.",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&NrPdcch::m_ulMaxShare),
                          MakeDoubleChecker<double>(0.0, 1.0));
    return tid;
}

NrPdcch::NrPdcch()
{
}

NrPdcch::~NrPdcch()
{
}

Ptr<NrPdcch>
NrPdcch::Get(uint16_t cellId)
{
    auto it = g_pdcch.find(cellId);
    if (it == g_pdcch.end())
    {
        it = g_pdcch.emplace(cellId, CreateObject<NrPdcch>()).first;
        if (!g_pdcchCleanupScheduled)
        {
            g_pdcchCleanupScheduled = true;
            Simulator::ScheduleDestroy(&ClearPdcch);
        }
    }
    return it->second;
}

Ptr<NrPdcch>
NrPdcch::Find(uint16_t cellId)
{
    const auto it = g_pdcch.find(cellId);
    return it == g_pdcch.end() ? nullptr : it->second;
}

int
NrPdcch::AlIndex(uint8_t al)
{
    switch (al)
    {
    case 1:
        return 0;
    case 2:
        return 1;
    case 4:
        return 2;
    case 8:
        return 3;
    case 16:
        return 4;
    default:
        return -1;
    }
}

void
NrPdcch::SetBlerTable(const std::string& table)
{
    std::string s = table;
    std::replace(s.begin(), s.end(), ',', ' ');
    std::replace(s.begin(), s.end(), ';', ' ');
    std::istringstream is(s);
    std::string pair;
    std::vector<std::pair<double, double>> points;
    while (is >> pair)
    {
        const auto colon = pair.find(':');
        NS_ABORT_MSG_IF(colon == std::string::npos,
                        "NrPdcch::BlerTable: '" << pair << "' is not SINR:BLER");
        const double sinrDb = std::stod(pair.substr(0, colon));
        const double bler = std::stod(pair.substr(colon + 1));
        NS_ABORT_MSG_UNLESS(bler > 0.0 && bler <= 1.0,
                            "NrPdcch::BlerTable: the block error rate " << bler
                                                                        << " is not in (0, 1]");
        points.emplace_back(sinrDb, std::log10(bler));
    }
    NS_ABORT_MSG_IF(points.size() < 2, "NrPdcch::BlerTable needs at least two SINR:BLER pairs");
    std::sort(points.begin(), points.end());
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        NS_ABORT_MSG_UNLESS(points[i].first > points[i - 1].first &&
                                points[i].second <= points[i - 1].second,
                            "NrPdcch::BlerTable: the block error rate must fall as the SINR rises");
    }
    NS_ABORT_MSG_UNLESS(points.back().second < points.front().second,
                        "NrPdcch::BlerTable: the block error rate must fall as the SINR rises");
    m_table = points;
}

std::string
NrPdcch::GetBlerTable() const
{
    std::ostringstream os;
    for (std::size_t i = 0; i < m_table.size(); ++i)
    {
        os << (i ? " " : "") << m_table[i].first << ":" << std::pow(10.0, m_table[i].second);
    }
    return os.str();
}

void
NrPdcch::SetCandidates(const std::string& candidates)
{
    std::string s = candidates;
    std::replace(s.begin(), s.end(), ',', ' ');
    std::istringstream is(s);
    std::array<uint32_t, 5> values{};
    for (auto& v : values)
    {
        NS_ABORT_MSG_UNLESS(is >> v,
                            "NrPdcch::Candidates needs five numbers, for aggregation level "
                            "1,2,4,8,16");
        // nrofCandidates: n0, n1, n2, n3, n4, n5, n6, n8 (TS 38.331 SearchSpace)
        NS_ABORT_MSG_UNLESS(v <= 6 || v == 8,
                            "NrPdcch::Candidates: " << v << " is not one of 0 to 6 and 8");
    }
    m_candidates = values;
}

std::string
NrPdcch::GetCandidatesString() const
{
    std::ostringstream os;
    for (std::size_t i = 0; i < m_candidates.size(); ++i)
    {
        os << (i ? "," : "") << m_candidates[i];
    }
    return os.str();
}

bool
NrPdcch::IsEnabled() const
{
    return m_enabled;
}

uint8_t
NrPdcch::GetConfiguredSymbols() const
{
    return m_symbols;
}

uint8_t
NrPdcch::GetMinSymbols() const
{
    return m_symbols == 0 ? 1 : m_symbols;
}

uint8_t
NrPdcch::GetMaxSymbols() const
{
    return m_symbols == 0 ? 3 : m_symbols;
}

uint32_t
NrPdcch::GetDciBits() const
{
    return m_dciBits;
}

void
NrPdcch::SetCarrier(uint32_t dlRbs, uint8_t numerology)
{
    // frequencyDomainResources: a bitmap of groups of 6 RBs (TS 38.213 10.1, TS 38.331)
    const uint32_t rbs = m_coresetRbsAttr == 0 ? (dlRbs / 6) * 6 : m_coresetRbsAttr;
    NS_ABORT_MSG_UNLESS(rbs % 6 == 0, "NrPdcch::CoresetRbs must be a multiple of 6");
    NS_ABORT_MSG_UNLESS(rbs >= 6 && rbs <= dlRbs,
                        "NrPdcch: a CORESET of " << rbs << " RBs does not fit a carrier of "
                                                 << dlRbs << " RBs");
    if (m_hasCarrier)
    {
        NS_ABORT_MSG_UNLESS(rbs == m_coresetRbs && numerology == m_numerology,
                            "NrPdcch: one PDCCH for two different downlink carriers of a cell");
        return;
    }
    m_coresetRbs = rbs;
    m_numerology = numerology;
    m_hasCarrier = true;
    NS_ABORT_MSG_UNLESS(m_sib1PeriodMs % 10 == 0, "NrPdcch::Sib1PeriodMs must be a multiple of 10");
    NS_ABORT_MSG_IF(m_initialAl != 0 && AlIndex(m_initialAl) < 0,
                    "NrPdcch::InitialAl must be 0, 1, 2, 4, 8 or 16");
    NS_ABORT_MSG_IF(ChooseAl(std::numeric_limits<double>::quiet_NaN()) == 0,
                    "NrPdcch: the CORESET of " << rbs << " RBs and " << +GetMaxSymbols()
                                               << " symbol(s) holds no PDCCH candidate of the "
                                                  "UE-specific search space");
}

bool
NrPdcch::HasCarrier() const
{
    return m_hasCarrier;
}

uint32_t
NrPdcch::GetCoresetRbs() const
{
    return m_coresetRbs;
}

uint32_t
NrPdcch::GetNumCce(uint8_t symbols) const
{
    // A CCE is 6 REGs, a REG one RB in one symbol (TS 38.211 7.3.2.2)
    return m_coresetRbs * symbols / 6;
}

uint32_t
NrPdcch::GetCandidates(uint8_t al) const
{
    const int i = AlIndex(al);
    return i < 0 ? 0 : m_candidates[i];
}

uint32_t
NrPdcch::GetMonitoredCandidates() const
{
    // The common search space set: 4 + 2 + 1 candidates, as far as the CORESET holds them
    uint32_t n = 0;
    const uint32_t cces = GetNumCce(GetMaxSymbols());
    for (const uint8_t al : g_levels)
    {
        if (al <= cces)
        {
            n += CommonCandidates(al) + GetCandidates(al);
        }
    }
    return n;
}

uint32_t
NrPdcch::GetMaxMonitoredCandidates(uint8_t numerology)
{
    // TS 38.213 Table 10.1-2, a single serving cell
    static const std::array<uint32_t, 4> limit = {44, 36, 22, 20};
    return limit.at(std::min<uint8_t>(numerology, 3));
}

uint8_t
NrPdcch::GetCommonAl() const
{
    const uint32_t cces = GetNumCce(GetMaxSymbols());
    uint8_t al = 0;
    for (const uint8_t l : {uint8_t(4), uint8_t(8), uint8_t(16)})
    {
        if (l <= m_commonAlAttr && l <= cces)
        {
            al = l;
        }
    }
    return al;
}

double
NrPdcch::GetBler(uint8_t al, double sinrDb, double boostDb) const
{
    NS_ASSERT(AlIndex(al) >= 0);
    // Payload and CRC (TS 38.212 7.3.2) on 108 coded bits per CCE: more than fits is never decoded
    const double payload = m_dciBits + 24.0;
    if (payload > 108.0 * al)
    {
        return 1.0;
    }
    // The table is for level 1 and a DCI of TableDciBits. L CCEs carry L times the energy; a
    // payload of another size needs its energy per information bit.
    const double x = sinrDb + m_sinrOffsetDb + boostDb + 10.0 * std::log10(static_cast<double>(al)) -
                     10.0 * std::log10(payload / (m_tableDciBits + 24.0));
    // Straight lines through log10(BLER), also beyond the two ends
    std::size_t hi = 1;
    while (hi + 1 < m_table.size() && x > m_table[hi].first)
    {
        ++hi;
    }
    const auto& a = m_table[hi - 1];
    const auto& b = m_table[hi];
    const double y = a.second + (b.second - a.second) * (x - a.first) / (b.first - a.first);
    return std::min(1.0, std::pow(10.0, y));
}

double
NrPdcch::GetSinrForBler(uint8_t al, double bler) const
{
    double lo = -100.0;
    double hi = 100.0;
    for (int i = 0; i < 60; ++i)
    {
        const double mid = 0.5 * (lo + hi);
        if (GetBler(al, mid) > bler)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    return hi;
}

uint8_t
NrPdcch::ChooseAl(double sinrDb, bool* meetsTarget) const
{
    const uint32_t cces = GetNumCce(GetMaxSymbols());
    const bool noReport = std::isnan(sinrDb);
    uint8_t highest = 0;
    if (meetsTarget)
    {
        *meetsTarget = true;
    }
    for (const uint8_t al : g_levels)
    {
        if (GetCandidates(al) == 0 || al > cces)
        {
            continue;
        }
        highest = al;
        if (noReport)
        {
            if (al == m_initialAl)
            {
                return al;
            }
            continue;
        }
        if (GetBler(al, sinrDb - m_sinrMarginDb) <= m_targetBler)
        {
            return al;
        }
    }
    if (meetsTarget && !noReport)
    {
        *meetsTarget = false;
    }
    return highest;
}

void
NrPdcch::ReportSinr(uint16_t rnti, double sinrDb)
{
    m_sinrDb[rnti] = sinrDb;
}

double
NrPdcch::GetSinr(uint16_t rnti) const
{
    const auto it = m_sinrDb.find(rnti);
    return it == m_sinrDb.end() ? std::numeric_limits<double>::quiet_NaN() : it->second;
}

void
NrPdcch::RemoveUe(uint16_t rnti)
{
    m_sinrDb.erase(rnti);
}

bool
NrPdcch::IsUplink(Purpose purpose) const
{
    return purpose == UL_NEW || purpose == UL_RETX || purpose == UL_CG_ACTIVATION ||
           purpose == UL_CG_RELEASE;
}

bool
NrPdcch::IsCommon(Purpose purpose) const
{
    return purpose == SIB1 || purpose == RAR;
}

int32_t
NrPdcch::CandidateCce(uint16_t rnti, bool common, uint8_t al, uint32_t m, uint8_t d) const
{
    const uint32_t nCce = GetNumCce(d);
    const uint32_t nCandidates = common ? CommonCandidates(al) : GetCandidates(al);
    if (al > nCce || m >= nCandidates)
    {
        return -1;
    }
    // TS 38.213 10.1: the CCEs of candidate m of aggregation level L in slot n of the frame are
    //   L x { (Y_{p,n} + floor(m N_CCE / (L M)) + n_CI) mod floor(N_CCE / L) } + i,  i = 0..L-1
    // with n_CI = 0 (no cross-carrier scheduling); Y = 0 for any common search space; for a
    // UE-specific one Y_{p,n} = (A_p Y_{p,n-1}) mod D, Y_{p,-1} = n_RNTI (the C-RNTI),
    // A_p = 39827, 39829, 39839 for p mod 3 = 0, 1, 2 and D = 65537.
    uint64_t y = 0;
    if (!common)
    {
        static const std::array<uint64_t, 3> a = {39827, 39829, 39839};
        // CORESET index p: 1 with a fixed number of symbols; with a dynamic one the CORESETs of
        // 1, 2 and 3 symbols are numbered 1, 2 and 3
        const uint32_t p = m_symbols == 0 ? d : 1;
        const uint32_t slotsPerFrame = 10U << m_numerology;
        const uint32_t n = static_cast<uint32_t>(m_absSlot % slotsPerFrame);
        y = rnti;
        for (uint32_t i = 0; i <= n; ++i)
        {
            y = (a[p % 3] * y) % 65537;
        }
    }
    const uint64_t index = (y + (static_cast<uint64_t>(m) * nCce) / (al * nCandidates)) % (nCce / al);
    return static_cast<int32_t>(al * index);
}

int32_t
NrPdcch::FindFree(uint16_t rnti, bool common, uint8_t al, uint8_t d) const
{
    const auto& owner = m_owner[d];
    const uint32_t nCandidates = common ? CommonCandidates(al) : GetCandidates(al);
    for (uint32_t m = 0; m < nCandidates; ++m)
    {
        const int32_t cce = CandidateCce(rnti, common, al, m, d);
        if (cce < 0)
        {
            return -1;
        }
        bool free = true;
        for (uint32_t i = 0; i < al && free; ++i)
        {
            free = owner[cce + i] < 0;
        }
        if (free)
        {
            return cce;
        }
    }
    return -1;
}

void
NrPdcch::Occupy(uint8_t d, int32_t cce, uint8_t al, uint32_t id)
{
    for (uint32_t i = 0; i < al; ++i)
    {
        m_owner[d][cce + i] = static_cast<int32_t>(id);
    }
}

void
NrPdcch::Vacate(uint8_t d, int32_t cce, uint8_t al)
{
    for (uint32_t i = 0; i < al; ++i)
    {
        m_owner[d][cce + i] = -1;
    }
}

void
NrPdcch::NewSlotIfNeeded()
{
    const int64_t now = Simulator::Now().GetNanoSeconds();
    if (now == m_slotNs)
    {
        return;
    }
    FinishSlot();
    m_prevDlDemandCce = m_dlDemandCce;
    m_slotNs = now;
    m_dlStarted = false;
    m_fixed = m_symbols != 0;
    m_slotSymbols = GetMaxSymbols();
    m_entries.clear();
    m_ulCce = 0;
    m_dlDemandCce = 0;
    for (uint8_t d = 1; d <= 3; ++d)
    {
        m_owner[d].assign(m_hasCarrier ? GetNumCce(d) : 0, -1);
    }
    if (!m_hasCarrier)
    {
        return;
    }
    // The credit of the uplink grants: UlMaxShare of the CCEs of every slot, saved up to the
    // size of the largest grant the CORESET holds (or to one slot's share, if that is more)
    const uint32_t nCce = GetNumCce(m_slotSymbols);
    const double share = m_ulMaxShare * nCce;
    m_ulCredit = m_ulMaxShare > 0.0
                     ? std::min(m_ulCredit + share, std::max<double>(share, std::min(16U, nCce)))
                     : 0.0;
    // The slot these requests are for: the downlink scheduler names it (StartDownlink); until it
    // does, it is as far ahead of the clock as it was the last time
    const int64_t nowSlot = (now << m_numerology) / 1000000;
    m_absSlot = static_cast<uint64_t>(nowSlot + m_latencySlots);
    // SIB1 has a PDCCH of its own in the common search space (DCI format 1_0, SI-RNTI); the
    // default period of its repetitions is 20 ms (TS 38.331 5.2.1). It is placed first.
    if (m_enabled && m_absSlotKnown && m_sib1PeriodMs > 0)
    {
        const uint64_t slotsPerFrame = 10U << m_numerology;
        const uint64_t frame = m_absSlot / slotsPerFrame;
        const uint64_t slotInFrame = m_absSlot % slotsPerFrame;
        if (frame % (m_sib1PeriodMs / 10) == 0 && slotInFrame == slotsPerFrame / 2)
        {
            const Grant grant = Request(0, SIB1);
            if (grant)
            {
                Commit(grant, SIB1, nullptr);
            }
        }
    }
}

bool
NrPdcch::StartDownlink(const SfnSf& target)
{
    const int64_t now = Simulator::Now().GetNanoSeconds();
    if (m_ulSeen && m_ulNs != now && m_deferNs != now)
    {
        // An uplink scheduler exists and has not been asked yet at this time: let it go first
        m_deferNs = now;
        return true;
    }
    const int64_t nowSlot = (now << m_numerology) / 1000000;
    const int64_t latency = static_cast<int64_t>(target.Normalize()) - nowSlot;
    if (!m_absSlotKnown || latency != m_latencySlots)
    {
        // First call, or the distance changed: what was assumed for this slot may be wrong, so
        // start it again with the right number (nothing can have been reserved the first time)
        NS_ABORT_MSG_IF(m_absSlotKnown && now == m_slotNs && !m_entries.empty(),
                        "NrPdcch: the slot a downlink scheduler works on moved");
        m_latencySlots = latency;
        m_absSlotKnown = true;
        if (now == m_slotNs)
        {
            m_slotNs = -1;
            m_dlDemandCce = m_prevDlDemandCce;
            m_entries.clear();
        }
    }
    NewSlotIfNeeded();
    m_dlStarted = true;
    return false;
}

void
NrPdcch::StartUplink()
{
    NewSlotIfNeeded();
    m_ulSeen = true;
    m_ulNs = Simulator::Now().GetNanoSeconds();
}

NrPdcch::Grant
NrPdcch::Request(uint16_t rnti, Purpose purpose)
{
    NewSlotIfNeeded();
    if (!m_hasCarrier)
    {
        return Grant();
    }
    const bool common = IsCommon(purpose);
    const bool uplink = IsUplink(purpose);
    const uint8_t d = m_slotSymbols; // the most the slot can have, or what it has been fixed to
    const uint32_t nCce = GetNumCce(d);

    Entry entry;
    entry.rnti = rnti;
    entry.purpose = purpose;
    uint8_t wanted;
    if (common)
    {
        wanted = GetCommonAl();
    }
    else
    {
        const double sinrDb = GetSinr(rnti);
        entry.noReport = std::isnan(sinrDb);
        wanted = ChooseAl(sinrDb, &entry.meetsTarget);
    }
    if (!uplink)
    {
        m_dlDemandCce += wanted;
    }

    int32_t cce = -1;
    uint8_t al = 0;
    uint8_t charge = 0;
    bool capped = false;
    for (const uint8_t l : g_levels)
    {
        if (l < wanted || l > nCce || (l > wanted && (common || !m_raiseAl)))
        {
            continue;
        }
        if (uplink && !common)
        {
            // Uplink grants are asked for before the downlink assignments of the same PDCCH. They
            // may take the CCEs the downlink did not ask for in the slot before, or their own
            // share, UlMaxShare of the CORESET, whichever is more. The share is a credit that
            // grows slot by slot, so that a grant larger than the share of one slot (level 4 in
            // a CORESET of 4 CCEs) still gets its turn.
            const uint32_t idle = nCce - std::min(nCce, m_prevDlDemandCce);
            if (m_ulCce + l > idle && m_ulCredit + 1e-9 < l)
            {
                // (held back by the share only if a candidate was free: a CORESET that the
                // uplink grants before this one have filled has no room either way)
                capped = capped || FindFree(rnti, common, l, d) >= 0;
                continue;
            }
            charge = static_cast<uint8_t>(std::min<double>(l, std::floor(m_ulCredit + 1e-9)));
        }
        cce = FindFree(rnti, common, l, d);
        if (cce >= 0)
        {
            al = l;
            break;
        }
    }
    if (cce < 0)
    {
        if (m_counting && m_enabled)
        {
            ++m_stats.blocked[purpose];
            m_stats.ulCapped += capped;
        }
        return Grant();
    }
    entry.id = m_nextId++;
    entry.al = al;
    entry.raised = al > wanted;
    entry.cce[d] = cce;
    entry.charged = charge;
    Occupy(d, cce, al, entry.id);
    if (uplink)
    {
        m_ulCce += al;
        m_ulCredit -= charge;
    }
    m_entries.push_back(entry);
    Grant grant;
    grant.id = entry.id;
    grant.al = al;
    return grant;
}

void
NrPdcch::Release(const Grant& grant, bool count)
{
    if (!grant)
    {
        return;
    }
    const auto it = std::find_if(m_entries.begin(), m_entries.end(), [&](const Entry& e) {
        return e.id == grant.id;
    });
    if (it == m_entries.end())
    {
        return; // of a slot that is over
    }
    Vacate(m_slotSymbols, it->cce[m_slotSymbols], it->al);
    if (IsUplink(it->purpose))
    {
        m_ulCce -= it->al;
        m_ulCredit += it->charged;
    }
    if (m_counting && count)
    {
        ++m_stats.unusedReservations;
    }
    m_entries.erase(it);
}

void
NrPdcch::Commit(const Grant& grant, Purpose purpose, const std::shared_ptr<DciInfoElementTdma>& dci)
{
    NS_ASSERT(grant);
    const auto it = std::find_if(m_entries.begin(), m_entries.end(), [&](const Entry& e) {
        return e.id == grant.id;
    });
    NS_ABORT_MSG_IF(it == m_entries.end(), "NrPdcch: a DCI for a PDCCH candidate of another slot");
    it->committed = true;
    it->purpose = purpose;
    it->dci = dci;
    if (dci)
    {
        dci->m_pdcchAl = it->al;
        if (m_fixed)
        {
            dci->m_pdcchCce = static_cast<uint16_t>(it->cce[m_slotSymbols]);
            dci->m_pdcchSymbols = m_slotSymbols;
        }
    }
}

void
NrPdcch::DoFixSymbols()
{
    if (m_fixed)
    {
        return;
    }
    m_fixed = true;
    // Dynamic: the fewest symbols whose CORESET holds every DCI of the slot. The DCIs are placed
    // again, in the order they were asked for, in the CORESET of 1 and then of 2 symbols; if
    // neither holds them all they stay where they are, in the CORESET of 3.
    const uint8_t top = m_slotSymbols;
    for (uint8_t d = GetMinSymbols(); d < top && m_hasCarrier; ++d)
    {
        m_owner[d].assign(GetNumCce(d), -1);
        bool all = true;
        for (auto& e : m_entries)
        {
            const int32_t cce = FindFree(e.rnti, IsCommon(e.purpose), e.al, d);
            if (cce < 0)
            {
                all = false;
                break;
            }
            e.cce[d] = cce;
            Occupy(d, cce, e.al, e.id);
        }
        if (all)
        {
            m_slotSymbols = d;
            break;
        }
    }
    for (auto& e : m_entries)
    {
        if (e.dci)
        {
            e.dci->m_pdcchCce = static_cast<uint16_t>(e.cce[m_slotSymbols]);
            e.dci->m_pdcchSymbols = m_slotSymbols;
        }
    }
}

uint8_t
NrPdcch::FixSymbols()
{
    NewSlotIfNeeded();
    DoFixSymbols();
    return m_slotSymbols;
}

void
NrPdcch::FinishSlot()
{
    if (m_slotNs < 0)
    {
        return;
    }
    DoFixSymbols();
    if (m_counting && m_enabled && m_hasCarrier && (m_dlStarted || !m_entries.empty()))
    {
        uint32_t used = 0;
        for (const auto& e : m_entries)
        {
            if (!e.committed)
            {
                ++m_stats.unusedReservations;
                continue;
            }
            const int i = AlIndex(e.al);
            used += e.al;
            ++m_stats.dci[e.purpose];
            m_stats.cce[e.purpose] += e.al;
            ++m_stats.dciByAl[i];
            m_stats.cceByAl[i] += e.al;
            m_stats.aboveTarget += !e.meetsTarget;
            m_stats.raised += e.raised;
            m_stats.noReport += e.noReport;
        }
        ++m_stats.slots;
        ++m_stats.slotsBySymbols[used ? m_slotSymbols : 0];
        m_stats.cceAvailable += GetNumCce(m_slotSymbols);
        m_stats.cceUsed += used;
    }
    m_slotNs = -1;
    m_entries.clear();
}

void
NrPdcch::NoteReception(uint8_t al, bool missed)
{
    const int i = AlIndex(al);
    if (m_counting && i >= 0)
    {
        ++m_stats.rxByAl[i];
        m_stats.missedByAl[i] += missed;
    }
}

void
NrPdcch::NoteEvent(Event event, uint64_t n)
{
    if (m_counting)
    {
        m_stats.events[event] += n;
    }
}

void
NrPdcch::ResetStats()
{
    m_stats = Stats();
}

void
NrPdcch::SetCounting(bool on)
{
    if (!on)
    {
        // What has been asked for up to now still counts
        const int64_t now = Simulator::Now().GetNanoSeconds();
        if (m_slotNs >= 0 && m_slotNs != now)
        {
            FinishSlot();
        }
    }
    m_counting = on;
}

NrPdcch::Stats
NrPdcch::GetStats()
{
    FinishSlot();
    return m_stats;
}

std::string
NrPdcch::GetPurposeName(Purpose purpose)
{
    switch (purpose)
    {
    case SIB1:
        return "system information (SIB1)";
    case RAR:
        return "random access response";
    case DL_NEW:
        return "downlink, new data";
    case DL_RETX:
        return "downlink, retransmission";
    case DL_SPS_ACTIVATION:
        return "downlink SPS activation";
    case DL_SPS_RELEASE:
        return "downlink SPS release";
    case UL_NEW:
        return "uplink, new data";
    case UL_RETX:
        return "uplink, retransmission";
    case UL_CG_ACTIVATION:
        return "uplink CG activation";
    case UL_CG_RELEASE:
        return "uplink CG release";
    default:
        return "?";
    }
}

} // namespace ns3
