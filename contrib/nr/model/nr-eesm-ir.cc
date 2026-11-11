// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-eesm-ir.h"

#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrEesmIr");
NS_OBJECT_ENSURE_REGISTERED(NrEesmIr);

NrEesmIr::NrEesmIr()
{
    NS_LOG_FUNCTION(this);
}

NrEesmIr::~NrEesmIr()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrEesmIr::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrEesmIr")
            .SetParent<NrEesmErrorModel>()
            .AddAttribute(
                "SoftBufferBitsPerTb",
                "Soft-buffer size of a transport block, in coded bits: the sum over its C code "
                "blocks of the circular-buffer size Ncb of TS 38.212 Sect. 5.4.2.1. The coded bits "
                "accumulated over HARQ-IR retransmissions are capped at this value, so the "
                "equivalent code rate does not fall below TBS / SoftBufferBitsPerTb. With "
                "limited-buffer rate matching Ncb = min(N, floor(TBS_LBRM / (C * R_LBRM))) with "
                "R_LBRM = 2/3, so once the LBRM bound applies this is about 1.5 * TBS_LBRM, "
                "whatever the TB size. 0 means unlimited.",
                UintegerValue(0),
                MakeUintegerAccessor(&NrEesmIr::m_softBufferBitsPerTb),
                MakeUintegerChecker<uint32_t>())
            .AddAttribute("CircularBuffer",
                          "Combined coded bits are bounded by the LDPC circular buffer of TS 38.212 5.4.2.1 (C x Ncb, Ncb = N, no LBRM); beyond it they repeat buffered bits.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrEesmIr::m_circularBuffer),
                          MakeBooleanChecker())
            .AddAttribute("ChaseFloor",
                          "A combined block's error rate is not above that of chase combining the same transmissions.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrEesmIr::m_chaseFloor),
                          MakeBooleanChecker())
            .AddAttribute("RedundancyVersions",
                          "Each transmission and repetition writes its coded bits from the k0 of its RV (0,2,3,1 per TS 38.214; TS 38.212 Table 5.4.2.1-2); overlaps count as repeated bits, not new ones.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrEesmIr::m_redundancyVersions),
                          MakeBooleanChecker());
    return tid;
}

double
NrEesmIr::ComputeSINR(const SpectrumValue& sinr,
                      const std::vector<int>& map,
                      uint8_t mcs,
                      uint32_t sizeBit,
                      const NrErrorModel::NrErrorModelHistory& sinrHistory) const
{
    NS_LOG_FUNCTION(this);
    // HARQ INCREMENTAL REDUNDANCY: update SINReff and ECR after retx, assuming
    // no repetition of coded bits.

    // compute equivalent effective code rate after retransmissions and total map size
    uint32_t codeBitsSum = 0;
    uint32_t infoBits = DynamicCast<NrEesmErrorModelOutput>(sinrHistory.front())
                            ->m_infoBits; // information bits of the first TB
    double mapSumSize = 0.0;

    for (const Ptr<NrErrorModelOutput>& output : sinrHistory)
    {
        Ptr<NrEesmErrorModelOutput> sinrHistorytemp = DynamicCast<NrEesmErrorModelOutput>(output);
        NS_ASSERT(sinrHistorytemp != nullptr);

        NS_LOG_DEBUG(" Exponential SINR sum " << sinrHistorytemp->m_sinrExp << " codeBits "
                                              << sinrHistorytemp->m_codeBits
                                              << " infoBits: " << sinrHistorytemp->m_infoBits);

        codeBitsSum += sinrHistorytemp->m_codeBits;
        mapSumSize += sinrHistorytemp->m_map.size();
    }
    mapSumSize += map.size();
    codeBitsSum += sizeBit / GetMcsEcrTable()->at(mcs);

    // Coded bits beyond the soft buffer repeat bits already buffered (TS 38.212 Sect. 5.4.2.1):
    // they do not lower the equivalent code rate, but the receiver adds their soft values to
    // the buffered ones, so they raise the SINR per buffered bit by the ratio of the coded bits
    // received to the buffer (chase combining of the repeated part). A buffer smaller than one
    // transmission cannot raise the equivalent MCS above the transmitted one: GetMcsEq only
    // searches below it.
    const uint32_t buffer = SoftBufferBits(infoBits, mcs);
    double repeatGain = 1.0;
    if (buffer > 0 && m_redundancyVersions)
    {
        // Where each transmission writes into the circular buffer of a code block (TS 38.212
        // 5.4.2.1): E = its coded bits per code block, from k0 of its redundancy version
        // (Table 5.4.2.1-2), wrapping round Ncb. The bits of the buffer that some transmission
        // covered are the code the decoder has; the rest of what came repeats covered bits.
        const GraphType bg = GetBaseGraphType(infoBits, mcs);
        const auto [k, c] = CodeBlockSegmentation(infoBits + 24, bg);
        const uint32_t zc = bg == FIRST ? k / 22 : k / 10;
        const uint32_t n = (bg == FIRST ? 66 : 50) * zc;
        const uint64_t ncb = std::max<uint64_t>(1, buffer / std::max<uint32_t>(c, 1));
        static const uint32_t k0Bg1[4] = {0, 17, 33, 56};
        static const uint32_t k0Bg2[4] = {0, 13, 25, 43};
        std::vector<std::pair<uint64_t, uint64_t>> covered; // [from, to) within [0, ncb)
        bool all = false;
        uint64_t total = 0;   // coded bits of all the transmissions
        uint64_t totalCb = 0; // the same per code block
        auto place = [&](uint32_t codeBits, uint8_t rvId) {
            total += codeBits;
            const uint64_t e = codeBits / std::max<uint32_t>(c, 1);
            totalCb += e;
            if (e >= ncb)
            {
                all = true;
                return;
            }
            const uint32_t num = (bg == FIRST ? k0Bg1 : k0Bg2)[rvId & 3];
            const uint64_t k0 = (num * ncb / n) * zc; // floor(num Ncb / (N_ref)) Zc
            const uint64_t from = k0 % ncb;
            const uint64_t to = from + e;
            if (to <= ncb)
            {
                covered.emplace_back(from, to);
            }
            else
            {
                covered.emplace_back(from, ncb);
                covered.emplace_back(0, to - ncb);
            }
        };
        for (const Ptr<NrErrorModelOutput>& output : sinrHistory)
        {
            const auto tx = DynamicCast<NrEesmErrorModelOutput>(output);
            place(tx->m_codeBits, tx->m_rvId);
        }
        place(static_cast<uint32_t>(sizeBit / GetMcsEcrTable()->at(mcs)), m_currentRvId);
        uint64_t unique = ncb;
        if (!all)
        {
            std::sort(covered.begin(), covered.end());
            unique = 0;
            uint64_t end = 0;
            for (const auto& [from, to] : covered)
            {
                if (to > end)
                {
                    unique += to - std::max(from, end);
                    end = to;
                }
            }
        }
        unique = std::max<uint64_t>(unique, 1);
        if (totalCb > unique)
        {
            repeatGain = static_cast<double>(totalCb) / unique;
            codeBitsSum = static_cast<uint32_t>(unique * std::max<uint32_t>(c, 1));
        }
        else
        {
            codeBitsSum = static_cast<uint32_t>(total);
        }
        NS_LOG_INFO(" RV " << +m_currentRvId << ": " << unique << " of Ncb " << ncb
                           << " covered per code block, coded bits " << total);
    }
    else if (buffer > 0 && codeBitsSum > buffer)
    {
        repeatGain = static_cast<double>(codeBitsSum) / buffer;
        codeBitsSum = buffer;
    }

    // (a block of no information bits, the smallest MCS on a few REs: the code rate of its MCS
    // over the transmissions)
    const_cast<NrEesmIr*>(this)->m_Reff =
        codeBitsSum > 0 ? infoBits / static_cast<double>(codeBitsSum)
                        : GetMcsEcrTable()->at(mcs) / static_cast<double>(sinrHistory.size() + 1);

    NS_LOG_INFO(" Reff " << m_Reff << " HARQ history (previous) " << sinrHistory.size());

    // compute effective SINR with expSINR_previousTx and mapSumSize
    double expSINR_previousTx = DynamicCast<NrEesmErrorModelOutput>(sinrHistory.back())->m_sinrExp;
    double sinrEff = SinrEff(sinr, map, mcs, expSINR_previousTx, mapSumSize) * repeatGain;
    if (repeatGain > 1.0)
    {
        NS_LOG_INFO(" coded bits beyond the soft buffer of " << buffer << ": SINR x " << repeatGain);
    }

    // The tables have no curve below the lowest code rate of the block's modulation, where
    // GetMcsEq() stops: a block at MCS 0 would gain nothing from its retransmissions (or PUSCH
    // repetitions), however many. The coded bits beyond that rate still carry energy: count
    // them as repetitions of the code of that lowest MCS, which scales its SINR by its code
    // rate over Reff. That is the gain of chase combining, which incremental redundancy at
    // these low code rates matches or exceeds.
    const uint8_t modOrder = GetMcsMTable()->at(mcs);
    double lowestEcr = GetMcsEcrTable()->at(mcs);
    for (uint8_t i = 0; i < mcs; ++i)
    {
        if (GetMcsMTable()->at(i) == modOrder)
        {
            lowestEcr = std::min(lowestEcr, GetMcsEcrTable()->at(i));
        }
    }
    if (m_Reff > 0.0 && m_Reff < lowestEcr)
    {
        sinrEff *= lowestEcr / m_Reff;
        NS_LOG_INFO(" Reff below the lowest ECR " << lowestEcr << ": SINR x " << lowestEcr / m_Reff
                                                  << " = " << sinrEff);
    }
    return sinrEff;
}

uint32_t
NrEesmIr::SoftBufferBits(uint32_t infoBits, uint8_t mcs) const
{
    uint32_t buffer = 0;
    if (m_circularBuffer)
    {
        // TS 38.212 5.4.2.1: per code block the circular buffer holds the N = 66 Zc (base graph
        // 1) or 50 Zc (base graph 2) coded bits of the LDPC codeword; without limited-buffer
        // rate matching Ncb = N. The base graph, the code blocks and Zc as the BLER mapping
        // takes them (GetBaseGraphType(), CodeBlockSegmentation(), 24-bit TB CRC).
        const GraphType bg = GetBaseGraphType(infoBits, mcs);
        const auto [k, c] = CodeBlockSegmentation(infoBits + 24, bg);
        const uint32_t zc = bg == FIRST ? k / 22 : k / 10;
        buffer = c * (bg == FIRST ? 66 : 50) * zc;
    }
    if (m_softBufferBitsPerTb > 0)
    {
        buffer = buffer > 0 ? std::min(buffer, m_softBufferBitsPerTb) : m_softBufferBitsPerTb;
    }
    return buffer;
}

bool
NrEesmIr::ChaseFloor() const
{
    return m_chaseFloor;
}

double
NrEesmIr::GetMcsEq(uint8_t mcsTx) const
{
    NS_LOG_FUNCTION(this);
    // PHY abstraction for HARQ-IR retx -> get closest ECR to Reff from the
    // available ones that belong to the same modulation order

    uint8_t mcs_eq = mcsTx;

    uint8_t ModOrder = GetMcsMTable()->at(mcsTx);

    NS_LOG_INFO(" Modulation order: " << +ModOrder);
    NS_LOG_INFO(" Reff: " << m_Reff);

    for (uint8_t mcsindex = (mcsTx - 1); mcsindex != 255; mcsindex--)
    // search from MCS=mcs-1 to MCS=0. end at 255 to account for wrap around of uint
    {
        if ((GetMcsMTable()->at(mcsindex) == ModOrder) && (GetMcsEcrTable()->at(mcsindex) > m_Reff))
        {
            mcs_eq--;
        }
    }

    return mcs_eq;
}

} // namespace ns3
