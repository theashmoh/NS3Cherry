// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
// Copyright (c) 2019 Interdigital <kevin.wanuga@interdigital.com> tables
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-eesm-error-model.h"

#include "fast-exp.h"
#include "nr-phy-mac-common.h"

#include "ns3/enum.h"
#include "ns3/log.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrEesmErrorModel");
NS_OBJECT_ENSURE_REGISTERED(NrEesmErrorModel);

/**
 * @brief Table of Lifting Sizes for LDPC
 */
static const std::vector<uint16_t> LiftingSizeTableBG = {
    2,  3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  18,  20,
    22, 24,  26,  28,  30,  32,  36,  40,  44,  48,  52,  56,  60,  64,  72,  80,  88,
    96, 104, 112, 120, 128, 144, 160, 176, 192, 208, 224, 240, 256, 288, 320, 352, 384};

std::vector<std::string> NrEesmErrorModel::m_bgTypeName = {"BG1", "BG2"};

NrEesmErrorModel::NrEesmErrorModel()
    : NrErrorModel()
{
    NS_LOG_FUNCTION(this);
}

NrEesmErrorModel::~NrEesmErrorModel()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrEesmErrorModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrEesmErrorModel").SetParent<NrErrorModel>();
    return tid;
}

double
NrEesmErrorModel::SinrEff(const SpectrumValue& sinr,
                          const std::vector<int>& map,
                          uint8_t mcs,
                          double a,
                          double b) const
{
    // it follows: SINReff = - beta * ln [1/b * (sum (exp (-sinr/beta)) + a)]
    // for HARQ-IR: b = sum (map.size()), a = sum_j(sum_n (exp (-sinr/beta))) (for previous retx,
    // till j=q-1) for HARQ-CC: b = map.size(), a = 0.0 (SINRs are already combined in sinr input)

    double sinrExpSum = SinrExp(sinr, map, mcs);
    double beta = GetBetaTable()->at(mcs);
    double SINR = -beta * log((a + sinrExpSum) / b);
    SINR = std::max(SINR, 0.0);

    NS_LOG_INFO(" Effective SINR = " << SINR);

    return SINR;
}

double
NrEesmErrorModel::SinrExp(const SpectrumValue& sinr, const std::vector<int>& map, uint8_t mcs) const
{
    // it returns sum_n (exp (-SINR/beta))
    NS_LOG_FUNCTION(sinr << &map << (uint8_t)mcs);
    NS_ABORT_MSG_IF(map.empty(),
                    " Error: number of allocated RBs cannot be 0 - EESM method - SinrEff function");

    double SINRexp = 0.0;
    double SINRsum = 0.0;
    double beta = GetBetaTable()->at(mcs);
    for (int i : map)
    {
        double sinrLin = sinr[i];
        SINRexp = exp21d(-sinrLin / beta);
        SINRsum += SINRexp;
    }
    return SINRsum;
}

double
NrEesmErrorModel::ChaseCombinedSinr(const SpectrumValue& sinr,
                                    const std::vector<int>& map,
                                    uint8_t mcs,
                                    const NrErrorModel::NrErrorModelHistory& sinrHistory) const
{
    NS_LOG_FUNCTION(this);
    // The SINRs of the transmissions summed RB by RB (chase combining: the same coded bits again)

    // make a vector of history that contains the last tx (but without modifying
    // sinrHistory, as it will be modified by the caller when it will be the time)
    Ptr<NrEesmErrorModelOutput> last = Create<NrEesmErrorModelOutput>(0.0);
    last->m_map = map;
    last->m_sinr = sinr;

    NrErrorModel::NrErrorModelHistory total = sinrHistory;
    total.emplace_back(last);

    // evaluate SINR_eff over "total", as per Chase Combining

    NS_ASSERT(sinr.GetSpectrumModel()->GetNumBands() == sinr.GetValuesN());

    SpectrumValue sinr_sum(sinr.GetSpectrumModel());
    auto historySize = static_cast<uint32_t>(total.size());
    uint32_t maxRBUsed = 0;
    for (uint32_t i = 0; i < historySize; ++i)
    {
        Ptr<NrEesmErrorModelOutput> output = DynamicCast<NrEesmErrorModelOutput>(total.at(i));
        maxRBUsed = std::max(maxRBUsed, static_cast<uint32_t>(output->m_map.size()));
    }

    std::vector<int> map_sum;
    map_sum.reserve(maxRBUsed);

    for (uint32_t i = 0; i < maxRBUsed; ++i)
    {
        sinr_sum[i] = 0;
        map_sum.push_back(static_cast<int>(i));
    }

    /* combine at the bit level. Example:
     * SINR{1}=[0 0 10 20 10 0 0];
     * SINR{2}=[1 2 1 2 1 0 3];
     * SINR{3}=[5 0 0 0 0 0 0];
     *
     * map{1}=[2 3 4];
     * map{2}=[0 1 2 3 4 6];
     * map{3}=[0];
     *
     * MAP_SUM = [0 1 2 3 4 5]
     * SINR_SUM = [16 27 16 17 26 18]
     *
     * (the value at SINR_SUM[0] is SINR{1}[2] + SINR{2}[0] + SINR{3}[0])
     */
    for (uint32_t i = 0; i < historySize; ++i)
    {
        Ptr<NrEesmErrorModelOutput> output = DynamicCast<NrEesmErrorModelOutput>(total.at(i));
        uint32_t size = output->m_map.size();
        for (uint32_t j = 0; j < maxRBUsed; ++j)
        {
            sinr_sum[j] += output->m_sinr[output->m_map[j % size]];
        }
    }

    NS_LOG_INFO("\tHISTORY:");
    for (const auto& element : total)
    {
        Ptr<NrEesmErrorModelOutput> output = DynamicCast<NrEesmErrorModelOutput>(element);
        NS_LOG_INFO("\tMAP:" << PrintMap(output->m_map));
        NS_LOG_INFO("\tSINR: " << output->m_sinr);
    }

    NS_LOG_INFO("MAP_SUM: " << PrintMap(map_sum));
    NS_LOG_INFO("SINR_SUM: " << sinr_sum);

    // compute effective SINR with the sinr_sum vector and map_sum RB map
    return SinrEff(sinr_sum, map_sum, mcs, 0.0, map_sum.size());
}

const std::vector<double>&
NrEesmErrorModel::GetSinrDbVectorFromSimulatedValues(NrEesmErrorModel::GraphType graphType,
                                                     uint8_t mcs,
                                                     uint32_t cbSizeIndex) const
{
    return std::get<0>(GetSimulatedBlerFromSINR()->at(graphType).at(mcs).at(cbSizeIndex));
}

const std::vector<double>&
NrEesmErrorModel::GetBLERVectorFromSimulatedValues(NrEesmErrorModel::GraphType graphType,
                                                   uint8_t mcs,
                                                   uint32_t cbSizeIndex) const
{
    return std::get<1>(GetSimulatedBlerFromSINR()->at(graphType).at(mcs).at(cbSizeIndex));
}

double
NrEesmErrorModel::MappingSinrBler(double sinr, uint8_t mcs, uint32_t cbSizeBit)
{
    NS_LOG_FUNCTION(sinr << (uint8_t)mcs << (uint32_t)cbSizeBit);
    NS_ABORT_MSG_IF(mcs > GetMaxMcs(),
                    "MCS out of range [0..27/28]: " << static_cast<uint8_t>(mcs));

    // use cbSize to obtain the index of CBSIZE in the map, jointly with mcs and sinr. take the
    // lowest CBSIZE simulated including this CB for removing CB size quatization
    // errors. sinr is also lower-bounded.
    double bler = 0.0;
    double sinr_db = 10 * log10(sinr);
    GraphType bg_type = GetBaseGraphType(cbSizeBit, mcs);

    // Get the index of CBSIZE in the map
    NS_LOG_INFO("For sinr " << sinr << " and mcs " << +mcs << " CbSizebit " << cbSizeBit
                            << " we got bg type " << m_bgTypeName[bg_type]);
    const auto& cbMap = GetSimulatedBlerFromSINR()->at(bg_type).at(mcs);
    auto cbIt = cbMap.upper_bound(cbSizeBit);

    if (cbIt != cbMap.begin())
    {
        cbIt--;
    }

    if (sinr_db < GetSinrDbVectorFromSimulatedValues(bg_type, mcs, cbIt->first).front())
    {
        bler = 1.0;
    }
    else if (sinr_db > GetSinrDbVectorFromSimulatedValues(bg_type, mcs, cbIt->first).back())
    {
        bler = 0.0;
    }
    else
    {
        // Get the index of SINR in the vector
        auto sinrIt =
            std::upper_bound(GetSinrDbVectorFromSimulatedValues(bg_type, mcs, cbIt->first).begin(),
                             GetSinrDbVectorFromSimulatedValues(bg_type, mcs, cbIt->first).end(),
                             sinr_db);

        if (sinrIt != GetSinrDbVectorFromSimulatedValues(bg_type, mcs, cbIt->first).begin())
        {
            sinrIt--;
        }

        auto sinr_index =
            std::distance(GetSinrDbVectorFromSimulatedValues(bg_type, mcs, cbIt->first).begin(),
                          sinrIt);
        bler = GetBLERVectorFromSimulatedValues(bg_type, mcs, cbIt->first).at(sinr_index);
    }

    NS_LOG_LOGIC("SINR effective: " << sinr << " BLER:" << bler);
    return bler;
}

NrEesmErrorModel::GraphType
NrEesmErrorModel::GetBaseGraphType(uint32_t tbSizeBit, uint8_t mcs) const
{
    double ecr = GetMcsEcrTable()->at(mcs);

    GraphType bg_type = FIRST;
    if (tbSizeBit <= 292 || ecr <= 0.25 || (tbSizeBit <= 3824 && ecr <= 0.67))
    {
        bg_type = SECOND;
    }
    return bg_type;
}

std::pair<uint32_t, uint32_t>
NrEesmErrorModel::CodeBlockSegmentation(uint32_t B, GraphType bg_type) const
{
    uint16_t Kcb;
    uint8_t Kb;
    if (bg_type == FIRST)
    {
        Kcb = 8448; // max size of a codeblock (including CRC) for BG1
        Kb = 22;
    }
    else
    {
        NS_ASSERT(bg_type == SECOND);
        Kcb = 3840; // max size of a codeblock (including CRC) for BG2
        if (B >= 640)
        {
            Kb = 10;
        }
        else if (B >= 560)
        {
            Kb = 9;
        }
        else if (B >= 192)
        {
            Kb = 8;
        }
        else
        {
            Kb = 6;
        }
    }

    uint32_t L = 0;
    uint32_t C = 0; // no. of codeblocks
    uint32_t B1 = 0;

    if (B <= Kcb)
    {
        // only one codeblock
        L = 0;
        C = 1;
        B1 = B;
    }
    else
    {
        L = 24;
        C = ceil((double)B / ((double)(Kcb - L)));
        B1 = B + C * L;
    }

    // Zc = minimum Z in all sets of lifting sizes table such that Kb * Z >= K1
    uint32_t K1 = B1 / C;

    // returns an iterator pointing to the first element in the range [first,last)
    // which compares greater than the third parameter.
    auto ZcIt = std::upper_bound(LiftingSizeTableBG.begin(),
                                 --LiftingSizeTableBG.end(),
                                 (static_cast<double>(K1) / Kb) + 0.001);
    uint16_t Zc = *ZcIt;
    uint32_t K;

    if (bg_type == FIRST)
    {
        K = Zc * 22; // no. of bits in each code block
    }
    else // bg_type==2
    {
        NS_ASSERT(bg_type == SECOND);
        K = Zc * 10; // no. of bits in each code block
    }

    NS_LOG_INFO("EESMErrorModel: TBS of " << B << " needs of " << B1 << " bits distributed in " << C
                                          << " CBs of " << K << " bits");
    return std::make_pair(K, C);
}

Ptr<NrErrorModelOutput>
NrEesmErrorModel::GetTbDecodificationStats(const SpectrumValue& sinr,
                                           const std::vector<int>& map,
                                           uint32_t size,
                                           uint8_t mcs,
                                           const NrErrorModelHistory& sinrHistory)
{
    return GetTbBitDecodificationStats(sinr, map, size * 8, mcs, sinrHistory);
}

std::string
NrEesmErrorModel::PrintMap(const std::vector<int>& map) const
{
    std::stringstream ss;

    for (const auto& v : map)
    {
        ss << v << ", ";
    }

    return ss.str();
}

Ptr<NrErrorModelOutput>
NrEesmErrorModel::GetTbBitDecodificationStats(const SpectrumValue& sinr,
                                              const std::vector<int>& map,
                                              uint32_t sizeBit,
                                              uint8_t mcs,
                                              const NrErrorModelHistory& sinrHistory)
{
    NS_LOG_FUNCTION(this);
    NS_ABORT_IF(mcs > GetMaxMcs());

    double tbSinr = SinrEff(sinr, map, mcs, 0, map.size()); // effective SINR for this tx
    double SINR = tbSinr;
    double sinrExpSum = SinrExp(sinr, map, mcs); // exponential sum of SINRs for this tx

    NS_LOG_DEBUG(" mcs " << +mcs << " TBSize in bit " << sizeBit << " history elements: "
                         << sinrHistory.size() << " SINR of the tx: " << tbSinr << std::endl
                         << "MAP: " << PrintMap(map) << std::endl
                         << "SINR: " << sinr);

    // the redundancy version of this transmission (see m_currentRvId)
    {
        static const uint8_t rvSequence[4] = {0, 2, 3, 1};
        m_currentRvCounter = m_nextRvCounter;
        uint32_t position = static_cast<uint32_t>(sinrHistory.size());
        if (m_currentRvCounter != 255)
        {
            uint32_t sameTx = 0; // earlier repetitions of this transmission
            for (auto it = sinrHistory.rbegin(); it != sinrHistory.rend(); ++it)
            {
                const auto out = DynamicCast<NrEesmErrorModelOutput>(*it);
                if (out == nullptr || out->m_rvCounter != m_currentRvCounter)
                {
                    break;
                }
                ++sameTx;
            }
            position = m_currentRvCounter + sameTx;
        }
        m_currentRvId = rvSequence[position % 4];
    }

    if (!sinrHistory.empty())
    {
        SINR = ComputeSINR(sinr, map, mcs, sizeBit, sinrHistory);
    }

    NS_LOG_DEBUG(" SINR after processing all retx (if any): " << SINR << " SINR last tx" << tbSinr);

    // LDPC base graph type selection (1 or 2), as per TS 38.212, using the payload (A)
    GraphType bg_type = GetBaseGraphType(sizeBit, mcs);
    NS_LOG_INFO("BG type selection: " << bg_type);

    // code block segmentation, as per TS 38.212, using payload + TB CRC attachment (B)
    uint32_t B = sizeBit + 24; // input to code block segmentation, in bits
    std::pair<uint32_t, uint32_t> cbSeg = CodeBlockSegmentation(B, bg_type);
    uint32_t K = cbSeg.first;
    uint32_t C = cbSeg.second;
    NS_LOG_INFO("EESMErrorModel: TBS of " << B << " bits distributed in " << C << " CBs of " << K
                                          << " bits");

    uint8_t mcs_eq = mcs;
    if ((!sinrHistory.empty()) && (mcs > 0))
    {
        mcs_eq = GetMcsEq(mcs);
    }

    NS_LOG_INFO(" MCS of tx " << +mcs << " Equivalent MCS for PHY abstraction (just for HARQ-IR) "
                              << +mcs_eq);

    double errorRate = 1.0;
    if (C != 1)
    {
        double cbler = MappingSinrBler(SINR, mcs_eq, K);
        errorRate = 1.0 - pow(1.0 - cbler, C);
    }
    else
    {
        errorRate = MappingSinrBler(SINR, mcs_eq, K);
    }

    // A receiver that keeps every coded bit of the earlier transmissions does at least as well
    // as chase combining of the same energy: where the incremental-redundancy abstraction
    // (an averaged SINR on the curve of a lower code rate) gives a higher error rate, the
    // error rate of the energy combined at the transmitted MCS stands (ChaseFloor())
    if (!sinrHistory.empty() && errorRate > 0.0 && ChaseFloor())
    {
        const double sinrCc = ChaseCombinedSinr(sinr, map, mcs, sinrHistory);
        const double cblerCc = MappingSinrBler(sinrCc, mcs, K);
        const double errorRateCc = C != 1 ? 1.0 - pow(1.0 - cblerCc, C) : cblerCc;
        if (errorRateCc < errorRate)
        {
            NS_LOG_INFO(" chase combining " << errorRateCc << " below incremental redundancy "
                                            << errorRate << ": SINR " << sinrCc);
            errorRate = errorRateCc;
            SINR = sinrCc;
        }
    }

    NS_LOG_DEBUG("Calculated Error rate " << errorRate);
    NS_ASSERT(GetMcsEcrTable() != nullptr);

    Ptr<NrEesmErrorModelOutput> ret = Create<NrEesmErrorModelOutput>(errorRate);
    ret->m_sinrEff = SINR;
    ret->m_sinr = sinr;
    ret->m_map = map;
    if (sinrHistory.empty())
    {
        ret->m_sinrExp = sinrExpSum; // it is first tx!
    }
    else
    {
        double m_sinrExpPrevious =
            DynamicCast<NrEesmErrorModelOutput>(sinrHistory.back())->m_sinrExp;
        ret->m_sinrExp = m_sinrExpPrevious + sinrExpSum; // it sums over previous tx (recursively)
    }
    ret->m_infoBits = sizeBit;
    ret->m_codeBits = sizeBit / GetMcsEcrTable()->at(mcs);
    ret->m_rvCounter = m_currentRvCounter;
    ret->m_rvId = m_currentRvId;

    return ret;
}

double
NrEesmErrorModel::GetSpectralEfficiencyForCqi(uint8_t cqi)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(GetSpectralEfficiencyForCqi() != nullptr);
    NS_ABORT_MSG_UNLESS(cqi >= 0 && cqi <= 15, "CQI must be in [0..15] = " << cqi);

    return GetSpectralEfficiencyForCqi()->at(cqi);
}

double
NrEesmErrorModel::GetSpectralEfficiencyForMcs(uint8_t mcs) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(GetSpectralEfficiencyForMcs() != nullptr);
    NS_ABORT_IF(mcs > GetMaxMcs());

    return GetSpectralEfficiencyForMcs()->at(mcs);
}

uint32_t
NrEesmErrorModel::GetPayloadSize(uint32_t usefulSc,
                                 uint8_t mcs,
                                 uint8_t rank,
                                 uint32_t rbNum,
                                 [[maybe_unused]] Mode mode) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(GetMcsEcrTable() != nullptr);
    const uint32_t rscElement = usefulSc * rbNum;
    double Rcode = GetMcsEcrTable()->at(mcs);
    uint8_t Qm = GetMcsMTable()->at(mcs);

    const double spectralEfficiency = rscElement * Qm * Rcode * rank;

    return static_cast<uint32_t>(std::floor(spectralEfficiency / 8));
}

uint32_t
NrEesmErrorModel::GetMaxCbSize(uint32_t tbSize, uint8_t mcs) const
{
    GraphType bgType = GetBaseGraphType(tbSize * 8, mcs);
    if (bgType == FIRST)
    {
        return LiftingSizeTableBG.back() * 22 / 8; // return CBsize in bytes
    }
    NS_ASSERT(bgType == SECOND);
    return LiftingSizeTableBG.back() * 10 / 8; // return CBsize in bytes
}

uint8_t
NrEesmErrorModel::GetMaxMcs() const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(GetMcsEcrTable() != nullptr);
    NS_LOG_INFO(" Max MCS: " << +(GetMcsEcrTable()->size() - 1));
    return static_cast<uint8_t>(GetMcsEcrTable()->size() - 1);
}

} // namespace ns3
