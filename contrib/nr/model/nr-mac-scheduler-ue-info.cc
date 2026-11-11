// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-ue-info.h"

#include "nr-mac-short-bsr-ce.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerUeInfo");

NrMacSchedulerUeInfo::NrMacSchedulerUeInfo(uint16_t rnti, BeamId beamId, const GetRbPerRbgFn& fn)
    : m_rnti(rnti),
      m_beamId(beamId),
      m_getNumRbPerRbg(fn)
{
}

NrMacSchedulerUeInfo::~NrMacSchedulerUeInfo()
{
}

std::vector<uint16_t>&
NrMacSchedulerUeInfo::GetDlRBG(const UePtr& ue)
{
    return ue->m_dlRBG;
}

std::vector<uint16_t>&
NrMacSchedulerUeInfo::GetUlRBG(const UePtr& ue)
{
    return ue->m_ulRBG;
}

std::vector<uint8_t>&
NrMacSchedulerUeInfo::GetDlSym(const UePtr& ue)
{
    return ue->m_dlSym;
}

std::vector<uint8_t>&
NrMacSchedulerUeInfo::GetUlSym(const UePtr& ue)
{
    return ue->m_ulSym;
}

uint8_t&
NrMacSchedulerUeInfo::GetDlMcs(const UePtr& ue)
{
    return ue->m_dlMcs;
}

template <typename T>
uint8_t
ComputeMcs(const NrMacSchedulerUeInfo* ueInfo,
           T NrMacSchedulerUeInfo::SbMcsInfo::* field,
           std::function<uint8_t(double)> postProcessing)
{
    // Compute average field of allocated RBGs
    const auto sum = std::transform_reduce(
        ueInfo->m_dlRBG.begin(),
        ueInfo->m_dlRBG.end(),
        0.0,
        [](auto a, auto b) { return a + b; },
        [ueInfo, field](auto a) {
            return ueInfo->m_dlSbMcsInfo.at(ueInfo->m_rbgToSb.at(a)).*field;
        });
    const auto avg = sum / ueInfo->m_dlRBG.size();
    return postProcessing(avg);
}

uint8_t
NrMacSchedulerUeInfo::GetDlMcs() const
{
    const auto csiMcs = GetDlMcsFromCsi();

    // Outer-loop link adaptation. The CSI-derived MCS above is an open-loop estimate: it is
    // what the UE predicted, on the CSI reference resource, would hit the CQI table's BLER
    // target (TS 38.214 Sect. 5.2.2.1). Any systematic bias in that prediction never corrects
    // itself, because nothing feeds the *measured* error rate back here. m_dlOlla is that
    // missing feedback, driven from first-transmission HARQ outcomes in
    // NrMacSchedulerNs3::ProcessHARQFeedbacks. 3GPP does not specify the loop, so it stays off
    // by default and this function then returns the CSI estimate unchanged.
    if (m_dlOllaMax == 0.0)
    {
        return csiMcs;
    }
    const auto maxMcs = static_cast<int>(m_dlAmc->GetMaxMcs());
    const auto corrected = static_cast<int>(std::lround(csiMcs + m_dlOlla));
    return static_cast<uint8_t>(std::clamp(corrected, 0, maxMcs));
}

bool
NrMacSchedulerUeInfo::IsPuschRepetitionValue(uint32_t k)
{
    return std::find(std::begin(kPuschRepetitionValues), std::end(kPuschRepetitionValues), k) !=
           std::end(kPuschRepetitionValues);
}

uint8_t
NrMacSchedulerUeInfo::PuschRepetitionsAtLeast(uint32_t k)
{
    for (const uint8_t value : kPuschRepetitionValues)
    {
        if (value >= k)
        {
            return value;
        }
    }
    return kPuschRepetitionValues[11];
}

NrMacSchedulerUeInfo::UlLink
NrMacSchedulerUeInfo::UlLinkFor(uint32_t rbs) const
{
    const auto measured = static_cast<uint32_t>(m_ulSinrMeas.size());
    if (!m_ulMcsPerRbs || measured == 0 || rbs == 0 || !m_ulModel ||
        (rbs == measured && m_puschRepetitions == 1))
    {
        // (a fixed number of repetitions makes the block more robust, at this MCS)
        return {m_ulMcs, m_ulCqi.m_wbCqi, std::max<uint8_t>(m_puschRepetitions, 1)};
    }
    auto it = m_ulMcsByRbs.find(rbs);
    if (it == m_ulMcsByRbs.end())
    {
        double mean = 0.0;
        const SpectrumValue sinr = UlSinrFor(rbs, &mean);
        // PUSCH repetition: the gNB combines the K transmissions of the block; K is the fewest
        // that bring K times the mean SINR to the target, the MCS the error model decodes then
        uint8_t reps = m_puschRepetitions;
        if (reps > 1 && m_ulWideRbs > 0 && rbs > m_ulNarrowRbs)
        {
            // a wide grant (UlWideGrantRbs): as many resource block slots as a narrow one, so
            // the repetitions are divided by how many times wider it is
            // (to a number TS 38.331 has: 9 / 2 = 4.5 is 7, not 5, the next above it)
            reps = PuschRepetitionsAtLeast((reps * m_ulNarrowRbs + rbs - 1) / rbs);
        }
        if (reps == 0)
        {
            reps = 32;
            for (const uint8_t k : kPuschRepetitionValues)
            {
                if (mean * k >= m_puschRepSinr)
                {
                    reps = k;
                    break;
                }
            }
        }
        uint8_t mcs = 0;
        uint8_t cqi = m_ulAmc->CreateCqiFeedbackSiso(sinr, mcs, reps);
        if (m_puschRepetitions == 0)
        {
            // ... or more, if the block of a slot at that MCS is under the 15 B a grant needs
            // (12 with the compact BSR; NrMacSchedulerOfdma::CreateUlDci() makes none for less,
            // and the UE sends no data in less): with fewer the UE would get no grant at all
            const uint32_t nprb = rbs * NrAmc::NR_AMC_NUM_SYMBOLS_DEFAULT;
            const uint32_t minBlock =
                std::max<uint32_t>(m_puschMinBlock, NrMacShortBsrCe::MinUlGrantBytes());
            for (const uint8_t k : kPuschRepetitionValues)
            {
                if (k <= reps || m_ulAmc->CalculateTbSize(mcs, 1, nprb) >= minBlock)
                {
                    continue;
                }
                reps = k;
                cqi = m_ulAmc->CreateCqiFeedbackSiso(sinr, mcs, reps);
            }
        }
        it = m_ulMcsByRbs.emplace(rbs, UlLink{mcs, cqi, reps}).first;
    }
    return it->second;
}

SpectrumValue
NrMacSchedulerUeInfo::UlSinrFor(uint32_t rbs, double* mean) const
{
    // The measured SINR of the RBs (in turn, for more RBs than were measured), times the share
    // of the UE's power each RB gets now against then
    const auto measured = static_cast<uint32_t>(m_ulSinrMeas.size());
    SpectrumValue sinr(m_ulModel);
    const double scale = static_cast<double>(measured) / rbs;
    const auto bands = std::min<uint32_t>(rbs, m_ulModel->GetNumBands());
    *mean = 0.0;
    for (uint32_t i = 0; i < bands; ++i)
    {
        sinr[i] = m_ulSinrMeas.at(i % measured) * scale;
        *mean += sinr[i] / bands;
    }
    return sinr;
}

uint8_t
NrMacSchedulerUeInfo::GetUlMcsForReps(uint32_t rbs, uint8_t reps, double scale) const
{
    if (!m_ulMcsPerRbs || m_ulSinrMeas.empty() || rbs == 0 || !m_ulModel)
    {
        return m_ulMcs;
    }
    if (scale != 1.0)
    {
        // with a margin: the MCS the error model decodes at that much less SINR (not cached)
        double mean = 0.0;
        SpectrumValue sinr = UlSinrFor(rbs, &mean);
        sinr *= scale;
        uint8_t mcs = 0;
        m_ulAmc->CreateCqiFeedbackSiso(sinr, mcs, std::max<uint8_t>(reps, 1));
        return mcs;
    }
    const uint32_t key = (rbs << 8) + reps;
    auto it = m_ulMcsByRbsReps.find(key);
    if (it == m_ulMcsByRbsReps.end())
    {
        double mean = 0.0;
        const SpectrumValue sinr = UlSinrFor(rbs, &mean);
        uint8_t mcs = 0;
        m_ulAmc->CreateCqiFeedbackSiso(sinr, mcs, std::max<uint8_t>(reps, 1));
        it = m_ulMcsByRbsReps.emplace(key, mcs).first;
    }
    return it->second;
}

double
NrMacSchedulerUeInfo::GetUlMeanSinrFor(uint32_t rbs) const
{
    if (m_ulSinrMeas.empty() || rbs == 0 || !m_ulModel)
    {
        return 0.0;
    }
    double mean = 0.0;
    UlSinrFor(rbs, &mean);
    return mean;
}

uint8_t
NrMacSchedulerUeInfo::GetUlRepetitionsFor(uint32_t rbs) const
{
    return UlLinkFor(rbs).reps;
}

uint8_t
NrMacSchedulerUeInfo::GetUlMcsFor(uint32_t rbs) const
{
    const int planned = UlLinkFor(rbs).mcs;
    int mcs = planned;
    if (m_ulOllaMax != 0.0)
    {
        mcs = std::clamp(static_cast<int>(std::lround(mcs + m_ulOlla)),
                         0,
                         static_cast<int>(m_ulAmc->GetMaxMcs()));
        if (mcs < planned && rbs > 0)
        {
            // The outer loop makes the block safer by a lower MCS, which also makes it smaller.
            // Not smaller than the least block of the plan (the 15 B a UE sends data in, or
            // PuschMinBlockBytes: a voice frame that does not fit one block takes two, and the
            // uplink has twice the load, with repetitions that is K more slots per frame).
            const uint32_t nprb = rbs * NrAmc::NR_AMC_NUM_SYMBOLS_DEFAULT;
            const uint32_t minBlock =
                std::max<uint32_t>(m_puschMinBlock, NrMacShortBsrCe::MinUlGrantBytes());
            while (mcs < planned && m_ulAmc->CalculateTbSize(static_cast<uint8_t>(mcs), 1, nprb) < minBlock)
            {
                ++mcs;
            }
        }
    }
    return static_cast<uint8_t>(std::clamp(mcs, 0, static_cast<int>(m_ulAmc->GetMaxMcs())));
}

uint32_t
NrMacSchedulerUeInfo::GetUlMaxRbs() const
{
    if (!m_ulMcsPerRbs || m_ulSinrMeas.empty() || !m_ulModel)
    {
        return UINT32_MAX;
    }
    if (m_ulMaxRbs == 0)
    {
        // The SINR of each RB falls as the RBs grow: the last count whose CQI is above 0
        const uint32_t bands = m_ulModel->GetNumBands();
        uint32_t most = 1;
        for (uint32_t rbs = 1; rbs <= bands; ++rbs)
        {
            if (UlLinkFor(rbs).cqi == 0)
            {
                break;
            }
            most = rbs;
        }
        m_ulMaxRbs = most;
    }
    return m_ulMaxRbs;
}

bool
NrMacSchedulerUeInfo::UlWideHalvesReps() const
{
    return m_ulWideRbs > m_ulNarrowRbs && m_puschRepetitions > 1;
}

bool
NrMacSchedulerUeInfo::UlWideOk() const
{
    if (!(m_ulWideRbs > 0 && m_ulMcsPerRbs && !m_ulSinrMeas.empty() && m_ulModel &&
          GetUlMeanSinrFor(1) >= m_ulWideSinr && GetUlMaxRbs() >= m_ulWideRbs))
    {
        return false;
    }
    if (m_puschRepetitions == 0)
    {
        // The repetitions follow the SINR of the grant the UE gets, and the wide grant has
        // 10 log10(W) dB less of it per RB. If that puts the wide grant under the repetition
        // target (UlWideSinrDb under PuschRepetitionSinrDb + 10 log10(W), or exactly at it:
        // 5 dB on one RB is 1.99 dB on two, under a 2 dB target), the wide grant takes more
        // slots than it saves. Give it only while its resource block slots, repetitions
        // included, are no more than the narrow grant's; otherwise the narrow grant and its
        // K carry the same bits for less. The decision is per grant, from the measurement
        // scaled to one RB, so it does not feed back on itself.
        const uint32_t narrow = std::max<uint32_t>(m_ulNarrowRbs, 1);
        const uint32_t wideSlots = UlLinkFor(m_ulWideRbs).reps * m_ulWideRbs;
        const uint32_t narrowSlots = UlLinkFor(narrow).reps * narrow;
        if (wideSlots > narrowSlots)
        {
            return false;
        }
    }
    return true;
}

uint32_t
NrMacSchedulerUeInfo::GetUlRbs() const
{
    std::vector<uint16_t> rbgs(m_ulRBG);
    std::sort(rbgs.begin(), rbgs.end());
    rbgs.erase(std::unique(rbgs.begin(), rbgs.end()), rbgs.end());
    return static_cast<uint32_t>(rbgs.size()) * GetNumRbPerRbg();
}

uint8_t
NrMacSchedulerUeInfo::GetDlMcsFromCsi() const
{
    // Return maximum allowed MCS according to Fronthaul control
    if (m_fhMaxMcsAssignable.has_value())
    {
        return m_fhMaxMcsAssignable.value();
    }

    // In case there is no sub-band info or no RBG has been allocated, return the wideband MCS
    if (m_dlSbMcsInfo.empty() || m_dlRBG.empty() || (m_mcsCsiSource == McsCsiSource::WIDEBAND_MCS))
    {
        return m_dlMcs;
    }

    // Otherwise, compute the SINR of allocated RBGs
    switch (m_mcsCsiSource)
    {
    // Estimate MCS based on the average MCS of allocated RBGs
    case McsCsiSource::AVG_MCS: {
        return ComputeMcs(this, &SbMcsInfo::mcs, [](double avg) { return (uint8_t)floor(avg); });
    }
    // Estimate MCS based on the average spectral efficiency of allocated RBGs
    case McsCsiSource::AVG_SPEC_EFF: {
        return ComputeMcs(this,
                          &SbMcsInfo::specEff,
                          std::bind_front(&NrAmc::GetMcsFromSpectralEfficiency, m_dlAmc));
    }
    // Estimate MCS based on the average SINR of allocated RBGs
    case McsCsiSource::AVG_SINR: {
        return ComputeMcs(this, &SbMcsInfo::sinr, [amc = m_dlAmc](double avgSinr) {
            return amc->GetMcsFromSpectralEfficiency(amc->GetSpectralEfficiencyForSinr(avgSinr));
        });
    }
    default:
        NS_ABORT_MSG("Invalid csi source for MCS computation");
    }
}

uint8_t&
NrMacSchedulerUeInfo::GetUlMcs(const UePtr& ue)
{
    return ue->m_ulMcs;
}

uint32_t&
NrMacSchedulerUeInfo::GetDlTBS(const UePtr& ue)
{
    return ue->m_dlTbSize;
}

std::unordered_map<uint8_t, LCGPtr>&
NrMacSchedulerUeInfo::GetDlLCG(const UePtr& ue)
{
    return ue->m_dlLCG;
}

std::unordered_map<uint8_t, LCGPtr>&
NrMacSchedulerUeInfo::GetUlLCG(const UePtr& ue)
{
    return ue->m_ulLCG;
}

NrMacHarqVector&
NrMacSchedulerUeInfo::GetDlHarqVector(const UePtr& ue)
{
    return ue->m_dlHarq;
}

NrMacHarqVector&
NrMacSchedulerUeInfo::GetUlHarqVector(const UePtr& ue)
{
    return ue->m_ulHarq;
}

void
NrMacSchedulerUeInfo::PrintLcInfo(uint16_t ue,
                                  uint8_t lcgId,
                                  uint8_t lcId,
                                  uint8_t fiveQi,
                                  uint8_t P,
                                  uint8_t minP)
{
    NS_LOG_DEBUG("UE " << ue << " LCG ID: " << static_cast<uint32_t>(lcgId) << " LC ID "
                       << static_cast<uint32_t>(lcId) << " QCI: " << static_cast<uint32_t>(fiveQi)
                       << " P: " << static_cast<uint32_t>(P) << " minP: " << +minP);
}

void
NrMacSchedulerUeInfo::ResetDlSchedInfo()
{
    m_dlMRBRetx = 0;
    m_dlRBG.clear();
    m_dlSym.clear();
    m_dlTbSize = 0;
}

void
NrMacSchedulerUeInfo::ResetUlSchedInfo()
{
    m_ulMRBRetx = 0;
    m_ulRBG.clear();
    m_ulSym.clear();
    m_ulTbSize = 0;
}

void
NrMacSchedulerUeInfo::UpdateDlMetric()
{
    if (m_dlRBG.empty())
    {
        m_dlTbSize = 0;
    }
    else
    {
        m_dlTbSize =
            m_dlAmc->CalculateTbSize(GetDlMcs(), m_dlRank, m_dlRBG.size() * GetNumRbPerRbg());
    }
}

void
NrMacSchedulerUeInfo::ResetDlMetric()
{
    m_dlTbSize = 0;
}

void
NrMacSchedulerUeInfo::UpdateUlMetric()
{
    if (m_ulRBG.empty())
    {
        m_ulTbSize = 0;
    }
    else
    {
        m_ulTbSize = m_ulAmc->CalculateTbSize(GetUlMcsFor(GetUlRbs()),
                                              m_ulRank,
                                              m_ulRBG.size() * GetNumRbPerRbg());
    }
}

void
NrMacSchedulerUeInfo::ResetUlMetric()
{
    m_ulTbSize = 0;
}

uint32_t
NrMacSchedulerUeInfo::GetTotalDlBuffer() const
{
    uint32_t totBuffer = 0;
    for (const auto& lcgInfo : m_dlLCG)
    {
        const auto& lcg = lcgInfo.second;
        totBuffer += lcg->GetTotalSize();
    }
    return totBuffer;
}

uint32_t
NrMacSchedulerUeInfo::GetTotalNumDlLc() const
{
    NS_LOG_FUNCTION(this);

    uint32_t total = 0;
    for (const auto& [lcgId, lcg] : m_dlLCG)
    {
        total += lcg->NumOfLC();
    }

    return total;
}

uint32_t
NrMacSchedulerUeInfo::GetTotalNumUlLc() const
{
    NS_LOG_FUNCTION(this);

    uint32_t total = 0;
    for (const auto& [lcgId, lcg] : m_ulLCG)
    {
        total += lcg->NumOfLC();
    }

    return total;
}

uint32_t
NrMacSchedulerUeInfo::GetTotalNumActiveDlLc() const
{
    NS_LOG_FUNCTION(this);

    uint32_t total = 0;
    for (const auto& [lcgId, lcg] : m_dlLCG)
    {
        total += lcg->NumOfActiveLC();
    }

    return total;
}

uint32_t
NrMacSchedulerUeInfo::GetTotalNumActiveUlLc() const
{
    NS_LOG_FUNCTION(this);

    uint32_t total = 0;
    for (const auto& [lcgId, lcg] : m_ulLCG)
    {
        total += lcg->NumOfActiveLC();
    }

    return total;
}

bool
NrMacSchedulerUeInfo::HasPendingDlSignalingTraffic() const
{
    constexpr uint8_t dlSignalingLcgId = 0;
    auto it = m_dlLCG.find(dlSignalingLcgId);
    return it != m_dlLCG.end() && it->second && it->second->GetTotalSize() > 0;
}

uint32_t
NrMacSchedulerUeInfo::GetNumRbPerRbg() const
{
    return m_getNumRbPerRbg();
}

void
NrMacSchedulerUeInfo::ReleaseLC(uint8_t lcid)
{
    for (auto lcgIt = m_dlLCG.begin(); lcgIt != m_dlLCG.end(); lcgIt++)
    {
        lcgIt->second->ReleaseLC(lcid);
    }
    for (auto lcgIt = m_ulLCG.begin(); lcgIt != m_ulLCG.end(); lcgIt++)
    {
        lcgIt->second->ReleaseLC(lcid);
    }
    auto it = m_dlLCG.begin();
    while (it != m_dlLCG.end())
    {
        if (it->second->GetLCId().empty())
        {
            m_dlLCG.erase(it);
            it = m_dlLCG.begin();
        }
        else
        {
            it++;
        }
    }
    it = m_ulLCG.begin();
    while (it != m_ulLCG.end())
    {
        if (it->second->GetLCId().empty())
        {
            m_ulLCG.erase(it);
            it = m_ulLCG.begin();
        }
        else
        {
            it++;
        }
    }
}

} // namespace ns3
