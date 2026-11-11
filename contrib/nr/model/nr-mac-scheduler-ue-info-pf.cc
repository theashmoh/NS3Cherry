// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-ue-info-pf.h"

#include "nr-mac-short-bsr-ce.h"

#include "ns3/log.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerUeInfoPF");

void
NrMacSchedulerUeInfoPF::UpdateDlPFMetric(const NrMacSchedulerNs3::FTResources& totAssigned,
                                         double timeWindow)
{
    NS_LOG_FUNCTION(this);

    NrMacSchedulerUeInfo::UpdateDlMetric();

    // No symbol assigned in this slot so far (an OFDMA scheduler says so after it has taken the
    // last RBGs back from a UE whose block was too small): the rate of the slot is 0. The
    // release divided by the 0 symbols; 0 / 0 made the average NaN for good, the metric of every
    // UE became its potential rate over 1e-9, and the scheduler served the best MCS only.
    m_currTputDl =
        totAssigned.m_sym > 0 ? static_cast<double>(m_dlTbSize) / totAssigned.m_sym : 0.0;
    m_avgTputDl = ((1.0 - (1.0 / static_cast<double>(timeWindow))) * m_lastAvgTputDl) +
                  ((1.0 / timeWindow) * m_currTputDl);

    NS_LOG_DEBUG("Update DL PF Metric for UE "
                 << m_rnti << " DL TBS: " << m_dlTbSize << " Updated currTputDl " << m_currTputDl
                 << " avgTputDl " << m_avgTputDl << " over n. of syms: " << +totAssigned.m_sym
                 << ", last Avg TH Dl " << m_lastAvgTputDl << " total sym assigned "
                 << static_cast<uint32_t>(totAssigned.m_sym)
                 << " updated DL metric: " << m_potentialTputDl / std::max(1E-9, m_avgTputDl));
}

void
NrMacSchedulerUeInfoPF::UpdateUlPFMetric(const NrMacSchedulerNs3::FTResources& totAssigned,
                                         double timeWindow)
{
    NS_LOG_FUNCTION(this);

    NrMacSchedulerUeInfo::UpdateUlMetric();

    // A block under PUSCH repetition takes K slots of the cell, not one: the rate it got is the
    // block's over K times the symbols (without this a UE that needs 8 repetitions would be
    // counted as served as cheaply as one that needs none, and PF would hand it as many
    // grants as the other, at 8 times the cost).
    const double reps = m_repAware && m_ulTbSize > 0
                            ? std::max<double>(1.0, GetUlRepetitionsFor(GetUlRbs()))
                            : 1.0;
    m_currTputUl = totAssigned.m_sym > 0
                       ? static_cast<double>(m_ulTbSize) / (totAssigned.m_sym * reps)
                       : 0.0;
    m_avgTputUl = ((1.0 - (1.0 / static_cast<double>(timeWindow))) * m_lastAvgTputUl) +
                  ((1.0 / timeWindow) * m_currTputUl);

    NS_LOG_DEBUG("Update UL PF Metric for UE "
                 << m_rnti << " UL TBS: " << m_ulTbSize << " Updated currTputUl " << m_currTputUl
                 << " avgTputUl " << m_avgTputUl << " over n. of syms: " << +totAssigned.m_sym
                 << ", last Avg TH Ul " << m_lastAvgTputUl << " total sym assigned "
                 << static_cast<uint32_t>(totAssigned.m_sym)
                 << " updated UL metric: " << m_potentialTputUl / std::max(1E-9, m_avgTputUl));
}

void
NrMacSchedulerUeInfoPF::CalculatePotentialTPutDl(
    const NrMacSchedulerNs3::FTResources& assignableInIteration)
{
    NS_LOG_FUNCTION(this);

    uint32_t rbsAssignable = assignableInIteration.m_rbg * GetNumRbPerRbg();
    m_potentialTputDl = m_dlAmc->GetPayloadSize(GetDlMcs(), m_dlRank, rbsAssignable);
    m_potentialTputDl /= assignableInIteration.m_sym;

    NS_LOG_INFO("UE " << m_rnti << " potentialTputDl " << m_potentialTputDl << " lastAvgThDl "
                      << m_lastAvgTputDl
                      << " DL metric: " << m_potentialTputDl / std::max(1E-9, m_avgTputDl));
}

void
NrMacSchedulerUeInfoPF::UpdateUlMarginal(uint32_t beamSym)
{
    NS_LOG_FUNCTION(this << beamSym);

    const uint32_t rbPerRbg = GetNumRbPerRbg();
    // the block now (m_ulTbSize is that of the groups assigned so far)
    const uint32_t nowTbs = m_ulRBG.empty() ? 0 : m_ulTbSize;
    // ... and with the next few groups more, each at the MCS of its width. The best gain per
    // group over a few of them: the size of a small block is not a smooth function of its
    // resources (the table of TS 38.214 5.1.3.2: 2 bytes for one group at MCS 0 and 1 for two).
    constexpr uint32_t kAhead = 4;
    double gain = 0.0;
    for (uint32_t k = 1; k <= kAhead; ++k)
    {
        const uint8_t mcs = GetUlMcsFor(GetUlRbs() + k * rbPerRbg);
        const uint32_t tbs =
            m_ulAmc->CalculateTbSize(mcs, m_ulRank, (m_ulRBG.size() + k * beamSym) * rbPerRbg);
        gain = std::max(gain, (static_cast<double>(tbs) - nowTbs) / k);
    }
    if (gain <= 0.0 && nowTbs < NrMacShortBsrCe::MinUlGrantBytes())
    {
        // not yet a block that a UE sends data in: it has to grow, whatever the table says
        gain = 1.0;
    }
    m_potentialTputUl = gain / beamSym;
    if (m_repAware)
    {
        m_potentialTputUl /= std::max<double>(
            1.0,
            GetUlRepetitionsFor(std::max(1U, std::min(GetUlRbs() + rbPerRbg, m_ulNarrowRbs))));
    }
    NS_LOG_INFO("UE " << m_rnti << " potentialTputUl " << m_potentialTputUl << " (the block is "
                      << nowTbs << " B, gain per group " << gain << " B)");
}

void
NrMacSchedulerUeInfoPF::CalculatePotentialTPutUl(
    const NrMacSchedulerNs3::FTResources& assignableInIteration)
{
    NS_LOG_FUNCTION(this);

    if (m_marginalUl)
    {
        // (a slot begins with no group assigned to the UE)
        UpdateUlMarginal(assignableInIteration.m_sym);
        return;
    }

    uint32_t rbsAssignable = assignableInIteration.m_rbg * GetNumRbPerRbg();
    m_potentialTputUl = m_ulAmc->GetPayloadSize(m_ulMcs, m_ulRank, rbsAssignable);
    m_potentialTputUl /= assignableInIteration.m_sym;
    if (m_repAware)
    {
        // ... at the repetitions of the grant it would get (the narrow one, UlGrantRbs)
        m_potentialTputUl /=
            std::max<double>(1.0, GetUlRepetitionsFor(std::max(1U, std::min(rbsAssignable, m_ulNarrowRbs))));
    }

    NS_LOG_INFO("UE " << m_rnti << " potentialTputUl " << m_potentialTputUl << " lastAvgThUl "
                      << m_lastAvgTputUl
                      << " UL metric: " << m_potentialTputUl / std::max(1E-9, m_avgTputUl));
}

} // namespace ns3
