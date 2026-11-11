// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-ofdma-pf.h"

#include "nr-mac-scheduler-ofdma-symbol-per-beam.h"
#include "nr-mac-scheduler-ue-info-pf.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerOfdmaPF");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerOfdmaPF);

TypeId
NrMacSchedulerOfdmaPF::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrMacSchedulerOfdmaPF")
            .SetParent<NrMacSchedulerOfdmaRR>()
            .AddConstructor<NrMacSchedulerOfdmaPF>()
            .AddAttribute("FairnessIndex",
                          "Value (between 0 and 1) that defines the PF metric (1 is the "
                          "traditional 3GPP PF, 0 is RR in throughput",
                          DoubleValue(1),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaPF::SetFairnessIndex,
                                             &NrMacSchedulerOfdmaPF::GetFairnessIndex),
                          MakeDoubleChecker<float>(0, 1))
            .AddAttribute(
                "LastAvgTPutWeight",
                "Weight of the last average throughput in the average throughput calculation",
                DoubleValue(99),
                MakeDoubleAccessor(&NrMacSchedulerOfdmaPF::SetTimeWindow,
                                   &NrMacSchedulerOfdmaPF::GetTimeWindow),
                MakeDoubleChecker<float>(0))
            .AddAttribute(
                "RepetitionAware",
                "PUSCH repetition: a block sent in K slots costs K slots, so the average "
                "throughput of the UE and its potential rate count it over K times the symbols. "
                "False (the default): as the module without repetitions (a block costs one "
                "slot). In a 30-UE voice cell the two gave the same on-time share to within "
                "0.3 points, three seeds, so the module's accounting is kept.",
                BooleanValue(false),
                MakeBooleanAccessor(&NrMacSchedulerOfdmaPF::m_repAware),
                MakeBooleanChecker())
            .AddAttribute(
                "UlMarginalRate",
                "Uplink: rank the UEs by what the next resource block group adds to the "
                "transport block of each, per average throughput, and give a UE no group that "
                "adds nothing. A UE spreads its power over the groups it gets, so with "
                "UlMcsPerRbs a wider grant has the MCS of its width and its block grows less "
                "than the groups: the whole band for the UE with the best rate, as PF gives it "
                "without this, is a block a UE at low SINR cannot send. false (the default): "
                "the rate of a fixed reference grant, as the module. Without UlMcsPerRbs the "
                "MCS does not depend on the width and the two rank alike, but for the rounding "
                "of the transport block size (0.1 % of the sum rate in a 10-UE test).",
                BooleanValue(false),
                MakeBooleanAccessor(&NrMacSchedulerOfdmaPF::m_marginalUl),
                MakeBooleanChecker());
    return tid;
}

NrMacSchedulerOfdmaPF::NrMacSchedulerOfdmaPF()
    : NrMacSchedulerOfdmaRR()
{
}

void
NrMacSchedulerOfdmaPF::SetFairnessIndex(double v)
{
    NS_LOG_FUNCTION(this);
    m_alpha = v;
}

double
NrMacSchedulerOfdmaPF::GetFairnessIndex() const
{
    NS_LOG_FUNCTION(this);
    return m_alpha;
}

void
NrMacSchedulerOfdmaPF::SetTimeWindow(double v)
{
    NS_LOG_FUNCTION(this);
    m_timeWindow = v;
}

double
NrMacSchedulerOfdmaPF::GetTimeWindow() const
{
    NS_LOG_FUNCTION(this);
    return m_timeWindow;
}

std::shared_ptr<NrMacSchedulerUeInfo>
NrMacSchedulerOfdmaPF::CreateUeRepresentation(
    const NrMacCschedSapProvider::CschedUeConfigReqParameters& params) const
{
    NS_LOG_FUNCTION(this);
    auto ue = std::make_shared<NrMacSchedulerUeInfoPF>(
        m_alpha,
        params.m_rnti,
        params.m_beamId,
        std::bind(&NrMacSchedulerOfdmaPF::GetNumRbPerRbg, this));
    ue->m_repAware = m_repAware;
    ue->m_marginalUl = m_marginalUl;
    return ue;
}

std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                   const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
NrMacSchedulerOfdmaPF::GetUeCompareDlFn() const
{
    return NrMacSchedulerUeInfoPF::CompareUeWeightsDl;
}

std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                   const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
NrMacSchedulerOfdmaPF::GetUeCompareUlFn() const
{
    return NrMacSchedulerUeInfoPF::CompareUeWeightsUl;
}

void
NrMacSchedulerOfdmaPF::AssignedDlResources(const UePtrAndBufferReq& ue,
                                           [[maybe_unused]] const FTResources& assigned,
                                           const FTResources& totAssigned) const
{
    NS_LOG_FUNCTION(this);
    auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first);
    uePtr->UpdateDlPFMetric(totAssigned, m_timeWindow);
}

void
NrMacSchedulerOfdmaPF::NotAssignedDlResources(
    const NrMacSchedulerNs3::UePtrAndBufferReq& ue,
    [[maybe_unused]] const NrMacSchedulerNs3::FTResources& assigned,
    const NrMacSchedulerNs3::FTResources& totAssigned) const
{
    NS_LOG_FUNCTION(this);
    auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first);
    uePtr->UpdateDlPFMetric(totAssigned, m_timeWindow);
}

void
NrMacSchedulerOfdmaPF::AssignedUlResources(const UePtrAndBufferReq& ue,
                                           [[maybe_unused]] const FTResources& assigned,
                                           const FTResources& totAssigned) const
{
    NS_LOG_FUNCTION(this);
    auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first);
    uePtr->UpdateUlPFMetric(totAssigned, m_timeWindow);
    if (m_marginalUl)
    {
        // the next group is worth what it adds to the block the UE has now
        uePtr->UpdateUlMarginal(totAssigned.m_sym);
    }
}

bool
NrMacSchedulerOfdmaPF::UlGrantCanGrow(const UePtrAndBufferReq& ue) const
{
    if (!m_marginalUl || ue.first->m_ulRBG.empty())
    {
        return true;
    }
    return std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first)->m_potentialTputUl > 0.0;
}

void
NrMacSchedulerOfdmaPF::NotAssignedUlResources(
    const NrMacSchedulerNs3::UePtrAndBufferReq& ue,
    [[maybe_unused]] const NrMacSchedulerNs3::FTResources& assigned,
    const NrMacSchedulerNs3::FTResources& totAssigned) const
{
    NS_LOG_FUNCTION(this);
    auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first);
    uePtr->UpdateUlPFMetric(totAssigned, m_timeWindow);
}

void
NrMacSchedulerOfdmaPF::BeforeDlSched(const UePtrAndBufferReq& ue,
                                     const FTResources& assignableInIteration) const
{
    NS_LOG_FUNCTION(this);
    auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first);
    uePtr->CalculatePotentialTPutDl(assignableInIteration);
}

void
NrMacSchedulerOfdmaPF::BeforeUlSched(const UePtrAndBufferReq& ue,
                                     const FTResources& assignableInIteration) const
{
    NS_LOG_FUNCTION(this);
    auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first);
    uePtr->CalculatePotentialTPutUl(assignableInIteration);
}

} // namespace ns3
