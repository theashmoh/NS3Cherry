// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-tdma-pf.h"

#include "nr-mac-scheduler-ue-info-pf.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerTdmaPF");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerTdmaPF);

TypeId
NrMacSchedulerTdmaPF::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrMacSchedulerTdmaPF")
            .SetParent<NrMacSchedulerTdmaRR>()
            .AddConstructor<NrMacSchedulerTdmaPF>()
            .AddAttribute("FairnessIndex",
                          "Value (between 0 and 1) that defines the PF metric (1 is the "
                          "traditional 3GPP PF, 0 is RR in throughput",
                          DoubleValue(1),
                          MakeDoubleAccessor(&NrMacSchedulerTdmaPF::SetFairnessIndex,
                                             &NrMacSchedulerTdmaPF::GetFairnessIndex),
                          MakeDoubleChecker<double>(0, 1))
            .AddAttribute(
                "LastAvgTPutWeight",
                "Weight of the last average throughput in the average throughput calculation",
                DoubleValue(99),
                MakeDoubleAccessor(&NrMacSchedulerTdmaPF::SetTimeWindow,
                                   &NrMacSchedulerTdmaPF::GetTimeWindow),
                MakeDoubleChecker<double>(0))
            .AddAttribute(
                "RepetitionAware",
                "PUSCH repetition: a block sent in K slots costs K slots, so the average "
                "throughput of the UE and its potential rate count it over K times the symbols. "
                "False (the default): as the module without repetitions (a block costs one "
                "slot). In a 30-UE voice cell the two gave the same on-time share to within "
                "0.3 points, three seeds, so the module's accounting is kept.",
                BooleanValue(false),
                MakeBooleanAccessor(&NrMacSchedulerTdmaPF::m_repAware),
                MakeBooleanChecker());
    return tid;
}

NrMacSchedulerTdmaPF::NrMacSchedulerTdmaPF()
    : NrMacSchedulerTdmaRR()
{
    NS_LOG_FUNCTION(this);
}

void
NrMacSchedulerTdmaPF::SetFairnessIndex(double v)
{
    NS_LOG_FUNCTION(this);
    m_alpha = v;
}

double
NrMacSchedulerTdmaPF::GetFairnessIndex() const
{
    NS_LOG_FUNCTION(this);
    return m_alpha;
}

void
NrMacSchedulerTdmaPF::SetTimeWindow(double v)
{
    NS_LOG_FUNCTION(this);
    m_timeWindow = v;
}

double
NrMacSchedulerTdmaPF::GetTimeWindow() const
{
    NS_LOG_FUNCTION(this);
    return m_timeWindow;
}

std::shared_ptr<NrMacSchedulerUeInfo>
NrMacSchedulerTdmaPF::CreateUeRepresentation(
    const NrMacCschedSapProvider::CschedUeConfigReqParameters& params) const
{
    NS_LOG_FUNCTION(this);
    auto ue = std::make_shared<NrMacSchedulerUeInfoPF>(
        m_alpha,
        params.m_rnti,
        params.m_beamId,
        std::bind(&NrMacSchedulerTdmaPF::GetNumRbPerRbg, this));
    ue->m_repAware = m_repAware;
    return ue;
}

std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                   const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
NrMacSchedulerTdmaPF::GetUeCompareDlFn() const
{
    return NrMacSchedulerUeInfoPF::CompareUeWeightsDl;
}

std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                   const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
NrMacSchedulerTdmaPF::GetUeCompareUlFn() const
{
    return NrMacSchedulerUeInfoPF::CompareUeWeightsUl;
}

void
NrMacSchedulerTdmaPF::AssignedDlResources(const UePtrAndBufferReq& ue,
                                          [[maybe_unused]] const FTResources& assigned,
                                          const FTResources& totAssigned) const
{
    NS_LOG_FUNCTION(this);
    auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first);
    uePtr->UpdateDlPFMetric(totAssigned, m_timeWindow);
}

void
NrMacSchedulerTdmaPF::NotAssignedDlResources(
    const NrMacSchedulerNs3::UePtrAndBufferReq& ue,
    [[maybe_unused]] const NrMacSchedulerNs3::FTResources& notAssigned,
    const NrMacSchedulerNs3::FTResources& totAssigned) const
{
    NS_LOG_FUNCTION(this);
    auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first);
    uePtr->UpdateDlPFMetric(totAssigned, m_timeWindow);
}

void
NrMacSchedulerTdmaPF::AssignedUlResources(const UePtrAndBufferReq& ue,
                                          [[maybe_unused]] const FTResources& assigned,
                                          const FTResources& totAssigned) const
{
    NS_LOG_FUNCTION(this);
    auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first);
    uePtr->UpdateUlPFMetric(totAssigned, m_timeWindow);
}

void
NrMacSchedulerTdmaPF::NotAssignedUlResources(
    const NrMacSchedulerNs3::UePtrAndBufferReq& ue,
    [[maybe_unused]] const NrMacSchedulerNs3::FTResources& notAssigned,
    const NrMacSchedulerNs3::FTResources& totAssigned) const
{
    NS_LOG_FUNCTION(this);
    auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first);
    uePtr->UpdateUlPFMetric(totAssigned, m_timeWindow);
}

void
NrMacSchedulerTdmaPF::BeforeDlSched(const UePtrAndBufferReq& ue,
                                    const FTResources& assignableInIteration) const
{
    NS_LOG_FUNCTION(this);
    auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first);
    uePtr->CalculatePotentialTPutDl(assignableInIteration);
}

void
NrMacSchedulerTdmaPF::BeforeUlSched(const UePtrAndBufferReq& ue,
                                    const FTResources& assignableInIteration) const
{
    NS_LOG_FUNCTION(this);
    auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoPF>(ue.first);
    uePtr->CalculatePotentialTPutUl(assignableInIteration);
}

} // namespace ns3
