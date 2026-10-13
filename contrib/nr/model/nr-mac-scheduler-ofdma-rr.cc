// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-ofdma-rr.h"

#include "nr-mac-scheduler-ofdma-symbol-per-beam.h"
#include "nr-mac-scheduler-ue-info-rr.h"

#include "ns3/log.h"

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("NrMacSchedulerOfdmaRR");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerOfdmaRR);

TypeId
NrMacSchedulerOfdmaRR::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrMacSchedulerOfdmaRR")
                            .SetParent<NrMacSchedulerOfdma>()
                            .AddConstructor<NrMacSchedulerOfdmaRR>();
    return tid;
}

NrMacSchedulerOfdmaRR::NrMacSchedulerOfdmaRR()
    : NrMacSchedulerOfdma()
{
}

std::shared_ptr<NrMacSchedulerUeInfo>
NrMacSchedulerOfdmaRR::CreateUeRepresentation(
    const NrMacCschedSapProvider::CschedUeConfigReqParameters& params) const
{
    NS_LOG_FUNCTION(this);
    return std::make_shared<NrMacSchedulerUeInfoRR>(
        params.m_rnti,
        params.m_beamId,
        std::bind(&NrMacSchedulerOfdmaRR::GetNumRbPerRbg, this));
}

void
NrMacSchedulerOfdmaRR::AssignedDlResources(const UePtrAndBufferReq& ue,
                                           [[maybe_unused]] const FTResources& assigned,
                                           [[maybe_unused]] const FTResources& totAssigned) const
{
    NS_LOG_FUNCTION(this);
    GetFirst GetUe;
    GetUe(ue)->UpdateDlMetric();
}

void
NrMacSchedulerOfdmaRR::AssignedUlResources(const UePtrAndBufferReq& ue,
                                           [[maybe_unused]] const FTResources& assigned,
                                           [[maybe_unused]] const FTResources& totAssigned) const
{
    NS_LOG_FUNCTION(this);
    GetFirst GetUe;
    GetUe(ue)->UpdateUlMetric();
}

std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                   const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
NrMacSchedulerOfdmaRR::GetUeCompareDlFn() const
{
    // The UE with the fewest RBGs of this slot first, as in uplink. Between UEs with as many,
    // the order they come in decides: the one served longest ago first
    // (NrMacSchedulerNs3::ComputeActiveUe), which makes the turns go round from slot to slot.
    //
    // The release kept a list of RNTIs here instead, in which a UE went to the end when its
    // transport block grew and to the front when it shrank. The assignment of every RBG tries
    // out all the remaining RBGs on the UE and takes them back again
    // (NrMacSchedulerOfdma::EstimateTotalTbCapacity): the block grew and shrank, the UE was at
    // the front again and got the next RBG too. One UE took the whole band of a slot while the
    // others waited, as long as it had data for it.
    return NrMacSchedulerUeInfoRR::CompareUeWeightsDl;
}

std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                   const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
NrMacSchedulerOfdmaRR::GetUeCompareUlFn() const
{
    return NrMacSchedulerUeInfoRR::CompareUeWeightsUl;
}

} // namespace ns3
