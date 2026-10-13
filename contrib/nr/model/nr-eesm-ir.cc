// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-eesm-ir.h"

#include "ns3/log.h"
#include "ns3/uinteger.h"

#include <algorithm>

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
                MakeUintegerChecker<uint32_t>());
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
    ;

    // Coded bits beyond the soft buffer repeat bits already buffered (TS 38.212 Sect. 5.4.2.1),
    // so they do not lower the equivalent code rate. A buffer smaller than one transmission
    // cannot raise the equivalent MCS above the transmitted one: GetMcsEq only searches below it.
    if (m_softBufferBitsPerTb > 0)
    {
        codeBitsSum = std::min(codeBitsSum, m_softBufferBitsPerTb);
    }

    const_cast<NrEesmIr*>(this)->m_Reff = infoBits / static_cast<double>(codeBitsSum);

    NS_LOG_INFO(" Reff " << m_Reff << " HARQ history (previous) " << sinrHistory.size());

    // compute effective SINR with expSINR_previousTx and mapSumSize
    double expSINR_previousTx = DynamicCast<NrEesmErrorModelOutput>(sinrHistory.back())->m_sinrExp;
    return SinrEff(sinr, map, mcs, expSINR_previousTx, mapSumSize);
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
