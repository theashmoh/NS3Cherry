// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#define NS_LOG_APPEND_CONTEXT                                                                      \
    do                                                                                             \
    {                                                                                              \
        std::clog << " [ CellId " << GetCellId() << ", bwpId " << GetBwpId() << ", rnti "          \
                  << m_rnti << "] ";                                                               \
    } while (false);

#include "nr-ue-mac.h"

#include "nr-control-messages.h"
#include "nr-mac-header-vs.h"

#include <cstdlib>
#include "nr-mac-short-bsr-ce.h"
#include "nr-pdcch.h"
#include "nr-phy-sap.h"
#include "nr-radio-bearer-tag.h"

#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrUeMac");
NS_OBJECT_ENSURE_REGISTERED(NrUeMac);

uint8_t NrUeMac::g_raPreambleId = 0;

///////////////////////////////////////////////////////////
// SAP forwarders
///////////////////////////////////////////////////////////

class UeMemberNrUeCmacSapProvider : public NrUeCmacSapProvider
{
  public:
    UeMemberNrUeCmacSapProvider(NrUeMac* mac);

    // inherited from NrUeCmacSapProvider
    void ConfigureRach(RachConfig rc) override;
    void StartContentionBasedRandomAccessProcedure() override;
    void StartNonContentionBasedRandomAccessProcedure(uint16_t rnti,
                                                      uint8_t preambleId,
                                                      uint8_t prachMask) override;
    void AddLc(uint8_t lcId,
               NrUeCmacSapProvider::LogicalChannelConfig lcConfig,
               NrMacSapUser* msu) override;
    void RemoveLc(uint8_t lcId) override;
    void Reset() override;
    void SetRnti(uint16_t rnti) override;
    void NotifyConnectionSuccessful() override;
    void SetImsi(uint64_t imsi) override;
    void RegisterToGnb(uint16_t cellId) override;

  private:
    NrUeMac* m_mac;
};

UeMemberNrUeCmacSapProvider::UeMemberNrUeCmacSapProvider(NrUeMac* mac)
    : m_mac(mac)
{
}

void
UeMemberNrUeCmacSapProvider::ConfigureRach(RachConfig rc)
{
    m_mac->DoConfigureRach(rc);
}

void
UeMemberNrUeCmacSapProvider::StartContentionBasedRandomAccessProcedure()
{
    m_mac->DoStartContentionBasedRandomAccessProcedure();
}

void
UeMemberNrUeCmacSapProvider::StartNonContentionBasedRandomAccessProcedure(uint16_t rnti,
                                                                          uint8_t preambleId,
                                                                          uint8_t prachMask)
{
    m_mac->DoStartNonContentionBasedRandomAccessProcedure(rnti, preambleId, prachMask);
}

void
UeMemberNrUeCmacSapProvider::AddLc(uint8_t lcId, LogicalChannelConfig lcConfig, NrMacSapUser* msu)
{
    m_mac->AddLc(lcId, lcConfig, msu);
}

void
UeMemberNrUeCmacSapProvider::RemoveLc(uint8_t lcid)
{
    m_mac->DoRemoveLc(lcid);
}

void
UeMemberNrUeCmacSapProvider::Reset()
{
    m_mac->DoReset();
}

void
UeMemberNrUeCmacSapProvider::SetRnti(uint16_t rnti)
{
    m_mac->SetRnti(rnti);
}

void
UeMemberNrUeCmacSapProvider::NotifyConnectionSuccessful()
{
    m_mac->DoNotifyConnectionSuccessful();
}

void
UeMemberNrUeCmacSapProvider::SetImsi(uint64_t imsi)
{
    m_mac->DoSetImsi(imsi);
}

void
UeMemberNrUeCmacSapProvider::RegisterToGnb(uint16_t cellId)
{
    m_mac->m_phySapProvider->RegisterToGnb(cellId);
}

class UeMemberNrMacSapProvider : public NrMacSapProvider
{
  public:
    UeMemberNrMacSapProvider(NrUeMac* mac);

    // inherited from NrMacSapProvider
    void TransmitPdu(TransmitPduParameters params) override;
    void BufferStatusReport(BufferStatusReportParameters params) override;

  private:
    NrUeMac* m_mac;
};

UeMemberNrMacSapProvider::UeMemberNrMacSapProvider(NrUeMac* mac)
    : m_mac(mac)
{
}

void
UeMemberNrMacSapProvider::TransmitPdu(TransmitPduParameters params)
{
    m_mac->DoTransmitPdu(params);
}

void
UeMemberNrMacSapProvider::BufferStatusReport(BufferStatusReportParameters params)
{
    m_mac->DoTransmitBufferStatusReport(params);
}

class NrUePhySapUser;

class MacUeMemberPhySapUser : public NrUePhySapUser
{
  public:
    MacUeMemberPhySapUser(NrUeMac* mac);

    void ReceivePhyPdu(Ptr<Packet> p) override;

    void ReceiveControlMessage(Ptr<NrControlMessage> msg) override;

    void SlotIndication(SfnSf sfn) override;

    // virtual void NotifyHarqDeliveryFailure (uint8_t harqId);

    uint8_t GetNumHarqProcess() const override;

  private:
    NrUeMac* m_mac;
};

MacUeMemberPhySapUser::MacUeMemberPhySapUser(NrUeMac* mac)
    : m_mac(mac)
{
}

void
MacUeMemberPhySapUser::ReceivePhyPdu(Ptr<Packet> p)
{
    m_mac->DoReceivePhyPdu(p);
}

void
MacUeMemberPhySapUser::ReceiveControlMessage(Ptr<NrControlMessage> msg)
{
    m_mac->DoReceiveControlMessage(msg);
}

void
MacUeMemberPhySapUser::SlotIndication(SfnSf sfn)
{
    m_mac->DoSlotIndication(sfn);
}

uint8_t
MacUeMemberPhySapUser::GetNumHarqProcess() const
{
    return m_mac->GetNumHarqProcess();
}

//-----------------------------------------------------------------------

TypeId
NrUeMac::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrUeMac")
            .SetParent<Object>()
            .AddConstructor<NrUeMac>()
            .AddAttribute(
                "NumHarqProcess",
                "Number of concurrent stop-and-wait Hybrid ARQ processes per user",
                UintegerValue(16),
                MakeUintegerAccessor(&NrUeMac::SetNumHarqProcess, &NrUeMac::GetNumHarqProcess),
                MakeUintegerChecker<uint8_t>())
            .AddAttribute("SrProhibitTimer",
                          "Minimum time between two transmissions of the scheduling request "
                          "(sr-ProhibitTimer of 3GPP TS 38.331). While the timer is running the "
                          "pending scheduling request is not transmitted, but it stays pending "
                          "and is sent on the first occasion after the timer expires. Set to zero "
                          "to transmit the scheduling request on every occasion.",
                          TimeValue(MilliSeconds(10)),
                          MakeTimeAccessor(&NrUeMac::m_srProhibitTimer),
                          MakeTimeChecker())
            // The buffer status report the UE puts in every uplink transport block. False: the module's, 5
            // bytes (the MAC subheader and the buffer levels of the 4 logical channel groups). True: the
            // short BSR of TS 38.321 6.1.3.1, 2 bytes (the subheader and one octet with the LCG ID and the
            // buffer level), for the logical channel group with the most data. On the small blocks of a
            // power-limited uplink the 3 bytes matter: with the BSR of 5 bytes no block under 15 bytes
            // carries data (10 bytes for a MAC subheader and an RLC entity, 5 for the BSR), with 2 bytes
            // no block under 12. The setting is global: every UE.
            .AddAttribute("CompactBsr",
                          "true: the 2-byte short BSR of TS 38.321 6.1.3.1 in every uplink block instead of the module's 5-byte report (global).",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrUeMac::SetCompactBsr, &NrUeMac::GetCompactBsr),
                          MakeBooleanChecker())
            // False (the default): the UE puts a buffer status report in every transport block of a grant,
            // new data or retransmission, and asks for a grant again whenever it is backlogged. True: as
            // TS 38.321 5.4.5. A report goes in a new block of a dynamic grant only when one is triggered:
            // data arrived in an empty buffer (regular BSR, which also triggers the scheduling request),
            // retxBSR-Timer ran out with data in the buffer (regular BSR), or periodicBSR-Timer ran out
            // (periodic BSR). A retransmission repeats the block with the report it had. A scheduling
            // request is repeated while the regular BSR is pending, not because the buffer is backlogged.
            // The blocks of a configured grant keep their report: the gNB reads them.
            .AddAttribute("CgBsrWhenRoom",
                          "A configured-grant block carries a BSR when the packet leaves 2 B for it or does not fit (TS 38.321 5.4.5); false: always (v9)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrUeMac::m_cgBsrWhenRoom),
                          MakeBooleanChecker())
            .AddAttribute("BsrOnDemand",
                          "true: BSRs only when triggered (TS 38.321 5.4.5: regular, retx-timer, periodic) and SRs only while a regular BSR is pending; false: a BSR in every block.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrUeMac::m_bsrOnDemand),
                          MakeBooleanChecker())
            .AddAttribute("PeriodicBsrTimer",
                          "periodicBSR-Timer (TS 38.331 BSR-Config): restarted when a BSR is "
                          "sent. Zero is infinity. Used with BsrOnDemand.",
                          TimeValue(MilliSeconds(20)),
                          MakeTimeAccessor(&NrUeMac::m_periodicBsrTimer),
                          MakeTimeChecker())
            // retxBSR-Timer (TS 38.331 BSR-Config): restarted when a BSR is sent and when a grant for new
            // data arrives; when it runs out with data in the buffer a regular BSR is triggered. Zero is
            // infinity. Used with BsrOnDemand.
            .AddAttribute("RetxBsrTimer",
                          "retxBSR-Timer (TS 38.331 BSR-Config): a regular BSR when it runs out with data in the buffer; 0 = infinity (with BsrOnDemand).",
                          TimeValue(MilliSeconds(80)),
                          MakeTimeAccessor(&NrUeMac::m_retxBsrTimer),
                          MakeTimeChecker())
            .AddAttribute("SrTransMax",
                          "Maximum number of transmissions of a pending scheduling request "
                          "(sr-TransMax of 3GPP TS 38.331). Once that many go unanswered the "
                          "request is cancelled and a random access procedure is started to get "
                          "uplink resources instead.",
                          UintegerValue(64),
                          MakeUintegerAccessor(&NrUeMac::m_srTransMax),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute(
                "IdealRandomAccess",
                "When true, the random access response timeout (ra-ResponseWindow) is never "
                "armed: the random access procedure cannot time out and the first RAR that "
                "answers the preamble completes it, whatever the propagation delay. Msg1 and "
                "Msg2 still cross the channel. When false, the timeout fires "
                "SlotPeriod * (6 + RaResponseWindowSize) + 2 * NrUePhy::TimingAdvance after "
                "the preamble, i.e. the window is shifted by the UE-gNB round-trip time as in "
                "TS 38.213 clause 8 for a cell with NTN-Config (Rel-17), and the preamble is "
                "retransmitted up to PreambleTransMax times. In both cases a RAR is accepted "
                "from the first slot after the preamble. Only set it when every preamble is "
                "answered: a preamble the gNB cannot fit a Msg3 grant for (it answers a few "
                "per slot) is retransmitted only by the timeout, so UEs that start random "
                "access in the same slot would otherwise never connect.",
                BooleanValue(false),
                MakeBooleanAccessor(&NrUeMac::m_idealRandomAccess),
                MakeBooleanChecker())
            .AddTraceSource("UeMacRxedCtrlMsgsTrace",
                            "Ue MAC Control Messages Traces.",
                            MakeTraceSourceAccessor(&NrUeMac::m_macRxedCtrlMsgsTrace),
                            "ns3::NrMacRxTrace::RxedUeMacCtrlMsgsTracedCallback")
            .AddTraceSource("UeMacTxedCtrlMsgsTrace",
                            "Ue MAC Control Messages Traces.",
                            MakeTraceSourceAccessor(&NrUeMac::m_macTxedCtrlMsgsTrace),
                            "ns3::NrMacRxTrace::TxedUeMacCtrlMsgsTracedCallback")
            .AddTraceSource("RaResponseTimeout",
                            "Trace fired upon RA response timeout",
                            MakeTraceSourceAccessor(&NrUeMac::m_raResponseTimeoutTrace),
                            "ns3::NrUeMac::RaResponseTimeoutTracedCallback")
            .AddTraceSource("UeMacStateMachineTrace",
                            "UE MAC state machine trace",
                            MakeTraceSourceAccessor(&NrUeMac::m_macUeStateMachine),
                            "ns3::NrUeMac::UeMacStateMachineTracedCallback");
    return tid;
}

NrUeMac::NrUeMac()
    : Object()
{
    NS_LOG_FUNCTION(this);
    m_cmacSapProvider = new UeMemberNrUeCmacSapProvider(this);
    m_macSapProvider = new UeMemberNrMacSapProvider(this);
    m_phySapUser = new MacUeMemberPhySapUser(this);
    m_raPreambleUniformVariable = CreateObject<UniformRandomVariable>();
}

NrUeMac::~NrUeMac()
{
}

void
NrUeMac::DoDispose()
{
    m_miUlHarqProcessesPacket.clear();
    m_miUlHarqProcessesPacketTimer.clear();
    m_ulBsrReceived.clear();
    m_lcInfoMap.clear();
    m_raPreambleUniformVariable = nullptr;
    delete m_macSapProvider;
    delete m_cmacSapProvider;
    delete m_phySapUser;
}

void
NrUeMac::SetRnti(uint16_t rnti)
{
    NS_LOG_FUNCTION(this);
    m_rnti = rnti;
}

void
NrUeMac::DoNotifyConnectionSuccessful()
{
    NS_LOG_FUNCTION(this);
    m_phySapProvider->NotifyConnectionSuccessful();
}

void
NrUeMac::DoSetImsi(uint64_t imsi)
{
    NS_LOG_FUNCTION(this);
    m_imsi = imsi;
}

uint16_t
NrUeMac::GetBwpId() const
{
    if (m_phySapProvider)
    {
        return m_phySapProvider->GetBwpId();
    }
    else
    {
        return UINT16_MAX;
    }
}

uint16_t
NrUeMac::GetCellId() const
{
    if (m_phySapProvider)
    {
        return m_phySapProvider->GetCellId();
    }
    else
    {
        return UINT16_MAX;
    }
}

uint16_t
NrUeMac::GetRnti() const
{
    return m_rnti;
}

uint64_t
NrUeMac::GetImsi() const
{
    return m_imsi;
}

void
NrUeMac::SetCurrentSlot(const SfnSf& sfn)
{
    m_currentSlot = sfn;
}

uint32_t
NrUeMac::GetTotalBufSize() const
{
    uint32_t ret = 0;
    for (const auto& it : m_ulBsrReceived)
    {
        ret += (it.second.txQueueSize + it.second.retxQueueSize + it.second.statusPduSize);
    }
    return ret;
}

/**
 * @brief Sets the number of HARQ processes
 * @param numHarqProcesses the maximum number of harq processes
 */
void
NrUeMac::SetNumHarqProcess(uint8_t numHarqProcess)
{
    m_numHarqProcess = numHarqProcess;

    m_miUlHarqProcessesPacket.resize(GetNumHarqProcess());
    for (auto& i : m_miUlHarqProcessesPacket)
    {
        if (i.m_pktBurst == nullptr)
        {
            Ptr<PacketBurst> pb = CreateObject<PacketBurst>();
            i.m_pktBurst = pb;
        }
    }
    m_miUlHarqProcessesPacketTimer.resize(GetNumHarqProcess(), 0);
}

/**
 * @return number of HARQ processes
 */
uint8_t
NrUeMac::GetNumHarqProcess() const
{
    return m_numHarqProcess;
}

// forwarded from MAC SAP
void
NrUeMac::DoTransmitPdu(NrMacSapProvider::TransmitPduParameters params)
{
    NS_LOG_FUNCTION(this);
    if (m_ulDci == nullptr)
    {
        NS_LOG_WARN("Null DCI received for transmission.");
        return;
    }
    NS_ASSERT(m_ulDci);
    NS_ASSERT(m_ulDci->m_harqProcess == params.harqProcessId);

    m_miUlHarqProcessesPacket.at(params.harqProcessId).m_lcidList.push_back(params.lcid);

    NrMacHeaderVs header;
    header.SetLcId(params.lcid);
    header.SetSize(params.pdu->GetSize());

    params.pdu->AddHeader(header);

    NrRadioBearerTag bearerTag(params.rnti, params.lcid, 0);
    params.pdu->AddPacketTag(bearerTag);

    if (!m_miUlHarqProcessesPacket.at(params.harqProcessId).m_pktBurst)
    {
        m_miUlHarqProcessesPacket.at(params.harqProcessId).m_pktBurst = CreateObject<PacketBurst>();
    }
    m_miUlHarqProcessesPacket.at(params.harqProcessId).m_pktBurst->AddPacket(params.pdu);
    m_miUlHarqProcessesPacketTimer.at(params.harqProcessId) = GetNumHarqProcess();

    m_ulDciTotalUsed += params.pdu->GetSize();

    NS_ASSERT_MSG(m_ulDciTotalUsed <= m_ulDci->m_tbSize,
                  "We used more data than the DCI allowed us.");

    m_phySapProvider->SendMacPdu(params.pdu, m_ulDciSfnsf, m_ulDci->m_symStart, m_ulDci->m_rnti);
}

void
NrUeMac::DoTransmitBufferStatusReport(NrMacSapProvider::BufferStatusReportParameters params)
{
    NS_LOG_FUNCTION(this << static_cast<uint32_t>(params.lcid));

    const bool bufferWasEmpty = GetTotalBufSize() == 0;
    auto it = m_ulBsrReceived.find(params.lcid);

    NS_LOG_INFO("Received BSR for LC Id" << static_cast<uint32_t>(params.lcid));

    if (it != m_ulBsrReceived.end())
    {
        // update entry
        (*it).second = params;
    }
    else
    {
        it = m_ulBsrReceived.insert(std::make_pair(params.lcid, params)).first;
    }

    // Re-arm the SR whenever new buffer status arrives while backlogged, as the UE keeps
    // requesting resources while a BSR is pending and no UL-SCH resources are available
    // (3GPP TS 38.321, clauses 5.4.4 and 5.4.5). The previous condition keyed the re-arm to
    // the last received UL DCI (HARQ process 0, or rv 3), which deadlocks the uplink when the
    // bootstrap BSR TBs are lost: the gNB stops granting without a BSR, the stale m_ulDci
    // never satisfies the condition, and no SR is ever sent again.
    if (m_bsrOnDemand)
    {
        // Regular BSR (TS 38.321 5.4.5): UL data became available while no logical channel had
        // any. It triggers a scheduling request (5.4.4) if there is no UL-SCH resource for it.
        // The request is repeated in DoSlotIndication() while the BSR is pending; it is not
        // re-armed here whenever the buffer changes.
        if (bufferWasEmpty && GetTotalBufSize() > 0)
        {
            m_regularBsrPending = true;
            if (m_srState == INACTIVE)
            {
                m_srCounter = 0;
                m_srState = TO_SEND;
            }
        }
        return;
    }
    if (m_srState == INACTIVE || (m_srState == ACTIVE && GetTotalBufSize() > 0))
    {
        if (m_srState == INACTIVE)
        {
            // A scheduling request is triggered while no other one is pending, so the count of
            // its transmissions starts over (3GPP TS 38.321, clause 5.4.4).
            m_srCounter = 0;
            NS_LOG_INFO("m_srState = INACTIVE -> TO_SEND, bufSize " << GetTotalBufSize());
            m_macUeStateMachine(m_imsi,
                                m_currentSlot,
                                GetCellId(),
                                m_rnti,
                                GetBwpId(),
                                m_srState,
                                m_ulBsrReceived,
                                1,
                                "DoTransmitBufferStatusReport");
        }
        else
        {
            NS_LOG_INFO("m_srState = ACTIVE (BSR Timer expired) -> TO_SEND, bufSize "
                        << GetTotalBufSize());
            m_macUeStateMachine(m_imsi,
                                m_currentSlot,
                                GetCellId(),
                                m_rnti,
                                GetBwpId(),
                                m_srState,
                                m_ulBsrReceived,
                                0,
                                "DoTransmitBufferStatusReport");
        }
        m_srState = TO_SEND;
    }
}

void
NrUeMac::SendBufferStatusReport(const SfnSf& dataSfn, uint8_t symStart)
{
    NS_LOG_FUNCTION(this);

    if (m_rnti == 0)
    {
        NS_LOG_INFO("MAC not initialized, BSR deferred");
        return;
    }

    if (m_ulBsrReceived.empty())
    {
        NS_LOG_INFO("No BSR report to transmit");
        return;
    }
    MacCeElement bsr = MacCeElement();
    bsr.m_rnti = m_rnti;
    bsr.m_macCeType = MacCeElement::BSR;

    // BSR is reported for each LCG
    std::unordered_map<uint8_t, NrMacSapProvider::BufferStatusReportParameters>::iterator it;
    std::vector<uint32_t> queue(4, 0); // one value per each of the 4 LCGs, initialized to 0
    for (it = m_ulBsrReceived.begin(); it != m_ulBsrReceived.end(); it++)
    {
        uint8_t lcid = it->first;
        std::unordered_map<uint8_t, LcInfo>::iterator lcInfoMapIt;
        lcInfoMapIt = m_lcInfoMap.find(lcid);
        NS_ASSERT(lcInfoMapIt != m_lcInfoMap.end());
        NS_ASSERT_MSG((lcid != 0) ||
                          (((*it).second.txQueueSize == 0) && ((*it).second.retxQueueSize == 0) &&
                           ((*it).second.statusPduSize == 0)),
                      "BSR should not be used for LCID 0");
        uint8_t lcg = lcInfoMapIt->second.lcConfig.logicalChannelGroup;
        queue.at(lcg) +=
            ((*it).second.txQueueSize + (*it).second.retxQueueSize + (*it).second.statusPduSize);

        // The reported size is the data plus what it takes to send it: the BSR itself and 3 bytes
        // for each MAC subheader. This is the nr module's own, kept so that the grant that
        // answers the report has room for the headers. TS 38.321 6.1.3.1 does not count them
        // ("the size of the RLC headers and MAC subheaders are not considered"); a gNB adds its
        // own allowance to what it reads. Checked against V19.4.0: a few bytes of difference
        // per grant, now one level of Table 6.1.3.1-1 more, now less.
        if (queue.at(lcg) != 0)
        {
            NS_LOG_DEBUG("Adding " << NrMacShortBsrCe::OverheadBytes()
                                   << " bytes for SHORT_BSR for LCID:" << +lcid);
            queue.at(lcg) += NrMacShortBsrCe::OverheadBytes();
        }
        if ((*it).second.txQueueSize > 0)
        {
            NS_LOG_DEBUG("Adding 3 bytes for TX subheader for LCID:" << +lcid);
            queue.at(lcg) += 3;
        }
        if ((*it).second.retxQueueSize > 0)
        {
            NS_LOG_DEBUG("Adding 3 bytes for RX subheader for LCID:" << +lcid);
            queue.at(lcg) += 3;
        }
        if ((*it).second.statusPduSize > 0)
        {
            NS_LOG_DEBUG("Adding 3 bytes for RLC status PDU subheader for LCID:" << +lcid);
            queue.at(lcg) += 3;
        }
    }

    NS_LOG_INFO("Sending BSR with this info for the LCG: "
                << queue.at(0) << " " << queue.at(1) << " " << queue.at(2) << " " << queue.at(3));
    // FF API says that all 4 LCGs are always present
    bsr.m_macCeValue.m_bufferStatus.push_back(NrMacShortBsrCe::FromBytesToLevel(queue.at(0)));
    bsr.m_macCeValue.m_bufferStatus.push_back(NrMacShortBsrCe::FromBytesToLevel(queue.at(1)));
    bsr.m_macCeValue.m_bufferStatus.push_back(NrMacShortBsrCe::FromBytesToLevel(queue.at(2)));
    bsr.m_macCeValue.m_bufferStatus.push_back(NrMacShortBsrCe::FromBytesToLevel(queue.at(3)));

    // create the message. It is used only for tracing, but we don't send it...
    Ptr<NrBsrMessage> msg = Create<NrBsrMessage>();
    msg->SetSourceBwpArfcn(m_phySapProvider->GetArfcn());
    msg->SetBsr(bsr);

    m_macTxedCtrlMsgsTrace(m_imsi, m_currentSlot, GetCellId(), bsr.m_rnti, GetBwpId(), msg);

    // Here we send the real SHORT_BSR, as a subpdu.
    Ptr<Packet> p = Create<Packet>();

    // Please note that the levels are defined from the standard. In this case,
    // we have 5 bit available, so use such standard levels. In the future,
    // when LONG BSR will be implemented, this have to change.
    NrMacShortBsrCe header;
    header.m_bufferSizeLevel_0 = NrMacShortBsrCe::FromBytesToLevel(queue.at(0));
    header.m_bufferSizeLevel_1 = NrMacShortBsrCe::FromBytesToLevel(queue.at(1));
    header.m_bufferSizeLevel_2 = NrMacShortBsrCe::FromBytesToLevel(queue.at(2));
    header.m_bufferSizeLevel_3 = NrMacShortBsrCe::FromBytesToLevel(queue.at(3));

    if (NrMacShortBsrCe::IsCompact())
    {
        // the short BSR of TS 38.321 6.1.3.1 reports one LCG: the one with the most data
        // (the 2 bytes it takes were kept in the grant; a UE with data in several LCGs reports
        // the largest, as a truncated BSR does)
        uint8_t big = 0;
        for (uint8_t i = 1; i < 4; ++i)
        {
            if (queue.at(i) > queue.at(big))
            {
                big = i;
            }
        }
        header.SetCompactLcg(big);
    }
    p->AddHeader(header);

    NrRadioBearerTag bearerTag(m_rnti, NrMacHeaderFsUl::SHORT_BSR, 0);
    p->AddPacketTag(bearerTag);

    m_ulDciTotalUsed += p->GetSize();
    NS_ASSERT_MSG(m_ulDciTotalUsed <= m_ulDci->m_tbSize,
                  "We used more data than the DCI allowed us.");

    m_harqBsr[m_ulDci->m_harqProcess] = p->Copy(); // for a retransmission, see ProcessUlDci()
    m_phySapProvider->SendMacPdu(p, dataSfn, symStart, m_ulDci->m_rnti);

    // The transmitted MAC PDU carries the buffer status, so the pending scheduling requests are
    // cancelled and their sr-ProhibitTimer is stopped (3GPP TS 38.321, clause 5.4.4). The gNB now
    // knows what the UE has, so the transmissions spent asking for this grant no longer count
    // against sr-TransMax: only requests that go unanswered do.
    m_srCounter = 0;
    m_lastSrSent = Time::Min();
    // the report is out: no regular BSR is pending, periodicBSR-Timer and retxBSR-Timer start
    // over (5.4.5), and the request for a grant that it answered is cancelled (5.4.4)
    m_regularBsrPending = false;
    m_lastBsrSent = Simulator::Now();
    m_retxBsrStart = Simulator::Now();
    if (m_bsrOnDemand)
    {
        m_srState = INACTIVE;
    }

    m_macUeStateMachine(m_imsi,
                        m_currentSlot,
                        GetCellId(),
                        m_rnti,
                        GetBwpId(),
                        m_srState,
                        m_ulBsrReceived,
                        m_ulDci->m_ndi,
                        "SendBufferStatusReport");
}

bool
NrUeMac::BsrTriggered() const
{
    if (m_cgBsrWhenRoom && m_ulDci->m_sps != DciInfoElementTdma::DYNAMIC)
    {
        // A configured-grant block: the packet in it found the buffer empty, a Regular BSR
        // (TS 38.321 5.4.5). It goes in when the block has room for it after the packet (and
        // any confirmation), or when the packet does not fit (it cannot be cancelled then);
        // it may be cancelled when the block holds the packet but not the report too.
        const uint32_t grant = GrantBytes();
        const uint32_t data = GetTotalBufSize();
        const uint32_t need =
            m_cgConfirmBytes + (data > 0 ? data + NrMacHeaderVs::SubheaderBytes(data) : 0);
        const uint32_t bsr = NrMacShortBsrCe::OverheadBytes();
        return need + bsr <= grant || need > grant;
    }
    if (!m_bsrOnDemand || m_ulDci->m_sps != DciInfoElementTdma::DYNAMIC)
    {
        return true;
    }
    if (m_regularBsrPending)
    {
        return true;
    }
    return m_periodicBsrTimer.IsStrictlyPositive() &&
           (m_lastBsrSent == Time::Min() || Simulator::Now() - m_lastBsrSent >= m_periodicBsrTimer);
}

void
NrUeMac::SetCompactBsr(bool compact)
{
    NrMacShortBsrCe::SetCompact(compact);
}

bool
NrUeMac::GetCompactBsr() const
{
    return NrMacShortBsrCe::IsCompact();
}

void
NrUeMac::SetUeCmacSapUser(NrUeCmacSapUser* s)
{
    m_cmacSapUser = s;
}

NrUeCmacSapProvider*
NrUeMac::GetUeCmacSapProvider()
{
    return m_cmacSapProvider;
}

void
NrUeMac::RefreshHarqProcessesPacketBuffer()
{
    NS_LOG_FUNCTION(this);

    for (std::size_t i = 0; i < m_miUlHarqProcessesPacketTimer.size(); i++)
    {
        if (m_miUlHarqProcessesPacketTimer.at(i) == 0 && m_miUlHarqProcessesPacket.at(i).m_pktBurst)
        {
            if (m_miUlHarqProcessesPacket.at(i).m_pktBurst->GetSize() > 0)
            {
                // timer expired: drop packets in buffer for this process
                NS_LOG_INFO("HARQ Proc Id " << i << " packets buffer expired");
                Ptr<PacketBurst> emptyPb = CreateObject<PacketBurst>();
                m_miUlHarqProcessesPacket.at(i).m_pktBurst = emptyPb;
                m_miUlHarqProcessesPacket.at(i).m_lcidList.clear();
            }
        }
        else
        {
            // m_miUlHarqProcessesPacketTimer.at (i)--;  // ignore HARQ timeout
        }
    }
}

void
NrUeMac::DoSlotIndication(const SfnSf& sfn)
{
    NS_LOG_FUNCTION(this);
    m_currentSlot = sfn;
    NS_LOG_INFO("Slot " << m_currentSlot);

    RefreshHarqProcessesPacketBuffer();

    // The blocks of the configured grant that are sent in the next slot: fill them now
    for (auto block = m_cgBlocks.begin(); block != m_cgBlocks.end();)
    {
        if (block->second.Normalize() > sfn.Normalize() + 1)
        {
            ++block;
            continue;
        }
        const auto due = *block;
        block = m_cgBlocks.erase(block);
        DoProcessUlGrant(due.first, due.second);
    }

    if (m_bsrOnDemand && m_rnti != 0)
    {
        // retxBSR-Timer ran out with data in the buffer: a regular BSR (TS 38.321 5.4.5)
        if (m_retxBsrTimer.IsStrictlyPositive() && m_retxBsrStart != Time::Min() &&
            GetTotalBufSize() > 0 && !m_regularBsrPending &&
            Simulator::Now() - m_retxBsrStart >= m_retxBsrTimer)
        {
            m_regularBsrPending = true;
            m_retxBsrStart = Simulator::Now();
            if (m_srState == INACTIVE)
            {
                m_srCounter = 0;
                m_srState = TO_SEND;
            }
        }
        // the scheduling request is pending until a grant takes the BSR: again after
        // sr-ProhibitTimer (5.4.4)
        if (m_regularBsrPending && m_srState == ACTIVE && m_lastSrSent != Time::Min() &&
            Simulator::Now() - m_lastSrSent >= m_srProhibitTimer)
        {
            m_srState = TO_SEND;
        }
    }

    // A pending scheduling request is transmitted on the first occasion on which
    // sr-ProhibitTimer is not running (3GPP TS 38.321, clause 5.4.4). While the timer runs the
    // request stays pending, so a backlogged UE asks again as soon as it is allowed to, without
    // repeating the request on every slot.
    if (m_srState == TO_SEND &&
        (m_lastSrSent == Time::Min() || Simulator::Now() - m_lastSrSent >= m_srProhibitTimer))
    {
        if (m_srCounter < m_srTransMax)
        {
            NS_LOG_INFO("Sending SR to PHY in slot " << sfn << ", transmission " << m_srCounter + 1
                                                     << " of " << m_srTransMax);
            SendSR();
            m_srCounter++;
            m_lastSrSent = Simulator::Now();
            m_srState = ACTIVE;
            NS_LOG_INFO("m_srState = TO_SEND -> ACTIVE");
            m_macUeStateMachine(m_imsi,
                                m_currentSlot,
                                GetCellId(),
                                m_rnti,
                                GetBwpId(),
                                m_srState,
                                m_ulBsrReceived,
                                1,
                                "DoSlotIndication");
        }
        else
        {
            // sr-TransMax transmissions went unanswered, so the scheduling request procedure is
            // abandoned: the pending requests are cancelled and a random access procedure is
            // started to get uplink resources instead (3GPP TS 38.321, clause 5.4.4).
            NS_LOG_WARN("Reached sr-TransMax ("
                        << m_srTransMax
                        << ") scheduling requests without a grant, falling back to random access");
            m_srState = INACTIVE;
            m_srCounter = 0;
            m_lastSrSent = Time::Min();
            m_macUeStateMachine(m_imsi,
                                m_currentSlot,
                                GetCellId(),
                                m_rnti,
                                GetBwpId(),
                                m_srState,
                                m_ulBsrReceived,
                                0,
                                "DoSlotIndication");

            if (m_rachConfigured)
            {
                RandomlySelectAndSendRaPreamble();
            }
            else
            {
                NS_LOG_WARN("RACH is not configured, cannot fall back to random access");
            }
        }
    }

    // Feedback missing
}

void
NrUeMac::SendSR() const
{
    NS_LOG_FUNCTION(this);

    if (m_rnti == 0)
    {
        NS_LOG_INFO("MAC not initialized, SR deferred");
        return;
    }

    // create the SR to send to the gNB
    Ptr<NrSRMessage> msg = Create<NrSRMessage>();
    msg->SetSourceBwpArfcn(m_phySapProvider->GetArfcn());
    msg->SetRNTI(m_rnti);

    m_macTxedCtrlMsgsTrace(m_imsi, m_currentSlot, GetCellId(), m_rnti, GetBwpId(), msg);
    m_phySapProvider->SendControlMessage(msg);
}

void
NrUeMac::DoReceivePhyPdu(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this);

    NrRadioBearerTag tag;
    p->RemovePacketTag(tag);

    if (tag.GetRnti() != m_rnti) // Packet is for another user
    {
        NS_LOG_WARN("Packet is for another user. RNTI does not correspond.");
        return;
    }

    NrMacHeaderVs header;
    p->RemoveHeader(header);

    NrMacSapUser::ReceivePduParameters rxParams;
    rxParams.p = p;
    rxParams.rnti = m_rnti;
    rxParams.lcid = header.GetLcId();

    auto it = m_lcInfoMap.find(header.GetLcId());
    // Ignore non-existing lcids
    if (it == m_lcInfoMap.end())
    {
        NS_LOG_WARN("LC info not found for this logical channel id:" << +header.GetLcId());
        return;
    }

    // p can be empty. Well, right now no, but when someone will add CE in downlink,
    // then p can be empty.
    if (rxParams.p->GetSize() > 0)
    {
        NS_LOG_INFO("Call MAC SAP user to receive PDU" << rxParams.p->GetSize());
        it->second.macSapUser->ReceivePdu(rxParams);
    }
    else
    {
        NS_LOG_WARN("Empty packet.");
    }
}

void
NrUeMac::RecvRaResponse(NrBuildRarListElement_s raResponse)
{
    NS_LOG_FUNCTION(this);
    m_waitingForRaResponse = false;
    m_noRaResponseReceivedEvent.Cancel();
    NS_LOG_INFO(" IMSI " << m_imsi << " RNTI " << m_rnti << " received RAR for RA preamble ID "
                         << +m_raPreambleId
                         << ", setting T-C-RNTI = " << raResponse.ulMsg3Dci->m_rnti
                         << " at: " << Simulator::Now().As(Time::MS));
    // Save data for MSG3
    m_rnti = raResponse.ulMsg3Dci->m_rnti;
    // The Msg3 offset counts from the gNB slot that carried the RAR (our current slot unless
    // the PHY applies a timing advance)
    m_ulDciSfnsf = m_phySapProvider->GetDlArrivalSlot(m_currentSlot);
    m_ulDciSfnsf.Add(raResponse.k2Delay);
    m_ulDciTotalUsed = 0;
    m_ulDci = raResponse.ulMsg3Dci;

    m_cmacSapUser->SetTemporaryCellRnti(m_rnti);
    // in principle we should wait for contention resolution,
    // but in the current NR model when two or more identical
    // preambles are sent no one is received, so there is no need
    // for contention resolution
    m_cmacSapUser->NotifyRandomAccessSuccessful();
    // Trigger Tx opportunity for Message 3 over LC 0
    // this is needed since Message 3's UL GRANT is in the RAR, not in UL-DCIs
    const uint8_t lc0Lcid = 0;
    auto lc0InfoIt = m_lcInfoMap.find(lc0Lcid);
    NS_ASSERT_MSG(lc0InfoIt != m_lcInfoMap.end(),
                  "LC0 not mapped to this UE MAC with bwpId:" << GetBwpId());
    auto lc0BsrIt = m_ulBsrReceived.find(lc0Lcid);
    if ((lc0BsrIt != m_ulBsrReceived.end()) && (lc0BsrIt->second.txQueueSize > 0))
    {
        NS_LOG_INFO(
            "Notify RLC about transmission opportunity for sending RRC CONNECTION REQUEST.");
        NS_ASSERT_MSG(raResponse.ulMsg3Dci->m_tbSize > lc0BsrIt->second.txQueueSize,
                      "segmentation of Message 3 is not allowed");
        NrMacSapUser::TxOpportunityParameters txOpParams;
        txOpParams.lcid = lc0Lcid;
        txOpParams.rnti = m_rnti;
        txOpParams.bytes = raResponse.ulMsg3Dci->m_tbSize;
        txOpParams.layer = 0;
        txOpParams.harqId = 0;
        txOpParams.componentCarrierId = GetBwpId();

        lc0InfoIt->second.macSapUser->NotifyTxOpportunity(txOpParams);
        lc0BsrIt->second.txQueueSize = 0;
        lc0BsrIt->second.retxQueueSize = 0;
        lc0BsrIt->second.statusPduSize = 0;
        NS_LOG_INFO("BSR erase lcid=" << +lc0Lcid << " reason Msg3 sent");
        m_ulBsrReceived.erase(lc0BsrIt);
    }
}

uint32_t
NrUeMac::GrantBytes() const
{
    return m_ulDci->m_spsTbBytes > 0 ? std::min(m_ulDci->m_spsTbBytes, m_ulDci->m_tbSize)
                                     : m_ulDci->m_tbSize;
}

void
NrUeMac::ProcessUlDci(const Ptr<NrUlDciMessage>& dciMsg)
{
    NS_LOG_FUNCTION(this);

    // Configured grant Type 2 (TS 38.321 5.8.2): the UE confirms a DCI that activates or
    // releases a configuration with a MAC CE in its next new transport block
    const auto& grant = dciMsg->GetDciInfoElement();
    if (grant->m_sps == DciInfoElementTdma::DYNAMIC && grant->m_cgConfirmAhead &&
        grant->m_cgSerial > m_cgLastActivationSerial)
    {
        // The DCIs that activate (and release) a configuration came before this grant's PUSCH:
        // it carries their confirmation, the configuration's first block does not. (If the
        // activation came first, it triggered the confirmation itself.)
        m_cgConfirmBytes = grant->m_cgConfirmBytes;
        m_cgAheadSerial = grant->m_cgSerial;
    }
    if (grant->m_sps == DciInfoElementTdma::SPS_RELEASE)
    {
        m_macRxedCtrlMsgsTrace(m_imsi, m_currentSlot, GetCellId(), m_rnti, GetBwpId(), dciMsg);
        if (grant->m_cgSerial + 1 != m_cgAheadSerial || m_cgAheadSerial == 0)
        {
            m_cgConfirmBytes = grant->m_cgConfirmBytes;
        }
        if (m_cgActiveConfig == grant->m_spsConfig)
        {
            // (the release of one configuration may come in the slot that activates another)
            m_cgActive = false;
        }
        return; // a release grants nothing
    }
    if (grant->m_sps == DciInfoElementTdma::SPS_ACTIVATION)
    {
        if (grant->m_cgSerial != m_cgAheadSerial || m_cgAheadSerial == 0)
        {
            m_cgConfirmBytes = grant->m_cgConfirmBytes;
        }
        m_cgLastActivationSerial = std::max(m_cgLastActivationSerial, grant->m_cgSerial);
        m_cgActive = true;
        m_cgActiveConfig = grant->m_spsConfig;
    }

    // K2 counts from the gNB slot that carried the DCI (our current slot unless the PHY applies
    // a timing advance)
    SfnSf dataSfn = m_phySapProvider->GetDlArrivalSlot(m_currentSlot);
    dataSfn.Add(dciMsg->GetKDelay());

    // (a further transmission of such a block, too: it repeats what the block will carry)
    const bool cgRepetition = grant->m_sps == DciInfoElementTdma::PUSCH_REPETITION &&
                              grant->m_repFirst &&
                              grant->m_repFirst->m_sps != DciInfoElementTdma::DYNAMIC;
    if (((grant->m_sps != DciInfoElementTdma::DYNAMIC &&
          grant->m_sps != DciInfoElementTdma::PUSCH_REPETITION) ||
         cgRepetition) &&
        dataSfn.Normalize() > m_currentSlot.Normalize() + 1)
    {
        // A block of a configured grant: filled when its slot comes (DoSlotIndication())
        m_cgBlocks.emplace_back(dciMsg, dataSfn);
        return;
    }
    DoProcessUlGrant(dciMsg, dataSfn);
}

void
NrUeMac::DoProcessUlGrant(const Ptr<NrUlDciMessage>& dciMsg, const SfnSf& dataSfn)
{
    NS_LOG_FUNCTION(this);
    const auto& grant = dciMsg->GetDciInfoElement();
    if (grant->m_sps == DciInfoElementTdma::SPS_OCCASION && GetTotalBufSize() == 0 &&
        m_cgConfirmBytes == 0)
    {
        // A block of the configuration and nothing to send in it: no MAC PDU is generated
        // (TS 38.321 5.4.3.1.3) and the block stays unused
        grant->m_cgSkipped = true;
        NS_LOG_LOGIC("UE " << m_rnti << " grant kind " << +grant->m_sps << ": nothing to send");
        return;
    }
    if (grant->m_sps == DciInfoElementTdma::PUSCH_REPETITION && grant->m_repFirst &&
        grant->m_repFirst->m_cgSkipped)
    {
        // A further transmission of a block of a configured grant the UE left unused: nothing
        grant->m_cgSkipped = true;
        return;
    }
    const uint32_t bufferBefore = GetTotalBufSize();

    // Saving the data we need in DoTransmitPdu
    m_ulDciSfnsf = dataSfn;
    m_ulDciTotalUsed = 0;
    m_ulDci = dciMsg->GetDciInfoElement();

    m_macRxedCtrlMsgsTrace(m_imsi, m_currentSlot, GetCellId(), m_rnti, GetBwpId(), dciMsg);

    NS_LOG_INFO("UL DCI received, transmit data in slot "
                << dataSfn << " Harq Process " << +m_ulDci->m_harqProcess << " TBS "
                << m_ulDci->m_tbSize << " total queue " << GetTotalBufSize());

    if (m_ulDci->m_sps == DciInfoElementTdma::PUSCH_REPETITION)
    {
        // PUSCH repetition: the transport block of the grant again, exactly as it was, with the
        // buffer status report it had (also when that was all it carried: unlike a HARQ
        // retransmission, a repetition comes before the gNB has decoded anything of the block)
        TransmitRetx();
        const auto sentBsr = m_harqBsr.find(m_ulDci->m_harqProcess);
        if (sentBsr != m_harqBsr.end())
        {
            m_phySapProvider->SendMacPdu(sentBsr->second->Copy(),
                                         dataSfn,
                                         m_ulDci->m_symStart,
                                         m_ulDci->m_rnti);
        }
    }
    else if (m_ulDci->m_ndi == 0)
    {
        // This method will retransmit the data saved in the harq buffer
        TransmitRetx();
        m_macUeStateMachine(m_imsi,
                            m_currentSlot,
                            GetCellId(),
                            m_rnti,
                            GetBwpId(),
                            m_srState,
                            m_ulBsrReceived,
                            m_ulDci->m_ndi,
                            "ProcessUlDci");

        // This method will transmit a new BSR. Not with a configured grant active: then a
        // retransmission is the transport block it was, with the buffer status report it had.
        // The gNB reads the reports to see what a block of the configuration left behind, and
        // a report of this instant would count the packet the next block is about to carry.
        const auto sentBsr = m_harqBsr.find(m_ulDci->m_harqProcess);
        if (!m_cgActive && !m_bsrOnDemand)
        {
            SendBufferStatusReport(dataSfn, m_ulDci->m_symStart);
        }
        else if (sentBsr != m_harqBsr.end())
        {
            m_phySapProvider->SendMacPdu(sentBsr->second->Copy(),
                                         dataSfn,
                                         m_ulDci->m_symStart,
                                         m_ulDci->m_rnti);
        }
    }
    else if (m_ulDci->m_ndi == 1)
    {
        // A grant for new data restarts retxBSR-Timer (TS 38.321 5.4.5)
        if (m_bsrOnDemand)
        {
            m_retxBsrStart = Simulator::Now();
        }
        m_bsrInTb = BsrTriggered();
        SendNewData();
        m_macUeStateMachine(m_imsi,
                            m_currentSlot,
                            GetCellId(),
                            m_rnti,
                            GetBwpId(),
                            m_srState,
                            m_ulBsrReceived,
                            m_ulDci->m_ndi,
                            "ProcessUlDci");

        NS_LOG_INFO("After sending NewData, bufSize " << GetTotalBufSize());

        // Send a new BSR. SendNewData() already took into account the size of
        // the BSR.
        if (m_bsrInTb)
        {
            SendBufferStatusReport(dataSfn, m_ulDci->m_symStart);
        }
        else
        {
            // this block has no report: a retransmission of it has none either
            m_harqBsr.erase(m_ulDci->m_harqProcess);
        }

        NS_LOG_INFO("UL DCI processing done, sent to PHY a total of "
                    << m_ulDciTotalUsed << " B out of " << m_ulDci->m_tbSize
                    << " allocated bytes ");
        NS_LOG_LOGIC("UE " << m_rnti << " grant kind " << +m_ulDci->m_sps << " of "
                           << GrantBytes() << " B for slot " << dataSfn.Normalize()
                           << ": buffer " << bufferBefore << " -> " << GetTotalBufSize()
                           << " B, sent " << m_ulDciTotalUsed << " B");
        static const bool ulTrace = std::getenv("NTN_ULTRACE") != nullptr;
        if (ulTrace)
        {
            std::cerr << "ULTRACE " << Simulator::Now().GetMicroSeconds() << " rnti " << m_rnti
                      << " kind " << +m_ulDci->m_sps << " grant " << GrantBytes() << " tb "
                      << m_ulDci->m_tbSize << " buf " << bufferBefore << "->" << GetTotalBufSize()
                      << " used " << m_ulDciTotalUsed << " bsr " << m_bsrInTb << " ahead "
                      << m_ulDci->m_cgConfirmAhead << " ce " << +m_ulDci->m_cgConfirmBytes
                      << std::endl;
        }

        if (GetTotalBufSize() == 0)
        {
            if (!m_bsrInTb)
            {
                // the block took everything without a report: the regular BSR is cancelled
                // (TS 38.321 5.4.5)
                m_regularBsrPending = false;
            }
            m_srState = INACTIVE;
            // The buffer was drained by a transport block that also carried the buffer status,
            // so the pending scheduling requests are cancelled and their sr-ProhibitTimer is
            // stopped (3GPP TS 38.321, clause 5.4.4): a request triggered by data arriving later
            // must not be held back by the request that this grant already answered, and it gets
            // the full sr-TransMax budget of its own.
            m_lastSrSent = Time::Min();
            m_srCounter = 0;
            NS_LOG_INFO("m_srState = ACTIVE -> INACTIVE, bufSize " << GetTotalBufSize());

            // the UE may have been scheduled, but we didn't use a single byte
            // of the allocation. So send an empty PDU. This happens because the
            // byte reporting in the BSR is not accurate, due to RLC and/or
            // BSR quantization.
            if (m_ulDciTotalUsed == 0)
            {
                NS_LOG_WARN("No byte used for this UL-DCI, sending empty PDU");

                NrMacSapProvider::TransmitPduParameters txParams;

                txParams.pdu = Create<Packet>();
                txParams.lcid = 3;
                txParams.rnti = m_rnti;
                txParams.layer = 0;
                txParams.harqProcessId = m_ulDci->m_harqProcess;
                txParams.componentCarrierId = GetBwpId();

                DoTransmitPdu(txParams);
            }
        }
    }
}

void
NrUeMac::TransmitRetx()
{
    NS_LOG_FUNCTION(this);

    Ptr<PacketBurst> pb = m_miUlHarqProcessesPacket.at(m_ulDci->m_harqProcess).m_pktBurst;

    if (pb == nullptr)
    {
        NS_LOG_WARN(
            "The previous transmission did not contain any new data; "
            "probably it was BSR only. To not send an old BSR to the scheduler, "
            "we don't send anything back in this allocation. Eventually, "
            "the Harq timer at gnb will expire, and soon this allocation will be forgotten.");
        return;
    }

    NS_LOG_DEBUG("UE MAC RETX HARQ " << +m_ulDci->m_harqProcess);

    NS_ASSERT(pb->GetNPackets() > 0);

    for (auto j = pb->Begin(); j != pb->End(); ++j)
    {
        Ptr<Packet> pkt = (*j)->Copy();
        NrRadioBearerTag bearerTag;
        if (!pkt->PeekPacketTag(bearerTag))
        {
            NS_FATAL_ERROR("No radio bearer tag");
        }
        m_phySapProvider->SendMacPdu(pkt, m_ulDciSfnsf, m_ulDci->m_symStart, m_ulDci->m_rnti);
    }

    m_miUlHarqProcessesPacketTimer.at(m_ulDci->m_harqProcess) = GetNumHarqProcess();
}

void
NrUeMac::SendRetxData(uint32_t usefulTbs, uint32_t activeLcsRetx)
{
    NS_LOG_FUNCTION(this);

    if (activeLcsRetx == 0)
    {
        NS_LOG_INFO("Send Retx called, but active LCS retransmitting is 0");
        return;
    }

    uint32_t bytesPerLcId = usefulTbs / activeLcsRetx;

    for (auto& itBsr : m_ulBsrReceived)
    {
        auto& bsr = itBsr.second;

        if (m_ulDciTotalUsed + bytesPerLcId <= usefulTbs)
        {
            NrMacSapUser::TxOpportunityParameters txParams;
            txParams.lcid = bsr.lcid;
            txParams.rnti = m_rnti;
            txParams.bytes = bytesPerLcId;
            txParams.layer = 0;
            txParams.harqId = m_ulDci->m_harqProcess;
            txParams.componentCarrierId = GetBwpId();

            NS_LOG_INFO("Notifying RLC of LCID " << +bsr.lcid
                                                 << " of a TxOpp "
                                                    "of "
                                                 << bytesPerLcId << " B for a RETX PDU");

            m_lcInfoMap.at(bsr.lcid).macSapUser->NotifyTxOpportunity(txParams);
            // After this call, m_ulDciTotalUsed has been updated with the
            // correct amount of bytes... but it is up to us in updating the BSR
            // value, subtracting the amount of bytes transmitted

            // We need to use std::min here because bytesPerLcId can be
            // greater than bsr.txQueueSize because scheduler can assign
            // more bytes than needed due to how TB size is computed.
            bsr.retxQueueSize -= std::min(bytesPerLcId, bsr.retxQueueSize);
        }
        else
        {
            NS_LOG_DEBUG("Something wrong with the calculation of overhead."
                         "Active LCS Retx: "
                         << activeLcsRetx << " assigned to this: " << bytesPerLcId
                         << ", with TBS of " << m_ulDci->m_tbSize << " usefulTbs " << usefulTbs
                         << " and total used " << m_ulDciTotalUsed);
        }
    }
}

void
NrUeMac::SendTxData(uint32_t usefulTbs, uint32_t activeTx)
{
    NS_LOG_FUNCTION(this);

    if (activeTx == 0)
    {
        NS_LOG_DEBUG("No active Tx flows for this UL-DCI");
        return;
    }

    constexpr uint8_t signalingLcid = 1;
    constexpr uint32_t minTxOpBytes = 7;
    constexpr uint32_t rlcAmWorstCaseHeaderBytes = 4;
    constexpr uint32_t rlcHeaderSlackBytes = 4;

    // Apply shortest-job first policy to prioritize lcids with less data
    while (m_ulDciTotalUsed < usefulTbs)
    {
        uint32_t availableBytes = usefulTbs - m_ulDciTotalUsed;
        if (availableBytes < minTxOpBytes)
        {
            NS_LOG_WARN("Not enough bytes available to send a TxPDU, skipping");
            break;
        }

        uint32_t smallestBufferBytes = std::numeric_limits<uint32_t>::max();
        uint8_t smallestBufferBsrLcid = std::numeric_limits<uint8_t>::max();
        auto signalingIt = m_ulBsrReceived.find(signalingLcid);
        if (signalingIt != m_ulBsrReceived.end() && signalingIt->second.txQueueSize > 0)
        {
            // Prioritize LCID 1 (signaling) when it has pending data.
            // We assume a worst-case RLC AM header of 4 bytes to reduce
            // fragmentation of small signaling SDUs.
            // Example: a 4-byte signaling SDU may need 8 bytes TxOpportunity
            // (4 payload + 4 AM header) to be sent in one shot.
            smallestBufferBsrLcid = signalingLcid;
            smallestBufferBytes = signalingIt->second.txQueueSize + rlcAmWorstCaseHeaderBytes;
            NS_LOG_DEBUG("LCID " << +signalingLcid
                                 << " has signaling priority, selecting it first");
        }
        else
        {
            for (auto& itBsr : m_ulBsrReceived)
            {
                const auto& bsr = itBsr.second;
                // Skip lcid with empty queue
                if (bsr.txQueueSize == 0)
                {
                    NS_LOG_DEBUG("LCID " << +bsr.lcid << "has no data to send, skipping");
                    continue;
                }
                if (bsr.txQueueSize < smallestBufferBytes)
                {
                    NS_LOG_DEBUG("LCID " << +bsr.lcid
                                         << " has less data to send so far, prioritizing it");
                    smallestBufferBsrLcid = bsr.lcid;
                    smallestBufferBytes = bsr.txQueueSize;
                }
            }
        }

        if (smallestBufferBytes == std::numeric_limits<uint32_t>::max())
        {
            NS_LOG_INFO("No LCID left to offer transmission opportunity, even though there are "
                        "bytes available.");
            break;
        }

        // We need to allocate at least 7 bytes per LCID due to RLC limitations
        // But we can allocate up to availableBytes
        // And a few bytes more than the buffer is said to hold, if the grant has them: after
        // a PDU that took a part of the buffer, the size kept here is what the RLC reported
        // less the bytes it was offered, which is short of what the RLC needs by the header of
        // the PDU still to come. Offered just that, the RLC leaves the last byte behind while
        // the buffer counts as empty here (and in the buffer status report), until a new
        // packet or the RLC's timer says otherwise. The RLC takes no more than it needs.
        uint32_t bytesPerLcId =
            std::min(availableBytes,
                     std::max<uint32_t>(smallestBufferBytes + rlcHeaderSlackBytes, minTxOpBytes));

        auto& bsr = m_ulBsrReceived.at(smallestBufferBsrLcid);
        NS_LOG_DEBUG("LCID " << +bsr.lcid << " assigned a TxOp of " << bytesPerLcId << "/"
                             << bsr.txQueueSize << ", from the pool of " << availableBytes
                             << "B available");

        NrMacSapUser::TxOpportunityParameters txParams;
        txParams.lcid = bsr.lcid;
        txParams.rnti = m_rnti;
        txParams.bytes = bytesPerLcId;
        txParams.layer = 0;
        txParams.harqId = m_ulDci->m_harqProcess;
        txParams.componentCarrierId = GetBwpId();

        NS_LOG_INFO("Notifying RLC of LCID " << +bsr.lcid
                                             << " of a TxOpp "
                                                "of "
                                             << bytesPerLcId << " B for a TX PDU");
        m_lcInfoMap.at(bsr.lcid).macSapUser->NotifyTxOpportunity(txParams);
        // After this call, m_ulDciTotalUsed has been updated with the
        // correct amount of bytes... but it is up to us in updating the BSR
        // value, subtracting the amount of bytes transmitted

        // We need to use std::min here because bytesPerLcId can be
        // greater than bsr.txQueueSize because scheduler can assign
        // more bytes than needed due to how TB size is computed.
        bsr.txQueueSize -= std::min(bytesPerLcId, bsr.txQueueSize);
    }
}

void
NrUeMac::SendNewData()
{
    NS_LOG_FUNCTION(this);
    // New transmission -> empty pkt buffer queue (for deleting eventual pkts not acked )
    Ptr<PacketBurst> pb = CreateObject<PacketBurst>();
    m_miUlHarqProcessesPacket.at(m_ulDci->m_harqProcess).m_pktBurst = pb;
    m_miUlHarqProcessesPacket.at(m_ulDci->m_harqProcess).m_lcidList.clear();
    NS_LOG_INFO("Reset HARQP " << +m_ulDci->m_harqProcess);

    // Sending the status data has no boundary: let's try to send the ACK as
    // soon as possible, filling the TBS, if necessary.
    SendNewStatusData();

    // Let's count how many LC we have, that are waiting with some data
    uint16_t activeLcsRetx = 0;
    uint16_t activeLcsTx = 0;
    for (const auto& itBsr : m_ulBsrReceived)
    {
        if (itBsr.second.retxQueueSize > 0)
        {
            activeLcsRetx++;
        }
        if (itBsr.second.txQueueSize > 0)
        {
            activeLcsTx++;
        }
    }

    // Of the TBS we received in the DCI, one part is gone for the status pdu,
    // where we didn't check much as it is the most important data, that has to go
    // out. For the rest that we have left, we can use only a part of it because of
    // the overhead of the SHORT_BSR, which is 5 bytes.
    const uint32_t grantBytes = GrantBytes();
    uint32_t bsrBytes = m_bsrInTb ? NrMacShortBsrCe::OverheadBytes() : 0;
    if (m_cgBsrWhenRoom && m_cgConfirmBytes > 0 && bsrBytes > 0 &&
        m_ulDciTotalUsed + bsrBytes + m_cgConfirmBytes > grantBytes &&
        m_ulDciTotalUsed + m_cgConfirmBytes <= grantBytes)
    {
        // the confirmation comes before the report (TS 38.321 5.4.3.1.3)
        m_bsrInTb = false;
        bsrBytes = 0;
    }
    NS_ASSERT_MSG(m_ulDciTotalUsed + bsrBytes <= grantBytes,
                  "The StatusPDU used " << m_ulDciTotalUsed
                                        << " B, we don't have any for the SHORT_BSR.");
    // The MAC CE that confirms the activation or the release of a configured grant comes
    // before the data (TS 38.321 5.4.3.1.3): its bytes are not there for the logical channels
    uint32_t confirmBytes = 0;
    if (m_cgConfirmBytes > 0 && m_ulDciTotalUsed + bsrBytes + m_cgConfirmBytes <= grantBytes)
    {
        confirmBytes = m_cgConfirmBytes;
        m_cgConfirmBytes = 0;
        if (const Ptr<NrPdcch> pdcch = NrPdcch::Find(GetCellId()))
        {
            pdcch->NoteEvent(NrPdcch::UL_CG_CONFIRMATION);
            pdcch->NoteEvent(NrPdcch::UL_CG_CONFIRMATION_BYTES, confirmBytes);
        }
    }
    // reserve some data for the SHORT_BSR
    uint32_t usefulTbs = grantBytes - m_ulDciTotalUsed - bsrBytes - confirmBytes;

    // Now, we have 3 bytes of overhead for each subPDU. Let's try to serve all
    // the queues with some RETX data.
    if (activeLcsRetx == 0 && activeLcsTx == 0 && usefulTbs > 0)
    {
        NS_LOG_LOGIC("This UE tx opportunity will be wasted: " << usefulTbs << " bytes.");
    }

    // this check is needed, because if there are no active LCS we should not
    // enter into else and call the function SendRetxData
    if (activeLcsRetx > 0 && usefulTbs > 0) // the queues with some RETX data.
    {
        // 10 because 3 bytes will go for MAC subheader
        // and we should ensure to pass to RLC AM at least 7 bytes
        if (activeLcsRetx * 10 > usefulTbs)
        {
            NS_LOG_DEBUG("The overhead for transmitting retx data is greater than the space for "
                         "transmitting it."
                         "Ignore the TBS of "
                         << usefulTbs << " B.");
        }
        else
        {
            usefulTbs -= activeLcsRetx * NrMacHeaderVs::SubheaderBytes(usefulTbs);
            SendRetxData(usefulTbs, activeLcsRetx);
        }
    }

    // Now we have to update our useful TBS for the next transmission.
    // Remember that m_ulDciTotalUsed keep count of data and overhead that we
    // used till now.
    NS_ASSERT_MSG(
        m_ulDciTotalUsed + bsrBytes + confirmBytes <= grantBytes,
        "The StatusPDU and RETX sending required all space, we don't have any for the SHORT_BSR.");
    usefulTbs = grantBytes - m_ulDciTotalUsed - bsrBytes - confirmBytes; // Update the usefulTbs.

    // The last part is for the queues with some non-RETX data. If there is no space left,
    // then nothing.
    if (activeLcsTx > 0 && usefulTbs > 0) // the queues with some TX data.
    {
        // 10 because 3 bytes will go for MAC subheader
        // and we should ensure to pass to RLC AM at least 7 bytes
        if (activeLcsTx * 10 > usefulTbs)
        {
            NS_LOG_DEBUG("The overhead for transmitting new data is greater than the space for "
                         "transmitting it."
                         "Ignore the TBS of "
                         << usefulTbs << " B.");
        }
        else
        {
            usefulTbs -= activeLcsTx * NrMacHeaderVs::SubheaderBytes(usefulTbs);
            SendTxData(usefulTbs, activeLcsTx);
        }
    }

    // If we did not used the packet burst, explicitly signal it to the HARQ
    // retx, if any.
    if (m_ulDciTotalUsed == 0)
    {
        m_miUlHarqProcessesPacket.at(m_ulDci->m_harqProcess).m_pktBurst = nullptr;
        m_miUlHarqProcessesPacket.at(m_ulDci->m_harqProcess).m_lcidList.clear();
    }
}

void
NrUeMac::SendNewStatusData()
{
    NS_LOG_FUNCTION(this);

    bool hasStatusPdu = false;
    bool sentOneStatusPdu = false;

    for (auto& bsrIt : m_ulBsrReceived)
    {
        auto& bsr = bsrIt.second;

        if (bsr.statusPduSize > 0)
        {
            hasStatusPdu = true;

            // Check if we have room to transmit the statusPdu
            if (m_ulDciTotalUsed + bsr.statusPduSize <= GrantBytes())
            {
                NrMacSapUser::TxOpportunityParameters txParams;
                txParams.lcid = bsr.lcid;
                txParams.rnti = m_rnti;
                txParams.bytes = bsr.statusPduSize;
                txParams.layer = 0;
                txParams.harqId = m_ulDci->m_harqProcess;
                txParams.componentCarrierId = GetBwpId();

                NS_LOG_INFO("Notifying RLC of LCID " << +bsr.lcid
                                                     << " of a TxOpp "
                                                        "of "
                                                     << bsr.statusPduSize << " B for a status PDU");

                m_lcInfoMap.at(bsr.lcid).macSapUser->NotifyTxOpportunity(txParams);
                // After this call, m_ulDciTotalUsed has been updated with the
                // correct amount of bytes... but it is up to us in updating the BSR
                // value, subtracting the amount of bytes transmitted
                bsr.statusPduSize = 0;
                sentOneStatusPdu = true;
            }
            else
            {
                NS_LOG_WARN("Cannot send StatusPdu of " << bsr.statusPduSize
                                                        << " B, we already used all the TBS");
            }
        }
    }

    NS_ABORT_MSG_IF(hasStatusPdu && !sentOneStatusPdu,
                    "The TBS of size " << m_ulDci->m_tbSize
                                       << " doesn't allow us "
                                          "to send one status PDU...");
}

void
NrUeMac::DoReceiveControlMessage(Ptr<NrControlMessage> msg)
{
    NS_LOG_FUNCTION(this << msg);

    switch (msg->GetMessageType())
    {
    case (NrControlMessage::UL_DCI): {
        ProcessUlDci(DynamicCast<NrUlDciMessage>(msg));
        break;
    }
    case (NrControlMessage::RAR): {
        NS_LOG_INFO("Received RAR in slot " << m_currentSlot);

        m_macRxedCtrlMsgsTrace(m_imsi, m_currentSlot, GetCellId(), m_rnti, GetBwpId(), msg);

        if (m_waitingForRaResponse)
        {
            Ptr<NrRarMessage> rarMsg = DynamicCast<NrRarMessage>(msg);
            NS_LOG_LOGIC("got RAR with RA-RNTI " << +rarMsg->GetRaRnti() << ", expecting "
                                                 << +m_raRnti);
            for (auto it = rarMsg->RarListBegin(); it != rarMsg->RarListEnd(); ++it)
            {
                if (it->rarPayload.raPreambleId == m_raPreambleId)
                {
                    RecvRaResponse(it->rarPayload);
                }
            }
        }
        break;
    }

    default:
        NS_LOG_LOGIC("Control message not supported/expected: " << msg->GetMessageType());
    }
}

NrUePhySapUser*
NrUeMac::GetPhySapUser()
{
    return m_phySapUser;
}

void
NrUeMac::SetPhySapProvider(NrPhySapProvider* ptr)
{
    m_phySapProvider = ptr;
}

void
NrUeMac::RaResponseTimeout(bool contention)
{
    NS_LOG_FUNCTION(this << contention);
    m_waitingForRaResponse = false;
    // 3GPP 36.321 5.1.4
    ++m_preambleTransmissionCounter;
    // fire RA response timeout trace
    m_raResponseTimeoutTrace(m_imsi,
                             contention,
                             m_preambleTransmissionCounter,
                             m_rachConfig.preambleTransMax + 1);
    if (m_preambleTransmissionCounter == m_rachConfig.preambleTransMax + 1)
    {
        NS_LOG_INFO("RAR timeout, preambleTransMax reached => giving up");
        m_cmacSapUser->NotifyRandomAccessFailed();
    }
    else
    {
        NS_LOG_INFO("RAR timeout while waiting for raPreambleId " << +m_raPreambleId
                                                                  << " re-send preamble");
        if (contention)
        {
            RandomlySelectAndSendRaPreamble();
        }
        else
        {
            SendRaPreamble(contention);
        }
    }
}

void
NrUeMac::DoConfigureRach(NrUeCmacSapProvider::RachConfig rc)
{
    NS_LOG_FUNCTION(this);
    m_rachConfig = rc;
    m_rachConfigured = true;
}

NrUeCmacSapProvider::RachConfig
NrUeMac::GetRachConfig() const
{
    return m_rachConfig;
}

void
NrUeMac::DoStartContentionBasedRandomAccessProcedure()
{
    NS_LOG_FUNCTION(this);
    RandomlySelectAndSendRaPreamble();
}

void
NrUeMac::RandomlySelectAndSendRaPreamble()
{
    NS_LOG_FUNCTION(this);
    // 3GPP 36.321 5.1.1
    NS_ASSERT_MSG(m_rachConfigured, "RACH not configured " << ", cellId " << GetCellId());
    // assume that there is no Random Access Preambles group B
    m_raPreambleId =
        m_raPreambleUniformVariable->GetInteger(0, m_rachConfig.numberOfRaPreambles - 1);
    NS_LOG_DEBUG(m_currentSlot << " Received System Information, send to PHY the "
                                  "RA preamble: "
                               << +m_raPreambleId << ", cellId " << GetCellId());
    SendRaPreamble(true);
}

void
NrUeMac::SendRaPreamble(bool contention)
{
    NS_LOG_FUNCTION(this << (uint32_t)m_raPreambleId << contention << ", cellId " << GetCellId());

    if (contention)
    {
        // m_raPreambleId = m_raPreambleUniformVariable->GetInteger (0, 64 - 1);
        m_raPreambleId = g_raPreambleId++;
        bool preambleOverflow = m_raPreambleId == 255;
        m_raPreambleId += preambleOverflow;
        g_raPreambleId += preambleOverflow;
    }
    /*raRnti should be subframeNo -1 */
    m_raRnti = 1;

    // 3GPP 36.321 5.1.4
    m_phySapProvider->SendRachPreamble(m_raPreambleId, m_raRnti);
    NS_LOG_INFO(" Sent preamble id " << +m_raPreambleId
                                     << " at: " << Simulator::Now().As(Time::MS));
    Time raWindowBegin = m_phySapProvider->GetSlotPeriod();
    Simulator::Schedule(raWindowBegin, &NrUeMac::StartWaitingForRaResponse, this);
    if (m_idealRandomAccess)
    {
        NS_LOG_INFO("Ideal random access: the RA response timeout is not armed");
    }
    else
    {
        // ra-ResponseWindow, shifted by the UE-gNB round-trip time (TS 38.213 clause 8 for a
        // cell with NTN-Config): the preamble needs one TimingAdvance to reach the gNB and the
        // RAR one more to come back. The window is only extended, never started later, so an
        // early RAR is never rejected.
        Time raWindowEnd =
            m_phySapProvider->GetSlotPeriod() * (6 + m_rachConfig.raResponseWindowSize) +
            2 * m_phySapProvider->GetTimingAdvance();
        m_noRaResponseReceivedEvent =
            Simulator::Schedule(raWindowEnd, &NrUeMac::RaResponseTimeout, this, contention);
    }

    // Tracing purposes
    Ptr<NrRachPreambleMessage> rachMsg = Create<NrRachPreambleMessage>();
    rachMsg->SetSourceBwpArfcn(m_phySapProvider->GetArfcn());
    m_macTxedCtrlMsgsTrace(m_imsi, m_currentSlot, GetCellId(), m_rnti, GetBwpId(), rachMsg);
}

void
NrUeMac::StartWaitingForRaResponse()
{
    NS_LOG_FUNCTION(this);
    m_waitingForRaResponse = true;
}

void
NrUeMac::DoStartNonContentionBasedRandomAccessProcedure(uint16_t rnti,
                                                        [[maybe_unused]] uint8_t preambleId,
                                                        uint8_t prachMask)
{
    NS_LOG_FUNCTION(this << rnti << (uint16_t)preambleId << (uint16_t)prachMask);
    NS_ASSERT_MSG(prachMask == 0,
                  "requested PRACH MASK = " << (uint32_t)prachMask
                                            << ", but only PRACH MASK = 0 is supported");
    m_rnti = rnti;
    m_raPreambleId = preambleId;
    m_preambleTransmissionCounter = 0;
    bool contention = false;
    SendRaPreamble(contention);
}

void
NrUeMac::AddLc(uint8_t lcId, NrUeCmacSapProvider::LogicalChannelConfig lcConfig, NrMacSapUser* msu)
{
    NS_LOG_FUNCTION(this << " lcId" << (uint32_t)lcId);
    NS_ASSERT_MSG(m_lcInfoMap.find(lcId) == m_lcInfoMap.end(),
                  "cannot add channel because LCID " << (uint16_t)lcId << " is already present");

    LcInfo lcInfo;
    lcInfo.lcConfig = lcConfig;
    lcInfo.macSapUser = msu;
    m_lcInfoMap[lcId] = lcInfo;
}

void
NrUeMac::DoRemoveLc(uint8_t lcId)
{
    NS_LOG_FUNCTION(this << " lcId" << lcId);
    NS_ASSERT_MSG(m_lcInfoMap.find(lcId) != m_lcInfoMap.end(),
                  "could not find LCID " << (uint16_t)lcId);
    m_lcInfoMap.erase(lcId);
    m_ulBsrReceived.erase(lcId); // empty BSR buffer for this lcId
    // for (auto& harqProcess: m_miUlHarqProcessesPacket)
    //{
    //     uint32_t packets = harqProcess.m_pktBurst->GetNPackets();
    //     auto itPackets = harqProcess.m_pktBurst->GetPackets();
    //     for (uint32_t p = 0; p < packets; p++)
    //     {
    //         uint32_t i = packets-p-1;
    //         if (harqProcess.m_lcidList.at(i) == lcId)
    //         {
    //             // how to erase from packetburst?
    //             //auto it = itPackets.begin();
    //             //std::advance(it, i);
    //             //harqProcess.m_pktBurst->GetPackets().erase(it);
    //             auto it2 = harqProcess.m_lcidList.begin();
    //             std::advance(it2, i);
    //             harqProcess.m_lcidList.erase(it2);
    //         }
    //     }
    // }
}

NrMacSapProvider*
NrUeMac::GetUeMacSapProvider()
{
    return m_macSapProvider;
}

void
NrUeMac::DoReset()
{
    NS_LOG_FUNCTION(this);
    auto it = m_lcInfoMap.begin();
    while (it != m_lcInfoMap.end())
    {
        // don't delete CCCH)
        if (it->first == 0)
        {
            ++it;
        }
        else
        {
            // note: use of postfix operator preserves validity of iterator
            m_lcInfoMap.erase(it++);
        }
    }
    // note: rnti will be assigned by the gNB using RA response message
    m_rnti = 0;
    m_noRaResponseReceivedEvent.Cancel();
    m_rachConfigured = false;
    m_ulBsrReceived.clear();
    m_srState = INACTIVE;
    m_lastSrSent = Time::Min();
    m_srCounter = 0;
    m_cgConfirmBytes = 0;
    m_cgAheadSerial = 0;
    m_cgLastActivationSerial = 0;
    m_cgActive = false;
    m_cgBlocks.clear();
    m_harqBsr.clear();
    m_regularBsrPending = false;
    m_lastBsrSent = Time::Min();
    m_retxBsrStart = Time::Min();
}

//////////////////////////////////////////////

int64_t
NrUeMac::AssignStreams(int64_t stream)
{
    return DoAssignStreams(stream);
}

int64_t
NrUeMac::DoAssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    m_raPreambleUniformVariable->SetStream(stream);
    return 1;
}

} // namespace ns3
