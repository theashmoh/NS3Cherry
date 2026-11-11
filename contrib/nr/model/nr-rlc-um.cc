// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>

#include "nr-rlc-um.h"

#include "nr-pdcp.h"
#include "nr-rlc-header.h"
#include "nr-rlc-sdu-status-tag.h"
#include "nr-rlc-tag.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrRlcUm");

NS_OBJECT_ENSURE_REGISTERED(NrRlcUm);

NrRlcUm::NrRlcUm()
    : m_maxTxBufferSize(10 * 1024),
      m_txBufferSize(0),
      m_sequenceNumber(0),
      m_vrUr(0),
      m_vrUx(0),
      m_vrUh(0),
      m_windowSize(512),
      m_expectedSeqNumber(0)
{
    NS_LOG_FUNCTION(this);
    m_reassemblingState = WAITING_S0_FULL;
}

NrRlcUm::~NrRlcUm()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrRlcUm::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrRlcUm")
            .SetParent<NrRlc>()
            .SetGroupName("Nr")
            .AddConstructor<NrRlcUm>()
            .AddAttribute("MaxTxBufferSize",
                          "Maximum Size of the Transmission Buffer (in Bytes). "
                          "If set to 0, the buffer is unlimited.",
                          UintegerValue(10 * 1024),
                          MakeUintegerAccessor(&NrRlcUm::m_maxTxBufferSize),
                          MakeUintegerChecker<uint32_t>())
            // sn-FieldLength of TS 36.331; header and window as in TS 36.322 6.2.1.3 and 7.2.
            // Both ends of a bearer need the same value.
            .AddAttribute("SegmentOffsetBytes",
                          "Bytes added to a PDU whose data starts inside an SDU: 2 = NR's 16-bit "
                          "SO (TS 38.322 6.2.2.3); 0 = E-UTRA framing (v9)",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrRlcUm::m_segmentOffsetBytes),
                          MakeUintegerChecker<uint32_t>(0, 4))
            .AddAttribute("SnFieldLength",
                          "UM sequence number bits: 10 (2-octet header, window 512) or 5 (1 "
                          "octet, window 16)",
                          UintegerValue(10),
                          MakeUintegerAccessor(&NrRlcUm::SetSnFieldLength,
                                               &NrRlcUm::GetSnFieldLength),
                          MakeUintegerChecker<uint8_t>(5, 10))
            .AddAttribute("ReorderingTimer",
                          "Value of the t-Reordering timer (See section 7.3 of 3GPP TS 36.322)",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&NrRlcUm::m_reorderingTimerValue),
                          MakeTimeChecker())
            .AddAttribute(
                "EnablePdcpDiscarding",
                "Whether to use the PDCP discarding, i.e., perform discarding at the moment "
                "of passing the PDCP SDU to RLC)",
                BooleanValue(true),
                MakeBooleanAccessor(&NrRlcUm::m_enablePdcpDiscarding),
                MakeBooleanChecker())
            .AddAttribute("DiscardTimerMs",
                          "Discard timer in milliseconds to be used to discard packets. "
                          "If set to 0 then packet delay budget will be used as the discard "
                          "timer value, otherwise it will be used this value.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrRlcUm::m_discardTimerMs),
                          MakeUintegerChecker<uint32_t>())
            // The part of the packet delay budget a packet has used up when it reaches this RLC, in ms:
            // the delay between UPF and gNB. Where the delay budget is the discard timer (DiscardTimerMs =
            // 0, and the budget is known: at the gNB), the timer is the budget less this, in whole ms.
            .AddAttribute("DelayBudgetSpentMs",
                          "Part of the packet delay budget already used when a packet reaches this RLC (UPF to gNB), ms; the discard timer is the budget less this.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrRlcUm::m_delayBudgetSpentMs),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("UeTxStaleMs",
                          "UE side: an SDU that has waited this long in the transmit buffer is "
                          "dropped, the oldest first; 0 = never",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrRlcUm::m_ueTxStaleMs),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("UeSide",
                          "Set by the UE's RRC on the entities it creates",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrRlcUm::m_ueSide),
                          MakeBooleanChecker())
            .AddAttribute("UeCellId",
                          "Set by the UE's RRC on the entities it creates: the cell of the UE, "
                          "for the counts of packets by bearer (two cells may give the same RNTI)",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrRlcUm::m_ueCellId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("BufferStatusReportTimer",
                          "How much to wait to issue a new Buffer Status Report while the "
                          "transmission buffer is not empty",
                          TimeValue(MilliSeconds(10)),
                          MakeTimeAccessor(&NrRlcUm::m_bsrTimerValue),
                          MakeTimeChecker())
            .AddAttribute("OutOfOrderDelivery",
                          "Whether to deliver RLC SDUs out of order without waiting for a "
                          "reordering timer to expire",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrRlcUm::m_outOfOrderDelivery),
                          MakeBooleanChecker());
    return tid;
}

void
NrRlcUm::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_reorderingTimer.Cancel();
    m_bsrTimer.Cancel();

    NrRlc::DoDispose();
}

/**
 * RLC SAP
 */

void
NrRlcUm::DiscardStale()
{
    if (!m_ueSide || m_ueTxStaleMs == 0)
    {
        return;
    }
    const Time limit = MilliSeconds(m_ueTxStaleMs);
    while (!m_txBuffer.empty() && Simulator::Now() - m_txBuffer.front().m_waitingSince > limit)
    {
        NS_LOG_INFO("Tx SDU waited over " << m_ueTxStaleMs << " ms in the UE. RLC SDU discarded");
        m_txBufferSize -= m_txBuffer.front().m_pdu->GetSize();
        m_txDropTrace(m_txBuffer.front().m_pdu);
        NrPdcp::NoteUeStaleDiscard(m_ueCellId, m_rnti, m_lcid); // for NrPdcp::GetPacketCounts()
        m_txBuffer.pop_front();
    }
}

void
NrRlcUm::DoTransmitPdcpPdu(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid << p->GetSize());
    DiscardStale();
    if ((m_txBufferSize + p->GetSize() <= m_maxTxBufferSize) || (m_maxTxBufferSize == 0))
    {
        if (m_enablePdcpDiscarding)
        {
            // discard the packet
            uint32_t headOfLineDelayInMs = 0;
            uint32_t discardTimerMs =
                (m_discardTimerMs > 0) ? m_discardTimerMs : m_packetDelayBudgetMs;
            if (m_discardTimerMs == 0 && m_packetDelayBudgetMs != UINT16_MAX)
            {
                // The budget is between UPF and UE: what the packet needed to get here is gone
                discardTimerMs = static_cast<uint32_t>(
                    std::floor(std::max(0.0, m_packetDelayBudgetMs - m_delayBudgetSpentMs)));
            }

            if (!m_txBuffer.empty())
            {
                headOfLineDelayInMs =
                    (Simulator::Now() - m_txBuffer.begin()->m_waitingSince).GetMilliSeconds();
            }
            NS_LOG_DEBUG("head of line delay in MS:" << headOfLineDelayInMs);
            if (headOfLineDelayInMs > discardTimerMs)
            {
                NS_LOG_INFO("Tx HOL is higher than this packet can allow. RLC SDU discarded");
                NS_LOG_DEBUG("headOfLineDelayInMs    = " << headOfLineDelayInMs);
                NS_LOG_DEBUG("m_packetDelayBudgetMs    = " << m_packetDelayBudgetMs);
                NS_LOG_DEBUG("packet size     = " << p->GetSize());
                m_txDropTrace(p);
                NrPdcp::NotePduDiscarded(); // for NrPdcp::GetPacketCounts()

                // The packet has been discarded, so do not store it in the Tx buffer.
                // A Buffer Status Report is still issued below to reflect the current state.
                // A running report timer is left alone: new data arriving is what triggers a
                // buffer status report, never a reason to stop reporting.
                DoTransmitBufferStatusReport();
                return;
            }
        }

        /** Store PDCP PDU */
        NrRlcSduStatusTag tag;
        tag.SetStatus(NrRlcSduStatusTag::FULL_SDU);
        p->AddPacketTag(tag);
        NS_LOG_INFO("Adding RLC SDU to Tx Buffer after adding NrRlcSduStatusTag: FULL_SDU");
        m_txBuffer.emplace_back(p, Simulator::Now());
        m_txBufferSize += p->GetSize();
        NS_LOG_LOGIC("NumOfBuffers = " << m_txBuffer.size());
        NS_LOG_LOGIC("txBufferSize = " << m_txBufferSize);
    }
    else
    {
        // Discard full RLC SDU
        NS_LOG_INFO("Tx Buffer is full. RLC SDU discarded");
        NS_LOG_LOGIC("MaxTxBufferSize = " << m_maxTxBufferSize);
        NS_LOG_LOGIC("txBufferSize    = " << m_txBufferSize);
        NS_LOG_LOGIC("packet size     = " << p->GetSize());
        m_txDropTrace(p);
        NrPdcp::NotePduDiscarded(); // for NrPdcp::GetPacketCounts()
    }

    /** Transmit Buffer Status Report */
    // A running report timer is left alone: new data arriving is what triggers a buffer status
    // report (3GPP TS 38.321, clause 5.4.5), never a reason to stop reporting. Cancelling it here
    // used to strand the uplink, as the timer is only armed again by a transmission opportunity:
    // once the uplink stalled, the last cancellation was final and the buffer status went stale.
    DoTransmitBufferStatusReport();
}

void
NrRlcUm::SetSnFieldLength(uint8_t bits)
{
    NS_ABORT_MSG_UNLESS(bits == 5 || bits == 10, "NrRlcUm::SnFieldLength must be 5 or 10");
    m_snFieldLength = bits;
    const uint16_t modulus = 1 << bits;
    m_windowSize = modulus / 2; // UM_Window_Size, TS 36.322 7.2: 16 or 512
    m_sequenceNumber.SetModulus(modulus);
    m_vrUr.SetModulus(modulus);
    m_vrUx.SetModulus(modulus);
    m_vrUh.SetModulus(modulus);
    m_expectedSeqNumber.SetModulus(modulus);
}

uint8_t
NrRlcUm::GetSnFieldLength() const
{
    return m_snFieldLength;
}

nr::SequenceNumber10
NrRlcUm::NewSn(uint16_t value) const
{
    nr::SequenceNumber10 sn(value);
    sn.SetModulus(1 << m_snFieldLength);
    return sn;
}

uint32_t
NrRlcUm::FixedHeaderSize() const
{
    return m_snFieldLength == 5 ? 1 : 2;
}

/**
 * MAC SAP
 */

void
NrRlcUm::DoNotifyTxOpportunity(NrMacSapUser::TxOpportunityParameters txOpParams)
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid << txOpParams.bytes);
    NS_LOG_INFO("RLC layer is preparing data for the following Tx opportunity of "
                << txOpParams.bytes << " bytes for RNTI=" << m_rnti << ", LCID=" << (uint32_t)m_lcid
                << ", CCID=" << (uint32_t)txOpParams.componentCarrierId << ", HARQ ID="
                << (uint32_t)txOpParams.harqId << ", MIMO Layer=" << (uint32_t)txOpParams.layer);

    if (txOpParams.bytes <= FixedHeaderSize())
    {
        // Stingy MAC: Header fix part is 2 bytes (1 with a 5-bit SN), we need more bytes for
        // the data
        NS_LOG_INFO("TX opportunity too small - Only " << txOpParams.bytes << " bytes");
        return;
    }

    Ptr<Packet> packet = Create<Packet>();
    NrRlcHeader rlcHeader;
    rlcHeader.SetSnFieldLength(m_snFieldLength);

    // Build Data field
    uint32_t nextSegmentSize = txOpParams.bytes - FixedHeaderSize();
    uint32_t nextSegmentId = 1;
    uint32_t dataFieldAddedSize = 0;
    std::vector<Ptr<Packet>> dataField;

    // Remove the first packet from the transmission buffer.
    // If only a segment of the packet is taken, then the remaining is given back later
    DiscardStale();
    if (m_txBuffer.empty())
    {
        NS_LOG_LOGIC("No data pending");
        return;
    }

    Ptr<Packet> firstSegment = m_txBuffer.begin()->m_pdu->Copy();
    Time firstSegmentTime = m_txBuffer.begin()->m_waitingSince;
    // NR (SegmentOffsetBytes): a PDU that starts inside an SDU carries the segment offset
    bool continues = false;
    if (m_segmentOffsetBytes > 0)
    {
        NrRlcSduStatusTag headTag;
        if (firstSegment->PeekPacketTag(headTag) &&
            (headTag.GetStatus() == NrRlcSduStatusTag::MIDDLE_SEGMENT ||
             headTag.GetStatus() == NrRlcSduStatusTag::LAST_SEGMENT))
        {
            continues = true;
            if (nextSegmentSize <= m_segmentOffsetBytes)
            {
                NS_LOG_INFO("TX opportunity too small for the segment offset");
                return;
            }
            nextSegmentSize -= m_segmentOffsetBytes;
        }
    }

    NS_LOG_LOGIC("SDUs in TxBuffer  = " << m_txBuffer.size());
    NS_LOG_LOGIC("First SDU buffer  = " << firstSegment);
    NS_LOG_LOGIC("First SDU size    = " << firstSegment->GetSize());
    NS_LOG_LOGIC("Next segment size = " << nextSegmentSize);
    NS_LOG_LOGIC("Remove SDU from TxBuffer");
    m_txBufferSize -= firstSegment->GetSize();
    NS_LOG_LOGIC("txBufferSize      = " << m_txBufferSize);
    m_txBuffer.erase(m_txBuffer.begin());

    while (firstSegment && (firstSegment->GetSize() > 0) && (nextSegmentSize > 0))
    {
        NS_LOG_LOGIC("WHILE ( firstSegment && firstSegment->GetSize > 0 && nextSegmentSize > 0 )");
        NS_LOG_LOGIC("    firstSegment size = " << firstSegment->GetSize());
        NS_LOG_LOGIC("    nextSegmentSize   = " << nextSegmentSize);
        if ((firstSegment->GetSize() > nextSegmentSize) ||
            // Segment larger than 2047 octets can only be mapped to the end of the Data field
            (firstSegment->GetSize() > 2047))
        {
            // Take the minimum size, due to the 2047-bytes 3GPP exception
            // This exception is due to the length of the LI field (just 11 bits)
            uint32_t currSegmentSize = std::min(firstSegment->GetSize(), nextSegmentSize);

            NS_LOG_LOGIC("    IF ( firstSegment > nextSegmentSize ||");
            NS_LOG_LOGIC("         firstSegment > 2047 )");

            // Segment txBuffer.FirstBuffer and
            // Give back the remaining segment to the transmission buffer
            Ptr<Packet> newSegment = firstSegment->CreateFragment(0, currSegmentSize);
            NS_LOG_LOGIC("    newSegment size   = " << newSegment->GetSize());

            // Status tag of the new and remaining segments
            // Note: This is the only place where a PDU is segmented and
            // therefore its status can change
            NrRlcSduStatusTag oldTag;
            NrRlcSduStatusTag newTag;
            firstSegment->RemovePacketTag(oldTag);
            newSegment->RemovePacketTag(newTag);
            if (oldTag.GetStatus() == NrRlcSduStatusTag::FULL_SDU)
            {
                newTag.SetStatus(NrRlcSduStatusTag::FIRST_SEGMENT);
                oldTag.SetStatus(NrRlcSduStatusTag::LAST_SEGMENT);
            }
            else if (oldTag.GetStatus() == NrRlcSduStatusTag::LAST_SEGMENT)
            {
                newTag.SetStatus(NrRlcSduStatusTag::MIDDLE_SEGMENT);
                // oldTag.SetStatus (NrRlcSduStatusTag::LAST_SEGMENT);
            }

            // Give back the remaining segment to the transmission buffer
            firstSegment->RemoveAtStart(currSegmentSize);
            NS_LOG_LOGIC(
                "    firstSegment size (after RemoveAtStart) = " << firstSegment->GetSize());
            if (firstSegment->GetSize() > 0)
            {
                firstSegment->AddPacketTag(oldTag);

                m_txBuffer.insert(m_txBuffer.begin(), TxPdu(firstSegment, firstSegmentTime));
                m_txBufferSize += m_txBuffer.begin()->m_pdu->GetSize();

                NS_LOG_LOGIC("    TX buffer: Give back the remaining segment");
                NS_LOG_LOGIC("    TX buffers = " << m_txBuffer.size());
                NS_LOG_LOGIC("    Front buffer size = " << m_txBuffer.begin()->m_pdu->GetSize());
                NS_LOG_LOGIC("    txBufferSize = " << m_txBufferSize);
            }
            else
            {
                // Whole segment was taken, so adjust tag
                if (newTag.GetStatus() == NrRlcSduStatusTag::FIRST_SEGMENT)
                {
                    newTag.SetStatus(NrRlcSduStatusTag::FULL_SDU);
                }
                else if (newTag.GetStatus() == NrRlcSduStatusTag::MIDDLE_SEGMENT)
                {
                    newTag.SetStatus(NrRlcSduStatusTag::LAST_SEGMENT);
                }
            }
            // Segment is completely taken or
            // the remaining segment is given back to the transmission buffer
            firstSegment = nullptr;

            // Put status tag once it has been adjusted
            newSegment->AddPacketTag(newTag);

            // Add Segment to Data field
            dataFieldAddedSize = newSegment->GetSize();
            dataField.push_back(newSegment);
            newSegment = nullptr;

            // ExtensionBit (Next_Segment - 1) = 0
            rlcHeader.PushExtensionBit(NrRlcHeader::DATA_FIELD_FOLLOWS);

            // no LengthIndicator for the last one

            nextSegmentSize -= dataFieldAddedSize;
            nextSegmentId++;

            // nextSegmentSize MUST be zero (only if segment is smaller or equal to 2047)

            // (NO more segments) → exit
            // break;
        }
        else if ((nextSegmentSize - firstSegment->GetSize() <= 2) || m_txBuffer.empty())
        {
            NS_LOG_LOGIC(
                "    IF nextSegmentSize - firstSegment->GetSize () <= 2 || txBuffer.size == 0");
            // Add txBuffer.FirstBuffer to DataField
            dataFieldAddedSize = firstSegment->GetSize();
            dataField.push_back(firstSegment);
            firstSegment = nullptr;

            // ExtensionBit (Next_Segment - 1) = 0
            rlcHeader.PushExtensionBit(NrRlcHeader::DATA_FIELD_FOLLOWS);

            // no LengthIndicator for the last one

            nextSegmentSize -= dataFieldAddedSize;
            nextSegmentId++;

            NS_LOG_LOGIC("        SDUs in TxBuffer  = " << m_txBuffer.size());
            if (!m_txBuffer.empty())
            {
                NS_LOG_LOGIC("        First SDU buffer  = " << m_txBuffer.begin()->m_pdu);
                NS_LOG_LOGIC(
                    "        First SDU size    = " << m_txBuffer.begin()->m_pdu->GetSize());
            }
            NS_LOG_LOGIC("        Next segment size = " << nextSegmentSize);

            // nextSegmentSize <= 2 (only if txBuffer is not empty)

            // (NO more segments) → exit
            // break;
        }
        else // (firstSegment->GetSize () < m_nextSegmentSize) && (m_txBuffer.size () > 0)
        {
            NS_LOG_LOGIC("    IF firstSegment < NextSegmentSize && txBuffer.size > 0");
            // Add txBuffer.FirstBuffer to DataField
            dataFieldAddedSize = firstSegment->GetSize();
            dataField.push_back(firstSegment);

            // ExtensionBit (Next_Segment - 1) = 1
            rlcHeader.PushExtensionBit(NrRlcHeader::E_LI_FIELDS_FOLLOWS);

            // LengthIndicator (Next_Segment)  = txBuffer.FirstBuffer.length()
            rlcHeader.PushLengthIndicator(firstSegment->GetSize());

            nextSegmentSize -= ((nextSegmentId % 2) ? (2) : (1)) + dataFieldAddedSize;
            nextSegmentId++;

            NS_LOG_LOGIC("        SDUs in TxBuffer  = " << m_txBuffer.size());
            if (!m_txBuffer.empty())
            {
                NS_LOG_LOGIC("        First SDU buffer  = " << m_txBuffer.begin()->m_pdu);
                NS_LOG_LOGIC(
                    "        First SDU size    = " << m_txBuffer.begin()->m_pdu->GetSize());
            }
            NS_LOG_LOGIC("        Next segment size = " << nextSegmentSize);
            NS_LOG_LOGIC("        Remove SDU from TxBuffer");

            // (more segments)
            firstSegment = m_txBuffer.begin()->m_pdu->Copy();
            firstSegmentTime = m_txBuffer.begin()->m_waitingSince;
            m_txBufferSize -= firstSegment->GetSize();
            m_txBuffer.pop_front();
            NS_LOG_LOGIC("        txBufferSize = " << m_txBufferSize);
        }
    }

    // Build RLC header
    rlcHeader.SetSequenceNumber(m_sequenceNumber++);

    // Build RLC PDU with DataField and Header
    auto it = dataField.begin();

    uint8_t framingInfo = 0;

    // FIRST SEGMENT
    NrRlcSduStatusTag tag;
    NS_ASSERT_MSG((*it)->PeekPacketTag(tag), "NrRlcSduStatusTag is missing");
    (*it)->PeekPacketTag(tag);
    if ((tag.GetStatus() == NrRlcSduStatusTag::FULL_SDU) ||
        (tag.GetStatus() == NrRlcSduStatusTag::FIRST_SEGMENT))
    {
        framingInfo |= NrRlcHeader::FIRST_BYTE;
    }
    else
    {
        framingInfo |= NrRlcHeader::NO_FIRST_BYTE;
    }

    while (it < dataField.end())
    {
        NS_LOG_LOGIC("Adding SDU/segment to packet, length = " << (*it)->GetSize());

        NS_ASSERT_MSG((*it)->PeekPacketTag(tag), "NrRlcSduStatusTag is missing");
        (*it)->RemovePacketTag(tag);
        if (packet->GetSize() > 0)
        {
            packet->AddAtEnd(*it);
        }
        else
        {
            packet = (*it);
        }
        it++;
    }

    // LAST SEGMENT (Note: There could be only one and be the first one)
    it--;
    if ((tag.GetStatus() == NrRlcSduStatusTag::FULL_SDU) ||
        (tag.GetStatus() == NrRlcSduStatusTag::LAST_SEGMENT))
    {
        framingInfo |= NrRlcHeader::LAST_BYTE;
    }
    else
    {
        framingInfo |= NrRlcHeader::NO_LAST_BYTE;
    }

    rlcHeader.SetFramingInfo(framingInfo);

    NS_LOG_LOGIC("RLC header: " << rlcHeader);
    if (continues)
    {
        NS_ASSERT((framingInfo & NrRlcHeader::NO_FIRST_BYTE) != 0);
        Ptr<Packet> so = Create<Packet>(m_segmentOffsetBytes);
        so->AddAtEnd(packet);
        packet = so;
    }
    packet->AddHeader(rlcHeader);

    // Sender timestamp and transmitting entity identity, on the header bytes. Not from byte 1:
    // with a 1-byte header that is an empty range, which ns-3 keeps after the header is removed,
    // so the tag would travel on with the SDU and confuse the next RLC entity it reaches.
    NrRlcTag rlcTag(Simulator::Now());
    rlcTag.SetTxEntityId(m_rlcEntityId);
    packet->AddByteTag(rlcTag, 0, rlcHeader.GetSerializedSize());
    m_txPdu(m_rnti, m_lcid, packet->GetSize());

    // Send RLC PDU to MAC layer
    NrMacSapProvider::TransmitPduParameters params;
    params.pdu = packet;
    params.rnti = m_rnti;
    params.lcid = m_lcid;
    params.layer = txOpParams.layer;
    params.harqProcessId = txOpParams.harqId;
    params.componentCarrierId = txOpParams.componentCarrierId;

    NS_LOG_INFO("Forward RLC PDU to MAC Layer");
    m_macSapProvider->TransmitPdu(params);

    RestartBsrTimer();
}

void
NrRlcUm::DoNotifyHarqDeliveryFailure()
{
    NS_LOG_FUNCTION(this);
}

void
NrRlcUm::DoReceivePdu(NrMacSapUser::ReceivePduParameters rxPduParams)
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid << rxPduParams.p->GetSize());

    // Receiver timestamp
    NrRlcTag rlcTag;
    Time delay;

    bool ret = rxPduParams.p->FindFirstMatchingByteTag(rlcTag);
    NS_ASSERT_MSG(ret, "NrRlcTag is missing");

    delay = Simulator::Now() - rlcTag.GetSenderTimestamp();
    m_rxPdu(m_rnti, m_lcid, rxPduParams.p->GetSize(), delay.GetNanoSeconds());

    // Old-epoch PDUs of a peer entity destroyed at handover alias into the new SN
    // space and would corrupt reassembly undetectably: discard them.
    if (!AcceptPduFromPeerEntity(rlcTag.GetTxEntityId()))
    {
        return;
    }

    // 5.1.2.2 Receive operations

    // Get RLC header parameters
    NrRlcHeader rlcHeader;
    rlcHeader.SetSnFieldLength(m_snFieldLength);
    rxPduParams.p->PeekHeader(rlcHeader);
    NS_LOG_LOGIC("RLC header: " << rlcHeader);
    nr::SequenceNumber10 seqNumber = rlcHeader.GetSequenceNumber();

    // 5.1.2.2.1 General
    // The receiving UM RLC entity shall maintain a reordering window according to state variable
    // VR(UH) as follows:
    // - a SN falls within the reordering window if (VR(UH) - UM_Window_Size) <= SN < VR(UH);
    // - a SN falls outside of the reordering window otherwise.
    // When receiving an UMD PDU from lower layer, the receiving UM RLC entity shall:
    // - either discard the received UMD PDU or place it in the reception buffer (see sub
    // clause 5.1.2.2.2);
    // - if the received UMD PDU was placed in the reception buffer:
    // - update state variables, reassemble and deliver RLC SDUs to upper layer and start/stop
    // t-Reordering as needed (see sub clause 5.1.2.2.3); When t-Reordering expires, the receiving
    // UM RLC entity shall:
    // - update state variables, reassemble and deliver RLC SDUs to upper layer and start
    // t-Reordering as needed (see sub clause 5.1.2.2.4).

    // A PDU whose SN falls outside the reordering window is normally the newest
    // transmission and slides the window forward (5.1.2.2.3). But a PDU delayed by
    // more than UM_Window_Size SNs (e.g. a last-round HARQ retransmission at high
    // rate) aliases to the same out-of-window position, and sliding the window back
    // onto it would discard live in-window traffic wholesale. The two cases are
    // indistinguishable in the 10-bit SN space, but the newest transmission always
    // carries the newest sender timestamp: discard an out-of-window PDU that was
    // sent before traffic already received.
    if (!IsInsideReorderingWindow(seqNumber) &&
        (rlcTag.GetSenderTimestamp() < m_maxRxSenderTimestamp))
    {
        NS_LOG_WARN("Discarding PDU with SN="
                    << seqNumber << " outside the reordering window but sent ("
                    << rlcTag.GetSenderTimestamp().As(Time::MS) << ") before already-received "
                    << "traffic (" << m_maxRxSenderTimestamp.As(Time::MS)
                    << "): severely delayed duplicate, not new traffic");
        return;
    }
    if (rlcTag.GetSenderTimestamp() > m_maxRxSenderTimestamp)
    {
        m_maxRxSenderTimestamp = rlcTag.GetSenderTimestamp();
    }

    // 5.1.2.2.2 Actions when an UMD PDU is received from lower layer
    // When an UMD PDU with SN = x is received from lower layer, the receiving UM RLC entity shall:
    // - if VR(UR) < x < VR(UH) and the UMD PDU with SN = x has been received before; or
    // - if (VR(UH) - UM_Window_Size) <= x < VR(UR):
    //    - discard the received UMD PDU;
    // - else:
    //    - place the received UMD PDU in the reception buffer.

    NS_LOG_LOGIC("VR(UR) = " << m_vrUr);
    NS_LOG_LOGIC("VR(UX) = " << m_vrUx);
    NS_LOG_LOGIC("VR(UH) = " << m_vrUh);
    NS_LOG_LOGIC("SN = " << seqNumber);

    m_vrUr.SetModulusBase(m_vrUh - m_windowSize);
    m_vrUh.SetModulusBase(m_vrUh - m_windowSize);
    seqNumber.SetModulusBase(m_vrUh - m_windowSize);

    if (((m_vrUr < seqNumber) && (seqNumber < m_vrUh) &&
         (m_rxBuffer.count(seqNumber.GetValue()) > 0)) ||
        (((m_vrUh - m_windowSize) <= seqNumber) && (seqNumber < m_vrUr)))
    {
        NS_LOG_LOGIC("PDU discarded");
        rxPduParams.p = nullptr;
        return;
    }
    else
    {
        NS_ASSERT_MSG(m_rxBuffer.count(seqNumber.GetValue()) == 0,
                      "INTERNAL ERROR: PDU with SN=" << seqNumber.GetValue()
                                                     << " already exists in the RX buffer.");

        NS_LOG_LOGIC("Place PDU in the reception buffer");
        m_rxBuffer[seqNumber.GetValue()] = rxPduParams.p;
    }

    // 5.1.2.2.3 Actions when an UMD PDU is placed in the reception buffer
    // When an UMD PDU with SN = x is placed in the reception buffer, the receiving UM RLC entity
    // shall:
    // - if rlc-OutOfOrderDelivery is configured:
    //   - if all byte segments of the UMD PDU are received:
    //   - reassemble the RLC SDU using the byte segments of the UMD PDU,
    //     remove RLC headers when doing so and deliver the reassembled RLC
    //     SDU to upper layer if not delivered before;

    if (m_outOfOrderDelivery)
    {
        // Deliver immediately, leaving tombstones so late duplicates of the
        // delivered PDUs are recognised and not delivered twice.
        ReassembleOutsideWindow(true);
    }

    // - if x falls outside of the reordering window:
    //    - update VR(UH) to x + 1;
    //    - reassemble RLC SDUs from any UMD PDUs with SN that falls outside of the reordering
    //    window, remove
    //      RLC headers when doing so and deliver the reassembled RLC SDUs to upper layer in
    //      ascending order of the RLC SN if not delivered before;
    //    - if VR(UR) falls outside of the reordering window:
    //        - set VR(UR) to (VR(UH) - UM_Window_Size);

    if (!IsInsideReorderingWindow(seqNumber))
    {
        NS_LOG_LOGIC("SN is outside the reordering window");

        m_vrUh = seqNumber + 1;
        NS_LOG_LOGIC("New VR(UH) = " << m_vrUh);

        ReassembleOutsideWindow();

        if (!IsInsideReorderingWindow(m_vrUr))
        {
            m_vrUr = m_vrUh - m_windowSize;
            NS_LOG_LOGIC("VR(UR) is outside the reordering window");
            NS_LOG_LOGIC("New VR(UR) = " << m_vrUr);
        }
    }

    // - if the reception buffer contains an UMD PDU with SN = VR(UR):
    //    - update VR(UR) to the SN of the first UMD PDU with SN > current VR(UR) that has not been
    //    received;
    //    - reassemble RLC SDUs from any UMD PDUs with SN < updated VR(UR), remove RLC headers when
    //    doing
    //      so and deliver the reassembled RLC SDUs to upper layer in ascending order of the RLC SN
    //      if not delivered before;

    if (m_rxBuffer.count(m_vrUr.GetValue()) > 0)
    {
        NS_LOG_LOGIC("Reception buffer contains SN = " << m_vrUr);

        nr::SequenceNumber10 oldVrUr = m_vrUr;

        // Advance with a SequenceNumber10 so the scan wraps at the 10-bit boundary:
        // a raw counter would stop at 1024 (never a buffer key) and strand buffered
        // or tombstoned entries at SN 0 while VR(UR) wraps onto them.
        nr::SequenceNumber10 newVrUr = m_vrUr + 1;
        while (m_rxBuffer.count(newVrUr.GetValue()) > 0)
        {
            newVrUr++;
        }
        m_vrUr = newVrUr;
        NS_LOG_LOGIC("New VR(UR) = " << m_vrUr);

        ReassembleSnInterval(oldVrUr, m_vrUr);
    }

    // m_vrUh can change previously, set new modulus base
    // for the t-Reordering timer-related comparisons
    m_vrUr.SetModulusBase(m_vrUh - m_windowSize);
    m_vrUx.SetModulusBase(m_vrUh - m_windowSize);
    m_vrUh.SetModulusBase(m_vrUh - m_windowSize);

    // - if t-Reordering is running:
    //    - if VR(UX) <= VR(UR); or
    //    - if VR(UX) falls outside of the reordering window and VR(UX) is not equal to VR(UH)::
    //        - stop and reset t-Reordering;
    if (m_reorderingTimer.IsPending())
    {
        NS_LOG_LOGIC("Reordering timer is running");

        if ((m_vrUx <= m_vrUr) || ((!IsInsideReorderingWindow(m_vrUx)) && (m_vrUx != m_vrUh)))
        {
            NS_LOG_LOGIC("Stop reordering timer");
            m_reorderingTimer.Cancel();
        }
    }

    // - if t-Reordering is not running (includes the case when t-Reordering is stopped due to
    // actions above):
    //    - if VR(UH) > VR(UR):
    //        - start t-Reordering;
    //        - set VR(UX) to VR(UH).
    if (!m_reorderingTimer.IsPending())
    {
        NS_LOG_LOGIC("Reordering timer is not running");

        if (m_vrUh > m_vrUr)
        {
            NS_LOG_LOGIC("VR(UH) > VR(UR)");
            NS_LOG_LOGIC("Start reordering timer");
            m_reorderingTimer =
                Simulator::Schedule(m_reorderingTimerValue, &NrRlcUm::ExpireReorderingTimer, this);
            m_vrUx = m_vrUh;
            NS_LOG_LOGIC("New VR(UX) = " << m_vrUx);
        }
    }
}

bool
NrRlcUm::IsInsideReorderingWindow(nr::SequenceNumber10 seqNumber)
{
    NS_LOG_FUNCTION(this << seqNumber);
    NS_LOG_LOGIC("Reordering Window: " << m_vrUh << " - " << m_windowSize << " <= " << seqNumber
                                       << " < " << m_vrUh);

    m_vrUh.SetModulusBase(m_vrUh - m_windowSize);
    seqNumber.SetModulusBase(m_vrUh - m_windowSize);

    if (((m_vrUh - m_windowSize) <= seqNumber) && (seqNumber < m_vrUh))
    {
        NS_LOG_LOGIC(seqNumber << " is INSIDE the reordering window");
        return true;
    }
    else
    {
        NS_LOG_LOGIC(seqNumber << " is OUTSIDE the reordering window");
        return false;
    }
}

void
NrRlcUm::ReassembleAndDeliver(Ptr<Packet> packet)
{
    NrRlcHeader rlcHeader;
    rlcHeader.SetSnFieldLength(m_snFieldLength);
    packet->RemoveHeader(rlcHeader);
    uint8_t framingInfo = rlcHeader.GetFramingInfo();
    nr::SequenceNumber10 currSeqNumber = rlcHeader.GetSequenceNumber();
    if (m_segmentOffsetBytes > 0 && (framingInfo & NrRlcHeader::NO_FIRST_BYTE) != 0)
    {
        packet->RemoveAtStart(m_segmentOffsetBytes); // the segment offset
    }
    bool expectedSnLost;

    if (currSeqNumber != m_expectedSeqNumber)
    {
        expectedSnLost = true;
        NS_LOG_LOGIC("There are losses. Expected SN = " << m_expectedSeqNumber
                                                        << ". Current SN = " << currSeqNumber);
        m_expectedSeqNumber = currSeqNumber + 1;
    }
    else
    {
        expectedSnLost = false;
        NS_LOG_LOGIC("No losses. Expected SN = " << m_expectedSeqNumber
                                                 << ". Current SN = " << currSeqNumber);
        m_expectedSeqNumber++;
    }

    // Build list of SDUs
    SplitDataFields(rlcHeader, packet, m_sdusBuffer);

    // Current reassembling state
    if (m_reassemblingState == WAITING_S0_FULL)
    {
        NS_LOG_LOGIC("Reassembling State = 'WAITING_S0_FULL'");
    }
    else if (m_reassemblingState == WAITING_SI_SF)
    {
        NS_LOG_LOGIC("Reassembling State = 'WAITING_SI_SF'");
    }
    else
    {
        NS_LOG_LOGIC("Reassembling State = Unknown state");
    }

    // Received framing Info
    NS_LOG_LOGIC("Framing Info = " << (uint16_t)framingInfo);

    // The FI field is the transmitter's ground truth about SDU boundaries, while
    // expectedSnLost is inferred from 10-bit SN continuity, which aliases: after a
    // handover both RLC entities are re-created and an old-epoch PDU still in flight
    // can reach the new entity at a position where SN == m_expectedSeqNumber, and
    // >= 1023 consecutive losses alias the same way. When the FI contradicts the
    // reassembly state, the awaited segment was lost even though the SN looks
    // consecutive, so the stale partial SDU is discarded rather than concatenated
    // with unrelated data, resynchronising on the next SDU boundary.
    const bool firstFieldContinuesSdu = ((framingInfo & NrRlcHeader::NO_FIRST_BYTE) != 0);
    const bool lastFieldIsComplete = ((framingInfo & NrRlcHeader::NO_LAST_BYTE) == 0);
    if (!expectedSnLost && firstFieldContinuesSdu != (m_reassemblingState == WAITING_SI_SF))
    {
        NS_LOG_WARN("SN=" << currSeqNumber
                          << " matches the expected SN but FI=" << (uint16_t)framingInfo
                          << " contradicts the reassembly state: resynchronising");
    }

    m_reassemblingState = ReassembleSdus(m_sdusBuffer,
                                         m_keepS0,
                                         firstFieldContinuesSdu,
                                         lastFieldIsComplete,
                                         !expectedSnLost)
                              ? WAITING_SI_SF
                              : WAITING_S0_FULL;
}

void
NrRlcUm::ReestablishRxSide()
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid);

    // TS 38.322 5.1.2: on re-establishment the RLC entity shall discard all RLC
    // SDUs, RLC SDU segments and RLC PDUs, stop and reset all timers, and reset all
    // state variables to their initial values.
    m_reorderingTimer.Cancel();
    m_rxBuffer.clear();
    m_keepS0 = nullptr;
    m_reassemblingState = WAITING_S0_FULL;
    // Assign fresh SequenceNumber10 objects rather than raw values:
    // operator=(uint16_t) keeps the old modulus base (see the AM twin).
    m_expectedSeqNumber = NewSn(0);
    m_vrUr = NewSn(0);
    m_vrUx = NewSn(0);
    m_vrUh = NewSn(0);
    m_maxRxSenderTimestamp = Time();
}

void
NrRlcUm::ReassembleOutsideWindow(bool tombstoneDelivered)
{
    NS_LOG_LOGIC("Reassemble Outside Window");

    auto it = m_rxBuffer.lower_bound(m_vrUr.GetValue()); // First SN such that SN >= VR(UR)

    // In tombstone mode the delivered entries stay in the buffer (as null), so the
    // iteration must be bounded by the number of entries rather than by emptiness.
    for (size_t remaining = m_rxBuffer.size(); remaining > 0; remaining--)
    {
        if (it == m_rxBuffer.end())
        {
            // wrap-around manual
            it = m_rxBuffer.begin();
        }

        if (IsInsideReorderingWindow(NewSn(it->first)))
        {
            break;
        }

        NS_LOG_LOGIC("SN = " << it->first);

        // Reassemble RLC SDUs and deliver the PDCP PDU to upper layer. A null entry
        // is a tombstone for a PDU already delivered out of order: consume it
        // without delivering again.
        if (it->second)
        {
            ReassembleAndDeliver(it->second);
        }

        if (tombstoneDelivered)
        {
            // Remember the SN as received so that a duplicate of this PDU arriving
            // later (once the SN has slid inside the reordering window, where only
            // buffered SNs are recognised as duplicates) is not delivered twice.
            it->second = nullptr;
            ++it;
        }
        else
        {
            auto it_tmp = it;
            ++it;
            m_rxBuffer.erase(it_tmp);
        }
    }

    if (it != m_rxBuffer.end())
    {
        NS_LOG_LOGIC("(SN = " << it->first << ") is inside the reordering window");
    }
}

void
NrRlcUm::ReassembleSnInterval(nr::SequenceNumber10 lowSeqNumber, nr::SequenceNumber10 highSeqNumber)
{
    NS_LOG_LOGIC("Reassemble SN between " << lowSeqNumber << " and " << highSeqNumber);

    nr::SequenceNumber10 reassembleSn = lowSeqNumber;
    NS_LOG_LOGIC("reassembleSN = " << reassembleSn);
    NS_LOG_LOGIC("highSeqNumber = " << highSeqNumber);
    while (reassembleSn < highSeqNumber)
    {
        NS_LOG_LOGIC("reassembleSn < highSeqNumber");
        auto it = m_rxBuffer.find(reassembleSn.GetValue());
        if (it != m_rxBuffer.end())
        {
            NS_LOG_LOGIC("it->first  = " << it->first);
            NS_LOG_LOGIC("it->second = " << it->second);
            NS_LOG_LOGIC("SN = " << it->first);

            // Reassemble RLC SDUs and deliver the PDCP PDU to upper layer. A null
            // entry is a tombstone for a PDU already delivered out of order:
            // consume it without delivering again.
            if (it->second)
            {
                ReassembleAndDeliver(it->second);
            }

            m_rxBuffer.erase(it);
        }

        reassembleSn++;
    }
}

void
NrRlcUm::DoTransmitBufferStatusReport()
{
    DiscardStale();
    Time holDelay(0);
    uint32_t queueSize = 0;

    if (!m_txBuffer.empty())
    {
        holDelay = Simulator::Now() - m_txBuffer.front().m_waitingSince;

        // Data in the tx queue and the header of the one PDU that would carry it all: the fixed
        // part, and for every SDU after the first a 12-bit extension (E + LI). The estimate was
        // one fixed header per SDU, which is a byte or more short of that from the second SDU
        // on. A MAC that offers exactly the bytes reported (NrUeMac does) then gets a PDU that
        // leaves the last bytes of the last SDU behind, and takes the buffer for empty.
        const auto extensions = static_cast<uint32_t>(m_txBuffer.size() - 1);
        queueSize = m_txBufferSize + FixedHeaderSize() + (extensions * 12 + 7) / 8;
        // (NR: a head that continues an SDU also carries the segment offset)
        NrRlcSduStatusTag headTag;
        if (m_segmentOffsetBytes > 0 && m_txBuffer.front().m_pdu->PeekPacketTag(headTag) &&
            (headTag.GetStatus() == NrRlcSduStatusTag::MIDDLE_SEGMENT ||
             headTag.GetStatus() == NrRlcSduStatusTag::LAST_SEGMENT))
        {
            queueSize += m_segmentOffsetBytes;
        }
    }

    NrMacSapProvider::BufferStatusReportParameters r;
    r.rnti = m_rnti;
    r.lcid = m_lcid;
    r.txQueueSize = queueSize;
    r.txQueueHolDelay = holDelay.GetMilliSeconds();
    r.retxQueueSize = 0;
    r.retxQueueHolDelay = 0;
    r.statusPduSize = 0;
    r.expBsrTimer = m_expBsrTimer;

    m_expBsrTimer = false;

    NS_LOG_LOGIC("Send BufferStatusReport = " << r.txQueueSize << ", " << r.txQueueHolDelay);
    m_macSapProvider->BufferStatusReport(r);
}

void
NrRlcUm::ExpireReorderingTimer()
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid);
    NS_LOG_LOGIC("Reordering timer has expired");

    // 5.1.2.2.4 Actions when t-Reordering expires
    // When t-Reordering expires, the receiving UM RLC entity shall:
    // - update VR(UR) to the SN of the first UMD PDU with SN >= VR(UX) that has not been received;
    // - reassemble RLC SDUs from any UMD PDUs with SN < updated VR(UR), remove RLC headers when
    // doing so
    //   and deliver the reassembled RLC SDUs to upper layer in ascending order of the RLC SN if not
    //   delivered before;
    // - if VR(UH) > VR(UR):
    //    - start t-Reordering;
    //    - set VR(UX) to VR(UH).

    nr::SequenceNumber10 newVrUr = m_vrUx;

    while (m_rxBuffer.find(newVrUr.GetValue()) != m_rxBuffer.end())
    {
        newVrUr++;
    }
    nr::SequenceNumber10 oldVrUr = m_vrUr;
    m_vrUr = newVrUr;
    NS_LOG_LOGIC("New VR(UR) = " << m_vrUr);

    ReassembleSnInterval(oldVrUr, m_vrUr);

    // t-Reordering expiry means the missing segment(s) at the front of the reordering
    // window are given up on. Per TS 38.322 (5.1.2.2.4 / 5.2.2.2), any RLC SDU that can
    // no longer be fully reassembled must be discarded. If a partially-reassembled SDU is
    // still held (WAITING_SI_SF, i.e. S0 is kept waiting for its continuation) after the
    // above in-window reassembly, its continuation was among the lost SNs and will never
    // arrive: drop it and resynchronise. Leaving it held lets it survive until the 10-bit
    // SN wraps (~1024 PDUs), after which an unrelated later segment aliases as the awaited
    // continuation and is concatenated onto the stale head, delivering a corrupt (non-IP)
    // SDU to upper layers (observed as the "Unknown IP type" abort during fast handovers).
    if (m_reassemblingState == WAITING_SI_SF)
    {
        NS_LOG_LOGIC("Discarding partially reassembled SDU stranded by t-Reordering expiry");
        m_keepS0 = nullptr;
        m_reassemblingState = WAITING_S0_FULL;
    }

    if (m_vrUh > m_vrUr)
    {
        NS_LOG_LOGIC("Start reordering timer");
        m_reorderingTimer =
            Simulator::Schedule(m_reorderingTimerValue, &NrRlcUm::ExpireReorderingTimer, this);
        m_vrUx = m_vrUh;
        NS_LOG_LOGIC("New VR(UX) = " << m_vrUx);
    }
}

void
NrRlcUm::ExpireBsrTimer()
{
    NS_LOG_LOGIC("BSR Timer expires");

    if (!m_txBuffer.empty())
    {
        m_expBsrTimer = true;
        DoTransmitBufferStatusReport();
        RestartBsrTimer();
    }
}

void
NrRlcUm::RestartBsrTimer()
{
    NS_LOG_FUNCTION(this);

    m_bsrTimer.Cancel();

    if (!m_txBuffer.empty())
    {
        m_bsrTimer = Simulator::Schedule(m_bsrTimerValue, &NrRlcUm::ExpireBsrTimer, this);
    }
}

} // namespace ns3
