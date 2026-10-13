// Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-ntn-timing-advance.cc
 *
 * @brief Test suite (nr-ntn-timing-advance): with NrUePhy::TimingAdvance set to the one-way
 * propagation delay, every uplink transmission reaches the gNB in the slot the gNB scheduled
 * it for, and DL HARQ feedback lands in the slot the DL DCI's K1 points at, in the gNB's slot
 * numbering.
 *
 * The cell is FDD (a DL-only and an UL-only BWP, as every 3GPP NR NTN band), the channel has a
 * ConstantSpeedPropagationDelayModel whose speed is scaled so that short distances give
 * millisecond delays without a huge path loss, and the UEs run bidirectional UDP traffic so
 * that UL data, UL control (HARQ feedback, CQI, SR) and DL data are all exercised. The one-way
 * delays are a whole number of slots (2.0 ms at numerology 0) and a fractional one (3.6 ms);
 * a two-UE case at both delays must run to completion in an assert-enabled build, where
 * NrSpectrumPhy::StartRxUlCtrl asserts that overlapping UL control receptions start at the
 * same instant. A TimingAdvance = 0 control at 2.0 ms shows the misalignment: every DL HARQ
 * feedback arrives 2 * delay after the slot K1 points at, and UL transport blocks miss the
 * gNB's expectation. Two UEs moving at those delays run over NrNtnPropagationDelayModel with the
 * timing advance re-read every slot: their arrivals miss each other by nanoseconds, which only
 * the uplink line-up of that model lets the receiver take. A unit case pins the line-up itself.
 */

#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-control-messages.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-ntn-link-models.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/nstime.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/position-allocator.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/uinteger.h"

#include <cmath>
#include <map>
#include <set>
#include <tuple>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrNtnTimingAdvanceTest");

namespace
{

/// Propagation speed of the test channel: 20 m of distance give 2 ms of one-way delay, so the
/// delays of the test are set by the UE positions while the log-distance path loss stays small.
constexpr double g_propagationSpeedMps = 10000.0;

/// One-way delay between the two nodes at the test's propagation speed
Time
GeometricDelay(Ptr<MobilityModel> ue, Ptr<MobilityModel> gnb)
{
    return Seconds(ue->GetDistanceFrom(gnb) / g_propagationSpeedMps);
}

} // namespace

/**
 * @ingroup nr-test
 *
 * @brief NrNtnPropagationDelayModel::AlignUplinkArrival moves an arrival onto the nearest
 * symbol start of the gNB grid within one cyclic prefix, and leaves it alone further off.
 */
class NrNtnUplinkAlignmentTestCase : public TestCase
{
  public:
    NrNtnUplinkAlignmentTestCase()
        : TestCase("Uplink arrivals line up on the gNB symbol grid within a cyclic prefix")
    {
    }

  private:
    void DoRun() override
    {
        // Numerology 0 as NrPhy computes it: 1 ms slots of 14 symbols of 71428 ns, which end
        // 8 ns before the slot does; one cyclic prefix is 4692 ns
        const Time slot = MilliSeconds(1);
        const Time symbol = slot / 14;
        auto align = [&](Time arrival) {
            return NrNtnPropagationDelayModel::AlignUplinkArrival(arrival, slot, symbol, 14);
        };
        const Time symbol3 = MilliSeconds(5) + 3 * symbol;
        NS_TEST_ASSERT_MSG_EQ(align(symbol3), symbol3, "An arrival on the grid must not move");
        NS_TEST_ASSERT_MSG_EQ(align(symbol3 + NanoSeconds(1)), symbol3, "1 ns late");
        NS_TEST_ASSERT_MSG_EQ(align(symbol3 - NanoSeconds(7)), symbol3, "7 ns early");
        NS_TEST_ASSERT_MSG_EQ(align(symbol3 + NanoSeconds(4692)),
                              symbol3,
                              "One cyclic prefix late still lines up");
        NS_TEST_ASSERT_MSG_EQ(align(symbol3 - NanoSeconds(4693)),
                              symbol3 - NanoSeconds(4693),
                              "Beyond a cyclic prefix the arrival keeps its time");
        NS_TEST_ASSERT_MSG_EQ(align(MilliSeconds(6) - NanoSeconds(3)),
                              MilliSeconds(6),
                              "Just before a slot start lines up on the slot start, not on the "
                              "end of the last symbol");
        NS_TEST_ASSERT_MSG_EQ(align(MilliSeconds(6) + NanoSeconds(3)),
                              MilliSeconds(6),
                              "Just after a slot start");
        // Numerology 1: 500 us slots of 35714 ns symbols
        const Time slot1 = MicroSeconds(500);
        const Time symbol1 = slot1 / 14;
        NS_TEST_ASSERT_MSG_EQ(
            NrNtnPropagationDelayModel::AlignUplinkArrival(slot1 + 13 * symbol1 + NanoSeconds(2),
                                                           slot1,
                                                           symbol1,
                                                           14),
            slot1 + 13 * symbol1,
            "Numerology 1, last symbol");
    }
};

/**
 * @ingroup nr-test
 *
 * @brief One FDD cell, one or two UEs at given one-way delays, bidirectional UDP traffic;
 * checks the slot alignment of UL data and of DL HARQ feedback at the gNB.
 */
class NrNtnTimingAdvanceTestCase : public TestCase
{
  public:
    /**
     * Constructor
     * @param name the name of the test case
     * @param delays one-way propagation delay of each UE (one entry per UE)
     * @param timingAdvance set NrUePhy::TimingAdvance to the delay (true) or leave it at 0
     * @param kOffset NrGnbPhy::CellSpecificKoffset
     * @param expectAligned whether every UL TB and HARQ feedback must be received in the slot
     * the gNB expects (true), or the misalignment of a zero timing advance must show (false)
     */
    NrNtnTimingAdvanceTestCase(const std::string& name,
                               std::vector<Time> delays,
                               bool timingAdvance,
                               uint32_t kOffset,
                               bool expectAligned,
                               double velocityMps = 0.0,
                               Time timingAdvanceUpdatePeriod = Seconds(0),
                               bool ntnLinkModels = false)
        : TestCase(name),
          m_delays(std::move(delays)),
          m_timingAdvance(timingAdvance),
          m_kOffset(kOffset),
          m_expectAligned(expectAligned),
          m_velocityMps(velocityMps),
          m_timingAdvanceUpdatePeriod(timingAdvanceUpdatePeriod),
          m_ntnLinkModels(ntnLinkModels)
    {
    }

  private:
    void DoRun() override;

    /// NrUePhy::ReportUplinkTbSize on the UL BWP of every UE: an UL TB was transmitted
    void UeUlTbSent(uint64_t imsi, uint64_t tbSize);
    /// NrSpectrumPhy::RxDataTrace on the gNB UL BWP: the slot in which an UL signal arrives
    void GnbUlDataArrival(const SfnSf& sfn,
                          Ptr<const SpectrumValue> psd,
                          const Time& duration,
                          uint16_t bwpId,
                          uint16_t cellId);
    /// NrSpectrumPhy::RxPacketTraceGnb on the gNB UL BWP: an UL TB matched an expected TB
    void GnbUlTbReceived(RxPacketTraceParams params);
    /// NrGnbPhy::GnbPhyTxedCtrlMsgsTrace on the gNB DL BWP: DL DCIs and their K1
    void GnbDlCtrlSent(SfnSf sfn,
                       uint16_t cellId,
                       uint16_t rnti,
                       uint8_t bwpId,
                       Ptr<const NrControlMessage> msg);
    /// NrGnbPhy::GnbPhyRxedCtrlMsgsTrace on the gNB UL BWP: DL HARQ feedback arrivals
    void GnbUlCtrlReceived(SfnSf sfn,
                           uint16_t cellId,
                           uint16_t rnti,
                           uint8_t bwpId,
                           Ptr<const NrControlMessage> msg);
    /// NrSpectrumPhy::RxPacketTraceUe on the DL BWP of every UE
    void UeDlTbReceived(RxPacketTraceParams params);
    /// Periodic timing advance update from the current geometry (moving UEs)
    void UpdateTimingAdvance();

    std::vector<Time> m_delays;       ///< One-way delay per UE
    bool m_timingAdvance;             ///< Configure the timing advance
    uint32_t m_kOffset;               ///< Cell-specific K_offset
    bool m_expectAligned;             ///< Expected outcome
    double m_velocityMps;             ///< Velocity of the UEs away from the gNB (0: static)
    Time m_timingAdvanceUpdatePeriod; ///< Period of the timing advance updates (0: none)
    bool m_ntnLinkModels; ///< NrNtnPropagationDelayModel and a timing advance read every slot
    Time m_alignmentTolerance{Seconds(0)};        ///< Allowed UL arrival error at the gNB
    Time m_lastUlArrivalTime;                     ///< Time of the latest UL data arrival at the gNB
    Time m_symbolPeriod;                          ///< OFDM symbol period of the cell
    Time m_slotPeriod;                            ///< Slot period of the cell
    NetDeviceContainer m_ueDevs;                  ///< The UE devices
    Ptr<MobilityModel> m_gnbMobility;             ///< The gNB mobility model
    std::vector<Ptr<MobilityModel>> m_ueMobility; ///< The UE mobility models

    uint32_t m_ueUlTbs{0};                ///< UL TBs transmitted by the UEs
    uint32_t m_gnbUlTbs{0};               ///< UL TBs matched to an expectation at the gNB
    uint32_t m_gnbUlTbsMisaligned{0};     ///< ... whose arrival slot differs from the expected one
    uint32_t m_ueDlTbs{0};                ///< DL TBs received by the UEs
    uint32_t m_harqFeedbacks{0};          ///< DL HARQ feedbacks received by the gNB
    uint32_t m_harqFeedbacksLate{0};      ///< ... received after the slot K1 points at
    uint32_t m_harqFeedbacksLateByOne{0}; ///< ... received in the slot after the one K1 points at
    uint32_t m_harqFeedbacksEarly{0};     ///< ... received before it
    uint32_t m_harqFeedbacksNoDci{0};     ///< ... for which no DL DCI was seen
    SfnSf m_lastUlArrival;                ///< Slot of the latest UL data arrival at the gNB
    std::map<std::pair<uint16_t, uint8_t>, SfnSf>
        m_expectedFeedbackSlot; ///< (RNTI, HARQ process) -> slot the last DL DCI's K1 points at
    std::set<std::tuple<uint64_t, uint16_t, uint8_t>>
        m_gnbUlTbKeys; ///< (slot, RNTI, first symbol) of the UL TBs matched at the gNB
};

void
NrNtnTimingAdvanceTestCase::UeUlTbSent([[maybe_unused]] uint64_t imsi,
                                       [[maybe_unused]] uint64_t tbSize)
{
    m_ueUlTbs++;
}

void
NrNtnTimingAdvanceTestCase::GnbUlDataArrival(const SfnSf& sfn,
                                             [[maybe_unused]] Ptr<const SpectrumValue> psd,
                                             [[maybe_unused]] const Time& duration,
                                             [[maybe_unused]] uint16_t bwpId,
                                             [[maybe_unused]] uint16_t cellId)
{
    m_lastUlArrival = sfn;
    m_lastUlArrivalTime = Simulator::Now();
}

void
NrNtnTimingAdvanceTestCase::GnbUlTbReceived(RxPacketTraceParams params)
{
    // The trace fires once per MAC PDU of the transport block: count the block once
    const SfnSf expected(params.m_frameNum,
                         params.m_subframeNum,
                         params.m_slotNum,
                         m_lastUlArrival.GetNumerology());
    if (!m_gnbUlTbKeys.insert({expected.Normalize(), params.m_rnti, params.m_symStart}).second)
    {
        return;
    }
    m_gnbUlTbs++;
    // The UE transmits 1 ns after the start of the allocated symbol (NrUePhy::UlData), so with
    // an exact timing advance the data reaches the gNB 1 ns after the symbol it was scheduled
    // in starts; a stale timing advance of a moving UE shifts it by the geometry change since
    // the last update
    const Time expectedArrival = m_slotPeriod * static_cast<int64_t>(expected.Normalize()) +
                                 m_symbolPeriod * params.m_symStart + NanoSeconds(1);
    const Time error = m_lastUlArrivalTime - expectedArrival;
    if (Abs(error) > m_alignmentTolerance)
    {
        m_gnbUlTbsMisaligned++;
        NS_LOG_INFO("UL TB of RNTI "
                    << params.m_rnti << " expected at " << expectedArrival.As(Time::US) << " (slot "
                    << expected << ") arrived at " << m_lastUlArrivalTime.As(Time::US) << " (slot "
                    << m_lastUlArrival << ")");
    }
}

void
NrNtnTimingAdvanceTestCase::UpdateTimingAdvance()
{
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        const double distance = m_ueMobility[i]->GetDistanceFrom(m_gnbMobility);
        const Time delay = Seconds(distance / g_propagationSpeedMps);
        for (uint32_t bwp = 0; bwp < 2; ++bwp)
        {
            NrHelper::GetUePhy(m_ueDevs.Get(i), bwp)
                ->SetAttribute("TimingAdvance", TimeValue(delay));
        }
    }
    Simulator::Schedule(m_timingAdvanceUpdatePeriod,
                        &NrNtnTimingAdvanceTestCase::UpdateTimingAdvance,
                        this);
}

void
NrNtnTimingAdvanceTestCase::GnbDlCtrlSent(SfnSf sfn,
                                          [[maybe_unused]] uint16_t cellId,
                                          [[maybe_unused]] uint16_t rnti,
                                          [[maybe_unused]] uint8_t bwpId,
                                          Ptr<const NrControlMessage> msg)
{
    if (msg->GetMessageType() != NrControlMessage::DL_DCI)
    {
        return;
    }
    auto dci = DynamicCast<NrDlDciMessage>(ConstCast<NrControlMessage>(msg));
    auto dciInfo = dci->GetDciInfoElement();
    SfnSf feedbackSlot = sfn;
    feedbackSlot.Add(dci->GetK1Delay());
    m_expectedFeedbackSlot[{dciInfo->m_rnti, dciInfo->m_harqProcess}] = feedbackSlot;
}

void
NrNtnTimingAdvanceTestCase::GnbUlCtrlReceived(SfnSf sfn,
                                              [[maybe_unused]] uint16_t cellId,
                                              [[maybe_unused]] uint16_t rnti,
                                              [[maybe_unused]] uint8_t bwpId,
                                              Ptr<const NrControlMessage> msg)
{
    if (msg->GetMessageType() != NrControlMessage::DL_HARQ)
    {
        return;
    }
    auto feedback =
        DynamicCast<NrDlHarqFeedbackMessage>(ConstCast<NrControlMessage>(msg))->GetDlHarqFeedback();
    m_harqFeedbacks++;
    auto it = m_expectedFeedbackSlot.find({feedback.m_rnti, feedback.m_harqProcessId});
    if (it == m_expectedFeedbackSlot.end())
    {
        m_harqFeedbacksNoDci++;
        return;
    }
    if (it->second < sfn)
    {
        m_harqFeedbacksLate++;
        SfnSf next = it->second;
        next.Add(1);
        if (next == sfn)
        {
            m_harqFeedbacksLateByOne++;
        }
        NS_LOG_INFO("DL HARQ feedback of RNTI " << feedback.m_rnti << " process "
                                                << +feedback.m_harqProcessId << " expected in "
                                                << it->second << " arrived in " << sfn);
    }
    else if (sfn < it->second)
    {
        m_harqFeedbacksEarly++;
    }
}

void
NrNtnTimingAdvanceTestCase::UeDlTbReceived([[maybe_unused]] RxPacketTraceParams params)
{
    m_ueDlTbs++;
}

void
NrNtnTimingAdvanceTestCase::DoRun()
{
    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetGnbPhyAttribute("CellSpecificKoffset", UintegerValue(m_kOffset));
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(0));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));

    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue(3.5));
    Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(35));

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(LogDistancePropagationLossModel::GetTypeId());

    // FDD: a DL-only and an UL-only carrier, one BWP each
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf dlBandConf(2.8e9, 20e6, static_cast<uint8_t>(1));
    CcBwpCreator::SimpleOperationBandConf ulBandConf(2.9e9, 20e6, static_cast<uint8_t>(1));
    OperationBandInfo dlBand = ccBwpCreator.CreateOperationBandContiguousCc(dlBandConf);
    OperationBandInfo ulBand = ccBwpCreator.CreateOperationBandContiguousCc(ulBandConf);
    channelHelper->AssignChannelsToBands({dlBand, ulBand});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({dlBand, ulBand});

    // Physical propagation delay on both carriers
    for (const auto& bwp : allBwps)
    {
        Ptr<ConstantSpeedPropagationDelayModel> delayModel;
        if (m_ntnLinkModels)
        {
            delayModel = CreateObject<NrNtnPropagationDelayModel>();
        }
        else
        {
            delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
        }
        delayModel->SetSpeed(g_propagationSpeedMps);
        bwp.get()->GetChannel()->SetPropagationDelayModel(delayModel);
    }

    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(0));

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(m_delays.size());

    auto posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));
    for (const auto& delay : m_delays)
    {
        posAlloc->Add(Vector(delay.GetSeconds() * g_propagationSpeedMps, 0, 0));
    }
    MobilityHelper mobilityHelper;
    mobilityHelper.SetPositionAllocator(posAlloc);
    mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityHelper.Install(gnbNodes);
    mobilityHelper.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobilityHelper.Install(ueNodes);
    m_gnbMobility = gnbNodes.Get(0)->GetObject<MobilityModel>();
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        auto mobility = ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        mobility->SetVelocity(Vector(m_velocityMps, 0, 0));
        m_ueMobility.push_back(mobility);
    }

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
    m_ueDevs = ueDevs;

    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)
        ->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 1)
        ->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));

    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        for (uint32_t bwp = 0; bwp < 2; ++bwp)
        {
            auto uePhy = NrHelper::GetUePhy(ueDevs.Get(i), bwp);
            if (m_timingAdvance && m_ntnLinkModels)
            {
                uePhy->SetTimingAdvanceCallback(
                    MakeBoundCallback(&GeometricDelay, m_ueMobility[i], m_gnbMobility));
            }
            else if (m_timingAdvance)
            {
                uePhy->SetAttribute("TimingAdvance", TimeValue(m_delays[i]));
            }
            if (bwp == 1)
            {
                uePhy->TraceConnectWithoutContext(
                    "ReportUplinkTbSize",
                    MakeCallback(&NrNtnTimingAdvanceTestCase::UeUlTbSent, this));
            }
            else
            {
                uePhy->GetSpectrumPhy()->TraceConnectWithoutContext(
                    "RxPacketTraceUe",
                    MakeCallback(&NrNtnTimingAdvanceTestCase::UeDlTbReceived, this));
            }
        }
    }

    auto gnbDlPhy = NrHelper::GetGnbPhy(gnbDevs.Get(0), 0);
    auto gnbUlPhy = NrHelper::GetGnbPhy(gnbDevs.Get(0), 1);
    m_symbolPeriod = gnbUlPhy->GetSymbolPeriod();
    m_slotPeriod = gnbUlPhy->GetSlotPeriod();
    if (m_timingAdvance && !m_timingAdvanceUpdatePeriod.IsZero())
    {
        // A stale advance is off by the delay change over one update period; when the advance
        // grows, the slot in which the update lands starts late by that much on top of it
        // (plus the nanosecond rounding of the delay and of the advance)
        m_alignmentTolerance =
            2 * Seconds(std::abs(m_velocityMps) * m_timingAdvanceUpdatePeriod.GetSeconds() /
                        g_propagationSpeedMps) +
            NanoSeconds(2);
        Simulator::Schedule(m_timingAdvanceUpdatePeriod,
                            &NrNtnTimingAdvanceTestCase::UpdateTimingAdvance,
                            this);
    }
    if (m_ntnLinkModels)
    {
        // The line-up puts UL data on the symbol start, 1 ns before the UE's own offset
        m_alignmentTolerance = NanoSeconds(1);
    }
    gnbDlPhy->TraceConnectWithoutContext(
        "GnbPhyTxedCtrlMsgsTrace",
        MakeCallback(&NrNtnTimingAdvanceTestCase::GnbDlCtrlSent, this));
    // The UE stamps its DL HARQ feedback with the BWP that received the data, so the gNB
    // routes it to the DL BWP PHY: watch the UL control arrivals on both gNB PHYs
    gnbDlPhy->TraceConnectWithoutContext(
        "GnbPhyRxedCtrlMsgsTrace",
        MakeCallback(&NrNtnTimingAdvanceTestCase::GnbUlCtrlReceived, this));
    gnbUlPhy->TraceConnectWithoutContext(
        "GnbPhyRxedCtrlMsgsTrace",
        MakeCallback(&NrNtnTimingAdvanceTestCase::GnbUlCtrlReceived, this));
    gnbUlPhy->GetSpectrumPhy()->TraceConnectWithoutContext(
        "RxDataTrace",
        MakeCallback(&NrNtnTimingAdvanceTestCase::GnbUlDataArrival, this));
    gnbUlPhy->GetSpectrumPhy()->TraceConnectWithoutContext(
        "RxPacketTraceGnb",
        MakeCallback(&NrNtnTimingAdvanceTestCase::GnbUlTbReceived, this));

    InternetStackHelper inetStackHelper;
    inetStackHelper.Install(ueNodes);
    Ipv4InterfaceContainer ueIfs = nrEpcHelper->AssignUeIpv4Address(ueDevs);

    auto [remoteHost, remoteHostAddr] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.000));

    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        nrHelper->AttachToGnb(ueDevs.Get(i), gnbDevs.Get(0));
    }

    // Bidirectional UDP traffic on the default bearer of every UE
    const uint16_t dlPort = 2000;
    const uint16_t ulPort = 2001;
    const Time appStart = Seconds(0.3);
    const Time appStop = Seconds(1.0);
    const Time interval = MilliSeconds(4);
    const uint32_t packetSize = 800;
    const uint32_t numPkts = 1000;

    PacketSinkHelper dlSinkHelper("ns3::UdpSocketFactory",
                                  InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer dlSinkApps = dlSinkHelper.Install(ueNodes);
    dlSinkApps.Start(appStart);

    PacketSinkHelper ulSinkHelper("ns3::UdpSocketFactory",
                                  InetSocketAddress(Ipv4Address::GetAny(), ulPort));
    ApplicationContainer ulSinkApps = ulSinkHelper.Install(remoteHost);
    ulSinkApps.Start(appStart);
    auto ulSink = ulSinkApps.Get(0)->GetObject<PacketSink>();

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        UdpEchoClientHelper dlClient(ueIfs.GetAddress(i), dlPort);
        dlClient.SetAttribute("MaxPackets", UintegerValue(numPkts));
        dlClient.SetAttribute("Interval", TimeValue(interval));
        dlClient.SetAttribute("PacketSize", UintegerValue(packetSize));
        ApplicationContainer dlClientApps = dlClient.Install(remoteHost);
        dlClientApps.Start(appStart);
        dlClientApps.Stop(appStop);

        UdpEchoClientHelper ulClient(remoteHostAddr, ulPort);
        ulClient.SetAttribute("MaxPackets", UintegerValue(numPkts));
        ulClient.SetAttribute("Interval", TimeValue(interval));
        ulClient.SetAttribute("PacketSize", UintegerValue(packetSize));
        ApplicationContainer ulClientApps = ulClient.Install(ueNodes.Get(i));
        ulClientApps.Start(appStart);
        ulClientApps.Stop(appStop);
    }

    Simulator::Stop(Seconds(1.2));
    Simulator::Run();

    uint64_t dlRx = 0;
    for (uint32_t i = 0; i < dlSinkApps.GetN(); ++i)
    {
        dlRx += dlSinkApps.Get(i)->GetObject<PacketSink>()->GetTotalRx();
    }

    NS_LOG_INFO(GetName() << ": UL TBs sent " << m_ueUlTbs << ", received " << m_gnbUlTbs
                          << " (misaligned " << m_gnbUlTbsMisaligned << "); DL TBs received "
                          << m_ueDlTbs << "; DL HARQ feedbacks " << m_harqFeedbacks << " (late "
                          << m_harqFeedbacksLate << ", of which by one slot "
                          << m_harqFeedbacksLateByOne << ", early " << m_harqFeedbacksEarly
                          << ", without DCI " << m_harqFeedbacksNoDci << "); DL bytes " << dlRx
                          << ", UL bytes " << ulSink->GetTotalRx());

    NS_TEST_ASSERT_MSG_GT(m_ueUlTbs, 0, "The UEs transmitted no UL transport block");
    NS_TEST_ASSERT_MSG_GT(m_ueDlTbs, 0, "The UEs received no DL transport block");
    NS_TEST_ASSERT_MSG_GT(m_harqFeedbacks, 0, "The gNB received no DL HARQ feedback");
    NS_TEST_ASSERT_MSG_GT(dlRx, 0, "No downlink packet reached the UEs");
    NS_TEST_ASSERT_MSG_EQ(m_harqFeedbacksEarly, 0, "DL HARQ feedback arrived before its slot");
    NS_TEST_ASSERT_MSG_EQ(m_harqFeedbacksNoDci, 0, "DL HARQ feedback without a DL DCI");

    if (m_expectAligned)
    {
        NS_TEST_ASSERT_MSG_EQ(m_gnbUlTbs,
                              m_ueUlTbs,
                              "An UL transport block arrived in a slot in which the gNB did "
                              "not expect one (expected-TB miss)");
        NS_TEST_ASSERT_MSG_EQ(m_gnbUlTbsMisaligned,
                              0,
                              "An UL transport block arrived at the gNB away from the symbol it "
                              "was scheduled in");
        if (m_alignmentTolerance.IsZero())
        {
            NS_TEST_ASSERT_MSG_EQ(m_harqFeedbacksLate,
                                  0,
                                  "DL HARQ feedback arrived after the slot its K1 points at");
        }
        else
        {
            // The UL CTRL of a UE whose advance is stale by a few ns ends a few ns after the
            // gNB's slot boundary, and the gNB stamps a control message with the slot in which
            // its reception ends: at most one slot late, never more
            NS_TEST_ASSERT_MSG_EQ(m_harqFeedbacksLate,
                                  m_harqFeedbacksLateByOne,
                                  "DL HARQ feedback arrived more than one slot after the slot "
                                  "its K1 points at");
        }
        NS_TEST_ASSERT_MSG_GT(ulSink->GetTotalRx(), 0, "No uplink packet reached the remote host");
    }
    else
    {
        // TimingAdvance = 0 control: the UE's slot counter lags the gNB's by the delay and the
        // uplink takes the delay again, so every DL HARQ feedback is 2 * delay late, and UL
        // transport blocks arrive in slots the gNB did not schedule them for.
        NS_TEST_ASSERT_MSG_EQ(m_harqFeedbacksLate,
                              m_harqFeedbacks,
                              "Without timing advance every DL HARQ feedback must be late");
        NS_TEST_ASSERT_MSG_LT(m_gnbUlTbs,
                              m_ueUlTbs,
                              "Without timing advance some UL transport blocks must miss the "
                              "gNB's expectation");
    }

    Simulator::Destroy();
}

/**
 * @ingroup nr-test
 * @brief Test suite nr-ntn-timing-advance
 */
class NrNtnTimingAdvanceTestSuite : public TestSuite
{
  public:
    NrNtnTimingAdvanceTestSuite()
        : TestSuite("nr-ntn-timing-advance", Type::SYSTEM)
    {
        // K_offset must cover the round trip: 2 * 3.6 ms = 7.2 slots at numerology 0
        const uint32_t kOffset = 8;
        AddTestCase(new NrNtnTimingAdvanceTestCase("One UE, 2.0-slot delay, timing advance",
                                                   {MilliSeconds(2)},
                                                   true,
                                                   kOffset,
                                                   true),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrNtnTimingAdvanceTestCase("One UE, 3.6-slot delay, timing advance",
                                                   {MicroSeconds(3600)},
                                                   true,
                                                   kOffset,
                                                   true),
                    TestCase::Duration::QUICK);
        AddTestCase(
            new NrNtnTimingAdvanceTestCase("Two UEs, 2.0- and 3.6-slot delays, timing advance",
                                           {MilliSeconds(2), MicroSeconds(3600)},
                                           true,
                                           kOffset,
                                           true),
            TestCase::Duration::QUICK);
        AddTestCase(new NrNtnTimingAdvanceTestCase("One UE, 2.0-slot delay, no timing advance",
                                                   {MilliSeconds(2)},
                                                   false,
                                                   kOffset,
                                                   false),
                    TestCase::Duration::QUICK);
        // Run-time updates: the UE moves at 0.01 m/s, i.e. the one-way delay drifts by 1 ns
        // every millisecond, and the timing advance is refreshed from the geometry every 10 ms,
        // so it is stale by up to 10 ns. Away from the gNB the advance only grows (every update
        // starts the next slot late on the new grid), towards it the advance only shrinks (every
        // update delays the next slot start). The drift within a slot stays below the 2 ns that
        // separate consecutive frames of one transmitter, which is all the cyclic-prefix-less
        // receiver model tolerates before two frames overlap.
        AddTestCase(new NrNtnTimingAdvanceTestCase(
                        "One UE moving away, 3.6-slot delay, timing advance updated every 10 ms",
                        {MicroSeconds(3600)},
                        true,
                        kOffset,
                        true,
                        0.01,
                        MilliSeconds(10)),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrNtnTimingAdvanceTestCase(
                        "One UE approaching, 3.6-slot delay, timing advance updated every 10 ms",
                        {MicroSeconds(3600)},
                        true,
                        kOffset,
                        true,
                        -0.01,
                        MilliSeconds(10)),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrNtnUplinkAlignmentTestCase(), TestCase::Duration::QUICK);
        // Both UEs move away at 5 ns of delay per ms: the timing advance of each is exact at
        // every slot start only, and the two round differently, so their uplink control misses
        // co-arrival by a nanosecond now and then; without the line-up the gNB aborts
        AddTestCase(new NrNtnTimingAdvanceTestCase("Two UEs moving away, 2.0- and 3.6-slot "
                                                   "delays, timing advance every slot, uplink "
                                                   "lined up",
                                                   {MilliSeconds(2), MicroSeconds(3600)},
                                                   true,
                                                   kOffset,
                                                   true,
                                                   0.05,
                                                   Seconds(0),
                                                   true),
                    TestCase::Duration::QUICK);
    }
};

/// Static variable for test initialization
static NrNtnTimingAdvanceTestSuite g_nrNtnTimingAdvanceTestSuite;
