// Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-ntn-random-access.cc
 *
 * @brief Test suite (nr-ntn-random-access): random access under physical propagation delay.
 *
 * The UE arms its RA response timeout SlotPeriod * (6 + RaResponseWindowSize) after the
 * preamble (9 ms by default at numerology 0), while the RAR comes back one round-trip time
 * plus the gNB's own RAR latency L0 later. The suite measures L0 at zero delay, shows that a
 * round trip beyond the window makes the timeout fire and the preamble retry when
 * NrUeMac::IdealRandomAccess is false and NrUePhy::TimingAdvance is zero, that the timeout
 * shifted by 2 * TimingAdvance (the UE-gNB round-trip time of TS 38.213 clause 8, Rel-17) lets
 * the same random access complete on the first preamble, that IdealRandomAccess never times
 * out, and that a delay that outlives NrGnbRrc::ConnectionRequestTimeoutDuration (15 ms by
 * default, armed when the gNB creates the UE context on preamble reception, before the RAR
 * even leaves) drops the UE context under ideal RRC unless the attribute is raised.
 *
 * The cell is FDD (a DL-only and an UL-only BWP) with a ConstantSpeedPropagationDelayModel
 * whose speed is scaled so that short distances give millisecond delays. Ideal RRC.
 */

#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/double.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-control-messages.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-gnb-rrc.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-mac.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/nstime.h"
#include "ns3/position-allocator.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrNtnRandomAccessTest");

namespace
{

/// Propagation speed of the test channel: 5 m of distance give 1 ms of one-way delay, so the
/// delays of the test are set by the UE position while the log-distance path loss stays small.
constexpr double g_propagationSpeedMps = 5000.0;

/// Default of NrGnbRrc::ConnectionRequestTimeoutDuration, restored before every case
const Time g_defaultConnectionRequestTimeout = MilliSeconds(15);

} // namespace

/**
 * @ingroup nr-test
 *
 * @brief One FDD cell and one UE at a given one-way delay that attaches with ideal RRC; the
 * random access and the gNB's UE context are observed through traces.
 */
class NrNtnRandomAccessTestCase : public TestCase
{
  public:
    /// What the case must show
    enum Outcome
    {
        RA_FIRST_PREAMBLE, ///< RA completes on the first preamble, no timeout
        RA_TIMES_OUT,      ///< the RA response timeout fires and preambles are retried
        CONTEXT_DROPPED,   ///< the gNB drops the UE context before the connection request
        CONTEXT_KEPT       ///< the gNB keeps the UE context and the connection is established
    };

    /**
     * Constructor
     * @param name the name of the test case
     * @param delay one-way propagation delay
     * @param idealRandomAccess NrUeMac::IdealRandomAccess
     * @param timingAdvance set NrUePhy::TimingAdvance to the delay (true) or leave it at 0
     * @param kOffset NrGnbPhy::CellSpecificKoffset
     * @param connectionRequestTimeout NrGnbRrc::ConnectionRequestTimeoutDuration
     * @param outcome what the case must show
     */
    NrNtnRandomAccessTestCase(const std::string& name,
                              Time delay,
                              bool idealRandomAccess,
                              bool timingAdvance,
                              uint32_t kOffset,
                              Time connectionRequestTimeout,
                              Outcome outcome)
        : TestCase(name),
          m_delay(delay),
          m_idealRandomAccess(idealRandomAccess),
          m_timingAdvance(timingAdvance),
          m_kOffset(kOffset),
          m_connectionRequestTimeout(connectionRequestTimeout),
          m_outcome(outcome)
    {
    }

  private:
    void DoRun() override;

    /// NrUeMac::UeMacTxedCtrlMsgsTrace: RACH preambles requested from the PHY
    void UeMacTxed(uint64_t imsi,
                   SfnSf sfn,
                   uint16_t cellId,
                   uint16_t rnti,
                   uint8_t bwpId,
                   Ptr<const NrControlMessage> msg);
    /// NrUeMac::UeMacRxedCtrlMsgsTrace: RARs
    void UeMacRxed(uint64_t imsi,
                   SfnSf sfn,
                   uint16_t cellId,
                   uint16_t rnti,
                   uint8_t bwpId,
                   Ptr<const NrControlMessage> msg);
    /// NrUeMac::RaResponseTimeout
    void RaResponseTimeout(uint64_t imsi, bool contention, uint8_t count, uint8_t max);
    /// NrUeRrc::RandomAccessSuccessful
    void RandomAccessSuccessful(uint64_t imsi, uint16_t cellId, uint16_t rnti);
    /// NrUeRrc::ConnectionEstablished
    void UeConnectionEstablished(uint64_t imsi, uint16_t cellId, uint16_t rnti);
    /// NrGnbRrc::ConnectionEstablished
    void GnbConnectionEstablished(uint64_t imsi, uint16_t cellId, uint16_t rnti);
    /// NrGnbRrc::NewUeContext
    void GnbNewUeContext(uint16_t cellId, uint16_t rnti);
    /// NrGnbRrc::RrcTimeout
    void GnbRrcTimeout(uint64_t imsi, uint16_t cellId, uint16_t rnti, std::string cause);
    /// Scheduled right after a connection request timeout: record whether the context is gone
    void CheckContextAfterTimeout(uint16_t rnti);

    Time m_delay;                    ///< One-way delay
    bool m_idealRandomAccess;        ///< NrUeMac::IdealRandomAccess
    bool m_timingAdvance;            ///< Configure the timing advance
    uint32_t m_kOffset;              ///< Cell-specific K_offset
    Time m_connectionRequestTimeout; ///< NrGnbRrc::ConnectionRequestTimeoutDuration
    Outcome m_outcome;               ///< Expected outcome

    Ptr<NrGnbRrc> m_gnbRrc;                  ///< The gNB RRC, for context checks
    uint32_t m_preambles{0};                 ///< RACH preambles requested by the UE MAC
    uint32_t m_rars{0};                      ///< RARs received by the UE MAC
    uint32_t m_raTimeouts{0};                ///< RA response timeouts
    uint32_t m_raSuccesses{0};               ///< Random access successes at the UE RRC
    uint32_t m_ueConnections{0};             ///< Connections established at the UE RRC
    uint32_t m_gnbConnections{0};            ///< Connections established at the gNB RRC
    uint32_t m_gnbContexts{0};               ///< UE contexts created at the gNB RRC
    uint32_t m_connectionRequestTimeouts{0}; ///< Connection request timeouts at the gNB RRC
    bool m_contextAfterTimeout{true};        ///< The UE context still exists after the timeout
    Time m_firstPreamble{Time::Min()};       ///< When the first preamble was requested
    Time m_firstRar{Time::Min()};            ///< When the first RAR was received
};

void
NrNtnRandomAccessTestCase::UeMacTxed([[maybe_unused]] uint64_t imsi,
                                     [[maybe_unused]] SfnSf sfn,
                                     [[maybe_unused]] uint16_t cellId,
                                     [[maybe_unused]] uint16_t rnti,
                                     [[maybe_unused]] uint8_t bwpId,
                                     Ptr<const NrControlMessage> msg)
{
    if (msg->GetMessageType() != NrControlMessage::RACH_PREAMBLE)
    {
        return;
    }
    if (m_preambles == 0)
    {
        m_firstPreamble = Simulator::Now();
    }
    m_preambles++;
    NS_LOG_INFO("Preamble " << m_preambles << " at " << Simulator::Now().As(Time::MS));
}

void
NrNtnRandomAccessTestCase::UeMacRxed([[maybe_unused]] uint64_t imsi,
                                     [[maybe_unused]] SfnSf sfn,
                                     [[maybe_unused]] uint16_t cellId,
                                     [[maybe_unused]] uint16_t rnti,
                                     [[maybe_unused]] uint8_t bwpId,
                                     Ptr<const NrControlMessage> msg)
{
    if (msg->GetMessageType() != NrControlMessage::RAR)
    {
        return;
    }
    if (m_rars == 0)
    {
        m_firstRar = Simulator::Now();
    }
    m_rars++;
    NS_LOG_INFO("RAR " << m_rars << " at " << Simulator::Now().As(Time::MS));
}

void
NrNtnRandomAccessTestCase::RaResponseTimeout([[maybe_unused]] uint64_t imsi,
                                             [[maybe_unused]] bool contention,
                                             uint8_t count,
                                             uint8_t max)
{
    m_raTimeouts++;
    NS_LOG_INFO("RA response timeout " << m_raTimeouts << " at " << Simulator::Now().As(Time::MS)
                                       << " (preamble transmission " << +count << " of " << +max
                                       << ")");
}

void
NrNtnRandomAccessTestCase::RandomAccessSuccessful([[maybe_unused]] uint64_t imsi,
                                                  [[maybe_unused]] uint16_t cellId,
                                                  [[maybe_unused]] uint16_t rnti)
{
    m_raSuccesses++;
}

void
NrNtnRandomAccessTestCase::UeConnectionEstablished([[maybe_unused]] uint64_t imsi,
                                                   [[maybe_unused]] uint16_t cellId,
                                                   [[maybe_unused]] uint16_t rnti)
{
    m_ueConnections++;
}

void
NrNtnRandomAccessTestCase::GnbConnectionEstablished([[maybe_unused]] uint64_t imsi,
                                                    [[maybe_unused]] uint16_t cellId,
                                                    [[maybe_unused]] uint16_t rnti)
{
    m_gnbConnections++;
}

void
NrNtnRandomAccessTestCase::GnbNewUeContext([[maybe_unused]] uint16_t cellId,
                                           [[maybe_unused]] uint16_t rnti)
{
    m_gnbContexts++;
}

void
NrNtnRandomAccessTestCase::GnbRrcTimeout([[maybe_unused]] uint64_t imsi,
                                         [[maybe_unused]] uint16_t cellId,
                                         uint16_t rnti,
                                         std::string cause)
{
    if (cause != "ConnectionRequestTimeout")
    {
        return;
    }
    m_connectionRequestTimeouts++;
    NS_LOG_INFO("Connection request timeout for RNTI " << rnti << " at "
                                                       << Simulator::Now().As(Time::MS));
    if (m_outcome == CONTEXT_DROPPED)
    {
        // The context is removed right after this trace. The RAR is already on its way to the
        // UE, whose ideal RRC connection request would then reach a gNB that has no UE manager
        // for the RNTI (NrGnbRrc::DoRecvRrcConnectionRequest does not check for it), so the
        // case ends here, once the removal is observable.
        Simulator::ScheduleNow(&NrNtnRandomAccessTestCase::CheckContextAfterTimeout, this, rnti);
    }
}

void
NrNtnRandomAccessTestCase::CheckContextAfterTimeout(uint16_t rnti)
{
    m_contextAfterTimeout = m_gnbRrc->HasUeManager(rnti);
    Simulator::Stop();
}

void
NrNtnRandomAccessTestCase::DoRun()
{
    Config::SetDefault("ns3::NrGnbRrc::ConnectionRequestTimeoutDuration",
                       TimeValue(m_connectionRequestTimeout));

    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetGnbPhyAttribute("CellSpecificKoffset", UintegerValue(m_kOffset));
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(0));
    nrHelper->SetUeMacAttribute("IdealRandomAccess", BooleanValue(m_idealRandomAccess));
    if (m_timingAdvance)
    {
        nrHelper->SetUePhyAttribute("TimingAdvance", TimeValue(m_delay));
    }

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

    for (const auto& bwp : allBwps)
    {
        auto delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
        delayModel->SetSpeed(g_propagationSpeedMps);
        bwp.get()->GetChannel()->SetPropagationDelayModel(delayModel);
    }

    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(0));

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    auto posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));
    posAlloc->Add(Vector(m_delay.GetSeconds() * g_propagationSpeedMps, 0, 0));
    MobilityHelper mobilityHelper;
    mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityHelper.SetPositionAllocator(posAlloc);
    mobilityHelper.Install(gnbNodes);
    mobilityHelper.Install(ueNodes);

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)
        ->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 1)
        ->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));

    for (uint32_t bwp = 0; bwp < 2; ++bwp)
    {
        auto ueMac = NrHelper::GetUeMac(ueDevs.Get(0), bwp);
        ueMac->TraceConnectWithoutContext(
            "UeMacTxedCtrlMsgsTrace",
            MakeCallback(&NrNtnRandomAccessTestCase::UeMacTxed, this));
        ueMac->TraceConnectWithoutContext(
            "UeMacRxedCtrlMsgsTrace",
            MakeCallback(&NrNtnRandomAccessTestCase::UeMacRxed, this));
        ueMac->TraceConnectWithoutContext(
            "RaResponseTimeout",
            MakeCallback(&NrNtnRandomAccessTestCase::RaResponseTimeout, this));
    }
    auto ueRrc = DynamicCast<NrUeNetDevice>(ueDevs.Get(0))->GetRrc();
    ueRrc->TraceConnectWithoutContext(
        "RandomAccessSuccessful",
        MakeCallback(&NrNtnRandomAccessTestCase::RandomAccessSuccessful, this));
    ueRrc->TraceConnectWithoutContext(
        "ConnectionEstablished",
        MakeCallback(&NrNtnRandomAccessTestCase::UeConnectionEstablished, this));
    m_gnbRrc = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(0))->GetRrc();
    m_gnbRrc->TraceConnectWithoutContext(
        "ConnectionEstablished",
        MakeCallback(&NrNtnRandomAccessTestCase::GnbConnectionEstablished, this));
    m_gnbRrc->TraceConnectWithoutContext(
        "NewUeContext",
        MakeCallback(&NrNtnRandomAccessTestCase::GnbNewUeContext, this));
    m_gnbRrc->TraceConnectWithoutContext(
        "RrcTimeout",
        MakeCallback(&NrNtnRandomAccessTestCase::GnbRrcTimeout, this));

    InternetStackHelper inetStackHelper;
    inetStackHelper.Install(ueNodes);
    Ipv4InterfaceContainer ueIfs = nrEpcHelper->AssignUeIpv4Address(ueDevs);
    nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.000));

    nrHelper->AttachToGnb(ueDevs.Get(0), gnbDevs.Get(0));

    Simulator::Stop(MilliSeconds(300));
    Simulator::Run();

    const Time rarLatency = (m_firstRar > m_firstPreamble) ? m_firstRar - m_firstPreamble : Time(0);
    NS_LOG_INFO(GetName() << ": preambles " << m_preambles << ", RARs " << m_rars
                          << ", RA timeouts " << m_raTimeouts << ", RA successes " << m_raSuccesses
                          << ", UE connections " << m_ueConnections << ", gNB contexts "
                          << m_gnbContexts << ", gNB connections " << m_gnbConnections
                          << ", connection request timeouts " << m_connectionRequestTimeouts
                          << ", first RAR " << rarLatency.As(Time::MS) << " after the preamble");
    if (m_delay.IsZero() && m_rars > 0)
    {
        NS_LOG_UNCOND("nr-ntn-random-access: zero-delay RAR latency L0 = "
                      << rarLatency.As(Time::US) << " (preamble to RAR, "
                      << (m_idealRandomAccess ? "ideal" : "realistic") << " random access)");
    }

    NS_TEST_ASSERT_MSG_GT(m_preambles, 0, "The UE sent no RACH preamble");
    NS_TEST_ASSERT_MSG_GT(m_gnbContexts, 0, "The gNB created no UE context");

    switch (m_outcome)
    {
    case RA_FIRST_PREAMBLE:
        NS_TEST_ASSERT_MSG_EQ(m_raTimeouts, 0, "The RA response timeout fired");
        NS_TEST_ASSERT_MSG_EQ(m_preambles, 1, "The preamble was retransmitted");
        NS_TEST_ASSERT_MSG_EQ(m_raSuccesses, 1, "Random access did not complete once");
        NS_TEST_ASSERT_MSG_EQ(m_ueConnections, 1, "The UE did not connect");
        NS_TEST_ASSERT_MSG_EQ(m_gnbConnections, 1, "The gNB did not report the connection");
        break;
    case RA_TIMES_OUT:
        NS_TEST_ASSERT_MSG_GT(m_raTimeouts, 0, "The RA response timeout did not fire");
        NS_TEST_ASSERT_MSG_GT(m_preambles, 1, "The preamble was not retransmitted");
        NS_TEST_ASSERT_MSG_EQ(m_raSuccesses,
                              0,
                              "Random access completed although the RAR cannot fit the window");
        NS_TEST_ASSERT_MSG_EQ(m_ueConnections, 0, "The UE connected");
        break;
    case CONTEXT_DROPPED:
        NS_TEST_ASSERT_MSG_GT(m_connectionRequestTimeouts,
                              0,
                              "NrGnbRrc::ConnectionRequestTimeoutDuration did not expire");
        NS_TEST_ASSERT_MSG_EQ(m_contextAfterTimeout,
                              false,
                              "The gNB kept the UE context after the timeout");
        NS_TEST_ASSERT_MSG_EQ(m_gnbConnections, 0, "The gNB established the connection");
        NS_TEST_ASSERT_MSG_EQ(m_ueConnections, 0, "The UE established the connection");
        break;
    case CONTEXT_KEPT:
        NS_TEST_ASSERT_MSG_EQ(m_connectionRequestTimeouts,
                              0,
                              "NrGnbRrc::ConnectionRequestTimeoutDuration expired");
        NS_TEST_ASSERT_MSG_EQ(m_raTimeouts, 0, "The RA response timeout fired");
        NS_TEST_ASSERT_MSG_EQ(m_raSuccesses, 1, "Random access did not complete once");
        NS_TEST_ASSERT_MSG_EQ(m_gnbConnections, 1, "The gNB did not establish the connection");
        NS_TEST_ASSERT_MSG_EQ(m_ueConnections, 1, "The UE did not establish the connection");
        NS_TEST_ASSERT_MSG_EQ(m_gnbRrc->GetUeMap().size(),
                              1,
                              "The gNB does not hold one UE context");
        break;
    }

    Simulator::Destroy();
    Config::SetDefault("ns3::NrGnbRrc::ConnectionRequestTimeoutDuration",
                       TimeValue(g_defaultConnectionRequestTimeout));
}

/**
 * @ingroup nr-test
 * @brief Test suite nr-ntn-random-access
 */
class NrNtnRandomAccessTestSuite : public TestSuite
{
  public:
    NrNtnRandomAccessTestSuite()
        : TestSuite("nr-ntn-random-access", Type::SYSTEM)
    {
        using TC = NrNtnRandomAccessTestCase;
        const Time defaultTimeout = g_defaultConnectionRequestTimeout;

        // (a) zero delay: RA completes within the default window; L0 is logged
        AddTestCase(new TC("Zero delay, ideal random access",
                           Seconds(0),
                           true,
                           false,
                           0,
                           defaultTimeout,
                           TC::RA_FIRST_PREAMBLE),
                    TestCase::Duration::QUICK);
        AddTestCase(new TC("Zero delay, realistic random access",
                           Seconds(0),
                           false,
                           false,
                           0,
                           defaultTimeout,
                           TC::RA_FIRST_PREAMBLE),
                    TestCase::Duration::QUICK);

        // (b)-(d) a 5 ms one-way delay: the 10 ms round trip plus L0 exceeds the 9 ms window.
        // K_offset covers the round trip (10 slots) for the Msg3 grant.
        const Time delay = MilliSeconds(5);
        const uint32_t kOffset = 12;
        AddTestCase(new TC("5 ms delay, realistic random access, no timing advance: times out",
                           delay,
                           false,
                           false,
                           kOffset,
                           defaultTimeout,
                           TC::RA_TIMES_OUT),
                    TestCase::Duration::QUICK);
        AddTestCase(new TC("5 ms delay, realistic random access, timing advance: first preamble",
                           delay,
                           false,
                           true,
                           kOffset,
                           defaultTimeout,
                           TC::RA_FIRST_PREAMBLE),
                    TestCase::Duration::QUICK);
        AddTestCase(new TC("5 ms delay, ideal random access, no timing advance: first preamble",
                           delay,
                           true,
                           false,
                           kOffset,
                           defaultTimeout,
                           TC::RA_FIRST_PREAMBLE),
                    TestCase::Duration::QUICK);

        // (e) a 16 ms one-way delay outlives the default 15 ms ConnectionRequestTimeoutDuration,
        // which the gNB arms on preamble reception: the RAR is still in flight when the context
        // is dropped. RTT + 20 ms keeps it. K_offset covers the 32-slot round trip.
        const Time longDelay = MilliSeconds(16);
        const uint32_t longKoffset = 34;
        AddTestCase(new TC("16 ms delay, default ConnectionRequestTimeoutDuration: context dropped",
                           longDelay,
                           true,
                           true,
                           longKoffset,
                           defaultTimeout,
                           TC::CONTEXT_DROPPED),
                    TestCase::Duration::QUICK);
        AddTestCase(new TC("16 ms delay, ConnectionRequestTimeoutDuration = RTT + 20 ms: kept",
                           longDelay,
                           true,
                           true,
                           longKoffset,
                           2 * longDelay + MilliSeconds(20),
                           TC::CONTEXT_KEPT),
                    TestCase::Duration::QUICK);
    }
};

/// Static variable for test initialization
static NrNtnRandomAccessTestSuite g_nrNtnRandomAccessTestSuite;
