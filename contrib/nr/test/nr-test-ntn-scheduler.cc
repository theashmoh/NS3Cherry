// Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/boolean.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/ideal-beamforming-algorithm.h"
#include "ns3/ideal-beamforming-helper.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nr-amc.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-gnb-rrc.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-ue-info.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nstime.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/type-id.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/uinteger.h"
#include "ns3/vector.h"

#include <cmath>
#include <limits>
#include <string>

/**
 * @file nr-test-ntn-scheduler.cc
 * @ingroup test
 *
 * @brief Validation of the NTN loop-delay behaviour of the scheduler, HARQ and RLC.
 *
 * The NTN timing in this fork hangs off one attribute: NrGnbPhy::CellSpecificKoffset, the
 * cell-specific scheduling offset of TS 38.213 cl. 4.2 (SIB19 cellSpecificKoffset-r17).
 * It moves the scheduled HARQ feedback (K1), the UL grant (K2) and the Msg3 grant by
 * K_offset reference slots of 1 ms, and the scheduler holds a HARQ process for
 * HarqProcessTimeout + K_offset slots before reaping it (NrMacSchedulerNs3::
 * ResetExpiredHARQ). Everything downstream of that - how long a HARQ process stays
 * busy, how deep the process pool has to be, and whether RLC still closes its
 * return-link conversation - is what these cases check. The terrestrial channel is
 * deliberate: the link budget and the physical delay are covered by nr-ntn-link-models
 * and nr-ntn-propagation-timing, so here only the scheduling offset varies, and the
 * arms with K_offset 0 are the terrestrial controls.
 *
 * The previous release injected the loop RTT as a delay on every uplink control
 * message instead (NrGnbMac::SetUlCtrlRttCallback, removed). Under K_offset the
 * feedback for a block sent in slot n is scheduled at n + K1 + K_offset rather than
 * arriving K_offset slots after n + K1; the gNB sees it in the same slot either way,
 * so the numeric expectations below carry over from that release.
 */

namespace ns3
{

namespace
{

struct NtnScenarioConfig
{
    uint32_t kOffset{0};              //!< NrGnbPhy::CellSpecificKoffset, in reference slots
    uint32_t numHarqProcess{16};      //!< HARQ process pool depth, H
    uint32_t harqProcessTimeout{0};   //!< slots a process is held beyond K_offset; 0 = H
    std::string rlcMode{"um"};        //!< "um" or "am"
    uint8_t fixedMcs{14};             //!< fixed DL MCS, so results depend on timing only
    double offeredMbps{100.0};        //!< offered downlink load
    Time duration{MilliSeconds(600)}; //!< simulated time
};

struct NtnScenarioResult
{
    uint64_t rxBytes{0}; //!< bytes delivered to the UE application
};

/**
 * @brief One gNB, one UE, full-buffer DL, with the NTN scheduling offset at the gNB PHY.
 */
NtnScenarioResult
RunNtnScenario(const NtnScenarioConfig& cfg)
{
    Config::Reset();

    // 15 kHz slots: one slot is 1 ms, so a slot count reads directly as milliseconds.
    const uint32_t numerology = 0;
    const double centralFrequency = 2e9;
    const double bandwidth = 20e6;

    // The NTN seam: the cell-specific K_offset delays the scheduled HARQ feedback (K1),
    // the UL grant (K2) and Msg3 by cfg.kOffset reference slots. At numerology 0 that is
    // cfg.kOffset ms, the loop delay it stands in for. Must be set before the PHY exists.
    Config::SetDefault("ns3::NrGnbPhy::CellSpecificKoffset", UintegerValue(cfg.kOffset));

    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(1);
    ueNodes.Create(1);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);
    gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 25.0));
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 100.0, 1.5));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    channelHelper->ConfigureFactories("UMa", "LOS");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    beamformingHelper->SetAttribute("BeamformingMethod",
                                    TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(40.0));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Fixed MCS: the cases below compare timing, and an adaptive MCS would fold link
    // adaptation into every difference.
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(true));
    nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(cfg.fixedMcs));
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(true));
    nrHelper->SetSchedulerAttribute("HarqProcessTimeout", UintegerValue(cfg.harqProcessTimeout));
    nrHelper->SetGnbMacAttribute("NumHarqProcess", UintegerValue(cfg.numHarqProcess));
    nrHelper->SetUeMacAttribute("NumHarqProcess", UintegerValue(cfg.numHarqProcess));

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(10 * 1024 * 1024));
    if (cfg.rlcMode == "am")
    {
        Config::SetDefault("ns3::NrGnbRrc::QosFlowToRlcMapping",
                           EnumValue(NrGnbRrc::RLC_AM_ALWAYS));
        Config::SetDefault("ns3::NrRlcAm::MaxTxBufferSize", UintegerValue(10 * 1024 * 1024));
        Config::SetDefault("ns3::NrRlcAm::TxOpportunityForRetxAlwaysBigEnough", BooleanValue(true));
        // The AM timers default to 20/10/10 ms, all shorter than an NTN loop: sized from the
        // scheduling offset (K_offset slots of 1 ms at numerology 0) so AM does not declare
        // loss on blocks whose status report is still waiting for its offset UL grant.
        const Time loop = MilliSeconds(cfg.kOffset);
        Config::SetDefault("ns3::NrRlcAm::PollRetransmitTimer", TimeValue(loop + MilliSeconds(20)));
        Config::SetDefault("ns3::NrRlcAm::ReorderingTimer", TimeValue(MilliSeconds(20)));
        Config::SetDefault("ns3::NrRlcAm::StatusProhibitTimer", TimeValue(MilliSeconds(10)));
    }

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    auto [remoteHost, remoteHostAddress] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0));
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpInterface =
        nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);

    const uint16_t dlPort = 1234;
    const uint32_t packetSize = 1000;
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer serverApps = sinkHelper.Install(ueNodes.Get(0));

    // Offered load well above what the link can carry, so the result is capacity, not demand.
    UdpClientHelper clientHelper(ueIpInterface.GetAddress(0), dlPort);
    clientHelper.SetAttribute("PacketSize", UintegerValue(packetSize));
    clientHelper.SetAttribute("MaxPackets", UintegerValue(std::numeric_limits<uint32_t>::max()));
    clientHelper.SetAttribute("Interval",
                              TimeValue(Seconds(packetSize * 8.0 / (cfg.offeredMbps * 1e6))));
    ApplicationContainer clientApps = clientHelper.Install(remoteHost);

    serverApps.Start(MilliSeconds(100));
    clientApps.Start(MilliSeconds(200));
    clientApps.Stop(cfg.duration);

    Simulator::Stop(cfg.duration + MilliSeconds(100));
    Simulator::Run();

    NtnScenarioResult result;
    result.rxBytes = serverApps.Get(0)->GetObject<PacketSink>()->GetTotalRx();

    Simulator::Destroy();
    Config::Reset();
    return result;
}

} // namespace

/**
 * @brief The OLLA offset corrects the CSI-derived MCS and stays inside its clamp.
 */
class NrNtnOllaOffsetTestCase : public TestCase
{
  public:
    NrNtnOllaOffsetTestCase()
        : TestCase("OLLA offset shifts the CSI MCS and respects its clamp")
    {
    }

  private:
    void DoRun() override
    {
        auto rbPerRbg = []() -> uint32_t { return 1; };
        NrMacSchedulerUeInfo ue(1, BeamId(0, 0.0), rbPerRbg);
        ue.m_dlAmc = CreateObject<NrAmc>();
        ue.m_dlMcs = 12;

        // OllaMaxOffset 0 disables the loop: upstream behaviour, CSI MCS passed through.
        ue.m_dlOllaMax = 0.0;
        ue.m_dlOlla = 3.0;
        NS_TEST_ASSERT_MSG_EQ(+ue.GetDlMcs(), 12, "A zero clamp must disable OLLA entirely");

        ue.m_dlOllaMax = 4.0;
        NS_TEST_ASSERT_MSG_EQ(+ue.GetDlMcs(), 15, "A +3 offset must raise the CSI MCS by 3");
        NS_TEST_ASSERT_MSG_EQ(+ue.GetDlMcsFromCsi(),
                              12,
                              "The CSI MCS itself must stay free of the OLLA offset");

        ue.m_dlOlla = -3.4;
        NS_TEST_ASSERT_MSG_EQ(+ue.GetDlMcs(), 9, "A -3.4 offset must round to the nearest MCS");

        // The offset is a correction, not a way out of the MCS table.
        ue.m_dlOllaMax = 40.0;
        ue.m_dlOlla = -30.0;
        NS_TEST_ASSERT_MSG_EQ(+ue.GetDlMcs(), 0, "The corrected MCS must not fall below 0");
        ue.m_dlOlla = 30.0;
        NS_TEST_ASSERT_MSG_EQ(+ue.GetDlMcs(),
                              ue.m_dlAmc->GetMaxMcs(),
                              "The corrected MCS must not exceed the table maximum");
    }
};

/**
 * @brief A HARQ pool shallower than the scheduling offset starves the downlink.
 *
 * With H processes and a 1 ms slot, a stop-and-wait pool covers H ms of pipeline. Once
 * the feedback for a block is scheduled K1 + K_offset slots after it, the scheduler runs
 * out of processes and the link idles waiting for feedback: throughput falls even though
 * nothing about the radio changed.
 */
class NrNtnHarqWindowTestCase : public TestCase
{
  public:
    NrNtnHarqWindowTestCase()
        : TestCase("HARQ pool depth bounds throughput once it is shorter than K_offset")
    {
    }

  private:
    void DoRun() override
    {
        NtnScenarioConfig deep;
        deep.kOffset = 20;
        deep.numHarqProcess = 32;
        deep.harqProcessTimeout = 40;
        const auto deepResult = RunNtnScenario(deep);

        NtnScenarioConfig shallow = deep;
        shallow.numHarqProcess = 4;
        const auto shallowResult = RunNtnScenario(shallow);

        NS_TEST_ASSERT_MSG_GT(deepResult.rxBytes, 0, "The deep-pool run must deliver data");
        NS_TEST_ASSERT_MSG_GT(shallowResult.rxBytes, 0, "The shallow-pool run must deliver data");
        NS_TEST_ASSERT_MSG_GT(deepResult.rxBytes,
                              shallowResult.rxBytes,
                              "A 4-process pool must throttle a 20-slot offset that 32 processes "
                              "keep full");

        // Same pools without the offset. 32 processes cover a 20-slot offset, so the deep run
        // must be indifferent to it; the 4-process run must not be.
        NtnScenarioConfig terrestrialDeep = deep;
        terrestrialDeep.kOffset = 0;
        NtnScenarioConfig terrestrialShallow = shallow;
        terrestrialShallow.kOffset = 0;
        const auto terrestrialDeepBytes = RunNtnScenario(terrestrialDeep).rxBytes;
        const auto terrestrialShallowBytes = RunNtnScenario(terrestrialShallow).rxBytes;

        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(deepResult.rxBytes),
                                  static_cast<double>(terrestrialDeepBytes),
                                  0.05 * terrestrialDeepBytes,
                                  "A pool deeper than the offset must hide the offset entirely");
        NS_TEST_ASSERT_MSG_GT(terrestrialShallowBytes,
                              2 * shallowResult.rxBytes,
                              "The same shallow pool must collapse once K_offset is 20 slots");

        // Stop-and-wait over H processes carries H slots of data per feedback cycle, so
        // throughput scales as H x slot / cycle. H=4, 1 ms slots, a cycle of K1 + K_offset =
        // 22 slots plus the few slots of scheduling latency: about a sixth of the
        // unthrottled rate.
        const double stalledFraction =
            static_cast<double>(shallowResult.rxBytes) / static_cast<double>(deepResult.rxBytes);
        NS_TEST_ASSERT_MSG_EQ_TOL(stalledFraction,
                                  4.0 / 25.0,
                                  0.06,
                                  "Throughput under a starved pool must follow H x slot / cycle");
    }
};

/**
 * @brief RLC carries its full load over the NTN offset in both modes.
 *
 * AM adds a return-link conversation - polls, status reports, a window that only advances
 * on them - and every uplink leg of it now waits K_offset slots for its grant. This checks
 * that the conversation still closes: AM delivers what UM delivers, both at an offered load
 * the link can carry and at one it cannot.
 *
 * Not covered: recovery of blocks HARQ gives up on. RLC AM skips a retransmission that
 * does not fit its grant instead of resegmenting it, so with HARQ retransmissions
 * disabled its window stalls instead of healing the gap. Asserting that here would pin
 * the defect in place; see the RLC AM notes in the repository.
 */
class NrNtnRlcModeTestCase : public TestCase
{
  public:
    NrNtnRlcModeTestCase()
        : TestCase("RLC AM matches UM over a 20-slot K_offset")
    {
    }

  private:
    void DoRun() override
    {
        NtnScenarioConfig um;
        um.kOffset = 20;
        um.numHarqProcess = 32;
        um.harqProcessTimeout = 40;
        um.offeredMbps = 5.0; // well inside what the link carries
        const auto umOffered = RunNtnScenario(um);

        NtnScenarioConfig am = um;
        am.rlcMode = "am";
        const auto amOffered = RunNtnScenario(am);

        // 5 Mbps offered from 200 ms to 600 ms, in 1000-byte datagrams.
        const uint64_t expectedBytes = 250000;
        NS_TEST_ASSERT_MSG_EQ(umOffered.rxBytes,
                              expectedBytes,
                              "RLC UM must deliver the whole offered load over the offset");
        NS_TEST_ASSERT_MSG_EQ(amOffered.rxBytes,
                              expectedBytes,
                              "RLC AM must deliver the whole offered load over the offset");

        // Saturated: AM now pays for its status reports out of the same grants.
        NtnScenarioConfig umFull = um;
        umFull.offeredMbps = 100.0;
        NtnScenarioConfig amFull = am;
        amFull.offeredMbps = 100.0;
        const auto umFullBytes = RunNtnScenario(umFull).rxBytes;
        const auto amFullBytes = RunNtnScenario(amFull).rxBytes;

        NS_TEST_ASSERT_MSG_GT(umFullBytes, expectedBytes, "The saturated run must carry more");
        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(amFullBytes),
                                  static_cast<double>(umFullBytes),
                                  0.02 * umFullBytes,
                                  "AM's return-link overhead must not cost throughput over the "
                                  "offset");
    }
};

/**
 * @brief At good SINR the offset costs nothing and throughput tracks the MCS.
 *
 * The NTN cases above all measure what a scheduling offset takes away. This one measures
 * what must survive it: with a pool deeper than the offset and a clean link, the delivered
 * rate has to be the one the AMC's transport block sizes promise, and the same one a
 * terrestrial run gets.
 */
class NrNtnGoodSinrThroughputTestCase : public TestCase
{
  public:
    NrNtnGoodSinrThroughputTestCase()
        : TestCase("Good-SINR throughput tracks the AMC and survives the offset")
    {
    }

  private:
    void DoRun() override
    {
        NtnScenarioConfig base;
        base.kOffset = 20;
        base.numHarqProcess = 32;
        base.harqProcessTimeout = 40;
        base.offeredMbps = 200.0; // above capacity: the result is the link, not the demand

        NtnScenarioConfig low = base;
        low.fixedMcs = 14;
        NtnScenarioConfig high = base;
        high.fixedMcs = 27;
        NtnScenarioConfig terrestrialHigh = high;
        terrestrialHigh.kOffset = 0;

        const auto lowBytes = RunNtnScenario(low).rxBytes;
        const auto highBytes = RunNtnScenario(high).rxBytes;
        const auto terrestrialHighBytes = RunNtnScenario(terrestrialHigh).rxBytes;

        // The delivered rate must be the one the AMC's transport block sizes promise: a
        // regression in the MCS table or in TB sizing shows up here as a ratio that no
        // longer matches, whatever the absolute numbers do.
        Ptr<NrAmc> amc = CreateObject<NrAmc>();
        const double tbSizeRatio = static_cast<double>(amc->CalculateTbSize(27, 1, 100)) /
                                   static_cast<double>(amc->CalculateTbSize(14, 1, 100));
        const double measuredRatio = static_cast<double>(highBytes) / static_cast<double>(lowBytes);
        NS_TEST_ASSERT_MSG_EQ_TOL(measuredRatio,
                                  tbSizeRatio,
                                  0.03 * tbSizeRatio,
                                  "MCS 27 against MCS 14 must scale like their TB sizes");

        // Absolute floor: 20 MHz at MCS 27 is about 4.5 bit/s/Hz once the TDD pattern and
        // the control overhead are paid for. Anything far below means the link is being
        // throttled somewhere other than the radio.
        const double durationS = (base.duration - MilliSeconds(200)).GetSeconds();
        const double highMbps = highBytes * 8.0 / durationS / 1e6;
        NS_TEST_ASSERT_MSG_GT(highMbps, 80.0, "MCS 27 on 20 MHz must carry more than 80 Mbit/s");

        // And none of it is spent on the offset: 32 processes cover 20 slots, so the NTN run
        // must match the terrestrial one.
        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(highBytes),
                                  static_cast<double>(terrestrialHighBytes),
                                  0.01 * terrestrialHighBytes,
                                  "A 20-slot offset must cost nothing once the pool covers it");
    }
};

/**
 * @brief NTN scheduler, HARQ and RLC validation suite.
 */
class NrNtnSchedulerTestSuite : public TestSuite
{
  public:
    NrNtnSchedulerTestSuite()
        : TestSuite("nr-ntn-scheduler", Type::SYSTEM)
    {
        AddTestCase(new NrNtnOllaOffsetTestCase, Duration::QUICK);
        AddTestCase(new NrNtnHarqWindowTestCase, Duration::EXTENSIVE);
        AddTestCase(new NrNtnRlcModeTestCase, Duration::EXTENSIVE);
        AddTestCase(new NrNtnGoodSinrThroughputTestCase, Duration::EXTENSIVE);
    }
};

static NrNtnSchedulerTestSuite g_nrNtnSchedulerTestSuite;

} // namespace ns3
