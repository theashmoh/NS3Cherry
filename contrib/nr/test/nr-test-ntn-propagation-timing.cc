// Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/constant-position-mobility-model.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/node.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ntn-link-models.h"
#include "ns3/nr-spectrum-signal-parameters.h"
#include "ns3/nstime.h"
#include "ns3/simple-net-device.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-phy.h"
#include "ns3/spectrum-value.h"
#include "ns3/test.h"
#include "ns3/vector.h"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

/**
 * @file nr-test-ntn-propagation-timing.cc
 * @ingroup test
 *
 * @brief The standard channel with NrNtnPropagationDelayModel delivers every signal after its
 * physical delay.
 *
 * The gNB is on the satellite. Transparent payload: the signal also crosses the feeder link,
 * GT -> satellite (reversed on the uplink), and pays the relay processing delay. Regenerative
 * payload: the waveform exists only on the satellite<->UE hop and the feeder link and the relay
 * processing no longer enter the radio delay. Both are checked against an independent geometric
 * expectation, never against GetOneWayDelay(), and against a moved endpoint, so a cached delay
 * would fail here.
 */

namespace ns3
{
namespace
{

constexpr double SpeedOfLightMps = 299792458.0;

// Identify the gNB side without constructing an unrelated RRC/MAC stack.
class TimingGnbDevice : public NrGnbNetDevice
{
  private:
    void DoInitialize() override
    {
    }

    void DoDispose() override
    {
        NrNetDevice::DoDispose();
    }
};

// Observe real StartRx events without hiding channel timing behind NR slot scheduling.
class ArrivalPhy : public SpectrumPhy
{
  public:
    void SetDevice(Ptr<NetDevice> device) override
    {
        m_device = device;
    }

    Ptr<NetDevice> GetDevice() const override
    {
        return m_device;
    }

    void SetMobility(Ptr<MobilityModel> mobility) override
    {
        m_mobility = mobility;
    }

    Ptr<MobilityModel> GetMobility() const override
    {
        return m_mobility;
    }

    void SetChannel(Ptr<SpectrumChannel>) override
    {
    }

    Ptr<const SpectrumModel> GetRxSpectrumModel() const override
    {
        return spectrum;
    }

    Ptr<Object> GetAntenna() const override
    {
        return nullptr;
    }

    void StartRx(Ptr<SpectrumSignalParameters> params) override
    {
        arrivals.push_back(Simulator::Now());
        signals.push_back(params);
    }

    Ptr<const SpectrumModel> spectrum;
    std::vector<Time> arrivals;
    std::vector<Ptr<SpectrumSignalParameters>> signals;

  private:
    Ptr<NetDevice> m_device;
    Ptr<MobilityModel> m_mobility;
};

/// The endpoints every case starts from: GT at the origin, the satellite (with the gNB) 1000 km
/// along x at 600 km altitude, UE 300 km off the satellite's nadir. Both devices sit on nodes
/// carrying their mobility, which is how the delay model tells the gNB end from the UE end.
struct Bench
{
    Ptr<ConstantPositionMobilityModel> gt;
    Ptr<ConstantPositionMobilityModel> sat;
    Ptr<ConstantPositionMobilityModel> ue;
    Ptr<MultiModelSpectrumChannel> channel;
    Ptr<NrNtnPropagationDelayModel> delay;
    Ptr<SpectrumModel> model;
    Ptr<ArrivalPhy> gnbPhy;
    Ptr<ArrivalPhy> uePhy;
};

Bench
MakeBench()
{
    auto at = [](Vector position) {
        auto mobility = CreateObject<ConstantPositionMobilityModel>();
        mobility->SetPosition(position);
        return mobility;
    };
    Bench b;
    b.gt = at(Vector(0, 0, 0));
    b.sat = at(Vector(1000000, 0, 600000));
    b.ue = at(Vector(1000000, 300000, 0));
    b.channel = CreateObject<MultiModelSpectrumChannel>();
    b.delay = CreateObject<NrNtnPropagationDelayModel>();
    b.delay->SetAttribute("RelayProcessingDelay", TimeValue(MicroSeconds(250)));
    b.channel->SetPropagationDelayModel(b.delay);

    // ns-3.48 SpectrumModel derives each band edge from its neighbour, so it needs at least two
    // center frequencies (spectrum-model.cc:35); the timing assertions do not depend on the PSD.
    b.model = Create<SpectrumModel>(std::vector<double>{3.5e9, 3.5e9 + 180e3});
    b.gnbPhy = CreateObject<ArrivalPhy>();
    b.uePhy = CreateObject<ArrivalPhy>();
    b.gnbPhy->spectrum = b.model;
    b.uePhy->spectrum = b.model;
    b.gnbPhy->SetMobility(b.sat);
    b.uePhy->SetMobility(b.ue);
    b.gnbPhy->SetDevice(CreateObject<TimingGnbDevice>());
    b.uePhy->SetDevice(CreateObject<SimpleNetDevice>());
    for (auto [phy, mobility] :
         {std::pair<Ptr<ArrivalPhy>, Ptr<MobilityModel>>{b.gnbPhy, b.sat}, {b.uePhy, b.ue}})
    {
        auto node = CreateObject<Node>();
        node->AggregateObject(mobility);
        node->AddDevice(phy->GetDevice());
    }
    return b;
}

/// One NR frame of the requested kind, 10 us long, flat unit PSD.
Ptr<SpectrumSignalParameters>
MakeSignal(bool uplink, bool control, Ptr<SpectrumPhy> tx, Ptr<const SpectrumModel> model)
{
    Ptr<SpectrumSignalParameters> signal;
    if (!control)
    {
        auto data = Create<NrSpectrumSignalParametersDataFrame>();
        data->cellId = 1;
        signal = data;
    }
    else if (uplink)
    {
        auto ctrl = Create<NrSpectrumSignalParametersUlCtrlFrame>();
        ctrl->cellId = 1;
        signal = ctrl;
    }
    else
    {
        auto ctrl = Create<NrSpectrumSignalParametersDlCtrlFrame>();
        ctrl->cellId = 1;
        ctrl->pss = false;
        signal = ctrl;
    }
    signal->txPhy = tx;
    signal->duration = MicroSeconds(10);
    signal->psd = Create<SpectrumValue>(model);
    *signal->psd = 1.0;
    return signal;
}

/// Satellite (1000, 0, 600) km to the UE (1000, 300, 0) km: the service hop of the bench.
const Time g_serviceDelay = Seconds(std::sqrt(450000000000.0) / SpeedOfLightMps);

class NtnPropagationTimingCase : public TestCase
{
  public:
    NtnPropagationTimingCase(bool uplink, bool control)
        : TestCase(std::string("Transparent ") + (uplink ? "UL " : "DL ") +
                   (control ? "control" : "data") + " crosses the feeder link and both relays"),
          m_uplink(uplink),
          m_control(control)
    {
    }

  private:
    void DoRun() override
    {
        Bench b = MakeBench();
        b.delay->SetFeeder(b.gt);
        Ptr<ArrivalPhy> tx = m_uplink ? b.uePhy : b.gnbPhy;
        Ptr<ArrivalPhy> rx = m_uplink ? b.gnbPhy : b.uePhy;
        b.channel->AddRx(rx);
        Ptr<SpectrumSignalParameters> signal = MakeSignal(m_uplink, m_control, tx, b.model);

        // Independent geometric expectation, deliberately not GetOneWayDelay().
        const double feederDistance = std::sqrt(1360000000000.0);
        const double firstDistance = feederDistance + std::sqrt(450000000000.0);
        const Time firstDelay = Seconds(firstDistance / SpeedOfLightMps) + MicroSeconds(250);
        const Time firstTx = MilliSeconds(1);
        Simulator::Schedule(firstTx, &MultiModelSpectrumChannel::StartTx, b.channel, signal);
        Simulator::Schedule(firstTx + firstDelay - NanoSeconds(1), [this, rx]() {
            NS_TEST_ASSERT_MSG_EQ(rx->arrivals.size(),
                                  0,
                                  "Signal arrived before its physical delay");
        });

        // A later transmission must use the new geometry, not a cached initial RTT.
        const Time secondTx = MilliSeconds(100);
        Simulator::Schedule(secondTx, [b, signal]() {
            b.ue->SetPosition(Vector(1000000, 800000, 0));
            b.channel->StartTx(signal);
        });
        const Time secondDelay =
            Seconds((feederDistance + 1000000.0) / SpeedOfLightMps) + MicroSeconds(250);
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(rx->arrivals.size(),
                              2,
                              "Both transmissions must arrive exactly once");
        if (rx->arrivals.size() == 2)
        {
            NS_TEST_ASSERT_MSG_EQ(rx->arrivals[0],
                                  firstTx + firstDelay,
                                  "Initial arrival must include all three hops and two relays");
            NS_TEST_ASSERT_MSG_EQ(rx->arrivals[1],
                                  secondTx + secondDelay,
                                  "Arrival must follow the updated geometry");
            NS_TEST_ASSERT_MSG_EQ(rx->signals[0]->duration,
                                  signal->duration,
                                  "Propagation must preserve signal duration");
        }
        Simulator::Destroy();
    }

    bool m_uplink;
    bool m_control;
};

/**
 * @brief Regenerative: the radio delay is the satellite<->UE hop, in both directions.
 *
 * The feeder link and the relay processing are backhaul, not radio: moving the gateway must
 * leave the arrival time alone, while moving the UE must still move it. The GetOneWayDelay() the
 * examples size K_offset and the timing advance from has to agree with what actually arrived.
 */
class NtnRegenerativeTimingCase : public TestCase
{
  public:
    NtnRegenerativeTimingCase(bool uplink, bool control)
        : TestCase(std::string("Regenerative ") + (uplink ? "UL " : "DL ") +
                   (control ? "control" : "data") + " is delayed by the service hop only"),
          m_uplink(uplink),
          m_control(control)
    {
    }

  private:
    void DoRun() override
    {
        Bench b = MakeBench(); // no feeder link: regenerative
        Ptr<ArrivalPhy> tx = m_uplink ? b.uePhy : b.gnbPhy;
        Ptr<ArrivalPhy> rx = m_uplink ? b.gnbPhy : b.uePhy;
        b.channel->AddRx(rx);
        Ptr<SpectrumSignalParameters> signal = MakeSignal(m_uplink, m_control, tx, b.model);

        // The service hop alone; the bench's 250 us relay processing must not appear.
        const Time firstTx = MilliSeconds(1);
        Simulator::Schedule(firstTx, &MultiModelSpectrumChannel::StartTx, b.channel, signal);
        Simulator::Schedule(firstTx + g_serviceDelay - NanoSeconds(1), [this, rx]() {
            NS_TEST_ASSERT_MSG_EQ(rx->arrivals.size(),
                                  0,
                                  "Signal arrived before the service-hop delay");
        });

        // Moving the gateway changes the feeder link, which a transparent walk would see. The
        // regenerative hop does not touch it.
        const Time secondTx = MilliSeconds(50);
        Simulator::Schedule(secondTx, [b, signal]() {
            b.gt->SetPosition(Vector(5000000, 5000000, 0));
            b.channel->StartTx(signal);
        });

        // The service hop itself still follows the live geometry: 800 km across, 600 km up.
        const Time thirdTx = MilliSeconds(100);
        Simulator::Schedule(thirdTx, [b, signal]() {
            b.ue->SetPosition(Vector(1000000, 800000, 0));
            b.channel->StartTx(signal);
        });
        const Time movedDelay = Seconds(1000000.0 / SpeedOfLightMps);
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(rx->arrivals.size(),
                              3,
                              "All three transmissions must arrive exactly once");
        if (rx->arrivals.size() == 3)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(rx->arrivals[0],
                                      firstTx + g_serviceDelay,
                                      NanoSeconds(1),
                                      "Regenerative arrival must be the service hop alone, with "
                                      "no relay processing");
            NS_TEST_ASSERT_MSG_EQ_TOL(rx->arrivals[1],
                                      secondTx + g_serviceDelay,
                                      NanoSeconds(1),
                                      "Moving the gateway must not change the "
                                      "regenerative delay");
            NS_TEST_ASSERT_MSG_EQ_TOL(rx->arrivals[2],
                                      thirdTx + movedDelay,
                                      NanoSeconds(1),
                                      "The regenerative delay must follow the live service-hop "
                                      "geometry");
            NS_TEST_ASSERT_MSG_EQ(rx->signals[0]->duration,
                                  signal->duration,
                                  "Propagation must preserve signal duration");
        }

        // The delay the timing advance and K_offset use agrees with what arrived.
        NS_TEST_ASSERT_MSG_EQ_TOL(b.delay->GetOneWayDelay(b.sat, b.ue),
                                  movedDelay,
                                  NanoSeconds(1),
                                  "Regenerative GetOneWayDelay() must be the observed delay");
        Simulator::Destroy();
    }

    bool m_uplink;
    bool m_control;
};

class NtnPropagationTimingSuite : public TestSuite
{
  public:
    NtnPropagationTimingSuite()
        : TestSuite("nr-ntn-propagation-timing", Type::UNIT)
    {
        for (bool uplink : {false, true})
        {
            for (bool control : {false, true})
            {
                AddTestCase(new NtnPropagationTimingCase(uplink, control), Duration::QUICK);
                AddTestCase(new NtnRegenerativeTimingCase(uplink, control), Duration::QUICK);
            }
        }
    }
};

static NtnPropagationTimingSuite g_ntnPropagationTimingSuite;

} // namespace
} // namespace ns3
