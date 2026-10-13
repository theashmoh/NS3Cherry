// Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/double.h"
#include "ns3/nr-ntn-link-models.h"
#include "ns3/nstime.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/vector.h"

#include <cmath>

/**
 * @file nr-test-ntn-link-models.cc
 * @ingroup test
 *
 * @brief Validation of the NTN link budget and loop delay of NrNtnFeederNoiseLossModel and
 * NrNtnPropagationDelayModel.
 *
 * The gNB sits on the serving satellite. A transparent payload adds the feeder link, gateway ->
 * satellite, to the delay and charges the noise the relay forwards to the downlink budget; a
 * regenerative one is the satellite <-> UE hop alone. These cases check the
 * closed-form results against hand-computed geometry, so a regression in the cascade, in the
 * amplify-and-forward noise accounting or in the regenerative delay fails here instead of
 * silently shifting every NTN throughput number.
 */

namespace ns3
{

namespace
{

constexpr double SpeedOfLightMps = 299792458.0;
constexpr double FrequencyHz = 3.5e9;

/// Free-space path loss in dB, what FriisPropagationLossModel gives at FrequencyHz
double
Fspl(double distanceM)
{
    return 20.0 * std::log10((4.0 * M_PI * distanceM * FrequencyHz) / SpeedOfLightMps);
}

Ptr<ConstantPositionMobilityModel>
At(double x, double y, double z)
{
    Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
    m->SetPosition(Vector(x, y, z));
    return m;
}

/// Geometry shared by the cases: gateway and UE on different sides of the satellite, so a
/// swapped hop shows up. The gNB is on the satellite.
struct Chain
{
    Chain()
        : gt(At(0, 0, 0)),
          sat(At(0, 0, 600e3)),
          ue(At(0, 300e3, 0))
    {
    }

    double FeederM() const
    {
        return 600e3;
    }

    double ServiceM() const
    {
        return std::sqrt(300e3 * 300e3 + 600e3 * 600e3);
    }

    Ptr<ConstantPositionMobilityModel> gt;
    Ptr<ConstantPositionMobilityModel> sat;
    Ptr<ConstantPositionMobilityModel> ue;
};

Ptr<PropagationLossModel>
Friis()
{
    Ptr<FriisPropagationLossModel> friis = CreateObject<FriisPropagationLossModel>();
    friis->SetFrequency(FrequencyHz);
    return friis;
}

} // namespace

/**
 * @brief Transparent: the one-way delay crosses the feeder and the service link and the relay;
 * regenerative: the service link alone.
 */
class NrNtnLoopDelayTestCase : public TestCase
{
  public:
    NrNtnLoopDelayTestCase()
        : TestCase("NTN one-way delay follows the transparent chain or the regenerative hop")
    {
    }

  private:
    void DoRun() override
    {
        Chain chain;
        Ptr<NrNtnPropagationDelayModel> delay = CreateObject<NrNtnPropagationDelayModel>();

        // Regenerative: the service hop, and the feeder geometry is irrelevant
        const double serviceS = chain.ServiceM() / SpeedOfLightMps;
        NS_TEST_ASSERT_MSG_EQ_TOL(delay->GetOneWayDelay(chain.sat, chain.ue).GetSeconds(),
                                  serviceS,
                                  1e-9,
                                  "Regenerative delay must be satellite <-> UE / c");
        delay->SetAttribute("RelayProcessingDelay", TimeValue(MicroSeconds(250)));
        NS_TEST_ASSERT_MSG_EQ_TOL(delay->GetOneWayDelay(chain.sat, chain.ue).GetSeconds(),
                                  serviceS,
                                  1e-9,
                                  "No relay is crossed, so its processing must not count");

        // Transparent: feeder + service, and one relay traversal per direction
        delay->SetFeeder(chain.gt);
        const double oneWayS = (chain.FeederM() + chain.ServiceM()) / SpeedOfLightMps;
        NS_TEST_ASSERT_MSG_EQ_TOL(
            delay->GetOneWayDelay(chain.sat, chain.ue).GetSeconds(),
            oneWayS + 250e-6,
            1e-9,
            "Transparent delay must be (feeder + service) / c plus the relay");
        chain.gt->SetPosition(Vector(0, 0, -300e3));
        NS_TEST_ASSERT_MSG_EQ_TOL(delay->GetOneWayDelay(chain.sat, chain.ue).GetSeconds(),
                                  oneWayS + 300e3 / SpeedOfLightMps + 250e-6,
                                  1e-9,
                                  "Transparent delay must follow the live feeder geometry");
    }
};

/**
 * @brief Amplify-and-forward pays for the noise the relay forwards; the uplink pays the feeder
 * loss minus the relay gain.
 */
class NrNtnFeederNoiseTestCase : public TestCase
{
  public:
    NrNtnFeederNoiseTestCase()
        : TestCase("Transparent feeder link: forwarded relay noise and uplink feeder loss")
    {
    }

  private:
    void DoRun() override
    {
        Chain chain;
        Ptr<NrNtnFeederNoiseLossModel> feeder = CreateObject<NrNtnFeederNoiseLossModel>();
        feeder->SetFeeder(chain.gt, Friis());
        feeder->SetAttribute("GatewayTxPowerDbm", DoubleValue(60.0));
        feeder->SetAttribute("SatRxAntennaGainDb", DoubleValue(30.0));
        feeder->SetAttribute("RelayLossDb", DoubleValue(3.0));
        feeder->SetAttribute("RelayInputNoisePowerDbm", DoubleValue(-100.0));
        feeder->SetAttribute("ReceiverNoisePowerDbm", DoubleValue(-100.0));

        NS_TEST_ASSERT_MSG_EQ_TOL(feeder->GetFeederLossDb(chain.sat),
                                  Fspl(chain.FeederM()),
                                  1e-6,
                                  "The feeder loss is the gateway -> satellite link");

        // Closed form of the cascade: 1/SNR = 1/SNR_feeder + 1/SNR_service, the relay adding
        // one unit of its own noise on top of each (the "+1" of an automatic gain control)
        const double eirpDbm = 78.0;
        const double serviceDb = Fspl(chain.ServiceM());
        const double feederSnr =
            std::pow(10.0, (60.0 + 30.0 - Fspl(chain.FeederM()) - 3.0 + 100.0) / 10.0);
        const double serviceSnr = std::pow(10.0, (eirpDbm - serviceDb + 100.0) / 10.0);
        const double cascadeDb =
            10.0 * std::log10(feederSnr * serviceSnr / (feederSnr + serviceSnr + 1.0));
        const double snrDb = feeder->GetDownlinkSnrDb(eirpDbm, Fspl(chain.FeederM()), serviceDb);
        NS_TEST_ASSERT_MSG_EQ_TOL(snrDb, cascadeDb, 1e-6, "Amplify-and-forward cascade SNR");

        // A quiet relay front end is the ideal case: the service hop at the relay EIRP
        feeder->SetAttribute("RelayInputNoisePowerDbm", DoubleValue(-130.0));
        NS_TEST_ASSERT_MSG_GT(feeder->GetDownlinkSnrDb(eirpDbm, Fspl(chain.FeederM()), serviceDb),
                              snrDb,
                              "A noisier relay front end must lower the downlink SNR");
        NS_TEST_ASSERT_MSG_EQ_TOL(
            feeder->GetDownlinkSnrDb(eirpDbm, Fspl(chain.FeederM()), serviceDb),
            eirpDbm - serviceDb + 100.0,
            0.05,
            "A noiseless relay must reduce to the service hop at the relay "
            "EIRP");

        // Links that are not gNB <-> UE, here two bare mobility models, keep the service loss
        Ptr<PropagationLossModel> service = Friis();
        feeder->SetServiceLoss(service);
        NS_TEST_ASSERT_MSG_EQ_TOL(feeder->CalcRxPower(0.0, chain.sat, chain.ue),
                                  -serviceDb,
                                  1e-6,
                                  "Without a gNB at either end only the service loss applies");
    }
};

/**
 * @brief The delay rate is the derivative of the one-way delay, feeder included, and a signal
 * is compressed by it rounded toward shorter.
 */
class NrNtnDelayRateTestCase : public TestCase
{
  public:
    NrNtnDelayRateTestCase()
        : TestCase("NTN delay rate follows the one-way delay; the squeeze rounds toward shorter")
    {
    }

  private:
    void DoRun() override
    {
        // The satellite descends and drifts sideways over a static gateway and UE
        Chain chain;
        for (bool transparent : {false, true})
        {
            Ptr<ConstantVelocityMobilityModel> sat = CreateObject<ConstantVelocityMobilityModel>();
            sat->SetPosition(chain.sat->GetPosition());
            sat->SetVelocity(Vector(7000.0, 0.0, -1000.0));
            Ptr<NrNtnPropagationDelayModel> delay = CreateObject<NrNtnPropagationDelayModel>();
            if (transparent)
            {
                delay->SetFeeder(chain.gt);
            }
            const Time d0 = delay->GetOneWayDelay(sat, chain.ue);
            const double rate = delay->GetOneWayDelayRate(sat, chain.ue);
            const Time step = MilliSeconds(100);
            Time d1;
            Simulator::Schedule(step, [&]() { d1 = delay->GetOneWayDelay(sat, chain.ue); });
            Simulator::Run();
            Simulator::Destroy();
            // Tolerance: the delays are whole ns (1 ns over the step) and the path curves, so the
            // rate drifts by ~1e-8 over 100 ms here
            const double finiteDiff = (d1 - d0).GetSeconds() / step.GetSeconds();
            NS_TEST_ASSERT_MSG_LT(rate, 0.0, "An approaching satellite must shorten the delay");
            NS_TEST_ASSERT_MSG_EQ_TOL(rate,
                                      finiteDiff,
                                      3e-8,
                                      "Delay rate is not the derivative of the one-way delay"
                                          << (transparent ? " with the feeder" : ""));
        }

        // 71428 ns sent while the delay shrinks by 1.79 ns: floor(-1.79) - 1 = -3 ns, so the
        // arrival ends no later than it physically does and rounding cannot close the PHY's gap
        const Time symbol = NanoSeconds(71428);
        NS_TEST_ASSERT_MSG_EQ(NrNtnPropagationDelayModel::SqueezeDuration(symbol, -2.5e-5),
                              NanoSeconds(71425),
                              "Closing path: rounded toward shorter, one ns more");
        NS_TEST_ASSERT_MSG_EQ(NrNtnPropagationDelayModel::SqueezeDuration(symbol, 2.5e-5),
                              NanoSeconds(71429),
                              "Opening path: stretched, rounded toward shorter");
        NS_TEST_ASSERT_MSG_EQ(NrNtnPropagationDelayModel::SqueezeDuration(symbol, 0.0),
                              symbol,
                              "A static path must not change the duration");
    }
};

/**
 * @brief NTN link model validation suite.
 */
class NrNtnLinkModelsTestSuite : public TestSuite
{
  public:
    NrNtnLinkModelsTestSuite()
        : TestSuite("nr-ntn-link-models", Type::UNIT)
    {
        AddTestCase(new NrNtnLoopDelayTestCase, Duration::QUICK);
        AddTestCase(new NrNtnFeederNoiseTestCase, Duration::QUICK);
        AddTestCase(new NrNtnDelayRateTestCase, Duration::QUICK);
    }
};

/// Static variable for test initialization
static NrNtnLinkModelsTestSuite g_nrNtnLinkModelsTestSuite;

} // namespace ns3
