// Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/constant-position-mobility-model.h"
#include "ns3/matrix-array.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/net-device.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-phy.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"
#include "ns3/test.h"

using namespace ns3;

/**
 * @ingroup spectrum-tests
 *
 * Minimal SpectrumPhy that records the last received signal and can re-tune to another
 * receive spectrum model.
 */
class RetuneTestPhy : public SpectrumPhy
{
  public:
    /**
     * Constructor.
     *
     * @param model The initial receive spectrum model.
     */
    RetuneTestPhy(Ptr<const SpectrumModel> model)
        : m_model(model)
    {
    }

    /**
     * Switch to another receive spectrum model and notify the channel, as a PHY that
     * re-tunes must.
     *
     * @param model The new receive spectrum model.
     */
    void Retune(Ptr<const SpectrumModel> model)
    {
        m_model = model;
        m_channel->AddRx(this);
    }

    void SetDevice(Ptr<NetDevice> d) override
    {
        m_device = d;
    }

    Ptr<NetDevice> GetDevice() const override
    {
        return m_device;
    }

    void SetMobility(Ptr<MobilityModel> m) override
    {
        m_mobility = m;
    }

    Ptr<MobilityModel> GetMobility() const override
    {
        return m_mobility;
    }

    void SetChannel(Ptr<SpectrumChannel> c) override
    {
        m_channel = c;
    }

    Ptr<const SpectrumModel> GetRxSpectrumModel() const override
    {
        return m_model;
    }

    Ptr<Object> GetAntenna() const override
    {
        return nullptr;
    }

    void StartRx(Ptr<SpectrumSignalParameters> params) override
    {
        m_rxParams = params;
    }

    Ptr<SpectrumSignalParameters> m_rxParams; ///< Last received signal

  private:
    Ptr<const SpectrumModel> m_model; ///< Receive spectrum model
    Ptr<NetDevice> m_device;          ///< Device
    Ptr<MobilityModel> m_mobility;    ///< Mobility model
    Ptr<SpectrumChannel> m_channel;   ///< Channel
};

/**
 * @ingroup spectrum-tests
 *
 * A receiver that switches spectrum model while a signal is in flight, as a PHY that
 * re-tunes during a long propagation delay does. The channel converts the PSD to the new
 * model on arrival; this checks what comes with it.
 *
 * - Precoder: a precoding matrix has one page per band of the model it was built for. After
 *   the conversion it no longer fits and must be dropped, as StartTx already does for a
 *   receiver whose model differs at transmission. A receiver that did not re-tune keeps it.
 * - Shared PSD: when the new model already has a converted PSD for this signal, that PSD is
 *   shared by every receiver (it is the transmitter's own PSD when the models match), so the
 *   receiver must work on a copy. Otherwise its path loss is applied to the shared one, and
 *   a second receiver on the same model gets both path losses.
 */
class MultiModelSpectrumChannelRetuneTestCase : public TestCase
{
  public:
    MultiModelSpectrumChannelRetuneTestCase();

  private:
    void DoRun() override;
};

MultiModelSpectrumChannelRetuneTestCase::MultiModelSpectrumChannelRetuneTestCase()
    : TestCase("Receiver switching spectrum model while a signal is in flight")
{
}

void
MultiModelSpectrumChannelRetuneTestCase::DoRun()
{
    // 10 MHz bands around 2 GHz: four for the transmitter, the two middle ones for the
    // narrow receivers, so the two models overlap and a converter exists between them.
    auto makeModel = [](double firstLowHz, int numBands) {
        Bands bands;
        for (int i = 0; i < numBands; ++i)
        {
            BandInfo b;
            b.fl = firstLowHz + i * 10e6;
            b.fc = b.fl + 5e6;
            b.fh = b.fl + 10e6;
            bands.push_back(b);
        }
        return Create<SpectrumModel>(bands);
    };
    Ptr<SpectrumModel> wide = makeModel(1.98e9, 4);
    Ptr<SpectrumModel> narrow = makeModel(1.99e9, 2);

    auto channel = CreateObject<MultiModelSpectrumChannel>();
    channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    auto loss = CreateObject<FixedRssLossModel>();
    loss->SetRss(-10); // dBm for 0 dBm sent: 10 dB, so every receiver gets 0.1 of the PSD
    channel->AddPropagationLossModel(loss);

    auto addPhy = [&](Ptr<const SpectrumModel> model, double xM) {
        auto phy = CreateObject<RetuneTestPhy>(model);
        auto mobility = CreateObject<ConstantPositionMobilityModel>();
        mobility->SetPosition(Vector(xM, 0, 0));
        phy->SetMobility(mobility);
        phy->SetChannel(channel);
        return phy;
    };
    auto txPhy = addPhy(wide, 0);
    // 3 km: the signal is in flight for 10 us
    auto stayPhy = addPhy(wide, 3000);    // never re-tunes
    auto narrowPhy = addPhy(wide, 3000);  // wide -> narrow in flight
    auto widePhyA = addPhy(narrow, 3000); // narrow -> wide in flight
    auto widePhyB = addPhy(narrow, 3000); // narrow -> wide in flight
    for (const auto& rx : {stayPhy, narrowPhy, widePhyA, widePhyB})
    {
        channel->AddRx(rx);
    }

    auto txPsd = Create<SpectrumValue>(wide);
    *txPsd = 1.0;
    auto params = Create<SpectrumSignalParameters>();
    params->psd = txPsd;
    params->duration = MicroSeconds(1);
    params->txPhy = txPhy;
    params->precodingMatrix = Create<const ComplexMatrixArray>(1, 1, wide->GetNumBands());

    const Time txTime = MilliSeconds(1);
    Simulator::Schedule(txTime, [&]() { channel->StartTx(params); });
    Simulator::Schedule(txTime + MicroSeconds(5), [&]() {
        narrowPhy->Retune(narrow);
        widePhyA->Retune(wide);
        widePhyB->Retune(wide);
    });
    Simulator::Run();

    for (const auto& rx : {stayPhy, narrowPhy, widePhyA, widePhyB})
    {
        NS_TEST_ASSERT_MSG_NE(rx->m_rxParams, nullptr, "Expected one reception");
    }

    // Precoder
    NS_TEST_ASSERT_MSG_EQ(narrowPhy->m_rxParams->psd->GetValuesN(),
                          narrow->GetNumBands(),
                          "PSD not converted to the receiver's new model");
    NS_TEST_ASSERT_MSG_EQ(narrowPhy->m_rxParams->precodingMatrix,
                          nullptr,
                          "Precoder built for the old model was not dropped");
    NS_TEST_ASSERT_MSG_NE(stayPhy->m_rxParams->precodingMatrix,
                          nullptr,
                          "Precoder dropped for a receiver whose model did not change");

    // Shared PSD
    for (const auto& rx : {stayPhy, widePhyA, widePhyB})
    {
        for (size_t i = 0; i < wide->GetNumBands(); ++i)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL((*rx->m_rxParams->psd)[i],
                                      0.1,
                                      1e-12,
                                      "Received PSD is not the transmitted PSD less one path loss");
        }
    }
    NS_TEST_ASSERT_MSG_EQ_TOL((*txPsd)[0], 1.0, 1e-12, "The channel modified the transmitted PSD");

    Simulator::Destroy();
}

/**
 * @ingroup spectrum-tests
 *
 * MultiModelSpectrumChannel test suite.
 */
class MultiModelSpectrumChannelTestSuite : public TestSuite
{
  public:
    MultiModelSpectrumChannelTestSuite();
};

MultiModelSpectrumChannelTestSuite::MultiModelSpectrumChannelTestSuite()
    : TestSuite("multi-model-spectrum-channel", Type::UNIT)
{
    AddTestCase(new MultiModelSpectrumChannelRetuneTestCase(), TestCase::Duration::QUICK);
}

/// Static variable for test initialization
static MultiModelSpectrumChannelTestSuite g_multiModelSpectrumChannelTestSuite;
