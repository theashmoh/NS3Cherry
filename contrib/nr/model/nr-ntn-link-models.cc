// Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-ntn-link-models.h"

#include "nr-gnb-net-device.h"
#include "nr-gnb-phy.h"
#include "nr-spectrum-phy.h"
#include "nr-ue-net-device.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrNtnLinkModels");

NS_OBJECT_ENSURE_REGISTERED(NrNtnPropagationDelayModel);
NS_OBJECT_ENSURE_REGISTERED(NrNtnFeederNoiseLossModel);
NS_OBJECT_ENSURE_REGISTERED(NrNtnUeToUeFilter);

namespace
{

/// The first device of type T on the node that carries @p mobility, if any
template <class T>
Ptr<T>
FindDevice(Ptr<MobilityModel> mobility)
{
    Ptr<Node> node = mobility->GetObject<Node>();
    for (uint32_t i = 0; node && i < node->GetNDevices(); ++i)
    {
        if (Ptr<T> device = DynamicCast<T>(node->GetDevice(i)))
        {
            return device;
        }
    }
    return nullptr;
}

/// Rate of change of the distance between @p a and @p b, in m/s: negative while they close
double
RangeRate(Ptr<MobilityModel> a, Ptr<MobilityModel> b)
{
    const Vector d = b->GetPosition() - a->GetPosition();
    const double range = d.GetLength();
    if (range == 0.0)
    {
        return 0.0;
    }
    const Vector v = b->GetVelocity() - a->GetVelocity();
    return (d.x * v.x + d.y * v.y + d.z * v.z) / range;
}

/// Power sum of two levels in dBm
double
SumDbm(double aDbm, double bDbm)
{
    return 10.0 * std::log10(std::pow(10.0, aDbm / 10.0) + std::pow(10.0, bDbm / 10.0));
}

} // namespace

TypeId
NrNtnPropagationDelayModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrNtnPropagationDelayModel")
            .SetParent<ConstantSpeedPropagationDelayModel>()
            .SetGroupName("Nr")
            .AddConstructor<NrNtnPropagationDelayModel>()
            .AddAttribute("RelayProcessingDelay",
                          "Processing delay of the transparent relay satellite (SetFeeder() "
                          "only).",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&NrNtnPropagationDelayModel::m_relayProcessingDelay),
                          MakeTimeChecker());
    return tid;
}

void
NrNtnPropagationDelayModel::SetFeeder(Ptr<MobilityModel> gateway)
{
    NS_ABORT_MSG_UNLESS(gateway, "The feeder link needs a gateway");
    m_gateway = gateway;
}

Time
NrNtnPropagationDelayModel::GetOneWayDelay(Ptr<MobilityModel> gnb, Ptr<MobilityModel> ue) const
{
    double meters = 0.0;
    Time processing;
    if (m_gateway)
    {
        meters = m_gateway->GetDistanceFrom(gnb);
        processing = m_relayProcessingDelay;
    }
    meters += gnb->GetDistanceFrom(ue);
    return Seconds(meters / GetSpeed()) + processing;
}

double
NrNtnPropagationDelayModel::GetOneWayDelayRate(Ptr<MobilityModel> gnb, Ptr<MobilityModel> ue) const
{
    // The derivative of GetOneWayDelay(): the relay processing delay is constant
    double rate = RangeRate(gnb, ue);
    if (m_gateway)
    {
        rate += RangeRate(m_gateway, gnb);
    }
    return rate / GetSpeed();
}

Time
NrNtnPropagationDelayModel::SqueezeDuration(Time duration, double rate)
{
    const double deltaNs = std::floor(rate * duration.GetNanoSeconds()) - (rate < 0 ? 1 : 0);
    return std::max(duration + NanoSeconds(static_cast<int64_t>(deltaNs)), NanoSeconds(1));
}

Time
NrNtnPropagationDelayModel::GetDelay(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
    Ptr<NrGnbNetDevice> txGnb = FindDevice<NrGnbNetDevice>(a);
    Ptr<NrGnbNetDevice> rxGnb = FindDevice<NrGnbNetDevice>(b);
    if (!txGnb == !rxGnb)
    {
        return ConstantSpeedPropagationDelayModel::GetDelay(a, b); // not a gNB <-> UE link
    }
    if (txGnb)
    {
        return GetOneWayDelay(a, b);
    }
    const Time delay = GetOneWayDelay(b, a);
    if (rxGnb->GetCcMapSize() == 0)
    {
        return delay; // a gNB without a PHY has no slot grid
    }
    // ponytail: BWP 0's grid, since every BWP of an NTN gNB shares its numerology; with mixed
    // numerologies this would have to find the PHY of the receiving BWP.
    Ptr<const NrGnbPhy> phy = rxGnb->GetPhy(0);
    const Time now = Simulator::Now();
    return AlignUplinkArrival(now + delay,
                              phy->GetSlotPeriod(),
                              phy->GetSymbolPeriod(),
                              phy->GetSymbolsPerSlot()) -
           now;
}

Time
NrNtnPropagationDelayModel::AlignUplinkArrival(Time arrival,
                                               Time slotPeriod,
                                               Time symbolPeriod,
                                               uint32_t symbolsPerSlot)
{
    const int64_t t = arrival.GetTimeStep();
    const int64_t slot = slotPeriod.GetTimeStep();
    const int64_t symbol = symbolPeriod.GetTimeStep();
    const int64_t slotStart = t / slot * slot;
    // Symbol periods are rounded down, so the symbols can end a few ns before the slot does:
    // past the last symbol start, the nearest start is the next slot's first symbol
    const int64_t index = (t - slotStart + symbol / 2) / symbol;
    const int64_t symbolStart = index < static_cast<int64_t>(symbolsPerSlot)
                                    ? slotStart + index * symbol
                                    : slotStart + slot;
    // ponytail: fixed at one normal cyclic prefix (144 of the 2192 samples of a symbol); make it
    // an attribute if a study needs another timing-error budget
    const int64_t cyclicPrefix = symbol * 144 / 2192;
    if (std::abs(t - symbolStart) > cyclicPrefix)
    {
        NS_LOG_WARN("Uplink arrival at " << arrival.As(Time::NS) << " is "
                                         << TimeStep(std::abs(t - symbolStart)).As(Time::NS)
                                         << " off the gNB symbol grid, more than a cyclic "
                                            "prefix: the timing advance does not track the path");
        return arrival;
    }
    return TimeStep(symbolStart);
}

TypeId
NrNtnFeederNoiseLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrNtnFeederNoiseLossModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("Nr")
            .AddConstructor<NrNtnFeederNoiseLossModel>()
            .AddAttribute("GatewayTxPowerDbm",
                          "Gateway transmit power on the feeder link.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrNtnFeederNoiseLossModel::m_gatewayTxPowerDbm),
                          MakeDoubleChecker<double>())
            .AddAttribute("SatRxAntennaGainDb",
                          "Receive antenna gain of the satellite on the feeder link.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrNtnFeederNoiseLossModel::m_satRxAntennaGainDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("RelayLossDb",
                          "Fixed loss between the relay's receive antenna and its amplifier.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrNtnFeederNoiseLossModel::m_relayLossDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("RelayEirpDbm",
                          "EIRP to which the relay amplifies signal plus input noise.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrNtnFeederNoiseLossModel::m_relayEirpDbm),
                          MakeDoubleChecker<double>())
            .AddAttribute("RelayInputNoisePowerDbm",
                          "Noise power at the relay input, amplified and forwarded with the "
                          "signal.",
                          DoubleValue(-100.0),
                          MakeDoubleAccessor(&NrNtnFeederNoiseLossModel::m_relayInputNoisePowerDbm),
                          MakeDoubleChecker<double>())
            .AddAttribute("ReceiverNoisePowerDbm",
                          "UE receiver noise power, against which the forwarded noise is charged.",
                          DoubleValue(-100.0),
                          MakeDoubleAccessor(&NrNtnFeederNoiseLossModel::m_receiverNoisePowerDbm),
                          MakeDoubleChecker<double>());
    return tid;
}

void
NrNtnFeederNoiseLossModel::SetServiceLoss(Ptr<PropagationLossModel> serviceLoss)
{
    m_serviceLoss = serviceLoss;
}

void
NrNtnFeederNoiseLossModel::SetFeeder(Ptr<MobilityModel> gateway,
                                     Ptr<PropagationLossModel> feederLoss)
{
    m_gateway = gateway;
    m_feederLoss = feederLoss;
}

double
NrNtnFeederNoiseLossModel::GetFeederLossDb(Ptr<MobilityModel> satellite) const
{
    NS_ABORT_MSG_UNLESS(m_gateway && m_feederLoss, "Call SetFeeder() first");
    return -m_feederLoss->CalcRxPower(0.0, m_gateway, satellite);
}

double
NrNtnFeederNoiseLossModel::GetDownlinkSnrDb(double relayEirpDbm,
                                            double feederLossDb,
                                            double serviceLossDb) const
{
    // The relay's automatic gain control brings signal plus input noise to its EIRP
    const double relayInputSignalDbm =
        m_gatewayTxPowerDbm + m_satRxAntennaGainDb - feederLossDb - m_relayLossDb;
    const double relayGainDb =
        relayEirpDbm - SumDbm(relayInputSignalDbm, m_relayInputNoisePowerDbm);
    const double ueSignalDbm = relayInputSignalDbm + relayGainDb - serviceLossDb;
    const double forwardedNoiseDbm = m_relayInputNoisePowerDbm + relayGainDb - serviceLossDb;
    return ueSignalDbm - SumDbm(forwardedNoiseDbm, m_receiverNoisePowerDbm);
}

double
NrNtnFeederNoiseLossModel::DoCalcRxPower(double txPowerDbm,
                                         Ptr<MobilityModel> a,
                                         Ptr<MobilityModel> b) const
{
    NS_ABORT_MSG_UNLESS(m_serviceLoss, "Call SetServiceLoss() first");
    const double serviceRxDbm = m_serviceLoss->CalcRxPower(txPowerDbm, a, b);
    Ptr<NrGnbNetDevice> txGnb = FindDevice<NrGnbNetDevice>(a);
    Ptr<NrGnbNetDevice> rxGnb = FindDevice<NrGnbNetDevice>(b);
    if (!txGnb == !rxGnb)
    {
        return serviceRxDbm; // not a gNB <-> UE link
    }
    if (rxGnb)
    {
        // Uplink: the feeder adds only delay. TR 38.821 treats the feeder's other impairments as
        // negligible (Table 6.1.1.1-5, note 1), so the satellite's receiver sets the SNR, as in a
        // regenerative payload.
        return serviceRxDbm;
    }
    // The gNB's power stands for the relay EIRP; lower the signal until the UE's own noise gives
    // the cascade SNR
    const double serviceLossDb = txPowerDbm - serviceRxDbm;
    return txPowerDbm + m_receiverNoisePowerDbm +
           GetDownlinkSnrDb(m_relayEirpDbm, GetFeederLossDb(a), serviceLossDb) - m_relayEirpDbm;
}

int64_t
NrNtnFeederNoiseLossModel::DoAssignStreams(int64_t stream)
{
    // The service model's streams, as when it was the channel's own model; the feeder model
    // keeps whatever it was given
    return m_serviceLoss ? m_serviceLoss->AssignStreams(stream) : 0;
}

TypeId
NrNtnUeToUeFilter::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrNtnUeToUeFilter")
                            .SetParent<SpectrumTransmitFilter>()
                            .SetGroupName("Nr")
                            .AddConstructor<NrNtnUeToUeFilter>();
    return tid;
}

bool
NrNtnUeToUeFilter::DoFilter(Ptr<const SpectrumSignalParameters> params,
                            Ptr<const SpectrumPhy> receiverPhy)
{
    auto tx = DynamicCast<const NrSpectrumPhy>(params->txPhy);
    auto rx = DynamicCast<const NrSpectrumPhy>(receiverPhy);
    return tx && rx && !tx->IsGnb() && !rx->IsGnb();
}

int64_t
NrNtnUeToUeFilter::DoAssignStreams(int64_t /* stream */)
{
    return 0;
}

} // namespace ns3
