// Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_NTN_LINK_MODELS_H
#define NR_NTN_LINK_MODELS_H

#include "ns3/nr-export.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/spectrum-transmit-filter.h"

namespace ns3
{

/**
 * @ingroup nr
 * @brief Speed-of-light delay for NTN links, with every UE -> gNB arrival lined up on the gNB's
 * symbol grid when it lands within one cyclic prefix of it.
 *
 * The gNB sits on the satellite that serves the UEs. That is the whole radio path of a
 * regenerative payload. A transparent payload also carries the waveform over the feeder link,
 * gateway -> serving satellite, and the relay adds its processing delay: SetFeeder() turns that
 * on. ponytail: one relay per path; a feeder through another satellite and an ISL is future
 * work.
 *
 * NrSpectrumPhy at the gNB needs overlapping uplink receptions to start at the same nanosecond
 * and back-to-back ones not to touch. Over a moving satellite the UEs miss that by nanoseconds:
 * the timing advance rounds and the path keeps changing within a slot. A real gNB takes every
 * UE through one FFT window per symbol and its cyclic prefix absorbs such offsets, so this
 * model delivers them on that window. An arrival further off than a cyclic prefix is a real
 * timing error and keeps its physical delay.
 */
class NR_EXPORT NrNtnPropagationDelayModel : public ConstantSpeedPropagationDelayModel
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief Make the payload transparent: every gNB <-> UE signal also crosses the feeder link
     * from the gateway to the satellite the gNB sits on.
     * @param gateway the ground gateway
     */
    void SetFeeder(Ptr<MobilityModel> gateway);

    /**
     * @brief One-way radio delay between the gNB and a UE, without the uplink line-up: the
     * delay the timing advance compensates and half the loop RTT that sizes K_offset.
     * @param gnb the gNB, on the serving satellite
     * @param ue the UE
     * @return the one-way delay
     */
    Time GetOneWayDelay(Ptr<MobilityModel> gnb, Ptr<MobilityModel> ue) const;

    Time GetDelay(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override;

    /**
     * @brief How fast GetOneWayDelay() changes, in seconds per second: negative while the path
     * shortens, e.g. under an approaching satellite, and including the feeder link of a
     * transparent payload. The UE's PHY uses it to compress a downlink signal by the delay drift
     * over its own duration (the Doppler squeeze).
     * @param gnb the gNB, on the serving satellite
     * @param ue the UE
     * @return the delay rate
     */
    virtual double GetOneWayDelayRate(Ptr<MobilityModel> gnb, Ptr<MobilityModel> ue) const;

    /**
     * @brief Arriving duration of a signal sent over @p duration while the delay changed at
     * @p rate: duration * (1 + rate). Rounded toward shorter, and one nanosecond shorter still
     * while the path closes, so that rounding this end and the next signal's start can never
     * close the 1-2 ns gap the NR PHYs leave between back-to-back signals.
     * @param duration the transmitted duration
     * @param rate the delay rate, from GetOneWayDelayRate()
     * @return the arriving duration, at least one nanosecond
     */
    static Time SqueezeDuration(Time duration, double rate);

    /**
     * @brief Move an uplink arrival onto the nearest symbol start of the gNB's slot grid, which
     * starts at t = 0, when it lies within one normal cyclic prefix of it.
     * @param arrival the physical arrival time
     * @param slotPeriod the gNB slot period
     * @param symbolPeriod the gNB symbol period
     * @param symbolsPerSlot the number of symbols in a slot
     * @return the lined-up arrival, or @p arrival when it is further off
     */
    static Time AlignUplinkArrival(Time arrival,
                                   Time slotPeriod,
                                   Time symbolPeriod,
                                   uint32_t symbolsPerSlot);

  private:
    Ptr<MobilityModel> m_gateway; //!< Feeder-link gateway (null: regenerative payload)
    Time m_relayProcessingDelay;  //!< Attribute RelayProcessingDelay
};

/**
 * @ingroup nr
 * @brief The link budget of a transparent payload: the service link seen from the serving
 * satellite, where the gNB sits, plus what the feeder link costs.
 *
 * It wraps the service-link loss model and must replace it as the channel's
 * PropagationLossModel. The feeder link runs from the gateway to the satellite the gNB of the
 * signal sits on. Downlink: the relay amplifies whatever reaches it from the gateway, its own
 * input noise included, to its EIRP (RelayEirpDbm), so the UE sees
 * 1/SNR = 1/SNR_feeder + 1/SNR_service; the received power is lowered until the UE's own noise
 * gives that SNR. Uplink: the service loss alone, the feeder adding only delay (TR 38.821 Table
 * 6.1.1.1-5, note 1). Signals between two UEs or two gNBs keep the service loss.
 */
class NR_EXPORT NrNtnFeederNoiseLossModel : public PropagationLossModel
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief Set the service-link loss model this one wraps.
     * @param serviceLoss the loss between the serving satellite and a UE
     */
    void SetServiceLoss(Ptr<PropagationLossModel> serviceLoss);

    /**
     * @brief Set the feeder link.
     * @param gateway the ground gateway
     * @param feederLoss the loss model of the feeder link
     */
    void SetFeeder(Ptr<MobilityModel> gateway, Ptr<PropagationLossModel> feederLoss);

    /**
     * @brief Current feeder-link loss from the gateway to a satellite.
     * @param satellite the satellite
     * @return the loss in dB
     */
    double GetFeederLossDb(Ptr<MobilityModel> satellite) const;

    /**
     * @brief End-to-end downlink SNR at the UE through the amplify-and-forward relay.
     * @param relayEirpDbm the relay's output EIRP
     * @param feederLossDb the feeder-link loss, gateway -> satellite
     * @param serviceLossDb the service-link loss, satellite -> UE
     * @return the SNR in dB
     */
    double GetDownlinkSnrDb(double relayEirpDbm, double feederLossDb, double serviceLossDb) const;

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    Ptr<PropagationLossModel> m_serviceLoss; //!< Service-link loss
    Ptr<PropagationLossModel> m_feederLoss;  //!< Feeder-link loss
    Ptr<MobilityModel> m_gateway;            //!< Feeder-link gateway
    double m_gatewayTxPowerDbm;              //!< Attribute GatewayTxPowerDbm
    double m_satRxAntennaGainDb;             //!< Attribute SatRxAntennaGainDb
    double m_relayLossDb;                    //!< Attribute RelayLossDb
    double m_relayEirpDbm;                   //!< Attribute RelayEirpDbm
    double m_relayInputNoisePowerDbm;        //!< Attribute RelayInputNoisePowerDbm
    double m_receiverNoisePowerDbm;          //!< Attribute ReceiverNoisePowerDbm
};

/**
 * @ingroup nr
 * @brief Drop UE -> UE signals. In FDD no UE listens on the uplink carrier, yet the channel
 * computes every UE pair there, and the 3GPP NTN models cannot take a ground-to-ground link.
 * Install it on FDD carriers only: in TDD, UE -> UE interference is real.
 */
class NR_EXPORT NrNtnUeToUeFilter : public SpectrumTransmitFilter
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    bool DoFilter(Ptr<const SpectrumSignalParameters> params,
                  Ptr<const SpectrumPhy> receiverPhy) override;

  protected:
    int64_t DoAssignStreams(int64_t stream) override;
};

} // namespace ns3

#endif /* NR_NTN_LINK_MODELS_H */
