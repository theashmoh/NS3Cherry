// Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file leo-link.cc
 * @brief LEO NTN link, transparent or regenerative payload, over any constellation.
 *
 * Every satellite of the orbit carries a gNB and each UE attaches to the closest one. The
 * satellites fly the ns-3.48 `LeoCircularOrbitMobilityModel` (continuous in time); the radio goes
 * through the stock NrHelper and the 3GPP NTN channel of `--scenario`, fading included, seen
 * from the satellite. `--payload` picks the architecture:
 *   - `transparent` (default): the satellite is an amplify-and-forward relay that terminates
 *     neither NR nor IP, so the waveform also crosses the feeder link from the gateway
 *     (`--gatewayLatDeg`/`--gatewayLonDeg`, UE 0's position by default) to the serving satellite.
 *     NrNtnPropagationDelayModel adds its delay and the relay processing, and
 *     NrNtnFeederNoiseLossModel the noise the relay forwards on the downlink; on the uplink the
 *     feeder adds only delay (TR 38.821 Table 6.1.1.1-5, note 1). ponytail: a feeder through
 *     another satellite and an ISL is future work.
 *   - `regenerative`: the gNB on the satellite terminates NR, and the radio link is the UE <->
 *     satellite hop alone.
 * This is the ns-3-NTN teaching example: OLLA, HARQ depth and soft buffer, RLC UM/AM, scheduler.
 *
 * Placement. UE 0 sits at `--observerLatDeg`/`--observerLonDeg`; `ntn/tools/aim_pass.py` solves
 * `--satLongitudeDeg`/`--satArgOfLatitudeDeg` for a pass over it. Without them, `--observerMode`
 * derives UE 0 from the first satellite: under it (`overhead`) or where it rises through
 * `--observerElevationBoundaryDeg` (`full-pass`). The run stops when the first satellite descends
 * through that boundary. More UEs are placed around UE 0 by radius (`--ueRadiiKm`,
 * `--footprintRadiusKm` or `--ueSpacingDeg`) and bearing (`--ueAzimuthsDeg`).
 *
 * Link budget. `--ntnPreset` sets the service link from TR 38.821: the satellite EIRP density
 * (`--satEIRP`, dBW/MHz, delivered as gNB power minus `--satAntennaGainDb` plus that element
 * gain), the terminal gain and noise figure, the carrier and the default orbit altitude.
 *
 * Duplexing. The cell is FDD, as every 3GPP NR NTN band is (Rel-17 n255 L-band, UL
 * 1626.5-1660.5 / DL 1525-1559 MHz; n256 S-band, UL 1980-2010 / DL 2170-2200 MHz; Rel-18 n254,
 * UL L-band 1610-1626.5 / DL S-band 2483.5-2500 MHz; the Ka-band VSAT bands n510-n512; TDD
 * exists only for NB-IoT NTN). Two bandwidth parts are created, BWP 0 downlink-only at
 * `--dlFrequencyHz` with the slot pattern `DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|` and BWP 1
 * uplink-only at `--ulFrequencyHz` with `UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|`. Each BWP has its own
 * channel at its own carrier frequency, with the same link-budget configuration. The static BWP
 * manager pins the default QoS flow to BWP 0 and the gNB advertises the DL/UL carrier pairing in
 * dedicated RRC configuration, so the UE routes its uplink to BWP 1 (the recipe of
 * `nr-fdd-ul-routing`). Ideal RRC only: the real RRC protocol does not carry the pairing.
 *
 * NTN timing (`--ntnTiming`, default on) sets three things from the geometry, never from a
 * numeric flag a user could set inconsistently with the orbit:
 *   - `NrGnbPhy::CellSpecificKoffset` = ceil(maxLoopRtt / 1 ms) reference slots, where
 *     maxLoopRtt is twice `NrNtnPropagationDelayModel::GetOneWayDelay()` from the first
 *     satellite at the worst geometry the simulated window actually visits: the fixed geometry
 *     under `--freezeGeometry`, otherwise the pass swept ahead of time up to `--duration` or the
 *     elevation boundary, whichever comes first. K_offset is cell-specific and static
 *     (TS 38.213 cl. 4.2), so it is sized once for the worst case of the window.
 *   - `NrUePhy::TimingAdvance` per UE = its one-way delay to the gNB it is attached to, feeder
 *     link included when transparent (ideal timing advance: the UE knows its delay exactly,
 *     standing in for GNSS + ephemeris in Rel-17). The UE re-reads it before every slot.
 *   - `NrGnbRrc::ConnectionRequestTimeoutDuration` = max(15 ms, maxLoopRtt + 20 ms). The gNB
 *     arms this timer when it creates the UE context on preamble reception, before the RAR is
 *     even sent, and it runs under ideal RRC too; at the default 15 ms the context dies before a
 *     one-way RAR lands.
 * With `--ntnTiming=0` none of the three attributes is touched (they stay at their defaults).
 * `--idealRa` (default on) sets `NrUeMac::IdealRandomAccess` so the RA response timeout is never
 * armed, and then also pins the connection-request timer to its 2000 ms ceiling. Msg1 and Msg2
 * still cross the channel with real delay, so attach costs one RTT + L0 and nothing else. Every
 * derived value is printed at startup and written to `NtnTimingConfig.txt` in the working
 * directory.
 *
 * Traces. `--traceFile` writes the colon-separated satellite geometry the RT and analysis
 * scripts parse; `--linkBudgetTraceFile` the tab-separated feeder/service link budget of UE 0
 * seen through the first satellite; `--channelConditionTraceFile` its LOS state.
 *
 * `--n1Delay` / `--n2Delay` default to 2 slots; v5.1 accepts 0-4.
 */

#include "ns3/application.h"
#include "ns3/bulk-send-helper.h"
#include "ns3/channel-condition-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/geocentric-ecef-mobility-model.h"
#include "ns3/geographic-positions.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/leo-circular-orbit-mobility-model.h"
#include "ns3/leo-orbit-node-helper.h"
#include "ns3/leo-orbital-shell.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-rx-trace.h"
#include "ns3/nr-module.h"
#include "ns3/nr-ntn-link-models.h"
#include "ns3/nr-phy-rx-trace.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/phased-array-model.h"
#include "ns3/three-gpp-antenna-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/uniform-planar-array.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LeoLinkExample");

namespace
{
constexpr double SpeedOfLightMps = 299792458.0;

double
LocalDegreesToRadians(double degrees)
{
    return degrees * M_PI / 180.0;
}

double
DbToLinear(double valueDb)
{
    return std::pow(10.0, valueDb / 10.0);
}

double
LinearToDb(double valueLinear)
{
    return 10.0 * std::log10(std::max(valueLinear, std::numeric_limits<double>::min()));
}

double
DbmToMilliwatts(double dbm)
{
    return std::pow(10.0, dbm / 10.0);
}

double
MilliwattsToDbm(double milliwatts)
{
    return 10.0 * std::log10(std::max(milliwatts, std::numeric_limits<double>::min()));
}

double
SumDbm(std::initializer_list<double> powersDbm)
{
    double sumMilliwatts = 0.0;
    for (double powerDbm : powersDbm)
    {
        sumMilliwatts += DbmToMilliwatts(powerDbm);
    }
    return MilliwattsToDbm(sumMilliwatts);
}

double
AmplifyForwardEndToEndSnrDb(double gamma1Db, double gamma2Db)
{
    const double gamma1 = DbToLinear(gamma1Db);
    const double gamma2 = DbToLinear(gamma2Db);
    return LinearToDb((gamma1 * gamma2) / (gamma1 + gamma2 + 1.0));
}

/**
 * Ground node whose ECEF position may move at a constant ECEF velocity, and which can report a
 * velocity without moving. ns-3.48's GeocentricEcefMobilityModel is static; this example needs
 * a terminal moving at --ueSpeedMps (the channel's Doppler term reads the velocity) and, under
 * --freezeGeometry, a satellite held at its initial position that still reports its orbital
 * velocity, as the fork's quantised orbit model did.
 */
class EcefKinematicMobilityModel : public GeocentricEcefMobilityModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::LeoLinkEcefKinematicMobilityModel")
                                .SetParent<GeocentricEcefMobilityModel>()
                                .SetGroupName("Mobility")
                                .AddConstructor<EcefKinematicMobilityModel>();
        return tid;
    }

    EcefKinematicMobilityModel() = default;
    ~EcefKinematicMobilityModel() override = default;

    /**
     * @param velocityEcef velocity reported by GetVelocity(), in ECEF m/s
     * @param integratePosition true: the position moves as position0 + velocity * t; false: the
     * position stays where it was set and only the reported velocity changes
     */
    void SetConstantVelocity(const Vector& velocityEcef, bool integratePosition)
    {
        m_velocity = velocityEcef;
        m_integrate = integratePosition;
    }

  private:
    Vector DoGetGeocentricPosition() const override
    {
        const Vector geo = GetGeographicPosition();
        Vector ecef =
            GeographicPositions::GeographicToCartesianCoordinates(geo.x,
                                                                  geo.y,
                                                                  geo.z,
                                                                  GeographicPositions::SPHERE);
        if (m_integrate)
        {
            ecef = ecef + m_velocity * Simulator::Now().GetSeconds();
        }
        return ecef;
    }

    Vector DoGetVelocity() const override
    {
        return m_velocity;
    }

    Vector m_velocity{0.0, 0.0, 0.0};
    bool m_integrate{false};
};

// Predicts where a circular-orbit satellite will be, in ECEF, at simulation time t. It
// reproduces LeoCircularOrbitMobilityModel::CalcPosition of ns-3.48 from the position and
// velocity the model reports at t = 0: a circular orbit in the plane spanned by pos0 and vel0
// at the model's angular velocity, then the Earth's rotation (2 pi per day, as the model does)
// applied as a rotation about the ECEF z axis. The model has no public "position at time t",
// and both the observer placement of full-pass mode and the NTN timing need the geometry of
// the window before the simulation runs. A predictor built with moving = false never moves.
struct SatellitePredictor
{
    Vector pos0;
    Vector vel0;
    double omegaRadPerS{0.0};

    Vector At(Time t) const
    {
        const double radius = pos0.GetLength();
        const double speed = vel0.GetLength();
        if (radius <= 0.0 || speed <= 0.0)
        {
            return pos0;
        }
        Vector normal = CrossProduct(pos0, vel0);
        normal = normal * (1.0 / normal.GetLength());
        const double theta = omegaRadPerS * t.GetSeconds();
        const Vector orbital =
            pos0 * std::cos(theta) + CrossProduct(normal, pos0) * std::sin(theta);
        const double phi = 2.0 * M_PI * t.GetSeconds() / Days(1).GetSeconds();
        return Vector(orbital.x * std::cos(phi) + orbital.y * std::sin(phi),
                      -orbital.x * std::sin(phi) + orbital.y * std::cos(phi),
                      orbital.z);
    }
};

SatellitePredictor
MakeSatellitePredictor(Ptr<MobilityModel> mobility, bool moving)
{
    SatellitePredictor predictor;
    predictor.pos0 = mobility->GetPosition();
    if (!moving)
    {
        return predictor;
    }
    predictor.vel0 = mobility->GetVelocity();
    Ptr<LeoCircularOrbitMobilityModel> leo = DynamicCast<LeoCircularOrbitMobilityModel>(mobility);
    if (leo)
    {
        predictor.omegaRadPerS = leo->GetAngularVelocity();
    }
    else if (predictor.pos0.GetLength() > 0.0)
    {
        predictor.omegaRadPerS = predictor.vel0.GetLength() / predictor.pos0.GetLength();
    }
    return predictor;
}

void
LogPhasedArrayPorts(const std::string& label, Ptr<NrSpectrumPhy> spectrumPhy)
{
    Ptr<PhasedArrayModel> antenna =
        spectrumPhy ? DynamicCast<PhasedArrayModel>(spectrumPhy->GetAntenna()) : nullptr;
    if (!antenna)
    {
        std::cout << label << " antenna: not a phased array" << std::endl;
        return;
    }

    std::cout << label << " antenna: ports=" << antenna->GetNumPorts() << " ("
              << antenna->GetNumVerticalPorts() << "V x " << antenna->GetNumHorizontalPorts()
              << "H), array=" << antenna->GetNumRows() << "x" << antenna->GetNumColumns()
              << " elements=" << antenna->GetNumElems()
              << " elemsPerPort=" << antenna->GetNumElemsPerPort()
              << " dualPol=" << antenna->IsDualPol() << std::endl;
}

// Spot-beam gain pattern (service link satellite -> UE). The beam is steered at the beam centre
// (UE 0);
// a UE at off-boresight angle theta sits on the uniform circular-aperture (Airy) pattern
// G(theta) = 4*|J1(ka*sin theta)/(ka*sin theta)|^2 (1 at boresight, -3 dB at the half-power point,
// proper first null + sidelobes). ka set so half-power is at HPBW/2. theta is from the live ECEF
// geometry, so the taper a fixed ground UE sees changes with elevation (beam distortion + the edge
// UEs sweep through the pattern null/sidelobes).
double
SpotBeamTaperDb(const Vector& satEcef,
                const Vector& beamCentreEcef,
                const Vector& ueEcef,
                double hpbwDeg,
                double maxAttenDb)
{
    const Vector vc = beamCentreEcef - satEcef;
    const Vector vu = ueEcef - satEcef;
    const double nc = vc.GetLength();
    const double nu = vu.GetLength();
    if (nc <= 0.0 || nu <= 0.0 || hpbwDeg <= 0.0)
    {
        return 0.0;
    }
    double cosT = (vc.x * vu.x + vc.y * vu.y + vc.z * vu.z) / (nc * nu);
    cosT = std::max(-1.0, std::min(1.0, cosT));
    const double sinTheta = std::sin(std::acos(cosT));
    if (sinTheta < 1e-12)
    {
        return 0.0;
    }
    const double ka = 1.616339 / std::sin(hpbwDeg / 2.0 * M_PI / 180.0);
    const double x = ka * sinTheta;
    const double f = j1(x) / x;
    const double g = 4.0 * f * f;
    return std::max(10.0 * std::log10(std::max(g, 1e-12)), -maxAttenDb);
}

// Recomputes each UE's effective antenna gain (base + spot-beam taper) from the current satellite
// geometry and pushes it into the UE antenna element, invalidating the cached channel on every
// BWP array of the UE so the new gain takes effect on the next query. Reschedules itself every
// `period`.
void
UpdateSpotBeamTaper(Ptr<MobilityModel> satMobility,
                    Ptr<MobilityModel> beamCentreMobility,
                    std::vector<Ptr<MobilityModel>> ueMobilities,
                    std::vector<Ptr<IsotropicAntennaModel>> ueElements,
                    std::vector<std::vector<Ptr<PhasedArrayModel>>> ueArrays,
                    double baseGainDb,
                    double hpbwDeg,
                    double maxAttenDb,
                    Time period)
{
    const Vector sat = satMobility->GetPosition();
    const Vector centre = beamCentreMobility->GetPosition();
    for (std::size_t i = 0; i < ueElements.size(); ++i)
    {
        const double taperDb =
            SpotBeamTaperDb(sat, centre, ueMobilities[i]->GetPosition(), hpbwDeg, maxAttenDb);
        ueElements[i]->SetAttribute("Gain", DoubleValue(baseGainDb + taperDb));
        for (auto& ueArray : ueArrays[i])
        {
            ueArray->SetBeamformingVector(ueArray->GetBeamformingVector());
        }
    }
    Simulator::Schedule(period,
                        &UpdateSpotBeamTaper,
                        satMobility,
                        beamCentreMobility,
                        ueMobilities,
                        ueElements,
                        ueArrays,
                        baseGainDb,
                        hpbwDeg,
                        maxAttenDb,
                        period);
}

double
LocalRadiansToDegrees(double radians)
{
    return radians * 180.0 / M_PI;
}

double
NormalizeLongitudeDeg(double longitudeDeg)
{
    while (longitudeDeg > 180.0)
    {
        longitudeDeg -= 360.0;
    }
    while (longitudeDeg <= -180.0)
    {
        longitudeDeg += 360.0;
    }
    return longitudeDeg;
}

double
ComputeCentralAngleForElevation(double satelliteAltitudeM, double elevationDeg)
{
    const double earthRadiusM = GeographicPositions::EARTH_SPHERE_RADIUS;
    const double satelliteRadiusM = earthRadiusM + satelliteAltitudeM;
    const double targetElevationRad = LocalDegreesToRadians(std::clamp(elevationDeg, 0.0, 89.999));
    const double horizonAngleRad = std::acos(earthRadiusM / satelliteRadiusM);

    auto elevationForCentralAngle = [earthRadiusM, satelliteRadiusM](double centralAngleRad) {
        const double upComponent = satelliteRadiusM * std::cos(centralAngleRad) - earthRadiusM;
        const double horizontalComponent = satelliteRadiusM * std::sin(centralAngleRad);
        return std::atan2(upComponent, horizontalComponent);
    };

    double low = 0.0;
    double high = horizonAngleRad;
    for (uint32_t i = 0; i < 80; ++i)
    {
        const double mid = 0.5 * (low + high);
        if (elevationForCentralAngle(mid) > targetElevationRad)
        {
            low = mid;
        }
        else
        {
            high = mid;
        }
    }

    return 0.5 * (low + high);
}

Vector
MoveGeographicPoint(const Vector& startGeo, double bearingRad, double centralAngleRad)
{
    const double lat1 = LocalDegreesToRadians(startGeo.x);
    const double lon1 = LocalDegreesToRadians(startGeo.y);

    const double sinLat1 = std::sin(lat1);
    const double cosLat1 = std::cos(lat1);
    const double sinDistance = std::sin(centralAngleRad);
    const double cosDistance = std::cos(centralAngleRad);

    const double lat2 =
        std::asin(sinLat1 * cosDistance + cosLat1 * sinDistance * std::cos(bearingRad));
    const double lon2 = lon1 + std::atan2(std::sin(bearingRad) * sinDistance * cosLat1,
                                          cosDistance - sinLat1 * std::sin(lat2));

    return Vector(LocalRadiansToDegrees(lat2),
                  NormalizeLongitudeDeg(LocalRadiansToDegrees(lon2)),
                  0.0);
}

// Where to put UE 0 so the satellite is at initialElevationDeg now and passes overhead later: on
// the satellite's ground track, ahead of it by the central angle of that elevation.
Vector
ComputeGroundStationPositionForInitialElevation(const Vector& satellitePositionECEF,
                                                const Vector& satelliteVelocityECEF,
                                                double initialElevationDeg,
                                                double headingLookaheadSeconds)
{
    const Vector satellitePositionGEO =
        GeographicPositions::CartesianToGeographicCoordinates(satellitePositionECEF,
                                                              GeographicPositions::SPHERE);
    const Vector subSatellitePointGEO(satellitePositionGEO.x, satellitePositionGEO.y, 0.0);

    if (initialElevationDeg >= 89.999)
    {
        return subSatellitePointGEO;
    }

    const double centralAngleRad =
        ComputeCentralAngleForElevation(satellitePositionGEO.z, initialElevationDeg);
    const double lookaheadSeconds = std::max(headingLookaheadSeconds, 1.0);
    const Vector futureSatellitePositionECEF =
        satellitePositionECEF + satelliteVelocityECEF * lookaheadSeconds;
    const Vector futureSatellitePositionGEO =
        GeographicPositions::CartesianToGeographicCoordinates(futureSatellitePositionECEF,
                                                              GeographicPositions::SPHERE);
    const Vector futureSubSatellitePointGEO(futureSatellitePositionGEO.x,
                                            futureSatellitePositionGEO.y,
                                            0.0);
    const Vector futureTrackEnu =
        GeographicPositions::GeographicToTopocentricCoordinates(futureSubSatellitePointGEO,
                                                                subSatellitePointGEO,
                                                                GeographicPositions::SPHERE);

    const double horizontalNorm =
        std::sqrt((futureTrackEnu.x * futureTrackEnu.x) + (futureTrackEnu.y * futureTrackEnu.y));
    const double bearingRad =
        (horizontalNorm > 0.0) ? std::atan2(futureTrackEnu.x, futureTrackEnu.y) : 0.0;

    return MoveGeographicPoint(subSatellitePointGEO, bearingRad, centralAngleRad);
}

void
LogChannelCondition(Ptr<ChannelConditionModel> channelConditionModel,
                    Ptr<const MobilityModel> satMobility,
                    Ptr<const MobilityModel> ueMobility,
                    Time period,
                    std::ostream* out)
{
    if (!out || !(*out) || !channelConditionModel)
    {
        return;
    }

    Ptr<ChannelCondition> sat2UeCondition =
        channelConditionModel->GetChannelCondition(satMobility, ueMobility);
    const bool gtSat1IsLos = true;
    const bool sat2UeIsLos = sat2UeCondition && sat2UeCondition->IsLos();
    const bool endToEndIsLos = sat2UeIsLos;

    *out << Simulator::Now().GetSeconds() << "\t" << (endToEndIsLos ? 1 : 0) << "\t";
    if (endToEndIsLos)
    {
        *out << "LOS";
    }
    else
    {
        *out << "NLOS";
    }
    *out << "\t" << (gtSat1IsLos ? 1 : 0) << "\t";
    *out << "LOS";
    *out << "\t" << (sat2UeIsLos ? 1 : 0) << "\t";
    if (sat2UeCondition)
    {
        *out << sat2UeCondition->GetLosCondition();
    }
    else
    {
        *out << "UNKNOWN";
    }
    *out << std::endl;

    Simulator::Schedule(period,
                        &LogChannelCondition,
                        channelConditionModel,
                        satMobility,
                        ueMobility,
                        period,
                        out);
}

double
SlantDistance(const Vector& aEcef, const Vector& bEcef)
{
    return (aEcef - bEcef).GetLength();
}

double
ElevationAngle(const Vector& satelliteEcef, const Vector& groundGeo)
{
    const Vector satelliteGeo =
        GeographicPositions::CartesianToGeographicCoordinates(satelliteEcef,
                                                              GeographicPositions::SPHERE);
    const Vector enu =
        GeographicPositions::GeographicToTopocentricCoordinates(satelliteGeo,
                                                                groundGeo,
                                                                GeographicPositions::SPHERE);
    const double distance = enu.GetLength();
    if (distance == 0.0)
    {
        return 90.0;
    }
    return std::asin(enu.z / distance) * 180.0 / M_PI;
}

// Worst-case loop RTT of the simulated window, for the cell-specific K_offset and the RRC
// timer: twice NrNtnPropagationDelayModel::GetOneWayDelay() from the first satellite, evaluated
// on a probe model whose endpoints are moved along the predicted geometry, so the sweep and the
// live channel cannot disagree on what the loop is (feeder + service both ways plus the relay
// processing, or the service hop only). Over every UE. The window ends where the run would: at
// --duration, or when the satellite descends through the elevation boundary or below UE 0's
// horizon.
struct LoopRttSweep
{
    Time maxLoopRtt;
    Time maxLoopRttAt;
    double minElevationDeg{90.0};
    Time windowEnd;
};

LoopRttSweep
SweepLoopRtt(const SatellitePredictor& satellite,
             const Vector& gatewayEcef,
             const std::vector<Vector>& uePositions,
             const std::vector<Vector>& ueVelocities,
             const Vector& ue0Geo,
             bool transparent,
             Time relayProcessingDelay,
             Time duration,
             Time step,
             double boundaryElevationDeg)
{
    Ptr<ConstantPositionMobilityModel> gatewayProbe = CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> satProbe = CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> ueProbe = CreateObject<ConstantPositionMobilityModel>();
    gatewayProbe->SetPosition(gatewayEcef);
    Ptr<NrNtnPropagationDelayModel> probe = CreateObject<NrNtnPropagationDelayModel>();
    probe->SetAttribute("RelayProcessingDelay", TimeValue(relayProcessingDelay));
    if (transparent)
    {
        probe->SetFeeder(gatewayProbe);
    }

    LoopRttSweep sweep;
    sweep.windowEnd = duration;
    for (Time t = Seconds(0); t <= duration; t += step)
    {
        const Vector sat = satellite.At(t);
        const double elevationDeg = ElevationAngle(sat, ue0Geo);
        if (t.IsStrictlyPositive() && (elevationDeg <= boundaryElevationDeg || elevationDeg <= 0.0))
        {
            sweep.windowEnd = t;
            break;
        }
        sweep.minElevationDeg = std::min(sweep.minElevationDeg, elevationDeg);
        satProbe->SetPosition(sat);
        Time maxLoopRtt;
        for (std::size_t i = 0; i < uePositions.size(); ++i)
        {
            ueProbe->SetPosition(uePositions[i] + ueVelocities[i] * t.GetSeconds());
            maxLoopRtt = std::max(maxLoopRtt, 2 * probe->GetOneWayDelay(satProbe, ueProbe));
        }
        if (maxLoopRtt > sweep.maxLoopRtt)
        {
            sweep.maxLoopRtt = maxLoopRtt;
            sweep.maxLoopRttAt = t;
        }
    }
    return sweep;
}

// One standard channel per carrier frequency: the 3GPP NTN model of the service link from the
// satellite, where the gNB sits, with the feeder link on top of it for a transparent payload.
struct NtnCarrier
{
    Ptr<SpectrumChannel> channel;
    Ptr<PropagationLossModel> serviceLoss; //!< satellite <-> UE
    /// Wraps serviceLoss as the channel's loss model when transparent; reports the feeder link
    /// either way
    Ptr<NrNtnFeederNoiseLossModel> feeder;
    Ptr<ChannelConditionModel> channelConditionModel;
    bool mimoChannelEnabled{false};
};

// Downlink path loss in the bent-pipe convention, relative to the gateway transmit power: the
// service hop rebased onto the satellite EIRP (regenerative), or the loss that takes the
// gateway power down to the UE's amplify-and-forward SNR above its noise floor (transparent).
double
DownlinkPathLossDb(Ptr<NrNtnFeederNoiseLossModel> feeder,
                   double feederLossDb,
                   double serviceLossDb,
                   bool transparent,
                   double gatewayTxPowerDbm,
                   double satEirpDbm,
                   double noiseFloorDbm)
{
    if (!transparent)
    {
        return serviceLossDb - (satEirpDbm - gatewayTxPowerDbm);
    }
    return gatewayTxPowerDbm - noiseFloorDbm -
           feeder->GetDownlinkSnrDb(satEirpDbm, feederLossDb, serviceLossDb);
}

// The link budget of UE 0 through the first satellite, one row per period, in the bent-pipe
// trace format (the ISL column stays for the scripts and is 0).
void
LogLinkBudget(Ptr<MobilityModel> gatewayMobility,
              Ptr<MobilityModel> satMobility,
              Ptr<MobilityModel> ueMobility,
              NtnCarrier dlCarrier,
              NtnCarrier ulCarrier,
              bool transparent,
              double satAntennaGainDb,
              Vector gatewayGeo,
              Vector ueGeo,
              Time period,
              std::ostream* out,
              double gatewayTxPowerDbm,
              double satFeederRxGainDb,
              double satEirpDbm,
              double relayLossDb,
              double noiseFloorDbm,
              double initialFeederLossDb,
              double initialServiceLossDb)
{
    if (!out || !(*out))
    {
        return;
    }

    const Vector gt = gatewayMobility->GetPosition();
    const Vector sat = satMobility->GetPosition();
    const Vector ue = ueMobility->GetPosition();
    const double feederLossDb = dlCarrier.feeder->GetFeederLossDb(satMobility);
    const double serviceLossDb = -dlCarrier.serviceLoss->CalcRxPower(0.0, satMobility, ueMobility);
    const double feederLossDeltaDb = feederLossDb - initialFeederLossDb;
    const double serviceLossDeltaDb = serviceLossDb - initialServiceLossDb;
    const double radioOneWayDelayMs =
        ((transparent ? SlantDistance(gt, sat) : 0.0) + SlantDistance(sat, ue)) / SpeedOfLightMps *
        1000.0;
    const double dlPathLossDb = DownlinkPathLossDb(dlCarrier.feeder,
                                                   feederLossDb,
                                                   serviceLossDb,
                                                   transparent,
                                                   gatewayTxPowerDbm,
                                                   satEirpDbm,
                                                   noiseFloorDbm);
    const double ulServiceLossDb =
        -ulCarrier.serviceLoss->CalcRxPower(0.0, ueMobility, satMobility);
    const double ulPathLossDb = ulServiceLossDb - satAntennaGainDb; // the feeder adds only delay
    const double snrSat1 = gatewayTxPowerDbm + satFeederRxGainDb - feederLossDb - noiseFloorDbm;
    const double snrSat2 = snrSat1 - relayLossDb;
    const double gamma2 = satEirpDbm - serviceLossDb - noiseFloorDbm;

    *out << Simulator::Now().GetSeconds() << "\t" << SlantDistance(gt, sat) << "\t" << 0.0 << "\t"
         << SlantDistance(sat, ue) << "\t" << ElevationAngle(sat, gatewayGeo) << "\t"
         << ElevationAngle(sat, ueGeo) << "\t" << dlPathLossDb << "\t" << ulPathLossDb << "\t"
         << feederLossDb << "\t" << feederLossDeltaDb << "\t" << serviceLossDb << "\t"
         << serviceLossDeltaDb << "\t" << (feederLossDeltaDb + serviceLossDeltaDb) << "\t"
         << radioOneWayDelayMs << "\t" << snrSat1 << "\t" << snrSat2 << "\t" << gamma2 << "\t"
         << AmplifyForwardEndToEndSnrDb(snrSat2, gamma2) << "\t"
         << gatewayTxPowerDbm - dlPathLossDb - noiseFloorDbm << std::endl;

    Simulator::Schedule(period,
                        &LogLinkBudget,
                        gatewayMobility,
                        satMobility,
                        ueMobility,
                        dlCarrier,
                        ulCarrier,
                        transparent,
                        satAntennaGainDb,
                        gatewayGeo,
                        ueGeo,
                        period,
                        out,
                        gatewayTxPowerDbm,
                        satFeederRxGainDb,
                        satEirpDbm,
                        relayLossDb,
                        noiseFloorDbm,
                        initialFeederLossDb,
                        initialServiceLossDb);
}

// Stops the run once the satellite, having risen above the boundary, descends through it.
void
StopAtElevationBoundary(Ptr<MobilityModel> satMobility,
                        Vector ueGeo,
                        double boundaryElevationDeg,
                        Time period,
                        bool hasEnteredPass)
{
    const double elevationDeg = ElevationAngle(satMobility->GetPosition(), ueGeo);
    const bool enteredPass = hasEnteredPass || elevationDeg > boundaryElevationDeg + 0.1;
    if (enteredPass && elevationDeg <= boundaryElevationDeg)
    {
        std::clog << "Elevation boundary reached at " << Simulator::Now()
                  << ": elevation=" << elevationDeg << " boundary=" << boundaryElevationDeg
                  << " degrees" << std::endl;
        Simulator::Stop();
        return;
    }

    Simulator::Schedule(period,
                        &StopAtElevationBoundary,
                        satMobility,
                        ueGeo,
                        boundaryElevationDeg,
                        period,
                        enteredPass);
}

// The satellite gNB a UE is attached to, or the closest one until the attachment is recorded
// (NrHelper::AttachWindow can defer it), which is the one AttachToClosestGnb picks.
Ptr<MobilityModel>
ServingGnbMobility(Ptr<NrUeNetDevice> ueDevice,
                   Ptr<MobilityModel> ueMobility,
                   const NetDeviceContainer& gnbDevices)
{
    if (ueDevice->GetTargetGnb())
    {
        return ueDevice->GetTargetGnb()->GetNode()->GetObject<MobilityModel>();
    }
    Ptr<MobilityModel> closest;
    double closestM = std::numeric_limits<double>::max();
    for (uint32_t i = 0; i < gnbDevices.GetN(); ++i)
    {
        Ptr<MobilityModel> candidate = gnbDevices.Get(i)->GetNode()->GetObject<MobilityModel>();
        const double distanceM = candidate->GetDistanceFrom(ueMobility);
        if (distanceM < closestM)
        {
            closestM = distanceM;
            closest = candidate;
        }
    }
    return closest;
}

// Points a satellite's arrays (one per BWP) at the beam centre and invalidates the cached
// channel matrices on both ends, then reschedules itself. The 3GPP channel model computes ray
// angles in ECEF, so alpha and beta are ECEF angles too: local ENU angles would drift from the
// channel's frame as the satellite moves.
void
UpdateAntennaOrientation(Ptr<Node> node,
                         std::vector<Ptr<UniformPlanarArray>> satelliteAntennas,
                         std::vector<Ptr<PhasedArrayModel>> groundAntennas,
                         Vector beamCentreEcef,
                         Time period)
{
    const Angles angles(beamCentreEcef, node->GetObject<MobilityModel>()->GetPosition());
    for (auto& satelliteAntenna : satelliteAntennas)
    {
        satelliteAntenna->SetAlpha(angles.GetAzimuth());
        satelliteAntenna->SetBeta(angles.GetInclination());
        satelliteAntenna->SetBeamformingVector(satelliteAntenna->GetBeamformingVector());
    }
    for (auto& groundAntenna : groundAntennas)
    {
        groundAntenna->SetBeamformingVector(groundAntenna->GetBeamformingVector());
    }
    Simulator::Schedule(period,
                        &UpdateAntennaOrientation,
                        node,
                        satelliteAntennas,
                        groundAntennas,
                        beamCentreEcef,
                        period);
}

// --traceFile: the stream, and UE 0, the reference of its elevation and slant columns
std::ostream* g_geometryTrace = nullptr;
Vector g_ue0Geo;
Vector g_ue0Ecef;

// The end of the satellite's antenna boresight at UE 0's range, the second point of a trace row
Vector
GeneratePoint(const Vector& nodePosECEF, double azimuth, double inclination)
{
    const Vector nodePosGEO =
        GeographicPositions::CartesianToGeographicCoordinates(nodePosECEF,
                                                              GeographicPositions::SPHERE);
    const double length = (g_ue0Ecef - nodePosECEF).GetLength();
    const double s = std::sin(inclination);
    const Vector topo(s * std::cos(azimuth) * length,
                      s * std::sin(azimuth) * length,
                      std::cos(inclination) * length);
    const Vector geo =
        GeographicPositions::TopocentricToGeographicCoordinates(topo,
                                                                nodePosGEO,
                                                                GeographicPositions::SPHERE);
    return GeographicPositions::GeographicToCartesianCoordinates(geo.x,
                                                                 geo.y,
                                                                 geo.z,
                                                                 GeographicPositions::SPHERE);
}

// Time:Satellite:x:y:z:x_2:y_2:z_2:elevation_deg:slant_distance_m, on every satellite
// CourseChange (the orbit model fires one every --precision)
void
CourseChange(Ptr<const MobilityModel> mob)
{
    Ptr<const Node> node = mob->GetObject<Node>();
    Ptr<UniformPlanarArray> antenna = node ? node->GetObject<UniformPlanarArray>() : nullptr;
    if (!g_geometryTrace || !antenna)
    {
        return;
    }
    const Vector pos = mob->GetPosition();
    const Vector boresight = GeneratePoint(pos, antenna->GetAlpha(), antenna->GetBeta());
    *g_geometryTrace << Simulator::Now() << ":" << node->GetId() << ":" << pos.x << ":" << pos.y
                     << ":" << pos.z << ":" << boresight.x << ":" << boresight.y << ":"
                     << boresight.z << ":" << ElevationAngle(pos, g_ue0Geo) << ":"
                     << SlantDistance(pos, g_ue0Ecef) << std::endl;
}

// UDP source that keeps the downlink backlogged (--udpRateMbps <= 0): bursts at 1 Gbps.
class BackloggedUdpSourceApplication : public Application
{
  public:
    void Setup(const Address& remote, uint32_t packetSize)
    {
        m_remote = remote;
        m_packetSize = packetSize;
        constexpr double backloggedRateBps = 1e9;
        const double packetsPerBurst =
            (backloggedRateBps * m_burstInterval.GetSeconds()) / (8.0 * m_packetSize);
        m_packetsPerBurst =
            std::max<uint32_t>(1, static_cast<uint32_t>(std::ceil(packetsPerBurst)));
    }

  private:
    void StartApplication() override
    {
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_socket->SetAllowBroadcast(true);
        m_socket->Connect(m_remote);
        m_sendEvent = Simulator::ScheduleNow(&BackloggedUdpSourceApplication::SendBurst, this);
    }

    void StopApplication() override
    {
        Simulator::Cancel(m_sendEvent);
        if (m_socket)
        {
            m_socket->Close();
            m_socket = nullptr;
        }
    }

    void SendBurst()
    {
        for (uint32_t i = 0; m_socket && i < m_packetsPerBurst; ++i)
        {
            if (m_socket->Send(Create<Packet>(m_packetSize)) < 0)
            {
                break;
            }
        }
        m_sendEvent =
            Simulator::Schedule(m_burstInterval, &BackloggedUdpSourceApplication::SendBurst, this);
    }

    Address m_remote;
    uint32_t m_packetSize{0};
    Ptr<Socket> m_socket;
    EventId m_sendEvent;
    Time m_burstInterval{MilliSeconds(1)};
    uint32_t m_packetsPerBurst{1};
};

NtnCarrier
CreateNtnCarrier(double frequencyHz,
                 const std::string& scenario,
                 Ptr<MobilityModel> gatewayMobility,
                 Ptr<NrNtnPropagationDelayModel> delayModel,
                 bool transparent)
{
    NtnCarrier carrier;
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories(scenario, "Default", "ThreeGpp");
    carrier.channel = channelHelper->CreateChannel(NrChannelHelper::INIT_PROPAGATION |
                                                   NrChannelHelper::INIT_FADING);
    carrier.serviceLoss = carrier.channel->GetPropagationLossModel();
    carrier.serviceLoss->SetAttributeFailSafe("Frequency", DoubleValue(frequencyHz));
    if (Ptr<PhasedArraySpectrumPropagationLossModel> sat2UeSpectrumLoss =
            carrier.channel->GetPhasedArraySpectrumPropagationLossModel())
    {
        PointerValue channelModelValue;
        if (sat2UeSpectrumLoss->GetAttributeFailSafe("ChannelModel", channelModelValue))
        {
            channelModelValue.Get<Object>()->SetAttributeFailSafe("Frequency",
                                                                  DoubleValue(frequencyHz));
        }
        carrier.mimoChannelEnabled = true;
    }
    if (auto threeGppLoss = DynamicCast<ThreeGppPropagationLossModel>(carrier.serviceLoss))
    {
        carrier.channelConditionModel = threeGppLoss->GetChannelConditionModel();
    }
    carrier.channel->SetPropagationDelayModel(delayModel);
    carrier.channel->AddSpectrumTransmitFilter(CreateObject<NrNtnUeToUeFilter>());

    // The feeder link is line of sight: the gateway is sited for it
    Ptr<NrChannelHelper> feederHelper = CreateObject<NrChannelHelper>();
    feederHelper->ConfigureFactories(scenario, "LOS", "ThreeGpp");
    Ptr<PropagationLossModel> feederLoss =
        feederHelper->CreateChannel(NrChannelHelper::INIT_PROPAGATION)->GetPropagationLossModel();
    feederLoss->SetAttributeFailSafe("Frequency", DoubleValue(frequencyHz));
    carrier.feeder = CreateObject<NrNtnFeederNoiseLossModel>();
    carrier.feeder->SetServiceLoss(carrier.serviceLoss);
    carrier.feeder->SetFeeder(gatewayMobility, feederLoss);
    if (transparent)
    {
        carrier.channel->SetAttribute("PropagationLossModel", PointerValue(carrier.feeder));
    }
    return carrier;
}
} // namespace

int
main(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    std::string payload = "transparent";
    std::string orbitFile;
    std::string traceFile;
    std::string linkBudgetTraceFile;
    std::string channelConditionTraceFile;
    std::string scenario = "NTN-Rural";
    uint32_t precision = 1000; // milliseconds
    // Channel regeneration period, decoupled from the orbit/mobility step. The satellite moves
    // slowly enough for a coarse mobility step, but the NTN channel is fast time-varying, and
    // tying both to one knob makes a ms-level channel cost a ms-level beamforming recomputation
    // for no physical reason. 0 = follow --precision (previous behaviour).
    uint32_t channelUpdateMs = 0;
    uint32_t duration = 4; // seconds
    // FDD carrier pair. The downlink keeps the fork's 3.5 GHz / 100 MHz; the uplink sits
    // 190 MHz below it, the duplex spacing of 3GPP band n256 (S-band NTN).
    double dlFrequencyHz = 3.5e9; // DL carrier centre in Hz (BWP 0)
    double ulFrequencyHz = 0.0;   // UL carrier centre in Hz (BWP 1); 0 = dlFrequencyHz - 190 MHz
    double bandwidthHz = 100e6;   // Hz, per direction
    // Service link: the satellite EIRP density, delivered as gNB power minus the satellite
    // element gain plus that gain (which also closes the uplink). The presets set satEIRP.
    double satEIRP = 34.0;         // dBW/MHz
    double satAntennaGainDb = 0.0; // dBi
    double ueTxPowerDbm = 23.0;
    // Transparent feeder link: gateway -> serving satellite
    double gatewayTxPowerDbm = 60.0;
    double satFeederRxGainDb = 30.0;
    double relayLossDb = 0.0;
    double gatewayLatDeg = std::numeric_limits<double>::quiet_NaN(); // NaN: UE 0's position
    double gatewayLonDeg = std::numeric_limits<double>::quiet_NaN();
    // Receiver noise figure at every node whose own --ueNoiseFigureDb / --gnbNoiseFigureDb is
    // unset. 5 dB is NrUePhy/NrGnbPhy's own default, so leaving it alone changes nothing. The
    // feeder model turns the relay SINR into a path loss against this same noise, so it must
    // be the figure the UE PHY really applies, or the PHY SINR drifts from the channel's.
    double receiverNfDb = 5.0;
    double noiseFloorDbm = -174.0 + 10.0 * std::log10(bandwidthHz) + receiverNfDb;
    double relayProcessingDelayUs = 0.0;
    double remoteHostDelayMs = 0.0;
    double s1uLinkDelayMs = 0.0;
    std::string transport = "udp";
    double udpRateMbps = 10.0; // <= 0: backlogged source
    uint32_t udpPacketSize = 1200;
    // Without consolidation every slot carrying a HARQ retransmission splits into two
    // time-multiplexed symbol groups, each using under half the band. Default on here.
    bool consolidateHarqRetx = true;
    // Beam-pointing refresh period. Near zenith the azimuth rate spikes, so a coarse
    // period may under-track the UE and depress SINR there.
    uint32_t beamUpdateMs = 100;
    bool useFixedMcs = false;
    uint32_t fixedMcs = 4;
    // Outer-loop link adaptation. On by default here, unlike upstream 5G-LENA: over a bent-pipe
    // path the CQI is stale by the loop RTT plus K_offset (about 17 ms at 620 km), so the inner
    // loop alone is biased high and never corrects. Real schedulers (srsRAN, OAI, vendor
    // "IBLER target") drive the MCS from measured first-transmission error rate.
    double ollaStepUp = 0.01;
    // 0.15, not the usual 0.1: with HARQ retransmission and soft combining a NACK is recovered,
    // not lost, so a target that treats errors as expensive gives up more MCS than it saves.
    // Measured at 29 dB single-pol: 0.15 delivers 67.1 MB against 61.7 MB at 0.10, closing most
    // of the gap to open-loop. Above about 25 dB with dual polarisation the MCS pins at 28 and
    // the target stops mattering.
    double ollaTargetBler = 0.15;
    // The measured stale-CQI bias on this path is about 5.5 MCS steps, so upstream's clamp of 4
    // pins the loop against its floor and holds first-transmission BLER near 0.16 whatever the
    // step size. Give the loop room to reach its target; a run that sits at the clamp is
    // measuring the clamp.
    double ollaMaxOffset = 12.0;
    // Rel-17 NTN downlinkHARQ-FeedbackDisabled. Distinct from --enableHarqReTx=0, which still
    // parks the process for a full loop RTT waiting for a report it ignores.
    bool dlHarqFeedbackDisabled = false;
    // RLC AM carries ARQ above HARQ; its timers default to terrestrial values that are shorter
    // than an NTN round trip, which makes AM retransmit blocks still in flight.
    std::string rlcMode = "um";
    uint32_t rlcAmPollRetxMs = 25;
    uint32_t rlcAmReorderingMs = 40;
    uint32_t rlcAmStatusProhibitMs = 10;
    bool rlcAmRetxAlwaysFits = false; // false = 3GPP behaviour, a retx too big is re-segmented
    bool perfectUplink =
        false; // disable the gNB data error model so UL (RLC AM STATUS) never fails
    // Receiver noise figures, referenced to T0 = 290 K as ns-3 requires, so they express the
    // whole system temperature rather than just the front end. A Ku VSAT terminal runs a
    // ~1.5 dB LNB (Te 120 K) and looks at cold sky (~50 K), giving Tsys ~170 K, which is
    // BELOW 290 K and therefore a negative effective figure. A satellite receiver looks at
    // the warm Earth (~290 K), so its system temperature stays high. -999 keeps the ns-3
    // default (5 dB) so existing runs are unchanged unless the flag is given.
    std::string ueAntennaElement = "isotropic"; // "isotropic" or "3gpp" (8 dBi element)
    // Flat gain applied to each UE antenna element. A VSAT dish is a single high-gain
    // aperture, not an array of isotropic elements: modelling it as one gives the right
    // gain in both directions with no array pattern, no beam squint and no large-matrix
    // channel behaviour. The published study parameter used here is 34.6 dBi at boresight.
    double ueAntennaElementGainDbi = 0.0;
    double ueNoiseFigureDb = -999.0;
    double gnbNoiseFigureDb = -999.0;
    // NTN needs retransmissions: at the target BLER of 0.1 one TB in ten fails, and with this
    // off those bytes are simply lost (48 % loss at the old fixed-MCS default).
    bool enableHarqReTx = true;
    uint32_t numHarqProcess = 16;
    bool lbrm = false;
    uint32_t n1Delay = 2;
    uint32_t n2Delay = 2;
    uint32_t rlcReorderingTimerMs = 100;
    uint32_t numUes = 1;
    double ueSpacingDeg = 0.05;
    double footprintRadiusKm = 0.0;
    double beamHpbwDeg = 0.0;     // >0: geometry-driven spot-beam taper, half-power beamwidth (deg)
    double beamMaxAttenDb = 30.0; // floor of the spot-beam pattern attenuation
    // Per-UE bearing in degrees clockwise from north, one entry per UE after UE 0. Empty keeps
    // the legacy latitude-only spread.
    std::string ueAzimuthsDegStr = "";
    // Terminal ground speed. The channel model's Doppler term reads each node's velocity, so a
    // stationary UE leaves fading entirely to satellite motion (~23 ms coherence at 2 GHz). A
    // moving terminal shortens coherence to v/lambda, which is the only way at LEO to reach the
    // regime where the control loop outlasts the channel.
    // Freeze the satellite at its initial position. The point is a terrestrial analogue: a
    // static gNB and UE, so elevation, slant range and loop delay all stay constant and the only
    // channel variation left is whatever the UE's own motion produces. Without this a
    // "no-propagation-delay" arm still sweeps elevation, and a rising run and a falling run from
    // the same start cover different parts of the pass -- measured at up to 71 % apart on
    // throughput with no loop delay involved at all.
    bool freezeGeometry = false;
    double ueSpeedMps = 0.0;
    double ueHeadingDeg = 0.0;
    std::string ueRadiiKmStr =
        ""; // explicit per-UE radial distances "0,30,30,50" (overrides auto-placement)
    std::string schedulerType = "ns3::NrMacSchedulerTdmaRR";
    std::string observerMode = "overhead";
    double observerElevationBoundaryDeg = 10.0;
    // Aim the pass at a real place. ntn/tools/aim_pass.py solves the satellite's starting orbital
    // position for a pass directly overhead a latitude/longitude and prints these four flags.
    // Unset (NaN) keeps the placement observerMode derives from the orbit.
    double observerLatDeg = std::numeric_limits<double>::quiet_NaN();
    double observerLonDeg = std::numeric_limits<double>::quiet_NaN();
    double satLongitudeDeg = std::numeric_limits<double>::quiet_NaN();
    double satArgOfLatitudeDeg = std::numeric_limits<double>::quiet_NaN();
    // NTN timing: cell-specific K_offset, ideal timing advance and the RRC connection-request
    // timer, all derived from the geometry of the run (see the file header).
    bool ntnTiming = true;
    // Ideal random access: the RA response timeout is never armed, so attach cannot fail at
    // elevations where the RAR window does not fit the round trip.
    bool idealRa = true;
    bool enableMimoFeedback = true;
    std::string pmSearchMethod = "ns3::NrPmSearchFull";
    std::string fullSearchCb = "ns3::NrCbTwoPort";
    uint32_t rankLimit = 2;
    double rankThreshold = 0.0;
    std::string rankTechnique = "Sasaoka";
    uint32_t subbandSize = 32;
    std::string downsamplingTechnique = "FirstPRB";
    uint32_t ueAntennaRows = 1;
    uint32_t ueAntennaColumns = 2;
    uint32_t ueNumVerticalPorts = 1;
    uint32_t ueNumHorizontalPorts = 2;
    uint32_t gnbAntennaRows = 2;
    uint32_t gnbAntennaColumns = 2;
    uint32_t gnbNumVerticalPorts = 1;
    uint32_t gnbNumHorizontalPorts = 1;
    // Cross-polar reuse is how a LOS satellite link earns a second layer: the NTN channel has
    // almost no spatial multiplexing, so with a single polarisation the UE sits at rank 1 and
    // half the capacity is unreachable. Measured at 29 dB: rank 2 on 100 % of TBs and 740 Mbps,
    // against 439 Mbps single-pol. The gNB was already dual-polarised.
    bool ueIsDualPolarized = true;
    bool gnbIsDualPolarized = true;
    double uePolSlantAngleDeg = 90.0;
    double gnbPolSlantAngleDeg = 0.0;

    // --- 3GPP TR 38.821 calibration presets ---------------------------------------------
    // Applied just before cmd.Parse, so an explicit flag on the command line still wins.
    //
    //   sc9handset   study case 9, the handheld reference and the DEFAULT here: LEO-600 seen
    //                at 30 deg elevation (1075 km slant), S-band, 30 MHz, EIRP 78.8 dBm =
    //                34 dBW/MHz, 0 dBi terminal elements, 7 dB noise figure. DL CNR 6.6 dB in
    //                Table 6.1.3.3-1, with 0 dB polarisation loss and a 3 dB shadowing margin.
    //                The simulation draws its own shadowing instead of subtracting a margin, so
    //                its median DL SNR at 30 deg compares with about 9.6 dB. The elevation is
    //                recovered from the FSPL column, 159.1 dB at 2 GHz being a 1075 km slant.
    //   sc14handset  study case 14: the same handheld under LEO-1200, 30 MHz, EIRP 84.8 dBm =
    //                40 dBW/MHz (Set-1 LEO-1200 S-band). FSPL 164.5 dB = 1999 km slant, i.e.
    //                30 deg again; DL CNR 7.2 dB with the same 3 dB shadowing margin.
    //   sc15handset  study case 15: sc14handset with frequency reuse option 2, 10 MHz,
    //                EIRP 80.0 dBm, same density and CNR.
    //   sc6vsat      study case 6, the VSAT reference: LEO-600 at 30 deg, Ka-band, 400 MHz,
    //                EIRP 60 dBm = 3.98 dBW/MHz, 39.7 dBi terminal, DL CNR 8.5 dB with no
    //                margin and no polarisation loss. The terminal's 150 K antenna
    //                temperature and 1.2 dB noise figure give 242 K, which ns-3's
    //                T0-referenced noise figure reaches at -0.78 dB.
    //
    // Every preset also sets the Set-1 satellite of Table 6.1.1.1-1: its Tx/Rx antenna gain
    // (--satAntennaGainDb; the gNB power is the EIRP minus it, so the downlink is unchanged and
    // the uplink gains it), its G/T as --gnbNoiseFigureDb, and its 3 dB beamwidth for the spot
    // beam taper (--beamHpbwDeg), and the terminal's transmit power (Table 6.1.1.1-3).
    //
    // Both presets use the Table 6.1.1.1-3 terminal, one element position with two +/-45 deg
    // cross-polarised ports, whose two Rx branches combine with no depolarisation loss
    // (Sec. 6.1.1.1). The satellite's beam gain is already in its EIRP, so the gNB is one element
    // position too (its array would otherwise add gain on top of the EIRP), and one stream per
    // beam means rank 1.
    //
    // The presets set the SERVICE link and the default orbit altitude (600 km, 1200 km for
    // sc14handset and sc15handset; --orbitFile replaces the orbit), not the elevation, which the
    // pass sets. With the transparent payload the UE also sees the feeder hop through the AF
    // cascade, so its SNR lands below the study case CNR.
    std::string ntnPreset = "sc9handset";
    bool subbandSizeSetByUser = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a(argv[i]);
        if (a.rfind("--ntnPreset=", 0) == 0)
        {
            ntnPreset = a.substr(12);
        }
        if (a.rfind("--subbandSize=", 0) == 0)
        {
            subbandSizeSetByUser = true;
        }
    }
    double presetAltitudeKm = 600;
    if (ntnPreset == "sc9handset" || ntnPreset == "sc14handset" || ntnPreset == "sc15handset")
    {
        const bool leo1200 = ntnPreset != "sc9handset";
        dlFrequencyHz = 2e9;
        bandwidthHz = ntnPreset == "sc15handset" ? 10e6 : 30e6;
        scenario = "NTN-Suburban";
        ueAntennaElementGainDbi = 0.0;
        ueNoiseFigureDb = 7.0;
        ueTxPowerDbm = 23.0;
        satEIRP = leo1200 ? 40.0 : 34.0; // dBW/MHz, Set-1 S-band
        presetAltitudeKm = leo1200 ? 1200 : 600;
        // Set-1 S-band satellite: 30 dBi Tx and Rx, 4.4127 deg beam, G/T 1.1 dB/K. G/T is
        // 30 dBi over 776 K, which ns-3's T0-referenced noise figure reaches at 4.28 dB.
        satAntennaGainDb = 30.0;
        gnbNoiseFigureDb = 4.28;
        beamHpbwDeg = 4.4127;
    }
    else if (ntnPreset == "sc6vsat")
    {
        dlFrequencyHz = 20e9;
        // Ka NTN is a 10 GHz split (DL 17.7-20.2, UL 27.5-30); the default 190 MHz
        // duplex spacing cannot hold a 400 MHz carrier.
        ulFrequencyHz = 30e9;
        bandwidthHz = 400e6;
        scenario = "NTN-Suburban";
        ueAntennaElementGainDbi = 39.7;
        ueNoiseFigureDb = -0.78;
        // 2 W into a 43.2 dBi Tx antenna, 76.2 dBm EIRP. The terminal has one element gain for
        // both directions, the 39.7 dBi of its receive side, so the power makes up the rest.
        ueTxPowerDbm = 33.0 + 43.2 - 39.7;
        satEIRP = 60.0 - 30.0 - 10.0 * std::log10(400.0); // dBW/MHz
        // Set-1 Ka-band satellite: 38.5 dBi Tx and Rx, 1.7647 deg beam, G/T 13 dB/K at 30 GHz:
        // 38.5 dBi over 355 K, a 0.88 dB T0-referenced noise figure.
        satAntennaGainDb = 38.5;
        gnbNoiseFigureDb = 0.88;
        beamHpbwDeg = 1.7647;
    }
    else
    {
        NS_FATAL_ERROR("Unknown --ntnPreset '" << ntnPreset
                                               << "'; use sc9handset, sc14handset, "
                                                  "sc15handset or sc6vsat");
    }
    ueAntennaRows = 1;
    ueAntennaColumns = 1;
    ueNumVerticalPorts = 1;
    ueNumHorizontalPorts = 1;
    ueIsDualPolarized = true;
    uePolSlantAngleDeg = 45.0;
    gnbAntennaRows = 1;
    gnbAntennaColumns = 1;
    rankLimit = 1;
    // The EIRP density holds per MHz whatever the load. LENA's default spreads the gNB power
    // over the RBs in use, which on a partly loaded carrier transmits above the density.
    Config::SetDefault("ns3::NrGnbPhy::PowerAllocationType", StringValue("UniformPowerAllocBw"));
    cmd.AddValue("ntnPreset",
                 "3GPP TR 38.821 calibration preset for the service link and the default "
                 "orbit: sc9handset (LEO-600 handheld, the default), sc14handset or "
                 "sc15handset (LEO-1200 handheld, 30 or 10 MHz) or sc6vsat (LEO-600 VSAT)",
                 ntnPreset);
    cmd.AddValue("payload",
                 "Satellite payload: transparent (amplify-and-forward relay, the waveform also "
                 "crosses the feeder link from the gateway) or regenerative (the gNB terminates "
                 "NR on the satellite)",
                 payload);
    cmd.AddValue("orbitFile",
                 "CSV file with orbit parameters in the ns-3.48 format (header line, then "
                 "altitudeKm,inclinationDegrees,numberOfPlanes,numberOfSatellitesPerPlane"
                 "[,phasing,raanSpanDeg] per row). Every satellite carries a gNB. Default: one "
                 "satellite at the --ntnPreset altitude",
                 orbitFile);
    cmd.AddValue("traceFile",
                 "Colon-separated satellite geometry trace (Time:Satellite:x:y:z:x_2:y_2:z_2:"
                 "elevation_deg:slant_distance_m), one row per satellite every --precision",
                 traceFile);
    cmd.AddValue("linkBudgetTraceFile",
                 "Tab-separated link budget of UE 0 through the first satellite, every "
                 "--precision",
                 linkBudgetTraceFile);
    cmd.AddValue("channelConditionTraceFile",
                 "Text file to store the 3GPP LOS/NLOS condition of the first satellite -> UE 0",
                 channelConditionTraceFile);
    cmd.AddValue("scenario", "3GPP NTN scenario of the service link", scenario);
    cmd.AddValue("precision",
                 "Mobility resolution in milliseconds: the period of the orbit model's "
                 "CourseChange notifications, of the traces, of the satellite antenna "
                 "re-pointing and (with --channelUpdateMs=0) of the channel regeneration. The "
                 "satellite position itself is continuous in time.",
                 precision);
    cmd.AddValue("channelUpdateMs",
                 "3GPP channel and channel-condition regeneration period in ms, independent of "
                 "the orbit step. 0 follows --precision",
                 channelUpdateMs);
    cmd.AddValue("duration", "Duration of the simulation in seconds", duration);
    cmd.AddValue("dlFrequencyHz", "Downlink carrier centre frequency in Hz (BWP 0)", dlFrequencyHz);
    cmd.AddValue("ulFrequencyHz",
                 "Uplink carrier centre frequency in Hz (BWP 1); 0 derives dlFrequencyHz - 190 MHz",
                 ulFrequencyHz);
    cmd.AddValue("bandwidthHz", "Channel bandwidth in Hz of each FDD carrier", bandwidthHz);
    cmd.AddValue("satEIRP", "Satellite EIRP density in dBW/MHz", satEIRP);
    cmd.AddValue("satAntennaGainDb",
                 "Satellite (gNB) element gain in dBi: the gNB transmits the EIRP minus this "
                 "gain, and the gain also closes the uplink",
                 satAntennaGainDb);
    cmd.AddValue("ueTxPowerDbm", "UE Tx power in dBm", ueTxPowerDbm);
    cmd.AddValue("gatewayTxPowerDbm",
                 "Gateway transmit power on the feeder link in dBm (transparent)",
                 gatewayTxPowerDbm);
    cmd.AddValue("satFeederRxGainDb",
                 "Satellite receive antenna gain on the feeder link in dBi (transparent)",
                 satFeederRxGainDb);
    cmd.AddValue("relayLossDb",
                 "Loss between the relay's receive antenna and its amplifier in dB (transparent)",
                 relayLossDb);
    cmd.AddValue("gatewayLatDeg",
                 "Gateway latitude in degrees (transparent); with --gatewayLonDeg. Default: UE 0",
                 gatewayLatDeg);
    cmd.AddValue("gatewayLonDeg", "Gateway longitude in degrees (transparent)", gatewayLonDeg);
    cmd.AddValue("receiverNfDb",
                 "Receiver noise figure in dB, applied at every node whose --ueNoiseFigureDb / "
                 "--gnbNoiseFigureDb is unset; the transparent feeder model charges the relay "
                 "noise against the UE figure too",
                 receiverNfDb);
    cmd.AddValue("relayProcessingDelayUs",
                 "Processing delay of the transparent relay satellite in us",
                 relayProcessingDelayUs);
    cmd.AddValue("remoteHostDelayMs", "RemoteHost-to-PGW one-way delay in ms", remoteHostDelayMs);
    cmd.AddValue("s1uLinkDelayMs", "S1-U backhaul one-way delay in ms", s1uLinkDelayMs);
    cmd.AddValue("transport", "Application transport: tcp or udp", transport);
    cmd.AddValue("udpRateMbps",
                 "Per-UE UDP offered load in Mbps; <= 0 keeps the downlink backlogged",
                 udpRateMbps);
    cmd.AddValue("udpPacketSize", "Application payload size in bytes", udpPacketSize);
    cmd.AddValue("beamUpdateMs",
                 "Beam-pointing refresh period in ms (ideal beamforming and spot-beam taper)",
                 beamUpdateMs);
    cmd.AddValue("consolidateHarqRetx",
                 "Reshape HARQ retransmission DCIs so they share symbols with new transmissions "
                 "instead of forming a separate time-multiplexed group",
                 consolidateHarqRetx);
    cmd.AddValue("ollaStepUp",
                 "DL outer-loop link adaptation step on ACK, in MCS units. 0 disables OLLA",
                 ollaStepUp);
    cmd.AddValue("ollaTargetBler",
                 "Target initial (first-transmission) BLER for DL OLLA",
                 ollaTargetBler);
    cmd.AddValue("dlHarqFeedbackDisabled",
                 "Rel-17 NTN: release the DL HARQ process at transmission instead of on feedback. "
                 "No retransmissions; reliability must come from RLC AM",
                 dlHarqFeedbackDisabled);
    cmd.AddValue("rlcMode", "RLC mode for data bearers: um or am", rlcMode);
    cmd.AddValue("rlcAmPollRetxMs", "RLC AM t-PollRetransmit in ms", rlcAmPollRetxMs);
    cmd.AddValue("rlcAmReorderingMs", "RLC AM t-Reordering in ms", rlcAmReorderingMs);
    cmd.AddValue("rlcAmStatusProhibitMs", "RLC AM t-StatusProhibit in ms", rlcAmStatusProhibitMs);
    cmd.AddValue("rlcAmRetxAlwaysFits",
                 "RLC AM TxOpportunityForRetxAlwaysBigEnough; true sends a retransmission whole "
                 "even when it overflows the grant, false re-segments it",
                 rlcAmRetxAlwaysFits);
    cmd.AddValue("ueAntennaElementGainDbi",
                 "Flat gain in dBi applied to each UE antenna element. With a 1x1 array this "
                 "models a dish of that boresight gain directly (published study value: 34.6 dBi).",
                 ueAntennaElementGainDbi);
    cmd.AddValue("ueAntennaElement",
                 "UE antenna element pattern: 'isotropic', or '3gpp' for the TR 38.901 element "
                 "with 8 dBi peak. A VSAT dish needs the directional element -- reaching 34.6 dBi "
                 "with isotropic elements would take ~2600 of them instead of ~420.",
                 ueAntennaElement);
    cmd.AddValue("ueNoiseFigureDb",
                 "UE receiver noise figure in dB referenced to T0=290K, i.e. 10log10(Tsys/290). "
                 "A representative Ku-band VSAT assumption is -2.3 dB (Tsys ~170 K: 1.5 dB "
                 "LNB plus cold sky). Unset keeps the ns-3 default of 5 dB.",
                 ueNoiseFigureDb);
    cmd.AddValue("gnbNoiseFigureDb",
                 "gNB/satellite receiver noise figure in dB referenced to T0=290K. A satellite "
                 "sees the warm Earth, so ~7 dB (Tsys ~1453 K). Unset keeps the ns-3 default.",
                 gnbNoiseFigureDb);
    cmd.AddValue("perfectUplink",
                 "Disable the gNB data error model so every UL TB decodes. With a weak uplink "
                 "budget RLC AM STATUS PDUs are otherwise lost and the AM window never "
                 "advances. Diagnostic only.",
                 perfectUplink);
    cmd.AddValue("ollaMaxOffset",
                 "Clamp on the DL OLLA offset in MCS units. A run that spends time clamped is "
                 "measuring the clamp rather than the step size",
                 ollaMaxOffset);
    cmd.AddValue("useFixedMcs", "Use fixed MCS in scheduler for DL/UL", useFixedMcs);
    cmd.AddValue("fixedMcs", "Fixed MCS index to use when --useFixedMcs=1", fixedMcs);
    cmd.AddValue("enableHarqReTx", "Enable HARQ retransmissions in the scheduler", enableHarqReTx);
    cmd.AddValue("numHarqProcess",
                 "Number of HARQ processes per UE. With --ntnTiming a process is kept for "
                 "numHarqProcess + K_offset slots; without it the pool must cover the loop RTT "
                 "in slots on its own.",
                 numHarqProcess);
    cmd.AddValue("lbrm",
                 "Limited-buffer rate matching (TS 38.212 5.4.2.1): bound HARQ-IR combining to "
                 "the soft buffer of one transport block, about 1.5 x TBS_LBRM, where TBS_LBRM "
                 "is the largest block this carrier supports (highest MCS, rankLimit, every RB). "
                 "Ncb has no HARQ-process term in NR, so the buffer does not depend on "
                 "numHarqProcess. Needs a HARQ-IR error model (NrEesmIrT1/T2); off leaves the "
                 "buffer unlimited.",
                 lbrm);
    cmd.AddValue("n1Delay",
                 "UE processing delay in slots for DL HARQ feedback (0-4); not a propagation "
                 "substitute.",
                 n1Delay);
    cmd.AddValue("n2Delay", "Slots between UL grant and UL transmission (0-4)", n2Delay);
    cmd.AddValue("rlcReorderingTimerMs",
                 "NrRlcUm t-Reordering in ms. Must exceed worst-case multi-retx HARQ delay.",
                 rlcReorderingTimerMs);
    cmd.AddValue("numUes", "Number of UEs sharing the beam", numUes);
    cmd.AddValue("ueSpacingDeg",
                 "Geographic latitude offset in degrees between successive UEs, spreading them "
                 "across the beam footprint",
                 ueSpacingDeg);
    cmd.AddValue("footprintRadiusKm",
                 "Beam footprint radius in km (>0 enables centre/edge placement + spot-beam gain "
                 "taper: 0 dB at centre, -3 dB at the footprint edge). 0 disables (uses "
                 "ueSpacingDeg).",
                 footprintRadiusKm);
    cmd.AddValue("beamHpbwDeg",
                 "Half-power beamwidth (deg) of a geometry-driven spot-beam taper "
                 "G(theta)=-min(12*(theta/HPBW)^2, beamMaxAttenDb); 0 disables (static r/R taper)",
                 beamHpbwDeg);
    cmd.AddValue("beamMaxAttenDb", "Max spot-beam pattern attenuation (dB)", beamMaxAttenDb);
    cmd.AddValue("ueAzimuthsDeg",
                 "Comma-separated per-UE bearing in degrees clockwise from north (one per UE "
                 "after UE 0). Combined with --ueRadiiKm this places UEs on a ring at equal "
                 "radius. Empty keeps the latitude-only spread",
                 ueAzimuthsDegStr);
    cmd.AddValue("freezeGeometry",
                 "Hold the satellites at their initial positions: static geometry, constant "
                 "elevation and delay. Use with --ueSpeedMps for a terrestrial analogue",
                 freezeGeometry);
    cmd.AddValue("ueSpeedMps", "UE ground speed in m/s (0 = stationary)", ueSpeedMps);
    cmd.AddValue("ueHeadingDeg", "UE heading in degrees clockwise from north", ueHeadingDeg);
    cmd.AddValue("ueRadiiKm",
                 "Comma-separated per-UE radial distances from beam centre in km "
                 "(e.g. 0,30,30,50); overrides the 2-centre+edge auto-placement",
                 ueRadiiKmStr);
    cmd.AddValue("scheduler",
                 "MAC scheduler TypeId (e.g. ns3::NrMacSchedulerTdmaRR or "
                 "ns3::NrMacSchedulerOfdmaRR)",
                 schedulerType);
    cmd.AddValue("observerMode", "Observer placement mode: overhead or full-pass", observerMode);
    cmd.AddValue("observerElevationBoundaryDeg",
                 "Elevation boundary in degrees: full-pass mode starts UE 0 where the first "
                 "satellite rises through it, and the run stops when the satellite descends "
                 "through it",
                 observerElevationBoundaryDeg);
    cmd.AddValue("observerLatDeg",
                 "UE 0 latitude in degrees; with --observerLonDeg, overrides --observerMode",
                 observerLatDeg);
    cmd.AddValue("observerLonDeg", "UE 0 longitude in degrees", observerLonDeg);
    cmd.AddValue("satLongitudeDeg",
                 "Starting longitude of the first satellite's orbital plane, degrees; with "
                 "--satArgOfLatitudeDeg this sets where the pass goes (see ntn/tools/aim_pass.py)",
                 satLongitudeDeg);
    cmd.AddValue("satArgOfLatitudeDeg",
                 "Starting argument of latitude of the first satellite, degrees",
                 satArgOfLatitudeDeg);
    cmd.AddValue("ntnTiming",
                 "Derive NrGnbPhy::CellSpecificKoffset, the per-UE NrUePhy::TimingAdvance and "
                 "NrGnbRrc::ConnectionRequestTimeoutDuration from the geometry of the run. 0 "
                 "leaves all three at their defaults (the like-for-like arm).",
                 ntnTiming);
    cmd.AddValue("idealRa",
                 "NrUeMac::IdealRandomAccess: never arm the RA response timeout, and pin the "
                 "RRC connection-request timer to its 2000 ms ceiling",
                 idealRa);
    cmd.AddValue("enableMimoFeedback", "Enable DL MIMO CQI/RI/PMI feedback", enableMimoFeedback);
    cmd.AddValue("pmSearchMethod", "MIMO precoding matrix search TypeId", pmSearchMethod);
    cmd.AddValue("fullSearchCb", "Codebook TypeId for full-search PMI", fullSearchCb);
    cmd.AddValue("rankLimit", "Maximum MIMO rank to use", rankLimit);
    cmd.AddValue("rankThreshold", "Rank selection threshold", rankThreshold);
    cmd.AddValue("rankTechnique", "Rank selection technique", rankTechnique);
    cmd.AddValue("subbandSize", "Subband size in PRBs for MIMO PMI search", subbandSize);
    cmd.AddValue("downsamplingTechnique", "Subband downsampling technique", downsamplingTechnique);
    cmd.AddValue("ueAntennaRows", "UE antenna array rows", ueAntennaRows);
    cmd.AddValue("ueAntennaColumns", "UE antenna array columns", ueAntennaColumns);
    cmd.AddValue("ueNumVerticalPorts", "UE vertical antenna ports", ueNumVerticalPorts);
    cmd.AddValue("ueNumHorizontalPorts", "UE horizontal antenna ports", ueNumHorizontalPorts);
    cmd.AddValue("gnbAntennaRows", "gNB antenna array rows", gnbAntennaRows);
    cmd.AddValue("gnbAntennaColumns", "gNB antenna array columns", gnbAntennaColumns);
    cmd.AddValue("gnbNumVerticalPorts", "gNB vertical antenna ports", gnbNumVerticalPorts);
    cmd.AddValue("gnbNumHorizontalPorts", "gNB horizontal antenna ports", gnbNumHorizontalPorts);
    cmd.AddValue("ueIsDualPolarized",
                 "Enable dual-polarized UE antenna elements",
                 ueIsDualPolarized);
    cmd.AddValue("gnbIsDualPolarized",
                 "Enable dual-polarized gNB antenna elements",
                 gnbIsDualPolarized);
    cmd.AddValue("uePolSlantAngleDeg",
                 "UE polarization slant angle in degrees",
                 uePolSlantAngleDeg);
    cmd.AddValue("gnbPolSlantAngleDeg",
                 "gNB polarization slant angle in degrees",
                 gnbPolSlantAngleDeg);

    cmd.Parse(argc, argv);

    // TS 38.214 Table 5.2.1.4-2 ties the CSI subband size to the BWP size in PRBs, and
    // NrPmSearch asserts on a mismatch: 24-72 PRBs take 4 or 8, 73-144 take 8 or 16,
    // 145-275 take 16 or 32. The default 32 therefore aborts on any carrier narrower than
    // about 30 MHz at 15 kHz spacing, which is what the demo harness runs. Derive it from
    // the configured bandwidth unless the user asked for a specific value.
    // RBs of each carrier at 15 kHz, as NrPhy counts them (RbOverhead default 0.04)
    const uint32_t carrierRbs = static_cast<uint32_t>(bandwidthHz * 0.96 / (15e3 * 12));
    if (!subbandSizeSetByUser)
    {
        subbandSize = carrierRbs <= 72 ? 8 : (carrierRbs <= 144 ? 16 : 32);
    }

    if (payload != "transparent" && payload != "regenerative")
    {
        NS_ABORT_MSG("--payload must be 'transparent' or 'regenerative'.");
    }
    const bool transparent = payload == "transparent";
    const double satEirpDbm = satEIRP + 30.0 + 10.0 * std::log10(bandwidthHz / 1e6);

    if (ulFrequencyHz <= 0.0)
    {
        // n256's 190 MHz spacing, widened when the carrier itself is wider: FDD needs the two
        // carriers separated by at least one bandwidth, and anything above 190 MHz would
        // otherwise overlap and abort. ponytail: a real wide-carrier run should name its own
        // band pair with --ulFrequencyHz; this only keeps the default from failing.
        ulFrequencyHz = dlFrequencyHz - std::max(190e6, bandwidthHz);
    }
    const double ueNfDb = ueNoiseFigureDb > -900.0 ? ueNoiseFigureDb : receiverNfDb;
    const double gnbNfDb = gnbNoiseFigureDb > -900.0 ? gnbNoiseFigureDb : receiverNfDb;
    noiseFloorDbm = -174.0 + 10.0 * std::log10(bandwidthHz) + ueNfDb;

    if (transport != "tcp" && transport != "udp")
    {
        NS_ABORT_MSG("Unsupported --transport value. Use 'tcp' or 'udp'.");
    }

    if (duration <= 0.6)
    {
        NS_ABORT_MSG("Duration must exceed 0.6 seconds so applications stop after they start.");
    }
    if (observerMode != "overhead" && observerMode != "full-pass")
    {
        NS_ABORT_MSG("Unsupported --observerMode value. Use 'overhead' or 'full-pass'.");
    }
    const double observerBoundaryElevationDeg = observerElevationBoundaryDeg;
    if (observerBoundaryElevationDeg < 0.0 || observerBoundaryElevationDeg >= 90.0)
    {
        NS_ABORT_MSG("Elevation boundary must be in [0, 90) degrees.");
    }
    if (dlFrequencyHz <= 0.0 || ulFrequencyHz <= 0.0 || bandwidthHz <= 0.0)
    {
        NS_ABORT_MSG("--dlFrequencyHz, --ulFrequencyHz and --bandwidthHz must be positive.");
    }
    if (std::abs(dlFrequencyHz - ulFrequencyHz) < bandwidthHz)
    {
        NS_ABORT_MSG("The DL and UL carriers overlap: |dlFrequencyHz - ulFrequencyHz| must be at "
                     "least --bandwidthHz.");
    }
    if (rankLimit > std::numeric_limits<uint8_t>::max() ||
        subbandSize > std::numeric_limits<uint8_t>::max())
    {
        NS_ABORT_MSG("rankLimit and subbandSize must be <= 255.");
    }
    if (ueNumVerticalPorts == 0 || ueNumHorizontalPorts == 0 || gnbNumVerticalPorts == 0 ||
        gnbNumHorizontalPorts == 0)
    {
        NS_ABORT_MSG("Antenna port counts must be greater than zero.");
    }
    if ((ueAntennaRows % ueNumVerticalPorts) != 0 ||
        (ueAntennaColumns % ueNumHorizontalPorts) != 0 ||
        (gnbAntennaRows % gnbNumVerticalPorts) != 0 ||
        (gnbAntennaColumns % gnbNumHorizontalPorts) != 0)
    {
        NS_ABORT_MSG("Antenna port counts must divide the corresponding row/column counts.");
    }
    const uint32_t ueNumPorts =
        (ueIsDualPolarized ? 2 : 1) * ueNumVerticalPorts * ueNumHorizontalPorts;
    const uint32_t gnbNumPorts =
        (gnbIsDualPolarized ? 2 : 1) * gnbNumVerticalPorts * gnbNumHorizontalPorts;
    if (enableMimoFeedback && fullSearchCb == "ns3::NrCbTwoPort" && gnbNumPorts != 2)
    {
        NS_ABORT_MSG("ns3::NrCbTwoPort requires exactly 2 gNB antenna ports. Adjust "
                     "--gnbNumVerticalPorts/--gnbNumHorizontalPorts/--gnbIsDualPolarized or use "
                     "--fullSearchCb=ns3::NrCbTypeOneSp.");
    }
    if (enableMimoFeedback && fullSearchCb == "ns3::NrCbTypeOneSp" && gnbNumPorts > 2 &&
        !gnbIsDualPolarized)
    {
        NS_ABORT_MSG("ns3::NrCbTypeOneSp requires dual-polarized gNB antenna elements when using "
                     "more than 2 gNB antenna ports.");
    }
    if (enableMimoFeedback && rankLimit > std::min(ueNumPorts, gnbNumPorts))
    {
        NS_ABORT_MSG("rankLimit cannot exceed min(UE antenna ports, gNB antenna ports).");
    }

    // ns-3 defaults to a fixed seed and run 1, so repeating a config reproduces it bit-for-bit.
    // Vary the standard --RngRun across repetitions to get independent draws for error bars.
    std::cout << "RNG seed=" << RngSeedManager::GetSeed() << " run=" << RngSeedManager::GetRun()
              << std::endl;

    const uint32_t channelUpdatePeriodMs = channelUpdateMs > 0 ? channelUpdateMs : precision;
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(channelUpdatePeriodMs)));
    Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                       TimeValue(MilliSeconds(channelUpdatePeriodMs)));
    std::cout << "Orbit step=" << precision << " ms, channel regeneration=" << channelUpdatePeriodMs
              << " ms" << std::endl;
    std::cout << "FDD: DL BWP 0 at " << dlFrequencyHz / 1e6 << " MHz, UL BWP 1 at "
              << ulFrequencyHz / 1e6 << " MHz, " << bandwidthHz / 1e6
              << " MHz each, duplex spacing " << (dlFrequencyHz - ulFrequencyHz) / 1e6 << " MHz"
              << std::endl;

    // The orbit model of ns-3.48 computes the satellite position continuously; the resolution
    // only sets its CourseChange notification period. Frozen satellites are replaced below by
    // static copies, so their orbit models need no notifications at all.
    LeoOrbitNodeHelper orbit(freezeGeometry ? Seconds(0) : Time(MilliSeconds(precision)));
    NodeContainer satellites =
        orbitFile.empty()
            ? orbit.CreateNodesAndInstallMobility(LeoOrbitalShell(presetAltitudeKm, 30, 1, 1))
            : orbit.CreateNodesAndInstallMobility(orbitFile);
    NS_ABORT_MSG_IF(satellites.GetN() < 1, "The example requires at least one satellite.");

    // The first satellite anchors the placement, the timing sweep, the traces and the stop.
    Ptr<MobilityModel> satMobility = satellites.Get(0)->GetObject<MobilityModel>();
    if (!std::isnan(satLongitudeDeg) && !std::isnan(satArgOfLatitudeDeg))
    {
        // LeoCircularOrbitMobilityModel reads SetPosition as (plane longitude, argument of
        // latitude, index), so this restarts the satellite at a chosen point of its orbit.
        satMobility->SetPosition(Vector(satLongitudeDeg, satArgOfLatitudeDeg, 0));
    }
    // Predictor of the pass geometry, built from the live orbit model at t = 0.
    const SatellitePredictor satPredictor = MakeSatellitePredictor(satMobility, !freezeGeometry);
    const Vector satVelocityAtStart = satMobility->GetVelocity();
    if (freezeGeometry)
    {
        // Static ECEF copies at the initial positions. They still report the orbital velocity, so
        // the satellite's Doppler contribution remains; fading is therefore not purely UE-driven.
        NodeContainer frozenSatellites;
        for (uint32_t i = 0; i < satellites.GetN(); ++i)
        {
            Ptr<MobilityModel> live = satellites.Get(i)->GetObject<MobilityModel>();
            Ptr<Node> frozenNode = CreateObject<Node>();
            Ptr<EcefKinematicMobilityModel> frozen = CreateObject<EcefKinematicMobilityModel>();
            frozen->SetGeocentricPosition(live->GetPosition());
            frozen->SetConstantVelocity(live->GetVelocity(), false);
            frozenNode->AggregateObject(frozen);
            frozenSatellites.Add(frozenNode);
        }
        satellites = frozenSatellites;
        satMobility = satellites.Get(0)->GetObject<MobilityModel>();
        std::cout << "Geometry frozen: elevation, slant range and loop delay held constant"
                  << std::endl;
    }

    // UE 0: at the given place, or derived from the first satellite.
    Vector ue0Geo;
    if (!std::isnan(observerLatDeg) && !std::isnan(observerLonDeg))
    {
        ue0Geo = Vector(observerLatDeg, observerLonDeg, 0);
    }
    else if (observerMode == "full-pass")
    {
        ue0Geo = ComputeGroundStationPositionForInitialElevation(satPredictor.pos0,
                                                                 satVelocityAtStart,
                                                                 observerBoundaryElevationDeg,
                                                                 precision / 1000.0);
    }
    else
    {
        const Vector satGeo =
            GeographicPositions::CartesianToGeographicCoordinates(satPredictor.pos0,
                                                                  GeographicPositions::SPHERE);
        ue0Geo = Vector(satGeo.x, satGeo.y, 0);
    }
    Ptr<Node> ue = CreateObject<Node>();
    Ptr<EcefKinematicMobilityModel> ueMobility = CreateObject<EcefKinematicMobilityModel>();
    ueMobility->SetGeographicPosition(ue0Geo);
    ue->AggregateObject(ueMobility);
    g_ue0Geo = ueMobility->GetGeographicPosition();
    g_ue0Ecef = ueMobility->GetPosition();

    // The gateway of the transparent feeder link, UE 0's position unless given. ponytail: the
    // feeder then mirrors UE 0's service link; place the gateway to study them apart.
    // It carries no device, but the 3GPP loss models key their state on node IDs.
    Ptr<Node> gateway = CreateObject<Node>();
    Ptr<EcefKinematicMobilityModel> gatewayMobility = CreateObject<EcefKinematicMobilityModel>();
    gatewayMobility->SetGeographicPosition(!std::isnan(gatewayLatDeg) && !std::isnan(gatewayLonDeg)
                                               ? Vector(gatewayLatDeg, gatewayLonDeg, 0)
                                               : ue0Geo);
    gateway->AggregateObject(gatewayMobility);
    std::cout << "UE 0 at lat " << ue0Geo.x << " lon " << ue0Geo.y << ": first satellite at "
              << ElevationAngle(satMobility->GetPosition(), ue0Geo) << " deg elevation, "
              << SlantDistance(satMobility->GetPosition(), g_ue0Ecef) / 1000.0 << " km slant"
              << std::endl;
    if (transparent)
    {
        const Vector gatewayGeo = gatewayMobility->GetGeographicPosition();
        std::cout << "Gateway at lat " << gatewayGeo.x << " lon " << gatewayGeo.y
                  << ": first satellite at "
                  << ElevationAngle(satMobility->GetPosition(), gatewayGeo) << " deg elevation"
                  << std::endl;
    }

    // UE 0 keeps the legacy placement (beam center / calibration reference); additional
    // UEs are spread across the beam footprint by successive latitude offsets.
    NS_ABORT_MSG_IF(numUes == 0, "numUes must be at least 1.");
    NodeContainer ueNodes;
    ueNodes.Add(ue);
    std::vector<Ptr<MobilityModel>> ueMobilities;
    ueMobilities.push_back(ueMobility);
    // Per-UE radial distance from the beam centre (UE 0) for the spot-beam gain taper. With
    // footprintRadiusKm>0: 2 UEs near the centre + the rest at the footprint edge on alternating
    // sides, enabling a direct centre-vs-edge comparison on the service link.
    const double kmPerDegLat = 111.195;
    std::vector<double> explicitRadiiKm;
    if (!ueRadiiKmStr.empty())
    {
        std::stringstream ss(ueRadiiKmStr);
        std::string tok;
        while (std::getline(ss, tok, ','))
        {
            if (!tok.empty())
            {
                explicitRadiiKm.push_back(std::stod(tok));
            }
        }
    }
    // Optional per-UE bearing. Without it UEs are offset in latitude only, so several UEs at the
    // same radius land on top of each other. Placing them on a ring at a common radius gives
    // equal path loss and equal beam taper, isolating link adaptation from geometry spread.
    std::vector<double> ueAzimuthsDeg;
    if (!ueAzimuthsDegStr.empty())
    {
        std::stringstream ss(ueAzimuthsDegStr);
        std::string tok;
        while (std::getline(ss, tok, ','))
        {
            if (!tok.empty())
            {
                ueAzimuthsDeg.push_back(std::stod(tok));
            }
        }
    }
    const double ueBaseLatDeg = ueMobility->GetGeographicPosition().x;
    const double kmPerDegLon = kmPerDegLat * std::cos(ueBaseLatDeg * M_PI / 180.0);
    std::vector<double> ueRadialKm(1, explicitRadiiKm.empty() ? 0.0 : explicitRadiiKm[0]); // UE 0
    for (uint32_t i = 1; i < numUes; ++i)
    {
        Ptr<Node> extraUe = CreateObject<Node>();
        Ptr<EcefKinematicMobilityModel> extraMobility = CreateObject<EcefKinematicMobilityModel>();
        Vector geoPosition = ueMobility->GetGeographicPosition();
        double radialKm;
        double latOffsetDeg;
        if (i < explicitRadiiKm.size())
        {
            radialKm = explicitRadiiKm[i];
            latOffsetDeg = ((i % 2 == 0) ? 1.0 : -1.0) * radialKm / kmPerDegLat;
        }
        else if (footprintRadiusKm > 0.0)
        {
            radialKm = (i == 1) ? 2.0 : footprintRadiusKm;
            latOffsetDeg = ((i % 2 == 0) ? 1.0 : -1.0) * radialKm / kmPerDegLat;
        }
        else
        {
            latOffsetDeg = i * ueSpacingDeg;
            radialKm = std::abs(latOffsetDeg) * kmPerDegLat;
        }
        if (i - 1 < ueAzimuthsDeg.size())
        {
            // Bearing measured clockwise from north: 0 = +lat, 90 = +lon.
            const double azRad = ueAzimuthsDeg[i - 1] * M_PI / 180.0;
            geoPosition.x += radialKm * std::cos(azRad) / kmPerDegLat;
            geoPosition.y += radialKm * std::sin(azRad) / kmPerDegLon;
        }
        else
        {
            geoPosition.x += latOffsetDeg;
        }
        extraMobility->SetGeographicPosition(geoPosition);
        extraUe->AggregateObject(extraMobility);
        ueNodes.Add(extraUe);
        ueMobilities.push_back(extraMobility);
        ueRadialKm.push_back(radialKm);
    }

    if (ueSpeedMps > 0.0)
    {
        // Convert a local ENU heading into ECEF at the UE's own latitude/longitude, so the
        // velocity vector is tangent to the surface rather than pointing through the Earth.
        const double hdg = ueHeadingDeg * M_PI / 180.0;
        for (const auto& mm : ueMobilities)
        {
            auto m = DynamicCast<EcefKinematicMobilityModel>(mm);
            NS_ABORT_MSG_IF(!m, "UE mobility is not EcefKinematicMobilityModel");
            const Vector geo = m->GetGeographicPosition();
            const double la = geo.x * M_PI / 180.0;
            const double lo = geo.y * M_PI / 180.0;
            // ENU basis at (lat, lon): east and north unit vectors in ECEF
            const Vector east(-std::sin(lo), std::cos(lo), 0.0);
            const Vector north(-std::sin(la) * std::cos(lo),
                               -std::sin(la) * std::sin(lo),
                               std::cos(la));
            const double e = ueSpeedMps * std::sin(hdg);
            const double n = ueSpeedMps * std::cos(hdg);
            m->SetConstantVelocity(Vector(e * east.x + n * north.x,
                                          e * east.y + n * north.y,
                                          e * east.z + n * north.z),
                                   true);
        }
        std::cout << "UE ground speed " << ueSpeedMps << " m/s, heading " << ueHeadingDeg
                  << " deg -> Doppler " << ueSpeedMps / (SpeedOfLightMps / dlFrequencyHz)
                  << " Hz, coherence ~"
                  << 0.4 / (ueSpeedMps / (SpeedOfLightMps / dlFrequencyHz)) * 1000.0 << " ms"
                  << std::endl;
    }

    // One standard channel per FDD carrier, same delay, same link budget, own frequency. The gNBs
    // sit on the satellites; a transparent payload adds the feeder link from the gateway.
    const Time relayProcessingDelay = MicroSeconds(relayProcessingDelayUs);
    Ptr<NrNtnPropagationDelayModel> ntnDelay = CreateObject<NrNtnPropagationDelayModel>();
    ntnDelay->SetAttribute("RelayProcessingDelay", TimeValue(relayProcessingDelay));
    if (transparent)
    {
        ntnDelay->SetFeeder(gatewayMobility);
    }
    NtnCarrier dlCarrier =
        CreateNtnCarrier(dlFrequencyHz, scenario, gatewayMobility, ntnDelay, transparent);
    NtnCarrier ulCarrier =
        CreateNtnCarrier(ulFrequencyHz, scenario, gatewayMobility, ntnDelay, transparent);
    Ptr<ChannelConditionModel> channelConditionModel = dlCarrier.channelConditionModel;
    const bool mimoChannelEnabled = dlCarrier.mimoChannelEnabled;

    // The link budget of UE 0 through the first satellite at t = 0
    const double l1Initial = dlCarrier.feeder->GetFeederLossDb(satMobility);
    const double l2Initial = -dlCarrier.serviceLoss->CalcRxPower(0.0, satMobility, ueMobility);
    const double initialRelayInputSignalDbm =
        gatewayTxPowerDbm + satFeederRxGainDb - l1Initial - relayLossDb;
    const double initialRelayInputTotalDbm = SumDbm({initialRelayInputSignalDbm, noiseFloorDbm});
    const double relayGainDb = satEirpDbm - initialRelayInputTotalDbm;
    for (const auto& carrier : {dlCarrier, ulCarrier})
    {
        carrier.feeder->SetAttribute("GatewayTxPowerDbm", DoubleValue(gatewayTxPowerDbm));
        carrier.feeder->SetAttribute("SatRxAntennaGainDb", DoubleValue(satFeederRxGainDb));
        carrier.feeder->SetAttribute("RelayEirpDbm", DoubleValue(satEirpDbm));
        carrier.feeder->SetAttribute("RelayLossDb", DoubleValue(relayLossDb));
        carrier.feeder->SetAttribute("RelayInputNoisePowerDbm", DoubleValue(noiseFloorDbm));
        carrier.feeder->SetAttribute("ReceiverNoisePowerDbm", DoubleValue(noiseFloorDbm));
    }
    std::cout << "Payload: " << payload << ", satellite EIRP " << satEIRP
              << " dBW/MHz = " << satEirpDbm << " dBm, satellite element gain " << satAntennaGainDb
              << " dBi" << std::endl;
    if (!transparent)
    {
        std::cout << "Regenerative payload: the radio link is the UE<->satellite hop alone. The "
                     "feeder link carries backhaul only, so the AF relay figures printed below "
                     "do not apply."
                  << std::endl;
    }
    const double initialAfPathLossDb = DownlinkPathLossDb(dlCarrier.feeder,
                                                          l1Initial,
                                                          l2Initial,
                                                          transparent,
                                                          gatewayTxPowerDbm,
                                                          satEirpDbm,
                                                          noiseFloorDbm);

    // NTN timing, derived from the geometry the run will actually visit.
    const Time stopCheckPeriod = MilliSeconds(std::max<uint32_t>(precision, 1));
    std::vector<Vector> ueInitialPositions;
    std::vector<Vector> ueVelocities;
    for (const auto& mm : ueMobilities)
    {
        ueInitialPositions.push_back(mm->GetPosition());
        ueVelocities.push_back(mm->GetVelocity());
    }
    const LoopRttSweep sweep = SweepLoopRtt(satPredictor,
                                            gatewayMobility->GetPosition(),
                                            ueInitialPositions,
                                            ueVelocities,
                                            ue0Geo,
                                            transparent,
                                            relayProcessingDelay,
                                            Seconds(duration),
                                            stopCheckPeriod,
                                            observerBoundaryElevationDeg);
    const Time initialLoopRtt = 2 * ntnDelay->GetOneWayDelay(satMobility, ueMobility);
    // K_offset in 15 kHz reference slots (1 ms): the worst-case loop RTT rounded up. The
    // attribute range is 0-1023 with 0 meaning absent.
    const uint32_t kOffsetSlots = static_cast<uint32_t>(
        std::clamp<double>(std::ceil(sweep.maxLoopRtt.GetSeconds() * 1000.0), 1.0, 1023.0));
    Time connectionRequestTimeout = MilliSeconds(15);
    if (ntnTiming)
    {
        connectionRequestTimeout = std::max(MilliSeconds(15), sweep.maxLoopRtt + MilliSeconds(20));
    }
    if (idealRa)
    {
        connectionRequestTimeout = MilliSeconds(2000);
    }
    connectionRequestTimeout = std::min(connectionRequestTimeout, MilliSeconds(2000));

    std::cout << "=== NTN timing ===" << std::endl;
    std::cout << "Loop RTT at t=0 (" << (transparent ? "GT->sat->UE" : "service hop only")
              << "): " << initialLoopRtt.GetMicroSeconds() / 1000.0 << " ms" << std::endl;
    std::cout << "Sweep window: 0 - " << sweep.windowEnd.GetSeconds() << " s, min sat->UE0 "
              << "elevation " << sweep.minElevationDeg << " deg, max loop RTT "
              << sweep.maxLoopRtt.GetMicroSeconds() / 1000.0 << " ms at "
              << sweep.maxLoopRttAt.GetSeconds() << " s" << std::endl;
    std::cout << "K_offset (cell-specific, 1 ms reference slots): " << kOffsetSlots
              << (ntnTiming ? " (applied)" : " (not applied: --ntnTiming=0, attribute default 0)")
              << std::endl;
    std::cout << "ConnectionRequestTimeoutDuration: " << connectionRequestTimeout.GetMilliSeconds()
              << " ms"
              << (idealRa ? " (ideal RA ceiling)"
                          : (ntnTiming ? " (max(15 ms, maxLoopRtt + 20 ms))"
                                       : " (attribute default, --ntnTiming=0)"))
              << std::endl;
    std::cout << "Ideal random access: " << (idealRa ? 1 : 0) << std::endl;

    // Spectrum configuration: an FDD carrier pair, the pure-FDD recipe of nr-fdd-ul-routing.
    // Two operation bands of one CC and one BWP each, so that BWP 0 is the downlink carrier and
    // BWP 1 the uplink carrier whichever of the two is higher in frequency (n255 puts the DL
    // below the UL, n256 and n254 above it; CcBwpCreator lays the BWPs of a single band out
    // contiguously around one centre, which cannot express a duplex gap). Each BWP gets its own
    // bent-pipe channel, assigned the way the fork assigned its single one.
    CcBwpCreator ccBwpCreator;
    constexpr uint8_t numCcPerBand = 1;
    CcBwpCreator::SimpleOperationBandConf dlBandConf(dlFrequencyHz, bandwidthHz, numCcPerBand);
    CcBwpCreator::SimpleOperationBandConf ulBandConf(ulFrequencyHz, bandwidthHz, numCcPerBand);
    OperationBandInfo dlBand = ccBwpCreator.CreateOperationBandContiguousCc(dlBandConf);
    OperationBandInfo ulBand = ccBwpCreator.CreateOperationBandContiguousCc(ulBandConf);
    dlBand.GetBwpAt(0, 0)->SetChannel(dlCarrier.channel);
    ulBand.GetBwpAt(0, 0)->SetChannel(ulCarrier.channel);
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({dlBand, ulBand});

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(s1uLinkDelayMs)));

    if (ntnTiming)
    {
        nrHelper->SetGnbPhyAttribute("CellSpecificKoffset", UintegerValue(kOffsetSlots));
    }
    if (ntnTiming || idealRa)
    {
        Config::SetDefault("ns3::NrGnbRrc::ConnectionRequestTimeoutDuration",
                           TimeValue(connectionRequestTimeout));
    }
    nrHelper->SetUeMacAttribute("IdealRandomAccess", BooleanValue(idealRa));

    // Static BWP manager: the default QoS flow goes to the DL carrier; the gNB advertises the
    // DL/UL pairing to the UE, which routes its uplink to BWP 1.
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(0));

    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    idealBeamformingHelper->SetAttribute("BeamformingPeriodicity",
                                         TimeValue(MilliSeconds(beamUpdateMs)));

    if (enableMimoFeedback)
    {
        NrHelper::MimoPmiParams mimoPmiParams;
        mimoPmiParams.pmSearchMethod = pmSearchMethod;
        mimoPmiParams.fullSearchCb = fullSearchCb;
        mimoPmiParams.rankLimit = static_cast<uint8_t>(rankLimit);
        mimoPmiParams.rankThreshold = rankThreshold;
        mimoPmiParams.rankTechnique = rankTechnique;
        mimoPmiParams.subbandSize = static_cast<uint8_t>(subbandSize);
        mimoPmiParams.downsamplingTechnique = downsamplingTechnique;
        nrHelper->SetupMimoPmi(mimoPmiParams);
    }

    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(schedulerType));
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(enableHarqReTx));
    nrHelper->SetGnbMacAttribute("NumHarqProcess", UintegerValue(numHarqProcess));
    nrHelper->SetUeMacAttribute("NumHarqProcess", UintegerValue(numHarqProcess));
    nrHelper->SetGnbPhyAttribute("N1Delay", UintegerValue(n1Delay));
    nrHelper->SetGnbPhyAttribute("N2Delay", UintegerValue(n2Delay));
    // FixedMcsDl stops the CQI inner loop but not the outer one: OLLA keeps subtracting its
    // offset from the fixed index, so a fixed-MCS run silently transmits at fixedMcs minus the
    // offset (at the clamp, --fixedMcs=28 becomes a constant MCS 16). The two are alternatives,
    // so asking for a fixed MCS turns OLLA off.
    if (useFixedMcs && ollaStepUp > 0.0)
    {
        std::cout << "Fixed MCS requested: OLLA disabled (it would offset the fixed index)"
                  << std::endl;
        ollaStepUp = 0.0;
    }
    nrHelper->SetSchedulerAttribute("OllaStepUp", DoubleValue(ollaStepUp));
    nrHelper->SetSchedulerAttribute("OllaTargetBler", DoubleValue(ollaTargetBler));
    nrHelper->SetSchedulerAttribute("OllaMaxOffset", DoubleValue(ollaMaxOffset));
    nrHelper->SetSchedulerAttribute("DlHarqFeedbackDisabled", BooleanValue(dlHarqFeedbackDisabled));

    if (ollaStepUp > 0.0)
    {
        std::cout << "OLLA on: stepUp=" << ollaStepUp
                  << " stepDown=" << ollaStepUp * (1.0 - ollaTargetBler) / ollaTargetBler
                  << " targetBler=" << ollaTargetBler << std::endl;
    }
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(useFixedMcs));
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(useFixedMcs));
    if (useFixedMcs)
    {
        nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(fixedMcs));
        nrHelper->SetSchedulerAttribute("StartingMcsUl", UintegerValue(fixedMcs));
    }

    nrHelper->SetGnbAntennaTypeId("ns3::UniformPlanarArray");
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(gnbAntennaRows));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(gnbAntennaColumns));
    nrHelper->SetGnbAntennaAttribute("NumVerticalPorts", UintegerValue(gnbNumVerticalPorts));
    nrHelper->SetGnbAntennaAttribute("NumHorizontalPorts", UintegerValue(gnbNumHorizontalPorts));
    nrHelper->SetGnbAntennaAttribute("IsDualPolarized", BooleanValue(gnbIsDualPolarized));
    nrHelper->SetGnbAntennaAttribute("PolSlantAngle",
                                     DoubleValue(LocalDegreesToRadians(gnbPolSlantAngleDeg)));
    // The gNB transmits the satellite EIRP minus the element gain, which the element adds back
    // and which also closes the uplink.
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
                                         "Gain",
                                         DoubleValue(satAntennaGainDb))));
    nrHelper->SetUeAntennaTypeId("ns3::UniformPlanarArray");
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(ueAntennaRows));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(ueAntennaColumns));
    nrHelper->SetUeAntennaAttribute("NumVerticalPorts", UintegerValue(ueNumVerticalPorts));
    nrHelper->SetUeAntennaAttribute("NumHorizontalPorts", UintegerValue(ueNumHorizontalPorts));
    nrHelper->SetUeAntennaAttribute("IsDualPolarized", BooleanValue(ueIsDualPolarized));
    nrHelper->SetUeAntennaAttribute("PolSlantAngle",
                                    DoubleValue(LocalDegreesToRadians(uePolSlantAngleDeg)));
    if (ueAntennaElement == "3gpp")
    {
        nrHelper->SetUeAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObject<ThreeGppAntennaModel>()));
    }
    else if (ueAntennaElement == "isotropic")
    {
        Ptr<IsotropicAntennaModel> ueElem = CreateObject<IsotropicAntennaModel>();
        ueElem->SetAttribute("Gain", DoubleValue(ueAntennaElementGainDbi));
        nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(ueElem));
    }
    else
    {
        NS_ABORT_MSG("--ueAntennaElement must be 'isotropic' or '3gpp'");
    }

    Config::SetDefault("ns3::NrMacSchedulerHarqRr::ConsolidateHarqRetx",
                       BooleanValue(consolidateHarqRetx));
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(10 * 1024 * 1024));
    Config::SetDefault("ns3::NrRlcUm::ReorderingTimer",
                       TimeValue(MilliSeconds(rlcReorderingTimerMs)));

    if (rlcMode == "am")
    {
        // nr-helper only overrides RLC_SM_ALWAYS, so an explicit AM selection survives.
        Config::SetDefault("ns3::NrGnbRrc::QosFlowToRlcMapping",
                           EnumValue(NrGnbRrc::RLC_AM_ALWAYS));
        // Defaults are 20/10/10 ms, all shorter than an NTN loop, so AM would declare loss on
        // blocks still in flight and retransmit them. Sized from the loop RTT instead:
        // t-Reordering must outlast the worst-case HARQ recovery (4 rv rounds), t-PollRetransmit
        // must outlast one round trip plus the status report's own scheduling delay.
        // NrRlcAm defaults to a 10 kB transmit buffer, 1000x smaller than the UM setting above.
        // Under full-buffer traffic that discards nearly everything before it reaches the air.
        Config::SetDefault("ns3::NrRlcAm::MaxTxBufferSize", UintegerValue(10 * 1024 * 1024));
        // A retransmission too big for its grant is re-segmented (TS 36.322 5.2.1). True sends it
        // whole instead, overflowing the grant: the MAC asserts on it.
        Config::SetDefault("ns3::NrRlcAm::TxOpportunityForRetxAlwaysBigEnough",
                           BooleanValue(rlcAmRetxAlwaysFits));
        Config::SetDefault("ns3::NrRlcAm::PollRetransmitTimer",
                           TimeValue(MilliSeconds(rlcAmPollRetxMs)));
        Config::SetDefault("ns3::NrRlcAm::ReorderingTimer",
                           TimeValue(MilliSeconds(rlcAmReorderingMs)));
        Config::SetDefault("ns3::NrRlcAm::StatusProhibitTimer",
                           TimeValue(MilliSeconds(rlcAmStatusProhibitMs)));
        std::cout << "RLC AM: pollRetx=" << rlcAmPollRetxMs << "ms reordering=" << rlcAmReorderingMs
                  << "ms statusProhibit=" << rlcAmStatusProhibitMs << "ms" << std::endl;
    }
    else if (rlcMode != "um")
    {
        NS_ABORT_MSG("--rlcMode must be 'um' or 'am'");
    }

    // One gNB per satellite
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(satellites, allBwps);
    if (lbrm)
    {
        // TBS_LBRM is the largest block the carrier supports, from LENA's own TBS procedure (a
        // fresh NrAmc takes this run's NrAmc defaults), over the N_RE = 156 per PRB (12
        // subcarriers x 13 symbols) TS 38.212 assumes: CalculateTbSize counts RB-symbols.
        // Ncb summed over the code blocks is
        // C * floor(TBS_LBRM / (C * R_LBRM)) ~ TBS_LBRM / R_LBRM with R_LBRM = 2/3, whatever C is.
        // The error models are created at the first reception, so the default still applies.
        Ptr<NrAmc> amc = CreateObject<NrAmc>();
        const uint32_t tbsLbrmBits =
            8 * amc->CalculateTbSize(static_cast<uint8_t>(amc->GetMaxMcs()),
                                     static_cast<uint8_t>(rankLimit),
                                     carrierRbs * 13);
        NS_ABORT_MSG_IF(tbsLbrmBits == 0, "--lbrm: TBS_LBRM came out 0 bits");
        const uint32_t softBufferBits = 3 * tbsLbrmBits / 2;
        Config::SetDefault("ns3::NrEesmIr::SoftBufferBitsPerTb", UintegerValue(softBufferBits));
        std::cout << "LBRM soft buffer: TBS_LBRM " << tbsLbrmBits << " bits (MCS "
                  << amc->GetMaxMcs() << ", rank " << rankLimit << ", " << carrierRbs << " RBs) => "
                  << softBufferBits << " bits per transport block" << std::endl;
    }
    // FDD slot patterns: BWP 0 carries only downlink slots, BWP 1 only uplink slots. The gNB
    // pairs the UL-only carrier with the DL carrier when the cell is configured at attach.
    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {
        NrHelper::GetGnbPhy(gnbNetDev.Get(i), 0)
            ->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
        NrHelper::GetGnbPhy(gnbNetDev.Get(i), 1)
            ->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));
    }
    // Install UEs individually so each carries its own spot-beam footprint gain taper:
    // Delta = -min(3*(r/R)^2, 20) dB on the UE antenna element gain, 0 dB at the beam centre,
    // -3 dB at the footprint edge, which lowers the edge UE's service-link SINR both ways.
    NetDeviceContainer ueNetDev;
    Vector sat2EcefForReport = satMobility->GetPosition();
    Vector beamCentreEcef = ueMobilities[0]->GetPosition();
    std::vector<Ptr<IsotropicAntennaModel>> ueBeamElements;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        double taperDb = 0.0;
        if (beamHpbwDeg > 0.0)
        {
            taperDb = SpotBeamTaperDb(sat2EcefForReport,
                                      beamCentreEcef,
                                      ueMobilities[i]->GetPosition(),
                                      beamHpbwDeg,
                                      beamMaxAttenDb);
        }
        else if (footprintRadiusKm > 0.0)
        {
            const double rr = ueRadialKm[i] / footprintRadiusKm;
            taperDb = -std::min(3.0 * rr * rr, 20.0);
        }
        // The terminal's own boresight gain sits on this element, on top of the spot-beam
        // taper. A VSAT dish is one high-gain aperture, so putting the whole 34.6 dBi here
        // gives the right gain in both directions with no array pattern -- large element
        // arrays were measured to break link adaptation badly (70% BLER at 20 dB SINR).
        Ptr<IsotropicAntennaModel> ueElement = CreateObjectWithAttributes<IsotropicAntennaModel>(
            "Gain",
            DoubleValue(ueAntennaElementGainDbi + taperDb));
        ueBeamElements.push_back(ueElement);
        nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(ueElement));
        ueNetDev.Add(nrHelper->InstallUeDevice(NodeContainer(ueNodes.Get(i)), allBwps));
        double slantKm = (sat2EcefForReport - ueMobilities[i]->GetPosition()).GetLength() / 1000.0;
        std::cout << "UE " << i << " footprint: radial=" << ueRadialKm[i]
                  << " km, gainTaper=" << taperDb << " dB, slant=" << slantKm << " km" << std::endl;
    }

    const uint32_t numBwps = allBwps.size();
    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {
        for (uint32_t bwp = 0; bwp < numBwps; ++bwp)
        {
            nrHelper->GetGnbPhy(gnbNetDev.Get(i), bwp)->SetTxPower(satEirpDbm - satAntennaGainDb);
        }
    }
    for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
    {
        for (uint32_t bwp = 0; bwp < numBwps; ++bwp)
        {
            nrHelper->GetUePhy(ueNetDev.Get(i), bwp)->SetTxPower(ueTxPowerDbm);
        }
    }

    // ponytail: the channel has one receiver noise level, the UE's, so when the gNB figure
    // differs the uplink PHY SINR is off from the channel's by that difference. Give the channel
    // per-receiver noise if uplink budgets start to matter.
    for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
    {
        for (uint32_t bwp = 0; bwp < numBwps; ++bwp)
        {
            nrHelper->GetUePhy(ueNetDev.Get(i), bwp)
                ->SetAttribute("NoiseFigure", DoubleValue(ueNfDb));
        }
    }
    std::cout << "UE noise figure " << ueNfDb << "dB (Tsys "
              << 290.0 * std::pow(10.0, ueNfDb / 10.0) << "K)" << std::endl;
    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {
        for (uint32_t bwp = 0; bwp < numBwps; ++bwp)
        {
            nrHelper->GetGnbPhy(gnbNetDev.Get(i), bwp)
                ->SetAttribute("NoiseFigure", DoubleValue(gnbNfDb));
        }
    }
    std::cout << "gNB noise figure " << gnbNfDb << "dB (Tsys "
              << 290.0 * std::pow(10.0, gnbNfDb / 10.0) << "K)" << std::endl;

    if (perfectUplink)
    {
        // Every UL transport block decodes. RLC AM carries its STATUS PDUs on the uplink, and
        // with a weak uplink budget none decode: VT(A) never advances and the bearer delivers
        // exactly one transmit window, silently. Disabling the error model at the gNB isolates
        // the downlink, whose error model is evaluated at the UE.
        for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
        {
            for (uint32_t bwp = 0; bwp < numBwps; ++bwp)
            {
                nrHelper->GetGnbPhy(gnbNetDev.Get(i), bwp)
                    ->GetSpectrumPhy()
                    ->SetAttribute("DataErrorModelEnabled", BooleanValue(false));
            }
        }
        std::cout << "perfectUplink: gNB data error model disabled, all UL TBs decode" << std::endl;
    }

    // OLLA offset trajectory. Needed to tell convergence from clamping: a loop pinned at the
    // clamp reports the same steady-state BLER as a converged one, but for a different reason.
    std::ofstream ollaTraceFile;
    if (ollaStepUp > 0.0)
    {
        ollaTraceFile.open("OllaOffsetTrace.txt");
        ollaTraceFile << "Time\trnti\toffsetMcs\tack\tclamped\n";
        for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
        {
            NrHelper::GetScheduler(gnbNetDev.Get(i), 0)
                ->TraceConnectWithoutContext("DlOllaUpdate",
                                             MakeBoundCallback(
                                                 +[](std::ofstream* out,
                                                     uint16_t rnti,
                                                     double offset,
                                                     bool ack,
                                                     bool clamped) {
                                                     *out << Simulator::Now().GetSeconds() << '\t'
                                                          << rnti << '\t' << offset << '\t'
                                                          << (ack ? 1 : 0) << '\t'
                                                          << (clamped ? 1 : 0) << '\n';
                                                 },
                                                 &ollaTraceFile));
        }
    }

    LogPhasedArrayPorts("gNB", nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->GetSpectrumPhy());
    LogPhasedArrayPorts("UE", nrHelper->GetUePhy(ueNetDev.Get(0), 0)->GetSpectrumPhy());

    auto [remoteHost, remoteHostIpv4Address] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, MilliSeconds(remoteHostDelayMs));

    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpInterface =
        nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    const uint16_t dlPortBase = 1234;
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        const uint16_t dlPort = dlPortBase + i;
        Ptr<Node> ueNode = ueNodes.Get(i);
        if (transport == "tcp")
        {
            PacketSinkHelper sinkHelper("ns3::TcpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            serverApps.Add(sinkHelper.Install(ueNode));

            BulkSendHelper clientHelper("ns3::TcpSocketFactory",
                                        InetSocketAddress(ueIpInterface.GetAddress(i), dlPort));
            clientHelper.SetAttribute("MaxBytes", UintegerValue(0));
            clientHelper.SetAttribute("SendSize", UintegerValue(udpPacketSize));
            clientApps.Add(clientHelper.Install(remoteHost));
        }
        else
        {
            PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            serverApps.Add(sinkHelper.Install(ueNode));

            if (udpRateMbps <= 0.0)
            {
                Ptr<BackloggedUdpSourceApplication> backlogged =
                    CreateObject<BackloggedUdpSourceApplication>();
                backlogged->Setup(InetSocketAddress(ueIpInterface.GetAddress(i), dlPort),
                                  udpPacketSize);
                remoteHost->AddApplication(backlogged);
                clientApps.Add(backlogged);
                continue;
            }
            // udpRateMbps is the per-UE offered load; aggregate load is numUes x rate
            UdpClientHelper clientHelper(ueIpInterface.GetAddress(i), dlPort);
            clientHelper.SetAttribute("PacketSize", UintegerValue(udpPacketSize));
            clientHelper.SetAttribute("MaxPackets",
                                      UintegerValue(std::numeric_limits<uint32_t>::max()));
            const double packetsPerSecond =
                (udpRateMbps * 1e6) / (8.0 * static_cast<double>(udpPacketSize));
            clientHelper.SetAttribute("Interval",
                                      TimeValue(Seconds(1.0 / std::max(1.0, packetsPerSecond))));
            clientApps.Add(clientHelper.Install(remoteHost));
        }
    }

    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);

    // Ideal timing advance: each UE advances its uplink by its one-way delay to the gNB it is
    // attached to, feeder link included when transparent, re-read before every slot.
    std::vector<Time> initialTimingAdvance;
    for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
    {
        auto ueDev = DynamicCast<NrUeNetDevice>(ueNetDev.Get(i));
        initialTimingAdvance.push_back(
            ntnDelay->GetOneWayDelay(ServingGnbMobility(ueDev, ueMobilities[i], gnbNetDev),
                                     ueMobilities[i]));
        for (uint32_t bwp = 0; ntnTiming && bwp < ueDev->GetCcMapSize(); ++bwp)
        {
            // A lambda, since MakeBoundCallback needs comparable arguments and a
            // NetDeviceContainer is not
            NrHelper::GetUePhy(ueDev, bwp)
                ->SetTimingAdvanceCallback(
                    Callback<Time>([ntnDelay, ueDev, ueMobility = ueMobilities[i], gnbNetDev]() {
                        return ntnDelay->GetOneWayDelay(
                            ServingGnbMobility(ueDev, ueMobility, gnbNetDev),
                            ueMobility);
                    }));
        }
    }
    for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
    {
        std::cout << "UE " << i << " timing advance (one-way delay at t=0): "
                  << initialTimingAdvance[i].GetMicroSeconds() / 1000.0 << " ms"
                  << (ntnTiming ? (freezeGeometry ? " (applied, geometry frozen)"
                                                  : " (applied, refreshed every slot)")
                                : " (not applied: --ntnTiming=0)")
                  << std::endl;
    }

    // Write the derived timing values next to the other traces.
    {
        const std::string timingPath = "NtnTimingConfig.txt";
        std::ofstream timingOut(timingPath);
        if (timingOut.is_open())
        {
            timingOut << "ntnTiming\t" << (ntnTiming ? 1 : 0) << "\n";
            timingOut << "idealRa\t" << (idealRa ? 1 : 0) << "\n";
            timingOut << "architecture\t" << payload << "\n";
            timingOut << "freezeGeometry\t" << (freezeGeometry ? 1 : 0) << "\n";
            timingOut << "initialLoopRttMs\t" << initialLoopRtt.GetMicroSeconds() / 1000.0 << "\n";
            timingOut << "sweepWindowEndS\t" << sweep.windowEnd.GetSeconds() << "\n";
            timingOut << "minElevationDeg\t" << sweep.minElevationDeg << "\n";
            timingOut << "maxLoopRttMs\t" << sweep.maxLoopRtt.GetMicroSeconds() / 1000.0 << "\n";
            timingOut << "maxLoopRttAtS\t" << sweep.maxLoopRttAt.GetSeconds() << "\n";
            timingOut << "kOffsetSlots\t" << (ntnTiming ? kOffsetSlots : 0) << "\n";
            timingOut << "connectionRequestTimeoutMs\t"
                      << ((ntnTiming || idealRa) ? connectionRequestTimeout.GetMilliSeconds() : 15)
                      << "\n";
            for (uint32_t i = 0; i < initialTimingAdvance.size(); ++i)
            {
                timingOut << "ue\t" << i << "\ttimingAdvanceUs\t"
                          << (ntnTiming ? initialTimingAdvance[i].GetMicroSeconds() : 0) << "\n";
            }
            std::cout << "NTN timing configuration written to " << timingPath << std::endl;
        }
    }

    // Every BWP array of every UE, whose cached channel must be dropped when the satellite
    // re-points or a UE's spot-beam gain changes
    std::vector<std::vector<Ptr<PhasedArrayModel>>> ueArrays;
    std::vector<Ptr<PhasedArrayModel>> allUeArrays;
    for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
    {
        auto ueDevice = DynamicCast<NrUeNetDevice>(ueNetDev.Get(i));
        std::vector<Ptr<PhasedArrayModel>> arrays;
        for (uint32_t bwp = 0; bwp < ueDevice->GetCcMapSize(); ++bwp)
        {
            arrays.push_back(DynamicCast<PhasedArrayModel>(
                ueDevice->GetPhy(bwp)->GetSpectrumPhy()->GetAntenna()));
            allUeArrays.push_back(arrays.back());
        }
        ueArrays.push_back(arrays);
    }

    // Every satellite points its arrays at the beam centre (UE 0) every --precision; the DL array
    // rides on the node for the --traceFile boresight column.
    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {
        auto gnbDevice = DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(i));
        std::vector<Ptr<UniformPlanarArray>> satAntennas;
        for (uint32_t bwp = 0; bwp < gnbDevice->GetCcMapSize(); ++bwp)
        {
            satAntennas.push_back(gnbDevice->GetPhy(bwp)
                                      ->GetSpectrumPhy()
                                      ->GetAntenna()
                                      ->GetObject<UniformPlanarArray>());
        }
        Ptr<Node> satNode = gnbDevice->GetNode();
        satNode->AggregateObject(satAntennas.front());
        UpdateAntennaOrientation(satNode,
                                 satAntennas,
                                 allUeArrays,
                                 g_ue0Ecef,
                                 MilliSeconds(precision));
    }

    // Geometry-driven spot-beam taper of the first satellite's beam: each UE's effective gain
    // follows the 3GPP/ITU beam pattern at its live off-boresight angle, so the footprint distorts
    // with elevation over the pass.
    if (beamHpbwDeg > 0.0)
    {
        UpdateSpotBeamTaper(satMobility,
                            ueMobilities[0],
                            ueMobilities,
                            ueBeamElements,
                            ueArrays,
                            ueAntennaElementGainDbi,
                            beamHpbwDeg,
                            beamMaxAttenDb,
                            MilliSeconds(beamUpdateMs));
    }

    serverApps.Start(MilliSeconds(400));
    clientApps.Start(MilliSeconds(400));
    serverApps.Stop(Seconds(duration));
    clientApps.Stop(Seconds(duration) - MilliSeconds(200));

    nrHelper->EnableTraces();

    std::ofstream geometryOut;
    if (!traceFile.empty())
    {
        geometryOut.open(traceFile);
        if (geometryOut.is_open())
        {
            g_geometryTrace = &geometryOut;
            geometryOut << "Time:Satellite:x:y:z:x_2:y_2:z_2:elevation_deg:slant_distance_m"
                        << std::endl;
            Config::ConnectWithoutContextFailSafe("/NodeList/*/$ns3::MobilityModel/CourseChange",
                                                  MakeCallback(&CourseChange));
        }
    }

    std::ofstream out;
    if (!linkBudgetTraceFile.empty())
    {
        out.open(linkBudgetTraceFile);
        if (out.is_open())
        {
            out << "Time\tGT_sat1_distance_m\tISL_distance_m\tsat2_UE_distance_m\t"
                << "GT_sat1_elevation_deg\tsat2_UE_elevation_deg\tDL_pathloss_db\t"
                << "UL_pathloss_db\tGT_sat1_loss_db\tGT_sat1_loss_delta_db\t"
                << "sat2_UE_loss_db\tsat2_UE_loss_delta_db\taccess_loss_delta_sum_db\t"
                << "radio_one_way_delay_ms\tSNR_sat1_db\tSNR_sat2_db\tgamma_2_db\t"
                << "gamma_UE_cascade_db\tSNR_UE_db" << std::endl;
            LogLinkBudget(gatewayMobility,
                          satMobility,
                          ueMobility,
                          dlCarrier,
                          ulCarrier,
                          transparent,
                          satAntennaGainDb,
                          gatewayMobility->GetGeographicPosition(),
                          ue0Geo,
                          stopCheckPeriod,
                          &out,
                          gatewayTxPowerDbm,
                          satFeederRxGainDb,
                          satEirpDbm,
                          relayLossDb,
                          noiseFloorDbm,
                          l1Initial,
                          l2Initial);
        }
    }

    std::ofstream channelConditionOut;
    if (!channelConditionTraceFile.empty())
    {
        channelConditionOut.open(channelConditionTraceFile);
        if (channelConditionOut.is_open())
        {
            channelConditionOut << "Time\tis_los\tcondition\tgt_sat1_is_los\tgt_sat1_condition\t"
                                << "sat2_ue_is_los\tsat2_ue_condition" << std::endl;
            LogChannelCondition(channelConditionModel,
                                satMobility,
                                ueMobility,
                                stopCheckPeriod,
                                &channelConditionOut);
        }
    }
    std::cout << "Initial GT-sat path loss (DL carrier): " << l1Initial << "dB" << std::endl;
    std::cout << "Initial sat-UE path loss (DL carrier): " << l2Initial << "dB" << std::endl;
    std::cout << "Initial GT-sat path loss (UL carrier): "
              << ulCarrier.feeder->GetFeederLossDb(satMobility) << "dB" << std::endl;
    std::cout << "Initial sat-UE path loss (UL carrier): "
              << -ulCarrier.serviceLoss->CalcRxPower(0.0, satMobility, ueMobility) << "dB"
              << std::endl;
    std::cout << "sat->UE MIMO channel model: " << (mimoChannelEnabled ? "enabled" : "disabled")
              << ", MIMO feedback=" << enableMimoFeedback << ", rankLimit=" << rankLimit
              << std::endl;
    if (transparent)
    {
        std::cout << "Feeder link: gateway " << gatewayTxPowerDbm << " dBm, satellite receive gain "
                  << satFeederRxGainDb << " dBi, relay loss " << relayLossDb
                  << " dB => initial AF relay gain " << relayGainDb << " dB" << std::endl;
        std::cout << "AF relay gain is recomputed from the instantaneous feeder loss and relay "
                  << "input noise during channel delivery." << std::endl;
        std::cout << "Link-budget trace gamma_UE_cascade_db uses gamma1*gamma2/(gamma1+gamma2+1), "
                  << "with gamma1=SNR_sat2_db after relay loss and gamma2=satellite downlink SNR."
                  << std::endl;
        std::cout << "Assuming receiver noise floor of " << noiseFloorDbm << "dBm, the expected "
                  << "SNR at the satellite is: "
                  << gatewayTxPowerDbm + satFeederRxGainDb - l1Initial - relayLossDb - noiseFloorDbm
                  << "dB" << std::endl;
    }
    std::cout << "Initial AF-equivalent downlink path loss: " << initialAfPathLossDb << "dB"
              << std::endl;
    std::cout << "Assuming downlink path loss as calculated above and a " << ueAntennaElementGainDbi
              << " dBi UE antenna, the expected SNR at UE is: "
              << gatewayTxPowerDbm - initialAfPathLossDb + ueAntennaElementGainDbi - noiseFloorDbm
              << "dB" << std::endl;
    std::cout << "NR PHY path: " << (transparent ? "gateway -> " : "") << gnbNetDev.GetN()
              << " satellite gNB(s), first on node " << satellites.Get(0)->GetId() << " -> UE node "
              << ue->GetId() << std::endl;
    std::cout << "Service-link channel scenario: " << scenario << std::endl;

    // The run ends when the first satellite, having risen above the boundary, descends through it.
    StopAtElevationBoundary(satMobility,
                            ue0Geo,
                            observerBoundaryElevationDeg,
                            stopCheckPeriod,
                            false);

    Simulator::Stop(Seconds(duration));
    Simulator::Run();

    out.close();
    geometryOut.close();
    channelConditionOut.close();

    uint64_t receivedBytes = 0;
    for (uint32_t i = 0; i < serverApps.GetN(); ++i)
    {
        Ptr<PacketSink> serverPacketSink = serverApps.Get(i)->GetObject<PacketSink>();
        const uint64_t ueBytes = serverPacketSink->GetTotalRx();
        receivedBytes += ueBytes;
        std::cout << "UE " << i << " received " << ueBytes << " bytes" << std::endl;
    }
    std::cout << "Received " << receivedBytes << " bytes (all UEs)" << std::endl;

    Simulator::Destroy();
    return receivedBytes > 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
