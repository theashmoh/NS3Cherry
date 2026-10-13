// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
// Author: Thiago Miyazaki <miyathiago@gmail.com> <t.miyazaki@unesp.br>

/**
 * @file leo-link-regen-RT.cc
 *
 * The ray-traced twin of leo-link --payload=regenerative. Identical orbit, FDD carriers,
 * K_offset, timing advance and traces; the only difference is the radio channel, which comes from
 * Sionna RT over real building geometry instead of the 3GPP statistical model. Run the two side by
 * side to see what the buildings do: the statistical model degrades smoothly with elevation, this
 * one cuts out when a rooftop is in the way and acquires when the satellite clears it.
 *
 * Sionna RT has no far-field source, and tracing a scene a kilometre across from 900 km away
 * finds the direct path but almost no reflections. So the satellite is shown to the ray tracer as
 * a proxy transmitter a few km along the true line of sight, which keeps the arrival angle, the
 * blockage and the multipath; the delay then comes from the true slant range and the path loss is
 * rescaled by the free-space ratio. Details and the measured validation are in rt.md.
 * @brief Regenerative LEO NTN link: the gNB rides on the satellite, the UEs sit on the ground.
 *
 * This is the port of the v4.1 fork's `leo-link.cc` to ns-3.48 and 5G-LENA v5.1. The satellite
 * is moved by the mainline `LeoCircularOrbitMobilityModel` (ns-3.48 `src/mobility`), the radio
 * goes through the stock NrHelper and the 3GPP NTN channel of `--scenario`, and every bandwidth
 * part carries a `ConstantSpeedPropagationDelayModel`, so control and data cross the service
 * link with the physical one-way delay in both directions. This is the regenerative timing
 * baseline: the HARQ loop is sat -> UE -> sat, one hop each way.
 *
 * Duplexing. The cell is FDD, as every 3GPP NR NTN band is (Rel-17 n255 L-band, UL
 * 1626.5-1660.5 / DL 1525-1559 MHz; n256 S-band, UL 1980-2010 / DL 2170-2200 MHz; Rel-18 n254,
 * UL L-band 1610-1626.5 / DL S-band 2483.5-2500 MHz; the Ka-band VSAT bands n510-n512; TDD
 * exists only for NB-IoT NTN). Two bandwidth parts are created, BWP 0 downlink-only at
 * `--dlFrequencyHz` with the slot pattern `DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|` and BWP 1
 * uplink-only at `--ulFrequencyHz` with `UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|`, each on its own 3GPP
 * NTN channel instance with its own propagation delay model. The static BWP manager pins the
 * default QoS flow to BWP 0 and the gNB advertises the DL/UL carrier pairing in dedicated RRC
 * configuration, so the UE routes its uplink to BWP 1 (the recipe of `nr-fdd-ul-routing`). Ideal
 * RRC only: the real RRC protocol does not carry the pairing. The defaults keep the fork's
 * 3.5 GHz / 100 MHz downlink and put the uplink 190 MHz below it (n256-like duplex spacing).
 *
 * NTN timing (`--ntnTiming`, default on) sets three things from the geometry, never from a
 * numeric flag a user could set inconsistently with the orbit:
 *   - `NrGnbPhy::CellSpecificKoffset` = ceil(maxLoopRtt / 1 ms) reference slots, where
 *     maxLoopRtt = 2 * slantRange / c at the lowest elevation the simulated window actually
 *     visits (the pass is swept ahead of time up to `--duration` or the elevation boundary,
 *     whichever comes first). K_offset is cell-specific and static (TS 38.213 cl. 4.2), so it
 *     is sized once for the worst case of the window, not updated as the satellite moves.
 *   - `NrUePhy::TimingAdvance` per UE = slantRange / c (ideal timing advance: the UE knows its
 *     one-way delay exactly, standing in for GNSS + ephemeris in Rel-17). The UE re-reads it
 *     from the geometry before every slot.
 *   - `NrGnbRrc::ConnectionRequestTimeoutDuration` = max(15 ms, maxLoopRtt + 20 ms). The gNB
 *     arms this timer when it creates the UE context on preamble reception, before the RAR is
 *     even sent, and it runs under ideal RRC too; at the default 15 ms the context dies before a
 *     one-way RAR lands.
 * With `--ntnTiming=0` none of the three attributes is touched (they stay at their defaults).
 * What it fixes: with the ideal timing advance the UE's HARQ feedback lands at K1 + K_offset in
 * the gNB's timeline and the scheduler's H + K_offset process expiry is the TS 38.213 design.
 * What it does not fix: the random-access response window still needs RTT + L0 (the gNB's own
 * RAR latency) inside it; `--idealRa` (default on) sets `NrUeMac::IdealRandomAccess` so the RA
 * response timeout is never armed, and then also pins the connection-request timer to its
 * 2000 ms ceiling whatever `--ntnTiming` says. Msg1 and Msg2 still cross the channel with real
 * delay, so attach costs one RTT + L0 and nothing else. The UE-specific K_offset MAC CE that
 * would trim the worst-case value over a pass is not modelled.
 * Every derived value (loop RTT, minimum elevation, K_offset, TA per UE, the RRC timer) is
 * printed at startup and written to `NtnTimingConfig.txt` in the working directory.
 *
 * `--n1Delay` / `--n2Delay` default to 2 slots; v5.1 accepts 0-4.
 */

#include "ns3/application.h"
#include "ns3/bulk-send-application.h"
#include "ns3/bulk-send-helper.h"
#include "ns3/channel-condition-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/geocentric-ecef-mobility-model.h"
#include "ns3/geographic-positions.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/leo-circular-orbit-mobility-model.h"
#include "ns3/leo-orbit-node-helper.h"
#include "ns3/leo-orbital-shell.h"
#include "ns3/matrix-based-channel-model.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-rx-trace.h"
#include "ns3/nr-module.h"
#include "ns3/nr-phy-rx-trace.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/phased-array-spectrum-propagation-loss-model.h"

#include <pybind11/embed.h>

namespace py = pybind11;
#include "ns3/propagation-delay-model.h"
#include "ns3/socket.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/udp-socket-factory.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LeoNrRtExample");

constexpr double SpeedOfLightMps = 299792458.0;

Vector groundNodePosGEO;
Vector groundNodePosECEF;

/**
 * Compute the slant distance (line-of-sight distance) between ground station and satellite
 * @param satellitePositionECEF Satellite position in ECEF coordinates
 * @return Slant distance in meters
 */
double
ComputeSlantDistance(const Vector& satellitePositionECEF)
{
    // Direct distance calculation in ECEF coordinates
    Vector slantVector = satellitePositionECEF - groundNodePosECEF;
    return slantVector.GetLength();
}

/**
 * Compute the elevation angle from ground station to satellite
 * @param satellitePositionECEF Satellite position in ECEF coordinates
 * @return Elevation angle in degrees (0° = horizon, 90° = zenith)
 */
double
ComputeElevationAngle(const Vector& satellitePositionECEF)
{
    // Convert satellite position from ECEF to Geographic coordinates
    const Vector satellitePositionGEO =
        GeographicPositions::CartesianToGeographicCoordinates(satellitePositionECEF,
                                                              GeographicPositions::SPHERE);

    // Convert to local ENU (East-North-Up) coordinate system at ground station
    const Vector enuVector =
        GeographicPositions::GeographicToTopocentricCoordinates(satellitePositionGEO,
                                                                groundNodePosGEO,
                                                                GeographicPositions::SPHERE);

    // Calculate elevation angle using the Up component and total distance
    double slantDistance = enuVector.GetLength();
    if (slantDistance == 0.0)
    {
        return 90.0; // Satellite directly overhead (theoretical case)
    }

    // Elevation angle = arcsin(up_component / slant_distance)
    double elevationRad = std::asin(enuVector.z / slantDistance);

    // Convert from radians to degrees
    double elevationDeg = elevationRad * 180.0 / M_PI;

    return elevationDeg;
}

/**
 * Check if a satellite link is viable based on minimum elevation angle
 * @param satellitePositionECEF Satellite position in ECEF coordinates
 * @param minElevationDeg Minimum elevation angle threshold in degrees (typical: 5-10°)
 * @return True if link is viable (elevation >= threshold), false otherwise
 */
bool
IsLinkViable(const Vector& satellitePositionECEF, double minElevationDeg = 5.0)
{
    double elevation = ComputeElevationAngle(satellitePositionECEF);
    return elevation >= minElevationDeg;
}

/**
 * Compute basic free space path loss for the satellite link
 * @param satellitePositionECEF Satellite position in ECEF coordinates
 * @param frequencyHz Operating frequency in Hz
 * @return Path loss in dB
 */
double
ComputeBasicPathLoss(const Vector& satellitePositionECEF, double frequencyHz)
{
    double distanceM = ComputeSlantDistance(satellitePositionECEF);
    double distanceKm = distanceM / 1000.0;

    // Basic free space path loss: 20*log10(4*π*d*f/c) where d in km, f in Hz
    // Simplified: 32.45 + 20*log10(f_MHz) + 20*log10(d_km)
    double freqMHz = frequencyHz / 1e6;
    double pathLossDb = 32.45 + 20.0 * std::log10(freqMHz) + 20.0 * std::log10(distanceKm);

    return pathLossDb;
}

double
LocalDegreesToRadians(double degrees)
{
    return degrees * M_PI / 180.0;
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

// Predicts where a circular-orbit satellite will be, in ECEF, at simulation time t. It
// reproduces LeoCircularOrbitMobilityModel::CalcPosition of ns-3.48 from the position and
// velocity the model reports at t = 0: a circular orbit in the plane spanned by pos0 and vel0
// at the model's angular velocity, then the Earth's rotation (2 pi per day, as the model does)
// applied as a rotation about the ECEF z axis. The model has no public "position at time t",
// and the NTN timing needs the worst-case geometry of the window before the simulation runs.
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
MakeSatellitePredictor(Ptr<MobilityModel> mobility)
{
    SatellitePredictor predictor;
    predictor.pos0 = mobility->GetPosition();
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

// Worst-case service-link geometry of the simulated window, for the cell-specific K_offset and
// the RRC timer. The sweep walks the pass at the stop-check period and ends where the run
// would: at --duration, or at the first sample from t = step on whose elevation is at or below
// the boundary (StopAtElevationBoundary with hasEnteredPass = true, as main() calls it), or at
// the horizon. The loop RTT is 2 x the longest UE slant range / c.
struct LoopRttSweep
{
    Time maxLoopRtt;
    Time maxLoopRttAt;
    double minElevationDeg{90.0};
    Time windowEnd;
};

LoopRttSweep
SweepRegenerativeLoopRtt(const SatellitePredictor& satellite,
                         const std::vector<Vector>& ueEcef,
                         Time duration,
                         Time step,
                         double boundaryElevationDeg)
{
    LoopRttSweep sweep;
    sweep.windowEnd = duration;
    for (Time t = Seconds(0); t <= duration; t += step)
    {
        const Vector sat = satellite.At(t);
        const double elevationDeg = ComputeElevationAngle(sat);
        if (t.IsStrictlyPositive() && (elevationDeg <= boundaryElevationDeg || elevationDeg <= 0.0))
        {
            sweep.windowEnd = t;
            break;
        }
        sweep.minElevationDeg = std::min(sweep.minElevationDeg, elevationDeg);
        double maxSlantM = 0.0;
        for (const auto& ue : ueEcef)
        {
            maxSlantM = std::max(maxSlantM, (sat - ue).GetLength());
        }
        const Time loopRtt = Seconds(2.0 * maxSlantM / SpeedOfLightMps);
        if (loopRtt > sweep.maxLoopRtt)
        {
            sweep.maxLoopRtt = loopRtt;
            sweep.maxLoopRttAt = t;
        }
    }
    return sweep;
}

// Ideal timing advance for one UE: the one-way delay (slant range / c) to the satellite gNB it
// is attached to. Until the attachment is recorded on the device (NrHelper::AttachWindow can
// defer it) the closest gNB is used, which is the one AttachToClosestGnb picks.
Time
ComputeRegenerativeTimingAdvance(Ptr<NetDevice> ueDevice,
                                 Ptr<MobilityModel> ueMobility,
                                 const NetDeviceContainer& gnbDevices)
{
    Ptr<MobilityModel> gnbMobility;
    Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(ueDevice);
    if (ueDev && ueDev->GetTargetGnb())
    {
        gnbMobility = ueDev->GetTargetGnb()->GetNode()->GetObject<MobilityModel>();
    }
    else
    {
        double closestM = std::numeric_limits<double>::max();
        for (uint32_t i = 0; i < gnbDevices.GetN(); ++i)
        {
            Ptr<MobilityModel> candidate = gnbDevices.Get(i)->GetNode()->GetObject<MobilityModel>();
            const double d = (candidate->GetPosition() - ueMobility->GetPosition()).GetLength();
            if (d < closestM)
            {
                closestM = d;
                gnbMobility = candidate;
            }
        }
    }
    if (!gnbMobility)
    {
        return Seconds(0);
    }
    const double slantM = (gnbMobility->GetPosition() - ueMobility->GetPosition()).GetLength();
    return Seconds(slantM / SpeedOfLightMps);
}

void
UpdateAntennaOrientation(Ptr<Node> node,
                         std::vector<Ptr<UniformPlanarArray>> satelliteNodeAntennas,
                         std::vector<Ptr<PhasedArrayModel>> groundAntennas,
                         Time period)
{
    auto mobility = node->GetObject<MobilityModel>();
    const auto satelliteNodePositionECEF = mobility->GetPosition();

    // Compute direction from satellite to ground in ECEF.
    // The 3GPP channel model computes GCS ray angles via Angles(uMobPos_ECEF, sMobPos_ECEF),
    // i.e. azimuth = atan2(Δy_ECEF, Δx_ECEF), inclination = acos(Δz_ECEF/r).
    // Alpha and beta must be expressed in the same ECEF GCS so that
    // UniformPlanarArray::GetElementFieldPattern rotates correctly from GCS to LCS.
    // Using local ENU angles here would be a frame mismatch and causes SINR errors
    // as the satellite's ENU frame rotates away from ECEF during the orbit.
    // The single beam is steered at the beam center (UE 0); the additional UEs sit
    // inside the same beam footprint. One array per BWP (DL and UL) points the same way.
    const Angles angles(groundNodePosECEF, satelliteNodePositionECEF);

    for (auto& satelliteNodeAntenna : satelliteNodeAntennas)
    {
        // The ray tracer places the satellite as a proxy in the scene's east/north/up frame and
        // reads alpha/beta straight through as its (yaw, pitch) there. Earth-centred angles
        // mean nothing in that frame, and they skewed the two halves of a pass differently:
        // ~6 dB between rising and setting at the same elevation with clear sky both ways, on a
        // scene measured symmetric offline. With isotropic elements and ideal direct-path
        // beamforming the array gain is the same in every direction, so the orientation carries
        // no physics here; leave it at identity so ns-3 and Sionna agree trivially. The
        // satellite's pointing gain is the scalar satAntennaGainDb.
        (void)angles;
        satelliteNodeAntenna->SetAlpha(0.0);
        satelliteNodeAntenna->SetBeta(0.0);

        // FORCE CHANNEL UPDATE: Update beamforming vectors to invalidate cached channel matrix
        // This ensures the 3GPP channel model regenerates parameters on next query
        // SetBeamformingVector() marks the antenna as changed internally
        satelliteNodeAntenna->SetBeamformingVector(satelliteNodeAntenna->GetBeamformingVector());
    }
    for (auto& groundAntenna : groundAntennas)
    {
        groundAntenna->SetBeamformingVector(groundAntenna->GetBeamformingVector());
    }

    // schedules itself again after a certain period
    Simulator::Schedule(period,
                        &UpdateAntennaOrientation,
                        node,
                        satelliteNodeAntennas,
                        groundAntennas,
                        period);
}

// Spot-beam gain pattern: the satellite beam is steered at the beam centre (UE 0). A UE at
// off-boresight angle theta (as seen from the satellite) sits on the uniform circular-aperture
// (Airy) pattern G(theta) = 4*|J1(ka*sin theta)/(ka*sin theta)|^2 (1 at boresight, -3 dB at the
// half-power point, with the proper first null + sidelobes). ka is set so the half-power point
// is at HPBW/2. theta is computed from the live ECEF geometry, so the taper a fixed ground UE
// sees changes over the pass (the beam footprint stretches at low elevation -> beam distortion;
// the edge UEs also sweep through the pattern's null/sidelobes).
double
SpotBeamTaperDb(const Vector& satEcef,
                const Vector& beamCentreEcef,
                const Vector& ueEcef,
                double hpbwDeg,
                double maxAttenDb)
{
    const Vector vc = beamCentreEcef - satEcef; // sat -> beam centre (boresight)
    const Vector vu = ueEcef - satEcef;         // sat -> UE
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
        return 0.0; // boresight
    }
    // ka chosen so the half-power point (4|J1(x)/x|^2 = 1/2) lands at theta = HPBW/2.
    const double ka = 1.616339 / std::sin(hpbwDeg / 2.0 * M_PI / 180.0);
    const double x = ka * sinTheta;
    const double f = j1(x) / x;   // J1(x)/x -> 1/2 as x -> 0
    const double g = 4.0 * f * f; // normalized power gain, 1.0 at boresight
    return std::max(10.0 * std::log10(std::max(g, 1e-12)), -maxAttenDb);
}

// Recomputes each UE's effective antenna gain (base + spot-beam taper) from the current
// satellite geometry and pushes it into the UE antenna element. Forces a channel-matrix
// invalidation on each UE array (one per BWP) so the new gain takes effect on the next 3GPP
// channel query. Reschedules itself every `period` (aligned with UpdateAntennaOrientation's
// cadence).
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
        // Mark the arrays changed so the cached channel matrix / beamforming gain regenerates.
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

Vector
GeneratePoint(const Vector& nodePosECEF, const double azimuth, const double inclination)
{
    // convert node position vector from ECEF (x, y, z) to GEO (Geodesic/Geographic : lon/lat/alt)
    const Vector nodePosGEO =
        GeographicPositions::CartesianToGeographicCoordinates(nodePosECEF,
                                                              GeographicPositions::SPHERE);

    // in the plot, this is the magnitude of the vector that represent the antenna orientation
    // leaving the node and reaching the ground station
    const double orientationVectorLength = (groundNodePosECEF - nodePosECEF).GetLength();

    // pre-calculate sin(inclination)
    double s = std::sin(inclination);

    // newly generated point
    const Vector newPointUnitVectorInTopo(s * std::cos(azimuth),
                                          s * std::sin(azimuth),
                                          std::cos(inclination));

    // apply magnitude to the unit vector
    const Vector newPointWithMagnitudeTopo = newPointUnitVectorInTopo * orientationVectorLength;

    // convert from Topocentric/ENU to Geographic (2nd arg. must be in GEO)
    const Vector newPointWithMagnitudeGEO =
        GeographicPositions::TopocentricToGeographicCoordinates(newPointWithMagnitudeTopo,
                                                                nodePosGEO,
                                                                GeographicPositions::SPHERE);

    // convert from Geographic to Cartesian - the plot uses cartesian coordinates.
    const Vector newPointWithMagnitudeECEF =
        GeographicPositions::GeographicToCartesianCoordinates(newPointWithMagnitudeGEO.x,
                                                              newPointWithMagnitudeGEO.y,
                                                              newPointWithMagnitudeGEO.z,
                                                              GeographicPositions::SPHERE);

    return newPointWithMagnitudeECEF;
}

// ---------------------------------------------------------------------------------------------
// The three pieces that turn a ray tracer meant for a city block into an NTN channel.
//
// They sit on the spectrum PHYs rather than on the nodes, so everything else in this example -
// elevation, slant range, timing advance, K_offset - keeps using the real Earth-centred geometry.
// ---------------------------------------------------------------------------------------------

/// Reports the satellite's proxy position in the scene frame, and remembers the true range.
class NtnProxyMobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnProxyMobilityModel")
                                .SetParent<MobilityModel>()
                                .AddConstructor<NtnProxyMobilityModel>();
        return tid;
    }

    void Setup(Ptr<const MobilityModel> satellite,
               Vector observerEcef,
               Vector terminalScene,
               double proxyRadiusM)
    {
        m_satellite = satellite;
        m_observer = observerEcef;
        m_terminal = terminalScene;
        m_radius = proxyRadiusM;
    }

    /// The distance the proxy position deliberately does not carry.
    double GetTrueRangeM() const
    {
        return CalculateDistance(m_satellite->GetPosition(), m_observer);
    }

    /// Rate of change of the true range, in m/s: negative while the satellite approaches
    double GetTrueRangeRateMps() const
    {
        const Vector d = m_satellite->GetPosition() - m_observer;
        const Vector v = m_satellite->GetVelocity();
        return (d.x * v.x + d.y * v.y + d.z * v.z) / d.GetLength();
    }

    double GetProxyRadiusM() const
    {
        return m_radius;
    }

    /// ns-3.48 made this pure virtual for wraparound support.
    Ptr<MobilityModel> Copy() const override
    {
        auto c = CreateObject<NtnProxyMobilityModel>();
        c->Setup(m_satellite, m_observer, m_terminal, m_radius);
        return c;
    }

  private:
    Vector DoGetPosition() const override
    {
        const Vector sat = m_satellite->GetPosition();
        const double r = std::sqrt(m_observer.x * m_observer.x + m_observer.y * m_observer.y +
                                   m_observer.z * m_observer.z);
        const Vector up(m_observer.x / r, m_observer.y / r, m_observer.z / r);
        const double en = std::hypot(-m_observer.y, m_observer.x);
        const Vector east =
            (en > 0) ? Vector(-m_observer.y / en, m_observer.x / en, 0.0) : Vector(1.0, 0.0, 0.0);
        const Vector north(up.y * east.z - up.z * east.y,
                           up.z * east.x - up.x * east.z,
                           up.x * east.y - up.y * east.x);
        const Vector d(sat.x - m_observer.x, sat.y - m_observer.y, sat.z - m_observer.z);
        const double e = d.x * east.x + d.y * east.y + d.z * east.z;
        const double n = d.x * north.x + d.y * north.y + d.z * north.z;
        const double u = d.x * up.x + d.y * up.y + d.z * up.z;
        const double norm = std::sqrt(e * e + n * n + u * u);
        if (norm <= 0.0)
        {
            return m_terminal;
        }
        return Vector(m_terminal.x + m_radius * e / norm,
                      m_terminal.y + m_radius * n / norm,
                      m_terminal.z + m_radius * u / norm);
    }

    void DoSetPosition(const Vector&) override
    {
    }

    Vector DoGetVelocity() const override
    {
        // The satellite reports its velocity in Earth-centred axes, but the scene the ray tracer
        // sees is east/north/up at the terminal. Handing Sionna the Earth-centred vector gives
        // every path the wrong Doppler, differently on each half of the pass. Rotate it into the
        // same basis DoGetPosition uses.
        //
        // It is also scaled by proxy radius / true range. Paths to different scatterers leave the
        // proxy at angles range/radius times wider than they leave the satellite, so at full
        // speed the Doppler difference between paths, which is what makes the channel fade,
        // comes out 50-170 times too large. At the scaled speed the proxy sweeps the same angle
        // per second as the satellite, which gets the differences right. What is lost is the
        // Doppler common to every path, which the terminal's frequency tracking removes anyway.
        if (!m_satellite)
        {
            return Vector(0, 0, 0);
        }
        const Vector v = m_satellite->GetVelocity() * (m_radius / GetTrueRangeM());
        const double r = std::sqrt(m_observer.x * m_observer.x + m_observer.y * m_observer.y +
                                   m_observer.z * m_observer.z);
        const Vector up(m_observer.x / r, m_observer.y / r, m_observer.z / r);
        const double en = std::hypot(-m_observer.y, m_observer.x);
        const Vector east =
            (en > 0) ? Vector(-m_observer.y / en, m_observer.x / en, 0.0) : Vector(1.0, 0.0, 0.0);
        const Vector north(up.y * east.z - up.z * east.y,
                           up.z * east.x - up.x * east.z,
                           up.x * east.y - up.y * east.x);
        return Vector(v.x * east.x + v.y * east.y + v.z * east.z,
                      v.x * north.x + v.y * north.y + v.z * north.z,
                      v.x * up.x + v.y * up.y + v.z * up.z);
    }

    Ptr<const MobilityModel> m_satellite;
    Vector m_observer{0, 0, 0};
    Vector m_terminal{0, 0, 0};
    double m_radius{10000.0};
};

/// Delay from the true slant range, ignoring the proxy positions the ray tracer is shown. Derives
/// from NrNtnPropagationDelayModel for its uplink line-up and so that the UE's PHY also compresses
/// the downlink by the true range's drift (NrNtnPropagationDelayModel::GetOneWayDelayRate).
class NtnTrueRangeDelayModel : public NrNtnPropagationDelayModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnTrueRangeDelayModel")
                                .SetParent<NrNtnPropagationDelayModel>()
                                .AddConstructor<NtnTrueRangeDelayModel>();
        return tid;
    }

    void SetProxy(Ptr<NtnProxyMobilityModel> proxy)
    {
        m_proxy = proxy;
    }

    /// The gNB PHY whose symbol grid uplink arrivals are lined up on
    void SetGnbPhy(Ptr<const NrGnbPhy> phy)
    {
        m_gnbPhy = phy;
    }

    // The proxy stands on a device-less node, so the base class cannot tell a gNB end from it;
    // an uplink arrival is one at the proxy, lined up as NrNtnPropagationDelayModel::GetDelay does
    Time GetDelay(Ptr<MobilityModel>, Ptr<MobilityModel> b) const override
    {
        const Time delay = Seconds(m_proxy ? m_proxy->GetTrueRangeM() / 299792458.0 : 0.0);
        if (!m_gnbPhy || b != m_proxy)
        {
            return delay;
        }
        const Time now = Simulator::Now();
        return AlignUplinkArrival(now + delay,
                                  m_gnbPhy->GetSlotPeriod(),
                                  m_gnbPhy->GetSymbolPeriod(),
                                  m_gnbPhy->GetSymbolsPerSlot()) -
               now;
    }

    double GetOneWayDelayRate(Ptr<MobilityModel>, Ptr<MobilityModel>) const override
    {
        return m_proxy ? m_proxy->GetTrueRangeRateMps() / 299792458.0 : 0.0;
    }

  private:
    int64_t DoAssignStreams(int64_t) override
    {
        return 0;
    }

    Ptr<NtnProxyMobilityModel> m_proxy;
    Ptr<const NrGnbPhy> m_gnbPhy;
};

/// The free-space loss the real slant range costs beyond the proxy distance.
///
/// The ray tracer already charges free-space loss over the few km to the proxy; this adds the
/// rest. It goes in the channel's scalar propagation-loss slot, which MultiModelSpectrumChannel
/// applies independently of the ray-traced one, so the two compose without either touching the
/// other. Frequency cancels in the ratio, so it needs none.
class NtnRangeCorrectionLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnRangeCorrectionLossModel")
                                .SetParent<PropagationLossModel>()
                                .AddConstructor<NtnRangeCorrectionLossModel>();
        return tid;
    }

    void SetProxy(Ptr<NtnProxyMobilityModel> proxy)
    {
        m_proxy = proxy;
    }

    /// Antenna gains the ray tracer does not model: it uses its own scene arrays, so the dish and
    /// satellite gains configured on the ns-3 antennas never reach the link and it falls ~70 dB
    /// short. They are part of the large-scale budget here, alongside the range.
    void SetAntennaGainDb(double gainDb)
    {
        m_antennaGainDb = gainDb;
    }

  private:
    double DoCalcRxPower(double txPowerDbm, Ptr<MobilityModel>, Ptr<MobilityModel>) const override
    {
        if (!m_proxy)
        {
            return txPowerDbm;
        }
        const double trueRange = std::max(1.0, m_proxy->GetTrueRangeM());
        const double proxyRange = std::max(1.0, m_proxy->GetProxyRadiusM());
        return txPowerDbm - 20.0 * std::log10(trueRange / proxyRange) + m_antennaGainDb;
    }

    int64_t DoAssignStreams(int64_t) override
    {
        return 0;
    }

    Ptr<NtnProxyMobilityModel> m_proxy;
    double m_antennaGainDb{0.0};
};

NS_OBJECT_ENSURE_REGISTERED(NtnProxyMobilityModel);
NS_OBJECT_ENSURE_REGISTERED(NtnTrueRangeDelayModel);
NS_OBJECT_ENSURE_REGISTERED(NtnRangeCorrectionLossModel);

void
CourseChange(Ptr<const MobilityModel> mob)
{
    const Vector pos = mob->GetPosition();
    Ptr<const Node> node = mob->GetObject<Node>();
    Ptr<UniformPlanarArray> ant = node->GetObject<UniformPlanarArray>();
    const Vector newP = GeneratePoint(pos, ant->GetAlpha(), ant->GetBeta());

    // Compute elevation angle and slant distance
    double elevationAngle = ComputeElevationAngle(pos);
    double slantDistance = ComputeSlantDistance(pos);

    // Enhanced logging: Time:Satellite:x:y:z:x_2:y_2:z_2:elevation:distance
    std::cout << Simulator::Now() << ":" << node->GetId() << ":" << pos.x << ":" << pos.y << ":"
              << pos.z << ":" << newP.x << ":" << newP.y << ":" << newP.z << ":" << elevationAngle
              << ":" << slantDistance << std::endl;
}

void
StopAtElevationBoundary(Ptr<MobilityModel> mobility,
                        double boundaryElevationDeg,
                        Time period,
                        bool hasEnteredPass)
{
    const double elevationDeg = ComputeElevationAngle(mobility->GetPosition());
    const bool enteredPass = hasEnteredPass || elevationDeg > boundaryElevationDeg + 0.1;
    if (enteredPass && elevationDeg <= boundaryElevationDeg)
    {
        std::clog << "Observer elevation boundary reached at " << Simulator::Now()
                  << ": elevation=" << elevationDeg << " boundary=" << boundaryElevationDeg
                  << " degrees" << std::endl;
        Simulator::Stop();
        return;
    }

    Simulator::Schedule(period,
                        &StopAtElevationBoundary,
                        mobility,
                        boundaryElevationDeg,
                        period,
                        enteredPass);
}

void
LogChannelCondition(Ptr<ChannelConditionModel> channelConditionModel,
                    Ptr<const MobilityModel> satelliteMobility,
                    Ptr<const MobilityModel> groundMobility,
                    Time period,
                    std::ostream* out)
{
    if (!out || !(*out) || !channelConditionModel)
    {
        return;
    }

    Ptr<ChannelCondition> condition =
        channelConditionModel->GetChannelCondition(satelliteMobility, groundMobility);
    const bool isLos = condition && condition->IsLos();
    *out << Simulator::Now().GetSeconds() << "\t" << (isLos ? 1 : 0) << "\t";
    if (condition)
    {
        *out << condition->GetLosCondition();
    }
    else
    {
        *out << "UNKNOWN";
    }
    *out << std::endl;

    Simulator::Schedule(period,
                        &LogChannelCondition,
                        channelConditionModel,
                        satelliteMobility,
                        groundMobility,
                        period,
                        out);
}

class BackloggedUdpSourceApplication : public Application
{
  public:
    BackloggedUdpSourceApplication() = default;

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
    void StartApplication() override;
    void StopApplication() override;
    void SendBurst();

    Address m_remote;
    uint32_t m_packetSize{0};
    Ptr<Socket> m_socket;
    EventId m_sendEvent;
    Time m_burstInterval{MilliSeconds(1)};
    uint32_t m_packetsPerBurst{1};
};

void
BackloggedUdpSourceApplication::StartApplication()
{
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_socket->SetAllowBroadcast(true);
    m_socket->Connect(m_remote);
    m_sendEvent = Simulator::ScheduleNow(&BackloggedUdpSourceApplication::SendBurst, this);
}

void
BackloggedUdpSourceApplication::StopApplication()
{
    Simulator::Cancel(m_sendEvent);
    if (m_socket)
    {
        m_socket->Close();
        m_socket = nullptr;
    }
}

void
BackloggedUdpSourceApplication::SendBurst()
{
    if (!m_socket)
    {
        return;
    }

    for (uint32_t packetIndex = 0; packetIndex < m_packetsPerBurst; ++packetIndex)
    {
        Ptr<Packet> packet = Create<Packet>(m_packetSize);
        if (m_socket->Send(packet) < 0)
        {
            break;
        }
    }

    m_sendEvent =
        Simulator::Schedule(m_burstInterval, &BackloggedUdpSourceApplication::SendBurst, this);
}

int
main(int argc, char* argv[])
{
    // Sionna runs in an embedded CPython, and the interpreter is the program's to own: the
    // channel model only imports into it. Without this the first channel computation calls the
    // Python C API on an uninitialised interpreter and segfaults inside PyImport_ImportModule.
    py::scoped_interpreter guard{};

    CommandLine cmd(__FILE__);
    std::string orbitFile;
    std::string traceFile;
    std::string channelConditionTraceFile;
    uint32_t precision = 1000; // milliseconds
    uint32_t duration = 4;     // seconds
    std::string scenario = "NTN-Rural";
    std::string sionnaScene = "munich";
    std::string sionnaTerminal = "0:-150:1.5";
    double sionnaProxyKm = 10.0;
    // Diffraction is often the only path left when a building blocks the direct ray, and the
    // path solver finds it by shooting rays: from a proxy kilometres out only a sliver of them
    // reach the scene, and at Sionna's default of one million none land near a roof edge.
    // Measured on UW Red Square: behind Suzzallo Library at 14.5 deg, 10 M rays found one path
    // at -230 dB and 100 M found seven at -187 dB. 100 M converges almost everywhere and costs
    // no measurable time on a GPU. Deep in a building's shadow even 300 M keeps finding
    // stronger diffraction, so there the traced power is a lower bound.
    bool sionnaDiffraction = true;
    uint32_t sionnaSamplesPerSource = 100000000;
    bool sionnaRender = false;
    std::string sionnaCamera = "-450:-450:320";
    // FDD carrier pair. The downlink keeps the fork's 3.5 GHz / 100 MHz; the uplink sits
    // 190 MHz below it, the duplex spacing of 3GPP band n256 (S-band NTN).
    double dlFrequencyHz = 3.5e9; // DL carrier centre in Hz (BWP 0)
    double ulFrequencyHz = 0.0;   // UL carrier centre in Hz (BWP 1); 0 = dlFrequencyHz - 190 MHz
    double bandwidthHz = 100e6;   // Hz, per direction
    double vsatAntennaGainDb = 0; // dBi
    double satAntennaGainDb = 0;  // dBi
    double groundTxPower = 0;     // dBm
    double satEIRP = 10;          // dBW/MHz
    double s1uLinkDelayMs = 0.0;
    double remoteHostDelayMs = 10.0;
    std::string transport = "udp"; // "tcp" or "udp"
    double udpRateMbps = 500.0;
    uint32_t udpPacketSize = 1200;
    bool useFixedMcs = false;
    uint32_t fixedMcs = 20;
    std::string observerMode = "overhead";
    // Aim the pass at a real place. ntn/tools/aim_pass.py solves the satellite's starting orbital
    // position for a pass directly overhead a latitude/longitude and prints these four flags.
    // Unset (NaN) keeps the placement observerMode derives from the orbit.
    double observerLatDeg = std::numeric_limits<double>::quiet_NaN();
    double observerLonDeg = std::numeric_limits<double>::quiet_NaN();
    double satLongitudeDeg = std::numeric_limits<double>::quiet_NaN();
    double satArgOfLatitudeDeg = std::numeric_limits<double>::quiet_NaN();
    double observerElevationBoundaryDeg = 10.0;
    bool enableMimoFeedback = true;
    std::string pmSearchMethod = "ns3::NrPmSearchFull";
    std::string fullSearchCb = "ns3::NrCbTwoPort";
    std::string schedulerType = "ns3::NrMacSchedulerTdmaRR";
    uint32_t rankLimit = 2;
    double rankThreshold = 0.0;
    std::string rankTechnique = "Sasaoka";
    uint32_t subbandSize = 32;
    std::string downsamplingTechnique = "FirstPRB";
    uint32_t ueAntennaRows = 1;
    uint32_t ueAntennaColumns = 2;
    uint32_t ueNumVerticalPorts = 1;
    uint32_t ueNumHorizontalPorts = 2;
    // One element per polarization. The spot beam's gain is already inside satEIRP, and the beam
    // carries control as well as data. A larger array adds its array gain on top of the EIRP
    // (6 dB for 2x2), and 5G-LENA sends control on a quasi-omni weighting that no satellite uses:
    // on a 2x2 that arrives ~11 dB under data and trips radio-link failure while data still gets
    // through.
    uint32_t gnbAntennaRows = 1;
    uint32_t gnbAntennaColumns = 1;
    uint32_t gnbNumVerticalPorts = 1;
    uint32_t gnbNumHorizontalPorts = 1;
    bool ueIsDualPolarized = false;
    bool gnbIsDualPolarized = true;
    // Co-polar with the gNB's first port (0 deg). A ray-traced direct path keeps its
    // polarization, so an antenna crossed with the gNB's receives nothing on it; the statistical
    // channel of leo-link hides that behind its finite cross-polar ratio.
    double uePolSlantAngleDeg = 0.0;
    double gnbPolSlantAngleDeg = 0.0;
    bool enableHarqReTx = false;
    bool consolidateHarqRetx = false;
    uint32_t numHarqProcess = 16;
    uint32_t n1Delay = 2;
    uint32_t n2Delay = 2;
    uint32_t rlcReorderingTimerMs = 100;
    uint32_t numUes = 1;
    double ueSpacingDeg = 0.05;
    double footprintRadiusKm = 0.0;
    double beamHpbwDeg = 0.0;     // >0: geometry-driven spot-beam taper, half-power beamwidth (deg)
    double beamMaxAttenDb = 30.0; // floor of the spot-beam pattern attenuation
    std::string ueRadiiKmStr =
        ""; // explicit per-UE radial distances, e.g. "0,30,30,50" (overrides auto-placement)
    // NTN timing: cell-specific K_offset, ideal timing advance and the RRC connection-request
    // timer, all derived from the geometry of the run (see the file header).
    bool ntnTiming = true;
    // Ideal random access: the RA response timeout is never armed, so attach cannot fail at
    // elevations where the RAR window does not fit the round trip.
    bool idealRa = true;
    cmd.AddValue("beamHpbwDeg",
                 "Half-power beamwidth (deg) of a geometry-driven spot-beam taper "
                 "G(theta)=-min(12*(theta/HPBW)^2, beamMaxAttenDb); 0 disables (static r/R taper)",
                 beamHpbwDeg);
    cmd.AddValue("beamMaxAttenDb", "Max spot-beam pattern attenuation (dB)", beamMaxAttenDb);
    cmd.AddValue("ueRadiiKm",
                 "Comma-separated per-UE radial distances from beam centre in km "
                 "(e.g. 0,30,30,50); overrides the 2-centre+edge auto-placement",
                 ueRadiiKmStr);
    cmd.AddValue("orbitFile",
                 "CSV file with orbit parameters in the ns-3.48 format (header line, then "
                 "altitudeKm,inclinationDegrees,numberOfPlanes,numberOfSatellitesPerPlane"
                 "[,phasing,raanSpanDeg] per row)",
                 orbitFile);
    cmd.AddValue("traceFile", "CSV file to store mobility trace in", traceFile);
    cmd.AddValue("channelConditionTraceFile",
                 "Text file to store channel LOS/NLOS condition trace in",
                 channelConditionTraceFile);
    cmd.AddValue("precision",
                 "Mobility resolution in milliseconds: the period of the orbit model's "
                 "CourseChange notifications, of the channel regeneration, of the antenna "
                 "re-pointing and of the timing-advance refresh. The satellite position itself "
                 "is continuous in time.",
                 precision);
    cmd.AddValue("duration", "Duration of the simulation in seconds", duration);
    cmd.AddValue("scenario", "Scenario for the 3GPP Channel Model", scenario);
    cmd.AddValue("sionnaScene",
                 "Sionna RT scene: munich, san_francisco, etoile, florence, the street canyons, "
                 "or a path to a Mitsuba .xml. Munich, San Francisco, Etoile and Florence are "
                 "surveyed cities; the canyons are synthetic.",
                 sionnaScene);
    cmd.AddValue("sionnaTerminal",
                 "Terminal position inside the scene, 'x:y:z' in metres. It has to stand on open "
                 "ground: the origin of the munich scene is inside a building and sees no sky at "
                 "any elevation, which looks exactly like a broken run.",
                 sionnaTerminal);
    cmd.AddValue("sionnaDiffraction",
                 "Trace diffracted paths in the ray tracer (Sionna RT turns it off by default)",
                 sionnaDiffraction);
    cmd.AddValue("sionnaSamplesPerSource",
                 "Rays launched per source by the ray tracer; a distant proxy needs many for "
                 "diffraction and reflections to be found",
                 sionnaSamplesPerSource);
    cmd.AddValue("sionnaProxyKm",
                 "Radius of the proxy transmitter standing in for the satellite. Larger is more "
                 "plane-wave-like: across the 200 m that matters for shadows, 2 km fans the rays "
                 "by 5.7 degrees and 10 km by 1.1.",
                 sionnaProxyKm);
    cmd.AddValue("sionnaRender",
                 "Write a scene image per channel update, showing the buildings and the traced "
                 "rays. Roughly doubles the cost of a run, so sample the geometry coarsely.",
                 sionnaRender);
    cmd.AddValue("sionnaCamera",
                 "Camera position for the scene images, 'x:y:z' in metres",
                 sionnaCamera);
    cmd.AddValue("dlFrequencyHz", "Downlink carrier centre frequency in Hz (BWP 0)", dlFrequencyHz);
    cmd.AddValue("ulFrequencyHz",
                 "Uplink carrier centre frequency in Hz (BWP 1); 0 derives dlFrequencyHz - 190 MHz",
                 ulFrequencyHz);
    cmd.AddValue("bandwidthHz", "Channel bandwidth in Hz of each FDD carrier", bandwidthHz);
    cmd.AddValue("vsatAntennaGainDb", "VSAT Antenna (UE) Gain in dBi", vsatAntennaGainDb);
    cmd.AddValue("satAntennaGainDb",
                 "Satellite Antenna (gNB) element gain in dBi",
                 satAntennaGainDb);
    cmd.AddValue("groundTxPower", "Set Ground Node TxPowerLevel at the PHY in dBm", groundTxPower);
    cmd.AddValue("satEIRP", "Set Satellite EIRP in dBW/MHz", satEIRP);
    cmd.AddValue("s1uLinkDelayMs", "S1-U backhaul one-way delay in ms", s1uLinkDelayMs);
    cmd.AddValue("remoteHostDelayMs", "RemoteHost-to-PGW one-way delay in ms", remoteHostDelayMs);
    cmd.AddValue("transport", "Application transport: tcp or udp", transport);
    cmd.AddValue("udpRateMbps",
                 "UDP offered load in Mbps (used only when --transport=udp, <=0 means "
                 "backlogged/unlimited source)",
                 udpRateMbps);
    cmd.AddValue("udpPacketSize",
                 "UDP payload size in bytes (used only when --transport=udp)",
                 udpPacketSize);
    cmd.AddValue("useFixedMcs", "Use fixed MCS in scheduler for DL/UL", useFixedMcs);
    cmd.AddValue("fixedMcs", "Fixed MCS index to use when --useFixedMcs=1", fixedMcs);
    cmd.AddValue("enableHarqReTx", "Enable HARQ retransmissions in the scheduler", enableHarqReTx);
    cmd.AddValue("consolidateHarqRetx",
                 "Reshape HARQ retransmission allocations to consolidate occupied resources",
                 consolidateHarqRetx);
    cmd.AddValue("numHarqProcess",
                 "Number of HARQ processes per UE. With --ntnTiming a process is kept for "
                 "numHarqProcess + K_offset slots; without it the pool must cover the loop RTT "
                 "in slots on its own.",
                 numHarqProcess);
    cmd.AddValue("n1Delay", "K1 slots between DL data and UE HARQ feedback (0-4)", n1Delay);
    cmd.AddValue("n2Delay", "Slots between UL grant and UL transmission (0-4)", n2Delay);
    cmd.AddValue("rlcReorderingTimerMs",
                 "NrRlcUm t-Reordering in ms. Must exceed worst-case multi-retx HARQ delay.",
                 rlcReorderingTimerMs);
    cmd.AddValue("numUes", "Number of ground UEs sharing the beam", numUes);
    cmd.AddValue("ueSpacingDeg",
                 "Geographic latitude offset in degrees between successive UEs",
                 ueSpacingDeg);
    cmd.AddValue("footprintRadiusKm",
                 "Beam footprint radius in km (>0 enables centre/edge placement + spot-beam gain "
                 "taper: 0 dB at centre, -3 dB at the footprint edge). 0 disables (uses "
                 "ueSpacingDeg).",
                 footprintRadiusKm);
    cmd.AddValue("observerMode", "Observer placement mode: overhead or full-pass", observerMode);
    cmd.AddValue("observerElevationBoundaryDeg",
                 "Elevation boundary in degrees for full-pass mode; the observer starts at this "
                 "elevation and the simulation stops when the pass descends below it",
                 observerElevationBoundaryDeg);
    cmd.AddValue("observerLatDeg",
                 "Terminal latitude in degrees; with --observerLonDeg, overrides --observerMode",
                 observerLatDeg);
    cmd.AddValue("observerLonDeg", "Terminal longitude in degrees", observerLonDeg);
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
    cmd.AddValue(
        "scheduler",
        "MAC scheduler TypeId (e.g. ns3::NrMacSchedulerTdmaRR, ns3::NrMacSchedulerOfdmaRR)",
        schedulerType);
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

    if (ulFrequencyHz <= 0.0)
    {
        ulFrequencyHz = dlFrequencyHz - 190e6;
    }

    if (transport != "tcp" && transport != "udp")
    {
        NS_ABORT_MSG("Unsupported --transport value. Use 'tcp' or 'udp'.");
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

    // Regenerate channel parameters periodically to keep them consistent with
    // the satellite's current geometry (elevation angle, distance).
    // Without this (UpdatePeriod=0), channel params (cluster angles, delays,
    // powers, K-factor) are generated ONCE at t=0 and never updated, causing
    // SINR to diverge from the expected trend as the satellite moves.
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(precision)));

    // Also re-evaluate channel condition (LOS/NLOS) at the same rate
    Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                       TimeValue(MilliSeconds(precision)));

    Config::SetDefault("ns3::NrMacSchedulerHarqRr::ConsolidateHarqRetx",
                       BooleanValue(consolidateHarqRetx));

    std::cout << "Run configuration: scheduler=" << schedulerType
              << " enableHarqReTx=" << (enableHarqReTx ? 1 : 0)
              << " consolidateHarqRetx=" << (consolidateHarqRetx ? 1 : 0)
              << " numHarqProcess=" << numHarqProcess << std::endl;
    std::cout << "FDD: DL BWP 0 at " << dlFrequencyHz / 1e6 << " MHz, UL BWP 1 at "
              << ulFrequencyHz / 1e6 << " MHz, " << bandwidthHz / 1e6
              << " MHz each, duplex spacing " << (dlFrequencyHz - ulFrequencyHz) / 1e6 << " MHz"
              << std::endl;

    // The orbit model of ns-3.48 computes the satellite position continuously; the resolution
    // only sets the CourseChange notification period (the mobility trace cadence).
    LeoOrbitNodeHelper orbit(Time(MilliSeconds(precision)));

    // creates the satellite nodes and put them into a container
    NodeContainer satellites;
    if (!orbitFile.empty())
    {
        satellites = orbit.CreateNodesAndInstallMobility(orbitFile);
    }
    else
    {
        satellites = orbit.CreateNodesAndInstallMobility(LeoOrbitalShell(600, 30, 1, 1));
    }

    // create node on the ground, create mobility, set position and aggregate mobility
    Ptr<Node> groundNode = CreateObject<Node>();
    Ptr<GeocentricEcefMobilityModel> groundNodeMobility =
        CreateObject<GeocentricEcefMobilityModel>();

    // Get the first satellite instantiated and derive the observer placement from it.
    auto firstSatelliteMobility = satellites.Get(0)->GetObject<MobilityModel>();
    if (!std::isnan(satLongitudeDeg) && !std::isnan(satArgOfLatitudeDeg))
    {
        // LeoCircularOrbitMobilityModel reads SetPosition as (plane longitude, argument of
        // latitude, index), so this restarts the satellite at a chosen point of its orbit.
        firstSatelliteMobility->SetPosition(Vector(satLongitudeDeg, satArgOfLatitudeDeg, 0));
    }
    auto firstSatellitePositionGEO =
        GeographicPositions::CartesianToGeographicCoordinates(firstSatelliteMobility->GetPosition(),
                                                              GeographicPositions::SPHERE);

    std::cout << "=== Satellite Mobility ===" << std::endl;
    std::cout << "Satellite GetPosition(): " << firstSatelliteMobility->GetPosition() << std::endl;

    Vector groundNodeGeographicPosition;
    if (!std::isnan(observerLatDeg) && !std::isnan(observerLonDeg))
    {
        groundNodeGeographicPosition = Vector(observerLatDeg, observerLonDeg, 0);
    }
    else if (observerMode == "full-pass")
    {
        groundNodeGeographicPosition =
            ComputeGroundStationPositionForInitialElevation(firstSatelliteMobility->GetPosition(),
                                                            firstSatelliteMobility->GetVelocity(),
                                                            observerBoundaryElevationDeg,
                                                            precision / 1000.0);
    }
    else
    {
        groundNodeGeographicPosition =
            Vector(firstSatellitePositionGEO.x, firstSatellitePositionGEO.y, 0);
    }
    groundNodeMobility->SetGeographicPosition(groundNodeGeographicPosition);

    // set global vars with GEO and ECEF positions for the Ground Node
    groundNodePosGEO = groundNodeMobility->GetGeographicPosition();
    groundNodePosECEF = groundNodeMobility->GetGeocentricPosition();

    std::cout << "=== Ground Node Mobility ===" << std::endl;
    std::cout << "Ground Node GetPosition(): " << groundNodeMobility->GetPosition() << std::endl;
    std::cout << "Ground Node GetGeocentricPosition(): " << groundNodePosECEF << std::endl;
    std::cout << "Observer Mode: " << observerMode << std::endl;
    if (observerMode == "full-pass")
    {
        std::cout << "Observer Elevation Boundary: " << observerBoundaryElevationDeg << " degrees"
                  << std::endl;
    }

    // Compute and display slant distance and elevation angle
    double slantDistance = ComputeSlantDistance(firstSatelliteMobility->GetPosition());
    double elevationAngle = ComputeElevationAngle(firstSatelliteMobility->GetPosition());
    std::cout << "=== Link Geometry ===" << std::endl;
    std::cout << "Slant Distance: " << slantDistance << " meters (" << slantDistance / 1000.0
              << " km)" << std::endl;
    std::cout << "Elevation Angle: " << elevationAngle << " degrees" << std::endl;

    groundNode->AggregateObject(groundNodeMobility);

    // UE 0 keeps the legacy placement (beam center / observer reference used by the
    // global geometry helpers); additional UEs are spread across the beam footprint
    // by successive latitude offsets.
    NS_ABORT_MSG_IF(numUes == 0, "numUes must be at least 1.");
    NodeContainer groundNodeContainer;
    groundNodeContainer.Add(groundNode);
    // Per-UE radial distance from the beam centre (UE 0), used for the spot-beam gain taper and
    // reported for the paper. With footprintRadiusKm>0 we place UEs as 2 near the centre + the rest
    // at the footprint edge (radius = footprintRadiusKm) on alternating sides, so centre-vs-edge
    // performance can be compared directly.
    const double kmPerDegLat = 111.195; // mean great-circle km per degree of latitude
    // Optional explicit per-UE radii (e.g. "0,30,30,50"): overrides the auto-placement so the
    // footprint geometry (centre / edge / deep-edge) can be set directly.
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
    std::vector<double> ueRadialKm(1, explicitRadiiKm.empty() ? 0.0 : explicitRadiiKm[0]); // UE 0
    for (uint32_t i = 1; i < numUes; ++i)
    {
        Ptr<Node> extraUe = CreateObject<Node>();
        Ptr<GeocentricEcefMobilityModel> extraMobility =
            CreateObject<GeocentricEcefMobilityModel>();
        Vector geoPosition = groundNodeMobility->GetGeographicPosition();
        double radialKm;
        double latOffsetDeg;
        if (i < explicitRadiiKm.size())
        {
            // Explicit placement: UE i at the given radius, alternating north/south sides.
            radialKm = explicitRadiiKm[i];
            latOffsetDeg = ((i % 2 == 0) ? 1.0 : -1.0) * radialKm / kmPerDegLat;
        }
        else if (footprintRadiusKm > 0.0)
        {
            // UE 1 is a second near-centre UE (small offset so its channel is not co-located with
            // UE 0); UE 2,3,... sit at the footprint edge on alternating north/south sides.
            radialKm = (i == 1) ? 2.0 : footprintRadiusKm;
            latOffsetDeg = ((i % 2 == 0) ? 1.0 : -1.0) * radialKm / kmPerDegLat;
        }
        else
        {
            latOffsetDeg = i * ueSpacingDeg;
            radialKm = std::abs(latOffsetDeg) * kmPerDegLat;
        }
        geoPosition.x += latOffsetDeg;
        extraMobility->SetGeographicPosition(geoPosition);
        extraUe->AggregateObject(extraMobility);
        groundNodeContainer.Add(extraUe);
        ueRadialKm.push_back(radialKm);
    }

    std::vector<Ptr<MobilityModel>> ueMobilities;
    std::vector<Vector> ueEcefPositions;
    for (uint32_t i = 0; i < groundNodeContainer.GetN(); ++i)
    {
        ueMobilities.push_back(groundNodeContainer.Get(i)->GetObject<MobilityModel>());
        ueEcefPositions.push_back(ueMobilities.back()->GetPosition());
    }

    // NTN timing, derived from the geometry the run will actually visit (first satellite).
    const Time stopCheckPeriod = MilliSeconds(std::max<uint32_t>(precision, 1));
    const LoopRttSweep sweep =
        SweepRegenerativeLoopRtt(MakeSatellitePredictor(firstSatelliteMobility),
                                 ueEcefPositions,
                                 Seconds(duration),
                                 stopCheckPeriod,
                                 observerBoundaryElevationDeg);
    const Time initialLoopRtt = Seconds(2.0 * slantDistance / SpeedOfLightMps);
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
    std::cout << "Loop RTT at t=0: " << initialLoopRtt.GetMicroSeconds() / 1000.0 << " ms"
              << std::endl;
    std::cout << "Sweep window: 0 - " << sweep.windowEnd.GetSeconds() << " s, min elevation "
              << sweep.minElevationDeg << " deg, max loop RTT "
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

    // Spectrum configuration: an FDD carrier pair, the pure-FDD recipe of nr-fdd-ul-routing.
    // Two operation bands of one CC and one BWP each, so that BWP 0 is the downlink carrier and
    // BWP 1 the uplink carrier whichever of the two is higher in frequency (n255 puts the DL
    // below the UL, n256 and n254 above it; CcBwpCreator lays the BWPs of a single band out
    // contiguously around one centre, which cannot express a duplex gap). Each BWP gets its own
    // 3GPP NTN channel at its own centre frequency.
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    constexpr uint8_t numCcPerBand = 1; // 1 CC per Band

    CcBwpCreator::SimpleOperationBandConf dlBandConf(dlFrequencyHz, bandwidthHz, numCcPerBand);
    CcBwpCreator::SimpleOperationBandConf ulBandConf(ulFrequencyHz, bandwidthHz, numCcPerBand);
    OperationBandInfo dlBand = ccBwpCreator.CreateOperationBandContiguousCc(dlBandConf);
    OperationBandInfo ulBand = ccBwpCreator.CreateOperationBandContiguousCc(ulBandConf);

    // Create the channel helper
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    // Set and configure the channel to the current band
    channelHelper->ConfigureFactories(
        scenario,
        "Default",
        "ThreeGpp"); // Use the scenario's default 3GPP channel condition model.
    channelHelper->AssignChannelsToBands({dlBand, ulBand});
    allBwps = CcBwpCreator::GetAllBwps({dlBand, ulBand});

    // Swap the statistical channel for the ray tracer. The helper still builds its channels
    // first, because it initialises more than the channel object itself; each is then replaced.
    Config::SetDefault("ns3::SionnaRtChannelModel::Scenario", StringValue(sionnaScene));
    Config::SetDefault("ns3::SionnaRtChannelModel::Diffraction", BooleanValue(sionnaDiffraction));
    Config::SetDefault("ns3::SionnaRtChannelModel::SamplesPerSource",
                       UintegerValue(sionnaSamplesPerSource));
    Config::SetDefault("ns3::SionnaRtChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(precision)));
    if (sionnaRender)
    {
        std::vector<double> cv;
        std::stringstream cs(sionnaCamera);
        std::string ctok;
        while (std::getline(cs, ctok, ':'))
        {
            cv.push_back(std::stod(ctok));
        }
        NS_ABORT_MSG_UNLESS(cv.size() == 3, "--sionnaCamera must be 'x:y:z'");
        std::vector<double> tv;
        std::stringstream ts2(sionnaTerminal);
        std::string ttok;
        while (std::getline(ts2, ttok, ':'))
        {
            tv.push_back(std::stod(ttok));
        }
        Config::SetDefault("ns3::SionnaRtChannelModel::IsImageRenderedEnabled", BooleanValue(true));
        Config::SetDefault("ns3::SionnaRtChannelModel::OutputImageDirectory", StringValue("."));
        Config::SetDefault("ns3::SionnaRtChannelModel::OutputImageName", StringValue("scene-"));
        Config::SetDefault("ns3::SionnaRtChannelModel::CameraPosition",
                           VectorValue(Vector(cv[0], cv[1], cv[2])));
        Config::SetDefault("ns3::SionnaRtChannelModel::CameraLookAt",
                           VectorValue(Vector(tv[0], tv[1], tv[2])));
    }
    Ptr<NtnProxyMobilityModel> ntnProxyMobility = CreateObject<NtnProxyMobilityModel>();

    TypeId sionnaTid;
    NS_ABORT_MSG_UNLESS(
        TypeId::LookupByNameFailSafe("ns3::SionnaRtSpectrumPropagationLossModel", &sionnaTid),
        "This example needs an ns-3 built with Sionna RT support; see rt.md for the configure "
        "line. For the statistical channel, use leo-link --payload=regenerative instead.");

    for (const auto& bwp : allBwps)
    {
        ObjectFactory sionnaFactory;
        sionnaFactory.SetTypeId(sionnaTid);
        auto sionna = sionnaFactory.Create<PhasedArraySpectrumPropagationLossModel>();

        // The carrier frequency has to go in through the ChannelModel pointer attribute. The loss
        // model has no Frequency attribute of its own and its channel model is a member rather
        // than an aggregate, so SetAttributeFailSafe on the loss model and GetObject of the
        // channel model both silently do nothing, leaving it at its 500 MHz default. That is
        // below the 1 GHz floor of the scenes' ITU materials, and Sionna aborts the run.
        {
            PointerValue pv;
            sionna->GetAttribute("ChannelModel", pv);
            auto channelModel = pv.Get<Object>();
            NS_ABORT_MSG_UNLESS(channelModel, "Sionna loss model has no channel model");
            channelModel->SetAttributeFailSafe("Frequency",
                                               DoubleValue(bwp.get()->m_centralFrequency));
            channelModel->SetAttributeFailSafe("RfBandwidth",
                                               DoubleValue(bwp.get()->m_channelBandwidth));
        }

        auto delay = CreateObject<NtnTrueRangeDelayModel>();
        delay->SetProxy(ntnProxyMobility);

        auto rangeLoss = CreateObject<NtnRangeCorrectionLossModel>();
        rangeLoss->SetProxy(ntnProxyMobility);
        rangeLoss->SetAntennaGainDb(vsatAntennaGainDb + satAntennaGainDb);

        auto channel = CreateObject<MultiModelSpectrumChannel>();
        channel->AddPhasedArraySpectrumPropagationLossModel(sionna);
        channel->AddPropagationLossModel(rangeLoss);
        channel->SetPropagationDelayModel(delay);
        bwp.get()->SetChannel(channel);
    }
    std::cout << "Sionna RT link: scene '" << sionnaScene << "', terminal " << sionnaTerminal
              << " m, proxy radius " << sionnaProxyKm << " km" << std::endl;

    Ptr<ChannelConditionModel> channelConditionModel;
    Ptr<SpectrumChannel> spectrumChannel = dlBand.GetBwpAt(0, 0)->GetChannel();
    if (spectrumChannel && spectrumChannel->GetPropagationLossModel())
    {
        Ptr<ThreeGppPropagationLossModel> propagationLossModel =
            DynamicCast<ThreeGppPropagationLossModel>(spectrumChannel->GetPropagationLossModel());
        if (propagationLossModel)
        {
            channelConditionModel = propagationLossModel->GetChannelConditionModel();
        }
    }

    // Static BWP manager: the default QoS flow goes to the DL carrier; the gNB advertises the
    // DL/UL pairing to the UE, which routes its uplink to BWP 1.
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(0));

    // Configure ideal beamforming method
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

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

    // Configure scheduler (TDMA-RR by default; OFDMA variants selectable via --scheduler)
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(schedulerType));
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(enableHarqReTx));
    nrHelper->SetGnbMacAttribute("NumHarqProcess", UintegerValue(numHarqProcess));
    nrHelper->SetUeMacAttribute("NumHarqProcess", UintegerValue(numHarqProcess));
    nrHelper->SetGnbPhyAttribute("N1Delay", UintegerValue(n1Delay));
    nrHelper->SetGnbPhyAttribute("N2Delay", UintegerValue(n2Delay));
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(useFixedMcs));
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(useFixedMcs));
    if (useFixedMcs)
    {
        nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(fixedMcs));
        nrHelper->SetSchedulerAttribute("StartingMcsUl", UintegerValue(fixedMcs));
    }

    // Antennas for the UEs
    nrHelper->SetUeAntennaTypeId("ns3::UniformPlanarArray");
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(ueAntennaRows));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(ueAntennaColumns));
    nrHelper->SetUeAntennaAttribute("NumVerticalPorts", UintegerValue(ueNumVerticalPorts));
    nrHelper->SetUeAntennaAttribute("NumHorizontalPorts", UintegerValue(ueNumHorizontalPorts));
    nrHelper->SetUeAntennaAttribute("IsDualPolarized", BooleanValue(ueIsDualPolarized));
    nrHelper->SetUeAntennaAttribute("PolSlantAngle",
                                    DoubleValue(LocalDegreesToRadians(uePolSlantAngleDeg)));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
                                        "Gain",
                                        DoubleValue(vsatAntennaGainDb))));

    // Antennas for the gNbs
    nrHelper->SetGnbAntennaTypeId("ns3::UniformPlanarArray");
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(gnbAntennaRows));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(gnbAntennaColumns));
    nrHelper->SetGnbAntennaAttribute("NumVerticalPorts", UintegerValue(gnbNumVerticalPorts));
    nrHelper->SetGnbAntennaAttribute("NumHorizontalPorts", UintegerValue(gnbNumHorizontalPorts));
    nrHelper->SetGnbAntennaAttribute("IsDualPolarized", BooleanValue(gnbIsDualPolarized));
    nrHelper->SetGnbAntennaAttribute("PolSlantAngle",
                                     DoubleValue(LocalDegreesToRadians(gnbPolSlantAngleDeg)));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
                                         "Gain",
                                         DoubleValue(satAntennaGainDb))));

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(10 * 1024 * 1024));
    Config::SetDefault("ns3::NrRlcUm::ReorderingTimer",
                       TimeValue(MilliSeconds(rlcReorderingTimerMs)));

    // Install nr net devices
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(satellites, allBwps);
    // FDD slot patterns: BWP 0 carries only downlink slots, BWP 1 only uplink slots. The gNB
    // pairs the UL-only carrier with the DL carrier when the cell is configured at attach.
    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {
        NrHelper::GetGnbPhy(gnbNetDev.Get(i), 0)
            ->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
        NrHelper::GetGnbPhy(gnbNetDev.Get(i), 1)
            ->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));
    }
    // Install UEs individually so each carries its own spot-beam footprint gain taper.
    // The satellite spot-beam is modelled as a per-UE effective gain offset that falls off with
    // radial distance r from the beam centre: Delta = -min(3*(r/R)^2, 20) dB -> 0 dB at the centre,
    // -3 dB at the footprint edge (the half-power contour). It rides on the UE antenna gain, which
    // is symmetric to reducing the satellite gain toward that UE (so it affects both UL and DL).
    NetDeviceContainer groundNodeNetDev;
    Vector satEcefForReport = firstSatelliteMobility->GetPosition();
    Vector beamCentreEcef = groundNodeContainer.Get(0)->GetObject<MobilityModel>()->GetPosition();
    // Element handles captured here are updated at runtime by UpdateSpotBeamTaper (when
    // beamHpbwDeg>0).
    std::vector<Ptr<IsotropicAntennaModel>> ueBeamElements;
    for (uint32_t i = 0; i < groundNodeContainer.GetN(); ++i)
    {
        Ptr<MobilityModel> ueMobForReport = groundNodeContainer.Get(i)->GetObject<MobilityModel>();
        double taperDb = 0.0;
        if (beamHpbwDeg > 0.0)
        {
            // Geometry-driven 3GPP/ITU spot-beam pattern, evaluated at the initial geometry; the
            // runtime updater keeps it current (elevation-dependent beam distortion) over the pass.
            taperDb = SpotBeamTaperDb(satEcefForReport,
                                      beamCentreEcef,
                                      ueMobForReport->GetPosition(),
                                      beamHpbwDeg,
                                      beamMaxAttenDb);
        }
        else if (footprintRadiusKm > 0.0)
        {
            const double rr = ueRadialKm[i] / footprintRadiusKm;
            taperDb = -std::min(3.0 * rr * rr, 20.0);
        }
        Ptr<IsotropicAntennaModel> ueElement = CreateObjectWithAttributes<IsotropicAntennaModel>(
            "Gain",
            DoubleValue(vsatAntennaGainDb + taperDb));
        ueBeamElements.push_back(ueElement);
        nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(ueElement));
        groundNodeNetDev.Add(
            nrHelper->InstallUeDevice(NodeContainer(groundNodeContainer.Get(i)), allBwps));
        double slantKm = (satEcefForReport - ueMobForReport->GetPosition()).GetLength() / 1000.0;
        std::cout << "UE " << i << " footprint: radial=" << ueRadialKm[i]
                  << " km, gainTaper=" << taperDb
                  << " dB, effUeGain=" << (vsatAntennaGainDb + taperDb)
                  << " dBi, initialSlant=" << slantKm << " km" << std::endl;
    }

    // Show the ray tracer the scene frame while everything else keeps the real Earth-centred
    // geometry: the proxy and the terminal go on the spectrum PHYs, not on the nodes.
    {
        std::vector<double> tv;
        std::stringstream ts(sionnaTerminal);
        std::string tok;
        while (std::getline(ts, tok, ':'))
        {
            tv.push_back(std::stod(tok));
        }
        NS_ABORT_MSG_UNLESS(tv.size() == 3, "--sionnaTerminal must be 'x:y:z' in metres");
        const Vector terminalScene(tv[0], tv[1], tv[2]);

        auto satMobility = satellites.Get(0)->GetObject<MobilityModel>();
        auto observerMobility = groundNodeContainer.Get(0)->GetObject<MobilityModel>();
        ntnProxyMobility->Setup(satMobility,
                                observerMobility->GetPosition(),
                                terminalScene,
                                sionnaProxyKm * 1000.0);

        auto terminalMobility = CreateObject<ConstantPositionMobilityModel>();
        terminalMobility->SetPosition(terminalScene);

        // The beamforming reaches from a mobility model back to its Node, so these stand-ins need
        // one. They carry a position and nothing else; no device, no stack.
        CreateObject<Node>()->AggregateObject(ntnProxyMobility);
        CreateObject<Node>()->AggregateObject(terminalMobility);

        for (uint32_t b = 0; b < allBwps.size(); ++b)
        {
            for (uint32_t d = 0; d < gnbNetDev.GetN(); ++d)
            {
                Ptr<NrGnbPhy> gnbPhy = nrHelper->GetGnbPhy(gnbNetDev.Get(d), b);
                gnbPhy->GetSpectrumPhy()->SetMobility(ntnProxyMobility);
                if (auto delay = DynamicCast<NtnTrueRangeDelayModel>(
                        gnbPhy->GetSpectrumPhy()->GetSpectrumChannel()->GetPropagationDelayModel()))
                {
                    delay->SetGnbPhy(gnbPhy);
                }
            }
            for (uint32_t d = 0; d < groundNodeNetDev.GetN(); ++d)
            {
                nrHelper->GetUePhy(groundNodeNetDev.Get(d), b)
                    ->GetSpectrumPhy()
                    ->SetMobility(terminalMobility);
            }
        }
        std::cout << "Sionna RT: proxy " << sionnaProxyKm << " km along the true line of sight, "
                  << "true range " << ntnProxyMobility->GetTrueRangeM() / 1000.0 << " km at t=0"
                  << std::endl;
    }

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(groundNodeNetDev, randomStream);

    // TxPower dBm = EIRP_dBm - antenna gain, so that TxPower + AntennaGain = EIRP.
    // EIRP_dBm = satEIRP (dBW/MHz) + 30 + 10*log10(bandwidth_MHz)
    double satTxPower = (satEIRP + 30) + (10 * std::log10(bandwidthHz / 1e6)) - satAntennaGainDb;

    for (uint32_t i = 0; i < gnbNetDev.GetN(); i++)
    {
        for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
        {
            nrHelper->GetGnbPhy(gnbNetDev.Get(i), bwp)->SetTxPower(satTxPower);
        }
    }

    for (uint32_t i = 0; i < groundNodeNetDev.GetN(); ++i)
    {
        for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
        {
            nrHelper->GetUePhy(groundNodeNetDev.Get(i), bwp)->SetTxPower(groundTxPower);
        }
    }

    // Create the internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    auto [remoteHost, remoteHostIpv4Address] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, MilliSeconds(remoteHostDelayMs));

    InternetStackHelper internet;
    internet.Install(groundNodeContainer);

    Ipv4InterfaceContainer ueIpInterface;
    ueIpInterface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(groundNodeNetDev));

    // Assign IP address to UEs, and install downlink applications.
    // udpRateMbps is the per-UE offered load; aggregate offered load is numUes x rate.
    const uint16_t dlPortBase = 1234;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;

    for (uint32_t i = 0; i < groundNodeContainer.GetN(); ++i)
    {
        const uint16_t dlPort = dlPortBase + i;
        Ptr<Node> ueNode = groundNodeContainer.Get(i);
        if (transport == "tcp")
        {
            PacketSinkHelper dlPacketSinkHelper("ns3::TcpSocketFactory",
                                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            serverApps.Add(dlPacketSinkHelper.Install(ueNode));

            BulkSendHelper dlClientHelper("ns3::TcpSocketFactory",
                                          InetSocketAddress(ueIpInterface.GetAddress(i), dlPort));
            dlClientHelper.SetAttribute("MaxBytes", UintegerValue(0)); // unlimited data
            dlClientHelper.SetAttribute("SendSize", UintegerValue(1500));
            clientApps.Add(dlClientHelper.Install(remoteHost));
        }
        else
        {
            PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            serverApps.Add(dlPacketSinkHelper.Install(ueNode));

            if (udpRateMbps <= 0.0)
            {
                Ptr<BackloggedUdpSourceApplication> backloggedUdp =
                    CreateObject<BackloggedUdpSourceApplication>();
                backloggedUdp->Setup(InetSocketAddress(ueIpInterface.GetAddress(i), dlPort),
                                     udpPacketSize);
                remoteHost->AddApplication(backloggedUdp);
                clientApps.Add(backloggedUdp);
            }
            else
            {
                UdpClientHelper dlClientHelper(ueIpInterface.GetAddress(i), dlPort);
                dlClientHelper.SetAttribute("PacketSize", UintegerValue(udpPacketSize));
                dlClientHelper.SetAttribute("MaxPackets",
                                            UintegerValue(std::numeric_limits<uint32_t>::max()));

                double packetsPerSecond =
                    (udpRateMbps * 1e6) / (8.0 * static_cast<double>(udpPacketSize));
                Time interval = Seconds(1.0 / std::max(1.0, packetsPerSecond));
                dlClientHelper.SetAttribute("Interval", TimeValue(interval));
                clientApps.Add(dlClientHelper.Install(remoteHost));
            }
        }
    }

    // Attach UEs to the closest gNB
    nrHelper->AttachToClosestGnb(groundNodeNetDev, gnbNetDev);

    // Ideal timing advance: each UE advances its uplink by its one-way delay to the satellite
    // it attached to, re-read from the geometry before every slot as the pass progresses.
    std::vector<Time> initialTimingAdvance;
    for (uint32_t i = 0; i < groundNodeNetDev.GetN(); ++i)
    {
        initialTimingAdvance.push_back(
            ComputeRegenerativeTimingAdvance(groundNodeNetDev.Get(i), ueMobilities[i], gnbNetDev));
        auto ueDev = DynamicCast<NrUeNetDevice>(groundNodeNetDev.Get(i));
        for (uint32_t bwp = 0; ntnTiming && bwp < ueDev->GetCcMapSize(); ++bwp)
        {
            // A lambda, since MakeBoundCallback needs comparable arguments and a
            // NetDeviceContainer is not
            NrHelper::GetUePhy(ueDev, bwp)
                ->SetTimingAdvanceCallback(
                    Callback<Time>([ueDev, ueMobility = ueMobilities[i], gnbNetDev]() {
                        return ComputeRegenerativeTimingAdvance(ueDev, ueMobility, gnbNetDev);
                    }));
        }
    }
    for (uint32_t i = 0; i < groundNodeNetDev.GetN(); ++i)
    {
        std::cout << "UE " << i << " timing advance (one-way delay at t=0): "
                  << initialTimingAdvance[i].GetMicroSeconds() / 1000.0 << " ms"
                  << (ntnTiming ? " (applied, refreshed every slot)"
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
            timingOut << "architecture\tregenerative\n";
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

    // Start server and client apps
    serverApps.Start(MilliSeconds(400));
    clientApps.Start(MilliSeconds(400));
    serverApps.Stop(Seconds(duration));
    clientApps.Stop(Seconds(duration) - MilliSeconds(200));

    // Enable the traces provided by the nr module.
    nrHelper->EnableTraces();

    // Get all ground station antennas (every BWP) for channel invalidation
    std::vector<Ptr<PhasedArrayModel>> groundAntennas;
    std::vector<std::vector<Ptr<PhasedArrayModel>>> groundAntennasPerUe;
    for (uint32_t i = 0; i < groundNodeNetDev.GetN(); ++i)
    {
        auto ueNetDevice = groundNodeNetDev.Get(i)->GetObject<NrUeNetDevice>();
        std::vector<Ptr<PhasedArrayModel>> arrays;
        for (uint32_t bwp = 0; bwp < ueNetDevice->GetCcMapSize(); ++bwp)
        {
            arrays.push_back(DynamicCast<PhasedArrayModel>(
                ueNetDevice->GetPhy(bwp)->GetSpectrumPhy()->GetAntenna()));
            groundAntennas.push_back(arrays.back());
        }
        groundAntennasPerUe.push_back(arrays);
    }

    // Schedule UpdateAntennaOrientation events for every sat. node to update antenna orientation
    // Now also invalidates channel matrix on each mobility update
    for (uint32_t i = 0; i < gnbNetDev.GetN(); i++)
    {
        auto satNetDevice = gnbNetDev.Get(i);
        auto gnbNetDevice = satNetDevice->GetObject<NrGnbNetDevice>();
        std::vector<Ptr<UniformPlanarArray>> satAntennas;
        for (uint32_t bwp = 0; bwp < gnbNetDevice->GetCcMapSize(); ++bwp)
        {
            auto spectrumPhy = gnbNetDevice->GetPhy(bwp)->GetSpectrumPhy();
            satAntennas.push_back(spectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>());
        }
        Ptr<Node> satNode = satNetDevice->GetNode();
        // The mobility trace reads the pointing of the DL array from the node.
        satNode->AggregateObject(satAntennas.front());
        // Pass ground antennas so we can invalidate channel on both ends
        UpdateAntennaOrientation(satNode, satAntennas, groundAntennas, MilliSeconds(precision));

        // Geometry-driven spot-beam taper: update each UE's effective gain from its live
        // off-boresight angle so the footprint distorts with elevation over the pass.
        if (beamHpbwDeg > 0.0)
        {
            UpdateSpotBeamTaper(satNode->GetObject<MobilityModel>(),
                                groundNodeContainer.Get(0)->GetObject<MobilityModel>(),
                                ueMobilities,
                                ueBeamElements,
                                groundAntennasPerUe,
                                vsatAntennaGainDb,
                                beamHpbwDeg,
                                beamMaxAttenDb,
                                MilliSeconds(precision));
        }
    }

    std::streambuf* coutbuf = std::cout.rdbuf();

    // Redirect cout if traceFile is specified
    std::ofstream out;
    out.open(traceFile);
    if (out.is_open())
    {
        Config::ConnectWithoutContextFailSafe("/NodeList/*/$ns3::MobilityModel/CourseChange",
                                              MakeCallback(&CourseChange));
        std::cout.rdbuf(out.rdbuf());
        std::cout << "Time:Satellite:x:y:z:x_2:y_2:z_2:elevation_deg:slant_distance_m" << std::endl;
    }

    std::ofstream channelConditionOut;
    if (!channelConditionTraceFile.empty())
    {
        channelConditionOut.open(channelConditionTraceFile);
        if (channelConditionOut.is_open())
        {
            channelConditionOut << "Time\tis_los\tcondition" << std::endl;
            LogChannelCondition(channelConditionModel,
                                firstSatelliteMobility,
                                groundNodeMobility,
                                stopCheckPeriod,
                                &channelConditionOut);
        }
    }

    // In overhead mode the satellite starts at ~90° and moves toward the horizon; in full-pass
    // mode it starts at the boundary and rises. Either way the run stops once the satellite
    // descends to the elevation boundary so that near-horizon / below-horizon conditions don't
    // corrupt the protocol stack (hasEnteredPass=true: the first check is one period in).
    StopAtElevationBoundary(firstSatelliteMobility,
                            observerBoundaryElevationDeg,
                            stopCheckPeriod,
                            true);

    Simulator::Stop(Time(Seconds(duration)));
    Simulator::Run();

    out.close();
    channelConditionOut.close();
    std::cout.rdbuf(coutbuf);

    // Get amount of bytes received
    uint64_t receivedPackets = 0;
    for (uint32_t i = 0; i < serverApps.GetN(); ++i)
    {
        Ptr<PacketSink> serverPacketSink = serverApps.Get(i)->GetObject<PacketSink>();
        const uint64_t ueBytes = serverPacketSink->GetTotalRx();
        receivedPackets += ueBytes;
        std::cout << "UE " << i << " received " << ueBytes << " bytes" << std::endl;
    }
    std::cout << "Received " << receivedPackets << " bytes (all UEs)" << std::endl;

    Simulator::Destroy();

    if (receivedPackets > 0)
    {
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
