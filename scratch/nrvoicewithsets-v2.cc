/*
 * ntnlab_nr.cc  --  ntnlab_n.cc with the LTE cell replaced by an NR-NTN cell (first, simple version)
 *
 *   UE i ~~ service link ~~ LEO satellite ~~ feeder link ~~ gNB at the gateway === S1-U === SGW/PGW
 *   7.0.0.(i+2)             600 km, transparent                                             ||
 *                                                                    remote host 1.0.0.2 == 10 Gbps, 10 ms
 *
 * Needs ns-3-NTN v1.1.0: ns-3.48 with the NTN changes, and nr-ntn-dev v1.1.0 in contrib/nr.
 * Put this file in scratch/.
 *
 * Kept from ntnlab_n.cc
 *   - one remote host, N calls, one NGMN VoIP generator per call, per-call voice activity factor
 *   - a dedicated voice flow (5QI 1) selected by the UDP port, 40 kbit/s GBR
 *   - FlowMonitor gives one line per call; the UE PHY gives the DL SINR
 *
 * What replaced the LTE cell
 *   - the radio settings are those of contrib/nr/examples/leo-link.cc with its default preset
 *     (TR 38.821 study case 9: LEO-600, S-band 2 GHz, FDD, 34 dBW/MHz, handheld UE), including
 *     K_offset, the timing advance and the feeder link of a transparent payload
 *   - --satSet=2 swaps in the Set-2 satellite of TR 38.821, --altitudeKm=1200 the LEO-1200 one;
 *     altitude, carriers, EIRP density, antenna gain, G/T and beamwidth are flags of their own
 *   - "distance" became "elevation", the elevation of the satellite seen from UE 0
 *   - UE i sits i x --spacingKm from the beam centre (UE 0) and loses the beam gain of that offset
 *
 * The satellite
 *   - --freezeGeometry=1 (default): it stands still at --elevation but reports its orbital
 *     velocity, so the fading keeps moving. Elevation, range and delay are constant.
 *   - --freezeGeometry=0: it flies ns-3.48's circular orbit, as in leo-link. It comes from the
 *     north, is at --elevation when the calls start, passes exactly over UE 0 and goes down in
 *     the south. Elevation, range and delay change during the call, K_offset covers the longest
 *     round trip of the run, the timing advance follows the delay, and the beam keeps pointing
 *     at UE 0. The calls must end before the satellite is down at 10 deg again; the header says
 *     how long that is. --elevation=10 with that duration is the whole pass.
 *     The fading then behaves differently: ns-3.48's channel model draws a new fading realisation
 *     whenever the satellite has moved 1 m, here every 1 to 2 ms, and lets the shadowing drift
 *     with the distance flown. When the satellite stands still, both are drawn once per
 *     line-of-sight state and only the Doppler term moves the fading.
 *
 * Two tables, one line per call: the call itself, then what the UE saw of the satellite during
 * its call: elevation (- before the satellite is overhead, + after), slant range, and the
 * propagation delay of the whole path, remote host to UE. LOS[%] is the share of the call with
 * line of sight to the satellite, RLF the number of radio link failures of the UE.
 *
 * ROHC: needs the nr module with the ROHC patch (NrPdcp::EnableRohc). It is on when the module
 * was built with the ROHC library, --rohc=0 turns it off, --rohcReport=full prints the details.
 *
 * NOT here, because this NR module does not have it: PDCCH CCE budget.
 * Uplink calls run, but the handheld uplink of this release is far below its link budget.
 *
 * Run:
 *   ./ns3 run "ntnlab_nr"                              10 calls, downlink, 30 deg
 *   ./ns3 run "ntnlab_nr --elevation=60 --nUes=50 --spacingKm=0.4"
 *   ./ns3 run "ntnlab_nr --freezeGeometry=0 --duration=100s"   the satellite flies: 30 -> 72 deg
 *   ./ns3 run "ntnlab_nr --freezeGeometry=0 --elevation=10 --duration=508s --nUes=2"   whole pass
 *   ./ns3 run "ntnlab_nr --satSet=2"                   Set-2 satellite: 28 dBW/MHz, 24 dBi
 *   ./ns3 run "ntnlab_nr --altitudeKm=1200"            LEO-1200 of the set: 40 dBW/MHz
 *   ./ns3 run "ntnlab_nr --satEirpDbwPerMHz=31"        the set, with 3 dB less power
 *   ./ns3 run "ntnlab_nr --direction=ul"
 *   ./ns3 run "ntnlab_nr --transparent=0"              gNB on the satellite, no feeder link
 *   ./ns3 run "ntnlab_nr --scenario=NTN-Rural"         or NTN-Urban, NTN-DenseUrban
 *   ./ns3 run "ntnlab_nr --rohc=0"                     without header compression
 *   ./ns3 run "ntnlab_nr --nUes=1 --rohcReport=full"   every ROHC detail of the call
 *   ./ns3 run "ntnlab_nr --nrTraces=1" --cwd=results/t1   also write the NR trace files there
 */

#include "ns3/angles.h"
#include "ns3/applications-module.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/geocentric-ecef-mobility-model.h"
#include "ns3/geographic-positions.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/leo-circular-orbit-mobility-model.h"
#include "ns3/network-module.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-ntn-link-models.h"
#include "ns3/point-to-point-module.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/traffic-generator-helper.h"
#include "ns3/traffic-generator-ngmn-voip.h"
#include "ns3/uniform-planar-array.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace ns3;

namespace
{

std::vector<double> g_dlSinrSum; // one entry per UE, linear
std::vector<uint32_t> g_dlSinrN;

// "DlCtrlSinr" of UE i's downlink PHY: the SINR of every control symbol the UE receives, so it
// is there for uplink calls too. i is bound when connecting.
void
DlSinr(uint32_t ue, uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
{
    g_dlSinrSum[ue] += sinr;
    ++g_dlSinrN[ue];
}

// "RadioLinkFailure" of UE i's RRC: the UE gave the link up (out of sync for too long) and
// has to attach again. A call with failures is not a steady call any more.
std::vector<uint32_t> g_rlf;

void
RadioLinkFailure(uint32_t ue, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    ++g_rlf[ue];
}

// Elevation of the higher of two nodes seen from the lower one, in degrees (negative below the
// horizon). The formula of ns-3.48's GeocentricConstantPositionMobilityModel::DoGetElevationAngle(),
// which however takes the node with the smaller ECEF z as the one on the ground. That is right
// while the satellite is north of the UE and wrong once it has passed to the south: 30 deg is
// then reported as 37.7 deg. The 3GPP channel models read the elevation through that function,
// so every node of this example overrides it with this one.
double
ElevationDeg(const Vector& p, const Vector& q)
{
    const bool pIsLower = p.GetLength() < q.GetLength();
    const Vector& a = pIsLower ? p : q; // the node on the ground
    const Vector bMinusA = (pIsLower ? q : p) - a;
    const double x = (a * bMinusA) / (a.GetLength() * bMinusA.GetLength());
    return (180.0 * M_1_PI) * std::asin(std::max(-1.0, std::min(1.0, x)));
}

// A node at a fixed place: the UEs, the gateway, and the satellite under --freezeGeometry=1. The
// frozen satellite reports its orbital velocity (what leo-link does under --freezeGeometry):
// elevation, range and delay stay constant, the channel's Doppler term does not.
class FixedMobilityModel : public GeocentricEcefMobilityModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabFixedMobilityModel")
                                .SetParent<GeocentricEcefMobilityModel>()
                                .SetGroupName("Mobility")
                                .AddConstructor<FixedMobilityModel>();
        return tid;
    }

    void SetReportedVelocity(const Vector& velocityEcef)
    {
        m_velocity = velocityEcef;
    }

  private:
    Vector DoGetVelocity() const override
    {
        return m_velocity;
    }

    double DoGetElevationAngle(Ptr<const GeocentricConstantPositionMobilityModel> other) override
    {
        return std::abs(ElevationDeg(GetGeocentricPosition(), other->GetGeocentricPosition()));
    }

    Vector m_velocity{0.0, 0.0, 0.0};
};

// The satellite under --freezeGeometry=0: ns-3.48's circular orbit, the model leo-link flies
class OrbitingMobilityModel : public LeoCircularOrbitMobilityModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabOrbitingMobilityModel")
                                .SetParent<LeoCircularOrbitMobilityModel>()
                                .SetGroupName("Mobility")
                                .AddConstructor<OrbitingMobilityModel>();
        return tid;
    }

  private:
    double DoGetElevationAngle(Ptr<const GeocentricConstantPositionMobilityModel> other) override
    {
        return std::abs(ElevationDeg(GetGeocentricPosition(), other->GetGeocentricPosition()));
    }
};

// Where the satellite will be at time t, in ECEF. The orbit model only tells where it is now,
// and K_offset has to be known before the run. Same computation as the model's CalcPosition()
// and as leo-link's SatellitePredictor: a circle in the plane of the position and velocity at
// t = 0, turned back by the Earth's rotation. main() checks it against the model after the run.
struct OrbitPredictor
{
    Vector pos0;
    Vector vel0;             // (0, 0, 0): the satellite stands still
    double omegaRadPerS{0.0}; // angular velocity along the orbit

    Vector At(Time t) const
    {
        if (vel0.GetLength() <= 0.0)
        {
            return pos0;
        }
        Vector normal = CrossProduct(pos0, vel0);
        normal = normal * (1.0 / normal.GetLength());
        const double theta = omegaRadPerS * t.GetSeconds();
        const Vector orbital = pos0 * std::cos(theta) + CrossProduct(normal, pos0) * std::sin(theta);
        const double phi = 2.0 * M_PI * t.GetSeconds() / Days(1).GetSeconds();
        return Vector(orbital.x * std::cos(phi) + orbital.y * std::sin(phi),
                      -orbital.x * std::sin(phi) + orbital.y * std::cos(phi),
                      orbital.z);
    }
};

// Gain of the satellite beam towards a UE, relative to the beam centre, in dB (<= 0).
// TR 38.811 6.4.1 pattern, same function as leo-link's SpotBeamTaperDb().
double
BeamTaperDb(const Vector& satEcef, const Vector& centreEcef, const Vector& ueEcef, double hpbwDeg)
{
    const Vector vc = centreEcef - satEcef;
    const Vector vu = ueEcef - satEcef;
    double cosT = (vc.x * vu.x + vc.y * vu.y + vc.z * vu.z) / (vc.GetLength() * vu.GetLength());
    cosT = std::max(-1.0, std::min(1.0, cosT));
    const double sinTheta = std::sin(std::acos(cosT));
    if (sinTheta < 1e-12)
    {
        return 0.0;
    }
    const double ka = 1.616339 / std::sin(hpbwDeg / 2.0 * M_PI / 180.0);
    const double x = ka * sinTheta;
    const double f = j1(x) / x;
    return std::max(10.0 * std::log10(std::max(4.0 * f * f, 1e-12)), -30.0);
}

// What the functions below work on while the simulation runs
struct Scene
{
    Ptr<GeocentricConstantPositionMobilityModel> satellite;
    std::vector<Ptr<FixedMobilityModel>> ues;
    Ptr<NrNtnPropagationDelayModel> delay;
    Ptr<ChannelConditionModel> condition;                     // carrier of the voice direction
    std::vector<Ptr<UniformPlanarArray>> satArrays;           // one per BWP
    std::vector<Ptr<IsotropicAntennaModel>> ueElements;       // one per UE, both its BWPs use it
    std::vector<std::vector<Ptr<PhasedArrayModel>>> ueArrays; // per UE, one per BWP
    double ueAntennaGainDbi{0.0};
    double beamHpbwDeg{0.0};
    bool flying{false};      // --freezeGeometry=0
    OrbitPredictor orbit;    // where the satellite will be
    double groundDelayMs{0}; // remote host to gNB: the N6 link
};
Scene g_scene;

// First, last, smallest and largest value of something sampled during a call
struct Span
{
    double first{NAN};
    double last{NAN};
    double min{NAN};
    double max{NAN};
    double sum{0.0};
    uint32_t n{0};

    void Add(double v)
    {
        if (n++ == 0)
        {
            first = min = max = v;
        }
        last = v;
        min = std::min(min, v);
        max = std::max(max, v);
        sum += v;
    }
};

// What a UE saw of the satellite during its call
struct CallView
{
    Time callStart;
    Time callEnd;
    Span elevationDeg;
    double sideAtStart{0.0}; // -1: the satellite is still coming closer, +1: it is moving away,
    double sideAtEnd{0.0};   //  0: it stands still
    Span rangeKm;        // satellite to UE
    Span delayMs;        // one way, remote host to UE: N6 link + feeder link (transparent) + service link
    Span taperDb;        // beam gain towards the UE, relative to the beam centre
    uint32_t losYes{0};  // line-of-sight samples of the call: how many said yes ...
    uint32_t losAll{0};  // ... out of how many
};
std::vector<CallView> g_view;

// Where the satellite is for UE i, now. The elevation is the one the channel model uses. The
// side says whether the satellite is still coming closer to this UE (before it is overhead) or
// moving away again (after).
void
RecordView(uint32_t i)
{
    const Vector sat = g_scene.satellite->GetPosition();
    const Vector ue = g_scene.ues[i]->GetPosition();
    CallView& view = g_view[i];
    if (g_scene.flying)
    {
        const Time now = Simulator::Now();
        const bool closer = (g_scene.orbit.At(now + MilliSeconds(1)) - ue).GetLength() <
                            (g_scene.orbit.At(now) - ue).GetLength();
        view.sideAtEnd = closer ? -1.0 : 1.0;
        if (view.elevationDeg.n == 0)
        {
            view.sideAtStart = view.sideAtEnd;
        }
    }
    view.elevationDeg.Add(g_scene.satellite->GetElevationAngle(g_scene.ues[i]));
    view.rangeKm.Add((sat - ue).GetLength() / 1000.0);
    view.delayMs.Add(
        g_scene.groundDelayMs +
        g_scene.delay->GetOneWayDelay(g_scene.satellite, g_scene.ues[i]).GetSeconds() * 1000.0);
    view.taperDb.Add(BeamTaperDb(sat, g_scene.ues[0]->GetPosition(), ue, g_scene.beamHpbwDeg));
}

// Every 100 ms of UE i's call, its first and last instant included
void
SampleCall(uint32_t i)
{
    RecordView(i);
    const CallView& view = g_view[i];
    const Time left = view.callEnd - Simulator::Now();
    if (left.IsStrictlyPositive())
    {
        Simulator::Schedule(std::min(left, Time(MilliSeconds(100))), &SampleCall, i);
    }
}

// Whether each UE has line of sight to the satellite, asked every 100 ms on the carrier that
// carries the voice and counted while the UE's call is on. The channel redraws the state once per
// second (see below). One tick for all UEs, in UE order: the question can be the one that makes
// the channel redraw, so who asks when decides the order of the random draws.
void
SampleLos()
{
    for (std::size_t i = 0; i < g_scene.ues.size(); ++i)
    {
        const bool los =
            g_scene.condition->GetChannelCondition(g_scene.satellite, g_scene.ues[i])->IsLos();
        CallView& view = g_view[i];
        if (Simulator::Now() >= view.callStart && Simulator::Now() <= view.callEnd)
        {
            ++view.losAll;
            view.losYes += los;
        }
    }
    Simulator::Schedule(MilliSeconds(100), &SampleLos);
}

// The satellite turns its arrays (one per BWP) towards the beam centre, UE 0. The channel model
// works in ECEF, so the two angles are ECEF angles.
void
PointBeam()
{
    const Angles toCentre(g_scene.ues[0]->GetPosition(), g_scene.satellite->GetPosition());
    for (auto& array : g_scene.satArrays)
    {
        array->SetAlpha(toCentre.GetAzimuth());
        array->SetBeta(toCentre.GetInclination());
        array->SetBeamformingVector(array->GetBeamformingVector());
    }
}

// --freezeGeometry=0 only, every 100 ms: the beam follows UE 0 and every UE's antenna gain gets
// the beam taper of the new geometry, as leo-link's UpdateAntennaOrientation() and
// UpdateSpotBeamTaper() do. Setting the beamforming vector again drops the cached channel.
void
FollowBeam()
{
    PointBeam();
    const Vector sat = g_scene.satellite->GetPosition();
    const Vector centre = g_scene.ues[0]->GetPosition();
    for (std::size_t i = 0; i < g_scene.ues.size(); ++i)
    {
        const double taperDb =
            BeamTaperDb(sat, centre, g_scene.ues[i]->GetPosition(), g_scene.beamHpbwDeg);
        g_scene.ueElements[i]->SetAttribute("Gain",
                                            DoubleValue(g_scene.ueAntennaGainDbi + taperDb));
        for (auto& array : g_scene.ueArrays[i])
        {
            array->SetBeamformingVector(array->GetBeamformingVector());
        }
    }
    Simulator::Schedule(MilliSeconds(100), &FollowBeam);
}

// Connect a trace and stop with a clear message if the name is wrong.
void
Connect(Ptr<Object> obj, const std::string& name, const CallbackBase& cb)
{
    bool ok = obj->TraceConnectWithoutContext(name, cb);
    NS_ABORT_MSG_UNLESS(ok,
                        "No trace source called \"" << name << "\" on "
                                                    << obj->GetInstanceTypeId().GetName());
}

// One channel per FDD carrier, built as leo-link's CreateNtnCarrier() builds it: the 3GPP
// TR 38.811 model of --scenario between satellite and UE, the propagation delay of the real
// path, and for a transparent payload the feeder link on top.
struct NtnCarrier
{
    Ptr<SpectrumChannel> channel;
    Ptr<PropagationLossModel> serviceLoss;
    Ptr<NrNtnFeederNoiseLossModel> feeder;
};

NtnCarrier
CreateNtnCarrier(double frequencyHz,
                 const std::string& scenario,
                 Ptr<MobilityModel> gateway,
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
    if (Ptr<PhasedArraySpectrumPropagationLossModel> fading =
            carrier.channel->GetPhasedArraySpectrumPropagationLossModel())
    {
        PointerValue channelModel;
        if (fading->GetAttributeFailSafe("ChannelModel", channelModel))
        {
            channelModel.Get<Object>()->SetAttributeFailSafe("Frequency", DoubleValue(frequencyHz));
        }
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
    carrier.feeder->SetFeeder(gateway, feederLoss);
    if (transparent)
    {
        carrier.channel->SetAttribute("PropagationLossModel", PointerValue(carrier.feeder));
    }
    return carrier;
}

// The LEO satellites of TR 38.821 Table 6.1.1.1-1 (Set-1) and Table 6.1.1.1-2 (Set-2), S-band
// rows. Set-2 is the smaller antenna (1 m instead of 2 m): 6 dB less gain and EIRP, a beam twice
// as wide. The tables also have a GEO column and Ka-band rows; this example is set up for
// neither (GEO needs other HARQ and RLC settings, Ka-band the VSAT terminal).
struct SatelliteColumn
{
    const char* orbit;
    double altitudeKm;
    double eirpDbwPerMHz; // satellite EIRP density
    double gainDbi;       // satellite Tx max gain = Rx max gain
    double hpbwDeg;       // 3 dB beamwidth
    double gtDbPerK;      // G/T
};
const SatelliteColumn g_satelliteSets[2][2] = {
    {{"LEO-600", 600.0, 34.0, 30.0, 4.4127, 1.1}, {"LEO-1200", 1200.0, 40.0, 30.0, 4.4127, 1.1}},
    {{"LEO-600", 600.0, 28.0, 24.0, 8.8320, -4.9}, {"LEO-1200", 1200.0, 34.0, 24.0, 8.8320, -4.9}},
};

} // namespace

int
main(int argc, char* argv[])
{
    uint32_t nUes = 10;
    double elevation = 30.0; // of the satellite, seen from UE 0 (deg): always, or when the calls start
    bool freezeGeometry = true; // false: the satellite flies its orbit
    double spacingKm = 2.0;  // UE i is i x spacingKm from the beam centre
    Time duration = Seconds(20);
    std::string direction = "dl";
    std::string scenario = "NTN-Suburban";
    bool transparent = true; // false: regenerative, the gNB is on the satellite
    std::string scheduler = "ns3::NrMacSchedulerTdmaRR"; // leo-link's default
    bool nrTraces = false; // true: NrHelper::EnableTraces(), about 25 .txt files in the run folder
    bool rohc = NrPdcp::IsRohcAvailable(); // on whenever the nr module was built with the ROHC library
    std::string rohcReport = "summary";    // none, summary (one line per call) or full
    double bandwidthMHz = 30.0;
    // The satellite. --satSet and --altitudeKm pick a column of the TR 38.821 table above; the
    // four values left at NaN are then taken from that column.
    uint32_t satSet = 1;
    double altitudeKm = 600.0;
    double dlFrequencyGHz = 2.0;
    double ulFrequencyGHz = 0.0; // 0 = 190 MHz below the downlink
    double satEirpDbwPerMHz = NAN;
    double satAntennaGainDbi = NAN;
    double satGtDbPerK = NAN;
    double beamHpbwDeg = NAN;
    double gatewayTxPowerDbm = 60.0; // feeder link, transparent only (leo-link's defaults)
    double satFeederRxGainDbi = 30.0;
    Time internetDelay = MilliSeconds(10);
    uint16_t port = 4000;
    uint32_t startOffsetMs = 1; // call i starts at 1 s + i x startOffsetMs
    double vafMean = 0.45; // voice activity factor of each call ~ Gaussian(vafMean, vafStd)
    double vafStd = 0.15;  // 0 = every call gets vafMean

    CommandLine cmd(__FILE__);
    cmd.AddValue("nUes", "Number of UEs, one call each", nUes);
    cmd.AddValue("elevation",
                 "Elevation of the satellite seen from UE 0 (deg), 10 to 90; when it flies, the "
                 "elevation at the moment the calls start",
                 elevation);
    cmd.AddValue("freezeGeometry",
                 "1 = the satellite stands still at --elevation, 0 = it flies: at --elevation when "
                 "the calls start, then over UE 0 and down on the other side",
                 freezeGeometry);
    cmd.AddValue("spacingKm", "UE i is i x spacingKm east of the beam centre (km)", spacingKm);
    cmd.AddValue("duration", "How long each call lasts", duration);
    cmd.AddValue("direction", "ul = UEs talk to the remote host, dl = remote host talks to UEs", direction);
    cmd.AddValue("scenario", "NTN-Suburban, NTN-Rural, NTN-Urban or NTN-DenseUrban", scenario);
    cmd.AddValue("transparent", "1 = gNB at the gateway (feeder link), 0 = gNB on the satellite", transparent);
    cmd.AddValue("bandwidthMHz", "Bandwidth of each FDD carrier (MHz)", bandwidthMHz);
    cmd.AddValue("satSet",
                 "Satellite of TR 38.821, S-band: 1 = Set-1 (Table 6.1.1.1-1), 2 = Set-2 "
                 "(Table 6.1.1.1-2)",
                 satSet);
    cmd.AddValue("altitudeKm",
                 "Altitude of the satellite (km), 300 to 2000. 600 and 1200 are the LEO-600 and "
                 "LEO-1200 columns of the set; another altitude takes the nearer of the two",
                 altitudeKm);
    cmd.AddValue("dlFrequencyGHz", "Downlink carrier (GHz), 0.6 to 6", dlFrequencyGHz);
    cmd.AddValue("ulFrequencyGHz", "Uplink carrier (GHz), 0 = 190 MHz below the downlink", ulFrequencyGHz);
    cmd.AddValue("satEirpDbwPerMHz", "Satellite EIRP density (dBW/MHz), default: the set", satEirpDbwPerMHz);
    cmd.AddValue("satAntennaGainDbi", "Satellite Tx and Rx antenna gain (dBi), default: the set", satAntennaGainDbi);
    cmd.AddValue("satGtDbPerK", "Satellite G/T (dB/K), default: the set", satGtDbPerK);
    cmd.AddValue("beamHpbwDeg", "3 dB width of the satellite beam (deg), default: the set", beamHpbwDeg);
    cmd.AddValue("gatewayTxPowerDbm", "Feeder link: power the gateway radiates (dBm)", gatewayTxPowerDbm);
    cmd.AddValue("satFeederRxGainDbi", "Feeder link: satellite receive antenna gain (dBi)", satFeederRxGainDbi);
    cmd.AddValue("scheduler", "MAC scheduler TypeId, e.g. ns3::NrMacSchedulerOfdmaRR", scheduler);
    cmd.AddValue("nrTraces", "Write the NR module's PHY/MAC/RLC/PDCP trace files", nrTraces);
    cmd.AddValue("rohc", "Compress the IP/UDP headers in PDCP with ROHC", rohc);
    cmd.AddValue("rohcReport", "ROHC report at the end: none, summary or full", rohcReport);
    cmd.AddValue("internetDelay", "Delay of the PGW to remote host link", internetDelay);
    cmd.AddValue("startOffsetMs", "Call i starts at 1 s + i x startOffsetMs", startOffsetMs);
    cmd.AddValue("vafMean", "Mean of the per-call voice activity factor", vafMean);
    cmd.AddValue("vafStd", "Standard deviation of the per-call voice activity factor", vafStd);
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_UNLESS(direction == "ul" || direction == "dl", "--direction must be ul or dl");
    NS_ABORT_MSG_UNLESS(elevation >= 10.0 && elevation <= 90.0,
                        "--elevation must be 10 to 90 (the 3GPP NTN channel stops at 10)");
    NS_ABORT_MSG_UNLESS(nUes >= 1, "--nUes must be at least 1");
    const bool ul = (direction == "ul");
    const std::string transportProtocol = "ns3::UdpSocketFactory";

    // One VAF per call, drawn from a Gaussian truncated to the generator's range [0, 0.99]
    // (values outside are drawn again). Change --RngRun to get a different set.
    Ptr<NormalRandomVariable> gauss = CreateObject<NormalRandomVariable>();
    gauss->SetAttribute("Mean", DoubleValue(vafMean));
    gauss->SetAttribute("Variance", DoubleValue(vafStd * vafStd));
    gauss->SetStream(500);
    std::vector<double> vaf(nUes);
    for (uint32_t i = 0; i < nUes; ++i)
    {
        do
        {
            vaf[i] = gauss->GetValue();
        } while (vaf[i] < 0.0 || vaf[i] > 0.99);
    }

    // ROHC in every PDCP entity of a data bearer, gNB and UE (unidirectional mode, no feedback)
    Config::SetDefault("ns3::NrPdcp::EnableRohc", BooleanValue(rohc));
    GlobalValue::Bind("NrRohcReport", StringValue(rohcReport));

    // VoIP settings for every generator, as in ntnlab_n.cc. The generator keeps 12.2 kbit/s while
    // talking, so 12-byte packets leave every 12 x 8 / 12200 s = 7.9 ms (20 ms needs 30.5 bytes).
    Config::SetDefault("ns3::TrafficGeneratorNgmnVoip::VoicePayload", UintegerValue(12));
    Config::SetDefault("ns3::TrafficGeneratorNgmnVoip::SIDPayload", UintegerValue(7));

    // --- The link budget ------------------------------------------------------------------------
    // The satellite comes from TR 38.821: Set-1 or Set-2, the LEO-600 or the LEO-1200 column,
    // whichever altitude is nearer. The defaults (Set-1, 600 km, 2 GHz, 30 MHz) are study case 9,
    // leo-link's default preset. A flag given on the command line replaces the table value.
    NS_ABORT_MSG_UNLESS(satSet == 1 || satSet == 2, "--satSet must be 1 or 2");
    NS_ABORT_MSG_UNLESS(altitudeKm >= 300.0 && altitudeKm <= 2000.0,
                        "--altitudeKm must be 300 to 2000: this example is set up for LEO");
    const SatelliteColumn& satColumn = g_satelliteSets[satSet - 1][altitudeKm < 900.0 ? 0 : 1];
    const bool satAsInTable = std::isnan(satEirpDbwPerMHz) && std::isnan(satAntennaGainDbi) &&
                              std::isnan(satGtDbPerK) && std::isnan(beamHpbwDeg) &&
                              altitudeKm == satColumn.altitudeKm;
    satEirpDbwPerMHz = std::isnan(satEirpDbwPerMHz) ? satColumn.eirpDbwPerMHz : satEirpDbwPerMHz;
    satAntennaGainDbi = std::isnan(satAntennaGainDbi) ? satColumn.gainDbi : satAntennaGainDbi;
    satGtDbPerK = std::isnan(satGtDbPerK) ? satColumn.gtDbPerK : satGtDbPerK;
    beamHpbwDeg = std::isnan(beamHpbwDeg) ? satColumn.hpbwDeg : beamHpbwDeg;
    const double altitudeM = altitudeKm * 1e3;
    const double dlFrequencyHz = dlFrequencyGHz * 1e9;
    // FDD: the uplink has its own carrier, 190 MHz below the downlink unless given
    const double ulFrequencyHz = ulFrequencyGHz > 0.0 ? ulFrequencyGHz * 1e9 : dlFrequencyHz - 190e6;
    const double bandwidthHz = bandwidthMHz * 1e6;
    for (double carrierHz : {dlFrequencyHz, ulFrequencyHz})
    {
        NS_ABORT_MSG_UNLESS(carrierHz >= 0.6e9 && carrierHz < 6e9,
                            "Both carriers must be 0.6 to 6 GHz: the UE is the handheld of the "
                            "S-band study cases (Ka-band needs the VSAT terminal)");
    }
    NS_ABORT_MSG_UNLESS(std::abs(dlFrequencyHz - ulFrequencyHz) >= bandwidthHz,
                        "The downlink and uplink carriers overlap: FDD needs them at least "
                        "--bandwidthMHz apart");
    const double satAntennaGainDb = satAntennaGainDbi; // the EIRP already contains it
    // G/T is the antenna gain over the system noise temperature T. The gNB PHY wants a noise
    // figure on 290 K instead: T / 290 K, in dB, to 0.01 dB as leo-link gives it (Set-1 and
    // Set-2 both make 776 K = 4.28 dB).
    const double satNoiseFigureDb =
        std::round((satAntennaGainDbi - satGtDbPerK - 10.0 * std::log10(290.0)) * 100.0) / 100.0;
    const double ueTxPowerDbm = 23.0;
    const double ueNoiseFigureDb = 7.0;
    const double ueAntennaGainDbi = 0.0;
    const double satFeederRxGainDb = satFeederRxGainDbi;
    const double satEirpDbm = satEirpDbwPerMHz + 30.0 + 10.0 * std::log10(bandwidthMHz);
    const double noiseFloorDbm = -174.0 + 10.0 * std::log10(bandwidthHz) + ueNoiseFigureDb;
    const Time channelUpdatePeriod = MilliSeconds(1000); // LOS state redrawn this often

    Config::SetDefault("ns3::NrGnbPhy::PowerAllocationType", StringValue("UniformPowerAllocBw"));
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(channelUpdatePeriod));
    Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                       TimeValue(channelUpdatePeriod));
    Config::SetDefault("ns3::NrMacSchedulerHarqRr::ConsolidateHarqRetx", BooleanValue(true));
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(10 * 1024 * 1024));
    Config::SetDefault("ns3::NrRlcUm::ReorderingTimer", TimeValue(MilliSeconds(100)));
    // Ideal random access, as leo-link's default: no RA timeout, 2 s to finish the connection
    Config::SetDefault("ns3::NrGnbRrc::ConnectionRequestTimeoutDuration",
                       TimeValue(MilliSeconds(2000)));
    // The UEs attach one after another over the first 0.5 s. All at once, the gNB answers only
    // about a dozen of them and the rest never connect (ideal random access does not retry).
    Config::SetDefault("ns3::NrHelper::AttachWindow", TimeValue(MilliSeconds(500)));

    // --- Where everything is -----------------------------------------------------------
    // UEs on the equator, UE 0 at longitude 0 and UE i x spacingKm further east. The satellite
    // comes from the north on a circular orbit, heading south, towards the UEs. Standing still, it
    // is on UE 0's meridian at the central angle that puts it at --elevation. Flying, it passes
    // exactly over UE 0, and it is at --elevation at the moment the calls start.
    const Time callsStart = Seconds(1);
    Time overheadAt; // flying only: when the satellite is over UE 0 ...
    Time downAt;     // ... and when it is back down at 10 deg on the other side
    const double earthRadiusM = GeographicPositions::EARTH_SPHERE_RADIUS;
    const double orbitRadiusM = earthRadiusM + altitudeM;
    const double elevationRad = elevation * M_PI / 180.0;
    const double centralAngleRad =
        std::acos(earthRadiusM / orbitRadiusM * std::cos(elevationRad)) - elevationRad;
    const double satLatDeg = centralAngleRad * 180.0 / M_PI;
    const double orbitSpeedMps = std::sqrt(3.986004418e14 / orbitRadiusM);

    Ptr<Node> satellite = CreateObject<Node>();
    Ptr<GeocentricConstantPositionMobilityModel> satMobility;
    OrbitPredictor orbit;
    if (freezeGeometry)
    {
        Ptr<FixedMobilityModel> frozen = CreateObject<FixedMobilityModel>();
        frozen->SetGeographicPosition(Vector(satLatDeg, 0.0, altitudeM));
        frozen->SetReportedVelocity(Vector(orbitSpeedMps * std::sin(centralAngleRad),
                                           0.0,
                                           -orbitSpeedMps * std::cos(centralAngleRad)));
        satMobility = frozen;
        orbit.pos0 = frozen->GetPosition();
    }
    else
    {
        // A polar orbit that passes exactly over UE 0. The Earth turns under the orbit, so s
        // seconds before it is overhead the satellite is omega x s of latitude to the north and
        // omegaEarth x s of longitude to the east of UE 0, which is a central angle c with
        // cos c = cos(omega s) cos(omegaEarth s). An elevation asks for a central angle, and so for s.
        Ptr<OrbitingMobilityModel> orbiting = CreateObject<OrbitingMobilityModel>();
        orbiting->SetAttribute("Altitude", DoubleValue(altitudeM / 1000.0)); // km
        orbiting->SetAttribute("Inclination", DoubleValue(90.0));
        orbiting->SetAttribute("Resolution", TimeValue(Seconds(1))); // CourseChange period, as leo-link
        const double omega = orbiting->GetAngularVelocity();
        const double omegaEarth = 2.0 * M_PI / Days(1).GetSeconds(); // as the orbit model turns the Earth
        auto secondsFromOverhead = [&](double elevationDeg) {
            const double e = elevationDeg * M_PI / 180.0;
            const double cosCentral = std::cos(std::acos(earthRadiusM / orbitRadiusM * std::cos(e)) - e);
            double low = 0.0;
            double high = std::acos(cosCentral) / omega;
            for (int i = 0; i < 100; ++i)
            {
                const double mid = 0.5 * (low + high);
                if (std::cos(omega * mid) * std::cos(omegaEarth * mid) > cosCentral)
                {
                    low = mid; // still closer than asked
                }
                else
                {
                    high = mid;
                }
            }
            return 0.5 * (low + high);
        };
        overheadAt = callsStart + Seconds(secondsFromOverhead(elevation));
        downAt = overheadAt + Seconds(secondsFromOverhead(10.0));
        // The model takes the longitude of the orbit plane and the angle along the orbit from its
        // northernmost point, here the North Pole; the satellite is 90 deg east of the plane's
        // longitude. At t = 0 it is overheadAt seconds before UE 0.
        const double toOverhead = overheadAt.GetSeconds();
        orbiting->SetPosition(Vector(omegaEarth * toOverhead * 180.0 / M_PI - 90.0,
                                     90.0 - omega * toOverhead * 180.0 / M_PI,
                                     0.0));
        satMobility = orbiting;
        orbit.pos0 = orbiting->GetPosition();
        orbit.vel0 = orbiting->GetVelocity();
        orbit.omegaRadPerS = orbiting->GetAngularVelocity();
        NS_ABORT_MSG_UNLESS(orbit.vel0.z < 0.0 && orbit.omegaRadPerS > 0.0,
                            "the orbit model does not fly the satellite south");
    }
    satellite->AggregateObject(satMobility);

    NodeContainer ueNodes;
    ueNodes.Create(nUes);
    std::vector<Ptr<FixedMobilityModel>> ueMobility(nUes);
    const double kmPerDegree = earthRadiusM * M_PI / 180.0 / 1000.0;
    for (uint32_t i = 0; i < nUes; ++i)
    {
        ueMobility[i] = CreateObject<FixedMobilityModel>();
        ueMobility[i]->SetGeographicPosition(Vector(0.0, i * spacingKm / kmPerDegree, 0.0));
        ueNodes.Get(i)->AggregateObject(ueMobility[i]);
    }

    // The gateway is where UE 0 is, as in leo-link: the feeder link then has the same length as
    // the service link. It carries no device; the channel only needs its position.
    Ptr<Node> gateway = CreateObject<Node>();
    Ptr<FixedMobilityModel> gatewayMobility = CreateObject<FixedMobilityModel>();
    gatewayMobility->SetGeographicPosition(Vector(0.0, 0.0, 0.0));
    gateway->AggregateObject(gatewayMobility);

    // --- Channels, delay and NTN timing ---------------------------------------------------
    Ptr<NrNtnPropagationDelayModel> ntnDelay = CreateObject<NrNtnPropagationDelayModel>();
    if (transparent)
    {
        ntnDelay->SetFeeder(gatewayMobility);
    }
    NtnCarrier dlCarrier =
        CreateNtnCarrier(dlFrequencyHz, scenario, gatewayMobility, ntnDelay, transparent);
    NtnCarrier ulCarrier =
        CreateNtnCarrier(ulFrequencyHz, scenario, gatewayMobility, ntnDelay, transparent);
    for (const auto& carrier : {dlCarrier, ulCarrier})
    {
        carrier.feeder->SetAttribute("GatewayTxPowerDbm", DoubleValue(gatewayTxPowerDbm));
        carrier.feeder->SetAttribute("SatRxAntennaGainDb", DoubleValue(satFeederRxGainDb));
        carrier.feeder->SetAttribute("RelayEirpDbm", DoubleValue(satEirpDbm));
        carrier.feeder->SetAttribute("RelayLossDb", DoubleValue(0.0));
        carrier.feeder->SetAttribute("RelayInputNoisePowerDbm", DoubleValue(noiseFloorDbm));
        carrier.feeder->SetAttribute("ReceiverNoisePowerDbm", DoubleValue(noiseFloorDbm));
    }

    // What the run will see of the satellite, every 100 ms from t = 0 to the end (one step when
    // it stands still): the longest round trip, which K_offset has to cover, and the elevation at
    // UE 0, which must be 10 deg or more while the calls are on (the 3GPP NTN channel is defined
    // from 10 deg up; in the second before the calls and the two after them it may be a little
    // lower, and the channel model then takes its 10 deg values).
    const Time lastCallStart = callsStart + MilliSeconds((nUes - 1) * startOffsetMs);
    const Time simEnd = callsStart + duration + Seconds(2);
    const Vector ue0Position = ueMobility[0]->GetPosition();
    if (!freezeGeometry)
    {
        // The pass is what was asked for: --elevation when the calls start, UE 0 right below later
        NS_ABORT_MSG_UNLESS(
            std::abs(ElevationDeg(orbit.At(callsStart), ue0Position) - elevation) < 1e-6 &&
                std::abs((orbit.At(overheadAt) - ue0Position).GetLength() - altitudeM) < 1.0,
            "the orbit does not pass over UE 0 as planned");
    }
    Ptr<ConstantPositionMobilityModel> satProbe = CreateObject<ConstantPositionMobilityModel>();
    Time maxLoopRtt;
    Time maxLoopRttAt;
    for (Time t = Seconds(0); t <= simEnd; t += MilliSeconds(100))
    {
        const Vector sat = orbit.At(t);
        satProbe->SetPosition(sat);
        for (uint32_t i = 0; i < nUes; ++i)
        {
            const Time loopRtt = 2 * ntnDelay->GetOneWayDelay(satProbe, ueMobility[i]);
            if (loopRtt > maxLoopRtt)
            {
                maxLoopRtt = loopRtt;
                maxLoopRttAt = t;
            }
        }
        NS_ABORT_MSG_IF(t >= callsStart && t <= lastCallStart + duration &&
                            ElevationDeg(sat, ue0Position) < 9.99,
                        "The satellite is down at 10 deg elevation again at t = "
                            << downAt.GetSeconds() << " s, before the calls end. Use --duration="
                            << std::floor((downAt - lastCallStart).GetSeconds()) << "s or less.");
        if (freezeGeometry)
        {
            break;
        }
    }
    // K_offset covers the longest round trip in the cell, in 1 ms slots (TS 38.213 4.2). It is
    // cell-specific and does not change during the run.
    const uint32_t kOffsetSlots = static_cast<uint32_t>(
        std::clamp<double>(std::ceil(maxLoopRtt.GetSeconds() * 1000.0), 1.0, 1023.0));

    // --- NR radio + core, configured as leo-link configures them ---------------------------
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf dlBandConf(dlFrequencyHz, bandwidthHz, 1);
    CcBwpCreator::SimpleOperationBandConf ulBandConf(ulFrequencyHz, bandwidthHz, 1);
    OperationBandInfo dlBand = ccBwpCreator.CreateOperationBandContiguousCc(dlBandConf);
    OperationBandInfo ulBand = ccBwpCreator.CreateOperationBandContiguousCc(ulBandConf);
    dlBand.GetBwpAt(0, 0)->SetChannel(dlCarrier.channel);
    ulBand.GetBwpAt(0, 0)->SetChannel(ulCarrier.channel);
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({dlBand, ulBand});

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);
    beamformingHelper->SetAttribute("BeamformingMethod",
                                    TypeIdValue(DirectPathBeamforming::GetTypeId()));
    beamformingHelper->SetAttribute("BeamformingPeriodicity", TimeValue(MilliSeconds(100)));

    nrHelper->SetGnbPhyAttribute("CellSpecificKoffset", UintegerValue(kOffsetSlots));
    nrHelper->SetUeMacAttribute("IdealRandomAccess", BooleanValue(true));
    // Every flow uses BWP 0, the downlink carrier; the gNB tells the UE that BWP 1 is its uplink
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(0));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(0));

    // Rank 1 over two cross-polarised ports at each end, CQI/PMI feedback on
    const uint32_t carrierRbs = static_cast<uint32_t>(bandwidthHz * 0.96 / (15e3 * 12));
    NrHelper::MimoPmiParams mimoPmiParams;
    mimoPmiParams.pmSearchMethod = "ns3::NrPmSearchFull";
    mimoPmiParams.fullSearchCb = "ns3::NrCbTwoPort";
    mimoPmiParams.rankLimit = 1;
    mimoPmiParams.rankThreshold = 0.0;
    mimoPmiParams.rankTechnique = "Sasaoka";
    mimoPmiParams.subbandSize = carrierRbs <= 72 ? 8 : (carrierRbs <= 144 ? 16 : 32);
    mimoPmiParams.downsamplingTechnique = "FirstPRB";
    nrHelper->SetupMimoPmi(mimoPmiParams);

    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(scheduler));
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(true));
    nrHelper->SetGnbMacAttribute("NumHarqProcess", UintegerValue(16));
    nrHelper->SetUeMacAttribute("NumHarqProcess", UintegerValue(16));
    nrHelper->SetGnbPhyAttribute("N1Delay", UintegerValue(2));
    nrHelper->SetGnbPhyAttribute("N2Delay", UintegerValue(2));
    nrHelper->SetSchedulerAttribute("OllaStepUp", DoubleValue(0.01));
    nrHelper->SetSchedulerAttribute("OllaTargetBler", DoubleValue(0.15));
    nrHelper->SetSchedulerAttribute("OllaMaxOffset", DoubleValue(12.0));

    // Satellite: one element, two polarisations, 30 dBi. The gNB transmits the EIRP minus that gain.
    nrHelper->SetGnbAntennaTypeId("ns3::UniformPlanarArray");
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumVerticalPorts", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumHorizontalPorts", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("IsDualPolarized", BooleanValue(true));
    nrHelper->SetGnbAntennaAttribute("PolSlantAngle", DoubleValue(0.0));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
                                         "Gain",
                                         DoubleValue(satAntennaGainDb))));
    // Handheld: one element, two +/-45 deg ports, 0 dBi
    nrHelper->SetUeAntennaTypeId("ns3::UniformPlanarArray");
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumVerticalPorts", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumHorizontalPorts", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("IsDualPolarized", BooleanValue(true));
    nrHelper->SetUeAntennaAttribute("PolSlantAngle", DoubleValue(45.0 * M_PI / 180.0));

    // --- One gNB (on the satellite node, where the radio is) and nUes UEs -------------------
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(NodeContainer(satellite), allBwps);
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)
        ->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 1)
        ->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));

    // Each UE is installed on its own because its antenna gain carries the beam taper of its position
    NetDeviceContainer ueDevs;
    for (uint32_t i = 0; i < nUes; ++i)
    {
        const double taperDb = BeamTaperDb(satMobility->GetPosition(),
                                           ueMobility[0]->GetPosition(),
                                           ueMobility[i]->GetPosition(),
                                           beamHpbwDeg);
        g_scene.ueElements.push_back(
            CreateObjectWithAttributes<IsotropicAntennaModel>("Gain",
                                                              DoubleValue(ueAntennaGainDbi + taperDb)));
        nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(g_scene.ueElements[i]));
        ueDevs.Add(nrHelper->InstallUeDevice(NodeContainer(ueNodes.Get(i)), allBwps));
    }

    for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
    {
        NrHelper::GetGnbPhy(gnbDevs.Get(0), bwp)->SetTxPower(satEirpDbm - satAntennaGainDb);
        NrHelper::GetGnbPhy(gnbDevs.Get(0), bwp)
            ->SetAttribute("NoiseFigure", DoubleValue(satNoiseFigureDb));
        for (uint32_t i = 0; i < nUes; ++i)
        {
            NrHelper::GetUePhy(ueDevs.Get(i), bwp)->SetTxPower(ueTxPowerDbm);
            NrHelper::GetUePhy(ueDevs.Get(i), bwp)
                ->SetAttribute("NoiseFigure", DoubleValue(ueNoiseFigureDb));
        }
    }

    // --- ONE remote host, far end of every call ---------------------------------------------
    auto [remoteHost, remoteHostAddr] = epcHelper->SetupRemoteHost("10Gb/s", 1500, internetDelay);

    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIfaces = epcHelper->AssignUeIpv4Address(ueDevs);
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // Timing advance: each UE sends early by its one-way delay to the gNB (feeder link included
    // when transparent), on both of its BWP PHYs. The UE PHY asks again before every slot, so the
    // advance follows a moving satellite.
    for (uint32_t i = 0; i < nUes; ++i)
    {
        for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
        {
            NrHelper::GetUePhy(ueDevs.Get(i), bwp)
                ->SetTimingAdvanceCallback(
                    Callback<Time>([ntnDelay, satMobility, ue = ueMobility[i]]() {
                        return ntnDelay->GetOneWayDelay(satMobility, ue);
                    }));
        }
    }

    // The satellite points its arrays at the beam centre (UE 0): once when it stands still, every
    // 100 ms when it flies, together with the beam gain towards every UE
    g_scene.satellite = satMobility;
    g_scene.ues = ueMobility;
    g_scene.delay = ntnDelay;
    g_scene.ueAntennaGainDbi = ueAntennaGainDbi;
    g_scene.beamHpbwDeg = beamHpbwDeg;
    g_scene.flying = !freezeGeometry;
    g_scene.orbit = orbit;
    g_scene.groundDelayMs = internetDelay.GetSeconds() * 1000.0;
    for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
    {
        g_scene.satArrays.push_back(NrHelper::GetGnbPhy(gnbDevs.Get(0), bwp)
                                        ->GetSpectrumPhy()
                                        ->GetAntenna()
                                        ->GetObject<UniformPlanarArray>());
    }
    for (uint32_t i = 0; i < nUes; ++i)
    {
        g_scene.ueArrays.emplace_back();
        for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
        {
            g_scene.ueArrays[i].push_back(DynamicCast<PhasedArrayModel>(
                NrHelper::GetUePhy(ueDevs.Get(i), bwp)->GetSpectrumPhy()->GetAntenna()));
        }
    }
    PointBeam();
    if (!freezeGeometry)
    {
        Simulator::Schedule(MilliSeconds(100), &FollowBeam);
    }

    // Same port for every call, so ONE rule fits all UEs: one line activates nUes voice flows
    Ptr<NrQosRule> rule = Create<NrQosRule>();
    NrQosRule::PacketFilter filter;
    filter.direction = ul ? NrQosRule::UPLINK : NrQosRule::DOWNLINK;
    if (ul)
    {
        filter.remotePortStart = filter.remotePortEnd = port;
    }
    else
    {
        filter.localPortStart = filter.localPortEnd = port;
    }
    rule->Add(filter);
    NrGbrQosInformation qos;
    qos.gbrUl = qos.gbrDl = qos.mbrUl = qos.mbrDl = 40000;
    nrHelper->ActivateDedicatedQosFlow(ueDevs, NrQosFlow(NrQosFlow::GBR_CONV_VOICE, qos), rule);

    // Sinks: dl = one per UE (one Install call), ul = one on the remote host for all calls
    PacketSinkHelper sinkHelper(transportProtocol, InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApps = sinkHelper.Install(ul ? NodeContainer(remoteHost) : ueNodes);
    sinkApps.Start(Seconds(0));
    sinkApps.Stop(Seconds(3) + duration);

    // The loop holds only what differs per call
    g_dlSinrSum.assign(nUes, 0);
    g_dlSinrN.assign(nUes, 0);
    g_rlf.assign(nUes, 0);
    g_view.assign(nUes, CallView());
    // Line of sight is read on the carrier that carries the voice
    g_scene.condition =
        DynamicCast<ThreeGppPropagationLossModel>((ul ? ulCarrier : dlCarrier).serviceLoss)
            ->GetChannelConditionModel();
    ApplicationContainer voipApps;
    for (uint32_t i = 0; i < nUes; ++i)
    {
        Ipv4Address dest = ul ? remoteHostAddr : ueIfaces.GetAddress(i);
        TrafficGeneratorHelper voipHelper(transportProtocol,
                                          InetSocketAddress(dest, port),
                                          TrafficGeneratorNgmnVoip::GetTypeId());
        voipHelper.SetAttribute("VoiceActivityFactor", DoubleValue(vaf[i])); // this call only
        ApplicationContainer voip = voipHelper.Install(ul ? ueNodes.Get(i) : remoteHost);
        const Time callStart = callsStart + MilliSeconds(i * startOffsetMs); // not phase-aligned
        voip.Start(callStart);
        voip.Stop(callStart + duration);
        voipApps.Add(voip);
        g_view[i].callStart = callStart;
        g_view[i].callEnd = callStart + duration;
        Simulator::Schedule(callStart, &SampleCall, i);
        if (!freezeGeometry && overheadAt > callStart && overheadAt < callStart + duration)
        {
            // one more look when the satellite is over UE 0, which the 100 ms steps can miss by 50 ms
            Simulator::Schedule(overheadAt, &RecordView, i);
        }

        Connect(NrHelper::GetUePhy(ueDevs.Get(i), 0), "DlCtrlSinr", MakeBoundCallback(&DlSinr, i));
        Connect(DynamicCast<NrUeNetDevice>(ueDevs.Get(i))->GetRrc(),
                "RadioLinkFailure",
                MakeBoundCallback(&RadioLinkFailure, i));
    }

    // FlowMonitor on the UEs and the remote host gives one flow per call
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.Install(NodeContainer(ueNodes, remoteHost));

    Simulator::Schedule(Seconds(1), &SampleLos);

    if (nrTraces)
    {
        nrHelper->EnableTraces();
    }

    // --- What was built --------------------------------------------------------------------
    // The delay of the whole path, one way and propagation only: the N6 link, then the feeder
    // link when the gNB is at the gateway, then the service link. With the gNB on the satellite
    // the feeder link carries N3, which has no delay in this model.
    const std::string path =
        transparent ? "remote host -> gateway -> satellite -> UE" : "remote host -> satellite -> UE";
    const double n6Ms = internetDelay.GetSeconds() * 1000.0;
    struct Ue0View
    {
        double elevationDeg;
        double rangeKm;
        double serviceMs;
        double feederMs;
    };
    auto ue0At = [&](Time t) {
        const Vector sat = orbit.At(t);
        satProbe->SetPosition(sat);
        Ue0View v;
        v.elevationDeg = ElevationDeg(sat, ue0Position);
        v.rangeKm = (sat - ue0Position).GetLength() / 1000.0;
        v.serviceMs = v.rangeKm * 1e6 / 299792458.0;
        v.feederMs =
            ntnDelay->GetOneWayDelay(satProbe, ueMobility[0]).GetSeconds() * 1000.0 - v.serviceMs;
        return v;
    };
    auto delayParts = [&](const Ue0View& v) {
        std::ostringstream o;
        o << std::fixed << std::setprecision(2) << n6Ms + v.feederMs + v.serviceMs << " ms = " << n6Ms
          << " N6";
        if (transparent)
        {
            o << " + " << v.feederMs << " feeder";
        }
        o << " + " << v.serviceMs << " service";
        return o.str();
    };
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "NR-NTN cell: " << (transparent ? "transparent" : "regenerative") << ", " << scenario
              << ", " << bandwidthMHz << " MHz FDD, DL " << std::setprecision(3) << dlFrequencyGHz
              << " / UL " << ulFrequencyHz / 1e9 << " GHz, " << std::setprecision(2) << nUes << " "
              << direction << " call(s), ROHC " << (rohc ? "on" : "off") << "\n";
    std::cout << "  satellite radio        : " << std::setprecision(0) << altitudeKm << " km, "
              << std::setprecision(2) << (satAsInTable ? "" : "changed from ") << "Set-" << satSet
              << " " << satColumn.orbit << " of TR 38.821\n"
              << "                           EIRP " << satEirpDbwPerMHz << " dBW/MHz = "
              << satEirpDbm << " dBm, antenna " << satAntennaGainDbi << " dBi, G/T " << satGtDbPerK
              << " dB/K (noise figure " << satNoiseFigureDb << " dB), beam "
              << std::setprecision(4) << beamHpbwDeg << " deg\n"
              << std::setprecision(2);
    if (transparent)
    {
        std::cout << "  feeder link            : gateway " << gatewayTxPowerDbm
                  << " dBm, satellite receive gain " << satFeederRxGainDbi << " dBi\n";
    }
    if (freezeGeometry)
    {
        const Ue0View now = ue0At(Seconds(0));
        std::cout << "  satellite              : stands still for the whole run "
                     "(--freezeGeometry=0 lets it fly)\n";
        std::cout << "  elevation at UE 0      : " << satMobility->GetElevationAngle(ueMobility[0])
                  << " deg (asked " << elevation << ")\n";
        std::cout << "  slant range to UE 0    : " << now.rangeKm << " km\n";
        std::cout << "  one-way delay to UE 0  : " << delayParts(now) << "\n";
        std::cout << "  longest round trip     : " << maxLoopRtt.GetMicroSeconds() / 1000.0
                  << " ms on the radio  =>  K_offset " << kOffsetSlots << " slots\n";
    }
    else
    {
        // UE 0 when its call starts and when it ends: the first line of the second table
        const Time ue0CallEnd = callsStart + duration;
        const Ue0View first = ue0At(callsStart);
        const Ue0View last = ue0At(ue0CallEnd);
        std::cout << "  satellite              : flies south at " << orbit.vel0.GetLength() / 1000.0
                  << " km/s, is over UE 0 at t = " << overheadAt.GetSeconds()
                  << " s and down at 10 deg again at t = " << downAt.GetSeconds() << " s\n"
                  << "                           (calls can last " << std::setprecision(0)
                  << std::floor((downAt - lastCallStart).GetSeconds()) << std::setprecision(2)
                  << " s). Elevation: - before it is overhead, + after.\n";
        std::cout << "  UE 0's call            : starts at t = " << callsStart.GetSeconds()
                  << " s  ->  ends at t = " << ue0CallEnd.GetSeconds() << " s\n";
        std::cout << "  elevation at UE 0      : " << (callsStart < overheadAt ? "-" : "+")
                  << first.elevationDeg << " deg (asked " << elevation << ")  ->  "
                  << (ue0CallEnd < overheadAt ? "-" : "+") << last.elevationDeg << " deg\n";
        std::cout << "  slant range to UE 0    : " << first.rangeKm << " km  ->  " << last.rangeKm
                  << " km\n";
        std::cout << "  one-way delay to UE 0  : " << delayParts(first) << "\n"
                  << "                       ->  " << delayParts(last) << "\n";
        std::cout << "  longest round trip     : " << maxLoopRtt.GetMicroSeconds() / 1000.0
                  << " ms on the radio, at t = " << maxLoopRttAt.GetSeconds()
                  << " s  =>  K_offset " << kOffsetSlots << " slots for the whole run\n";
    }
    if (ul)
    {
        std::cout << "  NOTE: the handheld uplink of this NR-NTN release is far below its link budget "
                     "(known limitation), so expect loss and long delays in uplink calls.\n";
    }
    std::cout << "\n";

    Simulator::Stop(simEnd);
    Simulator::Run();

    // The orbit prediction that sized K_offset must be where the model really flew the satellite
    NS_ABORT_MSG_IF((orbit.At(Simulator::Now()) - satMobility->GetPosition()).GetLength() > 1.0,
                    "the orbit prediction and the orbit model disagree");

    // --- One line per call -------------------------------------------------------------------
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    // lost = tx - rx: every generator stopped 2 s before the end, so nothing is still on its way.
    // beam[dB] is the mean over the call (it changes only when the satellite flies).
    std::vector<std::pair<uint32_t, uint32_t>> calls; // flow id, UE
    std::cout << "flow   UE address  off[km]  beam[dB]   VAF    tx    rx  lost  delay[ms]  jitter[ms]  "
                 "LOS[%]  RLF  DL SINR[dB]\n";
    for (const auto& [flowId, st] : monitor->GetFlowStats())
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        if (t.destinationPort != port)
        {
            continue; // not a voice flow
        }
        Ipv4Address ueAddr = ul ? t.sourceAddress : t.destinationAddress;
        uint32_t i = 0;
        while (i < nUes && ueIfaces.GetAddress(i) != ueAddr)
        {
            ++i; // which UE is this flow?
        }
        calls.emplace_back(flowId, i);
        const CallView& view = g_view[i];
        std::ostringstream address;
        address << ueAddr;
        std::cout << std::fixed << std::setw(4) << flowId << "  " << std::setw(11) << address.str()
                  << std::setw(9) << std::setprecision(1) << i * spacingKm << std::setw(10)
                  << std::setprecision(2) << view.taperDb.sum / view.taperDb.n << std::setw(6)
                  << vaf[i] << std::setw(6) << st.txPackets << std::setw(6) << st.rxPackets
                  << std::setw(6) << st.txPackets - st.rxPackets << std::setprecision(3)
                  << std::setw(11)
                  << (st.rxPackets ? st.delaySum.GetSeconds() * 1000.0 / st.rxPackets : NAN)
                  << std::setw(12)
                  << (st.rxPackets > 1 ? st.jitterSum.GetSeconds() * 1000.0 / (st.rxPackets - 1)
                                       : NAN)
                  << std::setprecision(0) << std::setw(8)
                  << 100.0 * view.losYes / view.losAll << std::setw(5) << g_rlf[i]
                  << std::setprecision(1) << std::setw(13)
                  << (g_dlSinrN[i] ? 10 * std::log10(g_dlSinrSum[i] / g_dlSinrN[i]) : NAN) << "\n";
    }

    // --- What each UE saw of the satellite during its call, sampled every 100 ms -----------------
    // The delay is the propagation delay of the whole path, one way: N6 + feeder + service.
    if (freezeGeometry)
    {
        std::cout << "\nSatellite seen from each UE: the same for the whole call, it stands still.\n"
                  << "Delay: one way, " << path << ", propagation only.\n"
                  << "flow   UE address  elevation[deg]  range[km]  delay[ms]\n";
    }
    else
    {
        // " ---- label ----", exactly as wide as the four columns below it
        auto group = [](const std::string& label, std::size_t width) {
            const std::size_t dashes = width - label.size() - 3;
            return " " + std::string(dashes / 2, '-') + " " + label + " " +
                   std::string(dashes - dashes / 2, '-');
        };
        std::cout << "\nSatellite seen from each UE during its call.\n"
                  << "Elevation: - before the satellite is overhead, + after; low and high are the "
                     "lowest and highest of the call.\n"
                  << "Delay: one way, " << path << ", propagation only.\n"
                  << std::string(17, ' ') << group("elevation [deg]", 32)
                  << group("slant range [km]", 40) << group("delay [ms]", 32) << "\n"
                  << "flow   UE address";
        for (const char* column : {"start", "end", "low", "high"})
        {
            std::cout << std::setw(8) << column;
        }
        for (int width : {10, 8})
        {
            for (const char* column : {"start", "end", "min", "max"})
            {
                std::cout << std::setw(width) << column;
            }
        }
        std::cout << "\n";
    }
    for (const auto& [flowId, i] : calls)
    {
        const CallView& view = g_view[i];
        std::ostringstream address;
        address << ueIfaces.GetAddress(i);
        std::cout << std::fixed << std::setw(4) << flowId << "  " << std::setw(11) << address.str();
        if (freezeGeometry)
        {
            std::cout << std::setprecision(2) << std::setw(16) << view.elevationDeg.first
                      << std::setw(11) << view.rangeKm.first << std::setprecision(3)
                      << std::setw(11) << view.delayMs.first << "\n";
            continue;
        }
        std::cout << std::setprecision(2) << std::showpos << std::setw(8)
                  << view.sideAtStart * view.elevationDeg.first << std::setw(8)
                  << view.sideAtEnd * view.elevationDeg.last << std::noshowpos << std::setw(8)
                  << view.elevationDeg.min << std::setw(8) << view.elevationDeg.max;
        for (double v : {view.rangeKm.first, view.rangeKm.last, view.rangeKm.min, view.rangeKm.max})
        {
            std::cout << std::setw(10) << v;
        }
        std::cout << std::setprecision(3);
        for (double v : {view.delayMs.first, view.delayMs.last, view.delayMs.min, view.delayMs.max})
        {
            std::cout << std::setw(8) << v;
        }
        std::cout << "\n";
    }

    g_scene = Scene(); // let go of the nodes before the simulator is destroyed
    Simulator::Destroy();
    return 0;
}
