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
 *   - "distance" became "elevation": the satellite stands still at --elevation, seen from UE 0,
 *     but reports its orbital velocity, so the fading keeps moving (leo-link's --freezeGeometry)
 *   - UE i sits i x --spacingKm from the beam centre (UE 0) and loses the beam gain of that offset
 *
 * New columns: LOS[%], the share of the run with line of sight to the satellite, and RLF, the
 * number of radio link failures of the UE.
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
 *   ./ns3 run "ntnlab_nr --direction=ul"
 *   ./ns3 run "ntnlab_nr --transparent=0"              gNB on the satellite, no feeder link
 *   ./ns3 run "ntnlab_nr --scenario=NTN-Rural"         or NTN-Urban, NTN-DenseUrban
 *   ./ns3 run "ntnlab_nr --rohc=0"                     without header compression
 *   ./ns3 run "ntnlab_nr --nUes=1 --rohcReport=full"   every ROHC detail of the call
 *   ./ns3 run "ntnlab_nr --nrTraces=1" --cwd=results/t1   also write the NR trace files there
 */

#include "ns3/angles.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/geocentric-ecef-mobility-model.h"
#include "ns3/geographic-positions.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
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

// How often each UE had line of sight to the satellite, sampled every 100 ms on the carrier
// that carries the voice. The channel redraws the state once per second (see below).
std::vector<uint32_t> g_losYes;
std::vector<uint32_t> g_losAll;

void
SampleLos(Ptr<ChannelConditionModel> conditionModel,
          Ptr<MobilityModel> satellite,
          std::vector<Ptr<MobilityModel>> ues)
{
    for (std::size_t i = 0; i < ues.size(); ++i)
    {
        ++g_losAll[i];
        if (conditionModel->GetChannelCondition(satellite, ues[i])->IsLos())
        {
            ++g_losYes[i];
        }
    }
    Simulator::Schedule(MilliSeconds(100), &SampleLos, conditionModel, satellite, ues);
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

// A satellite that stands still but reports its orbital velocity (what leo-link does under
// --freezeGeometry): elevation, range and delay stay constant, the channel's Doppler term does not.
class FrozenSatelliteMobilityModel : public GeocentricEcefMobilityModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnLabFrozenSatelliteMobilityModel")
                                .SetParent<GeocentricEcefMobilityModel>()
                                .SetGroupName("Mobility")
                                .AddConstructor<FrozenSatelliteMobilityModel>();
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

    Vector m_velocity{0.0, 0.0, 0.0};
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

} // namespace

int
main(int argc, char* argv[])
{
    uint32_t nUes = 10;
    double elevation = 30.0; // of the satellite, seen from UE 0 (deg), held for the whole run
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
    Time internetDelay = MilliSeconds(10);
    uint16_t port = 4000;
    uint32_t startOffsetMs = 1; // call i starts at 1 s + i x startOffsetMs
    double vafMean = 0.45; // voice activity factor of each call ~ Gaussian(vafMean, vafStd)
    double vafStd = 0.15;  // 0 = every call gets vafMean

    CommandLine cmd(__FILE__);
    cmd.AddValue("nUes", "Number of UEs, one call each", nUes);
    cmd.AddValue("elevation", "Elevation of the satellite seen from UE 0 (deg), 10 to 90", elevation);
    cmd.AddValue("spacingKm", "UE i is i x spacingKm east of the beam centre (km)", spacingKm);
    cmd.AddValue("duration", "How long each call lasts", duration);
    cmd.AddValue("direction", "ul = UEs talk to the remote host, dl = remote host talks to UEs", direction);
    cmd.AddValue("scenario", "NTN-Suburban, NTN-Rural, NTN-Urban or NTN-DenseUrban", scenario);
    cmd.AddValue("transparent", "1 = gNB at the gateway (feeder link), 0 = gNB on the satellite", transparent);
    cmd.AddValue("bandwidthMHz", "Bandwidth of each FDD carrier (MHz)", bandwidthMHz);
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

    // --- The link budget: leo-link's default preset, TR 38.821 study case 9 ----------
    const double altitudeM = 600e3;
    const double dlFrequencyHz = 2.0e9;
    const double ulFrequencyHz = dlFrequencyHz - 190e6; // FDD: its own carrier, 190 MHz below
    const double bandwidthHz = bandwidthMHz * 1e6;
    const double satEirpDbwPerMHz = 34.0;
    const double satAntennaGainDb = 30.0; // the EIRP already contains it
    const double satNoiseFigureDb = 4.28; // G/T 1.1 dB/K with 30 dBi
    const double beamHpbwDeg = 4.4127;
    const double ueTxPowerDbm = 23.0;
    const double ueNoiseFigureDb = 7.0;
    const double ueAntennaGainDbi = 0.0;
    const double gatewayTxPowerDbm = 60.0; // feeder link, transparent only
    const double satFeederRxGainDb = 30.0;
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
    // is north of UE 0 on the same meridian, at the central angle that puts it at --elevation,
    // and it reports the velocity of a circular orbit heading south, towards the UEs.
    const double earthRadiusM = GeographicPositions::EARTH_SPHERE_RADIUS;
    const double orbitRadiusM = earthRadiusM + altitudeM;
    const double elevationRad = elevation * M_PI / 180.0;
    const double centralAngleRad =
        std::acos(earthRadiusM / orbitRadiusM * std::cos(elevationRad)) - elevationRad;
    const double satLatDeg = centralAngleRad * 180.0 / M_PI;
    const double orbitSpeedMps = std::sqrt(3.986004418e14 / orbitRadiusM);

    Ptr<Node> satellite = CreateObject<Node>();
    Ptr<FrozenSatelliteMobilityModel> satMobility = CreateObject<FrozenSatelliteMobilityModel>();
    satMobility->SetGeographicPosition(Vector(satLatDeg, 0.0, altitudeM));
    satMobility->SetReportedVelocity(Vector(orbitSpeedMps * std::sin(centralAngleRad),
                                            0.0,
                                            -orbitSpeedMps * std::cos(centralAngleRad)));
    satellite->AggregateObject(satMobility);

    NodeContainer ueNodes;
    ueNodes.Create(nUes);
    std::vector<Ptr<GeocentricEcefMobilityModel>> ueMobility(nUes);
    const double kmPerDegree = earthRadiusM * M_PI / 180.0 / 1000.0;
    for (uint32_t i = 0; i < nUes; ++i)
    {
        ueMobility[i] = CreateObject<GeocentricEcefMobilityModel>();
        ueMobility[i]->SetGeographicPosition(Vector(0.0, i * spacingKm / kmPerDegree, 0.0));
        ueNodes.Get(i)->AggregateObject(ueMobility[i]);
    }

    // The gateway is where UE 0 is, as in leo-link: the feeder link then has the same length as
    // the service link. It carries no device; the channel only needs its position.
    Ptr<Node> gateway = CreateObject<Node>();
    Ptr<GeocentricEcefMobilityModel> gatewayMobility = CreateObject<GeocentricEcefMobilityModel>();
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

    // K_offset covers the longest round trip in the cell, in 1 ms slots (TS 38.213 4.2)
    Time maxLoopRtt;
    for (uint32_t i = 0; i < nUes; ++i)
    {
        maxLoopRtt = std::max(maxLoopRtt, 2 * ntnDelay->GetOneWayDelay(satMobility, ueMobility[i]));
    }
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
    std::vector<double> taperDb(nUes);
    for (uint32_t i = 0; i < nUes; ++i)
    {
        taperDb[i] = BeamTaperDb(satMobility->GetPosition(),
                                 ueMobility[0]->GetPosition(),
                                 ueMobility[i]->GetPosition(),
                                 beamHpbwDeg);
        nrHelper->SetUeAntennaAttribute(
            "AntennaElement",
            PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
                "Gain",
                DoubleValue(ueAntennaGainDbi + taperDb[i]))));
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
    // when transparent), on both of its BWP PHYs
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

    // Point the satellite's arrays at the beam centre (UE 0)
    {
        const Angles toCentre(ueMobility[0]->GetPosition(), satMobility->GetPosition());
        for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
        {
            Ptr<UniformPlanarArray> array = NrHelper::GetGnbPhy(gnbDevs.Get(0), bwp)
                                                ->GetSpectrumPhy()
                                                ->GetAntenna()
                                                ->GetObject<UniformPlanarArray>();
            array->SetAlpha(toCentre.GetAzimuth());
            array->SetBeta(toCentre.GetInclination());
            array->SetBeamformingVector(array->GetBeamformingVector());
        }
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
    ApplicationContainer voipApps;
    for (uint32_t i = 0; i < nUes; ++i)
    {
        Ipv4Address dest = ul ? remoteHostAddr : ueIfaces.GetAddress(i);
        TrafficGeneratorHelper voipHelper(transportProtocol,
                                          InetSocketAddress(dest, port),
                                          TrafficGeneratorNgmnVoip::GetTypeId());
        voipHelper.SetAttribute("VoiceActivityFactor", DoubleValue(vaf[i])); // this call only
        ApplicationContainer voip = voipHelper.Install(ul ? ueNodes.Get(i) : remoteHost);
        voip.Start(Seconds(1) + MilliSeconds(i * startOffsetMs)); // calls not phase-aligned
        voip.Stop(Seconds(1) + MilliSeconds(i * startOffsetMs) + duration);
        voipApps.Add(voip);

        Connect(NrHelper::GetUePhy(ueDevs.Get(i), 0), "DlCtrlSinr", MakeBoundCallback(&DlSinr, i));
        Connect(DynamicCast<NrUeNetDevice>(ueDevs.Get(i))->GetRrc(),
                "RadioLinkFailure",
                MakeBoundCallback(&RadioLinkFailure, i));
    }

    // FlowMonitor on the UEs and the remote host gives one flow per call
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.Install(NodeContainer(ueNodes, remoteHost));

    // Line-of-sight state of every UE on the carrier of the voice direction, from the first call on
    g_losYes.assign(nUes, 0);
    g_losAll.assign(nUes, 0);
    Ptr<ChannelConditionModel> conditionModel =
        DynamicCast<ThreeGppPropagationLossModel>((ul ? ulCarrier : dlCarrier).serviceLoss)
            ->GetChannelConditionModel();
    Simulator::Schedule(Seconds(1),
                        &SampleLos,
                        conditionModel,
                        satMobility,
                        std::vector<Ptr<MobilityModel>>(ueMobility.begin(), ueMobility.end()));

    if (nrTraces)
    {
        nrHelper->EnableTraces();
    }

    // --- What was built --------------------------------------------------------------------
    const double slantKm = satMobility->GetDistanceFrom(ueMobility[0]) / 1000.0;
    const Time oneWay = ntnDelay->GetOneWayDelay(satMobility, ueMobility[0]);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "NR-NTN cell: " << (transparent ? "transparent" : "regenerative") << " LEO-600, "
              << scenario << ", " << bandwidthMHz << " MHz FDD at 2 GHz, " << nUes << " "
              << direction << " call(s), ROHC " << (rohc ? "on" : "off") << "\n";
    std::cout << "  elevation at UE 0      : " << satMobility->GetElevationAngle(ueMobility[0])
              << " deg (asked " << elevation << ")\n";
    std::cout << "  slant range to UE 0    : " << slantKm << " km\n";
    std::cout << "  one-way radio delay    : " << oneWay.GetMicroSeconds() / 1000.0 << " ms"
              << (transparent ? " (gateway -> satellite -> UE 0)\n" : " (satellite -> UE 0)\n");
    std::cout << "  longest round trip     : " << maxLoopRtt.GetMicroSeconds() / 1000.0
              << " ms  =>  K_offset " << kOffsetSlots << " slots\n";
    if (ul)
    {
        std::cout << "  NOTE: the handheld uplink of this NR-NTN release is far below its link budget "
                     "(known limitation), so expect loss and long delays in uplink calls.\n";
    }
    std::cout << "\n";

    Simulator::Stop(Seconds(3) + duration);
    Simulator::Run();

    // --- One line per call -------------------------------------------------------------------
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    // lost = tx - rx: every generator stopped 2 s before the end, so nothing is still on its way
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
        std::ostringstream address;
        address << ueAddr;
        std::cout << std::fixed << std::setw(4) << flowId << "  " << std::setw(11) << address.str()
                  << std::setw(9) << std::setprecision(1) << i * spacingKm << std::setw(10)
                  << std::setprecision(2) << taperDb[i] << std::setw(6) << vaf[i] << std::setw(6)
                  << st.txPackets << std::setw(6) << st.rxPackets << std::setw(6)
                  << st.txPackets - st.rxPackets << std::setprecision(3) << std::setw(11)
                  << (st.rxPackets ? st.delaySum.GetSeconds() * 1000.0 / st.rxPackets : NAN)
                  << std::setw(12)
                  << (st.rxPackets > 1 ? st.jitterSum.GetSeconds() * 1000.0 / (st.rxPackets - 1)
                                       : NAN)
                  << std::setprecision(0) << std::setw(8) << 100.0 * g_losYes[i] / g_losAll[i]
                  << std::setw(5) << g_rlf[i]
                  << std::setprecision(1) << std::setw(13)
                  << (g_dlSinrN[i] ? 10 * std::log10(g_dlSinrSum[i] / g_dlSinrN[i]) : NAN) << "\n";
    }

    Simulator::Destroy();
    return 0;
}
