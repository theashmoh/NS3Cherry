// pftest.cc -- does the OFDMA proportional fair scheduler do what proportional fair should?
//
// One terrestrial gNB (TR 38.901 UMa, 3.5 GHz, 20 MHz, 30 kHz subcarrier spacing), N full-buffer UEs
// on a line at distances from --minDist to --maxDist, DL and UL UDP above what the cell can carry.
// The scheduler is chosen by --scheduler (RR, PF, MR, and the TDMA ones); --alpha is PF's exponent.
// Output (one CSV line per UE and direction):
//   scheduler,alpha,seed,ue,distM,dir,mbps,delayMs
// and a summary per run: sum rate, Jain's index of the rates, of the rates normalised to the
// best the UE could get alone (the share of the cell's time it got), the lowest UE.
//
// What proportional fair (alpha = 1) has to show, with full-buffer UEs and a channel that varies:
//   * every UE is served, so the lowest UE is not 0 (max rate starves the cell edge);
//   * the sum rate is not under round robin's (opportunistic gain of the fading);
//   * the share of time each UE gets is about the same, so rate / (best rate) is about equal.

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>

using namespace ns3;

int
main(int argc, char* argv[])
{
    uint32_t nUes = 10;
    double minDist = 40.0;
    double maxDist = 400.0;
    std::string scheduler = "ns3::NrMacSchedulerOfdmaPF";
    double alpha = 1.0;
    double simS = 4.0;
    double dlMbps = 30.0;
    double ulMbps = 10.0;
    double bwMHz = 20.0;
    bool dl = true;
    bool ul = true;
    std::string csv = "";
    std::string trace = "";
    CommandLine cmd(__FILE__);
    cmd.AddValue("nUes", "UEs", nUes);
    cmd.AddValue("minDist", "nearest UE (m)", minDist);
    cmd.AddValue("maxDist", "farthest UE (m)", maxDist);
    cmd.AddValue("scheduler", "scheduler TypeId", scheduler);
    cmd.AddValue("alpha", "PF exponent", alpha);
    cmd.AddValue("simS", "seconds", simS);
    cmd.AddValue("dlMbps", "offered DL per UE", dlMbps);
    cmd.AddValue("ulMbps", "offered UL per UE", ulMbps);
    cmd.AddValue("bwMHz", "bandwidth", bwMHz);
    cmd.AddValue("dl", "DL traffic", dl);
    cmd.AddValue("ul", "UL traffic", ul);
    cmd.AddValue("csv", "append CSV lines to this file", csv);
    cmd.AddValue("trace", "write every PUSCH the gNB receives (time, UE, RBs, MCS, TBS, SINR, rv, "
                 "corrupt) to this file", trace);
    cmd.Parse(argc, argv);

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(nUes);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    gnbPos->Add(Vector(0.0, 0.0, 25.0));
    mob.SetPositionAllocator(gnbPos);
    mob.Install(gnbNodes);
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    std::vector<double> dist(nUes);
    for (uint32_t i = 0; i < nUes; ++i)
    {
        dist[i] = nUes > 1 ? minDist + (maxDist - minDist) * i / (nUes - 1.0) : minDist;
        const double az = 2.0 * M_PI * i / nUes; // all around, so that the fading is not the same
        uePos->Add(Vector(dist[i] * std::cos(az), dist[i] * std::sin(az), 1.5));
    }
    mob.SetPositionAllocator(uePos);
    mob.Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> epc = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> bf = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nr = CreateObject<NrHelper>();
    nr->SetBeamformingHelper(bf);
    nr->SetEpcHelper(epc);

    CcBwpCreator creator;
    CcBwpCreator::SimpleOperationBandConf conf(3.5e9, bwMHz * 1e6, 1);
    OperationBandInfo band = creator.CreateOperationBandContiguousCc(conf);
    Ptr<NrChannelHelper> channel = CreateObject<NrChannelHelper>();
    channel->ConfigureFactories("UMa", "Default", "ThreeGpp");
    channel->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    channel->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channel->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector bwps = CcBwpCreator::GetAllBwps({band});

    bf->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));
    epc->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));
    nr->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nr->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nr->SetUeAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));
    nr->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nr->SetGnbAntennaAttribute("NumColumns", UintegerValue(4));
    nr->SetGnbAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));
    nr->SetUePhyAttribute("TxPower", DoubleValue(23.0));
    nr->SetSchedulerTypeId(TypeId::LookupByName(scheduler));
    if (scheduler.find("PF") != std::string::npos)
    {
        nr->SetSchedulerAttribute("FairnessIndex", DoubleValue(alpha));
    }
    nr->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));

    NetDeviceContainer gnbDevs = nr->InstallGnbDevice(gnbNodes, bwps);
    NetDeviceContainer ueDevs = nr->InstallUeDevice(ueNodes, bwps);
    int64_t stream = 1;
    stream += nr->AssignStreams(gnbDevs, stream);
    stream += nr->AssignStreams(ueDevs, stream);
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->SetAttribute("Numerology", UintegerValue(1));
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->SetAttribute("TxPower", DoubleValue(30.0));

    static std::ofstream puschTrace;
    if (!trace.empty())
    {
        puschTrace.open(trace);
        puschTrace << "t_s,rnti,rbs,mcs,tbs,sinr_db,rv,corrupt\n";
        NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)
            ->GetSpectrumPhy()
            ->TraceConnectWithoutContext("RxPacketTraceGnb", MakeBoundCallback(
                                             +[](std::ofstream* f, RxPacketTraceParams p) {
                                                 if (p.m_pdcchMissed || p.m_rbAssignedNum == 0)
                                                 {
                                                     return;
                                                 }
                                                 *f << Simulator::Now().GetSeconds() << ","
                                                    << p.m_rnti << ","
                                                    << static_cast<int>(p.m_rbAssignedNum) << ","
                                                    << static_cast<int>(p.m_mcs) << ","
                                                    << p.m_tbSize << ","
                                                    << 10.0 * std::log10(std::max(p.m_sinr, 1e-9))
                                                    << "," << static_cast<int>(p.m_rv) << ","
                                                    << (p.m_corrupt ? 1 : 0) << "\n";
                                             },
                                             &puschTrace));
    }

    auto [remoteHost, remoteAddr] = epc->SetupRemoteHost("100Gb/s", 2500, Seconds(0.0));
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIp = epc->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
    nr->AttachToClosestGnb(ueDevs, gnbDevs);

    const uint16_t port = 1234;
    UdpServerHelper ueSink(port);
    UdpServerHelper hostSink(port);
    ApplicationContainer servers = ueSink.Install(ueNodes);
    ApplicationContainer hostServers = hostSink.Install(remoteHost);
    const Time start = MilliSeconds(400);
    const uint32_t pkt = 1252;
    const NrQosFlow flow(NrQosFlow::NGBR_LOW_LAT_EMBB);
    Ptr<NrQosRule> rule = Create<NrQosRule>();
    NrQosRule::PacketFilter pfDl;
    pfDl.localPortStart = port; // downlink: the port of the UE's sink
    pfDl.localPortEnd = port;
    rule->Add(pfDl);
    NrQosRule::PacketFilter pfUl;
    pfUl.remotePortStart = port; // uplink: the port of the sink at the remote host
    pfUl.remotePortEnd = port;
    rule->Add(pfUl);
    ApplicationContainer clients;
    for (uint32_t i = 0; i < nUes; ++i)
    {
        if (dl)
        {
            UdpClientHelper c;
            c.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
            c.SetAttribute("PacketSize", UintegerValue(pkt));
            c.SetAttribute("Interval", TimeValue(Seconds(pkt * 8.0 / (dlMbps * 1e6))));
            c.SetAttribute("Remote",
                           AddressValue(addressUtils::ConvertToSocketAddress(ueIp.GetAddress(i), port)));
            clients.Add(c.Install(remoteHost));
        }
        if (ul)
        {
            UdpClientHelper c;
            c.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
            c.SetAttribute("PacketSize", UintegerValue(pkt));
            c.SetAttribute("Interval", TimeValue(Seconds(pkt * 8.0 / (ulMbps * 1e6))));
            c.SetAttribute("Remote", AddressValue(addressUtils::ConvertToSocketAddress(remoteAddr, port)));
            clients.Add(c.Install(ueNodes.Get(i)));
        }
        nr->ActivateDedicatedQosFlow(ueDevs.Get(i), flow, rule);
    }
    servers.Start(start);
    hostServers.Start(start);
    clients.Start(start);
    servers.Stop(Seconds(simS));
    hostServers.Stop(Seconds(simS));
    clients.Stop(Seconds(simS));

    FlowMonitorHelper fm;
    NodeContainer ends;
    ends.Add(remoteHost);
    ends.Add(ueNodes);
    Ptr<FlowMonitor> mon = fm.Install(ends);
    Simulator::Stop(Seconds(simS));
    Simulator::Run();
    mon->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> cls = DynamicCast<Ipv4FlowClassifier>(fm.GetClassifier());

    std::vector<double> dlRate(nUes, 0.0);
    std::vector<double> ulRate(nUes, 0.0);
    std::vector<double> dlDelay(nUes, 0.0);
    std::vector<double> ulDelay(nUes, 0.0);
    const double dur = simS - start.GetSeconds();
    for (const auto& [id, st] : mon->GetFlowStats())
    {
        const auto t = cls->FindFlow(id);
        const double mbps = st.rxBytes * 8.0 / dur / 1e6;
        const double dly = st.rxPackets ? 1e3 * st.delaySum.GetSeconds() / st.rxPackets : 0.0;
        for (uint32_t i = 0; i < nUes; ++i)
        {
            if (t.destinationAddress == ueIp.GetAddress(i))
            {
                dlRate[i] = mbps;
                dlDelay[i] = dly;
            }
            else if (t.sourceAddress == ueIp.GetAddress(i))
            {
                ulRate[i] = mbps;
                ulDelay[i] = dly;
            }
        }
    }
    std::ofstream out;
    if (!csv.empty())
    {
        out.open(csv, std::ios::app);
    }
    const uint32_t seed = RngSeedManager::GetRun();
    auto summary = [&](const char* name, const std::vector<double>& r) {
        const double sum = std::accumulate(r.begin(), r.end(), 0.0);
        double sq = 0.0;
        for (double x : r)
        {
            sq += x * x;
        }
        const double jain = sq > 0 ? sum * sum / (r.size() * sq) : 0.0;
        std::cout << std::setw(4) << name << " sum " << std::setw(8) << sum << " Mbps  min "
                  << *std::min_element(r.begin(), r.end()) << "  max "
                  << *std::max_element(r.begin(), r.end()) << "  Jain " << jain << "\n";
    };
    std::cout << scheduler << " alpha " << alpha << " seed " << seed << " nUes " << nUes << "\n";
    if (dl)
    {
        summary("DL", dlRate);
    }
    if (ul)
    {
        summary("UL", ulRate);
    }
    for (uint32_t i = 0; i < nUes; ++i)
    {
        std::cout << "  UE " << i << " " << std::setw(6) << dist[i] << " m  DL " << std::setw(7)
                  << dlRate[i] << "  UL " << std::setw(7) << ulRate[i] << "  delay " << dlDelay[i]
                  << " / " << ulDelay[i] << " ms\n";
        if (out.is_open())
        {
            out << scheduler << "," << alpha << "," << seed << "," << i << "," << dist[i] << ",dl,"
                << dlRate[i] << "," << dlDelay[i] << "\n"
                << scheduler << "," << alpha << "," << seed << "," << i << "," << dist[i] << ",ul,"
                << ulRate[i] << "," << ulDelay[i] << "\n";
        }
    }
    Simulator::Destroy();
    return 0;
}
