// Drives NrPdcch with a random load and prints everything it decides, for pdcchcheck.py to
// compute again from TS 38.211 / 38.213 and the rules in nr-pdcch.h, and compare.
//
//   ./ns3 run pdcchtest > pdcchtest.log && python3 pdcchcheck.py pdcchtest.log

#include "ns3/core-module.h"
#include "ns3/nr-module.h"

#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

using namespace ns3;

namespace
{

struct TestConfig
{
    const char* name;
    uint32_t rbs;
    uint32_t symbols; // 0 = dynamic
    uint32_t dciBits;
    uint32_t coresetRbs;
    double ulShare;
    bool raise;
    const char* candidates;
};

struct Asked
{
    uint16_t rnti;
    NrPdcch::Purpose purpose;
    NrPdcch::Grant grant;
    bool released;
    std::shared_ptr<DciInfoElementTdma> dci;
};

std::shared_ptr<DciInfoElementTdma>
MakeDci(uint16_t rnti, bool dl)
{
    return std::make_shared<DciInfoElementTdma>(rnti,
                                                dl ? DciInfoElementTdma::DL : DciInfoElementTdma::UL,
                                                1,
                                                1,
                                                0,
                                                1,
                                                nullptr,
                                                10,
                                                1,
                                                0,
                                                DciInfoElementTdma::DATA,
                                                0,
                                                0);
}

void
Slot(Ptr<NrPdcch> pdcch, const TestConfig* cfg, Ptr<UniformRandomVariable> rv, uint32_t nUes, uint64_t slot)
{
    std::vector<Asked> asked;
    auto ask = [&](uint16_t rnti, NrPdcch::Purpose purpose) {
        Asked a;
        a.rnti = rnti;
        a.purpose = purpose;
        a.grant = pdcch->Request(rnti, purpose);
        a.released = false;
        asked.push_back(a);
    };
    // The uplink scheduler first, as in the nr module with NrPdcch. (Nothing is asked for in
    // the very first slot: NrPdcch learns there which slot the schedulers work on.)
    pdcch->StartUplink();
    if (slot > 0 && rv->GetValue() < 0.05)
    {
        ask(0, NrPdcch::RAR);
    }
    const uint32_t nUl = slot > 0 ? rv->GetInteger(0, 6) : 0;
    for (uint32_t i = 0; i < nUl; ++i)
    {
        const auto rnti = static_cast<uint16_t>(rv->GetInteger(1, nUes));
        static const NrPdcch::Purpose kinds[] = {NrPdcch::UL_NEW,
                                                 NrPdcch::UL_RETX,
                                                 NrPdcch::UL_CG_ACTIVATION,
                                                 NrPdcch::UL_CG_RELEASE};
        ask(rnti, kinds[rv->GetInteger(0, 3)]);
    }
    // Then the downlink scheduler, for the slot two ahead
    SfnSf target(0, 0, 0, 0);
    target.Add(slot + 2);
    const bool defer = pdcch->StartDownlink(target);
    NS_ABORT_MSG_IF(defer, "the uplink was asked first: nothing to wait for");
    const uint32_t nDl = slot > 0 ? rv->GetInteger(0, 8) : 0;
    for (uint32_t i = 0; i < nDl; ++i)
    {
        const auto rnti = static_cast<uint16_t>(rv->GetInteger(1, nUes));
        static const NrPdcch::Purpose kinds[] = {NrPdcch::DL_NEW,
                                                 NrPdcch::DL_RETX,
                                                 NrPdcch::DL_SPS_ACTIVATION,
                                                 NrPdcch::DL_SPS_RELEASE};
        ask(rnti, kinds[rv->GetInteger(0, 3)]);
    }
    // Some reservations come to nothing, the others become DCIs
    for (auto& a : asked)
    {
        if (!a.grant)
        {
            continue;
        }
        if (rv->GetValue() < 0.1)
        {
            pdcch->Release(a.grant);
            a.released = true;
        }
    }
    for (auto& a : asked)
    {
        if (a.grant && !a.released)
        {
            a.dci = MakeDci(a.rnti, a.purpose < NrPdcch::UL_NEW);
            pdcch->Commit(a.grant, a.purpose, a.dci);
        }
    }
    const uint8_t symbols = pdcch->FixSymbols();
    std::cout << "SLOT " << cfg->name << " " << slot + 2 << " " << +symbols << "\n";
    for (const auto& a : asked)
    {
        std::cout << "ASK " << cfg->name << " " << slot + 2 << " " << a.rnti << " " << +a.purpose
                  << " " << +a.grant.al << " " << a.released << " "
                  << (a.dci ? static_cast<int>(a.dci->m_pdcchCce) : -1) << " "
                  << (a.dci ? +a.dci->m_pdcchSymbols : 0) << " " << (a.dci ? +a.dci->m_pdcchAl : 0)
                  << "\n";
    }
}

} // namespace

int
main(int argc, char* argv[])
{
    uint32_t slots = 400;
    CommandLine cmd(__FILE__);
    cmd.AddValue("slots", "Slots per configuration", slots);
    cmd.Parse(argc, argv);

    const TestConfig configs[] = {
        {"A", 26, 1, 34, 0, 0.5, true, "6,6,4,2,1"},  // 5 MHz, 4 CCEs
        {"B", 52, 0, 34, 0, 0.5, true, "6,6,4,2,1"},  // 10 MHz, 8/16/24 CCEs, dynamic
        {"C", 160, 1, 34, 0, 0.5, true, "6,6,4,2,1"}, // 30 MHz, 26 CCEs
        {"D", 106, 2, 60, 48, 0.25, false, "4,4,2,2,0"}, // other settings: 48 RB x 2 = 16 CCEs
        {"E", 79, 3, 12, 0, 1.0, true, "8,8,8,8,8"},  // 15 MHz, 3 symbols: 39 CCEs
        {"F", 160, 0, 34, 0, 0.0, true, "6,6,4,2,1"}, // dynamic, no share for the uplink
    };
    std::cout << std::setprecision(10);
    for (const TestConfig& cfg : configs)
    {
        Ptr<NrPdcch> pdcch = CreateObject<NrPdcch>();
        pdcch->SetAttribute("Enabled", BooleanValue(true));
        pdcch->SetAttribute("Symbols", UintegerValue(cfg.symbols));
        pdcch->SetAttribute("DciBits", UintegerValue(cfg.dciBits));
        pdcch->SetAttribute("CoresetRbs", UintegerValue(cfg.coresetRbs));
        pdcch->SetAttribute("UlMaxShare", DoubleValue(cfg.ulShare));
        pdcch->SetAttribute("RaiseAl", BooleanValue(cfg.raise));
        pdcch->SetAttribute("Candidates", StringValue(cfg.candidates));
        pdcch->SetCarrier(cfg.rbs, 0);
        std::cout << "CONFIG " << cfg.name << " " << cfg.rbs << " " << cfg.symbols << " "
                  << cfg.dciBits << " " << cfg.coresetRbs << " " << cfg.ulShare << " " << cfg.raise
                  << " " << cfg.candidates << " " << pdcch->GetCoresetRbs() << " "
                  << pdcch->GetNumCce(1) << " " << pdcch->GetNumCce(2) << " " << pdcch->GetNumCce(3)
                  << " " << +pdcch->GetCommonAl() << "\n";
        for (const uint8_t al : {1, 2, 4, 8, 16})
        {
            std::cout << "LEVEL " << cfg.name << " " << +al << " " << pdcch->GetCandidates(al) << " "
                      << pdcch->GetSinrForBler(al, 0.01) << " " << pdcch->GetBler(al, -3.3) << " "
                      << pdcch->GetBler(al, 0.7) << " " << pdcch->GetBler(al, -9.0, 4.0) << "\n";
        }
        Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable>();
        rv->SetStream(4711);
        const uint32_t nUes = 40;
        for (uint16_t rnti = 1; rnti <= nUes; ++rnti)
        {
            // One UE in eight has not reported yet; the others are anywhere from -15 to +6 dB
            if (rnti % 8 == 0)
            {
                std::cout << "UE " << cfg.name << " " << rnti << " nan\n";
                continue;
            }
            const double sinr = std::floor(rv->GetValue(-15.0, 6.0) * 2.0) / 2.0;
            pdcch->ReportSinr(rnti, sinr);
            std::cout << "UE " << cfg.name << " " << rnti << " " << sinr << "\n";
        }
        for (uint64_t slot = 0; slot < slots; ++slot)
        {
            Simulator::Schedule(MilliSeconds(slot), &Slot, pdcch, &cfg, rv, nUes, slot);
        }
        Simulator::Stop(MilliSeconds(slots));
        Simulator::Run();
        const NrPdcch::Stats st = pdcch->GetStats();
        std::cout << "STATS " << cfg.name << " " << st.slots << " " << st.slotsBySymbols[0] << " "
                  << st.slotsBySymbols[1] << " " << st.slotsBySymbols[2] << " "
                  << st.slotsBySymbols[3] << " " << st.cceAvailable << " " << st.cceUsed << " "
                  << st.aboveTarget << " " << st.raised << " " << st.noReport << " "
                  << st.unusedReservations << " " << st.ulCapped;
        for (std::size_t i = 0; i < NrPdcch::N_PURPOSES; ++i)
        {
            std::cout << " " << st.dci[i] << " " << st.cce[i] << " " << st.blocked[i];
        }
        for (std::size_t i = 0; i < 5; ++i)
        {
            std::cout << " " << st.dciByAl[i] << " " << st.cceByAl[i];
        }
        std::cout << "\n";
        Simulator::Destroy();
    }
    return 0;
}
