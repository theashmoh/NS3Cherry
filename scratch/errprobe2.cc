// BLER of the nr module's error model (NrEesmIrT1) for a flat SINR over the RBs, with K identical
// transmissions combined (the history of NrEesmErrorModel, as a HARQ process or PUSCH repetition
// uses it). Prints the SINR (dB) per transmission at which the block error rate is 10 % and 1 %.
#include "ns3/core-module.h"
#include "ns3/nr-module.h"
#include "ns3/spectrum-module.h"

#include <cmath>
#include <iostream>
#include <iomanip>

using namespace ns3;

double
Bler(Ptr<NrEesmIrT1> em, Ptr<SpectrumModel> sm, uint32_t nRb, uint32_t sizeBytes, uint8_t mcs,
     double sinrDb, uint32_t k)
{
    SpectrumValue sinr(sm);
    for (uint32_t i = 0; i < nRb; ++i)
    {
        sinr[i] = std::pow(10.0, sinrDb / 10.0);
    }
    std::vector<int> map;
    for (uint32_t i = 0; i < nRb; ++i)
    {
        map.push_back(i);
    }
    NrErrorModel::NrErrorModelHistory hist;
    Ptr<NrErrorModelOutput> out;
    for (uint32_t t = 0; t < k; ++t)
    {
        out = em->GetTbDecodificationStats(sinr, map, sizeBytes, mcs, hist);
        hist.push_back(out);
    }
    return out->m_tbler;
}

double
SinrFor(Ptr<NrEesmIrT1> em, Ptr<SpectrumModel> sm, uint32_t nRb, uint32_t size, uint8_t mcs,
        uint32_t k, double target)
{
    double lo = -40.0;
    double hi = 40.0;
    for (int i = 0; i < 50; ++i)
    {
        const double mid = 0.5 * (lo + hi);
        (Bler(em, sm, nRb, size, mcs, mid, k) > target ? lo : hi) = mid;
    }
    return hi;
}

int
main(int argc, char* argv[])
{
    Ptr<NrEesmIrT1> em = CreateObject<NrEesmIrT1>();
    Bands bands;
    for (uint32_t i = 0; i < 275; ++i)
    {
        BandInfo b;
        b.fl = 2.0e9 + i * 180e3;
        b.fh = b.fl + 180e3;
        b.fc = 0.5 * (b.fl + b.fh);
        bands.push_back(b);
    }
    Ptr<SpectrumModel> sm = Create<SpectrumModel>(bands);
    // R1-2205058 PUSCH: TBS 176 bit (22 B), 1 RB, QPSK, 14 symbols; compare plain-repetition curves
    // (their Fig. 2: K=4/8/16 reach BLER 0.1 at about -0.2/-3.0/-5.3 dB; ideal estimation here).
    std::cout << "table,usefulSc,mcs,payload_bytes,K,sinr10_db,sinr1_db" << std::endl;
    for (uint32_t table : {1u, 2u})
    {
        for (uint32_t useful : {11u, 12u})
        {
            for (uint8_t mcs = 0; mcs < 12; ++mcs)
            {
                const uint32_t payload = em->GetPayloadSize(useful, mcs, 1, 12 /*1 RB*/, NrErrorModel::UL);
                if (payload < 22 || payload > 30)
                {
                    continue;
                }
                for (uint32_t k : {1u, 4u, 8u, 16u})
                {
                    std::cout << table << "," << useful << "," << +mcs << "," << payload << "," << k << ","
                              << std::fixed << std::setprecision(2)
                              << SinrFor(em, sm, 1, 22, mcs, k, 0.1) << ","
                              << SinrFor(em, sm, 1, 22, mcs, k, 0.01) << std::endl;
                }
            }
        }
        break;  // MCS table is the object default; one pass
    }
    return 0;
}
