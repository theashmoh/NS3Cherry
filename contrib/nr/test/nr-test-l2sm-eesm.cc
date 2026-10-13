// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/enum.h"
#include "ns3/nr-eesm-cc-t1.h"
#include "ns3/nr-eesm-cc-t2.h"
#include "ns3/nr-eesm-error-model.h"
#include "ns3/nr-eesm-ir-t1.h"
#include "ns3/nr-eesm-ir-t2.h"
#include "ns3/nr-spectrum-value-helper.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

#include <cmath>

/**
 * @file nr-test-l2sm-eesm.cc
 * @ingroup test
 *
 * @brief This test validates specific functions of the NR PHY abstraction model.
 * The test checks three issues: 1) LDPC base graph (BG) selection works properly, 2)
 * BLER values are properly obtained from the BLER-SINR look up tables for different
 * block sizes, MCS Tables, BG types, and SINR values, and 3) the HARQ-IR soft buffer
 * bounds the equivalent code rate of retransmissions.
 *
 */
namespace ns3
{

/**
 * @brief NrL2smEesm testcase
 */
class NrL2smEesmTestCase : public TestCase
{
  public:
    NrL2smEesmTestCase(const std::string& name)
        : TestCase(name)
    {
    }

    /**
     * @brief Destroy the object instance
     */
    ~NrL2smEesmTestCase() override
    {
    }

  private:
    void DoRun() override;

    void TestMappingSinrBler1(const Ptr<NrEesmErrorModel>& em);
    void TestMappingSinrBler2(const Ptr<NrEesmErrorModel>& em);
    void TestBgType1(const Ptr<NrEesmErrorModel>& em);
    void TestBgType2(const Ptr<NrEesmErrorModel>& em);

    void TestEesmCcTable1();
    void TestEesmCcTable2();
    void TestEesmIrTable1();
    void TestEesmIrTable2();
    void TestSoftBufferLimit();
};

void
NrL2smEesmTestCase::TestBgType1(const Ptr<NrEesmErrorModel>& em)
{
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 18),
                          NrEesmErrorModel::SECOND,
                          "TestBgType1-a: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3900, 18),
                          NrEesmErrorModel::FIRST,
                          "TestBgType1-b: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(200, 18),
                          NrEesmErrorModel::SECOND,
                          "TestBgType1-c: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(4000, 0),
                          NrEesmErrorModel::SECOND,
                          "TestBgType1-d: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 28),
                          NrEesmErrorModel::FIRST,
                          "TestBgType1-e: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 2),
                          NrEesmErrorModel::SECOND,
                          "TestBgType2-f: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 16),
                          NrEesmErrorModel::SECOND,
                          "TestBgType2-g: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3900, 14),
                          NrEesmErrorModel::FIRST,
                          "TestBgType2-h: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
}

void
NrL2smEesmTestCase::TestBgType2(const Ptr<NrEesmErrorModel>& em)
{
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 18),
                          NrEesmErrorModel::FIRST,
                          "TestBgType2-a: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3900, 18),
                          NrEesmErrorModel::FIRST,
                          "TestBgType2-b: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(200, 18),
                          NrEesmErrorModel::SECOND,
                          "TestBgType2-c: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(4000, 0),
                          NrEesmErrorModel::SECOND,
                          "TestBgType2-d: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 27),
                          NrEesmErrorModel::FIRST,
                          "TestBgType2-e: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 2),
                          NrEesmErrorModel::SECOND,
                          "TestBgType2-f: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 16),
                          NrEesmErrorModel::FIRST,
                          "TestBgType2-g: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3900, 14),
                          NrEesmErrorModel::FIRST,
                          "TestBgType2-h: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
}

typedef std::tuple<double, uint8_t, uint32_t, double> MappingTable;

static std::vector<MappingTable> resultTable1 = {
    // sinr (lineal), mcs, cbsize, result

    // MCS 18, all CBS in continuation use BGtype2
    // CBS=3200, in table corresponds to 3104
    MappingTable{19.95, 18, 3200, 0.023},      // sinr 13 db
    MappingTable{15.849, 18, 3200, 0.7567365}, // sinr 12 db
    MappingTable{10, 18, 3200, 1.00},          // sinr 10 db
    // CBS=3500, in table corresponds to 3496
    MappingTable{19.95, 18, 3500, 0.0735},     // sinr 13 db
    MappingTable{15.849, 18, 3500, 0.7908951}, // sinr 12 db
    MappingTable{10, 18, 3500, 1.00},          // sinr 10 db

    // MCS 14, all CBS in continuation use BGtype1
    // CBS=3900, in table corresponds to 3840
    MappingTable{8.9125, 14, 3900, 0.3225703}, // sinr 9.5db
    MappingTable{7.9433, 14, 3900, 0.8827055}, // sinr 9 db
    MappingTable{6.3095, 14, 3900, 1.00},      // sinr 8 db
    // CBS=6300, in table corresponds to 6272
    MappingTable{8.9125, 14, 6300, 0.0237},    // sinr 9.5db
    MappingTable{7.9433, 14, 6300, 0.9990385}, // sinr 9 db
    MappingTable{6.3095, 14, 6300, 1.00}       // sinr 8 db

};
static std::vector<MappingTable> resultTable2 = {
    // sinr (lineal), mcs, cbsize, result

    // MCS 11, all CBS in continuation use BGtype2
    // CBS=3200, in table corresponds to 3104
    MappingTable{19.95, 11, 3200, 0.023},      // sinr 13 db
    MappingTable{15.849, 11, 3200, 0.7567365}, // sinr 12 db
    MappingTable{10, 11, 3200, 1.00},          // sinr 10 db
    // CBS=3500, in table corresponds to 3496
    MappingTable{19.95, 11, 3500, 0.0735},     // sinr 13 db
    MappingTable{15.849, 11, 3500, 0.7908951}, // sinr 12 db
    MappingTable{10, 11, 3500, 1.00},          // sinr 10 db

    // MCS 8, all CBS in continuation use BGtype1
    // CBS=3900, in table corresponds to 3840
    MappingTable{8.9125, 8, 3900, 0.3225703}, // sinr 9.5db
    MappingTable{7.9433, 8, 3900, 0.8827055}, // sinr 9 db
    MappingTable{6.3095, 8, 3900, 1.00},      // sinr 8 db
    // CBS=6300, in table corresponds to 6272
    MappingTable{8.9125, 8, 6300, 0.0237},    // sinr 9.5db
    MappingTable{7.9433, 8, 6300, 0.9990385}, // sinr 9 db
    MappingTable{6.3095, 8, 6300, 1.00}       // sinr 8 db

};

void
NrL2smEesmTestCase::TestMappingSinrBler1(const Ptr<NrEesmErrorModel>& em)
{
    for (auto result : resultTable1)
    {
        NS_TEST_ASSERT_MSG_EQ(
            em->MappingSinrBler(std::get<0>(result), std::get<1>(result), std::get<2>(result)),
            std::get<3>(result),
            "TestMappingSinrBler1: The calculated value differs from "
            " the SINR-BLER table. SINR="
                << std::get<0>(result) << " MCS " << static_cast<uint32_t>(std::get<1>(result))
                << " CBS " << std::get<2>(result));
    }
}

void
NrL2smEesmTestCase::TestMappingSinrBler2(const Ptr<NrEesmErrorModel>& em)
{
    for (auto result : resultTable2)
    {
        NS_TEST_ASSERT_MSG_EQ(
            em->MappingSinrBler(std::get<0>(result), std::get<1>(result), std::get<2>(result)),
            std::get<3>(result),
            "TestMappingSinrBler2: The calculated value differs from "
            " the SINR-BLER table. SINR="
                << std::get<0>(result) << " MCS " << static_cast<uint32_t>(std::get<1>(result))
                << " CBS " << std::get<2>(result));
    }
}

void
NrL2smEesmTestCase::TestEesmCcTable1()
{
    // Create an object of type NrEesmCcT1 and cast it to NrEesmErrorModel
    Ptr<NrEesmErrorModel> em = CreateObject<NrEesmCcT1>();

    // Check that the object was created
    bool ret = em == nullptr;
    NS_TEST_ASSERT_MSG_EQ(ret, false, "Could not create NrEesmCcT1 object");

    // Test here the functions:
    TestBgType1(em);
    TestMappingSinrBler1(em);
}

void
NrL2smEesmTestCase::TestEesmCcTable2()
{
    // Create an object of type NrEesmCcT2 and cast it to NrEesmErrorModel
    Ptr<NrEesmErrorModel> em = CreateObject<NrEesmCcT2>();

    // Check that the object was created
    bool ret = em == nullptr;
    NS_TEST_ASSERT_MSG_EQ(ret, false, "Could not create NrEesmCcT2 object");

    // Test here the functions:
    TestBgType2(em);
    TestMappingSinrBler2(em);
}

void
NrL2smEesmTestCase::TestEesmIrTable1()
{
    // Create an object of type NrEesmIrT1 and cast it to NrEesmErrorModel
    Ptr<NrEesmErrorModel> em = CreateObject<NrEesmIrT1>();

    // Check that the object was created
    bool ret = em == nullptr;
    NS_TEST_ASSERT_MSG_EQ(ret, false, "Could not create NrEesmIrT1 object");

    // Test here the functions:
    TestBgType1(em);
    TestMappingSinrBler1(em);
}

void
NrL2smEesmTestCase::TestEesmIrTable2()
{
    // Create an object of type NrEesmIrT2 and cast it to NrEesmErrorModel
    Ptr<NrEesmErrorModel> em = CreateObject<NrEesmIrT2>();

    // Check that the object was created
    bool ret = em == nullptr;
    NS_TEST_ASSERT_MSG_EQ(ret, false, "Could not create NrEesmIrT2 object");

    // Test here the functions:
    TestBgType2(em);
    TestMappingSinrBler2(em);
}

void
NrL2smEesmTestCase::TestSoftBufferLimit()
{
    // A peak-rate TB and its first retransmission on a 100 MHz carrier at 30 kHz (273 PRBs),
    // one layer, MCS Table 1, so 64QAM at most. Limited-buffer rate matching (TS 38.212
    // Sect. 5.4.2.1) sizes the soft buffer from TBS_LBRM, the TBS of TS 38.214 Sect. 5.1.3.2
    // for N_RE = 156 * 273, R = 948/1024, Qm = 6 and one layer
    const uint32_t tbsLbrm = 237776;
    // The TB fills the 273 PRBs with 144 data REs each at MCS 28 (R = 948/1024), and has C = 26
    // code blocks of K = 8448 bits (BG1, Zc = 384)
    const uint32_t numRbs = 273;
    const uint32_t tbSizeBytes = 217128 / 8;
    const uint8_t mcs = 28;
    const uint32_t numCbs = 26;
    // Ncb = min(N, Nref), Nref = floor(TBS_LBRM / (C * R_LBRM)) and R_LBRM = 2/3. Here
    // Nref = 13717 is below N = 66 * 384, so the TB buffers 26 * 13717 = 356642 coded bits:
    // about 1.5 transmissions of 234534 coded bits each
    const uint32_t softBufferBitsPerTb = numCbs * (3 * tbsLbrm / (2 * numCbs));

    Ptr<const SpectrumModel> spectrumModel =
        NrSpectrumValueHelper::GetSpectrumModel(numRbs, 3.5e9, 30000);
    SpectrumValue sinr(spectrumModel);
    sinr += std::pow(10, 15.0 / 10); // 15 dB: MCS 28 fails, MCS 19 decodes and MCS 22 fails

    std::vector<int> rbBitmap;
    rbBitmap.reserve(numRbs);
    for (uint32_t i = 0; i < numRbs; ++i)
    {
        rbBitmap.push_back(static_cast<int>(i));
    }

    Ptr<NrEesmErrorModel> em = CreateObject<NrEesmIrT1>();
    UintegerValue defaultValue;
    em->GetAttribute("SoftBufferBitsPerTb", defaultValue);
    NS_TEST_ASSERT_MSG_EQ(defaultValue.Get(),
                          0,
                          "TestSoftBufferLimit: the soft buffer must default to unlimited");

    Ptr<NrErrorModelOutput> first =
        em->GetTbDecodificationStats(sinr, rbBitmap, tbSizeBytes, mcs, {});
    const uint32_t codeBitsPerTx = DynamicCast<NrEesmErrorModelOutput>(first)->m_codeBits;
    NS_TEST_ASSERT_MSG_EQ(codeBitsPerTx, 234534, "TestSoftBufferLimit: unexpected coded bits");
    const NrErrorModel::NrErrorModelHistory history{first};

    // Unlimited, the retransmission halves the ECR to 0.463, which maps to MCS 19 (ECR 0.505),
    // two steps above the 64QAM floor at MCS 17 (ECR 0.428)
    const double unlimited =
        em->GetTbDecodificationStats(sinr, rbBitmap, tbSizeBytes, mcs, history)->m_tbler;
    NS_TEST_ASSERT_MSG_EQ(em->GetMcsEq(mcs),
                          19,
                          "TestSoftBufferLimit: unlimited IR must map to MCS 19");

    // A buffer that holds both transmissions is never full, so nothing changes
    em->SetAttribute("SoftBufferBitsPerTb", UintegerValue(2 * codeBitsPerTx));
    NS_TEST_ASSERT_MSG_EQ(
        em->GetTbDecodificationStats(sinr, rbBitmap, tbSizeBytes, mcs, history)->m_tbler,
        unlimited,
        "TestSoftBufferLimit: a soft buffer that holds every transmission must reproduce the "
        "unlimited result");
    NS_TEST_ASSERT_MSG_EQ(
        em->GetMcsEq(mcs),
        19,
        "TestSoftBufferLimit: a soft buffer that holds every transmission must map to MCS 19");

    // The LBRM buffer holds 1.5 transmissions, so the ECR only falls to 217128 / 356642 = 0.609,
    // which maps to MCS 22 (ECR 0.650): the retransmission no longer decodes
    em->SetAttribute("SoftBufferBitsPerTb", UintegerValue(softBufferBitsPerTb));
    const double limited =
        em->GetTbDecodificationStats(sinr, rbBitmap, tbSizeBytes, mcs, history)->m_tbler;
    NS_TEST_ASSERT_MSG_EQ(em->GetMcsEq(mcs),
                          22,
                          "TestSoftBufferLimit: the LBRM soft buffer must map to MCS 22");
    NS_TEST_ASSERT_MSG_GT(limited,
                          unlimited,
                          "TestSoftBufferLimit: a full soft buffer must not keep earning "
                          "incremental-redundancy gain");
}

void
NrL2smEesmTestCase::DoRun()
{
    TestEesmCcTable1();
    TestEesmCcTable2();
    TestEesmIrTable1();
    TestEesmIrTable2();
    TestSoftBufferLimit();
}

class NrTestL2smEesm : public TestSuite
{
  public:
    NrTestL2smEesm()
        : TestSuite("nr-test-l2sm-eesm", Type::UNIT)
    {
        AddTestCase(new NrL2smEesmTestCase("First test"), Duration::QUICK);
    }
};

static NrTestL2smEesm NrTestL2smEesmTestSuite; //!< Nr test suite

} // namespace ns3
