// Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-amc.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/object-factory.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

namespace ns3
{

/**
 * @ingroup test
 * @brief Check configuration, numerology scaling, timing maps and the RAR grant.
 */
class NrKoffsetTestCase : public TestCase
{
  public:
    NrKoffsetTestCase()
        : TestCase("Cell-specific K_offset configuration and grant timing")
    {
    }

  private:
    void DoRun() override
    {
        ObjectFactory factory;
        factory.SetTypeId(NrGnbPhy::GetTypeId());
        factory.Set("CellSpecificKoffset", UintegerValue(40));
        factory.Set("Numerology", UintegerValue(1));
        auto phy = factory.Create<NrGnbPhy>();
        NS_TEST_ASSERT_MSG_EQ(phy->GetCellSpecificKoffset(), 40, "Factory configuration failed");
        NS_TEST_ASSERT_MSG_EQ(phy->GetCellSpecificKoffsetSlots(), 80, "Use 15 kHz reference slots");
        NS_TEST_ASSERT_MSG_EQ(phy->m_dlHarqfbPosition.at(0), 82, "Factory order left stale maps");
        NS_TEST_ASSERT_MSG_EQ(phy->SetAttributeFailSafe("CellSpecificKoffset", UintegerValue(1024)),
                              false,
                              "Reject values outside NTN-Config's range");
        NS_TEST_ASSERT_MSG_EQ(phy->GetCellSpecificKoffset(), 40, "Rejected value changed state");

        // Configure through the base PHY as well as through the attribute. Both must
        // rebuild the maps when the numerology changes after the offset was set.
        Ptr<NrPhy> basePhy = phy;
        for (uint16_t mu : {0, 1, 2, 5})
        {
            basePhy->SetNumerology(mu);
            for (uint32_t offset : {0u, 1u, 40u, 1023u})
            {
                NS_TEST_ASSERT_MSG_EQ(
                    phy->SetAttributeFailSafe("CellSpecificKoffset", UintegerValue(offset)),
                    true,
                    "Valid attribute value rejected");
                const uint32_t slots = offset * (1u << mu);
                NS_TEST_ASSERT_MSG_EQ(phy->GetPhySapProvider()->GetCellSpecificKoffsetSlots(),
                                      slots,
                                      "The MAC must see the converted offset");
                for (uint32_t slot = 0; slot < 10; ++slot)
                {
                    NS_TEST_EXPECT_MSG_EQ(phy->m_dlHarqfbPosition.at(slot), 2 + slots, "Wrong K1");
                    NS_TEST_EXPECT_MSG_EQ(phy->m_toSendUl.at(slot).size(),
                                          1,
                                          "Wrong UL grant count");
                    NS_TEST_EXPECT_MSG_EQ(phy->m_toSendUl.at(slot).at(0), 2 + slots, "Wrong K2");
                    NS_TEST_EXPECT_MSG_EQ(phy->m_generateUl.at(slot).at(0),
                                          4 + slots,
                                          "UL allocation and grant must move together");
                    NS_TEST_EXPECT_MSG_EQ(phy->m_toSendDl.at(slot).at(0), 0, "K0 must stay zero");
                    NS_TEST_EXPECT_MSG_EQ(phy->m_generateDl.at(slot).at(0),
                                          2,
                                          "DL scheduling latency must stay unchanged");
                }
            }
        }

        phy->SetAttribute("Numerology", UintegerValue(1));
        phy->SetCellSpecificKoffset(40);
        CheckMsg3(phy, 82);

        // Mixed TDD slots can round K1/K2 further to a usable UL/DL occasion.
        // Check every map entry rather than only the sum of all delays.
        const std::vector<LteNrTddSlotType> pattern{DL, DL, S, UL, UL};
        phy->SetPattern("DL|DL|S|UL|UL|");
        for (uint32_t offset : {0u, 1u, 7u, 40u})
        {
            phy->SetCellSpecificKoffset(offset);
            const auto minimum = 2 + 2 * offset;
            for (const auto& [slot, delay] : phy->m_dlHarqfbPosition)
            {
                NS_TEST_EXPECT_MSG_LT_OR_EQ(minimum, delay, "K1 below processing plus NTN delay");
                NS_TEST_EXPECT_MSG_LT(delay, minimum + pattern.size(), "K1 skipped an occasion");
                NS_TEST_EXPECT_MSG_EQ(pattern[(slot + delay) % pattern.size()] >= S,
                                      true,
                                      "HARQ feedback must land in an UL-capable slot");
            }
            for (const auto& [slot, delays] : phy->m_toSendUl)
            {
                for (auto delay : delays)
                {
                    NS_TEST_EXPECT_MSG_LT_OR_EQ(minimum, delay, "K2 below required delay");
                    NS_TEST_EXPECT_MSG_LT(delay,
                                          minimum + pattern.size(),
                                          "K2 skipped an occasion");
                    NS_TEST_EXPECT_MSG_EQ(pattern[slot] <= F, true, "DCI needs a DL-capable slot");
                    NS_TEST_EXPECT_MSG_EQ(pattern[(slot + delay) % pattern.size()],
                                          UL,
                                          "The UL grant must refer to the reserved UL slot");
                }
            }
        }
        Simulator::Destroy();
    }

    /**
     * @brief Verify the RAR carries the delay used to locate its Msg3 allocation.
     * @param phy Configured gNB PHY.
     * @param delay Expected grant delay in active slots.
     */
    void CheckMsg3(Ptr<NrGnbPhy> phy, uint32_t delay)
    {
        const SfnSf current(0, 0, 0, 1);
        SfnSf target = current;
        target.Add(delay);
        SlotAllocInfo allocation(target);
        NrBuildRarListElement_s rar;
        rar.raPreambleId = 1;
        rar.ulMsg3Dci = std::make_shared<DciInfoElementTdma>(1,
                                                             1,
                                                             DciInfoElementTdma::UL,
                                                             DciInfoElementTdma::MSG3,
                                                             std::vector<bool>(1, true));
        allocation.m_buildRarList.push_back(rar);
        phy->m_currentSlot = current;
        phy->PushBackSlotAllocInfo(allocation);
        auto messages = phy->RetrieveMsgsFromDCIs(current);
        NS_TEST_ASSERT_MSG_EQ(messages.size(), 1, "The Msg3 allocation must produce one RAR");
        auto message = DynamicCast<NrRarMessage>(messages.front());
        NS_TEST_ASSERT_MSG_NE(message, nullptr, "Expected a RAR message");
        NS_TEST_ASSERT_MSG_EQ(message->RarListBegin()->rarPayload.k2Delay,
                              delay,
                              "Msg3 delay must include the converted K_offset");
    }
};

/** @ingroup test
 * @brief Minimal control SAP for the K_offset scheduler tests.
 */
class KoffsetCschedSapUser : public NrMacCschedSapUser
{
  public:
    void CschedCellConfigCnf(const CschedCellConfigCnfParameters&) override
    {
    }

    void CschedUeConfigCnf(const CschedUeConfigCnfParameters&) override
    {
    }

    void CschedLcConfigCnf(const CschedLcConfigCnfParameters&) override
    {
    }

    void CschedLcReleaseCnf(const CschedLcReleaseCnfParameters&) override
    {
    }

    void CschedUeReleaseCnf(const CschedUeReleaseCnfParameters&) override
    {
    }

    void CschedUeConfigUpdateInd(const CschedUeConfigUpdateIndParameters&) override
    {
    }

    void CschedCellConfigUpdateInd(const CschedCellConfigUpdateIndParameters&) override
    {
    }
};

/** @ingroup test
 * @brief Supply timing from a real PHY and count scheduled retransmissions.
 */
class KoffsetSchedSapUser : public NrMacSchedSapUser
{
  public:
    Ptr<NrGnbPhy> m_phy;           ///< PHY supplying the offset and numerology.
    uint32_t m_retransmissions{0}; ///< Data retransmissions scheduled by the real scheduler.

    void SchedConfigInd(const SchedConfigIndParameters& params) override
    {
        for (const auto& allocation : params.m_slotAllocInfo.m_varTtiAllocInfo)
        {
            if (allocation.m_dci->m_type == DciInfoElementTdma::DATA &&
                allocation.m_dci->m_ndi == 0)
            {
                ++m_retransmissions;
            }
        }
    }

    Ptr<const SpectrumModel> GetSpectrumModel() const override
    {
        return nullptr;
    }

    uint32_t GetNumRbPerRbg() const override
    {
        return 1;
    }

    uint8_t GetNumHarqProcess() const override
    {
        return 16;
    }

    uint16_t GetBwpId() const override
    {
        return 0;
    }

    uint16_t GetCellId() const override
    {
        return 0;
    }

    uint32_t GetSymbolsPerSlot() const override
    {
        return 14;
    }

    Time GetSlotPeriod() const override
    {
        return m_phy->GetSlotPeriod();
    }

    uint32_t GetCellSpecificKoffsetSlots() const override
    {
        return m_phy->GetPhySapProvider()->GetCellSpecificKoffsetSlots();
    }

    void BuildRarList(SlotAllocInfo&) override
    {
    }
};

/**
 * @ingroup test
 * @brief Exercise late ACK/NACK, retransmission and missing-feedback expiration in both directions.
 */
class NrKoffsetHarqTestCase : public TestCase
{
  public:
    /**
     * @param uplink Whether to test UL rather than DL HARQ.
     * @param offset K_offset in reference slots.
     * @param numerology Active BWP numerology.
     */
    NrKoffsetHarqTestCase(bool uplink, uint32_t offset, uint16_t numerology)
        : TestCase(std::string(uplink ? "UL" : "DL") + " HARQ offset=" + std::to_string(offset) +
                   " numerology=" + std::to_string(numerology)),
          m_uplink(uplink),
          m_offset(offset),
          m_numerology(numerology)
    {
    }

  private:
    void DoRun() override
    {
        KoffsetCschedSapUser csched;
        KoffsetSchedSapUser sap;
        sap.m_phy = CreateObject<NrGnbPhy>();
        sap.m_phy->SetNumerology(m_numerology);
        sap.m_phy->SetCellSpecificKoffset(m_offset);
        auto scheduler = CreateObject<NrMacSchedulerTdmaRR>();
        scheduler->SetMacSchedSapUser(&sap);
        scheduler->SetMacCschedSapUser(&csched);
        scheduler->InstallDlAmc(CreateObject<NrAmc>());
        scheduler->InstallUlAmc(CreateObject<NrAmc>());
        scheduler->SetSrsInFSlots(false);
        scheduler->SetSrsInUlSlots(false);
        NrMacCschedSapProvider::CschedCellConfigReqParameters cell{};
        cell.m_dlBandwidth = 10;
        cell.m_ulBandwidth = 10;
        scheduler->DoCschedCellConfigReq(cell);
        NrMacCschedSapProvider::CschedUeConfigReqParameters ue{};
        ue.m_rnti = 1;
        ue.m_beamId = BeamId(0, 0);
        scheduler->DoCschedUeConfigReq(ue);
        auto info = scheduler->m_ueMap.at(1);
        auto& harq = m_uplink ? info->m_ulHarq : info->m_dlHarq;
        const auto offsetSlots = m_offset * (1u << m_numerology);
        const auto timeout = 16 + offsetSlots;
        uint8_t id = Insert(harq);

        // A missing feedback must eventually free the process, including offsets
        // above 255 slots (the old timer was only eight bits wide).
        for (uint32_t tick = 0; tick < timeout; ++tick)
        {
            scheduler->ResetExpiredHARQ(1, &harq);
        }
        NS_TEST_ASSERT_MSG_EQ(harq.Get(id).m_active,
                              true,
                              "Expired before the waiting window ended");
        scheduler->ResetExpiredHARQ(1, &harq);
        NS_TEST_ASSERT_MSG_EQ(harq.Get(id).m_active,
                              false,
                              "Missing feedback must eventually expire");
        NS_TEST_ASSERT_MSG_EQ(harq.Size(), 0, "Expired process must release its capacity");

        id = Insert(harq);
        for (uint32_t tick = 0; tick < offsetSlots + 4; ++tick)
        {
            scheduler->ResetExpiredHARQ(1, &harq);
        }
        NS_TEST_ASSERT_MSG_EQ(harq.Get(id).m_active, true, "Packet expired before delayed NACK");
        Feedback(scheduler, id, false, 1);
        NS_TEST_ASSERT_MSG_EQ(sap.m_retransmissions,
                              1,
                              "Delayed NACK must schedule a retransmission");
        NS_TEST_ASSERT_MSG_EQ(harq.Get(id).m_timer,
                              0,
                              "Retransmission must restart the waiting window");
        for (uint32_t tick = 0; tick < offsetSlots + 4; ++tick)
        {
            scheduler->ResetExpiredHARQ(1, &harq);
        }
        NS_TEST_ASSERT_MSG_EQ(harq.Get(id).m_active,
                              true,
                              "Retransmission expired before delayed ACK");
        Feedback(scheduler, id, true, 2);
        NS_TEST_ASSERT_MSG_EQ(harq.Get(id).m_active, false, "Delayed ACK must release the process");
        NS_TEST_ASSERT_MSG_EQ(harq.Size(), 0, "ACK must restore available capacity");
        scheduler->Dispose();
        Simulator::Destroy();
    }

    /** @param harq Process vector. @return Inserted process ID. */
    uint8_t Insert(NrMacHarqVector& harq) const
    {
        auto dci = std::make_shared<DciInfoElementTdma>(1,
                                                        m_uplink ? DciInfoElementTdma::UL
                                                                 : DciInfoElementTdma::DL,
                                                        1,
                                                        2,
                                                        10,
                                                        1,
                                                        nullptr,
                                                        100,
                                                        1,
                                                        0,
                                                        DciInfoElementTdma::DATA,
                                                        0,
                                                        0);
        dci->m_rbgBitmask = std::vector<bool>(10, true);
        uint8_t id = 0;
        const bool inserted =
            harq.Insert(&id, HarqProcess(true, HarqProcess::WAITING_FEEDBACK, 0, dci));
        NS_ABORT_MSG_UNLESS(inserted, "Test setup could not allocate a HARQ process");
        dci->m_harqProcess = id;
        return id;
    }

    /**
     * @param scheduler Scheduler under test.
     * @param id HARQ process ID.
     * @param ack Whether to acknowledge successfully.
     * @param frame Distinct frame for this scheduling decision.
     */
    void Feedback(Ptr<NrMacSchedulerNs3> scheduler, uint8_t id, bool ack, uint16_t frame) const
    {
        if (m_uplink)
        {
            UlHarqInfo feedback{};
            feedback.m_rnti = 1;
            feedback.m_harqProcessId = id;
            feedback.m_receptionStatus = ack ? UlHarqInfo::Ok : UlHarqInfo::NotOk;
            NrMacSchedSapProvider::SchedUlTriggerReqParameters params{};
            params.m_snfSf = SfnSf(frame, 0, 0, m_numerology);
            params.m_slotType = F;
            params.m_ulHarqInfoList.push_back(feedback);
            scheduler->DoSchedUlTriggerReq(params);
        }
        else
        {
            DlHarqInfo feedback{};
            feedback.m_rnti = 1;
            feedback.m_harqProcessId = id;
            feedback.m_harqStatus = ack ? DlHarqInfo::ACK : DlHarqInfo::NACK;
            NrMacSchedSapProvider::SchedDlTriggerReqParameters params{};
            params.m_snfSf = SfnSf(frame, 0, 0, m_numerology);
            params.m_slotType = F;
            params.m_dlHarqInfoList.push_back(feedback);
            scheduler->DoSchedDlTriggerReq(params);
        }
    }

    bool m_uplink;         ///< Direction under test.
    uint32_t m_offset;     ///< Offset in reference slots.
    uint16_t m_numerology; ///< Active numerology.
};

/** @ingroup test
 * @brief Cell-specific NTN scheduling offset regression suite.
 */
class NrKoffsetTestSuite : public TestSuite
{
  public:
    NrKoffsetTestSuite()
        : TestSuite("nr-test-koffset", Type::UNIT)
    {
        AddTestCase(new NrKoffsetTestCase, Duration::QUICK);
        for (bool uplink : {false, true})
        {
            for (uint32_t offset : {0u, 40u, 1023u})
            {
                for (uint16_t mu : {0, 1, 5})
                {
                    AddTestCase(new NrKoffsetHarqTestCase(uplink, offset, mu), Duration::QUICK);
                }
            }
        }
    }
};

static NrKoffsetTestSuite g_nrKoffsetTestSuite;

} // namespace ns3
