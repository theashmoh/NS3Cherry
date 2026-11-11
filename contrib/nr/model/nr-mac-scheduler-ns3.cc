// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#define NS_LOG_APPEND_CONTEXT                                                                      \
    do                                                                                             \
    {                                                                                              \
        std::clog << " [ CellId " << GetCellId() << ", bwpId " << GetBwpId() << "] ";              \
    } while (false);

#include "nr-mac-scheduler-ns3.h"

#include "nr-gnb-phy.h"

#include "nr-fh-control.h"
#include "nr-mac-scheduler-harq-rr.h"
#include "nr-mac-scheduler-lc-rr.h"
#include "nr-mac-scheduler-srs-default.h"
#include "nr-mac-short-bsr-ce.h"
#include "nr-pdcp.h"
#include "resource-assignment-matrix.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/integer.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <sstream>
#include <cmath>
#include <memory>
#include <numeric>
#include <ranges>
#include <set>
#include <unordered_set>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerNs3");

// NTN_SPSTRACE=1 in the environment: the logic lines of the SPS/CG scheduling on stderr, also in
// an optimised build (where NS_LOG is compiled out); nothing else changes
static const bool g_ntnSpsTrace = std::getenv("NTN_SPSTRACE") != nullptr;
#undef NS_LOG_LOGIC
#define NS_LOG_LOGIC(msg)                                                                          \
    do                                                                                             \
    {                                                                                              \
        if (g_ntnSpsTrace)                                                                         \
        {                                                                                          \
            std::cerr << "SPST " << Simulator::Now().GetMicroSeconds() << " " << msg << "\n";     \
        }                                                                                          \
    } while (false)
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerNs3);

NrMacSchedulerNs3::NrMacSchedulerNs3()
    : NrMacScheduler()
{
    NS_LOG_FUNCTION_NOARGS();

    // Hardcoded, but the type can be a parameter if needed
    m_schedHarq = CreateObject<NrMacSchedulerHarqRr>();
    m_schedHarq->InstallGetBwInRBG(std::bind(&NrMacSchedulerNs3::GetBandwidthInRbg, this));
    m_schedHarq->InstallGetBwpIdFn(std::bind(&NrMacSchedulerNs3::GetBwpId, this));
    m_schedHarq->InstallGetCellIdFn(std::bind(&NrMacSchedulerNs3::GetCellId, this));
    m_schedHarq->InstallGetFhControlMethodFn(
        std::bind(&NrMacSchedulerNs3::GetFhControlMethod, this));
    m_schedHarq->InstallDoesFhAllocationFitFn(
        std::bind_front(&NrMacSchedulerNs3::DoesFhAllocationFit, this));
    m_schedHarq->InstallReshapeAllocation(
        std::bind_front(&NrMacSchedulerNs3::ReshapeAllocation, this));
    m_schedHarq->InstallGetDlBitmask(std::bind_front(&NrMacSchedulerNs3::GetDlBitmask, this));
    m_schedHarq->InstallGetUlBitmask(std::bind_front(&NrMacSchedulerNs3::GetUlBitmask, this));
    m_schedHarq->InstallTakePdcchFn(std::bind_front(&NrMacSchedulerNs3::TakePdcch, this));
    m_cqiManagement.InstallGetBwpIdFn(std::bind(&NrMacSchedulerNs3::GetBwpId, this));
    m_cqiManagement.InstallGetCellIdFn(std::bind(&NrMacSchedulerNs3::GetCellId, this));
    m_cqiManagement.InstallGetNrAmcDlFn([this]() { return m_dlAmc; });
    m_cqiManagement.InstallGetNrAmcUlFn([this]() { return m_ulAmc; });
    m_cqiManagement.InstallGetStartMcsDlFn([this]() { return m_startMcsDl; });
    m_cqiManagement.InstallGetStartMcsUlFn([this]() { return m_startMcsUl; });

    // If more Srs allocators will be created, then we will add an attribute
    m_schedulerSrs = CreateObject<NrMacSchedulerSrsDefault>();
    m_nrFhSchedSapUser = new MemberNrFhSchedSapUser<NrMacSchedulerNs3>(this);
}

NrMacSchedulerNs3::~NrMacSchedulerNs3()
{
    m_ueMap.clear();
    delete m_nrFhSchedSapUser;
    m_nrFhSchedSapUser = nullptr;
    m_nrFhSchedSapProvider = nullptr;
}

void
NrMacSchedulerNs3::InstallDlAmc(const Ptr<NrAmc>& dlAmc)
{
    m_dlAmc = dlAmc;
    m_dlAmc->SetDlMode();
}

void
NrMacSchedulerNs3::InstallUlAmc(const Ptr<NrAmc>& ulAmc)
{
    m_ulAmc = ulAmc;
    m_ulAmc->SetUlMode();
}

Ptr<const NrAmc>
NrMacSchedulerNs3::GetUlAmc() const
{
    NS_LOG_FUNCTION(this);
    return m_ulAmc;
}

Ptr<const NrAmc>
NrMacSchedulerNs3::GetDlAmc() const
{
    NS_LOG_FUNCTION(this);
    return m_dlAmc;
}

int64_t
NrMacSchedulerNs3::AssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    return m_schedulerSrs->AssignStreams(stream);
}

TypeId
NrMacSchedulerNs3::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrMacSchedulerNs3")
            .SetParent<NrMacScheduler>()
            .AddAttribute("CqiTimerThreshold",
                          "The time while a CQI is valid",
                          TimeValue(Seconds(1)),
                          MakeTimeAccessor(&NrMacSchedulerNs3::SetCqiTimerThreshold,
                                           &NrMacSchedulerNs3::GetCqiTimerThreshold),
                          MakeTimeChecker())
            // Slots a HARQ process is held, after its last repetition and the K_offset round trip,
            // before it is reclaimed without feedback (ResetExpiredHARQ(), counted from
            // HarqProcess::m_txSlot since 30 Oct 2026). 0: NumHarqProcess slots, the module's
            // behaviour, which has no 3GPP equivalent: NR holds the soft buffer until feedback
            // arrives or the process is reused by a grant with a toggled NDI. Set it beyond the
            // worst-case loop delay so that the process count only limits the outstanding blocks.
            .AddAttribute("HarqProcessTimeout",
                          "Slots a HARQ process is held after its last repetition and the round trip before it is reclaimed without feedback; 0 = NumHarqProcess.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_harqProcessTimeout),
                          MakeUintegerChecker<uint32_t>(0, 1023))
            .AddAttribute("FixedMcsDl",
                          "Fix MCS to value set in StartingMcsDl",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::SetFixedDlMcs,
                                              &NrMacSchedulerNs3::IsDlMcsFixed),
                          MakeBooleanChecker())
            .AddAttribute("FixedMcsUl",
                          "Fix MCS to value set in StartingMcsUl",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::SetFixedUlMcs,
                                              &NrMacSchedulerNs3::IsUlMcsFixed),
                          MakeBooleanChecker())
            .AddAttribute("StartingMcsDl",
                          "Starting MCS for DL",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::SetStartMcsDl,
                                               &NrMacSchedulerNs3::GetStartMcsDl),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("StartingMcsUl",
                          "Starting MCS for UL",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::SetStartMcsUl,
                                               &NrMacSchedulerNs3::GetStartMcsUl),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("DlCtrlSymbols",
                          "Number of symbols allocated for DL CTRL",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::SetDlCtrlSyms,
                                               &NrMacSchedulerNs3::GetDlCtrlSyms),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("UlCtrlSymbols",
                          "Number of symbols allocated for UL CTRL",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::SetUlCtrlSyms,
                                               &NrMacSchedulerNs3::GetUlCtrlSyms),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("SrsSymbols",
                          "Number of symbols allocated for UL SRS",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::SetSrsCtrlSyms,
                                               &NrMacSchedulerNs3::GetSrsCtrlSyms),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("EnableSrsInUlSlots",
                          "Denotes whether the SRSs will be transmitted only in F slots"
                          "or both in F and UL slots. If False, SRS is transmitted only"
                          "in F slots, if True in both (F/UL)",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::SetSrsInUlSlots,
                                              &NrMacSchedulerNs3::IsSrsInUlSlots),
                          MakeBooleanChecker())
            .AddAttribute("EnableSrsInFSlots",
                          "Denotes whether the SRSs will be transmitted in F slots"
                          "If true, it can be transmitted in F slots, otherwise "
                          "it cannot.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::SetSrsInFSlots,
                                              &NrMacSchedulerNs3::IsSrsInFSlots),
                          MakeBooleanChecker())
            .AddAttribute("DlAmc",
                          "The DL AMC of this scheduler",
                          PointerValue(),
                          MakePointerAccessor(&NrMacSchedulerNs3::m_dlAmc),
                          MakePointerChecker<NrAmc>())
            .AddAttribute("UlAmc",
                          "The UL AMC of this scheduler",
                          PointerValue(),
                          MakePointerAccessor(&NrMacSchedulerNs3::m_ulAmc),
                          MakePointerChecker<NrAmc>())
            .AddAttribute("MaxDlMcs",
                          "Maximum MCS index for DL",
                          IntegerValue(-1),
                          MakeIntegerAccessor(&NrMacSchedulerNs3::SetMaxDlMcs,
                                              &NrMacSchedulerNs3::GetMaxDlMcs),
                          MakeIntegerChecker<int8_t>(-1, 30))
            .AddAttribute("EnableHarqReTx",
                          "If true, it would set the max HARQ ReTx to 3; otherwise it set it to 0",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::EnableHarqReTx,
                                              &NrMacSchedulerNs3::IsHarqReTxEnable),
                          MakeBooleanChecker())
            .AddAttribute(
                "SpsScenario",
                "Semi-persistent scheduling of SpsFiveQi bearers: 0 off, 1 kept, 2 adapted.",
                UintegerValue(0),
                MakeUintegerAccessor(&NrMacSchedulerNs3::m_spsScenario),
                MakeUintegerChecker<uint8_t>(0, 2))
            .AddAttribute("SpsDl",
                          "SpsScenario on the downlink (SPS); false: the downlink is dynamic.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_spsDlOn),
                          MakeBooleanChecker())
            .AddAttribute("SpsUl",
                          "SpsScenario on the uplink (configured grant); false: the uplink is dynamic.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_spsUlOn),
                          MakeBooleanChecker())
            .AddAttribute("SpsFiveQi",
                          "5QI of the bearers served by semi-persistent scheduling.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_spsFiveQi),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("SpsPeriodMs",
                          "Period of the semi-persistent blocks while the speaker talks (ms).",
                          UintegerValue(20),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_spsPeriodMs),
                          MakeUintegerChecker<uint32_t>(1, 640))
            .AddAttribute("SpsSilencePeriodMs",
                          "SpsScenario 2: period of the blocks while the speaker is silent (ms).",
                          UintegerValue(160),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_spsSilencePeriodMs),
                          MakeUintegerChecker<uint32_t>(1, 640))
            .AddAttribute("SpsTbBytes",
                          "MAC PDU of one voice packet in the steady state of ROHC (bytes).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_spsTbBytes),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("SpsSilenceTbBytes",
                          "SpsScenario 2: MAC PDU of one silence descriptor packet (bytes).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_spsSilenceTbBytes),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute(
                "SpsFitExtraBytes",
                "SpsScenario 2: a new configuration may be this much larger for its packet.",
                UintegerValue(0),
                MakeUintegerAccessor(&NrMacSchedulerNs3::m_spsFitExtraBytes),
                MakeUintegerChecker<uint32_t>())
            .AddAttribute("CgBlockPacketOnly",
                          "Uplink: a configured block holds the packet only (BSR when there is room, confirmation ahead); false: packet + BSR + CE in every block (v9)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_cgBlockPacketOnly),
                          MakeBooleanChecker())
            .AddAttribute("CgStartAfterIr",
                          "Uplink: no configured grant starts while the last packet received is a ROHC IR packet (the downlink waits for a packet that fits)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_cgStartAfterIr),
                          MakeBooleanChecker())
            .AddAttribute("CgStartRetry",
                          "Uplink: an activation whose block is not placed two slots in a row waits a talk period, the UE answered with DCIs meanwhile (false: v9/v10, tried every slot)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_cgStartRetry),
                          MakeBooleanChecker())
            .AddAttribute("SrGrantLearned",
                          "Grant for a scheduling request: the last packet received + BSR (SrGrantBytes the first time); false: SrGrantBytes",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_srGrantLearned),
                          MakeBooleanChecker())
            .AddAttribute("SrGrantL2Bytes",
                          "SrGrantLearned: bytes below PDCP added to the last PDCP PDU (RLC + MAC subheader)",
                          UintegerValue(3),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_srGrantL2Bytes),
                          MakeUintegerChecker<uint32_t>(0, 16))
            .AddAttribute("SpsSidGapMs",
                          "Time from the first silence descriptor to the next (TS 26.093: 60 ms); 0 = SpsSilencePeriodMs (v9)",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_spsSidGapMs),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("SpsDlSilenceAtSecondSid",
                          "SpsScenario 2, downlink: the silence configuration starts with the second SID (first SID_UPDATE); false: the first (v9)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_spsDlSilenceAtSecondSid),
                          MakeBooleanChecker())
            .AddAttribute("DlSpsHysteresis",
                          "Downlink SPS reactivation with the time filters below; false: at once when the MCS is SpsMcsTolerance off (v9)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_dlSpsHysteresis),
                          MakeBooleanChecker())
            .AddAttribute("DlSpsRaiseTime",
                          "DlSpsHysteresis: the MCS must be SpsMcsTolerance worse this long before a reactivation",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&NrMacSchedulerNs3::m_dlSpsRaiseTime),
                          MakeTimeChecker())
            .AddAttribute("DlSpsLowerTime",
                          "DlSpsHysteresis: the MCS must be SpsMcsTolerance better this long before a reactivation",
                          TimeValue(MilliSeconds(400)),
                          MakeTimeAccessor(&NrMacSchedulerNs3::m_dlSpsLowerTime),
                          MakeTimeChecker())
            .AddAttribute("DlSpsMinInterval",
                          "DlSpsHysteresis: least time between two activations of a UE's SPS, unless urgent",
                          TimeValue(MilliSeconds(200)),
                          MakeTimeAccessor(&NrMacSchedulerNs3::m_dlSpsMinInterval),
                          MakeTimeChecker())
            .AddAttribute("DlSpsUrgentSteps",
                          "DlSpsHysteresis: MCS steps worse than the configuration's that reactivate at once (0: never)",
                          UintegerValue(6),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_dlSpsUrgentSteps),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("DlSpsFailures",
                          "DlSpsHysteresis: failed first transmissions of SPS blocks in a row that reactivate at once (0: never)",
                          UintegerValue(2),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_dlSpsFailures),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute(
                "SpsMcsTolerance",
                "A configuration is activated anew when the MCS of the UE is this far off.",
                UintegerValue(3),
                MakeUintegerAccessor(&NrMacSchedulerNs3::m_spsMcsTolerance),
                MakeUintegerChecker<uint8_t>())
            // Keep a log of the uplink HARQ bookkeeping (feedback in, decisions, blocks, retransmissions,
            // erasures) and print the UE's history if an inconsistency is found. Costs time: for debugging
            // only.
            .AddAttribute("DiagUlHarq",
                          "Log the uplink HARQ bookkeeping and print a UE's history on an inconsistency (debugging only, costs time).",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_diagUlHarq),
                          MakeBooleanChecker())
            // SpsScenario 2, uplink: bytes of the Multiple Entry Configured Grant Confirmation MAC CE (TS
            // 38.321 6.1.3.31: 4 octets and a 1-octet subheader) that confirms an activation or release
            // when the UE has several configurations. With one configuration the CE is its subheader
            // alone, 1 byte (6.1.3.7).
            .AddAttribute("CgMultiEntryCeBytes",
                          "SpsScenario 2, uplink: bytes of the Multiple Entry CG Confirmation MAC CE (TS 38.321 6.1.3.31: 4 + 2 eLCID subheader).",
                          UintegerValue(6),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_cgMultiEntryCeBytes),
                          MakeUintegerChecker<uint32_t>(1, 16))
            // SpsScenario 2, uplink, the speaker falls silent: false = the talk configuration stays, its
            // occasions unused, until the next silence descriptor is due, one period after the first; then
            // one DCI releases it and one activates the silence configuration, whose occasions fall where
            // the descriptors are. true = the talk configuration is released as soon as the gNB has the
            // first descriptor (no occasion kept), and the silence configuration is activated when the
            // next descriptor is due, as before (an activation starts its occasions K2 + K_offset after
            // the DCI: one sent at once would put them a period out of step, and the descriptors would
            // come with scheduling requests).
            .AddAttribute("CgSilenceSwitch",
                          "SpsScenario 2, uplink: true = release the talk configuration at the first silence descriptor, false = at the next one.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_cgSilenceSwitchNow),
                          MakeBooleanChecker())
            .AddAttribute("DlHarqFeedbackDisabled",
                          "Rel-17 NTN downlinkHARQ-FeedbackDisabled. Releases the DL HARQ process "
                          "at transmission rather than on feedback, removing the throttle of one "
                          "block per process per loop round trip. No retransmissions occur; "
                          "reliability has to come from RLC AM above.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_dlHarqFeedbackDisabled),
                          MakeBooleanChecker())
            // DL outer-loop link adaptation (OLLA) step applied to the MCS offset on each ACKed first
            // transmission, in MCS units. The step applied on NACK is scaled by
            // (1-OllaTargetBler)/OllaTargetBler, so the offset settles where the measured initial BLER
            // equals the target. 0 disables OLLA, which reproduces the open-loop behaviour.
            .AddAttribute("OllaStepUp",
                          "DL outer-loop link adaptation: MCS offset step per ACKed first transmission (NACK step scaled by (1-target)/target); 0 = off.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrMacSchedulerNs3::m_ollaStepUp),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("OllaTargetBler",
                          "Target initial (first-transmission) BLER for DL OLLA.",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&NrMacSchedulerNs3::m_ollaTargetBler),
                          MakeDoubleChecker<double>(0.001, 0.5))
            .AddAttribute("OllaMaxOffset",
                          "Absolute clamp on the DL OLLA MCS offset, in MCS units.",
                          DoubleValue(4.0),
                          MakeDoubleAccessor(&NrMacSchedulerNs3::m_ollaMaxOffset),
                          MakeDoubleChecker<double>(0.0, 30.0))
            // A dynamic downlink block gets whole resource blocks at the MCS of link adaptation and
            // often holds much less than they carry (a voice frame on 2 RBs at MCS 11: 22 of 49 B).
            // With DlMcsFill the scheduler then lowers the MCS to the lowest whose block on the
            // same resource blocks still holds what the UE's queues need: the same resources, a
            // lower code rate. Such a block is left out of the outer loop (its outcome says nothing
            // of the MCS the loop steers). OFDMA scheduler; HARQ retransmissions keep their MCS.
            .AddAttribute("DlMcsFill",
                          "OFDMA: lower a new downlink block's MCS to the lowest that still holds the UE's queued bytes on its RBs.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_dlMcsFill),
                          MakeBooleanChecker())
            // Uplink configured grant: when it is activated anew for a link that changed. Its MCS
            // and repetitions come with the activation DCI, so every change costs a DCI (and the
            // UE's confirmation MAC CE). With CgHysteresis the gNB raises K when the SINR has
            // been CgRaiseDb short of the target for CgRaiseTime (at once if CgUrgentDb short or
            // after CgFailures failed first transmissions in a row), lowers it when CgLowerDb
            // were to spare for CgLowerTime, chooses K and MCS for the CgSinrPercentile of the
            // SINR over CgSinrWindow, and activates no sooner than CgMinInterval after the last
            // activation unless urgent. A gNB policy: 3GPP leaves when to reactivate open.
            .AddAttribute("CgHysteresis",
                          "UL configured grant: activate anew per the Cg* attributes (false: at every change of K or MCS, as before).",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_cgHysteresis),
                          MakeBooleanChecker())
            .AddAttribute("CgRaiseDb",
                          "CgHysteresis: more repetitions when the SINR is this far (dB) under the target for CgRaiseTime.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NrMacSchedulerNs3::m_cgRaiseDb),
                          MakeDoubleChecker<double>(0.0, 30.0))
            .AddAttribute("CgRaiseTime",
                          "CgHysteresis: how long the SINR must be CgRaiseDb short before K goes up.",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&NrMacSchedulerNs3::m_cgRaiseTime),
                          MakeTimeChecker())
            .AddAttribute("CgUrgentDb",
                          "CgHysteresis: this far (dB) under the target, K goes up at once, whatever CgMinInterval.",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&NrMacSchedulerNs3::m_cgUrgentDb),
                          MakeDoubleChecker<double>(0.0, 30.0))
            .AddAttribute("CgLowerDb",
                          "CgHysteresis: fewer repetitions when they would leave this much (dB) to spare for CgLowerTime.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&NrMacSchedulerNs3::m_cgLowerDb),
                          MakeDoubleChecker<double>(0.0, 30.0))
            .AddAttribute("CgLowerTime",
                          "CgHysteresis: how long the spare margin must last before K goes down.",
                          TimeValue(MilliSeconds(400)),
                          MakeTimeAccessor(&NrMacSchedulerNs3::m_cgLowerTime),
                          MakeTimeChecker())
            .AddAttribute("CgSinrPercentile",
                          "CgHysteresis: K and MCS of an activation for this percentile of the SINR over CgSinrWindow (0: the last measurement).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrMacSchedulerNs3::m_cgSinrPercentile),
                          MakeDoubleChecker<double>(0.0, 50.0))
            .AddAttribute("CgSinrWindow",
                          "CgHysteresis: the window of SINR samples (one per talk period) for CgSinrPercentile.",
                          TimeValue(MilliSeconds(500)),
                          MakeTimeAccessor(&NrMacSchedulerNs3::m_cgSinrWindow),
                          MakeTimeChecker())
            .AddAttribute("CgMinInterval",
                          "CgHysteresis: least time between two activations of a UE's configured grant, unless urgent.",
                          TimeValue(MilliSeconds(200)),
                          MakeTimeAccessor(&NrMacSchedulerNs3::m_cgMinInterval),
                          MakeTimeChecker())
            .AddAttribute("CgFailures",
                          "CgHysteresis: this many failed first transmissions of its blocks in a row make it urgent (0: never).",
                          UintegerValue(2),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_cgFailures),
                          MakeUintegerChecker<uint32_t>(0, 100))
            // Uplink outer-loop link adaptation: an MCS offset per UE moved on the outcome of each first
            // transmission the UE sent (not on grants it missed), with OllaStepUp, OllaTargetBler and
            // OllaMaxOffset.
            .AddAttribute("UlOlla",
                          "Uplink outer-loop link adaptation on the outcome of each first transmission the UE sent (OllaStepUp, OllaTargetBler, OllaMaxOffset).",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_ulOlla),
                          MakeBooleanChecker())
            // Uplink link adaptation: time constant of the average of the SINR reports of a UE. 0 (the
            // module's behaviour): the MCS and the repetitions follow the SINR of the last reception
            // alone, fading included. Above 0: each report enters an exponentially weighted mean (weight 1
            // - exp(-dt / UlSinrAvgTime), in linear units, as the SINR of one resource block), as a CQI
            // filter would; a few times the coherence time of the channel (70 ms for a pedestrian)
            // averages the fading out and leaves the margin to the outer loop.
            .AddAttribute("UlSinrAvgTime",
                          "Uplink link adaptation: time constant of the exponential average of a UE's SINR reports; 0 = the last report alone.",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&NrMacSchedulerNs3::m_ulSinrAvgTime),
                          MakeTimeChecker())
            // true: the uplink MCS of a grant is for its resource blocks. The UE spreads its power over
            // the resource blocks it gets, so the SINR measured on its last transmission is scaled by the
            // ratio of the resource blocks then and now. false: the MCS of that SINR as it was.
            .AddAttribute("UlMcsPerRbs",
                          "true: the uplink MCS of a grant is for its resource blocks (the measured SINR scaled by the RB ratio); false: as measured.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerNs3::m_ulMcsPerRbs),
                          MakeBooleanChecker())
            // Transport block size the gNB aims at with the grant it sends for a scheduling request. The
            // default, 12 bytes and an RLC and a MAC header, holds a buffer status report and little else.
            .AddAttribute("SrGrantBytes",
                          "Transport block size (bytes) aimed at by the grant that answers a scheduling request.",
                          UintegerValue(12 + 2 + MAC_SUBHEADER_SIZE),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_srGrantBytes),
                          MakeUintegerChecker<uint32_t>(12, 10000))
            // OFDMA: the most resource blocks of an uplink grant (whole RBGs: rounded up to a multiple of
            // the RBs of an RBG), so that a UE puts its power into that much bandwidth, and the uplink
            // blocks of a configured grant are that wide. A UE whose uplink has not been measured (no
            // PUSCH within CqiTimerThreshold) gets one grant without the limit, from which the gNB learns
            // its SINR (a real gNB has its Msg3 or SRS for that). 0 = no limit. A TDMA scheduler ignores
            // it.
            .AddAttribute("UlGrantRbs",
                          "OFDMA: the most resource blocks of an uplink grant (whole RBGs), also the width of configured-grant blocks; 0 = no limit.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_ulGrantRbs),
                          MakeUintegerChecker<uint32_t>())
            // OFDMA, with UlGrantRbs and UlMcsPerRbs: the resource blocks of a wide uplink grant, which a
            // UE gets instead of UlGrantRbs when UlWideSinrDb and UlWideLoadPct allow it: a block twice as
            // wide with half the repetitions (a fixed PuschRepetitions is divided by the widening; with 0
            // the repetitions follow from the SINR of the wide grant), which sends the bytes in fewer
            // slots. 0 = off.
            .AddAttribute("UlWideGrantRbs",
                          "OFDMA: resource blocks of a wide uplink grant given instead of UlGrantRbs when UlWideSinrDb and UlWideLoadPct allow; 0 = off.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_ulWideRbs),
                          MakeUintegerChecker<uint32_t>())
            // UlWideGrantRbs: the SINR (dB) the UE must have on ONE resource block, one transmission,
            // before repetitions, from its last measurement. A wide grant has 10 log10(UlWideGrantRbs /
            // UlGrantRbs) dB less per RB. Lower: the UE needs repetitions in the wide grant too, and wide
            // grants cost more resource block slots than they save.
            .AddAttribute("UlWideSinrDb",
                          "UlWideGrantRbs: the SINR (dB) a UE needs on one resource block, one transmission, from its last measurement, for a wide grant.",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&NrMacSchedulerNs3::m_ulWideSinrDb),
                          MakeDoubleChecker<double>(-30.0, 40.0))
            // UlWideGrantRbs: a UE gets a wide grant only if all the UEs waiting in the slot have a first
            // resource block and the uplink resource blocks in use in the slot (repetitions and
            // retransmissions of earlier blocks, grants so far, and this one) are at most this % of the
            // band.
            .AddAttribute("UlWideLoadPct",
                          "UlWideGrantRbs: a wide grant only if every waiting UE has a first RB and the uplink RBs in use in the slot are at most this %.",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&NrMacSchedulerNs3::m_ulWideLoadPct),
                          MakeDoubleChecker<double>(0.0, 100.0))
            // PUSCH repetition type A (TS 38.214 6.1.2.1, Rel-17: up to 32): the uplink slots in a row
            // that carry each uplink block, which the gNB combines before it decodes; a retransmission is
            // repeated as often. 1 = none. 0 = per UE and grant, from its last measurement: the fewest of
            // 1, 2, 3, 4, 7, 8, 12, 16, 20, 24, 28, 32 that bring the SINR of the grant's resource blocks,
            // times them, to PuschRepetitionSinrDb, or more if the block of a slot would be under the
            // least block the UE sends data in: 15 B, 12 B with NrUeMac::CompactBsr (1 without a
            // measurement). The MCS is the one the error model decodes after the repetitions. With
            // UlGrantRbs, the blocks of a configured grant repeat too, as often as TS 38.214 6.1.2.3 has
            // (the numbers above, not more than the slots of the period; SpsUlLink()). Only these values
            // are accepted: 0, 1, 2, 3, 4, 7, 8, 12, 16, 20, 24, 28, 32 (TS 38.331
            // numberOfRepetitionsExt-r17). OFDMA only, with all UEs in one beam.
            .AddAttribute("PuschRepetitions",
                          "PUSCH repetition type A (TS 38.214 6.1.2.1): slots in a row per uplink block, 1 = none, 0 = per grant from the SINR (1..32, TS 38.331 numberOfRepetitionsExt-r17).",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::SetPuschRepetitions,
                                               &NrMacSchedulerNs3::GetPuschRepetitions),
                          MakeUintegerChecker<uint32_t>(0, 32))
            .AddAttribute("PuschRepetitionSinrDb",
                          "PuschRepetitions = 0: the SINR (dB) that the repetitions of a block "
                          "bring the SINR of its resource blocks to",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&NrMacSchedulerNs3::m_puschRepSinrDb),
                          MakeDoubleChecker<double>(-30.0, 40.0))
            // PuschRepetitions = 0: the least transport block of one slot (bytes) that the repetitions
            // have to give the MCS the error model decodes: if the block is smaller, the repetitions go up
            // (as far as 32) until it is not. 0 = the least block the UE sends data in (15 bytes, 12 with
            // NrUeMac::CompactBsr). A voice frame needs about 26 (23 with the compact BSR): the PDCP,
            // ROHC, RTP payload, RLC and MAC headers and the BSR. With smaller blocks a frame goes in two
            // or three and takes a round of repetitions each.
            .AddAttribute("PuschMinBlockBytes",
                          "PuschRepetitions = 0: the least one-slot transport block (bytes) the repetitions must give the MCS; 0 = the least block carrying data.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_puschMinBlockBytes),
                          MakeUintegerChecker<uint32_t>(0, 10000))
            // PDSCH aggregation (TS 38.214 5.1.2.1): a downlink block goes in K downlink slots in a
            // row, on the same symbols, with one DCI; the UE combines them and sends its HARQ-ACK
            // after the last (K1 counts from the last). A fixed K is pdsch-AggregationFactor of
            // PDSCH-Config (n2, n4, n8, TS 38.331; unchanged up to Rel-19 for unicast); for SPS the
            // same K is pdsch-AggregationFactor-r16 of SPS-Config. 0 = per grant: K is the
            // repetitionNumber-r16 of the TDRA row the DCI points at (2..8, 16; TS 38.214 5.1.2.1
            // with one TCI state), the fewest that bring K times the UE's SINR (the one its
            // PDCCH is sized with) to PdschAggregationSinrDb; an SPS configuration takes it from
            // n1..n8 at its activation and keeps it. The MCS stays the one of the CQI: the
            // repetitions buy reliability, not rate. A retransmission is repeated as often.
            // Every transmission keeps the redundancy version of the first (TS 38.214 Table
            // 5.1.2.1-2 cycles 0, 2, 3, 1: not modelled; the error model combines as chase).
            // OFDMA, all UEs in one beam, HarqRetxInFrequency.
            .AddAttribute("PdschAggregation",
                          "PDSCH aggregation (TS 38.214 5.1.2.1): slots per downlink block, 1 = none, 2/4/8 fixed, 0 = per grant from the SINR (Rel-16 repetitionNumber).",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_pdschAggregation),
                          MakeUintegerChecker<uint32_t>(0, 8))
            .AddAttribute("PdschAggregationSinrDb",
                          "PdschAggregation = 0: K brings K x the UE's downlink SINR to this (dB).",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&NrMacSchedulerNs3::m_pdschAggSinrDb),
                          MakeDoubleChecker<double>(-30.0, 40.0))
            .AddAttribute(
                "SchedLcAlgorithmType",
                "Type of the scheduling algorithm that assigns bytes to the different LCs.",
                TypeIdValue(NrMacSchedulerLcRR::GetTypeId()),
                MakeTypeIdAccessor(&NrMacSchedulerNs3::SetLcSched),
                //&NrMacSchedulerNs3::GetLcSched),
                MakeTypeIdChecker())
            .AddAttribute("RachUlGrantMcs",
                          "The MCS of the RACH UL grant, must be [0..15] (default 0)",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::SetRachUlGrantMcs),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("Msg3MaxRetx",
                          "Maximum number of HARQ retransmissions for msg3 (UL RRC Connection "
                          "Request). 0 disables msg3 retransmission (single-shot, legacy "
                          "behaviour, default). NOTE: when enabled, msg3 HARQ combining can "
                          "deliver a duplicate Connection Request that is not yet handled by the "
                          "gNB RRC state machine; do not enable without addressing that.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrMacSchedulerNs3::m_msg3MaxRetx),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute(
                "McsCsiSource",
                "Choose which CSI information is used to estimate DL MCS(default AVG_MCS)",
                EnumValue(NrMacSchedulerUeInfo::McsCsiSource::WIDEBAND_MCS),
                MakeEnumAccessor<NrMacSchedulerUeInfo::McsCsiSource>(
                    &NrMacSchedulerNs3::m_mcsCsiSource),
                MakeEnumChecker<NrMacSchedulerUeInfo::McsCsiSource>(
                    NrMacSchedulerUeInfo::McsCsiSource::AVG_MCS,
                    "AVG_MCS",
                    NrMacSchedulerUeInfo::McsCsiSource::AVG_SPEC_EFF,
                    "AVG_SPEC_EFF",
                    NrMacSchedulerUeInfo::McsCsiSource::AVG_SINR,
                    "AVG_SINR",
                    NrMacSchedulerUeInfo::McsCsiSource::WIDEBAND_MCS,
                    "WIDEBAND_MCS"))
            .AddTraceSource("CsiFeedbackReceived",
                            "Received CSI feedback post-processed by the scheduler CQI management",
                            MakeTraceSourceAccessor(&NrMacSchedulerNs3::m_csiFeedbackReceived),
                            "ns3::NrMacSchedulerNs3::CsiFeedbackReceived::TracedCallback")
            .AddTraceSource("DlOllaUpdate",
                            "DL OLLA offset after each first-transmission outcome: rnti, offset "
                            "in MCS units, ack, clamped",
                            MakeTraceSourceAccessor(&NrMacSchedulerNs3::m_dlOllaUpdate),
                            "ns3::NrMacSchedulerNs3::DlOllaUpdate::TracedCallback");

    return tid;
}

/**
 * @brief Set a fixed MCS.
 * @param mcs The MCS.
 *
 * Set a fixed MCS for all UE that will be registered *AFTER* the call to this
 * function.
 */
void
NrMacSchedulerNs3::DoSchedSetMcs(uint32_t mcs)
{
    NS_LOG_FUNCTION(this);
    m_fixedMcsDl = true;
    m_fixedMcsUl = true;
    m_startMcsDl = static_cast<uint8_t>(mcs);
    m_startMcsUl = static_cast<uint8_t>(mcs);
}

void
NrMacSchedulerNs3::DoSchedDlRachInfoReq(
    const NrMacSchedSapProvider::SchedDlRachInfoReqParameters& params)
{
    NS_LOG_FUNCTION(this);

    // The preambles of this slot join those that still wait for their Msg3 grant. A slot has
    // room for a few grants only (three in a 5 MHz carrier); the release dropped the preambles
    // beyond these, and a UE with ideal random access, which sends its preamble once, then
    // never connected.
    m_rachList.insert(m_rachList.end(), params.m_rachList.begin(), params.m_rachList.end());
}

void
NrMacSchedulerNs3::SetCqiTimerThreshold(const Time& v)
{
    NS_LOG_FUNCTION(this);
    m_cqiTimersThreshold = v;
}

Time
NrMacSchedulerNs3::GetCqiTimerThreshold() const
{
    NS_LOG_FUNCTION(this);
    return m_cqiTimersThreshold;
}

void
NrMacSchedulerNs3::SetFixedDlMcs(bool v)
{
    NS_LOG_FUNCTION(this);
    m_fixedMcsDl = v;
}

bool
NrMacSchedulerNs3::IsDlMcsFixed() const
{
    NS_LOG_FUNCTION(this);
    return m_fixedMcsDl;
}

void
NrMacSchedulerNs3::SetFixedUlMcs(bool v)
{
    NS_LOG_FUNCTION(this);
    m_fixedMcsUl = v;
}

bool
NrMacSchedulerNs3::IsUlMcsFixed() const
{
    NS_LOG_FUNCTION(this);
    return m_fixedMcsUl;
}

void
NrMacSchedulerNs3::SetStartMcsDl(uint8_t v)
{
    NS_LOG_FUNCTION(this);
    m_startMcsDl = v;
}

uint8_t
NrMacSchedulerNs3::GetStartMcsDl() const
{
    NS_LOG_FUNCTION(this);
    return m_startMcsDl;
}

void
NrMacSchedulerNs3::SetMaxDlMcs(int8_t v)
{
    NS_LOG_FUNCTION(this);
    m_maxDlMcs = v;
}

int8_t
NrMacSchedulerNs3::GetMaxDlMcs() const
{
    NS_LOG_FUNCTION(this);
    return m_maxDlMcs;
}

void
NrMacSchedulerNs3::SetRachUlGrantMcs(uint8_t v)
{
    m_rachUlGrantMcs = v;
}

void
NrMacSchedulerNs3::SetPuschRepetitions(uint32_t k)
{
    NS_ABORT_MSG_UNLESS(k == 0 || NrMacSchedulerUeInfo::IsPuschRepetitionValue(k),
                        "PuschRepetitions " << k
                                            << " is not a number of repetitions of TS 38.331 "
                                               "(numberOfRepetitionsExt-r17): use 0 (as many as "
                                               "needed), 1, 2, 3, 4, 7, 8, 12, 16, 20, 24, 28 "
                                               "or 32");
    m_puschRepetitions = k;
}

uint32_t
NrMacSchedulerNs3::GetPuschRepetitions() const
{
    return m_puschRepetitions;
}

void
NrMacSchedulerNs3::SetLcSched(const TypeId& type)
{
    NS_LOG_FUNCTION(this);
    ObjectFactory factory;
    m_schedLcType = type;

    factory.SetTypeId(m_schedLcType);
    m_schedLc = DynamicCast<NrMacSchedulerLcAlgorithm>(factory.Create());
    NS_ASSERT(m_schedLc != nullptr);
}

void
NrMacSchedulerNs3::SetStartMcsUl(uint8_t v)
{
    NS_LOG_FUNCTION(this);
    m_startMcsUl = v;
}

uint8_t
NrMacSchedulerNs3::GetStartMcsUl() const
{
    NS_LOG_FUNCTION(this);
    return m_startMcsUl;
}

void
NrMacSchedulerNs3::SetDlCtrlSyms(uint8_t v)
{
    m_dlCtrlSymbols = v;
}

uint8_t
NrMacSchedulerNs3::GetDlCtrlSyms() const
{
    return m_dlCtrlSymbols;
}

void
NrMacSchedulerNs3::SetUlCtrlSyms(uint8_t v)
{
    m_ulCtrlSymbols = v;
}

void
NrMacSchedulerNs3::SetDlNotchedRbgMask(const std::vector<bool>& dlNotchedRbgsMask)
{
    NS_LOG_FUNCTION(this);
    m_dlNotchedRbgsMask = dlNotchedRbgsMask;
    std::stringstream ss;

    // print the DL mask set (prefix + is added just for printing purposes)
    for (const auto& x : m_dlNotchedRbgsMask)
    {
        ss << +x << " ";
    }
    NS_LOG_INFO("Set DL notched mask: " << ss.str());
}

std::vector<bool>
NrMacSchedulerNs3::GetDlNotchedRbgMask() const
{
    return m_dlNotchedRbgsMask;
}

void
NrMacSchedulerNs3::SetUlNotchedRbgMask(const std::vector<bool>& ulNotchedRbgsMask)
{
    NS_LOG_FUNCTION(this);
    m_ulNotchedRbgsMask = ulNotchedRbgsMask;
    std::stringstream ss;

    // print the UL mask set (prefix + is added just for printing purposes)
    for (const auto& x : m_ulNotchedRbgsMask)
    {
        ss << +x << " ";
    }
    NS_LOG_INFO("Set UL notched mask: " << ss.str());
}

std::vector<bool>
NrMacSchedulerNs3::GetUlNotchedRbgMask() const
{
    return m_ulNotchedRbgsMask;
}

void
NrMacSchedulerNs3::SetSrsCtrlSyms(uint8_t v)
{
    m_srsCtrlSymbols = v;
}

uint8_t
NrMacSchedulerNs3::GetSrsCtrlSyms() const
{
    return m_srsCtrlSymbols;
}

void
NrMacSchedulerNs3::SetSrsInUlSlots(bool v)
{
    m_enableSrsInUlSlots = v;
}

bool
NrMacSchedulerNs3::IsSrsInUlSlots() const
{
    return m_enableSrsInUlSlots;
}

void
NrMacSchedulerNs3::SetSrsInFSlots(bool v)
{
    m_enableSrsInFSlots = v;
}

bool
NrMacSchedulerNs3::IsSrsInFSlots() const
{
    return m_enableSrsInFSlots;
}

void
NrMacSchedulerNs3::EnableHarqReTx(bool enableFlag)
{
    m_enableHarqReTx = enableFlag;
}

bool
NrMacSchedulerNs3::IsHarqReTxEnable() const
{
    return m_enableHarqReTx;
}

void
NrMacSchedulerNs3::SetNrFhSchedSapProvider(NrFhSchedSapProvider* s)
{
    m_nrFhSchedSapProvider = s;
}

NrFhSchedSapUser*
NrMacSchedulerNs3::GetNrFhSchedSapUser()
{
    return m_nrFhSchedSapUser;
}

uint8_t
NrMacSchedulerNs3::ScheduleDlHarq(PointInFTPlane* startingPoint,
                                  uint8_t symAvail,
                                  const NrMacSchedulerNs3::ActiveHarqMap& activeDlHarq,
                                  const std::unordered_map<uint16_t, UePtr>& ueMap,
                                  std::vector<DlHarqInfo>* dlHarqToRetransmit,
                                  const std::vector<DlHarqInfo>& dlHarqFeedback,
                                  SlotAllocInfo* slotAlloc) const
{
    NS_LOG_FUNCTION(this);
    return m_schedHarq->ScheduleDlHarq(startingPoint,
                                       symAvail,
                                       activeDlHarq,
                                       ueMap,
                                       dlHarqToRetransmit,
                                       dlHarqFeedback,
                                       slotAlloc);
}

uint8_t
NrMacSchedulerNs3::ScheduleUlHarq(PointInFTPlane* startingPoint,
                                  uint8_t symAvail,
                                  const std::unordered_map<uint16_t, UePtr>& ueMap,
                                  std::vector<UlHarqInfo>* ulHarqToRetransmit,
                                  const std::vector<UlHarqInfo>& ulHarqFeedback,
                                  SlotAllocInfo* slotAlloc) const
{
    NS_LOG_FUNCTION(this);
    return m_schedHarq->ScheduleUlHarq(startingPoint,
                                       symAvail,
                                       ueMap,
                                       ulHarqToRetransmit,
                                       ulHarqFeedback,
                                       slotAlloc);
}

void
NrMacSchedulerNs3::SortDlHarq(NrMacSchedulerNs3::ActiveHarqMap* activeDlHarq) const
{
    NS_LOG_FUNCTION(this);
    m_schedHarq->SortDlHarq(activeDlHarq);
}

void
NrMacSchedulerNs3::SortUlHarq(NrMacSchedulerNs3::ActiveHarqMap* activeUlHarq) const
{
    NS_LOG_FUNCTION(this);
    m_schedHarq->SortDlHarq(activeUlHarq);
}

uint8_t
NrMacSchedulerNs3::GetUlCtrlSyms() const
{
    return m_ulCtrlSymbols;
}

/**
 * @brief Cell configuration
 * @param params unused.
 *
 * Ignored. Always Success.
 */
void
NrMacSchedulerNs3::DoCschedCellConfigReq(
    const NrMacCschedSapProvider::CschedCellConfigReqParameters& params)
{
    NS_LOG_FUNCTION(this);

    NS_ASSERT(params.m_ulBandwidth == params.m_dlBandwidth);
    m_bandwidth = params.m_dlBandwidth;

    NrMacCschedSapUser::CschedUeConfigCnfParameters cnf;
    cnf.m_result = NrMacCschedSapUser::Result_e::SUCCESS;
    m_macCschedSapUser->CschedUeConfigCnf(cnf);
}

/**
 * @brief Register an UE
 * @param params params of the UE
 *
 * If the UE is not registered, then create its representation with a call to
 * CreateUeRepresentation, and then save its pointer in the m_ueMap map.
 *
 * If the UE is registered, update its corresponding beam.
 */
void
NrMacSchedulerNs3::DoCschedUeConfigReq(
    const NrMacCschedSapProvider::CschedUeConfigReqParameters& params)
{
    NS_LOG_FUNCTION(this << " RNTI " << params.m_rnti << " txMode "
                         << static_cast<uint32_t>(params.m_transmissionMode));

    // NOTE: msg3 pending entries are NOT cleared here. CschedUeConfigReq for a
    // T-C-RNTI fires when the UE is registered for RA reception (BEFORE the msg3
    // PUSCH is decoded), so erasing here would drop the pending entry before any
    // msg3 HARQ feedback and defeat retransmission. The entry is instead managed
    // by the UL HARQ feedback in DoSchedUlTriggerReq: an Ok closes it (success),
    // a NotOk retransmits (or closes it once the budget is exhausted).

    auto itUe = m_ueMap.find(params.m_rnti);
    GetSecond UeInfoOf;
    if (itUe == m_ueMap.end())
    {
        itUe = m_ueMap.insert(std::make_pair(params.m_rnti, CreateUeRepresentation(params))).first;

        UeInfoOf(*itUe)->m_dlHarq.SetMaxSize(
            static_cast<uint8_t>(m_macSchedSapUser->GetNumHarqProcess()));
        UeInfoOf(*itUe)->m_ulHarq.SetMaxSize(
            static_cast<uint8_t>(m_macSchedSapUser->GetNumHarqProcess()));
        UeInfoOf(*itUe)->m_dlMcs = m_startMcsDl;
        UeInfoOf(*itUe)->m_startMcsDlUe = m_startMcsDl;
        UeInfoOf(*itUe)->m_ulMcs = m_startMcsUl;
        UeInfoOf(*itUe)->m_dlAmc = m_dlAmc;
        UeInfoOf(*itUe)->m_ulAmc = m_ulAmc;
        UeInfoOf(*itUe)->m_mcsCsiSource = m_mcsCsiSource;
        UeInfoOf(*itUe)->m_ulMcsPerRbs = m_ulMcsPerRbs;
        UeInfoOf(*itUe)->m_puschRepetitions = static_cast<uint8_t>(m_puschRepetitions);
        UeInfoOf(*itUe)->m_puschRepSinr = std::pow(10.0, m_puschRepSinrDb / 10.0);
        UeInfoOf(*itUe)->m_puschMinBlock = m_puschMinBlockBytes;
        UeInfoOf(*itUe)->m_ulWideRbs = GetUlWideRbs();
        UeInfoOf(*itUe)->m_ulNarrowRbs = std::max(GetUlGrantRbs(), 1U);
        UeInfoOf(*itUe)->m_ulWideSinr = std::pow(10.0, m_ulWideSinrDb / 10.0);
        if (PdcchOn())
        {
            m_pdcch->AddUe(params.m_rnti); // the DCIs of its RRC messages
        }

        NrMacSchedulerSrs::SrsPeriodicityAndOffset srs = m_schedulerSrs->AddUe();

        if (!srs.m_isValid)
        {
            bool ret = m_schedulerSrs->IncreasePeriodicity(
                &m_ueMap); // The new UE will get the SRS offset/periodicity here
            NS_ASSERT(ret);
        }
        else
        {
            UeInfoOf(*itUe)->m_srsPeriodicity =
                srs.m_periodicity; // set the periodicity/offset based on the return value
            UeInfoOf(*itUe)->m_srsOffset = srs.m_offset;
        }

        NS_LOG_INFO("Creating user, beam " << params.m_beamId << " and ue " << params.m_rnti
                                           << " assigned SRS periodicity " << srs.m_periodicity
                                           << " and offset " << srs.m_offset);
    }
    else
    {
        NS_LOG_LOGIC("Updating Beam for UE " << params.m_rnti << " beam " << params.m_beamId);
        UeInfoOf(*itUe)->m_beamId = params.m_beamId;
    }
}

/**
 * @brief Release an UE
 * @param params params of the UE to release
 *
 * Remove the UE from the ueMap (m_ueMap) and release its SRS offset for
 * later usage.
 */
void
NrMacSchedulerNs3::DoCschedUeReleaseReq(
    const NrMacCschedSapProvider::CschedUeReleaseReqParameters& params)
{
    NS_LOG_FUNCTION(this << " Release RNTI " << params.m_rnti);

    auto itUe = m_ueMap.find(params.m_rnti);
    NS_ABORT_IF(itUe == m_ueMap.end());

    m_schedulerSrs->RemoveUe(itUe->second->m_srsOffset);
    m_ueMap.erase(itUe);
    std::erase(m_srList, params.m_rnti);

    // Drop any msg3 retransmission state still keyed to this RNTI. Without
    // this, an entry from a TC-RNTI that never completed random access
    // survives the release, and a later UE reusing the same numeric RNTI
    // would inherit a bogus msg3 retransmission grant.
    m_msg3Pending.erase(params.m_rnti);
    std::erase(m_msg3RetxList, params.m_rnti);

    m_dlPdcchLast.erase(params.m_rnti);
    m_ulPdcchLast.erase(params.m_rnti);
    for (auto it = m_ulMeasuredHarq.begin(); it != m_ulMeasuredHarq.end();)
    {
        it = it->first == params.m_rnti ? m_ulMeasuredHarq.erase(it) : std::next(it);
    }
    m_spsDl.erase(params.m_rnti);
    m_spsUl.erase(params.m_rnti);
    m_spsUlWholeBandMcs.erase(params.m_rnti);
    if (m_pdcch)
    {
        m_pdcch->RemoveUe(params.m_rnti);
    }

    // We clean m_ulAllocationMap in two steps, first we remove vector entries from map items
    // Later we remove map items with empty vectors
    NS_LOG_INFO("Removing uplink allocations for released RNTI " << params.m_rnti);
    for (auto& slotAlloc : m_ulAllocationMap)
    {
        std::erase_if(slotAlloc.second.m_ulAllocations,
                      [rnti = params.m_rnti](auto& allocElem) { return allocElem.m_rnti == rnti; });
    }

    NS_LOG_INFO("Release RNTI " << params.m_rnti);
}

uint64_t
NrMacSchedulerNs3::GetNumRbPerRbg() const
{
    return m_macSchedSapUser->GetNumRbPerRbg();
}

/**
 * @brief Create a logical channel starting from a configuration
 * @param config configuration of the logical channel
 *
 * A subclass can return its own representation of a logical channel by
 * implementing a proper subclass of NrMacSchedulerLC and returning a
 * pointer to a newly created instance.
 *
 * @return a pointer to the representation of a logical channel
 */
LCPtr
NrMacSchedulerNs3::CreateLC(const nr::LogicalChannelConfigListElement_s& config) const
{
    NS_LOG_FUNCTION(this);
    return std::make_unique<NrMacSchedulerLC>(config);
}

/**
 * @brief Create a logical channel group starting from a configuration
 * @param config configuration of the logical channel group
 *
 * A subclass can return its own representation of a logical channel by
 * implementing a proper subclass of NrMacSchedulerLCG and returning a
 * pointer to a newly created instance.
 *
 * @return a pointer to the representation of a logical channel group
 */
LCGPtr
NrMacSchedulerNs3::CreateLCG(const nr::LogicalChannelConfigListElement_s& config) const
{
    NS_LOG_FUNCTION(this);
    return std::make_unique<NrMacSchedulerLCG>(config.m_logicalChannelGroup);
}

/**
 * @brief Configure a logical channel for a UE
 * @param params the params of the LC
 *
 * The UE should be previously registered in the UE map. Then, for each logical
 * channel to configure, the UE representation is updated, creating an empty
 * LC. When the direction is set to DIR_BOTH, both UL and DL are created.
 *
 * Each LC is assigned to a LC group (LCG). If the group does not exists, then
 * it is created through the method CreateLCG, and then saved in the UE representation.
 * If the LCG exists or has been created, then the LC creation is done
 * through the method CreateLC and then saved in the UE representation.
 */
void
NrMacSchedulerNs3::DoCschedLcConfigReq(
    const NrMacCschedSapProvider::CschedLcConfigReqParameters& params)
{
    NS_LOG_FUNCTION(this << static_cast<uint32_t>(params.m_rnti));
    auto itUe = m_ueMap.find(params.m_rnti);
    GetSecond UeInfoOf;
    NS_ABORT_IF(itUe == m_ueMap.end());

    for (const auto& lcConfig : params.m_logicalChannelConfigList)
    {
        if (lcConfig.m_direction == nr::LogicalChannelConfigListElement_s::DIR_DL ||
            lcConfig.m_direction == nr::LogicalChannelConfigListElement_s::DIR_BOTH)
        {
            auto itDl = UeInfoOf(*itUe)->m_dlLCG.find(lcConfig.m_logicalChannelGroup);
            auto itDlEnd = UeInfoOf(*itUe)->m_dlLCG.end();
            if (itDl == itDlEnd)
            {
                NS_LOG_DEBUG("Created DL LCG for UE "
                             << UeInfoOf(*itUe)->m_rnti
                             << " ID=" << static_cast<uint32_t>(lcConfig.m_logicalChannelGroup));
                std::unique_ptr<NrMacSchedulerLCG> lcg = CreateLCG(lcConfig);
                itDl = UeInfoOf(*itUe)
                           ->m_dlLCG.emplace(lcConfig.m_logicalChannelGroup, std::move(lcg))
                           .first;
            }

            itDl->second->Insert(CreateLC(lcConfig));
            NS_LOG_DEBUG("Created DL LC for UE "
                         << UeInfoOf(*itUe)->m_rnti
                         << " ID=" << static_cast<uint32_t>(lcConfig.m_logicalChannelIdentity)
                         << " in LCG " << static_cast<uint32_t>(lcConfig.m_logicalChannelGroup));
        }
        if (lcConfig.m_direction == nr::LogicalChannelConfigListElement_s::DIR_UL ||
            lcConfig.m_direction == nr::LogicalChannelConfigListElement_s::DIR_BOTH)
        {
            auto itUl = UeInfoOf(*itUe)->m_ulLCG.find(lcConfig.m_logicalChannelGroup);
            auto itUlEnd = UeInfoOf(*itUe)->m_ulLCG.end();
            if (itUl == itUlEnd)
            {
                NS_LOG_DEBUG("Created UL LCG for UE "
                             << UeInfoOf(*itUe)->m_rnti
                             << " ID=" << static_cast<uint32_t>(lcConfig.m_logicalChannelGroup));
                std::unique_ptr<NrMacSchedulerLCG> lcg = CreateLCG(lcConfig);
                itUl = UeInfoOf(*itUe)
                           ->m_ulLCG.emplace(lcConfig.m_logicalChannelGroup, std::move(lcg))
                           .first;
            }

            // Create a LC ID only if it is the first. For detail, see documentation
            // of NrMacSchedulerLCG.
            if (itUl->second->NumOfLC() == 0)
            {
                itUl->second->Insert(CreateLC(lcConfig));
                NS_LOG_DEBUG("Created UL LC for UE "
                             << UeInfoOf(*itUe)->m_rnti
                             << " ID=" << static_cast<uint32_t>(lcConfig.m_logicalChannelIdentity)
                             << " in LCG "
                             << static_cast<uint32_t>(lcConfig.m_logicalChannelGroup));
            }
        }
    }
}

/**
 * @brief Release a LC
 * @param params params of the LC to release.
 */
void
NrMacSchedulerNs3::DoCschedLcReleaseReq(
    const NrMacCschedSapProvider::CschedLcReleaseReqParameters& params)
{
    NS_LOG_FUNCTION(this);

    for (const auto& lcId : params.m_logicalChannelIdentity)
    {
        auto itUe = m_ueMap.find(params.m_rnti);
        NS_ABORT_IF(itUe == m_ueMap.end());
        itUe->second->ReleaseLC(lcId);
    }
}

/**
 * @brief RLC informs of DL data
 * @param params parameters of the function
 *
 * The message contains the LC and the amount of data buffered. Therefore,
 * in this method we cycle through all the UE LCG to find the LC, and once
 * it is found, it is updated with the new amount of data.
 */
void
NrMacSchedulerNs3::DoSchedDlRlcBufferReq(
    const NrMacSchedSapProvider::SchedDlRlcBufferReqParameters& params)
{
    NS_LOG_FUNCTION(this << params.m_rnti
                         << static_cast<uint32_t>(params.m_logicalChannelIdentity));

    GetSecond UeInfoOf;
    auto itUe = m_ueMap.find(params.m_rnti);
    NS_ABORT_IF(itUe == m_ueMap.end());

    for (const auto& lcg : UeInfoOf(*itUe)->m_dlLCG)
    {
        if (lcg.second->Contains(params.m_logicalChannelIdentity))
        {
            NS_LOG_INFO("Updating DL LC Info: " << params
                                                << " in LCG: " << static_cast<uint32_t>(lcg.first));
            lcg.second->UpdateInfo(params);

            if (m_spsScenario != 0 &&
                lcg.second->GetFiveQi(params.m_logicalChannelIdentity) == m_spsFiveQi)
            {
                // The RLC reports its queue when a packet has come (and when it has sent a
                // part of one). Was the last packet a voice frame or a silence descriptor?
                SpsState& st = m_spsDl[params.m_rnti];
                st.lastKind = NrPdcp::GetLastSduKind(GetCellId(),
                                                     params.m_rnti,
                                                     params.m_logicalChannelIdentity,
                                                     true);
                // The first packet after the speaker started or stopped talking: how it is
                // sent is counted when it is (SpsAllocate, SettlePdcch)
                bool packets;
                st.change |= SpsVoiceChanges(st,
                                             params.m_rnti,
                                             params.m_logicalChannelIdentity,
                                             true,
                                             &packets);
            }

            if (m_nrFhSchedSapProvider)
            {
                m_nrFhSchedSapProvider->SetActiveUe(GetBwpId(),
                                                    params.m_rnti,
                                                    lcg.second->GetTotalSize());
            }
            return;
        }
    }
    // Fail miserabily because we didn't found any LC
    NS_FATAL_ERROR("The LC does not exist. Can't update");
}

/**
 * @brief Update the UL LC
 * @param bsr BSR received
 *
 * The UE notifies the buffer size as a sum of all the components. The BSR
 * is a vector of 4 uint8_t that represents the amount of data in each
 * LCG. A call to NrMacSchedulerLCG::UpdateInfo is then issued with
 * the amount of data as parameter.
 */
void
NrMacSchedulerNs3::BSRReceivedFromUe(const MacCeElement& bsr)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(bsr.m_macCeType == MacCeElement::BSR);
    GetSecond UeInfoOf;
    auto itUe = m_ueMap.find(bsr.m_rnti);
    NS_ABORT_IF(itUe == m_ueMap.end());

    // The buffer status is now known, so the bootstrap grant the scheduling request asked for is
    // no longer owed: from here on the uplink allocation follows the reported buffer.
    UeInfoOf(*itUe)->m_srBytesPending = 0;

    // The UE only notifies the buf size as sum of all components.
    // see nr-ue-mac.cc:395
    for (uint8_t lcg = 0; lcg < 4; ++lcg)
    {
        uint8_t bsrId = bsr.m_macCeValue.m_bufferStatus.at(lcg);
        uint32_t bufSize = NrMacShortBsrCe::FromLevelToBytes(bsrId);

        auto itLcg = UeInfoOf(*itUe)->m_ulLCG.find(lcg);
        if (itLcg == UeInfoOf(*itUe)->m_ulLCG.end())
        {
            // NS_ABORT_MSG_IF(bufSize > 0,
            //                 "LCG " << static_cast<uint32_t>(lcg) << " not found for UE "
            //                        << itUe->second->m_rnti);
            continue;
        }

        if (itLcg->second->GetTotalSize() > 0 || bufSize > 0)
        {
            NS_LOG_INFO("Updating UL LCG " << static_cast<uint32_t>(lcg) << " for UE " << bsr.m_rnti
                                           << " size " << bufSize);
        }

        itLcg->second->UpdateInfo(bufSize);
    }
}

/**
 * @brief Evaluate different types of control messages (only BSR for the moment)
 * @param params parameters of the control message
 *
 * For each BSR received, calls BSRReceivedFromUe. Ignore all the others control
 * messages.
 */
void
NrMacSchedulerNs3::DoSchedUlMacCtrlInfoReq(
    const NrMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters& params)
{
    NS_LOG_FUNCTION(this);

    for (const auto& element : params.m_macCeList)
    {
        if (element.m_macCeType == MacCeElement::BSR)
        {
            BSRReceivedFromUe(element);
        }
        else
        {
            NS_LOG_INFO("Ignoring received CTRL message because it's not a BSR");
        }
    }
}

/**
 * @brief Received a DL CQI message
 * @param params DL CQI message
 *
 * For each message in the list, calculate the expiration time in number of slots,
 * and then pass all the information to the NrMacSchedulerCQIManagement class.
 *
 * If the CQI is sub-band, the method NrMacSchedulerCQIManagement::SBCQIReported
 * will be called, otherwise NrMacSchedulerCQIManagement::WBCQIReported.
 */
void
NrMacSchedulerNs3::DoSchedDlCqiInfoReq(
    const NrMacSchedSapProvider::SchedDlCqiInfoReqParameters& params)
{
    NS_LOG_FUNCTION(this);

    // The SINR a UE reports with (or without) its CQI: the PDCCH aggregation level is chosen
    // from it (NrPdcch), by the scheduler of the downlink and by the one of the uplink carrier
    if (PdcchOn())
    {
        for (const auto& cqi : params.m_cqiList)
        {
            if (!std::isnan(cqi.m_sinrDb) && m_ueMap.contains(cqi.m_rnti))
            {
                m_pdcch->ReportSinr(cqi.m_rnti, cqi.m_sinrDb);
            }
        }
    }

    if (m_fixedMcsDl)
    {
        return;
    }

    NS_ASSERT(m_cqiTimersThreshold >= m_macSchedSapUser->GetSlotPeriod());

    uint32_t expirationTime =
        static_cast<uint32_t>(m_cqiTimersThreshold.GetNanoSeconds() /
                              m_macSchedSapUser->GetSlotPeriod().GetNanoSeconds());

    for (const auto& cqi : params.m_cqiList)
    {
        auto itUe = m_ueMap.find(cqi.m_rnti);
        if (itUe == m_ueMap.end())
        {
            // A DL CQI may legitimately arrive after the UE has been released from this
            // cell, e.g., after a handover or an RLF. There is nowhere to store it, so
            // drop the stale report instead of dereferencing an invalid iterator.
            NS_LOG_INFO("Dropping DL CQI for RNTI "
                        << cqi.m_rnti
                        << " because there is no UE context for it. This is expected when the "
                           "UE was released before the CQI arrived.");
            continue;
        }
        if (cqi.m_sinrOnly)
        {
            continue; // a SINR report: no CQI in it
        }
        const std::shared_ptr<NrMacSchedulerUeInfo>& ue = itUe->second;
        m_cqiManagement.DlCqiReported(cqi, ue, expirationTime, m_maxDlMcs, GetBandwidthInRbg());
        m_csiFeedbackReceived(GetCellId(), GetBwpId(), ue);
    }
}

/**
 * @brief Received a UL CQI message
 * @param params UL CQI message
 *
 * Calculate the expiration time in number of slots, and then pass all the
 * information to the NrMacSchedulerCQIManagement class.
 *
 * In UL, we have to know the previously allocated symbols and the total TBS
 * to be able to calculate CQI and MCS, so a special stack is maintained
 * (m_ulAllocationMap).
 *
 * Only UlCqiInfo::PUSCH is currently supported.
 */
void
NrMacSchedulerNs3::DoSchedUlCqiInfoReq(
    const NrMacSchedSapProvider::SchedUlCqiInfoReqParameters& params)
{
    NS_LOG_FUNCTION(this);

    if (m_fixedMcsUl)
    {
        return;
    }

    GetSecond UeInfoOf;

    uint32_t expirationTime =
        static_cast<uint32_t>(m_cqiTimersThreshold.GetNanoSeconds() /
                              m_macSchedSapUser->GetSlotPeriod().GetNanoSeconds());

    switch (params.m_ulCqi.m_type)
    {
    case UlCqiInfo::PUSCH: {
        bool found = false;
        uint8_t symStart = params.m_symStart;
        SfnSf ulSfnSf = params.m_sfnSf;

        NS_LOG_INFO("CQI for allocation: " << params.m_sfnSf << " started at sym: " << +symStart
                                           << " modified allocation " << ulSfnSf << " sym Start "
                                           << static_cast<uint32_t>(symStart));

        auto itAlloc = m_ulAllocationMap.find(ulSfnSf.GetEncoding());
        // NS_ASSERT_MSG(itAlloc != m_ulAllocationMap.end(), "Can't find allocation for " <<
        // ulSfnSf);
        if (itAlloc == m_ulAllocationMap.end() ||
            (itAlloc != m_ulAllocationMap.end() && itAlloc->second.m_ulAllocations.empty()))
        {
            NS_LOG_INFO("Dropping UL CQI for "
                        << ulSfnSf << " symStart=" << +symStart
                        << " because there is no pending UL allocation left for that slot. "
                           "This can happen when the matching allocation was removed earlier, "
                           "for example after UE release following RLF.");
            break; // early exit because there is nothing allocated to do
        }
        std::vector<AllocElem>& ulAllocations = itAlloc->second.m_ulAllocations;

        for (auto it = ulAllocations.cbegin(); it != ulAllocations.cend(); /* NO INC */)
        {
            const AllocElem& allocation = *(it);
            if (allocation.m_symStart == symStart)
            {
                auto itUe = m_ueMap.find(allocation.m_rnti);
                if (itUe == m_ueMap.end())
                {
                    // No UE context for this RNTI: e.g. a msg3 (RRC Connection
                    // Request) retransmission grant addressed to a T-C-RNTI that is
                    // not yet a registered UE. There is nowhere to store a CQI, so
                    // consume the allocation and skip it.
                    found = true;
                    it = ulAllocations.erase(it);
                    continue;
                }
                NS_ASSERT(allocation.m_numSym > 0);
                NS_ASSERT(allocation.m_tbs > 0);

                m_cqiManagement.UlSBCQIReported(expirationTime,
                                                allocation.m_tbs,
                                                params,
                                                UeInfoOf(*itUe),
                                                allocation.m_rbgMask,
                                                m_macSchedSapUser->GetNumRbPerRbg(),
                                                m_macSchedSapUser->GetSpectrumModel());
                if (m_ulSinrAvgTime.IsStrictlyPositive())
                {
                    SmoothUlSinr(UeInfoOf(*itUe));
                }
                if (m_spsScenario != 0)
                {
                    // For the blocks of a configured grant (SpsUlMcs()): the MCS the UE would
                    // have with this transmit power spread over the RBs of such a block, the
                    // whole band or UlGrantRbs. Its SINR per resource block is then lower or
                    // higher by the ratio of the widths.
                    const auto& ue = UeInfoOf(*itUe);
                    const uint32_t rbPerRbg = m_macSchedSapUser->GetNumRbPerRbg();
                    const auto band = GetUlBitmask();
                    const auto& sinr = params.m_ulCqi.m_sinr;
                    double sum = 0.0;
                    uint32_t had = 0;
                    uint32_t whole = 0;
                    for (uint32_t i = 0; i < band.size(); ++i)
                    {
                        for (uint32_t k = 0; k < rbPerRbg; ++k)
                        {
                            const uint32_t rb = i * rbPerRbg + k;
                            if (rb >= sinr.size())
                            {
                                continue;
                            }
                            whole += band[i] ? 1 : 0;
                            if (i < allocation.m_rbgMask.size() && allocation.m_rbgMask[i])
                            {
                                sum += sinr[rb];
                                ++had;
                            }
                        }
                    }
                    if (GetUlGrantRbs() > 0)
                    {
                        const uint32_t grantRbg = (GetUlGrantRbs() + rbPerRbg - 1) / rbPerRbg;
                        whole = std::min(whole, grantRbg * rbPerRbg);
                    }
                    uint8_t wholeBandMcs = ue->m_ulMcs;
                    if (had > 0 && whole > 0 && had != whole)
                    {
                        SpectrumValue spread(m_macSchedSapUser->GetSpectrumModel());
                        const double value = sum / whole; // (sum / had) x (had / whole)
                        uint32_t filled = 0;
                        for (uint32_t i = 0; i < band.size(); ++i)
                        {
                            for (uint32_t k = 0; k < rbPerRbg && band[i] && filled < whole; ++k)
                            {
                                if (i * rbPerRbg + k < sinr.size())
                                {
                                    spread[i * rbPerRbg + k] = value;
                                    ++filled;
                                }
                            }
                        }
                        m_ulAmc->CreateCqiFeedbackSiso(spread, wholeBandMcs);
                    }
                    m_spsUlWholeBandMcs[ue->m_rnti] = std::make_pair(ue->m_ulMcs, wholeBandMcs);
                }
                found = true;
                it = ulAllocations.erase(it);
            }
            else
            {
                ++it;
            }
        }
        if (!found)
        {
            // A CQI may legitimately arrive after its UL allocation has already been removed
            // from the pending map, e.g., after UE release due to RLF.
            NS_LOG_INFO("Dropping unmatched UL CQI for "
                        << ulSfnSf << " symStart=" << +symStart
                        << " because no pending UL allocation in that slot matches it. "
                           "This is expected when the UE was released before the CQI arrived.");
            return;
        }

        if (ulAllocations.empty())
        {
            // remove obsolete info on allocation; we already processed all the CQI
            NS_LOG_INFO("Removing allocation for " << ulSfnSf);
            m_ulAllocationMap.erase(itAlloc);
        }
    }
    break;
    default:
        NS_FATAL_ERROR("Unknown type of UL-CQI");
    }
}

/**
 * @brief Merge newly received HARQ feedbacks with existing feedbacks
 * @param existingFeedbacks a vector of old feedback (will be empty at the end)
 * @param inFeedbacks Received feedbacks
 * @param mode UL or DL, for debug printing
 * @return a vector of all the feedbacks (new + old)
 *
 * It is possible that, in one slot, some HARQ could not be transmitted (by
 * choice, or because there are not available resources). These feedbacks are
 * the 'old' ones, that should be merged with the newly arrived (the feedbacks
 * that arrived in the current slot) before processing.
 *
 */
template <typename T>
std::vector<T>
NrMacSchedulerNs3::MergeHARQ(std::vector<T>* existingFeedbacks,
                             const std::vector<T>& inFeedbacks,
                             const std::string& mode) const
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("To retransmit : " << existingFeedbacks->size() << " " << mode << " HARQ, received "
                                   << inFeedbacks.size() << " " << mode << " HARQ Feedback");
    uint64_t existingSize = existingFeedbacks->size();
    uint64_t inSize = inFeedbacks.size();
    existingFeedbacks->insert(existingFeedbacks->end(), inFeedbacks.begin(), inFeedbacks.end());
    NS_ASSERT(existingFeedbacks->size() == existingSize + inSize);

    auto ret = std::vector<T>(std::make_move_iterator(existingFeedbacks->begin()),
                              std::make_move_iterator(existingFeedbacks->end()));
    existingFeedbacks->clear();

    return ret;
}

/**
 * @brief Process HARQ feedbacks
 * @param harqInfo all the known HARQ feedbacks (can be UL or DL)
 * @param GetHarqVectorFn Function to retrieve the correct Harq Vector
 * @param direction "UL" or "DL" for debug messages
 *
 * For every received feedback (even the already processed ones) the method
 * checks if the feedback is ACK or NACK. In case of ACK (represented by
 * HarqInfo::IsReceivedOk) the feedback is eliminated and the corresponding
 * HARQ process erased; if the feedback is NACK, the corresponding process
 * is marked for retransmission. The decision to retransmit or not the process
 * will be taken later.
 *
 * @see DlHarqInfo
 * @see UlHarqInfo
 * @see HarqProcess
 */
void
NrMacSchedulerNs3::UpdateDlOlla(uint16_t rnti, bool ack) const
{
    if (m_ollaStepUp <= 0.0)
    {
        return;
    }
    const auto& ue = m_ueMap.find(rnti)->second;
    // Pushed from here rather than set at UE creation: the offset is 0 until the first
    // feedback anyway, so GetDlMcs() is unaffected before this runs.
    ue->m_dlOllaMax = m_ollaMaxOffset;
    // Stepping up on ACK by s and down on NACK by s*(1-target)/target gives an expected drift
    // of (s/target)*(target - p), so the offset is stationary exactly where the measured
    // initial BLER p equals the target. No BLER counter or window needed.
    const double stepDown = m_ollaStepUp * (1.0 - m_ollaTargetBler) / m_ollaTargetBler;
    const double raw = ue->m_dlOlla + (ack ? m_ollaStepUp : -stepDown);
    ue->m_dlOlla = std::clamp(raw, -m_ollaMaxOffset, m_ollaMaxOffset);
    // Clamped means the loop has run out of authority: a sweep is then measuring the clamp,
    // not the step size, so this has to be visible rather than inferred.
    m_dlOllaUpdate(rnti, ue->m_dlOlla, ack, raw != ue->m_dlOlla);
}

void
NrMacSchedulerNs3::SmoothUlSinr(const std::shared_ptr<NrMacSchedulerUeInfo>& ue) const
{
    // The report has just replaced ue->m_ulSinrMeas (UlSBCQIReported()). Its level, as the SINR
    // of one resource block (the UE's power is spread over the RBs it was given), goes into the
    // mean; the mean takes the place of the report's level and the report keeps its shape over
    // the RBs. The MCS and the repetitions are computed from m_ulSinrMeas later, per grant.
    const auto n = ue->m_ulSinrMeas.size();
    if (n == 0)
    {
        return;
    }
    double sum = 0.0;
    for (const double v : ue->m_ulSinrMeas)
    {
        sum += v;
    }
    const double oneRb = sum; // mean per RB times the RBs = the sum
    if (oneRb <= 0.0)
    {
        return;
    }
    const int64_t now = Simulator::Now().GetNanoSeconds();
    if (ue->m_ulSinrFilt > 0.0 && ue->m_ulSinrFiltAtNs >= 0)
    {
        const double dt = static_cast<double>(now - ue->m_ulSinrFiltAtNs);
        const double w = 1.0 - std::exp(-dt / static_cast<double>(m_ulSinrAvgTime.GetNanoSeconds()));
        ue->m_ulSinrFilt = (1.0 - w) * ue->m_ulSinrFilt + w * oneRb;
    }
    else
    {
        ue->m_ulSinrFilt = oneRb;
    }
    ue->m_ulSinrFiltAtNs = now;
    const double scale = ue->m_ulSinrFilt / oneRb;
    for (double& v : ue->m_ulSinrMeas)
    {
        v *= scale;
    }
}

void
NrMacSchedulerNs3::UpdateUlOlla(uint16_t rnti, bool ack, uint8_t mcs, bool measured) const
{
    if (!m_ulOlla || m_ollaStepUp <= 0.0)
    {
        return;
    }
    const auto& ue = m_ueMap.find(rnti)->second;
    if (!measured || (!ack && mcs == 0) || (ack && mcs >= ue->m_ulAmc->GetMaxMcs()))
    {
        return;
    }
    ue->m_ulOllaMax = m_ollaMaxOffset;
    const double stepDown = m_ollaStepUp * (1.0 - m_ollaTargetBler) / m_ollaTargetBler;
    ue->m_ulOlla = std::clamp(ue->m_ulOlla + (ack ? m_ollaStepUp : -stepDown),
                              -m_ollaMaxOffset,
                              m_ollaMaxOffset);
}

std::set<uint16_t>
NrMacSchedulerNs3::PlaceUlRepetitions(const PointInFTPlane& start,
                                      uint8_t numSym,
                                      SlotAllocInfo* allocInfo)
{
    NS_LOG_FUNCTION(this);
    std::set<uint16_t> ues;
    // The UEs with a block of a configured grant in this slot have their PUSCH already
    std::set<uint16_t> busy;
    for (const auto& alloc : allocInfo->m_varTtiAllocInfo)
    {
        const auto& dci = alloc.m_dci;
        if (dci->m_format == DciInfoElementTdma::UL && dci->m_type == DciInfoElementTdma::DATA &&
            (dci->m_sps == DciInfoElementTdma::SPS_ACTIVATION ||
             dci->m_sps == DciInfoElementTdma::SPS_OCCASION))
        {
            busy.insert(dci->m_rnti);
        }
    }
    auto& free = m_ulRbgFreeForData;
    for (auto it = m_ulRepetitions.begin(); it != m_ulRepetitions.end();)
    {
        const auto first = it->dci;
        if (m_ueMap.find(first->m_rnti) == m_ueMap.end())
        {
            it = m_ulRepetitions.erase(it); // the UE has gone (a radio link failure)
            continue;
        }
        if (ues.contains(first->m_rnti) || busy.contains(first->m_rnti))
        {
            ++it; // one PUSCH per UE and slot: its next block waits
            continue;
        }
        const auto need = static_cast<std::size_t>(
            std::count(first->m_rbgBitmask.begin(), first->m_rbgBitmask.end(), true));
        bool same = first->m_rbgBitmask.size() == free.size();
        for (std::size_t i = 0; same && i < free.size(); ++i)
        {
            same = !first->m_rbgBitmask[i] || free[i];
        }
        std::vector<bool> mask(free.size(), false);
        std::size_t got = 0;
        if (same)
        {
            mask = first->m_rbgBitmask;
            got = need;
        }
        else
        {
            for (std::size_t i = 0; i < free.size() && got < need; ++i)
            {
                if (free[i])
                {
                    mask[i] = true;
                    ++got;
                }
            }
        }
        if (got < need)
        {
            NS_LOG_INFO("No room for repetition " << +it->next << " of UE " << first->m_rnti);
            ++it;
            continue;
        }
        for (std::size_t i = 0; i < free.size(); ++i)
        {
            free[i] = free[i] && !mask[i];
        }
        auto dci = std::make_shared<DciInfoElementTdma>(start.m_sym - numSym,
                                                        numSym,
                                                        0,
                                                        first->m_rv,
                                                        *first);
        dci->m_rbgBitmask = std::move(mask);
        dci->m_repIndex = it->next;
        dci->m_sps = DciInfoElementTdma::PUSCH_REPETITION;
        dci->m_repFirst = first;
        if (first->m_sps != DciInfoElementTdma::DYNAMIC)
        {
            // A block of a configured grant: if the UE leaves it unused, these go too
            // (SpsPlanUl())
            auto sps = m_spsUl.find(first->m_rnti);
            if (sps != m_spsUl.end())
            {
                for (auto& sent : sps->second.blocks)
                {
                    if (sent.dci == first)
                    {
                        sent.repSfns.push_back(allocInfo->m_sfnSf.GetEncoding());
                    }
                }
            }
        }
        NS_LOG_INFO("UE " << first->m_rnti << " repetition " << +it->next << " of "
                          << +first->m_repetitions << ", HARQ process " << +first->m_harqProcess);
        allocInfo->m_varTtiAllocInfo.emplace_back(dci);
        ues.insert(first->m_rnti);
        if (++it->next >= first->m_repetitions)
        {
            it = m_ulRepetitions.erase(it);
        }
        else
        {
            ++it;
        }
    }
    return ues;
}

void
NrMacSchedulerNs3::QueueUlRepetitions(const SlotAllocInfo& allocInfo)
{
    for (const auto& alloc : allocInfo.m_varTtiAllocInfo)
    {
        const auto& dci = alloc.m_dci;
        // (a grant with a DCI, or a block of a configured grant)
        if (dci->m_format == DciInfoElementTdma::UL && dci->m_type == DciInfoElementTdma::DATA &&
            (dci->m_sps == DciInfoElementTdma::DYNAMIC ||
             dci->m_sps == DciInfoElementTdma::SPS_ACTIVATION ||
             dci->m_sps == DciInfoElementTdma::SPS_OCCASION) &&
            dci->m_repetitions > 1 && dci->m_repIndex == 0)
        {
            m_ulRepetitions.push_back({dci, 1});
        }
    }
}

uint8_t
NrMacSchedulerNs3::DlAggregationFor(const DciInfoElementTdma& dci, bool sps) const
{
    const uint16_t rnti = dci.m_rnti;
    if (m_pdschAggregation != 0)
    {
        NS_ABORT_MSG_UNLESS(m_pdschAggregation == 1 || m_pdschAggregation == 2 ||
                                m_pdschAggregation == 4 || m_pdschAggregation == 8,
                            "PdschAggregation " << m_pdschAggregation
                                                << " is not 0, 1, 2, 4 or 8 (pdsch-AggregationFactor)");
        return static_cast<uint8_t>(m_pdschAggregation);
    }
    NS_ABORT_MSG_UNLESS(m_pdcch && m_pdcch->IsEnabled(), "PdschAggregation = 0 needs the PDCCH model (NrPdcch)");
    // Power first, as for PUSCH (a UE repeats only at full power): the L1-SINR the UE reports
    // (control symbols, at the RB share) plus what the gNB can give the block's RBs over their
    // share (the idle RBs' power, MaxRbBoostDb, the PFD limit and the closed loop's cap of the UE:
    // NrGnbPhy::GetDataBoostDb). K covers what is left.
    double sinrDb = m_pdcch->GetSinr(rnti);
    if (std::isnan(sinrDb))
    {
        return 1; // no report yet
    }
    if (const NrGnbPhy* phy = NrGnbPhy::FindByCell(GetCellId()))
    {
        const auto rbgs = static_cast<uint32_t>(
            std::count(dci.m_rbgBitmask.begin(), dci.m_rbgBitmask.end(), true));
        sinrDb += phy->GetDataBoostDb(rnti, rbgs * GetNumRbPerRbg());
    }
    // repetitionNumber-r16 (dynamic), pdsch-AggregationFactor-r16 (SPS)
    static const std::vector<uint8_t> dynamicValues{1, 2, 3, 4, 5, 6, 7, 8, 16};
    static const std::vector<uint8_t> spsValues{1, 2, 4, 8};
    const auto& values = sps ? spsValues : dynamicValues;
    for (const uint8_t k : values)
    {
        if (sinrDb + 10.0 * std::log10(k) >= m_pdschAggSinrDb)
        {
            return k;
        }
    }
    return values.back();
}

std::set<uint16_t>
NrMacSchedulerNs3::DlRepeatingUes() const
{
    std::set<uint16_t> ues;
    for (const auto& rep : m_dlRepetitions)
    {
        ues.insert(rep.dci->m_rnti);
    }
    return ues;
}

std::set<uint16_t>
NrMacSchedulerNs3::PlaceDlRepetitions(const PointInFTPlane& start,
                                      uint8_t numSym,
                                      SlotAllocInfo* allocInfo)
{
    NS_LOG_FUNCTION(this);
    std::set<uint16_t> ues;
    // The UEs with a block of an SPS configuration in this slot have their PDSCH already
    std::set<uint16_t> busy;
    for (const auto& alloc : allocInfo->m_varTtiAllocInfo)
    {
        const auto& dci = alloc.m_dci;
        if (dci->m_format == DciInfoElementTdma::DL && dci->m_type == DciInfoElementTdma::DATA)
        {
            busy.insert(dci->m_rnti);
        }
    }
    auto& free = m_dlRbgFreeForData;
    for (auto it = m_dlRepetitions.begin(); it != m_dlRepetitions.end();)
    {
        const auto first = it->dci;
        const auto ue = m_ueMap.find(first->m_rnti);
        // The process must still hold the block: one reclaimed (ResetExpiredHARQ) and reused
        // would have the gNB MAC send another block under this one's DCI
        if (ue == m_ueMap.end() || !ue->second->m_dlHarq.Exist(first->m_harqProcess) ||
            ue->second->m_dlHarq.Get(first->m_harqProcess).m_dciElement != first)
        {
            NS_LOG_INFO("PDSCH repetition of RNTI " << first->m_rnti << " process "
                                                    << +first->m_harqProcess << " dropped");
            it = m_dlRepetitions.erase(it);
            continue;
        }
        if (ues.contains(first->m_rnti) || busy.contains(first->m_rnti))
        {
            ++it; // one PDSCH per UE and slot: its next block waits
            continue;
        }
        // As many RBGs over numSym symbols as keep the resource elements of the first
        const auto rbgsFirst = static_cast<uint32_t>(
            std::count(first->m_rbgBitmask.begin(), first->m_rbgBitmask.end(), true));
        const uint32_t units = rbgsFirst * first->m_numSym;
        const uint32_t need = numSym > 0 ? (units + numSym - 1) / numSym : UINT32_MAX;
        std::vector<bool> mask(free.size(), false);
        uint32_t got = 0;
        for (std::size_t i = 0; i < free.size() && got < need; ++i)
        {
            if (i < first->m_rbgBitmask.size() && first->m_rbgBitmask[i] && free[i])
            {
                mask[i] = true;
                ++got;
            }
        }
        for (std::size_t i = 0; i < free.size() && got < need; ++i)
        {
            if (!mask[i] && free[i])
            {
                mask[i] = true;
                ++got;
            }
        }
        if (got < need || units == 0)
        {
            NS_LOG_INFO("No room for PDSCH repetition " << +it->next << " of UE " << first->m_rnti);
            ++it;
            continue;
        }
        for (std::size_t i = 0; i < free.size(); ++i)
        {
            free[i] = free[i] && !mask[i];
        }
        auto dci =
            std::make_shared<DciInfoElementTdma>(start.m_sym, numSym, 0, first->m_rv, *first);
        dci->m_rbgBitmask = std::move(mask);
        dci->m_repIndex = it->next;
        dci->m_sps = DciInfoElementTdma::PDSCH_REPETITION;
        dci->m_repFirst = first;
        // The process waits for its feedback from the last transmission, wherever it lands: a
        // repetition that waited for room moves that later (ResetExpiredHARQ counts from
        // m_txSlot + K - 1)
        {
            auto& process = ue->second->m_dlHarq.Get(first->m_harqProcess);
            const uint64_t now = allocInfo->m_sfnSf.Normalize();
            if (now >= it->next && process.m_txSlot < now - it->next)
            {
                process.m_txSlot = now - it->next;
            }
        }
        NS_LOG_INFO("UE " << first->m_rnti << " PDSCH repetition " << +it->next << " of "
                          << +first->m_repetitions << ", HARQ process " << +first->m_harqProcess);
        allocInfo->m_varTtiAllocInfo.emplace_back(dci);
        ues.insert(first->m_rnti);
        if (++it->next >= first->m_repetitions)
        {
            it = m_dlRepetitions.erase(it);
        }
        else
        {
            ++it;
        }
    }
    return ues;
}

void
NrMacSchedulerNs3::QueueDlRepetitions(const SlotAllocInfo& allocInfo)
{
    for (const auto& alloc : allocInfo.m_varTtiAllocInfo)
    {
        const auto& dci = alloc.m_dci;
        if (dci->m_format != DciInfoElementTdma::DL || dci->m_type != DciInfoElementTdma::DATA ||
            dci->m_repIndex != 0 ||
            (dci->m_sps != DciInfoElementTdma::DYNAMIC &&
             dci->m_sps != DciInfoElementTdma::SPS_ACTIVATION &&
             dci->m_sps != DciInfoElementTdma::SPS_OCCASION))
        {
            continue;
        }
        if (dci->m_ndi == 1)
        {
            // a new block: its K (a retransmission has the K of its block, NrMacSchedulerHarqRr)
            if (dci->m_sps == DciInfoElementTdma::DYNAMIC)
            {
                dci->m_repetitions = DlAggregationFor(*dci, false);
            }
            else
            {
                if (dci->m_sps == DciInfoElementTdma::SPS_ACTIVATION)
                {
                    m_dlSpsAggregation[dci->m_rnti] = DlAggregationFor(*dci, true);
                }
                const auto k = m_dlSpsAggregation.find(dci->m_rnti);
                dci->m_repetitions = k != m_dlSpsAggregation.end() ? k->second : 1;
            }
        }
        if (dci->m_repetitions > 1)
        {
            m_dlRepetitions.push_back({dci, 1});
        }
    }
}

bool
NrMacSchedulerNs3::UseHarqInFrequency() const
{
    if (!HarqInFrequency())
    {
        return false;
    }
    // One beam for all UEs: with several, each beam has symbols of its own
    const BeamId* first = nullptr;
    for (const auto& [rnti, ue] : m_ueMap)
    {
        if (first == nullptr)
        {
            first = &ue->m_beamId;
        }
        else if (!(ue->m_beamId == *first))
        {
            return false;
        }
    }
    const auto fh = GetFhControlMethod();
    return fh != NrFhControl::FhControlMethod::Postponing &&
           fh != NrFhControl::FhControlMethod::OptimizeMcs &&
           fh != NrFhControl::FhControlMethod::OptimizeRBs;
}

template <typename T>
void
NrMacSchedulerNs3::ProcessHARQFeedbacks(
    std::vector<T>* harqInfo,
    const NrMacSchedulerUeInfo::GetHarqVectorFn& GetHarqVectorFn,
    const std::string& direction) const
{
    NS_LOG_FUNCTION(this);
    uint32_t nackReceived = 0;

    // Check the HARQ feedback, erase ACKed, updated NACKed
    for (auto harqFeedbackIt = harqInfo->begin(); harqFeedbackIt != harqInfo->end();
         /* nothing as increment */)
    {
        uint8_t harqId = harqFeedbackIt->m_harqProcessId;
        uint16_t rnti = harqFeedbackIt->m_rnti;
        NrMacHarqVector& ueHarqVector = GetHarqVectorFn(m_ueMap.find(rnti)->second);
        HarqProcess& ueProcess = ueHarqVector.Get(harqId);

        NS_LOG_INFO("Evaluating feedback: " << *harqFeedbackIt);
        if (!ueProcess.m_active)
        {
            NS_LOG_INFO("UE " << rnti << " HARQ vector: " << ueHarqVector);
            NS_FATAL_ERROR("Received feedback for a process which is not active");
        }
        NS_ABORT_IF(ueProcess.m_dciElement == nullptr);

        // RV number should not be greater than 3
        NS_ASSERT(ueProcess.m_dciElement->m_rv < 4);
        uint8_t maxHarqReTx = m_enableHarqReTx ? 3 : 0;

        // Outer-loop link adaptation. Only first transmissions (rv == 0) count: the quantity
        // vendors target is the *initial* BLER, before HARQ recovers anything. Sampled before
        // the ACK branch below erases the process.
        // A NACK whose retransmission found no room comes through here again in every slot
        // until it is placed (the retransmission scheduler puts the process back in
        // WAITING_FEEDBACK before it knows whether the block fits): m_ollaCounted makes it
        // count once (before 1 Nov 2026 it counted in every slot it waited).
        // A block whose DCI the UE missed was not received (downlink) or never sent (uplink):
        // that is no outcome of the MCS, and a gNB tells it apart (no HARQ-ACK, no PUSCH).
        // A downlink block sent K times (PdschAggregation) is decoded after K combined: its
        // outcome says nothing of the MCS of one transmission, which the loop steers.
        // A downlink block whose MCS was lowered to fill its RBs (DlMcsFill) neither: it was
        // sent at a lower MCS than the loop's.
        if (ueProcess.m_dciElement->m_rv == 0 &&
            ueProcess.m_status == HarqProcess::WAITING_FEEDBACK &&
            !ueProcess.m_dciElement->m_pdcchMissed && !ueProcess.m_ollaCounted &&
            (direction != "DL" || (ueProcess.m_dciElement->m_repetitions <= 1 &&
                                   ueProcess.m_dciElement->m_mcsPlanned == 255)))
        {
            ueProcess.m_ollaCounted = true;
            if (direction == "UL" && ueProcess.m_dciElement->m_sps != DciInfoElementTdma::DYNAMIC)
            {
                // a configured-grant block: its first transmissions failed in a row (CgFailures)
                const auto cg = m_spsUl.find(rnti);
                if (cg != m_spsUl.end())
                {
                    cg->second.failRun = harqFeedbackIt->IsReceivedOk()
                                             ? 0
                                             : static_cast<uint8_t>(std::min<uint32_t>(
                                                   cg->second.failRun + 1U, 255U));
                }
            }
            if (direction == "DL" && ueProcess.m_dciElement->m_sps != DciInfoElementTdma::DYNAMIC)
            {
                // a block of a DL SPS configuration: first transmissions failed in a row
                const auto sps = m_spsDl.find(rnti);
                if (sps != m_spsDl.end())
                {
                    sps->second.failRun = harqFeedbackIt->IsReceivedOk()
                                              ? 0
                                              : static_cast<uint8_t>(std::min<uint32_t>(
                                                    sps->second.failRun + 1U, 255U));
                }
            }
            if (direction == "DL")
            {
                UpdateDlOlla(rnti, harqFeedbackIt->IsReceivedOk());
            }
            else
            {
                UpdateUlOlla(rnti,
                             harqFeedbackIt->IsReceivedOk(),
                             ueProcess.m_dciElement->m_mcs,
                             m_ulMeasuredHarq.count({rnti, harqId}) > 0);
            }
        }
        if (m_spsScenario != 0)
        {
            SpsFeedback(rnti,
                        direction == "DL",
                        ueProcess,
                        harqFeedbackIt->IsReceivedOk(),
                        harqFeedbackIt->IsReceivedOk() ||
                            ueProcess.m_dciElement->m_rv == maxHarqReTx);
        }

        if (direction == "UL")
        if (m_diagUlHarq)
        {
            std::ostringstream o;
            o << "phf rnti " << rnti << " p" << +harqId << " "
              << (harqFeedbackIt->IsReceivedOk() ? "ACK" : "NACK") << " numRetx "
              << +harqFeedbackIt->m_numRetx << " dci rv " << +ueProcess.m_dciElement->m_rv
              << " reps " << +ueProcess.m_dciElement->m_repetitions << " status "
              << +ueProcess.m_status << " timer " << ueProcess.m_timer << " -> "
              << ((harqFeedbackIt->IsReceivedOk() || ueProcess.m_dciElement->m_rv == maxHarqReTx)
                      ? "erase"
                      : "retx pending");
            DiagLog(o.str());
        }
        if (harqFeedbackIt->IsReceivedOk() || ueProcess.m_dciElement->m_rv == maxHarqReTx)
        {
            ueHarqVector.Erase(harqId, direction == "DL" ? "ProcessHARQFeedbacks DL (ACK or last rv)" : "ProcessHARQFeedbacks UL (ACK or last rv)");
            harqFeedbackIt = harqInfo->erase(harqFeedbackIt);
            NS_LOG_INFO("Erased processID " << static_cast<uint32_t>(harqId) << " of UE " << rnti
                                            << " direction " << direction);
        }
        else if (!harqFeedbackIt->IsReceivedOk())
        {
            ueProcess.m_status = HarqProcess::RECEIVED_FEEDBACK;
            nackReceived++;
            ++harqFeedbackIt;
            NS_LOG_INFO("NACK received for UE " << static_cast<uint32_t>(rnti) << " process "
                                                << static_cast<uint32_t>(harqId) << " direction "
                                                << direction);
        }
    }

    NS_ASSERT(harqInfo->size() == nackReceived);
}

/**
 * @brief Reset expired HARQ
 * @param rnti RNTI of the user
 * @param harq HARQ process list
 *
 * For each process, check its timer. If it is expired, reset the
 * process.
 *
 * @see NrMacHarqVector
 * @see HarqProcess
 */
void
NrMacSchedulerNs3::ResetExpiredHARQ(uint16_t rnti, NrMacHarqVector* harq)
{
    NS_LOG_FUNCTION(this << harq);

    // K_offset is the scheduled part of the NTN wait and is always known; the timeout attribute
    // (or the process count when it is 0) stays the wait beyond it.
    const uint32_t koffsetSlots = m_macSchedSapUser->GetCellSpecificKoffsetSlots();
    const uint64_t nowSlot = static_cast<uint64_t>(
        Simulator::Now().GetNanoSeconds() / m_macSchedSapUser->GetSlotPeriod().GetNanoSeconds());
    for (auto harqIt = harq->Begin(); harqIt != harq->End(); ++harqIt)
    {
        HarqProcess& process = harqIt->second;
        uint8_t processId = harqIt->first;

        if (process.m_status == HarqProcess::INACTIVE)
        {
            continue;
        }

        // Its feedback already arrived and a retransmission is queued for this slot; reaping it
        // here would pull the process out from under ScheduleDlHarq, which asserts on the
        // RECEIVED_FEEDBACK status and, with assertions compiled out, walks a dead entry.
        // The timer exists for feedback that never came back, not for blocks about to be resent.
        if (process.m_status == HarqProcess::RECEIVED_FEEDBACK)
        {
            continue;
        }
        // PDSCH aggregation: a block whose transmissions are not all sent yet (they wait for
        // room) has no feedback to wait for; the clock starts at its last (PlaceDlRepetitions())
        if (!m_dlRepetitions.empty() && process.m_dciElement &&
            std::any_of(m_dlRepetitions.begin(),
                        m_dlRepetitions.end(),
                        [&process](const UlRepetition& r) { return r.dci == process.m_dciElement; }))
        {
            continue;
        }

        // A process is normally released by feedback. This timer only reclaims one whose
        // feedback never came back; tying it to the process count instead conflates "how many
        // blocks may be outstanding" with "how long to wait", so a window shorter than the loop
        // delay discards every outcome instead of stalling the UE. See HarqProcessTimeout.
        // (PUSCH repetition: the feedback comes after the last transmission of the block)
        const uint32_t harqLifetime =
            (m_harqProcessTimeout > 0 ? m_harqProcessTimeout
                                      : m_macSchedSapUser->GetNumHarqProcess()) +
            koffsetSlots +
            (process.m_dciElement ? 2U * (process.m_dciElement->m_repetitions - 1U) : 0U);

        // The block is received in its slot (a configured grant's is scheduled a period ahead,
        // every uplink one K_offset ahead) and its repetitions, and the feedback follows: the
        // wait counts from the last repetition, not from the scheduling, or the process is
        // reclaimed while its feedback is in flight and the next block on the same id gets it
        if (process.m_txSlot > 0)
        {
            const uint64_t reps = process.m_dciElement ? process.m_dciElement->m_repetitions : 1U;
            const uint64_t lastSlot = process.m_txSlot + (reps > 0 ? reps - 1 : 0);
            const uint64_t wait = (m_harqProcessTimeout > 0 ? m_harqProcessTimeout
                                                            : m_macSchedSapUser->GetNumHarqProcess()) +
                                  koffsetSlots;
            ++process.m_timer;
            if (nowSlot <= lastSlot + wait)
            {
                continue;
            }
            if (m_diagUlHarq)
            {
                std::ostringstream o;
                o << "expire rnti " << rnti << " p" << +processId << " txSlot " << process.m_txSlot
                  << " reps " << reps << " now " << nowSlot << " status " << +process.m_status
                  << (process.m_dciElement ? " rv " + std::to_string(process.m_dciElement->m_rv)
                                           : std::string());
                DiagLog(o.str());
            }
            harq->Erase(processId, "ResetExpiredHARQ (no feedback after the transmission)");
            continue;
        }

        if (process.m_timer < harqLifetime)
        {
            ++process.m_timer;
            NS_LOG_INFO("Updated process for UE " << rnti << " number "
                                                  << static_cast<uint32_t>(processId)
                                                  << ", resulting process: " << process);
        }
        else
        {
            if (m_diagUlHarq)
            {
                std::ostringstream o;
                o << "expire rnti " << rnti << " p" << +processId << " timer " << process.m_timer
                  << " lifetime " << harqLifetime << " status " << +process.m_status
                  << (process.m_dciElement
                          ? " rv " + std::to_string(process.m_dciElement->m_rv)
                          : std::string());
                DiagLog(o.str());
            }
            harq->Erase(processId, "ResetExpiredHARQ (timer)");
            NS_LOG_INFO("Erased process for UE " << rnti << " number "
                                                 << static_cast<uint32_t>(processId)
                                                 << " for time limits");
        }
    }
}

/**
 * @brief Prepend a CTRL symbol to the allocation list
 * @param symStart starting symbol
 * @param numSymToAllocate number of symbols to allocate (each CTRL take 1 symbol)
 * @param mode Mode of the allocation (UL, DL)
 * @param allocations list of allocations to which prepend the CTRL symbol
 * @return the symbol that can be used to append other things into the allocation list
 */
uint8_t
NrMacSchedulerNs3::PrependCtrlSym(uint8_t symStart,
                                  uint8_t numSymToAllocate,
                                  DciInfoElementTdma::DciFormat mode,
                                  std::deque<VarTtiAllocInfo>* allocations) const
{
    std::vector<bool> rbgBitmask(GetBandwidthInRbg(), true);

    NS_ASSERT_MSG(rbgBitmask.size() == GetBandwidthInRbg(),
                  "bitmask size " << rbgBitmask.size() << " conf " << GetBandwidthInRbg());
    if (mode == DciInfoElementTdma::DL)
    {
        NS_ASSERT(allocations->empty()); // no previous allocations
        NS_ASSERT(symStart == 0);        // start from the symbol 0
    }

    for (uint8_t sym = symStart; sym < symStart + numSymToAllocate; ++sym)
    {
        allocations->emplace_front(std::make_shared<DciInfoElementTdma>(sym,
                                                                        1,
                                                                        mode,
                                                                        DciInfoElementTdma::CTRL,
                                                                        rbgBitmask));
        NS_LOG_INFO("Allocating CTRL symbol, type"
                    << mode << " in TDMA. numSym=1, symStart=" << static_cast<uint32_t>(sym)
                    << " Remaining CTRL sym to allocate: " << sym - symStart);
    }
    return symStart + numSymToAllocate;
}

/**
 * @brief Append a CTRL symbol to the allocation list
 * @param symStart starting symbol
 * @param numSymToAllocate number of symbols to allocate (each CTRL take 1 symbol)
 * @param mode Mode of the allocation (UL, DL)
 * @param allocations list of allocations to which append the CTRL symbol
 * @return the VarTtiAllocInfo ID that can be used to append other things into the allocation list
 */
uint8_t
NrMacSchedulerNs3::AppendCtrlSym(uint8_t symStart,
                                 uint8_t numSymToAllocate,
                                 DciInfoElementTdma::DciFormat mode,
                                 std::deque<VarTtiAllocInfo>* allocations) const
{
    std::vector<bool> rbgBitmask(GetBandwidthInRbg(), true);

    NS_ASSERT(rbgBitmask.size() == GetBandwidthInRbg());
    if (mode == DciInfoElementTdma::DL)
    {
        NS_ASSERT(allocations->empty()); // no previous allocations
        NS_ASSERT(symStart == 0);        // start from the symbol 0
    }

    for (uint8_t sym = symStart; sym < symStart + numSymToAllocate; ++sym)
    {
        allocations->emplace_back(std::make_shared<DciInfoElementTdma>(sym,
                                                                       1,
                                                                       mode,
                                                                       DciInfoElementTdma::CTRL,
                                                                       rbgBitmask));
        NS_LOG_INFO("Allocating CTRL symbol, type"
                    << mode << " in TDMA. numSym=1, symStart=" << static_cast<uint32_t>(sym)
                    << " Remaining CTRL sym to allocate: " << sym - symStart);
    }
    return symStart + numSymToAllocate;
}

/**
 * @brief Compute the number of active DL HARQ to perform
 *
 * @param activeDlHarq list of DL HARQ to perform (should be empty at the beginning)
 * @param dlHarqFeedback list of DL HARQ feedback received
 *
 * After calculating the active HARQ, they should be sorted. It is done by
 * subclasses in the method SortDlHarq.
 * @see SortDlHarq
 */
void
NrMacSchedulerNs3::ComputeActiveHarq(ActiveHarqMap* activeDlHarq,
                                     const std::vector<DlHarqInfo>& dlHarqFeedback) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(activeDlHarq->empty());

    for (const auto& feedback : dlHarqFeedback)
    {
        uint16_t rnti = feedback.m_rnti;
        auto& schedInfo = m_ueMap.find(rnti)->second;
        auto beamIterator = activeDlHarq->find(schedInfo->m_beamId);

        if (beamIterator == activeDlHarq->end())
        {
            std::vector<NrMacHarqVector::iterator> harqVector;
            NS_ASSERT(schedInfo->m_dlHarq.Find(feedback.m_harqProcessId)->second.m_active);

            harqVector.emplace_back(schedInfo->m_dlHarq.Find(feedback.m_harqProcessId));
            activeDlHarq->emplace(std::make_pair(schedInfo->m_beamId, harqVector));
        }
        else
        {
            NS_ASSERT(schedInfo->m_dlHarq.Find(feedback.m_harqProcessId)->second.m_active);
            beamIterator->second.emplace_back(schedInfo->m_dlHarq.Find(feedback.m_harqProcessId));
        }
        NS_LOG_INFO("Received feedback for UE " << rnti << " ID "
                                                << static_cast<uint32_t>(feedback.m_harqProcessId)
                                                << " marked as active");
        NS_ASSERT(schedInfo->m_dlHarq.Find(feedback.m_harqProcessId)->second.m_status ==
                  HarqProcess::RECEIVED_FEEDBACK);
        if (m_nrFhSchedSapProvider)
        {
            m_nrFhSchedSapProvider->SetActiveHarqUes(GetBwpId(), rnti);
        }
    }

    SortDlHarq(activeDlHarq);
}

/**
 * @brief Compute the number of activeUL HARQ to perform
 *
 * @param activeUlHarq list of UL HARQ to perform
 * @param ulHarqFeedback list of UL HARQ feedback
 *
 * After calculating the active HARQ, they should be sorted. It is done by
 * subclasses in the method SortUlHarq.
 * @see SortUlHarq
 */
void
NrMacSchedulerNs3::ComputeActiveHarq(ActiveHarqMap* activeUlHarq,
                                     const std::vector<UlHarqInfo>& ulHarqFeedback) const
{
    NS_LOG_FUNCTION(this);

    for (const auto& feedback : ulHarqFeedback)
    {
        uint16_t rnti = feedback.m_rnti;
        // (DiagCheckUlFeedback() has dropped what has no pending process)
        auto& schedInfo = m_ueMap.find(rnti)->second;
        auto beamIterator = activeUlHarq->find(schedInfo->m_beamId);

        if (beamIterator == activeUlHarq->end())
        {
            std::vector<NrMacHarqVector::iterator> harqVector;
            NS_ASSERT(schedInfo->m_ulHarq.Find(feedback.m_harqProcessId)->second.m_active);
            harqVector.emplace_back(schedInfo->m_ulHarq.Find(feedback.m_harqProcessId));
            activeUlHarq->emplace(std::make_pair(schedInfo->m_beamId, harqVector));
        }
        else
        {
            NS_ASSERT(schedInfo->m_ulHarq.Find(feedback.m_harqProcessId)->second.m_active);
            beamIterator->second.emplace_back(schedInfo->m_ulHarq.Find(feedback.m_harqProcessId));
        }
    }
    SortUlHarq(activeUlHarq);
}

/**
 * @brief Compute the number of active DL and UL UE
 * @param activeDlUe map of active DL UE to be filled
 * @param GetLCGFn Function to retrieve the LCG of a UE
 * @param mode UL or DL (to be printed in debug messages)
 *
 * The function loops all available UEs and checks their LC. If one (or more)
 * LC contains bytes, they are marked active and inserted in one of the
 * list passed as input parameters. Every UE is marked as active if it has
 * data to transmit; it is a duty for someone else to not assign two DCI for
 * the same RNTI.
 */
void
NrMacSchedulerNs3::ComputeActiveUe(ActiveUeMap* activeUe,
                                   const NrMacSchedulerUeInfo::GetLCGFn& GetLCGFn,
                                   const NrMacSchedulerUeInfo::GetHarqVectorFn& GetHarqVector,
                                   const std::string& mode) const
{
    NS_LOG_FUNCTION(this);
    const bool isDl = mode == "DL";
    for (const auto& ueInfo : m_ueMap)
    {
        uint32_t totBuffer = 0;
        const auto& ue = ueInfo.second;

        // compute total DL and UL bytes buffered
        for (const auto& lcgInfo : GetLCGFn(ue))
        {
            const auto& lcg = lcgInfo.second;
            if (lcg->GetTotalSize() > 0)
            {
                NS_LOG_INFO("UE " << ue->m_rnti << " " << mode << " LCG "
                                  << static_cast<uint32_t>(lcgInfo.first) << " bytes "
                                  << lcg->GetTotalSize());
            }
            for (auto lcId : lcg->GetLCId())
            {
                totBuffer += lcg->GetTotalSizeOfLCPlusOverheads(lcId, isDl);
            }
        }

        // A UE that sent a scheduling request is eligible for uplink scheduling even though no
        // logical channel of it is known to hold data: the bootstrap grant it is owed is what
        // lets it report its buffer status (3GPP TS 38.321, clauses 5.4.4 and 5.4.5).
        if (!isDl)
        {
            totBuffer += ue->m_srBytesPending;
        }

        const auto& harqV = GetHarqVector(ue);

        if (totBuffer > 0 && harqV.CanInsert())
        {
            auto it = activeUe->find(ue->m_beamId);
            if (it == activeUe->end())
            {
                std::vector<std::pair<std::shared_ptr<NrMacSchedulerUeInfo>, uint32_t>> tmp;
                tmp.emplace_back(ue, totBuffer);
                activeUe->insert(std::make_pair(ue->m_beamId, tmp));
            }
            else
            {
                it->second.emplace_back(ue, totBuffer);
            }
        }
    }

    // The UE served longest ago first, and by RNTI among those of the same slot. m_ueMap has no
    // order of its own: taken as they come, the same UEs would be first in every slot, and with
    // more UEs waiting than a slot can serve the last ones would never get a turn. The policy
    // of the scheduler sorts this list again (a stable sort) and keeps this order between UEs
    // it ranks the same: round robin ranks all the same when a slot starts, so it serves them
    // in turn from slot to slot.
    const auto& last = isDl ? m_dlPdcchLast : m_ulPdcchLast;
    auto servedAt = [&last](const UePtrAndBufferReq& ue) {
        const auto it = last.find(ue.first->m_rnti);
        return std::make_pair(it == last.end() ? uint64_t{0} : it->second, ue.first->m_rnti);
    };
    for (auto& beam : *activeUe)
    {
        std::sort(beam.second.begin(),
                  beam.second.end(),
                  [&servedAt](const UePtrAndBufferReq& a, const UePtrAndBufferReq& b) {
                      return servedAt(a) < servedAt(b);
                  });
    }
}

void
NrMacSchedulerNs3::SortByPolicy([[maybe_unused]] std::vector<UePtrAndBufferReq>* ues,
                                [[maybe_unused]] bool isDl) const
{
    // a scheduler without a policy of its own leaves the UEs as they are: served longest ago first
}

void
NrMacSchedulerNs3::PassedOver([[maybe_unused]] const UePtrAndBufferReq& ue,
                              [[maybe_unused]] bool isDl) const
{
}

void
NrMacSchedulerNs3::NoteServed(const SlotAllocInfo& allocInfo, bool isDl)
{
    auto& last = isDl ? m_dlPdcchLast : m_ulPdcchLast;
    const auto format = isDl ? DciInfoElementTdma::DL : DciInfoElementTdma::UL;
    for (const auto& alloc : allocInfo.m_varTtiAllocInfo)
    {
        if (alloc.m_dci->m_format == format && alloc.m_dci->m_type == DciInfoElementTdma::DATA)
        {
            last[alloc.m_dci->m_rnti] = m_pdcchRound;
        }
    }
}

/**
 * @brief Scheduling new DL data
 * @param spoint Starting point of the blocks to add to the allocation list
 * @param symAvail Number of available symbols
 * @param activeDl List of active UE with data to transmit in DL
 * @param slotAlloc The allocation info to which append the allocations
 * @return The number of symbols used in the allocation
 *
 * The method is doing the scheduling of new data in the DL direction, delegating
 * three phases to subclasses:
 *
 * - How distribute the symbols between beams?
 * - How many RBG should be assigned to the each active UE?
 * - How to place the blocks in the 2D plan (in other words, how to create the DCIs)?
 *
 * The first two phases are managed by the function AssignDLRBG. Once the map
 * between the beamId and the symbols assigned to it has been returned, the
 * active user list has been updated by assigning to each user an amount of
 * RBG. Then, it is necessary to iterate through the beams, and for each beam,
 * iterating through the users of that beam, creating a DCI (function CreateDlDci).
 * Creating the DCI is the third phase in the previous list, because the DCI
 * specifies where the imaginary block containing data is placed.
 *
 * After the DCI has been created, it is necessary to prepare the HarqProcess,
 * store it in the UE pointer, and distribute the TBS among the active LC of the
 * UE, through the method AssignBytesToLC, and creating the corresponding list
 * of RlcPduInfo.
 *
 * Before looping and changing the beam, the starting point should be advanced.
 * How that is done is a matter for the subclasses (method ChangeDlBeam).
 */
uint8_t
NrMacSchedulerNs3::DoScheduleDlData(PointInFTPlane* spoint,
                                    uint32_t symAvail,
                                    const ActiveUeMap& activeDl,
                                    SlotAllocInfo* slotAlloc) const
{
    NS_LOG_FUNCTION(this << symAvail);
    NS_ASSERT(spoint->m_rbg == 0);
    BeamSymbolMap symPerBeam = AssignDLRBG(symAvail, activeDl);
    GetFirst GetBeam;
    uint8_t usedSym = 0;

    for (const auto& beam : activeDl)
    {
        uint32_t availableRBG =
            (GetBandwidthInRbg() - spoint->m_rbg) * symPerBeam.at(GetBeam(beam));
        bool assigned = false;
        std::unordered_set<uint8_t> symbStartDci;
        // allocSym is used to count the number of allocated symbols to the UEs of the beam
        // we are iterating over
        uint32_t allocSym = 0;

        NS_LOG_DEBUG(activeDl.size()
                     << " active DL beam, this beam has " << symPerBeam.at(GetBeam(beam))
                     << " SYM, starts from RB " << spoint->m_rbg << " and symbol "
                     << static_cast<uint32_t>(spoint->m_sym) << " for a total of " << availableRBG
                     << " RBG. In one symbol we have " << GetBandwidthInRbg() << " RBG.");

        if (symPerBeam.at(GetBeam(beam)) == 0)
        {
            NS_LOG_INFO("No available symbols for this beam, continue");
            continue;
        }

        for (const auto& ue : beam.second)
        {
            if (ue.first->m_dlRBG.empty())
            {
                NS_LOG_INFO("UE " << ue.first->m_rnti << " does not have RBG assigned");
                continue;
            }

            std::shared_ptr<DciInfoElementTdma> dci =
                CreateDlDci(spoint, ue.first, symPerBeam.at(GetBeam(beam)));
            if (dci == nullptr)
            {
                // By continuing to the next UE means that we are
                // wasting a resource assign to this UE. For a TDMA
                // scheduler this resource would be one or more
                // symbols, and for OFDMA scheduler it would be a
                // chunk of time + freq, i.e., one or more
                // symbols in time and one or more RBG in freq.
                // TODO To avoid this, a more accurate solution
                // is needed to assign resources. That is, a solution
                // that would not assign resources to a UE if the assigned resources
                // result a TB size of less than 7 bytes (3 mac header, 2 rlc header, 2 data).
                // Because if this happens CreateDlDci will not create DCI.
                NS_LOG_DEBUG("No DCI has been created, ignoring");
                ue.first->ResetDlMetric();
                continue;
            }

            assigned = true;

            if (symbStartDci.insert(dci->m_symStart).second)
            {
                allocSym += dci->m_numSym;
            }

            NS_LOG_INFO("UE " << ue.first->m_rnti << " has " << ue.first->m_dlRBG.size()
                              << " RBG assigned");
            NS_ASSERT_MSG(dci->m_symStart + dci->m_numSym <= m_macSchedSapUser->GetSymbolsPerSlot(),
                          "symStart: "
                              << static_cast<uint32_t>(dci->m_symStart)
                              << " symEnd: " << static_cast<uint32_t>(dci->m_numSym) << " symbols: "
                              << static_cast<uint32_t>(m_macSchedSapUser->GetSymbolsPerSlot()));

            HarqProcess harqProcess(true, HarqProcess::WAITING_FEEDBACK, 0, dci);
            harqProcess.m_txSlot = slotAlloc->m_sfnSf.Normalize();
            uint8_t id;

            if (!ue.first->m_dlHarq.CanInsert())
            {
                NS_LOG_INFO("Harq Vector condition for UE " << ue.first->m_rnti << std::endl
                                                            << ue.first->m_dlHarq);
                NS_FATAL_ERROR("UE " << ue.first->m_rnti << " does not have DL HARQ space");
            }

            ue.first->m_dlHarq.Insert(&id, harqProcess);
            ue.first->m_dlHarq.Get(id).m_dciElement->m_harqProcess = id;

            // distribute tbsize among the LCs of the UE
            // distributedBytes size is equal to the number of LCs
            auto distributedBytes =
                m_schedLc->AssignBytesToDlLC(ue.first->m_dlLCG,
                                             dci->m_tbSize,
                                             m_macSchedSapUser->GetSlotPeriod());

            std::vector<std::vector<NrMacSchedulerLcAlgorithm::Assignation>> bytesPerLc(
                distributedBytes.size());

            for (std::size_t numLc = 0; numLc < distributedBytes.size(); numLc++)
            {
                // For RLC (because of RLC AM), we assume at least 7 bytes of TX opportunity,
                // and subtract MAC_SUBHEADER_SIZE (3 bytes) from the assigned m_bytes.
                if (distributedBytes.at(numLc).m_bytes < 7 + MAC_SUBHEADER_SIZE)
                {
                    NS_LOG_WARN("TX opportunity is too small: "
                                << distributedBytes.at(numLc).m_bytes << " bytes.");
                }
                bytesPerLc.at(numLc).emplace_back(distributedBytes.at(numLc).m_lcg,
                                                  distributedBytes.at(numLc).m_lcId,
                                                  distributedBytes.at(numLc).m_bytes);
            }

            VarTtiAllocInfo slotInfo(dci);

            NS_LOG_INFO("Assigned process ID " << static_cast<uint32_t>(dci->m_harqProcess)
                                               << " to UE " << ue.first->m_rnti);
            NS_LOG_DEBUG(" UE" << dci->m_rnti << " gets DL symbols "
                               << static_cast<uint32_t>(dci->m_symStart) << "-"
                               << static_cast<uint32_t>(dci->m_symStart + dci->m_numSym) << " tbs "
                               << dci->m_tbSize << " mcs " << static_cast<uint32_t>(dci->m_mcs)
                               << " harqId " << static_cast<uint32_t>(id) << " rv "
                               << static_cast<uint32_t>(dci->m_rv));

            for (const auto& byteDistribution : distributedBytes)
            {
                NS_ASSERT(byteDistribution.m_bytes >= MAC_SUBHEADER_SIZE);
                uint8_t lcId = byteDistribution.m_lcId;
                uint8_t lcgId = byteDistribution.m_lcg;
                // Consider the subPdu overhead
                uint32_t bytes = MacSduIn(byteDistribution.m_bytes);

                RlcPduInfo newRlcPdu(lcId, bytes);
                HarqProcess& process = ue.first->m_dlHarq.Get(dci->m_harqProcess);

                slotInfo.m_rlcPduInfo.push_back(newRlcPdu);
                process.m_rlcPduInfo.push_back(newRlcPdu);

                ue.first->m_dlLCG.at(lcgId)->AssignedData(lcId, bytes, "DL");

                NS_LOG_DEBUG("DL LCG " << static_cast<uint32_t>(lcgId) << " LCID "
                                       << static_cast<uint32_t>(lcId) << " got bytes "
                                       << newRlcPdu.m_size);
            }

            NS_ABORT_IF(slotInfo.m_rlcPduInfo.empty());

            slotAlloc->m_varTtiAllocInfo.emplace_back(slotInfo);

            // Rel-17 NTN: downlinkHARQ-FeedbackDisabled. With feedback disabled the process is
            // released at transmission instead of when the ACK/NACK returns, so the pipeline is
            // no longer throttled to one block per process per loop round trip -- which is the
            // point of the feature. Setting the retransmission limit to zero does not do this:
            // the process still occupies its slot for a full round trip waiting for a report it
            // will ignore. Reliability then has to come from RLC AM above.
            if (m_dlHarqFeedbackDisabled)
            {
                ue.first->m_dlHarq.Erase(id, "DoScheduleDlData (feedback disabled)");
            }
        }
        if (assigned)
        {
            ChangeDlBeam(spoint, symPerBeam.at(GetBeam(beam)));
            usedSym += allocSym;
            slotAlloc->m_numSymAlloc += allocSym;
            NS_ASSERT_MSG(slotAlloc->m_numSymAlloc <= 14,
                          "Invalid number of symbols: " << slotAlloc->m_numSymAlloc
                                                        << " symbols. ");
        }
    }

    for (auto& beam : activeDl)
    {
        for (auto& ue : beam.second)
        {
            ue.first->ResetDlSchedInfo();
        }
    }

    NS_ASSERT(spoint->m_rbg == 0);

    return usedSym;
}

/**
 * @brief Scheduling new UL data
 * @param spoint Starting point of the blocks to add to the allocation list
 * @param symAvail Number of available symbols
 * @param activeDl List of active UE with data to transmit in UL
 * @param slotAlloc The allocation info to which append the allocations
 * @return The number of symbols used in the allocation
 *
 * The method is doing the scheduling of new data in the UL direction. Before
 * doing that, it is necessary to schedule the UEs that requested a SR.
 * Then, to decide how to schedule the data, it delegates three phases to subclasses:
 *
 * - How distribute the symbols between beams?
 * - How many RBG should be assigned to the each active UE?
 * - How to place the blocks in the 2D plan (in other words, how to create the DCIs)?
 *
 * The first two phases are managed by the function AssignULRBG. Once the map
 * between the beamId and the symbols assigned to it has been returned, the
 * active user list has been updated by assigning to each user an amount of
 * RBG. Then, it is necessary to iterate through the beams, and for each beam,
 * iterating through the users of that beam, creating a DCI (function CreateUlDci).
 * Creating the DCI is the third phase in the previous list, because the DCI
 * specifies where the imaginary block containing data is placed. The DCI should
 * be created by placing the blocks from the last available symbol (e.g., 13)
 * going backwards. This restriction comes from the need of keeping the order
 * of DCI: DL CTRL, DL Data, UL Data, UL CTRL.
 *
 * After the DCI has been created, it is necessary to prepare the HarqProcess,
 * store it in the UE pointer, and distribute the TBS among the active LC of the
 * UE, through the method AssignBytesToLC, and creating the corresponding list
 * of RlcPduInfo.
 *
 * Before looping and changing the beam, the starting point should be advanced.
 * How that is done is a matter for the subclasses (method ChangeUlBeam).
 */
uint8_t
NrMacSchedulerNs3::DoScheduleUlData(PointInFTPlane* spoint,
                                    uint32_t symAvail,
                                    const ActiveUeMap& activeUl,
                                    SlotAllocInfo* slotAlloc) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(symAvail > 0 && !activeUl.empty());
    NS_ASSERT(spoint->m_rbg == 0);

    BeamSymbolMap symPerBeam = AssignULRBG(symAvail, activeUl);
    uint8_t usedSym = 0;
    GetFirst GetBeam;

    for (const auto& beam : activeUl)
    {
        uint32_t availableRBG =
            (GetBandwidthInRbg() - spoint->m_rbg) * symPerBeam.at(GetBeam(beam));
        bool assigned = false;
        std::unordered_set<uint8_t> symbStartDci;
        // allocSym is used to count the number of allocated symbols to the UEs of the beam
        // we are iterating over
        uint32_t allocSym = 0;

        NS_LOG_DEBUG(activeUl.size()
                     << " active UL beam, this beam has " << symPerBeam.at(GetBeam(beam))
                     << " SYM, starts from RBG " << spoint->m_rbg << " and symbol "
                     << static_cast<uint32_t>(spoint->m_sym) << " (going backward) for a total of "
                     << availableRBG << " RBG. In one symbol we have " << GetBandwidthInRbg()
                     << " RBG.");

        if (symPerBeam.at(GetBeam(beam)) == 0)
        {
            NS_LOG_INFO("No available symbols for this beam, continue");
            continue;
        }

        for (const auto& ue : beam.second)
        {
            if (ue.first->m_ulRBG.empty())
            {
                NS_LOG_INFO("UE " << ue.first->m_rnti << " does not have RBG assigned");
                continue;
            }

            std::shared_ptr<DciInfoElementTdma> dci =
                CreateUlDci(spoint, ue.first, symPerBeam.at(GetBeam(beam)));

            if (dci == nullptr)
            {
                // By continuing to the next UE means that we are
                // wasting a resource assign to this UE. For a TDMA
                // scheduler this resource would be one or more
                // symbols, and for OFDMA scheduler it would be a
                // chunk of time + freq, i.e., one or more
                // symbols in time and one or more RBG in freq.
                // TODO To avoid this, a more accurate solution
                // is needed to assign resources. That is, a solution
                // that would not assign resources to a UE if the assigned resources
                // result a TB size of less than 7 bytes (3 mac header, 2 rlc header, 2
                // data). Because if this happens CreateUlDci will not create DCI.
                NS_LOG_DEBUG("No DCI has been created, ignoring");
                ue.first->ResetUlMetric();
                continue;
            }

            auto distributedBytes = m_schedLc->AssignBytesToUlLC(ue.first->m_ulLCG, dci->m_tbSize);

            // A UE that only answered a scheduling request has no known buffer to spread over
            // its logical channels, so there is nothing to distribute: the grant still has to be
            // issued, as it is what carries the buffer status report back (3GPP TS 38.321,
            // clause 5.4.4). Which logical channels fill it is decided by the UE.
            const bool bootstrapGrant = distributedBytes.empty() && ue.first->m_srBytesPending > 0;

            if (distributedBytes.empty() && !bootstrapGrant)
            {
                NS_LOG_DEBUG("Not enough bytes assigned to UL LCs. Skipping UE.");
                ue.first->ResetUlMetric();
                continue;
            }

            // The request has been answered by this grant.
            ue.first->m_srBytesPending = 0;
            if (ue.first->m_srConfirmAhead > 0)
            {
                // CgBlockPacketOnly: the configuration is activated with a DCI sent before this
                // grant's PUSCH, which carries its confirmation (TS 38.321 5.8.2)
                dci->m_cgConfirmBytes = ue.first->m_srConfirmAhead;
                dci->m_cgConfirmAhead = true;
                dci->m_cgSerial = ue.first->m_srConfirmAheadSerial;
                ue.first->m_srConfirmAhead = 0;
            }

            assigned = true;

            if (symbStartDci.insert(dci->m_symStart).second)
            {
                allocSym += dci->m_numSym;
            }

            if (!ue.first->m_ulHarq.CanInsert())
            {
                NS_LOG_INFO("Harq Vector condition for UE " << ue.first->m_rnti << std::endl
                                                            << ue.first->m_ulHarq);
                NS_FATAL_ERROR("UE " << ue.first->m_rnti << " does not have UL HARQ space");
            }

            HarqProcess harqProcess(true, HarqProcess::WAITING_FEEDBACK, 0, dci);
            harqProcess.m_txSlot = slotAlloc->m_sfnSf.Normalize();
            uint8_t id;
            ue.first->m_ulHarq.Insert(&id, harqProcess);

            ue.first->m_ulHarq.Get(id).m_dciElement->m_harqProcess = id;
            if (m_diagUlHarq)
            {
                std::ostringstream o;
                o << "new block rnti " << ue.first->m_rnti << " p" << +id << " reps "
                  << +dci->m_repetitions << " rbgs "
                  << std::count(dci->m_rbgBitmask.begin(), dci->m_rbgBitmask.end(), true);
                DiagLog(o.str());
            }
            if (ue.first->m_ulSinrMeas.empty())
            {
                m_ulMeasuredHarq.erase({ue.first->m_rnti, id});
            }
            else
            {
                m_ulMeasuredHarq.insert({ue.first->m_rnti, id});
            }

            VarTtiAllocInfo slotInfo(dci);

            NS_LOG_INFO("Assigned process ID " << static_cast<uint32_t>(dci->m_harqProcess)
                                               << " to UE " << ue.first->m_rnti);
            NS_LOG_DEBUG(" UE" << dci->m_rnti << " gets UL symbols "
                               << static_cast<uint32_t>(dci->m_symStart) << "-"
                               << static_cast<uint32_t>(dci->m_symStart + dci->m_numSym) << " tbs "
                               << dci->m_tbSize << " mcs " << static_cast<uint32_t>(dci->m_mcs)
                               << " harqId " << static_cast<uint32_t>(id) << " rv "
                               << static_cast<uint32_t>(dci->m_rv));

            bool assignedToLC = false;
            for (const auto& byteDistribution : distributedBytes)
            {
                assignedToLC = true;
                ue.first->m_ulLCG.at(byteDistribution.m_lcg)
                    ->AssignedData(byteDistribution.m_lcId, byteDistribution.m_bytes, "UL");
                NS_LOG_DEBUG("UL LCG " << static_cast<uint32_t>(byteDistribution.m_lcg)
                                       << " assigned bytes " << byteDistribution.m_bytes
                                       << " to LCID "
                                       << static_cast<uint32_t>(byteDistribution.m_lcId));
            }
            NS_ASSERT(assignedToLC || bootstrapGrant);
            slotAlloc->m_varTtiAllocInfo.emplace_front(slotInfo);
        }
        if (assigned)
        {
            ChangeUlBeam(spoint, symPerBeam.at(GetBeam(beam)));
            usedSym += allocSym;
            slotAlloc->m_numSymAlloc += allocSym;
            NS_ASSERT_MSG(slotAlloc->m_numSymAlloc <= 14,
                          "Invalid number of symbols: " << slotAlloc->m_numSymAlloc
                                                        << " symbols. ");
        }
    }

    for (auto& beam : activeUl)
    {
        for (auto& ue : beam.second)
        {
            ue.first->ResetUlSchedInfo();
        }
    }

    NS_ASSERT(spoint->m_rbg == 0);

    return usedSym;
}

/**
 * @brief Schedule received SR
 * @param spoint Starting point for allocation
 * @param rntiList list of RNTI which asked for a SR
 *
 * Each time an UE asks for SR, the scheduler owes it a small bootstrap grant,
 * recorded in NrMacSchedulerUeInfo::m_srBytesPending. Then, the routine for
 * scheduling the data will take care to create an assignation for the UE, to
 * be able to send some data and, eventually, a BSR.
 *
 * The request is not recorded as buffer of any logical channel group: it carries
 * neither a size nor a group (3GPP TS 38.321, clause 5.4.4), and only a buffer
 * status report tells the scheduler what the UE has and where (clause 5.4.5).
 *
 */
void
NrMacSchedulerNs3::DoScheduleUlSr(PointInFTPlane* spoint, const std::list<uint16_t>& rntiList) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(spoint->m_rbg == 0);

    // 12 bytes of payload, plus the RLC header and the MAC subheader the UE prepends to them:
    // the request must buy a transport block that actually fits a buffer status report and a
    // minimal RLC PDU, not just their payload.
    const uint32_t srGrantSize = m_srGrantBytes;

    for (const auto& v : rntiList)
    {
        // A scheduling request can race against the UE context lifetime during
        // a handover: a UE that re-tuned to (or away from) this cell may have an
        // SR in flight that reaches the scheduler before its context is added or
        // after it is removed. Such an SR has no UE info here, so skip it
        // instead of dereferencing a missing map entry (which would abort).
        auto ueIt = m_ueMap.find(v);
        if (ueIt == m_ueMap.end())
        {
            NS_LOG_WARN("Ignoring scheduling request from unknown RNTI "
                        << v << " (no UE context on this cell, likely a handover race)");
            continue;
        }
        NS_LOG_DEBUG("Owing a " << srGrantSize << " byte bootstrap grant to UE " << v
                                << " because of a SR");
        ueIt->second->m_srBytesPending = m_srGrantLearned ? LearnedSrGrant(v) : srGrantSize;
        NS_LOG_LOGIC("SR RNTI " << v << " grant " << ueIt->second->m_srBytesPending << " B");
    }
}

/**
 * @brief Do the process of scheduling for the DL
 * @param params scheduling parameters
 * @param dlHarqFeedback vector of DL HARQ negative feedback
 *
 * The scheduling process is composed by the UL and the DL parts, and it
 * decides how the resources should be divided between UEs. An important
 * thing to remember is that the slot being considered for DL decision can be
 * different for the slot for UL decision. This offset is due to the parameter
 * N2Delay (previously: UlSchedDelay).
 *
 * Another parameter to consider is the L1L2CtrlLatency that defines the delay (in slots
 * number) between the slot that is currently "in the air" and the slot which is being
 * prepared for DL. The default value for both L1L2CtrlLatency and N2Delay (previously:
 * UlSchedDelay) is 2, so it means that while the slot number (frame, subframe, slot) is in
 * the air, the scheduler in this function will take decisions for DL in slot number (frame,
 * subframe, slot) + 2 and for UL in slot number (frame, subframe, slot) + 4.
 *
 * The consequences are an additional complexity derived from the fact that the
 * DL scheduling for a slot should remember the previous UL scheduling done in the
 * past for that slot.
 *
 * The process of scheduling DL data is defined as follows:
 *
 * - Retrieve the allocation done in the past (UL) for this slot
 * - Prepend DL CTRL symbol to the allocation list;
 * - Compute the list of active HARQ/new data UE
 * - Perform a scheduling for DL HARQ/data for this (DoScheduleDl());
 * - Indicate to the MAC the decision for that slot through SchedConfigInd()
 *
 * To know how the scheduling for DL is performed, take a look to the
 * function documentation for DoScheduleDl().
 *
 * At the end, the return of DoScheduleDl is passed to MAC through the function
 * SchedConfigInd().
 *
 * @see PointInFTPlane
 */
void
NrMacSchedulerNs3::ScheduleDl(const NrMacSchedSapProvider::SchedDlTriggerReqParameters& params,
                              const std::vector<DlHarqInfo>& dlHarqFeedback)
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Scheduling invoked for slot " << params.m_snfSf << " of type "
                                               << params.m_slotType);
    ++m_pdcchRound;

    NrMacSchedSapUser::SchedConfigIndParameters dlSlot(params.m_snfSf);
    dlSlot.m_slotAllocInfo.m_sfnSf = params.m_snfSf;
    dlSlot.m_slotAllocInfo.m_type = SlotAllocInfo::DL;
    auto ulAllocationIt =
        m_ulAllocationMap.find(params.m_snfSf.GetEncoding()); // UL allocations for this slot
    if (ulAllocationIt == m_ulAllocationMap.end())
    {
        ulAllocationIt =
            m_ulAllocationMap.insert(std::make_pair(params.m_snfSf.GetEncoding(), SlotElem(0)))
                .first;
    }
    auto& ulAllocations = ulAllocationIt->second;

    const bool pdcchOn = PdcchOn();
    const bool spsOn = pdcchOn && m_spsScenario != 0 && m_spsDlOn;
    m_dlSpsSymsSlot = 0;
    if (spsOn)
    {
        // Semi-persistent scheduling: what the voice bearers get in this slot. Those that wait
        // for their occasion are out of sight of everything that follows.
        SpsPlanDl(params.m_snfSf.Normalize());
    }

    // One PDSCH per UE and slot (the UE receives one transport block in a slot): a UE whose
    // block has a transmission still to come (PDSCH aggregation, PlaceDlRepetitions()), or that
    // has a block of its SPS configuration in this slot, gets no other PDSCH and no DCI; its
    // retransmissions due wait for the next slot. (Before 2 Nov 2026 an SPS block and a HARQ
    // retransmission could go to one UE in one slot: the UE took one of them, the other block
    // was lost, about 3 % of the downlink voice with --scheduling=sps1. The uplink already had
    // this rule for configured grants.)
    std::vector<DlHarqInfo> feedback = dlHarqFeedback;
    std::set<uint16_t> repeating = DlRepeatingUes();
    if (spsOn)
    {
        for (const auto& block : m_spsBlocksSlot)
        {
            repeating.insert(block.rnti);
        }
    }
    if (!repeating.empty())
    {
        std::erase_if(feedback, [&](const DlHarqInfo& f) {
            const auto ue = m_ueMap.find(f.m_rnti);
            if (!repeating.contains(f.m_rnti) || ue == m_ueMap.end() ||
                !ue->second->m_dlHarq.Exist(f.m_harqProcessId) ||
                ue->second->m_dlHarq.Get(f.m_harqProcessId).m_status !=
                    HarqProcess::RECEIVED_FEEDBACK)
            {
                return false;
            }
            auto& process = ue->second->m_dlHarq.Find(f.m_harqProcessId)->second;
            process.m_status = HarqProcess::WAITING_FEEDBACK;
            process.m_timer = 0;
            m_dlHarqToRetransmit.push_back(f);
            return true;
        });
    }

    // compute active ue in the current subframe, group them by BeamId
    ActiveHarqMap activeDlHarq;
    ComputeActiveHarq(&activeDlHarq, feedback);

    ActiveUeMap activeDlUe;
    ComputeActiveUe(&activeDlUe,
                    &NrMacSchedulerUeInfo::GetDlLCG,
                    &NrMacSchedulerUeInfo::GetDlHarqVector,
                    "DL");
    if (!repeating.empty())
    {
        for (auto it = activeDlUe.begin(); it != activeDlUe.end();)
        {
            std::erase_if(it->second, [&repeating](const UePtrAndBufferReq& ue) {
                return repeating.contains(ue.first->m_rnti);
            });
            it = it->second.empty() ? activeDlUe.erase(it) : std::next(it);
        }
    }

    // With the PDCCH model on, the PDCCH comes first: a UE needs a candidate with free CCEs
    // to be scheduled, and the DCIs of the slot say how many symbols the PDCCH takes
    m_dlCtrlSymsSlot = m_dlCtrlSymbols;
    if (pdcchOn)
    {
        const uint32_t symbols = m_macSchedSapUser->GetSymbolsPerSlot();
        const uint32_t taken = m_pdcch->GetMinSymbols() + ulAllocations.m_totUlSym;
        AdmitDlToPdcch(activeDlHarq,
                       &activeDlUe,
                       GetMaxUesPerSlot(symbols > taken ? symbols - taken : 0));
        m_dlCtrlSymsSlot = m_pdcch->FixSymbols();
    }

    // add slot for DL control, at symbol 0
    PrependCtrlSym(0,
                   m_dlCtrlSymsSlot,
                   DciInfoElementTdma::DL,
                   &dlSlot.m_slotAllocInfo.m_varTtiAllocInfo);
    dlSlot.m_slotAllocInfo.m_numSymAlloc += m_dlCtrlSymsSlot;
    NS_ASSERT_MSG(dlSlot.m_slotAllocInfo.m_numSymAlloc <= 14,
                  "Invalid number of symbols: " << dlSlot.m_slotAllocInfo.m_numSymAlloc
                                                << " symbols. ");

    // In case of S slot, add UL CTRL and update the symbol used count
    if (params.m_slotType == LteNrTddSlotType::S)
    {
        NS_LOG_INFO("S slot, adding UL CTRL");
        AppendCtrlSym(static_cast<uint8_t>(m_macSchedSapUser->GetSymbolsPerSlot() - 1),
                      m_ulCtrlSymbols,
                      DciInfoElementTdma::UL,
                      &dlSlot.m_slotAllocInfo.m_varTtiAllocInfo);
        ulAllocations.m_totUlSym += m_ulCtrlSymbols;
        dlSlot.m_slotAllocInfo.m_numSymAlloc += m_ulCtrlSymbols;
        NS_ASSERT_MSG(dlSlot.m_slotAllocInfo.m_numSymAlloc <= 14,
                      "Invalid number of symbols: " << dlSlot.m_slotAllocInfo.m_numSymAlloc
                                                    << " symbols. ");
    }

    if (spsOn)
    {
        // The blocks of the semi-persistent configurations come right after the PDCCH
        const uint32_t symbols = m_macSchedSapUser->GetSymbolsPerSlot();
        const uint32_t taken = m_dlCtrlSymsSlot + ulAllocations.m_totUlSym;
        m_dlSpsSymsSlot = SpsAllocate(params.m_snfSf.Normalize(),
                                      params.m_snfSf.GetEncoding(),
                                      true,
                                      m_dlCtrlSymsSlot,
                                      symbols > taken ? symbols - taken : 0,
                                      &dlSlot.m_slotAllocInfo);
    }

    DoScheduleDl(feedback,
                 activeDlHarq,
                 &activeDlUe,
                 params.m_snfSf,
                 ulAllocations,
                 &dlSlot.m_slotAllocInfo);
    if (m_pdschAggregation != 1 || !m_dlRepetitions.empty())
    {
        QueueDlRepetitions(dlSlot.m_slotAllocInfo);
    }

    if (spsOn)
    {
        SpsUnhide();
    }
    if (pdcchOn)
    {
        SettlePdcch(&dlSlot.m_slotAllocInfo, true);
    }
    else
    {
        NoteServed(dlSlot.m_slotAllocInfo, true);
    }

    // if the number of allocated symbols is greater than GetUlCtrlSymbols (), then don't
    // delete the allocation, as it will be removed when the CQI will be processed.
    // Otherwise, delete the allocation history for the slot.
    if (ulAllocations.m_totUlSym <= GetUlCtrlSyms())
    {
        NS_LOG_INFO("Removing UL allocation for slot " << params.m_snfSf << " size "
                                                       << m_ulAllocationMap.size());
        m_ulAllocationMap.erase(ulAllocationIt);
    }

    NS_LOG_INFO("Total DCI for DL : " << dlSlot.m_slotAllocInfo.m_varTtiAllocInfo.size()
                                      << " including DL CTRL");

    if (m_nrFhSchedSapProvider)
    {
        if (m_nrFhSchedSapProvider->GetFhControlMethod() !=
                NrFhControl::FhControlMethod::Dropping &&
            m_nrFhSchedSapProvider->GetFhControlMethod() != UINT8_MAX)
        {
            Simulator::Schedule(NanoSeconds(1),
                                &NrMacSchedulerNs3::CallNrFhControlForMapUpdate,
                                this,
                                dlSlot.m_slotAllocInfo.m_varTtiAllocInfo,
                                m_ueMap); // 1ns delay to give time for scheduling in all cells
        }
    }

    m_macSchedSapUser->SchedConfigInd(dlSlot);
#ifdef NS3_ASSERT_ENABLE
    // Validates DL resource allocation
    ResourceAssignmentMatrix::CheckResourceMatrixFromVarTtiAllocInfo(
        dlSlot.m_slotAllocInfo.m_varTtiAllocInfo,
        m_ueMap,
        GetDlBitmask(),
        14);
#endif
}

/**
 * @brief Do the process of scheduling for the UL
 * @param params scheduling parameters
 * @param ulHarqFeedback vector of UL HARQ negative feedback
 *
 * The scheduling process is composed by the UL and the DL parts, and it
 * decides how the resources should be divided between UEs. An important
 * thing to remember is that the slot being considered for DL decision can be
 * different for the slot for UL decision. This offset is due to the parameter
 * N2Delay (previously: UlSchedDelay).
 *
 * Another parameter to consider is the L1L2CtrlLatency  that defines the delay (in slots
 * number) between the slot that is currently "in the air" and the slot which is being
 * prepared for DL. The default value for both L1L2CtrlLatency and N2Delay (previously:
 * UlSchedDelay) is 2, so it means that while the slot number (frame, subframe, slot) is in
 * the air, the scheduler in this function will take decisions for DL in slot number (frame,
 * subframe, slot) + 2 and for UL in slot number (frame, subframe, slot) + 4.
 *
 * The consequences are an additional complexity derived from the fact that the
 * DL scheduling for a slot should remember the previous UL scheduling done in the
 * past for that slot.
 *
 * The process of scheduling UL data is defined as follows:
 *
 * - Append a UL CTRL symbol to the allocation list;
 * - Perform a scheduling for UL HARQ/data for this (DoScheduleUl());
 * - Indicate to the MAC the decision for that slot through SchedConfigInd()
 *
 * To know how the scheduling for UL is performed, take a look to the
 * function documentation for DoScheduleUl().
 *
 * At the end, the return of DoScheduleUl is passed to MAC through the function
 * SchedConfigInd().
 *
 * @see PointInFTPlane
 */
void
NrMacSchedulerNs3::ScheduleUl(const NrMacSchedSapProvider::SchedUlTriggerReqParameters& params,
                              const std::vector<UlHarqInfo>& ulHarqFeedback)
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Scheduling invoked for slot " << params.m_snfSf);

    NrMacSchedSapUser::SchedConfigIndParameters ulSlot(params.m_snfSf);
    ulSlot.m_slotAllocInfo.m_sfnSf = params.m_snfSf;
    ulSlot.m_slotAllocInfo.m_type = SlotAllocInfo::UL;

    // add slot for UL control, at last symbol, for slot type F and UL.
    AppendCtrlSym(static_cast<uint8_t>(m_macSchedSapUser->GetSymbolsPerSlot() - 1),
                  m_ulCtrlSymbols,
                  DciInfoElementTdma::UL,
                  &ulSlot.m_slotAllocInfo.m_varTtiAllocInfo);
    ulSlot.m_slotAllocInfo.m_numSymAlloc += m_ulCtrlSymbols;
    NS_ASSERT_MSG(ulSlot.m_slotAllocInfo.m_numSymAlloc <= 14,
                  "Invalid number of symbols: " << ulSlot.m_slotAllocInfo.m_numSymAlloc
                                                << " symbols. ");

    // Doing UL for slot ulSlot
    DoScheduleUl(ulHarqFeedback, params.m_snfSf, &ulSlot.m_slotAllocInfo, params.m_slotType);

    NS_LOG_INFO("Total DCI for UL : " << ulSlot.m_slotAllocInfo.m_varTtiAllocInfo.size()
                                      << " including UL CTRL");
    m_macSchedSapUser->BuildRarList(ulSlot.m_slotAllocInfo);
    m_macSchedSapUser->SchedConfigInd(ulSlot);

#ifdef NS3_ASSERT_ENABLE
    // Validates UL resource allocation
    ResourceAssignmentMatrix::CheckResourceMatrixFromVarTtiAllocInfo(
        ulSlot.m_slotAllocInfo.m_varTtiAllocInfo,
        m_ueMap,
        GetUlBitmask(),
        14);
#endif
}

/**
 * @brief Schedule UL HARQ and data
 * @param activeUlHarq List of active HARQ processes in UL
 * @param ulSfn Slot number
 * @param allocInfo Allocation info pointer (where to save the allocations)
 * @param type LTE/NR TDD slot type
 * @return the number of symbols used for the UL allocation
 *
 * The UL phase is, in this implementation, based completely on TDMA, so
 * in this method (even if the code is modular enough to be subclassed to
 * create a OFDMA-based scheduler) expects a TDMA-based allocations. You have
 * been warned!!
 *
 * The UL allocation is done for a slot number that is greater or equal to the
 * slot number for DL. Therefore, the maximum symbols available for this phase
 * are all the available data symbols in the slot. This opens up fairness problems
 * with DL, but how to balance UL and DL is still an open problem.
 *
 * If there are HARQ to retransmit, they will be retransmitted in the function
 * ScheduleUlHarq() until the code runs out of available symbols or
 * available HARQ to retransmit. The UE that can be selected for such assignation
 * are decided in the function ComputeActiveHarq.
 *
 * Then, for all the UE that requested a SR, it will be allocated one entire
 * symbol. After that, if any symbol remains, the function will schedule data.
 * The data allocation is made in DoScheduleUlData(), only for UEs that
 * have been selected by the function ComputeActiveUe and the ones that does not
 * have already a grant for HARQ (or SR).
 *
 * If any assignation is made, then the member variable m_ulAllocationMap (SlotElem)
 * is updated by storing the total UL symbols used in this slot, and
 * the details of each assignation, which include TBS, symStart, numSym, and
 * MCS. The assignations are stored as a list of AllocElem.
 *
 * Please note that the allocation must be done backwards from the last available
 * symbol to respect the final slot composition: DL CTRL, DL Data, UL Data, UL CTRL.
 */
uint8_t
NrMacSchedulerNs3::DoScheduleUl(const std::vector<UlHarqInfo>& ulHarqFeedback,
                                const SfnSf& ulSfn,
                                SlotAllocInfo* allocInfo,
                                LteNrTddSlotType type)
{
    NS_LOG_FUNCTION(this);

    NS_ASSERT(allocInfo->m_varTtiAllocInfo.size() == 1); // Just the UL CTRL
    ++m_pdcchRound;

    const bool pdcchOn = PdcchOn();
    const bool spsOn = pdcchOn && m_spsScenario != 0 && m_spsUlOn;
    uint8_t dataSymPerSlot = m_macSchedSapUser->GetSymbolsPerSlot() - m_ulCtrlSymbols;
    if (type == LteNrTddSlotType::F)
    { // if it's a type F, we have to consider DL CTRL symbols, otherwise, don't
        // (with NrPdcch::Symbols = 0 the downlink part of this slot is scheduled later and may
        // need up to 3)
        dataSymPerSlot -= pdcchOn ? m_pdcch->GetMaxSymbols() : m_dlCtrlSymbols;
    }

    ActiveHarqMap activeUlHarq;
    ComputeActiveHarq(&activeUlHarq, ulHarqFeedback);

    // Start the assignation from the last available data symbol, and like a shrimp
    // go backward.
    uint8_t lastSym = m_macSchedSapUser->GetSymbolsPerSlot() - m_ulCtrlSymbols;
    PointInFTPlane ulAssignationStartPoint(0, lastSym);
    uint8_t ulSymAvail = dataSymPerSlot;

    // Create the UL allocation map entry
    m_ulAllocationMap.emplace(ulSfn.GetEncoding(), SlotElem(0));

    if ((m_enableSrsInFSlots && type == LteNrTddSlotType::F) ||
        (m_enableSrsInUlSlots && type == LteNrTddSlotType::UL))
    { // SRS are included in F slots, and in UL slots if m_enableSrsInUlSlots=true
        m_srsSlotCounter++; // It's an uint, don't worry about wrap around
        NS_ASSERT(m_srsCtrlSymbols <= ulSymAvail);
        uint8_t srsSym = DoScheduleSrs(&ulAssignationStartPoint, allocInfo);
        ulSymAvail -= srsSym;
    }

    NS_LOG_DEBUG("Scheduling UL " << ulSfn << " UL HARQ to retransmit: " << ulHarqFeedback.size()
                                  << " Active Beams UL HARQ: " << activeUlHarq.size()
                                  << " starting from (" << +ulAssignationStartPoint.m_rbg << ", "
                                  << +ulAssignationStartPoint.m_sym << ")");

    // RACH
    uint8_t usedMsg3 = 0;
    if (!m_rachList.empty() && (type == LteNrTddSlotType::F || type == LteNrTddSlotType::UL))
    {
        usedMsg3 = DoScheduleUlMsg3(&ulAssignationStartPoint, ulSymAvail, allocInfo);
        NS_ASSERT_MSG(ulSymAvail >= usedMsg3,
                      "Available: " << +ulSymAvail << " used by UL MSG3: " << +usedMsg3);
        NS_LOG_INFO("For the slot " << ulSfn << " reserved " << static_cast<uint32_t>(usedMsg3)
                                    << " symbols for UL MSG3");
        ulSymAvail -= usedMsg3;
        allocInfo->m_numSymAlloc += usedMsg3;
        NS_ASSERT_MSG(allocInfo->m_numSymAlloc <= 14,
                      "Invalid number of symbols: " << allocInfo->m_numSymAlloc << " symbols. ");
        if (pdcchOn && usedMsg3 > 0)
        {
            // The random access response that carries these grants is scheduled by a DCI with
            // RA-RNTI in the common search space (TS 38.213 10.1, Type1-PDCCH CSS). It is sent
            // whether or not its CCEs were free: if not, it is counted as blocked.
            if (const auto grant = m_pdcch->Request(0, NrPdcch::RAR))
            {
                m_pdcch->Commit(grant, NrPdcch::RAR, nullptr);
            }
        }
    }

    // msg3 (RRC Connection Request) HARQ retransmissions. Allocated after the
    // initial msg3 grants so they do not collide in the same slot/RBs.
    if (m_msg3MaxRetx > 0 && !m_msg3RetxList.empty() &&
        (type == LteNrTddSlotType::F || type == LteNrTddSlotType::UL))
    {
        uint8_t usedMsg3Retx =
            DoScheduleUlMsg3Retx(&ulAssignationStartPoint, ulSymAvail, allocInfo);
        NS_ASSERT_MSG(ulSymAvail >= usedMsg3Retx,
                      "Available: " << +ulSymAvail << " used by UL MSG3 retx: " << +usedMsg3Retx);
        NS_LOG_INFO("For the slot " << ulSfn << " reserved " << static_cast<uint32_t>(usedMsg3Retx)
                                    << " symbols for UL MSG3 retx");
        ulSymAvail -= usedMsg3Retx;
        allocInfo->m_numSymAlloc += usedMsg3Retx;
        NS_ASSERT_MSG(allocInfo->m_numSymAlloc <= 14,
                      "Invalid number of symbols: " << allocInfo->m_numSymAlloc << " symbols. ");
        if (pdcchOn && usedMsg3Retx > 0)
        {
            // A retransmission of msg3 is granted by a DCI with TC-RNTI, common search space too
            if (const auto grant = m_pdcch->Request(0, NrPdcch::RAR))
            {
                m_pdcch->Commit(grant, NrPdcch::RAR, nullptr);
            }
        }
    }

    // Narrow configured-grant blocks (UlGrantRbs) share the data symbols, as retransmissions do
    // (HarqInFrequency()): the RBGs they leave, and the UEs that have one
    std::vector<bool> spsFreeRbgs;
    std::set<uint16_t> spsUes;
    if (pdcchOn)
    {
        // The PDCCH first, as in downlink: the retransmissions due and the UEs that want a
        // grant (the scheduling requests are taken in here already to know them all)
        if (!m_srList.empty())
        {
            DoScheduleUlSr(&ulAssignationStartPoint, m_srList);
            m_srList.clear();
        }
        if (spsOn)
        {
            // Configured grants: the blocks of this slot, and the requests they answer, which
            // are out of sight of everything that follows
            SpsPlanUl(ulSfn.Normalize());
        }
        ActiveUeMap wanting;
        ComputeActiveUe(&wanting,
                        &NrMacSchedulerUeInfo::GetUlLCG,
                        &NrMacSchedulerUeInfo::GetUlHarqVector,
                        "UL");
        // A UE that sends a repetition of a block in this slot (PlaceUlRepetitions()) needs no
        // DCI here: it can have no other PUSCH in the slot
        std::vector<UlHarqInfo> admittable = ulHarqFeedback;
        if (!m_ulRepetitions.empty())
        {
            std::set<uint16_t> repeating;
            for (const auto& rep : m_ulRepetitions)
            {
                repeating.insert(rep.dci->m_rnti);
            }
            std::erase_if(admittable, [&repeating](const UlHarqInfo& f) {
                return repeating.contains(f.m_rnti);
            });
            for (auto& [beam, ues] : wanting)
            {
                std::erase_if(ues, [&repeating](const UePtrAndBufferReq& ue) {
                    return repeating.contains(ue.first->m_rnti);
                });
            }
        }
        AdmitUlToPdcch(admittable, wanting, GetMaxUesPerSlot(ulSymAvail));
        if (spsOn)
        {
            // The blocks take the last symbols that are free, before anything else is placed
            const uint8_t usedSps = SpsAllocate(ulSfn.Normalize(),
                                                ulSfn.GetEncoding(),
                                                false,
                                                ulAssignationStartPoint.m_sym,
                                                ulSymAvail,
                                                allocInfo);
            // Narrow blocks that all span the data symbols leave those symbols open to the
            // retransmissions and the new data of the other UEs, in the RBGs left
            bool shared =
                usedSps > 0 && usedSps == ulSymAvail && GetUlGrantRbs() > 0 && UseHarqInFrequency();
            std::vector<bool> left = GetUlBitmask();
            for (const auto& alloc : allocInfo->m_varTtiAllocInfo)
            {
                const auto& dci = alloc.m_dci;
                if (shared && dci->m_format == DciInfoElementTdma::UL &&
                    dci->m_type == DciInfoElementTdma::DATA &&
                    (dci->m_sps == DciInfoElementTdma::SPS_ACTIVATION ||
                     dci->m_sps == DciInfoElementTdma::SPS_OCCASION))
                {
                    shared = dci->m_numSym == ulSymAvail;
                    spsUes.insert(dci->m_rnti);
                    for (std::size_t i = 0; i < left.size() && i < dci->m_rbgBitmask.size(); ++i)
                    {
                        left[i] = left[i] && !dci->m_rbgBitmask[i];
                    }
                }
            }
            if (shared)
            {
                allocInfo->m_numSymAlloc -= usedSps; // counted with the region, below
                spsFreeRbgs = left;
            }
            else
            {
                spsUes.clear();
                ulAssignationStartPoint.m_sym -= usedSps;
                ulSymAvail -= usedSps;
            }
        }
    }

    // OFDMA: the retransmissions take resource block groups over the ulSymAvail data symbols
    // before ulAssignationStartPoint, which new data shares (HarqInFrequency()); so do narrow
    // configured-grant blocks
    m_ulRbgFreeForData.clear();
    uint8_t retxRegion = 0;
    if (!spsFreeRbgs.empty())
    {
        m_ulRbgFreeForData = spsFreeRbgs;
        retxRegion = ulSymAvail;
    }
    // PUSCH repetition: the further transmissions of earlier blocks come first, in the same
    // region as the retransmissions
    std::set<uint16_t> repUes;
    if (!m_ulRepetitions.empty() && ulSymAvail > 0)
    {
        NS_ABORT_MSG_UNLESS(UseHarqInFrequency(),
                            "PuschRepetitions needs an OFDMA scheduler and one beam");
        if (m_ulRbgFreeForData.empty())
        {
            m_ulRbgFreeForData = GetUlBitmask();
        }
        repUes = PlaceUlRepetitions(ulAssignationStartPoint, ulSymAvail, allocInfo);
        if (!repUes.empty())
        {
            retxRegion = ulSymAvail;
        }
    }
    if (!activeUlHarq.empty() && UseHarqInFrequency())
    {
        if (m_ulRbgFreeForData.empty())
        {
            m_ulRbgFreeForData = GetUlBitmask();
        }
        // One PUSCH per UE and slot: a UE with a block of its configured grant here retransmits
        // in a later slot
        std::vector<UlHarqInfo> feedback;
        for (const auto& f : ulHarqFeedback)
        {
            if (!spsUes.contains(f.m_rnti) && !repUes.contains(f.m_rnti))
            {
                feedback.push_back(f);
                continue;
            }
            auto& process = m_ueMap.at(f.m_rnti)->m_ulHarq.Find(f.m_harqProcessId)->second;
            process.m_status = HarqProcess::WAITING_FEEDBACK;
            process.m_timer = 0;
            m_ulHarqToRetransmit.push_back(f);
            if (m_diagUlHarq)
            {
                std::ostringstream o;
                o << "requeue(cg/rep UE) " << f;
                DiagLog(o.str());
            }
        }
        const std::size_t bufferedBefore = m_ulHarqToRetransmit.size();
        if (ulSymAvail > 0 && m_schedHarq->ScheduleUlHarqInFrequency(ulAssignationStartPoint,
                                                                     ulSymAvail,
                                                                     m_ueMap,
                                                                     &m_ulHarqToRetransmit,
                                                                     feedback,
                                                                     allocInfo,
                                                                     &m_ulRbgFreeForData) > 0)
        {
            retxRegion = ulSymAvail;
        }
        else if (ulSymAvail == 0)
        {
            // No data symbol in this slot: every retransmission waits for the next one. Those of
            // the UEs with a configured grant or a repetition are in m_ulHarqToRetransmit
            // already (above): only the rest, or a process would be queued twice and
            // retransmitted twice
            for (const auto& f : feedback)
            {
                auto& process = m_ueMap.at(f.m_rnti)->m_ulHarq.Find(f.m_harqProcessId)->second;
                process.m_status = HarqProcess::WAITING_FEEDBACK;
                process.m_timer = 0;
                m_ulHarqToRetransmit.push_back(f);
                if (m_diagUlHarq)
                {
                    std::ostringstream o;
                    o << "requeue(no symbols) " << f;
                    DiagLog(o.str());
                }
            }
        }
        for (std::size_t i = bufferedBefore; i < m_ulHarqToRetransmit.size(); ++i)
        if (m_diagUlHarq)
        {
            std::ostringstream o;
            o << "requeue(no room/one already) " << m_ulHarqToRetransmit[i];
            DiagLog(o.str());
        }
        for (const auto& alloc : allocInfo->m_varTtiAllocInfo)
        {
            const auto& dci = alloc.m_dci;
            if (dci->m_format == DciInfoElementTdma::UL && dci->m_type == DciInfoElementTdma::DATA &&
                dci->m_rv > 0)
            if (m_diagUlHarq)
            {
                std::ostringstream o;
                o << "placed retx rnti " << dci->m_rnti << " p" << +dci->m_harqProcess << " rv "
                  << +dci->m_rv << " reps " << +dci->m_repetitions << " for " << ulSfn;
                DiagLog(o.str());
            }
        }
    }
    else if (!activeUlHarq.empty())
    {
        uint8_t usedHarq = ScheduleUlHarq(&ulAssignationStartPoint,
                                          ulSymAvail,
                                          m_ueMap,
                                          &m_ulHarqToRetransmit,
                                          ulHarqFeedback,
                                          allocInfo);
        NS_ASSERT_MSG(ulSymAvail >= usedHarq,
                      "Available: " << +ulSymAvail << " used by HARQ: " << +usedHarq);
        NS_LOG_INFO("For the slot " << ulSfn << " reserved " << static_cast<uint32_t>(usedHarq)
                                    << " symbols for UL HARQ retx");
        ulSymAvail -= usedHarq;
    }

    NS_ASSERT(ulAssignationStartPoint.m_rbg == 0);

    if (ulSymAvail > 0 && !m_srList.empty())
    {
        DoScheduleUlSr(&ulAssignationStartPoint, m_srList);
        m_srList.clear();
    }

    ActiveUeMap activeUlUe;
    ComputeActiveUe(&activeUlUe,
                    &NrMacSchedulerUeInfo::GetUlLCG,
                    &NrMacSchedulerUeInfo::GetUlHarqVector,
                    "UL");

    GetSecond GetUeInfoList;
    for (const auto& alloc : allocInfo->m_varTtiAllocInfo)
    {
        for (auto it = activeUlUe.begin(); it != activeUlUe.end(); /* no incr */)
        {
            auto& ueInfos = GetUeInfoList(*it);
            for (auto ueIt = ueInfos.begin(); ueIt != ueInfos.end(); /* no incr */)
            {
                GetFirst GetUeInfoPtr;
                if (GetUeInfoPtr(*ueIt)->m_rnti == alloc.m_dci->m_rnti)
                {
                    NS_LOG_INFO("Removed RNTI " << alloc.m_dci->m_rnti
                                                << " from active ue list "
                                                   "because it has already an HARQ scheduled");
                    ueInfos.erase(ueIt);
                    break;
                }
                else
                {
                    ++ueIt;
                }
            }
            if (!ueInfos.empty())
            {
                ++it;
            }
            else
            {
                activeUlUe.erase(it);
                break;
            }
        }
    }

    if (pdcchOn)
    {
        KeepUesWithPdcch(&activeUlUe, m_ulPdcchGrants);
    }

    const auto dataMask = GetUlDataBitmask();
    const bool rbgLeft = std::find(dataMask.begin(), dataMask.end(), true) != dataMask.end();
    uint8_t usedUl = 0;
    if (ulSymAvail > 0 && !activeUlUe.empty() && rbgLeft)
    {
        usedUl = DoScheduleUlData(&ulAssignationStartPoint, ulSymAvail, activeUlUe, allocInfo);
        NS_LOG_INFO("For the slot " << ulSfn << " reserved " << static_cast<uint32_t>(usedUl)
                                    << " symbols for UL data tx");
        ulSymAvail -= usedUl;
    }
    if (retxRegion > 0)
    {
        // The new data, if any, took the same symbols as the retransmissions
        NS_ASSERT_MSG(usedUl == 0 || usedUl == retxRegion,
                      "UL data " << +usedUl << " symbols, retransmissions " << +retxRegion);
        if (usedUl == 0)
        {
            ulSymAvail -= retxRegion;
            ulAssignationStartPoint.m_sym -= retxRegion;
            allocInfo->m_numSymAlloc += retxRegion;
            NS_ASSERT_MSG(allocInfo->m_numSymAlloc <= 14,
                          "Invalid number of symbols: " << allocInfo->m_numSymAlloc);
        }
    }
    m_ulRbgFreeForData.clear();
    QueueUlRepetitions(*allocInfo);

    if (pdcchOn)
    {
        SettlePdcch(allocInfo, false);
    }
    else
    {
        NoteServed(*allocInfo, false);
    }
    if (spsOn)
    {
        SpsUnhide();
    }

    std::vector<uint32_t> symToAl;
    symToAl.resize(15, 0);

    auto& totUlSym = m_ulAllocationMap.at(ulSfn.GetEncoding()).m_totUlSym;
    auto& allocations = m_ulAllocationMap.at(ulSfn.GetEncoding()).m_ulAllocations;
    for (const auto& alloc : allocInfo->m_varTtiAllocInfo)
    {
        if (alloc.m_dci->m_format == DciInfoElementTdma::UL)
        {
            // Here we are assuming (with the assignment) that all the
            // allocations starting at a particular symbol will have the same
            // length.
            symToAl[alloc.m_dci->m_symStart] = alloc.m_dci->m_numSym;
            NS_LOG_INFO("UL Allocation. RNTI " << alloc.m_dci->m_rnti << ", symStart "
                                               << static_cast<uint32_t>(alloc.m_dci->m_symStart)
                                               << " numSym " << +alloc.m_dci->m_numSym);

            if (alloc.m_dci->m_type == DciInfoElementTdma::DATA ||
                alloc.m_dci->m_type == DciInfoElementTdma::MSG3)
            {
                NS_LOG_INFO("Placed the above allocation in the CQI map");
                allocations.emplace_back(alloc.m_dci->m_rnti,
                                         alloc.m_dci->m_tbSize,
                                         alloc.m_dci->m_symStart,
                                         alloc.m_dci->m_numSym,
                                         alloc.m_dci->m_mcs,
                                         alloc.m_dci->m_rank,
                                         alloc.m_dci->m_rbgBitmask);
            }
        }
    }

    for (const auto& v : symToAl)
    {
        totUlSym += v;
    }

    NS_ASSERT_MSG((dataSymPerSlot + m_ulCtrlSymbols) - ulSymAvail == totUlSym,
                  "UL symbols available: "
                      << static_cast<uint32_t>(dataSymPerSlot + m_ulCtrlSymbols)
                      << " UL symbols available at end of sched: "
                      << static_cast<uint32_t>(ulSymAvail)
                      << " total of symbols registered in the allocation: "
                      << static_cast<uint32_t>(totUlSym) << " slot type " << type);

    NS_LOG_INFO("For the slot " << ulSfn << " registered a total of "
                                << static_cast<uint32_t>(totUlSym) << " symbols and "
                                << allocations.size() << " data allocations, with a total of "
                                << allocInfo->m_varTtiAllocInfo.size());
    NS_ASSERT(m_ulAllocationMap.at(ulSfn.GetEncoding()).m_totUlSym == totUlSym);

    return dataSymPerSlot - ulSymAvail;
}

uint8_t
NrMacSchedulerNs3::DoScheduleSrs(PointInFTPlane* spoint, SlotAllocInfo* allocInfo)
{
    NS_LOG_FUNCTION(this);

    uint8_t used = 0;

    // Without UE, don't schedule any SRS
    if (m_ueMap.empty())
    {
        return used;
    }

    // Find the UE for which this is true:
    // absolute_slot_number % periodicity = offset_UEx
    // Assuming that all UEs share the same periodicity.

    uint32_t offset_UEx = m_srsSlotCounter % m_ueMap.begin()->second->m_srsPeriodicity;
    uint16_t rnti = 0;

    for (const auto& ue : m_ueMap)
    {
        if (ue.second->m_srsOffset == offset_UEx)
        {
            rnti = ue.second->m_rnti;
        }
    }

    if (rnti == 0)
    {
        return used; // No SRS in this slot!
    }

    // Schedule 4 allocation, of 1 symbol each, in TDMA mode, for the RNTI found.

    for (uint32_t i = 0; i < m_srsCtrlSymbols; ++i)
    {
        std::vector<bool> rbgAssigned(GetBandwidthInRbg(), true);

        NS_LOG_INFO("UE " << rnti << " assigned symbol " << +spoint->m_sym << " for SRS tx");

        spoint->m_sym--;

        uint8_t numSym{1};
        uint8_t mcs{0};
        uint8_t rank{1};
        Ptr<const ComplexMatrixArray> precMats{nullptr};
        uint32_t tbs{0};
        uint8_t ndi{1};
        uint8_t rv{0};
        auto dci = std::make_shared<DciInfoElementTdma>(rnti,
                                                        DciInfoElementTdma::UL,
                                                        spoint->m_sym,
                                                        numSym,
                                                        mcs,
                                                        rank,
                                                        precMats,
                                                        tbs,
                                                        ndi,
                                                        rv,
                                                        DciInfoElementTdma::SRS,
                                                        GetBwpId(),
                                                        GetTpc());
        dci->m_rbgBitmask = GetUlBitmask();

        allocInfo->m_numSymAlloc += 1;
        NS_ASSERT_MSG(allocInfo->m_numSymAlloc <= 14,
                      "Invalid number of symbols: " << dci->m_numSym << " symbols. ");

        allocInfo->m_varTtiAllocInfo.emplace_front(dci);

        used++;
    }

    return used;
}

namespace
{

/// Bytes NrUeMac keeps of every uplink transport block for its buffer status report: 5, or 2
/// with NrUeMac::CompactBsr (it was a fixed 5 here, so a configured grant sized for a packet
/// was 3 bytes too large for a UE with the short BSR)
uint32_t
UlBsrBytes()
{
    return NrMacShortBsrCe::OverheadBytes();
}

/// The bearer with a 5QI among the logical channels of a UE, null if it has none
NrMacSchedulerLC*
FindLcOfFiveQi(const std::unordered_map<uint8_t, LCGPtr>& lcgs,
               uint8_t fiveQi,
               uint8_t* lcgId,
               uint8_t* lcId)
{
    for (const auto& lcg : lcgs)
    {
        for (const uint8_t id : lcg.second->GetLCId())
        {
            if (lcg.second->GetFiveQi(id) == fiveQi)
            {
                *lcgId = lcg.first;
                *lcId = id;
                return lcg.second->PeekLC(id);
            }
        }
    }
    return nullptr;
}

} // namespace

uint32_t
NrMacSchedulerNs3::GetMaxUesPerSlot([[maybe_unused]] uint32_t symAvail) const
{
    return std::numeric_limits<uint32_t>::max();
}

bool
NrMacSchedulerNs3::PdcchOn()
{
    if (!m_pdcch)
    {
        if (!m_macSchedSapUser || GetCellId() == UINT16_MAX)
        {
            return false;
        }
        m_pdcch = NrPdcch::Get(GetCellId());
    }
    return m_pdcch->IsEnabled();
}

void
NrMacSchedulerNs3::KeepUesWithPdcch(ActiveUeMap* activeUe,
                                    const std::map<uint16_t, NrPdcch::Grant>& grants)
{
    for (auto it = activeUe->begin(); it != activeUe->end();)
    {
        std::erase_if(it->second,
                      [&grants](const auto& ue) { return !grants.contains(ue.first->m_rnti); });
        it = it->second.empty() ? activeUe->erase(it) : std::next(it);
    }
}

void
NrMacSchedulerNs3::AdmitDlToPdcch(const ActiveHarqMap& activeDlHarq,
                                  ActiveUeMap* activeDlUe,
                                  uint32_t maxUes)
{
    NS_LOG_FUNCTION(this);
    m_dlPdcchGrants.clear();

    // Retransmissions first: each has a DCI of its own, and one per UE fits a slot
    for (const auto& beam : activeDlHarq)
    {
        for (const auto& process : beam.second)
        {
            const uint16_t rnti = process->second.m_dciElement->m_rnti;
            if (m_dlPdcchGrants.contains(rnti))
            {
                continue;
            }
            if (const auto grant = m_pdcch->Request(rnti, NrPdcch::DL_RETX))
            {
                m_dlPdcchGrants.emplace(rnti, grant);
            }
        }
    }

    // Then the releases and activations of semi-persistent configurations
    SpsAdmit(true);

    // Then new data, in the order of the scheduling policy
    const std::vector<UePtrAndBufferReq> wanting = InPolicyOrder(*activeDlUe, true);
    for (const auto& ue : wanting)
    {
        const uint16_t rnti = ue.first->m_rnti;
        if (m_dlPdcchGrants.contains(rnti))
        {
            continue;
        }
        if (m_dlPdcchGrants.size() >= maxUes)
        {
            break;
        }
        if (const auto grant = m_pdcch->Request(rnti, NrPdcch::DL_NEW))
        {
            m_dlPdcchGrants.emplace(rnti, grant);
        }
    }
    for (const auto& ue : wanting)
    {
        if (!m_dlPdcchGrants.contains(ue.first->m_rnti))
        {
            PassedOver(ue, true);
        }
    }

    KeepUesWithPdcch(activeDlUe, m_dlPdcchGrants);
}

std::vector<NrMacSchedulerNs3::UePtrAndBufferReq>
NrMacSchedulerNs3::InPolicyOrder(const ActiveUeMap& activeUe, bool isDl) const
{
    // All beams together, the UE that had its last DCI longest ago first (within a beam
    // ComputeActiveUe() left them so); then as the policy ranks them: round robin leaves them
    // as they are, the others put their best UE first and keep this order between equals
    const auto& last = isDl ? m_dlPdcchLast : m_ulPdcchLast;
    auto servedAt = [&last](const UePtrAndBufferReq& ue) {
        const auto it = last.find(ue.first->m_rnti);
        return std::make_pair(it == last.end() ? uint64_t{0} : it->second, ue.first->m_rnti);
    };
    std::vector<UePtrAndBufferReq> ues;
    for (const auto& beam : activeUe)
    {
        ues.insert(ues.end(), beam.second.begin(), beam.second.end());
    }
    std::sort(ues.begin(), ues.end(), [&servedAt](const auto& a, const auto& b) {
        return servedAt(a) < servedAt(b);
    });
    SortByPolicy(&ues, isDl);
    return ues;
}

void
NrMacSchedulerNs3::AdmitUlToPdcch(const std::vector<UlHarqInfo>& ulHarqFeedback,
                                  const ActiveUeMap& activeUlUe,
                                  uint32_t maxUes)
{
    NS_LOG_FUNCTION(this);
    m_ulPdcchGrants.clear();
    m_ulPdcchTaken.clear();

    for (const auto& feedback : ulHarqFeedback)
    {
        const uint16_t rnti = feedback.m_rnti;
        if (m_ulPdcchGrants.contains(rnti) || !m_ueMap.contains(rnti))
        {
            continue;
        }
        if (const auto grant = m_pdcch->Request(rnti, NrPdcch::UL_RETX))
        {
            m_ulPdcchGrants.emplace(rnti, grant);
        }
    }

    // Then the releases and activations of configured grants
    SpsAdmit(false);

    const std::vector<UePtrAndBufferReq> wanting = InPolicyOrder(activeUlUe, false);
    for (const auto& ue : wanting)
    {
        const uint16_t rnti = ue.first->m_rnti;
        if (m_ulPdcchGrants.contains(rnti))
        {
            continue;
        }
        if (m_ulPdcchGrants.size() >= maxUes)
        {
            break;
        }
        if (const auto grant = m_pdcch->Request(rnti, NrPdcch::UL_NEW))
        {
            m_ulPdcchGrants.emplace(rnti, grant);
        }
    }
    for (const auto& ue : wanting)
    {
        if (!m_ulPdcchGrants.contains(ue.first->m_rnti))
        {
            PassedOver(ue, false);
        }
    }
}

bool
NrMacSchedulerNs3::TakePdcch(uint16_t rnti, bool isDl)
{
    if (!PdcchOn())
    {
        return true;
    }
    if (isDl)
    {
        return m_dlPdcchGrants.contains(rnti);
    }
    // In uplink several retransmissions of one UE could be granted in a slot; each would need
    // a DCI, and the UE has one PDCCH candidate reserved
    return m_ulPdcchGrants.contains(rnti) && m_ulPdcchTaken.insert(rnti).second;
}

void
NrMacSchedulerNs3::SettlePdcch(SlotAllocInfo* allocInfo, bool isDl)
{
    NS_LOG_FUNCTION(this);
    auto& grants = isDl ? m_dlPdcchGrants : m_ulPdcchGrants;
    auto& last = isDl ? m_dlPdcchLast : m_ulPdcchLast;
    const auto format = isDl ? DciInfoElementTdma::DL : DciInfoElementTdma::UL;
    for (auto& alloc : allocInfo->m_varTtiAllocInfo)
    {
        const auto& dci = alloc.m_dci;
        if (dci->m_format != format || dci->m_type != DciInfoElementTdma::DATA ||
            dci->m_sps != DciInfoElementTdma::DYNAMIC)
        {
            continue; // control and SRS symbols and msg3 (granted by the random access response)
                      // have no DCI; neither has a block of a semi-persistent configuration,
                      // and the one that comes with the activation has its PDCCH already
        }
        const auto it = grants.find(dci->m_rnti);
        NS_ABORT_MSG_IF(it == grants.end(),
                        "A " << (isDl ? "downlink assignment" : "uplink grant") << " for RNTI "
                             << dci->m_rnti << " without a PDCCH candidate");
        NrPdcch::Purpose purpose;
        if (dci->m_ndi == 1)
        {
            purpose = isDl ? NrPdcch::DL_NEW : NrPdcch::UL_NEW;
        }
        else
        {
            purpose = isDl ? NrPdcch::DL_RETX : NrPdcch::UL_RETX;
        }
        m_pdcch->Commit(it->second, purpose, dci);
        last[dci->m_rnti] = m_pdcchRound;
        grants.erase(it);
        if (purpose == NrPdcch::DL_NEW && m_spsScenario != 0)
        {
            // The first packet after the speaker started or stopped talking, if it goes with
            // this DCI: the assignment has bytes for its bearer
            const auto sps = m_spsDl.find(dci->m_rnti);
            uint8_t lcgId;
            uint8_t lcId;
            if (sps != m_spsDl.end() && (sps->second.change != 0 || sps->second.pieces != 0) &&
                FindLcOfFiveQi(m_ueMap.at(dci->m_rnti)->m_dlLCG, m_spsFiveQi, &lcgId, &lcId) &&
                std::any_of(alloc.m_rlcPduInfo.begin(),
                            alloc.m_rlcPduInfo.end(),
                            [lcId](const RlcPduInfo& pdu) { return pdu.m_lcid == lcId; }))
            {
                SpsState& st = sps->second;
                // (what was left of one that had started in pieces goes with it: one more PDU)
                SpsNoteChange(true, st.pieces, NrPdcch::CHANGE_PIECE_PDUS);
                SpsNoteChange(true, st.change, NrPdcch::CHANGE_DCI);
                st.pieces = 0;
                st.change = 0;
            }
        }
        if (purpose == NrPdcch::UL_NEW && m_spsScenario != 0)
        {
            // The DCI for what a block of a configured grant left: one is enough
            const auto sps = m_spsUl.find(dci->m_rnti);
            uint8_t backlogLcg;
            uint8_t backlogLc;
            NrMacSchedulerLC* backlog =
                sps != m_spsUl.end() && sps->second.dynamicUntilDrained && m_cgBlockPacketOnly
                    ? FindLcOfFiveQi(m_ueMap.at(dci->m_rnti)->m_ulLCG,
                                     m_spsFiveQi,
                                     &backlogLcg,
                                     &backlogLc)
                    : nullptr;
            if (backlog != nullptr && backlog->m_rlcTransmissionQueueSize > 0)
            {
                // CgBlockPacketOnly: a block of the configuration has no room to spare, so what
                // the reports say is left is granted in full, a grant per slot as long as the
                // UE's grants are smaller than that (UlGrantRbs), not one per round trip
                sps->second.leftover = 0;
            }
            else if (sps != m_spsUl.end() && sps->second.dynamicUntilDrained)
            {
                sps->second.dynamicUntilDrained = false;
                sps->second.leftover = 0;
                sps->second.seenReport = 0;
                sps->second.dynamicSlot = m_ulSlotAtHand;
                // What is left after this grant, the buffer status report in it will say
                uint8_t lcgId;
                uint8_t lcId;
                if (NrMacSchedulerLC* lc = FindLcOfFiveQi(m_ueMap.at(dci->m_rnti)->m_ulLCG,
                                                          m_spsFiveQi,
                                                          &lcgId,
                                                          &lcId))
                {
                    lc->m_rlcTransmissionQueueSize = 0;
                }
            }
        }
    }
    for (const auto& grant : grants)
    {
        m_pdcch->Release(grant.second);
    }
    grants.clear();
}

bool
NrMacSchedulerNs3::SpsUsesFullBand() const
{
    return false;
}

uint32_t
NrMacSchedulerNs3::SpsPeriodSlots(uint8_t config) const
{
    const uint32_t ms = config == 1 ? m_spsSilencePeriodMs : m_spsPeriodMs;
    return static_cast<uint32_t>(ms * MilliSeconds(1).GetNanoSeconds() /
                                 m_macSchedSapUser->GetSlotPeriod().GetNanoSeconds());
}

uint32_t
NrMacSchedulerNs3::SpsUlTbBytes(uint8_t config) const
{
    // The packet: in scenario 2 a configuration starts with the first packet of a talk spurt or
    // of a silence, which has the context update of ROHC, and the gNB cannot see a packet of
    // the UE before it grants it. Then what the UE adds: its buffer status report, and the MAC
    // CE that confirms the activation: one octet with one configuration (TS 38.321 6.1.3.7),
    // six with two (Multiple Entry Configured Grant Confirmation, 6.1.3.31).
    const uint32_t packet = (config == 1 ? m_spsSilenceTbBytes : m_spsTbBytes) +
                            (m_spsScenario == 2 ? m_spsFitExtraBytes : 0);
    const uint32_t ce = m_spsScenario == 2 ? m_cgMultiEntryCeBytes : 1U;
    if (m_cgBlockPacketOnly)
    {
        // The packet alone: its buffer status report goes in when there is room for it, and
        // the confirmation of an activation comes in the grant before the first block. The
        // silence configuration of scenario 2 starts with no grant before it: its block holds
        // the confirmation (and leaves room for a report in its later blocks).
        return packet + (m_spsScenario == 2 && config == 1 ? ce : 0);
    }
    return packet + UlBsrBytes() + ce;
}

uint32_t
NrMacSchedulerNs3::SpsUlTbBytesWithCe(uint8_t config) const
{
    // An activation the UE confirms in its first block: room for the confirmation
    const uint32_t tb = SpsUlTbBytes(config);
    if (!m_cgBlockPacketOnly || (m_spsScenario == 2 && config == 1))
    {
        return tb;
    }
    return tb + (m_spsScenario == 2 ? m_cgMultiEntryCeBytes : 1U);
}

uint32_t
NrMacSchedulerNs3::LearnedSrGrant(uint16_t rnti) const
{
    const auto ue = m_ueMap.find(rnti);
    if (!m_srGrantLearned || ue == m_ueMap.end())
    {
        return m_srGrantBytes;
    }
    uint8_t lcgId = 0;
    uint8_t lcId = 0;
    if (FindLcOfFiveQi(ue->second->m_ulLCG, m_spsFiveQi, &lcgId, &lcId) == nullptr)
    {
        return m_srGrantBytes;
    }
    const uint32_t last = NrPdcp::GetVoiceActivity(GetCellId(), rnti, lcId, false).lastPduBytes;
    // the first request of the call: SrGrantBytes; then the last packet received, as it came
    // (PDCP PDU, RLC header, MAC subheader), and a buffer status report
    return last == 0 ? m_srGrantBytes : last + m_srGrantL2Bytes + UlBsrBytes();
}

bool
NrMacSchedulerNs3::SpsGeometry(bool isDl,
                               uint8_t mcs,
                               uint8_t rank,
                               uint32_t needTb,
                               uint8_t maxSym,
                               SpsBlock* block) const
{
    const auto mask = isDl ? GetDlBitmask() : GetUlBitmask();
    const auto nRbg = static_cast<uint32_t>(std::count(mask.begin(), mask.end(), true));
    const uint32_t rbPerRbg = GetNumRbPerRbg();
    const Ptr<NrAmc>& amc = isDl ? m_dlAmc : m_ulAmc;
    if (!isDl && GetUlGrantRbs() > 0)
    {
        // An uplink block as wide as a grant (UlGrantRbs) over all the symbols it may have, as
        // a dynamic grant of OFDMA has them, which it shares with the other UEs; with the lowest
        // MCS, up to that of the link, that holds the bytes
        const uint32_t width = std::min(nRbg, (GetUlGrantRbs() + rbPerRbg - 1) / rbPerRbg);
        const uint32_t nprb = width * maxSym * rbPerRbg;
        if (maxSym == 0 || amc->CalculateTbSize(mcs, rank, nprb) < needTb)
        {
            return false;
        }
        uint8_t lowest = mcs;
        while (lowest > 0 && amc->CalculateTbSize(lowest - 1, rank, nprb) >= needTb)
        {
            --lowest;
        }
        block->numSym = maxSym;
        block->numRbg = static_cast<uint16_t>(width);
        block->mcs = lowest;
        block->rank = rank;
        block->tbs = amc->CalculateTbSize(lowest, rank, nprb);
        block->usable = block->tbs;
        return true;
    }
    // The fewest symbols whose whole band holds the bytes ...
    uint8_t numSym = 0;
    for (uint8_t sym = 1; sym <= maxSym && numSym == 0; ++sym)
    {
        if (amc->CalculateTbSize(mcs, rank, nRbg * sym * rbPerRbg) >= needTb)
        {
            numSym = sym;
        }
    }
    if (numSym == 0)
    {
        return false;
    }
    // ... and of them as few RBGs as hold them: that gives the size of the block
    uint32_t numRbg = nRbg;
    uint32_t lo = 1;
    while (lo < numRbg)
    {
        const uint32_t mid = (lo + numRbg) / 2;
        if (amc->CalculateTbSize(mcs, rank, mid * numSym * rbPerRbg) >= needTb)
        {
            numRbg = mid;
        }
        else
        {
            lo = mid + 1;
        }
    }
    block->usable = amc->CalculateTbSize(mcs, rank, numRbg * numSym * rbPerRbg);
    // The resources: these, or whole symbols. An uplink block (one not narrowed by UlGrantRbs,
    // above) has symbols of its own:
    // the transmissions of two UEs reach the gNB together only if their timing is exact.
    const bool wholeSymbols = !isDl || SpsUsesFullBand();
    block->numSym = numSym;
    block->numRbg = static_cast<uint16_t>(wholeSymbols ? nRbg : numRbg);
    block->mcs = mcs;
    block->rank = rank;
    block->tbs = wholeSymbols ? amc->CalculateTbSize(mcs, rank, nRbg * numSym * rbPerRbg)
                              : block->usable;
    return true;
}

uint8_t
NrMacSchedulerNs3::SpsSymbolBudget(bool isDl) const
{
    const uint32_t symbols = m_macSchedSapUser->GetSymbolsPerSlot();
    const uint32_t taken = isDl ? m_pdcch->GetMaxSymbols() : m_ulCtrlSymbols + m_srsCtrlSymbols;
    return static_cast<uint8_t>(symbols > taken ? symbols - taken : 0);
}

bool
NrMacSchedulerNs3::SpsHasRoom(bool isDl,
                              uint16_t rnti,
                              uint64_t slot,
                              uint8_t config,
                              uint32_t needTb,
                              uint8_t mcs,
                              uint8_t rank,
                              uint8_t reps) const
{
    const uint8_t budget = SpsSymbolBudget(isDl);
    SpsBlock shape;
    if (!SpsGeometry(isDl, mcs, rank, needTb, budget, &shape))
    {
        return false;
    }
    const uint64_t period = SpsPeriodSlots(config);
    uint32_t symbols = shape.numSym;
    uint32_t area = shape.numSym * shape.numRbg;
    // The configurations that start in this very slot ...
    for (const auto& planned : m_spsPlanned)
    {
        symbols += planned.first;
        area += planned.first * planned.second;
    }
    // ... and the active ones whose blocks come back in a slot these blocks come back in
    for (const auto& other : isDl ? m_spsDl : m_spsUl)
    {
        const SpsState& st = other.second;
        if (other.first == rnti || !st.active)
        {
            continue;
        }
        // Two series of slots, each with its period, meet if and only if their first slots
        // differ by a multiple of the greatest common divisor of the periods. With repetitions
        // a block takes as many slots in a row; two series meet if any of these do.
        const auto gcd = static_cast<int64_t>(std::gcd<uint64_t, uint64_t>(period, st.periodSlots));
        const int64_t apart =
            ((static_cast<int64_t>(slot) - static_cast<int64_t>(st.startSlot)) % gcd + gcd) % gcd;
        if (apart <= static_cast<int64_t>(st.reps) - 1 ||
            apart >= gcd - (static_cast<int64_t>(reps) - 1))
        {
            symbols += st.numSym;
            area += st.numSym * st.numRbg;
        }
    }
    const auto mask = isDl ? GetDlBitmask() : GetUlBitmask();
    const auto nRbg = static_cast<uint32_t>(std::count(mask.begin(), mask.end(), true));
    const bool wholeSymbols = SpsUsesFullBand() || (!isDl && GetUlGrantRbs() == 0);
    if (wholeSymbols ? symbols > budget : area > budget * nRbg)
    {
        return false;
    }
    m_spsPlanned.emplace_back(shape.numSym, shape.numRbg);
    return true;
}

uint8_t
NrMacSchedulerNs3::SpsUlMcs(const std::shared_ptr<NrMacSchedulerUeInfo>& ue) const
{
    const auto last = m_spsUlWholeBandMcs.find(ue->m_rnti);
    // (the MCS of the UE is not the one of its last transmission if the scheduler has set it
    // back to its starting value in between: then there is nothing to work from)
    return last != m_spsUlWholeBandMcs.end() && last->second.first == ue->m_ulMcs
               ? last->second.second
               : ue->m_ulMcs;
}

std::pair<uint8_t, uint8_t>
NrMacSchedulerNs3::SpsUlLink(const std::shared_ptr<NrMacSchedulerUeInfo>& ue,
                             uint32_t needTb,
                             double marginDb,
                             uint8_t reps,
                             double mcsMarginDb) const
{
    // the MCS at mcsMarginDb less SINR than measured (CgSinrPercentile)
    const double scale = mcsMarginDb > 0.0 ? std::pow(10.0, -mcsMarginDb / 10.0) : 1.0;
    const uint8_t fixed = ue->m_puschRepetitions;
    if (GetUlGrantRbs() == 0 || fixed == 1 || !UseHarqInFrequency())
    {
        return {SpsUlMcs(ue), 1};
    }
    // The resource blocks of a block (SpsGeometry())
    const auto mask = GetUlBitmask();
    const auto nRbg = static_cast<uint32_t>(std::count(mask.begin(), mask.end(), true));
    const uint32_t rbPerRbg = GetNumRbPerRbg();
    const uint32_t rbs = std::min(nRbg, (GetUlGrantRbs() + rbPerRbg - 1) / rbPerRbg) * rbPerRbg;
    if (reps == 0)
    {
        // K of a configured grant: numberOfRepetitions of its time domain allocation row, or repK
        // (TS 38.214 6.1.2.3), here the numbers of TS 38.331 numberOfRepetitionsExt-r17; the K
        // repetitions may not last longer than the period P (6.1.2.3.1), so not more than the
        // slots of the talk period (the silence period is longer)
        const uint32_t slotsOfPeriod = static_cast<uint32_t>(SpsPeriodSlots(0));
        std::vector<uint8_t> allowed;
        for (const uint8_t k : NrMacSchedulerUeInfo::kPuschRepetitionValues)
        {
            if (k <= std::max<uint32_t>(slotsOfPeriod, 1))
            {
                allowed.push_back(k);
            }
        }
        reps = allowed.back();
        const double sinr = ue->GetUlMeanSinrFor(rbs);
        const double target = ue->m_puschRepSinr * std::pow(10.0, marginDb / 10.0);
        for (const uint8_t k : allowed)
        {
            if (fixed > 1 ? k >= fixed : sinr * k >= target)
            {
                reps = k;
                break;
            }
        }
        if (fixed == 0 && sinr <= 0.0)
        {
            return {SpsUlMcs(ue), 1}; // no measurement yet
        }
        // The block has to hold its bytes in its resource blocks: more repetitions, a higher MCS
        const uint32_t nprb = rbs * SpsSymbolBudget(false);
        while (needTb > 0 && reps < allowed.back() &&
               m_ulAmc->CalculateTbSize(ue->GetUlMcsForReps(rbs, reps, scale),
                                        ue->m_ulRank,
                                        nprb) < needTb)
        {
            reps = *std::upper_bound(allowed.begin(), allowed.end(), reps);
        }
    }
    return {ue->GetUlMcsForReps(rbs, reps, scale), reps};
}

double
NrMacSchedulerNs3::SpsUlSinr(const std::shared_ptr<NrMacSchedulerUeInfo>& ue) const
{
    // the resource blocks of a block, as SpsUlLink() takes them
    const auto mask = GetUlBitmask();
    const auto nRbg = static_cast<uint32_t>(std::count(mask.begin(), mask.end(), true));
    const uint32_t rbPerRbg = GetNumRbPerRbg();
    const uint32_t rbs = std::min(nRbg, (GetUlGrantRbs() + rbPerRbg - 1) / rbPerRbg) * rbPerRbg;
    return ue->GetUlMeanSinrFor(rbs);
}

int
NrMacSchedulerNs3::CgOutOfDate(SpsState& st, const std::shared_ptr<NrMacSchedulerUeInfo>& ue) const
{
    const Time now = Simulator::Now();
    const double sinr = SpsUlSinr(ue);
    if (sinr <= 0.0)
    {
        return 0; // no measurement
    }
    // K per the SINR (PuschRepetitions = 0): how far the configuration's K falls short of the
    // target, or how much it leaves to spare
    const bool perSinr = ue->m_puschRepetitions == 0;
    const double shortDb =
        perSinr ? 10.0 * std::log10(ue->m_puschRepSinr) - 10.0 * std::log10(sinr * st.reps) : 0.0;
    // the MCS the configuration's K carries now, against the configuration's (SpsOutOfDate())
    const int mcsState =
        SpsOutOfDate(false, st, SpsUlLink(ue, 0, 0.0, st.reps).first, ue->m_ulRank);
    const bool worse = (perSinr && shortDb >= m_cgRaiseDb) || mcsState < 0;
    const bool better =
        !worse && ((perSinr && SpsUlLink(ue, st.needTb, m_cgLowerDb).second < st.reps) ||
                   mcsState > 0);
    st.worseSince = worse ? (st.worseSince == Time::Min() ? now : st.worseSince) : Time::Min();
    st.betterSince =
        better ? (st.betterSince == Time::Min() ? now : st.betterSince) : Time::Min();

    const bool urgent = (perSinr && shortDb >= m_cgUrgentDb) ||
                        (m_cgFailures > 0 && st.failRun >= m_cgFailures &&
                         ((perSinr && shortDb > 0.0) || mcsState < 0));
    if (urgent)
    {
        return -1;
    }
    const bool intervalOver =
        st.lastActivation == Time::Min() || now - st.lastActivation >= m_cgMinInterval;
    if (!intervalOver)
    {
        return 0;
    }
    if (worse && now - st.worseSince >= m_cgRaiseTime)
    {
        return -1;
    }
    if (better && now - st.betterSince >= m_cgLowerTime)
    {
        return 1;
    }
    return 0;
}

double
NrMacSchedulerNs3::CgMarginDb(SpsState& st,
                              const std::shared_ptr<NrMacSchedulerUeInfo>& ue,
                              uint64_t slot) const
{
    const Time now = Simulator::Now();
    const double sinr = SpsUlSinr(ue);
    if (sinr > 0.0 && (st.sinrDb.empty() || slot >= st.sinrSlot + SpsPeriodSlots(0)))
    {
        st.sinrDb.emplace_back(now, 10.0 * std::log10(sinr));
        st.sinrSlot = slot;
    }
    while (!st.sinrDb.empty() && now - st.sinrDb.front().first > m_cgSinrWindow)
    {
        st.sinrDb.pop_front();
    }
    if (m_cgSinrPercentile <= 0.0 || st.sinrDb.size() < 2 || sinr <= 0.0)
    {
        return 0.0;
    }
    std::vector<double> v;
    v.reserve(st.sinrDb.size());
    for (const auto& sample : st.sinrDb)
    {
        v.push_back(sample.second);
    }
    std::sort(v.begin(), v.end());
    // the percentile, interpolated between the nearest ranks
    const double pos = m_cgSinrPercentile / 100.0 * static_cast<double>(v.size() - 1);
    const auto lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = std::min(lo + 1, v.size() - 1);
    const double pDb = v[lo] + (pos - static_cast<double>(lo)) * (v[hi] - v[lo]);
    return std::max(0.0, 10.0 * std::log10(sinr) - pDb);
}

int
NrMacSchedulerNs3::DlSpsOutOfDate(SpsState& st, const std::shared_ptr<NrMacSchedulerUeInfo>& ue) const
{
    // As CgOutOfDate() for the uplink: the MCS the link carries now against the configuration's
    // (SpsOutOfDate(), SpsMcsTolerance), held for a while before the gNB activates again
    const Time now = Simulator::Now();
    const int state = SpsOutOfDate(true, st, ue->GetDlMcs(), ue->m_dlRank);
    const bool worse = state < 0;
    const bool better = state > 0;
    st.worseSince = worse ? (st.worseSince == Time::Min() ? now : st.worseSince) : Time::Min();
    st.betterSince =
        better ? (st.betterSince == Time::Min() ? now : st.betterSince) : Time::Min();
    const int stepsWorse = static_cast<int>(st.mcs) - static_cast<int>(ue->GetDlMcs());
    const bool urgent =
        (m_dlSpsUrgentSteps > 0 && stepsWorse >= static_cast<int>(m_dlSpsUrgentSteps)) ||
        (m_dlSpsFailures > 0 && st.failRun >= m_dlSpsFailures && stepsWorse > 0);
    if (urgent)
    {
        return -1;
    }
    if (st.lastActivation != Time::Min() && now - st.lastActivation < m_dlSpsMinInterval)
    {
        return 0;
    }
    if (worse && now - st.worseSince >= m_dlSpsRaiseTime)
    {
        return -1;
    }
    if (better && now - st.betterSince >= m_dlSpsLowerTime)
    {
        return 1;
    }
    return 0;
}

int
NrMacSchedulerNs3::SpsOutOfDate(bool isDl, const SpsState& st, uint8_t mcs, uint8_t rank) const
{
    const Ptr<NrAmc>& amc = isDl ? m_dlAmc : m_ulAmc;
    const uint32_t nprb = st.numRbg * st.numSym * GetNumRbPerRbg();
    // What the link carries now in the resources of the block, as an MCS at the rank of the
    // configuration (the rank a UE reports may change while its link stays as good as it was)
    const uint32_t tbsNow = amc->CalculateTbSize(mcs, rank, nprb);
    int equivalent = 0;
    for (int m = amc->GetMaxMcs(); m > 0; --m)
    {
        if (amc->CalculateTbSize(static_cast<uint8_t>(m), st.rank, nprb) <= tbsNow)
        {
            equivalent = m;
            break;
        }
    }
    if (equivalent + static_cast<int>(m_spsMcsTolerance) < static_cast<int>(st.mcs))
    {
        return -1; // the link is worse: the blocks would fail too often
    }
    if (equivalent > static_cast<int>(st.mcs) + static_cast<int>(m_spsMcsTolerance))
    {
        // The link is better: worth a DCI if the block then takes fewer resources
        SpsBlock smaller;
        return SpsGeometry(isDl, mcs, rank, st.needTb, st.numSym, &smaller) &&
                       smaller.numSym * smaller.numRbg < st.numSym * st.numRbg
                   ? 1
                   : 0;
    }
    return 0;
}

void
NrMacSchedulerNs3::SpsPlanDl(uint64_t slot)
{
    NS_LOG_FUNCTION(this << slot);
    m_spsBlocksSlot.clear();
    m_spsReleasesSlot.clear();
    m_spsHidden.clear();
    m_spsHiddenSr.clear();
    m_spsPlanned.clear();
    NS_ABORT_MSG_IF(m_spsTbBytes == 0, "NrMacSchedulerNs3::SpsTbBytes is not set");
    NS_ABORT_MSG_IF(m_spsScenario == 2 && m_spsSilenceTbBytes == 0,
                    "NrMacSchedulerNs3::SpsSilenceTbBytes is not set");

    std::vector<uint16_t> rntis;
    rntis.reserve(m_ueMap.size());
    for (const auto& ue : m_ueMap)
    {
        rntis.push_back(ue.first);
    }
    std::sort(rntis.begin(), rntis.end());

    for (const uint16_t rnti : rntis)
    {
        const auto& ue = m_ueMap.at(rnti);
        SpsBlock block;
        block.rnti = rnti;
        NrMacSchedulerLC* lc = FindLcOfFiveQi(ue->m_dlLCG, m_spsFiveQi, &block.lcgId, &block.lcId);
        if (lc == nullptr)
        {
            continue;
        }
        SpsState& st = m_spsDl[rnti];
        // The MAC PDU that empties the queue: the RLC PDUs (the RLC reports them with their
        // headers) and one MAC subheader
        const uint32_t queue = lc->m_rlcTransmissionQueueSize;
        const uint32_t need = queue > 0 ? queue + MacSubheaderFor(queue) : 0;
        if (queue == 0)
        {
            // (a first packet after a change that is still to be sent, and is not there any
            // more: the RLC has discarded it)
            st.change = 0;
            st.pieces = 0;
        }
        // The configuration the packet that came last asks for, and the block it would have
        // (SpsDlSilenceAtSecondSid: with TS 26.093 DTX the first SID_UPDATE comes 3 frames
        // after SID_FIRST and then every 8: the silence configuration starts with it, on its
        // grid; SID_FIRST still goes on the talk configuration)
        const bool silent =
            st.lastKind == NrPdcp::SDU_SILENCE &&
            (!m_spsDlSilenceAtSecondSid ||
             NrPdcp::GetVoiceActivity(GetCellId(), rnti, block.lcId, true).run >= 2);
        const uint8_t wantConfig = m_spsScenario == 2 && silent ? 1 : 0;
        const uint32_t baseTb = wantConfig == 1 ? m_spsSilenceTbBytes : m_spsTbBytes;
        if (queue > 0)
        {
            NS_LOG_LOGIC("SPS DL slot " << slot << " RNTI " << rnti << " queue " << queue << " need "
                                        << need << " kind " << +st.lastKind << " config "
                                        << +st.config << " want " << +wantConfig << " active "
                                        << st.active << " usable " << st.usable << " start "
                                        << st.startSlot << " period " << st.periodSlots);
        }
        const uint32_t fitTb = baseTb + (m_spsScenario == 2 ? m_spsFitExtraBytes : 0);
        bool hide = false;

        if (st.releaseAgain)
        {
            SpsRelease release;
            release.rnti = rnti;
            release.config = st.releaseConfig;
            release.again = true;
            m_spsReleasesSlot.push_back(release);
        }

        if (!st.active)
        {
            // Nothing active: a packet that fits starts a configuration. One that does not
            // (the first of a call, with the whole header) goes the usual way, with a DCI; so
            // does what is no packet of the speaker at all (ROHC feedback for the other
            // direction of a one-way call).
            if (need > 0 && need <= fitTb && st.lastKind != NrPdcp::SDU_UNKNOWN &&
                SpsHasRoom(true,
                           rnti,
                           slot,
                           wantConfig,
                           std::max(baseTb, need),
                           ue->GetDlMcs(),
                           ue->m_dlRank))
            {
                block.activation = true;
                block.config = wantConfig;
                block.needTb = std::max(baseTb, need);
                m_spsBlocksSlot.push_back(block);
                hide = true;
                NS_LOG_LOGIC("SPS DL slot " << slot << " RNTI " << rnti << " activate config "
                                            << +wantConfig << " for " << need << " B");
            }
        }
        else if (m_spsScenario == 2 && need > 0 && st.lastKind != NrPdcp::SDU_UNKNOWN &&
                 wantConfig != st.config)
        {
            // Scenario 2, the speaker started or stopped talking: one DCI releases the active
            // configuration, a second activates the other, with this packet in its first block.
            // (A packet too large even for that goes the usual way; the next one changes over.
            // So does a packet that comes when the slots of the other configuration are full.)
            if (need <= fitTb && SpsHasRoom(true,
                                            rnti,
                                            slot,
                                            wantConfig,
                                            std::max(baseTb, need),
                                            ue->GetDlMcs(),
                                            ue->m_dlRank))
            {
                SpsRelease release;
                release.rnti = rnti;
                release.config = st.config;
                m_spsReleasesSlot.push_back(release);
                block.activation = true;
                block.config = wantConfig;
                block.needTb = std::max(baseTb, need);
                m_spsBlocksSlot.push_back(block);
                hide = true;
                NS_LOG_LOGIC("SPS DL slot " << slot << " RNTI " << rnti << " change config "
                                            << +st.config << " -> " << +wantConfig << " for "
                                            << need << " B");
            }
            else if (wantConfig == 1 && st.config == 0)
            {
                // The speaker fell silent and his first silence descriptor, with its ROHC
                // context update, is larger than the silence block: like scenario 1, in pieces
                // over the occasions of the talk configuration that is still active (a
                // descriptor is not urgent). What is left after this occasion fits the silence
                // block and changes over, with it in the first block of the silence
                // configuration (above, in the next slot). Every configuration keeps the block
                // of its steady state: no room is carried for the update (decision of 30 Oct
                // 2026).
                hide = true; // it waits for the occasion
                if (slot > st.startSlot && (slot - st.startSlot) % st.periodSlots == 0)
                {
                    block.config = st.config;
                    block.segment = true;
                    m_spsBlocksSlot.push_back(block);
                    NS_LOG_LOGIC("SPS DL slot " << slot << " RNTI " << rnti << " descriptor of "
                                                << need
                                                << " B in pieces on config 0 before the change");
                }
            }
        }
        else
        {
            const bool occasion =
                slot > st.startSlot && (slot - st.startSlot) % st.periodSlots == 0;
            if (st.dynamicUntilDrained && need <= st.usable)
            {
                st.dynamicUntilDrained = false;
            }
            if (st.dynamicUntilDrained)
            {
                // a large packet is on its way with DCIs of its own: the scheduler sees it
            }
            else if (!occasion)
            {
                hide = true; // it waits for its occasion
            }
            else if (need == 0)
            {
                m_pdcch->NoteEvent(NrPdcch::DL_SPS_EMPTY);
                hide = true;
            }
            else if (need <= st.usable ||
                     (st.lastKind == NrPdcp::SDU_SILENCE && st.config == 0))
            {
                // It fits; or it does not but is a silence descriptor on the talk
                // configuration: then in pieces, over this occasion and the next
                block.config = st.config;
                block.segment = need > st.usable;
                const int outOfDate = m_dlSpsHysteresis
                                          ? DlSpsOutOfDate(st, ue)
                                          : SpsOutOfDate(true, st, ue->GetDlMcs(), ue->m_dlRank);
                const bool room =
                    outOfDate != 0 &&
                    SpsHasRoom(true, rnti, slot, st.config, st.needTb, ue->GetDlMcs(), ue->m_dlRank);
                if (outOfDate < 0 && !room)
                {
                    // The link is worse, and the larger block it takes has no room in the
                    // slots of this configuration. A block as it is would fail: the
                    // configuration is left unused from here on, this packet gets a DCI, and
                    // a later one activates the configuration anew in a slot that has room
                    // (which re-initialises it, TS 38.321 5.8.1: no release is needed).
                    st.active = false;
                    NS_LOG_LOGIC("SPS DL slot " << slot << " RNTI " << rnti << " MCS "
                                                << +st.mcs << " -> " << +ue->GetDlMcs()
                                                << " and no room for the larger block: given up");
                }
                else
                {
                    if (room)
                    {
                        // The link of the UE has moved away from the MCS of the
                        // configuration: activate it again with the new one (if a PDCCH
                        // candidate is free)
                        block.activation = true;
                        block.reactivation = true;
                        block.worse = outOfDate < 0;
                        block.needTb = st.needTb;
                        NS_LOG_LOGIC("SPS DL slot "
                                     << slot << " RNTI " << rnti << " MCS " << +st.mcs << " rank "
                                     << +st.rank << " -> " << +ue->GetDlMcs() << " rank "
                                     << +ue->m_dlRank << ": activate again");
                    }
                    m_spsBlocksSlot.push_back(block);
                    hide = true;
                }
            }
            else
            {
                // A voice frame larger than the block (with a context update of ROHC): the
                // occasion is left out and the packet sent with a DCI of its own
                m_pdcch->NoteEvent(NrPdcch::DL_SPS_LARGE);
                st.dynamicUntilDrained = true;
                NS_LOG_LOGIC("SPS DL slot " << slot << " RNTI " << rnti << " packet of " << need
                                            << " B over the block of " << st.usable
                                            << " B: a DCI of its own");
            }
        }
        if (hide)
        {
            m_spsHidden.emplace_back(lc, queue);
            lc->m_rlcTransmissionQueueSize = 0;
        }
    }
}

uint8_t
NrMacSchedulerNs3::SpsVoiceChanges(SpsState& st,
                                   uint16_t rnti,
                                   uint8_t lcId,
                                   bool isDl,
                                   bool* packets,
                                   std::array<uint64_t, 2>* changes,
                                   std::array<uint64_t, 2>* pdus) const
{
    const NrPdcp::VoiceActivity now = NrPdcp::GetVoiceActivity(GetCellId(), rnti, lcId, isDl);
    std::array<uint64_t, 2> n{};
    std::array<uint64_t, 2> p{};
    *packets = false;
    if (st.voiceKnown)
    {
        *packets = now.sdus != st.voiceSdus;
        n = {now.toSilence - st.voiceToSilence, now.toVoice - st.voiceToVoice};
        p = {now.toSilencePdus - st.voiceToSilencePdus, now.toVoicePdus - st.voiceToVoicePdus};
        for (const bool toVoice : {false, true})
        {
            m_pdcch->NoteEvent(NrPdcch::ChangeEvent(isDl, toVoice, NrPdcch::CHANGE_SEEN),
                               n[toVoice]);
        }
    }
    st.voiceKnown = true;
    st.voiceSdus = now.sdus;
    st.voiceToSilence = now.toSilence;
    st.voiceToVoice = now.toVoice;
    st.voiceToSilencePdus = now.toSilencePdus;
    st.voiceToVoicePdus = now.toVoicePdus;
    if (changes)
    {
        *changes = n;
    }
    if (pdus)
    {
        *pdus = p;
    }
    return static_cast<uint8_t>((n[0] > 0 ? 1 : 0) | (n[1] > 0 ? 2 : 0));
}

void
NrMacSchedulerNs3::SpsNoteChange(bool isDl, uint8_t changes, NrPdcch::Change what, uint64_t n) const
{
    if (changes & 1)
    {
        m_pdcch->NoteEvent(NrPdcch::ChangeEvent(isDl, false, what), n);
    }
    if (changes & 2)
    {
        m_pdcch->NoteEvent(NrPdcch::ChangeEvent(isDl, true, what), n);
    }
}

void
NrMacSchedulerNs3::SpsBlockDone(SpsState& st, uint32_t serial) const
{
    if (st.releaseDci && serial >= st.releaseSerial)
    {
        // The UE acknowledges a release: with HARQ-ACK in downlink (TS 38.213 10.2), with the
        // confirmation MAC CE in uplink (TS 38.321 5.8.2). By the time a later block is over
        // that would be here; if the UE missed the DCI it is sent once more.
        st.releaseAgain = st.releaseDci->m_pdcchMissed;
        st.releaseDci = nullptr;
    }
}

void
NrMacSchedulerNs3::SpsPlanUl(uint64_t slot)
{
    NS_LOG_FUNCTION(this << slot);
    m_spsBlocksSlot.clear();
    m_spsReleasesSlot.clear();
    m_spsHidden.clear();
    m_spsHiddenSr.clear();
    m_spsPlanned.clear();
    NS_ABORT_MSG_IF(m_spsTbBytes == 0, "NrMacSchedulerNs3::SpsTbBytes is not set");
    NS_ABORT_MSG_IF(m_spsScenario == 2 && m_spsSilenceTbBytes == 0,
                    "NrMacSchedulerNs3::SpsSilenceTbBytes is not set");

    const int64_t slotNs = m_macSchedSapUser->GetSlotPeriod().GetNanoSeconds();
    const uint64_t talkPeriod = SpsPeriodSlots(0);
    const uint64_t silencePeriod = SpsPeriodSlots(1);
    // the first silence descriptor to the next (TS 26.093: SID_FIRST, SID_UPDATE 3 frames on)
    const uint64_t sidGap =
        m_spsSidGapMs > 0
            ? static_cast<uint64_t>(m_spsSidGapMs * MilliSeconds(1).GetNanoSeconds() /
                                    m_macSchedSapUser->GetSlotPeriod().GetNanoSeconds())
            : silencePeriod;
    // The slot that is on the air now: what the gNB knows, it knows up to here
    const auto nowSlot = static_cast<uint64_t>(Simulator::Now().GetNanoSeconds() / slotNs);
    // A scheduling request arrives one propagation delay and a slot or two after the packet it
    // is for, and a UE fills a block one propagation delay and a slot before the gNB receives
    // it. So a block received this many slots after the request came, and every period from
    // there, finds the packets of the bearer just arrived.
    const auto margin = static_cast<uint64_t>(MilliSeconds(2).GetNanoSeconds() / slotNs);
    m_ulSlotAtHand = slot;

    std::vector<uint16_t> rntis;
    rntis.reserve(m_ueMap.size());
    for (const auto& ue : m_ueMap)
    {
        rntis.push_back(ue.first);
    }
    std::sort(rntis.begin(), rntis.end());

    for (const uint16_t rnti : rntis)
    {
        const auto& ue = m_ueMap.at(rnti);
        SpsBlock block;
        block.rnti = rnti;
        NrMacSchedulerLC* lc = FindLcOfFiveQi(ue->m_ulLCG, m_spsFiveQi, &block.lcgId, &block.lcId);
        if (lc == nullptr)
        {
            continue;
        }
        SpsState& st = m_spsUl[rnti];
        st.cgMargin = m_cgHysteresis ? CgMarginDb(st, ue, slot) : 0.0;

        // The blocks the UE left unused: it had nothing to send, or it did not know of them
        for (auto sent = st.blocks.begin(); sent != st.blocks.end();)
        {
            if (!sent->dci->m_cgSkipped)
            {
                ++sent;
                continue;
            }
            const uint8_t id = sent->dci->m_harqProcess;
            if (ue->m_ulHarq.Get(id).m_active && ue->m_ulHarq.Get(id).m_dciElement == sent->dci)
            {
                if (m_diagUlHarq)
                {
                    std::ostringstream o;
                    o << "cg skipped rnti " << rnti << " p" << +id << " block slot " << sent->slot
                      << " rv " << +sent->dci->m_rv;
                    DiagLog(o.str());
                }
                ue->m_ulHarq.Erase(id, "SpsPlanUl (block skipped by the UE)");
            }
            const auto alloc = m_ulAllocationMap.find(sent->sfn);
            if (alloc != m_ulAllocationMap.end())
            {
                std::erase_if(alloc->second.m_ulAllocations, [&](const AllocElem& a) {
                    return a.m_rnti == rnti && a.m_symStart == sent->dci->m_symStart;
                });
            }
            // Its repetitions: nothing comes in their slots either (one PUSCH per UE and slot)
            for (const uint64_t repSfn : sent->repSfns)
            {
                const auto repAlloc = m_ulAllocationMap.find(repSfn);
                if (repAlloc != m_ulAllocationMap.end())
                {
                    std::erase_if(repAlloc->second.m_ulAllocations,
                                  [rnti](const AllocElem& a) { return a.m_rnti == rnti; });
                }
            }
            std::erase_if(m_ulRepetitions,
                          [&sent](const UlRepetition& r) { return r.dci == sent->dci; });
            if (!sent->dci->m_pdcchMissed)
            {
                m_pdcch->NoteEvent(NrPdcch::UL_CG_EMPTY);
            }
            else if (st.active && sent->serial == st.serial)
            {
                // The UE missed the DCI that activated the configuration. The gNB sees it:
                // the first block came without the MAC CE that confirms the activation
                // (TS 38.321 5.8.2), in fact without anything. The next request activates it.
                st.active = false;
                NS_LOG_LOGIC("SPS UL slot " << slot << " RNTI " << rnti
                                            << " did not confirm: configuration lost");
            }
            SpsBlockDone(st, sent->serial);
            sent = st.blocks.erase(sent);
        }

        // What the gNB knows of the UE: the kind of the last packet it received from it, the
        // bytes its last buffer status report left, and a scheduling request
        const NrPdcp::SduKind kind = NrPdcp::GetLastSduKind(GetCellId(), rnti, block.lcId, false);
        bool packets;
        std::array<uint64_t, 2> changes;
        std::array<uint64_t, 2> pdus;
        SpsVoiceChanges(st, rnti, block.lcId, false, &packets, &changes, &pdus);
        for (const bool toVoice : {false, true})
        {
            if (changes[toVoice] == 0)
            {
                continue;
            }
            // The speaker started or stopped talking: the gNB has the first packet that tells
            // it. How it came: with a grant the gNB gave since the packet before it (the rest
            // a block left, or the answer to a scheduling request; and so everything comes
            // while no configuration is active); or else in blocks of the configuration, in
            // as many RLC PDUs as the RLC received it in.
            const auto note = [&](NrPdcch::Change what, uint64_t n) {
                m_pdcch->NoteEvent(NrPdcch::ChangeEvent(false, toVoice, what), n);
            };
            // (CgSilenceSwitch: a silence seen now on the talk configuration starts the change at
            // once; a talk spurt cancels it)
            st.switchNow = !toVoice;
            if (st.gaveDci || !st.active)
            {
                note(NrPdcch::CHANGE_DCI, changes[toVoice]);
            }
            else if (pdus[toVoice] > changes[toVoice])
            {
                note(NrPdcch::CHANGE_PIECES, changes[toVoice]);
                note(NrPdcch::CHANGE_PIECE_PDUS, pdus[toVoice]);
            }
            else
            {
                note(NrPdcch::CHANGE_BLOCK, changes[toVoice]);
            }
        }
        if (packets)
        {
            st.gaveDci = false;
        }
        const uint32_t reported = lc->m_rlcTransmissionQueueSize;
        const bool asked = ue->m_srBytesPending > 0;
        const bool wants = reported > 0 || asked;
        if (asked)
        {
            st.switchNow = false; // the speaker talks again
        }
        if (kind == NrPdcp::SDU_UNKNOWN || st.heardSlot == 0)
        {
            st.heardSlot = nowSlot; // (stays where it is once the kind is known)
        }
        // A grant with a DCI is on its way for data a block left: until it has come in, with
        // its buffer status report, what the reports of blocks say is older than that grant
        const bool waiting = st.dynamicSlot != 0 && nowSlot <= st.dynamicSlot;
        st.seenReport = std::max(st.seenReport, reported);
        if (st.checkLeftover && nowSlot >= st.checkSlot)
        {
            // A block has come in, and by now the buffer status report that was in it
            st.checkLeftover = false;
            st.leftoverBytes = st.seenReport;
            st.seenReport = 0;
            // (a block that came before the grant with a DCI did: what it left, that grant
            // was for)
            const bool stale = st.dynamicSlot != 0 && st.checkBlock <= st.dynamicSlot;
            st.leftover = st.leftoverBytes > 0 && !waiting && !stale
                              ? static_cast<uint8_t>(std::min(st.leftover + 1, 255))
                              : 0;
            NS_LOG_LOGIC("SPS UL slot " << slot << " RNTI " << rnti << " block of slot "
                                        << st.checkBlock << " left " << st.leftoverBytes
                                        << " B, " << +st.leftover << " in a row, kind " << +kind
                                        << (waiting ? ", a grant is on its way" : "")
                                        << (stale ? ", older than the grant" : ""));
            if (st.leftover == 1 && kind == NrPdcp::SDU_TALK)
            {
                m_pdcch->NoteEvent(NrPdcch::UL_CG_SEGMENT);
            }
        }

        if (st.releaseAgain)
        {
            SpsRelease release;
            release.rnti = rnti;
            release.config = st.releaseConfig;
            release.again = true;
            m_spsReleasesSlot.push_back(release);
        }

        // A configuration starts, or takes over from the active one, with a block in this slot
        bool starts = false;
        uint8_t startConfig = 0;
        if (st.pending && st.pendingConfig == 1 && kind == NrPdcp::SDU_TALK)
        {
            // The silence configuration was to start, and the speaker talks: the next
            // scheduling request times the talk configuration instead
            st.pending = false;
        }
        if (m_cgSilenceSwitchNow && st.switchNow && st.active && m_spsScenario == 2 &&
            st.config == 0 && kind == NrPdcp::SDU_SILENCE && slot > st.lastUsedSlot &&
            st.lastUsedSlot >= st.startSlot)
        {
            // CgSilenceSwitch: the speaker has fallen silent and the gNB has his first silence
            // descriptor: the talk configuration is released now (its occasions are not kept),
            // and the silence configuration is activated when the next descriptor is due, one
            // period after the first, so that its occasions fall where the descriptors are (an
            // activation starts its occasions K2 + K_offset after the DCI: one sent now would
            // put them a period out of step). If the speaker talks before that, the scheduling
            // request times the talk configuration instead.
            st.switchNow = false;
            SpsRelease release;
            release.rnti = rnti;
            release.config = st.config;
            m_spsReleasesSlot.push_back(release);
            st.active = false;
            st.pending = true;
            st.pendingConfig = 1;
            st.pendingSlot = st.lastUsedSlot + sidGap;
            NS_LOG_LOGIC("SPS UL slot " << slot << " RNTI " << rnti
                                        << " silent: release config 0 now, config 1 at slot "
                                        << st.pendingSlot);
        }
        if (st.pending && slot >= st.pendingSlot && slot >= st.startRetrySlot)
        {
            // (from the slot it was timed for on: if its DCIs find no PDCCH candidate there,
            // in the next)
            starts = true;
            startConfig = st.pendingConfig;
        }
        else if (st.active && m_spsScenario == 2 && st.config == 0 &&
                 kind == NrPdcp::SDU_SILENCE && slot > st.lastUsedSlot &&
                 st.lastUsedSlot >= st.startSlot &&
                 (slot - st.lastUsedSlot) % sidGap < margin + 2)
        {
            // The speaker has fallen silent: his first silence descriptor came in the block of
            // lastUsedSlot. The next is due one silence period later, and there the silence
            // configuration takes over (or within the few slots after it, if its DCIs find no
            // PDCCH candidate); if he talks before that, nothing changes. (A block of this
            // talk configuration, that is: right after it has taken over, the last packet the
            // gNB has is still the silence descriptor that came before the talk spurt.)
            starts = true;
            startConfig = 1;
            st.pending = true; // (so that it goes on in the next slot if only the release fits)
            st.pendingConfig = 1;
            st.pendingSlot = slot;
        }
        // (the block an activation here would have: with room for the confirmation unless it
        // was sent ahead; the room is checked for that size, the block below gets it)
        const uint32_t startTb =
            st.aheadSent ? SpsUlTbBytes(startConfig) : SpsUlTbBytesWithCe(startConfig);
        const auto [ulMcs, ulReps] =
            SpsUlLink(ue, starts ? startTb : (st.active ? st.needTb : 0), st.cgMargin, 0, st.cgMargin);
        if (starts && !SpsHasRoom(false,
                                  rnti,
                                  slot,
                                  startConfig,
                                  startTb,
                                  ulMcs,
                                  ue->m_ulRank,
                                  ulReps))
        {
            starts = false; // the slots it would come back in are full: in a later slot
        }
        if (starts)
        {
            st.switchNow = false; // (a silence seen before this configuration is over with)
            // (the same configuration in other slots is activated again there, which
            // re-initialises it, TS 38.321 5.8.2; only another configuration takes a release)
            if (st.active && st.config != startConfig)
            {
                SpsRelease release;
                release.rnti = rnti;
                release.config = st.config;
                m_spsReleasesSlot.push_back(release);
            }
            block.activation = true;
            block.config = startConfig;
            // (an activation no grant before it confirms ahead: room for the confirmation)
            block.needTb = startTb;
            m_spsBlocksSlot.push_back(block);
            NS_LOG_LOGIC("SPS UL slot " << slot << " RNTI " << rnti << " "
                                        << (st.active ? "change to" : "activate") << " config "
                                        << +startConfig << " of " << block.needTb << " B, MCS "
                                        << +ulMcs);
        }

        // What the UE asks for: answered by a block of its configuration, or by a DCI?
        bool byDci = true;
        if (st.active)
        {
            // The first occasion the UE can still use is the first that is not over
            const uint64_t toNext =
                nowSlot <= st.startSlot
                    ? st.startSlot - nowSlot
                    : (st.periodSlots - (nowSlot - st.startSlot) % st.periodSlots) % st.periodSlots;
            // (unless its block found no room in its slot, which the start of a configuration
            // makes sure of but a random access or a sounding signal in the slot can undo)
            const bool hole = nowSlot + toNext == st.holeSlot;
            const bool covered = toNext < talkPeriod && !hole;

            if (st.dynamicUntilDrained && !wants)
            {
                st.dynamicUntilDrained = false;
            }
            if (!waiting && !st.dynamicUntilDrained && st.leftover > 0 && !st.checkLeftover &&
                (kind == NrPdcp::SDU_SILENCE || st.leftover >= 2))
            {
                // Data left after a block. The speaker was silent: this is a voice frame,
                // larger than the block with its context update, and it gets a DCI of its
                // own. He was talking: it was taken for a silence descriptor and left to
                // the next occasion, and now there is data left again. (Not while the report
                // of a later block is awaited: that block may have brought the rest, as the
                // one after the first piece of a silence descriptor does, and with it the
                // gNB sees that the speaker is silent.)
                st.dynamicUntilDrained = true;
                st.leftover = 0;
                lc->m_rlcTransmissionQueueSize =
                    std::max(lc->m_rlcTransmissionQueueSize, st.leftoverBytes);
                m_pdcch->NoteEvent(NrPdcch::UL_CG_LARGE);
                st.gaveDci = true;
                NS_LOG_LOGIC("SPS UL slot " << slot << " RNTI " << rnti << " has "
                                            << st.leftoverBytes
                                            << " B left after its block: a DCI of its own");
            }
            if (m_spsScenario == 2 && st.config == 1 && !covered && !hole && !st.pending &&
                (asked || kind == NrPdcp::SDU_TALK))
            {
                // Silence, and a scheduling request away from the occasions: a silence
                // descriptor would have come with one, so the speaker talks again (or the
                // gNB has a voice frame of his already). The request is answered with a DCI,
                // and the talk configuration takes over with the next packet.
                st.pending = true;
                st.pendingConfig = 0;
                st.gaveDci = true;
                st.pendingSlot = slot + (talkPeriod + margin - (slot - nowSlot) % talkPeriod) %
                                            talkPeriod;
                ue->m_srBytesPending =
                    std::max(ue->m_srBytesPending,
                             m_spsTbBytes + m_spsFitExtraBytes + UlBsrBytes());
                if (m_cgBlockPacketOnly)
                {
                    // The talk spurt starts: the grant that answers the request carries the
                    // confirmation of the release of the silence configuration and the
                    // activation of the talk one (one Multiple Entry CE confirms both)
                    ue->m_srConfirmAhead = static_cast<uint8_t>(m_cgMultiEntryCeBytes);
                    ue->m_srConfirmAheadSerial = st.serial + 1;
                    st.aheadSent = true;
                    ue->m_srBytesPending = std::max(LearnedSrGrant(rnti), ue->m_srBytesPending) +
                                           m_cgMultiEntryCeBytes;
                }
            }
            else if (!st.dynamicUntilDrained && !st.pending && covered && wants)
            {
                // The next block carries it. What is left after it, the buffer status report
                // in that block will say.
                byDci = false;
                lc->m_rlcTransmissionQueueSize = 0;
                ue->m_srBytesPending = 0;
            }

            if (!starts && nowSlot < slot && slot > st.startSlot &&
                (slot - st.startSlot) % st.periodSlots == 0)
            {
                // The block of the occasion: it comes whether or not the UE has something
                // to send, which the gNB cannot know
                block.config = st.config;
                // With repetitions: more of them, or (with 1 dB to spare) fewer; else the MCS
                // for as many as the configuration has
                int outOfDate = 0;
                if (!m_cgHysteresis)
                {
                    if (ulReps > st.reps)
                    {
                        outOfDate = -1;
                    }
                    else if (ulReps < st.reps && SpsUlLink(ue, st.needTb, 1.0).second < st.reps)
                    {
                        outOfDate = 1;
                    }
                    else
                    {
                        const uint8_t mcsNow =
                            ulReps == st.reps ? ulMcs : SpsUlLink(ue, 0, 0.0, st.reps).first;
                        outOfDate = SpsOutOfDate(false, st, mcsNow, ue->m_ulRank);
                    }
                }
                else
                {
                    outOfDate = CgOutOfDate(st, ue);
                }
                if (outOfDate != 0 && SpsHasRoom(false,
                                                 rnti,
                                                 slot,
                                                 st.config,
                                                 std::max(st.needTb, SpsUlTbBytesWithCe(st.config)),
                                                 ulMcs,
                                                 ue->m_ulRank,
                                                 ulReps))
                {
                    block.activation = true;
                    block.reactivation = true;
                    block.worse = outOfDate < 0;
                    block.needTb = std::max(st.needTb, SpsUlTbBytesWithCe(st.config));
                    NS_LOG_LOGIC("SPS UL slot " << slot << " RNTI " << rnti << " MCS "
                                                << +st.mcs << " -> " << +ulMcs
                                                << ": activate again");
                }
                else if (outOfDate < 0)
                {
                    // The link is worse and the larger block it takes has no room in the slots
                    // of this configuration: it ends here (the UE would go on using it) and
                    // starts anew in the first slot that has room. Until then what the UE asks
                    // for is answered with DCIs.
                    SpsRelease release;
                    release.rnti = rnti;
                    release.config = st.config;
                    release.alone = true;
                    m_spsReleasesSlot.push_back(release);
                    if (!st.pending)
                    {
                        st.pending = true;
                        st.pendingConfig = st.config;
                        st.pendingSlot = slot + 1;
                    }
                    NS_LOG_LOGIC("SPS UL slot " << slot << " RNTI " << rnti << " MCS "
                                                << +st.mcs << " -> " << +ulMcs
                                                << " and no room for the larger block: release");
                }
                m_spsBlocksSlot.push_back(block);
            }
        }
        else if (asked && kind != NrPdcp::SDU_UNKNOWN && !st.pending &&
                 !(m_cgStartAfterIr &&
                   NrPdcp::GetVoiceActivity(GetCellId(), rnti, block.lcId, false).lastIr) &&
                 ((reported == 0 && ue->m_ulHarq.Size() == 0) ||
                  nowSlot > st.heardSlot + 10 * talkPeriod))
        {
            // Nothing active, and a scheduling request of a UE whose voice the gNB has heard,
            // whose buffer was empty and that has no grant on its way: a packet has just come
            // to it. The request is answered with a DCI, and a configuration starts with the
            // next packet. (Before that, the first packets of a call, there are only DCIs.
            // A UE that is never seen with an empty buffer gets its configuration all the
            // same, ten periods after its voice was first heard.)
            st.pending = true;
            st.gaveDci = true;
            st.pendingConfig = m_spsScenario == 2 && kind == NrPdcp::SDU_SILENCE ? 1 : 0;
            // The first block: margin slots after the request came in, plus as many periods of
            // the configuration as it takes to be a slot that can still be scheduled
            const uint64_t period = st.pendingConfig == 1 ? silencePeriod : talkPeriod;
            st.pendingSlot = slot + (period + margin - (slot - nowSlot) % period) % period;
            ue->m_srBytesPending =
                std::max(ue->m_srBytesPending, m_spsTbBytes + m_spsFitExtraBytes + UlBsrBytes());
            if (m_cgBlockPacketOnly && st.pendingConfig == 0)
            {
                // the grant that answers the request carries the confirmation
                ue->m_srConfirmAhead =
                    static_cast<uint8_t>(m_spsScenario == 2 ? m_cgMultiEntryCeBytes : 1U);
                ue->m_srConfirmAheadSerial = st.serial + 1;
                st.aheadSent = true;
                // (the grant above: the larger of the learned one and a talk block, + the CE)
                ue->m_srBytesPending += ue->m_srConfirmAhead;
            }
        }

        // Out of sight of the scheduler for this slot: a UE that gets a block in it (one
        // transport block per slot), and one whose data waits for a DCI already sent
        const bool blockNow = !m_spsBlocksSlot.empty() && m_spsBlocksSlot.back().rnti == rnti;
        if (blockNow || waiting || !byDci)
        {
            m_spsHidden.emplace_back(lc, lc->m_rlcTransmissionQueueSize);
            lc->m_rlcTransmissionQueueSize = 0;
            m_spsHiddenSr.emplace_back(ue, ue->m_srBytesPending);
            ue->m_srBytesPending = 0;
        }
    }
}

void
NrMacSchedulerNs3::SpsAdmit(bool isDl)
{
    if (m_spsScenario == 0 || !(isDl ? m_spsDlOn : m_spsUlOn))
    {
        return;
    }
    const auto releasePurpose = isDl ? NrPdcch::DL_SPS_RELEASE : NrPdcch::UL_CG_RELEASE;
    const auto activationPurpose = isDl ? NrPdcch::DL_SPS_ACTIVATION : NrPdcch::UL_CG_ACTIVATION;
    for (auto& release : m_spsReleasesSlot)
    {
        if (release.again || release.alone)
        {
            release.grant = m_pdcch->Request(release.rnti, releasePurpose);
        }
    }
    for (auto& block : m_spsBlocksSlot)
    {
        if (!block.activation)
        {
            continue;
        }
        // A change of configuration is two DCIs: the release first, then the activation. If
        // the PDCCH of this slot has room for the release only (a UE at level 4 in a CORESET
        // of 4 CCEs), the activation follows in a later slot.
        const auto release = std::find_if(m_spsReleasesSlot.begin(),
                                          m_spsReleasesSlot.end(),
                                          [&block](const SpsRelease& r) {
                                              return r.rnti == block.rnti && !r.again && !r.alone;
                                          });
        if (release != m_spsReleasesSlot.end())
        {
            release->grant = m_pdcch->Request(release->rnti, releasePurpose);
            if (!release->grant)
            {
                continue;
            }
        }
        block.grant = m_pdcch->Request(block.rnti, activationPurpose);
    }
}

uint8_t
NrMacSchedulerNs3::SpsAllocate(uint64_t slot,
                               uint64_t sfn,
                               bool isDl,
                               uint8_t firstSym,
                               uint8_t symAvail,
                               SlotAllocInfo* allocInfo)
{
    NS_LOG_FUNCTION(this << slot << isDl);
    auto& states = isDl ? m_spsDl : m_spsUl;
    const auto format = isDl ? DciInfoElementTdma::DL : DciInfoElementTdma::UL;
    const auto confirmBytes =
        static_cast<uint8_t>(isDl ? 0 : (m_spsScenario == 2 ? m_cgMultiEntryCeBytes : 1U));

    // The blocks: their shape. A block of an active configuration has the one its activation
    // gave it; a block that comes with an activation gets the smallest that holds its bytes.
    std::vector<SpsBlock*> blocks;
    for (auto& block : m_spsBlocksSlot)
    {
        const auto& ue = m_ueMap.at(block.rnti);
        SpsState& st = states[block.rnti];
        if (block.activation && block.grant)
        {
            const auto ulLink = isDl ? std::pair<uint8_t, uint8_t>{0, 1}
                                     : SpsUlLink(ue, block.needTb, st.cgMargin, 0, st.cgMargin);
            const uint8_t mcs = isDl ? ue->GetDlMcs() : ulLink.first;
            const uint8_t rank = isDl ? ue->m_dlRank : ue->m_ulRank;
            block.reps = ulLink.second;
            if (!SpsGeometry(isDl, mcs, rank, block.needTb, symAvail, &block))
            {
                m_pdcch->Release(block.grant);
                block.grant = NrPdcch::Grant();
            }
        }
        if (block.activation && !block.grant)
        {
            if (!block.reactivation)
            {
                // No PDCCH candidate, or no room for such a block: not this slot. The UE was
                // kept from the DCIs of this slot for it; after two such slots in a row it is
                // not tried again for a talk period, and what it asks for is answered with DCIs
                // (before: tried, and the UE kept from DCIs, slot after slot)
                if (!isDl && m_cgStartRetry && ++st.startFails >= 2)
                {
                    st.startFails = 0;
                    st.startRetrySlot = slot + std::max<uint64_t>(st.periodSlots, 20);
                    NS_LOG_LOGIC("SPS UL slot " << slot << " RNTI " << block.rnti
                                                << " activation not placed twice: DCIs until slot "
                                                << st.startRetrySlot);
                }
                continue;
            }
            block.activation = false; // the configuration stays as it was
            if (block.worse && isDl)
            {
                // ... unless the link is worse: its block would fail. It is left unused from
                // here on; the packet gets a DCI in the next slot, and a later one activates
                // the configuration anew in a slot whose PDCCH has room.
                st.active = false;
                NS_LOG_LOGIC("SPS DL slot " << slot << " RNTI " << block.rnti
                                            << " not activated again, no PDCCH candidate: given up");
                continue;
            }
            if (block.worse && !st.pending)
            {
                // In uplink the UE goes on using it. It is activated again in the first of
                // the next slots that has room and a PDCCH candidate, with its blocks from
                // there; until then what the UE asks for is answered with DCIs.
                st.pending = true;
                st.pendingConfig = st.config;
                st.pendingSlot = slot + 1;
                NS_LOG_LOGIC("SPS UL slot " << slot << " RNTI " << block.rnti
                                            << " not activated again, no PDCCH candidate: "
                                               "in a later slot");
            }
        }
        if (!block.activation)
        {
            if (!st.active)
            {
                continue;
            }
            block.numSym = st.numSym;
            block.numRbg = st.numRbg;
            block.mcs = st.mcs;
            block.rank = st.rank;
            block.reps = st.reps;
            block.tbs = st.tbs;
            block.usable = st.usable;
        }
        blocks.push_back(&block);
    }

    // Rows of blocks of one height and one beam, side by side, the highest rows first
    std::stable_sort(blocks.begin(), blocks.end(), [](const SpsBlock* a, const SpsBlock* b) {
        return a->numSym > b->numSym;
    });
    const auto mask = isDl ? GetDlBitmask() : GetUlBitmask();
    std::vector<uint32_t> rbgs;
    for (uint32_t i = 0; i < mask.size(); ++i)
    {
        if (mask[i])
        {
            rbgs.push_back(i);
        }
    }
    uint8_t used = 0;
    uint8_t rowSym = 0;
    uint8_t rowStart = 0;
    BeamId rowBeam;
    uint32_t x = 0;
    std::set<uint16_t> activated;
    for (SpsBlock* block : blocks)
    {
        const auto& ue = m_ueMap.at(block->rnti);
        SpsState& st = states[block->rnti];
        auto& harq = isDl ? ue->m_dlHarq : ue->m_ulHarq;
        const bool newRow = rowSym != block->numSym || x + block->numRbg > rbgs.size() ||
                            rowBeam != ue->m_beamId;
        if ((newRow && used + block->numSym > symAvail) || !harq.CanInsert())
        {
            // The slot is full, or the UE has no HARQ process left: not this time
            if (block->activation)
            {
                m_pdcch->Release(block->grant);
            }
            else if (!isDl)
            {
                // The UE does not get the block it may count on: what it asks for from now on
                // is answered with a DCI, and if a request was left to this block, it is too
                st.holeSlot = slot;
                for (auto& hidden : m_spsHiddenSr)
                {
                    if (hidden.first == ue)
                    {
                        hidden.second = std::max(hidden.second, block->usable);
                    }
                }
            }
            NS_LOG_LOGIC("SPS " << (isDl ? "DL" : "UL") << " slot " << slot << " RNTI "
                                << block->rnti << " block not placed: "
                                << (harq.CanInsert() ? "no symbols left" : "no HARQ process"));
            continue;
        }
        if (newRow)
        {
            used += block->numSym;
            rowSym = block->numSym;
            rowStart = isDl ? firstSym + used - rowSym : firstSym - used;
            rowBeam = ue->m_beamId;
            x = 0;
        }
        std::vector<bool> rbgBitmask(mask.size(), false);
        for (uint32_t i = 0; i < block->numRbg; ++i)
        {
            rbgBitmask[rbgs[x + i]] = true;
        }
        x += block->numRbg;

        auto dci =
            std::make_shared<DciInfoElementTdma>(block->rnti,
                                                 format,
                                                 rowStart,
                                                 block->numSym,
                                                 block->mcs,
                                                 block->rank,
                                                 isDl ? ue->m_dlPrecMats : ue->m_ulPrecMats,
                                                 block->tbs,
                                                 1,
                                                 0,
                                                 DciInfoElementTdma::DATA,
                                                 GetBwpId(),
                                                 GetTpc());
        dci->m_rbgBitmask = rbgBitmask;
        dci->m_sps =
            block->activation ? DciInfoElementTdma::SPS_ACTIVATION : DciInfoElementTdma::SPS_OCCASION;
        dci->m_spsConfig = block->config;
        dci->m_spsPeriodSlots = static_cast<uint16_t>(SpsPeriodSlots(block->config));
        dci->m_spsTbBytes = block->usable;
        dci->m_cgConfirmBytes = block->activation ? confirmBytes : 0;
        dci->m_cgSerial = block->activation ? states[block->rnti].serial + 1 : 0;
        // (uplink: the further transmissions of the block, PlaceUlRepetitions())
        dci->m_repetitions = block->reps;

        HarqProcess harqProcess(true, HarqProcess::WAITING_FEEDBACK, 0, dci);
        harqProcess.m_txSlot = slot;
        uint8_t id;
        harq.Insert(&id, harqProcess);
        harq.Get(id).m_dciElement->m_harqProcess = id;
        if (!isDl)
        {
            // (whether the block's MCS comes from a measurement of the UE's uplink, for the
            // UL OLLA, as for a dynamic grant; a stale flag of an earlier block on this id
            // moved the OLLA before 1 Nov 2026)
            if (ue->m_ulSinrMeas.empty())
            {
                m_ulMeasuredHarq.erase({ue->m_rnti, id});
            }
            else
            {
                m_ulMeasuredHarq.insert({ue->m_rnti, id});
            }
        }
        if (!isDl)
        if (m_diagUlHarq)
        {
            std::ostringstream o;
            o << "cg block rnti " << ue->m_rnti << " p" << +id << " reps " << +dci->m_repetitions
              << " for slot " << slot << (block->activation ? " (activation)" : "");
            DiagLog(o.str());
        }

        VarTtiAllocInfo slotInfo(dci);
        if (isDl)
        {
            // The whole block is for the bearer. Its queue is out of sight (SpsPlanDl): bring
            // it back for the bookkeeping and hide what is left of it again.
            NrMacSchedulerLC* lc = ue->m_dlLCG.at(block->lcgId)->PeekLC(block->lcId);
            const auto hidden = std::find_if(m_spsHidden.begin(),
                                             m_spsHidden.end(),
                                             [lc](const auto& h) { return h.first == lc; });
            NS_ASSERT(hidden != m_spsHidden.end());
            lc->m_rlcTransmissionQueueSize = hidden->second;
            const uint32_t bytes = MacSduIn(block->usable);
            RlcPduInfo rlcPdu(block->lcId, bytes);
            slotInfo.m_rlcPduInfo.push_back(rlcPdu);
            harq.Get(id).m_rlcPduInfo.push_back(rlcPdu);
            ue->m_dlLCG.at(block->lcgId)->AssignedData(block->lcId, bytes, "DL");
            hidden->second = lc->m_rlcTransmissionQueueSize;
            lc->m_rlcTransmissionQueueSize = 0;
            allocInfo->m_varTtiAllocInfo.emplace_back(slotInfo);
            if (m_dlHarqFeedbackDisabled)
            {
                harq.Erase(id, "SpsAllocate DL (feedback disabled)");
            }
        }
        else
        {
            allocInfo->m_varTtiAllocInfo.emplace_front(slotInfo);
        }

        if (block->activation)
        {
            m_pdcch->Commit(block->grant,
                            isDl ? NrPdcch::DL_SPS_ACTIVATION : NrPdcch::UL_CG_ACTIVATION,
                            dci);
            st.active = true;
            st.config = block->config;
            ++st.serial;
            st.startSlot = slot;
            st.lastActivation = Simulator::Now();
            st.aheadSent = false;
            st.worseSince = Time::Min();
            st.betterSince = Time::Min();
            st.failRun = 0;
            st.periodSlots = SpsPeriodSlots(block->config);
            st.needTb = block->needTb;
            st.numSym = block->numSym;
            st.numRbg = block->numRbg;
            st.mcs = block->mcs;
            st.rank = block->rank;
            st.reps = block->reps;
            st.tbs = block->tbs;
            st.usable = block->usable;
            st.dynamicUntilDrained = false;
            st.leftover = 0;
            st.pending = false;
            if (st.switchFrom >= 0)
            {
                // The other configuration was released in an earlier slot: the change of
                // configuration is complete with this DCI
                if (st.switchFrom != block->config)
                {
                    m_pdcch->NoteEvent(
                        NrPdcch::ChangeEvent(isDl, block->config == 0, NrPdcch::CHANGE_SWITCH));
                }
                st.switchFrom = -1;
            }
            activated.insert(block->rnti);
            st.startFails = 0;
            (isDl ? m_dlPdcchLast : m_ulPdcchLast)[block->rnti] = m_pdcchRound;
        }
        else if (isDl)
        {
            m_pdcch->NoteEvent(NrPdcch::DL_SPS_BLOCK);
            if (block->segment)
            {
                m_pdcch->NoteEvent(NrPdcch::DL_SPS_SEGMENT);
            }
        }
        if (isDl)
        {
            // The first packet after the speaker started or stopped talking, if it goes with
            // this block: whole (so in every first block of a configuration, which is made for
            // it) or a first piece of it; and what follows of one that went in pieces
            SpsNoteChange(true, st.pieces, NrPdcch::CHANGE_PIECE_PDUS);
            if (!block->segment)
            {
                st.pieces = 0;
            }
            if (block->segment)
            {
                SpsNoteChange(true, st.change, NrPdcch::CHANGE_PIECES);
                SpsNoteChange(true, st.change, NrPdcch::CHANGE_PIECE_PDUS);
                st.pieces |= st.change;
            }
            else
            {
                SpsNoteChange(true, st.change, NrPdcch::CHANGE_BLOCK);
            }
            st.change = 0;
        }
        dci->m_spsSerial = st.serial;
        // (a block that was being retransmitted on this HARQ process is over: the process is
        // free again)
        std::erase_if(st.blocks, [id](const auto& b) { return b.retx && b.harqId == id; });
        SpsState::Sent sent;
        sent.dci = dci;
        sent.harqId = id;
        sent.serial = st.serial;
        sent.slot = slot;
        sent.sfn = sfn;
        st.blocks.push_back(sent);
        if (st.blocks.size() > 64)
        {
            st.blocks.pop_front();
        }
    }

    // The releases: a DCI and nothing else (TS 38.213 10.2)
    for (const auto& release : m_spsReleasesSlot)
    {
        if (!release.grant)
        {
            continue;
        }
        auto dci = std::make_shared<DciInfoElementTdma>(release.rnti,
                                                        format,
                                                        0,
                                                        0,
                                                        0,
                                                        1,
                                                        nullptr,
                                                        0,
                                                        1,
                                                        0,
                                                        DciInfoElementTdma::DATA,
                                                        GetBwpId(),
                                                        GetTpc());
        dci->m_sps = DciInfoElementTdma::SPS_RELEASE;
        dci->m_spsConfig = release.config;
        dci->m_cgSerial = m_spsUl.count(release.rnti) ? m_spsUl.at(release.rnti).serial : 0;
        dci->m_cgConfirmBytes = confirmBytes;
        allocInfo->m_dciOnly.push_back(dci);
        m_pdcch->Commit(release.grant,
                        isDl ? NrPdcch::DL_SPS_RELEASE : NrPdcch::UL_CG_RELEASE,
                        dci);
        // Whether the UE got it shows when a later block is over: of the configuration
        // activated with it or after it, or (a repetition) of the one that is active now
        SpsState& st = states[release.rnti];
        const bool alone = !release.again && !activated.contains(release.rnti);
        if (alone && st.active && st.config == release.config)
        {
            st.active = false; // the activation of the other configuration is still to come
        }
        if (alone)
        {
            st.switchFrom = static_cast<int8_t>(release.config);
        }
        else if (!release.again && st.config != release.config)
        {
            // Released, and the other configuration activated in the same slot: a change of
            // configuration, two DCIs
            m_pdcch->NoteEvent(NrPdcch::ChangeEvent(isDl, st.config == 0, NrPdcch::CHANGE_SWITCH));
        }
        st.releaseDci = dci;
        st.releaseConfig = release.config;
        st.releaseAgain = false;
        st.releaseSerial = alone ? st.serial + 1 : st.serial;
    }
    allocInfo->m_numSymAlloc += used;
    return used;
}

void
NrMacSchedulerNs3::SpsUnhide()
{
    for (const auto& hidden : m_spsHidden)
    {
        hidden.first->m_rlcTransmissionQueueSize = hidden.second;
    }
    m_spsHidden.clear();
    for (const auto& hidden : m_spsHiddenSr)
    {
        hidden.first->m_srBytesPending = hidden.second;
    }
    m_spsHiddenSr.clear();
}

void
NrMacSchedulerNs3::SpsFeedback(uint16_t rnti,
                               bool isDl,
                               const HarqProcess& process,
                               bool ok,
                               bool last) const
{
    auto& states = isDl ? m_spsDl : m_spsUl;
    const auto it = states.find(rnti);
    if (it == states.end())
    {
        return;
    }
    SpsState& st = it->second;
    const auto& dci = process.m_dciElement;
    // The block: its own feedback, or that of a retransmission of it (which has a DCI of its
    // own, on the HARQ process of the block)
    auto sent = std::find_if(st.blocks.begin(), st.blocks.end(), [&dci](const auto& b) {
        return b.dci == dci;
    });
    if (sent == st.blocks.end())
    {
        sent = std::find_if(st.blocks.begin(), st.blocks.end(), [&dci](const auto& b) {
            return b.retx && b.harqId == dci->m_harqProcess;
        });
    }
    if (sent == st.blocks.end())
    {
        return; // not a block of a configuration
    }
    const bool first = !sent->retx;
    const bool occasion = sent->dci->m_sps == DciInfoElementTdma::SPS_OCCASION;
    const uint32_t serial = sent->serial;
    const uint64_t slot = sent->slot;
    if (last)
    {
        st.blocks.erase(sent);
    }
    else
    {
        sent->retx = true; // not received: retransmissions follow
    }
    if (first && dci->m_pdcchMissed && st.active && serial == st.serial)
    {
        // The UE knew nothing of this block: it had missed the DCI that activated the
        // configuration. The gNB sees it by the HARQ feedback that does not come and starts
        // over: the next packet activates the configuration again.
        st.active = false;
        NS_LOG_LOGIC("SPS " << (isDl ? "DL" : "UL") << " RNTI " << rnti
                            << " did not know of its block: configuration lost");
    }
    if (!isDl)
    {
        if (first)
        {
            // The UE sent something in this block
            st.lastUsedSlot = slot;
            if (occasion)
            {
                m_pdcch->NoteEvent(NrPdcch::UL_CG_BLOCK);
            }
        }
        if (ok && slot >= st.checkBlock)
        {
            // The gNB has the block, at once or after retransmissions, and with it the buffer
            // status report of the UE, which the MAC hands on a slot after this feedback if
            // the block ended late in its slot. (A block that is not received says nothing of
            // what it left, and what the blocks before it left stands; so does one that
            // comes, retransmitted, after a later block.)
            st.checkLeftover = true;
            st.checkBlock = slot;
            st.seenReport = 0;
            st.checkSlot =
                static_cast<uint64_t>(Simulator::Now().GetNanoSeconds() /
                                      m_macSchedSapUser->GetSlotPeriod().GetNanoSeconds()) +
                2;
        }
    }
    if (first)
    {
        SpsBlockDone(st, serial);
    }
}

uint16_t
NrMacSchedulerNs3::GetBwpId() const
{
    if (m_macSchedSapUser)
    {
        return m_macSchedSapUser->GetBwpId();
    }
    else
    {
        return UINT16_MAX;
    }
}

uint16_t
NrMacSchedulerNs3::GetCellId() const
{
    if (m_macSchedSapUser)
    {
        return m_macSchedSapUser->GetCellId();
    }
    else
    {
        return UINT16_MAX;
    }
}

uint16_t
NrMacSchedulerNs3::GetBandwidthInRbg() const
{
    return m_bandwidth;
}

uint8_t
NrMacSchedulerNs3::GetFhControlMethod() const
{
    NS_LOG_FUNCTION(this);
    if (m_nrFhSchedSapProvider)
    {
        return m_nrFhSchedSapProvider->GetFhControlMethod();
    }
    else
    {
        return UINT8_MAX;
    }
}

bool
NrMacSchedulerNs3::DoesFhAllocationFit(uint16_t bwpId,
                                       uint32_t mcs,
                                       uint32_t nRegs,
                                       uint8_t dlRank,
                                       uint8_t numSym) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_nrFhSchedSapProvider);
    return m_nrFhSchedSapProvider->DoesAllocationFit(bwpId, mcs, nRegs, dlRank, numSym);
}

/**
 * @brief Call to the centralized intelligence (which stores a map of the active
 * UEs and cells with the bytes in their queues) to update the maps based on the
 * allocations.
 * @param allocation the list of allocations
 */
void
NrMacSchedulerNs3::CallNrFhControlForMapUpdate(
    const std::deque<VarTtiAllocInfo>& allocation,
    const std::unordered_map<uint16_t, std::shared_ptr<NrMacSchedulerUeInfo>>& ueMap)
{
    m_nrFhSchedSapProvider->UpdateActiveUesMap(GetBwpId(), allocation, ueMap);
}

void
NrMacSchedulerNs3::LogUesWithPendingDlSignalingTraffic(const ActiveUeMap& activeDlUe) const
{
    NS_LOG_FUNCTION(this);
    for (const auto& beamEntry : activeDlUe)
    {
        for (const auto& ueEntry : beamEntry.second)
        {
            const auto& ueInfo = ueEntry.first;
            if (ueInfo->HasPendingDlSignalingTraffic())
            {
                NS_LOG_WARN("UE " << ueInfo->m_rnti << " has pending DL signaling traffic.");
            }
        }
    }
}

/**
 * @brief Schedule DL HARQ and data
 * @param dlSfnSf Slot number
 * @param ulAllocations Uplink allocation for this slot
 * @param allocInfo Allocation info pointer (where to save the allocations)
 * @return the number of symbols used for the DL allocation
 *
 * The DL phase can be OFDMA-based or TDMA-based. The method calculates the
 * number of available symbols as the total number of symbols in one slot
 * minus the number of symbols already allocated for UL. Then, it defines
 * the data starting point by keeping in consideration the number of symbols
 * previously allocated. In this way, DL and UL allocation will not overlap.
 *
 * HARQ retx processing is done in the function  ScheduleDlHarq(), while
 * DL new data processing in the function ScheduleDlData(). The method is
 * ensuring that if an UE gets a DCI for HARQ, it will not be scheduled for new
 * data as well. We have a limit of 1 DCI per UE.
 *
 */
uint8_t
NrMacSchedulerNs3::DoScheduleDl(const std::vector<DlHarqInfo>& dlHarqFeedback,
                                const ActiveHarqMap& activeDlHarq,
                                ActiveUeMap* activeDlUe,
                                const SfnSf& dlSfnSf,
                                const SlotElem& ulAllocations,
                                SlotAllocInfo* allocInfo)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(activeDlUe != nullptr);

    uint8_t dataSymPerSlot = m_macSchedSapUser->GetSymbolsPerSlot() - m_dlCtrlSymsSlot;

    uint8_t dlSymAvail = dataSymPerSlot - ulAllocations.m_totUlSym - m_dlSpsSymsSlot;
    PointInFTPlane dlAssignationStartPoint(0, m_dlCtrlSymsSlot + m_dlSpsSymsSlot);

    NS_LOG_DEBUG("Scheduling DL for slot "
                 << dlSfnSf << " DL HARQ to retransmit: " << dlHarqFeedback.size()
                 << " Active Beams DL HARQ: " << activeDlHarq.size()
                 << " sym available: " << static_cast<uint32_t>(dlSymAvail) << " starting from sym "
                 << static_cast<uint32_t>(m_dlCtrlSymsSlot));

    if (dlSymAvail == 0)
    {
        NS_LOG_WARN("No symbols available for DL data TX/RX.");
        LogUesWithPendingDlSignalingTraffic(*activeDlUe);
        // Keep the feedback for the next slot; dropping it would leave its processes in
        // RECEIVED_FEEDBACK with nothing to retransmit or reclaim them.
        m_dlHarqToRetransmit.insert(m_dlHarqToRetransmit.end(),
                                    dlHarqFeedback.begin(),
                                    dlHarqFeedback.end());
        return 0;
    }

    // OFDMA: the retransmissions take resource block groups over the data symbols, which new
    // data shares (HarqInFrequency()); the region is these dlSymAvail symbols
    m_dlRbgFreeForData.clear();
    uint8_t retxRegion = 0;
    // PDSCH aggregation: the further transmissions of earlier blocks come first, in the same
    // region as the retransmissions
    if (!m_dlRepetitions.empty())
    {
        NS_ABORT_MSG_UNLESS(UseHarqInFrequency(),
                            "PdschAggregation needs an OFDMA scheduler, one beam and "
                            "HarqRetxInFrequency");
        m_dlRbgFreeForData = GetDlBitmask();
        if (!PlaceDlRepetitions(dlAssignationStartPoint, dlSymAvail, allocInfo).empty())
        {
            retxRegion = dlSymAvail;
        }
    }
    if (!activeDlHarq.empty() && UseHarqInFrequency())
    {
        if (m_dlRbgFreeForData.empty())
        {
            m_dlRbgFreeForData = GetDlBitmask();
        }
        if (m_schedHarq->ScheduleDlHarqInFrequency(dlAssignationStartPoint,
                                                   dlSymAvail,
                                                   activeDlHarq,
                                                   m_ueMap,
                                                   &m_dlHarqToRetransmit,
                                                   dlHarqFeedback,
                                                   allocInfo,
                                                   &m_dlRbgFreeForData) > 0)
        {
            retxRegion = dlSymAvail;
        }
    }
    else if (!activeDlHarq.empty())
    {
        uint8_t usedHarq = ScheduleDlHarq(&dlAssignationStartPoint,
                                          dlSymAvail,
                                          activeDlHarq,
                                          m_ueMap,
                                          &m_dlHarqToRetransmit,
                                          dlHarqFeedback,
                                          allocInfo);
        NS_ASSERT_MSG(dlSymAvail >= usedHarq,
                      "DlSymAvail (" << +dlSymAvail << ") < usedHarq (" << +usedHarq << ")");
        dlSymAvail -= usedHarq;
        dlAssignationStartPoint.m_sym += usedHarq;
        allocInfo->m_numSymAlloc += usedHarq;
        NS_ASSERT_MSG(allocInfo->m_numSymAlloc <= 14,
                      "Invalid number of symbols: " << allocInfo->m_numSymAlloc << " symbols. ");
    }

    GetSecond GetUeInfoList;

    for (const auto& alloc : allocInfo->m_varTtiAllocInfo)
    {
        for (auto it = activeDlUe->begin(); it != activeDlUe->end(); /* no incr */)
        {
            auto& ueInfos = GetUeInfoList(*it);
            for (auto ueIt = ueInfos.begin(); ueIt != ueInfos.end(); /* no incr */)
            {
                GetFirst GetUeInfoPtr;
                if (GetUeInfoPtr(*ueIt)->m_rnti == alloc.m_dci->m_rnti)
                {
                    NS_LOG_INFO("Removed RNTI " << alloc.m_dci->m_rnti
                                                << " from active ue list "
                                                   "because it has already an HARQ scheduled");
                    ueInfos.erase(ueIt);
                    break;
                }
                else
                {
                    ++ueIt;
                }
            }
            if (!ueInfos.empty())
            {
                ++it;
            }
            else
            {
                activeDlUe->erase(it);
                break;
            }
        }
    }

    if (dlSymAvail == 0 && !activeDlUe->empty())
    {
        NS_LOG_WARN("No symbols available for new DL data transmissions.");
    }

    NS_ASSERT(dlAssignationStartPoint.m_rbg == 0);

    const auto dataMask = GetDlDataBitmask();
    const bool rbgLeft = std::find(dataMask.begin(), dataMask.end(), true) != dataMask.end();
    uint8_t usedDl = 0;
    if (dlSymAvail > 0 && !activeDlUe->empty() && rbgLeft)
    {
        usedDl = DoScheduleDlData(&dlAssignationStartPoint, dlSymAvail, *activeDlUe, allocInfo);
        NS_ASSERT(dlSymAvail >= usedDl);
        dlSymAvail -= usedDl;
    }
    if (retxRegion > 0)
    {
        // The new data, if any, took the same symbols as the retransmissions
        NS_ASSERT_MSG(usedDl == 0 || usedDl == retxRegion,
                      "DL data " << +usedDl << " symbols, retransmissions " << +retxRegion);
        if (usedDl == 0)
        {
            dlSymAvail -= retxRegion;
            dlAssignationStartPoint.m_sym += retxRegion;
            allocInfo->m_numSymAlloc += retxRegion;
            NS_ASSERT_MSG(allocInfo->m_numSymAlloc <= 14,
                          "Invalid number of symbols: " << allocInfo->m_numSymAlloc);
        }
    }
    m_dlRbgFreeForData.clear();

    return (dataSymPerSlot - ulAllocations.m_totUlSym - m_dlSpsSymsSlot) - dlSymAvail;
}

/**
 * @brief Decide how to fill the frequency/time of a DL slot
 * @param params parameters for the scheduler
 *
 * The function starts by refreshing the CQI received, and eventually resetting
 * the expired values. Then, the HARQ feedback are processed (ProcessHARQFeedbacks),
 * and finally the expired HARQs are canceled (ResetExpiredHARQ).
 *
 * @see ScheduleDl
 */
void
NrMacSchedulerNs3::DoSchedDlTriggerReq(
    const NrMacSchedSapProvider::SchedDlTriggerReqParameters& params)
{
    NS_LOG_FUNCTION(this);

    if (PdcchOn())
    {
        const uint32_t dlRbs = GetBandwidthInRbg() * GetNumRbPerRbg();
        m_pdcch->SetCarrier(dlRbs, params.m_snfSf.GetNumerology());
        // The data starts after the PDCCH symbols of each slot; the UE is told the number of
        // control symbols once, by RRC: the fewest a slot can have
        NS_ABORT_MSG_UNLESS(m_dlCtrlSymbols == m_pdcch->GetMinSymbols(),
                            "NrMacSchedulerNs3::DlCtrlSymbols must be "
                                << +m_pdcch->GetMinSymbols() << " with NrPdcch::Symbols = "
                                << +m_pdcch->GetConfiguredSymbols());
        m_pdcch->SetConfiguredUes(true,
                                  std::count_if(m_spsDl.begin(), m_spsDl.end(), [](const auto& e) {
                                      return e.second.active;
                                  }));
        if (m_pdcch->StartDownlink(params.m_snfSf))
        {
            // The uplink grants of this PDCCH are scheduled first: by the scheduler of the
            // uplink carrier, which has its turn at this very time
            Simulator::ScheduleNow(&NrMacSchedulerNs3::DoSchedDlTriggerReq, this, params);
            return;
        }
    }

    // process received CQIs
    m_cqiManagement.RefreshDlCqiMaps(m_ueMap);

    // Merge not-retransmitted and received feedback
    std::vector<DlHarqInfo> dlHarqFeedback;

    if (!params.m_dlHarqInfoList.empty() || !m_dlHarqToRetransmit.empty())
    {
        // m_dlHarqToRetransmit will be cleared inside MergeHARQ
        uint64_t existingSize = m_dlHarqToRetransmit.size();
        uint64_t inSize = params.m_dlHarqInfoList.size();

        dlHarqFeedback = MergeHARQ(&m_dlHarqToRetransmit, params.m_dlHarqInfoList, "DL");

        NS_ASSERT(m_dlHarqToRetransmit.empty());
        NS_ASSERT_MSG(existingSize + inSize == dlHarqFeedback.size(),
                      " existing: " << existingSize << " received: " << inSize
                                    << " calculated: " << dlHarqFeedback.size());

        std::unordered_map<uint16_t, std::set<uint32_t>> feedbacksDup;

        // Let's find out:
        // 1) Feedback that arrived late (i.e., their process has been marked inactive
        //    due to timings
        // 2) Duplicated feedbacks (same UE, same process ID). I don't know why
        //    these are generated.. but anyway..
        for (auto it = dlHarqFeedback.begin(); it != dlHarqFeedback.end(); /* no inc */)
        {
            if (m_ueMap.find(it->m_rnti) == m_ueMap.end())
            {
                NS_LOG_INFO("UE was released, but HARQ feedback remained in a buffer. We dispose "
                            "of it here.");
                it = dlHarqFeedback.erase(it);
                continue;
            }
            auto& ueInfo = m_ueMap.find(it->m_rnti)->second;
            auto& process = ueInfo->m_dlHarq.Find(it->m_harqProcessId)->second;
            NS_LOG_INFO("Analyzing feedback for UE " << it->m_rnti << " process "
                                                     << static_cast<uint32_t>(it->m_harqProcessId));
            if (!process.m_active)
            {
                NS_LOG_INFO("Feedback for UE " << it->m_rnti << " process "
                                               << static_cast<uint32_t>(it->m_harqProcessId)
                                               << " ignored because process is INACTIVE");
                // The process was reclaimed before this feedback got back, so HARQ can do
                // nothing with it. Link adaptation still can: the message carries the UE's real
                // ACK/NACK, and only the process bookkeeping is missing. Pass the true outcome
                // to OLLA rather than dropping it -- otherwise the loop sees only feedback that
                // returned in time, which is a small and optimistic slice whenever the process
                // lifetime is below the loop delay. Forcing these to NACK is equally wrong: it
                // reports failures that did not happen and starves the MCS down to the clamp.
                UpdateDlOlla(it->m_rnti, it->IsReceivedOk());
                it = dlHarqFeedback.erase(it); /* INC */
            }
            else if (!m_dlRepetitions.empty() &&
                     std::any_of(m_dlRepetitions.begin(),
                                 m_dlRepetitions.end(),
                                 [&process](const UlRepetition& r) {
                                     return r.dci == process.m_dciElement;
                                 }))
            {
                // PDSCH aggregation: the block of this process has transmissions still to come,
                // and the UE answers after the last. This feedback is for an earlier block of the
                // process (reclaimed by ResetExpiredHARQ and reused): stale.
                NS_LOG_WARN("Stale DL HARQ feedback for UE " << it->m_rnti << " process "
                                                             << +it->m_harqProcessId
                                                             << " (its block is still repeating)");
                it = dlHarqFeedback.erase(it); /* INC */
            }
            else
            {
                auto itDuplicated = feedbacksDup.find(it->m_rnti);
                if (itDuplicated == feedbacksDup.end())
                {
                    feedbacksDup.insert(std::make_pair(it->m_rnti, std::set<uint32_t>()));
                    feedbacksDup.at(it->m_rnti).insert(it->m_harqProcessId);
                    ++it; /* INC */
                }
                else
                {
                    if (itDuplicated->second.find(it->m_harqProcessId) ==
                        itDuplicated->second.end())
                    {
                        itDuplicated->second.insert(it->m_harqProcessId);
                        ++it; /* INC */
                    }
                    else
                    {
                        NS_LOG_INFO("Feedback for UE "
                                    << it->m_rnti << " process "
                                    << static_cast<uint32_t>(it->m_harqProcessId)
                                    << " ignored because is a duplicate of another feedback");
                        it = dlHarqFeedback.erase(it); /* INC */
                    }
                }
            }
        }

        ProcessHARQFeedbacks(&dlHarqFeedback, NrMacSchedulerUeInfo::GetDlHarqVector, "DL");
    }

    // Reclaim only after this slot's feedback has been consumed. Running it first destroys a
    // process in the very slot its own feedback arrives, which is an off-by-one that lands
    // exactly when the process lifetime equals the loop delay.
    for (const auto& itUe : m_ueMap)
    {
        ResetExpiredHARQ(itUe.second->m_rnti, &itUe.second->m_dlHarq);
    }

    ScheduleDl(params, dlHarqFeedback);
}

/**
 * @brief Decide how to fill the frequency/time of a UL slot
 * @param params parameters for the scheduler
 *
 * The function starts by refreshing the CQI received, and eventually resetting
 * the expired values. Then, the HARQ feedback are processed (ProcessHARQFeedbacks),
 * and finally the expired HARQs are canceled (ResetExpiredHARQ).
 *
 * @see ScheduleUl
 */
void
NrMacSchedulerNs3::DiagLog(const std::string& event) const
{
    if (!m_diagUlHarq)
    {
        return; // (every caller checks; a second net: no cost when the attribute is off)
    }
    std::ostringstream o;
    o << Simulator::Now().As(Time::MS) << " " << event;
    m_diagLog.push_back(o.str());
    if (m_diagLog.size() > 30000)
    {
        m_diagLog.pop_front();
    }
}

std::string
NrMacSchedulerNs3::DiagEvents(uint16_t rnti) const
{
    // The events of one UE (and every expiry), the whole window
    const std::string a = " rnti " + std::to_string(rnti) + " ";
    const std::string b = " of UE " + std::to_string(rnti) + " ";
    std::ostringstream o;
    for (const auto& e : m_diagLog)
    {
        if (e.find(a) != std::string::npos || e.find(b) != std::string::npos ||
            e.find("expire") != std::string::npos)
        {
            o << "  " << e << "\n";
        }
    }
    return o.str();
}

void
NrMacSchedulerNs3::DiagCheckUlFeedback(std::vector<UlHarqInfo>* ulHarqFeedback,
                                       const char* stage) const
{
    // A pending retransmission must have a live process in RECEIVED_FEEDBACK with a DCI. One
    // that has not (its process reclaimed or reused meanwhile) would be read by the sort of the
    // retransmissions and by their placement: it is dropped here, with a warning; with
    // DiagUlHarq the UE's history is printed too
    for (auto it = ulHarqFeedback->begin(); it != ulHarqFeedback->end();)
    {
        const auto ueIt = m_ueMap.find(it->m_rnti);
        bool ok = ueIt != m_ueMap.end() && ueIt->second->m_ulHarq.Exist(it->m_harqProcessId);
        std::string why = ok ? "" : "no such UE or process";
        if (ok)
        {
            const HarqProcess& p = ueIt->second->m_ulHarq.Get(it->m_harqProcessId);
            ok = p.m_active && p.m_status == HarqProcess::RECEIVED_FEEDBACK &&
                 p.m_dciElement != nullptr;
            if (!ok)
            {
                std::ostringstream o;
                o << "process active=" << p.m_active << " status=" << +p.m_status
                  << " dci=" << (p.m_dciElement ? "set" : "NULL");
                why = o.str();
            }
        }
        if (ok)
        {
            ++it;
            continue;
        }
        NS_LOG_WARN("UL HARQ " << stage << ": feedback " << *it << " without a pending process ("
                               << why << ") at " << Simulator::Now().As(Time::S) << ": dropped");
        if (m_diagUlHarq)
        {
            std::cerr << "DIAG UL HARQ " << stage << ": " << *it << " without a pending process ("
                      << why << ") at " << Simulator::Now().As(Time::S) << "\n";
            if (ueIt != m_ueMap.end())
            {
                std::cerr << ueIt->second->m_ulHarq
                          << "last erase: " << ueIt->second->m_ulHarq.LastErase() << "\nEVENTS\n"
                          << DiagEvents(it->m_rnti);
            }
        }
        it = ulHarqFeedback->erase(it);
    }
}

void
NrMacSchedulerNs3::DoSchedUlTriggerReq(
    const NrMacSchedSapProvider::SchedUlTriggerReqParameters& params)
{
    NS_LOG_FUNCTION(this);
    NrMacHarqVector::s_diagnostics = m_diagUlHarq;

    if (PdcchOn())
    {
        // (the UEs whose power control commands come in DCI 2_2)
        m_pdcch->SetConfiguredUes(false,
                                  std::count_if(m_spsUl.begin(), m_spsUl.end(), [](const auto& e) {
                                      return e.second.active;
                                  }));
        m_pdcch->StartUplink();
    }

    // process received CQIs
    m_cqiManagement.RefreshUlCqiMaps(m_ueMap);

    // Merge not-retransmitted and received feedback
    std::vector<UlHarqInfo> ulHarqFeedback;
    const uint64_t nowSlot = static_cast<uint64_t>(
        Simulator::Now().GetNanoSeconds() / m_macSchedSapUser->GetSlotPeriod().GetNanoSeconds());
    if (!params.m_ulHarqInfoList.empty() || !m_ulHarqToRetransmit.empty())
    {
        // m_ulHarqToRetransmit will be cleared inside MergeHARQ
        uint64_t existingSize = m_ulHarqToRetransmit.size();
        uint64_t inSize = params.m_ulHarqInfoList.size();

        for (const auto& f : m_ulHarqToRetransmit)
        if (m_diagUlHarq)
        {
            std::ostringstream o;
            o << "in(buffered) " << f;
            DiagLog(o.str());
        }
        for (const auto& f : params.m_ulHarqInfoList)
        if (m_diagUlHarq)
        {
            std::ostringstream o;
            o << "in(phy) " << f;
            DiagLog(o.str());
        }
        ulHarqFeedback = MergeHARQ(&m_ulHarqToRetransmit, params.m_ulHarqInfoList, "UL");

        NS_ASSERT(m_ulHarqToRetransmit.empty());
        NS_ASSERT_MSG(existingSize + inSize == ulHarqFeedback.size(),
                      " existing: " << existingSize << " received: " << inSize
                                    << " calculated: " << ulHarqFeedback.size());

        // if there are feedbacks for expired process, remove them
        for (auto it = ulHarqFeedback.begin(); it != ulHarqFeedback.end(); /* no inc */)
        {
            // msg3 (RRC Connection Request) HARQ feedback. The TC-RNTI is not yet
            // a registered UE (not in m_ueMap), so the regular HARQ machinery below
            // would crash on it. Handle it here: a NACK queues a retransmission,
            // an ACK or an exhausted retransmission budget closes the pending entry.
            if (m_msg3MaxRetx > 0)
            {
                auto msg3It = m_msg3Pending.find(it->m_rnti);
                if (msg3It != m_msg3Pending.end())
                {
                    if (it->m_receptionStatus == UlHarqInfo::NotOk &&
                        msg3It->second.attempts < m_msg3MaxRetx)
                    {
                        NS_LOG_INFO("msg3 NACK for TC-RNTI "
                                    << it->m_rnti << " (attempt "
                                    << static_cast<uint32_t>(msg3It->second.attempts)
                                    << "), scheduling retransmission");
                        ++msg3It->second.attempts;
                        m_msg3RetxList.push_back(it->m_rnti);
                    }
                    else
                    {
                        // ACK (msg3 decoded; UE config will follow) or no budget left:
                        // stop tracking. On success the entry is normally also erased
                        // in DoCschedUeConfigReq; doing it here is harmless and covers
                        // the give-up case.
                        NS_LOG_INFO(
                            "msg3 for TC-RNTI "
                            << it->m_rnti << " closed (status "
                            << (it->m_receptionStatus == UlHarqInfo::Ok ? "Ok" : "exhausted")
                            << ")");
                        m_msg3Pending.erase(msg3It);
                    }
                    // Never let TC-RNTI feedback reach ProcessHARQFeedbacks.
                    it = ulHarqFeedback.erase(it);
                    continue;
                }
            }
            if (m_ueMap.find(it->m_rnti) == m_ueMap.end())
            {
                NS_LOG_WARN("UE was released, but HARQ feedback remained in a buffer. We dispose "
                            "of it here.");
                if (m_diagUlHarq)
                {
                    std::ostringstream o;
                    o << "drop(no UE) " << *it;
                    DiagLog(o.str());
                }
                it = ulHarqFeedback.erase(it);
                continue;
            }
            auto& ueInfo = m_ueMap.find(it->m_rnti)->second;
            if (!ueInfo->m_ulHarq.Exist(it->m_harqProcessId))
            {
                // (a process id outside the UE's HARQ vector would be read through the map's end())
                NS_LOG_WARN("UL HARQ feedback for a process the UE has not: " << *it << ": dropped");
                it = ulHarqFeedback.erase(it);
                continue;
            }
            auto& process = ueInfo->m_ulHarq.Find(it->m_harqProcessId)->second;
            if (!process.m_active)
            {
                NS_LOG_INFO("Feedback for UE " << it->m_rnti << " process "
                                               << static_cast<uint32_t>(it->m_harqProcessId)
                                               << " ignored because process is INACTIVE");
                if (m_diagUlHarq)
                {
                    std::ostringstream o;
                    o << "drop(inactive) " << *it;
                    DiagLog(o.str());
                }
                it = ulHarqFeedback.erase(it);
                continue;
            }
            // A feedback is for the transmission the process holds now: the same redundancy
            // version, and not before that transmission can have been received. Otherwise it is
            // for an earlier block whose process was reclaimed (ResetExpiredHARQ) and reused:
            // stale, and taking it would retransmit, or release, the wrong block
            {
                const auto& dci = process.m_dciElement;
                const uint64_t reps = dci && dci->m_repetitions > 0 ? dci->m_repetitions : 1U;
                const bool tooSoon =
                    process.m_txSlot > 0 && nowSlot < process.m_txSlot + reps - 1;
                const bool otherRv = dci && it->m_numRetx != dci->m_rv;
                if (tooSoon || otherRv)
                {
                    NS_LOG_WARN("Stale UL HARQ feedback for UE "
                                << it->m_rnti << " process " << +it->m_harqProcessId
                                << (otherRv ? " (another redundancy version)" : " (too soon)")
                                << ": ignored");
                    if (m_diagUlHarq)
                    {
                        std::ostringstream o;
                        o << "drop(stale" << (otherRv ? ", rv " : ", too soon, txSlot ")
                          << (otherRv ? std::to_string(dci->m_rv)
                                      : std::to_string(process.m_txSlot))
                          << ") " << *it;
                        DiagLog(o.str());
                    }
                    it = ulHarqFeedback.erase(it);
                    continue;
                }
            }
            ++it;
        }

        ProcessHARQFeedbacks(&ulHarqFeedback, NrMacSchedulerUeInfo::GetUlHarqVector, "UL");
        DiagCheckUlFeedback(&ulHarqFeedback, "after ProcessHARQFeedbacks");
    }

    // Reclaim after this slot's feedback has been consumed; see the DL path for why.
    for (const auto& itUe : m_ueMap)
    {
        ResetExpiredHARQ(itUe.second->m_rnti, &itUe.second->m_ulHarq);
    }
    DiagCheckUlFeedback(&ulHarqFeedback, "after ResetExpiredHARQ");

    ScheduleUl(params, ulHarqFeedback);
}

/**
 * @brief Save the SR list into m_srList
 * @param params SR list
 *
 * m_srList will be evaluated in DoScheduleUlSr()
 */
void
NrMacSchedulerNs3::DoSchedUlSrInfoReq(
    const NrMacSchedSapProvider::SchedUlSrInfoReqParameters& params)
{
    NS_LOG_FUNCTION(this);

    // Merge RNTI in our current list
    for (const auto& ue : params.m_srList)
    {
        NS_LOG_INFO("UE " << ue << " asked for a SR ");

        auto it = std::find(m_srList.begin(), m_srList.end(), ue);
        if (it == m_srList.end())
        {
            m_srList.push_back(ue);
        }
    }
    NS_ASSERT(m_srList.size() >= params.m_srList.size());
}

/**
 * @brief Scheduler UL RACH
 * @param sPoint
 * @param symAvail
 * @param ueMap
 * @param slotAlloc
 *
 *  RLC TM SDU sent over resources specified in the UL Grant
 *  in the RAR (not in UL DCIs); the reason is that C-RNTI is
 *  not known yet at this stage. In the simulator, this is modeled
 *  as a real RLC TM RLC PDU whose UL resources are allocated
 *  by the scheduler upon call to SCHED_DL_RACH_INFO_REQ.
 *
 * @return The number of symbols used in the allocation
 */
std::pair<uint8_t, uint16_t>
NrMacSchedulerNs3::FitMsg3Grant(uint16_t estimatedSizeBits,
                                uint8_t symAvail,
                                uint16_t usableRbgs) const
{
    uint8_t allocSymbols = 0;
    uint16_t tbSizeBits = 0;

    // find the lowest TB size that fits UL grant estimated size
    while ((tbSizeBits < estimatedSizeBits) && (symAvail - allocSymbols > 0))
    {
        allocSymbols++;
        const auto nprbs = usableRbgs * GetNumRbPerRbg() * allocSymbols;
        tbSizeBits = GetUlAmc()->CalculateTbSize(m_rachUlGrantMcs, 1, nprbs) * 8;
    }
    return {allocSymbols, tbSizeBits};
}

uint8_t
NrMacSchedulerNs3::DoScheduleUlMsg3(PointInFTPlane* sPoint,
                                    uint8_t symAvail,
                                    SlotAllocInfo* slotAlloc)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(sPoint->m_rbg == 0);

    uint8_t symAvailBeforeRach = symAvail;
    auto rbgBitmask = GetUlBitmask();
    uint16_t usableRbgs = std::count(rbgBitmask.begin(), rbgBitmask.end(), true);

    std::size_t granted = 0;
    for (const auto& rachReq : m_rachList)
    {
        const auto [allocSymbols, tbSizeBits] =
            FitMsg3Grant(rachReq.m_estimatedSize, symAvail, usableRbgs);

        if (tbSizeBits < rachReq.m_estimatedSize)
        {
            // no more allocation space: the rest waits for the next uplink slot
            break;
        }
        ++granted;

        sPoint->m_sym -= allocSymbols;

        // DL-RACH Allocation
        // Ideal: no needs of configuring m_dci
        // UL-RACH Allocation
        uint8_t rank{1};
        Ptr<const ComplexMatrixArray> precMats{nullptr};
        std::shared_ptr<DciInfoElementTdma> ulMsg3Dci =
            std::make_shared<DciInfoElementTdma>(rachReq.m_rnti,
                                                 DciInfoElementTdma::UL,
                                                 sPoint->m_sym,
                                                 allocSymbols,
                                                 m_rachUlGrantMcs,
                                                 rank,
                                                 precMats,
                                                 tbSizeBits / 8,
                                                 1,
                                                 0,
                                                 DciInfoElementTdma::MSG3,
                                                 GetBwpId(),
                                                 GetTpc());

        ulMsg3Dci->m_rbgBitmask = rbgBitmask;
        symAvail -= allocSymbols;

        NS_LOG_INFO("MSG3 UL grant from cell ID "
                    << GetCellId() << " allocated to RNTI " << ulMsg3Dci->m_rnti
                    << " in slot: " << slotAlloc->m_sfnSf << " symStart " << +ulMsg3Dci->m_symStart
                    << " symEnd " << +ulMsg3Dci->m_numSym << " number of PRB"
                    << GetBandwidthInRbg() * GetNumRbPerRbg() << " MCS "
                    << (uint16_t)+ulMsg3Dci->m_mcs << " tbSize in bytes:" << +ulMsg3Dci->m_tbSize
                    << " BWP index: " << +ulMsg3Dci->m_bwpIndex
                    << " RBG bitmask:" << +ulMsg3Dci->m_rbgBitmask.size());

        VarTtiAllocInfo slotInfo(ulMsg3Dci);
        slotAlloc->m_varTtiAllocInfo.emplace_front(slotInfo);

        // Track this msg3 so that, if its UL PUSCH fails to decode, we can
        // retransmit it instead of letting the UE's T300 expire. Only the
        // INITIAL grant registers the entry; refresh on a re-grant of the same
        // TC-RNTI (RAR resent) without resetting the attempt counter.
        if (m_msg3MaxRetx > 0)
        {
            auto pendingIt = m_msg3Pending.find(rachReq.m_rnti);
            if (pendingIt == m_msg3Pending.end())
            {
                m_msg3Pending[rachReq.m_rnti] = Msg3RetxInfo{0, rachReq.m_estimatedSize};
            }
            else
            {
                pendingIt->second.estimatedSizeBits = rachReq.m_estimatedSize;
            }
        }
    }

    m_rachList.erase(m_rachList.begin(), m_rachList.begin() + granted);
    return (symAvailBeforeRach - symAvail);
}

uint8_t
NrMacSchedulerNs3::DoScheduleUlMsg3Retx(PointInFTPlane* sPoint,
                                        uint8_t symAvail,
                                        SlotAllocInfo* slotAlloc)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(sPoint->m_rbg == 0);

    uint8_t symAvailBeforeRetx = symAvail;
    auto rbgBitmask = GetUlBitmask();
    uint16_t usableRbgs = std::count(rbgBitmask.begin(), rbgBitmask.end(), true);

    for (const auto& rnti : m_msg3RetxList)
    {
        auto pendingIt = m_msg3Pending.find(rnti);
        if (pendingIt == m_msg3Pending.end())
        {
            // Should not happen: NACK handling keeps the pending entry alive while
            // it queues the retx. Skip defensively.
            NS_LOG_WARN("msg3 retx requested for TC-RNTI " << rnti
                                                           << " with no pending entry; skipping");
            continue;
        }

        const uint16_t estimatedSize = pendingIt->second.estimatedSizeBits;
        // rv carries the (capped) retransmission index; the HARQ combiner at the
        // gNB only distinguishes rv==3 (last redundancy version) from the rest.
        const uint8_t rv = std::min<uint8_t>(pendingIt->second.attempts, 3);

        // Same sizing rule as the initial msg3 grant
        const auto [allocSymbols, tbSizeBits] = FitMsg3Grant(estimatedSize, symAvail, usableRbgs);

        if (tbSizeBits < estimatedSize || allocSymbols == 0)
        {
            // No room left in this slot for this retx; keep it pending and try the
            // next UL slot (the entry stays in m_msg3Pending and we re-queue below).
            NS_LOG_INFO("Not enough UL symbols for msg3 retx of TC-RNTI " << rnti << " this slot");
            break;
        }

        sPoint->m_sym -= allocSymbols;

        uint8_t rank{1};
        Ptr<const ComplexMatrixArray> precMats{nullptr};
        std::shared_ptr<DciInfoElementTdma> ulMsg3RetxDci =
            std::make_shared<DciInfoElementTdma>(rnti,
                                                 DciInfoElementTdma::UL,
                                                 sPoint->m_sym,
                                                 allocSymbols,
                                                 m_rachUlGrantMcs,
                                                 rank,
                                                 precMats,
                                                 tbSizeBits / 8,
                                                 0, // NDI=0 -> UE retransmits buffered msg3
                                                 rv,
                                                 DciInfoElementTdma::DATA, // dispatched as UL DCI
                                                 GetBwpId(),
                                                 GetTpc());
        ulMsg3RetxDci->m_rbgBitmask = rbgBitmask;
        ulMsg3RetxDci->m_harqProcess = 0; // msg3 lives at HARQ process 0 on the UE
        symAvail -= allocSymbols;

        NS_LOG_INFO("MSG3 RETX UL grant from cell ID "
                    << GetCellId() << " allocated to TC-RNTI " << rnti << " in slot "
                    << slotAlloc->m_sfnSf << " symStart " << +ulMsg3RetxDci->m_symStart
                    << " numSym " << +ulMsg3RetxDci->m_numSym << " MCS "
                    << (uint16_t)+ulMsg3RetxDci->m_mcs << " tbSize " << +ulMsg3RetxDci->m_tbSize
                    << " rv " << +rv);

        VarTtiAllocInfo slotInfo(ulMsg3RetxDci);
        slotAlloc->m_varTtiAllocInfo.emplace_front(slotInfo);
    }

    // Clear the queue: retransmissions not placed this slot keep their pending
    // entry and will be re-queued by the next NACK.
    m_msg3RetxList.clear();
    return (symAvailBeforeRetx - symAvail);
}

bool
NrMacSchedulerNs3::IsMaxSrsReached() const
{
    return m_schedulerSrs->IsMaxSrsReached();
}

std::vector<bool>
NrMacSchedulerNs3::GetDlBitmask() const
{
    auto dlNotchedRBGsMask = GetDlNotchedRbgMask();
    dlNotchedRBGsMask = dlNotchedRBGsMask.empty() ? std::vector<bool>(GetBandwidthInRbg(), true)
                                                  : dlNotchedRBGsMask;
    return dlNotchedRBGsMask;
}

std::vector<bool>
NrMacSchedulerNs3::GetUlBitmask() const
{
    auto ulNotchedRBGsMask = GetUlNotchedRbgMask();
    ulNotchedRBGsMask = ulNotchedRBGsMask.empty() ? std::vector<bool>(GetBandwidthInRbg(), true)
                                                  : ulNotchedRBGsMask;
    return ulNotchedRBGsMask;
}

std::vector<bool>
NrMacSchedulerNs3::GetDlDataBitmask() const
{
    return m_dlRbgFreeForData.empty() ? GetDlBitmask() : m_dlRbgFreeForData;
}

std::vector<bool>
NrMacSchedulerNs3::GetUlDataBitmask() const
{
    return m_ulRbgFreeForData.empty() ? GetUlBitmask() : m_ulRbgFreeForData;
}

std::vector<DciInfoElementTdma>
NrMacSchedulerNs3::ReshapeAllocation(const std::vector<DciInfoElementTdma>& dcis,
                                     uint8_t& startingSymbol,
                                     uint8_t& numSymbols,
                                     std::vector<bool>& bitmask,
                                     const bool isDl)
{
    if (dcis.empty())
    {
        return {};
    }
    return DoReshapeAllocation(dcis, startingSymbol, numSymbols, bitmask, isDl, m_ueMap);
}

} // namespace ns3
