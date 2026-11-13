// Copyright (c) 2026
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_PDCCH_H
#define NR_PDCCH_H

#include "nr-phy-mac-common.h"
#include "sfnsf.h"

#include "ns3/nr-export.h"
#include "ns3/object.h"

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ns3
{

/**
 * @ingroup scheduler
 * @brief The PDCCH of one cell as a resource: CORESET, CCEs, aggregation levels, search spaces
 *
 * Without this class every DCI the schedulers make reaches the UE and costs nothing. With it
 * (attribute Enabled) a DCI needs a PDCCH candidate whose CCEs are free in its slot, at an
 * aggregation level that fits the SINR the UE reported; a UE that finds none is not scheduled in
 * that slot. The schedulers of the downlink and of the uplink carrier of the cell share one
 * object, found with Get(cellId).
 *
 * What follows the specifications (the clauses are quoted in the functions):
 *  - CORESET: N_RB resource blocks (whole groups of 6) and 1, 2 or 3 symbols; a REG is one RB in
 *    one symbol, a CCE is 6 REGs, N_CCE = N_RB x N_symb / 6 (TS 38.211 7.3.2.2)
 *  - aggregation levels 1, 2, 4, 8, 16 CCEs (TS 38.211 Table 7.3.2.1-1)
 *  - a PDCCH carries the DCI and a 24-bit CRC on 54 QPSK resource elements per CCE, 108 bits
 *    (TS 38.212 7.3.2 to 7.3.4; 3 of the 12 resource elements of a REG carry the DM-RS)
 *  - the CCEs of candidate m of level L: L x ((Y + floor(m N_CCE / (L M))) mod floor(N_CCE / L))
 *    and the L - 1 that follow, Y = 0 in a common search space and a pseudo-random number of the
 *    C-RNTI and the slot in a UE-specific one (TS 38.213 10.1)
 *  - common search space (SIB1, other system information, paging, random access response, group
 *    power control): levels 4, 8, 16 with 4, 2, 1 candidates (TS 38.213 Table 10.1-1); the level
 *    is one for each message (Sib1Al, OtherSiAl, PagingAl, RarAl, TpcAl; CommonAl if 0)
 *  - the size of the DCIs of those messages and of the RRC ones: format 1_0 (28 bits and the
 *    frequency domain resource assignment), to which formats 0_0 and 2_2 are padded; the
 *    reserved bits of the system information, paging and random access DCIs make them as large
 *    as the one for a UE (TS 38.212 7.3.1.0, 7.3.1.2.1, 7.3.1.3.3), GetDciBits(Purpose)
 *  - the DCIs the cell sends besides those of the schedulers (each in the first slots it fits):
 *    SIB1 every Sib1PeriodMs; an SI message, other system information, every period of
 *    SiPeriodsMs (TS 38.331 5.2.2; SIB19 of an NTN cell is one of them); paging in the paging
 *    occasion of a frame when it has a UE to page, which PagingRate pages a second make
 *    happen at random (TS 38.304 7.1: a paging message for all the UEs of the occasion);
 *    DCI 2_2 every TpcPeriodMs, one for TpcUesPerDci UEs that have an SPS or configured grant
 *    configuration (their power control commands; dynamic DCIs carry their own, TS 38.213 7.1
 *    and 7.2); and the DCIs of the RRC messages of a UE: RrcSetupDcis when it connects
 *    (RRCSetup, security, capabilities, reconfiguration, with their answers), RrcCallDcis when a
 *    call starts (the reconfiguration of its QoS flow, CallStarts()), a measurement report every
 *    MeasReportPeriodMs, one message every RrcStepMs (the round trip of the answer)
 *
 * What is simplified:
 *  - one CORESET and one UE-specific search space set, monitored in every slot; DCI formats
 *    0_0/1_0 and 0_1/1_1 are not told apart: every DCI of data has DciBits bits
 *  - the DCIs of system information, paging and RRC messages take CCEs only: the PDSCH or PUSCH
 *    of their messages takes no resource blocks from the schedulers
 *  - the block error rate of a PDCCH comes from one table (BlerTable: aggregation level 1, DCI of
 *    TableDciBits bits) moved by 10 log10(L) dB for level L and by the ratio of the coded payloads
 *    for another DCI size
 *  - PDCCH power boost (BoostMaxDb > 0): the resource elements of the CCEs the slot does not use
 *    (the CORESET has N_CCE of them, the DCIs take the rest) lend their energy to the CCEs in use.
 *    The network may set the power of a PDCCH as it likes: the UE assumes the same energy per
 *    resource element for the DM-RS and the data of a PDCCH (TS 38.213 does not signal any
 *    offset), so a boost needs no signalling and the channel estimate gains with it. In the
 *    symbols of the CORESET the total power is fixed, so the boost of every DCI of the slot is
 *    10 log10(N_CCE / used CCEs), at most BoostMaxDb (the PFD limit of the downlink bounds it
 *    in the script). It lowers the block error rate of the DCI only; the aggregation level is
 *    chosen without it (SinrMarginDb below 0 claims part of it in advance)
 *  - Symbols = 0 (dynamic): NR has no indicator of the size of the control region (the CORESET
 *    is configured by RRC); the model takes, slot by slot, the smallest of 1, 2, 3 symbols that
 *    holds all DCIs and starts the PDSCH right after it (the PDSCH start symbol is in the DCI:
 *    TS 38.214 Table 5.1.2.1-1), with the CCE numbering of a CORESET of that many symbols
 */
class NR_EXPORT NrPdcch : public Object
{
  public:
    /**
     * @brief GetTypeId
     * @return the TypeId of the class
     */
    static TypeId GetTypeId();
    NrPdcch();
    ~NrPdcch() override;

    /**
     * @brief The PDCCH of a cell, made with the default attribute values when first asked for
     * @param cellId the cell
     * @return the object, never null
     */
    static Ptr<NrPdcch> Get(uint16_t cellId);

    /**
     * @brief The PDCCH of a cell if there is one
     * @param cellId the cell
     * @return the object or null
     */
    static Ptr<NrPdcch> Find(uint16_t cellId);

    /// What a DCI is for. The order is the order of the report.
    enum Purpose : uint8_t
    {
        SIB1 = 0,          //!< system information, common search space
        OTHER_SI,          //!< an SI message (other system information), common search space
        PAGING,            //!< a paging message, common search space
        RAR,               //!< random access response, common search space
        TPC_GROUP,         //!< DCI 2_2: power control commands of a group of UEs, common
        RRC_DL,            //!< downlink assignment of an RRC message
        RRC_UL,            //!< uplink grant of an RRC message
        DL_NEW,            //!< downlink assignment, new data
        DL_RETX,           //!< downlink assignment, HARQ retransmission
        DL_SPS_ACTIVATION, //!< activation of a downlink SPS configuration
        DL_SPS_RELEASE,    //!< release of a downlink SPS configuration
        UL_NEW,            //!< uplink grant, new data
        UL_RETX,           //!< uplink grant, HARQ retransmission
        UL_CG_ACTIVATION,  //!< activation of an uplink configured grant Type 2
        UL_CG_RELEASE,     //!< release of an uplink configured grant Type 2
        N_PURPOSES
    };

    /// What is counted of the changes of a speaker between voice and silence: ChangeEvent()
    enum Change : uint8_t
    {
        CHANGE_SEEN = 0,   //!< the gNB saw the change: by the size of the first packet after it
        CHANGE_BLOCK,      //!< that packet went whole in a block of a configuration
        CHANGE_PIECES,     //!< ... in pieces, over a block and the next
        CHANGE_PIECE_PDUS, //!< the RLC PDUs of the packets that went in pieces
        CHANGE_DCI,        //!< ... with a DCI: a dynamic assignment or grant
        CHANGE_SWITCH,     //!< the configuration was released and the other one activated
        N_CHANGE
    };

    /// Things that happen to semi-persistent scheduling and configured grants, counted
    enum Event : uint8_t
    {
        DL_SPS_BLOCK = 0,   //!< a downlink block sent on an SPS occasion, without PDCCH
        DL_SPS_EMPTY,       //!< an SPS occasion with nothing to send
        DL_SPS_SEGMENT,     //!< an occasion that carried a part of a packet larger than its block
        DL_SPS_LARGE,       //!< a packet too large for the occasion, sent with a DCI of its own
        DL_SPS_LOST_CONFIG, //!< an occasion the UE could not receive: it had missed the activation
        UL_CG_BLOCK,        //!< an uplink block sent on a configured grant occasion
        UL_CG_EMPTY,        //!< a configured grant occasion the UE left unused
        UL_CG_SEGMENT,      //!< an occasion after which the UE still had a rest of the packet
        UL_CG_LARGE,        //!< data left over on a configured grant, served by a grant of its own
        UL_CG_LOST_CONFIG,  //!< an occasion the UE did not use: it had missed the activation
        UL_CG_CONFIRMATION, //!< a configured grant confirmation MAC CE sent
        UL_CG_CONFIRMATION_BYTES, //!< the bytes of these MAC CEs
        DL_SPS_WAIT,        //!< a packet sent with a DCI: its occasion was too far (SpsDlMaxWaitMs)
        VOICE_CHANGES,      //!< the first of 4 x N_CHANGE counters, see ChangeEvent()
        N_EVENTS = VOICE_CHANGES + 4 * N_CHANGE
    };

    /**
     * @brief The counter of a change between voice and silence
     * @param isDl the speaker of the downlink (the packets to the UE), or of the uplink
     * @param toVoice he started to talk, or he fell silent
     * @param what what is counted of it
     * @return the event to give to NoteEvent() and to look up in Stats::events
     */
    static Event ChangeEvent(bool isDl, bool toVoice, Change what)
    {
        return static_cast<Event>(VOICE_CHANGES +
                                  ((isDl ? 0 : 2) + (toVoice ? 1 : 0)) * N_CHANGE + what);
    }

    /// The counters behind the report
    struct Stats
    {
        uint64_t slots{0};                     //!< slots in which a PDCCH could be sent
        std::array<uint64_t, 4> slotsBySymbols{}; //!< [0] without any DCI, [d] with d symbols
        uint64_t cceAvailable{0};              //!< CCEs of the CORESET in all these slots
        uint64_t cceUsed{0};                   //!< CCEs that carried a DCI
        std::array<uint64_t, N_PURPOSES> dci{};     //!< DCIs sent, by purpose
        std::array<uint64_t, N_PURPOSES> cce{};     //!< their CCEs
        std::array<uint64_t, N_PURPOSES> blocked{}; //!< DCIs not sent: no candidate was free
        std::array<uint64_t, 5> dciByAl{};     //!< DCIs sent at level 1, 2, 4, 8, 16
        std::array<uint64_t, 5> cceByAl{};     //!< their CCEs
        std::array<uint64_t, 5> rxByAl{};      //!< DCIs for a UE that could have missed them
        std::array<uint64_t, 5> missedByAl{};  //!< of these, DCIs the UE did not decode
        uint64_t aboveTarget{0};  //!< DCIs sent although no level met the target at that SINR
        uint64_t raised{0};       //!< DCIs sent at a higher level: the chosen one had no free CCEs
        uint64_t noReport{0};     //!< DCIs sent before the UE had reported a SINR
        uint64_t unusedReservations{0}; //!< candidates reserved for a UE that got no DCI
        double ulShareSum{0.0};   //!< UlBalance demand: the uplink share of every slot, summed
        uint64_t ulCapped{0};     //!< of the blocked uplink grants: a candidate was free, but
                                  //!< UlMaxShare held them back
        double boostSumDb{0.0};   //!< sum of the PDCCH power boost (dB) of the DCIs sent, see BoostMaxDb
        uint64_t boosted{0};      //!< DCIs sent with a boost above 0.05 dB
        std::array<uint64_t, N_PURPOSES> repeated{}; //!< of the DCIs sent, those sent twice
                                                     //!< (Repetitions), by purpose
        std::array<uint64_t, 5> repeatedByAl{};      //!< ... by aggregation level
        uint64_t repetitionDropped{0}; //!< DCIs that wanted a copy and found no second candidate
        uint64_t repetitionByPower{0}; //!< Repetitions 0: copies dropped, the boost made one enough
        std::array<uint64_t, N_EVENTS> events{}; //!< see Event
    };

    /// A PDCCH candidate reserved for a DCI. False (level 0) if there was none.
    struct Grant
    {
        uint32_t id{0}; //!< handle, 0 = none
        uint8_t al{0};  //!< aggregation level, 0 = no candidate was free
        uint32_t copy{0}; //!< the second, linked candidate of the same DCI, 0 = none (Repetitions)
        explicit operator bool() const
        {
            return al != 0;
        }
    };

    // --- configuration -------------------------------------------------------------------

    /**
     * @return true if the PDCCH is modelled (attribute Enabled)
     */
    bool IsEnabled() const;
    /**
     * @return the attribute Symbols: 1, 2, 3, or 0 = dynamic
     */
    uint8_t GetConfiguredSymbols() const;
    /**
     * @return the fewest PDCCH symbols a slot can have: Symbols, or 1 if dynamic
     */
    uint8_t GetMinSymbols() const;
    /**
     * @return the most PDCCH symbols a slot can have: Symbols, or 3 if dynamic
     */
    uint8_t GetMaxSymbols() const;
    /**
     * @return the size of a DCI of new data, retransmissions and semi-persistent scheduling in
     * bits, without the CRC (the attribute DciBits)
     */
    uint32_t GetDciBits() const;
    /**
     * @brief The size of a DCI, without the CRC
     *
     * The DCIs of the cell's own messages follow TS 38.212 7.3.1 unless CellDciBits is set:
     * DCI format 1_0 has 28 bits and the frequency domain resource assignment of
     * ceil(log2(N (N + 1) / 2)) bits, with N the RBs of CORESET 0 (SI-RNTI, P-RNTI, RA-RNTI,
     * TC-RNTI: 15, 6 and 16 reserved bits make them the same size, 7.3.1.2.1) or of the
     * carrier (C-RNTI). Format 0_0 (29 bits on 25 RB) and format 2_2 (2 bits for each UE, 3 with
     * a closed loop indicator) are padded with zeros to the size of format 1_0 (7.3.1.0,
     * 7.3.1.3.3).
     * @param purpose what the DCI is for
     * @return the size in bits
     */
    uint32_t GetDciBits(Purpose purpose) const;
    /**
     * @brief Tell the PDCCH the carrier it is on. The scheduler of the downlink carrier does so.
     * @param dlRbs resource blocks of the downlink bandwidth part
     * @param numerology subcarrier spacing configuration mu
     */
    void SetCarrier(uint32_t dlRbs, uint8_t numerology);
    /**
     * @return true once SetCarrier() has been called
     */
    bool HasCarrier() const;
    /**
     * @return resource blocks of the CORESET
     */
    uint32_t GetCoresetRbs() const;
    /**
     * @param symbols symbols of the CORESET, 1 to 3
     * @return the number of CCEs of the CORESET
     */
    uint32_t GetNumCce(uint8_t symbols) const;
    /**
     * @param al aggregation level
     * @return the PDCCH candidates of the UE-specific search space at this level
     */
    uint32_t GetCandidates(uint8_t al) const;
    /**
     * @return the candidates a UE monitors in one slot: common and UE-specific search space
     */
    uint32_t GetMonitoredCandidates() const;
    /**
     * @param numerology subcarrier spacing configuration mu, 0 to 3
     * @return the most candidates a UE monitors per slot (TS 38.213 Table 10.1-2)
     */
    static uint32_t GetMaxMonitoredCandidates(uint8_t numerology);
    /**
     * @return the aggregation level of the DCIs of the common search space (CommonAl)
     */
    uint8_t GetCommonAl() const;
    /**
     * @param purpose a DCI of the common search space: SIB1, OTHER_SI, PAGING, RAR or TPC_GROUP
     * @return its aggregation level: the attribute of the message, or CommonAl if that is 0; the
     * highest of 4, 8, 16 up to it that the CORESET holds
     */
    uint8_t GetCommonAl(Purpose purpose) const;

    // --- the link ------------------------------------------------------------------------

    /**
     * @brief Block error rate of a PDCCH
     * @param al aggregation level
     * @param sinrDb SINR of the control symbols at the UE
     * @param boostDb power of the PDCCH above the reference signal (the hypothetical PDCCH of
     * the radio link monitoring has 4 dB, TS 38.133 Table 8.1.2.1-1)
     * @param dciBits size of the DCI without its CRC, 0 = DciBits
     * @param copies copies of the PDCCH the UE combines (Repetitions): 2 adds RepetitionGainDb
     * @return the probability that the UE does not decode the DCI
     */
    double GetBler(uint8_t al,
                   double sinrDb,
                   double boostDb = 0.0,
                   uint32_t dciBits = 0,
                   uint8_t copies = 1) const;
    /**
     * @brief The lowest SINR at which a level meets a block error rate
     * @param al aggregation level
     * @param bler the block error rate
     * @param dciBits size of the DCI without its CRC, 0 = DciBits
     * @return the SINR in dB (SinrMarginDb not included)
     */
    double GetSinrForBler(uint8_t al, double bler, uint32_t dciBits = 0) const;
    /**
     * @brief The aggregation level for a UE: the lowest that meets TargetBler at the reported
     * SINR less SinrMarginDb, of those the search space has candidates for and the CORESET
     * can hold; the highest of them if none does
     * @param sinrDb the SINR the UE reported, NaN if it has not reported any
     * @param meetsTarget if not null: whether the level meets the target
     * @param dciBits size of the DCI without its CRC, 0 = DciBits
     * @return the aggregation level, 0 if the CORESET holds none
     */
    uint8_t ChooseAl(double sinrDb, bool* meetsTarget = nullptr, uint32_t dciBits = 0) const;

    /**
     * @brief Note the SINR a UE reported
     * @param rnti the UE
     * @param sinrDb the SINR
     */
    void ReportSinr(uint16_t rnti, double sinrDb);
    /**
     * @param rnti the UE
     * @return the SINR it reported last, NaN if none
     */
    double GetSinr(uint16_t rnti) const;
    /**
     * @brief Forget a UE
     * @param rnti the UE
     */
    void RemoveUe(uint16_t rnti);
    /**
     * @brief A UE has connected: the DCIs of its RRC messages are to come (RrcSetupDcis), and
     * its measurement reports. Once per RNTI: the schedulers of both carriers call it.
     * @param rnti the UE
     */
    void AddUe(uint16_t rnti);
    /**
     * @brief A call of a UE starts: the DCIs of the RRC messages that set its QoS flow up
     * (RrcCallDcis)
     * @param rnti the UE
     */
    void CallStarts(uint16_t rnti);
    /**
     * @brief The UEs with an SPS (downlink) or configured grant (uplink) configuration, whose
     * power control commands come in DCI 2_2. Each scheduler tells its own.
     * @param isDl downlink SPS or uplink configured grants
     * @param n the UEs
     */
    void SetConfiguredUes(bool isDl, uint32_t n);

    // --- the PDCCH of the slot being scheduled -------------------------------------------
    //
    // A gNB PHY asks for the uplink of slot n + K2 and for the downlink of slot n at the same
    // moment, and the DCIs of both go into the PDCCH of slot n. So all the requests made at one
    // simulation time are for one PDCCH, and a new time starts a new one.

    /**
     * @brief The downlink scheduler is about to schedule a slot
     * @param target the slot
     * @return true if it has to come back in a moment (Simulator::ScheduleNow): the scheduler of
     * the uplink carrier has not been asked yet at this time, and uplink grants go first
     */
    bool StartDownlink(const SfnSf& target);
    /**
     * @brief The uplink scheduler is about to schedule a slot
     */
    void StartUplink();
    /**
     * @brief Reserve a PDCCH candidate for a DCI
     * @param rnti the UE (0 for the common search space)
     * @param purpose what the DCI is for
     * @return the candidate, false if none is free
     */
    Grant Request(uint16_t rnti, Purpose purpose);
    /**
     * @brief Give a reservation back: no DCI came of it
     * @param grant the candidate
     * @param count false if it is not to count as a reservation kept for nothing (the half of
     * a change of configuration whose other half found no candidate)
     */
    void Release(const Grant& grant, bool count = true);
    /**
     * @brief A DCI uses the reservation
     * @param grant the candidate
     * @param purpose what the DCI is for after all
     * @param dci the DCI: its aggregation level, first CCE and PDCCH symbols are filled in
     */
    void Commit(const Grant& grant, Purpose purpose, const std::shared_ptr<DciInfoElementTdma>& dci);
    /**
     * @brief Settle the number of PDCCH symbols of the slot. Requests made after this have to
     * fit into them.
     * @return the number of symbols
     */
    uint8_t FixSymbols();

    // --- counting ------------------------------------------------------------------------

    /**
     * @brief The UE received (or missed) a DCI
     * @param al its aggregation level
     * @param missed true if it did not decode it
     */
    void NoteReception(uint8_t al, bool missed);
    /**
     * @brief Count an event of semi-persistent scheduling or configured grants
     * @param event the event
     * @param n how many
     */
    void NoteEvent(Event event, uint64_t n = 1);
    /**
     * @brief Start the counters again from zero
     */
    void ResetStats();
    /**
     * @brief Stop or resume counting
     * @param on true to count
     */
    void SetCounting(bool on);
    /**
     * @return the counters, the slot in the making included
     */
    Stats GetStats();
    /**
     * @param purpose a purpose
     * @return its name in the report
     */
    static std::string GetPurposeName(Purpose purpose);

  private:
    /// One reservation
    struct Entry
    {
        uint32_t id{0};
        uint16_t rnti{0};
        Purpose purpose{DL_NEW};
        uint8_t al{0};
        std::array<int32_t, 4> cce{{-1, -1, -1, -1}}; //!< first CCE with d symbols, -1 = no room
        bool committed{false};
        bool meetsTarget{true};
        bool raised{false};
        bool noReport{false};
        uint8_t charged{0}; //!< CCEs taken from the credit of the uplink
        uint32_t copyOf{0}; //!< the second candidate of the DCI of entry copyOf (Repetitions)
        uint32_t copy{0};   //!< the id of this DCI's second candidate, 0 = none
        std::shared_ptr<DciInfoElementTdma> dci;
    };
    /// Whether a DCI of this purpose may be sent twice (RepetitionFor)
    bool MayRepeat(Purpose purpose) const;

    void SetBlerTable(const std::string& table);
    std::string GetBlerTable() const;
    void SetCandidates(const std::string& candidates);
    std::string GetCandidatesString() const;
    void SetSiPeriods(const std::string& periods);
    std::string GetSiPeriods() const;
    void SetRrcSetupDcis(const std::string& dcis);
    std::string GetRrcSetupDcis() const;
    void SetRrcCallDcis(const std::string& dcis);
    std::string GetRrcCallDcis() const;
    /// queue the DCIs of RRC messages of a UE: dl downlink and ul uplink, alternating
    void QueueRrc(uint16_t rnti, uint32_t dl, uint32_t ul);
    /// the DCIs the cell sends of its own in this slot (system information, paging, DCI 2_2,
    /// RRC messages), after those due earlier that found no room
    void PlaceCellDcis();
    /// a number in [0, 1) that depends on the slot only (paging): the same in every run
    double SlotRandom(uint64_t salt) const;
    void NewSlotIfNeeded();
    void FinishSlot();
    void DoFixSymbols();
    /// the power boost of the DCIs of the slot in the making (BoostMaxDb), from the CCEs it uses
    double ApplyBoost();
    bool IsUplink(Purpose purpose) const;
    bool IsCommon(Purpose purpose) const;
    /// first CCE of candidate m at level L in a CORESET of d symbols, -1 if there is none
    int32_t CandidateCce(uint16_t rnti, bool common, uint8_t al, uint32_t m, uint8_t d) const;
    /// first free candidate at level L with d symbols, -1 if none
    int32_t FindFree(uint16_t rnti, bool common, uint8_t al, uint8_t d) const;
    void Occupy(uint8_t d, int32_t cce, uint8_t al, uint32_t id);
    void Vacate(uint8_t d, int32_t cce, uint8_t al);
    static int AlIndex(uint8_t al);

    // attributes
    bool m_enabled{false};
    uint8_t m_symbols{1};
    uint32_t m_dciBits{34};
    uint32_t m_coresetRbsAttr{0};
    std::array<uint32_t, 5> m_candidates{{6, 6, 4, 2, 1}};
    std::vector<std::pair<double, double>> m_table; //!< SINR (dB), log10(BLER)
    uint32_t m_tableDciBits{34};
    double m_targetBler{0.01};
    double m_sinrMarginDb{0.0};
    double m_sinrOffsetDb{0.0};
    uint8_t m_commonAlAttr{8};
    uint8_t m_sib1AlAttr{0};
    uint8_t m_otherSiAlAttr{0};
    uint8_t m_pagingAlAttr{0};
    uint8_t m_rarAlAttr{0};
    uint8_t m_tpcAlAttr{0};
    uint32_t m_cellDciBits{0};
    uint8_t m_initialAl{0};
    bool m_raiseAl{true};
    uint8_t m_repetitions{1};       //!< Repetitions: 1, 2, 0 = when a single one misses the target
    double m_repetitionGainDb{3.0103}; //!< RepetitionGainDb
    std::string m_repetitionFor{"all"}; //!< RepetitionFor: all, sps
    uint32_t m_sib1PeriodMs{20};
    double m_ulMaxShare{0.5};
    double m_boostMaxDb{0.0};
    std::vector<uint32_t> m_siPeriodsMs{160, 160, 320};
    uint32_t m_pagingCycleMs{1280};
    uint32_t m_pagingFrames{0};
    double m_pagingRate{5.0};
    uint32_t m_tpcPeriodMs{40};
    uint32_t m_tpcUesPerDci{13};
    std::array<uint32_t, 2> m_rrcSetupDcis{{4, 5}}; //!< downlink, uplink
    std::array<uint32_t, 2> m_rrcCallDcis{{1, 1}};
    uint32_t m_rrcStepMs{20};
    uint32_t m_measReportPeriodMs{0};

    // the DCIs the cell sends of its own: due in a slot, or waiting for room since then
    struct CellDci
    {
        uint16_t rnti{0};
        Purpose purpose{SIB1};
        uint64_t due{0}; //!< absolute slot
    };

    std::vector<CellDci> m_cellDcis;
    uint64_t m_lastCellDciSlot{UINT64_MAX};    //!< the slot whose cell DCIs have been made
    std::map<uint16_t, uint64_t> m_connected;  //!< UEs and the slot they connected in
    std::array<uint32_t, 2> m_configuredUes{}; //!< with SPS (downlink), configured grant (uplink)

    // the carrier
    uint32_t m_coresetRbs{0};
    uint32_t m_carrierRbs{0};
    uint8_t m_numerology{0};
    bool m_hasCarrier{false};

    // per UE
    std::map<uint16_t, double> m_sinrDb;

    // the slot in the making
    int64_t m_slotNs{-1};             //!< simulation time of its requests
    uint64_t m_absSlot{0};            //!< its number, counted from frame 0
    bool m_absSlotKnown{false};
    int64_t m_latencySlots{0};        //!< from the time of the requests to the slot itself
    bool m_dlStarted{false};
    bool m_fixed{false};
    uint8_t m_slotSymbols{0};
    std::array<std::vector<int32_t>, 4> m_owner; //!< [d][cce] -> entry id, -1 = free
    std::array<bool, 4> m_valid{};               //!< [d]: everything granted so far has room
    std::vector<Entry> m_entries;
    uint32_t m_ulCce{0};
    double m_ulCredit{0.0}; //!< the share of the uplink grants: CCEs they may take in any case
    uint32_t m_dlDemandCce{0};
    uint32_t m_prevDlDemandCce{0};
    uint32_t m_ulDemandCce{0};      //!< CCEs the uplink grants of the slot asked for
    uint32_t m_prevUlDemandCce{0};
    std::string m_ulBalance{"fixed"}; //!< UlBalance attribute: fixed or demand
    double m_ulDemandShare{0.5};      //!< UlBalance demand: the uplink share, averaged
    double m_ulBalanceSlots{20.0};    //!< UlBalanceSlots attribute
    double m_ulBalanceMin{0.2};       //!< UlBalanceMin attribute
    uint32_t m_nextId{1};

    // uplink before downlink
    bool m_ulSeen{false};
    int64_t m_ulNs{-1};
    int64_t m_deferNs{-1};

    bool m_counting{true};
    Stats m_stats;
};

} // namespace ns3

#endif // NR_PDCCH_H
