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
 *  - common search space (SIB1, random access response): levels 4, 8, 16 with 4, 2, 1 candidates
 *    (TS 38.213 Table 10.1-1)
 *
 * What is simplified:
 *  - one CORESET and one UE-specific search space set, monitored in every slot; DCI formats
 *    0_0/1_0 and 0_1/1_1 are not told apart: every DCI has DciBits bits
 *  - the block error rate of a PDCCH comes from one table (BlerTable: aggregation level 1, DCI of
 *    TableDciBits bits) moved by 10 log10(L) dB for level L and by the ratio of the coded payloads
 *    for another DCI size
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
        RAR,               //!< random access response, common search space
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
        uint64_t ulCapped{0};     //!< of the blocked uplink grants: a candidate was free, but
                                  //!< UlMaxShare held them back
        std::array<uint64_t, N_EVENTS> events{}; //!< see Event
    };

    /// A PDCCH candidate reserved for a DCI. False (level 0) if there was none.
    struct Grant
    {
        uint32_t id{0}; //!< handle, 0 = none
        uint8_t al{0};  //!< aggregation level, 0 = no candidate was free
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
     * @return the size of a DCI in bits, without the CRC
     */
    uint32_t GetDciBits() const;
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
     * @return the aggregation level of the DCIs of the common search space
     */
    uint8_t GetCommonAl() const;

    // --- the link ------------------------------------------------------------------------

    /**
     * @brief Block error rate of a PDCCH
     * @param al aggregation level
     * @param sinrDb SINR of the control symbols at the UE
     * @param boostDb power of the PDCCH above the reference signal (the hypothetical PDCCH of
     * the radio link monitoring has 4 dB, TS 38.133 Table 8.1.2.1-1)
     * @return the probability that the UE does not decode the DCI
     */
    double GetBler(uint8_t al, double sinrDb, double boostDb = 0.0) const;
    /**
     * @brief The lowest SINR at which a level meets a block error rate
     * @param al aggregation level
     * @param bler the block error rate
     * @return the SINR in dB (SinrMarginDb not included)
     */
    double GetSinrForBler(uint8_t al, double bler) const;
    /**
     * @brief The aggregation level for a UE: the lowest that meets TargetBler at the reported
     * SINR less SinrMarginDb, of those the search space has candidates for and the CORESET
     * can hold; the highest of them if none does
     * @param sinrDb the SINR the UE reported, NaN if it has not reported any
     * @param meetsTarget if not null: whether the level meets the target
     * @return the aggregation level, 0 if the CORESET holds none
     */
    uint8_t ChooseAl(double sinrDb, bool* meetsTarget = nullptr) const;

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
        std::shared_ptr<DciInfoElementTdma> dci;
    };

    void SetBlerTable(const std::string& table);
    std::string GetBlerTable() const;
    void SetCandidates(const std::string& candidates);
    std::string GetCandidatesString() const;
    void NewSlotIfNeeded();
    void FinishSlot();
    void DoFixSymbols();
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
    uint8_t m_initialAl{0};
    bool m_raiseAl{true};
    uint32_t m_sib1PeriodMs{20};
    double m_ulMaxShare{0.5};

    // the carrier
    uint32_t m_coresetRbs{0};
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
