// Copyright (c) 2011-2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>

#ifndef NR_PDCP_H
#define NR_PDCP_H

#include "nr-pdcp-sap.h"
#include "nr-rlc-sap.h"

#include "ns3/event-id.h"
#include "ns3/ipv4-address.h"
#include "ns3/nr-export.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/traced-value.h"

#include <string>
#include <vector>

// The ROHC library's compressor and decompressor (only pointers are kept here, so this header
// does not need the library's headers)
struct rohc_comp;
struct rohc_decomp;

namespace ns3
{

/**
 * LTE PDCP entity, see 3GPP TS 36.323
 */
class NR_EXPORT NrPdcp : public Object // SimpleRefCount<NrPdcp>
{
    /// allow NrPdcpSpecificNrRlcSapUser class friend access
    friend class NrPdcpSpecificNrRlcSapUser;
    /// allow NrPdcpSpecificNrPdcpSapProvider<NrPdcp> class friend access
    friend class NrPdcpSpecificNrPdcpSapProvider<NrPdcp>;

  public:
    NrPdcp();
    ~NrPdcp() override;
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    void DoDispose() override;

    /**
     *
     *
     * @param rnti
     */
    void SetRnti(uint16_t rnti);

    /**
     * @brief The cell of the bearer: with the RNTI it tells the bearers of all cells apart in the
     * counts of GetPacketCounts(), GetVoiceActivity() and the ROHC report
     * @param cellId the cell ID
     */
    void SetCellId(uint16_t cellId);

    /**
     *
     *
     * @param lcId
     */
    void SetLcId(uint8_t lcId);

    /**
     *
     *
     * @param s the PDCP SAP user to be used by this NR_PDCP
     */
    void SetNrPdcpSapUser(NrPdcpSapUser* s);

    /**
     *
     *
     * @return the PDCP SAP Provider interface offered to the RRC by this NR_PDCP
     */
    NrPdcpSapProvider* GetNrPdcpSapProvider();

    /**
     *
     *
     * @param s the RLC SAP Provider to be used by this NR_PDCP
     */
    void SetNrRlcSapProvider(NrRlcSapProvider* s);

    /**
     *
     *
     * @return the RLC SAP User interface offered to the RLC by this NR_PDCP
     */
    NrRlcSapUser* GetNrRlcSapUser();

    /// maximum PDCP SN
    static const uint16_t MAX_PDCP_SN = 4096;

    /**
     * Status variables of the PDCP
     */
    struct NR_EXPORT Status
    {
        uint16_t txSn; ///< TX sequence number
        uint16_t rxSn; ///< RX sequence number
    };

    /**
     *
     * @return the current status of the PDCP
     */
    Status GetStatus() const;

    /**
     * Set the status of the PDCP
     *
     * @param s
     */
    void SetStatus(Status s);

    /**
     * @return true if this build of the module contains the ROHC library, i.e. the
     * `EnableRohc` attribute can be set to true
     */
    static bool IsRohcAvailable();

    /**
     * Turn robust header compression (ROHC) on or off for this entity. Aborts when it is
     * turned on in a build without the ROHC library.
     * @param enable true to compress the IP headers of data radio bearers
     */
    void SetEnableRohc(bool enable);

    /**
     * @return true if ROHC is on for this entity
     */
    bool GetEnableRohc() const;

    /**
     * The ROHC mode (RFC 3095, 4.4) the decompressor of this entity works in. A compressor
     * always starts unidirectional and follows what the feedback of the peer decompressor
     * asks for. The library has no reliable mode (R-mode).
     */
    enum RohcMode
    {
        ROHC_UNIDIRECTIONAL, ///< U-mode: no feedback, the context is refreshed periodically
        ROHC_OPTIMISTIC      ///< O-mode: feedback in PDCP control PDUs repairs the context
    };

    /**
     * What stands behind `EnableRohc`
     */
    enum RohcModel
    {
        ROHC_MODEL_LIBRARY, ///< the ROHC library compresses and decompresses the real headers
        ROHC_MODEL_TABLE    ///< a table of header sizes, see the `RohcModel` attribute
    };

    /**
     * What became of the packets of the data radio bearers of one UE in one direction, added
     * up over all the connections (RNTIs) the UE had. Counted with and without ROHC.
     */
    struct NR_EXPORT PacketCounts
    {
        uint64_t sdus{0};      ///< packets the sending PDCP entities were given
        uint64_t sent{0};      ///< of these, sent on (the rest: the compressor failed)
        uint64_t discarded{0}; ///< of the sent ones, refused by the sending RLC (see below)
        uint64_t arrived{0};   ///< data PDUs that reached the receiving PDCP entities intact
        uint64_t delivered{0}; ///< of these, passed on (the rest: the decompressor failed)
        uint64_t damaged{0};   ///< data PDUs that reached them damaged and were dropped
    };

    /**
     * The packet counts of one UE since the simulation started, for telling apart the packets
     * that never got a radio bearer (sent by the application, not in `sdus`), those lost
     * between the two PDCP entities (`sent` - `arrived`) and those the ROHC decompressor could
     * not restore (`arrived` - `delivered`). `discarded` are the packets the sending RLC UM
     * entity did not take: the packet at the head of its queue had waited longer than the
     * discard timer (`NrRlcUm::DiscardTimerMs`; at the gNB by default the packet delay budget
     * of the 5QI of the bearer), or its buffer was full. They are part of `sent` - `arrived`;
     * the rest of that difference was lost on the air, or reached the receiving entity
     * `damaged`: not the bytes that were sent, because the RLC had put pieces of two packets
     * together (after a burst of losses as long as its sequence number space, it cannot tell).
     * The simulator knows and drops such a PDU; in a real receiver the UDP checksum would
     * reject the packet. A bearer belongs to the UE whose IPv4 address its first packet had:
     * the destination in downlink, the source in uplink. One cell only: the bearers are told
     * apart by RNTI and LCID. Read it before Simulator::Destroy().
     * @param ue the IPv4 address of the UE
     * @param downlink true for the packets to the UE, false for those from it
     * @return the counts
     */
    static PacketCounts GetPacketCounts(Ipv4Address ue, bool downlink);

    /**
     * For the RLC: it does not take the PDCP PDU it is being given right now (it discards it).
     * Counted in `PacketCounts::discarded` of the bearer, if it is a data PDU of a data radio
     * bearer. Called by NrRlcUm.
     */
    static void NotePduDiscarded();

    /**
     * For the RLC: the PDCP PDU it hands up next came in this many RLC PDUs (1 if it was not
     * segmented). Counted for the voice bearers, see GetVoiceActivity(). Called by NrRlc.
     * @param pdus the RLC PDUs
     */
    static void NoteSduPdus(uint32_t pdus);

    /// What a voice bearer carried last, told by the size of the packet
    enum SduKind : int8_t
    {
        SDU_UNKNOWN = -1, //!< nothing yet, or SilenceSduMaxBytes is 0
        SDU_SILENCE = 0,  //!< a silence descriptor: up to SilenceSduMaxBytes
        SDU_TALK = 1      //!< a voice frame: larger
    };

    /**
     * The kind of the last packet (SDU) the gNB had on a data radio bearer: the last it sent to
     * the UE (downlink) or the last it received from it (uplink). A gNB sees the size of the IP
     * packets of a voice bearer and with it whether the speaker talks or is silent; a scheduler
     * that adapts semi-persistent scheduling to that (NrMacSchedulerNs3::SpsScenario) asks here.
     * The bearers are told apart by cell, RNTI and LCID.
     * @param cellId the cell
     * @param rnti the UE
     * @param lcid the logical channel of the bearer
     * @param downlink true for the packets to the UE, false for those from it
     * @return the kind
     */
    static SduKind GetLastSduKind(uint16_t cellId, uint16_t rnti, uint8_t lcid, bool downlink);

    /// What the gNB has seen of the speaker of a voice bearer, see GetVoiceActivity()
    struct VoiceActivity
    {
        SduKind last{SDU_UNKNOWN}; //!< the kind of the last packet
        uint64_t sdus{0};          //!< the packets so far
        uint64_t toSilence{0};     //!< of these, silence descriptors that followed a voice frame
        uint64_t toVoice{0};       //!< and voice frames that followed a silence descriptor
        uint64_t toSilencePdus{0}; //!< uplink: the RLC PDUs the packets of toSilence came in
        uint64_t toVoicePdus{0};   //!< uplink: the RLC PDUs the packets of toVoice came in
    };

    /**
     * The same bearer and the same packets as GetLastSduKind(), counted: how many packets, and
     * how often the speaker fell silent or started to talk. A packet is counted when the gNB
     * sends it (downlink) or has received it whole (uplink).
     * @param cellId the cell
     * @param rnti the UE
     * @param lcid the logical channel of the bearer
     * @param downlink true for the packets to the UE, false for those from it
     * @return the counts
     */
    static VoiceActivity GetVoiceActivity(uint16_t cellId,
                                          uint16_t rnti,
                                          uint8_t lcid,
                                          bool downlink);

    /**
     * TracedCallback for PDU transmission event.
     *
     * @param [in] rnti The C-RNTI identifying the UE.
     * @param [in] lcid The logical channel id corresponding to
     *             the sending RLC instance.
     * @param [in] size Packet size.
     */
    typedef void (*PduTxTracedCallback)(uint16_t rnti, uint8_t lcid, uint32_t size);

    /**
     * TracedCallback signature for PDU receive event.
     *
     * @param [in] rnti The C-RNTI identifying the UE.
     * @param [in] lcid The logical channel id corresponding to
     *             the sending RLC instance.
     * @param [in] size Packet size.
     * @param [in] delay Delay since packet sent, in ns..
     */
    typedef void (*PduRxTracedCallback)(const uint16_t rnti,
                                        const uint8_t lcid,
                                        const uint32_t size,
                                        const uint64_t delay);

  protected:
    /**
     * Interface provided to upper RRC entity
     *
     * @param params the TransmitPdcpSduParameters
     */
    virtual void DoTransmitPdcpSdu(NrPdcpSapProvider::TransmitPdcpSduParameters params);

    NrPdcpSapUser* m_pdcpSapUser;         ///< PDCP SAP user
    NrPdcpSapProvider* m_pdcpSapProvider; ///< PDCP SAP provider

    /**
     * Interface provided to lower RLC entity
     *
     * @param p packet
     */
    virtual void DoReceivePdu(Ptr<Packet> p);

    NrRlcSapUser* m_rlcSapUser;         ///< RLC SAP user
    NrRlcSapProvider* m_rlcSapProvider; ///< RLC SAP provider

    uint16_t m_rnti;      ///< RNTI
    uint16_t m_cellId{0}; ///< cell of the bearer (SetCellId())
    uint8_t m_lcid;  ///< LCID

    /**
     * Used to inform of a PDU delivery to the RLC SAP provider.
     * The parameters are RNTI, LCID and bytes delivered
     */
    TracedCallback<uint16_t, uint8_t, uint32_t> m_txPdu;
    /**
     * Used to inform of a PDU reception from the RLC SAP user.
     * The parameters are RNTI, LCID, bytes delivered and delivery delay in nanoseconds.
     */
    TracedCallback<uint16_t, uint8_t, uint32_t, uint64_t> m_rxPdu;

  private:
    /**
     * State variables. See section 7.1 in TS 36.323
     */
    uint16_t m_txSequenceNumber;
    /**
     * State variables. See section 7.1 in TS 36.323
     */
    uint16_t m_rxSequenceNumber;

    /**
     * Constants. See section 7.2 in TS 36.323
     */
    static const uint16_t m_maxPdcpSn = 4095;

    /**
     * Create the ROHC compressor of this entity, the first time it sends an SDU
     */
    void CreateRohcCompressor();
    /**
     * Create the ROHC decompressor of this entity, the first time it receives a data PDU
     */
    void CreateRohcDecompressor();
    /**
     * Free the ROHC compressor and decompressor and drop the feedback not sent yet. Called
     * from DoDispose(), which ns-3 runs when a bearer is removed and the entity deleted.
     */
    void FreeRohc();
    /**
     * Keep feedback of the decompressor for the control PDU that leaves at the end of this event
     * @param data the ROHC feedback
     * @param size its length in bytes
     */
    void QueueRohcFeedback(const uint8_t* data, uint32_t size);
    /**
     * Send the waiting feedback to the peer entity, in a PDCP control PDU for interspersed
     * ROHC feedback (TS 38.323, 5.7.6.1 and 6.2.3.2)
     */
    void SendRohcFeedback();
    /**
     * Take a PDCP control PDU; interspersed ROHC feedback goes to the compressor (TS 38.323,
     * 5.7.6.2), anything else is dropped
     * @param p the control PDU
     */
    void ReceiveControlPdu(Ptr<Packet> p);

    /**
     * Send an SDU with the size-table model of ROHC: its header chain becomes as many bytes as
     * the table gives the packet
     * @param p the SDU
     * @return the PDU body, or null if the SDU is not IPv4/UDP and goes as it is
     */
    Ptr<Packet> CompressWithTable(Ptr<Packet> p);
    /**
     * Take a PDU body of the size-table model and put its header chain back, if the context
     * of the decompressor allows (`RohcTableDynamicWindow`, `RohcTableStaticWindow`); in O-mode
     * queue the feedback the packet calls for
     * @param p the PDU body
     * @return the SDU, or null if the context cannot restore the packet (the PDU is dropped)
     */
    Ptr<Packet> DecompressWithTable(Ptr<Packet> p);
    /**
     * @return the bits of the PDCP sequence number of this entity: `SnSize` on a data radio
     * bearer, 12 on a signalling bearer
     */
    uint8_t SnSizeBits() const;

    uint8_t m_snSize{12};                             ///< the `SnSize` attribute
    RohcModel m_rohcModel{ROHC_MODEL_LIBRARY};        ///< the `RohcModel` attribute
    uint32_t m_rohcTableUo0Size{3};                   ///< the `RohcTableUo0Size` attribute
    uint32_t m_rohcTableUpdateSize{13};               ///< the `RohcTableUpdateSize` attribute
    uint32_t m_rohcTableIrPackets{1};                 ///< the `RohcTableIrPackets` attribute
    uint32_t m_rohcTableDynamicWindow{13};            ///< the `RohcTableDynamicWindow` attribute
    uint32_t m_rohcTableStaticWindow{61};             ///< the `RohcTableStaticWindow` attribute
    // Size table, compressor
    std::string m_tableFlow;        ///< the flow that has a context
    uint32_t m_tableLastSize{0};    ///< bytes of its last packet
    uint32_t m_tableNumber{1};      ///< the number of its next packet
    uint32_t m_tableIrLeft{0};      ///< IR packets still to send
    bool m_tableUpdateDue{false};   ///< a NACK or a periodic refresh asked for a context update
    bool m_tableOMode{false};       ///< feedback asked for O-mode: no periodic refreshes
    uint32_t m_tableSinceIr{0};     ///< packets since the last IR
    uint32_t m_tableSinceUpdate{0}; ///< packets since the last IR or context update
    Time m_tableLastIr{0};          ///< when the last IR left
    Time m_tableLastUpdate{0};      ///< when the last IR or context update left
    // Size table, decompressor
    bool m_tableRxStatic{false};  ///< the static context is valid
    bool m_tableRxDynamic{false}; ///< the dynamic context is valid
    uint32_t m_tableRxNumber{0};  ///< the number of the last packet that was restored
    uint32_t m_tableRxFailed{0};  ///< packets in a row that could not be restored
    uint8_t m_tableRxLastFeedback{0};    ///< the feedback sent last: 0 none, 1 NACK, 2 STATIC-NACK
    uint32_t m_tableRxSinceFeedback{0};  ///< failed packets since it was sent
    bool m_enableRohc{false};                         ///< the `EnableRohc` attribute
    RohcMode m_rohcMode{ROHC_UNIDIRECTIONAL};         ///< the `RohcMode` attribute
    uint32_t m_rohcMaxCid{15};                        ///< the `RohcMaxCid` attribute
    std::string m_rohcProfiles{"0x0000,0x0001,0x0002,0x0004"}; ///< the `RohcProfiles` attribute
    uint32_t m_rohcRepetitions{4};                    ///< the `RohcRepetitions` attribute
    uint32_t m_rohcIrRefresh{1700};                   ///< the `RohcIrRefresh` attribute
    uint32_t m_rohcFoRefresh{700};                    ///< the `RohcFoRefresh` attribute
    Time m_rohcIrRefreshTime{0};                      ///< the `RohcIrRefreshTime` attribute
    Time m_rohcFoRefreshTime{0};                      ///< the `RohcFoRefreshTime` attribute
    bool m_rohcCrcRepair{false};                      ///< the `RohcCrcRepair` attribute
    uint32_t m_rohcNackFailures{10};                  ///< the `RohcNackFailures` attribute
    uint32_t m_rohcFeedbackSpacing{32};               ///< the `RohcFeedbackSpacing` attribute
    uint32_t m_silenceSduMaxBytes{0};                 ///< the `SilenceSduMaxBytes` attribute
    struct rohc_comp* m_rohcCompressor{nullptr};      ///< for the SDUs this entity sends
    struct rohc_decomp* m_rohcDecompressor{nullptr};  ///< for the PDUs this entity receives
    int m_lastRohcState{0}; ///< compressor state of the last SDU sent (rohc_comp_state_t)
    std::vector<uint8_t> m_rohcFeedback; ///< feedback waiting for its control PDU
    EventId m_rohcFeedbackEvent;         ///< sends that control PDU
};

} // namespace ns3

#endif // NR_PDCP_H
