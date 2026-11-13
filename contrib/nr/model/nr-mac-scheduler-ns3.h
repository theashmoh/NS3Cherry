// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#pragma once

#include "nr-amc.h"
#include "nr-fh-sched-sap.h"
#include "nr-mac-harq-vector.h"
#include "nr-mac-scheduler-cqi-management.h"
#include "nr-mac-scheduler-lcg.h"
#include "nr-mac-scheduler-ue-info.h"
#include "nr-mac-scheduler.h"
#include "nr-pdcch.h"
#include "nr-phy-mac-common.h"

#include "ns3/nr-export.h"
#include "ns3/traced-callback.h"

#include <deque>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <set>

namespace ns3
{

class NrSchedGeneralTestCase;
class NrSchedOfdmaSymbolPerBeamTestCase;
class NrTestMacSchedulerHarqRrReshape;
class NrTestMacSchedulerHarqRrScheduleDlHarq;
class NrMacSchedulerHarqRr;
class NrMacSchedulerSrsDefault;
class NrMacSchedulerLcAlgorithm;
class NrFhSchedSapUser;
class NrFhSchedSapProvider;

/**
 * @ingroup scheduler
 * @brief A general scheduler for nr in NS3
 *
 * This abstract class is taking care of creating a solid base for any schedulers
 * in the nr world. The class implements all the API from the FemtoForum API,
 * but in doing so, it defines a new interface that must be followed when designing
 * and writing a new scheduler type.
 *
 * The architecture has a unique representation of a UE, that is valid across
 * all the schedulers. Each one can expand the definition, adding values or
 * functions to call while doing the scheduler job.
 * The base class is defined as NrMacSchedulerUeInfo. Please refer to
 * its documentation to know the default values, and how to use or expand it.
 *
 * The documentation continues by following every step involved in the scheduling.
 * Please refer to the function documentation to see a detailed description
 * of the steps done during all the phases.
 *
 * @section scheduler_registration Registration and Configuration
 *
 * The attribute of any scheduler can be set directly calling `SetAttribute`
 * on the pointer obtained through NrHelper::GetScheduler() or, before
 * it is created, through NrHelper::SetSchedulerAttribute(). The type of
 * the scheduler can be set only before its creation, through the method
 * NrHelper::SetSchedulerTypeId().
 *
 * @section scheduler_user_management User management (creation and removal)
 *
 * When a user arrives in the cell, it is registered with a call to
 * DoCschedUeConfigReq. When the user leaves, the class is made aware with
 * a call to DoCschedUeReleaseReq. The only important operation is the creation
 * of a UE representation and its storage in the general UE map (m_ueMap).
 *
 * A UE is represented through the class NrMacSchedulerUeInfo, which is
 * used in the internals of the general base class to store and retrieve
 * information such as Logical Channels, CQI, and other things. Please refer
 * to its documentation for a broader overview of its possibilities.
 *
 * @section scheduler_cell_conf Cell configuration
 *
 * The cell configuration, done with a call to DoCschedCellConfigReq, is ignored.
 *
 * @section scheduler_lc_creation LC creation and removal
 *
 * After the registration of a UE, the scheduler has to know how many bytes
 * there are in its queue, for both uplink and downlink. Before that,
 * the scheduler has to know how many Logical Channels are present
 * for each UE (DL and UL). Each time an LC is created, the MAC calls
 * the function DoCschedLcConfigReq(). Please refer to the documentation
 * of NrMacSchedulerUeInfo to know the details of the LC and LC Groups
 * representation in the code. The LC can be deleted with a call to
 * DoCschedLcReleaseReq (currently not implemented).
 *
 * A subclass of NrMacSchedulerNs3 can change the representation
 * of an LC and LCG by merely creating an appropriate subclass
 * of NrMacSchedulerLC or NrMacSchedulerLCG (the classes used by the
 * default implementation to store information about the LC or LCG) and then
 * reimplementing the methods CreateLCG() and CreateLC() to return a pointer to
 * a created instance.
 *
 * @section scheduler_update_lc Updating the LC bytes
 *
 * For the downlink case, the LC is updated with a message between the gNB RLC
 * layer and the MAC.  The scheduler receives a call to the method
 * DoSchedDlRlcBufferReq(), and inside this method is updating all the LC amount.
 *
 * For the uplink case, there are more passages involved. In the scheduler,
 * however, the important this is that is called the method DoSchedUlMacCtrlInfoReq().
 * Inside this method, the BSR coming from UE is used to update the LC.
 * More details can be found in the documentation of the class NrMacSchedulerLCG
 * and NrMacSchedulerLC.
 *
 * @section scheduler_cqi CQI Management
 *
 * The CQI is based on a parameter (m_cqiTimersThreshold) that indicates how
 * long a received CQI is valid. Every time that a Dl CQI is received, the
 * MAC calls the function DoSchedDlCqiInfoReq. In here, the CQI list is
 * traversed and each CQI is reported to the class NrMacSchedulerCQIManagement
 * that is responsible for calculating the CQI. The value is then stored inside
 * the UE representation, ready to be read in the future. The CQI is reset to
 * the default value once the validity timer expires. The default value permits
 * only to have an MCS value 0.
 *
 * For the UL case, the MAC is calling the method DoSchedUlCqiInfoReq(). The CQI
 * and MCS values are then derived by using a vector of SINR (see the documentation
 * of the function for the details).
 *
 * At the end of these evaluations, inside the UE representation, is available
 * the value of the DL/UL MCS, ready to be used.
 *
 * @section scheduler_scheduling_general Scheduling phase
 *
 * After gathering the information regarding CQI, active users and flows, it is
 * time to take a look into how the class manages the most important thing,
 * the scheduling. The work is about deciding how to fill the frequency/time
 * space, assigning resources to HARQ retransmission or DL/UL new data transmission.
 * The main scheduling function is NrMacSchedulerNs3::DoSchedTriggerReq.
 *
 * @section scheduler_cqi Refreshing CQI
 *
 * The refreshing of CQI consists in evaluating the validity timer of the value.
 * If the timer is equal to 0, the valued is expired, and the value is reset
 * to the default (MCS 0). The operation is managed inside the class
 * NrMacSchedulerCQIManagement, with the two functions
 * NrMacSchedulerCQIManagement::RefreshDlCQIMaps and
 * NrMacSchedulerCQIManagement::RefreshUlCQIMaps.
 *
 * @section scheduler_process_harq Process HARQ feedbacks
 *
 * To decide if it is necessary to perform HARQ retransmission, and to decide
 * how many retransmission perform, the first step is to evaluate the HARQ
 * feedback received as input. The UEs are reporting the feedbacks, and these
 * feedbacks are merged with the feedback of the previous slots that could not
 * be transmitted (function MergeDlHARQ()). Then, the code evaluates these
 * feedbacks by resetting HARQ processes with an ACK and preparing for
 * the retransmission of the HARQ processes marked with NACK (ProcessHARQFeedbacks())
 * for both UL and DL HARQs.
 *
 * At the end of the process, the code evaluates the HARQ timers, and reset the
 * processes with an expired timer (ResetExpiredHARQ()).
 *
 * To discover more about how HARQ processes are stored and managed, please take
 * a look at the HarqProcess and NrMacHarqVector documentation.
 *
 * @section scheduler_general The concept of scheduling
 *
 * The scheduling of the resources is a process that should fill the slot time
 * and the slot frequencies with retransmitted or fresh data. To simplify the
 * model, you could think that there is a 2D plan in which the "y" values are
 * the frequencies, and the "x" value is the time that passes. The plan should be
 * filled with data, or better said, with a series of blocks. Each block is the
 * minimum assignable resource, called Resource Block Group. There are constraints
 * on how these RBG can be distributed, and often these constraints follow some
 * limitation in the UE equipment, but also they are in place to limit the complexity
 * of a problem which is computationally hard. The position of each block is
 * defined by a starting point (PointInFTPlane) which is a pair (x,y) (or better
 * said (sym, rb)) that represent a point from which a block should be positioned.
 *
 * Please refer to the DoScheduling() documentation to know how the
 * scheduling is performed.
 *
 * @section scheduler_spat_mult Spatial multiplexing
 *
 * The code does not support Spatial Multiplexing. It means that it is not possible
 * to schedule UEs that are in different beams at the same time. While this has
 * no practical effects on a time-based distribution of resources, it affects
 * the frequency-based distribution. Therefore, to support both operational modes,
 * the scheduler should compute beforehand the number of active UEs, as well as
 * the number of retransmission to be done. These operations are done, respectively,
 * by the methods ComputeActiveUe() and ComputeActiveHarq(). These methods work on
 * data structures that group UE and retransmission by BeamID
 * (ActiveUeMap and ActiveHarqMap).
 *
 * @section scheduler_sched_ul Scheduling UL
 * It is worth explaining that the
 * schedulers working on slot x for DL, are working on slot \f$x + y\f$
 * (where y is the value of N2 delay).
 * This delay is implemented to simulate the fact that the UE receives the DCI at
 * time \f$t\f$, and then has some time (the delay) to prepare its UL data.
 * So, if the scheduler assigns some symbols for uplink data in slot \f$x + y\f$,
 * after y slots have passed (and so the scheduler is preparing the slot $x+y$
 * for DL data and HARQ retransmission) the scheduler has to remember that there
 * are fewer symbols available. Moreover, it is necessary to not overlap the
 * decisions for DL on top of the (already taken) decision for UL.
 *
 * All this details are considered in the functions ScheduleUl() and ScheduleDl().
 *
 *
 * @section scheduler_harq HARQ
 *
 * The HARQ scheduling is done, if symbols for HARQ are available, before transmitting
 * new data, and this happens for both DL and UL. The detailed documentation
 * is available in the methods ScheduleDlHarq() and ScheduleUlHarq().
 *
 * @section scheduler_sched Scheduling new data
 *
 * The scheduling of new data is performed by functions ScheduleUlData() and
 * ScheduleDlData(). The priority is for HARQ retransmission, so if the
 * retransmissions fill the slot time, there will no symbols available for
 * new data.
 * Please refer to the method documentation for more detailed information
 * about the scheduling process of new data.
 *
 * The scheduler distributes the assigned bytes among the different LCs of
 * a UE based on the chosen algorithm for LC bytes assignment. Currently two
 * algorithms are implemented. The default algorithm that assigns bytes to
 * LCs in RR fashion and an algorithm that shares bytes among the active LCs
 * by taking into account the resource type and the e_rabGuaranteedBitRate of
 * a flow. These two algorithm implementations can be found in the
 * NrMacSchedulerLcRR and NrMacSchedulerLcQos classes.
 *
 * The available schedulers are TDMA and OFDMA version of the Round Robin,
 * Proportional Fair, Maximum Rate, and QoS MAC scheduler.
 *
 * @see NrMacSchedulerOfdmaPF
 * @see NrMacSchedulerOfdmaRR
 * @see NrMacSchedulerOfdmaMR
 * @see NrMacSchedulerOfdmaQos
 * @see NrMacSchedulerTdmaPF
 * @see NrMacSchedulerTdmaRR
 * @see NrMacSchedulerTdmaMR
 * @see NrMacSchedulerTdmaQos
 */
class NR_EXPORT NrMacSchedulerNs3 : public NrMacScheduler
{
    friend class NrKoffsetHarqTestCase;
    friend class NrTestSchedulerAiCase;
    friend class NrSchedOfdmaMcsTestCase;
    friend class NrSchedOfdmaSymbolPerBeamTestCase;
    friend class NrSchedGeneralTestCase;
    friend class NrTestMacSchedulerHarqRrReshape;
    friend class NrTestMacSchedulerHarqRrScheduleDlHarq;
    friend class NrTestMacSchedulerHarqRrBeamOrder;
    friend class NrTestMacSchedulerHarqRrSymbolBudget;
    friend class NrTestMacSchedulerHarqRrOfdmaSharing;
    friend class NrTestMacSchedulerHarqProcessTimeout;
    friend class NrTestMacSchedulerDlOlla;
    friend class NrTestMacSchedulerDlHarqFeedbackDisabled;

  public:
    /**
     * @brief GetTypeId
     * @return The TypeId of the class
     */
    static TypeId GetTypeId();

    /**
     * @brief NrMacSchedulerNs3 default constructor
     */
    NrMacSchedulerNs3();

    /**
     * @brief NrMacSchedulerNs3 copy constructor (deleted)
     * @param other instance of NrMacSchedulerNs3 to be copied
     */
    NrMacSchedulerNs3(const NrMacSchedulerNs3& other) = delete;

    /**
     * @brief NrMacSchedulerNs3 deconstructor
     */
    ~NrMacSchedulerNs3() override;

    // FH Control SAPs
    void SetNrFhSchedSapProvider(NrFhSchedSapProvider* s) override;
    NrFhSchedSapUser* GetNrFhSchedSapUser() override;

    /**
     * @brief Install the AMC for the DL part
     * @param dlAmc DL AMC
     *
     * Usually called by the helper
     */
    void InstallDlAmc(const Ptr<NrAmc>& dlAmc);

    /**
     * @brief Install the AMC for the DL part
     * @param ulAmc DL AMC
     *
     * Usually called by the helper
     */
    void InstallUlAmc(const Ptr<NrAmc>& ulAmc);

    /**
     * @brief Get the AMC for UL
     * @return the UL AMC
     */
    Ptr<const NrAmc> GetUlAmc() const;

    /**
     * @brief Get the AMC for DL
     * @return the DL AMC
     */
    Ptr<const NrAmc> GetDlAmc() const;

    /**
     * @brief Point in the Frequency/Time plane
     *
     * The first element represents the RBG, and the second element represents the symbol. The
     * struct represents a point in the 2D time frequency space created by having frequencies on the
     * y and time on the x.
     */
    struct NR_EXPORT PointInFTPlane
    {
        /**
         * @brief PointInFTPlane constructor
         * @param rbg RBG
         * @param sym Symbol
         */
        PointInFTPlane(uint32_t rbg, uint8_t sym)
            : m_rbg(rbg),
              m_sym(sym)
        {
        }

        uint32_t m_rbg; //!< Represent the starting RBG
        uint8_t m_sym;  //!< Represent the starting symbol
    };

    typedef PointInFTPlane
        FTResources; //!< Represent an amount of RBG/symbols that can be, or is, assigned

    void DoCschedCellConfigReq(
        const NrMacCschedSapProvider::CschedCellConfigReqParameters& params) override;
    void DoCschedUeConfigReq(
        const NrMacCschedSapProvider::CschedUeConfigReqParameters& params) override;
    void DoSchedDlRlcBufferReq(
        const NrMacSchedSapProvider::SchedDlRlcBufferReqParameters& params) override;
    void DoSchedUlMacCtrlInfoReq(
        const NrMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters& params) override;
    void DoSchedDlCqiInfoReq(
        const NrMacSchedSapProvider::SchedDlCqiInfoReqParameters& params) override;
    void DoSchedUlCqiInfoReq(
        const NrMacSchedSapProvider::SchedUlCqiInfoReqParameters& params) override;
    void DoCschedUeReleaseReq(
        const NrMacCschedSapProvider::CschedUeReleaseReqParameters& params) override;
    void DoCschedLcConfigReq(
        const NrMacCschedSapProvider::CschedLcConfigReqParameters& params) override;
    void DoCschedLcReleaseReq(
        const NrMacCschedSapProvider::CschedLcReleaseReqParameters& params) override;
    void DoSchedDlTriggerReq(
        const NrMacSchedSapProvider::SchedDlTriggerReqParameters& params) override;
    /// DIAG: abort with a message if a pending UL retransmission has no live process
    void DiagCheckUlFeedback(std::vector<UlHarqInfo>* ulHarqFeedback, const char* stage) const;
    /// DIAG: keep the last events of the UL HARQ bookkeeping of this scheduler
    void DiagLog(const std::string& event) const;
    /// DIAG: the events kept, one per line
    std::string DiagEvents(uint16_t rnti) const;
    mutable std::deque<std::string> m_diagLog; //!< DIAG: the last UL HARQ events
    bool m_diagUlHarq{false};                  //!< DiagUlHarq attribute
    void DoSchedUlTriggerReq(
        const NrMacSchedSapProvider::SchedUlTriggerReqParameters& params) override;
    void DoSchedUlSrInfoReq(
        const NrMacSchedSapProvider::SchedUlSrInfoReqParameters& params) override;
    void DoSchedSetMcs(uint32_t mcs) override;
    void DoSchedDlRachInfoReq(
        const NrMacSchedSapProvider::SchedDlRachInfoReqParameters& params) override;
    uint8_t GetDlCtrlSyms() const override;
    uint8_t GetUlCtrlSyms() const override;
    bool IsMaxSrsReached() const override;

    /**
     * @brief Assign a fixed random variable stream number to the random variables
     * used by this model. Return the number of streams (possibly zero) that
     * have been assigned.
     *
     * @param stream first stream index to use
     * @return the number of stream indices assigned by this model
     */
    int64_t AssignStreams(int64_t stream) override;

    // to save some typing
    using HarqVectorIterator = NrMacHarqVector::iterator;
    using HarqVectorIteratorList = std::vector<HarqVectorIterator>;

    /**
     * @brief Pair between a pointer to NrMacSchedulerUeInfo and its buffer occupancy
     */
    typedef std::pair<UePtr, uint32_t> UePtrAndBufferReq;
    /**
     * @brief Map between a BeamId and a vector of UE (the UE are in that beam)
     */
    typedef std::unordered_map<BeamId, std::vector<UePtrAndBufferReq>, BeamIdHash> ActiveUeMap;
    /**
     * @brief Map between a BeamId and the symbol assigned to that beam
     */
    typedef std::unordered_map<BeamId, uint32_t, BeamIdHash> BeamSymbolMap;
    /**
     * @brief Map between a beamID and the HARQ of that beam
     */
    typedef std::unordered_map<BeamId, HarqVectorIteratorList, BeamIdHash> ActiveHarqMap;

    /**
     * @brief Set the CqiTimerThreshold
     * @param v the value to set
     */
    void SetCqiTimerThreshold(const Time& v);
    /**
     * @brief Get the CqiTimerThreshold
     * @return the value
     */
    Time GetCqiTimerThreshold() const;

    /**
     * @brief Set if the MCS in DL is fixed (in case, it will take the starting value)
     * @param v the value
     * @see SetStartMcsDl
     */
    void SetFixedDlMcs(bool v);
    /**
     * @brief Check if the MCS in DL is fixed
     * @return true if the DL MCS is fixed, false otherwise
     */
    bool IsDlMcsFixed() const;

    /**
     * @brief Set if the MCS in UL is fixed (in case, it will take the starting value)
     * @param v the value
     * @see SetStartMcsUl
     */
    void SetFixedUlMcs(bool v);
    /**
     * @brief Check if the MCS in UL is fixed
     * @return true if the UL MCS is fixed, false otherwise
     */
    bool IsUlMcsFixed() const;

    /**
     * @brief Set the starting value for the DL MCS
     * @param v the value
     */
    void SetStartMcsDl(uint8_t v);
    /**
     * @brief Get the DL MCS starting value
     * @return the value
     */
    uint8_t GetStartMcsDl() const;

    /**
     * @brief Set the maximum index for the DL MCS
     * @param v the value
     */
    void SetMaxDlMcs(int8_t v);
    /**
     * @brief Get the maximum DL MCS index
     * @return the value
     */
    int8_t GetMaxDlMcs() const;

    /**
     * @brief Set LC Scheduler Algorithm model type
     * @param type the LC Scheduler Algorithm Error model type
     */
    void SetLcSched(const TypeId& type);

    /**
     * @brief Set the starting value for the UL MCS
     * @param v the value
     */
    void SetStartMcsUl(uint8_t v);
    /**
     * @brief Get the DL MCS starting value
     * @return the value
     */
    uint8_t GetStartMcsUl() const;

    /**
     * @brief Set the number of DL ctrl symbols
     * @param v number of DL ctrl symbols
     */
    void SetDlCtrlSyms(uint8_t v);

    /**
     * @brief Set the number of UL ctrl symbols
     * @param v number of UL ctrl symbols
     */
    void SetUlCtrlSyms(uint8_t v);

    /**
     * @brief Set the notched (blank) RBGs Mask for the DL
     * @param dlNotchedRbgsMask The mask of notched RBGs
     */
    void SetDlNotchedRbgMask(const std::vector<bool>& dlNotchedRbgsMask);

    /**
     * @brief Get the notched (blank) RBGs Mask for the DL
     * @return The mask of notched RBGs
     */
    std::vector<bool> GetDlNotchedRbgMask() const;

    /**
     * @brief Set the notched (blank) RBGs Mask for the UL
     * @param ulNotchedRbgsMask The mask of notched RBGs
     */
    void SetUlNotchedRbgMask(const std::vector<bool>& ulNotchedRbgsMask);

    /**
     * @brief Get the notched (blank) RBGs Mask for the UL
     * @return The mask of notched RBGs
     */
    std::vector<bool> GetUlNotchedRbgMask() const;

    /**
     * @brief Set the number of UL SRS symbols
     * @param v number of SRS symbols
     */
    void SetSrsCtrlSyms(uint8_t v);

    /**
     * @brief Get the configured value for the SRS symbols
     * @return the number of SRS symbols that will be allocated
     */
    uint8_t GetSrsCtrlSyms() const;

    /**
     * @brief Set if the UL slots are allowed for SRS transmission (if True, UL
     * and F slots may carry SRS, if False, SRS are transmitted only in F slots)
     * @param v the value
     */
    void SetSrsInUlSlots(bool v);

    /**
     * @brief Check if the UL slots are allowed for SRS transmission
     * @return true if UL slots are available for SRS (UL and F slots available
     * for SRS), false otherwise (only F slots available for SRS)
     */
    bool IsSrsInUlSlots() const;

    /**
     * @brief Set if the F slots are allowed for SRS transmission
     * @param v the value
     */
    void SetSrsInFSlots(bool v);

    /**
     * @brief Check if the F slots are allowed for SRS transmission
     * @return true if F slots are available for SRS, false otherwise
     */
    bool IsSrsInFSlots() const;
    /**
     * @brief Enable HARQ ReTx function
     *
     * Remember we introduced the EnableHarqReTx attribute only
     * for FB calibration example. We want to disable HARQ ReTx
     * because ReTx are scheduled in OFDMA fashion. In a TDMA simulation,
     * such retransmissions change the SINR trends in a scenario. Also,
     * REMEMBER that this solution of disabling the HARQ ReTx not very
     * optimized because gNB MAC will still buffer the packet and UE
     * would still transmit the HARQ feedback for the first transmission.
     *
     * @param enableFlag If true, it would set the max HARQ ReTx to 3; otherwise it set it to 0
     */
    void EnableHarqReTx(bool enableFlag);
    /**
     * @brief Is HARQ ReTx enable function
     *
     * @return Returns true if HARQ ReTx are enabled; otherwise false
     */
    bool IsHarqReTxEnable() const override;

    /**
     * @brief Sets the default RACH UL
     * grant MCS
     * @param v the MCS to be used for RACH UL grant
     */
    void SetRachUlGrantMcs(uint8_t v);
    /**
     * @brief Set the attribute PuschRepetitions: 0 or a number of repetitions of TS 38.331
     * (NrMacSchedulerUeInfo::kPuschRepetitionValues); any other aborts
     * @param k the repetitions
     */
    void SetPuschRepetitions(uint32_t k);
    /// @return the attribute PuschRepetitions
    uint32_t GetPuschRepetitions() const;

  protected:
    /**
     * @brief Create an UE representation for the scheduler.
     *
     * The representation must save any important UE-specific value, and it is
     * shared across all the subclasses. A scheduler which want to save any
     * additional value per-UE, must subclass the class NrMacSchedulerUeInfo
     * and return a pointer to an instance of the new type.
     *
     * @param params Configure parameters for the UE
     * @return a pointer to the UE representation to save in the UE map (m_ueMap).
     */
    virtual std::shared_ptr<NrMacSchedulerUeInfo> CreateUeRepresentation(
        const NrMacCschedSapProvider::CschedUeConfigReqParameters& params) const = 0;

    /**
     * @brief Returns TPC command
     */
    virtual uint8_t GetTpc() const = 0;

    /**
     * @brief Giving the input, append to slotAlloc the allocations for the DL HARQ retransmissions
     * @param startingPoint starting point of the first retransmission.
     * It should be set to the next available starting point
     * @param symAvail Available symbols
     * @param activeDlHarq Map of the active HARQ processes
     * @param ueMap Map of the UEs
     * @param dlHarqToRetransmit HARQ feedbacks that could not be transmitted (to fill)
     * @param dlHarqFeedback all the HARQ feedbacks
     * @param slotAlloc Slot allocation info
     * @return the VarTtiSlotAlloc ID to use next
     */
    virtual uint8_t ScheduleDlHarq(NrMacSchedulerNs3::PointInFTPlane* startingPoint,
                                   uint8_t symAvail,
                                   const ActiveHarqMap& activeDlHarq,
                                   const std::unordered_map<uint16_t, UePtr>& ueMap,
                                   std::vector<DlHarqInfo>* dlHarqToRetransmit,
                                   const std::vector<DlHarqInfo>& dlHarqFeedback,
                                   SlotAllocInfo* slotAlloc) const;
    /**
     * @brief Giving the input, append to slotAlloc the allocations for the DL HARQ retransmissions
     * @param startingPoint starting point of the first retransmission.
     * It should be set to the next available starting point
     * @param symAvail Available symbols
     * @param ueMap Map of the UEs
     * @param ulHarqToRetransmit HARQ feedbacks that could not be transmitted (to fill)
     * @param ulHarqFeedback all the HARQ feedbacks
     * @param slotAlloc Slot allocation info
     * @return the VarTtiSlotAlloc ID to use next
     */
    virtual uint8_t ScheduleUlHarq(NrMacSchedulerNs3::PointInFTPlane* startingPoint,
                                   uint8_t symAvail,
                                   const std::unordered_map<uint16_t, UePtr>& ueMap,
                                   std::vector<UlHarqInfo>* ulHarqToRetransmit,
                                   const std::vector<UlHarqInfo>& ulHarqFeedback,
                                   SlotAllocInfo* slotAlloc) const;

    /**
     * @brief Assign the DL RBG to the active UE, and return the distribution of symbols per beam
     * @param symAvail available symbols for DL
     * @param activeDl Map of Beam and active UE per beam
     * @return a map of symbols dedicated to each beam
     *
     * The function should assign RBG to each UE, modifying the value m_dlRBG
     * for each UE in the activeDl map. In doing so, it has to calculate the number
     * of symbols assigned to each beam, and return it to the caller.
     *
     * The creation of DCI will be performed by calling CreateDlDci with
     * the appropriate input parameters.
     */
    virtual BeamSymbolMap AssignDLRBG(uint32_t symAvail, const ActiveUeMap& activeDl) const = 0;

    /**
     * @brief Put the UEs in the order the scheduling policy would serve them at the start of a
     * slot, best first; UEs the policy ranks the same keep the order they come in
     * @param ues the UEs with new data, the one served longest ago first (in, out)
     * @param isDl true for downlink
     *
     * Used with the PDCCH model, where a slot has DCIs for a limited number of UEs: these are
     * the first of this order. The default leaves the UEs as they are.
     */
    virtual void SortByPolicy(std::vector<UePtrAndBufferReq>* ues, bool isDl) const;
    /**
     * @brief A UE with new data that got no DCI in this slot: what the policy does for a UE
     * that was not assigned anything (a proportional fair scheduler lowers its average rate)
     * @param ue the UE
     * @param isDl true for downlink
     */
    virtual void PassedOver(const UePtrAndBufferReq& ue, bool isDl) const;

    /**
     * @brief Assign the UL RBG to the active UE, and return the distribution of symbols per beam
     * @param symAvail available symbols for UL
     * @param activeUl Map of Beam and active UE per beam
     * @return a map of symbols dedicated to each beam
     *
     * The function should assign RBG to each UE, modifying the value m_ulRBG
     * for each UE in the activeUl map. In doing so, it has to calculate the number
     * of symbols assigned to each UE, and return it to the caller.
     *
     * The creation of DCI will be performed by calling CreateDlDci with
     * the appropriate input parameters.
     */
    virtual BeamSymbolMap AssignULRBG(uint32_t symAvail, const ActiveUeMap& activeUl) const = 0;

    /**
     * @brief Create a DCI for the specified UE for DL data
     * @param spoint Starting point
     * @param ueInfo UE specified
     * @param maxSym maximum amount of symbols that can be assigned
     * @return a pointer to the DciInfoElementTdma
     *
     * The function should create a block in the 2D frequency-time plane in
     * which the specified UE will receive the DL data.
     */
    virtual std::shared_ptr<DciInfoElementTdma> CreateDlDci(
        PointInFTPlane* spoint,
        const std::shared_ptr<NrMacSchedulerUeInfo>& ueInfo,
        uint32_t maxSym) const = 0;

    /**
     * @brief Create a DCI for the specified UE for UL data
     * @param spoint Starting point
     * @param ueInfo UE specified
     * @param maxSym maximum amount of symbols that can be assigned
     * @return a pointer to the DciInfoElementTdma
     *
     * The function should create a block in the 2D frequency-time plane in
     * which the specified UE will receive the UL data.
     */
    virtual std::shared_ptr<DciInfoElementTdma> CreateUlDci(
        PointInFTPlane* spoint,
        const std::shared_ptr<NrMacSchedulerUeInfo>& ueInfo,
        uint32_t maxSym) const = 0;

    /**
     * @brief Perform a custom operation on the starting point each time all the UE of a DL beam
     * have been scheduled \param spoint starting point for the next beam to modify \param symOfBeam
     * number the symbol used for the beam
     */
    virtual void ChangeDlBeam(PointInFTPlane* spoint, uint32_t symOfBeam) const = 0;

    /**
     * @brief Perform a custom operation on the starting point each time all the UE of an UL beam
     * have been scheduled \param spoint starting point for the next beam to modify \param symOfBeam
     * number the symbol used for the beam
     */
    virtual void ChangeUlBeam(PointInFTPlane* spoint, uint32_t symOfBeam) const = 0;

    /**
     * @brief The most UEs that can get new data in one slot, whatever their PDCCH
     *
     * With the PDCCH model on (NrPdcch) a UE needs a PDCCH candidate before it is given data
     * resources, and candidates are not reserved for more UEs than the data region can serve.
     *
     * @param symAvail symbols left for data
     * @return the number of UEs; the default is no limit
     */
    virtual uint32_t GetMaxUesPerSlot(uint32_t symAvail) const;

    /**
     * @brief The shape of a downlink block of a semi-persistent configuration (SpsScenario)
     * @return true if it takes all resource blocks of its symbols, as every block of a TDMA
     * scheduler does; false (the default) if it takes as few as it needs and shares its symbols
     * with the blocks of other UEs of its beam. (An uplink block has symbols of its own, unless
     * UlGrantRbs makes it narrow: then it shares them as a downlink block does.)
     */
    virtual bool SpsUsesFullBand() const;

    /**
     * @brief Sort the DL HARQ retransmission
     * @param activeDlHarq HARQ DL to retransmit
     *
     * The HARQ are divided by beams. In each beam, the HARQ should be ordered
     * in a way that the first element should be the first to transmit, and
     * then (if there is space) the second, the third, and so on.
     */
    virtual void SortDlHarq(ActiveHarqMap* activeDlHarq) const;

    /**
     * @brief Sort the UL HARQ retransmission
     * @param activeUlHarq HARQ UL to retransmit
     *
     * The HARQ are divided by beams. In each beam, the HARQ should be ordered
     * in a way that the first element should be the first to transmit, and
     * then (if there is space) the second, the third, and so on.
     */
    virtual void SortUlHarq(ActiveHarqMap* activeUlHarq) const;

    virtual LCGPtr CreateLCG(const nr::LogicalChannelConfigListElement_s& config) const;

    virtual LCPtr CreateLC(const nr::LogicalChannelConfigListElement_s& config) const;

  public:
    /**
     * @brief Private function that is used to get the number of resource
     * blocks per resource block group and also to check whether this value is
     * configured.
     * @return Returns the number of RBs per RBG
     */
    uint64_t GetNumRbPerRbg() const;

  protected:
    Ptr<NrAmc> m_dlAmc; //!< AMC pointer
    Ptr<NrAmc> m_ulAmc; //!< AMC pointer
    NrMacSchedulerUeInfo::McsCsiSource
        m_mcsCsiSource; //!< CSI information source for DL MCS estimation

    bool m_activeDlAi{false}; //!< Flag for activating AI for downlink
    bool m_activeUlAi{false}; //!< Flag for activating AI for uplink

  private:
    /**
     * @brief Single UL allocation for calculating CQI and the number of reserved UL symbols in
     * slots.
     */
    struct NR_EXPORT AllocElem
    {
        /**
         * @brief AllocElem empty constructor (deleted)
         */
        AllocElem() = delete;
        /**
         * @brief AllocElement default copy constructor
         */
        AllocElem(const AllocElem& o) = default;

        /**
         * @brief AllocElem constructor
         * @param rnti RNTI
         * @param tbs Transport Block Size
         * @param numSym Number of symbols
         * @param mcs MCS
         * @param rank rank
         */
        AllocElem(uint16_t rnti,
                  uint32_t tbs,
                  uint8_t symStart,
                  uint8_t numSym,
                  uint8_t mcs,
                  uint8_t rank,
                  const std::vector<bool>& rbgMask)
            : m_rnti(rnti),
              m_tbs(tbs),
              m_symStart(symStart),
              m_numSym(numSym),
              m_mcs(mcs),
              m_rank(rank),
              m_rbgMask(rbgMask)
        {
        }

        uint16_t m_rnti{0};          //!< Allocated RNTI
        uint32_t m_tbs{0};           //!< Allocated TBS
        uint8_t m_symStart{0};       //!< Sym start
        uint8_t m_numSym{0};         //!< Allocated symbols
        uint8_t m_mcs{0};            //!< MCS of the transmission
        uint8_t m_rank{1};           //!< rank of the transmission
        std::vector<bool> m_rbgMask; //!< RBG Mask
    };

    /**
     * @brief A vector of UL allocations to calculate CQI and symbols reserved.
     */
    struct NR_EXPORT SlotElem
    {
        /**
         * @brief SlotElem default constructor (deleted)
         */
        SlotElem() = delete;
        /**
         * @brief SlotElem default copy constructor
         */
        SlotElem(const SlotElem& o) = default;

        /**
         * @brief SlotElem constructor with the number of total symbol used for UL
         * @param totUlSym symbols used for UL
         */
        SlotElem(uint8_t totUlSym)
            : m_totUlSym(totUlSym)
        {
        }

        uint8_t m_totUlSym;                     //!< Total symbols used for UL
        std::vector<AllocElem> m_ulAllocations; //!< List of UL allocations
    };

    void BSRReceivedFromUe(const MacCeElement& bsr);

    template <typename T>
    std::vector<T> MergeHARQ(std::vector<T>* existingFeedbacks,
                             const std::vector<T>& inFeedbacks,
                             const std::string& mode) const;

    void ResetExpiredHARQ(uint16_t rnti, NrMacHarqVector* harq);

    /**
     * @brief Move a UE's DL OLLA offset on one first-transmission outcome
     * @param rnti RNTI of the UE
     * @param ack true if the block was received OK, false on NACK or late feedback
     */
    void UpdateDlOlla(uint16_t rnti, bool ack) const;
    /**
     * @brief Move a UE's UL OLLA offset on one first-transmission outcome (UlOlla)
     *
     * Only the outcome of a grant whose MCS came from a measurement of the UE's uplink tells
     * whether that choice was right: the starting MCS of a UE the gNB has not measured (yet, or
     * after the measurement expired) is no choice of the link adaptation. And the offset does
     * not move past the end of the MCS range: a NACK at MCS 0 cannot lower the MCS any more, so
     * a lower offset would only delay the recovery (wind-up), and the same for an ACK at the
     * highest MCS.
     * @param rnti RNTI of the UE
     * @param ack true if the gNB decoded the block
     * @param mcs MCS of the block
     * @param measured true if the grant's MCS came from a measurement of the UE's uplink
     */
    void UpdateUlOlla(uint16_t rnti, bool ack, uint8_t mcs, bool measured) const;
    /**
     * @brief Put the level of a UE's newest uplink SINR report into the smoothed one
     *
     * Only with UlSinrAvgTime above 0. Called right after the report replaced the UE's
     * m_ulSinrMeas; the report keeps its shape over the resource blocks and gets the level of the
     * exponentially weighted mean (in the SINR of one resource block).
     * @param ue the UE
     */
    void SmoothUlSinr(const std::shared_ptr<NrMacSchedulerUeInfo>& ue) const;
    /// HarqInFrequency() and all UEs of the cell in one beam
    bool UseHarqInFrequency() const;
    /**
     * @brief PUSCH repetition: place this slot's transmission of every block under repetition,
     * on the RBGs of its first one if they are free, else on as many others, over the numSym
     * data symbols before start (OFDMA: the region of the retransmissions, which new data
     * shares); they leave m_ulRbgFreeForData. A block that finds no room, or whose UE has a
     * PUSCH in this slot already, waits for the next uplink slot.
     * @param start where the data symbols end
     * @param numSym the data symbols
     * @param allocInfo the slot's allocations
     * @return the UEs that got one (one PUSCH per UE and slot)
     */
    std::set<uint16_t> PlaceUlRepetitions(const PointInFTPlane& start,
                                          uint8_t numSym,
                                          SlotAllocInfo* allocInfo);
    /// PUSCH repetition: queue the further transmissions of the blocks this slot's grants (new
    /// data and retransmissions) asked to repeat
    void QueueUlRepetitions(const SlotAllocInfo& allocInfo);
    /**
     * @brief PDSCH aggregation: place this slot's transmission of every downlink block under
     * repetition, over the numSym data symbols from start, on as many RBGs as keep its resource
     * elements (its own first, as a non-adaptive retransmission); they leave m_dlRbgFreeForData.
     * A block that finds no room, or whose UE has a PDSCH in this slot already, waits.
     * @param start where the data symbols begin
     * @param numSym the data symbols
     * @param allocInfo the slot's allocations
     * @return the UEs that got one (one PDSCH per UE and slot)
     */
    std::set<uint16_t> PlaceDlRepetitions(const PointInFTPlane& start,
                                          uint8_t numSym,
                                          SlotAllocInfo* allocInfo);
    /// PDSCH aggregation: give this slot's new downlink blocks their number of transmissions
    /// (PdschAggregation) and queue the further transmissions of those that repeat
    void QueueDlRepetitions(const SlotAllocInfo& allocInfo);
    /// PDSCH aggregation: the transmissions of a new block of this UE, fixed or from its SINR
    uint8_t DlAggregationFor(const DciInfoElementTdma& dci, bool sps) const;
    /// the UEs with a downlink block under repetition, whose next transmission is still to come
    std::set<uint16_t> DlRepeatingUes() const;

    template <typename T>
    void ProcessHARQFeedbacks(std::vector<T>* harqInfo,
                              const NrMacSchedulerUeInfo::GetHarqVectorFn& GetHarqVectorFn,
                              const std::string& direction) const;

    void ScheduleDl(const NrMacSchedSapProvider::SchedDlTriggerReqParameters& params,
                    const std::vector<DlHarqInfo>& dlHarqInfo);

    void ScheduleUl(const NrMacSchedSapProvider::SchedUlTriggerReqParameters& params,
                    const std::vector<UlHarqInfo>& ulHarqInfo);

    uint8_t AppendCtrlSym(uint8_t symStart,
                          uint8_t numSymToAllocate,
                          DciInfoElementTdma::DciFormat mode,
                          std::deque<VarTtiAllocInfo>* allocations) const;
    uint8_t PrependCtrlSym(uint8_t symStart,
                           uint8_t numSymToAllocate,
                           DciInfoElementTdma::DciFormat mode,
                           std::deque<VarTtiAllocInfo>* allocations) const;

    void ComputeActiveUe(ActiveUeMap* activeDlUe,
                         const NrMacSchedulerUeInfo::GetLCGFn& GetLCGFn,
                         const NrMacSchedulerUeInfo::GetHarqVectorFn& GetHarqVector,
                         const std::string& mode) const;
    void ComputeActiveHarq(ActiveHarqMap* activeDlHarq,
                           const std::vector<DlHarqInfo>& dlHarqFeedback) const;
    void ComputeActiveHarq(ActiveHarqMap* activeUlHarq,
                           const std::vector<UlHarqInfo>& ulHarqFeedback) const;

    uint8_t DoScheduleDlData(PointInFTPlane* spoint,
                             uint32_t symAvail,
                             const ActiveUeMap& activeDl,
                             SlotAllocInfo* slotAlloc) const;
    uint8_t DoScheduleUlData(PointInFTPlane* spoint,
                             uint32_t symAvail,
                             const ActiveUeMap& activeUl,
                             SlotAllocInfo* slotAlloc) const;
    uint8_t DoScheduleUlMsg3(PointInFTPlane* sPoint, uint8_t symAvail, SlotAllocInfo* slotAlloc);
    /**
     * @brief Schedule msg3 (RRC Connection Request) HARQ retransmissions
     *
     * For each TC-RNTI queued in m_msg3RetxList (because its initial msg3 UL
     * PUSCH failed to decode at the gNB and a UL HARQ NACK was received), this
     * allocates fresh UL resources and builds a normal UL HARQ retransmission
     * DCI (m_type=DATA, m_format=UL, m_ndi=0, m_harqProcess=0, m_rv=attempts)
     * addressed to the TC-RNTI. The UE, which already buffered msg3 at HARQ
     * process 0, retransmits it upon receiving this DCI.
     *
     * @param sPoint Starting point in the FT plane (updated)
     * @param symAvail Number of available symbols
     * @param slotAlloc Slot allocation info (updated)
     * @return The number of symbols used by the msg3 retransmission grants
     */
    uint8_t DoScheduleUlMsg3Retx(PointInFTPlane* sPoint,
                                 uint8_t symAvail,
                                 SlotAllocInfo* slotAlloc);
    /**
     * @brief Find the smallest msg3 UL grant that fits an estimated size
     *
     * Computes the lowest number of UL symbols whose TB size (at the RACH UL
     * grant MCS, over the usable RBGs) fits the estimated msg3 size. Shared by
     * the initial msg3 grant and the msg3 HARQ retransmission grant so both
     * always apply the same sizing rule.
     *
     * @param estimatedSizeBits Estimated msg3 size in bits
     * @param symAvail Number of available symbols
     * @param usableRbgs Number of usable RBGs in the UL bitmask
     * @return Pair of allocated symbols and the resulting TB size in bits
     */
    std::pair<uint8_t, uint16_t> FitMsg3Grant(uint16_t estimatedSizeBits,
                                              uint8_t symAvail,
                                              uint16_t usableRbgs) const;
    void DoScheduleUlSr(PointInFTPlane* spoint, const std::list<uint16_t>& rntiList) const;
    uint8_t DoScheduleDl(const std::vector<DlHarqInfo>& dlHarqFeedback,
                         const ActiveHarqMap& activeDlHarq,
                         ActiveUeMap* activeDlUe,
                         const SfnSf& dlSfnSf,
                         const SlotElem& ulAllocations,
                         SlotAllocInfo* allocInfo);
    uint8_t DoScheduleUl(const std::vector<UlHarqInfo>& ulHarqFeedback,
                         const SfnSf& ulSfn,
                         SlotAllocInfo* allocInfo,
                         LteNrTddSlotType type);
    uint8_t DoScheduleSrs(PointInFTPlane* spoint, SlotAllocInfo* allocInfo);
    /**
     * @brief Log the UEs with pending downlink signaling traffic.
     * @param activeDlUe active DL UEs grouped by beam.
     */
    void LogUesWithPendingDlSignalingTraffic(const ActiveUeMap& activeDlUe) const;

    // --- The PDCCH as a resource (NrPdcch), when its attribute Enabled is set ---
    /**
     * @brief Is the PDCCH of this cell modelled?
     * @return true if it is (the object is looked up on the first call)
     */
    bool PdcchOn();
    /**
     * @brief The UEs of all beams in the order the scheduling policy serves them
     * @param activeUe the UEs with new data
     * @param isDl true for downlink
     * @return the UEs: served longest ago first, then sorted by SortByPolicy()
     */
    std::vector<UePtrAndBufferReq> InPolicyOrder(const ActiveUeMap& activeUe, bool isDl) const;
    /**
     * @brief Without the PDCCH model: remember which UEs the slot served (with it, SettlePdcch()
     * does), for the order in which ComputeActiveUe() lists them
     * @param allocInfo the allocations of the slot
     * @param isDl true for the downlink assignments, false for the uplink grants
     */
    void NoteServed(const SlotAllocInfo& allocInfo, bool isDl);
    /**
     * @brief Reserve PDCCH candidates for the downlink assignments of the slot, retransmissions
     * first and then new data in the order of the scheduling policy; a UE that finds no free
     * candidate is taken out of the list and gets nothing in this slot
     * @param activeDlHarq the retransmissions due
     * @param activeDlUe the UEs with new data (in, out)
     * @param maxUes the most UEs to reserve for
     */
    void AdmitDlToPdcch(const ActiveHarqMap& activeDlHarq, ActiveUeMap* activeDlUe, uint32_t maxUes);
    /**
     * @brief The same for the uplink grants of the slot
     * @param ulHarqFeedback the retransmissions due
     * @param activeUlUe the UEs that asked for a grant
     * @param maxUes the most UEs to reserve for
     */
    void AdmitUlToPdcch(const std::vector<UlHarqInfo>& ulHarqFeedback,
                        const ActiveUeMap& activeUlUe,
                        uint32_t maxUes);
    /**
     * @brief Give every DCI of the slot its PDCCH candidate and free the candidates no DCI uses
     * @param allocInfo the allocations of the slot
     * @param isDl true for the downlink assignments, false for the uplink grants
     */
    void SettlePdcch(SlotAllocInfo* allocInfo, bool isDl);
    /**
     * @brief For NrMacSchedulerHarqRr: may a retransmission be scheduled for this UE in this slot?
     * @param rnti the UE
     * @param isDl true for downlink
     * @return true if it has a PDCCH candidate (in uplink: one no other retransmission took)
     */
    bool TakePdcch(uint16_t rnti, bool isDl);
    /**
     * @brief Take the UEs without a PDCCH candidate out of a list of UEs
     * @param activeUe the list (in, out)
     * @param grants the candidates reserved
     */
    static void KeepUesWithPdcch(ActiveUeMap* activeUe,
                                 const std::map<uint16_t, NrPdcch::Grant>& grants);

    // --- Semi-persistent scheduling (downlink) and configured grant Type 2 (uplink) ---
    //
    // SpsScenario 1 or 2, with the PDCCH model on. The bearer with the 5QI SpsFiveQi (voice) of
    // each UE is served by blocks that come every SpsPeriodMs without a DCI, once a DCI has
    // activated the configuration (TS 38.321 5.8.1 and 5.8.2, TS 38.213 10.2):
    //   - a block has the resources, the MCS and so the size the activation gave it; the size is
    //     the smallest that holds SpsTbBytes, one voice packet in the steady state of ROHC
    //   - an occasion with nothing to send is left empty
    //   - a retransmission has a DCI of its own
    //   - a packet larger than the block: if it is a silence descriptor (the gNB sees the
    //     size of the IP packets of the bearer: NrPdcp::GetLastSduKind) it is sent in pieces
    //     over this occasion and the next; if it is a voice frame it is sent with a DCI of its
    //     own, because the next voice frame is 20 ms away and the queue would never empty
    //   - scenario 1: the configuration stays as it is when the speaker falls silent, and most
    //     occasions are then empty
    //   - scenario 2: there are two configurations, for talk (SpsPeriodMs, SpsTbBytes) and for
    //     silence (SpsSilencePeriodMs, SpsSilenceTbBytes); when the kind of packet changes, one
    //     DCI releases the active one and a second activates the other, with a block large
    //     enough for the packet at hand if that is no more than SpsFitExtraBytes above the size
    //   - a UE that missed the activation DCI knows nothing of the blocks; the gNB notices when
    //     their HARQ feedback fails to come and activates the configuration again
    //   - the size of a block (its transport block size) is the one of the fewest resource
    //     block groups that hold SpsTbBytes. An OFDMA scheduler gives the block just these, and
    //     blocks of one beam share symbols; a TDMA scheduler gives it whole symbols, as it does
    //     with every block, and what is beyond the size is padding
    //
    // Uplink (configured grant Type 2, TS 38.321 5.8.2) differs in what the gNB knows:
    //   - it learns of a packet from a scheduling request or a buffer status report, and of its
    //     kind only when it has received it. A UE whose voice the gNB has heard and that sends
    //     a scheduling request gets its grant with a DCI, and a configuration that starts with
    //     the next packet: its blocks come 2 ms after a packet is due, every period. A UE with
    //     nothing to send leaves a block unused (TS 38.321 5.4.3.1.3)
    //   - a request (scheduling request or buffer status report) is answered by the next block
    //     if that is less than SpsPeriodMs away, without a DCI
    //   - data left after a block (the buffer status report in it says so): if the speaker was
    //     silent it is a voice frame and gets a DCI of its own; if he was talking it is taken
    //     for a silence descriptor and waits for the next occasion, once
    //   - scenario 2: after the first silence descriptor the silence configuration takes over
    //     where the next one is due, one period later; from silence to talk it is the
    //     scheduling request, away from the occasions of the silence configuration, that tells:
    //     it gets its DCI and the talk configuration takes over with the next packet
    //   - the UE confirms every activation and release with a MAC CE in its next block, 1 byte
    //     with one configuration and CgMultiEntryCeBytes (5) with two
    //     (DciInfoElementTdma::m_cgConfirmBytes); a block
    //     is that much larger, and by the 5 bytes of the buffer status report of this module

    /// What the gNB keeps of one direction of one UE
    struct SpsState
    {
        bool active{false};       //!< a configuration is active
        uint8_t config{0};        //!< which: 0 = talk, 1 = silence
        uint32_t serial{0};       //!< counts the activations
        uint64_t startSlot{0};    //!< the slot of its first block
        uint64_t alignSince{0};   //!< SpsDlAlignMs: slot + 1 the packet that starts one was first seen
        uint32_t periodSlots{0};  //!< its period
        uint32_t needTb{0};       //!< the block size it was made for, in bytes
        uint8_t numSym{0};        //!< its block: symbols,
        uint16_t numRbg{0};       //!< resource block groups,
        uint8_t mcs{0};           //!< MCS,
        uint8_t rank{1};          //!< rank
        uint8_t reps{1};          //!< uplink: transmissions of each block (repK, TS 38.331)
        uint32_t tbs{0};          //!< and with these the bytes its resources hold
        uint32_t usable{0};       //!< its size: the bytes it carries (see above)
        bool dynamicUntilDrained{false}; //!< a packet too large for the block is being sent
                                         //!< with DCIs of its own
        int8_t lastKind{-1};      //!< NrPdcp::SduKind of the packet that came last
        /// A block whose HARQ feedback is awaited
        struct Sent
        {
            std::shared_ptr<DciInfoElementTdma> dci; //!< its DCI (sent or not)
            uint32_t serial{0};                      //!< the activation it belongs to
            uint64_t slot{0};                        //!< its slot, counted from frame 0
            uint64_t sfn{0};                         //!< the same as SfnSf::GetEncoding()
            uint8_t harqId{0};                       //!< its HARQ process
            bool retx{false};                        //!< it was not received and is being
                                                     //!< retransmitted on that process
            std::vector<uint64_t> repSfns; //!< the slots (SfnSf encoding) of its repetitions
        };
        std::deque<Sent> blocks; //!< the blocks whose HARQ feedback is awaited
        std::shared_ptr<DciInfoElementTdma> releaseDci; //!< the last release sent
        uint8_t releaseConfig{0};  //!< what it released
        uint32_t releaseSerial{0}; //!< the activation whose feedback tells if it arrived
        bool releaseAgain{false};  //!< the UE missed it: send it again
        // uplink only
        uint8_t leftover{0};       //!< blocks in a row after which the UE still had data
        bool checkLeftover{false}; //!< a block has come in: see what its status report says ...
        uint64_t checkSlot{0};     //!< ... from this slot on (the report follows the feedback)
        uint64_t checkBlock{0};    //!< the slot of that block
        uint32_t seenReport{0};    //!< the most a buffer status report said since the last look
        uint32_t leftoverBytes{0}; //!< what the report of the last block said was left
        uint64_t lastUsedSlot{0};  //!< the slot of the last block the UE sent something in
        uint64_t dynamicSlot{0};   //!< the slot of the grant (a DCI) given for what a block left
        uint64_t heardSlot{0};     //!< when the gNB first had a packet of the bearer
        uint64_t holeSlot{0};      //!< an occasion whose block found no room in its slot
        bool pending{false};       //!< a configuration is to start, or to take over, ...
        uint8_t pendingConfig{0};  //!< ... this one ...
        uint64_t pendingSlot{0};   //!< ... with a block in this slot (or a talk period later)
        // The speaker starts or stops talking (NrPdcch::Change): what is counted of it
        bool voiceKnown{false};     //!< the next three are set: NrPdcp::VoiceActivity of the
        uint64_t voiceSdus{0};      //!< bearer when it was last looked at: its packets, the
        uint64_t voiceToSilence{0}; //!< times the speaker fell silent
        uint64_t voiceToVoice{0};   //!< and the times he started to talk
        uint8_t change{0};          //!< downlink: the first packet after a change is still to be
                                    //!< sent: 1 a silence descriptor, 2 a voice frame (bits)
        uint8_t pieces{0};          //!< downlink: it is being sent in pieces (the same bits)
        int8_t switchFrom{-1};      //!< a release was sent without the activation of the other
                                    //!< configuration, which is still to come: of this one
        // uplink: when a configured grant is activated again (CgHysteresis)
        Time lastActivation{Time::Min()}; //!< its last activation, Time::Min() = none yet
        Time worseSince{Time::Min()};     //!< the link has been short of its K since then
        Time betterSince{Time::Min()};    //!< it could have done with fewer since then
        uint8_t failRun{0};               //!< first transmissions of its blocks failed in a row
        std::deque<std::pair<Time, double>> sinrDb; //!< the UE's SINR (dB) once a talk period
        uint64_t sinrSlot{0};             //!< the slot of the last of them
        double cgMargin{0.0};             //!< the margin (dB) of the next activation
        uint64_t voiceToSilencePdus{0}; //!< uplink: the RLC PDUs the first silence descriptors
        uint64_t voiceToVoicePdus{0};   //!< and the first voice frames came in
        bool aheadSent{false}; //!< CgBlockPacketOnly: the next activation is confirmed ahead
        uint8_t startFails{0};      //!< uplink: slots in a row an activation's block was not placed
        uint64_t startRetrySlot{0}; //!< ... after 2: no new try before this slot (DCIs meanwhile)
        bool switchNow{false}; //!< uplink, CgSilenceSwitch: a silence just seen on the talk configuration
        bool gaveDci{false};        //!< uplink: since the last packet of the bearer came in, the
                                    //!< gNB gave a grant with a DCI: for the rest a block left,
                                    //!< or for a scheduling request
    };

    /// A block planned for the slot being scheduled
    struct SpsBlock
    {
        uint16_t rnti{0};
        uint8_t lcgId{0};
        uint8_t lcId{0};
        bool activation{false};    //!< it comes with the DCI that activates `config`
        bool reactivation{false};  //!< ... again, with another MCS; without PDCCH: as it was
        bool worse{false};         //!< ... because the link is worse: as it was, it would fail
        bool segment{false};       //!< it carries a piece of a packet larger than itself
        uint8_t config{0};
        uint32_t needTb{0};        //!< activation: the size the block has to have
        NrPdcch::Grant grant;      //!< activation: its PDCCH candidate
        uint8_t numSym{0};
        uint16_t numRbg{0};
        uint8_t mcs{0};
        uint8_t rank{1};
        uint8_t reps{1}; //!< uplink: transmissions of the block (repK)
        uint32_t tbs{0};
        uint32_t usable{0};
    };

    /// A release planned for the slot being scheduled
    struct SpsRelease
    {
        uint16_t rnti{0};
        uint8_t config{0};
        bool again{false};    //!< a repetition: the UE missed the first
        bool alone{false};    //!< no activation comes with it: the configuration just ends
                              //!< (uplink; a downlink one is left unused and activated anew)
        NrPdcch::Grant grant; //!< its PDCCH candidate
    };

    /**
     * @brief Decide what each voice bearer gets in this downlink slot: a block of its
     * configuration, an activation, a release, a DCI of its own, or nothing; and take the
     * bearers that wait for their occasion out of the sight of the scheduler
     * @param slot the slot, counted from frame 0
     */
    void SpsPlanDl(uint64_t slot);
    /**
     * @brief The same for the uplink slot: the block of each active configured grant, an
     * activation for a UE that asks for a grant, a release; and take out of sight the requests
     * that a block of this or a coming occasion answers
     * @param slot the slot, counted from frame 0
     */
    void SpsPlanUl(uint64_t slot);
    /**
     * @brief A block of a configuration has come back (HARQ feedback) or was left unused: what
     * that tells about the release sent before it
     * @param st the state of the UE
     * @param serial the activation the block belonged to
     */
    void SpsBlockDone(SpsState& st, uint32_t serial) const;
    /**
     * @brief What the PDCP has seen of the speaker of a voice bearer since the last look: counts
     * the times he fell silent and the times he started to talk (NrPdcch::CHANGE_SEEN)
     * @param st the state of the UE
     * @param rnti the UE
     * @param lcId the logical channel of the bearer
     * @param isDl true for downlink
     * @param packets set to true if a packet has come since the last look
     * @param changes set to the times he fell silent [0] and started to talk [1], if not null
     * @param pdus uplink: set to the RLC PDUs the first packets after these came in, if not null
     * @return 1 if he fell silent, 2 if he started to talk, 3 if both (bits), else 0
     */
    uint8_t SpsVoiceChanges(SpsState& st,
                            uint16_t rnti,
                            uint8_t lcId,
                            bool isDl,
                            bool* packets,
                            std::array<uint64_t, 2>* changes = nullptr,
                            std::array<uint64_t, 2>* pdus = nullptr) const;
    /**
     * @brief Count something of a change between voice and silence
     * @param isDl true for downlink
     * @param changes which changes, as SpsVoiceChanges() returns them
     * @param what what to count of each
     * @param n how many
     */
    void SpsNoteChange(bool isDl, uint8_t changes, NrPdcch::Change what, uint64_t n = 1) const;
    /**
     * @brief Size of the block of an uplink configuration
     * @param config 0 = talk, 1 = silence
     * @return the bytes: the packet, the buffer status report and the confirmation MAC CE
     */
    uint32_t SpsUlTbBytes(uint8_t config) const;
    /// SpsUlTbBytes() with room for the activation's confirmation (CgBlockPacketOnly)
    uint32_t SpsUlTbBytesWithCe(uint8_t config) const;
    /// The grant for a scheduling request of a UE (SrGrantLearned)
    uint32_t LearnedSrGrant(uint16_t rnti) const;
    /// Downlink SPS reactivation with hysteresis (DlSpsHysteresis): -1 worse, 1 better, 0 keep
    int DlSpsOutOfDate(SpsState& st, const std::shared_ptr<NrMacSchedulerUeInfo>& ue) const;
    /**
     * @brief Reserve the PDCCH candidates of the releases and activations planned
     * @param isDl true for downlink
     */
    void SpsAdmit(bool isDl);
    /**
     * @brief Put the blocks and releases planned into the slot
     * @param slot the slot, counted from frame 0
     * @param sfn the same slot as SfnSf::GetEncoding()
     * @param isDl true for downlink
     * @param firstSym the first symbol they may take (uplink: the one after the last)
     * @param symAvail how many symbols they may take
     * @param allocInfo the allocations of the slot
     * @return the symbols they took
     */
    uint8_t SpsAllocate(uint64_t slot,
                        uint64_t sfn,
                        bool isDl,
                        uint8_t firstSym,
                        uint8_t symAvail,
                        SlotAllocInfo* allocInfo);
    /**
     * @brief Bring the bearers SpsPlanDl() took out of sight back
     */
    void SpsUnhide();
    /**
     * @brief The smallest block that holds a number of bytes
     * @param isDl true for downlink
     * @param mcs MCS
     * @param rank rank
     * @param needTb bytes to hold
     * @param maxSym the most symbols it may have
     * @param block the block: its symbols, RBGs and size are set
     * @return false if there is none
     */
    bool SpsGeometry(bool isDl,
                     uint8_t mcs,
                     uint8_t rank,
                     uint32_t needTb,
                     uint8_t maxSym,
                     SpsBlock* block) const;
    /**
     * @brief Is there room for the blocks of a configuration that starts in a slot?
     *
     * The blocks of a configuration come back every period, so they need room not in this
     * slot only but in every slot they share with the blocks of the other UEs: their symbols
     * (blocks of whole symbols) or their resource block groups must fit together.
     * @param isDl true for downlink
     * @param rnti the UE (the configuration it has now does not count: it is replaced)
     * @param slot the slot of the first block
     * @param config 0 = talk, 1 = silence
     * @param needTb the bytes a block has to hold
     * @param mcs the MCS of the UE
     * @param rank its rank
     * @param reps uplink: the transmissions of each block, in as many slots in a row
     * @return true if there is; the room is then taken for this slot (m_spsPlanned)
     */
    bool SpsHasRoom(bool isDl,
                    uint16_t rnti,
                    uint64_t slot,
                    uint8_t config,
                    uint32_t needTb,
                    uint8_t mcs,
                    uint8_t rank,
                    uint8_t reps = 1) const;
    /**
     * @brief Symbols of a slot that blocks of configurations may take
     * @param isDl true for downlink
     * @return the symbols: those of the slot without the control symbols (and the SRS)
     */
    uint8_t SpsSymbolBudget(bool isDl) const;
    /**
     * @brief Has the link of a UE moved away from the MCS its configuration was activated with?
     *
     * By more than SpsMcsTolerance: down, so that its blocks would fail too often; or up, if the
     * block would then take fewer resources.
     * @param isDl true for downlink
     * @param st the configuration
     * @param mcs the MCS of the UE now
     * @param rank its rank now
     * @return 0 if not; -1: the link is worse; 1: it is better and a block would take less
     */
    int SpsOutOfDate(bool isDl, const SpsState& st, uint8_t mcs, uint8_t rank) const;
    /**
     * @brief The uplink MCS of a UE for a block of a configured grant
     *
     * A UE has one transmit power for all its resource blocks, so its SINR per resource block,
     * and with it the MCS the gNB works out from a transmission, depend on how many resource
     * blocks that transmission had. A block of a configured grant takes the whole band, or
     * UlGrantRbs (SpsGeometry()): this is the MCS for that width, worked out from the last
     * transmission of the UE whatever its width.
     * @param ue the UE
     * @return the MCS
     */
    uint8_t SpsUlMcs(const std::shared_ptr<NrMacSchedulerUeInfo>& ue) const;
    /**
     * @brief The MCS and the repetitions of a block of a configured grant
     *
     * With PUSCH repetition on (PuschRepetitions other than 1) and narrow blocks (UlGrantRbs),
     * a configured grant repeats each block too, in repK slots in a row (TS 38.214 6.1.2.3,
     * TS 38.331 ConfiguredGrantConfig: 1, 2, 4 or 8): the fewest that bring repK times the
     * UE's SINR over the block's resource blocks to PuschRepetitionSinrDb plus `marginDb`
     * (PuschRepetitions = 0; else the fewest that are at least PuschRepetitions), and the MCS
     * the error model decodes after them; more of them, up to 8, if a block of `needTb` bytes
     * does not fit into its resource blocks at that MCS. Otherwise one transmission at
     * SpsUlMcs().
     * @param ue the UE
     * @param needTb the bytes a block has to hold, 0 = not looked at
     * @param marginDb more SINR to ask for (to go down to fewer repetitions only with a margin)
     * @param reps if not 0: these repetitions, and the MCS for them
     * @return the MCS and the repetitions
     */
    /**
     * @brief Uplink configured grant (CgHysteresis): record the UE's SINR once a talk period and
     * return the margin of the activation, its mean less the CgSinrPercentile of the window
     * @param st the UE's configured-grant state
     * @param ue the UE
     * @param slot the slot being planned
     * @return the margin in dB (0 if there is none to take)
     */
    double CgMarginDb(SpsState& st,
                      const std::shared_ptr<NrMacSchedulerUeInfo>& ue,
                      uint64_t slot) const;
    /**
     * @brief The uplink SINR of the UE on the resource blocks of a configured-grant block
     * @param ue the UE
     * @return the mean linear SINR, 0 without a measurement
     */
    double SpsUlSinr(const std::shared_ptr<NrMacSchedulerUeInfo>& ue) const;
    /**
     * @brief Uplink configured grant (CgHysteresis): whether to activate it anew at this
     * occasion, and the timers that decide it
     * @param st the UE's configured-grant state
     * @param ue the UE
     * @return -1 for more repetitions or a lower MCS, 1 for fewer or a higher MCS, 0 as it is
     */
    int CgOutOfDate(SpsState& st, const std::shared_ptr<NrMacSchedulerUeInfo>& ue) const;
    std::pair<uint8_t, uint8_t> SpsUlLink(const std::shared_ptr<NrMacSchedulerUeInfo>& ue,
                                          uint32_t needTb,
                                          double marginDb = 0.0,
                                          uint8_t reps = 0,
                                          double mcsMarginDb = 0.0) const;
    /**
     * @brief HARQ feedback for a block: if the UE did not know of it, the configuration is lost
     * @param rnti the UE
     * @param isDl true for downlink
     * @param process the HARQ process
     * @param ok the block was received (ACK)
     * @param last no retransmission follows: it was received, or this was the last one
     */
    void SpsFeedback(uint16_t rnti,
                     bool isDl,
                     const HarqProcess& process,
                     bool ok,
                     bool last) const;
    /**
     * @brief Period of a configuration
     * @param config 0 = talk, 1 = silence
     * @return the period in slots
     */
    uint32_t SpsPeriodSlots(uint8_t config) const;

    static const unsigned m_macHdrSize = 0; //!< Mac Header size
    static const unsigned m_rlcHdrSize = 3; //!< RLC Header size
    // The MAC sub header size defined in nr-mac-scheduler-lcg.h

  protected:
    /**
     * @brief Whether a new downlink block's MCS is lowered to fill its resource blocks
     * @return the attribute DlMcsFill
     */
    bool GetDlMcsFill() const
    {
        return m_dlMcsFill;
    }

    /**
     * @brief Get the bwp id of this MAC
     * @return the bwp id
     */
    uint16_t GetBwpId() const;

    /**
     * @brief Get the cell id of this MAC
     * @return the cell id
     */
    uint16_t GetCellId() const;

    /**
     * @return the bandwidth in RBG
     */
    uint16_t GetBandwidthInRbg() const;

    /**
     * @brief Get the FH Control Method.
     * @return the FH Control Method (uint8_t)
     */
    uint8_t GetFhControlMethod() const;

    /**
     * @brief Returns a boolean indicating whether the current allocation can
     *        fit in the available FH bandwidth (when FH Control is enabled).
     */
    bool DoesFhAllocationFit(uint16_t bwpId,
                             uint32_t mcs,
                             uint32_t nRegs,
                             uint8_t dlRank,
                             uint8_t numSym) const;

    // FFR SAPs
    NrFhSchedSapUser* m_nrFhSchedSapUser{nullptr};         //!< FH Control SAP user
    NrFhSchedSapProvider* m_nrFhSchedSapProvider{nullptr}; //!< FH Control SAP provider

    /**
     * @brief Returns a boolean vector indicating whether a resource is available to be scheduled
     * (true) or not (false) in the downlink.
     */
    std::vector<bool> GetDlBitmask() const;
    /**
     * @brief Returns a boolean vector indicating whether a resource is available to be scheduled
     * (true) or not (false) in the uplink.
     */
    std::vector<bool> GetUlBitmask() const;
    /**
     * @brief The downlink RBGs new data may take in the slot being scheduled: GetDlBitmask()
     * without those the HARQ retransmissions of the slot took (see HarqInFrequency())
     */
    std::vector<bool> GetDlDataBitmask() const;
    /**
     * @brief The uplink RBGs new data may take in the slot being scheduled: GetUlBitmask()
     * without those the HARQ retransmissions of the slot took (see HarqInFrequency())
     */
    std::vector<bool> GetUlDataBitmask() const;

    /// The grant for a scheduling request (attribute SrGrantBytes)
    uint32_t GetSrGrantBytes() const
    {
        return m_srGrantBytes;
    }

    /// The most RBs of an uplink grant, 0 = no limit (attribute UlGrantRbs; OFDMA only: a
    /// scheduler whose blocks take the full band, TDMA, gives whole symbols)
    uint32_t GetUlGrantRbs() const
    {
        return SpsUsesFullBand() ? 0 : m_ulGrantRbs;
    }

    /// The resource blocks of a wide uplink grant, 0 = none (attribute UlWideGrantRbs; it needs
    /// UlGrantRbs, below it, and UlMcsPerRbs)
    uint32_t GetUlWideRbs() const
    {
        return GetUlGrantRbs() > 0 && m_ulMcsPerRbs && m_ulWideRbs > GetUlGrantRbs() ? m_ulWideRbs
                                                                                     : 0;
    }

    /// The most uplink load of a slot at which a wide grant is given, % (UlWideLoadPct)
    double GetUlWideLoadPct() const
    {
        return m_ulWideLoadPct;
    }

    /**
     * @brief Whether HARQ retransmissions are placed in frequency, over the data symbols of the
     * slot that new data shares (OFDMA), instead of in symbols of their own (TDMA)
     *
     * Used only while every UE of the cell is in one beam: with several, each beam has symbols
     * of its own.
     */
    virtual bool HarqInFrequency() const
    {
        return false;
    }

    // Generic function serves as trampoline to Tdma and Ofdma, plus gives access to ueInfo map
    std::vector<DciInfoElementTdma> ReshapeAllocation(const std::vector<DciInfoElementTdma>& dcis,
                                                      uint8_t& startingSymbol,
                                                      uint8_t& numSymbols,
                                                      std::vector<bool>& bitmask,
                                                      const bool isDl);

    // Implementation from Tdma and Ofdma schedulers
    virtual std::vector<DciInfoElementTdma> DoReshapeAllocation(
        const std::vector<DciInfoElementTdma>& dcis,
        uint8_t& startingSymbol,
        uint8_t& numSymbols,
        std::vector<bool>& bitmask,
        const bool isDl,
        const std::unordered_map<uint16_t, std::shared_ptr<NrMacSchedulerUeInfo>>& ueMap) = 0;

  private:
    void CallNrFhControlForMapUpdate(
        const std::deque<VarTtiAllocInfo>& allocation,
        const std::unordered_map<uint16_t, std::shared_ptr<NrMacSchedulerUeInfo>>& ueMap);

    std::unordered_map<uint16_t, std::shared_ptr<NrMacSchedulerUeInfo>>
        m_ueMap; //!< The map of between RNTI and their data

    /**
     * Map of previous allocated UE per RBG
     * (used to retrieve info from UL-CQI)
     */
    std::map<uint64_t, SlotElem> m_ulAllocationMap;

    bool m_fixedMcsDl{false};  //!< Fixed MCS for *all* UE in DL
    bool m_fixedMcsUl{false};  //!< Fixed MCS for *all* UE in UL
    uint8_t m_startMcsDl{0};   //!< Starting (or fixed) value for DL MCS
    uint8_t m_startMcsUl{0};   //!< Starting (or fixed) value for UL MCS
    int8_t m_maxDlMcs{0};      //!< Maximum index for DL MCS
    Time m_cqiTimersThreshold; //!< The time while a CQI is valid

    uint8_t m_rachUlGrantMcs{0}; //!< The MCS that will be used for UL RACH grant
    uint8_t m_ulRachBwpIndex{
        0}; //!< The BWP index for UL RACH, i.e., RRC connection request message

    NrMacSchedulerCQIManagement m_cqiManagement; //!< CQI Management

    std::vector<DlHarqInfo>
        m_dlHarqToRetransmit; //!< List of DL HARQ that could not have been retransmitted
    std::vector<UlHarqInfo>
        m_ulHarqToRetransmit; //!< List of UL HARQ that could not have been retransmitted

    std::list<uint16_t> m_srList; //!< List of RNTI of UEs that asked for a SR

    std::vector<struct nr::RachListElement_s> m_rachList; //!< rach list

    /**
     * @brief Bookkeeping for an in-flight msg3 (RRC Connection Request) awaiting
     *        HARQ acknowledgement / retransmission.
     */
    struct Msg3RetxInfo
    {
        uint8_t attempts{0};           //!< Number of retransmissions issued so far
        uint16_t estimatedSizeBits{0}; //!< Estimated msg3 size in bits, from the RACH request
    };

    uint8_t m_msg3MaxRetx{0}; //!< Max msg3 HARQ retransmissions (0 disables the feature)

    /**
     * @brief Pending msg3 transmissions keyed by TC-RNTI. Populated when the
     *        initial msg3 grant is issued, refreshed/erased on HARQ feedback,
     *        and cleared once the TC-RNTI becomes a registered UE.
     */
    std::map<uint16_t, Msg3RetxInfo> m_msg3Pending;

    /**
     * @brief TC-RNTIs whose msg3 must be retransmitted in the next UL slot
     *        (a UL HARQ NACK was received for a pending msg3).
     */
    std::vector<uint16_t> m_msg3RetxList;

    uint16_t m_bandwidth{0};         //!< Bandwidth in number of RBG
    uint8_t m_dlCtrlSymbols{0};      //!< DL ctrl symbols (attribute)
    uint8_t m_dlCtrlSymsSlot{0};     //!< DL ctrl symbols of the slot being scheduled: the
                                     //!< attribute, or what NrPdcch needs (Symbols = 0)
    Ptr<NrPdcch> m_pdcch;            //!< the PDCCH of the cell, see PdcchOn()
    std::map<uint16_t, NrPdcch::Grant> m_dlPdcchGrants; //!< PDCCH candidates of the DL slot
    std::map<uint16_t, NrPdcch::Grant> m_ulPdcchGrants; //!< PDCCH candidates of the UL slot
    std::set<uint16_t> m_ulPdcchTaken; //!< of these, the ones a retransmission has taken
    std::map<uint16_t, uint64_t> m_dlPdcchLast; //!< when each UE last had DL data (a count);
                                                //!< kept with and without the PDCCH model
    std::map<uint16_t, uint64_t> m_ulPdcchLast; //!< when each UE last had an UL grant (a count)
    uint64_t m_pdcchRound{0};          //!< the count: scheduling calls so far

    uint8_t m_spsScenario{0};          //!< SpsScenario attribute: 0 = off, 1, 2
    bool m_spsDlOn{true};              //!< SpsDl attribute
    bool m_spsUlOn{true};              //!< SpsUl attribute
    uint8_t m_spsFiveQi{1};            //!< SpsFiveQi attribute
    uint32_t m_spsPeriodMs{20};        //!< SpsPeriodMs attribute
    uint32_t m_spsSilencePeriodMs{160}; //!< SpsSilencePeriodMs attribute
    uint32_t m_spsTbBytes{0};          //!< SpsTbBytes attribute
    uint32_t m_spsSilenceTbBytes{0};   //!< SpsSilenceTbBytes attribute
    uint32_t m_spsFitExtraBytes{0};    //!< SpsFitExtraBytes attribute
    uint8_t m_spsMcsTolerance{3};      //!< SpsMcsTolerance attribute
    uint32_t m_cgMultiEntryCeBytes{6}; //!< CgMultiEntryCeBytes attribute
    bool m_cgBlockPacketOnly{false};   //!< CgBlockPacketOnly attribute
    bool m_cgStartAfterIr{false};      //!< CgStartAfterIr attribute
    bool m_cgStartRetry{false};        //!< CgStartRetry attribute
    double m_cgMaxRepShare{1.0};       //!< CgMaxRepShare attribute
    bool m_spsSrsSafe{false};          //!< SpsSrsSafe attribute
    uint32_t m_spsDlMaxWaitMs{0};      //!< SpsDlMaxWaitMs attribute
    uint32_t m_spsDlAlignMs{0};        //!< SpsDlAlignMs attribute
    bool m_srGrantLearned{false};      //!< SrGrantLearned attribute
    uint32_t m_srGrantL2Bytes{3};      //!< SrGrantL2Bytes attribute
    uint32_t m_spsSidGapMs{0};         //!< SpsSidGapMs attribute
    bool m_spsDlSilenceAtSecondSid{false}; //!< SpsDlSilenceAtSecondSid attribute
    bool m_dlSpsHysteresis{false};     //!< DlSpsHysteresis attribute
    Time m_dlSpsRaiseTime;             //!< DlSpsRaiseTime attribute
    Time m_dlSpsLowerTime;             //!< DlSpsLowerTime attribute
    Time m_dlSpsMinInterval;           //!< DlSpsMinInterval attribute
    uint32_t m_dlSpsUrgentSteps{6};    //!< DlSpsUrgentSteps attribute
    uint32_t m_dlSpsFailures{2};       //!< DlSpsFailures attribute
    bool m_cgSilenceSwitchNow{false};  //!< CgSilenceSwitch attribute: true = immediate
    uint8_t m_dlSpsSymsSlot{0};        //!< symbols the SPS blocks take in the DL slot at hand
    uint64_t m_ulSlotAtHand{0};        //!< the uplink slot being scheduled, counted from frame 0
    mutable std::map<uint16_t, SpsState> m_spsDl; //!< downlink SPS of each UE
    mutable std::map<uint16_t, SpsState> m_spsUl; //!< uplink configured grant of each UE
    std::vector<SpsBlock> m_spsBlocksSlot;     //!< the blocks of the slot at hand
    std::vector<SpsRelease> m_spsReleasesSlot; //!< the releases of the slot at hand
    /// bearers out of sight of the scheduler for the slot at hand, and their real queue size
    std::vector<std::pair<NrMacSchedulerLC*, uint32_t>> m_spsHidden;
    /// symbols and RBGs of the configurations that start in the slot at hand (SpsHasRoom)
    mutable std::vector<std::pair<uint32_t, uint32_t>> m_spsPlanned;
    /// Of the last uplink transmission of each UE: the MCS it gave, and the MCS the same power
    /// gives over the whole band (SpsUlMcs())
    std::map<uint16_t, std::pair<uint8_t, uint8_t>> m_spsUlWholeBandMcs;
    /// scheduling requests out of sight for the slot at hand: the UE and what it is owed
    std::vector<std::pair<std::shared_ptr<NrMacSchedulerUeInfo>, uint32_t>> m_spsHiddenSr;
    uint8_t m_ulCtrlSymbols{0};      //!< UL ctrl symbols (attribute)
    uint8_t m_srsCtrlSymbols{0};     //!< SRS symbols (attribute)
    bool m_enableSrsInUlSlots{true}; //!< SRS allowed in UL slots (attribute)
    bool m_enableSrsInFSlots{true};  //!< SRS allowed in F slots (attribute)

    std::vector<bool> m_dlNotchedRbgsMask; //!< The mask of notched (blank) RBGs for the DL
    std::vector<bool> m_ulNotchedRbgsMask; //!< The mask of notched (blank) RBGs for the UL

    Ptr<NrMacSchedulerHarqRr> m_schedHarq; //!< Pointer to the real HARQ scheduler

    Ptr<NrMacSchedulerSrsDefault> m_schedulerSrs; //!< Pointer to the SRS algorithm
    Ptr<NrMacSchedulerLcAlgorithm>
        m_schedLc;        //!< Pointer to an instance of the LC scheduling algorithm
    TypeId m_schedLcType; //!< Type of the LC scheduling algorithm

    uint32_t m_srsSlotCounter{0}; //!< Counter for UL slots

    bool m_enableHarqReTx{true}; //!< Flag to enable or disable HARQ ReTx (attribute)

    bool m_dlHarqFeedbackDisabled{false}; //!< Rel-17 downlinkHARQ-FeedbackDisabled: release the
                                          //!< DL HARQ process at transmission instead of on
                                          //!< feedback, so the pipeline is not RTT-throttled

    double m_ollaStepUp{0.0};     //!< DL OLLA step applied on ACK, in MCS units. 0 disables OLLA
    double m_ollaTargetBler{0.1}; //!< DL OLLA target initial BLER; sets the down/up step ratio
    double m_ollaMaxOffset{4.0};  //!< DL OLLA offset clamp, in MCS units
    bool m_dlMcsFill{false};      //!< DlMcsFill: lower a new DL block's MCS to fill its RBs
    bool m_cgHysteresis{true};    //!< CgHysteresis: the reactivation policy below; false = as before
    double m_cgRaiseDb{1.0};      //!< CgRaiseDb: short of the SINR target by this much ...
    Time m_cgRaiseTime{MilliSeconds(100)}; //!< CgRaiseTime: ... this long: more repetitions
    double m_cgUrgentDb{3.0};     //!< CgUrgentDb: short by this much: at once, any interval
    double m_cgLowerDb{2.0};      //!< CgLowerDb: fewer repetitions with this much to spare ...
    Time m_cgLowerTime{MilliSeconds(400)}; //!< CgLowerTime: ... this long
    double m_cgSinrPercentile{0.0}; //!< CgSinrPercentile: K and MCS for this percentile, 0 = now
    Time m_cgSinrWindow{MilliSeconds(500)}; //!< CgSinrWindow: of the SINR over this window
    Time m_cgMinInterval{MilliSeconds(200)}; //!< CgMinInterval: between activations
    uint32_t m_cgFailures{2};     //!< CgFailures: failed first transmissions in a row: at once

    /// DL OLLA offset after each update: rnti, offset in MCS units, ack, clamped
    TracedCallback<uint16_t, double, bool, bool> m_dlOllaUpdate;
    bool m_ulOlla{false};        //!< UL OLLA on (attribute UlOlla), same steps as the DL one
    Time m_ulSinrAvgTime{Seconds(0)}; //!< UlSinrAvgTime: time constant of the UL SINR smoothing
    /// UL HARQ processes, as (RNTI, process), whose grant took its MCS from a measurement of the
    /// UE's uplink: only their first transmissions move the UL OLLA (UpdateUlOlla())
    mutable std::set<std::pair<uint16_t, uint8_t>> m_ulMeasuredHarq;

    /// PUSCH repetition: a block whose grant asked for repetitions, with the transmissions of it
    /// that are still to be placed, one per uplink slot (PlaceUlRepetitions())
    struct UlRepetition
    {
        std::shared_ptr<DciInfoElementTdma> dci; //!< the grant of the block (or its retransmission)
        uint8_t next{1};                         //!< the next transmission of it to place
    };

    std::list<UlRepetition> m_ulRepetitions; //!< the blocks under repetition, oldest first
    std::list<UlRepetition> m_dlRepetitions; //!< PDSCH aggregation: the same, downlink
    uint32_t m_pdschAggregation{1};          //!< attribute PdschAggregation
    double m_pdschAggSinrDb{2.0};            //!< attribute PdschAggregationSinrDb
    /// PDSCH aggregation of each UE's SPS configuration, set at its activation (RRC, SPS-Config)
    std::unordered_map<uint16_t, uint8_t> m_dlSpsAggregation;
    uint32_t m_puschRepetitions{1};          //!< attribute PuschRepetitions
    double m_puschRepSinrDb{2.0};            //!< attribute PuschRepetitionSinrDb
    uint32_t m_puschMinBlockBytes{0};        //!< attribute PuschMinBlockBytes
    bool m_ulMcsPerRbs{false};   //!< UL MCS for the RBs of each grant (attribute UlMcsPerRbs)
    uint32_t m_srGrantBytes{17}; //!< Grant after a scheduling request (attribute SrGrantBytes)
    uint32_t m_ulGrantRbs{0};    //!< Most RBs of an uplink grant, 0 = all (attribute UlGrantRbs)
    uint32_t m_ulWideRbs{0};     //!< RBs of a wide uplink grant, 0 = none (UlWideGrantRbs)
    double m_ulWideSinrDb{5.0};  //!< SINR on one RB for a wide grant (attribute UlWideSinrDb)
    double m_ulWideLoadPct{50.0}; //!< most slot load for a wide grant (attribute UlWideLoadPct)
    std::vector<bool> m_dlRbgFreeForData; //!< Slot being scheduled: DL RBGs left for new data
    std::vector<bool> m_ulRbgFreeForData; //!< Slot being scheduled: UL RBGs left for new data

    uint32_t m_harqProcessTimeout{0}; //!< slots a process is held without feedback; 0 = legacy
                                      //!< (NumHarqProcess), see the HarqProcessTimeout attribute

    TracedCallback<uint16_t, uint16_t, const std::shared_ptr<NrMacSchedulerUeInfo>&>
        m_csiFeedbackReceived; //!< Traced callback to access CSI feedback
};

} // namespace ns3
