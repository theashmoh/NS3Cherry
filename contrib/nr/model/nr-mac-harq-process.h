// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#pragma once

#include "nr-phy-mac-common.h"

#include "ns3/nr-export.h"

#include <memory>
#include <vector>

namespace ns3
{
/**
 * @ingroup scheduler
 * @brief The HarqProcess struct
 *
 * The struct represent a single HARQ process, identified by an ID
 * (unfortunately the ID is not stored in the struct, but somewhere else,
 * as it will be explained later).
 * The process has a status (HarqProcess::Status) and could be active or
 * inactive. Inside the process is stored a shared pointer to a DciInfoElementTdma,
 * which contains all the information for the retransmission of the data,
 * as well as the RLC PDU.
 *
 * The HarqProcess will be stored inside the class NrMacHarqVector, which
 * is a unordered map that maps the HARQ ID with the HARQ content (this struct).
 */
struct NR_EXPORT HarqProcess
{
    /**
     * @brief Status of the process
     *
     * Other than the obvious meaning of the values, it is worth to state that
     * the RECEIVED_FEEDBACK status is equivalent to "the process has received
     * a NACK feedback". An ACKed feedback will be erased immediately (after all,
     * it has been ACKed...). That is probably worth a name change.
     */
    enum Status : uint8_t
    {
        INACTIVE = 0,         //!< Inactive process
        WAITING_FEEDBACK = 1, //!< Data transmitted, waiting the feedback
        RECEIVED_FEEDBACK = 2 //!< Received feedback (NACK)
    };

    /**
     * @brief Default constructor
     */
    HarqProcess() = default;

    /**
     * @brief HarqProcess copy constructor
     * @param other other instance
     */
    HarqProcess(const HarqProcess& other)
        : m_active(other.m_active),
          m_status(other.m_status),
          m_timer(other.m_timer),
          m_dciElement(other.m_dciElement),
          m_rlcPduInfo(other.m_rlcPduInfo),
          m_txSlot(other.m_txSlot),
          m_ollaCounted(other.m_ollaCounted)
    {
    }

    /**
     * @brief HarqProcess value-by-value constructor
     * @param active Is the process active?
     * @param status Status of the process
     * @param timer Timer of the process (in slot)
     * @param dci DCI of the process
     */
    HarqProcess(bool active,
                Status status,
                uint32_t timer,
                const std::shared_ptr<DciInfoElementTdma>& dci)
        : m_active(active),
          m_status(status),
          m_timer(timer),
          m_dciElement(dci)
    {
    }

    /**
     * @brief Reset the Process content.
     */
    void Erase()
    {
        m_active = false;
        m_status = INACTIVE;
        m_timer = 0;
        m_dciElement.reset();
        m_rlcPduInfo.clear();
        m_txSlot = 0;
        m_ollaCounted = false;
    }

    bool m_active{false};      //!< False indicate that the process is not active
    Status m_status{INACTIVE}; //!< Status of the process
    uint32_t m_timer{0};       //!< Age in scheduling indications, including the NTN waiting window
    std::shared_ptr<DciInfoElementTdma> m_dciElement{}; //!< DCI element
    std::vector<RlcPduInfo> m_rlcPduInfo{};             //!< vector of RLC PDU
    /**
     * The slot (SfnSf::Normalize()) of the first transmission of the DCI the process holds now
     * (a retransmission: its own slot); 0 when not known. The feedback of a block comes after
     * its last repetition and a round trip: a process is reclaimed from there, not from when it
     * was scheduled (a block of a configured grant is scheduled a period ahead), and a feedback
     * for it that comes too soon, or for another redundancy version, is for an earlier
     * transmission on the same process id and is stale.
     */
    uint64_t m_txSlot{0};
    /**
     * True once the outcome of the first transmission has been given to the outer-loop link
     * adaptation. A NACK whose retransmission finds no room comes back through the feedback
     * list in every slot until it is placed; it must count once (review of 1 Nov 2026).
     */
    bool m_ollaCounted{false};
};

/**
 * @brief operator << for HARQ processes
 * @param os output stream
 * @param item process to print
 * @return a copy of the output stream
 */
static inline std::ostream&
operator<<(std::ostream& os, const HarqProcess& item)
{
    if (item.m_active)
    {
        os << "is active, timer=" << static_cast<uint32_t>(item.m_timer)
           << " Status: " << static_cast<uint32_t>(item.m_status) << " txSlot " << item.m_txSlot
           << (item.m_dciElement ? " rv " + std::to_string(item.m_dciElement->m_rv) +
                                       " reps " + std::to_string(item.m_dciElement->m_repetitions)
                                 : std::string());
    }
    return os;
}

} // namespace ns3
