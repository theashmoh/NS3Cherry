// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_EESM_IR_H
#define NR_EESM_IR_H

#include "nr-eesm-error-model.h"

#include "ns3/nr-export.h"

namespace ns3
{

/**
 * @ingroup error-models
 * @brief Eesm error model, based on the IR HARQ
 *
 * In HARQ-IR, every retransmission contains different coded bits than the previous
 * one. The different retransmissions typically use a different set of coding bits.
 * Therefore, both the effective SINR and the equivalent ECR need to be recomputed
 * after each retransmission.
 *
 * In HARQ-IR, the HARQ history contains the last computed effective SINR and
 * number of coded bits of each of the previous retransmissions. Given the current
 * SINR vector and the HARQ history, the effective SINR is computed according to EESM.
 *
 * NOTE: The method GetMcsEq() must be called after ComputeSINR(), as it uses
 * the value m_Reff.
 *
 * The receiver keeps the coded bits of a TB in a finite soft buffer, the circular
 * buffer of TS 38.212 Sect. 5.4.2.1 (one per code block, Ncb bits each). The attribute
 * SoftBufferBitsPerTb is its total size for the TB, i.e. the sum of Ncb over the code
 * blocks, and bounds the coded bits that count towards the equivalent ECR. As rate
 * matching splits every transmission evenly over the code blocks, bounding the TB
 * total is equivalent to bounding each code block by Ncb. Coded bits beyond the
 * buffer repeat bits already buffered: they lower the ECR no further, and the receiver
 * adds their soft values to the buffered ones, which raises the effective SINR by the
 * ratio of the coded bits received to the buffer. The attribute CircularBuffer (default
 * true) sets the buffer from TS 38.212 (Ncb = N, no limited-buffer rate matching);
 * SoftBufferBitsPerTb, if not 0, bounds it further (alone if CircularBuffer is false).
 *
 * With the attribute ChaseFloor (default true) the error rate of a combined block is not
 * above that of chase combining the same transmissions at the transmitted MCS: a receiver
 * that keeps every coded bit does at least that well, which the abstraction (an averaged
 * SINR on the curve of a lower code rate) does not ensure at low code rates.
 *
 * Below the lowest code rate of its modulation (a block at MCS 0 retransmitted, or sent
 * with PUSCH repetition) the tables have no curve: the block takes the curve of the lowest
 * MCS of its modulation, with its effective SINR scaled by that MCS's code rate over the
 * equivalent one (the gain of chase combining).
 *
 * Please, don't use this class directly, but one between NrEesmIrT1 or NrEesmIrT2,
 * depending on what table you want to use.
 *
 */
class NR_EXPORT NrEesmIr : public NrEesmErrorModel
{
  public:
    /**
     * @brief Get the type id of the object
     * @return the type id of the object
     */
    static TypeId GetTypeId();
    /**
     * @brief NrEesmIr constructor
     */
    NrEesmIr();
    /**
     * @brief ~NrEesmIr deconstructor
     */
    ~NrEesmIr() override;

  protected:
    // Inherited from NrEesmErrorModel
    /**
     * @brief Computes the effective SINR after retransmission combining with HARQ-IR.
     * Also, it updates the equivalent ECR after retransmissions (m_Reff).
     *
     * @param sinr the SINR vector of current transmission
     * @param map the RB map of current transmission
     * @param sizeBit the Transport block size in bits
     * @param mcs the MCS
     * @param sinrHistory the History of the previous transmissions of the same block
     * @return The effective SINR
     */
    double ComputeSINR(const SpectrumValue& sinr,
                       const std::vector<int>& map,
                       uint8_t mcs,
                       uint32_t sizeBit,
                       const NrErrorModel::NrErrorModelHistory& sinrHistory) const override;

    /**
     * @brief Returns the MCS corresponding to the ECR after retransmissions. In case of
     * HARQ-IR the equivalent ECR changes after retransmissions, and it is updated
     * inside ComputeSINR function. GetMcsEq gets the closest ECR to m_Reff from
     * the available ones that belong to the same modulation order.
     *
     * @param mcsTx the MCS of the transmission
     * @return The equivalent MCS after retransmissions
     */
    double GetMcsEq(uint8_t mcsTx) const override;

    /**
     * @brief Whether a combined block's error rate is bounded by that of chase combining
     * @return the attribute ChaseFloor
     */
    bool ChaseFloor() const override;

  private:
    /**
     * @brief The soft buffer of a TB in coded bits: the circular buffers of its code blocks
     * (attribute CircularBuffer), bounded by SoftBufferBitsPerTb if that is set
     * @param infoBits the information bits of the TB (A of TS 38.212)
     * @param mcs the MCS of its first transmission
     * @return the buffer in coded bits, 0 = unlimited
     */
    uint32_t SoftBufferBits(uint32_t infoBits, uint8_t mcs) const;

    double m_Reff{0.0};                //!< equivalent effective code rate after retransmissions
    uint32_t m_softBufferBitsPerTb{0}; //!< soft-buffer size of a TB in coded bits, 0 = unlimited
    bool m_circularBuffer{true}; //!< bound the coded bits by the TS 38.212 circular buffer
    bool m_chaseFloor{true};     //!< error rate not above that of chase combining
    bool m_redundancyVersions{true}; //!< coded bits placed by each transmission's RV (k0)
};

} // namespace ns3
#endif // NR_EESM_IR_H
