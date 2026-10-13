// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>

#ifndef NR_PDCP_HEADER_H
#define NR_PDCP_HEADER_H

#include "ns3/header.h"
#include "ns3/nr-export.h"

#include <list>

namespace ns3
{

/**
 * @ingroup nr
 * @brief The packet header for the Packet Data Convergence Protocol (PDCP) packets
 *
 * This class has fields corresponding to those in an PDCP header as well as
 * methods for serialization to and deserialization from a byte buffer.
 * It follows 3GPP TS 36.323 Packet Data Convergence Protocol (PDCP) specification.
 */
class NR_EXPORT NrPdcpHeader : public Header
{
  public:
    /**
     * @brief Constructor
     *
     * Creates a null header
     */
    NrPdcpHeader();
    ~NrPdcpHeader() override;

    /**
     * @brief Set DC bit
     *
     * @param dcBit DC bit to set
     */
    void SetDcBit(uint8_t dcBit);
    /**
     * @brief Set sequence number
     *
     * @param sequenceNumber sequence number
     */
    void SetSequenceNumber(uint16_t sequenceNumber);

    /**
     * @brief Choose the format of the data PDU
     *
     * 12 (the default): two octets, D/C, three reserved bits and a 12-bit sequence number.
     * 7: one octet, D/C and a 7-bit sequence number (TS 36.323, 6.2.4, the format for data
     * radio bearers on RLC UM). The receiver has to set the same value before it reads a header.
     *
     * @param bits 12 or 7
     */
    void SetSnSizeBits(uint8_t bits);

    /**
     * @brief Get DC bit
     *
     * @returns DC bit
     */
    uint8_t GetDcBit() const;
    /**
     * @brief Get sequence number
     *
     * @returns sequence number
     */
    uint16_t GetSequenceNumber() const;

    /// DcBit_t typedef
    enum
    {
        CONTROL_PDU = 0,
        DATA_PDU = 1
    } DcBit_t; ///< DcBit_t typedef

    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    void Print(std::ostream& os) const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;

  private:
    uint8_t m_snSizeBits{12};  ///< bits of the sequence number: 12 or 7
    uint8_t m_dcBit;           ///< the DC bit
    uint16_t m_sequenceNumber; ///< the sequence number
};

} // namespace ns3

#endif // NR_PDCP_HEADER_H
