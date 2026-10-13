// Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>

#ifndef NR_RLC_SEQUENCE_NUMBER_H
#define NR_RLC_SEQUENCE_NUMBER_H

#include "ns3/assert.h"
#include "ns3/nr-export.h"

#include <iostream>
#include <limits>
#include <stdint.h>

namespace ns3
{
namespace nr
{
/// SequenceNumber10 class
class NR_EXPORT SequenceNumber10
{
  public:
    SequenceNumber10()
        : m_value(0),
          m_modulusBase(0),
          m_modulus(1024)
    {
    }

    /**
     * Constructor
     *
     * @param value the value
     */
    explicit SequenceNumber10(uint16_t value)
        : m_value(value % 1024),
          m_modulusBase(0),
          m_modulus(1024)
    {
    }

    /**
     * Constructor
     *
     * @param value the value
     */
    SequenceNumber10(const SequenceNumber10& value)
        : m_value(value.m_value),
          m_modulusBase(value.m_modulusBase),
          m_modulus(value.m_modulus)
    {
    }

    /**
     * Copy assignment: value, modulus base and modulus
     *
     * @param value the value
     * @returns SequenceNumber10
     */
    SequenceNumber10& operator=(const SequenceNumber10& value) = default;

    /**
     * Assignment operator
     *
     * @param value the value
     * @returns SequenceNumber10
     */
    SequenceNumber10& operator=(uint16_t value)
    {
        m_value = value % m_modulus;
        return *this;
    }

    /**
     * @brief Set how many values the sequence number has
     *
     * 1024 (10 bits) unless set otherwise; 32 is the 5-bit sequence number of RLC UM
     * (TS 36.322, 6.2.1.3). The numbers that the operators return keep the modulus.
     *
     * @param modulus the number of values, a power of two
     */
    void SetModulus(uint16_t modulus)
    {
        NS_ASSERT(modulus > 0 && 65536 % modulus == 0);
        m_modulus = modulus;
        m_value %= modulus;
        m_modulusBase %= modulus;
    }

    /**
     * @returns how many values the sequence number has
     */
    uint16_t GetModulus() const
    {
        return m_modulus;
    }

    /**
     * @brief Extracts the numeric value of the sequence number
     * @returns the sequence number value
     */
    uint16_t GetValue() const
    {
        return m_value;
    }

    /**
     * @brief Set modulus base
     * @param modulusBase the modulus
     */
    void SetModulusBase(SequenceNumber10 modulusBase)
    {
        m_modulusBase = modulusBase.m_value;
    }

    /**
     * @brief Set modulus base
     * @param modulusBase the modulus
     */
    void SetModulusBase(uint16_t modulusBase)
    {
        m_modulusBase = modulusBase;
    }

    /**
     * postfix ++ operator
     * @returns SequenceNumber10
     */
    SequenceNumber10 operator++(int)
    {
        SequenceNumber10 retval(*this);
        m_value = ((uint32_t)m_value + 1) % m_modulus;
        return retval;
    }

    /**
     * addition operator
     * @param delta the amount to add
     * @returns SequenceNumber10
     */
    SequenceNumber10 operator+(uint16_t delta) const
    {
        SequenceNumber10 ret(*this);
        ret.m_value = (m_value + delta) % m_modulus;
        return ret;
    }

    /**
     * subtraction operator
     * @param delta the amount to subtract
     * @returns SequenceNumber10
     */
    SequenceNumber10 operator-(uint16_t delta) const
    {
        SequenceNumber10 ret(*this);
        // as a 16-bit number first, so a negative difference wraps the right way
        ret.m_value = static_cast<uint16_t>(m_value - delta) % m_modulus;
        return ret;
    }

    /**
     * subtraction operator
     * @param other the amount to subtract
     * @returns SequenceNumber10
     */
    uint16_t operator-(const SequenceNumber10& other) const
    {
        uint16_t diff = m_value - other.m_value;
        return diff;
    }

    /**
     * greater than operator
     * @param other the object to compare
     * @returns true if greater than
     */
    bool operator>(const SequenceNumber10& other) const
    {
        NS_ASSERT(m_modulusBase == other.m_modulusBase);
        uint16_t v1 = (m_value - m_modulusBase) % m_modulus;
        uint16_t v2 = (other.m_value - other.m_modulusBase) % other.m_modulus;
        return v1 > v2;
    }

    /**
     * equality operator
     * @param other the object to compare
     * @returns true if equal
     */
    bool operator==(const SequenceNumber10& other) const
    {
        return (m_value == other.m_value);
    }

    /**
     * inequality operator
     * @param other the object to compare
     * @returns true if not equal
     */
    bool operator!=(const SequenceNumber10& other) const
    {
        return (m_value != other.m_value);
    }

    /**
     * less than or equal operator
     * @param other the object to compare
     * @returns true if less than or equal
     */
    bool operator<=(const SequenceNumber10& other) const
    {
        return (!this->operator>(other));
    }

    /**
     * greater than or equal operator
     * @param other the object to compare
     * @returns true if greater than or equal
     */
    bool operator>=(const SequenceNumber10& other) const
    {
        return (this->operator>(other) || this->operator==(other));
    }

    /**
     * less than operator
     * @param other the object to compare
     * @returns true if less than
     */
    bool operator<(const SequenceNumber10& other) const
    {
        return !this->operator>(other) && m_value != other.m_value;
    }

    friend NR_EXPORT std::ostream& operator<<(std::ostream& os, const SequenceNumber10& val);

  private:
    uint16_t m_value;       ///< the value
    uint16_t m_modulusBase; ///< the modulus base
    uint16_t m_modulus;     ///< how many values the number has (1024 unless set)
};

} // namespace nr
} // namespace ns3

#endif // NR_RLC_SEQUENCE_NUMBER_H
