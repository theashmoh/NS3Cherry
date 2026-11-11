// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-harq-vector.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacHarqVector");

bool NrMacHarqVector::s_diagnostics = false;

bool
NrMacHarqVector::Erase(uint8_t id, const char* site)
{
    if (s_diagnostics)
    {
        std::ostringstream o;
        o << "process " << +id << " erased by " << site << " at " << Simulator::Now().As(Time::S)
          << " (was active=" << at(id).m_active << " status=" << +at(id).m_status
          << " timer=" << at(id).m_timer
          << (at(id).m_dciElement ? " rv=" + std::to_string(at(id).m_dciElement->m_rv) : "")
          << "); before: " << m_lastErase.substr(0, 160);
        m_lastErase = o.str();
    }
    if (!at(id).m_active)
    {
        // (erased twice: nothing to do, and m_usedSize must not go down again)
        NS_LOG_WARN("HARQ process " << +id << " erased twice (" << site << ")");
        return false;
    }
    at(id).Erase();
    --m_usedSize;
#ifdef NS3_ASSERT_ENABLE
    uint32_t count = 0;
    for (const auto& v : *this)
    {
        if (v.second.m_active)
        {
            ++count;
        }
    }
    NS_ASSERT(count == m_usedSize);
#endif
    return true;
}

bool
NrMacHarqVector::Insert(uint8_t* id, const HarqProcess& element)
{
    if (m_usedSize >= m_maxSize)
    {
        return false;
    }

    NS_ABORT_IF(element.m_active == false);

    *id = FirstAvailableId();
    if (*id == 255)
    {
        return false;
    }

    NS_ABORT_IF(at(*id).m_active == true);
    at(*id) = element;

    NS_ABORT_IF(at(*id).m_active == false);
    NS_ABORT_IF(this->FirstAvailableId() == *id);

    ++m_usedSize;
    return true;
}

std::ostream&
operator<<(std::ostream& os, const NrMacHarqVector& item)
{
    for (const auto& p : item)
    {
        os << "Process ID " << static_cast<uint32_t>(p.first) << ": " << p.second << std::endl;
    }
    return os;
}

} // namespace ns3
