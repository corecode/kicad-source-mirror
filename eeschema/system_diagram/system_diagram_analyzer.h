/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SYSTEM_DIAGRAM_ANALYZER_H
#define SYSTEM_DIAGRAM_ANALYZER_H

#include <memory>
#include <vector>
#include <map>
#include <set>
#include <wx/string.h>
#include <math/vector2d.h>

class SCHEMATIC;


/// A component box on the diagram (IC or connector)
struct SD_COMPONENT
{
    wxString    m_reference;     ///< "U1", "J2"
    wxString    m_value;         ///< "STM32F405", "USB-C"
    VECTOR2I    m_pos;           ///< Assigned by layout
    VECTOR2I    m_size;          ///< Assigned by layout (based on text + connections)
};


/// A bus bar connecting components
struct SD_BUS
{
    wxString                     m_aliasName;           ///< "SPI1", "I2C1"
    std::vector<wxString>        m_members;             ///< Member net names from BUS_ALIAS
    std::vector<SD_COMPONENT*>   m_connectedComponents;
};


/// A specially marked signal (net class = "System_Diagram")
struct SD_SIGNAL
{
    wxString                     m_netName;             ///< "UART_TX", "RESET_N"
    std::vector<SD_COMPONENT*>   m_connectedComponents;
};


/// Power tree node
struct SD_POWER_NODE
{
    enum TYPE { SOURCE, REGULATOR, RAIL };

    TYPE        m_type;
    wxString    m_reference;     ///< "J1", "U4", "" (for rail nodes)
    wxString    m_value;         ///< "USB-C", "LM1117", "+3V3"
    wxString    m_netName;       ///< Output net
    double      m_voltage;       ///< Volts, or 0 if unknown
    VECTOR2I    m_pos;           ///< Assigned by layout
    VECTOR2I    m_size;

    std::vector<SD_POWER_NODE*>  m_children;
    std::vector<wxString>        m_loadRefs;  ///< References of ICs on this rail
};


/// Top-level container for all system diagram data
struct SYSTEM_DIAGRAM_DATA
{
    std::vector<std::unique_ptr<SD_COMPONENT>>   m_components;
    std::vector<std::unique_ptr<SD_BUS>>         m_buses;
    std::vector<std::unique_ptr<SD_SIGNAL>>      m_signals;
    std::vector<std::unique_ptr<SD_POWER_NODE>>  m_powerRoots;  ///< Forest of power trees

    /// Lookup helper
    SD_COMPONENT* FindComponent( const wxString& aRef );

    bool HasContent() const
    {
        return !m_buses.empty() || !m_signals.empty() || !m_powerRoots.empty();
    }
};


/**
 * Walks the schematic and builds bus graph + power tree data for the system diagram.
 */
class SYSTEM_DIAGRAM_ANALYZER
{
public:
    SYSTEM_DIAGRAM_ANALYZER( SCHEMATIC* aSchematic );

    /**
     * Run all analysis phases.
     * @return true if the analysis found any content to display.
     */
    bool Analyze();

    SYSTEM_DIAGRAM_DATA& GetData() { return m_data; }

private:
    /// Phase 1: Discover which components appear on the diagram
    void discoverComponents();

    /// Phase 2: Build bus connectivity from BUS_ALIAS definitions
    void buildBusGraph();

    /// Phase 3: Find specially marked nets (net class "System_Diagram")
    void findMarkedSignals();

    /// Phase 4: Build power tree from PT_POWER_IN / PT_POWER_OUT pins
    void buildPowerTree();

    /// Extract a numeric voltage from a power net name (e.g. "+3V3" -> 3.3)
    static double parseVoltage( const wxString& aNetName );

    SCHEMATIC*           m_schematic;
    SYSTEM_DIAGRAM_DATA  m_data;

    /// Map from reference to component pointer (built during discoverComponents)
    std::map<wxString, SD_COMPONENT*> m_refToComponent;

    /// Map from net name to set of component references connected to it
    std::map<wxString, std::set<wxString>> m_netToRefs;
};


#endif // SYSTEM_DIAGRAM_ANALYZER_H
