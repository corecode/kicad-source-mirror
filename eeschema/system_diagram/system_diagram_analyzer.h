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

/// Field name prefix for power annotations (follows Sim.* dot-namespace convention).
/// Current draw fields use "Pwr.I." prefix, e.g.:
///   Pwr.I.typ = 150mA           (single-rail: mode only)
///   Pwr.I.VDD.typ = 100mA       (multi-rail: pin.mode)
///   Pwr.I.VDD = 80mA            (multi-rail: pin only, implicit "typ" mode)
#define SD_POWER_FIELD_PREFIX        wxT( "Pwr." )
#define SD_CURRENT_FIELD_PREFIX      wxT( "Pwr.I." )
#define SD_TYPE_FIELD                wxT( "Pwr.Type" )
#define SD_INPUT_PIN_FIELD           wxT( "Pwr.InputPin" )
#define SD_EFFICIENCY_FIELD_PREFIX   wxT( "Pwr.Eff." )

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
    enum CONVERTER_TYPE { CONV_UNKNOWN, CONV_LDO, CONV_SMPS, CONV_SWITCH };

    TYPE        m_type;
    wxString    m_reference;     ///< "J1", "U4", "" (for rail nodes)
    wxString    m_value;         ///< "USB-C", "LM1117", "+3V3"
    wxString    m_netName;       ///< Output net
    double      m_voltage;       ///< Volts, or 0 if unknown
    VECTOR2I    m_pos;           ///< Assigned by layout
    VECTOR2I    m_size;

    CONVERTER_TYPE              m_converterType = CONV_UNKNOWN;
    wxString                    m_inputNetName;      ///< Main power input net (for regulators)
    double                      m_inputVoltage = 0;  ///< Voltage on input net

    /// Efficiency per mode (0.0-1.0), from Pwr.Eff.* fields. Key is mode (e.g. "typ").
    std::map<wxString, double>  m_efficiencyByMode;

    /// Computed input current per mode (in amps), from bubble-up.
    std::map<wxString, double>  m_inputCurrentByMode;

    /// Non-main power input pins (bias supplies).
    struct BIAS_CONNECTION
    {
        wxString m_pinName;
        wxString m_netName;
        double   m_voltage = 0;
    };

    std::vector<BIAS_CONNECTION> m_biasConnections;

    std::vector<SD_POWER_NODE*>  m_children;
    std::vector<wxString>        m_loadRefs;  ///< References of ICs on this rail

    /// Aggregated output current per mode (in amps). Key is mode name (e.g. "typ", "max").
    /// After bubbleUpCurrents(), includes child regulator input currents.
    std::map<wxString, double>   m_currentByMode;
};


/// Top-level container for all system diagram data
struct SYSTEM_DIAGRAM_DATA
{
    std::vector<std::unique_ptr<SD_COMPONENT>>   m_components;
    std::vector<std::unique_ptr<SD_BUS>>         m_buses;
    std::vector<std::unique_ptr<SD_SIGNAL>>      m_signals;
    std::vector<std::unique_ptr<SD_POWER_NODE>>  m_allPowerNodes;  ///< Owns all power nodes
    std::vector<SD_POWER_NODE*>                  m_powerRoots;     ///< Root nodes (non-owning)

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

    /// Phase 5: Collect Pwr.Type, Pwr.Eff.* annotations on regulator symbols
    void collectPowerAnnotations();

    /// Phase 6: Collect Pwr.I.* field annotations and aggregate currents per power node
    void collectCurrentAnnotations();

    /// Phase 7: Propagate currents up the tree (child input currents → parent output)
    void bubbleUpCurrents();

public:
    /// Parse a current value string with SI suffix (e.g. "150mA") to amps
    static double parseCurrent( const wxString& aValue );

    /// Parse an efficiency value (e.g. "0.87", "87%", "87") to a 0-1 range
    static double parseEfficiency( const wxString& aValue );

    /// Format a current value in amps to a human-readable string (e.g. 0.15 -> "150.0mA")
    static wxString formatCurrent( double aAmps );

private:
    SCHEMATIC*           m_schematic;
    SYSTEM_DIAGRAM_DATA  m_data;

    /// Map from reference to component pointer (built during discoverComponents)
    std::map<wxString, SD_COMPONENT*> m_refToComponent;

    /// Map from net name to set of component references connected to it
    std::map<wxString, std::set<wxString>> m_netToRefs;
};


#endif // SYSTEM_DIAGRAM_ANALYZER_H
