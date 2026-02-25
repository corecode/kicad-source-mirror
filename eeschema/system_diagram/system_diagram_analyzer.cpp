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

#include "system_diagram_analyzer.h"

#include <schematic.h>
#include <sch_symbol.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <bus_alias.h>
#include <connection_graph.h>
#include <sch_connection.h>
#include <pin_type.h>
#include <project.h>
#include <project/project_file.h>
#include <project/net_settings.h>


SD_COMPONENT* SYSTEM_DIAGRAM_DATA::FindComponent( const wxString& aRef )
{
    for( auto& comp : m_components )
    {
        if( comp->m_reference == aRef )
            return comp.get();
    }

    return nullptr;
}


SYSTEM_DIAGRAM_ANALYZER::SYSTEM_DIAGRAM_ANALYZER( SCHEMATIC* aSchematic ) :
        m_schematic( aSchematic )
{
}


bool SYSTEM_DIAGRAM_ANALYZER::Analyze()
{
    discoverComponents();
    buildBusGraph();
    findMarkedSignals();
    buildPowerTree();

    // Remove components that don't participate in any bus, signal, or power relationship
    // (they may have been discovered optimistically)
    std::set<wxString> participatingRefs;

    for( const auto& bus : m_data.m_buses )
    {
        for( SD_COMPONENT* comp : bus->m_connectedComponents )
            participatingRefs.insert( comp->m_reference );
    }

    for( const auto& sig : m_data.m_signals )
    {
        for( SD_COMPONENT* comp : sig->m_connectedComponents )
            participatingRefs.insert( comp->m_reference );
    }

    // Power nodes reference components by name, so add those too
    std::function<void( SD_POWER_NODE* )> collectPowerRefs;
    collectPowerRefs = [&]( SD_POWER_NODE* node )
    {
        if( !node->m_reference.IsEmpty() )
            participatingRefs.insert( node->m_reference );

        for( const wxString& ref : node->m_loadRefs )
            participatingRefs.insert( ref );

        for( SD_POWER_NODE* child : node->m_children )
            collectPowerRefs( child );
    };

    for( const auto& root : m_data.m_powerRoots )
        collectPowerRefs( root );

    // Prune non-participating components
    m_data.m_components.erase(
            std::remove_if( m_data.m_components.begin(), m_data.m_components.end(),
                            [&]( const std::unique_ptr<SD_COMPONENT>& comp )
                            {
                                return participatingRefs.find( comp->m_reference )
                                       == participatingRefs.end();
                            } ),
            m_data.m_components.end() );

    return m_data.HasContent();
}


void SYSTEM_DIAGRAM_ANALYZER::discoverComponents()
{
    SCH_SHEET_LIST sheets = m_schematic->BuildUnorderedSheetList();

    for( const SCH_SHEET_PATH& sheetPath : sheets )
    {
        SCH_SCREEN* screen = sheetPath.LastScreen();

        if( !screen )
            continue;

        for( SCH_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );
            wxString    ref = symbol->GetRef( &sheetPath, false );

            // Only include ICs (U?) and connectors (J?)
            if( !ref.StartsWith( wxT( "U" ) ) && !ref.StartsWith( wxT( "J" ) ) )
                continue;

            // Skip if already discovered (same symbol on multiple sheets in hierarchy)
            if( m_refToComponent.count( ref ) )
                continue;

            auto comp = std::make_unique<SD_COMPONENT>();
            comp->m_reference = ref;
            comp->m_value = symbol->GetValue( true, &sheetPath, false );

            SD_COMPONENT* ptr = comp.get();
            m_data.m_components.push_back( std::move( comp ) );
            m_refToComponent[ref] = ptr;

            // Build net-to-refs map from this symbol's pins
            for( SCH_PIN* pin : symbol->GetPins( &sheetPath ) )
            {
                SCH_CONNECTION* conn = pin->Connection( &sheetPath );

                if( conn )
                {
                    wxString netName = conn->GetNetName();

                    if( !netName.IsEmpty() )
                        m_netToRefs[netName].insert( ref );
                }
                else
                {
                    // Try the default net name for power pins
                    wxString defaultNet = pin->GetDefaultNetName( sheetPath );

                    if( !defaultNet.IsEmpty() )
                        m_netToRefs[defaultNet].insert( ref );
                }
            }
        }
    }
}


void SYSTEM_DIAGRAM_ANALYZER::buildBusGraph()
{
    SCH_SHEET_LIST sheets = m_schematic->BuildUnorderedSheetList();

    // Collect all bus aliases from all screens
    std::map<wxString, std::shared_ptr<BUS_ALIAS>> aliases;

    for( const SCH_SHEET_PATH& sheetPath : sheets )
    {
        SCH_SCREEN* screen = sheetPath.LastScreen();

        if( !screen )
            continue;

        for( const std::shared_ptr<BUS_ALIAS>& alias : screen->GetBusAliases() )
        {
            if( !aliases.count( alias->GetName() ) )
                aliases[alias->GetName()] = alias;
        }
    }

    // For each bus alias, find which components connect to its member nets
    for( auto& [aliasName, alias] : aliases )
    {
        std::set<SD_COMPONENT*> connectedComponents;

        for( const wxString& memberNet : alias->Members() )
        {
            auto it = m_netToRefs.find( memberNet );

            if( it != m_netToRefs.end() )
            {
                for( const wxString& ref : it->second )
                {
                    SD_COMPONENT* comp = m_data.FindComponent( ref );

                    if( comp )
                        connectedComponents.insert( comp );
                }
            }
        }

        // Only create a bus entry if 2+ components connect
        if( connectedComponents.size() >= 2 )
        {
            auto bus = std::make_unique<SD_BUS>();
            bus->m_aliasName = aliasName;
            bus->m_members = alias->Members();
            bus->m_connectedComponents.assign( connectedComponents.begin(),
                                               connectedComponents.end() );
            m_data.m_buses.push_back( std::move( bus ) );
        }
    }
}


void SYSTEM_DIAGRAM_ANALYZER::findMarkedSignals()
{
    // Look for nets assigned to the "System_Diagram" net class
    std::shared_ptr<NET_SETTINGS> netSettings =
            m_schematic->Prj().GetProjectFile().NetSettings();

    if( !netSettings )
        return;

    const std::map<wxString, std::set<wxString>>& assignments =
            netSettings->GetNetclassLabelAssignments();

    // Find all nets assigned to the "System_Diagram" net class
    std::set<wxString> markedNets;

    for( const auto& [netName, netclasses] : assignments )
    {
        if( netclasses.count( wxT( "System_Diagram" ) ) )
            markedNets.insert( netName );
    }

    // For each marked net, find connected components
    for( const wxString& netName : markedNets )
    {
        std::set<SD_COMPONENT*> connectedComponents;

        auto it = m_netToRefs.find( netName );

        if( it != m_netToRefs.end() )
        {
            for( const wxString& ref : it->second )
            {
                SD_COMPONENT* comp = m_data.FindComponent( ref );

                if( comp )
                    connectedComponents.insert( comp );
            }
        }

        if( connectedComponents.size() >= 2 )
        {
            auto sig = std::make_unique<SD_SIGNAL>();
            sig->m_netName = netName;
            sig->m_connectedComponents.assign( connectedComponents.begin(),
                                               connectedComponents.end() );
            m_data.m_signals.push_back( std::move( sig ) );
        }
    }
}


void SYSTEM_DIAGRAM_ANALYZER::buildPowerTree()
{
    SCH_SHEET_LIST sheets = m_schematic->BuildUnorderedSheetList();

    // Structures to hold discovered power information
    struct PowerPin
    {
        wxString ref;
        wxString value;
        wxString netName;
        ELECTRICAL_PINTYPE type;
        bool isConnector;  // J? prefix
    };

    std::vector<PowerPin> powerOutPins;
    std::vector<PowerPin> powerInPins;

    // Map: output net name -> power node that drives it
    std::map<wxString, SD_POWER_NODE*> netToDriver;

    // Scan all symbols for power pins
    for( const SCH_SHEET_PATH& sheetPath : sheets )
    {
        SCH_SCREEN* screen = sheetPath.LastScreen();

        if( !screen )
            continue;

        for( SCH_ITEM* item : screen->Items().OfType( SCH_SYMBOL_T ) )
        {
            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );
            wxString    ref = symbol->GetRef( &sheetPath, false );
            wxString    value = symbol->GetValue( true, &sheetPath, false );
            bool        isConn = ref.StartsWith( wxT( "J" ) );

            // Skip power symbols themselves (they're just net labels)
            if( symbol->IsPower() )
                continue;

            for( SCH_PIN* pin : symbol->GetPins( &sheetPath ) )
            {
                ELECTRICAL_PINTYPE pinType = pin->GetType();
                wxString           netName;

                SCH_CONNECTION* conn = pin->Connection( &sheetPath );

                if( conn )
                    netName = conn->GetNetName();
                else
                    netName = pin->GetDefaultNetName( sheetPath );

                if( netName.IsEmpty() )
                    continue;

                if( pinType == ELECTRICAL_PINTYPE::PT_POWER_OUT )
                {
                    powerOutPins.push_back( { ref, value, netName, pinType, isConn } );
                }
                else if( pinType == ELECTRICAL_PINTYPE::PT_POWER_IN )
                {
                    powerInPins.push_back( { ref, value, netName, pinType, isConn } );
                }
            }
        }
    }

    // Group power-out pins by component reference to identify regulators vs sources
    // A regulator has both PT_POWER_IN and PT_POWER_OUT on different nets
    std::map<wxString, std::set<wxString>> refToPowerOutNets;
    std::map<wxString, std::set<wxString>> refToPowerInNets;
    std::map<wxString, wxString>           refToValue;

    for( const PowerPin& pp : powerOutPins )
    {
        refToPowerOutNets[pp.ref].insert( pp.netName );
        refToValue[pp.ref] = pp.value;
    }

    for( const PowerPin& pp : powerInPins )
    {
        refToPowerInNets[pp.ref].insert( pp.netName );

        if( refToValue.find( pp.ref ) == refToValue.end() )
            refToValue[pp.ref] = pp.value;
    }

    // Create power nodes for components with PT_POWER_OUT pins
    std::map<wxString, std::unique_ptr<SD_POWER_NODE>> allNodes;

    for( const auto& [ref, outNets] : refToPowerOutNets )
    {
        bool hasDistinctPowerIn = false;
        auto inIt = refToPowerInNets.find( ref );

        if( inIt != refToPowerInNets.end() )
        {
            // Check if the power-in nets are different from the power-out nets
            for( const wxString& inNet : inIt->second )
            {
                if( outNets.find( inNet ) == outNets.end() )
                {
                    hasDistinctPowerIn = true;
                    break;
                }
            }
        }

        bool isConnector = ref.StartsWith( wxT( "J" ) );

        for( const wxString& outNet : outNets )
        {
            auto node = std::make_unique<SD_POWER_NODE>();

            if( hasDistinctPowerIn && !isConnector )
                node->m_type = SD_POWER_NODE::REGULATOR;
            else
                node->m_type = SD_POWER_NODE::SOURCE;

            node->m_reference = ref;
            node->m_value = refToValue[ref];
            node->m_netName = outNet;
            node->m_voltage = parseVoltage( outNet );

            SD_POWER_NODE* ptr = node.get();
            wxString key = ref + wxT( ":" ) + outNet;
            netToDriver[outNet] = ptr;
            allNodes[key] = std::move( node );
        }
    }

    // Build tree edges: for each regulator, find its input net's driver
    for( auto& [key, node] : allNodes )
    {
        if( node->m_type != SD_POWER_NODE::REGULATOR )
            continue;

        auto inIt = refToPowerInNets.find( node->m_reference );

        if( inIt == refToPowerInNets.end() )
            continue;

        for( const wxString& inNet : inIt->second )
        {
            // Skip if this is the same as the output net
            if( inNet == node->m_netName )
                continue;

            auto driverIt = netToDriver.find( inNet );

            if( driverIt != netToDriver.end() )
            {
                driverIt->second->m_children.push_back( node.get() );
                break;  // Only connect to first found parent
            }
        }
    }

    // Find loads on each power output net
    for( auto& [key, node] : allNodes )
    {
        auto netIt = m_netToRefs.find( node->m_netName );

        if( netIt == m_netToRefs.end() )
            continue;

        for( const wxString& loadRef : netIt->second )
        {
            // Skip the node's own component and power symbols
            if( loadRef == node->m_reference )
                continue;

            // Only include U? components as loads (not other regulators' inputs)
            if( !loadRef.StartsWith( wxT( "U" ) ) )
                continue;

            // Check this component doesn't have a PT_POWER_OUT on this same net
            auto outIt = refToPowerOutNets.find( loadRef );

            if( outIt != refToPowerOutNets.end()
                && outIt->second.count( node->m_netName ) )
            {
                continue;
            }

            node->m_loadRefs.push_back( loadRef );
        }
    }

    // Identify root nodes (nodes that aren't children of any other node)
    std::set<SD_POWER_NODE*> childNodes;

    for( const auto& [key, node] : allNodes )
    {
        for( SD_POWER_NODE* child : node->m_children )
            childNodes.insert( child );
    }

    // Transfer ALL nodes to the data container for ownership, then record roots
    for( auto& [key, node] : allNodes )
    {
        SD_POWER_NODE* ptr = node.get();
        m_data.m_allPowerNodes.push_back( std::move( node ) );

        if( childNodes.find( ptr ) == childNodes.end() )
            m_data.m_powerRoots.push_back( ptr );
    }
}


double SYSTEM_DIAGRAM_ANALYZER::parseVoltage( const wxString& aNetName )
{
    // Parse voltage from common power net naming patterns:
    // "+3V3" -> 3.3, "+5V" -> 5.0, "+1V8" -> 1.8, "+12V" -> 12.0
    // "VCC" -> 0 (unknown), "GND" -> 0

    wxString name = aNetName;

    // Strip leading '+' or '-'
    if( name.StartsWith( wxT( "+" ) ) || name.StartsWith( wxT( "-" ) ) )
        name = name.Mid( 1 );

    // Try to parse patterns like "3V3", "5V", "1V8", "12V", "3.3V"
    wxString numStr;
    bool     foundV = false;
    bool     foundDecimal = false;

    for( size_t i = 0; i < name.Length(); i++ )
    {
        wxChar ch = name[i];

        if( ch >= '0' && ch <= '9' )
        {
            numStr += ch;
        }
        else if( ( ch == 'V' || ch == 'v' ) && !numStr.IsEmpty() )
        {
            foundV = true;

            // Check if there are digits after V (like "3V3" -> 3.3)
            if( i + 1 < name.Length() && name[i + 1] >= '0' && name[i + 1] <= '9' )
            {
                numStr += '.';
                foundDecimal = true;
            }
        }
        else if( ch == '.' && !foundDecimal )
        {
            numStr += '.';
            foundDecimal = true;
        }
    }

    if( foundV && !numStr.IsEmpty() )
    {
        double voltage = 0.0;

        if( numStr.ToDouble( &voltage ) )
        {
            if( aNetName.StartsWith( wxT( "-" ) ) )
                return -voltage;

            return voltage;
        }
    }

    return 0.0;
}
