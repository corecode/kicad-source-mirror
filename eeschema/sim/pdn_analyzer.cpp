/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2024-2026 KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 * or you may search the http://www.gnu.org website for the version 2 license,
 * or you may write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#include "pdn_analyzer.h"

#include <schematic.h>
#include <sch_screen.h>
#include <sch_symbol.h>
#include <sch_pin.h>
#include <sch_connection.h>
#include <sch_sheet_path.h>
#include <sch_reference_list.h>
#include <pin_type.h>
#include <connection_graph.h>
#include <sim/simulator.h>
#include <sim/spice_simulator.h>
#include <sim/spice_settings.h>
#include <sim/spice_value.h>
#include <locale_io.h>

#include <wx/regex.h>
#include <cmath>
#include <set>


static const std::map<wxString, PARASITIC_ENTRY> s_parasiticTable =
{
    { wxS( "0402" ), { 0.100,  70e-12 } },
    { wxS( "0603" ), { 0.050, 120e-12 } },
    { wxS( "1005" ), { 0.030, 200e-12 } },
    { wxS( "1608" ), { 0.020, 400e-12 } },
    { wxS( "2012" ), { 0.015, 600e-12 } },
    { wxS( "3216" ), { 0.010, 900e-12 } },
    { wxS( "3225" ), { 0.008,   1e-9  } },
    { wxS( "4532" ), { 0.005, 1.2e-9  } },
};

static const std::map<wxString, wxString> s_eiaToMetric =
{
    { wxS( "01005" ), wxS( "0402" ) },
    { wxS( "0201" ),  wxS( "0603" ) },
    { wxS( "0402" ),  wxS( "1005" ) },
    { wxS( "0603" ),  wxS( "1608" ) },
    { wxS( "0805" ),  wxS( "2012" ) },
    { wxS( "1206" ),  wxS( "3216" ) },
    { wxS( "1210" ),  wxS( "3225" ) },
    { wxS( "1812" ),  wxS( "4532" ) },
};


PDN_ANALYZER::PDN_ANALYZER() :
        m_schematic( nullptr )
{
}


PDN_ANALYZER::~PDN_ANALYZER()
{
}


void PDN_ANALYZER::addWarning( const wxString& aNetworkKey, const wxString& aWarning )
{
    m_warnings[aNetworkKey].push_back( aWarning );
}


std::vector<wxString> PDN_ANALYZER::GetWarnings( const wxString& aNetworkKey ) const
{
    if( !aNetworkKey.IsEmpty() )
    {
        auto it = m_warnings.find( aNetworkKey );

        if( it != m_warnings.end() )
            return it->second;

        return {};
    }

    std::vector<wxString> all;

    for( const auto& [key, warnings] : m_warnings )
        all.insert( all.end(), warnings.begin(), warnings.end() );

    return all;
}


wxString PDN_ANALYZER::ParseCaseSize( const wxString& aFootprint )
{
    // Find all numeric groups of 4-5 digits and match against known case sizes.
    // Some sizes (e.g. 0402, 0603) are ambiguous: they exist as both metric and
    // EIA sizes.  Prefer unambiguous metric matches, then unambiguous EIA
    // matches, then treat ambiguous matches as EIA.
    static wxRegEx digitGroup( wxS( "([0-9]{4,5})" ) );

    wxString remaining = aFootprint;
    wxString bestMetric;
    wxString bestEia;
    wxString firstAmbiguous;

    while( digitGroup.Matches( remaining ) )
    {
        wxString digits = digitGroup.GetMatch( remaining, 1 );

        bool inMetric = s_parasiticTable.count( digits ) > 0;
        bool inEia = s_eiaToMetric.count( digits ) > 0;

        if( inMetric && !inEia && bestMetric.IsEmpty() )
            bestMetric = digits;
        else if( inEia && !inMetric && bestEia.IsEmpty() )
            bestEia = s_eiaToMetric.at( digits );
        else if( inMetric && inEia && firstAmbiguous.IsEmpty() )
            firstAmbiguous = digits;

        size_t start, len;
        digitGroup.GetMatch( &start, &len, 0 );
        remaining = remaining.Mid( start + len );
    }

    if( !bestMetric.IsEmpty() )
        return bestMetric;

    if( !bestEia.IsEmpty() )
        return bestEia;

    if( !firstAmbiguous.IsEmpty() )
        return s_eiaToMetric.at( firstAmbiguous );

    return wxString();
}


std::optional<PARASITIC_ENTRY> PDN_ANALYZER::GetParasitics( const wxString& aCaseSize )
{
    auto it = s_parasiticTable.find( aCaseSize );

    if( it != s_parasiticTable.end() )
        return it->second;

    return std::nullopt;
}


/**
 * Classify which net is the supply rail and which is the reference rail.
 *
 * @param aNet0 first net name
 * @param aNet1 second net name
 * @param aNetPinCount map of net names to total pin counts
 * @param[out] aSupplyNet set to the supply rail name
 * @param[out] aRefNet set to the reference rail name
 */
static void classifySupplyAndRef( const wxString& aNet0, const wxString& aNet1,
                                  const std::map<wxString, int>& aNetPinCount, wxString& aSupplyNet,
                                  wxString& aRefNet )
{
    static wxRegEx refNetPattern( wxS( "(gnd|vss|ground|vee)" ), wxRE_ICASE );

    bool n0Ref = refNetPattern.Matches( aNet0 );
    bool n1Ref = refNetPattern.Matches( aNet1 );

    if( n1Ref && !n0Ref )
    {
        aSupplyNet = aNet0;
        aRefNet = aNet1;
    }
    else if( n0Ref && !n1Ref )
    {
        aSupplyNet = aNet1;
        aRefNet = aNet0;
    }
    else
    {
        auto it0 = aNetPinCount.find( aNet0 );
        auto it1 = aNetPinCount.find( aNet1 );
        int  count0 = ( it0 != aNetPinCount.end() ) ? it0->second : 0;
        int  count1 = ( it1 != aNetPinCount.end() ) ? it1->second : 0;

        if( count1 > count0 )
        {
            aSupplyNet = aNet0;
            aRefNet = aNet1;
        }
        else if( count0 > count1 )
        {
            aSupplyNet = aNet1;
            aRefNet = aNet0;
        }
        else
        {
            aSupplyNet = ( aNet0 < aNet1 ) ? aNet0 : aNet1;
            aRefNet = ( aNet0 < aNet1 ) ? aNet1 : aNet0;
        }
    }
}


void PDN_ANALYZER::FindPDNNetworks()
{
    m_networks.clear();
    m_warnings.clear();

    if( !m_schematic )
        return;

    SCH_SHEET_LIST sheets = m_schematic->Hierarchy();
    SCH_REFERENCE_LIST references;
    sheets.GetSymbols( references, false /* aIncludePowerSymbols */ );

    std::set<wxString>      powerNetNames;
    std::map<wxString, int> netPinCount;

    // Iterate all sheet instances.  Multi-instance sheets share the same
    // SCH_SCREEN (via LastScreen()), but Connection( &sheet ) returns
    // per-instance net names, so every instance is handled correctly.
    for( const SCH_SHEET_PATH& sheet : sheets )
    {
        for( SCH_ITEM* item : sheet.LastScreen()->Items() )
        {
            if( item->Type() != SCH_SYMBOL_T )
                continue;

            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            for( SCH_PIN* pin : symbol->GetPins( &sheet ) )
            {
                SCH_CONNECTION* conn = pin->Connection( &sheet );

                if( conn )
                    netPinCount[conn->Name()]++;

                if( pin->GetType() == ELECTRICAL_PINTYPE::PT_POWER_IN
                    || pin->GetType() == ELECTRICAL_PINTYPE::PT_POWER_OUT
                    || pin->IsGlobalPower() )
                {
                    if( conn )
                        powerNetNames.insert( conn->Name() );
                }
            }
        }
    }

    // Match C or CP prefix followed by a digit or '?' (unassigned refdes).
    // Rejects CONN, CMOS, CE, etc.
    static wxRegEx capRefRegex( wxS( "^CP?[0-9?]" ) );

    for( size_t i = 0; i < references.GetCount(); i++ )
    {
        const SCH_REFERENCE& ref = references[i];
        wxString             refStr = ref.GetRef();

        if( !capRefRegex.Matches( refStr ) )
            continue;

        SCH_SYMBOL*           symbol = ref.GetSymbol();
        const SCH_SHEET_PATH& sheetPath = ref.GetSheetPath();

        std::vector<SCH_PIN*> pins = symbol->GetPins( &sheetPath );

        if( pins.size() != 2 )
            continue;

        wxString net0, net1;

        if( SCH_CONNECTION* conn = pins[0]->Connection( &sheetPath ) )
            net0 = conn->Name();

        if( SCH_CONNECTION* conn = pins[1]->Connection( &sheetPath ) )
            net1 = conn->Name();

        if( net0.IsEmpty() || net1.IsEmpty() )
            continue;

        bool net0Power = powerNetNames.count( net0 ) > 0;
        bool net1Power = powerNetNames.count( net1 ) > 0;

        if( !net0Power || !net1Power )
            continue;

        wxString supplyNet, refNet;
        classifySupplyAndRef( net0, net1, netPinCount, supplyNet, refNet );

        wxString key = supplyNet + wxS( " / " ) + refNet;

        wxString valueStr = ref.GetValue();
        wxString footprint = ref.GetFootprint();

        double capacitance = SPICE_VALUE( valueStr ).ToDouble();

        if( capacitance <= 0.0 )
        {
            addWarning( key,
                        wxString::Format( _( "Skipping %s — unable to parse capacitance '%s'." ),
                                          refStr, valueStr ) );
            continue;
        }

        wxString caseSize = ParseCaseSize( footprint );

        if( caseSize.IsEmpty() )
        {
            addWarning( key, wxString::Format(
                                     _( "Skipping %s — unable to determine case size from '%s'." ),
                                     refStr, footprint ) );
            continue;
        }

        std::optional<PARASITIC_ENTRY> parasitics = GetParasitics( caseSize );

        if( !parasitics )
        {
            addWarning( key, wxString::Format(
                                     _( "Skipping %s — no parasitic data for case size '%s'." ),
                                     refStr, caseSize ) );
            continue;
        }

        PDN_CAPACITOR cap;
        cap.refdes = refStr;
        cap.capacitance = capacitance;
        cap.caseSize = caseSize;
        cap.esr = parasitics->esr;
        cap.esl = parasitics->esl;
        cap.netSupply = supplyNet;
        cap.netRef = refNet;
        cap.sheetPath = sheetPath;

        PDN_NETWORK& network = m_networks[key];
        network.supplyRail = supplyNet;
        network.refRail = refNet;
        network.components.push_back( cap );
    }
}


wxString PDN_ANALYZER::FindNetworkForNetName( const wxString& aNetName ) const
{
    // First try supply rail — always unique per network.
    for( const auto& [key, network] : m_networks )
    {
        if( network.supplyRail == aNetName )
            return key;
    }

    // Try reference rail only if it uniquely identifies one network.
    // Common rails like GND appear in many networks, so matching would
    // be ambiguous.
    wxString refMatch;

    for( const auto& [key, network] : m_networks )
    {
        if( network.refRail == aNetName )
        {
            if( !refMatch.IsEmpty() )
                return wxString(); // ambiguous — more than one network

            refMatch = key;
        }
    }

    return refMatch;
}


wxString PDN_ANALYZER::FindNetworkForSym( const wxString& aRefdes ) const
{
    for( const auto& [key, network] : m_networks )
    {
        for( const PDN_CAPACITOR& cap : network.components )
        {
            if( cap.refdes == aRefdes )
                return key;
        }
    }

    return wxString();
}


bool PDN_ANALYZER::NetworkHasSymsOnSheet( const wxString&       aNetworkKey,
                                          const SCH_SHEET_PATH& aSheet ) const
{
    auto it = m_networks.find( aNetworkKey );

    if( it == m_networks.end() )
        return false;

    wxString sheetKey = aSheet.PathAsString();

    for( const PDN_CAPACITOR& cap : it->second.components )
    {
        if( cap.sheetPath.PathAsString() == sheetKey )
            return true;
    }

    return false;
}


bool PDN_ANALYZER::NetworkIsMultiSheet( const wxString& aNetworkKey ) const
{
    auto it = m_networks.find( aNetworkKey );

    if( it == m_networks.end() )
        return false;

    std::set<wxString> sheetKeys;

    for( const PDN_CAPACITOR& cap : it->second.components )
        sheetKeys.insert( cap.sheetPath.PathAsString() );

    return sheetKeys.size() > 1;
}


wxString PDN_ANALYZER::BuildSpiceNetlist( const PDN_NETWORK& aNetwork,
                                          const SCH_SHEET_PATH& aObservationSheet,
                                          double aTraceInductance, double aTraceResistance,
                                          double aStartFreq, double aEndFreq,
                                          int aPointsPerDecade )
{
    wxString obsKey = aObservationSheet.PathAsString();

    std::map<wxString, std::vector<const PDN_CAPACITOR*>> sheetGroups;

    for( const PDN_CAPACITOR& cap : aNetwork.components )
        sheetGroups[cap.sheetPath.PathAsString()].push_back( &cap );

    wxString netlist;

    netlist += wxString::Format( wxS( "* PDN Impedance Analysis: %s / %s\n\n" ),
                                 aNetwork.supplyRail, aNetwork.refRail );

    // Generate subcircuits for all known case sizes
    for( const auto& [cs, parasitic] : s_parasiticTable )
    {
        netlist += wxString::Format( wxS( ".subckt CAP_%s p n c=100n\n" ), cs );
        netlist += wxS( "C1 p 1 {c}\n" );
        netlist += wxString::Format( wxS( "R1 1 2 %g\n" ), parasitic.esr );
        netlist += wxString::Format( wxS( "L1 2 n %g\n" ), parasitic.esl );
        netlist += wxS( ".ends\n\n" );
    }

    netlist += wxS( "I1 local gnd AC 1\n" );

    int idx = 1;
    int groupIdx = 1;

    // Local caps (on observation sheet) connect directly to the measurement node.
    auto localIt = sheetGroups.find( obsKey );

    if( localIt != sheetGroups.end() )
    {
        for( const PDN_CAPACITOR* cap : localIt->second )
        {
            netlist += wxString::Format( wxS( "X%d local gnd CAP_%s c=%g\n" ), idx, cap->caseSize,
                                         cap->capacitance );
            idx++;
        }
    }

    // Remote caps (other sheets) go through per-hop trace impedance.
    for( const auto& [sheetKey, caps] : sheetGroups )
    {
        if( sheetKey == obsKey )
            continue;

        wxString nodeL = wxString::Format( wxS( "rs%d" ), groupIdx );
        wxString nodeR = wxString::Format( wxS( "rs%db" ), groupIdx );

        netlist +=
                wxString::Format( wxS( "L_s%d local %s %g\n" ), groupIdx, nodeL, aTraceInductance );
        netlist += wxString::Format( wxS( "R_s%d %s %s %g\n" ), groupIdx, nodeL, nodeR,
                                     aTraceResistance );

        for( const PDN_CAPACITOR* cap : caps )
        {
            netlist += wxString::Format( wxS( "X%d %s gnd CAP_%s c=%g\n" ), idx, nodeR,
                                         cap->caseSize, cap->capacitance );
            idx++;
        }

        groupIdx++;
    }

    double decades = std::log10( aEndFreq / aStartFreq );
    int    totalPoints = static_cast<int>( decades * aPointsPerDecade );

    netlist += wxString::Format( wxS( "\n.ac dec %d %g %g\n" ), totalPoints, aStartFreq, aEndFreq );
    netlist += wxS( ".end\n" );

    return netlist;
}


bool PDN_ANALYZER::RunAnalysis( const PDN_NETWORK& aNetwork,
                                 const SCH_SHEET_PATH& aObservationSheet,
                                 double aTraceInductance, double aTraceResistance )
{
    LOCALE_IO dummy;

    m_frequencies.clear();
    m_impedance.clear();

    wxString key = aNetwork.supplyRail + wxS( " / " ) + aNetwork.refRail;

    if( aNetwork.components.empty() )
    {
        addWarning( key, wxString::Format( _( "Network '%s' has no capacitors." ),
                                           aNetwork.supplyRail ) );
        return false;
    }

    std::shared_ptr<SPICE_SIMULATOR> sim = SIMULATOR::CreateInstance( "ngspice" );

    if( !sim )
    {
        addWarning( key, _( "Failed to create ngspice simulator instance." ) );
        return false;
    }

    if( !sim->Settings() )
        sim->Settings() = std::make_shared<NGSPICE_SETTINGS>( nullptr, "" );

    sim->Init();

    wxString netlist = BuildSpiceNetlist( aNetwork, aObservationSheet,
                                          aTraceInductance, aTraceResistance );

    if( !sim->LoadNetlist( netlist.ToStdString() ) )
    {
        addWarning( key, wxString::Format( _( "Failed to load netlist for '%s'." ),
                                           aNetwork.supplyRail ) );
        return false;
    }

    if( !sim->Command( "run" ) )
    {
        addWarning( key,
                    wxString::Format( _( "Simulation failed for '%s'." ), aNetwork.supplyRail ) );
        return false;
    }

    m_frequencies = sim->GetGainVector( "frequency" );
    m_impedance = sim->GetGainVector( "v(local)" );

    if( m_frequencies.empty() || m_impedance.empty() )
    {
        addWarning( key,
                    wxString::Format( _( "No results returned for '%s'." ), aNetwork.supplyRail ) );
        return false;
    }

    return true;
}
