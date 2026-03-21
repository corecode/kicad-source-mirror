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
#include "spice_model_downloader.h"

#include <schematic.h>
#include <sch_screen.h>
#include <sch_symbol.h>
#include <sch_pin.h>
#include <sch_connection.h>
#include <sch_sheet.h>
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
#include <queue>
#include <set>


/**
 * Read Pwr.Zout and Pwr.BW fields from a symbol and populate VRM_PARAMS.
 *
 * Pwr.Zout sets rDamp (output impedance seen at the regulator output).
 * rVrm is derived as rDamp / 50 (DC resistance is a small fraction of Zout).
 * Pwr.BW sets the control loop bandwidth.
 * Missing fields keep their defaults.
 */
static VRM_PARAMS readVrmParams( SCH_SYMBOL* aSymbol, const SCH_SHEET_PATH& aSheet, const wxString& aNetworkKey,
                                 std::function<void( const wxString&, const wxString& )> aWarnFn )
{
    VRM_PARAMS params;

    for( const SCH_FIELD& field : aSymbol->GetFields() )
    {
        wxString name = field.GetName();

        if( name == wxT( "Pwr.Zout" ) )
        {
            wxString val = field.GetShownText( &aSheet, false );
            val.Trim( true ).Trim( false );

            try
            {
                double zout = SPICE_VALUE( val ).ToDouble();

                if( zout > 0.0 )
                {
                    params.rDamp = zout;
                    params.rVrm = zout / 50.0;
                }
            }
            catch( ... )
            {
                aWarnFn( aNetworkKey, wxString::Format( _( "%s: unable to parse Pwr.Zout '%s', using default." ),
                                                        aSymbol->GetRef( &aSheet, false ), val ) );
            }
        }
        else if( name == wxT( "Pwr.BW" ) )
        {
            wxString val = field.GetShownText( &aSheet, false );
            val.Trim( true ).Trim( false );

            try
            {
                double bw = SPICE_VALUE( val ).ToDouble();

                if( bw > 0.0 )
                    params.bandwidth = bw;
            }
            catch( ... )
            {
                aWarnFn( aNetworkKey, wxString::Format( _( "%s: unable to parse Pwr.BW '%s', using default." ),
                                                        aSymbol->GetRef( &aSheet, false ), val ) );
            }
        }
    }

    return params;
}


// Default ESL by metric case size [Henries].
// Source: Murata MLCC SPICE model database (23,603 parts), median values.
// ESL is primarily determined by package geometry and is nearly independent
// of capacitance value or dielectric type.
static const std::map<wxString, double> s_eslByCase = {
    { wxS( "0204" ), 116e-12 }, // 008004 EIA
    { wxS( "0402" ), 177e-12 }, // 01005
    { wxS( "0603" ), 219e-12 }, // 0201
    { wxS( "1005" ), 270e-12 }, // 0402
    { wxS( "1608" ), 380e-12 }, // 0603
    { wxS( "2012" ), 373e-12 }, // 0805
    { wxS( "2020" ), 373e-12 }, // 0808 (use 0805 value)
    { wxS( "3216" ), 542e-12 }, // 1206
    { wxS( "3225" ), 423e-12 }, // 1210
    { wxS( "4520" ), 841e-12 }, // 1808
    { wxS( "4532" ), 621e-12 }, // 1812
    { wxS( "5750" ), 1.22e-9 }, // 2220
};

// ESR heuristic for Class II MLCCs: ESR = k * C^alpha
// where C is capacitance in Farads and ESR is in Ohms.
// Derived from log-log regression on Murata SPICE model data for X5R/X7R/X8R.
// The exponent alpha is remarkably consistent across case sizes (median -0.43).
// k varies slightly by case size due to terminal geometry.
//
// This model is conservatively safe for PDN analysis: it tends to underestimate
// ESR at high capacitances (10 uF+), which produces pessimistically high-Q
// resonances — the user sees worst-case impedance peaks.
static constexpr double ESR_ALPHA = -0.429;

static const std::map<wxString, double> s_esrKByCase = {
    { wxS( "0204" ), 3.35e-5 }, // 008004 EIA
    { wxS( "0402" ), 3.87e-5 }, // 01005
    { wxS( "0603" ), 3.15e-5 }, // 0201
    { wxS( "1005" ), 3.15e-5 }, // 0402
    { wxS( "1608" ), 2.93e-5 }, // 0603
    { wxS( "2012" ), 2.93e-5 }, // 0805
    { wxS( "2020" ), 2.93e-5 }, // 0808 (use 0805 value)
    { wxS( "3216" ), 3.43e-5 }, // 1206
    { wxS( "3225" ), 2.39e-5 }, // 1210
    { wxS( "4520" ), 2.28e-5 }, // 1808
    { wxS( "4532" ), 2.28e-5 }, // 1812
    { wxS( "5750" ), 3.42e-5 }, // 2220
};

static const std::map<wxString, wxString> s_eiaToMetric = {
    { wxS( "008004" ), wxS( "0204" ) }, { wxS( "01005" ), wxS( "0402" ) }, { wxS( "0201" ), wxS( "0603" ) },
    { wxS( "0402" ), wxS( "1005" ) },   { wxS( "0603" ), wxS( "1608" ) },  { wxS( "0805" ), wxS( "2012" ) },
    { wxS( "0808" ), wxS( "2020" ) },   { wxS( "1206" ), wxS( "3216" ) },  { wxS( "1210" ), wxS( "3225" ) },
    { wxS( "1808" ), wxS( "4520" ) },   { wxS( "1812" ), wxS( "4532" ) },  { wxS( "2220" ), wxS( "5750" ) },
};


PDN_ANALYZER::PDN_ANALYZER() :
        m_schematic( nullptr ),
        m_downloader( nullptr )
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
        std::vector<wxString> result;

        // Network-specific warnings
        auto it = m_warnings.find( aNetworkKey );

        if( it != m_warnings.end() )
            result = it->second;

        // Also include global warnings (stored under empty key)
        auto globalIt = m_warnings.find( wxEmptyString );

        if( globalIt != m_warnings.end() )
            result.insert( result.end(), globalIt->second.begin(), globalIt->second.end() );

        return result;
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

        bool inMetric = s_eslByCase.count( digits ) > 0;
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


std::optional<PARASITIC_ENTRY> PDN_ANALYZER::GetParasitics( const wxString& aCaseSize, double aCapacitance )
{
    auto eslIt = s_eslByCase.find( aCaseSize );

    if( eslIt == s_eslByCase.end() )
        return std::nullopt;

    auto   kIt = s_esrKByCase.find( aCaseSize );
    double k = ( kIt != s_esrKByCase.end() ) ? kIt->second : 3.0e-5; // fallback k

    PARASITIC_ENTRY entry;
    entry.esl = eslIt->second;
    entry.esr = k * std::pow( aCapacitance, ESR_ALPHA );

    return entry;
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
                                  const std::map<wxString, int>& aNetPinCount, wxString& aSupplyNet, wxString& aRefNet )
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
    m_seriesElements.clear();
    m_warnings.clear();

    if( !m_schematic )
        return;

    SCH_SHEET_LIST     sheets = m_schematic->Hierarchy();
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
                    || pin->GetType() == ELECTRICAL_PINTYPE::PT_POWER_OUT || pin->IsGlobalPower() )
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
                        wxString::Format( _( "Skipping %s — unable to parse capacitance '%s'." ), refStr, valueStr ) );
            continue;
        }

        wxString caseSize = ParseCaseSize( footprint );

        if( caseSize.IsEmpty() )
        {
            addWarning( key, wxString::Format( _( "Skipping %s — unable to determine case size from '%s'." ), refStr,
                                               footprint ) );
            continue;
        }

        std::optional<PARASITIC_ENTRY> parasitics = GetParasitics( caseSize, capacitance );

        if( !parasitics )
        {
            addWarning( key, wxString::Format( _( "Skipping %s — no parasitic data for case size '%s'." ), refStr,
                                               caseSize ) );
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

    // Detect VRM devices: non-power-symbol components with PT_POWER_OUT on a supply
    // rail and PT_POWER_IN on a different net (i.e. regulators, not load switches).
    for( const SCH_SHEET_PATH& sheet : sheets )
    {
        for( SCH_ITEM* item : sheet.LastScreen()->Items() )
        {
            if( item->Type() != SCH_SYMBOL_T )
                continue;

            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            if( symbol->IsPower() )
                continue;

            wxString refStr = symbol->GetRef( &sheet, false );

            // Skip capacitors — already handled above
            if( capRefRegex.Matches( refStr ) )
                continue;

            // Check for Pwr.Type = SWITCH (load switch, not a regulator)
            bool isSwitch = false;

            for( const SCH_FIELD& field : symbol->GetFields() )
            {
                if( field.GetName() == wxT( "Pwr.Type" ) )
                {
                    wxString typeVal = field.GetText();
                    typeVal.Trim( true ).Trim( false );
                    typeVal.MakeUpper();

                    if( typeVal == wxT( "SWITCH" ) )
                        isSwitch = true;

                    break;
                }
            }

            if( isSwitch )
                continue;

            // Collect power-out and power-in nets for this symbol
            std::set<wxString> powerOutNets;
            std::set<wxString> powerInNets;

            for( SCH_PIN* pin : symbol->GetPins( &sheet ) )
            {
                SCH_CONNECTION* conn = pin->Connection( &sheet );

                if( !conn )
                    continue;

                wxString netName = conn->Name();

                if( pin->GetType() == ELECTRICAL_PINTYPE::PT_POWER_OUT )
                    powerOutNets.insert( netName );
                else if( pin->GetType() == ELECTRICAL_PINTYPE::PT_POWER_IN )
                    powerInNets.insert( netName );
            }

            // Must have distinct power-in and power-out nets to be a regulator
            bool hasDistinctInput = false;

            for( const wxString& inNet : powerInNets )
            {
                if( powerOutNets.find( inNet ) == powerOutNets.end() )
                {
                    hasDistinctInput = true;
                    break;
                }
            }

            if( !hasDistinctInput )
                continue;

            // Attach to any network whose supply rail matches a power-out net
            for( auto& [key, network] : m_networks )
            {
                if( network.vrm.has_value() )
                {
                    // Already has a VRM — warn about the duplicate
                    if( powerOutNets.count( network.supplyRail ) > 0 )
                    {
                        addWarning( key, wxString::Format( _( "Multiple regulators on '%s': using %s, "
                                                              "ignoring %s." ),
                                                           network.supplyRail, network.vrm->refdes, refStr ) );
                    }

                    continue;
                }

                if( powerOutNets.count( network.supplyRail ) > 0 )
                {
                    PDN_VRM vrm;
                    vrm.refdes = refStr;
                    vrm.sheetPath = sheet;
                    vrm.params = readVrmParams( symbol, sheet, key,
                                                [this]( const wxString& k, const wxString& w )
                                                {
                                                    addWarning( k, w );
                                                } );
                    network.vrm = vrm;
                }
            }
        }
    }

    // Detect series elements (resistors and ferrite beads) connecting power rails.
    // An element qualifies if both its pins are on rails (supply or reference) of
    // known PDN networks, and both rails serve the same role (both supply or both
    // reference).  This naturally handles ground-side ferrite beads.
    {
        std::set<wxString>                        supplyRails;
        std::set<wxString>                        refRails;
        std::map<wxString, std::vector<wxString>> railToKeys; // rail name → network keys

        for( const auto& [key, network] : m_networks )
        {
            supplyRails.insert( network.supplyRail );
            refRails.insert( network.refRail );
            railToKeys[network.supplyRail].push_back( key );
            railToKeys[network.refRail].push_back( key );
        }

        static wxRegEx ferriteRefRegex( wxS( "^FB[0-9?]" ) );
        static wxRegEx resistorSeriesRegex( wxS( "^R[0-9?]" ) );

        for( size_t i = 0; i < references.GetCount(); i++ )
        {
            const SCH_REFERENCE& ref = references[i];
            wxString             refStr = ref.GetRef();

            bool isResistor = resistorSeriesRegex.Matches( refStr );
            bool isFerrite = ferriteRefRegex.Matches( refStr );

            if( !isResistor && !isFerrite )
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

            if( net0.IsEmpty() || net1.IsEmpty() || net0 == net1 )
                continue;

            // Both nets must serve the same role: both supply rails or both ref rails
            bool bothSupply = supplyRails.count( net0 ) && supplyRails.count( net1 );
            bool bothRef = refRails.count( net0 ) && refRails.count( net1 );

            if( !bothSupply && !bothRef )
                continue;

            PDN_SERIES_ELEMENT elem;
            elem.refdes = refStr;
            elem.type = isResistor ? PDN_SERIES_TYPE::RESISTOR : PDN_SERIES_TYPE::FERRITE_BEAD;
            elem.resistance = 0.0;
            elem.railA = net0;
            elem.railB = net1;
            elem.sheetPath = sheetPath;

            // Collect network keys affected by this series element for warnings
            std::set<wxString> warnKeys;

            for( const wxString& k : railToKeys[net0] )
                warnKeys.insert( k );

            for( const wxString& k : railToKeys[net1] )
                warnKeys.insert( k );

            if( elem.type == PDN_SERIES_TYPE::RESISTOR )
            {
                wxString valueStr = ref.GetValue();

                try
                {
                    elem.resistance = SPICE_VALUE( valueStr ).ToDouble();

                    if( elem.resistance <= 0.0 )
                    {
                        wxString w =
                                wxString::Format( _( "Skipping series R %s — invalid value '%s'." ), refStr, valueStr );

                        for( const wxString& k : warnKeys )
                            addWarning( k, w );

                        continue;
                    }
                }
                catch( ... )
                {
                    wxString w = wxString::Format( _( "Skipping series R %s — unable to parse value '%s'." ), refStr,
                                                   valueStr );

                    for( const wxString& k : warnKeys )
                        addWarning( k, w );

                    continue;
                }
            }
            else
            {
                // Ferrite bead — read Sim.Library, Sim.Name, and MPN fields
                for( const SCH_FIELD& field : symbol->GetFields() )
                {
                    wxString name = field.GetName();

                    if( name == wxT( "Sim.Library" ) )
                        elem.spiceLibFile = field.GetShownText( &sheetPath, false );
                    else if( name == wxT( "Sim.Name" ) )
                        elem.spiceModel = field.GetShownText( &sheetPath, false );
                    else if( name == wxT( "Spice_Lib_File" ) && elem.spiceLibFile.IsEmpty() )
                        elem.spiceLibFile = field.GetShownText( &sheetPath, false );
                    else if( name == wxT( "Spice_Model" ) && elem.spiceModel.IsEmpty() )
                        elem.spiceModel = field.GetShownText( &sheetPath, false );

                    // Read MPN from common field names
                    if( elem.mpn.IsEmpty() )
                    {
                        wxString nameUpper = name.Upper();

                        if( nameUpper == wxT( "MPN" ) || nameUpper == wxT( "MANUFACTURER PART NUMBER" )
                            || nameUpper == wxT( "PART NUMBER" ) )
                        {
                            wxString val = field.GetShownText( &sheetPath, false );
                            val.Trim( true ).Trim( false );

                            if( !val.IsEmpty() )
                                elem.mpn = val;
                        }
                    }
                }

                // Fall back to Value field for MPN (FBs often use the part number as value)
                if( elem.mpn.IsEmpty() )
                {
                    wxString valueStr = ref.GetValue();
                    valueStr.Trim( true ).Trim( false );

                    if( !valueStr.IsEmpty() && m_downloader && m_downloader->HasSource( valueStr ) )
                        elem.mpn = valueStr;
                }

                // If no Sim fields, try auto-fetching from local cache via MPN
                if( ( elem.spiceModel.IsEmpty() || elem.spiceLibFile.IsEmpty() ) && !elem.mpn.IsEmpty()
                    && m_downloader )
                {
                    if( m_downloader->HasCachedModel( elem.mpn ) )
                    {
                        SPICE_MODEL_RESULT result = m_downloader->ReadCachedModel( elem.mpn );

                        if( result.success )
                        {
                            elem.spiceText = result.spiceText;
                            elem.spiceModel = result.subcktName;
                        }
                    }
                }

                if( elem.spiceModel.IsEmpty() || ( elem.spiceLibFile.IsEmpty() && elem.spiceText.IsEmpty() ) )
                {
                    wxString w = wxString::Format( _( "Ferrite bead %s (%s — %s) has no SPICE model "
                                                      "assigned (Sim.Library / Sim.Name) — modeled as "
                                                      "1 kΩ resistor." ),
                                                   refStr, net0, net1 );

                    for( const wxString& k : warnKeys )
                        addWarning( k, w );
                }
            }

            m_seriesElements.push_back( elem );
        }
    }

    // IC-centric SMPS detection for networks that still lack a VRM.
    //
    // Instead of BFS from the supply rail, we start from candidate ICs and
    // verify the SMPS topology: an IC that (1) has a foreign power input
    // (VIN != VOUT), (2) connects through an inductor to the supply rail
    // (SW -> L -> VOUT), and (3) has a feedback divider from the supply
    // rail to the reference rail through its FB pin.
    bool needsFeedbackScan = false;

    for( const auto& [k, net] : m_networks )
    {
        if( !net.vrm.has_value() )
        {
            needsFeedbackScan = true;
            break;
        }
    }

    if( !needsFeedbackScan )
        return;

    // Build resistor and inductor adjacency maps: net -> set of neighbor nets
    std::map<wxString, std::set<wxString>> resistorAdj;
    std::map<wxString, std::set<wxString>> inductorAdj;

    static wxRegEx resistorRefRegex( wxS( "^R[0-9?]" ) );
    static wxRegEx inductorRefRegex( wxS( "^L[0-9?]" ) );

    struct VRM_CANDIDATE
    {
        wxString              ref;
        SCH_SHEET_PATH        sheet;
        SCH_SYMBOL*           symbol;
        std::set<wxString>    powerInNets; // nets with PT_POWER_IN
        std::vector<wxString> signalNets;  // nets with non-power pins (FB, etc.)
        std::vector<wxString> allPinNets;  // all pin nets (for inductor check)
    };

    std::vector<VRM_CANDIDATE> candidates;

    for( const SCH_SHEET_PATH& sheet : sheets )
    {
        for( SCH_ITEM* item : sheet.LastScreen()->Items() )
        {
            if( item->Type() != SCH_SYMBOL_T )
                continue;

            SCH_SYMBOL* symbol = static_cast<SCH_SYMBOL*>( item );

            if( symbol->IsPower() )
                continue;

            wxString              refStr = symbol->GetRef( &sheet, false );
            std::vector<SCH_PIN*> pins = symbol->GetPins( &sheet );

            // Build resistor adjacency
            if( resistorRefRegex.Matches( refStr ) && pins.size() == 2 )
            {
                wxString net0, net1;

                if( SCH_CONNECTION* conn = pins[0]->Connection( &sheet ) )
                    net0 = conn->Name();

                if( SCH_CONNECTION* conn = pins[1]->Connection( &sheet ) )
                    net1 = conn->Name();

                if( !net0.IsEmpty() && !net1.IsEmpty() && net0 != net1 )
                {
                    resistorAdj[net0].insert( net1 );
                    resistorAdj[net1].insert( net0 );
                }

                continue;
            }

            // Build inductor adjacency
            if( inductorRefRegex.Matches( refStr ) && pins.size() == 2 )
            {
                wxString net0, net1;

                if( SCH_CONNECTION* conn = pins[0]->Connection( &sheet ) )
                    net0 = conn->Name();

                if( SCH_CONNECTION* conn = pins[1]->Connection( &sheet ) )
                    net1 = conn->Name();

                if( !net0.IsEmpty() && !net1.IsEmpty() && net0 != net1 )
                {
                    inductorAdj[net0].insert( net1 );
                    inductorAdj[net1].insert( net0 );
                }

                continue;
            }

            // Skip caps
            if( capRefRegex.Matches( refStr ) )
                continue;

            // Must have PT_POWER_IN somewhere to be a potential regulator
            bool hasPowerIn = false;

            for( SCH_PIN* pin : pins )
            {
                if( pin->GetType() == ELECTRICAL_PINTYPE::PT_POWER_IN )
                {
                    hasPowerIn = true;
                    break;
                }
            }

            if( !hasPowerIn )
                continue;

            // Check Pwr.Type = SWITCH
            bool isSwitch = false;

            for( const SCH_FIELD& field : symbol->GetFields() )
            {
                if( field.GetName() == wxT( "Pwr.Type" ) )
                {
                    wxString typeVal = field.GetText();
                    typeVal.Trim( true ).Trim( false );
                    typeVal.MakeUpper();

                    if( typeVal == wxT( "SWITCH" ) )
                        isSwitch = true;

                    break;
                }
            }

            if( isSwitch )
                continue;

            // Collect pin net information for this candidate
            VRM_CANDIDATE cand;
            cand.ref = refStr;
            cand.sheet = sheet;
            cand.symbol = symbol;

            for( SCH_PIN* pin : pins )
            {
                SCH_CONNECTION* conn = pin->Connection( &sheet );

                if( !conn )
                    continue;

                wxString netName = conn->Name();
                cand.allPinNets.push_back( netName );

                if( pin->GetType() == ELECTRICAL_PINTYPE::PT_POWER_IN )
                    cand.powerInNets.insert( netName );
                else if( pin->GetType() != ELECTRICAL_PINTYPE::PT_POWER_OUT )
                    cand.signalNets.push_back( netName );
            }

            if( !cand.powerInNets.empty() )
                candidates.push_back( std::move( cand ) );
        }
    }

    // Helper: can net reach target through 1-2 resistor hops?
    auto reachesThroughResistors = [&resistorAdj]( const wxString& aNet, const wxString& aTarget,
                                                   const wxString& aExclude, int aMaxHops ) -> bool
    {
        auto adjIt = resistorAdj.find( aNet );

        if( adjIt == resistorAdj.end() )
            return false;

        // 1 hop
        if( adjIt->second.count( aTarget ) > 0 )
            return true;

        if( aMaxHops < 2 )
            return false;

        // 2 hops
        for( const wxString& mid : adjIt->second )
        {
            if( mid == aExclude )
                continue;

            auto adjIt2 = resistorAdj.find( mid );

            if( adjIt2 != resistorAdj.end() && adjIt2->second.count( aTarget ) > 0 )
                return true;
        }

        return false;
    };

    // For each network without a VRM, check each candidate IC
    for( auto& [key, network] : m_networks )
    {
        if( network.vrm.has_value() )
            continue;

        for( const VRM_CANDIDATE& cand : candidates )
        {
            // Condition 1: IC must have PT_POWER_IN on a foreign rail
            // (not the supply rail and not the reference rail).
            std::set<wxString> foreignRails = cand.powerInNets;
            foreignRails.erase( network.supplyRail );
            foreignRails.erase( network.refRail );

            if( foreignRails.empty() )
                continue;

            // Condition 2: IC must connect through an inductor to the supply rail.
            // Check each pin's net for inductor adjacency to the supply rail.
            bool hasInductorToSupply = false;

            for( const wxString& pinNet : cand.allPinNets )
            {
                if( pinNet == network.supplyRail )
                    continue; // direct connection, not through inductor

                auto indIt = inductorAdj.find( pinNet );

                if( indIt != inductorAdj.end() && indIt->second.count( network.supplyRail ) > 0 )
                {
                    hasInductorToSupply = true;
                    break;
                }
            }

            if( !hasInductorToSupply )
                continue;

            // Condition 3: IC must have a non-power pin on a feedback divider
            // junction — a net reachable through resistors from both the supply
            // rail (1-3 hops) and the reference rail (1-2 hops).
            bool hasFeedbackDivider = false;

            for( const wxString& fbNet : cand.signalNets )
            {
                if( fbNet == network.supplyRail || fbNet == network.refRail )
                    continue;

                bool reachesSupply = reachesThroughResistors( fbNet, network.supplyRail, network.refRail, 2 );
                bool reachesRef = reachesThroughResistors( fbNet, network.refRail, network.supplyRail, 2 );

                if( reachesSupply && reachesRef )
                {
                    hasFeedbackDivider = true;
                    break;
                }
            }

            if( !hasFeedbackDivider )
                continue;

            // All three conditions met — this IC is the VRM
            PDN_VRM vrm;
            vrm.refdes = cand.ref;
            vrm.sheetPath = cand.sheet;
            vrm.params = readVrmParams( cand.symbol, cand.sheet, key,
                                        [this]( const wxString& k, const wxString& w )
                                        {
                                            addWarning( k, w );
                                        } );

            if( network.vrm.has_value() )
            {
                addWarning( key, wxString::Format( _( "Multiple regulators on '%s': using %s, "
                                                      "ignoring %s." ),
                                                   network.supplyRail, network.vrm->refdes, cand.ref ) );
            }
            else
            {
                network.vrm = vrm;
            }
        }
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

    for( const PDN_SERIES_ELEMENT& elem : m_seriesElements )
    {
        if( elem.refdes == aRefdes )
        {
            // Return the network whose supply rail matches one of the element's rails
            for( const auto& [key, network] : m_networks )
            {
                if( network.supplyRail == elem.railA || network.supplyRail == elem.railB )
                    return key;
            }
        }
    }

    return wxString();
}


bool PDN_ANALYZER::NetworkHasSymsOnSheet( const wxString& aNetworkKey, const SCH_SHEET_PATH& aSheet ) const
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


std::vector<wxString> PDN_ANALYZER::GetCoupledNetworkKeys( const wxString& aNetworkKey ) const
{
    std::vector<wxString> result;
    result.push_back( aNetworkKey );

    auto primaryIt = m_networks.find( aNetworkKey );

    if( primaryIt == m_networks.end() )
        return result;

    // Build rail adjacency graph from all series elements.  Ferrite beads
    // without a SPICE model are still included so that the network coupling
    // topology is detected; they default to a high impedance in the netlist.
    std::map<wxString, std::set<wxString>> railAdj;

    for( const PDN_SERIES_ELEMENT& elem : m_seriesElements )
    {
        railAdj[elem.railA].insert( elem.railB );
        railAdj[elem.railB].insert( elem.railA );
    }

    if( railAdj.empty() )
        return result;

    // BFS helper: check if two rails are connected through series elements.
    auto railsConnected = [&railAdj]( const wxString& aFrom, const wxString& aTo ) -> bool
    {
        if( aFrom == aTo )
            return true;

        std::set<wxString>   visited;
        std::queue<wxString> bfs;

        bfs.push( aFrom );
        visited.insert( aFrom );

        while( !bfs.empty() )
        {
            wxString cur = bfs.front();
            bfs.pop();

            auto adjIt = railAdj.find( cur );

            if( adjIt == railAdj.end() )
                continue;

            for( const wxString& neighbor : adjIt->second )
            {
                if( neighbor == aTo )
                    return true;

                if( visited.insert( neighbor ).second )
                    bfs.push( neighbor );
            }
        }

        return false;
    };

    // A network is coupled to the primary if both its supply rail and its
    // reference rail are reachable from the primary's respective rails.
    const wxString& primarySupply = primaryIt->second.supplyRail;
    const wxString& primaryRef = primaryIt->second.refRail;

    for( const auto& [key, network] : m_networks )
    {
        if( key == aNetworkKey )
            continue;

        if( railsConnected( primarySupply, network.supplyRail ) && railsConnected( primaryRef, network.refRail ) )
        {
            result.push_back( key );
        }
    }

    return result;
}


wxString PDN_ANALYZER::BuildSpiceNetlist( const PDN_NETWORK& aNetwork, const SCH_SHEET_PATH& aObservationSheet,
                                          double aTraceInductance, double aTraceResistance, double aStartFreq,
                                          double aEndFreq, int aPointsPerDecade )
{
    m_probeNodes.clear();

    wxString obsKey = aObservationSheet.PathAsString();

    std::map<wxString, std::vector<const PDN_CAPACITOR*>> sheetGroups;

    for( const PDN_CAPACITOR& cap : aNetwork.components )
        sheetGroups[cap.sheetPath.PathAsString()].push_back( &cap );

    wxString netlist;

    netlist +=
            wxString::Format( wxS( "* PDN Impedance Analysis: %s / %s\n\n" ), aNetwork.supplyRail, aNetwork.refRail );

    netlist += wxS( ".options KLU\n\n" );

    // Generic parameterized subcircuit — ESR and ESL are per-instance
    netlist += wxS( ".subckt CAP p n c=100n esr=0.01 esl=400p\n" );
    netlist += wxS( "C1 p 1 {c}\n" );
    netlist += wxS( "R1 1 2 {esr}\n" );
    netlist += wxS( "L1 2 n {esl}\n" );
    netlist += wxS( ".ends\n\n" );

    netlist += wxS( "I1 local gnd AC 1\n" );

    int idx = 1;
    int groupIdx = 1;

    // Local caps (on observation sheet) connect directly to the measurement node.
    auto localIt = sheetGroups.find( obsKey );

    if( localIt != sheetGroups.end() )
    {
        for( const PDN_CAPACITOR* cap : localIt->second )
        {
            netlist += wxString::Format( wxS( "X%d local gnd CAP c=%g esr=%g esl=%g\n" ), idx, cap->capacitance,
                                         cap->esr, cap->esl );
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

        netlist += wxString::Format( wxS( "L_s%d local %s %g\n" ), groupIdx, nodeL, aTraceInductance );
        netlist += wxString::Format( wxS( "R_s%d %s %s %g\n" ), groupIdx, nodeL, nodeR, aTraceResistance );

        for( const PDN_CAPACITOR* cap : caps )
        {
            netlist += wxString::Format( wxS( "X%d %s gnd CAP c=%g esr=%g esl=%g\n" ), idx, nodeR, cap->capacitance,
                                         cap->esr, cap->esl );
            idx++;
        }

        // Record probe node for this remote sheet group
        SCH_SHEET* sheet = caps[0]->sheetPath.Last();
        wxString   sheetLabel = sheet ? sheet->GetName() : wxString::Format( _( "Group %d" ), groupIdx );
        m_probeNodes.push_back( { sheetLabel, nodeR } );

        groupIdx++;
    }

    // VRM source impedance — only if a regulator was found on the supply rail
    if( aNetwork.vrm.has_value() )
    {
        const PDN_VRM& vrm = *aNetwork.vrm;
        double         lVrm = vrm.params.rDamp / ( 2.0 * M_PI * vrm.params.bandwidth );
        double         lDamp = lVrm / 10.0;
        bool           vrmLocal = ( vrm.sheetPath.PathAsString() == obsKey );

        wxString vrmConn;

        if( vrmLocal )
        {
            vrmConn = wxS( "local" );
        }
        else
        {
            wxString vrmNodeL = wxString::Format( wxS( "rs%d" ), groupIdx );
            wxString vrmNodeR = wxString::Format( wxS( "rs%db" ), groupIdx );

            netlist += wxString::Format( wxS( "\n* VRM trace (%s, remote)\n" ), vrm.refdes );
            netlist += wxString::Format( wxS( "L_s%d local %s %g\n" ), groupIdx, vrmNodeL, aTraceInductance );
            netlist += wxString::Format( wxS( "R_s%d %s %s %g\n" ), groupIdx, vrmNodeL, vrmNodeR, aTraceResistance );
            vrmConn = vrmNodeR;

            m_probeNodes.push_back( { wxString::Format( _( "VRM (%s)" ), vrm.refdes ), vrmNodeR } );

            groupIdx++;
        }

        netlist += wxString::Format( wxS( "\n* VRM source impedance (%s)\n" ), vrm.refdes );
        netlist += wxString::Format( wxS( "R_vrm %s vrm1 %g\n" ), vrmConn, vrm.params.rVrm );
        netlist += wxString::Format( wxS( "L_vrm vrm1 gnd %g\n" ), lVrm );
        netlist += wxString::Format( wxS( "R_damp %s vrm2 %g\n" ), vrmConn, vrm.params.rDamp );
        netlist += wxString::Format( wxS( "L_damp vrm2 gnd %g\n" ), lDamp );
    }

    netlist += wxString::Format( wxS( "\n.ac dec %d %g %g\n" ), aPointsPerDecade, aStartFreq, aEndFreq );
    netlist += wxS( ".end\n" );

    return netlist;
}


wxString PDN_ANALYZER::BuildSpiceNetlist( const wxString& aNetworkKey, const SCH_SHEET_PATH& aObservationSheet,
                                          double aTraceInductance, double aTraceResistance, double aStartFreq,
                                          double aEndFreq, int aPointsPerDecade )
{
    m_probeNodes.clear();

    auto primaryIt = m_networks.find( aNetworkKey );

    if( primaryIt == m_networks.end() )
        return wxString();

    std::vector<wxString> coupledKeys = GetCoupledNetworkKeys( aNetworkKey );

    // No coupling — delegate to the single-network overload
    if( coupledKeys.size() <= 1 )
    {
        return BuildSpiceNetlist( primaryIt->second, aObservationSheet, aTraceInductance, aTraceResistance, aStartFreq,
                                  aEndFreq, aPointsPerDecade );
    }

    wxString obsKey = aObservationSheet.PathAsString();
    wxString netlist;

    // Header
    netlist += wxString::Format( wxS( "* PDN Impedance Analysis: %s\n" ), aNetworkKey );

    for( const wxString& key : coupledKeys )
    {
        if( key != aNetworkKey )
            netlist += wxString::Format( wxS( "* Coupled: %s\n" ), key );
    }

    netlist += wxS( "\n.options KLU\n\n" );

    // Collect .include directives for ferrite bead SPICE libraries
    std::set<wxString> includedLibs;
    std::set<wxString> inlinedSubckts;

    for( const PDN_SERIES_ELEMENT& elem : m_seriesElements )
    {
        if( elem.type == PDN_SERIES_TYPE::FERRITE_BEAD && !elem.spiceModel.IsEmpty() )
        {
            if( !elem.spiceText.IsEmpty() )
                inlinedSubckts.insert( elem.spiceModel );
            else if( !elem.spiceLibFile.IsEmpty() )
                includedLibs.insert( elem.spiceLibFile );
        }
    }

    for( const wxString& lib : includedLibs )
        netlist += wxString::Format( wxS( ".include \"%s\"\n" ), lib );

    if( !includedLibs.empty() )
        netlist += wxS( "\n" );

    // Inline auto-fetched subcircuit definitions
    for( const PDN_SERIES_ELEMENT& elem : m_seriesElements )
    {
        if( !elem.spiceText.IsEmpty() && inlinedSubckts.count( elem.spiceModel ) > 0 )
        {
            netlist += wxString::Format( wxS( "* Auto-fetched model: %s (%s)\n" ), elem.refdes, elem.mpn );
            netlist += elem.spiceText;

            if( !elem.spiceText.EndsWith( wxS( "\n" ) ) )
                netlist += wxS( "\n" );

            netlist += wxS( "\n" );
            inlinedSubckts.erase( elem.spiceModel );
        }
    }

    // CAP subcircuit
    netlist += wxS( ".subckt CAP p n c=100n esr=0.01 esl=400p\n" );
    netlist += wxS( "C1 p 1 {c}\n" );
    netlist += wxS( "R1 1 2 {esr}\n" );
    netlist += wxS( "L1 2 n {esl}\n" );
    netlist += wxS( ".ends\n\n" );

    // Map each unique power rail to a SPICE node name.
    // Primary supply = "local", primary ref = "gnd", others = "rail_N".
    std::map<wxString, wxString> railToNode;

    const PDN_NETWORK& primaryNetwork = primaryIt->second;
    railToNode[primaryNetwork.supplyRail] = wxS( "local" );
    railToNode[primaryNetwork.refRail] = wxS( "gnd" );

    int railIdx = 1;

    for( const wxString& key : coupledKeys )
    {
        if( key == aNetworkKey )
            continue;

        const PDN_NETWORK& net = m_networks.at( key );

        if( railToNode.find( net.supplyRail ) == railToNode.end() )
            railToNode[net.supplyRail] = wxString::Format( wxS( "rail_%d" ), railIdx++ );

        if( railToNode.find( net.refRail ) == railToNode.end() )
            railToNode[net.refRail] = wxString::Format( wxS( "rail_%d" ), railIdx++ );
    }

    // AC current source on primary network measurement node
    netlist += wxS( "I1 local gnd AC 1\n" );

    int idx = 1;
    int groupIdx = 1;
    int vrmIdx = 1;

    // Emit caps and VRMs for each network
    for( const wxString& netKey : coupledKeys )
    {
        const PDN_NETWORK& network = m_networks.at( netKey );
        wxString           supplyNode = railToNode[network.supplyRail];
        wxString           refNode = railToNode[network.refRail];
        bool               isPrimary = ( netKey == aNetworkKey );

        netlist += wxString::Format( wxS( "\n* Network: %s\n" ), netKey );

        if( isPrimary )
        {
            // Primary network: local/remote sheet grouping
            std::map<wxString, std::vector<const PDN_CAPACITOR*>> sheetGroups;

            for( const PDN_CAPACITOR& cap : network.components )
                sheetGroups[cap.sheetPath.PathAsString()].push_back( &cap );

            auto localIt = sheetGroups.find( obsKey );

            if( localIt != sheetGroups.end() )
            {
                for( const PDN_CAPACITOR* cap : localIt->second )
                {
                    netlist += wxString::Format( wxS( "X%d %s %s CAP c=%g esr=%g esl=%g\n" ), idx, supplyNode, refNode,
                                                 cap->capacitance, cap->esr, cap->esl );
                    idx++;
                }
            }

            for( const auto& [sheetKey, caps] : sheetGroups )
            {
                if( sheetKey == obsKey )
                    continue;

                wxString nodeL = wxString::Format( wxS( "rs%d" ), groupIdx );
                wxString nodeR = wxString::Format( wxS( "rs%db" ), groupIdx );

                netlist += wxString::Format( wxS( "L_s%d %s %s %g\n" ), groupIdx, supplyNode, nodeL, aTraceInductance );
                netlist += wxString::Format( wxS( "R_s%d %s %s %g\n" ), groupIdx, nodeL, nodeR, aTraceResistance );

                for( const PDN_CAPACITOR* cap : caps )
                {
                    netlist += wxString::Format( wxS( "X%d %s %s CAP c=%g esr=%g esl=%g\n" ), idx, nodeR, refNode,
                                                 cap->capacitance, cap->esr, cap->esl );
                    idx++;
                }

                // Record probe node for this remote sheet group
                SCH_SHEET* sheet = caps[0]->sheetPath.Last();
                wxString   sheetLabel = sheet ? sheet->GetName() : wxString::Format( _( "Group %d" ), groupIdx );
                m_probeNodes.push_back( { sheetLabel, nodeR } );

                groupIdx++;
            }
        }
        else
        {
            // Coupled network: all caps between supply and ref nodes
            for( const PDN_CAPACITOR& cap : network.components )
            {
                netlist += wxString::Format( wxS( "X%d %s %s CAP c=%g esr=%g esl=%g\n" ), idx, supplyNode, refNode,
                                             cap.capacitance, cap.esr, cap.esl );
                idx++;
            }
        }

        // VRM on this network
        if( network.vrm.has_value() )
        {
            const PDN_VRM& vrm = *network.vrm;
            double         lVrm = vrm.params.rDamp / ( 2.0 * M_PI * vrm.params.bandwidth );
            double         lDamp = lVrm / 10.0;

            wxString vrmConn;

            if( isPrimary && vrm.sheetPath.PathAsString() != obsKey )
            {
                wxString vrmNodeL = wxString::Format( wxS( "rs%d" ), groupIdx );
                wxString vrmNodeR = wxString::Format( wxS( "rs%db" ), groupIdx );

                netlist += wxString::Format( wxS( "\n* VRM trace (%s, remote)\n" ), vrm.refdes );
                netlist +=
                        wxString::Format( wxS( "L_s%d %s %s %g\n" ), groupIdx, supplyNode, vrmNodeL, aTraceInductance );
                netlist +=
                        wxString::Format( wxS( "R_s%d %s %s %g\n" ), groupIdx, vrmNodeL, vrmNodeR, aTraceResistance );
                vrmConn = vrmNodeR;

                m_probeNodes.push_back( { wxString::Format( _( "VRM (%s)" ), vrm.refdes ), vrmNodeR } );

                groupIdx++;
            }
            else
            {
                vrmConn = supplyNode;
            }

            wxString va = wxString::Format( wxS( "v%da" ), vrmIdx );
            wxString vb = wxString::Format( wxS( "v%db" ), vrmIdx );

            netlist += wxString::Format( wxS( "\n* VRM source impedance (%s)\n" ), vrm.refdes );
            netlist += wxString::Format( wxS( "R_v%d %s %s %g\n" ), vrmIdx, vrmConn, va, vrm.params.rVrm );
            netlist += wxString::Format( wxS( "L_v%d %s %s %g\n" ), vrmIdx, va, refNode, lVrm );
            netlist += wxString::Format( wxS( "R_d%d %s %s %g\n" ), vrmIdx, vrmConn, vb, vrm.params.rDamp );
            netlist += wxString::Format( wxS( "L_d%d %s %s %g\n" ), vrmIdx, vb, refNode, lDamp );
            vrmIdx++;
        }

        // Record probe node for coupled network supply rails
        if( !isPrimary )
        {
            wxString label;

            for( const PDN_SERIES_ELEMENT& elem : m_seriesElements )
            {
                if( ( elem.railA == network.supplyRail || elem.railB == network.supplyRail )
                    && ( elem.railA == primaryNetwork.supplyRail || elem.railB == primaryNetwork.supplyRail ) )
                {
                    label = wxString::Format( wxS( "%s (via %s)" ), network.supplyRail, elem.refdes );
                    break;
                }
            }

            if( label.IsEmpty() )
                label = network.supplyRail;

            m_probeNodes.push_back( { label, supplyNode } );
        }
    }

    // Emit series elements connecting rails
    int seIdx = 1;

    for( const PDN_SERIES_ELEMENT& elem : m_seriesElements )
    {
        auto itA = railToNode.find( elem.railA );
        auto itB = railToNode.find( elem.railB );

        if( itA == railToNode.end() || itB == railToNode.end() )
            continue;

        if( elem.type == PDN_SERIES_TYPE::RESISTOR )
        {
            netlist += wxString::Format( wxS( "\n* Series R: %s (%s <-> %s)\n" ), elem.refdes, elem.railA, elem.railB );
            netlist += wxString::Format( wxS( "R_se%d %s %s %g\n" ), seIdx, itA->second, itB->second, elem.resistance );
        }
        else if( !elem.spiceModel.IsEmpty() && ( !elem.spiceLibFile.IsEmpty() || !elem.spiceText.IsEmpty() ) )
        {
            netlist += wxString::Format( wxS( "\n* Ferrite bead: %s (%s <-> %s)\n" ), elem.refdes, elem.railA,
                                         elem.railB );
            netlist += wxString::Format( wxS( "X_se%d %s %s %s\n" ), seIdx, itA->second, itB->second, elem.spiceModel );
        }
        else
        {
            // Ferrite bead without SPICE model — high impedance (conservative)
            netlist += wxString::Format( wxS( "\n* Ferrite bead (no model): %s (%s <-> %s)\n" ), elem.refdes,
                                         elem.railA, elem.railB );
            netlist += wxString::Format( wxS( "R_se%d %s %s 1k\n" ), seIdx, itA->second, itB->second );
        }

        seIdx++;
    }

    netlist += wxString::Format( wxS( "\n.ac dec %d %g %g\n" ), aPointsPerDecade, aStartFreq, aEndFreq );
    netlist += wxS( ".end\n" );

    return netlist;
}


bool PDN_ANALYZER::RunAnalysis( const PDN_NETWORK& aNetwork, const SCH_SHEET_PATH& aObservationSheet,
                                double aTraceInductance, double aTraceResistance )
{
    LOCALE_IO dummy;

    m_frequencies.clear();
    m_impedance.clear();
    m_transferImpedance.clear();

    wxString key = aNetwork.supplyRail + wxS( " / " ) + aNetwork.refRail;

    if( aNetwork.components.empty() )
    {
        addWarning( key, wxString::Format( _( "Network '%s' has no capacitors." ), aNetwork.supplyRail ) );
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

    // Suppress the reporter — our own sim instance doesn't need one, and
    // forwarding thousands of output lines is slow.
    sim->SetReporter( nullptr );

    sim->Command( "destroy all" );
    sim->Init();
    wxString netlist = BuildSpiceNetlist( aNetwork, aObservationSheet, aTraceInductance, aTraceResistance );

    if( !sim->LoadNetlist( netlist.ToStdString() ) )
    {
        addWarning( key, wxString::Format( _( "Failed to load netlist for '%s'." ), aNetwork.supplyRail ) );

        return false;
    }

    if( !sim->Command( "run" ) )
    {
        addWarning( key, wxString::Format( _( "Simulation failed for '%s'." ), aNetwork.supplyRail ) );

        return false;
    }

    m_frequencies = sim->GetGainVector( "frequency" );
    m_impedance = sim->GetGainVector( "v(local)" );

    for( const PDN_PROBE_NODE& probe : m_probeNodes )
    {
        std::string         vectorName = "v(" + probe.nodeName.ToStdString() + ")";
        std::vector<double> data = sim->GetGainVector( vectorName );

        if( data.empty() || data.size() != m_frequencies.size() )
            continue;

        if( m_transferImpedance.empty() )
        {
            m_transferImpedance = data;
        }
        else
        {
            for( size_t i = 0; i < data.size(); i++ )
                m_transferImpedance[i] = std::max( m_transferImpedance[i], data[i] );
        }
    }

    // Reporter was already set to nullptr on our own sim instance.

    if( m_frequencies.empty() || m_impedance.empty() )
    {
        addWarning( key, wxString::Format( _( "No results returned for '%s'." ), aNetwork.supplyRail ) );
        return false;
    }

    return true;
}


bool PDN_ANALYZER::RunAnalysis( const wxString& aNetworkKey, const SCH_SHEET_PATH& aObservationSheet,
                                double aTraceInductance, double aTraceResistance )
{
    LOCALE_IO dummy;

    m_frequencies.clear();
    m_impedance.clear();
    m_transferImpedance.clear();

    auto it = m_networks.find( aNetworkKey );

    if( it == m_networks.end() )
    {
        addWarning( aNetworkKey, wxString::Format( _( "Network '%s' not found." ), aNetworkKey ) );
        return false;
    }

    const PDN_NETWORK& network = it->second;

    if( network.components.empty() )
    {
        addWarning( aNetworkKey, wxString::Format( _( "Network '%s' has no capacitors." ), network.supplyRail ) );
        return false;
    }

    std::shared_ptr<SPICE_SIMULATOR> sim = SIMULATOR::CreateInstance( "ngspice" );

    if( !sim )
    {
        addWarning( aNetworkKey, _( "Failed to create ngspice simulator instance." ) );
        return false;
    }

    if( !sim->Settings() )
        sim->Settings() = std::make_shared<NGSPICE_SETTINGS>( nullptr, "" );

    sim->SetReporter( nullptr );

    sim->Command( "destroy all" );
    sim->Init();
    wxString netlist = BuildSpiceNetlist( aNetworkKey, aObservationSheet, aTraceInductance, aTraceResistance );

    if( !sim->LoadNetlist( netlist.ToStdString() ) )
    {
        addWarning( aNetworkKey, wxString::Format( _( "Failed to load netlist for '%s'." ), network.supplyRail ) );

        return false;
    }

    if( !sim->Command( "run" ) )
    {
        addWarning( aNetworkKey, wxString::Format( _( "Simulation failed for '%s'." ), network.supplyRail ) );

        return false;
    }

    m_frequencies = sim->GetGainVector( "frequency" );
    m_impedance = sim->GetGainVector( "v(local)" );

    for( const PDN_PROBE_NODE& probe : m_probeNodes )
    {
        std::string         vectorName = "v(" + probe.nodeName.ToStdString() + ")";
        std::vector<double> data = sim->GetGainVector( vectorName );

        if( data.empty() || data.size() != m_frequencies.size() )
            continue;

        if( m_transferImpedance.empty() )
        {
            m_transferImpedance = data;
        }
        else
        {
            for( size_t i = 0; i < data.size(); i++ )
                m_transferImpedance[i] = std::max( m_transferImpedance[i], data[i] );
        }
    }

    // Reporter was already set to nullptr on our own sim instance.

    if( m_frequencies.empty() || m_impedance.empty() )
    {
        addWarning( aNetworkKey, wxString::Format( _( "No results returned for '%s'." ), network.supplyRail ) );
        return false;
    }

    return true;
}
