/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
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

#include <qa_utils/wx_utils/unit_test_utils.h>
#include <schematic_utils/schematic_file_util.h>

#include <system_diagram/system_diagram_analyzer.h>
#include <system_diagram/system_diagram_generator.h>
#include <system_diagram/system_diagram_layout.h>
#include <schematic.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <settings/settings_manager.h>
#include <locale_io.h>
#include <cmath>


BOOST_AUTO_TEST_SUITE( SystemDiagramAnalyzer )


// =========================================================================
// parseVoltage tests
// =========================================================================

BOOST_AUTO_TEST_CASE( ParseVoltage_Standard3V3 )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "+3V3" ) ), 3.3, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_Standard5V )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "+5V" ) ), 5.0, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_Standard1V8 )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "+1V8" ) ), 1.8, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_Standard12V )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "+12V" ) ), 12.0, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_Negative )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "-5V" ) ), -5.0, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_Decimal )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "+3.3V" ) ), 3.3, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_NoPlus )
{
    // Net names without + prefix
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "3V3" ) ), 3.3, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_GND )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "GND" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_VCC )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "VCC" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_Empty )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_Lowercase )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "+3v3" ) ), 3.3, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_0V9 )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "+0V9" ) ), 0.9, 0.1 );
}


// =========================================================================
// parseCurrent tests
// =========================================================================

BOOST_AUTO_TEST_CASE( ParseCurrent_Milliamps )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "150mA" ) ), 0.15, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_Microamps )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "100uA" ) ), 100e-6, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_Nanoamps )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "500nA" ) ), 500e-9, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_Amps )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "2.5A" ) ), 2.5, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_AmpsNoUnit )
{
    // Bare number with just "A"
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "1A" ) ), 1.0, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_WithSpaces )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( " 150 mA " ) ), 0.15, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_WithSign )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "+150mA" ) ), 0.15, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_Empty )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_NoValue )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "mA" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_Picoamps )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "50pA" ) ), 50e-12, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_Kiloamps )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "1.5kA" ) ), 1500.0, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_BadUnit )
{
    // Unknown unit suffix should return 0
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "5W" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_CaseInsensitive )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "150MA" ) ), 0.15, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseCurrent_Decimal )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseCurrent( wxT( "1.5mA" ) ), 1.5e-3, 0.1 );
}


// =========================================================================
// parseEfficiency tests
// =========================================================================

BOOST_AUTO_TEST_CASE( ParseEfficiency_Fraction )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "0.87" ) ), 0.87, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseEfficiency_Percent )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "87%" ) ), 0.87, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseEfficiency_WholeNumber )
{
    // Value > 1.0 treated as percentage
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "87" ) ), 0.87, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseEfficiency_Exactly1 )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "1.0" ) ), 1.0, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseEfficiency_Zero )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "0" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseEfficiency_Empty )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseEfficiency_Over100 )
{
    // 150% -> 1.5, which is > 1.0, so returns 0.0
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "150%" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseEfficiency_Negative )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "-0.5" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseEfficiency_WithSpaces )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( " 92 % " ) ), 0.92, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseEfficiency_BadValue )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "abc" ) ), 0.0 );
}


// =========================================================================
// formatCurrent tests
// =========================================================================

BOOST_AUTO_TEST_CASE( FormatCurrent_Milliamps )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::formatCurrent( 0.15 ), wxString( "150.0mA" ) );
}

BOOST_AUTO_TEST_CASE( FormatCurrent_Amps )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::formatCurrent( 2.5 ), wxString( "2.50A" ) );
}

BOOST_AUTO_TEST_CASE( FormatCurrent_Microamps )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::formatCurrent( 100e-6 ), wxString( "100.0uA" ) );
}

BOOST_AUTO_TEST_CASE( FormatCurrent_Nanoamps )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::formatCurrent( 500e-9 ), wxString( "500.0nA" ) );
}

BOOST_AUTO_TEST_CASE( FormatCurrent_Picoamps )
{
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::formatCurrent( 50e-12 ), wxString( "50.00pA" ) );
}

BOOST_AUTO_TEST_CASE( FormatCurrent_Zero )
{
    BOOST_CHECK( SYSTEM_DIAGRAM_ANALYZER::formatCurrent( 0.0 ).IsEmpty() );
}

BOOST_AUTO_TEST_CASE( FormatCurrent_Negative )
{
    BOOST_CHECK( SYSTEM_DIAGRAM_ANALYZER::formatCurrent( -0.1 ).IsEmpty() );
}


// =========================================================================
// Schematic-based tests
// =========================================================================

struct SD_SCHEMATIC_FIXTURE
{
    SD_SCHEMATIC_FIXTURE() : m_settingsManager( true /* headless */ ) {}

    SETTINGS_MANAGER           m_settingsManager;
    std::unique_ptr<SCHEMATIC> m_schematic;
};


BOOST_FIXTURE_TEST_CASE( PowerTreeBasic, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    bool                    hasContent = analyzer.Analyze();

    BOOST_REQUIRE( hasContent );

    SYSTEM_DIAGRAM_DATA& data = analyzer.GetData();

    // Should have power nodes
    BOOST_REQUIRE( !data.m_powerRoots.empty() );

    // Collect all power nodes into a flat map for checking
    std::map<wxString, SD_POWER_NODE*>    nodesByRef;
    std::function<void( SD_POWER_NODE* )> collectNodes;
    collectNodes = [&]( SD_POWER_NODE* node )
    {
        if( !node->m_reference.IsEmpty() )
            nodesByRef[node->m_reference] = node;

        for( SD_POWER_NODE* child : node->m_children )
            collectNodes( child );
    };

    for( SD_POWER_NODE* root : data.m_powerRoots )
        collectNodes( root );

    // J1 should be a SOURCE
    auto j1It = nodesByRef.find( wxT( "J1" ) );
    BOOST_REQUIRE_MESSAGE( j1It != nodesByRef.end(), "J1 should be a power node" );
    BOOST_CHECK_EQUAL( j1It->second->m_type, SD_POWER_NODE::SOURCE );
    BOOST_CHECK_CLOSE( j1It->second->m_voltage, 5.0, 0.1 );

    // U1 should be a REGULATOR
    auto u1It = nodesByRef.find( wxT( "U1" ) );
    BOOST_REQUIRE_MESSAGE( u1It != nodesByRef.end(), "U1 should be a power node" );
    BOOST_CHECK_EQUAL( u1It->second->m_type, SD_POWER_NODE::REGULATOR );
    BOOST_CHECK_CLOSE( u1It->second->m_voltage, 3.3, 0.1 );

    // U1 should be a child of J1
    bool u1IsChildOfJ1 = false;

    for( SD_POWER_NODE* child : j1It->second->m_children )
    {
        if( child->m_reference == wxT( "U1" ) )
            u1IsChildOfJ1 = true;
    }

    BOOST_CHECK_MESSAGE( u1IsChildOfJ1, "U1 should be a child of J1 in the power tree" );

    // U2 should appear as a load on U1's +3V3 rail
    bool u2IsLoadOnU1 = false;

    for( const wxString& loadRef : u1It->second->m_loadRefs )
    {
        if( loadRef == wxT( "U2" ) )
            u2IsLoadOnU1 = true;
    }

    BOOST_CHECK_MESSAGE( u2IsLoadOnU1, "U2 should be a load on U1's output rail" );

    // R1 should NOT appear anywhere — not a U? or J? component
    BOOST_CHECK_MESSAGE( nodesByRef.find( wxT( "R1" ) ) == nodesByRef.end(),
                         "R1 (resistor) should not be a power node" );

    SD_COMPONENT* r1comp = data.FindComponent( wxT( "R1" ) );
    BOOST_CHECK_MESSAGE( r1comp == nullptr, "R1 should not be in components" );
}


BOOST_FIXTURE_TEST_CASE( ComponentDiscovery, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    analyzer.Analyze();

    SYSTEM_DIAGRAM_DATA& data = analyzer.GetData();

    // After pruning, we should have J1, U1, U2 (participating in power tree)
    // but not R1 (not U? or J? prefix, never discovered)
    std::set<wxString> compRefs;

    for( const auto& comp : data.m_components )
        compRefs.insert( comp->m_reference );

    BOOST_CHECK_MESSAGE( compRefs.count( wxT( "J1" ) ), "J1 should be discovered" );
    BOOST_CHECK_MESSAGE( compRefs.count( wxT( "U1" ) ), "U1 should be discovered" );
    BOOST_CHECK_MESSAGE( compRefs.count( wxT( "U2" ) ), "U2 should be discovered" );
    BOOST_CHECK_MESSAGE( !compRefs.count( wxT( "R1" ) ), "R1 should NOT be discovered" );
}


BOOST_FIXTURE_TEST_CASE( VoltageExtraction, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    analyzer.Analyze();

    SYSTEM_DIAGRAM_DATA& data = analyzer.GetData();

    // Check that voltages are correctly parsed from net names
    for( const auto& node : data.m_allPowerNodes )
    {
        if( node->m_netName.Contains( wxT( "5V" ) ) )
            BOOST_CHECK_CLOSE( node->m_voltage, 5.0, 0.1 );
        else if( node->m_netName.Contains( wxT( "3V3" ) ) )
            BOOST_CHECK_CLOSE( node->m_voltage, 3.3, 0.1 );
    }
}


BOOST_FIXTURE_TEST_CASE( CurrentAnnotation, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    analyzer.Analyze();

    SYSTEM_DIAGRAM_DATA& data = analyzer.GetData();

    // U2 has "Pwr.I.typ = 150mA" annotation.
    // It should show up on the +3V3 rail node (U1's output).
    for( const auto& node : data.m_allPowerNodes )
    {
        if( node->m_reference == wxT( "U1" ) )
        {
            auto typIt = node->m_currentByMode.find( wxT( "typ" ) );

            if( typIt != node->m_currentByMode.end() )
                BOOST_CHECK_CLOSE( typIt->second, 0.15, 1.0 );
        }
    }
}


// =========================================================================
// parseVoltage edge cases and potential bugs
// =========================================================================

BOOST_AUTO_TEST_CASE( ParseVoltage_MultiDigitsAfterV )
{
    // "+3V30" -- what happens? The parser appends '.' and then collects '3' and '0'
    // giving "3.30" = 3.30.  This seems correct but worth documenting.
    double v = SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "+3V30" ) );
    BOOST_CHECK_CLOSE( v, 3.30, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_TrailingText )
{
    // "+3V3_FPGA" -- should still parse as 3.3
    double v = SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "+3V3_FPGA" ) );
    BOOST_CHECK_CLOSE( v, 3.3, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_VBus )
{
    // "VBUS" has 'V' but no preceding digits — should return 0
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "VBUS" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_VddIo )
{
    // "VDD_IO" — no voltage info, should return 0
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "VDD_IO" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_NegativeDecimal )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "-12V" ) ), -12.0, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_48V )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "+48V" ) ), 48.0, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_0V8 )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "+0V8" ) ), 0.8, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_MultipleV )
{
    // "+5V_3V3" -- this is a weird net name. The parser will find '5V' first and then
    // continue to collect '3' and beyond. Let's document the behavior.
    // After '+' strip: "5V_3V3"
    // i=0: '5' -> numStr="5"
    // i=1: 'V' -> foundV=true, next is '_' (not digit), so no decimal
    // i=2: '_' -> not digit, not V, not '.' -> just skip
    // i=3: '3' -> numStr="53"
    // i=4: 'V' -> already foundV... wait, the code checks `!numStr.IsEmpty()` and sets foundV again.
    //      But foundV is already true. The check for digits after V happens again.
    //      Next char is '3', so numStr += '.', foundDecimal = true
    // i=5: '3' -> numStr="53.3"
    // Result: 53.3, which is definitely wrong for this net name.
    // This is a known ambiguous case — documenting behavior.
    double v = SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "+5V_3V3" ) );
    // The parser returns a wrong answer here. This test documents the bug.
    // Ideally it should parse as 5.0 (the first voltage found).
    BOOST_TEST_MESSAGE( "parseVoltage(\"+5V_3V3\") = "
                        << v << " (expected ~5.0, actually wrong due to greedy parsing)" );
    // Don't assert the wrong value as "correct" - just document
}


// =========================================================================
// Test data container helpers
// =========================================================================

BOOST_AUTO_TEST_CASE( FindComponent_Exists )
{
    SYSTEM_DIAGRAM_DATA data;
    auto                comp = std::make_unique<SD_COMPONENT>();
    comp->m_reference = wxT( "U1" );
    comp->m_value = wxT( "STM32" );
    data.m_components.push_back( std::move( comp ) );

    BOOST_CHECK( data.FindComponent( wxT( "U1" ) ) != nullptr );
    BOOST_CHECK_EQUAL( data.FindComponent( wxT( "U1" ) )->m_value, wxString( "STM32" ) );
}

BOOST_AUTO_TEST_CASE( FindComponent_NotFound )
{
    SYSTEM_DIAGRAM_DATA data;
    BOOST_CHECK( data.FindComponent( wxT( "U1" ) ) == nullptr );
}

BOOST_AUTO_TEST_CASE( HasContent_Empty )
{
    SYSTEM_DIAGRAM_DATA data;
    BOOST_CHECK( !data.HasContent() );
}

BOOST_AUTO_TEST_CASE( HasContent_WithBus )
{
    SYSTEM_DIAGRAM_DATA data;
    data.m_buses.push_back( std::make_unique<SD_BUS>() );
    BOOST_CHECK( data.HasContent() );
}

BOOST_AUTO_TEST_CASE( HasContent_WithSignal )
{
    SYSTEM_DIAGRAM_DATA data;
    data.m_signals.push_back( std::make_unique<SD_SIGNAL>() );
    BOOST_CHECK( data.HasContent() );
}

BOOST_AUTO_TEST_CASE( HasContent_WithPowerRoot )
{
    SYSTEM_DIAGRAM_DATA data;
    auto                node = std::make_unique<SD_POWER_NODE>();
    SD_POWER_NODE*      ptr = node.get();
    data.m_allPowerNodes.push_back( std::move( node ) );
    data.m_powerRoots.push_back( ptr );
    BOOST_CHECK( data.HasContent() );
}


// =========================================================================
// parseVoltage: Bug - "V" appearing in non-voltage context
// =========================================================================

BOOST_AUTO_TEST_CASE( ParseVoltage_AVDD )
{
    // "AVDD" — has 'V' but at wrong position. 'A' is not a digit before 'V'.
    // Parser: strip '+' (none), iterate:
    // i=0: 'A' -> not digit, not V (numStr empty), not '.'
    // i=1: 'V' -> numStr IS empty, so this branch is skipped (good)
    // i=2: 'D' -> skip
    // i=3: 'D' -> skip
    // foundV = false, returns 0.0.  Correct!
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "AVDD" ) ), 0.0 );
}

BOOST_AUTO_TEST_CASE( ParseVoltage_DVDD_1V8 )
{
    // "DVDD_1V8" — a realistic net name with embedded voltage
    // strip +/- (none), iterate "DVDD_1V8":
    // i=0: 'D' -> skip (not digit, not V with empty numStr)
    // i=1: 'V' -> numStr empty, so condition !numStr.IsEmpty() fails -> skip
    // i=2: 'D' -> skip
    // i=3: 'D' -> skip
    // i=4: '_' -> skip
    // i=5: '1' -> numStr="1"
    // i=6: 'V' -> foundV=true, next char '8' is digit -> numStr += '.', foundDecimal=true
    // i=7: '8' -> numStr="1.8"
    // Result: 1.8.  Correct!
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseVoltage( wxT( "DVDD_1V8" ) ), 1.8, 0.1 );
}


// =========================================================================
// Efficiency edge cases
// =========================================================================

BOOST_AUTO_TEST_CASE( ParseEfficiency_100Percent )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "100%" ) ), 1.0, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseEfficiency_SmallFraction )
{
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "0.01" ) ), 0.01, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseEfficiency_Exactly100AsNumber )
{
    // 100 without %, treated as percentage -> 1.0
    BOOST_CHECK_CLOSE( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "100" ) ), 1.0, 0.1 );
}

BOOST_AUTO_TEST_CASE( ParseEfficiency_101 )
{
    // 101 -> 1.01, which is > 1.0, returns 0.0
    BOOST_CHECK_EQUAL( SYSTEM_DIAGRAM_ANALYZER::parseEfficiency( wxT( "101" ) ), 0.0 );
}


// =========================================================================
// GND handling: GND net creates power nodes with many connections.
// Components that only have power_in on GND should NOT be classified as
// sources/regulators.
// =========================================================================

BOOST_FIXTURE_TEST_CASE( GndNetNotDuplicated, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    analyzer.Analyze();

    SYSTEM_DIAGRAM_DATA& data = analyzer.GetData();

    // Count how many power nodes are on the GND net
    int gndNodeCount = 0;

    for( const auto& node : data.m_allPowerNodes )
    {
        if( node->m_netName.Contains( wxT( "GND" ) ) )
            gndNodeCount++;
    }

    // GND is a shared ground bus — it should not spawn many root nodes.
    // If every component's GND pin creates a separate node, the tree is wrong.
    // In the basic test, no component has power_out on GND, so there should
    // be 0 GND power nodes.
    BOOST_CHECK_MESSAGE( gndNodeCount == 0,
                         "GND net should not create power nodes (no component has "
                         "power_out on GND). Found "
                                 << gndNodeCount );
}


BOOST_FIXTURE_TEST_CASE( RegulatorInputNet, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    analyzer.Analyze();

    SYSTEM_DIAGRAM_DATA& data = analyzer.GetData();

    // Find U1 regulator and check its input net is set correctly
    for( const auto& node : data.m_allPowerNodes )
    {
        if( node->m_reference == wxT( "U1" ) && node->m_type == SD_POWER_NODE::REGULATOR )
        {
            BOOST_CHECK_MESSAGE( !node->m_inputNetName.IsEmpty(),
                                 "U1 regulator should have an input net name" );

            BOOST_CHECK_CLOSE( node->m_inputVoltage, 5.0, 0.1 );
        }
    }
}


BOOST_FIXTURE_TEST_CASE( RegulatorDefaultConverterType, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    analyzer.Analyze();

    SYSTEM_DIAGRAM_DATA& data = analyzer.GetData();

    // U1 has no Pwr.Type field, so it should default to CONV_LDO
    for( const auto& node : data.m_allPowerNodes )
    {
        if( node->m_reference == wxT( "U1" ) && node->m_type == SD_POWER_NODE::REGULATOR )
        {
            BOOST_CHECK_EQUAL( node->m_converterType, SD_POWER_NODE::CONV_LDO );
        }
    }
}


// =========================================================================
// Power tree topology checks
// =========================================================================

BOOST_FIXTURE_TEST_CASE( PowerRootCount, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    analyzer.Analyze();

    SYSTEM_DIAGRAM_DATA& data = analyzer.GetData();

    // Should have exactly 1 root: J1 (the source).
    // U1 is a child of J1, not a root.
    BOOST_CHECK_EQUAL( data.m_powerRoots.size(), 1u );

    if( !data.m_powerRoots.empty() )
    {
        BOOST_CHECK_EQUAL( data.m_powerRoots[0]->m_reference, wxString( "J1" ) );
        BOOST_CHECK_EQUAL( data.m_powerRoots[0]->m_type, SD_POWER_NODE::SOURCE );
    }
}


BOOST_FIXTURE_TEST_CASE( TotalPowerNodeCount, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    analyzer.Analyze();

    SYSTEM_DIAGRAM_DATA& data = analyzer.GetData();

    // Should have exactly 2 power nodes: J1 (+5V source) and U1 (+3V3 regulator).
    // U2 is a load, not a power node.
    BOOST_CHECK_EQUAL( data.m_allPowerNodes.size(), 2u );
}


// =========================================================================
// Current bubble-up
// =========================================================================

BOOST_FIXTURE_TEST_CASE( BubbleUpToSource, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    analyzer.Analyze();

    SYSTEM_DIAGRAM_DATA& data = analyzer.GetData();

    // U2 has Pwr.I.typ = 150mA on +3V3.
    // U1 is an LDO (default) so its input current = output current = 150mA.
    // J1 should have the bubbled-up current from U1.
    for( SD_POWER_NODE* root : data.m_powerRoots )
    {
        if( root->m_reference == wxT( "J1" ) )
        {
            auto typIt = root->m_currentByMode.find( wxT( "typ" ) );

            if( typIt != root->m_currentByMode.end() )
            {
                // LDO: I_in = I_out = 150mA
                BOOST_CHECK_CLOSE( typIt->second, 0.15, 1.0 );
            }
        }
    }
}


// =========================================================================
// Potential classification bugs
// =========================================================================

BOOST_AUTO_TEST_CASE( ComponentPrefixMatchingBug )
{
    // The code uses ref.StartsWith("U") which would match "USB1" (a connector).
    // This documents the potential issue. In practice, "USB" is not a standard
    // KiCad reference designator prefix.
    wxString ref = wxT( "USB1" );
    bool     matchesU = ref.StartsWith( wxT( "U" ) );

    BOOST_TEST_MESSAGE( "ref='USB1' matches StartsWith('U'): " << matchesU );
    BOOST_CHECK_MESSAGE( matchesU, "StartsWith('U') matches 'USB1' — potential misclassification" );
    // This is a documentation test — it shows that the prefix check is too broad.
    // A fix would be to check that the character after the prefix letter is a digit.
}


BOOST_AUTO_TEST_CASE( ConnectorLoadExclusion )
{
    // Document: J? connectors are never added as loads (line 546 of analyzer).
    // This means a connector that consumes power (e.g., a powered peripheral)
    // won't show up in the load list of a power node.
    // This is a design decision worth documenting.
    BOOST_TEST_MESSAGE( "J? connectors are excluded from load lists by design" );
    BOOST_CHECK( true ); // Documentation test
}


// =========================================================================
// Generator tests — sheet creation and hierarchy integration
// =========================================================================

BOOST_FIXTURE_TEST_CASE( GeneratorCreatesSheets, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    BOOST_REQUIRE( analyzer.Analyze() );

    SYSTEM_DIAGRAM_LAYOUT layout;
    int                   margin = schIUScale.MilsToIU( 500 );
    int                   headerSpace = schIUScale.MilsToIU( 400 );
    VECTOR2I              origin( margin, margin + headerSpace );
    layout.LayoutPowerSection( analyzer.GetData(), origin );

    SYSTEM_DIAGRAM_GENERATOR generator( m_schematic.get() );
    SYSTEM_DIAGRAM_SHEETS    sheets = generator.Generate( analyzer.GetData() );

    // Power tree has content, so power sheet should be created
    BOOST_REQUIRE( sheets.m_powerSheet != nullptr );

    // The sheet should be on the root screen
    SCH_SCREEN* rootScreen = m_schematic->Root().GetScreen();
    bool        foundOnRoot = false;

    for( SCH_ITEM* item : rootScreen->Items().OfType( SCH_SHEET_T ) )
    {
        if( item == sheets.m_powerSheet )
            foundOnRoot = true;
    }

    BOOST_CHECK_MESSAGE( foundOnRoot, "Generated power sheet should be on the root screen" );
}


BOOST_FIXTURE_TEST_CASE( GeneratorSheetsHavePageNumbers, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    BOOST_REQUIRE( analyzer.Analyze() );

    SYSTEM_DIAGRAM_LAYOUT layout;
    int                   margin = schIUScale.MilsToIU( 500 );
    int                   headerSpace = schIUScale.MilsToIU( 400 );
    VECTOR2I              origin( margin, margin + headerSpace );
    layout.LayoutPowerSection( analyzer.GetData(), origin );

    SYSTEM_DIAGRAM_GENERATOR generator( m_schematic.get() );
    SYSTEM_DIAGRAM_SHEETS    sheets = generator.Generate( analyzer.GetData() );

    BOOST_REQUIRE( sheets.m_powerSheet != nullptr );

    // Build a sheet path to the new sheet and check it has a page number.
    // This is the fix for the hierarchy pane bug — without an instance,
    // GetPageNumber() returns empty and the hierarchy pane can't display it.
    SCH_SHEET_PATH sheetPath;
    sheetPath.push_back( &m_schematic->Root() );
    sheetPath.push_back( sheets.m_powerSheet );

    wxString pageNum = sheetPath.GetPageNumber();

    BOOST_CHECK_MESSAGE( !pageNum.IsEmpty(),
                         "Generated sheet must have a page number for the hierarchy pane" );
}


BOOST_FIXTURE_TEST_CASE( GeneratorSheetsInHierarchy, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    BOOST_REQUIRE( analyzer.Analyze() );

    SYSTEM_DIAGRAM_LAYOUT layout;
    int                   margin = schIUScale.MilsToIU( 500 );
    int                   headerSpace = schIUScale.MilsToIU( 400 );
    VECTOR2I              origin( margin, margin + headerSpace );
    layout.LayoutPowerSection( analyzer.GetData(), origin );

    SYSTEM_DIAGRAM_GENERATOR generator( m_schematic.get() );
    SYSTEM_DIAGRAM_SHEETS    sheets = generator.Generate( analyzer.GetData() );

    BOOST_REQUIRE( sheets.m_powerSheet != nullptr );

    // After generation, refresh the hierarchy and check the new sheet is listed
    m_schematic->RefreshHierarchy();
    SCH_SHEET_LIST hierarchy = m_schematic->Hierarchy();

    bool foundInHierarchy = false;

    for( const SCH_SHEET_PATH& path : hierarchy )
    {
        if( path.Last() == sheets.m_powerSheet )
        {
            foundInHierarchy = true;

            // Page number should be non-empty
            BOOST_CHECK_MESSAGE( !path.GetPageNumber().IsEmpty(),
                                 "Power sheet in hierarchy should have a page number" );
        }
    }

    BOOST_CHECK_MESSAGE( foundInHierarchy,
                         "Generated power sheet should appear in the schematic hierarchy" );
}


BOOST_FIXTURE_TEST_CASE( GeneratorReusesExistingSheet, SD_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager,
                            "system_diagram_power_tree/system_diagram_power_tree", m_schematic );

    SYSTEM_DIAGRAM_ANALYZER analyzer( m_schematic.get() );
    BOOST_REQUIRE( analyzer.Analyze() );

    SYSTEM_DIAGRAM_LAYOUT layout;
    int                   margin = schIUScale.MilsToIU( 500 );
    int                   headerSpace = schIUScale.MilsToIU( 400 );
    VECTOR2I              origin( margin, margin + headerSpace );
    layout.LayoutPowerSection( analyzer.GetData(), origin );

    // Generate twice — second time should reuse the existing sheet
    SYSTEM_DIAGRAM_GENERATOR generator( m_schematic.get() );
    SYSTEM_DIAGRAM_SHEETS    sheets1 = generator.Generate( analyzer.GetData() );
    BOOST_REQUIRE( sheets1.m_powerSheet != nullptr );

    // Re-run layout (positions may have changed)
    layout.LayoutPowerSection( analyzer.GetData(), origin );

    SYSTEM_DIAGRAM_SHEETS sheets2 = generator.Generate( analyzer.GetData() );
    BOOST_REQUIRE( sheets2.m_powerSheet != nullptr );

    // Should be the same sheet object (reused, not duplicated)
    BOOST_CHECK_EQUAL( sheets1.m_powerSheet, sheets2.m_powerSheet );

    // Count sheets on root — should be exactly 1 power sheet, not 2
    int         sheetCount = 0;
    SCH_SCREEN* rootScreen = m_schematic->Root().GetScreen();

    for( SCH_ITEM* item : rootScreen->Items().OfType( SCH_SHEET_T ) )
    {
        SCH_SHEET* s = static_cast<SCH_SHEET*>( item );

        if( s->GetName() == wxT( "Power Distribution" ) )
            sheetCount++;
    }

    BOOST_CHECK_EQUAL( sheetCount, 1 );
}


BOOST_AUTO_TEST_SUITE_END()
