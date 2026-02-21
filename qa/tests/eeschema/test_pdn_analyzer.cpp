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

#include <qa_utils/wx_utils/unit_test_utils.h>
#include <schematic_utils/schematic_file_util.h>

#include <sim/pdn_analyzer.h>
#include <sim/simulator.h>
#include <sim/spice_simulator.h>
#include <sim/spice_settings.h>
#include <schematic.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <settings/settings_manager.h>
#include <locale_io.h>
#include <cmath>
#include <wx/utils.h>


std::ostream& boost_test_print_type( std::ostream& os, wxString const& aStr )
{
    os << aStr.ToStdString();
    return os;
}


BOOST_AUTO_TEST_SUITE( PdnAnalyzer )


BOOST_AUTO_TEST_CASE( ParseCaseSizeMetric )
{
    BOOST_CHECK_EQUAL( PDN_ANALYZER::ParseCaseSize( wxS( "Capacitor_SMD:C_0402_1005Metric" ) ),
                       wxString( "1005" ) );
    BOOST_CHECK_EQUAL( PDN_ANALYZER::ParseCaseSize( wxS( "Capacitor_SMD:C_0603_1608Metric" ) ),
                       wxString( "1608" ) );
    BOOST_CHECK_EQUAL( PDN_ANALYZER::ParseCaseSize( wxS( "Capacitor_SMD:C_0805_2012Metric" ) ),
                       wxString( "2012" ) );
    BOOST_CHECK_EQUAL( PDN_ANALYZER::ParseCaseSize( wxS( "Capacitor_SMD:C_1206_3216Metric" ) ),
                       wxString( "3216" ) );
}


BOOST_AUTO_TEST_CASE( ParseCaseSizeEIA )
{
    BOOST_CHECK_EQUAL( PDN_ANALYZER::ParseCaseSize( wxS( "C_0402_something" ) ),
                       wxString( "1005" ) );
    BOOST_CHECK_EQUAL( PDN_ANALYZER::ParseCaseSize( wxS( "C_0603_something" ) ),
                       wxString( "1608" ) );
}


BOOST_AUTO_TEST_CASE( ParseCaseSizeGeneric )
{
    // Should find known sizes anywhere in the string without naming convention
    BOOST_CHECK_EQUAL( PDN_ANALYZER::ParseCaseSize( wxS( "MyLib:CAP_2012" ) ), wxString( "2012" ) );
    BOOST_CHECK_EQUAL( PDN_ANALYZER::ParseCaseSize( wxS( "1005_cap" ) ), wxString( "1005" ) );
}


BOOST_AUTO_TEST_CASE( ParseCaseSizeUnknown )
{
    // Unknown footprint should return empty string
    BOOST_CHECK( PDN_ANALYZER::ParseCaseSize( wxS( "unknown:footprint" ) ).IsEmpty() );
    BOOST_CHECK( PDN_ANALYZER::ParseCaseSize( wxS( "" ) ).IsEmpty() );
}


BOOST_AUTO_TEST_CASE( ParasiticsLookup )
{
    // Known case sizes should return valid parasitics.
    // ESR is computed from: k(case) * C^(-0.429)
    // ESL is a fixed per-case-size median from Murata SPICE data.

    // 1005, 100nF: ESR = 3.15e-5 * (100e-9)^(-0.429)
    auto p1005 = PDN_ANALYZER::GetParasitics( wxS( "1005" ), 100e-9 );
    BOOST_REQUIRE( p1005.has_value() );
    BOOST_CHECK_CLOSE( p1005->esr, 3.15e-5 * std::pow( 100e-9, -0.429 ), 0.1 );
    BOOST_CHECK_CLOSE( p1005->esl, 270e-12, 0.1 );

    // 1608, 100nF: ESR = 2.93e-5 * (100e-9)^(-0.429)
    auto p1608 = PDN_ANALYZER::GetParasitics( wxS( "1608" ), 100e-9 );
    BOOST_REQUIRE( p1608.has_value() );
    BOOST_CHECK_CLOSE( p1608->esr, 2.93e-5 * std::pow( 100e-9, -0.429 ), 0.1 );
    BOOST_CHECK_CLOSE( p1608->esl, 380e-12, 0.1 );

    // 3216, 10uF: ESR = 3.43e-5 * (10e-6)^(-0.429)
    auto p3216 = PDN_ANALYZER::GetParasitics( wxS( "3216" ), 10e-6 );
    BOOST_REQUIRE( p3216.has_value() );
    BOOST_CHECK_CLOSE( p3216->esr, 3.43e-5 * std::pow( 10e-6, -0.429 ), 0.1 );
    BOOST_CHECK_CLOSE( p3216->esl, 542e-12, 0.1 );

    // ESR should decrease with increasing capacitance (power law, negative exponent)
    auto p1608_small = PDN_ANALYZER::GetParasitics( wxS( "1608" ), 1e-9 );
    auto p1608_large = PDN_ANALYZER::GetParasitics( wxS( "1608" ), 10e-6 );
    BOOST_REQUIRE( p1608_small.has_value() );
    BOOST_REQUIRE( p1608_large.has_value() );
    BOOST_CHECK_GT( p1608_small->esr, p1608_large->esr );

    // ESL should be the same regardless of capacitance
    BOOST_CHECK_CLOSE( p1608_small->esl, p1608_large->esl, 0.1 );

    // Unknown case size should return nullopt
    BOOST_CHECK( !PDN_ANALYZER::GetParasitics( wxS( "9999" ), 100e-9 ).has_value() );
}


BOOST_AUTO_TEST_CASE( BuildNetlistSingleCap )
{
    LOCALE_IO    dummy;
    PDN_ANALYZER analyzer;
    PDN_NETWORK  network;

    network.supplyRail = wxS( "+3V3" );
    network.refRail = wxS( "GND" );

    SCH_SHEET    rootSheet;
    SCH_SHEET_PATH obsSheet;
    obsSheet.push_back( &rootSheet );

    PDN_CAPACITOR cap;
    cap.refdes = wxS( "C1" );
    cap.capacitance = 100e-9;
    cap.caseSize = wxS( "1608" );
    cap.esr = 0.020;
    cap.esl = 400e-12;
    cap.netSupply = wxS( "+3V3" );
    cap.netRef = wxS( "GND" );
    cap.sheetPath = obsSheet;
    network.components.push_back( cap );

    wxString netlist = analyzer.BuildSpiceNetlist( network, obsSheet );

    BOOST_CHECK( netlist.Contains( wxS( ".subckt CAP " ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "I1 local gnd AC 1" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "X1 local gnd CAP" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( ".ac dec" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( ".end" ) ) );
}


BOOST_AUTO_TEST_CASE( BuildNetlistMultipleCaps )
{
    LOCALE_IO    dummy;
    PDN_ANALYZER analyzer;
    PDN_NETWORK  network;

    network.supplyRail = wxS( "+5V" );
    network.refRail = wxS( "GND" );

    SCH_SHEET    rootSheet;
    SCH_SHEET_PATH obsSheet;
    obsSheet.push_back( &rootSheet );

    PDN_CAPACITOR cap1;
    cap1.refdes = wxS( "C1" );
    cap1.capacitance = 100e-9;
    cap1.caseSize = wxS( "1005" );
    cap1.esr = 0.030;
    cap1.esl = 200e-12;
    cap1.sheetPath = obsSheet;
    network.components.push_back( cap1 );

    PDN_CAPACITOR cap2;
    cap2.refdes = wxS( "C2" );
    cap2.capacitance = 10e-6;
    cap2.caseSize = wxS( "1608" );
    cap2.esr = 0.020;
    cap2.esl = 400e-12;
    cap2.sheetPath = obsSheet;
    network.components.push_back( cap2 );

    wxString netlist = analyzer.BuildSpiceNetlist( network, obsSheet );

    BOOST_CHECK( netlist.Contains( wxS( ".subckt CAP " ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "X1 local gnd CAP" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "X2 local gnd CAP" ) ) );
}


BOOST_AUTO_TEST_CASE( BuildNetlistNoLocalCaps )
{
    // When the observation sheet has no caps on the rail, all caps should be
    // remote (behind trace impedance).  No fallback to first group.
    LOCALE_IO    dummy;
    PDN_ANALYZER analyzer;
    PDN_NETWORK  network;

    network.supplyRail = wxS( "+3V3" );
    network.refRail = wxS( "GND" );

    SCH_SHEET rootSheet;
    SCH_SHEET childSheet;
    SCH_SHEET otherSheet;

    SCH_SHEET_PATH pathA;
    pathA.push_back( &rootSheet );

    SCH_SHEET_PATH pathB;
    pathB.push_back( &rootSheet );
    pathB.push_back( &childSheet );

    SCH_SHEET_PATH obsSheet;
    obsSheet.push_back( &rootSheet );
    obsSheet.push_back( &otherSheet );

    PDN_CAPACITOR cap1;
    cap1.refdes = wxS( "C1" );
    cap1.capacitance = 100e-9;
    cap1.caseSize = wxS( "1005" );
    cap1.esr = 0.030;
    cap1.esl = 200e-12;
    cap1.sheetPath = pathA;
    network.components.push_back( cap1 );

    PDN_CAPACITOR cap2;
    cap2.refdes = wxS( "C2" );
    cap2.capacitance = 22e-6;
    cap2.caseSize = wxS( "3216" );
    cap2.esr = 0.010;
    cap2.esl = 900e-12;
    cap2.sheetPath = pathB;
    network.components.push_back( cap2 );

    wxString netlist = analyzer.BuildSpiceNetlist( network, obsSheet );

    BOOST_CHECK( netlist.Contains( wxS( "I1 local gnd AC 1" ) ) );
    // No caps should be directly on "local" — all are remote
    BOOST_CHECK( !netlist.Contains( wxS( "X1 local gnd" ) ) );
    // Both groups should have trace elements
    BOOST_CHECK( netlist.Contains( wxS( "L_s1" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "R_s1" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "L_s2" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "R_s2" ) ) );
}


BOOST_AUTO_TEST_CASE( BuildNetlistClustered )
{
    LOCALE_IO    dummy;
    PDN_ANALYZER analyzer;
    PDN_NETWORK  network;

    network.supplyRail = wxS( "+3V3" );
    network.refRail = wxS( "GND" );

    SCH_SHEET rootSheet;
    SCH_SHEET childSheet;

    SCH_SHEET_PATH pathA;
    pathA.push_back( &rootSheet );

    SCH_SHEET_PATH pathB;
    pathB.push_back( &rootSheet );
    pathB.push_back( &childSheet );

    PDN_CAPACITOR cap1;
    cap1.refdes = wxS( "C1" );
    cap1.capacitance = 100e-9;
    cap1.caseSize = wxS( "1005" );
    cap1.esr = 0.030;
    cap1.esl = 200e-12;
    cap1.sheetPath = pathA;
    network.components.push_back( cap1 );

    PDN_CAPACITOR cap2;
    cap2.refdes = wxS( "C2" );
    cap2.capacitance = 22e-6;
    cap2.caseSize = wxS( "3216" );
    cap2.esr = 0.010;
    cap2.esl = 900e-12;
    cap2.sheetPath = pathB;
    network.components.push_back( cap2 );

    wxString netlist = analyzer.BuildSpiceNetlist( network, pathA );

    BOOST_CHECK( netlist.Contains( wxS( "I1 local gnd AC 1" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "X1 local gnd" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "L_s1" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "R_s1" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "X2 rs1b gnd" ) ) );
}


BOOST_AUTO_TEST_CASE( BuildNetlistClusteredReversed )
{
    LOCALE_IO    dummy;
    PDN_ANALYZER analyzer;
    PDN_NETWORK  network;

    network.supplyRail = wxS( "+3V3" );
    network.refRail = wxS( "GND" );

    SCH_SHEET rootSheet;
    SCH_SHEET childSheet;

    SCH_SHEET_PATH pathA;
    pathA.push_back( &rootSheet );

    SCH_SHEET_PATH pathB;
    pathB.push_back( &rootSheet );
    pathB.push_back( &childSheet );

    PDN_CAPACITOR cap1;
    cap1.refdes = wxS( "C1" );
    cap1.capacitance = 100e-9;
    cap1.caseSize = wxS( "1005" );
    cap1.esr = 0.030;
    cap1.esl = 200e-12;
    cap1.sheetPath = pathA;
    network.components.push_back( cap1 );

    PDN_CAPACITOR cap2;
    cap2.refdes = wxS( "C2" );
    cap2.capacitance = 22e-6;
    cap2.caseSize = wxS( "3216" );
    cap2.esr = 0.010;
    cap2.esl = 900e-12;
    cap2.sheetPath = pathB;
    network.components.push_back( cap2 );

    wxString netlist = analyzer.BuildSpiceNetlist( network, pathB );

    BOOST_CHECK( netlist.Contains( wxS( "I1 local gnd AC 1" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "X1 local gnd CAP" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "L_s1" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "X2 rs1b gnd CAP" ) ) );
}


BOOST_AUTO_TEST_CASE( BuildNetlistSingleSheetAllLocal )
{
    LOCALE_IO    dummy;
    PDN_ANALYZER analyzer;
    PDN_NETWORK  network;

    network.supplyRail = wxS( "+3V3" );
    network.refRail = wxS( "GND" );

    SCH_SHEET rootSheet;

    SCH_SHEET_PATH pathA;
    pathA.push_back( &rootSheet );

    PDN_CAPACITOR cap1;
    cap1.refdes = wxS( "C1" );
    cap1.capacitance = 100e-9;
    cap1.caseSize = wxS( "1005" );
    cap1.esr = 0.030;
    cap1.esl = 200e-12;
    cap1.sheetPath = pathA;
    network.components.push_back( cap1 );

    PDN_CAPACITOR cap2;
    cap2.refdes = wxS( "C2" );
    cap2.capacitance = 10e-6;
    cap2.caseSize = wxS( "1608" );
    cap2.esr = 0.020;
    cap2.esl = 400e-12;
    cap2.sheetPath = pathA;
    network.components.push_back( cap2 );

    wxString netlist = analyzer.BuildSpiceNetlist( network, pathA );

    BOOST_CHECK( netlist.Contains( wxS( "I1 local gnd AC 1" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "X1 local gnd" ) ) );
    BOOST_CHECK( netlist.Contains( wxS( "X2 local gnd" ) ) );
    BOOST_CHECK( !netlist.Contains( wxS( "L_s" ) ) );
    BOOST_CHECK( !netlist.Contains( wxS( "R_s" ) ) );
}


BOOST_AUTO_TEST_CASE( EmptySchematic )
{
    PDN_ANALYZER analyzer;

    analyzer.FindPDNNetworks();
    BOOST_CHECK( analyzer.GetNetworks().empty() );
}


BOOST_AUTO_TEST_CASE( EmptyNetworkAnalysis )
{
    PDN_ANALYZER analyzer;
    PDN_NETWORK  emptyNetwork;

    emptyNetwork.supplyRail = wxS( "+3V3" );
    emptyNetwork.refRail = wxS( "GND" );

    SCH_SHEET      rootSheet;
    SCH_SHEET_PATH obsSheet;
    obsSheet.push_back( &rootSheet );

    BOOST_CHECK( !analyzer.RunAnalysis( emptyNetwork, obsSheet ) );
}


BOOST_AUTO_TEST_CASE( SimulatorStepByStep )
{
    std::shared_ptr<SPICE_SIMULATOR> sim = SIMULATOR::CreateInstance( "ngspice" );

    BOOST_REQUIRE_MESSAGE( sim != nullptr, "SIMULATOR::CreateInstance returned null - "
                                           "ngspice library not found" );

    if( !sim->Settings() )
        sim->Settings() = std::make_shared<NGSPICE_SETTINGS>( nullptr, "" );

    sim->Init();
    BOOST_TEST_MESSAGE( "ngspice initialized successfully" );

    PDN_ANALYZER analyzer;
    PDN_NETWORK  network;

    network.supplyRail = wxS( "+3V3" );
    network.refRail = wxS( "GND" );

    SCH_SHEET      rootSheet;
    SCH_SHEET_PATH obsSheet;
    obsSheet.push_back( &rootSheet );

    PDN_CAPACITOR cap;
    cap.refdes = wxS( "C1" );
    cap.capacitance = 100e-9;
    cap.caseSize = wxS( "1608" );
    cap.esr = 0.020;
    cap.esl = 400e-12;
    cap.sheetPath = obsSheet;
    network.components.push_back( cap );

    wxString netlist = analyzer.BuildSpiceNetlist( network, obsSheet );
    BOOST_TEST_MESSAGE( "Generated netlist:\n" << netlist.ToStdString() );

    bool loaded = sim->LoadNetlist( netlist.ToStdString() );
    BOOST_REQUIRE_MESSAGE( loaded, "LoadNetlist failed - ngspice rejected the netlist" );

    BOOST_TEST_MESSAGE( "Current plot before run: " << sim->CurrentPlotName().ToStdString() );

    bool ran = sim->Command( "run" );
    BOOST_REQUIRE_MESSAGE( ran, "sim->Command(\"run\") returned false" );

    BOOST_TEST_MESSAGE( "Current plot after run: " << sim->CurrentPlotName().ToStdString() );

    std::vector<std::string> allVecs = sim->AllVectors();
    BOOST_TEST_MESSAGE( "Available vectors (" << allVecs.size() << "):" );

    for( const std::string& v : allVecs )
        BOOST_TEST_MESSAGE( "  " << v );

    std::vector<double> freqs = sim->GetGainVector( "frequency" );
    BOOST_TEST_MESSAGE( "frequency vector size: " << freqs.size() );
    BOOST_CHECK_MESSAGE( !freqs.empty(), "frequency vector is empty" );

    std::vector<double> vdd = sim->GetGainVector( "v(local)" );
    BOOST_TEST_MESSAGE( "v(local) vector size: " << vdd.size() );
    BOOST_CHECK_MESSAGE( !vdd.empty(), "v(local) vector is empty" );

    if( !freqs.empty() && !vdd.empty() )
    {
        BOOST_TEST_MESSAGE( "First freq: " << freqs[0] << " Hz, Z: " << vdd[0] << " Ohm" );
        BOOST_TEST_MESSAGE( "Last freq: " << freqs.back() << " Hz, Z: " << vdd.back() << " Ohm" );
    }
}


BOOST_AUTO_TEST_CASE( RunAnalysisSingleCap )
{
    PDN_ANALYZER analyzer;
    PDN_NETWORK  network;

    network.supplyRail = wxS( "+3V3" );
    network.refRail = wxS( "GND" );

    SCH_SHEET      rootSheet;
    SCH_SHEET_PATH obsSheet;
    obsSheet.push_back( &rootSheet );

    PDN_CAPACITOR cap;
    cap.refdes = wxS( "C1" );
    cap.capacitance = 100e-9;
    cap.caseSize = wxS( "1608" );
    cap.esr = 0.020;
    cap.esl = 400e-12;
    cap.sheetPath = obsSheet;
    network.components.push_back( cap );

    bool result = analyzer.RunAnalysis( network, obsSheet );
    BOOST_CHECK_MESSAGE( result, "RunAnalysis returned false" );

    if( result )
    {
        BOOST_TEST_MESSAGE( "Frequencies: " << analyzer.GetFrequencies().size() << " points" );
        BOOST_TEST_MESSAGE( "Impedance: " << analyzer.GetImpedance().size() << " points" );
        BOOST_CHECK( !analyzer.GetFrequencies().empty() );
        BOOST_CHECK( !analyzer.GetImpedance().empty() );

        for( double z : analyzer.GetImpedance() )
            BOOST_CHECK( z > 0.0 );
    }
}


struct PDN_SCHEMATIC_FIXTURE
{
    PDN_SCHEMATIC_FIXTURE() :
            m_settingsManager( true /* headless */ )
    { }

    SETTINGS_MANAGER           m_settingsManager;
    std::unique_ptr<SCHEMATIC> m_schematic;
};


BOOST_FIXTURE_TEST_CASE( FindNetworksRequiresBothPinsOnPowerNets, PDN_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager, "pdn_filter/pdn_filter", m_schematic );

    m_schematic->RefreshHierarchy();

    PDN_ANALYZER analyzer;
    analyzer.SetSchematic( m_schematic.get() );
    analyzer.FindPDNNetworks();

    const std::map<wxString, PDN_NETWORK>& networks = analyzer.GetNetworks();

    BOOST_REQUIRE_MESSAGE( networks.size() == 1,
                           "Expected 1 PDN network, found " << networks.size() );

    const PDN_NETWORK& net = networks.begin()->second;
    BOOST_CHECK( net.supplyRail.Contains( wxS( "+3V3" ) ) );
    BOOST_CHECK( net.refRail.Contains( wxS( "GND" ) ) );

    BOOST_CHECK_EQUAL( net.components.size(), 1u );

    if( !net.components.empty() )
    {
        BOOST_CHECK_EQUAL( net.components[0].refdes, wxString( "C1" ) );
    }
}


BOOST_AUTO_TEST_CASE( FindNetworkForNetNameLookup )
{
    PDN_ANALYZER analyzer;
    PDN_NETWORK  network;

    network.supplyRail = wxS( "+3V3" );
    network.refRail = wxS( "GND" );

    PDN_CAPACITOR cap;
    cap.refdes = wxS( "C1" );
    cap.capacitance = 100e-9;
    cap.caseSize = wxS( "1005" );
    cap.esr = 0.030;
    cap.esl = 200e-12;
    network.components.push_back( cap );

    // Manually insert network into analyzer's map via FindPDNNetworks won't work
    // without a schematic, so test the public API by building a network first.
    // We need to use the analyzer's internal map, so use a roundabout approach.
    // Instead, test with the fixture schematic below or test directly:

    // FindNetworkForNetName should return empty when no networks exist
    BOOST_CHECK( analyzer.FindNetworkForNetName( wxS( "+3V3" ) ).IsEmpty() );
    BOOST_CHECK( analyzer.FindNetworkForNetName( wxS( "GND" ) ).IsEmpty() );
    BOOST_CHECK( analyzer.FindNetworkForNetName( wxS( "" ) ).IsEmpty() );
}


BOOST_FIXTURE_TEST_CASE( FindNetworkForNetNameWithSchematic, PDN_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager, "pdn_filter/pdn_filter", m_schematic );
    m_schematic->RefreshHierarchy();

    PDN_ANALYZER analyzer;
    analyzer.SetSchematic( m_schematic.get() );
    analyzer.FindPDNNetworks();

    // The test schematic has a +3V3/GND network
    wxString key = analyzer.FindNetworkForNetName( wxS( "+3V3" ) );
    BOOST_CHECK( !key.IsEmpty() );

    // GND is the reference rail — should also match
    BOOST_CHECK( !analyzer.FindNetworkForNetName( wxS( "GND" ) ).IsEmpty() );

    // Non-existent net
    BOOST_CHECK( analyzer.FindNetworkForNetName( wxS( "NONEXISTENT" ) ).IsEmpty() );
}


BOOST_FIXTURE_TEST_CASE( FindNetworkForSymWithSchematic, PDN_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager, "pdn_filter/pdn_filter", m_schematic );
    m_schematic->RefreshHierarchy();

    PDN_ANALYZER analyzer;
    analyzer.SetSchematic( m_schematic.get() );
    analyzer.FindPDNNetworks();

    // C1 is in the network
    wxString key = analyzer.FindNetworkForSym( wxS( "C1" ) );
    BOOST_CHECK( !key.IsEmpty() );

    // C2 has one pin not on a power net, so it's filtered out
    BOOST_CHECK( analyzer.FindNetworkForSym( wxS( "C2" ) ).IsEmpty() );

    // Non-existent refdes
    BOOST_CHECK( analyzer.FindNetworkForSym( wxS( "C99" ) ).IsEmpty() );
}


BOOST_FIXTURE_TEST_CASE( NetworkHasSymsOnSheetTest, PDN_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager, "pdn_filter/pdn_filter", m_schematic );
    m_schematic->RefreshHierarchy();

    PDN_ANALYZER analyzer;
    analyzer.SetSchematic( m_schematic.get() );
    analyzer.FindPDNNetworks();

    const auto& networks = analyzer.GetNetworks();
    BOOST_REQUIRE( !networks.empty() );

    wxString key = networks.begin()->first;

    // Root sheet should have components
    SCH_SHEET_LIST sheets = m_schematic->Hierarchy();
    BOOST_REQUIRE( !sheets.empty() );

    BOOST_CHECK( analyzer.NetworkHasSymsOnSheet( key, sheets[0] ) );

    // Non-existent key
    BOOST_CHECK( !analyzer.NetworkHasSymsOnSheet( wxS( "FAKE / NET" ), sheets[0] ) );
}


BOOST_FIXTURE_TEST_CASE( NetworkIsMultiSheetTest, PDN_SCHEMATIC_FIXTURE )
{
    LOCALE_IO dummy;
    KI_TEST::LoadSchematic( m_settingsManager, "pdn_filter/pdn_filter", m_schematic );
    m_schematic->RefreshHierarchy();

    PDN_ANALYZER analyzer;
    analyzer.SetSchematic( m_schematic.get() );
    analyzer.FindPDNNetworks();

    const auto& networks = analyzer.GetNetworks();
    BOOST_REQUIRE( !networks.empty() );

    wxString key = networks.begin()->first;

    // Single-sheet schematic should not be multi-sheet
    BOOST_CHECK( !analyzer.NetworkIsMultiSheet( key ) );

    // Non-existent key
    BOOST_CHECK( !analyzer.NetworkIsMultiSheet( wxS( "FAKE / NET" ) ) );
}


BOOST_AUTO_TEST_CASE( WarningCollection )
{
    PDN_ANALYZER analyzer;

    BOOST_CHECK( analyzer.GetWarnings().empty() );

    // RunAnalysis with empty network should produce a warning keyed to the network
    PDN_NETWORK emptyNetwork;
    emptyNetwork.supplyRail = wxS( "+3V3" );
    emptyNetwork.refRail = wxS( "GND" );

    SCH_SHEET_PATH sheet;
    analyzer.RunAnalysis( emptyNetwork, sheet );

    // All warnings
    BOOST_CHECK( !analyzer.GetWarnings().empty() );
    BOOST_CHECK( analyzer.GetWarnings()[0].Contains( wxS( "+3V3" ) ) );

    // Per-network warnings
    BOOST_CHECK( !analyzer.GetWarnings( wxS( "+3V3 / GND" ) ).empty() );
    BOOST_CHECK( analyzer.GetWarnings( wxS( "FAKE / NET" ) ).empty() );

    analyzer.ClearWarnings();
    BOOST_CHECK( analyzer.GetWarnings().empty() );
}


BOOST_AUTO_TEST_SUITE_END()
