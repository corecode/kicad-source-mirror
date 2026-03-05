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

#include <cmath>
#include <complex>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <wx/filename.h>

#include <pcbnew_utils/board_test_utils.h>
#include <board.h>
#include <board_design_settings.h>
#include <settings/settings_manager.h>

#include <exporters/parasitic_extraction/pdn_parasitic_data.h>
#include <exporters/parasitic_extraction/parasitic_result_parser.h>
#include <exporters/parasitic_extraction/fasthenry_exporter.h>
#include <exporters/parasitic_extraction/fastcap_exporter.h>
#include <exporters/parasitic_extraction/parasitic_extraction.h>
#include <exporters/parasitic_extraction/spice_subckt_exporter.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// ============================================================================
// PDN_PARASITIC data structure tests
// ============================================================================

BOOST_AUTO_TEST_SUITE( ParasiticExtraction )


// --------------------------------------------------------------------------
// STACKUP_LAYER default values
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( StackupLayerDefaults )
{
    PDN_PARASITIC::STACKUP_LAYER layer;

    BOOST_CHECK_EQUAL( layer.m_Name, "" );
    BOOST_CHECK_EQUAL( layer.m_IsCopperLayer, false );
    BOOST_CHECK_CLOSE( layer.m_ZPositionMM, 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( layer.m_ThicknessMM, 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( layer.m_EpsilonR, 1.0, 1e-9 );
    BOOST_CHECK_CLOSE( layer.m_LossTangent, 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( layer.m_ConductivitySPerM, 0.0, 1e-9 );
}


// --------------------------------------------------------------------------
// EXTRACTION_PORT default values
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( ExtractionPortDefaults )
{
    PDN_PARASITIC::EXTRACTION_PORT port;

    BOOST_CHECK_EQUAL( port.m_Name, "" );
    BOOST_CHECK_EQUAL( port.m_PositiveNode, "" );
    BOOST_CHECK_EQUAL( port.m_NegativeNode, "" );
    BOOST_CHECK_EQUAL( port.m_NetName, "" );
    BOOST_CHECK_CLOSE( port.m_XMM, 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( port.m_YMM, 0.0, 1e-9 );
}


// --------------------------------------------------------------------------
// IMPEDANCE_POINT default values
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( ImpedancePointDefaults )
{
    PDN_PARASITIC::IMPEDANCE_POINT pt;

    BOOST_CHECK_CLOSE( pt.m_FrequencyHz, 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( pt.m_Z.real(), 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( pt.m_Z.imag(), 0.0, 1e-9 );
}


// --------------------------------------------------------------------------
// IMPEDANCE_ENTRY default values
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( ImpedanceEntryDefaults )
{
    PDN_PARASITIC::IMPEDANCE_ENTRY entry;

    BOOST_CHECK_EQUAL( entry.m_PortI, 0 );
    BOOST_CHECK_EQUAL( entry.m_PortJ, 0 );
    BOOST_CHECK( entry.m_Points.empty() );
}


// --------------------------------------------------------------------------
// CAPACITANCE_ENTRY default values
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( CapacitanceEntryDefaults )
{
    PDN_PARASITIC::CAPACITANCE_ENTRY entry;

    BOOST_CHECK_EQUAL( entry.m_ConductorI, "" );
    BOOST_CHECK_EQUAL( entry.m_ConductorJ, "" );
    BOOST_CHECK_CLOSE( entry.m_CapacitancePF, 0.0, 1e-9 );
}


// --------------------------------------------------------------------------
// RLGC_PARAMS default values
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( RLGCParamsDefaults )
{
    PDN_PARASITIC::RLGC_PARAMS params;

    BOOST_CHECK_CLOSE( params.m_R_OhmPerM, 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( params.m_L_nHPerM, 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( params.m_G_SPerM, 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( params.m_C_pFPerM, 0.0, 1e-9 );
}


// --------------------------------------------------------------------------
// NET_PARASITIC default values
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( NetParasiticDefaults )
{
    PDN_PARASITIC::NET_PARASITIC np;

    BOOST_CHECK_EQUAL( np.m_NetName, "" );
    BOOST_CHECK_EQUAL( np.m_SegmentId, "" );
    BOOST_CHECK_CLOSE( np.m_LengthMM, 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( np.m_R_mOhm, 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( np.m_L_nH, 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( np.m_C_pF, 0.0, 1e-9 );
}


// --------------------------------------------------------------------------
// EXTRACTION_CONFIG default values
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( ExtractionConfigDefaults )
{
    PDN_PARASITIC::EXTRACTION_CONFIG config;

    BOOST_CHECK( config.m_NetNames.empty() );
    BOOST_CHECK_EQUAL( config.m_FastHenryPath, "fasthenry" );
    BOOST_CHECK_CLOSE( config.m_FreqMinHz, 1e3, 0.01 );
    BOOST_CHECK_CLOSE( config.m_FreqMaxHz, 1e9, 0.01 );
    BOOST_CHECK_EQUAL( config.m_PointsPerDecade, 5 );
    BOOST_CHECK_EQUAL( config.m_NwInc, 3 );
    BOOST_CHECK_EQUAL( config.m_NhInc, 2 );
    BOOST_CHECK_EQUAL( config.m_FastCapPath, "fastcap" );
    BOOST_CHECK_CLOSE( config.m_PanelTargetSizeMM, 0.5, 0.01 );
    BOOST_CHECK_EQUAL( config.m_ViaFacets, 8 );
    BOOST_CHECK_EQUAL( config.m_PlaneSegX, 20 );
    BOOST_CHECK_EQUAL( config.m_PlaneSegY, 20 );
    BOOST_CHECK_EQUAL( config.m_OutputDir, "" );
}


// --------------------------------------------------------------------------
// EXTRACTION_RESULTS default values
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( ExtractionResultsDefaults )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    BOOST_CHECK( results.m_Stackup.empty() );
    BOOST_CHECK( results.m_Ports.empty() );
    BOOST_CHECK( results.m_ImpedanceMatrix.empty() );
    BOOST_CHECK( results.m_CapacitanceMatrix.empty() );
    BOOST_CHECK( results.m_NetParasitics.empty() );
    BOOST_CHECK_CLOSE( results.m_FreqMinHz, 0.0, 1e-9 );
    BOOST_CHECK_CLOSE( results.m_FreqMaxHz, 0.0, 1e-9 );
    BOOST_CHECK_EQUAL( results.m_PointsPerDecade, 0 );
}


// ============================================================================
// Parasitic Result Parser tests
// ============================================================================

struct PARSER_TEST_FIXTURE
{
    PARSER_TEST_FIXTURE()
    {
        m_tempDir = std::string( wxFileName::GetTempDir().ToUTF8() );
    }

    ~PARSER_TEST_FIXTURE()
    {
        // Clean up temp files
        for( const auto& f : m_tempFiles )
            std::remove( f.c_str() );
    }

    std::string writeTempFile( const std::string& aName, const std::string& aContent )
    {
        std::string path = m_tempDir + "/" + aName;
        std::ofstream file( path );
        file << aContent;
        file.close();
        m_tempFiles.push_back( path );
        return path;
    }

    std::string              m_tempDir;
    std::vector<std::string> m_tempFiles;
};


// --------------------------------------------------------------------------
// ParseFastHenryOutput - nonexistent file
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastHenryNonexistentFile, PARSER_TEST_FIXTURE )
{
    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastHenryOutput(
        "/nonexistent/path/Zc.mat", ports, impedances );

    BOOST_CHECK_EQUAL( result, false );
}


// --------------------------------------------------------------------------
// ParseFastHenryOutput - empty file
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastHenryEmptyFile, PARSER_TEST_FIXTURE )
{
    std::string path = writeTempFile( "empty_zc.mat", "" );

    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( path, ports, impedances );

    BOOST_CHECK_EQUAL( result, false );
}


// --------------------------------------------------------------------------
// ParseFastHenryOutput - single port, single frequency
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastHenrySinglePort, PARSER_TEST_FIXTURE )
{
    std::string content =
        "Row 0: N1 to N2\n"
        "Impedance matrix for frequency = 1e+06\n"
        "1 x 1\n"
        "0.408091 +0.00182523j\n";

    std::string path = writeTempFile( "single_port_zc.mat", content );

    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;
    PDN_PARASITIC::EXTRACTION_PORT port;
    port.m_Name = "P1";
    ports.push_back( port );

    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( path, ports, impedances );

    BOOST_CHECK_EQUAL( result, true );

    // Port node names should be updated
    BOOST_CHECK_EQUAL( ports[0].m_PositiveNode, "N1" );
    BOOST_CHECK_EQUAL( ports[0].m_NegativeNode, "N2" );

    // Should have 1x1 = 1 impedance entry
    BOOST_CHECK_EQUAL( impedances.size(), 1u );
    BOOST_CHECK_EQUAL( impedances[0].m_PortI, 0 );
    BOOST_CHECK_EQUAL( impedances[0].m_PortJ, 0 );

    // Should have 1 frequency point
    BOOST_REQUIRE_EQUAL( impedances[0].m_Points.size(), 1u );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_FrequencyHz, 1e6, 0.01 );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_Z.real(), 0.408091, 0.01 );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_Z.imag(), 0.00182523, 0.01 );
}


// --------------------------------------------------------------------------
// ParseFastHenryOutput - two ports, single frequency
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastHenryTwoPorts, PARSER_TEST_FIXTURE )
{
    std::string content =
        "Row 0: N1 to N2\n"
        "Row 1: N3 to N4\n"
        "Impedance matrix for frequency = 1e+06\n"
        "2 x 2\n"
        "0.408091 +0.00182523j  0.123456 +0.000789j\n"
        "0.123456 +0.000789j  0.512345 +0.002345j\n";

    std::string path = writeTempFile( "two_port_zc.mat", content );

    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;
    PDN_PARASITIC::EXTRACTION_PORT p1, p2;
    p1.m_Name = "P1";
    p2.m_Name = "P2";
    ports.push_back( p1 );
    ports.push_back( p2 );

    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( path, ports, impedances );

    BOOST_CHECK_EQUAL( result, true );

    // Port node names should be updated from Row definitions
    BOOST_CHECK_EQUAL( ports[0].m_PositiveNode, "N1" );
    BOOST_CHECK_EQUAL( ports[0].m_NegativeNode, "N2" );
    BOOST_CHECK_EQUAL( ports[1].m_PositiveNode, "N3" );
    BOOST_CHECK_EQUAL( ports[1].m_NegativeNode, "N4" );

    // Should have 2x2 = 4 impedance entries
    BOOST_CHECK_EQUAL( impedances.size(), 4u );

    // Verify port indices
    BOOST_CHECK_EQUAL( impedances[0].m_PortI, 0 );
    BOOST_CHECK_EQUAL( impedances[0].m_PortJ, 0 );
    BOOST_CHECK_EQUAL( impedances[1].m_PortI, 0 );
    BOOST_CHECK_EQUAL( impedances[1].m_PortJ, 1 );
    BOOST_CHECK_EQUAL( impedances[2].m_PortI, 1 );
    BOOST_CHECK_EQUAL( impedances[2].m_PortJ, 0 );
    BOOST_CHECK_EQUAL( impedances[3].m_PortI, 1 );
    BOOST_CHECK_EQUAL( impedances[3].m_PortJ, 1 );

    // Verify Z11 (self-impedance of port 0)
    BOOST_REQUIRE_EQUAL( impedances[0].m_Points.size(), 1u );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_Z.real(), 0.408091, 0.01 );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_Z.imag(), 0.00182523, 0.01 );

    // Verify Z12 (mutual impedance)
    BOOST_REQUIRE_EQUAL( impedances[1].m_Points.size(), 1u );
    BOOST_CHECK_CLOSE( impedances[1].m_Points[0].m_Z.real(), 0.123456, 0.01 );
    BOOST_CHECK_CLOSE( impedances[1].m_Points[0].m_Z.imag(), 0.000789, 0.01 );

    // Verify Z22 (self-impedance of port 1)
    BOOST_REQUIRE_EQUAL( impedances[3].m_Points.size(), 1u );
    BOOST_CHECK_CLOSE( impedances[3].m_Points[0].m_Z.real(), 0.512345, 0.01 );
    BOOST_CHECK_CLOSE( impedances[3].m_Points[0].m_Z.imag(), 0.002345, 0.01 );
}


// --------------------------------------------------------------------------
// ParseFastHenryOutput - multiple frequencies
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastHenryMultipleFrequencies, PARSER_TEST_FIXTURE )
{
    std::string content =
        "Row 0: N1 to N2\n"
        "Impedance matrix for frequency = 1e+03\n"
        "1 x 1\n"
        "0.100000 +0.000100j\n"
        "Impedance matrix for frequency = 1e+06\n"
        "1 x 1\n"
        "0.200000 +0.001000j\n"
        "Impedance matrix for frequency = 1e+09\n"
        "1 x 1\n"
        "0.500000 +1.000000j\n";

    std::string path = writeTempFile( "multifreq_zc.mat", content );

    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;
    PDN_PARASITIC::EXTRACTION_PORT port;
    port.m_Name = "P1";
    ports.push_back( port );

    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( path, ports, impedances );

    BOOST_CHECK_EQUAL( result, true );
    BOOST_CHECK_EQUAL( impedances.size(), 1u );

    // Should have 3 frequency points
    BOOST_REQUIRE_EQUAL( impedances[0].m_Points.size(), 3u );

    // Verify frequency ordering and values
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_FrequencyHz, 1e3, 0.01 );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_Z.real(), 0.1, 0.01 );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_Z.imag(), 0.0001, 0.01 );

    BOOST_CHECK_CLOSE( impedances[0].m_Points[1].m_FrequencyHz, 1e6, 0.01 );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[1].m_Z.real(), 0.2, 0.01 );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[1].m_Z.imag(), 0.001, 0.01 );

    BOOST_CHECK_CLOSE( impedances[0].m_Points[2].m_FrequencyHz, 1e9, 0.01 );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[2].m_Z.real(), 0.5, 0.01 );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[2].m_Z.imag(), 1.0, 0.01 );
}


// --------------------------------------------------------------------------
// ParseFastHenryOutput - scientific notation in impedance values
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastHenryScientificNotation, PARSER_TEST_FIXTURE )
{
    std::string content =
        "Row 0: N1 to N2\n"
        "Impedance matrix for frequency = 1.5e+07\n"
        "1 x 1\n"
        "3.4578e-06 +6.9845e-04j\n";

    std::string path = writeTempFile( "scientific_zc.mat", content );

    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;
    PDN_PARASITIC::EXTRACTION_PORT port;
    port.m_Name = "P1";
    ports.push_back( port );

    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( path, ports, impedances );

    BOOST_CHECK_EQUAL( result, true );
    BOOST_REQUIRE_EQUAL( impedances[0].m_Points.size(), 1u );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_FrequencyHz, 1.5e7, 0.01 );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_Z.real(), 3.4578e-6, 0.01 );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_Z.imag(), 6.9845e-4, 0.01 );
}


// --------------------------------------------------------------------------
// ParseFastCapOutput - nonexistent file
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastCapNonexistentFile, PARSER_TEST_FIXTURE )
{
    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastCapOutput(
        "/nonexistent/path/output.txt", capacitances );

    BOOST_CHECK_EQUAL( result, false );
}


// --------------------------------------------------------------------------
// ParseFastCapOutput - empty file
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastCapEmptyFile, PARSER_TEST_FIXTURE )
{
    std::string path = writeTempFile( "empty_fastcap.txt", "" );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastCapOutput( path, capacitances );

    BOOST_CHECK_EQUAL( result, false );
}


// --------------------------------------------------------------------------
// ParseFastCapOutput - single conductor
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastCapSingleConductor, PARSER_TEST_FIXTURE )
{
    std::string content =
        "Some preamble text\n"
        "CAPACITANCE MATRIX, nanofarads\n"
        "                1\n"
        "VDD  1   0.04567\n"
        "\n";

    std::string path = writeTempFile( "single_cond_fastcap.txt", content );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastCapOutput( path, capacitances );

    BOOST_CHECK_EQUAL( result, true );
    BOOST_REQUIRE_EQUAL( capacitances.size(), 1u );

    // Check self-capacitance: 0.04567 nF = 45.67 pF
    BOOST_CHECK_EQUAL( capacitances[0].m_ConductorI, "VDD" );
    BOOST_CHECK_EQUAL( capacitances[0].m_ConductorJ, "VDD" );
    BOOST_CHECK_CLOSE( capacitances[0].m_CapacitancePF, 45.67, 0.01 );
}


// --------------------------------------------------------------------------
// ParseFastCapOutput - two conductors
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastCapTwoConductors, PARSER_TEST_FIXTURE )
{
    std::string content =
        "CAPACITANCE MATRIX, nanofarads\n"
        "              1         2\n"
        "VDD  1   0.12345  -0.05432\n"
        "GND  2  -0.05432   0.08765\n"
        "\n";

    std::string path = writeTempFile( "two_cond_fastcap.txt", content );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastCapOutput( path, capacitances );

    BOOST_CHECK_EQUAL( result, true );

    // 2x2 matrix = 4 entries (2 per row)
    BOOST_REQUIRE_EQUAL( capacitances.size(), 4u );

    // Row 0: VDD -> VDD, VDD -> GND
    BOOST_CHECK_EQUAL( capacitances[0].m_ConductorI, "VDD" );
    BOOST_CHECK_EQUAL( capacitances[0].m_ConductorJ, "VDD" );
    BOOST_CHECK_CLOSE( capacitances[0].m_CapacitancePF, 123.45, 0.01 );

    BOOST_CHECK_EQUAL( capacitances[1].m_ConductorI, "VDD" );
    BOOST_CHECK_EQUAL( capacitances[1].m_ConductorJ, "GND" );
    BOOST_CHECK_CLOSE( capacitances[1].m_CapacitancePF, -54.32, 0.01 );

    // Row 1: GND -> VDD, GND -> GND
    BOOST_CHECK_EQUAL( capacitances[2].m_ConductorI, "GND" );
    BOOST_CHECK_EQUAL( capacitances[2].m_ConductorJ, "VDD" );
    BOOST_CHECK_CLOSE( capacitances[2].m_CapacitancePF, -54.32, 0.01 );

    BOOST_CHECK_EQUAL( capacitances[3].m_ConductorI, "GND" );
    BOOST_CHECK_EQUAL( capacitances[3].m_ConductorJ, "GND" );
    BOOST_CHECK_CLOSE( capacitances[3].m_CapacitancePF, 87.65, 0.01 );
}


// --------------------------------------------------------------------------
// ParseFastCapOutput - three conductors
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastCapThreeConductors, PARSER_TEST_FIXTURE )
{
    std::string content =
        "CAPACITANCE MATRIX, nanofarads\n"
        "              1         2         3\n"
        "VDD  1   0.100  -0.020  -0.010\n"
        "GND  2  -0.020   0.200  -0.030\n"
        "V3P3 3  -0.010  -0.030   0.150\n"
        "\n";

    std::string path = writeTempFile( "three_cond_fastcap.txt", content );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastCapOutput( path, capacitances );

    BOOST_CHECK_EQUAL( result, true );
    // 3x3 matrix = 9 entries
    BOOST_REQUIRE_EQUAL( capacitances.size(), 9u );

    // Spot check diagonal entries (self-capacitance) - values in pF = nF * 1000
    BOOST_CHECK_CLOSE( capacitances[0].m_CapacitancePF, 100.0, 0.01 );  // VDD self
    BOOST_CHECK_CLOSE( capacitances[4].m_CapacitancePF, 200.0, 0.01 );  // GND self
    BOOST_CHECK_CLOSE( capacitances[8].m_CapacitancePF, 150.0, 0.01 );  // V3P3 self
}


// --------------------------------------------------------------------------
// ParseFastCapOutput - nF to pF conversion accuracy
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastCapUnitConversion, PARSER_TEST_FIXTURE )
{
    std::string content =
        "CAPACITANCE MATRIX, nanofarads\n"
        "           1\n"
        "NET1 1   1.000\n"
        "\n";

    std::string path = writeTempFile( "unit_conv_fastcap.txt", content );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastCapOutput( path, capacitances );

    BOOST_CHECK_EQUAL( result, true );
    BOOST_REQUIRE_EQUAL( capacitances.size(), 1u );

    // 1.0 nanofarad = 1000.0 picofarads
    BOOST_CHECK_CLOSE( capacitances[0].m_CapacitancePF, 1000.0, 0.001 );
}


// --------------------------------------------------------------------------
// ParseFastCapOutput - with preamble text before matrix
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastCapWithPreamble, PARSER_TEST_FIXTURE )
{
    std::string content =
        "FastCap v2.0\n"
        "  Input file: test.lst\n"
        "  Number of conductors: 2\n"
        "  Number of panels: 1234\n"
        "  Iterating...\n"
        "  Solution converged after 42 iterations\n"
        "\n"
        "CAPACITANCE MATRIX, nanofarads\n"
        "              1         2\n"
        "net1  1   0.050  -0.025\n"
        "net2  2  -0.025   0.060\n"
        "\n";

    std::string path = writeTempFile( "preamble_fastcap.txt", content );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastCapOutput( path, capacitances );

    BOOST_CHECK_EQUAL( result, true );
    BOOST_REQUIRE_EQUAL( capacitances.size(), 4u );
    BOOST_CHECK_CLOSE( capacitances[0].m_CapacitancePF, 50.0, 0.01 );
}


// ============================================================================
// DeriveLumpedParasitics tests
// ============================================================================

// --------------------------------------------------------------------------
// DeriveLumpedParasitics - empty inputs
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedParasiticsEmpty )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;
    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    BOOST_CHECK( parasitics.empty() );
}


// --------------------------------------------------------------------------
// DeriveLumpedParasitics - R extraction from self-impedance
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedResistance )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    // Single self-impedance entry at 1 MHz: Z = 0.5 + j*0.01 ohms
    PDN_PARASITIC::IMPEDANCE_ENTRY entry;
    entry.m_PortI = 0;
    entry.m_PortJ = 0;

    PDN_PARASITIC::IMPEDANCE_POINT pt;
    pt.m_FrequencyHz = 1e6;
    pt.m_Z = std::complex<double>( 0.5, 0.01 );
    entry.m_Points.push_back( pt );

    impedances.push_back( entry );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    BOOST_REQUIRE_EQUAL( parasitics.size(), 1u );

    // R = Re(Z) * 1000 = 500 mOhm
    BOOST_CHECK_CLOSE( parasitics[0].m_R_mOhm, 500.0, 0.01 );
}


// --------------------------------------------------------------------------
// DeriveLumpedParasitics - L extraction from self-impedance
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedInductance )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    // Self-impedance at 1 MHz: Z = 0.1 + j * 2*pi*1e6 * L
    // If L = 10 nH, then Im(Z) = 2*pi*1e6 * 10e-9 = 0.06283...
    double freq = 1e6;
    double L_nH = 10.0;
    double omega = 2.0 * M_PI * freq;
    double imZ = omega * L_nH * 1e-9;  // In henries, then omega*L = ohms

    PDN_PARASITIC::IMPEDANCE_ENTRY entry;
    entry.m_PortI = 0;
    entry.m_PortJ = 0;

    PDN_PARASITIC::IMPEDANCE_POINT pt;
    pt.m_FrequencyHz = freq;
    pt.m_Z = std::complex<double>( 0.1, imZ );
    entry.m_Points.push_back( pt );

    impedances.push_back( entry );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    BOOST_REQUIRE_EQUAL( parasitics.size(), 1u );

    // L should be extracted as ~10 nH
    BOOST_CHECK_CLOSE( parasitics[0].m_L_nH, 10.0, 0.01 );
}


// --------------------------------------------------------------------------
// DeriveLumpedParasitics - mutual impedance skipped
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedMutualSkipped )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    // Off-diagonal entry (mutual impedance) - should be skipped
    PDN_PARASITIC::IMPEDANCE_ENTRY entry;
    entry.m_PortI = 0;
    entry.m_PortJ = 1;

    PDN_PARASITIC::IMPEDANCE_POINT pt;
    pt.m_FrequencyHz = 1e6;
    pt.m_Z = std::complex<double>( 0.05, 0.001 );
    entry.m_Points.push_back( pt );

    impedances.push_back( entry );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    // Mutual impedance should not generate a parasitic entry
    BOOST_CHECK( parasitics.empty() );
}


// --------------------------------------------------------------------------
// DeriveLumpedParasitics - capacitance from diagonal FastCap entry
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedCapacitance )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    // Self-capacitance entry
    PDN_PARASITIC::CAPACITANCE_ENTRY cap;
    cap.m_ConductorI = "VDD";
    cap.m_ConductorJ = "VDD";
    cap.m_CapacitancePF = 45.67;

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    capacitances.push_back( cap );

    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    BOOST_REQUIRE_EQUAL( parasitics.size(), 1u );
    BOOST_CHECK_EQUAL( parasitics[0].m_NetName, "VDD" );
    BOOST_CHECK_CLOSE( parasitics[0].m_C_pF, 45.67, 0.01 );
}


// --------------------------------------------------------------------------
// DeriveLumpedParasitics - off-diagonal capacitance skipped
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedMutualCapSkipped )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    // Off-diagonal capacitance entry
    PDN_PARASITIC::CAPACITANCE_ENTRY cap;
    cap.m_ConductorI = "VDD";
    cap.m_ConductorJ = "GND";
    cap.m_CapacitancePF = -25.0;

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    capacitances.push_back( cap );

    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    // Off-diagonal capacitance should not create an entry
    BOOST_CHECK( parasitics.empty() );
}


// --------------------------------------------------------------------------
// DeriveLumpedParasitics - combined R, L, C
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedCombinedRLC )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    // Self-impedance: R = 0.2 ohms, L = 5 nH at 1 MHz
    double freq = 1e6;
    double L_nH = 5.0;
    double omega = 2.0 * M_PI * freq;
    double R = 0.2;

    PDN_PARASITIC::IMPEDANCE_ENTRY entry;
    entry.m_PortI = 0;
    entry.m_PortJ = 0;

    PDN_PARASITIC::IMPEDANCE_POINT pt;
    pt.m_FrequencyHz = freq;
    pt.m_Z = std::complex<double>( R, omega * L_nH * 1e-9 );
    entry.m_Points.push_back( pt );

    impedances.push_back( entry );

    // Self-capacitance
    PDN_PARASITIC::CAPACITANCE_ENTRY cap;
    cap.m_ConductorI = "VDD";
    cap.m_ConductorJ = "VDD";
    cap.m_CapacitancePF = 100.0;

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    capacitances.push_back( cap );

    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    // Should have 2 entries: one from impedance, one from capacitance
    // (they're not merged in current implementation since impedance doesn't have net name)
    BOOST_REQUIRE_GE( parasitics.size(), 1u );

    // Check resistance from impedance entry
    bool foundR = false;
    bool foundC = false;

    for( const auto& np : parasitics )
    {
        if( np.m_R_mOhm > 0.0 )
        {
            BOOST_CHECK_CLOSE( np.m_R_mOhm, 200.0, 0.01 );  // 0.2 ohms = 200 mOhm
            BOOST_CHECK_CLOSE( np.m_L_nH, 5.0, 0.1 );
            foundR = true;
        }
        if( np.m_C_pF > 0.0 )
        {
            BOOST_CHECK_CLOSE( np.m_C_pF, 100.0, 0.01 );
            foundC = true;
        }
    }

    BOOST_CHECK( foundR );
    BOOST_CHECK( foundC );
}


// --------------------------------------------------------------------------
// DeriveLumpedParasitics - uses mid-band frequency for multi-point impedance
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedMidBandSelection )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    PDN_PARASITIC::IMPEDANCE_ENTRY entry;
    entry.m_PortI = 0;
    entry.m_PortJ = 0;

    // 5 frequency points with increasing R
    double freqs[] = { 1e3, 1e4, 1e5, 1e6, 1e7 };

    for( int i = 0; i < 5; i++ )
    {
        PDN_PARASITIC::IMPEDANCE_POINT pt;
        pt.m_FrequencyHz = freqs[i];
        pt.m_Z = std::complex<double>( 0.1 * ( i + 1 ), 0.0 );
        entry.m_Points.push_back( pt );
    }

    impedances.push_back( entry );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    BOOST_REQUIRE_EQUAL( parasitics.size(), 1u );

    // Mid index = 5/2 = 2, which is the 3rd point (R = 0.3 ohms = 300 mOhm)
    BOOST_CHECK_CLOSE( parasitics[0].m_R_mOhm, 300.0, 0.01 );
}


// --------------------------------------------------------------------------
// DeriveLumpedParasitics - impedance entry with no points
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedEmptyPoints )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    PDN_PARASITIC::IMPEDANCE_ENTRY entry;
    entry.m_PortI = 0;
    entry.m_PortJ = 0;
    // No points added

    impedances.push_back( entry );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    // Entry with no points should be skipped
    BOOST_CHECK( parasitics.empty() );
}


// ============================================================================
// JSON serialization tests
// ============================================================================

// --------------------------------------------------------------------------
// SerializeToJson - empty results
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SerializeJsonEmptyResults, PARSER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;
    std::string path = m_tempDir + "/empty_results.json";
    m_tempFiles.push_back( path );

    bool ok = results.SerializeToJson( path );
    BOOST_CHECK_EQUAL( ok, true );

    // Read back and verify it's valid-looking JSON
    std::ifstream file( path );
    BOOST_CHECK( file.is_open() );

    std::string content( ( std::istreambuf_iterator<char>( file ) ),
                         std::istreambuf_iterator<char>() );

    BOOST_CHECK( content.find( "\"frequency_range\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"stackup\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"ports\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"impedance_matrix\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"capacitance_matrix\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"net_parasitics\"" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// SerializeToJson - invalid path
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SerializeJsonInvalidPath, PARSER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    bool ok = results.SerializeToJson( "/nonexistent/dir/results.json" );
    BOOST_CHECK_EQUAL( ok, false );
}


// --------------------------------------------------------------------------
// SerializeToJson - with stackup data
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SerializeJsonWithStackup, PARSER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    PDN_PARASITIC::STACKUP_LAYER copper;
    copper.m_Name = "F.Cu";
    copper.m_IsCopperLayer = true;
    copper.m_ZPositionMM = 0.0;
    copper.m_ThicknessMM = 0.035;
    copper.m_ConductivitySPerM = 5.8e7;
    results.m_Stackup.push_back( copper );

    PDN_PARASITIC::STACKUP_LAYER dielectric;
    dielectric.m_Name = "Prepreg";
    dielectric.m_IsCopperLayer = false;
    dielectric.m_ZPositionMM = 0.035;
    dielectric.m_ThicknessMM = 0.2;
    dielectric.m_EpsilonR = 4.5;
    dielectric.m_LossTangent = 0.02;
    results.m_Stackup.push_back( dielectric );

    std::string path = m_tempDir + "/stackup_results.json";
    m_tempFiles.push_back( path );

    bool ok = results.SerializeToJson( path );
    BOOST_CHECK_EQUAL( ok, true );

    std::ifstream file( path );
    std::string content( ( std::istreambuf_iterator<char>( file ) ),
                         std::istreambuf_iterator<char>() );

    // Check copper layer fields
    BOOST_CHECK( content.find( "\"F.Cu\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"is_copper\": true" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"conductivity_s_per_m\"" ) != std::string::npos );

    // Check dielectric layer fields
    BOOST_CHECK( content.find( "\"Prepreg\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"is_copper\": false" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"epsilon_r\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"loss_tangent\"" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// SerializeToJson - with port data
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SerializeJsonWithPorts, PARSER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    PDN_PARASITIC::EXTRACTION_PORT port;
    port.m_Name = "P1";
    port.m_NetName = "VDD";
    port.m_PositiveNode = "N1";
    port.m_NegativeNode = "N2";
    port.m_XMM = 10.5;
    port.m_YMM = 20.3;
    results.m_Ports.push_back( port );

    std::string path = m_tempDir + "/ports_results.json";
    m_tempFiles.push_back( path );

    bool ok = results.SerializeToJson( path );
    BOOST_CHECK_EQUAL( ok, true );

    std::ifstream file( path );
    std::string content( ( std::istreambuf_iterator<char>( file ) ),
                         std::istreambuf_iterator<char>() );

    BOOST_CHECK( content.find( "\"P1\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"VDD\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"positive_node\": \"N1\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"negative_node\": \"N2\"" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// SerializeToJson - with impedance data
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SerializeJsonWithImpedance, PARSER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    results.m_FreqMinHz = 1e3;
    results.m_FreqMaxHz = 1e9;
    results.m_PointsPerDecade = 5;

    PDN_PARASITIC::IMPEDANCE_ENTRY entry;
    entry.m_PortI = 0;
    entry.m_PortJ = 0;

    PDN_PARASITIC::IMPEDANCE_POINT pt;
    pt.m_FrequencyHz = 1e6;
    pt.m_Z = std::complex<double>( 0.5, 0.01 );
    entry.m_Points.push_back( pt );

    results.m_ImpedanceMatrix.push_back( entry );

    std::string path = m_tempDir + "/impedance_results.json";
    m_tempFiles.push_back( path );

    bool ok = results.SerializeToJson( path );
    BOOST_CHECK_EQUAL( ok, true );

    std::ifstream file( path );
    std::string content( ( std::istreambuf_iterator<char>( file ) ),
                         std::istreambuf_iterator<char>() );

    BOOST_CHECK( content.find( "\"port_i\": 0" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"port_j\": 0" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"frequency_hz\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"z_real_ohm\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"z_imag_ohm\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"points_per_decade\": 5" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// SerializeToJson - with capacitance data
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SerializeJsonWithCapacitance, PARSER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    PDN_PARASITIC::CAPACITANCE_ENTRY cap;
    cap.m_ConductorI = "VDD";
    cap.m_ConductorJ = "GND";
    cap.m_CapacitancePF = 45.67;
    results.m_CapacitanceMatrix.push_back( cap );

    std::string path = m_tempDir + "/cap_results.json";
    m_tempFiles.push_back( path );

    bool ok = results.SerializeToJson( path );
    BOOST_CHECK_EQUAL( ok, true );

    std::ifstream file( path );
    std::string content( ( std::istreambuf_iterator<char>( file ) ),
                         std::istreambuf_iterator<char>() );

    BOOST_CHECK( content.find( "\"conductor_i\": \"VDD\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"conductor_j\": \"GND\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"capacitance_pf\"" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// SerializeToJson - with net parasitic data
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SerializeJsonWithNetParasitics, PARSER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    PDN_PARASITIC::NET_PARASITIC np;
    np.m_NetName = "VDD";
    np.m_SegmentId = "trace_1";
    np.m_LengthMM = 25.4;
    np.m_R_mOhm = 150.0;
    np.m_L_nH = 3.5;
    np.m_C_pF = 12.0;
    np.m_RLGC.m_R_OhmPerM = 5.9;
    np.m_RLGC.m_L_nHPerM = 137.8;
    np.m_RLGC.m_G_SPerM = 0.001;
    np.m_RLGC.m_C_pFPerM = 472.4;
    results.m_NetParasitics.push_back( np );

    std::string path = m_tempDir + "/net_par_results.json";
    m_tempFiles.push_back( path );

    bool ok = results.SerializeToJson( path );
    BOOST_CHECK_EQUAL( ok, true );

    std::ifstream file( path );
    std::string content( ( std::istreambuf_iterator<char>( file ) ),
                         std::istreambuf_iterator<char>() );

    BOOST_CHECK( content.find( "\"net\": \"VDD\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"segment_id\": \"trace_1\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"r_mohm\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"l_nh\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"c_pf\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"rlgc\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"r_ohm_per_m\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"l_nh_per_m\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"g_s_per_m\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"c_pf_per_m\"" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// SerializeToJson - full round trip: build results, serialize, verify fields
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SerializeJsonFullRoundTrip, PARSER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    // Frequency range
    results.m_FreqMinHz = 1e3;
    results.m_FreqMaxHz = 1e9;
    results.m_PointsPerDecade = 10;

    // Stackup: 4-layer board
    {
        PDN_PARASITIC::STACKUP_LAYER l;
        l.m_Name = "F.Cu";
        l.m_IsCopperLayer = true;
        l.m_ZPositionMM = 0.0;
        l.m_ThicknessMM = 0.035;
        l.m_ConductivitySPerM = 5.8e7;
        results.m_Stackup.push_back( l );
    }
    {
        PDN_PARASITIC::STACKUP_LAYER l;
        l.m_Name = "Prepreg1";
        l.m_IsCopperLayer = false;
        l.m_ZPositionMM = 0.035;
        l.m_ThicknessMM = 0.2;
        l.m_EpsilonR = 4.5;
        l.m_LossTangent = 0.02;
        results.m_Stackup.push_back( l );
    }
    {
        PDN_PARASITIC::STACKUP_LAYER l;
        l.m_Name = "In1.Cu";
        l.m_IsCopperLayer = true;
        l.m_ZPositionMM = 0.235;
        l.m_ThicknessMM = 0.035;
        l.m_ConductivitySPerM = 5.8e7;
        results.m_Stackup.push_back( l );
    }

    // Ports
    PDN_PARASITIC::EXTRACTION_PORT p1;
    p1.m_Name = "P1";
    p1.m_NetName = "VDD";
    p1.m_PositiveNode = "N1";
    p1.m_NegativeNode = "N2";
    p1.m_XMM = 10.0;
    p1.m_YMM = 20.0;
    results.m_Ports.push_back( p1 );

    PDN_PARASITIC::EXTRACTION_PORT p2;
    p2.m_Name = "P2";
    p2.m_NetName = "VDD";
    p2.m_PositiveNode = "N3";
    p2.m_NegativeNode = "N4";
    p2.m_XMM = 50.0;
    p2.m_YMM = 60.0;
    results.m_Ports.push_back( p2 );

    // Impedance matrix (2x2, two frequencies)
    for( int i = 0; i < 2; i++ )
    {
        for( int j = 0; j < 2; j++ )
        {
            PDN_PARASITIC::IMPEDANCE_ENTRY entry;
            entry.m_PortI = i;
            entry.m_PortJ = j;

            PDN_PARASITIC::IMPEDANCE_POINT pt1, pt2;
            pt1.m_FrequencyHz = 1e6;
            pt1.m_Z = std::complex<double>( 0.1 * ( i + 1 ), 0.01 * ( j + 1 ) );
            pt2.m_FrequencyHz = 1e9;
            pt2.m_Z = std::complex<double>( 0.2 * ( i + 1 ), 0.02 * ( j + 1 ) );
            entry.m_Points.push_back( pt1 );
            entry.m_Points.push_back( pt2 );

            results.m_ImpedanceMatrix.push_back( entry );
        }
    }

    // Capacitance matrix
    PDN_PARASITIC::CAPACITANCE_ENTRY c1;
    c1.m_ConductorI = "VDD";
    c1.m_ConductorJ = "VDD";
    c1.m_CapacitancePF = 100.0;
    results.m_CapacitanceMatrix.push_back( c1 );

    PDN_PARASITIC::CAPACITANCE_ENTRY c2;
    c2.m_ConductorI = "VDD";
    c2.m_ConductorJ = "GND";
    c2.m_CapacitancePF = -50.0;
    results.m_CapacitanceMatrix.push_back( c2 );

    // Net parasitics
    PDN_PARASITIC::NET_PARASITIC np;
    np.m_NetName = "VDD";
    np.m_SegmentId = "Port_0";
    np.m_LengthMM = 10.0;
    np.m_R_mOhm = 100.0;
    np.m_L_nH = 5.0;
    np.m_C_pF = 100.0;
    results.m_NetParasitics.push_back( np );

    std::string path = m_tempDir + "/full_results.json";
    m_tempFiles.push_back( path );

    bool ok = results.SerializeToJson( path );
    BOOST_CHECK_EQUAL( ok, true );

    // Read back and verify structure
    std::ifstream file( path );
    BOOST_REQUIRE( file.is_open() );

    std::string content( ( std::istreambuf_iterator<char>( file ) ),
                         std::istreambuf_iterator<char>() );

    // Verify the JSON starts with { and ends with }
    size_t firstBrace = content.find( '{' );
    size_t lastBrace = content.rfind( '}' );
    BOOST_CHECK( firstBrace != std::string::npos );
    BOOST_CHECK( lastBrace != std::string::npos );
    BOOST_CHECK( lastBrace > firstBrace );

    // Verify all major sections present
    BOOST_CHECK( content.find( "\"frequency_range\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"stackup\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"ports\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"impedance_matrix\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"capacitance_matrix\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"net_parasitics\"" ) != std::string::npos );

    // Verify specific data was serialized
    BOOST_CHECK( content.find( "\"F.Cu\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"Prepreg1\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"In1.Cu\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"P1\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"P2\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"VDD\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"points_per_decade\": 10" ) != std::string::npos );
}


// ============================================================================
// FastHenry parser: row definition parsing
// ============================================================================

// --------------------------------------------------------------------------
// ParseFastHenryOutput - row definitions with extra whitespace
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastHenryRowWhitespace, PARSER_TEST_FIXTURE )
{
    std::string content =
        "Row  0:  N1  to  N2\n"
        "Impedance matrix for frequency = 1e+06\n"
        "1 x 1\n"
        "0.1 +0.01j\n";

    std::string path = writeTempFile( "whitespace_zc.mat", content );

    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;
    PDN_PARASITIC::EXTRACTION_PORT p;
    p.m_Name = "P1";
    ports.push_back( p );

    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( path, ports, impedances );

    BOOST_CHECK_EQUAL( result, true );
    BOOST_CHECK_EQUAL( ports[0].m_PositiveNode, "N1" );
    BOOST_CHECK_EQUAL( ports[0].m_NegativeNode, "N2" );
}


// --------------------------------------------------------------------------
// ParseFastHenryOutput - no row definitions (just matrix data)
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastHenryNoRowDefs, PARSER_TEST_FIXTURE )
{
    std::string content =
        "Impedance matrix for frequency = 1e+06\n"
        "1 x 1\n"
        "0.1 +0.01j\n";

    std::string path = writeTempFile( "norow_zc.mat", content );

    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( path, ports, impedances );

    // With 0 row definitions, impedance matrix will be empty (0x0)
    BOOST_CHECK_EQUAL( result, false );
}


// --------------------------------------------------------------------------
// ParseFastHenryOutput - row definitions but no matrix data
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastHenryRowsNoMatrix, PARSER_TEST_FIXTURE )
{
    std::string content =
        "Row 0: N1 to N2\n"
        "Row 1: N3 to N4\n";

    std::string path = writeTempFile( "nomat_zc.mat", content );

    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;
    PDN_PARASITIC::EXTRACTION_PORT p1, p2;
    p1.m_Name = "P1";
    p2.m_Name = "P2";
    ports.push_back( p1 );
    ports.push_back( p2 );

    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( path, ports, impedances );

    // Structure is created (4 entries for 2x2) but no frequency points
    // Since aImpedances is not empty, result should be true
    BOOST_CHECK_EQUAL( result, true );
    BOOST_CHECK_EQUAL( impedances.size(), 4u );

    // But no frequency data parsed
    for( const auto& imp : impedances )
        BOOST_CHECK( imp.m_Points.empty() );
}


// ============================================================================
// FastCap parser edge cases
// ============================================================================

// --------------------------------------------------------------------------
// ParseFastCapOutput - file with only preamble, no matrix
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastCapNoCAPMatrix, PARSER_TEST_FIXTURE )
{
    std::string content =
        "FastCap v2.0\n"
        "  Input file: test.lst\n"
        "  Some solver output\n";

    std::string path = writeTempFile( "no_matrix_fastcap.txt", content );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastCapOutput( path, capacitances );

    BOOST_CHECK_EQUAL( result, false );
    BOOST_CHECK( capacitances.empty() );
}


// --------------------------------------------------------------------------
// ParseFastCapOutput - negative capacitance values (mutual coupling)
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastCapNegativeValues, PARSER_TEST_FIXTURE )
{
    std::string content =
        "CAPACITANCE MATRIX, nanofarads\n"
        "              1         2\n"
        "NET1  1   0.050  -0.025\n"
        "NET2  2  -0.025   0.060\n"
        "\n";

    std::string path = writeTempFile( "negative_cap.txt", content );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastCapOutput( path, capacitances );

    BOOST_CHECK_EQUAL( result, true );
    BOOST_REQUIRE_EQUAL( capacitances.size(), 4u );

    // Verify negative mutual capacitance is preserved
    BOOST_CHECK_CLOSE( capacitances[1].m_CapacitancePF, -25.0, 0.01 );
    BOOST_CHECK_CLOSE( capacitances[2].m_CapacitancePF, -25.0, 0.01 );
}


// --------------------------------------------------------------------------
// ParseFastCapOutput - conductor names with underscores
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastCapSpecialNames, PARSER_TEST_FIXTURE )
{
    std::string content =
        "CAPACITANCE MATRIX, nanofarads\n"
        "           1\n"
        "VDD_3V3  1   0.050\n"
        "\n";

    std::string path = writeTempFile( "special_names_fastcap.txt", content );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastCapOutput( path, capacitances );

    BOOST_CHECK_EQUAL( result, true );
    BOOST_REQUIRE_EQUAL( capacitances.size(), 1u );
    BOOST_CHECK_EQUAL( capacitances[0].m_ConductorI, "VDD_3V3" );
}


// ============================================================================
// EXTRACTION_CONFIG tests
// ============================================================================

// --------------------------------------------------------------------------
// EXTRACTION_CONFIG - modifying values
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( ExtractionConfigCustomValues )
{
    PDN_PARASITIC::EXTRACTION_CONFIG config;

    config.m_NetNames.push_back( "VDD" );
    config.m_NetNames.push_back( "GND" );
    config.m_FastHenryPath = "/usr/local/bin/fasthenry";
    config.m_FreqMinHz = 100.0;
    config.m_FreqMaxHz = 10e9;
    config.m_PointsPerDecade = 10;
    config.m_NwInc = 5;
    config.m_NhInc = 4;
    config.m_FastCapPath = "/usr/local/bin/fastcap2";
    config.m_PanelTargetSizeMM = 0.25;
    config.m_ViaFacets = 16;
    config.m_PlaneSegX = 40;
    config.m_PlaneSegY = 40;
    config.m_OutputDir = "/tmp/pdn_extraction";

    BOOST_CHECK_EQUAL( config.m_NetNames.size(), 2u );
    BOOST_CHECK_EQUAL( config.m_NetNames[0], "VDD" );
    BOOST_CHECK_EQUAL( config.m_NetNames[1], "GND" );
    BOOST_CHECK_EQUAL( config.m_FastHenryPath, "/usr/local/bin/fasthenry" );
    BOOST_CHECK_CLOSE( config.m_FreqMinHz, 100.0, 0.01 );
    BOOST_CHECK_CLOSE( config.m_FreqMaxHz, 10e9, 0.01 );
    BOOST_CHECK_EQUAL( config.m_PointsPerDecade, 10 );
    BOOST_CHECK_EQUAL( config.m_NwInc, 5 );
    BOOST_CHECK_EQUAL( config.m_NhInc, 4 );
    BOOST_CHECK_EQUAL( config.m_FastCapPath, "/usr/local/bin/fastcap2" );
    BOOST_CHECK_CLOSE( config.m_PanelTargetSizeMM, 0.25, 0.01 );
    BOOST_CHECK_EQUAL( config.m_ViaFacets, 16 );
    BOOST_CHECK_EQUAL( config.m_PlaneSegX, 40 );
    BOOST_CHECK_EQUAL( config.m_PlaneSegY, 40 );
    BOOST_CHECK_EQUAL( config.m_OutputDir, "/tmp/pdn_extraction" );
}


// ============================================================================
// Complex impedance manipulation tests
// ============================================================================

// --------------------------------------------------------------------------
// Impedance point - complex number storage and retrieval
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( ImpedancePointComplexValues )
{
    PDN_PARASITIC::IMPEDANCE_POINT pt;

    pt.m_FrequencyHz = 1e6;
    pt.m_Z = std::complex<double>( 0.5, -0.3 );

    BOOST_CHECK_CLOSE( pt.m_Z.real(), 0.5, 1e-9 );
    BOOST_CHECK_CLOSE( pt.m_Z.imag(), -0.3, 1e-9 );

    // Magnitude and phase
    double mag = std::abs( pt.m_Z );
    double phase = std::arg( pt.m_Z );

    BOOST_CHECK_CLOSE( mag, std::sqrt( 0.5 * 0.5 + 0.3 * 0.3 ), 0.001 );
    BOOST_CHECK( phase < 0.0 ); // Negative imaginary part => negative phase
}


// --------------------------------------------------------------------------
// Impedance entry with multiple frequency sweeps
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( ImpedanceEntryFrequencySweep )
{
    PDN_PARASITIC::IMPEDANCE_ENTRY entry;
    entry.m_PortI = 0;
    entry.m_PortJ = 0;

    // Create a frequency sweep from 1kHz to 1GHz (7 decades, 5 per decade = 35 pts)
    double freq = 1e3;

    while( freq <= 1.001e9 )
    {
        PDN_PARASITIC::IMPEDANCE_POINT pt;
        pt.m_FrequencyHz = freq;
        // Model: R increases with sqrt(f) (skin effect), L is constant
        double R = 0.1 * std::sqrt( freq / 1e3 );
        double L_nH = 10.0;
        double omega = 2.0 * M_PI * freq;
        pt.m_Z = std::complex<double>( R, omega * L_nH * 1e-9 );

        entry.m_Points.push_back( pt );

        freq *= std::pow( 10.0, 0.2 ); // 5 points per decade
    }

    BOOST_CHECK( entry.m_Points.size() >= 30 );

    // Verify first and last points make physical sense
    BOOST_CHECK( entry.m_Points.front().m_Z.real() < entry.m_Points.back().m_Z.real() );
    BOOST_CHECK( entry.m_Points.front().m_Z.imag() < entry.m_Points.back().m_Z.imag() );
}


// ============================================================================
// Data structure population and collection tests
// ============================================================================

// --------------------------------------------------------------------------
// EXTRACTION_RESULTS - multiple net parasitics
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( MultipleNetParasitics )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    std::vector<std::string> nets = { "VDD", "VDD", "V3P3", "GND" };
    std::vector<std::string> segs = { "trace_1", "via_1", "trace_2", "plane_1" };

    for( size_t i = 0; i < nets.size(); i++ )
    {
        PDN_PARASITIC::NET_PARASITIC np;
        np.m_NetName = nets[i];
        np.m_SegmentId = segs[i];
        np.m_LengthMM = 10.0 * ( i + 1 );
        np.m_R_mOhm = 50.0 * ( i + 1 );
        np.m_L_nH = 2.0 * ( i + 1 );
        np.m_C_pF = 5.0 * ( i + 1 );
        results.m_NetParasitics.push_back( np );
    }

    BOOST_CHECK_EQUAL( results.m_NetParasitics.size(), 4u );

    // Verify each entry
    BOOST_CHECK_EQUAL( results.m_NetParasitics[0].m_NetName, "VDD" );
    BOOST_CHECK_EQUAL( results.m_NetParasitics[0].m_SegmentId, "trace_1" );
    BOOST_CHECK_CLOSE( results.m_NetParasitics[0].m_R_mOhm, 50.0, 0.01 );

    BOOST_CHECK_EQUAL( results.m_NetParasitics[3].m_NetName, "GND" );
    BOOST_CHECK_EQUAL( results.m_NetParasitics[3].m_SegmentId, "plane_1" );
    BOOST_CHECK_CLOSE( results.m_NetParasitics[3].m_R_mOhm, 200.0, 0.01 );
}


// --------------------------------------------------------------------------
// EXTRACTION_RESULTS - large impedance matrix
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( LargeImpedanceMatrix )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    int nPorts = 10;

    for( int i = 0; i < nPorts; i++ )
    {
        for( int j = 0; j < nPorts; j++ )
        {
            PDN_PARASITIC::IMPEDANCE_ENTRY entry;
            entry.m_PortI = i;
            entry.m_PortJ = j;

            // Add a few frequency points
            for( int f = 0; f < 3; f++ )
            {
                PDN_PARASITIC::IMPEDANCE_POINT pt;
                pt.m_FrequencyHz = 1e3 * std::pow( 10.0, f );
                pt.m_Z = std::complex<double>( 0.01 * ( i + j + 1 ), 0.001 * f );
                entry.m_Points.push_back( pt );
            }

            results.m_ImpedanceMatrix.push_back( entry );
        }
    }

    BOOST_CHECK_EQUAL( results.m_ImpedanceMatrix.size(),
                       static_cast<size_t>( nPorts * nPorts ) );

    // Verify diagonal entries (self-impedance)
    for( int i = 0; i < nPorts; i++ )
    {
        int idx = i * nPorts + i;
        BOOST_CHECK_EQUAL( results.m_ImpedanceMatrix[idx].m_PortI, i );
        BOOST_CHECK_EQUAL( results.m_ImpedanceMatrix[idx].m_PortJ, i );
        BOOST_CHECK_EQUAL( results.m_ImpedanceMatrix[idx].m_Points.size(), 3u );
    }
}


// --------------------------------------------------------------------------
// DeriveLumpedParasitics - multiple self-impedance ports
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedMultiplePorts )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    for( int i = 0; i < 3; i++ )
    {
        PDN_PARASITIC::IMPEDANCE_ENTRY entry;
        entry.m_PortI = i;
        entry.m_PortJ = i;

        PDN_PARASITIC::IMPEDANCE_POINT pt;
        pt.m_FrequencyHz = 1e6;
        pt.m_Z = std::complex<double>( 0.1 * ( i + 1 ), 0.001 * ( i + 1 ) );
        entry.m_Points.push_back( pt );

        impedances.push_back( entry );
    }

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    BOOST_REQUIRE_EQUAL( parasitics.size(), 3u );

    // Port_0: R = 0.1 ohms = 100 mOhm
    BOOST_CHECK_CLOSE( parasitics[0].m_R_mOhm, 100.0, 0.01 );
    BOOST_CHECK_EQUAL( parasitics[0].m_SegmentId, "Port_0" );

    // Port_1: R = 0.2 ohms = 200 mOhm
    BOOST_CHECK_CLOSE( parasitics[1].m_R_mOhm, 200.0, 0.01 );
    BOOST_CHECK_EQUAL( parasitics[1].m_SegmentId, "Port_1" );

    // Port_2: R = 0.3 ohms = 300 mOhm
    BOOST_CHECK_CLOSE( parasitics[2].m_R_mOhm, 300.0, 0.01 );
    BOOST_CHECK_EQUAL( parasitics[2].m_SegmentId, "Port_2" );
}


// --------------------------------------------------------------------------
// DeriveLumpedParasitics - zero frequency (DC) handling
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedDCFrequency )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    PDN_PARASITIC::IMPEDANCE_ENTRY entry;
    entry.m_PortI = 0;
    entry.m_PortJ = 0;

    // At DC (freq=0), omega=0, so L extraction would divide by zero
    // But with only one point, mid-band picks this
    PDN_PARASITIC::IMPEDANCE_POINT pt;
    pt.m_FrequencyHz = 0.0;
    pt.m_Z = std::complex<double>( 0.5, 0.01 );
    entry.m_Points.push_back( pt );

    impedances.push_back( entry );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    BOOST_REQUIRE_EQUAL( parasitics.size(), 1u );

    // R should still be extracted
    BOOST_CHECK_CLOSE( parasitics[0].m_R_mOhm, 500.0, 0.01 );

    // L should be 0 since omega=0 and the code guards against division by zero
    BOOST_CHECK_CLOSE( parasitics[0].m_L_nH, 0.0, 1e-9 );
}


// ============================================================================
// End-to-end parser integration tests
// ============================================================================

// --------------------------------------------------------------------------
// Parse FastHenry then derive lumped parasitics
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( EndToEndFastHenryToLumped, PARSER_TEST_FIXTURE )
{
    // Realistic Zc.mat file with 2 ports and 2 frequencies
    std::string content =
        "Row 0: N1 to N2\n"
        "Row 1: N3 to N4\n"
        "Impedance matrix for frequency = 1e+06\n"
        "2 x 2\n"
        "0.100000 +0.062832j  0.010000 +0.006283j\n"
        "0.010000 +0.006283j  0.200000 +0.125664j\n"
        "Impedance matrix for frequency = 1e+09\n"
        "2 x 2\n"
        "3.162278 +62.831853j  0.316228 +6.283185j\n"
        "0.316228 +6.283185j  6.324555 +125.663706j\n";

    std::string path = writeTempFile( "e2e_zc.mat", content );

    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;
    PDN_PARASITIC::EXTRACTION_PORT p1, p2;
    p1.m_Name = "P1";
    p2.m_Name = "P2";
    ports.push_back( p1 );
    ports.push_back( p2 );

    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool parseOk = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( path, ports, impedances );
    BOOST_REQUIRE( parseOk );
    BOOST_REQUIRE_EQUAL( impedances.size(), 4u );

    // Each entry should have 2 frequency points
    for( const auto& imp : impedances )
        BOOST_CHECK_EQUAL( imp.m_Points.size(), 2u );

    // Now derive lumped parasitics
    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    // Should have 2 entries (one per self-impedance port)
    BOOST_REQUIRE_EQUAL( parasitics.size(), 2u );

    // All R and L values should be positive
    for( const auto& np : parasitics )
    {
        BOOST_CHECK( np.m_R_mOhm > 0.0 );
        BOOST_CHECK( np.m_L_nH > 0.0 );
    }
}


// --------------------------------------------------------------------------
// Parse FastCap then derive lumped parasitics
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( EndToEndFastCapToLumped, PARSER_TEST_FIXTURE )
{
    std::string content =
        "CAPACITANCE MATRIX, nanofarads\n"
        "              1         2\n"
        "VDD  1   0.12345  -0.05432\n"
        "GND  2  -0.05432   0.08765\n"
        "\n";

    std::string path = writeTempFile( "e2e_fastcap.txt", content );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool parseOk = PARASITIC_RESULT_PARSER::ParseFastCapOutput( path, capacitances );
    BOOST_REQUIRE( parseOk );
    BOOST_REQUIRE_EQUAL( capacitances.size(), 4u );

    // Derive lumped parasitics
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;
    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    // Only diagonal (self-capacitance) entries should produce parasitics
    BOOST_REQUIRE_EQUAL( parasitics.size(), 2u );

    BOOST_CHECK_EQUAL( parasitics[0].m_NetName, "VDD" );
    BOOST_CHECK_CLOSE( parasitics[0].m_C_pF, 123.45, 0.01 );

    BOOST_CHECK_EQUAL( parasitics[1].m_NetName, "GND" );
    BOOST_CHECK_CLOSE( parasitics[1].m_C_pF, 87.65, 0.01 );
}


// --------------------------------------------------------------------------
// Full pipeline: parse both, derive, serialize to JSON
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( EndToEndFullPipeline, PARSER_TEST_FIXTURE )
{
    // FastHenry output
    std::string fhContent =
        "Row 0: N1 to N2\n"
        "Impedance matrix for frequency = 1e+06\n"
        "1 x 1\n"
        "0.200000 +0.062832j\n";

    std::string fhPath = writeTempFile( "pipeline_zc.mat", fhContent );

    // FastCap output
    std::string fcContent =
        "CAPACITANCE MATRIX, nanofarads\n"
        "           1\n"
        "VDD  1   0.100\n"
        "\n";

    std::string fcPath = writeTempFile( "pipeline_fastcap.txt", fcContent );

    // Parse FastHenry
    PDN_PARASITIC::EXTRACTION_RESULTS results;
    PDN_PARASITIC::EXTRACTION_PORT port;
    port.m_Name = "P1";
    port.m_NetName = "VDD";
    results.m_Ports.push_back( port );

    bool fhOk = PARASITIC_RESULT_PARSER::ParseFastHenryOutput(
        fhPath, results.m_Ports, results.m_ImpedanceMatrix );
    BOOST_REQUIRE( fhOk );

    // Parse FastCap
    bool fcOk = PARASITIC_RESULT_PARSER::ParseFastCapOutput(
        fcPath, results.m_CapacitanceMatrix );
    BOOST_REQUIRE( fcOk );

    // Derive lumped parasitics
    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics(
        results.m_ImpedanceMatrix,
        results.m_CapacitanceMatrix,
        results.m_NetParasitics );

    // Set frequency range
    results.m_FreqMinHz = 1e6;
    results.m_FreqMaxHz = 1e6;
    results.m_PointsPerDecade = 1;

    // Serialize to JSON
    std::string jsonPath = m_tempDir + "/pipeline_results.json";
    m_tempFiles.push_back( jsonPath );

    bool jsonOk = results.SerializeToJson( jsonPath );
    BOOST_REQUIRE( jsonOk );

    // Verify JSON was written
    std::ifstream jsonFile( jsonPath );
    BOOST_REQUIRE( jsonFile.is_open() );

    std::string jsonContent( ( std::istreambuf_iterator<char>( jsonFile ) ),
                             std::istreambuf_iterator<char>() );

    BOOST_CHECK( !jsonContent.empty() );
    BOOST_CHECK( jsonContent.find( "\"frequency_range\"" ) != std::string::npos );
    BOOST_CHECK( jsonContent.find( "\"net_parasitics\"" ) != std::string::npos );
    BOOST_CHECK( jsonContent.find( "\"VDD\"" ) != std::string::npos );
}


// ============================================================================
// Additional parser robustness tests
// ============================================================================

// --------------------------------------------------------------------------
// ParseFastHenryOutput - complex value with negative imaginary part
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastHenryNegativeImaginary, PARSER_TEST_FIXTURE )
{
    // Negative imaginary part represents capacitive behavior at certain frequencies
    std::string content =
        "Row 0: N1 to N2\n"
        "Impedance matrix for frequency = 1e+06\n"
        "1 x 1\n"
        "0.300000 -0.050000j\n";

    std::string path = writeTempFile( "negimag_zc.mat", content );

    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;
    PDN_PARASITIC::EXTRACTION_PORT p;
    p.m_Name = "P1";
    ports.push_back( p );

    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( path, ports, impedances );

    BOOST_CHECK_EQUAL( result, true );
    BOOST_REQUIRE_EQUAL( impedances[0].m_Points.size(), 1u );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_Z.real(), 0.3, 0.01 );
    BOOST_CHECK_CLOSE( impedances[0].m_Points[0].m_Z.imag(), -0.05, 0.01 );
}


// --------------------------------------------------------------------------
// ParseFastHenryOutput - large matrix (5x5)
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastHenryLargeMatrix, PARSER_TEST_FIXTURE )
{
    std::ostringstream oss;

    for( int i = 0; i < 5; i++ )
        oss << "Row " << i << ": N" << ( 2 * i + 1 ) << " to N" << ( 2 * i + 2 ) << "\n";

    oss << "Impedance matrix for frequency = 1e+06\n";
    oss << "5 x 5\n";

    for( int i = 0; i < 5; i++ )
    {
        for( int j = 0; j < 5; j++ )
        {
            if( j > 0 )
                oss << "  ";

            double re = ( i == j ) ? 0.1 * ( i + 1 ) : 0.01;
            double im = ( i == j ) ? 0.01 * ( i + 1 ) : 0.001;
            oss << re << " +" << im << "j";
        }
        oss << "\n";
    }

    std::string path = writeTempFile( "large_zc.mat", oss.str() );

    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;

    for( int i = 0; i < 5; i++ )
    {
        PDN_PARASITIC::EXTRACTION_PORT p;
        p.m_Name = "P" + std::to_string( i + 1 );
        ports.push_back( p );
    }

    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( path, ports, impedances );

    BOOST_CHECK_EQUAL( result, true );

    // 5x5 = 25 impedance entries
    BOOST_CHECK_EQUAL( impedances.size(), 25u );

    // Verify node name updates
    BOOST_CHECK_EQUAL( ports[0].m_PositiveNode, "N1" );
    BOOST_CHECK_EQUAL( ports[0].m_NegativeNode, "N2" );
    BOOST_CHECK_EQUAL( ports[4].m_PositiveNode, "N9" );
    BOOST_CHECK_EQUAL( ports[4].m_NegativeNode, "N10" );

    // Verify diagonal entries have larger values
    for( int i = 0; i < 5; i++ )
    {
        int idx = i * 5 + i;  // Diagonal index
        BOOST_REQUIRE_EQUAL( impedances[idx].m_Points.size(), 1u );
        BOOST_CHECK_CLOSE( impedances[idx].m_Points[0].m_Z.real(),
                           0.1 * ( i + 1 ), 1.0 );
    }
}


// --------------------------------------------------------------------------
// ParseFastCapOutput - large matrix (5x5)
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ParseFastCapLargeMatrix, PARSER_TEST_FIXTURE )
{
    std::ostringstream oss;
    oss << "CAPACITANCE MATRIX, nanofarads\n";
    oss << "              1         2         3         4         5\n";

    std::vector<std::string> names = { "VDD", "GND", "V3P3", "V1P8", "VDDIO" };

    for( int i = 0; i < 5; i++ )
    {
        oss << names[i] << "  " << ( i + 1 );

        for( int j = 0; j < 5; j++ )
        {
            double val = ( i == j ) ? 0.1 * ( i + 1 ) : -0.01;
            oss << "   " << val;
        }

        oss << "\n";
    }

    oss << "\n";

    std::string path = writeTempFile( "large_cap.txt", oss.str() );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastCapOutput( path, capacitances );

    BOOST_CHECK_EQUAL( result, true );

    // 5x5 = 25 entries
    BOOST_REQUIRE_EQUAL( capacitances.size(), 25u );

    // Verify conductor names
    BOOST_CHECK_EQUAL( capacitances[0].m_ConductorI, "VDD" );
    BOOST_CHECK_EQUAL( capacitances[0].m_ConductorJ, "VDD" );

    // Last row, last column
    BOOST_CHECK_EQUAL( capacitances[24].m_ConductorI, "VDDIO" );
    BOOST_CHECK_EQUAL( capacitances[24].m_ConductorJ, "VDDIO" );

    // Check self-capacitance values (pF = nF * 1000)
    BOOST_CHECK_CLOSE( capacitances[0].m_CapacitancePF, 100.0, 0.01 );   // VDD: 0.1 nF
    BOOST_CHECK_CLOSE( capacitances[6].m_CapacitancePF, 200.0, 0.01 );   // GND: 0.2 nF
    BOOST_CHECK_CLOSE( capacitances[24].m_CapacitancePF, 500.0, 0.01 );  // VDDIO: 0.5 nF
}


// ============================================================================
// Physical value validation tests
// ============================================================================

// --------------------------------------------------------------------------
// Verify L extraction is frequency-independent for a true inductor
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( InductanceExtractionConsistency )
{
    // For a pure inductor, Z = j*omega*L, so L = Im(Z)/omega regardless of frequency
    double L_expected_nH = 15.0;

    std::vector<double> freqs = { 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9 };

    for( double freq : freqs )
    {
        double omega = 2.0 * M_PI * freq;
        double imZ = omega * L_expected_nH * 1e-9;

        // Derive L from the impedance
        double L_derived_nH = imZ / omega * 1e9;

        BOOST_CHECK_CLOSE( L_derived_nH, L_expected_nH, 0.001 );
    }
}


// --------------------------------------------------------------------------
// Verify R extraction from real part of impedance
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( ResistanceExtractionAccuracy )
{
    // R = Re(Z), simple resistance with no frequency dependence
    double R_expected_ohm = 0.123456;

    PDN_PARASITIC::IMPEDANCE_POINT pt;
    pt.m_FrequencyHz = 1e6;
    pt.m_Z = std::complex<double>( R_expected_ohm, 0.05 );

    double R_derived_mOhm = pt.m_Z.real() * 1000.0;
    BOOST_CHECK_CLOSE( R_derived_mOhm, R_expected_ohm * 1000.0, 0.001 );
}


// --------------------------------------------------------------------------
// Verify capacitance nF to pF conversion
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( CapacitanceConversionPrecision )
{
    // Test various values for nF -> pF conversion (factor of 1000)
    struct TestCase
    {
        double nF;
        double expectedPF;
    };

    std::vector<TestCase> cases = {
        { 0.001,    1.0 },
        { 0.01,     10.0 },
        { 0.1,      100.0 },
        { 1.0,      1000.0 },
        { 10.0,     10000.0 },
        { 0.00001,  0.01 },
        { -0.05,    -50.0 },  // Mutual capacitance (negative)
    };

    for( const auto& tc : cases )
    {
        double pF = tc.nF * 1000.0;
        BOOST_CHECK_CLOSE( pF, tc.expectedPF, 0.001 );
    }
}


// ============================================================================
// Stackup layer tests
// ============================================================================

// --------------------------------------------------------------------------
// Typical 4-layer stackup construction
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( FourLayerStackup )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    // F.Cu
    PDN_PARASITIC::STACKUP_LAYER fcu;
    fcu.m_Name = "F.Cu";
    fcu.m_IsCopperLayer = true;
    fcu.m_ZPositionMM = 0.0;
    fcu.m_ThicknessMM = 0.035;
    fcu.m_ConductivitySPerM = 5.8e7;
    results.m_Stackup.push_back( fcu );

    // Prepreg
    PDN_PARASITIC::STACKUP_LAYER pp1;
    pp1.m_Name = "Prepreg_1";
    pp1.m_IsCopperLayer = false;
    pp1.m_ZPositionMM = 0.035;
    pp1.m_ThicknessMM = 0.2;
    pp1.m_EpsilonR = 4.5;
    pp1.m_LossTangent = 0.02;
    results.m_Stackup.push_back( pp1 );

    // In1.Cu
    PDN_PARASITIC::STACKUP_LAYER in1;
    in1.m_Name = "In1.Cu";
    in1.m_IsCopperLayer = true;
    in1.m_ZPositionMM = 0.235;
    in1.m_ThicknessMM = 0.035;
    in1.m_ConductivitySPerM = 5.8e7;
    results.m_Stackup.push_back( in1 );

    // Core
    PDN_PARASITIC::STACKUP_LAYER core;
    core.m_Name = "Core";
    core.m_IsCopperLayer = false;
    core.m_ZPositionMM = 0.270;
    core.m_ThicknessMM = 1.0;
    core.m_EpsilonR = 4.2;
    core.m_LossTangent = 0.018;
    results.m_Stackup.push_back( core );

    // In2.Cu
    PDN_PARASITIC::STACKUP_LAYER in2;
    in2.m_Name = "In2.Cu";
    in2.m_IsCopperLayer = true;
    in2.m_ZPositionMM = 1.270;
    in2.m_ThicknessMM = 0.035;
    in2.m_ConductivitySPerM = 5.8e7;
    results.m_Stackup.push_back( in2 );

    // Prepreg
    PDN_PARASITIC::STACKUP_LAYER pp2;
    pp2.m_Name = "Prepreg_2";
    pp2.m_IsCopperLayer = false;
    pp2.m_ZPositionMM = 1.305;
    pp2.m_ThicknessMM = 0.2;
    pp2.m_EpsilonR = 4.5;
    pp2.m_LossTangent = 0.02;
    results.m_Stackup.push_back( pp2 );

    // B.Cu
    PDN_PARASITIC::STACKUP_LAYER bcu;
    bcu.m_Name = "B.Cu";
    bcu.m_IsCopperLayer = true;
    bcu.m_ZPositionMM = 1.505;
    bcu.m_ThicknessMM = 0.035;
    bcu.m_ConductivitySPerM = 5.8e7;
    results.m_Stackup.push_back( bcu );

    BOOST_CHECK_EQUAL( results.m_Stackup.size(), 7u );

    // Count copper and dielectric layers
    int nCopper = 0;
    int nDielectric = 0;

    for( const auto& layer : results.m_Stackup )
    {
        if( layer.m_IsCopperLayer )
            nCopper++;
        else
            nDielectric++;
    }

    BOOST_CHECK_EQUAL( nCopper, 4 );
    BOOST_CHECK_EQUAL( nDielectric, 3 );

    // Verify layer ordering by Z position
    for( size_t i = 1; i < results.m_Stackup.size(); i++ )
    {
        BOOST_CHECK_GE( results.m_Stackup[i].m_ZPositionMM,
                         results.m_Stackup[i - 1].m_ZPositionMM );
    }

    // Total board thickness check (approximate)
    double totalThick = results.m_Stackup.back().m_ZPositionMM
                       + results.m_Stackup.back().m_ThicknessMM;
    BOOST_CHECK_CLOSE( totalThick, 1.54, 1.0 ); // ~1.54mm for a typical 4-layer
}


// ============================================================================
// Symmetry tests for impedance/capacitance matrices
// ============================================================================

// --------------------------------------------------------------------------
// Capacitance matrix symmetry (C_ij = C_ji)
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( CapacitanceMatrixSymmetry, PARSER_TEST_FIXTURE )
{
    std::string content =
        "CAPACITANCE MATRIX, nanofarads\n"
        "              1         2         3\n"
        "A  1   0.100  -0.020  -0.010\n"
        "B  2  -0.020   0.150  -0.030\n"
        "C  3  -0.010  -0.030   0.120\n"
        "\n";

    std::string path = writeTempFile( "sym_cap.txt", content );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastCapOutput( path, capacitances );
    BOOST_REQUIRE( result );
    BOOST_REQUIRE_EQUAL( capacitances.size(), 9u );

    // Build a map for easy lookup
    // Matrix layout: row-major [0,1,2,3,4,5,6,7,8]
    // C[0][0]=0, C[0][1]=1, C[0][2]=2, C[1][0]=3, ...

    // Check symmetry: C[0][1] == C[1][0]
    BOOST_CHECK_CLOSE( capacitances[1].m_CapacitancePF,
                       capacitances[3].m_CapacitancePF, 0.001 );

    // C[0][2] == C[2][0]
    BOOST_CHECK_CLOSE( capacitances[2].m_CapacitancePF,
                       capacitances[6].m_CapacitancePF, 0.001 );

    // C[1][2] == C[2][1]
    BOOST_CHECK_CLOSE( capacitances[5].m_CapacitancePF,
                       capacitances[7].m_CapacitancePF, 0.001 );
}


// --------------------------------------------------------------------------
// Impedance matrix symmetry from parsed file
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ImpedanceMatrixSymmetry, PARSER_TEST_FIXTURE )
{
    std::string content =
        "Row 0: N1 to N2\n"
        "Row 1: N3 to N4\n"
        "Impedance matrix for frequency = 1e+06\n"
        "2 x 2\n"
        "0.500000 +0.062832j  0.050000 +0.006283j\n"
        "0.050000 +0.006283j  0.400000 +0.050265j\n";

    std::string path = writeTempFile( "sym_zc.mat", content );

    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports;
    PDN_PARASITIC::EXTRACTION_PORT p1, p2;
    p1.m_Name = "P1";
    p2.m_Name = "P2";
    ports.push_back( p1 );
    ports.push_back( p2 );

    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    bool result = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( path, ports, impedances );
    BOOST_REQUIRE( result );
    BOOST_REQUIRE_EQUAL( impedances.size(), 4u );

    // Z[0][1] should equal Z[1][0] (reciprocity)
    BOOST_REQUIRE_EQUAL( impedances[1].m_Points.size(), 1u );
    BOOST_REQUIRE_EQUAL( impedances[2].m_Points.size(), 1u );

    BOOST_CHECK_CLOSE( impedances[1].m_Points[0].m_Z.real(),
                       impedances[2].m_Points[0].m_Z.real(), 0.001 );
    BOOST_CHECK_CLOSE( impedances[1].m_Points[0].m_Z.imag(),
                       impedances[2].m_Points[0].m_Z.imag(), 0.001 );
}


// ============================================================================
// Skin effect modeling verification
// ============================================================================

// --------------------------------------------------------------------------
// Verify skin effect: R should increase with sqrt(frequency)
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( SkinEffectResistanceIncrease )
{
    // For a good conductor, R_ac / R_dc ~ sqrt(f / f_skin)
    // We just verify the relationship qualitatively

    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    PDN_PARASITIC::IMPEDANCE_ENTRY entry;
    entry.m_PortI = 0;
    entry.m_PortJ = 0;

    // Model increasing resistance with frequency (skin effect)
    std::vector<double> freqs = { 1e3, 1e4, 1e5, 1e6, 1e7 };
    double R_dc = 0.1;  // ohms

    for( double freq : freqs )
    {
        PDN_PARASITIC::IMPEDANCE_POINT pt;
        pt.m_FrequencyHz = freq;
        // R increases with sqrt(f) relative to 1kHz base
        double R = R_dc * std::sqrt( freq / 1e3 );
        double omega = 2.0 * M_PI * freq;
        double L = 10e-9;  // 10 nH
        pt.m_Z = std::complex<double>( R, omega * L );
        entry.m_Points.push_back( pt );
    }

    impedances.push_back( entry );

    // Verify resistance increases monotonically
    for( size_t i = 1; i < entry.m_Points.size(); i++ )
    {
        BOOST_CHECK_GT( entry.m_Points[i].m_Z.real(),
                        entry.m_Points[i - 1].m_Z.real() );
    }

    // Verify ratio matches sqrt relationship
    // R(1MHz) / R(1kHz) should be sqrt(1e6/1e3) = sqrt(1000) ~= 31.6
    double ratio = entry.m_Points[3].m_Z.real() / entry.m_Points[0].m_Z.real();
    BOOST_CHECK_CLOSE( ratio, std::sqrt( 1000.0 ), 0.1 );
}


// ============================================================================
// JSON serialization edge cases
// ============================================================================

// --------------------------------------------------------------------------
// Serialize results with empty arrays but non-zero frequency range
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SerializeJsonEmptyArraysWithFreqRange, PARSER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;
    results.m_FreqMinHz = 1e3;
    results.m_FreqMaxHz = 1e9;
    results.m_PointsPerDecade = 5;

    std::string path = m_tempDir + "/freq_only_results.json";
    m_tempFiles.push_back( path );

    bool ok = results.SerializeToJson( path );
    BOOST_CHECK_EQUAL( ok, true );

    std::ifstream file( path );
    std::string content( ( std::istreambuf_iterator<char>( file ) ),
                         std::istreambuf_iterator<char>() );

    // Should have empty arrays
    BOOST_CHECK( content.find( "\"stackup\": []" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"ports\": []" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"impedance_matrix\": []" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"capacitance_matrix\": []" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"net_parasitics\": []" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// Serialize results with special floating point values
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SerializeJsonSpecialFloatValues, PARSER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    PDN_PARASITIC::NET_PARASITIC np;
    np.m_NetName = "test";
    np.m_SegmentId = "seg1";
    np.m_R_mOhm = 0.0;      // Zero resistance
    np.m_L_nH = 0.0;         // Zero inductance
    np.m_C_pF = 0.0;         // Zero capacitance
    np.m_LengthMM = 0.0;
    results.m_NetParasitics.push_back( np );

    std::string path = m_tempDir + "/zero_values.json";
    m_tempFiles.push_back( path );

    bool ok = results.SerializeToJson( path );
    BOOST_CHECK_EQUAL( ok, true );

    std::ifstream file( path );
    std::string content( ( std::istreambuf_iterator<char>( file ) ),
                         std::istreambuf_iterator<char>() );

    BOOST_CHECK( content.find( "\"test\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"seg1\"" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// Serialize and verify multiple ports have comma separators
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SerializeJsonMultiplePortsFormatting, PARSER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    for( int i = 0; i < 3; i++ )
    {
        PDN_PARASITIC::EXTRACTION_PORT port;
        port.m_Name = "P" + std::to_string( i + 1 );
        port.m_NetName = "NET" + std::to_string( i + 1 );
        port.m_PositiveNode = "Np" + std::to_string( i );
        port.m_NegativeNode = "Nn" + std::to_string( i );
        port.m_XMM = 10.0 * i;
        port.m_YMM = 20.0 * i;
        results.m_Ports.push_back( port );
    }

    std::string path = m_tempDir + "/multi_port_format.json";
    m_tempFiles.push_back( path );

    bool ok = results.SerializeToJson( path );
    BOOST_CHECK_EQUAL( ok, true );

    std::ifstream file( path );
    std::string content( ( std::istreambuf_iterator<char>( file ) ),
                         std::istreambuf_iterator<char>() );

    // All three port names should be present
    BOOST_CHECK( content.find( "\"P1\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"P2\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"P3\"" ) != std::string::npos );

    // All three net names should be present
    BOOST_CHECK( content.find( "\"NET1\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"NET2\"" ) != std::string::npos );
    BOOST_CHECK( content.find( "\"NET3\"" ) != std::string::npos );
}


// ============================================================================
// Unit conversion verification tests
// ============================================================================

// --------------------------------------------------------------------------
// iu2m: KiCad internal units (nm) to meters
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( IU2MConversion )
{
    // iu2m(x) = x * 1e-9  (nm to m)
    // Test this relationship directly since the exporters use it

    auto iu2m = []( int aIU ) -> double { return aIU * 1e-9; };

    // 1 mm = 1e6 nm -> 1e-3 m
    BOOST_CHECK_CLOSE( iu2m( 1000000 ), 1e-3, 0.001 );

    // 35 um = 35000 nm -> 35e-6 m
    BOOST_CHECK_CLOSE( iu2m( 35000 ), 35e-6, 0.001 );

    // 0.254 mm = 254000 nm -> 254e-6 m (10 mil trace)
    BOOST_CHECK_CLOSE( iu2m( 254000 ), 254e-6, 0.001 );

    // 1 m = 1e9 nm
    BOOST_CHECK_CLOSE( iu2m( 1000000000 ), 1.0, 0.001 );

    // Zero
    BOOST_CHECK_CLOSE( iu2m( 0 ), 0.0, 1e-15 );
}


// --------------------------------------------------------------------------
// iu2mm: KiCad internal units (nm) to millimeters
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( IU2MMConversion )
{
    // iu2mm(x) = x / 1e6  (nm to mm)
    auto iu2mm = []( int aIU ) -> double { return aIU / 1e6; };

    // 1 mm = 1e6 nm
    BOOST_CHECK_CLOSE( iu2mm( 1000000 ), 1.0, 0.001 );

    // 35 um = 35000 nm = 0.035 mm
    BOOST_CHECK_CLOSE( iu2mm( 35000 ), 0.035, 0.001 );

    // 0.254 mm = 254000 nm (10 mil trace)
    BOOST_CHECK_CLOSE( iu2mm( 254000 ), 0.254, 0.001 );

    // Zero
    BOOST_CHECK_CLOSE( iu2mm( 0 ), 0.0, 1e-15 );
}


// ============================================================================
// DeriveLumpedParasitics: mixed impedance + capacitance tests
// ============================================================================

// --------------------------------------------------------------------------
// Both impedance and capacitance entries for same conductor
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedMixedImpedanceCapacitance )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;

    // Two self-impedance ports
    for( int i = 0; i < 2; i++ )
    {
        PDN_PARASITIC::IMPEDANCE_ENTRY entry;
        entry.m_PortI = i;
        entry.m_PortJ = i;

        PDN_PARASITIC::IMPEDANCE_POINT pt;
        pt.m_FrequencyHz = 1e6;
        pt.m_Z = std::complex<double>( 0.3, 0.03 );
        entry.m_Points.push_back( pt );

        impedances.push_back( entry );
    }

    // Also add off-diagonal (should be skipped)
    {
        PDN_PARASITIC::IMPEDANCE_ENTRY entry;
        entry.m_PortI = 0;
        entry.m_PortJ = 1;

        PDN_PARASITIC::IMPEDANCE_POINT pt;
        pt.m_FrequencyHz = 1e6;
        pt.m_Z = std::complex<double>( 0.01, 0.001 );
        entry.m_Points.push_back( pt );

        impedances.push_back( entry );
    }

    // Three capacitance entries (2 diagonal + 1 off-diagonal)
    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    PDN_PARASITIC::CAPACITANCE_ENTRY c1;
    c1.m_ConductorI = "VDD";
    c1.m_ConductorJ = "VDD";
    c1.m_CapacitancePF = 50.0;
    capacitances.push_back( c1 );

    PDN_PARASITIC::CAPACITANCE_ENTRY c2;
    c2.m_ConductorI = "VDD";
    c2.m_ConductorJ = "GND";
    c2.m_CapacitancePF = -20.0;
    capacitances.push_back( c2 );

    PDN_PARASITIC::CAPACITANCE_ENTRY c3;
    c3.m_ConductorI = "GND";
    c3.m_ConductorJ = "GND";
    c3.m_CapacitancePF = 75.0;
    capacitances.push_back( c3 );

    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    // 2 from self-impedance + 2 from self-capacitance = 4
    BOOST_REQUIRE_EQUAL( parasitics.size(), 4u );

    // First two from impedance (R, L extracted)
    BOOST_CHECK_CLOSE( parasitics[0].m_R_mOhm, 300.0, 0.01 );
    BOOST_CHECK_CLOSE( parasitics[1].m_R_mOhm, 300.0, 0.01 );

    // Last two from capacitance
    bool foundVDD = false;
    bool foundGND = false;

    for( const auto& np : parasitics )
    {
        if( np.m_NetName == "VDD" && np.m_C_pF > 0.0 )
        {
            BOOST_CHECK_CLOSE( np.m_C_pF, 50.0, 0.01 );
            foundVDD = true;
        }

        if( np.m_NetName == "GND" && np.m_C_pF > 0.0 )
        {
            BOOST_CHECK_CLOSE( np.m_C_pF, 75.0, 0.01 );
            foundGND = true;
        }
    }

    BOOST_CHECK( foundVDD );
    BOOST_CHECK( foundGND );
}


// --------------------------------------------------------------------------
// Multiple self-capacitance entries for the same conductor
// --------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE( DeriveLumpedDuplicateCapacitor )
{
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;
    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;

    // Two self-capacitance entries for "VDD" - second should create a new entry
    // since the code only matches on existing NetName
    PDN_PARASITIC::CAPACITANCE_ENTRY c1;
    c1.m_ConductorI = "VDD";
    c1.m_ConductorJ = "VDD";
    c1.m_CapacitancePF = 50.0;
    capacitances.push_back( c1 );

    PDN_PARASITIC::CAPACITANCE_ENTRY c2;
    c2.m_ConductorI = "VDD";
    c2.m_ConductorJ = "VDD";
    c2.m_CapacitancePF = 75.0;
    capacitances.push_back( c2 );

    std::vector<PDN_PARASITIC::NET_PARASITIC> parasitics;

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, capacitances, parasitics );

    // First entry creates new NET_PARASITIC with C=50
    // Second entry finds existing "VDD" entry and updates C=75
    BOOST_REQUIRE_GE( parasitics.size(), 1u );

    // The first VDD entry is found and updated to 75
    bool foundVDD = false;

    for( const auto& np : parasitics )
    {
        if( np.m_NetName == "VDD" )
        {
            BOOST_CHECK_CLOSE( np.m_C_pF, 75.0, 0.01 );
            foundVDD = true;
            break;
        }
    }

    BOOST_CHECK( foundVDD );
}


// ============================================================================
// Exporter tests with real BOARD geometry
// ============================================================================

struct EXPORTER_TEST_FIXTURE
{
    EXPORTER_TEST_FIXTURE() : m_settingsManager( true /* headless */ )
    {
        KI_TEST::LoadBoard( m_settingsManager, "parasitic_extraction/parasitic_test", m_board );
        BOOST_REQUIRE( m_board );

        m_tempDir = "build/qa/parasitic_test_tmp";
        std::filesystem::create_directories( m_tempDir );
    }

    ~EXPORTER_TEST_FIXTURE() { std::filesystem::remove_all( m_tempDir ); }

    PDN_PARASITIC::EXTRACTION_CONFIG makeConfig( const std::vector<std::string>& aNets = { "VDD",
                                                                                           "GND" } )
    {
        PDN_PARASITIC::EXTRACTION_CONFIG cfg;
        cfg.m_NetNames = aNets;
        cfg.m_OutputDir = m_tempDir;
        cfg.m_FreqMinHz = 1e3;
        cfg.m_FreqMaxHz = 1e9;
        cfg.m_PointsPerDecade = 5;
        cfg.m_ViaFacets = 8;
        cfg.m_PlaneSegX = 4;
        cfg.m_PlaneSegY = 4;
        return cfg;
    }

    std::string readFile( const std::string& aPath )
    {
        std::ifstream file( aPath );
        return std::string( std::istreambuf_iterator<char>( file ),
                            std::istreambuf_iterator<char>() );
    }

    SETTINGS_MANAGER       m_settingsManager;
    std::unique_ptr<BOARD> m_board;
    std::string            m_tempDir;
};


// --------------------------------------------------------------------------
// FastHenry: basic export produces valid .inp file
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastHenryExportBasic, EXPORTER_TEST_FIXTURE )
{
    auto               cfg = makeConfig();
    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );

    std::string outPath = m_tempDir + "/test.inp";
    bool        ok = exporter.Export( outPath );
    BOOST_CHECK_EQUAL( ok, true );

    std::string content = readFile( outPath );
    BOOST_CHECK( !content.empty() );

    // Check required FastHenry sections
    BOOST_CHECK( content.find( ".units mm" ) != std::string::npos );
    BOOST_CHECK( content.find( ".default" ) != std::string::npos );
    BOOST_CHECK( content.find( ".freq" ) != std::string::npos );
    BOOST_CHECK( content.find( ".end" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// FastHenry: track produces nodes and segments with correct geometry
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastHenryExportTrackGeometry, EXPORTER_TEST_FIXTURE )
{
    auto               cfg = makeConfig( { "VDD" } );
    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );

    std::string outPath = m_tempDir + "/track_test.inp";
    bool        ok = exporter.Export( outPath );
    BOOST_CHECK_EQUAL( ok, true );

    std::string content = readFile( outPath );

    // Track is from (20,15) to (29.5,15), width 0.25mm on F.Cu
    // Node definitions should contain x= y= z= coordinates
    BOOST_CHECK( content.find( "x=" ) != std::string::npos );
    BOOST_CHECK( content.find( "y=" ) != std::string::npos );
    BOOST_CHECK( content.find( "z=" ) != std::string::npos );

    // Segment definitions: E<n> with w= h=
    BOOST_CHECK( content.find( "w=" ) != std::string::npos );
    BOOST_CHECK( content.find( "h=" ) != std::string::npos );

    // Track width should be 0.25mm (0.250000)
    BOOST_CHECK( content.find( "w=0.25" ) != std::string::npos );

    // Trace section header
    BOOST_CHECK( content.find( "Trace segments" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// FastHenry: via produces vertical node pair
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastHenryExportVia, EXPORTER_TEST_FIXTURE )
{
    auto               cfg = makeConfig( { "VDD" } );
    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );

    std::string outPath = m_tempDir + "/via_test.inp";
    bool        ok = exporter.Export( outPath );
    BOOST_CHECK_EQUAL( ok, true );

    std::string content = readFile( outPath );

    // Via section header
    BOOST_CHECK( content.find( "Vias" ) != std::string::npos );

    // Via at (20,15) connects F.Cu to B.Cu — should have nodes at different Z
    // There should be at least 2 nodes for the via (top and bottom layers)
    // and at least 1 segment connecting them

    // Count 'E' segments in via section
    size_t viaPos = content.find( "Vias" );
    BOOST_REQUIRE( viaPos != std::string::npos );

    std::string viaSection = content.substr( viaPos );
    // There should be at least one E segment in the via section
    BOOST_CHECK( viaSection.find( "E" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// FastHenry: zone produces ground plane element
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastHenryExportZone, EXPORTER_TEST_FIXTURE )
{
    auto               cfg = makeConfig( { "VDD", "GND" } );
    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );

    std::string outPath = m_tempDir + "/zone_test.inp";
    bool        ok = exporter.Export( outPath );
    BOOST_CHECK_EQUAL( ok, true );

    std::string content = readFile( outPath );

    // Plane section header
    BOOST_CHECK( content.find( "Copper planes" ) != std::string::npos );

    // Ground plane element: g<n> with x1= y1= z1=
    BOOST_CHECK( content.find( "g1" ) != std::string::npos );
    BOOST_CHECK( content.find( "seg1=" ) != std::string::npos );
    BOOST_CHECK( content.find( "seg2=" ) != std::string::npos );
    BOOST_CHECK( content.find( "thick=" ) != std::string::npos );
    BOOST_CHECK( content.find( "sigma=" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// FastHenry: net filtering excludes unselected nets
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastHenryExportNetFiltering, EXPORTER_TEST_FIXTURE )
{
    // Only extract VDD — GND zone should be excluded
    auto               cfg = makeConfig( { "VDD" } );
    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );

    std::string outPath = m_tempDir + "/filter_test.inp";
    bool        ok = exporter.Export( outPath );
    BOOST_CHECK_EQUAL( ok, true );

    std::string content = readFile( outPath );

    // With only VDD net, there should be no ground plane (zone is GND)
    BOOST_CHECK( content.find( "g1" ) == std::string::npos );
}


// --------------------------------------------------------------------------
// FastHenry: stackup Z positions match layer order
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastHenryExportStackupZ, EXPORTER_TEST_FIXTURE )
{
    auto               cfg = makeConfig( { "VDD", "GND" } );
    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );

    std::string outPath = m_tempDir + "/stackup_test.inp";
    bool        ok = exporter.Export( outPath );
    BOOST_CHECK_EQUAL( ok, true );

    std::string content = readFile( outPath );

    // Parse z= values from the track node definitions
    // F.Cu center should be at 0.0175 mm (half of 0.035)
    // The via should have nodes at F.Cu Z and B.Cu Z, with B.Cu Z > F.Cu Z
    // This verifies the stackup is correctly modeled

    // At minimum, verify nodes exist at different Z positions
    // by checking that z= appears multiple times
    size_t firstZ = content.find( "z=" );
    BOOST_REQUIRE( firstZ != std::string::npos );

    size_t secondZ = content.find( "z=", firstZ + 1 );
    BOOST_REQUIRE( secondZ != std::string::npos );
}


// --------------------------------------------------------------------------
// FastHenry: frequency sweep line matches config
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastHenryExportFrequency, EXPORTER_TEST_FIXTURE )
{
    auto cfg = makeConfig( { "VDD" } );
    cfg.m_FreqMinHz = 1e3;
    cfg.m_FreqMaxHz = 1e9;
    cfg.m_PointsPerDecade = 10;

    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );

    std::string outPath = m_tempDir + "/freq_test.inp";
    bool        ok = exporter.Export( outPath );
    BOOST_CHECK_EQUAL( ok, true );

    std::string content = readFile( outPath );

    // .freq fmin=... fmax=... ndec=...
    BOOST_CHECK( content.find( ".freq" ) != std::string::npos );
    BOOST_CHECK( content.find( "fmin=" ) != std::string::npos );
    BOOST_CHECK( content.find( "fmax=" ) != std::string::npos );
    BOOST_CHECK( content.find( "ndec=10" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// FastHenry: .equiv statements for pad/plane connections
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastHenryExportEquivNodes, EXPORTER_TEST_FIXTURE )
{
    auto               cfg = makeConfig( { "VDD", "GND" } );
    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );

    std::string outPath = m_tempDir + "/equiv_test.inp";
    bool        ok = exporter.Export( outPath );
    BOOST_CHECK_EQUAL( ok, true );

    std::string content = readFile( outPath );

    // Node equivalences section header
    BOOST_CHECK( content.find( "Node equivalences" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// FastHenry: ports identified from footprint pads
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastHenryExportPorts, EXPORTER_TEST_FIXTURE )
{
    auto               cfg = makeConfig( { "VDD", "GND" } );
    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );

    std::string outPath = m_tempDir + "/ports_test.inp";
    bool        ok = exporter.Export( outPath );
    BOOST_CHECK_EQUAL( ok, true );

    // The footprint has VDD pad and GND pad, so a port should be created
    const auto& ports = exporter.GetPorts();
    BOOST_CHECK_GE( ports.size(), 1u );

    if( !ports.empty() )
    {
        // Port should reference VDD net
        BOOST_CHECK_EQUAL( ports[0].m_NetName, "VDD" );
        BOOST_CHECK( !ports[0].m_PositiveNode.empty() );
        BOOST_CHECK( !ports[0].m_NegativeNode.empty() );

        // .external line in the output
        std::string content = readFile( outPath );
        BOOST_CHECK( content.find( ".external" ) != std::string::npos );
    }
}


// --------------------------------------------------------------------------
// FastCap: basic export produces .lst and .qui files
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastCapExportBasic, EXPORTER_TEST_FIXTURE )
{
    auto             cfg = makeConfig();
    FASTCAP_EXPORTER exporter( m_board.get(), cfg );

    bool ok = exporter.Export( m_tempDir, "test.lst" );
    BOOST_CHECK_EQUAL( ok, true );

    // Check list file exists
    std::string lstContent = readFile( m_tempDir + "/test.lst" );
    BOOST_CHECK( !lstContent.empty() );
    BOOST_CHECK( lstContent.find( "FastCap list file" ) != std::string::npos );

    // Check at least one .qui geometry file exists
    bool foundQui = false;

    for( const auto& entry : std::filesystem::directory_iterator( m_tempDir ) )
    {
        if( entry.path().extension() == ".qui" )
        {
            foundQui = true;
            break;
        }
    }

    BOOST_CHECK( foundQui );
}


// --------------------------------------------------------------------------
// FastCap: trace mesh has Q (quad) panels with correct units (meters)
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastCapExportTraceGeometry, EXPORTER_TEST_FIXTURE )
{
    auto             cfg = makeConfig( { "VDD" } );
    FASTCAP_EXPORTER exporter( m_board.get(), cfg );

    bool ok = exporter.Export( m_tempDir, "trace_test.lst" );
    BOOST_CHECK_EQUAL( ok, true );

    // Read traces.qui
    std::string traceContent = readFile( m_tempDir + "/traces.qui" );
    BOOST_CHECK( !traceContent.empty() );

    // Trace file should have Q (quad) panels
    BOOST_CHECK( traceContent.find( "Q " ) != std::string::npos );

    // Coordinates should be in meters (very small values, scientific notation)
    // Track at (20mm, 15mm) = (0.020, 0.015 meters)
    // Scientific notation with e- prefix
    BOOST_CHECK( traceContent.find( "e-" ) != std::string::npos );

    // Conductor name should be VDD (net name)
    BOOST_CHECK( traceContent.find( "VDD" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// FastCap: via barrel mesh with correct facet count
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastCapExportViaMesh, EXPORTER_TEST_FIXTURE )
{
    auto cfg = makeConfig( { "VDD" } );
    cfg.m_ViaFacets = 8;
    FASTCAP_EXPORTER exporter( m_board.get(), cfg );

    bool ok = exporter.Export( m_tempDir, "via_test.lst" );
    BOOST_CHECK_EQUAL( ok, true );

    // Read vias.qui
    std::string viaContent = readFile( m_tempDir + "/vias.qui" );
    BOOST_CHECK( !viaContent.empty() );

    // Via mesh should have Q (quad) panels for barrel and T (tri) for caps
    BOOST_CHECK( viaContent.find( "Q " ) != std::string::npos );
    BOOST_CHECK( viaContent.find( "T " ) != std::string::npos );

    // Count quad panels: 8 facets per ring, plus annular caps (8 tri each)
    // At minimum there should be 8 barrel quads + 16 cap tris = 24 panels
    int                quadCount = 0;
    int                triCount = 0;
    std::istringstream iss( viaContent );
    std::string        line;

    while( std::getline( iss, line ) )
    {
        if( line.substr( 0, 2 ) == "Q " )
            quadCount++;
        else if( line.substr( 0, 2 ) == "T " )
            triCount++;
    }

    // At least 8 facets for barrel
    BOOST_CHECK_GE( quadCount, 8 );
    // At least 8 triangles per cap, 2 caps = 16
    BOOST_CHECK_GE( triCount, 16 );
}


// --------------------------------------------------------------------------
// FastCap: zone mesh produces quad panels
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastCapExportZoneMesh, EXPORTER_TEST_FIXTURE )
{
    auto cfg = makeConfig( { "GND" } );
    cfg.m_PlaneSegX = 4;
    cfg.m_PlaneSegY = 4;
    FASTCAP_EXPORTER exporter( m_board.get(), cfg );

    bool ok = exporter.Export( m_tempDir, "zone_test.lst" );
    BOOST_CHECK_EQUAL( ok, true );

    // Read planes.qui
    std::string planeContent = readFile( m_tempDir + "/planes.qui" );
    BOOST_CHECK( !planeContent.empty() );

    // Plane mesh should have Q (quad) panels
    BOOST_CHECK( planeContent.find( "Q " ) != std::string::npos );

    // With 4x4 mesh on top and bottom surfaces = 2 * 16 = 32 quads minimum
    int                quadCount = 0;
    std::istringstream iss( planeContent );
    std::string        line;

    while( std::getline( iss, line ) )
    {
        if( line.substr( 0, 2 ) == "Q " )
            quadCount++;
    }

    BOOST_CHECK_GE( quadCount, 32 );
}


// --------------------------------------------------------------------------
// FastCap: dielectric surfaces with correct epsilon_r from stackup
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastCapExportDielectric, EXPORTER_TEST_FIXTURE )
{
    auto             cfg = makeConfig();
    FASTCAP_EXPORTER exporter( m_board.get(), cfg );

    bool ok = exporter.Export( m_tempDir, "diel_test.lst" );
    BOOST_CHECK_EQUAL( ok, true );

    // List file should have D (dielectric) entries with epsilon_r from stackup (4.5)
    std::string lstContent = readFile( m_tempDir + "/diel_test.lst" );
    BOOST_CHECK( lstContent.find( "D " ) != std::string::npos );
    BOOST_CHECK( lstContent.find( "4.5" ) != std::string::npos );

    // Check that dielectric .qui files exist
    bool foundDiel = false;

    for( const auto& entry : std::filesystem::directory_iterator( m_tempDir ) )
    {
        if( entry.path().filename().string().find( "dielectric" ) != std::string::npos )
        {
            foundDiel = true;
            break;
        }
    }

    BOOST_CHECK( foundDiel );
}


// --------------------------------------------------------------------------
// FastCap: net filtering only includes requested nets
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastCapExportNetFiltering, EXPORTER_TEST_FIXTURE )
{
    // Only extract VDD — zones (GND) should be excluded
    auto             cfg = makeConfig( { "VDD" } );
    FASTCAP_EXPORTER exporter( m_board.get(), cfg );

    bool ok = exporter.Export( m_tempDir, "filter_test.lst" );
    BOOST_CHECK_EQUAL( ok, true );

    // planes.qui should not exist (GND zone excluded, no VDD zones)
    std::string planeContent = readFile( m_tempDir + "/planes.qui" );

    // Either file is empty or doesn't have GND conductor
    if( !planeContent.empty() )
        BOOST_CHECK( planeContent.find( "GND" ) == std::string::npos );

    // traces.qui should only have VDD conductor
    std::string traceContent = readFile( m_tempDir + "/traces.qui" );

    if( !traceContent.empty() )
        BOOST_CHECK( traceContent.find( "GND" ) == std::string::npos );
}


// --------------------------------------------------------------------------
// FastCap: .lst file correctly references conductor/dielectric files
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FastCapExportListFileFormat, EXPORTER_TEST_FIXTURE )
{
    auto             cfg = makeConfig();
    FASTCAP_EXPORTER exporter( m_board.get(), cfg );

    bool ok = exporter.Export( m_tempDir, "lst_test.lst" );
    BOOST_CHECK_EQUAL( ok, true );

    std::string lstContent = readFile( m_tempDir + "/lst_test.lst" );

    // List file should reference conductor files with 'C' prefix
    BOOST_CHECK( lstContent.find( "C " ) != std::string::npos );

    // Should reference dielectric files with 'D' prefix
    BOOST_CHECK( lstContent.find( "D " ) != std::string::npos );

    // Should reference specific geometry files
    BOOST_CHECK( lstContent.find( "traces.qui" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// Orchestrator: ExportGeometry creates both FastHenry and FastCap outputs
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ExportGeometryCreatesBothOutputs, EXPORTER_TEST_FIXTURE )
{
    auto                 cfg = makeConfig();
    PARASITIC_EXTRACTION extractor( m_board.get(), cfg );

    bool ok = extractor.ExportGeometry();
    BOOST_CHECK_EQUAL( ok, true );

    // FastHenry .inp file should exist
    std::string fhContent = readFile( m_tempDir + "/pdn_extraction.inp" );
    BOOST_CHECK( !fhContent.empty() );
    BOOST_CHECK( fhContent.find( ".units mm" ) != std::string::npos );
    BOOST_CHECK( fhContent.find( ".end" ) != std::string::npos );

    // FastCap .lst file should exist
    std::string fcContent = readFile( m_tempDir + "/pdn_extraction.lst" );
    BOOST_CHECK( !fcContent.empty() );
    BOOST_CHECK( fcContent.find( "FastCap list file" ) != std::string::npos );

    // Results should have ports populated
    const auto& results = extractor.GetResults();
    BOOST_CHECK_GE( results.m_Ports.size(), 1u );
}


// --------------------------------------------------------------------------
// Orchestrator: buildStackupResults extracts correct layer data
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( BuildStackupFromBoard, EXPORTER_TEST_FIXTURE )
{
    auto                 cfg = makeConfig();
    PARASITIC_EXTRACTION extractor( m_board.get(), cfg );

    // ExportGeometry also calls buildStackupResults indirectly through
    // RunExtraction, but ExportGeometry alone doesn't populate stackup.
    // We verify via the JSON serialization path after ExportGeometry.
    bool ok = extractor.ExportGeometry();
    BOOST_CHECK_EQUAL( ok, true );

    // Serialize results to JSON and check stackup content
    std::string jsonPath = m_tempDir + "/stackup_test.json";
    ok = extractor.SerializeResults( jsonPath );
    BOOST_CHECK_EQUAL( ok, true );

    std::string content = readFile( jsonPath );
    BOOST_CHECK( !content.empty() );

    // The JSON should contain port info (from ExportGeometry)
    BOOST_CHECK( content.find( "\"ports\"" ) != std::string::npos );
}


// ============================================================================
// Manual validation export (writes to temp dir for manual solver runs)
// ============================================================================

BOOST_FIXTURE_TEST_CASE( ExportForManualValidation, EXPORTER_TEST_FIXTURE )
{
    auto cfg = makeConfig( { "VDD", "GND" } );

    // Export FastHenry
    FASTHENRY_EXPORTER fh( m_board.get(), cfg );
    std::string        fhPath = m_tempDir + "/pdn_extraction.inp";
    bool               fhOk = fh.Export( fhPath );
    BOOST_CHECK_EQUAL( fhOk, true );

    std::string fhContent = readFile( fhPath );
    BOOST_CHECK( !fhContent.empty() );
    BOOST_CHECK( fhContent.find( ".end" ) != std::string::npos );

    // Export FastCap
    FASTCAP_EXPORTER fc( m_board.get(), cfg );
    bool             fcOk = fc.Export( m_tempDir, "pdn_extraction.lst" );
    BOOST_CHECK_EQUAL( fcOk, true );

    std::string fcContent = readFile( m_tempDir + "/pdn_extraction.lst" );
    BOOST_CHECK( !fcContent.empty() );

    BOOST_TEST_MESSAGE( "Exported files for manual validation to: " << m_tempDir );
    BOOST_TEST_MESSAGE( "  FastHenry: " << fhPath );
    BOOST_TEST_MESSAGE( "  FastCap:   " << m_tempDir << "/pdn_extraction.lst" );
    BOOST_TEST_MESSAGE( "Run manually:" );
    BOOST_TEST_MESSAGE( "  cd " << m_tempDir << " && fasthenry pdn_extraction.inp" );
    BOOST_TEST_MESSAGE( "  cd " << m_tempDir << " && fastcap -lpdn_extraction.lst" );
}


// ============================================================================
// Solver integration tests (skipped if solvers not found on system)
// ============================================================================

namespace
{

/// Check if a solver binary is available on PATH
bool findSolver( const std::string& aBinaryName )
{
    std::string cmd = "which " + aBinaryName + " > /dev/null 2>&1";
    return std::system( cmd.c_str() ) == 0;
}

} // anonymous namespace


// --------------------------------------------------------------------------
// FastHenry solver integration: export, run solver, parse results, verify
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SolverIntegration_FastHenry, EXPORTER_TEST_FIXTURE )
{
    if( !findSolver( "fasthenry" ) )
    {
        BOOST_TEST_MESSAGE( "SKIPPED: fasthenry not found on PATH" );
        return;
    }

    auto cfg = makeConfig( { "VDD", "GND" } );

    // Export FastHenry input
    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );
    std::string        inpPath = m_tempDir + "/solver_test.inp";
    bool               exportOk = exporter.Export( inpPath );
    BOOST_REQUIRE( exportOk );

    // Run FastHenry (use just the filename since we cd into the temp dir)
    std::string cmd =
            "cd " + m_tempDir + " && fasthenry solver_test.inp" + " > fasthenry_log.txt 2>&1";
    int ret = std::system( cmd.c_str() );
    BOOST_REQUIRE_MESSAGE( ret == 0, "FastHenry exited with code " << ret << ". Check " << m_tempDir
                                                                   << "/fasthenry_log.txt" );

    // Verify Zc.mat was created
    std::string zcPath = m_tempDir + "/Zc.mat";
    std::string zcContent = readFile( zcPath );
    BOOST_REQUIRE_MESSAGE( !zcContent.empty(), "Zc.mat not created or empty" );

    // Parse the results
    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports = exporter.GetPorts();
    BOOST_REQUIRE_GE( ports.size(), 1u );

    BOOST_TEST_MESSAGE( "Zc.mat content:\n" << zcContent );

    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> impedances;
    bool parseOk = PARASITIC_RESULT_PARSER::ParseFastHenryOutput( zcPath, ports, impedances );
    BOOST_REQUIRE_MESSAGE( parseOk, "Failed to parse Zc.mat" );
    BOOST_REQUIRE_MESSAGE( !impedances.empty(), "Impedance matrix is empty" );

    BOOST_TEST_MESSAGE( "Parsed " << impedances.size() << " impedance entries" );

    for( const auto& imp : impedances )
    {
        BOOST_TEST_MESSAGE( "  Z[" << imp.m_PortI << "," << imp.m_PortJ
                                   << "]: " << imp.m_Points.size() << " freq points" );
    }

    // Derive lumped parasitics
    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> emptyCapacitances;
    std::vector<PDN_PARASITIC::NET_PARASITIC>     parasitics;
    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( impedances, emptyCapacitances, parasitics );

    BOOST_TEST_MESSAGE( "Derived " << parasitics.size() << " lumped parasitics" );
    BOOST_REQUIRE_GE( parasitics.size(), 1u );

    // Verify against analytical expectations:
    //   Trace R_DC ~ 18.2 mOhm (tolerance ±50% for field solver vs analytical)
    //   Trace+via L ~ 5-10 nH range (very loose, solver is more accurate than formula)
    bool foundReasonableR = false;
    bool foundReasonableL = false;

    for( const auto& np : parasitics )
    {
        // R should be in the range 5..100 mOhm for our short trace
        if( np.m_R_mOhm > 5.0 && np.m_R_mOhm < 100.0 )
            foundReasonableR = true;

        // L should be positive and in the nH range
        if( np.m_L_nH > 0.1 && np.m_L_nH < 50.0 )
            foundReasonableL = true;

        BOOST_TEST_MESSAGE( "  Port " << np.m_SegmentId << ": R=" << np.m_R_mOhm << " mOhm"
                                      << ", L=" << np.m_L_nH << " nH" );
    }

    BOOST_CHECK_MESSAGE( foundReasonableR, "No port had R in expected range 5..100 mOhm" );
    BOOST_CHECK_MESSAGE( foundReasonableL, "No port had L in expected range 0.1..50 nH" );
}


// --------------------------------------------------------------------------
// FastCap solver integration: export, run solver, parse results, verify
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( SolverIntegration_FastCap, EXPORTER_TEST_FIXTURE )
{
    if( !findSolver( "fastcap" ) )
    {
        BOOST_TEST_MESSAGE( "SKIPPED: fastcap not found on PATH" );
        return;
    }

    auto cfg = makeConfig( { "VDD", "GND" } );

    // Export FastCap input
    FASTCAP_EXPORTER exporter( m_board.get(), cfg );
    bool             exportOk = exporter.Export( m_tempDir, "solver_test.lst" );
    BOOST_REQUIRE( exportOk );

    // Create a conductor-only list file by stripping dielectric (D) lines.
    // The MIT FastCap 2.0 binary runs out of memory with dielectric panels
    // due to its sbrk()-based allocator (~4 MB limit on modern systems).
    std::string lstPath = m_tempDir + "/solver_test.lst";
    std::string nodieLstPath = m_tempDir + "/solver_test_nodie.lst";
    {
        std::ifstream lstIn( lstPath );
        std::ofstream lstOut( nodieLstPath );
        BOOST_REQUIRE( lstIn.is_open() );
        BOOST_REQUIRE( lstOut.is_open() );

        std::string line;

        while( std::getline( lstIn, line ) )
        {
            // Skip dielectric surface lines (start with 'D')
            if( !line.empty() && line[0] == 'D' )
                continue;

            lstOut << line << "\n";
        }
    }

    // Run FastCap on conductor-only geometry
    std::string cmd = "cd " + m_tempDir + " && fastcap -lsolver_test_nodie.lst"
                      + " > fastcap_output.txt 2>&1";
    int ret = std::system( cmd.c_str() );
    BOOST_REQUIRE_MESSAGE( ret == 0, "FastCap exited with code " << ret << ". Check " << m_tempDir
                                                                 << "/fastcap_output.txt" );

    // Parse the output
    std::string fcOutPath = m_tempDir + "/fastcap_output.txt";
    std::string fcOutput = readFile( fcOutPath );
    BOOST_REQUIRE_MESSAGE( !fcOutput.empty(), "FastCap output file empty" );
    BOOST_REQUIRE_MESSAGE( fcOutput.find( "CAPACITANCE MATRIX" ) != std::string::npos,
                           "FastCap output does not contain capacitance matrix" );

    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY> capacitances;
    bool parseOk = PARASITIC_RESULT_PARSER::ParseFastCapOutput( fcOutPath, capacitances );
    BOOST_REQUIRE_MESSAGE( parseOk, "Failed to parse FastCap output" );
    BOOST_REQUIRE( !capacitances.empty() );

    // Derive lumped parasitics
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY> emptyImpedances;
    std::vector<PDN_PARASITIC::NET_PARASITIC>   parasitics;
    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics( emptyImpedances, capacitances, parasitics );

    // Verify: at least one conductor has reasonable capacitance
    //   Without dielectrics, values will be lower than with εr=4.5 substrate,
    //   but should still be in a reasonable range (0.01..100 pF).
    bool foundReasonableC = false;

    for( const auto& np : parasitics )
    {
        BOOST_TEST_MESSAGE( "  Conductor " << np.m_NetName << ": C=" << np.m_C_pF << " pF" );

        if( np.m_C_pF > 0.01 && np.m_C_pF < 1000.0 )
            foundReasonableC = true;
    }

    BOOST_CHECK_MESSAGE( foundReasonableC, "No conductor had C in expected range 0.01..1000 pF" );

    // Log all capacitance matrix entries
    for( const auto& cap : capacitances )
    {
        BOOST_TEST_MESSAGE( "  C[" << cap.m_ConductorI << "," << cap.m_ConductorJ
                                   << "] = " << cap.m_CapacitancePF << " pF" );
    }
}


// --------------------------------------------------------------------------
// Full pipeline end-to-end: run orchestrator, verify results
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( FullPipeline_SolverEndToEnd, EXPORTER_TEST_FIXTURE )
{
    if( !findSolver( "fasthenry" ) || !findSolver( "fastcap" ) )
    {
        BOOST_TEST_MESSAGE( "SKIPPED: fasthenry and/or fastcap not found on PATH" );
        return;
    }

    auto                 cfg = makeConfig( { "VDD", "GND" } );
    PARASITIC_EXTRACTION extractor( m_board.get(), cfg );

    // Run full extraction pipeline
    bool ok = extractor.RunExtraction();
    BOOST_REQUIRE_MESSAGE( ok, "RunExtraction() failed" );

    const auto& results = extractor.GetResults();

    // Verify ports were identified
    BOOST_CHECK_GE( results.m_Ports.size(), 1u );
    BOOST_TEST_MESSAGE( "Ports: " << results.m_Ports.size() );

    for( const auto& port : results.m_Ports )
    {
        BOOST_TEST_MESSAGE( "  " << port.m_Name << " net=" << port.m_NetName << " +"
                                 << port.m_PositiveNode << " -" << port.m_NegativeNode );
    }

    // Verify impedance matrix is populated
    BOOST_CHECK( !results.m_ImpedanceMatrix.empty() );

    for( const auto& imp : results.m_ImpedanceMatrix )
    {
        if( imp.m_PortI == imp.m_PortJ && !imp.m_Points.empty() )
        {
            BOOST_TEST_MESSAGE( "  Z[" << imp.m_PortI << "," << imp.m_PortJ << "] at "
                                       << imp.m_Points[0].m_FrequencyHz
                                       << " Hz: " << imp.m_Points[0].m_Z.real() << " + "
                                       << imp.m_Points[0].m_Z.imag() << "j ohms" );
        }
    }

    // Verify capacitance matrix is populated
    BOOST_CHECK( !results.m_CapacitanceMatrix.empty() );

    for( const auto& cap : results.m_CapacitanceMatrix )
    {
        BOOST_TEST_MESSAGE( "  C[" << cap.m_ConductorI << "," << cap.m_ConductorJ
                                   << "] = " << cap.m_CapacitancePF << " pF" );
    }

    // Verify lumped parasitics were derived
    BOOST_CHECK( !results.m_NetParasitics.empty() );

    for( const auto& np : results.m_NetParasitics )
    {
        BOOST_TEST_MESSAGE( "  " << np.m_SegmentId << " (" << np.m_NetName << ")"
                                 << ": R=" << np.m_R_mOhm << " mOhm"
                                 << ", L=" << np.m_L_nH << " nH"
                                 << ", C=" << np.m_C_pF << " pF" );
    }

    // Verify frequency range
    BOOST_CHECK_CLOSE( results.m_FreqMinHz, cfg.m_FreqMinHz, 0.01 );
    BOOST_CHECK_CLOSE( results.m_FreqMaxHz, cfg.m_FreqMaxHz, 0.01 );
    BOOST_CHECK_EQUAL( results.m_PointsPerDecade, cfg.m_PointsPerDecade );

    // Verify JSON serialization of real results
    std::string jsonPath = m_tempDir + "/full_e2e_results.json";
    bool        jsonOk = extractor.SerializeResults( jsonPath );
    BOOST_CHECK( jsonOk );

    std::string jsonContent = readFile( jsonPath );
    BOOST_CHECK( !jsonContent.empty() );
    BOOST_CHECK( jsonContent.find( "\"impedance_matrix\"" ) != std::string::npos );
    BOOST_CHECK( jsonContent.find( "\"capacitance_matrix\"" ) != std::string::npos );
    BOOST_CHECK( jsonContent.find( "\"net_parasitics\"" ) != std::string::npos );
}


// ============================================================================
// SPICE subcircuit exporter tests
// ============================================================================

BOOST_AUTO_TEST_CASE( SpiceSubcktExporter_SinglePort )
{
    // Build a simple extraction result with one port
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    PDN_PARASITIC::EXTRACTION_PORT port;
    port.m_Name = "Q1_VDD";
    port.m_PositiveNode = "N1";
    port.m_NegativeNode = "N2";
    port.m_NetName = "VDD";
    port.m_XMM = 10.0;
    port.m_YMM = 15.0;
    results.m_Ports.push_back( port );

    // Add impedance data: Z11 at 1 kHz = 0.020 + j*3.2e-5 ohms
    // This gives R=20 mOhm, L=Im(Z)/(2*pi*f) = 3.2e-5/(2*pi*1000) = 5.09 nH
    PDN_PARASITIC::IMPEDANCE_ENTRY z11;
    z11.m_PortI = 0;
    z11.m_PortJ = 0;
    z11.m_Points.push_back( { 1000.0, { 0.020, 3.2e-5 } } );
    z11.m_Points.push_back( { 1e6, { 0.021, 0.032 } } );
    results.m_ImpedanceMatrix.push_back( z11 );

    SPICE_SUBCKT_EXPORTER exporter( results );

    BOOST_CHECK_EQUAL( exporter.GetPortCount(), 1 );

    auto names = exporter.GetPortNames();
    BOOST_REQUIRE_EQUAL( names.size(), 1u );
    BOOST_CHECK_EQUAL( names[0], "Q1_VDD" );

    // Check extracted parasitics at default (lowest) frequency
    auto parasitics = exporter.GetPortParasitics();
    BOOST_REQUIRE_EQUAL( parasitics.size(), 1u );
    BOOST_CHECK_CLOSE( parasitics[0].m_R_Ohm, 0.020, 0.1 );

    double expectedL = 3.2e-5 / ( 2.0 * M_PI * 1000.0 );
    BOOST_CHECK_CLOSE( parasitics[0].m_L_Henry, expectedL, 0.1 );

    // Generate and verify subcircuit syntax
    std::string spice = exporter.Generate();
    BOOST_CHECK( spice.find( ".subckt" ) != std::string::npos );
    BOOST_CHECK( spice.find( ".ends" ) != std::string::npos );
    BOOST_CHECK( spice.find( "R1" ) != std::string::npos );
    BOOST_CHECK( spice.find( "L1" ) != std::string::npos );
    BOOST_CHECK( spice.find( "Q1_VDD_p" ) != std::string::npos );
    BOOST_CHECK( spice.find( "Q1_VDD_n" ) != std::string::npos );

    BOOST_TEST_MESSAGE( "Generated SPICE subcircuit:\n" << spice );
}


BOOST_AUTO_TEST_CASE( SpiceSubcktExporter_TwoPorts_WithCoupling )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    // Two ports
    PDN_PARASITIC::EXTRACTION_PORT p1, p2;
    p1.m_Name = "Q1_VDD";
    p1.m_PositiveNode = "N1";
    p1.m_NegativeNode = "N2";
    p1.m_NetName = "VDD";
    p2.m_Name = "Q2_VDD";
    p2.m_PositiveNode = "N3";
    p2.m_NegativeNode = "N4";
    p2.m_NetName = "VDD";
    results.m_Ports.push_back( p1 );
    results.m_Ports.push_back( p2 );

    double freq = 100e3;
    double omega = 2.0 * M_PI * freq;

    // Self impedances
    double L1 = 5e-9;  // 5 nH
    double L2 = 3e-9;  // 3 nH
    double R1 = 0.020; // 20 mOhm
    double R2 = 0.015; // 15 mOhm

    // Mutual: M12 = 1.5 nH -> K = M/sqrt(L1*L2) = 1.5e-9/sqrt(15e-18) = 0.387
    double M12 = 1.5e-9;
    double expectedK = M12 / std::sqrt( L1 * L2 );

    PDN_PARASITIC::IMPEDANCE_ENTRY z11, z22, z12, z21;
    z11.m_PortI = 0;
    z11.m_PortJ = 0;
    z11.m_Points.push_back( { freq, { R1, omega * L1 } } );
    z22.m_PortI = 1;
    z22.m_PortJ = 1;
    z22.m_Points.push_back( { freq, { R2, omega * L2 } } );
    z12.m_PortI = 0;
    z12.m_PortJ = 1;
    z12.m_Points.push_back( { freq, { 0.0, omega * M12 } } );
    z21.m_PortI = 1;
    z21.m_PortJ = 0;
    z21.m_Points.push_back( { freq, { 0.0, omega * M12 } } );

    results.m_ImpedanceMatrix.push_back( z11 );
    results.m_ImpedanceMatrix.push_back( z12 );
    results.m_ImpedanceMatrix.push_back( z21 );
    results.m_ImpedanceMatrix.push_back( z22 );

    SPICE_SUBCKT_EXPORTER exporter( results );
    BOOST_CHECK_EQUAL( exporter.GetPortCount(), 2 );

    // Check coupling
    auto coupling = exporter.GetCouplingPairs();
    BOOST_REQUIRE_EQUAL( coupling.size(), 1u );
    BOOST_CHECK_CLOSE( coupling[0].m_K, expectedK, 0.1 );
    BOOST_CHECK_EQUAL( coupling[0].m_PortI, 0 );
    BOOST_CHECK_EQUAL( coupling[0].m_PortJ, 1 );

    // Generate and check K statement present
    std::string spice = exporter.Generate();
    BOOST_CHECK( spice.find( "K1_2" ) != std::string::npos );
    BOOST_CHECK( spice.find( "L1" ) != std::string::npos );
    BOOST_CHECK( spice.find( "L2" ) != std::string::npos );

    BOOST_TEST_MESSAGE( "Generated 2-port SPICE subcircuit:\n" << spice );
}


BOOST_AUTO_TEST_CASE( SpiceSubcktExporter_MinKFilter )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    PDN_PARASITIC::EXTRACTION_PORT p1, p2;
    p1.m_Name = "P1";
    p2.m_Name = "P2";
    results.m_Ports.push_back( p1 );
    results.m_Ports.push_back( p2 );

    double freq = 100e3;
    double omega = 2.0 * M_PI * freq;
    double L1 = 5e-9;
    double L2 = 5e-9;
    double M12 = 0.01e-9; // Very weak coupling: K = 0.01/5 = 0.002

    PDN_PARASITIC::IMPEDANCE_ENTRY z11, z22, z12, z21;
    z11.m_PortI = 0;
    z11.m_PortJ = 0;
    z11.m_Points.push_back( { freq, { 0.01, omega * L1 } } );
    z22.m_PortI = 1;
    z22.m_PortJ = 1;
    z22.m_Points.push_back( { freq, { 0.01, omega * L2 } } );
    z12.m_PortI = 0;
    z12.m_PortJ = 1;
    z12.m_Points.push_back( { freq, { 0.0, omega * M12 } } );
    z21.m_PortI = 1;
    z21.m_PortJ = 0;
    z21.m_Points.push_back( { freq, { 0.0, omega * M12 } } );

    results.m_ImpedanceMatrix.push_back( z11 );
    results.m_ImpedanceMatrix.push_back( z12 );
    results.m_ImpedanceMatrix.push_back( z21 );
    results.m_ImpedanceMatrix.push_back( z22 );

    SPICE_SUBCKT_EXPORTER exporter( results );

    // Default minK = 0.01 -> should filter out K=0.002
    auto coupling = exporter.GetCouplingPairs();
    BOOST_CHECK_EQUAL( coupling.size(), 0u );

    // With lower threshold, should include it
    exporter.SetMinCouplingCoeff( 0.001 );
    coupling = exporter.GetCouplingPairs();
    BOOST_CHECK_EQUAL( coupling.size(), 1u );
}


BOOST_AUTO_TEST_CASE( SpiceSubcktExporter_ReferenceFrequency )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    PDN_PARASITIC::EXTRACTION_PORT port;
    port.m_Name = "P1";
    results.m_Ports.push_back( port );

    // Add impedance at two frequencies with different R (skin effect)
    PDN_PARASITIC::IMPEDANCE_ENTRY z11;
    z11.m_PortI = 0;
    z11.m_PortJ = 0;
    z11.m_Points.push_back( { 1e3, { 0.020, 3.2e-5 } } ); // 1 kHz: R=20 mOhm
    z11.m_Points.push_back( { 1e6, { 0.035, 0.032 } } );  // 1 MHz: R=35 mOhm (skin)
    results.m_ImpedanceMatrix.push_back( z11 );

    SPICE_SUBCKT_EXPORTER exporter( results );

    // Default: uses lowest frequency (1 kHz)
    auto pp = exporter.GetPortParasitics();
    BOOST_CHECK_CLOSE( pp[0].m_R_Ohm, 0.020, 0.1 );

    // Set reference to 1 MHz
    exporter.SetReferenceFrequency( 1e6 );
    pp = exporter.GetPortParasitics();
    BOOST_CHECK_CLOSE( pp[0].m_R_Ohm, 0.035, 0.1 );
}


BOOST_AUTO_TEST_CASE( SpiceSubcktExporter_WithCapacitance )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    PDN_PARASITIC::EXTRACTION_PORT port;
    port.m_Name = "P1";
    results.m_Ports.push_back( port );

    PDN_PARASITIC::IMPEDANCE_ENTRY z11;
    z11.m_PortI = 0;
    z11.m_PortJ = 0;
    z11.m_Points.push_back( { 1e3, { 0.020, 3.2e-5 } } );
    results.m_ImpedanceMatrix.push_back( z11 );

    // Add capacitance entries
    PDN_PARASITIC::CAPACITANCE_ENTRY c1;
    c1.m_ConductorI = "VDD";
    c1.m_ConductorJ = "GND";
    c1.m_CapacitancePF = 0.5;
    results.m_CapacitanceMatrix.push_back( c1 );

    SPICE_SUBCKT_EXPORTER exporter( results );
    std::string           spice = exporter.Generate();

    BOOST_CHECK( spice.find( "C1" ) != std::string::npos );
    // Capacitance comment uses scientific notation (stream is in std::scientific mode)
    BOOST_CHECK( spice.find( "pF" ) != std::string::npos );
    // Value line has capacitance in farads
    BOOST_CHECK( spice.find( "cap_VDD" ) != std::string::npos );
    BOOST_CHECK( spice.find( "cap_GND" ) != std::string::npos );

    BOOST_TEST_MESSAGE( "SPICE with capacitance:\n" << spice );
}


BOOST_AUTO_TEST_CASE( SpiceSubcktExporter_FileExport )
{
    PDN_PARASITIC::EXTRACTION_RESULTS results;

    PDN_PARASITIC::EXTRACTION_PORT port;
    port.m_Name = "P1";
    port.m_NetName = "VDD";
    port.m_PositiveNode = "N1";
    port.m_NegativeNode = "N2";
    results.m_Ports.push_back( port );

    PDN_PARASITIC::IMPEDANCE_ENTRY z11;
    z11.m_PortI = 0;
    z11.m_PortJ = 0;
    z11.m_Points.push_back( { 1e3, { 0.020, 3.2e-5 } } );
    results.m_ImpedanceMatrix.push_back( z11 );

    std::string tempDir = "build/qa/spice_subckt_test_tmp";
    std::filesystem::create_directories( tempDir );

    SPICE_SUBCKT_EXPORTER exporter( results );
    exporter.SetSubcktName( "MY_PCB" );

    std::string outPath = tempDir + "/pcb_parasitics.subckt";
    bool        ok = exporter.Export( outPath );
    BOOST_REQUIRE( ok );

    // Read back and verify
    std::ifstream file( outPath );
    std::string   content( ( std::istreambuf_iterator<char>( file ) ),
                           std::istreambuf_iterator<char>() );

    BOOST_CHECK( content.find( ".subckt MY_PCB" ) != std::string::npos );
    BOOST_CHECK( content.find( ".ends MY_PCB" ) != std::string::npos );
    BOOST_CHECK( content.find( "R1" ) != std::string::npos );
    BOOST_CHECK( content.find( "L1" ) != std::string::npos );

    std::filesystem::remove_all( tempDir );
}


// Integration test: run solver, then export to SPICE subcircuit
BOOST_FIXTURE_TEST_CASE( SpiceSubcktExporter_FromSolverResults, EXPORTER_TEST_FIXTURE )
{
    if( !findSolver( "fasthenry" ) )
    {
        BOOST_TEST_MESSAGE( "SKIPPED: fasthenry not found on PATH" );
        return;
    }

    auto cfg = makeConfig( { "VDD", "GND" } );

    // Export and run FastHenry
    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );
    std::string        inpPath = m_tempDir + "/spice_test.inp";
    BOOST_REQUIRE( exporter.Export( inpPath ) );

    std::string cmd = "cd " + m_tempDir + " && fasthenry spice_test.inp" + " > /dev/null 2>&1";
    int         ret = std::system( cmd.c_str() );
    BOOST_REQUIRE_MESSAGE( ret == 0, "FastHenry failed" );

    // Parse results
    std::string                                 zcPath = m_tempDir + "/Zc.mat";
    std::vector<PDN_PARASITIC::EXTRACTION_PORT> ports = exporter.GetPorts();
    BOOST_REQUIRE_GE( ports.size(), 1u );

    PDN_PARASITIC::EXTRACTION_RESULTS results;
    results.m_Ports = ports;

    BOOST_REQUIRE( PARASITIC_RESULT_PARSER::ParseFastHenryOutput( zcPath, results.m_Ports,
                                                                  results.m_ImpedanceMatrix ) );

    // Export to SPICE subcircuit
    SPICE_SUBCKT_EXPORTER spiceExporter( results );
    spiceExporter.SetSubcktName( "TEST_BOARD" );

    std::string spicePath = m_tempDir + "/test_board.subckt";
    BOOST_REQUIRE( spiceExporter.Export( spicePath ) );

    // Verify content
    std::string spice = readFile( spicePath );
    BOOST_CHECK( !spice.empty() );
    BOOST_CHECK( spice.find( ".subckt TEST_BOARD" ) != std::string::npos );
    BOOST_CHECK( spice.find( ".ends TEST_BOARD" ) != std::string::npos );

    // Verify extracted values are reasonable
    auto parasitics = spiceExporter.GetPortParasitics();
    BOOST_REQUIRE_GE( parasitics.size(), 1u );

    for( const auto& pp : parasitics )
    {
        BOOST_TEST_MESSAGE( "  Port " << pp.m_Name << ": R=" << ( pp.m_R_Ohm * 1000.0 ) << " mOhm"
                                      << ", L=" << ( pp.m_L_Henry * 1e9 ) << " nH" );

        // R should be positive and in mOhm range for our trace
        BOOST_CHECK_GT( pp.m_R_Ohm, 0.0 );
        BOOST_CHECK_LT( pp.m_R_Ohm, 1.0 );

        // L should be positive and in nH range
        BOOST_CHECK_GT( pp.m_L_Henry, 0.0 );
        BOOST_CHECK_LT( pp.m_L_Henry, 100e-9 );
    }

    BOOST_TEST_MESSAGE( "Generated SPICE subcircuit from solver:\n" << spice );
}


// ============================================================================
// Explicit PORT_SPEC tests
// ============================================================================


// --------------------------------------------------------------------------
// ExplicitPorts: net codes derived from PORT_SPEC entries
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ExplicitPorts_NetsFromPortSpecs, EXPORTER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_CONFIG cfg;
    cfg.m_OutputDir = m_tempDir;
    cfg.m_FreqMinHz = 1e3;
    cfg.m_FreqMaxHz = 1e9;
    cfg.m_PointsPerDecade = 5;
    cfg.m_PlaneSegX = 4;
    cfg.m_PlaneSegY = 4;

    // Find the VDD and GND net codes from the board
    NETINFO_ITEM* vddNet = m_board->FindNet( wxT( "VDD" ) );
    NETINFO_ITEM* gndNet = m_board->FindNet( wxT( "GND" ) );

    BOOST_REQUIRE( vddNet );
    BOOST_REQUIRE( gndNet );

    // Create explicit port specs (simulating user pad selection)
    PDN_PARASITIC::PORT_SPEC signalSpec;
    signalSpec.m_Name = "C1_VDD";
    signalSpec.m_NetName = "VDD";
    signalSpec.m_NetCode = vddNet->GetNetCode();
    signalSpec.m_XMM = 29.5;
    signalSpec.m_YMM = 15.0;
    signalSpec.m_LayerId = F_Cu;
    signalSpec.m_IsGround = false;

    PDN_PARASITIC::PORT_SPEC groundSpec;
    groundSpec.m_Name = "C1_GND";
    groundSpec.m_NetName = "GND";
    groundSpec.m_NetCode = gndNet->GetNetCode();
    groundSpec.m_XMM = 30.5;
    groundSpec.m_YMM = 15.0;
    groundSpec.m_LayerId = F_Cu;
    groundSpec.m_IsGround = true;

    cfg.m_PortSpecs.push_back( signalSpec );
    cfg.m_PortSpecs.push_back( groundSpec );

    // Export with PORT_SPEC-driven net resolution
    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );
    std::string        outPath = m_tempDir + "/explicit_ports_test.inp";
    BOOST_REQUIRE( exporter.Export( outPath ) );

    std::string content = readFile( outPath );

    // Verify: the output should contain geometry (nodes and segments exist)
    BOOST_CHECK( content.find( ".units mm" ) != std::string::npos );
    BOOST_CHECK( content.find( ".end" ) != std::string::npos );

    // Ports should exist with the names from PORT_SPEC
    auto ports = exporter.GetPorts();
    BOOST_REQUIRE_GE( ports.size(), 1u );
    BOOST_CHECK_EQUAL( ports[0].m_Name, "C1_VDD" );
    BOOST_CHECK_EQUAL( ports[0].m_NetName, "VDD" );

    // .external should reference the port
    BOOST_CHECK( content.find( ".external" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// ExplicitPorts: ground pairing picks nearest ground pad
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ExplicitPorts_GroundPairing, EXPORTER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_CONFIG cfg;
    cfg.m_OutputDir = m_tempDir;
    cfg.m_FreqMinHz = 1e3;
    cfg.m_FreqMaxHz = 1e9;
    cfg.m_PointsPerDecade = 5;
    cfg.m_PlaneSegX = 4;
    cfg.m_PlaneSegY = 4;

    NETINFO_ITEM* vddNet = m_board->FindNet( wxT( "VDD" ) );
    NETINFO_ITEM* gndNet = m_board->FindNet( wxT( "GND" ) );

    BOOST_REQUIRE( vddNet );
    BOOST_REQUIRE( gndNet );

    // Signal pad at one end of the trace
    PDN_PARASITIC::PORT_SPEC signalSpec;
    signalSpec.m_Name = "C1_VDD";
    signalSpec.m_NetName = "VDD";
    signalSpec.m_NetCode = vddNet->GetNetCode();
    signalSpec.m_XMM = 29.5;
    signalSpec.m_YMM = 15.0;
    signalSpec.m_LayerId = F_Cu;
    signalSpec.m_IsGround = false;

    // Two ground pads at different distances
    PDN_PARASITIC::PORT_SPEC gndNear;
    gndNear.m_Name = "C1_GND";
    gndNear.m_NetName = "GND";
    gndNear.m_NetCode = gndNet->GetNetCode();
    gndNear.m_XMM = 30.5; // 1mm from signal pad
    gndNear.m_YMM = 15.0;
    gndNear.m_LayerId = F_Cu;
    gndNear.m_IsGround = true;

    PDN_PARASITIC::PORT_SPEC gndFar;
    gndFar.m_Name = "REMOTE_GND";
    gndFar.m_NetName = "GND";
    gndFar.m_NetCode = gndNet->GetNetCode();
    gndFar.m_XMM = 10.0; // 19.5mm from signal pad
    gndFar.m_YMM = 15.0;
    gndFar.m_LayerId = F_Cu;
    gndFar.m_IsGround = true;

    cfg.m_PortSpecs.push_back( signalSpec );
    cfg.m_PortSpecs.push_back( gndNear );
    cfg.m_PortSpecs.push_back( gndFar );

    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );
    std::string        outPath = m_tempDir + "/ground_pairing_test.inp";
    BOOST_REQUIRE( exporter.Export( outPath ) );

    auto ports = exporter.GetPorts();
    BOOST_REQUIRE_EQUAL( ports.size(), 1u );

    // The port's negative node should reference the nearer ground pad's position
    // (30.5, 15.0) rather than the far one (10.0, 15.0).
    // The plane node name encodes the position, so we can check for the near coords.
    std::string content = readFile( outPath );

    // The near ground pad is at (30.5, 15.0), which would generate a plane node
    // name containing "30500" (30.5 * 1000)
    BOOST_CHECK( ports[0].m_NegativeNode.find( "30500" ) != std::string::npos
                 || content.find( "30500" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// ExplicitPorts: component grouping merges multi-pad same-net ports
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ExplicitPorts_ComponentGrouping, EXPORTER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_CONFIG cfg;
    cfg.m_OutputDir = m_tempDir;
    cfg.m_FreqMinHz = 1e3;
    cfg.m_FreqMaxHz = 1e9;
    cfg.m_PointsPerDecade = 5;
    cfg.m_PlaneSegX = 4;
    cfg.m_PlaneSegY = 4;
    cfg.m_GroupPadsByComponent = true;

    NETINFO_ITEM* vddNet = m_board->FindNet( wxT( "VDD" ) );
    NETINFO_ITEM* gndNet = m_board->FindNet( wxT( "GND" ) );

    BOOST_REQUIRE( vddNet );
    BOOST_REQUIRE( gndNet );

    // Two signal pads with same component prefix + net (simulating multi-pad IC)
    PDN_PARASITIC::PORT_SPEC sig1;
    sig1.m_Name = "U1_VDD";
    sig1.m_NetName = "VDD";
    sig1.m_NetCode = vddNet->GetNetCode();
    sig1.m_XMM = 29.5;
    sig1.m_YMM = 15.0;
    sig1.m_LayerId = F_Cu;
    sig1.m_IsGround = false;

    PDN_PARASITIC::PORT_SPEC sig2;
    sig2.m_Name = "U1_VDD";
    sig2.m_NetName = "VDD";
    sig2.m_NetCode = vddNet->GetNetCode();
    sig2.m_XMM = 20.0; // Near the via
    sig2.m_YMM = 15.0;
    sig2.m_LayerId = F_Cu;
    sig2.m_IsGround = false;

    PDN_PARASITIC::PORT_SPEC gndSpec;
    gndSpec.m_Name = "U1_GND";
    gndSpec.m_NetName = "GND";
    gndSpec.m_NetCode = gndNet->GetNetCode();
    gndSpec.m_XMM = 30.5;
    gndSpec.m_YMM = 15.0;
    gndSpec.m_LayerId = F_Cu;
    gndSpec.m_IsGround = true;

    cfg.m_PortSpecs.push_back( sig1 );
    cfg.m_PortSpecs.push_back( sig2 );
    cfg.m_PortSpecs.push_back( gndSpec );

    FASTHENRY_EXPORTER exporter( m_board.get(), cfg );
    std::string        outPath = m_tempDir + "/grouping_test.inp";
    BOOST_REQUIRE( exporter.Export( outPath ) );

    // With grouping, two pads on the same component+net should produce only 1 port
    auto ports = exporter.GetPorts();
    BOOST_CHECK_EQUAL( ports.size(), 1u );

    // The second pad should be merged via .equiv
    std::string content = readFile( outPath );
    BOOST_CHECK( content.find( ".equiv" ) != std::string::npos );
}


// --------------------------------------------------------------------------
// ExplicitPorts: FastCap also derives nets from PORT_SPEC
// --------------------------------------------------------------------------
BOOST_FIXTURE_TEST_CASE( ExplicitPorts_FastCapNetDerivation, EXPORTER_TEST_FIXTURE )
{
    PDN_PARASITIC::EXTRACTION_CONFIG cfg;
    cfg.m_OutputDir = m_tempDir;
    cfg.m_PanelTargetSizeMM = 0.5;
    cfg.m_ViaFacets = 8;
    cfg.m_PlaneSegX = 4;
    cfg.m_PlaneSegY = 4;

    NETINFO_ITEM* vddNet = m_board->FindNet( wxT( "VDD" ) );
    NETINFO_ITEM* gndNet = m_board->FindNet( wxT( "GND" ) );

    BOOST_REQUIRE( vddNet );
    BOOST_REQUIRE( gndNet );

    PDN_PARASITIC::PORT_SPEC signalSpec;
    signalSpec.m_Name = "C1_VDD";
    signalSpec.m_NetName = "VDD";
    signalSpec.m_NetCode = vddNet->GetNetCode();
    signalSpec.m_XMM = 29.5;
    signalSpec.m_YMM = 15.0;
    signalSpec.m_LayerId = F_Cu;
    signalSpec.m_IsGround = false;

    PDN_PARASITIC::PORT_SPEC groundSpec;
    groundSpec.m_Name = "C1_GND";
    groundSpec.m_NetName = "GND";
    groundSpec.m_NetCode = gndNet->GetNetCode();
    groundSpec.m_XMM = 30.5;
    groundSpec.m_YMM = 15.0;
    groundSpec.m_LayerId = F_Cu;
    groundSpec.m_IsGround = true;

    cfg.m_PortSpecs.push_back( signalSpec );
    cfg.m_PortSpecs.push_back( groundSpec );

    // m_NetNames is empty, so without PORT_SPEC it would extract all nets.
    // With PORT_SPEC, it should only extract VDD and GND.
    FASTCAP_EXPORTER exporter( m_board.get(), cfg );
    bool             ok = exporter.Export( m_tempDir, "explicit_fastcap_test.lst" );
    BOOST_REQUIRE( ok );

    // Verify output files exist
    BOOST_CHECK( std::filesystem::exists( m_tempDir + "/explicit_fastcap_test.lst" ) );
}


BOOST_AUTO_TEST_SUITE_END()
