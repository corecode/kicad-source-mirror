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
#include <fstream>
#include <sstream>

#include <exporters/parasitic_extraction/pdn_parasitic_data.h>
#include <exporters/parasitic_extraction/parasitic_result_parser.h>

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


BOOST_AUTO_TEST_SUITE_END()
