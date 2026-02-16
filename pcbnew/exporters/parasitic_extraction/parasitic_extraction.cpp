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

#include "parasitic_extraction.h"
#include "fasthenry_exporter.h"
#include "fastcap_exporter.h"
#include "parasitic_result_parser.h"

#include <board.h>
#include <board_design_settings.h>
#include <board_stackup_manager/board_stackup.h>

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>


/// Conductivity of copper in S/m
static constexpr double COPPER_SIGMA = 5.8e7;


PARASITIC_EXTRACTION::PARASITIC_EXTRACTION(
    BOARD* aBoard,
    const PDN_PARASITIC::EXTRACTION_CONFIG& aConfig ) :
    m_board( aBoard ),
    m_config( aConfig )
{
}


bool PARASITIC_EXTRACTION::ExportGeometry()
{
    if( !m_board )
        return false;

    const std::string& outDir = m_config.m_OutputDir;

    // Export FastHenry geometry
    reportProgress( "Exporting FastHenry geometry...", 10 );

    FASTHENRY_EXPORTER fhExporter( m_board, m_config );
    std::string fhPath = outDir + "/pdn_extraction.inp";

    if( !fhExporter.Export( fhPath ) )
    {
        reportProgress( "FastHenry export failed", -1 );
        return false;
    }

    // Export FastCap geometry
    reportProgress( "Exporting FastCap geometry...", 30 );

    FASTCAP_EXPORTER fcExporter( m_board, m_config );

    if( !fcExporter.Export( outDir, "pdn_extraction.lst" ) )
    {
        reportProgress( "FastCap export failed", -1 );
        return false;
    }

    reportProgress( "Geometry export complete", 50 );

    // Store port info from the FastHenry exporter
    m_results.m_Ports = fhExporter.GetPorts();

    return true;
}


bool PARASITIC_EXTRACTION::RunExtraction()
{
    if( !ExportGeometry() )
        return false;

    const std::string& outDir = m_config.m_OutputDir;

    // Run FastHenry
    reportProgress( "Running FastHenry solver...", 50 );

    if( !runFastHenry( outDir + "/pdn_extraction.inp", outDir ) )
    {
        reportProgress( "FastHenry solver failed", -1 );
        return false;
    }

    // Run FastCap
    reportProgress( "Running FastCap solver...", 70 );

    if( !runFastCap( outDir + "/pdn_extraction.lst", outDir ) )
    {
        reportProgress( "FastCap solver failed", -1 );
        return false;
    }

    // Parse FastHenry results
    reportProgress( "Parsing FastHenry results...", 85 );

    std::string zcMatPath = outDir + "/Zc.mat";

    if( !PARASITIC_RESULT_PARSER::ParseFastHenryOutput(
            zcMatPath, m_results.m_Ports, m_results.m_ImpedanceMatrix ) )
    {
        reportProgress( "Failed to parse FastHenry output", -1 );
        return false;
    }

    // Parse FastCap results
    reportProgress( "Parsing FastCap results...", 90 );

    std::string fcOutPath = outDir + "/fastcap_output.txt";

    if( !PARASITIC_RESULT_PARSER::ParseFastCapOutput(
            fcOutPath, m_results.m_CapacitanceMatrix ) )
    {
        reportProgress( "Failed to parse FastCap output", -1 );
        return false;
    }

    // Derive lumped parasitics
    reportProgress( "Computing lumped parasitics...", 95 );

    PARASITIC_RESULT_PARSER::DeriveLumpedParasitics(
        m_results.m_ImpedanceMatrix,
        m_results.m_CapacitanceMatrix,
        m_results.m_NetParasitics );

    // Store frequency range
    m_results.m_FreqMinHz = m_config.m_FreqMinHz;
    m_results.m_FreqMaxHz = m_config.m_FreqMaxHz;
    m_results.m_PointsPerDecade = m_config.m_PointsPerDecade;

    // Build stackup info for the results
    buildStackupResults();

    reportProgress( "Extraction complete", 100 );
    return true;
}


bool PARASITIC_EXTRACTION::runFastHenry( const std::string& aInputPath,
                                          const std::string& aOutputDir )
{
    // Build command line: fasthenry <input.inp>
    // FastHenry writes Zc.mat to the current directory
    std::ostringstream cmd;
    cmd << "cd " << aOutputDir << " && "
        << m_config.m_FastHenryPath << " " << aInputPath
        << " > fasthenry_log.txt 2>&1";

    int ret = std::system( cmd.str().c_str() );
    return ret == 0;
}


bool PARASITIC_EXTRACTION::runFastCap( const std::string& aListPath,
                                        const std::string& aOutputDir )
{
    // Build command line: fastcap -l<list_file>
    // FastCap writes capacitance matrix to stdout, which we redirect
    std::ostringstream cmd;
    cmd << "cd " << aOutputDir << " && "
        << m_config.m_FastCapPath << " -l" << aListPath
        << " > fastcap_output.txt 2>&1";

    int ret = std::system( cmd.str().c_str() );
    return ret == 0;
}


void PARASITIC_EXTRACTION::buildStackupResults()
{
    const BOARD_DESIGN_SETTINGS& bds = m_board->GetDesignSettings();
    const BOARD_STACKUP& stackup = bds.GetStackupDescriptor();

    m_results.m_Stackup.clear();

    double zMM = 0.0;

    for( const BOARD_STACKUP_ITEM* item : stackup.GetList() )
    {
        if( !item->IsEnabled() )
            continue;

        if( item->GetType() == BS_ITEM_TYPE_COPPER )
        {
            double thickMM = item->GetThickness() / 1e6;  // IU (nm) to mm

            if( thickMM <= 0.0 )
                thickMM = 0.035;

            PDN_PARASITIC::STACKUP_LAYER layer;
            layer.m_Name = item->GetLayerName().ToStdString();
            layer.m_IsCopperLayer = true;
            layer.m_ZPositionMM = zMM;
            layer.m_ThicknessMM = thickMM;
            layer.m_ConductivitySPerM = COPPER_SIGMA;

            m_results.m_Stackup.push_back( layer );
            zMM += thickMM;
        }
        else if( item->GetType() == BS_ITEM_TYPE_DIELECTRIC )
        {
            for( int sub = 0; sub < item->GetSublayersCount(); sub++ )
            {
                double thickMM = item->GetThickness( sub ) / 1e6;

                PDN_PARASITIC::STACKUP_LAYER layer;
                layer.m_Name = item->FormatDielectricLayerName().ToStdString();

                if( sub > 0 )
                    layer.m_Name += "_sub" + std::to_string( sub );

                layer.m_IsCopperLayer = false;
                layer.m_ZPositionMM = zMM;
                layer.m_ThicknessMM = thickMM;
                layer.m_EpsilonR = item->GetEpsilonR( sub );
                layer.m_LossTangent = item->GetLossTangent( sub );

                m_results.m_Stackup.push_back( layer );
                zMM += thickMM;
            }
        }
    }
}


bool PARASITIC_EXTRACTION::SerializeResults( const std::string& aJsonPath ) const
{
    return m_results.SerializeToJson( aJsonPath );
}


// JSON serialization for EXTRACTION_RESULTS
bool PDN_PARASITIC::EXTRACTION_RESULTS::SerializeToJson( const std::string& aFilePath ) const
{
    std::ofstream file( aFilePath );

    if( !file.is_open() )
        return false;

    file << std::fixed << std::setprecision( 6 );

    file << "{" << std::endl;

    // Frequency range
    file << "  \"frequency_range\": {" << std::endl;
    file << "    \"min_hz\": " << std::scientific << m_FreqMinHz << "," << std::endl;
    file << "    \"max_hz\": " << m_FreqMaxHz << "," << std::endl;
    file << "    \"points_per_decade\": " << m_PointsPerDecade << std::endl;
    file << "  }," << std::endl;

    // Stackup
    file << std::fixed;
    file << "  \"stackup\": [" << std::endl;

    for( size_t i = 0; i < m_Stackup.size(); i++ )
    {
        const auto& layer = m_Stackup[i];
        file << "    {" << std::endl;
        file << "      \"name\": \"" << layer.m_Name << "\"," << std::endl;
        file << "      \"is_copper\": " << ( layer.m_IsCopperLayer ? "true" : "false" )
             << "," << std::endl;
        file << "      \"z_position_mm\": " << layer.m_ZPositionMM << "," << std::endl;
        file << "      \"thickness_mm\": " << layer.m_ThicknessMM << "," << std::endl;

        if( layer.m_IsCopperLayer )
        {
            file << "      \"conductivity_s_per_m\": " << std::scientific
                 << layer.m_ConductivitySPerM << std::fixed << std::endl;
        }
        else
        {
            file << "      \"epsilon_r\": " << layer.m_EpsilonR << "," << std::endl;
            file << "      \"loss_tangent\": " << layer.m_LossTangent << std::endl;
        }

        file << "    }" << ( i < m_Stackup.size() - 1 ? "," : "" ) << std::endl;
    }

    file << "  ]," << std::endl;

    // Ports
    file << "  \"ports\": [" << std::endl;

    for( size_t i = 0; i < m_Ports.size(); i++ )
    {
        const auto& port = m_Ports[i];
        file << "    {" << std::endl;
        file << "      \"name\": \"" << port.m_Name << "\"," << std::endl;
        file << "      \"net\": \"" << port.m_NetName << "\"," << std::endl;
        file << "      \"positive_node\": \"" << port.m_PositiveNode << "\"," << std::endl;
        file << "      \"negative_node\": \"" << port.m_NegativeNode << "\"," << std::endl;
        file << "      \"x_mm\": " << port.m_XMM << "," << std::endl;
        file << "      \"y_mm\": " << port.m_YMM << std::endl;
        file << "    }" << ( i < m_Ports.size() - 1 ? "," : "" ) << std::endl;
    }

    file << "  ]," << std::endl;

    // Impedance matrix (from FastHenry)
    file << "  \"impedance_matrix\": [" << std::endl;

    for( size_t i = 0; i < m_ImpedanceMatrix.size(); i++ )
    {
        const auto& entry = m_ImpedanceMatrix[i];
        file << "    {" << std::endl;
        file << "      \"port_i\": " << entry.m_PortI << "," << std::endl;
        file << "      \"port_j\": " << entry.m_PortJ << "," << std::endl;
        file << "      \"points\": [" << std::endl;

        for( size_t j = 0; j < entry.m_Points.size(); j++ )
        {
            const auto& pt = entry.m_Points[j];
            file << "        {" << std::endl;
            file << "          \"frequency_hz\": " << std::scientific << pt.m_FrequencyHz
                 << "," << std::endl;
            file << "          \"z_real_ohm\": " << pt.m_Z.real() << "," << std::endl;
            file << "          \"z_imag_ohm\": " << pt.m_Z.imag() << std::endl;
            file << "        }" << ( j < entry.m_Points.size() - 1 ? "," : "" ) << std::endl;
        }

        file << std::fixed;
        file << "      ]" << std::endl;
        file << "    }" << ( i < m_ImpedanceMatrix.size() - 1 ? "," : "" ) << std::endl;
    }

    file << "  ]," << std::endl;

    // Capacitance matrix (from FastCap)
    file << "  \"capacitance_matrix\": [" << std::endl;

    for( size_t i = 0; i < m_CapacitanceMatrix.size(); i++ )
    {
        const auto& entry = m_CapacitanceMatrix[i];
        file << "    {" << std::endl;
        file << "      \"conductor_i\": \"" << entry.m_ConductorI << "\"," << std::endl;
        file << "      \"conductor_j\": \"" << entry.m_ConductorJ << "\"," << std::endl;
        file << "      \"capacitance_pf\": " << entry.m_CapacitancePF << std::endl;
        file << "    }" << ( i < m_CapacitanceMatrix.size() - 1 ? "," : "" ) << std::endl;
    }

    file << "  ]," << std::endl;

    // Lumped net parasitics
    file << "  \"net_parasitics\": [" << std::endl;

    for( size_t i = 0; i < m_NetParasitics.size(); i++ )
    {
        const auto& np = m_NetParasitics[i];
        file << "    {" << std::endl;
        file << "      \"net\": \"" << np.m_NetName << "\"," << std::endl;
        file << "      \"segment_id\": \"" << np.m_SegmentId << "\"," << std::endl;
        file << "      \"length_mm\": " << np.m_LengthMM << "," << std::endl;
        file << "      \"r_mohm\": " << np.m_R_mOhm << "," << std::endl;
        file << "      \"l_nh\": " << np.m_L_nH << "," << std::endl;
        file << "      \"c_pf\": " << np.m_C_pF << "," << std::endl;
        file << "      \"rlgc\": {" << std::endl;
        file << "        \"r_ohm_per_m\": " << np.m_RLGC.m_R_OhmPerM << "," << std::endl;
        file << "        \"l_nh_per_m\": " << np.m_RLGC.m_L_nHPerM << "," << std::endl;
        file << "        \"g_s_per_m\": " << np.m_RLGC.m_G_SPerM << "," << std::endl;
        file << "        \"c_pf_per_m\": " << np.m_RLGC.m_C_pFPerM << std::endl;
        file << "      }" << std::endl;
        file << "    }" << ( i < m_NetParasitics.size() - 1 ? "," : "" ) << std::endl;
    }

    file << "  ]" << std::endl;
    file << "}" << std::endl;

    file.close();
    return true;
}
