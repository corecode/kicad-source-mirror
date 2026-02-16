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

#include "parasitic_result_parser.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


bool PARASITIC_RESULT_PARSER::ParseFastHenryOutput(
    const std::string& aZcMatPath,
    std::vector<PDN_PARASITIC::EXTRACTION_PORT>& aPorts,
    std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY>& aImpedances )
{
    std::ifstream file( aZcMatPath );

    if( !file.is_open() )
        return false;

    std::string line;

    // Phase 1: Parse row definitions
    // "Row 0: node1 to node2"
    std::regex rowRegex( R"(Row\s+(\d+):\s+(\S+)\s+to\s+(\S+))" );
    std::vector<std::pair<std::string, std::string>> rowDefs;

    while( std::getline( file, line ) )
    {
        std::smatch match;

        if( std::regex_search( line, match, rowRegex ) )
        {
            rowDefs.push_back( { match[2].str(), match[3].str() } );
        }

        if( line.find( "Impedance matrix" ) != std::string::npos )
            break;
    }

    // Update port info from row definitions
    for( size_t i = 0; i < rowDefs.size() && i < aPorts.size(); i++ )
    {
        aPorts[i].m_PositiveNode = rowDefs[i].first;
        aPorts[i].m_NegativeNode = rowDefs[i].second;
    }

    // Initialize impedance entries for each port pair
    int nPorts = static_cast<int>( rowDefs.size() );
    aImpedances.clear();

    for( int i = 0; i < nPorts; i++ )
    {
        for( int j = 0; j < nPorts; j++ )
        {
            PDN_PARASITIC::IMPEDANCE_ENTRY entry;
            entry.m_PortI = i;
            entry.m_PortJ = j;
            aImpedances.push_back( entry );
        }
    }

    // Phase 2: Parse impedance matrices at each frequency
    // Format:
    //   Impedance matrix for frequency = <freq>
    //   <N> x <N>
    //   <re> <im>j  <re> <im>j ...

    // We may already have consumed the first "Impedance matrix" line above.
    // Re-read from current position.
    std::regex freqRegex( R"(Impedance matrix for frequency\s*=\s*([0-9eE.+\-]+))" );
    std::regex sizeRegex( R"((\d+)\s*x\s*(\d+))" );
    // Match complex numbers like "0.408091 +0.00182523j" or "3.4578e-06 +0.00069845j"
    std::regex complexRegex( R"(([0-9eE.+\-]+)\s+([0-9eE.+\-]+)j)" );

    // Process remaining file for frequency blocks
    // The first frequency line may already have been partially consumed
    double currentFreq = 0.0;
    bool inMatrix = false;
    int matRow = 0;
    int matSize = 0;

    // Re-check if we captured the frequency from the line we broke on
    {
        std::smatch match;

        if( std::regex_search( line, match, freqRegex ) )
        {
            currentFreq = std::stod( match[1].str() );
            inMatrix = false;
        }
    }

    while( std::getline( file, line ) )
    {
        std::smatch match;

        if( std::regex_search( line, match, freqRegex ) )
        {
            currentFreq = std::stod( match[1].str() );
            inMatrix = false;
            matRow = 0;
            continue;
        }

        if( std::regex_search( line, match, sizeRegex ) )
        {
            matSize = std::stoi( match[1].str() );
            inMatrix = true;
            matRow = 0;
            continue;
        }

        if( inMatrix && matRow < matSize )
        {
            // Parse complex values from this row
            std::sregex_iterator it( line.begin(), line.end(), complexRegex );
            std::sregex_iterator end;
            int col = 0;

            for( ; it != end && col < matSize; ++it, ++col )
            {
                double re = std::stod( ( *it )[1].str() );
                double im = std::stod( ( *it )[2].str() );

                // Find the matching impedance entry
                int idx = matRow * nPorts + col;

                if( idx < static_cast<int>( aImpedances.size() ) )
                {
                    PDN_PARASITIC::IMPEDANCE_POINT pt;
                    pt.m_FrequencyHz = currentFreq;
                    pt.m_Z = std::complex<double>( re, im );
                    aImpedances[idx].m_Points.push_back( pt );
                }
            }

            matRow++;

            if( matRow >= matSize )
                inMatrix = false;
        }
    }

    file.close();
    return !aImpedances.empty();
}


bool PARASITIC_RESULT_PARSER::ParseFastCapOutput(
    const std::string& aOutputPath,
    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY>& aCapacitances )
{
    std::ifstream file( aOutputPath );

    if( !file.is_open() )
        return false;

    std::string line;
    bool inMatrix = false;

    // Parse column headers and matrix rows
    // Format:
    //   CAPACITANCE MATRIX, nanofarads
    //                1         2         3
    //   name1  1   C11      C12       C13
    //   name2  2   C21      C22       C23
    //   name3  3   C31      C32       C33

    std::vector<std::string> conductorNames;

    while( std::getline( file, line ) )
    {
        if( line.find( "CAPACITANCE MATRIX" ) != std::string::npos )
        {
            inMatrix = true;
            // Skip the column header line
            std::getline( file, line );
            continue;
        }

        if( !inMatrix )
            continue;

        // Blank line ends the matrix
        if( line.find_first_not_of( " \t\r\n" ) == std::string::npos )
            break;

        // Parse a matrix row: "name  idx  C1  C2  C3 ..."
        std::istringstream iss( line );
        std::string name;
        int idx;

        iss >> name >> idx;

        if( iss.fail() )
            continue;

        conductorNames.push_back( name );

        std::vector<double> rowValues;
        double val;

        while( iss >> val )
            rowValues.push_back( val );

        // Store capacitance entries
        int rowIdx = static_cast<int>( conductorNames.size() ) - 1;

        for( size_t col = 0; col < rowValues.size(); col++ )
        {
            PDN_PARASITIC::CAPACITANCE_ENTRY entry;
            entry.m_ConductorI = name;

            if( col < conductorNames.size() )
                entry.m_ConductorJ = conductorNames[col];
            else
                entry.m_ConductorJ = "C" + std::to_string( col + 1 );

            // Convert nanofarads to picofarads
            entry.m_CapacitancePF = rowValues[col] * 1000.0;

            aCapacitances.push_back( entry );
        }
    }

    file.close();
    return !aCapacitances.empty();
}


void PARASITIC_RESULT_PARSER::DeriveLumpedParasitics(
    const std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY>& aImpedances,
    const std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY>& aCapacitances,
    std::vector<PDN_PARASITIC::NET_PARASITIC>& aNetParasitics )
{
    // Extract R and L from the diagonal impedance entries at a representative frequency.
    // Z_ii = R + j * omega * L
    //   R = Re(Z_ii)
    //   L = Im(Z_ii) / omega = Im(Z_ii) / (2*pi*f)

    for( const auto& imp : aImpedances )
    {
        if( imp.m_PortI != imp.m_PortJ )
            continue;  // Only self-impedance for lumped extraction

        if( imp.m_Points.empty() )
            continue;

        // Use a mid-band frequency point for lumped extraction
        size_t midIdx = imp.m_Points.size() / 2;
        const auto& pt = imp.m_Points[midIdx];

        double freq = pt.m_FrequencyHz;
        double omega = 2.0 * M_PI * freq;

        PDN_PARASITIC::NET_PARASITIC np;
        np.m_SegmentId = "Port_" + std::to_string( imp.m_PortI );
        np.m_R_mOhm = pt.m_Z.real() * 1000.0;  // Convert ohms to milli-ohms

        if( omega > 0.0 )
            np.m_L_nH = pt.m_Z.imag() / omega * 1e9;  // Convert H to nH

        aNetParasitics.push_back( np );
    }

    // Add capacitance from diagonal FastCap entries
    for( const auto& cap : aCapacitances )
    {
        if( cap.m_ConductorI != cap.m_ConductorJ )
            continue;

        // Find or create a NET_PARASITIC for this conductor
        bool found = false;

        for( auto& np : aNetParasitics )
        {
            if( np.m_NetName == cap.m_ConductorI )
            {
                np.m_C_pF = cap.m_CapacitancePF;
                found = true;
                break;
            }
        }

        if( !found )
        {
            PDN_PARASITIC::NET_PARASITIC np;
            np.m_NetName = cap.m_ConductorI;
            np.m_SegmentId = cap.m_ConductorI;
            np.m_C_pF = cap.m_CapacitancePF;
            aNetParasitics.push_back( np );
        }
    }
}
