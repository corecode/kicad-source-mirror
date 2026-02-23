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

#include "spice_subckt_exporter.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


SPICE_SUBCKT_EXPORTER::SPICE_SUBCKT_EXPORTER( const PDN_PARASITIC::EXTRACTION_RESULTS& aResults ) :
        m_results( aResults )
{
}


std::complex<double> SPICE_SUBCKT_EXPORTER::getZAtRefFreq( int aPortI, int aPortJ ) const
{
    // Determine the actual reference frequency to use
    double targetFreq = m_refFreqHz;

    if( targetFreq <= 0.0 && !m_results.m_ImpedanceMatrix.empty()
        && !m_results.m_ImpedanceMatrix[0].m_Points.empty() )
    {
        // Use the lowest frequency in the data (closest to DC)
        targetFreq = m_results.m_ImpedanceMatrix[0].m_Points[0].m_FrequencyHz;
    }

    // Find the impedance entry for (portI, portJ)
    for( const auto& entry : m_results.m_ImpedanceMatrix )
    {
        if( entry.m_PortI == aPortI && entry.m_PortJ == aPortJ )
        {
            if( entry.m_Points.empty() )
                return { 0.0, 0.0 };

            // Find the frequency point closest to targetFreq
            const PDN_PARASITIC::IMPEDANCE_POINT* best = &entry.m_Points[0];
            double                                bestDist =
                    std::abs( std::log10( best->m_FrequencyHz ) - std::log10( targetFreq ) );

            for( const auto& pt : entry.m_Points )
            {
                double dist = std::abs( std::log10( pt.m_FrequencyHz ) - std::log10( targetFreq ) );

                if( dist < bestDist )
                {
                    best = &pt;
                    bestDist = dist;
                }
            }

            return best->m_Z;
        }
    }

    return { 0.0, 0.0 };
}


std::vector<SPICE_SUBCKT_EXPORTER::PORT_PARASITIC> SPICE_SUBCKT_EXPORTER::GetPortParasitics() const
{
    std::vector<PORT_PARASITIC> result;

    double refFreq = m_refFreqHz;

    if( refFreq <= 0.0 && !m_results.m_ImpedanceMatrix.empty()
        && !m_results.m_ImpedanceMatrix[0].m_Points.empty() )
    {
        refFreq = m_results.m_ImpedanceMatrix[0].m_Points[0].m_FrequencyHz;
    }

    double omega = 2.0 * M_PI * refFreq;

    for( size_t i = 0; i < m_results.m_Ports.size(); i++ )
    {
        auto Z = getZAtRefFreq( static_cast<int>( i ), static_cast<int>( i ) );

        PORT_PARASITIC pp;
        pp.m_PortIndex = static_cast<int>( i );
        pp.m_Name = m_results.m_Ports[i].m_Name;
        pp.m_R_Ohm = Z.real();
        pp.m_L_Henry = ( omega > 0.0 ) ? Z.imag() / omega : 0.0;
        result.push_back( pp );
    }

    return result;
}


std::vector<SPICE_SUBCKT_EXPORTER::COUPLING_PAIR> SPICE_SUBCKT_EXPORTER::GetCouplingPairs() const
{
    std::vector<COUPLING_PAIR> result;

    auto portParasitics = GetPortParasitics();

    double refFreq = m_refFreqHz;

    if( refFreq <= 0.0 && !m_results.m_ImpedanceMatrix.empty()
        && !m_results.m_ImpedanceMatrix[0].m_Points.empty() )
    {
        refFreq = m_results.m_ImpedanceMatrix[0].m_Points[0].m_FrequencyHz;
    }

    double omega = 2.0 * M_PI * refFreq;

    for( size_t i = 0; i < m_results.m_Ports.size(); i++ )
    {
        for( size_t j = i + 1; j < m_results.m_Ports.size(); j++ )
        {
            auto   Zij = getZAtRefFreq( static_cast<int>( i ), static_cast<int>( j ) );
            double Mij = ( omega > 0.0 ) ? Zij.imag() / omega : 0.0;

            double Li = portParasitics[i].m_L_Henry;
            double Lj = portParasitics[j].m_L_Henry;

            double K = 0.0;

            if( Li > 0.0 && Lj > 0.0 )
                K = Mij / std::sqrt( Li * Lj );

            // Clamp to valid range
            K = std::max( -1.0, std::min( 1.0, K ) );

            if( std::abs( K ) >= m_minK )
            {
                COUPLING_PAIR cp;
                cp.m_PortI = static_cast<int>( i );
                cp.m_PortJ = static_cast<int>( j );
                cp.m_K = K;
                cp.m_M_Henry = Mij;
                result.push_back( cp );
            }
        }
    }

    return result;
}


int SPICE_SUBCKT_EXPORTER::GetPortCount() const
{
    return static_cast<int>( m_results.m_Ports.size() );
}


std::vector<std::string> SPICE_SUBCKT_EXPORTER::GetPortNames() const
{
    std::vector<std::string> names;

    for( const auto& port : m_results.m_Ports )
        names.push_back( port.m_Name );

    return names;
}


std::string SPICE_SUBCKT_EXPORTER::Generate() const
{
    std::ostringstream out;
    out << std::scientific << std::setprecision( 6 );

    // Determine reference frequency
    double refFreq = m_refFreqHz;

    if( refFreq <= 0.0 && !m_results.m_ImpedanceMatrix.empty()
        && !m_results.m_ImpedanceMatrix[0].m_Points.empty() )
    {
        refFreq = m_results.m_ImpedanceMatrix[0].m_Points[0].m_FrequencyHz;
    }

    // Header
    out << "* SPICE subcircuit: PCB parasitic model\n";
    out << "* Generated by KiCad parasitic extraction\n";
    out << "* Reference frequency: " << refFreq << " Hz\n";
    out << "* Ports: " << m_results.m_Ports.size() << "\n";
    out << "*\n";

    // Port documentation
    for( size_t i = 0; i < m_results.m_Ports.size(); i++ )
    {
        const auto& port = m_results.m_Ports[i];
        out << "* Port " << ( i + 1 ) << ": " << port.m_Name << " (net=" << port.m_NetName << ", +"
            << port.m_PositiveNode << " -" << port.m_NegativeNode << ", at " << std::fixed
            << std::setprecision( 2 ) << port.m_XMM << "," << port.m_YMM << " mm)\n";
    }

    out << std::scientific << std::setprecision( 6 );
    out << "*\n";

    // Build port terminal names for subcircuit interface.
    // Each port has a positive and negative terminal.
    std::vector<std::string> posTerminals;
    std::vector<std::string> negTerminals;

    for( size_t i = 0; i < m_results.m_Ports.size(); i++ )
    {
        const auto& port = m_results.m_Ports[i];
        std::string sanitized = port.m_Name;

        // Replace non-alphanumeric chars with underscores for SPICE compatibility
        for( char& c : sanitized )
        {
            if( !std::isalnum( c ) && c != '_' )
                c = '_';
        }

        posTerminals.push_back( sanitized + "_p" );
        negTerminals.push_back( sanitized + "_n" );
    }

    // Subcircuit header
    out << ".subckt " << m_subcktName;

    for( size_t i = 0; i < m_results.m_Ports.size(); i++ )
        out << " " << posTerminals[i] << " " << negTerminals[i];

    out << "\n";

    // Self-impedance: series R + L per port
    auto portParasitics = GetPortParasitics();

    out << "*\n";
    out << "* --- Self impedance per port (series R + L) ---\n";

    for( size_t i = 0; i < portParasitics.size(); i++ )
    {
        const auto& pp = portParasitics[i];
        std::string midNode = "mid_" + std::to_string( i + 1 );

        out << "R" << ( i + 1 ) << " " << posTerminals[i] << " " << midNode << " " << pp.m_R_Ohm
            << "\n";
        out << "L" << ( i + 1 ) << " " << midNode << " " << negTerminals[i] << " " << pp.m_L_Henry
            << "\n";
    }

    // Mutual coupling: K statements for significant port pairs
    auto couplingPairs = GetCouplingPairs();

    if( !couplingPairs.empty() )
    {
        out << "*\n";
        out << "* --- Mutual coupling between ports ---\n";

        for( const auto& cp : couplingPairs )
        {
            out << "K" << ( cp.m_PortI + 1 ) << "_" << ( cp.m_PortJ + 1 ) << " L"
                << ( cp.m_PortI + 1 ) << " L" << ( cp.m_PortJ + 1 ) << " " << std::fixed
                << std::setprecision( 6 ) << cp.m_K << "\n";
        }

        out << std::scientific << std::setprecision( 6 );
    }

    // Capacitance: shunt C from FastCap
    if( !m_results.m_CapacitanceMatrix.empty() )
    {
        out << "*\n";
        out << "* --- Capacitance between conductors (from FastCap) ---\n";

        int capIdx = 1;

        for( const auto& cap : m_results.m_CapacitanceMatrix )
        {
            // Only include positive (self) and negative (mutual) capacitance terms
            // that have meaningful magnitude
            if( std::abs( cap.m_CapacitancePF ) < 1e-6 )
                continue;

            double capFarads = cap.m_CapacitancePF * 1e-12;

            out << "* C[" << cap.m_ConductorI << ", " << cap.m_ConductorJ
                << "] = " << cap.m_CapacitancePF << " pF\n";
            out << "C" << capIdx << " cap_" << cap.m_ConductorI << " cap_" << cap.m_ConductorJ
                << " " << capFarads << "\n";
            capIdx++;
        }
    }

    out << "*\n";
    out << ".ends " << m_subcktName << "\n";

    return out.str();
}


bool SPICE_SUBCKT_EXPORTER::Export( const std::string& aOutputPath ) const
{
    std::ofstream file( aOutputPath );

    if( !file.is_open() )
        return false;

    file << Generate();
    file.close();

    return file.good();
}
