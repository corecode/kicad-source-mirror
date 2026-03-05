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

#ifndef SPICE_SUBCKT_EXPORTER_H
#define SPICE_SUBCKT_EXPORTER_H

#include <string>
#include <vector>

#include "pdn_parasitic_data.h"

/**
 * Converts parasitic extraction results (impedance + capacitance matrices) into
 * an LTspice/ngspice-compatible SPICE subcircuit (.subckt).
 *
 * The impedance matrix Z(f) from FastHenry is decomposed into:
 *   - Diagonal Z_ii(f0)  ->  series R_i + L_i per port
 *   - Off-diagonal Z_ij(f0)  ->  mutual inductance K_ij between ports
 *
 * The capacitance matrix from FastCap adds shunt C elements.
 *
 * Usage:
 *   SPICE_SUBCKT_EXPORTER exporter( results );
 *   exporter.SetReferenceFrequency( 100e3 );  // 100 kHz switching
 *   exporter.SetMinCouplingCoeff( 0.01 );      // skip K < 1%
 *   exporter.Export( "pcb_parasitics.subckt" );
 */
class SPICE_SUBCKT_EXPORTER
{
public:
    SPICE_SUBCKT_EXPORTER( const PDN_PARASITIC::EXTRACTION_RESULTS& aResults );

    /**
     * Set the reference frequency for extracting R and L from the impedance matrix.
     * Default: uses the lowest frequency in the data (closest to DC).
     */
    void SetReferenceFrequency( double aFreqHz ) { m_refFreqHz = aFreqHz; }

    /**
     * Set the minimum coupling coefficient magnitude to include in the output.
     * Coupling pairs with |K| < this value are omitted. Default: 0.01 (1%).
     */
    void SetMinCouplingCoeff( double aMinK ) { m_minK = aMinK; }

    /**
     * Set the subcircuit name. Default: "PCB_PARASITICS".
     */
    void SetSubcktName( const std::string& aName ) { m_subcktName = aName; }

    /**
     * Export the SPICE subcircuit to a file.
     * The format is compatible with both LTspice and ngspice.
     */
    bool Export( const std::string& aOutputPath ) const;

    /**
     * Generate the SPICE subcircuit as a string (for embedding in larger netlists).
     */
    std::string Generate() const;

    /// Return the number of ports (external terminals) in the subcircuit
    int GetPortCount() const;

    /// Return the port names (for documentation or symbol generation)
    std::vector<std::string> GetPortNames() const;

    struct PORT_PARASITIC
    {
        int         m_PortIndex;
        std::string m_Name;
        double      m_R_Ohm;   ///< Series resistance at reference frequency
        double      m_L_Henry; ///< Self inductance at reference frequency
    };

    struct COUPLING_PAIR
    {
        int    m_PortI;
        int    m_PortJ;
        double m_K;       ///< Coupling coefficient [-1, 1]
        double m_M_Henry; ///< Mutual inductance
    };

    /// Extract port parasitics and coupling for inspection/testing
    std::vector<PORT_PARASITIC> GetPortParasitics() const;
    std::vector<COUPLING_PAIR>  GetCouplingPairs() const;

private:
    /// Find the impedance matrix entry for Z(portI, portJ) and return the
    /// impedance point closest to the reference frequency
    std::complex<double> getZAtRefFreq( int aPortI, int aPortJ ) const;

    const PDN_PARASITIC::EXTRACTION_RESULTS& m_results;

    double      m_refFreqHz = 0.0; ///< 0 = use lowest frequency in data
    double      m_minK = 0.01;
    std::string m_subcktName = "PCB_PARASITICS";
};

#endif // SPICE_SUBCKT_EXPORTER_H
