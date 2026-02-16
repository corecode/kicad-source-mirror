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

#ifndef PARASITIC_RESULT_PARSER_H
#define PARASITIC_RESULT_PARSER_H

#include <string>
#include <vector>

#include "pdn_parasitic_data.h"

/**
 * Parses FastHenry impedance matrix output (Zc.mat) and FastCap capacitance
 * matrix output into structured PDN_PARASITIC data.
 *
 * FastHenry Zc.mat format:
 *   Row 0: node1 to node2
 *   Row 1: node3 to node4
 *   Impedance matrix for frequency = <freq_Hz>
 *   <N> x <N>
 *   <Z11_re> <Z11_im>j  <Z12_re> <Z12_im>j ...
 *   ...
 *
 * FastCap output format:
 *   CAPACITANCE MATRIX, nanofarads
 *               1         2
 *   name1  1   C11      C12
 *   name2  2   C21      C22
 */
class PARASITIC_RESULT_PARSER
{
public:
    /// Parse FastHenry Zc.mat impedance matrix file
    static bool ParseFastHenryOutput( const std::string& aZcMatPath,
                                      std::vector<PDN_PARASITIC::EXTRACTION_PORT>& aPorts,
                                      std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY>& aImpedances );

    /// Parse FastCap capacitance matrix from stdout capture or output file
    static bool ParseFastCapOutput( const std::string& aOutputPath,
                                    std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY>& aCapacitances );

    /// Derive lumped R, L, C parasitics per net segment from the raw matrices
    static void DeriveLumpedParasitics(
        const std::vector<PDN_PARASITIC::IMPEDANCE_ENTRY>& aImpedances,
        const std::vector<PDN_PARASITIC::CAPACITANCE_ENTRY>& aCapacitances,
        std::vector<PDN_PARASITIC::NET_PARASITIC>& aNetParasitics );
};

#endif  // PARASITIC_RESULT_PARSER_H
