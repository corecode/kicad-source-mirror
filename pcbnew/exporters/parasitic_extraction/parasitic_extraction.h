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

#ifndef PARASITIC_EXTRACTION_H
#define PARASITIC_EXTRACTION_H

#include <string>
#include <functional>

#include "pdn_parasitic_data.h"

class BOARD;

/**
 * Orchestrates parasitic extraction from a KiCad PCB layout using
 * FastHenry (R, L extraction) and FastCap (C extraction).
 *
 * Workflow:
 *   1. Export PCB geometry to FastHenry .inp format
 *   2. Export PCB geometry to FastCap .lst/.qui format
 *   3. Run FastHenry to compute impedance matrix (R + jwL)
 *   4. Run FastCap to compute capacitance matrix (C)
 *   5. Parse results and derive lumped/distributed RLGC parasitics
 *   6. Serialize results to JSON for consumption by external PDN analyzer
 *
 * The module can also operate in export-only mode (steps 1-2), producing
 * the geometry files for manual execution of FastHenry/FastCap.
 */
class PARASITIC_EXTRACTION
{
public:
    using ProgressCallback = std::function<void( const std::string& aMessage, int aPercent )>;

    PARASITIC_EXTRACTION( BOARD* aBoard, const PDN_PARASITIC::EXTRACTION_CONFIG& aConfig );

    /// Set a callback for progress reporting
    void SetProgressCallback( ProgressCallback aCallback ) { m_progress = aCallback; }

    /// Export-only mode: generate FastHenry and FastCap input files without running solvers.
    /// Returns true on success. Files are written to the configured output directory.
    bool ExportGeometry();

    /// Full extraction: export geometry, run solvers, parse results.
    /// Returns true on success. Results stored internally.
    bool RunExtraction();

    /// Get the extraction results (valid after RunExtraction succeeds)
    const PDN_PARASITIC::EXTRACTION_RESULTS& GetResults() const { return m_results; }

    /// Get the error message from the last failed operation
    const std::string& GetErrorMessage() const { return m_errorMsg; }

    /// Get the output directory path
    const std::string& GetOutputDir() const { return m_config.m_OutputDir; }

    /// Serialize results to JSON for the external PDN analyzer
    bool SerializeResults( const std::string& aJsonPath ) const;

private:
    /// Run FastHenry on the exported .inp file
    bool runFastHenry( const std::string& aInputPath, const std::string& aOutputDir );

    /// Run FastCap on the exported .lst file
    bool runFastCap( const std::string& aListPath, const std::string& aOutputDir );

    /// Build stackup data for the results
    void buildStackupResults();

    /// Report progress
    void reportProgress( const std::string& aMsg, int aPct );

    BOARD*                              m_board;
    PDN_PARASITIC::EXTRACTION_CONFIG    m_config;
    PDN_PARASITIC::EXTRACTION_RESULTS   m_results;
    ProgressCallback                    m_progress;
    std::string                         m_errorMsg;
};

#endif  // PARASITIC_EXTRACTION_H
