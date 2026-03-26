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

#ifndef PDN_ANALYZER_H
#define PDN_ANALYZER_H

#include <wx/string.h>
#include <sch_sheet_path.h>
#include <vector>
#include <map>
#include <optional>

class SCHEMATIC;
class SPICE_MODEL_DOWNLOADER;


struct PARASITIC_ENTRY
{
    double esr; // Ohms
    double esl; // Henries
};


/**
 * Represents a single capacitor in a PDN with its parasitic model.
 */
struct PDN_CAPACITOR
{
    wxString refdes;       ///< Reference designator, e.g. "C1"
    double   capacitance;  ///< Value in Farads
    wxString caseSize;     ///< Metric case size, e.g. "1005", "1608"
    double   esr;          ///< Equivalent Series Resistance in Ohms
    double   esl;          ///< Equivalent Series Inductance in Henries
    wxString       netSupply;   ///< Supply rail net name
    wxString       netRef;      ///< Reference rail net name (e.g. GND)
    SCH_SHEET_PATH sheetPath;   ///< Sheet where this cap lives
};


/**
 * Parameters for the VRM (Voltage Regulator Module) source impedance model.
 *
 * The VRM is modeled as two parallel series-RL branches (main + damping) connected
 * through one hop of trace impedance.  Derived inductances are computed at netlist
 * generation time from bandwidth and rDamp.
 */
struct VRM_PARAMS
{
    double bandwidth = 10e3; ///< VRM control loop bandwidth in Hz
    double rDamp = 50e-3;    ///< Damping resistance in Ohms
    double rVrm = 1e-3;      ///< DC output resistance in Ohms
};


/**
 * Describes the voltage regulator driving a PDN supply rail.
 *
 * Populated by FindPDNNetworks() when a non-switch regulator with PT_POWER_OUT on
 * the supply net is found.  If no regulator is found, the VRM model is omitted.
 */
struct PDN_VRM
{
    wxString       refdes;    ///< Reference designator, e.g. "U3"
    SCH_SHEET_PATH sheetPath; ///< Sheet where the regulator lives
    VRM_PARAMS     params;    ///< VRM impedance model parameters
};


enum class PDN_SERIES_TYPE
{
    RESISTOR,
    FERRITE_BEAD
};


/**
 * Represents a series element (resistor or ferrite bead) connecting two power rails.
 *
 * Series elements are detected between rails that serve as supply or reference
 * rails of known PDN networks.  Two networks are coupled when their supply rails
 * are connected through series elements AND their reference rails are either
 * identical or also connected through series elements.
 *
 * Ferrite beads require a SPICE subcircuit model (Sim.Library / Sim.Name);
 * without one the connection is left open and a warning is emitted.
 */
struct PDN_SERIES_ELEMENT
{
    wxString        refdes;       ///< Reference designator, e.g. "FB1" or "R5"
    PDN_SERIES_TYPE type;         ///< Element type
    double          resistance;   ///< Resistance value in Ohms (RESISTOR only)
    wxString        spiceModel;   ///< Subcircuit model name (FERRITE_BEAD, from Sim.Name)
    wxString        spiceLibFile; ///< Library file path (FERRITE_BEAD, from Sim.Library)
    wxString        mpn;          ///< Manufacturer part number (for auto-download)
    wxString        spiceText;    ///< Inline SPICE subcircuit text (auto-fetched from cache)
    wxString        railA;        ///< First rail net name
    wxString        railB;        ///< Second rail net name
    SCH_SHEET_PATH  sheetPath;    ///< Sheet where this element lives
};


/**
 * A SPICE node to probe for transfer impedance.
 *
 * By probing remote nodes (sheet groups, coupled rails, remote VRMs) and
 * comparing to v(local), we get the transfer impedance Z21 — how much noise
 * from the remote source reaches the observation point.
 */
struct PDN_PROBE_NODE
{
    wxString label;    ///< Display label, e.g. "Sheet 2" or "+3V3_ANA (via FB1)"
    wxString nodeName; ///< SPICE node name: "rs1b" or "rail_1"
};


/**
 * Represents a power delivery network (a pair of power rails connected by capacitors).
 */
struct PDN_NETWORK
{
    wxString                   supplyRail;  ///< Supply rail net name, e.g. "+3V3"
    wxString                   refRail;    ///< Reference rail net name, e.g. "GND"
    std::vector<PDN_CAPACITOR> components; ///< Capacitors connecting the two rails
    std::optional<PDN_VRM>     vrm;        ///< VRM driving the supply rail, if found
};


/**
 * PDN impedance analysis engine.
 *
 * Scans the schematic for capacitors connecting power rail pairs, builds an ngspice
 * AC analysis netlist with parasitic ESR/ESL models, runs the simulation, and provides
 * the resulting impedance vs. frequency data.
 */
class PDN_ANALYZER
{
public:
    PDN_ANALYZER();
    ~PDN_ANALYZER();

    void SetSchematic( SCHEMATIC* aSchematic ) { m_schematic = aSchematic; }
    void SetModelDownloader( SPICE_MODEL_DOWNLOADER* aDownloader ) { m_downloader = aDownloader; }

    /**
     * Scan the schematic for PDN networks (power rail pairs connected by capacitors).
     */
    void FindPDNNetworks();

    const std::map<wxString, PDN_NETWORK>& GetNetworks() const { return m_networks; }

    const std::vector<PDN_SERIES_ELEMENT>& GetSeriesElements() const { return m_seriesElements; }

    /**
     * Find all network keys coupled to @a aNetworkKey through series elements.
     *
     * Performs BFS through the series-element graph.  Only elements with a
     * usable model (resistors always, ferrite beads only when a SPICE model
     * is assigned) are traversed.  The result always includes @a aNetworkKey
     * itself.
     */
    std::vector<wxString> GetCoupledNetworkKeys( const wxString& aNetworkKey ) const;

    /**
     * Find the network key whose supply or reference rail matches @a aNetName.
     * @return network key, or empty string if no match.
     */
    wxString FindNetworkForNetName( const wxString& aNetName ) const;

    /**
     * Find the network key containing a component with reference @a aRefdes.
     * @return network key, or empty string if no match.
     */
    wxString FindNetworkForSym( const wxString& aRefdes ) const;

    /**
     * Check if a network has components on the given sheet.
     */
    bool NetworkHasSymsOnSheet( const wxString& aNetworkKey, const SCH_SHEET_PATH& aSheet ) const;

    /**
     * Check if a network spans multiple sheets.
     */
    bool NetworkIsMultiSheet( const wxString& aNetworkKey ) const;

    /**
     * Build an ngspice netlist for AC impedance analysis of the given PDN network.
     *
     * Caps on @a aObservationSheet connect directly to the measurement node ("local")
     * while caps on other sheets are placed behind a series R+L trace impedance
     * model ("remote").  When all caps end up local, the result is equivalent to a
     * flat (zero trace impedance) topology.
     *
     * @param aNetwork the PDN network to analyze
     * @param aObservationSheet sheet used as the measurement/observation point
     * @param aTraceInductance per-hop trace inductance in Henries
     * @param aTraceResistance per-hop trace resistance in Ohms
     * @param aStartFreq start frequency in Hz
     * @param aEndFreq end frequency in Hz
     * @param aPointsPerDecade number of simulation points per decade
     * @return the netlist as a string
     */
    wxString BuildSpiceNetlist( const PDN_NETWORK& aNetwork,
                                const SCH_SHEET_PATH& aObservationSheet,
                                double aTraceInductance = 10e-9,
                                double aTraceResistance = 5e-3,
                                double aStartFreq = 1e3,
                                double aEndFreq = 1e9,
                                int aPointsPerDecade = 100 );

    /**
     * Build an ngspice netlist for AC impedance analysis with coupled-network support.
     *
     * Discovers networks coupled to @a aNetworkKey through series resistors and
     * ferrite beads, and includes them in the netlist.  The measurement point is
     * at the primary network.
     */
    wxString BuildSpiceNetlist( const wxString&       aNetworkKey,
                                const SCH_SHEET_PATH& aObservationSheet,
                                double aTraceInductance = 10e-9, double aTraceResistance = 5e-3,
                                double aStartFreq = 1e3, double aEndFreq = 1e9,
                                int aPointsPerDecade = 100 );

    /**
     * Run the AC impedance analysis for the given PDN network.
     *
     * @param aNetwork the PDN network to analyze
     * @param aObservationSheet sheet used as the measurement/observation point
     * @param aTraceInductance per-hop trace inductance in Henries
     * @param aTraceResistance per-hop trace resistance in Ohms
     * @return true if the simulation completed successfully
     */
    bool RunAnalysis( const PDN_NETWORK& aNetwork,
                      const SCH_SHEET_PATH& aObservationSheet,
                      double aTraceInductance = 10e-9,
                      double aTraceResistance = 5e-3 );

    /**
     * Run the AC impedance analysis with coupled-network support.
     */
    bool RunAnalysis( const wxString& aNetworkKey, const SCH_SHEET_PATH& aObservationSheet,
                      double aTraceInductance = 10e-9, double aTraceResistance = 5e-3 );

    const std::vector<double>& GetFrequencies() const { return m_frequencies; }
    const std::vector<double>& GetImpedance() const { return m_impedance; }

    const std::vector<double>& GetTransferImpedance() const { return m_transferImpedance; }

    /**
     * Get collected warnings, optionally filtered by network key.
     * @param aNetworkKey if non-empty, return only warnings for that network.
     */
    std::vector<wxString> GetWarnings( const wxString& aNetworkKey = wxEmptyString ) const;
    void                  ClearWarnings() { m_warnings.clear(); }

    /**
     * Extract metric case size from a footprint field string.
     * @return the metric case size string (e.g. "1005") or empty if unrecognized.
     */
    static wxString ParseCaseSize( const wxString& aFootprint );

    /**
     * Estimate parasitic ESR/ESL for a given metric case size and capacitance.
     *
     * ESL is looked up from a table indexed by case size.  ESR is computed via
     * a power-law heuristic: ESR = k(case) * C^(-0.43), derived from Murata
     * MLCC SPICE model data (Class II ceramics).
     *
     * @param aCaseSize metric case size string (e.g. "1005", "1608")
     * @param aCapacitance capacitance in Farads
     * @return parasitic entry if the case size is known, or std::nullopt.
     */
    static std::optional<PARASITIC_ENTRY> GetParasitics( const wxString& aCaseSize,
                                                         double          aCapacitance );

private:
    void addWarning( const wxString& aNetworkKey, const wxString& aWarning );

    SCHEMATIC*                                m_schematic;
    SPICE_MODEL_DOWNLOADER*                   m_downloader;
    std::map<wxString, PDN_NETWORK>           m_networks;
    std::vector<PDN_SERIES_ELEMENT>           m_seriesElements;
    std::vector<double>                       m_frequencies;
    std::vector<double>                       m_impedance;
    std::vector<PDN_PROBE_NODE>               m_probeNodes;
    std::vector<double>                       m_transferImpedance;
    std::map<wxString, std::vector<wxString>> m_warnings;
};

#endif // PDN_ANALYZER_H
