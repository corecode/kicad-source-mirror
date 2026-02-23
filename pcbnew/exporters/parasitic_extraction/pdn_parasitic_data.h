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

#ifndef PDN_PARASITIC_DATA_H
#define PDN_PARASITIC_DATA_H

#include <complex>
#include <map>
#include <string>
#include <vector>

/**
 * Data structures for parasitic extraction results that can be consumed by an
 * external PDN (Power Distribution Network) analyzer.
 *
 * The extraction pipeline:
 *   KiCad PCB layout
 *     -> FastHenry .inp (R, L extraction from conductor geometry)
 *     -> FastCap .lst/.qui (C extraction from conductor/dielectric geometry)
 *     -> Parse results (Zc.mat impedance matrix, capacitance matrix)
 *     -> PDN_PARASITIC_DATA (structured for PDN analysis)
 */
namespace PDN_PARASITIC
{

/// Physical layer in the PCB stackup, with Z-position and material properties
struct STACKUP_LAYER
{
    std::string m_Name;
    bool        m_IsCopperLayer = false;
    double      m_ZPositionMM = 0.0;       ///< Z position from board top (mm)
    double      m_ThicknessMM = 0.0;       ///< Physical thickness (mm)
    double      m_EpsilonR = 1.0;          ///< Relative permittivity (dielectric layers)
    double      m_LossTangent = 0.0;       ///< Dielectric loss tangent
    double      m_ConductivitySPerM = 0.0; ///< Conductivity (copper layers), S/m
};

/// A port (terminal pair) defined for impedance extraction
struct EXTRACTION_PORT
{
    std::string m_Name;
    std::string m_PositiveNode;
    std::string m_NegativeNode;
    std::string m_NetName;             ///< Associated PCB net
    double      m_XMM = 0.0;          ///< Approximate location for visualization
    double      m_YMM = 0.0;
};

/// Complex impedance at a single frequency
struct IMPEDANCE_POINT
{
    double                 m_FrequencyHz = 0.0;
    std::complex<double>   m_Z;        ///< R + j*omega*L (ohms)
};

/// Frequency-dependent impedance between two ports (from FastHenry Zc.mat)
struct IMPEDANCE_ENTRY
{
    int m_PortI = 0;
    int m_PortJ = 0;
    std::vector<IMPEDANCE_POINT> m_Points;
};

/// Capacitance between two conductors (from FastCap output)
struct CAPACITANCE_ENTRY
{
    std::string m_ConductorI;
    std::string m_ConductorJ;
    double      m_CapacitancePF = 0.0; ///< Picofarads
};

/// RLGC per-unit-length parameters for a conductor segment, derived from extraction
struct RLGC_PARAMS
{
    double m_R_OhmPerM = 0.0;    ///< Resistance per unit length
    double m_L_nHPerM = 0.0;     ///< Inductance per unit length
    double m_G_SPerM = 0.0;      ///< Conductance per unit length (dielectric loss)
    double m_C_pFPerM = 0.0;     ///< Capacitance per unit length
};

/// A lumped parasitic element extracted for a specific net segment
struct NET_PARASITIC
{
    std::string m_NetName;
    std::string m_SegmentId;       ///< Identifier for this segment (trace/via/plane)
    double      m_LengthMM = 0.0;

    double      m_R_mOhm = 0.0;   ///< Lumped resistance (milli-ohms)
    double      m_L_nH = 0.0;     ///< Lumped inductance (nanohenries)
    double      m_C_pF = 0.0;     ///< Lumped capacitance (picofarads)

    RLGC_PARAMS m_RLGC;           ///< Per-unit-length parameters
};

/// Complete parasitic extraction results for a set of nets
struct EXTRACTION_RESULTS
{
    std::vector<STACKUP_LAYER>     m_Stackup;
    std::vector<EXTRACTION_PORT>   m_Ports;
    std::vector<IMPEDANCE_ENTRY>   m_ImpedanceMatrix;  ///< From FastHenry
    std::vector<CAPACITANCE_ENTRY> m_CapacitanceMatrix; ///< From FastCap
    std::vector<NET_PARASITIC>     m_NetParasitics;     ///< Per-segment lumped values

    /// Frequency range used for extraction
    double m_FreqMinHz = 0.0;
    double m_FreqMaxHz = 0.0;
    int    m_PointsPerDecade = 0;

    bool SerializeToJson( const std::string& aFilePath ) const;
};

/// Configuration for the extraction run
/// Explicit port specification from user pad selection
struct PORT_SPEC
{
    std::string m_Name;        ///< Port name (e.g. "Q1_VBUS")
    std::string m_NetName;     ///< Net this pad is on
    int         m_NetCode = 0; ///< Net code for geometry filtering
    double      m_XMM = 0.0;   ///< Pad position in mm
    double      m_YMM = 0.0;
    int         m_LayerId = 0;      ///< Copper layer the pad is on
    bool        m_IsGround = false; ///< True if this is a ground reference pad
};

struct EXTRACTION_CONFIG
{
    std::vector<std::string> m_NetNames;        ///< Nets to extract (empty = all power nets)

    /// FastHenry settings
    std::string m_FastHenryPath = "fasthenry";  ///< Path to FastHenry executable
    double      m_FreqMinHz = 1e3;              ///< Minimum frequency
    double      m_FreqMaxHz = 1e9;              ///< Maximum frequency
    int         m_PointsPerDecade = 5;          ///< Frequency points per decade
    int         m_NwInc = 3;                    ///< Width filament discretization
    int         m_NhInc = 2;                    ///< Height filament discretization

    /// FastCap settings
    std::string m_FastCapPath = "fastcap";      ///< Path to FastCap executable
    double      m_PanelTargetSizeMM = 0.5;      ///< Target panel size for meshing
    int         m_ViaFacets = 8;                ///< Number of facets for via barrel

    /// Ground plane meshing
    int         m_PlaneSegX = 20;               ///< Plane mesh segments in X
    int         m_PlaneSegY = 20;               ///< Plane mesh segments in Y

    /// Port generation
    bool m_GroupPadsByComponent = true; ///< Merge multi-pad ports per footprint+net
    std::vector<PORT_SPEC> m_PortSpecs; ///< Explicit pads for ports (overrides m_NetNames)

    /// Output
    std::string m_OutputDir;                    ///< Working directory for intermediate files
};

}  // namespace PDN_PARASITIC

#endif  // PDN_PARASITIC_DATA_H
