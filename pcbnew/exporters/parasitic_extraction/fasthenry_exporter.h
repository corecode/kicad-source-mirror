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

#ifndef FASTHENRY_EXPORTER_H
#define FASTHENRY_EXPORTER_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <fstream>

#include "pdn_parasitic_data.h"

class BOARD;
class BOARD_STACKUP;
class BOARD_STACKUP_ITEM;
class PCB_TRACK;
class PCB_ARC;
class PCB_VIA;
class ZONE;
class PAD;

/**
 * Exports KiCad PCB geometry to FastHenry input format (.inp).
 *
 * FastHenry computes frequency-dependent resistance and inductance of
 * 3D conductor geometries using a multipole-accelerated approach.
 *
 * Mapping from PCB features to FastHenry primitives:
 *   PCB_TRACK  -> N (node) + E (segment) with w (trace width), h (copper thickness)
 *   PCB_VIA    -> N (node pair at different z) + E (vertical segment)
 *   ZONE       -> G (ground plane) with thickness, conductivity, seg1/seg2 mesh
 *   PAD        -> N (node) at pad center, connected via .equiv to trace/via nodes
 *
 * The stackup provides z-coordinates for each copper layer and copper thickness.
 */
class FASTHENRY_EXPORTER
{
public:
    FASTHENRY_EXPORTER( BOARD* aBoard, const PDN_PARASITIC::EXTRACTION_CONFIG& aConfig );

    bool Export( const std::string& aOutputPath );

    const std::vector<PDN_PARASITIC::EXTRACTION_PORT>& GetPorts() const { return m_ports; }

private:
    /// Build the z-coordinate map from the board stackup
    void buildLayerZMap();

    /// Get z position (in mm) for the center of a copper layer
    double getLayerZMM( int aLayerId ) const;

    /// Get copper thickness (in mm) for a layer
    double getCopperThicknessMM( int aLayerId ) const;

    /// Convert KiCad internal units (nm) to millimeters
    static double iu2mm( int aIU ) { return aIU / 1e6; }

    /// Pre-register node names for all traces and vias (populates m_nodeMap)
    void registerNodes();

    /// Write the file header and units
    void writeHeader( std::ofstream& aFile );

    /// Write default segment parameters
    void writeDefaults( std::ofstream& aFile );

    /// Write ground/power plane zones as FastHenry G (ground plane) elements
    void writePlanes( std::ofstream& aFile );

    /// Write trace segments as N (nodes) and E (segments)
    void writeTraces( std::ofstream& aFile );

    /// Write vias as vertical segments
    void writeVias( std::ofstream& aFile );

    /// Write node definitions for pads not already defined by traces/vias
    void writePadNodes( std::ofstream& aFile );

    /// Write .equiv statements to connect co-located nodes
    void writeEquivNodes( std::ofstream& aFile );

    /// Write .external port definitions
    void writeExternals( std::ofstream& aFile );

    /// Write frequency sweep
    void writeFrequency( std::ofstream& aFile );

    /// Generate a unique node name for a track endpoint
    std::string makeTrackNodeName( const PCB_TRACK* aTrack, bool aStart );

    /// Generate a unique node name for a via at a specific layer
    std::string makeViaNodeName( const PCB_VIA* aVia, int aLayerId );

    /// Generate a unique node name for a pad
    std::string makePadNodeName( const PAD* aPad, int aLayerId );

    /// Generate a unique node name on a ground plane
    std::string makePlaneNodeName( const ZONE* aZone, double aXMM, double aYMM );

    /// Check if a net should be included in extraction
    bool isNetIncluded( int aNetCode ) const;

    /// Find pads that serve as ports (VRM connections, decap connections, IC power pins)
    void identifyPorts();

    /// Create ports from explicit PORT_SPEC entries (user-selected pads)
    void identifyExplicitPorts();

    /// Connect power vias to ground planes at layer crossings to form return paths.
    /// Skips vias whose nodes are used as port terminals.
    void identifyReturnPaths();

    BOARD*                                  m_board;
    PDN_PARASITIC::EXTRACTION_CONFIG        m_config;

    /// Map from PCB_LAYER_ID to Z position in mm (center of copper)
    std::map<int, double>                   m_layerZMM;

    /// Map from PCB_LAYER_ID to copper thickness in mm
    std::map<int, double>                   m_layerThickMM;

    /// Set of net codes to extract
    std::set<int>                           m_netCodes;

    /// Node counter for unique naming
    int                                     m_nodeCounter = 0;

    /// Segment counter for unique naming
    int                                     m_segCounter = 0;

    /// Plane counter
    int                                     m_planeCounter = 0;

    /// Map from (x_iu, y_iu, layer) to node name for de-duplication
    std::map<std::tuple<int, int, int>, std::string> m_nodeMap;

    /// Map from (x_iu, y_iu, layer) to net code (parallel to m_nodeMap)
    std::map<std::tuple<int, int, int>, int> m_nodeNetCode;

    /// Set of node names that have had their coordinate definition emitted
    std::set<std::string> m_definedNodes;

    /// Pairs of nodes that need .equiv (co-located nodes from different elements)
    std::vector<std::pair<std::string, std::string>> m_equivPairs;

    /// Ground plane nodes for port references: { name, x_mm, y_mm, z_mm }
    struct PLANE_PORT_NODE
    {
        std::string m_Name;
        double      m_XMM;
        double      m_YMM;
        double      m_ZMM;
    };

    std::vector<PLANE_PORT_NODE> m_groundPlanePortNodes;

    /// Port definitions
    std::vector<PDN_PARASITIC::EXTRACTION_PORT> m_ports;
};

#endif  // FASTHENRY_EXPORTER_H
