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

#ifndef FASTCAP_EXPORTER_H
#define FASTCAP_EXPORTER_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <fstream>

#include "pdn_parasitic_data.h"

class BOARD;
class PCB_TRACK;
class PCB_VIA;
class ZONE;
class PAD;

/**
 * Exports KiCad PCB geometry to FastCap input format (.lst + .qui files).
 *
 * FastCap computes the capacitance matrix of a 3D multi-conductor system
 * embedded in piecewise-constant dielectric regions using a boundary element
 * method (BEM) with multipole acceleration.
 *
 * Mapping from PCB features to FastCap primitives:
 *   PCB_TRACK  -> Q (quad) panels for top/bottom/side surfaces of the trace
 *   PCB_VIA    -> Q (quad) panels approximating the via barrel cylinder
 *   ZONE       -> Q (quad) panels meshing the copper plane surfaces
 *   Dielectric -> D lines in .lst with epsilon_r from stackup
 *
 * Each conductor surface is written to a separate .qui geometry file,
 * and all are referenced from a master .lst list file.
 */
class FASTCAP_EXPORTER
{
public:
    FASTCAP_EXPORTER( BOARD* aBoard, const PDN_PARASITIC::EXTRACTION_CONFIG& aConfig );

    /// Export the complete FastCap input (list file + geometry files)
    bool Export( const std::string& aOutputDir, const std::string& aListFileName );

private:
    /// Build the z-coordinate map from the board stackup
    void buildLayerZMap();

    /// Build the dielectric layer information from the board stackup
    void buildDielectricLayers();

    /// Get z position (in meters) for the top surface of a copper layer
    double getLayerZTopM( int aLayerId ) const;

    /// Get z position (in meters) for the bottom surface of a copper layer
    double getLayerZBotM( int aLayerId ) const;

    /// Get copper thickness (in meters)
    double getCopperThicknessM( int aLayerId ) const;

    /// Convert KiCad internal units (nm) to meters
    static double iu2m( int aIU ) { return aIU * 1e-9; }

    /// Write the master list file (.lst) referencing all geometry files
    void writeListFile( std::ofstream& aFile, const std::string& aOutputDir );

    /// Write trace geometry to a .qui file
    /// Returns the filename written
    std::string writeTraceGeometry( const std::string& aOutputDir );

    /// Write via geometry to a .qui file
    std::string writeViaGeometry( const std::string& aOutputDir );

    /// Write zone/plane geometry to a .qui file
    std::string writePlaneGeometry( const std::string& aOutputDir );

    /// Write a dielectric interface surface
    std::string writeDielectricSurface( const std::string& aOutputDir, int aIndex,
                                        double aZM, double aExtentXM, double aExtentYM,
                                        double aOriginXM, double aOriginYM,
                                        double aMeshSizeM );

    /// Write quad panel: 4 vertices defining a flat quadrilateral
    static void writeQuad( std::ofstream& aFile, const std::string& aConductor,
                           double x1, double y1, double z1,
                           double x2, double y2, double z2,
                           double x3, double y3, double z3,
                           double x4, double y4, double z4 );

    /// Write triangle panel
    static void writeTri( std::ofstream& aFile, const std::string& aConductor,
                          double x1, double y1, double z1,
                          double x2, double y2, double z2,
                          double x3, double y3, double z3 );

    /// Mesh a rectangular trace as panels (top, bottom, two sides, two end caps)
    void meshTrace( std::ofstream& aFile, const std::string& aConductor,
                    double x1, double y1, double x2, double y2,
                    double zTop, double zBot, double width );

    /// Mesh a via barrel as a faceted cylinder
    void meshViaBarrel( std::ofstream& aFile, const std::string& aConductor,
                        double cx, double cy, double zTop, double zBot,
                        double radius, int nFacets );

    /// Mesh a rectangular copper plane as a grid of quad panels
    void meshPlane( std::ofstream& aFile, const std::string& aConductor,
                    double x1, double y1, double x2, double y2,
                    double z, int nMeshX, int nMeshY );

    bool isNetIncluded( int aNetCode ) const;

    BOARD*                              m_board;
    PDN_PARASITIC::EXTRACTION_CONFIG    m_config;

    /// Map from PCB_LAYER_ID to Z position of copper top surface in meters
    std::map<int, double>               m_layerZTopM;

    /// Map from PCB_LAYER_ID to copper thickness in meters
    std::map<int, double>               m_layerThickM;

    /// Dielectric layers: pairs of (z_position_m, epsilon_r)
    struct DIELECTRIC_INFO
    {
        double m_ZTopM;
        double m_ZBotM;
        double m_EpsilonR;
        double m_LossTangent;
    };

    std::vector<DIELECTRIC_INFO>        m_dielectrics;

    /// Set of net codes to extract
    std::set<int>                       m_netCodes;

    /// Board bounding box in meters (for dielectric surface extent)
    double m_boardMinXM = 0.0;
    double m_boardMinYM = 0.0;
    double m_boardMaxXM = 0.0;
    double m_boardMaxYM = 0.0;

    /// List of geometry files and their properties for the .lst file
    struct GEOMETRY_FILE_ENTRY
    {
        std::string m_FileName;
        char        m_Type;       ///< 'C' = conductor, 'D' = dielectric, 'B' = both
        double      m_OuterPerm;
        double      m_InnerPerm;  ///< Only for D/B type
        double      m_RefX, m_RefY, m_RefZ;  ///< Reference point for D/B
    };

    std::vector<GEOMETRY_FILE_ENTRY>    m_geoFiles;
};

#endif  // FASTCAP_EXPORTER_H
