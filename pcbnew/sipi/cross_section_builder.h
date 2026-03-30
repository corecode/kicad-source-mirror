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

#ifndef CROSS_SECTION_BUILDER_H
#define CROSS_SECTION_BUILDER_H

#include "cross_section.h"
#include <sipi/stackup_reader.h>

#include <layer_ids.h>
#include <math/vector2d.h>
#include <geometry/seg.h>

#include <map>
#include <vector>

class BOARD;
class BOARD_CONNECTED_ITEM;
class DRC_RTREE;
class PCB_TRACK;


/**
 * A neighbor conductor found in the cross-section at a sample point.
 */
struct XS_NEIGHBOR
{
    int distNm;     ///< Signed lateral distance from signal center (nm)
    int widthNm;    ///< Apparent width in cross-section (nm)
};


/**
 * Parameters for a single cross-section sample point.
 */
struct XS_BUILD_PARAMS
{
    VECTOR2I       samplePos;         ///< Board position (nm)
    VECTOR2D       sampleTangent;     ///< Unit tangent along signal trace
    PCB_LAYER_ID   signalLayer = F_Cu;
    int            signalNetCode = 0;
    int            signalWidth = 0;   ///< Effective signal width (nm), may include pad
    int            couplingHorizon = 0; ///< Max edge-to-edge search distance (nm)
    double         sampleDist = 0.0;  ///< Distance along the walked path (nm)
    LAYER_GEOMETRY layerGeom;         ///< Stackup geometry at this point
};


/**
 * Builds 2D cross-section geometry from board data at a sample point along a trace.
 *
 * Finds neighboring conductors (tracks, arcs, zone fills, pads) by intersecting
 * a perpendicular cut line through the sample point with nearby copper features.
 * Produces an XS_GEOMETRY suitable for the BEM solver.
 */
class CROSS_SECTION_BUILDER
{
public:
    CROSS_SECTION_BUILDER();

    void SetSpatialIndex( DRC_RTREE* aRtree );
    void SetBoard( const BOARD* aBoard );

    /**
     * Set path-distance map for topological neighbor exclusion.
     * Same-net items closer than 2 * couplingHorizon along the path
     * are excluded (they are bends, not coupling neighbors).
     * Null disables this filter.
     */
    void SetPathItemDistances( const std::map<BOARD_CONNECTED_ITEM*, double>* aMap );

    void SetSignalTrack( const PCB_TRACK* aTrack );

    /**
     * Find neighbor conductors crossing the cross-section cut line.
     * Returns a sorted, deduplicated vector (max 4 neighbors).
     */
    std::vector<XS_NEIGHBOR> FindNeighbors( const XS_BUILD_PARAMS& aParams ) const;

    /**
     * Build a complete XS_GEOMETRY from the signal trace and its neighbors.
     */
    XS_GEOMETRY BuildGeometry( const XS_BUILD_PARAMS& aParams,
                               const std::vector<XS_NEIGHBOR>& aNeighbors ) const;

private:
    void findTrackNeighbors( const XS_BUILD_PARAMS& aParams,
                             const SEG& aCutSeg,
                             const VECTOR2D& aNormal,
                             std::vector<XS_NEIGHBOR>& aNeighbors ) const;

    void findShapeNeighbors( const XS_BUILD_PARAMS& aParams,
                             const SEG& aCutSeg,
                             const VECTOR2D& aNormal,
                             std::vector<XS_NEIGHBOR>& aNeighbors ) const;

    void findZoneNeighbors( const XS_BUILD_PARAMS& aParams,
                            const SEG& aCutSeg,
                            const VECTOR2D& aNormal,
                            std::vector<XS_NEIGHBOR>& aNeighbors ) const;

    static void deduplicateNeighbors( std::vector<XS_NEIGHBOR>& aNeighbors,
                                      int aMaxNeighbors = 4 );

    DRC_RTREE*                                          m_rtree;
    const BOARD*                                        m_board;
    const PCB_TRACK*                                    m_signalTrack;
    const std::map<BOARD_CONNECTED_ITEM*, double>*      m_pathItemDist;
};

#endif // CROSS_SECTION_BUILDER_H
