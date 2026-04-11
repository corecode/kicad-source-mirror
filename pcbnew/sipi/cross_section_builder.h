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
#include <math/box2.h>
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
 * A groundwire span found by intersecting the cross-section cut line
 * with zone fills on a reference plane layer.
 */
struct XS_GROUNDWIRE
{
    int    lateralNm;    ///< Signed center distance from signal center (nm)
    int    widthNm;      ///< Width of copper span in cross-section (nm)
    double yPositionM;   ///< Vertical position in stackup coordinates (meters)
    double thicknessM;   ///< Copper thickness (meters)
    bool   openLeft = false;   ///< Left edge is truncated (copper continues beyond)
    bool   openRight = false;  ///< Right edge is truncated (copper continues beyond)
};


/**
 * A diff-pair partner conductor found on the cross-section cut line.
 */
struct XS_DIFF_PAIR_CONDUCTOR
{
    int    lateralNm = 0;   ///< Signed lateral distance from signal center (nm)
    int    widthNm = 0;     ///< Track width (nm)
    bool   found = false;   ///< True if coupled-net track was found on cut line
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
    int            coupledNetCode = 0; ///< Coupled diff-pair net code (0 = not a diff pair)
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
     * Precompute zone fill edges near the trace path for fast per-sample queries.
     *
     * Walks all zone fill polygon edges on the given layers once, filtering by
     * proximity to the trace bounding box.  Subsequent calls to
     * findZoneFillEdgeCrossings on a precomputed layer use the cached edges
     * instead of walking the full polygon.
     *
     * @param aLayers    Copper layers to precompute (typically the reference layers)
     * @param aTraceBBox Bounding box of the trace path (in board coordinates, nm)
     * @param aExtent    Maximum perpendicular search distance (nm)
     */
    void PrecomputeNearbyFillEdges( const std::vector<PCB_LAYER_ID>& aLayers,
                                     const BOX2I& aTraceBBox, int aExtent );

    /**
     * Return the signal conductor width at the cross-section, accounting for
     * signal-net zone fills (teardrops, copper pours) that widen the conductor
     * beyond the trace width.
     *
     * Checks whether the sample position falls inside a zone fill on the signal
     * net and layer, and if so, intersects the fill polygon with the
     * perpendicular cut line to measure the actual copper extent.
     *
     * @param aParams  Sample parameters (position, tangent, signal layer/net, width)
     * @return The effective signal width (nm), >= aParams.signalWidth.
     */
    int FindSignalZoneWidth( const XS_BUILD_PARAMS& aParams ) const;

    /**
     * Find neighbor conductors crossing the cross-section cut line.
     * Returns a sorted, deduplicated vector (max 4 neighbors).
     */
    std::vector<XS_NEIGHBOR> FindNeighbors( const XS_BUILD_PARAMS& aParams ) const;

    /**
     * Find groundwire copper spans on reference and intermediate layers.
     *
     * Intersects the cross-section cut line with zone fills on:
     * 1. Intermediate layers (skipped during reference plane search)
     * 2. The reference layers themselves — if a ground edge is found within
     *    the coupling horizon, the reference is promoted to groundwires and
     *    the image ground is shifted to a deeper layer (or virtual earth).
     *
     * @param aParams    Sample parameters (position, tangent, layer geometry)
     * @param aLayerGeom Layer geometry, may be modified if a reference is demoted
     * @return Groundwire spans suitable for adding to XS_GEOMETRY as ground conductors
     */
    std::vector<XS_GROUNDWIRE> FindGroundWires( const XS_BUILD_PARAMS& aParams,
                                                LAYER_GEOMETRY& aLayerGeom ) const;

    /**
     * Find the diff-pair partner conductor on the cross-section cut line.
     *
     * Searches the R-tree for tracks on the coupled net that intersect the
     * perpendicular cut line.  If multiple candidates are found, the closest
     * one (by lateral distance) is returned.
     *
     * @param aParams  Sample parameters (must have coupledNetCode > 0).
     * @return Result with found=true and lateral position if a partner was detected.
     */
    XS_DIFF_PAIR_CONDUCTOR FindDiffPairConductor( const XS_BUILD_PARAMS& aParams ) const;

    /**
     * Build a complete XS_GEOMETRY from the signal trace, its neighbors,
     * and any groundwires from intermediate reference layers.
     *
     * @param aDiffPair  Optional diff-pair partner.  When present and found,
     *                   placed as conductors[1] (signal index 1) for BEM Zdiff
     *                   extraction.  Any neighbor at the same position is skipped.
     */
    XS_GEOMETRY BuildGeometry( const XS_BUILD_PARAMS& aParams,
                               const std::vector<XS_NEIGHBOR>& aNeighbors,
                               const std::vector<XS_GROUNDWIRE>& aGroundWires = {},
                               const XS_DIFF_PAIR_CONDUCTOR* aDiffPair = nullptr ) const;

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

    /**
     * Find lateral distances where zone fill edges cross a perpendicular cut line.
     *
     * Intersects the cut segment with fill polygon outlines and holes on the given
     * layer.  Uses the R-tree for spatial filtering when available; falls back to
     * iterating all board zones otherwise.
     *
     * @param aCutSeg    Cut line segment (perpendicular to trace direction)
     * @param aSamplePos Board position of the signal center (nm)
     * @param aNormal    Unit normal (perpendicular to trace tangent, defines lateral sign)
     * @param aLayer     Copper layer to query
     * @param aRadius    Search radius from aSamplePos (nm)
     * @return Sorted vector of signed lateral distances (nm) where edges cross the cut line
     */
    std::vector<double> findZoneFillEdgeCrossings( const SEG& aCutSeg,
                                                    const VECTOR2I& aSamplePos,
                                                    const VECTOR2D& aNormal,
                                                    PCB_LAYER_ID aLayer,
                                                    int aRadius,
                                                    int aSkipNetCode = 0 ) const;

    static void deduplicateNeighbors( std::vector<XS_NEIGHBOR>& aNeighbors,
                                      int aMaxNeighbors = 4 );

    DRC_RTREE*                                          m_rtree;
    const BOARD*                                        m_board;
    const PCB_TRACK*                                    m_signalTrack;
    const std::map<BOARD_CONNECTED_ITEM*, double>*      m_pathItemDist;

    /// Zone fill edge segment with its net code for net-aware filtering.
    struct FILL_EDGE
    {
        SEG seg;
        int netCode;
    };

    /// Precomputed zone fill edge segments near the trace, keyed by layer.
    /// When populated, findZoneFillEdgeCrossings uses these instead of
    /// walking full zone fill polygons.
    std::map<PCB_LAYER_ID, std::vector<FILL_EDGE>>      m_nearbyFillEdges;
};

#endif // CROSS_SECTION_BUILDER_H
