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

#ifndef TRACE_PATH_WALKER_H
#define TRACE_PATH_WALKER_H

#include <math/vector2d.h>
#include <layer_ids.h>
#include <map>
#include <optional>
#include <set>
#include <vector>

class BOARD;
class BOARD_CONNECTED_ITEM;
class PCB_ARC;
class PCB_TRACK;
class PCB_VIA;
class PAD;


/**
 * How a trace path terminated at one end.
 */
enum class PATH_TERMINUS
{
    PAD,            ///< Ended at a pad (normal termination)
    JUNCTION,       ///< Stopped at a T-junction / branch point
    DEAD_END,       ///< Reached a dangling endpoint (no pad, no branch)
    NONE            ///< Walk didn't reach this end (e.g. start track is a via)
};


/**
 * Summary of a completed trace walk.
 */
/**
 * Info about a junction where the walker stopped.
 */
struct PATH_JUNCTION
{
    VECTOR2I                                position;  ///< Board location of the junction
    std::vector<BOARD_CONNECTED_ITEM*>      branches;  ///< Unvisited items leading away
};


/**
 * Summary of a completed trace walk.
 */
struct WALK_RESULT
{
    PATH_TERMINUS              startTerminus = PATH_TERMINUS::NONE;
    PATH_TERMINUS              endTerminus = PATH_TERMINUS::NONE;
    int                        totalSegmentsOnNet = 0;  ///< Total non-via segments on the net
    int                        segmentsVisited = 0;     ///< How many we actually walked
    std::optional<PATH_JUNCTION> startJunction;          ///< Set if startTerminus == JUNCTION
    std::optional<PATH_JUNCTION> endJunction;            ///< Set if endTerminus == JUNCTION

    bool isComplete() const { return segmentsVisited == totalSegmentsOnNet; }
};


/**
 * A point along a walked trace path.
 */
struct PATH_POINT
{
    VECTOR2I                position;       ///< Board coordinates (nm)
    VECTOR2D                tangent;        ///< Unit tangent direction at this point
    PCB_LAYER_ID            layer;          ///< Copper layer
    double                  distFromStart;  ///< Cumulative distance from path start (nm)
    BOARD_CONNECTED_ITEM*   item;           ///< Source track segment or via
    bool                    isVia;          ///< True if this point is a via transition
};


/**
 * Walks a connected trace path on a single net, building an ordered sequence
 * of PATH_POINTs from one end to the other.
 *
 * Current limitations (MVP):
 * - Single path; stops at T-junctions (degree > 2) rather than branching
 * - No differential pair detection (added in Stage 5)
 */
class TRACE_PATH_WALKER
{
public:
    TRACE_PATH_WALKER( const BOARD* aBoard );

    /**
     * Walk the trace path starting from the given track segment.
     *
     * Builds the adjacency graph for the track's net, then walks in both
     * directions from the start track, concatenating the result into a single
     * ordered path from one terminal to the other.
     *
     * @param aStartTrack  A track segment to start walking from.
     * @return true if a path was successfully built (at least one segment).
     */
    bool Walk( PCB_TRACK* aStartTrack );

    /**
     * Walk the trace path for the given net, starting from the pad nearest
     * to the given position.
     *
     * @param aNetCode  Net code to walk.
     * @param aFrom     Optional starting position hint; picks the nearest pad.
     * @return true if a path was successfully built.
     */
    bool WalkNet( int aNetCode, const VECTOR2I& aFrom = VECTOR2I( 0, 0 ) );

    const std::vector<PATH_POINT>& GetPath() const { return m_path; }

    double GetTotalLength() const;

    /**
     * Get the pads found at the start and end of the walked path (if any).
     */
    PAD* GetStartPad() const { return m_startPad; }
    PAD* GetEndPad() const { return m_endPad; }

    const WALK_RESULT& GetResult() const { return m_result; }

private:
    /**
     * Build the adjacency graph for all tracks/vias on the given net.
     * Nodes are VECTOR2I endpoints; edges are track segments and vias.
     */
    void buildAdjacency( int aNetCode );

    /**
     * Walk from a starting endpoint in one direction until a terminal
     * (pad, dead end, or junction with degree > 2) is reached.
     *
     * @param aStartPos     Starting endpoint position.
     * @param aFirstItem    The first item to traverse (determines direction).
     * @param aVisited      Set of already-visited items (updated in place).
     * @param aPoints       Output points appended here.
     * @param aCumulDist    Running cumulative distance (updated in place).
     */
    PATH_TERMINUS walkDirection( const VECTOR2I& aStartPos, BOARD_CONNECTED_ITEM* aFirstItem,
                                std::set<BOARD_CONNECTED_ITEM*>& aVisited,
                                std::vector<PATH_POINT>& aPoints, double& aCumulDist,
                                std::optional<PATH_JUNCTION>& aJunction );

    /**
     * Reverse the walked path so that the start and end terminals swap.
     * Flips tangent vectors and re-numbers cumulative distances.
     */
    void reversePath();

    /**
     * Get the "other" endpoint of a track segment given one endpoint.
     * For vias, both endpoints are the same (the via position).
     */
    VECTOR2I otherEnd( BOARD_CONNECTED_ITEM* aItem, const VECTOR2I& aFrom ) const;

    /**
     * Get the length of a track item (segment or arc).
     */
    double itemLength( BOARD_CONNECTED_ITEM* aItem ) const;

    /**
     * Emit interpolated PATH_POINTs along a PCB_ARC, with correct tangent
     * directions perpendicular to the arc radius at each sample point.
     *
     * @param aArc       The arc to interpolate along.
     * @param aEntryPos  The endpoint where we enter the arc.
     * @param aPoints    Output points appended here.
     * @param aCumulDist Running cumulative distance (updated in place).
     */
    void emitArcPoints( PCB_ARC* aArc, const VECTOR2I& aEntryPos,
                        std::vector<PATH_POINT>& aPoints, double& aCumulDist );

    /**
     * Find the pad at the given position on the given net, if any.
     */
    PAD* findPadAt( const VECTOR2I& aPos, int aNetCode ) const;

    const BOARD* m_board;

    // Adjacency: endpoint position → list of connected items at that endpoint
    std::map<VECTOR2I, std::vector<BOARD_CONNECTED_ITEM*>> m_adjacency;

    // Pads on the current net, indexed by position
    std::map<VECTOR2I, PAD*> m_padMap;

    // Result
    std::vector<PATH_POINT> m_path;
    PAD* m_startPad;
    PAD* m_endPad;
    WALK_RESULT m_result;
};

#endif // TRACE_PATH_WALKER_H
