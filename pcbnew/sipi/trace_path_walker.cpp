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

#include "trace_path_walker.h"

#include <board.h>
#include <pcb_track.h>
#include <pad.h>
#include <core/typeinfo.h>
#include <connectivity/connectivity_data.h>

#include <algorithm>
#include <cmath>


TRACE_PATH_WALKER::TRACE_PATH_WALKER( const BOARD* aBoard ) :
        m_board( aBoard ),
        m_startPad( nullptr ),
        m_endPad( nullptr )
{
}


bool TRACE_PATH_WALKER::Walk( PCB_TRACK* aStartTrack )
{
    m_path.clear();
    m_startPad = nullptr;
    m_endPad = nullptr;
    m_result = WALK_RESULT();

    if( !aStartTrack || !m_board )
        return false;

    int netCode = aStartTrack->GetNetCode();

    if( netCode <= 0 )
        return false;

    buildPadMap( netCode );

    // Count total non-via segments on this net
    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() == netCode && track->Type() != PCB_VIA_T )
            m_result.totalSegmentsOnNet++;
    }

    // Walk in both directions from the start track.
    // Direction A: from GetStart() backward to a terminal
    // Direction B: from GetEnd() forward to a terminal

    std::set<BOARD_CONNECTED_ITEM*> visited;
    visited.insert( aStartTrack );

    VECTOR2I startPos = aStartTrack->GetStart();
    VECTOR2I endPos = aStartTrack->GetEnd();

    // For vias, start and end are the same point
    if( aStartTrack->Type() == PCB_VIA_T )
    {
        endPos = startPos;
    }

    // Walk backward from start endpoint
    std::vector<PATH_POINT>      backwardPoints;
    double                       backDist = 0.0;
    PATH_TERMINUS                backTerminus = PATH_TERMINUS::NONE;
    std::optional<PATH_JUNCTION> backJunction;

    BOARD_CONNECTED_ITEM* backVia = nullptr;
    BOARD_CONNECTED_ITEM* backNext = nullptr;
    int                   backDegree = 0;

    findNeighborsAt( aStartTrack, startPos, visited, backVia, backNext, backDegree );

    BOARD_CONNECTED_ITEM* backFirst = backVia ? backVia : backNext;

    if( backFirst )
    {
        backTerminus = walkDirection( startPos, backFirst, visited,
                                     backwardPoints, backDist, backJunction );
    }

    // Check if start endpoint itself is a pad (backward walk didn't happen)
    if( backTerminus == PATH_TERMINUS::NONE && m_padMap.count( startPos ) )
        backTerminus = PATH_TERMINUS::PAD;
    else if( backTerminus == PATH_TERMINUS::NONE )
        backTerminus = PATH_TERMINUS::DEAD_END;

    // Reverse the backward points so they go from terminal → start
    std::reverse( backwardPoints.begin(), backwardPoints.end() );

    // Re-number distances: backward points go from 0 to backDist
    double totalBack = backDist;

    for( auto& pt : backwardPoints )
        pt.distFromStart = totalBack - pt.distFromStart;

    // Add the start track itself as a point
    double startTrackLen = itemLength( aStartTrack );

    // Walk forward from end endpoint
    std::vector<PATH_POINT>      forwardPoints;
    double                       fwdDist = 0.0;
    PATH_TERMINUS                fwdTerminus = PATH_TERMINUS::NONE;
    std::optional<PATH_JUNCTION> fwdJunction;

    BOARD_CONNECTED_ITEM* fwdVia = nullptr;
    BOARD_CONNECTED_ITEM* fwdNext = nullptr;
    int                   fwdDegree = 0;

    findNeighborsAt( aStartTrack, endPos, visited, fwdVia, fwdNext, fwdDegree );

    BOARD_CONNECTED_ITEM* fwdFirst = fwdVia ? fwdVia : fwdNext;

    if( fwdFirst )
    {
        fwdTerminus = walkDirection( endPos, fwdFirst, visited,
                                    forwardPoints, fwdDist, fwdJunction );
    }

    // Check if end endpoint itself is a pad (forward walk didn't happen)
    if( fwdTerminus == PATH_TERMINUS::NONE && m_padMap.count( endPos ) )
        fwdTerminus = PATH_TERMINUS::PAD;
    else if( fwdTerminus == PATH_TERMINUS::NONE )
        fwdTerminus = PATH_TERMINUS::DEAD_END;

    // Assemble the full path: backward + start track + forward
    m_path = std::move( backwardPoints );

    bool isVia = ( aStartTrack->Type() == PCB_VIA_T );

    if( aStartTrack->Type() == PCB_ARC_T )
    {
        // Arc start track: interpolate along arc geometry
        double arcDist = totalBack;
        emitArcPoints( static_cast<PCB_ARC*>( aStartTrack ), startPos, m_path, arcDist );
    }
    else if( isVia )
    {
        PATH_POINT viaPt;
        viaPt.position = startPos;
        viaPt.tangent = VECTOR2D( 0, 0 );
        viaPt.layer = aStartTrack->GetLayer();
        viaPt.distFromStart = totalBack;
        viaPt.item = aStartTrack;
        viaPt.isVia = true;
        m_path.push_back( viaPt );
    }
    else
    {
        // Straight segment start track
        VECTOR2D tangent;

        if( startPos != endPos )
        {
            VECTOR2D dir( endPos.x - startPos.x, endPos.y - startPos.y );
            double len = dir.EuclideanNorm();

            if( len > 0 )
                tangent = dir / len;
        }

        PATH_POINT startPt;
        startPt.position = startPos;
        startPt.tangent = tangent;
        startPt.layer = aStartTrack->GetLayer();
        startPt.distFromStart = totalBack;
        startPt.item = aStartTrack;
        startPt.isVia = false;
        m_path.push_back( startPt );

        if( startPos != endPos )
        {
            PATH_POINT endPt;
            endPt.position = endPos;
            endPt.tangent = tangent;
            endPt.layer = aStartTrack->GetLayer();
            endPt.distFromStart = totalBack + startTrackLen;
            endPt.item = aStartTrack;
            endPt.isVia = false;
            m_path.push_back( endPt );
        }
    }

    double baseForward = totalBack + startTrackLen;

    // Offset forward points
    for( auto& pt : forwardPoints )
        pt.distFromStart += baseForward;

    m_path.insert( m_path.end(), forwardPoints.begin(), forwardPoints.end() );

    // Identify pads at terminals
    if( !m_path.empty() )
    {
        m_startPad = findPadAt( m_path.front().position, netCode );
        m_endPad = findPadAt( m_path.back().position, netCode );
    }

    // Count visited non-via segments
    std::set<BOARD_CONNECTED_ITEM*> visitedItems;

    for( const PATH_POINT& pt : m_path )
    {
        if( !pt.isVia )
            visitedItems.insert( pt.item );
    }

    m_result.segmentsVisited = static_cast<int>( visitedItems.size() );
    m_result.startTerminus = backTerminus;
    m_result.endTerminus = fwdTerminus;
    m_result.startJunction = std::move( backJunction );
    m_result.endJunction = std::move( fwdJunction );

    return !m_path.empty();
}


bool TRACE_PATH_WALKER::WalkNet( int aNetCode, const VECTOR2I& aFrom )
{
    if( aNetCode <= 0 || !m_board )
        return false;

    // Find a track on this net — prefer one closest to aFrom
    PCB_TRACK* bestTrack = nullptr;
    double     bestDist = std::numeric_limits<double>::max();

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() != aNetCode )
            continue;

        if( track->Type() == PCB_VIA_T )
            continue; // Prefer starting from a track, not a via

        VECTOR2I mid( ( track->GetStart().x + track->GetEnd().x ) / 2,
                      ( track->GetStart().y + track->GetEnd().y ) / 2 );

        double dist = VECTOR2D( mid - aFrom ).EuclideanNorm();

        if( dist < bestDist )
        {
            bestDist = dist;
            bestTrack = track;
        }
    }

    if( !bestTrack )
    {
        // Fall back to any track/via on the net
        for( PCB_TRACK* track : m_board->Tracks() )
        {
            if( track->GetNetCode() == aNetCode )
            {
                bestTrack = track;
                break;
            }
        }
    }

    if( !bestTrack )
        return false;

    if( !Walk( bestTrack ) )
        return false;

    // Normalize direction: ensure path start is the terminal closest to aFrom.
    if( m_path.size() >= 2 )
    {
        double startDist = VECTOR2D( m_path.front().position - aFrom ).EuclideanNorm();
        double endDist = VECTOR2D( m_path.back().position - aFrom ).EuclideanNorm();

        if( endDist < startDist )
            reversePath();
    }

    return true;
}


void TRACE_PATH_WALKER::reversePath()
{
    if( m_path.empty() )
        return;

    double totalLen = m_path.back().distFromStart;

    std::reverse( m_path.begin(), m_path.end() );

    for( auto& pt : m_path )
    {
        pt.distFromStart = totalLen - pt.distFromStart;
        pt.tangent = -pt.tangent;
    }

    std::swap( m_startPad, m_endPad );
    std::swap( m_result.startTerminus, m_result.endTerminus );
    std::swap( m_result.startJunction, m_result.endJunction );
}


double TRACE_PATH_WALKER::GetTotalLength() const
{
    if( m_path.empty() )
        return 0.0;

    return m_path.back().distFromStart;
}


void TRACE_PATH_WALKER::buildPadMap( int aNetCode )
{
    m_padMap.clear();

    for( PAD* pad : m_board->GetPads() )
    {
        if( pad->GetNetCode() == aNetCode )
            m_padMap[pad->GetPosition()] = pad;
    }
}


void TRACE_PATH_WALKER::findNeighborsAt( BOARD_CONNECTED_ITEM* aItem, const VECTOR2I& aPos,
                                          const std::set<BOARD_CONNECTED_ITEM*>& aVisited,
                                          BOARD_CONNECTED_ITEM*& aNextVia,
                                          BOARD_CONNECTED_ITEM*& aNext, int& aUnvisitedDegree,
                                          std::vector<BOARD_CONNECTED_ITEM*>* aBranches ) const
{
    aNextVia = nullptr;
    aNext = nullptr;
    aUnvisitedDegree = 0;

    auto connectivity = m_board->GetConnectivity();

    if( !connectivity )
        return;

    int netCode = aItem->GetNetCode();

    for( PCB_TRACK* other : connectivity->GetConnectedTracks(
                 static_cast<PCB_TRACK*>( aItem ) ) )
    {
        if( other->GetNetCode() != netCode )
            continue;

        if( aVisited.count( other ) )
            continue;

        // Check whether 'other' has an endpoint at aPos (exact match).
        // GetConnectedTracks already told us it's physically connected
        // to aItem; we just need to know which endpoint is at aPos
        // so we can determine direction.
        bool atPos = false;

        if( other->Type() == PCB_VIA_T )
            atPos = ( other->GetStart() == aPos );
        else
            atPos = ( other->GetStart() == aPos || other->GetEnd() == aPos );

        if( !atPos )
            continue;

        if( other->Type() == PCB_VIA_T )
        {
            aNextVia = other;
        }
        else
        {
            aUnvisitedDegree++;
            aNext = other;

            if( aBranches )
                aBranches->push_back( other );
        }
    }
}


PATH_TERMINUS TRACE_PATH_WALKER::walkDirection( const VECTOR2I& aStartPos,
                                                BOARD_CONNECTED_ITEM* aFirstItem,
                                                std::set<BOARD_CONNECTED_ITEM*>& aVisited,
                                                std::vector<PATH_POINT>& aPoints,
                                                double& aCumulDist,
                                                std::optional<PATH_JUNCTION>& aJunction )
{
    BOARD_CONNECTED_ITEM* current = aFirstItem;
    VECTOR2I              currentPos = aStartPos;
    PCB_LAYER_ID          entryLayer = UNDEFINED_LAYER; // layer before a via

    while( current )
    {
        aVisited.insert( current );

        bool isVia = ( current->Type() == PCB_VIA_T );

        if( isVia )
        {
            PCB_VIA* via = static_cast<PCB_VIA*>( current );

            // Via is a point — zero length, but marks a layer transition
            PATH_POINT pt;
            pt.position = via->GetPosition();
            pt.tangent = VECTOR2D( 0, 0 ); // No direction at a via
            pt.layer = via->TopLayer();     // Report the top layer of the via
            pt.distFromStart = aCumulDist;
            pt.item = current;
            pt.isVia = true;
            aPoints.push_back( pt );

            currentPos = via->GetPosition();
            // entryLayer was set by the previous track; kept for post-via filtering
        }
        else if( current->Type() == PCB_ARC_T )
        {
            // Arc segment: interpolate along actual arc geometry
            emitArcPoints( static_cast<PCB_ARC*>( current ), currentPos, aPoints, aCumulDist );
            currentPos = otherEnd( current, currentPos );
            entryLayer = static_cast<PCB_ARC*>( current )->GetLayer();
        }
        else
        {
            // Straight track segment
            PCB_TRACK* track = static_cast<PCB_TRACK*>( current );

            VECTOR2I otherPos = otherEnd( current, currentPos );
            double   len = itemLength( current );

            // Compute tangent direction (from currentPos toward otherPos)
            VECTOR2D tangent( 0, 0 );

            if( currentPos != otherPos )
            {
                VECTOR2D dir( otherPos.x - currentPos.x, otherPos.y - currentPos.y );
                double dirLen = dir.EuclideanNorm();

                if( dirLen > 0 )
                    tangent = dir / dirLen;
            }

            // Emit the entry point of this segment
            PATH_POINT pt;
            pt.position = currentPos;
            pt.tangent = tangent;
            pt.layer = track->GetLayer();
            pt.distFromStart = aCumulDist;
            pt.item = current;
            pt.isVia = false;
            aPoints.push_back( pt );

            aCumulDist += len;

            // Emit the exit point
            PATH_POINT pt2;
            pt2.position = otherPos;
            pt2.tangent = tangent;
            pt2.layer = track->GetLayer();
            pt2.distFromStart = aCumulDist;
            pt2.item = current;
            pt2.isVia = false;
            aPoints.push_back( pt2 );

            currentPos = otherPos;
            entryLayer = track->GetLayer();
        }

        // Find the next unvisited neighbor at currentPos using connectivity.
        // (Pad check moved below — only stop if it's a dead end.)
        BOARD_CONNECTED_ITEM* next = nullptr;
        BOARD_CONNECTED_ITEM* nextVia = nullptr;
        int                   unvisitedDegree = 0;

        std::vector<BOARD_CONNECTED_ITEM*> branches;
        findNeighborsAt( current, currentPos, aVisited, nextVia, next, unvisitedDegree,
                         &branches );

        // If there's an unvisited via at this position, always take it next.
        if( nextVia )
        {
            current = nextVia;
            continue;
        }

        // Filter out track fragments that are entirely inside the copper
        // of a via or pad at currentPos.  These are routing artifacts
        // (e.g. short segments inside a via annular ring or BGA pad)
        // that don't represent real path continuations.
        if( unvisitedDegree > 1 )
        {
            std::shared_ptr<SHAPE> parentShape;

            if( isVia )
            {
                PCB_VIA* via = static_cast<PCB_VIA*>( current );
                parentShape = via->GetEffectiveShape( via->TopLayer() );
            }
            else
            {
                auto itPad = m_padMap.find( currentPos );

                if( itPad != m_padMap.end() )
                    parentShape = itPad->second->GetEffectiveShape( itPad->second->GetLayer() );
            }

            if( parentShape )
            {
                std::vector<BOARD_CONNECTED_ITEM*> filtered;
                next = nullptr;
                unvisitedDegree = 0;

                for( BOARD_CONNECTED_ITEM* b : branches )
                {
                    // Convert the branch track to a polygon and check whether
                    // every vertex lies inside the parent via/pad shape.
                    PCB_LAYER_ID bLayer = b->Type() != PCB_VIA_T
                                             ? static_cast<PCB_TRACK*>( b )->GetLayer()
                                             : UNDEFINED_LAYER;
                    SHAPE_POLY_SET poly;
                    static_cast<PCB_TRACK*>( b )
                            ->GetEffectiveShape( bLayer )
                            ->TransformToPolygon( poly, ARC_LOW_DEF, ERROR_INSIDE );

                    bool inside = poly.OutlineCount() > 0;

                    for( int oi = 0; inside && oi < poly.OutlineCount(); oi++ )
                    {
                        for( int vi = 0; vi < poly.Outline( oi ).PointCount(); vi++ )
                        {
                            if( !parentShape->PointInside( poly.Outline( oi ).CPoint( vi ) ) )
                            {
                                inside = false;
                                break;
                            }
                        }
                    }

                    if( inside )
                        continue;   // entire track shape inside via/pad copper

                    // After a via, also skip tracks on the entry layer
                    if( isVia && entryLayer != UNDEFINED_LAYER
                        && b->Type() != PCB_VIA_T
                        && static_cast<PCB_TRACK*>( b )->GetLayer() == entryLayer )
                        continue;

                    unvisitedDegree++;
                    next = b;
                    filtered.push_back( b );
                }

                branches = std::move( filtered );
            }

            if( isVia )
                entryLayer = UNDEFINED_LAYER;
        }

        // Stop at junctions (more than 1 real branch after filtering)
        if( unvisitedDegree > 1 )
        {
            PATH_JUNCTION jct;
            jct.position = currentPos;
            jct.branches = std::move( branches );

            aJunction = std::move( jct );
            return PATH_TERMINUS::JUNCTION;
        }

        if( !next )
        {
            // If we're at a multi-layer pad (through-hole), it bridges layers
            // like a via.  Look for tracks connected to the pad on other layers.
            auto itPad = m_padMap.find( currentPos );

            if( itPad != m_padMap.end() )
            {
                PAD* pad = itPad->second;

                auto connectivity = m_board->GetConnectivity();

                if( pad->GetLayerSet().CuStack().size() > 1 && connectivity )
                {
                    const std::vector<KICAD_T> trackTypes = {
                        PCB_TRACE_T, PCB_ARC_T, PCB_VIA_T
                    };

                    for( BOARD_CONNECTED_ITEM* item :
                         connectivity->GetConnectedItemsAtAnchor( pad, currentPos,
                                                                  trackTypes ) )
                    {
                        if( aVisited.count( item ) )
                            continue;

                        if( item->Type() == PCB_VIA_T )
                        {
                            nextVia = item;
                        }
                        else
                        {
                            PCB_TRACK* t = static_cast<PCB_TRACK*>( item );

                            if( t->GetStart() != currentPos && t->GetEnd() != currentPos )
                                continue;

                            // Skip tracks on the entry layer — they're behind us
                            if( entryLayer != UNDEFINED_LAYER
                                && t->GetLayer() == entryLayer )
                                continue;

                            unvisitedDegree++;
                            next = item;
                        }
                    }

                    if( nextVia )
                    {
                        // Emit the PTH pad as a via-like path point before
                        // continuing through the actual via.
                        PATH_POINT pt;
                        pt.position = currentPos;
                        pt.tangent = VECTOR2D( 0, 0 );
                        pt.layer = entryLayer != UNDEFINED_LAYER ? entryLayer
                                                                 : pad->GetLayer();
                        pt.distFromStart = aCumulDist;
                        pt.item = pad;
                        pt.isVia = true;
                        aPoints.push_back( pt );

                        current = nextVia;
                        continue;
                    }

                    if( unvisitedDegree == 1 )
                    {
                        // Emit the PTH pad as a via-like layer transition
                        PATH_POINT pt;
                        pt.position = currentPos;
                        pt.tangent = VECTOR2D( 0, 0 );
                        pt.layer = entryLayer != UNDEFINED_LAYER ? entryLayer
                                                                 : pad->GetLayer();
                        pt.distFromStart = aCumulDist;
                        pt.item = pad;
                        pt.isVia = true;
                        aPoints.push_back( pt );

                        current = next;
                        continue;
                    }
                }

                return PATH_TERMINUS::PAD;
            }

            return PATH_TERMINUS::DEAD_END;
        }

        current = next;
    }

    return PATH_TERMINUS::DEAD_END;
}


VECTOR2I TRACE_PATH_WALKER::otherEnd( BOARD_CONNECTED_ITEM* aItem, const VECTOR2I& aFrom ) const
{
    if( aItem->Type() == PCB_VIA_T )
        return aFrom; // Via is a single point

    PCB_TRACK* track = static_cast<PCB_TRACK*>( aItem );

    if( track->GetStart() == aFrom )
        return track->GetEnd();
    else
        return track->GetStart();
}


double TRACE_PATH_WALKER::itemLength( BOARD_CONNECTED_ITEM* aItem ) const
{
    if( aItem->Type() == PCB_VIA_T )
        return 0.0;

    PCB_TRACK* track = static_cast<PCB_TRACK*>( aItem );
    return track->GetLength();
}


void TRACE_PATH_WALKER::emitArcPoints( PCB_ARC* aArc, const VECTOR2I& aEntryPos,
                                       std::vector<PATH_POINT>& aPoints,
                                       double& aCumulDist )
{
    VECTOR2D center( aArc->GetPosition() );
    double   radius = aArc->GetRadius();
    double   totalAngle = aArc->GetAngle().AsRadians(); // signed: positive = CCW

    // Determine walk direction: entering at Start walks the arc forward
    bool enterAtStart = ( aEntryPos == aArc->GetStart() );

    VECTOR2D entryVec = VECTOR2D( aEntryPos ) - center;
    double   entryAngle = std::atan2( entryVec.y, entryVec.x );

    // Sweep: start→end uses totalAngle, end→start uses -totalAngle
    double sweepAngle = enterAtStart ? totalAngle : -totalAngle;

    // Arc-length step ≈ one trace width (floor 0.25mm to avoid excessive subdivision)
    double arcLengthStep = std::max( static_cast<double>( aArc->GetWidth() ), 250000.0 );
    double stepAngleRad = arcLengthStep / radius;
    int    nSteps = std::max( 1, static_cast<int>(
                                         std::ceil( std::abs( sweepAngle ) / stepAngleRad ) ) );
    nSteps = std::min( nSteps, 72 );

    double arcLen = radius * std::abs( sweepAngle );
    double stepAngle = sweepAngle / nSteps;
    double stepLen = arcLen / nSteps;

    VECTOR2I exitPos = enterAtStart ? aArc->GetEnd() : aArc->GetStart();

    for( int i = 0; i <= nSteps; i++ )
    {
        double theta = entryAngle + i * stepAngle;

        VECTOR2I pos;

        if( i == 0 )
            pos = aEntryPos;
        else if( i == nSteps )
            pos = exitPos;
        else
            pos = VECTOR2I( KiROUND( center.x + radius * std::cos( theta ) ),
                            KiROUND( center.y + radius * std::sin( theta ) ) );

        // Tangent: perpendicular to radius, oriented in sweep direction
        double   sign = ( sweepAngle > 0 ) ? 1.0 : -1.0;
        VECTOR2D tangent( sign * -std::sin( theta ), sign * std::cos( theta ) );

        PATH_POINT pt;
        pt.position = pos;
        pt.tangent = tangent;
        pt.layer = aArc->GetLayer();
        pt.distFromStart = aCumulDist + i * stepLen;
        pt.item = aArc;
        pt.isVia = false;
        aPoints.push_back( pt );
    }

    aCumulDist += arcLen;
}


PAD* TRACE_PATH_WALKER::findPadAt( const VECTOR2I& aPos, int aNetCode ) const
{
    auto it = m_padMap.find( aPos );

    if( it != m_padMap.end() )
        return it->second;

    return nullptr;
}
