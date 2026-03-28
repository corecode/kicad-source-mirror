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

#include <algorithm>


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

    buildAdjacency( netCode );

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
    std::vector<PATH_POINT>    backwardPoints;
    double                     backDist = 0.0;
    PATH_TERMINUS              backTerminus = PATH_TERMINUS::NONE;
    std::optional<PATH_JUNCTION> backJunction;

    auto itStart = m_adjacency.find( startPos );

    if( itStart != m_adjacency.end() )
    {
        for( BOARD_CONNECTED_ITEM* neighbor : itStart->second )
        {
            if( visited.count( neighbor ) == 0 )
            {
                backTerminus = walkDirection( startPos, neighbor, visited,
                                             backwardPoints, backDist, backJunction );
                break;
            }
        }
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

    auto itEnd = m_adjacency.find( endPos );

    if( itEnd != m_adjacency.end() )
    {
        for( BOARD_CONNECTED_ITEM* neighbor : itEnd->second )
        {
            if( visited.count( neighbor ) == 0 )
            {
                fwdTerminus = walkDirection( endPos, neighbor, visited,
                                            forwardPoints, fwdDist, fwdJunction );
                break;
            }
        }
    }

    // Check if end endpoint itself is a pad (forward walk didn't happen)
    if( fwdTerminus == PATH_TERMINUS::NONE && m_padMap.count( endPos ) )
        fwdTerminus = PATH_TERMINUS::PAD;
    else if( fwdTerminus == PATH_TERMINUS::NONE )
        fwdTerminus = PATH_TERMINUS::DEAD_END;

    // Assemble the full path: backward + start track + forward
    m_path = std::move( backwardPoints );

    // Add start track's start point
    VECTOR2D tangent;
    bool isVia = ( aStartTrack->Type() == PCB_VIA_T );

    if( !isVia && startPos != endPos )
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
    startPt.isVia = isVia;
    m_path.push_back( startPt );

    if( !isVia && startPos != endPos )
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

    return Walk( bestTrack );
}


double TRACE_PATH_WALKER::GetTotalLength() const
{
    if( m_path.empty() )
        return 0.0;

    return m_path.back().distFromStart;
}


void TRACE_PATH_WALKER::buildAdjacency( int aNetCode )
{
    m_adjacency.clear();
    m_padMap.clear();

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() != aNetCode )
            continue;

        if( track->Type() == PCB_VIA_T )
        {
            // Vias have a single position; they connect to tracks at that position
            VECTOR2I pos = track->GetStart(); // Via position stored in m_Start
            m_adjacency[pos].push_back( track );
        }
        else
        {
            // Track segments (PCB_TRACK and PCB_ARC) have start and end points
            m_adjacency[track->GetStart()].push_back( track );
            m_adjacency[track->GetEnd()].push_back( track );
        }
    }

    // Build pad map
    for( PAD* pad : m_board->GetPads() )
    {
        if( pad->GetNetCode() == aNetCode )
            m_padMap[pad->GetPosition()] = pad;
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
        }
        else
        {
            // Track or arc segment
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
        }

        // Stop if we hit a pad
        if( m_padMap.count( currentPos ) )
            return PATH_TERMINUS::PAD;

        // Find the next unvisited neighbor at currentPos
        BOARD_CONNECTED_ITEM* next = nullptr;
        int                   unvisitedDegree = 0;

        auto it = m_adjacency.find( currentPos );

        if( it != m_adjacency.end() )
        {
            for( BOARD_CONNECTED_ITEM* neighbor : it->second )
            {
                if( aVisited.count( neighbor ) == 0 )
                {
                    unvisitedDegree++;
                    next = neighbor;
                }
            }
        }

        // Stop at junctions (more than 1 unvisited neighbor = T-junction)
        if( unvisitedDegree > 1 )
        {
            PATH_JUNCTION jct;
            jct.position = currentPos;

            for( BOARD_CONNECTED_ITEM* neighbor : it->second )
            {
                if( aVisited.count( neighbor ) == 0 )
                    jct.branches.push_back( neighbor );
            }

            aJunction = std::move( jct );
            return PATH_TERMINUS::JUNCTION;
        }

        if( !next )
            return PATH_TERMINUS::DEAD_END;

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


PAD* TRACE_PATH_WALKER::findPadAt( const VECTOR2I& aPos, int aNetCode ) const
{
    auto it = m_padMap.find( aPos );

    if( it != m_padMap.end() )
        return it->second;

    return nullptr;
}
