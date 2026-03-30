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

#include "cross_section_builder.h"

#include <board.h>
#include <board_connected_item.h>
#include <drc/drc_rtree.h>
#include <pcb_track.h>
#include <zone.h>

#include <geometry/shape.h>
#include <geometry/shape_poly_set.h>

#include <algorithm>
#include <cmath>


static constexpr int MIN_EDGE_TO_EDGE = 10000; // 10µm in nm


CROSS_SECTION_BUILDER::CROSS_SECTION_BUILDER() :
        m_rtree( nullptr ),
        m_board( nullptr ),
        m_signalTrack( nullptr ),
        m_pathItemDist( nullptr )
{
}


void CROSS_SECTION_BUILDER::SetSpatialIndex( DRC_RTREE* aRtree )
{
    m_rtree = aRtree;
}


void CROSS_SECTION_BUILDER::SetBoard( const BOARD* aBoard )
{
    m_board = aBoard;
}


void CROSS_SECTION_BUILDER::SetPathItemDistances(
        const std::map<BOARD_CONNECTED_ITEM*, double>* aMap )
{
    m_pathItemDist = aMap;
}


void CROSS_SECTION_BUILDER::SetSignalTrack( const PCB_TRACK* aTrack )
{
    m_signalTrack = aTrack;
}


std::vector<XS_NEIGHBOR> CROSS_SECTION_BUILDER::FindNeighbors(
        const XS_BUILD_PARAMS& aParams ) const
{
    std::vector<XS_NEIGHBOR> neighbors;

    VECTOR2D normal( -aParams.sampleTangent.y, aParams.sampleTangent.x );

    VECTOR2I cutA( aParams.samplePos.x - (int) ( normal.x * aParams.couplingHorizon ),
                   aParams.samplePos.y - (int) ( normal.y * aParams.couplingHorizon ) );
    VECTOR2I cutB( aParams.samplePos.x + (int) ( normal.x * aParams.couplingHorizon ),
                   aParams.samplePos.y + (int) ( normal.y * aParams.couplingHorizon ) );
    SEG cutSeg( cutA, cutB );

    findTrackNeighbors( aParams, cutSeg, normal, neighbors );
    // findShapeNeighbors uses the same R-tree query as findTrackNeighbors
    // but handles non-track items (pads, shapes). They share the query
    // since findTrackNeighbors skips non-track items and vice versa.
    findShapeNeighbors( aParams, cutSeg, normal, neighbors );

    if( m_board )
        findZoneNeighbors( aParams, cutSeg, normal, neighbors );

    deduplicateNeighbors( neighbors );

    return neighbors;
}


void CROSS_SECTION_BUILDER::findTrackNeighbors( const XS_BUILD_PARAMS& aParams,
                                                 const SEG& aCutSeg,
                                                 const VECTOR2D& aNormal,
                                                 std::vector<XS_NEIGHBOR>& aNeighbors ) const
{
    if( !m_rtree )
        return;

    auto nearbyItems = m_rtree->GetObjectsAt( aParams.samplePos, aParams.signalLayer,
                                               aParams.couplingHorizon );

    for( BOARD_ITEM* item : nearbyItems )
    {
        if( item->Type() == PCB_VIA_T || item->Type() == PCB_ZONE_T )
            continue;

        if( item->Type() != PCB_TRACE_T && item->Type() != PCB_ARC_T )
            continue;

        PCB_TRACK* other = static_cast<PCB_TRACK*>( item );

        if( other == m_signalTrack )
            continue;

        // Same-net path-distance check: exclude topological neighbors (bends)
        // but keep coupling neighbors (serpentine return legs).
        if( m_pathItemDist )
        {
            auto pathIt = m_pathItemDist->find( other );

            if( pathIt != m_pathItemDist->end() )
            {
                double pathSep = std::abs( pathIt->second - aParams.sampleDist );

                if( pathSep < aParams.couplingHorizon * 2.0 )
                    continue;
            }
        }

        // Same-net non-track items (pads on signal net) — skip
        if( other->GetNetCode() == aParams.signalNetCode )
        {
            // Only allow same-net tracks that passed the path-distance check above
            // (i.e., they're far on the path — serpentine return legs).
            // If they're not in pathItemDist at all (not on the walked path),
            // they could be branches — exclude them for safety.
            if( !m_pathItemDist || m_pathItemDist->find( other ) == m_pathItemDist->end() )
                continue;
        }

        // Use the effective shape (stadium = centerline + half-width endcaps)
        // to test intersection with the cross-section cut line.
        std::shared_ptr<SHAPE> shape = other->GetEffectiveShape( aParams.signalLayer );

        if( !shape )
            continue;

        VECTOR2I nearest;

        if( !shape->Collide( aCutSeg, 0, nullptr, &nearest ) )
            continue;

        // nearest is the closest point on the neighbor's centerline to the cut seg.
        // Project onto the normal to get signed lateral (center-to-center) distance.
        VECTOR2D delta( nearest.x - aParams.samplePos.x,
                        nearest.y - aParams.samplePos.y );
        double lateralDist = delta.x * aNormal.x + delta.y * aNormal.y;
        double edgeToEdge = std::abs( lateralDist )
                            - aParams.signalWidth / 2.0
                            - other->GetWidth() / 2.0;

        if( edgeToEdge < MIN_EDGE_TO_EDGE || edgeToEdge > aParams.couplingHorizon )
            continue;

        aNeighbors.push_back( { static_cast<int>( lateralDist ), other->GetWidth() } );
    }
}


void CROSS_SECTION_BUILDER::findShapeNeighbors( const XS_BUILD_PARAMS& aParams,
                                                 const SEG& aCutSeg,
                                                 const VECTOR2D& aNormal,
                                                 std::vector<XS_NEIGHBOR>& aNeighbors ) const
{
    if( !m_rtree )
        return;

    auto nearbyItems = m_rtree->GetObjectsAt( aParams.samplePos, aParams.signalLayer,
                                               aParams.couplingHorizon );

    for( BOARD_ITEM* item : nearbyItems )
    {
        if( item->Type() == PCB_VIA_T || item->Type() == PCB_ZONE_T )
            continue;

        if( item->Type() == PCB_TRACE_T || item->Type() == PCB_ARC_T )
            continue; // handled by findTrackNeighbors

        // Skip same-net items (signal pads, etc.)
        auto* connItem = dynamic_cast<BOARD_CONNECTED_ITEM*>( item );

        if( connItem && connItem->GetNetCode() == aParams.signalNetCode )
            continue;

        std::shared_ptr<SHAPE> shape = item->GetEffectiveShape( aParams.signalLayer );

        if( !shape )
            continue;

        VECTOR2I nearestOnCut;
        int      actualDist = 0;

        if( shape->Collide( aCutSeg, aParams.couplingHorizon, &actualDist, &nearestOnCut ) )
        {
            VECTOR2D delta( nearestOnCut.x - aParams.samplePos.x,
                            nearestOnCut.y - aParams.samplePos.y );
            double lateralDist = delta.x * aNormal.x + delta.y * aNormal.y;
            double edgeToEdge = std::abs( lateralDist ) - aParams.signalWidth / 2.0;

            if( edgeToEdge < aParams.couplingHorizon && edgeToEdge > MIN_EDGE_TO_EDGE )
            {
                int itemWidth = 200000; // approximate 200µm
                aNeighbors.push_back( { static_cast<int>( lateralDist ), itemWidth } );
            }
        }
    }
}


void CROSS_SECTION_BUILDER::findZoneNeighbors( const XS_BUILD_PARAMS& aParams,
                                                const SEG& aCutSeg,
                                                const VECTOR2D& aNormal,
                                                std::vector<XS_NEIGHBOR>& aNeighbors ) const
{
    double halfTraceW = aParams.signalWidth / 2.0;
    int    zoneWidth = (int) ( std::max( aParams.layerGeom.hAbove,
                                          aParams.layerGeom.hBelow ) * 3.0 / 1e-9 );

    if( zoneWidth < 100000 )
        zoneWidth = 100000; // min 100µm

    auto intersectChain = [&]( const SHAPE_LINE_CHAIN& aChain )
    {
        for( int ei = 0; ei < aChain.SegmentCount(); ei++ )
        {
            SEG edge = aChain.Segment( ei );
            OPT_VECTOR2I intersection = aCutSeg.IntersectLines( edge );

            if( !intersection )
                continue;

            if( !edge.Contains( *intersection ) )
                continue;

            VECTOR2D delta( intersection->x - aParams.samplePos.x,
                            intersection->y - aParams.samplePos.y );
            double lateralDist = delta.x * aNormal.x + delta.y * aNormal.y;
            double edgeToEdge = std::abs( lateralDist ) - halfTraceW;

            if( edgeToEdge > aParams.couplingHorizon || edgeToEdge < MIN_EDGE_TO_EDGE )
                continue;

            int zoneCenterDist;

            if( lateralDist > 0 )
                zoneCenterDist = (int) lateralDist + zoneWidth / 2;
            else
                zoneCenterDist = (int) lateralDist - zoneWidth / 2;

            aNeighbors.push_back( { zoneCenterDist, zoneWidth } );
        }
    };

    // Use the R-tree to find zones near the sample point, then intersect
    // the cut line with their fill polygon edges.
    if( !m_rtree )
        return;

    auto nearbyZones = m_rtree->GetObjectsAt( aParams.samplePos, aParams.signalLayer,
                                               aParams.couplingHorizon );

    // Collect unique zones (R-tree may return multiple entries per zone)
    std::set<ZONE*> processedZones;

    for( BOARD_ITEM* item : nearbyZones )
    {
        if( item->Type() != PCB_ZONE_T )
            continue;

        ZONE* zone = static_cast<ZONE*>( item );

        if( zone->GetIsRuleArea() )
            continue;

        if( !processedZones.insert( zone ).second )
            continue; // already processed

        const std::shared_ptr<SHAPE_POLY_SET>& fill =
                zone->GetFilledPolysList( aParams.signalLayer );

        if( !fill || fill->IsEmpty() )
            continue;

        for( int oi = 0; oi < fill->OutlineCount(); oi++ )
        {
            intersectChain( fill->Outline( oi ) );

            for( int hi = 0; hi < fill->HoleCount( oi ); hi++ )
                intersectChain( fill->Hole( oi, hi ) );
        }
    }

    // Also check zones directly from the board if the R-tree doesn't
    // contain zone fills (e.g., our temp R-tree only has tracks).
    if( processedZones.empty() && m_board )
    {
        for( ZONE* zone : m_board->Zones() )
        {
            if( !zone->IsOnLayer( aParams.signalLayer ) )
                continue;

            if( zone->GetIsRuleArea() )
                continue;

            const std::shared_ptr<SHAPE_POLY_SET>& fill =
                    zone->GetFilledPolysList( aParams.signalLayer );

            if( !fill || fill->IsEmpty() )
                continue;

            // No fast rejection in fallback — we only reach here when the R-tree
            // doesn't have zone data (temp R-tree case), so zone count is the only cost.

            for( int oi = 0; oi < fill->OutlineCount(); oi++ )
            {
                intersectChain( fill->Outline( oi ) );

                for( int hi = 0; hi < fill->HoleCount( oi ); hi++ )
                    intersectChain( fill->Hole( oi, hi ) );
            }
        }
    }
}


void CROSS_SECTION_BUILDER::deduplicateNeighbors( std::vector<XS_NEIGHBOR>& aNeighbors,
                                                   int aMaxNeighbors )
{
    std::sort( aNeighbors.begin(), aNeighbors.end(),
               []( const XS_NEIGHBOR& a, const XS_NEIGHBOR& b )
               {
                   return std::abs( a.distNm ) < std::abs( b.distNm );
               } );

    // Merge entries that are on the same side and within one trace width of each other.
    // Don't merge opposite-side neighbors even if they're at similar absolute distances.
    for( size_t i = 0; i + 1 < aNeighbors.size(); )
    {
        bool sameSign = ( aNeighbors[i].distNm > 0 ) == ( aNeighbors[i + 1].distNm > 0 );
        int  gap = std::abs( aNeighbors[i + 1].distNm ) - std::abs( aNeighbors[i].distNm );

        if( sameSign && gap < std::max( aNeighbors[i].widthNm, aNeighbors[i + 1].widthNm ) )
            aNeighbors.erase( aNeighbors.begin() + (int) i + 1 );
        else
            i++;
    }

    if( (int) aNeighbors.size() > aMaxNeighbors )
        aNeighbors.resize( aMaxNeighbors );
}


XS_GEOMETRY CROSS_SECTION_BUILDER::BuildGeometry(
        const XS_BUILD_PARAMS& aParams,
        const std::vector<XS_NEIGHBOR>& aNeighbors ) const
{
    XS_GEOMETRY xs;
    double condY = 0.0;

    const LAYER_GEOMETRY& geom = aParams.layerGeom;
    bool hasAbove = ( geom.hAbove > 0.0 );
    bool hasBelow = ( geom.hBelow > 0.0 );

    if( hasAbove && hasBelow )
    {
        xs.groundY = 0.0;
        xs.hasUpperGround = true;
        xs.upperGroundY = -( geom.hAbove + geom.hBelow );
        condY = -geom.hBelow;
        xs.epsilonR = ( geom.erAbove + geom.erBelow ) / 2.0;

        if( std::abs( geom.erAbove - geom.erBelow ) > 0.5 )
        {
            XS_DIELECTRIC_REGION regA, regB;
            regA.yTop = xs.upperGroundY;
            regA.yBottom = condY;
            regA.epsilonR = geom.erAbove;
            regB.yTop = condY;
            regB.yBottom = 0.0;
            regB.epsilonR = geom.erBelow;
            xs.dielectrics.push_back( regA );
            xs.dielectrics.push_back( regB );
        }
    }
    else if( hasBelow )
    {
        xs.groundY = 0.0;
        condY = -( geom.hBelow + geom.traceThickness / 2.0 );
        xs.epsilonR = geom.erBelow;

        XS_DIELECTRIC_REGION air, diel;
        air.yTop = -10e-3;
        air.yBottom = -geom.hBelow;
        air.epsilonR = 1.0;
        diel.yTop = -geom.hBelow;
        diel.yBottom = 0.0;
        diel.epsilonR = geom.erBelow;
        xs.dielectrics.push_back( air );
        xs.dielectrics.push_back( diel );
    }
    else if( hasAbove )
    {
        xs.groundY = -( geom.hAbove + geom.traceThickness );
        condY = -geom.traceThickness / 2.0;
        xs.epsilonR = geom.erAbove;

        XS_DIELECTRIC_REGION diel, air;
        diel.yTop = xs.groundY;
        diel.yBottom = -geom.traceThickness;
        diel.epsilonR = geom.erAbove;
        air.yTop = -geom.traceThickness;
        air.yBottom = 10e-3;
        air.epsilonR = 1.0;
        xs.dielectrics.push_back( diel );
        xs.dielectrics.push_back( air );
    }

    // Signal conductor at x=0
    XS_CONDUCTOR cond;
    cond.centerX = 0.0;
    cond.centerY = condY;
    cond.width = geom.traceWidth;
    cond.thickness = geom.traceThickness;
    xs.conductors.push_back( cond );

    // Neighbor conductors
    for( const XS_NEIGHBOR& nb : aNeighbors )
    {
        XS_CONDUCTOR nbCond;
        nbCond.centerX = nb.distNm * 1e-9;
        nbCond.centerY = condY;
        nbCond.width = nb.widthNm * 1e-9;
        nbCond.thickness = geom.traceThickness;
        xs.conductors.push_back( nbCond );
    }

    return xs;
}
