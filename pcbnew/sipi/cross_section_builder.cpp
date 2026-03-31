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
        if( item->Type() == PCB_ZONE_T )
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

        VECTOR2I nearest;

        // Zero clearance: pad must actually intersect the cut line.
        // This prevents detecting pads that are nearby in board space
        // but offset along the trace direction (not at the cross-section).
        if( !shape->Collide( aCutSeg, 0, nullptr, &nearest ) )
            continue;

        // Project nearest point onto the normal for lateral distance.
        VECTOR2D delta( nearest.x - aParams.samplePos.x,
                        nearest.y - aParams.samplePos.y );
        double lateralDist = delta.x * aNormal.x + delta.y * aNormal.y;

        // Estimate pad width along the cut from the bounding box.
        BOX2I bbox = item->GetBoundingBox();
        double minProj = 1e18, maxProj = -1e18;
        VECTOR2I corners[4] = {
            bbox.GetOrigin(),
            bbox.GetOrigin() + VECTOR2I( bbox.GetWidth(), 0 ),
            bbox.GetOrigin() + VECTOR2I( 0, bbox.GetHeight() ),
            bbox.GetEnd()
        };

        for( const VECTOR2I& c : corners )
        {
            VECTOR2D d( c.x - aParams.samplePos.x, c.y - aParams.samplePos.y );
            double proj = d.x * aNormal.x + d.y * aNormal.y;
            minProj = std::min( minProj, proj );
            maxProj = std::max( maxProj, proj );
        }

        int itemWidth = (int) ( maxProj - minProj );

        if( itemWidth < 50000 )
            itemWidth = 50000;

        // Use nearest-point lateral distance, adjusted to center
        double centerDist = lateralDist;

        // The nearest point is on the pad edge. Shift to approximate center
        // by adding half the item width in the sign direction.
        if( lateralDist > 0 )
            centerDist = lateralDist + itemWidth / 2.0;
        else
            centerDist = lateralDist - itemWidth / 2.0;

        double edgeToEdge = std::abs( centerDist )
                            - aParams.signalWidth / 2.0
                            - itemWidth / 2.0;

        if( edgeToEdge < aParams.couplingHorizon && edgeToEdge > MIN_EDGE_TO_EDGE )
        {
            aNeighbors.push_back( { static_cast<int>( centerDist ), itemWidth } );
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


std::vector<XS_GROUNDWIRE> CROSS_SECTION_BUILDER::FindGroundWires(
        const XS_BUILD_PARAMS& aParams,
        LAYER_GEOMETRY& aLayerGeom ) const
{
    std::vector<XS_GROUNDWIRE> groundWires;

    if( !m_board )
        return groundWires;

    // Construct the same cut line as FindNeighbors
    VECTOR2D normal( -aParams.sampleTangent.y, aParams.sampleTangent.x );

    // Search extent: coupling horizon or 5× dielectric height, capped at 5mm
    // to avoid scanning the entire board when virtual earth is active.
    double hRef = std::min( std::max( aLayerGeom.hAbove, aLayerGeom.hBelow ), 1e-3 );
    int    extent = std::clamp( std::max( aParams.couplingHorizon,
                                          (int) ( hRef * 5.0 / 1e-9 ) ),
                                500000, 5000000 );

    VECTOR2I cutA( aParams.samplePos.x - (int) ( normal.x * extent ),
                   aParams.samplePos.y - (int) ( normal.y * extent ) );
    VECTOR2I cutB( aParams.samplePos.x + (int) ( normal.x * extent ),
                   aParams.samplePos.y + (int) ( normal.y * extent ) );
    SEG cutSeg( cutA, cutB );

    double halfSignalW = aParams.signalWidth / 2.0;

    // Helper: sample copper coverage along the cut line on a given layer.
    // Returns copper spans as [left, right] lateral distances (nm) from signal center.
    // Uses HitTestFilledArea at discrete points — robust with overlapping zones
    // and complex polygon topologies (no parity pairing issues).
    static constexpr int MAX_GROUNDWIRES = 4;

    auto findCopperSpans = [&]( PCB_LAYER_ID aLayer ) -> std::vector<std::pair<double, double>>
    {
        std::vector<std::pair<double, double>> spans;

        // Sample interval: 50µm along the cut line
        static constexpr double SAMPLE_STEP = 50000.0; // nm

        bool   inCopper = false;
        double spanStart = 0.0;

        for( double lat = -extent; lat <= extent; lat += SAMPLE_STEP )
        {
            VECTOR2I pt( aParams.samplePos.x + (int) ( normal.x * lat ),
                         aParams.samplePos.y + (int) ( normal.y * lat ) );

            bool covered = false;

            for( ZONE* zone : m_board->Zones() )
            {
                if( !zone->IsOnLayer( aLayer ) || zone->GetIsRuleArea() )
                    continue;

                if( zone->HitTestFilledArea( aLayer, pt ) )
                {
                    covered = true;
                    break;
                }
            }

            if( covered && !inCopper )
            {
                spanStart = lat;
                inCopper = true;
            }
            else if( !covered && inCopper )
            {
                spans.push_back( { spanStart, lat - SAMPLE_STEP } );
                inCopper = false;
            }
        }

        if( inCopper )
            spans.push_back( { spanStart, (double) extent } );

        return spans;
    };

    // Helper: convert copper spans into groundwires.
    auto makeGroundWires = [&]( const std::vector<std::pair<double, double>>& aSpans,
                                double aZPosition, double aThickness,
                                bool aSkipSignalCenter )
    {
        double maxDist = aParams.couplingHorizon * 2.0;

        for( const auto& [left, right] : aSpans )
        {
            double spanWidth = right - left;

            if( spanWidth < 10000 ) // ignore tiny fragments (< 10µm)
                continue;

            double spanCenter = ( left + right ) / 2.0;

            if( aSkipSignalCenter && left < halfSignalW && right > -halfSignalW )
                continue;

            double nearestEdge = std::min( std::abs( left ), std::abs( right ) );

            if( nearestEdge > maxDist )
                continue;

            XS_GROUNDWIRE gw;
            gw.lateralNm = (int) spanCenter;
            gw.widthNm = (int) spanWidth;
            gw.yPositionM = aZPosition;
            gw.thicknessM = aThickness;
            groundWires.push_back( gw );
        }
    };

    // --- Check intermediate layers (antipad directly under signal) ---
    for( const auto& interLayer : aLayerGeom.intermediateLayers )
    {
        auto spans = findCopperSpans( interLayer.layerId );

        if( !spans.empty() )
            makeGroundWires( spans, interLayer.zPosition, interLayer.thickness, true );
    }

    // --- Check reference layers for nearby edges (adjacent antipad) ---
    // If the copper span containing the signal has an edge within the
    // coupling horizon, demote the reference to groundwires.  The image
    // ground shifts to virtual earth, but the original dielectric info
    // is preserved so BuildGeometry creates the correct layering.
    static constexpr double VIRTUAL_EARTH_M = 10e-3;

    auto checkRefLayer = [&]( PCB_LAYER_ID aRefLayer, double aRefZ, double aRefThickness,
                              bool& aHasRef, double& aH, double& aEr, double& aTanD,
                              double& aHOrig, double& aErOrig )
    {
        if( aRefLayer == UNDEFINED_LAYER )
            return;

        // Fast check via R-tree: are there pads or vias on the reference layer
        // near the sample point?  Antipads extend beyond the pad by the zone
        // clearance, so search with extra margin.
        static constexpr int ANTIPAD_MARGIN = 1000000; // 1mm

        bool hasNearbyObstacle = false;

        if( m_rtree )
        {
            auto nearby = m_rtree->GetObjectsAt( aParams.samplePos, aRefLayer,
                                                  aParams.couplingHorizon + ANTIPAD_MARGIN );

            for( BOARD_ITEM* item : nearby )
            {
                if( item->Type() == PCB_VIA_T || item->Type() == PCB_PAD_T )
                {
                    hasNearbyObstacle = true;
                    break;
                }
            }
        }

        if( !hasNearbyObstacle )
            return;

        auto spans = findCopperSpans( aRefLayer );

        if( spans.empty() )
            return;

        // Find the copper span containing the signal center.
        double spanLeft = 0.0;
        double spanRight = 0.0;
        bool   foundSignalSpan = false;

        for( const auto& [left, right] : spans )
        {
            if( left <= halfSignalW && right >= -halfSignalW )
            {
                spanLeft = left;
                spanRight = right;
                foundSignalSpan = true;
                break;
            }
        }

        if( !foundSignalSpan )
            return;

        // Only demote if the signal's span edge is within the coupling horizon.
        double nearestEdge = std::min( std::abs( spanLeft ), std::abs( spanRight ) );

        if( nearestEdge >= aParams.couplingHorizon )
            return;

        // Demote: reference layer copper becomes groundwires
        makeGroundWires( spans, aRefZ, aRefThickness, false );

        // Save original dielectric info before overwriting
        aHOrig = aH;
        aErOrig = aEr;

        // Shift image ground to virtual earth
        aHasRef = true;
        aH = VIRTUAL_EARTH_M;
        aEr = 1.0;
        aTanD = 0.0;
    };

    if( aLayerGeom.hasRefBelow && aLayerGeom.refLayerBelow != UNDEFINED_LAYER )
    {
        double refZ = aLayerGeom.signalZPosition
                      + aLayerGeom.hBelow + aLayerGeom.traceThickness / 2.0;

        checkRefLayer( aLayerGeom.refLayerBelow, refZ, aLayerGeom.traceThickness,
                       aLayerGeom.hasRefBelow, aLayerGeom.hBelow,
                       aLayerGeom.erBelow, aLayerGeom.tanDBelow,
                       aLayerGeom.hOrigBelow, aLayerGeom.erOrigBelow );
    }

    if( aLayerGeom.hasRefAbove && aLayerGeom.refLayerAbove != UNDEFINED_LAYER )
    {
        double refZ = aLayerGeom.signalZPosition
                      - aLayerGeom.hAbove - aLayerGeom.traceThickness / 2.0;

        checkRefLayer( aLayerGeom.refLayerAbove, refZ, aLayerGeom.traceThickness,
                       aLayerGeom.hasRefAbove, aLayerGeom.hAbove,
                       aLayerGeom.erAbove, aLayerGeom.tanDAbove,
                       aLayerGeom.hOrigAbove, aLayerGeom.erOrigAbove );
    }

    // Cap groundwire count — too many conductors makes the BEM matrix huge.
    // Keep the closest ones (sorted by distance from signal center).
    if( (int) groundWires.size() > MAX_GROUNDWIRES )
    {
        std::sort( groundWires.begin(), groundWires.end(),
                   []( const XS_GROUNDWIRE& a, const XS_GROUNDWIRE& b )
                   { return std::abs( a.lateralNm ) < std::abs( b.lateralNm ); } );

        groundWires.resize( MAX_GROUNDWIRES );
    }

    return groundWires;
}


XS_GEOMETRY CROSS_SECTION_BUILDER::BuildGeometry(
        const XS_BUILD_PARAMS& aParams,
        const std::vector<XS_NEIGHBOR>& aNeighbors,
        const std::vector<XS_GROUNDWIRE>& aGroundWires ) const
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

        if( geom.hOrigBelow > 0.0 )
        {
            // Demoted reference: substrate (original εr) from signal to
            // original ground level, air from there to virtual earth.
            // condY = -(hBelow + t/2), original ground was at hOrigBelow below signal
            double origRefY = condY + geom.hOrigBelow + geom.traceThickness / 2.0;

            XS_DIELECTRIC_REGION airAbove, substrate, airBelow;
            airAbove.yTop = -10e-3;
            airAbove.yBottom = condY + geom.traceThickness / 2.0;
            airAbove.epsilonR = 1.0;

            substrate.yTop = condY + geom.traceThickness / 2.0;
            substrate.yBottom = origRefY;
            substrate.epsilonR = geom.erOrigBelow;

            airBelow.yTop = origRefY;
            airBelow.yBottom = 0.0;
            airBelow.epsilonR = 1.0;

            xs.dielectrics.push_back( airAbove );
            xs.dielectrics.push_back( substrate );
            xs.dielectrics.push_back( airBelow );
            xs.epsilonR = geom.erOrigBelow;
        }
        else
        {
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

    // Groundwire conductors from intermediate reference layers.
    // Transform stackup z-positions to cross-section y-coordinates using the
    // same mapping as the signal conductor: offset from the signal's stackup z
    // converted to the XS coordinate system.
    for( const XS_GROUNDWIRE& gw : aGroundWires )
    {
        // The groundwire's offset from the signal in the stackup (meters).
        // Positive = toward the ground (below for hasBelow, above for hasAbove).
        double dzFromSignal = gw.yPositionM - geom.signalZPosition;

        // In the XS coordinate system, the signal is at condY.
        // Moving toward ground means moving toward y=0 (for hasBelow) or
        // toward upperGroundY (for hasAbove).  The stackup z-axis and XS
        // y-axis both increase downward, so the offset maps directly.
        double gwY = condY + dzFromSignal;

        XS_CONDUCTOR gwCond;
        gwCond.centerX = gw.lateralNm * 1e-9;
        gwCond.centerY = gwY;
        gwCond.width = gw.widthNm * 1e-9;
        gwCond.thickness = gw.thicknessM;
        gwCond.isGround = true;
        xs.conductors.push_back( gwCond );
    }

    return xs;
}
