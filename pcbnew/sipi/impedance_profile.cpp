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

#include "impedance_profile.h"

#include <board.h>
#include <footprint.h>
#include <pad.h>
#include <pcb_track.h>
#include <zone.h>
#include <geometry/shape_line_chain.h>

#include <drc/drc_rtree.h>
#include <sipi/bem_2d_solver.h>
#include <sipi/cross_section_builder.h>
#include <sipi/stackup_reader.h>
#include <sipi/trace_path_walker.h>
#include <sipi/via_model.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>


static constexpr double C_LIGHT_M_S = 299792458.0;


// ============================================================================
// Tangent blending at bend vertices
// ============================================================================

struct TANGENT_BOUNDARY
{
    double   dist;           // Path distance at the bend vertex (nm)
    VECTOR2D tangentBefore;  // Incoming segment tangent (unit)
    VECTOR2D tangentAfter;   // Outgoing segment tangent (unit)
    double   blendRadius;    // Transition half-width (nm)
};


static VECTOR2D blendTangent( double aSampleDist, const VECTOR2D& aDefaultTangent,
                              const std::vector<TANGENT_BOUNDARY>& aBoundaries )
{
    const TANGENT_BOUNDARY* nearest = nullptr;
    double                  nearestAbsDist = std::numeric_limits<double>::max();

    for( const TANGENT_BOUNDARY& bnd : aBoundaries )
    {
        double absDist = std::abs( aSampleDist - bnd.dist );

        if( absDist < bnd.blendRadius && absDist < nearestAbsDist )
        {
            nearestAbsDist = absDist;
            nearest = &bnd;
        }
    }

    if( !nearest )
        return aDefaultTangent;

    double d = aSampleDist - nearest->dist;
    double t = std::clamp( ( d + nearest->blendRadius ) / ( 2.0 * nearest->blendRadius ), 0.0,
                           1.0 );

    VECTOR2D blended( ( 1.0 - t ) * nearest->tangentBefore.x + t * nearest->tangentAfter.x,
                      ( 1.0 - t ) * nearest->tangentBefore.y + t * nearest->tangentAfter.y );

    double len = blended.EuclideanNorm();

    if( len < 1e-12 )
        return aDefaultTangent;

    return blended / len;
}


// ============================================================================
// SE_PROFILE
// ============================================================================

SE_PROFILE::SE_PROFILE() = default;


static void buildRtree( const BOARD* aBoard, DRC_RTREE& aRtree )
{
    for( PCB_TRACK* t : aBoard->Tracks() )
    {
        if( t->Type() == PCB_VIA_T )
        {
            for( PCB_LAYER_ID layer : t->GetLayerSet().CuStack() )
                aRtree.Insert( t, layer );
        }
        else
        {
            aRtree.Insert( t, t->GetLayer() );
        }
    }

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            for( PCB_LAYER_ID layer : pad->GetLayerSet().CuStack() )
                aRtree.Insert( pad, layer );
        }
    }
}


// ============================================================================
// Via cluster extraction at a via transition point
// ============================================================================

static constexpr int VIA_COUPLING_RADIUS_NM = 3000000; // 3mm in nm
static constexpr double NM_TO_M_VIA = 1e-9;

/**
 * Extract via model for a signal via on the traced path.
 *
 * Finds nearby vias within the coupling radius, classifies them as signal
 * or return based on net code, extracts geometry from the stackup, and
 * runs the analytical multi-via parasitic extraction.
 *
 * @param aBoard       The board
 * @param aSignalVia   The via on the signal path
 * @param aSignalLayer The layer the signal enters/exits on
 * @param aNetCode     Signal net code (for role classification)
 * @param aCoupledNet  Coupled diff-pair net code (0 if none)
 * @param aStackup     Stackup reader (already built)
 * @return Shared pointer to the extraction result, or nullptr on failure.
 */
static std::shared_ptr<VIA_CLUSTER_RESULT>
extractViaModel( const BOARD* aBoard, const PCB_VIA* aSignalVia,
                 PCB_LAYER_ID aSignalLayer, int aNetCode, int aCoupledNet,
                 STACKUP_READER& aStackup )
{
    double drillR = aSignalVia->GetDrillValue() / 2.0 * NM_TO_M_VIA;
    double padR = aSignalVia->GetWidth( aSignalVia->TopLayer() ) / 2.0 * NM_TO_M_VIA;

    // Get stackup geometry for the signal via
    VIA_PARAMS sigParams = aStackup.GetViaGeometry( aSignalVia->TopLayer(),
                                                     aSignalVia->BottomLayer(),
                                                     aSignalLayer,
                                                     aSignalVia->GetPosition(),
                                                     drillR, padR );

    sigParams.position = aSignalVia->GetPosition();
    sigParams.role = VIA_ROLE::SIGNAL_P;

    std::vector<VIA_PARAMS> cluster;
    cluster.push_back( sigParams );

    // Find nearby vias within coupling radius
    VECTOR2I viaPos = aSignalVia->GetPosition();
    BOX2I searchBox( viaPos - VECTOR2I( VIA_COUPLING_RADIUS_NM, VIA_COUPLING_RADIUS_NM ),
                     VECTOR2I( VIA_COUPLING_RADIUS_NM * 2, VIA_COUPLING_RADIUS_NM * 2 ) );

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( track->Type() != PCB_VIA_T )
            continue;

        PCB_VIA* otherVia = static_cast<PCB_VIA*>( track );

        if( otherVia == aSignalVia )
            continue;

        VECTOR2I otherPos = otherVia->GetPosition();
        double dx = otherPos.x - viaPos.x;
        double dy = otherPos.y - viaPos.y;
        double dist = std::sqrt( dx * dx + dy * dy );

        if( dist > VIA_COUPLING_RADIUS_NM )
            continue;

        // Check that this via spans overlapping layers with the signal via
        // (otherwise it's on a different part of the stackup)
        if( otherVia->TopLayer() > aSignalVia->BottomLayer()
            || otherVia->BottomLayer() < aSignalVia->TopLayer() )
            continue;

        double otherDrillR = otherVia->GetDrillValue() / 2.0 * NM_TO_M_VIA;
        double otherPadR = otherVia->GetWidth( otherVia->TopLayer() ) / 2.0 * NM_TO_M_VIA;

        VIA_PARAMS otherParams = aStackup.GetViaGeometry( otherVia->TopLayer(),
                                                           otherVia->BottomLayer(),
                                                           aSignalLayer,
                                                           otherVia->GetPosition(),
                                                           otherDrillR, otherPadR );

        otherParams.position = otherVia->GetPosition();

        // Classify role
        int otherNet = otherVia->GetNetCode();

        if( otherNet == aNetCode )
        {
            // Same net — shouldn't happen for a clean diff pair, but treat as signal
            otherParams.role = VIA_ROLE::SIGNAL_P;
        }
        else if( aCoupledNet > 0 && otherNet == aCoupledNet )
        {
            otherParams.role = VIA_ROLE::SIGNAL_N;
        }
        else
        {
            // Different net — treat as return (ground/power via)
            otherParams.role = VIA_ROLE::RETURN;
        }

        cluster.push_back( otherParams );
    }

    // Run the extraction
    VIA_MODEL_BUILDER builder;
    builder.SetCluster( cluster );

    if( !builder.Compute() )
        return nullptr;

    return std::make_shared<VIA_CLUSTER_RESULT>( builder.GetResult() );
}


bool SE_PROFILE::Compute( const BOARD* aBoard, int aNetCode, BEM_CACHE& aCache,
                           const VECTOR2I& aFrom, DRC_RTREE* aRtree,
                           int aCoupledNetCode )
{
    m_samples.clear();
    m_totalLength = 0.0;
    m_totalDelay = 0.0;
    m_startPad = nullptr;
    m_endPad = nullptr;
    m_error.clear();

    if( !aBoard )
    {
        m_error = wxS( "No board." );
        return false;
    }

    TRACE_PATH_WALKER walker( aBoard );

    if( !walker.WalkNet( aNetCode, aFrom ) )
    {
        m_error = wxS( "Could not walk net — no routed tracks found." );
        return false;
    }

    const auto& path = walker.GetPath();

    m_startPad = walker.GetStartPad();
    m_endPad = walker.GetEndPad();

    // Build or reuse spatial index
    DRC_RTREE  ownRtree;
    DRC_RTREE* rtree = aRtree;

    if( !rtree )
    {
        buildRtree( aBoard, ownRtree );
        rtree = &ownRtree;
    }

    STACKUP_READER stackup( aBoard );

    BEM_CACHE& cache = aCache;

    // ---------------------------------------------------------------
    // Pre-process vias: extract models and compute barrel lengths.
    // Via barrel length is added to the total path length so that
    // vias appear as segments in the impedance profile.
    // ---------------------------------------------------------------

    struct VIA_ON_PATH
    {
        double                              pathDist;      // Original path distance at via center (nm)
        double                              barrelNm;      // Barrel height in nm
        double                              padRadiusNm;   // Pad radius (nm) — via interval extends this far from center
        double                              drillRadiusNm; // Drill radius (nm) — horizontal signal path = padR - drillR
        double                              z0Coax;        // Coaxial Z₀ (Ω)
        double                              erEff;         // Effective εr for vertical barrel delay
        double                              erEffHoriz;    // Effective εr for horizontal pad delay
        std::shared_ptr<VIA_CLUSTER_RESULT> model;
        VECTOR2I                            position;
        PCB_LAYER_ID                        layer;
    };

    std::vector<VIA_ON_PATH> viasOnPath;

    // Build non-via path index first (needed for signal layer detection)
    struct PATH_SEG
    {
        double       dist;
        VECTOR2I     pos;
        VECTOR2D     tangent;
        PCB_TRACK*   track;
    };

    std::vector<PATH_SEG> segs;

    for( const PATH_POINT& pp : path )
    {
        if( pp.isVia )
            continue;

        segs.push_back( { pp.distFromStart, pp.position, pp.tangent,
                          static_cast<PCB_TRACK*>( pp.item ) } );
    }

    if( segs.empty() )
    {
        m_error = wxS( "No impedance data — trace has no non-via segments." );
        return false;
    }

    // Extract via models and compute barrel lengths
    for( const PATH_POINT& pp : path )
    {
        if( !pp.isVia )
            continue;

        PCB_VIA* via = static_cast<PCB_VIA*>( pp.item );

        // Find signal layer from the nearest non-via segment
        PCB_LAYER_ID signalLayer = F_Cu;

        for( const PATH_SEG& seg : segs )
        {
            if( std::abs( seg.dist - pp.distFromStart ) < 1e6 ) // within 1mm
            {
                signalLayer = seg.track->GetLayer();
                break;
            }
        }

        auto viaResult = extractViaModel( aBoard, via, signalLayer,
                                           aNetCode, aCoupledNetCode, stackup );

        if( !viaResult )
            continue;

        VIA_ON_PATH vop;
        vop.pathDist = pp.distFromStart;
        vop.barrelNm = viaResult->fMax > 0.0
                         ? ( C_LIGHT_M_S / ( 10.0 * viaResult->fMax ) ) * 1e9  // h = c/(10*fmax*√εr)... but we stored fmax = c/(10h√εr)
                         : 0.0;

        // Recover barrel height from the VIA_PARAMS used to build the model.
        // The via model stores fMax = v_p/(10*h). So h = v_p/(10*fMax).
        // But simpler: query the stackup directly.
        VIA_PARAMS vp = stackup.GetViaGeometry( via->TopLayer(), via->BottomLayer(),
                                                 signalLayer, via->GetPosition(),
                                                 via->GetDrillValue() / 2.0 * 1e-9,
                                                 via->GetWidth( via->TopLayer() ) / 2.0 * 1e-9 );

        vop.barrelNm = vp.barrelHeight * 1e9; // meters → nm
        vop.padRadiusNm = via->GetWidth( via->TopLayer() ) / 2.0;  // already in nm
        vop.drillRadiusNm = via->GetDrillValue() / 2.0;            // already in nm
        vop.erEff = vp.epsilonReff;

        // Horizontal pad εr: query stackup at the signal layer.
        // The pad is wide, so εr_eff ≈ εr of the dielectric at that layer.
        {
            LAYER_GEOMETRY padGeom = stackup.GetLayerGeometry( signalLayer,
                                                               via->GetPosition(),
                                                               via->GetWidth( via->TopLayer() ) );
            // Use the reference plane dielectric εr (stripline → εr, microstrip → close to εr
            // for a wide pad).  Pick the side that has a reference plane.
            if( padGeom.hasRefAbove && padGeom.hasRefBelow )
                vop.erEffHoriz = ( padGeom.erAbove + padGeom.erBelow ) / 2.0;
            else if( padGeom.hasRefBelow )
                vop.erEffHoriz = padGeom.erBelow;
            else if( padGeom.hasRefAbove )
                vop.erEffHoriz = padGeom.erAbove;
            else
                vop.erEffHoriz = vp.epsilonReff; // fallback to barrel εr
        }

        // Coaxial Z₀ = √(L/C). If C=0, estimate from geometry.
        if( viaResult->Ctotal.size() > 0 && viaResult->Ctotal[0] > 0.0 )
            vop.z0Coax = std::sqrt( viaResult->Ldiff / viaResult->Ctotal[0] );
        else
            vop.z0Coax = 40.0; // Typical via impedance when C unknown

        vop.model = viaResult;
        vop.position = pp.position;
        vop.layer = signalLayer;

        viasOnPath.push_back( vop );
    }

    // Sort vias by path distance
    std::sort( viasOnPath.begin(), viasOnPath.end(),
               []( const VIA_ON_PATH& a, const VIA_ON_PATH& b )
               { return a.pathDist < b.pathDist; } );

    // Total path length = trace length + sum of via barrel lengths
    double traceLen = walker.GetTotalLength(); // nm
    double viaLenTotal = 0.0;

    for( const VIA_ON_PATH& vop : viasOnPath )
        viaLenTotal += vop.barrelNm;

    double totalLen = traceLen + viaLenTotal;

    if( totalLen < 1.0 )
    {
        m_error = wxS( "Trace has zero length." );
        return false;
    }

    m_totalLength = totalLen;

    double sampleStep = std::max( totalLen / 500.0, 50000.0 ); // ~500 samples, min 50µm

    // ---------------------------------------------------------------
    // Pre-scan reference layers along the trace to find gap boundaries.
    // Probing isReferencePlane along the trace direction catches antipads
    // and zone splits that a perpendicular cut line might miss.
    // Gap boundaries become explicit sample points so the profile never
    // misses a reference plane discontinuity.
    // ---------------------------------------------------------------

    // ---------------------------------------------------------------
    // Find reference layer obstacles (vias/pads) along the trace path.
    // For each via/pad on the reference layer, project its position onto
    // the nearest trace segment to find the closest-approach distance.
    // Inserting these as explicit sample points guarantees the perpendicular
    // cut line at that sample passes closest to the antipad, so it will
    // be detected by findCopperSpans() in the demotion check.
    // ---------------------------------------------------------------

    std::vector<double> refObstacleDistances; // trace distances near ref layer disruptions

    {
        // Determine the reference layers by probing mid-trace (the start may be
        // inside a pad antipad where isReferencePlane returns false).
        int midIdx = (int) segs.size() / 2;
        LAYER_GEOMETRY geom0 = stackup.GetLayerGeometry( segs[midIdx].track->GetLayer(),
                                                          segs[midIdx].pos,
                                                          segs[midIdx].track->GetWidth() );

        PCB_LAYER_ID refLayers[2] = { geom0.refLayerBelow, geom0.refLayerAbove };
        double hRef = std::max( geom0.hAbove, geom0.hBelow );
        // Search radius = extent + typical antipad radius (~1mm).
        // The via center may be beyond the extent, but its antipad edge
        // extends close enough to affect the signal.
        double searchRadius = std::max( 500000.0, hRef * 5.0 * 1e9 ) + 1000000.0;

        // Phase 1: Find zone fill outline edges that cross the trace path.
        // For each reference layer, intersect every trace segment with the
        // zone fill outlines.  Each crossing is a reference plane boundary
        // that must have an explicit sample point.
        for( PCB_LAYER_ID refLayer : refLayers )
        {
            if( refLayer == UNDEFINED_LAYER )
                continue;

            for( ZONE* zone : aBoard->Zones() )
            {
                if( !zone->IsOnLayer( refLayer ) || zone->GetIsRuleArea() )
                    continue;

                const auto& fill = zone->GetFilledPolysList( refLayer );

                if( !fill || fill->IsEmpty() )
                    continue;

                for( int pi = 0; pi < fill->OutlineCount(); pi++ )
                {
                    const SHAPE_LINE_CHAIN& outline = fill->COutline( pi );

                    for( int si = 0; si + 1 < (int) segs.size(); si++ )
                    {
                        SEG traceSeg( segs[si].pos, segs[si + 1].pos );
                        SHAPE_LINE_CHAIN::INTERSECTIONS hits;
                        outline.Intersect( traceSeg, hits );

                        for( const auto& hit : hits )
                        {
                            // Convert board position to trace distance
                            double segStart = segs[si].dist;
                            double segEnd = segs[si + 1].dist;
                            double segLen = segEnd - segStart;

                            if( segLen < 1.0 )
                                continue;

                            double hitFrac = VECTOR2D( hit.p - segs[si].pos ).EuclideanNorm()
                                             / VECTOR2D( segs[si + 1].pos - segs[si].pos ).EuclideanNorm();

                            double hitDist = segStart + hitFrac * segLen;

                            // Offset slightly to each side of the edge so the
                            // sample falls clearly inside or outside the zone,
                            // not on the ambiguous boundary.
                            static constexpr double EDGE_NUDGE = 5000.0; // 5µm

                            refObstacleDistances.push_back( hitDist - EDGE_NUDGE );
                            refObstacleDistances.push_back( hitDist + EDGE_NUDGE );
                        }
                    }
                }
            }
        }

        // Phase 2: Collect all pads/vias on the reference layers
        std::set<BOARD_ITEM*> refObstacles;

        for( PCB_TRACK* t : aBoard->Tracks() )
        {
            if( t->Type() != PCB_VIA_T )
                continue;

            for( PCB_LAYER_ID refLayer : refLayers )
            {
                if( refLayer != UNDEFINED_LAYER && t->IsOnLayer( refLayer ) )
                    refObstacles.insert( t );
            }
        }

        for( FOOTPRINT* fp : aBoard->Footprints() )
        {
            for( PAD* pad : fp->Pads() )
            {
                for( PCB_LAYER_ID refLayer : refLayers )
                {
                    if( refLayer != UNDEFINED_LAYER && pad->IsOnLayer( refLayer ) )
                        refObstacles.insert( pad );
                }
            }
        }

        // For each obstacle, find the closest approach along any trace segment
        for( BOARD_ITEM* item : refObstacles )
        {
            VECTOR2I itemPos = item->GetPosition();
            double   bestDist = searchRadius;
            double   bestTraceDist = -1.0;

            for( int i = 0; i + 1 < (int) segs.size(); i++ )
            {
                VECTOR2D segDir( segs[i + 1].pos.x - segs[i].pos.x,
                                 segs[i + 1].pos.y - segs[i].pos.y );
                double segDirLen = segDir.EuclideanNorm();

                if( segDirLen < 1.0 )
                    continue;

                VECTOR2D segUnit = segDir / segDirLen;
                VECTOR2D toItem( itemPos.x - segs[i].pos.x,
                                 itemPos.y - segs[i].pos.y );

                double proj = toItem.x * segUnit.x + toItem.y * segUnit.y;
                proj = std::clamp( proj, 0.0, segDirLen );

                VECTOR2D closest( segs[i].pos.x + segUnit.x * proj,
                                  segs[i].pos.y + segUnit.y * proj );
                double perpDist = VECTOR2D( itemPos.x - closest.x,
                                            itemPos.y - closest.y ).EuclideanNorm();

                if( perpDist < bestDist )
                {
                    bestDist = perpDist;
                    bestTraceDist = segs[i].dist + proj;
                }
            }

            if( bestTraceDist >= 0.0 )
                refObstacleDistances.push_back( bestTraceDist );
        }

        fprintf( stderr, "SIPI: ref layers: below=%d above=%d\n",
                 (int) refLayers[0], (int) refLayers[1] );
        fprintf( stderr, "SIPI: ref obstacle scan: %d obstacles on ref layers, %d within range\n",
                 (int) refObstacles.size(), (int) refObstacleDistances.size() );

        for( double d : refObstacleDistances )
            fprintf( stderr, "  obstacle at trace dist=%.3fmm\n", d / 1e6 );
    }

    // Build sample distances: uniform grid + gap boundaries (with margin).
    // Margin samples before/after the gap show the impedance ramp.
    std::vector<double> sampleDists;
    int numUniform = std::max( 2, (int) ceil( totalLen / sampleStep ) + 1 );

    for( int si = 0; si < numUniform; si++ )
    {
        double d = si * sampleStep;

        if( d > totalLen )
            d = totalLen;

        sampleDists.push_back( d );
    }

    for( double gapDist : refObstacleDistances )
    {
        // Convert trace distance to profile distance (add via barrel offsets)
        double cumulViaOff = 0.0;

        for( const VIA_ON_PATH& vop : viasOnPath )
        {
            if( vop.pathDist < gapDist )
                cumulViaOff += vop.barrelNm;
        }

        double profileDist = gapDist + cumulViaOff;

        // Insert the boundary and samples at ±1× and ±2× sampleStep
        for( double offset : { -2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0 } )
        {
            double d = profileDist + offset * sampleStep;

            if( d >= 0.0 && d <= totalLen )
                sampleDists.push_back( d );
        }
    }

    // Sort and remove near-duplicates
    std::sort( sampleDists.begin(), sampleDists.end() );
    {
        std::vector<double> deduped;
        deduped.reserve( sampleDists.size() );

        for( double d : sampleDists )
        {
            if( deduped.empty() || d - deduped.back() > sampleStep * 0.1 )
                deduped.push_back( d );
        }

        sampleDists = std::move( deduped );
    }

    int numSamples = (int) sampleDists.size();

    // Detect bend vertices where the tangent changes direction.
    // Build a list of tangent boundaries for smooth blending.
    std::vector<TANGENT_BOUNDARY> tangentBoundaries;

    for( int i = 0; i + 1 < (int) segs.size(); i++ )
    {
        if( segs[i].track == segs[i + 1].track )
            continue;

        double dot = segs[i].tangent.x * segs[i + 1].tangent.x
                     + segs[i].tangent.y * segs[i + 1].tangent.y;

        if( dot > 0.9999 )
            continue;

        TANGENT_BOUNDARY bnd;
        bnd.dist = segs[i + 1].dist;
        bnd.tangentBefore = segs[i].tangent;
        bnd.tangentAfter = segs[i + 1].tangent;

        // Compute adjacent segment lengths for blend radius clamping
        double segLenBefore = 0.0;

        for( int j = i; j >= 0; j-- )
        {
            if( segs[j].track == segs[i].track )
                segLenBefore = segs[i].dist - segs[j].dist;
            else
                break;
        }

        double segLenAfter = 0.0;

        for( int j = i + 1; j < (int) segs.size(); j++ )
        {
            if( segs[j].track == segs[i + 1].track )
                segLenAfter = segs[j].dist - segs[i + 1].dist;
            else
                break;
        }

        double traceWidth = std::max( segs[i].track->GetWidth(),
                                      segs[i + 1].track->GetWidth() );
        bnd.blendRadius = 1.0 * traceWidth;

        if( segLenBefore > 0.0 )
            bnd.blendRadius = std::min( bnd.blendRadius, segLenBefore / 2.0 );

        if( segLenAfter > 0.0 )
            bnd.blendRadius = std::min( bnd.blendRadius, segLenAfter / 2.0 );

        if( bnd.blendRadius < 1000.0 )
            continue;

        tangentBoundaries.push_back( bnd );
    }

    // Map path items → path distance for topological neighbor exclusion
    std::map<BOARD_CONNECTED_ITEM*, double> pathItemDist;

    for( const PATH_POINT& pp : path )
    {
        if( !pp.isVia && pathItemDist.find( pp.item ) == pathItemDist.end() )
            pathItemDist[pp.item] = pp.distFromStart;
    }

    // Build via intervals in profile-distance space.
    // Each via occupies [padEdge, padEdge + 2*padR + barrel] in the profile.
    struct VIA_INTERVAL
    {
        double profileStart;  // Profile distance where via interval begins (pad edge, nm)
        double profileEnd;    // Profile distance where via interval ends (pad edge, nm)
        double barrelNm;      // Barrel length only (for profileToTrace offset)
        int    viaIdx;        // Index into viasOnPath
    };

    std::vector<VIA_INTERVAL> viaIntervals;
    {
        double cumulViaOffset = 0.0;

        // Walk trace distance and insert via intervals.
        // Each via interval extends padRadius before and after the via center
        // in trace space.  The pad portions replace trace; only the barrel
        // adds extra length to the profile.
        for( int vi = 0; vi < (int) viasOnPath.size(); vi++ )
        {
            double traceD = viasOnPath[vi].pathDist;    // trace distance of via center
            double padR = viasOnPath[vi].padRadiusNm;
            double barrel = viasOnPath[vi].barrelNm;

            // Via interval starts padRadius before the center in trace space
            double profileStart = ( traceD - padR ) + cumulViaOffset;
            double totalViaLen = padR + barrel + padR;  // top pad + barrel + bottom pad

            VIA_INTERVAL interval;
            interval.profileStart = std::max( 0.0, profileStart );
            interval.profileEnd = interval.profileStart + totalViaLen;
            interval.barrelNm = barrel;
            interval.viaIdx = vi;
            viaIntervals.push_back( interval );

            // Only the barrel adds net length; pad portions were already trace distance
            cumulViaOffset += barrel;
        }
    }

    // Helper: convert profile distance → trace distance (subtract via barrel lengths before this point).
    // Only the barrel portion is "extra" profile distance; the pad portions replaced trace
    // distance, so we subtract barrelNm per fully-passed via interval.
    auto profileToTrace = [&]( double aProfileDist ) -> double
    {
        double viaOffset = 0.0;

        for( const VIA_INTERVAL& vi : viaIntervals )
        {
            if( aProfileDist <= vi.profileStart )
                break;

            if( aProfileDist >= vi.profileEnd )
                viaOffset += vi.barrelNm;
            // If inside a via interval, profileToTrace shouldn't be called
            // (via samples are handled separately).  But if it is, don't
            // add partial offset — the caller will get a trace distance
            // near the via entry, which is acceptable for interpolation.
        }

        return aProfileDist - viaOffset;
    };

    // Helper: check if a profile distance is inside a via barrel
    auto findViaInterval = [&]( double aProfileDist ) -> const VIA_INTERVAL*
    {
        for( const VIA_INTERVAL& vi : viaIntervals )
        {
            if( aProfileDist >= vi.profileStart && aProfileDist < vi.profileEnd )
                return &vi;
        }

        return nullptr;
    };

    int    segIdx = 0;
    double cumulativeDelayPs = 0.0;
    double prevSampleDist = 0.0;

    for( int si = 0; si < numSamples; si++ )
    {
        double sampleDist = sampleDists[si];
        double actualStep = sampleDist - prevSampleDist; // non-uniform step
        prevSampleDist = sampleDist;

        // Check if this sample falls inside a via barrel
        const VIA_INTERVAL* inVia = findViaInterval( sampleDist );

        if( inVia )
        {
            const VIA_ON_PATH& vop = viasOnPath[inVia->viaIdx];

            // Via sample: determine sub-region (top pad / barrel / bottom pad)
            // and accumulate delay with the appropriate velocity.
            //
            // The via interval is: [profileStart .. +padR .. +padR+barrel .. +2*padR+barrel]
            //                       |-- top pad --|-- barrel --|-- bottom pad --|
            //
            // Signal path in pad = (padR - drillR), mapped over padR of profile distance.
            // Signal path in barrel = barrelH, mapped 1:1.
            double posInVia = sampleDist - inVia->profileStart;
            double padR = vop.padRadiusNm;
            double barrel = vop.barrelNm;
            double stepNm = actualStep;

            double erEffLocal;
            double effectiveStepNm;

            if( posInVia < padR )
            {
                // Top pad: physical signal path = (padR - drillR), spread over padR
                erEffLocal = vop.erEffHoriz;
                effectiveStepNm = ( padR > 0.0 )
                                      ? stepNm * ( padR - vop.drillRadiusNm ) / padR
                                      : 0.0;
            }
            else if( posInVia > padR + barrel )
            {
                // Bottom pad: same scaling as top pad
                erEffLocal = vop.erEffHoriz;
                effectiveStepNm = ( padR > 0.0 )
                                      ? stepNm * ( padR - vop.drillRadiusNm ) / padR
                                      : 0.0;
            }
            else
            {
                // Vertical barrel: 1:1 mapping
                erEffLocal = vop.erEff;
                effectiveStepNm = stepNm;
            }

            double velocity = C_LIGHT_M_S / std::sqrt( erEffLocal );
            double effectiveStepM = effectiveStepNm * 1e-9;
            cumulativeDelayPs += ( effectiveStepM / velocity ) * 1e12;

            IMPEDANCE_SAMPLE sample;
            sample.distNm = sampleDist;
            sample.delayPs = cumulativeDelayPs;
            sample.z0 = vop.z0Coax;
            sample.erEff = erEffLocal;
            sample.boardPos = vop.position;
            sample.tangent = VECTOR2D( 0, 0 );
            sample.layer = vop.layer;
            sample.signalWidth = 0;
            sample.isVia = true;
            sample.viaModel = vop.model;

            m_samples.push_back( sample );
            continue;
        }

        // Trace sample: map profile distance → trace distance for interpolation
        double traceDist = profileToTrace( sampleDist );

        // Advance to the segment containing this trace distance
        while( segIdx + 1 < (int) segs.size()
               && segs[segIdx + 1].dist <= traceDist )
        {
            segIdx++;
        }

        const PATH_SEG& seg = segs[segIdx];

        // Interpolate position along the segment
        VECTOR2I samplePos = seg.pos;

        if( segIdx + 1 < (int) segs.size() )
        {
            const PATH_SEG& next = segs[segIdx + 1];
            double segLen = next.dist - seg.dist;

            if( segLen > 1.0 )
            {
                double frac = std::clamp( ( traceDist - seg.dist ) / segLen, 0.0, 1.0 );
                samplePos.x = seg.pos.x + (int) ( frac * ( next.pos.x - seg.pos.x ) );
                samplePos.y = seg.pos.y + (int) ( frac * ( next.pos.y - seg.pos.y ) );
            }
        }

        VECTOR2D   sampleTangent = blendTangent( traceDist, seg.tangent,
                                                    tangentBoundaries );
        PCB_TRACK* track = seg.track;
        int        signalWidth = track->GetWidth();

        // Check if inside a pad — use pad width
        for( PAD* pad : aBoard->GetPads() )
        {
            if( pad->GetNetCode() != track->GetNetCode() )
                continue;

            if( !pad->IsOnLayer( track->GetLayer() ) )
                continue;

            if( pad->HitTest( samplePos, 0 ) )
            {
                VECTOR2D normal0( -sampleTangent.y, sampleTangent.x );
                BOX2I    padBBox = pad->GetBoundingBox();

                double minProj = 1e18, maxProj = -1e18;

                for( const VECTOR2I& corner :
                     { padBBox.GetOrigin(),
                       padBBox.GetOrigin() + VECTOR2I( padBBox.GetWidth(), 0 ),
                       padBBox.GetOrigin() + VECTOR2I( 0, padBBox.GetHeight() ),
                       padBBox.GetEnd() } )
                {
                    VECTOR2D d( corner.x - samplePos.x, corner.y - samplePos.y );
                    double proj = d.x * normal0.x + d.y * normal0.y;
                    minProj = std::min( minProj, proj );
                    maxProj = std::max( maxProj, proj );
                }

                int padWidth = (int) ( maxProj - minProj );

                if( padWidth > signalWidth )
                    signalWidth = padWidth;

                break;
            }
        }

        LAYER_GEOMETRY geom = stackup.GetLayerGeometry( track->GetLayer(),
                                                        samplePos, signalWidth );

        double hRef = std::max( geom.hAbove, geom.hBelow );
        int    couplingHorizon = std::clamp( (int) ( hRef * 3.0 * 1e9 ), 500000, 3000000 );

        CROSS_SECTION_BUILDER xsBuilder;
        xsBuilder.SetSpatialIndex( rtree );
        xsBuilder.SetBoard( aBoard );
        xsBuilder.SetPathItemDistances( &pathItemDist );
        xsBuilder.SetSignalTrack( track );

        XS_BUILD_PARAMS xsParams;
        xsParams.samplePos = samplePos;
        xsParams.sampleTangent = sampleTangent;
        xsParams.signalLayer = track->GetLayer();
        xsParams.signalNetCode = track->GetNetCode();
        xsParams.signalWidth = signalWidth;
        xsParams.couplingHorizon = couplingHorizon;
        xsParams.sampleDist = sampleDist;
        xsParams.coupledNetCode = aCoupledNetCode;
        xsParams.layerGeom = geom;

        // Find diff pair partner on the cut line (if coupled net specified)
        XS_DIFF_PAIR_CONDUCTOR dpConductor;

        if( aCoupledNetCode > 0 )
            dpConductor = xsBuilder.FindDiffPairConductor( xsParams );

        // Extend coupling horizon to see conductors on N's far side
        if( dpConductor.found )
            xsParams.couplingHorizon = std::abs( dpConductor.lateralNm ) + couplingHorizon;

        auto neighbors = xsBuilder.FindNeighbors( xsParams );
        auto groundWires = xsBuilder.FindGroundWires( xsParams, geom );

        xsParams.layerGeom = geom; // may be modified by demotion

        XS_GEOMETRY xs = xsBuilder.BuildGeometry( xsParams, neighbors, groundWires,
                                                   dpConductor.found ? &dpConductor
                                                                     : nullptr );

        // Build cache key
        BEM_CACHE::KEY key;
        key.layer = track->GetLayer();
        key.width = signalWidth;
        key.refAbove = geom.hasRefAbove;
        key.refBelow = geom.hasRefBelow;

        for( const XS_NEIGHBOR& nb : neighbors )
        {
            key.neighborKeys.push_back( nb.distNm / 10000 );
            key.neighborKeys.push_back( nb.widthNm );
        }

        for( const XS_GROUNDWIRE& gw : groundWires )
        {
            key.neighborKeys.push_back( gw.lateralNm / 10000 );
            key.neighborKeys.push_back( gw.widthNm );
        }

        if( dpConductor.found )
        {
            key.neighborKeys.push_back( dpConductor.lateralNm / 10000 );
            key.neighborKeys.push_back( dpConductor.widthNm );
        }

        // Solve (with cache)
        double z0 = 0.0;
        double erEff = 1.0;
        double zdiff = 0.0;
        double erEffOdd = 0.0;

        const BEM_CACHE::VALUE* cached = cache.Find( key );

        if( cached )
        {
            z0 = cached->z0;
            erEff = cached->erEff;
            zdiff = cached->zdiff;
            erEffOdd = cached->erEffOdd;
        }
        else
        {
            BEM_2D_SOLVER solver;
            solver.SetGeometry( xs );
            solver.SetPanelsPerEdge( 5 );

            if( solver.Solve() && solver.GetResult().Z0 > 0.0 )
            {
                z0 = solver.GetResult().Z0;
                erEff = std::max( solver.GetResult().erEff, 1.0 );
                zdiff = solver.GetResult().Zdiff;
                erEffOdd = solver.GetResult().erEffOdd;
            }

            cache.Insert( key, { z0, erEff, zdiff, erEffOdd } );
        }

        // Accumulate propagation delay
        double velocity = C_LIGHT_M_S / sqrt( erEff );
        double stepMeters = actualStep * 1e-9;
        cumulativeDelayPs += ( stepMeters / velocity ) * 1e12;

        // Count neighbor types
        int nbCount = 0;
        int gwCount = 0;

        for( size_t i = 1; i < xs.conductors.size(); i++ )
        {
            if( xs.conductors[i].isGround )
                gwCount++;
            else
                nbCount++;
        }

        // Store sample
        IMPEDANCE_SAMPLE sample;
        sample.distNm = sampleDist;
        sample.delayPs = cumulativeDelayPs;
        sample.z0 = z0;
        sample.erEff = erEff;
        sample.geometry = xs;
        sample.boardPos = samplePos;
        sample.tangent = sampleTangent;
        sample.layer = track->GetLayer();
        sample.signalWidth = signalWidth;
        sample.hasRefAbove = geom.hasRefAbove;
        sample.hasRefBelow = geom.hasRefBelow;
        sample.hAbove = geom.hAbove;
        sample.hBelow = geom.hBelow;
        sample.neighborCount = nbCount;
        sample.groundwireCount = gwCount;
        sample.zdiff = zdiff;
        sample.erEffOdd = erEffOdd;
        sample.isDiffPair = dpConductor.found && zdiff > 0.0;

        if( dpConductor.found )
        {
            sample.dpGapUm = ( std::abs( dpConductor.lateralNm )
                               - signalWidth / 2.0
                               - dpConductor.widthNm / 2.0 ) / 1000.0;
        }

        m_samples.push_back( sample );
    }

    if( !m_samples.empty() )
        m_totalDelay = m_samples.back().delayPs;

    return !m_samples.empty();
}


IMPEDANCE_SAMPLE SE_PROFILE::AtDelay( double aDelayPs ) const
{
    if( m_samples.empty() )
        return IMPEDANCE_SAMPLE();

    if( aDelayPs <= m_samples.front().delayPs )
        return m_samples.front();

    if( aDelayPs >= m_samples.back().delayPs )
        return m_samples.back();

    // Binary search for the bracketing samples
    auto it = std::lower_bound( m_samples.begin(), m_samples.end(), aDelayPs,
                                []( const IMPEDANCE_SAMPLE& s, double d )
                                { return s.delayPs < d; } );

    if( it == m_samples.begin() )
        return *it;

    const IMPEDANCE_SAMPLE& hi = *it;
    const IMPEDANCE_SAMPLE& lo = *std::prev( it );

    double span = hi.delayPs - lo.delayPs;

    if( span < 1e-12 )
        return lo;

    double frac = ( aDelayPs - lo.delayPs ) / span;

    // Linearly interpolate key fields
    IMPEDANCE_SAMPLE result = lo;
    result.delayPs = aDelayPs;
    result.distNm = lo.distNm + frac * ( hi.distNm - lo.distNm );
    result.z0 = lo.z0 + frac * ( hi.z0 - lo.z0 );
    result.erEff = lo.erEff + frac * ( hi.erEff - lo.erEff );
    result.boardPos.x = lo.boardPos.x + (int) ( frac * ( hi.boardPos.x - lo.boardPos.x ) );
    result.boardPos.y = lo.boardPos.y + (int) ( frac * ( hi.boardPos.y - lo.boardPos.y ) );
    result.signalWidth = lo.signalWidth + (int) ( frac * ( hi.signalWidth - lo.signalWidth ) );

    // Use lo's geometry/tangent/layer (not interpolated — discrete)
    return result;
}


// ============================================================================
// DIFF_PROFILE
// ============================================================================

DIFF_PROFILE::DIFF_PROFILE() = default;


bool DIFF_PROFILE::Compute( const BOARD* aBoard, int aNetCodeP, int aNetCodeN,
                             BEM_CACHE& aCache )
{
    m_diffSamples.clear();
    m_skewPs = 0.0;
    m_error.clear();

    if( !aBoard )
    {
        m_error = wxS( "No board." );
        return false;
    }

    // Build shared spatial index
    DRC_RTREE rtree;
    buildRtree( aBoard, rtree );

    // Walk P with diff-pair detection (finds N geometrically on the cut line)
    if( !m_profileP.Compute( aBoard, aNetCodeP, aCache, VECTOR2I( 0, 0 ), &rtree,
                             aNetCodeN ) )
    {
        m_error = wxS( "P: " ) + m_profileP.GetError();
        return false;
    }

    // Walk N from the same physical end as P so distance axes align.
    // Also pass P as coupled net so N sees the same neighbor environment.
    VECTOR2I nFrom( 0, 0 );

    if( m_profileP.GetStartPad() )
        nFrom = m_profileP.GetStartPad()->GetPosition();

    if( !m_profileN.Compute( aBoard, aNetCodeN, aCache, nFrom, &rtree, aNetCodeP ) )
    {
        m_error = wxS( "N: " ) + m_profileN.GetError();
        return false;
    }

    m_skewPs = m_profileN.GetTotalDelay() - m_profileP.GetTotalDelay();

    buildFromProfiles();

    return !m_diffSamples.empty();
}


void DIFF_PROFILE::buildFromProfiles()
{
    const auto& pSamples = m_profileP.GetSamples();
    const auto& nSamples = m_profileN.GetSamples();

    if( pSamples.empty() || nSamples.empty() )
        return;

    // Build anchor points at coupled samples where we know the exact P↔N
    // correspondence.  At coupled points the traces are close and parallel,
    // so board-position proximity reliably finds the matching N sample.
    // Between coupled regions (pad fan-out, via transitions, serpentine
    // offsets) we interpolate the path-distance offset from anchors.
    struct ANCHOR
    {
        double pDist;   // P path distance (nm)
        double nDist;   // Corresponding N path distance (nm)
    };

    std::vector<ANCHOR> anchors;

    for( const IMPEDANCE_SAMPLE& sp : pSamples )
    {
        if( !sp.isDiffPair )
            continue;

        double bestBoardDist = 1e18;
        double bestNDist = sp.distNm;

        for( const IMPEDANCE_SAMPLE& sn : nSamples )
        {
            double d = VECTOR2D( sp.boardPos - sn.boardPos ).EuclideanNorm();

            if( d < bestBoardDist )
            {
                bestBoardDist = d;
                bestNDist = sn.distNm;
            }
        }

        anchors.push_back( { sp.distNm, bestNDist } );
    }

    // Helper: interpolate N path distance for a given P path distance
    // using the anchor points.  Extrapolates with nearest anchor's offset.
    auto interpNDist = [&]( double aPDist ) -> double
    {
        if( anchors.empty() )
            return aPDist;  // no coupled points — assume 1:1

        if( aPDist <= anchors.front().pDist )
        {
            // Before first anchor — use first anchor's offset
            return aPDist + ( anchors.front().nDist - anchors.front().pDist );
        }

        if( aPDist >= anchors.back().pDist )
        {
            // After last anchor — use last anchor's offset
            return aPDist + ( anchors.back().nDist - anchors.back().pDist );
        }

        // Between anchors — binary search then lerp
        auto it = std::lower_bound( anchors.begin(), anchors.end(), aPDist,
                                    []( const ANCHOR& a, double d )
                                    { return a.pDist < d; } );

        if( it == anchors.begin() )
            return aPDist + ( it->nDist - it->pDist );

        const ANCHOR& hi = *it;
        const ANCHOR& lo = *std::prev( it );
        double frac = ( aPDist - lo.pDist ) / ( hi.pDist - lo.pDist );

        double loOffset = lo.nDist - lo.pDist;
        double hiOffset = hi.nDist - hi.pDist;

        return aPDist + loOffset + frac * ( hiOffset - loOffset );
    };

    // Helper: look up Z0 on N at an arbitrary path distance by linear
    // interpolation between surrounding N samples.
    auto lookupN = [&]( double aNDist ) -> double
    {
        auto it = std::lower_bound( nSamples.begin(), nSamples.end(), aNDist,
                                    []( const IMPEDANCE_SAMPLE& s, double d )
                                    { return s.distNm < d; } );

        if( it == nSamples.begin() )
            return it->z0;

        if( it == nSamples.end() )
            return nSamples.back().z0;

        const IMPEDANCE_SAMPLE& hi = *it;
        const IMPEDANCE_SAMPLE& lo = *std::prev( it );
        double span = hi.distNm - lo.distNm;

        if( span < 1.0 )
            return lo.z0;

        double frac = ( aNDist - lo.distNm ) / span;
        return lo.z0 + frac * ( hi.z0 - lo.z0 );
    };

    // Build diff samples
    for( const IMPEDANCE_SAMPLE& sp : pSamples )
    {
        DIFF_SAMPLE ds;
        ds.delayPs = sp.delayPs;
        ds.distMm = sp.distNm / 1e6;
        ds.z0P = sp.z0;
        ds.boardPosP = sp.boardPos;
        ds.geometry = sp.geometry;

        if( sp.isDiffPair )
        {
            ds.coupled = true;
            ds.zdiff = sp.zdiff;
            ds.erEffOdd = sp.erEffOdd;
            ds.gapUm = sp.dpGapUm;
        }

        double nDist = interpNDist( sp.distNm );
        ds.z0N = lookupN( nDist );

        if( !ds.coupled )
        {
            ds.zdiff = sp.z0 + ds.z0N;
            ds.erEffOdd = sp.erEff;
        }

        m_diffSamples.push_back( ds );
    }
}
