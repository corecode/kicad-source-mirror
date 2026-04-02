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

#include <drc/drc_rtree.h>
#include <sipi/bem_2d_solver.h>
#include <sipi/cross_section_builder.h>
#include <sipi/stackup_reader.h>
#include <sipi/trace_path_walker.h>

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


bool SE_PROFILE::Compute( const BOARD* aBoard, int aNetCode,
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

    // Cache keyed on quantized cross-section to avoid redundant BEM solves.
    struct XS_KEY
    {
        PCB_LAYER_ID       layer;
        int                width;
        bool               refAbove;
        bool               refBelow;
        std::vector<int>   neighborKeys;

        bool operator<( const XS_KEY& o ) const
        {
            if( layer != o.layer ) return layer < o.layer;
            if( width != o.width ) return width < o.width;
            if( refAbove != o.refAbove ) return refAbove < o.refAbove;
            if( refBelow != o.refBelow ) return refBelow < o.refBelow;
            return neighborKeys < o.neighborKeys;
        }
    };

    struct XS_CACHED { double z0; double erEff; double zdiff; double erEffOdd; };
    std::map<XS_KEY, XS_CACHED> cache;

    // Sample at uniform distance intervals along the path
    double totalLen = walker.GetTotalLength(); // nm

    if( totalLen < 1.0 )
    {
        m_error = wxS( "Trace has zero length." );
        return false;
    }

    m_totalLength = totalLen;

    double sampleStep = std::max( totalLen / 500.0, 50000.0 ); // ~500 samples, min 50µm
    int    numSamples = std::max( 2, (int) ceil( totalLen / sampleStep ) + 1 );

    // Build non-via path index for interpolation
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

    int    segIdx = 0;
    double cumulativeDelayPs = 0.0;

    for( int si = 0; si < numSamples; si++ )
    {
        double sampleDist = si * sampleStep;

        if( sampleDist > totalLen )
            sampleDist = totalLen;

        // Advance to the segment containing this distance
        while( segIdx + 1 < (int) segs.size()
               && segs[segIdx + 1].dist <= sampleDist )
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
                double frac = std::clamp( ( sampleDist - seg.dist ) / segLen, 0.0, 1.0 );
                samplePos.x = seg.pos.x + (int) ( frac * ( next.pos.x - seg.pos.x ) );
                samplePos.y = seg.pos.y + (int) ( frac * ( next.pos.y - seg.pos.y ) );
            }
        }

        VECTOR2D   sampleTangent = blendTangent( sampleDist, seg.tangent,
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
        XS_KEY key;
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
        auto cacheIt = cache.find( key );

        if( cacheIt != cache.end() )
        {
            z0 = cacheIt->second.z0;
            erEff = cacheIt->second.erEff;
            zdiff = cacheIt->second.zdiff;
            erEffOdd = cacheIt->second.erEffOdd;
        }
        else
        {
            BEM_2D_SOLVER solver;
            solver.SetGeometry( xs );
            solver.SetPanelsPerEdge( 12 );

            if( solver.Solve() && solver.GetResult().Z0 > 0.0 )
            {
                z0 = solver.GetResult().Z0;
                erEff = std::max( solver.GetResult().erEff, 1.0 );
                zdiff = solver.GetResult().Zdiff;
                erEffOdd = solver.GetResult().erEffOdd;
            }

            cache[key] = { z0, erEff, zdiff, erEffOdd };
        }

        // Accumulate propagation delay
        double velocity = C_LIGHT_M_S / sqrt( erEff );
        double stepMeters = sampleStep * 1e-9;
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


bool DIFF_PROFILE::Compute( const BOARD* aBoard, int aNetCodeP, int aNetCodeN )
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
    if( !m_profileP.Compute( aBoard, aNetCodeP, VECTOR2I( 0, 0 ), &rtree, aNetCodeN ) )
    {
        m_error = wxS( "P: " ) + m_profileP.GetError();
        return false;
    }

    // Walk N from the same physical end as P so distance axes align.
    // Also pass P as coupled net so N sees the same neighbor environment.
    VECTOR2I nFrom( 0, 0 );

    if( m_profileP.GetStartPad() )
        nFrom = m_profileP.GetStartPad()->GetPosition();

    if( !m_profileN.Compute( aBoard, aNetCodeN, nFrom, &rtree, aNetCodeP ) )
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
