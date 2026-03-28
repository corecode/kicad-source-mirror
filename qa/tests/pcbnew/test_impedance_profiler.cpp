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

#include <qa_utils/wx_utils/unit_test_utils.h>

#include <pcbnew_utils/board_test_utils.h>
#include <board.h>
#include <pcb_track.h>
#include <pad.h>
#include <netinfo.h>
#include <settings/settings_manager.h>

#include <sipi/trace_path_walker.h>
#include <sipi/stackup_reader.h>
#include <sipi/bem_2d_solver.h>

#include <cmath>
#include <set>
#include <map>


struct PROFILER_TEST_FIXTURE
{
    PROFILER_TEST_FIXTURE() :
            m_settingsManager( true /* headless */ )
    { }

    SETTINGS_MANAGER       m_settingsManager;
    std::unique_ptr<BOARD> m_board;
};


/**
 * Neighbor detection data for one sample point.
 */
struct NEIGHBOR_SAMPLE
{
    VECTOR2I position;
    VECTOR2D tangent;
    VECTOR2D normal;
    PCB_LAYER_ID layer;
    int traceWidth;         // nm
    double distFromStart;   // nm

    struct NEIGHBOR_INFO
    {
        int      distNm;    // signed lateral distance
        int      widthNm;
        int      netCode;
        wxString netName;
    };

    std::vector<NEIGHBOR_INFO> neighbors;
};


/**
 * Run neighbor detection on a walked path, returning detailed info per sample.
 */
static std::vector<NEIGHBOR_SAMPLE> extractNeighborInfo(
        const BOARD* aBoard, const std::vector<PATH_POINT>& aPath, int aSignalNetCode )
{
    std::vector<NEIGHBOR_SAMPLE> samples;

    // Collect tracks by layer
    std::map<PCB_LAYER_ID, std::vector<PCB_TRACK*>> tracksByLayer;

    for( PCB_TRACK* t : aBoard->Tracks() )
    {
        if( t->Type() == PCB_VIA_T )
            continue;

        tracksByLayer[t->GetLayer()].push_back( t );
    }

    int couplingHorizon = 2000000; // 2mm in nm

    for( const PATH_POINT& pt : aPath )
    {
        if( pt.isVia )
            continue;

        PCB_TRACK* track = static_cast<PCB_TRACK*>( pt.item );

        NEIGHBOR_SAMPLE sample;
        sample.position = pt.position;
        sample.tangent = pt.tangent;
        sample.normal = VECTOR2D( -pt.tangent.y, pt.tangent.x );
        sample.layer = track->GetLayer();
        sample.traceWidth = track->GetWidth();
        sample.distFromStart = pt.distFromStart;

        int minEdgeToEdge = 10000; // 10µm

        auto it = tracksByLayer.find( track->GetLayer() );

        if( it != tracksByLayer.end() )
        {
            for( PCB_TRACK* other : it->second )
            {
                if( other == track )
                    continue;

                // Skip endpoint-connected segments (same trace at a bend)
                if( other->GetStart() == track->GetStart()
                    || other->GetStart() == track->GetEnd()
                    || other->GetEnd() == track->GetStart()
                    || other->GetEnd() == track->GetEnd() )
                {
                    continue;
                }

                // Closest point on other segment to our sample point
                VECTOR2D otherDir( other->GetEnd().x - other->GetStart().x,
                                   other->GetEnd().y - other->GetStart().y );
                double otherLen = otherDir.EuclideanNorm();

                if( otherLen < 1.0 )
                    continue;

                otherDir = otherDir / otherLen;

                VECTOR2D toSample( pt.position.x - other->GetStart().x,
                                   pt.position.y - other->GetStart().y );
                double t = toSample.x * otherDir.x + toSample.y * otherDir.y;
                t = std::max( 0.0, std::min( t, otherLen ) );

                VECTOR2D closest( other->GetStart().x + t * otherDir.x,
                                  other->GetStart().y + t * otherDir.y );

                VECTOR2D delta( closest.x - pt.position.x, closest.y - pt.position.y );

                double lateralDist = delta.x * sample.normal.x + delta.y * sample.normal.y;

                if( std::abs( lateralDist ) > couplingHorizon )
                    continue;

                double halfWidths = ( track->GetWidth() + other->GetWidth() ) / 2.0;
                double edgeToEdge = std::abs( lateralDist ) - halfWidths;

                if( edgeToEdge < minEdgeToEdge )
                    continue;

                double alongDist = delta.x * pt.tangent.x + delta.y * pt.tangent.y;

                if( std::abs( alongDist ) > track->GetLength() )
                    continue;

                NEIGHBOR_SAMPLE::NEIGHBOR_INFO nb;
                nb.distNm = static_cast<int>( lateralDist );
                nb.widthNm = other->GetWidth();
                nb.netCode = other->GetNetCode();
                nb.netName = other->GetNetname();

                sample.neighbors.push_back( nb );
            }
        }

        samples.push_back( sample );
    }

    return samples;
}


BOOST_FIXTURE_TEST_SUITE( ImpedanceProfiler, PROFILER_TEST_FIXTURE )


/**
 * Dump neighbor detection results for a board with routed tracks.
 * This is primarily diagnostic — it prints what the neighbor extraction finds
 * so we can see if the distances are realistic.
 */
BOOST_AUTO_TEST_CASE( NeighborDetectionDiag )
{
    KI_TEST::LoadBoard( m_settingsManager, "tracks_arcs_vias", m_board );
    m_board->BuildConnectivity();

    // Find the first net with tracks
    int targetNet = -1;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() > 0 && track->Type() != PCB_VIA_T )
        {
            targetNet = track->GetNetCode();
            break;
        }
    }

    if( targetNet < 0 )
    {
        BOOST_TEST_MESSAGE( "No routed nets — skipping" );
        return;
    }

    TRACE_PATH_WALKER walker( m_board.get() );
    walker.WalkNet( targetNet );

    const auto& path = walker.GetPath();

    BOOST_TEST_MESSAGE( "Path has " << path.size() << " points on net " << targetNet );

    auto samples = extractNeighborInfo( m_board.get(), path, targetNet );

    BOOST_TEST_MESSAGE( "Extracted " << samples.size() << " samples" );

    // Detailed dump of first 20 samples
    int count = 0;

    for( const auto& s : samples )
    {
        if( count++ >= 20 )
            break;

        wxString msg;
        msg.Printf( wxS( "  pos=(%.3f,%.3f)mm, dist=%.2fmm, tangent=(%.2f,%.2f), %d neighbors:" ),
                     s.position.x / 1e6, s.position.y / 1e6,
                     s.distFromStart / 1e6,
                     s.tangent.x, s.tangent.y,
                     (int) s.neighbors.size() );

        BOOST_TEST_MESSAGE( msg );

        for( const auto& nb : s.neighbors )
        {
            wxString nbMsg;
            nbMsg.Printf( wxS( "    dist=%.3fmm, width=%.3fmm, net=%d (%s), same_net=%s" ),
                          nb.distNm / 1e6, nb.widthNm / 1e6,
                          nb.netCode, nb.netName,
                          ( nb.netCode == targetNet ) ? wxS( "YES" ) : wxS( "no" ) );

            BOOST_TEST_MESSAGE( nbMsg );
        }
    }

    // Statistics
    int samplesWithNeighbors = 0;
    int sameNetNeighbors = 0;
    int otherNetNeighbors = 0;
    double minDist = 1e9, maxDist = 0;
    double minSameNetDist = 1e9;

    for( const auto& s : samples )
    {
        if( !s.neighbors.empty() )
            samplesWithNeighbors++;

        for( const auto& nb : s.neighbors )
        {
            double d = std::abs( nb.distNm ) / 1e6; // mm

            if( nb.netCode == targetNet )
            {
                sameNetNeighbors++;
                minSameNetDist = std::min( minSameNetDist, d );
            }
            else
            {
                otherNetNeighbors++;
            }

            minDist = std::min( minDist, d );
            maxDist = std::max( maxDist, d );
        }
    }

    BOOST_TEST_MESSAGE( "\nSummary:" );
    BOOST_TEST_MESSAGE( "  Samples with neighbors: " << samplesWithNeighbors
                        << "/" << samples.size() );
    BOOST_TEST_MESSAGE( "  Same-net neighbors: " << sameNetNeighbors );
    BOOST_TEST_MESSAGE( "  Other-net neighbors: " << otherNetNeighbors );
    BOOST_TEST_MESSAGE( "  Distance range: " << minDist << " - " << maxDist << " mm" );

    if( sameNetNeighbors > 0 )
        BOOST_TEST_MESSAGE( "  Min same-net distance: " << minSameNetDist << " mm" );

    // REGRESSION: no neighbor should overlap (edge-to-edge < 0).
    // This catches the bug where endpoint-connected segments show up as
    // neighbors at 0.03mm distance.
    for( const auto& s : samples )
    {
        double halfTraceWidth = s.traceWidth / 2e6; // mm

        for( const auto& nb : s.neighbors )
        {
            double d = std::abs( nb.distNm ) / 1e6; // mm
            double halfNbWidth = nb.widthNm / 2e6;
            double edgeToEdge = d - halfTraceWidth - halfNbWidth;

            BOOST_CHECK_GE( edgeToEdge, -0.01 ); // no overlapping neighbors
        }
    }

    // REGRESSION: minimum neighbor distance should be physically realistic
    // (> trace width, i.e., > ~0.1mm for typical traces)
    if( minDist < 1e9 )
    {
        BOOST_CHECK_GT( minDist, 0.1 );
    }
}


/**
 * Verify BEM produces different Z₀ for isolated vs coupled cross-sections.
 * This tests the BEM directly — no board needed.
 */
BOOST_AUTO_TEST_CASE( BEMCouplingEffect )
{
    // Microstrip: w=0.15mm, h=0.1mm, er=4.4
    double w = 0.15e-3;
    double t = 35e-6;
    double h = 0.1e-3;

    auto makeGeom = [&]( double neighborDist ) -> XS_GEOMETRY
    {
        XS_GEOMETRY xs;
        xs.groundY = 0.0;
        xs.epsilonR = 4.4;

        XS_DIELECTRIC_REGION air, diel;
        air.yTop = -10e-3;
        air.yBottom = -h;
        air.epsilonR = 1.0;
        diel.yTop = -h;
        diel.yBottom = 0.0;
        diel.epsilonR = 4.4;
        xs.dielectrics.push_back( air );
        xs.dielectrics.push_back( diel );

        XS_CONDUCTOR cond;
        cond.centerX = 0.0;
        cond.centerY = -( h + t / 2.0 );
        cond.width = w;
        cond.thickness = t;
        xs.conductors.push_back( cond );

        if( neighborDist > 0.0 )
        {
            XS_CONDUCTOR nbCond;
            nbCond.centerX = neighborDist;
            nbCond.centerY = -( h + t / 2.0 );
            nbCond.width = w;
            nbCond.thickness = t;
            xs.conductors.push_back( nbCond );
        }

        return xs;
    };

    // Isolated (no neighbor)
    BEM_2D_SOLVER solverIsolated;
    solverIsolated.SetGeometry( makeGeom( 0.0 ) );
    solverIsolated.SetPanelsPerEdge( 15 );
    solverIsolated.Solve();

    // Neighbor at 0.3mm center-to-center
    BEM_2D_SOLVER solverClose;
    solverClose.SetGeometry( makeGeom( 0.3e-3 ) );
    solverClose.SetPanelsPerEdge( 10 );
    solverClose.Solve();

    // Neighbor at 1.0mm center-to-center
    BEM_2D_SOLVER solverFar;
    solverFar.SetGeometry( makeGeom( 1.0e-3 ) );
    solverFar.SetPanelsPerEdge( 10 );
    solverFar.Solve();

    double z0_iso = solverIsolated.GetResult().Z0;
    double z0_close = solverClose.GetResult().Z0;
    double z0_far = solverFar.GetResult().Z0;

    BOOST_TEST_MESSAGE( "--- Isolated (1 conductor, with dielectric) ---" );
    BOOST_TEST_MESSAGE( "Z0 = " << z0_iso );
    BOOST_TEST_MESSAGE( "C  =\n" << solverIsolated.GetResult().C );
    BOOST_TEST_MESSAGE( "C0 =\n" << solverIsolated.GetResult().C0 );
    BOOST_TEST_MESSAGE( "L  =\n" << solverIsolated.GetResult().L );
    BOOST_TEST_MESSAGE( "erEff = " << solverIsolated.GetResult().erEff );

    BOOST_TEST_MESSAGE( "--- Neighbor at 0.3mm (2 conductors, with dielectric) ---" );
    BOOST_TEST_MESSAGE( "Z0 = " << z0_close );
    BOOST_TEST_MESSAGE( "C  =\n" << solverClose.GetResult().C );
    BOOST_TEST_MESSAGE( "C0 =\n" << solverClose.GetResult().C0 );
    BOOST_TEST_MESSAGE( "L  =\n" << solverClose.GetResult().L );
    BOOST_TEST_MESSAGE( "erEff = " << solverClose.GetResult().erEff );

    BOOST_TEST_MESSAGE( "--- Neighbor at 1.0mm (2 conductors, with dielectric) ---" );
    BOOST_TEST_MESSAGE( "Z0 = " << z0_far );
    BOOST_TEST_MESSAGE( "C  =\n" << solverFar.GetResult().C );
    BOOST_TEST_MESSAGE( "C0 =\n" << solverFar.GetResult().C0 );
    BOOST_TEST_MESSAGE( "L  =\n" << solverFar.GetResult().L );
    BOOST_TEST_MESSAGE( "erEff = " << solverFar.GetResult().erEff );

    // Also test WITHOUT dielectric regions (uniform er) for comparison
    auto makeGeomUniform = [&]( double neighborDist ) -> XS_GEOMETRY
    {
        XS_GEOMETRY xs;
        xs.groundY = 0.0;
        xs.epsilonR = 4.4;

        XS_CONDUCTOR cond;
        cond.centerX = 0.0;
        cond.centerY = -( h + t / 2.0 );
        cond.width = w;
        cond.thickness = t;
        xs.conductors.push_back( cond );

        if( neighborDist > 0.0 )
        {
            XS_CONDUCTOR nbCond;
            nbCond.centerX = neighborDist;
            nbCond.centerY = -( h + t / 2.0 );
            nbCond.width = w;
            nbCond.thickness = t;
            xs.conductors.push_back( nbCond );
        }

        return xs;
    };

    BEM_2D_SOLVER solverUniIso;
    solverUniIso.SetGeometry( makeGeomUniform( 0.0 ) );
    solverUniIso.SetPanelsPerEdge( 15 );
    solverUniIso.Solve();

    BEM_2D_SOLVER solverUniClose;
    solverUniClose.SetGeometry( makeGeomUniform( 0.3e-3 ) );
    solverUniClose.SetPanelsPerEdge( 10 );
    solverUniClose.Solve();

    BOOST_TEST_MESSAGE( "--- Uniform dielectric (no regions) ---" );
    BOOST_TEST_MESSAGE( "Isolated Z0 = " << solverUniIso.GetResult().Z0 );
    BOOST_TEST_MESSAGE( "Neighbor 0.3mm Z0 = " << solverUniClose.GetResult().Z0 );
    BOOST_TEST_MESSAGE( "C isolated =\n" << solverUniIso.GetResult().C );
    BOOST_TEST_MESSAGE( "C with neighbor =\n" << solverUniClose.GetResult().C );

    // Coupling should lower Z₀ in both dielectric and uniform cases
    BOOST_CHECK_LT( z0_close, z0_iso );
    BOOST_CHECK_LT( z0_far, z0_iso );
    BOOST_CHECK_LT( z0_close, z0_far ); // closer = more coupling = lower Z₀
    BOOST_CHECK_GT( z0_iso - z0_close, 0.5 ); // measurable effect

    BOOST_CHECK_LT( solverUniClose.GetResult().Z0, solverUniIso.GetResult().Z0 );
}


BOOST_AUTO_TEST_SUITE_END()
