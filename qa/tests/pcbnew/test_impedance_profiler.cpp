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

#include <drc/drc_rtree.h>
#include <sipi/cross_section_builder.h>
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


BOOST_FIXTURE_TEST_SUITE( ImpedanceProfiler, PROFILER_TEST_FIXTURE )


/**
 * Test neighbor detection using CROSS_SECTION_BUILDER on a real board.
 * Loads tracks_arcs_vias test board, walks the first net, runs the builder
 * at each path point, and checks for unrealistic distances.
 */
BOOST_AUTO_TEST_CASE( NeighborDetectionDiag )
{
    KI_TEST::LoadBoard( m_settingsManager, "tracks_arcs_vias", m_board );
    KI_TEST::FillZones( m_board.get() );
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

    // Build R-tree and path-distance map
    DRC_RTREE rtree;

    for( PCB_TRACK* t : m_board->Tracks() )
    {
        if( t->Type() != PCB_VIA_T )
            rtree.Insert( t, t->GetLayer() );
    }

    std::map<BOARD_CONNECTED_ITEM*, double> pathItemDist;

    for( const PATH_POINT& pp : path )
    {
        if( !pp.isVia && pathItemDist.find( pp.item ) == pathItemDist.end() )
            pathItemDist[pp.item] = pp.distFromStart;
    }

    STACKUP_READER stackup( m_board.get() );

    // Run builder at each non-via path point
    double minDist = 1e9;
    int    tinyDistCount = 0;
    int    samplesWithNb = 0;
    int    totalSamples = 0;

    for( const PATH_POINT& pt : path )
    {
        if( pt.isVia )
            continue;

        totalSamples++;
        PCB_TRACK* track = static_cast<PCB_TRACK*>( pt.item );

        LAYER_GEOMETRY geom = stackup.GetLayerGeometry( track->GetLayer(),
                                                        pt.position,
                                                        track->GetWidth() );

        double hRef = std::max( geom.hAbove, geom.hBelow );
        int    couplingHorizon = std::max( (int) ( hRef * 3.0 * 1e9 ), 500000 );

        CROSS_SECTION_BUILDER builder;
        builder.SetSpatialIndex( &rtree );
        builder.SetBoard( m_board.get() );
        builder.SetPathItemDistances( &pathItemDist );
        builder.SetSignalTrack( track );

        XS_BUILD_PARAMS params;
        params.samplePos = pt.position;
        params.sampleTangent = pt.tangent;
        params.signalLayer = track->GetLayer();
        params.signalNetCode = track->GetNetCode();
        params.signalWidth = track->GetWidth();
        params.couplingHorizon = couplingHorizon;
        params.sampleDist = pt.distFromStart;
        params.layerGeom = geom;

        auto neighbors = builder.FindNeighbors( params );

        if( !neighbors.empty() )
            samplesWithNb++;

        for( const XS_NEIGHBOR& nb : neighbors )
        {
            double edgeToEdge = ( std::abs( nb.distNm ) - track->GetWidth() / 2.0
                                  - nb.widthNm / 2.0 ) / 1e6; // mm
            double dist = std::abs( nb.distNm ) / 1e6;

            minDist = std::min( minDist, dist );

            if( edgeToEdge < 0.01 ) // less than 10µm edge-to-edge
            {
                tinyDistCount++;
                BOOST_TEST_MESSAGE( "TINY: pos=(" << pt.position.x / 1e6 << ","
                                    << pt.position.y / 1e6 << ")mm dist="
                                    << pt.distFromStart / 1e6 << "mm nb_dist="
                                    << nb.distNm / 1e6 << "mm nb_w="
                                    << nb.widthNm / 1e6 << "mm e2e="
                                    << edgeToEdge << "mm" );
            }
        }
    }

    BOOST_TEST_MESSAGE( "\nSummary:" );
    BOOST_TEST_MESSAGE( "  Total samples: " << totalSamples );
    BOOST_TEST_MESSAGE( "  With neighbors: " << samplesWithNb );
    BOOST_TEST_MESSAGE( "  Min distance: " << minDist << " mm" );
    BOOST_TEST_MESSAGE( "  Tiny distance count: " << tinyDistCount );

    // REGRESSION: no tiny edge-to-edge distances
    BOOST_CHECK_EQUAL( tinyDistCount, 0 );
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

    // Use consistent panel count for all cases so that convergence differences
    // don't mask or invert the coupling effect.
    int panels = 12;

    // Isolated (no neighbor)
    BEM_2D_SOLVER solverIsolated;
    solverIsolated.SetGeometry( makeGeom( 0.0 ) );
    solverIsolated.SetPanelsPerEdge( panels );
    solverIsolated.Solve();

    // Neighbor at 0.3mm center-to-center
    BEM_2D_SOLVER solverClose;
    solverClose.SetGeometry( makeGeom( 0.3e-3 ) );
    solverClose.SetPanelsPerEdge( panels );
    solverClose.Solve();

    // Neighbor at 1.0mm center-to-center
    BEM_2D_SOLVER solverFar;
    solverFar.SetGeometry( makeGeom( 1.0e-3 ) );
    solverFar.SetPanelsPerEdge( panels );
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
    solverUniIso.SetPanelsPerEdge( panels );
    solverUniIso.Solve();

    BEM_2D_SOLVER solverUniClose;
    solverUniClose.SetGeometry( makeGeomUniform( 0.3e-3 ) );
    solverUniClose.SetPanelsPerEdge( panels );
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
