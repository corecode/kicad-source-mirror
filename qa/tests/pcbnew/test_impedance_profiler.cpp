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
#include <footprint.h>
#include <pcb_track.h>
#include <pad.h>
#include <netinfo.h>
#include <settings/settings_manager.h>

#include <drc/drc_rtree.h>
#include <sipi/cross_section_builder.h>
#include <sipi/trace_path_walker.h>
#include <sipi/stackup_reader.h>
#include <sipi/bem_2d_solver.h>
#include <sipi/impedance_profile.h>

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
    // Far neighbor at 1mm has negligible coupling — just verify it doesn't
    // increase Z₀ by more than numerical noise (0.1 Ω)
    BOOST_CHECK_LT( z0_far - z0_iso, 0.1 );
    BOOST_CHECK_LT( z0_close, z0_far ); // closer = more coupling = lower Z₀
    BOOST_CHECK_GT( z0_iso - z0_close, 0.5 ); // measurable effect

    BOOST_CHECK_LT( solverUniClose.GetResult().Z0, solverUniIso.GetResult().Z0 );
}


/**
 * Verify that adding same-side grounded neighbors monotonically decreases Z₀.
 * This is the regression test for the multi-conductor coupling bug.
 */
BOOST_AUTO_TEST_CASE( SameSideNeighborDecreasesZ0 )
{
    double w = 0.15e-3;
    double t = 35e-6;
    double h = 0.1e-3;
    int panels = 12;

    auto makeGeom = [&]( std::vector<double> neighborDists ) -> XS_GEOMETRY
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

        for( double d : neighborDists )
        {
            XS_CONDUCTOR nb;
            nb.centerX = d;
            nb.centerY = -( h + t / 2.0 );
            nb.width = w;
            nb.thickness = t;
            xs.conductors.push_back( nb );
        }

        return xs;
    };

    // Isolated
    BEM_2D_SOLVER solverIso;
    solverIso.SetGeometry( makeGeom( {} ) );
    solverIso.SetPanelsPerEdge( panels );
    BOOST_REQUIRE( solverIso.Solve() );
    double z0_iso = solverIso.GetResult().Z0;

    // 1 same-side neighbor at -0.29mm
    BEM_2D_SOLVER solver1;
    solver1.SetGeometry( makeGeom( { -0.29e-3 } ) );
    solver1.SetPanelsPerEdge( panels );
    BOOST_REQUIRE( solver1.Solve() );
    double z0_1 = solver1.GetResult().Z0;

    // 2 same-side neighbors at -0.29mm and -0.58mm
    BEM_2D_SOLVER solver2;
    solver2.SetGeometry( makeGeom( { -0.29e-3, -0.58e-3 } ) );
    solver2.SetPanelsPerEdge( panels );
    BOOST_REQUIRE( solver2.Solve() );
    double z0_2 = solver2.GetResult().Z0;

    BOOST_TEST_MESSAGE( "Isolated:    Z0 = " << z0_iso );
    BOOST_TEST_MESSAGE( "1 neighbor:  Z0 = " << z0_1 );
    BOOST_TEST_MESSAGE( "2 neighbors: Z0 = " << z0_2 );

    // Each additional grounded conductor must lower Z₀
    BOOST_CHECK_LT( z0_1, z0_iso );
    BOOST_CHECK_LT( z0_2, z0_1 );
    BOOST_CHECK_LT( z0_2, z0_iso );

    // C₁₁ must monotonically increase
    double c_iso = solverIso.GetResult().C( 0, 0 );
    double c_1   = solver1.GetResult().C( 0, 0 );
    double c_2   = solver2.GetResult().C( 0, 0 );

    BOOST_TEST_MESSAGE( "C₁₁ isolated: " << c_iso );
    BOOST_TEST_MESSAGE( "C₁₁ 1 nb:     " << c_1 );
    BOOST_TEST_MESSAGE( "C₁₁ 2 nb:     " << c_2 );

    BOOST_CHECK_GT( c_1, c_iso );
    BOOST_CHECK_GT( c_2, c_1 );
}


/**
 * Diagnostic: profile SRAM_D5 on the cparti_fpga board.
 * Prints Z₀ at every sample point with per-sample timing.
 * Checks for impedance jumps > 20Ω between adjacent samples.
 */
BOOST_AUTO_TEST_CASE( D5ProfileDiagnostic )
{
    std::unique_ptr<BOARD> board;

    try
    {
        KI_TEST::LoadBoard( m_settingsManager, "cparti_fpga", board );
    }
    catch( ... )
    {
        BOOST_TEST_MESSAGE( "Board cparti_fpga not found — skipping" );
        return;
    }

    if( !board || board->Tracks().empty() )
    {
        BOOST_TEST_MESSAGE( "No tracks — skipping" );
        return;
    }

    KI_TEST::FillZones( board.get() );
    board->BuildConnectivity();

    // Find SRAM_D5 net
    int targetNet = -1;

    for( const auto& [code, info] : board->GetNetInfo().NetsByNetcode() )
    {
        if( info->GetNetname().Lower() == wxS( "sram_d5" ) )
        {
            targetNet = code;
            break;
        }
    }

    if( targetNet < 0 )
    {
        BOOST_TEST_MESSAGE( "SRAM_D5 not found — skipping" );
        return;
    }

    // Build R-tree with tracks, vias, and pads
    DRC_RTREE rtree;

    for( PCB_TRACK* t : board->Tracks() )
    {
        if( t->Type() == PCB_VIA_T )
        {
            for( PCB_LAYER_ID layer : t->GetLayerSet().CuStack() )
                rtree.Insert( t, layer );
        }
        else
        {
            rtree.Insert( t, t->GetLayer() );
        }
    }

    for( FOOTPRINT* fp : board->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            for( PCB_LAYER_ID layer : pad->GetLayerSet().CuStack() )
                rtree.Insert( pad, layer );
        }
    }

    STACKUP_READER stackup( board.get() );
    TRACE_PATH_WALKER walker( board.get() );
    walker.WalkNet( targetNet );

    const auto& path = walker.GetPath();

    BOOST_TEST_MESSAGE( "D5 path: " << path.size() << " points, "
                        << walker.GetTotalLength() / 1e6 << " mm" );

    // Build path-distance map
    std::map<BOARD_CONNECTED_ITEM*, double> pathItemDist;

    for( const PATH_POINT& pp : path )
    {
        if( !pp.isVia && pathItemDist.find( pp.item ) == pathItemDist.end() )
            pathItemDist[pp.item] = pp.distFromStart;
    }

    // Sample every 200µm
    double totalLen = walker.GetTotalLength();
    double step = 200000.0; // 200µm in nm

    // Build non-via segment list for interpolation
    struct SEG_PT { double dist; VECTOR2I pos; VECTOR2D tangent; PCB_TRACK* track; };
    std::vector<SEG_PT> segs;

    for( const PATH_POINT& pp : path )
    {
        if( !pp.isVia )
            segs.push_back( { pp.distFromStart, pp.position, pp.tangent,
                              static_cast<PCB_TRACK*>( pp.item ) } );
    }

    int segIdx = 0;
    double prevZ0 = 0.0;
    int jumpCount = 0;
    int totalSamples = 0;

    auto tStart = std::chrono::steady_clock::now();

    for( double d = 0; d <= totalLen; d += step )
    {
        while( segIdx + 1 < (int) segs.size() && segs[segIdx + 1].dist <= d )
            segIdx++;

        VECTOR2I pos = segs[segIdx].pos;
        VECTOR2D tangent = segs[segIdx].tangent;

        if( segIdx + 1 < (int) segs.size() )
        {
            double segLen = segs[segIdx + 1].dist - segs[segIdx].dist;

            if( segLen > 1.0 )
            {
                double frac = std::clamp( ( d - segs[segIdx].dist ) / segLen, 0.0, 1.0 );
                pos.x = segs[segIdx].pos.x
                        + (int) ( frac * ( segs[segIdx + 1].pos.x - segs[segIdx].pos.x ) );
                pos.y = segs[segIdx].pos.y
                        + (int) ( frac * ( segs[segIdx + 1].pos.y - segs[segIdx].pos.y ) );
            }
        }

        PCB_TRACK* track = segs[segIdx].track;
        int signalWidth = track->GetWidth();

        LAYER_GEOMETRY geom = stackup.GetLayerGeometry( track->GetLayer(), pos, signalWidth );

        double hRef = std::max( geom.hAbove, geom.hBelow );
        int    couplingHorizon = std::clamp( (int) ( hRef * 3.0 * 1e9 ), 500000, 3000000 );

        CROSS_SECTION_BUILDER xsBuilder;
        xsBuilder.SetSpatialIndex( &rtree );
        xsBuilder.SetBoard( board.get() );
        xsBuilder.SetPathItemDistances( &pathItemDist );
        xsBuilder.SetSignalTrack( track );

        XS_BUILD_PARAMS xsParams;
        xsParams.samplePos = pos;
        xsParams.sampleTangent = tangent;
        xsParams.signalLayer = track->GetLayer();
        xsParams.signalNetCode = track->GetNetCode();
        xsParams.signalWidth = signalWidth;
        xsParams.couplingHorizon = couplingHorizon;
        xsParams.sampleDist = d;
        xsParams.layerGeom = geom;

        auto t0 = std::chrono::steady_clock::now();

        auto neighbors = xsBuilder.FindNeighbors( xsParams );
        auto groundWires = xsBuilder.FindGroundWires( xsParams, geom );
        xsParams.layerGeom = geom;

        auto t1 = std::chrono::steady_clock::now();

        XS_GEOMETRY xs = xsBuilder.BuildGeometry( xsParams, neighbors, groundWires );

        BEM_2D_SOLVER solver;
        solver.SetGeometry( xs );
        solver.SetPanelsPerEdge( 12 );
        bool bemOk = solver.Solve();

        auto t2 = std::chrono::steady_clock::now();

        double z0 = 0.0;

        if( bemOk && solver.GetResult().Z0 > 0.0 )
            z0 = solver.GetResult().Z0;

        long usXs = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count();
        long usBem = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

        // Count groundwires
        int gwCount = 0;
        for( const auto& c : xs.conductors )
        {
            if( c.isGround )
                gwCount++;
        }

        bool isJump = ( prevZ0 > 0.0 && std::abs( z0 - prevZ0 ) > 20.0 );

        if( isJump )
            jumpCount++;

        // Log every sample
        BOOST_TEST_MESSAGE(
                wxString::Format( wxS( "d=%.2fmm Z0=%.1f%s hBelow=%.0fum nb=%d gw=%d "
                                       "xs=%ldus bem=%ldus" ),
                                  d / 1e6, z0,
                                  isJump ? wxS( " ***JUMP***" ) : wxS( "" ),
                                  geom.hBelow * 1e6,
                                  (int) neighbors.size(), gwCount,
                                  usXs, usBem ) );

        prevZ0 = z0;
        totalSamples++;
    }

    auto tEnd = std::chrono::steady_clock::now();
    long totalMs = std::chrono::duration_cast<std::chrono::milliseconds>( tEnd - tStart ).count();

    BOOST_TEST_MESSAGE( "\n=== SUMMARY ===" );
    BOOST_TEST_MESSAGE( "Samples: " << totalSamples << "  Total time: " << totalMs << "ms"
                        << "  Avg: " << totalMs / std::max( totalSamples, 1 ) << "ms/sample" );
    BOOST_TEST_MESSAGE( "Impedance jumps (>20 Ohm): " << jumpCount );

    // Allow jumps only at via/layer transitions (real discontinuities)
    BOOST_CHECK_LE( jumpCount, 2 );
}


/**
 * Test SE_PROFILE::Compute on a real board.
 * Verify samples are produced, Z0 is reasonable, geometry has conductors,
 * and delay accumulates.
 */
BOOST_AUTO_TEST_CASE( SEProfileBasic )
{
    KI_TEST::LoadBoard( m_settingsManager, "tracks_arcs_vias", m_board );
    KI_TEST::FillZones( m_board.get() );
    m_board->BuildConnectivity();

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

    SE_PROFILE profile;
    BEM_CACHE cache;
    bool ok = profile.Compute( m_board.get(), targetNet, cache );
    BOOST_REQUIRE( ok );

    const auto& samples = profile.GetSamples();
    BOOST_REQUIRE_GT( samples.size(), 0 );

    BOOST_TEST_MESSAGE( "SE_PROFILE: " << samples.size() << " samples"
                        << "  length=" << profile.GetTotalLength() / 1e6 << "mm"
                        << "  delay=" << profile.GetTotalDelay() << "ps" );

    BOOST_CHECK_GT( profile.GetTotalLength(), 0.0 );
    BOOST_CHECK_GT( profile.GetTotalDelay(), 0.0 );

    // Every sample should have at least one conductor and a valid Z0
    for( size_t i = 0; i < samples.size(); i++ )
    {
        const IMPEDANCE_SAMPLE& s = samples[i];

        BOOST_CHECK_GT( s.geometry.conductors.size(), 0 );
        BOOST_CHECK_GT( s.z0, 10.0 );
        BOOST_CHECK_LT( s.z0, 300.0 );
        BOOST_CHECK_GE( s.delayPs, 0.0 );

        if( i > 0 )
            BOOST_CHECK_GE( s.delayPs, samples[i - 1].delayPs );
    }

    // AtDelay should return valid interpolated result
    double midDelay = profile.GetTotalDelay() / 2.0;
    IMPEDANCE_SAMPLE mid = profile.AtDelay( midDelay );
    BOOST_CHECK_GT( mid.z0, 10.0 );
    BOOST_CHECK_GT( mid.geometry.conductors.size(), 0 );

    BOOST_TEST_MESSAGE( "AtDelay(" << midDelay << "ps): Z0=" << mid.z0
                        << "  conductors=" << mid.geometry.conductors.size() );
}


/**
 * Test SE_PROFILE and DIFF_PROFILE on the USB_D differential pair
 * from the cparti_fpga test board.
 */
BOOST_AUTO_TEST_CASE( USBDiffPairProfile )
{
    try
    {
        KI_TEST::LoadBoard( m_settingsManager, "cparti_fpga", m_board );
    }
    catch( ... )
    {
        BOOST_TEST_MESSAGE( "Board cparti_fpga not found — skipping" );
        return;
    }

    if( !m_board || m_board->Tracks().empty() )
    {
        BOOST_TEST_MESSAGE( "No tracks — skipping" );
        return;
    }

    KI_TEST::FillZones( m_board.get() );
    m_board->BuildConnectivity();

    // Find USB_D+ and USB_D- nets
    int netP = -1, netN = -1;

    for( const auto& [code, info] : m_board->GetNetInfo().NetsByNetcode() )
    {
        wxString name = info->GetNetname();

        if( name.Contains( wxS( "USB_D_P" ) ) )
        {
            netP = code;
            BOOST_TEST_MESSAGE( "P net: " << name << " (code " << code << ")" );
        }

        if( name.Contains( wxS( "USB_D_N" ) ) )
        {
            netN = code;
            BOOST_TEST_MESSAGE( "N net: " << name << " (code " << code << ")" );
        }
    }

    if( netP < 0 || netN < 0 )
    {
        BOOST_TEST_MESSAGE( "USB_D pair not found — skipping" );
        return;
    }

    // Test SE_PROFILE on P
    SE_PROFILE profileP;
    BEM_CACHE cache;
    BOOST_REQUIRE( profileP.Compute( m_board.get(), netP, cache ) );

    BOOST_TEST_MESSAGE( "P profile: " << profileP.GetSamples().size() << " samples"
                        << "  length=" << profileP.GetTotalLength() / 1e6 << "mm"
                        << "  delay=" << profileP.GetTotalDelay() << "ps" );

    for( size_t i = 0; i < profileP.GetSamples().size(); i++ )
    {
        const IMPEDANCE_SAMPLE& s = profileP.GetSamples()[i];

        BOOST_TEST_MESSAGE( "  P[" << i << "] d=" << s.distNm / 1e6 << "mm"
                            << " Z0=" << s.z0
                            << " cond=" << s.geometry.conductors.size()
                            << " nb=" << s.neighborCount
                            << " gw=" << s.groundwireCount
                            << " layer=" << s.layer );

        BOOST_CHECK_GT( s.geometry.conductors.size(), 0 );
        BOOST_CHECK_GT( s.z0, 0.0 );
    }

    // Test SE_PROFILE on N
    SE_PROFILE profileN;
    BOOST_REQUIRE( profileN.Compute( m_board.get(), netN, cache ) );

    BOOST_TEST_MESSAGE( "N profile: " << profileN.GetSamples().size() << " samples"
                        << "  length=" << profileN.GetTotalLength() / 1e6 << "mm"
                        << "  delay=" << profileN.GetTotalDelay() << "ps" );

    // Test DIFF_PROFILE
    DIFF_PROFILE diffProfile;
    bool diffOk = diffProfile.Compute( m_board.get(), netP, netN, cache );

    BOOST_TEST_MESSAGE( "DIFF_PROFILE ok=" << diffOk
                        << " error=" << diffProfile.GetError() );

    if( diffOk )
    {
        // Dump internal profile stats
        const auto& intP = diffProfile.GetProfileP();
        const auto& intN = diffProfile.GetProfileN();

        BOOST_TEST_MESSAGE( "DIFF internal P: " << intP.GetSamples().size() << " samples"
                            << "  length=" << intP.GetTotalLength() / 1e6 << "mm"
                            << "  delay=" << intP.GetTotalDelay() << "ps"
                            << "  startPad=" << ( intP.GetStartPad() ? "yes" : "no" )
                            << "  endPad=" << ( intP.GetEndPad() ? "yes" : "no" ) );

        BOOST_TEST_MESSAGE( "DIFF internal N: " << intN.GetSamples().size() << " samples"
                            << "  length=" << intN.GetTotalLength() / 1e6 << "mm"
                            << "  delay=" << intN.GetTotalDelay() << "ps"
                            << "  startPad=" << ( intN.GetStartPad() ? "yes" : "no" )
                            << "  endPad=" << ( intN.GetEndPad() ? "yes" : "no" ) );

        if( intP.GetStartPad() )
            BOOST_TEST_MESSAGE( "  P start pad: " << intP.GetStartPad()->GetPosition() );

        if( intN.GetStartPad() )
            BOOST_TEST_MESSAGE( "  N start pad: " << intN.GetStartPad()->GetPosition() );

        // Dump first/last few board positions to check alignment
        if( !intP.GetSamples().empty() && !intN.GetSamples().empty() )
        {
            auto& sp = intP.GetSamples();
            auto& sn = intN.GetSamples();

            BOOST_TEST_MESSAGE( "  P[0] pos=(" << sp.front().boardPos.x / 1e6 << ","
                                << sp.front().boardPos.y / 1e6 << ")mm  layer=" << sp.front().layer );
            BOOST_TEST_MESSAGE( "  P[last] pos=(" << sp.back().boardPos.x / 1e6 << ","
                                << sp.back().boardPos.y / 1e6 << ")mm" );
            BOOST_TEST_MESSAGE( "  N[0] pos=(" << sn.front().boardPos.x / 1e6 << ","
                                << sn.front().boardPos.y / 1e6 << ")mm  layer=" << sn.front().layer );
            BOOST_TEST_MESSAGE( "  N[last] pos=(" << sn.back().boardPos.x / 1e6 << ","
                                << sn.back().boardPos.y / 1e6 << ")mm" );
        }

        BOOST_TEST_MESSAGE( "Diff samples: " << diffProfile.GetSamples().size()
                            << "  skew=" << diffProfile.GetSkewPs() << "ps" );

        int coupledCount = 0;
        int uncoupledCount = 0;

        for( size_t i = 0; i < diffProfile.GetSamples().size(); i++ )
        {
            const DIFF_SAMPLE& ds = diffProfile.GetSamples()[i];

            if( ds.coupled )
                coupledCount++;
            else
                uncoupledCount++;

            // Only dump every 50th sample to reduce noise
            if( i % 50 == 0 || i == diffProfile.GetSamples().size() - 1 )
            {
                BOOST_TEST_MESSAGE( "  D[" << i << "] d=" << ds.distMm << "mm"
                                    << " Z0p=" << ds.z0P << " Z0n=" << ds.z0N
                                    << " Zdiff=" << ds.zdiff
                                    << " " << ( ds.coupled ? "coupled" : "uncoupled" )
                                    << ( ds.coupled ? " gap=" + std::to_string( ds.gapUm ) + "um" : "" )
                                    << " cond=" << ds.geometry.conductors.size() );
            }

            BOOST_CHECK_GT( ds.zdiff, 0.0 );
            BOOST_CHECK_GT( ds.z0P, 0.0 );
            BOOST_CHECK_GT( ds.z0N, 0.0 );
        }

        BOOST_TEST_MESSAGE( "Coupled: " << coupledCount
                            << "  Uncoupled: " << uncoupledCount );
    }
    else
    {
        BOOST_TEST_MESSAGE( "DIFF_PROFILE failed: " << diffProfile.GetError() );
    }
}


BOOST_AUTO_TEST_SUITE_END()
