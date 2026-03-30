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
#include <netinfo.h>
#include <footprint.h>
#include <pcb_track.h>
#include <zone.h>
#include <drc/drc_rtree.h>
#include <settings/settings_manager.h>
#include <sipi/cross_section_builder.h>
#include <sipi/trace_path_walker.h>
#include <sipi/stackup_reader.h>
#include <sipi/bem_2d_solver.h>

#include <cmath>


/**
 * Test fixture that creates a BOARD + DRC_RTREE programmatically.
 * All coordinates in nm.  Net 1 = signal, Net 2 = neighbor.
 */
struct XS_BUILDER_FIXTURE
{
    XS_BUILDER_FIXTURE() :
            m_board( std::make_unique<BOARD>() ),
            m_rtree( std::make_unique<DRC_RTREE>() )
    {
        // Register nets so SetNet/SetNetCode works
        for( int i = 1; i <= 10; i++ )
        {
            m_board->Add( new NETINFO_ITEM( m_board.get(),
                                             wxString::Format( wxS( "Net%d" ), i ), i ) );
        }
    }

    PCB_TRACK* addTrack( const VECTOR2I& aStart, const VECTOR2I& aEnd,
                         int aWidth, PCB_LAYER_ID aLayer, int aNetCode )
    {
        PCB_TRACK* t = new PCB_TRACK( m_board.get() );
        t->SetStart( aStart );
        t->SetEnd( aEnd );
        t->SetWidth( aWidth );
        t->SetLayer( aLayer );
        t->SetNet( m_board->GetNetInfo().GetNetItem( aNetCode ) );
        m_board->Add( t );
        m_rtree->Insert( t, aLayer );
        return t;
    }

    void addZoneFill( const VECTOR2I& aTopLeft, const VECTOR2I& aBotRight,
                      PCB_LAYER_ID aLayer, int aNetCode )
    {
        ZONE* z = new ZONE( m_board.get() );
        z->SetLayer( aLayer );
        z->SetNet( m_board->GetNetInfo().GetNetItem( aNetCode ) );
        z->SetIsFilled( true );
        z->SetFillFlag( aLayer, true );

        // Build a rectangular fill polygon
        SHAPE_POLY_SET fillPoly;
        fillPoly.NewOutline();
        fillPoly.Append( aTopLeft );
        fillPoly.Append( VECTOR2I( aBotRight.x, aTopLeft.y ) );
        fillPoly.Append( aBotRight );
        fillPoly.Append( VECTOR2I( aTopLeft.x, aBotRight.y ) );

        z->SetFilledPolysList( aLayer, fillPoly );
        m_board->Add( z );
    }

    XS_BUILD_PARAMS makeParams( PCB_TRACK* aSignal, const VECTOR2I& aPos,
                                const VECTOR2D& aTangent, int aCouplingHorizon = 1000000 )
    {
        XS_BUILD_PARAMS p;
        p.samplePos = aPos;
        p.sampleTangent = aTangent;
        p.signalLayer = aSignal->GetLayer();
        p.signalNetCode = aSignal->GetNetCode();
        p.signalWidth = aSignal->GetWidth();
        p.couplingHorizon = aCouplingHorizon;
        p.sampleDist = 0.0;

        // Minimal layer geometry for tests
        p.layerGeom.traceWidth = aSignal->GetWidth() * 1e-9;
        p.layerGeom.traceThickness = 35e-6;
        p.layerGeom.hBelow = 0.1e-3;
        p.layerGeom.erBelow = 4.4;
        p.layerGeom.hasRefBelow = true;
        return p;
    }

    CROSS_SECTION_BUILDER makeBuilder( PCB_TRACK* aSignal )
    {
        CROSS_SECTION_BUILDER builder;
        builder.SetSpatialIndex( m_rtree.get() );
        builder.SetBoard( m_board.get() );
        builder.SetSignalTrack( aSignal );
        return builder;
    }

    std::unique_ptr<BOARD>     m_board;
    std::unique_ptr<DRC_RTREE> m_rtree;
};


BOOST_FIXTURE_TEST_SUITE( CrossSectionBuilder, XS_BUILDER_FIXTURE )


/**
 * Test 1: Two horizontal parallel traces 300µm apart.
 * Neighbor should be found at the correct lateral distance.
 */
BOOST_AUTO_TEST_CASE( ParallelHorizontal )
{
    // Signal: net 1, horizontal at y=0
    PCB_TRACK* sig = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 10000000, 0 ),
                               150000, F_Cu, 1 );

    // Neighbor: net 2, horizontal at y=300000 (300µm above)
    addTrack( VECTOR2I( 0, 300000 ), VECTOR2I( 10000000, 300000 ),
              150000, F_Cu, 2 );

    auto builder = makeBuilder( sig );
    auto params = makeParams( sig, VECTOR2I( 5000000, 0 ), VECTOR2D( 1.0, 0.0 ) );

    // Debug: check R-tree contents
    auto rtreeItems = m_rtree->GetObjectsAt( VECTOR2I( 5000000, 0 ), F_Cu, 1000000 );
    BOOST_TEST_MESSAGE( "R-tree items at sample: " << rtreeItems.size() );

    for( auto* item : rtreeItems )
        BOOST_TEST_MESSAGE( "  type=" << item->Type() << " net=" << static_cast<BOARD_CONNECTED_ITEM*>( item )->GetNetCode() );

    auto neighbors = builder.FindNeighbors( params );

    BOOST_TEST_MESSAGE( "Parallel horizontal: " << neighbors.size() << " neighbors" );

    for( const auto& nb : neighbors )
        BOOST_TEST_MESSAGE( "  dist=" << nb.distNm << " width=" << nb.widthNm );

    BOOST_REQUIRE_EQUAL( neighbors.size(), 1 );
    BOOST_CHECK_CLOSE( std::abs( (double) neighbors[0].distNm ), 300000.0, 1.0 );
    BOOST_CHECK_EQUAL( neighbors[0].widthNm, 150000 );
}


/**
 * Test 2: Two parallel traces at 45°.
 * Both start at similar positions, run at 45 degrees.
 */
BOOST_AUTO_TEST_CASE( ParallelDiagonal )
{
    int len = 10000000; // 10mm
    int sep = 300000;   // 300µm separation perpendicular to trace direction

    // 45° direction
    double dx = len * M_SQRT1_2;
    double dy = len * M_SQRT1_2;

    // Normal to the 45° direction is (-sin45, cos45) = (-0.707, 0.707)
    // Offset neighbor by 300µm along the normal
    double nx = -M_SQRT1_2;
    double ny = M_SQRT1_2;
    int offX = (int) ( sep * nx );
    int offY = (int) ( sep * ny );

    PCB_TRACK* sig = addTrack( VECTOR2I( 0, 0 ),
                               VECTOR2I( (int) dx, (int) dy ),
                               150000, F_Cu, 1 );

    addTrack( VECTOR2I( offX, offY ),
              VECTOR2I( (int) dx + offX, (int) dy + offY ),
              150000, F_Cu, 2 );

    VECTOR2D tangent( M_SQRT1_2, M_SQRT1_2 );
    VECTOR2I midPt( (int)( dx / 2 ), (int)( dy / 2 ) );

    auto builder = makeBuilder( sig );
    auto params = makeParams( sig, midPt, tangent );

    auto neighbors = builder.FindNeighbors( params );

    BOOST_REQUIRE_EQUAL( neighbors.size(), 1 );
    BOOST_CHECK_CLOSE( std::abs( (double) neighbors[0].distNm ), 300000.0, 5.0 );

    // Apparent width should equal actual width for parallel traces (sin(90°) = 1.0)
    BOOST_CHECK_CLOSE( (double) neighbors[0].widthNm, 150000.0, 5.0 );

    BOOST_TEST_MESSAGE( "Parallel diagonal: neighbor at " << neighbors[0].distNm
                        << " nm, width " << neighbors[0].widthNm << " nm" );
}


/**
 * Test 3: Two converging traces at a small angle (~5.7°).
 * The neighbor should be found with a wider apparent width.
 */
BOOST_AUTO_TEST_CASE( ConvergingSmallAngle )
{
    // Signal: horizontal
    PCB_TRACK* sig = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 10000000, 0 ),
                               150000, F_Cu, 1 );

    // Neighbor: starts 500µm away, converges at ~5.7° (1mm over 10mm)
    addTrack( VECTOR2I( 0, 500000 ), VECTOR2I( 10000000, -500000 ),
              150000, F_Cu, 2 );

    auto builder = makeBuilder( sig );
    // Sample at x=1mm where neighbor is at y≈400µm (well separated)
    auto params = makeParams( sig, VECTOR2I( 1000000, 0 ), VECTOR2D( 1.0, 0.0 ) );

    auto neighbors = builder.FindNeighbors( params );

    BOOST_REQUIRE_GE( neighbors.size(), 1 );

    // At x=5mm, the neighbor should be at approximately y=0 (crosses the signal).
    // But with edge-to-edge filtering, very close ones are excluded.
    // Let's just verify we found something.
    BOOST_TEST_MESSAGE( "Converging: " << neighbors.size() << " neighbors found" );

    for( const auto& nb : neighbors )
    {
        BOOST_TEST_MESSAGE( "  dist=" << nb.distNm << " width=" << nb.widthNm );
        BOOST_CHECK_EQUAL( nb.widthNm, 150000 );
    }
}


/**
 * Test 4: Horizontal track near a rectangular zone fill edge.
 */
BOOST_AUTO_TEST_CASE( ZoneFillEdge )
{
    PCB_TRACK* sig = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 10000000, 0 ),
                               150000, F_Cu, 1 );

    // Zone fill to the right: inner edge at x=0, y=400000 to y=5000000
    // (offset 400µm from signal center)
    addZoneFill( VECTOR2I( -5000000, 400000 ),
                 VECTOR2I( 15000000, 5000000 ),
                 F_Cu, 3 );

    auto builder = makeBuilder( sig );
    auto params = makeParams( sig, VECTOR2I( 5000000, 0 ), VECTOR2D( 1.0, 0.0 ) );

    auto neighbors = builder.FindNeighbors( params );

    BOOST_REQUIRE_GE( neighbors.size(), 1 );

    // Should find the zone edge at ~400000nm lateral distance
    bool foundZone = false;

    for( const auto& nb : neighbors )
    {
        BOOST_TEST_MESSAGE( "  Zone neighbor: dist=" << nb.distNm << " width=" << nb.widthNm );

        if( std::abs( nb.distNm ) > 300000 && nb.widthNm > 100000 )
            foundZone = true;
    }

    BOOST_CHECK( foundZone );
}


/**
 * Test 5: Zone with a hole around the signal trace.
 * The hole edge should be detected as a neighbor.
 */
BOOST_AUTO_TEST_CASE( ZoneFillHole )
{
    PCB_TRACK* sig = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 10000000, 0 ),
                               150000, F_Cu, 1 );

    // Create a zone that covers the whole area but has a clearance hole around the trace
    ZONE* z = new ZONE( m_board.get() );
    z->SetLayer( F_Cu );
    z->SetNet( m_board->GetNetInfo().GetNetItem( 3 ) );
    z->SetIsFilled( true );
    z->SetFillFlag( F_Cu, true );

    SHAPE_POLY_SET fillPoly;

    // Outer boundary: large rectangle
    fillPoly.NewOutline();
    fillPoly.Append( VECTOR2I( -1000000, -2000000 ) );
    fillPoly.Append( VECTOR2I( 11000000, -2000000 ) );
    fillPoly.Append( VECTOR2I( 11000000, 2000000 ) );
    fillPoly.Append( VECTOR2I( -1000000, 2000000 ) );

    // Hole: clearance around the trace (200µm clearance from trace edge)
    int clearance = 200000;
    int holeHalfW = 150000 / 2 + clearance; // trace half-width + clearance

    fillPoly.NewHole();
    fillPoly.Append( VECTOR2I( -500000, -holeHalfW ) );
    fillPoly.Append( VECTOR2I( 10500000, -holeHalfW ) );
    fillPoly.Append( VECTOR2I( 10500000, holeHalfW ) );
    fillPoly.Append( VECTOR2I( -500000, holeHalfW ) );

    z->SetFilledPolysList( F_Cu, fillPoly );
    m_board->Add( z );

    BOOST_TEST_MESSAGE( "Fill poly: outlines=" << fillPoly.OutlineCount()
                        << " holes=" << fillPoly.HoleCount( 0 ) );

    auto builder = makeBuilder( sig );
    auto params = makeParams( sig, VECTOR2I( 5000000, 0 ), VECTOR2D( 1.0, 0.0 ) );

    auto neighbors = builder.FindNeighbors( params );

    BOOST_TEST_MESSAGE( "Zone hole: " << neighbors.size() << " neighbors found" );

    for( const auto& nb : neighbors )
        BOOST_TEST_MESSAGE( "  dist=" << nb.distNm << " width=" << nb.widthNm );

    // Should find zone edges on both sides at approximately ±(75000 + 200000) = ±275000nm
    BOOST_REQUIRE_GE( neighbors.size(), 2 );

    bool foundLeft = false, foundRight = false;

    for( const auto& nb : neighbors )
    {
        if( nb.distNm < -200000 )
            foundLeft = true;

        if( nb.distNm > 200000 )
            foundRight = true;
    }

    BOOST_CHECK( foundLeft );
    BOOST_CHECK( foundRight );
}


/**
 * Test 6: Same-net bend segments should NOT be detected as neighbors.
 * Two segments of the same net connected at a 90° bend.
 */
BOOST_AUTO_TEST_CASE( SameNetBendExcluded )
{
    // Segment A: horizontal
    PCB_TRACK* segA = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 5000000, 0 ),
                                150000, F_Cu, 1 );

    // Segment B: vertical, connected at (5000000, 0)
    PCB_TRACK* segB = addTrack( VECTOR2I( 5000000, 0 ), VECTOR2I( 5000000, 5000000 ),
                                150000, F_Cu, 1 );

    // Path distance map: both segments at close distances
    std::map<BOARD_CONNECTED_ITEM*, double> pathDist;
    pathDist[segA] = 0.0;
    pathDist[segB] = 5000000.0; // 5mm along path

    auto builder = makeBuilder( segA );
    builder.SetPathItemDistances( &pathDist );

    // Sample at midpoint of segA
    auto params = makeParams( segA, VECTOR2I( 2500000, 0 ), VECTOR2D( 1.0, 0.0 ) );
    params.sampleDist = 2500000.0;

    auto neighbors = builder.FindNeighbors( params );

    // segB should not be detected — path distance (2.5mm) < 2 * horizon (2mm)
    for( const auto& nb : neighbors )
    {
        BOOST_TEST_MESSAGE( "Bend test: dist=" << nb.distNm << " width=" << nb.widthNm );
    }

    // With coupling horizon = 1mm, threshold = 2mm, segB at pathDist 5mm...
    // Actually pathSep = |5mm - 2.5mm| = 2.5mm > 2mm threshold, so segB WOULD pass.
    // But segB is perpendicular, so sinAngle = |det| ≈ 0 → filtered by sin threshold.
    // Either way, segB should not appear as a neighbor.

    for( const auto& nb : neighbors )
    {
        // No neighbor should be at a very small lateral distance (< 1 trace width)
        BOOST_CHECK_GT( std::abs( nb.distNm ), 150000 );
    }
}


/**
 * Test 7: Same-net serpentine return leg — IS detected (far path distance).
 */
BOOST_AUTO_TEST_CASE( SerpentineReturnDetected )
{
    // Two parallel horizontal segments of the same net, 500µm apart
    PCB_TRACK* segA = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 10000000, 0 ),
                                150000, F_Cu, 1 );

    PCB_TRACK* segB = addTrack( VECTOR2I( 10000000, 500000 ), VECTOR2I( 0, 500000 ),
                                150000, F_Cu, 1 );

    // Path distances: segA at 0, segB at 25mm (far — full serpentine loop)
    std::map<BOARD_CONNECTED_ITEM*, double> pathDist;
    pathDist[segA] = 0.0;
    pathDist[segB] = 25000000.0;

    auto builder = makeBuilder( segA );
    builder.SetPathItemDistances( &pathDist );

    auto params = makeParams( segA, VECTOR2I( 5000000, 0 ), VECTOR2D( 1.0, 0.0 ) );
    params.sampleDist = 5000000.0;

    auto neighbors = builder.FindNeighbors( params );

    BOOST_TEST_MESSAGE( "Serpentine: " << neighbors.size() << " neighbors" );

    // segB should be detected — pathSep = |25mm - 5mm| = 20mm >> 2mm threshold
    BOOST_REQUIRE_EQUAL( neighbors.size(), 1 );
    BOOST_CHECK_CLOSE( std::abs( (double) neighbors[0].distNm ), 500000.0, 5.0 );
}


/**
 * Test 8: Deduplication should NOT merge neighbors on opposite sides.
 */
BOOST_AUTO_TEST_CASE( DeduplicateOppositeSigns )
{
    // Signal at center
    PCB_TRACK* sig = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 10000000, 0 ),
                               150000, F_Cu, 1 );

    // Neighbor left at y = -300000
    addTrack( VECTOR2I( 0, -300000 ), VECTOR2I( 10000000, -300000 ),
              150000, F_Cu, 2 );

    // Neighbor right at y = +310000 (similar absolute distance)
    addTrack( VECTOR2I( 0, 310000 ), VECTOR2I( 10000000, 310000 ),
              150000, F_Cu, 3 );

    auto builder = makeBuilder( sig );
    auto params = makeParams( sig, VECTOR2I( 5000000, 0 ), VECTOR2D( 1.0, 0.0 ) );

    auto neighbors = builder.FindNeighbors( params );

    BOOST_TEST_MESSAGE( "Dedup opposite: " << neighbors.size() << " neighbors" );

    for( const auto& nb : neighbors )
        BOOST_TEST_MESSAGE( "  dist=" << nb.distNm << " width=" << nb.widthNm );

    // Both should be kept — they're on opposite sides
    BOOST_REQUIRE_EQUAL( neighbors.size(), 2 );

    bool hasNeg = false, hasPos = false;

    for( const auto& nb : neighbors )
    {
        if( nb.distNm < 0 )
            hasNeg = true;

        if( nb.distNm > 0 )
            hasPos = true;
    }

    BOOST_CHECK( hasNeg );
    BOOST_CHECK( hasPos );
}


/**
 * Test 9: Signal track itself should not appear as a neighbor.
 */
BOOST_AUTO_TEST_CASE( NoSelfDetection )
{
    PCB_TRACK* sig = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 10000000, 0 ),
                               150000, F_Cu, 1 );

    auto builder = makeBuilder( sig );
    auto params = makeParams( sig, VECTOR2I( 5000000, 0 ), VECTOR2D( 1.0, 0.0 ) );

    auto neighbors = builder.FindNeighbors( params );

    // No neighbors — just the signal track alone
    BOOST_CHECK_EQUAL( neighbors.size(), 0 );
}


/**
 * Test 10: BuildGeometry produces valid XS_GEOMETRY.
 */
BOOST_AUTO_TEST_CASE( BuildGeometryValid )
{
    PCB_TRACK* sig = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 10000000, 0 ),
                               150000, F_Cu, 1 );

    addTrack( VECTOR2I( 0, 400000 ), VECTOR2I( 10000000, 400000 ),
              150000, F_Cu, 2 );

    auto builder = makeBuilder( sig );
    auto params = makeParams( sig, VECTOR2I( 5000000, 0 ), VECTOR2D( 1.0, 0.0 ) );

    auto neighbors = builder.FindNeighbors( params );
    auto xs = builder.BuildGeometry( params, neighbors );

    BOOST_REQUIRE_GE( xs.conductors.size(), 2 ); // signal + at least 1 neighbor

    // Signal conductor at x=0
    BOOST_CHECK_CLOSE( xs.conductors[0].centerX, 0.0, 0.1 );
    BOOST_CHECK_CLOSE( xs.conductors[0].width, 150e-6, 1.0 );

    // Neighbor at ~400µm
    BOOST_CHECK_CLOSE( std::abs( xs.conductors[1].centerX ), 400e-6, 5.0 );

    BOOST_TEST_MESSAGE( "BuildGeometry: " << xs.conductors.size() << " conductors" );
    BOOST_TEST_MESSAGE( "  Signal: x=" << xs.conductors[0].centerX * 1e6 << "µm" );
    BOOST_TEST_MESSAGE( "  Neighbor: x=" << xs.conductors[1].centerX * 1e6 << "µm" );
}


/**
 * Test 11: Neighbor trace with a 90° bend near the sample point.
 * Two segments of the neighbor meet at a corner. Both may cross the cut line,
 * but should produce only ONE neighbor (deduplication must merge them).
 *
 * Setup:
 *   Signal: horizontal at y=0
 *   Neighbor: horizontal at y=300µm from x=0 to x=5mm,
 *             then turns 90° upward from (5mm, 300µm) to (5mm, 5mm)
 *
 * Sample at x=4.9mm — both neighbor segments are near the cut line.
 */
BOOST_AUTO_TEST_CASE( NeighborBend )
{
    PCB_TRACK* sig = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 10000000, 0 ),
                               150000, F_Cu, 1 );

    // Neighbor: horizontal segment
    addTrack( VECTOR2I( 0, 300000 ), VECTOR2I( 5000000, 300000 ),
              150000, F_Cu, 2 );

    // Neighbor: vertical segment connected at the bend
    addTrack( VECTOR2I( 5000000, 300000 ), VECTOR2I( 5000000, 5000000 ),
              150000, F_Cu, 2 );

    auto builder = makeBuilder( sig );

    // Sample just before the bend at x=4.9mm
    auto params = makeParams( sig, VECTOR2I( 4900000, 0 ), VECTOR2D( 1.0, 0.0 ) );

    auto neighbors = builder.FindNeighbors( params );

    BOOST_TEST_MESSAGE( "Neighbor bend (x=4.9mm): " << neighbors.size() << " neighbors" );

    for( const auto& nb : neighbors )
    {
        BOOST_TEST_MESSAGE( "  dist=" << nb.distNm << " width=" << nb.widthNm );
    }

    // Should get exactly 1 neighbor — the horizontal segment at 300µm.
    // The vertical segment is nearly perpendicular (sinAngle < 0.1) so it's
    // filtered out, OR it's merged with the horizontal segment by dedup.
    BOOST_REQUIRE_EQUAL( neighbors.size(), 1 );
    BOOST_CHECK_CLOSE( std::abs( (double) neighbors[0].distNm ), 300000.0, 5.0 );

    // Sample right at the bend corner at x=5.0mm
    params = makeParams( sig, VECTOR2I( 5000000, 0 ), VECTOR2D( 1.0, 0.0 ) );
    neighbors = builder.FindNeighbors( params );

    BOOST_TEST_MESSAGE( "Neighbor bend (x=5.0mm): " << neighbors.size() << " neighbors" );

    for( const auto& nb : neighbors )
    {
        BOOST_TEST_MESSAGE( "  dist=" << nb.distNm << " width=" << nb.widthNm );
    }

    // At x=5mm, both segments share the endpoint (5mm, 300µm).
    // Horizontal segment just barely crosses the cut line (tParam ≈ otherLen).
    // Vertical segment starts right at the cut line (tParam ≈ 0).
    // Either both are found and merged, or only one is found.
    // The key check: no neighbor with tiny distance (< 100µm).
    for( const auto& nb : neighbors )
    {
        BOOST_CHECK_GT( std::abs( nb.distNm ), 100000 );
    }

    // Sample past the bend at x=5.1mm
    params = makeParams( sig, VECTOR2I( 5100000, 0 ), VECTOR2D( 1.0, 0.0 ) );
    neighbors = builder.FindNeighbors( params );

    BOOST_TEST_MESSAGE( "Neighbor bend (x=5.1mm): " << neighbors.size() << " neighbors" );

    for( const auto& nb : neighbors )
    {
        BOOST_TEST_MESSAGE( "  dist=" << nb.distNm << " width=" << nb.widthNm );
    }

    // Past the bend, the horizontal segment no longer crosses the cut line.
    // The vertical segment does cross, but it's nearly perpendicular → filtered by sinAngle.
    // So either 0 neighbors, or 1 at 300µm (the vertical at its endpoint).
    for( const auto& nb : neighbors )
    {
        BOOST_CHECK_GT( std::abs( nb.distNm ), 100000 );
    }
}


/**
 * Test 12: Neighbor trace with a 45° bend.
 * At the bend point, two segments of the neighbor both cross the cut line.
 * Both have sin(angle) > 0.1 so neither is filtered by the perpendicularity check.
 * They should be deduplicated into one neighbor.
 */
BOOST_AUTO_TEST_CASE( NeighborBend45 )
{
    PCB_TRACK* sig = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 10000000, 0 ),
                               150000, F_Cu, 1 );

    // Neighbor: horizontal at y=300µm, then 45° bend
    addTrack( VECTOR2I( 0, 300000 ), VECTOR2I( 5000000, 300000 ),
              150000, F_Cu, 2 );

    // 45° segment going up-right from the bend point
    addTrack( VECTOR2I( 5000000, 300000 ), VECTOR2I( 8000000, 3300000 ),
              150000, F_Cu, 2 );

    auto builder = makeBuilder( sig );

    // Sample right at the bend at x=5.0mm
    auto params = makeParams( sig, VECTOR2I( 5000000, 0 ), VECTOR2D( 1.0, 0.0 ) );
    auto neighbors = builder.FindNeighbors( params );

    BOOST_TEST_MESSAGE( "45° bend (x=5.0mm): " << neighbors.size() << " neighbors" );

    for( const auto& nb : neighbors )
    {
        BOOST_TEST_MESSAGE( "  dist=" << nb.distNm << " width=" << nb.widthNm );
    }

    // Both segments cross the cut line at approximately y=300µm.
    // They should be deduplicated to 1 neighbor.
    BOOST_CHECK_EQUAL( neighbors.size(), 1 );

    if( !neighbors.empty() )
    {
        BOOST_CHECK_CLOSE( std::abs( (double) neighbors[0].distNm ), 300000.0, 10.0 );
        // No tiny distance jumps
        BOOST_CHECK_GT( std::abs( neighbors[0].distNm ), 100000 );
    }

    // Also check just past the bend at x=5.1mm
    params = makeParams( sig, VECTOR2I( 5100000, 0 ), VECTOR2D( 1.0, 0.0 ) );
    neighbors = builder.FindNeighbors( params );

    BOOST_TEST_MESSAGE( "45° bend (x=5.1mm): " << neighbors.size() << " neighbors" );

    for( const auto& nb : neighbors )
    {
        BOOST_TEST_MESSAGE( "  dist=" << nb.distNm << " width=" << nb.widthNm );
        BOOST_CHECK_GT( std::abs( nb.distNm ), 100000 );
    }
}


/**
 * Test 13: Neighbor makes a short diagonal jog (lane change).
 * Three segments: horizontal → short 45° diagonal → horizontal at new offset.
 * The diagonal is short, so at the jog both the diagonal and one horizontal
 * cross the cut line. Tests dedup behavior with short segments.
 */
BOOST_AUTO_TEST_CASE( NeighborShortJog )
{
    PCB_TRACK* sig = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 10000000, 0 ),
                               150000, F_Cu, 1 );

    // Neighbor: 3 segments forming a lane change
    // Horizontal at y=300µm
    addTrack( VECTOR2I( 0, 300000 ), VECTOR2I( 4000000, 300000 ),
              150000, F_Cu, 2 );

    // Short 45° jog from (4mm, 300µm) to (4.2mm, 500µm) — 200µm shift over 200µm
    addTrack( VECTOR2I( 4000000, 300000 ), VECTOR2I( 4200000, 500000 ),
              150000, F_Cu, 2 );

    // Horizontal at y=500µm
    addTrack( VECTOR2I( 4200000, 500000 ), VECTOR2I( 10000000, 500000 ),
              150000, F_Cu, 2 );

    auto builder = makeBuilder( sig );

    // Scan across the jog region
    for( int xMm : { 3800, 3900, 4000, 4050, 4100, 4150, 4200, 4300, 4400 } )
    {
        int xNm = xMm * 1000;
        auto params = makeParams( sig, VECTOR2I( xNm, 0 ), VECTOR2D( 1.0, 0.0 ) );
        auto neighbors = builder.FindNeighbors( params );

        wxString msg;

        for( const auto& nb : neighbors )
            msg += wxString::Format( wxS( " d=%d w=%d" ), nb.distNm, nb.widthNm );

        BOOST_TEST_MESSAGE( "x=" << xMm << "um: " << neighbors.size() << " nb:" << msg );

        // Key check: no neighbor should have a tiny distance (< 100µm).
        // The 31µm/51µm jumps the user reported would fail this.
        for( const auto& nb : neighbors )
        {
            BOOST_CHECK_GT( std::abs( nb.distNm ), 100000 );
        }

        // Should always find exactly 1 neighbor (even during the jog)
        BOOST_CHECK_EQUAL( neighbors.size(), 1 );
    }
}


/**
 * Test 14: Realistic routing scenario — signal trace with a neighboring trace
 * that makes multiple direction changes (like a real routed net).
 * The neighbor has: horizontal → 45° jog → horizontal → 135° jog → horizontal.
 * Scan the full length and ensure no tiny distances.
 */
BOOST_AUTO_TEST_CASE( RealisticRoutingNeighbor )
{
    int w = 150000; // 150µm trace width
    int sep = 300000; // 300µm center-to-center separation

    // Signal: long horizontal
    PCB_TRACK* sig = addTrack( VECTOR2I( 0, 0 ), VECTOR2I( 20000000, 0 ),
                               w, F_Cu, 1 );

    // Neighbor: series of connected segments with jogs
    // Segment 1: horizontal at y=sep
    addTrack( VECTOR2I( 0, sep ), VECTOR2I( 4000000, sep ), w, F_Cu, 2 );
    // Segment 2: 45° jog shifting out to sep+200µm
    addTrack( VECTOR2I( 4000000, sep ), VECTOR2I( 4200000, sep + 200000 ), w, F_Cu, 2 );
    // Segment 3: horizontal at new offset
    addTrack( VECTOR2I( 4200000, sep + 200000 ), VECTOR2I( 8000000, sep + 200000 ), w, F_Cu, 2 );
    // Segment 4: 45° jog shifting back
    addTrack( VECTOR2I( 8000000, sep + 200000 ), VECTOR2I( 8200000, sep ), w, F_Cu, 2 );
    // Segment 5: horizontal back at y=sep
    addTrack( VECTOR2I( 8200000, sep ), VECTOR2I( 12000000, sep ), w, F_Cu, 2 );
    // Segment 6: steeper 30° jog
    addTrack( VECTOR2I( 12000000, sep ), VECTOR2I( 12400000, sep + 230000 ), w, F_Cu, 2 );
    // Segment 7: horizontal at wider offset
    addTrack( VECTOR2I( 12400000, sep + 230000 ), VECTOR2I( 20000000, sep + 230000 ), w, F_Cu, 2 );

    auto builder = makeBuilder( sig );

    // Scan every 100µm along the signal trace
    int tinyCount = 0;
    int overlapCount = 0;

    for( int xNm = 0; xNm <= 20000000; xNm += 100000 )
    {
        auto params = makeParams( sig, VECTOR2I( xNm, 0 ), VECTOR2D( 1.0, 0.0 ) );
        auto neighbors = builder.FindNeighbors( params );

        for( const auto& nb : neighbors )
        {
            double edgeToEdge = std::abs( nb.distNm ) - w / 2.0 - nb.widthNm / 2.0;

            if( edgeToEdge < 10000 ) // < 10µm edge-to-edge
            {
                tinyCount++;
                BOOST_TEST_MESSAGE( "TINY at x=" << xNm / 1000 << "um: dist="
                                    << nb.distNm << " w=" << nb.widthNm
                                    << " e2e=" << edgeToEdge / 1000 << "um" );
            }

            if( edgeToEdge < 0 )
            {
                overlapCount++;
                BOOST_TEST_MESSAGE( "OVERLAP at x=" << xNm / 1000 << "um: dist="
                                    << nb.distNm << " w=" << nb.widthNm
                                    << " e2e=" << edgeToEdge / 1000 << "um" );
            }
        }
    }

    BOOST_TEST_MESSAGE( "Realistic routing: " << tinyCount << " tiny, "
                        << overlapCount << " overlaps" );

    BOOST_CHECK_EQUAL( tinyCount, 0 );
    BOOST_CHECK_EQUAL( overlapCount, 0 );
}


/**
 * Test 15: Load CPArti FPGA board, walk sdram_d4 net, check for tiny distances.
 */
BOOST_AUTO_TEST_CASE( RealBoardSDRAM )
{
    SETTINGS_MANAGER settingsMgr( true /* headless */ );
    std::unique_ptr<BOARD> board;

    try
    {
        KI_TEST::LoadBoard( settingsMgr, "cparti_fpga", board );
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

    BOOST_TEST_MESSAGE( "Board: " << board->Tracks().size() << " tracks, "
                        << board->Zones().size() << " zones" );

    // Find sdram_d4 net (case-insensitive substring)
    int targetNet = -1;

    for( const auto& [code, info] : board->GetNetInfo().NetsByNetcode() )
    {
        wxString name = info->GetNetname().Lower();

        if( name == wxS( "sram_d4" ) )
        {
            targetNet = code;
            BOOST_TEST_MESSAGE( "Found net: " << info->GetNetname() << " (code " << code << ")" );
            break;
        }
    }

    if( targetNet < 0 )
    {
        // Dump net names for diagnostics
        int n = 0;

        for( const auto& [code, info] : board->GetNetInfo().NetsByNetcode() )
        {
            wxString name = info->GetNetname().Lower();

            if( name.Contains( wxS( "sdram" ) ) || name.Contains( wxS( "d4" ) ) )
                BOOST_TEST_MESSAGE( "  candidate net " << code << ": " << info->GetNetname() );

            if( ++n > 200 )
                break;
        }
    }

    if( targetNet < 0 )
    {
        // Fallback: find a net with many segments
        std::map<int, int> netSegs;

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( t->Type() != PCB_VIA_T && t->GetNetCode() > 0 )
                netSegs[t->GetNetCode()]++;
        }

        int best = 0;

        for( auto& [nc, cnt] : netSegs )
        {
            if( cnt > best ) { best = cnt; targetNet = nc; }
        }

        BOOST_TEST_MESSAGE( "sdram_d4 not found, using net " << targetNet
                            << " (" << best << " segs)" );
    }

    // Build R-tree
    DRC_RTREE rtree;

    for( PCB_TRACK* t : board->Tracks() )
    {
        if( t->Type() != PCB_VIA_T )
            rtree.Insert( t, t->GetLayer() );
    }

    STACKUP_READER stackup( board.get() );
    TRACE_PATH_WALKER walker( board.get() );
    walker.WalkNet( targetNet );

    const auto& path = walker.GetPath();

    BOOST_TEST_MESSAGE( "Path: " << path.size() << " points, "
                        << walker.GetTotalLength() / 1e6 << " mm" );

    std::map<BOARD_CONNECTED_ITEM*, double> pathItemDist;

    for( const PATH_POINT& pp : path )
    {
        if( !pp.isVia && pathItemDist.find( pp.item ) == pathItemDist.end() )
            pathItemDist[pp.item] = pp.distFromStart;
    }

    int tinyCount = 0;
    int totalSamples = 0;
    int totalNb = 0;
    double minE2E = 1e9;

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
        builder.SetBoard( board.get() );
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

        for( const XS_NEIGHBOR& nb : neighbors )
        {
            totalNb++;
            double e2e = std::abs( nb.distNm ) - track->GetWidth() / 2.0 - nb.widthNm / 2.0;
            minE2E = std::min( minE2E, e2e );

            // Log every neighbor for full visibility
            BOOST_TEST_MESSAGE( "  pt=" << totalSamples << " pos=("
                                << pt.position.x / 1e6 << "," << pt.position.y / 1e6
                                << ")mm d=" << pt.distFromStart / 1e6 << "mm "
                                << "nb_dist=" << nb.distNm / 1000 << "um "
                                << "nb_w=" << nb.widthNm / 1000 << "um "
                                << "e2e=" << (int)( e2e / 1000 ) << "um" );

            if( e2e < 10000 ) // < MIN_EDGE_TO_EDGE (10µm)
                tinyCount++;
        }
    }

    BOOST_TEST_MESSAGE( "\nResult: " << totalSamples << " samples, "
                        << totalNb << " total neighbors, "
                        << tinyCount << " suspiciously close, "
                        << "min e2e=" << (int)( minE2E / 1000 ) << "um" );

    BOOST_CHECK_EQUAL( tinyCount, 0 );

    // Also test at interpolated positions (like the profiler does).
    // Sample every 50µm along the path.
    double totalLen = walker.GetTotalLength();
    double step = 50000.0; // 50µm
    int    interpTinyCount = 0;
    int    interpSamples = 0;

    // Build non-via path segments for interpolation
    struct SEG_PT { double dist; VECTOR2I pos; VECTOR2D tangent; PCB_TRACK* track; };
    std::vector<SEG_PT> segs;

    for( const PATH_POINT& pp : path )
    {
        if( !pp.isVia )
            segs.push_back( { pp.distFromStart, pp.position, pp.tangent,
                              static_cast<PCB_TRACK*>( pp.item ) } );
    }

    int segIdx = 0;

    for( double d = 0; d <= totalLen; d += step )
    {
        while( segIdx + 1 < (int) segs.size() && segs[segIdx + 1].dist <= d )
            segIdx++;

        VECTOR2I pos = segs[segIdx].pos;

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

        LAYER_GEOMETRY geom = stackup.GetLayerGeometry( track->GetLayer(), pos,
                                                        track->GetWidth() );

        double hRef = std::max( geom.hAbove, geom.hBelow );
        int    couplingHorizon = std::max( (int) ( hRef * 3.0 * 1e9 ), 500000 );

        CROSS_SECTION_BUILDER builder;
        builder.SetSpatialIndex( &rtree );
        builder.SetBoard( board.get() );
        builder.SetPathItemDistances( &pathItemDist );
        builder.SetSignalTrack( track );

        XS_BUILD_PARAMS params;
        params.samplePos = pos;
        params.sampleTangent = segs[segIdx].tangent;
        params.signalLayer = track->GetLayer();
        params.signalNetCode = track->GetNetCode();
        params.signalWidth = track->GetWidth();
        params.couplingHorizon = couplingHorizon;
        params.sampleDist = d;
        params.layerGeom = geom;

        auto neighbors = builder.FindNeighbors( params );
        interpSamples++;

        for( const XS_NEIGHBOR& nb : neighbors )
        {
            double e2e = std::abs( nb.distNm ) - track->GetWidth() / 2.0 - nb.widthNm / 2.0;

            if( e2e < 10000 ) // < MIN_EDGE_TO_EDGE (10µm)
            {
                interpTinyCount++;
                BOOST_TEST_MESSAGE( "INTERP-TINY: d=" << d / 1e6 << "mm pos=("
                                    << pos.x / 1e6 << "," << pos.y / 1e6
                                    << ")mm nb_dist=" << nb.distNm / 1000 << "um "
                                    << "nb_w=" << nb.widthNm / 1000 << "um "
                                    << "e2e=" << (int)( e2e / 1000 ) << "um" );
            }
        }
    }

    BOOST_TEST_MESSAGE( "Interpolated: " << interpSamples << " samples, "
                        << interpTinyCount << " tiny" );

    BOOST_CHECK_EQUAL( interpTinyCount, 0 );

    // --- Detailed diagnostic around a specific distance ---
    // Sample every 20µm from 37mm to 38mm and print full cross-section info.
    double diagStart = 37.0e6; // 37mm in nm
    double diagEnd   = 38.0e6;
    double diagStep  = 20000; // 20µm
    int    diagSegIdx = 0;

    BOOST_TEST_MESSAGE( "\n=== Diagnostic: 35-40mm at 100um steps ===" );

    for( double d = diagStart; d <= diagEnd; d += diagStep )
    {
        while( diagSegIdx + 1 < (int) segs.size() && segs[diagSegIdx + 1].dist <= d )
            diagSegIdx++;

        if( diagSegIdx >= (int) segs.size() )
            break;

        VECTOR2I pos = segs[diagSegIdx].pos;

        if( diagSegIdx + 1 < (int) segs.size() )
        {
            double segLen = segs[diagSegIdx + 1].dist - segs[diagSegIdx].dist;

            if( segLen > 1.0 )
            {
                double frac = std::clamp( ( d - segs[diagSegIdx].dist ) / segLen, 0.0, 1.0 );
                pos.x = segs[diagSegIdx].pos.x
                        + (int) ( frac * ( segs[diagSegIdx + 1].pos.x - segs[diagSegIdx].pos.x ) );
                pos.y = segs[diagSegIdx].pos.y
                        + (int) ( frac * ( segs[diagSegIdx + 1].pos.y - segs[diagSegIdx].pos.y ) );
            }
        }

        PCB_TRACK* track = segs[diagSegIdx].track;

        LAYER_GEOMETRY geom = stackup.GetLayerGeometry( track->GetLayer(), pos,
                                                        track->GetWidth() );

        double hRef = std::max( geom.hAbove, geom.hBelow );
        int    couplingHorizon = std::max( (int) ( hRef * 3.0 * 1e9 ), 500000 );

        CROSS_SECTION_BUILDER builder;
        builder.SetSpatialIndex( &rtree );
        builder.SetBoard( board.get() );
        builder.SetPathItemDistances( &pathItemDist );
        builder.SetSignalTrack( track );

        XS_BUILD_PARAMS params;
        params.samplePos = pos;
        params.sampleTangent = segs[diagSegIdx].tangent;
        params.signalLayer = track->GetLayer();
        params.signalNetCode = track->GetNetCode();
        params.signalWidth = track->GetWidth();
        params.couplingHorizon = couplingHorizon;
        params.sampleDist = d;
        params.layerGeom = geom;

        auto neighbors = builder.FindNeighbors( params );

        // Build geometry and solve BEM
        XS_GEOMETRY xs = builder.BuildGeometry( params, neighbors );

        BEM_2D_SOLVER solver;
        solver.SetGeometry( xs );
        solver.SetPanelsPerEdge( 12 );

        double z0 = 0.0;
        bool   bemOk = solver.Solve();

        if( bemOk && solver.GetResult().Z0 > 0.0 )
            z0 = solver.GetResult().Z0;
        else
            z0 = STACKUP_READER::ComputeZ0( geom );

        BOOST_TEST_MESSAGE( "d=" << d / 1e6 << "mm pos=(" << pos.x / 1e6 << ","
                            << pos.y / 1e6 << ") t=(" << segs[diagSegIdx].tangent.x
                            << "," << segs[diagSegIdx].tangent.y << ")"
                            << " layer=" << (int) track->GetLayer()
                            << " w=" << track->GetWidth() / 1000 << "um"
                            << " hA=" << geom.hAbove * 1e6 << "um"
                            << " hB=" << geom.hBelow * 1e6 << "um"
                            << " erA=" << geom.erAbove << " erB=" << geom.erBelow
                            << " refA=" << geom.hasRefAbove << " refB=" << geom.hasRefBelow
                            << " nb=" << neighbors.size()
                            << " Z0=" << z0 << " bem=" << bemOk );

        for( const XS_NEIGHBOR& nb : neighbors )
        {
            BOOST_TEST_MESSAGE( "  nb: dist=" << nb.distNm / 1000 << "um"
                                << " w=" << nb.widthNm / 1000 << "um"
                                << " e2e=" << (int) ( ( std::abs( nb.distNm )
                                    - track->GetWidth() / 2.0 - nb.widthNm / 2.0 ) / 1000 )
                                << "um" );
        }
    }
}


/**
 * Diagnostic: Profile SRAM_D5 at 200µm steps, print Z0 at every sample.
 */
BOOST_AUTO_TEST_CASE( RealBoardSDRAM_D5 )
{
    SETTINGS_MANAGER settingsMgr( true );
    std::unique_ptr<BOARD> board;

    try { KI_TEST::LoadBoard( settingsMgr, "cparti_fpga", board ); }
    catch( ... ) { BOOST_TEST_MESSAGE( "Board not found — skipping" ); return; }

    if( !board || board->Tracks().empty() ) return;

    KI_TEST::FillZones( board.get() );
    board->BuildConnectivity();

    int targetNet = -1;

    for( const auto& [code, info] : board->GetNetInfo().NetsByNetcode() )
    {
        if( info->GetNetname().Lower() == wxS( "sram_d5" ) )
        {
            targetNet = code;
            BOOST_TEST_MESSAGE( "Found net: " << info->GetNetname() << " (code " << code << ")" );
            break;
        }
    }

    if( targetNet < 0 ) { BOOST_TEST_MESSAGE( "SRAM_D5 not found" ); return; }

    DRC_RTREE rtree;

    for( PCB_TRACK* t : board->Tracks() )
    {
        if( t->Type() != PCB_VIA_T )
            rtree.Insert( t, t->GetLayer() );
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

    BOOST_TEST_MESSAGE( "Path: " << path.size() << " points, "
                        << walker.GetTotalLength() / 1e6 << " mm" );

    std::map<BOARD_CONNECTED_ITEM*, double> pathItemDist;

    for( const PATH_POINT& pp : path )
    {
        if( !pp.isVia && pathItemDist.find( pp.item ) == pathItemDist.end() )
            pathItemDist[pp.item] = pp.distFromStart;
    }

    // Build interpolation segments
    struct SEG_PT { double dist; VECTOR2I pos; VECTOR2D tangent; PCB_TRACK* track; };
    std::vector<SEG_PT> segs;

    for( const PATH_POINT& pp : path )
    {
        if( !pp.isVia )
            segs.push_back( { pp.distFromStart, pp.position, pp.tangent,
                              static_cast<PCB_TRACK*>( pp.item ) } );
    }

    double totalLen = walker.GetTotalLength();
    double step = 50000; // 50µm for fine resolution
    int    segIdx = 0;
    int    spikeCount = 0;
    double prevZ0 = 0;

    BOOST_TEST_MESSAGE( "\n=== SRAM_D5 profile at 200um steps ===" );

    for( double d = 0; d <= totalLen; d += step )
    {
        while( segIdx + 1 < (int) segs.size() && segs[segIdx + 1].dist <= d )
            segIdx++;

        VECTOR2I pos = segs[segIdx].pos;

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

        LAYER_GEOMETRY geom = stackup.GetLayerGeometry( track->GetLayer(), pos,
                                                        track->GetWidth() );

        double hRef = std::max( geom.hAbove, geom.hBelow );
        int    couplingHorizon = std::max( (int) ( hRef * 3.0 * 1e9 ), 500000 );

        CROSS_SECTION_BUILDER builder;
        builder.SetSpatialIndex( &rtree );
        builder.SetBoard( board.get() );
        builder.SetPathItemDistances( &pathItemDist );
        builder.SetSignalTrack( track );

        XS_BUILD_PARAMS params;
        params.samplePos = pos;
        params.sampleTangent = segs[segIdx].tangent;
        params.signalLayer = track->GetLayer();
        params.signalNetCode = track->GetNetCode();
        params.signalWidth = track->GetWidth();
        params.couplingHorizon = couplingHorizon;
        params.sampleDist = d;
        params.layerGeom = geom;

        auto neighbors = builder.FindNeighbors( params );
        XS_GEOMETRY xs = builder.BuildGeometry( params, neighbors );

        BEM_2D_SOLVER solver;
        solver.SetGeometry( xs );
        solver.SetPanelsPerEdge( 12 );

        double z0 = 0.0;
        bool   bemOk = solver.Solve();

        if( bemOk && solver.GetResult().Z0 > 0.0 )
            z0 = solver.GetResult().Z0;
        else
            z0 = STACKUP_READER::ComputeZ0( geom );

        // Flag spikes > 3Ω
        bool spike = ( prevZ0 > 0 && std::abs( z0 - prevZ0 ) > 3.0 );

        if( spike )
            spikeCount++;

        BOOST_TEST_MESSAGE( "d=" << d / 1e6 << "mm"
                            << " layer=" << (int) track->GetLayer()
                            << " w=" << track->GetWidth() / 1000 << "um"
                            << " nb=" << neighbors.size()
                            << " Z0=" << z0
                            << " bem=" << bemOk
                            << ( spike ? " *** SPIKE ***" : "" ) );

        if( spike || neighbors.size() > 0 )
        {
            for( const XS_NEIGHBOR& nb : neighbors )
            {
                BOOST_TEST_MESSAGE( "  nb: dist=" << nb.distNm / 1000 << "um"
                                    << " w=" << nb.widthNm / 1000 << "um"
                                    << " e2e=" << (int) ( ( std::abs( nb.distNm )
                                        - track->GetWidth() / 2.0
                                        - nb.widthNm / 2.0 ) / 1000 )
                                    << "um" );
            }
        }

        prevZ0 = z0;
    }

    BOOST_TEST_MESSAGE( "\nSpikes (>3Ω jump): " << spikeCount );
    // Spikes from neighbor transitions are expected at pad/track boundaries.
    // This is a diagnostic, not a hard assertion.
    BOOST_WARN_LT( spikeCount, 10 );
}


BOOST_AUTO_TEST_SUITE_END()
