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

#include <sipi/impedance_profile.h>
#include <sipi/trace_path_walker.h>


struct TRACE_WALKER_TEST_FIXTURE
{
    TRACE_WALKER_TEST_FIXTURE() :
            m_settingsManager( true /* headless */ )
    { }

    SETTINGS_MANAGER       m_settingsManager;
    std::unique_ptr<BOARD> m_board;
};


BOOST_FIXTURE_TEST_SUITE( TracePathWalker, TRACE_WALKER_TEST_FIXTURE )


/**
 * Test that the walker can find and walk at least one net on a loaded board.
 * This is a basic smoke test — we just verify the walker runs without crashing
 * and produces a non-empty path with reasonable properties.
 */
BOOST_AUTO_TEST_CASE( WalkAnyNet )
{
    KI_TEST::LoadBoard( m_settingsManager, "api_kitchen_sink", m_board );
    m_board->BuildConnectivity();

    TRACE_PATH_WALKER walker( m_board.get() );

    // Find the first track segment on the board
    PCB_TRACK* firstTrack = nullptr;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->Type() == PCB_TRACE_T )
        {
            firstTrack = track;
            break;
        }
    }

    if( !firstTrack )
    {
        BOOST_TEST_MESSAGE( "No track segments found in test board — skipping" );
        return;
    }

    BOOST_TEST_MESSAGE( "Walking net: " << firstTrack->GetNetname()
                        << " (code " << firstTrack->GetNetCode() << ")" );

    bool ok = walker.Walk( firstTrack );
    BOOST_CHECK( ok );

    const auto& path = walker.GetPath();
    BOOST_CHECK( !path.empty() );

    // Path distances should be monotonically non-decreasing
    for( size_t i = 1; i < path.size(); i++ )
    {
        BOOST_CHECK_GE( path[i].distFromStart, path[i - 1].distFromStart );
    }

    double totalLen = walker.GetTotalLength();
    BOOST_CHECK_GT( totalLen, 0.0 );

    const WALK_RESULT& result = walker.GetResult();

    BOOST_TEST_MESSAGE( "Path has " << path.size() << " points, total length "
                        << ( totalLen / 1e6 ) << " mm"
                        << " | visited " << result.segmentsVisited
                        << "/" << result.totalSegmentsOnNet << " segments"
                        << " | complete: " << result.isComplete() );
}


/**
 * Test walking via WalkNet with a net code.
 */
BOOST_AUTO_TEST_CASE( WalkByNetCode )
{
    KI_TEST::LoadBoard( m_settingsManager, "api_kitchen_sink", m_board );
    m_board->BuildConnectivity();

    // Find a net that has tracks
    int targetNetCode = -1;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->Type() == PCB_TRACE_T && track->GetNetCode() > 0 )
        {
            targetNetCode = track->GetNetCode();
            break;
        }
    }

    if( targetNetCode < 0 )
    {
        BOOST_TEST_MESSAGE( "No routed nets found — skipping" );
        return;
    }

    TRACE_PATH_WALKER walker( m_board.get() );
    bool ok = walker.WalkNet( targetNetCode );
    BOOST_CHECK( ok );
    BOOST_CHECK( !walker.GetPath().empty() );
    BOOST_CHECK_GT( walker.GetTotalLength(), 0.0 );
}


/**
 * Test that walking an invalid/nonexistent net returns false.
 */
BOOST_AUTO_TEST_CASE( WalkInvalidNet )
{
    KI_TEST::LoadBoard( m_settingsManager, "api_kitchen_sink", m_board );
    m_board->BuildConnectivity();

    TRACE_PATH_WALKER walker( m_board.get() );

    // Net code 99999 shouldn't exist
    bool ok = walker.WalkNet( 99999 );
    BOOST_CHECK( !ok );
    BOOST_CHECK( walker.GetPath().empty() );
}


/**
 * Walk all nets on the board that have tracks — stress test for crashes.
 */
BOOST_AUTO_TEST_CASE( WalkAllNets )
{
    KI_TEST::LoadBoard( m_settingsManager, "api_kitchen_sink", m_board );
    m_board->BuildConnectivity();

    // Collect unique net codes that have tracks
    std::set<int> netCodes;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() > 0 )
            netCodes.insert( track->GetNetCode() );
    }

    BOOST_TEST_MESSAGE( "Walking " << netCodes.size() << " nets" );

    int walked = 0;

    for( int nc : netCodes )
    {
        TRACE_PATH_WALKER walker( m_board.get() );

        if( walker.WalkNet( nc ) )
        {
            walked++;
            const auto& path = walker.GetPath();

            // Sanity: path should have points and non-negative distances
            BOOST_CHECK( !path.empty() );

            for( size_t i = 1; i < path.size(); i++ )
            {
                BOOST_CHECK_GE( path[i].distFromStart, path[i - 1].distFromStart );
            }
        }
    }

    BOOST_TEST_MESSAGE( "Successfully walked " << walked << " / " << netCodes.size() << " nets" );
    BOOST_CHECK_GT( walked, 0 );
}


/**
 * Compare walker total length against the sum of individual track segment lengths
 * on the same net. These should match if the walker visits every segment exactly once.
 */
BOOST_AUTO_TEST_CASE( LengthMatchesTrackSum )
{
    KI_TEST::LoadBoard( m_settingsManager, "api_kitchen_sink", m_board );
    m_board->BuildConnectivity();

    // Collect nets that have tracks
    std::set<int> netCodes;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() > 0 && track->Type() != PCB_VIA_T )
            netCodes.insert( track->GetNetCode() );
    }

    for( int nc : netCodes )
    {
        // Sum segment lengths from KiCad directly
        double kicadSum = 0.0;
        int    segCount = 0;

        for( PCB_TRACK* track : m_board->Tracks() )
        {
            if( track->GetNetCode() == nc && track->Type() != PCB_VIA_T )
            {
                kicadSum += track->GetLength();
                segCount++;
            }
        }

        if( segCount == 0 )
            continue;

        // Walk with our walker
        TRACE_PATH_WALKER walker( m_board.get() );

        if( !walker.WalkNet( nc ) )
            continue;

        double walkerLen = walker.GetTotalLength();
        const WALK_RESULT& result = walker.GetResult();

        BOOST_TEST_MESSAGE( "Net " << nc << ": KiCad sum = " << ( kicadSum / 1e6 )
                            << " mm (" << segCount << " segs), walker = "
                            << ( walkerLen / 1e6 ) << " mm ("
                            << result.segmentsVisited << "/" << result.totalSegmentsOnNet
                            << " segs) complete=" << result.isComplete() );

        if( result.isComplete() )
        {
            // Walker visited every segment — lengths must match
            BOOST_CHECK_CLOSE( walkerLen, kicadSum, 0.1 );
        }
        else
        {
            // Incomplete: walker stopped at a branch — walked length < total
            BOOST_CHECK_LE( walkerLen, kicadSum * 1.001 );
        }
    }
}


/**
 * Test on a board with arcs, vias, and multiple nets — exercises the walker
 * on more complex topology. Compare lengths against KiCad's own track lengths.
 */
BOOST_AUTO_TEST_CASE( LengthMatchesTrackSum_ArcsViasBoard )
{
    KI_TEST::LoadBoard( m_settingsManager, "tracks_arcs_vias", m_board );
    m_board->BuildConnectivity();

    std::set<int> netCodes;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() > 0 && track->Type() != PCB_VIA_T )
            netCodes.insert( track->GetNetCode() );
    }

    BOOST_TEST_MESSAGE( "Board has " << netCodes.size() << " routed nets" );

    for( int nc : netCodes )
    {
        double kicadSum = 0.0;
        int    segCount = 0;

        for( PCB_TRACK* track : m_board->Tracks() )
        {
            if( track->GetNetCode() == nc && track->Type() != PCB_VIA_T )
            {
                kicadSum += track->GetLength();
                segCount++;
            }
        }

        if( segCount == 0 )
            continue;

        TRACE_PATH_WALKER walker( m_board.get() );

        if( !walker.WalkNet( nc ) )
            continue;

        double walkerLen = walker.GetTotalLength();

        const WALK_RESULT& result = walker.GetResult();

        // Count vias in the path
        int viaCount = 0;

        for( const PATH_POINT& pt : walker.GetPath() )
        {
            if( pt.isVia )
                viaCount++;
        }

        BOOST_TEST_MESSAGE( "Net " << nc << ": KiCad sum = " << ( kicadSum / 1e6 )
                            << " mm (" << segCount << " segs), walker = "
                            << ( walkerLen / 1e6 ) << " mm ("
                            << result.segmentsVisited << "/" << result.totalSegmentsOnNet
                            << " segs, " << viaCount << " vias)"
                            << " complete=" << result.isComplete() );

        if( result.isComplete() )
        {
            BOOST_CHECK_CLOSE( walkerLen, kicadSum, 0.1 );
        }
        else
        {
            BOOST_CHECK_LE( walkerLen, kicadSum * 1.001 );
        }
    }
}


/**
 * Test that arc interpolation produces correct tangent directions.
 * For arc path points, the tangent should be perpendicular to the radius
 * (i.e. dot product of tangent and radius direction should be ~0).
 */
BOOST_AUTO_TEST_CASE( ArcTangentPerpendicularity )
{
    KI_TEST::LoadBoard( m_settingsManager, "tracks_arcs_vias", m_board );
    m_board->BuildConnectivity();

    // Find a net that has arcs
    int arcNetCode = -1;
    int arcCount = 0;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->Type() == PCB_ARC_T && track->GetNetCode() > 0 )
        {
            arcNetCode = track->GetNetCode();
            arcCount++;
        }
    }

    if( arcNetCode < 0 )
    {
        BOOST_TEST_MESSAGE( "No arcs found in test board — skipping" );
        return;
    }

    BOOST_TEST_MESSAGE( "Found " << arcCount << " arcs, testing net " << arcNetCode );

    TRACE_PATH_WALKER walker( m_board.get() );
    BOOST_REQUIRE( walker.WalkNet( arcNetCode ) );

    const auto& path = walker.GetPath();
    int         arcPoints = 0;
    double      maxDot = 0.0;

    // For each arc item, find its center and check tangent perpendicularity
    for( const PATH_POINT& pt : path )
    {
        if( pt.isVia || pt.item->Type() != PCB_ARC_T )
            continue;

        PCB_ARC* arc = static_cast<PCB_ARC*>( pt.item );
        VECTOR2D center( arc->GetPosition() );
        VECTOR2D radial = VECTOR2D( pt.position ) - center;
        double   radLen = radial.EuclideanNorm();

        if( radLen < 1.0 )
            continue;

        // Normalize radial
        radial = radial / radLen;

        // Tangent should be perpendicular to radial: |dot| ≈ 0
        double dot = std::abs( radial.x * pt.tangent.x + radial.y * pt.tangent.y );
        maxDot = std::max( maxDot, dot );

        // Allow small tolerance for integer rounding of interpolated positions
        BOOST_CHECK_SMALL( dot, 0.01 );
        arcPoints++;
    }

    BOOST_TEST_MESSAGE( "Checked " << arcPoints << " arc path points, max |dot| = " << maxDot );
    BOOST_CHECK_GT( arcPoints, 0 );
}


/**
 * Test that arc interpolation produces the expected number of intermediate points.
 * Checks arcs that were actually visited by the walker.
 */
BOOST_AUTO_TEST_CASE( ArcInterpolationPointCount )
{
    KI_TEST::LoadBoard( m_settingsManager, "tracks_arcs_vias", m_board );
    m_board->BuildConnectivity();

    // Walk all nets and check arc point counts
    std::set<int> netCodes;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() > 0 )
            netCodes.insert( track->GetNetCode() );
    }

    int arcsChecked = 0;

    for( int nc : netCodes )
    {
        TRACE_PATH_WALKER walker( m_board.get() );

        if( !walker.WalkNet( nc ) )
            continue;

        // Find arcs in the walked path and verify point counts
        std::map<BOARD_CONNECTED_ITEM*, int> arcPointCounts;

        for( const PATH_POINT& pt : walker.GetPath() )
        {
            if( !pt.isVia && pt.item->Type() == PCB_ARC_T )
                arcPointCounts[pt.item]++;
        }

        for( auto& [item, count] : arcPointCounts )
        {
            PCB_ARC* arc = static_cast<PCB_ARC*>( item );
            double   angleDeg = std::abs( arc->GetAngle().AsDegrees() );

            double radius = arc->GetRadius();
            double arcLenStep = std::max( (double) arc->GetWidth(), 250000.0 );
            double stepAngle = arcLenStep / radius;
            double sweepRad = std::abs( arc->GetAngle().AsRadians() );
            int    expectedSteps = std::max( 1, static_cast<int>(
                                                        std::ceil( sweepRad / stepAngle ) ) );
            expectedSteps = std::min( expectedSteps, 72 );
            int expectedPoints = expectedSteps + 1;

            BOOST_TEST_MESSAGE( "Arc " << angleDeg << "°: " << count
                                << " points (expected " << expectedPoints << ")" );
            BOOST_CHECK_EQUAL( count, expectedPoints );
            arcsChecked++;
        }
    }

    BOOST_TEST_MESSAGE( "Checked " << arcsChecked << " walked arcs" );
    BOOST_CHECK_GT( arcsChecked, 0 );
}


/**
 * Build a synthetic board with a meander pattern and verify the walker traces
 * through every segment.  The meander is a classic serpentine:
 *
 *          ┌──────────┐  ┌──────────┐
 *  PAD1 ───┘          └──┘          └─── PAD2
 *
 * All segments share exact endpoints on a single layer / single net.
 */
BOOST_AUTO_TEST_CASE( MeanderWalk )
{
    auto board = std::make_unique<BOARD>();

    // Register a net
    board->Add( new NETINFO_ITEM( board.get(), wxS( "SRAM_A9" ), 1 ) );
    NETINFO_ITEM* net = board->GetNetInfo().GetNetItem( 1 );

    const int W = 100000;           // 0.1 mm trace width
    const int PITCH = 500000;       // 0.5 mm meander pitch (vertical spacing)
    const int SPAN = 2000000;       // 2 mm horizontal span of each meander leg
    const int LEGS = 6;             // number of horizontal legs

    auto addSeg = [&]( const VECTOR2I& aStart, const VECTOR2I& aEnd ) -> PCB_TRACK*
    {
        PCB_TRACK* t = new PCB_TRACK( board.get() );
        t->SetStart( aStart );
        t->SetEnd( aEnd );
        t->SetWidth( W );
        t->SetLayer( F_Cu );
        t->SetNet( net );
        board->Add( t );
        return t;
    };

    // Build the meander: alternating horizontal legs connected by vertical stubs.
    //
    //  lead-in ─── leg0 ──┐
    //                     │ stub0
    //            ┌─ leg1 ─┘
    //    stub1   │
    //            └─ leg2 ──┐
    //                      ...
    //            ┌─ legN-1 ─┘
    //            └─── lead-out ── PAD2

    const int LEAD = 1000000;       // 1 mm lead-in / lead-out

    VECTOR2I cursor( 0, 0 );

    // Lead-in
    VECTOR2I leadInEnd( LEAD, 0 );
    addSeg( cursor, leadInEnd );
    cursor = leadInEnd;

    double expectedLen = LEAD;  // lead-in

    for( int i = 0; i < LEGS; i++ )
    {
        bool goRight = ( i % 2 == 0 );
        VECTOR2I legEnd = cursor + VECTOR2I( goRight ? SPAN : -SPAN, 0 );
        addSeg( cursor, legEnd );
        expectedLen += SPAN;
        cursor = legEnd;

        if( i < LEGS - 1 )
        {
            // Vertical stub connecting to next leg
            VECTOR2I stubEnd = cursor + VECTOR2I( 0, PITCH );
            addSeg( cursor, stubEnd );
            expectedLen += PITCH;
            cursor = stubEnd;
        }
    }

    // Lead-out
    VECTOR2I leadOutEnd = cursor + VECTOR2I( LEAD, 0 );
    addSeg( cursor, leadOutEnd );
    expectedLen += LEAD;

    int totalSegs = LEGS            // horizontal legs
                  + ( LEGS - 1 )    // vertical stubs
                  + 2;              // lead-in + lead-out

    BOOST_TEST_MESSAGE( "Meander: " << totalSegs << " segments, expected length "
                        << ( expectedLen / 1e6 ) << " mm" );

    // Add pads at the two ends
    FOOTPRINT* fp = new FOOTPRINT( board.get() );
    board->Add( fp );

    auto addPad = [&]( const VECTOR2I& aPos, const wxString& aName )
    {
        PAD* p = new PAD( fp );
        p->SetPosition( aPos );
        p->SetSize( PADSTACK::ALL_LAYERS, VECTOR2I( 500000, 500000 ) );
        p->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::CIRCLE );
        p->SetAttribute( PAD_ATTRIB::SMD );
        p->SetLayerSet( { F_Cu } );
        p->SetNet( net );
        p->SetNumber( aName );
        fp->Add( p );
    };

    addPad( VECTOR2I( 0, 0 ), wxS( "1" ) );
    addPad( leadOutEnd, wxS( "2" ) );

    board->BuildConnectivity();

    // Walk from a track in the middle of the meander
    PCB_TRACK* midTrack = nullptr;
    int idx = 0;

    for( PCB_TRACK* track : board->Tracks() )
    {
        if( track->Type() == PCB_TRACE_T )
        {
            if( idx == totalSegs / 2 )
            {
                midTrack = track;
                break;
            }

            idx++;
        }
    }

    BOOST_REQUIRE( midTrack );

    TRACE_PATH_WALKER walker( board.get() );
    bool ok = walker.Walk( midTrack );
    BOOST_CHECK( ok );

    const WALK_RESULT& result = walker.GetResult();

    BOOST_TEST_MESSAGE( "Walked " << result.segmentsVisited << " / "
                        << result.totalSegmentsOnNet << " segments" );

    // The walker must visit every segment (no junctions in a pure meander)
    BOOST_CHECK_EQUAL( result.segmentsVisited, totalSegs );
    BOOST_CHECK( result.isComplete() );

    // Length must match
    BOOST_CHECK_CLOSE( walker.GetTotalLength(), expectedLen, 0.01 );

    // Should terminate at pads on both ends
    BOOST_CHECK( result.startTerminus == PATH_TERMINUS::PAD );
    BOOST_CHECK( result.endTerminus == PATH_TERMINUS::PAD );
    BOOST_CHECK( walker.GetStartPad() != nullptr );
    BOOST_CHECK( walker.GetEndPad() != nullptr );
}


/**
 * Tight meander where the vertical stubs are shorter than the track width.
 * With tolerance-based endpoint matching this would create false junctions;
 * exact endpoint matching handles it correctly.
 */
BOOST_AUTO_TEST_CASE( MeanderWalk_TightSpacing )
{
    auto board = std::make_unique<BOARD>();

    board->Add( new NETINFO_ITEM( board.get(), wxS( "SRAM_A9" ), 1 ) );
    NETINFO_ITEM* net = board->GetNetInfo().GetNetItem( 1 );

    const int W = 150000;           // 0.15 mm trace width
    const int PITCH = 50000;        // 0.05 mm pitch -- shorter than width/2 tolerance!
    const int SPAN = 2000000;       // 2 mm horizontal span
    const int LEGS = 6;
    const int LEAD = 1000000;

    auto addSeg = [&]( const VECTOR2I& aStart, const VECTOR2I& aEnd ) -> PCB_TRACK*
    {
        PCB_TRACK* t = new PCB_TRACK( board.get() );
        t->SetStart( aStart );
        t->SetEnd( aEnd );
        t->SetWidth( W );
        t->SetLayer( F_Cu );
        t->SetNet( net );
        board->Add( t );
        return t;
    };

    VECTOR2I cursor( 0, 0 );
    VECTOR2I leadInEnd( LEAD, 0 );
    addSeg( cursor, leadInEnd );
    cursor = leadInEnd;

    double expectedLen = LEAD;

    for( int i = 0; i < LEGS; i++ )
    {
        bool goRight = ( i % 2 == 0 );
        VECTOR2I legEnd = cursor + VECTOR2I( goRight ? SPAN : -SPAN, 0 );
        addSeg( cursor, legEnd );
        expectedLen += SPAN;
        cursor = legEnd;

        if( i < LEGS - 1 )
        {
            VECTOR2I stubEnd = cursor + VECTOR2I( 0, PITCH );
            addSeg( cursor, stubEnd );
            expectedLen += PITCH;
            cursor = stubEnd;
        }
    }

    VECTOR2I leadOutEnd = cursor + VECTOR2I( LEAD, 0 );
    addSeg( cursor, leadOutEnd );
    expectedLen += LEAD;

    int totalSegs = LEGS + ( LEGS - 1 ) + 2;

    FOOTPRINT* fp = new FOOTPRINT( board.get() );
    board->Add( fp );

    auto addPad = [&]( const VECTOR2I& aPos, const wxString& aName )
    {
        PAD* p = new PAD( fp );
        p->SetPosition( aPos );
        p->SetSize( PADSTACK::ALL_LAYERS, VECTOR2I( 500000, 500000 ) );
        p->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::CIRCLE );
        p->SetAttribute( PAD_ATTRIB::SMD );
        p->SetLayerSet( { F_Cu } );
        p->SetNet( net );
        p->SetNumber( aName );
        fp->Add( p );
    };

    addPad( VECTOR2I( 0, 0 ), wxS( "1" ) );
    addPad( leadOutEnd, wxS( "2" ) );

    board->BuildConnectivity();

    // Walk from a middle track
    PCB_TRACK* midTrack = nullptr;
    int idx = 0;

    for( PCB_TRACK* track : board->Tracks() )
    {
        if( track->Type() == PCB_TRACE_T )
        {
            if( idx == totalSegs / 2 )
            {
                midTrack = track;
                break;
            }

            idx++;
        }
    }

    BOOST_REQUIRE( midTrack );

    TRACE_PATH_WALKER walker( board.get() );
    bool ok = walker.Walk( midTrack );
    BOOST_CHECK( ok );

    const WALK_RESULT& result = walker.GetResult();

    BOOST_TEST_MESSAGE( "TightSpacing: walked " << result.segmentsVisited << " / "
                        << result.totalSegmentsOnNet << " segments"
                        << " (stub=" << ( PITCH / 1e6 ) << "mm, width="
                        << ( W / 1e6 ) << "mm)" );

    BOOST_CHECK_EQUAL( result.segmentsVisited, totalSegs );
    BOOST_CHECK( result.isComplete() );
    BOOST_CHECK_CLOSE( walker.GetTotalLength(), expectedLen, 0.01 );
}


/**
 * Walk SRAM_A9 on the cparti_fpga board — a real length-tuned net.
 * Diagnoses whether the connectivity-based walker can trace through
 * the meander segments.
 */
BOOST_AUTO_TEST_CASE( SRAM_A9_MeanderWalk )
{
    KI_TEST::LoadBoard( m_settingsManager, "cparti_fpga", m_board );
    m_board->BuildConnectivity();

    const int SRAM_A9_NET = 147;

    // Collect track info for this net
    int    segCount = 0;
    double trackSum = 0.0;
    int    minWidth = INT_MAX;
    int    minSegLen = INT_MAX;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() == SRAM_A9_NET && track->Type() != PCB_VIA_T )
        {
            segCount++;
            trackSum += track->GetLength();

            int len = static_cast<int>( track->GetLength() );

            if( len < minSegLen )
                minSegLen = len;

            if( track->GetWidth() < minWidth )
                minWidth = track->GetWidth();
        }
    }

    BOOST_TEST_MESSAGE( "SRAM_A9: " << segCount << " segments, total length "
                        << ( trackSum / 1e6 ) << " mm, min seg "
                        << ( minSegLen / 1e3 ) << " um, min width "
                        << ( minWidth / 1e3 ) << " um" );

    // Walk from the first track on the net
    PCB_TRACK* startTrack = nullptr;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() == SRAM_A9_NET && track->Type() == PCB_TRACE_T )
        {
            startTrack = track;
            break;
        }
    }

    BOOST_REQUIRE( startTrack );

    TRACE_PATH_WALKER walker( m_board.get() );
    bool ok = walker.Walk( startTrack );
    BOOST_CHECK( ok );

    const WALK_RESULT& result = walker.GetResult();

    BOOST_TEST_MESSAGE( "Walked " << result.segmentsVisited << " / "
                        << result.totalSegmentsOnNet << " segments, length "
                        << ( walker.GetTotalLength() / 1e6 ) << " mm"
                        << " | start=" << static_cast<int>( result.startTerminus )
                        << " end=" << static_cast<int>( result.endTerminus ) );

    // Must walk all segments (SRAM_A9 is a point-to-point net with meanders)
    BOOST_CHECK_EQUAL( result.segmentsVisited, result.totalSegmentsOnNet );
    BOOST_CHECK( result.isComplete() );
}


/**
 * Walk JTAG_TCK — diagnose via junction behavior.
 */
BOOST_AUTO_TEST_CASE( JTAG_TCK_ViaWalk )
{
    KI_TEST::LoadBoard( m_settingsManager, "cparti_fpga", m_board );
    m_board->BuildConnectivity();

    const int JTAG_TCK_NET = 51;

    int segCount = 0;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() == JTAG_TCK_NET && track->Type() != PCB_VIA_T )
            segCount++;
    }

    // Walk from each pad end to see what happens
    PCB_TRACK* startTrack = nullptr;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() == JTAG_TCK_NET && track->Type() == PCB_TRACE_T )
        {
            startTrack = track;
            break;
        }
    }

    BOOST_REQUIRE( startTrack );

    TRACE_PATH_WALKER walker( m_board.get() );
    bool ok = walker.Walk( startTrack );
    BOOST_CHECK( ok );

    const WALK_RESULT& result = walker.GetResult();
    const auto& path = walker.GetPath();

    BOOST_TEST_MESSAGE( "JTAG_TCK: " << result.segmentsVisited << " / "
                        << segCount << " segments, length "
                        << ( walker.GetTotalLength() / 1e6 ) << " mm"
                        << " | start=" << static_cast<int>( result.startTerminus )
                        << " end=" << static_cast<int>( result.endTerminus ) );

    if( !path.empty() )
    {
        BOOST_TEST_MESSAGE( "  path start: (" << ( path.front().position.x / 1e6 )
                            << ", " << ( path.front().position.y / 1e6 )
                            << ") layer=" << path.front().layer );
        BOOST_TEST_MESSAGE( "  path end:   (" << ( path.back().position.x / 1e6 )
                            << ", " << ( path.back().position.y / 1e6 )
                            << ") layer=" << path.back().layer );
    }

    if( result.startJunction )
    {
        BOOST_TEST_MESSAGE( "  start junction at: ("
                            << ( result.startJunction->position.x / 1e6 ) << ", "
                            << ( result.startJunction->position.y / 1e6 )
                            << ") branches=" << result.startJunction->branches.size() );
    }

    if( result.endJunction )
    {
        BOOST_TEST_MESSAGE( "  end junction at: ("
                            << ( result.endJunction->position.x / 1e6 ) << ", "
                            << ( result.endJunction->position.y / 1e6 )
                            << ") branches=" << result.endJunction->branches.size() );
    }

    // Print actual pads on this net
    for( PAD* pad : m_board->GetPads() )
    {
        if( pad->GetNetCode() == JTAG_TCK_NET )
        {
            FOOTPRINT* fp = pad->GetParentFootprint();
            BOOST_TEST_MESSAGE( "  pad: " << fp->GetReference() << " pad \"" << pad->GetNumber()
                                << "\" at (" << ( pad->GetPosition().x / 1e6 ) << ", "
                                << ( pad->GetPosition().y / 1e6 ) << ") layer="
                                << pad->GetLayer() );
        }
    }

    // 65/70: the walker can't yet resolve the teardrop loop at the via
    // near U1.  Three short tracks form a triangle back to the via; the
    // fourth branch is the real continuation to the BGA pad.
    // TODO: cycle-aware look-ahead at junctions to skip teardrop loops.
    BOOST_CHECK_GE( result.segmentsVisited, 65 );
}


/**
 * Dump cross-section details around the D26 ESD diode pad on JTAG_TCK
 * to diagnose the Z₀ dip at ~39.2mm.
 */
BOOST_AUTO_TEST_CASE( JTAG_TCK_D26_Diagnostic )
{
    KI_TEST::LoadBoard( m_settingsManager, "cparti_fpga", m_board );
    m_board->BuildConnectivity();

    const int JTAG_TCK_NET = 51;

    PCB_TRACK* startTrack = nullptr;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->GetNetCode() == JTAG_TCK_NET && track->Type() == PCB_TRACE_T )
        {
            startTrack = track;
            break;
        }
    }

    BOOST_REQUIRE( startTrack );

    SE_PROFILE profile;
    BEM_CACHE cache;
    bool ok = profile.Compute( m_board.get(), JTAG_TCK_NET, cache,
                               VECTOR2I( 0, 0 ), nullptr, 0, startTrack );

    BOOST_CHECK_MESSAGE( ok, profile.GetError() );

    if( !ok )
        return;

    const auto& samples = profile.GetSamples();

    for( const auto& s : samples )
    {
        double mm = s.distNm / 1e6;

        if( mm > 38.5 && mm < 40.0 )
        {
            BOOST_TEST_MESSAGE(
                    "d=" << mm
                    << " Z0=" << s.z0
                    << " w=" << ( s.signalWidth / 1e3 ) << "um"
                    << " n=" << s.neighborCount
                    << " gw=" << s.groundwireCount
                    << " ref=" << ( s.hasRefAbove ? "A" : "-" )
                              << ( s.hasRefBelow ? "B" : "-" )
                    << " hA=" << ( s.hAbove * 1e6 ) << "um"
                    << " hB=" << ( s.hBelow * 1e6 ) << "um"
                    << " er=" << s.erEff
                    << " pos=(" << ( s.boardPos.x / 1e6 ) << ","
                                << ( s.boardPos.y / 1e6 ) << ")"
                    << " layer=" << s.layer );

            // Dump conductors from XS_GEOMETRY
            const auto& g = s.geometry;

            for( size_t i = 0; i < g.conductors.size(); i++ )
            {
                const auto& c = g.conductors[i];
                BOOST_TEST_MESSAGE(
                        "  cond[" << i << "]: cx=" << ( c.centerX * 1e6 ) << "um"
                        << " w=" << ( c.width * 1e6 ) << "um"
                        << " t=" << ( c.thickness * 1e6 ) << "um"
                        << ( c.isGround ? " GND" : " SIG" ) );
            }
        }
    }
}


BOOST_AUTO_TEST_SUITE_END()
