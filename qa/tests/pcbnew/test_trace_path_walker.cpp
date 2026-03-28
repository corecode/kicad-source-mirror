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


BOOST_AUTO_TEST_SUITE_END()
