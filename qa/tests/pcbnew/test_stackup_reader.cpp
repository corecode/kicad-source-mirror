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

#include <board.h>
#include <board_design_settings.h>
#include <board_stackup_manager/board_stackup.h>
#include <zone.h>
#include <netinfo.h>
#include <pcb_track.h>

#include <sipi/stackup_reader.h>


/**
 * Test fixture that creates a 2-layer board with a default stackup
 * and provides helpers to add zones and tracks.
 */
struct STACKUP_READER_FIXTURE
{
    STACKUP_READER_FIXTURE() :
            m_board( std::make_unique<BOARD>() )
    {
        // Register nets
        m_board->Add( new NETINFO_ITEM( m_board.get(), wxS( "GND" ), 1 ) );
        m_board->Add( new NETINFO_ITEM( m_board.get(), wxS( "SIG" ), 2 ) );
    }

    void setupStackup( int aCopperLayers )
    {
        BOARD_DESIGN_SETTINGS& bds = m_board->GetDesignSettings();
        bds.SetCopperLayerCount( aCopperLayers );
        bds.SetBoardThickness( pcbIUScale.mmToIU( 1.6 ) );

        BOARD_STACKUP& stackup = bds.GetStackupDescriptor();
        stackup.RemoveAll();
        stackup.BuildDefaultStackupList( &bds, aCopperLayers );
    }

    void addZoneFill( const VECTOR2I& aTopLeft, const VECTOR2I& aBotRight,
                      PCB_LAYER_ID aLayer, int aNetCode )
    {
        ZONE* z = new ZONE( m_board.get() );
        z->SetLayer( aLayer );
        z->SetNet( m_board->GetNetInfo().GetNetItem( aNetCode ) );
        z->SetIsFilled( true );
        z->SetFillFlag( aLayer, true );

        SHAPE_POLY_SET fillPoly;
        fillPoly.NewOutline();
        fillPoly.Append( aTopLeft );
        fillPoly.Append( VECTOR2I( aBotRight.x, aTopLeft.y ) );
        fillPoly.Append( aBotRight );
        fillPoly.Append( VECTOR2I( aTopLeft.x, aBotRight.y ) );

        z->SetFilledPolysList( aLayer, fillPoly );
        m_board->Add( z );
    }

    std::unique_ptr<BOARD> m_board;
};


BOOST_FIXTURE_TEST_SUITE( StackupReader, STACKUP_READER_FIXTURE )


/**
 * 2-layer board with a ground zone on B.Cu covering the full board.
 * Signal on F.Cu should see hasRefBelow=true.
 */
BOOST_AUTO_TEST_CASE( TwoLayerWithGroundZone )
{
    setupStackup( 2 );

    // Full-board ground zone on B.Cu
    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 50000000, 50000000 ), B_Cu, 1 );

    STACKUP_READER reader( m_board.get() );
    LAYER_GEOMETRY geom = reader.GetLayerGeometry( F_Cu, VECTOR2I( 25000000, 25000000 ),
                                                   150000 ); // 0.15mm trace

    BOOST_CHECK( geom.hasRefBelow );
    BOOST_CHECK( !geom.hasRefAbove ); // F.Cu is the top layer
    BOOST_CHECK_GT( geom.hBelow, 0.0 );

    BOOST_TEST_MESSAGE( "hBelow=" << geom.hBelow * 1e3 << "mm  erBelow=" << geom.erBelow );
}


/**
 * 2-layer board with NO zones at all.
 * Virtual earth fallback: hasRefBelow=true with large h and εr=1.0.
 */
BOOST_AUTO_TEST_CASE( TwoLayerNoZones )
{
    setupStackup( 2 );

    STACKUP_READER reader( m_board.get() );
    LAYER_GEOMETRY geom = reader.GetLayerGeometry( F_Cu, VECTOR2I( 25000000, 25000000 ),
                                                   150000 );

    // Fallback: use outermost copper layer (B.Cu) with actual dielectric stackup
    BOOST_CHECK( geom.hasRefBelow );
    BOOST_CHECK_GT( geom.hBelow, 0.5e-3 ); // actual F.Cu-to-B.Cu distance
    BOOST_CHECK_GT( geom.erBelow, 1.0 );   // real substrate εr, not air

    // F.Cu is the top layer — no layers above
    BOOST_CHECK( !geom.hasRefAbove );
}


/**
 * 2-layer board with a ground zone that has a gap (antipad).
 * Query inside zone → hasRefBelow=true at normal distance.
 * Query in the gap → hasRefBelow=true via virtual earth (large h).
 * The gap also produces an intermediate layer for groundwire detection.
 */
BOOST_AUTO_TEST_CASE( TwoLayerAntipad )
{
    setupStackup( 2 );

    // Zone covering left half of board
    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 20000000, 50000000 ), B_Cu, 1 );

    // Zone covering right half of board (gap from 20mm to 25mm)
    addZoneFill( VECTOR2I( 25000000, 0 ), VECTOR2I( 50000000, 50000000 ), B_Cu, 1 );

    STACKUP_READER reader( m_board.get() );

    // Inside left zone — real reference plane
    LAYER_GEOMETRY geomInside = reader.GetLayerGeometry( F_Cu, VECTOR2I( 10000000, 25000000 ),
                                                         150000 );
    BOOST_CHECK( geomInside.hasRefBelow );
    BOOST_CHECK_LT( geomInside.hBelow, 5e-3 ); // normal stackup distance

    // In the gap between zones — fallback to outermost layer (B.Cu)
    LAYER_GEOMETRY geomGap = reader.GetLayerGeometry( F_Cu, VECTOR2I( 22000000, 25000000 ),
                                                      150000 );
    BOOST_CHECK( geomGap.hasRefBelow );
    BOOST_CHECK_GT( geomGap.hBelow, 0.5e-3 ); // actual board thickness

    // The gap point should have B.Cu as an intermediate layer (for groundwires)
    BOOST_CHECK_EQUAL( (int) geomGap.intermediateLayers.size(), 1 );

    // Inside right zone — real reference plane
    LAYER_GEOMETRY geomRight = reader.GetLayerGeometry( F_Cu, VECTOR2I( 35000000, 25000000 ),
                                                        150000 );
    BOOST_CHECK( geomRight.hasRefBelow );
    BOOST_CHECK_LT( geomRight.hBelow, 5e-3 );
}


/**
 * 4-layer board: F.Cu / In1.Cu / In2.Cu / B.Cu.
 * In1.Cu has no zone, In2.Cu has a full zone.
 * Signal on F.Cu should find the reference on In2.Cu (skipping In1.Cu),
 * with a correspondingly larger dielectric height.
 */
BOOST_AUTO_TEST_CASE( FourLayerSkipToNextRef )
{
    setupStackup( 4 );

    // Zone only on In2.Cu (skip In1.Cu which has no zone)
    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 50000000, 50000000 ), In2_Cu, 1 );

    STACKUP_READER reader( m_board.get() );
    LAYER_GEOMETRY geom = reader.GetLayerGeometry( F_Cu, VECTOR2I( 25000000, 25000000 ),
                                                   150000 );

    BOOST_CHECK( geom.hasRefBelow );
    BOOST_CHECK( !geom.hasRefAbove );

    // The distance should be to In2.Cu (2 dielectric layers away), not In1.Cu
    // Default stackup for 4 layers: ~1.6mm total, so one inter-layer gap is ~0.5mm.
    // Two gaps ≈ 1.0mm.
    BOOST_TEST_MESSAGE( "hBelow (skip to In2)=" << geom.hBelow * 1e3 << "mm" );

    // Also check with zone on In1.Cu — should find the nearer reference
    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 50000000, 50000000 ), In1_Cu, 1 );

    // Need a fresh reader since buildLayerModel caches
    STACKUP_READER reader2( m_board.get() );
    LAYER_GEOMETRY geom2 = reader2.GetLayerGeometry( F_Cu, VECTOR2I( 25000000, 25000000 ),
                                                     150000 );

    BOOST_CHECK( geom2.hasRefBelow );
    BOOST_CHECK_LT( geom2.hBelow, geom.hBelow ); // nearer reference = smaller h

    BOOST_TEST_MESSAGE( "hBelow (In1)=" << geom2.hBelow * 1e3 << "mm" );
}


/**
 * 4-layer board with a stripline signal on In1.Cu.
 * Zones on F.Cu (above) and B.Cu (below).
 */
BOOST_AUTO_TEST_CASE( StriplineRefPlanes )
{
    setupStackup( 4 );

    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 50000000, 50000000 ), F_Cu, 1 );
    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 50000000, 50000000 ), B_Cu, 1 );

    STACKUP_READER reader( m_board.get() );
    LAYER_GEOMETRY geom = reader.GetLayerGeometry( In1_Cu, VECTOR2I( 25000000, 25000000 ),
                                                   150000 );

    BOOST_CHECK( geom.hasRefAbove );
    BOOST_CHECK( geom.hasRefBelow );
    BOOST_CHECK_GT( geom.hAbove, 0.0 );
    BOOST_CHECK_GT( geom.hBelow, 0.0 );

    BOOST_TEST_MESSAGE( "Stripline: hAbove=" << geom.hAbove * 1e3
                        << "mm  hBelow=" << geom.hBelow * 1e3 << "mm" );
}


/**
 * 4-layer stripline where the ground zone above has a gap at the query point.
 * In the gap: virtual earth above (large h), F.Cu becomes an intermediate layer
 * (groundwire candidate).  B.Cu ref below is found normally.
 */
BOOST_AUTO_TEST_CASE( StriplineWithGapAbove )
{
    setupStackup( 4 );

    // F.Cu zone with a gap around x=25mm (only covers 0-20mm and 30-50mm)
    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 20000000, 50000000 ), F_Cu, 1 );
    addZoneFill( VECTOR2I( 30000000, 0 ), VECTOR2I( 50000000, 50000000 ), F_Cu, 1 );

    // B.Cu full zone
    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 50000000, 50000000 ), B_Cu, 1 );

    STACKUP_READER reader( m_board.get() );

    // Query inside the F.Cu gap → virtual earth above, real ref below
    LAYER_GEOMETRY geomGap = reader.GetLayerGeometry( In1_Cu, VECTOR2I( 25000000, 25000000 ),
                                                      150000 );

    BOOST_CHECK( geomGap.hasRefAbove );
    // Fallback to outermost layer above (F.Cu) — actual stackup distance
    BOOST_CHECK( geomGap.hasRefBelow );
    BOOST_CHECK_LT( geomGap.hBelow, 5e-3 ); // real ref on B.Cu

    // F.Cu should be an intermediate layer (groundwire candidate)
    BOOST_CHECK_GE( (int) geomGap.intermediateLayers.size(), 1 );

    BOOST_TEST_MESSAGE( "Gap: hAbove=" << geomGap.hAbove * 1e3
                        << "mm  hBelow=" << geomGap.hBelow * 1e3
                        << "mm  intermediates=" << geomGap.intermediateLayers.size() );

    // Query inside the F.Cu zone → both real refs
    LAYER_GEOMETRY geomCovered = reader.GetLayerGeometry( In1_Cu, VECTOR2I( 10000000, 25000000 ),
                                                          150000 );

    BOOST_CHECK( geomCovered.hasRefAbove );
    BOOST_CHECK_LT( geomCovered.hAbove, 5e-3 ); // real ref on F.Cu
    BOOST_CHECK( geomCovered.hasRefBelow );
}


BOOST_AUTO_TEST_SUITE_END()
