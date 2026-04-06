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
#include <footprint.h>
#include <pcb_shape.h>
#include <zone.h>
#include <netinfo.h>
#include <pcb_track.h>

#include <sipi/stackup_reader.h>

#include <chrono>


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


/**
 * 2-layer board: solder mask fields should be populated for outer layers.
 * F.Cu gets solderMaskAbove=true, B.Cu gets solderMaskAbove=false.
 * Inner layers (if any) get solderMaskThickness=0.
 */
BOOST_AUTO_TEST_CASE( SolderMaskOnOuterLayers )
{
    setupStackup( 2 );

    // Full-board ground zone on B.Cu
    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 50000000, 50000000 ), B_Cu, 1 );

    STACKUP_READER reader( m_board.get() );

    // F.Cu — should have solder mask above
    LAYER_GEOMETRY geomF = reader.GetLayerGeometry( F_Cu, VECTOR2I( 25000000, 25000000 ),
                                                     150000 );
    BOOST_CHECK_GT( geomF.solderMaskThickness, 0.0 );
    BOOST_CHECK( geomF.solderMaskAbove );
    BOOST_CHECK_CLOSE( geomF.solderMaskEr, 3.3, 1.0 );

    BOOST_TEST_MESSAGE( "F.Cu SM: thickness=" << geomF.solderMaskThickness * 1e6
                        << "um  er=" << geomF.solderMaskEr );

    // B.Cu — should have solder mask below
    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 50000000, 50000000 ), F_Cu, 1 );

    STACKUP_READER reader2( m_board.get() );
    LAYER_GEOMETRY geomB = reader2.GetLayerGeometry( B_Cu, VECTOR2I( 25000000, 25000000 ),
                                                      150000 );
    BOOST_CHECK_GT( geomB.solderMaskThickness, 0.0 );
    BOOST_CHECK( !geomB.solderMaskAbove );
    BOOST_CHECK_CLOSE( geomB.solderMaskEr, 3.3, 1.0 );

    BOOST_TEST_MESSAGE( "B.Cu SM: thickness=" << geomB.solderMaskThickness * 1e6
                        << "um  er=" << geomB.solderMaskEr );
}


/**
 * 2-layer board with a filled rectangle on F.Mask creating a solder mask opening.
 * Inside the opening → solderMaskThickness=0.  Outside → normal SM.
 */
BOOST_AUTO_TEST_CASE( SolderMaskOpening )
{
    setupStackup( 2 );

    // Ground zone on B.Cu
    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 50000000, 50000000 ), B_Cu, 1 );

    // Mask opening: filled rectangle on F.Mask from (10mm,10mm) to (20mm,20mm)
    PCB_SHAPE* maskCutout = new PCB_SHAPE( m_board.get() );
    maskCutout->SetShape( SHAPE_T::RECTANGLE );
    maskCutout->SetStart( VECTOR2I( 10000000, 10000000 ) );
    maskCutout->SetEnd( VECTOR2I( 20000000, 20000000 ) );
    maskCutout->SetFilled( true );
    maskCutout->SetLayer( F_Mask );
    m_board->Add( maskCutout );

    STACKUP_READER reader( m_board.get() );

    // Inside the mask opening → no solder mask
    LAYER_GEOMETRY geomOpen = reader.GetLayerGeometry( F_Cu, VECTOR2I( 15000000, 15000000 ),
                                                        150000 );
    BOOST_CHECK_EQUAL( geomOpen.solderMaskThickness, 0.0 );

    // Outside the mask opening → normal solder mask
    STACKUP_READER reader2( m_board.get() );
    LAYER_GEOMETRY geomCovered = reader2.GetLayerGeometry( F_Cu, VECTOR2I( 30000000, 30000000 ),
                                                            150000 );
    BOOST_CHECK_GT( geomCovered.solderMaskThickness, 0.0 );
    BOOST_CHECK( geomCovered.solderMaskAbove );

    BOOST_TEST_MESSAGE( "Inside opening: SM=" << geomOpen.solderMaskThickness * 1e6 << "um"
                        << "  Outside: SM=" << geomCovered.solderMaskThickness * 1e6 << "um" );
}


/**
 * 4-layer board: inner layers should NOT have solder mask.
 */
BOOST_AUTO_TEST_CASE( NoSolderMaskOnInnerLayers )
{
    setupStackup( 4 );

    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 50000000, 50000000 ), F_Cu, 1 );
    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 50000000, 50000000 ), B_Cu, 1 );

    STACKUP_READER reader( m_board.get() );
    LAYER_GEOMETRY geom = reader.GetLayerGeometry( In1_Cu, VECTOR2I( 25000000, 25000000 ),
                                                    150000 );

    BOOST_CHECK_EQUAL( geom.solderMaskThickness, 0.0 );
}


/**
 * Performance test: GetLayerGeometry with many footprints and mask layer items.
 * Simulates a realistic board with 200 footprints (each with graphics on F.Mask)
 * and measures the time for 500 GetLayerGeometry calls.
 */
BOOST_AUTO_TEST_CASE( SolderMaskPerformance )
{
    setupStackup( 2 );

    // Ground zone on B.Cu
    addZoneFill( VECTOR2I( 0, 0 ), VECTOR2I( 100000000, 100000000 ), B_Cu, 1 );

    // Add 200 footprints with graphics on F.Mask (courtyard-like items)
    for( int i = 0; i < 200; i++ )
    {
        FOOTPRINT* fp = new FOOTPRINT( m_board.get() );
        int x = ( i % 20 ) * 5000000;
        int y = ( i / 20 ) * 5000000;
        fp->SetPosition( VECTOR2I( x, y ) );

        // Add a graphic on F.Mask (simulates mask opening in footprint)
        PCB_SHAPE* shape = new PCB_SHAPE( fp );
        shape->SetShape( SHAPE_T::RECTANGLE );
        shape->SetStart( VECTOR2I( x - 500000, y - 500000 ) );
        shape->SetEnd( VECTOR2I( x + 500000, y + 500000 ) );
        shape->SetFilled( true );
        shape->SetLayer( F_Mask );
        fp->Add( shape );

        m_board->Add( fp );
    }

    // Also add 50 board-level drawings on F.Mask
    for( int i = 0; i < 50; i++ )
    {
        PCB_SHAPE* shape = new PCB_SHAPE( m_board.get() );
        shape->SetShape( SHAPE_T::RECTANGLE );
        int x = ( i % 10 ) * 10000000;
        int y = ( i / 10 ) * 10000000 + 60000000;
        shape->SetStart( VECTOR2I( x, y ) );
        shape->SetEnd( VECTOR2I( x + 1000000, y + 1000000 ) );
        shape->SetFilled( true );
        shape->SetLayer( F_Mask );
        m_board->Add( shape );
    }

    STACKUP_READER reader( m_board.get() );

    // Time 500 GetLayerGeometry calls at various positions
    auto start = std::chrono::high_resolution_clock::now();

    for( int i = 0; i < 500; i++ )
    {
        int x = ( i * 200000 ) % 100000000;
        int y = 25000000 + ( i * 100000 ) % 50000000;
        reader.GetLayerGeometry( F_Cu, VECTOR2I( x, y ), 150000 );
    }

    auto end = std::chrono::high_resolution_clock::now();
    double elapsedMs = std::chrono::duration<double, std::milli>( end - start ).count();

    BOOST_TEST_MESSAGE( "500 GetLayerGeometry calls with 200 FPs + 50 drawings: "
                        << elapsedMs << "ms ("
                        << ( elapsedMs / 500.0 ) << " ms/call)" );

    // Should complete in well under 1 second (target: < 0.1ms per call)
    BOOST_CHECK_LT( elapsedMs, 500.0 );
}


BOOST_AUTO_TEST_SUITE_END()
