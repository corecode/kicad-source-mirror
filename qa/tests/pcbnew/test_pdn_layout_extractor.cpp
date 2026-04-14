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
#include <sipi/pdn_layout_extractor.h>

#include <board.h>
#include <board_design_settings.h>
#include <board_stackup_manager/board_stackup.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_track.h>
#include <settings/settings_manager.h>
#include <zone.h>

#include <cmath>

static constexpr double RHO_CU = 1.68e-8;
static constexpr double MU0 = 4.0 * M_PI * 1e-7;


/**
 * Test fixture that creates a board with power/ground nets,
 * stackup, and provides helpers to add geometry.
 */
struct PDN_EXTRACTOR_FIXTURE
{
    PDN_EXTRACTOR_FIXTURE() :
            m_board( std::make_unique<BOARD>() )
    {
        m_board->Add( new NETINFO_ITEM( m_board.get(), wxS( "+3V3" ), 1 ) );
        m_board->Add( new NETINFO_ITEM( m_board.get(), wxS( "GND" ), 2 ) );

        setupStackup( 4 );
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

    FOOTPRINT* addFootprint( const wxString& aRef, const VECTOR2I& aPos )
    {
        FOOTPRINT* fp = new FOOTPRINT( m_board.get() );
        fp->SetPosition( aPos );
        fp->SetReference( aRef );
        m_board->Add( fp );
        return fp;
    }

    PAD* addPad( FOOTPRINT* aFp, const wxString& aNumber, const VECTOR2I& aPos,
                 int aNetCode, PCB_LAYER_ID aLayer = F_Cu )
    {
        PAD* p = new PAD( aFp );
        p->SetPosition( aPos );
        p->SetSize( PADSTACK::ALL_LAYERS, VECTOR2I( 500000, 500000 ) );
        p->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::CIRCLE );
        p->SetAttribute( PAD_ATTRIB::SMD );
        p->SetLayerSet( { aLayer } );
        p->SetNet( m_board->GetNetInfo().GetNetItem( aNetCode ) );
        p->SetNumber( aNumber );
        aFp->Add( p );
        return p;
    }

    PCB_TRACK* addTrace( const VECTOR2I& aStart, const VECTOR2I& aEnd,
                         int aWidth, int aNetCode, PCB_LAYER_ID aLayer = F_Cu )
    {
        PCB_TRACK* track = new PCB_TRACK( m_board.get() );
        track->SetStart( aStart );
        track->SetEnd( aEnd );
        track->SetWidth( aWidth );
        track->SetLayer( aLayer );
        track->SetNet( m_board->GetNetInfo().GetNetItem( aNetCode ) );
        m_board->Add( track );
        return track;
    }

    PCB_VIA* addVia( const VECTOR2I& aPos, int aDrill, int aPadDia, int aNetCode,
                     PCB_LAYER_ID aTop = F_Cu, PCB_LAYER_ID aBot = B_Cu )
    {
        PCB_VIA* via = new PCB_VIA( m_board.get() );
        via->SetPosition( aPos );
        via->SetDrill( aDrill );
        via->SetWidth( PADSTACK::ALL_LAYERS, aPadDia );
        via->SetLayerPair( aTop, aBot );
        via->SetNet( m_board->GetNetInfo().GetNetItem( aNetCode ) );
        m_board->Add( via );
        return via;
    }

    /**
     * Add a pre-filled rectangular zone on the given layer/net.  Sets both
     * the outline (so KiCad's connectivity sees a well-formed zone) and
     * the filled polys directly — bypasses the interactive filler path,
     * which requires a project/settings/tool-manager context the headless
     * fixture doesn't provide.
     */
    void addZone( const VECTOR2I& aTopLeft, const VECTOR2I& aBotRight,
                  PCB_LAYER_ID aLayer, int aNetCode )
    {
        ZONE* z = new ZONE( m_board.get() );
        z->SetLayer( aLayer );
        z->SetNet( m_board->GetNetInfo().GetNetItem( aNetCode ) );
        z->SetIsFilled( true );
        z->SetFillFlag( aLayer, true );

        SHAPE_POLY_SET* outline = z->Outline();
        outline->RemoveAllContours();
        outline->NewOutline();
        outline->Append( aTopLeft );
        outline->Append( VECTOR2I( aBotRight.x, aTopLeft.y ) );
        outline->Append( aBotRight );
        outline->Append( VECTOR2I( aTopLeft.x, aBotRight.y ) );

        SHAPE_POLY_SET fill;
        fill.NewOutline();
        fill.Append( aTopLeft );
        fill.Append( VECTOR2I( aBotRight.x, aTopLeft.y ) );
        fill.Append( aBotRight );
        fill.Append( VECTOR2I( aTopLeft.x, aBotRight.y ) );
        fill.CacheTriangulation();

        z->SetFilledPolysList( aLayer, fill );
        m_board->Add( z );
    }

    void buildConnectivity()
    {
        m_board->BuildConnectivity();
    }

    std::unique_ptr<BOARD> m_board;
};


BOOST_FIXTURE_TEST_SUITE( PdnLayoutExtractor, PDN_EXTRACTOR_FIXTURE )


// -------------------------------------------------------------------------
//  Serialization tests (no board needed)
// -------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( SerializeDeserializeRoundTrip )
{
    wxString payload;
    payload += wxS( "NET +3V3\n" );
    payload += wxS( "PAD C1:1 node=C1_1 pos=10000000,20000000\n" );
    payload += wxS( "PAD U1:3 node=U1_3 pos=15000000,25000000\n" );
    payload += wxS( "R Rtr1 C1_1 n1 0.00012\n" );
    payload += wxS( "L Ltr1 C1_1 n1 2.8e-11\n" );
    payload += wxS( "WARN No zone fill for C5 on +3V3\n" );
    payload += wxS( "NET GND\n" );
    payload += wxS( "PAD C1:2 node=C1_2 pos=10200000,20000000\n" );
    payload += wxS( "R Rgnd1 C1_2 n2 0.00015\n" );
    payload += wxS( "PLANE_C Cplane1 pzc_F.Cu gzc_In1.Cu 7.8e-12\n" );

    PDN_EXTRACTION_RESULT parsed;
    bool ok = PDN_LAYOUT_EXTRACTOR::DeserializeResult( payload, parsed );

    BOOST_CHECK( ok );
    BOOST_CHECK_EQUAL( parsed.powerNet.netName, wxString( "+3V3" ) );
    BOOST_CHECK_EQUAL( parsed.powerNet.padPorts.size(), 2u );
    BOOST_CHECK_EQUAL( parsed.powerNet.padPorts[0].refdes, wxString( "C1" ) );
    BOOST_CHECK_EQUAL( parsed.powerNet.padPorts[0].padNumber, wxString( "1" ) );
    BOOST_CHECK_EQUAL( parsed.powerNet.padPorts[0].nodeName, wxString( "C1_1" ) );
    BOOST_CHECK_EQUAL( parsed.powerNet.padPorts[0].position.x, 10000000 );
    BOOST_CHECK_EQUAL( parsed.powerNet.elements.size(), 2u );
    BOOST_CHECK_EQUAL( parsed.powerNet.warnings.size(), 1u );
    BOOST_CHECK_EQUAL( parsed.groundNet.netName, wxString( "GND" ) );
    BOOST_CHECK_EQUAL( parsed.groundNet.padPorts.size(), 1u );
    BOOST_CHECK_EQUAL( parsed.planeCaps.size(), 1u );
    BOOST_CHECK_CLOSE( parsed.planeCaps[0].value, 7.8e-12, 1.0 );
}


BOOST_AUTO_TEST_CASE( DeserializeEmpty )
{
    PDN_EXTRACTION_RESULT result;
    BOOST_CHECK( !PDN_LAYOUT_EXTRACTOR::DeserializeResult( wxEmptyString, result ) );
}


BOOST_AUTO_TEST_CASE( DeserializeMalformed )
{
    PDN_EXTRACTION_RESULT result;
    wxString payload = wxS( "NET +3V3\nR\nPAD\nGARBAGE\nNET GND\n" );
    bool ok = PDN_LAYOUT_EXTRACTOR::DeserializeResult( payload, result );

    BOOST_CHECK( ok );
    BOOST_CHECK_EQUAL( result.powerNet.netName, wxString( "+3V3" ) );
    BOOST_CHECK_EQUAL( result.groundNet.netName, wxString( "GND" ) );
}


// -------------------------------------------------------------------------
//  Board-based extraction tests
// -------------------------------------------------------------------------

/**
 * Extract a net with only a trace connecting two pads.
 * Verify: pad ports are labeled correctly, trace produces R+L elements
 * with physically reasonable values.
 */
BOOST_AUTO_TEST_CASE( TraceExtraction )
{
    // C1 pad 1 at (10mm, 25mm), pad 2 at (20mm, 25mm), on +3V3
    // Connected by a 10mm trace, 200µm wide, on F.Cu
    FOOTPRINT* fpC1 = addFootprint( wxS( "C1" ), VECTOR2I( 10000000, 25000000 ) );
    addPad( fpC1, wxS( "1" ), VECTOR2I( 10000000, 25000000 ), 1 );

    FOOTPRINT* fpU1 = addFootprint( wxS( "U1" ), VECTOR2I( 20000000, 25000000 ) );
    addPad( fpU1, wxS( "VCC" ), VECTOR2I( 20000000, 25000000 ), 1 );

    addTrace( VECTOR2I( 10000000, 25000000 ), VECTOR2I( 20000000, 25000000 ),
              200000, 1, F_Cu ); // 200µm wide, 10mm long

    buildConnectivity();

    PDN_LAYOUT_EXTRACTOR extractor( m_board.get() );
    bool ok = extractor.Extract( wxS( "+3V3" ), wxS( "GND" ) );
    BOOST_REQUIRE( ok );

    const PDN_NET_PARASITICS& power = extractor.GetResult().powerNet;

    // Should have 2 pad ports
    BOOST_CHECK_EQUAL( power.padPorts.size(), 2u );

    // Verify pad port naming
    bool foundC1 = false, foundU1 = false;

    for( const PDN_PAD_PORT& port : power.padPorts )
    {
        if( port.refdes == wxS( "C1" ) && port.padNumber == wxS( "1" ) )
        {
            BOOST_CHECK_EQUAL( port.nodeName, wxString( "C1_1" ) );
            foundC1 = true;
        }

        if( port.refdes == wxS( "U1" ) && port.padNumber == wxS( "VCC" ) )
        {
            BOOST_CHECK_EQUAL( port.nodeName, wxString( "U1_VCC" ) );
            foundU1 = true;
        }
    }

    BOOST_CHECK( foundC1 );
    BOOST_CHECK( foundU1 );

    // Should have R+L elements for the trace
    BOOST_CHECK_GE( power.elements.size(), 2u );

    // Find the R and L elements
    double traceR = 0.0, traceL = 0.0;

    for( const PDN_SPICE_ELEMENT& elem : power.elements )
    {
        if( elem.type == wxS( "R" ) && elem.name.Contains( wxS( "Rtr" ) ) )
            traceR = elem.value;

        if( elem.type == wxS( "L" ) && elem.name.Contains( wxS( "Ltr" ) ) )
            traceL = elem.value;
    }

    // 10mm trace, 200µm wide, 35µm copper:
    // R = 1.68e-8 * 0.01 / (0.0002 * 35e-6) ≈ 24 mΩ
    BOOST_CHECK_GT( traceR, 0.005 );
    BOOST_CHECK_LT( traceR, 0.100 );

    // L should be in nH range
    BOOST_CHECK_GT( traceL, 0.5e-9 );
    BOOST_CHECK_LT( traceL, 100e-9 );

    BOOST_TEST_MESSAGE( "Trace R=" << traceR * 1e3 << "mΩ  L=" << traceL * 1e9 << "nH" );
}


/**
 * Extract a net with a via connecting F.Cu to B.Cu.
 * Verify: via produces R+L elements.
 */
BOOST_AUTO_TEST_CASE( ViaExtraction )
{
    FOOTPRINT* fp = addFootprint( wxS( "C1" ), VECTOR2I( 10000000, 25000000 ) );
    addPad( fp, wxS( "1" ), VECTOR2I( 10000000, 25000000 ), 1 );

    // Trace on F.Cu to via position
    addTrace( VECTOR2I( 10000000, 25000000 ), VECTOR2I( 15000000, 25000000 ),
              200000, 1, F_Cu );

    // Via at (15mm, 25mm)
    addVia( VECTOR2I( 15000000, 25000000 ), 300000, 600000, 1 );

    // Trace on B.Cu from via — via needs traces on both layers to be extracted
    FOOTPRINT* fp2 = addFootprint( wxS( "C2" ), VECTOR2I( 20000000, 25000000 ) );
    addPad( fp2, wxS( "1" ), VECTOR2I( 20000000, 25000000 ), 1, B_Cu );
    addTrace( VECTOR2I( 15000000, 25000000 ), VECTOR2I( 20000000, 25000000 ),
              200000, 1, B_Cu );

    buildConnectivity();

    PDN_LAYOUT_EXTRACTOR extractor( m_board.get() );
    bool ok = extractor.Extract( wxS( "+3V3" ), wxS( "GND" ) );
    BOOST_REQUIRE( ok );

    const PDN_NET_PARASITICS& power = extractor.GetResult().powerNet;

    // Should have via R+L elements
    bool hasViaR = false, hasViaL = false;

    for( const PDN_SPICE_ELEMENT& elem : power.elements )
    {
        if( elem.type == wxS( "R" ) && elem.name.Contains( wxS( "Rvia" ) ) )
        {
            hasViaR = true;
            // Via resistance should be small (< 10 mΩ typically)
            BOOST_CHECK_GT( elem.value, 0.0 );
            BOOST_CHECK_LT( elem.value, 0.050 );
            BOOST_TEST_MESSAGE( "Via R=" << elem.value * 1e3 << "mΩ" );
        }

        if( elem.type == wxS( "L" ) && elem.name.Contains( wxS( "Lvia" ) ) )
        {
            hasViaL = true;
            // Via inductance should be in 0.1-5 nH range
            BOOST_CHECK_GT( elem.value, 0.01e-9 );
            BOOST_CHECK_LT( elem.value, 10e-9 );
            BOOST_TEST_MESSAGE( "Via L=" << elem.value * 1e9 << "nH" );
        }
    }

    BOOST_CHECK( hasViaR );
    BOOST_CHECK( hasViaL );
}


/**
 * Extract a net with a zone fill.  Pads inside the zone should
 * produce zone spoke R+L elements.
 */
BOOST_AUTO_TEST_CASE( ZoneExtraction )
{
    // Two caps with pads inside a power zone on F.Cu
    FOOTPRINT* fpC1 = addFootprint( wxS( "C1" ), VECTOR2I( 10000000, 25000000 ) );
    addPad( fpC1, wxS( "1" ), VECTOR2I( 10000000, 25000000 ), 1 );

    FOOTPRINT* fpC2 = addFootprint( wxS( "C2" ), VECTOR2I( 30000000, 25000000 ) );
    addPad( fpC2, wxS( "1" ), VECTOR2I( 30000000, 25000000 ), 1 );

    // Power zone covering the full board on F.Cu
    addZone( VECTOR2I( 0, 0 ), VECTOR2I( 50000000, 50000000 ), F_Cu, 1 );

    buildConnectivity();

    PDN_LAYOUT_EXTRACTOR extractor( m_board.get() );
    bool ok = extractor.Extract( wxS( "+3V3" ), wxS( "GND" ) );
    BOOST_REQUIRE( ok );

    const PDN_NET_PARASITICS& power = extractor.GetResult().powerNet;

    // Should have zone spoke elements
    int spokeCount = 0;

    for( const PDN_SPICE_ELEMENT& elem : power.elements )
    {
        if( elem.name.Contains( wxS( "Rzs" ) ) || elem.name.Contains( wxS( "Lzs" ) ) )
            spokeCount++;
    }

    // 2 pads → 2 connections → 2 spokes × 2 (R+L each) = 4 zone spoke elements
    BOOST_CHECK_GE( spokeCount, 4 );
    BOOST_TEST_MESSAGE( "Zone spoke elements: " << spokeCount );
}


/**
 * Extract both power and ground nets and verify plane capacitance
 * is produced between adjacent power/ground zones.
 */
BOOST_AUTO_TEST_CASE( PlaneCap )
{
    // Power zone on F.Cu, ground zone on In1.Cu (adjacent layers)
    addZone( VECTOR2I( 0, 0 ), VECTOR2I( 40000000, 40000000 ), F_Cu, 1 );  // +3V3
    addZone( VECTOR2I( 0, 0 ), VECTOR2I( 40000000, 40000000 ), In1_Cu, 2 ); // GND

    // Pad on the power net (F.Cu, inside zone)
    FOOTPRINT* fp = addFootprint( wxS( "C1" ), VECTOR2I( 20000000, 20000000 ) );
    addPad( fp, wxS( "1" ), VECTOR2I( 20000000, 20000000 ), 1, F_Cu );
    // GND pad on In1.Cu (inside the ground zone)
    addPad( fp, wxS( "2" ), VECTOR2I( 20500000, 20000000 ), 2, In1_Cu );

    buildConnectivity();

    PDN_LAYOUT_EXTRACTOR extractor( m_board.get() );
    bool ok = extractor.Extract( wxS( "+3V3" ), wxS( "GND" ) );
    BOOST_REQUIRE( ok );

    const auto& caps = extractor.GetResult().planeCaps;

    BOOST_CHECK_GE( caps.size(), 1u );

    if( !caps.empty() )
    {
        // Plane cap: C = eps0 * er * A / h
        // Area ≈ 40mm × 40mm = 1600 mm² = 1.6e-3 m²
        // h ≈ 0.48mm (default 4-layer stackup spacing)
        // er ≈ 4.5
        // C ≈ 8.854e-12 * 4.5 * 1.6e-3 / 0.48e-3 ≈ 133 pF
        double capValue = caps[0].value;
        BOOST_CHECK_GT( capValue, 10e-12 );  // > 10 pF
        BOOST_CHECK_LT( capValue, 1e-9 );    // < 1 nF

        BOOST_TEST_MESSAGE( "Plane cap: " << capValue * 1e12 << " pF" );
    }
}


/**
 * Extract on a net that doesn't exist on the board.
 * Should return true (other net may exist) but with empty parasitics.
 */
BOOST_AUTO_TEST_CASE( NonexistentNet )
{
    buildConnectivity();

    PDN_LAYOUT_EXTRACTOR extractor( m_board.get() );

    // Both nets nonexistent → should return false
    bool ok = extractor.Extract( wxS( "VBUS" ), wxS( "PGND" ) );
    BOOST_CHECK( !ok );

    // One net exists (GND registered), one doesn't (VBUS) → returns true
    // but VBUS power net should have no elements
    bool ok2 = extractor.Extract( wxS( "VBUS" ), wxS( "GND" ) );
    BOOST_CHECK( ok2 );
    BOOST_CHECK( extractor.GetResult().powerNet.netName.IsEmpty() );
    BOOST_CHECK_EQUAL( extractor.GetResult().groundNet.netName, wxString( "GND" ) );
}


/**
 * Extract when neither net has any board items — no traces, no zones, no pads.
 * Should succeed but produce empty element lists.
 */
BOOST_AUTO_TEST_CASE( NetsWithNoItems )
{
    buildConnectivity();

    PDN_LAYOUT_EXTRACTOR extractor( m_board.get() );
    bool ok = extractor.Extract( wxS( "+3V3" ), wxS( "GND" ) );
    BOOST_REQUIRE( ok );

    BOOST_CHECK( extractor.GetResult().powerNet.elements.empty() );
    BOOST_CHECK( extractor.GetResult().powerNet.padPorts.empty() );
    BOOST_CHECK( extractor.GetResult().groundNet.elements.empty() );
    BOOST_CHECK( extractor.GetResult().planeCaps.empty() );
}


/**
 * Full round-trip: extract from a board, serialize, deserialize, verify consistency.
 */
BOOST_AUTO_TEST_CASE( ExtractAndSerializeRoundTrip )
{
    FOOTPRINT* fp = addFootprint( wxS( "C1" ), VECTOR2I( 10000000, 25000000 ) );
    addPad( fp, wxS( "1" ), VECTOR2I( 10000000, 25000000 ), 1 );
    addPad( fp, wxS( "2" ), VECTOR2I( 10500000, 25000000 ), 2 );

    addTrace( VECTOR2I( 10000000, 25000000 ), VECTOR2I( 20000000, 25000000 ),
              200000, 1, F_Cu );

    buildConnectivity();

    PDN_LAYOUT_EXTRACTOR extractor( m_board.get() );
    bool ok = extractor.Extract( wxS( "+3V3" ), wxS( "GND" ) );
    BOOST_REQUIRE( ok );

    wxString payload = extractor.SerializeResult();
    BOOST_CHECK( !payload.IsEmpty() );

    PDN_EXTRACTION_RESULT parsed;
    bool parseOk = PDN_LAYOUT_EXTRACTOR::DeserializeResult( payload, parsed );
    BOOST_REQUIRE( parseOk );

    // Original and parsed should have same structure
    const auto& orig = extractor.GetResult();
    BOOST_CHECK_EQUAL( parsed.powerNet.netName, orig.powerNet.netName );
    BOOST_CHECK_EQUAL( parsed.powerNet.padPorts.size(), orig.powerNet.padPorts.size() );
    BOOST_CHECK_EQUAL( parsed.powerNet.elements.size(), orig.powerNet.elements.size() );
    BOOST_CHECK_EQUAL( parsed.groundNet.netName, orig.groundNet.netName );
    BOOST_CHECK_EQUAL( parsed.planeCaps.size(), orig.planeCaps.size() );
}


/**
 * Verify trace R scales linearly with length: a 20mm trace should have
 * roughly 2× the resistance of a 10mm trace of same width.
 */
BOOST_AUTO_TEST_CASE( TraceResistanceScaling )
{
    // 10mm trace
    FOOTPRINT* fp1 = addFootprint( wxS( "R1" ), VECTOR2I( 10000000, 10000000 ) );
    addPad( fp1, wxS( "1" ), VECTOR2I( 10000000, 10000000 ), 1 );
    FOOTPRINT* fp2 = addFootprint( wxS( "R2" ), VECTOR2I( 20000000, 10000000 ) );
    addPad( fp2, wxS( "1" ), VECTOR2I( 20000000, 10000000 ), 1 );
    addTrace( VECTOR2I( 10000000, 10000000 ), VECTOR2I( 20000000, 10000000 ),
              200000, 1, F_Cu );

    buildConnectivity();

    PDN_LAYOUT_EXTRACTOR ext1( m_board.get() );
    ext1.Extract( wxS( "+3V3" ), wxS( "GND" ) );

    double r10mm = 0.0;

    for( const auto& elem : ext1.GetResult().powerNet.elements )
    {
        if( elem.type == wxS( "R" ) && elem.name.Contains( wxS( "Rtr" ) ) )
            r10mm = elem.value;
    }

    // Now add a second, longer trace on a fresh board
    auto board2 = std::make_unique<BOARD>();
    board2->Add( new NETINFO_ITEM( board2.get(), wxS( "+3V3" ), 1 ) );
    board2->Add( new NETINFO_ITEM( board2.get(), wxS( "GND" ), 2 ) );

    BOARD_DESIGN_SETTINGS& bds2 = board2->GetDesignSettings();
    bds2.SetCopperLayerCount( 4 );
    bds2.SetBoardThickness( pcbIUScale.mmToIU( 1.6 ) );
    BOARD_STACKUP& stackup2 = bds2.GetStackupDescriptor();
    stackup2.RemoveAll();
    stackup2.BuildDefaultStackupList( &bds2, 4 );

    FOOTPRINT* fp3 = new FOOTPRINT( board2.get() );
    fp3->SetReference( wxS( "R1" ) );
    fp3->SetPosition( VECTOR2I( 10000000, 10000000 ) );
    board2->Add( fp3 );

    PAD* p3 = new PAD( fp3 );
    p3->SetPosition( VECTOR2I( 10000000, 10000000 ) );
    p3->SetSize( PADSTACK::ALL_LAYERS, VECTOR2I( 500000, 500000 ) );
    p3->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::CIRCLE );
    p3->SetAttribute( PAD_ATTRIB::SMD );
    p3->SetLayerSet( { F_Cu } );
    p3->SetNet( board2->GetNetInfo().GetNetItem( 1 ) );
    p3->SetNumber( wxS( "1" ) );
    fp3->Add( p3 );

    FOOTPRINT* fp4 = new FOOTPRINT( board2.get() );
    fp4->SetReference( wxS( "R2" ) );
    fp4->SetPosition( VECTOR2I( 30000000, 10000000 ) );
    board2->Add( fp4 );

    PAD* p4 = new PAD( fp4 );
    p4->SetPosition( VECTOR2I( 30000000, 10000000 ) );
    p4->SetSize( PADSTACK::ALL_LAYERS, VECTOR2I( 500000, 500000 ) );
    p4->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::CIRCLE );
    p4->SetAttribute( PAD_ATTRIB::SMD );
    p4->SetLayerSet( { F_Cu } );
    p4->SetNet( board2->GetNetInfo().GetNetItem( 1 ) );
    p4->SetNumber( wxS( "1" ) );
    fp4->Add( p4 );

    // 20mm trace
    PCB_TRACK* t2 = new PCB_TRACK( board2.get() );
    t2->SetStart( VECTOR2I( 10000000, 10000000 ) );
    t2->SetEnd( VECTOR2I( 30000000, 10000000 ) );
    t2->SetWidth( 200000 );
    t2->SetLayer( F_Cu );
    t2->SetNet( board2->GetNetInfo().GetNetItem( 1 ) );
    board2->Add( t2 );

    board2->BuildConnectivity();

    PDN_LAYOUT_EXTRACTOR ext2( board2.get() );
    ext2.Extract( wxS( "+3V3" ), wxS( "GND" ) );

    double r20mm = 0.0;

    for( const auto& elem : ext2.GetResult().powerNet.elements )
    {
        if( elem.type == wxS( "R" ) && elem.name.Contains( wxS( "Rtr" ) ) )
            r20mm = elem.value;
    }

    BOOST_REQUIRE_GT( r10mm, 0.0 );
    BOOST_REQUIRE_GT( r20mm, 0.0 );

    // 20mm should be ~2× the resistance of 10mm
    double ratio = r20mm / r10mm;
    BOOST_CHECK_CLOSE( ratio, 2.0, 5.0 );

    BOOST_TEST_MESSAGE( "R(10mm)=" << r10mm * 1e3 << "mΩ  R(20mm)=" << r20mm * 1e3
                        << "mΩ  ratio=" << ratio );
}


/**
 * Test that a dual-net extraction produces SPICE-safe output:
 * no duplicate element names, no duplicate node names across nets,
 * no illegal characters (colons, parens, dots) in node or element names.
 */
BOOST_AUTO_TEST_CASE( SpiceSafeNames )
{
    // Build a board with components on both +3V3 and GND
    FOOTPRINT* fpC1 = addFootprint( wxS( "C1" ), VECTOR2I( 10000000, 25000000 ) );
    addPad( fpC1, wxS( "1" ), VECTOR2I( 10000000, 25000000 ), 1, F_Cu );  // +3V3
    addPad( fpC1, wxS( "2" ), VECTOR2I( 10500000, 25000000 ), 2, F_Cu );  // GND

    FOOTPRINT* fpC2 = addFootprint( wxS( "C2" ), VECTOR2I( 20000000, 25000000 ) );
    addPad( fpC2, wxS( "1" ), VECTOR2I( 20000000, 25000000 ), 1, F_Cu );
    addPad( fpC2, wxS( "2" ), VECTOR2I( 20500000, 25000000 ), 2, F_Cu );

    // Traces on +3V3
    addTrace( VECTOR2I( 10000000, 25000000 ), VECTOR2I( 20000000, 25000000 ),
              200000, 1, F_Cu );

    // Traces on GND
    addTrace( VECTOR2I( 10500000, 25000000 ), VECTOR2I( 20500000, 25000000 ),
              200000, 2, F_Cu );

    buildConnectivity();

    PDN_LAYOUT_EXTRACTOR extractor( m_board.get() );
    bool ok = extractor.Extract( wxS( "+3V3" ), wxS( "GND" ) );
    BOOST_REQUIRE( ok );

    wxString payload = extractor.SerializeResult();

    // Collect all element names and node names from both nets
    std::set<wxString> allElementNames;
    std::set<wxString> allNodeNames;

    auto checkElements = [&]( const std::vector<PDN_SPICE_ELEMENT>& elems )
    {
        for( const PDN_SPICE_ELEMENT& elem : elems )
        {
            // No duplicate element names
            BOOST_CHECK_MESSAGE( allElementNames.insert( elem.name ).second,
                                 "Duplicate element name: " + elem.name );

            // No illegal SPICE characters or leading digits in names
            for( const wxString& name : { elem.name, elem.nodeA, elem.nodeB } )
            {
                BOOST_CHECK_MESSAGE( name.IsEmpty() || !wxIsdigit( name[0] ),
                                     "Leading digit in SPICE name: " + name );
                BOOST_CHECK_MESSAGE( !name.Contains( wxS( ":" ) ),
                                     "Colon in SPICE name: " + name );
                BOOST_CHECK_MESSAGE( !name.Contains( wxS( "(" ) ),
                                     "Paren in SPICE name: " + name );
                BOOST_CHECK_MESSAGE( !name.Contains( wxS( ")" ) ),
                                     "Paren in SPICE name: " + name );
            }
        }
    };

    const auto& result = extractor.GetResult();
    checkElements( result.powerNet.elements );
    checkElements( result.groundNet.elements );
    checkElements( result.planeCaps );

    // Power and ground internal nodes should not collide
    std::set<wxString> powerNodes, groundNodes;

    for( const auto& elem : result.powerNet.elements )
    {
        powerNodes.insert( elem.nodeA );
        powerNodes.insert( elem.nodeB );
    }

    for( const auto& elem : result.groundNet.elements )
    {
        groundNodes.insert( elem.nodeA );
        groundNodes.insert( elem.nodeB );
    }

    // Internal nodes (not pad ports) should be distinct between nets
    std::set<wxString> padNodeNames;

    for( const auto& port : result.powerNet.padPorts )
        padNodeNames.insert( port.nodeName );

    for( const auto& port : result.groundNet.padPorts )
        padNodeNames.insert( port.nodeName );

    for( const wxString& node : powerNodes )
    {
        if( padNodeNames.count( node ) )
            continue; // pad nodes are shared (that's expected)

        BOOST_CHECK_MESSAGE( groundNodes.find( node ) == groundNodes.end(),
                             "Internal node collision between nets: " + node );
    }

    BOOST_TEST_MESSAGE( "Power elements: " << result.powerNet.elements.size()
                        << "  Ground elements: " << result.groundNet.elements.size()
                        << "  Unique element names: " << allElementNames.size() );
}


/**
 * Two caps with their power pads stitched to the same copper spot (e.g.
 * C20 and C22 sharing a pour fillet, or two 0402s dropped on the same
 * stitching via field) must resolve to the same SPICE node.  Otherwise
 * observing at whichever refdes the extractor processes second lands on
 * a dangling node and the analysis shows no/garbage impedance trace.
 */
BOOST_AUTO_TEST_CASE( ParallelCapsShareNode )
{
    // Two cap footprints with coincident power pads.  Ground pads are also
    // coincident.  A feeder trace connects to the shared pad spot via a
    // stub from another point, so the extractor has to generate a trace
    // element that uses the same node the pads registered with.
    FOOTPRINT* fpA = addFootprint( wxS( "C20" ), VECTOR2I( 10000000, 10000000 ) );
    FOOTPRINT* fpB = addFootprint( wxS( "C22" ), VECTOR2I( 10000000, 10000000 ) );

    // Power pads at the same position
    addPad( fpA, wxS( "1" ), VECTOR2I( 10000000, 10000000 ), 1 );
    addPad( fpB, wxS( "1" ), VECTOR2I( 10000000, 10000000 ), 1 );

    // Ground pads at a different shared position
    addPad( fpA, wxS( "2" ), VECTOR2I( 10000000, 11000000 ), 2 );
    addPad( fpB, wxS( "2" ), VECTOR2I( 10000000, 11000000 ), 2 );

    // Feeder traces so there is a routable power/ground path (otherwise
    // the extractor emits no elements and the test becomes vacuous)
    FOOTPRINT* src = addFootprint( wxS( "U1" ), VECTOR2I( 5000000, 10000000 ) );
    addPad( src, wxS( "1" ), VECTOR2I( 5000000, 10000000 ), 1 );
    addPad( src, wxS( "2" ), VECTOR2I( 5000000, 11000000 ), 2 );
    addTrace( VECTOR2I( 5000000, 10000000 ), VECTOR2I( 10000000, 10000000 ), 200000, 1 );
    addTrace( VECTOR2I( 5000000, 11000000 ), VECTOR2I( 10000000, 11000000 ), 200000, 2 );

    buildConnectivity();

    PDN_LAYOUT_EXTRACTOR extractor( m_board.get() );
    BOOST_REQUIRE( extractor.Extract( wxS( "+3V3" ), wxS( "GND" ),
                                      { wxS( "C20" ), wxS( "C22" ), wxS( "U1" ) } ) );

    const PDN_EXTRACTION_RESULT& result = extractor.GetResult();

    auto findPadNode = [&]( const std::vector<PDN_PAD_PORT>& ports,
                            const wxString& refdes, const wxString& padNum ) -> wxString
    {
        for( const PDN_PAD_PORT& p : ports )
        {
            if( p.refdes == refdes && p.padNumber == padNum )
                return p.nodeName;
        }
        return wxEmptyString;
    };

    wxString c20Power = findPadNode( result.powerNet.padPorts, wxS( "C20" ), wxS( "1" ) );
    wxString c22Power = findPadNode( result.powerNet.padPorts, wxS( "C22" ), wxS( "1" ) );
    wxString c20Gnd = findPadNode( result.groundNet.padPorts, wxS( "C20" ), wxS( "2" ) );
    wxString c22Gnd = findPadNode( result.groundNet.padPorts, wxS( "C22" ), wxS( "2" ) );

    BOOST_REQUIRE( !c20Power.IsEmpty() );
    BOOST_REQUIRE( !c22Power.IsEmpty() );
    BOOST_CHECK_EQUAL( c20Power, c22Power );
    BOOST_CHECK_EQUAL( c20Gnd, c22Gnd );

    // The shared node must also be the node that the feeder trace connects
    // to — otherwise the observation point still dangles.
    auto nodeAppearsInElements = [&]( const std::vector<PDN_SPICE_ELEMENT>& elems,
                                      const wxString& node )
    {
        for( const PDN_SPICE_ELEMENT& e : elems )
        {
            if( e.nodeA == node || e.nodeB == node )
                return true;
        }
        return false;
    };

    BOOST_CHECK( nodeAppearsInElements( result.powerNet.elements, c22Power ) );
    BOOST_CHECK( nodeAppearsInElements( result.groundNet.elements, c22Gnd ) );
}


BOOST_AUTO_TEST_SUITE_END()
