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

#include <wx/process.h>
#include <wx/tokenzr.h>
#include <wx/txtstrm.h>
#include <wx/utils.h>

#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>


std::ostream& boost_test_print_type( std::ostream& os, wxString const& aStr )
{
    os << aStr.ToStdString();
    return os;
}


#ifndef NGSPICE_EXECUTABLE_PATH
    #define NGSPICE_EXECUTABLE_PATH ""
#endif


struct PDN_E2E_FIXTURE
{
    PDN_E2E_FIXTURE() = default;

    void buildBoard( const wxString& aPowerNet, const wxString& aGroundNet )
    {
        m_board = std::make_unique<BOARD>();
        m_board->Add( new NETINFO_ITEM( m_board.get(), aPowerNet, 1 ) );
        m_board->Add( new NETINFO_ITEM( m_board.get(), aGroundNet, 2 ) );

        BOARD_DESIGN_SETTINGS& bds = m_board->GetDesignSettings();
        bds.SetCopperLayerCount( 4 );
        bds.SetBoardThickness( pcbIUScale.mmToIU( 1.6 ) );

        BOARD_STACKUP& stackup = bds.GetStackupDescriptor();
        stackup.RemoveAll();
        stackup.BuildDefaultStackupList( &bds, 4 );

        // Layout: VRM → C1 → C2 → Load on F.Cu, with GND returns
        addFootprintWithPads( wxS( "U1" ), VECTOR2I( 5000000, 25000000 ) );
        addFootprintWithPads( wxS( "C1" ), VECTOR2I( 15000000, 25000000 ) );
        addFootprintWithPads( wxS( "C2" ), VECTOR2I( 25000000, 25000000 ) );
        addFootprintWithPads( wxS( "U2" ), VECTOR2I( 35000000, 25000000 ) );

        // Power traces
        addTrace( VECTOR2I( 5000000, 25000000 ), VECTOR2I( 15000000, 25000000 ), 200000, 1 );
        addTrace( VECTOR2I( 15000000, 25000000 ), VECTOR2I( 25000000, 25000000 ), 200000, 1 );
        addTrace( VECTOR2I( 25000000, 25000000 ), VECTOR2I( 35000000, 25000000 ), 200000, 1 );

        // Ground return traces
        addTrace( VECTOR2I( 5000000, 26000000 ), VECTOR2I( 15000000, 26000000 ), 200000, 2 );
        addTrace( VECTOR2I( 15000000, 26000000 ), VECTOR2I( 25000000, 26000000 ), 200000, 2 );
        addTrace( VECTOR2I( 25000000, 26000000 ), VECTOR2I( 35000000, 26000000 ), 200000, 2 );

        m_board->BuildConnectivity();
    }

    void addFootprintWithPads( const wxString& aRef, const VECTOR2I& aPos )
    {
        FOOTPRINT* fp = new FOOTPRINT( m_board.get() );
        fp->SetPosition( aPos );
        fp->SetReference( aRef );
        m_board->Add( fp );

        // Pin 1 = power, pin 2 = ground (offset by 1mm)
        for( int i = 0; i < 2; i++ )
        {
            PAD*     p = new PAD( fp );
            VECTOR2I pos = aPos + VECTOR2I( 0, i * 1000000 );
            p->SetPosition( pos );
            p->SetSize( PADSTACK::ALL_LAYERS, VECTOR2I( 500000, 500000 ) );
            p->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::CIRCLE );
            p->SetAttribute( PAD_ATTRIB::SMD );
            p->SetLayerSet( { F_Cu } );
            p->SetNet( m_board->GetNetInfo().GetNetItem( i + 1 ) );
            p->SetNumber( wxString::Format( wxS( "%d" ), i + 1 ) );
            fp->Add( p );
        }
    }

    void addTrace( const VECTOR2I& aStart, const VECTOR2I& aEnd, int aWidth, int aNetCode )
    {
        PCB_TRACK* t = new PCB_TRACK( m_board.get() );
        t->SetStart( aStart );
        t->SetEnd( aEnd );
        t->SetWidth( aWidth );
        t->SetLayer( F_Cu );
        t->SetNet( m_board->GetNetInfo().GetNetItem( aNetCode ) );
        m_board->Add( t );
    }

    /**
     * Assemble a SPICE netlist from the extraction result, mimicking the
     * logic in PDN_ANALYZER::BuildSpiceNetlist (layout-aware overload).
     */
    wxString buildSpiceNetlist( const PDN_EXTRACTION_RESULT& aResult,
                                const wxString& aObsRefdes )
    {
        wxString netlist;

        netlist += wxS( "* PDN test netlist\n\n" );
        netlist += wxS( ".options KLU\n\n" );

        // Cap subcircuit
        netlist += wxS( ".subckt CAP p n c=100n esr=0.01 esl=400p\n" );
        netlist += wxS( "C1 p 1 {c}\n" );
        netlist += wxS( "R1 1 2 {esr}\n" );
        netlist += wxS( "L1 2 n {esl}\n" );
        netlist += wxS( ".ends\n\n" );

        // Parasitic elements
        netlist += wxS( "* Power net parasitics\n" );

        for( const PDN_SPICE_ELEMENT& elem : aResult.powerNet.elements )
        {
            netlist += wxString::Format( wxS( "%s_%s %s %s %g\n" ),
                                         elem.type, elem.name, elem.nodeA, elem.nodeB,
                                         elem.value );
        }

        netlist += wxS( "\n* Ground net parasitics\n" );

        for( const PDN_SPICE_ELEMENT& elem : aResult.groundNet.elements )
        {
            netlist += wxString::Format( wxS( "%s_%s %s %s %g\n" ),
                                         elem.type, elem.name, elem.nodeA, elem.nodeB,
                                         elem.value );
        }

        for( const PDN_SPICE_ELEMENT& elem : aResult.planeCaps )
        {
            netlist += wxString::Format( wxS( "%s_%s %s %s %g\n" ),
                                         elem.type, elem.name, elem.nodeA, elem.nodeB,
                                         elem.value );
        }

        // Build pad port map
        std::map<wxString, wxString> padPortMap;

        for( const PDN_PAD_PORT& p : aResult.powerNet.padPorts )
            padPortMap[p.refdes + wxS( ":" ) + p.padNumber] = p.nodeName;

        for( const PDN_PAD_PORT& p : aResult.groundNet.padPorts )
            padPortMap[p.refdes + wxS( ":" ) + p.padNumber] = p.nodeName;

        // Resolve observation nodes by net membership, not pad number.
        // Real-world cap footprints vary: some have pin 1 on power, some on
        // ground.  We know which is which because the extractor splits pad
        // ports between powerNet.padPorts and groundNet.padPorts.
        wxString obsP, obsG;

        for( const PDN_PAD_PORT& p : aResult.powerNet.padPorts )
        {
            if( p.refdes == aObsRefdes )
            {
                obsP = p.nodeName;
                break;
            }
        }

        for( const PDN_PAD_PORT& p : aResult.groundNet.padPorts )
        {
            if( p.refdes == aObsRefdes )
            {
                obsG = p.nodeName;
                break;
            }
        }

        BOOST_REQUIRE_MESSAGE( !obsP.IsEmpty(),
                               "No power pad port for " + aObsRefdes );
        BOOST_REQUIRE_MESSAGE( !obsG.IsEmpty(),
                               "No ground pad port for " + aObsRefdes );

        // Tie the observation ground to SPICE ground (node 0) for DC reference
        netlist += wxString::Format( wxS( "\nR_gnd_tie %s 0 1e-9\n" ), obsG );

        // Simple VRM model: a DC-conductive R+L branch between obsP and obsG.
        // Without this, the only bridges between power and ground are the CAP
        // subcircuits (C-R-L series → DC-open), making the DC op point singular.
        // This matches the VRM path that PDN_ANALYZER emits when a VRM exists.
        netlist += wxString::Format( wxS( "R_vrm %s vrm_int 1e-3\n" ), obsP );
        netlist += wxString::Format( wxS( "L_vrm vrm_int %s 1e-6\n" ), obsG );

        // AC source at observation point
        netlist += wxString::Format( wxS( "I1 %s %s AC 1\n" ), obsP, obsG );

        // Hook up all cap pad-pair nodes with bypass caps.  Map refdes to
        // its power and ground pad nodes so the CAP subckt instance straddles
        // the correct rails regardless of the cap footprint's pin numbering.
        std::map<wxString, wxString> capPower, capGround;

        for( const PDN_PAD_PORT& p : aResult.powerNet.padPorts )
        {
            if( p.refdes.StartsWith( wxS( "C" ) ) && !capPower.count( p.refdes ) )
                capPower[p.refdes] = p.nodeName;
        }

        for( const PDN_PAD_PORT& p : aResult.groundNet.padPorts )
        {
            if( p.refdes.StartsWith( wxS( "C" ) ) && !capGround.count( p.refdes ) )
                capGround[p.refdes] = p.nodeName;
        }

        int xIdx = 1;

        for( const auto& [refdes, pNode] : capPower )
        {
            auto gIt = capGround.find( refdes );

            if( gIt == capGround.end() )
                continue;

            netlist += wxString::Format( wxS( "X%d %s %s CAP c=100n esr=0.01 esl=400p\n" ),
                                         xIdx++, pNode, gIt->second );
        }

        netlist += wxS( "\n.ac dec 10 1e3 1e9\n" );
        netlist += wxS( ".print ac v(" ) + obsP + wxS( ")\n" );
        netlist += wxS( ".end\n" );

        return netlist;
    }

    /**
     * Run ngspice in batch mode on a netlist string, return true if it accepts
     * the netlist and runs without error.  Captures stdout/stderr for diagnostics.
     */
    bool runNgspice( const wxString& aNetlist, wxString* aErrorOut = nullptr )
    {
        if( wxString( NGSPICE_EXECUTABLE_PATH ).IsEmpty() )
        {
            BOOST_TEST_MESSAGE( "ngspice binary not found — skipping" );
            return true;
        }

        // Write netlist to temp file
        wxString tmpFile = wxFileName::CreateTempFileName( wxS( "pdn_test_" ) );
        tmpFile += wxS( ".sp" );

        {
            FILE* f = fopen( tmpFile.ToStdString().c_str(), "w" );

            if( !f )
                return false;

            fprintf( f, "%s", (const char*) aNetlist.utf8_str() );
            fclose( f );
        }

        // Run: ngspice -b -o output.log netlist.sp.  wxExecute spawns the
        // process directly (no shell), so shell redirections like 2>&1 are
        // not available — stdout/stderr are captured via the output/errors
        // arrays below.
        wxString outFile = tmpFile + wxS( ".out" );
        wxString cmd = wxString::Format( wxS( "\"%s\" -b -o \"%s\" \"%s\"" ),
                                         wxS( NGSPICE_EXECUTABLE_PATH ),
                                         outFile, tmpFile );

        wxArrayString output;
        wxArrayString errors;
        long exitCode = wxExecute( cmd, output, errors, wxEXEC_SYNC );

        // Read the ngspice log output
        wxString logText;

        {
            FILE* f = fopen( outFile.ToStdString().c_str(), "r" );

            if( f )
            {
                char buf[4096];

                while( fgets( buf, sizeof( buf ), f ) )
                    logText += wxString::FromUTF8( buf );

                fclose( f );
            }
        }

        // Also include captured stdout/stderr
        for( const wxString& line : output )
            logText += line + wxS( "\n" );

        for( const wxString& line : errors )
            logText += line + wxS( "\n" );

        // Cleanup
        wxRemoveFile( tmpFile );
        wxRemoveFile( outFile );

        if( aErrorOut )
            *aErrorOut = logText;

        // ngspice exits 0 even when it aborts (e.g. on a singular matrix),
        // so rely on log-text markers rather than the exit code alone.
        bool hasError = logText.Contains( wxS( "Error:" ) )
                        || logText.Contains( wxS( "fatal" ) )
                        || logText.Contains( wxS( "Unable to find" ) )
                        || logText.Contains( wxS( "singular matrix" ) )
                        || logText.Contains( wxS( "simulation(s) aborted" ) )
                        || logText.Contains( wxS( "no convergence" ) );

        return exitCode == 0 && !hasError;
    }

    std::unique_ptr<BOARD> m_board;
};


BOOST_FIXTURE_TEST_SUITE( PdnE2E, PDN_E2E_FIXTURE )


/**
 * Test the full pipeline for a given net name pair.
 */
static void fullPipelineTest( PDN_E2E_FIXTURE& fx, const wxString& aPower,
                              const wxString& aGround )
{
    BOOST_TEST_MESSAGE( "=== " << aPower.ToStdString() << " / " << aGround.ToStdString()
                        << " ===" );

    fx.buildBoard( aPower, aGround );

    std::set<wxString> pdnRefdes{ wxS( "C1" ), wxS( "C2" ), wxS( "U1" ), wxS( "U2" ) };

    PDN_LAYOUT_EXTRACTOR extractor( fx.m_board.get() );
    BOOST_REQUIRE_MESSAGE( extractor.Extract( aPower, aGround, pdnRefdes ),
                           "Extraction failed" );

    const PDN_EXTRACTION_RESULT& result = extractor.GetResult();

    BOOST_CHECK( !result.powerNet.padPorts.empty() );
    BOOST_CHECK( !result.groundNet.padPorts.empty() );
    BOOST_CHECK( !result.powerNet.elements.empty() );

    // Verify SPICE name safety
    auto checkName = [&]( const wxString& name )
    {
        BOOST_CHECK_MESSAGE( !name.IsEmpty() && !wxIsdigit( name[0] ),
                             wxString::Format( wxS( "Leading digit: %s" ), name ).ToStdString() );

        for( wxUniChar bad : { ':', '(', ')', ' ', '.' } )
        {
            BOOST_CHECK_MESSAGE( name.Find( bad ) == wxNOT_FOUND,
                                 wxString::Format( wxS( "Illegal char in %s" ), name )
                                         .ToStdString() );
        }
    };

    for( const PDN_SPICE_ELEMENT& e : result.powerNet.elements )
    {
        checkName( e.name );
        checkName( e.nodeA );
        checkName( e.nodeB );
    }

    for( const PDN_SPICE_ELEMENT& e : result.groundNet.elements )
    {
        checkName( e.name );
        checkName( e.nodeA );
        checkName( e.nodeB );
    }

    // Build and validate SPICE netlist via ngspice subprocess
    wxString netlist = fx.buildSpiceNetlist( result, wxS( "U2" ) );
    wxString log;
    bool     ok = fx.runNgspice( netlist, &log );

    BOOST_CHECK_MESSAGE( ok, wxString::Format(
            wxS( "ngspice rejected netlist for %s:\n%s\n--- netlist ---\n%s" ),
            aPower, log.Left( 1000 ), netlist ).ToStdString() );

    BOOST_TEST_MESSAGE( "Extracted: " << result.powerNet.elements.size() << " power elems, "
                        << result.groundNet.elements.size() << " ground elems" );
}


/**
 * The critical test: many net name variations, exercising the name-sanitization
 * paths in the extractor.  This is what caught the "+1V8" leading-digit bug.
 */
BOOST_AUTO_TEST_CASE( NetNameVariations )
{
    fullPipelineTest( *this, wxS( "+3V3" ), wxS( "GND" ) );
    fullPipelineTest( *this, wxS( "+1V8" ), wxS( "GND" ) );    // leading digit
    fullPipelineTest( *this, wxS( "+5V" ), wxS( "GND" ) );
    fullPipelineTest( *this, wxS( "+12V" ), wxS( "GND" ) );
    fullPipelineTest( *this, wxS( "1V0" ), wxS( "GND" ) );     // leading digit, no +
    fullPipelineTest( *this, wxS( "VCC" ), wxS( "GND" ) );
    fullPipelineTest( *this, wxS( "VDD" ), wxS( "VSS" ) );
    fullPipelineTest( *this, wxS( "+3V3_ANA" ), wxS( "AGND" ) );  // underscore
    fullPipelineTest( *this, wxS( "VBUS" ), wxS( "PGND" ) );
}


BOOST_AUTO_TEST_SUITE_END()


// --------------------------------------------------------------------
// Diagnostic probe: dump the extractor's view of specific caps on the
// cparti_fpga test board.  Motivated by user reports that C16 shows no
// trace (frequency range truncated), C20 shows no trace while C22 shows
// surprisingly high impedance on the PDN analyzer panel.
// --------------------------------------------------------------------


struct CPARTI_FPGA_FIXTURE
{
    CPARTI_FPGA_FIXTURE() :
            m_settingsManager( true /* headless */ )
    { }

    SETTINGS_MANAGER       m_settingsManager;
    std::unique_ptr<BOARD> m_board;
};


BOOST_FIXTURE_TEST_SUITE( PdnCpartiFpga, CPARTI_FPGA_FIXTURE )


/**
 * Run ngspice on a netlist and return the last .print block as a
 * vector of (freq, v_mag) rows so the test can report what the user
 * actually sees on the plot.
 */
static std::vector<std::pair<double, double>> runNetlistAndParse( const wxString& aNetlist )
{
    std::vector<std::pair<double, double>> rows;

    if( wxString( NGSPICE_EXECUTABLE_PATH ).IsEmpty() )
        return rows;

    wxString tmpFile = wxFileName::CreateTempFileName( wxS( "pdn_probe_" ) );
    tmpFile += wxS( ".sp" );

    {
        FILE* f = fopen( tmpFile.ToStdString().c_str(), "w" );

        if( !f )
            return rows;

        fprintf( f, "%s", (const char*) aNetlist.utf8_str() );
        fclose( f );
    }

    wxString outFile = tmpFile + wxS( ".out" );
    wxString cmd = wxString::Format( wxS( "\"%s\" -b -o \"%s\" \"%s\"" ),
                                     wxS( NGSPICE_EXECUTABLE_PATH ),
                                     outFile, tmpFile );
    wxArrayString output, errors;
    long exitCode = wxExecute( cmd, output, errors, wxEXEC_SYNC );

    BOOST_TEST_MESSAGE( "    ngspice exit=" << exitCode
                        << " stdout=" << output.GetCount()
                        << " stderr=" << errors.GetCount()
                        << " outFile=" << outFile.ToStdString() );

    for( size_t i = 0; i < output.GetCount() && i < 20; i++ )
        BOOST_TEST_MESSAGE( "      out: " << output[i].ToStdString() );

    for( size_t i = 0; i < errors.GetCount() && i < 20; i++ )
        BOOST_TEST_MESSAGE( "      err: " << errors[i].ToStdString() );

    FILE* f = fopen( outFile.ToStdString().c_str(), "r" );

    if( !f )
    {
        BOOST_TEST_MESSAGE( "    cannot open outFile" );
        return rows;
    }

    char buf[4096];
    wxString logText;

    while( fgets( buf, sizeof( buf ), f ) )
        logText += wxString::FromUTF8( buf );

    fclose( f );

    // Parse the .print ac output: lines like
    //   0   1.000000e+03   -1.00002e-03,   -6.28323e-03
    wxStringTokenizer lines( logText, wxS( "\n" ) );

    while( lines.HasMoreTokens() )
    {
        wxString line = lines.GetNextToken();
        line.Trim( true ).Trim( false );

        // Strip embedded tabs
        line.Replace( wxS( "\t" ), wxS( " " ) );

        wxStringTokenizer toks( line, wxS( " ," ) );

        if( toks.CountTokens() < 4 )
            continue;

        long   idx;
        double freq, vreal, vimag;
        wxString idxStr = toks.GetNextToken();

        if( !idxStr.ToLong( &idx ) )
            continue;

        if( !toks.GetNextToken().ToDouble( &freq ) )
            continue;

        if( !toks.GetNextToken().ToDouble( &vreal ) )
            continue;

        if( !toks.GetNextToken().ToDouble( &vimag ) )
            continue;

        double mag = std::hypot( vreal, vimag );
        rows.emplace_back( freq, mag );
    }

    wxRemoveFile( tmpFile );
    wxRemoveFile( outFile );

    return rows;
}


BOOST_AUTO_TEST_CASE( C16C20C22Diagnostic )
{
    try
    {
        KI_TEST::LoadBoard( m_settingsManager, "cparti_fpga", m_board );
    }
    catch( ... )
    {
        BOOST_TEST_MESSAGE( "cparti_fpga not loadable — skipping" );
        return;
    }

    if( !m_board )
    {
        BOOST_TEST_MESSAGE( "cparti_fpga missing — skipping" );
        return;
    }

    KI_TEST::FillZones( m_board.get() );
    m_board->BuildConnectivity();

    const std::vector<wxString> targets{ wxS( "C16" ), wxS( "C20" ), wxS( "C22" ) };

    // Build refdes -> footprint map for targets
    std::map<wxString, FOOTPRINT*> found;

    for( FOOTPRINT* fp : m_board->Footprints() )
    {
        wxString ref = fp->GetReference();

        if( std::find( targets.begin(), targets.end(), ref ) != targets.end() )
            found[ref] = fp;
    }

    // Dump each cap: pads, their nets, positions, and any neighbors sharing
    // the same pad position (which would have exercised the parallel-pad bug).
    std::set<std::pair<wxString, wxString>> netPairs; // (power, ground)

    for( const wxString& ref : targets )
    {
        BOOST_TEST_MESSAGE( "=== " << ref.ToStdString() << " ===" );

        auto it = found.find( ref );

        if( it == found.end() )
        {
            BOOST_TEST_MESSAGE( "  (not on board)" );
            continue;
        }

        FOOTPRINT* fp = it->second;

        wxString  powerNet;
        wxString  groundNet;

        for( PAD* pad : fp->Pads() )
        {
            wxString netName = pad->GetNet() ? pad->GetNet()->GetNetname() : wxString();
            VECTOR2I pos = pad->GetPosition();

            BOOST_TEST_MESSAGE( "  pad " << pad->GetNumber().ToStdString()
                                << " net=" << netName.ToStdString()
                                << " pos=(" << pos.x << "," << pos.y << ")" );

            // Find other footprints with a pad at the same position + net
            for( FOOTPRINT* other : m_board->Footprints() )
            {
                if( other == fp )
                    continue;

                for( PAD* op : other->Pads() )
                {
                    if( op->GetPosition() == pos
                        && op->GetNet() && op->GetNet()->GetNetname() == netName )
                    {
                        BOOST_TEST_MESSAGE( "    coincident with "
                                            << other->GetReference().ToStdString()
                                            << ":" << op->GetNumber().ToStdString() );
                    }
                }
            }

            // Detect ground by net name (GND/VSS/AGND/PGND/…) rather than
            // pin order, because real footprints vary in which pin is which.
            wxString upper = netName.Upper();

            if( upper == wxS( "GND" ) || upper.Contains( wxS( "GND" ) )
                || upper == wxS( "VSS" ) )
            {
                groundNet = netName;
            }
            else
            {
                powerNet = netName;
            }
        }

        if( !powerNet.IsEmpty() && !groundNet.IsEmpty() )
            netPairs.insert( { powerNet, groundNet } );
    }

    // Collect every cap refdes on each net pair so the extractor keeps the
    // relevant pad ports (it prunes unreachable items when aPdnRefdes is set).
    auto collectCapsOnPair = [&]( const wxString& aPower, const wxString& aGround )
    {
        std::set<wxString> refs;

        for( FOOTPRINT* fp : m_board->Footprints() )
        {
            wxString ref = fp->GetReference();

            if( !ref.StartsWith( wxS( "C" ) ) && !ref.StartsWith( wxS( "U" ) ) )
                continue;

            bool onPower = false;
            bool onGround = false;

            for( PAD* pad : fp->Pads() )
            {
                wxString net = pad->GetNet() ? pad->GetNet()->GetNetname() : wxString();

                if( net == aPower )
                    onPower = true;
                else if( net == aGround )
                    onGround = true;
            }

            if( onPower && onGround )
                refs.insert( ref );
        }

        return refs;
    };

    for( const auto& [powerNet, groundNet] : netPairs )
    {
        BOOST_TEST_MESSAGE( "--- extract " << powerNet.ToStdString()
                            << " / " << groundNet.ToStdString() << " ---" );

        std::set<wxString> refs = collectCapsOnPair( powerNet, groundNet );
        BOOST_TEST_MESSAGE( "  pdnRefdes set size: " << refs.size() );

        PDN_LAYOUT_EXTRACTOR extractor( m_board.get() );

        if( !extractor.Extract( powerNet, groundNet, refs ) )
        {
            BOOST_TEST_MESSAGE( "  extraction FAILED" );
            continue;
        }

        const PDN_EXTRACTION_RESULT& result = extractor.GetResult();
        BOOST_TEST_MESSAGE( "  power padPorts=" << result.powerNet.padPorts.size()
                            << " elems=" << result.powerNet.elements.size() );
        BOOST_TEST_MESSAGE( "  ground padPorts=" << result.groundNet.padPorts.size()
                            << " elems=" << result.groundNet.elements.size() );
        BOOST_TEST_MESSAGE( "  planeCaps=" << result.planeCaps.size() );

        // Gather node names that are actually referenced by any element.
        std::set<wxString> liveNodes;

        for( const PDN_SPICE_ELEMENT& e : result.powerNet.elements )
        {
            liveNodes.insert( e.nodeA );
            liveNodes.insert( e.nodeB );
        }

        for( const PDN_SPICE_ELEMENT& e : result.groundNet.elements )
        {
            liveNodes.insert( e.nodeA );
            liveNodes.insert( e.nodeB );
        }

        // For each target cap, report whether its pad ports land on a live
        // node or are dangling.
        for( const wxString& ref : targets )
        {
            bool sawPower = false;
            bool sawGround = false;

            auto dump = [&]( const std::vector<PDN_PAD_PORT>& ports, const char* tag,
                             bool& sawFlag )
            {
                for( const PDN_PAD_PORT& p : ports )
                {
                    if( p.refdes != ref )
                        continue;

                    sawFlag = true;
                    bool live = liveNodes.count( p.nodeName ) > 0;
                    BOOST_TEST_MESSAGE( "  " << ref.ToStdString() << ":"
                                        << p.padNumber.ToStdString() << " [" << tag
                                        << "] node=" << p.nodeName.ToStdString()
                                        << ( live ? " LIVE" : " DANGLING" ) );
                }
            };

            dump( result.powerNet.padPorts, "power", sawPower );
            dump( result.groundNet.padPorts, "ground", sawGround );

            if( !sawPower && !sawGround )
            {
                BOOST_TEST_MESSAGE( "  " << ref.ToStdString()
                                    << " NOT in padPorts for this net pair" );
                continue;
            }

            // Build the netlist (fixture helper uses the same pin1/pin2
            // convention as production BuildSpiceNetlist) and run ngspice
            // to see what the user would see on the plot for this cap.
            PDN_E2E_FIXTURE asm_fx;
            wxString netlist = asm_fx.buildSpiceNetlist( result, ref );

            // Dump netlist + raw ngspice log for inspection
            wxString tmpNl = wxFileName::CreateTempFileName(
                    wxString::Format( wxS( "pdn_%s_" ), ref ) );
            FILE* nf = fopen( tmpNl.ToStdString().c_str(), "w" );
            fprintf( nf, "%s", (const char*) netlist.utf8_str() );
            fclose( nf );
            BOOST_TEST_MESSAGE( "  netlist dumped to " << tmpNl.ToStdString() );

            auto rows = runNetlistAndParse( netlist );
            BOOST_TEST_MESSAGE( "  ngspice returned " << rows.size() << " AC points" );

            if( !rows.empty() )
            {
                auto dumpRow = [&]( size_t i )
                {
                    BOOST_TEST_MESSAGE( "    f=" << rows[i].first
                                        << " |V|=" << rows[i].second );
                };

                dumpRow( 0 );
                dumpRow( rows.size() / 4 );
                dumpRow( rows.size() / 2 );
                dumpRow( 3 * rows.size() / 4 );
                dumpRow( rows.size() - 1 );

                double minMag = rows[0].second, maxMag = rows[0].second;

                for( const auto& r : rows )
                {
                    minMag = std::min( minMag, r.second );
                    maxMag = std::max( maxMag, r.second );
                }

                BOOST_TEST_MESSAGE( "    |V| range: " << minMag << " .. " << maxMag );
            }
        }
    }
}


static void dumpPadConnectivity( CPARTI_FPGA_FIXTURE& fx, const wxString& aRef,
                                 const wxString& aPadNumber )
{
    if( !fx.m_board )
    {
        try
        {
            KI_TEST::LoadBoard( fx.m_settingsManager, "cparti_fpga", fx.m_board );
        }
        catch( ... )
        {
            BOOST_TEST_MESSAGE( "cparti_fpga not loadable — skipping" );
            return;
        }

        if( !fx.m_board )
            return;

        KI_TEST::FillZones( fx.m_board.get() );
        fx.m_board->BuildConnectivity();
    }

    FOOTPRINT* fp = nullptr;

    for( FOOTPRINT* f : fx.m_board->Footprints() )
    {
        if( f->GetReference() == aRef )
        {
            fp = f;
            break;
        }
    }

    BOOST_REQUIRE( fp );

    PAD*     target = nullptr;
    VECTOR2I p2pos;
    wxString padNet;

    for( PAD* pad : fp->Pads() )
    {
        if( pad->GetNumber() == aPadNumber )
        {
            target = pad;
            p2pos = pad->GetPosition();
            padNet = pad->GetNet() ? pad->GetNet()->GetNetname() : wxString();
            break;
        }
    }

    BOOST_REQUIRE( target );

    BOOST_TEST_MESSAGE( "=== " << aRef.ToStdString() << ":"
                        << aPadNumber.ToStdString() << " ===" );
    BOOST_TEST_MESSAGE( "pos=(" << p2pos.x << "," << p2pos.y
                        << ") net=" << padNet.ToStdString()
                        << " layers=" << target->GetLayerSet().FmtHex() );

    int powerNetCode = target->GetNetCode();

    auto connectivity = fx.m_board->GetConnectivity();

    std::vector<BOARD_CONNECTED_ITEM*> netItems = connectivity->GetNetItems(
            powerNetCode, { PCB_PAD_T, PCB_TRACE_T, PCB_ARC_T, PCB_VIA_T, PCB_ZONE_T } );

    BOOST_TEST_MESSAGE( "Net " << padNet.ToStdString() << " has "
                        << netItems.size() << " items" );

    int neighborCount = 0;

    for( BOARD_CONNECTED_ITEM* item : netItems )
    {
        if( item->Type() == PCB_TRACE_T || item->Type() == PCB_ARC_T )
        {
            PCB_TRACK* t = static_cast<PCB_TRACK*>( item );

            if( t->GetStart() == p2pos || t->GetEnd() == p2pos )
            {
                neighborCount++;
                BOOST_TEST_MESSAGE( "  TRACK layer=" << (int) t->GetLayer()
                                    << " " << t->GetStart().x << "," << t->GetStart().y
                                    << " -> " << t->GetEnd().x << "," << t->GetEnd().y );
            }
        }
        else if( item->Type() == PCB_VIA_T )
        {
            PCB_VIA* v = static_cast<PCB_VIA*>( item );

            if( v->GetPosition() == p2pos )
            {
                neighborCount++;
                BOOST_TEST_MESSAGE( "  VIA pos=" << v->GetPosition().x << ","
                                    << v->GetPosition().y
                                    << " top=" << (int) v->TopLayer()
                                    << " bot=" << (int) v->BottomLayer() );
            }
        }
    }

    int64_t padDia = std::max( target->GetSize( PADSTACK::ALL_LAYERS ).x,
                               target->GetSize( PADSTACK::ALL_LAYERS ).y );
    int64_t searchRadius = padDia;   // anything fully inside the pad's copper

    BOOST_TEST_MESSAGE( "Nearby items within pad radius (" << searchRadius << "nm):" );

    for( BOARD_CONNECTED_ITEM* item : netItems )
    {
        VECTOR2I pos;

        if( item->Type() == PCB_VIA_T )
        {
            pos = static_cast<PCB_VIA*>( item )->GetPosition();
        }
        else if( item->Type() == PCB_TRACE_T || item->Type() == PCB_ARC_T )
        {
            PCB_TRACK* t = static_cast<PCB_TRACK*>( item );
            int64_t    da = ( (int64_t) t->GetStart().x - p2pos.x )
                                    * ( (int64_t) t->GetStart().x - p2pos.x )
                            + ( (int64_t) t->GetStart().y - p2pos.y )
                                      * ( (int64_t) t->GetStart().y - p2pos.y );
            int64_t db = ( (int64_t) t->GetEnd().x - p2pos.x )
                                 * ( (int64_t) t->GetEnd().x - p2pos.x )
                         + ( (int64_t) t->GetEnd().y - p2pos.y )
                                   * ( (int64_t) t->GetEnd().y - p2pos.y );
            pos = ( da <= db ) ? t->GetStart() : t->GetEnd();
        }
        else
        {
            continue;
        }

        if( pos == p2pos )
            continue;

        int64_t dx = (int64_t) pos.x - p2pos.x;
        int64_t dy = (int64_t) pos.y - p2pos.y;
        int64_t d2 = dx * dx + dy * dy;

        if( d2 < searchRadius * searchRadius )
        {
            const char* kind = ( item->Type() == PCB_VIA_T ) ? "VIA" : "TRACK_END";
            BOOST_TEST_MESSAGE( "  " << kind << " at (" << pos.x << "," << pos.y
                                << ") distance=" << (int64_t) std::sqrt( (double) d2 )
                                << "nm" );
        }
    }

    BOOST_TEST_MESSAGE( "Direct neighbors (coincident): " << neighborCount );

    // What does connectivity->GetConnectedItems(pad, ...) return?
    std::vector<BOARD_CONNECTED_ITEM*> cdNeighbours = connectivity->GetConnectedItems(
            target, { PCB_PAD_T, PCB_TRACE_T, PCB_ARC_T, PCB_VIA_T, PCB_ZONE_T } );

    BOOST_TEST_MESSAGE( "connectivity->GetConnectedItems returned "
                        << cdNeighbours.size() << " neighbours:" );

    for( BOARD_CONNECTED_ITEM* nb : cdNeighbours )
    {
        wxString kind;

        switch( nb->Type() )
        {
        case PCB_PAD_T: kind = wxS( "PAD" ); break;
        case PCB_TRACE_T: kind = wxS( "TRACK" ); break;
        case PCB_ARC_T: kind = wxS( "ARC" ); break;
        case PCB_VIA_T: kind = wxS( "VIA" ); break;
        case PCB_ZONE_T: kind = wxS( "ZONE" ); break;
        default: kind = wxS( "?" ); break;
        }

        BOOST_TEST_MESSAGE( "  " << kind.ToStdString() );
    }

    BOOST_TEST_MESSAGE( "Zone coverage of pad position:" );

    for( BOARD_CONNECTED_ITEM* item : netItems )
    {
        if( item->Type() != PCB_ZONE_T )
            continue;

        ZONE* zone = static_cast<ZONE*>( item );

        for( PCB_LAYER_ID layer : zone->GetLayerSet().CuStack() )
        {
            if( !zone->HasFilledPolysForLayer( layer ) )
                continue;

            bool hit = zone->HitTestFilledArea( layer, p2pos );
            BOOST_TEST_MESSAGE( "  zone layer=" << (int) layer
                                << " hitTestFilledArea=" << ( hit ? "YES" : "no" ) );
        }
    }

    PDN_LAYOUT_EXTRACTOR extractor( fx.m_board.get() );
    BOOST_REQUIRE( extractor.Extract( padNet, wxS( "GND" ),
                                      { aRef, wxS( "U1" ) } ) );

    const PDN_EXTRACTION_RESULT& result = extractor.GetResult();

    wxString padNode;

    for( const PDN_PAD_PORT& p : result.powerNet.padPorts )
    {
        if( p.refdes == aRef && p.padNumber == aPadNumber )
        {
            padNode = p.nodeName;
            break;
        }
    }

    BOOST_TEST_MESSAGE( "Extractor registered " << aRef.ToStdString() << ":"
                        << aPadNumber.ToStdString() << " as node '"
                        << padNode.ToStdString() << "'" );

    int refsInPower = 0;

    for( const PDN_SPICE_ELEMENT& e : result.powerNet.elements )
    {
        if( e.nodeA == padNode || e.nodeB == padNode )
        {
            refsInPower++;
            BOOST_TEST_MESSAGE( "  " << e.name.ToStdString() << " "
                                << e.nodeA.ToStdString() << " "
                                << e.nodeB.ToStdString() );
        }
    }

    BOOST_TEST_MESSAGE( "Power-net elements referencing pad node: " << refsInPower );
}


BOOST_AUTO_TEST_CASE( C61PadConnectivityTrace )
{
    dumpPadConnectivity( *this, wxS( "C61" ), wxS( "2" ) );
}


BOOST_AUTO_TEST_CASE( C82PadConnectivityTrace )
{
    dumpPadConnectivity( *this, wxS( "C82" ), wxS( "2" ) );
}


BOOST_AUTO_TEST_CASE( C81PadConnectivityTrace )
{
    dumpPadConnectivity( *this, wxS( "C81" ), wxS( "2" ) );
}


/**
 * Is C81 in the same electrical cluster as C16 on +1V8 (per KiCad's
 * connectivity)?  If not, the board has a physical split and the
 * extractor's island is correct.  If yes, we're missing a connection.
 */
BOOST_AUTO_TEST_CASE( C81C16SameCluster )
{
    try
    {
        KI_TEST::LoadBoard( m_settingsManager, "cparti_fpga", m_board );
    }
    catch( ... )
    {
        BOOST_TEST_MESSAGE( "cparti_fpga not loadable — skipping" );
        return;
    }

    KI_TEST::FillZones( m_board.get() );
    m_board->BuildConnectivity();

    PAD* c81 = nullptr;
    PAD* c16 = nullptr;

    for( FOOTPRINT* fp : m_board->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            wxString netName = pad->GetNet() ? pad->GetNet()->GetNetname() : wxString();

            if( netName != wxS( "+1V8" ) )
                continue;

            if( fp->GetReference() == wxS( "C81" ) && !c81 )
                c81 = pad;
            else if( fp->GetReference() == wxS( "C16" ) && !c16 )
                c16 = pad;
        }
    }

    BOOST_REQUIRE( c81 );
    BOOST_REQUIRE( c16 );

    auto connectivity = m_board->GetConnectivity();

    auto c81Cluster = connectivity->GetConnectedItems(
            c81, { PCB_PAD_T, PCB_TRACE_T, PCB_ARC_T, PCB_VIA_T, PCB_ZONE_T } );

    bool c16InCluster = false;

    for( BOARD_CONNECTED_ITEM* item : c81Cluster )
    {
        if( item == c16 )
        {
            c16InCluster = true;
            break;
        }
    }

    BOOST_TEST_MESSAGE( "C81 cluster has " << c81Cluster.size() << " items" );
    BOOST_TEST_MESSAGE( "C16:+1V8 in C81's cluster: " << ( c16InCluster ? "YES" : "NO" ) );
}


BOOST_AUTO_TEST_SUITE_END()
