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

#include "impedance_profiler_panel.h"

#include <board.h>
#include <netinfo.h>
#include <pcb_track.h>
#include <pcb_edit_frame.h>
#include <widgets/mathplot.h>

#include <sipi/trace_path_walker.h>
#include <sipi/stackup_reader.h>
#include <sipi/analytical_impedance.h>
#include <sipi/bem_2d_solver.h>

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include <algorithm>
#include <set>




IMPEDANCE_PROFILER_PANEL::IMPEDANCE_PROFILER_PANEL( PCB_EDIT_FRAME* aParent ) :
        WX_PANEL( aParent ),
        m_frame( aParent ),
        m_netSelector( nullptr ),
        m_targetZ0Input( nullptr ),
        m_statusText( nullptr ),
        m_plotWindow( nullptr ),
        m_impedanceTrace( nullptr ),
        m_targetLine( nullptr ),
        m_xAxis( nullptr ),
        m_yAxis( nullptr ),
        m_currentNetCode( -1 ),
        m_targetZ0( 50.0 )
{
    buildUI();

    if( BOARD* board = m_frame->GetBoard() )
        board->AddListener( this );
}


IMPEDANCE_PROFILER_PANEL::~IMPEDANCE_PROFILER_PANEL()
{
    if( m_frame && m_frame->GetBoard() )
        m_frame->GetBoard()->RemoveListener( this );
}


void IMPEDANCE_PROFILER_PANEL::buildUI()
{
    wxBoxSizer* mainSizer = new wxBoxSizer( wxVERTICAL );

    // Top bar: net selector + target Z0 + analyse button
    wxBoxSizer* topSizer = new wxBoxSizer( wxHORIZONTAL );

    topSizer->Add( new wxStaticText( this, wxID_ANY, _( "Net:" ) ),
                   0, wxALIGN_CENTER_VERTICAL | wxLEFT, 4 );

    m_netSelector = new wxChoice( this, wxID_ANY );
    m_netSelector->SetMinSize( wxSize( 120, -1 ) );
    topSizer->Add( m_netSelector, 1, wxALIGN_CENTER_VERTICAL | wxLEFT, 4 );

    topSizer->Add( new wxStaticText( this, wxID_ANY, _( "Target:" ) ),
                   0, wxALIGN_CENTER_VERTICAL | wxLEFT, 8 );

    m_targetZ0Input = new wxTextCtrl( this, wxID_ANY, wxS( "50" ),
                                      wxDefaultPosition, wxSize( 50, -1 ) );
    topSizer->Add( m_targetZ0Input, 0, wxALIGN_CENTER_VERTICAL | wxLEFT, 4 );

    topSizer->Add( new wxStaticText( this, wxID_ANY, wxT( "\u03A9" ) ),
                   0, wxALIGN_CENTER_VERTICAL | wxLEFT, 2 );

    wxButton* analyseBtn = new wxButton( this, wxID_ANY, _( "Analyse" ) );
    topSizer->Add( analyseBtn, 0, wxALIGN_CENTER_VERTICAL | wxLEFT, 8 );

    mainSizer->Add( topSizer, 0, wxEXPAND | wxALL, 4 );

    // Plot
    m_plotWindow = new mpWindow( this, wxID_ANY );
    m_plotWindow->SetMinSize( wxSize( 200, 150 ) );
    m_plotWindow->EnableDoubleBuffer( true );

    wxPen tracePen( wxColour( 0, 120, 200 ), 2, wxPENSTYLE_SOLID );

    m_xAxis = new mpScaleX( _( "Position (mm)" ), mpALIGN_BOTTOM, true );
    m_yAxis = new mpScaleY( _( "Z0 (\u03A9)" ), mpALIGN_LEFT, true );

    m_plotWindow->AddLayer( m_xAxis, false );
    m_plotWindow->AddLayer( m_yAxis, false );

    m_impedanceTrace = new mpFXYVector( _( "Z0" ) );
    m_impedanceTrace->SetPen( tracePen );
    m_impedanceTrace->SetContinuity( true );
    m_impedanceTrace->SetScale( m_xAxis, m_yAxis );
    m_impedanceTrace->SetVisible( false );
    m_plotWindow->AddLayer( m_impedanceTrace, false );

    wxPen targetPen( wxColour( 200, 50, 50 ), 1, wxPENSTYLE_SHORT_DASH );

    m_targetLine = new mpFXYVector( _( "Target" ) );
    m_targetLine->SetPen( targetPen );
    m_targetLine->SetContinuity( true );
    m_targetLine->SetScale( m_xAxis, m_yAxis );
    m_targetLine->SetVisible( false );
    m_plotWindow->AddLayer( m_targetLine, false );

    m_plotWindow->SetMargins( 15, 10, 30, 50 );
    m_plotWindow->UpdateAll();

    mainSizer->Add( m_plotWindow, 1, wxEXPAND | wxLEFT | wxRIGHT, 4 );

    // Status bar
    m_statusText = new wxStaticText( this, wxID_ANY, _( "Select a net and click Analyse." ) );
    mainSizer->Add( m_statusText, 0, wxEXPAND | wxALL, 4 );

    SetSizer( mainSizer );

    // Bind events
    analyseBtn->Bind( wxEVT_BUTTON, &IMPEDANCE_PROFILER_PANEL::onAnalyseClicked, this );
    m_netSelector->Bind( wxEVT_CHOICE, &IMPEDANCE_PROFILER_PANEL::onNetSelected, this );
}


void IMPEDANCE_PROFILER_PANEL::OnShowPanel()
{
    populateNetList();
}


void IMPEDANCE_PROFILER_PANEL::populateNetList()
{
    m_netSelector->Clear();

    BOARD* board = m_frame->GetBoard();

    if( !board )
        return;

    // Collect nets that have routed tracks
    std::set<int> routedNets;

    for( PCB_TRACK* track : board->Tracks() )
    {
        if( track->GetNetCode() > 0 && track->Type() != PCB_VIA_T )
            routedNets.insert( track->GetNetCode() );
    }

    // Add them sorted by name
    struct NET_ENTRY
    {
        int      netCode;
        wxString name;
    };

    std::vector<NET_ENTRY> entries;

    for( int nc : routedNets )
    {
        NETINFO_ITEM* net = board->FindNet( nc );

        if( net )
            entries.push_back( { nc, net->GetNetname() } );
    }

    std::sort( entries.begin(), entries.end(),
               []( const NET_ENTRY& a, const NET_ENTRY& b )
               {
                   return a.name < b.name;
               } );

    for( const NET_ENTRY& e : entries )
    {
        int idx = m_netSelector->Append( e.name );
        m_netSelector->SetClientData( idx, reinterpret_cast<void*>( (intptr_t) e.netCode ) );
    }

    // Re-select the previously selected net if it still exists
    if( m_currentNetCode > 0 )
    {
        for( unsigned i = 0; i < m_netSelector->GetCount(); i++ )
        {
            intptr_t nc = reinterpret_cast<intptr_t>( m_netSelector->GetClientData( i ) );

            if( nc == m_currentNetCode )
            {
                m_netSelector->SetSelection( i );
                break;
            }
        }
    }
}


void IMPEDANCE_PROFILER_PANEL::onAnalyseClicked( wxCommandEvent& aEvent )
{
    int sel = m_netSelector->GetSelection();

    if( sel == wxNOT_FOUND )
    {
        updateStatus( _( "No net selected." ) );
        return;
    }

    intptr_t netCode = reinterpret_cast<intptr_t>( m_netSelector->GetClientData( sel ) );

    // Parse target impedance
    double target = 50.0;
    m_targetZ0Input->GetValue().ToDouble( &target );
    m_targetZ0 = target;

    runAnalysis( static_cast<int>( netCode ) );
}


void IMPEDANCE_PROFILER_PANEL::onNetSelected( wxCommandEvent& aEvent )
{
    // Auto-analyse on net change
    wxCommandEvent dummy;
    onAnalyseClicked( dummy );
}


void IMPEDANCE_PROFILER_PANEL::runAnalysis( int aNetCode )
{
    m_currentNetCode = aNetCode;

    BOARD* board = m_frame->GetBoard();

    if( !board )
        return;

    // Walk the trace
    TRACE_PATH_WALKER walker( board );

    if( !walker.WalkNet( aNetCode ) )
    {
        updateStatus( _( "Could not walk net — no routed tracks found." ) );
        return;
    }

    const auto& path = walker.GetPath();
    const WALK_RESULT& result = walker.GetResult();

    // Build impedance profile using BEM solver with cache
    STACKUP_READER stackup( board );

    std::vector<double> positions;
    std::vector<double> impedances;

    // Cache: avoid re-solving identical cross-sections.
    // Key includes layer, width, and reference plane presence (hasRefAbove/Below)
    // so that traces passing over zone gaps get different impedance values.
    struct XS_KEY
    {
        PCB_LAYER_ID layer;
        int          width;
        bool         refAbove;
        bool         refBelow;
        bool operator<( const XS_KEY& o ) const
        {
            return std::tie( layer, width, refAbove, refBelow )
                   < std::tie( o.layer, o.width, o.refAbove, o.refBelow );
        }
    };

    std::map<XS_KEY, double> z0Cache;

    BOARD_CONNECTED_ITEM* lastItem = nullptr;
    double                lastZ0 = 0.0;

    for( const PATH_POINT& pt : path )
    {
        if( pt.isVia )
        {
            lastItem = nullptr; // force recompute after via (layer may change)
            continue;
        }

        double z0 = 0.0;

        if( pt.item == lastItem )
        {
            z0 = lastZ0;
        }
        else
        {
            PCB_TRACK* track = static_cast<PCB_TRACK*>( pt.item );

            LAYER_GEOMETRY geom = stackup.GetLayerGeometry( track->GetLayer(),
                                                            pt.position,
                                                            track->GetWidth() );

            XS_KEY key = { track->GetLayer(), track->GetWidth(),
                           geom.hasRefAbove, geom.hasRefBelow };

            auto cacheIt = z0Cache.find( key );

            if( cacheIt != z0Cache.end() )
            {
                z0 = cacheIt->second;
            }
            else
            {

                // Build BEM cross-section from LAYER_GEOMETRY
                XS_GEOMETRY xs;

                XS_CONDUCTOR cond;
                cond.centerX = 0.0;
                cond.width = geom.traceWidth;
                cond.thickness = geom.traceThickness;

                bool hasAbove = ( geom.hAbove > 0.0 );
                bool hasBelow = ( geom.hBelow > 0.0 );

                if( hasAbove && hasBelow )
                {
                    // Stripline: conductor between two ground planes
                    // Ground below at y=0, conductor centered between planes
                    xs.groundY = 0.0;
                    xs.hasUpperGround = true;
                    xs.upperGroundY = -( geom.hAbove + geom.hBelow );

                    cond.centerY = -geom.hBelow;
                    xs.epsilonR = ( geom.erAbove + geom.erBelow ) / 2.0;

                    // If εr differs significantly, add dielectric regions
                    if( std::abs( geom.erAbove - geom.erBelow ) > 0.5 )
                    {
                        XS_DIELECTRIC_REGION regAbove;
                        regAbove.yTop = xs.upperGroundY;
                        regAbove.yBottom = cond.centerY;
                        regAbove.epsilonR = geom.erAbove;

                        XS_DIELECTRIC_REGION regBelow;
                        regBelow.yTop = cond.centerY;
                        regBelow.yBottom = 0.0;
                        regBelow.epsilonR = geom.erBelow;

                        xs.dielectrics.push_back( regAbove );
                        xs.dielectrics.push_back( regBelow );
                    }
                }
                else if( hasBelow )
                {
                    // Microstrip: conductor above ground, dielectric below, air above
                    xs.groundY = 0.0;
                    cond.centerY = -( geom.hBelow + geom.traceThickness / 2.0 );
                    xs.epsilonR = geom.erBelow;

                    XS_DIELECTRIC_REGION air;
                    air.yTop = -10e-3; // far above
                    air.yBottom = -geom.hBelow;
                    air.epsilonR = 1.0;

                    XS_DIELECTRIC_REGION diel;
                    diel.yTop = -geom.hBelow;
                    diel.yBottom = 0.0;
                    diel.epsilonR = geom.erBelow;

                    xs.dielectrics.push_back( air );
                    xs.dielectrics.push_back( diel );
                }
                else if( hasAbove )
                {
                    // Inverted microstrip: ground above, dielectric above, air below
                    xs.groundY = -( geom.hAbove + geom.traceThickness );
                    cond.centerY = -geom.traceThickness / 2.0;
                    xs.epsilonR = geom.erAbove;

                    XS_DIELECTRIC_REGION diel;
                    diel.yTop = xs.groundY;
                    diel.yBottom = -geom.traceThickness;
                    diel.epsilonR = geom.erAbove;

                    XS_DIELECTRIC_REGION air;
                    air.yTop = -geom.traceThickness;
                    air.yBottom = 10e-3;
                    air.epsilonR = 1.0;

                    xs.dielectrics.push_back( diel );
                    xs.dielectrics.push_back( air );
                }

                xs.conductors.push_back( cond );

                // Solve
                BEM_2D_SOLVER solver;
                solver.SetGeometry( xs );
                solver.SetPanelsPerEdge( 15 );

                if( solver.Solve() && solver.GetResult().Z0 > 0.0 )
                {
                    z0 = solver.GetResult().Z0;
                }
                else
                {
                    // Fallback to analytical
                    z0 = STACKUP_READER::ComputeZ0( geom );
                }

                z0Cache[key] = z0;
            }

            lastItem = pt.item;
            lastZ0 = z0;
        }

        positions.push_back( pt.distFromStart / 1e6 ); // nm -> mm
        impedances.push_back( z0 );
    }

    if( positions.empty() )
    {
        updateStatus( _( "No impedance data — trace has no non-via segments." ) );
        return;
    }

    // Debug: log first segment geometry for diagnosis
    bool allZero = true;

    for( double z : impedances )
    {
        if( z > 0.0 )
        {
            allZero = false;
            break;
        }
    }

    if( allZero )
    {
        // Try to diagnose why — get geometry for first segment
        PCB_TRACK* firstTrack = nullptr;

        for( const PATH_POINT& pt : path )
        {
            if( !pt.isVia )
            {
                firstTrack = static_cast<PCB_TRACK*>( pt.item );
                break;
            }
        }

        wxString diag;

        if( firstTrack )
        {
            LAYER_GEOMETRY geom = stackup.GetLayerGeometry( firstTrack->GetLayer(),
                                                            firstTrack->GetStart(),
                                                            firstTrack->GetWidth() );

            diag.Printf( _( "Z0=0: layer=%d, w=%.3fmm, hAbove=%.3fmm, hBelow=%.3fmm, "
                            "erAbove=%.1f, erBelow=%.1f, refAbove=%d, refBelow=%d, defaults=%d" ),
                         firstTrack->GetLayer(),
                         geom.traceWidth * 1e3,
                         geom.hAbove * 1e3, geom.hBelow * 1e3,
                         geom.erAbove, geom.erBelow,
                         geom.hasRefAbove, geom.hasRefBelow,
                         geom.usingDefaults );
        }
        else
        {
            diag = _( "Z0=0: no track segments found" );
        }

        updateStatus( diag );
        return;
    }

    updatePlot( positions, impedances );

    // Build status string
    wxString status;
    double   totalLen = walker.GetTotalLength() / 1e6; // mm

    if( result.isComplete() )
    {
        status.Printf( _( "%.1f mm, %d segments" ), totalLen, result.segmentsVisited );
    }
    else
    {
        status.Printf( _( "%.1f mm, %d/%d segments (stopped at branch)" ),
                       totalLen, result.segmentsVisited, result.totalSegmentsOnNet );
    }

    double minZ = *std::min_element( impedances.begin(), impedances.end() );
    double maxZ = *std::max_element( impedances.begin(), impedances.end() );

    status += wxString::Format( wxS( " | Z0: %.1f\u2013%.1f \u03A9" ), minZ, maxZ );

    if( stackup.GetLayerGeometry( static_cast<PCB_TRACK*>( path.front().item )->GetLayer(),
                                  path.front().position,
                                  static_cast<PCB_TRACK*>( path.front().item )->GetWidth() )
                .usingDefaults )
    {
        status += _( " (default stackup)" );
    }

    updateStatus( status );
}


void IMPEDANCE_PROFILER_PANEL::updatePlot( const std::vector<double>& aPositions,
                                           const std::vector<double>& aImpedances )
{
    m_impedanceTrace->SetData( aPositions, aImpedances );
    m_impedanceTrace->SetVisible( true );

    // Draw target impedance line across the full range
    if( !aPositions.empty() && m_targetZ0 > 0.0 )
    {
        std::vector<double> targetX = { aPositions.front(), aPositions.back() };
        std::vector<double> targetY = { m_targetZ0, m_targetZ0 };
        m_targetLine->SetData( targetX, targetY );
        m_targetLine->SetVisible( true );
    }

    // Propagate data extents to the scale objects — this is essential for
    // mpFXY::Plot to map data coordinates to pixels correctly.
    m_xAxis->ResetDataRange();
    m_yAxis->ResetDataRange();
    m_impedanceTrace->UpdateScales();

    if( m_targetLine->IsVisible() )
        m_targetLine->UpdateScales();

    // Extend Y range to include target and add padding
    double yMin = *std::min_element( aImpedances.begin(), aImpedances.end() );
    double yMax = *std::max_element( aImpedances.begin(), aImpedances.end() );

    if( m_targetZ0 > 0.0 )
    {
        yMin = std::min( yMin, m_targetZ0 );
        yMax = std::max( yMax, m_targetZ0 );
    }

    double yPad = std::max( ( yMax - yMin ) * 0.1, 5.0 );
    m_yAxis->ExtendDataRange( yMin - yPad, yMax + yPad );

    m_plotWindow->UpdateAll();
    m_plotWindow->Fit();
}


void IMPEDANCE_PROFILER_PANEL::updateStatus( const wxString& aText )
{
    m_statusText->SetLabel( aText );
}


// BOARD_LISTENER — schedule re-analysis on board changes
void IMPEDANCE_PROFILER_PANEL::OnBoardItemAdded( BOARD& aBoard, BOARD_ITEM* aBoardItem )
{
    if( IsShown() && m_currentNetCode > 0 )
        populateNetList();
}


void IMPEDANCE_PROFILER_PANEL::OnBoardItemsAdded( BOARD& aBoard,
                                                  std::vector<BOARD_ITEM*>& aBoardItems )
{
    if( IsShown() && m_currentNetCode > 0 )
        populateNetList();
}


void IMPEDANCE_PROFILER_PANEL::OnBoardItemRemoved( BOARD& aBoard, BOARD_ITEM* aBoardItem )
{
    if( IsShown() && m_currentNetCode > 0 )
        populateNetList();
}


void IMPEDANCE_PROFILER_PANEL::OnBoardItemsRemoved( BOARD& aBoard,
                                                    std::vector<BOARD_ITEM*>& aBoardItems )
{
    if( IsShown() && m_currentNetCode > 0 )
        populateNetList();
}


void IMPEDANCE_PROFILER_PANEL::OnBoardItemChanged( BOARD& aBoard, BOARD_ITEM* aBoardItem )
{
    if( IsShown() && m_currentNetCode > 0 )
        runAnalysis( m_currentNetCode );
}


void IMPEDANCE_PROFILER_PANEL::OnBoardItemsChanged( BOARD& aBoard,
                                                    std::vector<BOARD_ITEM*>& aBoardItems )
{
    if( IsShown() && m_currentNetCode > 0 )
        runAnalysis( m_currentNetCode );
}


void IMPEDANCE_PROFILER_PANEL::OnBoardHighlightNetChanged( BOARD& aBoard )
{
    // Could auto-switch to the highlighted net — for now, no-op
}
