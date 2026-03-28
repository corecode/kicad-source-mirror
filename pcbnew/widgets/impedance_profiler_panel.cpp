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

    m_impedanceTrace = new mpFXYVector( _( "Z0" ) );
    m_impedanceTrace->SetPen( tracePen );
    m_impedanceTrace->SetContinuity( true );
    wxPen targetPen( wxColour( 200, 50, 50 ), 1, wxPENSTYLE_SHORT_DASH );

    m_targetLine = new mpFXYVector( _( "Target" ) );
    m_targetLine->SetPen( targetPen );
    m_targetLine->SetContinuity( true );

    m_xAxis = new mpScaleX( _( "Position (mm)" ), mpALIGN_BOTTOM, true );
    m_yAxis = new mpScaleY( _( "Z0 (\u03A9)" ), mpALIGN_LEFT, true );

    m_plotWindow->AddLayer( m_xAxis );
    m_plotWindow->AddLayer( m_yAxis );
    m_plotWindow->AddLayer( m_impedanceTrace );
    m_plotWindow->AddLayer( m_targetLine );

    m_plotWindow->SetMargins( 15, 10, 30, 50 );

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

    // Build impedance profile
    STACKUP_READER stackup( board );

    std::vector<double> positions;
    std::vector<double> impedances;

    // Track which segment we're on to avoid redundant Z0 computation
    BOARD_CONNECTED_ITEM* lastItem = nullptr;
    double                lastZ0 = 0.0;

    for( const PATH_POINT& pt : path )
    {
        if( pt.isVia )
            continue;

        double z0;

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
            z0 = STACKUP_READER::ComputeZ0( geom );
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

    if( !impedances.empty() )
    {
        double minZ = *std::min_element( impedances.begin(), impedances.end() );
        double maxZ = *std::max_element( impedances.begin(), impedances.end() );

        status += wxString::Format( wxS( " | Z0: %.1f–%.1f \u03A9" ), minZ, maxZ );
    }

    updateStatus( status );
}


void IMPEDANCE_PROFILER_PANEL::updatePlot( const std::vector<double>& aPositions,
                                           const std::vector<double>& aImpedances )
{
    m_impedanceTrace->SetData( aPositions, aImpedances );

    // Draw target impedance line across the full range
    if( !aPositions.empty() && m_targetZ0 > 0.0 )
    {
        std::vector<double> targetX = { aPositions.front(), aPositions.back() };
        std::vector<double> targetY = { m_targetZ0, m_targetZ0 };
        m_targetLine->SetData( targetX, targetY );
    }

    m_plotWindow->Fit();
    m_plotWindow->Refresh();
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
