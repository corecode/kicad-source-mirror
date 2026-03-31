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
#include <footprint.h>
#include <pad.h>
#include <pcb_track.h>
#include <zone.h>
#include <pcb_edit_frame.h>
#include <widgets/mathplot.h>

#include <drc/drc_rtree.h>
#include <sipi/cross_section_builder.h>
#include <sipi/trace_path_walker.h>
#include <sipi/stackup_reader.h>
#include <sipi/analytical_impedance.h>
#include <sipi/bem_2d_solver.h>

#include <chrono>

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/dcbuffer.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include <algorithm>
#include <set>


// ============================================================================
// X/Y axis scale classes
// ============================================================================

class PROFILER_SCALE_X : public mpScaleX
{
public:
    PROFILER_SCALE_X() : mpScaleX( wxS( "mm" ), mpALIGN_BOTTOM, true ) {}

protected:
    void formatLabels() override
    {
        for( auto& tl : m_tickLabels )
        {
            if( tl.visible )
                tl.label = wxString::Format( wxS( "%.1f" ), tl.pos );
        }
    }
};


class PROFILER_SCALE_Y : public mpScaleY
{
public:
    PROFILER_SCALE_Y() : mpScaleY( wxS( "\u03A9" ), mpALIGN_LEFT, true ) {}

protected:
    void formatLabels() override
    {
        for( auto& tl : m_tickLabels )
        {
            if( tl.visible )
                tl.label = wxString::Format( wxS( "%.1f" ), tl.pos );
        }
    }
};


// ============================================================================
// XS_VIEW_PANEL — cross-section rendering
// ============================================================================

XS_VIEW_PANEL::XS_VIEW_PANEL( wxWindow* aParent ) :
        wxPanel( aParent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                 wxFULL_REPAINT_ON_RESIZE ),
        m_sample( nullptr ),
        m_xExtent( 0.0 )
{
    SetBackgroundStyle( wxBG_STYLE_PAINT );
    SetMinSize( wxSize( -1, 120 ) );
    Bind( wxEVT_PAINT, &XS_VIEW_PANEL::onPaint, this );
}


void XS_VIEW_PANEL::SetSample( const XS_SAMPLE* aSample )
{
    m_sample = aSample;
    Refresh();
}


void XS_VIEW_PANEL::onPaint( wxPaintEvent& aEvent )
{
    wxAutoBufferedPaintDC dc( this );
    wxRect rect = GetClientRect();

    dc.SetBackground( wxBrush( wxColour( 40, 40, 45 ) ) );
    dc.Clear();

    if( m_sample && !m_sample->geometry.conductors.empty() )
        drawCrossSection( dc, rect );
    else
    {
        dc.SetTextForeground( wxColour( 140, 140, 140 ) );
        dc.DrawText( _( "Click on the impedance plot to view cross-section" ),
                     rect.x + 10, rect.y + rect.height / 2 - 8 );
    }
}


void XS_VIEW_PANEL::drawCrossSection( wxDC& aDC, const wxRect& aRect )
{
    const XS_GEOMETRY& geom = m_sample->geometry;
    const XS_CONDUCTOR& sig = geom.conductors[0];

    // --- Compute Y extent from conductors + ground planes ---
    double yMin = sig.centerY - sig.thickness / 2.0;
    double yMax = sig.centerY + sig.thickness / 2.0;

    yMin = std::min( yMin, geom.groundY );
    yMax = std::max( yMax, geom.groundY );

    if( geom.hasUpperGround )
    {
        yMin = std::min( yMin, geom.upperGroundY );
        yMax = std::max( yMax, geom.upperGroundY );
    }

    // Include dielectric regions that are near the geometry (skip far-away air)
    double ySpan = yMax - yMin;

    for( const XS_DIELECTRIC_REGION& dr : geom.dielectrics )
    {
        double drLo = std::min( dr.yTop, dr.yBottom );
        double drHi = std::max( dr.yTop, dr.yBottom );

        if( drLo > yMin - ySpan * 3.0 && drHi < yMax + ySpan * 3.0 )
        {
            yMin = std::min( yMin, drLo );
            yMax = std::max( yMax, drHi );
        }
    }

    double yPad = ( yMax - yMin ) * 0.2;
    yMin -= yPad;
    yMax += yPad;

    // --- X extent: fixed across all samples for comparability ---
    double xHalf = m_xExtent;

    if( xHalf < 1e-9 )
    {
        // Fallback: auto from geometry
        for( const XS_CONDUCTOR& c : geom.conductors )
            xHalf = std::max( xHalf, std::abs( c.centerX ) + c.width );

        xHalf *= 1.5;
    }

    double xMin = -xHalf;
    double xMax = xHalf;

    // --- Mapping ---
    int margin = 8;
    int topMargin = 20; // room for info text
    int drawW = aRect.width - 2 * margin;
    int drawH = aRect.height - margin - topMargin;

    if( drawW < 20 || drawH < 20 )
        return;

    double sX = drawW / ( xMax - xMin );
    double sY = drawH / ( yMax - yMin );

    int offX = margin;
    int offY = topMargin;

    auto toPixX = [&]( double x ) -> int { return offX + (int) ( ( x - xMin ) * sX ); };
    auto toPixY = [&]( double y ) -> int { return offY + (int) ( ( y - yMin ) * sY ); };

    // --- Dielectric regions ---
    for( const XS_DIELECTRIC_REGION& dr : geom.dielectrics )
    {
        double drLo = std::min( dr.yTop, dr.yBottom );
        double drHi = std::max( dr.yTop, dr.yBottom );

        if( drLo < yMin || drHi > yMax )
            continue;

        int shade = 60 + (int) ( dr.epsilonR * 12 );
        shade = std::min( shade, 140 );

        aDC.SetBrush( wxBrush( wxColour( shade, shade + 10, shade - 10 ) ) );
        aDC.SetPen( wxPen( wxColour( 80, 80, 80 ), 1 ) );

        int py1 = toPixY( drHi );
        int py2 = toPixY( drLo );
        aDC.DrawRectangle( toPixX( xMin ), py1, drawW, py2 - py1 );

        wxString erLabel = wxString::Format( wxS( "\u03B5r=%.1f" ), dr.epsilonR );
        aDC.SetTextForeground( wxColour( 160, 160, 140 ) );
        wxSize ts = aDC.GetTextExtent( erLabel );
        aDC.DrawText( erLabel, toPixX( xMin ) + 4, ( py1 + py2 - ts.GetHeight() ) / 2 );
    }

    // --- Ground plane(s) ---
    aDC.SetPen( wxPen( wxColour( 200, 180, 60 ), 3 ) );
    int gndPy = toPixY( geom.groundY );
    aDC.DrawLine( toPixX( xMin ), gndPy, toPixX( xMax ), gndPy );

    // "GND" label
    aDC.SetTextForeground( wxColour( 200, 180, 60 ) );
    aDC.DrawText( wxS( "GND" ), toPixX( xMax ) - 30, gndPy - 14 );

    if( geom.hasUpperGround )
    {
        int ugPy = toPixY( geom.upperGroundY );
        aDC.DrawLine( toPixX( xMin ), ugPy, toPixX( xMax ), ugPy );
        aDC.DrawText( wxS( "GND" ), toPixX( xMax ) - 30, ugPy + 2 );
    }

    // --- Conductors ---
    for( int ci = 0; ci < (int) geom.conductors.size(); ci++ )
    {
        const XS_CONDUCTOR& c = geom.conductors[ci];

        int px = toPixX( c.centerX - c.width / 2.0 );
        int py = toPixY( c.centerY + c.thickness / 2.0 );
        int pw = std::max( toPixX( c.centerX + c.width / 2.0 ) - px, 3 );
        int ph = std::max( toPixY( c.centerY - c.thickness / 2.0 ) - py, 2 );

        if( c.isGround )
        {
            // Groundwire — gold color matching ground plane lines
            aDC.SetBrush( wxBrush( wxColour( 180, 160, 50 ) ) );
            aDC.SetPen( wxPen( wxColour( 200, 180, 60 ), 1 ) );
        }
        else if( ci == 0 )
        {
            aDC.SetBrush( wxBrush( wxColour( 220, 140, 40 ) ) );
            aDC.SetPen( wxPen( wxColour( 255, 180, 60 ), 2 ) );
        }
        else
        {
            aDC.SetBrush( wxBrush( wxColour( 160, 120, 60 ) ) );
            aDC.SetPen( wxPen( wxColour( 180, 140, 70 ), 1 ) );
        }

        aDC.DrawRectangle( px, py, pw, ph );

        // Width label below conductor (skip for groundwires — they clutter)
        if( !c.isGround )
        {
            wxString wLabel = wxString::Format( wxS( "w=%.0f\u00B5m" ), c.width * 1e6 );
            aDC.SetTextForeground( ci == 0 ? wxColour( 255, 200, 100 )
                                           : wxColour( 180, 160, 120 ) );
            wxSize ws = aDC.GetTextExtent( wLabel );
            aDC.DrawText( wLabel, px + pw / 2 - ws.GetWidth() / 2, py + ph + 2 );
        }

        // Edge-to-edge distance dimension line from signal to neighbor
        if( ci > 0 && !c.isGround )
        {
            double sigRight = sig.centerX + sig.width / 2.0;
            double sigLeft = sig.centerX - sig.width / 2.0;
            double nbRight = c.centerX + c.width / 2.0;
            double nbLeft = c.centerX - c.width / 2.0;

            // Pick the facing edges
            double edgeA, edgeB;

            if( c.centerX > sig.centerX )
            {
                edgeA = sigRight;
                edgeB = nbLeft;
            }
            else
            {
                edgeA = sigLeft;
                edgeB = nbRight;
            }

            double edgeDist = std::abs( edgeB - edgeA );

            // Draw dimension line at conductor mid-height
            int dimY = py + ph / 2;
            int dimX1 = toPixX( edgeA );
            int dimX2 = toPixX( edgeB );

            aDC.SetPen( wxPen( wxColour( 140, 200, 140 ), 1, wxPENSTYLE_SHORT_DASH ) );
            aDC.DrawLine( dimX1, dimY, dimX2, dimY );
            // End ticks
            aDC.DrawLine( dimX1, dimY - 4, dimX1, dimY + 4 );
            aDC.DrawLine( dimX2, dimY - 4, dimX2, dimY + 4 );

            wxString dLabel = wxString::Format( wxS( "%.0f\u00B5m" ), edgeDist * 1e6 );
            aDC.SetTextForeground( wxColour( 140, 200, 140 ) );
            wxSize ds = aDC.GetTextExtent( dLabel );
            aDC.DrawText( dLabel, ( dimX1 + dimX2 - ds.GetWidth() ) / 2, dimY - ds.GetHeight() - 2 );
        }
    }

    // --- Info text ---
    wxString info = wxString::Format( wxS( "Z\u2080=%.1f\u03A9 @ %.2fmm" ),
                                      m_sample->z0, m_sample->distMm );

    int nbCount = 0;
    int gwCount = 0;

    for( size_t i = 1; i < geom.conductors.size(); i++ )
    {
        if( geom.conductors[i].isGround )
            gwCount++;
        else
            nbCount++;
    }

    if( nbCount > 0 )
        info += wxString::Format( wxS( " %dnb" ), nbCount );

    if( gwCount > 0 )
        info += wxString::Format( wxS( " %dgw" ), gwCount );

    // Reference plane diagnostics
    wxString refInfo;

    if( m_sample->hasRefAbove )
        refInfo += wxString::Format( wxS( " h\u2191%.0f\u00B5m" ), m_sample->hAbove * 1e6 );

    if( m_sample->hasRefBelow )
        refInfo += wxString::Format( wxS( " h\u2193%.0f\u00B5m" ), m_sample->hBelow * 1e6 );

    if( !m_sample->hasRefAbove && !m_sample->hasRefBelow )
        refInfo = wxS( " NO REF PLANES" );

    aDC.SetTextForeground( wxColour( 220, 220, 220 ) );
    aDC.DrawText( info + refInfo, aRect.x + 6, aRect.y + 4 );
}


// ============================================================================
// IMPEDANCE_PROFILER_PANEL
// ============================================================================

IMPEDANCE_PROFILER_PANEL::IMPEDANCE_PROFILER_PANEL( PCB_EDIT_FRAME* aParent ) :
        WX_PANEL( aParent ),
        m_frame( aParent ),
        m_initialized( false ),
        m_netSelector( nullptr ),
        m_targetZ0Input( nullptr ),
        m_xAxisUnitSelector( nullptr ),
        m_statusText( nullptr ),
        m_xAxisIsTime( false ),
        m_plotWindow( nullptr ),
        m_impedanceTrace( nullptr ),
        m_targetLine( nullptr ),
        m_cursorLine( nullptr ),
        m_xAxis( nullptr ),
        m_yAxis( nullptr ),
        m_xsView( nullptr ),
        m_selectedSample( -1 ),
        m_currentNetCode( -1 ),
        m_targetZ0( 50.0 )
{
    // Defer buildUI() and listener registration to OnShowPanel() so that
    // pcbnew startup doesn't pay for mpWindow / plot-layer construction
    // when the panel is hidden.
}


IMPEDANCE_PROFILER_PANEL::~IMPEDANCE_PROFILER_PANEL()
{
    // If we never initialized, no listener was registered — nothing to clean up.
    // If we did initialize, the board listener is already removed by
    // PCB_EDIT_FRAME::~PCB_EDIT_FRAME() calling RemoveAllListeners() before
    // m_pcb is deleted.  Don't call GetBoard() here — m_pcb may already be
    // null (assert) since wxAuiManager destroys child panels after
    // ~PCB_BASE_FRAME deletes m_pcb.
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

    topSizer->AddStretchSpacer();

    topSizer->Add( new wxStaticText( this, wxID_ANY, _( "X:" ) ),
                   0, wxALIGN_CENTER_VERTICAL | wxLEFT, 8 );

    m_xAxisUnitSelector = new wxChoice( this, wxID_ANY );
    m_xAxisUnitSelector->Append( _( "mm" ) );
    m_xAxisUnitSelector->Append( _( "ps" ) );
    m_xAxisUnitSelector->SetSelection( 0 );
    topSizer->Add( m_xAxisUnitSelector, 0, wxALIGN_CENTER_VERTICAL | wxLEFT, 2 );

    mainSizer->Add( topSizer, 0, wxEXPAND | wxALL, 4 );

    // Plot
    m_plotWindow = new mpWindow( this, wxID_ANY );
    m_plotWindow->SetMinSize( wxSize( 200, 150 ) );
    m_plotWindow->EnableDoubleBuffer( true );

    wxPen tracePen( wxColour( 0, 120, 200 ), 2, wxPENSTYLE_SOLID );

    m_xAxis = new PROFILER_SCALE_X();
    m_yAxis = new PROFILER_SCALE_Y();

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

    // Cursor line (vertical marker at selected sample)
    wxPen cursorPen( wxColour( 255, 200, 60 ), 1, wxPENSTYLE_LONG_DASH );

    m_cursorLine = new mpFXYVector( _( "Cursor" ) );
    m_cursorLine->SetPen( cursorPen );
    m_cursorLine->SetContinuity( true );
    m_cursorLine->SetScale( m_xAxis, m_yAxis );
    m_cursorLine->SetVisible( false );
    m_cursorLine->ShowName( false );
    m_plotWindow->AddLayer( m_cursorLine, false );

    m_plotWindow->SetMargins( 15, 10, 30, 50 );
    m_plotWindow->UpdateAll();

    mainSizer->Add( m_plotWindow, 3, wxEXPAND | wxLEFT | wxRIGHT, 4 );

    // Cross-section view
    m_xsView = new XS_VIEW_PANEL( this );
    mainSizer->Add( m_xsView, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 4 );

    // Status bar
    m_statusText = new wxStaticText( this, wxID_ANY, _( "Select a net and click Analyse." ) );
    mainSizer->Add( m_statusText, 0, wxEXPAND | wxALL, 4 );

    SetSizer( mainSizer );

    // Bind events
    analyseBtn->Bind( wxEVT_BUTTON, &IMPEDANCE_PROFILER_PANEL::onAnalyseClicked, this );
    m_netSelector->Bind( wxEVT_CHOICE, &IMPEDANCE_PROFILER_PANEL::onNetSelected, this );
    m_xAxisUnitSelector->Bind( wxEVT_CHOICE, &IMPEDANCE_PROFILER_PANEL::onXAxisUnitChanged, this );
    m_plotWindow->Bind( wxEVT_MOTION, &IMPEDANCE_PROFILER_PANEL::onPlotClick, this );
}


void IMPEDANCE_PROFILER_PANEL::OnShowPanel()
{
    if( !m_initialized )
    {
        buildUI();

        if( BOARD* board = m_frame->GetBoard() )
            board->AddListener( this );

        m_initialized = true;
    }

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

    double target = 50.0;
    m_targetZ0Input->GetValue().ToDouble( &target );
    m_targetZ0 = target;

    runAnalysis( static_cast<int>( netCode ) );
}


void IMPEDANCE_PROFILER_PANEL::onNetSelected( wxCommandEvent& aEvent )
{
    wxCommandEvent dummy;
    onAnalyseClicked( dummy );
}


void IMPEDANCE_PROFILER_PANEL::onXAxisUnitChanged( wxCommandEvent& aEvent )
{
    m_xAxisIsTime = ( m_xAxisUnitSelector->GetSelection() == 1 );

    if( !m_samples.empty() )
        updatePlot();
}


void IMPEDANCE_PROFILER_PANEL::onPlotClick( wxMouseEvent& aEvent )
{
    if( m_samples.empty() )
    {
        aEvent.Skip();
        return;
    }

    // Convert pixel X → normalized plot coord → data coord (mm along trace).
    double normX = m_plotWindow->p2x( aEvent.GetX() );
    double plotX = m_xAxis->TransformFromPlot( normX );

    // Find the nearest sample
    int bestIdx = 0;
    double bestDist = 1e9;

    for( int i = 0; i < (int) m_samples.size(); i++ )
    {
        double sampleX = m_xAxisIsTime ? m_samples[i].timePsec : m_samples[i].distMm;
        double d = std::abs( sampleX - plotX );

        if( d < bestDist )
        {
            bestDist = d;
            bestIdx = i;
        }
    }

    selectSample( bestIdx );
    aEvent.Skip();
}


void IMPEDANCE_PROFILER_PANEL::selectSample( int aIndex )
{
    if( aIndex < 0 || aIndex >= (int) m_samples.size() )
        return;

    m_selectedSample = aIndex;
    m_xsView->SetSample( &m_samples[aIndex] );

    // Update cursor line on the plot (vertical line at the sample position)
    if( !m_samples.empty() )
    {
        double xPos = m_xAxisIsTime ? m_samples[aIndex].timePsec
                                     : m_samples[aIndex].distMm;

        // Find Y range from impedance data
        double yMin = 1e9, yMax = -1e9;

        for( const XS_SAMPLE& s : m_samples )
        {
            yMin = std::min( yMin, s.z0 );
            yMax = std::max( yMax, s.z0 );
        }

        double yPad = ( yMax - yMin ) * 0.15;
        std::vector<double> cx = { xPos, xPos };
        std::vector<double> cy = { yMin - yPad, yMax + yPad };

        m_cursorLine->SetData( cx, cy );
        m_cursorLine->SetVisible( true );
        m_plotWindow->UpdateAll();
        m_plotWindow->Refresh();
    }

    // Highlight the sample location on the board
    m_frame->FocusOnLocation( m_samples[aIndex].boardPos );
    m_frame->GetCanvas()->Refresh();
}


void IMPEDANCE_PROFILER_PANEL::runAnalysis( int aNetCode )
{
    m_currentNetCode = aNetCode;
    m_samples.clear();
    m_selectedSample = -1;

    BOARD* board = m_frame->GetBoard();

    if( !board )
        return;

    TRACE_PATH_WALKER walker( board );

    if( !walker.WalkNet( aNetCode ) )
    {
        updateStatus( _( "Could not walk net — no routed tracks found." ) );
        return;
    }

    const auto& path = walker.GetPath();
    const WALK_RESULT& result = walker.GetResult();

    STACKUP_READER stackup( board );

    // Cache keyed on quantized cross-section to avoid redundant BEM solves.
    struct XS_KEY
    {
        PCB_LAYER_ID       layer;
        int                width;
        bool               refAbove;
        bool               refBelow;
        std::vector<int>   neighborKeys;

        bool operator<( const XS_KEY& o ) const
        {
            if( layer != o.layer ) return layer < o.layer;
            if( width != o.width ) return width < o.width;
            if( refAbove != o.refAbove ) return refAbove < o.refAbove;
            if( refBelow != o.refBelow ) return refBelow < o.refBelow;
            return neighborKeys < o.neighborKeys;
        }
    };

    struct XS_CACHED { double z0; double erEff; };
    std::map<XS_KEY, XS_CACHED> z0Cache;

    // Build spatial index for neighbor search with tracks and pads.
    DRC_RTREE tempRtree;

    for( PCB_TRACK* t : board->Tracks() )
    {
        if( t->Type() == PCB_VIA_T )
        {
            // Insert vias on all their layers so they appear as neighbors
            for( PCB_LAYER_ID layer : t->GetLayerSet().CuStack() )
                tempRtree.Insert( t, layer );
        }
        else
        {
            tempRtree.Insert( t, t->GetLayer() );
        }
    }

    for( FOOTPRINT* fp : board->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            for( PCB_LAYER_ID layer : pad->GetLayerSet().CuStack() )
                tempRtree.Insert( pad, layer );
        }
    }

    DRC_RTREE* rtree = &tempRtree;

    // Sample at uniform distance intervals along the path, interpolating
    // position between path points.  Interpolation along straight segments
    // is exact (linear between endpoints).
    double totalLen = walker.GetTotalLength(); // nm

    if( totalLen < 1.0 )
    {
        updateStatus( _( "Trace has zero length." ) );
        return;
    }

    double sampleStep = std::max( totalLen / 500.0, 50000.0 ); // ~500 samples, min 50µm
    int    numSamples = std::max( 2, (int) ceil( totalLen / sampleStep ) + 1 );

    // Build non-via path index for interpolation
    struct PATH_SEG
    {
        double       dist;     ///< Path distance at this point
        VECTOR2I     pos;      ///< Board position
        VECTOR2D     tangent;  ///< Unit tangent
        PCB_TRACK*   track;    ///< Source track segment
    };

    std::vector<PATH_SEG> segs;

    for( const PATH_POINT& pp : path )
    {
        if( pp.isVia )
            continue;

        segs.push_back( { pp.distFromStart, pp.position, pp.tangent,
                          static_cast<PCB_TRACK*>( pp.item ) } );
    }

    if( segs.empty() )
    {
        updateStatus( _( "No impedance data — trace has no non-via segments." ) );
        return;
    }

    // Map path items → path distance for topological neighbor exclusion.
    std::map<BOARD_CONNECTED_ITEM*, double> pathItemDist;

    for( const PATH_POINT& pp : path )
    {
        if( !pp.isVia && pathItemDist.find( pp.item ) == pathItemDist.end() )
            pathItemDist[pp.item] = pp.distFromStart;
    }

    int    segIdx = 0;
    double cumulativeTimePsec = 0.0;
    long   usXsTotal = 0, usBemTotal = 0;
    int    bemSolves = 0, cacheHits = 0;

    for( int si = 0; si < numSamples; si++ )
    {
        double sampleDist = si * sampleStep;

        if( sampleDist > totalLen )
            sampleDist = totalLen;

        // Advance to the segment containing this distance
        while( segIdx + 1 < (int) segs.size()
               && segs[segIdx + 1].dist <= sampleDist )
        {
            segIdx++;
        }

        const PATH_SEG& seg = segs[segIdx];

        // Interpolate position along the segment (exact for straight tracks)
        VECTOR2I samplePos = seg.pos;

        if( segIdx + 1 < (int) segs.size() )
        {
            const PATH_SEG& next = segs[segIdx + 1];
            double segLen = next.dist - seg.dist;

            if( segLen > 1.0 )
            {
                double frac = ( sampleDist - seg.dist ) / segLen;
                frac = std::clamp( frac, 0.0, 1.0 );
                samplePos.x = seg.pos.x + (int) ( frac * ( next.pos.x - seg.pos.x ) );
                samplePos.y = seg.pos.y + (int) ( frac * ( next.pos.y - seg.pos.y ) );
            }
        }

        VECTOR2D     sampleTangent = seg.tangent;
        PCB_TRACK*   track = seg.track;
        int          signalWidth = track->GetWidth(); // may be overridden by pad

        // Check if the sample point is inside a pad on the signal net.
        // If so, the effective conductor width is the pad's cross-section extent.
        for( PAD* pad : board->GetPads() )
        {
            if( pad->GetNetCode() != track->GetNetCode() )
                continue;

            if( !pad->IsOnLayer( track->GetLayer() ) )
                continue;

            if( pad->HitTest( samplePos, 0 ) )
            {
                // Pad contains our sample point — use pad extent along the cut
                // line as the effective signal width.
                VECTOR2D normal0( -sampleTangent.y, sampleTangent.x );
                BOX2I    padBBox = pad->GetBoundingBox();

                // Project pad bounding box corners onto the cut-line normal
                // to get the pad's cross-section width.
                double minProj = 1e18, maxProj = -1e18;

                for( const VECTOR2I& corner :
                     { padBBox.GetOrigin(),
                       padBBox.GetOrigin() + VECTOR2I( padBBox.GetWidth(), 0 ),
                       padBBox.GetOrigin() + VECTOR2I( 0, padBBox.GetHeight() ),
                       padBBox.GetEnd() } )
                {
                    VECTOR2D d( corner.x - samplePos.x, corner.y - samplePos.y );
                    double proj = d.x * normal0.x + d.y * normal0.y;
                    minProj = std::min( minProj, proj );
                    maxProj = std::max( maxProj, proj );
                }

                int padWidth = (int) ( maxProj - minProj );

                if( padWidth > signalWidth )
                {
                    fprintf( stderr, "SIPI: pad expansion at d=%.3fmm pos=(%d,%d) "
                                     "pad=%s w=%d→%d\n",
                             sampleDist / 1e6, samplePos.x, samplePos.y,
                             (const char*) pad->GetNumber().utf8_str(),
                             signalWidth, padWidth );
                    signalWidth = padWidth;
                }

                break;
            }
        }

        LAYER_GEOMETRY geom = stackup.GetLayerGeometry( track->GetLayer(),
                                                        samplePos,
                                                        signalWidth );

        // Coupling horizon: 3× the nearest reference plane distance (edge-to-edge).
        // Cap at 3mm to avoid huge searches when virtual earth is active.
        double hRef = std::max( geom.hAbove, geom.hBelow );
        int    couplingHorizon = std::clamp( (int) ( hRef * 3.0 * 1e9 ), 500000, 3000000 );

        // Use the tested CROSS_SECTION_BUILDER for neighbor detection + geometry
        CROSS_SECTION_BUILDER xsBuilder;
        xsBuilder.SetSpatialIndex( rtree );
        xsBuilder.SetBoard( board );
        xsBuilder.SetPathItemDistances( &pathItemDist );
        xsBuilder.SetSignalTrack( track );

        XS_BUILD_PARAMS xsParams;
        xsParams.samplePos = samplePos;
        xsParams.sampleTangent = sampleTangent;
        xsParams.signalLayer = track->GetLayer();
        xsParams.signalNetCode = track->GetNetCode();
        xsParams.signalWidth = signalWidth;
        xsParams.couplingHorizon = couplingHorizon;
        xsParams.sampleDist = sampleDist;
        xsParams.layerGeom = geom;

        auto t0 = std::chrono::steady_clock::now();

        auto neighbors = xsBuilder.FindNeighbors( xsParams );
        auto groundWires = xsBuilder.FindGroundWires( xsParams, geom );

        // Update params with potentially modified geometry (reference demotion)
        xsParams.layerGeom = geom;

        XS_GEOMETRY xs = xsBuilder.BuildGeometry( xsParams, neighbors, groundWires );

        auto t1 = std::chrono::steady_clock::now();
        usXsTotal += std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count();

        // Build cache key from neighbor and groundwire configuration
        XS_KEY key;
        key.layer = track->GetLayer();
        key.width = signalWidth;
        key.refAbove = geom.hasRefAbove;
        key.refBelow = geom.hasRefBelow;

        for( const XS_NEIGHBOR& nb : neighbors )
        {
            key.neighborKeys.push_back( nb.distNm / 10000 );
            key.neighborKeys.push_back( nb.widthNm );
        }

        for( const XS_GROUNDWIRE& gw : groundWires )
        {
            key.neighborKeys.push_back( gw.lateralNm / 10000 );
            key.neighborKeys.push_back( gw.widthNm );
        }

        // Solve (with cache)
        double z0 = 0.0;
        double erEff = 1.0;
        auto cacheIt = z0Cache.find( key );

        if( cacheIt != z0Cache.end() )
        {
            z0 = cacheIt->second.z0;
            erEff = cacheIt->second.erEff;
            cacheHits++;
        }
        else
        {
            auto t2 = std::chrono::steady_clock::now();
            BEM_2D_SOLVER solver;
            solver.SetGeometry( xs );
            solver.SetPanelsPerEdge( 12 );

            bool bemOk = solver.Solve();

            if( bemOk && solver.GetResult().Z0 > 0.0 )
            {
                z0 = solver.GetResult().Z0;
                erEff = std::max( solver.GetResult().erEff, 1.0 );
            }
            else
            {
                // BEM failed — leave z0=0 so the failure is visible in the plot.
                // Common causes: no reference plane at this position (antipad,
                // ground slot), singular geometry, or unfilled zones.
                fprintf( stderr, "SIPI: BEM failed at d=%.3fmm "
                                 "bemOk=%d nb=%d w=%d refAbove=%d refBelow=%d\n",
                         sampleDist / 1e6, bemOk,
                         (int) neighbors.size(), signalWidth,
                         geom.hasRefAbove, geom.hasRefBelow );
            }

            z0Cache[key] = { z0, erEff };

            auto t3 = std::chrono::steady_clock::now();
            usBemTotal += std::chrono::duration_cast<std::chrono::microseconds>( t3 - t2 ).count();
            bemSolves++;
        }

        // Compute cumulative propagation delay
        static constexpr double C_LIGHT_M_S = 299792458.0;
        double velocity = C_LIGHT_M_S / sqrt( erEff );
        double stepMeters = sampleStep * 1e-9; // nm → m
        cumulativeTimePsec += ( stepMeters / velocity ) * 1e12; // s → ps

        // Store sample with full geometry
        XS_SAMPLE sample;
        sample.distMm = sampleDist / 1e6; // nm → mm
        sample.timePsec = cumulativeTimePsec;
        sample.z0 = z0;
        sample.geometry = xs;
        sample.boardPos = samplePos;
        sample.hasRefAbove = geom.hasRefAbove;
        sample.hasRefBelow = geom.hasRefBelow;
        sample.hAbove = geom.hAbove;
        sample.hBelow = geom.hBelow;
        m_samples.push_back( sample );
    }

    if( m_samples.empty() )
    {
        updateStatus( _( "No impedance data — trace has no non-via segments." ) );
        return;
    }

    // Compute the max X extent across all samples so the XS view uses a fixed
    // horizontal scale, making different points along the trace comparable.
    double maxXExtent = 0.0;

    for( const XS_SAMPLE& s : m_samples )
    {
        for( const XS_CONDUCTOR& c : s.geometry.conductors )
            maxXExtent = std::max( maxXExtent, std::abs( c.centerX ) + c.width );
    }

    m_xsView->SetXExtent( maxXExtent * 1.5 );

    updatePlot();

    // Select the middle sample to show an initial cross-section
    selectSample( (int) m_samples.size() / 2 );

    // Build status string
    wxString status;
    double   totalLenMm = totalLen / 1e6;

    double totalTimePsec = m_samples.empty() ? 0.0 : m_samples.back().timePsec;

    if( result.isComplete() )
        status.Printf( _( "%.1f mm (%.0f ps), %d segs" ),
                       totalLenMm, totalTimePsec, result.segmentsVisited );
    else
        status.Printf( _( "%.1f mm (%.0f ps), %d/%d segs" ),
                       totalLenMm, totalTimePsec,
                       result.segmentsVisited, result.totalSegmentsOnNet );

    double minZ = 1e9, maxZ = 0.0;

    for( const XS_SAMPLE& s : m_samples )
    {
        minZ = std::min( minZ, s.z0 );
        maxZ = std::max( maxZ, s.z0 );
    }

    status += wxString::Format( wxS( " | Z0: %.2f\u2013%.2f \u03A9 (\u0394%.2f)" ),
                                minZ, maxZ, maxZ - minZ );

    int samplesWithNeighbors = 0;

    for( const XS_SAMPLE& s : m_samples )
    {
        if( s.geometry.conductors.size() > 1 )
            samplesWithNeighbors++;
    }

    status += wxString::Format( wxS( " | %d/%d pts with neighbors, %d unique XS" ),
                                samplesWithNeighbors, (int) m_samples.size(),
                                (int) z0Cache.size() );

    fprintf( stderr, "SIPI timing: XS=%.1fms BEM=%.1fms (%d solves, %d cached)\n",
             usXsTotal / 1000.0, usBemTotal / 1000.0, bemSolves, cacheHits );

    updateStatus( status );
}


void IMPEDANCE_PROFILER_PANEL::updatePlot()
{
    std::vector<double> positions;
    std::vector<double> impedances;

    for( const XS_SAMPLE& s : m_samples )
    {
        positions.push_back( m_xAxisIsTime ? s.timePsec : s.distMm );
        impedances.push_back( s.z0 );
    }

    m_impedanceTrace->SetData( positions, impedances );
    m_impedanceTrace->SetVisible( true );
    m_targetLine->SetVisible( false );
    m_cursorLine->SetVisible( false );

    double yMin = *std::min_element( impedances.begin(), impedances.end() );
    double yMax = *std::max_element( impedances.begin(), impedances.end() );

    if( yMax - yMin < 0.5 )
    {
        double mid = ( yMax + yMin ) / 2.0;
        yMin = mid - 2.0;
        yMax = mid + 2.0;
    }

    double yPad = ( yMax - yMin ) * 0.15;

    // Fit with explicit data range — mpWindow::UpdateBBox() always returns 0–1
    // so the no-arg Fit() doesn't work.  Set the range on the scales manually
    // so tick labels match, then fit the viewport.
    // The scales transform data → normalized [0,1] via TransformToPlot.
    // mpWindow::Fit() then maps [0,1] to the viewport.
    // Set scale data ranges, then use no-arg Fit() (UpdateBBox returns 0–1).
    m_xAxis->ResetDataRange();
    m_yAxis->ResetDataRange();
    m_impedanceTrace->UpdateScales();
    m_yAxis->ExtendDataRange( yMin - yPad, yMax + yPad );
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
