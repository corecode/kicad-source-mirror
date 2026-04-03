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
#include <pcb_edit_frame.h>
#include <pcbnew_settings.h>
#include <widgets/mathplot.h>

#include <wx/dcbuffer.h>
#include <wx/menu.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include <algorithm>
#include <cmath>


// ============================================================================
// X/Y axis scale classes
// ============================================================================

class PROFILER_SCALE_X : public mpScaleX
{
public:
    PROFILER_SCALE_X() : mpScaleX( wxS( "mm" ), mpALIGN_BOTTOM, true ) {}

    void SetUnitLabel( const wxString& aLabel ) { SetName( aLabel ); }

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
// CROSSHAIR_LAYER — mpLayer that draws a crosshair at a data coordinate
// ============================================================================

class CROSSHAIR_LAYER : public mpLayer
{
public:
    CROSSHAIR_LAYER() : mpLayer()
    {
        m_type = mpLAYER_INFO;
        m_visible = false;
        SetPen( wxPen( wxColour( 255, 200, 60 ), 1, wxPENSTYLE_LONG_DASH ) );
    }

    void SetPosition( double aDataX, double aDataY, mpScaleX* aScaleX, mpScaleY* aScaleY )
    {
        m_dataX = aDataX;
        m_dataY = aDataY;
        m_scaleX = aScaleX;
        m_scaleY = aScaleY;
        m_visible = true;
    }

    void Plot( wxDC& dc, mpWindow& w ) override
    {
        if( !m_visible || !m_scaleX || !m_scaleY )
            return;

        dc.SetPen( m_pen );

        wxCoord left = w.GetMarginLeft();
        wxCoord right = w.GetScrX() - w.GetMarginRight();
        wxCoord top = w.GetMarginTop();
        wxCoord bottom = w.GetScrY() - w.GetMarginBottom();

        wxCoord pixX = w.x2p( m_scaleX->TransformToPlot( m_dataX ) );
        wxCoord pixY = w.y2p( m_scaleY->TransformToPlot( m_dataY ) );

        if( pixX >= left && pixX <= right )
            dc.DrawLine( pixX, top, pixX, bottom );

        if( pixY >= top && pixY <= bottom )
            dc.DrawLine( left, pixY, right, pixY );
    }

    bool HasBBox() const override { return false; }

private:
    double    m_dataX = 0.0;
    double    m_dataY = 0.0;
    mpScaleX* m_scaleX = nullptr;
    mpScaleY* m_scaleY = nullptr;
};


// ============================================================================
// XS_VIEW_PANEL — cross-section rendering
// ============================================================================

XS_VIEW_PANEL::XS_VIEW_PANEL( wxWindow* aParent ) :
        wxPanel( aParent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                 wxFULL_REPAINT_ON_RESIZE ),
        m_geometry( nullptr ),
        m_z0( 0.0 ),
        m_zdiff( 0.0 ),
        m_distMm( 0.0 ),
        m_isDiffPair( false ),
        m_xExtent( 0.0 )
{
    SetBackgroundStyle( wxBG_STYLE_PAINT );
    SetMinSize( wxSize( -1, 120 ) );
    Bind( wxEVT_PAINT, &XS_VIEW_PANEL::onPaint, this );
}


void XS_VIEW_PANEL::SetGeometry( const XS_GEOMETRY* aGeometry, double aZ0, double aZdiff,
                                  double aDistMm, bool aIsDiffPair )
{
    m_geometry = aGeometry;
    m_z0 = aZ0;
    m_zdiff = aZdiff;
    m_distMm = aDistMm;
    m_isDiffPair = aIsDiffPair;
    Refresh();
}


void XS_VIEW_PANEL::onPaint( wxPaintEvent& aEvent )
{
    wxAutoBufferedPaintDC dc( this );
    wxRect rect = GetClientRect();

    dc.SetBackground( wxBrush( wxColour( 40, 40, 45 ) ) );
    dc.Clear();

    if( m_geometry && !m_geometry->conductors.empty() )
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
    const XS_GEOMETRY& geom = *m_geometry;
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

    // --- X extent ---
    double xHalf = m_xExtent;

    if( xHalf < 1e-9 )
    {
        for( const XS_CONDUCTOR& c : geom.conductors )
            xHalf = std::max( xHalf, std::abs( c.centerX ) + c.width );

        xHalf *= 1.5;
    }

    double xMin = -xHalf;
    double xMax = xHalf;

    // --- Mapping ---
    int margin = 8;
    int topMargin = 20;
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
            aDC.SetBrush( wxBrush( wxColour( 180, 160, 50 ) ) );
            aDC.SetPen( wxPen( wxColour( 200, 180, 60 ), 1 ) );
        }
        else if( ci == 0 )
        {
            // Signal P — orange
            aDC.SetBrush( wxBrush( wxColour( 220, 140, 40 ) ) );
            aDC.SetPen( wxPen( wxColour( 255, 180, 60 ), 2 ) );
        }
        else if( ci == 1 && m_isDiffPair )
        {
            // Signal N — teal
            aDC.SetBrush( wxBrush( wxColour( 40, 160, 200 ) ) );
            aDC.SetPen( wxPen( wxColour( 60, 200, 240 ), 2 ) );
        }
        else
        {
            aDC.SetBrush( wxBrush( wxColour( 160, 120, 60 ) ) );
            aDC.SetPen( wxPen( wxColour( 180, 140, 70 ), 1 ) );
        }

        aDC.DrawRectangle( px, py, pw, ph );

        if( !c.isGround )
        {
            wxString wLabel = wxString::Format( wxS( "w=%.0f\u00B5m" ), c.width * 1e6 );
            aDC.SetTextForeground( ci == 0 ? wxColour( 255, 200, 100 )
                                           : wxColour( 180, 160, 120 ) );
            wxSize ws = aDC.GetTextExtent( wLabel );
            aDC.DrawText( wLabel, px + pw / 2 - ws.GetWidth() / 2, py + ph + 2 );
        }

        // Edge-to-edge distance dimension line
        if( ci > 0 && !c.isGround )
        {
            double sigRight = sig.centerX + sig.width / 2.0;
            double sigLeft = sig.centerX - sig.width / 2.0;
            double nbRight = c.centerX + c.width / 2.0;
            double nbLeft = c.centerX - c.width / 2.0;

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

            int dimY = py + ph / 2;
            int dimX1 = toPixX( edgeA );
            int dimX2 = toPixX( edgeB );

            aDC.SetPen( wxPen( wxColour( 140, 200, 140 ), 1, wxPENSTYLE_SHORT_DASH ) );
            aDC.DrawLine( dimX1, dimY, dimX2, dimY );
            aDC.DrawLine( dimX1, dimY - 4, dimX1, dimY + 4 );
            aDC.DrawLine( dimX2, dimY - 4, dimX2, dimY + 4 );

            wxString dLabel = wxString::Format( wxS( "%.0f\u00B5m" ), edgeDist * 1e6 );
            aDC.SetTextForeground( wxColour( 140, 200, 140 ) );
            wxSize ds = aDC.GetTextExtent( dLabel );
            aDC.DrawText( dLabel, ( dimX1 + dimX2 - ds.GetWidth() ) / 2, dimY - ds.GetHeight() - 2 );
        }
    }

    // --- Info text ---
    wxString info = wxString::Format( wxS( "Z\u2080=%.1f\u03A9" ), m_z0 );

    if( m_isDiffPair && m_zdiff > 0.0 )
        info += wxString::Format( wxS( "  Zdiff=%.1f\u03A9" ), m_zdiff );

    info += wxString::Format( wxS( " @ %.2fmm" ), m_distMm );

    aDC.SetTextForeground( wxColour( 220, 220, 220 ) );
    aDC.DrawText( info, aRect.x + 6, aRect.y + 4 );
}


// ============================================================================
// IMPEDANCE_PROFILER_PANEL
// ============================================================================

IMPEDANCE_PROFILER_PANEL::IMPEDANCE_PROFILER_PANEL( PCB_EDIT_FRAME* aParent ) :
        WX_PANEL( aParent ),
        m_frame( aParent ),
        m_initialized( false ),
        m_plotWindow( nullptr ),
        m_impedanceTrace( nullptr ),
        m_zdiffTrace( nullptr ),
        m_targetLine( nullptr ),
        m_crosshair( nullptr ),
        m_xAxis( nullptr ),
        m_yAxis( nullptr ),
        m_xsView( nullptr ),
        m_xsDiagText( nullptr ),
        m_statusText( nullptr ),
        m_selectedSample( -1 ),
        m_currentNetCode( -1 ),
        m_isDiffPairMode( false ),
        m_coupledNetCode( 0 ),
        m_xAxisIsTime( false ),
        m_showCrossSection( false )
{
    PCBNEW_SETTINGS* cfg = aParent->GetPcbNewSettings();

    if( cfg )
    {
        m_xAxisIsTime = cfg->m_ImpedanceProfiler.x_axis_is_time;
        m_showCrossSection = cfg->m_ImpedanceProfiler.show_cross_section;
    }
}


IMPEDANCE_PROFILER_PANEL::~IMPEDANCE_PROFILER_PANEL()
{
    if( PCBNEW_SETTINGS* cfg = m_frame->GetPcbNewSettings() )
    {
        cfg->m_ImpedanceProfiler.x_axis_is_time = m_xAxisIsTime;
        cfg->m_ImpedanceProfiler.show_cross_section = m_showCrossSection;
    }
}


void IMPEDANCE_PROFILER_PANEL::buildUI()
{
    wxBoxSizer* mainSizer = new wxBoxSizer( wxVERTICAL );

    m_plotWindow = new mpWindow( this, wxID_ANY );
    m_plotWindow->SetMinSize( wxSize( 200, 150 ) );
    m_plotWindow->EnableDoubleBuffer( true );

    wxPen tracePen( wxColour( 0, 120, 200 ), 2, wxPENSTYLE_SOLID );

    m_xAxis = new PROFILER_SCALE_X();
    m_yAxis = new PROFILER_SCALE_Y();

    if( m_xAxisIsTime )
        static_cast<PROFILER_SCALE_X*>( m_xAxis )->SetUnitLabel( wxS( "ps" ) );

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

    wxPen zdiffPen( wxColour( 40, 200, 120 ), 2, wxPENSTYLE_SOLID );

    m_zdiffTrace = new mpFXYVector( _( "Zdiff" ) );
    m_zdiffTrace->SetPen( zdiffPen );
    m_zdiffTrace->SetContinuity( true );
    m_zdiffTrace->SetScale( m_xAxis, m_yAxis );
    m_zdiffTrace->SetVisible( false );
    m_plotWindow->AddLayer( m_zdiffTrace, false );

    m_crosshair = new CROSSHAIR_LAYER();
    m_plotWindow->AddLayer( m_crosshair, false );

    m_plotWindow->SetMargins( 15, 10, 30, 50 );
    m_plotWindow->UpdateAll();

    mainSizer->Add( m_plotWindow, 3, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 4 );

    m_xsView = new XS_VIEW_PANEL( this );
    mainSizer->Add( m_xsView, 1, wxEXPAND | wxLEFT | wxRIGHT, 4 );

    m_xsDiagText = new wxStaticText( this, wxID_ANY, wxEmptyString );
    mainSizer->Add( m_xsDiagText, 0, wxEXPAND | wxLEFT | wxRIGHT, 4 );

    m_xsView->Show( m_showCrossSection );
    m_xsDiagText->Show( m_showCrossSection );

    m_statusText = new wxStaticText( this, wxID_ANY,
                                     _( "Right-click a trace and choose Analyze Impedance." ) );
    mainSizer->Add( m_statusText, 0, wxEXPAND | wxALL, 4 );

    SetSizer( mainSizer );

    wxMenu* popMenu = m_plotWindow->GetPopupMenu();
    popMenu->AppendSeparator();
    popMenu->AppendRadioItem( ID_XAXIS_MM, _( "X-axis: mm" ) );
    popMenu->AppendRadioItem( ID_XAXIS_PS, _( "X-axis: ps" ) );
    popMenu->Check( m_xAxisIsTime ? ID_XAXIS_PS : ID_XAXIS_MM, true );
    popMenu->AppendSeparator();
    popMenu->AppendCheckItem( ID_SHOW_XS, _( "Show cross-section" ) );
    popMenu->Check( ID_SHOW_XS, m_showCrossSection );

    m_plotWindow->Bind( wxEVT_MOTION, &IMPEDANCE_PROFILER_PANEL::onPlotMotion, this );
    m_plotWindow->Bind( wxEVT_MENU, &IMPEDANCE_PROFILER_PANEL::onContextMenuCommand, this,
                        ID_XAXIS_MM, ID_SHOW_XS );
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
}


void IMPEDANCE_PROFILER_PANEL::AnalyseNet( int aNetCode )
{
    if( !m_initialized )
    {
        buildUI();

        if( BOARD* board = m_frame->GetBoard() )
            board->AddListener( this );

        m_initialized = true;
    }

    // Look up net name and detect diff pair
    if( BOARD* board = m_frame->GetBoard() )
    {
        NETINFO_ITEM* net = board->FindNet( aNetCode );

        if( net )
        {
            m_currentNetName = net->GetNetname();

            NETINFO_ITEM* coupledNet = board->DpCoupledNet( net );
            m_coupledNetCode = coupledNet ? coupledNet->GetNetCode() : 0;
            m_isDiffPairMode = ( m_coupledNetCode > 0 );
            m_coupledNetName = coupledNet ? coupledNet->GetNetname() : wxString();
        }
    }

    runAnalysis( aNetCode );
}


void IMPEDANCE_PROFILER_PANEL::onPlotMotion( wxMouseEvent& aEvent )
{
    bool hasSamples = m_isDiffPairMode
                      ? ( m_diffProfile && !m_diffProfile->GetSamples().empty() )
                      : ( m_seProfile && !m_seProfile->GetSamples().empty() );

    if( !hasSamples )
    {
        aEvent.Skip();
        return;
    }

    double normX = m_plotWindow->p2x( aEvent.GetX() );
    double plotX = m_xAxis->TransformFromPlot( normX );

    // Find nearest sample
    int bestIdx = 0;
    double bestDist = 1e9;

    if( m_isDiffPairMode && m_diffProfile )
    {
        const auto& samples = m_diffProfile->GetSamples();

        for( int i = 0; i < (int) samples.size(); i++ )
        {
            double sampleX = m_xAxisIsTime ? samples[i].delayPs
                                           : ( m_diffProfile->GetProfileP().GetSamples().empty()
                                               ? 0.0
                                               : m_diffProfile->GetProfileP().AtDelay( samples[i].delayPs ).distNm / 1e6 );
            double d = std::abs( sampleX - plotX );

            if( d < bestDist )
            {
                bestDist = d;
                bestIdx = i;
            }
        }
    }
    else if( m_seProfile )
    {
        const auto& samples = m_seProfile->GetSamples();

        for( int i = 0; i < (int) samples.size(); i++ )
        {
            double sampleX = m_xAxisIsTime ? samples[i].delayPs : samples[i].distNm / 1e6;
            double d = std::abs( sampleX - plotX );

            if( d < bestDist )
            {
                bestDist = d;
                bestIdx = i;
            }
        }
    }

    selectSample( bestIdx );
    aEvent.Skip();
}


void IMPEDANCE_PROFILER_PANEL::onContextMenuCommand( wxCommandEvent& aEvent )
{
    switch( aEvent.GetId() )
    {
    case ID_XAXIS_MM:
        m_xAxisIsTime = false;
        static_cast<PROFILER_SCALE_X*>( m_xAxis )->SetUnitLabel( wxS( "mm" ) );

        if( m_seProfile || m_diffProfile )
            updatePlot();

        break;

    case ID_XAXIS_PS:
        m_xAxisIsTime = true;
        static_cast<PROFILER_SCALE_X*>( m_xAxis )->SetUnitLabel( wxS( "ps" ) );

        if( m_seProfile || m_diffProfile )
            updatePlot();

        break;

    case ID_SHOW_XS:
        m_showCrossSection = !m_showCrossSection;
        m_xsView->Show( m_showCrossSection );
        m_xsDiagText->Show( m_showCrossSection );
        GetSizer()->Layout();
        break;
    }

    if( PCBNEW_SETTINGS* cfg = m_frame->GetPcbNewSettings() )
    {
        cfg->m_ImpedanceProfiler.x_axis_is_time = m_xAxisIsTime;
        cfg->m_ImpedanceProfiler.show_cross_section = m_showCrossSection;
    }
}


void IMPEDANCE_PROFILER_PANEL::selectSample( int aIndex )
{
    if( m_isDiffPairMode && m_diffProfile )
    {
        const auto& samples = m_diffProfile->GetSamples();

        if( aIndex < 0 || aIndex >= (int) samples.size() )
            return;

        m_selectedSample = aIndex;
        const DIFF_SAMPLE& ds = samples[aIndex];

        double distMm = ds.distMm;

        if( m_showCrossSection )
        {
            m_xsView->SetGeometry( &ds.geometry, ds.z0P, ds.zdiff, distMm, ds.coupled );

            wxString diag;

            if( ds.coupled )
            {
                diag += wxString::Format( wxS( "gap=%.0f\u00B5m" ), ds.gapUm );
                diag += wxString::Format( wxS( "  \u03B5rOdd=%.2f" ), ds.erEffOdd );
            }
            else
            {
                diag += wxS( "uncoupled" );
            }

            m_xsDiagText->SetLabel( diag );
        }

        double dataX = m_xAxisIsTime ? ds.delayPs : distMm;
        m_crosshair->SetPosition( dataX, ds.zdiff, m_xAxis, m_yAxis );
        m_plotWindow->Refresh();

        wxString status = wxString::Format(
                wxS( "@ %.1fmm (%.0fps)  Z\u2080_P=%.1f\u03A9  Z\u2080_N=%.1f\u03A9  Zdiff=%.1f\u03A9" ),
                distMm, ds.delayPs, ds.z0P, ds.z0N, ds.zdiff );

        if( std::abs( m_diffProfile->GetSkewPs() ) > 0.01 )
            status += wxString::Format( wxS( "  skew=%.1fps" ), m_diffProfile->GetSkewPs() );

        updateStatus( status );

        m_frame->FocusOnLocation( ds.boardPosP );
        m_frame->GetCanvas()->Refresh();
    }
    else if( m_seProfile )
    {
        const auto& samples = m_seProfile->GetSamples();

        if( aIndex < 0 || aIndex >= (int) samples.size() )
            return;

        m_selectedSample = aIndex;
        const IMPEDANCE_SAMPLE& s = samples[aIndex];

        double distMm = s.distNm / 1e6;

        if( m_showCrossSection )
        {
            m_xsView->SetGeometry( &s.geometry, s.z0, 0.0, distMm, false );

            wxString diag;

            if( s.neighborCount > 0 )
                diag += wxString::Format( wxS( "%dnb" ), s.neighborCount );

            if( s.groundwireCount > 0 )
            {
                if( !diag.empty() )
                    diag += wxS( "  " );

                diag += wxString::Format( wxS( "%dgw" ), s.groundwireCount );
            }

            if( s.hasRefBelow )
            {
                if( !diag.empty() )
                    diag += wxS( "  " );

                diag += wxString::Format( wxS( "h\u2193%.0f\u00B5m" ), s.hBelow * 1e6 );
            }

            if( s.hasRefAbove )
            {
                if( !diag.empty() )
                    diag += wxS( "  " );

                diag += wxString::Format( wxS( "h\u2191%.0f\u00B5m" ), s.hAbove * 1e6 );
            }

            if( s.erEff > 1.0 )
            {
                if( !diag.empty() )
                    diag += wxS( "  " );

                diag += wxString::Format( wxS( "\u03B5rEff=%.2f" ), s.erEff );
            }

            m_xsDiagText->SetLabel( diag );
        }

        double dataX = m_xAxisIsTime ? s.delayPs : distMm;
        m_crosshair->SetPosition( dataX, s.z0, m_xAxis, m_yAxis );
        m_plotWindow->Refresh();

        wxString status = wxString::Format( wxS( "@ %.1fmm (%.0fps)  Z\u2080=%.1f\u03A9" ),
                                            distMm, s.delayPs, s.z0 );
        updateStatus( status );

        m_frame->FocusOnLocation( s.boardPos );
        m_frame->GetCanvas()->Refresh();
    }
}


void IMPEDANCE_PROFILER_PANEL::runAnalysis( int aNetCode )
{
    m_currentNetCode = aNetCode;
    m_selectedSample = -1;

    // Clear stale geometry pointer before destroying profiles
    m_xsView->SetGeometry( nullptr, 0.0, 0.0, 0.0, false );

    m_seProfile.reset();
    m_diffProfile.reset();

    BOARD* board = m_frame->GetBoard();

    if( !board )
        return;

    if( m_isDiffPairMode )
    {
        m_diffProfile = std::make_unique<DIFF_PROFILE>();

        if( !m_diffProfile->Compute( board, aNetCode, m_coupledNetCode, m_bemCache ) )
        {
            updateStatus( m_diffProfile->GetError() );
            return;
        }
    }
    else
    {
        m_seProfile = std::make_unique<SE_PROFILE>();

        if( !m_seProfile->Compute( board, aNetCode, m_bemCache ) )
        {
            updateStatus( m_seProfile->GetError() );
            return;
        }
    }

    // Compute max X extent for the XS view
    double maxXExtent = 0.0;

    if( m_isDiffPairMode && m_diffProfile )
    {
        for( const DIFF_SAMPLE& ds : m_diffProfile->GetSamples() )
        {
            for( const XS_CONDUCTOR& c : ds.geometry.conductors )
                maxXExtent = std::max( maxXExtent, std::abs( c.centerX ) + c.width );
        }
    }
    else if( m_seProfile )
    {
        for( const IMPEDANCE_SAMPLE& s : m_seProfile->GetSamples() )
        {
            for( const XS_CONDUCTOR& c : s.geometry.conductors )
                maxXExtent = std::max( maxXExtent, std::abs( c.centerX ) + c.width );
        }
    }

    m_xsView->SetXExtent( maxXExtent * 1.5 );

    updatePlot();

    // Select middle sample
    if( m_isDiffPairMode && m_diffProfile )
        selectSample( (int) m_diffProfile->GetSamples().size() / 2 );
    else if( m_seProfile )
        selectSample( (int) m_seProfile->GetSamples().size() / 2 );
}


void IMPEDANCE_PROFILER_PANEL::updatePlot()
{
    std::vector<double> positions;
    std::vector<double> impedances;
    std::vector<double> zdiffPositions;
    std::vector<double> zdiffValues;

    if( m_isDiffPairMode && m_diffProfile )
    {
        const auto& dSamples = m_diffProfile->GetSamples();

        for( size_t i = 0; i < dSamples.size(); i++ )
        {
            const DIFF_SAMPLE& ds = dSamples[i];

            double xVal = m_xAxisIsTime ? ds.delayPs : ds.distMm;

            positions.push_back( xVal );
            impedances.push_back( ds.z0P );

            zdiffPositions.push_back( xVal );
            zdiffValues.push_back( ds.zdiff );
        }
    }
    else if( m_seProfile )
    {
        for( const IMPEDANCE_SAMPLE& s : m_seProfile->GetSamples() )
        {
            positions.push_back( m_xAxisIsTime ? s.delayPs : s.distNm / 1e6 );
            impedances.push_back( s.z0 );
        }
    }

    if( positions.empty() )
        return;

    m_impedanceTrace->SetData( positions, impedances );
    m_impedanceTrace->SetVisible( true );
    m_targetLine->SetVisible( false );

    double yMin = *std::min_element( impedances.begin(), impedances.end() );
    double yMax = *std::max_element( impedances.begin(), impedances.end() );

    if( !zdiffValues.empty() )
    {
        m_zdiffTrace->SetData( zdiffPositions, zdiffValues );
        m_zdiffTrace->SetVisible( true );

        double zdMin = *std::min_element( zdiffValues.begin(), zdiffValues.end() );
        double zdMax = *std::max_element( zdiffValues.begin(), zdiffValues.end() );
        yMin = std::min( yMin, zdMin );
        yMax = std::max( yMax, zdMax );
    }
    else
    {
        m_zdiffTrace->SetVisible( false );
    }

    if( yMax - yMin < 0.5 )
    {
        double mid = ( yMax + yMin ) / 2.0;
        yMin = mid - 2.0;
        yMax = mid + 2.0;
    }

    double yPad = ( yMax - yMin ) * 0.15;

    m_xAxis->ResetDataRange();
    m_yAxis->ResetDataRange();
    m_impedanceTrace->UpdateScales();

    if( !zdiffValues.empty() )
        m_zdiffTrace->UpdateScales();

    m_yAxis->ExtendDataRange( yMin - yPad, yMax + yPad );
    m_plotWindow->Fit();
    m_plotWindow->Refresh();
}


void IMPEDANCE_PROFILER_PANEL::updateStatus( const wxString& aText )
{
    m_statusText->SetLabel( aText );
}


// BOARD_LISTENER
void IMPEDANCE_PROFILER_PANEL::OnBoardItemAdded( BOARD& aBoard, BOARD_ITEM* aBoardItem ) {}
void IMPEDANCE_PROFILER_PANEL::OnBoardItemsAdded( BOARD& aBoard, std::vector<BOARD_ITEM*>& ) {}
void IMPEDANCE_PROFILER_PANEL::OnBoardItemRemoved( BOARD& aBoard, BOARD_ITEM* aBoardItem ) {}
void IMPEDANCE_PROFILER_PANEL::OnBoardItemsRemoved( BOARD& aBoard, std::vector<BOARD_ITEM*>& ) {}

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

void IMPEDANCE_PROFILER_PANEL::OnBoardHighlightNetChanged( BOARD& aBoard ) {}
