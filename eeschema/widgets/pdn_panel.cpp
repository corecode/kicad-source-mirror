/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2024-2026 KiCad Developers, see AUTHORS.txt for contributors.
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

#include "pdn_panel.h"

#include <sch_commit.h>
#include <sch_connection.h>
#include <sch_edit_frame.h>
#include <sch_pin.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <tool/tool_manager.h>
#include <tools/sch_actions.h>
#include <tools/sch_selection_tool.h>
#include <bitmaps/bitmap_types.h>
#include <gal/color4d.h>
#include <widgets/mathplot.h>

#include <wx/menu.h>
#include <wx/msgdlg.h>
#include <wx/statbmp.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/textdlg.h>


static constexpr int LOG_Z_MIN = -3;
static constexpr int LOG_Z_MAX = 2;
static constexpr int MINOR_REDUCE_THRESHOLD = 80;


/**
 * Format a log10 value as an SI-prefixed label (e.g. log10(1e6) -> "1MHz").
 */
static wxString formatSILabel( double aLogValue, const wxString& aUnit )
{
    // idx 0 -> exp -15 (femto) ... idx 5 -> exp 0 (none) ... idx 9 -> exp 12 (tera)
    static const char prefixes[] = "fpnum kMGT";
    static const int  maxIdx = static_cast<int>( sizeof( prefixes ) ) - 2;
    int               exp = static_cast<int>( lround( aLogValue ) );
    int               idx = std::clamp( ( exp + 15 ) / 3, 0, maxIdx );
    double            mantissa = pow( 10.0, exp - ( idx * 3 - 15 ) );
    char              prefix = prefixes[idx];
    wxString          pfx = ( prefix == ' ' ) ? wxString() : wxString( prefix );

    return wxString::Format( wxS( "%g%s%s" ), mantissa, pfx, aUnit );
}


/**
 * Generate log-decade tick values and labels for a visible range in log10 space.
 *
 * Each decade gets 9 sub-ticks (1-9); decade boundaries (mult==1) become labels.
 * Boundaries are always on integer decades, so tick index i%9==0 reliably
 * identifies major (decade) ticks.
 */
static void calcLogDecadeTicks( std::vector<double>&                  aTickValues,
                                std::vector<mpScaleBase::TICK_LABEL>& aTickLabels, double aMinVis,
                                double aMaxVis, int aClampMin = INT_MIN, int aClampMax = INT_MAX )
{
    aTickValues.clear();
    aTickLabels.clear();

    int minDecade = std::max( static_cast<int>( floor( aMinVis ) ), aClampMin );
    int maxDecade = std::min( static_cast<int>( ceil( aMaxVis ) ), aClampMax );

    for( int decade = minDecade; decade <= maxDecade; decade++ )
    {
        for( int mult = 1; mult <= 9; mult++ )
        {
            double logVal = decade + log10( static_cast<double>( mult ) );

            if( logVal < aClampMin || logVal > aClampMax )
                continue;

            aTickValues.push_back( logVal );

            if( mult == 1 )
                aTickLabels.emplace_back( logVal );
        }
    }
}


/**
 * Log10-space axis scale for PDN plots.
 *
 * Both X (frequency) and Y (impedance) axes store pre-transformed log10 data
 * and use a linear base class for coordinate mapping.  This template provides
 * the shared tick calculation, label formatting, and accessor logic.
 */
template <typename BASE>
class PDN_LOG_SCALE : public BASE
{
public:
    PDN_LOG_SCALE( int aAlign, const wxString aUnit, int aClampMin = INT_MIN,
                   int aClampMax = INT_MAX ) :
            BASE( wxEmptyString, aAlign, false ), m_unit( aUnit ), m_clampMin( aClampMin ),
            m_clampMax( aClampMax )
    {
    }

    const std::vector<double>& GetTickValues() const { return this->m_tickValues; }

    const std::vector<typename BASE::TICK_LABEL>& GetTickLabels() const
    {
        return this->m_tickLabels;
    }

    void Plot( wxDC& dc, mpWindow& w ) override
    {
        this->m_offset = -this->m_minV;
        this->m_scale = 1.0 / ( this->m_maxV - this->m_minV );
        this->recalculateTicks( dc, w );
    }

protected:
    void recalculateTicks( wxDC& dc, mpWindow& w ) override
    {
        double minVvis, maxVvis;
        this->getVisibleDataRange( w, minVvis, maxVvis );
        this->m_absVisibleMaxV = std::max( std::abs( minVvis ), std::abs( maxVvis ) );
        calcLogDecadeTicks( this->m_tickValues, this->m_tickLabels, minVvis, maxVvis, m_clampMin,
                            m_clampMax );
        this->updateTickLabels( dc, w );
    }

    void formatLabels() override
    {
        for( auto& l : this->m_tickLabels )
        {
            l.label = formatSILabel( l.pos, m_unit );
            l.visible = true;
        }
    }

    wxString m_unit;
    int      m_clampMin;
    int      m_clampMax;
};


class PDN_XSCALE : public PDN_LOG_SCALE<mpScaleX>
{
public:
    PDN_XSCALE() : PDN_LOG_SCALE( mpALIGN_BORDER_BOTTOM, wxS( "Hz" ) ) {}
};


class PDN_YSCALE : public PDN_LOG_SCALE<mpScaleY>
{
public:
    PDN_YSCALE() : PDN_LOG_SCALE( mpALIGN_BORDER_LEFT, wxS( "\u03A9" ), LOG_Z_MIN, LOG_Z_MAX ) {}
};


/**
 * Overlay layer that draws the grid, axis labels, and network title.
 *
 * Draws minor grid lines first, then major, ensuring major lines are on top.
 */
class PDN_LABEL_OVERLAY : public mpLayer
{
public:
    PDN_LABEL_OVERLAY( PDN_XSCALE* aXAxis, PDN_YSCALE* aYAxis ) :
            m_xAxis( aXAxis ), m_yAxis( aYAxis ), m_gridMajor( 140, 140, 140 ),
            m_gridMinor( 210, 210, 210 )
    {
        m_type = mpLAYER_INFO;
    }

    bool HasBBox() const override { return false; }

    void SetNetworkLabel( const wxString& aLabel ) { m_networkLabel = aLabel; }
    void SetSheetLabel( const wxString& aLabel ) { m_sheetLabel = aLabel; }
    void SetReferenceLabel( const wxString& aLabel ) { m_refLabel = aLabel; }
    void SetReferenceLabelColor( const wxColour& aColor ) { m_refLabelColor = aColor; }

    void SetTransferLabel( const wxString& aLabel ) { m_transferLabel = aLabel; }
    void SetTransferLabelColor( const wxColour& aColor ) { m_transferLabelColor = aColor; }
    void SetTransferClampIndicators( bool aLeft, double aLeftX, bool aRight, double aRightX )
    {
        m_transferClampedLeft = aLeft;
        m_transferLeftX = aLeftX;
        m_transferClampedRight = aRight;
        m_transferRightX = aRightX;
    }

    void SetGridColors( const wxColour& aMajor, const wxColour& aMinor )
    {
        m_gridMajor = aMajor;
        m_gridMinor = aMinor;
    }

    void Plot( wxDC& dc, mpWindow& w ) override
    {
        drawGrid( dc, w, m_xAxis, true, false );
        drawGrid( dc, w, m_yAxis, false, false );
        drawGrid( dc, w, m_xAxis, true, true );
        drawGrid( dc, w, m_yAxis, false, true );

        drawLabels( dc, w, m_xAxis, true );
        drawLabels( dc, w, m_yAxis, false );

        if( !m_networkLabel.IsEmpty() )
        {
            dc.SetFont( m_font );
            dc.SetTextForeground( wxSystemSettings::GetColour( wxSYS_COLOUR_WINDOWTEXT ) );

            wxCoord tx, ty;
            dc.GetTextExtent( m_networkLabel, &tx, &ty );

            wxCoord plotWidth = w.GetScrX() - w.GetMarginLeft() - w.GetMarginRight();
            wxCoord xPos = w.GetMarginLeft() + ( plotWidth - tx ) / 2;
            wxCoord yPos = 4 + ty + 2;
            dc.DrawText( m_networkLabel, xPos, yPos );

            if( !m_sheetLabel.IsEmpty() )
            {
                wxCoord sx, sy;
                dc.GetTextExtent( m_sheetLabel, &sx, &sy );
                wxCoord sxPos = w.GetMarginLeft() + ( plotWidth - sx ) / 2;
                dc.DrawText( m_sheetLabel, sxPos, yPos + ty + 1 );
                yPos += ty + 1;
            }

            if( !m_refLabel.IsEmpty() )
            {
                dc.SetTextForeground( m_refLabelColor );

                wxCoord rx, ry;
                dc.GetTextExtent( m_refLabel, &rx, &ry );
                wxCoord rxPos = w.GetMarginLeft() + ( plotWidth - rx ) / 2;
                dc.DrawText( m_refLabel, rxPos, yPos + ty + 1 );
                yPos += ty + 1;
            }

            if( !m_transferLabel.IsEmpty() )
            {
                dc.SetTextForeground( m_transferLabelColor );

                wxCoord tlx, tly;
                dc.GetTextExtent( m_transferLabel, &tlx, &tly );
                wxCoord tlxPos = w.GetMarginLeft() + ( plotWidth - tlx ) / 2;
                dc.DrawText( m_transferLabel, tlxPos, yPos + ty + 1 );
                yPos += ty + 1;
            }
        }

        // Draw clamp indicators (small filled triangles) at plot bottom
        if( m_transferClampedLeft || m_transferClampedRight )
        {
            dc.SetBrush( wxBrush( m_transferLabelColor ) );
            dc.SetPen( wxPen( m_transferLabelColor, 1 ) );

            wxCoord plotBottom = w.GetScrY() - w.GetMarginBottom();
            int     triSize = 5;

            if( m_transferClampedLeft )
            {
                double plotPos = m_xAxis->TransformToPlot( m_transferLeftX );
                int    px = static_cast<int>( ( plotPos - w.GetPosX() ) * w.GetScaleX() );

                if( px >= w.GetMarginLeft() && px <= w.GetScrX() - w.GetMarginRight() )
                {
                    wxPoint tri[3] = { wxPoint( px - triSize, plotBottom - triSize * 2 ),
                                       wxPoint( px + triSize, plotBottom - triSize * 2 ),
                                       wxPoint( px, plotBottom ) };
                    dc.DrawPolygon( 3, tri );
                }
            }

            if( m_transferClampedRight )
            {
                double plotPos = m_xAxis->TransformToPlot( m_transferRightX );
                int    px = static_cast<int>( ( plotPos - w.GetPosX() ) * w.GetScaleX() );

                if( px >= w.GetMarginLeft() && px <= w.GetScrX() - w.GetMarginRight() )
                {
                    wxPoint tri[3] = { wxPoint( px - triSize, plotBottom - triSize * 2 ),
                                       wxPoint( px + triSize, plotBottom - triSize * 2 ),
                                       wxPoint( px, plotBottom ) };
                    dc.DrawPolygon( 3, tri );
                }
            }
        }
    }

private:
    void drawGrid( wxDC& dc, mpWindow& w, mpScaleBase* aScale, bool aHorizontal, bool aMajorOnly )
    {
        const std::vector<double>& ticks =
                aHorizontal ? m_xAxis->GetTickValues() : m_yAxis->GetTickValues();

        wxCoord startPx, endPx, crossMin, crossMax;
        if( aHorizontal )
        {
            startPx = w.GetMarginLeft();
            endPx = w.GetScrX() - w.GetMarginRight();
            crossMin = w.GetMarginTop();
            crossMax = w.GetScrY() - w.GetMarginBottom();
        }
        else
        {
            startPx = w.GetMarginTop();
            endPx = w.GetScrY() - w.GetMarginBottom();
            crossMin = w.GetMarginLeft();
            crossMax = w.GetScrX() - w.GetMarginRight();
        }

        dc.SetPen( wxPen( aMajorOnly ? m_gridMajor : m_gridMinor, 1 ) );

        bool halfMinors =
                ( ( endPx - startPx ) / ( ticks.size() / 9 + 1 ) ) < MINOR_REDUCE_THRESHOLD;
        for( size_t i = 0; i < ticks.size(); i++ )
        {
            size_t minor = i % 9;
            if( ( aMajorOnly && minor != 0 ) || ( halfMinors && ( minor % 2 == 1 ) ) )
                continue;

            double plotPos = aScale->TransformToPlot( ticks[i] );
            if( aHorizontal )
            {
                int p = ( plotPos - w.GetPosX() ) * w.GetScaleX();
                dc.DrawLine( p, crossMin, p, crossMax );
            }
            else
            {
                int p = ( w.GetPosY() - plotPos ) * w.GetScaleY();
                dc.DrawLine( crossMin, p, crossMax, p );
            }
        }
    }

    void drawLabels( wxDC& dc, mpWindow& w, mpScaleBase* aScale, bool aHorizontal )
    {
        const auto& labels = aHorizontal ? m_xAxis->GetTickLabels() : m_yAxis->GetTickLabels();

        dc.SetFont( m_font );

        for( const mpScaleBase::TICK_LABEL& tl : labels )
        {
            if( !tl.visible )
                continue;

            wxCoord tx, ty;
            dc.GetTextExtent( tl.label, &tx, &ty );

            double plotPos = aScale->TransformToPlot( tl.pos );
            if( aHorizontal )
            {
                int p = ( plotPos - w.GetPosX() ) * w.GetScaleX();
                int left = p - tx / 2;
                int right = left + tx;

                // Skip labels that would extend beyond the plot area
                if( left < w.GetMarginLeft() || right > w.GetScrX() - w.GetMarginRight() )
                    continue;

                dc.DrawText( tl.label, left, 4 );
            }
            else
            {
                int p = ( w.GetPosY() - plotPos ) * w.GetScaleY();
                int top = p - ty / 2;
                int bottom = top + ty;

                if( top < w.GetMarginTop() || bottom > w.GetScrY() - w.GetMarginBottom() )
                    continue;

                dc.DrawText( tl.label, 5, top );
            }
        }
    }

    PDN_XSCALE* m_xAxis;
    PDN_YSCALE* m_yAxis;
    wxString    m_networkLabel;
    wxString    m_sheetLabel;
    wxString    m_refLabel;
    wxColour    m_refLabelColor;
    wxString    m_transferLabel;
    wxColour    m_transferLabelColor;
    bool        m_transferClampedLeft = false;
    double      m_transferLeftX = 0.0;
    bool        m_transferClampedRight = false;
    double      m_transferRightX = 0.0;
    wxColour    m_gridMajor;
    wxColour    m_gridMinor;
};


PDN_PANEL::PDN_PANEL( SCH_EDIT_FRAME* aParent ) :
        WX_PANEL( aParent ), m_frame( aParent ), m_updatePending( false ),
        m_impedanceTrace( nullptr ), m_referenceTrace( nullptr ), m_transferTrace( nullptr ),
        m_xAxis( nullptr ), m_yAxis( nullptr ), m_labelOverlay( nullptr ), m_warningIcon( nullptr )
{
    wxBoxSizer* mainSizer = new wxBoxSizer( wxVERTICAL );

    m_plotWindow = new mpWindow( this, wxID_ANY );
    m_plotWindow->SetMargins( 0, 0, 0, 0 );
    m_plotWindow->EnableDoubleBuffer( true );
    m_plotWindow->EnableMousePanZoom( false );

    wxMenu* popMenu = m_plotWindow->GetPopupMenu();

    while( popMenu->GetMenuItemCount() > 0 )
        popMenu->Delete( popMenu->FindItemByPosition( 0 ) );

    m_xAxis = new PDN_XSCALE();
    m_yAxis = new PDN_YSCALE();

    m_plotWindow->AddLayer( m_xAxis, false );
    m_plotWindow->AddLayer( m_yAxis, false );

    m_labelOverlay = new PDN_LABEL_OVERLAY( m_xAxis, m_yAxis );
    m_plotWindow->AddLayer( m_labelOverlay, false );

    std::vector<double> seedX = { 2.9, 9.1 };
    std::vector<double> seedY = { -3.0, 2.0 };

    m_referenceTrace = new mpFXYVector( _( "Reference" ) );
    m_referenceTrace->SetContinuity( true );
    m_referenceTrace->SetData( seedX, seedY );
    m_referenceTrace->SetVisible( false );
    m_referenceTrace->SetScale( m_xAxis, m_yAxis );
    m_plotWindow->AddLayer( m_referenceTrace, false );

    m_showTransfer = true;

    m_transferTrace = new mpFXYVector( wxS( "Z21" ) );
    m_transferTrace->SetContinuity( true );
    m_transferTrace->SetData( seedX, seedY );
    m_transferTrace->SetVisible( false );
    m_transferTrace->SetScale( m_xAxis, m_yAxis );
    m_plotWindow->AddLayer( m_transferTrace, false );

    m_impedanceTrace = new mpFXYVector( _( "Z(f)" ) );
    m_impedanceTrace->SetContinuity( true );
    m_impedanceTrace->SetData( seedX, seedY );
    m_impedanceTrace->SetVisible( false );

    m_impedanceTrace->SetScale( m_xAxis, m_yAxis );
    m_plotWindow->AddLayer( m_impedanceTrace, false );

    wxBitmapBundle warnBundle = KiBitmapBundle( BITMAPS::dialog_warning );
    wxBitmap       warnBmp = warnBundle.GetBitmapFor( m_plotWindow );
    wxImage        warnImg = warnBmp.ConvertToImage();
    double         scaleFactor = m_plotWindow->GetContentScaleFactor();
    int            iconPx = std::max( 1, static_cast<int>( 16 * scaleFactor ) );
    warnImg.Rescale( iconPx, iconPx, wxIMAGE_QUALITY_HIGH );
    wxBitmap scaledBmp( warnImg );
    scaledBmp.SetScaleFactor( scaleFactor );
    m_warningIcon = new wxStaticBitmap( m_plotWindow, wxID_ANY, scaledBmp );
    m_warningIcon->Hide();

    updateThemeColors();

    mainSizer->Add( m_plotWindow, 1, wxEXPAND );

    SetSizer( mainSizer );

    m_plotWindow->UpdateAll();

    m_plotWindow->Bind( wxEVT_LEFT_DOWN, []( wxMouseEvent& ){} );
    m_plotWindow->Bind( wxEVT_LEFT_UP, []( wxMouseEvent& ){} );
    m_plotWindow->Bind( wxEVT_RIGHT_DOWN, &PDN_PANEL::onPlotRightClick, this );
    m_plotWindow->Bind(
            wxEVT_SIZE,
            [this]( wxSizeEvent& aEvent )
            {
                aEvent.Skip();

                if( m_warningIcon->IsShown() )
                {
                    wxSize iconSize = m_warningIcon->GetSize();
                    wxSize plotSize = m_plotWindow->GetClientSize();
                    m_warningIcon->SetPosition(
                            wxPoint( plotSize.x - iconSize.x - 4, plotSize.y - iconSize.y - 4 ) );
                }
            } );
    m_frame->Schematic().AddListener( this );
    m_frame->Bind( EDA_EVT_SCHEMATIC_CHANGED, &PDN_PANEL::onSchematicChanged, this );
    m_frame->Bind( EDA_EVT_SCH_SELECTION_CHANGED, &PDN_PANEL::OnSchSelectionChanged, this );
    Bind( wxEVT_SYS_COLOUR_CHANGED, &PDN_PANEL::onThemeChanged, this );
}


PDN_PANEL::~PDN_PANEL()
{
    // Note: RemoveListener is not needed here because SCH_EDIT_FRAME::~SCH_EDIT_FRAME
    // calls RemoveAllListeners() before child windows are destroyed.
    m_frame->Unbind( EDA_EVT_SCHEMATIC_CHANGED, &PDN_PANEL::onSchematicChanged, this );
    m_frame->Unbind( EDA_EVT_SCH_SELECTION_CHANGED, &PDN_PANEL::OnSchSelectionChanged, this );
}


void PDN_PANEL::OnSchItemsAdded( SCHEMATIC& aSch, std::vector<SCH_ITEM*>& aItems )
{
    if( IsShownOnScreen() )
        scheduleUpdate();
}


void PDN_PANEL::OnSchItemsRemoved( SCHEMATIC& aSch, std::vector<SCH_ITEM*>& aItems )
{
    if( IsShownOnScreen() )
        scheduleUpdate();
}


void PDN_PANEL::OnSchItemsChanged( SCHEMATIC& aSch, std::vector<SCH_ITEM*>& aItems )
{
    if( IsShownOnScreen() )
        scheduleUpdate();
}


void PDN_PANEL::scheduleUpdate()
{
    if( m_updatePending )
        return;

    m_updatePending = true;

    CallAfter(
            [this]()
            {
                m_updatePending = false;
                UpdateNetworks();
            } );
}


void PDN_PANEL::updateWarningIndicator()
{
    std::vector<wxString> warnings = m_analyzer.GetWarnings( m_selectedNetwork );

    if( warnings.empty() )
    {
        m_warningIcon->Hide();
        return;
    }

    wxString tooltip;

    for( const wxString& w : warnings )
    {
        if( !tooltip.IsEmpty() )
            tooltip += wxS( "\n" );

        tooltip += w;
    }

    m_warningIcon->SetToolTip( tooltip );
    m_warningIcon->Show();

    wxSize iconSize = m_warningIcon->GetSize();
    wxSize plotSize = m_plotWindow->GetClientSize();
    m_warningIcon->SetPosition(
            wxPoint( plotSize.x - iconSize.x - 4, plotSize.y - iconSize.y - 4 ) );
}


void PDN_PANEL::updateThemeColors()
{
    wxColour bg = wxSystemSettings::GetColour( wxSYS_COLOUR_WINDOW );
    wxColour fg = wxSystemSettings::GetColour( wxSYS_COLOUR_WINDOWTEXT );

    KIGFX::COLOR4D fgC( fg );
    KIGFX::COLOR4D bgC( bg );
    wxColour       gridMajor = fgC.Mix( bgC, 0.35 ).ToColour();
    wxColour       gridMinor = fgC.Mix( bgC, 0.15 ).ToColour();

    bool     isDark = KIGFX::COLOR4D( bg ).GetBrightness() < 0.5;
    wxColour traceColor = isDark ? wxColour( 100, 180, 255 ) : wxColour( 0, 80, 180 );

    wxColour refColor = isDark ? wxColour( 220, 200, 50 ) : wxColour( 180, 150, 0 );
    wxColour transferColor = isDark ? wxColour( 80, 220, 140 ) : wxColour( 0, 140, 80 );

    m_plotWindow->SetColourTheme( bg, fg, fg );
    m_labelOverlay->SetGridColors( gridMajor, gridMinor );
    m_labelOverlay->SetReferenceLabelColor( refColor );
    m_labelOverlay->SetTransferLabelColor( transferColor );
    m_impedanceTrace->SetPen( wxPen( traceColor, 2 ) );
    m_referenceTrace->SetPen( wxPen( refColor, 1, wxPENSTYLE_SOLID ) );
    m_transferTrace->SetPen( wxPen( transferColor, 1, wxPENSTYLE_SOLID ) );
}


void PDN_PANEL::onThemeChanged( wxSysColourChangedEvent& aEvent )
{
    aEvent.Skip();
    updateThemeColors();
    m_plotWindow->UpdateAll();
}


void PDN_PANEL::initModelDownloader()
{
    wxString projectPath = m_frame->Schematic().Prj().GetProjectPath();

    if( projectPath.IsEmpty() )
        return;

    m_downloader = std::make_unique<SPICE_MODEL_DOWNLOADER>( projectPath );
    m_analyzer.SetModelDownloader( m_downloader.get() );
}


void PDN_PANEL::UpdateNetworks()
{
    m_analyzer.SetSchematic( &m_frame->Schematic() );

    if( !m_downloader )
        initModelDownloader();

    m_analyzer.FindPDNNetworks();

    const std::map<wxString, PDN_NETWORK>& networks = m_analyzer.GetNetworks();

    if( !m_selectedNetwork.IsEmpty() && networks.find( m_selectedNetwork ) == networks.end() )
        m_selectedNetwork.clear();

    if( m_selectedNetwork.IsEmpty() && !networks.empty() )
        m_selectedNetwork = networks.begin()->first;

    if( !m_selectedNetwork.IsEmpty() )
        runAnalysisAndPlot( m_selectedNetwork );
    else
        m_plotWindow->Fit();
}


void PDN_PANEL::OnHighlightedNetChanged( const wxString& aNetName )
{
    if( aNetName.IsEmpty() )
        return;

    wxString key = m_analyzer.FindNetworkForNetName( aNetName );

    if( key.IsEmpty() )
        return;

    SCH_SHEET_PATH currentSheet = m_frame->GetCurrentSheet();

    if( key == m_selectedNetwork && currentSheet == m_observationSheet )
        return;

    m_observationSheet = currentSheet;
    m_selectedNetwork = key;
    runAnalysisAndPlot( m_selectedNetwork );
}


void PDN_PANEL::OnSchSelectionChanged( wxCommandEvent& aEvent )
{
    if( !IsShownOnScreen() )
        return;

    wxString highlighted = m_frame->GetHighlightedConnection();

    if( !highlighted.IsEmpty() && !m_analyzer.FindNetworkForNetName( highlighted ).IsEmpty() )
        return;

    TOOL_MANAGER*       toolMgr = m_frame->GetToolManager();
    SCH_SELECTION_TOOL* selTool = toolMgr->GetTool<SCH_SELECTION_TOOL>();
    wxString            key;

    for( EDA_ITEM* item : selTool->GetSelection() )
    {
        SCH_SYMBOL* symbol = dynamic_cast<SCH_SYMBOL*>( item );

        if( !symbol )
            symbol = dynamic_cast<SCH_SYMBOL*>( item->GetParent() );

        if( symbol )
        {
            wxString refdes = symbol->GetRef( &m_frame->GetCurrentSheet() );
            key = m_analyzer.FindNetworkForSym( refdes );

            if( key.IsEmpty() )
            {
                // Symbol isn't a cap in any network — check pin connections
                // (handles power symbols, etc.)
                for( SCH_PIN* pin : symbol->GetPins( &m_frame->GetCurrentSheet() ) )
                {
                    if( SCH_CONNECTION* conn = pin->Connection( &m_frame->GetCurrentSheet() ) )
                    {
                        key = m_analyzer.FindNetworkForNetName( conn->Name() );

                        if( !key.IsEmpty() )
                            break;
                    }
                }
            }

            if( !key.IsEmpty() )
                break;
        }

        if( SCH_ITEM* schItem = dynamic_cast<SCH_ITEM*>( item ) )
        {
            if( SCH_CONNECTION* conn = schItem->Connection( &m_frame->GetCurrentSheet() ) )
            {
                key = m_analyzer.FindNetworkForNetName( conn->Name() );

                if( !key.IsEmpty() )
                    break;
            }
        }
    }

    if( key.IsEmpty() )
        return;

    m_observationSheet = m_frame->GetCurrentSheet();
    m_selectedNetwork = key;
    runAnalysisAndPlot( m_selectedNetwork );
}


void PDN_PANEL::onPlotRightClick( wxMouseEvent& aEvent )
{
    const std::map<wxString, PDN_NETWORK>& networks = m_analyzer.GetNetworks();

    if( networks.empty() )
        return;

    SCH_SHEET_PATH previousSheet = m_observationSheet;
    m_observationSheet = m_frame->GetCurrentSheet();

    std::vector<wxString> visibleKeys;

    for( const auto& [key, network] : networks )
    {
        if( m_analyzer.NetworkHasSymsOnSheet( key, m_observationSheet ) )
            visibleKeys.push_back( key );
    }

    if( visibleKeys.empty() )
        return;

    wxMenu menu;
    int    menuId = wxID_HIGHEST + 1;

    for( const wxString& netKey : visibleKeys )
    {
        const PDN_NETWORK& network = networks.at( netKey );
        wxString label = wxString::Format( wxS( "%s / %s" ), network.supplyRail, network.refRail );

        wxMenuItem* item = menu.AppendCheckItem( menuId, label );

        if( netKey == m_selectedNetwork && m_observationSheet == previousSheet )
            item->Check( true );

        menuId++;
    }

    // "Get SPICE Model..." submenu for unresolved ferrite beads
    struct FB_MENU_ENTRY
    {
        wxString mpn;
        wxString refdes;
        wxString value; ///< Symbol Value field (fallback for MPN)
    };

    std::vector<FB_MENU_ENTRY> fbEntries;

    if( m_downloader )
    {
        // Look up symbol Value fields for ferrite beads
        SCH_SHEET_LIST     sheets = m_frame->Schematic().Hierarchy();
        SCH_REFERENCE_LIST references;
        sheets.GetSymbols( references, false );

        std::map<wxString, wxString> refToValue;

        for( size_t i = 0; i < references.GetCount(); i++ )
            refToValue[references[i].GetRef()] = references[i].GetValue();

        for( const PDN_SERIES_ELEMENT& elem : m_analyzer.GetSeriesElements() )
        {
            if( elem.type != PDN_SERIES_TYPE::FERRITE_BEAD )
                continue;

            // Only show unresolved ferrite beads (no model yet)
            if( !elem.spiceModel.IsEmpty()
                && ( !elem.spiceLibFile.IsEmpty() || !elem.spiceText.IsEmpty() ) )
                continue;

            FB_MENU_ENTRY entry;
            entry.refdes = elem.refdes;
            entry.mpn = elem.mpn;

            auto valIt = refToValue.find( elem.refdes );

            if( valIt != refToValue.end() )
                entry.value = valIt->second;

            fbEntries.push_back( entry );
        }
    }

    if( !fbEntries.empty() )
    {
        menu.AppendSeparator();

        wxMenu* fbMenu = new wxMenu();

        for( size_t i = 0; i < fbEntries.size(); i++ )
        {
            wxString label;
            wxString displayMpn = fbEntries[i].mpn;

            if( displayMpn.IsEmpty() )
                displayMpn = fbEntries[i].value;

            if( !displayMpn.IsEmpty() )
            {
                label = wxString::Format( _( "%s — download %s" ), fbEntries[i].refdes,
                                          displayMpn );
            }
            else
            {
                label = wxString::Format( _( "%s — enter part number..." ), fbEntries[i].refdes );
            }

            fbMenu->Append( menuId, label );
            menuId++;
        }

        menu.AppendSubMenu( fbMenu, _( "Get SPICE Model" ) );
    }

    int fbMenuBase = wxID_HIGHEST + 1 + static_cast<int>( visibleKeys.size() );

    menu.AppendSeparator();

    int         showTransferId = menuId++;
    wxMenuItem* transferItem =
            menu.AppendCheckItem( showTransferId, _( "Show Transfer Impedance" ) );
    transferItem->Check( m_showTransfer );

    bool hasImpedanceData = m_impedanceTrace->IsVisible();
    bool hasReference = !m_refFrequencies.empty();

    int storeRefId = menuId++;
    int clearRefId = menuId++;

    menu.Append( storeRefId, _( "Store as Reference" ) );
    menu.Enable( storeRefId, hasImpedanceData );

    menu.Append( clearRefId, _( "Clear Reference" ) );
    menu.Enable( clearRefId, hasReference );

    menu.Bind( wxEVT_COMMAND_MENU_SELECTED,
               [this, &visibleKeys, &fbEntries, fbMenuBase, showTransferId, storeRefId,
                clearRefId]( wxCommandEvent& evt )
               {
                   int id = evt.GetId();
                   int netIdx = id - ( wxID_HIGHEST + 1 );

                   if( netIdx >= 0 && netIdx < static_cast<int>( visibleKeys.size() ) )
                   {
                       wxString key = visibleKeys[netIdx];
                       m_selectedNetwork = key;
                       runAnalysisAndPlot( key );

                       const PDN_NETWORK& network = m_analyzer.GetNetworks().at( key );
                       m_frame->SetHighlightedConnection( network.supplyRail );
                       m_frame->GetToolManager()->RunAction(
                               SCH_ACTIONS::updateNetHighlighting );
                   }

                   int fbIdx = id - fbMenuBase;

                   if( fbIdx >= 0 && fbIdx < static_cast<int>( fbEntries.size() ) )
                   {
                       onGetSpiceModel( fbEntries[fbIdx].mpn, fbEntries[fbIdx].refdes,
                                        fbEntries[fbIdx].value );
                   }

                   if( id == showTransferId )
                   {
                       m_showTransfer = !m_showTransfer;

                       if( !m_showTransfer )
                       {
                           m_transferTrace->SetVisible( false );
                           m_labelOverlay->SetTransferLabel( wxEmptyString );
                           m_labelOverlay->SetTransferClampIndicators( false, 0, false, 0 );
                           m_plotWindow->UpdateAll();
                       }
                       else if( !m_selectedNetwork.IsEmpty() )
                       {
                           runAnalysisAndPlot( m_selectedNetwork );
                       }
                   }

                   if( id == storeRefId )
                   {
                       m_refFrequencies = m_analyzer.GetFrequencies();
                       m_refImpedance = m_analyzer.GetImpedance();

                       const auto& nets = m_analyzer.GetNetworks();
                       auto        netIt = nets.find( m_selectedNetwork );

                       if( netIt != nets.end() )
                       {
                           m_refLabel =
                                   wxString::Format( wxS( "%s / %s" ), netIt->second.supplyRail,
                                                     netIt->second.refRail );
                       }

                       // Convert to log10 and set on the trace
                       std::vector<double> logFreqs( m_refFrequencies.size() );
                       std::vector<double> logZ( m_refImpedance.size() );

                       for( size_t i = 0; i < m_refFrequencies.size(); i++ )
                       {
                           logFreqs[i] = ( m_refFrequencies[i] > 0.0 )
                                                 ? std::log10( m_refFrequencies[i] )
                                                 : 0.0;
                       }

                       for( size_t i = 0; i < m_refImpedance.size(); i++ )
                       {
                           logZ[i] = ( m_refImpedance[i] > 0.0 ) ? std::log10( m_refImpedance[i] )
                                                                 : -12.0;
                       }

                       m_referenceTrace->SetData( logFreqs, logZ );
                       m_referenceTrace->SetVisible( true );
                       m_labelOverlay->SetReferenceLabel(
                               wxString::Format( _( "ref: %s" ), m_refLabel ) );
                       m_plotWindow->UpdateAll();
                   }

                   if( id == clearRefId )
                   {
                       m_refFrequencies.clear();
                       m_refImpedance.clear();
                       m_refLabel.clear();
                       m_referenceTrace->SetVisible( false );
                       m_labelOverlay->SetReferenceLabel( wxEmptyString );
                       m_plotWindow->UpdateAll();
                   }
               } );

    m_plotWindow->PopupMenu( &menu, aEvent.GetPosition() );
}


void PDN_PANEL::onGetSpiceModel( const wxString& aMpn, const wxString& aRefdes,
                                 const wxString& aValue )
{
    if( !m_downloader )
        return;

    wxString mpn = aMpn;

    // Fall back to Value field if no explicit MPN
    if( mpn.IsEmpty() && !aValue.IsEmpty() && m_downloader->HasSource( aValue ) )
        mpn = aValue;

    if( mpn.IsEmpty() || !m_downloader->HasSource( mpn ) )
    {
        // Need to ask the user — pre-fill with whatever we have
        wxString prefill = !mpn.IsEmpty() ? mpn : aValue;

        wxTextEntryDialog dlg(
                this, wxString::Format( _( "Enter manufacturer part number for %s:" ), aRefdes ),
                _( "Get SPICE Model" ), prefill );

        if( dlg.ShowModal() != wxID_OK )
            return;

        mpn = dlg.GetValue();
        mpn.Trim( true ).Trim( false );

        if( mpn.IsEmpty() )
            return;

        if( !m_downloader->HasSource( mpn ) )
        {
            wxMessageBox( wxString::Format( _( "No known manufacturer source for '%s'.\n\n"
                                               "Part numbers must start with a recognized prefix "
                                               "(e.g. BLM, GRM for Murata)." ),
                                            mpn ),
                          _( "Get SPICE Model" ), wxICON_WARNING | wxOK, this );
            return;
        }
    }

    wxBusyCursor       wait;
    SPICE_MODEL_RESULT result = m_downloader->FetchModel( mpn );

    if( !result.success )
    {
        wxMessageBox( wxString::Format( _( "Failed to download model for '%s':\n%s" ), mpn,
                                        result.errorMsg ),
                      _( "Get SPICE Model" ), wxICON_ERROR | wxOK, this );
        return;
    }

    // Optionally assign Sim.Library / Sim.Name to the symbol for general simulator use
    SCH_SHEET_LIST     sheets = m_frame->Schematic().Hierarchy();
    SCH_REFERENCE_LIST references;
    sheets.GetSymbols( references, false );

    for( size_t i = 0; i < references.GetCount(); i++ )
    {
        const SCH_REFERENCE& ref = references[i];

        if( ref.GetRef() != aRefdes )
            continue;

        SCH_SYMBOL*           symbol = ref.GetSymbol();
        const SCH_SHEET_PATH& sheetPath = ref.GetSheetPath();
        SCH_SCREEN*           screen = sheetPath.LastScreen();

        SCH_COMMIT commit( m_frame->GetToolManager() );
        commit.Modify( symbol, screen );

        // Set or update Sim.Library field
        SCH_FIELD* libField = symbol->FindField( wxT( "Sim.Library" ) );

        if( !libField )
        {
            SCH_FIELD newField( symbol, symbol->GetFieldCount(), wxT( "Sim.Library" ) );
            newField.SetText( result.cachedPath );
            symbol->AddField( newField );
        }
        else
        {
            libField->SetText( result.cachedPath );
        }

        // Set or update Sim.Name field
        SCH_FIELD* nameField = symbol->FindField( wxT( "Sim.Name" ) );

        if( !nameField )
        {
            SCH_FIELD newField( symbol, symbol->GetFieldCount(), wxT( "Sim.Name" ) );
            newField.SetText( result.subcktName );
            symbol->AddField( newField );
        }
        else
        {
            nameField->SetText( result.subcktName );
        }

        commit.Push( wxString::Format( _( "Assign SPICE model to %s" ), aRefdes ) );
        break;
    }

    // Re-run analysis with the new model
    UpdateNetworks();
}


void PDN_PANEL::onSchematicChanged( wxCommandEvent& aEvent )
{
    aEvent.Skip();
    UpdateNetworks();
}


void PDN_PANEL::runAnalysisAndPlot( const wxString& aNetworkKey )
{
    const std::map<wxString, PDN_NETWORK>& networks = m_analyzer.GetNetworks();

    auto it = networks.find( aNetworkKey );

    if( it == networks.end() )
        return;

    const PDN_NETWORK& network = it->second;

    m_labelOverlay->SetNetworkLabel(
            wxString::Format( wxS( "%s / %s" ), network.supplyRail, network.refRail ) );

    bool multiSheet = m_analyzer.NetworkIsMultiSheet( aNetworkKey )
                      && m_analyzer.NetworkHasSymsOnSheet( aNetworkKey, m_observationSheet );

    if( multiSheet )
    {
        SCH_SHEET* last = m_observationSheet.Last();

        if( last )
            m_labelOverlay->SetSheetLabel( last->GetName() );
        else
            m_labelOverlay->SetSheetLabel( wxEmptyString );
    }
    else
    {
        m_labelOverlay->SetSheetLabel( wxEmptyString );
    }

    if( m_analyzer.RunAnalysis( aNetworkKey, m_observationSheet ) )
    {
        plotImpedance( m_analyzer.GetFrequencies(), m_analyzer.GetImpedance() );
    }
    else
    {
        m_labelOverlay->SetNetworkLabel( _( "Analysis failed" ) );
        m_labelOverlay->SetSheetLabel( wxEmptyString );
        m_plotWindow->UpdateAll();
    }

    updateWarningIndicator();
}


void PDN_PANEL::plotImpedance( const std::vector<double>& aFreqs,
                               const std::vector<double>& aImpedance )
{
    if( aFreqs.empty() || aImpedance.empty() || aFreqs.size() != aImpedance.size() )
        return;

    std::vector<double> freqs( aFreqs.size() );
    std::vector<double> impedance( aImpedance.size() );

    for( size_t i = 0; i < aFreqs.size(); i++ )
        freqs[i] = ( aFreqs[i] > 0.0 ) ? std::log10( aFreqs[i] ) : 0.0;

    for( size_t i = 0; i < aImpedance.size(); i++ )
        impedance[i] = ( aImpedance[i] > 0.0 ) ? std::log10( aImpedance[i] ) : -12.0;

    m_impedanceTrace->SetData( freqs, impedance );
    m_impedanceTrace->SetVisible( true );

    // Populate combined transfer impedance trace
    const std::vector<double>& transferData = m_analyzer.GetTransferImpedance();
    bool                       clampedLeft = false;
    bool                       clampedRight = false;
    double                     clampLeftX = 0.0;
    double                     clampRightX = 0.0;

    if( m_showTransfer && !transferData.empty() && transferData.size() == aFreqs.size() )
    {
        std::vector<double> logZ( transferData.size() );

        for( size_t i = 0; i < transferData.size(); i++ )
        {
            double raw = ( transferData[i] > 0.0 ) ? std::log10( transferData[i] ) : -12.0;
            logZ[i] = std::max( raw, static_cast<double>( LOG_Z_MIN ) );
        }

        // Track clamping at endpoints
        if( logZ.front() <= LOG_Z_MIN )
        {
            clampedLeft = true;
            clampLeftX = freqs.front();
        }

        if( logZ.back() <= LOG_Z_MIN )
        {
            clampedRight = true;
            clampRightX = freqs.back();
        }

        m_transferTrace->SetData( freqs, logZ );
        m_transferTrace->SetVisible( true );
        m_labelOverlay->SetTransferLabel( wxS( "\u2192 Z21" ) );
    }
    else
    {
        m_transferTrace->SetVisible( false );
        m_labelOverlay->SetTransferLabel( wxEmptyString );
    }

    m_labelOverlay->SetTransferClampIndicators( clampedLeft, clampLeftX, clampedRight,
                                                clampRightX );

    m_xAxis->ResetDataRange();
    m_yAxis->ResetDataRange();
    m_impedanceTrace->UpdateScales();

    if( m_transferTrace->IsVisible() )
        m_transferTrace->UpdateScales();

    if( m_referenceTrace->IsVisible() )
        m_referenceTrace->UpdateScales();

    // Snap axes to integer decades, with a small margin so major grid
    // lines don't sit right on the plot boundary.
    double xMin, xMax;
    m_xAxis->GetDataRange( xMin, xMax );
    m_xAxis->ResetDataRange();
    m_xAxis->ExtendDataRange( floor( xMin ) - 0.1, ceil( xMax ) + 0.1 );

    m_yAxis->ResetDataRange();
    m_yAxis->ExtendDataRange( LOG_Z_MIN, LOG_Z_MAX );

    m_plotWindow->UpdateAll();
    m_plotWindow->Fit();
}
