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

#ifndef IMPEDANCE_PROFILER_PANEL_H
#define IMPEDANCE_PROFILER_PANEL_H

#include <board.h>
#include <sipi/cross_section.h>
#include <widgets/wx_panel.h>

class CROSSHAIR_LAYER;
class PCB_EDIT_FRAME;
class mpWindow;
class mpFXYVector;
class mpScaleX;
class mpScaleY;
class wxStaticText;


/**
 * Per-sample snapshot of the cross-section used for the BEM solve.
 */
struct XS_SAMPLE
{
    double      distMm = 0.0;       ///< Distance along trace (mm)
    double      timePsec = 0.0;    ///< Cumulative propagation delay (ps)
    double      z0 = 0.0;          ///< Computed Z₀ (Ohms)
    XS_GEOMETRY geometry;           ///< Cross-section fed to solver
    VECTOR2I    boardPos;           ///< Board position of this sample (nm)
    bool        hasRefAbove = false;
    bool        hasRefBelow = false;
    double      hAbove = 0.0;       ///< Dielectric height above (m)
    double      hBelow = 0.0;       ///< Dielectric height below (m)
    double      erEff = 1.0;        ///< Effective εr at this sample
    int         neighborCount = 0;  ///< Number of neighbor conductors
    int         groundwireCount = 0;///< Number of groundwire conductors
};


/**
 * Small panel that renders a 2D cross-section view of the trace geometry
 * at a selected point along the impedance profile.
 */
class XS_VIEW_PANEL : public wxPanel
{
public:
    XS_VIEW_PANEL( wxWindow* aParent );

    void SetSample( const XS_SAMPLE* aSample );

    /// Set the fixed horizontal extent (meters) so all samples use the same scale.
    void SetXExtent( double aExtent ) { m_xExtent = aExtent; }

private:
    void onPaint( wxPaintEvent& aEvent );
    void drawCrossSection( wxDC& aDC, const wxRect& aRect );

    const XS_SAMPLE* m_sample;
    double           m_xExtent;    ///< Fixed half-width in meters (0 = auto)
};


/**
 * Dockable panel in Pcbnew showing trace impedance (Z0) vs. distance
 * along a selected net.
 *
 * Pipeline: track selection -> AnalyseNet() -> TRACE_PATH_WALKER -> BEM -> plot
 */
class IMPEDANCE_PROFILER_PANEL : public WX_PANEL, public BOARD_LISTENER
{
public:
    IMPEDANCE_PROFILER_PANEL( PCB_EDIT_FRAME* aParent );
    ~IMPEDANCE_PROFILER_PANEL() override;

    void OnShowPanel();

    /// Analyse a specific net (called from board context menu or selection).
    void AnalyseNet( int aNetCode );

    // BOARD_LISTENER overrides
    void OnBoardItemAdded( BOARD& aBoard, BOARD_ITEM* aBoardItem ) override;
    void OnBoardItemsAdded( BOARD& aBoard, std::vector<BOARD_ITEM*>& aBoardItems ) override;
    void OnBoardItemRemoved( BOARD& aBoard, BOARD_ITEM* aBoardItem ) override;
    void OnBoardItemsRemoved( BOARD& aBoard, std::vector<BOARD_ITEM*>& aBoardItems ) override;
    void OnBoardItemChanged( BOARD& aBoard, BOARD_ITEM* aBoardItem ) override;
    void OnBoardItemsChanged( BOARD& aBoard, std::vector<BOARD_ITEM*>& aBoardItems ) override;
    void OnBoardHighlightNetChanged( BOARD& aBoard ) override;

private:
    void buildUI();
    void onPlotMotion( wxMouseEvent& aEvent );
    void onContextMenuCommand( wxCommandEvent& aEvent );
    void runAnalysis( int aNetCode );
    void updatePlot();
    void updateStatus( const wxString& aText );
    void selectSample( int aIndex );

    PCB_EDIT_FRAME* m_frame;
    bool            m_initialized;  ///< true after first OnShowPanel() builds UI

    // Plot
    mpWindow*       m_plotWindow;
    mpFXYVector*      m_impedanceTrace;
    mpFXYVector*      m_targetLine;
    CROSSHAIR_LAYER*  m_crosshair;
    mpScaleX*         m_xAxis;
    mpScaleY*       m_yAxis;

    // Cross-section view
    XS_VIEW_PANEL*  m_xsView;
    wxStaticText*   m_xsDiagText;       ///< Diagnostic info inside XS panel

    // Status
    wxStaticText*   m_statusText;

    // Analysis results
    std::vector<XS_SAMPLE> m_samples;
    int             m_selectedSample;

    // State
    int             m_currentNetCode;
    wxString        m_currentNetName;
    bool            m_xAxisIsTime;        ///< true = ps, false = mm
    bool            m_showCrossSection;   ///< cross-section panel visibility

    // Context menu IDs
    enum
    {
        ID_XAXIS_MM = wxID_HIGHEST + 1,
        ID_XAXIS_PS,
        ID_SHOW_XS,
    };
};

#endif // IMPEDANCE_PROFILER_PANEL_H
