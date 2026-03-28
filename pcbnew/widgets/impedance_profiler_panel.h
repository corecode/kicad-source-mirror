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
#include <widgets/wx_panel.h>

class PCB_EDIT_FRAME;
class mpWindow;
class mpFXYVector;
class mpScaleX;
class mpScaleY;
class wxChoice;
class wxTextCtrl;
class wxStaticText;


/**
 * Dockable panel in Pcbnew showing trace impedance (Z0) vs. distance
 * along a selected net.
 *
 * Pipeline: net selection -> TRACE_PATH_WALKER -> STACKUP_READER -> Z0 -> plot
 */
class IMPEDANCE_PROFILER_PANEL : public WX_PANEL, public BOARD_LISTENER
{
public:
    IMPEDANCE_PROFILER_PANEL( PCB_EDIT_FRAME* aParent );
    ~IMPEDANCE_PROFILER_PANEL() override;

    /**
     * Called when the panel is shown — refreshes net list and optionally auto-analyses.
     */
    void OnShowPanel();

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
    void populateNetList();
    void onAnalyseClicked( wxCommandEvent& aEvent );
    void onNetSelected( wxCommandEvent& aEvent );
    void runAnalysis( int aNetCode );
    void updatePlot( const std::vector<double>& aPositions,
                     const std::vector<double>& aImpedances,
                     const std::vector<double>& aNeighborDists );
    void updateStatus( const wxString& aText );

    PCB_EDIT_FRAME* m_frame;

    // Controls
    wxChoice*       m_netSelector;
    wxTextCtrl*     m_targetZ0Input;
    wxStaticText*   m_statusText;

    // Plot
    mpWindow*       m_plotWindow;
    mpFXYVector*    m_impedanceTrace;
    mpFXYVector*    m_targetLine;
    mpFXYVector*    m_neighborTrace;    ///< Nearest neighbor distance overlay
    mpScaleX*       m_xAxis;
    mpScaleY*       m_yAxis;

    // State
    int             m_currentNetCode;
    double          m_targetZ0;
};

#endif // IMPEDANCE_PROFILER_PANEL_H
