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

#ifndef PDN_PANEL_H
#define PDN_PANEL_H

#include <widgets/wx_panel.h>
#include <sim/pdn_analyzer.h>
#include <sim/spice_model_downloader.h>
#include <schematic.h>
#include <sch_sheet_path.h>

#include <memory>

class SCH_EDIT_FRAME;
class mpWindow;
class mpFXYVector;
class PDN_XSCALE;
class PDN_YSCALE;
class PDN_LABEL_OVERLAY;
class wxStaticBitmap;


/**
 * Panel for displaying PDN (Power Delivery Network) impedance analysis.
 *
 * Shows an impedance vs. frequency plot for the selected PDN network.
 * Right-click to choose from detected networks; clicking a power net
 * in the schematic auto-selects the matching network.
 */
class PDN_PANEL : public WX_PANEL, public SCHEMATIC_LISTENER
{
public:
    PDN_PANEL( SCH_EDIT_FRAME* aParent );
    ~PDN_PANEL();

    /**
     * Re-scan the schematic for PDN networks.
     */
    void UpdateNetworks();

    /**
     * Called when net highlighting changes.  If the net matches a PDN network,
     * the panel updates to show that network.  Non-PDN nets are ignored.
     */
    void OnHighlightedNetChanged( const wxString& aNetName );

    /**
     * Called when the schematic selection changes.  Inspects the current
     * selection for capacitor symbols or nets and updates the panel
     * accordingly.  Checks its own visibility before doing any work.
     */
    void OnSchSelectionChanged();

    // SCHEMATIC_LISTENER
    void OnSchItemsAdded( SCHEMATIC& aSch, std::vector<SCH_ITEM*>& aItems ) override;
    void OnSchItemsRemoved( SCHEMATIC& aSch, std::vector<SCH_ITEM*>& aItems ) override;
    void OnSchItemsChanged( SCHEMATIC& aSch, std::vector<SCH_ITEM*>& aItems ) override;

private:
    void onPlotRightClick( wxMouseEvent& aEvent );
    void onSchematicChanged( wxCommandEvent& aEvent );
    void onThemeChanged( wxSysColourChangedEvent& aEvent );
    void updateThemeColors();
    void initModelDownloader();
    void runAnalysisAndPlot( const wxString& aNetworkKey );
    void plotImpedance( const std::vector<double>& aFreqs, const std::vector<double>& aImpedance );
    void updateWarningIndicator();
    void onGetSpiceModel( const wxString& aMpn, const wxString& aRefdes, const wxString& aValue );

    void scheduleUpdate();

    SCH_EDIT_FRAME*                         m_frame;
    PDN_ANALYZER                            m_analyzer;
    std::unique_ptr<SPICE_MODEL_DOWNLOADER> m_downloader;
    bool                                    m_updatePending;

    wxString           m_selectedNetwork;
    SCH_SHEET_PATH     m_observationSheet;
    mpWindow*          m_plotWindow;
    mpFXYVector*       m_impedanceTrace;
    mpFXYVector*       m_referenceTrace;
    mpFXYVector*       m_transferTrace;
    bool               m_showTransfer;
    PDN_XSCALE*        m_xAxis;
    PDN_YSCALE*        m_yAxis;
    PDN_LABEL_OVERLAY* m_labelOverlay;
    wxStaticBitmap*    m_warningIcon;

    std::vector<double> m_refFrequencies;
    std::vector<double> m_refImpedance;
    wxString            m_refLabel;
};

#endif // PDN_PANEL_H
