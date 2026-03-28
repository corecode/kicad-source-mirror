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

    // Build impedance profile using BEM solver with neighbor detection.
    // At each sample point, find nearby traces on the same layer and include
    // them in the BEM cross-section. This makes Z₀ vary along the route as
    // the trace passes through areas with different coupling.
    STACKUP_READER stackup( board );

    std::vector<double> positions;
    std::vector<double> impedances;

    // Coupling horizon: ignore traces farther than this (nm).
    // 3× the max dielectric height is the standard rule of thumb.
    int couplingHorizon = 2000000; // 2mm default

    // Cache keyed on quantized neighbor configuration to avoid redundant BEM solves.
    // Key: (layer, width, refAbove, refBelow, quantized neighbor distances)
    struct XS_KEY
    {
        PCB_LAYER_ID       layer;
        int                width;
        bool               refAbove;
        bool               refBelow;
        std::vector<int>   neighborKeys; // quantized (distance_um, width_nm) pairs

        bool operator<( const XS_KEY& o ) const
        {
            if( layer != o.layer ) return layer < o.layer;
            if( width != o.width ) return width < o.width;
            if( refAbove != o.refAbove ) return refAbove < o.refAbove;
            if( refBelow != o.refBelow ) return refBelow < o.refBelow;
            return neighborKeys < o.neighborKeys;
        }
    };

    std::map<XS_KEY, double> z0Cache;

    // Collect all track segments on the board (for neighbor search)
    // grouped by layer for efficiency.
    std::map<PCB_LAYER_ID, std::vector<PCB_TRACK*>> tracksByLayer;

    for( PCB_TRACK* t : board->Tracks() )
    {
        if( t->Type() == PCB_VIA_T )
            continue;

        tracksByLayer[t->GetLayer()].push_back( t );
    }

    // Sample at each path point (not just per-segment) since neighbors vary
    int  sampleInterval = std::max( 1, (int) path.size() / 200 ); // limit to ~200 samples
    bool lastWasVia = false;

    for( int pi = 0; pi < (int) path.size(); pi++ )
    {
        const PATH_POINT& pt = path[pi];

        if( pt.isVia )
        {
            lastWasVia = true;
            continue;
        }

        // Sample every Nth point (but always sample after a via and at start/end)
        if( pi % sampleInterval != 0 && !lastWasVia
            && pi != 0 && pi != (int) path.size() - 1 )
        {
            continue;
        }

        lastWasVia = false;

        PCB_TRACK* track = static_cast<PCB_TRACK*>( pt.item );

        LAYER_GEOMETRY geom = stackup.GetLayerGeometry( track->GetLayer(),
                                                        pt.position,
                                                        track->GetWidth() );

        // Find neighbor traces at this point.
        // Project each nearby track onto the perpendicular cut line and measure
        // the lateral distance from our signal trace center.
        struct NEIGHBOR
        {
            int distNm; // signed distance from signal center (nm), positive = right
            int widthNm;
        };

        std::vector<NEIGHBOR> neighbors;

        VECTOR2D normal( -pt.tangent.y, pt.tangent.x ); // perpendicular to trace direction

        auto it = tracksByLayer.find( track->GetLayer() );

        if( it != tracksByLayer.end() )
        {
            for( PCB_TRACK* other : it->second )
            {
                if( other == track )
                    continue;

                // Quick bounding-box rejection
                VECTOR2I mid( ( other->GetStart().x + other->GetEnd().x ) / 2,
                              ( other->GetStart().y + other->GetEnd().y ) / 2 );

                VECTOR2D delta( mid.x - pt.position.x, mid.y - pt.position.y );
                double alongDist = delta.x * pt.tangent.x + delta.y * pt.tangent.y;

                // Only consider tracks that overlap with our current segment
                double otherHalfLen = other->GetLength() / 2.0 + track->GetLength() / 2.0;

                if( std::abs( alongDist ) > otherHalfLen )
                    continue;

                // Lateral distance
                double lateralDist = delta.x * normal.x + delta.y * normal.y;

                if( std::abs( lateralDist ) > couplingHorizon )
                    continue;

                neighbors.push_back(
                        { static_cast<int>( lateralDist ), other->GetWidth() } );
            }
        }

        // Sort neighbors by absolute distance for deterministic cache key
        std::sort( neighbors.begin(), neighbors.end(),
                   []( const NEIGHBOR& a, const NEIGHBOR& b )
                   {
                       return std::abs( a.distNm ) < std::abs( b.distNm );
                   } );

        // Limit to 4 nearest neighbors (BEM cost grows with conductor count)
        if( neighbors.size() > 4 )
            neighbors.resize( 4 );

        // Build cache key with quantized neighbor distances (10µm buckets)
        XS_KEY key;
        key.layer = track->GetLayer();
        key.width = track->GetWidth();
        key.refAbove = geom.hasRefAbove;
        key.refBelow = geom.hasRefBelow;

        for( const NEIGHBOR& nb : neighbors )
        {
            key.neighborKeys.push_back( nb.distNm / 10000 ); // quantize to 10µm
            key.neighborKeys.push_back( nb.widthNm );
        }

        double z0 = 0.0;
        auto cacheIt = z0Cache.find( key );

        if( cacheIt != z0Cache.end() )
        {
            z0 = cacheIt->second;
        }
        else
        {
            // Build BEM cross-section
            XS_GEOMETRY xs;
            double      condY = 0.0;

            bool hasAbove = ( geom.hAbove > 0.0 );
            bool hasBelow = ( geom.hBelow > 0.0 );

            if( hasAbove && hasBelow )
            {
                xs.groundY = 0.0;
                xs.hasUpperGround = true;
                xs.upperGroundY = -( geom.hAbove + geom.hBelow );
                condY = -geom.hBelow;
                xs.epsilonR = ( geom.erAbove + geom.erBelow ) / 2.0;

                if( std::abs( geom.erAbove - geom.erBelow ) > 0.5 )
                {
                    XS_DIELECTRIC_REGION regA, regB;
                    regA.yTop = xs.upperGroundY;
                    regA.yBottom = condY;
                    regA.epsilonR = geom.erAbove;
                    regB.yTop = condY;
                    regB.yBottom = 0.0;
                    regB.epsilonR = geom.erBelow;
                    xs.dielectrics.push_back( regA );
                    xs.dielectrics.push_back( regB );
                }
            }
            else if( hasBelow )
            {
                xs.groundY = 0.0;
                condY = -( geom.hBelow + geom.traceThickness / 2.0 );
                xs.epsilonR = geom.erBelow;

                XS_DIELECTRIC_REGION air, diel;
                air.yTop = -10e-3;
                air.yBottom = -geom.hBelow;
                air.epsilonR = 1.0;
                diel.yTop = -geom.hBelow;
                diel.yBottom = 0.0;
                diel.epsilonR = geom.erBelow;
                xs.dielectrics.push_back( air );
                xs.dielectrics.push_back( diel );
            }
            else if( hasAbove )
            {
                xs.groundY = -( geom.hAbove + geom.traceThickness );
                condY = -geom.traceThickness / 2.0;
                xs.epsilonR = geom.erAbove;

                XS_DIELECTRIC_REGION diel, air;
                diel.yTop = xs.groundY;
                diel.yBottom = -geom.traceThickness;
                diel.epsilonR = geom.erAbove;
                air.yTop = -geom.traceThickness;
                air.yBottom = 10e-3;
                air.epsilonR = 1.0;
                xs.dielectrics.push_back( diel );
                xs.dielectrics.push_back( air );
            }

            // Signal conductor at x=0
            XS_CONDUCTOR cond;
            cond.centerX = 0.0;
            cond.centerY = condY;
            cond.width = geom.traceWidth;
            cond.thickness = geom.traceThickness;
            xs.conductors.push_back( cond );

            // Add neighbor conductors at their lateral offsets
            for( const NEIGHBOR& nb : neighbors )
            {
                XS_CONDUCTOR nbCond;
                nbCond.centerX = nb.distNm * 1e-9; // nm → meters
                nbCond.centerY = condY;             // same layer
                nbCond.width = nb.widthNm * 1e-9;
                nbCond.thickness = geom.traceThickness;
                xs.conductors.push_back( nbCond );
            }

            // Solve — Z₀ of conductor 0 (our signal) accounts for coupling
            BEM_2D_SOLVER solver;
            solver.SetGeometry( xs );
            solver.SetPanelsPerEdge( neighbors.empty() ? 15 : 10 );

            if( solver.Solve() && solver.GetResult().Z0 > 0.0 )
            {
                z0 = solver.GetResult().Z0;
            }
            else
            {
                z0 = STACKUP_READER::ComputeZ0( geom );
            }

            z0Cache[key] = z0;
        }

        positions.push_back( pt.distFromStart / 1e6 ); // nm → mm
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
