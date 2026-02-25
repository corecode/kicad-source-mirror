/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "system_diagram_generator.h"
#include "system_diagram_analyzer.h"
#include "system_diagram_layout.h"

#include <schematic.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <sch_shape.h>
#include <sch_text.h>
#include <sch_line.h>
#include <base_units.h>
#include <layer_ids.h>
#include <page_info.h>
#include <title_block.h>
#include <gal/color4d.h>
#include <stroke_params.h>

using KIGFX::COLOR4D;


// Drawing constants (in mils, converted to IU at usage)
static const int SECTION_HEADER_SIZE  = 80;   // Text size for section headers
static const int REF_TEXT_SIZE        = 50;    // Text size for reference designators
static const int VALUE_TEXT_SIZE      = 40;    // Text size for values
static const int LABEL_TEXT_SIZE      = 35;    // Text size for bus/signal labels
static const int WARNING_TEXT_SIZE    = 30;    // Text size for warning label
static const int BOX_LINE_WIDTH      = 10;    // Line width for component boxes
static const int BUS_BAR_WIDTH       = 20;    // Line width for bus bars
static const int BUS_BAR_HEIGHT      = 100;   // Height (thickness) of bus bars in mils
static const int SIGNAL_LINE_WIDTH   = 6;     // Line width for signal lines
static const int POWER_LINE_WIDTH    = 10;    // Line width for power edges
static const int HEADER_LINE_WIDTH   = 10;    // Line width for header underline
static const int ARROW_SIZE          = 60;    // Arrow head size in mils
static const int SECTION_GAP         = 400;   // Gap between sections in mils
static const int MARGIN              = 500;   // Page margin in mils


SYSTEM_DIAGRAM_GENERATOR::SYSTEM_DIAGRAM_GENERATOR( SCHEMATIC* aSchematic ) :
        m_schematic( aSchematic )
{
}


SCH_SHEET* SYSTEM_DIAGRAM_GENERATOR::Generate( SYSTEM_DIAGRAM_DATA& aData )
{
    SCH_SHEET* sheet = getOrCreateSheet();

    if( !sheet )
        return nullptr;

    SCH_SCREEN* screen = sheet->GetScreen();

    if( !screen )
        return nullptr;

    // Clear existing content
    clearSheet( screen );

    int margin = schIUScale.MilsToIU( MARGIN );
    int sectionGap = schIUScale.MilsToIU( SECTION_GAP );
    int headerHeight = schIUScale.MilsToIU( SECTION_HEADER_SIZE + 100 );

    VECTOR2I currentPos( margin, margin );

    // Draw warning label at top
    drawWarningLabel( screen, currentPos );
    currentPos.y += schIUScale.MilsToIU( 150 );

    // Draw bus section if there's content
    if( !aData.m_buses.empty() || !aData.m_signals.empty() )
    {
        drawSectionHeader( screen, wxT( "BUS CONNECTIVITY" ), currentPos );
        currentPos.y += headerHeight;

        drawBusSection( screen, aData, currentPos );

        // Calculate bus section height
        int maxBusY = currentPos.y;

        for( const auto& comp : aData.m_components )
            maxBusY = std::max( maxBusY, comp->m_pos.y + comp->m_size.y );

        currentPos.y = maxBusY + sectionGap;
    }

    // Draw power section if there's content
    if( !aData.m_powerRoots.empty() )
    {
        drawSectionHeader( screen, wxT( "POWER TOPOLOGY" ), currentPos );
        currentPos.y += headerHeight;

        drawPowerSection( screen, aData, currentPos );
    }

    // Calculate total content size and set page
    int maxX = margin;
    int maxY = currentPos.y;

    for( const auto& comp : aData.m_components )
        maxX = std::max( maxX, comp->m_pos.x + comp->m_size.x );

    std::function<void( const SD_POWER_NODE* )> findPowerExtent;
    findPowerExtent = [&]( const SD_POWER_NODE* node )
    {
        maxX = std::max( maxX, node->m_pos.x + node->m_size.x );
        maxY = std::max( maxY, node->m_pos.y + node->m_size.y );

        for( const SD_POWER_NODE* child : node->m_children )
            findPowerExtent( child );
    };

    for( const auto& root : aData.m_powerRoots )
        findPowerExtent( root.get() );

    setPageSize( screen, VECTOR2I( maxX + margin, maxY + margin ) );

    return sheet;
}


SCH_SHEET* SYSTEM_DIAGRAM_GENERATOR::getOrCreateSheet()
{
    SCH_SHEET& rootSheet = m_schematic->Root();
    SCH_SCREEN* rootScreen = rootSheet.GetScreen();

    // Look for existing "System Diagram" sheet
    for( SCH_ITEM* item : rootScreen->Items().OfType( SCH_SHEET_T ) )
    {
        SCH_SHEET* existingSheet = static_cast<SCH_SHEET*>( item );

        if( existingSheet->GetName() == wxT( "System Diagram" ) )
            return existingSheet;
    }

    // Create new sheet
    int margin = schIUScale.MilsToIU( MARGIN );

    SCH_SHEET* newSheet = new SCH_SHEET( &rootSheet,
                                         VECTOR2I( margin, margin ),
                                         VECTOR2I( schIUScale.MilsToIU( 2000 ),
                                                   schIUScale.MilsToIU( 500 ) ) );

    newSheet->SetName( wxT( "System Diagram" ) );
    newSheet->SetFileName( wxT( "system_diagram.kicad_sch" ) );

    // Create a screen for the sheet
    SCH_SCREEN* newScreen = new SCH_SCREEN( m_schematic );
    newSheet->SetScreen( newScreen );
    newScreen->SetFileName( wxT( "system_diagram.kicad_sch" ) );
    newScreen->SetContentModified();

    // Set title block
    TITLE_BLOCK titleBlock;
    titleBlock.SetTitle( wxT( "System Diagram" ) );
    titleBlock.SetComment( 0, wxT( "Auto-generated - edits will be overwritten" ) );
    newScreen->SetTitleBlock( titleBlock );

    // Add the sheet symbol to the root screen
    rootScreen->Append( newSheet );

    return newSheet;
}


void SYSTEM_DIAGRAM_GENERATOR::clearSheet( SCH_SCREEN* aScreen )
{
    aScreen->FreeDrawList();
}


void SYSTEM_DIAGRAM_GENERATOR::setPageSize( SCH_SCREEN* aScreen, const VECTOR2I& aContentSize )
{
    // Convert content size to mils
    double widthMils = schIUScale.IUTomm( aContentSize.x ) / 25.4 * 1000.0;
    double heightMils = schIUScale.IUTomm( aContentSize.y ) / 25.4 * 1000.0;

    // Try standard sizes, pick smallest that fits
    struct SheetSize
    {
        const wxChar* name;
        double        widthMils;
        double        heightMils;
    };

    SheetSize sizes[] = {
        { PAGE_INFO::A4,      8267,  11693 },
        { PAGE_INFO::A3,      11693, 16535 },
        { PAGE_INFO::A2,      16535, 23386 },
        { PAGE_INFO::A1,      23386, 33110 },
        { PAGE_INFO::A0,      33110, 46811 },
    };

    PAGE_INFO pageInfo;

    for( const SheetSize& sz : sizes )
    {
        // Try landscape
        if( widthMils <= sz.heightMils && heightMils <= sz.widthMils )
        {
            pageInfo.SetType( sz.name, false );
            aScreen->SetPageSettings( pageInfo );
            return;
        }

        // Try portrait
        if( widthMils <= sz.widthMils && heightMils <= sz.heightMils )
        {
            pageInfo.SetType( sz.name, true );
            aScreen->SetPageSettings( pageInfo );
            return;
        }
    }

    // Use custom size
    pageInfo.SetType( PAGE_INFO::Custom, false );
    pageInfo.SetWidthMils( widthMils );
    pageInfo.SetHeightMils( heightMils );
    aScreen->SetPageSettings( pageInfo );
}


void SYSTEM_DIAGRAM_GENERATOR::drawBusSection( SCH_SCREEN* aScreen,
                                                const SYSTEM_DIAGRAM_DATA& aData,
                                                VECTOR2I aOrigin )
{
    // Draw component boxes
    for( const auto& comp : aData.m_components )
    {
        // Only draw if this component participates in bus/signal connections
        bool inBus = false;

        for( const auto& bus : aData.m_buses )
        {
            for( const SD_COMPONENT* c : bus->m_connectedComponents )
            {
                if( c == comp.get() )
                {
                    inBus = true;
                    break;
                }
            }

            if( inBus )
                break;
        }

        if( !inBus )
        {
            for( const auto& sig : aData.m_signals )
            {
                for( const SD_COMPONENT* c : sig->m_connectedComponents )
                {
                    if( c == comp.get() )
                    {
                        inBus = true;
                        break;
                    }
                }

                if( inBus )
                    break;
            }
        }

        if( inBus )
            drawComponentBox( aScreen, *comp );
    }

    // Draw bus bars
    for( const auto& bus : aData.m_buses )
        drawBusBar( aScreen, *bus );

    // Draw signal lines
    for( const auto& sig : aData.m_signals )
        drawSignalLine( aScreen, *sig );
}


void SYSTEM_DIAGRAM_GENERATOR::drawComponentBox( SCH_SCREEN* aScreen,
                                                  const SD_COMPONENT& aComp )
{
    int lineWidth = schIUScale.MilsToIU( BOX_LINE_WIDTH );

    // Draw rectangle
    SCH_SHAPE* rect = new SCH_SHAPE( SHAPE_T::RECTANGLE, LAYER_NOTES, lineWidth,
                                     FILL_T::FILLED_WITH_COLOR );
    rect->SetPosition( aComp.m_pos );
    rect->SetEnd( aComp.m_pos + aComp.m_size );
    rect->SetFillColor( COLOR4D( 0.95, 0.95, 1.0, 1.0 ) );  // Light blue fill
    rect->SetStroke( STROKE_PARAMS( lineWidth, LINE_STYLE::SOLID,
                                    COLOR4D( 0.2, 0.2, 0.4, 1.0 ) ) );
    aScreen->Append( rect );

    // Draw reference text (bold, centered at top of box)
    int refSize = schIUScale.MilsToIU( REF_TEXT_SIZE );
    VECTOR2I refPos( aComp.m_pos.x + aComp.m_size.x / 2,
                     aComp.m_pos.y + aComp.m_size.y / 3 );

    SCH_TEXT* refText = new SCH_TEXT( refPos, aComp.m_reference, LAYER_NOTES );
    refText->SetTextSize( VECTOR2I( refSize, refSize ) );
    refText->SetBold( true );
    refText->SetHorizJustify( GR_TEXT_H_ALIGN_CENTER );
    refText->SetVertJustify( GR_TEXT_V_ALIGN_CENTER );
    aScreen->Append( refText );

    // Draw value text (normal, centered at bottom of box)
    int valSize = schIUScale.MilsToIU( VALUE_TEXT_SIZE );
    VECTOR2I valPos( aComp.m_pos.x + aComp.m_size.x / 2,
                     aComp.m_pos.y + 2 * aComp.m_size.y / 3 );

    SCH_TEXT* valText = new SCH_TEXT( valPos, aComp.m_value, LAYER_NOTES );
    valText->SetTextSize( VECTOR2I( valSize, valSize ) );
    valText->SetHorizJustify( GR_TEXT_H_ALIGN_CENTER );
    valText->SetVertJustify( GR_TEXT_V_ALIGN_CENTER );
    aScreen->Append( valText );
}


void SYSTEM_DIAGRAM_GENERATOR::drawBusBar( SCH_SCREEN* aScreen, const SD_BUS& aBus )
{
    if( aBus.m_connectedComponents.size() < 2 )
        return;

    int busLineWidth = schIUScale.MilsToIU( BUS_BAR_WIDTH );
    int busBarHeight = schIUScale.MilsToIU( BUS_BAR_HEIGHT );

    // Find the extent of connected components
    int minY = INT_MAX;
    int maxY = INT_MIN;
    int leftX = INT_MAX;
    int rightX = INT_MIN;

    for( const SD_COMPONENT* comp : aBus.m_connectedComponents )
    {
        int compCenterY = comp->m_pos.y + comp->m_size.y / 2;
        minY = std::min( minY, compCenterY );
        maxY = std::max( maxY, compCenterY );
        leftX = std::min( leftX, comp->m_pos.x + comp->m_size.x );
        rightX = std::max( rightX, comp->m_pos.x );
    }

    // Place bus bar between connected components
    int barX = ( leftX + rightX ) / 2;

    // Draw the bus bar as a thick line
    SCH_LINE* busBar = new SCH_LINE( VECTOR2I( barX, minY - busBarHeight / 2 ), LAYER_NOTES );
    busBar->SetEndPoint( VECTOR2I( barX, maxY + busBarHeight / 2 ) );
    busBar->SetLineWidth( busLineWidth );
    busBar->SetLineColor( COLOR4D( 0.0, 0.4, 0.7, 1.0 ) );  // Blue
    aScreen->Append( busBar );

    // Draw bus name label
    int labelSize = schIUScale.MilsToIU( LABEL_TEXT_SIZE );
    VECTOR2I labelPos( barX + schIUScale.MilsToIU( 30 ), ( minY + maxY ) / 2 );

    SCH_TEXT* label = new SCH_TEXT( labelPos, aBus.m_aliasName, LAYER_NOTES );
    label->SetTextSize( VECTOR2I( labelSize, labelSize ) );
    label->SetBold( true );
    label->SetTextColor( COLOR4D( 0.0, 0.3, 0.6, 1.0 ) );
    aScreen->Append( label );

    // Draw connection lines from each component to the bus bar
    int connLineWidth = schIUScale.MilsToIU( SIGNAL_LINE_WIDTH );

    for( const SD_COMPONENT* comp : aBus.m_connectedComponents )
    {
        int compCenterY = comp->m_pos.y + comp->m_size.y / 2;

        // Determine which side of the component faces the bus
        int compEdgeX;

        if( comp->m_pos.x + comp->m_size.x / 2 < barX )
            compEdgeX = comp->m_pos.x + comp->m_size.x;  // Right edge
        else
            compEdgeX = comp->m_pos.x;  // Left edge

        SCH_LINE* conn = new SCH_LINE( VECTOR2I( compEdgeX, compCenterY ), LAYER_NOTES );
        conn->SetEndPoint( VECTOR2I( barX, compCenterY ) );
        conn->SetLineWidth( connLineWidth );
        conn->SetLineColor( COLOR4D( 0.0, 0.4, 0.7, 1.0 ) );
        aScreen->Append( conn );
    }
}


void SYSTEM_DIAGRAM_GENERATOR::drawSignalLine( SCH_SCREEN* aScreen, const SD_SIGNAL& aSig )
{
    if( aSig.m_connectedComponents.size() < 2 )
        return;

    int lineWidth = schIUScale.MilsToIU( SIGNAL_LINE_WIDTH );
    int labelSize = schIUScale.MilsToIU( LABEL_TEXT_SIZE );

    // Draw lines between connected components (star topology from first component)
    const SD_COMPONENT* first = aSig.m_connectedComponents[0];

    for( size_t i = 1; i < aSig.m_connectedComponents.size(); i++ )
    {
        const SD_COMPONENT* other = aSig.m_connectedComponents[i];

        int firstCenterY = first->m_pos.y + first->m_size.y / 2;
        int otherCenterY = other->m_pos.y + other->m_size.y / 2;
        int firstEdgeX = first->m_pos.x + first->m_size.x;
        int otherEdgeX = other->m_pos.x;

        SCH_LINE* line = new SCH_LINE( VECTOR2I( firstEdgeX, firstCenterY ), LAYER_NOTES );
        line->SetEndPoint( VECTOR2I( otherEdgeX, otherCenterY ) );
        line->SetLineWidth( lineWidth );
        line->SetLineStyle( LINE_STYLE::DASH );
        line->SetLineColor( COLOR4D( 0.6, 0.3, 0.0, 1.0 ) );  // Brown/orange
        aScreen->Append( line );

        // Label at midpoint
        VECTOR2I midPoint( ( firstEdgeX + otherEdgeX ) / 2,
                           ( firstCenterY + otherCenterY ) / 2
                                   - schIUScale.MilsToIU( 30 ) );

        SCH_TEXT* label = new SCH_TEXT( midPoint, aSig.m_netName, LAYER_NOTES );
        label->SetTextSize( VECTOR2I( labelSize, labelSize ) );
        label->SetItalic( true );
        label->SetHorizJustify( GR_TEXT_H_ALIGN_CENTER );
        label->SetVertJustify( GR_TEXT_V_ALIGN_BOTTOM );
        label->SetTextColor( COLOR4D( 0.6, 0.3, 0.0, 1.0 ) );
        aScreen->Append( label );
    }
}


void SYSTEM_DIAGRAM_GENERATOR::drawPowerSection( SCH_SCREEN* aScreen,
                                                   const SYSTEM_DIAGRAM_DATA& aData,
                                                   VECTOR2I aOrigin )
{
    for( const auto& root : aData.m_powerRoots )
    {
        // Draw all nodes and edges recursively
        std::function<void( const SD_POWER_NODE* )> drawTree;
        drawTree = [&]( const SD_POWER_NODE* node )
        {
            drawPowerNode( aScreen, *node );

            for( const SD_POWER_NODE* child : node->m_children )
            {
                drawPowerEdge( aScreen, *node, *child );
                drawTree( child );
            }
        };

        drawTree( root.get() );
    }
}


void SYSTEM_DIAGRAM_GENERATOR::drawPowerNode( SCH_SCREEN* aScreen, const SD_POWER_NODE& aNode )
{
    int lineWidth = schIUScale.MilsToIU( BOX_LINE_WIDTH );

    // Choose color based on type
    COLOR4D fillColor;
    COLOR4D borderColor;

    switch( aNode.m_type )
    {
    case SD_POWER_NODE::SOURCE:
        fillColor = COLOR4D( 1.0, 0.95, 0.9, 1.0 );    // Light orange
        borderColor = COLOR4D( 0.7, 0.4, 0.0, 1.0 );    // Orange border
        break;
    case SD_POWER_NODE::REGULATOR:
        fillColor = COLOR4D( 0.9, 1.0, 0.9, 1.0 );      // Light green
        borderColor = COLOR4D( 0.0, 0.5, 0.0, 1.0 );    // Green border
        break;
    case SD_POWER_NODE::RAIL:
        fillColor = COLOR4D( 0.95, 0.95, 1.0, 1.0 );    // Light blue
        borderColor = COLOR4D( 0.2, 0.2, 0.6, 1.0 );    // Blue border
        break;
    }

    // Draw rectangle
    SCH_SHAPE* rect = new SCH_SHAPE( SHAPE_T::RECTANGLE, LAYER_NOTES, lineWidth,
                                     FILL_T::FILLED_WITH_COLOR );
    rect->SetPosition( aNode.m_pos );
    rect->SetEnd( aNode.m_pos + aNode.m_size );
    rect->SetFillColor( fillColor );
    rect->SetStroke( STROKE_PARAMS( lineWidth, LINE_STYLE::SOLID, borderColor ) );
    aScreen->Append( rect );

    // Draw text content
    int refSize = schIUScale.MilsToIU( REF_TEXT_SIZE );
    int valSize = schIUScale.MilsToIU( VALUE_TEXT_SIZE );
    int labelSize = schIUScale.MilsToIU( LABEL_TEXT_SIZE );

    int centerX = aNode.m_pos.x + aNode.m_size.x / 2;
    int currentY = aNode.m_pos.y + schIUScale.MilsToIU( 40 );
    int lineSpacing = schIUScale.MilsToIU( 60 );

    // Reference + Value
    if( !aNode.m_reference.IsEmpty() )
    {
        SCH_TEXT* refText = new SCH_TEXT(
                VECTOR2I( centerX, currentY ), aNode.m_reference, LAYER_NOTES );
        refText->SetTextSize( VECTOR2I( refSize, refSize ) );
        refText->SetBold( true );
        refText->SetHorizJustify( GR_TEXT_H_ALIGN_CENTER );
        aScreen->Append( refText );
        currentY += lineSpacing;

        SCH_TEXT* valText = new SCH_TEXT(
                VECTOR2I( centerX, currentY ), aNode.m_value, LAYER_NOTES );
        valText->SetTextSize( VECTOR2I( valSize, valSize ) );
        valText->SetHorizJustify( GR_TEXT_H_ALIGN_CENTER );
        aScreen->Append( valText );
        currentY += lineSpacing;
    }
    else
    {
        // Rail node: just show net name
        SCH_TEXT* netText = new SCH_TEXT(
                VECTOR2I( centerX, currentY ), aNode.m_netName, LAYER_NOTES );
        netText->SetTextSize( VECTOR2I( refSize, refSize ) );
        netText->SetBold( true );
        netText->SetHorizJustify( GR_TEXT_H_ALIGN_CENTER );
        aScreen->Append( netText );
        currentY += lineSpacing;
    }

    // Voltage
    if( aNode.m_voltage != 0.0 )
    {
        wxString voltStr = wxString::Format( wxT( "%.1fV" ), aNode.m_voltage );

        SCH_TEXT* voltText = new SCH_TEXT(
                VECTOR2I( centerX, currentY ), voltStr, LAYER_NOTES );
        voltText->SetTextSize( VECTOR2I( labelSize, labelSize ) );
        voltText->SetHorizJustify( GR_TEXT_H_ALIGN_CENTER );
        voltText->SetTextColor( COLOR4D( 0.0, 0.0, 0.8, 1.0 ) );
        aScreen->Append( voltText );
        currentY += lineSpacing;
    }

    // Load list
    if( !aNode.m_loadRefs.empty() )
    {
        wxString loadStr = wxT( "Loads: " );

        for( size_t i = 0; i < aNode.m_loadRefs.size() && i < 5; i++ )
        {
            if( i > 0 )
                loadStr += wxT( ", " );

            loadStr += aNode.m_loadRefs[i];
        }

        if( aNode.m_loadRefs.size() > 5 )
            loadStr += wxT( "..." );

        SCH_TEXT* loadText = new SCH_TEXT(
                VECTOR2I( centerX, currentY ), loadStr, LAYER_NOTES );
        loadText->SetTextSize( VECTOR2I( labelSize, labelSize ) );
        loadText->SetHorizJustify( GR_TEXT_H_ALIGN_CENTER );
        loadText->SetTextColor( COLOR4D( 0.4, 0.4, 0.4, 1.0 ) );
        aScreen->Append( loadText );
    }
}


void SYSTEM_DIAGRAM_GENERATOR::drawPowerEdge( SCH_SCREEN* aScreen,
                                               const SD_POWER_NODE& aParent,
                                               const SD_POWER_NODE& aChild )
{
    int lineWidth = schIUScale.MilsToIU( POWER_LINE_WIDTH );
    int arrowSize = schIUScale.MilsToIU( ARROW_SIZE );

    // Line from parent's right edge to child's left edge
    VECTOR2I startPt( aParent.m_pos.x + aParent.m_size.x,
                      aParent.m_pos.y + aParent.m_size.y / 2 );
    VECTOR2I endPt( aChild.m_pos.x,
                    aChild.m_pos.y + aChild.m_size.y / 2 );

    SCH_LINE* line = new SCH_LINE( startPt, LAYER_NOTES );
    line->SetEndPoint( endPt );
    line->SetLineWidth( lineWidth );
    line->SetLineColor( COLOR4D( 0.0, 0.5, 0.0, 1.0 ) );  // Green
    aScreen->Append( line );

    // Draw arrowhead at the end
    VECTOR2I arrowTop( endPt.x - arrowSize, endPt.y - arrowSize / 2 );
    VECTOR2I arrowBot( endPt.x - arrowSize, endPt.y + arrowSize / 2 );

    SCH_LINE* arrowLine1 = new SCH_LINE( endPt, LAYER_NOTES );
    arrowLine1->SetEndPoint( arrowTop );
    arrowLine1->SetLineWidth( lineWidth );
    arrowLine1->SetLineColor( COLOR4D( 0.0, 0.5, 0.0, 1.0 ) );
    aScreen->Append( arrowLine1 );

    SCH_LINE* arrowLine2 = new SCH_LINE( endPt, LAYER_NOTES );
    arrowLine2->SetEndPoint( arrowBot );
    arrowLine2->SetLineWidth( lineWidth );
    arrowLine2->SetLineColor( COLOR4D( 0.0, 0.5, 0.0, 1.0 ) );
    aScreen->Append( arrowLine2 );
}


void SYSTEM_DIAGRAM_GENERATOR::drawSectionHeader( SCH_SCREEN* aScreen, const wxString& aTitle,
                                                    VECTOR2I aPos )
{
    int headerSize = schIUScale.MilsToIU( SECTION_HEADER_SIZE );
    int lineWidth = schIUScale.MilsToIU( HEADER_LINE_WIDTH );

    // Draw title text
    SCH_TEXT* title = new SCH_TEXT( aPos, aTitle, LAYER_NOTES );
    title->SetTextSize( VECTOR2I( headerSize, headerSize ) );
    title->SetBold( true );
    title->SetTextColor( COLOR4D( 0.1, 0.1, 0.3, 1.0 ) );
    aScreen->Append( title );

    // Draw underline
    int textWidth = aTitle.Length() * headerSize * 6 / 10;  // Rough estimate
    VECTOR2I lineStart( aPos.x, aPos.y + headerSize + schIUScale.MilsToIU( 20 ) );
    VECTOR2I lineEnd( aPos.x + textWidth + schIUScale.MilsToIU( 200 ),
                      lineStart.y );

    SCH_LINE* underline = new SCH_LINE( lineStart, LAYER_NOTES );
    underline->SetEndPoint( lineEnd );
    underline->SetLineWidth( lineWidth );
    underline->SetLineColor( COLOR4D( 0.1, 0.1, 0.3, 1.0 ) );
    aScreen->Append( underline );
}


void SYSTEM_DIAGRAM_GENERATOR::drawWarningLabel( SCH_SCREEN* aScreen, VECTOR2I aPos )
{
    int warnSize = schIUScale.MilsToIU( WARNING_TEXT_SIZE );

    SCH_TEXT* warning = new SCH_TEXT(
            aPos,
            wxT( "Auto-generated diagram - edits will be overwritten on next generation" ),
            LAYER_NOTES );
    warning->SetTextSize( VECTOR2I( warnSize, warnSize ) );
    warning->SetItalic( true );
    warning->SetTextColor( COLOR4D( 0.6, 0.0, 0.0, 1.0 ) );  // Red
    aScreen->Append( warning );
}
