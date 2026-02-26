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

#ifndef SYSTEM_DIAGRAM_GENERATOR_H
#define SYSTEM_DIAGRAM_GENERATOR_H

#include <math/vector2d.h>

class SCHEMATIC;
class SCH_SHEET;
class SCH_SCREEN;
struct SD_COMPONENT;
struct SD_BUS;
struct SD_SIGNAL;
struct SD_POWER_NODE;
struct SYSTEM_DIAGRAM_DATA;


/**
 * Result of system diagram generation: two separate sheets.
 */
struct SYSTEM_DIAGRAM_SHEETS
{
    SCH_SHEET* m_connectionsSheet = nullptr;   ///< Bus/signal connectivity diagram
    SCH_SHEET* m_powerSheet       = nullptr;   ///< Power distribution diagram
};


/**
 * Creates drawing items on sub-sheets to render the system diagram.
 * Takes the laid-out data from the analyzer + layout engine and emits
 * native schematic drawing items (SCH_SHAPE, SCH_TEXT, SCH_LINE).
 *
 * Produces two separate sheets:
 *   - "System Connections" showing bus connectivity and marked signals
 *   - "Power Distribution" showing the power tree topology
 */
class SYSTEM_DIAGRAM_GENERATOR
{
public:
    SYSTEM_DIAGRAM_GENERATOR( SCHEMATIC* aSchematic );

    /**
     * Main entry point: create or update both diagram sheets.
     */
    SYSTEM_DIAGRAM_SHEETS Generate( SYSTEM_DIAGRAM_DATA& aData );

private:
    /// Generate the system connections (bus/signal) sheet
    SCH_SHEET* generateConnectionsSheet( SYSTEM_DIAGRAM_DATA& aData );

    /// Generate the power distribution sheet
    SCH_SHEET* generatePowerSheet( SYSTEM_DIAGRAM_DATA& aData );

    /// Find existing or create a named sub-sheet
    SCH_SHEET* getOrCreateSheet( const wxString& aName, const wxString& aFileName );

    /// Clear all items from the sheet's screen
    void clearSheet( SCH_SCREEN* aScreen );

    /// Set appropriate page size based on content bounds
    void setPageSize( SCH_SCREEN* aScreen, const VECTOR2I& aContentSize );

    /// Calculate content extent from component positions
    VECTOR2I calcBusExtent( const SYSTEM_DIAGRAM_DATA& aData );

    /// Calculate content extent from power tree
    VECTOR2I calcPowerExtent( const SYSTEM_DIAGRAM_DATA& aData );

    /// Bus section drawing
    void drawBusSection( SCH_SCREEN* aScreen, const SYSTEM_DIAGRAM_DATA& aData );
    void drawComponentBox( SCH_SCREEN* aScreen, const SD_COMPONENT& aComp );
    void drawBusBar( SCH_SCREEN* aScreen, const SD_BUS& aBus );
    void drawSignalLine( SCH_SCREEN* aScreen, const SD_SIGNAL& aSig );

    /// Power section drawing
    void drawPowerSection( SCH_SCREEN* aScreen, const SYSTEM_DIAGRAM_DATA& aData );
    void drawPowerNode( SCH_SCREEN* aScreen, const SD_POWER_NODE& aNode );
    void drawPowerEdge( SCH_SCREEN* aScreen, const SD_POWER_NODE& aParent,
                        const SD_POWER_NODE& aChild );
    void drawBiasEdge( SCH_SCREEN* aScreen, const SD_POWER_NODE& aBiasSource,
                       const SD_POWER_NODE& aConsumer,
                       const SD_POWER_NODE::BIAS_CONNECTION& aBias );

    /// Section headers and labels
    void drawSectionHeader( SCH_SCREEN* aScreen, const wxString& aTitle, VECTOR2I aPos );
    void drawWarningLabel( SCH_SCREEN* aScreen, VECTOR2I aPos );

    SCHEMATIC* m_schematic;
};


#endif // SYSTEM_DIAGRAM_GENERATOR_H
