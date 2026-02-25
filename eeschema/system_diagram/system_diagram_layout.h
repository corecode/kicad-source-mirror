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

#ifndef SYSTEM_DIAGRAM_LAYOUT_H
#define SYSTEM_DIAGRAM_LAYOUT_H

#include <math/box2.h>
#include <math/vector2d.h>
#include <map>
#include <vector>

struct SD_COMPONENT;
struct SD_BUS;
struct SD_POWER_NODE;
struct SYSTEM_DIAGRAM_DATA;


/**
 * Layout engine for the system diagram. Assigns positions and sizes to all nodes
 * using a simplified Sugiyama-style layered layout for buses and a recursive tree
 * layout for the power topology.
 */
class SYSTEM_DIAGRAM_LAYOUT
{
public:
    /**
     * Lay out the bus connectivity section. Returns bounding box.
     */
    BOX2I LayoutBusSection( SYSTEM_DIAGRAM_DATA& aData, VECTOR2I aOrigin );

    /**
     * Lay out the power topology section. Returns bounding box.
     */
    BOX2I LayoutPowerSection( SYSTEM_DIAGRAM_DATA& aData, VECTOR2I aOrigin );

private:
    /// Compute the size of a component box based on its text content
    VECTOR2I computeBoxSize( const wxString& aReference, const wxString& aValue );

    /// Compute the size of a power node box
    VECTOR2I computePowerNodeSize( const SD_POWER_NODE& aNode );

    /// Simplified Sugiyama: assign layers via BFS from hub
    void assignLayers( const std::vector<SD_COMPONENT*>& aNodes,
                       const std::vector<std::pair<SD_COMPONENT*, SD_COMPONENT*>>& aEdges,
                       std::map<SD_COMPONENT*, int>& aLayerMap );

    /// Minimize edge crossings using barycenter heuristic
    void minimizeCrossings( std::vector<std::vector<SD_COMPONENT*>>& aLayers,
                            const std::vector<std::pair<SD_COMPONENT*, SD_COMPONENT*>>& aEdges );

    /// Recursive tree layout for power section
    int layoutSubtree( SD_POWER_NODE* aNode, int aX, int aY );
};


#endif // SYSTEM_DIAGRAM_LAYOUT_H
