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

#include "system_diagram_layout.h"
#include "system_diagram_analyzer.h"

#include <base_units.h>
#include <algorithm>
#include <queue>
#include <set>
#include <map>


// Layout constants (in mils, converted to IU)
static const int BOX_PADDING     = 100;  // mils of padding inside boxes
static const int TEXT_HEIGHT      = 50;   // mils per text line
static const int MIN_BOX_WIDTH   = 600;  // mils minimum box width
static const int MIN_BOX_HEIGHT  = 200;  // mils minimum box height
static const int X_SPACING       = 800;  // mils between layers (horizontal)
static const int Y_SPACING       = 300;  // mils between nodes (vertical)
static const int POWER_X_SPACING = 700;  // mils between power tree levels
static const int POWER_Y_SPACING = 250;  // mils between power tree siblings


VECTOR2I SYSTEM_DIAGRAM_LAYOUT::computeBoxSize( const wxString& aReference,
                                                 const wxString& aValue )
{
    // Estimate box size from text content
    // Each character is roughly TEXT_HEIGHT * 0.6 wide
    int charWidth = TEXT_HEIGHT * 6 / 10;

    int refWidth = aReference.Length() * charWidth;
    int valWidth = aValue.Length() * charWidth;
    int textWidth = std::max( refWidth, valWidth );

    int width = std::max( textWidth + 2 * BOX_PADDING, MIN_BOX_WIDTH );
    int height = std::max( 2 * TEXT_HEIGHT + 2 * BOX_PADDING, MIN_BOX_HEIGHT );

    return VECTOR2I( schIUScale.MilsToIU( width ), schIUScale.MilsToIU( height ) );
}


VECTOR2I SYSTEM_DIAGRAM_LAYOUT::computePowerNodeSize( const SD_POWER_NODE& aNode )
{
    int charWidth = TEXT_HEIGHT * 6 / 10;

    int line1Width = 0;
    int line2Width = 0;
    int lines = 1;

    if( !aNode.m_reference.IsEmpty() )
    {
        line1Width = aNode.m_reference.Length() * charWidth;
        line2Width = aNode.m_value.Length() * charWidth;
        lines = 2;
    }
    else
    {
        line1Width = aNode.m_netName.Length() * charWidth;
    }

    // Add voltage text if available
    if( aNode.m_voltage != 0.0 )
    {
        lines++;
    }

    // Add load refs
    if( !aNode.m_loadRefs.empty() )
    {
        lines++;
        wxString loadStr;

        for( size_t i = 0; i < aNode.m_loadRefs.size() && i < 5; i++ )
        {
            if( i > 0 )
                loadStr += wxT( ", " );

            loadStr += aNode.m_loadRefs[i];
        }

        if( aNode.m_loadRefs.size() > 5 )
            loadStr += wxT( "..." );

        int loadWidth = loadStr.Length() * charWidth;
        line2Width = std::max( line2Width, loadWidth );
    }

    int textWidth = std::max( line1Width, line2Width );
    int width = std::max( textWidth + 2 * BOX_PADDING, MIN_BOX_WIDTH );
    int height = std::max( lines * TEXT_HEIGHT + 2 * BOX_PADDING, MIN_BOX_HEIGHT );

    return VECTOR2I( schIUScale.MilsToIU( width ), schIUScale.MilsToIU( height ) );
}


void SYSTEM_DIAGRAM_LAYOUT::assignLayers(
        const std::vector<SD_COMPONENT*>&                          aNodes,
        const std::vector<std::pair<SD_COMPONENT*, SD_COMPONENT*>>& aEdges,
        std::map<SD_COMPONENT*, int>&                              aLayerMap )
{
    if( aNodes.empty() )
        return;

    // Build adjacency list
    std::map<SD_COMPONENT*, std::set<SD_COMPONENT*>> adj;

    for( SD_COMPONENT* node : aNodes )
        adj[node] = {};

    for( const auto& [a, b] : aEdges )
    {
        adj[a].insert( b );
        adj[b].insert( a );
    }

    // Find the hub: component with the most connections
    SD_COMPONENT* hub = aNodes[0];
    size_t        maxConn = 0;

    for( SD_COMPONENT* node : aNodes )
    {
        if( adj[node].size() > maxConn )
        {
            maxConn = adj[node].size();
            hub = node;
        }
    }

    // BFS from hub to assign layers
    std::queue<SD_COMPONENT*> queue;
    queue.push( hub );
    aLayerMap[hub] = 0;

    while( !queue.empty() )
    {
        SD_COMPONENT* current = queue.front();
        queue.pop();

        int currentLayer = aLayerMap[current];

        for( SD_COMPONENT* neighbor : adj[current] )
        {
            if( aLayerMap.find( neighbor ) == aLayerMap.end() )
            {
                aLayerMap[neighbor] = std::min( currentLayer + 1, 2 );  // Cap at layer 2
                queue.push( neighbor );
            }
        }
    }

    // Assign any unvisited nodes to layer 2
    for( SD_COMPONENT* node : aNodes )
    {
        if( aLayerMap.find( node ) == aLayerMap.end() )
            aLayerMap[node] = 2;
    }
}


void SYSTEM_DIAGRAM_LAYOUT::minimizeCrossings(
        std::vector<std::vector<SD_COMPONENT*>>&                    aLayers,
        const std::vector<std::pair<SD_COMPONENT*, SD_COMPONENT*>>& aEdges )
{
    // Build position lookup
    auto getYIndex = [&]( SD_COMPONENT* node, int layer ) -> double
    {
        const auto& layerVec = aLayers[layer];

        for( size_t i = 0; i < layerVec.size(); i++ )
        {
            if( layerVec[i] == node )
                return (double) i;
        }

        return 0.0;
    };

    // Build neighbor map
    std::map<SD_COMPONENT*, std::set<SD_COMPONENT*>> adj;

    for( const auto& [a, b] : aEdges )
    {
        adj[a].insert( b );
        adj[b].insert( a );
    }

    // Barycenter heuristic: 4 iterations
    for( int iter = 0; iter < 4; iter++ )
    {
        // Left to right
        for( size_t layer = 1; layer < aLayers.size(); layer++ )
        {
            std::map<SD_COMPONENT*, double> barycenters;

            for( SD_COMPONENT* node : aLayers[layer] )
            {
                double sum = 0;
                int    count = 0;

                for( SD_COMPONENT* neighbor : adj[node] )
                {
                    // Find neighbor in previous layer
                    for( size_t i = 0; i < aLayers[layer - 1].size(); i++ )
                    {
                        if( aLayers[layer - 1][i] == neighbor )
                        {
                            sum += (double) i;
                            count++;
                        }
                    }
                }

                barycenters[node] = count > 0 ? sum / count : getYIndex( node, layer );
            }

            std::sort( aLayers[layer].begin(), aLayers[layer].end(),
                       [&]( SD_COMPONENT* a, SD_COMPONENT* b )
                       {
                           return barycenters[a] < barycenters[b];
                       } );
        }

        // Right to left
        for( int layer = (int) aLayers.size() - 2; layer >= 0; layer-- )
        {
            std::map<SD_COMPONENT*, double> barycenters;

            for( SD_COMPONENT* node : aLayers[layer] )
            {
                double sum = 0;
                int    count = 0;

                for( SD_COMPONENT* neighbor : adj[node] )
                {
                    for( size_t i = 0; i < aLayers[layer + 1].size(); i++ )
                    {
                        if( aLayers[layer + 1][i] == neighbor )
                        {
                            sum += (double) i;
                            count++;
                        }
                    }
                }

                barycenters[node] = count > 0 ? sum / count : getYIndex( node, layer );
            }

            std::sort( aLayers[layer].begin(), aLayers[layer].end(),
                       [&]( SD_COMPONENT* a, SD_COMPONENT* b )
                       {
                           return barycenters[a] < barycenters[b];
                       } );
        }
    }
}


BOX2I SYSTEM_DIAGRAM_LAYOUT::LayoutBusSection( SYSTEM_DIAGRAM_DATA& aData, VECTOR2I aOrigin )
{
    BOX2I bounds;
    bounds.SetOrigin( aOrigin );

    if( aData.m_buses.empty() && aData.m_signals.empty() )
    {
        bounds.SetSize( VECTOR2I( 0, 0 ) );
        return bounds;
    }

    // Collect all components that participate in bus/signal connections
    std::set<SD_COMPONENT*> participatingComps;

    for( const auto& bus : aData.m_buses )
    {
        for( SD_COMPONENT* comp : bus->m_connectedComponents )
            participatingComps.insert( comp );
    }

    for( const auto& sig : aData.m_signals )
    {
        for( SD_COMPONENT* comp : sig->m_connectedComponents )
            participatingComps.insert( comp );
    }

    std::vector<SD_COMPONENT*> nodes( participatingComps.begin(), participatingComps.end() );

    if( nodes.empty() )
    {
        bounds.SetSize( VECTOR2I( 0, 0 ) );
        return bounds;
    }

    // Build edge list from buses and signals
    std::vector<std::pair<SD_COMPONENT*, SD_COMPONENT*>> edges;

    for( const auto& bus : aData.m_buses )
    {
        for( size_t i = 0; i < bus->m_connectedComponents.size(); i++ )
        {
            for( size_t j = i + 1; j < bus->m_connectedComponents.size(); j++ )
            {
                edges.push_back( { bus->m_connectedComponents[i],
                                   bus->m_connectedComponents[j] } );
            }
        }
    }

    for( const auto& sig : aData.m_signals )
    {
        for( size_t i = 0; i < sig->m_connectedComponents.size(); i++ )
        {
            for( size_t j = i + 1; j < sig->m_connectedComponents.size(); j++ )
            {
                edges.push_back( { sig->m_connectedComponents[i],
                                   sig->m_connectedComponents[j] } );
            }
        }
    }

    // Assign layers
    std::map<SD_COMPONENT*, int> layerMap;
    assignLayers( nodes, edges, layerMap );

    // Group into layers
    int maxLayer = 0;

    for( const auto& [node, layer] : layerMap )
        maxLayer = std::max( maxLayer, layer );

    std::vector<std::vector<SD_COMPONENT*>> layers( maxLayer + 1 );

    for( const auto& [node, layer] : layerMap )
        layers[layer].push_back( node );

    // Minimize crossings
    minimizeCrossings( layers, edges );

    // Compute box sizes
    for( SD_COMPONENT* comp : nodes )
        comp->m_size = computeBoxSize( comp->m_reference, comp->m_value );

    // Assign coordinates
    int xSpacing = schIUScale.MilsToIU( X_SPACING );
    int ySpacing = schIUScale.MilsToIU( Y_SPACING );

    int maxX = 0;
    int maxY = 0;

    for( int layer = 0; layer <= maxLayer; layer++ )
    {
        int x = aOrigin.x + layer * xSpacing;
        int y = aOrigin.y;

        for( SD_COMPONENT* comp : layers[layer] )
        {
            comp->m_pos = VECTOR2I( x, y );
            y += comp->m_size.y + ySpacing;
            maxX = std::max( maxX, x + comp->m_size.x );
        }

        maxY = std::max( maxY, y );
    }

    bounds.SetEnd( VECTOR2I( maxX, maxY ) );
    return bounds;
}


BOX2I SYSTEM_DIAGRAM_LAYOUT::LayoutPowerSection( SYSTEM_DIAGRAM_DATA& aData, VECTOR2I aOrigin )
{
    BOX2I bounds;
    bounds.SetOrigin( aOrigin );

    if( aData.m_powerRoots.empty() )
    {
        bounds.SetSize( VECTOR2I( 0, 0 ) );
        return bounds;
    }

    int currentY = aOrigin.y;
    int maxX = aOrigin.x;
    int ySpacing = schIUScale.MilsToIU( POWER_Y_SPACING );

    for( const auto& root : aData.m_powerRoots )
    {
        int subtreeHeight = layoutSubtree( root, aOrigin.x, currentY );
        currentY += subtreeHeight + ySpacing;

        // Find max X extent in the tree
        std::function<void( SD_POWER_NODE* )> findMaxX;
        findMaxX = [&]( SD_POWER_NODE* node )
        {
            maxX = std::max( maxX, node->m_pos.x + node->m_size.x );

            for( SD_POWER_NODE* child : node->m_children )
                findMaxX( child );
        };

        findMaxX( root );
    }

    bounds.SetEnd( VECTOR2I( maxX, currentY ) );
    return bounds;
}


int SYSTEM_DIAGRAM_LAYOUT::layoutSubtree( SD_POWER_NODE* aNode, int aX, int aY )
{
    aNode->m_size = computePowerNodeSize( *aNode );

    int xSpacing = schIUScale.MilsToIU( POWER_X_SPACING );
    int ySpacing = schIUScale.MilsToIU( POWER_Y_SPACING );

    if( aNode->m_children.empty() )
    {
        aNode->m_pos = VECTOR2I( aX, aY );
        return aNode->m_size.y;
    }

    // Lay out children first
    int childX = aX + aNode->m_size.x + xSpacing;
    int childY = aY;
    int totalChildHeight = 0;

    for( size_t i = 0; i < aNode->m_children.size(); i++ )
    {
        int childHeight = layoutSubtree( aNode->m_children[i], childX, childY );
        childY += childHeight + ySpacing;
        totalChildHeight = childY - aY - ySpacing;
    }

    // Center parent vertically on its children
    int parentY = aY;

    if( aNode->m_children.size() > 1 )
    {
        int firstChildCenter = aNode->m_children.front()->m_pos.y
                               + aNode->m_children.front()->m_size.y / 2;
        int lastChildCenter = aNode->m_children.back()->m_pos.y
                              + aNode->m_children.back()->m_size.y / 2;
        parentY = ( firstChildCenter + lastChildCenter ) / 2 - aNode->m_size.y / 2;
    }

    aNode->m_pos = VECTOR2I( aX, parentY );

    return std::max( aNode->m_size.y, totalChildHeight );
}
