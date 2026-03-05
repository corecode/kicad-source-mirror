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

#include "fasthenry_exporter.h"

#include <board.h>
#include <board_design_settings.h>
#include <board_stackup_manager/board_stackup.h>
#include <footprint.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_track.h>
#include <zone.h>
#include <layer_ids.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>


/// Conductivity of copper in S/m
static constexpr double COPPER_CONDUCTIVITY = 5.8e7;

/// FastHenry sigma units are 1/(length_unit * Ohm).  With .units mm:
/// sigma_fh = sigma_SI * 1e-3  (since 1 S/m = 1e-3 S/mm)
static constexpr double SIGMA_SCALE_MM = 1e-3;


FASTHENRY_EXPORTER::FASTHENRY_EXPORTER( BOARD* aBoard,
                                        const PDN_PARASITIC::EXTRACTION_CONFIG& aConfig ) :
    m_board( aBoard ),
    m_config( aConfig )
{
}


bool FASTHENRY_EXPORTER::Export( const std::string& aOutputPath )
{
    if( !m_board )
        return false;

    // Build layer geometry from stackup
    buildLayerZMap();

    // Resolve which nets to extract
    m_netCodes.clear();

    if( !m_config.m_PortSpecs.empty() )
    {
        // Derive nets from explicit pad selection
        for( const PDN_PARASITIC::PORT_SPEC& spec : m_config.m_PortSpecs )
            m_netCodes.insert( spec.m_NetCode );
    }
    else if( m_config.m_NetNames.empty() )
    {
        // Default: extract all nets (caller should filter to power/ground)
        for( const PCB_TRACK* track : m_board->Tracks() )
            m_netCodes.insert( track->GetNetCode() );
    }
    else
    {
        for( const std::string& name : m_config.m_NetNames )
        {
            NETINFO_ITEM* net = m_board->FindNet( wxString::FromUTF8( name ) );

            if( net )
                m_netCodes.insert( net->GetNetCode() );
        }
    }

    if( m_netCodes.empty() )
        return false;

    // Pre-register node names for all traces and vias (populates m_nodeMap)
    // so that identifyPorts() can find nearest conductor nodes.
    registerNodes();

    // Identify port locations from pads
    identifyPorts();

    // Create return paths for power vias that cross ground plane layers.
    // Must run after identifyPorts() so we can exclude port terminal nodes.
    identifyReturnPaths();

    std::ofstream file( aOutputPath );

    if( !file.is_open() )
        return false;

    file << std::fixed << std::setprecision( 6 );

    writeHeader( file );
    writeDefaults( file );
    writePlanes( file );
    writeTraces( file );
    writeVias( file );
    writePadNodes( file );
    writeEquivNodes( file );
    writeExternals( file );
    writeFrequency( file );

    file << ".end" << std::endl;
    file.close();

    return true;
}


void FASTHENRY_EXPORTER::buildLayerZMap()
{
    const BOARD_DESIGN_SETTINGS& bds = m_board->GetDesignSettings();
    const BOARD_STACKUP& stackup = bds.GetStackupDescriptor();

    m_layerZMM.clear();
    m_layerThickMM.clear();

    // Walk the stackup from top to bottom, accumulating Z position
    double zMM = 0.0;

    for( const BOARD_STACKUP_ITEM* item : stackup.GetList() )
    {
        if( !item->IsEnabled() )
            continue;

        if( item->GetType() == BS_ITEM_TYPE_COPPER )
        {
            double thickMM = iu2mm( item->GetThickness() );

            if( thickMM <= 0.0 )
                thickMM = 0.035;  // Default 1oz copper = 35um

            int layerId = item->GetBrdLayerId();
            m_layerZMM[layerId] = zMM + thickMM / 2.0;   // Center of copper
            m_layerThickMM[layerId] = thickMM;

            zMM += thickMM;
        }
        else if( item->GetType() == BS_ITEM_TYPE_DIELECTRIC )
        {
            for( int sub = 0; sub < item->GetSublayersCount(); sub++ )
            {
                double thickMM = iu2mm( item->GetThickness( sub ) );
                zMM += thickMM;
            }
        }
        else if( item->GetType() == BS_ITEM_TYPE_SOLDERMASK )
        {
            // Skip solder mask for FastHenry (non-conductive, minimal effect on L/R)
        }
    }

    // If stackup is empty or incomplete, fall back to a simple 2-layer model
    if( m_layerZMM.empty() )
    {
        double boardThickMM = iu2mm( bds.GetBoardThickness() );
        double copperMM = 0.035;

        m_layerZMM[F_Cu] = copperMM / 2.0;
        m_layerThickMM[F_Cu] = copperMM;

        m_layerZMM[B_Cu] = boardThickMM - copperMM / 2.0;
        m_layerThickMM[B_Cu] = copperMM;
    }
}


double FASTHENRY_EXPORTER::getLayerZMM( int aLayerId ) const
{
    auto it = m_layerZMM.find( aLayerId );
    return ( it != m_layerZMM.end() ) ? it->second : 0.0;
}


double FASTHENRY_EXPORTER::getCopperThicknessMM( int aLayerId ) const
{
    auto it = m_layerThickMM.find( aLayerId );
    return ( it != m_layerThickMM.end() ) ? it->second : 0.035;
}


bool FASTHENRY_EXPORTER::isNetIncluded( int aNetCode ) const
{
    return m_netCodes.count( aNetCode ) > 0;
}


std::string FASTHENRY_EXPORTER::makeTrackNodeName( const PCB_TRACK* aTrack, bool aStart )
{
    VECTOR2I pt = aStart ? aTrack->GetStart() : aTrack->GetEnd();
    int layer = aTrack->GetLayer();

    auto key = std::make_tuple( pt.x, pt.y, layer );
    auto it = m_nodeMap.find( key );

    if( it != m_nodeMap.end() )
        return it->second;

    std::string name = "N" + std::to_string( ++m_nodeCounter );
    m_nodeMap[key] = name;
    return name;
}


std::string FASTHENRY_EXPORTER::makeViaNodeName( const PCB_VIA* aVia, int aLayerId )
{
    VECTOR2I pt = aVia->GetStart();
    auto key = std::make_tuple( pt.x, pt.y, aLayerId );
    auto it = m_nodeMap.find( key );

    if( it != m_nodeMap.end() )
        return it->second;

    std::string name = "N" + std::to_string( ++m_nodeCounter );
    m_nodeMap[key] = name;
    return name;
}


std::string FASTHENRY_EXPORTER::makePadNodeName( const PAD* aPad, int aLayerId )
{
    VECTOR2I pt = aPad->GetPosition();
    auto key = std::make_tuple( pt.x, pt.y, aLayerId );
    auto it = m_nodeMap.find( key );

    if( it != m_nodeMap.end() )
        return it->second;

    std::string name = "N" + std::to_string( ++m_nodeCounter );
    m_nodeMap[key] = name;
    return name;
}


std::string FASTHENRY_EXPORTER::makePlaneNodeName( const ZONE* aZone,
                                                    double aXMM, double aYMM )
{
    std::ostringstream ss;
    ss << "ngp" << m_planeCounter << "_" << static_cast<int>( aXMM * 1000 ) << "_"
       << static_cast<int>( aYMM * 1000 );
    return ss.str();
}


void FASTHENRY_EXPORTER::registerNodes()
{
    // Pre-create node names for all trace endpoints and via positions.
    // This populates m_nodeMap so identifyPorts() can find nearest conductor nodes.
    // Also populates m_nodeNetCode so searches can filter by net.

    for( const PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->Type() == PCB_VIA_T )
            continue;

        if( !isNetIncluded( track->GetNetCode() ) )
            continue;

        makeTrackNodeName( track, true );
        makeTrackNodeName( track, false );

        int  netCode = track->GetNetCode();
        auto key1 = std::make_tuple( track->GetStart().x, track->GetStart().y,
                                     static_cast<int>( track->GetLayer() ) );
        auto key2 = std::make_tuple( track->GetEnd().x, track->GetEnd().y,
                                     static_cast<int>( track->GetLayer() ) );
        m_nodeNetCode.emplace( key1, netCode );
        m_nodeNetCode.emplace( key2, netCode );
    }

    for( const PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->Type() != PCB_VIA_T )
            continue;

        const PCB_VIA* via = static_cast<const PCB_VIA*>( track );

        if( !isNetIncluded( via->GetNetCode() ) )
            continue;

        PCB_LAYER_ID topLayer, botLayer;
        via->LayerPair( &topLayer, &botLayer );

        double zTop = getLayerZMM( topLayer );
        double zBot = getLayerZMM( botLayer );
        double zMin = std::min( zTop, zBot );
        double zMax = std::max( zTop, zBot );

        int netCode = via->GetNetCode();

        for( auto& [layerId, zPos] : m_layerZMM )
        {
            if( zPos >= zMin - 1e-6 && zPos <= zMax + 1e-6 )
            {
                makeViaNodeName( via, layerId );

                auto key = std::make_tuple( via->GetStart().x, via->GetStart().y, layerId );
                m_nodeNetCode.emplace( key, netCode );
            }
        }
    }
}


void FASTHENRY_EXPORTER::writeHeader( std::ofstream& aFile )
{
    aFile << "* FastHenry input generated from KiCad PCB layout" << std::endl;
    aFile << "* Board: " << m_board->GetFileName().ToStdString() << std::endl;
    aFile << "* Nets:";

    for( const std::string& name : m_config.m_NetNames )
        aFile << " " << name;

    aFile << std::endl;
    aFile << "*" << std::endl;
    aFile << ".units mm" << std::endl;
    aFile << std::endl;
}


void FASTHENRY_EXPORTER::writeDefaults( std::ofstream& aFile )
{
    // Default copper conductivity scaled for mm units
    double sigmaFH = COPPER_CONDUCTIVITY * SIGMA_SCALE_MM;

    aFile << "* Default segment parameters" << std::endl;
    aFile << ".default sigma=" << std::scientific << sigmaFH
          << " nhinc=" << m_config.m_NhInc
          << " nwinc=" << m_config.m_NwInc
          << std::endl;
    aFile << std::fixed;
    aFile << std::endl;
}


void FASTHENRY_EXPORTER::writePlanes( std::ofstream& aFile )
{
    aFile << "* ====== Copper planes (zones) ======" << std::endl;

    m_planeCounter = 0;

    for( const ZONE* zone : m_board->Zones() )
    {
        if( !isNetIncluded( zone->GetNetCode() ) )
            continue;

        if( zone->GetIsRuleArea() )
            continue;

        // Get the zone's bounding box as the plane extent
        BOX2I bbox = zone->GetBoundingBox();

        // Only model large zones as ground planes; small zones get skipped
        // (they could be modeled as trace-like segments, but that adds complexity)
        double widthMM = iu2mm( bbox.GetWidth() );
        double heightMM = iu2mm( bbox.GetHeight() );

        if( widthMM < 1.0 || heightMM < 1.0 )
            continue;

        // Determine the layer for z-position
        LSET layers = zone->GetLayerSet();

        for( PCB_LAYER_ID layerId : layers.CuStack() )
        {
            double zMM = getLayerZMM( layerId );
            double thickMM = getCopperThicknessMM( layerId );
            double sigmaFH = COPPER_CONDUCTIVITY * SIGMA_SCALE_MM;

            // FastHenry g element: point 2 must be the right-angle corner
            // between edges (p2→p1) and (p2→p3)
            double x1 = iu2mm( bbox.GetX() );
            double y1 = iu2mm( bbox.GetY() );
            double x2 = x1 + widthMM;
            double y2 = y1;
            double x3 = x2;
            double y3 = y1 + heightMM;

            m_planeCounter++;

            aFile << "* Zone: net=" << zone->GetNetname().ToStdString()
                  << " layer=" << layerId << std::endl;
            aFile << "g" << m_planeCounter
                  << " x1=" << x1 << " y1=" << y1 << " z1=" << zMM << std::endl;
            aFile << "+ x2=" << x2 << " y2=" << y2 << " z2=" << zMM << std::endl;
            aFile << "+ x3=" << x3 << " y3=" << y3 << " z3=" << zMM << std::endl;
            aFile << "+ thick=" << thickMM << std::endl;
            aFile << std::scientific;
            aFile << "+ sigma=" << sigmaFH << std::endl;
            aFile << std::fixed;
            aFile << "+ seg1=" << m_config.m_PlaneSegX
                  << " seg2=" << m_config.m_PlaneSegY << std::endl;
            aFile << "+ nhinc=" << m_config.m_NhInc << std::endl;

            // Define named nodes on the plane at each pad location that connects
            // to this zone's net on this layer
            for( const FOOTPRINT* fp : m_board->Footprints() )
            {
                for( const PAD* pad : fp->Pads() )
                {
                    if( pad->GetNetCode() != zone->GetNetCode() )
                        continue;

                    if( !pad->IsOnLayer( static_cast<PCB_LAYER_ID>( layerId ) ) )
                        continue;

                    VECTOR2I pos = pad->GetPosition();
                    double px = iu2mm( pos.x );
                    double py = iu2mm( pos.y );

                    // Check if pad is within zone bounding box
                    if( px >= x1 && px <= x1 + widthMM &&
                        py >= y1 && py <= y1 + heightMM )
                    {
                        std::string nodeName = makePlaneNodeName( zone, px, py );
                        aFile << "+ " << nodeName
                              << " (" << px << "," << py << "," << zMM << ")"
                              << std::endl;

                        // Record equivalence with the pad node
                        std::string padNode = makePadNodeName( pad, layerId );
                        m_equivPairs.push_back( { nodeName, padNode } );
                    }
                }
            }

            // Add port reference nodes on this ground plane
            // (identified during identifyPorts for .external definitions)
            for( const auto& pn : m_groundPlanePortNodes )
            {
                double px = pn.m_XMM;
                double py = pn.m_YMM;

                if( px >= x1 && px <= x1 + widthMM && py >= y1 && py <= y1 + heightMM )
                {
                    aFile << "+ " << pn.m_Name << " (" << px << "," << py << "," << zMM << ")"
                          << std::endl;
                }
            }

            aFile << std::endl;
        }
    }

    aFile << std::endl;
}


void FASTHENRY_EXPORTER::writeTraces( std::ofstream& aFile )
{
    aFile << "* ====== Trace segments ======" << std::endl;

    for( const PCB_TRACK* track : m_board->Tracks() )
    {
        // Skip vias -- handled separately
        if( track->Type() == PCB_VIA_T )
            continue;

        if( !isNetIncluded( track->GetNetCode() ) )
            continue;

        int layerId = track->GetLayer();
        double zMM = getLayerZMM( layerId );
        double hMM = getCopperThicknessMM( layerId );
        double wMM = iu2mm( track->GetWidth() );

        if( wMM <= 0.0 )
            continue;

        VECTOR2I start = track->GetStart();
        VECTOR2I end = track->GetEnd();

        double x1 = iu2mm( start.x );
        double y1 = iu2mm( start.y );
        double x2 = iu2mm( end.x );
        double y2 = iu2mm( end.y );

        std::string nStart = makeTrackNodeName( track, true );
        std::string nEnd = makeTrackNodeName( track, false );

        // Write node definitions (only if not already defined)
        if( m_definedNodes.insert( nStart ).second )
            aFile << nStart << " x=" << x1 << " y=" << y1 << " z=" << zMM << std::endl;

        if( m_definedNodes.insert( nEnd ).second )
            aFile << nEnd << " x=" << x2 << " y=" << y2 << " z=" << zMM << std::endl;

        // Write segment
        m_segCounter++;
        aFile << "E" << m_segCounter << " " << nStart << " " << nEnd
              << " w=" << wMM << " h=" << hMM << std::endl;

        aFile << std::endl;
    }

    aFile << std::endl;
}


void FASTHENRY_EXPORTER::writeVias( std::ofstream& aFile )
{
    aFile << "* ====== Vias ======" << std::endl;

    for( const PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->Type() != PCB_VIA_T )
            continue;

        const PCB_VIA* via = static_cast<const PCB_VIA*>( track );

        if( !isNetIncluded( via->GetNetCode() ) )
            continue;

        PCB_LAYER_ID topLayer, botLayer;
        via->LayerPair( &topLayer, &botLayer );

        double drillMM = iu2mm( via->GetDrillValue() );

        if( drillMM <= 0.0 )
            continue;

        // Model the via barrel as a square cross-section with equivalent area
        // Area of annular ring: pi * ((OD/2)^2 - (ID/2)^2)
        // For simplicity, use drill diameter as the conductor cross-section width
        // (approximation: copper barrel thickness is small relative to drill)
        double viaSizeMM = drillMM;

        // Get all copper layers this via connects (between topLayer and botLayer
        // by z-position, since PCB_LAYER_ID values don't follow physical order)
        double zTop = getLayerZMM( topLayer );
        double zBot = getLayerZMM( botLayer );
        double zMin = std::min( zTop, zBot );
        double zMax = std::max( zTop, zBot );

        std::vector<int> connectedLayers;

        for( auto& [layerId, zPos] : m_layerZMM )
        {
            if( zPos >= zMin - 1e-6 && zPos <= zMax + 1e-6 )
                connectedLayers.push_back( layerId );
        }

        // Sort by Z position
        std::sort( connectedLayers.begin(), connectedLayers.end(),
                   [this]( int a, int b )
                   { return m_layerZMM.at( a ) < m_layerZMM.at( b ); } );

        // Create nodes at each layer and segments between consecutive layers
        std::string prevNodeName;

        for( size_t i = 0; i < connectedLayers.size(); i++ )
        {
            int layerId = connectedLayers[i];
            std::string nodeName = makeViaNodeName( via, layerId );

            double xMM = iu2mm( via->GetStart().x );
            double yMM = iu2mm( via->GetStart().y );
            double zMM = getLayerZMM( layerId );

            if( m_definedNodes.insert( nodeName ).second )
                aFile << nodeName << " x=" << xMM << " y=" << yMM << " z=" << zMM << std::endl;

            if( i > 0 )
            {
                m_segCounter++;
                aFile << "E" << m_segCounter << " " << prevNodeName << " " << nodeName
                      << " w=" << viaSizeMM << " h=" << viaSizeMM
                      << " nhinc=" << m_config.m_NhInc
                      << " nwinc=" << m_config.m_NwInc
                      << std::endl;
            }

            prevNodeName = nodeName;
        }

        aFile << std::endl;
    }

    aFile << std::endl;
}


void FASTHENRY_EXPORTER::writePadNodes( std::ofstream& aFile )
{
    // Emit node definitions for pad nodes that were registered during identifyPorts()
    // but not yet defined by writeTraces() or writeVias().
    // These nodes are referenced by .external and .equiv statements.

    aFile << "* ====== Pad nodes ======" << std::endl;

    for( const FOOTPRINT* fp : m_board->Footprints() )
    {
        for( const PAD* pad : fp->Pads() )
        {
            if( !isNetIncluded( pad->GetNetCode() ) )
                continue;

            VECTOR2I pos = pad->GetPosition();

            for( PCB_LAYER_ID layerId : { F_Cu, B_Cu } )
            {
                if( !pad->IsOnLayer( layerId ) )
                    continue;

                auto key = std::make_tuple( pos.x, pos.y, static_cast<int>( layerId ) );
                auto it = m_nodeMap.find( key );

                if( it == m_nodeMap.end() )
                    continue; // Node was never referenced

                const std::string& nodeName = it->second;

                if( m_definedNodes.insert( nodeName ).second )
                {
                    double xMM = iu2mm( pos.x );
                    double yMM = iu2mm( pos.y );
                    double zMM = getLayerZMM( layerId );

                    aFile << nodeName << " x=" << xMM << " y=" << yMM << " z=" << zMM << std::endl;
                }
            }
        }
    }

    aFile << std::endl;
}


void FASTHENRY_EXPORTER::writeEquivNodes( std::ofstream& aFile )
{
    aFile << "* ====== Node equivalences (connections) ======" << std::endl;

    // Connect co-located nodes from different geometric elements.
    // The m_nodeMap already de-duplicates: if a trace endpoint and via are at
    // the same (x, y, layer), they share the same node name.
    // However, we still need .equiv for ground plane named nodes connected to pads.
    for( const auto& [name1, name2] : m_equivPairs )
    {
        if( name1 != name2 )
            aFile << ".equiv " << name1 << " " << name2 << std::endl;
    }

    aFile << std::endl;
}


void FASTHENRY_EXPORTER::identifyPorts()
{
    m_ports.clear();

    // If the user specified explicit pad selections, use those instead of auto-discovery
    if( !m_config.m_PortSpecs.empty() )
    {
        identifyExplicitPorts();
        return;
    }

    // Strategy: create ports between power net pads and their nearest ground reference.
    // For PDN analysis, ports are typically at IC power pins and VRM output pins.
    //
    // Simple heuristic: for each pad on a target power net, create a port from that pad
    // to the first pad on the same footprint that connects to a ground net.

    std::set<int> groundNetCodes;

    // Find common ground nets
    for( int netCode : m_netCodes )
    {
        NETINFO_ITEM* net = m_board->FindNet( netCode );

        if( !net )
            continue;

        wxString name = net->GetNetname().Lower();

        if( name.Contains( "gnd" ) || name.Contains( "ground" ) ||
            name.Contains( "vss" ) || name.Contains( "gnd" ) )
        {
            groundNetCodes.insert( netCode );
        }
    }

    // Find the ground plane zone (for port reference nodes on the plane)
    const ZONE* groundZone = nullptr;
    int         groundZoneLayer = -1;

    for( const ZONE* zone : m_board->Zones() )
    {
        if( !zone->GetIsRuleArea() && groundNetCodes.count( zone->GetNetCode() ) )
        {
            groundZone = zone;
            LSET layers = zone->GetLayerSet();

            for( PCB_LAYER_ID id : layers.CuStack() )
            {
                groundZoneLayer = id;
                break;
            }

            break;
        }
    }

    // Track which (footprint, netCode) combinations already have a port,
    // so that when m_GroupPadsByComponent is enabled, multiple pads on the
    // same net and footprint get merged into a single port via .equiv.
    // Key: (footprint pointer, net code) -> positive node name of the first port
    std::map<std::pair<const FOOTPRINT*, int>, std::string> componentPortNodes;

    for( const FOOTPRINT* fp : m_board->Footprints() )
    {
        // Find ground pad(s) on this footprint
        const PAD* groundPad = nullptr;

        for( const PAD* pad : fp->Pads() )
        {
            if( groundNetCodes.count( pad->GetNetCode() ) )
            {
                groundPad = pad;
                break;
            }
        }

        if( !groundPad )
            continue;

        // For each power pad on this footprint, create a port
        for( const PAD* pad : fp->Pads() )
        {
            if( !isNetIncluded( pad->GetNetCode() ) )
                continue;

            if( groundNetCodes.count( pad->GetNetCode() ) )
                continue;  // Skip ground pads themselves

            // Find the nearest trace/via node on this net to use as the positive port node.
            // Using the actual conductor node (rather than the pad node) ensures the port
            // is connected to the conductor network that FastHenry solves.
            int layerId = pad->IsOnLayer( F_Cu ) ? F_Cu : B_Cu;
            VECTOR2I padPos = pad->GetPosition();
            int      padNetCode = pad->GetNetCode();

            std::string posNode;
            double      bestDistSq = std::numeric_limits<double>::max();

            for( const auto& [key, nodeName] : m_nodeMap )
            {
                auto [nx, ny, nl] = key;

                if( nl != layerId )
                    continue;

                // Only match nodes on the same net to avoid picking ground nodes
                auto netIt = m_nodeNetCode.find( key );

                if( netIt == m_nodeNetCode.end() || netIt->second != padNetCode )
                    continue;

                double dx = iu2mm( padPos.x - nx );
                double dy = iu2mm( padPos.y - ny );
                double distSq = dx * dx + dy * dy;

                if( distSq < bestDistSq )
                {
                    bestDistSq = distSq;
                    posNode = nodeName;
                }
            }

            if( posNode.empty() )
            {
                // No conductor node found on this layer — use pad node as fallback
                posNode = makePadNodeName( pad, layerId );
            }

            // When grouping by component, merge additional pads on the same
            // footprint+net into the first port's node via .equiv
            if( m_config.m_GroupPadsByComponent )
            {
                auto groupKey = std::make_pair( fp, pad->GetNetCode() );
                auto it = componentPortNodes.find( groupKey );

                if( it != componentPortNodes.end() )
                {
                    // This pad's node should be shorted to the first port's node
                    if( posNode != it->second )
                        m_equivPairs.push_back( { posNode, it->second } );

                    continue; // Don't create a duplicate port
                }

                componentPortNodes[groupKey] = posNode;
            }

            // For the ground reference, prefer a named node on the ground plane.
            // This ensures the node is part of the plane's conductor mesh.
            std::string negNode;

            if( groundZone && groundZoneLayer >= 0 )
            {
                double gndX = iu2mm( groundPad->GetPosition().x );
                double gndY = iu2mm( groundPad->GetPosition().y );
                negNode = makePlaneNodeName( groundZone, gndX, gndY );
                m_groundPlanePortNodes.push_back(
                        { negNode, gndX, gndY, getLayerZMM( groundZoneLayer ) } );
            }
            else
            {
                // Fallback: use the pad node directly
                int gndLayer = groundPad->IsOnLayer( F_Cu ) ? F_Cu : B_Cu;
                negNode = makePadNodeName( groundPad, gndLayer );
            }

            PDN_PARASITIC::EXTRACTION_PORT port;
            port.m_Name = fp->GetReference().ToStdString() + "_" + pad->GetNetname().ToStdString();
            port.m_PositiveNode = posNode;
            port.m_NegativeNode = negNode;
            port.m_NetName = pad->GetNetname().ToStdString();
            port.m_XMM = iu2mm( pad->GetPosition().x );
            port.m_YMM = iu2mm( pad->GetPosition().y );

            m_ports.push_back( port );
        }
    }
}


void FASTHENRY_EXPORTER::identifyExplicitPorts()
{
    m_ports.clear();

    // Split specs into signal and ground pads
    std::vector<const PDN_PARASITIC::PORT_SPEC*> signalSpecs;
    std::vector<const PDN_PARASITIC::PORT_SPEC*> groundSpecs;

    for( const auto& spec : m_config.m_PortSpecs )
    {
        if( spec.m_IsGround )
            groundSpecs.push_back( &spec );
        else
            signalSpecs.push_back( &spec );
    }

    if( signalSpecs.empty() || groundSpecs.empty() )
        return;

    // Find a ground plane zone (for creating named reference nodes on the plane mesh)
    const ZONE* groundZone = nullptr;
    int         groundZoneLayer = -1;

    std::set<int> groundNetCodes;

    for( const auto* gs : groundSpecs )
        groundNetCodes.insert( gs->m_NetCode );

    for( const ZONE* zone : m_board->Zones() )
    {
        if( !zone->GetIsRuleArea() && groundNetCodes.count( zone->GetNetCode() ) )
        {
            groundZone = zone;
            LSET layers = zone->GetLayerSet();

            for( PCB_LAYER_ID id : layers.CuStack() )
            {
                groundZoneLayer = id;
                break;
            }

            break;
        }
    }

    // Track which (name prefix, netCode) already have a port for component grouping
    std::map<std::pair<std::string, int>, std::string> componentPortNodes;

    for( const auto* spec : signalSpecs )
    {
        // Find the nearest conductor node on the same net as this pad
        int layerId = spec->m_LayerId;

        std::string posNode;
        double      bestDistSq = std::numeric_limits<double>::max();

        for( const auto& [key, nodeName] : m_nodeMap )
        {
            auto [nx, ny, nl] = key;

            if( nl != layerId )
                continue;

            // Only match nodes on the same net to avoid picking ground nodes
            auto netIt = m_nodeNetCode.find( key );

            if( netIt == m_nodeNetCode.end() || netIt->second != spec->m_NetCode )
                continue;

            double dx = spec->m_XMM - iu2mm( nx );
            double dy = spec->m_YMM - iu2mm( ny );
            double distSq = dx * dx + dy * dy;

            if( distSq < bestDistSq )
            {
                bestDistSq = distSq;
                posNode = nodeName;
            }
        }

        if( posNode.empty() )
            continue;

        // Component grouping: merge pads with same name prefix and net
        if( m_config.m_GroupPadsByComponent )
        {
            // Extract footprint reference prefix from the port name (e.g. "U1" from "U1_VDD")
            std::string namePrefix = spec->m_Name;
            size_t      sep = namePrefix.find( '_' );

            if( sep != std::string::npos )
                namePrefix = namePrefix.substr( 0, sep );

            auto groupKey = std::make_pair( namePrefix, spec->m_NetCode );
            auto it = componentPortNodes.find( groupKey );

            if( it != componentPortNodes.end() )
            {
                if( posNode != it->second )
                    m_equivPairs.push_back( { posNode, it->second } );

                continue;
            }

            componentPortNodes[groupKey] = posNode;
        }

        // Find nearest ground pad for the negative reference
        const PDN_PARASITIC::PORT_SPEC* nearestGnd = groundSpecs[0];
        double                          bestGndDistSq = std::numeric_limits<double>::max();

        for( const auto* gs : groundSpecs )
        {
            double dx = spec->m_XMM - gs->m_XMM;
            double dy = spec->m_YMM - gs->m_YMM;
            double distSq = dx * dx + dy * dy;

            if( distSq < bestGndDistSq )
            {
                bestGndDistSq = distSq;
                nearestGnd = gs;
            }
        }

        // Create the negative node from the ground reference
        std::string negNode;

        if( groundZone && groundZoneLayer >= 0 )
        {
            negNode = makePlaneNodeName( groundZone, nearestGnd->m_XMM, nearestGnd->m_YMM );
            m_groundPlanePortNodes.push_back( { negNode, nearestGnd->m_XMM, nearestGnd->m_YMM,
                                                getLayerZMM( groundZoneLayer ) } );
        }
        else
        {
            // No ground plane — find nearest conductor node on the ground net
            bestDistSq = std::numeric_limits<double>::max();

            for( const auto& [key, nodeName] : m_nodeMap )
            {
                auto [nx, ny, nl] = key;
                double dx = nearestGnd->m_XMM - iu2mm( nx );
                double dy = nearestGnd->m_YMM - iu2mm( ny );
                double distSq = dx * dx + dy * dy;

                if( distSq < bestDistSq )
                {
                    bestDistSq = distSq;
                    negNode = nodeName;
                }
            }
        }

        if( negNode.empty() )
            continue;

        PDN_PARASITIC::EXTRACTION_PORT port;
        port.m_Name = spec->m_Name;
        port.m_PositiveNode = posNode;
        port.m_NegativeNode = negNode;
        port.m_NetName = spec->m_NetName;
        port.m_XMM = spec->m_XMM;
        port.m_YMM = spec->m_YMM;

        m_ports.push_back( port );
    }
}


void FASTHENRY_EXPORTER::identifyReturnPaths()
{
    // For FastHenry to compute impedance, current must flow in a complete loop.
    // Ports (.external) define current injection between signal conductors and
    // the ground plane. Additional return paths are needed at other locations
    // to close the loop — specifically, power vias that pass through ground
    // plane layers need an electrical connection to the plane mesh.
    //
    // We must NOT equiv any node that is used as a port terminal, as that
    // would make the .external's positive and negative nodes the same.

    // Build a set of all node names used as port terminals
    std::set<std::string> portNodes;

    for( const auto& port : m_ports )
    {
        portNodes.insert( port.m_PositiveNode );
        portNodes.insert( port.m_NegativeNode );
    }

    // Collect ground net codes
    std::set<int> groundNetCodes;

    for( int netCode : m_netCodes )
    {
        NETINFO_ITEM* net = m_board->FindNet( netCode );

        if( !net )
            continue;

        wxString name = net->GetNetname().Lower();

        if( name.Contains( "gnd" ) || name.Contains( "ground" ) || name.Contains( "vss" ) )
        {
            groundNetCodes.insert( netCode );
        }
    }

    // Find ground plane zones and their layers
    struct GROUND_PLANE_INFO
    {
        const ZONE* zone;
        int         layerId;
        double      x1MM, y1MM, widthMM, heightMM;
    };

    std::vector<GROUND_PLANE_INFO> groundPlanes;

    for( const ZONE* zone : m_board->Zones() )
    {
        if( zone->GetIsRuleArea() || !groundNetCodes.count( zone->GetNetCode() ) )
            continue;

        BOX2I  bbox = zone->GetBoundingBox();
        double wMM = iu2mm( bbox.GetWidth() );
        double hMM = iu2mm( bbox.GetHeight() );

        if( wMM < 1.0 || hMM < 1.0 )
            continue;

        LSET layers = zone->GetLayerSet();

        for( PCB_LAYER_ID id : layers.CuStack() )
        {
            groundPlanes.push_back(
                    { zone, id, iu2mm( bbox.GetX() ), iu2mm( bbox.GetY() ), wMM, hMM } );
        }
    }

    // For each via on a power net, check if it crosses a ground plane layer
    for( const PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->Type() != PCB_VIA_T )
            continue;

        const PCB_VIA* via = static_cast<const PCB_VIA*>( track );

        if( !isNetIncluded( via->GetNetCode() ) )
            continue;

        // Skip vias on ground nets — they're already part of the plane
        if( groundNetCodes.count( via->GetNetCode() ) )
            continue;

        PCB_LAYER_ID topLayer, botLayer;
        via->LayerPair( &topLayer, &botLayer );

        double zTop = getLayerZMM( topLayer );
        double zBot = getLayerZMM( botLayer );
        double zMin = std::min( zTop, zBot );
        double zMax = std::max( zTop, zBot );

        double viaXMM = iu2mm( via->GetStart().x );
        double viaYMM = iu2mm( via->GetStart().y );

        for( const auto& gp : groundPlanes )
        {
            // Check if via passes through this ground plane layer (by z-position)
            double gpZ = getLayerZMM( gp.layerId );

            if( gpZ < zMin - 1e-6 || gpZ > zMax + 1e-6 )
                continue;

            // Check if via is within the ground plane bounding box
            if( viaXMM < gp.x1MM || viaXMM > gp.x1MM + gp.widthMM || viaYMM < gp.y1MM
                || viaYMM > gp.y1MM + gp.heightMM )
                continue;

            // Get the via's node name at this ground plane layer
            std::string viaNode = makeViaNodeName( via, gp.layerId );

            // Skip if this via node is used as a port terminal — equiv'ing it
            // to the ground plane would make the .external invalid
            if( portNodes.count( viaNode ) )
                continue;

            // Also check all via nodes on other layers — if any is a port terminal,
            // equiv'ing at this layer would still short the via barrel (which
            // connects all layers) to the ground plane
            bool usedAsPort = false;

            for( auto& [layerId, zPos] : m_layerZMM )
            {
                std::string nodeAtLayer = makeViaNodeName( via, layerId );

                if( portNodes.count( nodeAtLayer ) )
                {
                    usedAsPort = true;
                    break;
                }
            }

            if( usedAsPort )
                continue;

            // Create a named node on the ground plane at the via's (x,y)
            std::string planeNode = makePlaneNodeName( gp.zone, viaXMM, viaYMM );
            m_groundPlanePortNodes.push_back(
                    { planeNode, viaXMM, viaYMM, getLayerZMM( gp.layerId ) } );

            // Connect the via's node at this layer to the plane node via .equiv
            m_equivPairs.push_back( { viaNode, planeNode } );
        }
    }
}


void FASTHENRY_EXPORTER::writeExternals( std::ofstream& aFile )
{
    aFile << "* ====== Port definitions ======" << std::endl;

    for( const auto& port : m_ports )
    {
        aFile << "* Port " << port.m_Name << ": " << port.m_NetName << std::endl;
        aFile << ".external " << port.m_PositiveNode
              << " " << port.m_NegativeNode << std::endl;
    }

    aFile << std::endl;
}


void FASTHENRY_EXPORTER::writeFrequency( std::ofstream& aFile )
{
    aFile << "* ====== Frequency sweep ======" << std::endl;
    aFile << std::scientific;
    aFile << ".freq fmin=" << m_config.m_FreqMinHz
          << " fmax=" << m_config.m_FreqMaxHz
          << " ndec=" << m_config.m_PointsPerDecade
          << std::endl;
    aFile << std::fixed;
    aFile << std::endl;
}
