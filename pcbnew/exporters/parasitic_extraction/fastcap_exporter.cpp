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

#include "fastcap_exporter.h"

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
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


FASTCAP_EXPORTER::FASTCAP_EXPORTER( BOARD* aBoard,
                                    const PDN_PARASITIC::EXTRACTION_CONFIG& aConfig ) :
    m_board( aBoard ),
    m_config( aConfig )
{
}


bool FASTCAP_EXPORTER::Export( const std::string& aOutputDir,
                               const std::string& aListFileName )
{
    if( !m_board )
        return false;

    buildLayerZMap();
    buildDielectricLayers();

    // Resolve nets
    m_netCodes.clear();

    if( m_config.m_NetNames.empty() )
    {
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

    // Compute board extents for dielectric surfaces
    BOX2I boardBox = m_board->GetBoardEdgesBoundingBox();
    m_boardMinXM = iu2m( boardBox.GetX() );
    m_boardMinYM = iu2m( boardBox.GetY() );
    m_boardMaxXM = m_boardMinXM + iu2m( boardBox.GetWidth() );
    m_boardMaxYM = m_boardMinYM + iu2m( boardBox.GetHeight() );

    // Add margin
    double margin = 0.001; // 1mm in meters
    m_boardMinXM -= margin;
    m_boardMinYM -= margin;
    m_boardMaxXM += margin;
    m_boardMaxYM += margin;

    m_geoFiles.clear();

    // Write conductor geometry files
    std::string traceFile = writeTraceGeometry( aOutputDir );

    if( !traceFile.empty() )
    {
        GEOMETRY_FILE_ENTRY entry;
        entry.m_FileName = traceFile;
        entry.m_Type = 'C';
        // OuterPerm depends on embedding medium; we use the dominant dielectric
        entry.m_OuterPerm = m_dielectrics.empty() ? 1.0 : m_dielectrics[0].m_EpsilonR;
        m_geoFiles.push_back( entry );
    }

    std::string viaFile = writeViaGeometry( aOutputDir );

    if( !viaFile.empty() )
    {
        GEOMETRY_FILE_ENTRY entry;
        entry.m_FileName = viaFile;
        entry.m_Type = 'C';
        entry.m_OuterPerm = m_dielectrics.empty() ? 1.0 : m_dielectrics[0].m_EpsilonR;
        m_geoFiles.push_back( entry );
    }

    std::string planeFile = writePlaneGeometry( aOutputDir );

    if( !planeFile.empty() )
    {
        GEOMETRY_FILE_ENTRY entry;
        entry.m_FileName = planeFile;
        entry.m_Type = 'C';
        entry.m_OuterPerm = m_dielectrics.empty() ? 1.0 : m_dielectrics[0].m_EpsilonR;
        m_geoFiles.push_back( entry );
    }

    // Write dielectric interface surfaces
    double boardExtentX = m_boardMaxXM - m_boardMinXM;
    double boardExtentY = m_boardMaxYM - m_boardMinYM;
    double meshSize = m_config.m_PanelTargetSizeMM * 1e-3; // Convert mm to m

    for( size_t i = 0; i < m_dielectrics.size(); i++ )
    {
        // Top surface of dielectric (interface with copper or air above)
        std::string dielFile = writeDielectricSurface(
            aOutputDir, static_cast<int>( i ),
            m_dielectrics[i].m_ZTopM,
            boardExtentX, boardExtentY,
            m_boardMinXM, m_boardMinYM,
            meshSize );

        if( !dielFile.empty() )
        {
            GEOMETRY_FILE_ENTRY entry;
            entry.m_FileName = dielFile;
            entry.m_Type = 'D';
            entry.m_OuterPerm = 1.0;  // Above: air or different dielectric
            entry.m_InnerPerm = m_dielectrics[i].m_EpsilonR;

            // Reference point inside the dielectric
            double refZ = ( m_dielectrics[i].m_ZTopM + m_dielectrics[i].m_ZBotM ) / 2.0;
            entry.m_RefX = ( m_boardMinXM + m_boardMaxXM ) / 2.0;
            entry.m_RefY = ( m_boardMinYM + m_boardMaxYM ) / 2.0;
            entry.m_RefZ = refZ;

            m_geoFiles.push_back( entry );
        }
    }

    // Write the master list file
    std::string lstPath = aOutputDir + "/" + aListFileName;
    std::ofstream lstFile( lstPath );

    if( !lstFile.is_open() )
        return false;

    writeListFile( lstFile, aOutputDir );
    lstFile.close();

    return true;
}


void FASTCAP_EXPORTER::buildLayerZMap()
{
    const BOARD_DESIGN_SETTINGS& bds = m_board->GetDesignSettings();
    const BOARD_STACKUP& stackup = bds.GetStackupDescriptor();

    m_layerZTopM.clear();
    m_layerThickM.clear();

    double zM = 0.0;

    for( const BOARD_STACKUP_ITEM* item : stackup.GetList() )
    {
        if( !item->IsEnabled() )
            continue;

        if( item->GetType() == BS_ITEM_TYPE_COPPER )
        {
            double thickM = iu2m( item->GetThickness() );

            if( thickM <= 0.0 )
                thickM = 35e-6;  // Default 1oz = 35um

            int layerId = item->GetBrdLayerId();
            m_layerZTopM[layerId] = zM;
            m_layerThickM[layerId] = thickM;

            zM += thickM;
        }
        else if( item->GetType() == BS_ITEM_TYPE_DIELECTRIC )
        {
            for( int sub = 0; sub < item->GetSublayersCount(); sub++ )
                zM += iu2m( item->GetThickness( sub ) );
        }
    }

    // Fallback for boards without stackup
    if( m_layerZTopM.empty() )
    {
        double boardThickM = iu2m( bds.GetBoardThickness() );
        double copperM = 35e-6;

        m_layerZTopM[F_Cu] = 0.0;
        m_layerThickM[F_Cu] = copperM;

        m_layerZTopM[B_Cu] = boardThickM - copperM;
        m_layerThickM[B_Cu] = copperM;
    }
}


void FASTCAP_EXPORTER::buildDielectricLayers()
{
    const BOARD_DESIGN_SETTINGS& bds = m_board->GetDesignSettings();
    const BOARD_STACKUP& stackup = bds.GetStackupDescriptor();

    m_dielectrics.clear();

    double zM = 0.0;

    for( const BOARD_STACKUP_ITEM* item : stackup.GetList() )
    {
        if( !item->IsEnabled() )
            continue;

        if( item->GetType() == BS_ITEM_TYPE_COPPER )
        {
            double thickM = iu2m( item->GetThickness() );

            if( thickM <= 0.0 )
                thickM = 35e-6;

            zM += thickM;
        }
        else if( item->GetType() == BS_ITEM_TYPE_DIELECTRIC )
        {
            for( int sub = 0; sub < item->GetSublayersCount(); sub++ )
            {
                double thickM = iu2m( item->GetThickness( sub ) );

                DIELECTRIC_INFO di;
                di.m_ZTopM = zM;
                di.m_ZBotM = zM + thickM;
                di.m_EpsilonR = item->GetEpsilonR( sub );
                di.m_LossTangent = item->GetLossTangent( sub );

                if( di.m_EpsilonR <= 0.0 )
                    di.m_EpsilonR = 4.5;  // Default FR4

                m_dielectrics.push_back( di );
                zM += thickM;
            }
        }
    }

    // Fallback
    if( m_dielectrics.empty() )
    {
        double boardThickM = iu2m( bds.GetBoardThickness() );
        double copperM = 35e-6;

        DIELECTRIC_INFO di;
        di.m_ZTopM = copperM;
        di.m_ZBotM = boardThickM - copperM;
        di.m_EpsilonR = 4.5;
        di.m_LossTangent = 0.02;
        m_dielectrics.push_back( di );
    }
}


double FASTCAP_EXPORTER::getLayerZTopM( int aLayerId ) const
{
    auto it = m_layerZTopM.find( aLayerId );
    return ( it != m_layerZTopM.end() ) ? it->second : 0.0;
}


double FASTCAP_EXPORTER::getLayerZBotM( int aLayerId ) const
{
    return getLayerZTopM( aLayerId ) + getCopperThicknessM( aLayerId );
}


double FASTCAP_EXPORTER::getCopperThicknessM( int aLayerId ) const
{
    auto it = m_layerThickM.find( aLayerId );
    return ( it != m_layerThickM.end() ) ? it->second : 35e-6;
}


bool FASTCAP_EXPORTER::isNetIncluded( int aNetCode ) const
{
    return m_netCodes.count( aNetCode ) > 0;
}


void FASTCAP_EXPORTER::writeQuad( std::ofstream& aFile, const std::string& aConductor,
                                   double x1, double y1, double z1,
                                   double x2, double y2, double z2,
                                   double x3, double y3, double z3,
                                   double x4, double y4, double z4 )
{
    aFile << "Q " << aConductor
          << "  " << x1 << " " << y1 << " " << z1
          << "  " << x2 << " " << y2 << " " << z2
          << "  " << x3 << " " << y3 << " " << z3
          << "  " << x4 << " " << y4 << " " << z4
          << std::endl;
}


void FASTCAP_EXPORTER::writeTri( std::ofstream& aFile, const std::string& aConductor,
                                  double x1, double y1, double z1,
                                  double x2, double y2, double z2,
                                  double x3, double y3, double z3 )
{
    aFile << "T " << aConductor
          << "  " << x1 << " " << y1 << " " << z1
          << "  " << x2 << " " << y2 << " " << z2
          << "  " << x3 << " " << y3 << " " << z3
          << std::endl;
}


void FASTCAP_EXPORTER::meshTrace( std::ofstream& aFile, const std::string& aConductor,
                                   double x1, double y1, double x2, double y2,
                                   double zTop, double zBot, double width )
{
    // Compute the direction vector and perpendicular (for width)
    double dx = x2 - x1;
    double dy = y2 - y1;
    double len = std::sqrt( dx * dx + dy * dy );

    if( len < 1e-12 )
        return;

    // Unit direction and perpendicular
    double ux = dx / len;
    double uy = dy / len;
    double px = -uy * width / 2.0;  // Perpendicular offset for half-width
    double py = ux * width / 2.0;

    // Determine mesh segment count along the trace length
    double panelSizeM = m_config.m_PanelTargetSizeMM * 1e-3;
    int nSeg = std::max( 1, static_cast<int>( std::ceil( len / panelSizeM ) ) );
    double segLen = len / nSeg;

    for( int i = 0; i < nSeg; i++ )
    {
        double t0 = static_cast<double>( i ) / nSeg;
        double t1 = static_cast<double>( i + 1 ) / nSeg;

        double sx0 = x1 + dx * t0;
        double sy0 = y1 + dy * t0;
        double sx1 = x1 + dx * t1;
        double sy1 = y1 + dy * t1;

        // Top face
        writeQuad( aFile, aConductor,
                   sx0 + px, sy0 + py, zTop,
                   sx1 + px, sy1 + py, zTop,
                   sx1 - px, sy1 - py, zTop,
                   sx0 - px, sy0 - py, zTop );

        // Bottom face
        writeQuad( aFile, aConductor,
                   sx0 - px, sy0 - py, zBot,
                   sx1 - px, sy1 - py, zBot,
                   sx1 + px, sy1 + py, zBot,
                   sx0 + px, sy0 + py, zBot );

        // Left side
        writeQuad( aFile, aConductor,
                   sx0 + px, sy0 + py, zTop,
                   sx0 + px, sy0 + py, zBot,
                   sx1 + px, sy1 + py, zBot,
                   sx1 + px, sy1 + py, zTop );

        // Right side
        writeQuad( aFile, aConductor,
                   sx0 - px, sy0 - py, zBot,
                   sx0 - px, sy0 - py, zTop,
                   sx1 - px, sy1 - py, zTop,
                   sx1 - px, sy1 - py, zBot );
    }

    // End cap at start
    writeQuad( aFile, aConductor,
               x1 + px, y1 + py, zTop,
               x1 - px, y1 - py, zTop,
               x1 - px, y1 - py, zBot,
               x1 + px, y1 + py, zBot );

    // End cap at end
    writeQuad( aFile, aConductor,
               x2 - px, y2 - py, zTop,
               x2 + px, y2 + py, zTop,
               x2 + px, y2 + py, zBot,
               x2 - px, y2 - py, zBot );
}


void FASTCAP_EXPORTER::meshViaBarrel( std::ofstream& aFile, const std::string& aConductor,
                                       double cx, double cy, double zTop, double zBot,
                                       double radius, int nFacets )
{
    // Divide the barrel vertically into panels
    double panelSizeM = m_config.m_PanelTargetSizeMM * 1e-3;
    double height = std::abs( zBot - zTop );
    int nRows = std::max( 1, static_cast<int>( std::ceil( height / panelSizeM ) ) );

    double dAngle = 2.0 * M_PI / nFacets;

    for( int row = 0; row < nRows; row++ )
    {
        double z0 = zTop + ( zBot - zTop ) * row / nRows;
        double z1 = zTop + ( zBot - zTop ) * ( row + 1 ) / nRows;

        for( int f = 0; f < nFacets; f++ )
        {
            double a0 = f * dAngle;
            double a1 = ( f + 1 ) * dAngle;

            double x0 = cx + radius * std::cos( a0 );
            double y0 = cy + radius * std::sin( a0 );
            double x1 = cx + radius * std::cos( a1 );
            double y1 = cy + radius * std::sin( a1 );

            writeQuad( aFile, aConductor,
                       x0, y0, z0,
                       x1, y1, z0,
                       x1, y1, z1,
                       x0, y0, z1 );
        }
    }

    // Top annular cap (simplified as triangles from center)
    for( int f = 0; f < nFacets; f++ )
    {
        double a0 = f * dAngle;
        double a1 = ( f + 1 ) * dAngle;

        writeTri( aFile, aConductor,
                  cx, cy, zTop,
                  cx + radius * std::cos( a0 ), cy + radius * std::sin( a0 ), zTop,
                  cx + radius * std::cos( a1 ), cy + radius * std::sin( a1 ), zTop );
    }

    // Bottom annular cap
    for( int f = 0; f < nFacets; f++ )
    {
        double a0 = f * dAngle;
        double a1 = ( f + 1 ) * dAngle;

        writeTri( aFile, aConductor,
                  cx, cy, zBot,
                  cx + radius * std::cos( a1 ), cy + radius * std::sin( a1 ), zBot,
                  cx + radius * std::cos( a0 ), cy + radius * std::sin( a0 ), zBot );
    }
}


void FASTCAP_EXPORTER::meshPlane( std::ofstream& aFile, const std::string& aConductor,
                                   double x1, double y1, double x2, double y2,
                                   double z, int nMeshX, int nMeshY )
{
    double dx = ( x2 - x1 ) / nMeshX;
    double dy = ( y2 - y1 ) / nMeshY;

    for( int ix = 0; ix < nMeshX; ix++ )
    {
        for( int iy = 0; iy < nMeshY; iy++ )
        {
            double px = x1 + ix * dx;
            double py = y1 + iy * dy;

            writeQuad( aFile, aConductor,
                       px, py, z,
                       px + dx, py, z,
                       px + dx, py + dy, z,
                       px, py + dy, z );
        }
    }
}


std::string FASTCAP_EXPORTER::writeTraceGeometry( const std::string& aOutputDir )
{
    std::string filename = "traces.qui";
    std::string path = aOutputDir + "/" + filename;
    std::ofstream file( path );

    if( !file.is_open() )
        return "";

    file << std::scientific << std::setprecision( 8 );
    file << "0 PCB trace conductor surfaces" << std::endl;

    int traceCount = 0;

    for( const PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->Type() == PCB_VIA_T )
            continue;

        if( !isNetIncluded( track->GetNetCode() ) )
            continue;

        int layerId = track->GetLayer();
        double zTop = getLayerZTopM( layerId );
        double zBot = getLayerZBotM( layerId );
        double widthM = iu2m( track->GetWidth() );

        if( widthM <= 0.0 )
            continue;

        VECTOR2I start = track->GetStart();
        VECTOR2I end = track->GetEnd();

        // Conductor name is the net name (sanitized)
        wxString netName = track->GetNetname();
        netName.Replace( "/", "_" );
        netName.Replace( " ", "_" );
        std::string condName = netName.ToStdString();

        if( condName.empty() )
            condName = "NET_" + std::to_string( track->GetNetCode() );

        meshTrace( file, condName,
                   iu2m( start.x ), iu2m( start.y ),
                   iu2m( end.x ), iu2m( end.y ),
                   zTop, zBot, widthM );

        traceCount++;
    }

    file.close();

    return traceCount > 0 ? filename : "";
}


std::string FASTCAP_EXPORTER::writeViaGeometry( const std::string& aOutputDir )
{
    std::string filename = "vias.qui";
    std::string path = aOutputDir + "/" + filename;
    std::ofstream file( path );

    if( !file.is_open() )
        return "";

    file << std::scientific << std::setprecision( 8 );
    file << "0 PCB via barrel conductor surfaces" << std::endl;

    int viaCount = 0;

    for( const PCB_TRACK* track : m_board->Tracks() )
    {
        if( track->Type() != PCB_VIA_T )
            continue;

        const PCB_VIA* via = static_cast<const PCB_VIA*>( track );

        if( !isNetIncluded( via->GetNetCode() ) )
            continue;

        PCB_LAYER_ID topLayer, botLayer;
        via->LayerPair( &topLayer, &botLayer );

        double radiusM = iu2m( via->GetDrillValue() ) / 2.0;

        if( radiusM <= 0.0 )
            continue;

        double cx = iu2m( via->GetStart().x );
        double cy = iu2m( via->GetStart().y );
        double zTop = getLayerZTopM( topLayer );
        double zBot = getLayerZBotM( botLayer );

        wxString netName = via->GetNetname();
        netName.Replace( "/", "_" );
        netName.Replace( " ", "_" );
        std::string condName = netName.ToStdString();

        if( condName.empty() )
            condName = "NET_" + std::to_string( via->GetNetCode() );

        meshViaBarrel( file, condName, cx, cy, zTop, zBot,
                       radiusM, m_config.m_ViaFacets );

        viaCount++;
    }

    file.close();

    return viaCount > 0 ? filename : "";
}


std::string FASTCAP_EXPORTER::writePlaneGeometry( const std::string& aOutputDir )
{
    std::string filename = "planes.qui";
    std::string path = aOutputDir + "/" + filename;
    std::ofstream file( path );

    if( !file.is_open() )
        return "";

    file << std::scientific << std::setprecision( 8 );
    file << "0 PCB copper plane conductor surfaces" << std::endl;

    int planeCount = 0;

    for( const ZONE* zone : m_board->Zones() )
    {
        if( !isNetIncluded( zone->GetNetCode() ) )
            continue;

        if( zone->GetIsRuleArea() )
            continue;

        BOX2I bbox = zone->GetBoundingBox();
        double widthM = iu2m( bbox.GetWidth() );
        double heightM = iu2m( bbox.GetHeight() );

        if( widthM < 1e-3 || heightM < 1e-3 )
            continue;

        LSET layers = zone->GetLayerSet();

        for( PCB_LAYER_ID layerId : layers.CuStack() )
        {
            double zTop = getLayerZTopM( layerId );
            double zBot = getLayerZBotM( layerId );
            double x1 = iu2m( bbox.GetX() );
            double y1 = iu2m( bbox.GetY() );
            double x2 = x1 + widthM;
            double y2 = y1 + heightM;

            wxString netName = zone->GetNetname();
            netName.Replace( "/", "_" );
            netName.Replace( " ", "_" );
            std::string condName = netName.ToStdString();

            if( condName.empty() )
                condName = "ZONE_" + std::to_string( zone->GetNetCode() );

            // Top and bottom surfaces of the copper plane
            meshPlane( file, condName, x1, y1, x2, y2, zTop,
                       m_config.m_PlaneSegX, m_config.m_PlaneSegY );
            meshPlane( file, condName, x1, y1, x2, y2, zBot,
                       m_config.m_PlaneSegX, m_config.m_PlaneSegY );

            planeCount++;
        }
    }

    file.close();

    return planeCount > 0 ? filename : "";
}


std::string FASTCAP_EXPORTER::writeDielectricSurface( const std::string& aOutputDir,
                                                       int aIndex,
                                                       double aZM,
                                                       double aExtentXM, double aExtentYM,
                                                       double aOriginXM, double aOriginYM,
                                                       double aMeshSizeM )
{
    std::ostringstream nameSS;
    nameSS << "dielectric_" << aIndex << ".qui";
    std::string filename = nameSS.str();
    std::string path = aOutputDir + "/" + filename;

    std::ofstream file( path );

    if( !file.is_open() )
        return "";

    file << std::scientific << std::setprecision( 8 );
    file << "0 Dielectric interface surface at z=" << aZM << std::endl;

    int nX = std::max( 2, static_cast<int>( std::ceil( aExtentXM / aMeshSizeM ) ) );
    int nY = std::max( 2, static_cast<int>( std::ceil( aExtentYM / aMeshSizeM ) ) );

    // Cap the mesh density to avoid excessive panel counts.
    // FastCap memory usage scales as O(N^2) with panel count;
    // the original MIT FastCap2 has a ~4MB internal memory limit.
    nX = std::min( nX, 10 );
    nY = std::min( nY, 10 );

    double dx = aExtentXM / nX;
    double dy = aExtentYM / nY;

    for( int ix = 0; ix < nX; ix++ )
    {
        for( int iy = 0; iy < nY; iy++ )
        {
            double px = aOriginXM + ix * dx;
            double py = aOriginYM + iy * dy;

            // For dielectric surfaces, conductor name is ignored (use "0" or "diel")
            writeQuad( file, "0",
                       px, py, aZM,
                       px + dx, py, aZM,
                       px + dx, py + dy, aZM,
                       px, py + dy, aZM );
        }
    }

    file.close();
    return filename;
}


void FASTCAP_EXPORTER::writeListFile( std::ofstream& aFile, const std::string& aOutputDir )
{
    aFile << "* FastCap list file generated from KiCad PCB layout" << std::endl;
    aFile << "* Board: " << m_board->GetFileName().ToStdString() << std::endl;
    aFile << "*" << std::endl;

    aFile << std::scientific << std::setprecision( 8 );

    for( const auto& entry : m_geoFiles )
    {
        if( entry.m_Type == 'C' )
        {
            aFile << "C " << entry.m_FileName
                  << " " << entry.m_OuterPerm
                  << "  0.0 0.0 0.0"   // No translation
                  << std::endl;
        }
        else if( entry.m_Type == 'D' )
        {
            aFile << "D " << entry.m_FileName
                  << " " << entry.m_OuterPerm
                  << " " << entry.m_InnerPerm
                  << "  0.0 0.0 0.0"   // No translation
                  << "  " << entry.m_RefX << " " << entry.m_RefY << " " << entry.m_RefZ
                  << std::endl;
        }
        else if( entry.m_Type == 'B' )
        {
            aFile << "B " << entry.m_FileName
                  << " " << entry.m_OuterPerm
                  << " " << entry.m_InnerPerm
                  << "  0.0 0.0 0.0"
                  << "  " << entry.m_RefX << " " << entry.m_RefY << " " << entry.m_RefZ
                  << std::endl;
        }
    }

    aFile << std::fixed;
}
