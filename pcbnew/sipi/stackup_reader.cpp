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

#include "stackup_reader.h"

#include <board.h>
#include <board_design_settings.h>
#include <board_stackup_manager/board_stackup.h>
#include <footprint.h>
#include <pcb_shape.h>
#include <zone.h>


// KiCad default epsilon_r value (used to detect unconfigured stackups)
static constexpr double DEFAULT_ER = 4.5;

// Nanometers to meters conversion
static constexpr double NM_TO_M = 1e-9;


STACKUP_READER::STACKUP_READER( const BOARD* aBoard ) :
        m_board( aBoard ),
        m_usingDefaults( false ),
        m_built( false )
{
}


void STACKUP_READER::buildLayerModel()
{
    if( m_built )
        return;

    m_built = true;
    m_copperLayers.clear();
    m_dielectrics.clear();
    m_usingDefaults = false;

    if( !m_board )
        return;

    const BOARD_STACKUP& stackup =
            m_board->GetDesignSettings().GetStackupDescriptor();

    // Walk the stackup list top-to-bottom, tracking cumulative z position.
    // The list is ordered from the top of the board to the bottom.
    double z = 0.0;

    for( BOARD_STACKUP_ITEM* item : stackup.GetList() )
    {
        if( !item->IsEnabled() )
            continue;

        if( item->GetType() == BS_ITEM_TYPE_COPPER )
        {
            int thicknessNm = item->GetThickness();
            double thickness = thicknessNm * NM_TO_M;

            COPPER_LAYER_INFO info;
            info.layerId = item->GetBrdLayerId();
            info.zPosition = z + thickness / 2.0; // center of the copper layer
            info.thickness = thickness;
            m_copperLayers.push_back( info );

            z += thickness;
        }
        else if( item->GetType() == BS_ITEM_TYPE_DIELECTRIC )
        {
            // Dielectrics can have sublayers; use the first sublayer's properties
            int thicknessNm = item->GetThickness( 0 );
            double thickness = thicknessNm * NM_TO_M;
            double er = item->GetEpsilonR( 0 );
            double tanD = item->GetLossTangent( 0 );

            // Detect default/unconfigured values
            if( std::abs( er - DEFAULT_ER ) < 0.01 )
                m_usingDefaults = true;

            DIELECTRIC_INFO diel;
            diel.zTop = z;
            diel.zBottom = z + thickness;
            diel.epsilonR = er;
            diel.lossTangent = tanD;
            m_dielectrics.push_back( diel );

            z += thickness;
        }
        else if( item->GetType() == BS_ITEM_TYPE_SOLDERMASK )
        {
            double thickness = item->GetThickness() * NM_TO_M;
            double er = item->GetEpsilonR( 0 );
            bool isDefault = ( item->GetThickness()
                               == BOARD_STACKUP_ITEM::GetMaskDefaultThickness() );

            // Top solder mask appears before any copper; bottom appears after
            if( m_copperLayers.empty() )
            {
                m_solderMaskTop.thickness = thickness;
                m_solderMaskTop.epsilonR = er;
                m_solderMaskTop.present = true;
                m_solderMaskTop.isDefault = isDefault;
            }
            else
            {
                m_solderMaskBottom.thickness = thickness;
                m_solderMaskBottom.epsilonR = er;
                m_solderMaskBottom.present = true;
                m_solderMaskBottom.isDefault = isDefault;
            }

            z += thickness;
        }
    }

    buildMaskItemCache();
}


bool STACKUP_READER::isReferencePlane( PCB_LAYER_ID aLayer, const VECTOR2I& aPosition ) const
{
    // A layer is a reference plane at a given position if there is a filled zone
    // on that layer covering the position.  Zones must be filled (Edit > Fill All Zones)
    // for this to work — unfilled zones have no fill data to test against.
    for( ZONE* zone : m_board->Zones() )
    {
        if( !zone->IsOnLayer( aLayer ) )
            continue;

        if( zone->GetIsRuleArea() )
            continue;

        if( zone->HitTestFilledArea( aLayer, aPosition ) )
            return true;
    }

    return false;
}


LAYER_GEOMETRY STACKUP_READER::GetLayerGeometry( PCB_LAYER_ID aLayer,
                                                 const VECTOR2I& aPosition,
                                                 int aTraceWidth )
{
    buildLayerModel();

    LAYER_GEOMETRY geom;
    geom.signalLayer = aLayer;
    geom.traceWidth = aTraceWidth * NM_TO_M;
    geom.usingDefaults = m_usingDefaults;

    // Find the signal layer in our copper layer list
    int signalIdx = -1;

    for( int i = 0; i < (int) m_copperLayers.size(); i++ )
    {
        if( m_copperLayers[i].layerId == aLayer )
        {
            signalIdx = i;
            geom.traceThickness = m_copperLayers[i].thickness;
            break;
        }
    }

    if( signalIdx < 0 )
        return geom; // Layer not found in stackup

    double signalZ = m_copperLayers[signalIdx].zPosition;
    geom.signalZPosition = signalZ;

    // Helper: find the dielectric layer whose z-range overlaps the interval
    // between two copper layer centers. Use midpoint containment — the
    // dielectric whose center is between the two copper centers.
    auto findDielectric = [&]( double aZUpper, double aZLower, double& aEr, double& aTanD )
    {
        for( const DIELECTRIC_INFO& diel : m_dielectrics )
        {
            double dielCenter = ( diel.zTop + diel.zBottom ) / 2.0;

            if( dielCenter > aZUpper && dielCenter < aZLower )
            {
                aEr = diel.epsilonR;
                aTanD = diel.lossTangent;
                return;
            }
        }
    };

    // Helper: compute dielectric height between two copper layer indices
    auto copperSpacing = [&]( int aIdxA, int aIdxB ) -> double
    {
        double zA = m_copperLayers[aIdxA].zPosition;
        double zB = m_copperLayers[aIdxB].zPosition;
        double tA = m_copperLayers[aIdxA].thickness;
        double tB = m_copperLayers[aIdxB].thickness;
        // Edge-to-edge distance between copper layers
        return std::abs( zB - zA ) - tA / 2.0 - tB / 2.0;
    };

    // Zone-aware reference plane detection: scan copper layers above/below the
    // signal, checking isReferencePlane() for zone coverage at the query point.
    // Layers without zone coverage are skipped (recorded as intermediate layers
    // for groundwire detection).  If no zone-covered layer is found, hasRefAbove/
    // hasRefBelow remains false and the virtual earth fallback below handles it.
    for( int i = signalIdx - 1; i >= 0; i-- )
    {
        if( isReferencePlane( m_copperLayers[i].layerId, aPosition ) )
        {
            geom.hAbove = copperSpacing( i, signalIdx );
            geom.hasRefAbove = true;
            geom.refLayerAbove = m_copperLayers[i].layerId;
            geom.refThicknessAbove = m_copperLayers[i].thickness;
            findDielectric( m_copperLayers[i].zPosition, signalZ,
                            geom.erAbove, geom.tanDAbove );

            geom.hOrigAbove = geom.hAbove;
            geom.erOrigAbove = geom.erAbove;
            break;
        }

        // Not a reference plane here — record as intermediate layer
        LAYER_GEOMETRY::INTERMEDIATE_LAYER il;
        il.layerId = m_copperLayers[i].layerId;
        il.zPosition = m_copperLayers[i].zPosition;
        il.thickness = m_copperLayers[i].thickness;
        geom.intermediateLayers.push_back( il );
    }

    for( int i = signalIdx + 1; i < (int) m_copperLayers.size(); i++ )
    {
        if( isReferencePlane( m_copperLayers[i].layerId, aPosition ) )
        {
            geom.hBelow = copperSpacing( signalIdx, i );
            geom.hasRefBelow = true;
            geom.refLayerBelow = m_copperLayers[i].layerId;
            geom.refThicknessBelow = m_copperLayers[i].thickness;
            findDielectric( signalZ, m_copperLayers[i].zPosition,
                            geom.erBelow, geom.tanDBelow );

            geom.hOrigBelow = geom.hBelow;
            geom.erOrigBelow = geom.erBelow;
            break;
        }

        // Not a reference plane here — record as intermediate layer
        LAYER_GEOMETRY::INTERMEDIATE_LAYER il;
        il.layerId = m_copperLayers[i].layerId;
        il.zPosition = m_copperLayers[i].zPosition;
        il.thickness = m_copperLayers[i].thickness;
        geom.intermediateLayers.push_back( il );
    }

    // Compute fallback image ground for demotion: the outermost copper layer
    // in each direction, with the actual dielectric stackup.
    if( signalIdx > 0 )
    {
        int outerIdx = 0;
        geom.hFallbackAbove = copperSpacing( outerIdx, signalIdx );
        findDielectric( m_copperLayers[outerIdx].zPosition, signalZ,
                        geom.erFallbackAbove, geom.tanDFallbackAbove );
    }

    if( signalIdx < (int) m_copperLayers.size() - 1 )
    {
        int outerIdx = (int) m_copperLayers.size() - 1;
        geom.hFallbackBelow = copperSpacing( signalIdx, outerIdx );
        findDielectric( signalZ, m_copperLayers[outerIdx].zPosition,
                        geom.erFallbackBelow, geom.tanDFallbackBelow );
    }

    // Virtual earth fallback: if no reference plane was found in a direction,
    // use the outermost copper layer as the image ground with the actual
    // dielectric stackup.
    if( !geom.hasRefAbove && signalIdx > 0 )
    {
        geom.hAbove = geom.hFallbackAbove;
        geom.hasRefAbove = true;
        geom.erAbove = geom.erFallbackAbove;
        geom.tanDAbove = geom.tanDFallbackAbove;
    }

    if( !geom.hasRefBelow && signalIdx < (int) m_copperLayers.size() - 1 )
    {
        geom.hBelow = geom.hFallbackBelow;
        geom.hasRefBelow = true;
        geom.erBelow = geom.erFallbackBelow;
        geom.tanDBelow = geom.tanDFallbackBelow;
    }

    // Solder mask: only on outer copper layers, on the air-facing (outward) side.
    // F.Cu (topmost) gets top solder mask above; B.Cu (bottommost) gets bottom mask below.
    // Suppress if there's a mask opening (cutout, pad, etc.) at this position.
    if( signalIdx == 0 && m_solderMaskTop.present
        && !hasSolderMaskOpening( F_Mask, aPosition ) )
    {
        geom.solderMaskThickness = m_solderMaskTop.thickness;
        geom.solderMaskEr = m_solderMaskTop.epsilonR;
        geom.solderMaskAbove = true;
        geom.solderMaskIsDefault = m_solderMaskTop.isDefault;
    }
    else if( signalIdx == (int) m_copperLayers.size() - 1 && m_solderMaskBottom.present
             && !hasSolderMaskOpening( B_Mask, aPosition ) )
    {
        geom.solderMaskThickness = m_solderMaskBottom.thickness;
        geom.solderMaskEr = m_solderMaskBottom.epsilonR;
        geom.solderMaskAbove = false;
        geom.solderMaskIsDefault = m_solderMaskBottom.isDefault;
    }

    return geom;
}


bool STACKUP_READER::isReferencePlaneNearby( PCB_LAYER_ID aLayer, const VECTOR2I& aPosition,
                                              int aSearchRadius ) const
{
    // For via modeling: a layer is a reference plane if a filled zone exists
    // nearby, even if the exact via position is inside an antipad void.
    // The zone is still the reference — the antipad is just the clearance hole.
    for( ZONE* zone : m_board->Zones() )
    {
        if( !zone->IsOnLayer( aLayer ) )
            continue;

        if( zone->GetIsRuleArea() )
            continue;

        const std::shared_ptr<SHAPE_POLY_SET>& fill = zone->GetFilledPolysList( aLayer );

        if( !fill || fill->IsEmpty() )
            continue;

        // Check if the zone fill edge is within the search radius.
        // SquaredDistance returns 0 if the point is inside the fill.
        SEG::ecoord d2 = fill->SquaredDistance( aPosition );

        if( d2 <= (SEG::ecoord) aSearchRadius * aSearchRadius )
            return true;
    }

    return false;
}


double STACKUP_READER::findAntipadRadius( PCB_LAYER_ID aLayer, const VECTOR2I& aPosition,
                                          double aFallback ) const
{
    // Find the nearest zone fill edge on this layer at this position.
    // The via sits inside the antipad (clearance hole), so the nearest fill edge
    // gives the antipad radius.
    double bestDistSq = 1e30;

    for( ZONE* zone : m_board->Zones() )
    {
        if( !zone->IsOnLayer( aLayer ) )
            continue;

        if( zone->GetIsRuleArea() )
            continue;

        const std::shared_ptr<SHAPE_POLY_SET>& fill = zone->GetFilledPolysList( aLayer );

        if( !fill || fill->IsEmpty() )
            continue;

        SEG::ecoord d2 = fill->SquaredDistance( aPosition );

        if( d2 < bestDistSq )
            bestDistSq = d2;
    }

    if( bestDistSq < 1e29 )
    {
        double distNm = std::sqrt( (double) bestDistSq );
        double distM = distNm * 1e-9;

        if( distM > 0.0 )
            return distM;
    }

    return aFallback;
}


void STACKUP_READER::buildMaskItemCache()
{
    m_maskItemsTop.clear();
    m_maskItemsBottom.clear();

    if( !m_board )
        return;

    // Board-level drawings on mask layers
    for( BOARD_ITEM* item : m_board->Drawings() )
    {
        if( item->IsOnLayer( F_Mask ) )
            m_maskItemsTop.push_back( item );

        if( item->IsOnLayer( B_Mask ) )
            m_maskItemsBottom.push_back( item );
    }

    // Footprint graphics on mask layers
    for( FOOTPRINT* fp : m_board->Footprints() )
    {
        for( BOARD_ITEM* item : fp->GraphicalItems() )
        {
            if( item->IsOnLayer( F_Mask ) )
                m_maskItemsTop.push_back( item );

            if( item->IsOnLayer( B_Mask ) )
                m_maskItemsBottom.push_back( item );
        }
    }

    // Zones on mask layers
    for( ZONE* zone : m_board->Zones() )
    {
        if( zone->GetIsRuleArea() )
            continue;

        if( zone->IsOnLayer( F_Mask ) )
            m_maskItemsTop.push_back( zone );

        if( zone->IsOnLayer( B_Mask ) )
            m_maskItemsBottom.push_back( zone );
    }
}


bool STACKUP_READER::hasSolderMaskOpening( PCB_LAYER_ID aMaskLayer,
                                           const VECTOR2I& aPosition ) const
{
    const std::vector<BOARD_ITEM*>& items =
            ( aMaskLayer == F_Mask ) ? m_maskItemsTop : m_maskItemsBottom;

    for( BOARD_ITEM* item : items )
    {
        if( item->HitTest( aPosition, 0 ) )
            return true;
    }

    return false;
}


VIA_PARAMS STACKUP_READER::GetViaGeometry( PCB_LAYER_ID aTopLayer, PCB_LAYER_ID aBottomLayer,
                                            PCB_LAYER_ID aSignalLayer,
                                            const VECTOR2I& aPosition,
                                            double aDrillRadius, double aPadRadius )
{
    buildLayerModel();

    VIA_PARAMS params;
    params.drillRadius = aDrillRadius;
    params.barrelOuterRadius = aDrillRadius + 25e-6; // 25µm typical plating
    params.padRadius = aPadRadius;

    // Default antipad fallback: 1.5× drill diameter = 3× drill radius
    double antipadFallback = aDrillRadius * 3.0;

    // Find indices of top and bottom layers in our copper layer list
    int topIdx = -1;
    int bottomIdx = -1;

    for( int i = 0; i < (int) m_copperLayers.size(); i++ )
    {
        if( m_copperLayers[i].layerId == aTopLayer )
            topIdx = i;

        if( m_copperLayers[i].layerId == aBottomLayer )
            bottomIdx = i;
    }

    if( topIdx < 0 || bottomIdx < 0 || topIdx >= bottomIdx )
    {
        // Can't determine stackup geometry — return defaults
        params.barrelHeight = 1.6e-3;
        params.epsilonReff = 4.4;
        return params;
    }

    // Find signal layer index for stub calculation
    int signalIdx = -1;

    for( int i = 0; i < (int) m_copperLayers.size(); i++ )
    {
        if( m_copperLayers[i].layerId == aSignalLayer )
        {
            signalIdx = i;
            break;
        }
    }

    // Barrel height = sum of dielectric thicknesses between top and bottom layer
    double barrelH = 0.0;
    double erSum = 0.0;

    for( const DIELECTRIC_INFO& diel : m_dielectrics )
    {
        double dielCenter = ( diel.zTop + diel.zBottom ) / 2.0;
        double topZ = m_copperLayers[topIdx].zPosition;
        double botZ = m_copperLayers[bottomIdx].zPosition;

        if( dielCenter > topZ && dielCenter < botZ )
        {
            double thickness = diel.zBottom - diel.zTop;
            barrelH += thickness;
            erSum += diel.epsilonR * thickness;
        }
    }

    params.barrelHeight = barrelH;
    params.epsilonReff = ( barrelH > 0.0 ) ? erSum / barrelH : 4.4;

    // Via-specific reference plane detection: use "nearby" test because the via
    // position is inside the antipad void, not inside the zone fill copper.
    // The zone is still a reference plane — the antipad is just the clearance hole.
    static constexpr int VIA_PLANE_SEARCH_RADIUS = 2000000; // 2mm in nm

    // Plane crossings: each copper layer between top and bottom (exclusive)
    // that is a reference plane near this position
    for( int i = topIdx + 1; i < bottomIdx; i++ )
    {
        if( isReferencePlaneNearby( m_copperLayers[i].layerId, aPosition,
                                     VIA_PLANE_SEARCH_RADIUS ) )
        {
            VIA_PLANE_CROSSING pc;

            // Find dielectric around this plane
            for( const DIELECTRIC_INFO& diel : m_dielectrics )
            {
                double dielCenter = ( diel.zTop + diel.zBottom ) / 2.0;
                double planeZ = m_copperLayers[i].zPosition;

                // Use the dielectric closest to this plane
                if( std::abs( dielCenter - planeZ ) < 0.5e-3 )
                {
                    pc.epsilonR = diel.epsilonR;
                    pc.dielectricThickness = diel.zBottom - diel.zTop;
                    break;
                }
            }

            pc.antipadRadius = findAntipadRadius( m_copperLayers[i].layerId,
                                                   aPosition, antipadFallback );

            params.planeCrossings.push_back( pc );
        }
    }

    // Pad-to-plane capacitance: the via has a pad on every copper layer it spans.
    // Each non-reference-plane layer contributes C_pad to its nearest reference
    // plane above and below.  All contributions sum into C_total for the lumped
    // pi-model (no sectioning — valid up to f_max).
    for( int i = topIdx; i <= bottomIdx; i++ )
    {
        // Reference planes contribute barrel C (already in planeCrossings), not pad C
        if( i > topIdx && i < bottomIdx
            && isReferencePlaneNearby( m_copperLayers[i].layerId, aPosition,
                                        VIA_PLANE_SEARCH_RADIUS ) )
        {
            continue;
        }

        // Find nearest reference plane above this layer
        for( int j = i - 1; j >= topIdx; j-- )
        {
            if( !isReferencePlaneNearby( m_copperLayers[j].layerId, aPosition,
                                          VIA_PLANE_SEARCH_RADIUS ) )
                continue;

            for( const DIELECTRIC_INFO& diel : m_dielectrics )
            {
                double dielCenter = ( diel.zTop + diel.zBottom ) / 2.0;

                if( dielCenter > m_copperLayers[j].zPosition
                    && dielCenter < m_copperLayers[i].zPosition )
                {
                    VIA_PAD_PLANE pp;
                    pp.epsilonR = diel.epsilonR;
                    pp.thickness = diel.zBottom - diel.zTop;
                    pp.antipadRadius = findAntipadRadius( m_copperLayers[j].layerId,
                                                           aPosition, antipadFallback );
                    params.adjacentPlanes.push_back( pp );
                    break;
                }
            }

            break; // nearest plane above only
        }

        // Find nearest reference plane below this layer
        for( int j = i + 1; j <= bottomIdx; j++ )
        {
            if( !isReferencePlaneNearby( m_copperLayers[j].layerId, aPosition,
                                          VIA_PLANE_SEARCH_RADIUS ) )
                continue;

            for( const DIELECTRIC_INFO& diel : m_dielectrics )
            {
                double dielCenter = ( diel.zTop + diel.zBottom ) / 2.0;

                if( dielCenter > m_copperLayers[i].zPosition
                    && dielCenter < m_copperLayers[j].zPosition )
                {
                    VIA_PAD_PLANE pp;
                    pp.epsilonR = diel.epsilonR;
                    pp.thickness = diel.zBottom - diel.zTop;
                    pp.antipadRadius = findAntipadRadius( m_copperLayers[j].layerId,
                                                           aPosition, antipadFallback );
                    params.adjacentPlanes.push_back( pp );
                    break;
                }
            }

            break; // nearest plane below only
        }
    }

    // Stub length: barrel below the signal exit layer
    if( signalIdx >= 0 && signalIdx < bottomIdx )
    {
        double stubH = 0.0;

        for( const DIELECTRIC_INFO& diel : m_dielectrics )
        {
            double dielCenter = ( diel.zTop + diel.zBottom ) / 2.0;
            double sigZ = m_copperLayers[signalIdx].zPosition;
            double botZ = m_copperLayers[bottomIdx].zPosition;

            if( dielCenter > sigZ && dielCenter < botZ )
                stubH += diel.zBottom - diel.zTop;
        }

        params.stubLength = stubH;
    }

    return params;
}
