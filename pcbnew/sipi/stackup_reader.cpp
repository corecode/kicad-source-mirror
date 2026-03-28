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
#include "analytical_impedance.h"

#include <board.h>
#include <board_design_settings.h>
#include <board_stackup_manager/board_stackup.h>
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
            // Track solder mask thickness for z accumulation but don't store
            z += item->GetThickness() * NM_TO_M;
        }
    }
}


bool STACKUP_READER::isReferencePlane( PCB_LAYER_ID aLayer, const VECTOR2I& aPosition ) const
{
    // A layer is a reference plane at a given position if there is a filled zone
    // on that layer covering the position.
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

    // Search upward for the nearest copper layer that is a reference plane
    for( int i = signalIdx - 1; i >= 0; i-- )
    {
        if( isReferencePlane( m_copperLayers[i].layerId, aPosition ) )
        {
            geom.hAbove = copperSpacing( i, signalIdx );
            geom.hasRefAbove = true;
            findDielectric( m_copperLayers[i].zPosition, signalZ,
                            geom.erAbove, geom.tanDAbove );
            break;
        }
    }

    // Search downward for the nearest copper layer that is a reference plane
    for( int i = signalIdx + 1; i < (int) m_copperLayers.size(); i++ )
    {
        if( isReferencePlane( m_copperLayers[i].layerId, aPosition ) )
        {
            geom.hBelow = copperSpacing( signalIdx, i );
            geom.hasRefBelow = true;
            findDielectric( signalZ, m_copperLayers[i].zPosition,
                            geom.erBelow, geom.tanDBelow );
            break;
        }
    }

    // Fallback: if no zone-confirmed reference plane, assume adjacent copper
    // layers are planes. This is the common case for boards without zone fills
    // or for inner layers where the adjacent layer is always a plane.
    if( !geom.hasRefAbove && !geom.hasRefBelow )
    {
        // No zone-verified planes at all — use nearest copper layers as assumed planes
        if( signalIdx > 0 )
        {
            geom.hAbove = copperSpacing( signalIdx - 1, signalIdx );
            findDielectric( m_copperLayers[signalIdx - 1].zPosition, signalZ,
                            geom.erAbove, geom.tanDAbove );
        }

        if( signalIdx < (int) m_copperLayers.size() - 1 )
        {
            geom.hBelow = copperSpacing( signalIdx, signalIdx + 1 );
            findDielectric( signalZ, m_copperLayers[signalIdx + 1].zPosition,
                            geom.erBelow, geom.tanDBelow );
        }
    }
    else if( !geom.hasRefAbove && signalIdx > 0 )
    {
        geom.hAbove = copperSpacing( signalIdx - 1, signalIdx );
        findDielectric( m_copperLayers[signalIdx - 1].zPosition, signalZ,
                        geom.erAbove, geom.tanDAbove );
    }
    else if( !geom.hasRefBelow && signalIdx < (int) m_copperLayers.size() - 1 )
    {
        geom.hBelow = copperSpacing( signalIdx, signalIdx + 1 );
        findDielectric( signalZ, m_copperLayers[signalIdx + 1].zPosition,
                        geom.erBelow, geom.tanDBelow );
    }

    return geom;
}


double STACKUP_READER::ComputeZ0( const LAYER_GEOMETRY& aGeom )
{
    if( aGeom.traceWidth <= 0.0 )
        return 0.0;

    bool hasAbove = ( aGeom.hAbove > 0.0 );
    bool hasBelow = ( aGeom.hBelow > 0.0 );

    if( hasAbove && hasBelow )
    {
        // Stripline (two reference planes)
        if( std::abs( aGeom.hAbove - aGeom.hBelow ) < 0.01e-3
            && std::abs( aGeom.erAbove - aGeom.erBelow ) < 0.1 )
        {
            // Symmetric stripline
            return ANALYTICAL_IMPEDANCE::StriplineZ0( aGeom.traceWidth, aGeom.hAbove,
                                                      aGeom.erAbove, aGeom.traceThickness );
        }
        else
        {
            // Asymmetric stripline
            return ANALYTICAL_IMPEDANCE::AsymmetricStriplineZ0(
                    aGeom.traceWidth, aGeom.hAbove, aGeom.hBelow,
                    aGeom.erAbove, aGeom.erBelow, aGeom.traceThickness );
        }
    }
    else if( hasBelow )
    {
        // Microstrip (trace on top, ground below)
        return ANALYTICAL_IMPEDANCE::MicrostripZ0( aGeom.traceWidth, aGeom.hBelow,
                                                   aGeom.erBelow, aGeom.traceThickness );
    }
    else if( hasAbove )
    {
        // Inverted microstrip (trace on bottom, ground above)
        return ANALYTICAL_IMPEDANCE::MicrostripZ0( aGeom.traceWidth, aGeom.hAbove,
                                                   aGeom.erAbove, aGeom.traceThickness );
    }

    return 0.0; // No reference plane found
}
