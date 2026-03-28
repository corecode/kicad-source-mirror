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

#ifndef STACKUP_READER_H
#define STACKUP_READER_H

#include <layer_ids.h>
#include <math/vector2d.h>
#include <vector>

class BOARD;


/**
 * Geometry of the cross-section at a point on a trace, describing
 * the signal layer and its reference planes.
 */
struct LAYER_GEOMETRY
{
    PCB_LAYER_ID signalLayer = UNDEFINED_LAYER;

    double traceWidth = 0.0;        ///< meters
    double traceThickness = 0.0;    ///< meters

    double hAbove = 0.0;            ///< meters, dielectric thickness to ref plane above
    double hBelow = 0.0;            ///< meters, dielectric thickness to ref plane below
    double erAbove = 4.5;           ///< relative permittivity above
    double erBelow = 4.5;           ///< relative permittivity below
    double tanDAbove = 0.02;        ///< loss tangent above
    double tanDBelow = 0.02;        ///< loss tangent below

    bool hasRefAbove = false;       ///< true if a copper plane covers the trace above
    bool hasRefBelow = false;       ///< true if a copper plane covers the trace below

    bool usingDefaults = false;     ///< true if stackup data was default/unconfigured
};


/**
 * Reads the board stackup and determines the cross-section geometry
 * (dielectric heights, εr, reference planes) for a trace on any layer.
 */
class STACKUP_READER
{
public:
    STACKUP_READER( const BOARD* aBoard );

    /**
     * Get the cross-section geometry for a trace at a specific position and layer.
     *
     * @param aLayer       Copper layer the trace is on
     * @param aPosition    Board position to test for reference plane coverage
     * @param aTraceWidth  Width of the trace (nm, converted to meters internally)
     * @return LAYER_GEOMETRY with all fields populated
     */
    LAYER_GEOMETRY GetLayerGeometry( PCB_LAYER_ID aLayer, const VECTOR2I& aPosition,
                                    int aTraceWidth );

    /**
     * Compute Z₀ for the given layer geometry using analytical formulas.
     * Automatically selects microstrip or stripline based on reference planes.
     *
     * @return Impedance in Ohms, or 0 if geometry is invalid
     */
    static double ComputeZ0( const LAYER_GEOMETRY& aGeom );

private:
    struct COPPER_LAYER_INFO
    {
        PCB_LAYER_ID layerId;
        double       zPosition;   ///< meters from top surface, positive downward
        double       thickness;   ///< meters
    };

    struct DIELECTRIC_INFO
    {
        double zTop;         ///< meters
        double zBottom;      ///< meters
        double epsilonR;
        double lossTangent;
    };

    void buildLayerModel();
    bool isReferencePlane( PCB_LAYER_ID aLayer, const VECTOR2I& aPosition ) const;

    const BOARD* m_board;

    std::vector<COPPER_LAYER_INFO> m_copperLayers;  ///< ordered top to bottom
    std::vector<DIELECTRIC_INFO>   m_dielectrics;   ///< between copper layers
    bool                           m_usingDefaults;
    bool                           m_built;
};

#endif // STACKUP_READER_H
