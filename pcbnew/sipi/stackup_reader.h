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

#include <sipi/via_model.h>

#include <layer_ids.h>
#include <math/vector2d.h>
#include <vector>

class BOARD;
class BOARD_ITEM;


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

    PCB_LAYER_ID refLayerAbove = UNDEFINED_LAYER;  ///< Copper layer used as ref above
    PCB_LAYER_ID refLayerBelow = UNDEFINED_LAYER;  ///< Copper layer used as ref below

    double refThicknessAbove = 0.035e-3; ///< Reference copper thickness above (meters)
    double refThicknessBelow = 0.035e-3; ///< Reference copper thickness below (meters)

    double signalZPosition = 0.0;   ///< Signal layer z-position (meters, from stackup)

    /// Original reference distance before demotion (0 if not demoted).
    /// When a reference layer is demoted to groundwires, hBelow/hAbove are
    /// pushed to a deeper layer but the original substrate εr and thickness
    /// are preserved here so BuildGeometry creates the correct dielectric layering.
    double hOrigBelow = 0.0;
    double erOrigBelow = 0.0;
    double hOrigAbove = 0.0;
    double erOrigAbove = 0.0;

    /// Fallback image ground for demotion: the outermost copper layer
    /// beyond the current reference.  Used when the reference is demoted.
    double hFallbackBelow = 0.0;    ///< Distance to outermost layer below (meters)
    double erFallbackBelow = 4.5;   ///< εr to outermost layer below
    double tanDFallbackBelow = 0.02;
    double hFallbackAbove = 0.0;
    double erFallbackAbove = 4.5;
    double tanDFallbackAbove = 0.02;

    /// Intermediate copper layers between the signal and the image ground
    /// that may have partial zone coverage (groundwire candidates).
    struct INTERMEDIATE_LAYER
    {
        PCB_LAYER_ID layerId;
        double       zPosition;   ///< meters from top surface
        double       thickness;   ///< meters
    };

    std::vector<INTERMEDIATE_LAYER> intermediateLayers;

    /// Solder mask on the air-facing side (outer layers only).
    /// Nonzero thickness means the signal layer has solder mask coating.
    double solderMaskThickness = 0.0;  ///< meters (0 = no solder mask)
    double solderMaskEr = 3.3;         ///< solder mask relative permittivity
    bool   solderMaskAbove = true;     ///< true = mask above signal, false = below
    bool   solderMaskIsDefault = true; ///< true if thickness is the KiCad default

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
     * Get via geometry parameters for the analytical via model.
     *
     * Computes barrel height (sum of dielectric thicknesses between top and bottom
     * layers), plane crossings with per-crossing εr and antipad radius, and adjacent
     * reference planes for pad capacitance.
     *
     * @param aTopLayer     Via top copper layer
     * @param aBottomLayer  Via bottom copper layer
     * @param aSignalLayer  The layer the signal enters/exits (for stub calculation)
     * @param aPosition     Board position (for antipad radius detection)
     * @param aDrillRadius  Drill radius (m) for antipad fallback
     * @param aPadRadius    Pad radius (m)
     * @return VIA_PARAMS with stackup fields populated (position/role not set)
     */
    VIA_PARAMS GetViaGeometry( PCB_LAYER_ID aTopLayer, PCB_LAYER_ID aBottomLayer,
                               PCB_LAYER_ID aSignalLayer, const VECTOR2I& aPosition,
                               double aDrillRadius, double aPadRadius );

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
    void buildMaskItemCache();
    bool isReferencePlane( PCB_LAYER_ID aLayer, const VECTOR2I& aPosition ) const;
    bool isReferencePlaneNearby( PCB_LAYER_ID aLayer, const VECTOR2I& aPosition,
                                 int aSearchRadius ) const;
    double findAntipadRadius( PCB_LAYER_ID aLayer, const VECTOR2I& aPosition,
                              double aFallback ) const;
    bool hasSolderMaskOpening( PCB_LAYER_ID aMaskLayer, const VECTOR2I& aPosition ) const;

    struct SOLDER_MASK_INFO
    {
        double thickness = 0.0;   ///< meters
        double epsilonR = 3.3;    ///< relative permittivity
        bool   present = false;
        bool   isDefault = true;  ///< true if thickness matches KiCad default
    };

    const BOARD* m_board;

    std::vector<COPPER_LAYER_INFO> m_copperLayers;  ///< ordered top to bottom
    std::vector<DIELECTRIC_INFO>   m_dielectrics;   ///< between copper layers
    SOLDER_MASK_INFO               m_solderMaskTop;     ///< above F.Cu
    SOLDER_MASK_INFO               m_solderMaskBottom;  ///< below B.Cu
    std::vector<BOARD_ITEM*>       m_maskItemsTop;      ///< items on F.Mask
    std::vector<BOARD_ITEM*>       m_maskItemsBottom;   ///< items on B.Mask
    bool                           m_usingDefaults;
    bool                           m_built;
};

#endif // STACKUP_READER_H
