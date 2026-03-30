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

#ifndef BEM_2D_SOLVER_H
#define BEM_2D_SOLVER_H

#include "cross_section.h"

#include <Eigen/Dense>
#include <vector>


/**
 * 2D sub-region BEM solver for per-unit-length capacitance and impedance
 * of PCB microstrip cross-sections.
 *
 * Uses the vacuum+ground-image Green's function with explicit interface
 * panels that enforce D-normal continuity at the dielectric boundary.
 * The dielectric effect emerges from bound charge on the interface —
 * no dielectric images are used.
 *
 * Follows the formulation in subregion_bem_spec_rev4.md.
 *
 * Coordinate system (matching the spec):
 *   y = 0: dielectric interface
 *   y = -h: ground plane (Φ = 0)
 *   y > 0: air (εr = 1), conductors sit here with bottom face at y = 0
 *   y < 0: substrate (εr configurable)
 */
class BEM_2D_SOLVER
{
public:
    BEM_2D_SOLVER();

    void SetGeometry( const XS_GEOMETRY& aGeometry );
    void SetPanelsPerEdge( int aCount );

    bool Solve();

    const RLGC_RESULT& GetResult() const { return m_result; }

private:
    enum PANEL_TYPE
    {
        CONDUCTOR,
        INTERFACE
    };

    struct PANEL
    {
        double     cx, cy;        ///< Midpoint
        double     length;        ///< Panel length
        double     nx, ny;        ///< Outward normal
        int        conductorIdx;  ///< Which conductor (-1 for interface)
        PANEL_TYPE type;

        // Per-panel dielectric properties (following NMMTL convention)
        double     epsilonR;      ///< CONDUCTOR panels: εr of medium this face sees
        double     epsPlus;       ///< INTERFACE panels: εr on the +n̂ side (above)
        double     epsMinus;      ///< INTERFACE panels: εr on the -n̂ side (below)
    };

    /// Build conductor and interface panels per the spec
    void buildPanels();

    /// Vacuum Green's function with ground-plane image: G(r, r')
    double greenG( double x, double y, double xs, double ys ) const;

    /// Normal derivative ∂G/∂n at (x, y) due to source at (xs, ys), with n̂ = (0, +1)
    double greenDGDn( double x, double y, double xs, double ys ) const;

    /// Assemble and solve the full system (conductor + interface panels).
    /// Returns the NxN Maxwell capacitance matrix (N = number of conductors).
    Eigen::MatrixXd solveCapacitanceFull();

    /// Assemble and solve the air-only system (conductor panels only, no interface).
    /// Returns the NxN vacuum capacitance matrix.
    Eigen::MatrixXd solveCapacitanceAir();

    /// Analytic self-integral of the direct Green's function over a panel of length L
    double selfIntegralG( double aLength ) const;

    /// Look up the relative permittivity of the dielectric region at position y,
    /// on the side indicated by normalY (> 0 means look above y, < 0 means below).
    /// Returns 1.0 (air) if no dielectric region is found.
    double getEpsilonR( double aY, double aNormalY ) const;

    XS_GEOMETRY        m_geometry;
    RLGC_RESULT        m_result;
    int                m_panelsPerEdge;

    double             m_h;           ///< Distance from lowest interface to ground plane

    /// Dielectric boundaries in spec coordinates, sorted by y.
    /// Each entry: { y-level, εr above, εr below }.
    struct DIELECTRIC_BOUNDARY
    {
        double y;
        double epsAbove;    ///< εr of region above this boundary
        double epsBelow;    ///< εr of region below this boundary
    };
    std::vector<DIELECTRIC_BOUNDARY> m_dielectricBoundaries;

    std::vector<PANEL> m_conductorPanels;
    std::vector<PANEL> m_interfacePanels;
};

#endif // BEM_2D_SOLVER_H
