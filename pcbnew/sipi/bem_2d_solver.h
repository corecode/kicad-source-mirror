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
 * 2D Boundary Element Method solver for per-unit-length capacitance and
 * inductance of PCB transmission line cross-sections.
 *
 * Solves Laplace's equation (∇²Φ = 0) using constant-charge boundary elements
 * on the conductor surfaces. Ground planes are handled by the method of images
 * (no meshing of the planes themselves).
 *
 * Phase A: single conductor over one ground plane (microstrip).
 * Phase B (future): stripline (image series), coupled conductors, R(f), G(f).
 */
class BEM_2D_SOLVER
{
public:
    BEM_2D_SOLVER();

    void SetGeometry( const XS_GEOMETRY& aGeometry );

    /**
     * Number of constant-charge panels per edge of each conductor.
     * Total panels per conductor = 4 * panelsPerEdge (for a rectangle).
     * Default 10 (40 panels per conductor).
     */
    void SetPanelsPerEdge( int aCount );

    /**
     * Solve for per-unit-length C and L matrices, and derive Z0.
     * @return true on success.
     */
    bool Solve();

    const RLGC_RESULT& GetResult() const { return m_result; }

private:
    /**
     * A constant-charge boundary element (panel) on a conductor surface.
     */
    struct PANEL
    {
        double cx, cy;      ///< Center of the panel
        double length;      ///< Length of the panel
        double nx, ny;      ///< Outward normal direction
        int    conductorIdx; ///< Which conductor this panel belongs to
    };

    void buildPanels();
    void fillCoefficientMatrix( bool aVacuum );
    Eigen::MatrixXd solveCapacitance( bool aVacuum );

    /**
     * Green's function for a line charge in the presence of ground plane(s)
     * and (optionally) dielectric interfaces.
     *
     * @param aVacuum  If true, use εr=1 everywhere (no dielectric images).
     */
    double greenFunction( double x, double y, double xs, double ys, bool aVacuum ) const;

    XS_GEOMETRY             m_geometry;
    RLGC_RESULT             m_result;
    int                     m_panelsPerEdge;

    std::vector<PANEL>      m_panels;
    Eigen::MatrixXd         m_coeffMatrix;
};

#endif // BEM_2D_SOLVER_H
