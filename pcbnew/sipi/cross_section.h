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

#ifndef CROSS_SECTION_H
#define CROSS_SECTION_H

#include <Eigen/Dense>
#include <vector>


/**
 * A rectangular conductor in the 2D cross-section.
 * Coordinates: x is lateral, y is vertical (y=0 at top ground plane, positive downward).
 * All dimensions in meters.
 */
struct XS_CONDUCTOR
{
    double centerX = 0.0;
    double centerY = 0.0;
    double width = 0.0;
    double thickness = 0.0;
    bool   isGround = false;    ///< True for groundwire elements (V=0 in BEM)
};


/**
 * Cross-section geometry for the BEM solver.
 * Contains signal conductors and the reference plane configuration.
 */
/**
 * A dielectric region in the cross-section, spanning a y-range.
 * Regions are ordered top-to-bottom (increasing y downward).
 */
struct XS_DIELECTRIC_REGION
{
    double yTop = 0.0;        ///< Top of this region (meters, smaller y)
    double yBottom = 0.0;     ///< Bottom of this region (meters, larger y)
    double epsilonR = 1.0;    ///< Relative permittivity
};


/**
 * Cross-section geometry for the BEM solver.
 * Contains signal conductors, ground planes, and dielectric regions.
 *
 * Coordinate convention: y increases downward. Ground planes are horizontal.
 * For microstrip: ground at y=0, dielectric from 0 to -h (above ground),
 *                 conductor at y ≈ -h, air above.
 */
struct XS_GEOMETRY
{
    std::vector<XS_CONDUCTOR> conductors;   ///< Signal conductors (1 or 2 for diff pair)

    double groundY = 0.0;                   ///< Y position of the lower ground plane (meters)
    bool   hasUpperGround = false;          ///< True if there's a ground plane above
    double upperGroundY = 0.0;              ///< Y position of upper ground plane

    /// Dielectric regions. If empty, a single uniform dielectric is assumed.
    /// For microstrip: two regions — dielectric between ground and conductor,
    /// air above the conductor. The solver uses the region boundaries to
    /// construct the proper Green's function with dielectric images.
    std::vector<XS_DIELECTRIC_REGION> dielectrics;

    /// Fallback uniform εr used when dielectrics list is empty.
    double epsilonR = 4.4;
};


/**
 * Per-unit-length RLGC result from the BEM solver.
 * Matrices are NxN where N = number of signal conductors.
 */
struct RLGC_RESULT
{
    Eigen::MatrixXd C;      ///< Capacitance per unit length (F/m)
    Eigen::MatrixXd C0;     ///< Vacuum capacitance per unit length (F/m)
    Eigen::MatrixXd L;      ///< Inductance per unit length (H/m)
    double          Z0 = 0.0;     ///< Characteristic impedance (Ohms) — single-ended
    double          Zdiff = 0.0;     ///< Differential impedance (Ohms) — for 2-conductor
    double          erEff = 0.0;     ///< Effective dielectric constant
    double          erEffOdd = 0.0;  ///< Odd-mode effective εr (for diff pair delay)
};


#endif // CROSS_SECTION_H
