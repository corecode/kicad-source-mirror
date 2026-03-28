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
};


/**
 * Cross-section geometry for the BEM solver.
 * Contains signal conductors and the reference plane configuration.
 */
struct XS_GEOMETRY
{
    std::vector<XS_CONDUCTOR> conductors;   ///< Signal conductors (1 or 2 for diff pair)

    double groundY = 0.0;                   ///< Y position of the ground plane (meters)
    bool   hasUpperGround = false;          ///< True if there's a ground plane above
    double upperGroundY = 0.0;              ///< Y position of upper ground plane

    double epsilonR = 4.4;                  ///< Relative permittivity of the dielectric
};


/**
 * Per-unit-length RLGC result from the BEM solver.
 * Matrices are NxN where N = number of signal conductors.
 */
struct RLGC_RESULT
{
    Eigen::MatrixXd C;      ///< Capacitance per unit length (F/m)
    Eigen::MatrixXd L;      ///< Inductance per unit length (H/m)
    double          Z0 = 0.0;     ///< Characteristic impedance (Ohms) — single-ended
    double          Zdiff = 0.0;  ///< Differential impedance (Ohms) — for 2-conductor
    double          erEff = 0.0;  ///< Effective dielectric constant
};


#endif // CROSS_SECTION_H
