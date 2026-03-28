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

#ifndef ANALYTICAL_IMPEDANCE_H
#define ANALYTICAL_IMPEDANCE_H


/**
 * Closed-form impedance formulas for PCB transmission lines.
 *
 * All dimensions in meters. All results in Ohms.
 * These are the standard IPC-2141 / Wadell formulas, accurate to ~5%
 * for typical PCB geometries. Used as the fast-path (no field solver)
 * and as a reference to validate the BEM solver.
 */
namespace ANALYTICAL_IMPEDANCE
{

/**
 * Microstrip effective dielectric constant (Hammerstad-Jensen).
 *
 * @param aW   Trace width (m)
 * @param aH   Dielectric height to ground plane (m)
 * @param aEr  Relative permittivity of the dielectric
 * @return Effective relative permittivity
 */
double MicrostripEffectiveEr( double aW, double aH, double aEr );

/**
 * Microstrip characteristic impedance (IPC-2141 / Hammerstad-Jensen).
 *
 * @param aW   Trace width (m)
 * @param aH   Dielectric height to ground plane (m)
 * @param aEr  Relative permittivity of the dielectric
 * @param aT   Trace thickness (m), 0 for zero-thickness approximation
 * @return Z0 in Ohms
 */
double MicrostripZ0( double aW, double aH, double aEr, double aT = 0.0 );

/**
 * Symmetric stripline characteristic impedance.
 * Trace centered between two ground planes, each at distance h from the trace center.
 *
 * @param aW   Trace width (m)
 * @param aH   Distance from trace center to each ground plane (m) — half the total spacing
 * @param aEr  Relative permittivity (uniform dielectric assumed)
 * @param aT   Trace thickness (m)
 * @return Z0 in Ohms
 */
double StriplineZ0( double aW, double aH, double aEr, double aT = 0.0 );

/**
 * Asymmetric stripline characteristic impedance.
 * Trace between two ground planes at different distances.
 *
 * @param aW      Trace width (m)
 * @param aH1     Distance from trace to nearest ground plane (m)
 * @param aH2     Distance from trace to farther ground plane (m)
 * @param aEr1    Relative permittivity between trace and nearest plane
 * @param aEr2    Relative permittivity between trace and farther plane
 * @param aT      Trace thickness (m)
 * @return Approximate Z0 in Ohms
 */
double AsymmetricStriplineZ0( double aW, double aH1, double aH2,
                              double aEr1, double aEr2, double aT = 0.0 );

/**
 * Propagation delay per unit length for a given effective dielectric constant.
 *
 * @param aErEff  Effective relative permittivity
 * @return Delay in seconds per meter
 */
double PropagationDelay( double aErEff );

} // namespace ANALYTICAL_IMPEDANCE

#endif // ANALYTICAL_IMPEDANCE_H
