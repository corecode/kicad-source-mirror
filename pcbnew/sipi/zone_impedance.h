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

#ifndef ZONE_IMPEDANCE_H
#define ZONE_IMPEDANCE_H

#include <math/vector2d.h>
#include <vector>

class SHAPE_LINE_CHAIN;


/**
 * A single spoke in the zone star model, connecting a pad/via to the zone centroid.
 */
struct ZONE_SPOKE
{
    int    connectionIndex;  ///< Index into the connection point array
    double resistance;       ///< Spoke resistance in Ohms
    double inductance;       ///< Spoke inductance in Henries
    double squares;          ///< Number of squares along the spoke path
};


/**
 * Star-topology impedance model for a copper zone.
 *
 * Each connection point (pad, via) gets a spoke to the zone centroid.
 * The spoke impedance is computed by ray-integrating the sheet impedance
 * from the connection point to the centroid.
 */
struct ZONE_STAR_MODEL
{
    VECTOR2I                 centroid;   ///< Zone centroid position (nm)
    double                   area_m2;    ///< Zone area in square meters
    std::vector<ZONE_SPOKE>  spokes;     ///< One spoke per connection point
};


/**
 * Computes star-topology impedance models for copper zones.
 *
 * Uses ray integration of sheet impedance to estimate the resistance
 * and inductance of each spoke from a connection point to the zone centroid.
 */
class ZONE_IMPEDANCE
{
public:
    /**
     * Build a star model for a zone polygon with the given connection points.
     *
     * @param aOutline       Zone fill polygon outline (closed SHAPE_LINE_CHAIN, nm coords)
     * @param aConnections   Connection point positions (pads/vias inside the zone, nm)
     * @param aRsheet        Sheet resistance in Ohms/square (rho / t_copper)
     * @param aLsheet        Sheet inductance in H/square (mu0 * h_dielectric)
     * @return Star model with spoke R/L for each connection
     */
    static ZONE_STAR_MODEL Build( const SHAPE_LINE_CHAIN& aOutline,
                                  const std::vector<VECTOR2I>& aConnections,
                                  double aRsheet, double aLsheet );

    /**
     * Compute the number of squares along a path from aFrom to aTo through a polygon.
     *
     * Integrates ds/w(s) along the straight-line path, where w(s) is the perpendicular
     * width of the polygon at each sample point.  Uses the outline intersection method:
     * at each sample, cast a perpendicular ray and measure where it crosses the outline.
     *
     * @param aOutline  Closed polygon outline (nm coordinates)
     * @param aFrom     Start point (must be inside or on the polygon)
     * @param aTo       End point (must be inside or on the polygon)
     * @param aSamples  Number of integration samples along the path
     * @return Number of squares (dimensionless), or 1.0 if degenerate
     */
    static double RayIntegrateSquares( const SHAPE_LINE_CHAIN& aOutline,
                                       const VECTOR2I& aFrom, const VECTOR2I& aTo,
                                       int aSamples = 16 );
};

#endif // ZONE_IMPEDANCE_H
