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

#include "analytical_impedance.h"

#include <cmath>

// Speed of light in vacuum (m/s)
static constexpr double C0 = 299792458.0;


double ANALYTICAL_IMPEDANCE::MicrostripEffectiveEr( double aW, double aH, double aEr )
{
    // Hammerstad-Jensen formula for effective dielectric constant
    // Valid for w/h > 0.05
    double u = aW / aH;

    double er_eff = ( aEr + 1.0 ) / 2.0
                    + ( aEr - 1.0 ) / 2.0 * pow( 1.0 + 12.0 / u, -0.5 );

    return er_eff;
}


double ANALYTICAL_IMPEDANCE::MicrostripZ0( double aW, double aH, double aEr, double aT )
{
    // IPC-2141 Section 4.2.2, based on Hammerstad-Jensen
    //
    // For zero thickness:
    //   Z0 = (60/sqrt(er_eff)) * ln(8h/w + w/(4h))     for w/h <= 1
    //   Z0 = (120*pi) / (sqrt(er_eff) * (w/h + 1.393 + 0.667*ln(w/h + 1.444)))  for w/h > 1
    //
    // Thickness correction: replace w with effective width we = w + dt
    //   dt = (t/pi) * (1 + ln(4*pi*w/t))   for w/h >= 0.5/pi
    //   dt = (t/pi) * (1 + ln(2*h/t))      for w/h < 0.5/pi

    double w = aW;

    // Apply thickness correction if t > 0
    if( aT > 0.0 && aH > 0.0 )
    {
        double dt;

        if( w / aH >= 1.0 / ( 2.0 * M_PI ) )
            dt = ( aT / M_PI ) * ( 1.0 + log( 4.0 * M_PI * w / aT ) );
        else
            dt = ( aT / M_PI ) * ( 1.0 + log( 2.0 * aH / aT ) );

        w += dt;
    }

    double er_eff = MicrostripEffectiveEr( w, aH, aEr );
    double u = w / aH;
    double z0;

    if( u <= 1.0 )
    {
        z0 = ( 60.0 / sqrt( er_eff ) ) * log( 8.0 / u + u / 4.0 );
    }
    else
    {
        z0 = ( 120.0 * M_PI )
             / ( sqrt( er_eff ) * ( u + 1.393 + 0.667 * log( u + 1.444 ) ) );
    }

    return z0;
}


double ANALYTICAL_IMPEDANCE::StriplineZ0( double aW, double aH, double aEr, double aT )
{
    // IPC-2141 Section 4.3.1 symmetric stripline formula
    // aH is the distance from trace center to each ground plane.
    // The (0.8*w + t) term accounts for finite trace thickness.
    //
    // Z0 = (60 / sqrt(er)) * ln(4 * b / (0.67 * pi * (0.8 * w + t)))
    //
    // where b = 2*h (total ground-to-ground spacing).
    // For zero thickness, use t = 0 and the 0.8 factor still applies.

    if( aW <= 0.0 || aH <= 0.0 || aEr <= 0.0 )
        return 0.0;

    double b = 2.0 * aH;
    double denom = 0.67 * M_PI * ( 0.8 * aW + aT );

    if( denom <= 0.0 )
        return 0.0;

    double z0 = ( 60.0 / sqrt( aEr ) ) * log( 4.0 * b / denom );

    // Clamp to zero if geometry yields nonsensical result
    return ( z0 > 0.0 ) ? z0 : 0.0;
}


double ANALYTICAL_IMPEDANCE::AsymmetricStriplineZ0( double aW, double aH1, double aH2,
                                                    double aEr1, double aEr2, double aT )
{
    // Approximate asymmetric stripline as the parallel combination of
    // two microstrip impedances (Pozar approximation):
    //   1/Z0 ≈ 1/Z0_upper + 1/Z0_lower
    //
    // More precisely, use weighted effective Er and effective height:
    //   er_eff = (er1 * h1 + er2 * h2) / (h1 + h2)  (thickness-weighted)
    //   h_eff  = (h1 + h2) / 2                       (average half-spacing)

    double hTotal = aH1 + aH2;

    if( hTotal <= 0.0 || aW <= 0.0 )
        return 0.0;

    double erEff = ( aEr1 * aH1 + aEr2 * aH2 ) / hTotal;

    // Use symmetric stripline formula with effective parameters
    double hHalf = hTotal / 2.0;

    return StriplineZ0( aW, hHalf, erEff, aT );
}


double ANALYTICAL_IMPEDANCE::PropagationDelay( double aErEff )
{
    // tpd = sqrt(er_eff) / c0   [seconds per meter]
    if( aErEff <= 0.0 )
        return 0.0;

    return sqrt( aErEff ) / C0;
}
