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

#include "bem_2d_solver.h"

#include <cmath>

// Physical constants
static constexpr double EPS0 = 8.854187817e-12;    // Vacuum permittivity (F/m)
static constexpr double MU0 = 4.0e-7 * M_PI;       // Vacuum permeability (H/m)


BEM_2D_SOLVER::BEM_2D_SOLVER() :
        m_panelsPerEdge( 10 )
{
}


void BEM_2D_SOLVER::SetGeometry( const XS_GEOMETRY& aGeometry )
{
    m_geometry = aGeometry;
}


void BEM_2D_SOLVER::SetPanelsPerEdge( int aCount )
{
    m_panelsPerEdge = std::max( aCount, 4 );
}


void BEM_2D_SOLVER::buildPanels()
{
    m_panels.clear();

    int n = m_panelsPerEdge;

    for( int ci = 0; ci < (int) m_geometry.conductors.size(); ci++ )
    {
        const XS_CONDUCTOR& cond = m_geometry.conductors[ci];

        double hw = cond.width / 2.0;       // half-width
        double ht = cond.thickness / 2.0;   // half-thickness
        double cx = cond.centerX;
        double cy = cond.centerY;

        // Bottom edge: y = cy + ht, x from cx-hw to cx+hw, normal = (0, +1)
        for( int i = 0; i < n; i++ )
        {
            double t = ( i + 0.5 ) / n;
            PANEL p;
            p.cx = cx - hw + t * cond.width;
            p.cy = cy + ht;
            p.length = cond.width / n;
            p.nx = 0.0;
            p.ny = 1.0;
            p.conductorIdx = ci;
            m_panels.push_back( p );
        }

        // Top edge: y = cy - ht, x from cx-hw to cx+hw, normal = (0, -1)
        for( int i = 0; i < n; i++ )
        {
            double t = ( i + 0.5 ) / n;
            PANEL p;
            p.cx = cx - hw + t * cond.width;
            p.cy = cy - ht;
            p.length = cond.width / n;
            p.nx = 0.0;
            p.ny = -1.0;
            p.conductorIdx = ci;
            m_panels.push_back( p );
        }

        // Left edge: x = cx - hw, y from cy-ht to cy+ht, normal = (-1, 0)
        for( int i = 0; i < n; i++ )
        {
            double t = ( i + 0.5 ) / n;
            PANEL p;
            p.cx = cx - hw;
            p.cy = cy - ht + t * cond.thickness;
            p.length = cond.thickness / n;
            p.nx = -1.0;
            p.ny = 0.0;
            p.conductorIdx = ci;
            m_panels.push_back( p );
        }

        // Right edge: x = cx + hw, y from cy-ht to cy+ht, normal = (+1, 0)
        for( int i = 0; i < n; i++ )
        {
            double t = ( i + 0.5 ) / n;
            PANEL p;
            p.cx = cx + hw;
            p.cy = cy - ht + t * cond.thickness;
            p.length = cond.thickness / n;
            p.nx = 1.0;
            p.ny = 0.0;
            p.conductorIdx = ci;
            m_panels.push_back( p );
        }
    }
}


/**
 * Helper: log of squared distance, clamped to avoid log(0).
 */
static double logR2( double dx, double dy )
{
    double r2 = dx * dx + dy * dy;

    if( r2 < 1e-30 )
        r2 = 1e-30;

    return log( r2 );
}


double BEM_2D_SOLVER::greenFunction( double x, double y, double xs, double ys,
                                     bool aVacuum ) const
{
    //
    // 2D Green's function: potential at (x,y) from unit line charge at (xs,ys).
    //
    // Handles:
    // 1. PEC ground plane at y = groundY via method of images
    // 2. Dielectric interface: image series with reflection coefficient k,
    //    alternating between ground and interface reflections. The series
    //    decays as k^n and converges for |k| < 1 (always true for εr > 0).
    //
    // The image positions for ground at y_g, interface at y_i, source at y_s:
    //   spacing = y_g - y_i
    //
    //   n=0: ground image at 2*y_g - y_s, weight -1
    //   n≥1, group A: weight  k^n at y_s - 2n*sp
    //                 weight -k^n at 2n*sp - y_s + 2*y_g  [reflected form]
    //   n≥1, group B: weight  k^n at -2n*sp - y_s + 2*y_g [= 2*y_g - y_s - 2n*sp]
    //                 weight -k^n at y_s + 2n*sp
    //
    // where k = (ε_source_region - ε_other_region) / (ε_source + ε_other)
    //
    double dx = x - xs;
    double yg = m_geometry.groundY;

    // Determine the local εr at the source point (for the denominator).
    // Both source and observation are on conductor surfaces, which are all
    // in the same medium. The Green's function uses this medium's εr.
    double localEr = 1.0;

    if( !aVacuum )
    {
        localEr = m_geometry.epsilonR; // fallback uniform

        for( const XS_DIELECTRIC_REGION& reg : m_geometry.dielectrics )
        {
            if( ys >= reg.yTop && ys <= reg.yBottom )
            {
                localEr = reg.epsilonR;
                break;
            }
        }
    }

    double invEps = 1.0 / ( 4.0 * M_PI * EPS0 * localEr );

    // Direct source + ground plane image
    double potential = -logR2( dx, y - ys ) * invEps;                  // source
    potential += logR2( dx, y - ( 2.0 * yg - ys ) ) * invEps;         // ground image (weight -1)

    // Dielectric interface images: alternating reflections between ground
    // and interface produce image charges with weights ±k^n.
    //
    // For a single interface between dielectrics[0] (top) and dielectrics[1] (bottom):
    //   Interface at y = y_i = dielectrics[0].yBottom
    //   k = (ε_top - ε_bottom) / (ε_top + ε_bottom)
    //   spacing sp = y_g - y_i
    //
    //   For n = 1, 2, ...:
    //     Group A: +k^n at y_s - 2n·sp,  -k^n at (2·y_g - y_s) + 2n·sp
    //     Group B: +k^n at (2·y_g - y_s) - 2n·sp,  -k^n at y_s + 2n·sp

    if( !aVacuum && m_geometry.dielectrics.size() >= 2 )
    {
        double yInterface = m_geometry.dielectrics[0].yBottom;

        double erTop = m_geometry.dielectrics[0].epsilonR;
        double erBot = m_geometry.dielectrics[1].epsilonR;

        double k = ( erTop - erBot ) / ( erTop + erBot );
        double sp = yg - yInterface;

        if( sp > 0.0 )
        {
            double yGndImg = 2.0 * yg - ys;
            double kn = k;

            for( int n = 1; n <= 25; n++ )
            {
                double shift = 2.0 * n * sp;

                // Group A
                potential -= logR2( dx, y - ( ys - shift ) ) * invEps * kn;
                potential += logR2( dx, y - ( yGndImg + shift ) ) * invEps * kn;

                // Group B
                potential -= logR2( dx, y - ( yGndImg - shift ) ) * invEps * kn;
                potential += logR2( dx, y - ( ys + shift ) ) * invEps * kn;

                kn *= k;

                if( std::abs( kn ) < 1e-12 )
                    break;
            }
        }
    }

    // Upper ground plane image series (for stripline — no dielectric interface case)
    if( m_geometry.hasUpperGround && m_geometry.dielectrics.size() < 2 )
    {
        double yUpper = m_geometry.upperGroundY;
        double spacing = yg - yUpper;

        if( spacing > 0 )
        {
            for( int n = 1; n <= 20; n++ )
            {
                double shift = 2.0 * n * spacing;

                potential += logR2( dx, y - ( 2.0 * yUpper - ys + shift ) ) * invEps;
                potential += logR2( dx, y - ( 2.0 * yg - ys - shift ) ) * invEps;
                potential -= logR2( dx, y - ( ys - shift ) ) * invEps;
                potential -= logR2( dx, y - ( ys + shift ) ) * invEps;
            }
        }
    }

    return potential;
}


void BEM_2D_SOLVER::fillCoefficientMatrix( bool aVacuum )
{
    int nb = m_panels.size();
    m_coeffMatrix.resize( nb, nb );

    for( int i = 0; i < nb; i++ )
    {
        for( int j = 0; j < nb; j++ )
        {
            if( i == j )
            {
                // Self-interaction: evaluate greenFunction at a small normal
                // offset from center to get all image terms correctly, then
                // replace the singular direct term with the analytical integral.
                //
                // Analytical self-integral of -ln(r)/(2πε) over a panel of length L:
                //   A_ii^{direct} = L × (-ln(L/2) + 1) / (2πε)

                double L = m_panels[j].length;
                double cx = m_panels[j].cx;
                double cy = m_panels[j].cy;

                // Offset along the panel normal (1% of panel length)
                double offset = L * 0.01;
                double ox = cx + m_panels[j].nx * offset;
                double oy = cy + m_panels[j].ny * offset;

                // Full Green's function at offset (includes direct + all images)
                double gAtOffset = greenFunction( ox, oy, cx, cy, aVacuum );

                // Direct term at the offset distance: -ln(r²)/(4πε) = -ln(offset²)/(4πε)
                double localEr = 1.0;

                if( !aVacuum )
                {
                    localEr = m_geometry.epsilonR;

                    for( const XS_DIELECTRIC_REGION& reg : m_geometry.dielectrics )
                    {
                        if( cy >= reg.yTop && cy <= reg.yBottom )
                        {
                            localEr = reg.epsilonR;
                            break;
                        }
                    }
                }

                double invEps = 1.0 / ( 4.0 * M_PI * EPS0 * localEr );
                double directAtOffset = -logR2( ox - cx, oy - cy ) * invEps;

                // Analytical self-integral (replaces the direct term)
                double selfIntegral = L * ( -log( L / 2.0 ) + 1.0 ) * 2.0 * invEps;

                // A_ii = L × G(offset) - L × G_direct(offset) + self_integral
                // = L × G_images(offset) + self_integral
                m_coeffMatrix( i, j ) = L * gAtOffset - L * directAtOffset + selfIntegral;
            }
            else
            {
                // Off-diagonal: standard center-to-center evaluation
                m_coeffMatrix( i, j ) = m_panels[j].length
                                        * greenFunction( m_panels[i].cx, m_panels[i].cy,
                                                         m_panels[j].cx, m_panels[j].cy,
                                                         aVacuum );
            }
        }
    }
}


Eigen::MatrixXd BEM_2D_SOLVER::solveCapacitance( bool aVacuum )
{
    int numConductors = m_geometry.conductors.size();
    int nb = m_panels.size();

    fillCoefficientMatrix( aVacuum );

    Eigen::MatrixXd C( numConductors, numConductors );
    C.setZero();

    // For each conductor j, set it to 1V and all others to 0V, then solve for
    // the charge distribution. Total charge on conductor i gives C_ij.
    Eigen::PartialPivLU<Eigen::MatrixXd> lu( m_coeffMatrix );

    for( int j = 0; j < numConductors; j++ )
    {
        // RHS: potential = 1V on conductor j, 0V on others
        Eigen::VectorXd rhs( nb );

        for( int k = 0; k < nb; k++ )
            rhs( k ) = ( m_panels[k].conductorIdx == j ) ? 1.0 : 0.0;

        // Solve for charge density sigma on each panel
        Eigen::VectorXd sigma = lu.solve( rhs );

        // Sum charge on each conductor: Q_i = sum of sigma_k * L_k for panels on conductor i
        for( int k = 0; k < nb; k++ )
        {
            int ci = m_panels[k].conductorIdx;
            C( ci, j ) += sigma( k ) * m_panels[k].length;
        }
    }

    return C;
}


bool BEM_2D_SOLVER::Solve()
{
    if( m_geometry.conductors.empty() )
        return false;

    buildPanels();

    if( m_panels.empty() )
        return false;

    int numConductors = m_geometry.conductors.size();

    // Step 1: Solve with physical dielectric → capacitance matrix C
    Eigen::MatrixXd C = solveCapacitance( false );

    // Step 2: Solve with vacuum (eps_r = 1, no dielectric images) → C0
    // Then L = mu0*eps0 * C0^{-1}
    Eigen::MatrixXd C0 = solveCapacitance( true );
    Eigen::MatrixXd L = MU0 * EPS0 * C0.inverse();

    m_result.C = C;
    m_result.L = L;

    // Derive impedance
    if( numConductors == 1 )
    {
        // Single-ended impedance: Z0 = sqrt(L/C)
        double Lval = L( 0, 0 );
        double Cval = C( 0, 0 );

        if( Cval > 0.0 && Lval > 0.0 )
        {
            m_result.Z0 = sqrt( Lval / Cval );
            m_result.erEff = C( 0, 0 ) / C0( 0, 0 );
        }
    }
    else if( numConductors == 2 )
    {
        // Differential impedance from odd-mode:
        // Z_odd = sqrt(L_odd / C_odd)
        // In the Maxwell capacitance matrix, C12 < 0 (= -C_mutual).
        // Odd-mode: C_odd = C11 - C12 = C11 + |C12|  (coupling adds capacitance)
        // Odd-mode: L_odd = L11 - L12  (opposing currents reduce inductance)
        double L_odd = L( 0, 0 ) - L( 0, 1 );
        double C_odd = C( 0, 0 ) - C( 0, 1 );

        if( C_odd > 0.0 && L_odd > 0.0 )
        {
            double Z_odd = sqrt( L_odd / C_odd );
            m_result.Zdiff = 2.0 * Z_odd;
        }

        // Single-ended Z0 from self terms
        if( C( 0, 0 ) > 0.0 && L( 0, 0 ) > 0.0 )
        {
            m_result.Z0 = sqrt( L( 0, 0 ) / C( 0, 0 ) );
            m_result.erEff = C( 0, 0 ) / C0( 0, 0 );
        }
    }

    return true;
}
