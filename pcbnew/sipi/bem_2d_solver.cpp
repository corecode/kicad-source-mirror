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


double BEM_2D_SOLVER::greenFunction( double x, double y, double xs, double ys,
                                     double aEpsilonR ) const
{
    // 2D Green's function for Laplace's equation: G = -ln(r) / (2*pi*eps)
    //
    // With method of images for ground plane(s):
    // - Lower ground at y = groundY: image charge at y_image = 2*groundY - ys
    //   (reflected across the ground plane)
    // - Upper ground at y = upperGroundY: image series converges in ~20 terms
    //
    // For microstrip (single ground plane below):
    //   G = -1/(2*pi*eps) * [ln(r) - ln(r_image)]
    //   where r is distance to source, r_image is distance to image

    double eps = EPS0 * aEpsilonR;

    // Distance to the real source
    double dx = x - xs;
    double dy = y - ys;
    double r2 = dx * dx + dy * dy;

    // Avoid log(0) for self-interaction — use panel half-length as regularization
    if( r2 < 1e-30 )
        r2 = 1e-30;

    double potential = -log( r2 ) / ( 4.0 * M_PI * eps );
    // Note: log(r^2)/(4*pi) = log(r)/(2*pi)

    // Image in the lower ground plane
    double yImage = 2.0 * m_geometry.groundY - ys;
    double dyImage = y - yImage;
    double rImage2 = dx * dx + dyImage * dyImage;

    if( rImage2 < 1e-30 )
        rImage2 = 1e-30;

    // Image has opposite sign → subtract
    potential -= -log( rImage2 ) / ( 4.0 * M_PI * eps );

    if( m_geometry.hasUpperGround )
    {
        // For stripline: image series between two parallel ground planes.
        // Images alternate between reflections in upper and lower planes.
        // The series converges quickly (20 terms is more than enough).
        double hLower = m_geometry.groundY;
        double hUpper = m_geometry.upperGroundY;
        double spacing = hLower - hUpper; // Distance between planes (positive)

        for( int n = 1; n <= 20; n++ )
        {
            // Images from reflections in both planes, alternating signs
            // Even images (same sign as original): at ys + 2*n*spacing and ys - 2*n*spacing
            // Odd images (opposite sign): reflected across each plane

            // Image in upper plane, order n
            double yImgUp = 2.0 * n * spacing + ( 2.0 * hUpper - ys );
            double dyUp = y - yImgUp;
            double rUp2 = dx * dx + dyUp * dyUp;

            if( rUp2 > 1e-30 )
                potential -= -log( rUp2 ) / ( 4.0 * M_PI * eps ); // opposite sign image

            // Image in lower plane, order n
            double yImgDown = -2.0 * n * spacing + ( 2.0 * hLower - ys );
            double dyDown = y - yImgDown;
            double rDown2 = dx * dx + dyDown * dyDown;

            if( rDown2 > 1e-30 )
                potential -= -log( rDown2 ) / ( 4.0 * M_PI * eps );

            // Even-order images (same sign)
            double yEvenUp = ys - 2.0 * n * spacing;
            double dyEU = y - yEvenUp;
            double rEU2 = dx * dx + dyEU * dyEU;

            if( rEU2 > 1e-30 )
                potential += -log( rEU2 ) / ( 4.0 * M_PI * eps );

            double yEvenDown = ys + 2.0 * n * spacing;
            double dyED = y - yEvenDown;
            double rED2 = dx * dx + dyED * dyED;

            if( rED2 > 1e-30 )
                potential += -log( rED2 ) / ( 4.0 * M_PI * eps );
        }
    }

    return potential;
}


void BEM_2D_SOLVER::fillCoefficientMatrix( double aEpsilonR )
{
    int nb = m_panels.size();
    m_coeffMatrix.resize( nb, nb );

    for( int i = 0; i < nb; i++ )
    {
        for( int j = 0; j < nb; j++ )
        {
            // Potential at panel i center due to unit charge density on panel j
            // For constant-charge elements: phi_i = sum_j (sigma_j * L_j * G(r_i, r_j))
            // The coefficient matrix entry is L_j * G(center_i, center_j)
            m_coeffMatrix( i, j ) = m_panels[j].length
                                    * greenFunction( m_panels[i].cx, m_panels[i].cy,
                                                     m_panels[j].cx, m_panels[j].cy,
                                                     aEpsilonR );
        }
    }
}


Eigen::MatrixXd BEM_2D_SOLVER::solveCapacitance( double aEpsilonR )
{
    int numConductors = m_geometry.conductors.size();
    int nb = m_panels.size();

    fillCoefficientMatrix( aEpsilonR );

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
    Eigen::MatrixXd C = solveCapacitance( m_geometry.epsilonR );

    // Step 2: Solve with vacuum (eps_r = 1) → C0, then L = mu0*eps0 * C0^{-1}
    Eigen::MatrixXd C0 = solveCapacitance( 1.0 );
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
        // L_odd = L11 - L12,  C_odd = C11 + C12 (note: C12 is negative in the C matrix)
        double L_odd = L( 0, 0 ) - L( 0, 1 );
        double C_odd = C( 0, 0 ) + C( 0, 1 ); // C01 is typically negative

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
