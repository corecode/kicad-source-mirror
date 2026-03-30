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
#include <algorithm>

static constexpr double EPS0 = 8.854187817e-12;    // Vacuum permittivity (F/m)
static constexpr double C_LIGHT = 299792458.0;      // Speed of light (m/s)
static constexpr double TWO_PI_EPS0 = 2.0 * M_PI * EPS0;


BEM_2D_SOLVER::BEM_2D_SOLVER() :
        m_panelsPerEdge( 10 ),
        m_h( 0.0 )
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


double BEM_2D_SOLVER::getEpsilonR( double aY, double aNormalY ) const
{
    // Look slightly in the normal direction to find the region this panel faces.
    // For horizontal faces, aNormalY != 0 determines above/below.
    // For vertical (side) faces, aNormalY == 0, so use the region at aY directly.

    if( std::abs( aNormalY ) > 0.5 )
    {
        // Horizontal face: look in the normal direction
        double probeY = aY + aNormalY * 1e-9;

        for( const DIELECTRIC_BOUNDARY& db : m_dielectricBoundaries )
        {
            // Boundary at db.y: above has db.epsAbove, below has db.epsBelow.
            // If probe is just above a boundary, use epsAbove.
            // If probe is just below a boundary, use epsBelow.
            // We want the region that contains probeY.
            if( aNormalY > 0 && aY <= db.y + 1e-9 && probeY >= db.y - 1e-9 )
                return db.epsAbove; // face looks upward past this boundary

            if( aNormalY < 0 && aY >= db.y - 1e-9 && probeY <= db.y + 1e-9 )
                return db.epsBelow; // face looks downward past this boundary
        }

        // No boundary crossed: must be in air (above all dielectrics) or below ground
        return 1.0;
    }

    // Side face (vertical): find which dielectric region contains this y
    for( size_t i = 0; i + 1 < m_dielectricBoundaries.size(); i++ )
    {
        if( aY >= m_dielectricBoundaries[i].y && aY <= m_dielectricBoundaries[i + 1].y )
            return m_dielectricBoundaries[i].epsAbove;
    }

    return 1.0; // air
}


void BEM_2D_SOLVER::buildPanels()
{
    m_conductorPanels.clear();
    m_interfacePanels.clear();
    m_dielectricBoundaries.clear();

    // Map input geometry to spec coordinates (Section 2):
    //   Spec: ground at y = -h, interface at y = 0, conductors at y > 0
    //
    // Input geometry may have any arrangement of groundY and interface position.
    // We compute the signed offset to transform input y → spec y, such that
    // the interface maps to y=0 and conductors end up at positive y.

    double yInterfaceIn = 0.0;
    double yGroundIn = m_geometry.groundY;

    if( m_geometry.dielectrics.size() >= 2 )
    {
        yInterfaceIn = m_geometry.dielectrics[0].yBottom;
    }
    else if( !m_geometry.conductors.empty() )
    {
        // No explicit dielectric regions: infer interface position from conductor
        // and ground geometry.  Place it at the conductor face nearest the ground.
        const XS_CONDUCTOR& c0 = m_geometry.conductors[0];
        double faceA = c0.centerY - c0.thickness / 2.0;
        double faceB = c0.centerY + c0.thickness / 2.0;

        if( std::abs( faceA - yGroundIn ) < std::abs( faceB - yGroundIn ) )
            yInterfaceIn = faceA;
        else
            yInterfaceIn = faceB;
    }

    m_h = std::abs( yGroundIn - yInterfaceIn ); // substrate thickness (always positive)

    if( m_h < 1e-9 )
        return; // degenerate geometry

    // Determine which direction is "up" (away from ground, toward conductors).
    // Conductors should end up at y > 0 in spec coordinates.
    // The ground is on the substrate side of the interface.
    double ySign = 1.0; // +1 if input y increases away from ground, -1 otherwise

    if( !m_geometry.conductors.empty() )
    {
        double condY = m_geometry.conductors[0].centerY;

        // Conductor is on the opposite side of the interface from the ground
        if( ( condY - yInterfaceIn ) * ( yGroundIn - yInterfaceIn ) > 0 )
            ySign = -1.0; // conductor and ground are on the same side → flip
    }

    // Transform: spec_y = ySign * (input_y - yInterfaceIn)
    // This puts interface at y=0, and conductor at positive y.
    auto toSpecY = [&]( double inputY ) { return ySign * ( inputY - yInterfaceIn ); };

    // Verify: ground should map to negative y
    double specGround = toSpecY( yGroundIn );

    if( specGround > 0 )
        ySign = -ySign; // fix if our guess was wrong

    // Now specGround should be at y = -h
    specGround = toSpecY( yGroundIn );

    // --- Build dielectric boundary list ---
    // Each dielectric region has yTop, yBottom, epsilonR.  We find every
    // y-level where εr changes and record the εr on each side.

    if( m_geometry.dielectrics.size() >= 2 )
    {
        // Transform each region boundary to spec coordinates and collect
        // all unique y-levels with the εr on each side.
        struct REGION_SPEC
        {
            double yLow, yHigh;
            double er;
        };

        std::vector<REGION_SPEC> regions;

        for( const XS_DIELECTRIC_REGION& dr : m_geometry.dielectrics )
        {
            double y1 = toSpecY( dr.yTop );
            double y2 = toSpecY( dr.yBottom );
            double yLow = std::min( y1, y2 );
            double yHigh = std::max( y1, y2 );

            if( yHigh - yLow > 1e-12 )
                regions.push_back( { yLow, yHigh, dr.epsilonR } );
        }

        // Sort by yLow
        std::sort( regions.begin(), regions.end(),
                   []( const REGION_SPEC& a, const REGION_SPEC& b )
                   { return a.yLow < b.yLow; } );

        // At each boundary between adjacent regions, or at the top of the
        // topmost region (transition to air), create a dielectric boundary.
        for( size_t i = 0; i < regions.size(); i++ )
        {
            // Boundary at the top of this region
            double yBound = regions[i].yHigh;
            double erBelow = regions[i].er;
            double erAbove = 1.0; // default: air above

            // Check if another region starts at this boundary
            for( size_t j = 0; j < regions.size(); j++ )
            {
                if( j != i && std::abs( regions[j].yLow - yBound ) < 1e-12 )
                {
                    erAbove = regions[j].er;
                    break;
                }
            }

            // Only create boundary if εr actually changes
            if( std::abs( erAbove - erBelow ) > 1e-6 )
                m_dielectricBoundaries.push_back( { yBound, erAbove, erBelow } );
        }

        // Sort boundaries by y
        std::sort( m_dielectricBoundaries.begin(), m_dielectricBoundaries.end(),
                   []( const DIELECTRIC_BOUNDARY& a, const DIELECTRIC_BOUNDARY& b )
                   { return a.y < b.y; } );
    }
    // When no dielectric regions are specified, the medium is uniform.
    // No interface panels are needed — the Solve() method handles this by
    // scaling the air-only capacitance by the global εr.

    // Panel spacing for interface panels per spec Section 3.1
    double spacingI = m_h / 5.0;

    // Build conductor panels (spec Section 3.2)
    for( int ci = 0; ci < (int) m_geometry.conductors.size(); ci++ )
    {
        const XS_CONDUCTOR& cond = m_geometry.conductors[ci];

        double cx = cond.centerX;
        double cy = toSpecY( cond.centerY );
        double hw = cond.width / 2.0;
        double ht = cond.thickness / 2.0;

        // Face extents
        double xLeft = cx - hw;
        double xRight = cx + hw;
        double yBottom = cy - ht;  // should be ~0 for conductor on interface
        double yTop = cy + ht;

        // Panel counts per face based on user-specified panels-per-edge
        int nHoriz = std::clamp( m_panelsPerEdge, 4, 50 );
        int nVert = std::clamp( (int) round( m_panelsPerEdge * cond.thickness / cond.width ),
                                2, 20 );

        // Bottom face: y = yBottom, normal = (0, -1)
        for( int i = 0; i < nHoriz; i++ )
        {
            double t = ( i + 0.5 ) / nHoriz;

            PANEL p = {};
            p.cx = xLeft + t * cond.width;
            p.cy = yBottom;
            p.length = cond.width / nHoriz;
            p.nx = 0.0;
            p.ny = -1.0;
            p.conductorIdx = ci;
            p.type = CONDUCTOR;
            p.epsilonR = getEpsilonR( yBottom, -1.0 );
            m_conductorPanels.push_back( p );
        }

        // Top face: y = yTop, normal = (0, +1)
        for( int i = 0; i < nHoriz; i++ )
        {
            double t = ( i + 0.5 ) / nHoriz;

            PANEL p = {};
            p.cx = xLeft + t * cond.width;
            p.cy = yTop;
            p.length = cond.width / nHoriz;
            p.nx = 0.0;
            p.ny = 1.0;
            p.conductorIdx = ci;
            p.type = CONDUCTOR;
            p.epsilonR = getEpsilonR( yTop, 1.0 );
            m_conductorPanels.push_back( p );
        }

        // Left face: x = xLeft, normal = (-1, 0)
        for( int i = 0; i < nVert; i++ )
        {
            double t = ( i + 0.5 ) / nVert;

            PANEL p = {};
            p.cx = xLeft;
            p.cy = yBottom + t * cond.thickness;
            p.length = cond.thickness / nVert;
            p.nx = -1.0;
            p.ny = 0.0;
            p.conductorIdx = ci;
            p.type = CONDUCTOR;
            p.epsilonR = getEpsilonR( p.cy, 0.0 );
            m_conductorPanels.push_back( p );
        }

        // Right face: x = xRight, normal = (+1, 0)
        for( int i = 0; i < nVert; i++ )
        {
            double t = ( i + 0.5 ) / nVert;

            PANEL p = {};
            p.cx = xRight;
            p.cy = yBottom + t * cond.thickness;
            p.length = cond.thickness / nVert;
            p.nx = 1.0;
            p.ny = 0.0;
            p.conductorIdx = ci;
            p.type = CONDUCTOR;
            p.epsilonR = getEpsilonR( p.cy, 0.0 );
            m_conductorPanels.push_back( p );
        }
    }

    // Build interface panels at each dielectric boundary
    if( m_dielectricBoundaries.empty() )
        return;

    // Find conductor footprint extents (for excluding panels at each y-level)
    struct FOOTPRINT
    {
        double xLeft, xRight, yBottom, yTop;
    };

    std::vector<FOOTPRINT> footprints;
    double xMinAll = 1e9, xMaxAll = -1e9;

    for( const XS_CONDUCTOR& cond : m_geometry.conductors )
    {
        double cx = cond.centerX;
        double cy = toSpecY( cond.centerY );
        double hw = cond.width / 2.0;
        double ht = cond.thickness / 2.0;
        footprints.push_back( { cx - hw, cx + hw, cy - ht, cy + ht } );
        xMinAll = std::min( xMinAll, cx - hw );
        xMaxAll = std::max( xMaxAll, cx + hw );
    }

    double extent = 5.0 * m_h;
    double xStart = xMinAll - extent;
    double xEnd = xMaxAll + extent;
    int maxInterfacePanels = 500; // safety cap

    for( const DIELECTRIC_BOUNDARY& db : m_dielectricBoundaries )
    {
        double x = xStart;

        while( x < xEnd && (int) m_interfacePanels.size() < maxInterfacePanels )
        {
            double xMid = x + spacingI / 2.0;

            if( xMid >= xEnd )
                break;

            // Check if this panel overlaps any conductor face at this y-level
            bool inFootprint = false;

            for( const FOOTPRINT& fp : footprints )
            {
                if( xMid >= fp.xLeft && xMid <= fp.xRight
                    && db.y >= fp.yBottom - 1e-9 && db.y <= fp.yTop + 1e-9 )
                {
                    inFootprint = true;
                    break;
                }
            }

            if( !inFootprint )
            {
                PANEL p = {};
                p.cx = xMid;
                p.cy = db.y;
                p.length = spacingI;
                p.nx = 0.0;
                p.ny = 1.0; // normal = +ŷ (convention: plus side is above)
                p.conductorIdx = -1;
                p.type = INTERFACE;
                p.epsPlus = db.epsAbove;
                p.epsMinus = db.epsBelow;
                m_interfacePanels.push_back( p );
            }

            x += spacingI;
        }
    }
}


double BEM_2D_SOLVER::greenG( double x, double y, double xs, double ys ) const
{
    // Spec Section 4: G(r, r') = -1/(2πε₀) × [ln|r-r'| - ln|r-r''|]
    // where r'' = (xs, -2h - ys) is the ground-plane image.

    double dx = x - xs;
    double dy = y - ys;
    double dyImg = y - ( -2.0 * m_h - ys );

    double r2 = dx * dx + dy * dy;
    double rImg2 = dx * dx + dyImg * dyImg;

    if( r2 < 1e-30 )
        r2 = 1e-30;

    if( rImg2 < 1e-30 )
        rImg2 = 1e-30;

    return -1.0 / TWO_PI_EPS0 * ( 0.5 * log( r2 ) - 0.5 * log( rImg2 ) );
}


double BEM_2D_SOLVER::greenDGDn( double x, double y, double xs, double ys ) const
{
    // Spec Section 4: ∂G/∂n with n̂ = +ŷ (interface normal)
    // = -1/(2πε₀) × [(y-ys)/r² - (y-ys'')/rImg²]
    // where ys'' = -2h - ys

    double dx = x - xs;
    double dy = y - ys;
    double dyImg = y - ( -2.0 * m_h - ys );

    double r2 = dx * dx + dy * dy;
    double rImg2 = dx * dx + dyImg * dyImg;

    if( r2 < 1e-30 )
        r2 = 1e-30;

    if( rImg2 < 1e-30 )
        rImg2 = 1e-30;

    return -1.0 / TWO_PI_EPS0 * ( dy / r2 - dyImg / rImg2 );
}


double BEM_2D_SOLVER::selfIntegralG( double aLength ) const
{
    // Spec Section 5.1: analytic self-integral of the direct Green's function
    // ∫_{-L/2}^{L/2} -1/(2πε₀) × ln|s| ds = -L/(2πε₀) × (ln(L/2) - 1)

    return -aLength / TWO_PI_EPS0 * ( log( aLength / 2.0 ) - 1.0 );
}


Eigen::MatrixXd BEM_2D_SOLVER::solveCapacitanceFull()
{
    // Spec Sections 5 and 6: full system with conductor + interface panels.
    // Solve N times (once per conductor driven at 1V) to build the Maxwell C matrix.

    int numCond = (int) m_geometry.conductors.size();
    int nc = (int) m_conductorPanels.size();
    int ni = (int) m_interfacePanels.size();
    int n = nc + ni;

    Eigen::MatrixXd A( n, n );
    A.setZero();

    // Helper: get panel by global index
    auto panel = [&]( int idx ) -> const PANEL&
    {
        return ( idx < nc ) ? m_conductorPanels[idx] : m_interfacePanels[idx - nc];
    };

    // --- Conductor rows (spec Section 5.1) ---
    for( int i = 0; i < nc; i++ )
    {
        const PANEL& pi = m_conductorPanels[i];

        for( int j = 0; j < n; j++ )
        {
            const PANEL& pj = panel( j );

            if( i == j )
            {
                // Diagonal: analytic direct self-term + midpoint image term
                double imageY = -2.0 * m_h - pi.cy;
                double rImg = std::abs( pi.cy - imageY );
                double gImage = 1.0 / TWO_PI_EPS0 * 0.5 * log( rImg * rImg );

                A( i, j ) = selfIntegralG( pi.length ) + gImage * pi.length;
            }
            else
            {
                // Off-diagonal
                double dist2 = ( pi.cx - pj.cx ) * ( pi.cx - pj.cx )
                               + ( pi.cy - pj.cy ) * ( pi.cy - pj.cy );
                double threshold = 3.0 * std::max( pi.length, pj.length );

                if( sqrt( dist2 ) < threshold )
                {
                    // 4-point Gaussian quadrature for nearby panels
                    static const double gp[] = { -0.861136, -0.339981, 0.339981, 0.861136 };
                    static const double gw[] = { 0.347855, 0.652145, 0.652145, 0.347855 };

                    double sum = 0.0;

                    for( int q = 0; q < 4; q++ )
                    {
                        double xq = 0.0;
                        double yq = 0.0;

                        if( std::abs( pj.ny ) > 0.5 )
                        {
                            xq = pj.cx + gp[q] * pj.length / 2.0;
                            yq = pj.cy;
                        }
                        else
                        {
                            xq = pj.cx;
                            yq = pj.cy + gp[q] * pj.length / 2.0;
                        }

                        sum += gw[q] * greenG( pi.cx, pi.cy, xq, yq );
                    }

                    A( i, j ) = sum * pj.length / 2.0;
                }
                else
                {
                    A( i, j ) = greenG( pi.cx, pi.cy, pj.cx, pj.cy ) * pj.length;
                }
            }
        }
    }

    // --- Interface rows (spec Section 5.2) ---
    for( int ii = 0; ii < ni; ii++ )
    {
        int i = nc + ii;
        const PANEL& pi = m_interfacePanels[ii];

        double eps_plus = EPS0 * pi.epsPlus;
        double eps_minus = EPS0 * pi.epsMinus;

        for( int j = 0; j < n; j++ )
        {
            const PANEL& pj = panel( j );

            if( i == j )
            {
                double jumpTerm = -( eps_plus + eps_minus ) / ( 2.0 * EPS0 );
                double imageSelfDGDn = 1.0 / ( TWO_PI_EPS0 ) * ( 2.0 * m_h )
                                       / ( 4.0 * m_h * m_h );
                double imageSelfTerm = ( eps_plus - eps_minus ) * imageSelfDGDn * pi.length;

                A( i, j ) = jumpTerm + imageSelfTerm;
            }
            else if( pj.type == INTERFACE && std::abs( pj.cy - pi.cy ) < 1e-9 )
            {
                double dx = pi.cx - pj.cx;
                double denom = dx * dx + 4.0 * m_h * m_h;
                double dgdn_image = 1.0 / TWO_PI_EPS0 * ( 2.0 * m_h ) / denom;

                A( i, j ) = ( eps_plus - eps_minus ) * dgdn_image * pj.length;
            }
            else
            {
                A( i, j ) = ( eps_plus - eps_minus )
                            * greenDGDn( pi.cx, pi.cy, pj.cx, pj.cy ) * pj.length;
            }
        }
    }

    // Factorize once, solve for each conductor excitation
    Eigen::FullPivLU<Eigen::MatrixXd> lu( A );
    Eigen::MatrixXd Cmat( numCond, numCond );
    Cmat.setZero();

    for( int active = 0; active < numCond; active++ )
    {
        // RHS: 1V on conductor `active`, 0V on others, 0 for interface rows
        Eigen::VectorXd b = Eigen::VectorXd::Zero( n );

        for( int i = 0; i < nc; i++ )
            b( i ) = ( m_conductorPanels[i].conductorIdx == active ) ? 1.0 : 0.0;

        Eigen::VectorXd sigma = lu.solve( b );

        // Extract charge on each conductor, weighted by local εr
        for( int ci = 0; ci < numCond; ci++ )
        {
            double Q = 0.0;

            for( int i = 0; i < nc; i++ )
            {
                if( m_conductorPanels[i].conductorIdx == ci )
                    Q += m_conductorPanels[i].epsilonR * sigma( i )
                         * m_conductorPanels[i].length;
            }

            Cmat( ci, active ) = Q; // C_ij = Q_i when V_j = 1
        }
    }

    return Cmat;
}


Eigen::MatrixXd BEM_2D_SOLVER::solveCapacitanceAir()
{
    // Spec Section 7: reduced system, conductor panels only, vacuum Green's function

    int numCond = (int) m_geometry.conductors.size();
    int nc = (int) m_conductorPanels.size();

    Eigen::MatrixXd A( nc, nc );

    for( int i = 0; i < nc; i++ )
    {
        const PANEL& pi = m_conductorPanels[i];

        for( int j = 0; j < nc; j++ )
        {
            const PANEL& pj = m_conductorPanels[j];

            if( i == j )
            {
                double imageY = -2.0 * m_h - pi.cy;
                double rImg = std::abs( pi.cy - imageY );
                double gImage = 1.0 / TWO_PI_EPS0 * 0.5 * log( rImg * rImg );

                A( i, j ) = selfIntegralG( pi.length ) + gImage * pi.length;
            }
            else
            {
                A( i, j ) = greenG( pi.cx, pi.cy, pj.cx, pj.cy ) * pj.length;
            }
        }
    }

    Eigen::FullPivLU<Eigen::MatrixXd> lu( A );
    Eigen::MatrixXd Cmat( numCond, numCond );
    Cmat.setZero();

    for( int active = 0; active < numCond; active++ )
    {
        Eigen::VectorXd b = Eigen::VectorXd::Zero( nc );

        for( int i = 0; i < nc; i++ )
            b( i ) = ( m_conductorPanels[i].conductorIdx == active ) ? 1.0 : 0.0;

        Eigen::VectorXd sigma = lu.solve( b );

        for( int ci = 0; ci < numCond; ci++ )
        {
            double Q = 0.0;

            for( int i = 0; i < nc; i++ )
            {
                if( m_conductorPanels[i].conductorIdx == ci )
                    Q += sigma( i ) * m_conductorPanels[i].length;
            }

            Cmat( ci, active ) = Q;
        }
    }

    return Cmat;
}


bool BEM_2D_SOLVER::Solve()
{
    if( m_geometry.conductors.empty() )
        return false;

    buildPanels();

    if( m_conductorPanels.empty() )
        return false;

    int numCond = (int) m_geometry.conductors.size();

    // Solve the air-only system first (always needed).
    Eigen::MatrixXd Cair = solveCapacitanceAir();

    if( Cair( 0, 0 ) <= 0.0 )
        return false;

    // For uniform dielectric (no interface panels), scale C_air by εr.
    // Interface panels are only built when dielectric regions create boundaries.
    Eigen::MatrixXd Cfull;

    if( m_interfacePanels.empty() )
    {
        double er = std::max( m_geometry.epsilonR, 1.0 );
        Cfull = er * Cair;
    }
    else
    {
        Cfull = solveCapacitanceFull();

        if( Cfull( 0, 0 ) <= 0.0 )
            return false;
    }

    // Symmetrize: C must be symmetric by reciprocity; any asymmetry is numerical.
    m_result.C = ( Cfull + Cfull.transpose() ) / 2.0;
    m_result.C0 = ( Cair + Cair.transpose() ) / 2.0;

    // L = μ₀ε₀ × C₀⁻¹  (spec Section 7)
    static constexpr double MU0 = 4.0 * M_PI * 1e-7;

    m_result.L = MU0 * EPS0 * Cair.inverse();

    // Single-ended Z₀ from conductor 0: Z₀ = 1 / (c × √(C₁₁ × C₀₁₁))
    m_result.Z0 = 1.0 / ( C_LIGHT * sqrt( Cfull( 0, 0 ) * Cair( 0, 0 ) ) );
    m_result.erEff = Cfull( 0, 0 ) / Cair( 0, 0 );

    // Differential impedance for 2-conductor systems (even/odd mode analysis)
    if( numCond == 2 )
    {
        // Odd mode: conductor 0 at +1V, conductor 1 at -1V → C_odd = C₁₁ - C₁₂
        double Codd = Cfull( 0, 0 ) - Cfull( 0, 1 );
        double C0odd = Cair( 0, 0 ) - Cair( 0, 1 );

        if( Codd > 0.0 && C0odd > 0.0 )
        {
            double Zodd = 1.0 / ( C_LIGHT * sqrt( Codd * C0odd ) );
            m_result.Zdiff = 2.0 * Zodd;
        }
    }

    return true;
}
