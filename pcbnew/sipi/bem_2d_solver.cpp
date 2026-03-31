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

/**
 * @file bem_2d_solver.cpp
 *
 * 2D Galerkin BEM solver for per-unit-length RLGC parameters, ported from
 * the NMMTL (Numerical Multiconductor Transmission Line) formulation.
 *
 * Reference: TNT-MMTL, Mayo Foundation (K. Buchs, 1992).
 *
 * Key design choices matching NMMTL:
 *  - Green's function G = ln(d_image/d_direct), NO ε₀ in kernel
 *  - RHS (load vector) = ε₀ × V
 *  - Assembly constant = 1/(2π)
 *  - Interface diagonal uses mass matrix with length_scale factor
 *  - Interface off-diagonal uses (εr⁺ - εr⁻) × ∂G/∂n flux kernel
 *  - Charge extraction: Q = ∫ εr_local × σ × N dl (NMMTL convention)
 *  - Quadratic (3-node) Lagrangian elements
 *  - Galerkin weighted-residual (double integration)
 */

#include "bem_2d_solver.h"

#include <cmath>
#include <algorithm>
#include <map>


// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------
static constexpr double EPS0    = 8.854187817e-12;   // Vacuum permittivity (F/m)
static constexpr double C_LIGHT = 299792458.0;        // Speed of light (m/s)
static constexpr double TWO_PI  = 2.0 * M_PI;
static constexpr double INV_TWO_PI = 1.0 / ( 2.0 * M_PI );


// ---------------------------------------------------------------------------
// Gauss–Legendre quadrature on [0, 1]
// ---------------------------------------------------------------------------

// 10-point rule (assembly outer, load, charge)
static constexpr int GAUSS_N_OUTER = 10;
static const double GAUSS_PTS_10[10] = {
    0.01304673574141414, 0.06746831665550774, 0.16029521585048779,
    0.28330230293537640, 0.42556283050918439, 0.57443716949081561,
    0.71669769706462360, 0.83970478414951221, 0.93253168334449226,
    0.98695326425858586
};
static const double GAUSS_WTS_10[10] = {
    0.03333567215434407, 0.07472567457529029, 0.10954318125799103,
    0.13463335965499817, 0.14776211235737644, 0.14776211235737644,
    0.13463335965499817, 0.10954318125799103, 0.07472567457529029,
    0.03333567215434407
};

// 6-point rule (inner interval integration)
static constexpr int GAUSS_N_INNER = 6;
static const double GAUSS_PTS_6[6] = {
    0.03376524289842399, 0.16939530676686775, 0.38069040695840155,
    0.61930959304159845, 0.83060469323313225, 0.96623475710157601
};
static const double GAUSS_WTS_6[6] = {
    0.08566224618958517, 0.18038078652406930, 0.23395696728634553,
    0.23395696728634553, 0.18038078652406930, 0.08566224618958517
};


// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

BEM_2D_SOLVER::BEM_2D_SOLVER() :
        m_panelsPerEdge( 10 ),
        m_lengthScale( 0.0 ),
        m_numCondNodes( 0 ),
        m_numIntfNodes( 0 ),
        m_numConductors( 0 )
{
}


void BEM_2D_SOLVER::SetGeometry( const XS_GEOMETRY& aGeometry )
{
    m_geometry = aGeometry;
}


void BEM_2D_SOLVER::SetPanelsPerEdge( int aCount )
{
    m_panelsPerEdge = std::max( aCount, 3 );
}


// ---------------------------------------------------------------------------
// Shape functions — quadratic Lagrangian on [0,1]
// ---------------------------------------------------------------------------

void BEM_2D_SOLVER::shapeFunctions( double aXi, double aN[3] )
{
    double L1 = 1.0 - aXi;
    double L2 = aXi;

    aN[0] = L1 * ( 2.0 * L1 - 1.0 );   // node 0 (start)
    aN[1] = 4.0 * L1 * L2;              // node 1 (middle)
    aN[2] = L2 * ( 2.0 * L2 - 1.0 );   // node 2 (end)
}


void BEM_2D_SOLVER::shapeDerivatives( double aXi, double aDN[3] )
{
    double L1 = 1.0 - aXi;
    double L2 = aXi;

    aDN[0] = -4.0 * L1 + 1.0;
    aDN[1] =  4.0 * ( L1 - L2 );
    aDN[2] =  4.0 * L2 - 1.0;
}


double BEM_2D_SOLVER::jacobian( double aXi, const ELEMENT& aElem )
{
    double dN[3];
    shapeDerivatives( aXi, dN );

    double dx = 0.0, dy = 0.0;

    for( int i = 0; i < 3; i++ )
    {
        dx += dN[i] * aElem.xpts[i];
        dy += dN[i] * aElem.ypts[i];
    }

    return sqrt( dx * dx + dy * dy );
}


void BEM_2D_SOLVER::interpolate( double aXi, const ELEMENT& aElem,
                                  double& aX, double& aY )
{
    double N[3];
    shapeFunctions( aXi, N );

    aX = 0.0;
    aY = 0.0;

    for( int i = 0; i < 3; i++ )
    {
        aX += N[i] * aElem.xpts[i];
        aY += N[i] * aElem.ypts[i];
    }
}


// ---------------------------------------------------------------------------
// Green's function kernels — NO ε₀ (matching NMMTL)
//
// Ground plane at y = 0.  Image of source at (X, Y) is at (X, -Y).
//
// Potential kernel:  G = ln(d_image / d_direct)
//                      = 0.5 × ln( ((x-X)²+(y+Y)²) / ((x-X)²+(y-Y)²) )
//
// This gives G > 0 for sources above the ground plane and vanishes at y = 0.
// ---------------------------------------------------------------------------

double BEM_2D_SOLVER::greenPotential( double x, double y,
                                       double X, double Y ) const
{
    double dx = x - X;
    double dy_direct = y - Y;
    double dy_image  = y + Y;       // image at (X, -Y)

    double d2_direct = dx * dx + dy_direct * dy_direct;
    double d2_image  = dx * dx + dy_image  * dy_image;

    if( d2_direct < 1e-30 )
        d2_direct = 1e-30;

    if( d2_image < 1e-30 )
        d2_image = 1e-30;

    return 0.5 * log( d2_image / d2_direct );
}


double BEM_2D_SOLVER::greenFlux( double x, double y,
                                  double X, double Y,
                                  double nx, double ny ) const
{
    double dx = x - X;
    double dy_direct = y - Y;
    double dy_image  = y + Y;

    double d2_direct = dx * dx + dy_direct * dy_direct;
    double d2_image  = dx * dx + dy_image  * dy_image;

    if( d2_direct < 1e-30 )
        d2_direct = 1e-30;

    if( d2_image < 1e-30 )
        d2_image = 1e-30;

    // NMMTL convention: the flux kernel is (direct/d² - image/d²) · n̂,
    // NOT the gradient of G.  NMMTL stores A[source][obs] and relies on
    // the Fortran solver's column-major transpose to get A[obs][source].
    // With Eigen (row-major), we store A(obs, source) directly, so we
    // use the same kernel sign as NMMTL.

    double dGdx = dx / d2_direct - dx / d2_image;
    double dGdy = dy_direct / d2_direct - dy_image / d2_image;

    return dGdx * nx + dGdy * ny;
}


// ---------------------------------------------------------------------------
// Element generation
// ---------------------------------------------------------------------------

void BEM_2D_SOLVER::buildElements()
{
    m_condElements.clear();
    m_intfElements.clear();
    m_numCondNodes = 0;
    m_numIntfNodes = 0;
    m_numConductors = 0;

    if( m_geometry.conductors.empty() )
        return;

    // --- Coordinate transform ---
    // Map input geometry so that ground plane is at y = 0 and conductors
    // are at y > 0, matching the Green's function convention.

    double yGroundIn = m_geometry.groundY;
    double yInterfaceIn = 0.0;

    if( m_geometry.dielectrics.size() >= 2 )
    {
        yInterfaceIn = m_geometry.dielectrics[0].yBottom;
    }
    else if( !m_geometry.conductors.empty() )
    {
        const XS_CONDUCTOR& c0 = m_geometry.conductors[0];
        double faceA = c0.centerY - c0.thickness / 2.0;
        double faceB = c0.centerY + c0.thickness / 2.0;

        if( std::abs( faceA - yGroundIn ) < std::abs( faceB - yGroundIn ) )
            yInterfaceIn = faceA;
        else
            yInterfaceIn = faceB;
    }

    double h = std::abs( yGroundIn - yInterfaceIn );

    if( h < 1e-9 )
        return;

    // Determine sign so conductors end up at y > 0 in output coords.
    double ySign = 1.0;

    if( !m_geometry.conductors.empty() )
    {
        double condY = m_geometry.conductors[0].centerY;

        if( ( condY - yInterfaceIn ) * ( yGroundIn - yInterfaceIn ) > 0 )
            ySign = -1.0;
    }

    // Transform: output_y = ySign * (input_y - yInterfaceIn) + h
    // This puts ground at y=0 and interface at y=h.
    auto toY = [&]( double inputY )
    {
        return ySign * ( inputY - yInterfaceIn ) + h;
    };

    // Verify ground maps to y ≈ 0
    double groundY = toY( yGroundIn );

    if( std::abs( groundY ) > 1e-6 * h )
    {
        ySign = -ySign;
        groundY = toY( yGroundIn );
    }

    // --- Compute length_scale (NMMTL: half_minimum_dimension) ---
    double minDim = 1e9;

    for( const XS_CONDUCTOR& c : m_geometry.conductors )
    {
        minDim = std::min( minDim, c.width );
        minDim = std::min( minDim, c.thickness );
    }

    m_lengthScale = minDim / 2.0;

    // --- Build dielectric boundaries (same logic as before) ---
    struct DIELECTRIC_BOUNDARY
    {
        double y;
        double epsAbove;
        double epsBelow;
    };

    std::vector<DIELECTRIC_BOUNDARY> dielectricBoundaries;

    if( m_geometry.dielectrics.size() >= 2 )
    {
        struct REGION_SPEC { double yLow, yHigh, er; };

        std::vector<REGION_SPEC> regions;

        for( const XS_DIELECTRIC_REGION& dr : m_geometry.dielectrics )
        {
            double y1 = toY( dr.yTop );
            double y2 = toY( dr.yBottom );
            double yLow = std::min( y1, y2 );
            double yHigh = std::max( y1, y2 );

            if( yHigh - yLow > 1e-12 )
                regions.push_back( { yLow, yHigh, dr.epsilonR } );
        }

        std::sort( regions.begin(), regions.end(),
                   []( const REGION_SPEC& a, const REGION_SPEC& b )
                   { return a.yLow < b.yLow; } );

        for( size_t i = 0; i < regions.size(); i++ )
        {
            double yBound = regions[i].yHigh;
            double erBelow = regions[i].er;
            double erAbove = 1.0;

            for( size_t j = 0; j < regions.size(); j++ )
            {
                if( j != i && std::abs( regions[j].yLow - yBound ) < 1e-12 )
                {
                    erAbove = regions[j].er;
                    break;
                }
            }

            if( std::abs( erAbove - erBelow ) > 1e-6 )
                dielectricBoundaries.push_back( { yBound, erAbove, erBelow } );
        }

        std::sort( dielectricBoundaries.begin(), dielectricBoundaries.end(),
                   []( const DIELECTRIC_BOUNDARY& a, const DIELECTRIC_BOUNDARY& b )
                   { return a.y < b.y; } );
    }

    // --- Helper: find εr at a given y-level for a conductor face ---
    auto getEpsilonR = [&]( double aY, double aNormalY ) -> double
    {
        if( std::abs( aNormalY ) > 0.5 )
        {
            double probeY = aY + aNormalY * 1e-9;

            for( const DIELECTRIC_BOUNDARY& db : dielectricBoundaries )
            {
                if( aNormalY > 0 && aY <= db.y + 1e-9 && probeY >= db.y - 1e-9 )
                    return db.epsAbove;

                if( aNormalY < 0 && aY >= db.y - 1e-9 && probeY <= db.y + 1e-9 )
                    return db.epsBelow;
            }

            return 1.0;
        }

        for( size_t i = 0; i + 1 < dielectricBoundaries.size(); i++ )
        {
            if( aY >= dielectricBoundaries[i].y
                && aY <= dielectricBoundaries[i + 1].y )
                return dielectricBoundaries[i].epsAbove;
        }

        return 1.0;
    };

    // --- Build conductor elements ---
    // Node numbering: sequential, shared between adjacent elements on the same face.
    // Each face with N_elem elements has 2*N_elem + 1 nodes (quadratic).

    int nodeCounter = 0;

    // Helper to create elements along a straight line segment
    auto buildFace = [&]( double x0, double y0, double x1, double y1,
                          int nElem, double nx, double ny, int aCondIdx )
    {
        double er = getEpsilonR( ( y0 + y1 ) / 2.0, ny );

        for( int e = 0; e < nElem; e++ )
        {
            double t0 = (double) e / nElem;
            double t1 = (double) ( e + 1 ) / nElem;
            double tm = ( t0 + t1 ) / 2.0;

            ELEMENT el = {};
            el.xpts[0] = x0 + t0 * ( x1 - x0 );
            el.ypts[0] = y0 + t0 * ( y1 - y0 );
            el.xpts[1] = x0 + tm * ( x1 - x0 );
            el.ypts[1] = y0 + tm * ( y1 - y0 );
            el.xpts[2] = x0 + t1 * ( x1 - x0 );
            el.ypts[2] = y0 + t1 * ( y1 - y0 );
            el.conductorIdx = aCondIdx;
            el.epsilonR = er;

            if( e == 0 )
            {
                el.nodeIdx[0] = nodeCounter++;
                el.nodeIdx[1] = nodeCounter++;
                el.nodeIdx[2] = nodeCounter++;
            }
            else
            {
                el.nodeIdx[0] = m_condElements.back().nodeIdx[2]; // shared
                el.nodeIdx[1] = nodeCounter++;
                el.nodeIdx[2] = nodeCounter++;
            }

            m_condElements.push_back( el );
        }
    };

    // Signal conductor index counter (only non-ground conductors)
    int signalIdx = 0;

    for( int ci = 0; ci < (int) m_geometry.conductors.size(); ci++ )
    {
        const XS_CONDUCTOR& cond = m_geometry.conductors[ci];
        int condIdx = cond.isGround ? GROUND_CONDUCTOR_IDX : signalIdx++;

        double cx = cond.centerX;
        double cy = toY( cond.centerY );
        double hw = cond.width / 2.0;
        double ht = cond.thickness / 2.0;

        double xLeft  = cx - hw;
        double xRight = cx + hw;
        double yBot   = cy - ht;
        double yTop   = cy + ht;

        // Groundwires have smooth charge distributions — use fewer panels.
        int ppe = cond.isGround ? std::max( m_panelsPerEdge / 2, 4 ) : m_panelsPerEdge;
        int nHoriz = std::clamp( ppe, 3, 50 );
        int nVert  = std::clamp( (int) round( ppe * cond.thickness
                                               / cond.width ), 2, 20 );

        // Four faces: bottom, right, top (reversed), left (reversed)
        buildFace( xLeft, yBot, xRight, yBot, nHoriz, 0.0, -1.0, condIdx );
        buildFace( xRight, yBot, xRight, yTop, nVert, 1.0, 0.0, condIdx );
        buildFace( xRight, yTop, xLeft, yTop, nHoriz, 0.0, 1.0, condIdx );
        buildFace( xLeft, yTop, xLeft, yBot, nVert, -1.0, 0.0, condIdx );
    }

    m_numConductors = signalIdx;

    m_numCondNodes = nodeCounter;

    // --- Build interface elements ---
    if( dielectricBoundaries.empty() )
        return;

    // Conductor footprints for exclusion
    struct FOOTPRINT { double xLeft, xRight, yBot, yTop; };
    std::vector<FOOTPRINT> footprints;
    double xMinAll = 1e9, xMaxAll = -1e9;

    for( const XS_CONDUCTOR& cond : m_geometry.conductors )
    {
        double cx = cond.centerX;
        double cy = toY( cond.centerY );
        double hw = cond.width / 2.0;
        double ht = cond.thickness / 2.0;
        footprints.push_back( { cx - hw, cx + hw, cy - ht, cy + ht } );
        xMinAll = std::min( xMinAll, cx - hw );
        xMaxAll = std::max( xMaxAll, cx + hw );
    }

    double extent = 5.0 * h;
    double xStart = xMinAll - extent;
    double xEnd   = xMaxAll + extent;
    double spacingI = h / 5.0;

    for( const DIELECTRIC_BOUNDARY& db : dielectricBoundaries )
    {
        // Collect the x-ranges that are NOT under a conductor at this y-level
        // Build a list of free intervals by subtracting conductor footprints
        struct INTERVAL { double a, b; };
        std::vector<INTERVAL> free = { { xStart, xEnd } };

        for( const FOOTPRINT& fp : footprints )
        {
            if( db.y < fp.yBot - 1e-9 || db.y > fp.yTop + 1e-9 )
                continue;

            std::vector<INTERVAL> next;

            for( const INTERVAL& iv : free )
            {
                if( fp.xRight <= iv.a || fp.xLeft >= iv.b )
                {
                    next.push_back( iv );
                }
                else
                {
                    if( fp.xLeft > iv.a )
                        next.push_back( { iv.a, fp.xLeft } );

                    if( fp.xRight < iv.b )
                        next.push_back( { fp.xRight, iv.b } );
                }
            }

            free = next;
        }

        // Create quadratic elements along each free interval
        for( const INTERVAL& iv : free )
        {
            double len = iv.b - iv.a;

            if( len < spacingI * 0.5 )
                continue;

            int nElem = std::max( 1, (int) round( len / spacingI ) );

            for( int e = 0; e < nElem; e++ )
            {
                double t0 = (double) e / nElem;
                double t1 = (double) ( e + 1 ) / nElem;
                double tm = ( t0 + t1 ) / 2.0;

                ELEMENT el = {};
                el.xpts[0] = iv.a + t0 * len;
                el.ypts[0] = db.y;
                el.xpts[1] = iv.a + tm * len;
                el.ypts[1] = db.y;
                el.xpts[2] = iv.a + t1 * len;
                el.ypts[2] = db.y;
                el.conductorIdx = -1;
                el.epsPlus  = db.epsAbove;
                el.epsMinus = db.epsBelow;
                el.normalX  = 0.0;
                el.normalY  = 1.0;  // pointing up (toward air)

                if( e == 0 )
                {
                    el.nodeIdx[0] = nodeCounter++;
                    el.nodeIdx[1] = nodeCounter++;
                    el.nodeIdx[2] = nodeCounter++;
                }
                else
                {
                    el.nodeIdx[0] = m_intfElements.back().nodeIdx[2];
                    el.nodeIdx[1] = nodeCounter++;
                    el.nodeIdx[2] = nodeCounter++;
                }

                m_intfElements.push_back( el );
            }
        }
    }

    m_numIntfNodes = nodeCounter - m_numCondNodes;
}


// ---------------------------------------------------------------------------
// Inner interval integration — potential kernel
// ---------------------------------------------------------------------------

void BEM_2D_SOLVER::intervalConductor( double x, double y,
                                        const ELEMENT& aInner,
                                        double aValue[3] ) const
{
    aValue[0] = aValue[1] = aValue[2] = 0.0;

    for( int q = 0; q < GAUSS_N_INNER; q++ )
    {
        double xi = GAUSS_PTS_6[q];
        double N[3];
        shapeFunctions( xi, N );

        double X, Y;
        interpolate( xi, aInner, X, Y );

        double J = jacobian( xi, aInner );
        double G = greenPotential( x, y, X, Y );

        for( int i = 0; i < 3; i++ )
            aValue[i] += GAUSS_WTS_6[q] * N[i] * G * J;
    }
}


void BEM_2D_SOLVER::intervalSelfConductor( double x, double y,
                                            const ELEMENT& aElem,
                                            double aValue[3],
                                            double aXiOuter ) const
{
    // Split the integration at the singularity point aXiOuter.
    // Integrate [0, aXiOuter] and [aXiOuter, 1] separately.
    aValue[0] = aValue[1] = aValue[2] = 0.0;

    auto integrate = [&]( double a, double b )
    {
        if( b - a < 1e-12 )
            return;

        for( int q = 0; q < GAUSS_N_INNER; q++ )
        {
            double xi = a + GAUSS_PTS_6[q] * ( b - a );
            double N[3];
            shapeFunctions( xi, N );

            double X, Y;
            interpolate( xi, aElem, X, Y );

            double J = jacobian( xi, aElem );
            double G = greenPotential( x, y, X, Y );

            double w = GAUSS_WTS_6[q] * ( b - a );

            for( int i = 0; i < 3; i++ )
                aValue[i] += w * N[i] * G * J;
        }
    };

    integrate( 0.0, aXiOuter );
    integrate( aXiOuter, 1.0 );
}


// ---------------------------------------------------------------------------
// Inner interval integration — flux kernel ∂G/∂n
// ---------------------------------------------------------------------------

void BEM_2D_SOLVER::intervalFlux( double x, double y,
                                   const ELEMENT& aInner,
                                   double aValue[3],
                                   double nx, double ny ) const
{
    aValue[0] = aValue[1] = aValue[2] = 0.0;

    for( int q = 0; q < GAUSS_N_INNER; q++ )
    {
        double xi = GAUSS_PTS_6[q];
        double N[3];
        shapeFunctions( xi, N );

        double X, Y;
        interpolate( xi, aInner, X, Y );

        double J = jacobian( xi, aInner );
        double dGdn = greenFlux( x, y, X, Y, nx, ny );

        for( int i = 0; i < 3; i++ )
            aValue[i] += GAUSS_WTS_6[q] * N[i] * dGdn * J;
    }
}


void BEM_2D_SOLVER::intervalSelfFlux( double x, double y,
                                       const ELEMENT& aElem,
                                       double aValue[3], double aXiOuter,
                                       double nx, double ny ) const
{
    aValue[0] = aValue[1] = aValue[2] = 0.0;

    auto integrate = [&]( double a, double b )
    {
        if( b - a < 1e-12 )
            return;

        for( int q = 0; q < GAUSS_N_INNER; q++ )
        {
            double xi = a + GAUSS_PTS_6[q] * ( b - a );
            double N[3];
            shapeFunctions( xi, N );

            double X, Y;
            interpolate( xi, aElem, X, Y );

            double J = jacobian( xi, aElem );
            double dGdn = greenFlux( x, y, X, Y, nx, ny );

            double w = GAUSS_WTS_6[q] * ( b - a );

            for( int i = 0; i < 3; i++ )
                aValue[i] += w * N[i] * dGdn * J;
        }
    };

    integrate( 0.0, aXiOuter );
    integrate( aXiOuter, 1.0 );
}


// ---------------------------------------------------------------------------
// Load vector — NMMTL convention: b = ε₀ × V
// ---------------------------------------------------------------------------

void BEM_2D_SOLVER::buildLoadVector( Eigen::VectorXd& aB,
                                      int aActiveConductor ) const
{
    aB.setZero();

    for( const ELEMENT& el : m_condElements )
    {
        if( el.conductorIdx != aActiveConductor )
            continue;

        for( int q = 0; q < GAUSS_N_OUTER; q++ )
        {
            double xi = GAUSS_PTS_10[q];
            double N[3];
            shapeFunctions( xi, N );

            double J = jacobian( xi, el );

            for( int i = 0; i < 3; i++ )
                aB( el.nodeIdx[i] ) += EPS0 * GAUSS_WTS_10[q] * N[i] * J;
        }
    }
}


// ---------------------------------------------------------------------------
// Charge extraction — NMMTL convention: Q = ∫ εr × σ × N dl
// ---------------------------------------------------------------------------

void BEM_2D_SOLVER::extractCharge( const Eigen::VectorXd& aSigma,
                                    double aEpsFactor,
                                    Eigen::VectorXd& aQ ) const
{
    aQ.setZero();

    for( const ELEMENT& el : m_condElements )
    {
        int ci = el.conductorIdx;

        if( ci < 0 )
            continue; // skip ground and interface elements

        for( int q = 0; q < GAUSS_N_OUTER; q++ )
        {
            double xi = GAUSS_PTS_10[q];
            double N[3];
            shapeFunctions( xi, N );

            double J = jacobian( xi, el );

            for( int i = 0; i < 3; i++ )
            {
                aQ( ci ) += aEpsFactor * el.epsilonR * GAUSS_WTS_10[q]
                            * N[i] * aSigma( el.nodeIdx[i] ) * J;
            }
        }
    }
}


// ---------------------------------------------------------------------------
// Full-system assembly and solve (conductor + interface)
// ---------------------------------------------------------------------------

Eigen::MatrixXd BEM_2D_SOLVER::solveCapacitanceFull()
{
    int nTotal = m_numCondNodes + m_numIntfNodes;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero( nTotal, nTotal );

    // --- Conductor rows (Galerkin: outer on conductor, inner on all) ---
    for( const ELEMENT& outerEl : m_condElements )
    {
        for( int qo = 0; qo < GAUSS_N_OUTER; qo++ )
        {
            double xiOuter = GAUSS_PTS_10[qo];
            double No[3];
            shapeFunctions( xiOuter, No );

            double x, y;
            interpolate( xiOuter, outerEl, x, y );

            double Jo = jacobian( xiOuter, outerEl );

            // Inner loop — conductor elements
            for( const ELEMENT& innerEl : m_condElements )
            {
                double val[3];

                if( &innerEl == &outerEl )
                    intervalSelfConductor( x, y, innerEl, val, xiOuter );
                else
                    intervalConductor( x, y, innerEl, val );

                for( int i = 0; i < 3; i++ )
                    for( int j = 0; j < 3; j++ )
                        A( outerEl.nodeIdx[i], innerEl.nodeIdx[j] ) +=
                            INV_TWO_PI * GAUSS_WTS_10[qo] * No[i] * val[j] * Jo;
            }

            // Inner loop — interface elements
            for( const ELEMENT& innerEl : m_intfElements )
            {
                double val[3];
                intervalConductor( x, y, innerEl, val );

                for( int i = 0; i < 3; i++ )
                    for( int j = 0; j < 3; j++ )
                        A( outerEl.nodeIdx[i], innerEl.nodeIdx[j] ) +=
                            INV_TWO_PI * GAUSS_WTS_10[qo] * No[i] * val[j] * Jo;
            }
        }
    }

    // --- Interface rows ---
    for( const ELEMENT& outerEl : m_intfElements )
    {
        double coef1 = m_lengthScale * ( outerEl.epsPlus + outerEl.epsMinus ) / 2.0;
        double coef2 = m_lengthScale * ( outerEl.epsPlus - outerEl.epsMinus )
                       * INV_TWO_PI;

        // Diagonal — mass matrix (no Green's function)
        for( int qo = 0; qo < GAUSS_N_OUTER; qo++ )
        {
            double xi = GAUSS_PTS_10[qo];
            double No[3];
            shapeFunctions( xi, No );
            double Jo = jacobian( xi, outerEl );

            for( int i = 0; i < 3; i++ )
                for( int j = 0; j < 3; j++ )
                    A( outerEl.nodeIdx[j], outerEl.nodeIdx[i] ) +=
                        coef1 * GAUSS_WTS_10[qo] * No[i] * No[j] * Jo;
        }

        // Off-diagonal (flux kernel) — only if εr changes
        if( std::abs( coef2 ) < 1e-20 )
            continue;

        for( int qo = 0; qo < GAUSS_N_OUTER; qo++ )
        {
            double xiOuter = GAUSS_PTS_10[qo];
            double No[3];
            shapeFunctions( xiOuter, No );

            double x, y;
            interpolate( xiOuter, outerEl, x, y );
            double Jo = jacobian( xiOuter, outerEl );

            // Inner — conductor elements
            for( const ELEMENT& innerEl : m_condElements )
            {
                double val[3];
                intervalFlux( x, y, innerEl, val,
                              outerEl.normalX, outerEl.normalY );

                for( int i = 0; i < 3; i++ )
                    for( int j = 0; j < 3; j++ )
                        A( outerEl.nodeIdx[i], innerEl.nodeIdx[j] ) +=
                            coef2 * GAUSS_WTS_10[qo] * No[i] * val[j] * Jo;
            }

            // Inner — interface elements
            for( const ELEMENT& innerEl : m_intfElements )
            {
                double val[3];

                if( &innerEl == &outerEl )
                    intervalSelfFlux( x, y, innerEl, val, xiOuter,
                                     outerEl.normalX, outerEl.normalY );
                else
                    intervalFlux( x, y, innerEl, val,
                                  outerEl.normalX, outerEl.normalY );

                for( int i = 0; i < 3; i++ )
                    for( int j = 0; j < 3; j++ )
                        A( outerEl.nodeIdx[i], innerEl.nodeIdx[j] ) +=
                            coef2 * GAUSS_WTS_10[qo] * No[i] * val[j] * Jo;
            }
        }
    }

    // --- Solve for each conductor excitation ---
    Eigen::FullPivLU<Eigen::MatrixXd> lu( A );
    Eigen::MatrixXd Cmat( m_numConductors, m_numConductors );
    Cmat.setZero();

    for( int active = 0; active < m_numConductors; active++ )
    {
        Eigen::VectorXd b( nTotal );
        buildLoadVector( b, active );

        Eigen::VectorXd sigma = lu.solve( b );

        Eigen::VectorXd Q( m_numConductors );
        extractCharge( sigma, 1.0, Q );

        for( int ci = 0; ci < m_numConductors; ci++ )
            Cmat( ci, active ) = Q( ci );
    }

    return Cmat;
}


// ---------------------------------------------------------------------------
// Free-space (air-only) assembly and solve — conductor elements only
// ---------------------------------------------------------------------------

Eigen::MatrixXd BEM_2D_SOLVER::solveCapacitanceAir()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero( m_numCondNodes, m_numCondNodes );

    for( const ELEMENT& outerEl : m_condElements )
    {
        for( int qo = 0; qo < GAUSS_N_OUTER; qo++ )
        {
            double xiOuter = GAUSS_PTS_10[qo];
            double No[3];
            shapeFunctions( xiOuter, No );

            double x, y;
            interpolate( xiOuter, outerEl, x, y );
            double Jo = jacobian( xiOuter, outerEl );

            for( const ELEMENT& innerEl : m_condElements )
            {
                double val[3];

                if( &innerEl == &outerEl )
                    intervalSelfConductor( x, y, innerEl, val, xiOuter );
                else
                    intervalConductor( x, y, innerEl, val );

                for( int i = 0; i < 3; i++ )
                    for( int j = 0; j < 3; j++ )
                        A( outerEl.nodeIdx[i], innerEl.nodeIdx[j] ) +=
                            INV_TWO_PI * GAUSS_WTS_10[qo] * No[i] * val[j] * Jo;
            }
        }
    }

    Eigen::FullPivLU<Eigen::MatrixXd> lu( A );
    Eigen::MatrixXd Cmat( m_numConductors, m_numConductors );
    Cmat.setZero();

    for( int active = 0; active < m_numConductors; active++ )
    {
        Eigen::VectorXd b = Eigen::VectorXd::Zero( m_numCondNodes );

        // Load vector — ε₀ × V on active conductor nodes only
        for( const ELEMENT& el : m_condElements )
        {
            if( el.conductorIdx != active )
                continue;

            for( int q = 0; q < GAUSS_N_OUTER; q++ )
            {
                double xi = GAUSS_PTS_10[q];
                double N[3];
                shapeFunctions( xi, N );
                double J = jacobian( xi, el );

                for( int i = 0; i < 3; i++ )
                    b( el.nodeIdx[i] ) += EPS0 * GAUSS_WTS_10[q] * N[i] * J;
            }
        }

        Eigen::VectorXd sigma = lu.solve( b );

        // Charge extraction — εr = 1.0 (air)
        Eigen::VectorXd Q = Eigen::VectorXd::Zero( m_numConductors );

        for( const ELEMENT& el : m_condElements )
        {
            int ci = el.conductorIdx;

            if( ci < 0 )
                continue; // skip ground elements

            for( int q = 0; q < GAUSS_N_OUTER; q++ )
            {
                double xi = GAUSS_PTS_10[q];
                double N[3];
                shapeFunctions( xi, N );
                double J = jacobian( xi, el );

                for( int i = 0; i < 3; i++ )
                    Q( ci ) += 1.0 * GAUSS_WTS_10[q] * N[i] * sigma( el.nodeIdx[i] ) * J;
            }
        }

        for( int ci = 0; ci < m_numConductors; ci++ )
            Cmat( ci, active ) = Q( ci );
    }

    return Cmat;
}


// ---------------------------------------------------------------------------
// Top-level solve
// ---------------------------------------------------------------------------

bool BEM_2D_SOLVER::Solve()
{
    if( m_geometry.conductors.empty() )
        return false;

    buildElements();

    if( m_condElements.empty() )
        return false;

    // Air-only solve (always needed)
    Eigen::MatrixXd Cair = solveCapacitanceAir();

    if( Cair( 0, 0 ) <= 0.0 )
        return false;

    // Full solve with dielectric
    Eigen::MatrixXd Cfull;

    if( m_intfElements.empty() )
    {
        // Uniform dielectric: C_full = εr × C_air
        double er = std::max( m_geometry.epsilonR, 1.0 );
        Cfull = er * Cair;
    }
    else
    {
        Cfull = solveCapacitanceFull();

        if( Cfull( 0, 0 ) <= 0.0 )
            return false;
    }

    // Symmetrize
    m_result.C  = ( Cfull + Cfull.transpose() ) / 2.0;
    m_result.C0 = ( Cair + Cair.transpose() ) / 2.0;

    // L = μ₀ε₀ × C₀⁻¹
    static constexpr double MU0 = 4.0 * M_PI * 1e-7;
    m_result.L = MU0 * EPS0 * Cair.inverse();

    // Z₀ = 1 / (c × √(C₁₁ × C₀₁₁))
    m_result.Z0 = 1.0 / ( C_LIGHT * sqrt( Cfull( 0, 0 ) * Cair( 0, 0 ) ) );
    m_result.erEff = Cfull( 0, 0 ) / Cair( 0, 0 );

    // Differential impedance for 2-conductor systems
    if( m_numConductors == 2 )
    {
        double Codd  = Cfull( 0, 0 ) - Cfull( 0, 1 );
        double C0odd = Cair( 0, 0 ) - Cair( 0, 1 );

        if( Codd > 0.0 && C0odd > 0.0 )
        {
            double Zodd = 1.0 / ( C_LIGHT * sqrt( Codd * C0odd ) );
            m_result.Zdiff = 2.0 * Zodd;
        }
    }

    return true;
}
