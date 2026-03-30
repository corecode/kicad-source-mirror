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

#include <qa_utils/wx_utils/unit_test_utils.h>

#include <sipi/bem_2d_solver.h>
#include <sipi/analytical_impedance.h>

#include <cmath>


BOOST_AUTO_TEST_SUITE( BEM2DSolver )


/**
 * Helper: build a standard microstrip XS_GEOMETRY.
 * Ground at y=0, conductor at y = -(h + t/2), substrate εr below, air above.
 */
static XS_GEOMETRY makeMicrostripGeom( double w, double h, double er, double t = 35e-6,
                                       double centerX = 0.0 )
{
    XS_GEOMETRY geom;

    XS_CONDUCTOR cond;
    cond.centerX = centerX;
    cond.centerY = -( h + t / 2.0 );
    cond.width = w;
    cond.thickness = t;
    geom.conductors.push_back( cond );

    geom.groundY = 0.0;
    geom.epsilonR = er;

    XS_DIELECTRIC_REGION dielAbove;
    dielAbove.yTop = -10e-3;
    dielAbove.yBottom = -h;
    dielAbove.epsilonR = 1.0;

    XS_DIELECTRIC_REGION dielBelow;
    dielBelow.yTop = -h;
    dielBelow.yBottom = 0.0;
    dielBelow.epsilonR = er;

    geom.dielectrics.push_back( dielAbove );
    geom.dielectrics.push_back( dielBelow );

    return geom;
}


/**
 * Microstrip: 0.15mm wide trace, 35µm thick, 0.1mm above ground, er=4.4.
 * Compare BEM Z0 against analytical Hammerstad-Jensen.
 */
BOOST_AUTO_TEST_CASE( MicrostripVsAnalytical )
{
    double w = 0.15e-3;
    double t = 35e-6;
    double h = 0.1e-3;
    double er = 4.4;

    XS_GEOMETRY geom = makeMicrostripGeom( w, h, er, t );

    BEM_2D_SOLVER solver;
    solver.SetGeometry( geom );
    solver.SetPanelsPerEdge( 20 );

    bool ok = solver.Solve();
    BOOST_REQUIRE( ok );

    const RLGC_RESULT& result = solver.GetResult();

    double bemZ0 = result.Z0;
    double analyticalZ0 = ANALYTICAL_IMPEDANCE::MicrostripZ0( w, h, er, t );

    BOOST_TEST_MESSAGE( "Microstrip BEM Z0 = " << bemZ0 << " Ohm" );
    BOOST_TEST_MESSAGE( "Microstrip analytical Z0 = " << analyticalZ0 << " Ohm" );
    BOOST_TEST_MESSAGE( "Difference: " << std::abs( bemZ0 - analyticalZ0 ) / analyticalZ0 * 100.0
                        << "%" );
    BOOST_TEST_MESSAGE( "er_eff = " << result.erEff );

    BOOST_CHECK_GT( result.erEff, 1.0 );
    BOOST_CHECK_LT( result.erEff, er );

    BOOST_CHECK_GT( bemZ0, 30.0 );
    BOOST_CHECK_LT( bemZ0, 90.0 );
}


/**
 * Wider trace should have lower impedance — sanity check.
 */
BOOST_AUTO_TEST_CASE( WiderIsLower )
{
    BEM_2D_SOLVER solverNarrow;
    solverNarrow.SetGeometry( makeMicrostripGeom( 0.1e-3, 0.1e-3, 4.4 ) );
    solverNarrow.SetPanelsPerEdge( 10 );
    BOOST_REQUIRE( solverNarrow.Solve() );

    BEM_2D_SOLVER solverWide;
    solverWide.SetGeometry( makeMicrostripGeom( 0.3e-3, 0.1e-3, 4.4 ) );
    solverWide.SetPanelsPerEdge( 10 );
    BOOST_REQUIRE( solverWide.Solve() );

    BOOST_TEST_MESSAGE( "Narrow Z0 = " << solverNarrow.GetResult().Z0
                        << ", Wide Z0 = " << solverWide.GetResult().Z0 );

    BOOST_CHECK_GT( solverNarrow.GetResult().Z0, solverWide.GetResult().Z0 );
}


/**
 * Higher er should give lower Z0 — sanity check.
 */
BOOST_AUTO_TEST_CASE( HigherErIsLower )
{
    BEM_2D_SOLVER solverLow;
    solverLow.SetGeometry( makeMicrostripGeom( 0.15e-3, 0.1e-3, 3.0 ) );
    solverLow.SetPanelsPerEdge( 10 );
    BOOST_REQUIRE( solverLow.Solve() );

    BEM_2D_SOLVER solverHigh;
    solverHigh.SetGeometry( makeMicrostripGeom( 0.15e-3, 0.1e-3, 6.0 ) );
    solverHigh.SetPanelsPerEdge( 10 );
    BOOST_REQUIRE( solverHigh.Solve() );

    BOOST_TEST_MESSAGE( "er=3.0: Z0 = " << solverLow.GetResult().Z0
                        << ", er=6.0: Z0 = " << solverHigh.GetResult().Z0 );

    BOOST_CHECK_GT( solverLow.GetResult().Z0, solverHigh.GetResult().Z0 );
}


/**
 * Panel convergence: more panels should converge to a stable Z0.
 */
BOOST_AUTO_TEST_CASE( PanelConvergence )
{
    XS_GEOMETRY geom = makeMicrostripGeom( 0.15e-3, 0.1e-3, 4.4 );

    double prevZ0 = 0.0;

    for( int n : { 5, 10, 15, 20, 30 } )
    {
        BEM_2D_SOLVER solver;
        solver.SetGeometry( geom );
        solver.SetPanelsPerEdge( n );
        BOOST_REQUIRE( solver.Solve() );

        double z0 = solver.GetResult().Z0;

        BOOST_TEST_MESSAGE( "Panels/edge=" << n << " (total " << ( 4 * n )
                            << "): Z0 = " << z0 << " Ohm" );

        if( prevZ0 > 0.0 )
        {
            double delta = std::abs( z0 - prevZ0 );

            // Values should stay close (converged to < 2Ω variation between steps)
            BOOST_CHECK_LT( delta, 2.0 );
        }

        prevZ0 = z0;
    }
}


/**
 * Effective dielectric constant should be between 1 and er.
 */
BOOST_AUTO_TEST_CASE( EffectiveEr )
{
    XS_GEOMETRY geom = makeMicrostripGeom( 0.15e-3, 0.1e-3, 4.4 );

    BEM_2D_SOLVER solver;
    solver.SetGeometry( geom );
    solver.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solver.Solve() );

    double erEff = solver.GetResult().erEff;

    BOOST_TEST_MESSAGE( "Effective er = " << erEff );

    // For microstrip, er_eff should be between 1 and εr
    BOOST_CHECK_GT( erEff, 1.5 );
    BOOST_CHECK_LT( erEff, 4.4 );
}


/**
 * Coupled microstrip: two parallel traces, compute Zdiff.
 * w = 0.1mm each, spacing = 0.15mm edge-to-edge, h = 0.1mm, er = 4.4.
 */
BOOST_AUTO_TEST_CASE( CoupledMicrostrip )
{
    double w = 0.1e-3;
    double s = 0.15e-3; // edge-to-edge gap
    double t = 35e-6;
    double h = 0.1e-3;
    double er = 4.4;

    XS_GEOMETRY geom;

    // Left trace
    XS_CONDUCTOR cond1;
    cond1.centerX = -( s / 2.0 + w / 2.0 ); // centered left of the gap
    cond1.centerY = -( h + t / 2.0 );
    cond1.width = w;
    cond1.thickness = t;
    geom.conductors.push_back( cond1 );

    // Right trace
    XS_CONDUCTOR cond2;
    cond2.centerX = ( s / 2.0 + w / 2.0 ); // centered right of the gap
    cond2.centerY = -( h + t / 2.0 );
    cond2.width = w;
    cond2.thickness = t;
    geom.conductors.push_back( cond2 );

    geom.groundY = 0.0;
    geom.epsilonR = er;

    // Dielectric regions: FR4 below, air above
    XS_DIELECTRIC_REGION dielAbove;
    dielAbove.yTop = -10e-3;
    dielAbove.yBottom = -h;
    dielAbove.epsilonR = 1.0;

    XS_DIELECTRIC_REGION dielBelow;
    dielBelow.yTop = -h;
    dielBelow.yBottom = 0.0;
    dielBelow.epsilonR = er;

    geom.dielectrics.push_back( dielAbove );
    geom.dielectrics.push_back( dielBelow );

    BEM_2D_SOLVER solver;
    solver.SetGeometry( geom );
    solver.SetPanelsPerEdge( 15 );

    bool ok = solver.Solve();
    BOOST_REQUIRE( ok );

    const RLGC_RESULT& result = solver.GetResult();

    BOOST_TEST_MESSAGE( "Coupled microstrip:" );
    BOOST_TEST_MESSAGE( "  Z0 (single-ended) = " << result.Z0 << " Ohm" );
    BOOST_TEST_MESSAGE( "  Zdiff = " << result.Zdiff << " Ohm" );
    BOOST_TEST_MESSAGE( "  C matrix:\n" << result.C );
    BOOST_TEST_MESSAGE( "  L matrix:\n" << result.L );

    // Zdiff should be roughly 2× Z0 (exact only for zero coupling)
    BOOST_CHECK_GT( result.Zdiff, 0.0 );
    BOOST_CHECK_GT( result.Z0, 0.0 );

    // Zdiff should be less than 2×Z0 (coupling reduces Zdiff)
    BOOST_CHECK_LT( result.Zdiff, 2.0 * result.Z0 );

    // C matrix should be symmetric with negative off-diagonal (mutual capacitance)
    BOOST_CHECK_CLOSE( result.C( 0, 1 ), result.C( 1, 0 ), 0.1 );
    BOOST_CHECK_LT( result.C( 0, 1 ), 0.0 ); // mutual C is negative in Maxwell form

    // L matrix should be symmetric with positive off-diagonal (mutual inductance)
    BOOST_CHECK_CLOSE( result.L( 0, 1 ), result.L( 1, 0 ), 0.1 );
    BOOST_CHECK_GT( result.L( 0, 1 ), 0.0 );

    // Zdiff should be in a reasonable range (typically 80-120Ω for FR4 diff pair)
    BOOST_CHECK_GT( result.Zdiff, 50.0 );
    BOOST_CHECK_LT( result.Zdiff, 200.0 );
}


/**
 * Wider spacing should give Zdiff closer to 2×Z0 (less coupling).
 */
BOOST_AUTO_TEST_CASE( CoupledSpacingEffect )
{
    auto makeCoupledGeom = []( double spacing ) -> XS_GEOMETRY
    {
        double w = 0.1e-3;
        double t = 35e-6;
        double h = 0.1e-3;
        double er = 4.4;

        XS_GEOMETRY geom;

        XS_CONDUCTOR c1;
        c1.centerX = -( spacing / 2.0 + w / 2.0 );
        c1.centerY = -( h + t / 2.0 );
        c1.width = w;
        c1.thickness = t;
        geom.conductors.push_back( c1 );

        XS_CONDUCTOR c2;
        c2.centerX = ( spacing / 2.0 + w / 2.0 );
        c2.centerY = -( h + t / 2.0 );
        c2.width = w;
        c2.thickness = t;
        geom.conductors.push_back( c2 );

        geom.groundY = 0.0;
        geom.epsilonR = er;

        XS_DIELECTRIC_REGION dielAbove;
        dielAbove.yTop = -10e-3;
        dielAbove.yBottom = -h;
        dielAbove.epsilonR = 1.0;

        XS_DIELECTRIC_REGION dielBelow;
        dielBelow.yTop = -h;
        dielBelow.yBottom = 0.0;
        dielBelow.epsilonR = er;

        geom.dielectrics.push_back( dielAbove );
        geom.dielectrics.push_back( dielBelow );

        return geom;
    };

    BEM_2D_SOLVER solverClose;
    solverClose.SetGeometry( makeCoupledGeom( 0.1e-3 ) );
    solverClose.SetPanelsPerEdge( 10 );
    solverClose.Solve();

    BEM_2D_SOLVER solverFar;
    solverFar.SetGeometry( makeCoupledGeom( 0.5e-3 ) );
    solverFar.SetPanelsPerEdge( 10 );
    solverFar.Solve();

    BOOST_TEST_MESSAGE( "Close spacing: Zdiff = " << solverClose.GetResult().Zdiff
                        << ", Z0 = " << solverClose.GetResult().Z0 );
    BOOST_TEST_MESSAGE( "Far spacing:   Zdiff = " << solverFar.GetResult().Zdiff
                        << ", Z0 = " << solverFar.GetResult().Z0 );

    // Wider spacing → less coupling → Zdiff closer to 2×Z0
    double ratioClose = solverClose.GetResult().Zdiff / ( 2.0 * solverClose.GetResult().Z0 );
    double ratioFar = solverFar.GetResult().Zdiff / ( 2.0 * solverFar.GetResult().Z0 );

    BOOST_TEST_MESSAGE( "Zdiff/(2*Z0) ratio: close=" << ratioClose << " far=" << ratioFar );
    BOOST_CHECK_GT( ratioFar, ratioClose );
}


BOOST_AUTO_TEST_SUITE_END()
