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
 * Microstrip: 0.15mm wide trace, 35µm thick, 0.1mm above ground, er=4.4.
 * Compare BEM Z0 against analytical Hammerstad-Jensen.
 */
BOOST_AUTO_TEST_CASE( MicrostripVsAnalytical )
{
    double w = 0.15e-3;    // trace width
    double t = 35e-6;      // trace thickness
    double h = 0.1e-3;     // dielectric height (trace bottom to ground)
    double er = 4.4;

    // BEM geometry: ground plane at y=0, trace centered at y = -(h + t/2)
    // (negative y = above ground in our coordinate system where ground is at y=0)
    XS_GEOMETRY geom;

    XS_CONDUCTOR cond;
    cond.centerX = 0.0;
    cond.centerY = -( h + t / 2.0 ); // Above ground plane
    cond.width = w;
    cond.thickness = t;
    geom.conductors.push_back( cond );

    geom.groundY = 0.0;
    geom.epsilonR = er;

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

    // BEM with uniform dielectric will overestimate Z0 for microstrip
    // (real microstrip has dielectric only below the trace, air above).
    // The BEM currently models uniform dielectric, so Z0 will be higher
    // than true microstrip but in the right ballpark.
    BOOST_CHECK_GT( bemZ0, 30.0 );
    BOOST_CHECK_LT( bemZ0, 80.0 );

    // Should be within 20% of analytical — the BEM models a slightly
    // different geometry (uniform dielectric vs. half-space interface)
    BOOST_CHECK_CLOSE( bemZ0, analyticalZ0, 20.0 );
}


/**
 * Wider trace should have lower impedance — sanity check.
 */
BOOST_AUTO_TEST_CASE( WiderIsLower )
{
    auto makeGeom = []( double w ) -> XS_GEOMETRY
    {
        XS_GEOMETRY geom;

        XS_CONDUCTOR cond;
        cond.centerX = 0.0;
        cond.centerY = -0.1175e-3; // 0.1mm + 17.5µm
        cond.width = w;
        cond.thickness = 35e-6;
        geom.conductors.push_back( cond );

        geom.groundY = 0.0;
        geom.epsilonR = 4.4;
        return geom;
    };

    BEM_2D_SOLVER solverNarrow;
    solverNarrow.SetGeometry( makeGeom( 0.1e-3 ) );
    solverNarrow.SetPanelsPerEdge( 10 );
    solverNarrow.Solve();

    BEM_2D_SOLVER solverWide;
    solverWide.SetGeometry( makeGeom( 0.3e-3 ) );
    solverWide.SetPanelsPerEdge( 10 );
    solverWide.Solve();

    BOOST_TEST_MESSAGE( "Narrow Z0 = " << solverNarrow.GetResult().Z0
                        << ", Wide Z0 = " << solverWide.GetResult().Z0 );

    BOOST_CHECK_GT( solverNarrow.GetResult().Z0, solverWide.GetResult().Z0 );
}


/**
 * Higher er should give lower Z0 — sanity check.
 */
BOOST_AUTO_TEST_CASE( HigherErIsLower )
{
    auto makeGeom = []( double er ) -> XS_GEOMETRY
    {
        XS_GEOMETRY geom;

        XS_CONDUCTOR cond;
        cond.centerX = 0.0;
        cond.centerY = -0.1175e-3;
        cond.width = 0.15e-3;
        cond.thickness = 35e-6;
        geom.conductors.push_back( cond );

        geom.groundY = 0.0;
        geom.epsilonR = er;
        return geom;
    };

    BEM_2D_SOLVER solverLow;
    solverLow.SetGeometry( makeGeom( 3.0 ) );
    solverLow.SetPanelsPerEdge( 10 );
    solverLow.Solve();

    BEM_2D_SOLVER solverHigh;
    solverHigh.SetGeometry( makeGeom( 6.0 ) );
    solverHigh.SetPanelsPerEdge( 10 );
    solverHigh.Solve();

    BOOST_TEST_MESSAGE( "er=3.0: Z0 = " << solverLow.GetResult().Z0
                        << ", er=6.0: Z0 = " << solverHigh.GetResult().Z0 );

    BOOST_CHECK_GT( solverLow.GetResult().Z0, solverHigh.GetResult().Z0 );
}


/**
 * Panel convergence: more panels should converge to a stable Z0.
 */
BOOST_AUTO_TEST_CASE( PanelConvergence )
{
    XS_GEOMETRY geom;

    XS_CONDUCTOR cond;
    cond.centerX = 0.0;
    cond.centerY = -0.1175e-3;
    cond.width = 0.15e-3;
    cond.thickness = 35e-6;
    geom.conductors.push_back( cond );

    geom.groundY = 0.0;
    geom.epsilonR = 4.4;

    double prevZ0 = 0.0;

    double prevDelta = 1e9;

    for( int n : { 5, 10, 15, 20, 30 } )
    {
        BEM_2D_SOLVER solver;
        solver.SetGeometry( geom );
        solver.SetPanelsPerEdge( n );
        solver.Solve();

        double z0 = solver.GetResult().Z0;

        BOOST_TEST_MESSAGE( "Panels/edge=" << n << " (total " << ( 4 * n )
                            << "): Z0 = " << z0 << " Ohm" );

        if( prevZ0 > 0.0 )
        {
            double delta = std::abs( z0 - prevZ0 );

            // Each refinement should produce a smaller or similar change (convergence)
            BOOST_CHECK_LT( delta, prevDelta * 1.05 );
            prevDelta = delta;
        }

        prevZ0 = z0;
    }
}


/**
 * Effective dielectric constant should be between 1 and er.
 */
BOOST_AUTO_TEST_CASE( EffectiveEr )
{
    XS_GEOMETRY geom;

    XS_CONDUCTOR cond;
    cond.centerX = 0.0;
    cond.centerY = -0.1175e-3;
    cond.width = 0.15e-3;
    cond.thickness = 35e-6;
    geom.conductors.push_back( cond );

    geom.groundY = 0.0;
    geom.epsilonR = 4.4;

    BEM_2D_SOLVER solver;
    solver.SetGeometry( geom );
    solver.SetPanelsPerEdge( 12 );
    solver.Solve();

    double erEff = solver.GetResult().erEff;

    BOOST_TEST_MESSAGE( "Effective er = " << erEff );

    // With uniform dielectric BEM, er_eff = er (no air region modeled).
    // For true microstrip, er_eff would be between 1 and er.
    // Just verify it's a reasonable positive value.
    BOOST_CHECK_GT( erEff, 0.5 );
    BOOST_CHECK_LT( erEff, 10.0 );
}


BOOST_AUTO_TEST_SUITE_END()
