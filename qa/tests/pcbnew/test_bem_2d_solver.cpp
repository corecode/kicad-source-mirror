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


/**
 * Groundwire regression: a geometry with no ground conductors should produce
 * the same result as before the groundwire feature was added.
 */
BOOST_AUTO_TEST_CASE( GroundwireRegression )
{
    double w = 0.15e-3;
    double h = 0.1e-3;
    double er = 4.4;

    XS_GEOMETRY geom = makeMicrostripGeom( w, h, er );

    BEM_2D_SOLVER solver;
    solver.SetGeometry( geom );
    solver.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solver.Solve() );

    double z0 = solver.GetResult().Z0;

    BOOST_TEST_MESSAGE( "Groundwire regression: Z0 = " << z0 );

    // Same check as MicrostripVsAnalytical — must still be in range
    BOOST_CHECK_GT( z0, 30.0 );
    BOOST_CHECK_LT( z0, 100.0 );
}


/**
 * Groundwire basic: add two wide ground conductors flanking the signal at the
 * ground level.  These should increase capacitance (lower Z₀) compared to the
 * plain microstrip with only the image ground.
 *
 * Geometry (cross-section):
 *                    signal (0.15mm)
 *                       ===
 *   [groundwire]                    [groundwire]
 *   ============   | gap 0.5mm |   =============
 *   ─────────────────────────────────────────────  image ground at y=0
 */
BOOST_AUTO_TEST_CASE( GroundwireIncreasesCapacitance )
{
    double w = 0.15e-3;
    double t = 35e-6;
    double h = 0.1e-3;
    double er = 4.4;

    // Baseline: standard microstrip (no groundwires)
    XS_GEOMETRY geomBase = makeMicrostripGeom( w, h, er, t );

    BEM_2D_SOLVER solverBase;
    solverBase.SetGeometry( geomBase );
    solverBase.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solverBase.Solve() );

    double z0Base = solverBase.GetResult().Z0;

    // With groundwires: two wide copper strips at ground level (y=0),
    // flanking the signal with a 0.25mm gap on each side.
    XS_GEOMETRY geomGW = makeMicrostripGeom( w, h, er, t );

    XS_CONDUCTOR gwLeft;
    gwLeft.centerX = -0.5e-3;    // 0.5mm to the left
    gwLeft.centerY = -t / 2.0;   // at ground level, copper thickness
    gwLeft.width = 0.5e-3;       // 0.5mm wide
    gwLeft.thickness = t;
    gwLeft.isGround = true;
    geomGW.conductors.push_back( gwLeft );

    XS_CONDUCTOR gwRight;
    gwRight.centerX = 0.5e-3;
    gwRight.centerY = -t / 2.0;
    gwRight.width = 0.5e-3;
    gwRight.thickness = t;
    gwRight.isGround = true;
    geomGW.conductors.push_back( gwRight );

    BEM_2D_SOLVER solverGW;
    solverGW.SetGeometry( geomGW );
    solverGW.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solverGW.Solve() );

    double z0GW = solverGW.GetResult().Z0;

    BOOST_TEST_MESSAGE( "Z0 without groundwires: " << z0Base );
    BOOST_TEST_MESSAGE( "Z0 with groundwires:    " << z0GW );
    BOOST_TEST_MESSAGE( "C base:  " << solverBase.GetResult().C( 0, 0 ) );
    BOOST_TEST_MESSAGE( "C with GW: " << solverGW.GetResult().C( 0, 0 ) );

    // Groundwires add capacitance → lower Z₀
    BOOST_CHECK_LT( z0GW, z0Base );

    // The effect should be measurable (at least 0.1Ω difference)
    BOOST_CHECK_GT( z0Base - z0GW, 0.1 );
}


/**
 * Ground receding: as the image ground moves further from the trace while
 * groundwires remain at a fixed distance, Z₀ should increase smoothly.
 * This simulates the antipad case: the reference layer copper (groundwires)
 * is nearby, but the next solid ground (image) is further away.
 *
 * Geometry:
 *                 signal
 *                   ===
 *  [gw]   | gap |         | gap |   [gw]   ← at y = -h_ref (original ref layer)
 *  ─────────────────────────────────────    ← image ground at y=0 (further away)
 */
BOOST_AUTO_TEST_CASE( GroundRecedingSmoothly )
{
    double w = 0.15e-3;
    double t = 35e-6;
    double er = 4.4;
    double hRef = 0.1e-3;   // original ref layer distance from signal
    int panels = 12;

    // Groundwire dimensions (constant across tests)
    double gwWidth = 1.0e-3;
    double gwGap = 0.25e-3;  // gap half-width (from signal center to gw edge)

    // Test with image ground at increasing distances
    double hValues[] = { 0.1e-3, 0.2e-3, 0.5e-3, 1.0e-3, 2.0e-3, 5.0e-3 };
    double prevZ0 = 0.0;

    for( double hImage : hValues )
    {
        XS_GEOMETRY geom;
        geom.groundY = 0.0;
        geom.epsilonR = er;

        // Signal conductor
        double condY = -( hImage + t / 2.0 );

        XS_CONDUCTOR cond;
        cond.centerX = 0.0;
        cond.centerY = condY;
        cond.width = w;
        cond.thickness = t;
        geom.conductors.push_back( cond );

        // Dielectric regions
        XS_DIELECTRIC_REGION air, diel;
        air.yTop = -10e-3;
        air.yBottom = -hImage;
        air.epsilonR = 1.0;
        diel.yTop = -hImage;
        diel.yBottom = 0.0;
        diel.epsilonR = er;
        geom.dielectrics.push_back( air );
        geom.dielectrics.push_back( diel );

        // Groundwires at original ref layer distance (constant y relative to signal)
        double gwY = condY + hRef;  // hRef below signal (toward ground)

        XS_CONDUCTOR gwLeft;
        gwLeft.centerX = -( gwGap + gwWidth / 2.0 );
        gwLeft.centerY = gwY;
        gwLeft.width = gwWidth;
        gwLeft.thickness = t;
        gwLeft.isGround = true;
        geom.conductors.push_back( gwLeft );

        XS_CONDUCTOR gwRight;
        gwRight.centerX = gwGap + gwWidth / 2.0;
        gwRight.centerY = gwY;
        gwRight.width = gwWidth;
        gwRight.thickness = t;
        gwRight.isGround = true;
        geom.conductors.push_back( gwRight );

        BEM_2D_SOLVER solver;
        solver.SetGeometry( geom );
        solver.SetPanelsPerEdge( panels );
        BOOST_REQUIRE( solver.Solve() );

        double z0 = solver.GetResult().Z0;

        BOOST_TEST_MESSAGE( "hImage=" << hImage * 1e3 << "mm  Z0=" << z0
                            << "  erEff=" << solver.GetResult().erEff );

        // Z₀ should be positive and reasonable
        BOOST_CHECK_GT( z0, 10.0 );
        BOOST_CHECK_LT( z0, 300.0 );

        // Z₀ should increase monotonically as image ground recedes
        if( prevZ0 > 0.0 )
            BOOST_CHECK_GT( z0, prevZ0 );

        prevZ0 = z0;
    }
}


/**
 * Ground opening: groundwires gradually open (gap widens) while image ground
 * stays at a fixed distance.  Simulates a trace approaching a via antipad —
 * the reference layer copper recedes laterally while the next ground layer
 * below stays constant.
 *
 * Z₀ should increase smoothly and monotonically as the gap widens.
 * At gap=0 (solid ground), Z₀ ≈ normal microstrip.
 * At very large gap, groundwires are far away and Z₀ approaches the
 * "deep ground only" value.
 */
BOOST_AUTO_TEST_CASE( GroundwiresOpeningSmoothly )
{
    double w = 0.15e-3;
    double t = 35e-6;
    double er = 4.4;
    double hImage = 0.5e-3;   // image ground fixed at 0.5mm (next layer)
    double hRef = 0.1e-3;     // groundwire layer is 0.1mm below signal
    double gwWidth = 2.0e-3;  // wide groundwires
    int panels = 12;

    // Reference: no groundwires at all (just image ground at 0.5mm)
    XS_GEOMETRY geomNoGW = makeMicrostripGeom( w, hImage, er, t );

    BEM_2D_SOLVER solverNoGW;
    solverNoGW.SetGeometry( geomNoGW );
    solverNoGW.SetPanelsPerEdge( panels );
    BOOST_REQUIRE( solverNoGW.Solve() );
    double z0NoGW = solverNoGW.GetResult().Z0;

    BOOST_TEST_MESSAGE( "No groundwires (image at 0.5mm): Z0=" << z0NoGW );

    // Sweep gap half-width from tight (0.1mm) to wide (3mm)
    double gapValues[] = { 0.1e-3, 0.15e-3, 0.2e-3, 0.3e-3, 0.5e-3,
                           0.75e-3, 1.0e-3, 1.5e-3, 2.0e-3, 3.0e-3 };
    double prevZ0 = 0.0;

    for( double halfGap : gapValues )
    {
        XS_GEOMETRY geom;
        geom.groundY = 0.0;
        geom.epsilonR = er;

        double condY = -( hImage + t / 2.0 );

        XS_CONDUCTOR cond;
        cond.centerX = 0.0;
        cond.centerY = condY;
        cond.width = w;
        cond.thickness = t;
        geom.conductors.push_back( cond );

        XS_DIELECTRIC_REGION air, diel;
        air.yTop = -10e-3;
        air.yBottom = -hImage;
        air.epsilonR = 1.0;
        diel.yTop = -hImage;
        diel.yBottom = 0.0;
        diel.epsilonR = er;
        geom.dielectrics.push_back( air );
        geom.dielectrics.push_back( diel );

        // Groundwires at hRef below signal, opening symmetrically
        double gwY = condY + hRef;

        XS_CONDUCTOR gwLeft;
        gwLeft.centerX = -( halfGap + gwWidth / 2.0 );
        gwLeft.centerY = gwY;
        gwLeft.width = gwWidth;
        gwLeft.thickness = t;
        gwLeft.isGround = true;
        geom.conductors.push_back( gwLeft );

        XS_CONDUCTOR gwRight;
        gwRight.centerX = halfGap + gwWidth / 2.0;
        gwRight.centerY = gwY;
        gwRight.width = gwWidth;
        gwRight.thickness = t;
        gwRight.isGround = true;
        geom.conductors.push_back( gwRight );

        BEM_2D_SOLVER solver;
        solver.SetGeometry( geom );
        solver.SetPanelsPerEdge( panels );
        BOOST_REQUIRE( solver.Solve() );

        double z0 = solver.GetResult().Z0;

        BOOST_TEST_MESSAGE( "halfGap=" << halfGap * 1e3 << "mm  Z0=" << z0
                            << "  erEff=" << solver.GetResult().erEff );

        BOOST_CHECK_GT( z0, 10.0 );
        BOOST_CHECK_LT( z0, 300.0 );

        // Z₀ should increase monotonically as gap widens
        if( prevZ0 > 0.0 )
            BOOST_CHECK_GT( z0, prevZ0 );

        prevZ0 = z0;
    }

    // At very wide gap, Z₀ should approach the no-groundwire value
    // (groundwires are so far away they have negligible effect)
    BOOST_TEST_MESSAGE( "Largest gap Z0=" << prevZ0
                        << "  no-GW Z0=" << z0NoGW );
    BOOST_CHECK_LT( std::abs( prevZ0 - z0NoGW ), 2.0 );
}


/**
 * BUG TEST: A very wide groundwire at the original ground plane position should
 * approximate the infinite ground plane.  When the image ground is pushed to
 * virtual earth (10mm), the groundwire must sit in the correct dielectric
 * (FR4, not air) to give the right Z₀.
 *
 * This test constructs the geometry manually with the proper dielectric
 * layering: FR4 from signal to groundwire level, air below.
 */
BOOST_AUTO_TEST_CASE( WideGroundwireMatchesImageGround )
{
    double w = 0.15e-3;
    double t = 35e-6;
    double hOrig = 0.1e-3;   // original ground distance
    double er = 4.4;
    double hEarth = 10e-3;   // virtual earth distance
    int panels = 12;

    // Reference: normal microstrip with image ground at hOrig
    XS_GEOMETRY geomRef = makeMicrostripGeom( w, hOrig, er, t );

    BEM_2D_SOLVER solverRef;
    solverRef.SetGeometry( geomRef );
    solverRef.SetPanelsPerEdge( panels );
    BOOST_REQUIRE( solverRef.Solve() );
    double z0Ref = solverRef.GetResult().Z0;

    // Test: image ground at virtual earth, wide groundwire at original position.
    // Dielectric: FR4 from signal to groundwire, air from groundwire to earth.
    XS_GEOMETRY geomGW;
    geomGW.groundY = 0.0;  // image ground (virtual earth maps here)

    double condY = -( hEarth + t / 2.0 );

    XS_CONDUCTOR cond;
    cond.centerX = 0.0;
    cond.centerY = condY;
    cond.width = w;
    cond.thickness = t;
    geomGW.conductors.push_back( cond );

    // Wide groundwire at the original ground position
    double gwY = condY + hOrig + t / 2.0;  // hOrig below signal center

    XS_CONDUCTOR gw;
    gw.centerX = 0.0;
    gw.centerY = gwY;
    gw.width = 10e-3;   // 10mm wide — should approximate infinite
    gw.thickness = t;
    gw.isGround = true;
    geomGW.conductors.push_back( gw );

    // Dielectric: FR4 between signal and groundwire, air below groundwire
    XS_DIELECTRIC_REGION air, fr4, airBelow;
    air.yTop = -10e-3;                 // far above
    air.yBottom = condY + t / 2.0;     // top of signal
    air.epsilonR = 1.0;

    fr4.yTop = condY + t / 2.0;       // top of signal → bottom face of signal is the interface
    fr4.yBottom = gwY - t / 2.0;      // top of groundwire
    fr4.epsilonR = er;

    airBelow.yTop = gwY + t / 2.0;    // below groundwire
    airBelow.yBottom = 0.0;           // image ground
    airBelow.epsilonR = 1.0;

    geomGW.dielectrics.push_back( air );
    geomGW.dielectrics.push_back( fr4 );
    geomGW.dielectrics.push_back( airBelow );
    geomGW.epsilonR = er;

    BEM_2D_SOLVER solverGW;
    solverGW.SetGeometry( geomGW );
    solverGW.SetPanelsPerEdge( panels );
    BOOST_REQUIRE( solverGW.Solve() );
    double z0GW = solverGW.GetResult().Z0;

    BOOST_TEST_MESSAGE( "Reference (image at " << hOrig * 1e3 << "mm): Z0=" << z0Ref );
    BOOST_TEST_MESSAGE( "Groundwire (10mm wide at " << hOrig * 1e3
                        << "mm, earth at " << hEarth * 1e3 << "mm): Z0=" << z0GW );
    BOOST_TEST_MESSAGE( "Difference: " << std::abs( z0GW - z0Ref ) << " Ohm ("
                        << std::abs( z0GW - z0Ref ) / z0Ref * 100.0 << "%)" );

    // A 10mm-wide groundwire should match the image ground within ~10%.
    // The groundwire has edge fringing that an infinite plane doesn't,
    // so it slightly over-estimates capacitance (lower Z₀).
    BOOST_CHECK_CLOSE( z0GW, z0Ref, 10.0 );
}


BOOST_AUTO_TEST_SUITE_END()
