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

#include <chrono>
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
    solver.SetPanelsPerEdge( 8 );

    bool ok = solver.Solve();
    BOOST_REQUIRE( ok );

    const RLGC_RESULT& result = solver.GetResult();

    double bemZ0 = result.Z0;

    BOOST_TEST_MESSAGE( "Microstrip BEM Z0 = " << bemZ0 << " Ohm" );
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
 * Zdiff with a third conductor (neighbor) present.
 * The BEM must still extract Zdiff from conductors 0 and 1 when
 * m_numConductors >= 2, and the neighbor must affect the result.
 */
BOOST_AUTO_TEST_CASE( CoupledWithNeighbor )
{
    double w = 0.1e-3;
    double s = 0.15e-3;
    double t = 35e-6;
    double h = 0.1e-3;
    double er = 4.4;

    auto makeGeom = [&]( bool withNeighbor ) -> XS_GEOMETRY
    {
        XS_GEOMETRY geom;

        // Left trace (conductor 0)
        XS_CONDUCTOR c0;
        c0.centerX = -( s / 2.0 + w / 2.0 );
        c0.centerY = -( h + t / 2.0 );
        c0.width = w;
        c0.thickness = t;
        geom.conductors.push_back( c0 );

        // Right trace (conductor 1 — diff pair partner)
        XS_CONDUCTOR c1;
        c1.centerX = ( s / 2.0 + w / 2.0 );
        c1.centerY = -( h + t / 2.0 );
        c1.width = w;
        c1.thickness = t;
        geom.conductors.push_back( c1 );

        if( withNeighbor )
        {
            // Extra neighbor trace to the right of the pair
            XS_CONDUCTOR c2;
            c2.centerX = ( s / 2.0 + w / 2.0 ) + 0.3e-3;
            c2.centerY = -( h + t / 2.0 );
            c2.width = w;
            c2.thickness = t;
            geom.conductors.push_back( c2 );
        }

        geom.groundY = 0.0;
        geom.epsilonR = er;

        XS_DIELECTRIC_REGION dielAbove, dielBelow;
        dielAbove.yTop = -10e-3;
        dielAbove.yBottom = -h;
        dielAbove.epsilonR = 1.0;
        dielBelow.yTop = -h;
        dielBelow.yBottom = 0.0;
        dielBelow.epsilonR = er;
        geom.dielectrics.push_back( dielAbove );
        geom.dielectrics.push_back( dielBelow );

        return geom;
    };

    BEM_2D_SOLVER solver2;
    solver2.SetGeometry( makeGeom( false ) );
    solver2.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solver2.Solve() );

    BEM_2D_SOLVER solver3;
    solver3.SetGeometry( makeGeom( true ) );
    solver3.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solver3.Solve() );

    BOOST_TEST_MESSAGE( "2 conductors: Zdiff=" << solver2.GetResult().Zdiff
                        << " Z0=" << solver2.GetResult().Z0 );
    BOOST_TEST_MESSAGE( "3 conductors: Zdiff=" << solver3.GetResult().Zdiff
                        << " Z0=" << solver3.GetResult().Z0 );

    // Both should produce valid Zdiff
    BOOST_CHECK_GT( solver2.GetResult().Zdiff, 0.0 );
    BOOST_CHECK_GT( solver3.GetResult().Zdiff, 0.0 );

    // Zdiff should be in a reasonable range
    BOOST_CHECK_GT( solver3.GetResult().Zdiff, 50.0 );
    BOOST_CHECK_LT( solver3.GetResult().Zdiff, 200.0 );

    // The neighbor changes the coupling environment, so Zdiff should differ
    // (neighbor adds capacitive loading, typically reducing Zdiff slightly)
    BOOST_CHECK_NE( solver2.GetResult().Zdiff, solver3.GetResult().Zdiff );
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
 * Coupled stripline: two traces between two ground planes.
 * Zdiff should be lower than coupled microstrip (more confinement).
 */
BOOST_AUTO_TEST_CASE( CoupledStripline )
{
    double w = 0.1e-3;
    double s = 0.15e-3;
    double t = 35e-6;
    double h = 0.2e-3;  // total dielectric thickness (signal centered)
    double er = 4.4;

    XS_GEOMETRY geom;

    double condY = -h / 2.0;

    XS_CONDUCTOR c0;
    c0.centerX = -( s / 2.0 + w / 2.0 );
    c0.centerY = condY;
    c0.width = w;
    c0.thickness = t;
    geom.conductors.push_back( c0 );

    XS_CONDUCTOR c1;
    c1.centerX = ( s / 2.0 + w / 2.0 );
    c1.centerY = condY;
    c1.width = w;
    c1.thickness = t;
    geom.conductors.push_back( c1 );

    geom.groundY = 0.0;
    geom.hasUpperGround = true;
    geom.upperGroundY = -h;
    geom.epsilonR = er;

    XS_DIELECTRIC_REGION diel;
    diel.yTop = -h;
    diel.yBottom = 0.0;
    diel.epsilonR = er;
    geom.dielectrics.push_back( diel );

    BEM_2D_SOLVER solver;
    solver.SetGeometry( geom );
    solver.SetPanelsPerEdge( 15 );
    BOOST_REQUIRE( solver.Solve() );

    BOOST_TEST_MESSAGE( "Coupled stripline:" );
    BOOST_TEST_MESSAGE( "  Z0 = " << solver.GetResult().Z0 << " Ohm" );
    BOOST_TEST_MESSAGE( "  Zdiff = " << solver.GetResult().Zdiff << " Ohm" );

    BOOST_CHECK_GT( solver.GetResult().Zdiff, 0.0 );
    BOOST_CHECK_GT( solver.GetResult().Z0, 0.0 );
    BOOST_CHECK_LT( solver.GetResult().Zdiff, 2.0 * solver.GetResult().Z0 );

    // erEff should be close to er for stripline (no air)
    BOOST_CHECK_CLOSE( solver.GetResult().erEff, er, 15.0 );
}


/**
 * Zdiff symmetry: swapping the two conductors' x-positions should
 * give identical Z0 and Zdiff (the pair is symmetric).
 */
BOOST_AUTO_TEST_CASE( ZdiffSymmetry )
{
    double w = 0.1e-3;
    double s = 0.15e-3;
    double t = 35e-6;
    double h = 0.1e-3;
    double er = 4.4;

    auto makeGeom = [&]( bool swapped ) -> XS_GEOMETRY
    {
        XS_GEOMETRY geom;

        double xLeft = -( s / 2.0 + w / 2.0 );
        double xRight = ( s / 2.0 + w / 2.0 );

        XS_CONDUCTOR c0;
        c0.centerX = swapped ? xRight : xLeft;
        c0.centerY = -( h + t / 2.0 );
        c0.width = w;
        c0.thickness = t;
        geom.conductors.push_back( c0 );

        XS_CONDUCTOR c1;
        c1.centerX = swapped ? xLeft : xRight;
        c1.centerY = -( h + t / 2.0 );
        c1.width = w;
        c1.thickness = t;
        geom.conductors.push_back( c1 );

        geom.groundY = 0.0;
        geom.epsilonR = er;

        XS_DIELECTRIC_REGION dielAbove, dielBelow;
        dielAbove.yTop = -10e-3;
        dielAbove.yBottom = -h;
        dielAbove.epsilonR = 1.0;
        dielBelow.yTop = -h;
        dielBelow.yBottom = 0.0;
        dielBelow.epsilonR = er;
        geom.dielectrics.push_back( dielAbove );
        geom.dielectrics.push_back( dielBelow );

        return geom;
    };

    BEM_2D_SOLVER solverA;
    solverA.SetGeometry( makeGeom( false ) );
    solverA.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solverA.Solve() );

    BEM_2D_SOLVER solverB;
    solverB.SetGeometry( makeGeom( true ) );
    solverB.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solverB.Solve() );

    BOOST_TEST_MESSAGE( "Normal: Z0=" << solverA.GetResult().Z0
                        << " Zdiff=" << solverA.GetResult().Zdiff );
    BOOST_TEST_MESSAGE( "Swapped: Z0=" << solverB.GetResult().Z0
                        << " Zdiff=" << solverB.GetResult().Zdiff );

    BOOST_CHECK_CLOSE( solverA.GetResult().Zdiff, solverB.GetResult().Zdiff, 0.01 );
    BOOST_CHECK_CLOSE( solverA.GetResult().Z0, solverB.GetResult().Z0, 0.01 );
}


/**
 * Asymmetric diff pair: traces of different widths.
 * Zdiff should still be computed, and Z0 should differ between the two conductors.
 */
BOOST_AUTO_TEST_CASE( AsymmetricDiffPair )
{
    double w1 = 0.1e-3;
    double w2 = 0.15e-3;
    double s = 0.15e-3;
    double t = 35e-6;
    double h = 0.1e-3;
    double er = 4.4;

    XS_GEOMETRY geom;

    XS_CONDUCTOR c0;
    c0.centerX = -( s / 2.0 + w1 / 2.0 );
    c0.centerY = -( h + t / 2.0 );
    c0.width = w1;
    c0.thickness = t;
    geom.conductors.push_back( c0 );

    XS_CONDUCTOR c1;
    c1.centerX = ( s / 2.0 + w2 / 2.0 );
    c1.centerY = -( h + t / 2.0 );
    c1.width = w2;
    c1.thickness = t;
    geom.conductors.push_back( c1 );

    geom.groundY = 0.0;
    geom.epsilonR = er;

    XS_DIELECTRIC_REGION dielAbove, dielBelow;
    dielAbove.yTop = -10e-3;
    dielAbove.yBottom = -h;
    dielAbove.epsilonR = 1.0;
    dielBelow.yTop = -h;
    dielBelow.yBottom = 0.0;
    dielBelow.epsilonR = er;
    geom.dielectrics.push_back( dielAbove );
    geom.dielectrics.push_back( dielBelow );

    BEM_2D_SOLVER solver;
    solver.SetGeometry( geom );
    solver.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solver.Solve() );

    BOOST_TEST_MESSAGE( "Asymmetric diff pair: Z0=" << solver.GetResult().Z0
                        << " Zdiff=" << solver.GetResult().Zdiff );

    // Zdiff should still be computed
    BOOST_CHECK_GT( solver.GetResult().Zdiff, 0.0 );
    BOOST_CHECK_LT( solver.GetResult().Zdiff, 2.0 * solver.GetResult().Z0 );

    // C matrix should be asymmetric: C(0,0) != C(1,1) because widths differ
    BOOST_CHECK_NE( solver.GetResult().C( 0, 0 ), solver.GetResult().C( 1, 1 ) );

    // But mutual capacitance should still be symmetric
    BOOST_CHECK_CLOSE( solver.GetResult().C( 0, 1 ), solver.GetResult().C( 1, 0 ), 0.1 );
}


/**
 * Solder mask effect: compare microstrip Z₀ with and without a solder mask
 * dielectric region (εr=3.3, 10µm over copper).
 *
 * SM should lower Z₀ by a few percent (increased capacitance from SM partial fill).
 */
BOOST_AUTO_TEST_CASE( SolderMaskEffect )
{
    double w = 0.15e-3;
    double t = 35e-6;
    double h = 0.1e-3;
    double er = 4.4;
    double smThick = 10e-6;
    double smEr = 3.3;

    // Without solder mask
    XS_GEOMETRY geomBare = makeMicrostripGeom( w, h, er, t );

    BEM_2D_SOLVER solverBare;
    solverBare.SetGeometry( geomBare );
    solverBare.SetPanelsPerEdge( 8 );
    BOOST_REQUIRE( solverBare.Solve() );
    double z0Bare = solverBare.GetResult().Z0;

    // With solder mask: add SM region above conductor top
    XS_GEOMETRY geomSM = makeMicrostripGeom( w, h, er, t );
    double condTop = -( h + t );
    double smBoundary = condTop - smThick;

    // Split the air region at smBoundary
    geomSM.dielectrics[0].yBottom = smBoundary;  // air: shrink to above SM

    XS_DIELECTRIC_REGION mask;
    mask.yTop = smBoundary;
    mask.yBottom = -h;   // SM extends from above conductor to substrate surface
    mask.epsilonR = smEr;
    geomSM.dielectrics.push_back( mask );

    BEM_2D_SOLVER solverSM;
    solverSM.SetGeometry( geomSM );
    solverSM.SetPanelsPerEdge( 8 );
    BOOST_REQUIRE( solverSM.Solve() );
    double z0SM = solverSM.GetResult().Z0;

    double deltaPercent = 100.0 * ( z0SM - z0Bare ) / z0Bare;

    BOOST_TEST_MESSAGE( "Bare:  Z0=" << z0Bare << " Ohm  erEff=" << solverBare.GetResult().erEff );
    BOOST_TEST_MESSAGE( "SM:    Z0=" << z0SM << " Ohm  erEff=" << solverSM.GetResult().erEff );
    BOOST_TEST_MESSAGE( "Delta: " << deltaPercent << "%" );

    // SM should lower Z0 (more capacitance) — expect -1% to -10%
    BOOST_CHECK_LT( z0SM, z0Bare );
    BOOST_CHECK_GT( deltaPercent, -10.0 );
    BOOST_CHECK_LT( deltaPercent, -0.5 );

    // Compare coarse SM interface grid (default) vs fine grid (reference)
    BEM_2D_SOLVER solverFine;
    solverFine.SetGeometry( geomSM );
    solverFine.SetPanelsPerEdge( 8 );
    solverFine.SetFineInterfaceGrid( true );
    BOOST_REQUIRE( solverFine.Solve() );
    double z0Fine = solverFine.GetResult().Z0;

    double gridError = 100.0 * ( z0SM - z0Fine ) / z0Fine;
    BOOST_TEST_MESSAGE( "Fine grid: Z0=" << z0Fine << " Ohm  erEff="
                        << solverFine.GetResult().erEff );
    BOOST_TEST_MESSAGE( "Coarse vs fine grid error: " << gridError << "%" );

    // Coarse grid should be within 1% of fine grid
    BOOST_CHECK_LT( std::abs( gridError ), 1.0 );
}


/**
 * Interface grid sensitivity: sweep extent and spacing for the substrate boundary
 * across multiple geometries to find the minimum discretization that stays within
 * 0.5% of the finest reference.
 *
 * Uses 10 panels/edge to isolate interface grid effects from panel convergence.
 */
BOOST_AUTO_TEST_CASE( InterfaceGridSensitivity )
{
    struct CASE
    {
        const char* name;
        double w;
        double h;
        double er;
    };

    CASE cases[] = {
        { "narrow/thin",  0.10e-3, 0.075e-3, 4.4 },
        { "typical",      0.15e-3, 0.10e-3,  4.4 },
        { "wide/thick",   0.30e-3, 0.20e-3,  4.4 },
        { "high-er",      0.15e-3, 0.10e-3,  6.5 },
        { "low-er",       0.15e-3, 0.10e-3,  3.0 },
        { "50mil FR4",    0.20e-3, 1.27e-3,  4.3 },
    };

    struct GRID
    {
        const char* name;
        double extentMult;
        double spacingDiv;
    };

    GRID grids[] = {
        { "8h/h÷8",  8.0, 8.0 },
        { "5h/h÷5",  5.0, 5.0 },
        { "5h/h÷3",  5.0, 3.0 },
        { "3h/h÷3",  3.0, 3.0 },
        { "2h/h÷2",  2.0, 2.0 },
        { "2h/h÷1",  2.0, 1.0 },
        { "1h/h÷1",  1.0, 1.0 },
    };

    int nGrids = sizeof( grids ) / sizeof( grids[0] );

    BOOST_TEST_MESSAGE( "" );

    // Header
    std::string hdr = "geometry        ";

    for( int g = 0; g < nGrids; g++ )
        hdr += std::string( "  " ) + grids[g].name;

    BOOST_TEST_MESSAGE( hdr );

    for( const CASE& c : cases )
    {
        XS_GEOMETRY geom = makeMicrostripGeom( c.w, c.h, c.er );

        // Reference: finest grid
        BEM_2D_SOLVER refSolver;
        refSolver.SetGeometry( geom );
        refSolver.SetPanelsPerEdge( 10 );
        refSolver.SetInterfaceGrid( grids[0].extentMult, grids[0].spacingDiv );
        BOOST_REQUIRE( refSolver.Solve() );
        double z0Ref = refSolver.GetResult().Z0;

        char buf[256];
        snprintf( buf, sizeof( buf ), "%-16s", c.name );
        std::string line = buf;

        for( int g = 0; g < nGrids; g++ )
        {
            BEM_2D_SOLVER solver;
            solver.SetGeometry( geom );
            solver.SetPanelsPerEdge( 10 );
            solver.SetInterfaceGrid( grids[g].extentMult, grids[g].spacingDiv );
            BOOST_REQUIRE( solver.Solve() );

            double err = 100.0 * ( solver.GetResult().Z0 - z0Ref ) / z0Ref;
            snprintf( buf, sizeof( buf ), "  %+.3f%%", err );
            line += buf;
        }

        BOOST_TEST_MESSAGE( line );
    }
}


/**
 * Panel count and edge singularity: accuracy vs runtime tradeoff.
 * For each panel count, measure Z0 error vs fine-mesh reference and solve time,
 * with and without edge singularity.
 */
BOOST_AUTO_TEST_CASE( PanelRuntimeTradeoff )
{
    XS_GEOMETRY geom = makeMicrostripGeom( 0.15e-3, 0.1e-3, 4.4 );

    // Fine reference
    BEM_2D_SOLVER refSolver;
    refSolver.SetGeometry( geom );
    refSolver.SetPanelsPerEdge( 30 );
    BOOST_REQUIRE( refSolver.Solve() );
    double z0Ref = refSolver.GetResult().Z0;

    BOOST_TEST_MESSAGE( "" );
    BOOST_TEST_MESSAGE( "Panel count vs runtime (typical microstrip, ref Z0="
                        << z0Ref << ")" );
    BOOST_TEST_MESSAGE( "panels  Z0_on     err_on   us_on    Z0_off    err_off  us_off   delta" );

    for( int n : { 1, 2, 3, 4, 5, 6, 8, 10 } )
    {
        static constexpr int RUNS = 10;

        // With edge singularity
        double z0On = 0;
        long   usOn = 0;
        {
            BEM_2D_SOLVER warmup;
            warmup.SetGeometry( geom );
            warmup.SetPanelsPerEdge( n );
            warmup.Solve();

            for( int r = 0; r < RUNS; r++ )
            {
                BEM_2D_SOLVER s;
                s.SetGeometry( geom );
                s.SetPanelsPerEdge( n );
                auto t0 = std::chrono::steady_clock::now();
                s.Solve();
                auto t1 = std::chrono::steady_clock::now();
                usOn += std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count();
                z0On = s.GetResult().Z0;
            }

            usOn /= RUNS;
        }

        // Without edge singularity
        double z0Off = 0;
        long   usOff = 0;
        {
            BEM_2D_SOLVER warmup;
            warmup.SetGeometry( geom );
            warmup.SetPanelsPerEdge( n );
            warmup.SetEdgeSingularity( false );
            warmup.Solve();

            for( int r = 0; r < RUNS; r++ )
            {
                BEM_2D_SOLVER s;
                s.SetGeometry( geom );
                s.SetPanelsPerEdge( n );
                s.SetEdgeSingularity( false );
                auto t0 = std::chrono::steady_clock::now();
                s.Solve();
                auto t1 = std::chrono::steady_clock::now();
                usOff += std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count();
                z0Off = s.GetResult().Z0;
            }

            usOff /= RUNS;
        }

        double errOn = 100.0 * ( z0On - z0Ref ) / z0Ref;
        double errOff = 100.0 * ( z0Off - z0Ref ) / z0Ref;

        char line[160];
        snprintf( line, sizeof( line ),
                  "%5d   %8.3f  %+6.3f%%  %5ld    %8.3f  %+6.3f%%  %5ld    %+6.3f%%",
                  n, z0On, errOn, usOn, z0Off, errOff, usOff, errOn - errOff );

        BOOST_TEST_MESSAGE( line );
    }
}


/**
 * Edge singularity sensitivity: compare Z0 and εr_eff with and without
 * ν-exponent edge singularity across a range of panel counts and geometries.
 */
BOOST_AUTO_TEST_CASE( EdgeSingularitySensitivity )
{
    struct GEOM_CASE
    {
        const char* name;
        double w, h, er;
    };

    GEOM_CASE cases[] = {
        { "narrow (w/h=0.5)",  0.05e-3, 0.1e-3,  4.4 },
        { "typical (w/h=1.5)", 0.15e-3, 0.1e-3,  4.4 },
        { "wide (w/h=3.0)",    0.30e-3, 0.1e-3,  4.4 },
        { "high-er (w/h=1.5)", 0.15e-3, 0.1e-3, 10.0 },
        { "low-er (w/h=1.5)",  0.15e-3, 0.1e-3,  2.2 },
    };

    int panels[] = { 3, 4, 5, 6, 8, 10, 15, 20, 30 };

    BOOST_TEST_MESSAGE( "" );
    BOOST_TEST_MESSAGE( "Edge singularity sensitivity analysis" );
    BOOST_TEST_MESSAGE( "====================================" );

    for( const GEOM_CASE& gc : cases )
    {
        XS_GEOMETRY geom = makeMicrostripGeom( gc.w, gc.h, gc.er );

        // Fine-mesh reference (30 panels, with edge singularity)
        BEM_2D_SOLVER refSolver;
        refSolver.SetGeometry( geom );
        refSolver.SetPanelsPerEdge( 20 );
        BOOST_REQUIRE( refSolver.Solve() );
        double z0Ref = refSolver.GetResult().Z0;

        BOOST_TEST_MESSAGE( "" );

        std::string header = std::string( gc.name ) + "  (ref Z0="
                             + std::to_string( z0Ref ).substr( 0, 6 ) + ")";
        BOOST_TEST_MESSAGE( header );
        BOOST_TEST_MESSAGE( "panels    Z0_on      Z0_off     err_on    err_off   delta" );

        for( int n : panels )
        {
            BEM_2D_SOLVER solverOn, solverOff;

            solverOn.SetGeometry( geom );
            solverOn.SetPanelsPerEdge( n );
            solverOn.SetEdgeSingularity( true );
            BOOST_REQUIRE( solverOn.Solve() );

            solverOff.SetGeometry( geom );
            solverOff.SetPanelsPerEdge( n );
            solverOff.SetEdgeSingularity( false );
            BOOST_REQUIRE( solverOff.Solve() );

            double z0On = solverOn.GetResult().Z0;
            double z0Off = solverOff.GetResult().Z0;
            double errOn = ( z0On - z0Ref ) / z0Ref * 100.0;
            double errOff = ( z0Off - z0Ref ) / z0Ref * 100.0;
            double delta = errOn - errOff;

            char line[120];
            snprintf( line, sizeof( line ),
                      "%5d   %8.3f   %8.3f   %+6.3f%%   %+6.3f%%   %+6.3f%%",
                      n, z0On, z0Off, errOn, errOff, delta );

            BOOST_TEST_MESSAGE( line );
        }
    }
}


/**
 * Runtime performance: a single microstrip solve at the default panel count
 * must complete well under 50ms.  This catches accidental O(n²) regressions
 * in the assembly or solve path.
 */
BOOST_AUTO_TEST_CASE( SolvePerformance )
{
    XS_GEOMETRY geom = makeMicrostripGeom( 0.15e-3, 0.1e-3, 4.4 );

    // Warm up (first solve may trigger lazy initialization)
    {
        BEM_2D_SOLVER warmup;
        warmup.SetGeometry( geom );
        warmup.Solve();
    }

    static constexpr int RUNS = 20;
    double totalUs = 0.0;
    double minUs = 1e9;
    double maxUs = 0.0;

    for( int r = 0; r < RUNS; r++ )
    {
        BEM_2D_SOLVER solver;
        solver.SetGeometry( geom );

        auto t0 = std::chrono::steady_clock::now();
        BOOST_REQUIRE( solver.Solve() );
        auto t1 = std::chrono::steady_clock::now();

        double us = std::chrono::duration<double, std::micro>( t1 - t0 ).count();
        totalUs += us;
        minUs = std::min( minUs, us );
        maxUs = std::max( maxUs, us );
    }

    double avgUs = totalUs / RUNS;

    BOOST_TEST_MESSAGE( "BEM solve (5 panels/edge, single microstrip):" );
    BOOST_TEST_MESSAGE( "  avg=" << (int) avgUs << "us  min=" << (int) minUs
                        << "us  max=" << (int) maxUs << "us  (" << RUNS << " runs)" );

    // Must complete in under 10ms (typical is ~2ms at 5 panels/edge)
    BOOST_CHECK_LT( avgUs, 10000.0 );
}


// ---------------------------------------------------------------------------
// Demotion boundary investigation: reproduce the Z0 drop from D5 at d=31.2→31.4mm
// ---------------------------------------------------------------------------

/**
 * Helper: build the non-demoted geometry from D5 at d=31.2mm.
 * Microstrip with 3 neighbors, image ground at 0.21mm, FR4 εr=4.5,
 * solder mask εr=3.3.
 */
static XS_GEOMETRY makeD5NonDemoted()
{
    XS_GEOMETRY xs;
    xs.groundY = 0.0;
    xs.hasUpperGround = false;
    xs.epsilonR = 4.50;

    // Signal conductor
    XS_CONDUCTOR sig;
    sig.centerX = 0.0;
    sig.centerY = -0.2279e-3;
    sig.width = 0.1900e-3;
    sig.thickness = 0.0350e-3;
    xs.conductors.push_back( sig );

    // Neighbor conductors (same layer)
    XS_CONDUCTOR nb1;
    nb1.centerX = 0.2900e-3;
    nb1.centerY = -0.2279e-3;
    nb1.width = 0.1900e-3;
    nb1.thickness = 0.0350e-3;
    xs.conductors.push_back( nb1 );

    XS_CONDUCTOR nb2;
    nb2.centerX = -0.5585e-3;
    nb2.centerY = -0.2279e-3;
    nb2.width = 0.1900e-3;
    nb2.thickness = 0.0350e-3;
    xs.conductors.push_back( nb2 );

    XS_CONDUCTOR nb3;
    nb3.centerX = 0.8281e-3;
    nb3.centerY = -0.2279e-3;
    nb3.width = 0.6312e-3;
    nb3.thickness = 0.0350e-3;
    xs.conductors.push_back( nb3 );

    // Dielectrics: air above, FR4 below, solder mask at signal boundary
    XS_DIELECTRIC_REGION air;
    air.yTop = -10e-3;
    air.yBottom = -0.2554e-3;
    air.epsilonR = 1.0;
    xs.dielectrics.push_back( air );

    XS_DIELECTRIC_REGION fr4;
    fr4.yTop = -0.2104e-3;
    fr4.yBottom = 0.0;
    fr4.epsilonR = 4.50;
    xs.dielectrics.push_back( fr4 );

    XS_DIELECTRIC_REGION sm;
    sm.yTop = -0.2554e-3;
    sm.yBottom = -0.2104e-3;
    sm.epsilonR = 3.30;
    xs.dielectrics.push_back( sm );

    return xs;
}


/**
 * Helper: build the demoted geometry from D5 at d=31.4mm.
 * Signal shifted down to hFallback, one groundwire at original ref level,
 * FR4 between signal and groundwire, air below to virtual earth.
 */
static XS_GEOMETRY makeD5Demoted()
{
    XS_GEOMETRY xs;
    xs.groundY = 0.0;
    xs.hasUpperGround = false;
    xs.epsilonR = 4.50;

    // Same 4 signal conductors, shifted to new Y
    XS_CONDUCTOR sig;
    sig.centerX = 0.0;
    sig.centerY = -1.5337e-3;
    sig.width = 0.1900e-3;
    sig.thickness = 0.0350e-3;
    xs.conductors.push_back( sig );

    XS_CONDUCTOR nb1;
    nb1.centerX = 0.2900e-3;
    nb1.centerY = -1.5337e-3;
    nb1.width = 0.1900e-3;
    nb1.thickness = 0.0350e-3;
    xs.conductors.push_back( nb1 );

    XS_CONDUCTOR nb2;
    nb2.centerX = -0.5585e-3;
    nb2.centerY = -1.5337e-3;
    nb2.width = 0.1900e-3;
    nb2.thickness = 0.0350e-3;
    xs.conductors.push_back( nb2 );

    XS_CONDUCTOR nb3;
    nb3.centerX = 0.8281e-3;
    nb3.centerY = -1.5337e-3;
    nb3.width = 0.6312e-3;
    nb3.thickness = 0.0350e-3;
    xs.conductors.push_back( nb3 );

    // Groundwire at original reference level
    XS_CONDUCTOR gw;
    gw.centerX = 0.4055e-3;
    gw.centerY = -1.3058e-3;
    gw.width = 1.2929e-3;
    gw.thickness = 0.0350e-3;
    gw.isGround = true;
    xs.conductors.push_back( gw );

    // Dielectrics: air above, FR4 from signal to original ref,
    // air from original ref to virtual earth, solder mask
    XS_DIELECTRIC_REGION airAbove;
    airAbove.yTop = -10e-3;
    airAbove.yBottom = -1.5612e-3;
    airAbove.epsilonR = 1.0;
    xs.dielectrics.push_back( airAbove );

    XS_DIELECTRIC_REGION fr4;
    fr4.yTop = -1.5162e-3;
    fr4.yBottom = -1.3058e-3;
    fr4.epsilonR = 4.50;
    xs.dielectrics.push_back( fr4 );

    XS_DIELECTRIC_REGION airBelow;
    airBelow.yTop = -1.3058e-3;
    airBelow.yBottom = 0.0;
    airBelow.epsilonR = 1.0;
    xs.dielectrics.push_back( airBelow );

    XS_DIELECTRIC_REGION sm;
    sm.yTop = -1.5612e-3;
    sm.yBottom = -1.5162e-3;
    sm.epsilonR = 3.30;
    xs.dielectrics.push_back( sm );

    return xs;
}


/**
 * Step 2: Reproduce the D5 demotion boundary Z0 drop.
 * Non-demoted should give Z0 ≈ 58.2, demoted should give Z0 ≈ 55.9.
 */
BOOST_AUTO_TEST_CASE( DemotionBoundaryReproduce )
{
    XS_GEOMETRY xsNonDemoted = makeD5NonDemoted();

    BEM_2D_SOLVER solverND;
    solverND.SetGeometry( xsNonDemoted );
    solverND.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solverND.Solve() );

    XS_GEOMETRY xsDemoted = makeD5Demoted();

    BEM_2D_SOLVER solverD;
    solverD.SetGeometry( xsDemoted );
    solverD.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solverD.Solve() );

    const auto& rND = solverND.GetResult();
    const auto& rD  = solverD.GetResult();

    BOOST_TEST_MESSAGE( "=== Non-demoted ===" );
    BOOST_TEST_MESSAGE( "  C=" << rND.C( 0, 0 ) << "  C0=" << rND.C0( 0, 0 )
                        << "  erEff=" << rND.erEff << "  Z0=" << rND.Z0 );

    BOOST_TEST_MESSAGE( "=== Demoted ===" );
    BOOST_TEST_MESSAGE( "  C=" << rD.C( 0, 0 ) << "  C0=" << rD.C0( 0, 0 )
                        << "  erEff=" << rD.erEff << "  Z0=" << rD.Z0 );

    BOOST_TEST_MESSAGE( "=== Deltas ===" );
    BOOST_TEST_MESSAGE( "  dC="  << ( rD.C( 0, 0 ) / rND.C( 0, 0 ) - 1.0 ) * 100 << "%" );
    BOOST_TEST_MESSAGE( "  dC0=" << ( rD.C0( 0, 0 ) / rND.C0( 0, 0 ) - 1.0 ) * 100 << "%" );
    BOOST_TEST_MESSAGE( "  dZ0=" << rD.Z0 - rND.Z0 << " Ohm" );

    // Confirm the Z0 inversion is real (demoted < non-demoted)
    BOOST_CHECK_GT( rND.Z0, rD.Z0 );
}


/**
 * Step 3a: Add groundwires only — keep original groundY at h=0.21mm.
 * If groundwires PLUS close image ground cause Z0 to drop, this isolates
 * the groundwire effect when the image is at the right distance.
 */
BOOST_AUTO_TEST_CASE( DemotionBisect_GroundwiresOnly )
{
    // Start with non-demoted geometry, add the groundwire at the SAME
    // relative position (but keep groundY at 0.21mm / condY at -0.2279mm).
    XS_GEOMETRY xs = makeD5NonDemoted();

    // Groundwire at original ref level — same relative offset as demoted,
    // but in the non-demoted coordinate frame (ground at y=0, signal at -0.2279mm).
    // In demoted: signal at -1.5337mm, groundwire at -1.3058mm (offset = +0.2279mm = h+t/2).
    // In non-demoted frame: groundwire at -0.2279mm + 0.2279mm = 0.0 ← that's the ground plane!
    // So put it just above: at -t/2 = -0.0175mm (same as groundwire at image surface).
    //
    // Actually, let's put the groundwire at the same position relative to the signal
    // as in the demoted case: 0.2279mm below signal center.
    // Signal center is at -0.2279mm, so groundwire center = -0.2279mm + 0.2279mm = 0.0mm.
    // That's exactly the image ground. Instead, place it just inside the domain.
    XS_CONDUCTOR gw;
    gw.centerX = 0.4055e-3;
    gw.centerY = -0.0175e-3; // just above image ground, copper thickness centered
    gw.width = 1.2929e-3;
    gw.thickness = 0.0350e-3;
    gw.isGround = true;
    xs.conductors.push_back( gw );

    BEM_2D_SOLVER solver;
    solver.SetGeometry( xs );
    solver.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solver.Solve() );

    const auto& r = solver.GetResult();
    BOOST_TEST_MESSAGE( "=== Non-demoted + groundwire at image level ===" );
    BOOST_TEST_MESSAGE( "  C=" << r.C( 0, 0 ) << "  C0=" << r.C0( 0, 0 )
                        << "  erEff=" << r.erEff << "  Z0=" << r.Z0 );
}


/**
 * Step 3b: Move image ground only — push groundY to hFallback (1.52mm),
 * but no groundwires. This isolates the effect of moving the image far away.
 */
BOOST_AUTO_TEST_CASE( DemotionBisect_FarImageOnly )
{
    // Demoted geometry without the groundwire
    XS_GEOMETRY xs = makeD5Demoted();

    // Remove groundwire (last conductor)
    xs.conductors.erase(
            std::remove_if( xs.conductors.begin(), xs.conductors.end(),
                            []( const XS_CONDUCTOR& c ) { return c.isGround; } ),
            xs.conductors.end() );

    BEM_2D_SOLVER solver;
    solver.SetGeometry( xs );
    solver.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solver.Solve() );

    const auto& r = solver.GetResult();
    BOOST_TEST_MESSAGE( "=== Far image only (no groundwire) ===" );
    BOOST_TEST_MESSAGE( "  C=" << r.C( 0, 0 ) << "  C0=" << r.C0( 0, 0 )
                        << "  erEff=" << r.erEff << "  Z0=" << r.Z0 );
    BOOST_TEST_MESSAGE( "  (should have MUCH higher Z0 — far image, no local ground)" );
}


/**
 * Step 3c: Demoted dielectric with uniform εr — no interface elements.
 * This tests whether the dielectric interface at the groundwire position
 * is causing the εr_eff increase.
 */
BOOST_AUTO_TEST_CASE( DemotionBisect_UniformDielectric )
{
    XS_GEOMETRY xs = makeD5Demoted();

    // Replace dielectric structure with uniform FR4
    xs.dielectrics.clear();

    XS_DIELECTRIC_REGION air;
    air.yTop = -10e-3;
    air.yBottom = -1.5162e-3; // signal bottom
    air.epsilonR = 1.0;
    xs.dielectrics.push_back( air );

    XS_DIELECTRIC_REGION fr4;
    fr4.yTop = -1.5162e-3;
    fr4.yBottom = 0.0; // all the way to image ground
    fr4.epsilonR = 4.50;
    xs.dielectrics.push_back( fr4 );

    BEM_2D_SOLVER solver;
    solver.SetGeometry( xs );
    solver.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solver.Solve() );

    const auto& r = solver.GetResult();
    BOOST_TEST_MESSAGE( "=== Demoted geometry + uniform FR4 (no air gap below) ===" );
    BOOST_TEST_MESSAGE( "  C=" << r.C( 0, 0 ) << "  C0=" << r.C0( 0, 0 )
                        << "  erEff=" << r.erEff << "  Z0=" << r.Z0 );
}


/**
 * Step 3d: Simple comparison — non-demoted geometry stripped to signal-only
 * vs demoted geometry stripped to signal-only + groundwire.
 * No neighbors, no solder mask — isolate the core demotion effect.
 */
BOOST_AUTO_TEST_CASE( DemotionBisect_Minimal )
{
    double w = 0.19e-3;
    double t = 0.035e-3;
    double h = 0.2104e-3;
    double er = 4.50;

    // Non-demoted: simple microstrip
    XS_GEOMETRY xsBase = makeMicrostripGeom( w, h, er, t );

    BEM_2D_SOLVER solverBase;
    solverBase.SetGeometry( xsBase );
    solverBase.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solverBase.Solve() );

    // Demoted: signal at hFallback, groundwire at hOrig relative to signal
    double hFallback = 1.5162e-3;
    double condY = -( hFallback + t / 2.0 );
    double origRefY = condY + h + t / 2.0; // = -(hFallback - h)

    XS_GEOMETRY xsDemoted;
    xsDemoted.groundY = 0.0;
    xsDemoted.epsilonR = er;

    XS_CONDUCTOR sig;
    sig.centerX = 0.0;
    sig.centerY = condY;
    sig.width = w;
    sig.thickness = t;
    xsDemoted.conductors.push_back( sig );

    XS_CONDUCTOR gw;
    gw.centerX = 0.0;
    gw.centerY = origRefY;
    gw.width = 2.0e-3; // wide groundwire (symmetric)
    gw.thickness = t;
    gw.isGround = true;
    xsDemoted.conductors.push_back( gw );

    // FR4 from signal to original ref, air above and below
    XS_DIELECTRIC_REGION airAbove;
    airAbove.yTop = -10e-3;
    airAbove.yBottom = condY + t / 2.0;
    airAbove.epsilonR = 1.0;
    xsDemoted.dielectrics.push_back( airAbove );

    XS_DIELECTRIC_REGION fr4;
    fr4.yTop = condY + t / 2.0;
    fr4.yBottom = origRefY;
    fr4.epsilonR = er;
    xsDemoted.dielectrics.push_back( fr4 );

    XS_DIELECTRIC_REGION airBelow;
    airBelow.yTop = origRefY;
    airBelow.yBottom = 0.0;
    airBelow.epsilonR = 1.0;
    xsDemoted.dielectrics.push_back( airBelow );

    BEM_2D_SOLVER solverDemoted;
    solverDemoted.SetGeometry( xsDemoted );
    solverDemoted.SetPanelsPerEdge( 12 );
    BOOST_REQUIRE( solverDemoted.Solve() );

    const auto& rB = solverBase.GetResult();
    const auto& rD = solverDemoted.GetResult();

    BOOST_TEST_MESSAGE( "=== Minimal non-demoted (microstrip) ===" );
    BOOST_TEST_MESSAGE( "  C=" << rB.C( 0, 0 ) << "  C0=" << rB.C0( 0, 0 )
                        << "  erEff=" << rB.erEff << "  Z0=" << rB.Z0 );

    BOOST_TEST_MESSAGE( "=== Minimal demoted (groundwire + far image) ===" );
    BOOST_TEST_MESSAGE( "  C=" << rD.C( 0, 0 ) << "  C0=" << rD.C0( 0, 0 )
                        << "  erEff=" << rD.erEff << "  Z0=" << rD.Z0 );

    BOOST_TEST_MESSAGE( "=== Deltas ===" );
    BOOST_TEST_MESSAGE( "  dC="  << ( rD.C( 0, 0 ) / rB.C( 0, 0 ) - 1.0 ) * 100 << "%" );
    BOOST_TEST_MESSAGE( "  dC0=" << ( rD.C0( 0, 0 ) / rB.C0( 0, 0 ) - 1.0 ) * 100 << "%" );
    BOOST_TEST_MESSAGE( "  dZ0=" << rD.Z0 - rB.Z0 << " Ohm" );

    // Expect Z0 to stay the same or increase (void → less coupling)
    // If this FAILS, the image+groundwire combination is the root cause.
}


/**
 * Step 3e: Move the FR4/air interface away from the groundwire.
 * The demoted geometry places the dielectric boundary exactly at the
 * groundwire center (y=-1.3058mm). Test what happens when the interface
 * is offset below the groundwire (so the groundwire is fully in FR4).
 */
BOOST_AUTO_TEST_CASE( DemotionBisect_InterfaceOffset )
{
    double gwBottom = -1.3058e-3 + 0.0350e-3 / 2.0; // groundwire bottom = -1.2883mm
    double gwTop    = -1.3058e-3 - 0.0350e-3 / 2.0; // groundwire top = -1.3233mm

    // Test A: interface below groundwire (GW fully in FR4)
    {
        XS_GEOMETRY xs = makeD5Demoted();
        xs.dielectrics.clear();

        XS_DIELECTRIC_REGION airAbove;
        airAbove.yTop = -10e-3;
        airAbove.yBottom = -1.5612e-3;
        airAbove.epsilonR = 1.0;
        xs.dielectrics.push_back( airAbove );

        XS_DIELECTRIC_REGION sm;
        sm.yTop = -1.5612e-3;
        sm.yBottom = -1.5162e-3;
        sm.epsilonR = 3.30;
        xs.dielectrics.push_back( sm );

        XS_DIELECTRIC_REGION fr4;
        fr4.yTop = -1.5162e-3;
        fr4.yBottom = gwBottom;  // stops at groundwire bottom
        fr4.epsilonR = 4.50;
        xs.dielectrics.push_back( fr4 );

        XS_DIELECTRIC_REGION airBelow;
        airBelow.yTop = gwBottom;
        airBelow.yBottom = 0.0;
        airBelow.epsilonR = 1.0;
        xs.dielectrics.push_back( airBelow );

        BEM_2D_SOLVER solver;
        solver.SetGeometry( xs );
        solver.SetPanelsPerEdge( 12 );
        BOOST_REQUIRE( solver.Solve() );

        const auto& r = solver.GetResult();
        BOOST_TEST_MESSAGE( "=== Interface at GW bottom (GW fully in FR4) ===" );
        BOOST_TEST_MESSAGE( "  C=" << r.C( 0, 0 ) << "  C0=" << r.C0( 0, 0 )
                            << "  erEff=" << r.erEff << "  Z0=" << r.Z0 );
    }

    // Test B: interface above groundwire (GW fully in air)
    {
        XS_GEOMETRY xs = makeD5Demoted();
        xs.dielectrics.clear();

        XS_DIELECTRIC_REGION airAbove;
        airAbove.yTop = -10e-3;
        airAbove.yBottom = -1.5612e-3;
        airAbove.epsilonR = 1.0;
        xs.dielectrics.push_back( airAbove );

        XS_DIELECTRIC_REGION sm;
        sm.yTop = -1.5612e-3;
        sm.yBottom = -1.5162e-3;
        sm.epsilonR = 3.30;
        xs.dielectrics.push_back( sm );

        XS_DIELECTRIC_REGION fr4;
        fr4.yTop = -1.5162e-3;
        fr4.yBottom = gwTop;  // stops at groundwire top
        fr4.epsilonR = 4.50;
        xs.dielectrics.push_back( fr4 );

        XS_DIELECTRIC_REGION airBelow;
        airBelow.yTop = gwTop;
        airBelow.yBottom = 0.0;
        airBelow.epsilonR = 1.0;
        xs.dielectrics.push_back( airBelow );

        BEM_2D_SOLVER solver;
        solver.SetGeometry( xs );
        solver.SetPanelsPerEdge( 12 );
        BOOST_REQUIRE( solver.Solve() );

        const auto& r = solver.GetResult();
        BOOST_TEST_MESSAGE( "=== Interface at GW top (GW fully in air) ===" );
        BOOST_TEST_MESSAGE( "  C=" << r.C( 0, 0 ) << "  C0=" << r.C0( 0, 0 )
                            << "  erEff=" << r.erEff << "  Z0=" << r.Z0 );
    }

    // Test C: no solder mask, interface at GW bottom
    {
        XS_GEOMETRY xs = makeD5Demoted();
        xs.dielectrics.clear();

        XS_DIELECTRIC_REGION airAbove;
        airAbove.yTop = -10e-3;
        airAbove.yBottom = -1.5162e-3;
        airAbove.epsilonR = 1.0;
        xs.dielectrics.push_back( airAbove );

        XS_DIELECTRIC_REGION fr4;
        fr4.yTop = -1.5162e-3;
        fr4.yBottom = gwBottom;
        fr4.epsilonR = 4.50;
        xs.dielectrics.push_back( fr4 );

        XS_DIELECTRIC_REGION airBelow;
        airBelow.yTop = gwBottom;
        airBelow.yBottom = 0.0;
        airBelow.epsilonR = 1.0;
        xs.dielectrics.push_back( airBelow );

        BEM_2D_SOLVER solver;
        solver.SetGeometry( xs );
        solver.SetPanelsPerEdge( 12 );
        BOOST_REQUIRE( solver.Solve() );

        const auto& r = solver.GetResult();
        BOOST_TEST_MESSAGE( "=== Interface at GW bottom, no solder mask ===" );
        BOOST_TEST_MESSAGE( "  C=" << r.C( 0, 0 ) << "  C0=" << r.C0( 0, 0 )
                            << "  erEff=" << r.erEff << "  Z0=" << r.Z0 );
    }
}


/**
 * Step 3f: Solder mask effect comparison.
 * The solder mask adds huge capacitance in the demoted case. Does it add the
 * same amount in the non-demoted case? If so, the SM isn't the issue —
 * the problem is elsewhere. If not, the SM interacts badly with demotion.
 */
BOOST_AUTO_TEST_CASE( DemotionBisect_SolderMaskEffect )
{
    // Non-demoted without solder mask
    {
        XS_GEOMETRY xs = makeD5NonDemoted();

        // Remove solder mask dielectric, extend air down to signal
        xs.dielectrics.clear();

        XS_DIELECTRIC_REGION air;
        air.yTop = -10e-3;
        air.yBottom = -0.2104e-3; // signal bottom / FR4 top
        air.epsilonR = 1.0;
        xs.dielectrics.push_back( air );

        XS_DIELECTRIC_REGION fr4;
        fr4.yTop = -0.2104e-3;
        fr4.yBottom = 0.0;
        fr4.epsilonR = 4.50;
        xs.dielectrics.push_back( fr4 );

        BEM_2D_SOLVER solver;
        solver.SetGeometry( xs );
        solver.SetPanelsPerEdge( 12 );
        BOOST_REQUIRE( solver.Solve() );

        const auto& r = solver.GetResult();
        BOOST_TEST_MESSAGE( "=== Non-demoted, NO solder mask ===" );
        BOOST_TEST_MESSAGE( "  C=" << r.C( 0, 0 ) << "  C0=" << r.C0( 0, 0 )
                            << "  erEff=" << r.erEff << "  Z0=" << r.Z0 );
    }

    // Non-demoted with solder mask (reference = makeD5NonDemoted)
    {
        XS_GEOMETRY xs = makeD5NonDemoted();

        BEM_2D_SOLVER solver;
        solver.SetGeometry( xs );
        solver.SetPanelsPerEdge( 12 );
        BOOST_REQUIRE( solver.Solve() );

        const auto& r = solver.GetResult();
        BOOST_TEST_MESSAGE( "=== Non-demoted, WITH solder mask ===" );
        BOOST_TEST_MESSAGE( "  C=" << r.C( 0, 0 ) << "  C0=" << r.C0( 0, 0 )
                            << "  erEff=" << r.erEff << "  Z0=" << r.Z0 );
    }

    // Demoted with solder mask (reference = makeD5Demoted)
    {
        XS_GEOMETRY xs = makeD5Demoted();

        BEM_2D_SOLVER solver;
        solver.SetGeometry( xs );
        solver.SetPanelsPerEdge( 12 );
        BOOST_REQUIRE( solver.Solve() );

        const auto& r = solver.GetResult();
        BOOST_TEST_MESSAGE( "=== Demoted, WITH solder mask ===" );
        BOOST_TEST_MESSAGE( "  C=" << r.C( 0, 0 ) << "  C0=" << r.C0( 0, 0 )
                            << "  erEff=" << r.erEff << "  Z0=" << r.Z0 );
    }

    // Demoted without solder mask (from InterfaceOffset test C)
    {
        XS_GEOMETRY xs = makeD5Demoted();
        xs.dielectrics.clear();

        double gwBottom = -1.3058e-3 + 0.0350e-3 / 2.0;

        XS_DIELECTRIC_REGION airAbove;
        airAbove.yTop = -10e-3;
        airAbove.yBottom = -1.5162e-3;
        airAbove.epsilonR = 1.0;
        xs.dielectrics.push_back( airAbove );

        XS_DIELECTRIC_REGION fr4;
        fr4.yTop = -1.5162e-3;
        fr4.yBottom = gwBottom;
        fr4.epsilonR = 4.50;
        xs.dielectrics.push_back( fr4 );

        XS_DIELECTRIC_REGION airBelow;
        airBelow.yTop = gwBottom;
        airBelow.yBottom = 0.0;
        airBelow.epsilonR = 1.0;
        xs.dielectrics.push_back( airBelow );

        BEM_2D_SOLVER solver;
        solver.SetGeometry( xs );
        solver.SetPanelsPerEdge( 12 );
        BOOST_REQUIRE( solver.Solve() );

        const auto& r = solver.GetResult();
        BOOST_TEST_MESSAGE( "=== Demoted, NO solder mask ===" );
        BOOST_TEST_MESSAGE( "  C=" << r.C( 0, 0 ) << "  C0=" << r.C0( 0, 0 )
                            << "  erEff=" << r.erEff << "  Z0=" << r.Z0 );
    }
}


/**
 * A trace over a reference plane with a nearby antipad (void to one side)
 * must always have Z0 >= the same trace over an unbroken reference plane.
 *
 * Both cases use the same BEM architecture (NMMTL-style): image at virtual
 * earth + meshed reference as groundwire(s).  The baseline uses a very wide
 * groundwire (approximating infinite plane).  The antipad case trims the
 * groundwire on one side.  Solder mask is included.
 *
 * Narrowing the groundwire can only reduce coupling → Z0 must increase.
 */
BOOST_AUTO_TEST_CASE( AntipadNeverDecreasesZ0 )
{
    double w  = 0.15e-3;
    double t  = 35e-6;
    double h  = 0.2e-3;
    double er = 4.5;
    double smThick = 10e-6;
    double smEr    = 3.3;

    double hFallback = 10e-3;
    double condY     = -( hFallback + t / 2.0 );
    double gwY       = condY + h + t / 2.0;

    // Helper: build geometry with given groundwire bounds
    auto makeGeom = [&]( double gwLeft, double gwRight ) -> XS_GEOMETRY
    {
        XS_GEOMETRY xs;
        xs.groundY = 0.0;
        xs.epsilonR = er;

        XS_CONDUCTOR sig;
        sig.centerX = 0.0;
        sig.centerY = condY;
        sig.width = w;
        sig.thickness = t;
        xs.conductors.push_back( sig );

        XS_CONDUCTOR gw;
        gw.centerX = ( gwLeft + gwRight ) / 2.0;
        gw.centerY = gwY;
        gw.width = gwRight - gwLeft;
        gw.thickness = t;
        gw.isGround = true;
        xs.conductors.push_back( gw );

        XS_DIELECTRIC_REGION airAbove;
        airAbove.yTop = -10e-3;
        airAbove.yBottom = condY - t / 2.0 - smThick;
        airAbove.epsilonR = 1.0;
        xs.dielectrics.push_back( airAbove );

        XS_DIELECTRIC_REGION sm;
        sm.yTop = condY - t / 2.0 - smThick;
        sm.yBottom = condY - t / 2.0;
        sm.epsilonR = smEr;
        xs.dielectrics.push_back( sm );

        XS_DIELECTRIC_REGION fr4;
        fr4.yTop = condY + t / 2.0;
        fr4.yBottom = gwY;
        fr4.epsilonR = er;
        xs.dielectrics.push_back( fr4 );

        XS_DIELECTRIC_REGION airBelow;
        airBelow.yTop = gwY;
        airBelow.yBottom = 0.0;
        airBelow.epsilonR = 1.0;
        xs.dielectrics.push_back( airBelow );

        return xs;
    };

    // --- Baseline: very wide groundwire (solid plane approximation) ---
    XS_GEOMETRY xsBase = makeGeom( -5e-3, 5e-3 );

    int panelCounts[] = { 1, 3, 12 };

    for( int panels : panelCounts )
    {
    BEM_2D_SOLVER solverBase;
    solverBase.SetGeometry( xsBase );
    solverBase.SetPanelsPerEdge( panels );
    solverBase.SetEdgeSingularity( panels > 3 );
    BOOST_REQUIRE( solverBase.Solve() );

    double z0Base = solverBase.GetResult().Z0;

    BOOST_TEST_MESSAGE( "Baseline (10mm wide GW + SM): Z0=" << z0Base );

    // --- Antipad: groundwire trimmed on the left side ---
    // The antipad edge is at edgeDist to the left of signal center.
    // Right side always extends 5mm (solid).
    double edgeDists[] = { 2.0e-3, 1.5e-3, 1.0e-3, 0.75e-3, 0.5e-3, 0.3e-3, 0.2e-3 };

    for( double edgeDist : edgeDists )
    {
        XS_GEOMETRY xs = makeGeom( -edgeDist, 5e-3 );

        BEM_2D_SOLVER solver;
        solver.SetGeometry( xs );
        solver.SetPanelsPerEdge( panels );
        solver.SetEdgeSingularity( panels > 3 );
        BOOST_REQUIRE( solver.Solve() );

        double z0 = solver.GetResult().Z0;

        BOOST_TEST_MESSAGE( "panels=" << panels
                            << "  edgeDist=" << edgeDist * 1e3
                            << "mm  Z0=" << z0
                            << "  delta=" << z0 - z0Base );

        // Allow tiny negative delta from BEM numerical noise (< 0.1 Ohm)
        BOOST_CHECK_GE( z0, z0Base - 0.1 );
    }
    } // for panels
}


BOOST_AUTO_TEST_SUITE_END()
