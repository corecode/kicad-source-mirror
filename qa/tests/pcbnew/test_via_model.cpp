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

#include <sipi/via_model.h>

#include <cmath>


BOOST_AUTO_TEST_SUITE( ViaModel )


// ---------------------------------------------------------------------------
// Helper: build a standard via for testing
// ---------------------------------------------------------------------------

static VIA_PARAMS makeVia( int aXnm, int aYnm, VIA_ROLE aRole,
                           double aDrill = 0.3e-3, double aPlating = 25e-6,
                           double aPad = 0.5e-3, double aH = 1.6e-3,
                           double aStub = 0.0 )
{
    VIA_PARAMS v;
    v.position = VECTOR2I( aXnm, aYnm );
    v.drillRadius = aDrill / 2.0;
    v.barrelOuterRadius = aDrill / 2.0 + aPlating;
    v.padRadius = aPad / 2.0;
    v.barrelHeight = aH;
    v.stubLength = aStub;
    v.epsilonReff = 4.4;
    v.role = aRole;

    // One plane crossing in the middle (typical inner-layer via)
    VIA_PLANE_CROSSING pc;
    pc.epsilonR = 4.4;
    pc.dielectricThickness = 0.1e-3;
    pc.antipadRadius = aDrill * 0.75; // 1.5× drill diameter → radius = 0.75× drill
    v.planeCrossings.push_back( pc );

    // Entry and exit pad planes (with antipad in the reference plane)
    VIA_PAD_PLANE pp;
    pp.epsilonR = 4.4;
    pp.thickness = 0.1e-3;
    pp.antipadRadius = aDrill * 0.75; // Same antipad as barrel crossings
    v.adjacentPlanes.push_back( pp );
    v.adjacentPlanes.push_back( pp );

    return v;
}


// ---------------------------------------------------------------------------
// Part 1: Self-inductance sanity checks
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( SelfInductance_Standard )
{
    // 0.3mm drill, 1.6mm FR4 → L ≈ 0.7-1.0 nH
    double L = VIA_MODEL_BUILDER::SelfInductance( 1.6e-3, 0.15e-3 );

    BOOST_TEST_MESSAGE( "L_self (0.3mm drill, 1.6mm) = " << L * 1e9 << " nH" );

    BOOST_CHECK_GT( L, 0.5e-9 );
    BOOST_CHECK_LT( L, 1.5e-9 );
}


BOOST_AUTO_TEST_CASE( SelfInductance_ShortVia )
{
    // 0.3mm drill, 0.2mm depth (microvia) → much smaller L
    double Lshort = VIA_MODEL_BUILDER::SelfInductance( 0.2e-3, 0.15e-3 );
    double Llong = VIA_MODEL_BUILDER::SelfInductance( 1.6e-3, 0.15e-3 );

    BOOST_TEST_MESSAGE( "L_self (microvia 0.2mm) = " << Lshort * 1e12 << " pH" );

    BOOST_CHECK_LT( Lshort, Llong );
    BOOST_CHECK_GT( Lshort, 0.0 );
}


BOOST_AUTO_TEST_CASE( SelfInductance_Zero )
{
    BOOST_CHECK_EQUAL( VIA_MODEL_BUILDER::SelfInductance( 0.0, 0.15e-3 ), 0.0 );
    BOOST_CHECK_EQUAL( VIA_MODEL_BUILDER::SelfInductance( 1.6e-3, 0.0 ), 0.0 );
}


// ---------------------------------------------------------------------------
// Part 2: Mutual inductance
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( MutualInductance_LessThanSelf )
{
    double h = 1.6e-3;
    double r = 0.15e-3;
    double d = 1.0e-3; // 1mm apart

    double Lself = VIA_MODEL_BUILDER::SelfInductance( h, r );
    double M = VIA_MODEL_BUILDER::MutualInductance( h, d );

    BOOST_TEST_MESSAGE( "M (1mm apart) = " << M * 1e9 << " nH, L_self = "
                        << Lself * 1e9 << " nH" );

    BOOST_CHECK_GT( M, 0.0 );
    BOOST_CHECK_LT( M, Lself );
}


BOOST_AUTO_TEST_CASE( MutualInductance_DecreasesWithDistance )
{
    double h = 1.6e-3;

    double M_close = VIA_MODEL_BUILDER::MutualInductance( h, 0.5e-3 );
    double M_far = VIA_MODEL_BUILDER::MutualInductance( h, 3.0e-3 );

    BOOST_TEST_MESSAGE( "M (0.5mm) = " << M_close * 1e12 << " pH" );
    BOOST_TEST_MESSAGE( "M (3.0mm) = " << M_far * 1e12 << " pH" );

    BOOST_CHECK_GT( M_close, M_far );
}


// ---------------------------------------------------------------------------
// Part 4: Schur complement — single signal via with return vias
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( SchurComplement_SingleSignalNoReturn )
{
    // Single signal via, no returns → L_eff = L_self
    VIA_MODEL_BUILDER builder;
    std::vector<VIA_PARAMS> cluster;
    cluster.push_back( makeVia( 0, 0, VIA_ROLE::SIGNAL_P ) );

    builder.SetCluster( cluster );
    BOOST_REQUIRE( builder.Compute() );

    const auto& result = builder.GetResult();
    double Lself = VIA_MODEL_BUILDER::SelfInductance( 1.6e-3, 0.15e-3 );

    BOOST_TEST_MESSAGE( "L_eff (no returns) = " << result.Leff( 0, 0 ) * 1e9 << " nH" );
    BOOST_CHECK_CLOSE( result.Leff( 0, 0 ), Lself, 0.1 );
}


BOOST_AUTO_TEST_CASE( SchurComplement_OneReturn )
{
    // Signal + 1 return 1mm away → L_eff reduced
    VIA_MODEL_BUILDER builder;
    std::vector<VIA_PARAMS> cluster;
    cluster.push_back( makeVia( 0, 0, VIA_ROLE::SIGNAL_P ) );
    cluster.push_back( makeVia( 1000000, 0, VIA_ROLE::RETURN ) ); // 1mm = 1e6 nm

    builder.SetCluster( cluster );
    BOOST_REQUIRE( builder.Compute() );

    const auto& result = builder.GetResult();
    double Lself = VIA_MODEL_BUILDER::SelfInductance( 1.6e-3, 0.15e-3 );

    BOOST_TEST_MESSAGE( "L_eff (1 return @ 1mm) = " << result.Leff( 0, 0 ) * 1e12 << " pH" );
    BOOST_TEST_MESSAGE( "L_self = " << Lself * 1e9 << " nH" );
    BOOST_TEST_MESSAGE( "Reduction = " << ( 1.0 - result.Leff( 0, 0 ) / Lself ) * 100 << "%" );

    BOOST_CHECK_LT( result.Leff( 0, 0 ), Lself );
    BOOST_CHECK_GT( result.Leff( 0, 0 ), 0.0 );
}


BOOST_AUTO_TEST_CASE( SchurComplement_FourReturns )
{
    // Signal + 4 returns at ±1mm → significant L reduction
    VIA_MODEL_BUILDER builder;
    std::vector<VIA_PARAMS> cluster;
    cluster.push_back( makeVia( 0, 0, VIA_ROLE::SIGNAL_P ) );
    cluster.push_back( makeVia( 1000000, 0, VIA_ROLE::RETURN ) );
    cluster.push_back( makeVia( -1000000, 0, VIA_ROLE::RETURN ) );
    cluster.push_back( makeVia( 0, 1000000, VIA_ROLE::RETURN ) );
    cluster.push_back( makeVia( 0, -1000000, VIA_ROLE::RETURN ) );

    builder.SetCluster( cluster );
    BOOST_REQUIRE( builder.Compute() );

    const auto& result = builder.GetResult();
    double Lself = VIA_MODEL_BUILDER::SelfInductance( 1.6e-3, 0.15e-3 );

    double reduction = 1.0 - result.Leff( 0, 0 ) / Lself;

    BOOST_TEST_MESSAGE( "L_eff (4 returns @ 1mm) = " << result.Leff( 0, 0 ) * 1e12 << " pH" );
    BOOST_TEST_MESSAGE( "Reduction = " << reduction * 100 << "%" );

    // Expect 30–90% reduction with 4 ground vias at 1mm
    BOOST_CHECK_GT( reduction, 0.25 );
    BOOST_CHECK_LT( reduction, 0.95 );
}


// ---------------------------------------------------------------------------
// Part 4: Differential pair with ground returns
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( DiffPair_NoReturns )
{
    // Two signal vias 1.5mm apart, no returns
    VIA_MODEL_BUILDER builder;
    std::vector<VIA_PARAMS> cluster;
    cluster.push_back( makeVia( 0, 0, VIA_ROLE::SIGNAL_P ) );
    cluster.push_back( makeVia( 1500000, 0, VIA_ROLE::SIGNAL_N ) ); // 1.5mm

    builder.SetCluster( cluster );
    BOOST_REQUIRE( builder.Compute() );

    const auto& result = builder.GetResult();

    BOOST_TEST_MESSAGE( "L_diff (no returns) = " << result.Ldiff * 1e12 << " pH" );
    BOOST_TEST_MESSAGE( "L_cm   (no returns) = " << result.Lcm * 1e12 << " pH" );

    // L_diff = L_self - M, L_cm = L_self + M
    BOOST_CHECK_GT( result.Ldiff, 0.0 );
    BOOST_CHECK_GT( result.Lcm, result.Ldiff ); // common mode > differential
}


BOOST_AUTO_TEST_CASE( DiffPair_WithReturns )
{
    // Diff pair + 4 symmetric ground vias
    VIA_MODEL_BUILDER builder;
    std::vector<VIA_PARAMS> cluster;

    // Signal pair at ±0.75mm from center
    cluster.push_back( makeVia( -750000, 0, VIA_ROLE::SIGNAL_P ) );
    cluster.push_back( makeVia( 750000, 0, VIA_ROLE::SIGNAL_N ) );

    // 4 ground vias at corners (symmetric about both axes)
    cluster.push_back( makeVia( -1500000, 1000000, VIA_ROLE::RETURN ) );
    cluster.push_back( makeVia( 1500000, 1000000, VIA_ROLE::RETURN ) );
    cluster.push_back( makeVia( -1500000, -1000000, VIA_ROLE::RETURN ) );
    cluster.push_back( makeVia( 1500000, -1000000, VIA_ROLE::RETURN ) );

    builder.SetCluster( cluster );
    BOOST_REQUIRE( builder.Compute() );

    const auto& result = builder.GetResult();

    BOOST_TEST_MESSAGE( "L_diff (with returns) = " << result.Ldiff * 1e12 << " pH" );
    BOOST_TEST_MESSAGE( "L_cm   (with returns) = " << result.Lcm * 1e12 << " pH" );
    BOOST_TEST_MESSAGE( "L_dc   (symmetric) = " << result.Ldc * 1e15 << " fH" );
    BOOST_TEST_MESSAGE( "Symmetric: " << result.symmetric );

    // With returns, L_diff should be significantly reduced
    BOOST_CHECK_GT( result.Ldiff, 0.0 );

    // Symmetric cluster → L_dc ≈ 0
    BOOST_CHECK( result.symmetric );
}


// ---------------------------------------------------------------------------
// Part 5: Pad capacitance
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( PadCapacitance_Standard )
{
    // 0.5mm pad, 0.3mm drill, antipad=0.225mm (1.5× drill), εr=4.4, 0.1mm dielectric
    double C = VIA_MODEL_BUILDER::PadCapacitance( 0.25e-3, 0.15e-3, 0.225e-3, 4.4, 0.1e-3 );

    BOOST_TEST_MESSAGE( "C_pad (with antipad) = " << C * 1e15 << " fF" );

    // Reduced vs no-antipad case because overlap is only r_antipad to r_pad
    BOOST_CHECK_GT( C, 1e-15 );
    BOOST_CHECK_LT( C, 500e-15 );
}


BOOST_AUTO_TEST_CASE( PadCapacitance_AntipadEffect )
{
    double rPad = 0.25e-3;
    double rDrill = 0.15e-3;
    double er = 4.4;
    double t = 0.1e-3;

    // No antipad (or antipad smaller than drill) → full overlap
    double C_noAntipad = VIA_MODEL_BUILDER::PadCapacitance( rPad, rDrill, 0.0, er, t );

    // Antipad = 0.225mm → partial overlap (0.225 to 0.25mm ring)
    double C_smallAntipad = VIA_MODEL_BUILDER::PadCapacitance( rPad, rDrill, 0.225e-3, er, t );

    // Antipad larger than pad → NO parallel-plate overlap, only fringing
    double C_largeAntipad = VIA_MODEL_BUILDER::PadCapacitance( rPad, rDrill, 0.30e-3, er, t );

    BOOST_TEST_MESSAGE( "C_pad (no antipad)     = " << C_noAntipad * 1e15 << " fF" );
    BOOST_TEST_MESSAGE( "C_pad (small antipad)  = " << C_smallAntipad * 1e15 << " fF" );
    BOOST_TEST_MESSAGE( "C_pad (large antipad)  = " << C_largeAntipad * 1e15 << " fF" );

    // Larger antipad → less capacitance
    BOOST_CHECK_GT( C_noAntipad, C_smallAntipad );
    BOOST_CHECK_GT( C_smallAntipad, C_largeAntipad );

    // Large antipad still has fringing
    BOOST_CHECK_GT( C_largeAntipad, 0.0 );
}


// ---------------------------------------------------------------------------
// Part 6: Barrel capacitance
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( BarrelCapacitance_Standard )
{
    // εr=4.4, 0.1mm dielectric, antipad=0.225mm (1.5× drill), drill=0.15mm
    double C = VIA_MODEL_BUILDER::BarrelCapacitance( 4.4, 0.1e-3, 0.225e-3, 0.15e-3 );

    BOOST_TEST_MESSAGE( "C_barrel (per plane) = " << C * 1e15 << " fF" );

    BOOST_CHECK_GT( C, 10e-15 );
    BOOST_CHECK_LT( C, 500e-15 );
}


// ---------------------------------------------------------------------------
// Part 7: Barrel-to-barrel capacitance
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( BarrelToBarrel_DiffPair )
{
    // 0.3mm drill, 0.6mm pitch, 1.6mm FR4 → C_bb ≈ 80-200 fF
    double Cbb = VIA_MODEL_BUILDER::BarrelToBarrelCapacitance( 4.4, 1.6e-3, 0.6e-3,
                                                               0.175e-3 );

    BOOST_TEST_MESSAGE( "C_bb (0.6mm pitch) = " << Cbb * 1e15 << " fF" );

    BOOST_CHECK_GT( Cbb, 50e-15 );
    BOOST_CHECK_LT( Cbb, 500e-15 );
}


BOOST_AUTO_TEST_CASE( BarrelToBarrel_DecreasesWithDistance )
{
    double Cclose = VIA_MODEL_BUILDER::BarrelToBarrelCapacitance( 4.4, 1.6e-3, 0.6e-3,
                                                                  0.175e-3 );
    double Cfar = VIA_MODEL_BUILDER::BarrelToBarrelCapacitance( 4.4, 1.6e-3, 3.0e-3,
                                                                0.175e-3 );

    BOOST_CHECK_GT( Cclose, Cfar );
}


// ---------------------------------------------------------------------------
// Part 9: Resistance
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( Resistance_DC )
{
    // 1.6mm barrel, 0.15mm drill, 0.175mm outer → typical sub-milliohm
    double Rdc = VIA_MODEL_BUILDER::BarrelResistance( 1.6e-3, 0.15e-3, 0.175e-3,
                                                      5.8e7, 0.0 );

    BOOST_TEST_MESSAGE( "R_dc = " << Rdc * 1000 << " mOhm" );

    BOOST_CHECK_GT( Rdc, 0.0 );
    BOOST_CHECK_LT( Rdc, 0.01 ); // < 10 mΩ
}


BOOST_AUTO_TEST_CASE( Resistance_IncreasesWithFrequency )
{
    double Rdc = VIA_MODEL_BUILDER::BarrelResistance( 1.6e-3, 0.15e-3, 0.175e-3,
                                                      5.8e7, 0.0 );
    double R1G = VIA_MODEL_BUILDER::BarrelResistance( 1.6e-3, 0.15e-3, 0.175e-3,
                                                      5.8e7, 1e9 );

    BOOST_TEST_MESSAGE( "R_dc = " << Rdc * 1000 << " mOhm" );
    BOOST_TEST_MESSAGE( "R(1GHz) = " << R1G * 1000 << " mOhm" );

    BOOST_CHECK_GT( R1G, Rdc );
}


// ---------------------------------------------------------------------------
// Part 10: Stub resonance
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( StubResonance_Standard )
{
    // 5mm stub in FR4 (εr=4.4) → f_res = c/(4×5mm×√4.4) ≈ 7.1 GHz
    double fres = VIA_MODEL_BUILDER::StubResonanceFreq( 5e-3, 4.4 );

    BOOST_TEST_MESSAGE( "f_res (5mm stub) = " << fres / 1e9 << " GHz" );

    BOOST_CHECK_GT( fres, 5e9 );
    BOOST_CHECK_LT( fres, 10e9 );
}


BOOST_AUTO_TEST_CASE( StubResonance_NoStub )
{
    BOOST_CHECK_EQUAL( VIA_MODEL_BUILDER::StubResonanceFreq( 0.0, 4.4 ), 0.0 );
}


// ---------------------------------------------------------------------------
// Part 11: Validity limit
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( ValidityLimit_Standard )
{
    // 1.6mm FR4 → f_max = c/(10×1.6mm×√4.4) ≈ 8.9 GHz
    double fmax = VIA_MODEL_BUILDER::ValidityLimit( 1.6e-3, 4.4 );

    BOOST_TEST_MESSAGE( "f_max (1.6mm FR4) = " << fmax / 1e9 << " GHz" );

    BOOST_CHECK_GT( fmax, 7e9 );
    BOOST_CHECK_LT( fmax, 12e9 );
}


BOOST_AUTO_TEST_CASE( ValidityLimit_ThinBoard )
{
    // 0.5mm HDI → f_max ≈ 28 GHz
    double fmax = VIA_MODEL_BUILDER::ValidityLimit( 0.5e-3, 4.4 );

    BOOST_TEST_MESSAGE( "f_max (0.5mm HDI) = " << fmax / 1e9 << " GHz" );

    BOOST_CHECK_GT( fmax, 20e9 );
    BOOST_CHECK_LT( fmax, 40e9 );
}


// ---------------------------------------------------------------------------
// Part 12: Symmetry check
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( Symmetry_Symmetric )
{
    // Perfectly symmetric diff pair + 2 symmetric ground vias
    VIA_MODEL_BUILDER builder;
    std::vector<VIA_PARAMS> cluster;
    cluster.push_back( makeVia( -500000, 0, VIA_ROLE::SIGNAL_P ) );
    cluster.push_back( makeVia( 500000, 0, VIA_ROLE::SIGNAL_N ) );
    cluster.push_back( makeVia( 0, 1000000, VIA_ROLE::RETURN ) );
    cluster.push_back( makeVia( 0, -1000000, VIA_ROLE::RETURN ) );

    builder.SetCluster( cluster );
    BOOST_REQUIRE( builder.Compute() );

    BOOST_CHECK( builder.GetResult().symmetric );
    BOOST_CHECK_SMALL( builder.GetResult().Ldc, 1e-18 ); // < 1 aH
}


BOOST_AUTO_TEST_CASE( Symmetry_Asymmetric )
{
    // Asymmetric: one ground via much closer to P than N
    VIA_MODEL_BUILDER builder;
    std::vector<VIA_PARAMS> cluster;
    cluster.push_back( makeVia( -500000, 0, VIA_ROLE::SIGNAL_P ) );
    cluster.push_back( makeVia( 500000, 0, VIA_ROLE::SIGNAL_N ) );
    cluster.push_back( makeVia( -600000, 500000, VIA_ROLE::RETURN ) ); // close to P

    builder.SetCluster( cluster );
    BOOST_REQUIRE( builder.Compute() );

    BOOST_TEST_MESSAGE( "L_dc (asymmetric) = " << builder.GetResult().Ldc * 1e12 << " pH" );

    // L_dc should be non-zero for asymmetric layout
    BOOST_CHECK_GT( std::abs( builder.GetResult().Ldc ), 1e-15 );
    BOOST_CHECK( !builder.GetResult().symmetric );
}


// ---------------------------------------------------------------------------
// Full cluster extraction — integration test
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE( FullCluster_SingleVia )
{
    VIA_MODEL_BUILDER builder;
    std::vector<VIA_PARAMS> cluster;
    cluster.push_back( makeVia( 0, 0, VIA_ROLE::SIGNAL_P ) );

    builder.SetCluster( cluster );
    BOOST_REQUIRE( builder.Compute() );

    const auto& r = builder.GetResult();

    BOOST_CHECK_EQUAL( r.numVias, 1 );
    BOOST_CHECK_GT( r.Ldiff, 0.0 );
    BOOST_CHECK_GT( r.Ctotal[0], 0.0 );
    BOOST_CHECK_GT( r.Rdc[0], 0.0 );
    BOOST_CHECK_GT( r.fMax, 0.0 );

    BOOST_TEST_MESSAGE( "Single via: L=" << r.Ldiff * 1e9 << " nH"
                        << " C=" << r.Ctotal[0] * 1e15 << " fF"
                        << " Rdc=" << r.Rdc[0] * 1000 << " mΩ"
                        << " fmax=" << r.fMax / 1e9 << " GHz" );
}


BOOST_AUTO_TEST_SUITE_END()
