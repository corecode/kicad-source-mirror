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

#include <sipi/analytical_impedance.h>
#include <sipi/stackup_reader.h>


BOOST_AUTO_TEST_SUITE( AnalyticalImpedance )


/**
 * Standard 50-ohm microstrip: w ≈ 0.17mm on 0.1mm FR4 (er=4.4), 35µm copper.
 * Reference: Saturn PCB Toolkit / online calculators give ~50 Ω ± 5%.
 */
BOOST_AUTO_TEST_CASE( Microstrip50Ohm )
{
    double w  = 0.17e-3;   // 0.17 mm
    double h  = 0.1e-3;    // 0.1 mm dielectric
    double er = 4.4;
    double t  = 35e-6;     // 35 µm (1 oz copper)

    double z0 = ANALYTICAL_IMPEDANCE::MicrostripZ0( w, h, er, t );

    BOOST_TEST_MESSAGE( "Microstrip Z0 = " << z0 << " Ohm" );

    // Should be in the 45-55 Ohm range for this geometry
    BOOST_CHECK_GT( z0, 40.0 );
    BOOST_CHECK_LT( z0, 60.0 );
}


/**
 * Wider microstrip should have lower impedance.
 */
BOOST_AUTO_TEST_CASE( MicrostripWiderIsLower )
{
    double h  = 0.1e-3;
    double er = 4.4;

    double z0_narrow = ANALYTICAL_IMPEDANCE::MicrostripZ0( 0.1e-3, h, er );
    double z0_wide   = ANALYTICAL_IMPEDANCE::MicrostripZ0( 0.3e-3, h, er );

    BOOST_TEST_MESSAGE( "Narrow Z0 = " << z0_narrow << ", Wide Z0 = " << z0_wide );
    BOOST_CHECK_GT( z0_narrow, z0_wide );
}


/**
 * Higher dielectric constant should lower impedance.
 */
BOOST_AUTO_TEST_CASE( MicrostripHigherErIsLower )
{
    double w = 0.15e-3;
    double h = 0.1e-3;

    double z0_low_er  = ANALYTICAL_IMPEDANCE::MicrostripZ0( w, h, 3.0 );
    double z0_high_er = ANALYTICAL_IMPEDANCE::MicrostripZ0( w, h, 6.0 );

    BOOST_TEST_MESSAGE( "er=3.0: Z0 = " << z0_low_er << ", er=6.0: Z0 = " << z0_high_er );
    BOOST_CHECK_GT( z0_low_er, z0_high_er );
}


/**
 * Stripline should give lower impedance than microstrip for same w/h,
 * because the trace is fully enclosed in dielectric.
 */
BOOST_AUTO_TEST_CASE( StriplineLowerThanMicrostrip )
{
    double w  = 0.15e-3;
    double h  = 0.1e-3;
    double er = 4.4;
    double t  = 35e-6;

    double z0_ms = ANALYTICAL_IMPEDANCE::MicrostripZ0( w, h, er, t );
    double z0_sl = ANALYTICAL_IMPEDANCE::StriplineZ0( w, h, er, t );

    BOOST_TEST_MESSAGE( "Microstrip = " << z0_ms << " Ohm, Stripline = " << z0_sl << " Ohm" );
    BOOST_CHECK_GT( z0_ms, z0_sl );
}


/**
 * Symmetric stripline: w = 0.1mm, h = 0.1mm (each side), er = 4.4, t = 18µm.
 * Reference: ~50 Ω per standard impedance calculators.
 */
BOOST_AUTO_TEST_CASE( Stripline50Ohm )
{
    double w  = 0.1e-3;   // 0.1 mm
    double h  = 0.1e-3;   // 0.1 mm each side
    double er = 4.4;
    double t  = 18e-6;    // half-ounce copper

    double z0 = ANALYTICAL_IMPEDANCE::StriplineZ0( w, h, er, t );

    BOOST_TEST_MESSAGE( "Stripline Z0 = " << z0 << " Ohm" );

    // Should be in the 40-60 Ohm range
    BOOST_CHECK_GT( z0, 35.0 );
    BOOST_CHECK_LT( z0, 65.0 );
}


/**
 * ComputeZ0 selects microstrip when only one reference plane exists.
 */
BOOST_AUTO_TEST_CASE( ComputeZ0_Microstrip )
{
    LAYER_GEOMETRY geom;
    geom.traceWidth = 0.15e-3;
    geom.traceThickness = 35e-6;
    geom.hBelow = 0.1e-3;
    geom.erBelow = 4.4;
    geom.hasRefBelow = true;

    double z0 = STACKUP_READER::ComputeZ0( geom );

    BOOST_TEST_MESSAGE( "ComputeZ0 (microstrip) = " << z0 << " Ohm" );
    BOOST_CHECK_GT( z0, 30.0 );
    BOOST_CHECK_LT( z0, 80.0 );

    // Should match direct microstrip call
    double z0_direct = ANALYTICAL_IMPEDANCE::MicrostripZ0( geom.traceWidth, geom.hBelow,
                                                           geom.erBelow, geom.traceThickness );
    BOOST_CHECK_CLOSE( z0, z0_direct, 0.01 );
}


/**
 * ComputeZ0 selects stripline when two reference planes exist.
 */
BOOST_AUTO_TEST_CASE( ComputeZ0_Stripline )
{
    LAYER_GEOMETRY geom;
    geom.traceWidth = 0.1e-3;
    geom.traceThickness = 18e-6;
    geom.hAbove = 0.1e-3;
    geom.hBelow = 0.1e-3;
    geom.erAbove = 4.4;
    geom.erBelow = 4.4;
    geom.hasRefAbove = true;
    geom.hasRefBelow = true;

    double z0 = STACKUP_READER::ComputeZ0( geom );

    BOOST_TEST_MESSAGE( "ComputeZ0 (stripline) = " << z0 << " Ohm" );
    BOOST_CHECK_GT( z0, 30.0 );
    BOOST_CHECK_LT( z0, 70.0 );
}


/**
 * Propagation delay should be ~5-7 ns/m for FR4 microstrip (er_eff ≈ 3.2).
 */
BOOST_AUTO_TEST_CASE( PropagationDelay )
{
    double er_eff = ANALYTICAL_IMPEDANCE::MicrostripEffectiveEr( 0.15e-3, 0.1e-3, 4.4 );
    double tpd = ANALYTICAL_IMPEDANCE::PropagationDelay( er_eff );

    BOOST_TEST_MESSAGE( "er_eff = " << er_eff << ", tpd = " << ( tpd * 1e9 ) << " ns/m" );

    // er_eff should be between 1 and er_bulk
    BOOST_CHECK_GT( er_eff, 1.0 );
    BOOST_CHECK_LT( er_eff, 4.4 );

    // tpd should be ~5-7 ns/m
    BOOST_CHECK_GT( tpd * 1e9, 4.0 );
    BOOST_CHECK_LT( tpd * 1e9, 8.0 );
}


BOOST_AUTO_TEST_SUITE_END()
