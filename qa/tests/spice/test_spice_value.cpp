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
#include <sim/spice_value.h>


BOOST_AUTO_TEST_SUITE( SpiceValue )


static void checkValue( const wxString& aInput, double aExpected,
                         SPICE_VALUE::NOTATION aNotation = SPICE_VALUE::NOTATION_SPICE )
{
    SPICE_VALUE val( aInput, aNotation );
    double      result = val.ToDouble();

    BOOST_TEST_CONTEXT( "Input: \"" << aInput << "\" expected: " << aExpected
                                    << " got: " << result )
    {
        if( aExpected == 0.0 )
            BOOST_CHECK_SMALL( result, 1e-9 );
        else
            BOOST_CHECK_CLOSE( result, aExpected, 1e-6 );
    }
}


BOOST_AUTO_TEST_CASE( StandardSpiceNotation )
{
    checkValue( "100", 100.0 );
    checkValue( "4.7k", 4700.0 );
    checkValue( "100n", 100e-9 );
    checkValue( "2.2u", 2.2e-6 );
    checkValue( "10m", 10e-3 );
    checkValue( "4Meg", 4e6 );
    checkValue( "1.5G", 1.5e9 );
    checkValue( "0.1p", 0.1e-12 );
    checkValue( "1e3", 1000.0 );
}


BOOST_AUTO_TEST_CASE( EngineeringNotation )
{
    // Letter-as-decimal-point notation (works in both modes)
    checkValue( "4k7", 4700.0 );
    checkValue( "2u2", 2.2e-6 );
    checkValue( "1n5", 1.5e-9 );
    checkValue( "3m3", 3.3e-3 );
    checkValue( "10k0", 10000.0 );
    checkValue( "1p2", 1.2e-12 );
    checkValue( "47k0", 47000.0 );
}


BOOST_AUTO_TEST_CASE( ResistorRNotation )
{
    // R as decimal point (ohms)
    checkValue( "4R7", 4.7 );
    checkValue( "0R1", 0.1 );
    checkValue( "0R01", 0.01 );
    checkValue( "10R0", 10.0 );
    checkValue( "1R0", 1.0 );

    // R as unit indicator without fractional part
    checkValue( "100R", 100.0 );
    checkValue( "0R", 0.0 );
    checkValue( "47R", 47.0 );
}


BOOST_AUTO_TEST_CASE( SpiceMegaMilliConvention )
{
    // Default (SPICE) mode: m/M both mean milli, Meg = mega
    checkValue( "1m", 1e-3 );
    checkValue( "1M", 1e-3 );
    checkValue( "1Meg", 1e6 );
    checkValue( "10m", 10e-3 );
    checkValue( "10M", 10e-3 );
}


BOOST_AUTO_TEST_CASE( SIMegaMilliConvention )
{
    // In SI mode: m = milli, M = mega (case-sensitive)
    checkValue( "1m", 1e-3, SPICE_VALUE::NOTATION_SI );
    checkValue( "1M", 1e6, SPICE_VALUE::NOTATION_SI );
    checkValue( "1Meg", 1e6, SPICE_VALUE::NOTATION_SI );
    checkValue( "10m", 10e-3, SPICE_VALUE::NOTATION_SI );
    checkValue( "10M", 10e6, SPICE_VALUE::NOTATION_SI );
    checkValue( "2M2", 2.2e6, SPICE_VALUE::NOTATION_SI );
    checkValue( "4m7", 4.7e-3, SPICE_VALUE::NOTATION_SI );
    checkValue( "1K", 1000.0, SPICE_VALUE::NOTATION_SI );
    checkValue( "4K7", 4700.0, SPICE_VALUE::NOTATION_SI );
}


BOOST_AUTO_TEST_CASE( EmptyAndEdgeCases )
{
    checkValue( "", 0.0 );
    checkValue( "0", 0.0 );
    checkValue( "1", 1.0 );
}


BOOST_AUTO_TEST_SUITE_END()
