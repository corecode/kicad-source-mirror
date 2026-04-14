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

#include <sipi/zone_impedance.h>
#include <geometry/shape_line_chain.h>

#include <cmath>


BOOST_AUTO_TEST_SUITE( ZoneImpedance )


/**
 * Helper: build a rectangular polygon outline in nm.
 * Origin at (0,0), extends to (width, height).
 */
static SHAPE_LINE_CHAIN makeRectangle( int width, int height, int originX = 0, int originY = 0 )
{
    SHAPE_LINE_CHAIN outline;
    outline.Append( originX, originY );
    outline.Append( originX + width, originY );
    outline.Append( originX + width, originY + height );
    outline.Append( originX, originY + height );
    outline.SetClosed( true );
    return outline;
}


/**
 * Test ray integration on a uniform-width rectangle.
 *
 * A rectangle 10mm wide x 10mm tall with a connection at the center of the
 * short (bottom) edge going to the center of the opposite (top) edge
 * should give exactly 1.0 squares (length/width = 10/10 = 1.0).
 */
BOOST_AUTO_TEST_CASE( RectangleOneSquare )
{
    // 10mm x 10mm rectangle in nm
    int side = 10000000; // 10mm
    SHAPE_LINE_CHAIN rect = makeRectangle( side, side );

    // Bottom center to top center
    VECTOR2I from( side / 2, 0 );
    VECTOR2I to( side / 2, side );

    double squares = ZONE_IMPEDANCE::RayIntegrateSquares( rect, from, to, 32 );

    // Should be very close to 1.0 (10mm path through 10mm width)
    BOOST_CHECK_CLOSE( squares, 1.0, 5.0 ); // 5% tolerance
}


/**
 * A 20mm wide x 10mm tall rectangle gives 0.5 squares (10/20).
 */
BOOST_AUTO_TEST_CASE( RectangleHalfSquare )
{
    int w = 20000000; // 20mm
    int h = 10000000; // 10mm
    SHAPE_LINE_CHAIN rect = makeRectangle( w, h );

    VECTOR2I from( w / 2, 0 );
    VECTOR2I to( w / 2, h );

    double squares = ZONE_IMPEDANCE::RayIntegrateSquares( rect, from, to, 32 );

    BOOST_CHECK_CLOSE( squares, 0.5, 5.0 );
}


/**
 * A 5mm wide x 10mm tall rectangle gives 2.0 squares (10/5).
 */
BOOST_AUTO_TEST_CASE( RectangleTwoSquares )
{
    int w = 5000000;  // 5mm
    int h = 10000000; // 10mm
    SHAPE_LINE_CHAIN rect = makeRectangle( w, h );

    VECTOR2I from( w / 2, 0 );
    VECTOR2I to( w / 2, h );

    double squares = ZONE_IMPEDANCE::RayIntegrateSquares( rect, from, to, 32 );

    BOOST_CHECK_CLOSE( squares, 2.0, 5.0 );
}


/**
 * Test an L-shaped polygon.
 *
 * An L-shape has a bottleneck — the far connection should see more squares
 * than a direct rectangle would give.
 *
 *   +--------+
 *   |        |
 *   |        +----+
 *   |             |
 *   +---+---------+
 *       |
 *       from
 *
 * The L is 20mm wide x 20mm tall, with a 10mm x 10mm notch cut from bottom-left.
 */
BOOST_AUTO_TEST_CASE( LShapeMoreSquares )
{
    // L-shape: 20mm x 20mm with bottom-left 10mm x 10mm cut out
    //
    // Points (nm): clockwise from bottom-right
    //   (20M, 0) -> (20M, 20M) -> (0, 20M) -> (0, 10M) -> (10M, 10M) -> (10M, 0)
    int m = 1000000; // 1mm in nm

    SHAPE_LINE_CHAIN outline;
    outline.Append( 10 * m, 0 );
    outline.Append( 20 * m, 0 );
    outline.Append( 20 * m, 20 * m );
    outline.Append( 0, 20 * m );
    outline.Append( 0, 10 * m );
    outline.Append( 10 * m, 10 * m );
    outline.SetClosed( true );

    // Connection at bottom center of the lower leg
    VECTOR2I from( 15 * m, 0 );
    // Far connection at top center
    VECTOR2I to( 10 * m, 20 * m );

    double squares = ZONE_IMPEDANCE::RayIntegrateSquares( outline, from, to, 32 );

    // Should be more than a straight 20x20 rectangle (which would give 1.0 square)
    // because the L-shape bottleneck increases the effective number of squares
    BOOST_CHECK_GT( squares, 1.0 );
}


/**
 * Test the full star model build with a simple rectangle zone.
 */
BOOST_AUTO_TEST_CASE( StarModelRectangle )
{
    int w = 10000000; // 10mm
    int h = 10000000; // 10mm
    SHAPE_LINE_CHAIN rect = makeRectangle( w, h );

    // Two connection points: at bottom center and top center
    std::vector<VECTOR2I> connections;
    connections.push_back( VECTOR2I( w / 2, 0 ) );
    connections.push_back( VECTOR2I( w / 2, h ) );

    // Typical 1oz copper on FR4: Rsheet ~ 0.5 mΩ/sq, Lsheet ~ mu0 * h
    double rho_cu = 1.68e-8;  // Ωm
    double t_cu = 35e-6;      // 1oz = 35µm
    double rSheet = rho_cu / t_cu;
    double h_diel = 0.2e-3;   // 200µm dielectric
    double mu0 = 4.0 * M_PI * 1e-7;
    double lSheet = mu0 * h_diel;

    ZONE_STAR_MODEL model = ZONE_IMPEDANCE::Build( rect, connections, rSheet, lSheet );

    // Check centroid is near the center of the rectangle
    BOOST_CHECK_CLOSE( static_cast<double>( model.centroid.x ), w / 2.0, 1.0 );
    BOOST_CHECK_CLOSE( static_cast<double>( model.centroid.y ), h / 2.0, 1.0 );

    // Check area (10mm x 10mm = 100mm² = 1e-4 m²)
    BOOST_CHECK_CLOSE( model.area_m2, 1e-4, 1.0 );

    // Check we got two spokes
    BOOST_CHECK_EQUAL( model.spokes.size(), 2u );

    // Both connections are symmetric (same distance from center), so
    // both spokes should have similar squares and R/L values
    if( model.spokes.size() == 2 )
    {
        BOOST_CHECK_CLOSE( model.spokes[0].squares, model.spokes[1].squares, 10.0 );
        BOOST_CHECK_CLOSE( model.spokes[0].resistance, model.spokes[1].resistance, 10.0 );
        BOOST_CHECK_CLOSE( model.spokes[0].inductance, model.spokes[1].inductance, 10.0 );

        // R = Rsheet * squares, L = Lsheet * squares
        BOOST_CHECK_CLOSE( model.spokes[0].resistance,
                           rSheet * model.spokes[0].squares, 0.1 );
        BOOST_CHECK_CLOSE( model.spokes[0].inductance,
                           lSheet * model.spokes[0].squares, 0.1 );
    }
}


/**
 * Test star model with zero-area polygon returns gracefully.
 */
BOOST_AUTO_TEST_CASE( StarModelDegenerate )
{
    SHAPE_LINE_CHAIN empty;

    std::vector<VECTOR2I> connections;
    connections.emplace_back( 0, 0 );

    ZONE_STAR_MODEL model = ZONE_IMPEDANCE::Build( empty, connections, 0.001, 1e-9 );

    BOOST_CHECK_EQUAL( model.area_m2, 0.0 );
    // Empty outline → early return, no spokes
    BOOST_CHECK_EQUAL( model.spokes.size(), 0u );
}


/**
 * Test that spoke resistance and inductance scale correctly with Rsheet/Lsheet.
 */
BOOST_AUTO_TEST_CASE( StarModelScaling )
{
    int side = 10000000; // 10mm
    SHAPE_LINE_CHAIN rect = makeRectangle( side, side );

    std::vector<VECTOR2I> connections;
    connections.push_back( VECTOR2I( side / 2, 0 ) );

    double rSheet1 = 0.001;
    double lSheet1 = 1e-9;

    ZONE_STAR_MODEL model1 = ZONE_IMPEDANCE::Build( rect, connections, rSheet1, lSheet1 );

    double rSheet2 = 0.002;
    double lSheet2 = 2e-9;

    ZONE_STAR_MODEL model2 = ZONE_IMPEDANCE::Build( rect, connections, rSheet2, lSheet2 );

    // Same geometry → same squares, but R/L should double
    BOOST_CHECK_CLOSE( model1.spokes[0].squares, model2.spokes[0].squares, 0.1 );
    BOOST_CHECK_CLOSE( model2.spokes[0].resistance, 2.0 * model1.spokes[0].resistance, 0.1 );
    BOOST_CHECK_CLOSE( model2.spokes[0].inductance, 2.0 * model1.spokes[0].inductance, 0.1 );
}


BOOST_AUTO_TEST_SUITE_END()
