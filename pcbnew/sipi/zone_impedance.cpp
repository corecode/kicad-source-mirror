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

#include <sipi/zone_impedance.h>

#include <geometry/shape_line_chain.h>
#include <cmath>


// Convert nm^2 to m^2
static constexpr double NM2_TO_M2 = 1e-18;

// Convert nm to m
static constexpr double NM_TO_M = 1e-9;


ZONE_STAR_MODEL ZONE_IMPEDANCE::Build( const SHAPE_LINE_CHAIN& aOutline,
                                       const std::vector<VECTOR2I>& aConnections,
                                       double aRsheet, double aLsheet )
{
    ZONE_STAR_MODEL model;

    // Compute centroid from the polygon outline
    VECTOR2D sum( 0.0, 0.0 );
    int      n = aOutline.PointCount();

    if( n == 0 )
    {
        model.centroid = VECTOR2I( 0, 0 );
        model.area_m2 = 0.0;
        return model;
    }

    // Signed area and centroid using the shoelace formula
    double signedArea2 = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    for( int i = 0; i < n; i++ )
    {
        const VECTOR2I& p0 = aOutline.CPoint( i );
        const VECTOR2I& p1 = aOutline.CPoint( ( i + 1 ) % n );

        double cross = static_cast<double>( p0.x ) * p1.y - static_cast<double>( p1.x ) * p0.y;
        signedArea2 += cross;
        cx += ( p0.x + p1.x ) * cross;
        cy += ( p0.y + p1.y ) * cross;
    }

    double area = std::abs( signedArea2 ) / 2.0;

    if( area > 0.0 )
    {
        cx /= ( 3.0 * signedArea2 );
        cy /= ( 3.0 * signedArea2 );
        model.centroid = VECTOR2I( KiROUND( cx ), KiROUND( cy ) );
    }
    else
    {
        // Degenerate polygon — use average of points
        for( int i = 0; i < n; i++ )
            sum += VECTOR2D( aOutline.CPoint( i ) );

        model.centroid = VECTOR2I( KiROUND( sum.x / n ), KiROUND( sum.y / n ) );
    }

    model.area_m2 = area * NM2_TO_M2;

    // Squares estimate per spoke: treat the fill as a strip of width ≈
    // sqrt(area) and the spoke as a straight line from the connection point
    // to the centroid.  squares = length / width.  This is a crude lumped
    // approximation (the real plane distributes current radially), but it
    // captures the near/far distinction between spokes and costs O(1) per
    // spoke instead of O(samples * outline_points) for ray integration.
    double widthM = ( model.area_m2 > 0.0 ) ? std::sqrt( model.area_m2 ) : 1e-3;

    for( size_t i = 0; i < aConnections.size(); i++ )
    {
        ZONE_SPOKE spoke{};
        spoke.connectionIndex = static_cast<int>( i );

        VECTOR2D delta( aConnections[i] - model.centroid );
        double   lengthM = delta.EuclideanNorm() * NM_TO_M;

        spoke.squares = ( widthM > 0.0 ) ? lengthM / widthM : 0.0;

        if( spoke.squares < 0.01 )
            spoke.squares = 0.01;   // floor so a centroid-coincident connection
                                    // still has some finite sheet impedance

        spoke.resistance = aRsheet * spoke.squares;
        spoke.inductance = aLsheet * spoke.squares;
        model.spokes.push_back( spoke );
    }

    return model;
}


double ZONE_IMPEDANCE::RayIntegrateSquares( const SHAPE_LINE_CHAIN& aOutline,
                                            const VECTOR2I& aFrom, const VECTOR2I& aTo,
                                            int aSamples )
{
    if( aSamples < 2 )
        aSamples = 2;

    VECTOR2D from( aFrom );
    VECTOR2D to( aTo );
    VECTOR2D path = to - from;
    double   pathLen = path.EuclideanNorm();

    if( pathLen < 1.0 )
        return 1.0; // degenerate

    VECTOR2D dir = path / pathLen;

    // Perpendicular direction for width sampling
    VECTOR2D perp( -dir.y, dir.x );

    // We'll cast perpendicular rays at each sample point to find the polygon width.
    // Use trapezoidal integration of ds/w(s).
    double integral = 0.0;
    double prevInvWidth = 0.0;

    for( int i = 0; i <= aSamples; i++ )
    {
        double t = static_cast<double>( i ) / aSamples;
        VECTOR2D samplePt = from + path * t;

        // Cast a long perpendicular ray through the sample point.
        // Find where it intersects the outline on both sides.
        double halfExtent = pathLen * 10.0; // generous extent
        VECTOR2I rayA( KiROUND( samplePt.x - perp.x * halfExtent ),
                       KiROUND( samplePt.y - perp.y * halfExtent ) );
        VECTOR2I rayB( KiROUND( samplePt.x + perp.x * halfExtent ),
                       KiROUND( samplePt.y + perp.y * halfExtent ) );

        SEG raySeg( rayA, rayB );

        SHAPE_LINE_CHAIN::INTERSECTIONS intersections;
        aOutline.Intersect( raySeg, intersections );

        double width = 0.0;

        if( intersections.size() >= 2 )
        {
            // Find the two intersections bracketing the sample point.
            // Project all intersections onto the perpendicular axis and find
            // the nearest ones on each side.
            double sampleProj =
                    ( samplePt.x - rayA.x ) * perp.x + ( samplePt.y - rayA.y ) * perp.y;

            double nearestBelow = -1e30;
            double nearestAbove = 1e30;

            for( const auto& isect : intersections )
            {
                double proj = ( isect.p.x - rayA.x ) * perp.x + ( isect.p.y - rayA.y ) * perp.y;

                if( proj <= sampleProj && proj > nearestBelow )
                    nearestBelow = proj;

                if( proj >= sampleProj && proj < nearestAbove )
                    nearestAbove = proj;
            }

            if( nearestBelow > -1e29 && nearestAbove < 1e29 )
                width = ( nearestAbove - nearestBelow ) * NM_TO_M;
            else
                width = pathLen * NM_TO_M; // fallback
        }
        else
        {
            // No valid intersections — point may be at edge, use path length as width estimate
            width = pathLen * NM_TO_M;
        }

        if( width < 1e-9 )
            width = 1e-9; // prevent division by zero

        double invWidth = 1.0 / width;

        if( i > 0 )
            integral += 0.5 * ( prevInvWidth + invWidth );

        prevInvWidth = invWidth;
    }

    // integral is sum of (1/w) at sample intervals
    // Number of squares = integral * (pathLen / aSamples) in meters
    double ds = pathLen * NM_TO_M / aSamples;
    double squares = integral * ds;

    return ( squares > 0.0 ) ? squares : 1.0;
}
