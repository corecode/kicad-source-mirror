/*
 * Minimal BEM matrix debug: build the system by hand with a tiny number
 * of panels, print every matrix entry, and verify against the spec formulas.
 */

#include <qa_utils/wx_utils/unit_test_utils.h>

#include <Eigen/Dense>
#include <cmath>

static constexpr double EPS0 = 8.854187817e-12;
static constexpr double PI = M_PI;
static constexpr double C_LIGHT = 299792458.0;

static double GF( double x, double y, double xs, double ys, double h )
{
    double dx = x - xs;
    double dy = y - ys;
    double dyImg = y - ( -2.0 * h - ys );

    double r2 = dx * dx + dy * dy;
    double rImg2 = dx * dx + dyImg * dyImg;

    if( r2 < 1e-30 ) r2 = 1e-30;
    if( rImg2 < 1e-30 ) rImg2 = 1e-30;

    return -1.0 / ( 2.0 * PI * EPS0 ) * ( 0.5 * log( r2 ) - 0.5 * log( rImg2 ) );
}

static double dGdn_y( double x, double y, double xs, double ys, double h )
{
    double dx = x - xs;
    double dy = y - ys;
    double dyImg = y - ( -2.0 * h - ys );

    double r2 = dx * dx + dy * dy;
    double rImg2 = dx * dx + dyImg * dyImg;

    if( r2 < 1e-30 ) r2 = 1e-30;
    if( rImg2 < 1e-30 ) rImg2 = 1e-30;

    return -1.0 / ( 2.0 * PI * EPS0 ) * ( dy / r2 - dyImg / rImg2 );
}

static double selfIntG( double L )
{
    return -L / ( 2.0 * PI * EPS0 ) * ( log( L / 2.0 ) - 1.0 );
}


BOOST_AUTO_TEST_SUITE( BEMInterfaceDebug )

BOOST_AUTO_TEST_CASE( MinimalSystem )
{
    double h = 0.1e-3;
    double w = 0.15e-3;
    double t = 35e-6;
    double er = 4.4;
    double eps1 = EPS0;
    double eps2 = EPS0 * er;

    // 2 conductor panels + many interface panels
    struct P { double x, y, dl; char name[16]; };

    double spacingI = h / 5.0;  // per spec
    double extent = 5.0 * h;
    double hw = w / 2.0;

    std::vector<P> panels;

    // Conductor panels — multiple per face per spec Section 3.1
    int nHoriz = std::max( 4, (int) round( w / ( h / 10.0 ) ) );
    int nVert = std::max( 2, (int) round( t / ( h / 10.0 ) ) );

    // Bottom face
    for( int i = 0; i < nHoriz; i++ )
    {
        double x = -hw + ( i + 0.5 ) / nHoriz * w;
        panels.push_back( { x, 0.0, w / nHoriz, "" } );
        snprintf( panels.back().name, 16, "Cb%d", i );
    }

    // Top face
    for( int i = 0; i < nHoriz; i++ )
    {
        double x = -hw + ( i + 0.5 ) / nHoriz * w;
        panels.push_back( { x, t, w / nHoriz, "" } );
        snprintf( panels.back().name, 16, "Ct%d", i );
    }

    // Left side
    for( int i = 0; i < nVert; i++ )
    {
        double y = ( i + 0.5 ) / nVert * t;
        panels.push_back( { -hw, y, t / nVert, "" } );
        snprintf( panels.back().name, 16, "Cl%d", i );
    }

    // Right side
    for( int i = 0; i < nVert; i++ )
    {
        double y = ( i + 0.5 ) / nVert * t;
        panels.push_back( { hw, y, t / nVert, "" } );
        snprintf( panels.back().name, 16, "Cr%d", i );
    }

    int nc = 2 * nHoriz + 2 * nVert;

    // Interface panels: from -(extent+hw) to -hw, then +hw to +(extent+hw)
    double xStart = -( extent + hw );

    for( double x = xStart + spacingI / 2.0; x < -hw; x += spacingI )
    {
        char name[16];
        snprintf( name, sizeof( name ), "I_%+.0f", x * 1e6 ); // label in µm

        panels.push_back( { x, 0.0, spacingI, "" } );
        snprintf( panels.back().name, 16, "I%+.0f", x * 1e6 );
    }

    for( double x = hw + spacingI / 2.0; x < hw + extent; x += spacingI )
    {
        panels.push_back( { x, 0.0, spacingI, "" } );
        snprintf( panels.back().name, 16, "I%+.0f", x * 1e6 );
    }

    int ni = (int) panels.size() - nc;
    int n = (int) panels.size();
    P* p = panels.data();

    Eigen::MatrixXd A( n, n );
    Eigen::VectorXd b( n );

    // Conductor rows
    for( int i = 0; i < nc; i++ )
    {
        b( i ) = 1.0;

        for( int j = 0; j < n; j++ )
        {
            if( i == j )
            {
                double imgY = -2.0 * h - p[i].y;
                double rImg = std::abs( p[i].y - imgY );
                double gImg = 1.0 / ( 2.0 * PI * EPS0 ) * 0.5 * log( rImg * rImg );

                A( i, j ) = selfIntG( p[i].dl ) + gImg * p[i].dl;
            }
            else
            {
                A( i, j ) = GF( p[i].x, p[i].y, p[j].x, p[j].y, h ) * p[j].dl;
            }
        }
    }

    // Interface rows
    for( int ii = 0; ii < ni; ii++ )
    {
        int i = nc + ii;
        b( i ) = 0.0;

        for( int j = 0; j < n; j++ )
        {
            if( i == j )
            {
                double jump = -( eps1 + eps2 ) / ( 2.0 * EPS0 );
                double imgSelf = ( eps1 - eps2 ) / ( 4.0 * PI * EPS0 * h ) * p[i].dl;

                A( i, j ) = jump + imgSelf;
            }
            else if( j >= nc )
            {
                // iface-to-iface: direct vanishes at y=0, image only
                double dx = p[i].x - p[j].x;
                double denom = dx * dx + 4.0 * h * h;
                double dgdnImg = 1.0 / ( 2.0 * PI * EPS0 ) * 2.0 * h / denom;

                A( i, j ) = ( eps1 - eps2 ) * dgdnImg * p[j].dl;
            }
            else
            {
                // iface-to-conductor
                A( i, j ) = ( eps1 - eps2 )
                            * dGdn_y( p[i].x, p[i].y, p[j].x, p[j].y, h )
                            * p[j].dl;
            }
        }
    }

    BOOST_TEST_MESSAGE( "\nSystem: nc=" << nc << " ni=" << ni << " total=" << n );
    BOOST_TEST_MESSAGE( "  Ground at y = " << -h * 1e3 << " mm, interface at y = 0" );

    // Print first conductor rows (column headers: just first 2 + first/last iface)
    BOOST_TEST_MESSAGE( "\nKey matrix entries:" );
    BOOST_TEST_MESSAGE( "  A[C_bot][C_bot] = " << A( 0, 0 ) );
    BOOST_TEST_MESSAGE( "  A[C_bot][C_top] = " << A( 0, 1 ) );
    BOOST_TEST_MESSAGE( "  A[C_bot][I_first] = " << A( 0, nc ) );
    BOOST_TEST_MESSAGE( "  A[I_first][C_bot] = " << A( nc, 0 ) );
    BOOST_TEST_MESSAGE( "  A[I_first][C_top] = " << A( nc, 1 ) );
    BOOST_TEST_MESSAGE( "  A[I_first][I_first] = " << A( nc, nc ) );
    BOOST_TEST_MESSAGE( "  A[I_first][I_next] = " << ( ni > 1 ? A( nc, nc + 1 ) : 0.0 ) );

    // Solve
    Eigen::VectorXd sigma = A.fullPivLu().solve( b );

    BOOST_TEST_MESSAGE( "\nSolution:" );

    for( int i = 0; i < n; i++ )
    {
        char buf[128];
        snprintf( buf, sizeof( buf ), "  %-7s σ = %+12.6e", p[i].name, sigma( i ) );
        BOOST_TEST_MESSAGE( buf );
    }

    // Charge extraction weighted by local εr (NMMTL convention).
    // Bottom face panels (y ≈ 0) face substrate → εr = er.
    // Top/side panels face air → εr = 1.
    double Q = 0.0;

    for( int i = 0; i < nc; i++ )
    {
        double localEr = ( p[i].y < 1e-9 ) ? er : 1.0; // bottom face at y=0
        Q += localEr * sigma( i ) * p[i].dl;
    }

    // Air solve (no interface)
    Eigen::MatrixXd Aa( nc, nc );
    Eigen::VectorXd ba( nc );

    for( int i = 0; i < nc; i++ )
    {
        ba( i ) = 1.0;

        for( int j = 0; j < nc; j++ )
        {
            if( i == j )
            {
                double imgY = -2.0 * h - p[i].y;
                double rImg = std::abs( p[i].y - imgY );
                double gImg = 1.0 / ( 2.0 * PI * EPS0 ) * 0.5 * log( rImg * rImg );

                Aa( i, j ) = selfIntG( p[i].dl ) + gImg * p[i].dl;
            }
            else
            {
                Aa( i, j ) = GF( p[i].x, p[i].y, p[j].x, p[j].y, h ) * p[j].dl;
            }
        }
    }

    Eigen::VectorXd sa = Aa.fullPivLu().solve( ba );
    double Qa = 0.0;

    for( int i = 0; i < nc; i++ )
        Qa += sa( i ) * p[i].dl;

    BOOST_TEST_MESSAGE( "\nResults:" );
    BOOST_TEST_MESSAGE( "  C      = " << Q << " F/m" );
    BOOST_TEST_MESSAGE( "  C_air  = " << Qa << " F/m" );
    BOOST_TEST_MESSAGE( "  er_eff = " << Q / Qa );
    BOOST_TEST_MESSAGE( "  Z0     = " << 1.0 / ( C_LIGHT * sqrt( std::abs( Q * Qa ) ) ) << " Ohm" );

    // εr_eff must be > 1
    BOOST_CHECK_GT( Q, Qa );
}

BOOST_AUTO_TEST_SUITE_END()
