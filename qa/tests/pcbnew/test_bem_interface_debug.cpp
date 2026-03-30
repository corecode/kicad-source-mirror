/*
 * Minimal BEM matrix debug: build the NMMTL-style system by hand with
 * constant panels, print every key matrix entry, and verify that the
 * interface panels produce a reasonable εr_eff.
 *
 * Uses the NMMTL formulation:
 *   - Green's function G = ln(d_image / d_direct), no ε₀
 *   - Assembly constant = 1/(2π)
 *   - RHS = ε₀ × V
 *   - Interface diagonal: mass matrix × length_scale × (εr+ + εr-)/2
 *   - Interface off-diagonal: length_scale × (εr+ - εr-)/(2π) × flux kernel
 *   - Charge: Q = Σ εr_local × σ × dl
 */

#include <qa_utils/wx_utils/unit_test_utils.h>

#include <Eigen/Dense>
#include <cmath>

static constexpr double EPS0 = 8.854187817e-12;
static constexpr double PI = M_PI;
static constexpr double C_LIGHT = 299792458.0;


// Potential kernel: G = ln(d_image / d_direct), ground at y=0.
static double GP( double x, double y, double xs, double ys )
{
    double dx = x - xs;
    double dy_dir = y - ys;
    double dy_img = y + ys;   // image at (xs, -ys)

    double d2_dir = dx * dx + dy_dir * dy_dir;
    double d2_img = dx * dx + dy_img * dy_img;

    if( d2_dir < 1e-30 ) d2_dir = 1e-30;
    if( d2_img < 1e-30 ) d2_img = 1e-30;

    return 0.5 * log( d2_img / d2_dir );
}


// Flux kernel (NMMTL sign convention): (direct/d² - image/d²) · n̂
static double GF( double x, double y, double xs, double ys, double ny )
{
    double dx = x - xs;
    double dy_dir = y - ys;
    double dy_img = y + ys;

    double d2_dir = dx * dx + dy_dir * dy_dir;
    double d2_img = dx * dx + dy_img * dy_img;

    if( d2_dir < 1e-30 ) d2_dir = 1e-30;
    if( d2_img < 1e-30 ) d2_img = 1e-30;

    // n̂ = (0, ny) for horizontal interface
    return ny * ( dy_dir / d2_dir - dy_img / d2_img );
}


// Analytic self-integral of ln(d_image/d_direct) for a panel of length L
// at position (x, y).  Only the DIRECT term is singular; the image is smooth.
static double selfIntGP( double L, double y )
{
    // Direct: ∫ ln|s| ds from -L/2 to L/2 = L(ln(L/2) - 1)
    double direct = L * ( log( L / 2.0 ) - 1.0 );
    // Image: midpoint approximation of ln(d_image), image distance = 2y
    double d2_img = 4.0 * y * y;
    double image = 0.5 * log( std::max( d2_img, 1e-30 ) ) * L;

    return image - direct;   // G = ln(d_img) - ln(d_dir)
}


BOOST_AUTO_TEST_SUITE( BEMInterfaceDebug )

BOOST_AUTO_TEST_CASE( MinimalSystem )
{
    // Geometry: ground at y=0, substrate εr=4.4 from y=0 to y=h,
    // air above y=h, conductor sitting on interface at y=h.
    double h = 0.1e-3;
    double w = 0.15e-3;
    double t = 35e-6;
    double er = 4.4;

    double hw = w / 2.0;
    double condBot = h;           // conductor bottom at interface
    double condTop = h + t;

    // length_scale = half minimum conductor dimension (NMMTL)
    double lengthScale = std::min( w, t ) / 2.0;

    // --- Conductor panels (constant, for simplicity) ---
    struct P { double x, y, dl; double localEr; char name[16]; };
    std::vector<P> cPanels;

    int nH = 10;
    int nV = 2;

    // Bottom face (at y = h, facing substrate → εr = er)
    for( int i = 0; i < nH; i++ )
    {
        double x = -hw + ( i + 0.5 ) / nH * w;
        cPanels.push_back( { x, condBot, w / nH, er, "" } );
        snprintf( cPanels.back().name, 16, "Cb%d", i );
    }

    // Top face (at y = h+t, facing air → εr = 1)
    for( int i = 0; i < nH; i++ )
    {
        double x = -hw + ( i + 0.5 ) / nH * w;
        cPanels.push_back( { x, condTop, w / nH, 1.0, "" } );
        snprintf( cPanels.back().name, 16, "Ct%d", i );
    }

    // Left side (εr = 1, in air)
    for( int i = 0; i < nV; i++ )
    {
        double y = condBot + ( i + 0.5 ) / nV * t;
        cPanels.push_back( { -hw, y, t / nV, 1.0, "" } );
        snprintf( cPanels.back().name, 16, "Cl%d", i );
    }

    // Right side
    for( int i = 0; i < nV; i++ )
    {
        double y = condBot + ( i + 0.5 ) / nV * t;
        cPanels.push_back( { hw, y, t / nV, 1.0, "" } );
        snprintf( cPanels.back().name, 16, "Cr%d", i );
    }

    int nc = (int) cPanels.size();

    // --- Interface panels at y = h (air above, substrate below) ---
    double spacingI = h / 5.0;
    double extent = 5.0 * h;

    struct IP { double x, dl; char name[16]; };
    std::vector<IP> iPanels;

    for( double x = -( extent + hw ) + spacingI / 2.0; x < -hw; x += spacingI )
    {
        iPanels.push_back( { x, spacingI, "" } );
        snprintf( iPanels.back().name, 16, "I%+.0f", x * 1e6 );
    }

    for( double x = hw + spacingI / 2.0; x < hw + extent; x += spacingI )
    {
        iPanels.push_back( { x, spacingI, "" } );
        snprintf( iPanels.back().name, 16, "I%+.0f", x * 1e6 );
    }

    int ni = (int) iPanels.size();
    int n = nc + ni;

    double epsPlus = 1.0;    // air (above interface, +ŷ direction)
    double epsMinus = er;     // substrate (below interface)

    // --- Assemble NMMTL-style system ---
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero( n, n );
    Eigen::VectorXd b = Eigen::VectorXd::Zero( n );

    // Conductor rows: 1/(2π) × ∫ G × σ dl = ε₀
    for( int i = 0; i < nc; i++ )
    {
        b( i ) = EPS0;  // NMMTL: RHS = ε₀ × V

        for( int j = 0; j < nc; j++ )
        {
            if( i == j )
                A( i, j ) = 1.0 / ( 2.0 * PI ) * selfIntGP( cPanels[i].dl, cPanels[i].y );
            else
                A( i, j ) = 1.0 / ( 2.0 * PI )
                             * GP( cPanels[i].x, cPanels[i].y,
                                   cPanels[j].x, cPanels[j].y )
                             * cPanels[j].dl;
        }

        for( int j = 0; j < ni; j++ )
        {
            A( i, nc + j ) = 1.0 / ( 2.0 * PI )
                             * GP( cPanels[i].x, cPanels[i].y,
                                   iPanels[j].x, h )
                             * iPanels[j].dl;
        }
    }

    // Interface rows
    double coef1 = lengthScale * ( epsPlus + epsMinus ) / 2.0;
    double coef2 = lengthScale * ( epsPlus - epsMinus ) / ( 2.0 * PI );

    for( int ii = 0; ii < ni; ii++ )
    {
        int i = nc + ii;

        // Diagonal: mass matrix (constant panel → dl)
        A( i, i ) = coef1 * iPanels[ii].dl;

        // Off-diagonal: flux kernel to conductor panels
        for( int j = 0; j < nc; j++ )
        {
            A( i, j ) = coef2
                         * GF( iPanels[ii].x, h,
                               cPanels[j].x, cPanels[j].y, 1.0 )
                         * cPanels[j].dl;
        }

        // Off-diagonal: flux kernel to other interface panels
        for( int jj = 0; jj < ni; jj++ )
        {
            if( jj == ii )
                continue;

            A( i, nc + jj ) = coef2
                               * GF( iPanels[ii].x, h,
                                     iPanels[jj].x, h, 1.0 )
                               * iPanels[jj].dl;
        }
    }

    BOOST_TEST_MESSAGE( "\nSystem: nc=" << nc << " ni=" << ni << " total=" << n );
    BOOST_TEST_MESSAGE( "  lengthScale = " << lengthScale * 1e6 << " µm" );
    BOOST_TEST_MESSAGE( "  coef1 = " << coef1 << ", coef2 = " << coef2 );

    BOOST_TEST_MESSAGE( "\nKey matrix entries:" );
    BOOST_TEST_MESSAGE( "  A[Cb0][Cb0]    = " << A( 0, 0 ) );
    BOOST_TEST_MESSAGE( "  A[Cb0][I_first] = " << A( 0, nc ) );
    BOOST_TEST_MESSAGE( "  A[I_first][Cb0] = " << A( nc, 0 ) );
    BOOST_TEST_MESSAGE( "  A[I_first][I_first] = " << A( nc, nc ) );
    BOOST_TEST_MESSAGE( "  b[Cb0] = " << b( 0 ) );

    // Solve
    Eigen::VectorXd sigma = A.fullPivLu().solve( b );

    BOOST_TEST_MESSAGE( "\nSolution (first few):" );

    for( int i = 0; i < std::min( n, 8 ); i++ )
    {
        const char* name = ( i < nc ) ? cPanels[i].name : iPanels[i - nc].name;
        BOOST_TEST_MESSAGE( "  " << name << " σ = " << sigma( i ) );
    }

    BOOST_TEST_MESSAGE( "  ..." );

    for( int i = std::max( nc - 1, 0 ); i < std::min( nc + 3, n ); i++ )
    {
        const char* name = ( i < nc ) ? cPanels[i].name : iPanels[i - nc].name;
        BOOST_TEST_MESSAGE( "  " << name << " σ = " << sigma( i ) );
    }

    // Charge extraction: Q = Σ εr_local × σ × dl (NMMTL convention)
    double Q = 0.0;

    for( int i = 0; i < nc; i++ )
        Q += cPanels[i].localEr * sigma( i ) * cPanels[i].dl;

    // Air solve (no interface panels)
    Eigen::MatrixXd Aa = Eigen::MatrixXd::Zero( nc, nc );
    Eigen::VectorXd ba = Eigen::VectorXd::Zero( nc );

    for( int i = 0; i < nc; i++ )
    {
        ba( i ) = EPS0;

        for( int j = 0; j < nc; j++ )
        {
            if( i == j )
                Aa( i, j ) = 1.0 / ( 2.0 * PI ) * selfIntGP( cPanels[i].dl, cPanels[i].y );
            else
                Aa( i, j ) = 1.0 / ( 2.0 * PI )
                              * GP( cPanels[i].x, cPanels[i].y,
                                    cPanels[j].x, cPanels[j].y )
                              * cPanels[j].dl;
        }
    }

    Eigen::VectorXd sa = Aa.fullPivLu().solve( ba );
    double Qa = 0.0;

    for( int i = 0; i < nc; i++ )
        Qa += 1.0 * sa( i ) * cPanels[i].dl;

    double erEff = Q / Qa;
    double Z0 = 1.0 / ( C_LIGHT * sqrt( std::abs( Q * Qa ) ) );

    BOOST_TEST_MESSAGE( "\nResults:" );
    BOOST_TEST_MESSAGE( "  C      = " << Q << " F/m" );
    BOOST_TEST_MESSAGE( "  C_air  = " << Qa << " F/m" );
    BOOST_TEST_MESSAGE( "  er_eff = " << erEff );
    BOOST_TEST_MESSAGE( "  Z0     = " << Z0 << " Ohm" );

    // εr_eff must be > 1 and < εr
    BOOST_CHECK_GT( erEff, 1.5 );
    BOOST_CHECK_LT( erEff, er );
}

BOOST_AUTO_TEST_SUITE_END()
