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

#include "via_model.h"

#include <cmath>


// Physical constants
static constexpr double MU0 = 4.0 * M_PI * 1e-7;          // H/m
static constexpr double EPS0 = 8.854187817e-12;            // F/m
static constexpr double C_LIGHT = 299792458.0;             // m/s
static constexpr double SIGMA_COPPER = 5.8e7;              // S/m


VIA_MODEL_BUILDER::VIA_MODEL_BUILDER() = default;


void VIA_MODEL_BUILDER::SetCluster( const std::vector<VIA_PARAMS>& aVias )
{
    m_vias = aVias;
    m_signalIdx.clear();
    m_returnIdx.clear();

    for( int i = 0; i < (int) m_vias.size(); i++ )
    {
        if( m_vias[i].role == VIA_ROLE::RETURN )
            m_returnIdx.push_back( i );
        else
            m_signalIdx.push_back( i );
    }
}


bool VIA_MODEL_BUILDER::Compute()
{
    int n = (int) m_vias.size();

    if( n == 0 )
        return false;

    m_result = VIA_CLUSTER_RESULT();
    m_result.numVias = n;

    buildInductanceMatrix();
    schurReduce();
    computeCapacitances();
    computeBarrelToBarrel();
    computeResistance();
    computeStubResonance();
    m_result.symmetric = checkSymmetry();

    // Validity limit: use the longest barrel in the cluster
    double maxH = 0.0;
    double maxErEff = 1.0;

    for( const VIA_PARAMS& v : m_vias )
    {
        if( v.barrelHeight > maxH )
        {
            maxH = v.barrelHeight;
            maxErEff = v.epsilonReff;
        }
    }

    m_result.fMax = ValidityLimit( maxH, maxErEff );

    return true;
}


// ---------------------------------------------------------------------------
// Part 1 — Self-inductance (Goldfarb-Pucel)
// ---------------------------------------------------------------------------

double VIA_MODEL_BUILDER::SelfInductance( double aH, double aR )
{
    if( aH <= 0.0 || aR <= 0.0 )
        return 0.0;

    double r2 = aR * aR;
    double h2 = aH * aH;
    double sqrtSum = std::sqrt( h2 + r2 );

    // L = (µ₀/2π) × [h·ln((h+√(h²+r²))/r) + 1.5(r-√(h²+r²))]
    return ( MU0 / ( 2.0 * M_PI ) )
           * ( aH * std::log( ( aH + sqrtSum ) / aR ) + 1.5 * ( aR - sqrtSum ) );
}


// ---------------------------------------------------------------------------
// Part 2 — Mutual inductance (Neumann formula for parallel filaments)
// ---------------------------------------------------------------------------

double VIA_MODEL_BUILDER::MutualInductance( double aH, double aD )
{
    if( aH <= 0.0 || aD <= 0.0 )
        return 0.0;

    double d2 = aD * aD;
    double h2 = aH * aH;
    double sqrtSum = std::sqrt( h2 + d2 );

    // M = (µ₀/2π) × [h·ln((h+√(h²+d²))/d) - √(h²+d²) + d]
    return ( MU0 / ( 2.0 * M_PI ) )
           * ( aH * std::log( ( aH + sqrtSum ) / aD ) - sqrtSum + aD );
}


// ---------------------------------------------------------------------------
// Part 3 — Build N×N inductance matrix
// ---------------------------------------------------------------------------

void VIA_MODEL_BUILDER::buildInductanceMatrix()
{
    int n = (int) m_vias.size();

    m_result.L = Eigen::MatrixXd::Zero( n, n );

    for( int i = 0; i < n; i++ )
    {
        m_result.L( i, i ) = SelfInductance( m_vias[i].barrelHeight,
                                              m_vias[i].drillRadius );

        for( int j = i + 1; j < n; j++ )
        {
            // Centre-to-centre distance in board plane (positions in nm → meters)
            double dx = ( m_vias[i].position.x - m_vias[j].position.x ) * 1e-9;
            double dy = ( m_vias[i].position.y - m_vias[j].position.y ) * 1e-9;
            double d = std::sqrt( dx * dx + dy * dy );

            // Use the shorter barrel height (conservative for unequal-length vias)
            double h = std::min( m_vias[i].barrelHeight, m_vias[j].barrelHeight );

            double M = MutualInductance( h, d );

            m_result.L( i, j ) = M;
            m_result.L( j, i ) = M;
        }
    }
}


// ---------------------------------------------------------------------------
// Part 4 — Schur complement: reduce out return vias
// ---------------------------------------------------------------------------

void VIA_MODEL_BUILDER::schurReduce()
{
    int ns = (int) m_signalIdx.size();
    int nr = (int) m_returnIdx.size();

    if( ns == 0 )
        return;

    // Extract sub-matrices
    Eigen::MatrixXd Lss( ns, ns );
    Eigen::MatrixXd Lsr( ns, nr );
    Eigen::MatrixXd Lrs( nr, ns );
    Eigen::MatrixXd Lrr( nr, nr );

    for( int i = 0; i < ns; i++ )
    {
        for( int j = 0; j < ns; j++ )
            Lss( i, j ) = m_result.L( m_signalIdx[i], m_signalIdx[j] );

        for( int j = 0; j < nr; j++ )
            Lsr( i, j ) = m_result.L( m_signalIdx[i], m_returnIdx[j] );
    }

    for( int i = 0; i < nr; i++ )
    {
        for( int j = 0; j < ns; j++ )
            Lrs( i, j ) = m_result.L( m_returnIdx[i], m_signalIdx[j] );

        for( int j = 0; j < nr; j++ )
            Lrr( i, j ) = m_result.L( m_returnIdx[i], m_returnIdx[j] );
    }

    // L_eff = L_SS - L_SR × inv(L_RR) × L_RS
    if( nr > 0 )
        m_result.Leff = Lss - Lsr * Lrr.inverse() * Lrs;
    else
        m_result.Leff = Lss;

    // Extract modal inductances for differential pair (2 signal vias)
    if( ns >= 2 )
    {
        m_result.Ldiff = m_result.Leff( 0, 0 ) - m_result.Leff( 0, 1 );
        m_result.Lcm = m_result.Leff( 0, 0 ) + m_result.Leff( 0, 1 );
        m_result.Ldc = ( m_result.Leff( 0, 0 ) - m_result.Leff( 1, 1 ) ) / 2.0;
    }
    else if( ns == 1 )
    {
        // Single signal via — effective self-inductance with returns shorted
        m_result.Ldiff = m_result.Leff( 0, 0 );
        m_result.Lcm = m_result.Leff( 0, 0 );
        m_result.Ldc = 0.0;
    }
}


// ---------------------------------------------------------------------------
// Part 5 — Pad-to-plane capacitance
// ---------------------------------------------------------------------------

double VIA_MODEL_BUILDER::PadCapacitance( double aRpad, double aRdrill, double aRantipad,
                                          double aEr, double aT )
{
    if( aRpad <= aRdrill || aT <= 0.0 || aEr <= 0.0 )
        return 0.0;

    // The parallel-plate overlap is between the pad annulus and the reference
    // plane copper.  The reference plane has an antipad hole of radius r_antipad,
    // so the overlap inner radius is max(r_drill, r_antipad).
    double rInner = std::max( aRdrill, aRantipad );

    double Cpp = 0.0;

    if( aRpad > rInner )
    {
        double annularArea = M_PI * ( aRpad * aRpad - rInner * rInner );
        Cpp = EPS0 * aEr * annularArea / aT;
    }

    // Fringing from the pad outer edge.  This contributes even when the pad
    // is entirely within the antipad (r_antipad >= r_pad).
    double w = aRpad - aRdrill;

    if( w <= 0.0 )
        return Cpp;

    double Cfringe = EPS0 * aEr * 2.0 * w * std::log( 1.0 + 2.0 * aT / w );

    return Cpp + Cfringe;
}


// ---------------------------------------------------------------------------
// Part 6 — Barrel-to-antipad coaxial capacitance
// ---------------------------------------------------------------------------

double VIA_MODEL_BUILDER::BarrelCapacitance( double aEr, double aT, double aRantipad,
                                             double aRdrill )
{
    if( aRantipad <= aRdrill || aT <= 0.0 || aEr <= 0.0 )
        return 0.0;

    return 2.0 * M_PI * EPS0 * aEr * aT / std::log( aRantipad / aRdrill );
}


// ---------------------------------------------------------------------------
// Part 5+6 combined — compute all capacitances per via
// ---------------------------------------------------------------------------

void VIA_MODEL_BUILDER::computeCapacitances()
{
    int n = (int) m_vias.size();

    m_result.Ctotal.resize( n, 0.0 );

    for( int i = 0; i < n; i++ )
    {
        const VIA_PARAMS& v = m_vias[i];
        double Csum = 0.0;

        // Pad-to-plane capacitance (entry and exit sides)
        for( const VIA_PAD_PLANE& pp : v.adjacentPlanes )
            Csum += PadCapacitance( v.padRadius, v.drillRadius, pp.antipadRadius,
                                    pp.epsilonR, pp.thickness );

        // Barrel-to-antipad at each plane crossing
        for( const VIA_PLANE_CROSSING& pc : v.planeCrossings )
        {
            Csum += BarrelCapacitance( pc.epsilonR, pc.dielectricThickness,
                                       pc.antipadRadius, v.drillRadius );
        }

        m_result.Ctotal[i] = Csum;
    }
}


// ---------------------------------------------------------------------------
// Part 7 — Barrel-to-barrel capacitance (twin-cylinder formula)
// ---------------------------------------------------------------------------

double VIA_MODEL_BUILDER::BarrelToBarrelCapacitance( double aEr, double aH, double aD,
                                                     double aR )
{
    if( aD <= 2.0 * aR || aH <= 0.0 || aEr <= 0.0 )
        return 0.0;

    double ratio = aD / ( 2.0 * aR );

    // acosh(x) = ln(x + sqrt(x²-1))
    double acoshVal = std::acosh( ratio );

    if( acoshVal <= 0.0 )
        return 0.0;

    // C_bb = π·ε₀·εr·h / acosh(d/(2r)) × 0.75 shielding factor
    return M_PI * EPS0 * aEr * aH / acoshVal * 0.75;
}


void VIA_MODEL_BUILDER::computeBarrelToBarrel()
{
    int n = (int) m_vias.size();

    m_result.Cbb = Eigen::MatrixXd::Zero( n, n );

    for( int i = 0; i < n; i++ )
    {
        for( int j = i + 1; j < n; j++ )
        {
            double dx = ( m_vias[i].position.x - m_vias[j].position.x ) * 1e-9;
            double dy = ( m_vias[i].position.y - m_vias[j].position.y ) * 1e-9;
            double d = std::sqrt( dx * dx + dy * dy );

            double h = std::min( m_vias[i].barrelHeight, m_vias[j].barrelHeight );

            // Use the larger barrel outer radius for the cylinder model
            double r = std::max( m_vias[i].barrelOuterRadius, m_vias[j].barrelOuterRadius );

            // Use average εr
            double er = ( m_vias[i].epsilonReff + m_vias[j].epsilonReff ) / 2.0;

            double Cbb = BarrelToBarrelCapacitance( er, h, d, r );

            m_result.Cbb( i, j ) = Cbb;
            m_result.Cbb( j, i ) = Cbb;
        }
    }
}


// ---------------------------------------------------------------------------
// Part 9 — Frequency-dependent series resistance
// ---------------------------------------------------------------------------

double VIA_MODEL_BUILDER::BarrelResistance( double aH, double aRdrill, double aRbarrel,
                                            double aSigma, double aFreq )
{
    if( aH <= 0.0 || aRbarrel <= aRdrill || aSigma <= 0.0 )
        return 0.0;

    double annularArea = M_PI * ( aRbarrel * aRbarrel - aRdrill * aRdrill );
    double Rdc = aH / ( aSigma * annularArea );

    if( aFreq <= 0.0 )
        return Rdc;

    double deltaS = std::sqrt( 1.0 / ( M_PI * aFreq * MU0 * aSigma ) );
    double Rhf = aH / ( aSigma * 2.0 * M_PI * aRbarrel * deltaS );

    // Quadrature blend: smooth transition between DC and skin-effect regimes
    return std::sqrt( Rdc * Rdc + Rhf * Rhf );
}


void VIA_MODEL_BUILDER::computeResistance()
{
    int n = (int) m_vias.size();

    m_result.Rdc.resize( n, 0.0 );

    for( int i = 0; i < n; i++ )
    {
        m_result.Rdc[i] = BarrelResistance( m_vias[i].barrelHeight,
                                            m_vias[i].drillRadius,
                                            m_vias[i].barrelOuterRadius,
                                            SIGMA_COPPER, 0.0 );
    }
}


// ---------------------------------------------------------------------------
// Part 10 — Stub resonance
// ---------------------------------------------------------------------------

double VIA_MODEL_BUILDER::StubResonanceFreq( double aHstub, double aErEff )
{
    if( aHstub <= 0.0 || aErEff <= 0.0 )
        return 0.0;

    double vp = C_LIGHT / std::sqrt( aErEff );
    return vp / ( 4.0 * aHstub );
}


void VIA_MODEL_BUILDER::computeStubResonance()
{
    int n = (int) m_vias.size();

    m_result.fStubRes.resize( n, 0.0 );

    for( int i = 0; i < n; i++ )
        m_result.fStubRes[i] = StubResonanceFreq( m_vias[i].stubLength, m_vias[i].epsilonReff );
}


// ---------------------------------------------------------------------------
// Part 11 — Validity limit
// ---------------------------------------------------------------------------

double VIA_MODEL_BUILDER::ValidityLimit( double aH, double aErEff )
{
    if( aH <= 0.0 || aErEff <= 0.0 )
        return 0.0;

    double vp = C_LIGHT / std::sqrt( aErEff );
    return vp / ( 10.0 * aH );
}


// ---------------------------------------------------------------------------
// Part 12 — Symmetry check
// ---------------------------------------------------------------------------

bool VIA_MODEL_BUILDER::checkSymmetry() const
{
    int ns = (int) m_signalIdx.size();

    if( ns < 2 )
        return true; // single signal via is trivially symmetric

    if( m_result.Ldiff <= 0.0 )
        return false;

    return std::abs( m_result.Ldc ) / m_result.Ldiff < 1e-6;
}
