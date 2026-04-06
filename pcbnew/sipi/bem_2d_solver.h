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

#ifndef BEM_2D_SOLVER_H
#define BEM_2D_SOLVER_H

#include "cross_section.h"

#include <Eigen/Dense>
#include <vector>


/**
 * 2D Galerkin BEM solver for per-unit-length capacitance and impedance
 * of PCB cross-sections with dielectric interfaces.
 *
 * Follows the NMMTL (Numerical Multiconductor Transmission Line) formulation:
 * - Quadratic (3-node) boundary elements with Lagrangian shape functions
 * - Galerkin weighted-residual method (double integration)
 * - Vacuum Green's function with ground-plane image (no ε₀ in kernel)
 * - Interface panels enforce D-normal continuity via flux equation
 * - Charge extraction with local εr weighting (NMMTL convention)
 * - Edge singularity handling at conductor corners (ν-exponent)
 *
 * Interface element grid sizing (see BEM2DSolver/InterfaceGridSensitivity test):
 *   Default:      extent = 3h, spacing = h/3  (< 0.42% vs finest reference)
 *   Low-contrast: extent = 2h, spacing = h    (for εr < 4 on both sides,
 *                                               e.g. air/solder-mask: < 0.1%)
 *
 * Coordinate system:
 *   y = 0: ground plane (Φ = 0)
 *   Conductors and dielectric interfaces at y > 0
 *   Image charges at y < 0
 */
class BEM_2D_SOLVER
{
public:
    BEM_2D_SOLVER();

    void SetGeometry( const XS_GEOMETRY& aGeometry );
    void SetPanelsPerEdge( int aCount );
    void SetFineInterfaceGrid( bool aFine ) { m_fineInterfaceGrid = aFine; }
    void SetEdgeSingularity( bool aEnable ) { m_edgeSingularity = aEnable; }

    /// Override interface element grid: extent = aMult × h, spacing = h / aDiv.
    /// When set, applies to ALL boundaries (ignoring low-contrast optimization).
    void SetInterfaceGrid( double aExtentMult, double aSpacingDiv )
    {
        m_intfExtentMult = aExtentMult;
        m_intfSpacingDiv = aSpacingDiv;
        m_intfGridOverride = true;
    }

    bool Solve();

    const RLGC_RESULT& GetResult() const { return m_result; }

private:
    /// A quadratic boundary element with 3 nodes.
    struct ELEMENT
    {
        double xpts[3];         ///< Global x-coordinates of the 3 nodes
        double ypts[3];         ///< Global y-coordinates of the 3 nodes
        int    nodeIdx[3];      ///< Global node indices into the system matrix
        int    conductorIdx;    ///< Which conductor (>=0), or -1 for interface
        double epsilonR;        ///< CONDUCTOR: local relative permittivity
        double epsPlus;         ///< INTERFACE: εr on the +normal side
        double epsMinus;        ///< INTERFACE: εr on the -normal side
        double normalX;         ///< INTERFACE: outward normal x-component
        double normalY;         ///< INTERFACE: outward normal y-component

        /// Edge singularity data (ν-exponent at conductor corners).
        double nu[2] = { 0.0, 0.0 };       ///< Singular exponent at [0]-end and [2]-end
        bool   isEdge[2] = { false, false }; ///< True if node is a conductor corner
    };

    // --- Shape functions and geometry ---

    /// Evaluate quadratic Lagrangian shape functions at local coordinate ξ ∈ [0,1].
    /// Returns N[0], N[1], N[2] for start, middle, end nodes.
    static void shapeFunctions( double aXi, double aN[3] );

    /// Evaluate shape functions with edge singularity modification.
    /// Falls back to standard shape functions when the element has no edge.
    static void shapeFunctionsEdge( double aXi, const ELEMENT& aElem, double aN[3] );

    /// Evaluate derivatives of shape functions at local coordinate ξ.
    static void shapeDerivatives( double aXi, double aDN[3] );

    /// Compute singular exponent ν for a conductor corner at the junction
    /// of two dielectric materials.  Minimizes the NMMTL transcendental equation.
    /// Returns π/θ₂ for uniform dielectric (ε₁ = ε₂).
    static double findNu( double aEps1, double aEps2, double aTheta1, double aTheta2 );

    /// Compute the Jacobian |dl/dξ| at local coordinate ξ for an element.
    static double jacobian( double aXi, const ELEMENT& aElem );

    /// Interpolate global (x,y) position at local coordinate ξ.
    static void interpolate( double aXi, const ELEMENT& aElem, double& aX, double& aY );

    // --- Green's function kernels (no ε₀) ---

    /// Potential kernel: G(r, r') = ln(d_image / d_direct)
    /// where d_image uses the ground-plane image at (X, -Y).
    double greenPotential( double x, double y, double X, double Y ) const;

    /// Normal-derivative kernel: ∂G/∂n evaluated at (x,y) from source at (X,Y)
    /// with interface normal (nx, ny).
    double greenFlux( double x, double y, double X, double Y,
                      double nx, double ny ) const;

    // --- Element generation ---

    /// Build all conductor and interface elements from the input geometry.
    void buildElements();

    // --- Assembly and solve ---

    /// Assemble the conductor-conductor potential-kernel block (shared
    /// between air and full solves).
    void assembleConductorBlock( Eigen::MatrixXd& aA ) const;

    /// Solve the full system using a pre-assembled conductor block.
    Eigen::MatrixXd solveCapacitanceFull( const Eigen::MatrixXd& aCondBlock );

    /// Solve the conductor-only (free-space) system using a pre-assembled
    /// conductor block.
    Eigen::MatrixXd solveCapacitanceAir( const Eigen::MatrixXd& aCondBlock );

    /// Integrate the potential kernel G over an inner element, evaluated
    /// at observation point (x,y).  Returns value[3] for each node.
    void intervalConductor( double x, double y, const ELEMENT& aInner,
                            double aValue[3] ) const;

    /// Self-element integration with singularity splitting.
    void intervalSelfConductor( double x, double y, const ELEMENT& aElem,
                                double aValue[3], double aXiOuter ) const;

    /// Integrate the flux kernel ∂G/∂n over an inner element.
    void intervalFlux( double x, double y, const ELEMENT& aInner,
                       double aValue[3], double nx, double ny ) const;

    /// Self-element integration for interface flux kernel.
    void intervalSelfFlux( double x, double y, const ELEMENT& aElem,
                           double aValue[3], double aXiOuter,
                           double nx, double ny ) const;

    /// Build and solve the load vector for a given active conductor.
    void buildLoadVector( Eigen::VectorXd& aB, int aActiveConductor ) const;

    /// Extract charge on each conductor from the solved σ vector.
    void extractCharge( const Eigen::VectorXd& aSigma, double aEpsFactor,
                        Eigen::VectorXd& aQ ) const;

    // --- Data ---

    XS_GEOMETRY   m_geometry;
    RLGC_RESULT   m_result;
    int           m_panelsPerEdge;
    bool          m_edgeSingularity;
    bool          m_fineInterfaceGrid;
    bool          m_intfGridOverride;
    double        m_intfExtentMult;
    double        m_intfSpacingDiv;

    double        m_lengthScale;    ///< NMMTL length_scale = half_minimum_dimension

    /// All conductor elements, grouped by conductor index.
    std::vector<ELEMENT> m_condElements;
    /// All interface elements.
    std::vector<ELEMENT> m_intfElements;

    int m_numCondNodes;     ///< Total conductor + ground nodes
    int m_numIntfNodes;     ///< Total interface nodes
    int m_numConductors;    ///< Number of signal conductors (excludes ground)

    /// Conductor index for groundwire elements (V=0, charge not extracted).
    static constexpr int GROUND_CONDUCTOR_IDX = -2;
};

#endif // BEM_2D_SOLVER_H
