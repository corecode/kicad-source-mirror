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

#ifndef VIA_MODEL_H
#define VIA_MODEL_H

#include <math/vector2d.h>

#include <Eigen/Dense>
#include <vector>


/**
 * Role of a via in a cluster: signal positive, signal negative, or return (ground).
 */
enum class VIA_ROLE
{
    SIGNAL_P,
    SIGNAL_N,
    RETURN
};


/**
 * A reference plane crossing along the via barrel.
 * Each crossing contributes a coaxial barrel-to-plane capacitance.
 */
struct VIA_PLANE_CROSSING
{
    double epsilonR = 4.5;       ///< Dielectric εr at this crossing
    double dielectricThickness = 0.0; ///< Thickness of dielectric at this plane (m)
    double antipadRadius = 0.0;  ///< Antipad clearance radius at this plane (m)
};


/**
 * An adjacent reference plane for pad-to-plane capacitance.
 */
struct VIA_PAD_PLANE
{
    double epsilonR = 4.5;       ///< Dielectric εr between pad and plane
    double thickness = 0.0;      ///< Dielectric layer thickness (m)
    double antipadRadius = 0.0;  ///< Antipad clearance in the reference plane (m), 0 = unknown
};


/**
 * Input parameters for one via in a cluster.
 * All dimensions in meters.
 */
struct VIA_PARAMS
{
    VECTOR2I position;               ///< Center (x,y) in board coordinates (nm)
    double   drillRadius = 0.0;      ///< Inner radius of copper barrel (m)
    double   barrelOuterRadius = 0.0;///< Outer radius = drillRadius + plating thickness (m)
    double   padRadius = 0.0;        ///< Pad radius (m), or equivalent for rectangular
    double   barrelHeight = 0.0;     ///< Total barrel length through dielectric (m)
    double   stubLength = 0.0;       ///< Unused stub below signal exit (m), 0 for blind
    double   epsilonReff = 4.4;      ///< Effective εr for stub velocity calculation

    VIA_ROLE role = VIA_ROLE::RETURN;

    /// Per-reference-plane crossing data for barrel-to-antipad capacitance.
    std::vector<VIA_PLANE_CROSSING> planeCrossings;

    /// Adjacent reference planes for pad-to-plane capacitance (typically 1 or 2).
    std::vector<VIA_PAD_PLANE> adjacentPlanes;
};


/**
 * Results of via cluster parasitic extraction.
 */
struct VIA_CLUSTER_RESULT
{
    int numVias = 0;               ///< Total vias in cluster

    // --- Raw N×N matrices ---
    Eigen::MatrixXd L;             ///< Partial inductance matrix (H)
    Eigen::MatrixXd Cbb;           ///< Barrel-to-barrel capacitance matrix (F)

    // --- Schur-reduced effective inductance at signal ports ---
    Eigen::MatrixXd Leff;          ///< Effective L at signal ports (H), Ns × Ns
    double Ldiff = 0.0;            ///< Differential mode inductance (H)
    double Lcm = 0.0;              ///< Common mode inductance (H)
    double Ldc = 0.0;              ///< Mode-conversion inductance (H), ~0 for symmetric

    // --- Per-via scalar parasitics ---
    std::vector<double> Ctotal;    ///< C_pad + C_barrel per via (F)
    std::vector<double> Rdc;       ///< DC resistance per via (Ω)
    std::vector<double> fStubRes;  ///< Stub resonance frequency (Hz), 0 if no stub

    double fMax = 0.0;             ///< Pi-model validity limit (Hz)
    bool   symmetric = false;      ///< True if symmetry check passed
};


/**
 * Analytical multi-via parasitic extraction.
 *
 * Given a cluster of N parallel via barrels (signal + return), computes the
 * partial inductance matrix, reduces it via Schur complement to effective
 * differential/common-mode inductances, computes all capacitive parasitics,
 * and assembles pi-equivalent circuit parameters.
 *
 * All formulas are exact closed-form (Neumann integral for parallel filaments,
 * coaxial capacitance, twin-cylinder capacitance). Valid up to f_max = v_p/(10h).
 *
 * Reference: physics spec "Via parasitic extraction — physics reference for implementors"
 */
class VIA_MODEL_BUILDER
{
public:
    VIA_MODEL_BUILDER();

    /**
     * Set the via cluster to analyze.
     */
    void SetCluster( const std::vector<VIA_PARAMS>& aVias );

    /**
     * Run the extraction.  Returns true on success.
     */
    bool Compute();

    const VIA_CLUSTER_RESULT& GetResult() const { return m_result; }

    // --- Individual formula accessors (public for testing) ---

    /**
     * Part 1: Goldfarb-Pucel self-inductance of a cylindrical via barrel.
     * @param aH  Barrel length through dielectric (m)
     * @param aR  Drill radius (m)
     * @return Self-inductance (H)
     */
    static double SelfInductance( double aH, double aR );

    /**
     * Part 2: Neumann mutual inductance between two parallel via barrels.
     * @param aH  Barrel length (m), assumed equal for both vias
     * @param aD  Centre-to-centre distance (m), must be > 2*radius
     * @return Mutual inductance (H)
     */
    static double MutualInductance( double aH, double aD );

    /**
     * Part 5: Pad-to-plane capacitance (parallel plate + fringing).
     *
     * Accounts for the antipad clearance in the reference plane: the parallel-plate
     * overlap is only the annular ring from max(r_drill, r_antipad) to r_pad.
     * If r_antipad >= r_pad, the pad is entirely within the antipad void and
     * only fringing capacitance contributes.
     *
     * @param aRpad      Pad radius (m)
     * @param aRdrill    Drill radius (m)
     * @param aRantipad  Antipad radius in the reference plane (m), 0 = no antipad
     * @param aEr        Dielectric εr
     * @param aT         Dielectric thickness (m)
     * @return Capacitance (F)
     */
    static double PadCapacitance( double aRpad, double aRdrill, double aRantipad,
                                  double aEr, double aT );

    /**
     * Part 6: Barrel-to-antipad coaxial capacitance per plane crossing.
     * @param aEr       Dielectric εr
     * @param aT        Dielectric thickness at this plane (m)
     * @param aRantipad Antipad radius (m)
     * @param aRdrill   Drill radius (m)
     * @return Capacitance (F)
     */
    static double BarrelCapacitance( double aEr, double aT, double aRantipad, double aRdrill );

    /**
     * Part 7: Barrel-to-barrel capacitance (twin-cylinder formula).
     * @param aEr  Effective εr of surrounding dielectric
     * @param aH   Barrel length (m)
     * @param aD   Centre-to-centre distance (m)
     * @param aR   Barrel outer radius (m)
     * @return Capacitance (F), includes 0.75 shielding correction
     */
    static double BarrelToBarrelCapacitance( double aEr, double aH, double aD, double aR );

    /**
     * Part 9: Frequency-dependent series resistance.
     * @param aH         Barrel length (m)
     * @param aRdrill    Drill radius (m)
     * @param aRbarrel   Barrel outer radius (m)
     * @param aSigma     Copper conductivity (S/m)
     * @param aFreq      Frequency (Hz), 0 for DC
     * @return Resistance (Ω)
     */
    static double BarrelResistance( double aH, double aRdrill, double aRbarrel,
                                    double aSigma, double aFreq );

    /**
     * Part 10: Stub resonance frequency.
     * @param aHstub  Stub length (m)
     * @param aErEff  Effective εr for velocity calculation
     * @return Resonance frequency (Hz), 0 if no stub
     */
    static double StubResonanceFreq( double aHstub, double aErEff );

    /**
     * Part 11: Pi-model validity frequency limit.
     * @param aH     Barrel length (m)
     * @param aErEff Effective εr
     * @return Maximum valid frequency (Hz)
     */
    static double ValidityLimit( double aH, double aErEff );

private:
    void buildInductanceMatrix();
    void schurReduce();
    void computeCapacitances();
    void computeBarrelToBarrel();
    void computeResistance();
    void computeStubResonance();
    bool checkSymmetry() const;

    std::vector<VIA_PARAMS> m_vias;
    VIA_CLUSTER_RESULT      m_result;

    // Indices of signal and return vias within m_vias
    std::vector<int> m_signalIdx;
    std::vector<int> m_returnIdx;
};


#endif // VIA_MODEL_H
