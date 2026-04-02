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

#ifndef IMPEDANCE_PROFILE_H
#define IMPEDANCE_PROFILE_H

#include <sipi/cross_section.h>

#include <layer_ids.h>
#include <math/vector2d.h>

#include <vector>

#include <wx/string.h>

class BOARD;
class DRC_RTREE;
class PAD;


/**
 * Per-sample result from an impedance profile.
 */
struct IMPEDANCE_SAMPLE
{
    double       distNm = 0.0;        ///< Distance along trace (nm)
    double       delayPs = 0.0;       ///< Cumulative propagation delay (ps)
    double       z0 = 0.0;            ///< Single-ended Z₀ (Ohms)
    double       erEff = 1.0;         ///< Effective εr
    XS_GEOMETRY  geometry;             ///< Cross-section geometry
    VECTOR2I     boardPos;             ///< Board position (nm)
    VECTOR2D     tangent;              ///< Unit tangent at this point
    PCB_LAYER_ID layer = F_Cu;         ///< Copper layer
    int          signalWidth = 0;      ///< Effective signal width (nm)
    bool         hasRefAbove = false;
    bool         hasRefBelow = false;
    double       hAbove = 0.0;        ///< Dielectric height above (m)
    double       hBelow = 0.0;        ///< Dielectric height below (m)
    int          neighborCount = 0;
    int          groundwireCount = 0;

    // Differential pair fields (populated when coupledNetCode > 0)
    double       zdiff = 0.0;         ///< Differential impedance (0 = not coupled)
    double       erEffOdd = 0.0;      ///< Odd-mode εr_eff
    bool         isDiffPair = false;   ///< True if coupled partner found at this sample
    double       dpGapUm = 0.0;       ///< Edge-to-edge gap P↔N (µm)
};


/**
 * Per-sample result from a differential impedance profile.
 */
struct DIFF_SAMPLE
{
    double      delayPs = 0.0;        ///< Propagation delay from P (ps)
    double      distMm = 0.0;         ///< Distance along P (mm)
    double      z0P = 0.0;            ///< Single-ended Z₀ of P (Ohms)
    double      z0N = 0.0;            ///< Single-ended Z₀ of N (Ohms)
    double      zdiff = 0.0;          ///< Differential impedance (Ohms)
    double      erEffOdd = 1.0;       ///< Odd-mode εr_eff (coupled only)
    bool        coupled = false;      ///< True if traces are coupled at this point
    double      gapUm = 0.0;          ///< Edge-to-edge gap P↔N (µm, coupled only)
    VECTOR2I    boardPosP;             ///< P trace board position
    XS_GEOMETRY geometry;              ///< Cross-section (2-conductor when coupled)
};


/**
 * Single-ended impedance profile for one net.
 *
 * Walks the net, samples at uniform distance intervals, builds cross-section
 * geometry at each point, and solves via the BEM for Z₀ and εr_eff.
 * Optionally detects a diff-pair partner on the cut line at each sample.
 */
class SE_PROFILE
{
public:
    SE_PROFILE();

    /**
     * Compute the impedance profile for a single net.
     *
     * @param aBoard           Board to analyze.
     * @param aNetCode         Net code to walk.
     * @param aFrom            Starting position hint (nearest pad selected).
     * @param aRtree           Optional shared spatial index.
     * @param aCoupledNetCode  Coupled diff-pair net (0 = single-ended only).
     * @return true on success, false on error (check GetError()).
     */
    bool Compute( const BOARD* aBoard, int aNetCode,
                  const VECTOR2I& aFrom = VECTOR2I( 0, 0 ),
                  DRC_RTREE* aRtree = nullptr,
                  int aCoupledNetCode = 0 );

    const std::vector<IMPEDANCE_SAMPLE>& GetSamples() const { return m_samples; }
    double GetTotalLength() const { return m_totalLength; }
    double GetTotalDelay() const { return m_totalDelay; }

    IMPEDANCE_SAMPLE AtDelay( double aDelayPs ) const;

    PAD* GetStartPad() const { return m_startPad; }
    PAD* GetEndPad() const { return m_endPad; }
    const wxString& GetError() const { return m_error; }

private:
    std::vector<IMPEDANCE_SAMPLE> m_samples;
    double   m_totalLength = 0.0;
    double   m_totalDelay = 0.0;
    PAD*     m_startPad = nullptr;
    PAD*     m_endPad = nullptr;
    wxString m_error;
};


/**
 * Differential impedance profile for a pair of nets.
 *
 * Walks P with diff-pair detection (finds N geometrically on the cut line).
 * Walks N independently for skew and uncoupled Z₀_N fallback.
 */
class DIFF_PROFILE
{
public:
    DIFF_PROFILE();

    bool Compute( const BOARD* aBoard, int aNetCodeP, int aNetCodeN );

    const SE_PROFILE& GetProfileP() const { return m_profileP; }
    const SE_PROFILE& GetProfileN() const { return m_profileN; }
    const std::vector<DIFF_SAMPLE>& GetSamples() const { return m_diffSamples; }

    double GetSkewPs() const { return m_skewPs; }
    const wxString& GetError() const { return m_error; }

private:
    void buildFromProfiles();

    SE_PROFILE  m_profileP;
    SE_PROFILE  m_profileN;
    std::vector<DIFF_SAMPLE> m_diffSamples;
    double      m_skewPs = 0.0;
    wxString    m_error;
};


#endif // IMPEDANCE_PROFILE_H
