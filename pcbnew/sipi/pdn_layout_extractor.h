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

#ifndef PDN_LAYOUT_EXTRACTOR_H
#define PDN_LAYOUT_EXTRACTOR_H

#include <math/vector2d.h>
#include <layer_ids.h>

#include <wx/string.h>
#include <map>
#include <set>
#include <vector>

class BOARD;


/**
 * A single SPICE element (R, L, or C) extracted from the PCB layout.
 */
struct PDN_SPICE_ELEMENT
{
    wxString type;    ///< "R", "L", or "C"
    wxString name;    ///< Unique element name (e.g. "Rtr1", "Lvia2")
    wxString nodeA;   ///< First node name
    wxString nodeB;   ///< Second node name
    double   value;   ///< Value in SI units (Ohms, Henries, or Farads)
};


/**
 * A pad port — maps a footprint pad to a SPICE node name.
 */
struct PDN_PAD_PORT
{
    wxString refdes;     ///< Footprint reference designator
    wxString padNumber;  ///< Pad number string
    wxString nodeName;   ///< SPICE node name (e.g. "C1:1")
    VECTOR2I position;   ///< Board position (nm)
};


/**
 * Zone-center node recorded during the zone star-model pass.  Plane-cap
 * extraction needs to reference the same SPICE node a zone's spokes fan out
 * from, otherwise the plane capacitance hangs on nodes nothing else touches.
 */
struct PDN_ZONE_CENTER
{
    PCB_LAYER_ID layer;      ///< Copper layer this center lives on
    wxString     nodeName;   ///< SPICE node name emitted for this zone center
    double       area;       ///< Fill area on this layer (m²), for plane-cap sizing
};


/**
 * Extracted parasitics for a single net.
 */
struct PDN_NET_PARASITICS
{
    wxString                       netName;
    std::vector<PDN_SPICE_ELEMENT> elements;
    std::vector<PDN_PAD_PORT>      padPorts;
    std::vector<wxString>          warnings;
    std::vector<PDN_ZONE_CENTER>   zoneCenters;   ///< One per (zone, layer) with a spoke set
};


/**
 * Complete extraction result for a power/ground net pair.
 */
struct PDN_EXTRACTION_RESULT
{
    PDN_NET_PARASITICS              powerNet;
    PDN_NET_PARASITICS              groundNet;
    std::vector<PDN_SPICE_ELEMENT>  planeCaps;  ///< Plane capacitance bridging power↔ground
};


/**
 * Extracts parasitic R, L, C elements from the PCB layout for PDN analysis.
 *
 * Walks traces, vias, and zones on the power and ground nets, builds a
 * position-keyed graph, and emits SPICE elements. Pads get named ports
 * (refdes:pad_number) for connection to the schematic-side PDN model.
 */
class PDN_LAYOUT_EXTRACTOR
{
public:
    PDN_LAYOUT_EXTRACTOR( const BOARD* aBoard );

    /**
     * Extract parasitics for a power/ground net pair.
     *
     * @param aPowerNetName   Name of the power supply net (e.g. "+3V3")
     * @param aGroundNetName  Name of the ground/reference net (e.g. "GND")
     * @param aPdnRefdes      If non-empty, only keep parasitic paths connecting
     *                        these components (caps, VRM, observation point).
     *                        Dangling stubs to other pads are pruned.
     * @return true if extraction succeeded (at least one net had items)
     */
    bool Extract( const wxString& aPowerNetName, const wxString& aGroundNetName,
                  const std::set<wxString>& aPdnRefdes = {} );

    const PDN_EXTRACTION_RESULT& GetResult() const { return m_result; }

    /**
     * Serialize the extraction result to a text payload for KIWAY transport.
     */
    wxString SerializeResult() const;

    /**
     * Deserialize a text payload back into a PDN_EXTRACTION_RESULT.
     * @return true if parsing succeeded.
     */
    static bool DeserializeResult( const wxString& aPayload, PDN_EXTRACTION_RESULT& aResult );

private:
    PDN_NET_PARASITICS extractNet( int aNetCode,
                                   const std::set<wxString>& aPdnRefdes = {} );
    void findPlaneCaps();

    /// Collapse degree-2 non-terminal nodes in the extracted element list.
    /// Two series R elements become one, two series L become one.  Exact —
    /// no approximation.  Drops numerically trivial elements (sub-µΩ,
    /// sub-pH).  Run once per net after extraction.
    static void coalesceSeries( PDN_NET_PARASITICS& aResult );

    const BOARD*           m_board;
    PDN_EXTRACTION_RESULT  m_result;
};

#endif // PDN_LAYOUT_EXTRACTOR_H
