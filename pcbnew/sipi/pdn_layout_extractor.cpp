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

#include <sipi/pdn_layout_extractor.h>
#include <sipi/stackup_reader.h>
#include <sipi/zone_impedance.h>

#include <board.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
#include <pad.h>
#include <pcb_track.h>
#include <zone.h>

#include <geometry/shape_poly_set.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>
#include <vector>


static constexpr double NM_TO_M = 1e-9;
static constexpr double MU0 = 4.0 * M_PI * 1e-7;
static constexpr double RHO_CU = 1.68e-8; // copper resistivity Ωm
static constexpr double EPS0 = 8.854187817e-12;

// 10 µm coincidence tolerance for anchor merging.  Larger than any router-
// induced rounding drift, smaller than any reasonable pad pitch.
static constexpr int ANCHOR_MERGE_NM = 10000;

// Elements below these thresholds are dropped during coalescing — they are
// numerical noise (µΩ trace stubs, pH of air) that only clutter the netlist.
static constexpr double R_DROP_THRESHOLD = 1e-6;   // 1 µΩ
static constexpr double L_DROP_THRESHOLD = 1e-12;  // 1 pH


namespace
{

/// Disjoint-set / union-find keyed by int.  Path compression + rank.
class UnionFind
{
public:
    explicit UnionFind( int aSize ) : m_parent( aSize ), m_rank( aSize, 0 )
    {
        for( int i = 0; i < aSize; ++i )
            m_parent[i] = i;
    }

    int Find( int aX )
    {
        while( m_parent[aX] != aX )
        {
            m_parent[aX] = m_parent[m_parent[aX]];
            aX = m_parent[aX];
        }
        return aX;
    }

    void Union( int aA, int aB )
    {
        aA = Find( aA );
        aB = Find( aB );

        if( aA == aB )
            return;

        if( m_rank[aA] < m_rank[aB] )
            std::swap( aA, aB );

        m_parent[aB] = aA;

        if( m_rank[aA] == m_rank[aB] )
            m_rank[aA]++;
    }

private:
    std::vector<int> m_parent;
    std::vector<int> m_rank;
};


/// One electrical terminal: an item's connection point on a specific copper
/// layer.  Multiple anchors within ANCHOR_MERGE_NM on the same layer collapse
/// to one SPICE node.
struct Anchor
{
    VECTOR2I               pos;
    PCB_LAYER_ID           layer;
    BOARD_CONNECTED_ITEM*  item;       ///< Owning pad / track / via
    int                    endpointIdx; ///< 0 = start/single, 1 = end (tracks)
};


/// Lookup key for (item, endpoint) → anchor index.
using ItemEndpoint = std::pair<const BOARD_CONNECTED_ITEM*, int>;

struct ItemEndpointHash
{
    size_t operator()( const ItemEndpoint& aKey ) const noexcept
    {
        return std::hash<const void*>{}( aKey.first )
             ^ ( static_cast<size_t>( aKey.second ) * 0x9e3779b97f4a7c15ULL );
    }
};


/// Per-layer anchor key keyed for (item, layer) lookup on multi-layer items
/// (pads, vias).  For tracks, endpointIdx distinguishes start from end; layer
/// is implicit.
struct ItemLayer
{
    const BOARD_CONNECTED_ITEM* item;
    PCB_LAYER_ID                layer;

    bool operator==( const ItemLayer& aOther ) const
    {
        return item == aOther.item && layer == aOther.layer;
    }
};

struct ItemLayerHash
{
    size_t operator()( const ItemLayer& aKey ) const noexcept
    {
        return std::hash<const void*>{}( aKey.item )
             ^ ( static_cast<size_t>( aKey.layer ) * 0x9e3779b97f4a7c15ULL );
    }
};


/// Build an SPICE-safe net prefix: alnum only, lowercase, leading-digit
/// escape.  Empty net name falls back to "net<code>_".
wxString buildNetPrefix( const NETINFO_ITEM* aNet, int aNetCode )
{
    wxString prefix;

    if( aNet )
    {
        for( wxUniChar ch : aNet->GetNetname() )
        {
            if( wxIsalnum( ch ) )
                prefix += wxUniChar( wxTolower( ch ) );
        }
    }

    if( prefix.IsEmpty() )
        prefix = wxString::Format( wxS( "net%d" ), aNetCode );

    if( wxIsdigit( prefix[0] ) )
        prefix = wxS( "n" ) + prefix;

    prefix += wxS( "_" );
    return prefix;
}


/// Enumerate every copper layer a via passes through, in stackup order.
LSEQ viaLayerSpan( const BOARD* aBoard, const PCB_VIA* aVia )
{
    LSEQ allCu = aBoard->GetEnabledLayers().CuStack();
    LSEQ span;
    bool inside = false;

    for( PCB_LAYER_ID layer : allCu )
    {
        if( layer == aVia->TopLayer() )
            inside = true;

        if( inside )
            span.push_back( layer );

        if( layer == aVia->BottomLayer() )
            break;
    }

    return span;
}


} // namespace


PDN_LAYOUT_EXTRACTOR::PDN_LAYOUT_EXTRACTOR( const BOARD* aBoard ) :
        m_board( aBoard )
{
}


bool PDN_LAYOUT_EXTRACTOR::Extract( const wxString& aPowerNetName, const wxString& aGroundNetName,
                                    const std::set<wxString>& aPdnRefdes )
{
    m_result = PDN_EXTRACTION_RESULT();

    NETINFO_ITEM* powerNet = m_board->FindNet( aPowerNetName );
    NETINFO_ITEM* groundNet = m_board->FindNet( aGroundNetName );

    if( !powerNet && !groundNet )
        return false;

    int powerNetCode = powerNet ? powerNet->GetNetCode() : -1;
    int groundNetCode = groundNet ? groundNet->GetNetCode() : -1;

    if( powerNetCode > 0 )
    {
        m_result.powerNet = extractNet( powerNetCode, aPdnRefdes );
        m_result.powerNet.netName = aPowerNetName;
    }

    if( groundNetCode > 0 )
    {
        m_result.groundNet = extractNet( groundNetCode, aPdnRefdes );
        m_result.groundNet.netName = aGroundNetName;
    }

    findPlaneCaps();

    return true;
}


PDN_NET_PARASITICS PDN_LAYOUT_EXTRACTOR::extractNet( int aNetCode,
                                                     const std::set<wxString>& aPdnRefdes )
{
    PDN_NET_PARASITICS result;

    auto connectivity = m_board->GetConnectivity();

    if( !connectivity )
        return result;

    std::vector<BOARD_CONNECTED_ITEM*> items = connectivity->GetNetItems(
            aNetCode, { PCB_PAD_T, PCB_TRACE_T, PCB_ARC_T, PCB_VIA_T, PCB_ZONE_T } );

    if( items.empty() )
        return result;

    // ------------------------------------------------------------------
    // Phase 1: enumerate synthetic anchors
    // ------------------------------------------------------------------
    // One anchor per (item, copper-layer it occupies, optional endpoint).
    // These are the electrical terminals union-find will merge.  Zones are
    // not anchors — they emit spokes to other items' anchors in phase 5.
    //
    // Iteration order is the connectivity-reported item order, which is
    // stable for a given build.  For determinism across builds we also sort
    // items by (type, uuid, endpoint) below.

    std::sort( items.begin(), items.end(),
               []( const BOARD_CONNECTED_ITEM* aA, const BOARD_CONNECTED_ITEM* aB )
               {
                   if( aA->Type() != aB->Type() )
                       return aA->Type() < aB->Type();

                   return aA->m_Uuid < aB->m_Uuid;
               } );

    std::vector<Anchor>                                           anchors;
    std::unordered_map<ItemEndpoint, int, ItemEndpointHash>       endpointToAnchor;
    std::unordered_map<ItemLayer, int, ItemLayerHash>             itemLayerToAnchor;

    for( BOARD_CONNECTED_ITEM* item : items )
    {
        switch( item->Type() )
        {
        case PCB_PAD_T:
        {
            PAD* pad = static_cast<PAD*>( item );

            for( PCB_LAYER_ID layer : pad->GetLayerSet().CuStack() )
            {
                int idx = static_cast<int>( anchors.size() );
                anchors.push_back( { pad->GetPosition(), layer, item, 0 } );
                itemLayerToAnchor[{ item, layer }] = idx;
            }
            break;
        }

        case PCB_TRACE_T:
        case PCB_ARC_T:
        {
            PCB_TRACK* track = static_cast<PCB_TRACK*>( item );
            PCB_LAYER_ID layer = track->GetLayer();

            int idxS = static_cast<int>( anchors.size() );
            anchors.push_back( { track->GetStart(), layer, item, 0 } );
            endpointToAnchor[{ item, 0 }] = idxS;

            int idxE = static_cast<int>( anchors.size() );
            anchors.push_back( { track->GetEnd(), layer, item, 1 } );
            endpointToAnchor[{ item, 1 }] = idxE;
            break;
        }

        case PCB_VIA_T:
        {
            PCB_VIA* via = static_cast<PCB_VIA*>( item );

            for( PCB_LAYER_ID layer : viaLayerSpan( m_board, via ) )
            {
                int idx = static_cast<int>( anchors.size() );
                anchors.push_back( { via->GetPosition(), layer, item, 0 } );
                itemLayerToAnchor[{ item, layer }] = idx;
            }
            break;
        }

        default:
            break; // zones handled in phase 5
        }
    }

    // ------------------------------------------------------------------
    // Phase 2: union-find merge on spatial-hash buckets
    // ------------------------------------------------------------------
    // Two anchors on the same layer whose true distance ≤ ANCHOR_MERGE_NM
    // are the same SPICE node.  Net filtering was already done by
    // GetNetItems, so any coincidence here is genuine electrical union.

    UnionFind uf( static_cast<int>( anchors.size() ) );

    using BucketKey = std::tuple<int, int, int>; // (bucket_x, bucket_y, layer)
    std::map<BucketKey, std::vector<int>> buckets;

    auto bucketOf = []( const VECTOR2I& aPos, PCB_LAYER_ID aLayer ) -> BucketKey
    {
        return { aPos.x / ANCHOR_MERGE_NM, aPos.y / ANCHOR_MERGE_NM,
                 static_cast<int>( aLayer ) };
    };

    const int64_t tol2 = static_cast<int64_t>( ANCHOR_MERGE_NM )
                       * static_cast<int64_t>( ANCHOR_MERGE_NM );

    for( int i = 0; i < static_cast<int>( anchors.size() ); ++i )
    {
        const Anchor& a = anchors[i];
        BucketKey     center = bucketOf( a.pos, a.layer );
        int           cx = std::get<0>( center );
        int           cy = std::get<1>( center );

        for( int dx = -1; dx <= 1; ++dx )
        {
            for( int dy = -1; dy <= 1; ++dy )
            {
                auto it = buckets.find( { cx + dx, cy + dy, std::get<2>( center ) } );

                if( it == buckets.end() )
                    continue;

                for( int j : it->second )
                {
                    if( ( anchors[j].pos - a.pos ).SquaredEuclideanNorm() <= tol2 )
                        uf.Union( i, j );
                }
            }
        }

        buckets[center].push_back( i );
    }

    // ------------------------------------------------------------------
    // Phase 3: assign SPICE node names
    // ------------------------------------------------------------------
    // Pads contribute preferred names (refdes_padnum).  Remaining unions
    // get auto-generated internal names.  Two pads unioned together keep
    // the first pad's name (iteration order is deterministic from the
    // sort above).

    wxString netPrefix = buildNetPrefix( m_board->FindNet( aNetCode ), aNetCode );

    std::unordered_map<int, wxString> nodeName;   // uf root → name

    for( int i = 0; i < static_cast<int>( anchors.size() ); ++i )
    {
        PAD* pad = dynamic_cast<PAD*>( anchors[i].item );

        if( !pad )
            continue;

        int root = uf.Find( i );

        if( nodeName.count( root ) )
            continue;

        FOOTPRINT* fp = pad->GetParentFootprint();

        if( !fp )
            continue;

        nodeName[root] = fp->GetReference() + wxS( "_" ) + pad->GetNumber();
    }

    int internalIdx = 1;

    for( int i = 0; i < static_cast<int>( anchors.size() ); ++i )
    {
        int root = uf.Find( i );

        if( nodeName.count( root ) )
            continue;

        nodeName[root] = wxString::Format( wxS( "%sn%d" ), netPrefix, internalIdx++ );
    }

    auto anchorNode = [&]( int aAnchorIdx ) -> wxString
    {
        return nodeName[uf.Find( aAnchorIdx )];
    };

    // ------------------------------------------------------------------
    // Phase 4: record pad ports
    // ------------------------------------------------------------------

    std::set<const BOARD_CONNECTED_ITEM*> portedPads;

    for( int i = 0; i < static_cast<int>( anchors.size() ); ++i )
    {
        PAD* pad = dynamic_cast<PAD*>( anchors[i].item );

        if( !pad )
            continue;

        if( !portedPads.insert( pad ).second )
            continue;

        FOOTPRINT* fp = pad->GetParentFootprint();

        if( !fp )
            continue;

        PDN_PAD_PORT port;
        port.refdes = fp->GetReference();
        port.padNumber = pad->GetNumber();
        port.nodeName = anchorNode( i );
        port.position = pad->GetPosition();
        result.padPorts.push_back( port );
    }

    // ------------------------------------------------------------------
    // Phase 5: emit parasitic elements
    // ------------------------------------------------------------------

    STACKUP_READER stackup( m_board );
    int            elemIdx = 1;

    // --- Traces → R+L ---
    for( BOARD_CONNECTED_ITEM* item : items )
    {
        if( item->Type() != PCB_TRACE_T && item->Type() != PCB_ARC_T )
            continue;

        PCB_TRACK* track = static_cast<PCB_TRACK*>( item );

        auto startIt = endpointToAnchor.find( { item, 0 } );
        auto endIt = endpointToAnchor.find( { item, 1 } );

        if( startIt == endpointToAnchor.end() || endIt == endpointToAnchor.end() )
            continue;

        wxString nodeA = anchorNode( startIt->second );
        wxString nodeB = anchorNode( endIt->second );

        if( nodeA == nodeB )
            continue;

        double length = track->GetLength() * NM_TO_M;
        double width = track->GetWidth() * NM_TO_M;

        LAYER_GEOMETRY layerGeom = stackup.GetLayerGeometry(
                track->GetLayer(), track->GetStart(), track->GetWidth() );

        double tCopper = layerGeom.traceThickness > 0.0 ? layerGeom.traceThickness : 35e-6;

        double traceR = RHO_CU * length / ( width * tCopper );

        double h = 0.0;

        if( layerGeom.hasRefBelow )
            h = layerGeom.hBelow;
        else if( layerGeom.hasRefAbove )
            h = layerGeom.hAbove;
        else
            h = 0.2e-3;

        double traceL = MU0 * h * length / width;

        wxString rName   = wxString::Format( wxS( "%sRtr%d" ), netPrefix, elemIdx );
        wxString lName   = wxString::Format( wxS( "%sLtr%d" ), netPrefix, elemIdx );
        wxString midNode = wxString::Format( wxS( "%strm%d" ), netPrefix, elemIdx );

        result.elements.push_back( { wxS( "R" ), rName, nodeA, midNode, traceR } );
        result.elements.push_back( { wxS( "L" ), lName, midNode, nodeB, traceL } );
        elemIdx++;
    }

    // --- Vias → R+L per inter-layer segment ---
    for( BOARD_CONNECTED_ITEM* item : items )
    {
        if( item->Type() != PCB_VIA_T )
            continue;

        PCB_VIA* via = static_cast<PCB_VIA*>( item );
        LSEQ     span = viaLayerSpan( m_board, via );

        if( span.size() < 2 )
            continue;

        double drillRadius = via->GetDrillValue() * 0.5 * NM_TO_M;
        double padRadius = via->GetWidth( via->TopLayer() ) * 0.5 * NM_TO_M;
        double barrelOuter = drillRadius + 25e-6; // ~1mil plating

        VIA_PARAMS viaGeom = stackup.GetViaGeometry(
                via->TopLayer(), via->BottomLayer(), via->TopLayer(),
                via->GetPosition(), drillRadius, padRadius );

        double barrelLen = viaGeom.barrelHeight > 0.0 ? viaGeom.barrelHeight : 1.6e-3;
        double barrelArea = M_PI * ( barrelOuter * barrelOuter - drillRadius * drillRadius );
        double viaRTotal = RHO_CU * barrelLen / barrelArea;

        double antipadR = padRadius * 1.5;

        if( !viaGeom.planeCrossings.empty() )
            antipadR = viaGeom.planeCrossings[0].antipadRadius;

        double viaLTotal = ( MU0 / ( 2.0 * M_PI ) ) * barrelLen
                           * std::log( antipadR / barrelOuter );

        if( viaLTotal < 0.0 )
            viaLTotal = 0.1e-9;

        int    nSeg = static_cast<int>( span.size() ) - 1;
        double segR = viaRTotal / nSeg;
        double segL = viaLTotal / nSeg;

        for( int i = 0; i < nSeg; ++i )
        {
            auto aIt = itemLayerToAnchor.find( { item, span[i] } );
            auto bIt = itemLayerToAnchor.find( { item, span[i + 1] } );

            if( aIt == itemLayerToAnchor.end() || bIt == itemLayerToAnchor.end() )
                continue;

            wxString aNode = anchorNode( aIt->second );
            wxString bNode = anchorNode( bIt->second );

            wxString rName = wxString::Format( wxS( "%sRvia%d_%d" ), netPrefix, elemIdx, i );
            wxString lName = wxString::Format( wxS( "%sLvia%d_%d" ), netPrefix, elemIdx, i );
            wxString mid   = wxString::Format( wxS( "%svia%d_%dm" ), netPrefix, elemIdx, i );

            result.elements.push_back( { wxS( "R" ), rName, aNode, mid, segR } );
            result.elements.push_back( { wxS( "L" ), lName, mid, bNode, segL } );
        }

        elemIdx++;
    }

    // --- Zones → star model with spoke R+L ---
    //
    // A zone connects to any item on the same net whose anchor falls inside
    // the fill polygon on the zone's layer.  We go straight to geometric
    // containment instead of consulting connectivity's zone-item links —
    // all anchors here are already net-filtered (extractNet scope), and
    // containment is stable even when the interactive zone filler path
    // hasn't built those links (e.g. headless test fixtures).

    std::set<const ZONE*> processedZones;

    for( BOARD_CONNECTED_ITEM* item : items )
    {
        if( item->Type() != PCB_ZONE_T )
            continue;

        ZONE* zone = static_cast<ZONE*>( item );

        if( !processedZones.insert( zone ).second )
            continue;

        for( PCB_LAYER_ID layer : zone->GetLayerSet().Seq() )
        {
            if( !IsCopperLayer( layer ) )
                continue;

            if( !zone->HasFilledPolysForLayer( layer ) )
                continue;

            const std::shared_ptr<SHAPE_POLY_SET>& fills = zone->GetFilledPolysList( layer );

            if( !fills || fills->OutlineCount() == 0 )
                continue;

            // Collect (position, node-name) pairs for every anchor on this
            // layer whose position is inside the fill.  Dedupe by node-name
            // so two anchors already unioned to one SPICE node (a pad and
            // the track endpoint landing on it) collapse to one spoke.
            std::vector<VECTOR2I> connPts;
            std::vector<wxString> connNodes;
            std::set<wxString>    seen;

            for( int i = 0; i < static_cast<int>( anchors.size() ); ++i )
            {
                const Anchor& a = anchors[i];

                if( a.layer != layer )
                    continue;

                if( a.item == zone )
                    continue;   // self-reference

                if( !fills->Contains( a.pos ) )
                    continue;

                wxString nn = anchorNode( i );

                if( !seen.insert( nn ).second )
                    continue;

                connPts.push_back( a.pos );
                connNodes.push_back( nn );
            }

            if( connPts.empty() )
                continue;

            LAYER_GEOMETRY layerGeom = stackup.GetLayerGeometry( layer, connPts[0], 1000000 );

            double tCopper = layerGeom.traceThickness > 0.0 ? layerGeom.traceThickness : 35e-6;
            double rSheet = RHO_CU / tCopper;
            double hDiel = 0.0;

            if( layerGeom.hasRefBelow )
                hDiel = layerGeom.hBelow;
            else if( layerGeom.hasRefAbove )
                hDiel = layerGeom.hAbove;
            else
                hDiel = 0.2e-3;

            double lSheet = MU0 * hDiel;

            const SHAPE_LINE_CHAIN& outline = fills->Outline( 0 );

            ZONE_STAR_MODEL starModel =
                    ZONE_IMPEDANCE::Build( outline, connPts, rSheet, lSheet );

            wxString zcNode = wxString::Format( wxS( "%szc%d" ), netPrefix, elemIdx );
            elemIdx++;

            for( size_t si = 0; si < starModel.spokes.size(); ++si )
            {
                const ZONE_SPOKE& spoke = starModel.spokes[si];
                wxString          connNode = connNodes[spoke.connectionIndex];

                wxString rName = wxString::Format( wxS( "%sRzs%d_%d" ), netPrefix, elemIdx,
                                                   static_cast<int>( si ) );
                wxString lName = wxString::Format( wxS( "%sLzs%d_%d" ), netPrefix, elemIdx,
                                                   static_cast<int>( si ) );
                wxString mid = wxString::Format( wxS( "%szsm%d_%d" ), netPrefix, elemIdx,
                                                 static_cast<int>( si ) );

                result.elements.push_back(
                        { wxS( "R" ), rName, connNode, mid, spoke.resistance } );
                result.elements.push_back(
                        { wxS( "L" ), lName, mid, zcNode, spoke.inductance } );
            }

            result.zoneCenters.push_back(
                    { layer, zcNode, fills->Area() * NM_TO_M * NM_TO_M } );
        }
    }

    // ------------------------------------------------------------------
    // Phase 6: filter pad ports to user-selected refdes set
    // ------------------------------------------------------------------

    if( !aPdnRefdes.empty() )
    {
        std::vector<PDN_PAD_PORT> kept;
        kept.reserve( result.padPorts.size() );

        for( const PDN_PAD_PORT& p : result.padPorts )
        {
            if( aPdnRefdes.count( p.refdes ) )
                kept.push_back( p );
        }

        result.padPorts = std::move( kept );
    }

    // ------------------------------------------------------------------
    // Phase 7: series coalescing — collapse degree-2 internal nodes
    // ------------------------------------------------------------------

    coalesceSeries( result );

    return result;
}


void PDN_LAYOUT_EXTRACTOR::coalesceSeries( PDN_NET_PARASITICS& aResult )
{
    // Nodes that must survive: anything externally observable.  Zone-center
    // nodes are referenced by plane caps (even though findPlaneCaps hasn't
    // run yet — it will reference by the exact names we record here).
    std::set<wxString> terminalNodes;

    for( const PDN_PAD_PORT& p : aResult.padPorts )
        terminalNodes.insert( p.nodeName );

    for( const PDN_ZONE_CENTER& zc : aResult.zoneCenters )
        terminalNodes.insert( zc.nodeName );

    std::vector<PDN_SPICE_ELEMENT>& elems = aResult.elements;

    // Drop trivially-small elements up front.
    elems.erase( std::remove_if( elems.begin(), elems.end(),
                                 []( const PDN_SPICE_ELEMENT& aE )
                                 {
                                     if( aE.type == wxS( "R" ) )
                                         return aE.value < R_DROP_THRESHOLD;
                                     if( aE.type == wxS( "L" ) )
                                         return aE.value < L_DROP_THRESHOLD;
                                     return false;
                                 } ),
                 elems.end() );

    bool changed = true;

    while( changed )
    {
        changed = false;

        // Build adjacency: node → list of element indices
        std::map<wxString, std::vector<int>> adj;

        for( int i = 0; i < static_cast<int>( elems.size() ); ++i )
        {
            adj[elems[i].nodeA].push_back( i );
            adj[elems[i].nodeB].push_back( i );
        }

        std::vector<bool> dead( elems.size(), false );

        for( const auto& [node, incident] : adj )
        {
            if( terminalNodes.count( node ) )
                continue;

            if( incident.size() != 2 )
                continue;

            int i1 = incident[0];
            int i2 = incident[1];

            if( dead[i1] || dead[i2] )
                continue;

            PDN_SPICE_ELEMENT& e1 = elems[i1];
            PDN_SPICE_ELEMENT& e2 = elems[i2];

            if( e1.type != e2.type )
                continue;   // R+L chain — keep the mid-node

            // Sum in series, retire the mid-node.
            wxString farA = ( e1.nodeA == node ) ? e1.nodeB : e1.nodeA;
            wxString farB = ( e2.nodeA == node ) ? e2.nodeB : e2.nodeA;

            if( farA == farB )
                continue;   // would create a self-loop; safer to leave alone

            e1.value = e1.value + e2.value;
            e1.nodeA = farA;
            e1.nodeB = farB;
            dead[i2] = true;
            changed = true;
        }

        if( changed )
        {
            std::vector<PDN_SPICE_ELEMENT> kept;
            kept.reserve( elems.size() );

            for( size_t i = 0; i < elems.size(); ++i )
            {
                if( !dead[i] )
                    kept.push_back( std::move( elems[i] ) );
            }

            elems = std::move( kept );
        }
    }
}


void PDN_LAYOUT_EXTRACTOR::findPlaneCaps()
{
    if( m_result.powerNet.zoneCenters.empty() || m_result.groundNet.zoneCenters.empty() )
        return;

    STACKUP_READER stackup( m_board );

    LSEQ                        cuLayers = m_board->GetEnabledLayers().CuStack();
    std::map<PCB_LAYER_ID, int> layerIdx;

    for( size_t i = 0; i < cuLayers.size(); ++i )
        layerIdx[cuLayers[i]] = static_cast<int>( i );

    int capIdx = 1;

    for( const PDN_ZONE_CENTER& pCenter : m_result.powerNet.zoneCenters )
    {
        auto pIt = layerIdx.find( pCenter.layer );

        if( pIt == layerIdx.end() )
            continue;

        for( const PDN_ZONE_CENTER& gCenter : m_result.groundNet.zoneCenters )
        {
            auto gIt = layerIdx.find( gCenter.layer );

            if( gIt == layerIdx.end() )
                continue;

            if( std::abs( pIt->second - gIt->second ) != 1 )
                continue;

            double overlapArea = std::min( pCenter.area, gCenter.area );

            if( overlapArea <= 0.0 )
                continue;

            LAYER_GEOMETRY layerGeom =
                    stackup.GetLayerGeometry( pCenter.layer, VECTOR2I( 0, 0 ), 1000000 );

            double h = ( pIt->second < gIt->second ) ? layerGeom.hBelow : layerGeom.hAbove;
            double er = ( pIt->second < gIt->second ) ? layerGeom.erBelow : layerGeom.erAbove;

            if( h <= 0.0 )
                h = 0.1e-3;

            double capValue = EPS0 * er * overlapArea / h;

            if( capValue <= 0.0 )
                continue;

            wxString capName = wxString::Format( wxS( "Cplane%d" ), capIdx++ );

            m_result.planeCaps.push_back(
                    { wxS( "C" ), capName, pCenter.nodeName, gCenter.nodeName, capValue } );
        }
    }
}


wxString PDN_LAYOUT_EXTRACTOR::SerializeResult() const
{
    wxString payload;

    // Power net
    payload += wxString::Format( wxS( "NET %s\n" ), m_result.powerNet.netName );

    for( const PDN_PAD_PORT& port : m_result.powerNet.padPorts )
    {
        payload += wxString::Format( wxS( "PAD %s:%s node=%s pos=%d,%d\n" ),
                                     port.refdes, port.padNumber, port.nodeName,
                                     port.position.x, port.position.y );
    }

    for( const PDN_SPICE_ELEMENT& elem : m_result.powerNet.elements )
    {
        payload += wxString::Format( wxS( "%s %s %s %s %g\n" ),
                                     elem.type, elem.name, elem.nodeA, elem.nodeB, elem.value );
    }

    for( const wxString& warn : m_result.powerNet.warnings )
        payload += wxString::Format( wxS( "WARN %s\n" ), warn );

    // Ground net
    payload += wxString::Format( wxS( "NET %s\n" ), m_result.groundNet.netName );

    for( const PDN_PAD_PORT& port : m_result.groundNet.padPorts )
    {
        payload += wxString::Format( wxS( "PAD %s:%s node=%s pos=%d,%d\n" ),
                                     port.refdes, port.padNumber, port.nodeName,
                                     port.position.x, port.position.y );
    }

    for( const PDN_SPICE_ELEMENT& elem : m_result.groundNet.elements )
    {
        payload += wxString::Format( wxS( "%s %s %s %s %g\n" ),
                                     elem.type, elem.name, elem.nodeA, elem.nodeB, elem.value );
    }

    for( const wxString& warn : m_result.groundNet.warnings )
        payload += wxString::Format( wxS( "WARN %s\n" ), warn );

    // Plane capacitances
    for( const PDN_SPICE_ELEMENT& elem : m_result.planeCaps )
    {
        payload += wxString::Format( wxS( "PLANE_C %s %s %s %g\n" ),
                                     elem.name, elem.nodeA, elem.nodeB, elem.value );
    }

    return payload;
}


bool PDN_LAYOUT_EXTRACTOR::DeserializeResult( const wxString& aPayload,
                                               PDN_EXTRACTION_RESULT& aResult )
{
    aResult = PDN_EXTRACTION_RESULT();

    PDN_NET_PARASITICS* currentNet = nullptr;
    wxArrayString lines = wxStringTokenize( aPayload, wxS( "\n" ) );

    for( const wxString& line : lines )
    {
        if( line.IsEmpty() )
            continue;

        if( line.StartsWith( wxS( "NET " ) ) )
        {
            wxString netName = line.Mid( 4 ).Trim();

            if( !currentNet )
            {
                aResult.powerNet.netName = netName;
                currentNet = &aResult.powerNet;
            }
            else
            {
                aResult.groundNet.netName = netName;
                currentNet = &aResult.groundNet;
            }

            continue;
        }

        if( line.StartsWith( wxS( "PAD " ) ) && currentNet )
        {
            wxString rest = line.Mid( 4 );
            wxString refPad = rest.BeforeFirst( ' ' );
            wxString attrs = rest.AfterFirst( ' ' );

            PDN_PAD_PORT port;
            port.refdes = refPad.BeforeFirst( ':' );
            port.padNumber = refPad.AfterFirst( ':' );

            wxString nodeAttr = attrs.BeforeFirst( ' ' );

            if( nodeAttr.StartsWith( wxS( "node=" ) ) )
                port.nodeName = nodeAttr.Mid( 5 );

            wxString posAttr = attrs.AfterFirst( ' ' );

            if( posAttr.StartsWith( wxS( "pos=" ) ) )
            {
                wxString posStr = posAttr.Mid( 4 );
                long x = 0, y = 0;
                posStr.BeforeFirst( ',' ).ToLong( &x );
                posStr.AfterFirst( ',' ).ToLong( &y );
                port.position = VECTOR2I( x, y );
            }

            currentNet->padPorts.push_back( port );
            continue;
        }

        if( line.StartsWith( wxS( "WARN " ) ) && currentNet )
        {
            currentNet->warnings.push_back( line.Mid( 5 ) );
            continue;
        }

        if( line.StartsWith( wxS( "PLANE_C " ) ) )
        {
            wxArrayString tokens = wxStringTokenize( line, wxS( " " ) );

            if( tokens.size() >= 5 )
            {
                PDN_SPICE_ELEMENT elem;
                elem.type = wxS( "C" );
                elem.name = tokens[1];
                elem.nodeA = tokens[2];
                elem.nodeB = tokens[3];
                tokens[4].ToDouble( &elem.value );
                aResult.planeCaps.push_back( elem );
            }

            continue;
        }

        if( currentNet && ( line.StartsWith( wxS( "R " ) ) || line.StartsWith( wxS( "L " ) )
                            || line.StartsWith( wxS( "C " ) ) ) )
        {
            wxArrayString tokens = wxStringTokenize( line, wxS( " " ) );

            if( tokens.size() >= 5 )
            {
                PDN_SPICE_ELEMENT elem;
                elem.type = tokens[0];
                elem.name = tokens[1];
                elem.nodeA = tokens[2];
                elem.nodeB = tokens[3];
                tokens[4].ToDouble( &elem.value );
                currentNet->elements.push_back( elem );
            }
        }
    }

    return !aResult.powerNet.netName.IsEmpty() || !aResult.groundNet.netName.IsEmpty();
}
