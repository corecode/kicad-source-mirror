#!/usr/bin/env python3
"""
MLCC Parasitic Statistics Analyzer

Analyzes the SQLite database produced by collect_mlcc_data.py to extract
statistical summaries of ESR, ESL, and SRF across case sizes, dielectrics,
and capacitance decades.  The goal is to derive sensible default parasitics
for a PDN impedance analyzer when no specific SPICE model is available.

Usage:
    python3 analyze_mlcc_data.py [--db murata_mlcc.db] [--csv output_dir]
"""

import argparse
import csv
import math
import os
import re
import sqlite3
import sys
from collections import defaultdict

# Import decoder from the collector script (same directory)
sys.path.insert( 0, os.path.dirname( __file__ ) )
from collect_mlcc_data import _MURATA_PN_RE, DIELECTRIC_CODES


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def percentile( sorted_vals, p ):
    """Return the p-th percentile (0–100) from an already-sorted list."""
    if not sorted_vals:
        return float( "nan" )
    k = ( len( sorted_vals ) - 1 ) * p / 100.0
    f = int( k )
    c = f + 1
    if c >= len( sorted_vals ):
        return sorted_vals[f]
    return sorted_vals[f] + ( k - f ) * ( sorted_vals[c] - sorted_vals[f] )


def eng( val, unit="" ):
    """Format a value with engineering prefix."""
    if val is None or math.isnan( val ):
        return "N/A"
    prefixes = [
        ( 1e12,  "T" ), ( 1e9,  "G" ), ( 1e6, "M" ), ( 1e3, "k" ),
        ( 1,     ""  ), ( 1e-3, "m" ), ( 1e-6, "u" ), ( 1e-9, "n" ),
        ( 1e-12, "p" ), ( 1e-15, "f" ),
    ]
    for scale, prefix in prefixes:
        if abs( val ) >= scale * 0.999:
            return "{:.3g} {}{}".format( val / scale, prefix, unit )
    return "{:.3g} {}".format( val, unit )


def cap_decade_label( cap_F ):
    """Return a human-readable decade label for a capacitance value."""
    if cap_F is None or cap_F <= 0:
        return None
    exp = math.floor( math.log10( cap_F ) )
    labels = {
        -12: "1 pF",   -11: "10 pF",  -10: "100 pF",
        -9:  "1 nF",   -8:  "10 nF",  -7:  "100 nF",
        -6:  "1 uF",   -5:  "10 uF",  -4:  "100 uF",
    }
    return labels.get( exp, "1e{} F".format( exp ) )


def cap_decade_key( cap_F ):
    """Return the log10 exponent for grouping into decades."""
    if cap_F is None or cap_F <= 0:
        return None
    return math.floor( math.log10( cap_F ) )


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_data( db_path ):
    """Load joined parts + parasitics into a list of dicts."""
    conn = sqlite3.connect( db_path )
    conn.row_factory = sqlite3.Row
    rows = conn.execute( """
        SELECT p.part_number, p.series, p.case_size_eia, p.case_size_metric,
               p.dielectric, p.capacitance_F, p.voltage_V, p.tolerance,
               pa.esr_ohm, pa.esl_H, pa.srf_Hz, pa.c_effective_F,
               pa.q_at_srf, pa.z_at_1mhz, pa.z_at_10mhz, pa.z_at_100mhz
        FROM parts p
        JOIN parasitics pa USING( part_number )
    """ ).fetchall()
    conn.close()
    return [ dict( r ) for r in rows ]


# ---------------------------------------------------------------------------
# Statistical grouping
# ---------------------------------------------------------------------------

def compute_stats( values ):
    """Compute statistics for a list of numeric values (NaN/None filtered)."""
    clean = sorted( v for v in values if v is not None and not math.isnan( v ) and v > 0 )
    if not clean:
        return None
    mean = sum( clean ) / len( clean )
    p25 = percentile( clean, 25 )
    p50 = percentile( clean, 50 )
    p75 = percentile( clean, 75 )

    # Geometric standard deviation (multiplicative spread) — useful because
    # parasitics span orders of magnitude.  GSD = exp(std(ln(x))).
    # A GSD of 2.0 means the ±1σ range is [median/2, median*2].
    log_vals = [ math.log( v ) for v in clean ]
    log_mean = sum( log_vals ) / len( log_vals )
    if len( clean ) > 1:
        log_var = sum( ( lv - log_mean )**2 for lv in log_vals ) / ( len( log_vals ) - 1 )
        gsd = math.exp( math.sqrt( log_var ) )
    else:
        gsd = 1.0

    # IQR ratio: p75/p25 — intuitive multiplicative spread of the middle 50%
    iqr_ratio = p75 / p25 if p25 > 0 else float( "inf" )

    return {
        "n":         len( clean ),
        "min":       clean[0],
        "p10":       percentile( clean, 10 ),
        "p25":       p25,
        "p50":       p50,
        "mean":      mean,
        "p75":       p75,
        "p90":       percentile( clean, 90 ),
        "max":       clean[-1],
        "gsd":       gsd,
        "iqr_ratio": iqr_ratio,
    }


def group_by( data, key_func ):
    """Group data rows by key_func, returning dict of key → [rows]."""
    groups = defaultdict( list )
    for row in data:
        k = key_func( row )
        if k is not None:
            groups[k].append( row )
    return dict( groups )


# ---------------------------------------------------------------------------
# Analysis sections
# ---------------------------------------------------------------------------

def print_header( title ):
    print()
    print( "=" * 78 )
    print( "  " + title )
    print( "=" * 78 )


def print_stat_table( headers, rows, col_widths=None ):
    """Print a formatted table."""
    if col_widths is None:
        col_widths = []
        for i, h in enumerate( headers ):
            w = len( h )
            for r in rows:
                w = max( w, len( str( r[i] ) ) )
            col_widths.append( w + 1 )

    fmt = "  ".join( "{:<" + str( w ) + "}" for w in col_widths )
    print( fmt.format( *headers ) )
    print( fmt.format( *( "-" * w for w in col_widths ) ) )
    for r in rows:
        print( fmt.format( *r ) )


def analyze_overview( data ):
    """Print high-level overview."""
    print_header( "DATABASE OVERVIEW" )

    case_sizes = set()
    dielectrics = set()
    series = set()
    for r in data:
        if r["case_size_eia"]:
            case_sizes.add( r["case_size_eia"] )
        if r["dielectric"]:
            dielectrics.add( r["dielectric"] )
        if r["series"]:
            series.add( r["series"] )

    print( "  Total parts with parasitics: {:,}".format( len( data ) ) )
    print( "  Case sizes:    {} ({})".format(
        len( case_sizes ), ", ".join( sorted( case_sizes ) ) ) )
    print( "  Dielectrics:   {} ({})".format(
        len( dielectrics ), ", ".join( sorted( dielectrics ) ) ) )
    print( "  Series:        {} ({})".format(
        len( series ), ", ".join( sorted( series ) ) ) )

    # Parts with decoded metadata
    decoded = sum( 1 for r in data if r["case_size_eia"] and r["dielectric"] )
    print( "  Fully decoded: {:,} ({:.1f}%)".format(
        decoded, 100.0 * decoded / len( data ) if data else 0 ) )


def analyze_esl_by_case( data ):
    """ESL statistics grouped by case size."""
    print_header( "ESL BY CASE SIZE" )

    groups = group_by( data, lambda r: r["case_size_eia"] )
    headers = [ "Case", "N", "P25", "Median", "P75", "P75/P25", "GSD" ]
    rows = []

    # Sort by median ESL ascending
    sorted_keys = sorted( groups.keys(),
        key=lambda k: percentile(
            sorted( r["esl_H"] for r in groups[k]
                    if r["esl_H"] and r["esl_H"] > 0 ), 50 ) )

    for key in sorted_keys:
        stats = compute_stats( [ r["esl_H"] for r in groups[key] ] )
        if stats:
            rows.append( (
                key, stats["n"],
                eng( stats["p25"], "H" ),
                eng( stats["p50"], "H" ),
                eng( stats["p75"], "H" ),
                "{:.2f}x".format( stats["iqr_ratio"] ),
                "{:.2f}x".format( stats["gsd"] ),
            ) )

    print_stat_table( headers, rows )


def analyze_esr_by_case( data ):
    """ESR statistics grouped by case size."""
    print_header( "ESR BY CASE SIZE" )

    groups = group_by( data, lambda r: r["case_size_eia"] )
    headers = [ "Case", "N", "P25", "Median", "P75", "P75/P25", "GSD" ]
    rows = []

    sorted_keys = sorted( groups.keys(),
        key=lambda k: percentile(
            sorted( r["esr_ohm"] for r in groups[k]
                    if r["esr_ohm"] and r["esr_ohm"] > 0 ), 50 ) )

    for key in sorted_keys:
        stats = compute_stats( [ r["esr_ohm"] for r in groups[key] ] )
        if stats:
            rows.append( (
                key, stats["n"],
                eng( stats["p25"], "ohm" ),
                eng( stats["p50"], "ohm" ),
                eng( stats["p75"], "ohm" ),
                "{:.1f}x".format( stats["iqr_ratio"] ),
                "{:.1f}x".format( stats["gsd"] ),
            ) )

    print_stat_table( headers, rows )


def _fmt_cross_cell( stats, scale, fmt_str, unit="" ):
    """Format a cross-tab cell: median [P25–P75] (n=N)."""
    if not stats:
        return "-"
    p25 = fmt_str.format( stats["p25"] * scale )
    med = fmt_str.format( stats["p50"] * scale )
    p75 = fmt_str.format( stats["p75"] * scale )
    if stats["n"] < 3:
        return "({}) n={}".format( med, stats["n"] )
    return "{} [{}-{}] n={}".format( med, p25, p75, stats["n"] )


def analyze_esl_by_case_dielectric( data ):
    """ESL cross-tabulated by case size and dielectric."""
    print_header( "ESL (nH: MEDIAN [P25-P75]) BY CASE SIZE x DIELECTRIC" )

    groups = group_by( data, lambda r: (
        r["case_size_eia"], r["dielectric"] )
        if r["case_size_eia"] and r["dielectric"] else None )

    case_sizes = sorted( set( k[0] for k in groups.keys() ) )
    dielectrics = sorted( set( k[1] for k in groups.keys() ) )

    headers = [ "Case" ] + dielectrics
    rows = []

    for cs in case_sizes:
        row = [ cs ]
        for di in dielectrics:
            key = ( cs, di )
            stats = compute_stats( [ r["esl_H"] for r in groups[key] ] ) \
                if key in groups else None
            row.append( _fmt_cross_cell( stats, 1e9, "{:.3f}" ) )
        rows.append( tuple( row ) )

    print_stat_table( headers, rows )


def analyze_esr_by_case_dielectric( data ):
    """ESR cross-tabulated by case size and dielectric."""
    print_header( "ESR (mohm: MEDIAN [P25-P75]) BY CASE SIZE x DIELECTRIC" )

    groups = group_by( data, lambda r: (
        r["case_size_eia"], r["dielectric"] )
        if r["case_size_eia"] and r["dielectric"] else None )

    case_sizes = sorted( set( k[0] for k in groups.keys() ) )
    dielectrics = sorted( set( k[1] for k in groups.keys() ) )

    headers = [ "Case" ] + dielectrics
    rows = []

    for cs in case_sizes:
        row = [ cs ]
        for di in dielectrics:
            key = ( cs, di )
            stats = compute_stats( [ r["esr_ohm"] for r in groups[key] ] ) \
                if key in groups else None
            row.append( _fmt_cross_cell( stats, 1000, "{:.1f}" ) )
        rows.append( tuple( row ) )

    print_stat_table( headers, rows )


def analyze_esr_by_capacitance( data ):
    """ESR vs capacitance decade — shows the strong ESR-capacitance correlation."""
    print_header( "ESR BY CAPACITANCE DECADE" )

    groups = group_by( data, lambda r: cap_decade_label( r["capacitance_F"] ) )
    headers = [ "Decade", "N", "P25", "Median", "P75", "P75/P25", "GSD" ]
    rows = []

    sorted_keys = sorted( groups.keys(),
        key=lambda k: min( r["capacitance_F"] for r in groups[k]
                           if r["capacitance_F"] ) )

    for key in sorted_keys:
        stats = compute_stats( [ r["esr_ohm"] for r in groups[key] ] )
        if stats:
            rows.append( (
                key, stats["n"],
                eng( stats["p25"], "ohm" ),
                eng( stats["p50"], "ohm" ),
                eng( stats["p75"], "ohm" ),
                "{:.1f}x".format( stats["iqr_ratio"] ),
                "{:.1f}x".format( stats["gsd"] ),
            ) )

    print_stat_table( headers, rows )


def analyze_esl_by_capacitance( data ):
    """ESL vs capacitance decade — check if ESL varies with capacitance."""
    print_header( "ESL BY CAPACITANCE DECADE" )

    groups = group_by( data, lambda r: cap_decade_label( r["capacitance_F"] ) )
    headers = [ "Decade", "N", "P25", "Median", "P75", "P75/P25", "GSD" ]
    rows = []

    sorted_keys = sorted( groups.keys(),
        key=lambda k: min( r["capacitance_F"] for r in groups[k]
                           if r["capacitance_F"] ) )

    for key in sorted_keys:
        stats = compute_stats( [ r["esl_H"] for r in groups[key] ] )
        if stats:
            rows.append( (
                key, stats["n"],
                eng( stats["p25"], "H" ),
                eng( stats["p50"], "H" ),
                eng( stats["p75"], "H" ),
                "{:.2f}x".format( stats["iqr_ratio"] ),
                "{:.2f}x".format( stats["gsd"] ),
            ) )

    print_stat_table( headers, rows )


def analyze_esr_by_cap_and_case( data ):
    """ESR (median) cross-tabulated by capacitance decade and case size.

    This is the most useful view for default parasitics: ESR depends strongly
    on both capacitance (thinner dielectric → lower ESR at high C) and package
    (larger packages → lower ESR due to wider terminals).
    """
    print_header( "ESR (mohm: MEDIAN [P25-P75]) BY CAP DECADE x CASE SIZE" )

    groups = group_by( data, lambda r: (
        cap_decade_label( r["capacitance_F"] ), r["case_size_eia"] )
        if r["case_size_eia"] and r["capacitance_F"] else None )

    cap_labels = sorted( set( k[0] for k in groups.keys() ),
        key=lambda l: min( r["capacitance_F"] for r in groups.get(
            ( l, next( k[1] for k in groups if k[0] == l ) ), [{"capacitance_F": 0}] )
            if r["capacitance_F"] ) )
    case_sizes = sorted( set( k[1] for k in groups.keys() ) )

    headers = [ "Cap \\ Case" ] + case_sizes
    rows = []

    for cl in cap_labels:
        row = [ cl ]
        for cs in case_sizes:
            key = ( cl, cs )
            stats = compute_stats( [ r["esr_ohm"] for r in groups[key] ] ) \
                if key in groups else None
            row.append( _fmt_cross_cell( stats, 1000, "{:.1f}" ) )
        rows.append( tuple( row ) )

    print_stat_table( headers, rows )


def analyze_srf( data ):
    """SRF statistics by case size."""
    print_header( "SRF BY CASE SIZE" )

    groups = group_by( data, lambda r: r["case_size_eia"] )
    headers = [ "Case", "N", "P25", "Median", "P75", "P75/P25", "GSD" ]
    rows = []

    sorted_keys = sorted( groups.keys() )

    for key in sorted_keys:
        stats = compute_stats( [ r["srf_Hz"] for r in groups[key] ] )
        if stats:
            rows.append( (
                key, stats["n"],
                eng( stats["p25"], "Hz" ),
                eng( stats["p50"], "Hz" ),
                eng( stats["p75"], "Hz" ),
                "{:.1f}x".format( stats["iqr_ratio"] ),
                "{:.1f}x".format( stats["gsd"] ),
            ) )

    print_stat_table( headers, rows )


def analyze_srf_by_cap( data ):
    """SRF vs capacitance — SRF = 1/(2*pi*sqrt(L*C)), so it should drop
    with sqrt(C) for constant L."""
    print_header( "SRF BY CAPACITANCE DECADE" )

    groups = group_by( data, lambda r: cap_decade_label( r["capacitance_F"] ) )
    headers = [ "Decade", "N", "P25", "Median", "P75", "P75/P25", "GSD" ]
    rows = []

    sorted_keys = sorted( groups.keys(),
        key=lambda k: min( r["capacitance_F"] for r in groups[k]
                           if r["capacitance_F"] ) )

    for key in sorted_keys:
        stats = compute_stats( [ r["srf_Hz"] for r in groups[key] ] )
        if stats:
            rows.append( (
                key, stats["n"],
                eng( stats["p25"], "Hz" ),
                eng( stats["p50"], "Hz" ),
                eng( stats["p75"], "Hz" ),
                "{:.1f}x".format( stats["iqr_ratio"] ),
                "{:.1f}x".format( stats["gsd"] ),
            ) )

    print_stat_table( headers, rows )


def analyze_c0g_vs_class2( data ):
    """Compare C0G (Class I) vs Class II dielectrics (X5R/X7R/X8R)."""
    print_header( "C0G vs CLASS II DIELECTRICS" )

    # We don't have C0G explicitly decoded, but parts without a decoded
    # dielectric might be C0G.  Check what we have.
    diel_counts = defaultdict( int )
    for r in data:
        diel_counts[r["dielectric"] or "(unknown)"] += 1

    print( "  Dielectric distribution:" )
    for d, n in sorted( diel_counts.items(), key=lambda x: -x[1] ):
        print( "    {}: {:,}".format( d, n ) )

    # Compare known dielectrics
    for diel in [ "X5R", "X7R", "X8R" ]:
        subset = [ r for r in data if r["dielectric"] == diel ]
        if not subset:
            continue
        esr = compute_stats( [ r["esr_ohm"] for r in subset ] )
        esl = compute_stats( [ r["esl_H"] for r in subset ] )
        print( "\n  {} (n={}):".format( diel, len( subset ) ) )
        if esr:
            print( "    ESR median: {}  (P25–P75: {} – {})".format(
                eng( esr["p50"], "ohm" ),
                eng( esr["p25"], "ohm" ), eng( esr["p75"], "ohm" ) ) )
        if esl:
            print( "    ESL median: {}  (P25–P75: {} – {})".format(
                eng( esl["p50"], "H" ),
                eng( esl["p25"], "H" ), eng( esl["p75"], "H" ) ) )


# ---------------------------------------------------------------------------
# Decode coverage analysis
# ---------------------------------------------------------------------------

def analyze_decode_coverage( data ):
    """Analyze which part numbers fail to decode and why.

    The collector's regex-based decoder misses some dielectric codes.
    This identifies the gaps so we can improve coverage or understand
    what fraction of parts are truly usable.
    """
    print_header( "PART NUMBER DECODE COVERAGE" )

    total = len( data )
    has_case = sum( 1 for r in data if r["case_size_eia"] )
    has_diel = sum( 1 for r in data if r["dielectric"] )
    has_cap  = sum( 1 for r in data if r["capacitance_F"] )
    has_volt = sum( 1 for r in data if r["voltage_V"] )
    full     = sum( 1 for r in data
                    if r["case_size_eia"] and r["dielectric"]
                    and r["capacitance_F"] and r["voltage_V"] )

    pct = lambda n: "{:.1f}%".format( 100.0 * n / total ) if total else "N/A"
    print( "  Total parts:      {:>8,}".format( total ) )
    print( "  Has case size:    {:>8,}  ({})".format( has_case, pct( has_case ) ) )
    print( "  Has dielectric:   {:>8,}  ({})".format( has_diel, pct( has_diel ) ) )
    print( "  Has capacitance:  {:>8,}  ({})".format( has_cap, pct( has_cap ) ) )
    print( "  Has voltage:      {:>8,}  ({})".format( has_volt, pct( has_volt ) ) )
    print( "  Fully decoded:    {:>8,}  ({})".format( full, pct( full ) ) )

    # Analyze unmatched dielectric codes from part numbers
    unknown_diel_codes = defaultdict( int )
    unknown_pns = []
    for r in data:
        if r["dielectric"]:
            continue
        pn = r["part_number"]
        m = _MURATA_PN_RE.match( pn )
        if m:
            diel_code = m.group( "diel" ).upper()
            unknown_diel_codes[diel_code] += 1
        else:
            unknown_pns.append( pn )

    if unknown_diel_codes:
        print( "\n  Unrecognized dielectric codes (from regex match):" )
        print( "  {:>6s}  {:>6s}  {}".format( "Code", "Count", "Known mapping" ) )
        print( "  {:>6s}  {:>6s}  {}".format( "------", "------", "-------------" ) )

        # Known Murata codes not in the collector's DIELECTRIC_CODES dict
        extra_known = {
            "C1": "C0G/NP0",  "C2": "C0G/NP0",  "C3": "C0G/NP0",
            "C4": "C0G/NP0",  "C5": "C0G/NP0",
            "J1": "C0G/NP0",  "J2": "C0G/NP0",
            "U2": "U2J",      "U3": "U2J",
            "R1": "X5R",      "R2": "X5R",
            "B1": "X7R",      "B2": "X7R",
            "E1": "X7R",      "E2": "X7R",
            "D1": "X7R",      "D2": "X7R",
            "S1": "X7R",      "S3": "X7R",
        }

        for code, cnt in sorted( unknown_diel_codes.items(), key=lambda x: -x[1] ):
            known = extra_known.get( code, DIELECTRIC_CODES.get( code, "?" ) )
            in_collector = "(in collector)" if code in DIELECTRIC_CODES else ""
            print( "  {:>6s}  {:>6,}  {} {}".format(
                code, cnt, known, in_collector ) )

    if unknown_pns:
        print( "\n  Part numbers not matching Murata regex: {:,}".format(
            len( unknown_pns ) ) )
        for pn in unknown_pns[:10]:
            print( "    {}".format( pn ) )
        if len( unknown_pns ) > 10:
            print( "    ... and {:,} more".format( len( unknown_pns ) - 10 ) )

    # Show parasitics for undecoded parts (are they still useful?)
    undecoded = [ r for r in data if not r["dielectric"] ]
    if undecoded:
        esr = compute_stats( [ r["esr_ohm"] for r in undecoded ] )
        esl = compute_stats( [ r["esl_H"] for r in undecoded ] )
        print( "\n  Parasitics of {:,} undecoded-dielectric parts:".format(
            len( undecoded ) ) )
        if esr:
            print( "    ESR median: {}  (P25–P75: {} – {})".format(
                eng( esr["p50"], "ohm" ),
                eng( esr["p25"], "ohm" ), eng( esr["p75"], "ohm" ) ) )
        if esl:
            print( "    ESL median: {}  (P25–P75: {} – {})".format(
                eng( esl["p50"], "H" ),
                eng( esl["p25"], "H" ), eng( esl["p75"], "H" ) ) )


# ---------------------------------------------------------------------------
# ESR heuristic fitting
# ---------------------------------------------------------------------------

CLASS_II_DIELECTRICS = { "X5R", "X7R", "X7S", "X8R", "X8L", "X8M" }


def _linreg( xs, ys ):
    """Simple linear regression. Returns (slope, intercept, r_squared)."""
    n = len( xs )
    if n < 3:
        return None, None, None
    sx = sum( xs )
    sy = sum( ys )
    sxx = sum( x * x for x in xs )
    sxy = sum( x * y for x, y in zip( xs, ys ) )
    syy = sum( y * y for y in ys )

    denom = n * sxx - sx * sx
    if denom == 0:
        return None, None, None

    slope = ( n * sxy - sx * sy ) / denom
    intercept = ( sy - slope * sx ) / n

    ss_res = sum( ( y - ( slope * x + intercept ) )**2 for x, y in zip( xs, ys ) )
    ss_tot = syy - sy * sy / n
    r_sq = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0

    return slope, intercept, r_sq


def analyze_esr_heuristic( data ):
    """Fit ESR = k * C^alpha for Class II ceramics, optionally per case size.

    Physics motivation:
    - Higher C → more dielectric layers in parallel → lower ESR
    - ESR ∝ C^alpha where alpha is typically -0.4 to -0.7
    - Case size affects ESR through terminal geometry (wider = lower R)
    """
    print_header( "ESR HEURISTIC FITTING (Class II ceramics only)" )

    # Filter to Class II with valid ESR and capacitance
    class2 = [ r for r in data
                if r["dielectric"] in CLASS_II_DIELECTRICS
                and r["esr_ohm"] and r["esr_ohm"] > 0
                and r["capacitance_F"] and r["capacitance_F"] > 0 ]

    if not class2:
        print( "  No Class II data available." )
        return

    print( "  Class II parts with valid ESR + capacitance: {:,}".format(
        len( class2 ) ) )

    # --- Global fit: log(ESR) = alpha * log(C) + log(k) ---
    log_c = [ math.log10( r["capacitance_F"] ) for r in class2 ]
    log_esr = [ math.log10( r["esr_ohm"] ) for r in class2 ]

    alpha, log_k, r_sq = _linreg( log_c, log_esr )
    if alpha is not None:
        k = 10**log_k
        print( "\n  Global fit (all Class II, all case sizes):" )
        print( "    ESR = {:.4e} * C^({:.3f})".format( k, alpha ) )
        print( "    R^2 = {:.4f}".format( r_sq ) )
        print( "    (C in Farads, ESR in Ohms)" )

        # Show what this gives at key capacitances
        print( "\n    Predicted ESR vs actual median:" )
        print( "    {:>8s}  {:>12s}  {:>12s}  {:>8s}".format(
            "Cap", "Predicted", "Actual P50", "Ratio" ) )
        print( "    {:>8s}  {:>12s}  {:>12s}  {:>8s}".format(
            "--------", "------------", "------------", "--------" ) )

        for decade in range( -9, -3 ):
            cap_val = 10.0**decade
            predicted = k * cap_val**alpha
            subset = [ r for r in class2
                        if decade <= math.log10( r["capacitance_F"] ) < decade + 1 ]
            if subset:
                actual = compute_stats( [ r["esr_ohm"] for r in subset ] )
                if actual:
                    ratio = predicted / actual["p50"]
                    print( "    {:>8s}  {:>12s}  {:>12s}  {:>7.2f}x".format(
                        cap_decade_label( cap_val ),
                        eng( predicted, "ohm" ),
                        eng( actual["p50"], "ohm" ),
                        ratio ) )

    # --- Per-case-size fit ---
    print( "\n  Per-case-size fits (ESR = k * C^alpha):" )
    print( "  {:>8s}  {:>6s}  {:>12s}  {:>8s}  {:>6s}".format(
        "Case", "N", "k", "alpha", "R^2" ) )
    print( "  {:>8s}  {:>6s}  {:>12s}  {:>8s}  {:>6s}".format(
        "--------", "------", "------------", "--------", "------" ) )

    case_fits = {}
    groups = group_by( class2, lambda r: r["case_size_eia"] )
    for cs in sorted( groups.keys() ):
        subset = groups[cs]
        if len( subset ) < 10:
            continue
        lc = [ math.log10( r["capacitance_F"] ) for r in subset ]
        le = [ math.log10( r["esr_ohm"] ) for r in subset ]
        a, lk, rsq = _linreg( lc, le )
        if a is not None:
            case_fits[cs] = ( 10**lk, a, rsq )
            print( "  {:>8s}  {:>6d}  {:>12.4e}  {:>8.3f}  {:>6.4f}".format(
                cs, len( subset ), 10**lk, a, rsq ) )

    # --- Check if alpha is consistent across case sizes ---
    if case_fits:
        alphas = sorted( v[1] for v in case_fits.values() )
        alpha_median = alphas[len( alphas ) // 2]
        alpha_p25 = percentile( alphas, 25 )
        alpha_p75 = percentile( alphas, 75 )
        print( "\n  Alpha across case sizes: median={:.3f}  "
               "P25={:.3f}  P75={:.3f}".format(
                   alpha_median, alpha_p25, alpha_p75 ) )

    # --- Two-parameter model: ESR = k_case * C^alpha_common ---
    # Use a common alpha (median across case sizes) and fit k per case size
    if case_fits:
        common_alpha = alphas[len( alphas ) // 2]
        print( "\n  Simplified model: ESR = k_case * C^({:.3f})".format(
            common_alpha ) )
        print( "  {:>8s}  {:>6s}  {:>12s}  {:>12s}  {:>12s}".format(
            "Case", "N", "k", "ESR@100nF", "ESR@10uF" ) )
        print( "  {:>8s}  {:>6s}  {:>12s}  {:>12s}  {:>12s}".format(
            "--------", "------", "------------", "------------", "------------" ) )

        k_by_case = {}
        for cs in sorted( groups.keys() ):
            subset = groups[cs]
            if len( subset ) < 10:
                continue
            # Fit k with fixed alpha: log(ESR) = alpha*log(C) + log(k)
            # → log(k) = mean(log(ESR) - alpha*log(C))
            log_k_vals = [ math.log10( r["esr_ohm"] ) - common_alpha * math.log10( r["capacitance_F"] )
                           for r in subset ]
            median_log_k = sorted( log_k_vals )[len( log_k_vals ) // 2]
            k_case = 10**median_log_k
            k_by_case[cs] = k_case

            esr_100nf = k_case * ( 100e-9 )**common_alpha
            esr_10uf  = k_case * ( 10e-6 )**common_alpha

            print( "  {:>8s}  {:>6d}  {:>12.4e}  {:>12s}  {:>12s}".format(
                cs, len( subset ), k_case,
                eng( esr_100nf, "ohm" ), eng( esr_10uf, "ohm" ) ) )

        # Validate: show residuals by decade
        print( "\n  Validation — predicted vs actual median (simplified model):" )
        print( "  {:>8s}  {:>8s}  {:>12s}  {:>12s}  {:>8s}  {:>6s}".format(
            "Case", "Decade", "Predicted", "Actual P50", "Ratio", "N" ) )
        print( "  {:>8s}  {:>8s}  {:>12s}  {:>12s}  {:>8s}  {:>6s}".format(
            "--------", "--------", "------------", "------------",
            "--------", "------" ) )

        for cs in sorted( k_by_case.keys() ):
            for decade in range( -9, -3 ):
                subset = [ r for r in groups[cs]
                           if decade <= math.log10( r["capacitance_F"] ) < decade + 1 ]
                if len( subset ) < 3:
                    continue
                cap_mid = 10**( decade + 0.5 )
                predicted = k_by_case[cs] * cap_mid**common_alpha
                actual = compute_stats( [ r["esr_ohm"] for r in subset ] )
                if actual:
                    ratio = predicted / actual["p50"]
                    print( "  {:>8s}  {:>8s}  {:>12s}  {:>12s}  {:>7.2f}x  {:>6d}".format(
                        cs, cap_decade_label( 10.0**decade ),
                        eng( predicted, "ohm" ), eng( actual["p50"], "ohm" ),
                        ratio, actual["n"] ) )

        # Output the final heuristic as C code
        print( "\n  --- C-style heuristic (for pdn_analyzer.cpp) ---" )
        print()
        print( "  // ESR heuristic for Class II MLCCs (X5R/X7R/X8R):" )
        print( "  //   ESR = k_case * pow( C, {:.3f} )".format( common_alpha ) )
        print( "  // where C is in Farads, ESR in Ohms." )
        print( "  // k_case by EIA case size:" )
        for cs, k_val in sorted( k_by_case.items() ):
            esr_ex = k_val * ( 100e-9 )**common_alpha
            print( '  // {{ "{}",  {:.4e} }},  // ESR@100nF ≈ {}'.format(
                cs, k_val, eng( esr_ex, "ohm" ) ) )


# ---------------------------------------------------------------------------
# Default parasitic recommendations
# ---------------------------------------------------------------------------

def recommend_defaults( data ):
    """Derive recommended default parasitics for the PDN analyzer.

    Strategy:
    - ESL depends primarily on case size (package geometry), not capacitance.
    - ESR depends on both case size and capacitance value.
    - For ESL: use median by case size.
    - For ESR: use median by (case size, capacitance decade).
    - Fall back to case-size-only median if the cross-tab has too few samples.
    """
    print_header( "RECOMMENDED DEFAULT PARASITICS FOR PDN ANALYZER" )

    # --- ESL by case size (primary driver) ---
    print( "\n  ESL defaults (by case size — primary driver):" )
    print( "  {:>8s}  {:>10s}  {:>10s}  {:>10s}".format(
        "Case", "Median", "P25", "P75" ) )
    print( "  {:>8s}  {:>10s}  {:>10s}  {:>10s}".format(
        "--------", "----------", "----------", "----------" ) )

    esl_by_case = {}
    groups = group_by( data, lambda r: r["case_size_eia"] )
    for cs in sorted( groups.keys() ):
        stats = compute_stats( [ r["esl_H"] for r in groups[cs] ] )
        if stats:
            esl_by_case[cs] = stats["p50"]
            print( "  {:>8s}  {:>10s}  {:>10s}  {:>10s}".format(
                cs,
                eng( stats["p50"], "H" ),
                eng( stats["p25"], "H" ),
                eng( stats["p75"], "H" ) ) )

    # --- ESR by (case size, cap decade) ---
    print( "\n  ESR defaults (by case size + capacitance decade):" )

    esr_cross = {}
    groups = group_by( data, lambda r: (
        r["case_size_eia"], cap_decade_key( r["capacitance_F"] ) )
        if r["case_size_eia"] and r["capacitance_F"] else None )

    for key in sorted( groups.keys() ):
        stats = compute_stats( [ r["esr_ohm"] for r in groups[key] ] )
        if stats and stats["n"] >= 3:
            esr_cross[key] = stats["p50"]

    # Also compute ESR by case size only as fallback
    esr_by_case = {}
    case_groups = group_by( data, lambda r: r["case_size_eia"] )
    for cs in sorted( case_groups.keys() ):
        stats = compute_stats( [ r["esr_ohm"] for r in case_groups[cs] ] )
        if stats:
            esr_by_case[cs] = stats["p50"]

    case_sizes = sorted( set( k[0] for k in esr_cross.keys() ) )
    cap_decades = sorted( set( k[1] for k in esr_cross.keys() ) )

    header_labels = [ cap_decade_label( 10**d ) or "1e{}".format( d )
                      for d in cap_decades ]
    print( "  {:>8s}  {}".format( "Case",
        "  ".join( "{:>12s}".format( l ) for l in header_labels ) ) )
    print( "  {:>8s}  {}".format( "--------",
        "  ".join( "{:>12s}".format( "------------" ) for _ in cap_decades ) ) )

    for cs in case_sizes:
        cells = []
        for cd in cap_decades:
            key = ( cs, cd )
            if key in esr_cross:
                cells.append( "{:>12s}".format( eng( esr_cross[key], "ohm" ) ) )
            else:
                cells.append( "{:>12s}".format( "-" ) )
        print( "  {:>8s}  {}".format( cs, "  ".join( cells ) ) )

    # --- Summary table for code integration ---
    print( "\n  --- C-style lookup table (for pdn_analyzer.cpp) ---" )
    print()
    print( "  // ESL defaults by EIA case size [H]" )
    print( "  // Source: Murata MLCC database, median of {:,} parts".format(
        len( data ) ) )

    esl_items = sorted( esl_by_case.items() )
    for cs, val in esl_items:
        print( '  // {{ "{}",  {:.3e} }},  // {}'.format( cs, val, eng( val, "H" ) ) )

    print()
    print( "  // ESR defaults by EIA case size [ohm] (fallback, all cap values)" )
    esr_items = sorted( esr_by_case.items() )
    for cs, val in esr_items:
        print( '  // {{ "{}",  {:.3e} }},  // {}'.format( cs, val, eng( val, "ohm" ) ) )


# ---------------------------------------------------------------------------
# CSV export
# ---------------------------------------------------------------------------

def export_csv( data, output_dir ):
    """Export key statistics as CSV files for external analysis."""
    os.makedirs( output_dir, exist_ok=True )

    # 1. Raw per-part summary
    path = os.path.join( output_dir, "parts_parasitics.csv" )
    fields = [ "part_number", "series", "case_size_eia", "dielectric",
               "capacitance_F", "voltage_V", "esr_ohm", "esl_H", "srf_Hz" ]
    with open( path, "w", newline="" ) as f:
        w = csv.DictWriter( f, fieldnames=fields, extrasaction="ignore" )
        w.writeheader()
        for r in data:
            w.writerow( r )
    print( "  Wrote {}".format( path ) )

    # 2. ESL by case size
    path = os.path.join( output_dir, "esl_by_case.csv" )
    groups = group_by( data, lambda r: r["case_size_eia"] )
    with open( path, "w", newline="" ) as f:
        w = csv.writer( f )
        w.writerow( [ "case_size_eia", "n", "min_H", "p25_H", "median_H",
                       "mean_H", "p75_H", "p90_H", "max_H" ] )
        for cs in sorted( groups.keys() ):
            s = compute_stats( [ r["esl_H"] for r in groups[cs] ] )
            if s:
                w.writerow( [ cs, s["n"], s["min"], s["p25"], s["p50"],
                              s["mean"], s["p75"], s["p90"], s["max"] ] )
    print( "  Wrote {}".format( path ) )

    # 3. ESR by case size x cap decade
    path = os.path.join( output_dir, "esr_by_case_cap.csv" )
    groups = group_by( data, lambda r: (
        r["case_size_eia"], cap_decade_label( r["capacitance_F"] ) )
        if r["case_size_eia"] and r["capacitance_F"] else None )
    with open( path, "w", newline="" ) as f:
        w = csv.writer( f )
        w.writerow( [ "case_size_eia", "cap_decade", "n", "min_ohm", "p25_ohm",
                       "median_ohm", "mean_ohm", "p75_ohm", "p90_ohm", "max_ohm" ] )
        for key in sorted( groups.keys() ):
            s = compute_stats( [ r["esr_ohm"] for r in groups[key] ] )
            if s:
                w.writerow( [ key[0], key[1], s["n"], s["min"], s["p25"],
                              s["p50"], s["mean"], s["p75"], s["p90"], s["max"] ] )
    print( "  Wrote {}".format( path ) )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Analyze MLCC parasitic data from SQLite database" )
    parser.add_argument( "--db", default=os.path.join(
                             os.path.dirname( __file__ ), "murata_mlcc.db" ),
                         help="Path to SQLite database "
                              "(default: murata_mlcc.db in script dir)" )
    parser.add_argument( "--csv", default=None, metavar="DIR",
                         help="Export CSV summary files to DIR" )
    args = parser.parse_args()

    if not os.path.exists( args.db ):
        print( "Error: database not found: {}".format( args.db ), file=sys.stderr )
        print( "Run collect_mlcc_data.py first.", file=sys.stderr )
        sys.exit( 1 )

    data = load_data( args.db )
    if not data:
        print( "Error: no data in database.", file=sys.stderr )
        sys.exit( 1 )

    analyze_overview( data )
    analyze_esl_by_case( data )
    analyze_esr_by_case( data )
    analyze_esl_by_case_dielectric( data )
    analyze_esr_by_case_dielectric( data )
    analyze_esr_by_capacitance( data )
    analyze_esl_by_capacitance( data )
    analyze_esr_by_cap_and_case( data )
    analyze_srf( data )
    analyze_srf_by_cap( data )
    analyze_c0g_vs_class2( data )
    analyze_decode_coverage( data )
    analyze_esr_heuristic( data )
    recommend_defaults( data )

    if args.csv:
        print_header( "CSV EXPORT" )
        export_csv( data, args.csv )

    print()


if __name__ == "__main__":
    main()
