#!/usr/bin/env python3
"""
MLCC SPICE Netlist Collector & Bulk Simulator

Downloads manufacturer SPICE model archives (Murata), simulates each subcircuit
via ngspice AC analysis, extracts parasitic parameters (ESR, ESL, SRF, C_eff),
and stores everything in a SQLite database.

Usage:
    python3 collect_mlcc_data.py [options]

Requires ngspice on PATH. No external Python packages (stdlib only).
"""

import argparse
import concurrent.futures
import logging
import math
import os
import re
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
import zipfile

log = logging.getLogger( "collect_mlcc" )

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

MURATA_URL_TEMPLATE = (
    "https://www.murata.com/-/media/webrenewal/tool/netlist/mlcc/v75/"
    "{series}-n-v75.ashx"
)

MURATA_SERIES = [
    "ga2",  "ga3",  "ga9",  "gcb",  "gcc",  "gcm",  "gcd",  "gce",
    "gcj",  "gcq",  "gj2",  "gjm",  "gqm",  "gr3",  "grj",  "grm",
    "grs",  "grt",  "grz",  "kca",  "kcm",  "krm",  "lll",  "nca",
    "ncm",  "nfm",  "ngm",  "nlm",  "nrm",  "ta2",  "tba",  "tca",
    "tcm",  "tpa",  "tpm",
]

AC_FSTART = 1e3     # 1 kHz
AC_FSTOP  = 1e9     # 1 GHz
AC_POINTS = 100     # points per decade (6 decades → 600 total)

DEFAULT_BATCH_SIZE  = 500
DEFAULT_WORKERS     = 4
NGSPICE_TIMEOUT     = 300   # seconds per batch

# ---------------------------------------------------------------------------
# Part number decoding — Murata MLCC
# ---------------------------------------------------------------------------

# Size code → (EIA string, metric string, L_mm, W_mm)
SIZE_CODES = {
    "01": ( "008004", "0204", 0.25, 0.125 ),
    "02": ( "01005", "0402",  0.4,  0.2 ),
    "03": ( "0201",  "0603",  0.6,  0.3 ),
    "05": ( "0202",  "0505",  0.5,  0.5 ),
    "15": ( "0402",  "1005",  1.0,  0.5 ),
    "18": ( "0603",  "1608",  1.6,  0.8 ),
    "21": ( "0805",  "2012",  2.0,  1.25 ),
    "22": ( "0808",  "2020",  2.0,  2.0 ),
    "31": ( "1206",  "3216",  3.2,  1.6 ),
    "32": ( "1210",  "3225",  3.2,  2.5 ),
    "42": ( "1808",  "4520",  4.5,  2.0 ),
    "43": ( "1812",  "4532",  4.5,  3.2 ),
    "55": ( "2220",  "5750",  5.7,  5.0 ),
}

# Voltage code → rated voltage in volts (standard 2-char codes)
VOLTAGE_CODES = {
    "0E": 2.5,    "0G": 4.0,    "0J": 6.3,   "1A": 10.0,
    "1C": 16.0,   "1E": 25.0,   "1V": 35.0,   "1H": 50.0,
    "2A": 100.0,  "2E": 250.0,  "2W": 450.0,  "2J": 630.0,
    "3A": 1000.0, "3D": 2000.0, "3F": 3150.0,
}

# Single-character voltage codes used in automotive/GCM-style part numbers.
# These correspond to the second character of the 2-char code at the most
# common voltage tier for MLCCs.
VOLTAGE_CODES_1CHAR = {
    "B": 6.3,     "D": 20.0,    "E": 25.0,    "F": 50.0,
    "G": 4.0,     "H": 50.0,    "J": 6.3,     "K": 80.0,
    "A": 10.0,    "C": 16.0,
}

# Dielectric code prefix → dielectric name
DIELECTRIC_CODES = {
    # Class II / III
    "R1": "X5R",  "R6": "X5R",  "R7": "X7R",  "R8": "X8R",  "R9": "X7S",
    "B1": "X7R",  "B3": "X7R",  "B7": "X7R",
    "C6": "X5R",  "C7": "X7R",  "C8": "X8R",
    "D7": "X7R",  "D8": "X8R",
    "E2": "X7R",  "E7": "X7R",
    "F5": "C0G",  "F7": "X7R",
    "S2": "X7R",
    # Class I (C0G / NP0 / U2J)
    "C0": "C0G",  "C1": "C0G",  "C2": "C0G",  "C3": "C0G",
    "G1": "C0G",  "G2": "C0G",
    "J3": "C0G",  "J4": "C0G",  "J5": "C0G",
    "U0": "U2J",  "U1": "U2J",  "U2": "U2J",  "U3": "U2J",
    # Additional
    "L8": "X8L",  "M8": "X8M",
    "X0": "X5R",  "X1": "X5R",  "X2": "X7R",
    "Z7": "X7R",
}

TOLERANCE_CODES = {
    "B": "±0.1pF", "C": "±0.25pF", "D": "±0.5pF",
    "F": "±1%",    "G": "±2%",     "J": "±5%",
    "K": "±10%",   "M": "±20%",    "Z": "+80/-20%",
}


def decode_capacitance( code ):
    """Decode 3-char capacitance code → value in Farads.

    Examples: '104' → 100e-9, '010' → 1e-12, 'R10' → 0.1e-12
    """
    if not code or len( code ) != 3:
        return None
    # Handle R notation (e.g., 'R10' = 0.10 pF)
    if code[0] == "R":
        try:
            val = float( "0." + code[1:] )
            return val * 1e-12
        except ValueError:
            return None
    if code[1] == "R":
        try:
            val = float( code[0] + "." + code[2] )
            return val * 1e-12
        except ValueError:
            return None
    try:
        sig = int( code[:2] )
        exp = int( code[2] )
        return sig * 10**exp * 1e-12
    except ValueError:
        return None


# Standard Murata part number regex (GRM-style)
# Format: Series(3) Size(2) Thick(1) Diel(2) Volt(2) Cap(3) Tol(1) Suffix
_MURATA_PN_RE = re.compile(
    r"^(?P<series>[A-Z]{2}[A-Z0-9])" # GRM, GCM, LLL, GR3, GA2, etc.
    r"(?P<size>\d{2})"                # size code
    r"(?P<thick>[A-Z0-9])"           # thickness code (alphanumeric)
    r"(?P<diel>[A-Z]\d)"             # dielectric code
    r"(?P<volt>[A-Z0-9]{2})"          # voltage code (2-char)
    r"(?P<cap>[R0-9]{3})"            # capacitance code
    r"(?P<tol>[A-Z])"                # tolerance code
    r"(?P<suffix>.*)",                # packaging/suffix
    re.IGNORECASE,
)

# Automotive / GCM-style regex — extra digit after thickness, 1-char voltage
# Format: Series(3) Size(2) Thick(1) Extra(1) Diel(2) Volt(1) Cap(3) Tol(1) Suffix
_MURATA_GCM_PN_RE = re.compile(
    r"^(?P<series>[A-Z]{2}[A-Z0-9])" # GCM, GCJ, GCQ, KCA, etc.
    r"(?P<size>\d{2})"                # size code
    r"(?P<thick>\d)"                  # thickness code (digit)
    r"(?P<extra>\d)"                  # extra characteristic digit
    r"(?P<diel>[A-Z][A-Z0-9])"       # dielectric code
    r"(?P<volt>[A-Z])"               # voltage code (1-char)
    r"(?P<cap>[R0-9]{3})"            # capacitance code
    r"(?P<tol>[A-Z])"                # tolerance code
    r"(?P<suffix>.*)",                # packaging/suffix
    re.IGNORECASE,
)

_EMPTY_DECODE = {
    "series": None, "case_size_eia": None, "case_size_metric": None,
    "dielectric": None, "capacitance_F": None, "voltage_V": None,
    "tolerance": None,
}


def decode_murata_pn( part_number ):
    """Decode a Murata MLCC part number into a metadata dict.

    Tries the standard GRM-style format first, then the automotive GCM-style
    format (which has an extra characteristic digit and single-char voltage).

    Returns dict with keys: series, case_size_eia, case_size_metric,
    dielectric, capacitance_F, voltage_V, tolerance.
    Values may be None if decoding fails for that field.
    """
    m = _MURATA_PN_RE.match( part_number )
    if m:
        series = m.group( "series" ).upper()
        size_code = m.group( "size" )
        diel_code = m.group( "diel" ).upper()
        volt_code = m.group( "volt" ).upper()
        cap_code  = m.group( "cap" ).upper()
        tol_code  = m.group( "tol" ).upper()

        size_info = SIZE_CODES.get( size_code )

        return {
            "series":           series,
            "case_size_eia":    size_info[0] if size_info else None,
            "case_size_metric": size_info[1] if size_info else None,
            "dielectric":       DIELECTRIC_CODES.get( diel_code ),
            "capacitance_F":    decode_capacitance( cap_code ),
            "voltage_V":        VOLTAGE_CODES.get( volt_code ),
            "tolerance":        TOLERANCE_CODES.get( tol_code ),
        }

    m = _MURATA_GCM_PN_RE.match( part_number )
    if m:
        series = m.group( "series" ).upper()
        size_code = m.group( "size" )
        diel_code = m.group( "diel" ).upper()
        volt_code = m.group( "volt" ).upper()
        cap_code  = m.group( "cap" ).upper()
        tol_code  = m.group( "tol" ).upper()

        size_info = SIZE_CODES.get( size_code )

        return {
            "series":           series,
            "case_size_eia":    size_info[0] if size_info else None,
            "case_size_metric": size_info[1] if size_info else None,
            "dielectric":       DIELECTRIC_CODES.get( diel_code ),
            "capacitance_F":    decode_capacitance( cap_code ),
            "voltage_V":        VOLTAGE_CODES_1CHAR.get( volt_code ),
            "tolerance":        TOLERANCE_CODES.get( tol_code ),
        }

    return dict( _EMPTY_DECODE )


# ---------------------------------------------------------------------------
# Subcircuit scanning
# ---------------------------------------------------------------------------

_SUBCKT_RE = re.compile(
    r"^\s*\.subckt\s+(\S+)\s+(.*)", re.IGNORECASE | re.MULTILINE
)


def scan_subcircuits( netlist_text ):
    """Scan SPICE netlist text for .subckt definitions.

    Returns list of (name, [pin_names]) tuples.
    """
    results = []
    for m in _SUBCKT_RE.finditer( netlist_text ):
        name = m.group( 1 )
        pins = m.group( 2 ).strip().split()
        # Filter out parameter assignments (e.g., params: ...)
        clean_pins = []
        for p in pins:
            if "=" in p or p.lower() == "params:":
                break
            clean_pins.append( p )
        results.append( ( name, clean_pins ) )
    return results


# ---------------------------------------------------------------------------
# ngspice simulation
# ---------------------------------------------------------------------------

def build_batch_circuit( batch_items ):
    """Build a SPICE circuit that simulates a batch of subcircuits.

    Args:
        batch_items: list of (name, [pins], netlist_path) tuples

    Returns:
        (circuit_text, ordered_probe_names)
    """
    lines = []
    lines.append( "* MLCC batch AC simulation" )

    # Collect unique netlist files to include
    included = set()
    for _name, _pins, netlist_path in batch_items:
        if netlist_path not in included:
            lines.append( ".include {}".format( netlist_path ) )
            included.add( netlist_path )
    lines.append( "" )

    probe_names = []
    for i, ( name, pins, _path ) in enumerate( batch_items ):
        idx = "{:04d}".format( i )
        node = "in_{}".format( idx )

        # Current source: AC 1A into node
        lines.append( "I_{idx} {node} 0 AC 1".format( idx=idx, node=node ) )

        # Instantiate subcircuit — connect node to first pin, 0 to second
        if len( pins ) >= 2:
            lines.append( "X_{idx} {node} 0 {name}".format(
                idx=idx, node=node, name=name ) )
        else:
            # Single-pin or zero-pin — unusual, skip safely
            lines.append( "* SKIP: {name} has {n} pins".format(
                name=name, n=len( pins ) ) )
            continue

        probe_names.append( ( name, node ) )
        lines.append( "" )

    lines.append( ".ac dec {pts} {fstart} {fstop}".format(
        pts=AC_POINTS, fstart=AC_FSTART, fstop=AC_FSTOP ) )
    lines.append( "" )
    lines.append( ".control" )
    lines.append( "run" )

    # Build wrdata command with all probe nodes
    if probe_names:
        probes = " ".join( "v({})".format( node ) for _, node in probe_names )
        lines.append( "wrdata output.dat " + probes )

    lines.append( ".endc" )
    lines.append( ".end" )

    return "\n".join( lines ), probe_names


def parse_wrdata( data_text, n_probes ):
    """Parse ngspice wrdata output for AC analysis.

    wrdata AC format: each probe gets its own (freq, Re, Im) triplet:
        freq0 Re(v0) Im(v0) freq1 Re(v1) Im(v1) ...
    So each line has 3*n_probes columns.

    Returns dict mapping probe_index → list of (freq, z_real, z_imag) tuples.
    Since I=1A, V=Z directly.
    """
    results = { i: [] for i in range( n_probes ) }
    lines = data_text.strip().split( "\n" )

    # Filter out empty lines and comments
    lines = [ l for l in lines if l.strip() and not l.strip().startswith( "#" ) ]

    if not lines:
        return results

    expected_cols = 3 * n_probes

    for line in lines:
        parts = line.split()
        if len( parts ) < expected_cols:
            continue

        for p in range( n_probes ):
            try:
                freq   = float( parts[3 * p] )
                z_real = float( parts[3 * p + 1] )
                z_imag = float( parts[3 * p + 2] )
                results[p].append( ( freq, z_real, z_imag ) )
            except ( ValueError, IndexError ):
                pass

    return results


def run_ngspice_batch( batch_args ):
    """Worker function: run ngspice on a batch circuit.

    Args:
        batch_args: (batch_items, batch_id)
            batch_items: list of (name, [pins], netlist_path)

    Returns:
        list of (part_name, [(freq, z_real, z_imag), ...]) or
        (part_name, None) on failure
    """
    batch_items, batch_id = batch_args

    circuit_text, probe_names = build_batch_circuit( batch_items )

    if not probe_names:
        return [ ( name, None ) for name, _, _ in batch_items ]

    tmpdir = tempfile.mkdtemp( prefix="mlcc_batch_{}_".format( batch_id ) )
    try:
        cir_path = os.path.join( tmpdir, "batch.cir" )
        out_path = os.path.join( tmpdir, "output.dat" )

        with open( cir_path, "w" ) as f:
            f.write( circuit_text )

        try:
            proc = subprocess.run(
                [ "ngspice", "-b", cir_path ],
                capture_output=True, text=True,
                timeout=NGSPICE_TIMEOUT,
                cwd=tmpdir,
            )
        except subprocess.TimeoutExpired:
            log.warning( "Batch %d: ngspice timeout", batch_id )
            return [ ( name, None ) for name, _ in probe_names ]
        except FileNotFoundError:
            log.error( "ngspice not found on PATH" )
            return [ ( name, None ) for name, _ in probe_names ]

        if proc.returncode != 0:
            # Log first few lines of stderr for debugging
            stderr_preview = "\n".join( proc.stderr.split( "\n" )[:10] )
            log.warning( "Batch %d: ngspice returned %d:\n%s",
                         batch_id, proc.returncode, stderr_preview )
            return [ ( name, None ) for name, _ in probe_names ]

        if not os.path.exists( out_path ):
            log.warning( "Batch %d: no output file produced", batch_id )
            return [ ( name, None ) for name, _ in probe_names ]

        with open( out_path, "r" ) as f:
            data_text = f.read()

        parsed = parse_wrdata( data_text, len( probe_names ) )

        results = []
        for idx, ( name, _node ) in enumerate( probe_names ):
            curve = parsed.get( idx )
            if curve:
                results.append( ( name, curve ) )
            else:
                results.append( ( name, None ) )

        return results

    finally:
        shutil.rmtree( tmpdir, ignore_errors=True )


# ---------------------------------------------------------------------------
# Parasitic extraction
# ---------------------------------------------------------------------------

def extract_parasitics( curve ):
    """Extract parasitic parameters from an impedance curve.

    Args:
        curve: list of (freq, z_real, z_imag) tuples

    Returns:
        dict with keys: esr_ohm, esl_H, srf_Hz, c_effective_F,
                        q_at_srf, z_at_1mhz, z_at_10mhz, z_at_100mhz
        Values may be NaN if extraction fails.
    """
    nan = float( "nan" )
    result = {
        "esr_ohm": nan, "esl_H": nan, "srf_Hz": nan,
        "c_effective_F": nan, "q_at_srf": nan,
        "z_at_1mhz": nan, "z_at_10mhz": nan, "z_at_100mhz": nan,
    }

    if not curve or len( curve ) < 3:
        return result

    # Find SRF: frequency where |Z| is minimum
    freqs = [ c[0] for c in curve ]
    z_mags = [ math.sqrt( c[1]**2 + c[2]**2 ) for c in curve ]

    min_idx = min( range( len( z_mags ) ), key=lambda i: z_mags[i] )
    srf = freqs[min_idx]
    result["srf_Hz"] = srf

    # ESR: Re(Z) at SRF
    esr = curve[min_idx][1]
    result["esr_ohm"] = abs( esr )

    # Q at SRF
    if esr != 0:
        result["q_at_srf"] = abs( curve[min_idx][2] ) / abs( esr )

    # Determine sign convention from data: in ngspice with our test circuit
    # setup, capacitive Im(Z) may be positive or negative depending on
    # current source polarity. Detect by checking sign at low frequencies
    # (below SRF, impedance is capacitive).
    cap_sign = 0
    for f, _zr, zi in curve:
        if f < srf / 2.0 and abs( zi ) > abs( _zr ) * 0.1:
            cap_sign = 1 if zi > 0 else -1
            break

    # ESL: median of |Im(Z)|/(2*pi*f) for f > 2*SRF in the inductive region
    # (opposite sign from capacitive region)
    esl_samples = []
    for f, _zr, zi in curve:
        if f > 2.0 * srf and zi * cap_sign < 0:
            esl = abs( zi ) / ( 2.0 * math.pi * f )
            esl_samples.append( esl )

    if esl_samples:
        esl_samples.sort()
        result["esl_H"] = esl_samples[len( esl_samples ) // 2]

    # C_eff: median of 1/(2*pi*f*|Im(Z)|) for f < SRF/2 in the capacitive region
    c_samples = []
    for f, _zr, zi in curve:
        if f < srf / 2.0 and zi * cap_sign > 0 and abs( zi ) > 0:
            c = 1.0 / ( 2.0 * math.pi * f * abs( zi ) )
            if c > 0:
                c_samples.append( c )

    if c_samples:
        c_samples.sort()
        result["c_effective_F"] = c_samples[len( c_samples ) // 2]

    # |Z| at specific frequencies via log-log interpolation
    for target_f, key in [
        ( 1e6,  "z_at_1mhz" ),
        ( 10e6, "z_at_10mhz" ),
        ( 100e6, "z_at_100mhz" ),
    ]:
        result[key] = _interpolate_z_mag( curve, target_f )

    return result


def _interpolate_z_mag( curve, target_freq ):
    """Log-log interpolate |Z| at target_freq from curve data."""
    if not curve:
        return float( "nan" )

    # Find bracketing points
    for i in range( len( curve ) - 1 ):
        f0 = curve[i][0]
        f1 = curve[i + 1][0]
        if f0 <= target_freq <= f1:
            z0 = math.sqrt( curve[i][1]**2 + curve[i][2]**2 )
            z1 = math.sqrt( curve[i + 1][1]**2 + curve[i + 1][2]**2 )
            if z0 <= 0 or z1 <= 0 or f0 <= 0 or f1 <= 0:
                return float( "nan" )
            # Log-log interpolation
            log_f0 = math.log10( f0 )
            log_f1 = math.log10( f1 )
            log_ft = math.log10( target_freq )
            t = ( log_ft - log_f0 ) / ( log_f1 - log_f0 )
            log_z = math.log10( z0 ) + t * ( math.log10( z1 ) - math.log10( z0 ) )
            return 10**log_z

    return float( "nan" )


# ---------------------------------------------------------------------------
# SQLite database
# ---------------------------------------------------------------------------

DB_SCHEMA = """
CREATE TABLE IF NOT EXISTS parts (
    part_number    TEXT PRIMARY KEY,
    series         TEXT,
    case_size_eia  TEXT,
    case_size_metric TEXT,
    dielectric     TEXT,
    capacitance_F  REAL,
    voltage_V      REAL,
    tolerance      TEXT,
    source_file    TEXT
);

CREATE TABLE IF NOT EXISTS parasitics (
    part_number    TEXT PRIMARY KEY REFERENCES parts(part_number),
    esr_ohm        REAL,
    esl_H          REAL,
    srf_Hz         REAL,
    c_effective_F  REAL,
    q_at_srf       REAL,
    z_at_1mhz      REAL,
    z_at_10mhz     REAL,
    z_at_100mhz    REAL
);

CREATE TABLE IF NOT EXISTS impedance_curves (
    part_number    TEXT REFERENCES parts(part_number),
    freq_Hz        REAL,
    z_real_ohm     REAL,
    z_imag_ohm     REAL,
    PRIMARY KEY (part_number, freq_Hz)
);
"""


def init_db( db_path ):
    """Initialize SQLite database with schema."""
    conn = sqlite3.connect( db_path )
    conn.executescript( DB_SCHEMA )
    conn.execute( "PRAGMA journal_mode=WAL" )
    conn.execute( "PRAGMA synchronous=NORMAL" )
    conn.commit()
    return conn


def get_existing_parts( conn ):
    """Return set of part_numbers already in the parasitics table."""
    cursor = conn.execute( "SELECT part_number FROM parasitics" )
    return { row[0] for row in cursor.fetchall() }


def store_part( conn, part_number, metadata, source_file ):
    """Insert or replace a part record."""
    conn.execute(
        "INSERT OR REPLACE INTO parts "
        "(part_number, series, case_size_eia, case_size_metric, "
        " dielectric, capacitance_F, voltage_V, tolerance, source_file) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
        ( part_number,
          metadata.get( "series" ),
          metadata.get( "case_size_eia" ),
          metadata.get( "case_size_metric" ),
          metadata.get( "dielectric" ),
          metadata.get( "capacitance_F" ),
          metadata.get( "voltage_V" ),
          metadata.get( "tolerance" ),
          source_file ),
    )


def store_parasitics( conn, part_number, params ):
    """Insert or replace parasitic parameters."""
    conn.execute(
        "INSERT OR REPLACE INTO parasitics "
        "(part_number, esr_ohm, esl_H, srf_Hz, c_effective_F, "
        " q_at_srf, z_at_1mhz, z_at_10mhz, z_at_100mhz) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
        ( part_number,
          params["esr_ohm"], params["esl_H"], params["srf_Hz"],
          params["c_effective_F"], params["q_at_srf"],
          params["z_at_1mhz"], params["z_at_10mhz"], params["z_at_100mhz"] ),
    )


def store_impedance_curve( conn, part_number, curve ):
    """Insert impedance curve data points."""
    conn.executemany(
        "INSERT OR REPLACE INTO impedance_curves "
        "(part_number, freq_Hz, z_real_ohm, z_imag_ohm) VALUES (?, ?, ?, ?)",
        [ ( part_number, f, zr, zi ) for f, zr, zi in curve ],
    )


# ---------------------------------------------------------------------------
# Download & extraction
# ---------------------------------------------------------------------------

def download_series( series, cache_dir ):
    """Download a Murata series ZIP to cache_dir. Returns path or None."""
    url = MURATA_URL_TEMPLATE.format( series=series )
    zip_name = "{}-n-v75.zip".format( series )
    zip_path = os.path.join( cache_dir, zip_name )

    if os.path.exists( zip_path ):
        log.info( "Using cached %s", zip_name )
        return zip_path

    log.info( "Downloading %s ...", url )
    try:
        req = urllib.request.Request( url, headers={
            "User-Agent": "KiCad-MLCC-Collector/1.0",
        } )
        with urllib.request.urlopen( req, timeout=60 ) as resp:
            data = resp.read()
    except ( urllib.error.URLError, urllib.error.HTTPError, OSError ) as e:
        log.warning( "Failed to download %s: %s", series, e )
        return None

    # Verify it's a ZIP
    if not data[:4] == b"PK\x03\x04":
        # Might be a redirect page or error — try to save anyway
        log.warning( "Downloaded %s doesn't look like a ZIP (%d bytes)",
                     series, len( data ) )
        # Still save it — might be valid
        pass

    with open( zip_path, "wb" ) as f:
        f.write( data )
    log.info( "Saved %s (%d KB)", zip_name, len( data ) // 1024 )
    return zip_path


def extract_netlists( zip_path ):
    """Extract SPICE netlist files from a ZIP archive.

    Returns list of (filename, content_text) tuples.
    """
    results = []
    try:
        with zipfile.ZipFile( zip_path, "r" ) as zf:
            for info in zf.infolist():
                # Look for SPICE netlist files
                name_lower = info.filename.lower()
                if name_lower.endswith( ( ".lib", ".cir", ".sp", ".spice",
                                          ".net", ".mod" ) ):
                    try:
                        data = zf.read( info.filename )
                        # Try UTF-8 first, then Latin-1 as fallback
                        try:
                            text = data.decode( "utf-8" )
                        except UnicodeDecodeError:
                            text = data.decode( "latin-1" )
                        results.append( ( info.filename, text ) )
                    except Exception as e:
                        log.warning( "Failed to read %s from ZIP: %s",
                                     info.filename, e )
    except zipfile.BadZipFile:
        log.warning( "Bad ZIP file: %s", zip_path )
    return results


# ---------------------------------------------------------------------------
# Batch orchestration
# ---------------------------------------------------------------------------

def make_batches( items, batch_size ):
    """Split items into batches of at most batch_size."""
    for i in range( 0, len( items ), batch_size ):
        yield items[i:i + batch_size]


def extract_netlist_to_file( netlist_text, tmpdir, filename ):
    """Write netlist text to a temporary file, return the path."""
    path = os.path.join( tmpdir, filename )
    with open( path, "w" ) as f:
        f.write( netlist_text )
    return path


def simulate_all( all_items, existing_parts, batch_size, workers, force ):
    """Simulate all subcircuits collected from a series.

    Args:
        all_items: list of (name, [pins], netlist_path) tuples
        existing_parts: set of already-processed part numbers
        batch_size: initial batch size (auto-tuned)
        workers: number of parallel ngspice processes
        force: if True, re-simulate existing parts

    Returns:
        list of (part_name, parasitics_dict, curve_or_None) tuples
    """
    # Filter already processed
    if not force:
        to_sim = [ s for s in all_items if s[0] not in existing_parts ]
    else:
        to_sim = list( all_items )

    if not to_sim:
        return []

    log.info( "Simulating %d subcircuits (batch_size=%d, workers=%d)",
              len( to_sim ), batch_size, workers )

    all_results = []
    batch_id = 0
    batches = list( make_batches( to_sim, batch_size ) )
    batch_args = [
        ( batch, batch_id + i )
        for i, batch in enumerate( batches )
    ]

    failed_items = []
    t0 = time.time()

    with concurrent.futures.ProcessPoolExecutor(
            max_workers=workers ) as executor:
        futures = {
            executor.submit( run_ngspice_batch, args ): args
            for args in batch_args
        }

        for future in concurrent.futures.as_completed( futures ):
            try:
                batch_results = future.result( timeout=NGSPICE_TIMEOUT + 30 )
            except Exception as e:
                args = futures[future]
                log.warning( "Batch %d failed: %s", args[1], e )
                # Mark all parts in this batch for retry
                for item in args[0]:
                    failed_items.append( item )
                continue

            for part_name, curve in batch_results:
                if curve is None:
                    # Find original item for retry
                    failed_items.append( None )  # placeholder
                    log.debug( "Part %s: no curve data", part_name )
                    continue
                params = extract_parasitics( curve )
                all_results.append( ( part_name, params, curve ) )

    elapsed = time.time() - t0
    batch_id += len( batches )

    if elapsed > 0:
        log.info( "Processed %d parts in %.1fs (%.0f parts/sec)",
                  len( to_sim ), elapsed, len( to_sim ) / elapsed )

    # Retry failed items from multi-part batches individually
    retry_items = [ item for item in failed_items
                    if item is not None ]
    if retry_items:
        log.info( "Retrying %d parts individually", len( retry_items ) )
        for item in retry_items:
            result = run_ngspice_batch( ( [item], batch_id ) )
            batch_id += 1
            for part_name, curve in result:
                if curve is not None:
                    params = extract_parasitics( curve )
                    all_results.append( ( part_name, params, curve ) )
                else:
                    log.warning( "Part %s: retry failed, skipping",
                                 part_name )

    return all_results


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

def run_self_test():
    """Run internal validation tests."""
    print( "Running self-tests..." )
    errors = 0

    # --- Part number decoding ---
    test_cases = [
        # Standard GRM-style
        ( "GRM155R71C104KA88", {
            "series": "GRM", "case_size_eia": "0402",
            "case_size_metric": "1005", "dielectric": "X7R",
            "voltage_V": 16.0, "tolerance": "±10%",
        } ),
        ( "GRM188R61A106KE69", {
            "series": "GRM", "case_size_eia": "0603",
            "case_size_metric": "1608", "dielectric": "X5R",
            "voltage_V": 10.0, "tolerance": "±10%",
        } ),
        ( "GRM21BR71E105KA99", {
            "series": "GRM", "case_size_eia": "0805",
            "case_size_metric": "2012", "dielectric": "X7R",
            "voltage_V": 25.0, "tolerance": "±10%",
        } ),
        # Automotive GCM-style (extra digit, 1-char voltage)
        ( "GCM0332C1H910FA01", {
            "series": "GCM", "case_size_eia": "0201",
            "dielectric": "C0G", "voltage_V": 50.0,
            "tolerance": "±1%",
        } ),
        ( "GCM1555C1E105KA16", {
            "series": "GCM", "case_size_eia": "0402",
            "dielectric": "C0G", "voltage_V": 25.0,
            "tolerance": "±10%",
        } ),
    ]

    for pn, expected in test_cases:
        decoded = decode_murata_pn( pn )
        for key, val in expected.items():
            if decoded.get( key ) != val:
                print( "  FAIL: {}  {}: expected {!r}, got {!r}".format(
                    pn, key, val, decoded.get( key ) ) )
                errors += 1

    # --- Capacitance decoding ---
    cap_tests = [
        ( "104", 100e-9 ),
        ( "105", 1e-6 ),
        ( "106", 10e-6 ),
        ( "010", 1e-12 ),
        ( "476", 47e6 * 1e-12 ),    # 47 * 10^6 pF = 47 µF
    ]

    for code, expected_F in cap_tests:
        got = decode_capacitance( code )
        if got is None:
            print( "  FAIL: cap '{}': expected {}, got None".format(
                code, expected_F ) )
            errors += 1
        elif abs( got - expected_F ) / expected_F > 0.01:
            print( "  FAIL: cap '{}': expected {}, got {}".format(
                code, expected_F, got ) )
            errors += 1

    # --- Subcircuit scanning ---
    test_netlist = """
* Test netlist
.subckt GRM155R71C104KA88 1 2
R1 1 3 0.5
L1 3 4 0.7n
C1 4 2 100n
.ends GRM155R71C104KA88

.subckt GRM188R61A106KE69 p1 p2
R1 p1 3 0.3
C1 3 p2 10u
.ends
"""
    subs = scan_subcircuits( test_netlist )
    if len( subs ) != 2:
        print( "  FAIL: expected 2 subcircuits, got {}".format( len( subs ) ) )
        errors += 1
    elif subs[0][0] != "GRM155R71C104KA88":
        print( "  FAIL: first subcircuit name: {}".format( subs[0][0] ) )
        errors += 1
    elif subs[0][1] != ["1", "2"]:
        print( "  FAIL: first subcircuit pins: {}".format( subs[0][1] ) )
        errors += 1

    # --- Parasitic extraction ---
    # Construct a synthetic impedance curve for a known RLC circuit:
    # C=100nF, R=0.05 ohm, L=0.7nH
    # SRF = 1/(2*pi*sqrt(L*C)) ≈ 19.02 MHz
    test_C = 100e-9
    test_R = 0.05
    test_L = 0.7e-9
    test_curve = []
    f = AC_FSTART
    while f <= AC_FSTOP:
        omega = 2 * math.pi * f
        z_imag = omega * test_L - 1.0 / ( omega * test_C )
        test_curve.append( ( f, test_R, z_imag ) )
        f *= 10**( 1.0 / AC_POINTS )

    params = extract_parasitics( test_curve )

    expected_srf = 1.0 / ( 2.0 * math.pi * math.sqrt( test_L * test_C ) )
    if abs( params["srf_Hz"] - expected_srf ) / expected_srf > 0.05:
        print( "  FAIL: SRF expected ~{:.0f}, got {:.0f}".format(
            expected_srf, params["srf_Hz"] ) )
        errors += 1

    if abs( params["esr_ohm"] - test_R ) / test_R > 0.1:
        print( "  FAIL: ESR expected ~{}, got {}".format(
            test_R, params["esr_ohm"] ) )
        errors += 1

    if abs( params["esl_H"] - test_L ) / test_L > 0.15:
        print( "  FAIL: ESL expected ~{}, got {}".format(
            test_L, params["esl_H"] ) )
        errors += 1

    if abs( params["c_effective_F"] - test_C ) / test_C > 0.15:
        print( "  FAIL: C_eff expected ~{}, got {}".format(
            test_C, params["c_effective_F"] ) )
        errors += 1

    if errors == 0:
        print( "All self-tests passed." )
    else:
        print( "{} self-test(s) FAILED.".format( errors ) )

    return errors == 0


# ---------------------------------------------------------------------------
# Re-decode existing part metadata
# ---------------------------------------------------------------------------

def redecode_parts( db_path ):
    """Re-decode part number metadata for all parts already in the DB.

    Updates the parts table in-place without re-running simulations.
    Useful after improving the part number decoder.
    """
    if not os.path.exists( db_path ):
        log.error( "Database not found: %s", db_path )
        return

    conn = sqlite3.connect( db_path )
    rows = conn.execute( "SELECT part_number, source_file FROM parts" ).fetchall()

    total = len( rows )
    updated = 0
    newly_decoded = 0

    # Snapshot before-state for reporting
    before_case = conn.execute(
        "SELECT COUNT(*) FROM parts WHERE case_size_eia IS NOT NULL"
    ).fetchone()[0]
    before_diel = conn.execute(
        "SELECT COUNT(*) FROM parts WHERE dielectric IS NOT NULL"
    ).fetchone()[0]

    for part_number, source_file in rows:
        meta = decode_murata_pn( part_number )
        conn.execute(
            "UPDATE parts SET series=?, case_size_eia=?, case_size_metric=?, "
            "dielectric=?, capacitance_F=?, voltage_V=?, tolerance=? "
            "WHERE part_number=?",
            ( meta.get( "series" ),
              meta.get( "case_size_eia" ),
              meta.get( "case_size_metric" ),
              meta.get( "dielectric" ),
              meta.get( "capacitance_F" ),
              meta.get( "voltage_V" ),
              meta.get( "tolerance" ),
              part_number ),
        )
        if meta.get( "case_size_eia" ):
            updated += 1

    conn.commit()

    after_case = conn.execute(
        "SELECT COUNT(*) FROM parts WHERE case_size_eia IS NOT NULL"
    ).fetchone()[0]
    after_diel = conn.execute(
        "SELECT COUNT(*) FROM parts WHERE dielectric IS NOT NULL"
    ).fetchone()[0]

    conn.close()

    print( "Re-decoded {:,} parts in {}".format( total, db_path ) )
    print( "  Case size:   {:,} → {:,}  (+{:,})".format(
        before_case, after_case, after_case - before_case ) )
    print( "  Dielectric:  {:,} → {:,}  (+{:,})".format(
        before_diel, after_diel, after_diel - before_diel ) )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="MLCC SPICE Netlist Collector & Bulk Simulator" )

    parser.add_argument( "--db", default="murata_mlcc.db",
                         help="Output SQLite database path "
                              "(default: murata_mlcc.db)" )
    parser.add_argument( "--cache-dir", default="./murata_cache",
                         help="Directory for downloaded ZIP cache "
                              "(default: ./murata_cache)" )
    parser.add_argument( "--series", nargs="+", default=None,
                         help="Process specific series only (default: all)" )
    parser.add_argument( "--workers", type=int, default=DEFAULT_WORKERS,
                         help="Parallel ngspice processes "
                              "(default: {})".format( DEFAULT_WORKERS ) )
    parser.add_argument( "--batch-size", type=int, default=DEFAULT_BATCH_SIZE,
                         help="Initial subcircuits per ngspice run "
                              "(default: {})".format( DEFAULT_BATCH_SIZE ) )
    parser.add_argument( "--no-curves", action="store_true",
                         help="Skip storing full Z(f) curves" )
    parser.add_argument( "--force", action="store_true",
                         help="Re-simulate parts already in DB" )
    parser.add_argument( "--dry-run", action="store_true",
                         help="Parse and count without simulating" )
    parser.add_argument( "--redecode", action="store_true",
                         help="Re-decode part number metadata in DB "
                              "(no simulation, updates parts table only)" )
    parser.add_argument( "--self-test", action="store_true",
                         help="Run internal validation tests" )
    parser.add_argument( "-v", "--verbose", action="store_true",
                         help="Debug logging" )

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-5s %(message)s",
        datefmt="%H:%M:%S",
    )

    if args.self_test:
        ok = run_self_test()
        sys.exit( 0 if ok else 1 )

    if args.redecode:
        redecode_parts( args.db )
        sys.exit( 0 )

    # Check ngspice availability
    if not args.dry_run:
        if shutil.which( "ngspice" ) is None:
            log.error( "ngspice not found on PATH. Install ngspice first." )
            sys.exit( 1 )

    # Determine series to process
    series_list = args.series if args.series else MURATA_SERIES
    series_list = [ s.lower() for s in series_list ]

    # Create cache directory
    os.makedirs( args.cache_dir, exist_ok=True )

    # Initialize database
    conn = init_db( args.db )
    existing_parts = get_existing_parts( conn ) if not args.force else set()

    total_subcircuits = 0
    total_simulated = 0
    total_stored = 0
    total_failed = 0

    # Process each series
    for series in series_list:
        log.info( "=== Processing series: %s ===", series.upper() )

        # Download
        zip_path = download_series( series, args.cache_dir )
        if zip_path is None:
            continue

        # Extract netlist files
        netlist_files = extract_netlists( zip_path )
        if not netlist_files:
            log.warning( "No netlist files found in %s", series )
            continue

        log.info( "Found %d netlist file(s) in %s",
                  len( netlist_files ), series )

        # Create temp dir for extracted netlists
        tmpdir = tempfile.mkdtemp( prefix="mlcc_{}_".format( series ) )
        try:
            # Phase 1: Scan all files, extract netlists to temp dir,
            # collect (name, pins, netlist_path, source) tuples
            all_items = []      # (name, pins, netlist_path)
            source_map = {}     # name → source string

            for filename, text in netlist_files:
                safe_name = os.path.basename( filename ).replace( " ", "_" )
                netlist_path = extract_netlist_to_file( text, tmpdir,
                                                       safe_name )
                subcircuits = scan_subcircuits( text )
                if not subcircuits:
                    continue

                total_subcircuits += len( subcircuits )

                for name, pins in subcircuits:
                    all_items.append( ( name, pins, netlist_path ) )
                    source_map[name] = "{}:{}".format( series, safe_name )

                    if args.dry_run:
                        meta = decode_murata_pn( name )
                        if meta["series"]:
                            log.debug( "  %s → %s %s %s %.0fV",
                                       name,
                                       meta["series"],
                                       meta["case_size_eia"] or "??",
                                       meta["dielectric"] or "??",
                                       meta["voltage_V"] or 0 )

            log.info( "  Total: %d subcircuits from %d files",
                      len( all_items ), len( netlist_files ) )

            if args.dry_run or not all_items:
                continue

            # Phase 2: Simulate all subcircuits in batches
            results = simulate_all(
                all_items, existing_parts,
                args.batch_size, args.workers, args.force )

            # Phase 3: Store results
            for part_name, params, curve in results:
                source = source_map.get( part_name, series )
                meta = decode_murata_pn( part_name )
                store_part( conn, part_name, meta, source )
                store_parasitics( conn, part_name, params )

                if curve and not args.no_curves:
                    store_impedance_curve( conn, part_name, curve )

                total_stored += 1

            total_simulated += len( results )
            conn.commit()
            log.info( "  Stored %d results for %s", len( results ), series )

        finally:
            shutil.rmtree( tmpdir, ignore_errors=True )

    conn.close()

    # Summary
    print( "\n" + "=" * 60 )
    print( "Summary" )
    print( "=" * 60 )
    print( "  Subcircuits found:  {:>8d}".format( total_subcircuits ) )

    if not args.dry_run:
        print( "  Simulated:          {:>8d}".format( total_simulated ) )
        print( "  Stored in DB:       {:>8d}".format( total_stored ) )
        print( "  Database:           {}".format( os.path.abspath( args.db ) ) )

        print( "\nSample queries:" )
        print( "  sqlite3 {} \"SELECT case_size_eia, COUNT(*), "
               "AVG(esr_ohm), AVG(esl_H*1e9) AS esl_nH "
               "FROM parts JOIN parasitics USING(part_number) "
               "WHERE dielectric='X7R' GROUP BY case_size_eia "
               "ORDER BY esl_nH;\"".format( args.db ) )

    print()


if __name__ == "__main__":
    main()
