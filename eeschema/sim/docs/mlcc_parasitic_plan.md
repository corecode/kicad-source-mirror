# MLCC Parasitic Modeling: Findings & Implementation

## Context

The PDN analysis tool needs default ESR and ESL estimates for MLCCs based on case size
and capacitance, for schematics where no vendor SPICE model is available. The data feeds
into:

```cpp
struct PARASITIC_ENTRY
{
    double esr; // Ohms
    double esl; // Henries
};
```

## Data Source

Murata MLCC SPICE model database: **23,603 parts** from GRM (standard) and GCM
(automotive) series, collected via `collect_mlcc_data.py`. Part number metadata decoded
for **93.7%** of parts (GRM 5-digit + GCM 6-digit formats). Database stored at
`eeschema/sim/tools/murata_mlcc.db`.

Each part's SPICE netlist was simulated across 1 kHz – 10 GHz to extract ESR at SRF,
total ESL, and impedance curves. The results are stored in the `parasitics` and
`impedance_curves` tables.

Analysis performed by `analyze_mlcc_data.py` (run with `--csv DIR` for export).

## ESL: Lookup Table by Case Size

ESL depends primarily on package geometry and is nearly independent of capacitance value
or dielectric type. A simple lookup table by metric case size is sufficient.

Median ESL values from the Murata SPICE database:

| Metric | EIA    | Median ESL (pH) |
|--------|--------|-----------------|
| 0204   | 008004 | 116             |
| 0402   | 01005  | 177             |
| 0603   | 0201   | 219             |
| 1005   | 0402   | 270             |
| 1608   | 0603   | 380             |
| 2012   | 0805   | 373             |
| 3216   | 1206   | 542             |
| 3225   | 1210   | 423             |
| 4532   | 1812   | 621             |
| 5750   | 2220   | 1220            |

These are median values across all dielectrics and capacitance values within each case
size. The spread is small (GSD typically 1.1–1.3×).

## ESR: Power-Law Heuristic

ESR varies strongly with capacitance (more layers in parallel → lower R) and weakly with
case size (terminal geometry). A log-log regression on Class II ceramics
(X5R/X7R/X7S/X8R/X8L/X8M) yields:

```
ESR = k(case) × C^α
```

where:
- **α = −0.429** — remarkably consistent across case sizes (IQR: −0.45 to −0.42)
- **k** varies by case size (~2.3e-5 to ~3.9e-5), reflecting terminal geometry

Per-case k coefficients:

| Metric | k        |
|--------|----------|
| 0204   | 3.35e-5  |
| 0402   | 3.87e-5  |
| 0603   | 3.15e-5  |
| 1005   | 3.15e-5  |
| 1608   | 2.93e-5  |
| 2012   | 2.93e-5  |
| 3216   | 3.43e-5  |
| 3225   | 2.39e-5  |
| 4532   | 2.28e-5  |
| 5750   | 3.42e-5  |

### Model quality

- Global R² = 0.93 (explains 93% of ESR variance in log space)
- Typical prediction error: within 2× of actual (the spread inherent in MLCCs)
- The model tends to **underestimate ESR at high capacitances** (10 µF+), which is the
  conservative direction for PDN analysis: low ESR → high Q → pessimistic anti-resonance
  peaks. The user sees worst-case impedance spikes.

### What is NOT modeled

- **Voltage rating**: higher voltage → thicker dielectric → fewer layers → higher ESR.
  The effect is real (~V^0.3) but was excluded to keep the model to two inputs
  (case size + capacitance). It's within the existing prediction scatter.
- **C0G/NP0**: These follow a different ESR distribution (lower, flatter). The heuristic
  is trained on Class II only. C0G caps are rare in PDN decoupling.
- **DC bias derating**: not modeled.
- **Tantalums / electrolytics**: completely different ESR characteristics.

## Implementation

In `pdn_analyzer.cpp`:

```cpp
static const std::map<wxString, double> s_eslByCase = { ... };
static constexpr double ESR_ALPHA = -0.429;
static const std::map<wxString, double> s_esrKByCase = { ... };

std::optional<PARASITIC_ENTRY> PDN_ANALYZER::GetParasitics(
        const wxString& aCaseSize, double aCapacitance )
{
    // ESL from table, ESR = k * pow(C, alpha)
}
```

The SPICE netlist uses a single generic parameterized subcircuit:

```spice
.subckt CAP p n c=100n esr=0.01 esl=400p
C1 p 1 {c}
R1 1 2 {esr}
L1 2 n {esl}
.ends
```

Each capacitor instance passes its computed ESR and ESL:

```spice
X1 local gnd CAP c=100e-9 esr=0.0295 esl=380e-12
```

## Tools

| Script | Purpose |
|--------|---------|
| `collect_mlcc_data.py` | Download & simulate Murata SPICE models, populate SQLite DB |
| `analyze_mlcc_data.py` | Statistical analysis, ESR heuristic fitting, CSV export |

Both live in `eeschema/sim/tools/`.

### collect_mlcc_data.py

- Downloads Murata SPICE netlist ZIPs (GRM, GCM, GR3, GRJ series)
- Decodes part numbers: GRM-style (5-digit) and GCM-style (6-digit, automotive)
- Simulates each netlist via ngspice to extract ESR, ESL, SRF, impedance curves
- Stores results in `murata_mlcc.db`
- `--redecode` flag to re-parse metadata without re-simulating

### analyze_mlcc_data.py

- Reads `murata_mlcc.db` and produces statistics by case size, dielectric, capacitance
- Computes ESR heuristic fit (per-case k, global alpha)
- Generates validation tables (predicted vs actual at each case×decade cell)
- `--csv DIR` exports all tables as CSV files

## What We Tried That Didn't Work

- **AVX formula** `L(pH) = 394.727 * 1.052^L * 1.317^(L/W)`: Based on old MLCC
  construction. Vastly overpredicts for modern parts.
- **Polynomial fit for ESL**: Cubic on diagonal gives R²=0.99 but a lookup table is
  simpler and more honest given only ~12 case sizes.
- **Physics-first ESR model**: Electrode sheet resistance / N_layers gives the right
  shape but absolute calibration requires internal geometry knowledge. Semi-empirical
  calibration against real data was the way to go.
- **Fixed ESR per case size**: The original implementation used a single ESR value per
  case size. This is too coarse — ESR varies ~1000× across the capacitance range within
  a case size, while case-to-case variation in k is only ~1.7×.
