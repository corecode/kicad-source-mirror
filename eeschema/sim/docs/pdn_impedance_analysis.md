# PDN Impedance Analysis — Technical Reference

Design rationale for the heuristics and simplifications in `pdn_analyzer.{h,cpp}`.

The analyzer builds a lumped-element SPICE model from schematic data alone — no board
geometry, no vendor models — and runs an ngspice AC sweep to produce an impedance curve.
The intent is to show the shape of Z(f), not to predict milliohm-accurate impedance.


## Capacitor detection

A symbol is included if all three conditions hold:

1. Refdes matches `^CP?[0-9?]`.
2. Exactly two pins.
3. Both pins on power nets (`PT_POWER_IN`, `PT_POWER_OUT`, or global power symbol).

The regex accepts `C1`, `CP3`, `C?` but rejects `CONN1`, `CMOS1`, `CE1`. The two-pin
filter eliminates connectors and other multi-terminal parts that share the `C` prefix.
Requiring both nets to be power nets excludes signal-path coupling and DC-blocking caps
that do not belong in a PDN model. Ferrite beads, series inductors, and other power-path
components are not modeled.

Symbols whose value or footprint cannot be parsed are skipped with a per-network warning.


## Supply vs reference classification

`classifySupplyAndRef()` decides which of a capacitor's two nets is supply and which is
reference, using a three-step cascade:

1. Regex `(gnd|vss|ground|vee)` (case-insensitive) — if exactly one net matches, it is
   the reference rail.
2. Pin-count fallback — the net with more total pin connections is treated as reference,
   on the assumption that ground nets fan out more widely.
3. Alphabetical tiebreaker — deterministic but arbitrary; only reached when both nets
   are indistinguishable by the first two rules.

This misclassifies negative supplies named `VEE` (caught by step 1) and unconventional
designs where the supply has higher fan-out than ground. No manual override exists.


## Case size parsing

`ParseCaseSize()` extracts a metric case size from the footprint string by scanning for
4–5 digit groups and checking each against two tables:

- `s_eslByCase` — keyed by metric size (0204 … 5750).
- `s_eiaToMetric` — maps EIA codes (008004 … 2220) to their metric equivalents.

Some codes (notably `0402` and `0603`) appear in both systems. Resolution order:

1. Unambiguous metric hit → use directly.
2. Unambiguous EIA hit → convert to metric.
3. Ambiguous → treat as EIA (most KiCad library footprints use EIA designations).

Returns empty on failure — no guessing, no defaults.

This handles `C_0805_2012Metric`, bare `0805`, `CAP_0R1_0402`, etc. without assuming
any particular footprint naming convention.


## Parasitic ESR/ESL

Derived from Murata's MLCC SPICE model database (23,603 parts across GRM and GCM
series). See `mlcc_parasitic_plan.md` for full methodology.

**ESL** is a per-case-size lookup (median from Murata data). ESL depends on package
geometry and is nearly independent of capacitance or dielectric type.

**ESR** is computed from a power-law heuristic calibrated on Class II ceramics:

```
ESR = k(case) × C^(−0.429)
```

where k varies by case size (~2.3e-5 to ~3.9e-5). The exponent α = −0.429 is consistent
across all case sizes (IQR: −0.45 to −0.42). This captures the dominant effect: more
capacitance → more electrode layers in parallel → lower resistance.

The model explains 93% of ESR variance (R² = 0.93 in log space). Typical prediction
error is within 2× of the actual value — well within the inherent scatter of MLCC
parasitics across dielectrics and voltage ratings.

The heuristic tends to **underestimate ESR at high capacitances**, which is the
conservative direction for PDN analysis: low ESR means high Q, producing pessimistically
large anti-resonance peaks. The user sees worst-case impedance.

Not modeled: voltage rating dependence (~V^0.3, within existing scatter), C0G ceramics
(different ESR distribution, rare in PDN decoupling), DC bias derating, tantalums,
electrolytics.


## Circuit model

Each capacitor becomes a series C–R–L subcircuit (the standard lumped PDN model),
instantiated with per-component ESR and ESL:

```spice
.subckt CAP p n c=100n esr=0.01 esl=400p
C1 p 1 {c}
R1 1 2 {esr}
L1 2 n {esl}
.ends

X1 local gnd CAP c=100e-9 esr=0.0295 esl=380e-12
```

Caps are grouped by schematic sheet. The sheet the user is viewing is the *observation
sheet*; caps on it connect directly to the measurement node `local`. Caps on other
sheets go through a per-group trace impedance (series L + R):

```
I1 (1A AC) ──┬── local ──┬── cap (local)
              │           ├── cap (local)
              │           └──[L_trace]──[R_trace]── node_remote
              │                                      ├── cap (remote)
              │                                      └── cap (remote)
              └── gnd
```

Default trace impedance per hop: 10 nH, 5 mOhm — a short PCB trace or via pair.

Sheet hierarchy is used as a proxy for physical distance: designers typically place
local decoupling on the same sheet as the IC it serves. When all caps fall on one
sheet, the trace impedance drops out and the result is a flat parallel combination.

Not modeled: plane capacitance, mutual coupling, spreading inductance, via
inductance, actual stackup. Each remote sheet shares a single trace impedance rather
than per-cap paths. The heuristic breaks down when schematic organization does not
reflect board layout.


## VRM source impedance

When a voltage regulator is detected on the supply rail, a VRM (Voltage Regulator
Module) impedance model is included to provide correct low-frequency behavior. Without
it, impedance goes to infinity below the bulk capacitor resonance. The VRM also creates
the VRM-capacitor anti-resonance peak that's critical for PDN design validation.

If no regulator is found on the supply rail, no VRM model is added.

### VRM detection

Detection runs during `FindPDNNetworks()` using two methods in priority order.

**Method 1 — Direct power output**: A component has a `PT_POWER_OUT` pin connected
to the supply net and a `PT_POWER_IN` pin on a different net. This catches regulators
where the output pin is typed correctly (LDOs, integrated SMPS modules).

**Method 2 — IC-centric SMPS detection**: Many SMPS regulators have an external
output inductor, so their output pin is not `PT_POWER_OUT` on the supply rail. These
are detected by starting from candidate ICs and verifying the SMPS topology. An IC
qualifies when all three conditions hold:

1. **Foreign power input** — the IC has `PT_POWER_IN` on a net that is neither the
   supply rail nor the reference rail. This proves it converts power from a different
   rail (e.g. VIN) rather than just consuming from the output rail.
2. **Output inductor** — at least one of the IC's pins connects through a single
   inductor to the supply rail (the SW → L → VOUT path).
3. **Feedback divider** — the IC has a non-power pin on a net that reaches both the
   supply rail and the reference rail through 1–2 resistor hops each (the
   VOUT → R\_top → FB → R\_bot → GND voltage divider).

This handles typical feedback divider topologies:
- Simple: supply → R\_top → FB → R\_bot → GND
- Series: supply → R\_t1 → R\_t2 → FB → R\_b1 → R\_b2 → GND
- Parallel: multiple resistors between the same nets

Both methods exclude:
- Power symbols (just net labels).
- Capacitors and inductors (passive components).
- Components with `Pwr.Type = SWITCH` — load switches pass power through without
  regulation and should not be modeled as impedance sources.

If multiple regulators drive the same rail, the first one found is used and a warning
is issued about the others. If no regulator is found, no VRM model is added.

### Circuit model

The VRM connects to the measurement node like any other component — locally if on the
observation sheet, or through one hop of trace impedance if on a different sheet:

```
        L_trace    R_trace
local ──(((──────/\/\/── vrm_node    (remote only)
                           │
             R_VRM  L_VRM  │
   gnd ──/\/\/──(((──────┤
                           │
           R_damp   L_damp │
   gnd ──/\/\/──(((──────┘
```

The main branch (R\_VRM + L\_VRM) models the regulator's output impedance rising with
frequency as it loses control loop gain. The damping branch (R\_damp + L\_damp) prevents
an excessively sharp anti-resonance peak at the VRM-capacitor crossover.

### Parameters and schematic fields

Users can annotate the regulator symbol with custom fields to override the defaults:

| Field      | Default  | Description                                |
|------------|----------|--------------------------------------------|
| `Pwr.Zout` | 50 mOhm  | Regulator output impedance (sets R\_damp)  |
| `Pwr.BW`   | 10 kHz   | Control loop bandwidth                     |

Values accept SI suffixes (e.g. `50m`, `10k`, `20kHz`). When `Pwr.Zout` is specified,
R\_damp is set to the given value and R\_VRM (DC resistance) is derived as Zout / 50.

Internally the model uses three parameters:

| Internal    | Default     | Derived from                          |
|-------------|-------------|---------------------------------------|
| R\_damp     | 50 mOhm     | `Pwr.Zout` directly                  |
| R\_VRM      | 1 mOhm      | `Pwr.Zout` / 50                      |
| bandwidth   | 10 kHz       | `Pwr.BW` directly                    |
| L\_VRM      | ~795.8 nH    | R\_damp / (2 \* pi \* bandwidth)     |
| L\_damp     | ~79.6 nH     | L\_VRM / 10                          |

### Estimating Pwr.Zout and Pwr.BW from datasheets

**Pwr.BW** (bandwidth) can be estimated by several methods:
- From load transient recovery time: BW ≈ 1 / (3 × t\_recovery)
- From switching frequency: typically 1/15th to 1/50th of f\_sw
- From output inductance: f\_BW ≈ 120 / sqrt(L\_out) where L\_out is in Henries

**Pwr.Zout** (output impedance) can be estimated from:
- Load transient voltage dip: Zout ≈ V\_dip / I\_step
- Adaptive Voltage Positioning (AVP) load-line slope
- Typical values: 10–100 mOhm for modern DC-DC converters, higher for LDOs

The default values are conservative estimates suitable for a generic switching regulator.


## Simulation

| Parameter       | Value        |
|-----------------|--------------|
| Frequency range | 1 kHz – 1 GHz |
| Points/decade   | 100          |
| Excitation      | 1 A AC current source across `local`–`gnd` |
| Probe           | `v(local)` magnitude → \|Z(f)\| directly |

1 kHz captures the bulk-cap region; 1 GHz is roughly where the lumped model stops being
physically meaningful. 100 points/decade gives smooth curves. The 1 A source normalizes
impedance to voltage (Z = V), avoiding post-processing.

The simulation runs synchronously (`Command("run")`, not `bg_run`) with `LOCALE_IO`
to force C locale for SPICE number formatting. Typical PDN netlists complete in
milliseconds; the synchronous call avoids the need for a wxWidgets event loop.

The frequency range is not yet user-configurable.
