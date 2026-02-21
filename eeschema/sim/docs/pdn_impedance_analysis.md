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
