# KiCad SI/PI — MVP Implementation Plan

## Philosophy

Get a visible, working impedance profiler panel as fast as possible using analytical formulas. Exercise all KiCad integration points (connectivity API, stackup, zone fills, AUI panels) before investing in the BEM solver. Each stage is independently testable and produces a visible result or a clear pass/fail. FastHenry2/FastCap2 are out of scope — we'll address PEEC only if the need arises later.

---

## Stage 1: Trace Path Walker (slim)

**Goal**: Given a track segment, follow connectivity to build an ordered pad-to-pad path with cumulative arc-length. This is the first thing that touches the KiCad board API — we want to find the walls early.

**Scope**:
- Straight segments (`PCB_TRACK`) and vias (`PCB_VIA`) only. No `PCB_ARC` handling yet (treat as straight chord if encountered).
- Single net, single path. No T-junction branching, no diff pair detection.
- Terminate at pads or dead ends.
- Use `VECTOR2I` integer coordinates throughout.

**Files**:
- `pcbnew/sipi/trace_path_walker.h`
- `pcbnew/sipi/trace_path_walker.cpp`

**Data structures**:
```
PATH_POINT {
    VECTOR2I    position;       // board coords (nm)
    VECTOR2D    tangent;        // unit tangent direction
    PCB_LAYER_ID layer;
    double      distFromStart;  // nm, cumulative
    BOARD_CONNECTED_ITEM* item; // source track or via
    bool        isVia;
};
```

**Class** `TRACE_PATH_WALKER`:
```
TRACE_PATH_WALKER( const BOARD* aBoard )
bool Walk( PCB_TRACK* aStartTrack )
const std::vector<PATH_POINT>& GetPath() const
double GetTotalLength() const  // nm
```

**Algorithm**:
1. Collect all tracks + vias on the net (`board->Tracks()` filtered by netcode)
2. Build adjacency: map from `VECTOR2I` endpoint → list of connected items
3. Pick a direction from the start track (toward the pad, or arbitrary)
4. Greedy walk: at each endpoint, pick the single connected item that continues the path (skip already-visited). Stop at pad, dead end, or junction (degree > 2 — just stop, don't branch).
5. Accumulate arc-length.

**Test** — `qa/tests/pcbnew/test_trace_path_walker.cpp`:
- Load a simple test `.kicad_pcb` with a straight trace between two pads. Verify path length equals expected.
- Trace with a via: verify layer change recorded, path continues on other side.
- Trace hitting a T-junction: verify walker stops cleanly.

**Risks to discover**: Does `board->Tracks()` actually give us everything? Do endpoint coordinates match exactly at junctions? Are there off-by-one issues at pad connections?

---

## Stage 2: Stackup Reader + Analytical Z₀

**Goal**: For each segment in the path, determine the stackup geometry and compute Z₀ using closed-form IPC-2141 formulas. No Eigen, no solver — just math.h. This tests whether `BOARD_STACKUP` actually gives us useful εr and thickness data.

**Scope**:
- Read per-layer thickness, εr, tan δ from `BOARD_STACKUP`
- Identify the reference plane: nearest copper layer above/below with a filled zone covering the trace location
- Compute Z₀ using standard microstrip or stripline formula depending on geometry
- Flag missing/default stackup data
- No neighbor coupling, no differential mode

**Files**:
- `pcbnew/sipi/stackup_reader.h`
- `pcbnew/sipi/stackup_reader.cpp`
- `pcbnew/sipi/analytical_impedance.h`
- `pcbnew/sipi/analytical_impedance.cpp`

**Data structures**:
```
LAYER_GEOMETRY {
    PCB_LAYER_ID signalLayer;
    double       traceWidth;      // meters
    double       traceThickness;  // meters
    double       hAbove;          // meters, to nearest ref plane above (0 if none)
    double       hBelow;          // meters, to nearest ref plane below (0 if none)
    double       erAbove;
    double       erBelow;
    bool         hasRefAbove;
    bool         hasRefBelow;
    bool         usingDefaults;   // true if εr was KiCad default 4.5
};
```

**Class** `STACKUP_READER`:
```
STACKUP_READER( const BOARD* aBoard )
LAYER_GEOMETRY GetLayerGeometry( PCB_LAYER_ID aLayer, const VECTOR2I& aPosition, double aTraceWidth )
```

Iterates `BOARD_STACKUP_ITEM` list, builds cumulative z-positions. For reference plane check: `ZONE::HitTestFilledArea( aLayer, aPosition )`.

**Analytical formulas** — `ANALYTICAL_IMPEDANCE`:
```
static double MicrostripZ0( double w, double h, double er, double t )
  // IPC-2141 / Hammerstad-Jensen:
  // Z0 = (87 / sqrt(er+1.41)) * ln(5.98*h / (0.8*w + t))

static double StriplineZ0( double w, double h, double er, double t )
  // Z0 = (60 / sqrt(er)) * ln(4*h / (0.67*pi*(0.8*w + t)))

static double EffectiveEr( double w, double h, double er )
  // Microstrip effective dielectric constant
```

Returns Z₀ per path segment. Segment boundaries at: width change, layer change (via), reference plane change.

**Test** — `qa/tests/pcbnew/test_stackup_reader.cpp`:
- Board with known stackup: verify extracted h, εr match design.
- Microstrip 50Ω trace (w=0.2mm on 0.2mm FR4): verify Z₀ ≈ 50Ω ±5%.
- Stripline: verify formula gives expected result.
- Board with default stackup: verify `usingDefaults` flag set.
- Trace with no zone below: verify `hasRefBelow = false`.

**Risks to discover**: Are zone fills current? Does `HitTestFilledArea` work on unfilled zones? Does `GetStackupDescriptor()` return sensible data on boards where the user hasn't configured the stackup?

---

## Stage 3: Bare-Bones Profiler Panel

**Goal**: First visible result. An AUI dockable panel in Pcbnew that shows Z₀ vs. distance for a selected net. Even if the numbers are approximate (analytical formulas), seeing a plot proves the whole pipeline works.

**Scope**:
- AUI panel registered in `PCB_EDIT_FRAME`
- Net selector dropdown (populated from board nets)
- "Analyse" button
- mpWindow plot: X = position (mm), Y = impedance (Ω)
- Horizontal target impedance line (manual input field)
- Via transition markers (vertical lines)
- Warning text area for stackup issues
- No click-to-highlight, no diff pair, no export yet

**Files to create**:
- `pcbnew/widgets/impedance_profiler_panel.h`
- `pcbnew/widgets/impedance_profiler_panel.cpp`

**Files to modify**:
- `pcbnew/pcb_edit_frame.h` — add panel member, `ToggleImpedanceProfiler()`
- `pcbnew/pcb_edit_frame.cpp` — construct panel (~line 297), AUI registration (~line 367), show/hide (~line 382)
- `pcbnew/pcbnew_settings.h` — add `show_impedance_profiler` to `AUI_PANELS`
- `pcbnew/pcbnew_settings.cpp` — add setting param
- `pcbnew/tools/pcb_actions.h/.cpp` — add `toggleImpedanceProfiler` action
- `pcbnew/tools/board_editor_control.cpp` — handle toggle
- `pcbnew/menubar_pcb_editor.cpp` — View menu entry
- `pcbnew/CMakeLists.txt` — add all new sipi/ sources + panel source

**Class** `IMPEDANCE_PROFILER_PANEL`:
```
Inherits WX_PANEL, BOARD_LISTENER
  OnBoardItemsChanged() → schedule re-analysis
  OnAnalyseClicked() → run pipeline

Pipeline:
  1. Get selected net from dropdown
  2. Find a track on that net, pick one end
  3. TRACE_PATH_WALKER::Walk() → ordered path
  4. For each segment: STACKUP_READER::GetLayerGeometry() → ANALYTICAL_IMPEDANCE::Z0()
  5. Plot Z₀ vs cumulative distance
```

**Panel layout** (vertical sizer):
```
[Net selector dropdown] [Analyse button] [Target Z₀: ___Ω]
[========== mpWindow impedance plot ==========]
[Warning/status text]
```

Plot features (reuse patterns from `pdn_panel.cpp`):
- `mpFXYVector` for impedance trace
- `mpFXYVector` for target impedance line
- Log or linear Y-axis (impedance range is usually 30-120Ω, linear is fine)
- Via markers as additional mpFXYVector vertical lines

**Test**: Manual — load a controlled-impedance board, analyse a net, verify plot shows reasonable values. Automated test of the engine pipeline (walker → stackup → Z₀) without GUI.

**Risks to discover**: AUI pane registration quirks, mpWindow rendering in pcbnew context (it works in eeschema — should work here too), event handling for net selection.

---

## Stage 4: BEM Cross-Section Solver

**Goal**: Replace the analytical formulas with a proper 2D BEM solver. Now that the pipeline is proven, this is a drop-in upgrade. Start simple (microstrip only), validate against the analytical results from Stage 2, then extend.

**Scope — Phase A** (microstrip, one reference plane):
- Single conductor over a single ground plane
- Method of images: one image charge below the plane
- 20 constant-charge panels per conductor edge (4 edges = 80 panels total)
- Dense matrix fill + `Eigen::PartialPivLU` solve
- Extract C → derive Z₀ = 1/(v_phase × C) where v_phase = c/√εr_eff
- Validate: Z₀ must agree with Stage 2 analytical to <3%

**Scope — Phase B** (stripline, extend):
- Two reference planes: image series (truncate at 20 terms)
- Coupled conductors (2 traces → 2×2 C matrix → Z_odd, Z_even, Z_diff)
- L via dual problem (ε=ε₀)
- R(f) via perturbation surface integral
- G(f) via ω·C·tan δ
- Cross-section cache keyed on geometry hash

**Requires**: Eigen dependency added to `pcbnew/CMakeLists.txt`

**Files**:
- `pcbnew/sipi/bem_2d_solver.h`
- `pcbnew/sipi/bem_2d_solver.cpp`
- `pcbnew/sipi/cross_section.h` — `XS_GEOMETRY`, `RLGC_RESULT` structs
- `pcbnew/sipi/cross_section.cpp`
- `pcbnew/sipi/xs_cache.h/.cpp`

**Integration**: Add a flag or config option in the profiler panel to switch between analytical and BEM. This lets us A/B compare and provides a fallback.

**Test** — `qa/tests/pcbnew/test_bem_2d_solver.cpp`:
- Microstrip: compare to IPC-2141, <2% error
- Stripline: compare to analytical, <1% error
- Coupled stripline Zdiff: compare to Wadell handbook, <2%
- Panel convergence: 10→20→40 panels, results converge
- Cache: solve same geometry twice, second is instant

---

## Stage 5: Diff Pair + Click-to-Highlight

**Goal**: Extend the profiler with the two most-requested features. Diff pair impedance profiling, and clicking on the plot to highlight the corresponding board location.

**Scope**:
- **Path walker extension**: `FindDiffPairComplement()` via net name suffix matching (`+`/`-`, `_P`/`_N`)
- **Diff pair profiling**: walk both nets, align by arc-length, compute Z_diff from coupled BEM (or from analytical odd-mode formula as fallback)
- **Click-to-highlight**: maintain a `(x_pixel_range → PATH_POINT)` lookup table, rebuilt on resize/replot. On `wxEVT_LEFT_DOWN`, find the segment, call `PCB_SELECTION_TOOL` to highlight it.
- **Color coding**: green (within ±10% of target), amber (±10-20%), red (>±20%)

**Files to modify**:
- `pcbnew/sipi/trace_path_walker.h/.cpp` — add `FindDiffPairComplement()`
- `pcbnew/widgets/impedance_profiler_panel.h/.cpp` — diff pair UI toggle, click handler, color coding

---

## Stage 6: Via Model (Tier 1+2 Analytical)

**Goal**: Via impedance for the channel model. Pure analytical — Goldfarb-Pucel for L, coaxial for C. Optionally Simonovich TL model for higher frequencies.

**Scope**:
- `VIA_MODEL::SolveAnalytical()` — L, C, R from geometry + stackup
- `VIA_MODEL::SolveSimonovichTL()` — coaxial TL model, captures stub resonance
- Extract via geometry from board: drill, pad diameter, antipad from zone data, layer span from stackup
- Stub length detection and resonance frequency warning

**Files**:
- `pcbnew/sipi/via_model.h`
- `pcbnew/sipi/via_model.cpp`

**Test** — `qa/tests/pcbnew/test_via_model.cpp`:
- Standard via (0.3mm drill, 1.6mm board): L ≈ 0.8-1.2 nH
- Tier 1 vs Tier 2 agree within 5% at 1 GHz
- Stub resonance frequency matches c/(4·l_stub·√εr)

---

## Stage 7: ABCD Cascade + Touchstone Export

**Goal**: Build a channel model from trace RLGC + via pi-models. Frequency sweep. Export Touchstone .s2p/.s4p.

**Scope**:
- `ABCD_MATRIX` class (2×2 complex): TL segment, shunt Y, series Z, cascade multiply, convert to S-params
- `CHANNEL_BUILDER`: walks a path, builds section list (trace segments + vias), evaluates ABCD cascade at each frequency
- `TOUCHSTONE_EXPORTER`: writes .s2p/.s4p in standard format
- For diff pair: 4×4 ABCD with modal decomposition, mixed-mode S-param conversion

**Files**:
- `pcbnew/sipi/abcd_matrix.h/.cpp`
- `pcbnew/sipi/channel_builder.h/.cpp`
- `pcbnew/sipi/touchstone_export.h/.cpp`

**Test** — `qa/tests/pcbnew/test_channel_builder.cpp`:
- Single TL: |S21| = exp(−αl), phase = βl
- AD − BC = 1 (reciprocity)
- Touchstone export round-trip

---

## Dependency Graph

```
Stage 1 (path walker) ──┬──> Stage 3 (panel MVP) ──> Stage 5 (diff pair + click)
                         │         ↑
Stage 2 (stackup + Z₀) ─┘         │
                                   │
Stage 4 (BEM solver) ─────────────┘
                         │
Stage 6 (via model) ─────┤
                         │
                         └──> Stage 7 (ABCD + Touchstone)
```

**MVP = Stages 1-3**: Visible impedance profiler with analytical Z₀. Exercises all integration points.

**Stages 4-5**: Upgrade accuracy and usability.

**Stages 6-7**: Channel analysis capability.

---

## What's Deferred

These are real features but not MVP. We'll add them when the foundation is proven:

- **FDFD plane cavity solver** (L3) — large effort, independent of profiler
- **PDN cavity → eeschema integration** — depends on FDFD
- **PEEC via extraction** (FastHenry2/FastCap2) — dropped; analytical is sufficient for now
- **PCB_ARC handling** in path walker — treat as chord initially
- **T-junction / stub detection** — stop at junctions initially
- **Cross-section neighbor coupling** — BEM handles it in Stage 4B, but not in MVP
- **CSV export, statistics table** — polish after core works
- **Surface roughness models** (Huray, Hammerstad-Jensen) — add to BEM later
- **Channel assembly tool** (registered PCB_TOOL_BASE) — Touchstone export from Stage 7 is enough initially

---

## Build System

New source group in `pcbnew/CMakeLists.txt`:
```cmake
set( PCBNEW_SIPI_SRCS
    sipi/trace_path_walker.cpp
    sipi/stackup_reader.cpp
    sipi/analytical_impedance.cpp
    sipi/cross_section.cpp          # Stage 4
    sipi/bem_2d_solver.cpp          # Stage 4
    sipi/xs_cache.cpp               # Stage 4
    sipi/via_model.cpp              # Stage 6
    sipi/abcd_matrix.cpp            # Stage 7
    sipi/channel_builder.cpp        # Stage 7
    sipi/touchstone_export.cpp      # Stage 7
    widgets/impedance_profiler_panel.cpp  # Stage 3
)
```

Eigen dependency added in Stage 4 (not needed for Stages 1-3).

---

## Verification Checkpoints

| After Stage | What you can verify |
|-------------|-------------------|
| 1 | Load a board, walk a net, print path length — matches ruler measurement? |
| 2 | Print Z₀ per segment — matches online impedance calculator? |
| 3 | See a plot in pcbnew — does it look right? Does stackup warning appear for unconfigured boards? |
| 4 | BEM Z₀ agrees with analytical ±2%? Convergence with panel count? |
| 5 | Click on plot → correct segment highlights? Diff pair shows Zdiff? |
| 6 | Via L matches published values? Stub resonance flagged? |
| 7 | Touchstone opens in external tool (e.g., scikit-rf)? S21 magnitude reasonable? |
