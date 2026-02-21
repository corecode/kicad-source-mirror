# Implementation Plan: Power Distribution Chart for KiCad Eeschema

## 1. Executive Summary

This plan describes how to add a **Power Distribution Chart** feature to KiCad's schematic editor (eeschema). The chart will provide a visual, hierarchical representation of how power flows through a design—from input sources through regulators/converters to load rails—enabling engineers to verify power budgets, identify over-loaded rails, and document power architecture at a glance.

**Target codebase:** KiCad 9.0.x (master branch, commit-based; equivalent approach works for KiCad 10 with no architectural differences—see Section 10).

---

## 2. Feature Overview

### What It Does

The Power Distribution Chart:

1. **Discovers power sources** — Identifies power input connectors, batteries, and external supply points by analyzing power flag symbols (`PWR_FLAG`), power symbols (e.g., `+5V`, `+3V3`), and user-annotated fields.

2. **Discovers regulators/converters** — Identifies voltage regulators (LDOs, DC-DC converters, charge pumps) by matching component library names against a configurable pattern database and by reading user-defined fields like `Power_Type`, `Input_Voltage`, `Output_Voltage`, and `Max_Current`.

3. **Traces power nets** — Follows net connections from source outputs through regulator inputs/outputs to build a directed power tree.

4. **Computes load budgets** — Sums estimated current consumption for each rail based on component fields (`Current_Draw`, `Typical_Current`) or user-provided defaults.

5. **Renders a tree chart** — Displays the power distribution as a visual tree/Sankey-style diagram in a dedicated panel, with:
   - Rail voltages and names
   - Current budget (available vs. consumed) with color-coded warnings
   - Hierarchical parent-child relationships (e.g., `VIN → 5V_REG → +5V → 3V3_REG → +3V3`)
   - Click-to-navigate: clicking a node highlights the corresponding component in the schematic

### What It Does NOT Do

- **No SPICE simulation** — This is a static analysis tool based on component metadata, not a circuit simulator.
- **No PCB-level analysis** — No IR-drop, copper thickness, or trace resistance calculations (that's the domain of a PDN analyzer).
- **No automatic current estimation from datasheets** — Users must annotate components with current draw fields; the tool does not scrape datasheets.

---

## 3. Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    SCH_EDIT_FRAME                            │
│  ┌───────────────────────────────────────────────────────┐  │
│  │              POWER_DISTRIBUTION_PANEL                  │  │
│  │  (wxPanel, docked in right or bottom panel area)      │  │
│  │  ┌─────────────────────────────────────────────────┐  │  │
│  │  │          POWER_CHART_CANVAS                     │  │  │
│  │  │  (Custom wxPanel with owner-draw rendering)     │  │  │
│  │  └─────────────────────────────────────────────────┘  │  │
│  │  ┌──────────────┐  ┌──────────────┐                   │  │
│  │  │ Toolbar/Opts │  │ Summary Bar  │                   │  │
│  │  └──────────────┘  └──────────────┘                   │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  Underlying data model (not UI):                            │
│  ┌───────────────────────────────────────────────────────┐  │
│  │           POWER_DISTRIBUTION_MODEL                    │  │
│  │  - std::vector<POWER_SOURCE>    m_sources             │  │
│  │  - std::vector<POWER_REGULATOR> m_regulators          │  │
│  │  - std::vector<POWER_RAIL>      m_rails               │  │
│  │  - std::vector<POWER_LOAD>      m_loads               │  │
│  │  - POWER_TREE_NODE*             m_rootNode             │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  Analysis engine:                                           │
│  ┌───────────────────────────────────────────────────────┐  │
│  │         POWER_DISTRIBUTION_ANALYZER                   │  │
│  │  - Walks SCH_SHEET_LIST, SCH_SYMBOL, SCH_FIELD       │  │
│  │  - Classifies components as sources/regulators/loads  │  │
│  │  - Builds net-based power tree via CONNECTION_GRAPH   │  │
│  │  - Computes current budgets per rail                  │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### Module Breakdown

| Module | Location | Purpose |
|--------|----------|---------|
| `POWER_DISTRIBUTION_ANALYZER` | `eeschema/power_distribution/power_distribution_analyzer.{h,cpp}` | Schematic analysis: walks all sheets, classifies components, builds power tree |
| `POWER_DISTRIBUTION_MODEL` | `eeschema/power_distribution/power_distribution_model.{h,cpp}` | Data structures: `POWER_SOURCE`, `POWER_REGULATOR`, `POWER_RAIL`, `POWER_LOAD`, `POWER_TREE_NODE` |
| `POWER_DISTRIBUTION_PANEL` | `eeschema/power_distribution/power_distribution_panel.{h,cpp}` | UI panel: hosts chart canvas, toolbar, summary |
| `POWER_CHART_CANVAS` | `eeschema/power_distribution/power_chart_canvas.{h,cpp}` | Custom rendering of the power tree diagram |
| `POWER_DISTRIBUTION_SETTINGS` | `eeschema/power_distribution/power_distribution_settings.{h,cpp}` | Configurable pattern DB, field name mappings, display preferences |
| Integration | `eeschema/sch_edit_frame.cpp`, `eeschema/toolbars_sch_editor.cpp` | Menu item, toolbar button, panel registration |

---

## 4. Detailed Design

### 4.1 Data Model (`power_distribution_model.h`)

```cpp
#ifndef POWER_DISTRIBUTION_MODEL_H
#define POWER_DISTRIBUTION_MODEL_H

#include <wx/string.h>
#include <vector>
#include <memory>

// Forward declarations
class SCH_SYMBOL;
class SCH_SHEET_PATH;

// Represents a power input source (connector, battery, test point)
struct POWER_SOURCE
{
    wxString         m_name;           // User-friendly name (e.g., "USB_VBUS")
    wxString         m_netName;        // Net name this source drives
    double           m_voltage;        // Nominal voltage (V)
    double           m_maxCurrent;     // Maximum available current (A)
    SCH_SYMBOL*      m_symbol;         // Back-pointer to schematic symbol
    SCH_SHEET_PATH   m_sheetPath;      // Sheet where this source lives
};

// Represents a voltage regulator or DC-DC converter
struct POWER_REGULATOR
{
    wxString         m_name;           // Reference designator + value
    wxString         m_type;           // "LDO", "Buck", "Boost", "Buck-Boost", "Charge Pump"
    wxString         m_inputNet;       // Input power net
    wxString         m_outputNet;      // Output power net
    double           m_inputVoltage;   // Input voltage (V)
    double           m_outputVoltage;  // Output voltage (V)
    double           m_maxOutputCurrent; // Max output current (A)
    double           m_efficiency;     // Conversion efficiency (0.0-1.0), default 1.0 for LDO
    SCH_SYMBOL*      m_symbol;
    SCH_SHEET_PATH   m_sheetPath;
};

// Represents a power rail (a net carrying a specific voltage)
struct POWER_RAIL
{
    wxString         m_netName;        // Net name (e.g., "+3V3")
    double           m_voltage;        // Nominal voltage (V)
    double           m_availableCurrent; // Total current available on this rail (A)
    double           m_consumedCurrent;  // Sum of all loads on this rail (A)
    double           m_utilizationPct;   // consumedCurrent / availableCurrent * 100
};

// Represents a current-consuming load on a rail
struct POWER_LOAD
{
    wxString         m_name;           // Reference designator
    wxString         m_netName;        // Power net it consumes from
    double           m_currentDraw;    // Estimated current draw (A)
    SCH_SYMBOL*      m_symbol;
    SCH_SHEET_PATH   m_sheetPath;
};

// Tree node for hierarchical power distribution visualization
struct POWER_TREE_NODE
{
    enum NODE_TYPE { SOURCE, REGULATOR, RAIL, LOAD };

    NODE_TYPE        m_type;
    wxString         m_label;          // Display label
    double           m_voltage;
    double           m_currentAvailable;
    double           m_currentConsumed;
    SCH_SYMBOL*      m_symbol;         // May be nullptr for virtual nodes
    SCH_SHEET_PATH   m_sheetPath;

    std::vector<std::unique_ptr<POWER_TREE_NODE>> m_children;
    POWER_TREE_NODE* m_parent = nullptr;
};

// Top-level model container
class POWER_DISTRIBUTION_MODEL
{
public:
    void Clear();
    bool IsEmpty() const;

    POWER_TREE_NODE*              GetRootNode() { return m_rootNode.get(); }
    std::vector<POWER_SOURCE>&    GetSources()    { return m_sources; }
    std::vector<POWER_REGULATOR>& GetRegulators() { return m_regulators; }
    std::vector<POWER_RAIL>&      GetRails()      { return m_rails; }
    std::vector<POWER_LOAD>&      GetLoads()      { return m_loads; }

    // Computed summaries
    double GetTotalPowerInput() const;    // Sum of source voltages * currents
    double GetTotalPowerConsumed() const; // Sum of load currents * rail voltages
    int    GetOverloadedRailCount() const;

private:
    std::unique_ptr<POWER_TREE_NODE>  m_rootNode;
    std::vector<POWER_SOURCE>         m_sources;
    std::vector<POWER_REGULATOR>      m_regulators;
    std::vector<POWER_RAIL>           m_rails;
    std::vector<POWER_LOAD>           m_loads;

    friend class POWER_DISTRIBUTION_ANALYZER;
};

#endif // POWER_DISTRIBUTION_MODEL_H
```

### 4.2 Analyzer (`power_distribution_analyzer.h/.cpp`)

The analyzer is the core engine. Its responsibilities:

#### 4.2.1 Component Classification

The analyzer walks every `SCH_SYMBOL` across all sheets (via `SCH_SHEET_LIST::GetAllSheetPaths()`) and classifies each component:

**Power Source Detection:**
- Symbols with a field named `Power_Type` set to `"Source"`, `"Input"`, or `"Supply"`
- Symbols whose library ID contains patterns like `Connector`, `Battery`, `USB`, `Barrel_Jack` (configurable)
- Symbols connected to `PWR_FLAG` on a net with no other driver

**Regulator Detection:**
- Symbols with a field named `Power_Type` set to `"Regulator"`, `"LDO"`, `"DCDC"`, `"Buck"`, `"Boost"`
- Symbols whose library ID matches configurable patterns: `Regulator_Linear`, `Regulator_Switching`, `LM317`, `LM7805`, `AP2112`, `TPS6*`, `LT308*`, etc.
- Must have identifiable input and output pins (by pin name patterns: `VIN`/`IN`/`INPUT` and `VOUT`/`OUT`/`OUTPUT`/`FB`)

**Load Detection:**
- Any symbol connected to a power rail that is not classified as a source or regulator
- Current draw is read from fields: `Current_Draw`, `Typical_Current`, `Max_Current`, `I_supply`
- If no current field is present, the load is listed with current = 0 (unknown) and flagged in the UI

#### 4.2.2 Net Tracing

Uses KiCad's existing `CONNECTION_GRAPH` (from `eeschema/connection_graph.h`) to:

1. Identify all nets driven by power sources
2. For each regulator, map `input_net → output_net`
3. Build a directed acyclic graph (DAG) of power flow: `Source → Net1 → Regulator → Net2 → Regulator → Net3 → Loads`

Key existing APIs leveraged:
- `CONNECTION_GRAPH::GetBusesAndNets()` — enumerate all nets
- `SCH_SYMBOL::GetPins()` — get pin-to-net mappings
- `SCH_PIN::GetNet()` / `SCH_PIN::Connection()` — net connectivity
- `SCH_SHEET_LIST::GetAllSheetPaths()` — cross-sheet traversal

#### 4.2.3 Power Tree Construction

```
Algorithm BuildPowerTree():
  1. For each POWER_SOURCE:
     - Create a SOURCE node
     - Find the output net
     - Create a RAIL node for that net
     - Find all POWER_REGULATORs whose input_net matches
     - For each regulator:
       - Create a REGULATOR node
       - Recurse: find its output net, create a RAIL node, find downstream regulators
     - Find all POWER_LOADs on this net
     - Attach as LOAD leaf nodes

  2. Compute current budgets bottom-up:
     - For each RAIL: sum all direct LOAD currents
     - For regulators: add the regulator's output rail consumption (divided by efficiency) to its input rail
     - Propagate up to sources

  3. Detect issues:
     - Rail overload: consumed > available
     - Unknown loads: components with no current annotation
     - Orphan rails: power nets with no identified source
```

#### 4.2.4 Field Reading Strategy

The analyzer reads component fields using `SCH_SYMBOL::GetFieldByName()`:

| Field Name | Purpose | Example Value |
|------------|---------|---------------|
| `Power_Type` | Classify component role | `"Source"`, `"LDO"`, `"Buck"`, `"Load"` |
| `Input_Voltage` | Regulator input voltage | `"12"` (volts) |
| `Output_Voltage` | Regulator output voltage | `"3.3"` (volts) |
| `Max_Current` | Max output current (source/regulator) | `"0.5"` (amps) |
| `Current_Draw` | Load current consumption | `"0.025"` (amps) |
| `Efficiency` | Regulator conversion efficiency | `"0.85"` (85%) |

These field names are **configurable** via the settings dialog (see Section 4.5). The defaults above are chosen to be intuitive and not conflict with existing KiCad conventions.

When fields are absent, the analyzer falls back to:
1. Parsing the `Value` field for voltage hints (e.g., `"LM7805"` → 5V output, `"AP2112K-3.3"` → 3.3V output)
2. Reading the net name for voltage hints (e.g., `"+5V"` → 5.0V, `"+3V3"` → 3.3V)
3. Marking unknowns for user review

### 4.3 UI Panel (`power_distribution_panel.h/.cpp`)

#### Panel Registration

The panel is added to eeschema's AUI (Advanced User Interface) manager as a dockable panel, similar to existing panels like the Properties Panel or Hierarchy Navigator.

Integration point in `sch_edit_frame.cpp`:
```cpp
// In SCH_EDIT_FRAME::setupUIConditions() or similar init
m_powerDistPanel = new POWER_DISTRIBUTION_PANEL( this );
m_auimgr.AddPane( m_powerDistPanel,
    EDA_PANE().Right().Name( "PowerDistribution" )
    .Caption( _( "Power Distribution" ) ).Hide() );
```

Menu integration in `toolbars_sch_editor.cpp`:
- **Inspect menu** → "Power Distribution Chart..." (alongside existing "Net Navigator", "Electrical Rules Checker")
- **Toolbar** → Optional icon button in the right toolbar

#### Panel Layout

```
┌──────────────────────────────────────────────┐
│ [Refresh] [Settings] [Export]    Filter: [__] │
├──────────────────────────────────────────────┤
│                                              │
│  ┌─ USB_VBUS (5.0V) ──── 500mA ──────────┐ │
│  │                                         │ │
│  │  ├─ U1: LM1117-3.3 (LDO)              │ │
│  │  │  └─ +3V3 (3.3V) ──── 300mA         │ │
│  │  │     ├─ U2: MCU ──── 50mA            │ │
│  │  │     ├─ U3: Flash ──── 25mA          │ │
│  │  │     └─ [... 4 more loads: 120mA]    │ │
│  │  │                                      │ │
│  │  ├─ U4: TPS62160 (Buck)                │ │
│  │  │  └─ +1V8 (1.8V) ──── 200mA         │ │
│  │  │     └─ U2: MCU Core ──── 150mA      │ │
│  │  │                                      │ │
│  │  └─ Direct loads: 30mA                 │ │
│  └─────────────────────────────────────────┘ │
│                                              │
├──────────────────────────────────────────────┤
│ Total: 0.95W consumed / 2.5W available       │
│ ⚠ 1 rail near capacity (>80%)               │
└──────────────────────────────────────────────┘
```

#### Rendering Approach

The chart uses a **custom owner-drawn wxPanel** (`POWER_CHART_CANVAS`) with `wxDC` drawing, similar to how KiCad's schematic canvas works but much simpler. This approach:
- Avoids adding new library dependencies
- Is consistent with KiCad's existing rendering patterns
- Supports themes (uses `COLOR_SETTINGS` for colors)

**Color coding:**
- Green: Rail utilization < 60%
- Yellow: Rail utilization 60-80%
- Orange: Rail utilization 80-95%
- Red: Rail utilization > 95% (overloaded)
- Gray: Unknown (missing current data)

**Interactions:**
- **Click on node** → Navigates to the component in the schematic (`SCH_EDIT_FRAME::FocusOnItem()`)
- **Hover** → Tooltip with detailed info (voltage, current, efficiency, component reference)
- **Right-click** → Context menu: "Edit Symbol Fields...", "Show in Schematic", "Copy Rail Summary"
- **Mouse wheel** → Scroll through large trees
- **Collapse/expand** → Toggle subtree visibility for complex designs

### 4.4 Schematic Integration

#### How Users Annotate Components

Users add power metadata to their symbols via **standard KiCad symbol fields**. No changes to the schematic file format are required. Users can add fields through:

1. **Symbol Properties dialog** (double-click a component → Add Field)
2. **Symbol Fields Table** (Tools → Edit Symbol Fields... → add column)
3. **Library symbols** (pre-populate fields in library symbols for common regulators)

Example: To annotate an LM1117-3.3:
```
Reference: U1
Value: LM1117-3.3
Footprint: Package_TO_SOT_SMD:SOT-223-3_TabPin2
Power_Type: LDO
Output_Voltage: 3.3
Max_Current: 0.8
Efficiency: 1.0
```

#### Round-Trip Safety

This feature is **fully round-trip safe**:
- It only **reads** existing schematic data; it never modifies the schematic
- It uses standard `SCH_FIELD` access (`GetFieldByName`, `GetFieldText`) which are read-only queries
- The power distribution model is computed fresh on each "Refresh" and is not serialized into the `.kicad_sch` file
- No changes to the schematic file format (`.kicad_sch` S-expression) are needed
- No changes to the project settings format (`.kicad_pro`) are needed beyond adding a settings section for the panel preferences

### 4.5 Settings (`power_distribution_settings.h/.cpp`)

Stored in the project settings JSON (`.kicad_pro`) under a new `"power_distribution"` key:

```json
{
  "power_distribution": {
    "field_names": {
      "power_type": "Power_Type",
      "input_voltage": "Input_Voltage",
      "output_voltage": "Output_Voltage",
      "max_current": "Max_Current",
      "current_draw": "Current_Draw",
      "efficiency": "Efficiency"
    },
    "source_patterns": [
      "Connector*", "Battery*", "USB*", "Barrel_Jack*",
      "TestPoint*Power*"
    ],
    "regulator_patterns": [
      "Regulator_Linear*", "Regulator_Switching*",
      "LM78*", "LM117*", "LM317*", "AP2112*", "AMS1117*",
      "TPS6*", "TPS54*", "TPS82*", "LT308*", "LT176*",
      "MP2*", "RT6*", "SY8*", "MIC29*"
    ],
    "default_load_current_mA": 0,
    "voltage_net_patterns": {
      "\\+([0-9.]+)V": "voltage_from_name",
      "\\+([0-9]+)V([0-9]+)": "voltage_with_decimal"
    },
    "warning_threshold_pct": 80,
    "critical_threshold_pct": 95
  }
}
```

A settings dialog (accessible from the panel toolbar) allows editing these values.

### 4.6 Export

The panel supports exporting the power distribution data:

1. **CSV export** — Table format: Rail, Voltage, Available_mA, Consumed_mA, Utilization%
2. **Text report** — Human-readable tree format (similar to the panel display)
3. **Clipboard copy** — Copy selected node or full tree summary

---

## 5. Implementation Steps

### Phase 1: Data Model and Analyzer (Core Logic)

**Files to create:**
- `eeschema/power_distribution/power_distribution_model.h`
- `eeschema/power_distribution/power_distribution_model.cpp`
- `eeschema/power_distribution/power_distribution_analyzer.h`
- `eeschema/power_distribution/power_distribution_analyzer.cpp`
- `eeschema/power_distribution/power_distribution_settings.h`
- `eeschema/power_distribution/power_distribution_settings.cpp`

**Files to modify:**
- `eeschema/CMakeLists.txt` — Add new source files to the build

**Steps:**
1. Create the `eeschema/power_distribution/` directory
2. Implement `POWER_DISTRIBUTION_MODEL` with all data structures
3. Implement `POWER_DISTRIBUTION_SETTINGS` with default patterns and field mappings
4. Implement `POWER_DISTRIBUTION_ANALYZER`:
   a. `AnalyzeSchematic(SCHEMATIC* aSchematic)` — main entry point
   b. `ClassifySymbol(SCH_SYMBOL*, SCH_SHEET_PATH&)` — source/regulator/load classification
   c. `BuildNetGraph()` — map nets to components using `CONNECTION_GRAPH`
   d. `BuildPowerTree()` — construct hierarchical tree from net graph
   e. `ComputeCurrentBudgets()` — bottom-up current summation
   f. `DetectIssues()` — identify overloads and unknowns
5. Add unit tests for the analyzer logic

### Phase 2: UI Panel and Chart Rendering

**Files to create:**
- `eeschema/power_distribution/power_distribution_panel.h`
- `eeschema/power_distribution/power_distribution_panel.cpp`
- `eeschema/power_distribution/power_chart_canvas.h`
- `eeschema/power_distribution/power_chart_canvas.cpp`

**Files to modify:**
- `eeschema/CMakeLists.txt` — Add UI source files
- `eeschema/sch_edit_frame.h` — Add `m_powerDistPanel` member
- `eeschema/sch_edit_frame.cpp` — Register panel with AUI manager
- `eeschema/toolbars_sch_editor.cpp` — Add menu entry under Inspect menu

**Steps:**
1. Implement `POWER_CHART_CANVAS`:
   a. Tree layout algorithm (recursive, top-down with horizontal branching)
   b. Node rendering (rounded rectangles with voltage, current, utilization bar)
   c. Edge rendering (lines/arrows connecting parent to children)
   d. Color coding based on utilization thresholds
   e. Hit testing for click-to-navigate
   f. Scroll and collapse/expand support
2. Implement `POWER_DISTRIBUTION_PANEL`:
   a. Toolbar with Refresh, Settings, Export buttons
   b. Host `POWER_CHART_CANVAS`
   c. Summary bar at bottom
   d. Wire up Refresh to re-run analyzer
3. Register the panel in `SCH_EDIT_FRAME`
4. Add menu item: **Inspect → Power Distribution Chart**

### Phase 3: Settings Dialog and Export

**Files to create:**
- `eeschema/power_distribution/dialog_power_distribution_settings.h`
- `eeschema/power_distribution/dialog_power_distribution_settings.cpp`
- `eeschema/power_distribution/dialog_power_distribution_settings_base.h` (wxFormBuilder)
- `eeschema/power_distribution/dialog_power_distribution_settings_base.cpp` (wxFormBuilder)

**Steps:**
1. Create the settings dialog (field name mapping, pattern lists, thresholds)
2. Implement CSV and text report export
3. Implement clipboard copy for selected nodes
4. Wire settings button in panel toolbar to settings dialog

### Phase 4: Polish and Testing

**Steps:**
1. Test with demo schematics (`demos/vme-wren/power-supply-*.kicad_sch`, `demos/royalblue54L_feather/sch/nPM1300.kicad_sch`)
2. Handle edge cases:
   - Multi-sheet designs with power distributed across sheets
   - Hierarchical pins carrying power between sheets
   - Multiple sources on the same net
   - Feedback loops (e.g., boost converter output fed back)
   - No annotated components (graceful empty state)
3. Theme support (light/dark mode colors from `COLOR_SETTINGS`)
4. Localization (all strings wrapped in `_()` for i18n)
5. Add automated tests for analyzer and model

---

## 6. Key Existing Code to Leverage

| Existing Code | File | How We Use It |
|---------------|------|---------------|
| `SCH_SYMBOL` | `eeschema/sch_symbol.h` | Read symbol data, fields, library ID |
| `SCH_FIELD::GetText()` | `eeschema/sch_field.h` | Read field values for power metadata |
| `SCH_SYMBOL::GetFieldByName()` | `eeschema/sch_symbol.h` | Look up custom power fields |
| `SCH_SHEET_LIST` | `eeschema/sch_sheet_path.h` | Traverse all sheets in design |
| `CONNECTION_GRAPH` | `eeschema/connection_graph.h` | Net connectivity information |
| `SCH_CONNECTION` | `eeschema/sch_connection.h` | Per-item net connection data |
| `SCHEMATIC` | `eeschema/schematic.h` | Top-level schematic access |
| `SCH_EDIT_FRAME` | `eeschema/sch_edit_frame.h` | Frame integration, `FocusOnItem()` |
| `EDA_PANE` | `include/widgets/eda_pane.h` | AUI panel helper |
| `COLOR_SETTINGS` | `common/settings/color_settings.h` | Theme colors |
| `NET_NAVIGATOR_PANEL` | `eeschema/widgets/net_navigator.h` | Pattern for panel implementation |
| `HIERARCHY_PANE` | `eeschema/widgets/hierarchy_pane.h` | Pattern for tree-style panel |

---

## 7. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Users don't annotate fields → empty chart | High | Medium | Auto-detect from library names and net names; show clear guidance for missing data |
| Complex designs with 100+ regulators → slow analysis | Low | Medium | Analysis is O(n) in symbols; limit tree depth; lazy rendering |
| `CONNECTION_GRAPH` not available when panel opens | Medium | High | Re-run `RecalculateConnections()` before analysis if needed; cache results |
| Regulator input/output pin identification fails | Medium | Medium | Fall back to user `Power_Type` field; allow manual override in settings |
| Circular power dependencies (rare, invalid designs) | Low | Low | DAG cycle detection with graceful error message |

---

## 8. Testing Strategy

1. **Unit tests** (`qa/tests/eeschema/test_power_distribution_analyzer.cpp`):
   - Test component classification with mock symbols
   - Test power tree construction with known topologies
   - Test current budget computation (simple chain, branching tree, multi-source)
   - Test pattern matching for source/regulator detection

2. **Integration tests** with demo schematics:
   - `demos/vme-wren/` — Complex multi-sheet design with multiple power supplies
   - `demos/royalblue54L_feather/` — Modern design with nPM1300 PMIC
   - Create a dedicated test schematic with annotated power fields

3. **Manual testing**:
   - Verify click-to-navigate highlights correct component
   - Verify panel updates on schematic changes (add/remove component)
   - Verify export formats are correct
   - Verify dark/light theme rendering

---

## 9. File Summary

### New Files (14)

```
eeschema/power_distribution/
├── power_distribution_model.h
├── power_distribution_model.cpp
├── power_distribution_analyzer.h
├── power_distribution_analyzer.cpp
├── power_distribution_settings.h
├── power_distribution_settings.cpp
├── power_distribution_panel.h
├── power_distribution_panel.cpp
├── power_chart_canvas.h
├── power_chart_canvas.cpp
├── dialog_power_distribution_settings.h
├── dialog_power_distribution_settings.cpp
├── dialog_power_distribution_settings_base.h
└── dialog_power_distribution_settings_base.cpp

qa/tests/eeschema/
└── test_power_distribution_analyzer.cpp
```

### Modified Files (4)

```
eeschema/CMakeLists.txt          — Add new source files
eeschema/sch_edit_frame.h        — Add panel member pointer
eeschema/sch_edit_frame.cpp      — Register panel with AUI
eeschema/toolbars_sch_editor.cpp — Add Inspect menu entry
```

---

## 10. KiCad 9 vs KiCad 10 Considerations

Based on thorough research (see separate analysis), **there is no meaningful advantage to waiting for KiCad 10** for this feature:

- Neither version has native power analysis capabilities
- Neither version has a schematic IPC API (targeted for KiCad 11)
- The schematic data model differences are minimal (KiCad 10 adds variants and local power symbols, neither relevant here)
- The UI framework and panel system are identical
- The `CONNECTION_GRAPH`, `SCH_SYMBOL`, and `SCH_FIELD` APIs are unchanged
- Both use wxWidgets with the same AUI panel infrastructure

The implementation described in this plan works identically on both KiCad 9 and 10. The only code difference would be the branch point and any minor API signature changes in imported headers.

**Recommendation:** Target the current codebase (master/9.x) and forward-port to 10.x if needed. The forward port should be trivial given the architectural stability.

---

## 11. Future Extensions (Out of Scope)

These are documented for future consideration but are **not part of this implementation**:

1. **SPICE-backed validation** — Run `.op` simulation to verify static power analysis against SPICE results
2. **PCB copper analysis** — Integrate with PCB editor for trace width / via current capacity checks
3. **BOM integration** — Auto-populate power fields from component database / BOM tools
4. **Power sequencing** — Visualize power-up/power-down ordering constraints
5. **Thermal estimation** — Calculate regulator power dissipation and thermal margins
6. **IPC API exposure** — When the eeschema IPC API ships (KiCad 11+), expose power distribution data through it
