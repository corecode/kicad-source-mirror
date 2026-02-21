# Implementation Plan: Power Distribution Chart for KiCad Eeschema (Revised)

## 1. Executive Summary

This plan describes how to add a **Power Distribution Chart** feature to KiCad's schematic editor (eeschema). Instead of building a custom wxPanel visualization, the chart is **generated directly as schematic drawing items on a dedicated sheet** — making it printable, exportable to PDF, versionable in git, and fully native to KiCad's existing rendering/editing/export pipeline.

Additionally, new **ERC (Electrical Rules Check) rules** flag power-related issues (overloaded rails, missing annotations, orphan power nets), integrating power analysis into the familiar ERC workflow.

**Target codebase:** KiCad 9.0.x (master branch).

---

## 2. Approach: Sheet-Based Chart + ERC Integration

### Why a Sheet Instead of a Custom Panel

| Sheet-based (this plan) | Custom wxPanel (previous plan) |
|-------------------------|-------------------------------|
| Printable / PDF export for free | Needs custom export code |
| Editable — users can annotate the chart | Read-only |
| Saved in `.kicad_sch` — versioned in git | Transient, recomputed each session |
| Uses existing renderer, plotter, search | Custom drawing code |
| No new UI framework dependencies | Needs owner-drawn wxPanel |
| Visible in hierarchy navigator | Hidden panel |
| Works with all existing tools (zoom, pan, print, plot) | Needs reimplementation |

### What Users See

1. **Inspect → Generate Power Distribution Chart** menu item
2. A new hierarchical sheet is created (or updated) named "Power Distribution"
3. The sheet contains:
   - A **summary table** at the top (total power input, consumed, rail count)
   - A **power tree diagram** drawn with lines, rectangles, and text showing the hierarchy from sources through regulators to rails and loads
   - **Color-coded rail status** rectangles (green/yellow/orange/red based on utilization)
   - **Per-rail budget tables** showing loads on each rail
4. Running **ERC** additionally checks for power-related issues and reports them as standard ERC markers

### How It Updates

- User triggers **Inspect → Generate Power Distribution Chart** (or a toolbar button)
- The analyzer reads all annotations, rebuilds the model, and regenerates the sheet content
- The existing sheet is cleared and repopulated (the sheet file is overwritten)
- The user can also just run ERC to get power warnings without regenerating the chart

---

## 3. Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    SCH_EDIT_FRAME                             │
│                                                              │
│  Analysis Engine:                                            │
│  ┌────────────────────────────────────────────────────────┐  │
│  │           POWER_DISTRIBUTION_ANALYZER                  │  │
│  │  - Walks SCH_SHEET_LIST, SCH_SYMBOL, SCH_FIELD        │  │
│  │  - Classifies components as sources/regulators/loads   │  │
│  │  - Builds net-based power tree via CONNECTION_GRAPH    │  │
│  │  - Computes current budgets per rail                   │  │
│  │  - Populates POWER_DISTRIBUTION_MODEL                  │  │
│  └────────────────────────────────────────────────────────┘  │
│                           │                                   │
│              ┌────────────┴────────────┐                     │
│              ▼                         ▼                     │
│  ┌─────────────────────┐  ┌──────────────────────┐          │
│  │  CHART GENERATOR    │  │  ERC POWER CHECKS    │          │
│  │  Writes SCH_TABLE,  │  │  New ERCE_* codes    │          │
│  │  SCH_TEXT, SCH_LINE, │  │  in ERC_TESTER       │          │
│  │  SCH_SHAPE onto a   │  │  Rail overload,      │          │
│  │  dedicated sheet     │  │  missing annotations │          │
│  └─────────────────────┘  └──────────────────────┘          │
│              │                         │                     │
│              ▼                         ▼                     │
│  ┌─────────────────────┐  ┌──────────────────────┐          │
│  │  "Power             │  │  Standard ERC        │          │
│  │   Distribution"     │  │  markers/violations  │          │
│  │  .kicad_sch sheet   │  │  in ERC dialog       │          │
│  └─────────────────────┘  └──────────────────────┘          │
└──────────────────────────────────────────────────────────────┘
```

### Module Breakdown

| Module | Location | Purpose |
|--------|----------|---------|
| `POWER_DISTRIBUTION_MODEL` | `eeschema/power_distribution/power_distribution_model.{h,cpp}` | Data structures for power tree |
| `POWER_DISTRIBUTION_ANALYZER` | `eeschema/power_distribution/power_distribution_analyzer.{h,cpp}` | Schematic analysis engine |
| `POWER_CHART_GENERATOR` | `eeschema/power_distribution/power_chart_generator.{h,cpp}` | Generates drawing items on sheet |
| `POWER_DISTRIBUTION_SETTINGS` | `eeschema/power_distribution/power_distribution_settings.{h,cpp}` | Configurable patterns and field names |
| ERC integration | `eeschema/erc/erc.{h,cpp}`, `erc_settings.h`, `erc_item.{h,cpp}` | New power-related ERC checks |
| Menu integration | `eeschema/menubar.cpp`, `eeschema/tools/sch_actions.{h,cpp}` | Menu item and action |
| Tool handler | `eeschema/tools/sch_editor_control.{h,cpp}` | Action handler for generate command |

---

## 4. Detailed Design

### 4.1 Data Model (`power_distribution_model.h`)

Same as previous plan — structs for `POWER_SOURCE`, `POWER_REGULATOR`, `POWER_RAIL`, `POWER_LOAD`, `POWER_TREE_NODE`, and the container `POWER_DISTRIBUTION_MODEL`. No changes needed.

```cpp
#ifndef POWER_DISTRIBUTION_MODEL_H
#define POWER_DISTRIBUTION_MODEL_H

#include <wx/string.h>
#include <vector>
#include <memory>

class SCH_SYMBOL;
class SCH_SHEET_PATH;

struct POWER_SOURCE
{
    wxString         m_name;
    wxString         m_netName;
    double           m_voltage;
    double           m_maxCurrent;    // Amps
    SCH_SYMBOL*      m_symbol;
    SCH_SHEET_PATH   m_sheetPath;
};

struct POWER_REGULATOR
{
    wxString         m_name;
    wxString         m_type;          // "LDO", "Buck", "Boost", etc.
    wxString         m_inputNet;
    wxString         m_outputNet;
    double           m_inputVoltage;
    double           m_outputVoltage;
    double           m_maxOutputCurrent;
    double           m_efficiency;    // 0.0-1.0
    SCH_SYMBOL*      m_symbol;
    SCH_SHEET_PATH   m_sheetPath;
};

struct POWER_RAIL
{
    wxString         m_netName;
    double           m_voltage;
    double           m_availableCurrent;
    double           m_consumedCurrent;
    double           m_utilizationPct;
};

struct POWER_LOAD
{
    wxString         m_name;
    wxString         m_netName;
    double           m_currentDraw;   // Amps
    SCH_SYMBOL*      m_symbol;
    SCH_SHEET_PATH   m_sheetPath;
};

struct POWER_TREE_NODE
{
    enum NODE_TYPE { SOURCE, REGULATOR, RAIL, LOAD };

    NODE_TYPE        m_type;
    wxString         m_label;
    double           m_voltage;
    double           m_currentAvailable;
    double           m_currentConsumed;
    SCH_SYMBOL*      m_symbol = nullptr;
    SCH_SHEET_PATH   m_sheetPath;

    std::vector<std::unique_ptr<POWER_TREE_NODE>> m_children;
    POWER_TREE_NODE* m_parent = nullptr;
};

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

    double GetTotalPowerInput() const;
    double GetTotalPowerConsumed() const;
    int    GetOverloadedRailCount() const;

private:
    std::unique_ptr<POWER_TREE_NODE>  m_rootNode;
    std::vector<POWER_SOURCE>         m_sources;
    std::vector<POWER_REGULATOR>      m_regulators;
    std::vector<POWER_RAIL>           m_rails;
    std::vector<POWER_LOAD>           m_loads;

    friend class POWER_DISTRIBUTION_ANALYZER;
};

#endif
```

### 4.2 Analyzer (`power_distribution_analyzer.h/.cpp`)

Unchanged from previous plan. Walks the schematic, classifies components, builds power tree, computes budgets. Uses `CONNECTION_GRAPH` for net tracing. See previous plan sections 4.2.1-4.2.4 for full detail.

### 4.3 Chart Generator (`power_chart_generator.h/.cpp`) — NEW

This is the key change. Instead of a custom wxPanel, we generate native schematic drawing items on a dedicated sheet.

```cpp
#ifndef POWER_CHART_GENERATOR_H
#define POWER_CHART_GENERATOR_H

class SCHEMATIC;
class SCH_SHEET;
class SCH_SCREEN;
class POWER_DISTRIBUTION_MODEL;
class POWER_TREE_NODE;

// Generates a power distribution chart as native schematic items on a sheet
class POWER_CHART_GENERATOR
{
public:
    POWER_CHART_GENERATOR( SCHEMATIC* aSchematic );

    // Main entry: create or update the power distribution sheet
    // Returns the sheet, or nullptr on error
    SCH_SHEET* Generate( const POWER_DISTRIBUTION_MODEL& aModel );

private:
    // Find or create the "Power Distribution" sub-sheet
    SCH_SHEET* getOrCreateSheet();

    // Clear all items from the sheet's screen
    void clearSheet( SCH_SCREEN* aScreen );

    // Layout sections
    void drawTitle( SCH_SCREEN* aScreen, const POWER_DISTRIBUTION_MODEL& aModel,
                    VECTOR2I& aCursor );

    void drawSummaryTable( SCH_SCREEN* aScreen, const POWER_DISTRIBUTION_MODEL& aModel,
                           VECTOR2I& aCursor );

    void drawPowerTree( SCH_SCREEN* aScreen, const POWER_TREE_NODE* aNode,
                        VECTOR2I aPos, int aDepth, int& aMaxY );

    void drawRailBox( SCH_SCREEN* aScreen, const POWER_TREE_NODE* aNode,
                      VECTOR2I aPos );

    void drawRegulatorBox( SCH_SCREEN* aScreen, const POWER_TREE_NODE* aNode,
                           VECTOR2I aPos );

    void drawLoadTable( SCH_SCREEN* aScreen, const POWER_TREE_NODE* aRailNode,
                        VECTOR2I aPos );

    void drawConnectionLine( SCH_SCREEN* aScreen, VECTOR2I aFrom, VECTOR2I aTo );

    // Helpers
    SCH_TABLE* createTable( int aCols, const std::vector<int>& aColWidths );
    SCH_TEXTBOX* createTextBox( const wxString& aText, VECTOR2I aPos, VECTOR2I aSize,
                                int aFontSize = 0 );

    SCHEMATIC*  m_schematic;
};

#endif
```

#### Chart Layout

The generator places items on the sheet in this layout:

```
┌─────────────────────────────────────────────────────────────────────┐
│  POWER DISTRIBUTION CHART                                           │
│  Generated: 2025-01-15  |  Project: MyBoard                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  SUMMARY                                                            │
│  ┌────────────────────────────────────────────────────┐             │
│  │ Total Input: 2.5W  │ Total Consumed: 1.8W │ Rails: 4 │         │
│  │ Overloaded: 0      │ Warnings: 1          │          │         │
│  └────────────────────────────────────────────────────┘             │
│                                                                     │
│  POWER TREE                                                         │
│                                                                     │
│  ┌──────────────┐                                                   │
│  │ USB_VBUS     │    ┌──────────────┐    ┌──────────────┐          │
│  │ 5.0V         │───▶│ U1: LM1117   │───▶│ +3V3         │          │
│  │ 500mA avail  │    │ LDO          │    │ 3.3V         │          │
│  └──────────────┘    │ η=100%       │    │ 195mA/300mA  │          │
│         │            └──────────────┘    │ ████████░░ 65%│          │
│         │                                └──────────────┘          │
│         │                                       │                   │
│         │                                ┌──────┴──────┐           │
│         │                                │ LOADS:       │           │
│         │                                │ U2 MCU  50mA│           │
│         │                                │ U3 Flash 25mA│          │
│         │                                │ R1-R10  120mA│          │
│         │                                └─────────────┘           │
│         │                                                           │
│         │            ┌──────────────┐    ┌──────────────┐          │
│         └───────────▶│ U4: TPS62160 │───▶│ +1V8         │          │
│                      │ Buck         │    │ 1.8V         │          │
│                      │ η=92%        │    │ 150mA/200mA  │          │
│                      └──────────────┘    │ ████████░░ 75%│          │
│                                          └──────────────┘          │
└─────────────────────────────────────────────────────────────────────┘
```

Items used:
- **SCH_TEXT** — Title, section headers, labels
- **SCH_SHAPE (RECTANGLE)** — Node boxes (source/regulator/rail), with fill color based on status
- **SCH_LINE (LAYER_NOTES)** — Connection arrows between nodes
- **SCH_TABLE** — Summary table, load lists per rail
- **SCH_TEXTBOX** — Multi-line info inside node boxes

#### Color Coding (via `SCH_SHAPE::SetFillColor()`)

| Utilization | Color | Meaning |
|-------------|-------|---------|
| 0-60% | Light green `#C8E6C9` | Healthy |
| 60-80% | Light yellow `#FFF9C4` | Watch |
| 80-95% | Light orange `#FFE0B2` | Warning |
| >95% | Light red `#FFCDD2` | Overloaded |
| Unknown | Light gray `#E0E0E0` | Missing data |

These are fill colors for the rail boxes. The border and text remain dark for readability.

### 4.4 ERC Integration — NEW

#### New ERC Error Codes

Add to `erc_settings.h` ERCE_T enum:

```cpp
ERCE_POWER_RAIL_OVERLOAD,         // Rail consumed current > available current
ERCE_POWER_MISSING_ANNOTATION,    // Regulator/source missing required fields
ERCE_POWER_UNKNOWN_LOAD_CURRENT,  // Load connected to power rail with no Current_Draw field
ERCE_POWER_ORPHAN_RAIL,           // Power net with no identified source
```

#### ERC Check Implementation

Add to `ERC_TESTER`:

```cpp
int TestPowerDistribution();
```

This method:
1. Instantiates `POWER_DISTRIBUTION_ANALYZER` and runs it
2. For each overloaded rail: creates `ERCE_POWER_RAIL_OVERLOAD` marker on the regulator/source driving that rail
3. For each regulator missing `Output_Voltage` or `Max_Current` fields: creates `ERCE_POWER_MISSING_ANNOTATION` marker
4. For each load on a power rail with no `Current_Draw` field: creates `ERCE_POWER_UNKNOWN_LOAD_CURRENT` marker
5. For each power net with no identified source: creates `ERCE_POWER_ORPHAN_RAIL` marker

**Key advantage:** ERC markers appear in the standard ERC dialog, can be navigated with Next/Prev Marker, can be excluded individually, and their severity is configurable (Error/Warning/Ignore) just like any other ERC check.

#### Registration in `erc_item.cpp`:

```cpp
ERC_ITEM ERC_ITEM::powerRailOverload( ERCE_POWER_RAIL_OVERLOAD,
        _HKI( "Power rail current consumption exceeds available current" ),
        wxT( "power_rail_overload" ) );

ERC_ITEM ERC_ITEM::powerMissingAnnotation( ERCE_POWER_MISSING_ANNOTATION,
        _HKI( "Power component missing required annotation fields" ),
        wxT( "power_missing_annotation" ) );

ERC_ITEM ERC_ITEM::powerUnknownLoadCurrent( ERCE_POWER_UNKNOWN_LOAD_CURRENT,
        _HKI( "Component on power rail has no current draw annotation" ),
        wxT( "power_unknown_load_current" ) );

ERC_ITEM ERC_ITEM::powerOrphanRail( ERCE_POWER_ORPHAN_RAIL,
        _HKI( "Power net has no identified power source" ),
        wxT( "power_orphan_rail" ) );
```

#### Default Severities:

| Check | Default Severity | Rationale |
|-------|-----------------|-----------|
| `ERCE_POWER_RAIL_OVERLOAD` | Error | Design may not work |
| `ERCE_POWER_MISSING_ANNOTATION` | Warning | Chart will be incomplete |
| `ERCE_POWER_UNKNOWN_LOAD_CURRENT` | Warning | Budget can't be computed |
| `ERCE_POWER_ORPHAN_RAIL` | Warning | May be intentional (off-board supply) |

### 4.5 Settings (`power_distribution_settings.h/.cpp`)

Same as previous plan. Stored in `.kicad_pro` under `"power_distribution"` key. Contains:
- Field name mappings (`Power_Type`, `Output_Voltage`, `Max_Current`, `Current_Draw`, `Efficiency`)
- Source patterns (`Connector*`, `Battery*`, `USB*`, etc.)
- Regulator patterns (`Regulator_Linear*`, `Regulator_Switching*`, etc.)
- Warning/critical thresholds (80%, 95%)
- Chart sheet name (default: `"Power Distribution"`)

### 4.6 Annotation Workflow

Users annotate their symbols using standard KiCad fields:

1. **Symbol Properties** → Add Field → `Power_Type` = `"LDO"`, `Output_Voltage` = `"3.3"`, etc.
2. **Symbol Fields Table** (Tools → Edit Symbol Fields...) — bulk edit across all symbols
3. **Library symbols** — pre-populate fields in library symbols for common regulators

The ERC checks nudge users toward completeness:
- Running ERC shows warnings like: *"U1 (LM1117-3.3): Power component missing required annotation fields (Output_Voltage, Max_Current)"*
- User adds the fields, reruns ERC — warning goes away
- User generates chart — now shows complete power tree

This creates a natural workflow: **Annotate → ERC → Fix → Generate Chart**.

---

## 5. Implementation Steps

### Phase 1: Data Model, Settings, and Analyzer

**Files to create:**
- `eeschema/power_distribution/power_distribution_model.h`
- `eeschema/power_distribution/power_distribution_model.cpp`
- `eeschema/power_distribution/power_distribution_analyzer.h`
- `eeschema/power_distribution/power_distribution_analyzer.cpp`
- `eeschema/power_distribution/power_distribution_settings.h`
- `eeschema/power_distribution/power_distribution_settings.cpp`

**Files to modify:**
- `eeschema/CMakeLists.txt`

### Phase 2: Chart Generator

**Files to create:**
- `eeschema/power_distribution/power_chart_generator.h`
- `eeschema/power_distribution/power_chart_generator.cpp`

**Files to modify:**
- `eeschema/CMakeLists.txt`

### Phase 3: ERC Integration

**Files to modify:**
- `eeschema/erc/erc_settings.h` — Add `ERCE_POWER_*` enum values
- `eeschema/erc/erc_item.h` — Add static ERC_ITEM declarations
- `eeschema/erc/erc_item.cpp` — Register items, add to allItemTypes, add to Create()
- `eeschema/erc/erc.h` — Add `TestPowerDistribution()` declaration
- `eeschema/erc/erc.cpp` — Implement `TestPowerDistribution()`, hook into `RunTests()`
- `eeschema/erc/erc_settings.cpp` — Set default severities

### Phase 4: Menu and Action Integration

**Files to modify:**
- `eeschema/tools/sch_actions.h` — Add `generatePowerDistChart` action
- `eeschema/tools/sch_actions.cpp` — Define the action
- `eeschema/tools/sch_editor_control.h` — Add handler declaration
- `eeschema/tools/sch_editor_control.cpp` — Implement handler
- `eeschema/menubar.cpp` — Add to Inspect menu

### Phase 5: Polish and Testing

- Test with demo schematics
- Handle edge cases (empty designs, circular dependencies, multi-source rails)
- Ensure chart sheet is properly saved/loaded
- Add unit tests

---

## 6. File Summary

### New Files (8)

```
eeschema/power_distribution/
├── power_distribution_model.h
├── power_distribution_model.cpp
├── power_distribution_analyzer.h
├── power_distribution_analyzer.cpp
├── power_distribution_settings.h
├── power_distribution_settings.cpp
├── power_chart_generator.h
└── power_chart_generator.cpp
```

### Modified Files (10)

```
eeschema/CMakeLists.txt              — Add new source files
eeschema/erc/erc_settings.h          — Add ERCE_POWER_* enum values
eeschema/erc/erc_item.h              — Add static ERC_ITEM declarations
eeschema/erc/erc_item.cpp            — Register power ERC items
eeschema/erc/erc.h                   — Add TestPowerDistribution()
eeschema/erc/erc.cpp                 — Implement power ERC checks, hook into RunTests()
eeschema/erc/erc_settings.cpp        — Set default severities for power checks
eeschema/tools/sch_actions.h         — Add generatePowerDistChart action
eeschema/tools/sch_actions.cpp       — Define the action
eeschema/tools/sch_editor_control.cpp — Implement action handler
eeschema/menubar.cpp                 — Add Inspect menu entry
```

---

## 7. Key Design Decisions

### Q: Why a sub-sheet and not items on the root sheet?
A: The chart can be large. A dedicated sub-sheet keeps it organized and avoids cluttering the user's design. The sheet appears in the hierarchy navigator and can be navigated to like any other sheet.

### Q: What happens when the user regenerates?
A: The chart sheet's screen is cleared and repopulated. All previous drawing items are removed. The sheet itself (position, size, name) is preserved. User annotations ON the chart sheet will be lost — but this is expected and documented, since the sheet is auto-generated.

### Q: Can users edit the chart manually?
A: Yes, since it's a regular schematic sheet. But edits will be overwritten on next generation. A warning text is placed on the sheet: "Auto-generated — edits will be overwritten on next generation."

### Q: How does this interact with undo?
A: The generation operation is wrapped in a `SCH_COMMIT` so it's a single undo step. The user can Ctrl+Z to revert the generation.

### Q: What if CONNECTION_GRAPH isn't built yet?
A: The handler calls `RecalculateConnections()` before running the analyzer, ensuring fresh connectivity data.

---

## 8. Round-Trip Safety

This feature is **fully round-trip safe**:
- The analyzer **only reads** existing schematic data
- The chart sheet is a standard `.kicad_sch` file — fully compatible with KiCad's file format
- ERC markers are standard `SCH_MARKER` objects — no format changes
- The chart can be deleted by simply removing the sub-sheet — no orphaned data
- No changes to the `.kicad_sch` S-expression format
- Settings stored in `.kicad_pro` under a new `"power_distribution"` key — ignored by older KiCad versions

---

## 9. ERC Error Messages (Examples)

**ERCE_POWER_RAIL_OVERLOAD:**
> Power rail '+3V3' overloaded: 350mA consumed / 300mA available (117%)
> Source: U1 (LM1117-3.3)

**ERCE_POWER_MISSING_ANNOTATION:**
> U1 (LM1117-3.3): Missing power annotation field 'Max_Current'
> Add field via Symbol Properties to enable power budget analysis

**ERCE_POWER_UNKNOWN_LOAD_CURRENT:**
> U2 (STM32F405) on power rail '+3V3': No 'Current_Draw' annotation
> Add field via Symbol Properties to include in power budget

**ERCE_POWER_ORPHAN_RAIL:**
> Power net '+12V' has no identified power source
> Add 'Power_Type: Source' field to the supply component, or configure source patterns in power distribution settings
