# Implementation Plan: Power Distribution Chart for KiCad Eeschema (Revised)

## 1. Executive Summary

This plan describes how to add a **Power Distribution Chart** feature to KiCad's schematic editor (eeschema). Instead of building a custom wxPanel visualization, the chart is **generated directly as schematic drawing items on a dedicated sheet** — making it printable, exportable to PDF, versionable in git, and fully native to KiCad's existing rendering/editing/export pipeline.

Additionally, new **ERC (Electrical Rules Check) rules** flag power-related issues (overloaded rails, missing annotations, orphan power nets), integrating power analysis into the familiar ERC workflow.

The data model is designed as the **shared foundation** that ties together the existing PDN impedance analyzer (side pane in pcbnew) and FastCap/FastHenry parasitic extraction (also pcbnew, on other branches) — living in `common/` with in-process sharing via the project object and optional JSON export.

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

### System Overview

Three tools form a power integrity suite across the two editors. The PDN analyzer
and FastCap/FastHenry integration already exist on other branches; this plan adds
the schematic-level analysis and the shared data model that ties them together.

```
┌─────────────────────────────────────────────────────────────────────┐
│                         common/                                      │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │  POWER_DISTRIBUTION_MODEL        (shared data structures)     │  │
│  │  POWER_DISTRIBUTION_SETTINGS     (shared configuration)       │  │
│  │  JSON serialization              (file-based exchange)        │  │
│  └──────────────────────┬────────────────────────────────────────┘  │
└─────────────────────────┼───────────────────────────────────────────┘
                          │
          ┌───────────────┴──────────────────────────┐
          │                                          │
          ▼                                          ▼
┌──────────────────────┐               ┌──────────────────────────────┐
│    EESCHEMA            │               │          PCBNEW                │
│                        │               │                                │
│ ANALYZER               │               │  PDN Analyzer (exists)         │
│  walks schematic,      │  in-process   │   discovers rails via          │
│  classifies components,│──────────────▶│   power_in/power_out pins      │
│  populates model       │   or JSON     │   + caps on those nets         │
│                        │               │   shows Z(f) in side pane      │
│ CHART GENERATOR        │               │                                │
│  power tree on sheet   │               │  FastCap/FastHenry (exists)    │
│                        │               │   BEM parasitic C/L extraction │
│ ERC CHECKS             │               │   from copper geometry         │
│  power rule violations │               │                                │
└──────────────────────┘               └──────────────────────────────┘
```

### Data Flow Between Tools

```
   EESCHEMA                                         PCBNEW
┌──────────────────┐                    ┌───────────────────────────────┐
│                  │                    │                               │
│  User annotates  │                    │  PDN Analyzer side pane       │
│  Power_Type,     │                    │  ┌─────────────────────────┐  │
│  Max_Current,    │    in-process:     │  │ Z(f) impedance plot     │  │
│  Current_Draw    │    project holds   │  │                         │  │
│  on symbols      │    model object    │  │ discovers rails via     │  │
│       │          │────────────────────│──│ power_in/power_out pins │  │
│       ▼          │        or          │  │ + decoupling caps       │  │
│  Analyze +       │    file-based:     │  │                         │  │
│  Generate Chart  │    .json export    │  │ enriched with:          │  │
│       │          │────────────────────│──│ • load currents         │  │
│       ▼          │                    │  │ • VRM params            │  │
│  Power tree      │                    │  │ • return net pairs      │  │
│  on sheet        │                    │  └─────────────────────────┘  │
│  + ERC markers   │                    │             │                 │
│                  │                    │             ▼                 │
│                  │                    │  FastCap/FastHenry            │
│                  │                    │  • uses net + return_net      │
│                  │                    │    for conductor pair IDs     │
│                  │                    │  • extracted C/L feed back    │
│                  │                    │    into PDN analyzer model    │
└──────────────────┘                    └───────────────────────────────┘
```

### Two Data Exchange Paths

| Path | When | Mechanism |
|------|------|-----------|
| **In-process** | Both editors open on same project (normal KiCad use) | `POWER_DISTRIBUTION_MODEL` object held in project state; pcbnew reads it directly from memory via `common/` types |
| **File-based** | CI/automation, or user exports explicitly | `.kicad_power_dist.json` written next to `.kicad_pro`; PDN analyzer loads on demand |

The in-process path is the default — when the user generates the chart in eeschema, the model is stored in the project. When they switch to pcbnew and open the PDN analyzer side pane, it finds the model already populated. The JSON file path exists for automation and for cases where the schematic and layout are analyzed separately.

### Module Breakdown

| Module | Location | Purpose |
|--------|----------|---------|
| `POWER_DISTRIBUTION_MODEL` | `common/power_distribution/power_distribution_model.{h,cpp}` | Shared data structures + JSON serialization |
| `POWER_DISTRIBUTION_SETTINGS` | `common/power_distribution/power_distribution_settings.{h,cpp}` | Shared configuration (field names, patterns, thresholds) |
| `POWER_DISTRIBUTION_ANALYZER` | `eeschema/power_distribution/power_distribution_analyzer.{h,cpp}` | Schematic analysis engine |
| `POWER_CHART_GENERATOR` | `eeschema/power_distribution/power_chart_generator.{h,cpp}` | Generates drawing items on sheet |
| ERC integration | `eeschema/erc/erc.{h,cpp}`, `erc_settings.h`, `erc_item.{h,cpp}` | New power-related ERC checks |
| Menu integration | `eeschema/menubar.cpp`, `eeschema/tools/sch_actions.{h,cpp}` | Menu item and action |
| Tool handler | `eeschema/tools/sch_editor_control.{h,cpp}` | Action handler for generate command |

---

## 4. Detailed Design

### 4.1 Data Model (`common/power_distribution/power_distribution_model.h`)

The model lives in `common/` so it can be consumed by both eeschema (analyzer, chart, ERC) and pcbnew (PDN analyzer, DC IR drop, FastCap/FastHenry). It contains no schematic-specific types — component references use strings and pin numbers rather than `SCH_SYMBOL*` pointers, making the model serializable and usable across editor boundaries.

Eeschema-side code holds `SCH_SYMBOL*` pointers separately in the analyzer for navigation/markers, but the model itself is editor-agnostic.

```cpp
#ifndef POWER_DISTRIBUTION_MODEL_H
#define POWER_DISTRIBUTION_MODEL_H

#include <wx/string.h>
#include <vector>
#include <memory>

namespace nlohmann { class json; }  // forward decl for serialization


// Identifies a specific pin on a component, bridging schematic and PCB domains.
// The pin number matches the footprint pad number, enabling PCB-side tools
// (PDN analyzer, DC IR drop, FastCap/FastHenry) to locate exact current
// injection/extraction points on copper geometry.
struct PIN_REFERENCE
{
    wxString  m_pinNumber;   // Schematic pin number = PCB pad number
    wxString  m_pinName;     // Human-readable: "VIN", "VOUT", "GND", etc.
    wxString  m_netName;     // Net this pin connects to
};


struct POWER_SOURCE
{
    wxString         m_name;
    wxString         m_reference;     // "J1", "BT1" — PCB cross-reference
    wxString         m_netName;
    double           m_voltage;
    double           m_maxCurrent;    // Amps

    std::vector<PIN_REFERENCE>  m_powerPins;   // Pins carrying power out
    std::vector<PIN_REFERENCE>  m_groundPins;  // Return/ground pins

    // Eeschema-only (not serialized): symbol pointer + sheet path
    // Held by the analyzer for ERC marker placement, not part of the model.
};

struct POWER_REGULATOR
{
    wxString         m_name;
    wxString         m_reference;     // "U1" — PCB cross-reference
    wxString         m_type;          // "LDO", "Buck", "Boost", etc.
    wxString         m_inputNet;
    wxString         m_outputNet;
    double           m_inputVoltage;
    double           m_outputVoltage;
    double           m_maxOutputCurrent;
    double           m_efficiency;    // 0.0-1.0

    std::vector<PIN_REFERENCE>  m_inputPins;   // VIN pins
    std::vector<PIN_REFERENCE>  m_outputPins;  // VOUT pins
    std::vector<PIN_REFERENCE>  m_groundPins;  // GND/exposed-pad pins
};

struct POWER_RAIL
{
    wxString         m_netName;
    wxString         m_returnNet;     // Paired return net (e.g. "GND" for "+3V3")
    double           m_voltage;
    double           m_availableCurrent;
    double           m_consumedCurrent;
    double           m_utilizationPct;
};

struct POWER_LOAD
{
    wxString         m_name;
    wxString         m_reference;     // "U2", "R5" — PCB cross-reference
    wxString         m_netName;
    double           m_currentDraw;   // Amps

    std::vector<PIN_REFERENCE>  m_powerPins;   // VCC/VDD pins on this net
};

struct POWER_TREE_NODE
{
    enum NODE_TYPE { SOURCE, REGULATOR, RAIL, LOAD };

    NODE_TYPE        m_type;
    wxString         m_label;
    wxString         m_reference;     // Component reference for PCB matching
    double           m_voltage;
    double           m_currentAvailable;
    double           m_currentConsumed;

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

    // --- Serialization (integration point for PDN analyzer, DC IR drop, etc.) ---
    void     ToJSON( nlohmann::json& aOut ) const;
    bool     FromJSON( const nlohmann::json& aIn );
    void     SaveToFile( const wxString& aPath ) const;
    bool     LoadFromFile( const wxString& aPath );

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

#### JSON Export Format

The JSON format is the **integration contract** between tools. The PDN analyzer
and FastCap/FastHenry exporter in pcbnew consume this to identify rail pairs,
boundary conditions, and component-to-pad mapping without re-analyzing the
schematic.

```json
{
  "version": 1,
  "total_power_input_w": 2.5,
  "total_power_consumed_w": 1.8,
  "rails": [
    {
      "net": "+3V3",
      "return_net": "GND",
      "voltage": 3.3,
      "available_current_a": 0.3,
      "consumed_current_a": 0.195,
      "utilization_pct": 65.0,
      "sources": [
        {
          "reference": "U1",
          "name": "LM1117-3.3",
          "type": "LDO",
          "input_net": "+5V",
          "output_voltage": 3.3,
          "max_output_current_a": 0.3,
          "efficiency": 1.0,
          "input_pins":  [{"pad": "1", "name": "VIN",  "net": "+5V"}],
          "output_pins": [{"pad": "3", "name": "VOUT", "net": "+3V3"}],
          "ground_pins": [{"pad": "2", "name": "GND",  "net": "GND"}]
        }
      ],
      "loads": [
        {
          "reference": "U2",
          "name": "STM32F405",
          "current_a": 0.05,
          "pins": [
            {"pad": "12", "name": "VDD1", "net": "+3V3"},
            {"pad": "48", "name": "VDD2", "net": "+3V3"}
          ]
        },
        {
          "reference": "U3",
          "name": "W25Q128",
          "current_a": 0.025,
          "pins": [{"pad": "8", "name": "VCC", "net": "+3V3"}]
        }
      ]
    }
  ]
}
```

**Consumers (in-process or via JSON):**

The PDN analyzer and FastCap/FastHenry already exist on other branches and work standalone. They discover rails independently by walking `power_in`/`power_out` pins and finding decoupling capacitors on those nets. This model provides **enrichment** — data they cannot derive from pin types alone. See Section 7 for detailed integration design.

| Consumer | What it reads | What it does with it |
|----------|--------------|---------------------|
| **PDN Analyzer** | `rails[].return_net` | Pre-selects return plane pairing (resolves AGND/DGND/GND ambiguity without guessing from net names) |
| **PDN Analyzer** | `loads[].current_a` + `pins[].pad` | Sets current sink magnitudes for target impedance: `Z_target = V_ripple / I_transient` |
| **PDN Analyzer** | `sources[].max_output_current_a`, `efficiency` | VRM impedance model parameters in the side pane |
| **PDN Analyzer** | `sources[].output_pins[].pad` | VRM source pad location for decoupling analysis |
| **FastCap/FastHenry** | `rails[].net` + `return_net` | Conductor pair presets for extraction (avoids extracting every combination) |
| **DC IR Drop** (future) | `loads[]/sources[]` current + pad | Boundary conditions for resistive mesh solve |

### 4.2 Analyzer (`eeschema/power_distribution/power_distribution_analyzer.h/.cpp`)

Walks the schematic, classifies components, builds power tree, computes budgets. Uses `CONNECTION_GRAPH` for net tracing. The analyzer holds eeschema-specific data (SCH_SYMBOL pointers, SCH_SHEET_PATHs) for ERC marker placement, while populating the editor-agnostic model.

**Key additions for cross-tool integration:**

#### 4.2.1 Component Reference and Pin Capture

When classifying a symbol as a source, regulator, or load, the analyzer now also captures:
- `m_reference` — the component's reference designator string (e.g. "U1"), which matches the footprint reference in pcbnew
- `PIN_REFERENCE` entries — for each power-relevant pin, the pin number (= PCB pad number), pin name, and connected net

This data is already available during the schematic walk (from `SCH_SYMBOL::GetRef()` and `SCH_PIN::GetNumber()`/`GetName()`/`GetNet()`), so this is not additional analysis work — just additional capture.

#### 4.2.2 Return Net Identification

New method: `IdentifyReturnNets()` — called after the power tree is built.

For each `POWER_RAIL`, determines the paired return net (typically GND). Strategy:
1. For rails fed by a regulator: the regulator's ground pin net is the return net
2. For rails fed by a source (connector, battery): the source's ground pin net is the return net
3. Fallback: look for a net named "GND" or matching a configurable return-net pattern

This is critical for the PDN analyzer, which needs power/return plane pairs.

```cpp
class POWER_DISTRIBUTION_ANALYZER
{
public:
    POWER_DISTRIBUTION_ANALYZER( SCHEMATIC* aSchematic,
                                  const POWER_DISTRIBUTION_SETTINGS& aSettings );

    bool Analyze();

    POWER_DISTRIBUTION_MODEL& GetModel() { return m_model; }

    // Eeschema-specific: map from component reference to SCH_SYMBOL* + sheet path
    // Used by ERC marker placement — not part of the serializable model.
    struct SYMBOL_LOCATION
    {
        SCH_SYMBOL*    m_symbol;
        SCH_SHEET_PATH m_sheetPath;
    };

    const SYMBOL_LOCATION* GetSymbolLocation( const wxString& aReference ) const;

private:
    void classifyComponents();
    void buildPowerTree();
    void computeBudgets();
    void identifyReturnNets();     // NEW: pairs each rail with its return net
    void captureComponentPins();   // NEW: populates PIN_REFERENCE vectors

    SCHEMATIC*                      m_schematic;
    POWER_DISTRIBUTION_SETTINGS     m_settings;
    POWER_DISTRIBUTION_MODEL        m_model;

    // Eeschema-only lookup for marker placement
    std::map<wxString, SYMBOL_LOCATION>  m_symbolLocations;
};
```

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

### 4.5 Settings (`common/power_distribution/power_distribution_settings.h/.cpp`)

Stored in `.kicad_pro` under `"power_distribution"` key. Lives in `common/` so pcbnew-side tools can read the same configuration (e.g. the PDN analyzer needs to know the field name mappings to cross-reference). Contains:
- Field name mappings (`Power_Type`, `Output_Voltage`, `Max_Current`, `Current_Draw`, `Efficiency`)
- Source patterns (`Connector*`, `Battery*`, `USB*`, etc.)
- Regulator patterns (`Regulator_Linear*`, `Regulator_Switching*`, etc.)
- Warning/critical thresholds (80%, 95%)
- Chart sheet name (default: `"Power Distribution"`)
- Return net patterns (default: `"GND"`, `"AGND"`, `"DGND"`, `"GND_*"`) — used by `IdentifyReturnNets()`

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

### Phase 1: Shared Data Model and Settings (in `common/`)

**Files to create:**
- `common/power_distribution/power_distribution_model.h`
- `common/power_distribution/power_distribution_model.cpp` — includes JSON serialization
- `common/power_distribution/power_distribution_settings.h`
- `common/power_distribution/power_distribution_settings.cpp`

**Files to modify:**
- `common/CMakeLists.txt` — Add new source files to `common` library

### Phase 2: Schematic Analyzer (in `eeschema/`)

**Files to create:**
- `eeschema/power_distribution/power_distribution_analyzer.h`
- `eeschema/power_distribution/power_distribution_analyzer.cpp` — includes `identifyReturnNets()`, `captureComponentPins()`

**Files to modify:**
- `eeschema/CMakeLists.txt`

### Phase 3: Chart Generator

**Files to create:**
- `eeschema/power_distribution/power_chart_generator.h`
- `eeschema/power_distribution/power_chart_generator.cpp`

**Files to modify:**
- `eeschema/CMakeLists.txt`

### Phase 4: ERC Integration

**Files to modify:**
- `eeschema/erc/erc_settings.h` — Add `ERCE_POWER_*` enum values
- `eeschema/erc/erc_item.h` — Add static ERC_ITEM declarations
- `eeschema/erc/erc_item.cpp` — Register items, add to allItemTypes, add to Create()
- `eeschema/erc/erc.h` — Add `TestPowerDistribution()` declaration
- `eeschema/erc/erc.cpp` — Implement `TestPowerDistribution()`, hook into `RunTests()`
- `eeschema/erc/erc_settings.cpp` — Set default severities

### Phase 5: Menu and Action Integration

**Files to modify:**
- `eeschema/tools/sch_actions.h` — Add `generatePowerDistChart` action
- `eeschema/tools/sch_actions.cpp` — Define the action
- `eeschema/tools/sch_editor_control.h` — Add handler declaration
- `eeschema/tools/sch_editor_control.cpp` — Implement handler, store model in project, optional JSON export
- `eeschema/menubar.cpp` — Add to Inspect menu

### Phase 6: Project-Level Model Storage (Integration Glue)

**Files to modify:**
- `common/project/project.h` — Add `POWER_DISTRIBUTION_MODEL` pointer, getter/setter
- `common/project/project.cpp` — Implement getter/setter, cleanup on project close

This is the in-process integration point. After generating the chart, the handler stores the model in the project. When the user switches to pcbnew, the PDN analyzer side pane finds it there.

### Phase 7: Polish and Testing

- Test with demo schematics
- Handle edge cases (empty designs, circular dependencies, multi-source rails)
- Ensure chart sheet is properly saved/loaded
- Verify JSON export round-trips correctly (write → read → compare)
- Test in-process model handoff: generate chart in eeschema, switch to pcbnew, verify PDN analyzer sees the model
- Add unit tests for model, analyzer, and serialization

---

## 6. File Summary

### New Files (8)

```
common/power_distribution/
├── power_distribution_model.h          ← shared between eeschema + pcbnew
├── power_distribution_model.cpp        ← includes JSON ToJSON/FromJSON
├── power_distribution_settings.h       ← shared configuration
└── power_distribution_settings.cpp

eeschema/power_distribution/
├── power_distribution_analyzer.h       ← schematic-specific (walks SCH_SYMBOL)
├── power_distribution_analyzer.cpp
├── power_chart_generator.h             ← schematic-specific (writes to SCH_SCREEN)
└── power_chart_generator.cpp
```

### Modified Files (13)

```
common/CMakeLists.txt                — Add common/power_distribution/ source files
common/project/project.h             — Add POWER_DISTRIBUTION_MODEL storage (integration glue)
common/project/project.cpp           — Implement model getter/setter
eeschema/CMakeLists.txt              — Add eeschema/power_distribution/ source files
eeschema/erc/erc_settings.h          — Add ERCE_POWER_* enum values
eeschema/erc/erc_item.h              — Add static ERC_ITEM declarations
eeschema/erc/erc_item.cpp            — Register power ERC items
eeschema/erc/erc.h                   — Add TestPowerDistribution()
eeschema/erc/erc.cpp                 — Implement power ERC checks, hook into RunTests()
eeschema/erc/erc_settings.cpp        — Set default severities for power checks
eeschema/tools/sch_actions.h         — Add generatePowerDistChart action
eeschema/tools/sch_actions.cpp       — Define the action
eeschema/tools/sch_editor_control.cpp — Implement action handler, store model in project
eeschema/menubar.cpp                 — Add Inspect menu entry
```

---

## 7. Integration with PDN Analyzer and FastCap/FastHenry

The PDN analyzer and FastCap/FastHenry integration already exist on other branches. This section describes the concrete code needed to tie everything together when the branches are merged.

### 7.1 What Already Exists (Other Branches)

| Tool | What it does | UI | Rail discovery |
|------|-------------|-----|----------------|
| **PDN Analyzer** | Frequency-domain impedance Z(f) of power/return plane pairs with decoupling cap ESR/ESL models | Side pane in pcbnew showing impedance plot | Independent — walks `power_in`/`power_out` pins, finds decoupling caps on those nets |
| **FastCap/FastHenry** | BEM quasistatic extraction of parasitic capacitance and inductance from copper geometry | Export/import in pcbnew | Operates on nets specified by user or by PDN analyzer |

Both work standalone today. The integration adds **enrichment**, not dependency.

### 7.2 What the Power Distribution Model Adds to Each

The PDN analyzer discovers rails on its own but cannot determine **how much current** each load draws, **which component is the VRM** vs a load, or **what the regulator's limits are**. The power distribution model fills these gaps.

#### For the PDN Analyzer

| What the PDN analyzer knows alone | What it gains from the model |
|-----------------------------------|------------------------------|
| Which nets are power rails (via pin types) | Which net is the paired return (via regulator ground pin tracing, resolving AGND/DGND/GND ambiguity) |
| Which pads are on power nets | How much current each load draws (for target impedance: `Z_target = V_ripple / I_transient`) |
| Which components are decoupling caps (via capacitor detection) | Which component is the VRM source (for VRM impedance model placement) |
| — | VRM max output current + efficiency (for VRM model parameters) |
| — | Regulator type (LDO vs buck vs boost — affects VRM output impedance characteristics) |

**Concrete integration point:** When the PDN analyzer side pane opens for a rail, it checks the project for a `POWER_DISTRIBUTION_MODEL`. If present, it:
1. Pre-selects the return net from `POWER_RAIL::m_returnNet` instead of guessing from net names
2. Sets load current sinks from `POWER_LOAD::m_currentDraw` values at the corresponding pad locations
3. Configures VRM source model from `POWER_REGULATOR::m_maxOutputCurrent` and `m_efficiency`
4. Displays rail utilization from the budget in the side pane header

If no model is present, the PDN analyzer works exactly as it does today — the user manually sets these parameters.

#### For FastCap/FastHenry

| What it knows alone | What it gains from the model |
|---------------------|------------------------------|
| All copper geometry on specified nets | Which conductor pairs matter (net + return_net) — avoids extracting every possible combination |
| — | Which pads are current injection/extraction points (scopes the extraction region) |

### 7.3 In-Process Integration (Primary Path)

When both editors are open on the same project (normal KiCad workflow), the model is shared via the project object — no JSON file needed.

```cpp
// In common/project/project.h — add model storage to the project
class PROJECT
{
    // ... existing members ...

    // Power distribution model — populated by eeschema analyzer,
    // consumed by pcbnew PDN analyzer and FastCap/FastHenry.
    // Null until the user generates the chart or runs power ERC.
    POWER_DISTRIBUTION_MODEL* GetPowerDistModel() const;
    void SetPowerDistModel( std::unique_ptr<POWER_DISTRIBUTION_MODEL> aModel );
};
```

**Eeschema side** (this plan, in the action handler):
```cpp
// After analysis completes:
m_frame->Prj().SetPowerDistModel( std::move( analyzer.TakeModel() ) );
```

**Pcbnew side** (integration changes to the PDN analyzer branch):
```cpp
// When the PDN analyzer side pane opens or refreshes:
const POWER_DISTRIBUTION_MODEL* model = m_frame->Prj().GetPowerDistModel();
if( model )
{
    // Look up the rail being analyzed
    for( const POWER_RAIL& rail : model->GetRails() )
    {
        if( rail.m_netName == currentRailNet )
        {
            setReturnNet( rail.m_returnNet );
            setTargetCurrent( rail.m_consumedCurrent );
            // ... set VRM params from matching source/regulator ...
            break;
        }
    }
}
// else: PDN analyzer works standalone as before
```

### 7.4 File-Based Integration (Secondary Path)

For CI/automation, or when the user wants to export data explicitly.

- **Export:** Inspect → Generate Power Distribution Chart offers "Also export JSON" checkbox (or always writes automatically alongside the chart). Writes `<project>.kicad_power_dist.json` next to the `.kicad_pro`.
- **Import:** PDN analyzer side pane has a "Load Power Budget..." button that reads the JSON file and populates the model into the project.

This path is also useful when someone wants to share power budget data without sharing the full schematic.

### 7.5 End-to-End User Workflow

```
1. EESCHEMA: Annotate symbols
   └─ Add Power_Type, Max_Current, Current_Draw, Efficiency fields
   └─ Use Symbol Fields Table for bulk editing

2. EESCHEMA: Run ERC
   └─ Power ERC checks flag missing annotations
   └─ Fix warnings, re-run

3. EESCHEMA: Inspect → Generate Power Distribution Chart
   └─ Analyzer runs, model is populated, chart sheet is created
   └─ Model is stored in project (available to pcbnew immediately)
   └─ Optional: JSON export for archival/CI

4. PCBNEW: Open PDN Analyzer side pane, select a rail
   └─ PDN analyzer discovers the rail independently (power pins + caps)
   └─ Finds POWER_DISTRIBUTION_MODEL in project → auto-fills:
      • Return net (from regulator ground pin tracing)
      • Load current sinks (from Current_Draw annotations)
      • VRM model (from regulator Max_Current + Efficiency)
   └─ User sees Z(f) impedance plot with realistic current targets
   └─ Without the model: same plot, but with generic/manual parameters

5. PCBNEW: Run FastCap/FastHenry extraction on the rail
   └─ Uses net + return_net pair from model (or user selection)
   └─ Extracted parasitics feed back into PDN analyzer model
   └─ Updated Z(f) plot reflects field-solved C/L

6. Iterate: change decoupling cap values → re-run PDN analysis
   └─ Change regulator → re-generate chart in eeschema → model updates
```

### 7.6 Integration Code Changes Needed (Other Branches)

These are the specific changes needed in the PDN analyzer and FastCap/FastHenry branches to consume the model. Listed here so the branches can be coordinated.

**In `common/project/project.h`** (or equivalent project-level storage):
- Add `POWER_DISTRIBUTION_MODEL` pointer and getter/setter (see 7.3)

**In the PDN analyzer branch** (pcbnew):
- When opening the side pane for a rail, check `Prj().GetPowerDistModel()`
- If model has a matching rail: pre-fill return net, load currents, VRM params
- Add "Load Power Budget..." button for file-based import fallback
- Display rail utilization percentage in the side pane header when model is available

**In the FastCap/FastHenry branch** (pcbnew):
- When selecting nets for extraction, offer model's rail pairs as presets
- If model available, scope extraction to electrically relevant pads

### 7.7 Design Decisions

- **PDN analyzer is standalone**: It finds rails via `power_in`/`power_out` pins + capacitor detection. The model is consumed when available but not required. No hard dependency between eeschema and pcbnew analysis workflows.
- **Model in `common/`**: Both editors link to the same types. In-process sharing via the project object avoids serialization overhead.
- **JSON as secondary path**: For automation and explicit export. Not required for normal interactive use.
- **`m_returnNet` from ground pin tracing**: More authoritative than net-name heuristics. The PDN analyzer can discover rails but cannot reliably pair them with return nets in split-ground designs without this data.
- **Enrichment, not control**: The model provides data to improve analysis quality. It never overrides the PDN analyzer's own rail discovery — if the model says a net is a rail but the PDN analyzer doesn't see power pins on it, the PDN analyzer's view prevails.

---

## 8. Key Design Decisions

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

## 9. Round-Trip Safety

This feature is **fully round-trip safe**:
- The analyzer **only reads** existing schematic data
- The chart sheet is a standard `.kicad_sch` file — fully compatible with KiCad's file format
- ERC markers are standard `SCH_MARKER` objects — no format changes
- The chart can be deleted by simply removing the sub-sheet — no orphaned data
- No changes to the `.kicad_sch` S-expression format
- Settings stored in `.kicad_pro` under a new `"power_distribution"` key — ignored by older KiCad versions
- JSON export is a separate file (`.kicad_power_dist.json`) — optional, not required for the chart or ERC to function
- The JSON file can be regenerated at any time from the schematic — it is derived data, not a source of truth

---

## 10. ERC Error Messages (Examples)

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
