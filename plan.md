# Implementation Plan: System Diagram Generator for KiCad Eeschema

## 1. Overview

A **system diagram** is a high-level, auto-generated schematic sheet that shows how the major
components in a design are interconnected — without drowning in per-pin detail. It answers
the question *"what talks to what, and how is it powered?"* at a glance.

**Scope (v1 — this plan):**

- **Bus connectivity diagram**: Only buses with an explicit `BUS_ALIAS` defined in Schematic
  Setup appear. Each aliased bus is drawn as a bar. ICs (`U?`) and connectors (`J?`) that
  touch any member net of that bus are shown as labeled boxes connected to the bar.
- **Specially marked signals**: Individual nets the user marks with a `System_Diagram`
  net class are also shown as simple lines between their component boxes.
- **Power topology diagram**: A tree showing how power flows from sources through regulators
  to rails and on to loads. Detected automatically from `PT_POWER_IN` / `PT_POWER_OUT` pin
  types — no user annotation needed. Voltages shown where available from power symbol values.
  No current accounting, no utilization, no ERC checks.
- **Auto layout**: A layered (Sugiyama-style) graph layout algorithm places the boxes and
  routes the connections automatically.
- **Output**: A dedicated sub-sheet in the schematic hierarchy, rendered with native
  schematic drawing items (SCH_SHAPE, SCH_TEXT, SCH_LINE). Printable, PDF-exportable,
  git-versionable for free.

**Deferred (not in this plan):**

- Current accounting and power budgets
- ERC checks for power (overloaded rails, missing annotations)
- JSON export / cross-tool data model for PDN analyzer integration
- Pin-level detail on the diagram (pin names, pin numbers)
- Custom fields on symbols (`Power_Type`, `Max_Current`, `Current_Draw`, etc.)
- Settings UI dialog
- Net class auto-detection of buses (inferring interfaces without explicit aliases)
- Board-level (pcbnew) integration

---

## 2. What Users See

1. **Inspect → Generate System Diagram** menu item (and/or toolbar button)
2. A new hierarchical sheet named "System Diagram" is created (or updated)
3. The sheet contains two sections:

```
┌─────────────────────────────────────────────────────────────────────┐
│  SYSTEM DIAGRAM               MyProject   2025-07-01               │
│                                                                     │
│  ── BUS CONNECTIVITY ──────────────────────────────────────────     │
│                                                                     │
│  ┌─────────┐       SPI1       ┌─────────┐                          │
│  │  U1      │═════════════════│  U2      │                          │
│  │ STM32F4  │       I2C1      │ BME280   │                          │
│  │          │═════════════════│          │                          │
│  │          │       SDIO      ┌─────────┐                          │
│  │          │═════════════════│  J2      │                          │
│  │          │                 │ SD Card  │                          │
│  │          │       USB       ┌─────────┐                          │
│  │          │═════════════════│  J1      │                          │
│  └─────────┘                  │ USB-C    │                          │
│                               └─────────┘                          │
│  ┌─────────┐                                                        │
│  │  U3      │    UART_DEBUG   ┌─────────┐                          │
│  │ ESP32    │────────────────→│  J3      │                          │
│  └─────────┘                  │ Header   │                          │
│                                                                     │
│  ── POWER TOPOLOGY ────────────────────────────────────────────     │
│                                                                     │
│  ┌──────────┐    ┌────────────┐    ┌───────┐                       │
│  │ J1 USB-C │───→│ U4 LM1117  │───→│ +3V3  │──→ U1, U2, U3       │
│  │ +5V      │    │ LDO → 3.3V │    └───────┘                       │
│  │          │    └────────────┘                                     │
│  │          │    ┌────────────┐    ┌───────┐                       │
│  │          │───→│ U5 TPS6216 │───→│ +1V8  │──→ U1 (core)         │
│  └──────────┘    │ Buck → 1.8V│    └───────┘                       │
│                  └────────────┘                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**How it updates:** The user re-runs Inspect → Generate System Diagram. The sheet is
cleared and regenerated. Edits to the generated sheet are lost (a warning label on the
sheet says so). Generation is wrapped in `SCH_COMMIT` for single-step undo.

---

## 3. Architecture

### Module Breakdown

Everything lives in `eeschema/`. No `common/` changes needed for v1.

| Module | Location | Purpose |
|--------|----------|---------|
| `SYSTEM_DIAGRAM_ANALYZER` | `eeschema/system_diagram/system_diagram_analyzer.{h,cpp}` | Walks schematic, builds bus graph + power tree |
| `SYSTEM_DIAGRAM_LAYOUT` | `eeschema/system_diagram/system_diagram_layout.{h,cpp}` | Layered graph layout algorithm |
| `SYSTEM_DIAGRAM_GENERATOR` | `eeschema/system_diagram/system_diagram_generator.{h,cpp}` | Creates drawing items on the sub-sheet |
| Menu + action | `eeschema/tools/sch_actions.{h,cpp}`, `sch_editor_control.{h,cpp}`, `menubar.cpp` | Menu entry and action handler |

### Data Flow

```
SCHEMATIC
    │
    ▼
SYSTEM_DIAGRAM_ANALYZER
    │  Reads: BUS_ALIAS list, CONNECTION_GRAPH, SCH_SYMBOLs, pin types
    │  Outputs: SYSTEM_DIAGRAM_DATA (bus graph + power tree)
    ▼
SYSTEM_DIAGRAM_LAYOUT
    │  Inputs: SYSTEM_DIAGRAM_DATA (abstract graph)
    │  Outputs: SYSTEM_DIAGRAM_DATA with (x, y) positions assigned
    ▼
SYSTEM_DIAGRAM_GENERATOR
    │  Inputs: laid-out SYSTEM_DIAGRAM_DATA
    │  Outputs: SCH_SHAPE / SCH_TEXT / SCH_LINE items on a SCH_SCREEN
    ▼
"System Diagram" sub-sheet (visible in hierarchy)
```

---

## 4. Detailed Design

### 4.1 Data Structures (`system_diagram_analyzer.h`)

These are internal structs, not serialized or shared with pcbnew.

```cpp
// A component box on the diagram (IC or connector)
struct SD_COMPONENT
{
    wxString    m_reference;     // "U1", "J2"
    wxString    m_value;         // "STM32F405", "USB-C"
    VECTOR2I    m_pos;           // Assigned by layout
    VECTOR2I    m_size;          // Assigned by layout (based on text + connections)
};

// A bus bar connecting components
struct SD_BUS
{
    wxString                     m_aliasName;     // "SPI1", "I2C1"
    std::vector<wxString>        m_members;       // Member net names from BUS_ALIAS
    std::vector<SD_COMPONENT*>   m_connectedComponents;
};

// A specially marked signal (net class = "System_Diagram")
struct SD_SIGNAL
{
    wxString                     m_netName;       // "UART_TX", "RESET_N"
    std::vector<SD_COMPONENT*>   m_connectedComponents;
};

// Power tree structures
struct SD_POWER_NODE
{
    enum TYPE { SOURCE, REGULATOR, RAIL };

    TYPE        m_type;
    wxString    m_reference;     // "J1", "U4", "" (for rail nodes)
    wxString    m_value;         // "USB-C", "LM1117", "+3V3"
    wxString    m_netName;       // Output net
    double      m_voltage;       // Volts, or 0 if unknown
    VECTOR2I    m_pos;           // Assigned by layout
    VECTOR2I    m_size;

    std::vector<SD_POWER_NODE*>  m_children;
    std::vector<wxString>        m_loadRefs;  // References of ICs on this rail
};

// Top-level container
struct SYSTEM_DIAGRAM_DATA
{
    std::vector<std::unique_ptr<SD_COMPONENT>>   m_components;
    std::vector<std::unique_ptr<SD_BUS>>         m_buses;
    std::vector<std::unique_ptr<SD_SIGNAL>>      m_signals;
    std::vector<std::unique_ptr<SD_POWER_NODE>>  m_powerRoots;  // Forest of power trees

    // Lookup helper
    SD_COMPONENT* FindComponent( const wxString& aRef );
};
```

### 4.2 Analyzer (`system_diagram_analyzer.{h,cpp}`)

```cpp
class SYSTEM_DIAGRAM_ANALYZER
{
public:
    SYSTEM_DIAGRAM_ANALYZER( SCHEMATIC* aSchematic );

    bool Analyze();

    SYSTEM_DIAGRAM_DATA& GetData() { return m_data; }

private:
    // Phase 1: Discover which components appear on the diagram
    void discoverComponents();

    // Phase 2: Build bus connectivity from BUS_ALIAS definitions
    void buildBusGraph();

    // Phase 3: Find specially marked nets (net class "System_Diagram")
    void findMarkedSignals();

    // Phase 4: Build power tree from PT_POWER_IN / PT_POWER_OUT pins
    void buildPowerTree();

    SCHEMATIC*           m_schematic;
    SYSTEM_DIAGRAM_DATA  m_data;
};
```

#### Phase 1: `discoverComponents()`

Walk all `SCH_SYMBOL` instances across all sheets. A component is included if:
- It has reference prefix `U` (IC) or `J` (connector), **and**
- At least one of its pins connects to a net that is a member of a `BUS_ALIAS`,
  OR connects to a net in the `System_Diagram` net class,
  OR has a `PT_POWER_OUT` pin (making it a power source/regulator)

This is the "pull" model — the diagram only shows components that are relevant to
at least one bus, marked signal, or power relationship.

#### Phase 2: `buildBusGraph()`

For each `BUS_ALIAS` in the schematic (via `SCH_SCREEN::GetBusAliases()` iterated
across `Hierarchy()`):

1. Collect the set of member net names from `alias->Members()`
2. For each `SD_COMPONENT`, check if any of its pins connect to any member net
   (using `CONNECTION_GRAPH` to resolve net names)
3. If ≥ 2 components connect to the same bus alias, create an `SD_BUS` linking them
4. If only 1 component connects, skip (single-endpoint bus isn't useful on the diagram)

**Key API calls:**
- `screen->GetBusAliases()` → iterate aliases
- `alias->Members()` → get member net names
- `CONNECTION_GRAPH` → resolve which symbols connect to which nets

#### Phase 3: `findMarkedSignals()`

Walk all nets. If a net's net class is `System_Diagram` (configurable name, but
hardcoded for v1 — settings dialog is deferred), find which `SD_COMPONENT` instances
connect to it and create an `SD_SIGNAL`.

**Key API calls:**
- `SCHEMATIC::GetNetClassAssignmentCandidates()` or walk `NET_SETTINGS` from project
  to find nets assigned to the `System_Diagram` net class
- Alternatively, use a custom field `System_Diagram=yes` on a net label — but net class
  is the lighter-weight, already-existing mechanism

#### Phase 4: `buildPowerTree()`

Build a forest of power trees using pin types already present in the schematic:

1. **Find power sources**: Components with `PT_POWER_OUT` pins where the output net is
   a power rail. Connectors (`J?`) with power pins are also sources.
   For each source, create an `SD_POWER_NODE` with `TYPE::SOURCE`.

2. **Find regulators**: Components with both `PT_POWER_IN` and `PT_POWER_OUT` pins on
   different nets. These are intermediate nodes (`TYPE::REGULATOR`). The input net
   connects to a parent source or another regulator. The output net is a new rail.

3. **Build tree edges**: For each regulator, find its input net. Walk upstream to find
   the source or regulator that drives that net. Attach as child.

4. **Find loads on each rail**: For each `SD_POWER_NODE` of type `RAIL` (or the output
   side of a regulator), find all `U?` components that have `PT_POWER_IN` pins on that
   net. Store their references in `m_loadRefs`.

5. **Extract voltage**: Where the power net has an associated power symbol (e.g. `+3V3`),
   extract the voltage from the symbol's value field (parse numeric prefix).

**Key API calls:**
- `SCH_PIN::GetType()` → check for `PT_POWER_IN`, `PT_POWER_OUT`
- `SCH_PIN::GetNet()` → get connected net name
- `SCH_SYMBOL::GetRef()` → reference designator
- `SCH_SYMBOL::GetValueFieldText()` → value (for regulator identification)
- Iterate via `SCH_SHEET_LIST::GetAllItems()` or walk `Hierarchy()` sheets

### 4.3 Layout Engine (`system_diagram_layout.{h,cpp}`)

This is the hardest part. The layout engine takes the abstract graph from the analyzer
and assigns `(x, y)` positions to every node and routing to every edge.

```cpp
class SYSTEM_DIAGRAM_LAYOUT
{
public:
    // Lay out the bus connectivity section. Returns bounding box.
    BOX2I LayoutBusSection( SYSTEM_DIAGRAM_DATA& aData );

    // Lay out the power topology section. Returns bounding box.
    BOX2I LayoutPowerSection( SYSTEM_DIAGRAM_DATA& aData );

private:
    // Sugiyama-style layered layout (4 phases)
    void assignLayers( std::vector<SD_COMPONENT*>& aNodes,
                       std::vector<std::pair<SD_COMPONENT*, SD_COMPONENT*>>& aEdges );
    void minimizeCrossings();
    void assignXPositions();
    void routeEdges();

    // Simple tree layout for power section
    void layoutTree( SD_POWER_NODE* aRoot, VECTOR2I aOrigin, int aDepth );
};
```

#### Bus Section Layout: Sugiyama (Simplified)

The bus section is a bipartite-ish graph: components connected by buses. A full Sugiyama
implementation is complex, so v1 uses a **simplified layered approach**:

1. **Layer assignment**: Find the component with the most bus connections (the "hub" —
   typically the MCU). Place it in layer 0 (left). All components directly connected to
   it via a bus go in layer 1 (right). Components only connected to layer-1 components
   go in layer 2. (Rarely more than 2 layers in practice.)

2. **Ordering within layers**: Sort components within each layer to minimize edge crossings.
   Use the barycenter heuristic: position each node at the average Y position of its
   neighbors in the adjacent layer. Iterate 2-3 times.

3. **Bus bar placement**: Each bus is drawn as a horizontal bar between its leftmost and
   rightmost connected component. Buses are spaced vertically with a gap.

4. **Coordinate assignment**: Each component box is sized based on its text content
   (reference + value). Horizontal spacing is fixed. Vertical position comes from the
   ordering step.

**Simplifications for v1:**
- Maximum 3 layers (left/center/right). Components that don't fit are placed in an
  overflow area.
- Bus bars are horizontal only (no routing around obstacles).
- If the graph is disconnected (multiple separate clusters of buses), lay out each
  cluster independently, stacked vertically.

#### Power Section Layout: Tree

Power is inherently a tree (or forest). Layout is simpler:

1. Root nodes (sources) at the left.
2. Children (regulators) one level to the right.
3. Leaf nodes (rails with load lists) at the rightmost level.
4. Vertical spacing to avoid overlap, with enough room for load reference lists.

This is a standard recursive tree layout: each subtree is laid out, then the parent
is centered on its children.

#### Sheet Sizing

After both sections are laid out, the generator knows the total bounding box. The
generated sheet is sized to fit (using standard KiCad sheet sizes: A4, A3, A2, etc.,
picking the smallest that fits). If the diagram is very large, it uses a custom sheet
size.

### 4.4 Generator (`system_diagram_generator.{h,cpp}`)

Takes the laid-out data and emits native schematic drawing items.

```cpp
class SYSTEM_DIAGRAM_GENERATOR
{
public:
    SYSTEM_DIAGRAM_GENERATOR( SCHEMATIC* aSchematic );

    // Main entry point: create or update the "System Diagram" sub-sheet
    SCH_SHEET* Generate( SYSTEM_DIAGRAM_DATA& aData );

private:
    SCH_SHEET* getOrCreateSheet();
    void clearSheet( SCH_SCREEN* aScreen );

    // Bus section drawing
    void drawBusSection( SCH_SCREEN* aScreen, const SYSTEM_DIAGRAM_DATA& aData,
                         VECTOR2I aOrigin );
    void drawComponentBox( SCH_SCREEN* aScreen, const SD_COMPONENT& aComp );
    void drawBusBar( SCH_SCREEN* aScreen, const SD_BUS& aBus );
    void drawSignalLine( SCH_SCREEN* aScreen, const SD_SIGNAL& aSig );

    // Power section drawing
    void drawPowerSection( SCH_SCREEN* aScreen, const SYSTEM_DIAGRAM_DATA& aData,
                           VECTOR2I aOrigin );
    void drawPowerNode( SCH_SCREEN* aScreen, const SD_POWER_NODE& aNode );
    void drawPowerEdge( SCH_SCREEN* aScreen, const SD_POWER_NODE& aParent,
                        const SD_POWER_NODE& aChild );

    // Section headers
    void drawSectionHeader( SCH_SCREEN* aScreen, const wxString& aTitle, VECTOR2I aPos );
    void drawWarningLabel( SCH_SCREEN* aScreen, VECTOR2I aPos );

    SCHEMATIC* m_schematic;
};
```

#### Drawing Item Mapping

| Diagram element | SCH item type | Details |
|-----------------|---------------|---------|
| Component box | `SCH_SHAPE` (RECTANGLE) + `SCH_TEXT` | Light fill, dark border. Two text items: reference (bold) and value (normal). |
| Bus bar | `SCH_SHAPE` (RECTANGLE) | Wide, thin rectangle with alias name as `SCH_TEXT` centered on it. Thicker than a signal line. |
| Signal line | `SCH_LINE` (LAYER_NOTES) | Thin line with net name as `SCH_TEXT` label at midpoint. |
| Power source box | `SCH_SHAPE` (RECTANGLE) + `SCH_TEXT` | Reference, value, voltage label. |
| Power regulator box | `SCH_SHAPE` (RECTANGLE) + `SCH_TEXT` | Reference, value, input→output voltage. |
| Power rail box | `SCH_SHAPE` (RECTANGLE) + `SCH_TEXT` | Net name, voltage, load list. |
| Power edge | `SCH_LINE` (LAYER_NOTES) | Arrow from source/regulator output to child input. |
| Section header | `SCH_TEXT` | Large font, underlined via a horizontal `SCH_LINE`. |
| Warning label | `SCH_TEXT` | "Auto-generated — edits will be overwritten" in small italic. |

#### Sub-Sheet Management

- **Find existing**: Walk root sheet's sub-sheets looking for one named "System Diagram".
- **Create new**: Create `SCH_SHEET` + `SCH_SCREEN` pair. Add the sheet symbol to the
  root sheet at a standard position. File name: `system_diagram.kicad_sch`.
- **Clear**: Remove all items from the sheet's `SCH_SCREEN` via `DeleteAllItems()` (or
  iterate and delete, preserving the title block).
- **Undo**: Wrap the entire generation in `SCH_COMMIT` so Ctrl+Z reverts it as one step.

### 4.5 Menu and Action Integration

**Action definition** (`sch_actions.h/cpp`):

```cpp
static TOOL_ACTION generateSystemDiagram;
// Name: "eeschema.InspectionTool.generateSystemDiagram"
// Label: "Generate System Diagram"
// Tooltip: "Generate a high-level system diagram showing bus connectivity and power topology"
// Menu: Inspect → Generate System Diagram
```

**Handler** (`sch_editor_control.cpp`):

```cpp
int SCH_EDITOR_CONTROL::GenerateSystemDiagram( const TOOL_EVENT& aEvent )
{
    // 1. Ensure connectivity is up to date
    m_frame->RecalculateConnections( nullptr, GLOBAL_CLEANUP );

    // 2. Run analyzer
    SYSTEM_DIAGRAM_ANALYZER analyzer( &m_frame->Schematic() );
    if( !analyzer.Analyze() )
    {
        // Report: no buses/power found
        return 0;
    }

    // 3. Run layout
    SYSTEM_DIAGRAM_LAYOUT layout;
    layout.LayoutBusSection( analyzer.GetData() );
    layout.LayoutPowerSection( analyzer.GetData() );

    // 4. Generate drawing items
    SYSTEM_DIAGRAM_GENERATOR generator( &m_frame->Schematic() );
    SCH_SHEET* sheet = generator.Generate( analyzer.GetData() );

    // 5. Navigate to the generated sheet
    if( sheet )
        m_frame->GetToolManager()->RunAction( EE_ACTIONS::enterSheet, sheet );

    return 0;
}
```

---

## 5. The Hard Part: Auto Layout

The auto layout is the primary engineering challenge. Here's the approach in more detail.

### Why Sugiyama (Layered) for Buses

The bus connectivity graph in a typical embedded design has a natural structure:
- One or two "hub" ICs (MCU, FPGA) with many bus connections
- Peripheral ICs with few connections (often just 1 bus to the hub)
- Connectors

This naturally forms a **star or tree-like** topology, which Sugiyama handles well.
The layered approach puts the hub on the left and peripherals on the right, which
matches how engineers mentally model their designs.

### Simplified Sugiyama Implementation

Full Sugiyama has 4 phases. We simplify each:

**Phase 1 — Layer Assignment (BFS from hub):**
```
hub_component = component with max(bus_connections)
layer[hub] = 0
BFS from hub through bus edges:
    for each neighbor not yet assigned:
        layer[neighbor] = layer[current] + 1
Cap at layer 3 (anything beyond goes to layer 3)
```

**Phase 2 — Crossing Minimization (Barycenter):**
```
for iteration in 1..4:
    for each layer L from left to right:
        for each node N in layer L:
            N.y_priority = average(y_positions of neighbors in layer L-1)
        sort layer L by y_priority
    for each layer L from right to left:
        (same, using neighbors in layer L+1)
```

**Phase 3 — Coordinate Assignment:**
```
x_spacing = 400 mils (10.16 mm) between layers
y_spacing = 200 mils (5.08 mm) between nodes within a layer
for each layer L:
    x = L * x_spacing
    for each node N in layer L (in sorted order):
        N.pos = (x, y)
        y += N.height + y_spacing
```

**Phase 4 — Edge Routing:**
```
For v1: straight lines between component boxes and bus bars.
Bus bars are horizontal, centered between the two layers they span.
No bend routing or obstacle avoidance in v1.
```

### Tree Layout for Power

Power trees use a simpler recursive algorithm:

```
layoutSubtree(node, x, y):
    node.pos = (x, y)
    child_y = y
    for each child:
        layoutSubtree(child, x + x_spacing, child_y)
        child_y += subtreeHeight(child) + y_gap
    // center parent on children
    node.pos.y = average(first_child.y, last_child.y)
```

### Sizing Component Boxes

Box width and height are computed from text content:
```
width  = max(textWidth(reference), textWidth(value)) + 2 * padding
height = lineHeight * num_lines + 2 * padding
```

Text dimensions come from `KIFONT::FONT::StringBoundaryLimits()` (KiCad's existing
text measurement, used throughout the schematic editor).

### Edge Cases

- **Disconnected clusters**: Multiple independent groups of bus-connected components.
  Lay out each cluster independently, stack vertically with a gap.
- **No buses defined**: Bus section is omitted. Only power section shown.
- **No power tree**: Power section is omitted. Only bus section shown.
- **Very large designs**: If the diagram exceeds a reasonable sheet size, use multiple
  columns or a larger sheet. For v1, just use a custom sheet size.
- **Component on many buses**: The hub component appears once; multiple bus bars
  connect to it. The layout naturally handles this since the hub is in layer 0.

---

## 6. Implementation Steps

### Phase 1: Analyzer

**Files to create:**
- `eeschema/system_diagram/system_diagram_analyzer.h`
- `eeschema/system_diagram/system_diagram_analyzer.cpp`

**Files to modify:**
- `eeschema/CMakeLists.txt` — add new source files

**Work:**
- Define data structures (`SD_COMPONENT`, `SD_BUS`, `SD_SIGNAL`, `SD_POWER_NODE`, `SYSTEM_DIAGRAM_DATA`)
- Implement `discoverComponents()` — walk schematic symbols
- Implement `buildBusGraph()` — walk bus aliases, resolve connectivity
- Implement `findMarkedSignals()` — check net class assignments
- Implement `buildPowerTree()` — walk power pins, build tree
- Unit-test the analyzer with a test schematic

### Phase 2: Layout Engine

**Files to create:**
- `eeschema/system_diagram/system_diagram_layout.h`
- `eeschema/system_diagram/system_diagram_layout.cpp`

**Work:**
- Implement simplified Sugiyama for bus section (BFS layers, barycenter ordering, coordinate assignment)
- Implement recursive tree layout for power section
- Implement box sizing from text measurement
- Unit-test layout with mock graph data

### Phase 3: Generator

**Files to create:**
- `eeschema/system_diagram/system_diagram_generator.h`
- `eeschema/system_diagram/system_diagram_generator.cpp`

**Work:**
- Implement sub-sheet creation/lookup/clearing
- Implement drawing: component boxes, bus bars, signal lines, power nodes, edges, headers
- Wrap in SCH_COMMIT for undo
- Test end-to-end with a real schematic

### Phase 4: Menu Integration

**Files to modify:**
- `eeschema/tools/sch_actions.h` — add `generateSystemDiagram` action
- `eeschema/tools/sch_actions.cpp` — define the action
- `eeschema/tools/sch_editor_control.h` — add handler declaration
- `eeschema/tools/sch_editor_control.cpp` — implement handler
- `eeschema/menubar.cpp` — add to Inspect menu

---

## 7. File Summary

### New Files (6)

```
eeschema/system_diagram/
├── system_diagram_analyzer.h
├── system_diagram_analyzer.cpp
├── system_diagram_layout.h
├── system_diagram_layout.cpp
├── system_diagram_generator.h
└── system_diagram_generator.cpp
```

### Modified Files (5)

```
eeschema/CMakeLists.txt              — add system_diagram/ source files
eeschema/tools/sch_actions.h         — add generateSystemDiagram action
eeschema/tools/sch_actions.cpp       — define the action
eeschema/tools/sch_editor_control.h  — add handler declaration
eeschema/tools/sch_editor_control.cpp — implement handler
eeschema/menubar.cpp                 — add Inspect menu entry
```

---

## 8. Design Decisions

### Q: Why bus aliases only, not auto-detected interfaces?
A: Auto-detection requires classifying pin functions (SPI_CLK vs GPIO) which is
fragile and error-prone. Bus aliases are already a first-class KiCad concept that
users define in Schematic Setup. If the user defined the alias, they intended it
to be a logical bus — perfect trigger for the diagram. No guessing.

### Q: Why net class for "specially marked" signals instead of a custom field?
A: Net classes already exist in KiCad and can be assigned to nets in Schematic Setup
or via net label properties. No new UI needed. The user adds a net to the
`System_Diagram` net class and it appears on the diagram. If we later want a custom
field approach, it's additive.

### Q: Why no pins on the diagram?
A: The system diagram is a high-level overview. Pin-level detail belongs on the
actual schematic sheets. Showing pins would clutter the diagram and defeat its
purpose. The component boxes show reference + value, which is enough to identify
what each block is.

### Q: Why not use an external graph layout library?
A: KiCad avoids external dependencies where possible. The graph we're laying out
is small (typically <30 nodes, <50 edges for even a complex design) and structurally
simple (star/tree topology). A simplified Sugiyama implementation is ~300-400 lines
of code and gives good results for this class of graph. If the layout quality proves
insufficient, we can always swap in a more sophisticated algorithm later.

### Q: What about regeneration — does it clobber user edits?
A: Yes. The sheet is fully auto-generated. A warning text on the sheet says
"Auto-generated — edits will be overwritten on next generation." This is the same
approach KiCad uses for other generated outputs (BOM, netlist). Users who want
a custom block diagram should create a regular sheet.

### Q: How does this interact with hierarchical designs?
A: The analyzer walks all sheets in the hierarchy. Components from any sheet can
appear on the diagram. The bus aliases can be defined on any sheet. The generated
diagram is a flat overview of the entire design — it intentionally collapses the
hierarchy into a single high-level view.

---

## 9. Deferred Enhancements

These are natural extensions that build on v1 but are explicitly out of scope:

- **Current accounting**: Add `Current_Draw`, `Max_Current` fields. Show utilization
  bars on power rail boxes. Color-code by utilization.
- **Power ERC checks**: `ERCE_POWER_RAIL_OVERLOAD`, `ERCE_POWER_MISSING_ANNOTATION`, etc.
- **JSON export**: Serialize the power model for CI/automation and PDN analyzer integration.
- **Settings dialog**: Let users configure net class name, layout spacing, sheet size
  preferences, which reference prefixes to include.
- **Pin-level detail mode**: Optional mode that shows pin names on bus connections.
- **Cross-reference links**: Clicking a component box navigates to it in the schematic.
- **PCB integration**: Share power model with pcbnew's PDN analyzer via project object.
- **Interactive layout adjustment**: Let users drag boxes on the generated sheet, with
  positions remembered across regenerations.
- **Net class auto-coloring**: Color bus bars by net class color.
- **Multiple diagram sheets**: Split bus and power into separate sheets for very large
  designs.
