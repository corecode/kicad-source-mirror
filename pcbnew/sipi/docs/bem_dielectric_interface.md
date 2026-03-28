# BEM Dielectric Interface Handling — Status and Path Forward

## The Problem

PCB microstrip traces sit directly on the substrate surface. The conductor bottom face touches the dielectric interface — air (εr=1) above, FR4 (εr≈4.4) below. The BEM solver needs to correctly account for this two-medium environment to compute per-unit-length capacitance C and characteristic impedance Z₀.

## Geometry

```
        AIR  εr = 1
    ┌─────────────┐  ← conductor (width w, thickness t)
────┴─────────────┴──── dielectric interface (y = -h)
        FR4  εr = 4.4
═════════════════════ ground plane (y = 0)
```

The conductor bottom face is AT the interface. Top face is t above the interface. Both sides have panels in the BEM discretization.

## What We Tried

### 1. Uniform dielectric (εr everywhere)

Set εr = 4.4 for the entire domain. This overcounts the dielectric — the field above the conductor is in air, not FR4. Result: εr_eff = εr = 4.4 (too high), Z₀ too low.

### 2. Dielectric image series (Weeks 1970)

The rigorous approach for a source ABOVE a dielectric interface backed by a ground plane. Derived from the spectral-domain solution of the two-region BVP:

```
G = -1/(4πε₀) × [ lnR²(source)
                  + k × lnR²(D₀ image)
                  - (1-k²) × Σ k^m × lnR²(combined ground image m) ]

where k = (1-εr)/(1+εr), D₀ at y = 2·y_interface - y_source
```

This is exact for sources strictly in air (y_s > interface). For a single conductor, it gave εr_eff ≈ 3.9 (reasonable). **But it diverges for panels AT the interface** — the D₀ image of a bottom-face panel lands at the same position as the panel itself, creating infinite self-energy.

For multiple conductors, this divergence corrupted the entire capacitance matrix: C₁₁ dropped by 12% when adding a neighbor (should increase), producing inverted coupling.

### 3. Current: interface-average εr = (ε₁+ε₂)/2

The classical exact result for a line charge AT a planar dielectric interface (Jackson, Electrodynamics, Ch. 4.4). The potential on both sides of the interface is equivalent to a charge in a uniform medium with εr_eff = (ε₁+ε₂)/2.

For microstrip on FR4: εr_eff = (1 + 4.4)/2 = 2.7.

**What this gets right:**
- Coupling direction: adding a grounded neighbor increases C₁₁, lowering Z₀ ✓
- Coupling magnitude: Z₀ drops ~1Ω for a neighbor at 0.3mm ✓
- Convergence: the BEM matrix is well-conditioned, same as the uniform case ✓
- Impedance variation along a trace: correct shape (relative differences are accurate) ✓

**What this gets wrong:**
- εr_eff = 2.94 vs Hammerstad-Jensen 3.27 (10% low)
- Ignores the geometry dependence of εr_eff on w/h ratio
- The ground plane pulls more field into the dielectric than a simple average captures
- Absolute Z₀ is ~5Ω too high (53.7 vs analytical ~48Ω)

## The Right Fix: Sub-Region BEM with Interface Elements

Instead of images, explicitly discretize the dielectric interface with boundary elements that enforce the continuity condition:

```
ε₁ × ∂Φ/∂n |_above = ε₂ × ∂Φ/∂n |_below
```

The interface is a horizontal line from (x_min, y_interface) to (x_max, y_interface). It gets ~75–150 constant-charge panels. The interface panels carry "bound charge" — they don't have a prescribed voltage, instead they have the continuity constraint as their equation.

### Modified BEM system

With N_c conductor panels and N_i interface panels:

```
[ A_cc  A_ci ] [ σ_c ]   [ V_c ]
[ A_ic  A_ii ] [ σ_i ] = [  0  ]
```

- A_cc: conductor-conductor interactions (same as current BEM, using vacuum Green's function)
- A_ci: conductor-interface interactions
- A_ic: interface-conductor interactions
- A_ii: interface-interface interactions
- σ_c: charge density on conductor panels (unknowns)
- σ_i: bound charge on interface panels (unknowns)
- V_c: prescribed voltage on conductors (1V or 0V)
- The interface equation row enforces: the potential jump across the interface matches the ε₁/ε₂ ratio of the normal derivatives

The Green's function for ALL interactions is the vacuum+ground-plane kernel:
```
G(r, r') = -1/(4πε₀) × [ln|r-r'| - ln|r-r_image'|]
```

No dielectric images. The dielectric effect emerges from the bound charge on the interface panels.

### Advantages

- Works for conductors AT the interface (no divergence)
- Works for any number of conductors
- Handles conductors partially embedded in the dielectric
- Captures the geometry-dependent εr_eff (w/h ratio effect)
- System size: ~80 conductor panels + ~100 interface panels = 180×180 matrix. Still sub-millisecond.

### Implementation estimate

- Add `XS_INTERFACE` to `XS_GEOMETRY`
- Extend `buildPanels()` to generate interface panels
- Modify `fillCoefficientMatrix()` to build the partitioned system
- The interface equation: for panel i on the interface, the equation is that the potential is continuous (same as other panels) but the charge represents bound surface charge
- Actually, the simpler formulation: use two layers of panels at the interface (one for each side), with the constraint that ε₁σ₁ + ε₂σ₂ = 0 (no free charge at the interface)

### References

- Weeks, W.T., "Calculation of Coefficients of Capacitance of Multiconductor Transmission Lines in the Presence of a Dielectric Interface," IEEE Trans. MTT, 1970
- Silvester, P. & Ferrari, R., "Finite Elements for Electrical Engineers," Ch. 6
- Harrington, R.F., "Field Computation by Moment Methods," Ch. 4
- Balanis, C., "Advanced Engineering Electromagnetics," Ch. 12
- IPC-2141, "Design Guide for High-Speed Controlled Impedance Circuit Boards"
