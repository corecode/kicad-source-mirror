# Sub-Region BEM Debug Report

## Setup

Geometry per the spec: h=0.1mm substrate, w=0.15mm conductor, t=35µm copper, εr=4.4 FR4.

Debug test builds the system by hand with 36 conductor panels + 50 interface panels = 86×86 matrix. Every entry computed from the spec's formulas. Solved with Eigen FullPivLU.

Expected: εr_eff ≈ 3.27 (Hammerstad-Jensen), Z₀ ≈ 48Ω.
Got: εr_eff = 1.20, Z₀ = 84Ω.

## The Matrix Structure

The system `A × σ = b` has two types of rows:

**Conductor rows** (rows 0..35): enforce `Φ(r_i) = V_i`
```
A[i][j] = ∫ G(r_i, r') dl'
```
G = -1/(2πε₀) × [ln|r-r'| - ln|r-r''|] contains the 1/ε₀ factor.

Typical diagonal: A[C_bot][C_bot] = **8.4 × 10⁵**
Typical conductor-interface off-diagonal: A[C_bot][I_first] = **2.7 × 10⁴**

**Interface rows** (rows 36..85): enforce `ε₁ ∂Φ/∂n|₊ - ε₂ ∂Φ/∂n|₋ = 0`
```
A[i][j] = (ε₁-ε₂) × ∂G/∂n × dl_j    (off-diagonal)
A[i][i] = -(ε₁+ε₂)/(2ε₀)             (diagonal, jump term)
```
The (ε₁-ε₂) prefactor has units F/m, and ∂G/∂n has units 1/F (because G has 1/ε₀). These cancel, leaving the interface rows **dimensionless**.

Typical diagonal: A[I_first][I_first] = **-2.75**
Typical interface-conductor off-diagonal: A[I_first][C_bot] = **-0.004**

## The Scale Mismatch

| Entry type | Magnitude | Units |
|-----------|-----------|-------|
| Conductor diagonal | 10⁵ – 10⁶ | m²/F |
| Conductor-interface | 10⁴ | m²/F |
| Interface diagonal | ~3 | dimensionless |
| Interface-conductor | ~0.004 | dimensionless |
| Interface-interface | ~0.05 | dimensionless |

The conductor block and interface block live on scales that differ by **5 orders of magnitude**.

## What Happens When We Solve

The solution is:
- Conductor σ ≈ 10⁻⁷ C/m² (free charge density)
- Interface σ ≈ 10⁻⁸ C/m² (bound charge density)

The interface charge is about 1/10 of the conductor charge — physically reasonable for εr = 4.4.

**But:** the contribution of interface charge to the conductor potential equation is:

```
Σ_j A[C][I_j] × σ_{I_j} ≈ 2.7×10⁴ × (-10⁻⁸) × 50 panels ≈ -0.014
```

The RHS of the conductor equation is **1.0** (volts). So the interface charge shifts the conductor potential by **1.4%**. This is why εr_eff ≈ 1.2 — the interface contributes a small perturbation rather than the factor-of-3 enhancement it should.

## Why This Is Wrong

In reality, the dielectric should enhance the capacitance by a factor of ~3.3. The bound charge on the interface should significantly alter the conductor charge distribution. The BEM system should produce this naturally — the interface equations couple bidirectionally with the conductor equations.

The coupling IS bidirectional, but the magnitudes are imbalanced:
- **Interface → conductor effect**: A[C][I] × σ_I ≈ 0.014 out of 1.0 → 1.4% effect
- **Conductor → interface effect**: A[I][C] × σ_C ≈ 0.004 × 10⁻⁷ ≈ 4×10⁻¹⁰ out of 0 → determines σ_I entirely

The interface charge σ_I is entirely determined by the conductor charge (via A[I][C] × σ_C), but σ_I barely feeds back into the conductor equation (via A[C][I] × σ_I). The loop gain is << 1.

## Possible Causes

### 1. The G kernel includes 1/ε₀ in both rows

The conductor rows use `G × dl` which has a 1/ε₀ factor, making entries ~10⁵. The interface rows use `(ε₁-ε₂) × ∂G/∂n × dl` where (ε₁-ε₂) ∝ ε₀ cancels the 1/ε₀ in ∂G/∂n, leaving entries ~1. This inherent asymmetry might be correct mathematically but numerically problematic.

**Question:** Should the G kernel be defined WITHOUT ε₀ (as `G̃ = -1/(2π) × [ln|r| - ln|r''|]`), with ε₀ factored into the equations differently? Some BEM formulations use `G̃` to keep all matrix entries on the same scale.

### 2. The interface-conductor coupling is too weak

A[I_first][C_bot] = -0.004. This is `(ε₁-ε₂) × ∂G/∂n × dl_C`. The ∂G/∂n at the interface (y=0) from a conductor bottom panel (also at y=0) involves only the image term (direct vanishes when both are at y=0). The image is at y = -2h = -0.2mm. So:

```
∂G_image/∂n = 1/(2πε₀) × 2h / ((x_i-x_j)² + (2h)²)
```

For x_i - x_j ≈ 0.5mm (interface panel to conductor panel), this gives:
```
= 1/(5.56×10⁻¹¹) × 2×10⁻⁴ / (2.5×10⁻⁷ + 4×10⁻⁸) = 1.8×10¹⁰ × 6.9×10⁻⁴ = 1.24×10⁷
```

Then `(ε₁-ε₂) × ∂G/∂n × dl = -3.01×10⁻¹¹ × 1.24×10⁷ × 10⁻⁵ = -0.004`. ✓

The math checks out. The small value comes from the ε₀ cancellation: (ε₁-ε₂) ∝ ε₀, and ∂G/∂n ∝ 1/ε₀. Their product is O(1). But the conductor equations have G ∝ 1/ε₀ without the cancellation, so they're O(10⁵).

### 3. This might be a known issue with mixed potential/flux BEM

The system mixes a **potential equation** (conductor rows, `∫ G σ dl = V`) with a **flux equation** (interface rows, `ε₁∂Φ/∂n|₊ - ε₂∂Φ/∂n|₋ = 0`). These have different physical dimensions. Standard sub-region BEM references may use a different formulation — e.g., both row types as potential equations, or a normalized version.

## What to Check Next

1. **Look at actual sub-region BEM reference implementations** (e.g., TNT-MMTL source code, or the BEM formulation in Harrington's textbook) to see how they handle the conductor/interface row scaling.

2. **Try removing ε₀ from G**: Use `G̃ = -1/(2π) × [ln|r| - ln|r''|]`, then the conductor equation becomes `∫ G̃ σ dl = V × ε₀`. The interface equation becomes `(εr₁-εr₂) × ∂G̃/∂n × dl` with all relative εr values. This might balance the scales.

3. **Try double-layer potential formulation**: Instead of single-layer (charge density σ) for interface panels, use a double-layer (dipole density μ). The double-layer formulation for dielectric interfaces has the form `(1+εr)/(2) μ_i + ∫ ∂G/∂n μ dl = -∫ G σ_c dl`, which might be better conditioned.

## Concrete Numbers for Verification

If someone reimplements and gets different results, compare these values:

| Quantity | Our Value | Expected |
|----------|----------|----------|
| C_air | 28.3 pF/m | ~36 pF/m (2 panels are too few for convergence) |
| C_physical | 30.2 pF/m | ~118 pF/m |
| εr_eff | 1.07 – 1.20 | 3.27 |
| σ_conductor (bottom) | 1.6×10⁻⁷ C/m² | — |
| σ_interface (nearest) | -2.6×10⁻⁸ C/m² | — |
| Interface ΔΦ contribution | 1.4% of V | should be ~70% |
