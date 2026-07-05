# Design: magnetic core loss (iGSE) + electro-thermal self-heating

Status: proposal. Addresses the two **critical** fidelity gaps from the code
review (`REVIEW_FINDINGS.md`): inductor loss is DCR-copper-only (no core loss),
and all losses are computed at 25 °C nominal parameters (no self-heating), so
efficiency and thermal numbers are systematically optimistic — worst exactly
where a designer decides whether the thermal design closes.

Scope: buck/boost/buck-boost (`buck-sim-ui/src/sim.rs::compute_losses`) and PFC
(`buck-sim-ui/src/pfc_sim.rs`). New physics lives in a reusable
`electronics-sim/src/core_loss.rs` and `electronics-sim/src/thermal.rs`.

Back-compat: every new field is `#[serde(default)]` (= 0). With defaults the
result is **identical to today** — zero core loss, isothermal 25 °C. The models
are strictly opt-in per component.

---

## 1. Physics

### 1.1 Core loss — improved Generalized Steinmetz (iGSE)

Classic Steinmetz `Pv = k·fᵅ·B̂ᵝ` (W/m³) is only valid for **sinusoidal** flux.
Switching-converter flux is **triangular** (and, for PFC, a triangular ripple
riding a 100/120 Hz envelope), so we use iGSE:

```
Pv = (1/T) ∫₀ᵀ ki · |dB/dt|ᵅ · (ΔB)^(β−α) dt          [W/m³]

ki = k / [ (2π)^(α−1) · ∫₀^(2π) |cosθ|ᵅ dθ · 2^(β−α) ]
   ≈ k / [ 2^(β+1) · π^(α−1) · (0.2761 + 1.7061/(α+1.354)) ]
```

For a **piecewise-linear (triangular)** B(t) with an on-segment of length `D·T`
and an off-segment of length `(1−D)·T`, both with flux swing `ΔB`, the integral
is analytic:

```
Pv = ki · (ΔB)ᵝ · f_swᵅ · [ D^(1−α) + (1−D)^(1−α) ]     [W/m³]
P_core = Pv · Ve                                          [W]
```

where the flux swing comes from the ripple the sim already computes:

```
ΔB = L(I) · ΔI_pp / (N · Ae)        ΔI_pp = i_max − i_min (per phase)
B_dc = L(I) · I_avg / (N · Ae)      (for saturation / DC-bias premag)
```

`N` = turns, `Ae` = effective core area [m²], `Ve` = effective core volume [m³].
This closed form needs only `(D, ΔI_pp, f_sw, L)` — all present per cycle.

**PFC:** don't use a single operating point. Step the existing line-cycle loop
(`pfc_sim.rs`), evaluate the triangular-segment iGSE per switching cycle with the
*local* `D(t)` and `ΔB(t)`, and average over the line period. This captures the
dominant minor-loop swing correctly (the reason PFC chokes need iGSE, not
sinusoidal Steinmetz).

Optional refinements (phase 2): DC-bias premagnetization multiplier; temperature
coefficient `Pv(T) = Pv₂₅·(1 + a·(T_core−25) + b·(T_core−25)²)` (ferrites have a
loss minimum near 80–100 °C).

### 1.2 AC winding loss — Dowell (phase 2)

DCR against full ripple RMS under-counts HF loss. Add
`Rac(f)/Rdc = Δ·[ (sinh2Δ+sin2Δ)/(cosh2Δ−cos2Δ) + (2(m²−1)/3)(sinh Δ−sin Δ)/(cosh Δ+cos Δ) ]`
with `Δ = h/δ`, `δ = √(ρ/(π f μ₀))`, `m` = layers. Apply per harmonic of the
triangular current (or a single effective `Rac` at `f_sw` for a first cut):
`P_ac = Σ_h Rac(h·f_sw)·I_h,rms²`. The DC term keeps using `Rdc`.

### 1.3 Electro-thermal self-heating

Temperature-dependent device parameters (all linearized about 25 °C):

| Parameter | Model | Typical |
|-----------|-------|---------|
| MOSFET Rds(on) | `Rds25·(1 + α_rds·(Tj−25))` | Si α≈0.004–0.006; GaN α≈0.006–0.010 /°C |
| Diode Vf | `Vf25 + β_vf·(Tj−25)` | β_vf ≈ −2 mV/°C |
| Cap ESR | `ESR25·(1 + α_esr·(Tj−25))` | electrolytic strongly +; MLCC ~flat |
| Inductor DCR | `DCR25·(1 + 0.00393·(Tcu−25))` | copper +0.393 %/°C |

**Steady-state coupling** — a fixed-point iteration (replaces the single-shot
`compute_losses`):

```
Tj[dev] = Tamb                       # seed
repeat (≤ 8×, until max|ΔTj| < 0.1 °C):
    P[dev] = loss_terms(params(Tj))  # Rds(Tj), Vf(Tj), ESR(Tj), DCR(Tcu)
    Tj[dev] = Tamb + Rth_ja[dev] · P[dev]
```

Converges when `dP/dTj · Rth_ja < 1`. If it does **not** converge (Rds tempco ×
Rth too high), that *is* thermal runaway — surface it as a warning/flag instead
of silently iterating to the cap. This is a genuinely useful new output.

`Rth_ja = Rth_jc + Rth_ca` (junction-to-case from datasheet + case-to-ambient
from the heatsink/board design). Phase 2: shared-heatsink coupling matrix so HS
and LS heat each other.

**Transient (phase 3):** Foster network `Zth(t) = Σ Ri(1−e^(−t/τi))` for
load-step junction temperature and short-term overload ratings.

---

## 2. Data model

### 2.1 `electronics-sim/src/core_loss.rs` (new)

```rust
#[derive(Debug, Clone, Copy)]
pub struct SteinmetzParams { pub k: f64, pub alpha: f64, pub beta: f64 } // SI, W/m³

#[derive(Debug, Clone)]
pub struct MagneticsProfile {
    pub steinmetz: SteinmetzParams,
    pub n_turns:  f64,
    pub a_e_m2:   f64,   // effective area
    pub v_e_m3:   f64,   // effective volume
    pub l_e_m:    f64,   // effective path length (H-field / saturation)
    pub b_sat_t:  f64,   // saturation flux density (flag if exceeded)
    // winding (phase 2)
    pub dc_resistance_ohm: f64,
    pub winding: Option<DowellParams>,
}
impl MagneticsProfile {
    pub fn ki(&self) -> f64 { /* eq. in §1.1 */ }
    /// Core loss [W] for one triangular-ripple operating point.
    pub fn core_loss_triangular(&self, l_h: f64, duty: f64, di_pp: f64, f_sw: f64) -> f64 { /* §1.1 */ }
    pub fn b_swing(&self, l_h: f64, di_pp: f64) -> f64 { l_h * di_pp / (self.n_turns * self.a_e_m2) }
    pub fn b_dc(&self, l_h: f64, i_avg: f64) -> f64 { l_h * i_avg / (self.n_turns * self.a_e_m2) }
}
```

Attach to the existing `InductorModel` (`electronics-sim/src/lib.rs`) as
`pub magnetics: Option<MagneticsProfile>` — `InductorModel` keeps owning `L(I)`;
`MagneticsProfile` owns loss + geometry. `None` ⇒ no core loss (today's behavior).

### 2.2 `FetProfile` additions (`buck-sim-ui/src/sim.rs:74`)

```rust
#[serde(default)] pub rds_tempco_per_c: f64,  // 0 ⇒ isothermal (default)
#[serde(default)] pub r_th_jc_c_per_w: f64,
// Coss energy — lets us fix the confirmed hs_eoss ½CV² / single-side bug:
#[serde(default)] pub eoss_nj: f64,           // datasheet Eoss …
#[serde(default)] pub v_eoss_test_v: f64,     // … at this V_DS (0 ⇒ fall back to ½·Coss·V²)
```

### 2.3 `ThermalParams` (new, on `SimParams`)

```rust
#[derive(Debug, Clone)] pub struct ThermalParams {
    pub t_ambient_c: f64,          // default 25.0
    pub r_th_ca_hs_c_per_w: f64,   // case→ambient per device (heatsink/board)
    pub r_th_ca_ls_c_per_w: f64,
    pub r_th_ind_c_per_w: f64,     // inductor→ambient
}
```
`Default` = 25 °C ambient, all Rth_ca = 0 ⇒ with Rth_jc=0 → Tj=Tamb → isothermal.

### 2.4 `LossBreakdown` additions (`buck-sim-ui/src/sim.rs:1322`)

```rust
pub core_loss_w:  f64,
pub ac_winding_w: f64,   // phase 2
pub t_j_hs_c: f64,
pub t_j_ls_c: f64,
pub t_core_c: f64,
pub thermal_runaway: bool,   // iteration failed to converge
```

---

## 3. `compute_losses` restructure (buck)

Wrap the existing body (`sim.rs:1420–1454`) in the §1.3 fixed-point loop:

```rust
let mut tj_hs = tamb; let mut tj_ls = tamb; let mut tcu = tamb;
for _ in 0..8 {
    let rds_hs = p.hs_fet.rds_on_mohm*1e-3 * (1.0 + p.hs_fet.rds_tempco_per_c*(tj_hs-25.0));
    let rds_ls = p.ls_fet.rds_on_mohm*1e-3 * (1.0 + p.ls_fet.rds_tempco_per_c*(tj_ls-25.0));
    let dcr    = p.dcr_mohm*1e-3          * (1.0 + 0.00393*(tcu-25.0));
    let hs_conduction = rds_hs * avg_d       * per_phase_i_rms2 * n_phases;
    let ls_conduction = rds_ls * (1.0-avg_d) * per_phase_i_rms2 * n_phases;
    let inductor_dcr  = dcr    * per_phase_i_rms2 * n_phases;
    let core = p.inductor_magnetics.as_ref()
        .map(|m| m.core_loss_triangular(l_eff, avg_d, avg_i_max-avg_i_min, f_sw) * n_phases)
        .unwrap_or(0.0);
    // hs_eoss: sum BOTH FET Coss (fixes confirmed bug) or use datasheet Eoss(V)
    let hs_eoss = eoss_energy(&p.hs_fet, v_in) + eoss_energy(&p.ls_fet, v_in);
    // … switching, gate, ringing as before …
    let (new_tj_hs, new_tj_ls, new_tcu) = (
        tamb + (p.hs_fet.r_th_jc_c_per_w + th.r_th_ca_hs_c_per_w)*(hs_conduction/n_phases + hs_sw_per_dev),
        tamb + (p.ls_fet.r_th_jc_c_per_w + th.r_th_ca_ls_c_per_w)*(ls_conduction/n_phases + ls_sw_per_dev),
        tamb + th.r_th_ind_c_per_w*((inductor_dcr + core)/n_phases),
    );
    if max_delta(tj_hs,new_tj_hs, …) < 0.1 { converged = true; … break }
    tj_hs = new_tj_hs; tj_ls = new_tj_ls; tcu = new_tcu;
}
```

`eoss_energy()` returns `eoss_nj·1e-9·(V/v_eoss_test)^~2.2` when a datasheet Eoss
is given (nonlinear Coss(V) scaling), else `0.5·Coss·V²` — folding in the
confirmed `hs_eoss` findings (single-side Coss + flat-Coss).

PFC (`pfc_sim.rs`): same fixed-point wrapper around its loss sums, plus the
per-line-cycle iGSE integral, plus the missing switching / reverse-recovery terms
(several confirmed PFC-loss findings resolved together here).

---

## 4. Rollout (each increment compiles + tests green)

- **Inc 0** — `core_loss.rs`: `SteinmetzParams`, `MagneticsProfile`, `ki()`,
  `core_loss_triangular()`. Unit tests vs published ferrite curves (e.g. 3C95 /
  N87 at a datasheet `(f, B̂)` point) and vs the sinusoidal limit. No wiring.
- **Inc 1** — `InductorModel.magnetics`; `core_loss_w` in `LossBreakdown`; wire
  into buck `compute_losses`. Test: at light load, core loss dominates DCR.
- **Inc 2** — electro-thermal fixed-point (Rds(Tj), DCR(Tcu)); `t_j_*`,
  `thermal_runaway`. Tests: hot Rds raises conduction loss & drops η; converges;
  runaway case flags.
- **Inc 3** — `eoss_energy()` fix (HS+LS Coss, datasheet Eoss(V)); closes the
  `hs_eoss` findings.
- **Inc 4** — PFC path: line-cycle iGSE + switching + reverse-recovery loss in
  `pfc_sim.rs`; closes the PFC efficiency-model findings.
- **Inc 5** — AC winding loss (Dowell) + core-loss temp coefficient.
- **Inc 6** — transient Foster `Zth(t)` for load-step Tj / overload ratings.
- **Inc 7** — UI (`app.rs`): magnetics + thermal inputs, Tj / core-loss readouts,
  presets with real datasheet magnetics & Rth.

## 5. Validation

- Datasheet cross-checks: ferrite `Pv(f, B̂)` points (< ~15 %); MOSFET
  `Rds(Tj)/Rds25`; a full worked buck loss budget vs a hand calc.
- Invariants: `η = Pout/(Pout+Ploss)` monotone in each loss term; Tj ≥ Tamb;
  isothermal fixed-point (all tempco = 0) reproduces current numbers exactly.
- Reuse `monte_carlo.rs` to sweep `(Tamb, Rth)` corners once wired.

## 6. Interactions with review findings

Implementing this **closes** these confirmed items: no-core-loss (×crit), no
electro-thermal self-heating (×crit), no thermal RC/Zth, AC winding loss,
`hs_eoss` single-side/flat-Coss, PFC efficiency omits switching/reverse-recovery/
core. It also gives `protection.rs` a real `Tj` to drive the (currently missing)
OTP / thermal-foldback path.
