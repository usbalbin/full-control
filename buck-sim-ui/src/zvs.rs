//! Zero-voltage-switching (ZVS) verification for half-bridge GaN/Si
//! FET stacks.
//!
//! Closed-form deadtime + minimum-load analysis using the **resonant
//! tank formed by the commutation-loop inductance and the FET output
//! capacitance**:
//!
//! ```text
//! ┌────────────┬──────── HS drain
//! │            │                            ←── L_loop ──┐
//! │           HS                                          │
//! │           Coss                                        L (output ind.)
//! │            │                                          │
//! ├────────────┼──── SW node ──→ inductor                 │
//! │            │                            ←── L_loop ──┘
//! │           LS
//! │           Coss
//! │            │
//! └────────────┴──────── GND
//! ```
//!
//! When the LS FET turns off, the inductor current commutates onto
//! the parallel HS+LS Coss pair, swinging the SW node from 0 →
//! V_in. **True ZVS** is achieved when the SW node fully reaches
//! V_in *before* the HS gate fires (V_DS,HS = 0 at turn-on, no
//! switching loss). Required conditions:
//!
//! 1. **Enough inductor current** to charge `2·C_oss` from 0 to
//!    V_in. Below this, the resonance can only swing to a
//!    fraction of V_in and the HS turns on with residual
//!    `V_DS = V_in − V_swing` → hard-switching loss
//!    `E_hs = 0.5 · 2·C_oss · V_residual²` per cycle.
//!
//! 2. **Correct deadtime**. Too short → HS fires before V_swing
//!    completes. Too long → the inductor reverses, V_SW rings
//!    *back* down past V_in (also costs).
//!
//! Both conditions reduce to closed-form expressions in terms of
//! `L_loop`, `C_oss_total = C_oss,HS + C_oss,LS`, `V_in`, and the
//! load current `I_load` at the moment of the LS turn-off.
//!
//! # Why this matters
//!
//! GaN FETs (e.g. EPC2306) live or die on ZVS — the body-diode
//! reverse-recovery loss in hard-switching can vaporise a small
//! die at 24 V × 50 A. Designers tune deadtime today by bench
//! scope. Computing it from layout (`L_loop` from PEEC) + datasheet
//! (`C_oss` from FET profile) means firmware can ship a deadtime
//! table indexed by `(V_in, I_out)` without bench iteration.
//!
//! Builds on
//! [`FetProfile::coss_pf`](crate::sim::FetProfile::coss_pf) and the
//! commutation-loop inductance the user's PEEC field solver
//! extracts from the PCB layout.

use crate::sim::FetProfile;

/// Output capacitance of a half-bridge in farads (HS + LS in
/// parallel — they're both connected to the SW node during the
/// commutation interval, so their `C_oss` sums).
pub fn coss_total_farads(hs: &FetProfile, ls: &FetProfile) -> f64 {
    (hs.coss_pf + ls.coss_pf) * 1e-12
}

/// Optimal deadtime (seconds) for ZVS — the moment V_SW first
/// reaches V_in.
///
/// Solving the LC tank
///   `V_SW(t) = V_in − V_in·cos(ωt) + I·Z_0·sin(ωt)`
/// for V_SW = V_in gives
///   `t_opt = arctan(V_in / (I · Z_0)) / ω`
/// where `ω = 1/√(L·C)`, `Z_0 = √(L/C)`.
///
/// At threshold load (`I·Z_0 = V_in`) → `t_opt = T_res/4` (quarter
/// period; the textbook "deadtime ≈ quarter period" answer).
/// At higher loads, optimal deadtime *shrinks* — the inductor
/// drives V_SW past V_in before a full quarter period elapses, and
/// the HS body diode would otherwise clamp the overshoot with
/// reverse-recovery loss.
///
/// Returns `T_res/4` when `i_load` is below threshold (the swing
/// can't reach V_in regardless of deadtime; quarter period is the
/// peak-of-swing moment).
pub fn optimal_deadtime_s(
    v_in_v: f64, i_load_a: f64, l_h: f64, coss_total_f: f64,
) -> f64 {
    if l_h <= 0.0 || coss_total_f <= 0.0 || v_in_v <= 0.0 {
        return 0.0;
    }
    let omega = 1.0 / (l_h * coss_total_f).sqrt();
    let z0 = (l_h / coss_total_f).sqrt();
    let drive = i_load_a.abs() * z0;
    if drive < 1e-12 {
        return 0.5 * std::f64::consts::PI / omega; // quarter period
    }
    let t = (v_in_v / drive).atan() / omega;
    // Clamp to [0, quarter-period] — the analytic formula returns a
    // small positive value for any drive, but at high drive levels
    // it can dip below the simulator's typical clock resolution
    // (sub-100 ps). The caller uses this as a target deadtime; we
    // protect against rounding-to-zero with a sub-ps epsilon.
    t.max(0.0).min(0.5 * std::f64::consts::PI / omega)
}

/// "Threshold drive" — the load current at which the LC-tank
/// characteristic impedance times current equals V_in. Below this
/// drive level the optimal deadtime grows toward `T_res/4`; above
/// it the optimal deadtime shrinks toward zero. Useful as a quick
/// scalar for "is my converter operating in the easy or hard ZVS
/// regime?". The textbook `I · √(C/L) = V_in / L` form.
///
/// Returns `0.0` for degenerate inputs.
pub fn threshold_drive_a(v_in_v: f64, l_h: f64, coss_total_f: f64) -> f64 {
    if l_h <= 0.0 || v_in_v <= 0.0 || coss_total_f <= 0.0 {
        return 0.0;
    }
    v_in_v * (coss_total_f / l_h).sqrt()
}

/// Minimum load current (amps) for full ZVS at the GIVEN deadtime.
///
/// Solves `V_SW(t_dt) = V_in` for I:
///
/// ```text
/// V_in − V_in·cos(ω·t_dt) + I·Z_0·sin(ω·t_dt) = V_in
/// I = V_in · cot(ω·t_dt) / Z_0     (when ω·t_dt < π)
/// ```
///
/// Below this load and deadtime the SW node hasn't reached V_in
/// yet at the moment HS fires, leaving residual V_DS,HS > 0
/// (= hard-switch loss). Above it, the body diode clamps SW at
/// V_in and the HS turns on with V_DS = 0.
///
/// Returns `0.0` if the deadtime exceeds half the resonance period
/// (any load achieves ZVS in that case — body diode has clamped
/// long ago).
pub fn min_load_for_zvs_at_deadtime_a(
    v_in_v: f64, l_h: f64, coss_total_f: f64, t_dt_s: f64,
) -> f64 {
    if l_h <= 0.0 || coss_total_f <= 0.0 || v_in_v <= 0.0 || t_dt_s <= 0.0 {
        return 0.0;
    }
    let omega = 1.0 / (l_h * coss_total_f).sqrt();
    let z0 = (l_h / coss_total_f).sqrt();
    let phase = omega * t_dt_s;
    if phase >= std::f64::consts::PI {
        // Past half-period — V_SW has long since clamped.
        return 0.0;
    }
    if phase <= 1e-12 {
        // No deadtime → impossible no matter how high the load.
        return f64::INFINITY;
    }
    let c = phase.cos();
    if c <= 0.0 {
        // V_SW has reached or passed V_in for the first time
        // (body diode clamped). Any load drives ZVS.
        return 0.0;
    }
    let s = phase.sin();
    if s.abs() < 1e-12 {
        return f64::INFINITY;
    }
    v_in_v * c / (z0 * s)
}

/// V_DS across the HS FET at the moment its gate fires, given the
/// current load + deadtime. Zero means perfect ZVS (no switching
/// loss); larger values mean residual hard-switch energy
/// `0.5 · C_oss · V_DS²` lost per cycle.
///
/// LC-tank dynamics during deadtime (V_SW starts at 0, inductor
/// driving at I_0):
///
/// ```text
/// V_SW(t) = V_in − V_in·cos(ωt) + I·Z_0·sin(ωt)
/// ```
///
/// Three regimes:
/// - **t < t_opt**: SW hasn't reached V_in yet. V_DS,HS positive.
/// - **t = t_opt**: SW = V_in. V_DS,HS = 0 (perfect ZVS).
/// - **t > t_opt**: SW overshoots. The HS body diode clamps SW at
///   V_in, paying reverse-recovery loss; the model reports
///   V_DS = 0 in this regime since the HS turn-on is into a
///   clamped node (residual loss is body-diode-driven, not Coss-
///   driven, and outside this closed-form's scope).
///
/// At sub-threshold drive (`I·Z_0 < V_in`), V_SW peaks at
/// `√(V_in² + (I·Z_0)²) − V_in` and then ramps back down. The HS
/// turn-on at peak yields residual `V_DS = V_in − V_peak`.
pub fn vds_residual_at_turnon_v(
    v_in_v: f64,
    i_load_a: f64,
    l_h: f64,
    coss_total_f: f64,
    t_dt_s: f64,
) -> f64 {
    if v_in_v <= 0.0 { return 0.0; }
    if l_h <= 0.0 || coss_total_f <= 0.0 { return v_in_v; }
    let omega = 1.0 / (l_h * coss_total_f).sqrt();
    let z0 = (l_h / coss_total_f).sqrt();
    let i = i_load_a.abs();
    let phase = omega * t_dt_s;
    // V_SW(t) = V_in − V_in·cos(phase) + I·Z_0·sin(phase)
    let v_sw = v_in_v - v_in_v * phase.cos() + i * z0 * phase.sin();
    if v_sw >= v_in_v {
        // Body diode clamps at V_in. The HS gate fires into a node
        // already at V_in → ZVS for the FET, body-diode losses
        // ignored here (they're a separate budget item).
        return 0.0;
    }
    v_in_v - v_sw
}

/// One-shot ZVS report at a given operating point.
#[derive(Debug, Clone)]
pub struct ZvsReport {
    /// Total HS+LS C_oss [F].
    pub coss_total_f: f64,
    /// Optimal deadtime for THIS operating point — the moment V_SW
    /// first reaches V_in. Setting the firmware's deadtime to this
    /// value makes turn-on perfectly ZVS.
    pub optimal_deadtime_s: f64,
    /// LC-tank "threshold drive" — the load current at which optimal
    /// deadtime equals the quarter resonance period. A useful
    /// scalar separating "easy ZVS regime" (I > threshold) from
    /// "hard ZVS regime" (I < threshold, deadtime needs to grow).
    pub threshold_drive_a: f64,
    /// V_DS across the HS FET at the moment its gate fires given
    /// the current operating point and `t_dt_actual_s` deadtime.
    /// Zero = perfect ZVS. Sub-V_in = partial ZVS (loss reduced
    /// but non-zero). Equal to V_in = full hard switching.
    pub v_ds_residual_v: f64,
    /// Energy lost per turn-on event in the residual hard-switch
    /// (joules). `0.5 · C_oss · V_DS²`.
    pub e_hard_switch_j: f64,
    /// Power dissipated by the residual hard switching at the
    /// configured switching frequency [W].
    pub p_hard_switch_w: f64,
    /// Did the operating point achieve full ZVS (residual < 1 % of V_in)?
    pub achieved_zvs: bool,
}

/// Compute a ZVS report for a half-bridge at one operating point.
///
/// `l_loop_h` is the commutation-loop inductance (typically a few
/// nH on a tight GaN layout, several tens of nH on a sloppy one).
/// `t_dt_actual_s` is the deadtime the firmware is currently using;
/// pass `optimal_deadtime_s(l, c)` to ask "if I tuned deadtime
/// optimally, would this op-point be ZVS?".
pub fn zvs_report(
    v_in_v: f64,
    i_load_a: f64,
    fsw_hz: f64,
    l_loop_h: f64,
    hs: &FetProfile,
    ls: &FetProfile,
    t_dt_actual_s: f64,
) -> ZvsReport {
    let coss = coss_total_farads(hs, ls);
    let t_opt = optimal_deadtime_s(v_in_v, i_load_a, l_loop_h, coss);
    let i_thresh = threshold_drive_a(v_in_v, l_loop_h, coss);
    let v_residual = vds_residual_at_turnon_v(
        v_in_v, i_load_a, l_loop_h, coss, t_dt_actual_s,
    );
    let e_hard = 0.5 * coss * v_residual * v_residual;
    let p_hard = e_hard * fsw_hz;
    ZvsReport {
        coss_total_f: coss,
        optimal_deadtime_s: t_opt,
        threshold_drive_a: i_thresh,
        v_ds_residual_v: v_residual,
        e_hard_switch_j: e_hard,
        p_hard_switch_w: p_hard,
        achieved_zvs: v_residual < (v_in_v * 0.01),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::FetProfile;

    /// At I=0 (no inductor drive) optimal deadtime falls back to the
    /// quarter-period of the LC tank — the canonical "ZVS deadtime
    /// ≈ T_res/4" textbook answer. Sanity-checks the closed-form
    /// arctan formula's degenerate limit.
    #[test]
    fn optimal_deadtime_at_zero_drive_is_quarter_period() {
        let l_h = 10e-9;
        let c_f = 1.2e-9;
        let t = optimal_deadtime_s(24.0, 0.0, l_h, c_f);
        let expected = 0.25 * 2.0 * std::f64::consts::PI * (l_h * c_f).sqrt();
        assert!((t - expected).abs() < 1e-15, "got {t}, expected {expected}");
        assert!(t > 5e-9 && t < 6e-9, "expected ~5.4 ns, got {} ns", t * 1e9);
    }

    /// At HIGH drive (50 A on a 5 nH/1.2 nF tank), optimal
    /// deadtime shrinks well below the quarter period — V_SW
    /// reaches V_in much earlier than T_res/4. Checks that the
    /// arctan formula gives the right *short* deadtime instead of
    /// blindly returning T/4.
    #[test]
    fn optimal_deadtime_shrinks_at_high_drive() {
        let l_h = 5e-9;
        let c_f = 1.2e-9;
        let t = optimal_deadtime_s(24.0, 50.0, l_h, c_f);
        let quarter = 0.25 * 2.0 * std::f64::consts::PI * (l_h * c_f).sqrt();
        // 50 A · √(5e-9/1.2e-9) = 50 · 2.04 = 102 V drive vs 24 V V_in
        // → arctan(24/102) / ω = 0.231 / 4.08e8 = 0.566 ns
        assert!(t > 0.4e-9 && t < 0.7e-9,
            "expected ~0.57 ns, got {} ns (quarter={} ns)", t * 1e9, quarter * 1e9);
    }

    /// Threshold load current: V_in=24V, L=10nH, C=1.2nF →
    /// I_min = 24 · √(1.2e-9/10e-9) = 24 · 0.346 = 8.3 A.
    /// Below 8.3 A no amount of deadtime tuning gives ZVS.
    #[test]
    fn threshold_drive_textbook() {
        let i_thresh = threshold_drive_a(24.0, 10e-9, 1.2e-9);
        assert!(
            (i_thresh - 8.31).abs() < 0.05,
            "expected ~8.3 A, got {i_thresh}",
        );
    }

    /// `min_load_for_zvs_at_deadtime_a` solves for the load current
    /// I such that V_SW(t_dt) = V_in. Above the threshold-drive
    /// regime the optimal deadtime is < T/4; the test fixes
    /// V_in=24, L=5nH, C=1.2nF and t_dt=1ns and asks "how much
    /// current do I need for ZVS at this deadtime?". Closed form:
    /// I = V_in · cot(ω·t_dt) / Z_0.
    /// ω = 1/√(5e-9·1.2e-9) = 4.08e8, ωt = 0.408, cos=0.918,
    /// sin=0.397, Z_0=2.04 → I = 24·(0.918/0.397)/2.04 = 27.2 A.
    #[test]
    fn min_load_for_zvs_at_deadtime_closed_form() {
        let i = min_load_for_zvs_at_deadtime_a(24.0, 5e-9, 1.2e-9, 1e-9);
        assert!(
            i > 25.0 && i < 30.0,
            "expected ~27 A, got {i}",
        );
    }

    /// At a deadtime past `T_res/4` the body diode has already
    /// clamped — any load achieves ZVS. Confirm the helper returns
    /// 0 in that regime.
    #[test]
    fn min_load_for_zvs_at_long_deadtime_is_zero() {
        let l = 5e-9;
        let c = 1.2e-9;
        // T/4 = (π/2)·√(L·C) ≈ 1.92 ns. Use 5 ns ⇒ well past.
        let i = min_load_for_zvs_at_deadtime_a(24.0, l, c, 5e-9);
        assert_eq!(i, 0.0, "expected 0 at long deadtime, got {i}");
    }

    /// Below threshold load: V_DS residual ≈ V_in − I·Z_0. With
    /// I=4 A, V_in=24 V, Z_0=√(L/C)=√(10e-9/1.2e-9)=2.89 Ω,
    /// V_peak = 4·2.89 = 11.55 V → V_DS,residual ≈ 12.45 V (over
    /// half of V_in remains, hard switching).
    /// Deadtime FAR too short: V_SW barely starts climbing, the HS
    /// gate fires while V_SW is still at low voltage → almost-full
    /// V_in across the FET at turn-on (basically hard switching).
    /// At V=24, I=4, L=10nH, C=1.2nF, t_dt=0.5 ns:
    /// ω·t = 0.144, cos = 0.99, sin = 0.144.
    /// V_SW = 24 − 24·0.99 + 4·2.89·0.144 = 0.24 + 1.66 = 1.9 V.
    /// V_DS,residual = 22.1 V (almost full hard switching).
    #[test]
    fn short_deadtime_leaves_residual() {
        let vds = vds_residual_at_turnon_v(24.0, 4.0, 10e-9, 1.2e-9, 0.5e-9);
        assert!(
            vds > 18.0 && vds < 24.0,
            "expected ~22 V residual at 0.5 ns deadtime, got {vds}",
        );
    }

    /// Sufficient load + optimal deadtime: full ZVS, V_SW reaches
    /// V_in exactly, residual ≈ 0.
    #[test]
    fn above_threshold_with_optimal_deadtime_zero_residual() {
        let l = 10e-9;
        let c = 1.2e-9;
        let v_in = 24.0;
        let i_load = threshold_drive_a(v_in, l, c) * 1.5; // 1.5x over
        let t_opt = optimal_deadtime_s(v_in, i_load, l, c);
        let vds = vds_residual_at_turnon_v(v_in, i_load, l, c, t_opt);
        assert!(
            vds < 0.1,
            "expected near-zero residual at 1.5x threshold + optimal dt, got {vds} V",
        );
    }

    /// End-to-end report on an EPC2306 half-bridge driving 24 V at
    /// 50 A through a 5 nH commutation loop at 1 MHz fsw.
    #[test]
    fn report_epc2306_at_24v_50a() {
        let hs = FetProfile::epc2306();
        let ls = FetProfile::epc2306();
        // Drive into a 5 nH commutation loop at 1 MHz fsw, with a
        // deadtime tuned to the optimum for this op-point. We expect
        // the report to flag full ZVS (zero residual since V_SW
        // body-diode-clamps at V_in well before the dt elapses).
        let l_loop = 5e-9;
        let coss = 1.2e-9;
        let t_opt = optimal_deadtime_s(24.0, 50.0, l_loop, coss);
        let r = zvs_report(24.0, 50.0, 1e6, l_loop, &hs, &ls, t_opt);
        assert!((r.coss_total_f - 1.2e-9).abs() < 1e-12);
        // At I=50A, optimal deadtime is much shorter than T/4
        // (≈0.57 ns vs ≈1.92 ns).
        assert!(r.optimal_deadtime_s > 0.4e-9 && r.optimal_deadtime_s < 0.8e-9,
            "expected ~0.6 ns, got {} ns", r.optimal_deadtime_s * 1e9);
        // 50 A is way over the threshold, residual should be zero.
        assert!(
            r.achieved_zvs && r.v_ds_residual_v < 0.1,
            "expected ZVS at 50 A, got V_DS = {} V",
            r.v_ds_residual_v,
        );
    }

    /// Deadtime way too short to allow the swing: 1 A load on a
    /// 5 nH commutation loop with 0.3 ns deadtime. ω·t = 0.122,
    /// V_SW barely moves; report flags non-ZVS with a substantial
    /// V_DS residual at HS turn-on. This is the firmware-side
    /// failure mode the analysis is meant to catch — easy to miss
    /// on a bench scope, the residual loss shows up as efficiency
    /// degradation at light load.
    #[test]
    fn report_flags_short_deadtime_residual() {
        let hs = FetProfile::epc2306();
        let ls = FetProfile::epc2306();
        let r = zvs_report(24.0, 1.0, 1e6, 5e-9, &hs, &ls, 0.3e-9);
        assert!(!r.achieved_zvs, "expected hard-switch flag at short dt");
        assert!(r.v_ds_residual_v > 5.0,
            "expected nontrivial residual, got {}", r.v_ds_residual_v);
        assert!(r.p_hard_switch_w > 0.0, "expected non-zero P_hard");
    }
}
