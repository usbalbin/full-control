use core::f64::consts::PI;

use crate::control_2p2z::TwoPoleTwoZeroParams;
use crate::math::{atan, pow2, sqrt, tan};

/// Parameters for PFC boost average current mode compensator design.
#[derive(Debug, Clone, Copy)]
pub struct PfcParameters {
    /// DC bus output voltage [V], e.g. 400.
    pub v_out: f64,
    /// AC input RMS voltage [V], e.g. 230.
    pub v_in_rms: f64,
    /// Line frequency [Hz]: 50 or 60.
    pub f_line: f64,
    /// Switching frequency [Hz].
    pub f_sw: f64,
    /// Boost inductor [H].
    pub l_boost: f64,
    /// Output (bus) capacitor [F].
    pub c_out: f64,
    /// Output cap ESR [Ohm].
    pub r_esr_out: f64,
    /// Rated output power [W].
    pub p_rated: f64,
    /// Current sense resistance [Ohm].
    pub r_sense: f64,
    /// Inner current loop target crossover [Hz].
    pub current_crossover_hz: f64,
    /// Outer voltage loop target crossover [Hz].
    pub voltage_crossover_hz: f64,
    /// Inner current loop phase margin [rad].
    pub phase_margin_current: f64,
    /// Outer voltage loop phase margin [rad].
    pub phase_margin_voltage: f64,
}

/// Continuous-time design summary for both PFC control loops.
/// Used by the Bode plot module to evaluate loop gains at arbitrary frequencies.
#[derive(Debug, Clone, Copy)]
pub struct PfcDesignSummary {
    // ── Inner current loop ────────────────────────────────────
    /// Plant DC gain: V_out / L (A/(s·V) — magnitude of integrator numerator).
    pub inner_plant_gain: f64,
    /// Current sense resistance [Ohm].
    pub r_sense: f64,
    /// Integrator gain [rad/s].
    pub inner_omega_cp0: f64,
    /// Compensator zero [rad/s].
    pub inner_omega_cz1: f64,
    /// Compensator high-frequency pole [rad/s].
    pub inner_omega_cp1: f64,
    /// Design crossover [rad/s].
    pub inner_omega_x: f64,
    /// Achieved phase margin [rad].
    pub inner_phase_margin: f64,

    // ── Outer voltage loop ────────────────────────────────────
    /// Voltage plant gain: V_in_pk / (2 × V_out).
    pub outer_plant_gain: f64,
    /// Output capacitance [F].
    pub c_out: f64,
    /// Output ESR [Ohm].
    pub r_esr_out: f64,
    /// Integrator gain [rad/s].
    pub outer_omega_cp0: f64,
    /// Compensator zero [rad/s].
    pub outer_omega_cz1: f64,
    /// Compensator pole [rad/s] — at 2×f_line for ripple rejection.
    pub outer_omega_cp1: f64,
    /// Design crossover [rad/s].
    pub outer_omega_x: f64,
    /// Achieved phase margin [rad].
    pub outer_phase_margin: f64,

    // ── Common ────────────────────────────────────────────────
    pub f_sw: f64,
    pub f_line: f64,
}

/// Result of PFC compensator design: discrete coefficients for both loops.
#[derive(Debug, Clone, Copy)]
pub struct PfcDesignResult {
    /// Inner current loop 2P2Z coefficients (sampled at f_sw).
    pub inner: TwoPoleTwoZeroParams<f32>,
    /// Outer voltage loop 2P2Z coefficients (sampled at f_sw).
    pub outer: TwoPoleTwoZeroParams<f32>,
    /// Continuous-time design summary for Bode plots.
    pub summary: PfcDesignSummary,
}

/// Design a Type II compensator (integrator + 1 zero + 1 pole).
///
/// H_c(s) = ω_cp0/s × (1 + s/ω_cz1) / (1 + s/ω_cp1)
///
/// Returns (ω_cp0, ω_cz1, ω_cp1) or None if infeasible.
fn design_type2(
    plant_mag_at_crossover: f64,
    omega_x: f64,
    omega_cp1: f64,
    phase_margin: f64,
) -> Option<(f64, f64, f64)> {
    // Compensator phase at crossover:
    //   φ_c = -90° (integrator) + atan(ω_x/ω_cz1) - atan(ω_x/ω_cp1)
    //
    // Plant phase for G_id = V_out/(sL) is -90°.
    // Total loop phase = -90° (plant) + φ_c
    //                  = -180° + atan(ω_x/ω_cz1) - atan(ω_x/ω_cp1)
    // Set to -(180° - φ_m):
    //   atan(ω_x/ω_cz1) = φ_m + atan(ω_x/ω_cp1)

    let phi_target = phase_margin + atan(omega_x / omega_cp1);

    // Guard: if φ_target ≥ π/2, tan flips sign → infeasible.
    if phi_target >= 0.5 * PI {
        return None;
    }

    let omega_cz1 = omega_x / tan(phi_target);

    // Set integrator gain so |T(jω_x)| = 1:
    //   |G_plant(jω_x)| × |H_c(jω_x)| = 1
    //   plant_mag × (ω_cp0/ω_x) × sqrt(1 + (ω_x/ω_cz1)²) / sqrt(1 + (ω_x/ω_cp1)²) = 1
    let k_z = sqrt(1.0 + pow2(omega_x / omega_cz1));
    let k_p = sqrt(1.0 + pow2(omega_x / omega_cp1));
    let omega_cp0 = omega_x / (plant_mag_at_crossover * k_z / k_p);

    Some((omega_cp0, omega_cz1, omega_cp1))
}

/// Bilinear (Tustin) transform of a Type II compensator to discrete 2P2Z coefficients.
///
/// H_c(s) = ω_cp0/s × (1 + s/ω_cz1) / (1 + s/ω_cp1)
///
/// Uses the same formulas as control_2p2z.rs.
fn bilinear_type2(
    omega_cp0: f64,
    omega_cz1: f64,
    omega_cp1: f64,
    t_s: f64,
) -> TwoPoleTwoZeroParams<f32> {
    let b0 = t_s * omega_cp0 * omega_cp1 * (2.0 + t_s * omega_cz1)
        / (2.0 * (2.0 + t_s * omega_cp1) * omega_cz1);

    let b1 = pow2(t_s) * omega_cp0 * omega_cp1 / (2.0 + t_s * omega_cp1);

    let b2 = t_s * omega_cp0 * omega_cp1 * (-2.0 + t_s * omega_cz1)
        / (2.0 * (2.0 + t_s * omega_cp1) * omega_cz1);

    let a1 = 4.0 / (2.0 + t_s * omega_cp1);
    let a2 = (-2.0 + t_s * omega_cp1) / (2.0 + t_s * omega_cp1);

    TwoPoleTwoZeroParams {
        a1: a1 as f32,
        a2: a2 as f32,
        b0: b0 as f32,
        b1: b1 as f32,
        b2: b2 as f32,
    }
}

impl PfcParameters {
    /// Design both PFC control loop compensators.
    ///
    /// Returns None if either loop design is infeasible (phase budget exhausted).
    pub fn design(&self) -> Option<PfcDesignResult> {
        let t_s = 1.0 / self.f_sw;
        let v_in_pk = self.v_in_rms * sqrt(2.0);

        // ── Inner current loop ────────────────────────────────
        //
        // Plant: G_id(s) = V_out / (s × L)
        // Sense: H_sense = R_sense
        // Open-loop plant gain seen by compensator: |G_id × H_sense| at ω_x
        //   = V_out × R_sense / (ω_x × L)

        let omega_x_i = 2.0 * PI * self.current_crossover_hz;
        let inner_plant_gain = self.v_out / self.l_boost; // V_out / L
        let plant_mag_at_fx = inner_plant_gain * self.r_sense / omega_x_i;

        // HF pole at Nyquist: ω_cp1 = π × f_sw
        let inner_omega_cp1 = PI * self.f_sw;

        let (inner_omega_cp0, inner_omega_cz1, _) = design_type2(
            plant_mag_at_fx,
            omega_x_i,
            inner_omega_cp1,
            self.phase_margin_current,
        )?;

        let inner_coeffs = bilinear_type2(inner_omega_cp0, inner_omega_cz1, inner_omega_cp1, t_s);

        // ── Outer voltage loop ────────────────────────────────
        //
        // Plant (with inner loop closed, power balance model):
        //   i_ref(t) = k × |sin(θ)|, so P_in = V_pk × k / 2
        //   Energy on output cap: C_out × V_out × dv/dt = V_pk×k/2 - P_load
        //   Linearized: G_vi(s) = V_pk / (2 × V_out × s × C_out)
        //
        // ESR zero at high freq: × (1 + s×R_esr×C_out), but negligible at 10-20 Hz crossover.

        let omega_x_v = 2.0 * PI * self.voltage_crossover_hz;
        let outer_plant_gain = v_in_pk / (2.0 * self.v_out);
        let outer_plant_mag_at_fx = outer_plant_gain / (omega_x_v * self.c_out);

        // Pole at 2×f_line for 2nd harmonic ripple rejection
        let outer_omega_cp1 = 2.0 * PI * (2.0 * self.f_line);

        let (outer_omega_cp0, outer_omega_cz1, _) = design_type2(
            outer_plant_mag_at_fx,
            omega_x_v,
            outer_omega_cp1,
            self.phase_margin_voltage,
        )?;

        let outer_coeffs = bilinear_type2(outer_omega_cp0, outer_omega_cz1, outer_omega_cp1, t_s);

        let summary = PfcDesignSummary {
            inner_plant_gain,
            r_sense: self.r_sense,
            inner_omega_cp0,
            inner_omega_cz1,
            inner_omega_cp1,
            inner_omega_x: omega_x_i,
            inner_phase_margin: self.phase_margin_current,

            outer_plant_gain,
            c_out: self.c_out,
            r_esr_out: self.r_esr_out,
            outer_omega_cp0,
            outer_omega_cz1,
            outer_omega_cp1,
            outer_omega_x: omega_x_v,
            outer_phase_margin: self.phase_margin_voltage,

            f_sw: self.f_sw,
            f_line: self.f_line,
        };

        Some(PfcDesignResult {
            inner: inner_coeffs,
            outer: outer_coeffs,
            summary,
        })
    }

    /// Maximum feasible inner-loop crossover frequency.
    ///
    /// Binary searches for the highest crossover where the inner-loop
    /// phase budget is still positive (φ_target < π/2).
    pub fn max_inner_crossover_hz(&self) -> f64 {
        let omega_cp1 = PI * self.f_sw;
        let mut lo = 1.0_f64;
        let mut hi = self.f_sw / 2.0;

        for _ in 0..50 {
            let mid = (lo + hi) / 2.0;
            let omega_x = 2.0 * PI * mid;
            let phi_target = self.phase_margin_current + atan(omega_x / omega_cp1);
            if phi_target < 0.5 * PI {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Reference PFC design: 400V bus, 230Vac, 300W, 65kHz, 500µH inductor.
    fn reference_params() -> PfcParameters {
        PfcParameters {
            v_out: 400.0,
            v_in_rms: 230.0,
            f_line: 50.0,
            f_sw: 65_000.0,
            l_boost: 500e-6,
            c_out: 220e-6,
            r_esr_out: 100e-3,
            p_rated: 300.0,
            r_sense: 50e-3,
            current_crossover_hz: 6_500.0, // f_sw / 10
            voltage_crossover_hz: 10.0,
            phase_margin_current: 60.0_f64.to_radians(),
            phase_margin_voltage: 60.0_f64.to_radians(),
        }
    }

    #[test]
    fn design_succeeds_for_reference_params() {
        let result = reference_params().design();
        assert!(result.is_some(), "design should succeed for reference params");
    }

    #[test]
    fn inner_loop_crossover_matches_target() {
        let p = reference_params();
        let result = p.design().unwrap();
        let ds = &result.summary;

        // Evaluate inner loop gain at crossover: |G_id × H_ci × H_sense| should be ~1
        let omega = ds.inner_omega_x;

        // Plant: V_out / (jω × L) → magnitude = V_out / (ω × L)
        let g_plant = ds.inner_plant_gain / omega; // V_out/L / ω
        let h_sense = ds.r_sense;

        // Compensator: ω_cp0/ω × K_z / K_p
        let k_z = (1.0 + pow2(omega / ds.inner_omega_cz1)).sqrt();
        let k_p = (1.0 + pow2(omega / ds.inner_omega_cp1)).sqrt();
        let h_comp = ds.inner_omega_cp0 / omega * k_z / k_p;

        let loop_mag = g_plant * h_comp * h_sense;
        assert!(
            (loop_mag - 1.0).abs() < 0.01,
            "inner loop gain at crossover should be ~1.0, got {loop_mag}"
        );
    }

    #[test]
    fn outer_loop_crossover_matches_target() {
        let p = reference_params();
        let result = p.design().unwrap();
        let ds = &result.summary;

        // Evaluate outer loop gain at crossover
        let omega = ds.outer_omega_x;

        // Plant: outer_plant_gain / (ω × C_out)
        let g_plant = ds.outer_plant_gain / (omega * ds.c_out);

        // Compensator magnitude
        let k_z = (1.0 + pow2(omega / ds.outer_omega_cz1)).sqrt();
        let k_p = (1.0 + pow2(omega / ds.outer_omega_cp1)).sqrt();
        let h_comp = ds.outer_omega_cp0 / omega * k_z / k_p;

        let loop_mag = g_plant * h_comp;
        assert!(
            (loop_mag - 1.0).abs() < 0.05,
            "outer loop gain at crossover should be ~1.0, got {loop_mag}"
        );
    }

    #[test]
    fn inner_coefficients_are_finite() {
        let result = reference_params().design().unwrap();
        let c = result.inner;
        assert!(c.b0.is_finite() && c.b0 > 0.0, "b0 = {}", c.b0);
        assert!(c.b1.is_finite(), "b1 = {}", c.b1);
        assert!(c.b2.is_finite(), "b2 = {}", c.b2);
        assert!(c.a1.is_finite() && c.a1 > 0.0, "a1 = {}", c.a1);
        assert!(c.a2.is_finite(), "a2 = {}", c.a2);
    }

    #[test]
    fn outer_coefficients_are_finite() {
        let result = reference_params().design().unwrap();
        let c = result.outer;
        assert!(c.b0.is_finite() && c.b0 > 0.0, "b0 = {}", c.b0);
        assert!(c.b1.is_finite(), "b1 = {}", c.b1);
        assert!(c.b2.is_finite(), "b2 = {}", c.b2);
        assert!(c.a1.is_finite() && c.a1 > 0.0, "a1 = {}", c.a1);
        assert!(c.a2.is_finite(), "a2 = {}", c.a2);
    }

    #[test]
    fn max_inner_crossover_is_reasonable() {
        let p = reference_params();
        let max_fx = p.max_inner_crossover_hz();
        // Should be between 1 kHz and f_sw/2
        assert!(max_fx > 1_000.0, "max crossover too low: {max_fx}");
        assert!(max_fx < p.f_sw / 2.0, "max crossover too high: {max_fx}");

        // Design at max crossover should succeed
        let p2 = PfcParameters {
            current_crossover_hz: max_fx * 0.99,
            ..p
        };
        assert!(p2.design().is_some(), "design at max crossover should succeed");
    }

    #[test]
    fn infeasible_crossover_returns_none() {
        let p = PfcParameters {
            current_crossover_hz: 100_000.0, // way above f_sw/2
            ..reference_params()
        };
        assert!(p.design().is_none(), "should be infeasible");
    }

    #[test]
    fn design_with_different_line_frequencies() {
        for f_line in [50.0, 60.0] {
            let p = PfcParameters { f_line, ..reference_params() };
            let result = p.design();
            assert!(result.is_some(), "design should succeed for f_line={f_line}");
            let ds = result.unwrap().summary;
            // Outer pole should be at 2×f_line
            let expected_pole = 2.0 * PI * 2.0 * f_line;
            assert!(
                (ds.outer_omega_cp1 - expected_pole).abs() < 1.0,
                "outer pole at {}, expected {expected_pole}",
                ds.outer_omega_cp1
            );
        }
    }
}
