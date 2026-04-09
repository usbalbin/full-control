//! d-q synchronous frame current controller for 3-phase PFC / VSR.
//!
//! Controls the d-axis current to regulate DC bus voltage (via the outer
//! voltage loop's amplitude command `k`) and the q-axis current to zero for
//! unity power factor.
//!
//! The controller pipeline per switching cycle:
//! 1. Clarke (ABC → αβ) on voltages and currents
//! 2. PLL update (αβ voltages → grid angle)
//! 3. Park (αβ → dq) on currents (and voltages for feedforward)
//! 4. PI on d-axis error, PI on q-axis error
//! 5. Optional feedforward + cross-coupling decoupling
//! 6. Inverse Park (dq → αβ) + inverse Clarke (αβ → ABC) → per-phase duties

use crate::control_2p2z::Scalar;
use crate::pll::{PllOutput, SrfPll};
use crate::transforms::{clarke, inv_clarke, inv_park, park};

/// Simple PI controller with anti-windup clamping.
pub struct PiController<S: Scalar> {
    kp: S,
    ki: S,
    integrator: S,
    out_min: S,
    out_max: S,
}

impl<S: Scalar> PiController<S> {
    pub fn new(kp: S, ki: S, out_min: S, out_max: S) -> Self {
        Self {
            kp,
            ki,
            integrator: S::ZERO,
            out_min,
            out_max,
        }
    }

    #[inline(always)]
    pub fn update(&mut self, error: S, dt: S) -> S {
        self.integrator = self.integrator + self.ki * error * dt;
        self.integrator = self.integrator.clamp(self.out_min, self.out_max);
        let output = self.kp * error + self.integrator;
        output.clamp(self.out_min, self.out_max)
    }

    pub fn reset(&mut self) {
        self.integrator = S::ZERO;
    }
}

/// Output from the d-q current controller.
#[derive(Debug, Clone, Copy)]
pub struct DqOutput<S> {
    /// Per-phase duty cycles (a, b, c), range \[0, 1\].
    /// Convention: VSR/inverter — duty=1 connects phase to V_dc+.
    pub duty_a: S,
    pub duty_b: S,
    pub duty_c: S,
    /// Measured d-axis current \[A\].
    pub i_d: S,
    /// Measured q-axis current \[A\].
    pub i_q: S,
    /// PLL output (angle, frequency).
    pub pll: PllOutput<S>,
}

/// d-q synchronous frame current controller for 3-phase systems.
///
/// # Duty cycle convention
///
/// Outputs duties for a 2-level VSR (voltage source rectifier):
/// `duty_k = 0.5 + v_cmd_k / V_dc`, clamped to \[0, 1\].
///
/// For boost-type PFC topologies, the caller should invert: `duty_boost = 1 - duty_vsr`.
pub struct DqCurrentController<S: Scalar> {
    pll: SrfPll<S>,
    pi_d: PiController<S>,
    pi_q: PiController<S>,
    feedforward: bool,
    decoupling_l: S,
}

impl<S: Scalar> DqCurrentController<S> {
    /// Create a new d-q controller.
    ///
    /// - `pll`: configured PLL for grid angle tracking
    /// - `kp`, `ki`: PI gains for both d and q axes
    /// - `v_dc_max`: maximum expected DC bus voltage (for PI clamping)
    pub fn new(pll: SrfPll<S>, kp: S, ki: S, v_dc_max: S) -> Self {
        Self {
            pll,
            pi_d: PiController::new(kp, ki, -v_dc_max, v_dc_max),
            pi_q: PiController::new(kp, ki, -v_dc_max, v_dc_max),
            feedforward: true,
            decoupling_l: S::ZERO,
        }
    }

    /// Enable or disable grid voltage feedforward (enabled by default).
    pub fn with_feedforward(mut self, enabled: bool) -> Self {
        self.feedforward = enabled;
        self
    }

    /// Enable cross-coupling decoupling with the given filter inductance \[H\].
    pub fn with_decoupling(mut self, l_filter: S) -> Self {
        self.decoupling_l = l_filter;
        self
    }

    /// Update with measured 3-phase voltages and currents.
    ///
    /// - `v_a, v_b, v_c`: grid phase voltages (signed) \[V\]
    /// - `i_a, i_b, i_c`: phase currents (signed) \[A\]
    /// - `i_d_ref`: d-axis current reference \[A\] (from outer voltage loop)
    /// - `i_q_ref`: q-axis current reference \[A\] (0 for unity PF)
    /// - `v_dc`: DC bus voltage \[V\]
    /// - `dt`: time step \[s\]
    #[inline(always)]
    pub fn update(
        &mut self,
        v_a: S,
        v_b: S,
        v_c: S,
        i_a: S,
        i_b: S,
        i_c: S,
        i_d_ref: S,
        i_q_ref: S,
        v_dc: S,
        dt: S,
    ) -> DqOutput<S> {
        // Clarke: ABC → αβ
        let (v_alpha, v_beta) = clarke(v_a, v_b, v_c);
        let (i_alpha, i_beta) = clarke(i_a, i_b, i_c);

        // PLL: track grid angle
        let pll_out = self.pll.update(v_alpha, v_beta, dt);
        let (sin_t, cos_t) = (pll_out.sin_theta, pll_out.cos_theta);

        // Park: αβ → dq
        let (i_d, i_q) = park(i_alpha, i_beta, sin_t, cos_t);

        // PI controllers (error = reference − measured)
        let u_d = self.pi_d.update(i_d_ref - i_d, dt);
        let u_q = self.pi_q.update(i_q_ref - i_q, dt);

        // Converter voltage command in dq frame
        // v_conv = v_grid − u  (u drives the current error to zero)
        let mut v_d_cmd = -u_d;
        let mut v_q_cmd = -u_q;

        if self.feedforward {
            let (v_d_grid, v_q_grid) = park(v_alpha, v_beta, sin_t, cos_t);
            v_d_cmd = v_d_cmd + v_d_grid;
            v_q_cmd = v_q_cmd + v_q_grid;
        }

        // Cross-coupling decoupling
        let l = self.decoupling_l;
        if !(l == S::ZERO) {
            let omega = pll_out.omega;
            v_d_cmd = v_d_cmd + omega * l * i_q;
            v_q_cmd = v_q_cmd - omega * l * i_d;
        }

        // Inverse Park: dq → αβ
        let (v_alpha_cmd, v_beta_cmd) = inv_park(v_d_cmd, v_q_cmd, sin_t, cos_t);

        // Inverse Clarke: αβ → ABC modulation voltages
        let (v_a_cmd, v_b_cmd, v_c_cmd) = inv_clarke(v_alpha_cmd, v_beta_cmd);

        // Duty cycles: duty = 0.5 + v_cmd / V_dc
        let half = S::from_f32(0.5);
        let one = S::from_f32(1.0);
        let inv_vdc = one / v_dc;
        let duty_a = (half + v_a_cmd * inv_vdc).clamp(S::ZERO, one);
        let duty_b = (half + v_b_cmd * inv_vdc).clamp(S::ZERO, one);
        let duty_c = (half + v_c_cmd * inv_vdc).clamp(S::ZERO, one);

        DqOutput {
            duty_a,
            duty_b,
            duty_c,
            i_d,
            i_q,
            pll: pll_out,
        }
    }

    /// Access the internal PLL.
    pub fn pll(&self) -> &SrfPll<S> {
        &self.pll
    }

    /// Reset all state (PLL + PI integrators).
    pub fn reset(&mut self) {
        self.pll.reset();
        self.pi_d.reset();
        self.pi_q.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TAU: f32 = core::f32::consts::TAU;

    /// Balanced 3-phase voltages (sin-based: v_a = V_pk·sin(ωt)).
    fn grid_v(v_pk: f32, f_hz: f32, t: f32) -> (f32, f32, f32) {
        let w = TAU * f_hz;
        let offset = TAU / 3.0;
        (
            v_pk * (w * t).sin(),
            v_pk * (w * t - offset).sin(),
            v_pk * (w * t - 2.0 * offset).sin(),
        )
    }

    fn make_controller(v_pk: f32, f_hz: f32, v_dc: f32) -> DqCurrentController<f32> {
        // PLL: omega_n = 2π×30, zeta = 0.707
        let omega_n = TAU * 30.0;
        let zeta = 0.707_f32;
        let kp_pll = 2.0 * zeta * omega_n / v_pk;
        let ki_pll = omega_n * omega_n / v_pk;
        let pll = SrfPll::new(f_hz, kp_pll, ki_pll, 5.0);

        // Current PI: ω_bw ≈ 2π×500 (current loop bandwidth)
        // kp = L × ω_bw, ki = R × ω_bw (assuming L=1mH, R=0.1Ω as typical)
        let l = 1e-3_f32;
        let omega_bw = TAU * 500.0;
        let kp = l * omega_bw;
        let ki = 0.1 * omega_bw;

        DqCurrentController::new(pll, kp, ki, v_dc)
    }

    /// Simulate an RL load driven by the controller for `duration` seconds.
    /// Returns (final i_d, final i_q, final duties).
    fn simulate_rl(
        ctrl: &mut DqCurrentController<f32>,
        v_pk: f32,
        f_hz: f32,
        v_dc: f32,
        i_d_ref: f32,
        duration: f32,
        f_sw: f32,
    ) -> DqOutput<f32> {
        let dt = 1.0 / f_sw;
        let l = 1e-3_f32; // 1 mH
        let r = 0.1_f32; // 0.1 Ω

        let mut i_a = 0.0_f32;
        let mut i_b = 0.0_f32;
        let mut i_c = 0.0_f32;

        let n = (duration * f_sw) as usize;
        let mut out = DqOutput {
            duty_a: 0.5,
            duty_b: 0.5,
            duty_c: 0.5,
            i_d: 0.0,
            i_q: 0.0,
            pll: PllOutput {
                sin_theta: 0.0,
                cos_theta: 1.0,
                omega: TAU * f_hz,
            },
        };

        for i in 0..n {
            let t = i as f32 * dt;
            let (v_a, v_b, v_c) = grid_v(v_pk, f_hz, t);

            out = ctrl.update(v_a, v_b, v_c, i_a, i_b, i_c, i_d_ref, 0.0, v_dc, dt);

            // Converter voltages from duties
            let v_conv_a = (out.duty_a - 0.5) * 2.0 * v_dc;
            let v_conv_b = (out.duty_b - 0.5) * 2.0 * v_dc;
            let v_conv_c = (out.duty_c - 0.5) * 2.0 * v_dc;

            // RL model: di/dt = (v_grid - v_conv - R*i) / L
            i_a += (v_a - v_conv_a - r * i_a) / l * dt;
            i_b += (v_b - v_conv_b - r * i_b) / l * dt;
            i_c += (v_c - v_conv_c - r * i_c) / l * dt;
        }

        out
    }

    #[test]
    fn dq_controller_tracks_id_reference() {
        let v_pk = 325.0_f32; // 230V RMS
        let f_hz = 50.0;
        let v_dc = 700.0;
        let i_d_ref = 10.0; // 10A d-axis reference

        let mut ctrl = make_controller(v_pk, f_hz, v_dc);
        let out = simulate_rl(&mut ctrl, v_pk, f_hz, v_dc, i_d_ref, 0.3, 20_000.0);

        // After 300ms, i_d should track the reference within 10%
        assert!(
            (out.i_d - i_d_ref).abs() < i_d_ref * 0.10,
            "i_d = {}, ref = {i_d_ref}",
            out.i_d
        );
        // i_q should be near zero
        assert!(out.i_q.abs() < i_d_ref * 0.10, "i_q = {}", out.i_q);
    }

    #[test]
    fn dq_controller_zero_reference_gives_zero_current() {
        let v_pk = 325.0_f32;
        let f_hz = 50.0;
        let v_dc = 700.0;

        let mut ctrl = make_controller(v_pk, f_hz, v_dc);
        let out = simulate_rl(&mut ctrl, v_pk, f_hz, v_dc, 0.0, 0.3, 20_000.0);

        // Both d and q currents should be ~0
        assert!(out.i_d.abs() < 0.5, "i_d = {}", out.i_d);
        assert!(out.i_q.abs() < 0.5, "i_q = {}", out.i_q);
    }

    #[test]
    fn dq_controller_duties_in_range() {
        let v_pk = 325.0_f32;
        let f_hz = 50.0;
        let v_dc = 700.0;

        let mut ctrl = make_controller(v_pk, f_hz, v_dc);
        let dt = 1.0 / 20_000.0;

        for i in 0..6000 {
            let t = i as f32 * dt;
            let (v_a, v_b, v_c) = grid_v(v_pk, f_hz, t);
            let out = ctrl.update(v_a, v_b, v_c, 0.0, 0.0, 0.0, 5.0, 0.0, v_dc, dt);

            assert!(out.duty_a >= 0.0 && out.duty_a <= 1.0, "duty_a = {}", out.duty_a);
            assert!(out.duty_b >= 0.0 && out.duty_b <= 1.0, "duty_b = {}", out.duty_b);
            assert!(out.duty_c >= 0.0 && out.duty_c <= 1.0, "duty_c = {}", out.duty_c);
        }
    }

    #[test]
    fn dq_controller_reset() {
        let v_pk = 325.0_f32;
        let f_hz = 50.0;
        let v_dc = 700.0;

        let mut ctrl = make_controller(v_pk, f_hz, v_dc);

        // Run a bit
        let dt = 1.0 / 20_000.0;
        for i in 0..1000 {
            let t = i as f32 * dt;
            let (v_a, v_b, v_c) = grid_v(v_pk, f_hz, t);
            ctrl.update(v_a, v_b, v_c, 0.0, 0.0, 0.0, 5.0, 0.0, v_dc, dt);
        }

        ctrl.reset();
        assert_eq!(ctrl.pll().sin_theta(), 0.0);
        assert_eq!(ctrl.pll().cos_theta(), 1.0);
    }

    #[test]
    fn dq_with_decoupling_settles() {
        let v_pk = 325.0_f32;
        let f_hz = 50.0;
        let v_dc = 700.0;
        let i_d_ref = 10.0;

        let mut ctrl = make_controller(v_pk, f_hz, v_dc).with_decoupling(1e-3);
        let out = simulate_rl(&mut ctrl, v_pk, f_hz, v_dc, i_d_ref, 0.3, 20_000.0);

        assert!(
            (out.i_d - i_d_ref).abs() < i_d_ref * 0.10,
            "i_d = {}, ref = {i_d_ref}",
            out.i_d
        );
        assert!(out.i_q.abs() < i_d_ref * 0.10, "i_q = {}", out.i_q);
    }
}
