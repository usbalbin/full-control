//! SRF-PLL (Synchronous Reference Frame Phase-Locked Loop) for 3-phase grid
//! synchronization.
//!
//! Tracks the grid voltage angle without requiring sin/cos computation by using
//! a rotating phasor with small-angle approximation. This makes it suitable for
//! fixed-point firmware.
//!
//! The PLL takes Clarke-domain (αβ) voltages and outputs sin(θ)/cos(θ) of the
//! estimated grid angle, plus the estimated angular frequency.
//!
//! # Convention
//!
//! With sin-based phase voltages (v_a = V·sin(ωt)), the PLL locks to
//! θ_est = ωt − π/2 so that v_d = V_pk and v_q = 0 at lock.

use crate::control_2p2z::Scalar;

/// Output of a PLL update step.
#[derive(Debug, Clone, Copy)]
pub struct PllOutput<S> {
    /// Sine of the estimated grid angle.
    pub sin_theta: S,
    /// Cosine of the estimated grid angle.
    pub cos_theta: S,
    /// Estimated angular frequency \[rad/s\].
    pub omega: S,
}

/// SRF-PLL for 3-phase grid angle tracking.
///
/// # How it works
///
/// 1. Park-transform the αβ voltages using the current angle estimate
/// 2. The q-axis voltage is the phase error (zero when locked)
/// 3. A PI controller adjusts the estimated frequency to drive v_q → 0
/// 4. The angle advances via a rotating phasor (small-angle approx),
///    avoiding sin/cos computation entirely
///
/// # Tuning
///
/// For a 3-phase system with line-to-neutral peak voltage V_pk:
/// - `kp = 2 × ζ × ω_n / V_pk`
/// - `ki = ω_n² / V_pk`
///
/// where ω_n is the PLL natural frequency (typically 2π×20‥50 rad/s)
/// and ζ is the damping ratio (0.707 for critically damped).
pub struct SrfPll<S: Scalar> {
    sin_theta: S,
    cos_theta: S,
    omega: S,
    omega_nominal: S,
    kp: S,
    ki: S,
    integrator: S,
    omega_max_dev: S,
}

impl<S: Scalar> SrfPll<S> {
    /// Create a new PLL.
    ///
    /// - `f_nominal_hz`: expected grid frequency \[Hz\]
    /// - `kp`, `ki`: PI controller gains (see struct-level docs for tuning)
    /// - `max_freq_dev_hz`: maximum allowed frequency deviation \[Hz\]
    pub fn new(f_nominal_hz: S, kp: S, ki: S, max_freq_dev_hz: S) -> Self {
        let two_pi = S::from_f32(core::f32::consts::TAU);
        Self {
            sin_theta: S::ZERO,
            cos_theta: S::from_f32(1.0),
            omega: f_nominal_hz * two_pi,
            omega_nominal: f_nominal_hz * two_pi,
            kp,
            ki,
            integrator: S::ZERO,
            omega_max_dev: max_freq_dev_hz * two_pi,
        }
    }

    /// Update the PLL with new αβ voltage measurements.
    ///
    /// `v_alpha`, `v_beta`: Clarke-domain voltages.
    /// `dt`: time step since last call \[s\].
    #[inline(always)]
    pub fn update(&mut self, v_alpha: S, v_beta: S, dt: S) -> PllOutput<S> {
        // Park transform q-axis: phase error signal
        let v_q = v_beta * self.cos_theta - v_alpha * self.sin_theta;

        // PI controller
        self.integrator = self.integrator + self.ki * v_q * dt;
        self.integrator = self.integrator.clamp(
            S::ZERO - self.omega_max_dev,
            self.omega_max_dev,
        );

        let omega_correction = self.kp * v_q + self.integrator;
        self.omega = self.omega_nominal + omega_correction;

        // Advance angle: rotating phasor with small-angle approximation
        // dθ = ω·dt ≪ 1 at switching frequencies
        let d_theta = self.omega * dt;
        let half = S::from_f32(0.5);
        let one = S::from_f32(1.0);
        let cos_d = one - half * d_theta * d_theta;
        let sin_d = d_theta;

        let new_cos = self.cos_theta * cos_d - self.sin_theta * sin_d;
        let new_sin = self.sin_theta * cos_d + self.cos_theta * sin_d;

        // Normalize: 1st-order Taylor of 1/√x around x = 1 → (3 − x)/2
        let mag_sq = new_cos * new_cos + new_sin * new_sin;
        let scale = (S::from_f32(3.0) - mag_sq) * half;

        self.cos_theta = new_cos * scale;
        self.sin_theta = new_sin * scale;

        PllOutput {
            sin_theta: self.sin_theta,
            cos_theta: self.cos_theta,
            omega: self.omega,
        }
    }

    /// Current sin(θ) estimate.
    pub fn sin_theta(&self) -> S { self.sin_theta }

    /// Current cos(θ) estimate.
    pub fn cos_theta(&self) -> S { self.cos_theta }

    /// Current estimated angular frequency \[rad/s\].
    pub fn omega(&self) -> S { self.omega }

    /// Current estimated frequency \[Hz\].
    pub fn frequency_hz(&self) -> S {
        self.omega * S::from_f32(1.0 / core::f32::consts::TAU)
    }

    /// Reset to initial state.
    pub fn reset(&mut self) {
        self.sin_theta = S::ZERO;
        self.cos_theta = S::from_f32(1.0);
        self.omega = self.omega_nominal;
        self.integrator = S::ZERO;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f32 = 1e-2;
    const TAU: f32 = core::f32::consts::TAU;

    /// Helper: balanced 3-phase αβ voltages (sin-based phase A).
    /// Returns (v_alpha, v_beta) = (V_pk·sin(ωt), −V_pk·cos(ωt)).
    fn grid_alpha_beta(v_pk: f32, f_hz: f32, t: f32) -> (f32, f32) {
        let theta = TAU * f_hz * t;
        (v_pk * theta.sin(), -v_pk * theta.cos())
    }

    /// Build a PLL tuned for a given V_pk and grid frequency.
    fn make_pll(v_pk: f32, f_hz: f32) -> SrfPll<f32> {
        // omega_n = 2π×30, zeta = 0.707
        let omega_n = TAU * 30.0;
        let zeta = 0.707_f32;
        let kp = 2.0 * zeta * omega_n / v_pk;
        let ki = omega_n * omega_n / v_pk;
        SrfPll::new(f_hz, kp, ki, 5.0)
    }

    #[test]
    fn locks_to_50hz() {
        let v_pk = 325.0_f32;
        let f_grid = 50.0;
        let f_sw = 20_000.0;
        let dt = 1.0 / f_sw;
        let mut pll = make_pll(v_pk, f_grid);

        let n = (0.2 * f_sw) as usize; // 200ms
        let mut out = PllOutput { sin_theta: 0.0, cos_theta: 1.0, omega: 0.0 };

        for i in 0..n {
            let t = i as f32 * dt;
            let (va, vb) = grid_alpha_beta(v_pk, f_grid, t);
            out = pll.update(va, vb, dt);
        }

        let f_est = out.omega / TAU;
        assert!((f_est - 50.0).abs() < 0.5, "f_est = {f_est}");
    }

    #[test]
    fn phasor_stays_normalized() {
        let v_pk = 325.0_f32;
        let f_sw = 20_000.0;
        let dt = 1.0 / f_sw;
        let mut pll = make_pll(v_pk, 50.0);

        // Run 1 second (20k steps)
        for i in 0..20_000 {
            let t = i as f32 * dt;
            let (va, vb) = grid_alpha_beta(v_pk, 50.0, t);
            let out = pll.update(va, vb, dt);
            let mag = out.sin_theta * out.sin_theta + out.cos_theta * out.cos_theta;
            assert!(
                (mag - 1.0).abs() < 1e-4,
                "mag = {mag} at step {i}",
            );
        }
    }

    #[test]
    fn vd_converges_to_vpk() {
        let v_pk = 325.0_f32;
        let f_sw = 20_000.0;
        let dt = 1.0 / f_sw;
        let mut pll = make_pll(v_pk, 50.0);

        // Settle for 200ms
        for i in 0..4000 {
            let t = i as f32 * dt;
            let (va, vb) = grid_alpha_beta(v_pk, 50.0, t);
            pll.update(va, vb, dt);
        }

        // Check v_d ≈ V_pk and v_q ≈ 0 for the next few steps
        for i in 4000..4100 {
            let t = i as f32 * dt;
            let (va, vb) = grid_alpha_beta(v_pk, 50.0, t);
            let out = pll.update(va, vb, dt);

            // Full Park transform
            let v_d = va * out.cos_theta + vb * out.sin_theta;
            let v_q = vb * out.cos_theta - va * out.sin_theta;

            assert!(
                (v_d - v_pk).abs() < v_pk * 0.02,
                "v_d = {v_d}, expected {v_pk}",
            );
            assert!(v_q.abs() < v_pk * 0.02, "v_q = {v_q}");
        }
    }

    #[test]
    fn tracks_frequency_step() {
        let v_pk = 325.0_f32;
        let f_sw = 20_000.0;
        let dt = 1.0 / f_sw;
        let mut pll = make_pll(v_pk, 50.0);

        // Lock to 50Hz for 200ms
        for i in 0..4000 {
            let t = i as f32 * dt;
            let (va, vb) = grid_alpha_beta(v_pk, 50.0, t);
            pll.update(va, vb, dt);
        }

        // Step to 51Hz, run another 200ms
        let t_offset = 4000.0 * dt;
        let mut out = PllOutput { sin_theta: 0.0, cos_theta: 1.0, omega: 0.0 };
        for i in 0..4000 {
            let t = t_offset + i as f32 * dt;
            let (va, vb) = grid_alpha_beta(v_pk, 51.0, t);
            out = pll.update(va, vb, dt);
        }

        let f_est = out.omega / TAU;
        assert!((f_est - 51.0).abs() < 0.5, "f_est after step = {f_est}");
    }

    #[test]
    fn reset_returns_to_initial() {
        let v_pk = 325.0_f32;
        let f_sw = 20_000.0;
        let dt = 1.0 / f_sw;
        let mut pll = make_pll(v_pk, 50.0);

        // Run a bit
        for i in 0..1000 {
            let t = i as f32 * dt;
            let (va, vb) = grid_alpha_beta(v_pk, 50.0, t);
            pll.update(va, vb, dt);
        }

        pll.reset();
        assert_eq!(pll.sin_theta(), 0.0);
        assert_eq!(pll.cos_theta(), 1.0);
        assert!((pll.frequency_hz() - 50.0).abs() < TOL);
    }
}
