//! Magnetic core loss via the improved Generalized Steinmetz Equation (iGSE).
//!
//! The classic Steinmetz law `Pv = k·fᵅ·B̂ᵝ` (W/m³) is only valid for a
//! *sinusoidal* flux. Switching-converter inductor flux is **triangular** (and,
//! for PFC, a triangular switching ripple riding a line-frequency envelope), so
//! we use iGSE, which integrates the instantaneous rate of change of flux:
//!
//! ```text
//! Pv = (1/T) ∫₀ᵀ ki · |dB/dt|ᵅ · (ΔB)^(β−α) dt          [W/m³]
//!
//! ki = k / [ (2π)^(α−1) · 2^(β−α) · ∫₀^(2π) |cosθ|ᵅ dθ ]
//! ```
//!
//! `ki` is chosen so that iGSE reproduces the sinusoidal Steinmetz result for a
//! pure sine of peak-to-peak swing `ΔB = 2·B̂`. It depends only on the material
//! coefficients (not the operating point), so it is computed once.
//!
//! For a **piecewise-linear (triangular)** B(t) — an on-segment of length `D·T`
//! and an off-segment of length `(1−D)·T`, both spanning the same flux swing
//! `ΔB` — the iGSE integral is closed form:
//!
//! ```text
//! Pv = ki · (ΔB)ᵝ · f_swᵅ · [ D^(1−α) + (1−D)^(1−α) ]     [W/m³]
//! ```
//!
//! with the flux swing taken from the switching ripple the simulator already
//! computes:
//!
//! ```text
//! ΔB = L(I)·ΔI_pp / (N·Ae)      (ΔI_pp = i_max − i_min, per phase)
//! ```
//!
//! `k`, `α`, `β` are the SI Steinmetz coefficients (f in Hz, B in tesla, Pv in
//! W/m³) obtained by fitting a material's datasheet loss curves over the target
//! frequency/flux/temperature range. This module does not ship material data;
//! callers supply it from the part they actually use.

use std::f64::consts::PI;

/// SI Steinmetz coefficients: `Pv = k·fᵅ·B̂ᵝ` (W/m³, f in Hz, B̂ in tesla).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteinmetzParams {
    pub k: f64,
    pub alpha: f64,
    pub beta: f64,
}

/// A magnetic component's core geometry + loss coefficients.
///
/// Loss and geometry only; the current-dependent inductance `L(I)` stays in
/// [`crate::InductorModel`]. `None` magnetics ⇒ no core loss (default).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagneticsProfile {
    pub steinmetz: SteinmetzParams,
    /// Turns.
    pub n_turns: f64,
    /// Effective core cross-section area Aₑ [m²].
    pub a_e_m2: f64,
    /// Effective core volume Vₑ [m³].
    pub v_e_m3: f64,
    /// Effective magnetic path length lₑ [m] (used for H / saturation checks).
    pub l_e_m: f64,
    /// Saturation flux density B_sat [T] — for the `saturated()` flag.
    pub b_sat_t: f64,
}

/// Numerically integrate `∫₀^(2π) |cosθ|ᵅ dθ = 4·∫₀^(π/2) cosᵅθ dθ`.
///
/// The `2^(α+1)·(0.2761 + 1.7061/(α+1.354))` closed-form approximation from the
/// literature is only accurate near α≈1, so we integrate directly. The
/// integrand is smooth, so a few hundred trapezoidal points are exact to many
/// digits, and this is evaluated once per material via [`MagneticsProfile::ki`].
fn abs_cos_pow_integral(alpha: f64) -> f64 {
    const N: usize = 512;
    let h = (PI / 2.0) / N as f64;
    let mut sum = 0.0;
    for i in 0..=N {
        let theta = i as f64 * h;
        // Trapezoidal weights: half at the two endpoints.
        let w = if i == 0 || i == N { 0.5 } else { 1.0 };
        // clamp guards the tiny negative cos at θ=π/2 from powf() NaN.
        sum += w * theta.cos().max(0.0).powf(alpha);
    }
    4.0 * sum * h
}

impl MagneticsProfile {
    /// The iGSE constant `ki` for these material coefficients.
    ///
    /// Computed from `k`, `α`, `β`; independent of the operating point. For a
    /// hot loop (e.g. per-switching-cycle over a PFC line period) compute it
    /// once and reuse via [`Self::core_loss_pv_with_ki`].
    pub fn ki(&self) -> f64 {
        let SteinmetzParams { k, alpha, beta } = self.steinmetz;
        k / ((2.0 * PI).powf(alpha - 1.0) * 2.0f64.powf(beta - alpha) * abs_cos_pow_integral(alpha))
    }

    /// Peak-to-peak flux swing `ΔB` [T] for a peak-to-peak current ripple.
    pub fn b_swing(&self, l_h: f64, di_pp: f64) -> f64 {
        if self.n_turns <= 0.0 || self.a_e_m2 <= 0.0 {
            return 0.0;
        }
        l_h * di_pp / (self.n_turns * self.a_e_m2)
    }

    /// DC flux density `B_dc` [T] from the average current (for saturation).
    pub fn b_dc(&self, l_h: f64, i_avg: f64) -> f64 {
        if self.n_turns <= 0.0 || self.a_e_m2 <= 0.0 {
            return 0.0;
        }
        l_h * i_avg / (self.n_turns * self.a_e_m2)
    }

    /// `true` if the peak instantaneous flux (`B_dc + ΔB/2`) exceeds `B_sat`.
    pub fn saturated(&self, l_h: f64, i_avg: f64, di_pp: f64) -> bool {
        self.b_sat_t > 0.0
            && (self.b_dc(l_h, i_avg).abs() + 0.5 * self.b_swing(l_h, di_pp)) > self.b_sat_t
    }

    /// Core-loss power density `Pv` [W/m³] for a triangular flux ripple, given a
    /// precomputed `ki` (from [`Self::ki`]).
    pub fn core_loss_pv_with_ki(&self, ki: f64, l_h: f64, duty: f64, di_pp: f64, f_sw: f64) -> f64 {
        if di_pp <= 0.0 || f_sw <= 0.0 {
            return 0.0;
        }
        let alpha = self.steinmetz.alpha;
        let beta = self.steinmetz.beta;
        let delta_b = self.b_swing(l_h, di_pp);
        if delta_b <= 0.0 {
            return 0.0;
        }
        // iGSE diverges for an instantaneous flux step (D→0 or 1); a real
        // converter never runs there. Clamp to a physical duty band.
        let d = duty.clamp(0.02, 0.98);
        let shape = d.powf(1.0 - alpha) + (1.0 - d).powf(1.0 - alpha);
        ki * delta_b.powf(beta) * f_sw.powf(alpha) * shape
    }

    /// Core loss [W] for one triangular-ripple operating point.
    ///
    /// `l_h` = inductance at the operating current, `duty` = switch duty (0–1),
    /// `di_pp` = peak-to-peak ripple current [A], `f_sw` = switching freq [Hz].
    pub fn core_loss_triangular(&self, l_h: f64, duty: f64, di_pp: f64, f_sw: f64) -> f64 {
        self.core_loss_pv_with_ki(self.ki(), l_h, duty, di_pp, f_sw) * self.v_e_m3
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_material() -> SteinmetzParams {
        // Representative ferrite-ish coefficients (SI). Not a specific datasheet
        // fit — used only to exercise the math.
        SteinmetzParams { k: 3.0, alpha: 1.5, beta: 2.5 }
    }

    /// Reference iGSE evaluated by brute-force numerical integration over an
    /// arbitrary sampled B(t). Used to validate the analytic paths.
    fn igse_numeric(ki: f64, alpha: f64, beta: f64, b: &[f64], period: f64) -> f64 {
        let n = b.len();
        let dt = period / n as f64;
        let bmax = b.iter().cloned().fold(f64::MIN, f64::max);
        let bmin = b.iter().cloned().fold(f64::MAX, f64::min);
        let delta_b = bmax - bmin;
        let mut acc = 0.0;
        for i in 0..n {
            let dbdt = (b[(i + 1) % n] - b[i]) / dt;
            acc += ki * dbdt.abs().powf(alpha) * delta_b.powf(beta - alpha) * dt;
        }
        acc / period
    }

    #[test]
    fn ki_makes_igse_reproduce_steinmetz_for_a_sinusoid() {
        // The defining property of ki: for a pure sine of peak B̂, iGSE must
        // return the classic Steinmetz Pv = k·fᵅ·B̂ᵝ.
        let sp = sample_material();
        let prof = MagneticsProfile {
            steinmetz: sp, n_turns: 1.0, a_e_m2: 1.0, v_e_m3: 1.0, l_e_m: 1.0, b_sat_t: 0.0,
        };
        let ki = prof.ki();
        let f = 100_000.0_f64;
        let b_hat = 0.1_f64;
        let period = 1.0 / f;
        let n = 4000;
        let samples: Vec<f64> = (0..n)
            .map(|i| b_hat * (2.0 * PI * i as f64 / n as f64).sin())
            .collect();
        let pv_num = igse_numeric(ki, sp.alpha, sp.beta, &samples, period);
        let pv_steinmetz = sp.k * f.powf(sp.alpha) * b_hat.powf(sp.beta);
        let rel = (pv_num - pv_steinmetz).abs() / pv_steinmetz;
        assert!(rel < 0.01, "iGSE sine {pv_num} vs Steinmetz {pv_steinmetz} (rel {rel})");
    }

    #[test]
    fn triangular_closed_form_matches_numeric_igse() {
        let sp = sample_material();
        let prof = MagneticsProfile {
            steinmetz: sp, n_turns: 1.0, a_e_m2: 1.0, v_e_m3: 1.0, l_e_m: 1.0, b_sat_t: 0.0,
        };
        let ki = prof.ki();
        let f = 100_000.0_f64;
        let period = 1.0 / f;
        let l = 1.0; // with N=Ae=1, ΔB = L·ΔI = di_pp
        let di_pp = 0.2; // ΔB = 0.2 T
        for &duty in &[0.25_f64, 0.4, 0.6, 0.75] {
            // Build the triangular B(t): rise over D·T, fall over (1−D)·T.
            let n = 6000;
            let n_on = (duty * n as f64) as usize;
            let delta_b = l * di_pp;
            let samples: Vec<f64> = (0..n)
                .map(|i| {
                    if i < n_on {
                        delta_b * (i as f64 / n_on as f64)
                    } else {
                        delta_b * (1.0 - (i - n_on) as f64 / (n - n_on) as f64)
                    }
                })
                .collect();
            let pv_num = igse_numeric(ki, sp.alpha, sp.beta, &samples, period);
            let pv_closed = prof.core_loss_pv_with_ki(ki, l, duty, di_pp, f);
            let rel = (pv_num - pv_closed).abs() / pv_closed;
            assert!(rel < 0.02, "D={duty}: numeric {pv_num} vs closed {pv_closed} (rel {rel})");
        }
    }

    #[test]
    fn scaling_laws() {
        let prof = MagneticsProfile {
            steinmetz: sample_material(),
            n_turns: 1.0, a_e_m2: 1.0, v_e_m3: 1.0, l_e_m: 1.0, b_sat_t: 0.0,
        };
        let (l, d, di, f) = (1.0, 0.5, 0.2, 100_000.0);
        let base = prof.core_loss_triangular(l, d, di, f);
        // Pv ∝ ΔBᵝ  → doubling ΔB (via di_pp) multiplies by 2ᵝ.
        let dbl_b = prof.core_loss_triangular(l, d, 2.0 * di, f);
        assert!((dbl_b / base - 2.0f64.powf(prof.steinmetz.beta)).abs() < 1e-6);
        // Pv ∝ f_swᵅ.
        let dbl_f = prof.core_loss_triangular(l, d, di, 2.0 * f);
        assert!((dbl_f / base - 2.0f64.powf(prof.steinmetz.alpha)).abs() < 1e-6);
        // Symmetric in D ↔ (1−D).
        let a = prof.core_loss_triangular(l, 0.3, di, f);
        let b = prof.core_loss_triangular(l, 0.7, di, f);
        assert!((a - b).abs() / a < 1e-9);
    }

    #[test]
    fn defaults_and_guards_give_zero() {
        let prof = MagneticsProfile {
            steinmetz: sample_material(),
            n_turns: 1.0, a_e_m2: 1.0, v_e_m3: 1.0, l_e_m: 1.0, b_sat_t: 0.3,
        };
        assert_eq!(prof.core_loss_triangular(1.0, 0.5, 0.0, 100_000.0), 0.0); // no ripple
        assert_eq!(prof.core_loss_triangular(1.0, 0.5, 0.2, 0.0), 0.0); // no freq
        assert!(prof.saturated(1.0, 0.4, 0.1)); // B_dc 0.4 > 0.3 B_sat
        assert!(!prof.saturated(1.0, 0.1, 0.05)); // 0.1 + 0.025 < 0.3
    }
}
