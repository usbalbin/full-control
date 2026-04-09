//! Clarke and Park frame transforms for 3-phase systems.
//!
//! All transforms are generic over [`Scalar`] so firmware can use f32 or
//! fixed-point. The Park transform takes pre-computed `sin(theta)` and
//! `cos(theta)` so the caller can use whatever method suits the platform
//! (lookup table, CORDIC, etc.).
//!
//! Conventions:
//! - **Amplitude-invariant** Clarke transform (standard in power electronics).
//! - Park transform: d-axis aligned with the rotating voltage vector.

use crate::control_2p2z::Scalar;

/// Clarke transform: ABC → αβ (amplitude-invariant).
///
/// Transforms three-phase quantities into the stationary two-axis frame.
/// For balanced systems (a + b + c = 0), this simplifies to α = a,
/// β = (a + 2b) / √3.
#[inline(always)]
pub fn clarke<S: Scalar>(a: S, b: S, c: S) -> (S, S) {
    let two_thirds = S::from_f32(2.0 / 3.0);
    let half = S::from_f32(0.5);
    let inv_sqrt3 = S::from_f32(0.57735027); // 1/√3

    // α = 2/3 * (a - b/2 - c/2)
    let alpha = two_thirds * (a - half * b - half * c);
    // β = 2/3 * (√3/2 * (b - c)) = (b - c) / √3
    let beta = (b - c) * inv_sqrt3;
    (alpha, beta)
}

/// Inverse Clarke transform: αβ → ABC.
#[inline(always)]
pub fn inv_clarke<S: Scalar>(alpha: S, beta: S) -> (S, S, S) {
    let half = S::from_f32(0.5);
    let sqrt3_2 = S::from_f32(0.8660254); // √3/2

    let a = alpha;
    let b = -half * alpha + sqrt3_2 * beta;
    let c = -half * alpha - sqrt3_2 * beta;
    (a, b, c)
}

/// Park transform: αβ → dq (rotating frame).
///
/// `sin_theta` and `cos_theta` are the sine and cosine of the grid angle.
/// The caller computes these however suits the platform.
#[inline(always)]
pub fn park<S: Scalar>(alpha: S, beta: S, sin_theta: S, cos_theta: S) -> (S, S) {
    let d = alpha * cos_theta + beta * sin_theta;
    let q = beta * cos_theta - alpha * sin_theta;
    (d, q)
}

/// Inverse Park transform: dq → αβ.
#[inline(always)]
pub fn inv_park<S: Scalar>(d: S, q: S, sin_theta: S, cos_theta: S) -> (S, S) {
    let alpha = d * cos_theta - q * sin_theta;
    let beta = d * sin_theta + q * cos_theta;
    (alpha, beta)
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f32 = 1e-4;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < TOL
    }

    #[test]
    fn clarke_balanced_phase_a_peak() {
        // Phase A at peak: a=1, b=-0.5, c=-0.5 (balanced)
        let (alpha, beta) = clarke(1.0_f32, -0.5, -0.5);
        // α = 2/3 * (1 + 0.25 + 0.25) = 1.0
        assert!(approx_eq(alpha, 1.0), "alpha={alpha}");
        // β = (-0.5 - (-0.5)) / √3 = 0
        assert!(approx_eq(beta, 0.0), "beta={beta}");
    }

    #[test]
    fn clarke_balanced_120deg() {
        // Phase B at peak: a=-0.5, b=1, c=-0.5
        let (alpha, beta) = clarke(-0.5_f32, 1.0, -0.5);
        // α = 2/3 * (-0.5 - 0.5 + 0.25) = -0.5
        assert!(approx_eq(alpha, -0.5), "alpha={alpha}");
        // β = (1 - (-0.5)) / √3 = 1.5/√3 ≈ 0.866
        assert!(approx_eq(beta, 0.8660254), "beta={beta}");
    }

    #[test]
    fn clarke_inv_clarke_roundtrip() {
        let (a, b, c) = (0.8_f32, -0.3, -0.5);
        let (alpha, beta) = clarke(a, b, c);
        let (a2, b2, c2) = inv_clarke(alpha, beta);
        assert!(approx_eq(a, a2), "a: {a} vs {a2}");
        assert!(approx_eq(b, b2), "b: {b} vs {b2}");
        assert!(approx_eq(c, c2), "c: {c} vs {c2}");
    }

    #[test]
    fn park_at_zero_angle() {
        // theta=0: sin=0, cos=1 → d=alpha, q=beta
        let (d, q) = park(0.5_f32, 0.3, 0.0, 1.0);
        assert!(approx_eq(d, 0.5), "d={d}");
        assert!(approx_eq(q, 0.3), "q={q}");
    }

    #[test]
    fn park_at_90deg() {
        // theta=90°: sin=1, cos=0 → d=beta, q=-alpha
        let (d, q) = park(0.5_f32, 0.3, 1.0, 0.0);
        assert!(approx_eq(d, 0.3), "d={d}");
        assert!(approx_eq(q, -0.5), "q={q}");
    }

    #[test]
    fn park_inv_park_roundtrip() {
        let (alpha, beta) = (0.7_f32, -0.4);
        let sin_t = 0.6_f32;
        let cos_t = 0.8_f32;
        let (d, q) = park(alpha, beta, sin_t, cos_t);
        let (a2, b2) = inv_park(d, q, sin_t, cos_t);
        assert!(approx_eq(alpha, a2), "alpha: {alpha} vs {a2}");
        assert!(approx_eq(beta, b2), "beta: {beta} vs {b2}");
    }

    #[test]
    fn full_roundtrip_abc_dq_abc() {
        let (a, b, c) = (0.8_f32, -0.3, -0.5);
        let sin_t = 0.5_f32;
        let cos_t = 0.8660254_f32; // 30°

        let (alpha, beta) = clarke(a, b, c);
        let (d, q) = park(alpha, beta, sin_t, cos_t);
        let (alpha2, beta2) = inv_park(d, q, sin_t, cos_t);
        let (a2, b2, c2) = inv_clarke(alpha2, beta2);

        assert!(approx_eq(a, a2), "a: {a} vs {a2}");
        assert!(approx_eq(b, b2), "b: {b} vs {b2}");
        assert!(approx_eq(c, c2), "c: {c} vs {c2}");
    }
}
