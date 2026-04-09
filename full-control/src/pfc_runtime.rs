//! Per-switching-cycle PFC control functions.
//!
//! These are pure, stateless computations used inside the ISR of a PFC boost
//! converter. All functions are generic over [`Scalar`] so firmware can use
//! f32 or fixed-point without being forced to f64.
//!
//! Typical PCMC call sequence per switching cycle:
//! ```text
//! let i_ref = pcmc_reference_corrected(i_avg_desired, v_in, v_out, i_start, l, t_sw, oc);
//! let duty  = pcmc_comparator(i_ref, i_start, oc, v_in, l, t_sw);
//! ```

use crate::control_2p2z::Scalar;

/// Corrected peak-current reference for PCMC that produces sinusoidal average
/// current in both DCM and CCM.
///
/// In boost DCM the average current is a nonlinear function of the peak:
///
///   `i_avg = 0.5 * i_peak^2 * L * V_out / (V_in * (V_out - V_in) * t_sw)`
///
/// so `i_avg ~ i_ref^2` — a naive sinusoidal reference gives ~50% THD.
///
/// In CCM, volt-second balance forces D = 1 - V_in/V_out regardless of
/// current level, so the relationship is linear and no correction is needed.
///
/// This function computes two references:
/// - CCM: `i_ref = i_avg + (oc + 0.5) * delta_i`
/// - DCM: `i_ref = (1+oc) * sqrt(2 * i_avg * V_in * (V_out-V_in) * t_sw / (V_out * L))`
///
/// Both evaluate to `(1+oc) * delta_i` at the exact CCM/DCM boundary
/// (where `i_start = 0, i_avg = delta_i/2`), so the transition is inherently
/// continuous. A smooth blend via `i_start / (delta_i/2)` removes any residual
/// discontinuity in the neighbourhood.
#[inline(always)]
pub fn pcmc_reference_corrected<S: Scalar>(
    i_avg_desired: S,
    v_in: S,
    v_out: S,
    i_start: S,
    l: S,
    t_sw: S,
    slope_overcomp: S,
) -> S {
    let zero = S::ZERO;
    let half = S::from_f32(0.5);
    let one = S::from_f32(1.0);
    let threshold = S::from_f32(0.1);
    let small = S::from_f32(0.001);
    let two = S::from_f32(2.0);
    let max_duty = S::from_f32(0.98);

    // Estimated duty and ripple current
    let d_est = (one - v_in / v_out).clamp(zero, max_duty);
    let delta_i = v_in * d_est * t_sw / l;

    // CCM reference: inverts i_avg = i_ref - (oc + 0.5) * delta_i
    let i_ref_ccm = i_avg_desired + (slope_overcomp + half) * delta_i;

    // DCM reference: inverts the quadratic i_avg equation,
    // accounting for slope-comp absorption (i_peak = i_ref / (1+oc))
    let i_ref_dcm = if v_in > threshold && i_avg_desired > zero {
        let v_diff = (v_out - v_in).clamp(zero, v_out);
        let inner = two * i_avg_desired * v_in * v_diff * t_sw / (v_out * l);
        if inner > zero {
            (one + slope_overcomp) * inner.sqrt()
        } else {
            zero
        }
    } else {
        zero
    };

    // Blend: 0 in DCM (i_start ~ 0), 1 in CCM (i_start >= delta_i/2)
    let blend = if delta_i > small {
        (i_start / (half * delta_i)).clamp(zero, one)
    } else {
        if i_start > small { one } else { zero }
    };

    i_ref_dcm * (one - blend) + i_ref_ccm * blend
}

/// PCMC comparator: converts peak reference + proportional slope compensation
/// into duty cycle.
///
/// Models the comparator trip point where the sensed ramp meets the
/// compensated reference: `i_start + (V_in/L) * t = i_ref - S_e * t`,
/// with `S_e = oc * V_in/L` (proportional to on-ramp for constant absorption).
#[inline(always)]
pub fn pcmc_comparator<S: Scalar>(
    i_ref: S,
    i_start: S,
    slope_overcomp: S,
    v_in: S,
    l: S,
    t_sw: S,
) -> S {
    let zero = S::ZERO;
    let max_duty = S::from_f32(0.98);

    let di_dt_on = v_in / l;
    let se = slope_overcomp * di_dt_on;
    let total_ramp = di_dt_on + se;

    let t_on = if total_ramp > zero && i_ref > i_start {
        (i_ref - i_start) / total_ramp
    } else {
        zero
    };

    (t_on / t_sw).clamp(zero, max_duty)
}

/// CrCM on-time from voltage loop output.
///
/// `t_on = 2L * k / (V_in_pk * N)`, clamped to 95% of `t_sw_max`.
/// This gives `i_avg = k * sin(theta) / N` naturally because
/// `i_peak = V_in * t_on / L = 2k * sin(theta) / N` and in CrCM
/// the average is half the peak.
#[inline(always)]
pub fn crcm_on_time<S: Scalar>(
    l: S,
    k: S,
    v_in_pk: S,
    num_phases: S,
    t_sw_max: S,
) -> S {
    let zero = S::ZERO;
    let one = S::from_f32(1.0);
    let two = S::from_f32(2.0);
    let max_frac = S::from_f32(0.95);

    if v_in_pk > one {
        let t_on = two * l * k / (v_in_pk * num_phases);
        let limit = max_frac * t_sw_max;
        if t_on < limit { t_on } else { limit }
    } else {
        zero
    }
}

/// Zero-crossing blanking for synchronous / totem-pole PFC.
///
/// Near zero crossings the rectified input voltage is too low for proper
/// boost operation. Returns `true` when blanking should be active
/// (switch to diode mode): `v_in < v_out * threshold`.
#[inline(always)]
pub fn zc_blanking_active<S: Scalar>(v_in: S, v_out: S, threshold: S) -> bool {
    v_in < v_out * threshold
}

#[cfg(test)]
mod tests {
    use super::*;

    const L: f32 = 500e-6;
    const T_SW: f32 = 1.0 / 65e3;
    const V_OUT: f32 = 400.0;
    const OC: f32 = 1.5;

    #[test]
    fn dcm_reference_at_zero_i_start() {
        // Pure DCM: i_start = 0, blend = 0, should get DCM formula
        let v_in: f32 = 100.0;
        let i_avg: f32 = 0.5;
        let i_ref = pcmc_reference_corrected(i_avg, v_in, V_OUT, 0.0, L, T_SW, OC);

        // Manual DCM formula using std sqrt for reference
        let v_diff = V_OUT - v_in;
        let inner = 2.0_f64 * 0.5 * 100.0 * v_diff as f64 * T_SW as f64
            / (V_OUT as f64 * L as f64);
        let expected = (1.0 + OC as f64) * inner.sqrt();

        // ~5% tolerance for micromath sqrt approximation
        let rel_err = ((i_ref as f64 - expected) / expected).abs();
        assert!(
            rel_err < 0.05,
            "DCM ref: got {i_ref}, expected {expected}, rel_err={rel_err:.3}"
        );
    }

    #[test]
    fn ccm_reference_at_large_i_start() {
        // Deep CCM: i_start >> delta_i/2, blend = 1, should get CCM formula
        let v_in: f32 = 200.0;
        let i_avg: f32 = 5.0;
        let d_est = (1.0 - v_in / V_OUT).clamp(0.0, 0.98);
        let delta_i = v_in * d_est * T_SW / L;
        let i_start = delta_i * 2.0; // well above delta_i/2

        let i_ref = pcmc_reference_corrected(i_avg, v_in, V_OUT, i_start, L, T_SW, OC);
        let expected = i_avg + (OC + 0.5) * delta_i;

        assert!(
            (i_ref - expected).abs() < 0.01,
            "CCM ref: got {i_ref}, expected {expected}"
        );
    }

    #[test]
    fn boundary_continuity() {
        // At the CCM/DCM boundary: i_start = 0, i_avg = delta_i / 2
        // Both formulas should give (1+oc) * delta_i
        let v_in: f32 = 150.0;
        let d_est = (1.0 - v_in / V_OUT).clamp(0.0, 0.98);
        let delta_i = v_in * d_est * T_SW / L;
        let i_avg = delta_i / 2.0;

        // CCM: i_avg + (oc+0.5)*delta_i = delta_i/2 + (oc+0.5)*delta_i = (1+oc)*delta_i
        let ccm = i_avg + (OC + 0.5) * delta_i;

        // DCM: (1+oc)*sqrt(2*i_avg*v_in*(v_out-v_in)*t_sw/(v_out*l))
        //     = (1+oc)*sqrt(delta_i * v_in*(v_out-v_in)*t_sw/(v_out*l))
        //     = (1+oc)*sqrt((v_in*d*t_sw/l) * v_in*(v_out-v_in)*t_sw/(v_out*l))
        //     with d = 1-v_in/v_out = (v_out-v_in)/v_out:
        //     = (1+oc)*sqrt(v_in^2*(v_out-v_in)^2*t_sw^2/(v_out^2*l^2))
        //     = (1+oc)*v_in*(v_out-v_in)*t_sw/(v_out*l)
        //     = (1+oc)*delta_i  (since delta_i = v_in*d*t_sw/l)
        let v_diff = V_OUT - v_in;
        let inner = 2.0 * i_avg * v_in * v_diff * T_SW / (V_OUT * L);
        let dcm = (1.0 + OC) * inner.sqrt();

        assert!(
            (ccm - dcm).abs() < 0.01,
            "Boundary: CCM={ccm}, DCM={dcm}"
        );
    }

    #[test]
    fn comparator_known_duty() {
        let v_in: f32 = 200.0;
        let i_start: f32 = 2.0;
        let i_ref: f32 = 4.0;

        let duty = pcmc_comparator(i_ref, i_start, OC, v_in, L, T_SW);

        // di_dt_on = v_in/l = 200/500e-6 = 400_000
        // se = 1.5 * 400_000 = 600_000
        // total_ramp = 1_000_000
        // t_on = (4-2)/1_000_000 = 2e-6
        // duty = 2e-6 / t_sw = 2e-6 * 65e3 = 0.13
        let expected = 2e-6 * 65e3;
        assert!(
            (duty - expected as f32).abs() < 0.001,
            "duty: got {duty}, expected {expected}"
        );
    }

    #[test]
    fn comparator_zero_when_ref_below_start() {
        let duty = pcmc_comparator(1.0_f32, 2.0, OC, 200.0, L, T_SW);
        assert_eq!(duty, 0.0);
    }

    #[test]
    fn crcm_on_time_basic() {
        let l: f32 = 500e-6;
        let k: f32 = 2.0;
        let v_in_pk: f32 = 325.0;
        let t_sw_max: f32 = T_SW;

        let t_on = crcm_on_time(l, k, v_in_pk, 1.0_f32, t_sw_max);
        // 2 * 500e-6 * 2 / (325 * 1) = 2e-3 / 325 ≈ 6.15e-6
        let expected: f32 = 2.0 * 500e-6 * 2.0 / 325.0;
        assert!(
            (t_on - expected).abs() < 1e-9,
            "t_on: got {t_on}, expected {expected}"
        );
    }

    #[test]
    fn crcm_on_time_clamped() {
        let l: f32 = 500e-6;
        let k: f32 = 500.0; // very large → would exceed t_sw
        let v_in_pk: f32 = 50.0;
        let t_sw_max: f32 = T_SW;

        let t_on = crcm_on_time(l, k, v_in_pk, 1.0_f32, t_sw_max);
        let limit = 0.95 * t_sw_max;
        assert!(
            (t_on - limit).abs() < 1e-9,
            "Should clamp to 95%: got {t_on}, expected {limit}"
        );
    }

    #[test]
    fn crcm_on_time_zero_at_low_vin() {
        let t_on = crcm_on_time(500e-6_f32, 5.0, 0.5, 1.0, T_SW);
        assert_eq!(t_on, 0.0);
    }

    #[test]
    fn zc_blanking_below_threshold() {
        assert!(zc_blanking_active(15.0_f32, 400.0, 0.05));
    }

    #[test]
    fn zc_blanking_above_threshold() {
        assert!(!zc_blanking_active(25.0_f32, 400.0, 0.05));
    }

    #[test]
    fn reference_zero_when_v_in_zero() {
        let i_ref = pcmc_reference_corrected(1.0_f32, 0.0, V_OUT, 0.0, L, T_SW, OC);
        assert_eq!(i_ref, 0.0);
    }

    #[test]
    fn reference_zero_when_i_avg_zero() {
        let i_ref = pcmc_reference_corrected(0.0_f32, 200.0, V_OUT, 0.0, L, T_SW, OC);
        assert_eq!(i_ref, 0.0);
    }
}
