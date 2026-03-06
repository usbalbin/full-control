/* SPDX-License-Identifier: MIT
 * origin: musl src/math/floor.c */

//! Generic `floor` algorithm.
//!
//! Note that this uses the algorithm from musl's `floorf` rather than `floor` or `floorl` because
//! performance seems to be better (based on icount) and it does not seem to experience rounding
//! errors on i386.

use super::{support_env::FpResult, EXP_BIAS, EXP_SAT, SIG_BITS, SIG_MASK};

#[inline]
pub const fn floor(x: f64) -> f64 {
    floor_status(x).val
}

/// Returns the exponent, not adjusting for bias, not accounting for subnormals or zero.
const fn ex(x: f64) -> u32 {
    (x.to_bits() >> SIG_BITS) as u32 & EXP_SAT
}

/// Extract the exponent and adjust it for bias, not accounting for subnormals or zero.
const fn exp_unbiased(x: f64) -> i32 {
    ex(x) as i32 - (EXP_BIAS as i32)
}

#[inline]
pub const fn floor_status(x: f64) -> FpResult<f64> {
    let mut ix = x.to_bits();
    let e = exp_unbiased(x);

    // If the represented value has no fractional part, no truncation is needed.
    if e >= SIG_BITS as i32 {
        return FpResult::ok(x);
    }

    let res = if e >= 0 {
        // |x| >= 1.0
        let m = (SIG_MASK >> e) as u64;
        if ix & m == 0 {
            // Portion to be masked is already zero; no adjustment needed.
            return FpResult::ok(x);
        }

        if x.is_sign_negative() {
            ix += m;
        }

        ix &= !m;
        f64::from_bits(ix)
    } else {
        if x.is_sign_positive() {
            // 0.0 <= x < 1.0; rounding down goes toward +0.0.
            0.0
        } else if ix << 1 != 0 {
            // -1.0 < x < 0.0; rounding down goes toward -1.0.
            -1.0
        } else {
            // -0.0 remains unchanged
            x
        }
    };

    FpResult::new(res)
}
