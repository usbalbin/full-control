mod atan;
mod floor;
mod generic_floor;
mod rem_pio2;
mod rem_pio2_large;
mod scalbn;
mod support_env;

mod k_tan;
mod tan;

pub use atan::atan;
pub use tan::tan;

// Significant number of bits for f64
pub const SIG_BITS: u32 = 52;
pub const SIG_MASK: u64 = (1 << SIG_BITS) - 1;
pub const EXP_SAT: u32 = 0b11111111111;
pub const EXP_BIAS: u32 = 1023;

const fn from_parts(negative: bool, exponent: u32, significand: u64) -> f64 {
    let sign = if negative { 1 } else { 0 };
    f64::from_bits(
        (sign << (64 - 1)) | (((exponent & EXP_SAT) as u64) << SIG_BITS) | (significand & SIG_MASK),
    )
}

pub const fn fabs(x: f64) -> f64 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}

pub const fn sqrt(x: f64) -> f64 {
    if x == 0.0 {
        return x; // ±0 → ±0 (preserves the sign of zero)
    }
    if x < 0.0 {
        panic!("sqrt of a negative number");
    }
    if !(x < f64::INFINITY) {
        return x; // +∞ → +∞ ; NaN → NaN (both fail `x < ∞`)
    }
    // Seed by halving the exponent via the classic sqrt bit hack, then refine
    // with Newton–Raphson. The seed is within ~a factor of 2 across the ENTIRE
    // normal range, so a handful of quadratically-converging steps suffice.
    // (The old `x / 2.0` seed needed O(exponent) steps and so silently failed
    // to converge outside roughly [2^-190, 2^180] with a fixed iteration count.)
    let mut res = f64::from_bits((x.to_bits() >> 1) + 0x1ff8_0000_0000_0000u64);
    let mut i = 0;
    while i < 8 {
        res = 0.5 * (res + x / res);
        i += 1;
    }
    res
}

pub const fn pow2(x: f64) -> f64 {
    x * x
}

// From https://github.com/rust-lang/libm/blob/8f7436d260f000f054042545bb6e4c0d99fe35b2/libm/src/math/mod.rs

macro_rules! i {
    ($array:expr, $index:expr) => {
        $array[$index]
    };
    ($array:expr, $index:expr, = , $rhs:expr) => {
        $array[$index] = $rhs;
    };
    ($array:expr, $index:expr, -= , $rhs:expr) => {
        $array[$index] -= $rhs;
    };
    ($array:expr, $index:expr, += , $rhs:expr) => {
        $array[$index] += $rhs;
    };
    ($array:expr, $index:expr, &= , $rhs:expr) => {
        $array[$index] &= $rhs;
    };
    ($array:expr, $index:expr, == , $rhs:expr) => {
        $array[$index] == $rhs
    };
}

// Temporary macro to avoid panic codegen for division (in debug mode too). At
// the time of this writing this is only used in a few places, and once
// rust-lang/rust#72751 is fixed then this macro will no longer be necessary and
// the native `/` operator can be used and panics won't be codegen'd.
macro_rules! div {
    ($a:expr, $b:expr) => {
        $a / $b
    };
}

pub(crate) use div;
pub(crate) use i;

// From https://github.com/rust-lang/libm/blob/master/libm/src/math/mod.rs#L394

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::sqrt;

    #[test]
    fn sqrt_matches_std_across_wide_range() {
        // Includes exponents that the old x/2-seed, fixed-100-iteration Newton
        // failed to converge for (roughly outside [2^-190, 2^180]).
        let xs = [
            0.0_f64, 1.0, 2.0, 4.0, 0.25, 1e-6, 1e6, 1e-30, 1e30,
            2.0_f64.powi(180), 2.0_f64.powi(300), 2.0_f64.powi(-300),
            1e-300, 1e300, f64::MIN_POSITIVE, 123456.789,
        ];
        for &x in &xs {
            let got = sqrt(x);
            let want = x.sqrt();
            let tol = want * 1e-12 + 1e-300;
            assert!(
                (got - want).abs() <= tol,
                "sqrt({x:e}) = {got:e}, std = {want:e}",
            );
        }
    }

    #[test]
    fn sqrt_special_values() {
        assert_eq!(sqrt(0.0), 0.0);
        assert!(sqrt(f64::INFINITY).is_infinite());
        assert!(sqrt(f64::NAN).is_nan());
    }
}