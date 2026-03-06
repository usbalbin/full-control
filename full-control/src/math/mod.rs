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
    let mut res = x / 2.0;
    if x == 0.0 {
        return 0.0;
    }
    if x < 0.0 {
        panic!("Invalid input");
    }
    let mut i = 0;
    while i < 100 {
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