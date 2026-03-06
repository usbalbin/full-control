//! Support for rounding directions and status flags as specified by IEEE 754.
//!
//! Rust does not support the floating point environment so rounding mode is passed as an argument
//! and status flags are returned as part of the result. There is currently not much support for
//! this; most existing ports from musl use a form of `force_eval!` to raise exceptions, but this
//! has no side effects in Rust. Further, correct behavior relies on elementary operations making
//! use of the correct rounding and raising relevant exceptions, which is not the case for Rust.
//!
//! This module exists so no functionality is lost when porting algorithms that respect floating
//! point environment, and so that some functionality may be tested (that which does not rely on
//! side effects from elementary operations). Full support would require wrappers around basic
//! operations, but there is no plan to add this at the current time.

/// A value combined with a floating point status.
pub struct FpResult<T> {
    pub val: T,
}

impl<T> FpResult<T> {
    pub const fn new(val: T) -> Self {
        Self { val }
    }

    /// Return `val` with `Status::OK`.
    pub const fn ok(val: T) -> Self {
        Self { val }
    }
}
