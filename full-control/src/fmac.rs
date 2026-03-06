/// STM32G474 FMAC peripheral IIR emulator — 2-pole 2-zero, q1.15 arithmetic.
///
/// Implements the FMAC recurrence faithfully:
///
///   y[n] = 2^R × ( b[0]·x[n] + b[1]·x[n-1] + b[2]·x[n-2]
///                + a[0]·y[n-1] + a[1]·y[n-2] )
///
/// All coefficients and samples are **q1.15** (i16 raw bits, range [-1, +1)).
/// Products are accumulated in `i64` — modelling the FMAC's wider internal
/// accumulator — so rounding to q1.15 happens only once per cycle, exactly as
/// the hardware does. This cannot be replicated by `TwoPoleTwoZero<I1F15>`
/// because per-term addition in i16 overflows when the sum of five products
/// exceeds ±1.0 (which occurs with typical b-coefficients > 1).
///
/// # Coefficient preparation
///
/// Use [`TwoPoleTwoZeroParams::min_fmac_r`] and [`TwoPoleTwoZeroParams::fmac_coeffs`]
/// to obtain the gain exponent R and the q1.15 coefficient bits from the f32
/// code-domain weights.
///
/// # I/O normalization
///
/// FMAC inputs and outputs must be in q1.15. With FMAC_SCALE = 2^(15−R):
/// - **code → q1.15 input**: `x_bits = code << R`   (multiply by 2^R)
/// - **q1.15 output → code**: `code = (y_bits + (1 << (R-1))) >> R`  (divide with rounding)
///
/// Limits (`y_min`, `y_max`) must be pre-scaled the same way:
/// `y_max_bits = dac_max_code << R`.
pub struct FmacIir {
    /// `[b0, b1, b2]` in q1.15 = actual_b_coeff / 2^R
    b: [i16; 3],
    /// `[a1, a2]` in q1.15 = actual_a_coeff / 2^R
    a: [i16; 2],
    /// FMAC gain exponent R ∈ [1, 14].  Net accumulator right-shift = 15 − R.
    r: u32,
    /// Input history `[x[n-1], x[n-2]]` in q1.15.
    x: [i16; 2],
    /// Output history `[y[n-1], y[n-2]]` in q1.15 (clamped value stored for anti-windup).
    y: [i16; 2],
    y_min: i16,
    y_max: i16,
}

impl FmacIir {
    /// Create a new filter.
    ///
    /// - `b`: `[b0, b1, b2]` coefficient bits in q1.15 (= real_coeff / 2^R × 32768, rounded)
    /// - `a`: `[a1, a2]` coefficient bits in q1.15
    /// - `r`: FMAC gain exponent.  Must satisfy 1 ≤ r ≤ 14 (so shift = 15−r is in [1, 14]).
    /// - `y_min`, `y_max`: output clamp limits in q1.15 (= code_limit << r)
    pub const fn new(b: [i16; 3], a: [i16; 2], r: u32, y_min: i16, y_max: i16) -> Self {
        Self { b, a, r, x: [0; 2], y: [0; 2], y_min, y_max }
    }

    /// Run one filter step.
    ///
    /// `x0` is the new input in q1.15.  Returns the clamped q1.15 output.
    /// The clamped value is written back into the y-history (anti-windup).
    pub fn update(&mut self, x0: i16) -> i16 {
        // Accumulate all five MAC terms in i64.
        // Each i16×i16 product is exact in i32 (q2.30); extending to i64 is free.
        // Maximum |acc| ≈ 5 × 32767² ≈ 5.4e9, well within i64 (9.2e18).
        let acc: i64 = (self.b[0] as i64) * (x0 as i64)
            + (self.b[1] as i64) * (self.x[0] as i64)
            + (self.b[2] as i64) * (self.x[1] as i64)
            + (self.a[0] as i64) * (self.y[0] as i64)
            + (self.a[1] as i64) * (self.y[1] as i64);

        // acc is in q2.30 (30 fractional bits from i16×i16 products).
        // FMAC gain 2^R is a left-shift; combined with the 30→15 conversion:
        //   net right-shift = 30 − 15 − R = 15 − R.
        // Add half-ULP rounding before shifting.
        let shift = 15 - self.r;
        let y_raw = (acc + (1i64 << (shift - 1))) >> shift;

        // Clamp and store (anti-windup: history tracks actual output, not raw).
        let y_out = y_raw.clamp(self.y_min as i64, self.y_max as i64) as i16;
        self.x = [x0, self.x[0]];
        self.y = [y_out, self.y[0]];
        y_out
    }

    /// Pre-load input and output histories for bumpless transfer.
    ///
    /// Set `y` to the last output in q1.15 and `e` to the last error in q1.15
    /// before switching to this controller.
    pub fn prime(&mut self, y: i16, e: i16) {
        self.y = [y, y];
        self.x = [e, e];
    }

    /// Return the most-recent (clamped) output from the history.
    pub fn last_output(&self) -> i16 {
        self.y[0]
    }

    /// Replace the most-recently stored output with `y`.
    ///
    /// Use this when an external clamp further restricts the output beyond
    /// `y_min`/`y_max` — rare if the limits are set correctly at construction.
    pub fn set_last_output(&mut self, y: i16) {
        self.y[0] = y;
    }
}
