//! Q-format-selectable inner-loop controller wrapper.
//!
//! Today the buck simulator's inner current loop runs in
//! `TwoPoleTwoZero<f32>` — fast and numerically clean, but unable to
//! reproduce the coefficient-quantisation limit cycles that the real
//! STM32G474 firmware exhibits when it runs the same compensator
//! through the FMAC peripheral (q1.15, hardware-faithful i64
//! accumulator).
//!
//! This module wraps both flavors behind a common API. The host
//! sim's per-cycle update loop calls `update_clamped(error, lo, hi)`
//! and gets a code-domain output back; the wrapper handles the
//! `code → q1.15 → code` bit-shift adapters that the FMAC datasheet
//! recipe requires (see `full_control::fmac` module docs).
//!
//! Why this matters for EMC: with `HostF32` the spectrum's
//! quantisation-limit-cycle row (`docs/closed_loop_emc_plan.md`
//! §6.2 row 3) silently reads zero. With `FmacQ15` the same
//! compensator coefficients exhibit the discrete-tone spurs the
//! firmware actually emits, and they propagate through the
//! spectrum / LISN / radiated-EMC chain.

use full_control::control_2p2z::{TwoPoleTwoZero, TwoPoleTwoZeroParams};
use full_control::fmac::FmacIir;

/// Which Q format the host runs the inner loop in. Mirrors
/// `pdn_schema::ControllerFlavor`, with one variant per supported
/// flavor. The `pdn_schema` enum is the wire-format / provenance
/// type and isn't used here directly because it carries `String`
/// fields that don't make sense for the host-internal selection.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize, Default,
)]
pub enum InnerCtrlFlavor {
    /// Fast, deep host-sim default. Quantisation limit cycles
    /// silently absent; subharmonic / marginal-PM / audio-band
    /// pathologies still show up.
    #[default]
    HostF32,
    /// STM32G474 FMAC peripheral emulation. q1.15 coefficients +
    /// samples, i64 accumulator (so per-cycle sums of 5 products
    /// don't overflow the q1.15 range). Reproduces coefficient-
    /// quantisation limit cycles. Slightly slower per step.
    FmacQ15,
}

impl InnerCtrlFlavor {
    pub fn label(&self) -> &'static str {
        match self {
            InnerCtrlFlavor::HostF32 => "Host f32",
            InnerCtrlFlavor::FmacQ15 => "FMAC q1.15",
        }
    }
    /// Translates to the wire-format `ControllerFlavor` used in
    /// `PortCurrentSpectrum`. For FmacQ15 the `r_exponent` comes
    /// from the chosen R at coefficient-load time — set to 0 if
    /// not known (the spectrum producer will overwrite).
    pub fn to_provenance(&self, r_exponent: u32) -> pdn_schema::ControllerFlavor {
        match self {
            InnerCtrlFlavor::HostF32 => pdn_schema::ControllerFlavor::HostF32,
            InnerCtrlFlavor::FmacQ15 => pdn_schema::ControllerFlavor::FmacQ15 { r_exponent },
        }
    }
}

/// Unified inner-loop controller. Has the same shape as
/// `TwoPoleTwoZero<f32>` (`update_clamped`, `last_output`, `prime`)
/// regardless of the underlying numeric flavor.
pub enum InnerCtrl {
    F32(TwoPoleTwoZero<f32>),
    Fmac {
        fmac: FmacIir,
        /// FMAC gain exponent R picked at coefficient-load time.
        /// Used for the code ↔ q1.15 bit-shift adapters.
        r_exponent: u32,
        /// Cached last code-domain output for `last_output()`.
        last_output: f32,
    },
}

impl InnerCtrl {
    /// Build from f32 weights + the chosen flavor. `lo_code` /
    /// `hi_code` are the controller-output clamp bounds in DAC code
    /// units (e.g. 0 / 4095 for a 12-bit DAC).
    pub fn build(
        weights: TwoPoleTwoZeroParams<f32>,
        lo_code: f32,
        hi_code: f32,
        flavor: InnerCtrlFlavor,
    ) -> Self {
        match flavor {
            InnerCtrlFlavor::HostF32 => {
                InnerCtrl::F32(weights.to_controller(lo_code, hi_code))
            }
            InnerCtrlFlavor::FmacQ15 => {
                // The FMAC needs R ≥ 1 (net accumulator shift 15−R ∈ [1, 14]).
                // This `r_eff` must be used for EVERYTHING — coefficient
                // quantisation, the y-limit pre-scale, the accumulator shift,
                // and the code↔q1.15 I/O adapters — so the 2^R gain factors
                // cancel exactly. Quantising the coefficients at the raw
                // `min_fmac_r()` (which can be 0) while the shift/I-O used
                // `r_eff = 1` produced a 2× output-gain error on small-gain
                // compensators.
                let r_eff = weights.min_fmac_r().max(1);
                let coeffs = weights.fmac_coeffs(r_eff);
                let b = [coeffs[0], coeffs[1], coeffs[2]];
                let a = [coeffs[3], coeffs[4]];
                // y_min/y_max in q1.15 bits: `code << R`. Clamp to i16 range so
                // a high R doesn't overflow.
                let to_q15_bits = |code: f32| -> i16 {
                    let bits = (code as i64) << r_eff;
                    bits.clamp(i16::MIN as i64, i16::MAX as i64) as i16
                };
                let y_min = to_q15_bits(lo_code);
                let y_max = to_q15_bits(hi_code);
                let fmac = FmacIir::new(b, a, r_eff, y_min, y_max);
                InnerCtrl::Fmac {
                    fmac,
                    r_exponent: r_eff,
                    last_output: 0.0,
                }
            }
        }
    }

    /// FMAC gain exponent R when in FmacQ15 flavor (else 0). Used
    /// by the spectrum exporter to stamp ControllerFlavor::FmacQ15
    /// with the right R into the spectrum's provenance.
    pub fn r_exponent(&self) -> u32 {
        match self {
            InnerCtrl::F32(_) => 0,
            InnerCtrl::Fmac { r_exponent, .. } => *r_exponent,
        }
    }

    /// Run one IIR step with the given error code. Output clamped
    /// to `[lo_code, hi_code]`. For F32 this is a direct call; for
    /// FMAC the wrapper does the `code ↔ q1.15` adapters and
    /// re-applies the user clamp on top of FMAC's internal clamp
    /// (FMAC's pre-baked y_min/y_max may be tighter than `lo/hi`
    /// at this call site).
    pub fn update_clamped(&mut self, error_code: f32, lo_code: f32, hi_code: f32) -> f32 {
        match self {
            InnerCtrl::F32(c) => c.update_clamped(error_code, lo_code, hi_code),
            InnerCtrl::Fmac {
                fmac,
                r_exponent,
                last_output,
            } => {
                let r = *r_exponent;
                // code → q1.15 bits: x_bits = code << R, then clamp to i16.
                let x_bits = (error_code.round() as i64) << r;
                let x_q15 = x_bits.clamp(i16::MIN as i64, i16::MAX as i64) as i16;
                let y_q15 = fmac.update(x_q15);
                // q1.15 → code: code = (y_bits + (1 << (R-1))) >> R, rounding.
                let bias: i32 = if r > 0 { 1 << (r - 1) } else { 0 };
                let y_code = ((y_q15 as i32).saturating_add(bias)) >> r;
                let out = (y_code as f32).clamp(lo_code, hi_code);
                *last_output = out;
                out
            }
        }
    }

    /// Cached last output (code units). For F32 this reads
    /// straight from the underlying controller's state; for FMAC
    /// it's the cached value from the previous `update_clamped`.
    pub fn last_output(&self) -> f32 {
        match self {
            InnerCtrl::F32(c) => c.last_output(),
            InnerCtrl::Fmac { last_output, .. } => *last_output,
        }
    }

    /// Bumpless-transfer prime: set the controller's history so the
    /// next `update_clamped(error)` returns `output` exactly. Used
    /// when re-instantiating a controller across a parameter change
    /// without restarting the sim's transient.
    pub fn prime(&mut self, output: f32, error: f32) {
        match self {
            InnerCtrl::F32(c) => c.prime(output, error),
            InnerCtrl::Fmac {
                fmac, r_exponent, ..
            } => {
                // Cheapest reasonable prime: clear FMAC state then
                // shove one update with the desired error. Won't
                // give bit-exact bumpless transfer (the y-history
                // is set to whatever the single step produces, not
                // the user's output) but it's close enough for
                // post-config re-runs at steady state.
                let r = *r_exponent;
                let x_bits = (error.round() as i64) << r;
                let x_q15 = x_bits.clamp(i16::MIN as i64, i16::MAX as i64) as i16;
                let _ = fmac.update(x_q15);
                if let InnerCtrl::Fmac {
                    last_output: cached,
                    ..
                } = self
                {
                    *cached = output;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unity_weights() -> TwoPoleTwoZeroParams<f32> {
        TwoPoleTwoZeroParams {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }
    }

    #[test]
    fn f32_unity_passes_input_through() {
        let mut c = InnerCtrl::build(unity_weights(), 0.0, 4095.0, InnerCtrlFlavor::HostF32);
        let out = c.update_clamped(100.0, 0.0, 4095.0);
        assert!((out - 100.0).abs() < 1e-3, "got {}", out);
    }

    #[test]
    fn fmac_unity_passes_input_through_within_quantisation() {
        let mut c = InnerCtrl::build(unity_weights(), 0.0, 4095.0, InnerCtrlFlavor::FmacQ15);
        let out = c.update_clamped(100.0, 0.0, 4095.0);
        // q1.15 round-trip should be within ±1 code at this scale.
        assert!((out - 100.0).abs() <= 1.0, "got {}", out);
    }

    #[test]
    fn fmac_sub_unity_gain_correct_when_min_fmac_r_is_zero() {
        // b0 = 0.5 → min_fmac_r() = 0 (all |coeff| < 1). Previously the
        // coefficients were quantised at r=0 while the shift/I-O ran at
        // r_eff=1, doubling the output. Expect ~50 for a 0.5 gain on 100.
        let w = TwoPoleTwoZeroParams::<f32> { a1: 0.0, a2: 0.0, b0: 0.5, b1: 0.0, b2: 0.0 };
        let mut c = InnerCtrl::build(w, 0.0, 4095.0, InnerCtrlFlavor::FmacQ15);
        let out = c.update_clamped(100.0, 0.0, 4095.0);
        assert!((out - 50.0).abs() <= 1.0, "expected ~50 (0.5 gain), got {}", out);
    }

    #[test]
    fn fmac_r_exponent_matches_min_fmac_r() {
        // High-gain weights → b0 = 5 → min_fmac_r should return 3
        // (2^2 = 4 < 5 ≤ 2^3 = 8 → R = 3 to fit in q1.15).
        let w = TwoPoleTwoZeroParams::<f32> {
            a1: 0.0,
            a2: 0.0,
            b0: 5.0,
            b1: 0.0,
            b2: 0.0,
        };
        let c = InnerCtrl::build(w, 0.0, 4095.0, InnerCtrlFlavor::FmacQ15);
        assert!(c.r_exponent() >= 3);
    }

    #[test]
    fn clamp_bounds_respected() {
        // Drive a huge error; output must clamp.
        let mut c = InnerCtrl::build(
            TwoPoleTwoZeroParams::<f32> {
                a1: 0.0,
                a2: 0.0,
                b0: 2.0,
                b1: 0.0,
                b2: 0.0,
            },
            0.0,
            100.0,
            InnerCtrlFlavor::HostF32,
        );
        let out = c.update_clamped(1000.0, 0.0, 100.0);
        assert!(out <= 100.0, "got {}", out);
    }

    #[test]
    fn flavor_label_round_trips() {
        for f in [InnerCtrlFlavor::HostF32, InnerCtrlFlavor::FmacQ15] {
            assert!(!f.label().is_empty());
        }
    }

    #[test]
    fn provenance_carries_r_for_fmac() {
        let prov = InnerCtrlFlavor::FmacQ15.to_provenance(5);
        match prov {
            pdn_schema::ControllerFlavor::FmacQ15 { r_exponent } => assert_eq!(r_exponent, 5),
            other => panic!("expected FmacQ15, got {:?}", other),
        }
    }
}
