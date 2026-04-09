use core::{
    f64::consts::PI,
    ops::{Add, Div, Mul, Neg, Sub},
};

#[cfg(feature = "defmt")]
use defmt::println;

use crate::math::{atan, pow2, sqrt, tan};

pub trait Scalar:
    Sized
    + Copy
    + PartialOrd
    + Add<Self, Output = Self>
    + Sub<Self, Output = Self>
    + Mul<Self, Output = Self>
    + Div<Self, Output = Self>
    + Neg<Output = Self>
{
    const ZERO: Self;
    fn from_f32(f: f32) -> Self;
    fn clamp(self, min: Self, max: Self) -> Self;
    fn sqrt(self) -> Self;
}

impl Scalar for f32 {
    const ZERO: Self = 0.0;
    fn from_f32(f: f32) -> Self {
        f as _
    }

    fn clamp(self, min: Self, max: Self) -> Self {
        self.clamp(min, max)
    }

    fn sqrt(self) -> Self {
        micromath::F32Ext::sqrt(self)
    }
}

#[cfg(feature = "std")]
impl Scalar for f64 {
    const ZERO: Self = 0.0;
    fn from_f32(f: f32) -> Self {
        f as _
    }

    fn clamp(self, min: Self, max: Self) -> Self {
        self.clamp(min, max)
    }

    fn sqrt(self) -> Self {
        f64::sqrt(self)
    }
}

macro_rules! impl_scalar {
    ($($t:ident),*) => {$(
        impl Scalar for fixed::types::$t {
            const ZERO: Self = Self::ZERO;
            fn from_f32(f: f32) -> Self {
                Self::from_num(f)
            }

            fn clamp(self, min: Self, max: Self) -> Self {
                Ord::clamp(self, min, max)
            }

            fn sqrt(self) -> Self {
                self.sqrt()
            }
        }
    )*};
}

impl_scalar!(
    I32F0, I31F1, I30F2, I29F3, I28F4, I27F5, I26F6, I25F7, I24F8, I23F9, I22F10, I21F11, I20F12,
    I19F13, I18F14, I17F15, I16F16, I15F17, I14F18, I13F19, I12F20, I11F21, I10F22, I9F23, I8F24,
    I7F25, I6F26, I5F27, I4F28, I3F29, I2F30, I1F31, I0F32
);
impl_scalar!(
    I0F16, I1F15, I2F14, I3F13, I4F12, I5F11, I6F10, I7F9, I8F8, I9F7, I10F6, I11F5, I12F4, I13F3,
    I14F2, I15F1, I16F0
);

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TwoPoleTwoZeroParams<T> {
    pub a1: T,
    pub a2: T,

    pub b0: T,
    pub b1: T,
    pub b2: T,
}

impl<T: Scalar> TwoPoleTwoZeroParams<T> {
    #[inline(always)]
    pub const fn to_controller(self, limit_min: T, limit_max: T) -> TwoPoleTwoZero<T> {
        TwoPoleTwoZero {
            params: self,
            limit_min,
            limit_max,
            outputs: [T::ZERO; _],
            errors: [T::ZERO; _],
        }
    }
}

impl TwoPoleTwoZeroParams<f32> {
    pub fn to_t<T: Scalar>(self) -> TwoPoleTwoZeroParams<T> {
        let Self { a1, a2, b0, b1, b2 } = self;
        TwoPoleTwoZeroParams {
            a1: T::from_f32(a1),
            a2: T::from_f32(a2),
            b0: T::from_f32(b0),
            b1: T::from_f32(b1),
            b2: T::from_f32(b2),
        }
    }

    /// Scale b-coefficients from physical domain (error in Volts) to code domain
    /// (error in ADC codes) by dividing by the feedback divider ratio.
    ///
    /// The a-coefficients are dimensionless and remain unchanged.
    pub const fn to_code_domain(self, divider_ratio: f64) -> Self {
        Self {
            a1: self.a1,
            a2: self.a2,
            b0: (self.b0 as f64 / divider_ratio) as f32,
            b1: (self.b1 as f64 / divider_ratio) as f32,
            b2: (self.b2 as f64 / divider_ratio) as f32,
        }
    }

    /// Minimum FMAC gain exponent R such that every coefficient divided by 2^R
    /// fits in q1.15 (i.e. |coeff| / 2^R < 1.0).
    ///
    /// Load coefficients into FMAC hardware as `coeff / 2^R` in q1.15 format,
    /// then configure the FMAC `PARAM` register with this R value.
    pub const fn min_fmac_r(self) -> u32 {
        // Inline absolute value — closures are not callable in const fn.
        let a = if self.b0 < 0.0 { -self.b0 } else { self.b0 };
        let b = if self.b1 < 0.0 { -self.b1 } else { self.b1 };
        let c = if self.b2 < 0.0 { -self.b2 } else { self.b2 };
        let d = if self.a1 < 0.0 { -self.a1 } else { self.a1 };
        let e = if self.a2 < 0.0 { -self.a2 } else { self.a2 };
        let m1 = if a > b { a } else { b };
        let m2 = if c > d { c } else { d };
        let m3 = if m1 > m2 { m1 } else { m2 };
        let max_abs = if m3 > e { m3 } else { e };
        if max_abs < 1.0 {
            return 0;
        }
        let mut r = 0u32;
        while (1u32 << r) as f32 <= max_abs {
            r += 1;
        }
        r
    }

    /// Convert coefficients to FMAC q1.15 hardware register values for the given R.
    ///
    /// Returns `[b0, b1, b2, a1, a2]` as i16 raw bits, matching the FMAC
    /// coefficient buffer layout. Use `min_fmac_r()` to obtain R.
    pub fn fmac_coeffs(self, r: u32) -> [i16; 5] {
        let scale = (1u32 << r) as f32;
        // Round half-away-from-zero without f32::round() (not available in no_std).
        let q = |x: f32| -> i16 {
            let v = x / scale * 32768.0;
            (if v >= 0.0 { v + 0.5 } else { v - 0.5 }) as i16
        };
        [q(self.b0), q(self.b1), q(self.b2), q(self.a1), q(self.a2)]
    }
}

pub struct TwoPoleTwoZero<T> {
    params: TwoPoleTwoZeroParams<T>,
    /// Outputs will be clamped to limit_min..=limit_max
    limit_min: T,

    /// Outputs will be clamped to limit_min..=limit_max
    limit_max: T,

    /// History of outputs with newest value at index 0
    outputs: [T; 2],

    /// History of errors with newest value at index 0
    errors: [T; 2],
}

impl<T: Scalar> TwoPoleTwoZero<T> {
    #[inline(always)]
    pub fn update(&mut self, error: T) -> T {
        let output = self.params.a1 * self.outputs[0]
            + self.params.a2 * self.outputs[1]
            + self.params.b0 * error
            + self.params.b1 * self.errors[0]
            + self.params.b2 * self.errors[1];
        self.outputs.rotate_right(1);
        let clamped = output.clamp(self.limit_min, self.limit_max);
        self.outputs[0] = clamped;

        self.errors.rotate_right(1);
        self.errors[0] = error;

        clamped
    }

    pub fn reset(&mut self) {
        *self = self.params.to_controller(self.limit_min, self.limit_max);
    }

    /// Pre-load the output and error histories to `u` and `e` respectively.
    ///
    /// Call this on a dormant controller immediately before activating it to
    /// achieve bumpless transfer.  With a type-1 (integrating) 2P2Z where
    /// `a1 + a2 ≈ 1`, the first output after the switch will be:
    ///   `y ≈ u + b0 * (error_new - e)`
    /// which is a small bump proportional only to the *change* in error across
    /// the mode boundary — not a step from zero.
    pub fn prime(&mut self, u: T, e: T) {
        self.outputs = [u, u];
        self.errors = [e, e];
    }

    /// Return a copy of the most-recent output stored in the output history.
    /// Used by the mode supervisor to read the active controller's last command
    /// before priming the incoming controller.
    pub fn last_output(&self) -> T {
        self.outputs[0]
    }

    /// Replace the most-recently stored output with `u`.
    ///
    /// Call this immediately after external clamping to implement clamped-feedback
    /// anti-windup.  Replacing the raw (unclamped) output with the actual saturated
    /// value prevents the embedded integrator from winding beyond the output limits.
    /// After two consecutive clamped cycles the entire output history is bounded,
    /// and the controller exits saturation cleanly as soon as the error reverses.
    pub fn set_last_output(&mut self, u: T) {
        self.outputs[0] = u;
    }

    /// Update the controller and apply an external dynamic output clamp.
    ///
    /// The internal limits (set at construction) apply first via `update()`.
    /// The result is then further clamped to `[ext_min, ext_max]` and the
    /// output history is updated for clamped-feedback anti-windup.
    #[inline(always)]
    pub fn update_clamped(&mut self, error: T, ext_min: T, ext_max: T) -> T {
        let output = self.update(error);
        let clamped = output.clamp(ext_min, ext_max);
        self.set_last_output(clamped);
        clamped
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Topology {
    Buck,
    Boost,
    BuckBoost,
}

#[derive(Clone, Copy)]
pub struct Parameters {
    /// Nominal output voltage (V)
    pub v_out: f64,

    /// Voltage drop of the diode (V)
    pub v_diode: f64,

    /// Capacitance of output capacitor (F)
    pub c_out: f64,

    /// Switching frequency (Hz)
    pub f_sw: f64,

    /// Inductance of inductor (H)
    pub l_inductor: f64,

    /// ESR of output capacitor (Ohm)
    pub r_esr_out_cap: f64,

    /// Current sense gain (V/A)
    pub current_sense_gain: f64,
    /// Safety margin for the crossover frequency selection.
    /// The crossover frequency is chosen automatically as the highest f_x that
    /// keeps `b0 × ΔV_out_ripple ≤ vpp / safety_factor`, preventing limit cycling.
    /// Higher values → lower bandwidth → more stability margin.
    /// Typical: 2.0 (ripple fits in half the DAC linear range).
    pub safety_factor: f64,

    /// Target crossover frequency (Hz) for the voltage loop.
    ///
    /// Sets an upper bound on the compensator bandwidth.  The actual crossover
    /// may be lower if the ripple criterion (`b0 × dV ≤ vpp / safety_factor`)
    /// or the RHP zero constraint (boost/buck-boost) is more restrictive.
    ///
    /// Rule of thumb for peak current mode:
    ///   f_sw / 10  — conservative, works with most plants
    ///   f_sw / 20  — safe default for decimated loops (cycles_per_tick > 1)
    ///   f_sw / 5   — aggressive, requires low-ESR output caps
    pub crossover_hz: f64,

    /// Nominal load (A)
    pub i_load: f64,

    /// TODO: Figure out this
    /// Time taken in seconds from the ADC reading of Vout, the calculation of the control function and to setting the DAC value
    pub phase_margin: PhaseMargin,

    /// Number of switching cycles between consecutive control-loop updates.
    /// 1 = the controller runs every switching cycle (maximum bandwidth).
    /// N > 1 = the MCU skips N-1 cycles between updates; the bilinear-transform
    /// sample period is scaled to T_ctrl = N / f_sw so the continuous-time design
    /// (crossover frequency, phase margin) is preserved at the slower update rate.
    pub cycles_per_tick: usize,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DacSettings {
    /// DAC slope in Volts/second
    pub dac_slope: f64,

    /// Voltage peak to peak in Volts — the full DAC ramp swing per switching cycle.
    /// Used as the denominator in the limit-cycle criterion: `b0 × ΔV_ripple ≤ vpp`.
    vpp: f64,
}

impl DacSettings {
    /// The DAC ramp peak-to-peak voltage (V).  One switching cycle of slope
    /// compensation produces this much voltage swing.  The 2P2Z stays out of
    /// limit-cycling saturation when `b0 × ΔV_out_ripple ≤ vpp`.
    pub const fn vpp(&self) -> f64 {
        self.vpp
    }

    /// Slope ramp accumulated during the on-time, in ADC/DAC codes.
    ///
    /// Add this to `dac_max_code` to get the dynamic controller output limit.
    /// Initialise the controller with a high internal limit (e.g. 4096),
    /// then externally clamp to `dynamic_limit` each cycle and call
    /// `set_last_output(clamped)` for anti-windup.
    ///
    /// `duty` is the steady-state switch duty cycle (0–1).
    /// `lsb` is the ADC/DAC least-significant-bit voltage (V_ref / 2^N).
    pub const fn slope_offset_codes(&self, duty: f64, lsb: f64) -> f64 {
        self.vpp * duty / lsb
    }
}

macro_rules! p {
    ($val:expr, $val2:expr) => {
        #[cfg(false)]
        eprintln!(
            "[{}:{}:{}] {} = {:#?}, {}",
            file!(),
            line!(),
            column!(),
            stringify!($val),
            &$val,
            $val2
        );
    };
}

impl Parameters {
    /// Returns the highest crossover frequency (Hz) satisfying all constraints:
    ///
    /// 1. **Phase feasibility**: `to_2p2z()` returns `Some` (φ_v < π/2)
    /// 2. **Ripple / limit-cycling**: `b0 × ΔV_out_ripple ≤ vpp / safety_factor`
    /// 3. **RHP zero** (boost/buck-boost): `f_x ≤ f_RHP / (2 × safety_factor)`
    ///
    /// Call this to discover the limit before setting `crossover_hz`.
    pub const fn max_feasible_crossover_hz(&self, v_in: f64, topology: Topology) -> f64 {
        // Steady-state duty cycle and inductor voltage during the on-phase.
        let d = match topology {
            Topology::Buck => (self.v_out + self.v_diode) / v_in,
            Topology::Boost => 1.0 - v_in / (self.v_out - self.v_diode),
            Topology::BuckBoost => self.v_out / (v_in + self.v_out - self.v_diode),
        };
        let v_l_on = match topology {
            Topology::Buck => v_in - self.v_out - self.v_diode,
            Topology::Boost | Topology::BuckBoost => v_in,
        };

        // Estimated peak-to-peak output voltage ripple (capacitive + ESR components).
        let di_l = v_l_on * d / (self.f_sw * self.l_inductor);
        let dv_out = match topology {
            Topology::Buck => di_l * (self.r_esr_out_cap + 1.0 / (8.0 * self.f_sw * self.c_out)),
            Topology::Boost | Topology::BuckBoost => {
                self.i_load * d / (self.f_sw * self.c_out)
                    + di_l * self.r_esr_out_cap
            }
        };

        // vpp is independent of crossover — probe at a reference frequency.
        let (_, dac) = self.to_transfer_function_inner(self.f_sw / 8.0, v_in, topology);
        let vpp = dac.vpp;
        let b0_max = vpp / (dv_out * self.safety_factor);

        // Binary search for the highest feasible crossover frequency.
        // Upper bound: f_sw / 8 (bilinear transform breaks down near Nyquist).
        // Lower bound: 1 Hz.
        // b0 increases monotonically with crossover frequency in the valid region.
        let mut lo = 1.0_f64;
        let mut hi = self.f_sw / 8.0;
        let mut i = 0;
        while i < 60 {
            let mid = (lo + hi) / 2.0;
            let (tf, _) = self.to_transfer_function_inner(mid, v_in, topology);
            let b0 = match tf.to_2p2z() {
                Some(w) => w.b0 as f64,
                None => f64::MAX, // phase infeasible — too high
            };
            if b0 <= b0_max {
                lo = mid; // criterion met — try higher frequency
            } else {
                hi = mid; // criterion violated — reduce frequency
            }
            i += 1;
        }

        // Additional constraint for Boost and BuckBoost: RHP zero.
        let rhp_limit = match topology {
            Topology::Buck => f64::MAX,
            Topology::Boost | Topology::BuckBoost => {
                let d_prime = 1.0 - d;
                let r_load = self.v_out / self.i_load;
                let omega_rhp = d_prime * d_prime * r_load / self.l_inductor;
                omega_rhp / (4.0 * PI * self.safety_factor)
            }
        };

        // Return the most conservative limit.
        if lo < rhp_limit { lo } else { rhp_limit }
    }

    /// Design the 2P2Z compensator for these circuit parameters.
    ///
    /// Uses `crossover_hz` as the exact target crossover frequency.
    /// If the design is infeasible (phase or ripple), `to_2p2z()` will return `None`
    /// or the caller should check the ripple criterion separately.
    /// Use `max_feasible_crossover_hz()` to find the highest valid crossover.
    pub const fn to_transfer_function(
        self,
        v_in: f64,
        topology: Topology,
    ) -> (TransferFunction, DacSettings) {
        self.to_transfer_function_inner(self.crossover_hz, v_in, topology)
    }

    /// Compute DAC slope-compensation settings at the design-point `v_out`.
    ///
    /// Equivalent to `self.dac_settings_at(v_in, self.v_out, topology, 1.0)`.
    /// See [`dac_settings_at`](Self::dac_settings_at) for details.
    pub const fn dac_settings(self, v_in: f64, topology: Topology) -> DacSettings {
        self.dac_settings_at(v_in, self.v_out, topology, 1.0)
    }

    /// Compute DAC slope-compensation settings for arbitrary Vin/Vout.
    ///
    /// Unlike [`to_transfer_function`], this function is cheap — suitable for
    /// calling from a runtime control task after measuring the actual bus
    /// voltage (Vin feed-forward) or during soft-start when the target
    /// voltage is ramping.
    ///
    /// `overcomp` is a multiplicative safety margin applied to the slope
    /// magnitude (`S_e`).  Use `1.0` for the theoretical value, `1.5` for
    /// the 50 % over-compensation used in the firmware.
    ///
    /// # Runtime slope update pattern
    ///
    /// ```ignore
    /// // In a 1 ms task, after measuring v_in and knowing the current target:
    /// let s = CTRL_PARAMS.dac_settings_at(v_in, v_target, Topology::Buck, 1.5);
    /// // INCDATA = ceil(|dac_slope| / LSB  ×  16 / F_HR  ×  HR_TICKS_PER_DAC_INC)
    /// let step = ((-s.dac_slope / LSB) * 16.0 / F_HR * CR2).ceil().max(1.0) as u16;
    /// DAC_STEP_LIVE.store(step, Ordering::Relaxed);
    /// // slope_offset for controller dynamic limit:
    /// let offset = s.slope_offset_codes(v_target / v_in, LSB) as u16;
    /// ```
    pub const fn dac_settings_at(
        self,
        v_in: f64,
        v_out: f64,
        topology: Topology,
        overcomp: f64,
    ) -> DacSettings {
        let t_sw = 1.0 / self.f_sw;
        let (steady_state_duty, inductor_current_up_slope) = match topology {
            Topology::Buck => (
                (v_out + self.v_diode) / v_in,
                (v_in - v_out - self.v_diode) * self.current_sense_gain / self.l_inductor,
            ),
            Topology::Boost => (
                1.0 - v_in / (v_out - self.v_diode),
                v_in * self.current_sense_gain / self.l_inductor,
            ),
            Topology::BuckBoost => (
                v_out / (v_in + v_out - self.v_diode),
                v_in * self.current_sense_gain / self.l_inductor,
            ),
        };
        let inv_steady_state_duty = 1.0 - steady_state_duty;
        let slope_compensation_factor = (1.0 + PI / 2.0) / (PI * inv_steady_state_duty);
        let dac_down_slope =
            -(slope_compensation_factor - 1.0) * inductor_current_up_slope * overcomp;
        let vpp = -dac_down_slope * t_sw;
        DacSettings {
            dac_slope: dac_down_slope,
            vpp,
        }
    }

    /// Compute the transfer function for a given crossover frequency (Hz).
    const fn to_transfer_function_inner(
        self,
        crossover_hz: f64,
        v_in: f64,
        topology: Topology,
    ) -> (TransferFunction, DacSettings) {
        //
        // https://centaur.reading.ac.uk/31751/1/Microcontroller%20Based%20Peak%20Current%20Mode%20Control%20Using%20Digital%20Slope%20Compensation%20-%20Hallworth%202012.pdf
        //

        // https://www.biricha.com/articles/step-by-step-design-guide-for-digital-peak-current-mode-control-a-single-chip-solution
        // https://www.st.com/en/embedded-software/x-cube-dpower.html
        // https://www.ti.com/lit/an/sprabe7a/sprabe7a.pdf?ts=1723931480534
        // https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/171/Presentation_5F002D005F00_Mr._5F00_Ali_5F00_Shirsavar.pdf
        // https://www.st.com/resource/en/application_note/an5497-introduction-to-the-buck-current-mode-with-the-bg474edpow1-discovery-kit-stmicroelectronics.pdf

        use core::f64::consts::PI;

        let Parameters {
            v_out,
            c_out,
            v_diode: diode_drop,
            f_sw,
            l_inductor,
            r_esr_out_cap,
            current_sense_gain,
            i_load,
            phase_margin,
            safety_factor: _,
            crossover_hz: _,
            cycles_per_tick,
        } = self;

        p!(v_in, "16");
        p!(v_out, "8");
        p!(i_load, "2");
        p!(c_out, "440e-6");
        p!(l_inductor, "22e-6");
        p!(current_sense_gain, "0.48");
        p!(r_esr_out_cap, "31e-3");

        p!(f_sw, "200e3");

        let t_sw = 1.0 / f_sw;
        let r_load = v_out / i_load; // ohm

        // Topology-dependent steady-state duty cycle and inductor up-slope (S_n).
        // The diode is in series with the output during the off-phase for all topologies.
        //   Buck:      D  = (V_out + V_d) / V_in,       V_L_on = V_in - V_out - V_d
        //   Boost:     D  = 1 - V_in / (V_out - V_d),   V_L_on = V_in
        //   BuckBoost: D  = V_out / (V_in + V_out - V_d), V_L_on = V_in
        let (steady_state_duty, inductor_current_up_slope) = match topology {
            Topology::Buck => (
                (v_out + diode_drop) / v_in,
                (v_in - v_out - diode_drop) * current_sense_gain / l_inductor,
            ),
            Topology::Boost => (
                1.0 - v_in / (v_out - diode_drop),
                v_in * current_sense_gain / l_inductor,
            ),
            Topology::BuckBoost => (
                v_out / (v_in + v_out - diode_drop),
                v_in * current_sense_gain / l_inductor,
            ),
        };
        let inv_steady_state_duty = 1.0 - steady_state_duty; // D'

        p!(steady_state_duty, "0.5375");

        // m_c
        let slope_compensation_factor = (1.0 + PI / 2.0) / (PI * inv_steady_state_duty);

        // S_e
        let dac_down_slope = -(slope_compensation_factor - 1.0) * inductor_current_up_slope; // volts/second

        let vpp = -dac_down_slope * t_sw; // V_PP is positive (paper eq. 6: V_PP = S_E * T_S)

        //
        let q_inv_no_pi = slope_compensation_factor * inv_steady_state_duty - 0.5;

        // For boost and buck-boost the inductor current only transfers to the output during D',
        // and the effective load impedance seen by the current loop is D'^2 * R_load (from the
        // 1:D' effective transformer in the averaged model).  For buck D' = 1 in that sense.
        let (h_dc, ohmega_p1) = match topology {
            Topology::Buck => (
                r_load / current_sense_gain / (1.0 + q_inv_no_pi * r_load * t_sw / l_inductor),
                1.0 / (r_load * c_out) + q_inv_no_pi * t_sw / (l_inductor * c_out),
            ),
            Topology::Boost | Topology::BuckBoost => {
                let d_prime_sq = inv_steady_state_duty * inv_steady_state_duty;
                (
                    d_prime_sq * r_load
                        / current_sense_gain
                        / (1.0 + q_inv_no_pi * d_prime_sq * r_load * t_sw / l_inductor),
                    d_prime_sq / (r_load * c_out) + q_inv_no_pi * t_sw / (l_inductor * c_out),
                )
            }
        };
        let ohmega_esr = 1.0 / (c_out * r_esr_out_cap);
        p!(ohmega_p1, "732.6");
        p!(ohmega_esr, "aka ωCP1 (and ωZ1 ?) 73 310");
        // let h_ctrl_to_output = |s| h_h(s) * h_p(s) * h_dc;

        //------------------------
        p!(h_dc, "6.4631");
        (
            TransferFunction {
                f_sw,
                crossover_hz,
                phase_margin,
                cycles_per_tick,

                ohmega_p1,
                ohmega_esr,
                h_dc,
            },
            DacSettings {
                dac_slope: dac_down_slope,
                vpp,
            },
        )
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PhaseMargin {
    Manual {
        phase_margin: f64,
    },
    Calculated {
        t_adc: f64,
        t_processing: f64,
        t_dac: f64,
    },
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TransferFunction {
    f_sw: f64,
    crossover_hz: f64,
    phase_margin: PhaseMargin,
    cycles_per_tick: usize,

    ohmega_p1: f64,
    ohmega_esr: f64,
    h_dc: f64,
}

/// All continuous-time design parameters needed to plot Bode diagrams.
#[derive(Debug, Clone, Copy)]
pub struct DesignSummary {
    /// Plant pole [rad/s]
    pub omega_p1: f64,
    /// ESR zero [rad/s]
    pub omega_esr: f64,
    /// DC gain
    pub h_dc: f64,
    /// Inner current-loop natural freq [rad/s] = π·f_sw
    pub omega_n: f64,
    /// Integrator gain [rad/s]
    pub omega_cp0: f64,
    /// Compensator pole [rad/s] (= omega_esr)
    pub omega_cp1: f64,
    /// Compensator zero [rad/s]
    pub omega_cz1: f64,
    /// Crossover frequency [rad/s]
    pub omega_x: f64,
    /// Effective phase margin [rad]
    pub phase_margin_rad: f64,
    /// Switching frequency [Hz]
    pub f_sw: f64,
}

impl TransferFunction {
    /// Return all continuous-time design parameters for Bode plot rendering.
    ///
    /// Re-derives compensator poles/zeros from stored plant parameters using the
    /// same formulas as `to_2p2z()`, stopping at the continuous-time stage
    /// (before bilinear transform to discrete coefficients).
    pub const fn design_summary(&self) -> DesignSummary {
        let ohmega_n = self.ohmega_n();
        let f_x = self.crossover_hz;
        let ohmega_x = 2.0 * PI * f_x;

        let phase_margin = match self.phase_margin {
            PhaseMargin::Manual { phase_margin } => phase_margin,
            PhaseMargin::Calculated {
                t_adc,
                t_processing,
                t_dac,
            } => {
                let t_hold = (self.cycles_per_tick - 1) as f64 / self.f_sw;
                let phase_erosion = 2.0 * PI * f_x * (t_adc + t_processing + t_dac + t_hold);
                50.0_f64.to_radians() + phase_erosion
            }
        };

        let r = ohmega_x / ohmega_n;
        let complex_pole_pair = atan(r / (1.0 - pow2(r)));
        let phi_v = -0.5 * PI + phase_margin + atan(ohmega_x / self.ohmega_p1) + complex_pole_pair;

        let ohmega_cp1 = self.ohmega_esr;
        let ohmega_cz1 = ohmega_x / tan(phi_v);

        let k1 = sqrt(
            (1.0 + pow2(ohmega_x / ohmega_cz1)) / (1.0 + pow2(ohmega_x / self.ohmega_p1)),
        );
        let k2 = sqrt(1.0
            / (pow2(1.0 - pow2(ohmega_x / ohmega_n)) + pow2(ohmega_x / ohmega_n)));
        let ohmega_cp0 = ohmega_x / (self.h_dc * k1 * k2);

        DesignSummary {
            omega_p1: self.ohmega_p1,
            omega_esr: self.ohmega_esr,
            h_dc: self.h_dc,
            omega_n: ohmega_n,
            omega_cp0: ohmega_cp0,
            omega_cp1: ohmega_cp1,
            omega_cz1: ohmega_cz1,
            omega_x: ohmega_x,
            phase_margin_rad: phase_margin,
            f_sw: self.f_sw,
        }
    }

    /// Convert to discrete 2P2Z coefficients via bilinear (Tustin) transform.
    ///
    /// Returns `None` when the compensator design is infeasible — specifically
    /// when the required compensator zero phase angle φ_v ≥ 90°.  This happens
    /// when total transport delay (ADC + processing + DAC + ZOH hold) erodes
    /// too much phase at the chosen crossover frequency.
    ///
    /// In firmware `const` contexts, use `.unwrap()` or `.expect("…")` to get
    /// a compile-time error when parameters are infeasible.
    pub const fn to_2p2z(self) -> Option<TwoPoleTwoZeroParams<f32>> {
        let TransferFunction {
            f_sw,
            crossover_hz,
            phase_margin,
            cycles_per_tick,
            ohmega_p1,
            ohmega_esr,
            h_dc,
        } = self;

        let ohmega_n = self.ohmega_n();
        p!(ohmega_n, "628 300");

        let f_x = crossover_hz;

        //println!("----------------------------------");
        //println!("----------------------------------");
        //println!("----------------------------------");

        // Crossover frequency as rad/s
        let ohmega_x = 2.0 * PI * f_x;
        p!(ohmega_x, "?");

        let phase_margin = match phase_margin {
            PhaseMargin::Manual { phase_margin } => phase_margin,
            PhaseMargin::Calculated {
                t_adc,
                t_processing,
                t_dac,
            } => {
                // Include (cycles_per_tick - 1) extra switching-cycle delays that arise
                // because the MCU holds its output constant between control updates.
                let t_hold = (cycles_per_tick - 1) as f64 / f_sw;
                let phase_erosion = 2.0 * PI * f_x * (t_adc + t_processing + t_dac + t_hold);
                // TODO: Is 50 enough?
                50.0f64.to_radians() + phase_erosion
            }
        };

        //#[cfg(not(feature = "hardware"))]
        //assert!(phase_erosion < 90.0f64.to_radians());

        // ChatGPT's suggestion
        let r = ohmega_x / ohmega_n;
        let complex_pole_pair = atan(r / (1.0 - pow2(r)));
        //dbg!(complex_pole_pair);

        // p1 ok
        let phi_v = -0.5 * PI + phase_margin + atan(ohmega_x / ohmega_p1) + complex_pole_pair;
        //dbg!(ohmega_x);

        // Guard: at φ_v ≥ π/2, tan(φ_v) ≤ 0 and ω_cz1 flips sign, producing
        // non-physical coefficients.  This means the transport delays erode too
        // much phase at the chosen crossover frequency — reduce f_x, lower
        // cycles_per_tick, or shorten ADC/processing/DAC delays.
        if phi_v >= 0.5 * PI {
            return None;
        }

        let ohmega_cp1 = ohmega_esr; // eq. 8
        let ohmega_cz1 = ohmega_x / tan(phi_v); // eq. 10
        p!(ohmega_cp1, "73_310");
        p!(ohmega_cz1, "11_110");

        let k1 = sqrt((1.0 + pow2(ohmega_x / ohmega_cz1)) / (1.0 + pow2(ohmega_x / ohmega_p1))); // eq. 18
        let k2 = sqrt(1.0 / (pow2(1.0 - pow2(ohmega_x / ohmega_n)) + pow2(ohmega_x / ohmega_n))); // eq. 19 (Q_C=1)

        // pole at origin, eq. 16
        let ohmega_cp0 = ohmega_x / (h_dc * k1 * k2);
        p!(ohmega_cp0, "217_100");

        // Compensator transfer function in the analog domain
        // let h_c = |s: Complex| ohmega_cp0 / s * (1.0 + s / ohmega_cz1) / (1.0 + s / ohmega_cp1);

        // ----------

        // Bilinear (Tustin) transform using the control sample period T_ctrl = N / f_sw.
        // When cycles_per_tick > 1 the controller updates less frequently than the
        // switching cycle; T_ctrl > T_sw scales all coefficients so the continuous-time
        // crossover frequency and phase margin are preserved at the slower sample rate.
        // Note: ohmega_n stays at π·f_sw because it models the inner current-mode ZOH
        // which still runs every switching cycle regardless of the outer-loop decimation.
        let t_ctrl = cycles_per_tick as f64 / f_sw;
        let b0 = t_ctrl * ohmega_cp0 * ohmega_cp1 * (2.0 + t_ctrl * ohmega_cz1)
            / (2.0 * (2.0 + t_ctrl * ohmega_cp1) * ohmega_cz1);

        let b1 = pow2(t_ctrl) * ohmega_cp0 * ohmega_cp1 / (2.0 + t_ctrl * ohmega_cp1);

        let b2 = t_ctrl * ohmega_cp0 * ohmega_cp1 * (-2.0 + t_ctrl * ohmega_cz1)
            / (2.0 * (2.0 + t_ctrl * ohmega_cp1) * ohmega_cz1);

        let a1 = 4.0 / (2.0 + t_ctrl * ohmega_cp1);
        let a2 = (-2.0 + t_ctrl * ohmega_cp1) / (2.0 + t_ctrl * ohmega_cp1);

        Some(TwoPoleTwoZeroParams {
            a1: a1 as _,
            a2: a2 as _,
            b0: b0 as _,
            b1: b1 as _,
            b2: b2 as _,
        })
    }

    pub const fn ohmega_n(&self) -> f64 {
        PI * self.f_sw // F_SW in Hz, or PI * F_SW with F_SW in rad/s // TODO
    }

    #[cfg(any(feature = "std", feature = "defmt"))]
    pub fn print_h_p_transfer_func(&self) {
        let ohmega_esr = self.ohmega_esr;
        let ohmega_p1 = self.ohmega_p1;

        //let h_p = |s: Complex| (1.0 + s / ohmega_esr) / (1.0 + s / ohmega_p1);

        println!("(1.0 + s / {}) / (1.0 + s / {})", ohmega_esr, ohmega_p1);
        println!(
            "(1.0 + s / {:.2}) / (1.0 + s / {:.2})",
            ohmega_esr, ohmega_p1
        );
    }

    #[cfg(any(feature = "std", feature = "defmt"))]
    pub fn print_high_freq_transfer_func(&self) {
        let ohmega_n = self.ohmega_n();
        let q_inv = 1.0_f64; // Q_c = 1 assumption
        // High frequency transfer function
        // let h_h = |s: Complex| 1.0 / (s * s / (ohmega_n * ohmega_n) + s * q_inv / ohmega_n + 1.0);

        println!(
            "1.0 / (s * s / {}^2 + s * {} / {} + 1.0)",
            ohmega_n, q_inv, ohmega_n
        );

        println!(
            "1.0 / (s * s / {:.2}^2 + s * {:.2} / {:.2} + 1.0)",
            ohmega_n, q_inv, ohmega_n
        );
    }

    #[cfg(any(feature = "std", feature = "defmt"))]
    pub fn print_dc_gain(&self) {
        println!("{}", self.h_dc);
        println!("{:.2}", self.h_dc);
    }
}

// ── Integration tests against Hallworth 2012 ─────────────────────────────────
//
// Reference: M. Hallworth, S.A. Shirsavar, "Microcontroller-based peak current
// mode control using digital slope compensation", IEEE Trans. Power Electron.,
// vol. 27, no. 7, pp. 3340–3351, Jul. 2012.
// DOI: 10.1109/TPEL.2011.2182210
//
// The design example in Section V (Tables I–III) specifies a 16 W buck converter
// and derives the complete compensator analytically.  These tests verify that
// our code reproduces the paper's published numerical results.
#[cfg(test)]
mod hallworth_2012_tests {
    use super::*;
    use core::f64::consts::PI;

    /// Helper: assert `actual` is within `tol_pct` percent of `expected`.
    fn assert_close(actual: f64, expected: f64, tol_pct: f64, label: &str) {
        let rel_err = ((actual - expected) / expected).abs() * 100.0;
        assert!(
            rel_err < tol_pct,
            "{label}: expected {expected}, got {actual} (rel err {rel_err:.4}% > {tol_pct}%)",
        );
    }

    // ── Paper Table Ia — Specification ───────────────────────────────────────
    const V_IN: f64 = 16.0;       // V
    const V_OUT: f64 = 8.0;       // V
    const I_OUT: f64 = 2.0;       // A
    const C_OUT: f64 = 440e-6;    // F
    const L_OUT: f64 = 22e-6;     // H
    const R_I: f64 = 0.48;        // V/A  (current sense gain)
    const R_ESR: f64 = 31e-3;     // Ω
    const V_DIODE: f64 = 0.6;     // V
    const F_SW: f64 = 200e3;      // Hz
    const F_X: f64 = 15e3;        // Hz   (crossover frequency)
    const PHI_M_DEG: f64 = 75.0;  // °    (phase margin)

    fn hallworth_params() -> Parameters {
        Parameters {
            v_out: V_OUT,
            v_diode: V_DIODE,
            c_out: C_OUT,
            f_sw: F_SW,
            l_inductor: L_OUT,
            r_esr_out_cap: R_ESR,
            current_sense_gain: R_I,
            i_load: I_OUT,
            phase_margin: PhaseMargin::Manual {
                phase_margin: PHI_M_DEG.to_radians(),
            },
            safety_factor: 2.0,
            crossover_hz: F_X,
            cycles_per_tick: 1,
        }
    }

    // ── Table Ib — Operational parameters ────────────────────────────────────

    /// Verify duty cycle D = (V_O + V_DIODE) / V_IN = 0.5375 (Table Ib)
    #[test]
    fn duty_cycle() {
        let d = (V_OUT + V_DIODE) / V_IN;
        assert_close(d, 0.5375, 0.01, "D");
    }

    /// Verify slope compensation factor m_C = 1.7693 (Table Ib, eq. 2 with Q_C=1)
    #[test]
    fn slope_compensation_factor() {
        let d = (V_OUT + V_DIODE) / V_IN;
        let m_c = (1.0 + PI / 2.0) / (PI * (1.0 - d));
        assert_close(m_c, 1.7693, 0.01, "m_C");
    }

    /// Verify plant pole ω_P1 = 732.6 rad/s (Table Ib)
    #[test]
    fn plant_pole_omega_p1() {
        let p = hallworth_params();
        let (tf, _) = p.to_transfer_function(V_IN, Topology::Buck);
        let ds = tf.design_summary();
        assert_close(ds.omega_p1, 732.6, 0.05, "ω_P1");
    }

    /// Verify ESR zero ω_Z1 = ω_ESR = 7.331e4 rad/s (Table Ib)
    #[test]
    fn esr_zero_omega_z1() {
        let p = hallworth_params();
        let (tf, _) = p.to_transfer_function(V_IN, Topology::Buck);
        let ds = tf.design_summary();
        // ω_ESR = 1/(R_ESR × C_out)
        let omega_esr_expected = 1.0 / (R_ESR * C_OUT);
        assert_close(ds.omega_esr, omega_esr_expected, 0.01, "ω_ESR exact");
        assert_close(ds.omega_esr, 7.331e4, 0.05, "ω_ESR paper");
    }

    /// Verify subharmonic double-pole frequency ω_N = 6.283e5 rad/s (Table Ib)
    #[test]
    fn subharmonic_pole_omega_n() {
        let p = hallworth_params();
        let (tf, _) = p.to_transfer_function(V_IN, Topology::Buck);
        let ds = tf.design_summary();
        assert_close(ds.omega_n, PI * F_SW, 0.01, "ω_N exact");
        assert_close(ds.omega_n, 6.283e5, 0.05, "ω_N paper");
    }

    /// Verify DC gain K_DC = 6.4631 (Table Ib, eq. 17)
    #[test]
    fn dc_gain() {
        let p = hallworth_params();
        let (tf, _) = p.to_transfer_function(V_IN, Topology::Buck);
        let ds = tf.design_summary();
        assert_close(ds.h_dc, 6.4631, 0.05, "K_DC");
    }

    /// Verify slope compensation ramp V_PP = 0.621 V (Table Ib, eq. 6)
    #[test]
    fn slope_ramp_vpp() {
        let p = hallworth_params();
        let (_, dac) = p.to_transfer_function(V_IN, Topology::Buck);
        assert_close(dac.vpp(), 0.621, 0.5, "V_PP");
    }

    // ── Table Ic — Compensator poles and zeros ───────────────────────────────

    /// Verify compensator pole ω_CP1 = 7.331e4 rad/s (Table Ic, eq. 8)
    /// The compensator pole is placed at the ESR zero frequency.
    #[test]
    fn compensator_pole_omega_cp1() {
        let p = hallworth_params();
        let (tf, _) = p.to_transfer_function(V_IN, Topology::Buck);
        let ds = tf.design_summary();
        assert_close(ds.omega_cp1, 7.331e4, 0.05, "ω_CP1");
        // ω_CP1 must equal ω_ESR (eq. 8)
        assert!(
            (ds.omega_cp1 - ds.omega_esr).abs() < 1e-6,
            "ω_CP1 must equal ω_ESR"
        );
    }

    /// Verify compensator zero ω_CZ1 = 1.111e4 rad/s (Table Ic, eq. 10)
    #[test]
    fn compensator_zero_omega_cz1() {
        let p = hallworth_params();
        let (tf, _) = p.to_transfer_function(V_IN, Topology::Buck);
        let ds = tf.design_summary();
        assert_close(ds.omega_cz1, 1.111e4, 0.5, "ω_CZ1");
    }

    /// Verify integrator gain ω_CP0 = 2.171e5 rad/s (Table Ic, eq. 16)
    #[test]
    fn integrator_gain_omega_cp0() {
        let p = hallworth_params();
        let (tf, _) = p.to_transfer_function(V_IN, Topology::Buck);
        let ds = tf.design_summary();
        assert_close(ds.omega_cp0, 2.171e5, 0.5, "ω_CP0");
    }

    // ── Table II — Digital controller coefficients (bilinear transform) ──────

    /// Verify all five 2P2Z coefficients match Table II of the paper.
    ///
    /// B0 =  3.112327,  B1 =  0.168173,  B2 = -2.944154
    /// A1 =  1.690211,  A2 = -0.690211
    #[test]
    fn bilinear_coefficients() {
        let p = hallworth_params();
        let (tf, _) = p.to_transfer_function(V_IN, Topology::Buck);
        let w = tf.to_2p2z().expect("compensator design must be feasible");

        // The paper tabulates 6-7 significant figures.  Use 0.5% tolerance
        // to allow for minor differences in intermediate rounding.
        let tol = 0.5; // percent

        assert_close(w.b0 as f64, 3.112327, tol, "B0");
        assert_close(w.b1 as f64, 0.168173, tol, "B1");
        assert_close(w.b2 as f64, -2.944154, tol, "B2");
        assert_close(w.a1 as f64, 1.690211, tol, "A1");
        assert_close(w.a2 as f64, -0.690211, tol, "A2");
    }

    /// Verify analytical coefficient formulas (eqs. 23–27) directly.
    ///
    /// Uses the paper's exact compensator pole/zero values and confirms
    /// the bilinear transform produces the tabulated coefficients.
    #[test]
    fn bilinear_formulas_direct() {
        // Paper's compensator values (Table Ic)
        let omega_cp0 = 2.171e5;
        let omega_cp1 = 7.331e4;
        let omega_cz1 = 1.111e4;
        let t_s = 1.0 / F_SW;

        // Eq. 23
        let b0 = t_s * omega_cp0 * omega_cp1 * (2.0 + t_s * omega_cz1)
            / (2.0 * (2.0 + t_s * omega_cp1) * omega_cz1);
        // Eq. 24
        let b1 = t_s * t_s * omega_cp0 * omega_cp1 / (2.0 + t_s * omega_cp1);
        // Eq. 25
        let b2 = t_s * omega_cp0 * omega_cp1 * (-2.0 + t_s * omega_cz1)
            / (2.0 * (2.0 + t_s * omega_cp1) * omega_cz1);
        // Eq. 26
        let a1 = 4.0 / (2.0 + t_s * omega_cp1);
        // Eq. 27
        let a2 = (-2.0 + t_s * omega_cp1) / (2.0 + t_s * omega_cp1);

        // These are computed from the paper's rounded intermediate values,
        // so use 1% tolerance against the paper's final tabulated results.
        let tol = 1.0;
        assert_close(b0, 3.112327, tol, "B0 formula");
        assert_close(b1, 0.168173, tol, "B1 formula");
        assert_close(b2, -2.944154, tol, "B2 formula");
        assert_close(a1, 1.690211, tol, "A1 formula");
        assert_close(a2, -0.690211, tol, "A2 formula");
    }

    /// Verify that the compensator has unity gain at crossover: |H_P × H_C| = 1.
    ///
    /// This is the fundamental design requirement (eq. 15).
    #[test]
    fn unity_gain_at_crossover() {
        let p = hallworth_params();
        let (tf, _) = p.to_transfer_function(V_IN, Topology::Buck);
        let ds = tf.design_summary();

        let omega_x = ds.omega_x;

        // Plant magnitude at crossover: K_DC × |H_p| × |H_h|
        // H_p(jω) = (1 + jω/ω_esr) / (1 + jω/ω_p1)
        let hp_num = (1.0 + (omega_x / ds.omega_esr).powi(2)).sqrt();
        let hp_den = (1.0 + (omega_x / ds.omega_p1).powi(2)).sqrt();
        let hp_mag = hp_num / hp_den;

        // H_h(jω) = 1 / |1 - (ω/ω_n)² + j·ω/ω_n|   (Q_C = 1)
        let r = omega_x / ds.omega_n;
        let hh_mag = 1.0 / ((1.0 - r * r).powi(2) + r * r).sqrt();

        let plant_mag = ds.h_dc * hp_mag * hh_mag;

        // Compensator magnitude at crossover: ω_cp0/ω × |1 + jω/ω_cz1| / |1 + jω/ω_cp1|
        let hc_int = ds.omega_cp0 / omega_x;
        let hc_num = (1.0 + (omega_x / ds.omega_cz1).powi(2)).sqrt();
        let hc_den = (1.0 + (omega_x / ds.omega_cp1).powi(2)).sqrt();
        let comp_mag = hc_int * hc_num / hc_den;

        let open_loop_mag = plant_mag * comp_mag;
        assert_close(open_loop_mag, 1.0, 1.0, "|T(jω_x)| = 1");
    }

    /// Verify the integrating property: a1 + a2 ≈ 1.
    ///
    /// The 2P2Z has a pole at z=1 (integrator), so the denominator
    /// is (1 − z⁻¹)(1 − p·z⁻¹) and the sum of feedback coefficients = 1.
    #[test]
    fn integrating_property() {
        let p = hallworth_params();
        let (tf, _) = p.to_transfer_function(V_IN, Topology::Buck);
        let w = tf.to_2p2z().unwrap();
        let sum = w.a1 as f64 + w.a2 as f64;
        assert_close(sum, 1.0, 0.01, "a1 + a2");
    }

    // ── Table III — Digital slope compensation ───────────────────────────────

    /// Verify digital slope compensation parameters (Table III).
    ///
    /// V_DAC = 3.3V, n_DAC = 10 bits → Ramp = V_PP × (2^10 - 1) / 3.3 = 192.53
    /// T_STEP = 50ns, T_SLOPE = 3950ns → Steps = 79, ΔRamp = -2.437
    #[test]
    fn digital_slope_compensation() {
        let p = hallworth_params();
        let (_, dac) = p.to_transfer_function(V_IN, Topology::Buck);

        let v_dac = 3.3;
        let n_dac = 10;

        // Eq. 31: Ramp = V_PP × (2^n_DAC - 1) / V_DAC
        let ramp = dac.vpp() * ((1u32 << n_dac) - 1) as f64 / v_dac;
        assert_close(ramp, 192.53, 0.5, "Ramp");

        // Eq. 32: Steps = T_SLOPE / T_STEP
        let t_step: f64 = 50e-9; // 50 ns
        let t_slope: f64 = 3950e-9; // 3950 ns (= T_S - T_START, where T_START ≈ 400 ns + blanking)
        let steps = (t_slope / t_step).round();
        assert_close(steps, 79.0, 0.01, "Steps");

        // Eq. 33: ΔRamp = -Ramp / Steps
        let delta_ramp = -ramp / steps;
        assert_close(delta_ramp, -2.437, 0.5, "ΔRamp");
    }

    /// Verify that the dac_slope in V/s matches S_E from the paper.
    ///
    /// S_N = (V_IN - V_OUT - V_DIODE) × R_I / L_O   (eq. 4, current-sense domain)
    /// S_E = (m_C - 1) × S_N                          (eq. 5)
    /// dac_slope = -S_E
    #[test]
    fn dac_slope_matches_se() {
        let p = hallworth_params();
        let (_, dac) = p.to_transfer_function(V_IN, Topology::Buck);

        let s_n = (V_IN - V_OUT - V_DIODE) * R_I / L_OUT;
        let d = (V_OUT + V_DIODE) / V_IN;
        let m_c = (1.0 + PI / 2.0) / (PI * (1.0 - d));
        let s_e = (m_c - 1.0) * s_n;

        assert_close(-dac.dac_slope, s_e, 0.1, "S_E = -dac_slope");
        // Confirm V_PP = S_E × T_S
        assert_close(dac.vpp(), s_e / F_SW, 0.1, "V_PP = S_E × T_S");
    }
}
