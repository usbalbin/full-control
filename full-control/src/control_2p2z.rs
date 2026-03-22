use core::{
    f64::consts::PI,
    ops::{Add, Mul},
};

#[cfg(feature = "defmt")]
use defmt::println;

use crate::math::{atan, pow2, sqrt, tan};

pub trait Scalar:
    Sized + Copy + PartialOrd + Add<Self, Output = Self> + Mul<Self, Output = Self>
{
    const ZERO: Self;
    fn from_f32(f: f32) -> Self;

    fn clamp(self, min: Self, max: Self) -> Self;
}

impl Scalar for f32 {
    const ZERO: Self = 0.0;
    fn from_f32(f: f32) -> Self {
        f as _
    }

    fn clamp(self, min: Self, max: Self) -> Self {
        self.clamp(min, max)
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
    /// Returns the crossover frequency as `f_sw / divisor`.
    ///
    /// This is the smallest divisor in the valid range (f_x ≤ f_sw/8) for which
    /// `b0 × ΔV_out_ripple ≤ vpp / self.safety_factor`.  Lower divisor = higher
    /// bandwidth = better transient response, at the cost of less ripple margin.
    ///
    /// Useful for informational display: call `params.crossover_divisor()` to
    /// inspect the selected f_x without re-running the full compensator design.
    pub const fn crossover_divisor(self, topology: Topology, v_in: f64) -> f64 {
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
            // Buck: triangular ripple current flows through cap the whole cycle.
            //   ΔV_cap = ΔI_L / (8·f_sw·C),  ΔV_ESR = ΔI_L · R_ESR
            Topology::Buck => di_l * (self.r_esr_out_cap + 1.0 / (8.0 * self.f_sw * self.c_out)),
            // Boost/BuckBoost: cap is disconnected from the inductor during the ON
            // phase and must supply the load alone, so:
            //   ΔV_cap = I_load · D / (f_sw · C)
            //   ΔV_ESR = ΔI_L · R_ESR  (current step at the ON→OFF transition)
            Topology::Boost | Topology::BuckBoost => {
                self.i_load * d / (self.f_sw * self.c_out)
                    + di_l * self.r_esr_out_cap
            }
        };

        // vpp is independent of f_x_divisor — probe at a reference divisor.
        // to_transfer_function_inner does NOT call crossover_divisor, so no recursion.
        let (_, dac) = self.to_transfer_function_inner(1.0, v_in, topology);
        let vpp = dac.vpp;
        let b0_max = vpp / (dv_out * self.safety_factor);

        // Binary search in the valid region (divisor ≥ 8, i.e. f_x ≤ f_sw/8).
        // Below divisor=8 the bilinear transform breaks down near the Nyquist
        // frequency and produces non-physical b0 values.
        // In the valid region b0 is monotonically decreasing with divisor.
        let mut lo = 8.0_f64;
        let mut hi = 100_000.0_f64;
        let mut i = 0;
        while i < 60 {
            let mid = (lo + hi) / 2.0;
            let (tf, _) = self.to_transfer_function_inner(mid, v_in, topology);
            let b0 = tf.to_2p2z().b0 as f64;
            if b0 <= b0_max {
                hi = mid; // criterion met — try lower divisor (higher bandwidth)
            } else {
                lo = mid; // criterion violated — increase divisor (lower bandwidth)
            }
            i += 1;
        }

        // Additional constraint for Boost and BuckBoost: RHP zero.
        //
        // Boost-type topologies have a right-half-plane zero at
        //   ω_RHP = D'^2 × R_load / L
        // that limits the outer voltage-loop bandwidth regardless of whether
        // peak current mode control is used.  Exceeding it causes the output
        // to move in the wrong direction in response to a control action,
        // leading to large oscillations — exactly what is observed when the
        // controller exits the clamped region after a long saturated transient.
        //
        // Constraint: f_x ≤ f_RHP / (2 × safety_factor)
        //   → divisor ≥ 4π × safety_factor × f_sw / ω_RHP
        let rhp_divisor = match topology {
            Topology::Buck => 0.0, // Buck has no RHP zero
            Topology::Boost | Topology::BuckBoost => {
                let d_prime = 1.0 - d;
                let r_load = self.v_out / self.i_load;
                let omega_rhp = d_prime * d_prime * r_load / self.l_inductor;
                4.0 * PI * self.safety_factor * self.f_sw / omega_rhp
            }
        };

        // Return the more conservative of the two constraints.
        if rhp_divisor > hi { rhp_divisor } else { hi }
    }

    /// Design the 2P2Z compensator for these circuit parameters.
    ///
    /// The crossover frequency is selected automatically via `crossover_divisor()`:
    /// the highest bandwidth where `b0 × ΔV_out_ripple ≤ vpp / safety_factor`.
    pub const fn to_transfer_function(
        self,
        v_in: f64,
        topology: Topology,
    ) -> (TransferFunction, DacSettings) {
        self.to_transfer_function_inner(self.crossover_divisor(topology, v_in), v_in, topology)
    }

    /// Compute only the DAC slope-compensation settings for a given input voltage.
    ///
    /// Unlike [`to_transfer_function`], this function is cheap and depends only on
    /// `v_in` — suitable for calling from a control task after measuring the actual
    /// bus voltage for Vin feed-forward.
    ///
    /// The 2P2Z coefficients produced by `to_transfer_function` do **not** need to
    /// be recomputed when Vin changes for a **Buck** converter: `q_inv_no_pi` always
    /// simplifies to the constant `1/π` regardless of duty cycle, so `h_dc` and
    /// `ω_p1` depend only on the load and reactive components.  For Boost/BuckBoost
    /// the `D'²` factor in `h_dc` and `ω_p1` does vary with Vin.
    ///
    /// # Runtime slope update pattern
    ///
    /// ```ignore
    /// // In a 1 ms task, after measuring v_in:
    /// let s = CTRL_PARAMS.dac_settings(v_in, Topology::Buck);
    /// // INCDATA = ceil(|dac_slope| / LSB  ×  16 / F_HR  ×  HR_TICKS_PER_DAC_INC)
    /// let step = ((-s.dac_slope / LSB) * 16.0 / F_HR * CR2).ceil().max(1.0) as u16;
    /// DAC_STEP_LIVE.store(step, Ordering::Relaxed);
    /// ```
    pub const fn dac_settings(self, v_in: f64, topology: Topology) -> DacSettings {
        let t_sw = 1.0 / self.f_sw;
        let (steady_state_duty, inductor_current_up_slope) = match topology {
            Topology::Buck => (
                (self.v_out + self.v_diode) / v_in,
                (v_in - self.v_out - self.v_diode) * self.current_sense_gain / self.l_inductor,
            ),
            Topology::Boost => (
                1.0 - v_in / (self.v_out - self.v_diode),
                v_in * self.current_sense_gain / self.l_inductor,
            ),
            Topology::BuckBoost => (
                self.v_out / (v_in + self.v_out - self.v_diode),
                v_in * self.current_sense_gain / self.l_inductor,
            ),
        };
        let inv_steady_state_duty = 1.0 - steady_state_duty;
        let slope_compensation_factor = (1.0 + PI / 2.0) / (PI * inv_steady_state_duty);
        let dac_down_slope = -(slope_compensation_factor - 1.0) * inductor_current_up_slope;
        let vpp = -dac_down_slope * t_sw;
        DacSettings {
            dac_slope: dac_down_slope,
            vpp,
        }
    }

    /// Compute the transfer function for a given crossover divisor.
    /// This is the inner implementation called by both `to_transfer_function`
    /// (which passes the auto-computed divisor) and `crossover_divisor`
    /// (which probes different divisor values during the binary search).
    /// Neither calls the other — there is no recursion.
    const fn to_transfer_function_inner(
        self,
        f_x_divisor: f64,
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
                f_x_divisor,
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
    f_x_divisor: f64,
    phase_margin: PhaseMargin,
    cycles_per_tick: usize,

    ohmega_p1: f64,
    ohmega_esr: f64,
    h_dc: f64,
}

impl TransferFunction {
    pub const fn to_2p2z(self) -> TwoPoleTwoZeroParams<f32> {
        let TransferFunction {
            f_sw,
            f_x_divisor,
            phase_margin,
            cycles_per_tick,
            ohmega_p1,
            ohmega_esr,
            h_dc,
        } = self;

        let ohmega_n = self.ohmega_n();
        p!(ohmega_n, "628 300");

        // Crossover frequency: f_x = f_sw / f_x_divisor.
        // Typical: 13.33 (≈ 7.5 % of f_sw) for Buck.  Use 50–100 for large-Cout
        // Boost/BuckBoost where the very low plant pole (ω_p1 ≈ D'^2/(R·C)) would
        // otherwise force extremely large b-coefficients that cause limit cycling.
        let f_x = f_sw / f_x_divisor;
        //p!(f_x, "15000");

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

        TwoPoleTwoZeroParams {
            a1: a1 as _,
            a2: a2 as _,
            b0: b0 as _,
            b1: b1 as _,
            b2: b2 as _,
        }
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
        let q_inv = self.q_inv;
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
