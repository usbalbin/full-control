use electronics_sim::{
    Capacitance, Current, CurrentConduction, CurrentModeConverter, Inductance,
    Parameters as SimParameters, Resistance, Time, Voltage,
};
use full_control::{
    buck_boost::Mode,
    control_2p2z::{
        DacSettings, Parameters, PhaseMargin, Topology as ControlTopology, TransferFunction,
        TwoPoleTwoZeroParams,
    },
    fmac::FmacIir,
};

// ── Physical circuit constants (same as control-test.rs) ────────────────────
const F_SW: f64 = 500e3; // 500 kHz
const T_PERIOD: Time = Time(1.0 / F_SW);
const V_IN: Voltage = Voltage(24.0);
const V_TARGET: Voltage = Voltage(13.5);
const C_OUT: Capacitance = Capacitance(47e-6); // 47 µF
const L_INDUCTOR: Inductance = Inductance(4e-6); // 4 µH
const R_ESR: Resistance = Resistance(10e-3); // 10 mΩ
const R_SERIES: Resistance = Resistance(35e-3); // 35 mΩ
const CS_GAIN: f64 = 0.066; // ACS37030: 66 mV/A
const R_LOAD: f64 = 6.0; // 6 Ω → ~2.25 A at 13.5 V
/// Maximum trip-current the controller can command (A).
/// Must exceed the peak inductor current PLUS the slope-compensation offset
/// at the trip point (≈ |slope| × t_on ≈ 2.6 A at D = 0.56).  Set higher than
/// the physical inductor-current limit to provide DAC headroom.
const MAX_CURRENT: Current = Current(10.0);

// ── STM32G474 ADC / DAC ────────────────────────────────────────────────────
const V_REF: f64 = 3.3;
const ADC_MAX: f64 = 4095.0; // 12-bit
const LSB: f64 = V_REF / ADC_MAX; // ~0.806 mV per code

// ── Feedback resistor divider ───────────────────────────────────────────────
const R_FB_HI: f64 = 47_000.0; // 47 kΩ
const R_FB_LO: f64 = 10_000.0; // 10 kΩ
const DIVIDER_RATIO: f64 = R_FB_LO / (R_FB_HI + R_FB_LO); // ≈ 0.1754

// ── Controller target and DAC limits in codes ───────────────────────────────
const TARGET_CODE: f64 = V_TARGET.0 * DIVIDER_RATIO / LSB;
const DAC_MAX_CODE: f64 = MAX_CURRENT.0 * CS_GAIN / LSB; // ≈ 819

// ── 2P2Z compensator design ────────────────────────────────────────────────
const CTRL_PARAMS: Parameters = Parameters {
    v_out: V_TARGET.0,
    c_out: C_OUT.0,
    f_sw: F_SW,
    l_inductor: L_INDUCTOR.0,
    r_esr_out_cap: R_ESR.0,
    current_sense_gain: CS_GAIN,
    i_load: V_TARGET.0 / R_LOAD,
    v_diode: 0.0,
    phase_margin: PhaseMargin::Manual {
        phase_margin: 75.0f64.to_radians(),
    },
    safety_factor: 2.0,
    cycles_per_tick: 1,
};

// Transfer function and DAC settings at the Buck operating point
const TF_DAC: (TransferFunction, DacSettings) =
    CTRL_PARAMS.to_transfer_function(V_IN.0, ControlTopology::Buck);

// Physical-domain weights (error in Volts, output in current-sense Volts)
const WEIGHTS_PHYS: TwoPoleTwoZeroParams<f32> = TF_DAC.0.to_2p2z();

// Code-domain weights: b-coefficients scaled by 1/divider_ratio.
// The a-coefficients are unchanged (they multiply past outputs already in codes).
const WEIGHTS_CODE: TwoPoleTwoZeroParams<f32> = TwoPoleTwoZeroParams {
    a1: WEIGHTS_PHYS.a1,
    a2: WEIGHTS_PHYS.a2,
    b0: (WEIGHTS_PHYS.b0 as f64 / DIVIDER_RATIO) as f32,
    b1: (WEIGHTS_PHYS.b1 as f64 / DIVIDER_RATIO) as f32,
    b2: (WEIGHTS_PHYS.b2 as f64 / DIVIDER_RATIO) as f32,
};

// FMAC gain exponent: smallest R such that all |coeff| / 2^R < 1.0 (fits q1.15).
// Derived automatically from WEIGHTS_CODE — updates if circuit constants change.
const FMAC_R: u32 = WEIGHTS_CODE.min_fmac_r();

// Slope compensation in A/s for the simulator
const SLOPE_AMP_PER_SEC: f64 = TF_DAC.1.dac_slope / CS_GAIN;

#[cfg_attr(not(feature = "text-log"), allow(unused_variables))]
fn main() {
    // ── FMAC coefficient setup ─────────────────────────────────────────────
    // Convert WEIGHTS_CODE to q1.15 hardware register values.
    // I/O normalization: codes × 2^R → q1.15;  q1.15 → codes by >> R.
    let coeffs = WEIGHTS_CODE.fmac_coeffs(FMAC_R);
    let b = [coeffs[0], coeffs[1], coeffs[2]];
    let a = [coeffs[3], coeffs[4]];
    let y_min: i16 = 0;
    let y_max: i16 = (DAC_MAX_CODE as i32 * (1i32 << FMAC_R)) as i16;

    // ── Controller design summary ──────────────────────────────────────────
    let divisor = CTRL_PARAMS.crossover_divisor(ControlTopology::Buck, V_IN.0);
    println!("Control-test-FMAC: Pure Buck, V_in={:.0}V → V_out={:.1}V", V_IN.0, V_TARGET.0);
    println!("  f_x        = {:.0} Hz (divisor={:.1})", F_SW / divisor, divisor);
    println!(
        "  Code-domain: b0={:.4}  b1={:.4}  b2={:.4}  a1={:.4}  a2={:.4}",
        WEIGHTS_CODE.b0, WEIGHTS_CODE.b1, WEIGHTS_CODE.b2, WEIGHTS_CODE.a1, WEIGHTS_CODE.a2,
    );
    println!(
        "  FMAC R={}    b=[{:#06x},{:#06x},{:#06x}]  a=[{:#06x},{:#06x}]",
        FMAC_R, b[0] as u16, b[1] as u16, b[2] as u16, a[0] as u16, a[1] as u16,
    );
    println!("  I/O shift  : codes × {} → q1.15;  y_bits >> {} → codes", 1u32 << FMAC_R, FMAC_R);
    println!("  Target code: {:.0}  (q1.15 = {:#06x})", TARGET_CODE, target_q115());
    println!("  DAC max    : {:.0}  (q1.15 y_max = {:#06x})", DAC_MAX_CODE, y_max as u16);
    println!("  Slope      : {:.0} A/s", SLOPE_AMP_PER_SEC);
    println!();

    // ── Simulator ──────────────────────────────────────────────────────────
    let sim_params = SimParameters {
        period: T_PERIOD,
        slope_amp_per_sec: SLOPE_AMP_PER_SEC,
        r_series: R_SERIES,
        r_esr: R_ESR,
        c_out: C_OUT,
        l_inductor: L_INDUCTOR,
        c_in: Capacitance(0.0),
        r_esr_cin: Resistance(0.0),
        r_in: Resistance(0.0),
        l_in: Inductance(0.0),
        tau_current_sense: Time(0.0),
        tau_dac: Time(0.0),
        t_prop_delay: Time(0.0),
        t_dac_sample: Time(0.0),
        current_conduction: CurrentConduction::Diode,
    };
    let mut sim = CurrentModeConverter::new(sim_params, Mode::Buck);

    // FMAC controller — operates entirely in q1.15
    let mut ctrl = FmacIir::new(b, a, FMAC_R, y_min, y_max);

    let mut time = Time(0.0);

    #[cfg(feature = "text-log")]
    {
        println!(
            "{:>9} {:>8} {:>7} {:>5} {:>5} {:>7} {:>5}",
            "t[ms]", "Vout[V]", "err[V]", "adc", "dac", "IL[A]", "d[%]",
        );
        println!("{}", "-".repeat(62));
    }

    // ── Soft-start ─────────────────────────────────────────────────────────
    let soft_cycles = 3000_usize;
    for i in 0..soft_cycles {
        // Ramp target from 0 to TARGET_CODE in q1.15
        let soft_tgt_q115 =
            ((TARGET_CODE * (i + 1) as f64 / soft_cycles as f64) as i32 * (1i32 << FMAC_R))
                as i16;

        let adc_code = adc_read(sim.v_out);
        let x0 = q115_error(soft_tgt_q115, adc_code);
        let y_bits = ctrl.update(x0);
        let dac_code = q115_to_dac(y_bits);
        let trip = dac_to_trip(dac_code);

        let (t_on, _i_max) = sim.tick(V_IN, trip, |v| Current(v.0 / R_LOAD));

        time += T_PERIOD;

        #[cfg(feature = "text-log")]
        if i % 100 == 0 || i >= soft_cycles - 5 {
            log_line(time, &sim, t_on, adc_code, dac_code);
        }
    }

    // ── Steady-state ───────────────────────────────────────────────────────
    #[cfg(feature = "text-log")]
    println!(">>> STEADY STATE (R_load = {:.1} Ω)", R_LOAD);

    let steady_cycles = 5000_usize;
    for i in 0..steady_cycles {
        let adc_code = adc_read(sim.v_out);
        let x0 = q115_error(target_q115(), adc_code);
        let y_bits = ctrl.update(x0);
        let dac_code = q115_to_dac(y_bits);
        let trip = dac_to_trip(dac_code);

        let (t_on, _i_max) = sim.tick(V_IN, trip, |v| Current(v.0 / R_LOAD));

        time += T_PERIOD;

        #[cfg(feature = "text-log")]
        if i % 200 == 0 || i >= steady_cycles - 5 {
            log_line(time, &sim, t_on, adc_code, dac_code);
        }
    }

    // ── Load step: 6 Ω → 3 Ω ──────────────────────────────────────────────
    let load_r = 3.0;
    #[cfg(feature = "text-log")]
    println!(">>> LOAD STEP: R_load {:.1} Ω → {:.1} Ω", R_LOAD, load_r);

    let load_cycles = 5000_usize;
    for i in 0..load_cycles {
        let adc_code = adc_read(sim.v_out);
        let x0 = q115_error(target_q115(), adc_code);
        let y_bits = ctrl.update(x0);
        let dac_code = q115_to_dac(y_bits);
        let trip = dac_to_trip(dac_code);

        let (t_on, _i_max) = sim.tick(V_IN, trip, |v| Current(v.0 / load_r));

        time += T_PERIOD;

        #[cfg(feature = "text-log")]
        if i < 200 || i % 200 == 0 || i >= load_cycles - 5 {
            log_line(time, &sim, t_on, adc_code, dac_code);
        }
    }
}

/// Target ADC code in q1.15 (= TARGET_CODE × 2^FMAC_R).
#[inline(always)]
fn target_q115() -> i16 {
    (TARGET_CODE as i32 * (1i32 << FMAC_R)) as i16
}

/// Model 12-bit ADC read: output voltage through resistor divider, quantized.
fn adc_read(v_out: Voltage) -> u16 {
    let v_adc = v_out.0 * DIVIDER_RATIO;
    (v_adc / LSB).round().clamp(0.0, ADC_MAX) as u16
}

/// Compute q1.15 error: target_q115 − adc_code × 2^R, clamped to i16.
#[inline(always)]
fn q115_error(target_q115: i16, adc_code: u16) -> i16 {
    let adc_q115 = (adc_code as i32) << FMAC_R;
    (target_q115 as i32 - adc_q115).clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

/// Convert q1.15 FMAC output to a 12-bit DAC code (codes = y_bits >> R, rounded).
#[inline(always)]
fn q115_to_dac(y_bits: i16) -> u16 {
    let rounding = 1i32 << (FMAC_R - 1);
    ((y_bits as i32 + rounding) >> FMAC_R).clamp(0, DAC_MAX_CODE as i32) as u16
}

/// Convert a 12-bit DAC code to a physical trip current for the simulator.
fn dac_to_trip(dac_code: u16) -> Current {
    let trip_v = dac_code as f64 * LSB;
    Current((trip_v / CS_GAIN).clamp(0.0, MAX_CURRENT.0))
}

#[cfg(feature = "text-log")]
fn log_line(time: Time, sim: &CurrentModeConverter, t_on: Time, adc_code: u16, dac_code: u16) {
    let d_pct = t_on.0 * F_SW * 100.0;
    let err_v = V_TARGET.0 - sim.v_out.0;
    println!(
        "{:9.4} {:8.4} {:+7.4} {:5} {:5} {:7.4} {:5.1}",
        time.0 * 1e3,
        sim.v_out.0,
        err_v,
        adc_code,
        dac_code,
        sim.i_inductor.0,
        d_pct,
    );
}
