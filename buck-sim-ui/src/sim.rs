use electronics_sim::{
    Capacitance, Current, CurrentModeConverter, Inductance,
    Parameters as SimParameters, Resistance, Time, Voltage,
};
pub use electronics_sim::CurrentConduction;
use full_control::{
    buck_boost::Mode,
    control_2p2z::{Parameters, PhaseMargin, Topology as ControlTopology, TwoPoleTwoZeroParams},
};

// ── Hardware profiles ────────────────────────────────────────────────────────

/// MCU-specific parameters (comparator, ADC, processing, HRTIM slope quantization).
#[derive(Debug, Clone, PartialEq)]
pub struct McuProfile {
    pub name: String,
    pub comp_delay_ns: f64,           // Comparator propagation delay [ns]
    pub t_adc_us: f64,                // ADC conversion + sampling [µs]
    pub t_processing_us: f64,         // ISR execution [µs]
    pub min_slope_steps_on_time: u16, // Min DAC steps during on-time (0 = ideal)
}

impl McuProfile {
    pub fn ideal() -> Self {
        Self {
            name: "Ideal".into(),
            comp_delay_ns: 0.0,
            t_adc_us: 0.0,
            t_processing_us: 0.0,
            min_slope_steps_on_time: 0,
        }
    }

    pub fn stm32g4() -> Self {
        Self {
            name: "STM32G4".into(),
            comp_delay_ns: 30.0,
            t_adc_us: 1.2,
            t_processing_us: 1.5,
            min_slope_steps_on_time: 55,
        }
    }

    pub fn presets() -> Vec<Self> {
        vec![Self::ideal(), Self::stm32g4()]
    }
}

/// Current sensor parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct CsProfile {
    pub name: String,
    pub cs_bandwidth_khz: f64, // Current-sense amplifier bandwidth [kHz] (0 = ideal)
}

impl CsProfile {
    pub fn ideal() -> Self {
        Self { name: "Ideal".into(), cs_bandwidth_khz: 0.0 }
    }

    pub fn acs37030() -> Self {
        Self { name: "ACS37030".into(), cs_bandwidth_khz: 1000.0 }
    }

    pub fn presets() -> Vec<Self> {
        vec![Self::ideal(), Self::acs37030()]
    }
}

/// Slope compensation DAC parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct DacProfile {
    pub name: String,
    pub dac_filter_bw_khz: f64, // DAC output LP filter bandwidth [kHz] (0 = ideal)
    pub t_dac_us: f64,          // DAC settling / update latency [µs]
}

impl DacProfile {
    pub fn ideal() -> Self {
        Self { name: "Ideal".into(), dac_filter_bw_khz: 0.0, t_dac_us: 0.0 }
    }

    pub fn dac_1msps() -> Self {
        Self { name: "1 MSPS (buffered)".into(), dac_filter_bw_khz: 0.0, t_dac_us: 1.7 }
    }

    pub fn dac_15msps() -> Self {
        Self { name: "15 MSPS (internal)".into(), dac_filter_bw_khz: 0.0, t_dac_us: 0.07 }
    }

    pub fn presets() -> Vec<Self> {
        vec![Self::ideal(), Self::dac_1msps(), Self::dac_15msps()]
    }
}

/// True when any transport delays are non-zero (triggers Calculated phase margin).
pub fn has_transport_delays(mcu: &McuProfile, dac: &DacProfile) -> bool {
    mcu.t_adc_us != 0.0 || mcu.t_processing_us != 0.0 || dac.t_dac_us != 0.0
}

// Fixed ADC/DAC constants (STM32G474)
const V_REF: f64 = 3.3;
const ADC_MAX: f64 = 4095.0;
const LSB: f64 = V_REF / ADC_MAX;

#[derive(Debug, Clone, PartialEq)]
pub enum LoadKind {
    Steps,
    Battery,
}

/// All adjustable simulation parameters — one field per slider.
#[derive(Debug, Clone, PartialEq)]
pub struct SimParams {
    pub v_in: f64,          // Input voltage [V]
    pub v_out_target: f64,  // Output voltage setpoint [V]
    pub f_sw_khz: f64,      // Switching frequency [kHz]
    pub l_uh: f64,          // Inductance [µH]
    pub c_out_uf: f64,      // Output capacitance [µF]
    pub r_esr_mohm: f64,    // Output cap ESR [mΩ]
    pub r_series_mohm: f64, // Inductor DCR + switch R_ds(on) [mΩ]
    pub cs_gain_mv_a: f64,  // Current-sense gain [mV/A]
    pub max_current: f64,   // Maximum trip current [A]

    pub load_kind: LoadKind,

    pub current_conduction: CurrentConduction,

    /// Load phases [Ω]: first element is nominal (soft-start + first steady phase),
    /// each subsequent element adds a 2000-cycle step.  Used when load_kind == Steps.
    pub r_loads: Vec<f64>,

    // ── Controller tuning ─────────────────────────────────────────────────
    pub crossover_khz: f64,    // Target crossover frequency [kHz]
    pub cycles_per_tick: usize, // Switching cycles per control update

    // Battery parameters — used when load_kind == Battery.
    pub bat_v_init: f64,     // Initial battery OCV [V]
    pub bat_r_int_mohm: f64, // Battery internal resistance [mΩ]
    pub bat_c_mf: f64,       // Battery capacitance [mF] (sets charging speed in sim)

    // ── Hardware profiles ─────────────────────────────────────────────────
    pub mcu: McuProfile,
    pub cs: CsProfile,
    pub dac: DacProfile,
}

impl Default for SimParams {
    fn default() -> Self {
        Self {
            v_in: 24.0,
            v_out_target: 13.5,
            f_sw_khz: 500.0,
            l_uh: 4.0,
            c_out_uf: 47.0,
            r_esr_mohm: 10.0,
            r_series_mohm: 35.0,
            cs_gain_mv_a: 66.0,
            max_current: 10.0,
            current_conduction: CurrentConduction::Synchronous,
            load_kind: LoadKind::Steps,
            r_loads: vec![6.0, 3.0, 100.0],
            crossover_khz: 50.0,
            cycles_per_tick: 1,
            bat_v_init: 11.0,
            bat_r_int_mohm: 50.0,
            bat_c_mf: 20.0,
            mcu: McuProfile::ideal(),
            cs: CsProfile::ideal(),
            dac: DacProfile::ideal(),
        }
    }
}

/// One sample captured per switching cycle.
#[derive(Debug, Clone)]
pub struct SimPoint {
    pub t_ms: f32,
    pub t_on: f32,
    pub v_out: f32,
    pub duty_pct: f32,
    pub i_l_min: f32,
    pub i_l_max: f32,
    /// Battery open-circuit voltage [V].  0.0 when not in Battery mode.
    pub v_bat: f32,
}

/// Build the compensator `Parameters` from UI-level `SimParams`.
///
/// Returns `None` when the operating point is invalid.
pub fn build_ctrl_params(p: &SimParams) -> Option<Parameters> {
    if p.v_out_target >= p.v_in * 0.99 {
        return None;
    }
    if p.f_sw_khz <= 0.0 || p.l_uh <= 0.0 || p.c_out_uf <= 0.0 {
        return None;
    }
    let f_sw = p.f_sw_khz * 1e3;
    let l_inductor = p.l_uh * 1e-6;
    let c_out = p.c_out_uf * 1e-6;
    let r_esr = p.r_esr_mohm * 1e-3;
    let cs_gain = p.cs_gain_mv_a * 1e-3;
    let nominal_r = match p.load_kind {
        LoadKind::Steps => p.r_loads[0],
        LoadKind::Battery => p.v_out_target / p.max_current * 2.0,
    };
    let phase_margin = if has_transport_delays(&p.mcu, &p.dac) {
        PhaseMargin::Calculated {
            t_adc: p.mcu.t_adc_us * 1e-6,
            t_processing: p.mcu.t_processing_us * 1e-6,
            t_dac: p.dac.t_dac_us * 1e-6,
        }
    } else {
        PhaseMargin::Manual { phase_margin: 75.0_f64.to_radians() }
    };

    Some(Parameters {
        v_out: p.v_out_target,
        c_out,
        f_sw,
        l_inductor,
        r_esr_out_cap: r_esr,
        current_sense_gain: cs_gain,
        i_load: p.v_out_target / nominal_r,
        v_diode: 0.0,
        phase_margin,
        safety_factor: 2.0,
        crossover_hz: p.crossover_khz * 1e3,
        cycles_per_tick: p.cycles_per_tick,
    })
}

/// Simple lead-acid battery model: capacitor C with series resistance R_int.
///
/// `v_oc` is the open-circuit voltage (rises as the battery charges).
/// When the converter applies voltage v, charging current is `(v - v_oc) / R_int`,
/// clamped to zero — the battery does not discharge into the converter.
struct Battery {
    v_oc: f64, // open-circuit voltage [V]
    r_int: f64, // internal resistance [Ω]
    c: f64,    // capacitance [F] — determines how quickly v_oc rises
}

impl Battery {
    fn new(v_init: f64, r_int: f64, c: f64) -> Self {
        Self { v_oc: v_init, r_int, c }
    }

    /// Update OCV from the average charging current this cycle.
    ///
    /// `avg_i` is the average inductor (= output) current in Amperes.
    /// `dt` is the switching period in seconds.
    fn update(&mut self, avg_i: f64, dt: f64) {
        self.v_oc += avg_i * dt / self.c;
    }
}

/// Run soft-start → steady-state → load-step (or battery charging) simulation.
///
/// Returns `None` when parameters are invalid (e.g. V_out ≥ V_in, or the
/// controller design produces non-finite coefficients).
pub fn run_simulation(p: &SimParams) -> Option<Vec<SimPoint>> {
    if p.v_out_target >= p.v_in * 0.99 {
        return None; // not a Buck operating point
    }
    if p.f_sw_khz <= 0.0 || p.l_uh <= 0.0 || p.c_out_uf <= 0.0 {
        return None;
    }

    // Validate load-specific params
    match p.load_kind {
        LoadKind::Steps => {
            if p.r_loads.is_empty() || p.r_loads.iter().any(|&r| r <= 0.0) {
                return None;
            }
        }
        LoadKind::Battery => {
            if p.bat_v_init >= p.v_out_target
                || p.bat_r_int_mohm <= 0.0
                || p.bat_c_mf <= 0.0
            {
                return None;
            }
        }
    }

    // Convert to SI
    let f_sw = p.f_sw_khz * 1e3;
    let t_period = 1.0 / f_sw;
    let l_inductor = p.l_uh * 1e-6;
    let c_out = p.c_out_uf * 1e-6;
    let r_esr = p.r_esr_mohm * 1e-3;
    let r_series = p.r_series_mohm * 1e-3;
    let cs_gain = p.cs_gain_mv_a * 1e-3;

    // Auto-scale feedback divider so V_adc ≈ 75 % of ADC range at the target.
    let divider_ratio = (V_REF * 0.75) / p.v_out_target;
    if divider_ratio >= 1.0 {
        return None; // v_out_target too small (< V_REF * 0.75)
    }

    let target_code = p.v_out_target * divider_ratio / LSB;
    let dac_max_code = p.max_current * cs_gain / LSB;

    // Design the 2P2Z controller at the nominal operating point
    let ctrl_params = build_ctrl_params(p)?;

    let (tf, dac) = ctrl_params.to_transfer_function(p.v_in, ControlTopology::Buck);
    let slope_amp_per_sec = dac.dac_slope / cs_gain;

    let slope_step_size_a = if p.mcu.min_slope_steps_on_time > 0 {
        let d_nom = p.v_out_target / p.v_in;
        let steps_per_period = (p.mcu.min_slope_steps_on_time as f64 / d_nom).ceil();
        slope_amp_per_sec.abs() * t_period / steps_per_period
    } else {
        0.0
    };

    let weights_phys = tf.to_2p2z();
    let weights_code = TwoPoleTwoZeroParams {
        a1: weights_phys.a1,
        a2: weights_phys.a2,
        b0: (weights_phys.b0 as f64 / divider_ratio) as f32,
        b1: (weights_phys.b1 as f64 / divider_ratio) as f32,
        b2: (weights_phys.b2 as f64 / divider_ratio) as f32,
    };

    if !weights_code.b0.is_finite() || weights_code.b0.abs() > 1e6 {
        return None;
    }

    let mut ctrl = weights_code.to_controller(0.0_f32, dac_max_code as f32);

    let sim_params = SimParameters {
        period: Time(t_period),
        slope_amp_per_sec,
        r_series: Resistance(r_series),
        r_esr: Resistance(r_esr),
        c_out: Capacitance(c_out),
        l_inductor: Inductance(l_inductor),
        c_in: Capacitance(0.0),
        r_esr_cin: Resistance(0.0),
        r_in: Resistance(0.0),
        l_in: Inductance(0.0),
        tau_current_sense: SimParameters::bw_to_tau(p.cs.cs_bandwidth_khz * 1e3),
        tau_dac: SimParameters::bw_to_tau(p.dac.dac_filter_bw_khz * 1e3),
        t_prop_delay: Time(p.mcu.comp_delay_ns * 1e-9),
        t_dac_sample: Time(0.0),
        current_conduction: p.current_conduction,
    };
    let mut sim = CurrentModeConverter::new(sim_params, Mode::Buck);

    let v_in = Voltage(p.v_in);
    let mut time = 0.0_f64;
    let soft_cycles = 1500_usize;
    let mut held_dac_code: u16 = 0;
    let mut cycle_counter: usize = 0;

    match p.load_kind {
        LoadKind::Steps => {
            let capacity = 3000 + p.r_loads.len().saturating_sub(1) * 2000;
            let mut results = Vec::with_capacity(capacity);

            // ── Soft-start ────────────────────────────────────────────────────
            for i in 0..soft_cycles {
                let target = target_code as f32 * (i + 1) as f32 / soft_cycles as f32;
                let r0 = p.r_loads[0];
                let ctrl_update = cycle_counter % p.cycles_per_tick == 0;
                let pt = tick_one(
                    &mut ctrl, &mut sim, v_in, target,
                    |v| Current(v.0 / r0),
                    divider_ratio, dac_max_code, cs_gain, p.max_current, f_sw, time,
                    ctrl_update, &mut held_dac_code, slope_step_size_a,
                );
                results.push(pt);
                time += t_period;
                cycle_counter += 1;
            }

            // ── Steady-state ─────────────────────────────────────────────────
            for _ in 0..1500_usize {
                let r0 = p.r_loads[0];
                let ctrl_update = cycle_counter % p.cycles_per_tick == 0;
                let pt = tick_one(
                    &mut ctrl, &mut sim, v_in, target_code as f32,
                    |v| Current(v.0 / r0),
                    divider_ratio, dac_max_code, cs_gain, p.max_current, f_sw, time,
                    ctrl_update, &mut held_dac_code, slope_step_size_a,
                );
                results.push(pt);
                time += t_period;
                cycle_counter += 1;
            }

            // ── Load steps: 2000 cycles each for r_loads[1..] ─────────────────
            for &r in &p.r_loads[1..] {
                for _ in 0..2000_usize {
                    let ctrl_update = cycle_counter % p.cycles_per_tick == 0;
                    let pt = tick_one(
                        &mut ctrl, &mut sim, v_in, target_code as f32,
                        |v| Current(v.0 / r),
                        divider_ratio, dac_max_code, cs_gain, p.max_current, f_sw, time,
                        ctrl_update, &mut held_dac_code, slope_step_size_a,
                    );
                    results.push(pt);
                    time += t_period;
                    cycle_counter += 1;
                }
            }

            Some(results)
        }

        LoadKind::Battery => {
            // 1500 soft-start + 10 000 charging cycles
            let bat_cycles = 10_000_usize;
            let mut results = Vec::with_capacity(soft_cycles + bat_cycles);

            let mut bat = Battery::new(
                p.bat_v_init,
                p.bat_r_int_mohm * 1e-3,
                p.bat_c_mf * 1e-3,
            );

            // ── Soft-start ────────────────────────────────────────────────────
            // Battery acts as open circuit while V_out < V_oc (current clamped to 0),
            // so the converter charges its output cap normally until V_out rises above
            // the battery voltage, at which point charging begins naturally.
            for i in 0..soft_cycles {
                let target = target_code as f32 * (i + 1) as f32 / soft_cycles as f32;
                let v_oc = bat.v_oc;
                let r_int = bat.r_int;
                let ctrl_update = cycle_counter % p.cycles_per_tick == 0;
                let mut pt = tick_one(
                    &mut ctrl, &mut sim, v_in, target,
                    |v| Current(((v.0 - v_oc) / r_int).max(0.0)),
                    divider_ratio, dac_max_code, cs_gain, p.max_current, f_sw, time,
                    ctrl_update, &mut held_dac_code, slope_step_size_a,
                );
                let avg_i = (pt.i_l_min as f64 + pt.i_l_max as f64) / 2.0;
                bat.update(avg_i, t_period);
                pt.v_bat = bat.v_oc as f32;
                results.push(pt);
                time += t_period;
                cycle_counter += 1;
            }

            // ── CC-CV battery charging ────────────────────────────────────────
            // The converter naturally limits current (peak current mode) while the
            // battery voltage is low (CC phase).  As V_oc approaches V_out_target
            // the current tapers to zero (CV / absorption phase).
            for _ in 0..bat_cycles {
                let v_oc = bat.v_oc;
                let r_int = bat.r_int;
                let ctrl_update = cycle_counter % p.cycles_per_tick == 0;
                let mut pt = tick_one(
                    &mut ctrl, &mut sim, v_in, target_code as f32,
                    |v| Current(((v.0 - v_oc) / r_int).max(0.0)),
                    divider_ratio, dac_max_code, cs_gain, p.max_current, f_sw, time,
                    ctrl_update, &mut held_dac_code, slope_step_size_a,
                );
                let avg_i = (pt.i_l_min as f64 + pt.i_l_max as f64) / 2.0;
                bat.update(avg_i, t_period);
                pt.v_bat = bat.v_oc as f32;
                results.push(pt);
                time += t_period;
                cycle_counter += 1;
            }

            Some(results)
        }
    }
}

fn tick_one(
    ctrl: &mut full_control::control_2p2z::TwoPoleTwoZero<f32>,
    sim: &mut CurrentModeConverter,
    v_in: Voltage,
    target_code: f32,
    load: impl FnMut(Voltage) -> Current,
    divider_ratio: f64,
    dac_max_code: f64,
    cs_gain: f64,
    max_current: f64,
    f_sw: f64,
    time: f64,
    ctrl_update: bool,
    held_dac_code: &mut u16,
    slope_step_size_a: f64,
) -> SimPoint {
    // Decimation: only update controller when ctrl_update is true
    let dac_code = if ctrl_update {
        let adc_code = {
            let v_adc = sim.v_out.0 * divider_ratio;
            (v_adc / LSB).round().clamp(0.0, ADC_MAX) as u16
        };
        let error = target_code - adc_code as f32;
        let output = ctrl.update(error);
        let code = (output.round() as i32).clamp(0, dac_max_code as i32) as u16;
        *held_dac_code = code;
        code
    } else {
        *held_dac_code // ZOH: reuse last control output
    };

    let mut trip = Current((dac_code as f64 * LSB / cs_gain).clamp(0.0, max_current));

    // Slope step quantization: snap trip to nearest step boundary
    if slope_step_size_a > 0.0 {
        trip = Current((trip.0 / slope_step_size_a).round() * slope_step_size_a);
        trip = Current(trip.0.clamp(0.0, max_current));
    }

    let i_l_min = sim.i_inductor.0 as f32;
    let (t_on, i_l_max) = sim.tick(v_in, trip, load);
    let i_l_max = i_l_max.0 as f32;

    SimPoint {
        t_ms: (time * 1e3) as f32,
        t_on: t_on.0 as f32,
        v_out: sim.v_out.0 as f32,
        duty_pct: (t_on.0 * f_sw * 100.0) as f32,
        i_l_min,
        i_l_max,
        v_bat: 0.0, // filled in by battery loop when applicable
    }
}
