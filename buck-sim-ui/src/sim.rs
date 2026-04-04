use electronics_sim::{
    Capacitance, Current, CurrentModeConverter, Inductance,
    Parameters as SimParameters, Resistance, Time, Voltage,
};
pub use electronics_sim::CurrentConduction;
use full_control::{
    buck_boost::Mode,
    control_2p2z::{Parameters, PhaseMargin, Topology as ControlTopology},
};

// ── Hardware profiles ────────────────────────────────────────────────────────

/// MCU-specific parameters (comparator, ADC, processing, HRTIM slope quantization).
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
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
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CsProfile {
    pub name: String,
    pub cs_gain_mv_a: f64,     // Sensitivity [mV/A]
    pub cs_bandwidth_khz: f64, // Current-sense amplifier bandwidth [kHz] (0 = ideal)
}

impl CsProfile {
    pub fn ideal() -> Self {
        Self { name: "Ideal".into(), cs_gain_mv_a: 66.0, cs_bandwidth_khz: 0.0 }
    }

    pub fn acs37030() -> Self {
        Self { name: "ACS37030 ±20A".into(), cs_gain_mv_a: 66.0, cs_bandwidth_khz: 5000.0 }
    }

    pub fn presets() -> Vec<Self> {
        vec![Self::ideal(), Self::acs37030()]
    }
}

/// Slope compensation DAC parameters.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
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

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum LoadKind {
    Steps,
    Battery,
}

/// All adjustable simulation parameters — one field per slider.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SimParams {
    pub v_in: f64,          // Input voltage [V]
    pub v_out_target: f64,  // Output voltage setpoint [V]
    pub f_sw_khz: f64,      // Switching frequency [kHz]
    pub l_uh: f64,          // Inductance [µH]
    pub c_out_uf: f64,      // Output capacitance [µF]
    pub r_esr_mohm: f64,    // Output cap ESR [mΩ]
    pub r_series_mohm: f64, // Inductor DCR + switch R_ds(on) [mΩ]
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

    // ── Timing ─────────────────────────────────────────────────────────────
    pub blanking_ns: f64,       // Comparator blanking window [ns] (0 = no blanking)
    pub adc_sample_ns: f64,     // ADC sample point [ns from period start] (0 = start of cycle)
    pub max_duty_pct: f64,      // Maximum duty cycle [%] (100 = no limit)
    pub slope_overcomp: f64,    // Slope over-compensation factor (1.0 = none, 1.5 = firmware default)

    // ── Hardware profiles ─────────────────────────────────────────────────
    pub mcu: McuProfile,
    pub cs: CsProfile,
    pub dac: DacProfile,

    // ── Multi-phase ──────────────────────────────────────────────────────
    pub num_phases: usize,
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
            max_current: 10.0,
            current_conduction: CurrentConduction::Synchronous,
            load_kind: LoadKind::Steps,
            r_loads: vec![6.0, 3.0, 100.0],
            crossover_khz: 50.0,
            cycles_per_tick: 1,
            bat_v_init: 11.0,
            bat_r_int_mohm: 50.0,
            bat_c_mf: 20.0,
            blanking_ns: 0.0,
            adc_sample_ns: 0.0,
            max_duty_pct: 100.0,
            slope_overcomp: 1.5,
            mcu: McuProfile::ideal(),
            cs: CsProfile::ideal(),
            dac: DacProfile::ideal(),
            num_phases: 1,
        }
    }
}

/// Per-phase data captured each switching cycle.
#[derive(Debug, Clone)]
pub struct PhasePoint {
    pub t_on: f32,
    pub duty_pct: f32,
    pub i_l_min: f32,
    pub i_l_max: f32,
}

/// One sample captured per switching cycle.
#[derive(Debug, Clone)]
pub struct SimPoint {
    pub t_ms: f32,
    pub v_out: f32,
    pub phases: Vec<PhasePoint>,
    /// Combined (interleaved) total current envelope — accounts for ripple cancellation.
    pub i_total_min: f32,
    pub i_total_max: f32,
    /// Battery open-circuit voltage [V].  0.0 when not in Battery mode.
    pub v_bat: f32,
}

impl SimPoint {
    /// Average total inductor current across all phases.
    pub fn i_total_avg(&self) -> f32 {
        self.phases.iter().map(|p| (p.i_l_min + p.i_l_max) / 2.0).sum()
    }

    /// Average duty cycle across all phases.
    pub fn duty_pct_avg(&self) -> f32 {
        if self.phases.is_empty() {
            return 0.0;
        }
        self.phases.iter().map(|p| p.duty_pct).sum::<f32>() / self.phases.len() as f32
    }
}

/// Build the per-phase compensator `Parameters` from UI-level `SimParams`.
///
/// Returns `None` when the operating point is invalid.
///
/// For multi-phase converters, use [`build_ctrl_params_multi`] to get
/// parameters with the plant gain adjusted for N parallel phases.
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
    let cs_gain = p.cs.cs_gain_mv_a * 1e-3;
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

/// Build compensator `Parameters` with the plant gain adjusted for N phases.
///
/// In a multi-phase PCMC converter, all N phases respond to the same trip
/// current.  The effective control-to-output DC gain is N× higher than a
/// single phase: `h_dc_multi = N × R_load / R_i / (...)`.
///
/// We model this by dividing `current_sense_gain` by N, which makes the
/// compensator design produce N× less loop gain to compensate.  All other
/// parameters (slope comp, crossover target, phase margin) remain per-phase.
pub fn build_ctrl_params_multi(p: &SimParams) -> Option<Parameters> {
    let mut params = build_ctrl_params(p)?;
    let n = p.num_phases.max(1) as f64;
    if n > 1.0 {
        params.current_sense_gain /= n;
    }
    Some(params)
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
/// Returns `Err(reason)` when parameters are invalid or the compensator design
/// is infeasible, so the UI can display the specific cause.
pub fn run_simulation(p: &SimParams) -> Result<Vec<SimPoint>, String> {
    if p.v_out_target >= p.v_in * 0.99 {
        return Err("V_out must be less than V_in".into());
    }
    if p.f_sw_khz <= 0.0 || p.l_uh <= 0.0 || p.c_out_uf <= 0.0 {
        return Err("f_sw, L, and C_out must be positive".into());
    }

    // Validate load-specific params
    match p.load_kind {
        LoadKind::Steps => {
            if p.r_loads.is_empty() || p.r_loads.iter().any(|&r| r <= 0.0) {
                return Err("Load resistances must be positive".into());
            }
        }
        LoadKind::Battery => {
            if p.bat_v_init >= p.v_out_target {
                return Err("Battery initial voltage must be below V_out target".into());
            }
            if p.bat_r_int_mohm <= 0.0 || p.bat_c_mf <= 0.0 {
                return Err("Battery R_int and C must be positive".into());
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
    let cs_gain = p.cs.cs_gain_mv_a * 1e-3;

    // Auto-scale feedback divider so V_adc ≈ 75 % of ADC range at the target.
    let divider_ratio = (V_REF * 0.75) / p.v_out_target;
    if divider_ratio >= 1.0 {
        return Err("V_out target too low (< V_ref × 0.75)".into());
    }

    let target_code = p.v_out_target * divider_ratio / LSB;
    let dac_max_code = p.max_current * cs_gain / LSB;

    // Per-phase parameters: used for slope compensation and DAC settings.
    let ctrl_params = build_ctrl_params(p)
        .ok_or("Invalid operating point for controller design")?;

    // Multi-phase parameters: used for compensator TF design.
    // With N phases responding to the same trip current, the effective plant
    // gain is N× higher.  Dividing current_sense_gain by N makes the
    // compensator design produce correspondingly less loop gain.
    let ctrl_params_multi = build_ctrl_params_multi(p)
        .ok_or("Invalid operating point for controller design")?;

    // Slope comp and DAC settings use per-phase parameters
    let (_, dac) = ctrl_params.to_transfer_function(p.v_in, ControlTopology::Buck);
    let slope_amp_per_sec = dac.dac_slope / cs_gain;

    let slope_step_size_a = if p.mcu.min_slope_steps_on_time > 0 {
        let d_nom = p.v_out_target / p.v_in;
        let steps_per_period = (p.mcu.min_slope_steps_on_time as f64 / d_nom).ceil();
        slope_amp_per_sec.abs() * t_period / steps_per_period
    } else {
        0.0
    };

    // Compensator design uses multi-phase-aware parameters
    let (tf, _) = ctrl_params_multi.to_transfer_function(p.v_in, ControlTopology::Buck);

    let weights_phys = tf.to_2p2z().ok_or_else(|| {
        format!(
            "Compensator infeasible: transport delays erode too much phase \
             at this crossover frequency. Max feasible: {:.1} kHz. \
             Reduce f_x, lower cycles/tick, or reduce ADC/processing/DAC delays.",
            ctrl_params_multi.max_feasible_crossover_hz(p.v_in, ControlTopology::Buck) / 1e3
        )
    })?;

    // Ripple / limit-cycling check: b0 × ΔV_ripple must fit inside vpp / safety_factor
    {
        let d = p.v_out_target / p.v_in;
        let v_l_on = p.v_in - p.v_out_target;
        let di_l = v_l_on * d / (f_sw * l_inductor);
        let dv_out = di_l * (r_esr + 1.0 / (8.0 * f_sw * c_out));
        let b0_dv = (weights_phys.b0 as f64).abs() * dv_out;
        let vpp_sf = dac.vpp() / ctrl_params_multi.safety_factor;
        if b0_dv > vpp_sf {
            return Err(format!(
                "Crossover too high: b0 \u{00d7} \u{0394}V_ripple ({:.2}) exceeds vpp/SF ({:.2}), \
                 limit cycling likely. Max feasible: {:.1} kHz",
                b0_dv,
                vpp_sf,
                ctrl_params_multi.max_feasible_crossover_hz(p.v_in, ControlTopology::Buck) / 1e3
            ));
        }
    }
    let weights_code = weights_phys.to_code_domain(divider_ratio);

    if !weights_code.b0.is_finite() || weights_code.b0.abs() > 1e6 {
        return Err("Controller coefficients out of range (b0 non-finite or > 1e6)".into());
    }

    let mut ctrl = weights_code.to_controller(0.0_f32, 4096.0_f32);

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
        t_blanking: Time(p.blanking_ns * 1e-9),
        t_adc_sample_point: Time(p.adc_sample_ns * 1e-9),
        max_duty: p.max_duty_pct / 100.0,
    };
    let num_phases = p.num_phases.max(1);
    let mut sims: Vec<CurrentModeConverter> = (0..num_phases)
        .map(|_| CurrentModeConverter::new(sim_params, Mode::Buck))
        .collect();

    let v_in = Voltage(p.v_in);
    let mut time = 0.0_f64;
    let soft_cycles = 1500_usize;
    let mut held_dac_code: u16 = 0;
    let mut cycle_counter: usize = 0;

    // Helper: tick using multi-phase or single-phase path
    macro_rules! do_tick {
        ($target:expr, $load:expr) => {{
            let ctrl_update = cycle_counter % p.cycles_per_tick == 0;
            if num_phases > 1 {
                tick_multi(
                    &mut ctrl, &mut sims, v_in, $target,
                    $load,
                    divider_ratio, dac_max_code, &ctrl_params, p.max_current,
                    time, ctrl_update, &mut held_dac_code, slope_step_size_a,
                    p.slope_overcomp,
                )
            } else {
                tick_one(
                    &mut ctrl, &mut sims[0], v_in, $target,
                    $load,
                    divider_ratio, dac_max_code, &ctrl_params, p.max_current,
                    time, ctrl_update, &mut held_dac_code, slope_step_size_a,
                    p.slope_overcomp,
                )
            }
        }};
    }

    match p.load_kind {
        LoadKind::Steps => {
            let capacity = 3000 + p.r_loads.len().saturating_sub(1) * 2000;
            let mut results = Vec::with_capacity(capacity);

            // ── Soft-start ────────────────────────────────────────────────────
            for i in 0..soft_cycles {
                let target = target_code as f32 * (i + 1) as f32 / soft_cycles as f32;
                let r0 = p.r_loads[0];
                let pt = do_tick!(target, |v: Voltage| Current(v.0 / r0));
                results.push(pt);
                time += t_period;
                cycle_counter += 1;
            }

            // ── Steady-state ─────────────────────────────────────────────────
            for _ in 0..1500_usize {
                let r0 = p.r_loads[0];
                let pt = do_tick!(target_code as f32, |v: Voltage| Current(v.0 / r0));
                results.push(pt);
                time += t_period;
                cycle_counter += 1;
            }

            // ── Load steps: 2000 cycles each for r_loads[1..] ─────────────────
            for &r in &p.r_loads[1..] {
                for _ in 0..2000_usize {
                    let pt = do_tick!(target_code as f32, |v: Voltage| Current(v.0 / r));
                    results.push(pt);
                    time += t_period;
                    cycle_counter += 1;
                }
            }

            Ok(results)
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
            for i in 0..soft_cycles {
                let target = target_code as f32 * (i + 1) as f32 / soft_cycles as f32;
                let v_oc = bat.v_oc;
                let r_int = bat.r_int;
                let mut pt = do_tick!(target, |v: Voltage| Current(((v.0 - v_oc) / r_int).max(0.0)));
                let avg_i = pt.i_total_avg() as f64;
                bat.update(avg_i, t_period);
                pt.v_bat = bat.v_oc as f32;
                results.push(pt);
                time += t_period;
                cycle_counter += 1;
            }

            // ── CC-CV battery charging ────────────────────────────────────────
            for _ in 0..bat_cycles {
                let v_oc = bat.v_oc;
                let r_int = bat.r_int;
                let mut pt = do_tick!(target_code as f32, |v: Voltage| Current(((v.0 - v_oc) / r_int).max(0.0)));
                let avg_i = pt.i_total_avg() as f64;
                bat.update(avg_i, t_period);
                pt.v_bat = bat.v_oc as f32;
                results.push(pt);
                time += t_period;
                cycle_counter += 1;
            }

            Ok(results)
        }
    }
}

/// Compute the min and max of the total interleaved inductor current.
///
/// Each phase has a triangular current profile (rises from i_min to i_max
/// over t_on, falls from i_max back to i_min over T - t_on), offset by
/// k * T / N.  The total current is the sum of all shifted triangles.
///
/// This is piecewise-linear with breakpoints at each phase's ON->OFF and
/// OFF->ON transitions.  We evaluate i_total at every breakpoint to find
/// the global min and max.
fn interleaved_envelope(phases: &[PhasePoint], period: f32) -> (f32, f32) {
    let n = phases.len();
    if n <= 1 {
        let p = &phases[0];
        return (p.i_l_min, p.i_l_max);
    }

    let offset_k = |k: usize| k as f32 * period / n as f32;

    // Evaluate phase k's current at global time t
    let phase_current = |k: usize, t: f32| -> f32 {
        let p = &phases[k];
        // Local time within this phase's period
        let t_local = (t - offset_k(k)).rem_euclid(period);
        if t_local <= p.t_on {
            // ON ramp: i_min -> i_max
            let frac = if p.t_on > 0.0 { t_local / p.t_on } else { 0.0 };
            p.i_l_min + (p.i_l_max - p.i_l_min) * frac
        } else {
            // OFF ramp: i_max -> i_min
            let t_off = period - p.t_on;
            let frac = if t_off > 0.0 { (t_local - p.t_on) / t_off } else { 0.0 };
            p.i_l_max - (p.i_l_max - p.i_l_min) * frac
        }
    };

    // Collect all breakpoint times (ON->OFF and OFF->ON transitions for each phase)
    let mut events: Vec<f32> = Vec::with_capacity(2 * n);
    for k in 0..n {
        let off = offset_k(k);
        events.push(off % period);                      // OFF->ON
        events.push((off + phases[k].t_on) % period);   // ON->OFF
    }
    events.sort_by(|a, b| a.partial_cmp(b).unwrap());
    events.dedup();

    // Evaluate total current at each breakpoint
    let mut i_min = f32::MAX;
    let mut i_max = f32::MIN;
    for &t in &events {
        let i_total: f32 = (0..n).map(|k| phase_current(k, t)).sum();
        i_min = i_min.min(i_total);
        i_max = i_max.max(i_total);
    }

    (i_min, i_max)
}

/// Tick N interleaved phases sharing a common v_out.
///
/// The controller updates once using `sims[0].v_out_at_adc`, computes a shared
/// trip current, then ticks each phase with `load / N`.  Charge balance gives
/// the combined delta_v which is applied to all phases.
fn tick_multi(
    ctrl: &mut full_control::control_2p2z::TwoPoleTwoZero<f32>,
    sims: &mut [CurrentModeConverter],
    v_in: Voltage,
    target_code: f32,
    load: impl FnMut(Voltage) -> Current + Clone,
    divider_ratio: f64,
    dac_max_code: f64,
    ctrl_params: &Parameters,
    max_current: f64,
    time: f64,
    ctrl_update: bool,
    held_dac_code: &mut u16,
    slope_step_size_a: f64,
    slope_overcomp: f64,
) -> SimPoint {
    let cs_gain = ctrl_params.current_sense_gain;
    let f_sw = ctrl_params.f_sw;
    let n = sims.len();
    let period = 1.0 / f_sw;

    // Controller update once using first phase's ADC reading
    let dac_code = if ctrl_update {
        let adc_code = {
            let v_adc = sims[0].v_out_at_adc.0 * divider_ratio;
            (v_adc / LSB).round().clamp(0.0, ADC_MAX) as u16
        };
        let error = target_code - adc_code as f32;

        let v_target_eff = target_code as f64 * LSB / divider_ratio;
        let dac = ctrl_params.dac_settings_at(
            v_in.0, v_target_eff, ControlTopology::Buck, slope_overcomp,
        );
        let d = (v_target_eff / v_in.0).clamp(0.01, 0.99);
        let slope_offset = dac.slope_offset_codes(d, LSB);

        let dynamic_limit = (dac_max_code + slope_offset) as f32;
        let output = ctrl.update_clamped(error, 0.0_f32, dynamic_limit);

        let code = (output.round() as i32).clamp(0, i32::MAX) as u16;
        *held_dac_code = code;
        code
    } else {
        *held_dac_code
    };

    let trip_limit = max_current * 2.0;
    let mut trip = Current((dac_code as f64 * LSB / cs_gain).clamp(0.0, trip_limit));

    if slope_step_size_a > 0.0 {
        trip = Current((trip.0 / slope_step_size_a).round() * slope_step_size_a);
        trip = Current(trip.0.clamp(0.0, trip_limit));
    }

    // Record shared v_out before ticking
    let v_out_start = sims[0].v_out.0;

    // Tick each phase, collecting per-phase data and delta_v
    let mut phase_points = Vec::with_capacity(n);
    let mut total_delta_v = 0.0_f64;

    for sim in sims.iter_mut() {
        let i_l_min = sim.i_inductor.0 as f32;
        let mut phase_load = load.clone();
        let (t_on, i_l_max_val) = sim.tick(v_in, trip, |v| {
            let full = phase_load(v);
            Current(full.0 / n as f64)
        });
        let i_l_max = i_l_max_val.0 as f32;
        let delta_v = sim.v_out.0 - v_out_start;
        total_delta_v += delta_v;

        phase_points.push(PhasePoint {
            t_on: t_on.0 as f32,
            duty_pct: (t_on.0 * f_sw * 100.0) as f32,
            i_l_min,
            i_l_max,
        });
    }

    // Synchronize all phases to the shared output voltage
    let v_out_final = v_out_start + total_delta_v;
    for sim in sims.iter_mut() {
        sim.v_out = Voltage(v_out_final);
        sim.v_out_at_adc = Voltage(v_out_final);
    }

    // Compute interleaved envelope
    let (i_total_min, i_total_max) = interleaved_envelope(&phase_points, period as f32);

    SimPoint {
        t_ms: (time * 1e3) as f32,
        v_out: v_out_final as f32,
        phases: phase_points,
        i_total_min,
        i_total_max,
        v_bat: 0.0,
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
    ctrl_params: &Parameters,
    max_current: f64,
    time: f64,
    ctrl_update: bool,
    held_dac_code: &mut u16,
    slope_step_size_a: f64,
    slope_overcomp: f64,
) -> SimPoint {
    let cs_gain = ctrl_params.current_sense_gain;
    let f_sw = ctrl_params.f_sw;

    // Decimation: only update controller when ctrl_update is true
    let dac_code = if ctrl_update {
        let adc_code = {
            let v_adc = sim.v_out_at_adc.0 * divider_ratio;
            (v_adc / LSB).round().clamp(0.0, ADC_MAX) as u16
        };
        let error = target_code - adc_code as f32;

        // Recompute slope offset from current operating point (matches firmware task1)
        let v_target_eff = target_code as f64 * LSB / divider_ratio;
        let dac = ctrl_params.dac_settings_at(
            v_in.0, v_target_eff, ControlTopology::Buck, slope_overcomp,
        );
        let d = (v_target_eff / v_in.0).clamp(0.01, 0.99);
        let slope_offset = dac.slope_offset_codes(d, LSB);

        let dynamic_limit = (dac_max_code + slope_offset) as f32;
        let output = ctrl.update_clamped(error, 0.0_f32, dynamic_limit);

        let code = (output.round() as i32).clamp(0, i32::MAX) as u16;
        *held_dac_code = code;
        code
    } else {
        *held_dac_code // ZOH: reuse last control output
    };

    let trip_limit = max_current * 2.0;
    let mut trip = Current((dac_code as f64 * LSB / cs_gain).clamp(0.0, trip_limit));

    // Slope step quantization: snap trip to nearest step boundary
    if slope_step_size_a > 0.0 {
        trip = Current((trip.0 / slope_step_size_a).round() * slope_step_size_a);
        trip = Current(trip.0.clamp(0.0, trip_limit));
    }

    let i_l_min = sim.i_inductor.0 as f32;
    let (t_on, i_l_max) = sim.tick(v_in, trip, load);
    let i_l_max = i_l_max.0 as f32;

    let phase = PhasePoint {
        t_on: t_on.0 as f32,
        duty_pct: (t_on.0 * f_sw * 100.0) as f32,
        i_l_min,
        i_l_max,
    };

    SimPoint {
        t_ms: (time * 1e3) as f32,
        v_out: sim.v_out.0 as f32,
        phases: vec![phase],
        i_total_min: i_l_min,
        i_total_max: i_l_max,
        v_bat: 0.0, // filled in by battery loop when applicable
    }
}
