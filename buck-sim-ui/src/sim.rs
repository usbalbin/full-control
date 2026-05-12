use electronics_sim::{
    Capacitance, Current, CurrentModeConverter, Inductance,
    Parameters as SimParameters, Resistance, Time, Voltage,
    cap_bank::{CapBank, CapType},
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

/// FET hardware parameters for loss estimation.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FetProfile {
    pub name: String,
    pub rds_on_mohm: f64,  // Rds(on) [mΩ]
    pub coss_pf: f64,      // Output capacitance [pF] — for Eoss switching loss
    pub qg_nc: f64,        // Total gate charge [nC] — for gate drive loss
    pub vgs_v: f64,        // Gate drive voltage [V]
    pub t_rise_ns: f64,    // Current rise time [ns] — for V×I overlap loss
    pub t_fall_ns: f64,    // Current fall time [ns] — for V×I overlap loss
    /// Body-diode reverse-recovery charge [nC]. Datasheet Q_rr.
    /// Drives the di/dt spike on the input current at HS turn-on
    /// (the LS body diode discharges Q_rr through HS as the switch
    /// node is pulled up — a transient current spike of area Q_rr
    /// adds to the rising edge). Zero for GaN HEMTs (no body diode).
    #[serde(default)]
    pub qrr_nc: f64,
    /// Body-diode reverse-recovery time [ns]. Datasheet t_rr. Sets
    /// the width of the Q_rr triangular pulse on the input current
    /// at HS turn-on. Together with `qrr_nc` they pin both the
    /// triangle's area (Q_rr) and base (t_rr), so the peak is
    /// `2·Q_rr / t_rr`. Datasheet values: Si MOSFETs ~10–100 ns;
    /// GaN HEMTs have no body diode → set both to 0.
    #[serde(default)]
    pub trr_ns: f64,
}

impl FetProfile {
    pub fn ideal() -> Self {
        Self {
            name: "Ideal".into(),
            rds_on_mohm: 0.0,
            coss_pf: 0.0,
            qg_nc: 0.0,
            vgs_v: 5.0,
            t_rise_ns: 0.0,
            t_fall_ns: 0.0,
            qrr_nc: 0.0,
            trr_ns: 0.0,
        }
    }

    pub fn epc2306() -> Self {
        // EPC2306 GaN: no body diode → Q_rr ≈ 0. (eGaN HEMTs have a
        // ~0 reverse-conduction stored charge; the input-current spike
        // comes only from i_L, not from a body-diode recovery.)
        Self {
            name: "EPC2306".into(),
            rds_on_mohm: 3.1,
            coss_pf: 600.0,
            qg_nc: 1.1,
            vgs_v: 5.0,
            t_rise_ns: 1.5,
            t_fall_ns: 1.5,
            qrr_nc: 0.0,
            trr_ns: 0.0,
        }
    }

    pub fn epc23102() -> Self {
        Self {
            name: "EPC23102".into(),
            rds_on_mohm: 5.2,
            coss_pf: 370.0,
            qg_nc: 12.0,
            vgs_v: 5.0,
            t_rise_ns: 2.0,
            t_fall_ns: 2.0,
            qrr_nc: 0.0,
            trr_ns: 0.0,
        }
    }

    pub fn presets() -> Vec<Self> {
        vec![Self::ideal(), Self::epc2306(), Self::epc23102()]
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

/// Number of switching cycles for the soft-start ramp.
pub const SOFT_START_CYCLES: usize = 1500;
/// Number of switching cycles at the initial load after soft-start.
pub const STEADY_STATE_CYCLES: usize = 1500;
/// Number of switching cycles per load-step transition.
pub const LOAD_STEP_CYCLES: usize = 2000;

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
    pub dcr_mohm: f64,      // Inductor DCR [mΩ]
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
    pub hs_fet: FetProfile,
    pub ls_fet: FetProfile,
    pub mcu: McuProfile,
    pub cs: CsProfile,
    pub dac: DacProfile,

    // ── Load capacitance (not used in compensator design) ─────────────
    pub c_load_uf: f64,     // Extra load capacitance [µF] (plant-only, not in compensator)

    // ── Input impedance (not used in compensator design) ─────────────
    pub r_in_mohm: f64,    // Input cable resistance [mΩ] (plant-only)
    pub l_in_nh: f64,      // Input cable inductance [nH] (plant-only)
    pub c_in_uf: f64,      // Input decoupling capacitance [µF] (plant-only)

    // ── Output capacitor bank (overrides c_out_uf / r_esr_mohm when non-empty) ──
    pub output_caps: Vec<CapTypeUi>,

    // ── Multi-phase ──────────────────────────────────────────────────────
    pub num_phases: usize,
}

/// UI-friendly capacitor type with user-facing units.
/// When SimParams::output_caps is non-empty, these define the output cap bank
/// and the scalar c_out_uf / r_esr_mohm fields are ignored.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CapTypeUi {
    pub c_uf: f64,       // Capacitance per unit [µF]
    pub count: usize,    // Number of units in parallel
    pub esr_mohm: f64,   // ESR per unit [mΩ]
    pub esl_nh: f64,     // ESL per unit [nH]
}

impl CapTypeUi {
    /// Convert from UI-friendly units to SI units for the solver.
    pub fn to_cap_type(&self) -> CapType {
        CapType {
            c: self.c_uf * 1e-6,
            count: self.count,
            esr: self.esr_mohm * 1e-3,
            esl: self.esl_nh * 1e-9,
        }
    }
}

/// Convert a slice of UI cap types to SI cap types.
pub fn to_cap_types(ui: &[CapTypeUi]) -> Vec<CapType> {
    ui.iter().map(CapTypeUi::to_cap_type).collect()
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
            dcr_mohm: 4.1,
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
            hs_fet: FetProfile::ideal(),
            ls_fet: FetProfile::ideal(),
            mcu: McuProfile::ideal(),
            cs: CsProfile::ideal(),
            dac: DacProfile::ideal(),
            c_load_uf: 0.0,
            r_in_mohm: 0.0,
            l_in_nh: 0.0,
            c_in_uf: 0.0,
            output_caps: Vec::new(),
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
    /// Minimum output voltage (including ESR) within this switching cycle.
    pub v_out_min: f32,
    /// Maximum output voltage (including ESR) within this switching cycle.
    pub v_out_max: f32,
    /// Battery open-circuit voltage [V].  0.0 when not in Battery mode.
    pub v_bat: f32,
    /// Input capacitor voltage [V].  0.0 when input impedance is disabled.
    pub v_in_cap: f32,
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
    // When a cap bank is defined, use its aggregate C and ESR for compensator design.
    // The Bode plot will show the actual composite impedance response.
    let (c_out, r_esr) = if !p.output_caps.is_empty() {
        let si_caps = to_cap_types(&p.output_caps);
        let c_total = CapBank::total_capacitance(&si_caps);
        let esr_eff = CapBank::effective_esr(&si_caps, f_sw);
        (c_total, esr_eff)
    } else {
        (p.c_out_uf * 1e-6, p.r_esr_mohm * 1e-3)
    };
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
    // When cap bank is active, use aggregate C and ESR for checks and the
    // analytical fallback sim (the cap bank path overrides the L-C dynamics).
    let (c_out, r_esr) = if !p.output_caps.is_empty() {
        let si_caps = to_cap_types(&p.output_caps);
        (CapBank::total_capacitance(&si_caps), CapBank::effective_esr(&si_caps, f_sw))
    } else {
        (p.c_out_uf * 1e-6, p.r_esr_mohm * 1e-3)
    };
    let d_nom = (p.v_out_target / p.v_in).clamp(0.01, 0.99);
    let r_series = (p.dcr_mohm
        + d_nom * p.hs_fet.rds_on_mohm
        + (1.0 - d_nom) * p.ls_fet.rds_on_mohm) * 1e-3;
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
        c_out: Capacitance(c_out + p.c_load_uf * 1e-6),
        l_inductor: Inductance(l_inductor),
        c_in: Capacitance(p.c_in_uf * 1e-6),
        r_esr_cin: Resistance(0.0),
        r_in: Resistance(p.r_in_mohm * 1e-3),
        l_in: Inductance(p.l_in_nh * 1e-9),
        tau_current_sense: SimParameters::bw_to_tau(p.cs.cs_bandwidth_khz * 1e3),
        tau_dac: SimParameters::bw_to_tau(p.dac.dac_filter_bw_khz * 1e3),
        t_prop_delay: Time(p.mcu.comp_delay_ns * 1e-9),
        t_dac_sample: Time(0.0),
        current_conduction: p.current_conduction,
        t_blanking: Time(p.blanking_ns * 1e-9),
        t_adc_sample_point: Time(p.adc_sample_ns * 1e-9),
        max_duty: p.max_duty_pct / 100.0,
        inductor_model: None,
        t_dead: Time(0.0),
        v_body_diode: Voltage(0.0),
    };
    let num_phases = p.num_phases.max(1);
    let mut sims: Vec<CurrentModeConverter> = (0..num_phases)
        .map(|_| CurrentModeConverter::new(sim_params, Mode::Buck))
        .collect();

    // Create cap bank if output_caps is non-empty.
    // The cap bank handles the output L-C dynamics via state-space RK4,
    // correctly modelling frequency-dependent impedance of mixed cap types.
    let use_cap_bank = !p.output_caps.is_empty();
    let mut cap_bank_opt = if use_cap_bank {
        let mut si_caps = to_cap_types(&p.output_caps);
        // Add load capacitance as an extra no-ESR, no-ESL cap type if present.
        if p.c_load_uf > 0.0 {
            si_caps.push(CapType {
                c: p.c_load_uf * 1e-6,
                count: 1,
                esr: 1e-3, // small but nonzero ESR to avoid division by zero
                esl: 0.0,
            });
        }
        let bank = CapBank::new(si_caps, 0.0, 0.0);

        // ── Budget check: estimate total RK4 work and reject if too expensive ──
        // This prevents the UI from freezing when caps have very fast RC time
        // constants (e.g. 1µF @ 0.1mΩ → τ=0.1ns → thousands of steps/phase).
        let total_cycles = match p.load_kind {
            LoadKind::Steps => {
                SOFT_START_CYCLES + STEADY_STATE_CYCLES
                    + p.r_loads.len().saturating_sub(1) * LOAD_STEP_CYCLES
            }
            LoadKind::Battery => SOFT_START_CYCLES + STEADY_STATE_CYCLES + 5000,
        };
        let phases_per_cycle = 2 * p.num_phases;
        let steps_per_phase = bank.steps_for_phase(t_period / 2.0);
        let total_steps = total_cycles * phases_per_cycle * steps_per_phase;
        const MAX_TOTAL_STEPS: usize = 20_000_000;
        if total_steps > MAX_TOTAL_STEPS {
            // Find the bottleneck cap for the error message.
            let worst = p.output_caps.iter()
                .filter(|c| c.esl_nh == 0.0)
                .min_by(|a, b| {
                    let ta = a.esr_mohm * a.c_uf;
                    let tb = b.esr_mohm * b.c_uf;
                    ta.partial_cmp(&tb).unwrap()
                });
            let hint = if let Some(w) = worst {
                format!(
                    " Bottleneck: {:.1} µF @ {:.1} mΩ (τ = {:.1} ns, need {} steps/phase). \
                     Increase ESR or capacitance to speed up.",
                    w.c_uf, w.esr_mohm,
                    w.esr_mohm * w.c_uf, // mΩ × µF = ns
                    steps_per_phase,
                )
            } else {
                String::new()
            };
            return Err(format!(
                "Cap bank simulation too expensive (~{:.0}M RK4 steps).{hint}",
                total_steps as f64 / 1e6,
            ));
        }

        Some(bank)
    } else {
        None
    };

    let v_in = Voltage(p.v_in);
    let mut time = 0.0_f64;
    let mut held_dac_code: u16 = 0;
    let mut cycle_counter: usize = 0;

    // Helper: tick using multi-phase or single-phase path,
    // with cap bank or analytical solver depending on configuration.
    macro_rules! do_tick {
        ($target:expr, $load:expr) => {{
            let ctrl_update = cycle_counter % p.cycles_per_tick == 0;
            if let Some(ref mut cap_bank) = cap_bank_opt {
                if num_phases > 1 {
                    tick_multi_cap_bank(
                        &mut ctrl, &mut sims, cap_bank, v_in, $target,
                        $load,
                        divider_ratio, dac_max_code, &ctrl_params, p.max_current,
                        time, ctrl_update, &mut held_dac_code, slope_step_size_a,
                        p.slope_overcomp,
                    )
                } else {
                    tick_one_cap_bank(
                        &mut ctrl, &mut sims[0], cap_bank, v_in, $target,
                        $load,
                        divider_ratio, dac_max_code, &ctrl_params, p.max_current,
                        time, ctrl_update, &mut held_dac_code, slope_step_size_a,
                        p.slope_overcomp,
                    )
                }
            } else {
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
            }
        }};
    }

    match p.load_kind {
        LoadKind::Steps => {
            let capacity = SOFT_START_CYCLES + STEADY_STATE_CYCLES
                + p.r_loads.len().saturating_sub(1) * LOAD_STEP_CYCLES;
            let mut results = Vec::with_capacity(capacity);

            // ── Soft-start ────────────────────────────────────────────────────
            for i in 0..SOFT_START_CYCLES {
                let target = target_code as f32 * (i + 1) as f32 / SOFT_START_CYCLES as f32;
                let r0 = p.r_loads[0];
                let pt = do_tick!(target, |v: Voltage| Current(v.0 / r0));
                results.push(pt);
                time += t_period;
                cycle_counter += 1;
            }

            // ── Steady-state ─────────────────────────────────────────────────
            for _ in 0..STEADY_STATE_CYCLES {
                let r0 = p.r_loads[0];
                let pt = do_tick!(target_code as f32, |v: Voltage| Current(v.0 / r0));
                results.push(pt);
                time += t_period;
                cycle_counter += 1;
            }

            // ── Load steps: LOAD_STEP_CYCLES each for r_loads[1..] ───────────
            for &r in &p.r_loads[1..] {
                for _ in 0..LOAD_STEP_CYCLES {
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
            let mut results = Vec::with_capacity(SOFT_START_CYCLES + bat_cycles);

            let mut bat = Battery::new(
                p.bat_v_init,
                p.bat_r_int_mohm * 1e-3,
                p.bat_c_mf * 1e-3,
            );

            // ── Soft-start ────────────────────────────────────────────────────
            for i in 0..SOFT_START_CYCLES {
                let target = target_code as f32 * (i + 1) as f32 / SOFT_START_CYCLES as f32;
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

    // Tick each phase with incremental v_out propagation.
    // Each phase sees v_out after all preceding phases have contributed their
    // charge, giving more accurate intra-period dynamics during transients.
    let mut phase_points = Vec::with_capacity(n);

    for k in 0..n {
        if k > 0 {
            // Propagate v_out from previous phase so phase k sees the
            // incrementally updated output voltage.
            sims[k].v_out = sims[k - 1].v_out;
        }
        let i_l_min = sims[k].i_inductor.0 as f32;
        let mut phase_load = load.clone();
        let (t_on, i_l_max_val) = sims[k].tick(v_in, trip, |v| {
            let full = phase_load(v);
            Current(full.0 / n as f64)
        });
        let i_l_max = i_l_max_val.0 as f32;

        phase_points.push(PhasePoint {
            t_on: t_on.0 as f32,
            duty_pct: (t_on.0 * f_sw * 100.0) as f32,
            i_l_min,
            i_l_max,
        });
    }

    // Last sim's v_out naturally includes all phase contributions.
    let v_out_final = sims[n - 1].v_out.0;
    // Preserve phase 0's ADC reading (set inside tick() at the ADC sample point).
    let adc_reading = sims[0].v_out_at_adc;
    // Synchronize all phases to the shared output voltage.
    for sim in sims.iter_mut() {
        sim.v_out = Voltage(v_out_final);
        sim.v_out_at_adc = Voltage(v_out_final);
    }
    // Restore phase 0's ADC value so the compensator sees the correct voltage.
    sims[0].v_out_at_adc = adc_reading;

    // Compute interleaved envelope
    let (i_total_min, i_total_max) = interleaved_envelope(&phase_points, period as f32);

    // v_out min/max across all phase ticks
    let v_out_min = sims.iter().map(|s| s.v_out_min_cycle.0 as f32).fold(f32::INFINITY, f32::min);
    let v_out_max = sims.iter().map(|s| s.v_out_max_cycle.0 as f32).fold(f32::NEG_INFINITY, f32::max);

    SimPoint {
        t_ms: (time * 1e3) as f32,
        v_out: v_out_final as f32,
        phases: phase_points,
        i_total_min,
        i_total_max,
        v_out_min,
        v_out_max,
        v_bat: 0.0,
        v_in_cap: sims[0].v_in_cap.0 as f32,
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
        v_out_min: sim.v_out_min_cycle.0 as f32,
        v_out_max: sim.v_out_max_cycle.0 as f32,
        v_bat: 0.0, // filled in by battery loop when applicable
        v_in_cap: sim.v_in_cap.0 as f32,
    }
}

fn tick_one_cap_bank(
    ctrl: &mut full_control::control_2p2z::TwoPoleTwoZero<f32>,
    sim: &mut CurrentModeConverter,
    cap_bank: &mut CapBank,
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

    // Control logic is identical to tick_one
    let dac_code = if ctrl_update {
        let adc_code = {
            let v_adc = sim.v_out_at_adc.0 * divider_ratio;
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

    let i_l_min = sim.i_inductor.0 as f32;
    // Use tick_cap_bank instead of tick — cap bank handles L-C dynamics
    let (t_on, i_l_max) = sim.tick_cap_bank(v_in, trip, load, cap_bank);
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
        v_out_min: cap_bank.v_out_min() as f32,
        v_out_max: cap_bank.v_out_max() as f32,
        v_bat: 0.0,
        v_in_cap: sim.v_in_cap.0 as f32,
    }
}

fn tick_multi_cap_bank(
    ctrl: &mut full_control::control_2p2z::TwoPoleTwoZero<f32>,
    sims: &mut [CurrentModeConverter],
    cap_bank: &mut CapBank,
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

    // Control logic identical to tick_multi
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

    // All phases share the single cap bank (output caps are shared).
    // Each phase ticks sequentially — the cap bank state carries forward.
    let mut phase_points = Vec::with_capacity(n);

    for k in 0..n {
        if k > 0 {
            // Propagate v_out from cap bank to this phase's sim
            sims[k].v_out = Voltage(cap_bank.v_out());
        }
        let i_l_min = sims[k].i_inductor.0 as f32;
        let mut phase_load = load.clone();
        let (t_on, i_l_max_val) = sims[k].tick_cap_bank(v_in, trip, |v| {
            let full = phase_load(v);
            Current(full.0 / n as f64)
        }, cap_bank);
        let i_l_max = i_l_max_val.0 as f32;

        phase_points.push(PhasePoint {
            t_on: t_on.0 as f32,
            duty_pct: (t_on.0 * f_sw * 100.0) as f32,
            i_l_min,
            i_l_max,
        });
    }

    // Final v_out from cap bank
    let v_out_final = cap_bank.v_out();
    let adc_reading = sims[0].v_out_at_adc;
    for sim in sims.iter_mut() {
        sim.v_out = Voltage(v_out_final);
        sim.v_out_at_adc = Voltage(v_out_final);
    }
    sims[0].v_out_at_adc = adc_reading;

    let (i_total_min, i_total_max) = interleaved_envelope(&phase_points, period as f32);

    SimPoint {
        t_ms: (time * 1e3) as f32,
        v_out: v_out_final as f32,
        phases: phase_points,
        i_total_min,
        i_total_max,
        v_out_min: cap_bank.v_out_min() as f32,
        v_out_max: cap_bank.v_out_max() as f32,
        v_bat: 0.0,
        v_in_cap: sims[0].v_in_cap.0 as f32,
    }
}

// ── Power loss estimation ───────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct LossBreakdown {
    pub hs_conduction_w: f64,
    pub ls_conduction_w: f64,
    pub inductor_dcr_w: f64,
    pub hs_switching_w: f64,
    pub ls_switching_w: f64,
    pub hs_eoss_w: f64,
    pub gate_drive_w: f64,
    /// Commutation-loop ringing energy dissipated per second:
    ///   P_ring = N · L · I_pk_per_phase² · f_sw
    /// where L is the PEEC-extracted commutation-loop self-inductance
    /// (from a `pdn_schema::LoopExtraction`) and I_pk_per_phase is the
    /// per-phase peak inductor current. Each switching transition
    /// stores ½·L·I² in the loop's stray L which then rings down into
    /// loop R; both HS turn-on and HS turn-off contribute, so a full
    /// L·I² per cycle. Zero when no LoopExtraction is loaded.
    pub commutation_ringing_w: f64,
    pub total_loss_w: f64,
    pub p_out_w: f64,
    pub efficiency_pct: f64,
}

/// Compute power losses from simulation data and parameters.
///
/// For Steps mode, uses cycles 1500–3000 (steady-state at nominal load before
/// any load steps).  For Battery mode, uses the last 200 cycles.
///
/// `l_commutation_loop_h` is the PEEC-extracted per-phase commutation-
/// loop inductance (from a `pdn_schema::LoopExtraction`). Pass `None`
/// to set the ringing-energy term to zero.
pub fn compute_losses(
    p: &SimParams,
    data: &[SimPoint],
    l_commutation_loop_h: Option<f64>,
) -> Option<LossBreakdown> {
    if data.is_empty() {
        return None;
    }

    let window = match p.load_kind {
        LoadKind::Steps => {
            let start = SOFT_START_CYCLES.min(data.len());
            let end = (SOFT_START_CYCLES + STEADY_STATE_CYCLES).min(data.len());
            if end <= start {
                return None;
            }
            &data[start..end]
        }
        LoadKind::Battery => {
            let n = 200.min(data.len());
            &data[data.len() - n..]
        }
    };

    if window.is_empty() {
        return None;
    }

    let n_phases = p.num_phases.max(1) as f64;
    let f_sw = p.f_sw_khz * 1e3;
    let v_in = p.v_in;

    // Accumulate per-phase averages across the window
    let mut sum_i_rms2 = 0.0_f64; // sum of I²_rms across all phases
    let mut sum_d = 0.0_f64;      // sum of duty across all phases
    let mut sum_i_min = 0.0_f64;  // sum of i_min (valley) across all phases
    let mut sum_i_max = 0.0_f64;  // sum of i_max (peak) across all phases
    let mut sum_v_out = 0.0_f64;
    let mut sum_i_avg_total = 0.0_f64;
    let count = window.len() as f64;

    for pt in window {
        sum_v_out += pt.v_out as f64;
        sum_i_avg_total += pt.i_total_avg() as f64;

        for ph in &pt.phases {
            let i_min = ph.i_l_min as f64;
            let i_max = ph.i_l_max as f64;
            // RMS² for triangle waveform: (i_min² + i_min×i_max + i_max²) / 3
            let i_rms2 = (i_min * i_min + i_min * i_max + i_max * i_max) / 3.0;
            sum_i_rms2 += i_rms2;
            sum_d += ph.duty_pct as f64 / 100.0;
            sum_i_min += i_min;
            sum_i_max += i_max;
        }
    }

    let avg_i_rms2 = sum_i_rms2 / count;         // average total I²_rms (summed across N phases)
    let avg_d = sum_d / count / n_phases;          // average duty per phase
    let avg_i_min = sum_i_min / count / n_phases;  // average valley per phase
    let avg_i_max = sum_i_max / count / n_phases;  // average peak per phase
    let avg_v_out = sum_v_out / count;
    let avg_i_total = sum_i_avg_total / count;

    // Per-phase I²_rms (divide total sum by N to get per-phase)
    let per_phase_i_rms2 = avg_i_rms2 / n_phases;

    // Conduction losses (summed over all N phases)
    let hs_conduction = p.hs_fet.rds_on_mohm * 1e-3 * avg_d * per_phase_i_rms2 * n_phases;
    let ls_conduction = p.ls_fet.rds_on_mohm * 1e-3 * (1.0 - avg_d) * per_phase_i_rms2 * n_phases;
    let inductor_dcr = p.dcr_mohm * 1e-3 * per_phase_i_rms2 * n_phases;

    // Switching losses (per phase, summed over N phases)
    let t_rise = p.hs_fet.t_rise_ns * 1e-9;
    let t_fall = p.hs_fet.t_fall_ns * 1e-9;
    let hs_switching = 0.5 * v_in * (avg_i_min * t_rise + avg_i_max * t_fall) * f_sw * n_phases;

    // LS overlap loss (body diode commutation — typically small for sync FETs)
    let t_rise_ls = p.ls_fet.t_rise_ns * 1e-9;
    let t_fall_ls = p.ls_fet.t_fall_ns * 1e-9;
    let ls_switching = 0.5 * v_in * (avg_i_max * t_rise_ls + avg_i_min * t_fall_ls) * f_sw * n_phases;

    // Eoss: LS Coss charged/discharged each cycle (hard-switched by HS turn-on)
    let coss_ls = p.ls_fet.coss_pf * 1e-12;
    let hs_eoss = 0.5 * coss_ls * v_in * v_in * f_sw * n_phases;

    // Gate drive loss: both FETs per phase
    let qg_total = (p.hs_fet.qg_nc + p.ls_fet.qg_nc) * 1e-9;
    let vgs = p.hs_fet.vgs_v.max(p.ls_fet.vgs_v);
    let gate_drive = qg_total * vgs * f_sw * n_phases;

    // Commutation-loop ringing. Each switching transition stores
    // ½·L·I_pk_per_phase² in the loop's stray L, dissipated in loop
    // R during the ring-down. Two transitions per cycle → L·I²·f_sw
    // per loop; one loop per phase → multiply by n_phases.
    let commutation_ringing = match l_commutation_loop_h {
        Some(l) if l > 0.0 => l * avg_i_max * avg_i_max * f_sw * n_phases,
        _ => 0.0,
    };

    let total_loss = hs_conduction + ls_conduction + inductor_dcr
        + hs_switching + ls_switching + hs_eoss + gate_drive + commutation_ringing;
    let p_out = avg_v_out * avg_i_total;
    let efficiency = if p_out + total_loss > 0.0 {
        p_out / (p_out + total_loss) * 100.0
    } else {
        0.0
    };

    Some(LossBreakdown {
        hs_conduction_w: hs_conduction,
        ls_conduction_w: ls_conduction,
        inductor_dcr_w: inductor_dcr,
        hs_switching_w: hs_switching,
        ls_switching_w: ls_switching,
        hs_eoss_w: hs_eoss,
        gate_drive_w: gate_drive,
        commutation_ringing_w: commutation_ringing,
        total_loss_w: total_loss,
        p_out_w: p_out,
        efficiency_pct: efficiency,
    })
}

/// Compute the effective series resistance [mΩ] from component parameters.
pub fn computed_r_series_mohm(p: &SimParams) -> f64 {
    let d_nom = (p.v_out_target / p.v_in).clamp(0.01, 0.99);
    p.dcr_mohm + d_nom * p.hs_fet.rds_on_mohm + (1.0 - d_nom) * p.ls_fet.rds_on_mohm
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Same parameters as in "Microcontroller-based peak current mode control using digital slope compensation"
    /// https://centaur.reading.ac.uk/31751/1/Microcontroller%20Based%20Peak%20Current%20Mode%20Control%20Using%20Digital%20Slope%20Compensation%20-%20Hallworth%202012.pdf
    fn hallworth_params() -> SimParams {
        SimParams {
            v_in: 16.0,
            v_out_target: 8.0,
            f_sw_khz: 200.0,
            l_uh: 22.0,
            c_out_uf: 440.0,
            r_esr_mohm: 31.0,
            dcr_mohm: 0.0,
            max_current: 10.0,
            current_conduction: CurrentConduction::Diode,
            load_kind: LoadKind::Steps,
            r_loads: vec![4.0, 6.0, 4.0],
            crossover_khz: 15.0,
            cycles_per_tick: 1,
            bat_v_init: 0.0, // Not used
            bat_r_int_mohm: 0.0,// Not used
            bat_c_mf: 0.0,// Not used
            blanking_ns: 0.0,
            adc_sample_ns: 0.0,
            max_duty_pct: 100.0,
            slope_overcomp: 1.0,
            hs_fet: FetProfile::ideal(),
            ls_fet: FetProfile::ideal(),
            mcu: McuProfile::ideal(),
            cs: CsProfile::ideal(),
            dac: DacProfile::ideal(),
            c_load_uf: 0.0,
            r_in_mohm: 0.0,
            l_in_nh: 0.0,
            c_in_uf: 0.0,
            output_caps: Vec::new(),
            num_phases: 1,
        }
    }

    /// Default params for tests — a stable single-phase buck converter.
    fn test_params() -> SimParams {
        SimParams {
            v_in: 24.0,
            v_out_target: 12.0,
            f_sw_khz: 500.0,
            l_uh: 4.0,
            c_out_uf: 47.0,
            r_esr_mohm: 10.0,
            dcr_mohm: 35.0, // high DCR to match old r_series_mohm for test stability
            max_current: 10.0,
            current_conduction: CurrentConduction::Synchronous,
            load_kind: LoadKind::Steps,
            r_loads: vec![6.0, 3.0],
            crossover_khz: 30.0,
            cycles_per_tick: 1,
            bat_v_init: 11.0,
            bat_r_int_mohm: 50.0,
            bat_c_mf: 20.0,
            blanking_ns: 0.0,
            adc_sample_ns: 0.0,
            max_duty_pct: 100.0,
            slope_overcomp: 1.5,
            hs_fet: FetProfile::ideal(),
            ls_fet: FetProfile::ideal(),
            mcu: McuProfile::ideal(),
            cs: CsProfile::ideal(),
            dac: DacProfile::ideal(),
            c_load_uf: 0.0,
            r_in_mohm: 0.0,
            l_in_nh: 0.0,
            c_in_uf: 0.0,
            output_caps: Vec::new(),
            num_phases: 1,
        }
    }

    /// Check that the last N cycles of a simulation are "settled":
    /// v_out within `tol_v` of `v_target`, no NaN/Inf, and positive current.
    fn assert_settled(data: &[SimPoint], n_tail: usize, v_target: f64, tol_v: f64) {
        let tail = &data[data.len() - n_tail..];
        for pt in tail {
            assert!(pt.v_out.is_finite(), "v_out is NaN/Inf");
            assert!(
                (pt.v_out as f64 - v_target).abs() < tol_v,
                "v_out = {:.3} V, expected {:.1} V ± {:.1}",
                pt.v_out,
                v_target,
                tol_v,
            );
            assert!(
                pt.i_total_max >= pt.i_total_min,
                "i_total_max ({}) < i_total_min ({})",
                pt.i_total_max,
                pt.i_total_min,
            );
        }
    }

    // ── Single-phase baseline ──────────────────────────────────────────────

    #[test]
    fn single_phase_settles() {
        let p = test_params();
        let data = run_simulation(&p).expect("simulation should succeed");
        // After soft-start (1500 cycles) the converter should be settled.
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn single_phase_hallworth_settles() {
        let p = hallworth_params();
        let data = run_simulation(&p).expect("simulation should succeed");
        // After soft-start (1500 cycles) the converter should be settled.
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn single_phase_produces_one_phase_point() {
        let p = test_params();
        let data = run_simulation(&p).unwrap();
        for pt in &data {
            assert_eq!(pt.phases.len(), 1, "single-phase should have 1 PhasePoint");
        }
    }

    // ── Multi-phase compensator gain scaling ───────────────────────────────

    #[test]
    fn multi_phase_scales_current_sense_gain() {
        let mut p = test_params();
        let single = build_ctrl_params(&p).unwrap();

        for n in [2, 3, 4, 6] {
            p.num_phases = n;
            let multi = build_ctrl_params_multi(&p).unwrap();
            let expected = single.current_sense_gain / n as f64;
            assert!(
                (multi.current_sense_gain - expected).abs() < 1e-12,
                "N={}: cs_gain_multi={}, expected={}",
                n,
                multi.current_sense_gain,
                expected,
            );
        }
    }

    #[test]
    fn multi_phase_plant_gain_scales_by_n() {
        // The plant DC gain h_dc ∝ 1/current_sense_gain, so with N phases
        // (cs_gain / N) the h_dc should be N× larger.
        let mut p = test_params();
        let single = build_ctrl_params(&p).unwrap();
        let (tf1, _) = single.to_transfer_function(p.v_in, ControlTopology::Buck);
        let h_dc_1 = tf1.design_summary().h_dc;

        for n in [2, 3, 4] {
            p.num_phases = n;
            let multi = build_ctrl_params_multi(&p).unwrap();
            let (tf_n, _) = multi.to_transfer_function(p.v_in, ControlTopology::Buck);
            let h_dc_n = tf_n.design_summary().h_dc;
            let ratio = h_dc_n / h_dc_1;
            assert!(
                (ratio - n as f64).abs() < 0.1,
                "N={}: h_dc ratio = {:.2}, expected {:.1}",
                n,
                ratio,
                n as f64,
            );
        }
    }

    #[test]
    fn single_phase_params_unchanged_by_multi() {
        // build_ctrl_params_multi must not change slope comp or other per-phase params.
        let mut p = test_params();
        let single = build_ctrl_params(&p).unwrap();

        p.num_phases = 3;
        let multi = build_ctrl_params_multi(&p).unwrap();

        assert_eq!(single.l_inductor, multi.l_inductor);
        assert_eq!(single.c_out, multi.c_out);
        assert_eq!(single.r_esr_out_cap, multi.r_esr_out_cap);
        assert_eq!(single.f_sw, multi.f_sw);
        assert_eq!(single.v_out, multi.v_out);
    }

    #[test]
    fn build_ctrl_params_multi_identity_for_n1() {
        let mut p = test_params();
        p.num_phases = 1;
        let single = build_ctrl_params(&p).unwrap();
        let multi = build_ctrl_params_multi(&p).unwrap();
        assert_eq!(single.current_sense_gain, multi.current_sense_gain);
    }

    // ── Multi-phase simulation stability ───────────────────────────────────

    #[test]
    fn two_phase_settles() {
        let mut p = test_params();
        p.num_phases = 2;
        let data = run_simulation(&p).expect("2-phase simulation should succeed");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn three_phase_settles() {
        let mut p = test_params();
        p.num_phases = 3;
        let data = run_simulation(&p).expect("3-phase simulation should succeed");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn six_phase_settles() {
        let mut p = test_params();
        p.num_phases = 6;
        let data = run_simulation(&p).expect("6-phase simulation should succeed");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn multi_phase_produces_n_phase_points() {
        for n in [2, 3, 4] {
            let mut p = test_params();
            p.num_phases = n;
            let data = run_simulation(&p).unwrap();
            for pt in &data {
                assert_eq!(
                    pt.phases.len(),
                    n,
                    "N={}: expected {} PhasePoints, got {}",
                    n,
                    n,
                    pt.phases.len(),
                );
            }
        }
    }

    // ── Multi-phase v_out must match single-phase (same operating point) ──

    #[test]
    fn multi_phase_same_steady_state_voltage() {
        let p1 = test_params();
        let data1 = run_simulation(&p1).unwrap();
        let v_out_1: f64 = data1[data1.len() - 200..].iter().map(|p| p.v_out as f64).sum::<f64>() / 200.0;

        for n in [2, 3] {
            let mut pn = test_params();
            pn.num_phases = n;
            let data_n = run_simulation(&pn).unwrap();
            let v_out_n: f64 = data_n[data_n.len() - 200..].iter().map(|p| p.v_out as f64).sum::<f64>() / 200.0;

            assert!(
                (v_out_n - v_out_1).abs() < 0.5,
                "N={}: steady-state v_out = {:.3} V, single-phase = {:.3} V (diff = {:.3})",
                n,
                v_out_n,
                v_out_1,
                (v_out_n - v_out_1).abs(),
            );
        }
    }

    // ── Interleaved envelope correctness ───────────────────────────────────

    #[test]
    fn interleaved_envelope_single_phase_passthrough() {
        let phase = PhasePoint {
            t_on: 0.5e-6,
            duty_pct: 25.0,
            i_l_min: 1.0,
            i_l_max: 3.0,
        };
        let (lo, hi) = interleaved_envelope(&[phase], 2e-6);
        assert_eq!(lo, 1.0);
        assert_eq!(hi, 3.0);
    }

    #[test]
    fn interleaved_envelope_two_phase_d50_cancellation() {
        // Two identical phases at D = 0.50 should give near-complete ripple
        // cancellation: the combined waveform is nearly flat.
        let period: f32 = 2e-6;
        let t_on = period * 0.5;
        let phase = PhasePoint {
            t_on,
            duty_pct: 50.0,
            i_l_min: 2.0,
            i_l_max: 4.0,
        };
        let (lo, hi) = interleaved_envelope(&[phase.clone(), phase], period);
        let ripple = hi - lo;
        let per_phase_ripple = 4.0 - 2.0;
        assert!(
            ripple < per_phase_ripple * 0.15,
            "2-phase D=0.5 ripple = {:.3}, expected near-zero (per-phase = {:.1})",
            ripple,
            per_phase_ripple,
        );
    }

    #[test]
    fn interleaved_envelope_three_phase_d33_cancellation() {
        // Three identical phases at D ≈ 1/3 → near-complete cancellation.
        let period: f32 = 2e-6;
        let t_on = period / 3.0;
        let phase = PhasePoint {
            t_on,
            duty_pct: 33.3,
            i_l_min: 2.0,
            i_l_max: 4.0,
        };
        let phases = vec![phase.clone(), phase.clone(), phase];
        let (lo, hi) = interleaved_envelope(&phases, period);
        let ripple = hi - lo;
        let per_phase_ripple = 4.0 - 2.0;
        assert!(
            ripple < per_phase_ripple * 0.15,
            "3-phase D=1/3 ripple = {:.3}, expected near-zero (per-phase = {:.1})",
            ripple,
            per_phase_ripple,
        );
    }

    #[test]
    fn interleaved_envelope_two_phase_reduces_ripple() {
        // For a generic duty cycle (not exactly 0.5), two phases should still
        // produce less combined ripple than a single phase.
        let period: f32 = 2e-6;
        let t_on = period * 0.3;
        let phase = PhasePoint {
            t_on,
            duty_pct: 30.0,
            i_l_min: 2.0,
            i_l_max: 4.0,
        };
        let (_, hi_1) = interleaved_envelope(&[phase.clone()], period);
        let (lo_1, _) = interleaved_envelope(&[phase.clone()], period);
        let ripple_1 = hi_1 - lo_1;

        let (lo_2, hi_2) = interleaved_envelope(&[phase.clone(), phase], period);
        let ripple_2 = hi_2 - lo_2;
        assert!(
            ripple_2 < ripple_1 * 2.0,
            "2-phase combined ripple ({:.3}) should be less than 2 × single-phase ({:.3})",
            ripple_2,
            ripple_1 * 2.0,
        );
    }

    #[test]
    fn interleaved_envelope_total_current_correct() {
        // The average of the interleaved envelope should be N × per-phase average.
        let period: f32 = 2e-6;
        let t_on = period * 0.4;
        let phase = PhasePoint {
            t_on,
            duty_pct: 40.0,
            i_l_min: 2.0,
            i_l_max: 4.0,
        };
        let per_phase_avg = (2.0 + 4.0) / 2.0;

        for n in [2, 3, 4] {
            let phases: Vec<PhasePoint> = (0..n).map(|_| phase.clone()).collect();
            let (lo, hi) = interleaved_envelope(&phases, period);
            let combined_mid = (lo + hi) / 2.0;
            let expected = per_phase_avg * n as f32;
            assert!(
                (combined_mid - expected).abs() < 0.3,
                "N={}: combined midpoint = {:.2}, expected {:.1}",
                n,
                combined_mid,
                expected,
            );
        }
    }

    // ── Load step transient with multi-phase ───────────────────────────────

    #[test]
    fn multi_phase_handles_load_step() {
        let mut p = test_params();
        p.r_loads = vec![6.0, 2.0, 12.0]; // nom → heavy → light
        p.num_phases = 3;
        let data = run_simulation(&p).expect("3-phase load-step sim should succeed");
        // After the last load step settles
        assert_settled(&data, 200, p.v_out_target, 1.0);
    }

    // ── Battery mode with multi-phase ──────────────────────────────────────

    #[test]
    fn multi_phase_battery_mode() {
        let mut p = test_params();
        p.load_kind = LoadKind::Battery;
        p.bat_v_init = 9.0;
        p.bat_r_int_mohm = 50.0;
        p.bat_c_mf = 10.0;
        p.num_phases = 2;
        let data = run_simulation(&p).expect("2-phase battery sim should succeed");
        // Battery OCV should rise over the simulation
        let first_bat = data.iter().find(|p| p.v_bat > 0.0).unwrap().v_bat;
        let last_bat = data.last().unwrap().v_bat;
        assert!(
            last_bat > first_bat,
            "Battery OCV should rise: first={:.2}, last={:.2}",
            first_bat,
            last_bat,
        );
        // No NaN/Inf in output
        for pt in &data {
            assert!(pt.v_out.is_finite(), "v_out is NaN/Inf in battery mode");
        }
    }

    // ── SimPoint convenience methods ───────────────────────────────────────

    #[test]
    fn sim_point_i_total_avg() {
        let pt = SimPoint {
            t_ms: 0.0,
            v_out: 12.0,
            phases: vec![
                PhasePoint { t_on: 0.5e-6, duty_pct: 25.0, i_l_min: 1.0, i_l_max: 3.0 },
                PhasePoint { t_on: 0.5e-6, duty_pct: 25.0, i_l_min: 2.0, i_l_max: 4.0 },
            ],
            i_total_min: 4.0,
            i_total_max: 6.0,
            v_out_min: 11.9,
            v_out_max: 12.1,
            v_bat: 0.0,
            v_in_cap: 0.0,
        };
        let avg = pt.i_total_avg();
        // phase0 avg = 2.0, phase1 avg = 3.0 → total = 5.0
        assert!((avg - 5.0).abs() < 1e-6);
    }

    #[test]
    fn sim_point_duty_pct_avg() {
        let pt = SimPoint {
            t_ms: 0.0,
            v_out: 12.0,
            phases: vec![
                PhasePoint { t_on: 0.5e-6, duty_pct: 30.0, i_l_min: 1.0, i_l_max: 3.0 },
                PhasePoint { t_on: 0.5e-6, duty_pct: 50.0, i_l_min: 2.0, i_l_max: 4.0 },
            ],
            i_total_min: 4.0,
            i_total_max: 6.0,
            v_out_min: 11.9,
            v_out_max: 12.1,
            v_bat: 0.0,
            v_in_cap: 0.0,
        };
        let avg = pt.duty_pct_avg();
        assert!((avg - 40.0).abs() < 1e-6);
    }

    // ── Serde backward compatibility ───────────────────────────────────────

    #[test]
    fn serde_defaults_for_new_fields() {
        // Old localStorage data without FET profiles or dcr_mohm should
        // deserialize with defaults (Ideal FETs, dcr_mohm = 4.1).
        let json = r#"{
            "v_in": 24.0, "v_out_target": 12.0, "f_sw_khz": 500.0,
            "l_uh": 4.0, "c_out_uf": 47.0, "r_esr_mohm": 10.0,
            "max_current": 10.0,
            "current_conduction": "Synchronous",
            "load_kind": "Steps", "r_loads": [6.0],
            "crossover_khz": 30.0, "cycles_per_tick": 1,
            "bat_v_init": 11.0, "bat_r_int_mohm": 50.0, "bat_c_mf": 20.0,
            "blanking_ns": 0.0, "adc_sample_ns": 0.0, "max_duty_pct": 100.0,
            "slope_overcomp": 1.5,
            "mcu": {"name":"Ideal","comp_delay_ns":0.0,"t_adc_us":0.0,"t_processing_us":0.0,"min_slope_steps_on_time":0},
            "cs": {"name":"Ideal","cs_gain_mv_a":66.0,"cs_bandwidth_khz":0.0},
            "dac": {"name":"Ideal","dac_filter_bw_khz":0.0,"t_dac_us":0.0}
        }"#;
        let p: SimParams = serde_json::from_str(json).expect("should deserialize");
        assert_eq!(p.num_phases, 1, "missing num_phases should default to 1");
        assert_eq!(p.hs_fet.name, "Ideal");
        assert_eq!(p.ls_fet.name, "Ideal");
        assert!((p.dcr_mohm - 4.1).abs() < 1e-6);
    }

    // ── Loss computation ──────────────────────────────────────────────────

    #[test]
    fn compute_losses_sanity() {
        let mut p = test_params();
        p.hs_fet = FetProfile::epc2306();
        p.ls_fet = FetProfile::epc2306();
        p.dcr_mohm = 4.1;
        let data = run_simulation(&p).expect("sim should succeed");
        let lb = compute_losses(&p, &data, None).expect("losses should compute");
        assert!(lb.total_loss_w > 0.0, "total loss should be positive");
        assert!(lb.p_out_w > 0.0, "output power should be positive");
        assert!(lb.efficiency_pct > 0.0 && lb.efficiency_pct < 100.0,
            "efficiency should be between 0 and 100, got {:.1}%", lb.efficiency_pct);
        assert!(lb.hs_conduction_w >= 0.0);
        assert!(lb.ls_conduction_w >= 0.0);
        assert!(lb.inductor_dcr_w >= 0.0);
        assert!(lb.hs_switching_w >= 0.0);
        assert!(lb.gate_drive_w >= 0.0);
    }

    #[test]
    fn compute_losses_ideal_fets_zero_switching() {
        let p = test_params(); // Ideal FETs
        let data = run_simulation(&p).expect("sim should succeed");
        let lb = compute_losses(&p, &data, None).expect("losses should compute");
        // Ideal FETs: no switching losses, no gate drive, no Rds(on) conduction
        assert!(lb.hs_conduction_w.abs() < 1e-12, "Ideal HS should have zero conduction loss");
        assert!(lb.ls_conduction_w.abs() < 1e-12, "Ideal LS should have zero conduction loss");
        assert!(lb.hs_switching_w.abs() < 1e-12, "Ideal should have zero switching loss");
        assert!(lb.gate_drive_w.abs() < 1e-12, "Ideal should have zero gate drive loss");
        // DCR loss should still exist (35 mΩ in test_params)
        assert!(lb.inductor_dcr_w > 0.0, "DCR loss should be nonzero");
    }

    // ── ADC quantization limit-cycling regression ──────────────────────────

    #[test]
    fn large_cout_sweep_crossover() {
        // Sweep crossover frequencies with C_out = 300 µF.
        // f_esr ≈ 53 kHz.  Low-crossover designs must be stable;
        // high-crossover designs near f_esr may be rejected by the
        // compensator design or produce an unstable simulation.
        for fx in [5.0, 10.0, 20.0, 30.0, 40.0] {
            let mut p = SimParams::default();
            p.c_out_uf = 300.0;
            p.crossover_khz = fx;

            let data = run_simulation(&p)
                .unwrap_or_else(|e| panic!("f_x={fx}kHz unexpectedly rejected: {e}"));
            let tail = &data[data.len() - 500..];
            let duty_min = tail.iter().map(|pt| pt.phases[0].duty_pct).fold(f32::MAX, f32::min);
            let duty_max = tail.iter().map(|pt| pt.phases[0].duty_pct).fold(f32::MIN, f32::max);
            let spread = duty_max - duty_min;
            assert!(
                spread < 10.0,
                "f_x={fx}kHz: duty oscillating (spread={spread:.1}%)"
            );
        }
    }

    #[test]
    fn large_cout_stable_with_lower_crossover() {
        // With C_out = 300 µF but crossover reduced to 30 kHz (well below
        // f_esr ≈ 53 kHz), the simulation should run and produce stable output.
        let mut p = SimParams::default();
        p.c_out_uf = 300.0;
        p.crossover_khz = 30.0;

        let data = run_simulation(&p).expect("Should run with lower crossover");
        let tail = &data[data.len() - 500..];
        let duty_min = tail.iter().map(|pt| pt.phases[0].duty_pct).fold(f32::MAX, f32::min);
        let duty_max = tail.iter().map(|pt| pt.phases[0].duty_pct).fold(f32::MIN, f32::max);
        assert!(
            duty_max - duty_min < 10.0,
            "Duty cycle oscillating at 30 kHz crossover: spread {:.1}%",
            duty_max - duty_min,
        );
    }

    // ── Input impedance stability ─────────────────────────────────────────

    #[test]
    fn input_rc_settles() {
        // Moderate cable resistance + input cap, no inductance (RC recharge path).
        let mut p = test_params();
        p.r_in_mohm = 100.0;  // 100 mΩ cable
        p.c_in_uf = 10.0;     // 10 µF input cap
        let data = run_simulation(&p).expect("RC input should not break sim");
        assert_settled(&data, 500, p.v_out_target, 0.5);
        // v_in_cap should be below v_in due to resistive droop
        let tail = &data[data.len() - 200..];
        for pt in tail {
            assert!(pt.v_in_cap > 0.0, "v_in_cap should be initialised");
            assert!(
                (pt.v_in_cap as f64) < p.v_in + 0.1,
                "v_in_cap ({:.2}) should not exceed v_in ({:.1})",
                pt.v_in_cap, p.v_in,
            );
        }
    }

    #[test]
    fn input_rlc_settles() {
        // Full RLC input filter: cable R + L with input decoupling cap.
        let mut p = test_params();
        p.r_in_mohm = 100.0;   // 100 mΩ cable
        p.l_in_nh = 500.0;     // 500 nH (~25 cm cable)
        p.c_in_uf = 10.0;      // 10 µF input cap
        let data = run_simulation(&p).expect("RLC input should not break sim");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn input_rlc_long_cable_settles() {
        // Long cable: higher R and L, larger input cap to compensate.
        let mut p = test_params();
        p.r_in_mohm = 500.0;   // 500 mΩ (long thin cable)
        p.l_in_nh = 2000.0;    // 2 µH (~1 m cable)
        p.c_in_uf = 100.0;     // 100 µF bulk input cap
        let data = run_simulation(&p).expect("long cable RLC should not break sim");
        assert_settled(&data, 500, p.v_out_target, 1.0);
    }

    #[test]
    fn input_impedance_zero_matches_ideal() {
        // With all input impedance at zero, v_in_cap should remain 0 (disabled).
        let p = test_params();
        assert_eq!(p.r_in_mohm, 0.0);
        assert_eq!(p.l_in_nh, 0.0);
        assert_eq!(p.c_in_uf, 0.0);
        let data = run_simulation(&p).expect("default params should work");
        for pt in &data {
            assert_eq!(pt.v_in_cap, 0.0, "v_in_cap should be 0 when input impedance disabled");
        }
    }

    // ── Cap bank tests ──────────────────────────────────────────────────────

    /// Helper: test_params but using a cap bank instead of scalar C/ESR.
    /// The bank is configured to match the scalar params exactly:
    /// one cap type with the same total C and ESR.
    fn test_params_single_cap_bank() -> SimParams {
        let mut p = test_params();
        p.output_caps = vec![CapTypeUi {
            c_uf: p.c_out_uf,
            count: 1,
            esr_mohm: p.r_esr_mohm,
            esl_nh: 0.0,
        }];
        p
    }

    /// Helper: test_params with a mixed ceramic + electrolytic cap bank.
    fn test_params_mixed_bank() -> SimParams {
        let mut p = test_params();
        p.output_caps = vec![
            // 6x 10µF ceramic, ESR = 3 mΩ, no ESL
            CapTypeUi { c_uf: 10.0, count: 6, esr_mohm: 3.0, esl_nh: 0.0 },
            // 2x 220µF electrolytic, ESR = 30 mΩ, ESL = 5 nH
            CapTypeUi { c_uf: 220.0, count: 2, esr_mohm: 30.0, esl_nh: 5.0 },
        ];
        // Use a lower crossover since the effective ESR is very low for ceramics
        p.crossover_khz = 20.0;
        p
    }

    #[test]
    fn cap_bank_single_type_settles() {
        // A single-cap-type bank should settle to the target voltage,
        // just like the scalar path does.
        let p = test_params_single_cap_bank();
        let data = run_simulation(&p).expect("single cap bank should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_single_type_handles_load_step() {
        // Load step with single cap bank should recover cleanly.
        let p = test_params_single_cap_bank();
        let data = run_simulation(&p).expect("single cap bank load step should work");
        // After initial settling (1500 soft-start + 1500 steady + 2000 step),
        // v_out should be near target.
        let last = data.last().unwrap();
        assert!(
            (last.v_out as f64 - p.v_out_target).abs() < 1.0,
            "v_out={}, expected ~{}", last.v_out, p.v_out_target
        );
    }

    #[test]
    fn cap_bank_mixed_ceramics_and_electrolytic_settles() {
        // Mixed bank: ceramics (low ESR, low C) + electrolytics (high ESR, high C, ESL).
        // This is the primary use case for the cap bank feature.
        let p = test_params_mixed_bank();
        let data = run_simulation(&p).expect("mixed cap bank should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_mixed_handles_load_step() {
        // Mixed bank should handle a load step without diverging.
        let p = test_params_mixed_bank();
        let data = run_simulation(&p).expect("mixed bank load step should work");
        let last = data.last().unwrap();
        assert!(
            (last.v_out as f64 - p.v_out_target).abs() < 1.0,
            "v_out={}, expected ~{}", last.v_out, p.v_out_target
        );
    }

    #[test]
    fn cap_bank_ceramics_only_settles() {
        // Pure ceramic bank (multiple caps, no ESL).
        let mut p = test_params();
        p.output_caps = vec![
            CapTypeUi { c_uf: 10.0, count: 6, esr_mohm: 3.0, esl_nh: 0.0 },
        ];
        p.crossover_khz = 20.0;
        let data = run_simulation(&p).expect("ceramics-only bank should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_many_types_settles() {
        // Three different cap types in parallel.
        let mut p = test_params();
        p.output_caps = vec![
            CapTypeUi { c_uf: 10.0, count: 6, esr_mohm: 3.0, esl_nh: 0.0 },
            CapTypeUi { c_uf: 100.0, count: 2, esr_mohm: 20.0, esl_nh: 3.0 },
            CapTypeUi { c_uf: 470.0, count: 1, esr_mohm: 50.0, esl_nh: 10.0 },
        ];
        p.crossover_khz = 15.0;
        let data = run_simulation(&p).expect("3-type bank should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_with_load_capacitance() {
        // Cap bank + extra load capacitance should still settle.
        let mut p = test_params_mixed_bank();
        p.c_load_uf = 100.0; // 100 µF extra load cap
        let data = run_simulation(&p).expect("cap bank + load cap should work");
        assert_settled(&data, 1000, p.v_out_target, 1.0);
    }

    #[test]
    fn cap_bank_with_input_impedance() {
        // Cap bank + input RC filter.
        let mut p = test_params_mixed_bank();
        p.r_in_mohm = 50.0;
        p.c_in_uf = 100.0;
        let data = run_simulation(&p).expect("cap bank + input impedance should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_multi_phase_settles() {
        // 2-phase converter with cap bank.
        let mut p = test_params_mixed_bank();
        p.num_phases = 2;
        let data = run_simulation(&p).expect("2-phase cap bank should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_multi_phase_three_settles() {
        // 3-phase converter with cap bank.
        let mut p = test_params_mixed_bank();
        p.num_phases = 3;
        let data = run_simulation(&p).expect("3-phase cap bank should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_effective_params_used_for_compensator() {
        // Verify that build_ctrl_params uses the cap bank's aggregate C and ESR
        // when output_caps is non-empty.
        let p = test_params_mixed_bank();
        let params = build_ctrl_params(&p).unwrap();

        // Expected total C: 6*10µF + 2*220µF = 500µF
        let expected_c = (6.0 * 10.0 + 2.0 * 220.0) * 1e-6;
        assert!(
            (params.c_out - expected_c).abs() / expected_c < 0.01,
            "c_out={}, expected={}", params.c_out, expected_c
        );

        // ESR should be the effective ESR at f_sw (not zero, not the scalar value).
        assert!(
            params.r_esr_out_cap > 0.0,
            "r_esr_out_cap should be positive, got {}", params.r_esr_out_cap
        );
        assert!(
            params.r_esr_out_cap < 0.01, // Should be in the mΩ range
            "r_esr_out_cap should be small (mΩ), got {} Ω", params.r_esr_out_cap
        );
    }

    #[test]
    fn cap_bank_diode_mode_dcm() {
        // Cap bank with diode mode should handle DCM correctly.
        let mut p = test_params_single_cap_bank();
        p.current_conduction = CurrentConduction::Diode;
        p.r_loads = vec![100.0]; // Light load → DCM likely
        let data = run_simulation(&p).expect("cap bank DCM should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_empty_uses_scalar_path() {
        // When output_caps is empty, the scalar c_out/r_esr path should be used
        // (identical to the existing behavior).
        let p = test_params();
        assert!(p.output_caps.is_empty());
        let data = run_simulation(&p).expect("scalar path should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_hallworth_params() {
        // Hallworth parameters with a cap bank matching the scalar values.
        let mut p = hallworth_params();
        p.output_caps = vec![CapTypeUi {
            c_uf: p.c_out_uf,
            count: 1,
            esr_mohm: p.r_esr_mohm,
            esl_nh: 0.0,
        }];
        let data = run_simulation(&p).expect("Hallworth cap bank should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_with_app_defaults() {
        // Reproduce: cap bank mode ON with the app's default SimParams.
        // The GUI seeds one cap type from the existing c_out/r_esr sliders.
        let mut p = SimParams::default();
        p.output_caps = vec![CapTypeUi {
            c_uf: p.c_out_uf,
            count: 1,
            esr_mohm: p.r_esr_mohm,
            esl_nh: 0.0,
        }];
        let data = run_simulation(&p).expect("cap bank with app defaults should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);

        // Compare cap bank vs scalar path — they should agree closely
        // since the cap bank has a single type matching the scalar params.
        let mut p_scalar = SimParams::default();
        p_scalar.output_caps.clear();
        let data_scalar = run_simulation(&p_scalar).expect("scalar defaults should work");
        // Steady-state v_out should match within a few percent
        let v_cb = data[SOFT_START_CYCLES + STEADY_STATE_CYCLES - 1].v_out;
        let v_sc = data_scalar[SOFT_START_CYCLES + STEADY_STATE_CYCLES - 1].v_out;
        assert!(
            (v_cb as f64 - v_sc as f64).abs() < 0.3,
            "cap bank v_out={v_cb:.3} vs scalar v_out={v_sc:.3} differ too much"
        );
    }

    #[test]
    #[ignore] // Run with --ignored to see diagnostic output
    fn cap_bank_vs_scalar_trajectory() {
        // Print side-by-side v_out for cap bank vs scalar at key points.
        let mut p_cb = SimParams::default();
        p_cb.output_caps = vec![CapTypeUi {
            c_uf: p_cb.c_out_uf,
            count: 1,
            esr_mohm: p_cb.r_esr_mohm,
            esl_nh: 0.0,
        }];
        let data_cb = run_simulation(&p_cb).expect("cap bank");

        let p_sc = SimParams::default();
        let data_sc = run_simulation(&p_sc).expect("scalar");

        eprintln!("\n{:>8} {:>10} {:>10} {:>10}", "cycle", "v_cb", "v_sc", "diff");
        for i in (0..data_cb.len()).step_by(100) {
            let v_cb = data_cb[i].v_out;
            let v_sc = data_sc[i].v_out;
            let diff = v_cb - v_sc;
            eprintln!("{:>8} {:>10.4} {:>10.4} {:>10.4}", i, v_cb, v_sc, diff);
        }

        // Check last 200 cycles: ripple should be similar magnitude
        let tail = 200;
        let cb_ripple: f32 = data_cb[data_cb.len()-tail..].iter()
            .map(|p| p.i_total_max - p.i_total_min)
            .fold(0.0_f32, f32::max);
        let sc_ripple: f32 = data_sc[data_sc.len()-tail..].iter()
            .map(|p| p.i_total_max - p.i_total_min)
            .fold(0.0_f32, f32::max);
        eprintln!("\nCap bank ripple I_pp: {cb_ripple:.4} A");
        eprintln!("Scalar   ripple I_pp: {sc_ripple:.4} A");
    }

    #[test]
    fn cap_bank_with_app_defaults_2phase() {
        let mut p = SimParams::default();
        p.num_phases = 2;
        p.output_caps = vec![CapTypeUi {
            c_uf: p.c_out_uf,
            count: 1,
            esr_mohm: p.r_esr_mohm,
            esl_nh: 0.0,
        }];
        let data = run_simulation(&p).expect("cap bank 2-phase defaults should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_hw_params_single_phase() {
        // Test with realistic hardware params from CLAUDE.md:
        // 6× GRM188R60J106ME47D, 7.7µF@12V each, ESR=3mΩ
        let mut p = SimParams::default();
        p.output_caps = vec![CapTypeUi {
            c_uf: 7.7,
            count: 6,
            esr_mohm: 3.0,
            esl_nh: 0.0,
        }];
        let data = run_simulation(&p).expect("hw cap bank 1-phase should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_hw_params_two_phase() {
        let mut p = SimParams::default();
        p.num_phases = 2;
        p.output_caps = vec![CapTypeUi {
            c_uf: 7.7,
            count: 6,
            esr_mohm: 3.0,
            esl_nh: 0.0,
        }];
        let data = run_simulation(&p).expect("hw cap bank 2-phase should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_low_esr_ceramics() {
        // Very low ESR ceramics — minimal damping, tests numerical stability.
        let mut p = SimParams::default();
        p.output_caps = vec![CapTypeUi {
            c_uf: 10.0,
            count: 10,
            esr_mohm: 1.0, // 1mΩ per unit → 0.1mΩ parallel
            esl_nh: 0.0,
        }];
        p.crossover_khz = 30.0; // lower crossover for low-ESR stability
        let data = run_simulation(&p).expect("low ESR cap bank should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn voltage_ripple_scalar_path() {
        // Verify v_out_min/v_out_max are sensible in the scalar path.
        let p = SimParams::default();
        let data = run_simulation(&p).expect("scalar sim");
        // Check settled cycles
        let tail = &data[data.len() - 200..];
        for pt in tail {
            assert!(pt.v_out_max >= pt.v_out_min,
                "v_out_max ({}) < v_out_min ({})", pt.v_out_max, pt.v_out_min);
            let vpp = pt.v_out_max - pt.v_out_min;
            assert!(vpp > 0.0, "voltage ripple should be non-zero at steady state");
            assert!(vpp < 1.0, "voltage ripple should be well under 1V, got {:.4}", vpp);
        }
        let worst_vpp: f32 = tail.iter()
            .map(|p| p.v_out_max - p.v_out_min)
            .fold(0.0_f32, f32::max);
        eprintln!("Scalar voltage ripple V_pp: {:.2} mV", worst_vpp * 1e3);
    }

    #[test]
    fn voltage_ripple_cap_bank_path() {
        // Verify v_out_min/v_out_max are sensible in the cap bank path.
        let mut p = SimParams::default();
        p.output_caps = vec![CapTypeUi {
            c_uf: p.c_out_uf,
            count: 1,
            esr_mohm: p.r_esr_mohm,
            esl_nh: 0.0,
        }];
        let data = run_simulation(&p).expect("cap bank sim");
        let tail = &data[data.len() - 200..];
        for pt in tail {
            assert!(pt.v_out_max >= pt.v_out_min,
                "v_out_max ({}) < v_out_min ({})", pt.v_out_max, pt.v_out_min);
            let vpp = pt.v_out_max - pt.v_out_min;
            assert!(vpp > 0.0, "voltage ripple should be non-zero at steady state");
            assert!(vpp < 1.0, "voltage ripple should be well under 1V, got {:.4}", vpp);
        }
        let worst_vpp: f32 = tail.iter()
            .map(|p| p.v_out_max - p.v_out_min)
            .fold(0.0_f32, f32::max);
        eprintln!("Cap bank voltage ripple V_pp: {:.2} mV", worst_vpp * 1e3);
    }

    #[test]
    fn cap_bank_three_mixed_types() {
        // Repro: default settings + cap bank with 3 types → was producing all zeros
        let mut p = SimParams::default();
        p.output_caps = vec![
            CapTypeUi { c_uf: 47.0, count: 1, esr_mohm: 10.0, esl_nh: 0.0 },
            CapTypeUi { c_uf: 1.0,  count: 1, esr_mohm: 1.0,  esl_nh: 0.0 },
            CapTypeUi { c_uf: 10.0, count: 1, esr_mohm: 3.0,  esl_nh: 0.0 },
        ];
        let data = run_simulation(&p).expect("three-type cap bank should work");
        assert_settled(&data, 500, p.v_out_target, 0.5);
    }

    #[test]
    fn cap_bank_extreme_fast_tau_rejected() {
        // A cap with τ = ESR×C = 0.1mΩ × 0.1µF = 0.01ns would need ~100k steps/phase.
        // The budget check should reject this before running.
        let mut p = SimParams::default();
        p.output_caps = vec![
            CapTypeUi { c_uf: 47.0, count: 1, esr_mohm: 10.0, esl_nh: 0.0 },
            CapTypeUi { c_uf: 0.1,  count: 1, esr_mohm: 0.1,  esl_nh: 0.0 },
        ];
        let err = run_simulation(&p).unwrap_err();
        assert!(err.contains("too expensive"), "expected budget error, got: {err}");
    }
}
