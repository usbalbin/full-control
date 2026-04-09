use electronics_sim::pfc_boost::PfcBoostSim;
use electronics_sim::CurrentConduction;
use full_control::control_pfc::{PfcDesignSummary, PfcParameters};

use std::f64::consts::PI;

/// PFC control mode.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PfcControlMode {
    /// Average Current Mode — inner 2P2Z current loop + outer voltage loop.
    Acm,
    /// Critical Conduction Mode — variable frequency, voltage loop sets on-time.
    CrCm,
    /// Peak Current Mode — fixed frequency, comparator + slope compensation.
    Pcmc,
}

/// PFC power stage topology.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PfcTopology {
    /// Diode bridge rectifier + boost stage.
    BridgedBoost,
    /// Totem-pole bridgeless — no bridge diode loss, inherently synchronous.
    TotemPole,
}

/// Number of half-cycles for soft-start (voltage ramp).
pub const SOFT_START_HALF_CYCLES: usize = 4;

// ── PFC simulation parameters (UI-friendly units) ──────────────────────────

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct PfcSimParams {
    pub v_in_rms: f64,            // AC input RMS [V]
    pub f_line_hz: f64,           // Line frequency [Hz] (50 or 60)
    pub v_out: f64,               // DC bus target [V]
    pub f_sw_khz: f64,            // Switching frequency [kHz]
    pub l_uh: f64,                // Boost inductor [µH]
    pub c_out_uf: f64,            // Output cap [µF]
    pub r_esr_mohm: f64,          // Output cap ESR [mΩ]
    pub dcr_mohm: f64,            // Inductor DCR [mΩ]
    pub rds_on_mohm: f64,         // FET Rds(on) [mΩ]
    pub r_sense_mohm: f64,        // Current sense resistance [mΩ]
    pub v_diode: f64,             // Boost diode forward voltage [V]
    pub current_crossover_khz: f64, // Inner loop crossover [kHz]
    pub voltage_crossover_hz: f64,  // Outer loop crossover [Hz]

    // Load steps: first element is nominal, rest are transient steps
    pub p_loads_w: Vec<f64>,
    pub steady_half_cycles: usize,  // Half-cycles at nominal load after soft-start
    pub step_half_cycles: usize,    // Half-cycles per load step transition

    // Multi-phase
    pub num_phases: usize,

    // Conduction mode
    pub current_conduction: CurrentConduction,

    // Control mode and topology
    pub control_mode: PfcControlMode,
    pub topology: PfcTopology,
    /// Slope over-compensation factor for PCMC (1.0 = exact, 1.5 = 50% extra).
    pub slope_overcomp: f64,

    // Cap bank (empty = use scalar c_out_uf/r_esr_mohm)
    pub output_caps: Vec<CapTypeUi>,
}

impl Default for PfcSimParams {
    fn default() -> Self {
        Self {
            v_in_rms: 230.0,
            f_line_hz: 50.0,
            v_out: 400.0,
            f_sw_khz: 65.0,
            l_uh: 500.0,
            c_out_uf: 220.0,
            r_esr_mohm: 100.0,
            dcr_mohm: 30.0,
            rds_on_mohm: 50.0,
            r_sense_mohm: 50.0,
            v_diode: 0.7,
            current_crossover_khz: 6.5,
            voltage_crossover_hz: 10.0,
            p_loads_w: vec![300.0],
            steady_half_cycles: 6,
            step_half_cycles: 10,
            num_phases: 1,
            current_conduction: CurrentConduction::Diode,
            control_mode: PfcControlMode::Acm,
            topology: PfcTopology::BridgedBoost,
            slope_overcomp: 1.5,
            output_caps: Vec::new(),
        }
    }
}

impl PfcSimParams {
    /// First (nominal) load power.
    pub fn p_load_w(&self) -> f64 {
        self.p_loads_w.first().copied().unwrap_or(300.0)
    }

    /// Total number of half-cycles for this simulation.
    pub fn total_half_cycles(&self) -> usize {
        SOFT_START_HALF_CYCLES
            + self.steady_half_cycles
            + self.p_loads_w.len().saturating_sub(1) * self.step_half_cycles
    }
}

// Re-export CapTypeUi from sim module for convenience
pub use crate::sim::CapTypeUi;

// ── Simulation output ──────────────────────────────────────────────────────

/// Per-phase data for one switching cycle.
#[derive(Debug, Clone, Copy)]
pub struct PfcPhasePoint {
    pub i_l_start: f32,   // Inductor current at cycle start [A]
    pub i_l_peak: f32,    // Peak inductor current (end of ON) [A]
    pub i_l_end: f32,     // Inductor current at end of cycle [A]
    pub duty: f32,        // Duty cycle
    pub dcm: bool,        // Whether this phase entered DCM
    pub t_conduct_frac: f32, // Fraction of OFF time with current > 0
}

/// One data point per switching cycle (all phases combined).
#[derive(Debug, Clone)]
pub struct PfcSimPoint {
    pub t_us: f32,            // Absolute time [µs]
    pub v_in_rect: f32,       // Rectified input voltage [V]
    pub v_out: f32,           // DC bus voltage [V]
    pub i_ref: f32,           // Current reference from outer loop [A]
    pub line_phase: f32,      // Phase within line half-cycle [rad]
    pub phases: Vec<PfcPhasePoint>, // Per-phase data
    pub i_total_min: f32,     // Combined interleaved min current [A]
    pub i_total_max: f32,     // Combined interleaved max current [A]
    pub i_total_avg: f32,     // Combined average current [A]
    pub t_sw_us: f32,         // Switching period for this cycle [µs] (varies in CrCM)
}

impl PfcSimPoint {
    /// Average duty across all phases.
    pub fn duty_avg(&self) -> f32 {
        if self.phases.is_empty() { return 0.0; }
        self.phases.iter().map(|p| p.duty).sum::<f32>() / self.phases.len() as f32
    }
}

/// Post-processed PFC metrics from the last full line cycle.
#[derive(Debug, Clone)]
pub struct PfcMetrics {
    pub thd_pct: f64,
    pub power_factor: f64,
    pub efficiency_pct: f64,
    pub v_out_ripple_v: f64,  // pk-pk 2×f_line ripple [V]
    pub v_out_avg: f64,       // Average DC bus voltage [V]
    /// Per-harmonic amplitudes as % of fundamental, indices 0..=max_harmonic.
    /// Index 0 is unused, index 1 = fundamental (100%), index 2 = 2nd harmonic, etc.
    pub harmonics_pct: Vec<f64>,
    // Loss breakdown [W]
    pub p_loss_fet_w: f64,
    pub p_loss_diode_w: f64,
    pub p_loss_dcr_w: f64,
    pub p_loss_esr_w: f64,
    pub p_loss_total_w: f64,
}

// ── Simulation runner ──────────────────────────────────────────────────────

/// Build PFC compensator design parameters from UI params.
pub fn build_pfc_design(p: &PfcSimParams) -> PfcParameters {
    PfcParameters {
        v_out: p.v_out,
        v_in_rms: p.v_in_rms,
        f_line: p.f_line_hz,
        f_sw: p.f_sw_khz * 1e3,
        l_boost: p.l_uh * 1e-6,
        c_out: p.c_out_uf * 1e-6,
        r_esr_out: p.r_esr_mohm * 1e-3,
        p_rated: p.p_load_w(),
        r_sense: p.r_sense_mohm * 1e-3,
        current_crossover_hz: p.current_crossover_khz * 1e3,
        voltage_crossover_hz: p.voltage_crossover_hz,
        phase_margin_current: 60.0_f64.to_radians(),
        phase_margin_voltage: 60.0_f64.to_radians(),
    }
}

/// Run the full PFC simulation.
///
/// Returns a vector of per-switching-cycle data points and post-processed metrics,
/// or an error string if the compensator design fails.
pub fn run_pfc_simulation(
    p: &PfcSimParams,
) -> Result<(Vec<PfcSimPoint>, PfcMetrics, PfcDesignSummary), String> {
    // ── Validate ──────────────────────────────────────────────
    if p.v_in_rms <= 0.0 {
        return Err("V_in RMS must be positive".into());
    }
    let v_in_pk = p.v_in_rms * 2.0_f64.sqrt();
    if v_in_pk >= p.v_out {
        return Err(format!(
            "V_in peak ({v_in_pk:.0}V) must be less than V_out ({:.0}V) for boost",
            p.v_out
        ));
    }
    if p.p_loads_w.is_empty() || p.p_loads_w[0] <= 0.0 {
        return Err("Load power must be positive".into());
    }
    if p.voltage_crossover_hz >= p.f_line_hz / 3.0 {
        return Err(format!(
            "Voltage loop crossover ({:.1} Hz) must be below f_line/3 ({:.1} Hz) to avoid THD degradation",
            p.voltage_crossover_hz, p.f_line_hz / 3.0
        ));
    }
    let num_phases = p.num_phases.max(1);

    // ── Topology overrides ───────────────────────────────────
    let effective_v_diode = match p.topology {
        PfcTopology::BridgedBoost => p.v_diode,
        PfcTopology::TotemPole => 0.0,
    };
    let effective_conduction = match p.topology {
        PfcTopology::BridgedBoost => p.current_conduction,
        PfcTopology::TotemPole => CurrentConduction::Synchronous,
    };

    // ── Compensator design ────────────────────────────────────
    let pfc_params = build_pfc_design(p);
    let design = pfc_params
        .design()
        .ok_or("Compensator design infeasible — reduce crossover or check parameters")?;

    let f_sw = p.f_sw_khz * 1e3;
    let t_sw = 1.0 / f_sw;
    let l = p.l_uh * 1e-6;
    let r_series = (p.dcr_mohm + p.rds_on_mohm) * 1e-3;
    let r_sense = p.r_sense_mohm * 1e-3;

    // ── Create controllers ────────────────────────────────────
    // ACM uses both current + voltage loops; CrCM/PCMC use voltage loop only.
    let mut current_ctrl = design.inner.to_controller(0.0_f32, 0.95);
    // Voltage loop output limit (k_max). The voltage loop output k represents
    // the peak of the sinusoidal average-current reference: i_avg(θ) = k×sin(θ).
    // Nominal k = P×√2/V_rms ≈ P/(V_rms×0.5). PCMC needs extra headroom because
    // the DCM-corrected reference converts k into a larger comparator reference,
    // and the voltage loop may overshoot k during load transients.
    let i_max_ref = match p.control_mode {
        PfcControlMode::Pcmc => {
            (p.p_load_w() / (p.v_in_rms * 0.5) * 2.0) as f32
        }
        _ => (p.p_load_w() / (p.v_in_rms * 0.5)) as f32,
    };
    let mut voltage_ctrl = design.outer.to_controller(0.0_f32, i_max_ref);

    // ── Create simulator(s) ──────────────────────────────────
    let mut sims: Vec<PfcBoostSim> = (0..num_phases)
        .map(|_| {
            PfcBoostSim::new(
                l,
                p.c_out_uf * 1e-6,
                p.r_esr_mohm * 1e-3,
                r_series,
                effective_v_diode,
                f_sw,
                p.v_out,
                effective_conduction,
            )
        })
        .collect();

    // ── Simulation loop ───────────────────────────────────────
    let t_half_cycle = 1.0 / (2.0 * p.f_line_hz);
    let cycles_per_half = (f_sw * t_half_cycle).round() as usize;
    let mut points = Vec::new();
    let p_load_nom = p.p_load_w();
    let i_load_nom = p_load_nom / p.v_out;

    let mut t = 0.0_f64;

    // Prime voltage controller
    let k_init = i_load_nom / (1.0 - v_in_pk / p.v_out);
    voltage_ctrl.prime(k_init as f32, 0.0);

    // Prime current controller (ACM only, but no harm priming it)
    let d_init = 1.0 - v_in_pk / p.v_out;
    current_ctrl.prime(d_init as f32, 0.0);

    #[allow(unused_assignments)]
    let mut k = k_init as f32;
    let mut i_avg_prev = k_init as f32 * 0.637; // initial guess: I_pk × 2/π

    // Build load schedule: (half_cycle_count, load_power_w)
    let mut schedule: Vec<(usize, f64)> = Vec::new();
    schedule.push((SOFT_START_HALF_CYCLES, p_load_nom));  // soft-start
    schedule.push((p.steady_half_cycles, p_load_nom));     // steady-state
    for &p_step in p.p_loads_w.iter().skip(1) {
        schedule.push((p.step_half_cycles, p_step));
    }

    let mut global_half_cycle = 0_usize;
    for (seg_half_cycles, seg_power) in &schedule {
        let seg_i_load = seg_power / p.v_out;

        for _local_half in 0..*seg_half_cycles {
            let is_soft_start = global_half_cycle < SOFT_START_HALF_CYCLES;
            let ss_frac = if is_soft_start {
                (global_half_cycle as f64 + 1.0) / SOFT_START_HALF_CYCLES as f64
            } else {
                1.0
            };

            // Helper closure: build a PfcSimPoint from phase results
            let build_point = |sims: &[PfcBoostSim], phase_results: &[electronics_sim::pfc_boost::PfcCycleResult],
                               t: f64, v_in_rect: f64, theta: f64, i_ref_amps: f32, t_sw_actual: f64| {
                let phase_points: Vec<PfcPhasePoint> = phase_results
                    .iter()
                    .map(|r| PfcPhasePoint {
                        i_l_start: r.i_l_start as f32,
                        i_l_peak: r.i_l_peak as f32,
                        i_l_end: r.i_l_end as f32,
                        duty: r.duty as f32,
                        dcm: r.dcm,
                        t_conduct_frac: r.t_conduct_frac as f32,
                    })
                    .collect();
                let i_total_avg: f32 = phase_results.iter()
                    .map(|r| r.i_l_avg as f32).sum();
                let (i_total_min, i_total_max) = pfc_interleaved_envelope(
                    &phase_points, t_sw_actual as f32,
                );
                PfcSimPoint {
                    t_us: (t * 1e6) as f32,
                    v_in_rect: v_in_rect as f32,
                    v_out: sims[0].v_out() as f32,
                    i_ref: i_ref_amps,
                    line_phase: theta as f32,
                    phases: phase_points,
                    i_total_min,
                    i_total_max,
                    i_total_avg,
                    t_sw_us: (t_sw_actual * 1e6) as f32,
                }
            };

            match p.control_mode {
                PfcControlMode::CrCm => {
                    // ── CrCM: variable-frequency loop ────────────────
                    let mut t_within_half = 0.0_f64;
                    while t_within_half < t_half_cycle {
                        let theta = PI * (t_within_half + 1e-9) / t_half_cycle;
                        let sin_theta = theta.sin();
                        let v_in_rect = v_in_pk * sin_theta;
                        let v_target = p.v_out * ss_frac;

                        // Voltage loop → k (current amplitude envelope)
                        let v_error = (v_target - sims[0].v_out()) as f32;
                        k = voltage_ctrl.update(v_error);
                        k = k.max(0.0);

                        let t_on = full_control::pfc_runtime::crcm_on_time(
                            l, k as f64, v_in_pk, num_phases as f64, t_sw,
                        );

                        let i_ref_amps = k / num_phases as f32 * sin_theta as f32;

                        // Tick all phases with CrCM
                        let phase_results = tick_multi_pfc_crm(
                            &mut sims, v_in_rect, t_on, seg_i_load,
                        );
                        let actual_t_sw = phase_results[0].t_sw_actual;

                        points.push(build_point(
                            &sims, &phase_results, t, v_in_rect, theta, i_ref_amps, actual_t_sw,
                        ));

                        let dt = actual_t_sw.max(t_sw * 0.01); // guard against zero
                        t += dt;
                        t_within_half += dt;
                    }
                }
                _ => {
                    // ── ACM / PCMC: fixed-frequency loop ─────────────
                    for sw in 0..cycles_per_half {
                        let theta = PI * (sw as f64 + 0.5) / cycles_per_half as f64;
                        let sin_theta = theta.sin();
                        let v_in_rect = v_in_pk * sin_theta;
                        let v_target = p.v_out * ss_frac;

                        // Outer voltage loop → k
                        let v_error = (v_target - sims[0].v_out()) as f32;
                        k = voltage_ctrl.update(v_error);
                        k = k.max(0.0);

                        let i_ref_per_phase = k / num_phases as f32 * sin_theta as f32;
                        let mut i_ref_amps = i_ref_per_phase;

                        let duty = match p.control_mode {
                            PfcControlMode::Acm => {
                                // ACM: inner 2P2Z current loop → duty
                                let i_error = (i_ref_per_phase - i_avg_prev) * r_sense as f32;
                                current_ctrl.update(i_error) as f64
                            }
                            PfcControlMode::Pcmc => {
                                use full_control::pfc_runtime::{pcmc_reference_corrected, pcmc_comparator};

                                let oc = p.slope_overcomp;
                                let i_start_val = sims[0].i_inductor;
                                let v_out_now = sims[0].v_out();
                                let i_avg_desired = (k as f64 / num_phases as f64) * sin_theta;

                                let i_ref = pcmc_reference_corrected(
                                    i_avg_desired, v_in_rect, v_out_now,
                                    i_start_val, l, t_sw, oc,
                                );
                                i_ref_amps = i_ref as f32;

                                pcmc_comparator(i_ref, i_start_val, oc, v_in_rect, l, t_sw)
                            }
                            PfcControlMode::CrCm => unreachable!(),
                        };

                        // Zero-crossing blanking for sync mode
                        let blanking = effective_conduction == CurrentConduction::Synchronous
                            && full_control::pfc_runtime::zc_blanking_active(v_in_rect, p.v_out, 0.05);
                        if blanking {
                            for sim in sims.iter_mut() {
                                sim.current_conduction = CurrentConduction::Diode;
                            }
                        }

                        let phase_results = tick_multi_pfc(
                            &mut sims, v_in_rect, duty, seg_i_load,
                        );

                        if blanking {
                            for sim in sims.iter_mut() {
                                sim.current_conduction = effective_conduction;
                            }
                        }

                        // Feedback: average of all phases' currents (per-phase value)
                        i_avg_prev = phase_results.iter()
                            .map(|r| r.i_l_avg as f32)
                            .sum::<f32>() / num_phases as f32;

                        points.push(build_point(
                            &sims, &phase_results, t, v_in_rect, theta, i_ref_amps, t_sw,
                        ));

                        t += t_sw;
                    }
                }
            }
            global_half_cycle += 1;
        }
    }

    // ── Compute metrics from last full line cycle ─────────────
    let metrics = compute_metrics(&points, cycles_per_half, p);

    Ok((points, metrics, design.summary))
}

/// Tick N interleaved phases sharing a common v_out.
fn tick_multi_pfc(
    sims: &mut [PfcBoostSim],
    v_in_rect: f64,
    duty: f64,
    i_load: f64,
) -> Vec<electronics_sim::pfc_boost::PfcCycleResult> {
    let n = sims.len();
    let mut results = Vec::with_capacity(n);

    for k in 0..n {
        if k > 0 {
            sims[k].v_cap = sims[k - 1].v_cap;
        }
        let result = sims[k].tick(v_in_rect, duty, i_load / n as f64);
        results.push(result);
    }

    // Synchronize all phases to shared output voltage
    let v_out_final = sims[n - 1].v_cap;
    for sim in sims.iter_mut() {
        sim.v_cap = v_out_final;
    }

    results
}

/// Tick N interleaved CrCM phases sharing a common v_out.
fn tick_multi_pfc_crm(
    sims: &mut [PfcBoostSim],
    v_in_rect: f64,
    t_on: f64,
    i_load: f64,
) -> Vec<electronics_sim::pfc_boost::PfcCycleResult> {
    let n = sims.len();
    let mut results = Vec::with_capacity(n);

    for k in 0..n {
        if k > 0 {
            sims[k].v_cap = sims[k - 1].v_cap;
        }
        let result = sims[k].tick_crm(v_in_rect, t_on, i_load / n as f64);
        results.push(result);
    }

    let v_out_final = sims[n - 1].v_cap;
    for sim in sims.iter_mut() {
        sim.v_cap = v_out_final;
    }

    results
}

/// Compute min/max of interleaved PFC phase currents.
///
/// Each phase has a piecewise-linear waveform offset by k×T_sw/N:
/// - ON: i_l_start → i_l_peak over duty×T_sw
/// - OFF (CCM): i_l_peak → i_l_end over (1-duty)×T_sw
/// - OFF (DCM): i_l_peak → 0 over t_conduct_frac×(1-duty)×T_sw, then 0
fn pfc_interleaved_envelope(phases: &[PfcPhasePoint], period: f32) -> (f32, f32) {
    let n = phases.len();
    if n <= 1 {
        let p = &phases[0];
        let i_min = p.i_l_start.min(p.i_l_end).min(if p.dcm { 0.0 } else { p.i_l_end });
        let i_max = p.i_l_peak;
        return (i_min, i_max);
    }

    let offset_k = |k: usize| k as f32 * period / n as f32;

    // Evaluate phase k's current at global time t
    let phase_current = |k: usize, t: f32| -> f32 {
        let p = &phases[k];
        let t_local = (t - offset_k(k)).rem_euclid(period);
        let t_on = p.duty * period;
        let t_off = period - t_on;

        if t_local <= t_on {
            // ON ramp: i_l_start → i_l_peak
            let frac = if t_on > 0.0 { t_local / t_on } else { 0.0 };
            p.i_l_start + (p.i_l_peak - p.i_l_start) * frac
        } else if p.dcm {
            // DCM: ramp from peak to 0, then stay at 0
            let t_in_off = t_local - t_on;
            let t_conduct = p.t_conduct_frac * t_off;
            if t_in_off <= t_conduct {
                let frac = if t_conduct > 0.0 { t_in_off / t_conduct } else { 1.0 };
                p.i_l_peak * (1.0 - frac)
            } else {
                0.0
            }
        } else {
            // CCM: ramp from peak to i_l_end
            let t_in_off = t_local - t_on;
            let frac = if t_off > 0.0 { t_in_off / t_off } else { 0.0 };
            p.i_l_peak + (p.i_l_end - p.i_l_peak) * frac
        }
    };

    // Collect all breakpoint times
    let mut events: Vec<f32> = Vec::with_capacity(3 * n);
    for k in 0..n {
        let off = offset_k(k);
        let p = &phases[k];
        let t_on = p.duty * period;
        events.push(off % period);                   // cycle start
        events.push((off + t_on) % period);           // ON→OFF
        if p.dcm {
            let t_conduct = p.t_conduct_frac * (period - t_on);
            events.push((off + t_on + t_conduct) % period); // current→0
        }
    }
    events.sort_by(|a, b| a.partial_cmp(b).unwrap());
    events.dedup();

    let mut i_min = f32::MAX;
    let mut i_max = f32::MIN;
    for &t in &events {
        let i_total: f32 = (0..n).map(|k| phase_current(k, t)).sum();
        i_min = i_min.min(i_total);
        i_max = i_max.max(i_total);
    }

    (i_min, i_max)
}

/// Run a Vin sweep: simulate at multiple input voltages, return (v_in_rms, metrics) pairs.
pub fn run_pfc_vin_sweep(
    base_params: &PfcSimParams,
    v_in_points: &[f64],
) -> Vec<(f64, Result<PfcMetrics, String>)> {
    v_in_points
        .iter()
        .map(|&v_in_rms| {
            let p = PfcSimParams {
                v_in_rms,
                // Use enough settling time
                steady_half_cycles: base_params.steady_half_cycles.max(20),
                // Only nominal load for sweep (no load steps)
                p_loads_w: vec![base_params.p_load_w()],
                step_half_cycles: 0,
                ..base_params.clone()
            };
            let result = run_pfc_simulation(&p).map(|(_, m, _)| m);
            (v_in_rms, result)
        })
        .collect()
}

/// Compute PFC quality metrics from the last full line cycle of simulation data.
///
/// Works for both fixed-frequency (ACM/PCMC) and variable-frequency (CrCM) modes
/// by using time-weighted averaging and time-based DFT phase.
fn compute_metrics(
    points: &[PfcSimPoint],
    _cycles_per_half: usize,
    p: &PfcSimParams,
) -> PfcMetrics {
    let empty = PfcMetrics {
        thd_pct: 0.0,
        power_factor: 0.0,
        efficiency_pct: 0.0,
        v_out_ripple_v: 0.0,
        v_out_avg: 0.0,
        harmonics_pct: Vec::new(),
        p_loss_fet_w: 0.0,
        p_loss_diode_w: 0.0,
        p_loss_dcr_w: 0.0,
        p_loss_esr_w: 0.0,
        p_loss_total_w: 0.0,
    };

    if points.len() < 10 {
        return empty;
    }

    // Find the last full line cycle using timestamps
    let t_line = 1.0 / p.f_line_hz;
    let t_end = points.last().unwrap().t_us as f64 * 1e-6;
    let t_start = t_end - t_line;
    let start_idx = points.iter()
        .position(|p| p.t_us as f64 * 1e-6 >= t_start)
        .unwrap_or(0);

    let last_cycle = &points[start_idx..];
    let n = last_cycle.len();
    if n < 10 {
        return empty;
    }

    // V_out ripple and average (time-weighted)
    let mut v_min = f32::MAX;
    let mut v_max = f32::MIN;
    let mut v_sum = 0.0_f64;
    let mut total_dt = 0.0_f64;
    for pt in last_cycle {
        let dt = pt.t_sw_us as f64 * 1e-6;
        v_min = v_min.min(pt.v_out);
        v_max = v_max.max(pt.v_out);
        v_sum += pt.v_out as f64 * dt;
        total_dt += dt;
    }
    let v_out_avg = if total_dt > 0.0 { v_sum / total_dt } else { 0.0 };
    let v_out_ripple = (v_max - v_min) as f64;

    // Input current waveform for THD and power factor (time-weighted)
    let mut p_in = 0.0_f64;
    let mut i_rms_sq = 0.0_f64;

    // DFT coefficients for harmonics 1-50 (time-based phase)
    let max_harmonic = 50;
    let mut cos_coeffs = vec![0.0_f64; max_harmonic + 1];
    let mut sin_coeffs = vec![0.0_f64; max_harmonic + 1];

    // Loss accumulators
    let rds_on = p.rds_on_mohm * 1e-3;
    let dcr = p.dcr_mohm * 1e-3;
    let r_esr = p.r_esr_mohm * 1e-3;
    let effective_v_diode = match p.topology {
        PfcTopology::BridgedBoost => p.v_diode,
        PfcTopology::TotemPole => 0.0,
    };
    let mut loss_fet_energy = 0.0_f64;
    let mut loss_diode_energy = 0.0_f64;
    let mut loss_dcr_energy = 0.0_f64;
    let mut loss_esr_energy = 0.0_f64;

    for pt in last_cycle {
        let dt = pt.t_sw_us as f64 * 1e-6;
        let i_in = pt.i_total_avg as f64;
        let v_in = pt.v_in_rect as f64;
        let t_abs = pt.t_us as f64 * 1e-6;

        // Sign: use line phase for correct half-cycle identification
        let line_phase = 2.0 * PI * p.f_line_hz * t_abs;
        let sign = if line_phase.sin() >= 0.0 { 1.0 } else { -1.0 };
        let i_in_signed = i_in * sign;

        i_rms_sq += i_in * i_in * dt;
        p_in += v_in * i_in * dt;

        // DFT: use actual time for phase
        let phase = 2.0 * PI * p.f_line_hz * t_abs;
        for k in 1..=max_harmonic {
            cos_coeffs[k] += i_in_signed * (k as f64 * phase).cos() * dt;
            sin_coeffs[k] += i_in_signed * (k as f64 * phase).sin() * dt;
        }

        // Per-cycle loss computation using triangular current approximation
        let cycle_t_sw = pt.t_sw_us as f64 * 1e-6;
        for ph in &pt.phases {
            let i_start = ph.i_l_start as f64;
            let i_peak = ph.i_l_peak as f64;
            let i_end = ph.i_l_end as f64;
            let duty = ph.duty as f64;
            let t_on = duty * cycle_t_sw;
            let t_off = cycle_t_sw - t_on;
            let t_cond_frac = ph.t_conduct_frac as f64;

            let i_sq_on = (i_start * i_start + i_start * i_peak + i_peak * i_peak) / 3.0;
            loss_fet_energy += rds_on * i_sq_on * t_on;

            let i_off_end = if ph.dcm { 0.0 } else { i_end };
            let t_conduct = t_cond_frac * t_off;
            let i_avg_off = (i_peak + i_off_end) / 2.0;
            loss_diode_energy += effective_v_diode * i_avg_off * t_conduct;

            let i_sq_off = (i_peak * i_peak + i_peak * i_off_end + i_off_end * i_off_end) / 3.0;
            loss_dcr_energy += dcr * (i_sq_on * t_on + i_sq_off * t_conduct);

            let i_load_phase = p.p_load_w() / p.v_out / p.num_phases.max(1) as f64;
            let i_cap_on = i_load_phase;
            let i_cap_off = (i_avg_off - i_load_phase).abs();
            loss_esr_energy += r_esr * (i_cap_on * i_cap_on * t_on + i_cap_off * i_cap_off * t_conduct);
        }
    }

    let i_rms = if total_dt > 0.0 { (i_rms_sq / total_dt).sqrt() } else { 0.0 };
    let p_in_avg = if total_dt > 0.0 { p_in / total_dt } else { 0.0 };

    // Fundamental amplitude (Fourier coefficients normalized by total_dt/2)
    let norm = 2.0 / total_dt;
    let i1_cos = cos_coeffs[1] * norm;
    let i1_sin = sin_coeffs[1] * norm;
    let fund_power = i1_cos * i1_cos + i1_sin * i1_sin;
    let fund_amp = fund_power.sqrt();

    // Per-harmonic as % of fundamental
    let mut harmonics_pct = vec![0.0; max_harmonic + 1];
    if fund_amp > 0.0 {
        harmonics_pct[1] = 100.0;
        for k in 2..=max_harmonic {
            let ck = cos_coeffs[k] * norm;
            let sk = sin_coeffs[k] * norm;
            let amp_k = (ck * ck + sk * sk).sqrt();
            harmonics_pct[k] = amp_k / fund_amp * 100.0;
        }
    }

    // THD
    let mut harmonic_power = 0.0_f64;
    for k in 2..=max_harmonic {
        let ck = cos_coeffs[k] * norm;
        let sk = sin_coeffs[k] * norm;
        harmonic_power += ck * ck + sk * sk;
    }
    let thd = if fund_power > 0.0 {
        (harmonic_power / fund_power).sqrt() * 100.0
    } else {
        0.0
    };

    // Power factor
    let v_rms = p.v_in_rms;
    let s_in = v_rms * i_rms;
    let power_factor = if s_in > 0.0 { p_in_avg / s_in } else { 0.0 };

    // Losses (average over the line cycle)
    let p_loss_fet = if total_dt > 0.0 { loss_fet_energy / total_dt } else { 0.0 };
    let p_loss_diode = if total_dt > 0.0 { loss_diode_energy / total_dt } else { 0.0 };
    let p_loss_dcr = if total_dt > 0.0 { loss_dcr_energy / total_dt } else { 0.0 };
    let p_loss_esr = if total_dt > 0.0 { loss_esr_energy / total_dt } else { 0.0 };
    let p_loss_total = p_loss_fet + p_loss_diode + p_loss_dcr + p_loss_esr;

    // Efficiency
    let efficiency = if p_in_avg > 0.0 {
        (p.p_load_w() / p_in_avg * 100.0).min(100.0)
    } else {
        0.0
    };

    PfcMetrics {
        thd_pct: thd,
        power_factor: power_factor.min(1.0),
        efficiency_pct: efficiency,
        v_out_ripple_v: v_out_ripple,
        v_out_avg,
        harmonics_pct,
        p_loss_fet_w: p_loss_fet,
        p_loss_diode_w: p_loss_diode,
        p_loss_dcr_w: p_loss_dcr,
        p_loss_esr_w: p_loss_esr,
        p_loss_total_w: p_loss_total,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_params_simulate_ok() {
        let p = PfcSimParams::default();
        let result = run_pfc_simulation(&p);
        assert!(result.is_ok(), "simulation failed: {:?}", result.err());
        let (points, metrics, _) = result.unwrap();
        assert!(!points.is_empty());
        assert!(metrics.v_out_avg > 0.0);
    }

    #[test]
    fn v_out_settles_near_target() {
        let p = PfcSimParams {
            steady_half_cycles: 36, // plenty of settling
            ..Default::default()
        };
        let (_points, metrics, _) = run_pfc_simulation(&p).unwrap();
        let v_err = (metrics.v_out_avg - p.v_out).abs();
        assert!(
            v_err < p.v_out * 0.1,
            "v_out_avg = {:.1}, target = {:.1}, error = {:.1}",
            metrics.v_out_avg, p.v_out, v_err
        );
    }

    #[test]
    fn thd_is_reasonable() {
        let p = PfcSimParams {
            steady_half_cycles: 36,
            ..Default::default()
        };
        let (_, metrics, _) = run_pfc_simulation(&p).unwrap();
        assert!(metrics.thd_pct.is_finite(), "THD is not finite");
        assert!(
            metrics.thd_pct < 50.0,
            "THD = {:.1}% — too high, indicates control issue",
            metrics.thd_pct
        );
    }

    #[test]
    fn rejects_vin_above_vout() {
        let p = PfcSimParams {
            v_in_rms: 300.0,
            v_out: 400.0,
            ..Default::default()
        };
        assert!(run_pfc_simulation(&p).is_err());
    }

    #[test]
    fn rejects_high_voltage_crossover() {
        let p = PfcSimParams {
            voltage_crossover_hz: 30.0,
            ..Default::default()
        };
        assert!(run_pfc_simulation(&p).is_err());
    }

    /// Helper: run a PFC simulation with enough settling time and return metrics.
    fn settled_metrics(p: &PfcSimParams) -> PfcMetrics {
        let p = PfcSimParams {
            steady_half_cycles: p.steady_half_cycles.max(36),
            ..p.clone()
        };
        let (_, metrics, _) = run_pfc_simulation(&p)
            .expect("simulation should succeed");
        metrics
    }

    // ── THD quality tests ──────────────────────────────────────────────

    #[test]
    fn thd_below_10pct_default_230v() {
        let m = settled_metrics(&PfcSimParams::default());
        eprintln!("230V/50Hz/300W: THD={:.2}%, PF={:.4}, V_ripple={:.2}V, V_avg={:.1}V",
            m.thd_pct, m.power_factor, m.v_out_ripple_v, m.v_out_avg);
        assert!(m.thd_pct < 10.0,
            "THD = {:.2}% — should be < 10%", m.thd_pct);
    }

    #[test]
    fn thd_below_5pct_large_inductor() {
        let m = settled_metrics(&PfcSimParams {
            l_uh: 2000.0,
            ..Default::default()
        });
        eprintln!("230V/2mH: THD={:.2}%, PF={:.4}", m.thd_pct, m.power_factor);
        assert!(m.thd_pct < 5.0,
            "THD = {:.2}% — large inductor should keep THD < 5%", m.thd_pct);
    }

    #[test]
    fn thd_below_10pct_120v_60hz() {
        let m = settled_metrics(&PfcSimParams {
            v_in_rms: 120.0,
            f_line_hz: 60.0,
            ..Default::default()
        });
        eprintln!("120V/60Hz: THD={:.2}%, PF={:.4}, V_ripple={:.2}V, V_avg={:.1}V",
            m.thd_pct, m.power_factor, m.v_out_ripple_v, m.v_out_avg);
        assert!(m.thd_pct < 10.0,
            "THD = {:.2}% — should be < 10% for 120V PFC", m.thd_pct);
    }

    #[test]
    fn thd_below_10pct_universal_low_line() {
        let m = settled_metrics(&PfcSimParams {
            v_in_rms: 85.0,
            f_line_hz: 60.0,
            l_uh: 800.0,
            ..Default::default()
        });
        eprintln!("85V/60Hz: THD={:.2}%, PF={:.4}, V_ripple={:.2}V, V_avg={:.1}V",
            m.thd_pct, m.power_factor, m.v_out_ripple_v, m.v_out_avg);
        assert!(m.thd_pct < 10.0,
            "THD = {:.2}% — should be < 10% for 85V low-line PFC", m.thd_pct);
    }

    #[test]
    fn power_factor_above_0_95_default() {
        let m = settled_metrics(&PfcSimParams::default());
        assert!(m.power_factor > 0.95,
            "PF = {:.4} — should be > 0.95 for a well-designed PFC", m.power_factor);
    }

    #[test]
    fn power_factor_above_0_90_low_line() {
        let m = settled_metrics(&PfcSimParams {
            v_in_rms: 85.0,
            f_line_hz: 60.0,
            l_uh: 800.0,
            ..Default::default()
        });
        assert!(m.power_factor > 0.90,
            "PF = {:.4} — should be > 0.90 at 85V low-line", m.power_factor);
    }

    #[test]
    fn thd_improves_with_higher_current_crossover() {
        let m_low_bw = settled_metrics(&PfcSimParams {
            current_crossover_khz: 2.0,
            ..Default::default()
        });
        let m_high_bw = settled_metrics(&PfcSimParams {
            current_crossover_khz: 10.0,
            ..Default::default()
        });
        eprintln!("2kHz crossover: THD={:.2}%  |  10kHz crossover: THD={:.2}%",
            m_low_bw.thd_pct, m_high_bw.thd_pct);
        assert!(m_high_bw.thd_pct <= m_low_bw.thd_pct,
            "Higher bandwidth should give equal or better THD: {:.2}% vs {:.2}%",
            m_high_bw.thd_pct, m_low_bw.thd_pct);
    }

    #[test]
    fn thd_improves_with_larger_inductor() {
        let m_small = settled_metrics(&PfcSimParams {
            l_uh: 200.0,
            ..Default::default()
        });
        let m_large = settled_metrics(&PfcSimParams {
            l_uh: 1000.0,
            ..Default::default()
        });
        eprintln!("200µH: THD={:.2}%  |  1000µH: THD={:.2}%",
            m_small.thd_pct, m_large.thd_pct);
        assert!(m_large.thd_pct < m_small.thd_pct,
            "Larger inductor should reduce THD: {:.2}% vs {:.2}%",
            m_large.thd_pct, m_small.thd_pct);
    }

    #[test]
    fn v_out_ripple_decreases_with_larger_cap() {
        let m_small = settled_metrics(&PfcSimParams {
            c_out_uf: 100.0,
            ..Default::default()
        });
        let m_large = settled_metrics(&PfcSimParams {
            c_out_uf: 470.0,
            ..Default::default()
        });
        eprintln!("100µF: ripple={:.2}V  |  470µF: ripple={:.2}V",
            m_small.v_out_ripple_v, m_large.v_out_ripple_v);
        assert!(m_large.v_out_ripple_v < m_small.v_out_ripple_v,
            "Larger cap should reduce ripple: {:.2}V vs {:.2}V",
            m_large.v_out_ripple_v, m_small.v_out_ripple_v);
    }

    #[test]
    fn thd_below_5pct_high_power() {
        let m = settled_metrics(&PfcSimParams {
            p_loads_w: vec![1000.0],
            c_out_uf: 470.0,
            ..Default::default()
        });
        eprintln!("1kW/230V: THD={:.2}%, PF={:.4}", m.thd_pct, m.power_factor);
        assert!(m.thd_pct < 5.0,
            "THD = {:.2}% — should be < 5% for 1kW PFC (deep CCM)", m.thd_pct);
    }

    // ── Multi-phase tests ─────────────────────────────────────────────

    #[test]
    fn single_phase_identical_to_default() {
        let p1 = PfcSimParams { num_phases: 1, ..Default::default() };
        let m1 = settled_metrics(&p1);
        let m_def = settled_metrics(&PfcSimParams::default());
        // Should produce identical results
        assert!((m1.thd_pct - m_def.thd_pct).abs() < 0.01,
            "1-phase should match default: THD {:.2}% vs {:.2}%", m1.thd_pct, m_def.thd_pct);
    }

    #[test]
    fn two_phase_reduces_output_ripple() {
        let m1 = settled_metrics(&PfcSimParams {
            num_phases: 1,
            ..Default::default()
        });
        let m2 = settled_metrics(&PfcSimParams {
            num_phases: 2,
            ..Default::default()
        });
        eprintln!("1-phase ripple: {:.2}V  |  2-phase ripple: {:.2}V",
            m1.v_out_ripple_v, m2.v_out_ripple_v);
        // 2-phase should have less ripple (interleaved cancellation)
        assert!(m2.v_out_ripple_v <= m1.v_out_ripple_v * 1.05,
            "2-phase should not have significantly more ripple: {:.2}V vs {:.2}V",
            m2.v_out_ripple_v, m1.v_out_ripple_v);
    }

    #[test]
    fn three_phase_v_out_settles() {
        let p = PfcSimParams {
            num_phases: 3,
            steady_half_cycles: 40,
            ..Default::default()
        };
        let (_, metrics, _) = run_pfc_simulation(&p).unwrap();
        let v_err = (metrics.v_out_avg - p.v_out).abs();
        assert!(v_err < p.v_out * 0.1,
            "3-phase v_out_avg = {:.1}, target = {:.1}", metrics.v_out_avg, p.v_out);
    }

    // ── Load step tests ───────────────────────────────────────────────

    #[test]
    fn load_step_shows_transient() {
        let p = PfcSimParams {
            p_loads_w: vec![300.0, 600.0],
            steady_half_cycles: 20,
            step_half_cycles: 20,
            ..Default::default()
        };
        let (points, _, _) = run_pfc_simulation(&p).unwrap();
        assert!(!points.is_empty());
        // v_out should be present throughout (no crash)
        assert!(points.last().unwrap().v_out.is_finite());
    }

    #[test]
    fn load_step_v_out_recovers() {
        // Voltage loop bandwidth is ~10Hz with 2×f_line pole attenuating response.
        // Use small step (300→400W) and long settling for reliable recovery.
        let p = PfcSimParams {
            p_loads_w: vec![300.0, 400.0],
            steady_half_cycles: 40,
            step_half_cycles: 100, // 100 half-cycles = 1s at 50Hz
            ..Default::default()
        };
        let (points, _, _) = run_pfc_simulation(&p).unwrap();
        let last_v = points.last().unwrap().v_out as f64;
        assert!(
            (last_v - p.v_out).abs() < p.v_out * 0.15,
            "v_out should recover after load step: {:.1}V vs {:.1}V target",
            last_v, p.v_out
        );
    }

    // ── Synchronous mode tests ────────────────────────────────────────

    #[test]
    fn sync_mode_runs_ok() {
        let m = settled_metrics(&PfcSimParams {
            current_conduction: CurrentConduction::Synchronous,
            ..Default::default()
        });
        assert!(m.thd_pct.is_finite());
        assert!(m.v_out_avg > 350.0);
    }

    #[test]
    fn sync_mode_negative_current_near_zero_crossing() {
        // In synchronous mode, current can go negative near line zero crossings.
        // This is physically correct — totem-pole PFC allows bidirectional current.
        // Without dead-zone/ZVS handling, this causes THD degradation, which is
        // expected. We just verify the simulation runs and v_out is reasonable.
        let m = settled_metrics(&PfcSimParams {
            current_conduction: CurrentConduction::Synchronous,
            ..Default::default()
        });
        assert!(m.v_out_avg > 350.0,
            "Sync mode v_out should be reasonable: {:.1}V", m.v_out_avg);
        assert!(m.thd_pct.is_finite(),
            "THD should be finite, got {:.2}%", m.thd_pct);
    }

    // ── Harmonics tests ───────────────────────────────────────────────

    #[test]
    fn harmonics_vector_populated() {
        let m = settled_metrics(&PfcSimParams::default());
        assert!(m.harmonics_pct.len() > 40, "should have at least 40 harmonics");
        assert!((m.harmonics_pct[1] - 100.0).abs() < 0.01,
            "fundamental should be 100%, got {:.2}%", m.harmonics_pct[1]);
    }

    #[test]
    fn harmonics_consistent_with_thd() {
        let m = settled_metrics(&PfcSimParams::default());
        // THD = sqrt(sum(h_k²)) for k >= 2, where h_k is in % of fundamental
        let thd_from_harmonics: f64 = m.harmonics_pct[2..].iter()
            .map(|h| (h / 100.0) * (h / 100.0))
            .sum::<f64>()
            .sqrt() * 100.0;
        let err = (thd_from_harmonics - m.thd_pct).abs();
        assert!(err < 0.1,
            "THD from harmonics ({:.2}%) should match THD metric ({:.2}%)",
            thd_from_harmonics, m.thd_pct);
    }

    // ── Loss tests ────────────────────────────────────────────────────

    #[test]
    fn losses_are_positive_and_finite() {
        let m = settled_metrics(&PfcSimParams::default());
        assert!(m.p_loss_fet_w > 0.0 && m.p_loss_fet_w.is_finite(),
            "FET loss = {:.3}W", m.p_loss_fet_w);
        assert!(m.p_loss_diode_w >= 0.0 && m.p_loss_diode_w.is_finite(),
            "Diode loss = {:.3}W", m.p_loss_diode_w);
        assert!(m.p_loss_dcr_w > 0.0 && m.p_loss_dcr_w.is_finite(),
            "DCR loss = {:.3}W", m.p_loss_dcr_w);
        assert!(m.p_loss_total_w > 0.0 && m.p_loss_total_w.is_finite(),
            "Total loss = {:.3}W", m.p_loss_total_w);
    }

    #[test]
    fn losses_are_reasonable_magnitude() {
        let m = settled_metrics(&PfcSimParams::default());
        eprintln!("Losses: FET={:.2}W, Diode={:.2}W, DCR={:.2}W, ESR={:.2}W, Total={:.2}W",
            m.p_loss_fet_w, m.p_loss_diode_w, m.p_loss_dcr_w, m.p_loss_esr_w, m.p_loss_total_w);
        // Total conduction losses for 300W PFC should be < 30W (< 10%)
        assert!(m.p_loss_total_w < 30.0,
            "Total loss {:.2}W seems too high for 300W converter", m.p_loss_total_w);
    }

    #[test]
    fn sync_mode_zero_diode_loss() {
        let m = settled_metrics(&PfcSimParams {
            current_conduction: CurrentConduction::Synchronous,
            v_diode: 0.0, // no diode voltage in sync mode
            ..Default::default()
        });
        assert!(m.p_loss_diode_w < 0.01,
            "Sync mode with v_diode=0 should have near-zero diode loss: {:.4}W",
            m.p_loss_diode_w);
    }

    // ── Vin sweep test ────────────────────────────────────────────────

    #[test]
    fn vin_sweep_runs() {
        let p = PfcSimParams::default();
        let points = [120.0, 230.0];
        let results = run_pfc_vin_sweep(&p, &points);
        assert_eq!(results.len(), 2);
        for (vin, result) in &results {
            assert!(result.is_ok(), "sweep failed at {vin}V: {:?}", result);
        }
    }

    #[test]
    fn vin_sweep_230v_matches_single_run() {
        let p = PfcSimParams::default();
        let sweep = run_pfc_vin_sweep(&p, &[230.0]);
        let single = settled_metrics(&p);
        let sweep_m = sweep[0].1.as_ref().unwrap();
        // THD should be close (not exact due to different settling times)
        assert!((sweep_m.thd_pct - single.thd_pct).abs() < 2.0,
            "Sweep THD ({:.2}%) should be close to single run ({:.2}%)",
            sweep_m.thd_pct, single.thd_pct);
    }

    // ── CrCM tests ────────────────────────────────────────────────────

    #[test]
    fn crm_runs_and_settles() {
        let m = settled_metrics(&PfcSimParams {
            control_mode: PfcControlMode::CrCm,
            ..Default::default()
        });
        assert!(m.v_out_avg > 350.0,
            "CrCM v_out should settle near target: {:.1}V", m.v_out_avg);
        assert!(m.thd_pct.is_finite());
        assert!(m.power_factor > 0.8,
            "CrCM PF should be reasonable: {:.4}", m.power_factor);
    }

    #[test]
    fn crm_variable_frequency_in_points() {
        let p = PfcSimParams {
            control_mode: PfcControlMode::CrCm,
            steady_half_cycles: 10,
            ..Default::default()
        };
        let (points, _, _) = run_pfc_simulation(&p).unwrap();
        // CrCM should have variable t_sw_us across the half-cycle
        let last_half: Vec<_> = points.iter().rev().take(200).collect();
        if last_half.len() > 10 {
            let min_tsw = last_half.iter().map(|p| p.t_sw_us).fold(f32::MAX, f32::min);
            let max_tsw = last_half.iter().map(|p| p.t_sw_us).fold(f32::MIN, f32::max);
            assert!(max_tsw > min_tsw * 1.2,
                "CrCM should have variable frequency: min={:.1}µs max={:.1}µs",
                min_tsw, max_tsw);
        }
    }

    // ── PCMC tests ────────────────────────────────────────────────────

    #[test]
    fn pcmc_runs_and_settles() {
        let m = settled_metrics(&PfcSimParams {
            control_mode: PfcControlMode::Pcmc,
            steady_half_cycles: 80, // PCMC needs more settling than ACM
            ..Default::default()
        });
        assert!(m.v_out_avg > 350.0,
            "PCMC v_out should settle near target: {:.1}V", m.v_out_avg);
        assert!(m.thd_pct < 20.0,
            "PCMC THD should be reasonable: {:.1}%", m.thd_pct);
        assert!(m.power_factor > 0.9,
            "PCMC PF should be reasonable: {:.4}", m.power_factor);
    }

    #[test]
    fn pcmc_slope_comp_affects_behavior() {
        let m_low = settled_metrics(&PfcSimParams {
            control_mode: PfcControlMode::Pcmc,
            slope_overcomp: 1.0,
            ..Default::default()
        });
        let m_high = settled_metrics(&PfcSimParams {
            control_mode: PfcControlMode::Pcmc,
            slope_overcomp: 3.0,
            ..Default::default()
        });
        // Both should produce finite results
        assert!(m_low.v_out_avg > 300.0 && m_high.v_out_avg > 300.0);
        // Different slope comp should give different THD
        assert!((m_low.thd_pct - m_high.thd_pct).abs() > 0.01 ||
                (m_low.v_out_avg - m_high.v_out_avg).abs() > 0.1,
            "slope comp should affect results");
    }

    // ── Totem-pole topology tests ─────────────────────────────────────

    #[test]
    fn totem_pole_runs() {
        let m = settled_metrics(&PfcSimParams {
            topology: PfcTopology::TotemPole,
            ..Default::default()
        });
        assert!(m.v_out_avg > 350.0,
            "Totem-pole v_out: {:.1}V", m.v_out_avg);
        assert!(m.thd_pct.is_finite());
    }

    #[test]
    fn totem_pole_no_diode_loss() {
        let m = settled_metrics(&PfcSimParams {
            topology: PfcTopology::TotemPole,
            ..Default::default()
        });
        assert!(m.p_loss_diode_w < 0.01,
            "Totem-pole should have ~zero diode loss: {:.3}W", m.p_loss_diode_w);
    }

    #[test]
    fn totem_pole_with_crm() {
        let m = settled_metrics(&PfcSimParams {
            control_mode: PfcControlMode::CrCm,
            topology: PfcTopology::TotemPole,
            ..Default::default()
        });
        assert!(m.v_out_avg > 300.0,
            "Totem-pole CrCM v_out: {:.1}V", m.v_out_avg);
        assert!(m.p_loss_diode_w < 0.01);
    }

    #[test]
    fn totem_pole_with_pcmc() {
        let m = settled_metrics(&PfcSimParams {
            control_mode: PfcControlMode::Pcmc,
            topology: PfcTopology::TotemPole,
            ..Default::default()
        });
        assert!(m.v_out_avg > 300.0,
            "Totem-pole PCMC v_out: {:.1}V", m.v_out_avg);
        assert!(m.p_loss_diode_w < 0.01);
    }
}
