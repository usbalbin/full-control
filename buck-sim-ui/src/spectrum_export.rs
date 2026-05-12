//! Export the input-port current spectrum from a buck simulation as
//! a `pdn_schema::PortCurrentSpectrum` JSON — the producer side of
//! the kicad_field_solver `current-injection` mode.
//!
//! Two reconstruction modes balance bandwidth vs. fidelity:
//!
//! - **Envelope.** FFT the cycle-rate input-current envelope
//!   (`duty × i_L_avg` per cycle). One sample per switching cycle,
//!   so the resulting spectrum covers DC – fsw/2. Captures
//!   control-loop oscillation, audio-band ripple, subharmonic fsw/2
//!   peaks; *does not* contain content at the switching frequency or
//!   above. Useful for phase 4 (control-loop oscillation → conducted
//!   EM) where the failure modes are sub-fsw.
//!
//! - **PWM-reconstructed.** Synthesize an idealized switching waveform
//!   at `samples_per_cycle` samples per switching cycle (HS on for
//!   `duty × T_sw`, inductor-current ramp from `i_L_min` to `i_L_max`;
//!   HS off at zero), then FFT. Spectrum covers DC – N×fsw/2. Picks
//!   up fsw and its harmonics — the dominant conducted-EM content
//!   for a synchronous buck.
//!
//!   Sub-modes:
//!     - `trapezoidal: false` — instantaneous transitions (perfect
//!       rectangular pulse).
//!     - `trapezoidal: true` — finite-slope edges from
//!       `SimParams.hs_fet.t_rise_ns / t_fall_ns`. Bandlimits the
//!       harmonics above ~`1/(π·t_rise)`. (Cycle-average drops a
//!       few percent vs sharp-edge mode — physically correct,
//!       since at HS turn-on the input current rises from 0
//!       rather than instantaneously jumping to `i_L_min`.)
//!     - `ringing: Some(RingingParams)` — physics-based damped
//!       sinusoid at the commutation-loop resonance after each HS
//!       transition. Amplitude `V_step/(L·ω_d)` from the
//!       step-response of a series RLC; decay `α = R/(2L)`. Picks
//!       up the f_ring peak that the trapezoidal-only model
//!       silently drops.
//!     - `qrr_nc + trr_ns > 0` — LS body-diode reverse-recovery
//!       triangle pulse of area Q_rr and base t_rr at HS turn-on.
//!       Adds the broadband sinc content that real Si MOSFETs
//!       inject on the input rail.
//!
//!   **Miller plateau** is now modeled when `hs_fet.qgd_nc > 0`
//!   and `hs_fet.rg_ohm > 0`. The plateau width
//!     t_miller = Q_gd · R_g / (V_drive − V_miller)
//!   is inserted as a constant-current shoulder at `i_L_min` right
//!   after the rising edge (drain current held while V_DS swings
//!   high→low) and at `i_L_max` right before the falling edge.
//!   Negligible (~1 ns) for fast low-voltage GaN/Si at 12 V; 10–
//!   50 ns at 400 V or with slow drivers — relevant for offline
//!   PFC / high-voltage designs. The inductor-ramp window between
//!   the two plateaus is shortened to fit. If the on-time can't
//!   accommodate both plateaus + a ramp, Miller modelling is
//!   skipped silently for that cycle (the trapezoidal-only shape
//!   remains).
//!
//! Both modes apply a Hann window over the steady-state half of the
//! captured cycles, divide by the window's coherent gain so a pure
//! sine at amplitude A reads A, and double the non-DC / non-Nyquist
//! bins for one-sided convention. The producer's choice of mode and
//! windowing is recorded in `PortCurrentSpectrum::source`.

use pdn_schema::{ControllerFlavor, PortCurrentSpectrum};
use rustfft::{num_complex::Complex, FftPlanner};
use std::f64::consts::PI;

use crate::sim::{SimParams, SimPoint};

/// Physics-based commutation-loop ringing model. At each HS
/// transition the loop sees a voltage step of magnitude `V_step`
/// (≈ V_in for a buck — the switch-node voltage moves between 0 and
/// V_in, and the integral of that delta across the loop's stray L
/// is what drives the ring). The loop's L, R, and the FETs' Coss
/// form a series RLC; its step response is
///
///     i_ring(t) = (V_step / (L · ω_d)) · exp(-α·t) · sin(ω_d·t)
///
/// with `ω_n² = 1/(LC)`, `α = R/(2L)`, `ω_d² = ω_n² - α²`. The
/// peak amplitude `V_step/(L·ω_d)` is derived from physics; no
/// fudge factor.
///
/// For HS turn-on the transient is added to the input current
/// (rising edge); for HS turn-off the same shape is added with the
/// opposite sign on the falling edge.
///
/// Over-damped systems (`α² ≥ ω_n²` → `R ≥ 2·√(L/C)`) → waveform
/// returns 0. Typical hard-switched buck has R_loop ≪ 2√(L/C) so
/// the system is deeply under-damped and Q ≈ √(L/C)/R is hundreds.
#[derive(Debug, Clone, Copy)]
pub struct RingingParams {
    pub l_loop_h: f64,
    pub r_loop_ohm: f64,
    pub c_oss_total_f: f64,
    /// Voltage step at each transition (typically V_in).
    pub v_step_v: f64,
}

impl RingingParams {
    fn omega_n_sq(&self) -> f64 {
        1.0 / (self.l_loop_h * self.c_oss_total_f)
    }
    fn alpha(&self) -> f64 {
        self.r_loop_ohm / (2.0 * self.l_loop_h)
    }
    fn omega_d_sq(&self) -> f64 {
        self.omega_n_sq() - self.alpha().powi(2)
    }
    /// Damped resonance frequency. 0 → over-damped (no ring).
    pub fn f_ring_hz(&self) -> f64 {
        let wd2 = self.omega_d_sq();
        if wd2 > 0.0 {
            wd2.sqrt() / (2.0 * std::f64::consts::PI)
        } else {
            0.0
        }
    }
    /// Q factor of the resonator. `Q = (1/R)·√(L/C)` for series RLC.
    pub fn q_factor(&self) -> f64 {
        if self.r_loop_ohm > 0.0 {
            (self.l_loop_h / self.c_oss_total_f).sqrt() / self.r_loop_ohm
        } else {
            f64::INFINITY
        }
    }
    /// Step-response current at `t_s` seconds after a `+V_step`
    /// voltage step. Amplitude is `V_step / (L·ω_d)`; multiply by
    /// the caller's sign to handle HS turn-off.
    fn waveform(&self, t_s: f64, sign: f64) -> f64 {
        let wd2 = self.omega_d_sq();
        if wd2 <= 0.0 || t_s < 0.0 {
            return 0.0;
        }
        let wd = wd2.sqrt();
        let amp = self.v_step_v / (self.l_loop_h * wd);
        sign * amp * (-self.alpha() * t_s).exp() * (wd * t_s).sin()
    }
}

/// Precomputed parameters for one PWM-reconstruction pass.
/// Cheap to build; reused across cycles by `export_input_current_spectrum`
/// and by `synthesize_steady_state_cycle` (the scope view), so the
/// two consumers always draw from identical math.
#[derive(Debug, Clone, Copy)]
pub struct PwmReconParams {
    pub t_sw_s: f64,
    pub samples_per_cycle: usize,
    pub n_phases: f64,
    pub t_rise_s: f64,
    pub t_fall_s: f64,
    pub t_miller_s: f64,
    pub qrr_c: f64,
    pub t_rr_s: f64,
    pub ringing: Option<RingingParams>,
    pub ring_decay_window_s: f64,
}

impl PwmReconParams {
    pub fn build(
        p: &SimParams,
        samples_per_cycle: usize,
        trapezoidal: bool,
        ringing: Option<RingingParams>,
        qrr_nc: f64,
        trr_ns: f64,
        n_phases: f64,
    ) -> Self {
        let fsw = p.f_sw_khz * 1e3;
        let t_sw_s = if fsw > 0.0 { 1.0 / fsw } else { 1.0 };
        let (t_rise_s, t_fall_s) = if trapezoidal {
            let r = (p.hs_fet.t_rise_ns * 1e-9).max(0.0);
            let f = (p.hs_fet.t_fall_ns * 1e-9).max(0.0);
            (r.min(0.25 * t_sw_s), f.min(0.25 * t_sw_s))
        } else {
            (0.0, 0.0)
        };
        let t_miller_s = {
            let qgd = p.hs_fet.qgd_nc * 1e-9;
            let rg = p.hs_fet.rg_ohm;
            let v_miller_eff = if p.hs_fet.v_miller_v > 0.0 {
                p.hs_fet.v_miller_v
            } else {
                0.5 * p.hs_fet.vgs_v
            };
            let v_overdrive = p.hs_fet.vgs_v - v_miller_eff;
            if trapezoidal && qgd > 0.0 && rg > 0.0 && v_overdrive > 0.0 {
                (qgd * rg / v_overdrive).min(0.15 * t_sw_s)
            } else {
                0.0
            }
        };
        let qrr_c = (qrr_nc * 1e-9).max(0.0);
        let t_rr_s = if trr_ns > 0.0 {
            (trr_ns * 1e-9).min(0.5 * t_sw_s)
        } else if qrr_c > 0.0 {
            20e-9
        } else {
            0.0
        };
        let ring_decay_window_s = ringing
            .and_then(|r| {
                let a = r.alpha();
                if a > 0.0 {
                    Some(8.0 / a)
                } else {
                    None
                }
            })
            .unwrap_or(f64::INFINITY);
        Self {
            t_sw_s,
            samples_per_cycle,
            n_phases,
            t_rise_s,
            t_fall_s,
            t_miller_s,
            qrr_c,
            t_rr_s,
            ringing,
            ring_decay_window_s,
        }
    }
}

/// Boundary points of one switching cycle, relative to cycle start
/// in seconds. Used by the scope view to annotate the regions of
/// the waveform.
#[derive(Debug, Clone, Copy, Default)]
pub struct CycleMarkers {
    pub t_flat_start: f64,        // end of current-rise / start of turn-on Miller
    pub t_miller_on_end: f64,     // end of turn-on Miller / start of inductor ramp
    pub t_miller_off_start: f64,  // end of inductor ramp / start of turn-off Miller
    pub t_flat_end: f64,          // end of turn-off Miller / start of current-fall
    pub t_on_end: f64,            // end of HS on / start of HS off (dead-time absorbed in t_fall)
}

/// Synthesize one switching cycle of input-port current. Appends
/// `samples_per_cycle` samples to `samples_out` (in amperes,
/// already multiplied by `n_phases` so the result is the
/// interleaved sum the input cap sees). `transitions` is mutated to
/// record HS-on / HS-off events for the ringing model — the caller
/// should pass the same Vec across cycles so ringing decays
/// correctly across cycle boundaries.
pub fn synthesize_one_cycle(
    params: &PwmReconParams,
    cycle_idx: usize,
    duty: f64,
    i_min_per_phase: f64,
    i_max_per_phase: f64,
    transitions: &mut Vec<(f64, f64)>,
    samples_out: &mut Vec<f64>,
) -> CycleMarkers {
    let m = params.samples_per_cycle;
    let t_sw_s = params.t_sw_s;
    let cycle_start_s = (cycle_idx as f64) * t_sw_s;
    let duty = duty.clamp(0.0, 1.0);
    let i_min = i_min_per_phase;
    let i_max = i_max_per_phase;
    let t_on_s = duty * t_sw_s;
    let t_flat_start = params.t_rise_s.min(t_on_s);
    let t_flat_end = (t_on_s - params.t_fall_s).max(t_flat_start);
    let total_miller = 2.0 * params.t_miller_s;
    let ramp_budget = (t_flat_end - t_flat_start).max(0.0);
    let t_miller_eff = if ramp_budget > total_miller {
        params.t_miller_s
    } else {
        0.0
    };
    let t_miller_on_end = t_flat_start + t_miller_eff;
    let t_miller_off_start = t_flat_end - t_miller_eff;

    if duty > 0.0 && params.ringing.is_some() {
        transitions.push((cycle_start_s + t_flat_start * 0.5, 1.0));
        let toff_mid = t_flat_end + (t_on_s - t_flat_end) * 0.5;
        transitions.push((cycle_start_s + toff_mid, -1.0));
    }

    for k in 0..m {
        let tau_s = (k as f64) / (m as f64) * t_sw_s;
        let abs_t = cycle_start_s + tau_s;

        // 1. Trapezoidal base + Miller plateaus.
        let mut v = if duty <= 0.0 {
            0.0
        } else if tau_s < t_flat_start {
            let frac = if t_flat_start > 0.0 {
                tau_s / t_flat_start
            } else {
                1.0
            };
            frac * i_min
        } else if tau_s < t_miller_on_end {
            i_min
        } else if tau_s < t_miller_off_start {
            let denom = (t_miller_off_start - t_miller_on_end).max(1e-18);
            let frac = ((tau_s - t_miller_on_end) / denom).clamp(0.0, 1.0);
            i_min + frac * (i_max - i_min)
        } else if tau_s < t_flat_end {
            i_max
        } else if tau_s < t_on_s {
            let denom = (t_on_s - t_flat_end).max(1e-18);
            let frac = ((tau_s - t_flat_end) / denom).clamp(0.0, 1.0);
            i_max * (1.0 - frac)
        } else {
            0.0
        };

        // 2. Body-diode reverse-recovery (LS Qrr).
        if duty > 0.0 && params.qrr_c > 0.0 && params.t_rr_s > 0.0 {
            let center = params.t_rise_s.max(params.t_rr_s) * 0.5;
            let half_w = params.t_rr_s * 0.5;
            if (tau_s - center).abs() < half_w {
                let peak = 2.0 * params.qrr_c / params.t_rr_s;
                let dist = (tau_s - center).abs();
                v += peak * (1.0 - dist / half_w);
            }
        }

        // 3. Commutation-loop ringing.
        if let Some(r) = params.ringing.as_ref() {
            for &(t_event, sign) in transitions.iter() {
                let dt = abs_t - t_event;
                if dt < 0.0 || dt > params.ring_decay_window_s {
                    continue;
                }
                v += r.waveform(dt, sign);
            }
        }

        samples_out.push(params.n_phases * v);
    }

    CycleMarkers {
        t_flat_start,
        t_miller_on_end,
        t_miller_off_start,
        t_flat_end,
        t_on_end: t_on_s,
    }
}

/// Snapshot of one steady-state switching cycle, suitable for an
/// oscilloscope-style scope panel. Generated with several warmup
/// cycles of ringing history so the returned samples reflect
/// periodic steady-state (no startup transient).
#[derive(Debug, Clone)]
pub struct SteadyStateCycle {
    /// Input-port current samples for one full T_sw, amperes
    /// (interleaved sum). Length = `samples_per_cycle`.
    pub samples: Vec<f64>,
    /// Switching period in seconds.
    pub t_sw_s: f64,
    /// Region boundaries within the cycle.
    pub markers: CycleMarkers,
    /// Precomputed params used for the synthesis (lets the UI
    /// display the same physics readouts the spectrum export does).
    pub params: PwmReconParams,
}

/// Generate one steady-state cycle of input-port current for the
/// scope view. Picks the operating point from `(duty, i_min, i_max)`
/// (typically pulled from the last SimPoint in `sim_data` for
/// steady-state operation) and runs the same PWM-reconstruction
/// synthesis the spectrum exporter uses — with a few warmup cycles
/// first so the ringing model has reached periodic steady state.
pub fn synthesize_steady_state_cycle(
    p: &SimParams,
    duty: f64,
    i_total_min: f64,
    i_total_max: f64,
    n_phases: f64,
    samples_per_cycle: u32,
    trapezoidal: bool,
    ringing: Option<RingingParams>,
    qrr_nc: f64,
    trr_ns: f64,
) -> Option<SteadyStateCycle> {
    if samples_per_cycle < 4 {
        return None;
    }
    let m = samples_per_cycle as usize;
    let n_phases = n_phases.max(1.0);
    let params = PwmReconParams::build(p, m, trapezoidal, ringing, qrr_nc, trr_ns, n_phases);
    let i_min = i_total_min / n_phases;
    let i_max = i_total_max / n_phases;

    // Warmup so ringing reaches periodic steady-state. 4 cycles is
    // enough at typical Q values; high-Q rings (Q >> 1000) may need
    // more — capped at 8 cycles so the scope view stays snappy.
    let n_warmup = 4;
    let mut transitions: Vec<(f64, f64)> = Vec::new();
    let mut all_samples: Vec<f64> = Vec::with_capacity(m * (n_warmup + 1));
    let mut last_markers = CycleMarkers::default();
    for cyc in 0..=n_warmup {
        last_markers = synthesize_one_cycle(
            &params,
            cyc,
            duty,
            i_min,
            i_max,
            &mut transitions,
            &mut all_samples,
        );
    }
    let last_start = m * n_warmup;
    let samples = all_samples[last_start..].to_vec();
    Some(SteadyStateCycle {
        samples,
        t_sw_s: params.t_sw_s,
        markers: last_markers,
        params,
    })
}

/// Reconstruction strategy. See module docs.
#[derive(Debug, Clone, Copy)]
pub enum ExportMode {
    Envelope,
    /// Reconstruct idealized PWM at `samples_per_cycle` samples per
    /// switching cycle, with optional trapezoidal rise/fall edges
    /// modelling the FET's finite transition time. When `trapezoidal`
    /// is `true`, the rise/fall times come from `SimParams.hs_fet`
    /// and bandlimit the harmonics above ~`1/(π·t_rise)`, which is
    /// the dominant first-order effect that distinguishes a real
    /// switching spectrum from a perfect square wave.
    ///
    /// `ringing` superimposes a damped sinusoid at the commutation-
    /// loop resonance `1/(2π√(L_loop·C_oss))` after each HS
    /// transition. Adds a spectral peak at `f_ring` that the
    /// trapezoidal-only model misses. Requires a `LoopExtraction` to
    /// be loaded (for `L_loop`, `R_loop`).
    ///
    /// `qrr_nc > 0` adds a body-diode reverse-recovery di/dt spike
    /// at HS turn-on — a triangular current pulse of area `Q_rr`
    /// and base `t_rr` centered on the rising edge, peak
    /// `2·Q_rr/t_rr`. Adds positive current at the HS drain (the
    /// stored body-diode charge is pulled *through* HS as the
    /// switch node moves up). Set both to 0 for GaN HEMTs.
    PwmReconstructed {
        samples_per_cycle: u32,
        trapezoidal: bool,
        ringing: Option<RingingParams>,
        qrr_nc: f64,
        trr_ns: f64,
    },
}

/// FFT the input-port current over the steady-state half of a buck
/// simulation and produce a `PortCurrentSpectrum` ready for
/// serialization. The reported `ControllerFlavor` is `HostF32` —
/// today's only buck-sim-ui instantiation. Future commits switch the
/// inner-loop call sites to `FmacIir` and emit `FmacQ15` instead
/// (docs/closed_loop_emc_plan.md §6.1).
pub fn export_input_current_spectrum(
    sim_data: &[SimPoint],
    p: &SimParams,
    mode: ExportMode,
) -> Result<PortCurrentSpectrum, String> {
    if sim_data.is_empty() {
        return Err("no simulation samples".into());
    }
    let fsw = p.f_sw_khz * 1e3;
    if fsw <= 0.0 {
        return Err(format!("invalid switching frequency: {} kHz", p.f_sw_khz));
    }

    // Steady-state window: last 50 % of the captured cycles. Skips
    // startup / soft-start transients without requiring user tuning.
    let n_cycles = sim_data.len();
    let win_start = n_cycles / 2;
    if n_cycles - win_start < 8 {
        return Err(format!(
            "too few cycles ({}); need at least 16 to take a meaningful FFT \
             of the steady-state half",
            n_cycles,
        ));
    }
    let cycles = &sim_data[win_start..];

    let (samples, sample_rate_hz, mode_tag) = match mode {
        ExportMode::Envelope => {
            let s: Vec<f64> = cycles
                .iter()
                .map(|c| {
                    let duty = c.duty_pct_avg() as f64 / 100.0;
                    let i_l = c.i_total_avg() as f64;
                    duty * i_l
                })
                .collect();
            (s, fsw, "mode=envelope".to_string())
        }
        ExportMode::PwmReconstructed {
            samples_per_cycle,
            trapezoidal,
            ringing,
            qrr_nc,
            trr_ns,
        } => {
            if samples_per_cycle < 4 {
                return Err("samples_per_cycle must be >= 4".into());
            }
            let m = samples_per_cycle as usize;
            // n_phases varies cycle-by-cycle in theory, but in
            // practice phase count is fixed at config time. Take it
            // from the first cycle of the window.
            let n_phases = cycles[0].phases.len().max(1) as f64;
            let params = PwmReconParams::build(
                p, m, trapezoidal, ringing, qrr_nc, trr_ns, n_phases,
            );
            let mut transitions: Vec<(f64, f64)> = Vec::new();
            let mut s: Vec<f64> = Vec::with_capacity(m * cycles.len());
            for (cyc_idx, c) in cycles.iter().enumerate() {
                let duty = (c.duty_pct_avg() as f64 / 100.0).clamp(0.0, 1.0);
                let cyc_n_phases = c.phases.len().max(1) as f64;
                let i_min = c.i_total_min as f64 / cyc_n_phases;
                let i_max = c.i_total_max as f64 / cyc_n_phases;
                synthesize_one_cycle(
                    &params, cyc_idx, duty, i_min, i_max,
                    &mut transitions, &mut s,
                );
            }
            let edge_tag = if trapezoidal {
                let miller_tag = if params.t_miller_s > 0.0 {
                    format!(",miller={:.2}ns", params.t_miller_s * 1e9)
                } else {
                    String::new()
                };
                format!(
                    ",trapezoidal=t_rise={:.2}ns,t_fall={:.2}ns{}",
                    p.hs_fet.t_rise_ns, p.hs_fet.t_fall_ns, miller_tag,
                )
            } else {
                ",trapezoidal=off".to_string()
            };
            let ringing_tag = if let Some(r) = ringing.as_ref() {
                format!(
                    ",ringing=f_ring={:.2}MHz_Q={:.1}",
                    r.f_ring_hz() / 1e6,
                    r.q_factor(),
                )
            } else {
                ",ringing=off".to_string()
            };
            let qrr_tag = if params.qrr_c > 0.0 {
                format!(",qrr={:.2}nC_trr={:.1}ns", qrr_nc, params.t_rr_s * 1e9)
            } else {
                ",qrr=off".to_string()
            };
            (
                s,
                fsw * m as f64,
                format!(
                    "mode=pwm-reconstructed,samples_per_cycle={}{}{}{}",
                    m, edge_tag, ringing_tag, qrr_tag,
                ),
            )
        }
    };

    let n_samples = samples.len();
    if n_samples < 4 {
        return Err("too few samples after window selection".into());
    }
    let peak = samples.iter().fold(0.0_f64, |a, &b| a.max(b.abs()));

    // Hann window + normalization. Coherent gain = mean(w); dividing
    // FFT output by (N × coherent_gain) makes a pure sine at
    // amplitude A read A in the one-sided spectrum (after the ×2
    // for non-DC/non-Nyquist bins below).
    let denom_factor = (n_samples - 1).max(1) as f64;
    let hann: Vec<f64> = (0..n_samples)
        .map(|k| 0.5 * (1.0 - (2.0 * PI * (k as f64) / denom_factor).cos()))
        .collect();
    let coherent_gain = hann.iter().sum::<f64>() / n_samples as f64;

    let mut buf: Vec<Complex<f64>> = samples
        .iter()
        .zip(&hann)
        .map(|(&x, &w)| Complex::new(x * w, 0.0))
        .collect();

    let mut planner = FftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(n_samples);
    fft.process(&mut buf);

    let n_pos = n_samples / 2 + 1;
    let norm = 1.0 / (n_samples as f64 * coherent_gain);
    let mut freqs_hz = Vec::with_capacity(n_pos);
    let mut i_re_amp = Vec::with_capacity(n_pos);
    let mut i_im_amp = Vec::with_capacity(n_pos);
    for k in 0..n_pos {
        let f = (k as f64) * sample_rate_hz / n_samples as f64;
        // Conjugate-symmetric FFT: scale non-DC, non-Nyquist bins by
        // 2 to fold the negative-frequency partner.
        let scale = if k == 0 || k * 2 == n_samples { 1.0 } else { 2.0 };
        let val = buf[k] * (norm * scale);
        freqs_hz.push(f);
        i_re_amp.push(val.re);
        i_im_amp.push(val.im);
    }

    let source = format!(
        "buck-sim-ui v{} input-current spectrum ({}, Hann window, \
         {} steady-state cycles, fsw = {:.3} kHz)",
        env!("CARGO_PKG_VERSION"),
        mode_tag,
        cycles.len(),
        fsw / 1e3,
    );
    Ok(PortCurrentSpectrum::new(
        source,
        ControllerFlavor::HostF32,
        "VIN (input port, cycle-averaged HS-FET drain)",
        "GND",
        freqs_hz,
        i_re_amp,
        i_im_amp,
        peak,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{PhasePoint, SimPoint};

    /// Build a fake steady-state SimPoint stream: every cycle has the
    /// same duty/current. Useful as a regression fixture for the FFT
    /// magnitude — the spectrum of a constant should be a single DC
    /// spike.
    fn const_stream(n_cycles: usize, duty: f32, i_l: f32) -> Vec<SimPoint> {
        let p = PhasePoint {
            t_on: duty,
            duty_pct: duty * 100.0,
            i_l_min: i_l - 0.5,
            i_l_max: i_l + 0.5,
        };
        (0..n_cycles)
            .map(|k| SimPoint {
                t_ms: k as f32 * 1e-3,
                v_out: 12.0,
                phases: vec![p.clone()],
                i_total_min: i_l - 0.5,
                i_total_max: i_l + 0.5,
                v_out_min: 12.0,
                v_out_max: 12.0,
                v_bat: 0.0,
                v_in_cap: 0.0,
            })
            .collect()
    }

    fn default_params() -> SimParams {
        SimParams::default()
    }

    #[test]
    fn envelope_constant_stream_dc_bin_recovers_amplitude() {
        let stream = const_stream(64, 0.5, 10.0);
        let spec = export_input_current_spectrum(&stream, &default_params(), ExportMode::Envelope)
            .unwrap();
        // DC bin should match duty × i_L = 0.5 × 10 = 5 A. A Hann
        // window's coherent gain is 0.5, which my normalization
        // divides back out, so DC should read the actual amplitude.
        //
        // Note: a Hann-windowed constant is NOT a pure delta in
        // frequency — Hann itself has bin ±1 sidelobes at half DC
        // amplitude. So we only assert DC magnitude here; the PWM
        // test below covers the harder "right tone at right
        // frequency" case.
        let dc = spec.i_re_amp[0];
        assert!(
            (dc - 5.0).abs() / 5.0 < 0.05,
            "DC bin {} should be ~5 A",
            dc,
        );
        assert_eq!(spec.i_im_amp[0], 0.0);
        // i_peak_amp_ref should equal the time-domain peak.
        assert!((spec.i_peak_amp_ref - 5.0).abs() / 5.0 < 1e-9);
    }

    #[test]
    fn ringing_params_physics_sanity() {
        // L=5 nH, R=1 mΩ, C=600 pF → f_ring ≈ 92 MHz,
        // Q = √(L/C)/R = √(5e-9/600e-12) / 1e-3 ≈ 2887.
        let r = RingingParams {
            l_loop_h: 5e-9,
            r_loop_ohm: 1e-3,
            c_oss_total_f: 600e-12,
            v_step_v: 12.0,
        };
        let f = r.f_ring_hz();
        assert!(
            (f - 92e6).abs() / 92e6 < 0.05,
            "f_ring should be ~92 MHz, got {} Hz",
            f
        );
        assert!(r.q_factor() > 1000.0, "Q={}; should be >>1", r.q_factor());
        // At t=0 the waveform is sin(0)=0; sample at quarter period
        // to hit the peak overshoot.
        let t_quarter = 1.0 / (4.0 * f);
        let peak_expected = r.v_step_v / (r.l_loop_h * 2.0 * std::f64::consts::PI * f);
        let measured = r.waveform(t_quarter, 1.0);
        // Light damping → measured peak should be within a few %
        // of the undamped V/(Lω) amplitude.
        assert!(
            (measured - peak_expected).abs() / peak_expected < 0.05,
            "ring peak {} should be ~{} A (V/Lω at quarter period)",
            measured,
            peak_expected,
        );
    }

    #[test]
    fn pwm_ringing_adds_peak_at_f_ring() {
        // 64 cycles × 256 samples/cycle = 16384 samples at 128 MHz
        // (m × fsw = 256 × 500 kHz). Nyquist 64 MHz, so the test
        // f_ring at ~30 MHz is comfortably in band.
        let stream = const_stream(64, 0.5, 10.0);
        // Pick L/C for f_ring = 30 MHz: ω² = 1/LC = (2π·30e6)² →
        // LC = 2.81e-17; pick L=10 nH → C = 2.81e-9 F. C is much
        // bigger than realistic Coss, but gives an isolated test
        // bin we can target unambiguously.
        let ring = RingingParams {
            l_loop_h: 10e-9,
            r_loop_ohm: 1e-3,
            c_oss_total_f: 2.81e-9,
            v_step_v: 12.0,
        };
        let spec_no_ring = export_input_current_spectrum(
            &stream,
            &default_params(),
            ExportMode::PwmReconstructed {
                samples_per_cycle: 256,
                trapezoidal: true,
                ringing: None,
                qrr_nc: 0.0,
                trr_ns: 0.0,
            },
        )
        .unwrap();
        let spec_ring = export_input_current_spectrum(
            &stream,
            &default_params(),
            ExportMode::PwmReconstructed {
                samples_per_cycle: 256,
                trapezoidal: true,
                ringing: Some(ring),
                qrr_nc: 0.0,
                trr_ns: 0.0,
            },
        )
        .unwrap();
        let mag = |s: &PortCurrentSpectrum, k: usize| {
            (s.i_re_amp[k].powi(2) + s.i_im_amp[k].powi(2)).sqrt()
        };
        let bin_step = spec_ring.freqs_hz[1] - spec_ring.freqs_hz[0];
        let k_ring = (ring.f_ring_hz() / bin_step).round() as usize;
        let mag_at_fring_no_ring = mag(&spec_no_ring, k_ring);
        let mag_at_fring_ring = mag(&spec_ring, k_ring);
        assert!(
            mag_at_fring_ring > 3.0 * mag_at_fring_no_ring,
            "ringing should boost f_ring bin: ring={} vs no_ring={}",
            mag_at_fring_ring,
            mag_at_fring_no_ring,
        );
    }

    #[test]
    fn miller_plateau_changes_waveform_at_high_voltage() {
        // 400 V offline PFC scenario where Miller plateau is real:
        // Q_gd = 30 nC, R_g = 5 Ω, V_drive = 12 V, V_miller = 6 V
        //   → t_miller = 30e-9 · 5 / 6 = 25 ns
        // That's a measurable shoulder relative to T_sw = 10 µs
        // (100 kHz typical PFC fsw).
        let mut p = default_params();
        p.f_sw_khz = 100.0;
        p.hs_fet.t_rise_ns = 50.0;
        p.hs_fet.t_fall_ns = 50.0;
        p.hs_fet.qgd_nc = 30.0;
        p.hs_fet.rg_ohm = 5.0;
        p.hs_fet.vgs_v = 12.0;
        p.hs_fet.v_miller_v = 6.0;

        let stream = const_stream(64, 0.5, 10.0);
        let mut p_no_miller = p.clone();
        p_no_miller.hs_fet.qgd_nc = 0.0;
        let spec_no = export_input_current_spectrum(
            &stream,
            &p_no_miller,
            ExportMode::PwmReconstructed {
                samples_per_cycle: 256,
                trapezoidal: true,
                ringing: None,
                qrr_nc: 0.0,
                trr_ns: 0.0,
            },
        )
        .unwrap();
        let spec_miller = export_input_current_spectrum(
            &stream,
            &p,
            ExportMode::PwmReconstructed {
                samples_per_cycle: 256,
                trapezoidal: true,
                ringing: None,
                qrr_nc: 0.0,
                trr_ns: 0.0,
            },
        )
        .unwrap();
        // Spectra must differ somewhere — Miller adds two constant-
        // current shoulders, which redistribute spectral energy
        // (the inductor-ramp section shrinks).
        let total_diff: f64 = spec_no
            .i_re_amp
            .iter()
            .zip(spec_miller.i_re_amp.iter())
            .map(|(a, b)| (a - b).abs())
            .sum();
        assert!(
            total_diff > 1e-2,
            "Miller plateau should change the spectrum; total |Δ| = {}",
            total_diff,
        );
        // Stamped tag should report the t_miller computation.
        assert!(
            spec_miller.source.contains("miller="),
            "source string should record the t_miller width: {}",
            spec_miller.source,
        );
    }

    #[test]
    fn miller_plateau_disabled_when_qgd_zero() {
        let stream = const_stream(64, 0.5, 10.0);
        let spec = export_input_current_spectrum(
            &stream,
            &default_params(),
            ExportMode::PwmReconstructed {
                samples_per_cycle: 64,
                trapezoidal: true,
                ringing: None,
                qrr_nc: 0.0,
                trr_ns: 0.0,
            },
        )
        .unwrap();
        // Default FetProfile has qgd_nc = 0 → no Miller tag.
        assert!(
            !spec.source.contains("miller="),
            "no miller tag expected when qgd_nc = 0: {}",
            spec.source,
        );
    }

    #[test]
    fn qrr_spike_adds_high_harmonic_content() {
        let stream = const_stream(64, 0.5, 10.0);
        let spec_no_qrr = export_input_current_spectrum(
            &stream,
            &default_params(),
            ExportMode::PwmReconstructed {
                samples_per_cycle: 256,
                trapezoidal: true,
                ringing: None,
                qrr_nc: 0.0,
                trr_ns: 0.0,
            },
        )
        .unwrap();
        let spec_qrr = export_input_current_spectrum(
            &stream,
            &default_params(),
            ExportMode::PwmReconstructed {
                samples_per_cycle: 256,
                trapezoidal: true,
                ringing: None,
                qrr_nc: 100.0, // 100 nC — typical Si MOSFET body diode
                trr_ns: 30.0,  // 30 ns recovery — typical t_rr
            },
        )
        .unwrap();
        // Sum magnitude² across the 5–30 MHz band. A narrow Qrr
        // triangle has broadband sinc content that lifts the
        // mid-band energy.
        let band_energy = |s: &PortCurrentSpectrum| -> f64 {
            let mut e = 0.0;
            for (i, &f) in s.freqs_hz.iter().enumerate() {
                if (5e6..=30e6).contains(&f) {
                    e += s.i_re_amp[i].powi(2) + s.i_im_amp[i].powi(2);
                }
            }
            e
        };
        let e_no = band_energy(&spec_no_qrr);
        let e_qrr = band_energy(&spec_qrr);
        assert!(
            e_qrr > 1.5 * e_no,
            "Qrr should raise 5–30 MHz energy: with={}, without={}",
            e_qrr,
            e_no,
        );
    }

    #[test]
    fn pwm_constant_stream_shows_fsw_harmonic_peak() {
        // 64 cycles, 32 samples/cycle → 2048 total samples; fsw at
        // bin index 64 of the one-sided 1025-bin spectrum.
        let stream = const_stream(64, 0.5, 10.0);
        let spec = export_input_current_spectrum(
            &stream,
            &default_params(),
            ExportMode::PwmReconstructed {
                samples_per_cycle: 32,
                trapezoidal: false,
                ringing: None,
                qrr_nc: 0.0,
                trr_ns: 0.0,
            },
        )
        .unwrap();
        // fsw bin = (n_window_cycles × samples_per_cycle / 2) /
        // (samples_per_cycle / 2) = n_window_cycles. Find magnitude
        // there and at 2×fsw; both should be non-trivial.
        let fsw_hz = default_params().f_sw_khz * 1e3;
        let bin_step = spec.freqs_hz[1] - spec.freqs_hz[0];
        let fsw_bin = (fsw_hz / bin_step).round() as usize;
        let twosw_bin = ((2.0 * fsw_hz) / bin_step).round() as usize;
        let mag = |k: usize| {
            (spec.i_re_amp[k] * spec.i_re_amp[k] + spec.i_im_amp[k] * spec.i_im_amp[k]).sqrt()
        };
        let m_fsw = mag(fsw_bin);
        let m_2sw = mag(twosw_bin);
        let m_dc = spec.i_re_amp[0].abs();
        // Square wave half-duty: fsw bin should be the dominant AC
        // term (≈ A × 2/π ≈ 0.64×amplitude where amplitude ≈ 10 A).
        assert!(m_fsw > 3.0, "fsw bin magnitude {} too small", m_fsw);
        // 2×fsw is the second harmonic of a 50 %-duty PWM — *exactly*
        // zero in the closed form. With finite-step reconstruction +
        // Hann window it's small but non-zero; require it < 25 % of
        // fsw bin magnitude to confirm we're in the right regime.
        assert!(
            m_2sw < 0.25 * m_fsw,
            "2nd harmonic {} should be << fsw bin {}",
            m_2sw,
            m_fsw,
        );
        assert!(m_dc > 1.0, "DC bin {} too small", m_dc);
    }

    #[test]
    fn trapezoidal_mode_runs_and_differs_from_sharp() {
        // Smoke test: trapezoidal mode produces a spectrum that
        // (a) is finite, (b) differs from sharp-edge mode in the
        // high-frequency band (the precise physics — DC shift from
        // edge-area-correction vs band-energy attenuation — depends
        // on the chosen rise/fall ratio and is documented in
        // spectrum_export.rs's module docs).
        let n_cycles = 64;
        let m = 128;
        let stream = const_stream(n_cycles, 0.5, 10.0);
        let mut p_slow = default_params();
        p_slow.hs_fet.t_rise_ns = 200.0;
        p_slow.hs_fet.t_fall_ns = 200.0;
        let spec_sharp = export_input_current_spectrum(
            &stream,
            &default_params(),
            ExportMode::PwmReconstructed {
                samples_per_cycle: m,
                trapezoidal: false,
                ringing: None,
                qrr_nc: 0.0,
                trr_ns: 0.0,
            },
        )
        .unwrap();
        let spec_trap = export_input_current_spectrum(
            &stream,
            &p_slow,
            ExportMode::PwmReconstructed {
                samples_per_cycle: m,
                trapezoidal: true,
                ringing: None,
                qrr_nc: 0.0,
                trr_ns: 0.0,
            },
        )
        .unwrap();
        assert!(spec_trap.i_re_amp.iter().all(|x| x.is_finite()));
        assert!(spec_trap.i_im_amp.iter().all(|x| x.is_finite()));
        // The spectra must differ — if they didn't, trapezoidal mode
        // would be a no-op.
        let mut total_diff = 0.0_f64;
        for k in 0..spec_sharp.freqs_hz.len() {
            total_diff += (spec_sharp.i_re_amp[k] - spec_trap.i_re_amp[k]).abs();
        }
        assert!(
            total_diff > 1e-3,
            "trapezoidal mode should differ from sharp mode, total |Δ| = {}",
            total_diff,
        );
    }

    #[test]
    fn rejects_short_capture() {
        let stream = const_stream(4, 0.5, 10.0); // < 16 cycles
        let r = export_input_current_spectrum(&stream, &default_params(), ExportMode::Envelope);
        assert!(r.is_err());
    }
}
