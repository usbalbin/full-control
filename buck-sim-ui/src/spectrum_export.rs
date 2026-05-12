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
//!   for a synchronous buck. Does *not* model FET dV/dt rise/fall
//!   broadband content (that's path 3 future work, requires sub-cycle
//!   sampling inside `sim::run_simulation`).
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

/// Reconstruction strategy. See module docs.
#[derive(Debug, Clone, Copy)]
pub enum ExportMode {
    Envelope,
    PwmReconstructed { samples_per_cycle: u32 },
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
        ExportMode::PwmReconstructed { samples_per_cycle } => {
            if samples_per_cycle < 4 {
                return Err("samples_per_cycle must be >= 4".into());
            }
            let m = samples_per_cycle as usize;
            let mut s: Vec<f64> = Vec::with_capacity(m * cycles.len());
            for c in cycles {
                let duty = (c.duty_pct_avg() as f64 / 100.0).clamp(0.0, 1.0);
                // Per-phase ramp; for multi-phase the i_total min/max
                // are interleaved sums, divide back out. Approximation:
                // assumes all phases share the same duty + ramp shape,
                // which is true for steady-state symmetric phase
                // interleaving but lossy for transient asymmetry.
                let n_phases = c.phases.len().max(1) as f64;
                let i_min = c.i_total_min as f64 / n_phases;
                let i_max = c.i_total_max as f64 / n_phases;
                for k in 0..m {
                    let tau = (k as f64) / (m as f64); // ∈ [0, 1)
                    let v = if tau < duty && duty > 0.0 {
                        let frac = tau / duty;
                        // Multiply by num_phases to recover the
                        // interleaved-sum input current seen by the
                        // shared input cap.
                        n_phases * (i_min + frac * (i_max - i_min))
                    } else {
                        0.0
                    };
                    s.push(v);
                }
            }
            (
                s,
                fsw * m as f64,
                format!("mode=pwm-reconstructed,samples_per_cycle={}", m),
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
    fn pwm_constant_stream_shows_fsw_harmonic_peak() {
        // 64 cycles, 32 samples/cycle → 2048 total samples; fsw at
        // bin index 64 of the one-sided 1025-bin spectrum.
        let stream = const_stream(64, 0.5, 10.0);
        let spec = export_input_current_spectrum(
            &stream,
            &default_params(),
            ExportMode::PwmReconstructed { samples_per_cycle: 32 },
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
    fn rejects_short_capture() {
        let stream = const_stream(4, 0.5, 10.0); // < 16 cycles
        let r = export_input_current_spectrum(&stream, &default_params(), ExportMode::Envelope);
        assert!(r.is_err());
    }
}
