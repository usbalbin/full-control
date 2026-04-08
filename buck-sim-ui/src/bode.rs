use std::f64::consts::PI;

use egui::Color32;
use egui_plot::{HLine, Line, PlotPoints, VLine};
use electronics_sim::cap_bank::{CapBank, CapType};
use full_control::control_2p2z::DesignSummary;

use crate::sim::{SimParams, has_transport_delays};

/// Number of frequency points in the sweep.
const N_POINTS: usize = 500;

/// A complex number represented as (re, im).
type C = (f64, f64);

const fn c_mul(a: C, b: C) -> C {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

const fn c_div(a: C, b: C) -> C {
    let denom = b.0 * b.0 + b.1 * b.1;
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

fn c_mag(a: C) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

fn c_phase_deg(a: C) -> f64 {
    a.1.atan2(a.0).to_degrees()
}

/// Non-ideal effects that modify the loop gain but not the ideal plant/compensator.
pub struct NonIdealParams {
    /// Total transport delay [s]: t_adc + t_processing + t_dac + t_hold.
    /// Zero when PhaseMargin::Manual (ideal).
    pub tau_delay: f64,
    /// ZOH control period [s] = cycles_per_tick / f_sw.
    pub t_ctrl: f64,
    /// Current sensor −3 dB bandwidth [rad/s]. 0 = ideal.
    pub omega_cs: f64,
    /// DAC output filter −3 dB bandwidth [rad/s]. 0 = ideal.
    pub omega_dac: f64,
}

impl NonIdealParams {
    pub fn from_sim_params(p: &SimParams) -> Self {
        let f_sw = p.f_sw_khz * 1e3;
        let tau_delay = if has_transport_delays(&p.mcu, &p.dac) {
            let t_adc = p.mcu.t_adc_us * 1e-6;
            let t_processing = p.mcu.t_processing_us * 1e-6;
            let t_dac = p.dac.t_dac_us * 1e-6;
            let t_hold = (p.cycles_per_tick as f64 - 1.0) / f_sw;
            t_adc + t_processing + t_dac + t_hold
        } else {
            0.0
        };
        let t_ctrl = p.cycles_per_tick as f64 / f_sw;
        let omega_cs = if p.cs.cs_bandwidth_khz > 0.0 {
            2.0 * PI * p.cs.cs_bandwidth_khz * 1e3
        } else {
            0.0
        };
        let omega_dac = if p.dac.dac_filter_bw_khz > 0.0 {
            2.0 * PI * p.dac.dac_filter_bw_khz * 1e3
        } else {
            0.0
        };
        Self { tau_delay, t_ctrl, omega_cs, omega_dac }
    }
}

/// Evaluate the combined non-ideal transfer function at angular frequency ω.
///
/// Returns a complex number (re, im) representing the product of:
/// - Transport delay: e^{−jωτ}
/// - ZOH sinc droop: sinc(ω·T_ctrl/2) (magnitude only, phase already in τ)
/// - Current sensor pole: 1 / (1 + jω/ω_cs)
/// - DAC filter pole: 1 / (1 + jω/ω_dac)
fn non_ideal(omega: f64, ni: &NonIdealParams) -> C {
    let mut result: C = (1.0, 0.0);

    // Transport delay: e^{-jωτ}
    if ni.tau_delay > 0.0 {
        let phi = omega * ni.tau_delay;
        result = c_mul(result, (phi.cos(), -phi.sin()));
    }

    // ZOH sinc droop (magnitude only — phase lag already captured in tau_delay)
    if ni.t_ctrl > 0.0 {
        let x = omega * ni.t_ctrl / 2.0;
        let sinc = if x.abs() < 1e-12 { 1.0 } else { x.sin() / x };
        result = (result.0 * sinc, result.1 * sinc);
    }

    // Current sensor first-order pole: 1 / (1 + jω/ω_cs)
    if ni.omega_cs > 0.0 {
        let pole: C = (1.0, omega / ni.omega_cs);
        result = c_div(result, pole);
    }

    // DAC filter first-order pole: 1 / (1 + jω/ω_dac)
    if ni.omega_dac > 0.0 {
        let pole: C = (1.0, omega / ni.omega_dac);
        result = c_div(result, pole);
    }

    result
}

/// Pre-computed Bode sweep data.
pub struct BodeData {
    /// Frequency points [Hz].
    pub freq: Vec<f64>,
    /// Plant magnitude [dB].
    pub plant_mag: Vec<f64>,
    /// Plant phase [deg].
    pub plant_phase: Vec<f64>,
    /// Compensator magnitude [dB].
    pub comp_mag: Vec<f64>,
    /// Compensator phase [deg].
    pub comp_phase: Vec<f64>,
    /// Loop (T = Plant × Comp) magnitude [dB].
    pub loop_mag: Vec<f64>,
    /// Loop phase [deg].
    pub loop_phase: Vec<f64>,

    // Key frequencies [Hz] for annotation.
    pub f_p1: f64,
    pub f_esr: f64,
    pub f_cz1: f64,
    pub f_cp1: f64,
    pub f_n: f64,
    pub f_sw: f64,
    pub f_x_design: f64,
    pub h_dc: f64,

    // Non-ideal annotation frequencies [Hz] for vertical markers.
    pub f_cs: Option<f64>,
    pub f_dac_filter: Option<f64>,
    /// Transport delay [µs] for summary display.
    pub tau_delay_us: f64,

    // Measured from sweep.
    pub f_x_actual: f64,
    pub phase_margin_deg: f64,
    pub gain_margin_db: f64,
}

/// Evaluate the plant transfer function H_plant(jω).
///
/// H_plant = h_dc × (1 + jω/ω_esr) / (1 + jω/ω_p1) × 1/((jω/ω_n)² + jω/ω_n + 1)
fn plant(omega: f64, ds: &DesignSummary) -> C {
    let jw_over_esr: C = (1.0, omega / ds.omega_esr);
    let jw_over_p1: C = (1.0, omega / ds.omega_p1);

    let r = omega / ds.omega_n;
    // (jω/ω_n)² = -r², so denominator = (1 - r²) + j·r
    let double_pole: C = (1.0 - r * r, r);

    let num = (ds.h_dc, 0.0);
    let num = c_mul(num, jw_over_esr);
    let den = c_mul(jw_over_p1, double_pole);
    c_div(num, den)
}

/// Evaluate the plant transfer function using composite cap bank impedance.
///
/// Instead of a single ESR zero `(1 + jω/ω_esr)`, we use the actual impedance
/// of the parallel cap bank:
///
///   Z_out_normalized(jω) = Z_out(jω) × jω × C_total
///
/// This equals 1 at DC (pure capacitive) and shows the real ESR/ESL behavior
/// at higher frequencies, including resonances and anti-resonances from mixed
/// cap types.
fn plant_cap_bank(omega: f64, ds: &DesignSummary, caps: &[CapType], c_total: f64) -> C {
    // Compute composite impedance of the cap bank at this frequency.
    let (z_re, z_im) = CapBank::impedance_at(caps, omega);

    // Normalize: multiply by jω × C_total.
    // This makes the impedance transfer function dimensionless and equal to 1 at DC.
    //   Z_norm = Z_out × jω × C_total
    //          = (z_re + j·z_im) × (0 + j·ω·C_total)
    //          = (-z_im·ω·C_total) + j·(z_re·ω·C_total)
    let wc = omega * c_total;
    let z_norm: C = (-z_im * wc, z_re * wc);

    // Rest of the plant: H_dc / [(1 + jω/ω_p1) × double_pole]
    // The ESR zero is now replaced by z_norm above.
    let jw_over_p1: C = (1.0, omega / ds.omega_p1);
    let r = omega / ds.omega_n;
    let double_pole: C = (1.0 - r * r, r);

    let num = c_mul((ds.h_dc, 0.0), z_norm);
    let den = c_mul(jw_over_p1, double_pole);
    c_div(num, den)
}

/// Evaluate the compensator transfer function H_c(jω).
///
/// H_c = ω_cp0/(jω) × (1 + jω/ω_cz1) / (1 + jω/ω_cp1)
fn compensator(omega: f64, ds: &DesignSummary) -> C {
    let jw: C = (0.0, omega);
    let integrator = c_div((ds.omega_cp0, 0.0), jw);
    let zero: C = (1.0, omega / ds.omega_cz1);
    let pole: C = (1.0, omega / ds.omega_cp1);
    c_mul(integrator, c_div(zero, pole))
}

impl BodeData {
    /// Compute Bode data. When `cap_bank_caps` is Some, the plant uses composite
    /// impedance instead of a single ESR zero.
    pub fn compute(ds: &DesignSummary, ni: &NonIdealParams) -> Self {
        Self::compute_inner(ds, ni, None)
    }

    /// Compute Bode data with composite cap bank impedance in the plant.
    pub fn compute_with_cap_bank(
        ds: &DesignSummary,
        ni: &NonIdealParams,
        caps: &[CapType],
    ) -> Self {
        let c_total = CapBank::total_capacitance(caps);
        Self::compute_inner(ds, ni, Some((caps, c_total)))
    }

    fn compute_inner(
        ds: &DesignSummary,
        ni: &NonIdealParams,
        cap_bank: Option<(&[CapType], f64)>,
    ) -> Self {
        let f_min = 1.0_f64;
        let f_max = ds.f_sw;
        let log_min = f_min.log10();
        let log_max = f_max.log10();

        let mut freq = Vec::with_capacity(N_POINTS);
        let mut plant_mag = Vec::with_capacity(N_POINTS);
        let mut plant_phase = Vec::with_capacity(N_POINTS);
        let mut comp_mag = Vec::with_capacity(N_POINTS);
        let mut comp_phase = Vec::with_capacity(N_POINTS);
        let mut loop_mag = Vec::with_capacity(N_POINTS);
        let mut loop_phase = Vec::with_capacity(N_POINTS);

        for i in 0..N_POINTS {
            let t = i as f64 / (N_POINTS - 1) as f64;
            let f = 10.0_f64.powf(log_min + t * (log_max - log_min));
            let omega = 2.0 * PI * f;

            let hp = match cap_bank {
                Some((caps, c_total)) => plant_cap_bank(omega, ds, caps, c_total),
                None => plant(omega, ds),
            };
            let hc = compensator(omega, ds);
            let hni = non_ideal(omega, ni);
            let ht = c_mul(c_mul(hp, hc), hni);

            freq.push(f);
            plant_mag.push(20.0 * c_mag(hp).log10());
            plant_phase.push(c_phase_deg(hp));
            comp_mag.push(20.0 * c_mag(hc).log10());
            comp_phase.push(c_phase_deg(hc));
            loop_mag.push(20.0 * c_mag(ht).log10());
            loop_phase.push(c_phase_deg(ht));
        }

        // Find actual crossover: first point where loop_mag crosses 0 dB downward.
        let mut f_x_actual = ds.omega_x / (2.0 * PI);
        let mut phase_margin_deg = ds.phase_margin_rad.to_degrees();
        for i in 1..N_POINTS {
            if loop_mag[i - 1] >= 0.0 && loop_mag[i] < 0.0 {
                // Linear interpolation for the crossing frequency.
                let frac = loop_mag[i - 1] / (loop_mag[i - 1] - loop_mag[i]);
                f_x_actual = freq[i - 1] + frac * (freq[i] - freq[i - 1]);
                let phase_at_cross = loop_phase[i - 1] + frac * (loop_phase[i] - loop_phase[i - 1]);
                phase_margin_deg = 180.0 + phase_at_cross;
                break;
            }
        }

        // Find gain margin: loop gain at the frequency where phase = -180°.
        let mut gain_margin_db = f64::INFINITY;
        for i in 1..N_POINTS {
            if loop_phase[i - 1] > -180.0 && loop_phase[i] <= -180.0 {
                let frac = (loop_phase[i - 1] + 180.0) / (loop_phase[i - 1] - loop_phase[i]);
                let mag_at_cross = loop_mag[i - 1] + frac * (loop_mag[i] - loop_mag[i - 1]);
                gain_margin_db = -mag_at_cross;
                break;
            }
        }

        let f_cs = if ni.omega_cs > 0.0 { Some(ni.omega_cs / (2.0 * PI)) } else { None };
        let f_dac_filter = if ni.omega_dac > 0.0 { Some(ni.omega_dac / (2.0 * PI)) } else { None };

        BodeData {
            freq,
            plant_mag,
            plant_phase,
            comp_mag,
            comp_phase,
            loop_mag,
            loop_phase,
            f_p1: ds.omega_p1 / (2.0 * PI),
            f_esr: ds.omega_esr / (2.0 * PI),
            f_cz1: ds.omega_cz1 / (2.0 * PI),
            f_cp1: ds.omega_cp1 / (2.0 * PI),
            f_n: ds.omega_n / (2.0 * PI),
            f_sw: ds.f_sw,
            f_x_design: ds.omega_x / (2.0 * PI),
            h_dc: ds.h_dc,
            f_cs,
            f_dac_filter,
            tau_delay_us: ni.tau_delay * 1e6,
            f_x_actual,
            phase_margin_deg,
            gain_margin_db,
        }
    }
}

fn to_log_points(freq: &[f64], values: &[f64]) -> Vec<[f64; 2]> {
    freq.iter()
        .zip(values.iter())
        .map(|(&f, &v)| [f.log10(), v])
        .collect()
}

/// Format a frequency nicely for labels.
fn fmt_freq(f: f64) -> String {
    if f >= 1e6 {
        format!("{:.1} MHz", f / 1e6)
    } else if f >= 1e3 {
        format!("{:.1} kHz", f / 1e3)
    } else {
        format!("{:.0} Hz", f)
    }
}

const PLANT_COLOR: Color32 = Color32::from_rgb(80, 140, 255);
const COMP_COLOR: Color32 = Color32::from_rgb(80, 200, 120);
const LOOP_COLOR: Color32 = Color32::from_rgb(255, 160, 50);

/// Render the Bode plot into the given `Ui`.
pub fn show_bode(ui: &mut egui::Ui, data: &BodeData) {
    let total_h = ui.available_height();
    let mag_h = (total_h * 0.55).max(120.0);
    let phase_h = (total_h * 0.40).max(80.0);

    // Annotation lines at key frequencies (in log10 space).
    let mut markers: Vec<(f64, &str, Color32)> = vec![
        (data.f_p1, "f_p1", Color32::from_rgb(180, 180, 180)),
        (data.f_esr, "f_esr", Color32::from_rgb(180, 180, 180)),
        (data.f_cz1, "f_cz1", Color32::from_rgb(180, 180, 180)),
        (data.f_x_actual, "f_x", Color32::from_rgb(255, 80, 80)),
    ];
    if let Some(f) = data.f_cs {
        markers.push((f, "f_cs", Color32::from_rgb(200, 140, 255)));
    }
    if let Some(f) = data.f_dac_filter {
        markers.push((f, "f_dac", Color32::from_rgb(140, 200, 255)));
    }

    // Summary text
    ui.horizontal(|ui| {
        let mut text = format!(
            "f_x = {}   PM = {:.1}°   GM = {:.1} dB",
            fmt_freq(data.f_x_actual),
            data.phase_margin_deg,
            data.gain_margin_db,
        );
        if data.tau_delay_us > 0.0 {
            text.push_str(&format!("   τ = {:.1} µs", data.tau_delay_us));
        }
        ui.label(egui::RichText::new(text).strong());
    });

    // Pole/zero table
    egui::CollapsingHeader::new("Poles & Zeros")
        .default_open(true)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(format!(
                    "Plant:  pole {}, double-pole {}, zero {} (ESR), H_dc = {:.1} dB",
                    fmt_freq(data.f_p1),
                    fmt_freq(data.f_n),
                    fmt_freq(data.f_esr),
                    20.0 * data.h_dc.log10(),
                )).small().monospace());
            });
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(format!(
                    "Comp:   zero {}, pole {} (cancels ESR zero)",
                    fmt_freq(data.f_cz1),
                    fmt_freq(data.f_cp1),
                )).small().monospace());
            });
        });
    ui.add_space(2.0);

    let x_link = egui::Vec2b::new(true, false);

    // ── Magnitude plot ──────────────────────────────────────────────────
    egui_plot::Plot::new("bode_mag")
        .height(mag_h)
        .y_axis_label("Magnitude [dB]")
        .x_axis_label("")
        .link_axis("bode_freq", x_link)
        .x_axis_formatter(log10_freq_formatter)
        .label_formatter(bode_label_formatter)
        .show(ui, |plot_ui| {
            plot_ui.line(
                Line::new("Plant", PlotPoints::new(to_log_points(&data.freq, &data.plant_mag)))
                    .color(PLANT_COLOR),
            );
            plot_ui.line(
                Line::new("Compensator", PlotPoints::new(to_log_points(&data.freq, &data.comp_mag)))
                    .color(COMP_COLOR),
            );
            plot_ui.line(
                Line::new("Loop T", PlotPoints::new(to_log_points(&data.freq, &data.loop_mag)))
                    .color(LOOP_COLOR)
                    .width(2.0),
            );
            plot_ui.hline(
                HLine::new("0 dB", 0.0)
                    .color(Color32::from_rgb(100, 100, 100))
                    .style(egui_plot::LineStyle::dashed_dense()),
            );
            for &(f, label, color) in &markers {
                plot_ui.vline(
                    VLine::new(label, f.log10())
                        .color(color)
                        .style(egui_plot::LineStyle::dashed_dense()),
                );
            }
        });

    // ── Phase plot ──────────────────────────────────────────────────────
    egui_plot::Plot::new("bode_phase")
        .height(phase_h)
        .y_axis_label("Phase [°]")
        .x_axis_label("Frequency")
        .link_axis("bode_freq", x_link)
        .x_axis_formatter(log10_freq_formatter)
        .label_formatter(bode_label_formatter)
        .show(ui, |plot_ui| {
            plot_ui.line(
                Line::new("Plant", PlotPoints::new(to_log_points(&data.freq, &data.plant_phase)))
                    .color(PLANT_COLOR),
            );
            plot_ui.line(
                Line::new("Compensator", PlotPoints::new(to_log_points(&data.freq, &data.comp_phase)))
                    .color(COMP_COLOR),
            );
            plot_ui.line(
                Line::new("Loop T", PlotPoints::new(to_log_points(&data.freq, &data.loop_phase)))
                    .color(LOOP_COLOR)
                    .width(2.0),
            );
            plot_ui.hline(
                HLine::new("-180°", -180.0)
                    .color(Color32::from_rgb(255, 80, 80))
                    .style(egui_plot::LineStyle::dashed_dense()),
            );
            for &(f, label, color) in &markers {
                plot_ui.vline(
                    VLine::new(label, f.log10())
                        .color(color)
                        .style(egui_plot::LineStyle::dashed_dense()),
                );
            }
        });
}

/// Format the x-axis (log10 of frequency) as human-readable Hz/kHz/MHz.
fn log10_freq_formatter(
    mark: egui_plot::GridMark,
    _range: &std::ops::RangeInclusive<f64>,
) -> String {
    let f = 10.0_f64.powf(mark.value);
    fmt_freq(f)
}

/// Format cursor hover labels: show frequency + value.
fn bode_label_formatter(name: &str, point: &egui_plot::PlotPoint) -> String {
    let f = 10.0_f64.powf(point.x);
    format!("{}\n{}\n{:.1}", name, fmt_freq(f), point.y)
}
