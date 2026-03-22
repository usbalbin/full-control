use std::f64::consts::PI;

use egui::Color32;
use egui_plot::{HLine, Line, PlotPoints, VLine};
use full_control::control_2p2z::DesignSummary;

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
    pub f_x_design: f64,

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
    pub fn compute(ds: &DesignSummary) -> Self {
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

            let hp = plant(omega, ds);
            let hc = compensator(omega, ds);
            let ht = c_mul(hp, hc);

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
            f_x_design: ds.omega_x / (2.0 * PI),
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
    let markers: Vec<(f64, &str, Color32)> = vec![
        (data.f_p1, "f_p1", Color32::from_rgb(180, 180, 180)),
        (data.f_esr, "f_esr", Color32::from_rgb(180, 180, 180)),
        (data.f_cz1, "f_cz1", Color32::from_rgb(180, 180, 180)),
        (data.f_x_actual, "f_x", Color32::from_rgb(255, 80, 80)),
    ];

    // Summary text
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new(format!(
                "f_x = {}   PM = {:.1}°   GM = {:.1} dB",
                fmt_freq(data.f_x_actual),
                data.phase_margin_deg,
                data.gain_margin_db,
            ))
            .strong(),
        );
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
