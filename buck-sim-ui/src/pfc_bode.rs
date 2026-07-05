use std::f64::consts::PI;

use egui::{Color32, Vec2b};
use egui_plot::{HLine, Line, PlotPoints, VLine};
use full_control::control_pfc::PfcDesignSummary;

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

/// Pre-computed Bode sweep data for PFC inner and outer loops.
pub struct PfcBodeData {
    pub freq: Vec<f64>,

    // Inner current loop
    pub inner_plant_mag: Vec<f64>,
    pub inner_plant_phase: Vec<f64>,
    pub inner_comp_mag: Vec<f64>,
    pub inner_comp_phase: Vec<f64>,
    pub inner_loop_mag: Vec<f64>,
    pub inner_loop_phase: Vec<f64>,
    pub inner_f_x_actual: f64,
    pub inner_phase_margin_deg: f64,
    pub inner_gain_margin_db: f64,

    // Outer voltage loop
    pub outer_plant_mag: Vec<f64>,
    pub outer_plant_phase: Vec<f64>,
    pub outer_comp_mag: Vec<f64>,
    pub outer_comp_phase: Vec<f64>,
    pub outer_loop_mag: Vec<f64>,
    pub outer_loop_phase: Vec<f64>,
    pub outer_f_x_actual: f64,
    pub outer_phase_margin_deg: f64,
    pub outer_gain_margin_db: f64,

    // Output impedance
    pub z_out_open_db: Vec<f64>,
    pub z_out_closed_db: Vec<f64>,

    // Key frequencies for markers
    pub f_sw: f64,
    pub f_line: f64,
}

/// Inner current loop plant: G_id(jω) = V_out / (jω × L)
fn inner_plant(omega: f64, ds: &PfcDesignSummary) -> C {
    // V_out / (jω × L) = (V_out/L) / (jω) = plant_gain / (jω)
    let jw: C = (0.0, omega);
    c_div((ds.inner_plant_gain, 0.0), jw)
}

/// Inner current loop compensator: H_ci(jω) = ω_cp0/(jω) × (1 + jω/ω_cz1) / (1 + jω/ω_cp1)
fn inner_compensator(omega: f64, ds: &PfcDesignSummary) -> C {
    let jw: C = (0.0, omega);
    let integrator = c_div((ds.inner_omega_cp0, 0.0), jw);
    let zero: C = (1.0, omega / ds.inner_omega_cz1);
    let pole: C = (1.0, omega / ds.inner_omega_cp1);
    c_mul(integrator, c_div(zero, pole))
}

/// Loop transport-delay term e^{-jω·τ}, using the delay the design actually
/// reserved (`PfcDesignSummary::inner_loop_delay_s`) so the Bode plot's phase
/// margin matches the design intent. `τ = 0` (ideal profile) ⇒ no delay term.
fn transport_delay(omega: f64, delay_s: f64) -> C {
    let phi = -omega * delay_s;
    (phi.cos(), phi.sin())
}

/// Outer voltage loop plant: G_vi(jω) = V_pk / (2 × V_out × jω × C_out)
///                                     = outer_plant_gain / (jω × C_out)
/// With ESR zero: × (1 + jω × R_esr × C_out)
fn outer_plant(omega: f64, ds: &PfcDesignSummary) -> C {
    let jw: C = (0.0, omega);
    let base = c_div((ds.outer_plant_gain, 0.0), c_mul(jw, (ds.c_out, 0.0)));
    let esr_zero: C = (1.0, omega * ds.r_esr_out * ds.c_out);
    c_mul(base, esr_zero)
}

/// Outer voltage loop compensator: same Type II structure
fn outer_compensator(omega: f64, ds: &PfcDesignSummary) -> C {
    let jw: C = (0.0, omega);
    let integrator = c_div((ds.outer_omega_cp0, 0.0), jw);
    let zero: C = (1.0, omega / ds.outer_omega_cz1);
    let pole: C = (1.0, omega / ds.outer_omega_cp1);
    c_mul(integrator, c_div(zero, pole))
}

impl PfcBodeData {
    pub fn compute(ds: &PfcDesignSummary, r_load: f64) -> Self {
        let f_min = 1.0_f64;
        let f_max = ds.f_sw;
        let log_min = f_min.log10();
        let log_max = f_max.log10();

        let mut freq = Vec::with_capacity(N_POINTS);
        let mut inner_plant_mag = Vec::with_capacity(N_POINTS);
        let mut inner_plant_phase = Vec::with_capacity(N_POINTS);
        let mut inner_comp_mag = Vec::with_capacity(N_POINTS);
        let mut inner_comp_phase = Vec::with_capacity(N_POINTS);
        let mut inner_loop_mag = Vec::with_capacity(N_POINTS);
        let mut inner_loop_phase = Vec::with_capacity(N_POINTS);
        let mut outer_plant_mag = Vec::with_capacity(N_POINTS);
        let mut outer_plant_phase = Vec::with_capacity(N_POINTS);
        let mut outer_comp_mag = Vec::with_capacity(N_POINTS);
        let mut outer_comp_phase = Vec::with_capacity(N_POINTS);
        let mut outer_loop_mag = Vec::with_capacity(N_POINTS);
        let mut outer_loop_phase = Vec::with_capacity(N_POINTS);
        let mut z_out_open_db = Vec::with_capacity(N_POINTS);
        let mut z_out_closed_db = Vec::with_capacity(N_POINTS);

        for i in 0..N_POINTS {
            let t = i as f64 / (N_POINTS - 1) as f64;
            let f = 10.0_f64.powf(log_min + t * (log_max - log_min));
            let omega = 2.0 * PI * f;

            // ── Inner current loop ────────────────────────────
            let hp_i = inner_plant(omega, ds);
            let hc_i = inner_compensator(omega, ds);
            let h_sense: C = (ds.r_sense, 0.0);
            let h_delay = transport_delay(omega, ds.inner_loop_delay_s);
            let ht_i = c_mul(c_mul(c_mul(hp_i, hc_i), h_sense), h_delay);

            inner_plant_mag.push(20.0 * c_mag(c_mul(hp_i, h_sense)).log10());
            inner_plant_phase.push(c_phase_deg(c_mul(hp_i, h_sense)));
            inner_comp_mag.push(20.0 * c_mag(hc_i).log10());
            inner_comp_phase.push(c_phase_deg(hc_i));
            inner_loop_mag.push(20.0 * c_mag(ht_i).log10());
            inner_loop_phase.push(c_phase_deg(ht_i));

            // ── Outer voltage loop ────────────────────────────
            let hp_v = outer_plant(omega, ds);
            let hc_v = outer_compensator(omega, ds);
            let ht_v = c_mul(hp_v, hc_v);

            outer_plant_mag.push(20.0 * c_mag(hp_v).log10());
            outer_plant_phase.push(c_phase_deg(hp_v));
            outer_comp_mag.push(20.0 * c_mag(hc_v).log10());
            outer_comp_phase.push(c_phase_deg(hc_v));
            outer_loop_mag.push(20.0 * c_mag(ht_v).log10());
            outer_loop_phase.push(c_phase_deg(ht_v));

            // ── Output impedance ──────────────────────────────
            // Z_cap = R_ESR + 1/(jωC)
            let z_cap: C = (ds.r_esr_out, -1.0 / (omega * ds.c_out));
            // Z_open = Z_cap ∥ R_load
            let z_open = c_div(
                c_mul(z_cap, (r_load, 0.0)),
                (z_cap.0 + r_load, z_cap.1),
            );
            // Z_closed = Z_open / (1 + T_v)
            let one_plus_tv: C = (1.0 + ht_v.0, ht_v.1);
            let z_closed = c_div(z_open, one_plus_tv);

            z_out_open_db.push(20.0 * c_mag(z_open).log10());
            z_out_closed_db.push(20.0 * c_mag(z_closed).log10());

            freq.push(f);
        }

        // Find actual crossover and margins for both loops
        let (inner_f_x_actual, inner_pm, inner_gm) =
            find_margins(&freq, &inner_loop_mag, &inner_loop_phase, ds.inner_omega_x);
        let (outer_f_x_actual, outer_pm, outer_gm) =
            find_margins(&freq, &outer_loop_mag, &outer_loop_phase, ds.outer_omega_x);

        PfcBodeData {
            freq,
            inner_plant_mag,
            inner_plant_phase,
            inner_comp_mag,
            inner_comp_phase,
            inner_loop_mag,
            inner_loop_phase,
            inner_f_x_actual,
            inner_phase_margin_deg: inner_pm,
            inner_gain_margin_db: inner_gm,
            outer_plant_mag,
            outer_plant_phase,
            outer_comp_mag,
            outer_comp_phase,
            outer_loop_mag,
            outer_loop_phase,
            outer_f_x_actual,
            outer_phase_margin_deg: outer_pm,
            outer_gain_margin_db: outer_gm,
            z_out_open_db,
            z_out_closed_db,
            f_sw: ds.f_sw,
            f_line: ds.f_line,
        }
    }
}

/// Find crossover frequency, phase margin, and gain margin from Bode data.
fn find_margins(
    freq: &[f64],
    loop_mag: &[f64],
    loop_phase: &[f64],
    omega_x_design: f64,
) -> (f64, f64, f64) {
    let f_x_design = omega_x_design / (2.0 * PI);

    // Find actual crossover: first 0dB downward crossing
    let mut f_x_actual = f_x_design;
    let mut phase_margin_deg = 60.0;
    for i in 1..freq.len() {
        if loop_mag[i - 1] >= 0.0 && loop_mag[i] < 0.0 {
            let frac = loop_mag[i - 1] / (loop_mag[i - 1] - loop_mag[i]);
            f_x_actual = freq[i - 1] + frac * (freq[i] - freq[i - 1]);
            let phase_at_cross = loop_phase[i - 1] + frac * (loop_phase[i] - loop_phase[i - 1]);
            phase_margin_deg = 180.0 + phase_at_cross;
            break;
        }
    }

    // Find gain margin: magnitude at -180° phase crossing. loop_phase is
    // atan2-wrapped to (-180, 180], so a raw scan for a downward -180 crossing
    // never fires (the value jumps to +180 at the wrap). Unwrap first.
    let mut unwrapped = loop_phase.to_vec();
    for i in 1..unwrapped.len() {
        let mut d = unwrapped[i] - unwrapped[i - 1];
        while d > 180.0 {
            unwrapped[i] -= 360.0;
            d -= 360.0;
        }
        while d < -180.0 {
            unwrapped[i] += 360.0;
            d += 360.0;
        }
    }
    let mut gain_margin_db = f64::INFINITY;
    for i in 1..freq.len() {
        if unwrapped[i - 1] > -180.0 && unwrapped[i] <= -180.0 {
            let frac = (unwrapped[i - 1] + 180.0) / (unwrapped[i - 1] - unwrapped[i]);
            let mag_at_cross = loop_mag[i - 1] + frac * (loop_mag[i] - loop_mag[i - 1]);
            gain_margin_db = -mag_at_cross;
            break;
        }
    }

    (f_x_actual, phase_margin_deg, gain_margin_db)
}

fn to_log_points(freq: &[f64], values: &[f64]) -> Vec<[f64; 2]> {
    freq.iter()
        .zip(values)
        .map(|(&f, &v)| [f.log10(), v])
        .collect()
}

/// Format a frequency for display.
fn fmt_freq(f: f64) -> String {
    if f >= 1e3 {
        format!("{:.1} kHz", f / 1e3)
    } else {
        format!("{:.1} Hz", f)
    }
}

// ── egui rendering ─────────────────────────────────────────────────────────

const PLANT_COLOR: Color32 = Color32::from_rgb(100, 180, 255);
const COMP_COLOR: Color32 = Color32::from_rgb(255, 180, 100);
const LOOP_COLOR: Color32 = Color32::from_rgb(100, 255, 100);
const ZOPEN_COLOR: Color32 = Color32::from_rgb(180, 180, 180);
const ZCLOSED_COLOR: Color32 = Color32::from_rgb(255, 100, 100);

/// Show the inner or outer loop Bode plot.
pub fn show_pfc_bode(
    ui: &mut egui::Ui,
    data: &PfcBodeData,
    inner: bool, // true = inner current loop, false = outer voltage loop
) {
    let (plant_mag, _plant_phase, comp_mag, _comp_phase, loop_mag, loop_phase,
         f_x_actual, pm_deg, gm_db, title) = if inner {
        (&data.inner_plant_mag, &data.inner_plant_phase,
         &data.inner_comp_mag, &data.inner_comp_phase,
         &data.inner_loop_mag, &data.inner_loop_phase,
         data.inner_f_x_actual, data.inner_phase_margin_deg, data.inner_gain_margin_db,
         "Inner Current Loop")
    } else {
        (&data.outer_plant_mag, &data.outer_plant_phase,
         &data.outer_comp_mag, &data.outer_comp_phase,
         &data.outer_loop_mag, &data.outer_loop_phase,
         data.outer_f_x_actual, data.outer_phase_margin_deg, data.outer_gain_margin_db,
         "Outer Voltage Loop")
    };

    // Summary line
    ui.label(format!(
        "{title}:  f_x = {}, PM = {pm_deg:.1}°, GM = {gm_db:.1} dB",
        fmt_freq(f_x_actual),
    ));

    let available = ui.available_size();
    let h_mag = available.y * 0.38;
    let h_phase = available.y * 0.28;
    let h_zout = available.y * 0.28;

    // ── Magnitude plot ────────────────────────────────────────
    let mag_plot = egui_plot::Plot::new(if inner { "pfc_mag_inner" } else { "pfc_mag_outer" })
        .height(h_mag)
        .x_axis_label("log₁₀(f)")
        .y_axis_label("dB")
        .show_axes([true, true])
        .link_axis(if inner { "pfc_inner_freq" } else { "pfc_outer_freq" }, Vec2b::new(true, false));

    mag_plot.show(ui, |plot_ui| {
        plot_ui.line(Line::new("Plant", PlotPoints::new(to_log_points(&data.freq, plant_mag)))
            .color(PLANT_COLOR));
        plot_ui.line(Line::new("Compensator", PlotPoints::new(to_log_points(&data.freq, comp_mag)))
            .color(COMP_COLOR));
        plot_ui.line(Line::new("Loop T", PlotPoints::new(to_log_points(&data.freq, loop_mag)))
            .color(LOOP_COLOR).width(2.0));
        plot_ui.hline(HLine::new("0 dB", 0.0).color(Color32::DARK_GRAY));
        // Crossover marker
        plot_ui.vline(VLine::new("f_x", f_x_actual.log10()).color(Color32::YELLOW));
        // f_sw marker
        plot_ui.vline(VLine::new("f_sw", data.f_sw.log10()).color(Color32::DARK_RED));
        if !inner {
            // 2×f_line marker for outer loop
            plot_ui.vline(VLine::new("2×f_line", (2.0 * data.f_line).log10())
                .color(Color32::from_rgb(255, 150, 255)));
        }
    });

    // ── Phase plot ────────────────────────────────────────────
    let phase_plot = egui_plot::Plot::new(if inner { "pfc_phase_inner" } else { "pfc_phase_outer" })
        .height(h_phase)
        .x_axis_label("log₁₀(f)")
        .y_axis_label("degrees")
        .show_axes([true, true])
        .link_axis(if inner { "pfc_inner_freq" } else { "pfc_outer_freq" }, Vec2b::new(true, false));

    phase_plot.show(ui, |plot_ui| {
        plot_ui.line(Line::new("Loop phase", PlotPoints::new(to_log_points(&data.freq, loop_phase)))
            .color(LOOP_COLOR).width(2.0));
        plot_ui.hline(HLine::new("-180°", -180.0).color(Color32::DARK_GRAY));
        plot_ui.vline(VLine::new("f_x", f_x_actual.log10()).color(Color32::YELLOW));
    });

    // ── Output impedance (only on outer loop tab) ─────────────
    if !inner {
        let z_plot = egui_plot::Plot::new("pfc_zout")
            .height(h_zout)
            .x_axis_label("log₁₀(f)")
            .y_axis_label("dBΩ")
            .show_axes([true, true])
            .link_axis("pfc_outer_freq", Vec2b::new(true, false));

        z_plot.show(ui, |plot_ui| {
            plot_ui.line(Line::new("Z_out open", PlotPoints::new(to_log_points(&data.freq, &data.z_out_open_db)))
                .color(ZOPEN_COLOR));
            plot_ui.line(Line::new("Z_out closed", PlotPoints::new(to_log_points(&data.freq, &data.z_out_closed_db)))
                .color(ZCLOSED_COLOR).width(2.0));
        });
    }
}
