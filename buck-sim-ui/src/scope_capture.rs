//! Scope tab — loads + renders waveforms produced by the
//! `scope-capture` binary in `electronics-sim`.
//!
//! `scope-capture` writes two artifact shapes:
//!
//!  - **CSV** with the columns
//!    `t_ns,v_gs_hs,v_gs_ls,v_sw,i_l,i_g_hs,i_d_hs,i_d_ls,i_diode_ls,diode_state`.
//!  - **JSON** with `{"summary": {...}, "samples": [{...}, ...]}`,
//!    where each sample mirrors the CSV columns but with `t_s` (in
//!    seconds — the on-wire `EdgeSample` repr) instead of `t_ns`, and
//!    `summary` carries the headline scalars
//!    (`t_v_th_crossed_ns`, `t_rr_entered_ns`, `t_rr_done_ns`,
//!    `i_rr_peak_a`, `v_sw_overshoot_v`, `f_ring_est_hz`,
//!    optionally `dv_sw_dt_peak_v_per_s`, `di_d_dt_peak_a_per_s`,
//!    `v_gs_ls_peak_v`, `ls_parasitic_turn_on`,
//!    `i_d_ls_peak_a`, `v_in_ripple_peak_v` for forward-compatibility).
//!
//! This module owns:
//!  - the parsed in-memory [`ScopeData`] (time-series + summary),
//!  - the egui rendering of three linked-x panels
//!    (voltages / currents / diode state) + a summary readout,
//!  - the native-only "Load JSON / Load CSV" file-picker controls,
//!  - the optional "Run scope-capture…" sub-process invocation.

use egui::Color32;
use egui_plot::{Line, Plot, PlotPoints, Polygon, VLine};
use serde::Deserialize;

/// Discrete LS-FET body-diode state — mirrors
/// `electronics_sim::edge_transient::DiodeState`.
///
/// We deliberately do **not** depend on `electronics_sim` here so the
/// scope tab can also load CSV files produced by any other compatible
/// run (the on-wire `Debug`-formatted variant names are stable).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub enum DiodeState {
    Forward,
    ReverseRecovery,
    Off,
}

impl DiodeState {
    /// Parse the column produced by `scope-capture`'s `write_csv`
    /// (which prints the `Debug` representation of the enum).
    fn parse(s: &str) -> Option<Self> {
        match s.trim() {
            "Forward" => Some(DiodeState::Forward),
            "ReverseRecovery" => Some(DiodeState::ReverseRecovery),
            "Off" => Some(DiodeState::Off),
            _ => None,
        }
    }

    /// Display colour used for the diode-state band on the bottom
    /// panel.
    fn color(self) -> Color32 {
        match self {
            DiodeState::Forward => Color32::from_rgb(80, 200, 120),
            DiodeState::ReverseRecovery => Color32::from_rgb(255, 120, 80),
            DiodeState::Off => Color32::from_rgb(140, 140, 140),
        }
    }

    /// Numeric "band height" for the bottom panel — Forward = 1,
    /// ReverseRecovery = 2, Off = 3 — so a non-zero ribbon is always
    /// visible at every sample.
    fn band_y(self) -> f64 {
        match self {
            DiodeState::Forward => 1.0,
            DiodeState::ReverseRecovery => 2.0,
            DiodeState::Off => 3.0,
        }
    }

    /// Human-readable label used in the legend / tooltip.
    pub fn label(self) -> &'static str {
        match self {
            DiodeState::Forward => "Forward",
            DiodeState::ReverseRecovery => "ReverseRecovery",
            DiodeState::Off => "Off",
        }
    }
}

/// One time-step in the loaded scope capture, in display units
/// (time in nanoseconds — both CSV and JSON are normalised at load
/// time).
#[derive(Debug, Clone, Copy)]
pub struct ScopeSample {
    pub t_ns: f64,
    pub v_gs_hs: f64,
    pub v_gs_ls: f64,
    pub v_sw: f64,
    pub i_l: f64,
    pub i_g_hs: f64,
    pub i_d_hs: f64,
    pub i_d_ls: f64,
    pub i_diode_ls: f64,
    pub diode_state: DiodeState,
}

/// Headline scalars surfaced under the plots. Mirrors the
/// `summary` block written by `scope-capture --json`. Optional
/// fields stay `None` when missing from the on-wire JSON so older
/// captures still load cleanly.
#[derive(Debug, Clone, Copy, Default)]
pub struct ScopeSummary {
    pub n_samples: usize,
    pub t_v_th_crossed_ns: Option<f64>,
    pub t_rr_entered_ns: Option<f64>,
    pub t_rr_done_ns: Option<f64>,
    pub i_rr_peak_a: f64,
    pub v_sw_overshoot_v: f64,
    pub f_ring_est_hz: f64,
    /// Peak |dV_SW/dt| [V/s]. Optional: only present on captures
    /// emitted by newer scope-capture builds; falls back to a
    /// finite-difference estimate over `samples` when missing.
    pub dv_sw_dt_peak_v_per_s: Option<f64>,
    /// Peak |dI_D/dt| [A/s]. Same forward-compat story.
    pub di_d_dt_peak_a_per_s: Option<f64>,
    pub v_gs_ls_peak_v: Option<f64>,
    pub ls_parasitic_turn_on: Option<bool>,
    pub i_d_ls_peak_a: Option<f64>,
    pub v_th_ls_v: Option<f64>,
}

/// In-memory scope-capture payload — time-series + headline scalars,
/// in display units (ns / V / A).
#[derive(Debug, Clone, Default)]
pub struct ScopeData {
    pub samples: Vec<ScopeSample>,
    pub summary: ScopeSummary,
    /// Source string for the on-hover tooltip on the loader strip.
    pub source: String,
}

impl ScopeData {
    /// Parse a scope-capture JSON blob (the `--json` output).
    ///
    /// Time-series fields are normalised to display units (ns / V /
    /// A) at load time, regardless of whether the on-wire
    /// representation used `t_s` (seconds — current shape) or
    /// `t_ns` (a future-compatible shape).
    pub fn from_json(text: &str) -> Result<Self, String> {
        let raw: RawJson = serde_json::from_str(text)
            .map_err(|e| format!("parse JSON: {e}"))?;
        let mut samples = Vec::with_capacity(raw.samples.len());
        for s in &raw.samples {
            let t_ns = s.t_ns.unwrap_or_else(|| s.t_s.unwrap_or(0.0) * 1e9);
            let diode_state = s.diode_state.unwrap_or(DiodeState::Forward);
            samples.push(ScopeSample {
                t_ns,
                v_gs_hs: s.v_gs_hs,
                v_gs_ls: s.v_gs_ls,
                v_sw: s.v_sw,
                i_l: s.i_l,
                i_g_hs: s.i_g_hs,
                i_d_hs: s.i_d_hs,
                i_d_ls: s.i_d_ls,
                i_diode_ls: s.i_diode_ls,
                diode_state,
            });
        }
        let mut summary = ScopeSummary {
            n_samples: raw.summary.n_samples.unwrap_or(samples.len()),
            t_v_th_crossed_ns: raw.summary.t_v_th_crossed_ns,
            t_rr_entered_ns: raw.summary.t_rr_entered_ns,
            t_rr_done_ns: raw.summary.t_rr_done_ns,
            i_rr_peak_a: raw.summary.i_rr_peak_a.unwrap_or(0.0),
            v_sw_overshoot_v: raw.summary.v_sw_overshoot_v.unwrap_or(0.0),
            f_ring_est_hz: raw.summary.f_ring_est_hz.unwrap_or(0.0),
            dv_sw_dt_peak_v_per_s: raw.summary.dv_sw_dt_peak_v_per_s,
            di_d_dt_peak_a_per_s: raw.summary.di_d_dt_peak_a_per_s,
            v_gs_ls_peak_v: raw.summary.v_gs_ls_peak_v,
            ls_parasitic_turn_on: raw.summary.ls_parasitic_turn_on,
            i_d_ls_peak_a: raw.summary.i_d_ls_peak_a,
            v_th_ls_v: raw.summary.v_th_ls_v,
        };
        if summary.dv_sw_dt_peak_v_per_s.is_none() {
            summary.dv_sw_dt_peak_v_per_s = Some(estimate_peak_slope(
                &samples,
                |s| s.t_ns * 1e-9,
                |s| s.v_sw,
            ));
        }
        if summary.di_d_dt_peak_a_per_s.is_none() {
            summary.di_d_dt_peak_a_per_s = Some(estimate_peak_slope(
                &samples,
                |s| s.t_ns * 1e-9,
                |s| s.i_d_hs,
            ));
        }
        if summary.v_gs_ls_peak_v.is_none() {
            summary.v_gs_ls_peak_v = samples
                .iter()
                .map(|s| s.v_gs_ls)
                .fold(None, |acc, v| Some(acc.map_or(v, |a: f64| a.max(v))));
        }
        Ok(ScopeData {
            samples,
            summary,
            source: String::new(),
        })
    }

    /// Parse a scope-capture CSV blob (the `--csv` output). Only
    /// available natively; called by the file picker. The summary is
    /// rebuilt from the samples on a best-effort basis (CSV carries
    /// no summary block).
    pub fn from_csv(text: &str) -> Result<Self, String> {
        let mut lines = text.lines();
        let header = lines
            .next()
            .ok_or("CSV is empty")?
            .trim();
        if !header.starts_with("t_ns,v_gs_hs") {
            return Err(format!(
                "unexpected CSV header (got {:?}); expected the scope-capture column layout",
                header,
            ));
        }
        let mut samples = Vec::new();
        for (line_idx, line) in lines.enumerate() {
            if line.trim().is_empty() {
                continue;
            }
            let cols: Vec<&str> = line.split(',').collect();
            if cols.len() < 10 {
                return Err(format!(
                    "CSV line {} has {} columns; expected 10",
                    line_idx + 2,
                    cols.len()
                ));
            }
            let parse_f = |c: &str, name: &str| -> Result<f64, String> {
                c.trim().parse::<f64>().map_err(|e| {
                    format!("CSV line {}: bad {} {:?}: {}", line_idx + 2, name, c, e)
                })
            };
            samples.push(ScopeSample {
                t_ns: parse_f(cols[0], "t_ns")?,
                v_gs_hs: parse_f(cols[1], "v_gs_hs")?,
                v_gs_ls: parse_f(cols[2], "v_gs_ls")?,
                v_sw: parse_f(cols[3], "v_sw")?,
                i_l: parse_f(cols[4], "i_l")?,
                i_g_hs: parse_f(cols[5], "i_g_hs")?,
                i_d_hs: parse_f(cols[6], "i_d_hs")?,
                i_d_ls: parse_f(cols[7], "i_d_ls")?,
                i_diode_ls: parse_f(cols[8], "i_diode_ls")?,
                diode_state: DiodeState::parse(cols[9]).ok_or_else(|| {
                    format!("CSV line {}: bad diode_state {:?}", line_idx + 2, cols[9])
                })?,
            });
        }
        let summary = summary_from_samples(&samples);
        Ok(ScopeData {
            samples,
            summary,
            source: String::new(),
        })
    }
}

/// On-wire JSON layout — mirrors the producer's `write_json` output.
#[derive(Deserialize)]
struct RawJson {
    #[serde(default)]
    summary: RawSummary,
    #[serde(default)]
    samples: Vec<RawSample>,
}

#[derive(Default, Deserialize)]
struct RawSummary {
    n_samples: Option<usize>,
    t_v_th_crossed_ns: Option<f64>,
    t_rr_entered_ns: Option<f64>,
    t_rr_done_ns: Option<f64>,
    i_rr_peak_a: Option<f64>,
    v_sw_overshoot_v: Option<f64>,
    f_ring_est_hz: Option<f64>,
    dv_sw_dt_peak_v_per_s: Option<f64>,
    di_d_dt_peak_a_per_s: Option<f64>,
    v_gs_ls_peak_v: Option<f64>,
    ls_parasitic_turn_on: Option<bool>,
    i_d_ls_peak_a: Option<f64>,
    v_th_ls_v: Option<f64>,
}

#[derive(Deserialize)]
struct RawSample {
    /// On the producer side `EdgeSample` serializes its time as
    /// `t_s` (seconds). We tolerate either field name so future
    /// captures can drop the conversion.
    #[serde(default)]
    t_s: Option<f64>,
    #[serde(default)]
    t_ns: Option<f64>,
    #[serde(default)]
    v_gs_hs: f64,
    #[serde(default)]
    v_gs_ls: f64,
    #[serde(default)]
    v_sw: f64,
    #[serde(default)]
    i_l: f64,
    #[serde(default)]
    i_g_hs: f64,
    #[serde(default)]
    i_d_hs: f64,
    #[serde(default)]
    i_d_ls: f64,
    #[serde(default)]
    i_diode_ls: f64,
    #[serde(default)]
    diode_state: Option<DiodeState>,
}

/// Finite-difference peak slope estimator. `t(s)` returns seconds,
/// `y(s)` returns the y-channel of interest. Used for backfilling
/// `dv_sw_dt_peak` / `di_d_dt_peak` on CSV / older-JSON loads.
fn estimate_peak_slope<F, G>(samples: &[ScopeSample], t: F, y: G) -> f64
where
    F: Fn(&ScopeSample) -> f64,
    G: Fn(&ScopeSample) -> f64,
{
    let mut peak = 0.0_f64;
    for w in samples.windows(2) {
        let dt = t(&w[1]) - t(&w[0]);
        if dt > 0.0 {
            let dy_dt = ((y(&w[1]) - y(&w[0])) / dt).abs();
            if dy_dt > peak {
                peak = dy_dt;
            }
        }
    }
    peak
}

/// Rebuild a [`ScopeSummary`] from raw samples — used when a CSV
/// capture is loaded and no JSON summary block is available.
fn summary_from_samples(samples: &[ScopeSample]) -> ScopeSummary {
    let mut i_rr_peak_a = 0.0_f64;
    let mut v_sw_overshoot_v = 0.0_f64;
    let mut v_gs_ls_peak_v = 0.0_f64;
    let mut t_v_th_crossed_ns: Option<f64> = None;
    let mut t_rr_entered_ns: Option<f64> = None;
    let mut t_rr_done_ns: Option<f64> = None;
    let mut prev_state: Option<DiodeState> = None;
    // V_GS_HS rising-edge crossing of ~half of V_drive — a coarse
    // V_th proxy. CSV has no driver model, so this approximates the
    // JSON summary marker.
    let v_max = samples
        .iter()
        .map(|s| s.v_gs_hs)
        .fold(0.0_f64, f64::max);
    let v_th_proxy = v_max * 0.5;
    for s in samples {
        if s.i_diode_ls.abs() > i_rr_peak_a {
            i_rr_peak_a = s.i_diode_ls.abs();
        }
        if s.v_sw > v_sw_overshoot_v {
            v_sw_overshoot_v = s.v_sw;
        }
        if s.v_gs_ls > v_gs_ls_peak_v {
            v_gs_ls_peak_v = s.v_gs_ls;
        }
        if t_v_th_crossed_ns.is_none() && s.v_gs_hs >= v_th_proxy && v_th_proxy > 0.0 {
            t_v_th_crossed_ns = Some(s.t_ns);
        }
        match (prev_state, s.diode_state) {
            (Some(DiodeState::Forward), DiodeState::ReverseRecovery) => {
                if t_rr_entered_ns.is_none() {
                    t_rr_entered_ns = Some(s.t_ns);
                }
            }
            (Some(DiodeState::ReverseRecovery), DiodeState::Off) => {
                if t_rr_done_ns.is_none() {
                    t_rr_done_ns = Some(s.t_ns);
                }
            }
            _ => {}
        }
        prev_state = Some(s.diode_state);
    }
    let dv_sw_dt = estimate_peak_slope(samples, |s| s.t_ns * 1e-9, |s| s.v_sw);
    let di_d_dt = estimate_peak_slope(samples, |s| s.t_ns * 1e-9, |s| s.i_d_hs);
    ScopeSummary {
        n_samples: samples.len(),
        t_v_th_crossed_ns,
        t_rr_entered_ns,
        t_rr_done_ns,
        i_rr_peak_a,
        // V_SW overshoot vs V_in is unknowable from CSV alone; the
        // best we can do is report the raw peak, which the readout
        // panel labels as such.
        v_sw_overshoot_v,
        f_ring_est_hz: 0.0,
        dv_sw_dt_peak_v_per_s: Some(dv_sw_dt),
        di_d_dt_peak_a_per_s: Some(di_d_dt),
        v_gs_ls_peak_v: Some(v_gs_ls_peak_v),
        ls_parasitic_turn_on: None,
        i_d_ls_peak_a: None,
        v_th_ls_v: None,
    }
}

/// UI state owned by the Scope tab. Lives on `BuckSimApp`.
#[derive(Debug, Default)]
pub struct ScopeUi {
    /// Currently-loaded capture, or `None` if nothing has been
    /// loaded yet.
    pub data: Option<ScopeData>,
    /// File path the user typed for the "Load JSON" button.
    pub json_path: String,
    /// File path the user typed for the "Load CSV" button.
    pub csv_path: String,
    /// Last load attempt — success message or error string.
    pub load_status: Option<Result<String, String>>,

    // ── "Run scope-capture…" sub-process invocation ──────────────
    pub parasitics_path: String,
    pub v_in: f64,
    pub v_out: f64,
    pub i_l: f64,
    /// "turn-on" or "turn-off".
    pub edge: String,
    pub output_json_path: String,
    pub run_status: Option<Result<String, String>>,
}

impl ScopeUi {
    /// Defaults matched against `scope-capture --help` example values
    /// so the user can hit "Run" without filling in every field.
    pub fn new() -> Self {
        Self {
            data: None,
            json_path: String::new(),
            csv_path: String::new(),
            load_status: None,
            parasitics_path: String::new(),
            v_in: 12.0,
            v_out: 3.3,
            i_l: 3.0,
            edge: "turn-on".to_string(),
            output_json_path: "/tmp/scope_capture.json".to_string(),
            run_status: None,
        }
    }
}

/// Render the Scope tab into `ui`. The egui state is owned by
/// `ScopeUi`; the function is free-standing (rather than a method
/// on `BuckSimApp`) so the tab can be unit-tested independently.
pub fn show_scope_capture(ui: &mut egui::Ui, state: &mut ScopeUi) {
    show_loader_strip(ui, state);
    show_run_strip(ui, state);

    if let Some(status) = state.load_status.as_ref() {
        match status {
            Ok(msg) => {
                ui.colored_label(Color32::from_rgb(120, 200, 120), msg);
            }
            Err(msg) => {
                ui.colored_label(Color32::from_rgb(255, 120, 120), msg);
            }
        }
    }
    if let Some(status) = state.run_status.as_ref() {
        match status {
            Ok(msg) => {
                ui.colored_label(Color32::from_rgb(120, 200, 120), msg);
            }
            Err(msg) => {
                ui.colored_label(Color32::from_rgb(255, 120, 120), msg);
            }
        }
    }
    ui.separator();

    let Some(data) = state.data.as_ref() else {
        ui.centered_and_justified(|ui| {
            ui.colored_label(
                Color32::from_rgb(200, 200, 200),
                "Load a scope-capture JSON or CSV (see file pickers above) \
                 to render the captured edge.",
            );
        });
        return;
    };
    if data.samples.is_empty() {
        ui.centered_and_justified(|ui| {
            ui.colored_label(
                Color32::from_rgb(255, 100, 100),
                "Loaded capture has zero samples.",
            );
        });
        return;
    }

    render_plots(ui, data);
    ui.separator();
    render_summary(ui, data);
}

/// Top loader strip: status badge + JSON/CSV path inputs + buttons.
/// Mirrors the existing `show_loop_loader` pattern in `app.rs`.
fn show_loader_strip(ui: &mut egui::Ui, state: &mut ScopeUi) {
    ui.horizontal_wrapped(|ui| match state.data.as_ref() {
        Some(d) => {
            ui.colored_label(
                Color32::from_rgb(120, 200, 255),
                format!(
                    "Scope: loaded ({} samples, {:.2}–{:.2} ns)",
                    d.samples.len(),
                    d.samples.first().map(|s| s.t_ns).unwrap_or(0.0),
                    d.samples.last().map(|s| s.t_ns).unwrap_or(0.0),
                ),
            )
            .on_hover_text(if d.source.is_empty() {
                "(no source path recorded)".to_string()
            } else {
                d.source.clone()
            });
            if ui.button("Clear").clicked() {
                state.data = None;
                state.load_status = None;
            }
        }
        None => {
            ui.label("Scope: (no capture loaded)");
        }
    });

    ui.horizontal(|ui| {
        ui.label("JSON:");
        ui.add(
            egui::TextEdit::singleline(&mut state.json_path)
                .desired_width(320.0)
                .hint_text("/path/to/scope_capture.json"),
        );
        let load_clicked = ui.button("Load JSON").clicked();
        #[cfg(target_arch = "wasm32")]
        {
            let _ = load_clicked;
            ui.colored_label(
                Color32::GRAY,
                "(loading from disk is unavailable on the web build)",
            );
        }
        #[cfg(not(target_arch = "wasm32"))]
        if load_clicked {
            state.load_status = Some(try_load_json(state));
        }
    });

    ui.horizontal(|ui| {
        ui.label("CSV: ");
        ui.add(
            egui::TextEdit::singleline(&mut state.csv_path)
                .desired_width(320.0)
                .hint_text("/path/to/scope_capture.csv"),
        );
        let load_clicked = ui.button("Load CSV").clicked();
        #[cfg(target_arch = "wasm32")]
        {
            let _ = load_clicked;
        }
        #[cfg(not(target_arch = "wasm32"))]
        if load_clicked {
            state.load_status = Some(try_load_csv(state));
        }
    });
}

/// Optional "Run scope-capture…" sub-process invocation. Hidden
/// behind a collapsing header so it does not crowd the main loader
/// strip when the user just wants to drop a JSON in.
fn show_run_strip(ui: &mut egui::Ui, state: &mut ScopeUi) {
    egui::CollapsingHeader::new("Run scope-capture…")
        .default_open(false)
        .show(ui, |ui| {
            #[cfg(target_arch = "wasm32")]
            {
                let _ = state;
                ui.colored_label(
                    Color32::GRAY,
                    "(running the scope-capture binary is unavailable on the web build)",
                );
            }
            #[cfg(not(target_arch = "wasm32"))]
            {
                ui.horizontal(|ui| {
                    ui.label("BuckParasiticSet JSON:");
                    ui.add(
                        egui::TextEdit::singleline(&mut state.parasitics_path)
                            .desired_width(320.0)
                            .hint_text("/tmp/buck_parasitics.json"),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("V_in [V]:");
                    ui.add(egui::DragValue::new(&mut state.v_in).range(1.0..=400.0).speed(0.1));
                    ui.label("V_out [V]:");
                    ui.add(egui::DragValue::new(&mut state.v_out).range(0.3..=200.0).speed(0.1));
                    ui.label("I_L [A]:");
                    ui.add(egui::DragValue::new(&mut state.i_l).range(0.0..=200.0).speed(0.1));
                });
                ui.horizontal(|ui| {
                    ui.label("Edge:");
                    ui.selectable_value(&mut state.edge, "turn-on".to_string(), "turn-on");
                    ui.selectable_value(&mut state.edge, "turn-off".to_string(), "turn-off");
                    ui.separator();
                    ui.label("→ JSON:");
                    ui.add(
                        egui::TextEdit::singleline(&mut state.output_json_path)
                            .desired_width(280.0)
                            .hint_text("/tmp/scope_capture.json"),
                    );
                });
                if ui.button("Run scope-capture & load").clicked() {
                    state.run_status = Some(try_run_and_load(state));
                }
            }
        });
}

/// Render the three vertically stacked, linked-x panels.
fn render_plots(ui: &mut egui::Ui, data: &ScopeData) {
    let x_link = egui::Vec2b::new(true, false);
    // Total height divided 3-way; leave room for the readout panel.
    let total_h = (ui.available_height() - 200.0).max(280.0);
    let panel_h = total_h / 3.0;

    // ── Voltages ────────────────────────────────────────────────
    let v_gs_hs: PlotPoints = data
        .samples
        .iter()
        .map(|s| [s.t_ns, s.v_gs_hs])
        .collect();
    let v_gs_ls: PlotPoints = data
        .samples
        .iter()
        .map(|s| [s.t_ns, s.v_gs_ls])
        .collect();
    let v_sw: PlotPoints = data.samples.iter().map(|s| [s.t_ns, s.v_sw]).collect();

    Plot::new("scope_capture_v")
        .height(panel_h)
        .y_axis_label("V [V]")
        .x_axis_label("")
        .legend(egui_plot::Legend::default())
        .link_axis("scope_capture_time", x_link)
        .link_cursor("scope_capture_time", x_link)
        .show(ui, |plot_ui| {
            plot_ui.line(
                Line::new("V_GS_HS", v_gs_hs).color(Color32::from_rgb(80, 200, 255)),
            );
            plot_ui.line(
                Line::new("V_GS_LS", v_gs_ls).color(Color32::from_rgb(255, 200, 80)),
            );
            plot_ui.line(
                Line::new("V_SW", v_sw).color(Color32::from_rgb(255, 120, 200)),
            );
            draw_event_markers(plot_ui, &data.summary);
        });

    // ── Currents ────────────────────────────────────────────────
    let i_l: PlotPoints = data.samples.iter().map(|s| [s.t_ns, s.i_l]).collect();
    let i_d_hs: PlotPoints = data.samples.iter().map(|s| [s.t_ns, s.i_d_hs]).collect();
    let i_d_ls: PlotPoints = data.samples.iter().map(|s| [s.t_ns, s.i_d_ls]).collect();
    let i_diode_ls: PlotPoints = data
        .samples
        .iter()
        .map(|s| [s.t_ns, s.i_diode_ls])
        .collect();

    Plot::new("scope_capture_i")
        .height(panel_h)
        .y_axis_label("I [A]")
        .x_axis_label("")
        .legend(egui_plot::Legend::default())
        .link_axis("scope_capture_time", x_link)
        .link_cursor("scope_capture_time", x_link)
        .show(ui, |plot_ui| {
            plot_ui.line(Line::new("I_L", i_l).color(Color32::from_rgb(80, 200, 120)));
            plot_ui.line(
                Line::new("I_D_HS", i_d_hs).color(Color32::from_rgb(80, 140, 255)),
            );
            plot_ui.line(
                Line::new("I_D_LS", i_d_ls).color(Color32::from_rgb(255, 160, 50)),
            );
            plot_ui.line(
                Line::new("I_diode_LS", i_diode_ls).color(Color32::from_rgb(255, 80, 80)),
            );
            draw_event_markers(plot_ui, &data.summary);
        });

    // ── Diode state ─────────────────────────────────────────────
    //
    // Render as a step-line on a 1/2/3 axis (Forward / RR / Off) so
    // the trace itself is the legend-toggleable representation, and
    // overlay translucent polygons per state for the coloured-band
    // effect.
    Plot::new("scope_capture_diode")
        .height(panel_h.min(120.0))
        .y_axis_label("diode")
        .x_axis_label("t [ns]")
        .legend(egui_plot::Legend::default())
        .link_axis("scope_capture_time", x_link)
        .link_cursor("scope_capture_time", x_link)
        .show(ui, |plot_ui| {
            for (state, label) in [
                (DiodeState::Forward, "Forward"),
                (DiodeState::ReverseRecovery, "ReverseRecovery"),
                (DiodeState::Off, "Off"),
            ] {
                let bands = collect_state_bands(&data.samples, state);
                for (i, (t0, t1)) in bands.iter().enumerate() {
                    let mut poly_color = state.color();
                    poly_color = Color32::from_rgba_unmultiplied(
                        poly_color.r(),
                        poly_color.g(),
                        poly_color.b(),
                        80,
                    );
                    // Only the first polygon per state carries the
                    // legend name so the legend stays tidy.
                    let name = if i == 0 { label } else { "" };
                    plot_ui.polygon(
                        Polygon::new(
                            name,
                            PlotPoints::new(vec![
                                [*t0, 0.0],
                                [*t1, 0.0],
                                [*t1, 4.0],
                                [*t0, 4.0],
                            ]),
                        )
                        .fill_color(poly_color),
                    );
                }
            }
            // Step-line for the discrete diode-state value, so the
            // user can read the exact transition timing.
            let step: PlotPoints = data
                .samples
                .iter()
                .map(|s| [s.t_ns, s.diode_state.band_y()])
                .collect();
            plot_ui.line(
                Line::new("state(t)", step).color(Color32::from_rgb(220, 220, 220)),
            );
            draw_event_markers(plot_ui, &data.summary);
        });
}

/// Collect contiguous `t_ns` ranges where `samples` are in
/// `target_state`. Returns `(t_start_ns, t_end_ns)` pairs suitable
/// for filling as polygons on the diode-state panel.
fn collect_state_bands(samples: &[ScopeSample], target_state: DiodeState) -> Vec<(f64, f64)> {
    let mut bands = Vec::new();
    let mut run_start: Option<f64> = None;
    let mut last_t = 0.0;
    for s in samples {
        if s.diode_state == target_state {
            if run_start.is_none() {
                run_start = Some(s.t_ns);
            }
            last_t = s.t_ns;
        } else if let Some(t0) = run_start.take() {
            bands.push((t0, last_t));
        }
    }
    if let Some(t0) = run_start {
        bands.push((t0, last_t));
    }
    bands
}

/// Draw the three event-marker vertical lines (V_th crossing, RR
/// start, RR end) on whichever plot is currently open.
fn draw_event_markers(plot_ui: &mut egui_plot::PlotUi, s: &ScopeSummary) {
    let style = egui_plot::LineStyle::dashed_dense();
    if let Some(t) = s.t_v_th_crossed_ns {
        plot_ui.vline(
            VLine::new("V_th", t)
                .color(Color32::from_rgb(220, 220, 120))
                .style(style),
        );
    }
    if let Some(t) = s.t_rr_entered_ns {
        plot_ui.vline(
            VLine::new("RR start", t)
                .color(Color32::from_rgb(255, 140, 80))
                .style(style),
        );
    }
    if let Some(t) = s.t_rr_done_ns {
        plot_ui.vline(
            VLine::new("RR end", t)
                .color(Color32::from_rgb(200, 80, 200))
                .style(style),
        );
    }
}

/// Bottom text strip — peak |I_RR|, V_SW overshoot, ring frequency,
/// peak |dV_SW/dt|, peak |dI_D/dt|, LS-FET parasitic turn-on flag.
fn render_summary(ui: &mut egui::Ui, data: &ScopeData) {
    let s = &data.summary;
    let dv_sw_dt_v_per_ns = s.dv_sw_dt_peak_v_per_s.unwrap_or(0.0) / 1e9;
    let di_d_dt_a_per_ns = s.di_d_dt_peak_a_per_s.unwrap_or(0.0) / 1e9;
    let f_ring_mhz = s.f_ring_est_hz / 1e6;
    ui.horizontal_wrapped(|ui| {
        ui.label(format!("Peak |I_RR| = {:.3} A", s.i_rr_peak_a));
        ui.separator();
        ui.label(format!("V_SW overshoot = {:.3} V", s.v_sw_overshoot_v));
        ui.separator();
        ui.label(format!("f_ring (parasitic LC) = {:.3} MHz", f_ring_mhz));
    });
    ui.horizontal_wrapped(|ui| {
        ui.label(format!("Peak |dV_SW/dt| = {:.3} V/ns", dv_sw_dt_v_per_ns));
        ui.separator();
        ui.label(format!("Peak |dI_D/dt| = {:.3} A/ns", di_d_dt_a_per_ns));
    });
    if let Some(v_gs_ls_peak) = s.v_gs_ls_peak_v {
        let v_th = s.v_th_ls_v;
        let parasitic = s
            .ls_parasitic_turn_on
            .unwrap_or_else(|| v_th.map_or(false, |th| v_gs_ls_peak > th));
        let color = if parasitic {
            Color32::from_rgb(255, 120, 80)
        } else {
            Color32::from_rgb(150, 220, 150)
        };
        let flag = if parasitic {
            "LS-FET parasitic turn-on detected"
        } else {
            "LS-FET parasitic turn-on: none"
        };
        let v_th_str = v_th
            .map(|v| format!("{:.2} V", v))
            .unwrap_or_else(|| "—".to_string());
        ui.colored_label(
            color,
            format!(
                "{flag}: peak V_GS_LS = {:.3} V (V_th_LS = {})",
                v_gs_ls_peak, v_th_str,
            ),
        );
    }
}

/// Native-only: read the JSON file at `state.json_path`, parse it as
/// a `ScopeData`, and store it on the UI state. Mirrors
/// [`crate::app::BuckSimApp::try_load_loop`].
#[cfg(not(target_arch = "wasm32"))]
pub fn try_load_json(state: &mut ScopeUi) -> Result<String, String> {
    let path = state.json_path.trim().to_string();
    if path.is_empty() {
        return Err("Enter a JSON path first.".into());
    }
    let text = std::fs::read_to_string(&path).map_err(|e| format!("read {path}: {e}"))?;
    let mut data = ScopeData::from_json(&text).map_err(|e| format!("parse {path}: {e}"))?;
    data.source = format!("JSON: {path}");
    let msg = format!(
        "Loaded {} samples ({:.2} ns window).",
        data.samples.len(),
        data.samples.last().map(|s| s.t_ns).unwrap_or(0.0)
            - data.samples.first().map(|s| s.t_ns).unwrap_or(0.0),
    );
    state.data = Some(data);
    Ok(msg)
}

/// Native-only: read the CSV file at `state.csv_path`, parse it as a
/// `ScopeData`, and store it on the UI state.
#[cfg(not(target_arch = "wasm32"))]
pub fn try_load_csv(state: &mut ScopeUi) -> Result<String, String> {
    let path = state.csv_path.trim().to_string();
    if path.is_empty() {
        return Err("Enter a CSV path first.".into());
    }
    let text = std::fs::read_to_string(&path).map_err(|e| format!("read {path}: {e}"))?;
    let mut data = ScopeData::from_csv(&text).map_err(|e| format!("parse {path}: {e}"))?;
    data.source = format!("CSV: {path}");
    let msg = format!("Loaded {} samples.", data.samples.len());
    state.data = Some(data);
    Ok(msg)
}

/// Native-only: shell out to the `scope-capture` binary using the
/// fields in `state`, then auto-load the produced JSON.
///
/// Binary resolution order:
///   1. `SCOPE_CAPTURE_BIN` env var (if set).
///   2. `$HOME/my_projects/full-control/electronics-sim/target/release/scope-capture`.
#[cfg(not(target_arch = "wasm32"))]
pub fn try_run_and_load(state: &mut ScopeUi) -> Result<String, String> {
    let bin = resolve_scope_capture_bin()?;
    let parasitics = state.parasitics_path.trim();
    if parasitics.is_empty() {
        return Err("Enter a BuckParasiticSet JSON path first.".into());
    }
    let out_json = state.output_json_path.trim();
    if out_json.is_empty() {
        return Err("Enter an output JSON path first.".into());
    }
    let mut cmd = std::process::Command::new(&bin);
    cmd.arg("--parasitics")
        .arg(parasitics)
        .arg("--v-in")
        .arg(state.v_in.to_string())
        .arg("--v-out")
        .arg(state.v_out.to_string())
        .arg("--i-l")
        .arg(state.i_l.to_string())
        .arg("--edge")
        .arg(&state.edge)
        .arg("--json")
        .arg(out_json);
    let output = cmd
        .output()
        .map_err(|e| format!("spawn {bin:?}: {e}"))?;
    if !output.status.success() {
        return Err(format!(
            "scope-capture exited with {}: {}",
            output.status,
            String::from_utf8_lossy(&output.stderr),
        ));
    }
    state.json_path = out_json.to_string();
    try_load_json(state)
}

#[cfg(not(target_arch = "wasm32"))]
fn resolve_scope_capture_bin() -> Result<String, String> {
    if let Ok(p) = std::env::var("SCOPE_CAPTURE_BIN") {
        if !p.trim().is_empty() {
            return Ok(p);
        }
    }
    let home = std::env::var("HOME").map_err(|_| "$HOME is not set".to_string())?;
    Ok(format!(
        "{home}/my_projects/full-control/electronics-sim/target/release/scope-capture"
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_json_minimal_roundtrip() {
        // Minimal inline fixture matching the on-wire shape written
        // by `electronics-sim/src/bin/scope_capture.rs` — three
        // samples spanning the three diode states, plus a summary
        // block.
        let json = r#"
        {
            "summary": {
                "n_samples": 3,
                "t_v_th_crossed_ns": 1.5,
                "t_rr_entered_ns": 4.0,
                "t_rr_done_ns": 8.0,
                "i_rr_peak_a": 2.5,
                "v_sw_overshoot_v": 3.1,
                "f_ring_est_hz": 1.5e8
            },
            "samples": [
                {
                    "t_s": 0.0e-9,
                    "v_gs_hs": 0.0, "v_gs_ls": 0.1, "v_sw": 0.0,
                    "i_l": 3.0, "i_g_hs": 0.0,
                    "i_d_hs": 0.0, "i_d_ls": 0.0, "i_diode_ls": 0.0,
                    "diode_state": "Forward"
                },
                {
                    "t_s": 5.0e-9,
                    "v_gs_hs": 6.0, "v_gs_ls": 0.2, "v_sw": 6.0,
                    "i_l": 3.1, "i_g_hs": 0.5,
                    "i_d_hs": 4.0, "i_d_ls": 0.0, "i_diode_ls": -2.5,
                    "diode_state": "ReverseRecovery"
                },
                {
                    "t_s": 10.0e-9,
                    "v_gs_hs": 10.0, "v_gs_ls": 0.0, "v_sw": 12.5,
                    "i_l": 3.0, "i_g_hs": 0.0,
                    "i_d_hs": 3.0, "i_d_ls": 0.0, "i_diode_ls": 0.0,
                    "diode_state": "Off"
                }
            ]
        }
        "#;
        let data = ScopeData::from_json(json).expect("parse JSON");
        assert_eq!(data.samples.len(), 3);
        // Time conversion: t_s -> t_ns
        assert!((data.samples[0].t_ns - 0.0).abs() < 1e-9);
        assert!((data.samples[1].t_ns - 5.0).abs() < 1e-9);
        assert!((data.samples[2].t_ns - 10.0).abs() < 1e-9);
        // Diode states picked up correctly.
        assert_eq!(data.samples[0].diode_state, DiodeState::Forward);
        assert_eq!(data.samples[1].diode_state, DiodeState::ReverseRecovery);
        assert_eq!(data.samples[2].diode_state, DiodeState::Off);
        // Summary surfaces.
        assert_eq!(data.summary.n_samples, 3);
        assert_eq!(data.summary.t_v_th_crossed_ns, Some(1.5));
        assert_eq!(data.summary.t_rr_entered_ns, Some(4.0));
        assert_eq!(data.summary.t_rr_done_ns, Some(8.0));
        assert!((data.summary.i_rr_peak_a - 2.5).abs() < 1e-12);
        assert!((data.summary.v_sw_overshoot_v - 3.1).abs() < 1e-12);
        assert!((data.summary.f_ring_est_hz - 1.5e8).abs() < 1.0);
        // dV_SW/dt backfilled from samples: (12.5 - 6.0) / 5 ns = 1.3 V/ns.
        let dv = data.summary.dv_sw_dt_peak_v_per_s.expect("backfilled");
        assert!((dv - 1.3e9).abs() < 1e6, "dV_SW/dt was {} V/s", dv);
    }

    #[test]
    fn parse_csv_basic() {
        let csv = "t_ns,v_gs_hs,v_gs_ls,v_sw,i_l,i_g_hs,i_d_hs,i_d_ls,i_diode_ls,diode_state\n\
                   0.0,0.0,0.1,0.0,3.0,0.0,0.0,0.0,0.0,Forward\n\
                   5.0,6.0,0.2,6.0,3.1,0.5,4.0,0.0,-2.5,ReverseRecovery\n\
                   10.0,10.0,0.0,12.5,3.0,0.0,3.0,0.0,0.0,Off\n";
        let data = ScopeData::from_csv(csv).expect("parse CSV");
        assert_eq!(data.samples.len(), 3);
        assert_eq!(data.samples[1].diode_state, DiodeState::ReverseRecovery);
        // Rebuilt summary: |i_diode_ls| peak = 2.5.
        assert!((data.summary.i_rr_peak_a - 2.5).abs() < 1e-12);
        // RR-entry t was detected via Forward → ReverseRecovery transition.
        assert_eq!(data.summary.t_rr_entered_ns, Some(5.0));
        assert_eq!(data.summary.t_rr_done_ns, Some(10.0));
    }

    #[test]
    fn collect_state_bands_finds_contiguous_runs() {
        let mk = |t: f64, st: DiodeState| ScopeSample {
            t_ns: t,
            v_gs_hs: 0.0,
            v_gs_ls: 0.0,
            v_sw: 0.0,
            i_l: 0.0,
            i_g_hs: 0.0,
            i_d_hs: 0.0,
            i_d_ls: 0.0,
            i_diode_ls: 0.0,
            diode_state: st,
        };
        let samples = vec![
            mk(0.0, DiodeState::Forward),
            mk(1.0, DiodeState::Forward),
            mk(2.0, DiodeState::ReverseRecovery),
            mk(3.0, DiodeState::ReverseRecovery),
            mk(4.0, DiodeState::Off),
            mk(5.0, DiodeState::Forward),
        ];
        let fwd = collect_state_bands(&samples, DiodeState::Forward);
        // Two bands: 0–1 ns and 5–5 ns.
        assert_eq!(fwd.len(), 2);
        assert_eq!(fwd[0], (0.0, 1.0));
        assert_eq!(fwd[1].0, 5.0);
        let rr = collect_state_bands(&samples, DiodeState::ReverseRecovery);
        assert_eq!(rr.len(), 1);
        assert_eq!(rr[0], (2.0, 3.0));
    }
}
