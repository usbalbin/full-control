use egui::Color32;
use egui_plot::{HLine, Line, Plot, PlotPoints};
use full_control::control_2p2z::Topology as ControlTopology;

use crate::bode::{BodeData, NonIdealParams, show_bode};
use crate::pfc_bode::{PfcBodeData, show_pfc_bode};
use crate::pfc_sim::{PfcSimParams, PfcSimPoint, PfcMetrics, PfcControlMode, PfcTopology, run_pfc_simulation, run_pfc_vin_sweep, SOFT_START_HALF_CYCLES};
use crate::sim::{CapTypeUi, CurrentConduction, FetProfile, McuProfile, CsProfile, DacProfile, LossBreakdown, LoadKind, SimParams, SimPoint, build_ctrl_params_multi, compute_losses, computed_r_series_mohm, run_simulation, SOFT_START_CYCLES, STEADY_STATE_CYCLES, LOAD_STEP_CYCLES};

#[derive(Debug, Clone, Copy, PartialEq)]
enum ConverterMode {
    BuckPcmc,
    PfcBoostAcm,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Tab {
    Simulation,
    Bode,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum PfcTab {
    Simulation,
    CurrentLoopBode,
    VoltageLoopBode,
    Harmonics,
    VinSweep,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum IecClass {
    A,
    B,
    C,
    D,
}

pub struct BuckSimApp {
    // ── Slider state (human-friendly units) ────────────────────────────────
    v_in: f64,          // [V]
    v_out_target: f64,  // [V]
    f_sw_khz: f64,      // [kHz]
    l_uh: f64,          // [µH]
    c_out_uf: f64,      // [µF]
    r_esr_mohm: f64,    // [mΩ]
    dcr_mohm: f64,      // [mΩ]
    max_current: f64,   // [A]

    // ── Controller tuning ──────────────────────────────────────────────────
    crossover_khz: f64,    // [kHz]
    cycles_per_tick: usize,

    // ── Rectifier configuration ──────────────────────────────────────────────
    current_conduction: CurrentConduction,

    // ── Load configuration ───────────────────────────────────────────────────
    load_kind: LoadKind,
    r_loads: Vec<f64>,      // [Ω] per-phase load resistances (Steps mode)
    c_load_uf: f64,         // [µF] extra load capacitance (plant-only)
    r_in_mohm: f64,         // [mΩ] input cable resistance (plant-only)
    l_in_nh: f64,           // [nH] input cable inductance (plant-only)
    c_in_uf: f64,           // [µF] input decoupling capacitance (plant-only)
    bat_v_init: f64,        // [V]   battery initial OCV
    bat_r_int_mohm: f64,    // [mΩ]  battery internal resistance
    bat_c_mf: f64,          // [mF]  battery capacitance (controls charging speed)

    // ── Slope compensation ─────────────────────────────────────────────────────
    slope_overcomp: f64,    // over-compensation factor (1.0 = none, 1.5 = firmware default)

    // ── Timing ────────────────────────────────────────────────────────────────
    blanking_ns: f64,       // [ns]
    adc_sample_ns: f64,     // [ns]
    max_duty_pct: f64,      // [%]

    // ── Hardware profiles ─────────────────────────────────────────────────────
    hs_fet: FetProfile,
    hs_fet_preset_idx: usize,
    ls_fet: FetProfile,
    ls_fet_preset_idx: usize,
    mcu: McuProfile,
    mcu_preset_idx: usize,
    cs: CsProfile,
    cs_preset_idx: usize,
    dac: DacProfile,
    dac_preset_idx: usize,

    // ── Output capacitor bank ─────────────────────────────────────────────
    output_caps: Vec<CapTypeUi>,
    use_cap_bank: bool,  // UI toggle: when false, use scalar c_out/r_esr

    // ── Multi-phase ──────────────────────────────────────────────────────────
    num_phases: usize,

    // ── Cached simulation output ────────────────────────────────────────────
    sim_data: Result<Vec<SimPoint>, String>,
    bode_data: Option<BodeData>,
    loss_breakdown: Option<LossBreakdown>,
    last_params: SimParams,

    // ── UI state ─────────────────────────────────────────────────────────────
    converter_mode: ConverterMode,
    tab: Tab,
    plot_option: PlotOption,

    // ── PFC state ────────────────────────────────────────────────────────────
    pfc: PfcState,
}

struct PfcState {
    // Slider params (UI-friendly units)
    v_in_rms: f64,
    f_line_hz: f64,
    v_out: f64,
    f_sw_khz: f64,
    l_uh: f64,
    c_out_uf: f64,
    r_esr_mohm: f64,
    dcr_mohm: f64,
    rds_on_mohm: f64,
    r_sense_mohm: f64,
    v_diode: f64,
    current_crossover_khz: f64,
    voltage_crossover_hz: f64,

    // Load steps
    p_loads_w: Vec<f64>,
    steady_half_cycles: usize,
    step_half_cycles: usize,

    // Multi-phase
    num_phases: usize,

    // Conduction mode
    current_conduction: CurrentConduction,

    // Control mode and topology
    control_mode: PfcControlMode,
    topology: PfcTopology,
    slope_overcomp: f64,

    // AC phases (1 = single-phase, 3 = three-phase)
    ac_phases: usize,

    // Cap bank (empty = scalar mode)
    output_caps: Vec<CapTypeUi>,

    // Cached results
    sim_data: Result<Vec<PfcSimPoint>, String>,
    metrics: Option<PfcMetrics>,
    bode_data: Option<PfcBodeData>,
    vin_sweep: Option<Vec<(f64, PfcMetrics)>>,
    last_params: PfcSimParams,

    // UI
    tab: PfcTab,
    pfc_plot_option: PlotOption,
    iec_class: IecClass,
}

#[derive(Debug, Clone, PartialEq)]
enum PlotOption {
    Min,
    Max,
    Average,
    MinAndMax,
    Waveform,
}

impl PfcState {
    fn new() -> Self {
        let p = PfcSimParams::default();
        let (sim_data, metrics, bode_data) = match run_pfc_simulation(&p) {
            Ok((points, m, ds)) => {
                let r_load = p.v_out * p.v_out / p.p_load_w();
                let bode = PfcBodeData::compute(&ds, r_load);
                (Ok(points), Some(m), Some(bode))
            }
            Err(e) => (Err(e), None, None),
        };
        Self {
            v_in_rms: p.v_in_rms,
            f_line_hz: p.f_line_hz,
            v_out: p.v_out,
            f_sw_khz: p.f_sw_khz,
            l_uh: p.l_uh,
            c_out_uf: p.c_out_uf,
            r_esr_mohm: p.r_esr_mohm,
            dcr_mohm: p.dcr_mohm,
            rds_on_mohm: p.rds_on_mohm,
            r_sense_mohm: p.r_sense_mohm,
            v_diode: p.v_diode,
            current_crossover_khz: p.current_crossover_khz,
            voltage_crossover_hz: p.voltage_crossover_hz,
            p_loads_w: p.p_loads_w.clone(),
            steady_half_cycles: p.steady_half_cycles,
            step_half_cycles: p.step_half_cycles,
            num_phases: p.num_phases,
            current_conduction: p.current_conduction,
            control_mode: p.control_mode,
            topology: p.topology,
            slope_overcomp: p.slope_overcomp,
            ac_phases: p.ac_phases,
            output_caps: p.output_caps.clone(),
            sim_data,
            metrics,
            bode_data,
            vin_sweep: None,
            last_params: p,
            tab: PfcTab::Simulation,
            pfc_plot_option: PlotOption::Average,
            iec_class: IecClass::A,
        }
    }

    fn current_params(&self) -> PfcSimParams {
        PfcSimParams {
            v_in_rms: self.v_in_rms,
            f_line_hz: self.f_line_hz,
            v_out: self.v_out,
            f_sw_khz: self.f_sw_khz,
            l_uh: self.l_uh,
            c_out_uf: self.c_out_uf,
            r_esr_mohm: self.r_esr_mohm,
            dcr_mohm: self.dcr_mohm,
            rds_on_mohm: self.rds_on_mohm,
            r_sense_mohm: self.r_sense_mohm,
            v_diode: self.v_diode,
            current_crossover_khz: self.current_crossover_khz,
            voltage_crossover_hz: self.voltage_crossover_hz,
            p_loads_w: self.p_loads_w.clone(),
            steady_half_cycles: self.steady_half_cycles,
            step_half_cycles: self.step_half_cycles,
            num_phases: self.num_phases,
            current_conduction: self.current_conduction,
            control_mode: self.control_mode,
            topology: self.topology,
            slope_overcomp: self.slope_overcomp,
            ac_phases: self.ac_phases,
            output_caps: self.output_caps.clone(),
        }
    }

    /// Reconstruct per-stage triangular inductor current waveform.
    fn pfc_phase_waveform(&self, data: &[PfcSimPoint], stage_idx: usize) -> Vec<[f64; 2]> {
        let n = self.num_phases.max(1);
        let mut pts = Vec::with_capacity(data.len() * 4);
        for p in data {
            if stage_idx >= p.phases.len() { continue; }
            let ph = &p.phases[stage_idx];
            // Use per-point t_sw_us (varies in CrCM) instead of global f_sw_khz
            let t_sw_ms = p.t_sw_us as f64 / 1000.0;
            let offset_ms = stage_idx as f64 * t_sw_ms / n as f64;
            let t0 = p.t_us as f64 / 1000.0 + offset_ms; // ms
            let t_on_ms = ph.duty as f64 * t_sw_ms;
            let t_off_ms = t_sw_ms - t_on_ms;

            // Start: i_l_start
            pts.push([t0, ph.i_l_start as f64]);
            // End of ON: i_l_peak
            pts.push([t0 + t_on_ms, ph.i_l_peak as f64]);

            if ph.dcm {
                // DCM: ramp to 0, then stay at 0
                let t_conduct_ms = ph.t_conduct_frac as f64 * t_off_ms;
                pts.push([t0 + t_on_ms + t_conduct_ms, 0.0]);
                pts.push([t0 + t_sw_ms, 0.0]);
            } else {
                // CCM: ramp to i_l_end
                pts.push([t0 + t_sw_ms, ph.i_l_end as f64]);
            }
        }
        pts
    }
}

impl Default for BuckSimApp {
    fn default() -> Self {
        if let Some(saved) = load_from_storage() {
            let presets = SavedPresets {
                hs_fet_preset_idx: saved.hs_fet_preset_idx,
                ls_fet_preset_idx: saved.ls_fet_preset_idx,
                mcu_preset_idx: saved.mcu_preset_idx,
                cs_preset_idx: saved.cs_preset_idx,
                dac_preset_idx: saved.dac_preset_idx,
            };
            Self::from_params(saved.params, presets)
        } else {
            Self::from_defaults()
        }
    }
}

fn build_bode(p: &SimParams) -> Option<BodeData> {
    use crate::bode::OutputImpedanceParams;
    use crate::sim::to_cap_types;
    // Use multi-phase-aware parameters so the Bode plot reflects
    // the actual N-phase plant gain seen by the compensator.
    let ctrl = build_ctrl_params_multi(p)?;
    let (tf, _) = ctrl.to_transfer_function(p.v_in, ControlTopology::Buck);
    let ds = tf.design_summary();
    let ni = NonIdealParams::from_sim_params(p);

    // Nominal load resistance for output impedance calculation.
    let r_load = match p.load_kind {
        LoadKind::Steps => p.r_loads.first().copied().unwrap_or(10.0),
        LoadKind::Battery => p.bat_r_int_mohm * 1e-3,
    };

    if !p.output_caps.is_empty() {
        let si_caps = to_cap_types(&p.output_caps);
        let c_total = electronics_sim::cap_bank::CapBank::total_capacitance(&si_caps);
        let esr_eff = electronics_sim::cap_bank::CapBank::effective_esr(&si_caps, p.f_sw_khz * 1e3);
        let zi = OutputImpedanceParams { r_load, c_out: c_total, r_esr: esr_eff };
        Some(BodeData::compute_with_cap_bank(&ds, &ni, &si_caps, &zi))
    } else {
        let zi = OutputImpedanceParams {
            r_load,
            c_out: p.c_out_uf * 1e-6,
            r_esr: p.r_esr_mohm * 1e-3,
        };
        Some(BodeData::compute(&ds, &ni, &zi))
    }
}

// ── Settings persistence via localStorage (wasm) ─────────────────────────────

#[cfg(target_arch = "wasm32")]
const STORAGE_KEY: &str = "buck_sim_settings";

#[derive(Debug, Clone, Default)]
struct SavedPresets {
    hs_fet_preset_idx: usize,
    ls_fet_preset_idx: usize,
    mcu_preset_idx: usize,
    cs_preset_idx: usize,
    dac_preset_idx: usize,
}

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(default)]
struct SavedSettings {
    params: SimParams,
    hs_fet_preset_idx: usize,
    ls_fet_preset_idx: usize,
    mcu_preset_idx: usize,
    cs_preset_idx: usize,
    dac_preset_idx: usize,
}

impl Default for SavedSettings {
    fn default() -> Self {
        Self {
            params: SimParams::default(),
            hs_fet_preset_idx: 0,
            ls_fet_preset_idx: 0,
            mcu_preset_idx: 0,
            cs_preset_idx: 0,
            dac_preset_idx: 0,
        }
    }
}

#[cfg(target_arch = "wasm32")]
fn local_storage() -> Option<web_sys::Storage> {
    web_sys::window()?.local_storage().ok()?
}

#[cfg(target_arch = "wasm32")]
fn save_to_storage(settings: &SavedSettings) {
    if let Some(storage) = local_storage() {
        if let Ok(json) = serde_json::to_string(settings) {
            let _ = storage.set_item(STORAGE_KEY, &json);
        }
    }
}

#[cfg(target_arch = "wasm32")]
fn load_from_storage() -> Option<SavedSettings> {
    let json = local_storage()?.get_item(STORAGE_KEY).ok()??;
    serde_json::from_str(&json).ok()
}

#[cfg(target_arch = "wasm32")]
fn clear_storage() {
    if let Some(storage) = local_storage() {
        let _ = storage.remove_item(STORAGE_KEY);
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn save_to_storage(_settings: &SavedSettings) {}

#[cfg(not(target_arch = "wasm32"))]
fn load_from_storage() -> Option<SavedSettings> {
    None
}

#[cfg(not(target_arch = "wasm32"))]
fn clear_storage() {}

impl BuckSimApp {
    fn from_params(p: SimParams, presets: SavedPresets) -> Self {
        let sim_data = run_simulation(&p);
        let loss_breakdown = sim_data.as_ref().ok().and_then(|d| compute_losses(&p, d));
        let bode_data = build_bode(&p);
        Self {
            v_in: p.v_in,
            v_out_target: p.v_out_target,
            f_sw_khz: p.f_sw_khz,
            l_uh: p.l_uh,
            c_out_uf: p.c_out_uf,
            r_esr_mohm: p.r_esr_mohm,
            dcr_mohm: p.dcr_mohm,
            max_current: p.max_current,
            crossover_khz: p.crossover_khz,
            cycles_per_tick: p.cycles_per_tick,
            slope_overcomp: p.slope_overcomp,
            current_conduction: p.current_conduction,
            load_kind: p.load_kind.clone(),
            r_loads: p.r_loads.clone(),
            c_load_uf: p.c_load_uf,
            r_in_mohm: p.r_in_mohm,
            l_in_nh: p.l_in_nh,
            c_in_uf: p.c_in_uf,
            bat_v_init: p.bat_v_init,
            bat_r_int_mohm: p.bat_r_int_mohm,
            bat_c_mf: p.bat_c_mf,
            blanking_ns: p.blanking_ns,
            adc_sample_ns: p.adc_sample_ns,
            max_duty_pct: p.max_duty_pct,
            hs_fet: p.hs_fet.clone(),
            hs_fet_preset_idx: presets.hs_fet_preset_idx,
            ls_fet: p.ls_fet.clone(),
            ls_fet_preset_idx: presets.ls_fet_preset_idx,
            mcu: p.mcu.clone(),
            mcu_preset_idx: presets.mcu_preset_idx,
            cs: p.cs.clone(),
            cs_preset_idx: presets.cs_preset_idx,
            dac: p.dac.clone(),
            dac_preset_idx: presets.dac_preset_idx,
            use_cap_bank: !p.output_caps.is_empty(),
            output_caps: p.output_caps.clone(),
            num_phases: p.num_phases,
            sim_data,
            bode_data,
            loss_breakdown,
            last_params: p,
            converter_mode: ConverterMode::BuckPcmc,
            tab: Tab::Simulation,
            plot_option: PlotOption::Average,
            pfc: PfcState::new(),
        }
    }

    fn from_defaults() -> Self {
        Self::from_params(SimParams::default(), SavedPresets::default())
    }

    fn to_settings(&self) -> SavedSettings {
        SavedSettings {
            params: self.current_params(),
            hs_fet_preset_idx: self.hs_fet_preset_idx,
            ls_fet_preset_idx: self.ls_fet_preset_idx,
            mcu_preset_idx: self.mcu_preset_idx,
            cs_preset_idx: self.cs_preset_idx,
            dac_preset_idx: self.dac_preset_idx,
        }
    }

    fn current_params(&self) -> SimParams {
        SimParams {
            v_in: self.v_in,
            v_out_target: self.v_out_target,
            f_sw_khz: self.f_sw_khz,
            l_uh: self.l_uh,
            c_out_uf: self.c_out_uf,
            r_esr_mohm: self.r_esr_mohm,
            dcr_mohm: self.dcr_mohm,
            max_current: self.max_current,
            crossover_khz: self.crossover_khz,
            cycles_per_tick: self.cycles_per_tick,
            current_conduction: self.current_conduction,
            load_kind: self.load_kind.clone(),
            r_loads: self.r_loads.clone(),
            c_load_uf: self.c_load_uf,
            r_in_mohm: self.r_in_mohm,
            l_in_nh: self.l_in_nh,
            c_in_uf: self.c_in_uf,
            bat_v_init: self.bat_v_init,
            bat_r_int_mohm: self.bat_r_int_mohm,
            bat_c_mf: self.bat_c_mf,
            blanking_ns: self.blanking_ns,
            adc_sample_ns: self.adc_sample_ns,
            max_duty_pct: self.max_duty_pct,
            slope_overcomp: self.slope_overcomp,
            hs_fet: self.hs_fet.clone(),
            ls_fet: self.ls_fet.clone(),
            mcu: self.mcu.clone(),
            cs: self.cs.clone(),
            dac: self.dac.clone(),
            output_caps: self.output_caps.clone(),
            num_phases: self.num_phases,
        }
    }

    fn select_points(&self, data: &[SimPoint]) -> Vec<[f64; 2]> {
        match self.plot_option {
            PlotOption::Min => data
                .iter()
                .map(|p| [p.t_ms as f64, p.i_total_min as f64])
                .collect(),
            PlotOption::Max => data
                .iter()
                .map(|p| [p.t_ms as f64, p.i_total_max as f64])
                .collect(),
            PlotOption::Average => data
                .iter()
                .map(|p| [p.t_ms as f64, p.i_total_avg() as f64])
                .collect(),
            PlotOption::MinAndMax | PlotOption::Waveform => {
                // Handled separately in the plot code
                vec![]
            }
        }
    }

    /// Reconstruct the actual triangular inductor current waveform for a single
    /// interleaved stage within each switching cycle.
    ///
    /// Each cycle: current ramps from `i_l_min` → `i_l_max` over `t_on`,
    /// then from `i_l_max` → `i_l_min` over `T - t_on`.
    /// Stage `k` is offset by `k × T / N` from the cycle start.
    fn phase_waveform(&self, data: &[SimPoint], stage_idx: usize) -> Vec<[f64; 2]> {
        let n = self.num_phases.max(1);
        let t_sw_ms = 1.0 / self.f_sw_khz; // switching period in ms
        let offset_ms = stage_idx as f64 * t_sw_ms / n as f64;
        let mut pts = Vec::with_capacity(data.len() * 3);
        for p in data {
            if stage_idx >= p.phases.len() {
                continue;
            }
            let ph = &p.phases[stage_idx];
            let t_on_ms = ph.t_on as f64 * 1e3;
            let t0 = p.t_ms as f64 + offset_ms;
            // Start of this phase's cycle: i_l_min
            pts.push([t0, ph.i_l_min as f64]);
            // End of ON time: i_l_max
            pts.push([t0 + t_on_ms, ph.i_l_max as f64]);
            // End of this phase's cycle: back to i_l_min
            pts.push([t0 + t_sw_ms, ph.i_l_min as f64]);
        }
        pts
    }

}

// Spacing between vertically stacked plots
const PLOT_SPACING: f32 = 4.0;

impl eframe::App for BuckSimApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // ── Parameter panel ──────────────────────────────────────────────────
        egui::SidePanel::left("params")
            .min_width(320.0)
            .max_width(380.0)
            .show(ctx, |ui| {
                // ── Converter mode selector ──────────────────────────
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.converter_mode, ConverterMode::BuckPcmc, "Buck PCMC");
                    ui.selectable_value(&mut self.converter_mode, ConverterMode::PfcBoostAcm, "PFC Boost ACM");
                });
                ui.separator();

                match self.converter_mode {
                    ConverterMode::BuckPcmc => self.show_buck_params(ui),
                    ConverterMode::PfcBoostAcm => self.show_pfc_params(ui),
                }
            });

        // ── Central panel — plots ────────────────────────────────────────────
        egui::CentralPanel::default().show(ctx, |ui| {
            match self.converter_mode {
                ConverterMode::BuckPcmc => self.show_buck_central(ui),
                ConverterMode::PfcBoostAcm => self.show_pfc_central(ui),
            }
        });
    }
}

impl BuckSimApp {
    fn show_buck_params(&mut self, ui: &mut egui::Ui) {
                ui.heading("Buck Converter Parameters");
                let t_sw_us = 1000.0 / self.f_sw_khz;
                ui.separator();

                egui::ScrollArea::vertical().show(ui, |ui| {

                ui.add(
                    egui::Slider::new(&mut self.v_in, 5.0..=60.0)
                        .text("V_in [V]")
                        .step_by(0.5),
                ).on_hover_text("DC input supply voltage");
                // Clamp v_out below v_in
                if self.v_out_target >= self.v_in {
                    self.v_out_target = self.v_in - 0.5;
                }
                let v_out_max = (self.v_in - 0.5).max(0.5);
                ui.add(
                    egui::Slider::new(&mut self.v_out_target, 0.5..=v_out_max)
                        .text("V_out [V]")
                        .step_by(0.1),
                ).on_hover_text("Target regulated output voltage (must be below V_in for buck)");

                egui::CollapsingHeader::new("Input Cable / Filter")
                    .default_open(false)
                    .show(ui, |ui| {
                ui.add(
                    egui::Slider::new(&mut self.r_in_mohm, 0.0..=1000.0)
                        .text("R_cable [m\u{2126}]")
                        .step_by(1.0),
                ).on_hover_text(
                    "Cable + connector resistance between supply and input cap. \
                     Only affects plant simulation, not compensator design."
                );
                ui.add(
                    egui::Slider::new(&mut self.l_in_nh, 0.0..=10000.0)
                        .text("L_cable [nH]")
                        .step_by(1.0),
                ).on_hover_text(
                    "Cable inductance (~2 nH/cm). 50 cm \u{2248} 100 nH, 1 m \u{2248} 200 nH. \
                     Only affects plant simulation, not compensator design."
                );
                ui.add(
                    egui::Slider::new(&mut self.c_in_uf, 0.0..=1000.0)
                        .text("C_in [\u{00b5}F]")
                        .logarithmic(true)
                        .max_decimals(1),
                ).on_hover_text(
                    "Input decoupling capacitance on the converter PCB. \
                     Set to 0 for an ideal stiff source. \
                     Only affects plant simulation, not compensator design."
                );
                });

                ui.separator();
                ui.add(
                    egui::Slider::new(&mut self.f_sw_khz, 50.0..=2000.0)
                        .text("f_sw [kHz]")
                        .step_by(10.0),
                ).on_hover_text("PWM switching frequency");
                {
                    let mut np = self.num_phases as f64;
                    ui.add(
                        egui::Slider::new(&mut np, 1.0..=6.0)
                            .text("Phases")
                            .step_by(1.0),
                    ).on_hover_text("Number of interleaved power stages sharing the output capacitor");
                    self.num_phases = np as usize;
                    if self.num_phases > 1 {
                        let eff_freq = self.f_sw_khz * self.num_phases as f64;
                        let phase_offset = 360.0 / self.num_phases as f64;
                        ui.label(
                            egui::RichText::new(format!(
                                "Eff. ripple freq: {:.0} kHz, phase offset: {:.0}\u{00b0}",
                                eff_freq, phase_offset,
                            ))
                            .small()
                            .color(Color32::GRAY),
                        );
                        ui.label(
                            egui::RichText::new("L, DCR, FETs: per phase. C_out, R_ESR: total.")
                            .small()
                            .color(Color32::GRAY),
                        );
                    }
                }

                egui::CollapsingHeader::new("Switching Timing")
                    .default_open(false)
                    .show(ui, |ui| {
                ui.add(
                    egui::Slider::new(&mut self.blanking_ns, 0.0..=500.0)
                        .text("Blanking [ns]")
                        .step_by(1.0),
                ).on_hover_text("Leading-edge blanking: ignores current sense noise after switch turn-on");
                if self.blanking_ns > 0.0 {
                    ui.label(
                        egui::RichText::new(format!("{:.1}% of T_sw", self.blanking_ns / 1000.0 / t_sw_us * 100.0))
                            .small()
                            .color(Color32::GRAY),
                    );
                }
                {
                    let dac_settle_ns = self.dac.t_dac_us * 1000.0;
                    if dac_settle_ns > 0.0 && self.blanking_ns < dac_settle_ns {
                        ui.label(
                            egui::RichText::new(format!(
                                "Blanking ({:.0} ns) < DAC settle ({:.0} ns)",
                                self.blanking_ns, dac_settle_ns,
                            ))
                            .small()
                            .color(Color32::from_rgb(255, 180, 50)),
                        );
                    }
                }
                ui.add(
                    egui::Slider::new(&mut self.adc_sample_ns, 0.0..=2000.0)
                        .text("ADC sample [ns]")
                        .step_by(1.0),
                ).on_hover_text("Time for the ADC to sample the output voltage");
                ui.add(
                    egui::Slider::new(&mut self.max_duty_pct, 50.0..=100.0)
                        .text("Max duty [%]")
                        .step_by(0.1),
                ).on_hover_text("Maximum allowed duty cycle; limits on-time to ensure current sense sampling");
                });

                ui.separator();
                ui.label("Passive components").on_hover_text("Power stage inductor and output capacitor");
                ui.add(
                    egui::Slider::new(&mut self.l_uh, 0.5..=500.0)
                        .text("L [µH]")
                        .step_by(0.5),
                ).on_hover_text("Per-phase inductor value (each phase has its own inductor)");
                ui.checkbox(&mut self.use_cap_bank, "Cap bank mode")
                    .on_hover_text("Toggle between simple C+ESR and per-type capacitor bank.\n\
                                    Cap bank mode models frequency-dependent impedance of mixed cap types.");
                if self.use_cap_bank {
                    // ── Cap bank table ──────────────────────────────
                    // If switching from simple to bank for the first time, seed with one entry
                    if self.output_caps.is_empty() {
                        self.output_caps.push(CapTypeUi {
                            c_uf: self.c_out_uf,
                            count: 1,
                            esr_mohm: self.r_esr_mohm,
                            esl_nh: 0.0,
                        });
                    }

                    let mut remove_idx = None;
                    let n_caps = self.output_caps.len();
                    for i in 0..n_caps {
                        ui.horizontal(|ui| {
                            ui.label(format!("#{}", i + 1));
                            let cap = &mut self.output_caps[i];
                            ui.add(egui::DragValue::new(&mut cap.c_uf)
                                .range(0.1..=10000.0).speed(1.0).suffix(" µF"));
                            ui.add(egui::DragValue::new(&mut cap.count)
                                .range(1..=100_usize).speed(0.1).suffix("x"));
                            ui.add(egui::DragValue::new(&mut cap.esr_mohm)
                                .range(0.1..=1000.0).speed(0.1).suffix(" mΩ"));
                            ui.add(egui::DragValue::new(&mut cap.esl_nh)
                                .range(0.0..=1000.0).speed(0.1).suffix(" nH"));
                            if n_caps > 1 && ui.small_button("\u{2212}").clicked() {
                                remove_idx = Some(i);
                            }
                        });
                    }
                    if let Some(idx) = remove_idx {
                        self.output_caps.remove(idx);
                    }
                    if ui.small_button("+ Add cap type").clicked() {
                        self.output_caps.push(CapTypeUi {
                            c_uf: 10.0, count: 1, esr_mohm: 3.0, esl_nh: 0.0,
                        });
                    }

                    // Show computed totals
                    let si_caps: Vec<_> = self.output_caps.iter().map(|c| c.to_cap_type()).collect();
                    let c_total_uf = electronics_sim::cap_bank::CapBank::total_capacitance(&si_caps) * 1e6;
                    let esr_eff = electronics_sim::cap_bank::CapBank::effective_esr(&si_caps, self.f_sw_khz * 1e3);
                    ui.label(egui::RichText::new(
                        format!("C_total: {:.1} µF, ESR_eff @ f_sw: {:.2} mΩ", c_total_uf, esr_eff * 1e3)
                    ).small().color(Color32::GRAY));
                    ui.label(egui::RichText::new(
                        format!("(Compensator designed for C={:.1}µF, ESR={:.2}mΩ)", c_total_uf, esr_eff * 1e3)
                    ).small().color(Color32::LIGHT_YELLOW));
                } else {
                    // Clear cap bank when switching back to simple mode
                    self.output_caps.clear();
                    ui.add(
                        egui::Slider::new(&mut self.c_out_uf, 1.0..=5000.0)
                            .text("C_out [µF]")
                            .step_by(1.0),
                    ).on_hover_text("Total output capacitance (shared across all phases)");
                    ui.add(
                        egui::Slider::new(&mut self.r_esr_mohm, 0.0..=500.0)
                            .text("R_ESR [mΩ]")
                            .step_by(0.1),
                    ).on_hover_text("Total ESR of the shared output capacitor bank");
                }
                ui.add(
                    egui::Slider::new(&mut self.dcr_mohm, 0.0..=50.0)
                        .text("DCR [mΩ]")
                        .step_by(0.1),
                ).on_hover_text("Per-phase inductor DC resistance");
                {
                    let r_path = computed_r_series_mohm(&self.current_params());
                    ui.label(
                        egui::RichText::new(format!("R_path: {:.1} mΩ (DCR + Rds weighted)", r_path))
                            .small()
                            .color(Color32::GRAY),
                    );
                }

                ui.separator();
                ui.label("Current sense").on_hover_text("Current measurement for peak current mode control");
                ui.add(
                    egui::Slider::new(&mut self.max_current, 0.5..=100.0)
                        .text("I_max [A]")
                        .step_by(0.5),
                ).on_hover_text("Overcurrent protection threshold");

                // ── Controller tuning ───────────────────────────────────────
                ui.separator();
                ui.label("Controller").on_hover_text("2P2Z voltage loop compensator tuning");
                let f_sw_half = self.f_sw_khz / 2.0;
                ui.add(
                    egui::Slider::new(&mut self.crossover_khz, 1.0..=f_sw_half)
                        .text("f_x [kHz]")
                        .logarithmic(true)
                        .max_decimals(1),
                ).on_hover_text("Desired loop crossover (bandwidth) frequency");
                if let Some(ctrl) = build_ctrl_params_multi(&self.current_params()) {
                    let max_fx = ctrl.max_feasible_crossover_hz(self.v_in, ControlTopology::Buck);
                    ui.label(
                        egui::RichText::new(format!("max: {:.1} kHz", max_fx / 1e3))
                            .small()
                            .color(Color32::GRAY),
                    );
                }
                let mut cpt = self.cycles_per_tick as f64;
                ui.add(
                    egui::Slider::new(&mut cpt, 1.0..=8.0)
                        .text("cycles/tick")
                        .step_by(1.0),
                ).on_hover_text("PWM cycles between compensator updates; >1 reduces CPU load but lowers effective bandwidth");
                self.cycles_per_tick = cpt as usize;
                {
                    let cpu_time_us = self.mcu.t_adc_us + self.mcu.t_processing_us + self.dac.t_dac_us;
                    let cpu_pct = cpu_time_us / (t_sw_us * self.cycles_per_tick as f64) * 100.0;
                    let color = if cpu_pct > 100.0 {
                        Color32::from_rgb(255, 180, 50)
                    } else {
                        Color32::GRAY
                    };
                    ui.label(
                        egui::RichText::new(format!("CPU: {:.1}%", cpu_pct))
                            .small()
                            .color(color),
                    );
                    if cpu_pct > 100.0 {
                        let needed = (cpu_time_us / t_sw_us).ceil() as usize;
                        ui.label(
                            egui::RichText::new(format!("increase cycles/tick to {}", needed))
                                .small()
                                .color(Color32::from_rgb(255, 180, 50)),
                        );
                    }
                }
                ui.add(
                    egui::Slider::new(&mut self.slope_overcomp, 1.0..=3.0)
                        .text("Slope overcomp")
                        .step_by(0.1),
                ).on_hover_text("Slope compensation margin above the subharmonic stability limit (1.0 = exact, 1.5 = 50% extra)");

                // ── Hardware profiles ───────────────────────────────────
                ui.separator();
                egui::CollapsingHeader::new("Hardware")
                    .default_open(false)
                    .show(ui, |ui| {
                        // ── High-Side FET ────────────────────────────────
                        ui.label("High-Side FET").on_hover_text("High-side switch parameters for loss estimation");
                        {
                            let presets = FetProfile::presets();
                            let label = if self.hs_fet_preset_idx < presets.len() {
                                presets[self.hs_fet_preset_idx].name.as_str()
                            } else { "Custom" };
                            egui::ComboBox::from_id_salt("hs_fet_preset")
                                .selected_text(label)
                                .show_ui(ui, |ui| {
                                    for (i, p) in presets.iter().enumerate() {
                                        if ui.selectable_value(
                                            &mut self.hs_fet_preset_idx, i, &p.name,
                                        ).clicked() {
                                            self.hs_fet = presets[i].clone();
                                        }
                                    }
                                    ui.selectable_value(
                                        &mut self.hs_fet_preset_idx, usize::MAX, "Custom",
                                    );
                                });

                            let before = self.hs_fet.clone();
                            ui.add(egui::Slider::new(&mut self.hs_fet.rds_on_mohm, 0.0..=100.0)
                                .text("Rds(on) [mΩ]").step_by(0.1))
                                .on_hover_text("On-state drain-source resistance");
                            ui.add(egui::Slider::new(&mut self.hs_fet.coss_pf, 0.0..=5000.0)
                                .text("Coss [pF]").step_by(10.0))
                                .on_hover_text("Output capacitance (for Eoss switching loss)");
                            ui.add(egui::Slider::new(&mut self.hs_fet.qg_nc, 0.0..=100.0)
                                .text("Qg [nC]").step_by(0.1))
                                .on_hover_text("Total gate charge (for gate drive loss)");
                            ui.add(egui::Slider::new(&mut self.hs_fet.vgs_v, 1.0..=20.0)
                                .text("Vgs [V]").step_by(0.1))
                                .on_hover_text("Gate drive voltage");
                            ui.add(egui::Slider::new(&mut self.hs_fet.t_rise_ns, 0.0..=50.0)
                                .text("t_rise [ns]").step_by(0.1))
                                .on_hover_text("Current rise time (V×I overlap at turn-on)");
                            ui.add(egui::Slider::new(&mut self.hs_fet.t_fall_ns, 0.0..=50.0)
                                .text("t_fall [ns]").step_by(0.1))
                                .on_hover_text("Current fall time (V×I overlap at turn-off)");

                            if self.hs_fet != before && self.hs_fet_preset_idx != usize::MAX {
                                self.hs_fet_preset_idx = usize::MAX;
                            }
                        }

                        ui.separator();

                        // ── Low-Side FET ─────────────────────────────────
                        ui.label("Low-Side FET").on_hover_text("Low-side (synchronous) switch parameters");
                        {
                            let presets = FetProfile::presets();
                            let label = if self.ls_fet_preset_idx < presets.len() {
                                presets[self.ls_fet_preset_idx].name.as_str()
                            } else { "Custom" };
                            egui::ComboBox::from_id_salt("ls_fet_preset")
                                .selected_text(label)
                                .show_ui(ui, |ui| {
                                    for (i, p) in presets.iter().enumerate() {
                                        if ui.selectable_value(
                                            &mut self.ls_fet_preset_idx, i, &p.name,
                                        ).clicked() {
                                            self.ls_fet = presets[i].clone();
                                        }
                                    }
                                    ui.selectable_value(
                                        &mut self.ls_fet_preset_idx, usize::MAX, "Custom",
                                    );
                                });

                            let before = self.ls_fet.clone();
                            ui.add(egui::Slider::new(&mut self.ls_fet.rds_on_mohm, 0.0..=100.0)
                                .text("Rds(on) [mΩ]").step_by(0.1))
                                .on_hover_text("On-state drain-source resistance");
                            ui.add(egui::Slider::new(&mut self.ls_fet.coss_pf, 0.0..=5000.0)
                                .text("Coss [pF]").step_by(10.0))
                                .on_hover_text("Output capacitance (discharged through HS each cycle)");
                            ui.add(egui::Slider::new(&mut self.ls_fet.qg_nc, 0.0..=100.0)
                                .text("Qg [nC]").step_by(0.1))
                                .on_hover_text("Total gate charge");
                            ui.add(egui::Slider::new(&mut self.ls_fet.vgs_v, 1.0..=20.0)
                                .text("Vgs [V]").step_by(0.1))
                                .on_hover_text("Gate drive voltage");
                            ui.add(egui::Slider::new(&mut self.ls_fet.t_rise_ns, 0.0..=50.0)
                                .text("t_rise [ns]").step_by(0.1))
                                .on_hover_text("Current rise time");
                            ui.add(egui::Slider::new(&mut self.ls_fet.t_fall_ns, 0.0..=50.0)
                                .text("t_fall [ns]").step_by(0.1))
                                .on_hover_text("Current fall time");

                            if self.ls_fet != before && self.ls_fet_preset_idx != usize::MAX {
                                self.ls_fet_preset_idx = usize::MAX;
                            }
                        }

                        ui.separator();

                        // ── MCU ───────────────────────────────────────────
                        ui.label("MCU").on_hover_text("Microcontroller timing parameters");
                        {
                            let presets = McuProfile::presets();
                            let label = if self.mcu_preset_idx < presets.len() {
                                presets[self.mcu_preset_idx].name.as_str()
                            } else { "Custom" };
                            egui::ComboBox::from_id_salt("mcu_preset")
                                .selected_text(label)
                                .show_ui(ui, |ui| {
                                    for (i, p) in presets.iter().enumerate() {
                                        if ui.selectable_value(
                                            &mut self.mcu_preset_idx, i, &p.name,
                                        ).clicked() {
                                            self.mcu = presets[i].clone();
                                        }
                                    }
                                    ui.selectable_value(
                                        &mut self.mcu_preset_idx, usize::MAX, "Custom",
                                    );
                                });

                            let before = self.mcu.clone();
                            ui.add(
                                egui::Slider::new(&mut self.mcu.comp_delay_ns, 0.0..=500.0)
                                    .text("Comp delay [ns]")
                                    .step_by(1.0),
                            ).on_hover_text("Comparator propagation delay from current threshold crossing to switch turn-off");
                            if self.mcu.comp_delay_ns > 0.0 {
                                ui.label(
                                    egui::RichText::new(format!("{:.1}% of T_sw", self.mcu.comp_delay_ns / 1000.0 / t_sw_us * 100.0))
                                        .small()
                                        .color(Color32::GRAY),
                                );
                            }
                            ui.add(
                                egui::Slider::new(&mut self.mcu.t_adc_us, 0.0..=10.0)
                                    .text("t_ADC [us]")
                                    .step_by(0.1),
                            ).on_hover_text("ADC conversion time per sample");
                            if self.mcu.t_adc_us > 0.0 {
                                ui.label(
                                    egui::RichText::new(format!("{:.1}% of T_sw", self.mcu.t_adc_us / t_sw_us * 100.0))
                                        .small()
                                        .color(Color32::GRAY),
                                );
                            }
                            ui.add(
                                egui::Slider::new(&mut self.mcu.t_processing_us, 0.0..=10.0)
                                    .text("t_proc [us]")
                                    .step_by(0.1),
                            ).on_hover_text("CPU processing time for compensator computation");
                            if self.mcu.t_processing_us > 0.0 {
                                ui.label(
                                    egui::RichText::new(format!("{:.1}% of T_sw", self.mcu.t_processing_us / t_sw_us * 100.0))
                                        .small()
                                        .color(Color32::GRAY),
                                );
                            }
                            let mut steps = self.mcu.min_slope_steps_on_time as f64;
                            ui.add(
                                egui::Slider::new(&mut steps, 0.0..=200.0)
                                    .text("min steps/t_on")
                                    .step_by(1.0),
                            ).on_hover_text("Min DAC slope steps during on-time (0 = ideal)");
                            self.mcu.min_slope_steps_on_time = steps as u16;

                            if self.mcu != before && self.mcu_preset_idx != usize::MAX {
                                self.mcu_preset_idx = usize::MAX;
                            }
                        }

                        ui.separator();

                        // ── Current sensor ────────────────────────────────
                        ui.label("Current sensor").on_hover_text("Current sense amplifier characteristics");
                        {
                            let presets = CsProfile::presets();
                            let label = if self.cs_preset_idx < presets.len() {
                                presets[self.cs_preset_idx].name.as_str()
                            } else { "Custom" };
                            egui::ComboBox::from_id_salt("cs_preset")
                                .selected_text(label)
                                .show_ui(ui, |ui| {
                                    for (i, p) in presets.iter().enumerate() {
                                        if ui.selectable_value(
                                            &mut self.cs_preset_idx, i, &p.name,
                                        ).clicked() {
                                            self.cs = presets[i].clone();
                                        }
                                    }
                                    ui.selectable_value(
                                        &mut self.cs_preset_idx, usize::MAX, "Custom",
                                    );
                                });

                            let before = self.cs.clone();
                            ui.add(
                                egui::Slider::new(&mut self.cs.cs_gain_mv_a, 10.0..=500.0)
                                    .text("CS gain [mV/A]")
                                    .step_by(1.0),
                            ).on_hover_text("Current sensor transimpedance gain (output voltage per amp of inductor current)");
                            ui.add(
                                egui::Slider::new(&mut self.cs.cs_bandwidth_khz, 0.0..=10000.0)
                                    .text("CS BW [kHz]")
                                    .logarithmic(true)
                                    .max_decimals(0),
                            ).on_hover_text("Current sensor analog bandwidth (-3 dB); affects sensed ripple attenuation");

                            if self.cs != before && self.cs_preset_idx != usize::MAX {
                                self.cs_preset_idx = usize::MAX;
                            }
                        }

                        ui.separator();

                        // ── Slope DAC ─────────────────────────────────────
                        ui.label("Slope DAC").on_hover_text("DAC used to generate the slope compensation ramp");
                        {
                            let presets = DacProfile::presets();
                            let label = if self.dac_preset_idx < presets.len() {
                                presets[self.dac_preset_idx].name.as_str()
                            } else { "Custom" };
                            egui::ComboBox::from_id_salt("dac_preset")
                                .selected_text(label)
                                .show_ui(ui, |ui| {
                                    for (i, p) in presets.iter().enumerate() {
                                        if ui.selectable_value(
                                            &mut self.dac_preset_idx, i, &p.name,
                                        ).clicked() {
                                            self.dac = presets[i].clone();
                                        }
                                    }
                                    ui.selectable_value(
                                        &mut self.dac_preset_idx, usize::MAX, "Custom",
                                    );
                                });

                            let before = self.dac.clone();
                            ui.add(
                                egui::Slider::new(&mut self.dac.dac_filter_bw_khz, 0.0..=10000.0)
                                    .text("DAC BW [kHz]")
                                    .logarithmic(true)
                                    .max_decimals(0),
                            ).on_hover_text("Slope DAC output filter bandwidth; limits ramp slew rate");
                            ui.add(
                                egui::Slider::new(&mut self.dac.t_dac_us, 0.0..=10.0)
                                    .text("t_DAC [us]")
                                    .step_by(0.01),
                            ).on_hover_text("DAC output settling time after code update");
                            if self.dac.t_dac_us > 0.0 {
                                ui.label(
                                    egui::RichText::new(format!("{:.1}% of T_sw", self.dac.t_dac_us / t_sw_us * 100.0))
                                        .small()
                                        .color(Color32::GRAY),
                                );
                            }

                            if self.dac != before && self.dac_preset_idx != usize::MAX {
                                self.dac_preset_idx = usize::MAX;
                            }
                        }
                    });

                ui.separator();
                ui.horizontal(|ui| {
                    ui.label("Rectifier:");
                    ui.radio_value(
                        &mut self.current_conduction,
                        CurrentConduction::Synchronous,
                        "Sync",
                    ).on_hover_text("Synchronous rectification: forced continuous conduction mode (CCM)");
                    ui.radio_value(
                        &mut self.current_conduction,
                        CurrentConduction::Diode,
                        "Diode",
                    ).on_hover_text("Diode rectification: allows discontinuous conduction mode (DCM) at light load");
                });

                // ── Load section ─────────────────────────────────────────────
                ui.separator();
                egui::CollapsingHeader::new("Load")
                    .default_open(true)
                    .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.load_kind, LoadKind::Steps, "Load steps")
                        .on_hover_text("Stepped resistive load for transient response testing");
                    ui.radio_value(&mut self.load_kind, LoadKind::Battery, "Battery")
                        .on_hover_text("Constant-voltage battery charging load model");
                });

                match self.load_kind {
                    LoadKind::Steps => {
                        ui.add_space(2.0);
                        let i = self.v_out_target / self.r_loads[0];
                        let p = self.v_out_target * i;
                        // Nominal load — always present, no remove button
                        ui.add(
                            egui::Slider::new(&mut self.r_loads[0], 0.01..=1000.0)
                                .text(format!("R_nom [Ω], {i:.1}A, {p:.1}W"))
                                .logarithmic(true)
                                .max_decimals(2),
                        ).on_hover_text("Initial load resistance before any steps");

                        // Additional load steps — each has a remove button
                        let mut to_remove: Option<usize> = None;
                        for step in 1..self.r_loads.len() {
                            let clicked = ui
                                .horizontal(|ui| {
                                    let i = self.v_out_target / self.r_loads[step];
                                    let p = self.v_out_target * i;
                                    ui.add(
                                        egui::Slider::new(&mut self.r_loads[step], 0.1..=10000.0)
                                            .text(format!("[Ω], {:.1}A, {:.1}W", i, p))
                                            .logarithmic(true)
                                            .max_decimals(2),
                                    ).on_hover_text("Load resistance for this step; steps are applied sequentially");
                                    ui.small_button("−").clicked()
                                })
                                .inner;
                            if clicked {
                                to_remove = Some(step);
                                break;
                            }
                        }
                        if let Some(i) = to_remove {
                            self.r_loads.remove(i);
                        }

                        if ui.small_button("+ Add step").clicked() {
                            let last = *self.r_loads.last().unwrap();
                            self.r_loads.push(last);
                        }
                    }

                    LoadKind::Battery => {
                        ui.add_space(2.0);
                        // Clamp bat_v_init below v_out_target
                        self.bat_v_init = self.bat_v_init.min(self.v_out_target - 0.1);
                        let bat_v_max = (self.v_out_target - 0.1).max(0.5);
                        ui.add(
                            egui::Slider::new(&mut self.bat_v_init, 0.5..=bat_v_max)
                                .text("V_bat init [V]")
                                .step_by(0.1),
                        ).on_hover_text("Battery initial open-circuit voltage before charging begins");
                        ui.add(
                            egui::Slider::new(&mut self.bat_r_int_mohm, 5.0..=500.0)
                                .text("R_int [mΩ]")
                                .step_by(1.0),
                        ).on_hover_text("Battery internal series resistance");
                        ui.add(
                            egui::Slider::new(&mut self.bat_c_mf, 1.0..=1000.0)
                                .text("C_bat [mF]")
                                .logarithmic(true)
                                .max_decimals(1),
                        ).on_hover_text("Battery equivalent capacitance (models charge storage; larger = slower voltage rise)");
                    }
                }

                ui.add_space(2.0);
                ui.add(
                    egui::Slider::new(&mut self.c_load_uf, 0.0..=10000.0)
                        .text("C_load [µF]")
                        .logarithmic(true)
                        .max_decimals(1),
                ).on_hover_text(
                    "Extra capacitance on the load side. Only affects the plant simulation, \
                     NOT the compensator design. Use this to test how much extra capacitance \
                     the control loop can tolerate."
                );
                }); // Load

                ui.separator();
                ui.label("Plot Options").on_hover_text("Select which inductor current trace to display");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.plot_option, PlotOption::Min, "Min")
                        .on_hover_text("Inductor current valley (minimum per cycle)");
                    ui.radio_value(&mut self.plot_option, PlotOption::Max, "Max")
                        .on_hover_text("Inductor current peak (maximum per cycle)");
                    ui.radio_value(&mut self.plot_option, PlotOption::Average, "Average")
                        .on_hover_text("Average inductor current per cycle");
                    ui.radio_value(&mut self.plot_option, PlotOption::MinAndMax, "Min and Max")
                        .on_hover_text("Both peak and valley inductor current");
                });
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.plot_option, PlotOption::Waveform, "Waveform")
                        .on_hover_text("Reconstructed per-phase triangular inductor current waveforms");
                });

                // ── Power Losses ─────────────────────────────────────────
                ui.separator();
                egui::CollapsingHeader::new("Power Losses")
                    .default_open(true)
                    .show(ui, |ui| {
                        if let Some(ref lb) = self.loss_breakdown {
                            let fmt_mw = |w: f64| -> String {
                                if w >= 1.0 {
                                    format!("{:.2} W", w)
                                } else {
                                    format!("{:.1} mW", w * 1e3)
                                }
                            };
                            ui.label("Conduction:");
                            ui.label(egui::RichText::new(format!("  HS FET:    {}", fmt_mw(lb.hs_conduction_w))).small().monospace());
                            ui.label(egui::RichText::new(format!("  LS FET:    {}", fmt_mw(lb.ls_conduction_w))).small().monospace());
                            ui.label(egui::RichText::new(format!("  Inductor:  {}", fmt_mw(lb.inductor_dcr_w))).small().monospace());
                            ui.label("Switching:");
                            ui.label(egui::RichText::new(format!("  HS overlap: {}", fmt_mw(lb.hs_switching_w))).small().monospace());
                            ui.label(egui::RichText::new(format!("  LS overlap: {}", fmt_mw(lb.ls_switching_w))).small().monospace());
                            ui.label(egui::RichText::new(format!("  Eoss:       {}", fmt_mw(lb.hs_eoss_w))).small().monospace());
                            ui.label(egui::RichText::new(format!("  Gate drive: {}", fmt_mw(lb.gate_drive_w))).small().monospace());
                            ui.separator();
                            ui.label(egui::RichText::new(format!("Total loss:   {}", fmt_mw(lb.total_loss_w))).monospace());
                            ui.label(egui::RichText::new(format!("P_out:        {:.2} W", lb.p_out_w)).monospace());
                            ui.label(egui::RichText::new(format!("Efficiency:   {:.1}%", lb.efficiency_pct)).monospace());
                        } else {
                            ui.label(egui::RichText::new("No data").small().color(Color32::GRAY));
                        }
                    });

                // ── Output Capacitor ───────────────────────────────────
                ui.separator();
                egui::CollapsingHeader::new("Output Capacitor")
                    .default_open(true)
                    .show(ui, |ui| {
                        if let Ok(ref data) = self.sim_data {
                            let settled_tail = STEADY_STATE_CYCLES / 4;
                            let mut worst_pp: f32 = 0.0;
                            let mut worst_load = 0.0_f64;

                            match self.load_kind {
                                LoadKind::Steps => {
                                    // Steady-state segment at r_loads[0]
                                    let mut segments: Vec<(usize, usize, f64)> = vec![
                                        (SOFT_START_CYCLES, SOFT_START_CYCLES + STEADY_STATE_CYCLES, self.r_loads[0]),
                                    ];
                                    let mut offset = SOFT_START_CYCLES + STEADY_STATE_CYCLES;
                                    for &r in &self.r_loads[1..] {
                                        segments.push((offset, offset + LOAD_STEP_CYCLES, r));
                                        offset += LOAD_STEP_CYCLES;
                                    }
                                    for &(start, end, r_load) in &segments {
                                        let end = end.min(data.len());
                                        let start = start.min(end);
                                        let tail_n = settled_tail.min(end - start);
                                        if tail_n == 0 { continue; }
                                        let tail = &data[end - tail_n..end];
                                        let pp: f32 = tail.iter()
                                            .map(|p| p.i_total_max - p.i_total_min)
                                            .fold(0.0_f32, f32::max);
                                        if pp > worst_pp {
                                            worst_pp = pp;
                                            worst_load = r_load;
                                        }
                                    }
                                }
                                LoadKind::Battery => {
                                    let n = settled_tail.min(data.len());
                                    let tail = &data[data.len() - n..];
                                    worst_pp = tail.iter()
                                        .map(|p| p.i_total_max - p.i_total_min)
                                        .fold(0.0_f32, f32::max);
                                }
                            }

                            let rms = worst_pp as f64 / (2.0 * 3.0_f64.sqrt());
                            ui.label(egui::RichText::new(
                                format!("Ripple I_pp:  {:.2} A", worst_pp)
                            ).monospace());
                            ui.label(egui::RichText::new(
                                format!("Ripple I_rms: {:.2} A", rms)
                            ).monospace());
                            if self.load_kind == LoadKind::Steps && worst_load > 0.0 {
                                ui.label(egui::RichText::new(
                                    format!("Worst load:   {:.1} \u{2126}", worst_load)
                                ).small().monospace());
                            }

                            // ── Voltage ripple (same logic: worst-case from settled tails) ──
                            ui.separator();
                            let mut worst_v_pp: f32 = 0.0;
                            let mut worst_v_load = 0.0_f64;
                            match self.load_kind {
                                LoadKind::Steps => {
                                    let mut segments: Vec<(usize, usize, f64)> = vec![
                                        (SOFT_START_CYCLES, SOFT_START_CYCLES + STEADY_STATE_CYCLES, self.r_loads[0]),
                                    ];
                                    let mut offset = SOFT_START_CYCLES + STEADY_STATE_CYCLES;
                                    for &r in &self.r_loads[1..] {
                                        segments.push((offset, offset + LOAD_STEP_CYCLES, r));
                                        offset += LOAD_STEP_CYCLES;
                                    }
                                    for &(start, end, r_load) in &segments {
                                        let end = end.min(data.len());
                                        let start = start.min(end);
                                        let tail_n = settled_tail.min(end - start);
                                        if tail_n == 0 { continue; }
                                        let tail = &data[end - tail_n..end];
                                        let vpp: f32 = tail.iter()
                                            .map(|p| p.v_out_max - p.v_out_min)
                                            .fold(0.0_f32, f32::max);
                                        if vpp > worst_v_pp {
                                            worst_v_pp = vpp;
                                            worst_v_load = r_load;
                                        }
                                    }
                                }
                                LoadKind::Battery => {
                                    let n = settled_tail.min(data.len());
                                    let tail = &data[data.len() - n..];
                                    worst_v_pp = tail.iter()
                                        .map(|p| p.v_out_max - p.v_out_min)
                                        .fold(0.0_f32, f32::max);
                                }
                            }
                            ui.label(egui::RichText::new(
                                format!("Ripple V_pp:  {:.1} mV", worst_v_pp * 1e3)
                            ).monospace());
                            if self.load_kind == LoadKind::Steps && worst_v_load > 0.0 {
                                ui.label(egui::RichText::new(
                                    format!("Worst load:   {:.1} \u{2126}", worst_v_load)
                                ).small().monospace());
                            }
                        } else {
                            ui.label(egui::RichText::new("No data").small().color(Color32::GRAY));
                        }
                    });

                ui.separator();
                ui.label(
                    egui::RichText::new("Plot updates live as sliders move.")
                        .small()
                        .color(Color32::GRAY),
                );

                ui.add_space(8.0);
                if ui.button("Reset to defaults").clicked() {
                    clear_storage();
                    *self = Self::from_defaults();
                }
                }); // ScrollArea
    }

    fn show_buck_central(&mut self, ui: &mut egui::Ui) {
        // Re-run simulation / bode only when params changed
        let params = self.current_params();
        if params != self.last_params {
            self.sim_data = run_simulation(&params);
            self.loss_breakdown = self.sim_data.as_ref().ok().and_then(|d| compute_losses(&params, d));
            self.bode_data = build_bode(&params);
            self.last_params = params;
            save_to_storage(&self.to_settings());
        }

        // Tab bar
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.tab, Tab::Simulation, "Simulation");
            ui.selectable_value(&mut self.tab, Tab::Bode, "Bode");
        });
        ui.separator();

        match self.tab {
            Tab::Simulation => self.show_simulation(ui),
            Tab::Bode => {
                match &self.bode_data {
                    Some(data) => show_bode(ui, data),
                    None => {
                        ui.centered_and_justified(|ui| {
                            ui.colored_label(
                                Color32::from_rgb(255, 100, 100),
                                "Invalid parameters for Bode computation.",
                            );
                        });
                    }
                }
            }
        }
    }

    fn show_pfc_params(&mut self, ui: &mut egui::Ui) {
        ui.heading("PFC Boost Parameters");
        ui.separator();

        egui::ScrollArea::vertical().show(ui, |ui| {
            // Control mode
            ui.horizontal(|ui| {
                ui.label("Control:");
                ui.radio_value(&mut self.pfc.control_mode, PfcControlMode::Acm, "ACM")
                    .on_hover_text("Average Current Mode — inner 2P2Z current loop + outer voltage loop");
                ui.radio_value(&mut self.pfc.control_mode, PfcControlMode::CrCm, "CrCM")
                    .on_hover_text("Critical Conduction Mode — variable frequency, voltage loop sets on-time. Natural PFC, ZVS capable.");
                ui.radio_value(&mut self.pfc.control_mode, PfcControlMode::Pcmc, "PCMC")
                    .on_hover_text("Peak Current Mode — fixed frequency, comparator + slope compensation. D>0.5 requires careful slope comp.");
            });

            // Topology
            ui.horizontal(|ui| {
                ui.label("Topology:");
                ui.radio_value(&mut self.pfc.topology, PfcTopology::BridgedBoost, "Bridged")
                    .on_hover_text("Diode bridge rectifier + boost stage");
                ui.radio_value(&mut self.pfc.topology, PfcTopology::TotemPole, "Totem-pole")
                    .on_hover_text("Bridgeless totem-pole — no bridge diode loss, inherently synchronous.");
                ui.radio_value(&mut self.pfc.topology, PfcTopology::Vienna, "Vienna")
                    .on_hover_text("3-level Vienna rectifier — split DC bus, 3-phase only.");
                ui.radio_value(&mut self.pfc.topology, PfcTopology::ActiveBridge, "6-switch")
                    .on_hover_text("6-switch active bridge (B6 VSR) — d-q control, 3-phase only.");
            });

            // Force 3-phase for Vienna and ActiveBridge
            if matches!(self.pfc.topology, PfcTopology::Vienna | PfcTopology::ActiveBridge) {
                self.pfc.ac_phases = 3;
            }

            // AC phases
            ui.horizontal(|ui| {
                ui.label("AC Phases:");
                let three_phase_only = matches!(
                    self.pfc.topology,
                    PfcTopology::Vienna | PfcTopology::ActiveBridge
                );
                if !three_phase_only {
                    ui.selectable_value(&mut self.pfc.ac_phases, 1, "1-phase");
                    ui.selectable_value(&mut self.pfc.ac_phases, 3, "3-phase");
                } else {
                    ui.label("3-phase (required)");
                }
            });

            ui.separator();

            ui.add(egui::Slider::new(&mut self.pfc.v_in_rms, 85.0..=265.0)
                .text("V_in RMS [V]").step_by(1.0))
                .on_hover_text("AC mains RMS voltage (line-to-neutral for 3-phase)");

            ui.horizontal(|ui| {
                ui.label("f_line:");
                ui.selectable_value(&mut self.pfc.f_line_hz, 50.0, "50 Hz");
                ui.selectable_value(&mut self.pfc.f_line_hz, 60.0, "60 Hz");
            });

            ui.add(egui::Slider::new(&mut self.pfc.v_out, 200.0..=800.0)
                .text("V_out [V]").step_by(1.0))
                .on_hover_text("DC bus output voltage");

            ui.add(egui::Slider::new(&mut self.pfc.f_sw_khz, 20.0..=200.0)
                .text("f_sw [kHz]").step_by(1.0))
                .on_hover_text("Switching frequency");

            ui.add(egui::Slider::new(&mut self.pfc.l_uh, 50.0..=5000.0)
                .text("L [µH]").logarithmic(true))
                .on_hover_text("Boost inductor");

            // Multi-phase
            {
                let mut np = self.pfc.num_phases as f64;
                ui.add(egui::Slider::new(&mut np, 1.0..=4.0)
                    .text("Phases").step_by(1.0))
                    .on_hover_text("Number of interleaved PFC phases");
                self.pfc.num_phases = np as usize;
                if self.pfc.num_phases > 1 {
                    let eff_freq = self.pfc.f_sw_khz * self.pfc.num_phases as f64;
                    ui.label(egui::RichText::new(format!(
                        "  Eff. ripple freq: {eff_freq:.0} kHz  |  Phase offset: {:.0}\u{00B0}",
                        360.0 / self.pfc.num_phases as f64
                    )).small());
                }
            }

            // Load steps
            {
                if self.pfc.p_loads_w.is_empty() {
                    self.pfc.p_loads_w.push(300.0);
                }
                let i_nom = self.pfc.p_loads_w[0] / self.pfc.v_out;
                ui.add(egui::Slider::new(&mut self.pfc.p_loads_w[0], 10.0..=3000.0)
                    .text(format!("P_nom [W] ({i_nom:.2}A)")).step_by(10.0))
                    .on_hover_text("Nominal output load power");

                let mut to_remove: Option<usize> = None;
                for step in 1..self.pfc.p_loads_w.len() {
                    let clicked = ui.horizontal(|ui| {
                        ui.add(egui::Slider::new(&mut self.pfc.p_loads_w[step], 10.0..=3000.0)
                            .text("Step [W]").step_by(10.0));
                        ui.small_button("x").clicked()
                    }).inner;
                    if clicked { to_remove = Some(step); }
                }
                if let Some(i) = to_remove { self.pfc.p_loads_w.remove(i); }
                if ui.small_button("+ Add load step").clicked() {
                    let last = *self.pfc.p_loads_w.last().unwrap();
                    self.pfc.p_loads_w.push(last);
                }
            }

            egui::CollapsingHeader::new("Output Capacitor")
                .default_open(false)
                .show(ui, |ui| {
                    ui.add(egui::Slider::new(&mut self.pfc.c_out_uf, 10.0..=2000.0)
                        .text("C_out [µF]").logarithmic(true))
                        .on_hover_text("DC bus capacitor");
                    ui.add(egui::Slider::new(&mut self.pfc.r_esr_mohm, 1.0..=500.0)
                        .text("ESR [mΩ]").logarithmic(true))
                        .on_hover_text("Output cap ESR");
                });

            egui::CollapsingHeader::new("FET / Diode / Inductor")
                .default_open(false)
                .show(ui, |ui| {
                    if self.pfc.topology == PfcTopology::BridgedBoost {
                        ui.horizontal(|ui| {
                            ui.label("Rectifier:");
                            ui.radio_value(&mut self.pfc.current_conduction, CurrentConduction::Diode, "Diode")
                                .on_hover_text("Standard boost diode — DCM at light load and zero crossings");
                            ui.radio_value(&mut self.pfc.current_conduction, CurrentConduction::Synchronous, "Sync FET")
                                .on_hover_text("Synchronous rectification — current can go negative");
                        });
                        if self.pfc.current_conduction == CurrentConduction::Diode {
                            ui.add(egui::Slider::new(&mut self.pfc.v_diode, 0.0..=2.0)
                                .text("V_diode [V]").step_by(0.1));
                        }
                    } else {
                        ui.label(egui::RichText::new("Totem-pole: sync FET, no bridge diode").small());
                    }
                    ui.add(egui::Slider::new(&mut self.pfc.rds_on_mohm, 1.0..=500.0)
                        .text("Rds(on) [mΩ]").logarithmic(true));
                    ui.add(egui::Slider::new(&mut self.pfc.dcr_mohm, 1.0..=200.0)
                        .text("DCR [mΩ]").logarithmic(true));
                    ui.add(egui::Slider::new(&mut self.pfc.r_sense_mohm, 1.0..=200.0)
                        .text("R_sense [mΩ]").logarithmic(true));
                });

            egui::CollapsingHeader::new("Controller")
                .default_open(true)
                .show(ui, |ui| {
                    if self.pfc.control_mode == PfcControlMode::Acm {
                        ui.add(egui::Slider::new(&mut self.pfc.current_crossover_khz, 0.5..=50.0)
                            .text("I-loop f_x [kHz]").logarithmic(true))
                            .on_hover_text("Inner current loop crossover frequency (ACM only)");
                    }
                    if self.pfc.control_mode == PfcControlMode::Pcmc {
                        ui.add(egui::Slider::new(&mut self.pfc.slope_overcomp, 1.0..=5.0)
                            .text("Slope overcomp").step_by(0.1))
                            .on_hover_text("Slope compensation margin (1.0 = exact stability limit, 1.5 = 50% extra)");
                    }
                    let max_v_xover = (self.pfc.f_line_hz / 3.0 - 0.1).max(1.0);
                    if self.pfc.voltage_crossover_hz > max_v_xover {
                        self.pfc.voltage_crossover_hz = max_v_xover;
                    }
                    ui.add(egui::Slider::new(&mut self.pfc.voltage_crossover_hz, 1.0..=max_v_xover)
                        .text("V-loop f_x [Hz]").step_by(0.5))
                        .on_hover_text("Outer voltage loop crossover (must be < f_line/3)");
                });

            egui::CollapsingHeader::new("Simulation")
                .default_open(false)
                .show(ui, |ui| {
                    ui.add(egui::Slider::new(&mut self.pfc.steady_half_cycles, 2..=60)
                        .text("Steady half-cycles"))
                        .on_hover_text("Half-cycles at nominal load after soft-start");
                    if self.pfc.p_loads_w.len() > 1 {
                        ui.add(egui::Slider::new(&mut self.pfc.step_half_cycles, 4..=60)
                            .text("Step half-cycles"))
                            .on_hover_text("Half-cycles per load step transition");
                    }
                    let total = SOFT_START_HALF_CYCLES + self.pfc.steady_half_cycles
                        + self.pfc.p_loads_w.len().saturating_sub(1) * self.pfc.step_half_cycles;
                    ui.label(egui::RichText::new(format!("Total: {} half-cycles", total)).small());

                    // Plot option
                    ui.separator();
                    ui.label("I_L plot:");
                    ui.horizontal(|ui| {
                        ui.radio_value(&mut self.pfc.pfc_plot_option, PlotOption::Average, "Avg");
                        ui.radio_value(&mut self.pfc.pfc_plot_option, PlotOption::MinAndMax, "Min/Max");
                        ui.radio_value(&mut self.pfc.pfc_plot_option, PlotOption::Waveform, "Waveform");
                    });
                });

            // ── Metrics ──────────────────────────────────────────
            ui.separator();
            if let Some(m) = &self.pfc.metrics {
                ui.label(egui::RichText::new("PFC Metrics").strong());
                ui.label(format!("V_out avg: {:.1} V", m.v_out_avg));
                ui.label(format!("V_out ripple: {:.2} V pk-pk", m.v_out_ripple_v));
                ui.label(format!("THD: {:.1}%", m.thd_pct));
                ui.label(format!("Power factor: {:.3}", m.power_factor));
                ui.label(format!("Efficiency: {:.1}%", m.efficiency_pct));

                // Loss breakdown
                if m.p_loss_total_w > 0.0 {
                    ui.add_space(4.0);
                    ui.label(egui::RichText::new("Conduction Losses").strong());
                    ui.label(format!("FET:   {:.2} W", m.p_loss_fet_w));
                    ui.label(format!("Diode: {:.2} W", m.p_loss_diode_w));
                    ui.label(format!("DCR:   {:.2} W", m.p_loss_dcr_w));
                    ui.label(format!("ESR:   {:.2} W", m.p_loss_esr_w));
                    ui.label(format!("Total: {:.2} W", m.p_loss_total_w));
                }
            }

            ui.separator();
            ui.label(
                egui::RichText::new("Plot updates live as sliders move.")
                    .small()
                    .color(Color32::GRAY),
            );
            ui.add_space(8.0);
            if ui.button("Reset PFC defaults").clicked() {
                self.pfc = PfcState::new();
            }
        }); // ScrollArea
    }

    fn show_pfc_central(&mut self, ui: &mut egui::Ui) {
        // Re-run PFC simulation when params changed
        let params = self.pfc.current_params();
        if params != self.pfc.last_params {
            match run_pfc_simulation(&params) {
                Ok((points, metrics, ds)) => {
                    let r_load = params.v_out * params.v_out / params.p_load_w();
                    self.pfc.bode_data = Some(PfcBodeData::compute(&ds, r_load));
                    self.pfc.metrics = Some(metrics);
                    self.pfc.sim_data = Ok(points);
                }
                Err(e) => {
                    self.pfc.sim_data = Err(e);
                    self.pfc.metrics = None;
                    self.pfc.bode_data = None;
                }
            }
            self.pfc.vin_sweep = None; // invalidate sweep cache
            self.pfc.last_params = params;
        }

        // Redirect away from CurrentLoopBode if mode is not ACM
        if self.pfc.tab == PfcTab::CurrentLoopBode && self.pfc.control_mode != PfcControlMode::Acm {
            self.pfc.tab = PfcTab::Simulation;
        }

        // Tab bar
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.pfc.tab, PfcTab::Simulation, "Simulation");
            if self.pfc.control_mode == PfcControlMode::Acm {
                ui.selectable_value(&mut self.pfc.tab, PfcTab::CurrentLoopBode, "Current Loop");
            }
            ui.selectable_value(&mut self.pfc.tab, PfcTab::VoltageLoopBode, "Voltage Loop");
            ui.selectable_value(&mut self.pfc.tab, PfcTab::Harmonics, "Harmonics");
            ui.selectable_value(&mut self.pfc.tab, PfcTab::VinSweep, "Vin Sweep");
        });
        ui.separator();

        match self.pfc.tab {
            PfcTab::Simulation => self.show_pfc_simulation(ui),
            PfcTab::CurrentLoopBode => {
                match &self.pfc.bode_data {
                    Some(data) => show_pfc_bode(ui, data, true),
                    None => {
                        ui.colored_label(Color32::from_rgb(255, 100, 100),
                            "Invalid parameters for Bode computation.");
                    }
                }
            }
            PfcTab::VoltageLoopBode => {
                match &self.pfc.bode_data {
                    Some(data) => show_pfc_bode(ui, data, false),
                    None => {
                        ui.colored_label(Color32::from_rgb(255, 100, 100),
                            "Invalid parameters for Bode computation.");
                    }
                }
            }
            PfcTab::Harmonics => self.show_pfc_harmonics(ui),
            PfcTab::VinSweep => self.show_pfc_vin_sweep(ui),
        }
    }

    fn show_pfc_simulation(&self, ui: &mut egui::Ui) {
        match &self.pfc.sim_data {
            Err(reason) => {
                ui.centered_and_justified(|ui| {
                    ui.colored_label(Color32::from_rgb(255, 100, 100), reason);
                });
            }
            Ok(data) if data.is_empty() => {
                ui.label("No data.");
            }
            Ok(data) => {
                let total_h = ui.available_height();
                let plot_h = ((total_h / 4.0) - 4.0).max(60.0);

                let x_link = egui::Vec2b::new(true, false);

                let phase_colors = [
                    Color32::from_rgb(100, 180, 255),
                    Color32::from_rgb(255, 130, 130),
                    Color32::from_rgb(130, 255, 130),
                    Color32::from_rgb(255, 200, 100),
                ];

                // ── V_out plot ────────────────────────────────────
                let v_out_pts: Vec<[f64; 2]> = data.iter()
                    .map(|p| [p.t_us as f64 / 1000.0, p.v_out as f64]).collect();
                Plot::new("pfc_vout")
                    .height(plot_h)
                    .y_axis_label("V_out [V]")
                    .link_axis("pfc_time", x_link)
                    .show(ui, |plot_ui| {
                        plot_ui.line(Line::new("V_out", PlotPoints::new(v_out_pts))
                            .color(Color32::GREEN));
                    });

                // ── I_L plot (with i_ref overlay) ─────────────────
                let i_ref_pts: Vec<[f64; 2]> = data.iter()
                    .map(|p| [p.t_us as f64 / 1000.0, p.i_ref as f64]).collect();
                Plot::new("pfc_il")
                    .height(plot_h)
                    .y_axis_label("I_L [A]")
                    .link_axis("pfc_time", x_link)
                    .show(ui, |plot_ui| {
                        match &self.pfc.pfc_plot_option {
                            PlotOption::Average => {
                                let i_avg_pts: Vec<[f64; 2]> = data.iter()
                                    .map(|p| [p.t_us as f64 / 1000.0, p.i_total_avg as f64]).collect();
                                plot_ui.line(Line::new("I_L avg", PlotPoints::new(i_avg_pts))
                                    .color(phase_colors[0]));
                            }
                            PlotOption::MinAndMax | PlotOption::Min | PlotOption::Max => {
                                let i_min_pts: Vec<[f64; 2]> = data.iter()
                                    .map(|p| [p.t_us as f64 / 1000.0, p.i_total_min as f64]).collect();
                                let i_max_pts: Vec<[f64; 2]> = data.iter()
                                    .map(|p| [p.t_us as f64 / 1000.0, p.i_total_max as f64]).collect();
                                plot_ui.line(Line::new("I_total min", PlotPoints::new(i_min_pts))
                                    .color(Color32::from_rgb(100, 130, 200)));
                                plot_ui.line(Line::new("I_total max", PlotPoints::new(i_max_pts))
                                    .color(phase_colors[0]));
                            }
                            PlotOption::Waveform => {
                                let np = self.pfc.num_phases.max(1);
                                for phase_k in 0..np {
                                    let wf = self.pfc.pfc_phase_waveform(data, phase_k);
                                    let name = if np == 1 { "I_L".to_string() }
                                        else { format!("I_L ph{}", phase_k) };
                                    plot_ui.line(Line::new(name, PlotPoints::new(wf))
                                        .color(phase_colors[phase_k % phase_colors.len()]));
                                }
                            }
                        }
                        plot_ui.line(Line::new("I_ref", PlotPoints::new(i_ref_pts))
                            .color(Color32::from_rgb(255, 180, 100))
                            .style(egui_plot::LineStyle::dashed_dense()));
                    });

                // ── Duty plot ─────────────────────────────────────
                let duty_pts: Vec<[f64; 2]> = data.iter()
                    .map(|p| [p.t_us as f64 / 1000.0, p.duty_avg() as f64 * 100.0]).collect();
                Plot::new("pfc_duty")
                    .height(plot_h)
                    .y_axis_label("Duty [%]")
                    .link_axis("pfc_time", x_link)
                    .show(ui, |plot_ui| {
                        plot_ui.line(Line::new("Duty", PlotPoints::new(duty_pts))
                            .color(Color32::from_rgb(255, 255, 100)));
                    });

                // ── V_in_rect plot ────────────────────────────────
                let vin_pts: Vec<[f64; 2]> = data.iter()
                    .map(|p| [p.t_us as f64 / 1000.0, p.v_in_rect as f64]).collect();
                Plot::new("pfc_vin")
                    .height(plot_h)
                    .y_axis_label("V_in [V]")
                    .x_axis_label("Time [ms]")
                    .link_axis("pfc_time", x_link)
                    .show(ui, |plot_ui| {
                        plot_ui.line(Line::new("V_in rect", PlotPoints::new(vin_pts))
                            .color(Color32::from_rgb(180, 180, 180)));
                    });
            }
        }
    }

    fn show_pfc_harmonics(&mut self, ui: &mut egui::Ui) {
        let metrics = match &self.pfc.metrics {
            Some(m) if !m.harmonics_pct.is_empty() => m,
            _ => {
                ui.label("No harmonic data available.");
                return;
            }
        };

        let max_display = 40.min(metrics.harmonics_pct.len() - 1);
        let bars: Vec<egui_plot::Bar> = (1..=max_display)
            .map(|k| {
                egui_plot::Bar::new(k as f64, metrics.harmonics_pct[k])
                    .width(0.7)
            })
            .collect();

        // IEC class selector
        ui.horizontal(|ui| {
            ui.label("IEC 61000-3-2:");
            ui.radio_value(&mut self.pfc.iec_class, IecClass::A, "Class A")
                .on_hover_text("Balanced 3-phase equipment, household appliances, and everything not in B/C/D. Absolute mA limits.");
            ui.radio_value(&mut self.pfc.iec_class, IecClass::B, "Class B")
                .on_hover_text("Portable tools and non-professional arc welding equipment. 1.5x Class A limits.");
            ui.radio_value(&mut self.pfc.iec_class, IecClass::C, "Class C")
                .on_hover_text("Lighting equipment >25W. Limits as % of fundamental, 3rd harmonic scales with power factor.");
            ui.radio_value(&mut self.pfc.iec_class, IecClass::D, "Class D")
                .on_hover_text("Equipment 75-600W with special current waveshape (peaky). PCs, TVs, monitors. Limits in mA/W with absolute caps.");
        });

        // IEC 61000-3-2 limits (% of fundamental)
        let (iec_limits, iec_label) = match self.pfc.iec_class {
            IecClass::A => {
                // Class A: absolute mA limits, convert to % of fundamental.
                // I_1 = P / (V_rms × PF) for the fundamental.
                let i_fund_ma = if metrics.harmonics_pct.len() > 1 {
                    // harmonics_pct[1] == 100% by definition; we need absolute I_1.
                    // P = V_out × I_load, I_in_rms ≈ P / (V_in_rms × PF)
                    // I_1 ≈ I_in_rms (fundamental dominates)
                    let p_w = self.pfc.p_loads_w.first().copied().unwrap_or(300.0);
                    let i_fund = p_w / (self.pfc.v_in_rms * metrics.power_factor.max(0.01));
                    i_fund * 1000.0 // mA
                } else {
                    1000.0
                };
                // Class A absolute limits [mA], convert to % of fundamental
                let ma_to_pct = |ma: f64| ma / i_fund_ma * 100.0;
                let mut limits = Vec::new();
                // Odd harmonics
                limits.push((3, ma_to_pct(2300.0)));
                limits.push((5, ma_to_pct(1140.0)));
                limits.push((7, ma_to_pct(770.0)));
                limits.push((9, ma_to_pct(400.0)));
                limits.push((11, ma_to_pct(330.0)));
                limits.push((13, ma_to_pct(210.0)));
                for k in (15..=39).step_by(2) {
                    limits.push((k, ma_to_pct(150.0 * 15.0 / k as f64)));
                }
                // Even harmonics
                limits.push((2, ma_to_pct(1080.0)));
                limits.push((4, ma_to_pct(430.0)));
                limits.push((6, ma_to_pct(300.0)));
                for k in (8..=40).step_by(2) {
                    limits.push((k, ma_to_pct(230.0 * 8.0 / k as f64)));
                }
                limits.sort_by_key(|(k, _)| *k);
                (limits, "IEC 61000-3-2 Class A")
            }
            IecClass::B => {
                // Class B = 1.5 × Class A absolute limits
                let i_fund_ma = if metrics.harmonics_pct.len() > 1 {
                    let p_w = self.pfc.p_loads_w.first().copied().unwrap_or(300.0);
                    let i_fund = p_w / (self.pfc.v_in_rms * metrics.power_factor.max(0.01));
                    i_fund * 1000.0
                } else {
                    1000.0
                };
                let ma_to_pct = |ma: f64| ma / i_fund_ma * 100.0;
                let mut limits = Vec::new();
                limits.push((3, ma_to_pct(3450.0)));
                limits.push((5, ma_to_pct(1710.0)));
                limits.push((7, ma_to_pct(1155.0)));
                limits.push((9, ma_to_pct(600.0)));
                limits.push((11, ma_to_pct(495.0)));
                limits.push((13, ma_to_pct(315.0)));
                for k in (15..=39).step_by(2) {
                    limits.push((k, ma_to_pct(225.0 * 15.0 / k as f64)));
                }
                limits.push((2, ma_to_pct(1620.0)));
                limits.push((4, ma_to_pct(645.0)));
                limits.push((6, ma_to_pct(450.0)));
                for k in (8..=40).step_by(2) {
                    limits.push((k, ma_to_pct(345.0 * 8.0 / k as f64)));
                }
                limits.sort_by_key(|(k, _)| *k);
                (limits, "IEC 61000-3-2 Class B")
            }
            IecClass::C => {
                let pf_pct = metrics.power_factor * 100.0;
                let mut limits = Vec::new();
                limits.push((2, 2.0));
                limits.push((3, 30.0 * pf_pct / 100.0));
                limits.push((5, 10.0));
                limits.push((7, 7.0));
                limits.push((9, 5.0));
                for k in (11..=39).step_by(2) {
                    limits.push((k, 3.0));
                }
                (limits, "IEC 61000-3-2 Class C")
            }
            IecClass::D => {
                // Class D: mA/W limits (equipment ≤600W), convert to % of fundamental
                let p_w = self.pfc.p_loads_w.first().copied().unwrap_or(300.0);
                let i_fund_ma = if metrics.harmonics_pct.len() > 1 {
                    let i_fund = p_w / (self.pfc.v_in_rms * metrics.power_factor.max(0.01));
                    i_fund * 1000.0
                } else {
                    1000.0
                };
                // Limits are mA/W × P_w, capped at absolute max mA
                let maw_to_pct = |maw: f64, max_ma: f64| {
                    let limit_ma = (maw * p_w).min(max_ma);
                    limit_ma / i_fund_ma * 100.0
                };
                let mut limits = Vec::new();
                limits.push((3, maw_to_pct(3.4, 1140.0)));
                limits.push((5, maw_to_pct(1.9, 570.0)));
                limits.push((7, maw_to_pct(1.0, 300.0)));
                limits.push((9, maw_to_pct(0.5, 150.0)));
                limits.push((11, maw_to_pct(0.35, 115.0)));
                for k in (13..=39).step_by(2) {
                    limits.push((k, maw_to_pct(3.85 / k as f64, 150.0 * 15.0 / k as f64)));
                }
                limits.sort_by_key(|(k, _)| *k);
                (limits, "IEC 61000-3-2 Class D")
            }
        };

        Plot::new("pfc_harmonics")
            .height(ui.available_height())
            .y_axis_label("% of fundamental")
            .x_axis_label("Harmonic number")
            .show(ui, |plot_ui| {
                plot_ui.bar_chart(
                    egui_plot::BarChart::new("Harmonics", bars)
                        .color(Color32::from_rgb(100, 180, 255))
                );

                let iec_pts: Vec<[f64; 2]> = iec_limits.iter()
                    .map(|(k, lim)| [*k as f64, *lim])
                    .collect();
                plot_ui.line(Line::new(iec_label, PlotPoints::new(iec_pts))
                    .color(Color32::from_rgb(255, 80, 80))
                    .style(egui_plot::LineStyle::dashed_dense()));
            });
    }

    fn show_pfc_vin_sweep(&mut self, ui: &mut egui::Ui) {
        if ui.button("Run Vin Sweep (85V - 265V)").clicked() {
            let params = self.pfc.current_params();
            let v_in_points: Vec<f64> = (0..19).map(|i| 85.0 + i as f64 * 10.0).collect();
            let results = run_pfc_vin_sweep(&params, &v_in_points);
            let sweep: Vec<(f64, PfcMetrics)> = results.into_iter()
                .filter_map(|(v, r)| r.ok().map(|m| (v, m)))
                .collect();
            self.pfc.vin_sweep = Some(sweep);
        }

        let sweep = match &self.pfc.vin_sweep {
            Some(s) if !s.is_empty() => s,
            _ => {
                ui.label("Click the button above to run a Vin sweep.");
                return;
            }
        };

        let total_h = ui.available_height();
        let plot_h = ((total_h / 4.0) - 4.0).max(60.0);
        let x_link = egui::Vec2b::new(true, false);

        // THD vs Vin
        let thd_pts: Vec<[f64; 2]> = sweep.iter()
            .map(|(v, m)| [*v, m.thd_pct]).collect();
        Plot::new("pfc_sweep_thd")
            .height(plot_h)
            .y_axis_label("THD [%]")
            .link_axis("pfc_sweep_x", x_link)
            .show(ui, |plot_ui| {
                plot_ui.line(Line::new("THD", PlotPoints::new(thd_pts))
                    .color(Color32::from_rgb(255, 100, 100)));
            });

        // PF vs Vin
        let pf_pts: Vec<[f64; 2]> = sweep.iter()
            .map(|(v, m)| [*v, m.power_factor]).collect();
        Plot::new("pfc_sweep_pf")
            .height(plot_h)
            .y_axis_label("Power Factor")
            .link_axis("pfc_sweep_x", x_link)
            .show(ui, |plot_ui| {
                plot_ui.line(Line::new("PF", PlotPoints::new(pf_pts))
                    .color(Color32::from_rgb(100, 255, 100)));
            });

        // Efficiency vs Vin
        let eff_pts: Vec<[f64; 2]> = sweep.iter()
            .map(|(v, m)| [*v, m.efficiency_pct]).collect();
        Plot::new("pfc_sweep_eff")
            .height(plot_h)
            .y_axis_label("Efficiency [%]")
            .link_axis("pfc_sweep_x", x_link)
            .show(ui, |plot_ui| {
                plot_ui.line(Line::new("Efficiency", PlotPoints::new(eff_pts))
                    .color(Color32::from_rgb(100, 180, 255)));
            });

        // Ripple vs Vin
        let rip_pts: Vec<[f64; 2]> = sweep.iter()
            .map(|(v, m)| [*v, m.v_out_ripple_v]).collect();
        Plot::new("pfc_sweep_ripple")
            .height(plot_h)
            .y_axis_label("V_out ripple [V]")
            .x_axis_label("V_in RMS [V]")
            .link_axis("pfc_sweep_x", x_link)
            .show(ui, |plot_ui| {
                plot_ui.line(Line::new("Ripple", PlotPoints::new(rip_pts))
                    .color(Color32::from_rgb(255, 255, 100)));
            });
    }

    fn show_simulation(&self, ui: &mut egui::Ui) {
        match &self.sim_data {
            Err(reason) => {
                ui.centered_and_justified(|ui| {
                    ui.colored_label(
                        Color32::from_rgb(255, 100, 100),
                        reason,
                    );
                });
            }
            Ok(data) => {
                // Allocate height: 3 signal plots share most space, load plot gets 15%
                let total_h = ui.available_height();
                let load_plot_h = (total_h * 0.15).max(50.0);
                let plot_h = ((total_h - load_plot_h) / 3.0 - PLOT_SPACING).max(80.0);

                let v_out_line = Line::new(
                    "V_out [V]",
                    data.iter()
                        .map(|p| [p.t_ms as f64, p.v_out as f64])
                        .collect::<PlotPoints>(),
                );
                let duty_line = Line::new(
                    "Duty [%]",
                    data.iter()
                        .map(|p| [p.t_ms as f64, p.duty_pct_avg() as f64])
                        .collect::<PlotPoints>(),
                );

                let selected_points = self.select_points(data);

                let target = self.v_out_target;
                let t_end_ms = data.last().map(|p| p.t_ms as f64).unwrap_or(0.0);

                // Bottom plot: load profile (Steps) or battery OCV (Battery)
                let (bottom_y_label, bottom_line) = match self.load_kind {
                    LoadKind::Steps => {
                        let step_ms = 2000.0 / self.f_sw_khz;
                        let mut pts = vec![[0.0_f64, self.r_loads[0]]];
                        let mut t = 3000.0 / self.f_sw_khz;
                        for i in 1..self.r_loads.len() {
                            pts.push([t, self.r_loads[i - 1]]);
                            pts.push([t, self.r_loads[i]]);
                            t += step_ms;
                        }
                        pts.push([t_end_ms, *self.r_loads.last().unwrap()]);
                        ("R_load [Ω]", Line::new("R_load [Ω]", PlotPoints::new(pts)))
                    }
                    LoadKind::Battery => {
                        let pts: PlotPoints = data
                            .iter()
                            .filter(|p| p.v_bat > 0.0)
                            .map(|p| [p.t_ms as f64, p.v_bat as f64])
                            .collect();
                        ("V_bat OCV [V]", Line::new("V_bat OCV [V]", pts))
                    }
                };

                let x_link = egui::Vec2b::new(true, false);

                // V_out plot
                Plot::new("v_out")
                    .height(plot_h)
                    .y_axis_label("V_out [V]")
                    .x_axis_label("")
                    .link_axis("time_axis", x_link)
                    .show(ui, |plot_ui| {
                        plot_ui.line(v_out_line);
                        plot_ui.hline(
                            HLine::new("Target", target)
                                .color(Color32::from_rgb(255, 80, 80))
                                .style(egui_plot::LineStyle::dashed_dense()),
                        );
                        if self.c_in_uf > 0.0 {
                            plot_ui.line(
                                Line::new(
                                    "V_in_cap [V]",
                                    data.iter()
                                        .map(|p| [p.t_ms as f64, p.v_in_cap as f64])
                                        .collect::<PlotPoints>(),
                                ).color(Color32::from_rgb(255, 160, 50)),
                            );
                        }
                    });

                // Duty cycle plot
                Plot::new("duty")
                    .height(plot_h)
                    .y_axis_label("Duty [%]")
                    .x_axis_label("")
                    .link_axis("time_axis", x_link)
                    .show(ui, |plot_ui| {
                        plot_ui.line(duty_line);
                    });

                let num_phases = self.num_phases.max(1);
                let phase_colors = [
                    Color32::from_rgb(80, 140, 255),   // blue
                    Color32::from_rgb(255, 80, 80),    // red
                    Color32::from_rgb(80, 200, 120),   // green
                    Color32::from_rgb(255, 160, 50),   // orange
                    Color32::from_rgb(180, 100, 255),  // purple
                    Color32::from_rgb(50, 200, 200),   // cyan
                ];

                // Inductor current plot
                Plot::new("i_l")
                    .height(plot_h)
                    .y_axis_label("I_L [A]")
                    .x_axis_label("")
                    .link_axis("time_axis", x_link)
                    .show(ui, |plot_ui| {
                        match self.plot_option {
                            PlotOption::Waveform => {
                                for k in 0..num_phases {
                                    let pts = self.phase_waveform(data, k);
                                    plot_ui.line(
                                        Line::new(
                                            if num_phases > 1 {
                                                format!("Phase {} [A]", k + 1)
                                            } else {
                                                "I_L [A]".into()
                                            },
                                            PlotPoints::new(pts),
                                        )
                                        .color(phase_colors[k % phase_colors.len()]),
                                    );
                                }
                            }
                            PlotOption::MinAndMax => {
                                let min_pts: PlotPoints = data.iter()
                                    .map(|p| [p.t_ms as f64, p.i_total_min as f64])
                                    .collect();
                                let max_pts: PlotPoints = data.iter()
                                    .map(|p| [p.t_ms as f64, p.i_total_max as f64])
                                    .collect();
                                plot_ui.line(Line::new("I_min [A]", min_pts));
                                plot_ui.line(Line::new("I_max [A]", max_pts));
                            }
                            _ => {
                                plot_ui.line(Line::new("I_L [A]", PlotPoints::new(selected_points.clone())));
                            }
                        }
                    });

                // Load / battery plot
                Plot::new("bottom")
                    .height(load_plot_h)
                    .y_axis_label(bottom_y_label)
                    .x_axis_label("t [ms]")
                    .link_axis("time_axis", x_link)
                    .show(ui, |plot_ui| {
                        plot_ui.line(bottom_line);
                    });
            }
        }
    }
}
