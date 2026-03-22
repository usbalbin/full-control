use egui::Color32;
use egui_plot::{HLine, Line, Plot, PlotPoints};
use full_control::control_2p2z::Topology as ControlTopology;

use crate::bode::{BodeData, show_bode};
use crate::sim::{CurrentConduction, McuProfile, CsProfile, DacProfile, LoadKind, SimParams, SimPoint, build_ctrl_params, run_simulation};

#[derive(Debug, Clone, Copy, PartialEq)]
enum Tab {
    Simulation,
    Bode,
}

pub struct BuckSimApp {
    // ── Slider state (human-friendly units) ────────────────────────────────
    v_in: f64,          // [V]
    v_out_target: f64,  // [V]
    f_sw_khz: f64,      // [kHz]
    l_uh: f64,          // [µH]
    c_out_uf: f64,      // [µF]
    r_esr_mohm: f64,    // [mΩ]
    r_series_mohm: f64, // [mΩ]
    cs_gain_mv_a: f64,  // [mV/A]
    max_current: f64,   // [A]

    // ── Controller tuning ──────────────────────────────────────────────────
    crossover_khz: f64,    // [kHz]
    cycles_per_tick: usize,

    // ── Rectifier configuration ──────────────────────────────────────────────
    current_conduction: CurrentConduction,

    // ── Load configuration ───────────────────────────────────────────────────
    load_kind: LoadKind,
    r_loads: Vec<f64>,      // [Ω] per-phase load resistances (Steps mode)
    bat_v_init: f64,        // [V]   battery initial OCV
    bat_r_int_mohm: f64,    // [mΩ]  battery internal resistance
    bat_c_mf: f64,          // [mF]  battery capacitance (controls charging speed)

    // ── Hardware profiles ─────────────────────────────────────────────────────
    mcu: McuProfile,
    mcu_preset_idx: usize,
    cs: CsProfile,
    cs_preset_idx: usize,
    dac: DacProfile,
    dac_preset_idx: usize,

    // ── Cached simulation output ────────────────────────────────────────────
    sim_data: Option<Vec<SimPoint>>,
    bode_data: Option<BodeData>,
    last_params: SimParams,

    // ── UI state ─────────────────────────────────────────────────────────────
    tab: Tab,
    plot_option: PlotOption,
}

#[derive(Debug, Clone, PartialEq)]
enum PlotOption {
    Min,
    Max,
    Average,
    MinAndMax,
}

impl Default for BuckSimApp {
    fn default() -> Self {
        let defaults = SimParams::default();
        let sim_data = run_simulation(&defaults);
        let bode_data = build_bode(&defaults);
        Self {
            v_in: defaults.v_in,
            v_out_target: defaults.v_out_target,
            f_sw_khz: defaults.f_sw_khz,
            l_uh: defaults.l_uh,
            c_out_uf: defaults.c_out_uf,
            r_esr_mohm: defaults.r_esr_mohm,
            r_series_mohm: defaults.r_series_mohm,
            cs_gain_mv_a: defaults.cs_gain_mv_a,
            max_current: defaults.max_current,
            crossover_khz: defaults.crossover_khz,
            cycles_per_tick: defaults.cycles_per_tick,
            current_conduction: defaults.current_conduction,
            load_kind: defaults.load_kind.clone(),
            r_loads: defaults.r_loads.clone(),
            bat_v_init: defaults.bat_v_init,
            bat_r_int_mohm: defaults.bat_r_int_mohm,
            bat_c_mf: defaults.bat_c_mf,
            mcu: defaults.mcu.clone(),
            mcu_preset_idx: 0,
            cs: defaults.cs.clone(),
            cs_preset_idx: 0,
            dac: defaults.dac.clone(),
            dac_preset_idx: 0,
            sim_data,
            bode_data,
            last_params: defaults,
            tab: Tab::Simulation,
            plot_option: PlotOption::Average,
        }
    }
}

fn build_bode(p: &SimParams) -> Option<BodeData> {
    let ctrl = build_ctrl_params(p)?;
    let (tf, _) = ctrl.to_transfer_function(p.v_in, ControlTopology::Buck);
    let ds = tf.design_summary();
    Some(BodeData::compute(&ds))
}

impl BuckSimApp {
    fn current_params(&self) -> SimParams {
        SimParams {
            v_in: self.v_in,
            v_out_target: self.v_out_target,
            f_sw_khz: self.f_sw_khz,
            l_uh: self.l_uh,
            c_out_uf: self.c_out_uf,
            r_esr_mohm: self.r_esr_mohm,
            r_series_mohm: self.r_series_mohm,
            cs_gain_mv_a: self.cs_gain_mv_a,
            max_current: self.max_current,
            crossover_khz: self.crossover_khz,
            cycles_per_tick: self.cycles_per_tick,
            current_conduction: self.current_conduction,
            load_kind: self.load_kind.clone(),
            r_loads: self.r_loads.clone(),
            bat_v_init: self.bat_v_init,
            bat_r_int_mohm: self.bat_r_int_mohm,
            bat_c_mf: self.bat_c_mf,
            mcu: self.mcu.clone(),
            cs: self.cs.clone(),
            dac: self.dac.clone(),
        }
    }

    fn select_points(&self, data: &[SimPoint]) -> Vec<[f64; 2]> {
        match self.plot_option {
            PlotOption::Min => data
                .iter()
                .map(|p| [p.t_ms as f64, p.i_l_min as f64])
                .collect(),
            PlotOption::Max => data
                .iter()
                .map(|p| [(p.t_ms + p.t_on) as f64, p.i_l_max as f64])
                .collect(),
            PlotOption::Average => data
                .iter()
                .map(|p| [p.t_ms as f64, 0.5 * (p.i_l_min + p.i_l_max) as f64])
                .collect(),
            PlotOption::MinAndMax => data
                .iter()
                .map(|p| {
                    [
                        [p.t_ms as f64, p.i_l_min as f64],
                        [(p.t_ms + p.t_on) as f64, p.i_l_max as f64],
                    ]
                })
                .flatten()
                .collect(),
        }
    }
}

// Spacing between vertically stacked plots
const PLOT_SPACING: f32 = 4.0;

impl eframe::App for BuckSimApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // ── Parameter panel ──────────────────────────────────────────────────
        egui::SidePanel::left("params")
            .min_width(240.0)
            .max_width(320.0)
            .show(ctx, |ui| {
                ui.heading("Buck Converter Parameters");
                ui.separator();

                ui.label("Source");
                ui.add(
                    egui::Slider::new(&mut self.v_in, 5.0..=60.0)
                        .text("V_in [V]")
                        .step_by(0.5),
                );
                // Clamp v_out below v_in
                if self.v_out_target >= self.v_in {
                    self.v_out_target = self.v_in - 0.5;
                }

                ui.separator();
                ui.label("Output");
                let v_out_max = (self.v_in - 0.5).max(0.5);
                ui.add(
                    egui::Slider::new(&mut self.v_out_target, 0.5..=v_out_max)
                        .text("V_out [V]")
                        .step_by(0.1),
                );

                ui.separator();
                ui.label("Switching");
                ui.add(
                    egui::Slider::new(&mut self.f_sw_khz, 50.0..=2000.0)
                        .text("f_sw [kHz]")
                        .step_by(10.0),
                );

                ui.separator();
                ui.label("Passive components");
                ui.add(
                    egui::Slider::new(&mut self.l_uh, 0.5..=500.0)
                        .text("L [µH]")
                        .step_by(0.5),
                );
                ui.add(
                    egui::Slider::new(&mut self.c_out_uf, 1.0..=5000.0)
                        .text("C_out [µF]")
                        .step_by(1.0),
                );
                ui.add(
                    egui::Slider::new(&mut self.r_esr_mohm, 0.0..=500.0)
                        .text("R_ESR [mΩ]")
                        .step_by(0.1),
                );
                ui.add(
                    egui::Slider::new(&mut self.r_series_mohm, 0.0..=500.0)
                        .text("R_series [mΩ]")
                        .step_by(1.0),
                );

                ui.separator();
                ui.label("Current sense");
                ui.add(
                    egui::Slider::new(&mut self.cs_gain_mv_a, 10.0..=500.0)
                        .text("CS gain [mV/A]")
                        .step_by(1.0),
                );
                ui.add(
                    egui::Slider::new(&mut self.max_current, 0.5..=50.0)
                        .text("I_max [A]")
                        .step_by(0.5),
                );

                // ── Controller tuning ───────────────────────────────────────
                ui.separator();
                ui.label("Controller");
                let f_sw_half = self.f_sw_khz / 2.0;
                ui.add(
                    egui::Slider::new(&mut self.crossover_khz, 1.0..=f_sw_half)
                        .text("f_x [kHz]")
                        .logarithmic(true)
                        .max_decimals(1),
                );
                let mut cpt = self.cycles_per_tick as f64;
                ui.add(
                    egui::Slider::new(&mut cpt, 1.0..=8.0)
                        .text("cycles/tick")
                        .step_by(1.0),
                );
                self.cycles_per_tick = cpt as usize;

                // ── Hardware profiles ───────────────────────────────────
                ui.separator();
                egui::CollapsingHeader::new("Hardware")
                    .default_open(false)
                    .show(ui, |ui| {
                        // ── MCU ───────────────────────────────────────────
                        ui.label("MCU");
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
                            );
                            ui.add(
                                egui::Slider::new(&mut self.mcu.t_adc_us, 0.0..=10.0)
                                    .text("t_ADC [us]")
                                    .step_by(0.1),
                            );
                            ui.add(
                                egui::Slider::new(&mut self.mcu.t_processing_us, 0.0..=10.0)
                                    .text("t_proc [us]")
                                    .step_by(0.1),
                            );
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
                        ui.label("Current sensor");
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
                                egui::Slider::new(&mut self.cs.cs_bandwidth_khz, 0.0..=10000.0)
                                    .text("CS BW [kHz]")
                                    .logarithmic(true)
                                    .max_decimals(0),
                            );

                            if self.cs != before && self.cs_preset_idx != usize::MAX {
                                self.cs_preset_idx = usize::MAX;
                            }
                        }

                        ui.separator();

                        // ── Slope DAC ─────────────────────────────────────
                        ui.label("Slope DAC");
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
                            );
                            ui.add(
                                egui::Slider::new(&mut self.dac.t_dac_us, 0.0..=10.0)
                                    .text("t_DAC [us]")
                                    .step_by(0.01),
                            );

                            if self.dac != before && self.dac_preset_idx != usize::MAX {
                                self.dac_preset_idx = usize::MAX;
                            }
                        }
                    });

                ui.separator();
                ui.label("Rectifier");
                ui.horizontal(|ui| {
                    ui.radio_value(
                        &mut self.current_conduction,
                        CurrentConduction::Diode,
                        "Diode (DCM)",
                    );
                    ui.radio_value(
                        &mut self.current_conduction,
                        CurrentConduction::Synchronous,
                        "Synchronous",
                    );
                });

                // ── Load section ─────────────────────────────────────────────
                ui.separator();
                ui.label("Load");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.load_kind, LoadKind::Steps, "Load steps");
                    ui.radio_value(&mut self.load_kind, LoadKind::Battery, "Battery");
                });

                match self.load_kind {
                    LoadKind::Steps => {
                        ui.add_space(2.0);
                        // Nominal load — always present, no remove button
                        ui.add(
                            egui::Slider::new(&mut self.r_loads[0], 0.5..=1000.0)
                                .text("R_nominal [Ω]")
                                .logarithmic(true)
                                .max_decimals(1),
                        );

                        // Additional load steps — each has a remove button
                        let mut to_remove: Option<usize> = None;
                        for i in 1..self.r_loads.len() {
                            let clicked = ui
                                .horizontal(|ui| {
                                    ui.add(
                                        egui::Slider::new(&mut self.r_loads[i], 0.5..=10000.0)
                                            .text(format!("step {} [Ω]", i))
                                            .logarithmic(true)
                                            .max_decimals(1),
                                    );
                                    ui.small_button("−").clicked()
                                })
                                .inner;
                            if clicked {
                                to_remove = Some(i);
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
                        );
                        ui.add(
                            egui::Slider::new(&mut self.bat_r_int_mohm, 5.0..=500.0)
                                .text("R_int [mΩ]")
                                .step_by(1.0),
                        );
                        ui.add(
                            egui::Slider::new(&mut self.bat_c_mf, 1.0..=1000.0)
                                .text("C_bat [mF]")
                                .logarithmic(true)
                                .max_decimals(1),
                        );
                    }
                }

                ui.separator();
                ui.label("Plot Options");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.plot_option, PlotOption::Min, "Min");
                    ui.radio_value(&mut self.plot_option, PlotOption::Max, "Max");
                    ui.radio_value(&mut self.plot_option, PlotOption::Average, "Average");
                    ui.radio_value(&mut self.plot_option, PlotOption::MinAndMax, "Min and Max");
                });

                ui.separator();
                ui.label(
                    egui::RichText::new("Plot updates live as sliders move.")
                        .small()
                        .color(Color32::GRAY),
                );
            });

        // ── Central panel — plots ────────────────────────────────────────────
        egui::CentralPanel::default().show(ctx, |ui| {
            // Re-run simulation / bode only when params changed
            let params = self.current_params();
            if params != self.last_params {
                self.sim_data = run_simulation(&params);
                self.bode_data = build_bode(&params);
                self.last_params = params;
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
        });
    }
}

impl BuckSimApp {
    fn show_simulation(&self, ui: &mut egui::Ui) {
        match &self.sim_data {
            None => {
                ui.centered_and_justified(|ui| {
                    ui.colored_label(
                        Color32::from_rgb(255, 100, 100),
                        "Invalid parameters — V_out must be less than V_in.",
                    );
                });
            }
            Some(data) => {
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
                        .map(|p| [p.t_ms as f64, p.duty_pct as f64])
                        .collect::<PlotPoints>(),
                );

                let selected_points = self.select_points(data);
                let il_line = Line::new("I_L [A]", PlotPoints::new(selected_points));

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

                // Inductor current plot
                Plot::new("i_l")
                    .height(plot_h)
                    .y_axis_label("I_L [A]")
                    .x_axis_label("")
                    .link_axis("time_axis", x_link)
                    .show(ui, |plot_ui| {
                        plot_ui.line(il_line);
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
