//! KWR103 Power Supply GUI - Web Application
//!
//! This module provides a web-based frontend using egui wasm.
//! Since browsers cannot directly access USB/Ethernet devices, this
//! provides a demonstration mode with simulated power supply behavior.

use eframe::WebOptions;
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;

use crate::{PowerSupplyUi, MockPowerSupply, PowerSupplyControl, PowerSupplyState, MeasurementHistory, LimitMode, infer_limit_mode, apply_sequence_step};

fn now_ms() -> f64 {
    web_sys::window()
        .expect("no window")
        .performance()
        .expect("no performance")
        .now()
}

/// Web application entry point
#[wasm_bindgen(start)]
pub fn run() {
    let web_options = WebOptions::default();

    let canvas = web_sys::window()
        .unwrap()
        .document()
        .unwrap()
        .get_element_by_id("kwr103_canvas")
        .unwrap()
        .dyn_into::<web_sys::HtmlCanvasElement>()
        .unwrap();

    wasm_bindgen_futures::spawn_local(async move {
        eframe::WebRunner::new()
            .start(
                canvas,
                web_options,
                Box::new(|cc| Ok(Box::new(WebApp::new(cc)))),
            )
            .await
            .expect("failed to start eframe");
    });
}

struct WebApp {
    ui_state: PowerSupplyUi,
    supply: MockPowerSupply,
    last_state: Option<PowerSupplyState>,
    history: MeasurementHistory,
    start_ms: f64,
    last_refresh_ms: f64,
    sequence_playing: bool,
    sequence_repeat: bool,
    sequence_last_update_ms: f64,
}

impl WebApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let now = now_ms();
        Self {
            ui_state: PowerSupplyUi::new(),
            supply: MockPowerSupply::new(),
            last_state: None,
            history: MeasurementHistory::new(60.0),
            start_ms: now,
            last_refresh_ms: now,
            sequence_playing: false,
            sequence_repeat: false,
            sequence_last_update_ms: now,
        }
    }

    fn show_sequence_editor(&mut self, ui: &mut egui::Ui) {
        use crate::SequenceStep;

        ui.horizontal(|ui| {
            if ui.button("+ Add Step").clicked() {
                let step = SequenceStep {
                    voltage: self.ui_state.target_voltage,
                    current_limit: self.ui_state.target_current,
                    duration_ms: 1000,
                    output_on: true,
                };
                self.ui_state.sequence.add_step(step);
            }
            if ui.button("Play").clicked() {
                self.ui_state.sequence.reset();
                self.sequence_playing = true;
                self.sequence_last_update_ms = now_ms();
            }
            if ui.button("Stop").clicked() {
                self.ui_state.sequence.reset();
                self.sequence_playing = false;
            }
            ui.checkbox(&mut self.sequence_repeat, "Repeat");

            // Show playing status
            if self.sequence_playing && !self.ui_state.sequence.is_empty() {
                ui.label("▶ Playing");
            }
        });

        // Show sequence steps
        let num_steps = self.ui_state.sequence.len();
        let mut to_remove = None;
        egui::ScrollArea::vertical().show(ui, |ui| {
            for i in 0..num_steps {
                ui.horizontal(|ui| {
                    ui.label(format!("Step {}:", i + 1));

                    let step = &self.ui_state.sequence.steps()[i];
                    let mut v = step.voltage;
                    let mut c = step.current_limit;
                    let mut d = step.duration_ms;

                    if ui.add(egui::DragValue::new(&mut v).range(0.0..=30.0).suffix(" V")).changed() {
                        self.ui_state.sequence.steps_mut()[i].voltage = v;
                    }
                    if ui.add(egui::DragValue::new(&mut c).range(0.0..=5.0).suffix(" A")).changed() {
                        self.ui_state.sequence.steps_mut()[i].current_limit = c;
                    }
                    if ui.add(egui::DragValue::new(&mut d).range(100..=60000).suffix(" ms")).changed() {
                        self.ui_state.sequence.steps_mut()[i].duration_ms = d;
                    }
                    if ui.button("X").clicked() {
                        to_remove = Some(i);
                    }
                });
            }
        });
        if let Some(i) = to_remove {
            self.ui_state.sequence.remove_step(i);
        }

        // Show playback progress
        if !self.ui_state.sequence.is_empty() {
            let current = self.ui_state.sequence.get_current_step();
            if let Some(step) = current {
                ui.horizontal(|ui| {
                    ui.label("Current:");
                    ui.monospace(format!("{:.1} V / {:.2} A", step.voltage, step.current_limit));
                });
                let progress = self.ui_state.sequence.get_playback_progress();
                ui.add(egui::ProgressBar::new(progress).text("Sequence Progress"));
            }
        }
    }

    fn update_sequence(&mut self, delta_ms: u32) {
        if !self.sequence_playing || self.ui_state.sequence.is_empty() {
            return;
        }

        match self.ui_state.sequence.update(delta_ms) {
            Some(step) => {
                let _ = apply_sequence_step(&mut self.supply, step);
            }
            None => {
                if self.sequence_repeat {
                    self.ui_state.sequence.reset();
                } else {
                    self.sequence_playing = false;
                }
            }
        }
    }
}

impl eframe::App for WebApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // ── Header ───────────────────────────────────────────────────────────
        egui::TopBottomPanel::top("header").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("KWR103 Power Supply Control (Web Demo)");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("Reset").clicked() {
                        self.ui_state = PowerSupplyUi::new();
                    }
                });
            });
        });

        // ── Main content ───────────────────────────────────────────────────
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                ui.heading("Simulation Mode");
                ui.label("This web version runs in simulation mode since browsers");
                ui.label("cannot directly access USB/Ethernet devices.");

                ui.separator();

                // ── Status display ─────────────────────────────────────────
                ui.collapsing("Device Status", |ui| {
                    if let Some(state) = self.last_state {
                        ui.horizontal(|ui| {
                            ui.label("Output:");
                            let color = if state.output_on {
                                egui::Color32::from_rgb(0, 255, 0)
                            } else {
                                egui::Color32::from_rgb(255, 0, 0)
                            };
                            ui.colored_label(color, if state.output_on { "ON" } else { "OFF" });

                            match infer_limit_mode(&state, self.ui_state.target_current) {
                                Some(LimitMode::CV) => {
                                    ui.colored_label(egui::Color32::GREEN, "CV");
                                }
                                Some(LimitMode::CC) => {
                                    ui.colored_label(egui::Color32::from_rgb(255, 165, 0), "CC");
                                }
                                None => {}
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("Voltage: ");
                            ui.monospace(format!("{:.3} V", state.voltage));
                        });
                        ui.horizontal(|ui| {
                            ui.label("Current: ");
                            ui.monospace(format!("{:.3} A", state.current));
                        });
                    } else {
                        ui.label("Waiting for data...");
                    }
                });

                ui.separator();

                // ── Plot ──────────────────────────────────────────────────
                crate::show_measurement_plot(&self.history, ui);

                ui.separator();

                // ── Voltage/Current controls ───────────────────────────────
                ui.collapsing("Settings", |ui| {
                    ui.add(
                        egui::Slider::new(&mut self.ui_state.target_voltage, 0.0..=30.0)
                            .text("Voltage [V]")
                            .step_by(0.1),
                    );
                    ui.add(
                        egui::Slider::new(&mut self.ui_state.target_current, 0.0..=5.0)
                            .text("Current Limit [A]")
                            .step_by(0.05),
                    );
                    ui.add(
                        egui::Slider::new(&mut self.supply.load_resistance, 1.0..=1000.0)
                            .text("Load Resistance [Ω]")
                            .logarithmic(true),
                    );

                    ui.horizontal(|ui| {
                        let btn_text = if self.ui_state.output_enabled { "Turn OFF" } else { "Turn ON" };
                        if ui.button(btn_text).clicked() {
                            self.ui_state.output_enabled = !self.ui_state.output_enabled;
                            let _ = self.supply.set_output(self.ui_state.output_enabled);
                        }

                        if ui.button("Apply Settings").clicked() {
                            let _ = self.supply.set_voltage(self.ui_state.target_voltage);
                            let _ = self.supply.set_current(self.ui_state.target_current);
                        }
                    });
                });

                ui.separator();

                // ── Sequence editor ────────────────────────────────────────
                ui.collapsing("Voltage Sequence", |ui| {
                    self.show_sequence_editor(ui);
                });
            });
        });

        // ── Auto-refresh state ─────────────────────────────────────────────
        let now = now_ms();

        // Update sequence playback
        if self.sequence_playing {
            let delta_ms = (now - self.sequence_last_update_ms) as u32;
            self.update_sequence(delta_ms);
            self.sequence_last_update_ms = now;
        }

        if now - self.last_refresh_ms >= 100.0 {
            self.last_refresh_ms = now;
            let _ = self.supply.refresh();
            self.last_state = self.supply.get_state();
            if let Some(state) = self.last_state {
                let t = (now - self.start_ms) / 1000.0;
                self.history.push(t, state.voltage, state.current);
            }
        }

        ctx.request_repaint();
    }
}
