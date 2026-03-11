//! KWR103 Power Supply GUI - Web Application
//!
//! This module provides a web-based frontend using egui wasm.
//! Since browsers cannot directly access USB/Ethernet devices, this
//! provides a demonstration mode with simulated power supply behavior.

use eframe::WebOptions;
use egui;

use kwr103_ui::{PowerSupplyUi, MockPowerSupply, PowerSupplyControl, PowerSupplyState};

/// Web application entry point
pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    let web_options = WebOptions::default();

    wasm_bindgen_futures::spawn_local(async {
        let error = eframe::WebRunner::new()
            .start(
                "eframe-app",
                web_options,
                Box::new(|cc| Box::new(WebApp::new(cc))),
            )
            .await
            .unwrap_err();

        wasm_bindgen::JsCast::console_error(&error);
    });

    Ok(())
}

struct WebApp {
    ui_state: PowerSupplyUi,
    supply: MockPowerSupply,
    last_state: Option<PowerSupplyState>,
    last_refresh: std::time::Instant,
}

impl WebApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Self {
            ui_state: PowerSupplyUi::new(),
            supply: MockPowerSupply::new(),
            last_state: None,
            last_refresh: std::time::Instant::now(),
        }
    }
}

impl eframe::App for WebApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // ── Header ───────────────────────────────────────────────────────────
        egui::TopPanel::top("header").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("KWR103 Power Supply Control (Web Demo)");
                ui.with_layout(egui::Layout::right_to_left(), |ui| {
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

                        if ui.button("Refresh").clicked() {
                            let _ = self.supply.refresh();
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
        let now = std::time::Instant::now();
        if now.duration_since(self.last_refresh).as_millis() >= 100 {
            self.last_refresh = now;
            let _ = self.supply.refresh();
            self.last_state = self.supply.get_state();
        }
    }
}

impl WebApp {
    fn show_sequence_editor(&mut self, ui: &mut egui::Ui) {
        use kwr103_ui::SequenceStep;

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
            }
            if ui.button("Stop").clicked() {
                self.ui_state.sequence.reset();
            }
        });

        // Show sequence steps
        ui.scroll_area().show(ui, |ui| {
            for (i, step) in self.ui_state.sequence.steps.iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(format!("Step {}:", i + 1));
                    ui.add(
                        egui::DragValue::new(&mut step.voltage)
                            .range(0.0..=30.0)
                            .suffix(" V"),
                    );
                    ui.add(
                        egui::DragValue::new(&mut step.current_limit)
                            .range(0.0..=5.0)
                            .suffix(" A"),
                    );
                    ui.add(
                        egui::DragValue::new(&mut step.duration_ms)
                            .range(100..=60000)
                            .suffix(" ms"),
                    );
                    if ui.button("X").clicked() {
                        self.ui_state.sequence.steps.remove(i);
                    }
                });
            }
        });

        // Show playback progress
        if !self.ui_state.sequence.steps.is_empty() {
            let total_duration: u32 = self.ui_state.sequence.steps.iter()
                .map(|s| s.duration_ms).sum();
            let current = self.ui_state.sequence.get_current_step();
            if let Some(step) = current {
                ui.horizontal(|ui| {
                    ui.label("Current:");
                    ui.monospace(format!("{:.1} V / {:.2} A", step.voltage, step.current_limit));
                });
                let progress = (self.ui_state.sequence.playback_progress as f32 / total_duration as f32).min(1.0);
                ui.add(egui::ProgressBar::new(progress).text("Sequence Progress"));
            }
        }
    }
}