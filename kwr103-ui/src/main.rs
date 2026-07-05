//! KWR103 Power Supply GUI - Desktop Application

use std::cell::RefCell;
use std::rc::Rc;
use eframe::NativeOptions;

use kwr103_ui::{PowerSupplyUi, ConnectionType, PowerSupplyControl, MeasurementHistory, LimitMode, infer_limit_mode, apply_sequence_step, SequenceUpdate};

// Desktop app needs interior mutability since Kwr103 is !Send + !Sync
type SupplyRef = Rc<RefCell<Option<kwr103_ui::RealPowerSupply>>>;

struct PowerSupplyApp {
    ui_state: PowerSupplyUi,
    supply: SupplyRef,
    connection_type: ConnectionType,
    ip_address: String,
    last_state: Option<kwr103_ui::PowerSupplyState>,
    history: MeasurementHistory,
    start_time: std::time::Instant,
    // Sequence playback state
    sequence_playing: bool,
    sequence_repeat: bool,
    sequence_last_update: std::time::Instant,
}

impl PowerSupplyApp {
    fn new() -> Self {
        Self {
            ui_state: PowerSupplyUi::new(),
            supply: Rc::new(RefCell::new(None)),
            connection_type: ConnectionType::Usb,
            ip_address: "192.168.1.100".to_string(),
            last_state: None,
            history: MeasurementHistory::new(60.0),
            start_time: std::time::Instant::now(),
            sequence_playing: false,
            sequence_repeat: false,
            sequence_last_update: std::time::Instant::now(),
        }
    }

    fn connect(&mut self) -> Result<(), String> {
        // For Ethernet, use the IP the user typed into the text field rather
        // than the placeholder baked into the radio button's ConnectionType.
        let conn = match &self.connection_type {
            ConnectionType::Eth { .. } => {
                let ip = self.ip_address.trim();
                let ip = if ip.is_empty() { "192.168.1.100" } else { ip };
                ConnectionType::Eth { ip: ip.to_string() }
            }
            other => other.clone(),
        };
        let supply = kwr103_ui::RealPowerSupply::new(conn)?;
        *self.supply.borrow_mut() = Some(supply);
        Ok(())
    }

    fn disconnect(&mut self) {
        *self.supply.borrow_mut() = None;
    }
}

impl Default for PowerSupplyApp {
    fn default() -> Self {
        Self::new()
    }
}

impl eframe::App for PowerSupplyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // ── Connection panel ──────────────────────────────────────────────────
        egui::TopBottomPanel::top("connection_panel").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("Connect to:");
                ui.radio_value(&mut self.connection_type, ConnectionType::Usb, "USB");
                ui.radio_value(&mut self.connection_type, ConnectionType::Eth {
                    ip: "192.168.1.100".to_string(),
                }, "Ethernet");

                if matches!(self.connection_type, ConnectionType::Eth { .. }) {
                    ui.add(
                        egui::TextEdit::singleline(&mut self.ip_address)
                            .hint_text("IP address"),
                    );
                }

                if ui.button("Connect").clicked() {
                    if self.supply.borrow().is_none() {
                        if let Err(e) = self.connect() {
                            eprintln!("Connection failed: {}", e);
                        }
                    }
                }

                if self.supply.borrow().is_some() {
                    if ui.button("Disconnect").clicked() {
                        self.disconnect();
                    }
                }
            });
        });

        // ── Main UI layout ───────────────────────────────────────────────────
        egui::CentralPanel::default().show(ctx, |ui| {
            if self.supply.borrow().is_none() {
                ui.centered_and_justified(|ui| {
                    ui.colored_label(
                        egui::Color32::YELLOW,
                        "Connect to a KWR103 power supply to begin.",
                    );
                });
                return;
            }

            // ── Status display ──────────────────────────────────────────────
            ui.collapsing("Device Status", |ui| {
                if let Some(state) = self.last_state {
                    ui.horizontal(|ui| {
                        ui.label("Output:");
                        let color = if state.output_on { egui::Color32::GREEN } else { egui::Color32::RED };
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

            // ── Plot ────────────────────────────────────────────────────────
            kwr103_ui::show_measurement_plot(&self.history, ui);

            ui.separator();

            // ── Voltage/Current controls ────────────────────────────────────
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
                        if let Some(s) = self.supply.borrow_mut().as_mut() {
                            let _ = s.set_output(self.ui_state.output_enabled);
                        }
                    }
                    if ui.button("Apply Settings").clicked() {
                        if let Some(s) = self.supply.borrow_mut().as_mut() {
                            let _ = s.set_voltage(self.ui_state.target_voltage);
                            let _ = s.set_current(self.ui_state.target_current);
                        }
                    }
                });
            });

            ui.separator();

            // ── Sequence editor ─────────────────────────────────────────────
            ui.collapsing("Voltage Sequence", |ui| {
                self.show_sequence_editor(ui);
            });
        });

        // ── Sequence playback ───────────────────────────────────────────────
        if self.sequence_playing && !self.ui_state.sequence.is_empty() {
            let elapsed = self.sequence_last_update.elapsed();
            self.sequence_last_update = std::time::Instant::now();

            let delta_ms = elapsed.as_millis() as u32;

            match self.ui_state.sequence.update(delta_ms) {
                SequenceUpdate::Step(step) => {
                    if let Some(supply) = self.supply.borrow_mut().as_mut() {
                        let _ = apply_sequence_step(&mut *supply, step);
                    }
                }
                SequenceUpdate::Unchanged => {}
                SequenceUpdate::Finished => {
                    if self.sequence_repeat {
                        self.ui_state.sequence.reset();
                    } else {
                        self.sequence_playing = false;
                    }
                }
            }
        }

        // ── Auto-refresh state ─────────────────────────────────────────────
        static REFRESH_TICK: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let tick = REFRESH_TICK.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if tick % 10 == 0 {
            if let Some(s) = self.supply.borrow_mut().as_mut() {
                let _ = s.refresh();
                self.last_state = s.get_state();
            }
            if let Some(state) = self.last_state {
                let t = self.start_time.elapsed().as_secs_f64();
                self.history.push(t, state.voltage, state.current);
            }
        }

        ctx.request_repaint_after(std::time::Duration::from_millis(10));
    }
}

impl PowerSupplyApp {
    fn show_sequence_editor(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.button("+ Add Step").clicked() {
                let step = kwr103_ui::SequenceStep {
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
                self.sequence_last_update = std::time::Instant::now();
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
        let num_steps = self.ui_state.sequence.steps().len();
        let mut to_remove: Option<usize> = None;
        for i in 0..num_steps {
            ui.horizontal(|ui| {
                ui.label(format!("Step {}:", i + 1));

                // Read current values first
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
                    // Defer removal: calling remove_step(i) here and `return`ing
                    // only exits this closure, not the `for` loop, so the next
                    // iteration would index the shrunk Vec out of bounds and panic.
                    to_remove = Some(i);
                }
            });
        }
        if let Some(i) = to_remove {
            self.ui_state.sequence.remove_step(i);
        }

        // Show playback progress
        if !self.ui_state.sequence.is_empty() {
            let _total_duration: u32 = self.ui_state.sequence.steps().iter()
                .map(|s| s.duration_ms).sum();

            // Get current step data first to avoid borrow issues
            let (current_voltage, current_current) = if let Some(step) = self.ui_state.sequence.current_step() {
                (step.voltage, step.current_limit)
            } else {
                (0.0, 0.0)
            };

            ui.horizontal(|ui| {
                ui.label("Current step:");
                ui.monospace(format!("{:.1} V / {:.2} A", current_voltage, current_current));
            });

            let progress = self.ui_state.sequence.get_playback_progress();
            ui.add(egui::ProgressBar::new(progress).text("Sequence Progress"));
        }
    }
}

fn main() -> Result<(), eframe::Error> {
    let app = PowerSupplyApp::new();
    eframe::run_native(
        "KWR103 Power Supply Control",
        NativeOptions::default(),
        Box::new(|_cc| Ok(Box::new(app))),
    )
}