//! KWR103 Power Supply GUI
//!
//! This library provides shared logic for controlling a KWR103 programmable
//! power supply via USB or Ethernet.

#[cfg(target_arch = "wasm32")]
pub mod web;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

/// Connection type for the power supply
#[derive(Debug, Clone, PartialEq)]
pub enum ConnectionType {
    Usb,
    Eth { ip: String },
}

/// Current state of the power supply
#[derive(Debug, Clone, Copy)]
pub struct PowerSupplyState {
    pub output_on: bool,
    pub voltage: f32,
    pub current: f32,
}

/// Configuration for voltage/current limits
#[derive(Debug, Clone, Copy)]
pub struct PowerSupplyConfig {
    pub max_voltage: f32,
    pub max_current: f32,
}

impl Default for PowerSupplyConfig {
    fn default() -> Self {
        Self {
            max_voltage: 30.0,
            max_current: 5.0,
        }
    }
}

/// Power supply control trait - desktop version uses RefCell for interior mutability
pub trait PowerSupplyControl {
    fn get_state(&mut self) -> Option<PowerSupplyState>;
    fn set_voltage(&mut self, voltage: f32) -> Result<(), String>;
    fn set_current(&mut self, current: f32) -> Result<(), String>;
    fn set_output(&mut self, enabled: bool) -> Result<(), String>;
    fn refresh(&mut self) -> Result<(), String>;
}

/// Atomic f32 wrapper for use with Arc
struct AtomicF32(AtomicU32);

impl AtomicF32 {
    const fn new(f: f32) -> Self {
        Self(AtomicU32::new(f.to_bits()))
    }

    fn store(&self, f: f32, order: Ordering) {
        self.0.store(f.to_bits(), order)
    }

    fn load(&self, order: Ordering) -> f32 {
        f32::from_bits(self.0.load(order))
    }
}

/// Mock implementation for web/demo use
#[derive(Clone)]
pub struct MockPowerSupply {
    state: Arc<PowerSupplyState>,
    output_on: Arc<AtomicBool>,
    target_voltage: Arc<AtomicF32>,
    target_current: Arc<AtomicF32>,
    /// Simulated load resistance in ohms (0 = short / current-limited)
    pub load_resistance: f32,
}

impl MockPowerSupply {
    pub fn new() -> Self {
        Self {
            state: Arc::new(PowerSupplyState {
                output_on: false,
                voltage: 0.0,
                current: 0.0,
            }),
            output_on: Arc::new(AtomicBool::new(false)),
            target_voltage: Arc::new(AtomicF32::new(0.0)),
            target_current: Arc::new(AtomicF32::new(0.0)),
            load_resistance: 10.0,
        }
    }
}

impl Default for MockPowerSupply {
    fn default() -> Self {
        Self::new()
    }
}

impl PowerSupplyControl for MockPowerSupply {
    fn get_state(&mut self) -> Option<PowerSupplyState> {
        Some(*self.state)
    }

    fn set_voltage(&mut self, voltage: f32) -> Result<(), String> {
        self.target_voltage.store(voltage, Ordering::SeqCst);
        Ok(())
    }

    fn set_current(&mut self, current: f32) -> Result<(), String> {
        self.target_current.store(current, Ordering::SeqCst);
        Ok(())
    }

    fn set_output(&mut self, enabled: bool) -> Result<(), String> {
        self.output_on.store(enabled, Ordering::SeqCst);
        Ok(())
    }

    fn refresh(&mut self) -> Result<(), String> {
        let target_v = self.target_voltage.load(Ordering::SeqCst);
        let target_c = self.target_current.load(Ordering::SeqCst);
        let output = self.output_on.load(Ordering::SeqCst);

        let (voltage, current) = if output {
            let load_current = if self.load_resistance > 0.0 {
                target_v / self.load_resistance
            } else {
                target_c // short circuit → current limited
            };
            let actual_current = load_current.min(target_c);
            // Voltage sags to V = I * R when current-limited
            let actual_voltage = if self.load_resistance > 0.0 {
                actual_current * self.load_resistance
            } else {
                0.0
            };
            (actual_voltage, actual_current)
        } else {
            (0.0, 0.0)
        };

        self.state = Arc::new(PowerSupplyState { output_on: output, voltage, current });
        Ok(())
    }
}

/// Real KWR103 implementation using the kwr103 crate
pub struct RealPowerSupply {
    device: kwr103::Kwr103,
}

impl RealPowerSupply {
    pub fn new(connection: ConnectionType) -> Result<Self, String> {
        let device = match connection {
            ConnectionType::Usb => {
                use kwr103::UsbConnection;
                let conn = UsbConnection::new("/dev/ttyACM0", 115200, None)
                    .map_err(|e| format!("Failed to open USB: {}", e))?;
                kwr103::Kwr103::from(conn)
            }
            ConnectionType::Eth { ip } => {
                use kwr103::EthConnection;
                let conn = EthConnection::new(&ip)
                    .map_err(|e| format!("Failed to connect to {}: {}", ip, e))?;
                kwr103::Kwr103::from(conn)
            }
        };
        Ok(Self { device })
    }
}

impl PowerSupplyControl for RealPowerSupply {
    fn get_state(&mut self) -> Option<PowerSupplyState> {
        use kwr103::command::Status;
        match self.device.query::<Status>() {
            Ok(status) => Some(PowerSupplyState {
                output_on: status.power == kwr103::command::Switch::On,
                voltage: status.voltage,
                current: status.current,
            }),
            Err(_) => None,
        }
    }

    fn set_voltage(&mut self, voltage: f32) -> Result<(), String> {
        use kwr103::command::Voltage;
        self.device.command(Voltage(voltage))
            .map_err(|e| format!("Failed to set voltage: {}", e))
    }

    fn set_current(&mut self, current: f32) -> Result<(), String> {
        use kwr103::command::Current;
        self.device.command(Current(current))
            .map_err(|e| format!("Failed to set current: {}", e))
    }

    fn set_output(&mut self, enabled: bool) -> Result<(), String> {
        use kwr103::command::Output;
        let state = if enabled { kwr103::command::Switch::On } else { kwr103::command::Switch::Off };
        self.device.command(Output(state))
            .map_err(|e| format!("Failed to set output: {}", e))
    }

    fn refresh(&mut self) -> Result<(), String> {
        // Refresh is implicit through queries
        Ok(())
    }
}

/// A simple voltage sequence step
#[derive(Debug, Clone, Copy)]
pub struct SequenceStep {
    pub voltage: f32,
    pub current_limit: f32,
    pub duration_ms: u32,
    pub output_on: bool,
}

/// Result of advancing a [`VoltageSequence`] by one tick.
pub enum SequenceUpdate {
    /// A new step became active this tick — apply it to the device.
    Step(SequenceStep),
    /// Still within the current step — nothing to send.
    Unchanged,
    /// The sequence reached its end.
    Finished,
}

/// Voltage sequence manager
pub struct VoltageSequence {
    pub(crate) steps: Vec<SequenceStep>,
    current_step: usize,
    step_timer: u32,
    /// Index of the step last handed to the caller, so `update` emits a `Step`
    /// only on entry instead of every tick (avoids flooding the device with
    /// duplicate set_voltage/current/output commands ~100×/s).
    last_applied: Option<usize>,
}

impl VoltageSequence {
    pub fn new() -> Self {
        Self {
            steps: Vec::new(),
            current_step: 0,
            step_timer: 0,
            last_applied: None,
        }
    }

    pub fn get_current_step(&self) -> Option<SequenceStep> {
        self.steps.get(self.current_step).copied()
    }

    pub fn update(&mut self, delta_ms: u32) -> SequenceUpdate {
        if self.steps.is_empty() {
            return SequenceUpdate::Finished;
        }

        self.step_timer += delta_ms;
        let current = self.steps[self.current_step];

        if self.step_timer >= current.duration_ms {
            let next = self.current_step + 1;
            if next >= self.steps.len() {
                // Sequence finished - reset for re-use.
                self.current_step = 0;
                self.step_timer = 0;
                self.last_applied = None;
                return SequenceUpdate::Finished;
            }
            self.current_step = next;
            self.step_timer = 0;
        }

        // Emit the step only when it first becomes active; subsequent ticks
        // within the same step return `Unchanged`, so the caller does not
        // re-send commands to the device every frame.
        if self.last_applied != Some(self.current_step) {
            self.last_applied = Some(self.current_step);
            SequenceUpdate::Step(self.steps[self.current_step])
        } else {
            SequenceUpdate::Unchanged
        }
    }

    pub fn reset(&mut self) {
        self.current_step = 0;
        self.step_timer = 0;
        self.last_applied = None;
    }

    pub fn is_empty(&self) -> bool {
        self.steps.is_empty()
    }

    pub fn len(&self) -> usize {
        self.steps.len()
    }

    pub fn steps(&self) -> &[SequenceStep] {
        &self.steps
    }

    pub fn steps_mut(&mut self) -> &mut Vec<SequenceStep> {
        &mut self.steps
    }

    pub fn remove_step(&mut self, index: usize) {
        if index < self.steps.len() {
            self.steps.remove(index);
        }
    }

    pub fn add_step(&mut self, step: SequenceStep) {
        self.steps.push(step);
    }

    pub fn current_step(&self) -> Option<&SequenceStep> {
        if self.steps.is_empty() || self.step_timer == 0 {
            return None;
        }
        Some(&self.steps[self.current_step])
    }

    pub fn get_playback_progress(&self) -> f32 {
        if self.steps.is_empty() || self.step_timer == 0 {
            return 0.0;
        }
        let current = self.steps[self.current_step];
        (self.step_timer as f32 / current.duration_ms as f32).min(1.0)
    }

    /// Get the current step to apply to power supply, returns None if not playing or no steps
    pub fn get_current_active_step(&self) -> Option<SequenceStep> {
        if self.steps.is_empty() || self.step_timer == 0 {
            return None;
        }
        Some(self.steps[self.current_step])
    }

}

/// Apply a sequence step to the power supply (works with both RealPowerSupply and MockPowerSupply)
pub fn apply_sequence_step<S>(supply: &mut S, step: SequenceStep) -> Result<(), String>
where
    S: PowerSupplyControl + ?Sized,
{
    if let Err(e) = supply.set_voltage(step.voltage) {
        eprintln!("Failed to set voltage: {}", e);
    }
    if let Err(e) = supply.set_current(step.current_limit) {
        eprintln!("Failed to set current: {}", e);
    }
    if step.output_on {
        if let Err(e) = supply.set_output(true) {
            eprintln!("Failed to turn on output: {}", e);
        }
    } else {
        if let Err(e) = supply.set_output(false) {
            eprintln!("Failed to turn off output: {}", e);
        }
    }
    Ok(())
}

impl Default for VoltageSequence {
    fn default() -> Self {
        Self::new()
    }
}

/// UI state for the power supply control
pub struct PowerSupplyUi {
    pub target_voltage: f32,
    pub target_current: f32,
    pub output_enabled: bool,
    pub sequence: VoltageSequence,
    pub playback_progress: f32,
}

impl PowerSupplyUi {
    pub fn new() -> Self {
        Self {
            target_voltage: 5.0,
            target_current: 1.0,
            output_enabled: false,
            sequence: VoltageSequence::new(),
            playback_progress: 0.0,
        }
    }

    pub fn apply_config(&mut self, config: &PowerSupplyConfig) {
        self.target_voltage = self.target_voltage.min(config.max_voltage);
        self.target_current = self.target_current.min(config.max_current);
    }

}


impl Default for PowerSupplyUi {
    fn default() -> Self {
        Self::new()
    }
}

/// Whether the supply is regulating voltage (CV) or current (CC).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LimitMode {
    /// Constant Voltage – load current is below the limit
    CV,
    /// Constant Current – output is current-limited; voltage has sagged
    CC,
}

/// Infer CC/CV mode by comparing measured current against the current setpoint.
/// Returns `None` when the output is off (mode is meaningless).
pub fn infer_limit_mode(
    state: &PowerSupplyState,
    target_current: f32,
) -> Option<LimitMode> {
    if !state.output_on {
        return None;
    }
    // Allow 2 % headroom to avoid flickering near the boundary
    if target_current > 0.0 && state.current >= target_current * 0.98 {
        Some(LimitMode::CC)
    } else {
        Some(LimitMode::CV)
    }
}

/// Rolling buffer of (time_s, voltage_V, current_A) samples
pub struct MeasurementHistory {
    samples: std::collections::VecDeque<(f64, f32, f32)>,
    max_age_s: f64,
}

impl MeasurementHistory {
    pub fn new(max_age_s: f64) -> Self {
        Self {
            samples: std::collections::VecDeque::new(),
            max_age_s,
        }
    }

    pub fn push(&mut self, time_s: f64, voltage: f32, current: f32) {
        self.samples.push_back((time_s, voltage, current));
        let cutoff = time_s - self.max_age_s;
        while self.samples.front().map_or(false, |s| s.0 < cutoff) {
            self.samples.pop_front();
        }
    }

    pub fn voltage_points(&self) -> Vec<[f64; 2]> {
        self.samples.iter().map(|&(t, v, _)| [t, v as f64]).collect()
    }

    pub fn current_points(&self) -> Vec<[f64; 2]> {
        self.samples.iter().map(|&(t, _, c)| [t, c as f64]).collect()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
}

/// Render a two-panel voltage/current plot from a MeasurementHistory.
pub fn show_measurement_plot(history: &MeasurementHistory, ui: &mut egui::Ui) {
    use egui_plot::{Line, Plot};

    let voltage_pts = history.voltage_points();
    let current_pts = history.current_points();

    Plot::new("voltage_plot")
        .height(120.0)
        .y_axis_label("Voltage [V]")
        .include_y(0.0)
        .allow_zoom(false)
        .allow_drag(false)
        .show(ui, |plot_ui| {
            plot_ui.line(
                Line::new("Voltage", voltage_pts)
                    .color(egui::Color32::YELLOW)
                    .width(1.5),
            );
        });

    Plot::new("current_plot")
        .height(80.0)
        .y_axis_label("Current [A]")
        .include_y(0.0)
        .allow_zoom(false)
        .allow_drag(false)
        .show(ui, |plot_ui| {
            plot_ui.line(
                Line::new("Current", current_pts)
                    .color(egui::Color32::from_rgb(100, 200, 255))
                    .width(1.5),
            );
        });
}