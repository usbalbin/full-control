//! KWR103 Power Supply GUI
//!
//! This library provides shared logic for controlling a KWR103 programmable
//! power supply via USB or Ethernet.

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
    config: Arc<PowerSupplyConfig>,
    output_on: Arc<AtomicBool>,
    target_voltage: Arc<AtomicF32>,
    target_current: Arc<AtomicF32>,
}

impl MockPowerSupply {
    pub fn new() -> Self {
        Self {
            state: Arc::new(PowerSupplyState {
                output_on: false,
                voltage: 0.0,
                current: 0.0,
            }),
            config: Arc::new(PowerSupplyConfig::default()),
            output_on: Arc::new(AtomicBool::new(false)),
            target_voltage: Arc::new(AtomicF32::new(0.0)),
            target_current: Arc::new(AtomicF32::new(0.0)),
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
        // Update state based on target values
        let target_v = self.target_voltage.load(Ordering::SeqCst);
        let target_c = self.target_current.load(Ordering::SeqCst);
        let output = self.output_on.load(Ordering::SeqCst);

        let new_state = PowerSupplyState {
            output_on: output,
            voltage: if output { target_v } else { 0.0 },
            current: if output && target_v > 0.0 { target_c } else { 0.0 },
        };

        self.state = Arc::new(new_state);
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
                use kwr103::{UsbConnection, Transport};
                let conn = UsbConnection::new("/dev/ttyACM0", 115200, None)
                    .map_err(|e| format!("Failed to open USB: {}", e))?;
                kwr103::Kwr103::from(conn)
            }
            ConnectionType::Eth { ip } => {
                use kwr103::{EthConnection, Transport};
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

/// Voltage sequence manager
pub struct VoltageSequence {
    pub(crate) steps: Vec<SequenceStep>,
    current_step: usize,
    step_timer: u32,
}

impl VoltageSequence {
    pub fn new() -> Self {
        Self {
            steps: Vec::new(),
            current_step: 0,
            step_timer: 0,
        }
    }

    pub fn get_current_step(&self) -> Option<SequenceStep> {
        self.steps.get(self.current_step).copied()
    }

    pub fn update(&mut self, delta_ms: u32) -> Option<SequenceStep> {
        if self.steps.is_empty() {
            return None;
        }

        self.step_timer += delta_ms;
        let current = self.steps[self.current_step];

        if self.step_timer >= current.duration_ms {
            // Move to next step
            self.current_step = (self.current_step + 1) % self.steps.len();
            self.step_timer = 0;
            Some(self.steps[self.current_step])
        } else {
            Some(current)
        }
    }

    pub fn reset(&mut self) {
        self.current_step = 0;
        self.step_timer = 0;
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

    pub(crate) fn set_current_step(&mut self, step: usize) {
        self.current_step = step;
    }

    pub fn get_playback_progress(&self) -> f32 {
        if self.steps.is_empty() || self.step_timer == 0 {
            return 0.0;
        }
        let current = self.steps[self.current_step];
        (self.step_timer as f32 / current.duration_ms as f32).min(1.0)
    }
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

    pub(crate) fn get_sequence_mut(&mut self) -> &mut VoltageSequence {
        &mut self.sequence
    }
}

impl Default for PowerSupplyUi {
    fn default() -> Self {
        Self::new()
    }
}