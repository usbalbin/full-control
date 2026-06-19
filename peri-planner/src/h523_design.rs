//! H523 design state. Minimum viable: a list of pin locks. Lives next to
//! the (G474-shaped) `Design` in `requirements.rs` rather than trying to
//! unify them — H523 has no HRTIM/EEV/COMP/OPAMP and the G474 Design's
//! solver, requirements, and assignments are all HRTIM-rich, so a unified
//! type would be mostly empty fields and runtime "applicable here?"
//! checks. A fresh struct is cleaner.
//!
//! Future H523 features (timer pin claims, ADC channel assignments,
//! comms-instance selection) extend this struct directly.

use std::collections::HashMap;

use crate::mcu_pinout::PinId;

/// A pinned (peripheral, role, pin) triple. Identifies the peripheral
/// signal by its metapac names ("USART1", "TX") so the lock survives
/// MCU-data regeneration without ID renaming. `pin` carries (port, num).
#[derive(Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct PinLock {
    pub peripheral: String,
    pub role: String,
    pub port: char,
    pub num: u8,
}

impl PinLock {
    pub fn pin(&self) -> PinId { PinId { port: self.port, num: self.num } }
    pub fn signal_key(&self) -> (&str, &str) { (&self.peripheral, &self.role) }
}

#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct H523Design {
    pub format_version: u32,
    pub pin_locks: Vec<PinLock>,
}

pub const H523_DESIGN_FORMAT_VERSION: u32 = 1;

impl H523Design {
    pub fn new() -> Self {
        Self { format_version: H523_DESIGN_FORMAT_VERSION, pin_locks: Vec::new() }
    }

    /// Pin currently locked for this (peripheral, role) signal, if any.
    pub fn locked_pin(&self, peripheral: &str, role: &str) -> Option<PinId> {
        self.pin_locks.iter()
            .find(|l| l.peripheral == peripheral && l.role == role)
            .map(|l| l.pin())
    }

    /// `(peripheral, role)` currently occupying this pin, if any.
    pub fn occupant_of(&self, pin: PinId) -> Option<(&str, &str)> {
        self.pin_locks.iter()
            .find(|l| l.port == pin.port && l.num == pin.num)
            .map(|l| (l.peripheral.as_str(), l.role.as_str()))
    }

    /// All pins currently taken (by any signal). Used to grey out
    /// already-locked pins in candidate lists.
    pub fn taken_pins(&self) -> HashMap<PinId, (&str, &str)> {
        self.pin_locks.iter()
            .map(|l| (l.pin(), (l.peripheral.as_str(), l.role.as_str())))
            .collect()
    }

    /// Lock `signal` to `pin`. Idempotent: replaces any prior lock for
    /// the same signal. Removes any other lock that was on this pin.
    pub fn lock(&mut self, peripheral: &str, role: &str, pin: PinId) {
        self.pin_locks.retain(|l| {
            !(l.peripheral == peripheral && l.role == role)
                && !(l.port == pin.port && l.num == pin.num)
        });
        self.pin_locks.push(PinLock {
            peripheral: peripheral.to_string(),
            role: role.to_string(),
            port: pin.port,
            num: pin.num,
        });
    }

    pub fn unlock(&mut self, peripheral: &str, role: &str) {
        self.pin_locks.retain(|l| !(l.peripheral == peripheral && l.role == role));
    }

    /// A copy-paste pin-allocation summary for `part_name`.
    pub fn export_summary(&self, part_name: &str) -> String {
        use std::fmt::Write as _;
        let mut s = format!("{part_name} — pin allocation ({} locks)\n", self.pin_locks.len());
        let mut locks = self.pin_locks.clone();
        locks.sort_by(|a, b| {
            (a.peripheral.as_str(), a.role.as_str()).cmp(&(b.peripheral.as_str(), b.role.as_str()))
        });
        for l in &locks {
            let _ = writeln!(s, "  {} {}: P{}{}", l.peripheral, l.role, l.port, l.num);
        }
        s
    }
}
