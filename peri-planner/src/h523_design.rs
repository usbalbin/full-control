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
use crate::mcu_raw::RawMcuData;

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

/// A declared peripheral use — "I am using USART2 for TX+RX". The intent layer
/// above `PinLock`s: a use names a metapac instance and the roles wanted; each
/// role is then placed on a pin (a `PinLock` keyed on the same `(peripheral,
/// role)`). This is the family-agnostic equivalent of the G474 `Design`'s typed
/// requirements, built directly on descriptor strings.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct PeripheralUse {
    pub peripheral: String,
    pub roles: Vec<String>,
    #[serde(default)]
    pub note: String,
}

/// A reason a declared design isn't complete/realizable on the active package.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum H523Problem {
    /// A declared role has no pin locked yet.
    Unplaced { peripheral: String, role: String },
    /// A declared role cannot be placed on this package at all (no AF pin).
    Unreachable { peripheral: String, role: String },
}

impl PinLock {
    pub fn pin(&self) -> PinId { PinId { port: self.port, num: self.num } }
    pub fn signal_key(&self) -> (&str, &str) { (&self.peripheral, &self.role) }
}

#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct H523Design {
    pub format_version: u32,
    pub pin_locks: Vec<PinLock>,
    /// Declared peripheral uses — the intent layer above `pin_locks`.
    #[serde(default)]
    pub uses: Vec<PeripheralUse>,
}

/// `default()` is the canonical empty design — identical to `new()` (notably
/// carrying the current `format_version`, not a derived 0) so that map
/// `or_default()` / `unwrap_or_default()` produce a coherent fresh design.
impl Default for H523Design {
    fn default() -> Self {
        Self::new()
    }
}

pub const H523_DESIGN_FORMAT_VERSION: u32 = 1;

impl H523Design {
    pub fn new() -> Self {
        Self {
            format_version: H523_DESIGN_FORMAT_VERSION,
            pin_locks: Vec::new(),
            uses: Vec::new(),
        }
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

    /// Declare a peripheral use. No-op if the instance is already declared.
    pub fn add_use(&mut self, peripheral: &str, roles: &[&str]) {
        if self.uses.iter().any(|u| u.peripheral == peripheral) {
            return;
        }
        self.uses.push(PeripheralUse {
            peripheral: peripheral.to_string(),
            roles: roles.iter().map(|r| r.to_string()).collect(),
            note: String::new(),
        });
    }

    /// Remove a declared use (by index) and drop its pin locks.
    pub fn remove_use(&mut self, index: usize) {
        if index < self.uses.len() {
            let u = self.uses.remove(index);
            self.pin_locks.retain(|l| l.peripheral != u.peripheral);
        }
    }

    /// Toggle a role on a declared use; unlocking the role if it's removed.
    pub fn set_role(&mut self, index: usize, role: &str, on: bool) {
        if let Some(u) = self.uses.get_mut(index) {
            let has = u.roles.iter().any(|r| r == role);
            if on && !has {
                u.roles.push(role.to_string());
            } else if !on && has {
                let peripheral = u.peripheral.clone();
                u.roles.retain(|r| r != role);
                self.unlock(&peripheral, role);
            }
        }
    }

    /// Equality ignoring the free-text `note` fields. Undo uses this so typing a
    /// note doesn't create one undo step per keystroke (notes are low-stakes
    /// annotations, reverted only as part of the next structural edit's undo).
    pub fn eq_ignoring_notes(&self, other: &H523Design) -> bool {
        self.pin_locks == other.pin_locks
            && self.uses.len() == other.uses.len()
            && self
                .uses
                .iter()
                .zip(&other.uses)
                .all(|(a, b)| a.peripheral == b.peripheral && a.roles == b.roles)
    }

    /// Every `(peripheral, role)` declared across all uses.
    pub fn declared_signals(&self) -> Vec<(&str, &str)> {
        self.uses
            .iter()
            .flat_map(|u| u.roles.iter().map(move |r| (u.peripheral.as_str(), r.as_str())))
            .collect()
    }

    /// Completeness / reachability check for the declared design on `raw`'s
    /// package: a declared role that can't be placed on this package at all
    /// (`Unreachable`) or hasn't been pinned yet (`Unplaced`). Empty == every
    /// declared role is placed on a real pin.
    pub fn validate(&self, raw: &'static RawMcuData) -> Vec<H523Problem> {
        let rows: Vec<_> = crate::mcu_pinout::af_rows(raw).collect();
        let mut out = Vec::new();
        for u in &self.uses {
            for role in &u.roles {
                let reachable = rows
                    .iter()
                    .any(|r| r.signal.peripheral == u.peripheral && r.signal.role == *role);
                if !reachable {
                    out.push(H523Problem::Unreachable {
                        peripheral: u.peripheral.clone(),
                        role: role.clone(),
                    });
                } else if self.locked_pin(&u.peripheral, role).is_none() {
                    out.push(H523Problem::Unplaced {
                        peripheral: u.peripheral.clone(),
                        role: role.clone(),
                    });
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mcu::Package;
    use crate::mcu_pinout::{pins_for, SignalId};

    #[test]
    fn add_use_dedups_and_set_role_toggles() {
        let mut d = H523Design::new();
        d.add_use("USART2", &["TX", "RX"]);
        d.add_use("USART2", &["TX"]); // already declared -> ignored
        assert_eq!(d.uses.len(), 1);
        assert_eq!(d.uses[0].roles, vec!["TX", "RX"]);
        d.set_role(0, "CTS", true);
        assert!(d.uses[0].roles.iter().any(|r| r == "CTS"));
        d.set_role(0, "RX", false);
        assert!(!d.uses[0].roles.iter().any(|r| r == "RX"));
    }

    #[test]
    fn validate_flags_unplaced_then_clears_when_locked() {
        let raw = Package::H523R.descriptor().raw;
        let mut d = H523Design::new();
        d.add_use("USART1", &["TX", "RX"]);
        assert_eq!(d.validate(raw).len(), 2); // both unplaced
        let tx = pins_for(raw, SignalId { peripheral: "USART1", role: "TX" })[0];
        d.lock("USART1", "TX", tx);
        let p = d.validate(raw);
        assert_eq!(p.len(), 1);
        assert!(matches!(p[0], H523Problem::Unplaced { ref role, .. } if role == "RX"));
    }

    #[test]
    fn eq_ignoring_notes_ignores_note_but_not_structure() {
        let mut a = H523Design::new();
        a.add_use("USART1", &["TX"]);
        let mut b = a.clone();
        b.uses[0].note = "buck leg A".to_string();
        assert!(a.eq_ignoring_notes(&b), "a note-only change must compare equal");
        assert_ne!(a, b, "but PartialEq still distinguishes the note");
        b.set_role(0, "RX", true);
        assert!(!a.eq_ignoring_notes(&b), "a role change is structural");
    }

    #[test]
    fn validate_flags_unreachable_role() {
        let raw = Package::H523R.descriptor().raw;
        let mut d = H523Design::new();
        d.add_use("USART1", &["NONSENSE"]);
        assert_eq!(
            d.validate(raw),
            vec![H523Problem::Unreachable {
                peripheral: "USART1".to_string(),
                role: "NONSENSE".to_string(),
            }]
        );
    }

    #[test]
    fn remove_use_drops_its_pin_locks() {
        let raw = Package::H523R.descriptor().raw;
        let mut d = H523Design::new();
        d.add_use("USART1", &["TX"]);
        let tx = pins_for(raw, SignalId { peripheral: "USART1", role: "TX" })[0];
        d.lock("USART1", "TX", tx);
        assert_eq!(d.pin_locks.len(), 1);
        d.remove_use(0);
        assert!(d.uses.is_empty() && d.pin_locks.is_empty());
    }
}
