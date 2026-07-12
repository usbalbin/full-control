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
            !((l.peripheral == peripheral && l.role == role)
                || (l.port == pin.port && l.num == pin.num))
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

    /// Lower this (generic, descriptor-driven) design into the unified
    /// [`PinPlan`](crate::pin_plan::PinPlan). One-way / derived: the result is
    /// never edited back (see `docs/firmware-codegen-design.md` §8). This is the
    /// *generic* lowerer — it touches nothing family-specific, so it serves H5,
    /// C5A3, and any future descriptor-only family unchanged. `raw` is the
    /// active package's descriptor, used only to look up AF numbers.
    ///
    /// `routes`/`slot_claims` stay empty: a descriptor-only family has no analog
    /// crossbar. `dma`/`irqs`/`package_pin`/`net` stay unset (no producer yet);
    /// their `serde(default)` slots mean adding those later won't reshape the type.
    pub fn to_pin_plan(
        &self,
        target: crate::pin_plan::Target,
        raw: &'static RawMcuData,
    ) -> crate::pin_plan::PinPlan {
        use crate::mcu_pinout::{af_rows, OwnedSignal};
        use crate::pin_plan::{Placement, PinOrigin, PinPlan, RoleKind};

        // AF lookup for a placed (peripheral, role, pin). `None` for analog pins.
        let af_of = |peripheral: &str, role: &str, pin: PinId| -> Option<u8> {
            af_rows(raw)
                .find(|r| {
                    r.pin == pin && r.signal.peripheral == peripheral && r.signal.role == role
                })
                .and_then(|r| r.af)
        };

        let placed = |signal: OwnedSignal, pin: Option<PinId>, af: Option<u8>| Placement {
            signal,
            pin,
            // Everything in this model is user-driven intent (a lock, or a
            // declared-but-unplaced use), so the origin is always `Locked`.
            origin: PinOrigin::Locked,
            af,
            role_kind: RoleKind::Gpio, // model carries no typed comms flags
            irqs: Vec::new(),
            package_pin: None,
            net: None,
        };

        let mut plan = PinPlan::empty(target);

        // 1. Every pin-locked signal → a placed Placement (pin_locks authoritative).
        for l in &self.pin_locks {
            let pin = l.pin();
            let sig = OwnedSignal { peripheral: l.peripheral.clone(), role: l.role.clone() };
            let af = af_of(&l.peripheral, &l.role, pin);
            plan.placements.push(placed(sig, Some(pin), af));
        }

        // 2. Declared-but-unplaced roles → a Placement with `pin: None`, so the
        //    plan is self-describing without re-running `validate()`.
        for (peripheral, role) in self.declared_signals() {
            if self.locked_pin(peripheral, role).is_none() {
                let sig = OwnedSignal { peripheral: peripheral.to_string(), role: role.to_string() };
                plan.placements.push(placed(sig, None, None));
            }
        }

        plan.sort_placements();
        plan
    }

    /// Completeness / reachability check for the declared design on `raw`'s
    /// package: a declared role that can't be placed on this package at all
    /// (`Unreachable`) or hasn't been pinned yet (`Unplaced`). Empty == every
    /// declared role is placed on a real pin.
    ///
    /// Memoized: this is called up to twice per egui frame (status line + the
    /// pin-map view) and scans the whole AF table, so a single-slot thread-local
    /// memo keyed on `(raw pointer, this design)` collapses the repeated calls to
    /// one recompute per (chip, design) change. The design fingerprint (a clone,
    /// compared by value) is load-bearing — a raw-only key would serve a stale
    /// problem set after every placement/lock/role edit.
    pub fn validate(&self, raw: &'static RawMcuData) -> Vec<H523Problem> {
        thread_local! {
            static MEMO: std::cell::RefCell<Option<(usize, H523Design, Vec<H523Problem>)>> =
                const { std::cell::RefCell::new(None) };
        }
        let key = raw as *const RawMcuData as usize;
        MEMO.with(|m| {
            let mut slot = m.borrow_mut();
            if let Some((k, d, res)) = slot.as_ref()
                && *k == key
                && d == self
            {
                return res.clone();
            }
            let res = self.validate_uncached(raw);
            *slot = Some((key, self.clone(), res.clone()));
            res
        })
    }

    fn validate_uncached(&self, raw: &'static RawMcuData) -> Vec<H523Problem> {
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
    fn to_pin_plan_lowers_locks_and_unplaced_generically() {
        use crate::pin_plan::{PinOrigin, Target, PIN_PLAN_FORMAT_VERSION};
        let raw = Package::H523R.descriptor().raw;
        let mut d = H523Design::new();
        d.add_use("USART1", &["TX", "RX"]);
        let tx = pins_for(raw, SignalId { peripheral: "USART1", role: "TX" })[0];
        d.lock("USART1", "TX", tx);

        let plan = d.to_pin_plan(
            Target { package: "H523RE".into(), family: "H5".into() },
            raw,
        );

        // Descriptor-only family: no analog fabric.
        assert_eq!(plan.format_version, PIN_PLAN_FORMAT_VERSION);
        assert!(plan.routes.is_empty() && plan.slot_claims.is_empty());

        // TX is placed (origin Locked, real pin, an AF pin so af is Some).
        let tx_p = plan.placements.iter().find(|p| p.signal.role == "TX").unwrap();
        assert_eq!(tx_p.signal.peripheral, "USART1");
        assert_eq!(tx_p.pin, Some(tx));
        assert_eq!(tx_p.origin, PinOrigin::Locked);
        assert!(tx_p.af.is_some(), "USART TX is an alternate-function pin");

        // RX is declared but unplaced → carried with pin: None.
        let rx_p = plan.placements.iter().find(|p| p.signal.role == "RX").unwrap();
        assert_eq!(rx_p.pin, None);

        // Stable order by (peripheral, role): RX sorts before TX.
        assert_eq!(plan.placements.len(), 2);
        assert_eq!(plan.placements[0].signal.role, "RX");
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
