//! Descriptor-driven pin/AF lookups, sourced from `mcu_data` (extracted
//! by `tools/extract`). Per-package pin availability is *not* yet
//! modelled — `RawMcuData` carries one representative chip per family
//! (G474RE, H523RE). Per-package masks live in `pinout.rs` for G474 and
//! will be extended later when other H523 packages are needed.

use crate::mcu_raw::{RawMcuData, RawPin};

/// MCU-independent pin identity. "PA0" → `PinId { port: 'A', num: 0 }`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct PinId {
    pub port: char,
    pub num: u8,
}

impl PinId {
    pub fn from_metapac(s: &str) -> Option<PinId> {
        let b = s.as_bytes();
        if b.len() < 3 || b[0] != b'P' { return None; }
        let port = b[1] as char;
        if !port.is_ascii_uppercase() { return None; }
        let num: u8 = std::str::from_utf8(&b[2..]).ok()?.parse().ok()?;
        Some(PinId { port, num })
    }

    pub fn name(self) -> String { format!("P{}{}", self.port, self.num) }
}

/// Logical pin role on a peripheral instance. `peripheral` is metapac's
/// instance name ("USART1"), `role` is the signal name ("TX"). Both are
/// `&'static str` borrowed from the generated `mcu_data` tables.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct SignalId {
    pub peripheral: &'static str,
    pub role: &'static str,
}

/// One AF-table row: a pin that carries `signal` at AF `af`. `None` AF
/// means analog (DAC outputs, ADC inputs, COMP/OPAMP I/O) — pin is
/// dedicated, no alternate-function selection.
#[derive(Copy, Clone, Debug)]
pub struct AfRow {
    pub pin: PinId,
    pub signal: SignalId,
    pub af: Option<u8>,
}

/// Walk all peripherals × pins. Each yielded `AfRow` is a single
/// (peripheral, pin, signal, af) tuple from `RawMcuData`.
pub fn af_rows(raw: &'static RawMcuData) -> impl Iterator<Item = AfRow> + 'static {
    raw.peripherals.iter().flat_map(|p| {
        p.pins.iter().filter_map(move |pp: &RawPin| {
            Some(AfRow {
                pin: PinId::from_metapac(pp.pin)?,
                signal: SignalId { peripheral: p.name, role: pp.signal },
                af: pp.af,
            })
        })
    })
}

/// All AF placements for a logical signal (e.g. USART1.TX).
pub fn placements_for(raw: &'static RawMcuData, sig: SignalId) -> Vec<AfRow> {
    af_rows(raw)
        .filter(|r| r.signal == sig)
        .collect()
}

/// All signals available on a pin (with their AF numbers).
pub fn signals_on(raw: &'static RawMcuData, pin: PinId) -> Vec<AfRow> {
    af_rows(raw)
        .filter(|r| r.pin == pin)
        .collect()
}

/// All distinct peripherals on this MCU that carry at least one pin.
/// Useful for filter dropdowns in the AF view.
pub fn peripherals_with_pins(raw: &'static RawMcuData) -> Vec<&'static str> {
    let mut v: Vec<&str> = raw.peripherals.iter()
        .filter(|p| !p.pins.is_empty())
        .map(|p| p.name)
        .collect();
    v.sort();
    v.dedup();
    v
}
