//! Raw chip-metadata schema. Populated by `tools/extract.rs` from
//! `stm32-metapac`; one generated file per supported MCU lives in
//! `src/mcu_data/`. The interpretation layer (timer-kind classification,
//! fast-channel rules, HRTIM fabric, DAC→COMP edges) lives in `mcu.rs`.

pub struct RawMcuData {
    pub name: &'static str,
    pub family: &'static str,
    pub peripherals: &'static [RawPeripheral],
}

pub struct RawPeripheral {
    pub name: &'static str,
    pub address: u64,
    pub pins: &'static [RawPin],
}

#[derive(Copy, Clone)]
pub struct RawPin {
    /// Pin name in metapac form, e.g. "PA0".
    pub pin: &'static str,
    /// Signal name within the peripheral, e.g. "TX", "MOSI", "INP3".
    pub signal: &'static str,
    /// Alternate-function number (0..15). `None` for analog signals that
    /// aren't AF-mapped (DAC outputs, ADC inputs, COMP/OPAMP I/O).
    pub af: Option<u8>,
}
