//! Multi-MCU framework — descriptor derived from `stm32-metapac` data.
//!
//! Slice 2b: peripheral inventory + AF tables come from `mcu_data::*`
//! (extracted by `tools/extract.rs`). The interpretive layer (timer-kind
//! classification, fast-channel rules, HRTIM fabric, DAC→COMP routing)
//! lives below — those facts are documented in the reference manuals
//! but not present in metapac, so they're hand-encoded per MCU.

use std::sync::LazyLock;

use crate::g474::PeripheralKind;
use crate::mcu_raw::RawMcuData;
use crate::pinout::ChipVariant;

// ---------- MCU family selector ----------

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Mcu {
    G474,
    H523,
}

impl Default for Mcu {
    fn default() -> Self { Self::G474 }
}

impl Mcu {
    pub const ALL: &'static [Mcu] = &[Mcu::G474, Mcu::H523];

    pub fn label(self) -> &'static str {
        match self { Self::G474 => "STM32G474", Self::H523 => "STM32H523" }
    }

    pub fn descriptor(self) -> &'static McuDescriptor {
        match self {
            Self::G474 => &G474_DESC,
            Self::H523 => &H523_DESC,
        }
    }

    pub fn is_implemented(self) -> bool {
        match self { Self::G474 => true, Self::H523 => false }
    }

    pub fn default_g474_variant(self) -> ChipVariant { ChipVariant::G474R }
}

// ---------- Schema ----------

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum TimerKind {
    /// TIM1, TIM8 (G474 also TIM20). Complementary outputs, BKIN, dead-time.
    Advanced,
    /// 32-bit GP timers (TIM2, TIM5).
    General32,
    /// 16-bit GP timers (TIM3, TIM4, TIM12, TIM15, TIM16, TIM17).
    General16,
    /// TIM6, TIM7 — counter-only, used as DAC/ADC time bases.
    Basic,
    /// LPTIM* — runs in Stop mode. Present on H523, absent on G474.
    LowPower,
}

#[derive(Copy, Clone, Debug)]
pub struct TimerInstance {
    pub number: u8,
    pub kind: TimerKind,
    pub channels: u8,
    pub has_complementary: bool,
    pub width_bits: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct AdcInstance {
    pub number: u8,
    /// Channels marked "fast" (low R_AIN). Per-MCU rule, hand-encoded.
    pub fast_channels: &'static [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct DacInstance {
    pub number: u8,
    pub channels: u8,
    /// 15-Msps sample-and-hold "fast" DAC (G474 DAC3/DAC4 — output is
    /// internal-only).
    pub fast: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct CompInstance { pub number: u8 }

#[derive(Copy, Clone, Debug)]
pub struct OpampInstance { pub number: u8 }

#[derive(Clone, Debug, Default)]
pub struct CommsInventory {
    pub spi: Vec<u8>,
    pub i2c: Vec<u8>,
    pub i3c: Vec<u8>,
    pub usart: Vec<u8>,
    pub uart: Vec<u8>,
    pub lpuart: Vec<u8>,
    pub fdcan: Vec<u8>,
    pub ucpd: Vec<u8>,
    pub has_usb: bool,
    pub octospi: u8,
    pub has_sdmmc: bool,
    pub has_fmc: bool,
    pub has_hdmi_cec: bool,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct PeripheralEdge {
    pub from: PeripheralRef,
    pub to: PeripheralRef,
    pub kind: EdgeKind,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum EdgeKind { DacToComp }

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct PeripheralRef {
    pub kind: PeripheralKind,
    pub instance: u8,
    pub channel: u8,
}

const fn dac_ref(inst: u8, ch: u8) -> PeripheralRef {
    PeripheralRef { kind: PeripheralKind::Dac, instance: inst, channel: ch }
}
const fn comp_ref(inst: u8) -> PeripheralRef {
    PeripheralRef { kind: PeripheralKind::Comp, instance: inst, channel: 0 }
}

pub struct HrtimFabric {
    pub sub_timer_count: u8,
    pub eev_count: u8,
    pub flt_count: u8,
    pub adc_trigger_count: u8,
}

pub struct McuDescriptor {
    pub name: &'static str,
    pub family: &'static str,
    pub timers: Vec<TimerInstance>,
    pub adcs: Vec<AdcInstance>,
    pub dacs: Vec<DacInstance>,
    pub comps: Vec<CompInstance>,
    pub opamps: Vec<OpampInstance>,
    pub comms: CommsInventory,
    pub edges: &'static [PeripheralEdge],
    pub hrtim: Option<&'static HrtimFabric>,
    pub raw: &'static RawMcuData,
}

// ---------- Per-MCU hand-encoded annotations ----------

// G474 fast ADC channels: per DS12288 footnote, ADCx_IN1..IN5 are fast on
// every ADC.
const G474_FAST_ADC: &[u8] = &[1, 2, 3, 4, 5];
// H523 fast ADC: TBD; populate after cross-check against DS14540 §5.3.22.
const H523_FAST_ADC: &[u8] = &[];

const G474_HRTIM: HrtimFabric = HrtimFabric {
    sub_timer_count: 6, // TIMA..TIMF
    eev_count: 10,
    flt_count: 6,
    adc_trigger_count: 10,
};

/// DAC→COMP routing per RM0440 §25 / `g474::DAC_TO_COMP`. Hand-encoded —
/// metapac doesn't carry inter-peripheral analog connectivity.
const G474_EDGES: &[PeripheralEdge] = &[
    PeripheralEdge { from: dac_ref(1, 1), to: comp_ref(1), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(1, 1), to: comp_ref(3), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(1, 1), to: comp_ref(4), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(1, 2), to: comp_ref(2), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(1, 2), to: comp_ref(5), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(2, 1), to: comp_ref(6), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(2, 1), to: comp_ref(7), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(3, 1), to: comp_ref(1), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(3, 1), to: comp_ref(3), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(3, 2), to: comp_ref(2), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(3, 2), to: comp_ref(4), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(4, 1), to: comp_ref(5), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(4, 1), to: comp_ref(7), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_ref(4, 2), to: comp_ref(6), kind: EdgeKind::DacToComp },
];

// ---------- Build descriptor from raw metapac data ----------

static G474_DESC: LazyLock<McuDescriptor> = LazyLock::new(|| {
    let mut d = build_inventory(&crate::mcu_data::g474::RAW);
    for a in &mut d.adcs { a.fast_channels = G474_FAST_ADC; }
    // G474 DAC3/DAC4 are 2-channel fast S&H DACs whose outputs are
    // internal-only (no pins); metapac reports zero `OUT*` signals so we
    // override here.
    for dac in &mut d.dacs {
        if matches!(dac.number, 3 | 4) {
            dac.fast = true;
            dac.channels = 2;
        }
    }
    d.edges = G474_EDGES;
    d.hrtim = Some(&G474_HRTIM);
    d
});

static H523_DESC: LazyLock<McuDescriptor> = LazyLock::new(|| {
    let mut d = build_inventory(&crate::mcu_data::h523::RAW);
    for a in &mut d.adcs { a.fast_channels = H523_FAST_ADC; }
    // H523 has no fast S&H DACs.
    d
});

/// Walks `raw.peripherals` and bins each instance by name prefix into the
/// typed inventory. The classification is the only piece of MCU-specific
/// interpretation here; everything below it is data-driven.
fn build_inventory(raw: &'static RawMcuData) -> McuDescriptor {
    let mut d = McuDescriptor {
        name: raw.name,
        family: raw.family,
        timers: Vec::new(),
        adcs: Vec::new(),
        dacs: Vec::new(),
        comps: Vec::new(),
        opamps: Vec::new(),
        comms: CommsInventory::default(),
        edges: &[],
        hrtim: None,
        raw,
    };

    for p in raw.peripherals {
        let name = p.name;
        // Skip aggregate / shared blocks that aren't user-configurable.
        if name.ends_with("_COMMON") || name.ends_with("RAM") || name.ends_with("RAM1")
            || name.ends_with("RAM2") || name == "ADC12_COMMON" || name == "ADC345_COMMON"
        { continue; }

        if let Some(n) = strip_prefix_num(name, "ADC") {
            d.adcs.push(AdcInstance { number: n, fast_channels: &[] });
        } else if let Some(n) = strip_prefix_num(name, "DAC") {
            // Channel count: DAC1 always has 2 channels on supported MCUs;
            // G474's DAC2 has 1 channel. Counted from `OUT*` signals among
            // the peripheral's pins.
            let channels = p.pins.iter().filter(|pp| pp.signal.starts_with("OUT")).count();
            d.dacs.push(DacInstance { number: n, channels: channels as u8, fast: false });
        } else if let Some(n) = strip_prefix_num(name, "COMP") {
            d.comps.push(CompInstance { number: n });
        } else if let Some(n) = strip_prefix_num(name, "OPAMP") {
            d.opamps.push(OpampInstance { number: n });
        } else if name == "HRTIM" || name == "HRTIM1" {
            // Presence detected; structural data carried in HrtimFabric.
        } else if let Some(t) = classify_timer(name) {
            d.timers.push(t);
        } else if let Some(n) = strip_prefix_num(name, "SPI") {
            d.comms.spi.push(n);
        } else if let Some(n) = strip_prefix_num(name, "I2C") {
            d.comms.i2c.push(n);
        } else if let Some(n) = strip_prefix_num(name, "I3C") {
            d.comms.i3c.push(n);
        } else if let Some(n) = strip_prefix_num(name, "USART") {
            d.comms.usart.push(n);
        } else if let Some(n) = strip_prefix_num(name, "UART") {
            d.comms.uart.push(n);
        } else if name == "LPUART1" {
            d.comms.lpuart.push(1);
        } else if let Some(n) = strip_prefix_num(name, "FDCAN") {
            d.comms.fdcan.push(n);
        } else if let Some(n) = strip_prefix_num(name, "UCPD") {
            d.comms.ucpd.push(n);
        } else if name == "USB" || name == "USB_OTG_FS" || name == "USB_OTG_HS" {
            d.comms.has_usb = true;
        } else if name.starts_with("OCTOSPI") {
            d.comms.octospi = d.comms.octospi.saturating_add(1);
        } else if name.starts_with("SDMMC") {
            d.comms.has_sdmmc = true;
        } else if name == "FMC" {
            d.comms.has_fmc = true;
        } else if name == "HDMI_CEC" || name == "CEC" {
            d.comms.has_hdmi_cec = true;
        }
    }

    d.timers.sort_by_key(|t| (t.kind as u8, t.number));
    d.adcs.sort_by_key(|a| a.number);
    d.dacs.sort_by_key(|d| d.number);
    d.comps.sort_by_key(|c| c.number);
    d.opamps.sort_by_key(|o| o.number);
    d.comms.spi.sort();
    d.comms.i2c.sort();
    d.comms.i3c.sort();
    d.comms.usart.sort();
    d.comms.uart.sort();
    d.comms.fdcan.sort();
    d.comms.ucpd.sort();

    d
}

/// "TIM12" → Some(12), "USART3" → None (wrong prefix).
fn strip_prefix_num(name: &str, prefix: &str) -> Option<u8> {
    let rest = name.strip_prefix(prefix)?;
    rest.parse::<u8>().ok()
}

fn classify_timer(name: &str) -> Option<TimerInstance> {
    if let Some(n) = strip_prefix_num(name, "LPTIM") {
        return Some(TimerInstance {
            number: n, kind: TimerKind::LowPower,
            channels: 2, has_complementary: false, width_bits: 16,
        });
    }
    let n = strip_prefix_num(name, "TIM")?;
    let (kind, channels, comp, width) = match n {
        1 | 8 | 20 => (TimerKind::Advanced,  4, true,  16),
        2 | 5      => (TimerKind::General32, 4, false, 32),
        3 | 4      => (TimerKind::General16, 4, false, 16),
        12         => (TimerKind::General16, 2, false, 16),
        15         => (TimerKind::General16, 2, true,  16),
        16 | 17    => (TimerKind::General16, 1, true,  16),
        6 | 7      => (TimerKind::Basic,     0, false, 16),
        _ => return None,
    };
    Some(TimerInstance {
        number: n, kind, channels, has_complementary: comp, width_bits: width,
    })
}
