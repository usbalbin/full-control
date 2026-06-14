//! Multi-MCU framework — descriptor derived from `stm32-metapac` data.
//!
//! Inventory is **per-package** because smaller packages omit peripherals
//! that need pins those packages don't break out (e.g. STM32H523H lacks
//! DCMI/FMC/SDMMC1/USART3/USART6 that the LQFP144 H523Z exposes). Each
//! `Package` variant maps to a generated `mcu_data::<pkg>::RAW` and
//! produces its own `McuDescriptor` lazily.
//!
//! The interpretive layer (timer-kind classification, fast ADC channels,
//! HRTIM fabric, DAC→COMP routing) lives below — those facts are in the
//! reference manuals but not in metapac.

use std::collections::HashMap;
use std::sync::OnceLock;

use crate::g474::PeripheralKind;
use crate::mcu_raw::RawMcuData;
use crate::pinout::ChipVariant;

// ---------- Mcu / Package ----------

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Mcu { G474, H523 }

impl Default for Mcu { fn default() -> Self { Self::G474 } }

impl Mcu {
    pub const ALL: &'static [Mcu] = &[Mcu::G474, Mcu::H523];

    pub fn label(self) -> &'static str {
        match self { Self::G474 => "STM32G474", Self::H523 => "STM32H523" }
    }

    pub fn packages(self) -> &'static [Package] {
        match self {
            Self::G474 => &[
                Package::G474C, Package::G474M, Package::G474P,
                Package::G474Q, Package::G474R, Package::G474V,
            ],
            Self::H523 => &[
                Package::H523C, Package::H523H, Package::H523R,
                Package::H523V, Package::H523Z,
            ],
        }
    }

    pub fn default_package(self) -> Package {
        match self { Self::G474 => Package::G474R, Self::H523 => Package::H523R }
    }

    /// True when planning views (fabric, HRTIM, package-view) are wired
    /// up. H523 currently exposes Inventory + Pin/AF only.
    pub fn is_implemented(self) -> bool {
        match self { Self::G474 => true, Self::H523 => false }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Package {
    G474C, G474M, G474P, G474Q, G474R, G474V,
    H523C, H523H, H523R, H523V, H523Z,
}

impl Default for Package { fn default() -> Self { Self::G474R } }

impl Package {
    pub const ALL: &'static [Package] = &[
        Self::G474C, Self::G474M, Self::G474P, Self::G474Q, Self::G474R, Self::G474V,
        Self::H523C, Self::H523H, Self::H523R, Self::H523V, Self::H523Z,
    ];

    pub fn mcu(self) -> Mcu {
        match self {
            Self::G474C | Self::G474M | Self::G474P
            | Self::G474Q | Self::G474R | Self::G474V => Mcu::G474,
            _ => Mcu::H523,
        }
    }

    /// Package style as used in datasheet titles (LQFP48/UFBGA100/...).
    pub fn package_label(self) -> &'static str {
        match self {
            Self::G474C => "LQFP48",
            Self::G474M => "WLCSP81",
            Self::G474P => "TFBGA100",
            Self::G474Q => "UFBGA121",
            Self::G474R => "LQFP64",
            Self::G474V => "LQFP100",
            Self::H523C => "LQFP48",
            Self::H523H => "UFBGA100",
            Self::H523R => "LQFP64",
            Self::H523V => "LQFP100",
            Self::H523Z => "LQFP144",
        }
    }

    /// "STM32H523Z (LQFP144)" — single-line label for combo boxes.
    pub fn display_label(self) -> String {
        let (mcu, letter) = match self {
            Self::G474C => ("G474", 'C'), Self::G474M => ("G474", 'M'),
            Self::G474P => ("G474", 'P'), Self::G474Q => ("G474", 'Q'),
            Self::G474R => ("G474", 'R'), Self::G474V => ("G474", 'V'),
            Self::H523C => ("H523", 'C'), Self::H523H => ("H523", 'H'),
            Self::H523R => ("H523", 'R'), Self::H523V => ("H523", 'V'),
            Self::H523Z => ("H523", 'Z'),
        };
        format!("STM32{}{} ({})", mcu, letter, self.package_label())
    }

    pub fn raw(self) -> &'static RawMcuData {
        match self {
            Self::G474C => &crate::mcu_data::g474c::RAW,
            Self::G474M => &crate::mcu_data::g474m::RAW,
            Self::G474P => &crate::mcu_data::g474p::RAW,
            Self::G474Q => &crate::mcu_data::g474q::RAW,
            Self::G474R => &crate::mcu_data::g474r::RAW,
            Self::G474V => &crate::mcu_data::g474v::RAW,
            Self::H523C => &crate::mcu_data::h523c::RAW,
            Self::H523H => &crate::mcu_data::h523h::RAW,
            Self::H523R => &crate::mcu_data::h523r::RAW,
            Self::H523V => &crate::mcu_data::h523v::RAW,
            Self::H523Z => &crate::mcu_data::h523z::RAW,
        }
    }

    pub fn descriptor(self) -> &'static McuDescriptor {
        DESCRIPTORS
            .get_or_init(|| {
                Self::ALL.iter().map(|&p| (p, build_descriptor(p))).collect()
            })
            .get(&self)
            .expect("Package::ALL covers every variant")
    }

    /// Bridge into legacy G474-specific `ChipVariant`. Returns `None` for
    /// H523 packages (which don't have a `ChipVariant` representation —
    /// `pinout.rs` is still G474-only and gets migrated next slice).
    pub fn to_g474_variant(self) -> Option<ChipVariant> {
        Some(match self {
            Self::G474C => ChipVariant::G474C,
            Self::G474M => ChipVariant::G474M,
            Self::G474P => ChipVariant::G474P,
            Self::G474Q => ChipVariant::G474Q,
            Self::G474R => ChipVariant::G474R,
            Self::G474V => ChipVariant::G474V,
            _ => return None,
        })
    }

    pub fn from_g474_variant(v: ChipVariant) -> Self {
        match v {
            ChipVariant::G474C => Self::G474C,
            ChipVariant::G474M => Self::G474M,
            ChipVariant::G474P => Self::G474P,
            ChipVariant::G474Q => Self::G474Q,
            ChipVariant::G474R => Self::G474R,
            ChipVariant::G474V => Self::G474V,
        }
    }
}

static DESCRIPTORS: OnceLock<HashMap<Package, McuDescriptor>> = OnceLock::new();

// ---------- Schema ----------

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum TimerKind { Advanced, General32, General16, Basic, LowPower }

#[derive(Copy, Clone, Debug)]
pub struct TimerInstance {
    pub number: u8, pub kind: TimerKind,
    pub channels: u8, pub has_complementary: bool, pub width_bits: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct AdcInstance {
    pub number: u8,
    pub fast_channels: &'static [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct DacInstance {
    pub number: u8, pub channels: u8, pub fast: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct CompInstance { pub number: u8 }

#[derive(Copy, Clone, Debug)]
pub struct OpampInstance { pub number: u8 }

#[derive(Clone, Debug, Default)]
pub struct CommsInventory {
    pub spi: Vec<u8>, pub i2c: Vec<u8>, pub i3c: Vec<u8>,
    pub usart: Vec<u8>, pub uart: Vec<u8>, pub lpuart: Vec<u8>,
    pub fdcan: Vec<u8>, pub ucpd: Vec<u8>,
    pub has_usb: bool, pub octospi: u8,
    pub has_sdmmc: bool, pub has_fmc: bool, pub has_hdmi_cec: bool,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct PeripheralEdge {
    pub from: PeripheralRef, pub to: PeripheralRef, pub kind: EdgeKind,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum EdgeKind { DacToComp }

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct PeripheralRef {
    pub kind: PeripheralKind, pub instance: u8, pub channel: u8,
}

const fn dac_ref(inst: u8, ch: u8) -> PeripheralRef {
    PeripheralRef { kind: PeripheralKind::Dac, instance: inst, channel: ch }
}
const fn comp_ref(inst: u8) -> PeripheralRef {
    PeripheralRef { kind: PeripheralKind::Comp, instance: inst, channel: 0 }
}

pub struct HrtimFabric {
    pub sub_timer_count: u8, pub eev_count: u8,
    pub flt_count: u8, pub adc_trigger_count: u8,
}

pub struct McuDescriptor {
    pub mcu: Mcu,
    pub package: Package,
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

const G474_FAST_ADC: &[u8] = &[1, 2, 3, 4, 5];
const H523_FAST_ADC: &[u8] = &[]; // TBD per DS14540 §5.3.22

const G474_HRTIM: HrtimFabric = HrtimFabric {
    sub_timer_count: 6, eev_count: 10, flt_count: 6, adc_trigger_count: 10,
};

/// DAC→COMP routing per RM0440 §25; metapac doesn't carry inter-peripheral
/// analog connectivity so this stays hand-encoded.
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

// ---------- Build descriptor from raw data ----------

fn build_descriptor(pkg: Package) -> McuDescriptor {
    let raw = pkg.raw();
    let mcu = pkg.mcu();
    let mut d = build_inventory(mcu, pkg, raw);

    match mcu {
        Mcu::G474 => {
            for a in &mut d.adcs { a.fast_channels = G474_FAST_ADC; }
            // G474 DAC3/DAC4 are 2-channel internal-only fast S&H DACs;
            // metapac sees no `OUT*` pins on them so we override here.
            for dac in &mut d.dacs {
                if matches!(dac.number, 3 | 4) { dac.fast = true; dac.channels = 2; }
            }
            d.edges = G474_EDGES;
            d.hrtim = Some(&G474_HRTIM);
        }
        Mcu::H523 => {
            for a in &mut d.adcs { a.fast_channels = H523_FAST_ADC; }
        }
    }

    d
}

fn build_inventory(mcu: Mcu, package: Package, raw: &'static RawMcuData) -> McuDescriptor {
    let mut d = McuDescriptor {
        mcu, package,
        name: raw.name, family: raw.family,
        timers: Vec::new(), adcs: Vec::new(), dacs: Vec::new(),
        comps: Vec::new(), opamps: Vec::new(),
        comms: CommsInventory::default(),
        edges: &[], hrtim: None, raw,
    };

    for p in raw.peripherals {
        let name = p.name;
        if name.ends_with("_COMMON") || name.ends_with("RAM") || name.ends_with("RAM1")
            || name.ends_with("RAM2") || name == "ADC12_COMMON" || name == "ADC345_COMMON"
        { continue; }

        if let Some(n) = strip_prefix_num(name, "ADC") {
            d.adcs.push(AdcInstance { number: n, fast_channels: &[] });
        } else if let Some(n) = strip_prefix_num(name, "DAC") {
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
    d.dacs.sort_by_key(|x| x.number);
    d.comps.sort_by_key(|c| c.number);
    d.opamps.sort_by_key(|o| o.number);
    d.comms.spi.sort();    d.comms.i2c.sort();
    d.comms.i3c.sort();    d.comms.usart.sort();
    d.comms.uart.sort();   d.comms.fdcan.sort();
    d.comms.ucpd.sort();

    d
}

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
    Some(TimerInstance { number: n, kind, channels, has_complementary: comp, width_bits: width })
}

#[cfg(test)]
mod fabric_validation {
    use super::*;

    /// The Tier-3 generator (`gen_fabric`) recovers DAC->COMP routing from
    /// ST's CubeMX COMP modes XML. It must reproduce the hand-coded
    /// `G474_EDGES` exactly — this is the oracle that lets us trust the
    /// generated fabric and then retire the hand table.
    #[test]
    fn cubedb_dac_to_comp_matches_hand_table() {
        let mut generated: Vec<(u8, u8, u8)> = crate::fabric_data::G4_DAC_TO_COMP.to_vec();
        generated.sort_unstable();

        let mut hand: Vec<(u8, u8, u8)> = G474_EDGES
            .iter()
            .filter(|e| e.kind == EdgeKind::DacToComp)
            .map(|e| (e.from.instance, e.from.channel, e.to.instance))
            .collect();
        hand.sort_unstable();

        assert_eq!(
            generated, hand,
            "cubedb-generated DAC->COMP must match hand-coded G474_EDGES \
             (run `cargo run --bin gen_fabric --features gen-fabric -- <COMP modes xml>`)"
        );
    }
}
