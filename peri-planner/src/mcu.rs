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
use crate::mcu_raw::{DmaPoolDef, RawDmaLeg, RawMcuData};
use crate::pinout::ChipVariant;

// ---------- Mcu / Package ----------

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Mcu { G474, H523, C5A3, C531 }

impl Default for Mcu { fn default() -> Self { Self::G474 } }

impl Mcu {
    pub const ALL: &'static [Mcu] = &[Mcu::G474, Mcu::H523, Mcu::C5A3, Mcu::C531];

    pub fn label(self) -> &'static str {
        match self {
            Self::G474 => "STM32G474",
            Self::H523 => "STM32H523",
            Self::C5A3 => "STM32C5A3",
            Self::C531 => "STM32C531",
        }
    }

    /// Compiled packages for this MCU, from the part registry (data-driven).
    pub fn packages(self) -> Vec<Package> {
        Package::ALL.iter().copied().filter(|p| p.mcu() == self).collect()
    }

    pub fn default_package(self) -> Package {
        match self {
            Self::G474 => Package::G474R,
            Self::H523 => Package::H523R,
            Self::C5A3 => Package::C5A3Z,
            Self::C531 => Package::C531R,
        }
    }

    /// True when planning views (fabric, HRTIM, package-view) are wired
    /// up. H523/C5A3/C531 currently expose Inventory + Pin/AF only.
    pub fn is_implemented(self) -> bool {
        match self {
            Self::G474 => true,
            Self::H523 | Self::C5A3 | Self::C531 => false,
        }
    }
}

/// One compiled part — the per-part **registry** row. Add a part by adding its
/// `mcu_data` module and one `Package` const + `ALL` entry; no new match arms
/// anywhere. Name, prefix, line and package letter all derive from the extracted
/// `raw.name`; only `package_label` and the behavior `mcu` tag live here because
/// they aren't encoded in the chip name.
pub struct PartInfo {
    pub raw: &'static RawMcuData,
    pub package_label: &'static str,
    pub mcu: Mcu,
}

impl PartInfo {
    /// `"STM32<line><letter>"` prefix (first 10 chars of the name) — the
    /// descriptor key every flash/temp/package variant of this letter shares.
    pub fn chip_prefix(&self) -> &'static str {
        self.raw.name.get(..10).unwrap_or(self.raw.name)
    }
}

/// A compiled part: a lightweight handle into the [`PartInfo`] registry. The
/// named consts (`Package::G474R`, …) are the interactive/planner parts; any
/// part — including descriptor-only catalog lines — is just an `ALL` entry.
/// Equality / hashing / serde are by canonical part name.
#[derive(Copy, Clone)]
pub struct Package {
    info: &'static PartInfo,
}

impl PartialEq for Package {
    fn eq(&self, o: &Self) -> bool {
        self.info.raw.name == o.info.raw.name
    }
}
impl Eq for Package {}
impl std::hash::Hash for Package {
    fn hash<H: std::hash::Hasher>(&self, h: &mut H) {
        self.info.raw.name.hash(h)
    }
}
impl std::fmt::Debug for Package {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Package({})", self.info.raw.name)
    }
}
impl Default for Package {
    fn default() -> Self {
        Self::G474R
    }
}
impl serde::Serialize for Package {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(self.info.raw.name)
    }
}
impl<'de> serde::Deserialize<'de> for Package {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let name = String::deserialize(d)?;
        Package::for_chip_name(&name)
            .ok_or_else(|| serde::de::Error::custom(format!("unknown package {name}")))
    }
}

impl Package {
    pub const G474C: Package = Package { info: &PartInfo { raw: &crate::mcu_data::g474c::RAW, package_label: "LQFP48",   mcu: Mcu::G474 } };
    pub const G474M: Package = Package { info: &PartInfo { raw: &crate::mcu_data::g474m::RAW, package_label: "WLCSP81",  mcu: Mcu::G474 } };
    pub const G474P: Package = Package { info: &PartInfo { raw: &crate::mcu_data::g474p::RAW, package_label: "TFBGA100", mcu: Mcu::G474 } };
    pub const G474Q: Package = Package { info: &PartInfo { raw: &crate::mcu_data::g474q::RAW, package_label: "UFBGA121", mcu: Mcu::G474 } };
    pub const G474R: Package = Package { info: &PartInfo { raw: &crate::mcu_data::g474r::RAW, package_label: "LQFP64",   mcu: Mcu::G474 } };
    pub const G474V: Package = Package { info: &PartInfo { raw: &crate::mcu_data::g474v::RAW, package_label: "LQFP100",  mcu: Mcu::G474 } };
    pub const H523C: Package = Package { info: &PartInfo { raw: &crate::mcu_data::h523c::RAW, package_label: "LQFP48",   mcu: Mcu::H523 } };
    pub const H523H: Package = Package { info: &PartInfo { raw: &crate::mcu_data::h523h::RAW, package_label: "UFBGA100", mcu: Mcu::H523 } };
    pub const H523R: Package = Package { info: &PartInfo { raw: &crate::mcu_data::h523r::RAW, package_label: "LQFP64",   mcu: Mcu::H523 } };
    pub const H523V: Package = Package { info: &PartInfo { raw: &crate::mcu_data::h523v::RAW, package_label: "LQFP100",  mcu: Mcu::H523 } };
    pub const H523Z: Package = Package { info: &PartInfo { raw: &crate::mcu_data::h523z::RAW, package_label: "LQFP144",  mcu: Mcu::H523 } };
    pub const C5A3Z: Package = Package { info: &PartInfo { raw: &crate::mcu_data::c5a3z::RAW, package_label: "LQFP144",  mcu: Mcu::C5A3 } };
    pub const C531R: Package = Package { info: &PartInfo { raw: &crate::mcu_data::c531r::RAW, package_label: "LQFP64",   mcu: Mcu::C531 } };

    /// The part registry. THE single place a part is listed.
    pub const ALL: &'static [Package] = &[
        Self::G474C, Self::G474M, Self::G474P, Self::G474Q, Self::G474R, Self::G474V,
        Self::H523C, Self::H523H, Self::H523R, Self::H523V, Self::H523Z,
        Self::C5A3Z,
        Self::C531R,
    ];

    pub fn mcu(self) -> Mcu {
        self.info.mcu
    }

    pub fn raw(self) -> &'static RawMcuData {
        self.info.raw
    }

    /// Canonical part name, e.g. "STM32G474RE" / "STM32C531RCT6".
    pub fn name(self) -> &'static str {
        self.info.raw.name
    }

    /// Package style as used in datasheet titles (LQFP48/UFBGA100/...).
    pub fn package_label(self) -> &'static str {
        self.info.package_label
    }

    /// `"STM32<line><package-letter>"` (e.g. "STM32G474R") — the descriptor key
    /// shared by every flash/temp/package variant of this letter.
    pub fn chip_prefix(self) -> &'static str {
        self.info.chip_prefix()
    }

    /// "STM32G474R (LQFP64)" — single-line label for combo boxes.
    pub fn display_label(self) -> String {
        format!("{} ({})", self.chip_prefix(), self.info.package_label)
    }

    /// The compiled-in package whose descriptor represents `name` — the
    /// catalog-part → descriptor **bridge**. Exact match first, then the
    /// `chip_prefix` (family + package letter), so every flash / temperature /
    /// packaging variant of a supported package letter resolves (e.g.
    /// `STM32G474RB`, `STM32G474RET6`, and `STM32C531RC` — whose catalog name
    /// differs from the metapac RAW name `STM32C531RCT6`). Sound because pins and
    /// peripheral instances are fixed by the package letter, not the flash code.
    pub fn for_chip_name(name: &str) -> Option<Package> {
        if let Some(p) = Self::ALL.iter().copied().find(|p| p.raw().name == name) {
            return Some(p);
        }
        Self::ALL.iter().copied().find(|p| name.starts_with(p.chip_prefix()))
    }

    pub fn descriptor(self) -> &'static McuDescriptor {
        DESCRIPTORS
            .get_or_init(|| Self::ALL.iter().map(|&p| (p, build_descriptor(p))).collect())
            .get(&self)
            .expect("Package::ALL covers every part")
    }

    /// Bridge into the legacy G474-specific `ChipVariant`. `None` for non-G474
    /// parts (the HRTIM planner is G474-only).
    pub fn to_g474_variant(self) -> Option<ChipVariant> {
        if self.info.mcu != Mcu::G474 {
            return None;
        }
        // Package letter = 10th char of the name (after "STM32" + 4-char line).
        Some(match self.info.raw.name.as_bytes().get(9).copied() {
            Some(b'C') => ChipVariant::G474C,
            Some(b'M') => ChipVariant::G474M,
            Some(b'P') => ChipVariant::G474P,
            Some(b'Q') => ChipVariant::G474Q,
            Some(b'R') => ChipVariant::G474R,
            Some(b'V') => ChipVariant::G474V,
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

/// Generic, data-driven analog cross-peripheral routing for a chip. Numeric
/// (instance/channel `u8`, plus the crossbar source's name) so it is
/// family-agnostic — every field mirrors a `fabric_data` table. Adding a
/// family's fabric is data: extract its tables, point a `ChipFabric` at them.
pub struct ChipFabric {
    /// (dac_instance, dac_channel, comp_instance)
    pub dac_to_comp: &'static [(u8, u8, u8)],
    /// (comp_instance, eev_number)
    pub comp_to_eev: &'static [(u8, u8)],
    /// (comp_instance, flt_number)
    pub comp_to_flt: &'static [(u8, u8)],
    /// (crossbar-source name, &[adc-trigger numbers])
    pub crossbar_to_adc_trigger: &'static [(&'static str, &'static [u8])],

    // ---- Timer-based fabric (non-HRTIM families, e.g. C5). Empty for G4. ----
    /// (comp_instance, tim_instance, break_input 1|2): a comparator output
    /// routed to an advanced-timer break input — the hardware over-current
    /// path for a timer-PWM converter.
    pub comp_to_tim_break: &'static [(u8, u8, u8)],
    /// (trigger-source name e.g. "TIM1_CC1", &[adc instances]). Display-only.
    pub tim_to_adc_trigger: &'static [(&'static str, &'static [u8])],
    /// (trigger-source name e.g. "TIM6_TRGO", dac instance). Display-only.
    pub tim_to_dac_trigger: &'static [(&'static str, u8)],
}

impl ChipFabric {
    /// Comparator instances whose output can drive `tim`'s break input
    /// `break_input` (1 = BRK, 2 = BRK2) — the candidate hardware over-current
    /// sources for a timer-PWM converter. Table order is preserved (candidate
    /// ordering); empty if that timer/input has no comparator break path.
    pub fn comps_for_tim_break(&self, tim: u8, break_input: u8) -> Vec<u8> {
        self.comp_to_tim_break
            .iter()
            .filter(|&&(_, t, b)| t == tim && b == break_input)
            .map(|&(comp, _, _)| comp)
            .collect()
    }

    /// DAC `(instance, channel)` sources that can set comparator `comp`'s
    /// inverting-input threshold. Empty if no DAC routes to that comparator.
    pub fn dac_threshold_sources_for_comp(&self, comp: u8) -> Vec<(u8, u8)> {
        self.dac_to_comp
            .iter()
            .filter(|&&(_, _, c)| c == comp)
            .map(|&(dac, ch, _)| (dac, ch))
            .collect()
    }
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
    /// DAC->COMP edges, derived from `fabric.dac_to_comp` (was `G474_EDGES`).
    pub edges: Vec<PeripheralEdge>,
    pub hrtim: Option<&'static HrtimFabric>,
    /// Generic analog routing, data-driven from `fabric_data`. `None` for
    /// families whose fabric isn't modeled yet (H523, C5A3).
    pub fabric: Option<&'static ChipFabric>,
    pub raw: &'static RawMcuData,
}

impl McuDescriptor {
    /// Physical DMA channel pools (supply): one entry per controller with its
    /// channel count. The constraint-selector's DMA capacity source. Empty for
    /// families whose DMA metadata the compiled metapac lacks (C5 today — its
    /// catalog count comes from stm32-data directly; the descriptor populates on
    /// a metapac refresh).
    pub fn dma_pools(&self) -> &'static [DmaPoolDef] {
        self.raw.dma_pools
    }

    /// Total physical DMA channels across all controllers.
    pub fn dma_channel_total(&self) -> u16 {
        self.raw.dma_pools.iter().map(|p| p.channels as u16).sum()
    }

    /// The DMA legs (signal → allowed controller pools) of a peripheral
    /// instance, by metapac name (e.g. "USART1"). Empty if the peripheral has no
    /// DMA or isn't present. A leg consumes one channel from any one of its pools.
    pub fn dma_routes(&self, peripheral: &str) -> &'static [RawDmaLeg] {
        self.raw
            .peripherals
            .iter()
            .find(|p| p.name == peripheral)
            .map(|p| p.dma)
            .unwrap_or(&[])
    }
}

// ---------- Per-MCU hand-encoded annotations ----------

const G474_FAST_ADC: &[u8] = &[1, 2, 3, 4, 5];
const H523_FAST_ADC: &[u8] = &[]; // TBD per DS14540 §5.3.22

const G474_HRTIM: HrtimFabric = HrtimFabric {
    sub_timer_count: 6, eev_count: 10, flt_count: 6, adc_trigger_count: 10,
};

/// The G474 analog fabric, sourced entirely from the generated + validated
/// `fabric_data` tables. The DAC->COMP edges (formerly the hand-coded
/// `G474_EDGES`) are derived from `dac_to_comp` in `build_descriptor`.
const G4_FABRIC: ChipFabric = ChipFabric {
    dac_to_comp: crate::fabric_data::G4_DAC_TO_COMP,
    comp_to_eev: crate::fabric_data::G4_COMP_TO_EEV,
    comp_to_flt: crate::fabric_data::G4_COMP_TO_FLT,
    crossbar_to_adc_trigger: crate::fabric_data::G4_CROSSBAR_TO_ADC_TRIGGER,
    // G4 uses the HRTIM fabric above, not the timer-based edges.
    comp_to_tim_break: &[],
    tim_to_adc_trigger: &[],
    tim_to_dac_trigger: &[],
};

/// The STM32C531 analog fabric, hand-cited from RM0522 (see `fabric_data_c5`).
/// C5 has no HRTIM, so the EEV/FLT/crossbar fields are empty; the power-analog
/// chain lives in `dac_to_comp` (threshold) + `comp_to_tim_break` (hardware
/// over-current trip into the TIM1/TIM8 break inputs).
const C5_FABRIC: ChipFabric = ChipFabric {
    dac_to_comp: crate::fabric_data_c5::C5_DAC_TO_COMP,
    comp_to_eev: &[],
    comp_to_flt: &[],
    crossbar_to_adc_trigger: &[],
    comp_to_tim_break: crate::fabric_data_c5::C5_COMP_TO_TIM_BREAK,
    tim_to_adc_trigger: &[],
    tim_to_dac_trigger: &[],
};

/// Build the descriptor's `edges` (DAC->COMP) from a fabric's numeric table.
fn dac_to_comp_edges(fabric: &ChipFabric) -> Vec<PeripheralEdge> {
    fabric
        .dac_to_comp
        .iter()
        .map(|&(dac, ch, comp)| PeripheralEdge {
            from: dac_ref(dac, ch),
            to: comp_ref(comp),
            kind: EdgeKind::DacToComp,
        })
        .collect()
}

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
            d.fabric = Some(&G4_FABRIC);
            d.edges = dac_to_comp_edges(&G4_FABRIC);
            d.hrtim = Some(&G474_HRTIM);
        }
        Mcu::H523 => {
            for a in &mut d.adcs { a.fast_channels = H523_FAST_ADC; }
        }
        Mcu::C531 => {
            // C531 is the first plannable C5: attach its RM0522-cited timer-based
            // fabric (DAC->COMP threshold + COMP->TIM-break over-current). Still
            // no HRTIM.
            d.fabric = Some(&C5_FABRIC);
            d.edges = dac_to_comp_edges(&C5_FABRIC);
        }
        Mcu::C5A3 => {
            // Inventory-only for now: its analog fabric is not yet transcribed
            // (C531 is the verified first plannable C5).
        }
    }

    d
}

/// An inventory + DMA descriptor for an arbitrary part's raw data — no analog
/// fabric. Used by the constraint selector to verify (Tier-2) the whole lineup
/// from the runtime descriptor asset, where parts have no compiled `Package`.
/// The `mcu`/`package` fields are placeholders (the selector reads neither —
/// only `comms`/`timers`/`adcs` instances, `raw` for pins, and `dma_pools`).
pub fn build_asset_descriptor(raw: &'static RawMcuData) -> McuDescriptor {
    build_inventory(Mcu::G474, Package::G474R, raw)
}

fn build_inventory(mcu: Mcu, package: Package, raw: &'static RawMcuData) -> McuDescriptor {
    let mut d = McuDescriptor {
        mcu, package,
        name: raw.name, family: raw.family,
        timers: Vec::new(), adcs: Vec::new(), dacs: Vec::new(),
        comps: Vec::new(), opamps: Vec::new(),
        comms: CommsInventory::default(),
        edges: Vec::new(), hrtim: None, fabric: None, raw,
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
        } else if let Some(t) = classify_timer(name, p.block) {
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

/// Classify a timer instance from its metapac register-block id (`block`) —
/// the normalized, family-agnostic peripheral-variant signal. Replaces the old
/// hard-coded per-number table: kind / channel count / complementary outputs /
/// counter width all follow directly from the block. LPTIM carries no kind
/// variation and is keyed by name.
///
/// Returns `None` for a non-timer name or an unknown/absent block. A missing
/// classification would silently drop the timer from the inventory, so
/// `every_timer_classifies` asserts full coverage across all supported chips.
fn classify_timer(name: &str, block: Option<&str>) -> Option<TimerInstance> {
    if let Some(n) = strip_prefix_num(name, "LPTIM") {
        return Some(TimerInstance {
            number: n, kind: TimerKind::LowPower,
            channels: 2, has_complementary: false, width_bits: 16,
        });
    }
    let n = strip_prefix_num(name, "TIM")?;
    // (kind, channels, complementary, counter width) straight from the block.
    // Only TIM_GP32 is 32-bit; TIM_ADV and the *_CMP variants drive
    // complementary outputs.
    let (kind, channels, comp, width) = match block? {
        "TIM_ADV"     => (TimerKind::Advanced,  4, true,  16),
        "TIM_GP32"    => (TimerKind::General32, 4, false, 32),
        "TIM_GP16"    => (TimerKind::General16, 4, false, 16),
        "TIM_2CH"     => (TimerKind::General16, 2, false, 16),
        "TIM_2CH_CMP" => (TimerKind::General16, 2, true,  16),
        "TIM_1CH"     => (TimerKind::General16, 1, false, 16),
        "TIM_1CH_CMP" => (TimerKind::General16, 1, true,  16),
        "TIM_BASIC"   => (TimerKind::Basic,     0, false, 16),
        _ => return None,
    };
    Some(TimerInstance { number: n, kind, channels, has_complementary: comp, width_bits: width })
}

#[cfg(test)]
mod fabric_validation {
    use super::*;

    /// DMA supply pools and per-peripheral routes normalize both DMA models into
    /// "leg → allowed controller pools": on G474 (DMAMUX) a signal fans out to
    /// every controller behind the mux; on named-controller families the
    /// controllers are listed directly. Every leg's pools must exist in supply.
    #[test]
    fn dma_pools_and_routes_normalize_across_models() {
        // G474 (DMAMUX): two 8-channel controllers; a peripheral signal fans out
        // to BOTH via the mux.
        let g4 = Package::G474R.descriptor();
        assert_eq!(g4.dma_channel_total(), 16, "G474 = DMA1(8) + DMA2(8)");
        let pools: Vec<(&str, u8)> = g4.dma_pools().iter().map(|p| (p.name, p.channels)).collect();
        assert!(
            pools.contains(&("DMA1", 8)) && pools.contains(&("DMA2", 8)),
            "got {pools:?}"
        );
        let usart = g4.dma_routes("USART1");
        assert!(!usart.is_empty(), "G474 USART1 should carry DMA legs");
        assert!(
            usart.iter().any(|l| l.pools.contains(&"DMA1") && l.pools.contains(&"DMA2")),
            "a G474 USART1 leg should fan out to both controllers via DMAMUX, got {:?}",
            usart.iter().map(|l| (l.signal, l.pools)).collect::<Vec<_>>(),
        );

        // Soundness / name-drift guard across ALL chips: every demand pool a leg
        // references must exist in that chip's supply pools. (C5 has empty DMA in
        // the compiled metapac, so its loop is vacuous — a known data-currency gap.)
        for &pkg in Package::ALL {
            let d = pkg.descriptor();
            let supply: std::collections::HashSet<&str> =
                d.dma_pools().iter().map(|p| p.name).collect();
            for p in d.raw.peripherals {
                for leg in p.dma {
                    for &pool in leg.pools {
                        assert!(
                            supply.contains(pool),
                            "{}: {} leg {:?} references pool {:?} absent from supply {:?}",
                            d.name, p.name, leg.signal, pool, supply,
                        );
                    }
                }
            }
        }
    }

    /// The catalog-part → descriptor bridge resolves flash/temp/packaging
    /// variants of a supported package letter to that package, and refuses
    /// unsupported letters/families. Exact full-name matching (the old behavior)
    /// reached ~1 part per family; the prefix bridge reaches every variant.
    #[test]
    fn for_chip_name_bridges_package_variants() {
        // Exact compiled name still resolves to its own package.
        assert_eq!(Package::for_chip_name("STM32G474RE"), Some(Package::G474R));
        // Flash / temp / packaging variants of a SUPPORTED package letter resolve
        // to that package's (flash-invariant) descriptor.
        assert_eq!(Package::for_chip_name("STM32G474RB"), Some(Package::G474R));
        assert_eq!(Package::for_chip_name("STM32G474RET6"), Some(Package::G474R));
        assert_eq!(Package::for_chip_name("STM32G474VC"), Some(Package::G474V));
        // C531: catalog name "STM32C531RC" differs from metapac RAW name
        // "STM32C531RCT6" — exact match resolved NEITHER; the bridge resolves both.
        assert_eq!(Package::for_chip_name("STM32C531RC"), Some(Package::C531R));
        assert_eq!(Package::for_chip_name("STM32C531RBT6"), Some(Package::C531R));
        // Unsupported package letters / families must NOT falsely resolve.
        assert_eq!(Package::for_chip_name("STM32C531CB"), None); // C package not compiled
        assert_eq!(Package::for_chip_name("STM32C5A3RG"), None); // only Z compiled
        assert_eq!(Package::for_chip_name("STM32F103RB"), None); // unsupported family
        // The bridge resolves materially more of the catalog than exact match.
        let resolved = crate::catalog::CATALOG
            .iter()
            .filter(|e| Package::for_chip_name(&e.name).is_some())
            .count();
        assert!(resolved >= 25, "bridge should resolve many catalog parts, got {resolved}");
    }

    /// Every timer peripheral metapac reports — for every supported package —
    /// must classify from its register block. A `None` here means a block id
    /// the data-driven `classify_timer` doesn't map yet, which would silently
    /// drop the timer from the inventory. Guards the de-hardcoded path.
    #[test]
    fn every_timer_classifies() {
        for &pkg in Package::ALL {
            let raw = pkg.raw();
            for p in raw.peripherals {
                let is_timer = strip_prefix_num(p.name, "LPTIM").is_some()
                    || strip_prefix_num(p.name, "TIM").is_some();
                if is_timer {
                    assert!(
                        classify_timer(p.name, p.block).is_some(),
                        "{}: timer {:?} (block {:?}) failed to classify",
                        raw.name, p.name, p.block,
                    );
                }
            }
        }
    }

    /// Golden snapshot of the RM0440-verified DAC->COMP routing. `fabric_data`
    /// is now the source (the descriptor's `edges` are derived from it), so this
    /// guards against a bad regeneration silently changing the data — the
    /// expectation here is authored independently of the generator.
    #[test]
    fn g4_dac_to_comp_golden() {
        let golden: &[(u8, u8, u8)] = &[
            (1, 1, 1), (1, 1, 3), (1, 1, 4), (1, 2, 2), (1, 2, 5),
            (2, 1, 6), (2, 1, 7), (3, 1, 1), (3, 1, 3), (3, 2, 2),
            (3, 2, 4), (4, 1, 5), (4, 1, 7), (4, 2, 6),
        ];
        let mut got = crate::fabric_data::G4_DAC_TO_COMP.to_vec();
        got.sort_unstable();
        let mut want = golden.to_vec();
        want.sort_unstable();
        assert_eq!(got, want, "G4_DAC_TO_COMP drifted from RM-verified golden");

        // The descriptor's derived edges must match the same data.
        let d = Package::G474R.descriptor();
        let mut edges: Vec<(u8, u8, u8)> = d
            .edges
            .iter()
            .map(|e| (e.from.instance, e.from.channel, e.to.instance))
            .collect();
        edges.sort_unstable();
        assert_eq!(edges, want, "descriptor edges drifted from fabric data");
    }
}

#[cfg(test)]
mod c5_support {
    use super::*;

    /// C5A3 is registered as a 3rd (inventory-only) family; its descriptor must
    /// build from the extracted data and surface the analog/control + serial
    /// peripherals, with no HRTIM.
    #[test]
    fn c5a3_descriptor_inventories_expected_peripherals() {
        let d = Package::C5A3Z.descriptor();
        assert_eq!(d.mcu, Mcu::C5A3);
        assert_eq!(d.name, "STM32C5A3ZGT6");
        assert!(d.comps.iter().any(|c| c.number == 1), "COMP1");
        assert!(d.dacs.iter().any(|x| x.number == 1), "DAC1");
        assert!(
            d.timers
                .iter()
                .any(|t| t.number == 1 && matches!(t.kind, TimerKind::Advanced)),
            "TIM1 advanced"
        );
        assert!(d.timers.iter().any(|t| t.number == 8), "TIM8");
        let serial = d.comms.usart.len() + d.comms.uart.len() + d.comms.lpuart.len();
        assert!(serial >= 7, "expected many async-serial channels, got {serial}");
        assert!(d.comms.fdcan.len() >= 2, "FDCAN1/2");
        assert!(d.hrtim.is_none(), "C5 has no HRTIM");
    }

    /// C531 is the analog-rich C5 line: it carries an OPAMP (which C5A3 lacks),
    /// plus COMP + DAC + advanced timers, still no HRTIM.
    #[test]
    fn c531_descriptor_inventories_analog() {
        let d = Package::C531R.descriptor();
        assert_eq!(d.mcu, Mcu::C531);
        assert_eq!(d.name, "STM32C531RCT6");
        assert!(d.opamps.iter().any(|o| o.number == 1), "OPAMP1");
        assert!(d.comps.iter().any(|c| c.number == 1), "COMP1");
        assert!(d.dacs.iter().any(|x| x.number == 1), "DAC1");
        assert!(
            d.timers
                .iter()
                .any(|t| t.number == 1 && matches!(t.kind, TimerKind::Advanced)),
            "TIM1 advanced"
        );
        assert!(d.hrtim.is_none(), "C5 has no HRTIM");
    }

    /// Golden snapshot of the C531 analog fabric, cross-checked against ST's
    /// CubeMX2 die descriptor (D44F_peripherals.json) intersected with C531's
    /// actual peripheral set. Guards against an edit silently changing the
    /// hardware over-current routing. NB: the die lists `COMP2 <- DAC2`, but
    /// DAC2 is absent on C531, so COMP2 has no internal DAC threshold here.
    #[test]
    fn c531_fabric_golden() {
        // DAC -> COMP inverting input: only DAC1 -> COMP1 on C531 (DAC2, the
        // die-level source for COMP2, is not populated on this part).
        let dac_to_comp: &[(u8, u8, u8)] = &[(1, 1, 1)];
        let mut got = crate::fabric_data_c5::C5_DAC_TO_COMP.to_vec();
        got.sort_unstable();
        let mut want = dac_to_comp.to_vec();
        want.sort_unstable();
        assert_eq!(got, want, "C5_DAC_TO_COMP drifted from RM0522 golden");

        // COMP output -> advanced-timer break (RM0522 section 31.3.2). TIM1 and
        // TIM8 each take COMP1 and COMP2 on BOTH break input 1 (BRK) and 2 (BRK2).
        let comp_to_break: &[(u8, u8, u8)] = &[
            (1, 1, 1), (2, 1, 1), (1, 1, 2), (2, 1, 2),
            (1, 8, 1), (2, 8, 1), (1, 8, 2), (2, 8, 2),
        ];
        let mut gotb = crate::fabric_data_c5::C5_COMP_TO_TIM_BREAK.to_vec();
        gotb.sort_unstable();
        let mut wantb = comp_to_break.to_vec();
        wantb.sort_unstable();
        assert_eq!(gotb, wantb, "C5_COMP_TO_TIM_BREAK drifted from RM0522 golden");

        // The descriptor must attach the fabric and derive its DAC->COMP edges.
        let d = Package::C531R.descriptor();
        let fab = d.fabric.expect("C531 should have an attached fabric");
        assert_eq!(
            fab.comp_to_tim_break,
            crate::fabric_data_c5::C5_COMP_TO_TIM_BREAK,
            "descriptor fabric must point at the C5 break table"
        );
        let mut edges: Vec<(u8, u8, u8)> = d
            .edges
            .iter()
            .map(|e| (e.from.instance, e.from.channel, e.to.instance))
            .collect();
        edges.sort_unstable();
        assert_eq!(edges, want, "C531 descriptor edges drifted from fabric data");
    }

    /// The fabric query layer that Inc 5 (`TimerDesign` break-validation) and the
    /// Inc 6 UI (COMP/DAC dropdowns) will consume must answer the C531 routing
    /// correctly, and be empty where there is no hardware path.
    #[test]
    fn c531_fabric_queries() {
        let d = Package::C531R.descriptor();
        let fab = d.fabric.expect("C531 fabric");

        // Both advanced timers, both break inputs, accept COMP1 and COMP2.
        assert_eq!(fab.comps_for_tim_break(1, 1), vec![1, 2], "TIM1 BRK");
        assert_eq!(fab.comps_for_tim_break(1, 2), vec![1, 2], "TIM1 BRK2");
        assert_eq!(fab.comps_for_tim_break(8, 1), vec![1, 2], "TIM8 BRK");
        assert_eq!(fab.comps_for_tim_break(8, 2), vec![1, 2], "TIM8 BRK2");
        // A general-purpose timer has no comparator break path; nor a 3rd input.
        assert!(fab.comps_for_tim_break(2, 1).is_empty(), "TIM2 has no break");
        assert!(fab.comps_for_tim_break(1, 3).is_empty(), "no 3rd break input");

        // DAC threshold sources: COMP1 <- DAC1. COMP2 has none on C531 (its
        // die-level DAC source DAC2 is not populated on this part).
        assert_eq!(fab.dac_threshold_sources_for_comp(1), vec![(1, 1)], "COMP1");
        assert!(
            fab.dac_threshold_sources_for_comp(2).is_empty(),
            "COMP2 has no internal DAC on C531 (DAC2 absent)"
        );
        assert!(fab.dac_threshold_sources_for_comp(3).is_empty(), "no COMP3");

        // G4 routes OCP through HRTIM EEV, not timer breaks — the timer-break
        // accessor is empty there even though G4 has a rich DAC->COMP fabric.
        let g4 = Package::G474R.descriptor();
        let g4f = g4.fabric.expect("G4 fabric");
        assert!(g4f.comps_for_tim_break(1, 1).is_empty(), "G4 uses HRTIM breaks");
        assert!(!g4f.dac_threshold_sources_for_comp(1).is_empty(), "G4 DAC1->COMP1");
    }
}
