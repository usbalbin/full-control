//! Multi-MCU framework — descriptor-driven peripheral inventory.
//!
//! Slice 2a (this version): rich `McuDescriptor` schema, G474 + H523 fully
//! populated from RM0440 / RM0481 + cross-checked against `stm32-metapac`'s
//! per-chip metadata. A peripheral-inventory view reads the descriptor and
//! works for both MCUs; the existing G474-shaped views still consume
//! `g474.rs` typed enums and migrate one-by-one in later slices.
//!
//! Authoritative sources:
//!  - G474 inventory/connectivity: `g474.rs` (RM0440)
//!  - H523 inventory: `metapac` `metadata_0393.rs` (RM0481, DS14540)
//!  - HRTIM crossbar/fabric: G474 only — `g474.rs` keeps the typed tables
//!    until a future slice migrates them in.

use crate::g474::PeripheralKind;
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
        match self { Self::G474 => &G474, Self::H523 => &H523 }
    }

    /// True when planning views (fabric, HRTIM, package) are populated for
    /// this MCU. H523 has the inventory but no AF table or solver hookup
    /// yet, so user-facing views are limited to what reads the descriptor
    /// directly.
    pub fn is_implemented(self) -> bool {
        match self { Self::G474 => true, Self::H523 => false }
    }

    pub fn default_g474_variant(self) -> ChipVariant {
        ChipVariant::G474R
    }
}

// ---------- Schema ----------

/// Coarse classification used by inventory views to group timers in a way
/// that matches the RM's chapter structure (advanced control / general
/// purpose / basic / low-power).
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
    /// Numeric instance, e.g. `1` for TIM1, `15` for TIM15, `1` for LPTIM1
    /// (disambiguated via `kind`).
    pub number: u8,
    pub kind: TimerKind,
    /// Number of capture/compare channels (4 for TIM1/8/2-5, 2 for TIM12/15,
    /// 1 for TIM16/17, 0 for basic/low-power).
    pub channels: u8,
    pub has_complementary: bool,
    /// Bit width of the counter — 32 for TIM2/5, 16 elsewhere.
    pub width_bits: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct AdcInstance {
    pub number: u8,
    /// Channels marked "fast" (low R_AIN) — important for cycle-by-cycle
    /// power-loop sampling. RM0440 §21 / RM0481 §38.
    pub fast_channels: &'static [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct DacInstance {
    pub number: u8,
    /// Number of output channels on this DAC (1 or 2).
    pub channels: u8,
    /// 15-Msps sample-and-hold "fast" DAC (G474 DAC3/DAC4); buffered output
    /// is internal-only. Always false on H523 (no fast DACs).
    pub fast: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct CompInstance { pub number: u8 }

#[derive(Copy, Clone, Debug)]
pub struct OpampInstance { pub number: u8 }

/// Communications-peripheral inventory. The lists are instance numbers
/// (e.g. `&[1, 2, 3, 4]` for SPI1..SPI4); empty `&[]` means "not present".
#[derive(Copy, Clone, Debug)]
pub struct CommsInventory {
    pub spi: &'static [u8],
    pub i2c: &'static [u8],
    /// I3C is H523-only; empty on G474.
    pub i3c: &'static [u8],
    pub usart: &'static [u8],
    pub uart: &'static [u8],
    pub lpuart: &'static [u8],
    pub fdcan: &'static [u8],
    pub ucpd: &'static [u8],
    pub has_usb: bool,
    /// 0 = none, 1 = OCTOSPI1, 2 = OCTOSPI1+OCTOSPI2.
    pub octospi: u8,
    pub has_sdmmc: bool,
    pub has_fmc: bool,
    pub has_hdmi_cec: bool,
}

/// Static cross-peripheral connectivity edge. Today the only edge kind used
/// is DAC→COMP analog routing (G474). Future kinds can be added as new
/// variants of `EdgeKind`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct PeripheralEdge {
    pub from: PeripheralPin,
    pub to: PeripheralPin,
    pub kind: EdgeKind,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum EdgeKind { DacToComp }

/// Disambiguating pin reference: peripheral instance + (optional) channel
/// number. For DAC, `channel` is 1 or 2; for single-channel peripherals
/// it's 0.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct PeripheralPin {
    pub kind: PeripheralKind,
    pub instance: u8,
    pub channel: u8,
}

const fn dac_pin(inst: u8, ch: u8) -> PeripheralPin {
    PeripheralPin { kind: PeripheralKind::Dac, instance: inst, channel: ch }
}
const fn comp_pin(inst: u8) -> PeripheralPin {
    PeripheralPin { kind: PeripheralKind::Comp, instance: inst, channel: 0 }
}

/// HRTIM crossbar-fabric description. `None` on MCUs without HRTIM (H523).
/// Today this struct just carries the structural counts (EEVs, faults,
/// triggers, sub-timers); the typed routing tables in `g474.rs` are still
/// the source of truth for the HRTIM tab. A later slice migrates those in.
pub struct HrtimFabric {
    pub sub_timer_count: u8,
    pub eev_count: u8,
    pub flt_count: u8,
    pub adc_trigger_count: u8,
}

pub struct McuDescriptor {
    pub name: &'static str,
    pub timers: &'static [TimerInstance],
    pub adcs: &'static [AdcInstance],
    pub dacs: &'static [DacInstance],
    pub comps: &'static [CompInstance],
    pub opamps: &'static [OpampInstance],
    pub comms: CommsInventory,
    pub edges: &'static [PeripheralEdge],
    pub hrtim: Option<&'static HrtimFabric>,
}

impl McuDescriptor {
    pub fn comps_for_dac(&self, dac_inst: u8, dac_ch: u8) -> impl Iterator<Item = u8> + '_ {
        self.edges.iter().filter_map(move |e| {
            (e.kind == EdgeKind::DacToComp
                && e.from.kind == PeripheralKind::Dac
                && e.from.instance == dac_inst
                && e.from.channel == dac_ch)
                .then_some(e.to.instance)
        })
    }
}

// ---------- G474 (RM0440) ----------

const G474_TIMERS: &[TimerInstance] = &[
    TimerInstance { number: 1,  kind: TimerKind::Advanced,  channels: 4, has_complementary: true,  width_bits: 16 },
    TimerInstance { number: 8,  kind: TimerKind::Advanced,  channels: 4, has_complementary: true,  width_bits: 16 },
    TimerInstance { number: 20, kind: TimerKind::Advanced,  channels: 4, has_complementary: true,  width_bits: 16 },
    TimerInstance { number: 2,  kind: TimerKind::General32, channels: 4, has_complementary: false, width_bits: 32 },
    TimerInstance { number: 5,  kind: TimerKind::General32, channels: 4, has_complementary: false, width_bits: 32 },
    TimerInstance { number: 3,  kind: TimerKind::General16, channels: 4, has_complementary: false, width_bits: 16 },
    TimerInstance { number: 4,  kind: TimerKind::General16, channels: 4, has_complementary: false, width_bits: 16 },
    TimerInstance { number: 15, kind: TimerKind::General16, channels: 2, has_complementary: true,  width_bits: 16 },
    TimerInstance { number: 16, kind: TimerKind::General16, channels: 1, has_complementary: true,  width_bits: 16 },
    TimerInstance { number: 17, kind: TimerKind::General16, channels: 1, has_complementary: true,  width_bits: 16 },
    TimerInstance { number: 6,  kind: TimerKind::Basic,     channels: 0, has_complementary: false, width_bits: 16 },
    TimerInstance { number: 7,  kind: TimerKind::Basic,     channels: 0, has_complementary: false, width_bits: 16 },
];

// G474 fast-channel rule: per DS12288 footnote, ADCx_IN1..IN5 are fast on
// every ADC. Encoded uniformly here.
const G474_FAST_ADC: &[u8] = &[1, 2, 3, 4, 5];
const G474_ADCS: &[AdcInstance] = &[
    AdcInstance { number: 1, fast_channels: G474_FAST_ADC },
    AdcInstance { number: 2, fast_channels: G474_FAST_ADC },
    AdcInstance { number: 3, fast_channels: G474_FAST_ADC },
    AdcInstance { number: 4, fast_channels: G474_FAST_ADC },
    AdcInstance { number: 5, fast_channels: G474_FAST_ADC },
];

const G474_DACS: &[DacInstance] = &[
    DacInstance { number: 1, channels: 2, fast: false },
    DacInstance { number: 2, channels: 1, fast: false },
    DacInstance { number: 3, channels: 2, fast: true },
    DacInstance { number: 4, channels: 2, fast: true },
];

const G474_COMPS: &[CompInstance] = &[
    CompInstance { number: 1 }, CompInstance { number: 2 },
    CompInstance { number: 3 }, CompInstance { number: 4 },
    CompInstance { number: 5 }, CompInstance { number: 6 },
    CompInstance { number: 7 },
];

const G474_OPAMPS: &[OpampInstance] = &[
    OpampInstance { number: 1 }, OpampInstance { number: 2 },
    OpampInstance { number: 3 }, OpampInstance { number: 4 },
    OpampInstance { number: 5 }, OpampInstance { number: 6 },
];

/// DAC→COMP routing per RM0440 §25 / DAC connectivity table (mirrors
/// `g474::DAC_TO_COMP`).
const G474_EDGES: &[PeripheralEdge] = &[
    // DAC1 channel 1 → COMP1, COMP3, COMP4
    PeripheralEdge { from: dac_pin(1, 1), to: comp_pin(1), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_pin(1, 1), to: comp_pin(3), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_pin(1, 1), to: comp_pin(4), kind: EdgeKind::DacToComp },
    // DAC1 channel 2 → COMP2, COMP5
    PeripheralEdge { from: dac_pin(1, 2), to: comp_pin(2), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_pin(1, 2), to: comp_pin(5), kind: EdgeKind::DacToComp },
    // DAC2 channel 1 → COMP6, COMP7
    PeripheralEdge { from: dac_pin(2, 1), to: comp_pin(6), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_pin(2, 1), to: comp_pin(7), kind: EdgeKind::DacToComp },
    // DAC3 channel 1 → COMP1, COMP3
    PeripheralEdge { from: dac_pin(3, 1), to: comp_pin(1), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_pin(3, 1), to: comp_pin(3), kind: EdgeKind::DacToComp },
    // DAC3 channel 2 → COMP2, COMP4
    PeripheralEdge { from: dac_pin(3, 2), to: comp_pin(2), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_pin(3, 2), to: comp_pin(4), kind: EdgeKind::DacToComp },
    // DAC4 channel 1 → COMP5, COMP7
    PeripheralEdge { from: dac_pin(4, 1), to: comp_pin(5), kind: EdgeKind::DacToComp },
    PeripheralEdge { from: dac_pin(4, 1), to: comp_pin(7), kind: EdgeKind::DacToComp },
    // DAC4 channel 2 → COMP6
    PeripheralEdge { from: dac_pin(4, 2), to: comp_pin(6), kind: EdgeKind::DacToComp },
];

const G474_HRTIM: HrtimFabric = HrtimFabric {
    sub_timer_count: 6, // TIMA..TIMF
    eev_count: 10,
    flt_count: 6,
    adc_trigger_count: 10,
};

pub const G474: McuDescriptor = McuDescriptor {
    name: "STM32G474",
    timers: G474_TIMERS,
    adcs: G474_ADCS,
    dacs: G474_DACS,
    comps: G474_COMPS,
    opamps: G474_OPAMPS,
    comms: CommsInventory {
        spi: &[1, 2, 3],
        i2c: &[1, 2, 3, 4],
        i3c: &[],
        usart: &[1, 2, 3],
        uart: &[4, 5],
        lpuart: &[1],
        fdcan: &[1, 2, 3],
        ucpd: &[1],
        has_usb: true,
        octospi: 0,
        has_sdmmc: false,
        has_fmc: true, // G474 has FMC on larger packages; flag is package-agnostic
        has_hdmi_cec: false,
    },
    edges: G474_EDGES,
    hrtim: Some(&G474_HRTIM),
};

// ---------- H523 (RM0481, DS14540, cross-checked vs metapac H523RE) ----------

const H523_TIMERS: &[TimerInstance] = &[
    TimerInstance { number: 1,  kind: TimerKind::Advanced,  channels: 4, has_complementary: true,  width_bits: 16 },
    TimerInstance { number: 8,  kind: TimerKind::Advanced,  channels: 4, has_complementary: true,  width_bits: 16 },
    TimerInstance { number: 2,  kind: TimerKind::General32, channels: 4, has_complementary: false, width_bits: 32 },
    TimerInstance { number: 5,  kind: TimerKind::General32, channels: 4, has_complementary: false, width_bits: 32 },
    TimerInstance { number: 3,  kind: TimerKind::General16, channels: 4, has_complementary: false, width_bits: 16 },
    TimerInstance { number: 4,  kind: TimerKind::General16, channels: 4, has_complementary: false, width_bits: 16 },
    TimerInstance { number: 12, kind: TimerKind::General16, channels: 2, has_complementary: false, width_bits: 16 },
    TimerInstance { number: 15, kind: TimerKind::General16, channels: 2, has_complementary: true,  width_bits: 16 },
    TimerInstance { number: 6,  kind: TimerKind::Basic,     channels: 0, has_complementary: false, width_bits: 16 },
    TimerInstance { number: 7,  kind: TimerKind::Basic,     channels: 0, has_complementary: false, width_bits: 16 },
    TimerInstance { number: 1,  kind: TimerKind::LowPower,  channels: 2, has_complementary: false, width_bits: 16 },
    TimerInstance { number: 2,  kind: TimerKind::LowPower,  channels: 2, has_complementary: false, width_bits: 16 },
];

// H523 fast-channel rule: TBD — needs cross-check against DS14540 §5.3.22
// (12-bit ADC characteristics) and the AF/pin table to identify which
// channels have low R_AIN. Empty for now; populate before solver migration.
const H523_FAST_ADC: &[u8] = &[];
const H523_ADCS: &[AdcInstance] = &[
    AdcInstance { number: 1, fast_channels: H523_FAST_ADC },
    AdcInstance { number: 2, fast_channels: H523_FAST_ADC },
];

const H523_DACS: &[DacInstance] = &[
    DacInstance { number: 1, channels: 2, fast: false },
];

pub const H523: McuDescriptor = McuDescriptor {
    name: "STM32H523",
    timers: H523_TIMERS,
    adcs: H523_ADCS,
    dacs: H523_DACS,
    comps: &[],   // H523 has no comparators
    opamps: &[],  // H523 has no opamps
    comms: CommsInventory {
        spi: &[1, 2, 3, 4],
        i2c: &[1, 2, 3],
        i3c: &[1, 2],
        usart: &[1, 2, 3, 6],
        uart: &[4, 5],
        lpuart: &[1],
        fdcan: &[1, 2],
        ucpd: &[1],
        has_usb: true,
        octospi: 1,
        has_sdmmc: true,
        has_fmc: true,
        has_hdmi_cec: true,
    },
    edges: &[],   // No DAC→COMP wiring (no COMPs)
    hrtim: None,  // No HRTIM
};
