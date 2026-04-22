//! STM32G474 pin / alternate-function table for layout-planning.
//!
//! Scope: all STM32G474 chip variants (part numbers), HRTIM signals only
//! (channels, EEVs, FLTs, sync). Data source: cross-checked against
//! `stm32-metapac 19.0.0` per-variant metadata (which itself comes from
//! ST's CubeMX XML). Grouped by "pinout class" — within G474 the HRTIM
//! signals split cleanly into the C-class (fewest PC-port pins) and the
//! rest.
//!
//! Analog signals (DAC_OUT, COMP_OUT, OPAMP, ADC inputs) are datasheet
//! "additional functions" (not AF-mappable); added in a follow-up pass.

use crate::g474::*;

/// STM32G474 chip pinout classes. Only the part-number letter after
/// "G474" matters for pinout; the trailing flash-size letter (B/C/E for
/// 128/256/512 KiB) is orthogonal and not tracked here.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum ChipVariant {
    G474C, // LQFP48
    G474M, // WLCSP81
    G474P, // TFBGA100
    G474Q, // UFBGA121
    G474R, // LQFP64
    G474V, // LQFP100
}

impl ChipVariant {
    pub const ALL: &'static [ChipVariant] = &[
        ChipVariant::G474C,
        ChipVariant::G474M,
        ChipVariant::G474P,
        ChipVariant::G474Q,
        ChipVariant::G474R,
        ChipVariant::G474V,
    ];

    pub fn part_family(self) -> &'static str {
        match self {
            Self::G474C => "STM32G474C",
            Self::G474M => "STM32G474M",
            Self::G474P => "STM32G474P",
            Self::G474Q => "STM32G474Q",
            Self::G474R => "STM32G474R",
            Self::G474V => "STM32G474V",
        }
    }

    /// Nominal package per ST's part-number letter convention.
    pub fn package(self) -> &'static str {
        match self {
            Self::G474C => "LQFP48",
            Self::G474M => "WLCSP81",
            Self::G474P => "TFBGA100",
            Self::G474Q => "UFBGA121",
            Self::G474R => "LQFP64",
            Self::G474V => "LQFP100",
        }
    }

    pub fn display_label(self) -> String {
        format!("{}x ({})", self.part_family(), self.package())
    }

    const fn bit(self) -> u32 {
        1u32 << self as u8
    }
}

const ALL_V: u32 = {
    ChipVariant::G474C.bit()
        | ChipVariant::G474M.bit()
        | ChipVariant::G474P.bit()
        | ChipVariant::G474Q.bit()
        | ChipVariant::G474R.bit()
        | ChipVariant::G474V.bit()
};
/// Everything except the C class. Per metapac, the C-class is the only
/// G474 pinout that drops several PC-port HRTIM pins; M/P/Q/R/V all carry
/// the full PC-port HRTIM pin set.
const NON_C: u32 = ALL_V & !ChipVariant::G474C.bit();
/// PC pins kept on the C class (PC6, PC10, PC11).
const PC_KEPT_ON_C: u32 = ALL_V;

// Extra package-subset masks used by the generated Comms/Timers block
// below. Kept alongside NON_C so the AF table stays dense and scannable.
/// Pins that land on P/Q/V only (typical for PE*/PF* pins that the LQFP64
/// and LQFP48 packages don't break out, but WLCSP81 also lacks).
const LARGE_PQV: u32 =
    ChipVariant::G474P.bit() | ChipVariant::G474Q.bit() | ChipVariant::G474V.bit();
/// BGA-only pins (TFBGA100 + UFBGA121). LQFP variants don't break these out.
const BGA_PQ: u32 = ChipVariant::G474P.bit() | ChipVariant::G474Q.bit();
/// Everything except the two LQFP-only classes (C = LQFP48, R = LQFP64).
const NON_CR: u32 = ChipVariant::G474M.bit()
    | ChipVariant::G474P.bit()
    | ChipVariant::G474Q.bit()
    | ChipVariant::G474V.bit();
/// Pins that only the UFBGA121 (Q) exposes — typically PG-port pins.
const BGA_Q: u32 = ChipVariant::G474Q.bit();

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Ord, PartialOrd, serde::Serialize, serde::Deserialize)]
pub struct Pin {
    pub port: char,
    pub num: u8,
}

impl Pin {
    pub const fn new(port: char, num: u8) -> Self { Self { port, num } }
    pub fn name(self) -> String { format!("P{}{}", self.port, self.num) }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Signal {
    HrtimChannel { timer: HrtimId, ch: HrtimCh },
    HrtimEev(CrossbarSource),
    HrtimFlt(HrtimFltId),
    HrtimScin,
    HrtimScout,
    CompInp(CompId),
    CompInm(CompId),
    CompOut(CompId),
    DacOut(DacId),
    AdcIn { adc: AdcInstance, channel: u8 },
    OpampVinp(OpampId),
    OpampVinm(OpampId),
    OpampVout(OpampId),
    // ---- Comms ----
    SpiMosi(SpiId),
    SpiMiso(SpiId),
    SpiSck(SpiId),
    SpiNss(SpiId),
    I2cSda(I2cId),
    I2cScl(I2cId),
    I2cSmba(I2cId),
    UsartTx(UsartId),
    UsartRx(UsartId),
    UsartCts(UsartId),
    UsartRts(UsartId),
    UsartCk(UsartId),
    UartTx(UartId),
    UartRx(UartId),
    UartCts(UartId),
    UartRts(UartId),
    LpuartTx(LpuartId),
    LpuartRx(LpuartId),
    LpuartCts(LpuartId),
    LpuartRts(LpuartId),
    CanTx(CanId),
    CanRx(CanId),
    UsbDp,
    UsbDm,
    UcpdCc1(UcpdId),
    UcpdCc2(UcpdId),
    // ---- Timers ----
    TimCh(TimId, TimCh),
    TimChN(TimId, TimCh),
    TimBkin(TimId),
    TimBkin2(TimId),
    TimEtr(TimId),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum HrtimCh { Ch1, Ch2 }

impl Signal {
    pub fn name(self) -> String {
        match self {
            Self::HrtimChannel { timer, ch } => {
                let t = match timer {
                    HrtimId::TimA => "A", HrtimId::TimB => "B", HrtimId::TimC => "C",
                    HrtimId::TimD => "D", HrtimId::TimE => "E", HrtimId::TimF => "F",
                };
                let c = match ch { HrtimCh::Ch1 => "1", HrtimCh::Ch2 => "2" };
                format!("HRTIM_CH{}{}", t, c)
            }
            Self::HrtimEev(e) => format!("HRTIM_{:?}", e).to_uppercase(),
            Self::HrtimFlt(f) => format!("HRTIM_{:?}", f).to_uppercase(),
            Self::HrtimScin => "HRTIM_SCIN".to_string(),
            Self::HrtimScout => "HRTIM_SCOUT".to_string(),
            Self::CompInp(c) => format!("{:?}_INP", c).to_uppercase(),
            Self::CompInm(c) => format!("{:?}_INM", c).to_uppercase(),
            Self::CompOut(c) => format!("{:?}_OUT", c).to_uppercase(),
            Self::DacOut(d) => {
                let (n, ch) = match d {
                    DacId::Dac1Ch1 => (1, 1),
                    DacId::Dac1Ch2 => (1, 2),
                    DacId::Dac2Ch1 => (2, 1),
                    DacId::Dac3Ch1 => (3, 1),
                    DacId::Dac3Ch2 => (3, 2),
                    DacId::Dac4Ch1 => (4, 1),
                    DacId::Dac4Ch2 => (4, 2),
                };
                format!("DAC{}_OUT{}", n, ch)
            }
            Self::AdcIn { adc, channel } => format!("ADC{}_IN{}", adc.number(), channel),
            Self::OpampVinp(o) => format!("OPAMP{}_VINP", o.number()),
            Self::OpampVinm(o) => format!("OPAMP{}_VINM", o.number()),
            Self::OpampVout(o) => format!("OPAMP{}_VOUT", o.number()),
            Self::SpiMosi(s) => format!("SPI{}_MOSI", s.number()),
            Self::SpiMiso(s) => format!("SPI{}_MISO", s.number()),
            Self::SpiSck(s) => format!("SPI{}_SCK", s.number()),
            Self::SpiNss(s) => format!("SPI{}_NSS", s.number()),
            Self::I2cSda(i) => format!("I2C{}_SDA", i.number()),
            Self::I2cScl(i) => format!("I2C{}_SCL", i.number()),
            Self::I2cSmba(i) => format!("I2C{}_SMBA", i.number()),
            Self::UsartTx(u) => format!("USART{}_TX", u.number()),
            Self::UsartRx(u) => format!("USART{}_RX", u.number()),
            Self::UsartCts(u) => format!("USART{}_CTS", u.number()),
            Self::UsartRts(u) => format!("USART{}_RTS", u.number()),
            Self::UsartCk(u) => format!("USART{}_CK", u.number()),
            Self::UartTx(u) => format!("UART{}_TX", u.number()),
            Self::UartRx(u) => format!("UART{}_RX", u.number()),
            Self::UartCts(u) => format!("UART{}_CTS", u.number()),
            Self::UartRts(u) => format!("UART{}_RTS", u.number()),
            Self::LpuartTx(_) => "LPUART1_TX".to_string(),
            Self::LpuartRx(_) => "LPUART1_RX".to_string(),
            Self::LpuartCts(_) => "LPUART1_CTS".to_string(),
            Self::LpuartRts(_) => "LPUART1_RTS".to_string(),
            Self::CanTx(c) => format!("FDCAN{}_TX", c.number()),
            Self::CanRx(c) => format!("FDCAN{}_RX", c.number()),
            Self::UsbDp => "USB_DP".to_string(),
            Self::UsbDm => "USB_DM".to_string(),
            Self::UcpdCc1(_) => "UCPD1_CC1".to_string(),
            Self::UcpdCc2(_) => "UCPD1_CC2".to_string(),
            Self::TimCh(t, ch) => format!("TIM{}_CH{}", t.number(), ch.number()),
            Self::TimChN(t, ch) => format!("TIM{}_CH{}N", t.number(), ch.number()),
            Self::TimBkin(t) => format!("TIM{}_BKIN", t.number()),
            Self::TimBkin2(t) => format!("TIM{}_BKIN2", t.number()),
            Self::TimEtr(t) => format!("TIM{}_ETR", t.number()),
        }
    }
}

pub struct AfOption {
    pub pin: Pin,
    pub af: u8,
    pub signal: Signal,
    pub variants: u32,
}

macro_rules! af {
    ($port:expr, $num:expr, $af:expr, $signal:expr, $vs:expr) => {
        AfOption { pin: Pin::new($port, $num), af: $af, signal: $signal, variants: $vs }
    };
}

/// HRTIM AF table. Cross-checked against stm32-metapac 19.0.0 — 34 entries
/// match exactly; 1 (PB1 SCOUT AF13, not AF12 as the datasheet AF-table row
/// appears) was fixed during cross-check.
const HRTIM_AF: &[AfOption] = &[
    // HRTIM channels
    af!('A', 8,  13, Signal::HrtimChannel { timer: HrtimId::TimA, ch: HrtimCh::Ch1 }, ALL_V),
    af!('A', 9,  13, Signal::HrtimChannel { timer: HrtimId::TimA, ch: HrtimCh::Ch2 }, ALL_V),
    af!('A', 10, 13, Signal::HrtimChannel { timer: HrtimId::TimB, ch: HrtimCh::Ch1 }, ALL_V),
    af!('A', 11, 13, Signal::HrtimChannel { timer: HrtimId::TimB, ch: HrtimCh::Ch2 }, ALL_V),
    af!('B', 12, 13, Signal::HrtimChannel { timer: HrtimId::TimC, ch: HrtimCh::Ch1 }, ALL_V),
    af!('B', 13, 13, Signal::HrtimChannel { timer: HrtimId::TimC, ch: HrtimCh::Ch2 }, ALL_V),
    af!('B', 14, 13, Signal::HrtimChannel { timer: HrtimId::TimD, ch: HrtimCh::Ch1 }, ALL_V),
    af!('B', 15, 13, Signal::HrtimChannel { timer: HrtimId::TimD, ch: HrtimCh::Ch2 }, ALL_V),
    af!('C', 8,  3,  Signal::HrtimChannel { timer: HrtimId::TimE, ch: HrtimCh::Ch1 }, NON_C),
    af!('C', 9,  3,  Signal::HrtimChannel { timer: HrtimId::TimE, ch: HrtimCh::Ch2 }, NON_C),
    af!('C', 6,  13, Signal::HrtimChannel { timer: HrtimId::TimF, ch: HrtimCh::Ch1 }, PC_KEPT_ON_C),
    af!('C', 7,  13, Signal::HrtimChannel { timer: HrtimId::TimF, ch: HrtimCh::Ch2 }, NON_C),
    // FLT inputs
    af!('A', 12, 13, Signal::HrtimFlt(HrtimFltId::Flt1), ALL_V),
    af!('A', 15, 13, Signal::HrtimFlt(HrtimFltId::Flt2), ALL_V),
    af!('B', 10, 13, Signal::HrtimFlt(HrtimFltId::Flt3), ALL_V),
    af!('B', 11, 13, Signal::HrtimFlt(HrtimFltId::Flt4), ALL_V),
    af!('B', 0,  13, Signal::HrtimFlt(HrtimFltId::Flt5), ALL_V),
    af!('C', 7,  3,  Signal::HrtimFlt(HrtimFltId::Flt5), NON_C), // alt
    af!('C', 10, 13, Signal::HrtimFlt(HrtimFltId::Flt6), PC_KEPT_ON_C),
    // EEV inputs
    af!('C', 12, 3,  Signal::HrtimEev(CrossbarSource::Eev1),  NON_C),
    af!('C', 11, 3,  Signal::HrtimEev(CrossbarSource::Eev2),  PC_KEPT_ON_C),
    af!('B', 7,  13, Signal::HrtimEev(CrossbarSource::Eev3),  ALL_V),
    af!('B', 6,  13, Signal::HrtimEev(CrossbarSource::Eev4),  ALL_V),
    af!('B', 9,  13, Signal::HrtimEev(CrossbarSource::Eev5),  ALL_V),
    af!('B', 5,  13, Signal::HrtimEev(CrossbarSource::Eev6),  ALL_V),
    af!('B', 4,  13, Signal::HrtimEev(CrossbarSource::Eev7),  ALL_V),
    af!('B', 8,  13, Signal::HrtimEev(CrossbarSource::Eev8),  ALL_V),
    af!('B', 3,  13, Signal::HrtimEev(CrossbarSource::Eev9),  ALL_V),
    af!('C', 5,  13, Signal::HrtimEev(CrossbarSource::Eev10), NON_C),
    af!('C', 6,  3,  Signal::HrtimEev(CrossbarSource::Eev10), PC_KEPT_ON_C), // alt
    // Sync
    af!('B', 1,  13, Signal::HrtimScout, ALL_V),
    af!('B', 3,  12, Signal::HrtimScout, ALL_V), // alt (conflicts w/ EEV9)
    af!('B', 2,  13, Signal::HrtimScin,  ALL_V),
    af!('B', 6,  12, Signal::HrtimScin,  ALL_V), // alt (conflicts w/ EEV4)

    // ---- Analog: COMP inputs / outputs, DAC outputs ----
    //
    // AF==0 marker for pure-analog pins (no GPIO_AFR slot; peripheral
    // register enables the pin). Cross-checked against stm32-metapac.
    // Package availability for these: low-port pins (PA*, PB*) are on all
    // variants; where metapac lists a pin on one class and not another,
    // mark NON_C accordingly.

    // DAC outputs (only slow DACs have pins; DAC3/DAC4 are internal-only)
    af!('A', 4, 0, Signal::DacOut(DacId::Dac1Ch1), ALL_V),
    af!('A', 5, 0, Signal::DacOut(DacId::Dac1Ch2), ALL_V),
    af!('A', 6, 0, Signal::DacOut(DacId::Dac2Ch1), ALL_V),

    // COMP1 inputs / output
    af!('A', 1, 0, Signal::CompInp(CompId::Comp1), ALL_V),
    af!('B', 1, 0, Signal::CompInp(CompId::Comp1), ALL_V),
    af!('A', 0, 0, Signal::CompInm(CompId::Comp1), ALL_V),
    af!('A', 4, 0, Signal::CompInm(CompId::Comp1), ALL_V),
    af!('A', 0,  8, Signal::CompOut(CompId::Comp1), ALL_V),
    af!('A', 6,  8, Signal::CompOut(CompId::Comp1), ALL_V),
    af!('A', 11, 8, Signal::CompOut(CompId::Comp1), ALL_V),
    af!('B', 8,  8, Signal::CompOut(CompId::Comp1), ALL_V),

    // COMP2 inputs / output
    af!('A', 3, 0, Signal::CompInp(CompId::Comp2), ALL_V),
    af!('A', 7, 0, Signal::CompInp(CompId::Comp2), ALL_V),
    af!('A', 2, 0, Signal::CompInm(CompId::Comp2), ALL_V),
    af!('A', 5, 0, Signal::CompInm(CompId::Comp2), ALL_V),
    af!('A', 2,  8, Signal::CompOut(CompId::Comp2), ALL_V),
    af!('A', 7,  8, Signal::CompOut(CompId::Comp2), ALL_V),
    af!('A', 12, 8, Signal::CompOut(CompId::Comp2), ALL_V),
    af!('B', 9,  8, Signal::CompOut(CompId::Comp2), ALL_V),

    // COMP3 inputs / output
    af!('B', 14, 0, Signal::CompInp(CompId::Comp3), ALL_V),
    af!('C', 1,  0, Signal::CompInp(CompId::Comp3), PC_KEPT_ON_C),
    af!('C', 0,  0, Signal::CompInm(CompId::Comp3), PC_KEPT_ON_C),
    af!('F', 1,  0, Signal::CompInm(CompId::Comp3), ALL_V),
    af!('B', 7,  8, Signal::CompOut(CompId::Comp3), ALL_V),
    af!('B', 15, 3, Signal::CompOut(CompId::Comp3), ALL_V),

    // COMP4 inputs / output
    af!('B', 0,  0, Signal::CompInp(CompId::Comp4), ALL_V),
    af!('E', 7,  0, Signal::CompInp(CompId::Comp4), NON_C),
    af!('B', 2,  0, Signal::CompInm(CompId::Comp4), ALL_V),
    af!('E', 8,  0, Signal::CompInm(CompId::Comp4), NON_C),
    af!('B', 1,  8, Signal::CompOut(CompId::Comp4), ALL_V),
    af!('B', 6,  8, Signal::CompOut(CompId::Comp4), ALL_V),
    af!('B', 14, 8, Signal::CompOut(CompId::Comp4), ALL_V),

    // COMP5 inputs / output (not all variants have this)
    af!('B', 13, 0, Signal::CompInp(CompId::Comp5), ALL_V),
    af!('D', 12, 0, Signal::CompInp(CompId::Comp5), NON_C),
    af!('B', 10, 0, Signal::CompInm(CompId::Comp5), ALL_V),
    af!('D', 13, 0, Signal::CompInm(CompId::Comp5), NON_C),
    af!('A', 9,  8, Signal::CompOut(CompId::Comp5), ALL_V),
    af!('C', 7,  7, Signal::CompOut(CompId::Comp5), NON_C),

    // COMP6 inputs / output
    af!('B', 11, 0, Signal::CompInp(CompId::Comp6), ALL_V),
    af!('D', 11, 0, Signal::CompInp(CompId::Comp6), NON_C),
    af!('B', 15, 0, Signal::CompInm(CompId::Comp6), ALL_V),
    af!('D', 10, 0, Signal::CompInm(CompId::Comp6), NON_C),
    af!('A', 10, 8, Signal::CompOut(CompId::Comp6), ALL_V),
    af!('C', 6,  7, Signal::CompOut(CompId::Comp6), PC_KEPT_ON_C),

    // COMP7 inputs / output
    af!('B', 14, 0, Signal::CompInp(CompId::Comp7), ALL_V),
    af!('B', 12, 0, Signal::CompInm(CompId::Comp7), ALL_V),
    af!('A', 8,  8, Signal::CompOut(CompId::Comp7), ALL_V),
    af!('C', 8,  7, Signal::CompOut(CompId::Comp7), NON_C),

    // ---- ADC single-ended input pins (ADC1..5) ----
    // Extracted from stm32-metapac; each ADC-channel-number has a
    // specific GPIO pin on each ADC instance.

    // ADC1 (14 inputs — channel-13 routed from internal VREFINT+, not a pin)
    af!('A', 0, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 1 }, ALL_V),
    af!('A', 1, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 2 }, ALL_V),
    af!('A', 2, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 3 }, ALL_V),
    af!('A', 3, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 4 }, ALL_V),
    af!('B', 14, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 5 }, ALL_V),
    af!('C', 0, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 6 }, PC_KEPT_ON_C),
    af!('C', 1, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 7 }, PC_KEPT_ON_C),
    af!('C', 2, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 8 }, PC_KEPT_ON_C),
    af!('C', 3, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 9 }, PC_KEPT_ON_C),
    af!('F', 0, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 10 }, ALL_V),
    af!('B', 12, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 11 }, ALL_V),
    af!('B', 1, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 12 }, ALL_V),
    af!('B', 11, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 14 }, ALL_V),
    af!('B', 0, 0, Signal::AdcIn { adc: AdcInstance::Adc1, channel: 15 }, ALL_V),

    // ADC2 (16 inputs)
    af!('A', 0, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 1 }, ALL_V),
    af!('A', 1, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 2 }, ALL_V),
    af!('A', 6, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 3 }, ALL_V),
    af!('A', 7, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 4 }, ALL_V),
    af!('C', 4, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 5 }, NON_C),
    af!('C', 0, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 6 }, PC_KEPT_ON_C),
    af!('C', 1, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 7 }, PC_KEPT_ON_C),
    af!('C', 2, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 8 }, PC_KEPT_ON_C),
    af!('C', 3, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 9 }, PC_KEPT_ON_C),
    af!('F', 1, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 10 }, ALL_V),
    af!('C', 5, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 11 }, NON_C),
    af!('B', 2, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 12 }, ALL_V),
    af!('A', 5, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 13 }, ALL_V),
    af!('B', 11, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 14 }, ALL_V),
    af!('B', 15, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 15 }, ALL_V),
    af!('A', 4, 0, Signal::AdcIn { adc: AdcInstance::Adc2, channel: 17 }, ALL_V),

    // ADC3 (3 external inputs)
    af!('B', 1, 0, Signal::AdcIn { adc: AdcInstance::Adc3, channel: 1 }, ALL_V),
    af!('B', 13, 0, Signal::AdcIn { adc: AdcInstance::Adc3, channel: 5 }, ALL_V),
    af!('B', 0, 0, Signal::AdcIn { adc: AdcInstance::Adc3, channel: 12 }, ALL_V),

    // ADC4 (3 external inputs)
    af!('B', 12, 0, Signal::AdcIn { adc: AdcInstance::Adc4, channel: 3 }, ALL_V),
    af!('B', 14, 0, Signal::AdcIn { adc: AdcInstance::Adc4, channel: 4 }, ALL_V),
    af!('B', 15, 0, Signal::AdcIn { adc: AdcInstance::Adc4, channel: 5 }, ALL_V),

    // ADC5 (2 external inputs)
    af!('A', 8, 0, Signal::AdcIn { adc: AdcInstance::Adc5, channel: 1 }, ALL_V),
    af!('A', 9, 0, Signal::AdcIn { adc: AdcInstance::Adc5, channel: 2 }, ALL_V),

    // ---- OPAMP inputs / outputs ----
    //
    // Sourced from stm32-metapac 19.0.0 for STM32G474RE (LQFP64).
    // Each OPAMP has up to 3 VINP options, 1-2 VINM options, 1 VOUT.
    // Signals treated as analog "additional functions" (AF=0) matching the
    // existing COMP/DAC/ADC handling in this table. PC5 is not on LQFP48
    // (C class); PD*/PE* VINP options on larger packages aren't added here
    // pending a full per-package pass.

    // OPAMP1
    af!('A', 1, 0, Signal::OpampVinp(OpampId::Opamp1), ALL_V),
    af!('A', 3, 0, Signal::OpampVinp(OpampId::Opamp1), ALL_V),
    af!('A', 7, 0, Signal::OpampVinp(OpampId::Opamp1), ALL_V),
    af!('A', 3, 0, Signal::OpampVinm(OpampId::Opamp1), ALL_V),
    af!('C', 5, 0, Signal::OpampVinm(OpampId::Opamp1), NON_C),
    af!('A', 2, 0, Signal::OpampVout(OpampId::Opamp1), ALL_V),

    // OPAMP2
    af!('A', 7, 0, Signal::OpampVinp(OpampId::Opamp2), ALL_V),
    af!('B', 0, 0, Signal::OpampVinp(OpampId::Opamp2), ALL_V),
    af!('B', 14, 0, Signal::OpampVinp(OpampId::Opamp2), ALL_V),
    af!('A', 5, 0, Signal::OpampVinm(OpampId::Opamp2), ALL_V),
    af!('C', 5, 0, Signal::OpampVinm(OpampId::Opamp2), NON_C),
    af!('A', 6, 0, Signal::OpampVout(OpampId::Opamp2), ALL_V),

    // OPAMP3
    af!('A', 1, 0, Signal::OpampVinp(OpampId::Opamp3), ALL_V),
    af!('B', 0, 0, Signal::OpampVinp(OpampId::Opamp3), ALL_V),
    af!('B', 13, 0, Signal::OpampVinp(OpampId::Opamp3), ALL_V),
    af!('B', 2, 0, Signal::OpampVinm(OpampId::Opamp3), ALL_V),
    af!('B', 10, 0, Signal::OpampVinm(OpampId::Opamp3), ALL_V),
    af!('B', 1, 0, Signal::OpampVout(OpampId::Opamp3), ALL_V),

    // OPAMP4
    af!('B', 11, 0, Signal::OpampVinp(OpampId::Opamp4), ALL_V),
    af!('B', 13, 0, Signal::OpampVinp(OpampId::Opamp4), ALL_V),
    af!('B', 10, 0, Signal::OpampVinm(OpampId::Opamp4), ALL_V),
    af!('B', 12, 0, Signal::OpampVout(OpampId::Opamp4), ALL_V),

    // OPAMP5
    af!('B', 14, 0, Signal::OpampVinp(OpampId::Opamp5), ALL_V),
    af!('C', 3, 0, Signal::OpampVinp(OpampId::Opamp5), PC_KEPT_ON_C),
    af!('A', 3, 0, Signal::OpampVinm(OpampId::Opamp5), ALL_V),
    af!('B', 15, 0, Signal::OpampVinm(OpampId::Opamp5), ALL_V),
    af!('A', 8, 0, Signal::OpampVout(OpampId::Opamp5), ALL_V),

    // OPAMP6
    af!('B', 12, 0, Signal::OpampVinp(OpampId::Opamp6), ALL_V),
    af!('B', 13, 0, Signal::OpampVinp(OpampId::Opamp6), ALL_V),
    af!('A', 1, 0, Signal::OpampVinm(OpampId::Opamp6), ALL_V),
    af!('B', 1, 0, Signal::OpampVinm(OpampId::Opamp6), ALL_V),
    af!('B', 11, 0, Signal::OpampVout(OpampId::Opamp6), ALL_V),

    // ---- Comms + Timers (SPI / I2C / (L)(US)ART / FDCAN / TIM / USB / UCPD) ----
    //
    // Generated per-package from stm32-metapac 19.0.0 across all six
    // G474 pinout classes (C/M/P/Q/R/V). Each entry's variant bitmask
    // reflects which packages actually expose that pin — LQFP48 drops
    // most PC-port pins and all PD/PE/PF/PG, WLCSP81 drops PE/PF/PG,
    // LQFP64 drops PE/PF/PG, LQFP100 keeps PD/PE but drops PF high pins
    // and PG, UFBGA121 is the only package with the full PG-port set.
    //
    // Deduplicated: USART DE aliases RTS pin; SPI I2S_* aliases skipped;
    // UCPD DBCCx / FRSTXx not modeled yet.
    // FDCAN1
    af!('A', 11,  9, Signal::CanRx(CanId::Fdcan1), ALL_V),
    af!('A', 12,  9, Signal::CanTx(CanId::Fdcan1), ALL_V),
    af!('B',  8,  9, Signal::CanRx(CanId::Fdcan1), ALL_V),
    af!('B',  9,  9, Signal::CanTx(CanId::Fdcan1), ALL_V),
    af!('D',  0,  9, Signal::CanRx(CanId::Fdcan1), NON_CR),
    af!('D',  1,  9, Signal::CanTx(CanId::Fdcan1), NON_CR),
    // FDCAN2
    af!('B', 12,  9, Signal::CanRx(CanId::Fdcan2), ALL_V),
    af!('B', 13,  9, Signal::CanTx(CanId::Fdcan2), ALL_V),
    af!('B',  5,  9, Signal::CanRx(CanId::Fdcan2), ALL_V),
    af!('B',  6,  9, Signal::CanTx(CanId::Fdcan2), ALL_V),
    // FDCAN3
    af!('A', 15, 11, Signal::CanTx(CanId::Fdcan3), ALL_V),
    af!('A',  8, 11, Signal::CanRx(CanId::Fdcan3), ALL_V),
    af!('B',  3, 11, Signal::CanRx(CanId::Fdcan3), ALL_V),
    af!('B',  4, 11, Signal::CanTx(CanId::Fdcan3), ALL_V),
    // I2C1
    af!('A', 13,  4, Signal::I2cScl(I2cId::I2c1), ALL_V),
    af!('A', 14,  4, Signal::I2cSda(I2cId::I2c1), ALL_V),
    af!('A', 15,  4, Signal::I2cScl(I2cId::I2c1), ALL_V),
    af!('B',  5,  4, Signal::I2cSmba(I2cId::I2c1), ALL_V),
    af!('B',  7,  4, Signal::I2cSda(I2cId::I2c1), ALL_V),
    af!('B',  8,  4, Signal::I2cScl(I2cId::I2c1), ALL_V),
    af!('B',  9,  4, Signal::I2cSda(I2cId::I2c1), ALL_V),
    // I2C2
    af!('A', 10,  4, Signal::I2cSmba(I2cId::I2c2), ALL_V),
    af!('A',  8,  4, Signal::I2cSda(I2cId::I2c2), ALL_V),
    af!('A',  9,  4, Signal::I2cScl(I2cId::I2c2), ALL_V),
    af!('B', 12,  4, Signal::I2cSmba(I2cId::I2c2), ALL_V),
    af!('C',  4,  4, Signal::I2cScl(I2cId::I2c2), ALL_V),
    af!('F',  0,  4, Signal::I2cSda(I2cId::I2c2), ALL_V),
    af!('F',  2,  4, Signal::I2cSmba(I2cId::I2c2), LARGE_PQV),
    af!('F',  6,  4, Signal::I2cScl(I2cId::I2c2), BGA_PQ),
    // I2C3
    af!('A',  8,  2, Signal::I2cScl(I2cId::I2c3), ALL_V),
    af!('A',  9,  2, Signal::I2cSmba(I2cId::I2c3), ALL_V),
    af!('B',  2,  4, Signal::I2cSmba(I2cId::I2c3), ALL_V),
    af!('B',  5,  8, Signal::I2cSda(I2cId::I2c3), ALL_V),
    af!('C', 11,  8, Signal::I2cSda(I2cId::I2c3), ALL_V),
    af!('C',  8,  8, Signal::I2cScl(I2cId::I2c3), NON_C),
    af!('C',  9,  8, Signal::I2cSda(I2cId::I2c3), NON_C),
    af!('F',  3,  4, Signal::I2cScl(I2cId::I2c3), BGA_PQ),
    af!('F',  4,  4, Signal::I2cSda(I2cId::I2c3), BGA_PQ),
    af!('G',  6,  4, Signal::I2cSmba(I2cId::I2c3), BGA_Q),
    af!('G',  7,  4, Signal::I2cScl(I2cId::I2c3), BGA_Q),
    af!('G',  8,  4, Signal::I2cSda(I2cId::I2c3), BGA_Q),
    // I2C4
    af!('A', 13,  3, Signal::I2cScl(I2cId::I2c4), ALL_V),
    af!('A', 14,  3, Signal::I2cSmba(I2cId::I2c4), ALL_V),
    af!('B',  7,  3, Signal::I2cSda(I2cId::I2c4), ALL_V),
    af!('C',  6,  8, Signal::I2cScl(I2cId::I2c4), ALL_V),
    af!('C',  7,  8, Signal::I2cSda(I2cId::I2c4), NON_C),
    af!('D', 11,  4, Signal::I2cSmba(I2cId::I2c4), NON_CR),
    af!('F', 13,  4, Signal::I2cSmba(I2cId::I2c4), BGA_PQ),
    af!('F', 14,  4, Signal::I2cScl(I2cId::I2c4), BGA_PQ),
    af!('F', 15,  4, Signal::I2cSda(I2cId::I2c4), BGA_PQ),
    af!('G',  3,  4, Signal::I2cScl(I2cId::I2c4), BGA_PQ),
    af!('G',  4,  4, Signal::I2cSda(I2cId::I2c4), BGA_PQ),
    // LPUART1
    af!('A',  2, 12, Signal::LpuartTx(LpuartId::Lpuart1), ALL_V),
    af!('A',  3, 12, Signal::LpuartRx(LpuartId::Lpuart1), ALL_V),
    af!('A',  6, 12, Signal::LpuartCts(LpuartId::Lpuart1), ALL_V),
    af!('B',  1, 12, Signal::LpuartRts(LpuartId::Lpuart1), ALL_V),
    af!('B', 10,  8, Signal::LpuartRx(LpuartId::Lpuart1), ALL_V),
    af!('B', 11,  8, Signal::LpuartTx(LpuartId::Lpuart1), ALL_V),
    af!('B', 12,  8, Signal::LpuartRts(LpuartId::Lpuart1), ALL_V),
    af!('B', 13,  8, Signal::LpuartCts(LpuartId::Lpuart1), ALL_V),
    af!('C',  0,  8, Signal::LpuartRx(LpuartId::Lpuart1), NON_C),
    af!('C',  1,  8, Signal::LpuartTx(LpuartId::Lpuart1), NON_C),
    af!('G',  5,  8, Signal::LpuartCts(LpuartId::Lpuart1), BGA_Q),
    af!('G',  6,  8, Signal::LpuartRts(LpuartId::Lpuart1), BGA_Q),
    af!('G',  7,  8, Signal::LpuartTx(LpuartId::Lpuart1), BGA_Q),
    af!('G',  8,  8, Signal::LpuartRx(LpuartId::Lpuart1), BGA_Q),
    // SPI1
    af!('A', 15,  5, Signal::SpiNss(SpiId::Spi1), ALL_V),
    af!('A',  4,  5, Signal::SpiNss(SpiId::Spi1), ALL_V),
    af!('A',  5,  5, Signal::SpiSck(SpiId::Spi1), ALL_V),
    af!('A',  6,  5, Signal::SpiMiso(SpiId::Spi1), ALL_V),
    af!('A',  7,  5, Signal::SpiMosi(SpiId::Spi1), ALL_V),
    af!('B',  3,  5, Signal::SpiSck(SpiId::Spi1), ALL_V),
    af!('B',  4,  5, Signal::SpiMiso(SpiId::Spi1), ALL_V),
    af!('B',  5,  5, Signal::SpiMosi(SpiId::Spi1), ALL_V),
    af!('G',  2,  5, Signal::SpiSck(SpiId::Spi1), BGA_PQ),
    af!('G',  3,  5, Signal::SpiMiso(SpiId::Spi1), BGA_PQ),
    af!('G',  4,  5, Signal::SpiMosi(SpiId::Spi1), BGA_PQ),
    af!('G',  5,  5, Signal::SpiNss(SpiId::Spi1), BGA_Q),
    // SPI2
    af!('A', 10,  5, Signal::SpiMiso(SpiId::Spi2), ALL_V),
    af!('A', 11,  5, Signal::SpiMosi(SpiId::Spi2), ALL_V),
    af!('B', 12,  5, Signal::SpiNss(SpiId::Spi2), ALL_V),
    af!('B', 13,  5, Signal::SpiSck(SpiId::Spi2), ALL_V),
    af!('B', 14,  5, Signal::SpiMiso(SpiId::Spi2), ALL_V),
    af!('B', 15,  5, Signal::SpiMosi(SpiId::Spi2), ALL_V),
    af!('D', 15,  6, Signal::SpiNss(SpiId::Spi2), LARGE_PQV),
    af!('F',  0,  5, Signal::SpiNss(SpiId::Spi2), ALL_V),
    af!('F',  1,  5, Signal::SpiSck(SpiId::Spi2), ALL_V),
    af!('F', 10,  5, Signal::SpiSck(SpiId::Spi2), LARGE_PQV),
    af!('F',  9,  5, Signal::SpiSck(SpiId::Spi2), LARGE_PQV),
    // SPI3
    af!('A', 15,  6, Signal::SpiNss(SpiId::Spi3), ALL_V),
    af!('A',  4,  6, Signal::SpiNss(SpiId::Spi3), ALL_V),
    af!('B',  3,  6, Signal::SpiSck(SpiId::Spi3), ALL_V),
    af!('B',  4,  6, Signal::SpiMiso(SpiId::Spi3), ALL_V),
    af!('B',  5,  6, Signal::SpiMosi(SpiId::Spi3), ALL_V),
    af!('C', 10,  6, Signal::SpiSck(SpiId::Spi3), ALL_V),
    af!('C', 11,  6, Signal::SpiMiso(SpiId::Spi3), ALL_V),
    af!('C', 12,  6, Signal::SpiMosi(SpiId::Spi3), NON_C),
    af!('G',  9,  6, Signal::SpiSck(SpiId::Spi3), BGA_Q),
    // TIM1
    af!('A', 10,  6, Signal::TimCh(TimId::Tim1, TimCh::Ch3), ALL_V),
    af!('A', 11,  6, Signal::TimChN(TimId::Tim1, TimCh::Ch1), ALL_V),
    af!('A', 11, 11, Signal::TimCh(TimId::Tim1, TimCh::Ch4), ALL_V),
    af!('A', 11, 12, Signal::TimBkin2(TimId::Tim1), ALL_V),
    af!('A', 12,  6, Signal::TimChN(TimId::Tim1, TimCh::Ch2), ALL_V),
    af!('A', 12, 11, Signal::TimEtr(TimId::Tim1), ALL_V),
    af!('A', 14,  6, Signal::TimBkin(TimId::Tim1), ALL_V),
    af!('A', 15,  9, Signal::TimBkin(TimId::Tim1), ALL_V),
    af!('A',  6,  6, Signal::TimBkin(TimId::Tim1), ALL_V),
    af!('A',  7,  6, Signal::TimChN(TimId::Tim1, TimCh::Ch1), ALL_V),
    af!('A',  8,  6, Signal::TimCh(TimId::Tim1, TimCh::Ch1), ALL_V),
    af!('A',  9,  6, Signal::TimCh(TimId::Tim1, TimCh::Ch2), ALL_V),
    af!('B',  0,  6, Signal::TimChN(TimId::Tim1, TimCh::Ch2), ALL_V),
    af!('B',  1,  6, Signal::TimChN(TimId::Tim1, TimCh::Ch3), ALL_V),
    af!('B', 10, 12, Signal::TimBkin(TimId::Tim1), ALL_V),
    af!('B', 12,  6, Signal::TimBkin(TimId::Tim1), ALL_V),
    af!('B', 13,  6, Signal::TimChN(TimId::Tim1, TimCh::Ch1), ALL_V),
    af!('B', 14,  6, Signal::TimChN(TimId::Tim1, TimCh::Ch2), ALL_V),
    af!('B', 15,  4, Signal::TimChN(TimId::Tim1, TimCh::Ch3), ALL_V),
    af!('B',  8, 12, Signal::TimBkin(TimId::Tim1), ALL_V),
    af!('B',  9, 12, Signal::TimChN(TimId::Tim1, TimCh::Ch3), ALL_V),
    af!('C',  0,  2, Signal::TimCh(TimId::Tim1, TimCh::Ch1), NON_C),
    af!('C',  1,  2, Signal::TimCh(TimId::Tim1, TimCh::Ch2), NON_C),
    af!('C', 13,  2, Signal::TimBkin(TimId::Tim1), ALL_V),
    af!('C', 13,  4, Signal::TimChN(TimId::Tim1, TimCh::Ch1), ALL_V),
    af!('C',  2,  2, Signal::TimCh(TimId::Tim1, TimCh::Ch3), NON_C),
    af!('C',  3,  2, Signal::TimCh(TimId::Tim1, TimCh::Ch4), NON_C),
    af!('C',  3,  6, Signal::TimBkin2(TimId::Tim1), NON_C),
    af!('C',  4,  2, Signal::TimEtr(TimId::Tim1), ALL_V),
    af!('C',  5,  6, Signal::TimChN(TimId::Tim1, TimCh::Ch4), NON_C),
    af!('E', 10,  2, Signal::TimChN(TimId::Tim1, TimCh::Ch2), NON_CR),
    af!('E', 11,  2, Signal::TimCh(TimId::Tim1, TimCh::Ch2), NON_CR),
    af!('E', 12,  2, Signal::TimChN(TimId::Tim1, TimCh::Ch3), NON_CR),
    af!('E', 13,  2, Signal::TimCh(TimId::Tim1, TimCh::Ch3), NON_CR),
    af!('E', 14,  2, Signal::TimCh(TimId::Tim1, TimCh::Ch4), NON_CR),
    af!('E', 14,  6, Signal::TimBkin2(TimId::Tim1), NON_CR),
    af!('E', 15,  2, Signal::TimBkin(TimId::Tim1), NON_CR),
    af!('E', 15,  6, Signal::TimChN(TimId::Tim1, TimCh::Ch4), NON_CR),
    af!('E',  7,  2, Signal::TimEtr(TimId::Tim1), NON_CR),
    af!('E',  8,  2, Signal::TimChN(TimId::Tim1, TimCh::Ch1), NON_CR),
    af!('E',  9,  2, Signal::TimCh(TimId::Tim1, TimCh::Ch1), NON_CR),
    af!('F',  0,  6, Signal::TimChN(TimId::Tim1, TimCh::Ch3), ALL_V),
    // TIM15
    af!('A',  1,  9, Signal::TimChN(TimId::Tim15, TimCh::Ch1), ALL_V),
    af!('A',  2,  9, Signal::TimCh(TimId::Tim15, TimCh::Ch1), ALL_V),
    af!('A',  3,  9, Signal::TimCh(TimId::Tim15, TimCh::Ch2), ALL_V),
    af!('A',  9,  9, Signal::TimBkin(TimId::Tim15), ALL_V),
    af!('B', 14,  1, Signal::TimCh(TimId::Tim15, TimCh::Ch1), ALL_V),
    af!('B', 15,  1, Signal::TimCh(TimId::Tim15, TimCh::Ch2), ALL_V),
    af!('B', 15,  2, Signal::TimChN(TimId::Tim15, TimCh::Ch1), ALL_V),
    af!('C',  5,  2, Signal::TimBkin(TimId::Tim15), NON_C),
    af!('F', 10,  3, Signal::TimCh(TimId::Tim15, TimCh::Ch2), LARGE_PQV),
    af!('F',  9,  3, Signal::TimCh(TimId::Tim15, TimCh::Ch1), LARGE_PQV),
    af!('G',  9, 14, Signal::TimChN(TimId::Tim15, TimCh::Ch1), BGA_Q),
    // TIM16
    af!('A', 12,  1, Signal::TimCh(TimId::Tim16, TimCh::Ch1), ALL_V),
    af!('A', 13,  1, Signal::TimChN(TimId::Tim16, TimCh::Ch1), ALL_V),
    af!('A',  6,  1, Signal::TimCh(TimId::Tim16, TimCh::Ch1), ALL_V),
    af!('B',  4,  1, Signal::TimCh(TimId::Tim16, TimCh::Ch1), ALL_V),
    af!('B',  5,  1, Signal::TimBkin(TimId::Tim16), ALL_V),
    af!('B',  6,  1, Signal::TimChN(TimId::Tim16, TimCh::Ch1), ALL_V),
    af!('B',  8,  1, Signal::TimCh(TimId::Tim16, TimCh::Ch1), ALL_V),
    af!('E',  0,  4, Signal::TimCh(TimId::Tim16, TimCh::Ch1), LARGE_PQV),
    // TIM17
    af!('A', 10,  1, Signal::TimBkin(TimId::Tim17), ALL_V),
    af!('A',  7,  1, Signal::TimCh(TimId::Tim17, TimCh::Ch1), ALL_V),
    af!('B',  4, 10, Signal::TimBkin(TimId::Tim17), ALL_V),
    af!('B',  5, 10, Signal::TimCh(TimId::Tim17, TimCh::Ch1), ALL_V),
    af!('B',  7,  1, Signal::TimChN(TimId::Tim17, TimCh::Ch1), ALL_V),
    af!('B',  9,  1, Signal::TimCh(TimId::Tim17, TimCh::Ch1), ALL_V),
    af!('E',  1,  4, Signal::TimCh(TimId::Tim17, TimCh::Ch1), LARGE_PQV),
    // TIM2
    af!('A',  0,  1, Signal::TimCh(TimId::Tim2, TimCh::Ch1), ALL_V),
    af!('A',  0, 14, Signal::TimEtr(TimId::Tim2), ALL_V),
    af!('A',  1,  1, Signal::TimCh(TimId::Tim2, TimCh::Ch2), ALL_V),
    af!('A', 10, 10, Signal::TimCh(TimId::Tim2, TimCh::Ch4), ALL_V),
    af!('A', 15,  1, Signal::TimCh(TimId::Tim2, TimCh::Ch1), ALL_V),
    af!('A', 15, 14, Signal::TimEtr(TimId::Tim2), ALL_V),
    af!('A',  2,  1, Signal::TimCh(TimId::Tim2, TimCh::Ch3), ALL_V),
    af!('A',  3,  1, Signal::TimCh(TimId::Tim2, TimCh::Ch4), ALL_V),
    af!('A',  5,  1, Signal::TimCh(TimId::Tim2, TimCh::Ch1), ALL_V),
    af!('A',  5,  2, Signal::TimEtr(TimId::Tim2), ALL_V),
    af!('A',  9, 10, Signal::TimCh(TimId::Tim2, TimCh::Ch3), ALL_V),
    af!('B', 10,  1, Signal::TimCh(TimId::Tim2, TimCh::Ch3), ALL_V),
    af!('B', 11,  1, Signal::TimCh(TimId::Tim2, TimCh::Ch4), ALL_V),
    af!('B',  3,  1, Signal::TimCh(TimId::Tim2, TimCh::Ch2), ALL_V),
    af!('D',  3,  2, Signal::TimCh(TimId::Tim2, TimCh::Ch1), LARGE_PQV),
    af!('D',  3,  2, Signal::TimEtr(TimId::Tim2), LARGE_PQV),
    af!('D',  4,  2, Signal::TimCh(TimId::Tim2, TimCh::Ch2), LARGE_PQV),
    af!('D',  6,  2, Signal::TimCh(TimId::Tim2, TimCh::Ch4), LARGE_PQV),
    af!('D',  7,  2, Signal::TimCh(TimId::Tim2, TimCh::Ch3), LARGE_PQV),
    // TIM20
    af!('B',  2,  3, Signal::TimCh(TimId::Tim20, TimCh::Ch1), ALL_V),
    af!('C',  2,  6, Signal::TimCh(TimId::Tim20, TimCh::Ch2), NON_C),
    af!('C',  8,  6, Signal::TimCh(TimId::Tim20, TimCh::Ch3), NON_C),
    af!('E',  0,  3, Signal::TimChN(TimId::Tim20, TimCh::Ch4), LARGE_PQV),
    af!('E',  0,  6, Signal::TimEtr(TimId::Tim20), LARGE_PQV),
    af!('E',  1,  6, Signal::TimCh(TimId::Tim20, TimCh::Ch4), LARGE_PQV),
    af!('E',  2,  6, Signal::TimCh(TimId::Tim20, TimCh::Ch1), LARGE_PQV),
    af!('E',  3,  6, Signal::TimCh(TimId::Tim20, TimCh::Ch2), LARGE_PQV),
    af!('E',  4,  6, Signal::TimChN(TimId::Tim20, TimCh::Ch1), LARGE_PQV),
    af!('E',  5,  6, Signal::TimChN(TimId::Tim20, TimCh::Ch2), LARGE_PQV),
    af!('E',  6,  6, Signal::TimChN(TimId::Tim20, TimCh::Ch3), LARGE_PQV),
    af!('F', 10,  2, Signal::TimBkin2(TimId::Tim20), LARGE_PQV),
    af!('F', 11,  2, Signal::TimEtr(TimId::Tim20), BGA_PQ),
    af!('F', 12,  2, Signal::TimCh(TimId::Tim20, TimCh::Ch1), BGA_PQ),
    af!('F', 13,  2, Signal::TimCh(TimId::Tim20, TimCh::Ch2), BGA_PQ),
    af!('F', 14,  2, Signal::TimCh(TimId::Tim20, TimCh::Ch3), BGA_PQ),
    af!('F', 15,  2, Signal::TimCh(TimId::Tim20, TimCh::Ch4), BGA_PQ),
    af!('F',  2,  2, Signal::TimCh(TimId::Tim20, TimCh::Ch3), LARGE_PQV),
    af!('F',  3,  2, Signal::TimCh(TimId::Tim20, TimCh::Ch4), BGA_PQ),
    af!('F',  4,  3, Signal::TimChN(TimId::Tim20, TimCh::Ch1), BGA_PQ),
    af!('F',  5,  2, Signal::TimChN(TimId::Tim20, TimCh::Ch2), BGA_PQ),
    af!('F',  7,  2, Signal::TimBkin(TimId::Tim20), BGA_PQ),
    af!('F',  8,  2, Signal::TimBkin2(TimId::Tim20), BGA_PQ),
    af!('F',  9,  2, Signal::TimBkin(TimId::Tim20), LARGE_PQV),
    af!('G',  0,  2, Signal::TimChN(TimId::Tim20, TimCh::Ch1), BGA_PQ),
    af!('G',  1,  2, Signal::TimChN(TimId::Tim20, TimCh::Ch2), BGA_PQ),
    af!('G',  2,  2, Signal::TimChN(TimId::Tim20, TimCh::Ch3), BGA_PQ),
    af!('G',  3,  2, Signal::TimBkin(TimId::Tim20), BGA_PQ),
    af!('G',  3,  6, Signal::TimChN(TimId::Tim20, TimCh::Ch4), BGA_PQ),
    af!('G',  4,  2, Signal::TimBkin2(TimId::Tim20), BGA_PQ),
    af!('G',  5,  2, Signal::TimEtr(TimId::Tim20), BGA_Q),
    af!('G',  6,  2, Signal::TimBkin(TimId::Tim20), BGA_Q),
    // TIM3
    af!('A',  4,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch2), ALL_V),
    af!('A',  6,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch1), ALL_V),
    af!('A',  7,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch2), ALL_V),
    af!('B',  0,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch3), ALL_V),
    af!('B',  1,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch4), ALL_V),
    af!('B',  3, 10, Signal::TimEtr(TimId::Tim3), ALL_V),
    af!('B',  4,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch1), ALL_V),
    af!('B',  5,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch2), ALL_V),
    af!('B',  7, 10, Signal::TimCh(TimId::Tim3, TimCh::Ch4), ALL_V),
    af!('C',  6,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch1), ALL_V),
    af!('C',  7,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch2), NON_C),
    af!('C',  8,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch3), NON_C),
    af!('C',  9,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch4), NON_C),
    af!('D',  2,  2, Signal::TimEtr(TimId::Tim3), NON_C),
    af!('E',  2,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch1), LARGE_PQV),
    af!('E',  3,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch2), LARGE_PQV),
    af!('E',  4,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch3), LARGE_PQV),
    af!('E',  5,  2, Signal::TimCh(TimId::Tim3, TimCh::Ch4), LARGE_PQV),
    // TIM4
    af!('A', 11, 10, Signal::TimCh(TimId::Tim4, TimCh::Ch1), ALL_V),
    af!('A', 12, 10, Signal::TimCh(TimId::Tim4, TimCh::Ch2), ALL_V),
    af!('A', 13, 10, Signal::TimCh(TimId::Tim4, TimCh::Ch3), ALL_V),
    af!('A',  8, 10, Signal::TimEtr(TimId::Tim4), ALL_V),
    af!('B',  3,  2, Signal::TimEtr(TimId::Tim4), ALL_V),
    af!('B',  6,  2, Signal::TimCh(TimId::Tim4, TimCh::Ch1), ALL_V),
    af!('B',  7,  2, Signal::TimCh(TimId::Tim4, TimCh::Ch2), ALL_V),
    af!('B',  8,  2, Signal::TimCh(TimId::Tim4, TimCh::Ch3), ALL_V),
    af!('B',  9,  2, Signal::TimCh(TimId::Tim4, TimCh::Ch4), ALL_V),
    af!('D', 12,  2, Signal::TimCh(TimId::Tim4, TimCh::Ch1), LARGE_PQV),
    af!('D', 13,  2, Signal::TimCh(TimId::Tim4, TimCh::Ch2), LARGE_PQV),
    af!('D', 14,  2, Signal::TimCh(TimId::Tim4, TimCh::Ch3), LARGE_PQV),
    af!('D', 15,  2, Signal::TimCh(TimId::Tim4, TimCh::Ch4), LARGE_PQV),
    af!('E',  0,  2, Signal::TimEtr(TimId::Tim4), LARGE_PQV),
    af!('F',  6,  2, Signal::TimCh(TimId::Tim4, TimCh::Ch4), BGA_PQ),
    // TIM5
    af!('A',  0,  2, Signal::TimCh(TimId::Tim5, TimCh::Ch1), ALL_V),
    af!('A',  1,  2, Signal::TimCh(TimId::Tim5, TimCh::Ch2), ALL_V),
    af!('A',  2,  2, Signal::TimCh(TimId::Tim5, TimCh::Ch3), ALL_V),
    af!('A',  3,  2, Signal::TimCh(TimId::Tim5, TimCh::Ch4), ALL_V),
    af!('B', 12,  2, Signal::TimEtr(TimId::Tim5), ALL_V),
    af!('B',  2,  2, Signal::TimCh(TimId::Tim5, TimCh::Ch1), ALL_V),
    af!('C', 12,  1, Signal::TimCh(TimId::Tim5, TimCh::Ch2), NON_C),
    af!('D', 11,  1, Signal::TimEtr(TimId::Tim5), NON_CR),
    af!('E',  8,  1, Signal::TimCh(TimId::Tim5, TimCh::Ch3), NON_CR),
    af!('E',  9,  1, Signal::TimCh(TimId::Tim5, TimCh::Ch4), NON_CR),
    af!('F',  6,  1, Signal::TimEtr(TimId::Tim5), BGA_PQ),
    af!('F',  6,  6, Signal::TimCh(TimId::Tim5, TimCh::Ch1), BGA_PQ),
    af!('F',  7,  6, Signal::TimCh(TimId::Tim5, TimCh::Ch2), BGA_PQ),
    af!('F',  8,  6, Signal::TimCh(TimId::Tim5, TimCh::Ch3), BGA_PQ),
    af!('F',  9,  6, Signal::TimCh(TimId::Tim5, TimCh::Ch4), LARGE_PQV),
    // TIM8
    af!('A',  0,  9, Signal::TimBkin(TimId::Tim8), ALL_V),
    af!('A',  0, 10, Signal::TimEtr(TimId::Tim8), ALL_V),
    af!('A', 10, 11, Signal::TimBkin(TimId::Tim8), ALL_V),
    af!('A', 14,  5, Signal::TimCh(TimId::Tim8, TimCh::Ch2), ALL_V),
    af!('A', 15,  2, Signal::TimCh(TimId::Tim8, TimCh::Ch1), ALL_V),
    af!('A',  6,  4, Signal::TimBkin(TimId::Tim8), ALL_V),
    af!('A',  7,  4, Signal::TimChN(TimId::Tim8, TimCh::Ch1), ALL_V),
    af!('B',  0,  4, Signal::TimChN(TimId::Tim8, TimCh::Ch2), ALL_V),
    af!('B',  1,  4, Signal::TimChN(TimId::Tim8, TimCh::Ch3), ALL_V),
    af!('B',  3,  4, Signal::TimChN(TimId::Tim8, TimCh::Ch1), ALL_V),
    af!('B',  4,  4, Signal::TimChN(TimId::Tim8, TimCh::Ch2), ALL_V),
    af!('B',  5,  3, Signal::TimChN(TimId::Tim8, TimCh::Ch3), ALL_V),
    af!('B',  6,  5, Signal::TimCh(TimId::Tim8, TimCh::Ch1), ALL_V),
    af!('B',  6,  6, Signal::TimEtr(TimId::Tim8), ALL_V),
    af!('B',  6, 10, Signal::TimBkin2(TimId::Tim8), ALL_V),
    af!('B',  7,  5, Signal::TimBkin(TimId::Tim8), ALL_V),
    af!('B',  8, 10, Signal::TimCh(TimId::Tim8, TimCh::Ch2), ALL_V),
    af!('B',  9, 10, Signal::TimCh(TimId::Tim8, TimCh::Ch3), ALL_V),
    af!('C', 10,  4, Signal::TimChN(TimId::Tim8, TimCh::Ch1), ALL_V),
    af!('C', 11,  4, Signal::TimChN(TimId::Tim8, TimCh::Ch2), ALL_V),
    af!('C', 12,  4, Signal::TimChN(TimId::Tim8, TimCh::Ch3), NON_C),
    af!('C', 13,  6, Signal::TimChN(TimId::Tim8, TimCh::Ch4), ALL_V),
    af!('C',  6,  4, Signal::TimCh(TimId::Tim8, TimCh::Ch1), ALL_V),
    af!('C',  7,  4, Signal::TimCh(TimId::Tim8, TimCh::Ch2), NON_C),
    af!('C',  8,  4, Signal::TimCh(TimId::Tim8, TimCh::Ch3), NON_C),
    af!('C',  9,  4, Signal::TimCh(TimId::Tim8, TimCh::Ch4), NON_C),
    af!('C',  9,  6, Signal::TimBkin2(TimId::Tim8), NON_C),
    af!('D',  0,  6, Signal::TimChN(TimId::Tim8, TimCh::Ch4), NON_CR),
    af!('D',  1,  4, Signal::TimCh(TimId::Tim8, TimCh::Ch4), NON_CR),
    af!('D',  1,  6, Signal::TimBkin2(TimId::Tim8), NON_CR),
    af!('D',  2,  4, Signal::TimBkin(TimId::Tim8), NON_C),
    // UART4
    af!('A', 15,  8, Signal::UartRts(UartId::Uart4), ALL_V),
    af!('B',  7, 14, Signal::UartCts(UartId::Uart4), ALL_V),
    af!('C', 10,  5, Signal::UartTx(UartId::Uart4), ALL_V),
    af!('C', 11,  5, Signal::UartRx(UartId::Uart4), ALL_V),
    // UART5
    af!('B',  4,  8, Signal::UartRts(UartId::Uart5), NON_C),
    af!('B',  5, 14, Signal::UartCts(UartId::Uart5), NON_C),
    af!('C', 12,  5, Signal::UartTx(UartId::Uart5), NON_C),
    af!('D',  2,  5, Signal::UartRx(UartId::Uart5), NON_C),
    // UCPD1
    af!('B',  4,  0, Signal::UcpdCc2(UcpdId::Ucpd1), ALL_V),
    af!('B',  6,  0, Signal::UcpdCc1(UcpdId::Ucpd1), ALL_V),
    // USART1
    af!('A', 10,  7, Signal::UsartRx(UsartId::Usart1), ALL_V),
    af!('A', 11,  7, Signal::UsartCts(UsartId::Usart1), ALL_V),
    af!('A', 12,  7, Signal::UsartRts(UsartId::Usart1), ALL_V),
    af!('A',  8,  7, Signal::UsartCk(UsartId::Usart1), ALL_V),
    af!('A',  9,  7, Signal::UsartTx(UsartId::Usart1), ALL_V),
    af!('B',  6,  7, Signal::UsartTx(UsartId::Usart1), ALL_V),
    af!('B',  7,  7, Signal::UsartRx(UsartId::Usart1), ALL_V),
    af!('C',  4,  7, Signal::UsartTx(UsartId::Usart1), ALL_V),
    af!('C',  5,  7, Signal::UsartRx(UsartId::Usart1), NON_C),
    af!('E',  0,  7, Signal::UsartTx(UsartId::Usart1), LARGE_PQV),
    af!('E',  1,  7, Signal::UsartRx(UsartId::Usart1), LARGE_PQV),
    af!('G',  9,  7, Signal::UsartTx(UsartId::Usart1), BGA_Q),
    // USART2
    af!('A',  0,  7, Signal::UsartCts(UsartId::Usart2), ALL_V),
    af!('A',  1,  7, Signal::UsartRts(UsartId::Usart2), ALL_V),
    af!('A', 14,  7, Signal::UsartTx(UsartId::Usart2), ALL_V),
    af!('A', 15,  7, Signal::UsartRx(UsartId::Usart2), ALL_V),
    af!('A',  2,  7, Signal::UsartTx(UsartId::Usart2), ALL_V),
    af!('A',  3,  7, Signal::UsartRx(UsartId::Usart2), ALL_V),
    af!('A',  4,  7, Signal::UsartCk(UsartId::Usart2), ALL_V),
    af!('B',  3,  7, Signal::UsartTx(UsartId::Usart2), ALL_V),
    af!('B',  4,  7, Signal::UsartRx(UsartId::Usart2), ALL_V),
    af!('B',  5,  7, Signal::UsartCk(UsartId::Usart2), ALL_V),
    af!('D',  3,  7, Signal::UsartCts(UsartId::Usart2), LARGE_PQV),
    af!('D',  4,  7, Signal::UsartRts(UsartId::Usart2), LARGE_PQV),
    af!('D',  5,  7, Signal::UsartTx(UsartId::Usart2), LARGE_PQV),
    af!('D',  6,  7, Signal::UsartRx(UsartId::Usart2), LARGE_PQV),
    af!('D',  7,  7, Signal::UsartCk(UsartId::Usart2), LARGE_PQV),
    // USART3
    af!('A', 13,  7, Signal::UsartCts(UsartId::Usart3), ALL_V),
    af!('B', 10,  7, Signal::UsartTx(UsartId::Usart3), ALL_V),
    af!('B', 11,  7, Signal::UsartRx(UsartId::Usart3), ALL_V),
    af!('B', 12,  7, Signal::UsartCk(UsartId::Usart3), ALL_V),
    af!('B', 13,  7, Signal::UsartCts(UsartId::Usart3), ALL_V),
    af!('B', 14,  7, Signal::UsartRts(UsartId::Usart3), ALL_V),
    af!('B',  8,  7, Signal::UsartRx(UsartId::Usart3), ALL_V),
    af!('B',  9,  7, Signal::UsartTx(UsartId::Usart3), ALL_V),
    af!('C', 10,  7, Signal::UsartTx(UsartId::Usart3), ALL_V),
    af!('C', 11,  7, Signal::UsartRx(UsartId::Usart3), ALL_V),
    af!('C', 12,  7, Signal::UsartCk(UsartId::Usart3), NON_C),
    af!('D', 10,  7, Signal::UsartCk(UsartId::Usart3), NON_CR),
    af!('D', 11,  7, Signal::UsartCts(UsartId::Usart3), NON_CR),
    af!('D', 12,  7, Signal::UsartRts(UsartId::Usart3), LARGE_PQV),
    af!('D',  8,  7, Signal::UsartTx(UsartId::Usart3), NON_CR),
    af!('D',  9,  7, Signal::UsartRx(UsartId::Usart3), NON_CR),
    af!('E', 15,  7, Signal::UsartRx(UsartId::Usart3), NON_CR),
    af!('F',  6,  7, Signal::UsartRts(UsartId::Usart3), BGA_PQ),
    // USB
    af!('A', 11,  0, Signal::UsbDm, ALL_V),
    af!('A', 12,  0, Signal::UsbDp, ALL_V),
];

fn pin_on(opt: &AfOption, variant: ChipVariant) -> bool {
    opt.variants & variant.bit() != 0
}

pub fn pins_for(signal: Signal, variant: ChipVariant) -> Vec<Pin> {
    HRTIM_AF.iter()
        .filter(|o| o.signal == signal && pin_on(o, variant))
        .map(|o| o.pin)
        .collect()
}

pub fn signals_on(pin: Pin, variant: ChipVariant) -> Vec<(u8, Signal)> {
    HRTIM_AF.iter()
        .filter(|o| o.pin == pin && pin_on(o, variant))
        .map(|o| (o.af, o.signal))
        .collect()
}

pub fn conflicting_signal_pairs(
    used: &[Signal],
    variant: ChipVariant,
) -> Vec<(Signal, Signal, Pin)> {
    let mut pairs = Vec::new();
    for (i, a) in used.iter().enumerate() {
        for b in &used[i + 1..] {
            if a == b { continue; }
            for pa in pins_for(*a, variant) {
                if pins_for(*b, variant).contains(&pa) {
                    pairs.push((*a, *b, pa));
                }
            }
        }
    }
    pairs
}

pub fn unreachable_signals(used: &[Signal], variant: ChipVariant) -> Vec<Signal> {
    used.iter().copied().filter(|s| pins_for(*s, variant).is_empty()).collect()
}

/// Pin candidates for a signal, excluding pins already pinned to other
/// signals. Pass `locked` as a map from signal → pin (the user's explicit
/// choices). A signal's own lock is preserved (returns just its pin).
pub fn pin_candidates_respecting_locks(
    signal: Signal,
    variant: ChipVariant,
    locked: &std::collections::HashMap<Signal, Pin>,
) -> Vec<Pin> {
    if let Some(pin) = locked.get(&signal) {
        return vec![*pin];
    }
    let taken: std::collections::HashSet<Pin> = locked
        .iter()
        .filter(|(s, _)| **s != signal)
        .map(|(_, p)| *p)
        .collect();
    pins_for(signal, variant)
        .into_iter()
        .filter(|p| !taken.contains(p))
        .collect()
}
