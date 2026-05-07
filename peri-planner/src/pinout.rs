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

}

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

/// Translate a typed `Signal` to its (peripheral name, role) form used by
/// metapac. Used by `pins_for` / `signals_on` to drive descriptor lookups.
fn signal_to_metapac(s: Signal) -> (&'static str, String) {
    use Signal::*;
    match s {
        HrtimChannel { timer, ch } => {
            let t = match timer {
                HrtimId::TimA => 'A', HrtimId::TimB => 'B', HrtimId::TimC => 'C',
                HrtimId::TimD => 'D', HrtimId::TimE => 'E', HrtimId::TimF => 'F',
            };
            let c = match ch { HrtimCh::Ch1 => '1', HrtimCh::Ch2 => '2' };
            ("HRTIM1", format!("CH{}{}", t, c))
        }
        HrtimEev(ev) => {
            let n = match ev {
                CrossbarSource::Eev1 => 1, CrossbarSource::Eev2 => 2,
                CrossbarSource::Eev3 => 3, CrossbarSource::Eev4 => 4,
                CrossbarSource::Eev5 => 5, CrossbarSource::Eev6 => 6,
                CrossbarSource::Eev7 => 7, CrossbarSource::Eev8 => 8,
                CrossbarSource::Eev9 => 9, CrossbarSource::Eev10 => 10,
                _ => return ("HRTIM1", String::new()), // master/sub-timer cross-bar — no pin
            };
            ("HRTIM1", format!("EEV{}", n))
        }
        HrtimFlt(f) => {
            let n = match f {
                HrtimFltId::Flt1 => 1, HrtimFltId::Flt2 => 2, HrtimFltId::Flt3 => 3,
                HrtimFltId::Flt4 => 4, HrtimFltId::Flt5 => 5, HrtimFltId::Flt6 => 6,
            };
            ("HRTIM1", format!("FLT{}", n))
        }
        HrtimScin  => ("HRTIM1", "SCIN".to_string()),
        HrtimScout => ("HRTIM1", "SCOUT".to_string()),
        CompInp(c) => (comp_name(c), "INP".to_string()),
        CompInm(c) => (comp_name(c), "INM".to_string()),
        CompOut(c) => (comp_name(c), "OUT".to_string()),
        DacOut(d) => {
            let (inst, ch) = match d {
                DacId::Dac1Ch1 => (1, 1), DacId::Dac1Ch2 => (1, 2),
                DacId::Dac2Ch1 => (2, 1),
                DacId::Dac3Ch1 => (3, 1), DacId::Dac3Ch2 => (3, 2),
                DacId::Dac4Ch1 => (4, 1), DacId::Dac4Ch2 => (4, 2),
            };
            (dac_name(inst), format!("OUT{}", ch))
        }
        AdcIn { adc, channel } => (adc_name(adc), format!("IN{}", channel)),
        OpampVinp(o) => (opamp_name(o), "VINP".to_string()),
        OpampVinm(o) => (opamp_name(o), "VINM".to_string()),
        OpampVout(o) => (opamp_name(o), "VOUT".to_string()),
        SpiMosi(s) => (spi_name(s), "MOSI".to_string()),
        SpiMiso(s) => (spi_name(s), "MISO".to_string()),
        SpiSck(s)  => (spi_name(s), "SCK".to_string()),
        SpiNss(s)  => (spi_name(s), "NSS".to_string()),
        I2cSda(i)  => (i2c_name(i), "SDA".to_string()),
        I2cScl(i)  => (i2c_name(i), "SCL".to_string()),
        I2cSmba(i) => (i2c_name(i), "SMBA".to_string()),
        UsartTx(u)  => (usart_name(u), "TX".to_string()),
        UsartRx(u)  => (usart_name(u), "RX".to_string()),
        UsartCts(u) => (usart_name(u), "CTS".to_string()),
        UsartRts(u) => (usart_name(u), "RTS".to_string()),
        UsartCk(u)  => (usart_name(u), "CK".to_string()),
        UartTx(u)  => (uart_name(u), "TX".to_string()),
        UartRx(u)  => (uart_name(u), "RX".to_string()),
        UartCts(u) => (uart_name(u), "CTS".to_string()),
        UartRts(u) => (uart_name(u), "RTS".to_string()),
        LpuartTx(_)  => ("LPUART1", "TX".to_string()),
        LpuartRx(_)  => ("LPUART1", "RX".to_string()),
        LpuartCts(_) => ("LPUART1", "CTS".to_string()),
        LpuartRts(_) => ("LPUART1", "RTS".to_string()),
        CanTx(c) => (fdcan_name(c), "TX".to_string()),
        CanRx(c) => (fdcan_name(c), "RX".to_string()),
        UsbDp => ("USB", "DP".to_string()),
        UsbDm => ("USB", "DM".to_string()),
        UcpdCc1(_) => ("UCPD1", "CC1".to_string()),
        UcpdCc2(_) => ("UCPD1", "CC2".to_string()),
        TimCh(t, ch)  => (tim_name(t), format!("CH{}", ch.number())),
        TimChN(t, ch) => (tim_name(t), format!("CH{}N", ch.number())),
        TimBkin(t)    => (tim_name(t), "BKIN".to_string()),
        TimBkin2(t)   => (tim_name(t), "BKIN2".to_string()),
        TimEtr(t)     => (tim_name(t), "ETR".to_string()),
    }
}

fn comp_name(c: CompId) -> &'static str {
    match c {
        CompId::Comp1 => "COMP1", CompId::Comp2 => "COMP2", CompId::Comp3 => "COMP3",
        CompId::Comp4 => "COMP4", CompId::Comp5 => "COMP5", CompId::Comp6 => "COMP6",
        CompId::Comp7 => "COMP7",
    }
}
fn dac_name(n: u8) -> &'static str {
    match n { 1 => "DAC1", 2 => "DAC2", 3 => "DAC3", 4 => "DAC4", _ => "" }
}
fn adc_name(a: AdcInstance) -> &'static str {
    match a {
        AdcInstance::Adc1 => "ADC1", AdcInstance::Adc2 => "ADC2",
        AdcInstance::Adc3 => "ADC3", AdcInstance::Adc4 => "ADC4",
        AdcInstance::Adc5 => "ADC5",
    }
}
fn opamp_name(o: OpampId) -> &'static str {
    match o {
        OpampId::Opamp1 => "OPAMP1", OpampId::Opamp2 => "OPAMP2",
        OpampId::Opamp3 => "OPAMP3", OpampId::Opamp4 => "OPAMP4",
        OpampId::Opamp5 => "OPAMP5", OpampId::Opamp6 => "OPAMP6",
    }
}
fn spi_name(s: SpiId) -> &'static str {
    match s { SpiId::Spi1 => "SPI1", SpiId::Spi2 => "SPI2", SpiId::Spi3 => "SPI3" }
}
fn i2c_name(i: I2cId) -> &'static str {
    match i { I2cId::I2c1 => "I2C1", I2cId::I2c2 => "I2C2", I2cId::I2c3 => "I2C3", I2cId::I2c4 => "I2C4" }
}
fn usart_name(u: UsartId) -> &'static str {
    match u { UsartId::Usart1 => "USART1", UsartId::Usart2 => "USART2", UsartId::Usart3 => "USART3" }
}
fn uart_name(u: UartId) -> &'static str {
    match u { UartId::Uart4 => "UART4", UartId::Uart5 => "UART5" }
}
fn fdcan_name(c: CanId) -> &'static str {
    match c { CanId::Fdcan1 => "FDCAN1", CanId::Fdcan2 => "FDCAN2", CanId::Fdcan3 => "FDCAN3" }
}
fn tim_name(t: TimId) -> &'static str {
    match t {
        TimId::Tim1 => "TIM1",   TimId::Tim2 => "TIM2",   TimId::Tim3 => "TIM3",
        TimId::Tim4 => "TIM4",   TimId::Tim5 => "TIM5",   TimId::Tim6 => "TIM6",
        TimId::Tim7 => "TIM7",   TimId::Tim8 => "TIM8",
        TimId::Tim15 => "TIM15", TimId::Tim16 => "TIM16",
        TimId::Tim17 => "TIM17", TimId::Tim20 => "TIM20",
    }
}

/// Reverse translate a metapac (peripheral, role) to a `Signal`. Returns
/// `None` for signals not represented in the typed enum (OPAMP secondary
/// inputs, ADC negative pins, GPIO, etc.).
fn metapac_to_signal(peripheral: &str, role: &str) -> Option<Signal> {
    // HRTIM channels and fabric.
    if peripheral == "HRTIM1" {
        if let Some(rest) = role.strip_prefix("CH") {
            let mut chars = rest.chars();
            let t_ch = chars.next()?;
            let c_ch = chars.next()?;
            if chars.next().is_some() { return None; }
            let timer = match t_ch {
                'A' => HrtimId::TimA, 'B' => HrtimId::TimB, 'C' => HrtimId::TimC,
                'D' => HrtimId::TimD, 'E' => HrtimId::TimE, 'F' => HrtimId::TimF,
                _ => return None,
            };
            let ch = match c_ch { '1' => HrtimCh::Ch1, '2' => HrtimCh::Ch2, _ => return None };
            return Some(Signal::HrtimChannel { timer, ch });
        }
        if let Some(n) = role.strip_prefix("EEV").and_then(|s| s.parse::<u8>().ok()) {
            let ev = match n {
                1 => CrossbarSource::Eev1, 2 => CrossbarSource::Eev2, 3 => CrossbarSource::Eev3,
                4 => CrossbarSource::Eev4, 5 => CrossbarSource::Eev5, 6 => CrossbarSource::Eev6,
                7 => CrossbarSource::Eev7, 8 => CrossbarSource::Eev8, 9 => CrossbarSource::Eev9,
                10 => CrossbarSource::Eev10, _ => return None,
            };
            return Some(Signal::HrtimEev(ev));
        }
        if let Some(n) = role.strip_prefix("FLT").and_then(|s| s.parse::<u8>().ok()) {
            let f = match n {
                1 => HrtimFltId::Flt1, 2 => HrtimFltId::Flt2, 3 => HrtimFltId::Flt3,
                4 => HrtimFltId::Flt4, 5 => HrtimFltId::Flt5, 6 => HrtimFltId::Flt6,
                _ => return None,
            };
            return Some(Signal::HrtimFlt(f));
        }
        return Some(match role {
            "SCIN"  => Signal::HrtimScin,
            "SCOUT" => Signal::HrtimScout,
            _ => return None,
        });
    }

    // COMP1..COMP7
    if let Some(n) = peripheral.strip_prefix("COMP").and_then(|s| s.parse::<u8>().ok()) {
        let c = match n {
            1 => CompId::Comp1, 2 => CompId::Comp2, 3 => CompId::Comp3, 4 => CompId::Comp4,
            5 => CompId::Comp5, 6 => CompId::Comp6, 7 => CompId::Comp7, _ => return None,
        };
        return Some(match role {
            "INP" => Signal::CompInp(c), "INM" => Signal::CompInm(c), "OUT" => Signal::CompOut(c),
            _ => return None,
        });
    }

    // DAC1/2/3/4 channels
    if let Some(n) = peripheral.strip_prefix("DAC").and_then(|s| s.parse::<u8>().ok()) {
        let ch = role.strip_prefix("OUT").and_then(|s| s.parse::<u8>().ok())?;
        let id = match (n, ch) {
            (1, 1) => DacId::Dac1Ch1, (1, 2) => DacId::Dac1Ch2,
            (2, 1) => DacId::Dac2Ch1,
            (3, 1) => DacId::Dac3Ch1, (3, 2) => DacId::Dac3Ch2,
            (4, 1) => DacId::Dac4Ch1, (4, 2) => DacId::Dac4Ch2,
            _ => return None,
        };
        return Some(Signal::DacOut(id));
    }

    // ADCs — only the positive `IN<n>` channels are represented in Signal.
    if let Some(n) = peripheral.strip_prefix("ADC").and_then(|s| s.parse::<u8>().ok()) {
        let adc = match n {
            1 => AdcInstance::Adc1, 2 => AdcInstance::Adc2, 3 => AdcInstance::Adc3,
            4 => AdcInstance::Adc4, 5 => AdcInstance::Adc5, _ => return None,
        };
        let channel = role.strip_prefix("IN").filter(|s| !s.starts_with('N'))
            .and_then(|s| s.parse::<u8>().ok())?;
        return Some(Signal::AdcIn { adc, channel });
    }

    // OPAMPs — ignore the *_SEC variants and numbered VINP0/VINP1 selectors;
    // only the bare VINP/VINM/VOUT roles map to a Signal.
    if let Some(n) = peripheral.strip_prefix("OPAMP").and_then(|s| s.parse::<u8>().ok()) {
        let o = match n {
            1 => OpampId::Opamp1, 2 => OpampId::Opamp2, 3 => OpampId::Opamp3,
            4 => OpampId::Opamp4, 5 => OpampId::Opamp5, 6 => OpampId::Opamp6,
            _ => return None,
        };
        return Some(match role {
            "VINP" => Signal::OpampVinp(o), "VINM" => Signal::OpampVinm(o), "VOUT" => Signal::OpampVout(o),
            _ => return None,
        });
    }

    // SPI / I2C / USART / UART / LPUART / FDCAN / USB / UCPD / TIMx
    if let Some(n) = peripheral.strip_prefix("SPI").and_then(|s| s.parse::<u8>().ok()) {
        let s = match n { 1 => SpiId::Spi1, 2 => SpiId::Spi2, 3 => SpiId::Spi3, _ => return None };
        return Some(match role {
            "MOSI" => Signal::SpiMosi(s), "MISO" => Signal::SpiMiso(s),
            "SCK"  => Signal::SpiSck(s),  "NSS"  => Signal::SpiNss(s),
            _ => return None,
        });
    }
    if let Some(n) = peripheral.strip_prefix("I2C").and_then(|s| s.parse::<u8>().ok()) {
        let i = match n {
            1 => I2cId::I2c1, 2 => I2cId::I2c2, 3 => I2cId::I2c3, 4 => I2cId::I2c4, _ => return None,
        };
        return Some(match role {
            "SDA" => Signal::I2cSda(i), "SCL" => Signal::I2cScl(i), "SMBA" => Signal::I2cSmba(i),
            _ => return None,
        });
    }
    if let Some(n) = peripheral.strip_prefix("USART").and_then(|s| s.parse::<u8>().ok()) {
        let u = match n { 1 => UsartId::Usart1, 2 => UsartId::Usart2, 3 => UsartId::Usart3, _ => return None };
        return Some(match role {
            "TX"  => Signal::UsartTx(u), "RX"  => Signal::UsartRx(u),
            "CTS" => Signal::UsartCts(u), "RTS" => Signal::UsartRts(u),
            "CK"  => Signal::UsartCk(u),
            _ => return None,
        });
    }
    if let Some(n) = peripheral.strip_prefix("UART").and_then(|s| s.parse::<u8>().ok()) {
        let u = match n { 4 => UartId::Uart4, 5 => UartId::Uart5, _ => return None };
        return Some(match role {
            "TX"  => Signal::UartTx(u), "RX"  => Signal::UartRx(u),
            "CTS" => Signal::UartCts(u), "RTS" => Signal::UartRts(u),
            _ => return None,
        });
    }
    if peripheral == "LPUART1" {
        return Some(match role {
            "TX" => Signal::LpuartTx(LpuartId::Lpuart1),
            "RX" => Signal::LpuartRx(LpuartId::Lpuart1),
            "CTS" => Signal::LpuartCts(LpuartId::Lpuart1),
            "RTS" => Signal::LpuartRts(LpuartId::Lpuart1),
            _ => return None,
        });
    }
    if let Some(n) = peripheral.strip_prefix("FDCAN").and_then(|s| s.parse::<u8>().ok()) {
        let c = match n { 1 => CanId::Fdcan1, 2 => CanId::Fdcan2, 3 => CanId::Fdcan3, _ => return None };
        return Some(match role {
            "TX" => Signal::CanTx(c), "RX" => Signal::CanRx(c), _ => return None,
        });
    }
    if peripheral == "USB" {
        return Some(match role {
            "DP" => Signal::UsbDp, "DM" => Signal::UsbDm, _ => return None,
        });
    }
    if peripheral == "UCPD1" {
        return Some(match role {
            "CC1" => Signal::UcpdCc1(UcpdId::Ucpd1),
            "CC2" => Signal::UcpdCc2(UcpdId::Ucpd1),
            _ => return None,
        });
    }
    if let Some(n) = peripheral.strip_prefix("TIM").and_then(|s| s.parse::<u8>().ok()) {
        let t = match n {
            1 => TimId::Tim1, 2 => TimId::Tim2, 3 => TimId::Tim3, 4 => TimId::Tim4,
            5 => TimId::Tim5, 6 => TimId::Tim6, 7 => TimId::Tim7, 8 => TimId::Tim8,
            15 => TimId::Tim15, 16 => TimId::Tim16, 17 => TimId::Tim17, 20 => TimId::Tim20,
            _ => return None,
        };
        if let Some(rest) = role.strip_prefix("CH") {
            // "1", "2", "3", "4" or "1N", "2N", "3N"
            let (n_str, complementary) = match rest.strip_suffix('N') {
                Some(s) => (s, true),
                None => (rest, false),
            };
            let ch = match n_str.parse::<u8>().ok()? {
                1 => TimCh::Ch1, 2 => TimCh::Ch2, 3 => TimCh::Ch3, 4 => TimCh::Ch4, _ => return None,
            };
            return Some(if complementary { Signal::TimChN(t, ch) } else { Signal::TimCh(t, ch) });
        }
        return Some(match role {
            "BKIN"  => Signal::TimBkin(t),
            "BKIN2" => Signal::TimBkin2(t),
            "ETR"   => Signal::TimEtr(t),
            _ => return None,
        });
    }

    None
}

pub fn pins_for(signal: Signal, variant: ChipVariant) -> Vec<Pin> {
    let pkg = crate::mcu::Package::from_g474_variant(variant);
    let raw = pkg.raw();
    let (peripheral, role) = signal_to_metapac(signal);
    if role.is_empty() { return Vec::new(); }
    raw.peripherals.iter()
        .filter(|p| p.name == peripheral)
        .flat_map(|p| p.pins.iter())
        .filter(|pp| pp.signal == role.as_str())
        .filter_map(|pp| {
            let port = pp.pin.as_bytes().get(1).copied()? as char;
            let num: u8 = pp.pin.get(2..)?.parse().ok()?;
            Some(Pin::new(port, num))
        })
        .collect()
}

pub fn signals_on(pin: Pin, variant: ChipVariant) -> Vec<(u8, Signal)> {
    let pkg = crate::mcu::Package::from_g474_variant(variant);
    let raw = pkg.raw();
    let pin_name = pin.name();
    let mut out = Vec::new();
    for p in raw.peripherals {
        for pp in p.pins {
            if pp.pin != pin_name { continue; }
            let Some(sig) = metapac_to_signal(p.name, pp.signal) else { continue; };
            // Analog signals (af = None) are reported as AF=0 to match the
            // existing `(u8, Signal)` API where 0 stands for "no AF".
            let af = pp.af.unwrap_or(0);
            if !out.contains(&(af, sig)) {
                out.push((af, sig));
            }
        }
    }
    out
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
