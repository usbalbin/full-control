//! STM32G474 power-relevant peripheral fabric: connectivity data only.
//! Source: RM0440 chapters on HRTIM, COMP, DAC, ADC (see PDFs in repo root).

/// Coarse category used by the GUI to group peripheral instances into tabs
/// and palette sections. Distinct from a specific peripheral-id enum
/// (HrtimId, DacId, ...): those identify a specific instance, while this
/// enum names the kind of peripheral.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum PeripheralKind {
    // Power-fabric kinds (first-class in the existing Fabric view).
    Hrtim,
    Dac,
    Comp,
    Opamp,
    Adc,
    // Comms kinds (planned Stage 3 Comms tab).
    Can,
    Usart,
    Uart,
    Lpuart,
    I2c,
    Spi,
    Usb,
    Ucpd,
    // General-purpose / advanced-control timers (planned Stage 3 Timers tab).
    Tim,
}

impl PeripheralKind {
    pub const COMMS: &'static [PeripheralKind] = &[
        Self::Can, Self::Usart, Self::Uart, Self::Lpuart,
        Self::I2c, Self::Spi, Self::Usb, Self::Ucpd,
    ];
    pub fn label(self) -> &'static str {
        match self {
            Self::Hrtim => "HRTIM", Self::Dac => "DAC", Self::Comp => "COMP",
            Self::Opamp => "OPAMP", Self::Adc => "ADC",
            Self::Can => "CAN", Self::Usart => "USART", Self::Uart => "UART",
            Self::Lpuart => "LPUART", Self::I2c => "I2C", Self::Spi => "SPI",
            Self::Usb => "USB", Self::Ucpd => "UCPD",
            Self::Tim => "TIM",
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum HrtimId {
    TimA,
    TimB,
    TimC,
    TimD,
    TimE,
    TimF,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum DacId {
    Dac1Ch1,
    Dac1Ch2,
    Dac2Ch1,
    Dac3Ch1,
    Dac3Ch2,
    Dac4Ch1,
    Dac4Ch2,
}

impl DacId {
    /// DAC3/DAC4 are the 15 Msps "fast" sample-and-hold DACs whose output is
    /// internal-only — the right choice for cycle-by-cycle peak-current-mode
    /// thresholds at 1 MHz HRTIM.
    pub const fn is_fast(self) -> bool {
        matches!(
            self,
            DacId::Dac3Ch1 | DacId::Dac3Ch2 | DacId::Dac4Ch1 | DacId::Dac4Ch2
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum CompId {
    Comp1,
    Comp2,
    Comp3,
    Comp4,
    Comp5,
    Comp6,
    Comp7,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum OpampId {
    Opamp1,
    Opamp2,
    Opamp3,
    Opamp4,
    Opamp5,
    Opamp6,
}

impl OpampId {
    pub const ALL: &'static [OpampId] = &[
        OpampId::Opamp1, OpampId::Opamp2, OpampId::Opamp3,
        OpampId::Opamp4, OpampId::Opamp5, OpampId::Opamp6,
    ];
    pub fn number(self) -> u8 {
        match self {
            Self::Opamp1 => 1, Self::Opamp2 => 2, Self::Opamp3 => 3,
            Self::Opamp4 => 4, Self::Opamp5 => 5, Self::Opamp6 => 6,
        }
    }
}

// ----- Comms peripheral ids -----

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum SpiId { Spi1, Spi2, Spi3 }
impl SpiId {
    pub const ALL: &'static [SpiId] = &[Self::Spi1, Self::Spi2, Self::Spi3];
    pub fn number(self) -> u8 { match self { Self::Spi1 => 1, Self::Spi2 => 2, Self::Spi3 => 3 } }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum I2cId { I2c1, I2c2, I2c3, I2c4 }
impl I2cId {
    pub const ALL: &'static [I2cId] = &[Self::I2c1, Self::I2c2, Self::I2c3, Self::I2c4];
    pub fn number(self) -> u8 {
        match self { Self::I2c1 => 1, Self::I2c2 => 2, Self::I2c3 => 3, Self::I2c4 => 4 }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum UsartId { Usart1, Usart2, Usart3 }
impl UsartId {
    pub const ALL: &'static [UsartId] = &[Self::Usart1, Self::Usart2, Self::Usart3];
    pub fn number(self) -> u8 {
        match self { Self::Usart1 => 1, Self::Usart2 => 2, Self::Usart3 => 3 }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum UartId { Uart4, Uart5 }
impl UartId {
    pub const ALL: &'static [UartId] = &[Self::Uart4, Self::Uart5];
    pub fn number(self) -> u8 { match self { Self::Uart4 => 4, Self::Uart5 => 5 } }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum LpuartId { Lpuart1 }
impl LpuartId {
    pub const ALL: &'static [LpuartId] = &[Self::Lpuart1];
    pub fn number(self) -> u8 { 1 }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum CanId { Fdcan1, Fdcan2, Fdcan3 }
impl CanId {
    pub const ALL: &'static [CanId] = &[Self::Fdcan1, Self::Fdcan2, Self::Fdcan3];
    pub fn number(self) -> u8 {
        match self { Self::Fdcan1 => 1, Self::Fdcan2 => 2, Self::Fdcan3 => 3 }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum UcpdId { Ucpd1 }
impl UcpdId {
    pub const ALL: &'static [UcpdId] = &[Self::Ucpd1];
    pub fn number(self) -> u8 { 1 }
}

// ----- Timer ids + signal roles -----

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum TimId {
    Tim1, Tim2, Tim3, Tim4, Tim5, Tim6, Tim7, Tim8,
    Tim15, Tim16, Tim17, Tim20,
}
impl TimId {
    pub const ALL: &'static [TimId] = &[
        Self::Tim1, Self::Tim2, Self::Tim3, Self::Tim4, Self::Tim5,
        Self::Tim6, Self::Tim7, Self::Tim8,
        Self::Tim15, Self::Tim16, Self::Tim17, Self::Tim20,
    ];
    pub fn number(self) -> u8 {
        match self {
            Self::Tim1 => 1, Self::Tim2 => 2, Self::Tim3 => 3, Self::Tim4 => 4,
            Self::Tim5 => 5, Self::Tim6 => 6, Self::Tim7 => 7, Self::Tim8 => 8,
            Self::Tim15 => 15, Self::Tim16 => 16, Self::Tim17 => 17, Self::Tim20 => 20,
        }
    }
    /// Advanced-control timers (TIM1/8/20) have complementary outputs, BKIN,
    /// dead-time generator. Others are general-purpose / basic.
    pub fn is_advanced(self) -> bool {
        matches!(self, Self::Tim1 | Self::Tim8 | Self::Tim20)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum TimCh { Ch1, Ch2, Ch3, Ch4 }
impl TimCh {
    pub fn number(self) -> u8 {
        match self { Self::Ch1 => 1, Self::Ch2 => 2, Self::Ch3 => 3, Self::Ch4 => 4 }
    }
}

/// Sources that can drive HRTIM output set/reset crossbars and the ADC
/// trigger crossbar. Two separate fabrics on the chip; kept in one enum
/// because their source spaces overlap heavily and can be split later.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum CrossbarSource {
    Mcr1,
    Mcr2,
    Mcr3,
    Mcr4,
    Mper,

    Eev1,
    Eev2,
    Eev3,
    Eev4,
    Eev5,
    Eev6,
    Eev7,
    Eev8,
    Eev9,
    Eev10,

    TimACr2,
    TimACr3,
    TimACr4,
    TimACrPer,
    TimACrRst,

    TimBCr2,
    TimBCr3,
    TimBCr4,
    TimBCrPer,
    TimBCrRst,

    TimCCr2,
    TimCCr3,
    TimCCr4,
    TimCCrPer,
    TimCCrRst,

    TimDCr2,
    TimDCr3,
    TimDCr4,
    TimDCrPer,
    TimDCrRst,

    TimECr2,
    TimECr3,
    TimECr4,
    TimECrPer,
    TimECrRst,

    TimFCr2,
    TimFCr3,
    TimFCr4,
    TimFCrPer,
    TimFCrRst,
}

/// User-configurable compare slots on each HRTIM sub-timer (TIMA..TIMF) and
/// on the master timer. Each slot can drive outputs, EEVs, ADC triggers, DMA
/// requests, or DAC step/reset. CrPer and CrRst are automatic timer events
/// (period end, timer reset) and are not listed here — they're always
/// available but not "slot"-scarce in the same way CR1..4 are.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum TimerCompareSlot {
    Cr1,
    Cr2,
    Cr3,
    Cr4,
}

pub const ALL_CR_SLOTS: &[TimerCompareSlot] = &[
    TimerCompareSlot::Cr1,
    TimerCompareSlot::Cr2,
    TimerCompareSlot::Cr3,
    TimerCompareSlot::Cr4,
];

/// Capture units per HRTIM sub-timer. Two per timer. Per RM0440 §28.3
/// "Auto-delayed mode": CPT1 is paired with CMP2 (auto-delayed-CMP2 needs
/// CPT1 as its trigger source), CPT2 is paired with CMP4.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum TimerCaptureUnit {
    Cpt1,
    Cpt2,
}

pub const ALL_CAPTURE_UNITS: &[TimerCaptureUnit] =
    &[TimerCaptureUnit::Cpt1, TimerCaptureUnit::Cpt2];

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum HrtimFltId {
    Flt1,
    Flt2,
    Flt3,
    Flt4,
    Flt5,
    Flt6,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum AdcTriggerId {
    Trig1,
    Trig2,
    Trig3,
    Trig4,
    Trig5,
    Trig6,
    Trig7,
    Trig8,
    Trig9,
    Trig10,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum AdcChannel {
    Adc1Regular,
    Adc1Injected,
    Adc2Regular,
    Adc2Injected,
    Adc3Regular,
    Adc3Injected,
    Adc4Regular,
    Adc4Injected,
    Adc5Regular,
    Adc5Injected,
}

/// Physical ADC instance (distinct from sequencer slots). ADC1/2 are a
/// coupled pair supporting dual-regular/injected-simultaneous modes; the
/// same for ADC3/4. ADC5 is standalone.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum AdcInstance {
    Adc1, Adc2, Adc3, Adc4, Adc5,
}

impl AdcInstance {
    pub const ALL: &'static [AdcInstance] = &[
        AdcInstance::Adc1, AdcInstance::Adc2, AdcInstance::Adc3,
        AdcInstance::Adc4, AdcInstance::Adc5,
    ];
    pub fn number(self) -> u8 {
        match self {
            Self::Adc1 => 1, Self::Adc2 => 2, Self::Adc3 => 3,
            Self::Adc4 => 4, Self::Adc5 => 5,
        }
    }
}

/// Per STM32G474 datasheet (DS12288 Table 67 / footnote): fast channels
/// are ADCx_IN1..IN5 on every ADC. These have lower max R_AIN (input
/// resistance) and thus support the highest sampling rates — the only
/// practical choice for cycle-by-cycle measurements at ≥1 MHz.
pub const fn is_fast_adc_channel(channel: u8) -> bool {
    channel >= 1 && channel <= 5
}

// ---------- Connectivity tables (data-driven from `fabric_data`) ----------
//
// The analog routing is NO LONGER hand-authored here. It is grouped, once,
// from the generated + validated numeric tables in `fabric_data` (extracted
// from CubeMX modes + RM0440 by `gen_fabric`) into the solver's id-enum
// vocabulary. Provenance/golden checks live in the `fabric_validation` tests.

fn dac_id(inst: u8, ch: u8) -> Option<DacId> {
    use DacId::*;
    Some(match (inst, ch) {
        (1, 1) => Dac1Ch1, (1, 2) => Dac1Ch2, (2, 1) => Dac2Ch1,
        (3, 1) => Dac3Ch1, (3, 2) => Dac3Ch2, (4, 1) => Dac4Ch1, (4, 2) => Dac4Ch2,
        _ => return None,
    })
}
fn comp_id(n: u8) -> Option<CompId> {
    use CompId::*;
    Some(match n {
        1 => Comp1, 2 => Comp2, 3 => Comp3, 4 => Comp4,
        5 => Comp5, 6 => Comp6, 7 => Comp7, _ => return None,
    })
}
fn eev_source(n: u8) -> Option<CrossbarSource> {
    use CrossbarSource::*;
    Some(match n {
        1 => Eev1, 2 => Eev2, 3 => Eev3, 4 => Eev4, 5 => Eev5,
        6 => Eev6, 7 => Eev7, 8 => Eev8, 9 => Eev9, 10 => Eev10,
        _ => return None,
    })
}
fn flt_id(n: u8) -> Option<HrtimFltId> {
    use HrtimFltId::*;
    Some(match n {
        1 => Flt1, 2 => Flt2, 3 => Flt3, 4 => Flt4, 5 => Flt5, 6 => Flt6,
        _ => return None,
    })
}

/// Group flat `(key, value)` pairs into `(key, Vec<value>)`, sorted by key —
/// giving a stable, enum-defined order independent of the source row order.
fn group_pairs<K: Ord + Copy, V: Copy>(
    pairs: impl Iterator<Item = (K, V)>,
) -> Vec<(K, Vec<V>)> {
    let mut map: std::collections::BTreeMap<K, Vec<V>> = std::collections::BTreeMap::new();
    for (k, v) in pairs {
        map.entry(k).or_default().push(v);
    }
    map.into_iter().collect()
}

pub static DAC_TO_COMP: std::sync::LazyLock<Vec<(DacId, Vec<CompId>)>> =
    std::sync::LazyLock::new(|| {
        group_pairs(crate::fabric_data::G4_DAC_TO_COMP.iter().map(|&(di, dc, ci)| {
            (dac_id(di, dc).expect("bad DAC in fabric_data"), comp_id(ci).expect("bad COMP"))
        }))
    });

pub static COMP_TO_EEV: std::sync::LazyLock<Vec<(CompId, Vec<CrossbarSource>)>> =
    std::sync::LazyLock::new(|| {
        group_pairs(crate::fabric_data::G4_COMP_TO_EEV.iter().map(|&(ci, e)| {
            (comp_id(ci).expect("bad COMP"), eev_source(e).expect("bad EEV"))
        }))
    });

pub static COMP_TO_FLT: std::sync::LazyLock<Vec<(CompId, Vec<HrtimFltId>)>> =
    std::sync::LazyLock::new(|| {
        group_pairs(crate::fabric_data::G4_COMP_TO_FLT.iter().map(|&(ci, f)| {
            (comp_id(ci).expect("bad COMP"), flt_id(f).expect("bad FLT"))
        }))
    });

const TRIG_1234: &[AdcTriggerId] = &[
    AdcTriggerId::Trig1,
    AdcTriggerId::Trig2,
    AdcTriggerId::Trig3,
    AdcTriggerId::Trig4,
];
const TRIG_13: &[AdcTriggerId] = &[AdcTriggerId::Trig1, AdcTriggerId::Trig3];
const TRIG_24: &[AdcTriggerId] = &[AdcTriggerId::Trig2, AdcTriggerId::Trig4];

pub const CROSSBAR_TO_ADC_TRIGGER: &[(CrossbarSource, &[AdcTriggerId])] = &[
    (CrossbarSource::Mcr1, TRIG_1234),
    (CrossbarSource::Mcr2, TRIG_1234),
    (CrossbarSource::Mcr3, TRIG_1234),
    (CrossbarSource::Mcr4, TRIG_1234),
    (CrossbarSource::Mper, TRIG_1234),

    (CrossbarSource::Eev1, TRIG_13),
    (CrossbarSource::Eev2, TRIG_13),
    (CrossbarSource::Eev3, TRIG_13),
    (CrossbarSource::Eev4, TRIG_13),
    (CrossbarSource::Eev5, TRIG_13),
    (CrossbarSource::Eev6, TRIG_24),
    (CrossbarSource::Eev7, TRIG_24),
    (CrossbarSource::Eev8, TRIG_24),
    (CrossbarSource::Eev9, TRIG_24),
    (CrossbarSource::Eev10, TRIG_24),

    (CrossbarSource::TimACr2, TRIG_24),
    (CrossbarSource::TimACr3, TRIG_13),
    (CrossbarSource::TimACr4, TRIG_1234),
    (CrossbarSource::TimACrPer, TRIG_1234),
    (CrossbarSource::TimACrRst, TRIG_13),

    (CrossbarSource::TimBCr2, TRIG_24),
    (CrossbarSource::TimBCr3, TRIG_13),
    (CrossbarSource::TimBCr4, TRIG_1234),
    (CrossbarSource::TimBCrPer, TRIG_1234),
    (CrossbarSource::TimBCrRst, TRIG_13),

    (CrossbarSource::TimCCr2, TRIG_24),
    (CrossbarSource::TimCCr3, TRIG_13),
    (CrossbarSource::TimCCr4, TRIG_1234),
    (CrossbarSource::TimCCrPer, TRIG_1234),
    (CrossbarSource::TimCCrRst, TRIG_24),

    (CrossbarSource::TimDCr2, TRIG_24),
    (CrossbarSource::TimDCr3, TRIG_13),
    (CrossbarSource::TimDCr4, TRIG_1234),
    (CrossbarSource::TimDCrPer, TRIG_1234),
    (CrossbarSource::TimDCrRst, TRIG_24),

    (CrossbarSource::TimECr2, TRIG_24),
    // RM0440 HRTIM_ADC1R..4R: TimE CMP3 present in all four triggers; TimE
    // period in ADC1R/ADC3R (odd) only. (Both were wrong in the old hand table.)
    (CrossbarSource::TimECr3, TRIG_1234),
    (CrossbarSource::TimECr4, TRIG_1234),
    (CrossbarSource::TimECrPer, TRIG_13),
    (CrossbarSource::TimECrRst, TRIG_24),

    (CrossbarSource::TimFCr2, TRIG_24),
    (CrossbarSource::TimFCr3, TRIG_1234),
    (CrossbarSource::TimFCr4, TRIG_1234),
    (CrossbarSource::TimFCrPer, TRIG_1234),
    (CrossbarSource::TimFCrRst, TRIG_13),
];

const ADC_ODD: &[AdcChannel] = &[
    AdcChannel::Adc1Regular,
    AdcChannel::Adc2Regular,
    AdcChannel::Adc3Regular,
    AdcChannel::Adc3Injected,
    AdcChannel::Adc4Regular,
    AdcChannel::Adc4Injected,
    AdcChannel::Adc5Regular,
    AdcChannel::Adc5Injected,
];
const ADC_EVEN: &[AdcChannel] = &[
    AdcChannel::Adc1Injected,
    AdcChannel::Adc2Injected,
    AdcChannel::Adc3Regular,
    AdcChannel::Adc3Injected,
    AdcChannel::Adc4Regular,
    AdcChannel::Adc4Injected,
    AdcChannel::Adc5Regular,
    AdcChannel::Adc5Injected,
];

pub const ADC_TRIGGER_TO_ADC: &[(AdcTriggerId, &[AdcChannel])] = &[
    (AdcTriggerId::Trig1, ADC_ODD),
    (AdcTriggerId::Trig2, ADC_EVEN),
    (AdcTriggerId::Trig3, ADC_ODD),
    (AdcTriggerId::Trig4, ADC_EVEN),
    (AdcTriggerId::Trig5, ADC_ODD),
    (AdcTriggerId::Trig6, ADC_EVEN),
    (AdcTriggerId::Trig7, ADC_ODD),
    (AdcTriggerId::Trig8, ADC_EVEN),
    (AdcTriggerId::Trig9, ADC_ODD),
    (AdcTriggerId::Trig10, ADC_EVEN),
];

// ---------- Query helpers ----------

pub fn comps_for_dac(dac: DacId) -> &'static [CompId] {
    DAC_TO_COMP
        .iter()
        .find(|(d, _)| *d == dac)
        .map(|(_, c)| c.as_slice())
        .unwrap_or(&[])
}

pub fn dacs_for_comp(comp: CompId) -> impl Iterator<Item = DacId> {
    DAC_TO_COMP
        .iter()
        .filter_map(move |(d, cs)| cs.contains(&comp).then_some(*d))
}

pub fn eevs_for_comp(comp: CompId) -> &'static [CrossbarSource] {
    COMP_TO_EEV
        .iter()
        .find(|(c, _)| *c == comp)
        .map(|(_, e)| e.as_slice())
        .unwrap_or(&[])
}

pub fn flts_for_comp(comp: CompId) -> &'static [HrtimFltId] {
    COMP_TO_FLT
        .iter()
        .find(|(c, _)| *c == comp)
        .map(|(_, f)| f.as_slice())
        .unwrap_or(&[])
}

pub fn adc_triggers_for(event: CrossbarSource) -> &'static [AdcTriggerId] {
    CROSSBAR_TO_ADC_TRIGGER
        .iter()
        .find(|(e, _)| *e == event)
        .map(|(_, t)| *t)
        .unwrap_or(&[])
}

pub fn adc_channels_for(trig: AdcTriggerId) -> &'static [AdcChannel] {
    ADC_TRIGGER_TO_ADC
        .iter()
        .find(|(t, _)| *t == trig)
        .map(|(_, c)| *c)
        .unwrap_or(&[])
}

#[cfg(test)]
mod fabric_validation {
    use super::*;

    /// Golden snapshot of the RM-verified COMP->EEV routing (15 edges).
    /// `fabric_data` is now the source for the typed `COMP_TO_EEV` view (the
    /// solver consumes it via `eevs_for_comp`), so this guards the data with an
    /// expectation authored independently of the generator.
    #[test]
    fn g4_comp_to_eev_golden() {
        let golden: &[(u8, u8)] = &[
            (1, 4), (1, 6), (2, 1), (2, 6), (3, 5), (3, 8), (4, 2), (4, 7),
            (4, 9), (5, 4), (5, 9), (6, 3), (6, 8), (7, 5), (7, 10),
        ];
        let mut got = crate::fabric_data::G4_COMP_TO_EEV.to_vec();
        got.sort_unstable();
        let mut want = golden.to_vec();
        want.sort_unstable();
        assert_eq!(got, want, "G4_COMP_TO_EEV drifted from RM-verified golden");
    }

    /// Golden snapshot of RM0440 Table 228 COMP->FLT (6 edges; COMP7 has none).
    #[test]
    fn g4_comp_to_flt_golden() {
        let golden: &[(u8, u8)] = &[(1, 4), (2, 1), (3, 5), (4, 2), (5, 6), (6, 3)];
        let mut got = crate::fabric_data::G4_COMP_TO_FLT.to_vec();
        got.sort_unstable();
        let mut want = golden.to_vec();
        want.sort_unstable();
        assert_eq!(got, want, "G4_COMP_TO_FLT drifted from RM0440 Table 228 golden");
    }

    fn trig_num(t: AdcTriggerId) -> u8 {
        format!("{t:?}").trim_start_matches("Trig").parse().unwrap()
    }

    /// `gen_fabric` extracts the HRTIM crossbar->ADC-trigger map from CubeMX
    /// `ADCTRIGGEREVENT{13|24}` modes (one RM0440-verified fixup). Cross-checking
    /// it against the hand table found TWO bugs in the hand table (TimECr3,
    /// TimECrPer) and ONE in cubedb (TimFCr2); all three are now RM-correct.
    #[test]
    fn cubedb_crossbar_to_adc_trigger_matches_hand_table() {
        let mut generated: Vec<(String, Vec<u8>)> = crate::fabric_data::G4_CROSSBAR_TO_ADC_TRIGGER
            .iter()
            .map(|(s, ts)| (s.to_string(), ts.to_vec()))
            .collect();
        generated.sort();

        let mut hand: Vec<(String, Vec<u8>)> = CROSSBAR_TO_ADC_TRIGGER
            .iter()
            .map(|(src, trigs)| {
                let mut ts: Vec<u8> = trigs.iter().map(|t| trig_num(*t)).collect();
                ts.sort_unstable();
                (format!("{src:?}"), ts)
            })
            .collect();
        hand.sort();

        assert_eq!(
            generated, hand,
            "cubedb+RM-fixup crossbar->ADC must match (corrected) CROSSBAR_TO_ADC_TRIGGER"
        );
    }
}
