//! Abstract requirements + resource-claim framework.
//!
//! Each `RequirementSpec` declares an intent (e.g. "a PCM phase with DEM",
//! "a short-circuit fault channel", "an ADC regular sequencer triggered by
//! the HRTIM master"); it can be `enumerate`d against a pool of already-used
//! resources to yield concrete `Assignment`s. A `Design` holds a list of
//! requirements + their current assignments, plus stable `u32` identifiers
//! used for cross-referencing (e.g. an `AdcConversion` refers to its parent
//! sequencer by id).

use std::collections::HashSet;

use crate::g474::*;
use crate::solver::{PhaseAllocation, ShareBusDrive, ShortCircuitFault};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum MonitorPurpose {
    VOut,
    VIn,
    Ntc,
    ShareBusSense,
    InternalTemp,
    PhaseCurrent(u8),
    Custom,
}

impl MonitorPurpose {
    pub fn short_name(self) -> String {
        match self {
            Self::VOut => "V_OUT".to_string(),
            Self::VIn => "V_IN".to_string(),
            Self::Ntc => "NTC".to_string(),
            Self::ShareBusSense => "share-bus sense".to_string(),
            Self::InternalTemp => "internal temp".to_string(),
            Self::PhaseCurrent(n) => format!("phase {} current", n),
            Self::Custom => "custom".to_string(),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum ThresholdSource {
    /// Internal COMP + fast DAC for peak detection. With DEM, the same
    /// COMP handles zero-crossing via INM-swap (DMA or firmware).
    Internal,
    /// Internal COMP + fast DAC for peak (with slope comp), plus a
    /// dedicated external comp on a second EEV for zero-crossing.
    /// Meaningful only when DEM is enabled.
    InternalExtZcd,
    /// External comp for peak detection (no slope comp — slow DAC can't
    /// drive a sawtooth at 1 MHz). With DEM, a second external comp on
    /// another EEV handles zero-crossing.
    External,
}

/// ADC sequencer mode. `Dual*` kinds couple Adc1+Adc2 or Adc3+Adc4 and can
/// only be placed on the master (Adc1 or Adc3).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum SequencerKind {
    Regular,
    Injected,
    DualRegular,
    DualInjected,
}

/// Strict channel-speed filter for an ADC conversion. Per G474 datasheet
/// (DS12288 Table 67 footnote): fast channels = ADCx_IN1..IN5.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum SpeedPref {
    #[default]
    Any,
    Fast,
    Slow,
}

impl SpeedPref {
    pub fn allows(self, channel: u8) -> bool {
        match self {
            Self::Any => true,
            Self::Fast => crate::g474::is_fast_adc_channel(channel),
            Self::Slow => !crate::g474::is_fast_adc_channel(channel),
        }
    }
    pub fn short(self) -> &'static str {
        match self { Self::Any => "any", Self::Fast => "fast", Self::Slow => "slow" }
    }
}

impl SequencerKind {
    pub fn is_dual(self) -> bool {
        matches!(self, Self::DualRegular | Self::DualInjected)
    }
}

/// What starts an ADC conversion sequence.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum TriggerSource {
    /// Software-started (no hardware trigger).
    Software,
    /// Hardware-triggered via an HRTIM / timer event on the ADC trigger crossbar.
    Event(CrossbarSource),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Resource {
    Dac(DacId),
    Comp(CompId),
    Eev(CrossbarSource),
    Flt(HrtimFltId),
    Timer(HrtimId),
    AdcTrigger(AdcTriggerId),
    TimerSlot(HrtimId, TimerCompareSlot),
    TimerCapture(HrtimId, TimerCaptureUnit),
    /// An ADC's regular or injected sequencer (exclusive; one consumer).
    /// For dual-simultaneous modes, the *master* ADC's sequencer is
    /// claimed, plus the slave ADC's same-kind sequencer.
    AdcSequencer(AdcInstance, SequencerKind),
    /// An ADC input channel on a specific ADC instance. Corresponds to a
    /// unique GPIO pin per `pinout::Signal::AdcIn`.
    AdcInput(AdcInstance, u8),
    /// A physical GPIO pin. Claimed by signals that have a fixed pin (only
    /// one option on this chip variant) or that the user has explicitly
    /// pinned via the pin-map UI. Lets the planner detect cross-fabric
    /// conflicts (e.g. HRTIM_CHA1 = PA8 also being ADC5_IN1).
    Pin(crate::pinout::Pin),
    /// HRTIM master-timer compare register MCR1..MCR4. Claimed when an ADC
    /// sequencer is triggered from the matching master-compare event — two
    /// sequencers on the same compare would need the same tick-value,
    /// which normally means the user should combine them.
    MasterCompareSlot(u8),
    Opamp(OpampId),
    Spi(SpiId),
    I2c(I2cId),
    Usart(UsartId),
    Uart(UartId),
    Lpuart(LpuartId),
    Can(CanId),
    Usb,
    Ucpd(UcpdId),
    Tim(TimId),
}

/// Which compare-slot resource (if any) a crossbar trigger event consumes.
/// Period and reset events use the timer's inherent counter state — no
/// compare register needed. EEVs are external inputs.
pub fn compare_slot_for_event(ev: CrossbarSource) -> Option<Resource> {
    use crate::g474::TimerCompareSlot as TS;
    match ev {
        CrossbarSource::Mcr1 => Some(Resource::MasterCompareSlot(1)),
        CrossbarSource::Mcr2 => Some(Resource::MasterCompareSlot(2)),
        CrossbarSource::Mcr3 => Some(Resource::MasterCompareSlot(3)),
        CrossbarSource::Mcr4 => Some(Resource::MasterCompareSlot(4)),
        CrossbarSource::TimACr2 => Some(Resource::TimerSlot(HrtimId::TimA, TS::Cr2)),
        CrossbarSource::TimACr3 => Some(Resource::TimerSlot(HrtimId::TimA, TS::Cr3)),
        CrossbarSource::TimACr4 => Some(Resource::TimerSlot(HrtimId::TimA, TS::Cr4)),
        CrossbarSource::TimBCr2 => Some(Resource::TimerSlot(HrtimId::TimB, TS::Cr2)),
        CrossbarSource::TimBCr3 => Some(Resource::TimerSlot(HrtimId::TimB, TS::Cr3)),
        CrossbarSource::TimBCr4 => Some(Resource::TimerSlot(HrtimId::TimB, TS::Cr4)),
        CrossbarSource::TimCCr2 => Some(Resource::TimerSlot(HrtimId::TimC, TS::Cr2)),
        CrossbarSource::TimCCr3 => Some(Resource::TimerSlot(HrtimId::TimC, TS::Cr3)),
        CrossbarSource::TimCCr4 => Some(Resource::TimerSlot(HrtimId::TimC, TS::Cr4)),
        CrossbarSource::TimDCr2 => Some(Resource::TimerSlot(HrtimId::TimD, TS::Cr2)),
        CrossbarSource::TimDCr3 => Some(Resource::TimerSlot(HrtimId::TimD, TS::Cr3)),
        CrossbarSource::TimDCr4 => Some(Resource::TimerSlot(HrtimId::TimD, TS::Cr4)),
        CrossbarSource::TimECr2 => Some(Resource::TimerSlot(HrtimId::TimE, TS::Cr2)),
        CrossbarSource::TimECr3 => Some(Resource::TimerSlot(HrtimId::TimE, TS::Cr3)),
        CrossbarSource::TimECr4 => Some(Resource::TimerSlot(HrtimId::TimE, TS::Cr4)),
        CrossbarSource::TimFCr2 => Some(Resource::TimerSlot(HrtimId::TimF, TS::Cr2)),
        CrossbarSource::TimFCr3 => Some(Resource::TimerSlot(HrtimId::TimF, TS::Cr3)),
        CrossbarSource::TimFCr4 => Some(Resource::TimerSlot(HrtimId::TimF, TS::Cr4)),
        // Period/reset events use inherent timer state; EEVs are external.
        _ => None,
    }
}

pub type ResourceBag = HashSet<Resource>;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum RequirementSpec {
    PcmPhase {
        dem: bool,
        threshold: ThresholdSource,
        /// Pin this phase to a specific HRTIM sub-timer. `None` = solver
        /// picks the first free timer (legacy behavior, useful when you
        /// just need N phases and don't care which sub-timers they land
        /// on). Set to `Some` when a specific sub-timer matters — e.g.
        /// reserving TimF for an auxiliary topology that needs its TimF
        /// event slots, or making sure TimA is kept for a fast phase.
        #[serde(default)]
        preferred_timer: Option<HrtimId>,
    },
    ShortCircuitFault,
    ShareBusDrive,
    AdcSequencer {
        adc: AdcInstance,
        kind: SequencerKind,
        trigger: TriggerSource,
    },
    AdcConversion {
        /// Stable id of the parent sequencer requirement. Normalize
        /// handles stale refs (orphan → assignment None) but the
        /// containing requirement is not deleted; user can reattach.
        group: u32,
        purpose: MonitorPurpose,
        /// Restrict to a specific ADC unit (only meaningful when the
        /// parent is a dual sequencer, which otherwise offers both ADCs
        /// of the coupled pair). `None` = either ADC of the parent pool.
        #[serde(default)]
        adc_pref: Option<AdcInstance>,
        /// Restrict to fast / slow channels. Lets the user express
        /// "this measurement must land on a fast channel" (e.g. for
        /// cycle-by-cycle sampling) without pinning a specific channel.
        #[serde(default)]
        speed: SpeedPref,
    },
    /// Use a specific OPAMP instance with a configurable mix of
    /// external-pin vs internal routing. VOUT pin can be skipped for
    /// OPAMP→ADC internal routing (follower mode); VINM can be skipped
    /// when feedback comes from the internal DAC.
    UseOpamp {
        instance: OpampId,
        external_vinp: bool,
        external_vinm: bool,
        external_vout: bool,
    },
    UseSpi { instance: SpiId, needs_miso: bool, needs_nss: bool },
    UseI2c { instance: I2cId, needs_smba: bool },
    UseUsart { instance: UsartId, flow_control: bool, synchronous: bool },
    UseUart { instance: UartId, flow_control: bool },
    UseLpuart { instance: LpuartId, flow_control: bool },
    UseCan { instance: CanId },
    UseUsb,
    UseUcpd { instance: UcpdId },
    /// Claim a timer with a selection of signals routed to pins. Channel
    /// mask is CH1..CH4 (bit 0..3). For advanced-control timers (TIM1/8/
    /// 20) set `complementary` to also claim CHxN pins, and `bkin` for
    /// the safety break input.
    UseTim {
        instance: TimId,
        #[serde(default = "default_tim_channels_mask")]
        channels_mask: u8,
        #[serde(default)]
        complementary: bool,
        #[serde(default)]
        bkin: bool,
        #[serde(default)]
        etr: bool,
    },
}

fn default_tim_channels_mask() -> u8 { 0b0011 }

impl RequirementSpec {
    /// Canonical "add one of each" defaults for the top-panel Add
    /// buttons. One entry per distinct requirement *shape* — PCM-phase
    /// variants (DEM / threshold) and ADC-conversion purposes get picked
    /// via the per-row spec combobox after add, not via separate buttons.
    pub fn add_palette() -> &'static [Self] {
        &[
            // Power
            Self::PcmPhase {
                dem: false, threshold: ThresholdSource::Internal, preferred_timer: None,
            },
            Self::ShortCircuitFault,
            Self::ShareBusDrive,
            Self::UseOpamp {
                instance: OpampId::Opamp1,
                external_vinp: true, external_vinm: true, external_vout: true,
            },
            // ADC
            Self::AdcSequencer {
                adc: AdcInstance::Adc1,
                kind: SequencerKind::DualRegular,
                trigger: TriggerSource::Event(CrossbarSource::Mcr1),
            },
            Self::AdcConversion {
                group: 0, purpose: MonitorPurpose::VOut,
                adc_pref: None, speed: SpeedPref::Any,
            },
            // Timer (HRTIM has its own tab's Add flow; this is for TIM*)
            Self::UseTim {
                instance: TimId::Tim2, channels_mask: 0b0011,
                complementary: false, bkin: false, etr: false,
            },
        ]
    }

    /// Comms peripheral Add entries, shown in a collapsible menu so the
    /// eight families (SPI/I2C/USART/UART/LPUART/CAN/USB/UCPD) don't
    /// dominate the horizontal palette.
    pub fn comms_palette() -> &'static [Self] {
        &[
            Self::UseSpi { instance: SpiId::Spi1, needs_miso: true, needs_nss: false },
            Self::UseI2c { instance: I2cId::I2c1, needs_smba: false },
            Self::UseUsart { instance: UsartId::Usart1, flow_control: false, synchronous: false },
            Self::UseUart { instance: UartId::Uart4, flow_control: false },
            Self::UseLpuart { instance: LpuartId::Lpuart1, flow_control: false },
            Self::UseCan { instance: CanId::Fdcan1 },
            Self::UseUsb,
            Self::UseUcpd { instance: UcpdId::Ucpd1 },
        ]
    }

    pub fn palette() -> &'static [Self] {
        &[
            Self::PcmPhase { dem: false, threshold: ThresholdSource::Internal, preferred_timer: None },
            Self::PcmPhase { dem: true, threshold: ThresholdSource::Internal, preferred_timer: None },
            Self::PcmPhase { dem: true, threshold: ThresholdSource::InternalExtZcd, preferred_timer: None },
            Self::PcmPhase { dem: false, threshold: ThresholdSource::External, preferred_timer: None },
            Self::PcmPhase { dem: true, threshold: ThresholdSource::External, preferred_timer: None },
            Self::ShortCircuitFault,
            Self::ShareBusDrive,
            Self::AdcSequencer {
                adc: AdcInstance::Adc1,
                kind: SequencerKind::DualRegular,
                trigger: TriggerSource::Event(CrossbarSource::Mcr1),
            },
            Self::AdcSequencer {
                adc: AdcInstance::Adc1,
                kind: SequencerKind::Regular,
                trigger: TriggerSource::Software,
            },
            Self::AdcConversion {
                group: 0, purpose: MonitorPurpose::VOut,
                adc_pref: None, speed: SpeedPref::Any,
            },
            Self::AdcConversion {
                group: 0, purpose: MonitorPurpose::VIn,
                adc_pref: None, speed: SpeedPref::Any,
            },
            Self::AdcConversion {
                group: 0, purpose: MonitorPurpose::Ntc,
                adc_pref: None, speed: SpeedPref::Any,
            },
            Self::AdcConversion {
                group: 0, purpose: MonitorPurpose::PhaseCurrent(1),
                adc_pref: None, speed: SpeedPref::Fast,
            },
            Self::UseOpamp {
                instance: OpampId::Opamp1,
                external_vinp: true,
                external_vinm: true,
                external_vout: true,
            },
            Self::UseSpi { instance: SpiId::Spi1, needs_miso: true, needs_nss: false },
            Self::UseI2c { instance: I2cId::I2c1, needs_smba: false },
            Self::UseUsart { instance: UsartId::Usart1, flow_control: false, synchronous: false },
            Self::UseUart { instance: UartId::Uart4, flow_control: false },
            Self::UseLpuart { instance: LpuartId::Lpuart1, flow_control: false },
            Self::UseCan { instance: CanId::Fdcan1 },
            Self::UseUsb,
            Self::UseUcpd { instance: UcpdId::Ucpd1 },
            Self::UseTim {
                instance: TimId::Tim2, channels_mask: 0b0011,
                complementary: false, bkin: false, etr: false,
            },
            Self::UseTim {
                instance: TimId::Tim1, channels_mask: 0b0011,
                complementary: true, bkin: true, etr: false,
            },
        ]
    }

    pub fn name(self) -> String {
        match self {
            Self::PcmPhase { dem, threshold, preferred_timer } => {
                let base = match (dem, threshold) {
                    (false, ThresholdSource::Internal) => "PCM phase",
                    (true,  ThresholdSource::Internal) => "PCM phase + DEM",
                    (_,     ThresholdSource::InternalExtZcd) => "PCM phase + DEM (int peak, ext ZCD)",
                    (false, ThresholdSource::External) => "PCM phase (ext COMP)",
                    (true,  ThresholdSource::External) => "PCM phase + DEM (ext COMP)",
                };
                match preferred_timer {
                    Some(t) => format!("{} on {:?}", base, t),
                    None => base.to_string(),
                }
            }
            Self::ShortCircuitFault => "Short-circuit fault".into(),
            Self::ShareBusDrive => "Share-bus drive".into(),
            Self::AdcSequencer { adc, kind, trigger } => {
                let trig = match trigger {
                    TriggerSource::Software => "SW".into(),
                    TriggerSource::Event(e) => format!("{:?}", e),
                };
                format!("ADC sequencer: {:?} {:?} ({})", adc, kind, trig)
            }
            Self::AdcConversion { purpose, adc_pref, speed, .. } => {
                let adc = adc_pref.map(|a| format!(" ADC{}", a.number())).unwrap_or_default();
                let sp = if speed == SpeedPref::Any { String::new() } else { format!(" [{}]", speed.short()) };
                format!("ADC conversion: {}{}{}", purpose.short_name(), adc, sp)
            }
            Self::UseOpamp { instance, .. } => format!("OPAMP{}", instance.number()),
            Self::UseSpi { instance, .. } => format!("SPI{}", instance.number()),
            Self::UseI2c { instance, .. } => format!("I2C{}", instance.number()),
            Self::UseUsart { instance, synchronous, .. } => {
                if synchronous { format!("USART{} (sync)", instance.number()) }
                else { format!("USART{}", instance.number()) }
            }
            Self::UseUart { instance, .. } => format!("UART{}", instance.number()),
            Self::UseLpuart { .. } => "LPUART1".to_string(),
            Self::UseCan { instance } => format!("FDCAN{}", instance.number()),
            Self::UseUsb => "USB".to_string(),
            Self::UseUcpd { .. } => "UCPD1".to_string(),
            Self::UseTim { instance, complementary, bkin, .. } => {
                let mut tags: Vec<&str> = Vec::new();
                if complementary { tags.push("±"); }
                if bkin { tags.push("BKIN"); }
                if tags.is_empty() { format!("TIM{}", instance.number()) }
                else { format!("TIM{} [{}]", instance.number(), tags.join(",")) }
            }
        }
    }

    pub fn enumerate(
        self,
        used: &ResourceBag,
        all_specs: &[(u32, RequirementSpec)],
        variant: crate::pinout::ChipVariant,
    ) -> Vec<Assignment> {
        match self {
            Self::PcmPhase { dem, threshold: ThresholdSource::Internal, preferred_timer } => {
                enumerate_pcm_phase_internal(dem, None, preferred_timer, used)
            }
            Self::PcmPhase { threshold: ThresholdSource::InternalExtZcd, preferred_timer, .. } => {
                enumerate_pcm_phase_hybrid(preferred_timer, used)
            }
            Self::PcmPhase { dem, threshold: ThresholdSource::External, preferred_timer } => {
                enumerate_pcm_phase_external(dem, preferred_timer, used)
            }
            Self::ShortCircuitFault => enumerate_fault(used),
            Self::ShareBusDrive => enumerate_drive(used),
            Self::AdcSequencer { adc, kind, trigger } => {
                enumerate_sequencer(adc, kind, trigger, used)
            }
            Self::AdcConversion { group, purpose, adc_pref, speed } => {
                enumerate_conversion(group, purpose, adc_pref, speed, used, all_specs, variant)
            }
            Self::UseOpamp { instance, external_vinp, external_vinm, external_vout } => {
                enumerate_opamp(instance, external_vinp, external_vinm, external_vout, used)
            }
            Self::UseSpi { instance, needs_miso, needs_nss } => {
                if used.contains(&Resource::Spi(instance)) { Vec::new() }
                else { vec![Assignment::Spi { instance, needs_miso, needs_nss }] }
            }
            Self::UseI2c { instance, needs_smba } => {
                if used.contains(&Resource::I2c(instance)) { Vec::new() }
                else { vec![Assignment::I2c { instance, needs_smba }] }
            }
            Self::UseUsart { instance, flow_control, synchronous } => {
                if used.contains(&Resource::Usart(instance)) { Vec::new() }
                else { vec![Assignment::Usart { instance, flow_control, synchronous }] }
            }
            Self::UseUart { instance, flow_control } => {
                if used.contains(&Resource::Uart(instance)) { Vec::new() }
                else { vec![Assignment::Uart { instance, flow_control }] }
            }
            Self::UseLpuart { instance, flow_control } => {
                if used.contains(&Resource::Lpuart(instance)) { Vec::new() }
                else { vec![Assignment::Lpuart { instance, flow_control }] }
            }
            Self::UseCan { instance } => {
                if used.contains(&Resource::Can(instance)) { Vec::new() }
                else { vec![Assignment::Can { instance }] }
            }
            Self::UseUsb => {
                if used.contains(&Resource::Usb) { Vec::new() }
                else { vec![Assignment::Usb] }
            }
            Self::UseUcpd { instance } => {
                if used.contains(&Resource::Ucpd(instance)) { Vec::new() }
                else { vec![Assignment::Ucpd { instance }] }
            }
            Self::UseTim { instance, channels_mask, complementary, bkin, etr } => {
                if used.contains(&Resource::Tim(instance)) { Vec::new() }
                else { vec![Assignment::Tim { instance, channels_mask, complementary, bkin, etr }] }
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Assignment {
    PcmPhase {
        alloc: PhaseAllocation,
        /// External-ZCD EEV when using the hybrid "internal peak, external
        /// ZCD" path. `None` for pure-internal (INM-swap DEM) or non-DEM.
        zcd_eev: Option<CrossbarSource>,
        timer: HrtimId,
        dem: bool,
    },
    PcmPhaseExternal {
        dac: Option<DacId>,
        peak_eev: CrossbarSource,
        zcd_eev: Option<CrossbarSource>,
        timer: HrtimId,
        dem: bool,
    },
    ShortCircuitFault(ShortCircuitFault),
    ShareBusDrive(ShareBusDrive),
    AdcSequencer {
        adc: AdcInstance,
        kind: SequencerKind,
        trigger: TriggerSource,
        /// Which of the 10 HRTIM ADC-trigger crossbar slots (TRIG1..TRIG10)
        /// is used to route the trigger event into this ADC. `None` when
        /// the sequencer is software-triggered. Chosen by the solver so
        /// that two hardware-triggered sequencers don't double-book the
        /// same slot.
        #[serde(default)]
        trig_slot: Option<AdcTriggerId>,
    },
    AdcConversion {
        /// Physical ADC + input-channel the conversion lands on.
        adc: AdcInstance,
        channel: u8,
        purpose: MonitorPurpose,
    },
    Opamp {
        instance: OpampId,
        external_vinp: bool,
        external_vinm: bool,
        external_vout: bool,
    },
    Spi { instance: SpiId, needs_miso: bool, needs_nss: bool },
    I2c { instance: I2cId, needs_smba: bool },
    Usart { instance: UsartId, flow_control: bool, synchronous: bool },
    Uart { instance: UartId, flow_control: bool },
    Lpuart { instance: LpuartId, flow_control: bool },
    Can { instance: CanId },
    Usb,
    Ucpd { instance: UcpdId },
    Tim {
        instance: TimId,
        channels_mask: u8,
        complementary: bool,
        bkin: bool,
        etr: bool,
    },
}

impl Assignment {
    pub fn consumed(&self) -> Vec<Resource> {
        match self {
            Self::PcmPhase { alloc, zcd_eev, timer, dem } => {
                let mut r = vec![
                    Resource::Dac(alloc.dac),
                    Resource::Comp(alloc.comp),
                    Resource::Eev(alloc.eev),
                    Resource::Timer(*timer),
                    Resource::TimerSlot(*timer, TimerCompareSlot::Cr2),
                ];
                if let Some(z) = zcd_eev {
                    r.push(Resource::Eev(*z));
                }
                if *dem {
                    r.push(Resource::TimerSlot(*timer, TimerCompareSlot::Cr4));
                    r.push(Resource::TimerCapture(*timer, TimerCaptureUnit::Cpt2));
                }
                r
            }
            Self::PcmPhaseExternal { dac, peak_eev, zcd_eev, timer, dem } => {
                let mut r = vec![Resource::Eev(*peak_eev), Resource::Timer(*timer)];
                if let Some(d) = dac {
                    r.push(Resource::Dac(*d));
                }
                if let Some(z) = zcd_eev {
                    r.push(Resource::Eev(*z));
                }
                if *dem {
                    r.push(Resource::TimerSlot(*timer, TimerCompareSlot::Cr4));
                    r.push(Resource::TimerCapture(*timer, TimerCaptureUnit::Cpt2));
                }
                r
            }
            Self::ShortCircuitFault(f) => vec![
                Resource::Dac(f.dac),
                Resource::Comp(f.comp),
                Resource::Flt(f.flt),
            ],
            Self::ShareBusDrive(d) => vec![Resource::Dac(d.dac)],
            Self::AdcSequencer { adc, kind, trigger, trig_slot } => {
                let mut r = vec![Resource::AdcSequencer(*adc, *kind)];
                if let Some(t) = trig_slot {
                    r.push(Resource::AdcTrigger(*t));
                }
                if kind.is_dual() {
                    // Claim the slave ADC's same-kind sequencer too.
                    let slave_kind = match kind {
                        SequencerKind::DualRegular => SequencerKind::Regular,
                        SequencerKind::DualInjected => SequencerKind::Injected,
                        _ => unreachable!(),
                    };
                    let slave = match adc {
                        AdcInstance::Adc1 => AdcInstance::Adc2,
                        AdcInstance::Adc3 => AdcInstance::Adc4,
                        _ => return r, // invalid (caught by enumerator)
                    };
                    r.push(Resource::AdcSequencer(slave, slave_kind));
                }
                if let TriggerSource::Event(ev) = trigger {
                    // Claim whichever compare-slot the event depends on —
                    // sub-timer CR2/3/4 or master CR1..4. Period / reset /
                    // EEV events don't consume a compare register.
                    if let Some(slot) = compare_slot_for_event(*ev) {
                        r.push(slot);
                    }
                }
                r
            }
            Self::AdcConversion { adc, channel, .. } => {
                vec![Resource::AdcInput(*adc, *channel)]
            }
            Self::Opamp { instance, .. } => {
                // Pin claims are materialized via forced_pin_claims when
                // the user pins a specific VINP/VINM/VOUT option — the
                // OPAMP itself just claims its instance here.
                vec![Resource::Opamp(*instance)]
            }
            Self::Spi { instance, .. }    => vec![Resource::Spi(*instance)],
            Self::I2c { instance, .. }    => vec![Resource::I2c(*instance)],
            Self::Usart { instance, .. }  => vec![Resource::Usart(*instance)],
            Self::Uart { instance, .. }   => vec![Resource::Uart(*instance)],
            Self::Lpuart { instance, .. } => vec![Resource::Lpuart(*instance)],
            Self::Can { instance }        => vec![Resource::Can(*instance)],
            Self::Usb                     => vec![Resource::Usb],
            Self::Ucpd { instance }       => vec![Resource::Ucpd(*instance)],
            Self::Tim { instance, .. }    => vec![Resource::Tim(*instance)],
        }
    }

    pub fn label(&self) -> String {
        match self {
            Self::PcmPhase { alloc, zcd_eev, timer, dem } => {
                let zcd = match zcd_eev {
                    Some(z) => format!(", extZCD={:?}", z),
                    None => String::new(),
                };
                format!(
                    "{:?}: {:?} -> {:?} -> {:?}{}{}",
                    timer, alloc.dac, alloc.comp, alloc.eev, zcd,
                    if *dem { " +DEM" } else { "" }
                )
            }
            Self::PcmPhaseExternal { dac, peak_eev, zcd_eev, timer, dem } => {
                let src = match dac {
                    Some(d) => format!("{:?}", d),
                    None => "fixed-ref".to_string(),
                };
                let zcd = match zcd_eev {
                    Some(z) => format!(", ZCD={:?}", z),
                    None => String::new(),
                };
                format!(
                    "{:?}: [ext] {} -> peak={:?}{}{}",
                    timer, src, peak_eev, zcd,
                    if *dem { " +DEM" } else { "" }
                )
            }
            Self::ShortCircuitFault(f) => {
                format!("{:?} -> {:?} -> {:?}", f.dac, f.comp, f.flt)
            }
            Self::ShareBusDrive(d) => format!("{:?}", d.dac),
            Self::AdcSequencer { adc, kind, trigger, trig_slot } => {
                let trig = match trigger {
                    TriggerSource::Software => "SW".to_string(),
                    TriggerSource::Event(e) => format!("{:?}", e),
                };
                let slot = match trig_slot {
                    Some(t) => format!(" via {:?}", t),
                    None => String::new(),
                };
                format!("{:?} {:?} trig={}{}", adc, kind, trig, slot)
            }
            Self::AdcConversion { adc, channel, purpose } => {
                let pin = crate::pinout::pins_for(
                    crate::pinout::Signal::AdcIn { adc: *adc, channel: *channel },
                    crate::pinout::ChipVariant::G474R,
                )
                .first()
                .map(|p| format!(" @ {}", p.name()))
                .unwrap_or_default();
                let speed = if crate::g474::is_fast_adc_channel(*channel) { "fast" } else { "slow" };
                format!(
                    "{} -> ADC{} [{}] IN{}{}",
                    purpose.short_name(), adc.number(), speed, channel, pin,
                )
            }
            Self::Opamp { instance, external_vinp, external_vinm, external_vout } => {
                let mut tags: Vec<&str> = Vec::new();
                if *external_vinp { tags.push("VINP"); }
                if *external_vinm { tags.push("VINM"); }
                if *external_vout { tags.push("VOUT"); }
                if tags.is_empty() {
                    format!("OPAMP{} (all-internal)", instance.number())
                } else {
                    format!("OPAMP{} ext: {}", instance.number(), tags.join(","))
                }
            }
            Self::Spi { instance, needs_miso, needs_nss } => {
                let mut tags = vec!["MOSI", "SCK"];
                if *needs_miso { tags.push("MISO"); }
                if *needs_nss { tags.push("NSS"); }
                format!("SPI{} [{}]", instance.number(), tags.join(","))
            }
            Self::I2c { instance, needs_smba } => {
                let extra = if *needs_smba { ",SMBA" } else { "" };
                format!("I2C{} [SDA,SCL{}]", instance.number(), extra)
            }
            Self::Usart { instance, flow_control, synchronous } => {
                let mut tags = vec!["TX", "RX"];
                if *flow_control { tags.push("CTS"); tags.push("RTS"); }
                if *synchronous { tags.push("CK"); }
                format!("USART{} [{}]", instance.number(), tags.join(","))
            }
            Self::Uart { instance, flow_control } => {
                let tags = if *flow_control { "TX,RX,CTS,RTS" } else { "TX,RX" };
                format!("UART{} [{}]", instance.number(), tags)
            }
            Self::Lpuart { flow_control, .. } => {
                let tags = if *flow_control { "TX,RX,CTS,RTS" } else { "TX,RX" };
                format!("LPUART1 [{}]", tags)
            }
            Self::Can { instance } => format!("FDCAN{} [TX,RX]", instance.number()),
            Self::Usb => "USB [DP,DM]".to_string(),
            Self::Ucpd { .. } => "UCPD1 [CC1,CC2]".to_string(),
            Self::Tim { instance, channels_mask, complementary, bkin, etr } => {
                let mut tags: Vec<String> = Vec::new();
                for i in 0..4u8 {
                    if channels_mask & (1 << i) != 0 {
                        tags.push(format!("CH{}", i + 1));
                        if *complementary { tags.push(format!("CH{}N", i + 1)); }
                    }
                }
                if *bkin { tags.push("BKIN".to_string()); }
                if *etr { tags.push("ETR".to_string()); }
                format!("TIM{} [{}]", instance.number(), tags.join(","))
            }
        }
    }
}

const FAST_DACS: &[DacId] = &[
    DacId::Dac3Ch1, DacId::Dac3Ch2, DacId::Dac4Ch1, DacId::Dac4Ch2,
];
const SLOW_DACS: &[DacId] = &[DacId::Dac1Ch1, DacId::Dac1Ch2, DacId::Dac2Ch1];
const ALL_TIMERS: &[HrtimId] = &[
    HrtimId::TimA, HrtimId::TimB, HrtimId::TimC,
    HrtimId::TimD, HrtimId::TimE, HrtimId::TimF,
];

fn enumerate_pcm_phase_internal(
    dem: bool,
    zcd_eev_override: Option<CrossbarSource>,
    preferred_timer: Option<HrtimId>,
    used: &ResourceBag,
) -> Vec<Assignment> {
    let mut out = Vec::new();
    let pinned = preferred_timer.map(|t| [t]);
    let timers: &[HrtimId] = match &pinned {
        Some(arr) => arr,
        None => ALL_TIMERS,
    };
    for &dac in FAST_DACS {
        if used.contains(&Resource::Dac(dac)) { continue; }
        for &comp in comps_for_dac(dac) {
            if used.contains(&Resource::Comp(comp)) { continue; }
            for &eev in eevs_for_comp(comp) {
                if used.contains(&Resource::Eev(eev)) { continue; }
                for &timer in timers {
                    if used.contains(&Resource::Timer(timer)) { continue; }
                    if used.contains(&Resource::TimerSlot(timer, TimerCompareSlot::Cr2)) { continue; }
                    if dem && used.contains(&Resource::TimerSlot(timer, TimerCompareSlot::Cr4)) { continue; }
                    out.push(Assignment::PcmPhase {
                        alloc: PhaseAllocation { dac, comp, eev },
                        zcd_eev: zcd_eev_override,
                        timer,
                        dem,
                    });
                }
            }
        }
    }
    out
}

fn enumerate_pcm_phase_hybrid(
    preferred_timer: Option<HrtimId>,
    used: &ResourceBag,
) -> Vec<Assignment> {
    const EEVS: &[CrossbarSource] = &[
        CrossbarSource::Eev1, CrossbarSource::Eev2, CrossbarSource::Eev3,
        CrossbarSource::Eev4, CrossbarSource::Eev5, CrossbarSource::Eev6,
        CrossbarSource::Eev7, CrossbarSource::Eev8, CrossbarSource::Eev9,
        CrossbarSource::Eev10,
    ];
    let mut out = Vec::new();
    for &zcd in EEVS {
        if used.contains(&Resource::Eev(zcd)) { continue; }
        for candidate in enumerate_pcm_phase_internal(true, Some(zcd), preferred_timer, used) {
            if let Assignment::PcmPhase { alloc, .. } = &candidate {
                if alloc.eev == zcd { continue; }
            }
            out.push(candidate);
        }
    }
    out
}

fn enumerate_pcm_phase_external(
    dem: bool,
    preferred_timer: Option<HrtimId>,
    used: &ResourceBag,
) -> Vec<Assignment> {
    const EEVS: &[CrossbarSource] = &[
        CrossbarSource::Eev1, CrossbarSource::Eev2, CrossbarSource::Eev3,
        CrossbarSource::Eev4, CrossbarSource::Eev5, CrossbarSource::Eev6,
        CrossbarSource::Eev7, CrossbarSource::Eev8, CrossbarSource::Eev9,
        CrossbarSource::Eev10,
    ];
    let mut out = Vec::new();
    let dac_options: Vec<Option<DacId>> = {
        let mut v = vec![None];
        for &d in SLOW_DACS {
            if !used.contains(&Resource::Dac(d)) { v.push(Some(d)); }
        }
        v
    };
    let pinned = preferred_timer.map(|t| [t]);
    let timers: &[HrtimId] = match &pinned {
        Some(arr) => arr,
        None => ALL_TIMERS,
    };
    for &peak_eev in EEVS {
        if used.contains(&Resource::Eev(peak_eev)) { continue; }
        let zcd_options: Vec<Option<CrossbarSource>> = if dem {
            EEVS.iter().copied()
                .filter(|e| *e != peak_eev && !used.contains(&Resource::Eev(*e)))
                .map(Some).collect()
        } else { vec![None] };
        for &timer in timers {
            if used.contains(&Resource::Timer(timer)) { continue; }
            if dem && used.contains(&Resource::TimerSlot(timer, TimerCompareSlot::Cr4)) { continue; }
            for dac in &dac_options {
                for zcd in &zcd_options {
                    out.push(Assignment::PcmPhaseExternal {
                        dac: *dac, peak_eev, zcd_eev: *zcd, timer, dem,
                    });
                }
            }
        }
    }
    out
}

fn enumerate_fault(used: &ResourceBag) -> Vec<Assignment> {
    let mut out = Vec::new();
    for &dac in SLOW_DACS {
        if used.contains(&Resource::Dac(dac)) { continue; }
        for &comp in comps_for_dac(dac) {
            if used.contains(&Resource::Comp(comp)) { continue; }
            for &flt in flts_for_comp(comp) {
                if used.contains(&Resource::Flt(flt)) { continue; }
                out.push(Assignment::ShortCircuitFault(ShortCircuitFault { dac, comp, flt }));
            }
        }
    }
    out
}

fn enumerate_opamp(
    instance: OpampId,
    external_vinp: bool,
    external_vinm: bool,
    external_vout: bool,
    used: &ResourceBag,
) -> Vec<Assignment> {
    if used.contains(&Resource::Opamp(instance)) {
        return Vec::new();
    }
    vec![Assignment::Opamp {
        instance,
        external_vinp,
        external_vinm,
        external_vout,
    }]
}

fn enumerate_drive(used: &ResourceBag) -> Vec<Assignment> {
    [DacId::Dac1Ch1, DacId::Dac1Ch2, DacId::Dac2Ch1]
        .iter().copied()
        .filter(|d| !used.contains(&Resource::Dac(*d)))
        .map(|dac| Assignment::ShareBusDrive(ShareBusDrive { dac }))
        .collect()
}

fn enumerate_sequencer(
    adc: AdcInstance,
    kind: SequencerKind,
    trigger: TriggerSource,
    used: &ResourceBag,
) -> Vec<Assignment> {
    // Dual modes are only valid on Adc1 or Adc3 (they're the masters of
    // their respective dual pairs).
    if kind.is_dual() && !matches!(adc, AdcInstance::Adc1 | AdcInstance::Adc3) {
        return Vec::new();
    }
    if used.contains(&Resource::AdcSequencer(adc, kind)) {
        return Vec::new();
    }
    // If the trigger event depends on a compare slot, that slot must be free.
    if let TriggerSource::Event(ev) = trigger {
        if let Some(slot) = compare_slot_for_event(ev) {
            if used.contains(&slot) {
                return Vec::new();
            }
        }
    }
    if kind.is_dual() {
        let slave_kind = match kind {
            SequencerKind::DualRegular => SequencerKind::Regular,
            SequencerKind::DualInjected => SequencerKind::Injected,
            _ => unreachable!(),
        };
        let slave = match adc {
            AdcInstance::Adc1 => AdcInstance::Adc2,
            AdcInstance::Adc3 => AdcInstance::Adc4,
            _ => unreachable!(),
        };
        if used.contains(&Resource::AdcSequencer(slave, slave_kind)) {
            return Vec::new();
        }
    }
    match trigger {
        TriggerSource::Software => {
            // Software trigger doesn't route through the crossbar — no slot.
            vec![Assignment::AdcSequencer { adc, kind, trigger, trig_slot: None }]
        }
        TriggerSource::Event(ev) => {
            let slots = eligible_trig_slots(ev, adc, kind);
            slots
                .into_iter()
                .filter(|s| !used.contains(&Resource::AdcTrigger(*s)))
                .map(|s| Assignment::AdcSequencer {
                    adc, kind, trigger, trig_slot: Some(s),
                })
                .collect()
        }
    }
}

/// TRIG1..10 slots that can both (a) route the given crossbar event and
/// (b) feed this ADC's regular/injected sequencer. Per RM0440 §28 the
/// crossbar routes are fixed; ST splits them into odd/even slots for
/// different ADC-channel classes.
fn eligible_trig_slots(
    ev: CrossbarSource,
    adc: AdcInstance,
    kind: SequencerKind,
) -> Vec<AdcTriggerId> {
    // Dual modes trigger the master ADC only; the slave follows in HW.
    let (effective_adc, regular) = match kind {
        SequencerKind::Regular => (adc, true),
        SequencerKind::DualRegular => (adc, true),
        SequencerKind::Injected => (adc, false),
        SequencerKind::DualInjected => (adc, false),
    };
    let target_channel = match (effective_adc, regular) {
        (AdcInstance::Adc1, true)  => AdcChannel::Adc1Regular,
        (AdcInstance::Adc1, false) => AdcChannel::Adc1Injected,
        (AdcInstance::Adc2, true)  => AdcChannel::Adc2Regular,
        (AdcInstance::Adc2, false) => AdcChannel::Adc2Injected,
        (AdcInstance::Adc3, true)  => AdcChannel::Adc3Regular,
        (AdcInstance::Adc3, false) => AdcChannel::Adc3Injected,
        (AdcInstance::Adc4, true)  => AdcChannel::Adc4Regular,
        (AdcInstance::Adc4, false) => AdcChannel::Adc4Injected,
        (AdcInstance::Adc5, true)  => AdcChannel::Adc5Regular,
        (AdcInstance::Adc5, false) => AdcChannel::Adc5Injected,
    };
    let event_slots: &[AdcTriggerId] = crate::g474::adc_triggers_for(ev);
    event_slots
        .iter()
        .copied()
        .filter(|trig| crate::g474::adc_channels_for(*trig).contains(&target_channel))
        .collect()
}

fn enumerate_conversion(
    group: u32,
    purpose: MonitorPurpose,
    adc_pref: Option<AdcInstance>,
    speed: SpeedPref,
    used: &ResourceBag,
    all_specs: &[(u32, RequirementSpec)],
    variant: crate::pinout::ChipVariant,
) -> Vec<Assignment> {
    // Find the parent sequencer's ADC and kind. If the group is stale or
    // doesn't point at a sequencer, yield nothing.
    let parent = all_specs.iter().find_map(|(id, spec)| {
        if *id != group { return None; }
        match spec {
            RequirementSpec::AdcSequencer { adc, kind, .. } => Some((*adc, *kind)),
            _ => None,
        }
    });
    let Some((parent_adc, parent_kind)) = parent else { return Vec::new(); };
    // For dual modes, both ADCs of the coupled pair contribute candidates
    // (one conversion per ADC per simultaneous slot).
    let mut adcs: Vec<AdcInstance> = if parent_kind.is_dual() {
        match parent_adc {
            AdcInstance::Adc1 => vec![AdcInstance::Adc1, AdcInstance::Adc2],
            AdcInstance::Adc3 => vec![AdcInstance::Adc3, AdcInstance::Adc4],
            other => vec![other],
        }
    } else {
        vec![parent_adc]
    };
    // If user pinned adc_pref, narrow the pool (ignored silently when it
    // doesn't intersect — caller shows a warning).
    if let Some(pref) = adc_pref {
        adcs.retain(|a| *a == pref);
    }
    let mut out = Vec::new();
    for adc in adcs {
        for ch in 1..=18u8 {
            if !speed.allows(ch) { continue; }
            if used.contains(&Resource::AdcInput(adc, ch)) { continue; }
            let pins = crate::pinout::pins_for(
                crate::pinout::Signal::AdcIn { adc, channel: ch },
                variant,
            );
            let Some(pin) = pins.first() else { continue; };
            // Skip if pin already claimed by another fabric signal.
            if used.contains(&Resource::Pin(*pin)) { continue; }
            out.push(Assignment::AdcConversion { adc, channel: ch, purpose });
        }
    }
    out
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Severity {
    Ok,
    Info,
    Warn,
}

#[derive(Clone, Debug)]
pub struct CapabilityCheck {
    pub severity: Severity,
    pub message: String,
    pub fix: Option<Fix>,
}

#[derive(Clone, Debug)]
pub enum Fix {
    AddRequirement(RequirementSpec),
    ConvertRequirementSpec { idx: usize, spec: RequirementSpec },
}

impl Fix {
    pub fn label(&self) -> String {
        match self {
            Fix::AddRequirement(spec) => format!("Add {}", spec.name()),
            Fix::ConvertRequirementSpec { idx, spec } => {
                format!("Convert #{} -> {}", idx + 1, spec.name())
            }
        }
    }
}

fn ok(m: impl Into<String>) -> CapabilityCheck {
    CapabilityCheck { severity: Severity::Ok, message: m.into(), fix: None }
}
fn info(m: impl Into<String>) -> CapabilityCheck {
    CapabilityCheck { severity: Severity::Info, message: m.into(), fix: None }
}
fn warn(m: impl Into<String>) -> CapabilityCheck {
    CapabilityCheck { severity: Severity::Warn, message: m.into(), fix: None }
}
fn warn_with(m: impl Into<String>, fix: Fix) -> CapabilityCheck {
    CapabilityCheck { severity: Severity::Warn, message: m.into(), fix: Some(fix) }
}

/// Logical HRTIM timer-output event slot.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum TimerInputSlot {
    Set1, Rst1, Set2, Rst2,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct PhaseEdge {
    pub eev: CrossbarSource,
    pub timer: HrtimId,
    pub slot: TimerInputSlot,
}

fn default_variant() -> crate::pinout::ChipVariant {
    crate::pinout::ChipVariant::G474R
}
fn default_format_version() -> u32 { DESIGN_FORMAT_VERSION }

/// Bump whenever the Design schema changes in a way that can't safely
/// round-trip older saves. The app's STORAGE_KEY is bumped in lockstep
/// so old saves don't load under a newer schema.
pub const DESIGN_FORMAT_VERSION: u32 = 2;

#[derive(Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Design {
    #[serde(default = "default_format_version")]
    pub format_version: u32,
    pub ids: Vec<u32>,
    pub requirements: Vec<RequirementSpec>,
    pub assignments: Vec<Option<Assignment>>,
    pub locks: ResourceBag,
    #[serde(default)]
    pub pin_assignments: std::collections::HashMap<crate::pinout::Signal, crate::pinout::Pin>,
    pub next_id: u32,
    /// Chip package variant. Drives per-package pin-availability in the
    /// AF table; persisted with the design so loading a saved file
    /// restores the correct package-specific pin lookups.
    #[serde(default = "default_variant")]
    pub variant: crate::pinout::ChipVariant,
}

impl Default for Design {
    fn default() -> Self {
        let mut d = Self {
            format_version: DESIGN_FORMAT_VERSION,
            ids: Vec::new(),
            requirements: Vec::new(),
            assignments: Vec::new(),
            locks: ResourceBag::new(),
            pin_assignments: std::collections::HashMap::new(),
            next_id: 1,
            variant: default_variant(),
        };
        for _ in 0..4 {
            d.add(RequirementSpec::PcmPhase {
                dem: false, threshold: ThresholdSource::Internal, preferred_timer: None,
            });
        }
        d.add(RequirementSpec::ShortCircuitFault);
        d.add(RequirementSpec::ShareBusDrive);
        let seq_id = d.next_id;
        d.add(RequirementSpec::AdcSequencer {
            adc: AdcInstance::Adc1,
            kind: SequencerKind::DualRegular,
            trigger: TriggerSource::Event(CrossbarSource::Mcr1),
        });
        d.add(RequirementSpec::AdcConversion {
            group: seq_id, purpose: MonitorPurpose::VOut,
            adc_pref: None, speed: SpeedPref::Any,
        });
        d.add(RequirementSpec::AdcConversion {
            group: seq_id, purpose: MonitorPurpose::VIn,
            adc_pref: None, speed: SpeedPref::Any,
        });
        d.add(RequirementSpec::AdcConversion {
            group: seq_id, purpose: MonitorPurpose::Ntc,
            adc_pref: None, speed: SpeedPref::Any,
        });
        d.normalize();
        d
    }
}

impl Design {
    fn alloc_id(&mut self) -> u32 {
        let id = self.next_id;
        self.next_id = self.next_id.checked_add(1).unwrap_or(1);
        id
    }

    pub fn add(&mut self, spec: RequirementSpec) -> u32 {
        // Smart default for conversions added via the palette (group: 0 is
        // the placeholder): attach to the first existing sequencer so the
        // new conversion has a chance of being assignable.
        let spec = match spec {
            RequirementSpec::AdcConversion { group: 0, purpose, adc_pref, speed } => {
                let first = self.sequencer_ids().first().map(|(id, _)| *id).unwrap_or(0);
                RequirementSpec::AdcConversion { group: first, purpose, adc_pref, speed }
            }
            other => other,
        };
        let id = self.alloc_id();
        self.ids.push(id);
        self.requirements.push(spec);
        self.assignments.push(None);
        self.normalize();
        id
    }

    /// Remove requirement at `idx`. If it's an `AdcSequencer`, cascade-delete
    /// any `AdcConversion`s that reference it (per your decision #3a).
    pub fn remove(&mut self, idx: usize) {
        let removed_id = self.ids[idx];
        let removing_sequencer = matches!(
            self.requirements[idx],
            RequirementSpec::AdcSequencer { .. }
        );
        self.ids.remove(idx);
        self.requirements.remove(idx);
        self.assignments.remove(idx);
        if removing_sequencer {
            let mut i = 0;
            while i < self.requirements.len() {
                if matches!(
                    self.requirements[i],
                    RequirementSpec::AdcConversion { group, .. } if group == removed_id
                ) {
                    self.ids.remove(i);
                    self.requirements.remove(i);
                    self.assignments.remove(i);
                    continue;
                }
                i += 1;
            }
        }
        self.normalize();
    }

    pub fn set_spec(&mut self, idx: usize, spec: RequirementSpec) {
        self.requirements[idx] = spec;
        self.assignments[idx] = None;
        self.normalize();
    }

    pub fn set_assignment(&mut self, idx: usize, assignment: Assignment) {
        self.assignments[idx] = Some(assignment);
        self.normalize();
    }

    pub fn apply_fix(&mut self, fix: Fix) {
        match fix {
            Fix::AddRequirement(spec) => {
                self.add(spec);
            }
            Fix::ConvertRequirementSpec { idx, spec } => self.set_spec(idx, spec),
        }
    }

    pub fn set_pin(&mut self, signal: crate::pinout::Signal, pin: crate::pinout::Pin) {
        self.pin_assignments.insert(signal, pin);
    }
    pub fn clear_pin(&mut self, signal: crate::pinout::Signal) {
        self.pin_assignments.remove(&signal);
    }
    pub fn clear_all_pins(&mut self) { self.pin_assignments.clear(); }

    pub fn set_variant(&mut self, variant: crate::pinout::ChipVariant) {
        self.variant = variant;
        self.normalize();
    }

    pub fn toggle_lock(&mut self, r: Resource) {
        if !self.locks.remove(&r) {
            self.locks.insert(r);
        }
        self.normalize();
    }
    pub fn clear_lock(&mut self, r: Resource) {
        self.locks.remove(&r);
        self.normalize();
    }

    pub fn normalize(&mut self) {
        for _ in 0..8 {
            let mut changed = false;
            // Take a snapshot of all (id, spec) for enumerators that
            // cross-reference (AdcConversion → parent sequencer).
            let spec_refs: Vec<(u32, RequirementSpec)> = self
                .ids
                .iter()
                .copied()
                .zip(self.requirements.iter().copied())
                .collect();
            for i in 0..self.requirements.len() {
                let used = self.used_excluding(i);
                let cands = self.requirements[i].enumerate(&used, &spec_refs, self.variant);
                let current = self.assignments[i].clone();
                match current {
                    Some(ref a) if !cands.contains(a) => {
                        self.assignments[i] = None;
                        changed = true;
                    }
                    None => {
                        if let Some(first) = cands.into_iter().next() {
                            self.assignments[i] = Some(first);
                            changed = true;
                        }
                    }
                    _ => {}
                }
            }
            if !changed { break; }
        }
    }

    pub fn used_excluding(&self, idx: usize) -> ResourceBag {
        let mut bag = self.locks.clone();
        bag.extend(
            self.assignments
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != idx)
                .filter_map(|(_, a)| a.as_ref())
                .flat_map(|a| a.consumed()),
        );
        // Fold in pin claims for signals that resolve to a single pin —
        // either because the user explicitly pinned them, or because the
        // chip variant only offers one pin for that signal. Lets ADC
        // conversion enumeration avoid landing on a pin already required
        // by an HRTIM channel / DAC output / forced FLT input / etc.
        bag.extend(self.forced_pin_claims(idx));
        bag
    }

    fn forced_pin_claims(&self, exclude_idx: usize) -> Vec<Resource> {
        use crate::pinout::{pins_for, HrtimCh, Signal};
        let mut out = Vec::new();
        let variant = self.variant;
        for (i, asn) in self.assignments.iter().enumerate() {
            if i == exclude_idx { continue; }
            let Some(asn) = asn else { continue; };
            let signals: Vec<Signal> = match asn {
                Assignment::PcmPhase { timer, zcd_eev, .. } => {
                    let mut s = vec![
                        Signal::HrtimChannel { timer: *timer, ch: HrtimCh::Ch1 },
                        Signal::HrtimChannel { timer: *timer, ch: HrtimCh::Ch2 },
                    ];
                    if let Some(z) = zcd_eev { s.push(Signal::HrtimEev(*z)); }
                    s
                }
                Assignment::PcmPhaseExternal { dac, peak_eev, zcd_eev, timer, .. } => {
                    let mut s = vec![
                        Signal::HrtimChannel { timer: *timer, ch: HrtimCh::Ch1 },
                        Signal::HrtimChannel { timer: *timer, ch: HrtimCh::Ch2 },
                        Signal::HrtimEev(*peak_eev),
                    ];
                    if let Some(z) = zcd_eev { s.push(Signal::HrtimEev(*z)); }
                    if let Some(d) = dac { s.push(Signal::DacOut(*d)); }
                    s
                }
                Assignment::ShortCircuitFault(f) => vec![Signal::HrtimFlt(f.flt)],
                Assignment::ShareBusDrive(d) => vec![Signal::DacOut(d.dac)],
                Assignment::Opamp { instance, external_vinp, external_vinm, external_vout } => {
                    let mut s = Vec::new();
                    if *external_vinp { s.push(Signal::OpampVinp(*instance)); }
                    if *external_vinm { s.push(Signal::OpampVinm(*instance)); }
                    if *external_vout { s.push(Signal::OpampVout(*instance)); }
                    s
                }
                Assignment::Spi { instance, needs_miso, needs_nss } => {
                    let mut s = vec![Signal::SpiMosi(*instance), Signal::SpiSck(*instance)];
                    if *needs_miso { s.push(Signal::SpiMiso(*instance)); }
                    if *needs_nss { s.push(Signal::SpiNss(*instance)); }
                    s
                }
                Assignment::I2c { instance, needs_smba } => {
                    let mut s = vec![Signal::I2cSda(*instance), Signal::I2cScl(*instance)];
                    if *needs_smba { s.push(Signal::I2cSmba(*instance)); }
                    s
                }
                Assignment::Usart { instance, flow_control, synchronous } => {
                    let mut s = vec![Signal::UsartTx(*instance), Signal::UsartRx(*instance)];
                    if *flow_control {
                        s.push(Signal::UsartCts(*instance));
                        s.push(Signal::UsartRts(*instance));
                    }
                    if *synchronous { s.push(Signal::UsartCk(*instance)); }
                    s
                }
                Assignment::Uart { instance, flow_control } => {
                    let mut s = vec![Signal::UartTx(*instance), Signal::UartRx(*instance)];
                    if *flow_control {
                        s.push(Signal::UartCts(*instance));
                        s.push(Signal::UartRts(*instance));
                    }
                    s
                }
                Assignment::Lpuart { instance, flow_control } => {
                    let mut s = vec![Signal::LpuartTx(*instance), Signal::LpuartRx(*instance)];
                    if *flow_control {
                        s.push(Signal::LpuartCts(*instance));
                        s.push(Signal::LpuartRts(*instance));
                    }
                    s
                }
                Assignment::Can { instance } => {
                    vec![Signal::CanTx(*instance), Signal::CanRx(*instance)]
                }
                Assignment::Usb => vec![Signal::UsbDp, Signal::UsbDm],
                Assignment::Ucpd { instance } => {
                    vec![Signal::UcpdCc1(*instance), Signal::UcpdCc2(*instance)]
                }
                Assignment::Tim { instance, channels_mask, complementary, bkin, etr } => {
                    let mut s = Vec::new();
                    let chs = [
                        crate::g474::TimCh::Ch1, crate::g474::TimCh::Ch2,
                        crate::g474::TimCh::Ch3, crate::g474::TimCh::Ch4,
                    ];
                    for (i, ch) in chs.iter().enumerate() {
                        if channels_mask & (1 << i as u8) != 0 {
                            s.push(Signal::TimCh(*instance, *ch));
                            if *complementary { s.push(Signal::TimChN(*instance, *ch)); }
                        }
                    }
                    if *bkin { s.push(Signal::TimBkin(*instance)); }
                    if *etr { s.push(Signal::TimEtr(*instance)); }
                    s
                }
                _ => Vec::new(),
            };
            for sig in signals {
                let pin = if let Some(p) = self.pin_assignments.get(&sig) {
                    Some(*p)
                } else {
                    let pins = pins_for(sig, variant);
                    if pins.len() == 1 { Some(pins[0]) } else { None }
                };
                if let Some(p) = pin {
                    out.push(Resource::Pin(p));
                }
            }
        }
        out
    }

    pub fn candidates_for(&self, idx: usize) -> Vec<Assignment> {
        let used = self.used_excluding(idx);
        let spec_refs: Vec<(u32, RequirementSpec)> = self
            .ids.iter().copied()
            .zip(self.requirements.iter().copied())
            .collect();
        self.requirements[idx].enumerate(&used, &spec_refs, self.variant)
    }

    pub fn sequencer_ids(&self) -> Vec<(u32, usize)> {
        self.ids
            .iter().copied()
            .zip(self.requirements.iter())
            .enumerate()
            .filter_map(|(i, (id, spec))| {
                if matches!(spec, RequirementSpec::AdcSequencer { .. }) {
                    Some((id, i))
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn phases(&self) -> Vec<PhaseAllocation> {
        self.assignments.iter().flatten().filter_map(|a| match a {
            Assignment::PcmPhase { alloc, .. } => Some(*alloc),
            _ => None,
        }).collect()
    }

    pub fn external_eevs(&self) -> Vec<CrossbarSource> {
        let mut out = Vec::new();
        for a in self.assignments.iter().flatten() {
            match a {
                Assignment::PcmPhaseExternal { peak_eev, zcd_eev, .. } => {
                    out.push(*peak_eev);
                    if let Some(z) = zcd_eev { out.push(*z); }
                }
                Assignment::PcmPhase { zcd_eev: Some(z), .. } => {
                    out.push(*z);
                }
                _ => {}
            }
        }
        out
    }

    pub fn phase_edges(&self) -> Vec<PhaseEdge> {
        let mut out = Vec::new();
        for a in self.assignments.iter().flatten() {
            match a {
                Assignment::PcmPhase { alloc, zcd_eev, timer, .. } => {
                    out.push(PhaseEdge { eev: alloc.eev, timer: *timer, slot: TimerInputSlot::Rst1 });
                    if let Some(z) = zcd_eev {
                        out.push(PhaseEdge { eev: *z, timer: *timer, slot: TimerInputSlot::Rst2 });
                    }
                }
                Assignment::PcmPhaseExternal { peak_eev, zcd_eev, timer, .. } => {
                    out.push(PhaseEdge { eev: *peak_eev, timer: *timer, slot: TimerInputSlot::Rst1 });
                    if let Some(z) = zcd_eev {
                        out.push(PhaseEdge { eev: *z, timer: *timer, slot: TimerInputSlot::Rst2 });
                    }
                }
                _ => {}
            }
        }
        out
    }

    pub fn phase_dac_timers(&self) -> Vec<(DacId, HrtimId)> {
        self.assignments.iter().flatten().filter_map(|a| match a {
            Assignment::PcmPhase { alloc, timer, .. } => Some((alloc.dac, *timer)),
            _ => None,
        }).collect()
    }

    pub fn phase_timers_and_dem(&self) -> Vec<(HrtimId, bool)> {
        self.assignments.iter().flatten().filter_map(|a| match a {
            Assignment::PcmPhase { timer, dem, .. }
            | Assignment::PcmPhaseExternal { timer, dem, .. } => Some((*timer, *dem)),
            _ => None,
        }).collect()
    }

    pub fn first_fault(&self) -> Option<ShortCircuitFault> {
        self.assignments.iter().flatten().find_map(|a| match a {
            Assignment::ShortCircuitFault(f) => Some(*f), _ => None,
        })
    }

    pub fn first_drive_dac(&self) -> Option<DacId> {
        self.assignments.iter().flatten().find_map(|a| match a {
            Assignment::ShareBusDrive(d) => Some(d.dac), _ => None,
        })
    }

    /// First sequencer whose trigger event routes through the master timer.
    pub fn master_triggered_sequencer(&self) -> Option<Assignment> {
        self.assignments.iter().flatten().find_map(|a| match a {
            Assignment::AdcSequencer { trigger: TriggerSource::Event(ev), .. }
                if matches!(
                    ev,
                    CrossbarSource::Mcr1 | CrossbarSource::Mcr2 |
                    CrossbarSource::Mcr3 | CrossbarSource::Mcr4 | CrossbarSource::Mper
                ) => Some(a.clone()),
            _ => None,
        })
    }

    pub fn used_signals(&self) -> Vec<crate::pinout::Signal> {
        use crate::pinout::{HrtimCh, Signal};
        let mut out = Vec::new();
        for a in self.assignments.iter().flatten() {
            match a {
                Assignment::PcmPhase { timer, zcd_eev, .. } => {
                    out.push(Signal::HrtimChannel { timer: *timer, ch: HrtimCh::Ch1 });
                    out.push(Signal::HrtimChannel { timer: *timer, ch: HrtimCh::Ch2 });
                    if let Some(z) = zcd_eev { out.push(Signal::HrtimEev(*z)); }
                }
                Assignment::PcmPhaseExternal { dac, peak_eev, zcd_eev, timer, .. } => {
                    out.push(Signal::HrtimChannel { timer: *timer, ch: HrtimCh::Ch1 });
                    out.push(Signal::HrtimChannel { timer: *timer, ch: HrtimCh::Ch2 });
                    out.push(Signal::HrtimEev(*peak_eev));
                    if let Some(z) = zcd_eev { out.push(Signal::HrtimEev(*z)); }
                    if let Some(d) = dac { out.push(Signal::DacOut(*d)); }
                }
                Assignment::ShortCircuitFault(f) => { out.push(Signal::HrtimFlt(f.flt)); }
                Assignment::ShareBusDrive(d) => { out.push(Signal::DacOut(d.dac)); }
                Assignment::AdcConversion { adc, channel, .. } => {
                    out.push(Signal::AdcIn { adc: *adc, channel: *channel });
                }
                Assignment::AdcSequencer { .. } => {}
                Assignment::Opamp { instance, external_vinp, external_vinm, external_vout } => {
                    if *external_vinp { out.push(Signal::OpampVinp(*instance)); }
                    if *external_vinm { out.push(Signal::OpampVinm(*instance)); }
                    if *external_vout { out.push(Signal::OpampVout(*instance)); }
                }
                Assignment::Spi { instance, needs_miso, needs_nss } => {
                    out.push(Signal::SpiMosi(*instance));
                    out.push(Signal::SpiSck(*instance));
                    if *needs_miso { out.push(Signal::SpiMiso(*instance)); }
                    if *needs_nss  { out.push(Signal::SpiNss(*instance)); }
                }
                Assignment::I2c { instance, needs_smba } => {
                    out.push(Signal::I2cSda(*instance));
                    out.push(Signal::I2cScl(*instance));
                    if *needs_smba { out.push(Signal::I2cSmba(*instance)); }
                }
                Assignment::Usart { instance, flow_control, synchronous } => {
                    out.push(Signal::UsartTx(*instance));
                    out.push(Signal::UsartRx(*instance));
                    if *flow_control {
                        out.push(Signal::UsartCts(*instance));
                        out.push(Signal::UsartRts(*instance));
                    }
                    if *synchronous { out.push(Signal::UsartCk(*instance)); }
                }
                Assignment::Uart { instance, flow_control } => {
                    out.push(Signal::UartTx(*instance));
                    out.push(Signal::UartRx(*instance));
                    if *flow_control {
                        out.push(Signal::UartCts(*instance));
                        out.push(Signal::UartRts(*instance));
                    }
                }
                Assignment::Lpuart { instance, flow_control } => {
                    out.push(Signal::LpuartTx(*instance));
                    out.push(Signal::LpuartRx(*instance));
                    if *flow_control {
                        out.push(Signal::LpuartCts(*instance));
                        out.push(Signal::LpuartRts(*instance));
                    }
                }
                Assignment::Can { instance } => {
                    out.push(Signal::CanTx(*instance));
                    out.push(Signal::CanRx(*instance));
                }
                Assignment::Usb => {
                    out.push(Signal::UsbDp);
                    out.push(Signal::UsbDm);
                }
                Assignment::Ucpd { instance } => {
                    out.push(Signal::UcpdCc1(*instance));
                    out.push(Signal::UcpdCc2(*instance));
                }
                Assignment::Tim { instance, channels_mask, complementary, bkin, etr } => {
                    let chs = [
                        crate::g474::TimCh::Ch1, crate::g474::TimCh::Ch2,
                        crate::g474::TimCh::Ch3, crate::g474::TimCh::Ch4,
                    ];
                    for (i, ch) in chs.iter().enumerate() {
                        if channels_mask & (1 << i as u8) != 0 {
                            out.push(Signal::TimCh(*instance, *ch));
                            if *complementary { out.push(Signal::TimChN(*instance, *ch)); }
                        }
                    }
                    if *bkin { out.push(Signal::TimBkin(*instance)); }
                    if *etr  { out.push(Signal::TimEtr(*instance)); }
                }
            }
        }
        out
    }

    pub fn warnings(&self) -> Vec<CapabilityCheck> {
        let mut out = Vec::new();
        let (pcm_int, pcm_hybrid, pcm_ext) = self.pcm_architecture_breakdown();
        let pcm_total = pcm_int + pcm_hybrid + pcm_ext;
        let pcm_reqs = self.requirements.iter()
            .filter(|r| matches!(r, RequirementSpec::PcmPhase { .. }))
            .count();
        if pcm_reqs > 0 && pcm_total < pcm_reqs {
            out.push(warn(format!(
                "{} PCM phase requirement(s) have no valid assignment",
                pcm_reqs - pcm_total
            )));
        }
        for (i, a) in self.assignments.iter().enumerate() {
            if let Some(Assignment::PcmPhaseExternal { dem, .. }) = a {
                out.push(warn(format!(
                    "Phase #{}: external peak comp has no hardware slope comp. Verify D<0.5 or add external RC.{}",
                    i + 1,
                    if *dem { " (DEM OK.)" } else { "" }
                )));
            }
        }
        let archs_present = [pcm_int > 0, pcm_hybrid > 0, pcm_ext > 0].iter().filter(|b| **b).count();
        if archs_present > 1 {
            out.push(info(format!(
                "Mixed PCM architectures: {} internal, {} hybrid, {} external.",
                pcm_int, pcm_hybrid, pcm_ext
            )));
        }
        if self.first_fault().is_none() {
            out.push(warn_with(
                "No hardware short-circuit fault configured.",
                Fix::AddRequirement(RequirementSpec::ShortCircuitFault),
            ));
        } else {
            out.push(ok("Short-circuit fault: HW-latched via COMP -> HRTIM FLT"));
        }
        let sequencer_count = self.requirements.iter()
            .filter(|r| matches!(r, RequirementSpec::AdcSequencer { .. }))
            .count();
        let has_master = self.master_triggered_sequencer().is_some();
        if pcm_total > 0 && sequencer_count == 0 {
            out.push(warn_with(
                "No ADC sequencer configured -- voltage loop / monitors need ADC.",
                Fix::AddRequirement(RequirementSpec::AdcSequencer {
                    adc: AdcInstance::Adc1,
                    kind: SequencerKind::DualRegular,
                    trigger: TriggerSource::Event(CrossbarSource::Mcr1),
                }),
            ));
        } else if sequencer_count > 0 && !has_master {
            out.push(info("No sequencer is triggered from the master timer."));
        }
        // Diagnose unassigned ADC conversions: stale group ref vs all pins
        // claimed vs no free input channel.
        for (i, asn) in self.assignments.iter().enumerate() {
            if asn.is_some() { continue; }
            if let RequirementSpec::AdcConversion { group, purpose, .. } = self.requirements[i] {
                let parent = self.ids.iter().zip(&self.requirements).find_map(|(id, spec)| {
                    if *id != group { return None; }
                    match spec {
                        RequirementSpec::AdcSequencer { adc, kind, .. } => Some((*adc, *kind)),
                        _ => None,
                    }
                });
                let Some((parent_adc, parent_kind)) = parent else {
                    out.push(warn(format!(
                        "Conversion #{} ({}): parent sequencer (id={}) not found. \
                         Reattach via the group dropdown.",
                        i + 1, purpose.short_name(), group
                    )));
                    continue;
                };
                // Walk the candidate input space. Categorize blocked
                // candidates: pin claimed elsewhere vs ADC input claimed.
                let used = self.used_excluding(i);
                let adcs: Vec<AdcInstance> = if parent_kind.is_dual() {
                    match parent_adc {
                        AdcInstance::Adc1 => vec![AdcInstance::Adc1, AdcInstance::Adc2],
                        AdcInstance::Adc3 => vec![AdcInstance::Adc3, AdcInstance::Adc4],
                        other => vec![other],
                    }
                } else {
                    vec![parent_adc]
                };
                let mut pin_blocked: Vec<String> = Vec::new();
                let mut input_taken: Vec<String> = Vec::new();
                let mut total_inputs = 0usize;
                for adc in adcs {
                    for ch in 1..=18u8 {
                        let pins = crate::pinout::pins_for(
                            crate::pinout::Signal::AdcIn { adc, channel: ch },
                            self.variant,
                        );
                        let Some(pin) = pins.first() else { continue; };
                        total_inputs += 1;
                        if used.contains(&Resource::AdcInput(adc, ch)) {
                            input_taken.push(format!(
                                "ADC{}_IN{} ({})",
                                adc.number(),
                                ch,
                                pin.name()
                            ));
                        } else if used.contains(&Resource::Pin(*pin)) {
                            pin_blocked.push(format!(
                                "ADC{}_IN{} ({}: claimed by HRTIM/DAC/locked)",
                                adc.number(),
                                ch,
                                pin.name()
                            ));
                        }
                    }
                }
                let detail = match (pin_blocked.is_empty(), input_taken.is_empty()) {
                    (true, true) if total_inputs == 0 => "no input pin available on this chip variant".to_string(),
                    (false, true) => format!(
                        "all {}/{} candidate pins are claimed by other fabric signals (e.g. {})",
                        pin_blocked.len(),
                        total_inputs,
                        pin_blocked.first().cloned().unwrap_or_default()
                    ),
                    (true, false) => format!(
                        "all {}/{} input channels are taken by other conversions",
                        input_taken.len(),
                        total_inputs
                    ),
                    (false, false) => format!(
                        "{} pins blocked by other signals, {} channels taken by other conversions",
                        pin_blocked.len(),
                        input_taken.len()
                    ),
                    _ => "no candidate available".to_string(),
                };
                out.push(warn(format!(
                    "Conversion #{} ({}): {}",
                    i + 1,
                    purpose.short_name(),
                    detail
                )));
            }
        }

        if pcm_total > 0 && out.iter().all(|c| c.severity != Severity::Warn) {
            out.push(ok("All PCM phases assigned, no resource pressure."));
        }
        out
    }

    pub fn export_summary(&self) -> String {
        use std::fmt::Write as _;
        let mut s = String::new();
        let _ = writeln!(s, "# STM32G474 peripheral-planner design summary");
        let _ = writeln!(s);
        let _ = writeln!(
            s,
            "## Requirements ({}): {} assigned",
            self.requirements.len(),
            self.assignments.iter().flatten().count()
        );
        let _ = writeln!(s);
        for (i, ((id, spec), asn)) in self.ids.iter().zip(&self.requirements).zip(&self.assignments).enumerate() {
            let body = asn.as_ref().map(|a| a.label()).unwrap_or_else(|| "(no valid assignment)".to_string());
            let _ = writeln!(s, "  #{:<2} id={:<3} [{}] {}", i + 1, id, spec.name(), body);
        }
        let _ = writeln!(s);
        let (pcm_int, pcm_hybrid, pcm_ext) = self.pcm_architecture_breakdown();
        let _ = writeln!(s, "## PCM architecture breakdown");
        let _ = writeln!(s, "  internal:           {}", pcm_int);
        let _ = writeln!(s, "  hybrid:             {}", pcm_hybrid);
        let _ = writeln!(s, "  external:           {}", pcm_ext);
        let _ = writeln!(s);
        s
    }

    fn pcm_architecture_breakdown(&self) -> (usize, usize, usize) {
        let mut internal = 0;
        let mut hybrid = 0;
        let mut external = 0;
        for a in self.assignments.iter().flatten() {
            match a {
                Assignment::PcmPhase { zcd_eev: None, .. } => internal += 1,
                Assignment::PcmPhase { zcd_eev: Some(_), .. } => hybrid += 1,
                Assignment::PcmPhaseExternal { .. } => external += 1,
                _ => {}
            }
        }
        (internal, hybrid, external)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pinout::Signal;

    #[test]
    fn opamp_allocates_and_claims_signals() {
        let mut d = Design {
            format_version: DESIGN_FORMAT_VERSION,
            ids: Vec::new(),
            requirements: Vec::new(),
            assignments: Vec::new(),
            locks: ResourceBag::new(),
            pin_assignments: std::collections::HashMap::new(),
            next_id: 1,
            variant: default_variant(),
        };
        d.add(RequirementSpec::UseOpamp {
            instance: OpampId::Opamp1,
            external_vinp: true,
            external_vinm: true,
            external_vout: true,
        });
        d.normalize();
        let asn = d.assignments[0].clone().expect("opamp should resolve");
        assert!(matches!(
            asn,
            Assignment::Opamp { instance: OpampId::Opamp1, .. }
        ));
        assert!(asn.consumed().contains(&Resource::Opamp(OpampId::Opamp1)));
        let sigs = d.used_signals();
        assert!(sigs.contains(&Signal::OpampVinp(OpampId::Opamp1)));
        assert!(sigs.contains(&Signal::OpampVinm(OpampId::Opamp1)));
        assert!(sigs.contains(&Signal::OpampVout(OpampId::Opamp1)));
    }

    #[test]
    fn opamp_instance_is_exclusive() {
        let mut d = Design {
            format_version: DESIGN_FORMAT_VERSION,
            ids: Vec::new(),
            requirements: Vec::new(),
            assignments: Vec::new(),
            locks: ResourceBag::new(),
            pin_assignments: std::collections::HashMap::new(),
            next_id: 1,
            variant: default_variant(),
        };
        d.add(RequirementSpec::UseOpamp {
            instance: OpampId::Opamp1,
            external_vinp: true, external_vinm: true, external_vout: true,
        });
        d.add(RequirementSpec::UseOpamp {
            instance: OpampId::Opamp1,
            external_vinp: true, external_vinm: true, external_vout: true,
        });
        d.normalize();
        // Second request for the same instance has no candidate.
        assert!(d.assignments[0].is_some());
        assert!(d.assignments[1].is_none());
    }
}
