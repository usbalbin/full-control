//! Brute-force allocator for peak-current-mode phase requirements on STM32G474.
//!
//! Each phase needs three resources, all unique across the design:
//!   - a fast DAC channel (DAC3/DAC4) for the cycle-by-cycle threshold,
//!   - a COMP reachable from that DAC,
//!   - an EEV reachable from that COMP (routed into the phase's HRTIM timer).
//!
//! HRTIM timer choice is omitted from the search: every TIMA..TIMF can react
//! to every EEV, so it adds no constraint here. ADC sampling is intentionally
//! *not* modelled per phase — the real requirement is one shared HRTIM
//! trigger driving ADC1+ADC2 dual mode, which is a separate global plan.

use crate::g474::*;

#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct PhaseAllocation {
    pub dac: DacId,
    pub comp: CompId,
    pub eev: CrossbarSource,
}

const FAST_DACS: &[DacId] = &[
    DacId::Dac3Ch1,
    DacId::Dac3Ch2,
    DacId::Dac4Ch1,
    DacId::Dac4Ch2,
];

pub fn enumerate_pcm_phases(n: usize) -> Vec<Vec<PhaseAllocation>> {
    let mut results = Vec::new();
    let mut current = Vec::with_capacity(n);
    backtrack(n, &mut current, &mut results);
    results
}

fn backtrack(
    n: usize,
    current: &mut Vec<PhaseAllocation>,
    results: &mut Vec<Vec<PhaseAllocation>>,
) {
    if current.len() == n {
        results.push(current.clone());
        return;
    }
    for &dac in FAST_DACS {
        if current.iter().any(|p| p.dac == dac) {
            continue;
        }
        for &comp in comps_for_dac(dac) {
            if current.iter().any(|p| p.comp == comp) {
                continue;
            }
            for &eev in eevs_for_comp(comp) {
                if current.iter().any(|p| p.eev == eev) {
                    continue;
                }
                current.push(PhaseAllocation { dac, comp, eev });
                backtrack(n, current, results);
                current.pop();
            }
        }
    }
}

// ---------- Design intent + HRTIM CR accounting ----------

#[derive(Copy, Clone, Debug, Default)]
pub struct Intent {
    /// Diode-emulation mode via DMA-driven COMP INM swap. Claims CR2+CR3
    /// on each phase's timer (HS-start and LS-start DMA triggers).
    pub dem_enabled: bool,
}

/// Per-phase claim on a timer's user-configurable compare slots.
#[derive(Clone, Debug)]
pub struct TimerSlotUsage {
    pub timer: HrtimId,
    pub claims: Vec<(TimerCompareSlot, &'static str)>,
}

impl TimerSlotUsage {
    pub fn is_free(&self, slot: TimerCompareSlot) -> bool {
        !self.claims.iter().any(|(s, _)| *s == slot)
    }

    pub fn free_slots(&self) -> Vec<TimerCompareSlot> {
        ALL_CR_SLOTS
            .iter()
            .copied()
            .filter(|s| self.is_free(*s))
            .collect()
    }
}

/// The N phases are assigned to TIMA..TIMA+N-1 by phase index. This is a
/// convention, not a constraint — any 4-of-6 ordering would work equivalently
/// given the uniform EEV routing.
pub fn phase_timers(n: usize) -> Vec<HrtimId> {
    const ORDER: &[HrtimId] = &[
        HrtimId::TimA,
        HrtimId::TimB,
        HrtimId::TimC,
        HrtimId::TimD,
        HrtimId::TimE,
        HrtimId::TimF,
    ];
    ORDER.iter().copied().take(n).collect()
}

pub fn compute_cr_usage(
    phase_timers: &[HrtimId],
    intent: &Intent,
) -> Vec<TimerSlotUsage> {
    phase_timers
        .iter()
        .map(|&t| {
            // RM0440 §28.3.21: DAC sawtooth (slope comp, DCDE=1 DCDS=0)
            //   reset_trg on counter roll-over (automatic, free)
            //   step_trg  on CMP2 — CR2 exclusively consumed in this mode.
            //
            // RM0440 §28.3.7 note on "Concurrent set requests": auto-delayed
            // mode is only available on CMP2 and CMP4. DEM requires
            // auto-delayed deadtime (regular deadtime module can't handle
            // variable-delay ZVS turn-on). CMP2 is taken by the sawtooth,
            // so auto-delayed deadtime claims CMP4.
            //
            // DMA-triggered INM swap (if used): can fire on timer reset and
            // output-1-reset edge, neither of which consumes a CR slot.
            let mut claims = vec![(
                TimerCompareSlot::Cr2,
                "DAC sawtooth step (CMP2, slope comp)",
            )];
            if intent.dem_enabled {
                claims.push((
                    TimerCompareSlot::Cr4,
                    "DEM auto-delayed deadtime (CMP4 forced: CMP2 taken)",
                ));
            }
            TimerSlotUsage { timer: t, claims }
        })
        .collect()
}

// ---------- ADC channel accounting ----------
//
// The 10 (regular, injected) channels of ADC1..5 are a flat resource pool.
// The sampling plan pair consumes both halves of its pair's regular slot;
// monitor inputs (V_OUT, V_IN, NTC, share-bus sense) consume one each.

pub const ALL_CR_SLOTS_HELPER: &[TimerCompareSlot] = ALL_CR_SLOTS;

pub const ALL_ADC_CHANNELS: &[AdcChannel] = &[
    AdcChannel::Adc1Regular,
    AdcChannel::Adc1Injected,
    AdcChannel::Adc2Regular,
    AdcChannel::Adc2Injected,
    AdcChannel::Adc3Regular,
    AdcChannel::Adc3Injected,
    AdcChannel::Adc4Regular,
    AdcChannel::Adc4Injected,
    AdcChannel::Adc5Regular,
    AdcChannel::Adc5Injected,
];

pub fn channels_consumed_by_plan(plan: &AdcSamplingPlan) -> [AdcChannel; 2] {
    match plan.adc_pair {
        AdcPair::Adc12 => [AdcChannel::Adc1Regular, AdcChannel::Adc2Regular],
        AdcPair::Adc34 => [AdcChannel::Adc3Regular, AdcChannel::Adc4Regular],
    }
}

pub fn free_adc_channels(consumed: &[AdcChannel]) -> Vec<AdcChannel> {
    ALL_ADC_CHANNELS
        .iter()
        .copied()
        .filter(|c| !consumed.contains(c))
        .collect()
}

// ---------- Share-bus drive ----------
//
// Drive an analog voltage proportional to module current contribution onto
// the analog backplane share bus. DAC1 and DAC2 channels have external pin
// outputs directly (PA4/PA5/PA6). An OPAMP follower would add drive
// strength but isn't a hard constraint — modelled later.

const PIN_CAPABLE_SLOW_DACS: &[DacId] = &[
    DacId::Dac1Ch1,
    DacId::Dac1Ch2,
    DacId::Dac2Ch1,
];

#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct ShareBusDrive {
    pub dac: DacId,
}

pub fn enumerate_share_bus_drives() -> Vec<ShareBusDrive> {
    PIN_CAPABLE_SLOW_DACS
        .iter()
        .copied()
        .map(|dac| ShareBusDrive { dac })
        .collect()
}

pub fn share_bus_drives_compatible_with(
    fault: Option<&ShortCircuitFault>,
    drives: &[ShareBusDrive],
) -> Vec<ShareBusDrive> {
    drives
        .iter()
        .copied()
        .filter(|d| fault.is_none_or(|f| f.dac != d.dac))
        .collect()
}

// ---------- Short-circuit fault ----------
//
// Hardware-only OCP fault path: one COMP compares the ACS37030 sense signal
// against a fixed catastrophic threshold (driven by a slow DAC channel — fast
// DACs are reserved for cycle-by-cycle PCM thresholds), and the COMP routes
// to an HRTIM FLT input which forces all timer outputs to a safe state in
// hardware on assertion. Independent of the PCM control loop's threshold so
// a firmware bug or stalled control loop can't disable the protection.

const SLOW_DACS: &[DacId] = &[DacId::Dac1Ch1, DacId::Dac1Ch2, DacId::Dac2Ch1];

#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct ShortCircuitFault {
    pub dac: DacId,
    pub comp: CompId,
    pub flt: HrtimFltId,
}

pub fn enumerate_short_circuit_faults() -> Vec<ShortCircuitFault> {
    let mut faults = Vec::new();
    for &dac in SLOW_DACS {
        for &comp in comps_for_dac(dac) {
            for &flt in flts_for_comp(comp) {
                faults.push(ShortCircuitFault { dac, comp, flt });
            }
        }
    }
    faults
}

/// Faults that don't collide on COMP or DAC with any phase in `phases`.
pub fn faults_compatible_with(
    phases: &[PhaseAllocation],
    faults: &[ShortCircuitFault],
) -> Vec<ShortCircuitFault> {
    faults
        .iter()
        .copied()
        .filter(|f| {
            !phases.iter().any(|p| p.comp == f.comp || p.dac == f.dac)
        })
        .collect()
}

// ---------- ADC sampling plan ----------
//
// Separate global requirement: one HRTIM event drives one ADC trigger which
// fires an ADC pair in dual-regular-simultaneous mode. The pair's master ADC
// (ADC1 of the 1+2 pair, ADC3 of the 3+4 pair) must be reachable from the
// chosen trigger via its regular sequencer.

#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum AdcPair {
    Adc12,
    Adc34,
}

impl AdcPair {
    fn master_regular(self) -> AdcChannel {
        match self {
            AdcPair::Adc12 => AdcChannel::Adc1Regular,
            AdcPair::Adc34 => AdcChannel::Adc3Regular,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct AdcSamplingPlan {
    pub trigger_event: CrossbarSource,
    pub trigger: AdcTriggerId,
    pub adc_pair: AdcPair,
}

pub fn enumerate_adc_sampling_plans() -> Vec<AdcSamplingPlan> {
    let mut plans = Vec::new();
    for &(event, triggers) in CROSSBAR_TO_ADC_TRIGGER {
        for &trigger in triggers {
            let channels = adc_channels_for(trigger);
            for &pair in &[AdcPair::Adc12, AdcPair::Adc34] {
                if channels.contains(&pair.master_regular()) {
                    plans.push(AdcSamplingPlan {
                        trigger_event: event,
                        trigger,
                        adc_pair: pair,
                    });
                }
            }
        }
    }
    plans
}

/// Plans whose trigger event comes from the HRTIM master timer — preferred
/// for global sampling because they're not tied to any single phase's timer.
pub fn is_master_timer_event(event: CrossbarSource) -> bool {
    matches!(
        event,
        CrossbarSource::Mcr1
            | CrossbarSource::Mcr2
            | CrossbarSource::Mcr3
            | CrossbarSource::Mcr4
            | CrossbarSource::Mper
    )
}

pub fn unique_resource_sets(
    solutions: &[Vec<PhaseAllocation>],
) -> Vec<Vec<PhaseAllocation>> {
    let mut seen: Vec<Vec<PhaseAllocation>> = Vec::new();
    for sol in solutions {
        let mut canon = sol.clone();
        canon.sort_by_key(|p| format!("{:?}", p.dac));
        if !seen.iter().any(|s| s == &canon) {
            seen.push(canon);
        }
    }
    seen
}
