//! Pin-swap logic for the package view.
//!
//! A "picked role" is a movable thing the user has grabbed off a pin:
//! either an ADC conversion (swappable across channels on the same ADC)
//! or a signal with multiple AF pin options (swappable via pin-lock).
//! For each destination pin we compute a `MoveResult` that captures the
//! impact of placing the picked role there — direct swap, semantic change
//! (e.g. fast→slow ADC channel), a one-step cascade (displaces an
//! occupant that has a free alternative), or blocked.

use std::collections::{HashMap, HashSet};

use eframe::egui::{Color32, Stroke};

use crate::g474::{is_fast_adc_channel, AdcInstance, HrtimId, TimerCaptureUnit, TimerCompareSlot};
use crate::package_view::{
    self, PinPaint, C_PIN, C_PIN_ASSIGNED, C_PIN_BLOCKED, C_PIN_CASCADE1, C_PIN_CASCADE_MULTI,
    C_PIN_DIMMED, C_PIN_DIRECT, C_PIN_HELD_SOURCE, C_PIN_SEMANTIC,
};
use crate::pinout::{self, ChipVariant, HrtimCh, Pin, Signal};
use crate::requirements::{Assignment, Design, RequirementSpec, Resource, SpeedPref};

/// What the user currently has grabbed off a pin.
#[derive(Clone, Copy, Debug)]
pub enum PickedRole {
    /// An ADC conversion requirement. Moveable to any pin on the same ADC
    /// instance; fast/slow may change.
    AdcConversion {
        req_idx: usize,
        current_pin: Pin,
        adc: AdcInstance,
        current_channel: u8,
    },
    /// A signal with more than one AF pin option (e.g. HRTIM_FLT5 on PB0 or
    /// PC7). Moving is a pin-lock swap.
    AltPinSignal {
        signal: Signal,
        current_pin: Pin,
    },
    /// A PCM phase's HRTIM channel pin. Picking this up lets the user
    /// retarget the whole phase to a different timer — the paired channel
    /// (CH1↔CH2 on the same timer) moves automatically.
    PcmPhaseChannel {
        req_idx: usize,
        current_pin: Pin,
        current_timer: HrtimId,
        current_ch: HrtimCh,
    },
}

impl PickedRole {
    pub fn current_pin(&self) -> Pin {
        match self {
            Self::AdcConversion { current_pin, .. }
            | Self::AltPinSignal { current_pin, .. }
            | Self::PcmPhaseChannel { current_pin, .. } => *current_pin,
        }
    }

    pub fn description(&self, design: &Design) -> String {
        match self {
            Self::AdcConversion { adc, current_channel, req_idx, .. } => {
                let purpose = match design.requirements.get(*req_idx) {
                    Some(RequirementSpec::AdcConversion { purpose, .. }) => purpose.short_name(),
                    _ => "conversion".to_string(),
                };
                let speed = if is_fast_adc_channel(*current_channel) { "fast" } else { "slow" };
                format!(
                    "{} on ADC{}_IN{} [{}]",
                    purpose, adc.number(), current_channel, speed
                )
            }
            Self::AltPinSignal { signal, .. } => signal.name(),
            Self::PcmPhaseChannel { current_timer, current_ch, req_idx, .. } => {
                let ch = match current_ch { HrtimCh::Ch1 => "CH1", HrtimCh::Ch2 => "CH2" };
                format!("PCM phase #{} {} on {:?}", req_idx + 1, ch, current_timer)
            }
        }
    }
}

/// Identify the picked role from the signal currently on `pin`, given the
/// design. Returns None if the pin has no signal, or the signal can't be
/// swapped (single-option pins like DAC outputs, fixed GPIO).
pub fn role_for_pin(
    pin: Pin,
    signal: Signal,
    variant: ChipVariant,
    design: &Design,
) -> Option<PickedRole> {
    match signal {
        Signal::AdcIn { adc, channel } => {
            // Find the AdcConversion assignment matching this (adc, channel).
            for (idx, asn) in design.assignments.iter().enumerate() {
                if let Some(Assignment::AdcConversion { adc: a, channel: c, .. }) = asn {
                    if *a == adc && *c == channel {
                        return Some(PickedRole::AdcConversion {
                            req_idx: idx,
                            current_pin: pin,
                            adc,
                            current_channel: channel,
                        });
                    }
                }
            }
            None
        }
        Signal::HrtimChannel { timer, ch } => {
            // Find the PcmPhase assignment using this timer. Retargeting it
            // to a different timer moves both CH1 and CH2.
            for (idx, asn) in design.assignments.iter().enumerate() {
                if let Some(Assignment::PcmPhase { timer: t, .. })
                    | Some(Assignment::PcmPhaseExternal { timer: t, .. }) = asn
                {
                    if *t == timer {
                        return Some(PickedRole::PcmPhaseChannel {
                            req_idx: idx,
                            current_pin: pin,
                            current_timer: timer,
                            current_ch: ch,
                        });
                    }
                }
            }
            None
        }
        _ => {
            // AltPinSignal only makes sense if there's more than one AF pin.
            let alts = pinout::pins_for(signal, variant);
            if alts.len() > 1 {
                Some(PickedRole::AltPinSignal { signal, current_pin: pin })
            } else {
                None
            }
        }
    }
}

/// Impact of placing the picked role on a given destination pin.
#[derive(Clone, Debug)]
pub enum MoveResult {
    /// Same-semantics drop-in. Paint green.
    Direct { note: String },
    /// Works, but changes a meaningful property (fast↔slow channel). Paint teal.
    Semantic { note: String },
    /// Displaces an occupant; the occupant has a free alternative. Paint yellow.
    Cascade1 { chain: Vec<(Signal, Pin)>, note: String },
    /// Multi-step cascade (>1 displacement). Paint orange.
    CascadeMulti { chain: Vec<(Signal, Pin)>, note: String },
    /// Pin is valid for this role type but no placement works. Paint red/dim.
    Blocked { reason: String },
    /// Pin isn't a candidate for this role at all (wrong ADC / no AF). Skip.
    NotApplicable,
}

/// For every GPIO pin on the variant, compute its `PinPaint` given the
/// current design state and optional picked role.
pub fn build_pin_paints(
    design: &Design,
    variant: ChipVariant,
    picked: Option<PickedRole>,
) -> HashMap<Pin, PinPaint> {
    let mut out = HashMap::new();
    // Pin -> signal map (explicit locks + singleton-AF forced pins).
    let pin_signals = current_pin_signals(design, variant);

    // Enumerate every GPIO that appears in the AF table for this variant.
    let all_pins: HashSet<Pin> = all_gpio_pins(variant);

    for pin in all_pins {
        let paint = if let Some(role) = picked {
            if pin == role.current_pin() {
                PinPaint {
                    fill: C_PIN_HELD_SOURCE,
                    border: Some(Stroke::new(1.5, Color32::WHITE)),
                    sublabel: Some(format!("{}  (source)", signal_sublabel(&pin_signals, pin))),
                    tooltip: Some(format!("Currently: {}\nClick here or outside to cancel", role.description(design))),
                    interactive: true,
                }
            } else {
                let r = compute_move(design, variant, role, pin, &pin_signals);
                paint_for_move(&r, &pin_signals, pin)
            }
        } else {
            // Idle state: green if assigned, neutral if free.
            if let Some(sig) = pin_signals.get(&pin) {
                PinPaint {
                    fill: C_PIN_ASSIGNED,
                    border: None,
                    sublabel: Some(sig.name()),
                    tooltip: Some(format!(
                        "{}\nClick to pick up (if swappable)",
                        sig.name()
                    )),
                    interactive: true,
                }
            } else {
                PinPaint {
                    fill: C_PIN,
                    border: None,
                    sublabel: None,
                    tooltip: None,
                    interactive: true,
                }
            }
        };
        out.insert(pin, paint);
    }
    out
}

fn signal_sublabel(map: &HashMap<Pin, Signal>, pin: Pin) -> String {
    map.get(&pin).map(|s| s.name()).unwrap_or_default()
}

fn paint_for_move(r: &MoveResult, pin_signals: &HashMap<Pin, Signal>, pin: Pin) -> PinPaint {
    let current = pin_signals.get(&pin).map(|s| s.name());
    match r {
        MoveResult::Direct { note } => PinPaint {
            fill: C_PIN_DIRECT,
            border: Some(Stroke::new(1.0, Color32::from_gray(230))),
            sublabel: current.clone(),
            tooltip: Some(format!("Drop here - direct\n{}", note)),
            interactive: true,
        },
        MoveResult::Semantic { note } => PinPaint {
            fill: C_PIN_SEMANTIC,
            border: Some(Stroke::new(1.0, Color32::from_gray(230))),
            sublabel: current.clone(),
            tooltip: Some(format!("Drop here - semantics change\n{}", note)),
            interactive: true,
        },
        MoveResult::Cascade1 { chain, note } => {
            let chain_str = chain.iter()
                .map(|(s, p)| format!("  {} -> {}", s.name(), p.name()))
                .collect::<Vec<_>>()
                .join("\n");
            PinPaint {
                fill: C_PIN_CASCADE1,
                border: Some(Stroke::new(1.0, Color32::from_gray(230))),
                sublabel: current.clone(),
                tooltip: Some(format!(
                    "Drop here - 1-step cascade:\n{}\n{}",
                    chain_str, note
                )),
                interactive: true,
            }
        }
        MoveResult::CascadeMulti { chain, note } => {
            let chain_str = chain.iter()
                .map(|(s, p)| format!("  {} -> {}", s.name(), p.name()))
                .collect::<Vec<_>>()
                .join("\n");
            PinPaint {
                fill: C_PIN_CASCADE_MULTI,
                border: Some(Stroke::new(1.0, Color32::from_gray(230))),
                sublabel: current.clone(),
                tooltip: Some(format!(
                    "Drop here - {}-step cascade:\n{}\n{}",
                    chain.len(), chain_str, note
                )),
                interactive: true,
            }
        }
        MoveResult::Blocked { reason } => PinPaint {
            fill: C_PIN_BLOCKED,
            border: None,
            sublabel: current.clone(),
            tooltip: Some(format!("Blocked\n{}", reason)),
            interactive: false,
        },
        MoveResult::NotApplicable => PinPaint {
            fill: C_PIN_DIMMED,
            border: None,
            sublabel: current.clone(),
            tooltip: None,
            interactive: false,
        },
    }
}

/// Snapshot of pin → signal for every signal that currently has a pin we
/// can resolve (explicit lock OR singleton-AF derivation OR ADC-conversion
/// channel lookup).
pub fn current_pin_signals(design: &Design, variant: ChipVariant) -> HashMap<Pin, Signal> {
    let mut out = HashMap::new();
    for s in design.used_signals() {
        let pin = match s {
            Signal::AdcIn { adc, channel } => {
                pinout::pins_for(Signal::AdcIn { adc, channel }, variant).first().copied()
            }
            _ => {
                if let Some(p) = design.pin_assignments.get(&s) {
                    Some(*p)
                } else {
                    let opts = pinout::pins_for(s, variant);
                    if opts.len() == 1 { Some(opts[0]) } else { None }
                }
            }
        };
        if let Some(p) = pin {
            out.insert(p, s);
        }
    }
    out
}

fn all_gpio_pins(variant: ChipVariant) -> HashSet<Pin> {
    use crate::pinout::signals_on;
    let mut out = HashSet::new();
    // Brute: try every port/num combination seen in practice on G474 LQFP64.
    for port in ['A', 'B', 'C', 'D', 'F', 'G'] {
        for num in 0..=15u8 {
            let p = Pin::new(port, num);
            if !signals_on(p, variant).is_empty() {
                out.insert(p);
            }
        }
    }
    out
}

fn compute_move(
    design: &Design,
    variant: ChipVariant,
    role: PickedRole,
    target: Pin,
    pin_signals: &HashMap<Pin, Signal>,
) -> MoveResult {
    match role {
        PickedRole::AdcConversion { req_idx, adc, current_channel, .. } => {
            compute_adc_move(design, variant, req_idx, adc, current_channel, target, pin_signals)
        }
        PickedRole::AltPinSignal { signal, current_pin } => {
            compute_alt_pin_move(design, variant, signal, current_pin, target, pin_signals)
        }
        PickedRole::PcmPhaseChannel { req_idx, current_timer, current_ch, .. } => {
            compute_phase_move(design, variant, req_idx, current_timer, current_ch, target)
        }
    }
}

fn compute_phase_move(
    design: &Design,
    variant: ChipVariant,
    req_idx: usize,
    current_timer: HrtimId,
    current_ch: HrtimCh,
    target: Pin,
) -> MoveResult {
    // Target must be an HRTIM channel pin matching our channel number.
    let target_timer = pinout::signals_on(target, variant)
        .into_iter()
        .find_map(|(_, s)| match s {
            Signal::HrtimChannel { timer, ch } if ch == current_ch => Some(timer),
            _ => None,
        });
    let Some(new_timer) = target_timer else {
        return MoveResult::NotApplicable;
    };
    if new_timer == current_timer {
        return MoveResult::NotApplicable;
    }
    // Is this phase DEM-capable? (We need to preserve CR4/CPT2 claims.)
    let dem = match design.assignments.get(req_idx) {
        Some(Some(Assignment::PcmPhase { dem, .. })) => *dem,
        Some(Some(Assignment::PcmPhaseExternal { dem, .. })) => *dem,
        _ => false,
    };
    // Resource pool excluding this phase's own claims.
    let used = design.used_excluding(req_idx);
    let timer_free = !used.contains(&Resource::Timer(new_timer))
        && !used.contains(&Resource::TimerSlot(new_timer, TimerCompareSlot::Cr2))
        && (!dem
            || (!used.contains(&Resource::TimerSlot(new_timer, TimerCompareSlot::Cr4))
                && !used.contains(&Resource::TimerCapture(new_timer, TimerCaptureUnit::Cpt2))));
    // Also check: the paired-channel pin on the new timer must be free in
    // the design (i.e. no other signal holds it).
    let pair_ch = match current_ch { HrtimCh::Ch1 => HrtimCh::Ch2, HrtimCh::Ch2 => HrtimCh::Ch1 };
    let pair_pin = pinout::pins_for(
        Signal::HrtimChannel { timer: new_timer, ch: pair_ch }, variant,
    ).first().copied();
    if timer_free {
        let note = match pair_pin {
            Some(p) => format!(
                "Retarget phase to {:?}. {:?} pair moves to {}",
                new_timer,
                match pair_ch { HrtimCh::Ch1 => "CH1", HrtimCh::Ch2 => "CH2" },
                p.name(),
            ),
            None => format!("Retarget phase to {:?}", new_timer),
        };
        MoveResult::Direct { note }
    } else {
        MoveResult::Blocked {
            reason: format!(
                "{:?} is already in use (its slots / paired pins are claimed elsewhere)",
                new_timer
            ),
        }
    }
}

fn compute_adc_move(
    design: &Design,
    variant: ChipVariant,
    _req_idx: usize,
    adc: AdcInstance,
    current_channel: u8,
    target: Pin,
    pin_signals: &HashMap<Pin, Signal>,
) -> MoveResult {
    let sigs = pinout::signals_on(target, variant);
    let target_channel = sigs.iter().find_map(|(_af, s)| match s {
        Signal::AdcIn { adc: a, channel } if *a == adc => Some(*channel),
        _ => None,
    });
    let Some(tch) = target_channel else {
        return MoveResult::NotApplicable;
    };
    let was_fast = is_fast_adc_channel(current_channel);
    let is_fast = is_fast_adc_channel(tch);
    let note = if was_fast == is_fast {
        format!(
            "ADC{}_IN{} -> IN{} (both {})",
            adc.number(), current_channel, tch,
            if is_fast { "fast" } else { "slow" },
        )
    } else {
        format!(
            "ADC{}_IN{} ({}) -> IN{} ({}) - speed changes, check Rain budget",
            adc.number(), current_channel, if was_fast { "fast" } else { "slow" },
            tch, if is_fast { "fast" } else { "slow" },
        )
    };

    let source_pin = pinout::pins_for(
        Signal::AdcIn { adc, channel: current_channel }, variant,
    ).first().copied();

    if let Some(occ) = pin_signals.get(&target).copied() {
        if matches!(occ, Signal::AdcIn { adc: a, channel: c } if a == adc && c == current_channel) {
            return MoveResult::NotApplicable;
        }
        // Treat the source pin as free for the cascade search (it'll be
        // vacated by the primary move).
        let mut ps = pin_signals.clone();
        if let Some(sp) = source_pin { ps.remove(&sp); }
        let mut blocked = std::collections::HashSet::new();
        blocked.insert(target);
        match find_relocation_chain(occ, design, variant, &ps, &mut blocked, 2) {
            Some(chain) if chain.len() == 1 => MoveResult::Cascade1 { chain, note },
            Some(chain) => MoveResult::CascadeMulti { chain, note },
            None => MoveResult::Blocked {
                reason: format!("Occupied by {} with no free relocation (depth <= 2)", occ.name()),
            },
        }
    } else if was_fast == is_fast {
        MoveResult::Direct { note }
    } else {
        MoveResult::Semantic { note }
    }
}

/// Alternative pins for a signal — either sibling ADC channels on the same
/// ADC (for AdcIn) or the signal's other AF mappings. Used by the cascade
/// search to find a fresh home for a displaced signal.
fn alt_pins_for(signal: Signal, variant: ChipVariant, design: &Design) -> Vec<Pin> {
    match signal {
        Signal::AdcIn { adc, channel } => {
            let mut out = Vec::new();
            for ch in 1..=18u8 {
                if ch == channel { continue; }
                let Some(&p) = pinout::pins_for(
                    Signal::AdcIn { adc, channel: ch }, variant,
                ).first() else { continue; };
                // The ADC channel itself must not be taken by another conversion.
                let taken = design.assignments.iter().any(|a| matches!(
                    a, Some(Assignment::AdcConversion { adc: aa, channel: cc, .. })
                        if *aa == adc && *cc == ch
                ));
                if !taken {
                    out.push(p);
                }
            }
            out
        }
        _ => pinout::pins_for(signal, variant),
    }
}

/// Search for a relocation chain for `signal`: a list of `(signal, new_pin)`
/// moves that frees the original pin while keeping every other signal
/// placed. `blocked` tracks pins already claimed by the in-progress path.
/// Depth bounds the search so the UI stays responsive.
fn find_relocation_chain(
    signal: Signal,
    design: &Design,
    variant: ChipVariant,
    pin_signals: &HashMap<Pin, Signal>,
    blocked: &mut std::collections::HashSet<Pin>,
    depth: usize,
) -> Option<Vec<(Signal, Pin)>> {
    if depth == 0 { return None; }
    for alt in alt_pins_for(signal, variant, design) {
        if blocked.contains(&alt) { continue; }
        let occupant = pin_signals.get(&alt).copied();
        match occupant {
            None => {
                return Some(vec![(signal, alt)]);
            }
            Some(occ) if occ == signal => {
                // Its own current pin; skip.
                continue;
            }
            Some(occ) => {
                blocked.insert(alt);
                let sub = find_relocation_chain(occ, design, variant, pin_signals, blocked, depth - 1);
                blocked.remove(&alt);
                if let Some(mut chain) = sub {
                    chain.insert(0, (signal, alt));
                    return Some(chain);
                }
            }
        }
    }
    None
}

fn compute_alt_pin_move(
    design: &Design,
    variant: ChipVariant,
    signal: Signal,
    current_pin: Pin,
    target: Pin,
    pin_signals: &HashMap<Pin, Signal>,
) -> MoveResult {
    if target == current_pin { return MoveResult::NotApplicable; }
    let alts = pinout::pins_for(signal, variant);
    if !alts.contains(&target) {
        return MoveResult::NotApplicable;
    }
    let note = format!("{} -> {}", signal.name(), target.name());
    if let Some(&occ) = pin_signals.get(&target) {
        // Source pin becomes free for cascade purposes.
        let mut ps = pin_signals.clone();
        ps.remove(&current_pin);
        let mut blocked = std::collections::HashSet::new();
        blocked.insert(target);
        match find_relocation_chain(occ, design, variant, &ps, &mut blocked, 2) {
            Some(chain) if chain.len() == 1 => MoveResult::Cascade1 { chain, note },
            Some(chain) => MoveResult::CascadeMulti { chain, note },
            None => MoveResult::Blocked {
                reason: format!("Occupied by {} with no free relocation (depth <= 2)", occ.name()),
            },
        }
    } else {
        MoveResult::Direct { note }
    }
}

/// A concrete pin-move to apply. `primary` swaps the picked role; `cascades`
/// (if any) are the chain of displacements, ordered nearest-to-farthest.
/// They're applied in reverse so each step lands on a freshly-vacated pin.
pub struct AppliedMove {
    pub primary: PrimaryMove,
    pub cascades: Vec<CascadeMove>,
}

pub enum PrimaryMove {
    /// Change an AdcConversion assignment to a new (adc, channel). The
    /// speed preference is relaxed to whichever the new channel is so
    /// normalize() doesn't revert the move.
    AdcSetChannel { req_idx: usize, adc: AdcInstance, new_channel: u8 },
    /// Update a signal's pin-lock (for AltPinSignal).
    PinLock { signal: Signal, new_pin: Pin, old_pin: Pin },
    /// Retarget a PCM phase to run on a different HRTIM timer.
    RetargetPhaseTimer { req_idx: usize, new_timer: HrtimId },
}

pub struct CascadeMove {
    pub signal: Signal,
    pub new_pin: Pin,
}

/// Resolve the user-intended move given the picked role and the destination
/// pin. Returns None if the destination isn't viable (e.g. NotApplicable or
/// Blocked).
pub fn resolve_move(
    design: &Design,
    variant: ChipVariant,
    role: PickedRole,
    target: Pin,
) -> Option<AppliedMove> {
    let pin_signals = current_pin_signals(design, variant);
    let r = compute_move(design, variant, role, target, &pin_signals);
    let cascades_from_chain = |chain: Vec<(Signal, Pin)>| -> Vec<CascadeMove> {
        chain.into_iter().map(|(signal, new_pin)| CascadeMove { signal, new_pin }).collect()
    };
    match (r, role) {
        (MoveResult::Direct { .. } | MoveResult::Semantic { .. },
         PickedRole::AdcConversion { req_idx, adc, .. }) => {
            let new_channel = pinout::signals_on(target, variant)
                .iter()
                .find_map(|(_, s)| match s {
                    Signal::AdcIn { adc: a, channel } if *a == adc => Some(*channel),
                    _ => None,
                })?;
            Some(AppliedMove {
                primary: PrimaryMove::AdcSetChannel { req_idx, adc, new_channel },
                cascades: Vec::new(),
            })
        }
        (MoveResult::Cascade1 { chain, .. } | MoveResult::CascadeMulti { chain, .. },
         PickedRole::AdcConversion { req_idx, adc, .. }) => {
            let new_channel = pinout::signals_on(target, variant)
                .iter()
                .find_map(|(_, s)| match s {
                    Signal::AdcIn { adc: a, channel } if *a == adc => Some(*channel),
                    _ => None,
                })?;
            Some(AppliedMove {
                primary: PrimaryMove::AdcSetChannel { req_idx, adc, new_channel },
                cascades: cascades_from_chain(chain),
            })
        }
        (MoveResult::Direct { .. } | MoveResult::Semantic { .. },
         PickedRole::AltPinSignal { signal, current_pin }) => {
            Some(AppliedMove {
                primary: PrimaryMove::PinLock { signal, new_pin: target, old_pin: current_pin },
                cascades: Vec::new(),
            })
        }
        (MoveResult::Cascade1 { chain, .. } | MoveResult::CascadeMulti { chain, .. },
         PickedRole::AltPinSignal { signal, current_pin }) => {
            Some(AppliedMove {
                primary: PrimaryMove::PinLock { signal, new_pin: target, old_pin: current_pin },
                cascades: cascades_from_chain(chain),
            })
        }
        (MoveResult::Direct { .. } | MoveResult::Semantic { .. },
         PickedRole::PcmPhaseChannel { req_idx, current_ch, .. }) => {
            let new_timer = pinout::signals_on(target, variant)
                .into_iter()
                .find_map(|(_, s)| match s {
                    Signal::HrtimChannel { timer, ch } if ch == current_ch => Some(timer),
                    _ => None,
                })?;
            Some(AppliedMove {
                primary: PrimaryMove::RetargetPhaseTimer { req_idx, new_timer },
                cascades: Vec::new(),
            })
        }
        _ => None,
    }
}

/// Apply an `AppliedMove` to the design. Cascade chain is applied in reverse
/// order so each step lands on a freshly-vacated pin before the next claim.
pub fn apply_move(design: &mut Design, variant: ChipVariant, mv: AppliedMove) {
    // Chain is ordered nearest → farthest: chain[0] leaves the primary's
    // target, chain[1] leaves chain[0]'s destination, etc. Applying in
    // reverse ensures the "farthest" move (landing on a free pin) is
    // scheduled first; each subsequent move then lands on the pin vacated
    // by the previous step.
    for c in mv.cascades.iter().rev() {
        apply_cascade_step(design, variant, c);
    }
    match mv.primary {
        PrimaryMove::AdcSetChannel { req_idx, adc, new_channel } => {
            if let RequirementSpec::AdcConversion { group, purpose, adc_pref, .. } =
                design.requirements[req_idx]
            {
                // Relax speed to match the destination channel so normalize
                // doesn't revert the move.
                design.set_spec(req_idx, RequirementSpec::AdcConversion {
                    group, purpose, adc_pref,
                    speed: SpeedPref::Any,
                });
                design.set_assignment(req_idx, Assignment::AdcConversion {
                    adc, channel: new_channel, purpose,
                });
            }
        }
        PrimaryMove::PinLock { signal, new_pin, old_pin: _ } => {
            design.set_pin(signal, new_pin);
        }
        PrimaryMove::RetargetPhaseTimer { req_idx, new_timer } => {
            // Mutate the PcmPhase / PcmPhaseExternal assignment to point at
            // the new timer, keeping everything else.
            let new_asn = match design.assignments.get(req_idx).and_then(|a| a.clone()) {
                Some(Assignment::PcmPhase { alloc, zcd_eev, dem, .. }) => {
                    Some(Assignment::PcmPhase { alloc, zcd_eev, timer: new_timer, dem })
                }
                Some(Assignment::PcmPhaseExternal { dac, peak_eev, zcd_eev, dem, .. }) => {
                    Some(Assignment::PcmPhaseExternal {
                        dac, peak_eev, zcd_eev, timer: new_timer, dem,
                    })
                }
                _ => None,
            };
            if let Some(a) = new_asn {
                design.set_assignment(req_idx, a);
            }
        }
    }
}

fn apply_cascade_step(design: &mut Design, variant: ChipVariant, c: &CascadeMove) {
    match c.signal {
        Signal::AdcIn { adc, channel: old_ch } => {
            // Translate new_pin → new_channel on the same ADC.
            let new_channel = pinout::signals_on(c.new_pin, variant).iter().find_map(|(_, s)| {
                match s {
                    Signal::AdcIn { adc: a, channel } if *a == adc => Some(*channel),
                    _ => None,
                }
            });
            if let Some(ch) = new_channel {
                for i in 0..design.assignments.len() {
                    if let Some(Assignment::AdcConversion { adc: a, channel: c_ch, purpose }) =
                        design.assignments[i]
                    {
                        if a == adc && c_ch == old_ch {
                            design.set_assignment(i, Assignment::AdcConversion {
                                adc, channel: ch, purpose,
                            });
                            if let RequirementSpec::AdcConversion {
                                group, purpose: p, adc_pref, ..
                            } = design.requirements[i] {
                                design.set_spec(i, RequirementSpec::AdcConversion {
                                    group, purpose: p, adc_pref,
                                    speed: SpeedPref::Any,
                                });
                            }
                            break;
                        }
                    }
                }
            }
        }
        other => {
            design.set_pin(other, c.new_pin);
        }
    }
}

/// Utility for app.rs — silence the unused-const warnings for colors
/// exported from package_view.
#[allow(dead_code)]
fn _keep_colors_alive() {
    let _ = [
        package_view::C_PIN_DIRECT,
        package_view::C_PIN_SEMANTIC,
        package_view::C_PIN_CASCADE1,
        package_view::C_PIN_CASCADE_MULTI,
        package_view::C_PIN_BLOCKED,
    ];
}
