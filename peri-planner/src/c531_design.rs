//! STM32C531 timer-PWM converter design state.
//!
//! A parallel model to the (HRTIM-shaped) G474 `Design` and the minimal
//! `H523Design`, following the same "fresh struct, not a unified type" choice
//! (see `h523_design.rs`). C531 has no HRTIM; it plans a DC/DC converter from
//! advanced-timer (TIM1/TIM8) PWM *legs*, each optionally carrying:
//!   - a complementary output pair + dead-time (half-bridge / sync-rect),
//!   - a hardware over-current trip routed COMP -> timer break input (the C5
//!     equivalent of HRTIM's EEV fault fold),
//!   - a DAC threshold feeding that comparator's inverting input,
//!   - an ADC channel sensing the leg.
//!
//! It reuses the shared *leaf* types (`TimId`, `CompId`, `DacId`,
//! `AdcInstance`, `Signal`, `pins_for_pkg`, the `ChipFabric` queries) but not
//! the G474 `Assignment`/solver machinery, which is HRTIM-coupled. The
//! cross-peripheral links (COMP->break, DAC->COMP) are internal silicon routing
//! validated against the chip fabric — they claim no pins; only the timer
//! channel/break/ETR signals resolve to GPIO.

use crate::g474::{AdcInstance, CompId, DacId, TimCh, TimId};
use crate::mcu::Package;
use crate::mcu_pinout::{OwnedSignal, PinId};
use crate::pinout::{pins_for_pkg, signal_to_owned, Pin, Signal};

/// Which timer break input a comparator trips: 1 = BRK, 2 = BRK2.
pub type BreakInput = u8;

/// A hardware over-current path: a comparator output routed to a timer break
/// input, optionally with a DAC setting the comparator's threshold. Whether the
/// routing is physically possible is validated against the chip fabric
/// (`ChipFabric::comps_for_tim_break` / `dac_threshold_sources_for_comp`).
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Ocp {
    pub comp: CompId,
    pub break_input: BreakInput,
    /// DAC channel driving the comparator's inverting-input threshold.
    pub threshold_dac: Option<DacId>,
}

/// One converter leg built on an advanced-control timer.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct ConverterLeg {
    pub tim: TimId,
    /// CH1..CH4 as a bitmask (bit 0 = CH1).
    pub channels_mask: u8,
    /// Claim the complementary CHxN outputs (half-bridge / synchronous rect).
    pub complementary: bool,
    /// Dead-time insertion — a timer property; surfaced as an annotation, it
    /// claims no pin or resource.
    pub dead_time: bool,
    /// Claim the external break-input *pin* (a board-level fault signal). This
    /// is independent of `ocp`, which is an internal comparator route.
    pub bkin: bool,
    /// Internal hardware over-current trip (COMP -> break), if any.
    pub ocp: Option<Ocp>,
    /// An ADC `(instance, channel)` sensing this leg.
    pub adc_sense: Option<(AdcInstance, u8)>,
}

impl ConverterLeg {
    /// A bare PWM leg on `tim` driving CH1 only — the starting point an Add
    /// button creates; flags are toggled from there.
    pub fn pwm(tim: TimId) -> Self {
        Self {
            tim,
            channels_mask: 0b0001,
            complementary: false,
            dead_time: false,
            bkin: false,
            ocp: None,
            adc_sense: None,
        }
    }

    /// The pin-bearing signals this leg claims: each enabled channel's CHx (and
    /// CHxN when complementary), plus the break-input pin when `bkin`. The
    /// COMP->break and DAC->COMP links are internal and contribute no signal.
    pub fn signals(&self) -> Vec<Signal> {
        let chs = [TimCh::Ch1, TimCh::Ch2, TimCh::Ch3, TimCh::Ch4];
        let mut out = Vec::new();
        for (i, ch) in chs.iter().enumerate() {
            if self.channels_mask & (1 << i as u8) != 0 {
                out.push(Signal::TimCh(self.tim, *ch));
                if self.complementary {
                    out.push(Signal::TimChN(self.tim, *ch));
                }
            }
        }
        if self.bkin {
            out.push(Signal::TimBkin(self.tim));
        }
        out
    }

    /// Each claimed signal paired with the pins it can occupy on `package`,
    /// resolved through the package-generic engine (`pins_for_pkg`). This is the
    /// concrete bridge from the converter model to the chip's GPIO.
    pub fn pin_candidates(&self, package: Package) -> Vec<(Signal, Vec<Pin>)> {
        self.signals()
            .into_iter()
            .map(|s| (s, pins_for_pkg(s, package)))
            .collect()
    }
}

/// CompId -> instance number (CompId carries no `number()` helper).
pub fn comp_num(c: CompId) -> u8 {
    match c {
        CompId::Comp1 => 1, CompId::Comp2 => 2, CompId::Comp3 => 3, CompId::Comp4 => 4,
        CompId::Comp5 => 5, CompId::Comp6 => 6, CompId::Comp7 => 7,
    }
}

/// Instance number -> CompId. Inverse of `comp_num`; the view uses it to turn
/// fabric query results (`comps_for_tim_break` yields raw `u8`s) back into the
/// typed ids the model stores.
pub fn comp_from_num(n: u8) -> Option<CompId> {
    Some(match n {
        1 => CompId::Comp1, 2 => CompId::Comp2, 3 => CompId::Comp3, 4 => CompId::Comp4,
        5 => CompId::Comp5, 6 => CompId::Comp6, 7 => CompId::Comp7,
        _ => return None,
    })
}

/// DacId -> (instance, channel). `DacId` packs both into one variant.
pub fn dac_inst_ch(d: DacId) -> (u8, u8) {
    match d {
        DacId::Dac1Ch1 => (1, 1), DacId::Dac1Ch2 => (1, 2),
        DacId::Dac2Ch1 => (2, 1),
        DacId::Dac3Ch1 => (3, 1), DacId::Dac3Ch2 => (3, 2),
        DacId::Dac4Ch1 => (4, 1), DacId::Dac4Ch2 => (4, 2),
    }
}

/// (instance, channel) -> DacId. Inverse of `dac_inst_ch`; turns the fabric's
/// `dac_threshold_sources_for_comp` pairs back into typed ids for the model.
pub fn dac_from_inst_ch(inst: u8, ch: u8) -> Option<DacId> {
    Some(match (inst, ch) {
        (1, 1) => DacId::Dac1Ch1, (1, 2) => DacId::Dac1Ch2,
        (2, 1) => DacId::Dac2Ch1,
        (3, 1) => DacId::Dac3Ch1, (3, 2) => DacId::Dac3Ch2,
        (4, 1) => DacId::Dac4Ch1, (4, 2) => DacId::Dac4Ch2,
        _ => return None,
    })
}

/// A reason a converter plan is not realizable on the chip. Validation is
/// resource-level (no pin placement yet); pin conflicts come from the generic
/// engine in the view layer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Problem {
    /// The chosen comparator cannot drive that timer's break input on this chip
    /// (wrong comparator, a non-advanced timer, or a non-existent break input).
    OcpUnroutable { leg: usize, comp: CompId, tim: TimId, break_input: BreakInput },
    /// The chosen DAC channel cannot set that comparator's inverting-input
    /// threshold on this chip.
    ThresholdUnroutable { leg: usize, dac: DacId, comp: CompId },
    /// Two legs claim the same timer instance.
    TimerConflict { legs: (usize, usize), tim: TimId },
    /// Two legs route their OCP through the same comparator.
    CompConflict { legs: (usize, usize), comp: CompId },
    /// Two legs use the same DAC channel as a threshold.
    DacConflict { legs: (usize, usize), dac: DacId },
    /// Two legs sense on the same ADC input channel.
    AdcConflict { legs: (usize, usize), adc: AdcInstance, channel: u8 },
    /// A leg senses on an ADC channel with no bonded input pin on this chip.
    SenseChannelUnavailable { leg: usize, adc: AdcInstance, channel: u8 },
}

/// The bonded single-ended input channels of `ADC{adc}` on `raw` — channels that
/// have a real `IN<n>` pin, i.e. the ones an ADC sense can actually reach.
fn adc_bonded_channels(raw: &'static crate::mcu_raw::RawMcuData, adc: u8) -> Vec<u8> {
    let name = format!("ADC{adc}");
    let mut chs: Vec<u8> = crate::mcu_pinout::af_rows(raw)
        .filter(|r| r.af.is_none() && r.signal.peripheral == name)
        .filter_map(|r| {
            let t = r.signal.role.strip_prefix("IN")?;
            if t.is_empty() || !t.bytes().all(|b| b.is_ascii_digit()) {
                return None;
            }
            t.parse().ok()
        })
        .collect();
    chs.sort_unstable();
    chs.dedup();
    chs
}

pub const C531_DESIGN_FORMAT_VERSION: u32 = 1;

/// The C531 converter plan: an ordered list of timer-PWM legs.
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct C531Design {
    pub format_version: u32,
    pub legs: Vec<ConverterLeg>,
    /// User-chosen pins for the converter's GPIO signals (timer channels / break
    /// pins). `to_pin_plan` honors a lock over its auto-materialized default; an
    /// unlisted signal is auto-placed. A pin serves at most one signal.
    #[serde(default)]
    pub pin_locks: Vec<(OwnedSignal, PinId)>,
}

impl C531Design {
    pub fn new() -> Self {
        Self { format_version: C531_DESIGN_FORMAT_VERSION, legs: Vec::new(), pin_locks: Vec::new() }
    }

    /// The pin the user locked for `signal`, if any.
    pub fn locked_pin(&self, signal: Signal) -> Option<PinId> {
        let owned = signal_to_owned(signal)?;
        self.pin_locks.iter().find(|(s, _)| *s == owned).map(|(_, p)| *p)
    }

    /// The signal currently locked onto `pin`, if any (for conflict display).
    pub fn occupant_of(&self, pin: PinId) -> Option<&OwnedSignal> {
        self.pin_locks.iter().find(|(_, p)| *p == pin).map(|(s, _)| s)
    }

    /// Lock `signal` to `pin`, evicting any prior lock on that signal or that pin
    /// (a pin serves one signal, a signal has one pin).
    pub fn lock_pin(&mut self, signal: Signal, pin: PinId) {
        let Some(owned) = signal_to_owned(signal) else { return };
        self.pin_locks.retain(|(s, p)| *s != owned && *p != pin);
        self.pin_locks.push((owned, pin));
    }

    /// Drop `signal`'s pin lock (revert to auto-placement).
    pub fn clear_pin(&mut self, signal: Signal) {
        if let Some(owned) = signal_to_owned(signal) {
            self.pin_locks.retain(|(s, _)| *s != owned);
        }
    }

    pub fn add_leg(&mut self, leg: ConverterLeg) {
        self.legs.push(leg);
    }

    pub fn remove_leg(&mut self, index: usize) {
        if index < self.legs.len() {
            self.legs.remove(index);
        }
    }

    /// Every reason this plan is not realizable on `package`: per-leg fabric
    /// routability (can the comparator reach that break input? can the DAC reach
    /// that comparator?) plus cross-leg resource conflicts (shared timer, comp,
    /// DAC channel, or ADC input). Empty == realizable at the resource level.
    pub fn validate(&self, package: Package) -> Vec<Problem> {
        let descriptor = package.descriptor();
        let fabric = descriptor.fabric;
        let mut problems = Vec::new();

        // Per-leg: an ADC sense channel must have a real input pin on this chip.
        for (i, leg) in self.legs.iter().enumerate() {
            if let Some((adc, ch)) = leg.adc_sense
                && !adc_bonded_channels(descriptor.raw, adc.number()).contains(&ch)
            {
                problems.push(Problem::SenseChannelUnavailable { leg: i, adc, channel: ch });
            }
        }

        // Per-leg: the internal COMP->break and DAC->COMP routes must exist in
        // the chip fabric. Without a modeled fabric we can't check, so skip.
        for (i, leg) in self.legs.iter().enumerate() {
            let Some(ocp) = &leg.ocp else { continue };
            let Some(fab) = fabric else { continue };

            if !fab
                .comps_for_tim_break(leg.tim.number(), ocp.break_input)
                .contains(&comp_num(ocp.comp))
            {
                problems.push(Problem::OcpUnroutable {
                    leg: i, comp: ocp.comp, tim: leg.tim, break_input: ocp.break_input,
                });
            }

            if let Some(dac) = ocp.threshold_dac
                && !fab
                    .dac_threshold_sources_for_comp(comp_num(ocp.comp))
                    .contains(&dac_inst_ch(dac))
                {
                    problems.push(Problem::ThresholdUnroutable { leg: i, dac, comp: ocp.comp });
                }
        }

        // Cross-leg: each exclusive resource may be claimed by only one leg.
        for a in 0..self.legs.len() {
            for b in (a + 1)..self.legs.len() {
                let (la, lb) = (&self.legs[a], &self.legs[b]);
                if la.tim == lb.tim {
                    problems.push(Problem::TimerConflict { legs: (a, b), tim: la.tim });
                }
                if let (Some(oa), Some(ob)) = (&la.ocp, &lb.ocp) {
                    if oa.comp == ob.comp {
                        problems.push(Problem::CompConflict { legs: (a, b), comp: oa.comp });
                    }
                    if let (Some(da), Some(db)) = (oa.threshold_dac, ob.threshold_dac)
                        && da == db {
                            problems.push(Problem::DacConflict { legs: (a, b), dac: da });
                        }
                }
                if let (Some(sa), Some(sb)) = (la.adc_sense, lb.adc_sense)
                    && sa == sb {
                        problems.push(Problem::AdcConflict {
                            legs: (a, b), adc: sa.0, channel: sa.1,
                        });
                    }
            }
        }

        problems
    }

    /// Whether the plan is realizable on `package` at the resource level.
    pub fn is_valid(&self, package: Package) -> bool {
        self.validate(package).is_empty()
    }

    /// Auto-allocate each leg's over-current route (COMP -> timer break, optional
    /// DAC threshold) and ADC-sense channel to a **conflict-free, fabric-valid**
    /// assignment — the non-HRTIM equivalent of the G474 solver. Preserves the
    /// user's INTENT (which legs want OCP / a DAC threshold / sense) and their
    /// timer + channel topology; only fills the internal routing + sense channel.
    /// Legs it can't satisfy on this chip's fabric are reported and left as-is.
    pub fn auto_assign(&mut self, package: Package) -> AutoAssign {
        let mut result = AutoAssign::default();
        let Some(fab) = package.descriptor().fabric else {
            return result; // no modeled fabric -> nothing to solve against
        };

        // ---- OCP: distinct-comp, distinct-dac routes, break-1 preferred ----
        let ocp_legs: Vec<usize> = self
            .legs
            .iter()
            .enumerate()
            .filter(|(_, l)| l.ocp.is_some())
            .map(|(i, _)| i)
            .collect();
        let cands: Vec<Vec<OcpCand>> = ocp_legs
            .iter()
            .map(|&i| {
                let leg = &self.legs[i];
                let want_dac = leg.ocp.as_ref().and_then(|o| o.threshold_dac).is_some();
                ocp_candidates(fab, leg.tim.number(), want_dac)
            })
            .collect();
        let assignment = max_ocp_assignment(&cands);
        for (slot, &leg_i) in ocp_legs.iter().enumerate() {
            match assignment[slot] {
                Some(c) => {
                    let ocp = self.legs[leg_i].ocp.as_mut().unwrap();
                    ocp.comp = comp_from_num(c.comp).unwrap();
                    ocp.break_input = c.brk;
                    ocp.threshold_dac = c.dac.and_then(|(di, dc)| dac_from_inst_ch(di, dc));
                }
                None => {
                    // Can't route this leg's OCP on this fabric. KEEP the user's
                    // intent (never silently delete it) and report it — validate()
                    // then surfaces it as a Problem the user resolves.
                    result.ocp_unassignable.push(leg_i);
                }
            }
        }

        // ---- ADC sense: a distinct BONDED channel per sensing leg (its own ADC).
        // Keep a leg's channel if it's already valid + conflict-free; only
        // reassign the rest, and only to channels with a real input pin.
        let raw = package.raw();
        let mut used: std::collections::HashSet<(u8, u8)> = std::collections::HashSet::new();
        let mut reassign: Vec<usize> = Vec::new();
        for (i, leg) in self.legs.iter().enumerate() {
            let Some((adc, ch)) = leg.adc_sense else { continue };
            if adc_bonded_channels(raw, adc.number()).contains(&ch)
                && used.insert((adc.number(), ch))
            {
                // Already a bonded, conflict-free channel — leave the user's choice.
            } else {
                reassign.push(i);
            }
        }
        for i in reassign {
            let (adc, _) = self.legs[i].adc_sense.unwrap();
            match adc_bonded_channels(raw, adc.number())
                .into_iter()
                .find(|ch| used.insert((adc.number(), *ch)))
            {
                Some(ch) => self.legs[i].adc_sense = Some((adc, ch)),
                // No free bonded channel left — keep intent, report it.
                None => result.sense_unassignable.push(i),
            }
        }
        result
    }

    /// Lower this converter plan into the unified
    /// [`PinPlan`](crate::pin_plan::PinPlan). One-way / derived (see
    /// `docs/firmware-codegen-design.md` §8).
    ///
    /// Unlike H5/C5A3, this model stores **no pins** — they are derived on
    /// demand. The lowerer therefore *materializes* one pin per signal,
    /// conflict-avoiding across the whole plan, and stores it **by value**
    /// (never a `pins_for_pkg` candidate index — data order can shift on
    /// regeneration). The OCP path (`leg.ocp`) is internal silicon routing that
    /// claims no pin, so it lowers to pinless fabric `routes`
    /// (`CompToTimBreak`, plus `DacToComp` for the threshold).
    ///
    /// Known unlowered gaps (the leg model can't express them): `TimEtr`,
    /// `TimBkin2`. Dead-time is structural (`PwmOut.dead_time`); the ns is a
    /// codegen hole.
    pub fn to_pin_plan(
        &self,
        package: Package,
        target: crate::pin_plan::Target,
    ) -> crate::pin_plan::PinPlan {
        use crate::mcu_pinout::af_rows;
        use crate::pin_plan::{
            EdgeKind, FabricNode, Placement, PinOrigin, PinPlan, RoleKind, RouteEdge,
        };
        use std::collections::{HashMap, HashSet};

        let raw = package.raw();
        // User pin choices, honored over the auto-materialized default.
        let locks: HashMap<OwnedSignal, PinId> = self.pin_locks.iter().cloned().collect();

        // Materialize one pin for a typed signal: a valid user lock wins; else the
        // first conflict-free candidate (single candidate => Forced, else Solver).
        // Stored BY VALUE.
        type Materialized = (crate::mcu_pinout::OwnedSignal, Option<PinId>, PinOrigin, Option<u8>);
        let materialize = |signal: Signal, taken: &mut HashSet<PinId>| -> Option<Materialized> {
            let owned = signal_to_owned(signal)?;
            let cands: Vec<PinId> = pins_for_pkg(signal, package)
                .into_iter()
                .map(|p| PinId { port: p.port, num: p.num })
                .collect();
            let (pin, origin) = if let Some(&locked) = locks.get(&owned)
                && cands.contains(&locked)
            {
                (Some(locked), PinOrigin::Locked)
            } else {
                match cands.as_slice() {
                    [] => (None, PinOrigin::Solver), // unreachable on this package
                    [only] => (Some(*only), PinOrigin::Forced),
                    many => {
                        let chosen = many.iter().copied().find(|p| !taken.contains(p)).unwrap_or(many[0]);
                        (Some(chosen), PinOrigin::Solver)
                    }
                }
            };
            if let Some(p) = pin {
                taken.insert(p);
            }
            let af = pin.and_then(|p| {
                af_rows(raw)
                    .find(|r| {
                        r.pin == p
                            && r.signal.peripheral == owned.peripheral
                            && r.signal.role == owned.role
                    })
                    .and_then(|r| r.af)
            });
            Some((owned, pin, origin, af))
        };

        let mut plan = PinPlan::empty(target);
        let mut taken: HashSet<PinId> = HashSet::new();
        // Reserve every user-locked pin so auto-placed signals never steal one.
        for (_, p) in &self.pin_locks {
            taken.insert(*p);
        }

        for (i, leg) in self.legs.iter().enumerate() {
            let group = Some(i as u32);

            // Pin-bearing signals: PWM channels (+ complementary) and the
            // external break-input pin when `bkin`.
            for sig in leg.signals() {
                let Some((signal, pin, origin, af)) = materialize(sig, &mut taken) else { continue };
                let role_kind = match sig {
                    Signal::TimCh(..) | Signal::TimChN(..) => RoleKind::PwmOut {
                        complementary: leg.complementary,
                        dead_time: leg.dead_time,
                        etr: false,
                    },
                    Signal::TimBkin(..) => RoleKind::FaultIn,
                    _ => RoleKind::Gpio,
                };
                plan.placements.push(Placement {
                    signal, pin, origin, af, role_kind,
                    irqs: Vec::new(), package_pin: None, net: None,
                });
            }

            // ADC sense: `signals()` drops it, so synthesize the AdcIn signal.
            if let Some((adc, channel)) = leg.adc_sense {
                let sig = Signal::AdcIn { adc, channel };
                if let Some((signal, pin, origin, af)) = materialize(sig, &mut taken) {
                    let adc_name = signal.peripheral.clone();
                    plan.placements.push(Placement {
                        signal, pin, origin, af,
                        role_kind: RoleKind::AdcInput {
                            adc: adc_name, channel, purpose: "sense".into(), sequencer_group: None,
                        },
                        irqs: Vec::new(), package_pin: None, net: None,
                    });
                }
            }

            // OCP: internal silicon route, no pins. COMP -> timer break input,
            // plus DAC -> COMP when a threshold DAC is set.
            if let Some(ocp) = &leg.ocp {
                let comp = FabricNode {
                    class: "COMP".into(), instance: comp_num(ocp.comp), channel: None, event: None,
                };
                let tim = FabricNode {
                    class: "TIM".into(), instance: leg.tim.number(), channel: None, event: None,
                };
                plan.routes.push(RouteEdge {
                    from: comp.clone(), to: tim,
                    kind: EdgeKind::CompToTimBreak { break_input: ocp.break_input }, group,
                });
                if let Some(dac) = ocp.threshold_dac {
                    let (di, dc) = dac_inst_ch(dac);
                    plan.routes.push(RouteEdge {
                        from: FabricNode {
                            class: "DAC".into(), instance: di, channel: Some(dc), event: None,
                        },
                        to: comp, kind: EdgeKind::DacToComp, group,
                    });
                }
            }
        }

        plan.sort_placements();
        plan
    }

    /// A copy-paste converter-plan summary for `part_name`.
    pub fn export_summary(&self, part_name: &str) -> String {
        use std::fmt::Write as _;
        let mut s = format!("{part_name} — converter plan ({} legs)\n", self.legs.len());
        for (i, leg) in self.legs.iter().enumerate() {
            let _ = write!(s, "  leg {}: {:?} ch-mask={:#06b}", i + 1, leg.tim, leg.channels_mask);
            if leg.complementary {
                let _ = write!(s, " complementary");
            }
            if leg.dead_time {
                let _ = write!(s, " dead-time");
            }
            if let Some(ocp) = &leg.ocp {
                let _ = write!(s, " OCP({:?}→BRK{})", ocp.comp, ocp.break_input);
            }
            if let Some((adc, ch)) = leg.adc_sense {
                let _ = write!(s, " sense({adc:?}.{ch})");
            }
            let _ = writeln!(s);
        }
        s
    }
}

/// Outcome of [`C531Design::auto_assign`]: the legs it could NOT satisfy.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct AutoAssign {
    /// Legs whose OCP request has no fabric-valid, conflict-free route here.
    pub ocp_unassignable: Vec<usize>,
    /// Legs whose ADC sense could not be given a free channel.
    pub sense_unassignable: Vec<usize>,
}

impl AutoAssign {
    pub fn fully_solved(&self) -> bool {
        self.ocp_unassignable.is_empty() && self.sense_unassignable.is_empty()
    }
}

#[derive(Clone, Copy)]
struct OcpCand {
    comp: u8,
    brk: u8,
    dac: Option<(u8, u8)>,
}

/// Candidate (comp, break, optional DAC) OCP routes for `tim`; break input 1
/// listed first (preferred). When `want_dac`, only comps that HAVE a DAC
/// threshold source qualify, paired with each such source.
fn ocp_candidates(fab: &crate::mcu::ChipFabric, tim: u8, want_dac: bool) -> Vec<OcpCand> {
    let mut out = Vec::new();
    for brk in [1u8, 2] {
        for comp in fab.comps_for_tim_break(tim, brk) {
            let dacs = fab.dac_threshold_sources_for_comp(comp);
            if want_dac {
                for (di, dc) in dacs {
                    out.push(OcpCand { comp, brk, dac: Some((di, dc)) });
                }
            } else {
                out.push(OcpCand { comp, brk, dac: None });
            }
        }
    }
    out
}

/// Assign as many legs as possible a distinct-comp, distinct-dac candidate
/// (exhaustive backtracking — the C5 fabric is tiny). Per-leg `Some`/`None`.
fn max_ocp_assignment(cands: &[Vec<OcpCand>]) -> Vec<Option<OcpCand>> {
    let n = cands.len();
    let mut cur = vec![None; n];
    let mut best = vec![None; n];
    let mut best_count = 0usize;
    let mut used_comp = std::collections::HashSet::new();
    let mut used_dac = std::collections::HashSet::new();
    rec_ocp(cands, 0, &mut cur, &mut used_comp, &mut used_dac, &mut best, &mut best_count);
    best
}

#[allow(clippy::too_many_arguments)]
fn rec_ocp(
    cands: &[Vec<OcpCand>],
    idx: usize,
    cur: &mut Vec<Option<OcpCand>>,
    used_comp: &mut std::collections::HashSet<u8>,
    used_dac: &mut std::collections::HashSet<(u8, u8)>,
    best: &mut Vec<Option<OcpCand>>,
    best_count: &mut usize,
) {
    if idx == cands.len() {
        let count = cur.iter().filter(|x| x.is_some()).count();
        if count > *best_count {
            *best_count = count;
            *best = cur.clone();
        }
        return;
    }
    // Leave this leg unassigned (so a fully-infeasible leg doesn't block others).
    cur[idx] = None;
    rec_ocp(cands, idx + 1, cur, used_comp, used_dac, best, best_count);
    // Or take any still-free candidate.
    for &cand in &cands[idx] {
        if used_comp.contains(&cand.comp) {
            continue;
        }
        if let Some(d) = cand.dac
            && used_dac.contains(&d) {
                continue;
            }
        cur[idx] = Some(cand);
        used_comp.insert(cand.comp);
        if let Some(d) = cand.dac {
            used_dac.insert(d);
        }
        rec_ocp(cands, idx + 1, cur, used_comp, used_dac, best, best_count);
        used_comp.remove(&cand.comp);
        if let Some(d) = cand.dac {
            used_dac.remove(&d);
        }
        cur[idx] = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mk_ocp_leg(tim: TimId, comp: CompId, dac: Option<DacId>, sense: Option<(AdcInstance, u8)>) -> ConverterLeg {
        ConverterLeg {
            tim,
            channels_mask: 0b0001,
            complementary: false,
            dead_time: false,
            bkin: false,
            ocp: Some(Ocp { comp, break_input: 1, threshold_dac: dac }),
            adc_sense: sense,
        }
    }

    #[test]
    fn pin_lock_overrides_auto_placement_and_reverts_on_clear() {
        use crate::g474::TimCh;

        let mut d = C531Design::new();
        d.add_leg(ConverterLeg::pwm(TimId::Tim1)); // CH1 only
        let sig = Signal::TimCh(TimId::Tim1, TimCh::Ch1);
        let cands = pins_for_pkg(sig, Package::C531R);
        assert!(cands.len() >= 2, "TIM1 CH1 has multiple candidate pins on C531: {cands:?}");

        let auto = PinId { port: cands[0].port, num: cands[0].num };
        let chosen = PinId { port: cands[1].port, num: cands[1].num }; // NOT the auto default

        let plan_kind = |d: &C531Design| {
            d.to_pin_plan(Package::C531R, crate::pin_plan::Target { package: "C531R".into(), family: "C5".into() })
                .placements
                .into_iter()
                .find(|p| p.signal.peripheral == "TIM1" && p.signal.role == "CH1")
                .map(|p| (p.pin, p.origin))
                .unwrap()
        };

        // Default: auto-placed on the first candidate.
        assert_eq!(plan_kind(&d), (Some(auto), crate::pin_plan::PinOrigin::Solver));

        // Locked: the lowerer uses the user's pin with a Locked origin.
        d.lock_pin(sig, chosen);
        assert_eq!(d.locked_pin(sig), Some(chosen));
        assert_eq!(plan_kind(&d), (Some(chosen), crate::pin_plan::PinOrigin::Locked));

        // Cleared: reverts to the auto default.
        d.clear_pin(sig);
        assert_eq!(d.locked_pin(sig), None);
        assert_eq!(plan_kind(&d), (Some(auto), crate::pin_plan::PinOrigin::Solver));
    }

    #[test]
    fn auto_assign_resolves_comp_and_adc_conflicts() {
        // Two OCP legs both initially on COMP1 + ADC1 ch1 (double conflict), no
        // DAC threshold — the solver must split them onto distinct comps/channels.
        let mut d = C531Design::new();
        d.add_leg(mk_ocp_leg(TimId::Tim1, CompId::Comp1, None, Some((AdcInstance::Adc1, 1))));
        d.add_leg(mk_ocp_leg(TimId::Tim8, CompId::Comp1, None, Some((AdcInstance::Adc1, 1))));
        assert!(!d.is_valid(Package::C531R), "starts conflicted");

        let r = d.auto_assign(Package::C531R);
        assert!(r.fully_solved(), "both legs assignable on C531: {r:?}");
        assert!(d.is_valid(Package::C531R), "solver produced a realizable plan");
        assert_ne!(
            d.legs[0].ocp.as_ref().unwrap().comp,
            d.legs[1].ocp.as_ref().unwrap().comp,
            "distinct comparators"
        );
        assert_ne!(d.legs[0].adc_sense, d.legs[1].adc_sense, "distinct ADC channels");
    }

    #[test]
    fn auto_assign_reports_c531_single_dac_threshold_limit() {
        // On C531 only COMP1 has an internal DAC threshold, so at most ONE
        // DAC-thresholded OCP leg can route — the solver surfaces the hardware limit.
        let mut d = C531Design::new();
        d.add_leg(mk_ocp_leg(TimId::Tim1, CompId::Comp1, Some(DacId::Dac1Ch1), None));
        d.add_leg(mk_ocp_leg(TimId::Tim8, CompId::Comp1, Some(DacId::Dac1Ch1), None));
        let r = d.auto_assign(Package::C531R);
        assert_eq!(r.ocp_unassignable.len(), 1, "second DAC-thresholded OCP is unroutable: {r:?}");
        assert!(!r.fully_solved());
        // Intent is KEPT, never silently deleted — both legs still carry their OCP.
        assert!(d.legs.iter().all(|l| l.ocp.is_some()), "auto_assign must not delete OCP intent");
        // …and the unroutable leg is surfaced by validate() as a real problem
        // (the two legs now conflict on COMP1), not swept under the rug.
        assert!(!d.is_valid(Package::C531R), "the kept-but-unroutable leg is reported by validate");
    }

    #[test]
    fn auto_assign_sense_uses_only_bonded_channels_and_keeps_valid_ones() {
        let raw = Package::C531R.raw();
        let bonded: u8 = crate::mcu_pinout::af_rows(raw)
            .filter(|r| r.af.is_none() && r.signal.peripheral == "ADC1")
            .filter_map(|r| {
                let t = r.signal.role.strip_prefix("IN")?;
                (!t.is_empty() && t.bytes().all(|b| b.is_ascii_digit())).then(|| t.parse().ok()).flatten()
            })
            .next()
            .expect("ADC1 has a bonded channel on C531");

        // (a) A leg already on a valid bonded channel is NOT clobbered.
        let mut d = C531Design::new();
        let mut leg = ConverterLeg::pwm(TimId::Tim1);
        leg.adc_sense = Some((AdcInstance::Adc1, bonded));
        d.add_leg(leg);
        d.auto_assign(Package::C531R);
        assert_eq!(d.legs[0].adc_sense, Some((AdcInstance::Adc1, bonded)), "valid channel preserved");

        // (b) A non-existent channel is flagged by validate and reassigned to a
        // real bonded channel by auto_assign.
        let mut d2 = C531Design::new();
        let mut leg2 = ConverterLeg::pwm(TimId::Tim8);
        leg2.adc_sense = Some((AdcInstance::Adc1, 250));
        d2.add_leg(leg2);
        assert!(
            d2.validate(Package::C531R).iter().any(|p| matches!(p, Problem::SenseChannelUnavailable { .. })),
            "a channel with no input pin is reported"
        );
        d2.auto_assign(Package::C531R);
        assert_ne!(d2.legs[0].adc_sense, Some((AdcInstance::Adc1, 250)), "auto_assign moved off the bad channel");
        assert!(
            !d2.validate(Package::C531R).iter().any(|p| matches!(p, Problem::SenseChannelUnavailable { .. })),
            "no unavailable-channel problem after auto_assign"
        );
    }

    /// A canonical synchronous buck leg on C531: TIM1 CH1 + CH1N complementary
    /// with dead-time, hardware OCP via COMP1 on BRK with a DAC1 threshold, and
    /// an ADC1 sense channel. Its claimed signals and their C531 pin candidates
    /// must resolve through the generic engine — proving the timer-PWM converter
    /// primitive composes on a non-G474 chip.
    #[test]
    fn buck_leg_signals_and_pins_resolve_on_c531() {
        let leg = ConverterLeg {
            tim: TimId::Tim1,
            channels_mask: 0b0001,
            complementary: true,
            dead_time: true,
            bkin: false,
            ocp: Some(Ocp {
                comp: CompId::Comp1,
                break_input: 1,
                threshold_dac: Some(DacId::Dac1Ch1),
            }),
            adc_sense: Some((AdcInstance::Adc1, 1)),
        };

        // Complementary CH1: exactly TIM1 CH1 + CH1N, no break pin (OCP is
        // internal), no spurious channels.
        let sigs = leg.signals();
        assert_eq!(
            sigs,
            vec![
                Signal::TimCh(TimId::Tim1, TimCh::Ch1),
                Signal::TimChN(TimId::Tim1, TimCh::Ch1),
            ],
        );

        // The high-side CH1 output must resolve to at least one real C531 pin.
        let cands = leg.pin_candidates(Package::C531R);
        let ch1 = cands
            .iter()
            .find(|(s, _)| matches!(s, Signal::TimCh(TimId::Tim1, TimCh::Ch1)))
            .expect("CH1 signal present");
        assert!(!ch1.1.is_empty(), "TIM1 CH1 should resolve to a C531 pin");
    }

    #[test]
    fn to_pin_plan_materializes_pins_and_lifts_ocp_to_routes() {
        use crate::pin_plan::{EdgeKind, PinOrigin, RoleKind, Target};
        let mut d = C531Design::new();
        d.add_leg(ConverterLeg {
            tim: TimId::Tim1,
            channels_mask: 0b0001,
            complementary: true,
            dead_time: true,
            bkin: false,
            ocp: Some(Ocp { comp: CompId::Comp1, break_input: 1, threshold_dac: Some(DacId::Dac1Ch1) }),
            adc_sense: Some((AdcInstance::Adc1, 1)),
        });

        let plan = d.to_pin_plan(
            Package::C531R,
            Target { package: "C531R".into(), family: "C5".into() },
        );

        // HS (CH1) and LS (CH1N) both materialized to DISTINCT real pins.
        let ch1 = plan.placements.iter().find(|p| p.signal.role == "CH1").expect("CH1");
        let ch1n = plan.placements.iter().find(|p| p.signal.role == "CH1N").expect("CH1N");
        assert!(ch1.pin.is_some() && ch1n.pin.is_some());
        assert_ne!(ch1.pin, ch1n.pin, "HS and LS must land on distinct pins");
        assert!(matches!(
            ch1.role_kind,
            RoleKind::PwmOut { complementary: true, dead_time: true, etr: false }
        ));
        assert!(matches!(ch1.origin, PinOrigin::Solver | PinOrigin::Forced));
        // (af is looked up but data-dependent — C5 descriptor data carries no AF
        // number for timer channels; the H523 test covers the af-present path.)

        // ADC sense materialized; analog pin so af is None; AdcInput role.
        let sense = plan
            .placements
            .iter()
            .find(|p| matches!(p.role_kind, RoleKind::AdcInput { .. }))
            .expect("adc sense placement");
        assert!(sense.pin.is_some());
        assert_eq!(sense.af, None, "an ADC input is a dedicated analog pin");

        // OCP lowers to pinless fabric routes: COMP1 -> TIM1 break, DAC1 -> COMP1.
        assert!(plan.routes.iter().any(|r| matches!(r.kind, EdgeKind::CompToTimBreak { break_input: 1 })
            && r.from.class == "COMP" && r.from.instance == 1
            && r.to.class == "TIM" && r.to.instance == 1));
        assert!(plan.routes.iter().any(|r| matches!(r.kind, EdgeKind::DacToComp)
            && r.from.class == "DAC" && r.to.class == "COMP" && r.to.instance == 1));
        assert_eq!(plan.target.family, "C5");
    }

    #[test]
    fn pwm_constructor_is_single_channel() {
        let leg = ConverterLeg::pwm(TimId::Tim8);
        assert_eq!(leg.channels_mask, 0b0001);
        assert_eq!(leg.signals(), vec![Signal::TimCh(TimId::Tim8, TimCh::Ch1)]);
    }

    #[test]
    fn design_add_remove_round_trips_through_serde() {
        let mut d = C531Design::new();
        assert_eq!(d.format_version, C531_DESIGN_FORMAT_VERSION);
        d.add_leg(ConverterLeg::pwm(TimId::Tim1));
        d.add_leg(ConverterLeg::pwm(TimId::Tim8));
        assert_eq!(d.legs.len(), 2);

        let json = serde_json::to_string(&d).unwrap();
        let back: C531Design = serde_json::from_str(&json).unwrap();
        assert_eq!(d, back);

        d.remove_leg(0);
        assert_eq!(d.legs.len(), 1);
        assert_eq!(d.legs[0].tim, TimId::Tim8);
    }

    /// Helper: a leg with an OCP route and ADC sense.
    fn ocp_leg(
        tim: TimId, comp: CompId, brk: BreakInput, dac: Option<DacId>, adc: (AdcInstance, u8),
    ) -> ConverterLeg {
        ConverterLeg {
            tim, channels_mask: 0b0001, complementary: true, dead_time: true, bkin: false,
            ocp: Some(Ocp { comp, break_input: brk, threshold_dac: dac }),
            adc_sense: Some(adc),
        }
    }

    /// Two well-formed legs that respect the C531 fabric — TIM1/COMP1/BRK with a
    /// DAC1 threshold, TIM8/COMP2/BRK2 with NO DAC threshold (COMP2 has no
    /// internal DAC on C531 — only DAC1->COMP1 exists), distinct ADC channels —
    /// have no problems.
    #[test]
    fn valid_two_leg_design_has_no_problems() {
        let mut d = C531Design::new();
        d.add_leg(ocp_leg(TimId::Tim1, CompId::Comp1, 1, Some(DacId::Dac1Ch1), (AdcInstance::Adc1, 1)));
        d.add_leg(ocp_leg(TimId::Tim8, CompId::Comp2, 2, None, (AdcInstance::Adc1, 2)));
        assert!(d.is_valid(Package::C531R), "got {:?}", d.validate(Package::C531R));
    }

    /// Routability: COMP3 doesn't exist in the C531 break fabric (only COMP1/2),
    /// and COMP1's threshold can only come from dac1_ch1 — dac1_ch2 is invalid.
    #[test]
    fn detects_unroutable_ocp_and_threshold() {
        // COMP3 can't drive any C531 timer break.
        let mut bad_comp = C531Design::new();
        bad_comp.add_leg(ocp_leg(TimId::Tim1, CompId::Comp3, 1, None, (AdcInstance::Adc1, 1)));
        assert_eq!(
            bad_comp.validate(Package::C531R),
            vec![Problem::OcpUnroutable {
                leg: 0, comp: CompId::Comp3, tim: TimId::Tim1, break_input: 1,
            }],
        );

        // A non-advanced timer (TIM2) has no comparator break path either.
        let mut bad_tim = C531Design::new();
        bad_tim.add_leg(ocp_leg(TimId::Tim2, CompId::Comp1, 1, None, (AdcInstance::Adc1, 1)));
        assert_eq!(
            bad_tim.validate(Package::C531R),
            vec![Problem::OcpUnroutable {
                leg: 0, comp: CompId::Comp1, tim: TimId::Tim2, break_input: 1,
            }],
        );

        // COMP1 threshold from dac1_ch2 is not a valid route (Table 172).
        let mut bad_dac = C531Design::new();
        bad_dac.add_leg(ocp_leg(TimId::Tim1, CompId::Comp1, 1, Some(DacId::Dac1Ch2), (AdcInstance::Adc1, 1)));
        assert_eq!(
            bad_dac.validate(Package::C531R),
            vec![Problem::ThresholdUnroutable {
                leg: 0, dac: DacId::Dac1Ch2, comp: CompId::Comp1,
            }],
        );
    }

    /// Cross-leg conflicts: same timer, same comparator, same DAC, same ADC.
    #[test]
    fn detects_cross_leg_resource_conflicts() {
        let mut d = C531Design::new();
        // Both on TIM1, both COMP1/dac1_ch1, both sensing ADC1 ch1 — every
        // exclusive resource collides.
        d.add_leg(ocp_leg(TimId::Tim1, CompId::Comp1, 1, Some(DacId::Dac1Ch1), (AdcInstance::Adc1, 1)));
        d.add_leg(ocp_leg(TimId::Tim1, CompId::Comp1, 1, Some(DacId::Dac1Ch1), (AdcInstance::Adc1, 1)));
        let problems = d.validate(Package::C531R);
        assert!(problems.contains(&Problem::TimerConflict { legs: (0, 1), tim: TimId::Tim1 }));
        assert!(problems.contains(&Problem::CompConflict { legs: (0, 1), comp: CompId::Comp1 }));
        assert!(problems.contains(&Problem::DacConflict { legs: (0, 1), dac: DacId::Dac1Ch1 }));
        assert!(problems.contains(&Problem::AdcConflict {
            legs: (0, 1), adc: AdcInstance::Adc1, channel: 1,
        }));
    }
}
