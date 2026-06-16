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
use crate::pinout::{pins_for_pkg, Pin, Signal};

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
}

pub const C531_DESIGN_FORMAT_VERSION: u32 = 1;

/// The C531 converter plan: an ordered list of timer-PWM legs.
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct C531Design {
    pub format_version: u32,
    pub legs: Vec<ConverterLeg>,
}

impl C531Design {
    pub fn new() -> Self {
        Self { format_version: C531_DESIGN_FORMAT_VERSION, legs: Vec::new() }
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

            if let Some(dac) = ocp.threshold_dac {
                if !fab
                    .dac_threshold_sources_for_comp(comp_num(ocp.comp))
                    .contains(&dac_inst_ch(dac))
                {
                    problems.push(Problem::ThresholdUnroutable { leg: i, dac, comp: ocp.comp });
                }
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
                    if let (Some(da), Some(db)) = (oa.threshold_dac, ob.threshold_dac) {
                        if da == db {
                            problems.push(Problem::DacConflict { legs: (a, b), dac: da });
                        }
                    }
                }
                if let (Some(sa), Some(sb)) = (la.adc_sense, lb.adc_sense) {
                    if sa == sb {
                        problems.push(Problem::AdcConflict {
                            legs: (a, b), adc: sa.0, channel: sa.1,
                        });
                    }
                }
            }
        }

        problems
    }

    /// Whether the plan is realizable on `package` at the resource level.
    pub fn is_valid(&self, package: Package) -> bool {
        self.validate(package).is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
