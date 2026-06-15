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
}
