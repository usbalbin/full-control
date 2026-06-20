//! The Capability Kernel's claim/conflict engine.
//! See `docs/constraint-selector-design.md` §1, §3.
//!
//! Everything here is generic over [`Res`] tokens: it accumulates claimed tokens
//! in a [`Ledger`] and reports a conflict iff a claim reuses an already-held
//! exclusive token (or repeats one within itself). It never inspects what a token
//! *is* — the single contract a family model must satisfy is [`Consume`].
//!
//! This is the no-choice baseline (greedy). Candidate enumeration (a consumer
//! with *several* possible token-sets) and bounded backtracking are the selector
//! path, added in later increments; the data structures here are shaped to carry
//! them (`used_excluding` is the "what's claimed by everyone else" query a
//! backtracker needs).

use std::collections::HashSet;

use super::tokens::Res;

/// Anything that reserves resources when placed — a peripheral use, a converter
/// leg, an HRTIM phase. The one contract that survives the family-specific
/// models (it mirrors `requirements.rs`'s `used_signals`/claim emission, but over
/// generic tokens instead of the typed G474 `Signal`).
pub trait Consume {
    /// The exclusive tokens this consumer holds in its *current* placement.
    fn consumed(&self) -> Vec<Res>;
}

/// A running tally of claimed exclusive tokens.
#[derive(Default, Clone, Debug)]
pub struct Ledger {
    held: HashSet<Res>,
}

impl Ledger {
    pub fn new() -> Self {
        Self::default()
    }

    /// The tokens claimed so far.
    pub fn held(&self) -> &HashSet<Res> {
        &self.held
    }

    pub fn is_empty(&self) -> bool {
        self.held.is_empty()
    }

    /// The tokens in `items` that can't be claimed — either already held, or
    /// repeated within `items` itself (a consumer demanding the same exclusive
    /// resource twice). Empty ⇒ `items` is claimable. This is the conflict
    /// *witness*; order follows `items`, duplicates included once per extra use.
    pub fn conflicts(&self, items: &[Res]) -> Vec<Res> {
        let mut within = HashSet::new();
        let mut out = Vec::new();
        for &r in items {
            // `within.insert` is false on a repeat inside this same claim.
            if self.held.contains(&r) || !within.insert(r) {
                out.push(r);
            }
        }
        out
    }

    /// True iff `items` collides with nothing held (and has no internal repeat).
    pub fn can_claim(&self, items: &[Res]) -> bool {
        self.conflicts(items).is_empty()
    }

    /// Claim `items` atomically: on ANY conflict, hold nothing new and return the
    /// conflicting tokens; otherwise add them all.
    pub fn claim(&mut self, items: &[Res]) -> Result<(), Vec<Res>> {
        let conflict = self.conflicts(items);
        if !conflict.is_empty() {
            return Err(conflict);
        }
        self.held.extend(items.iter().copied());
        Ok(())
    }

    /// [`claim`](Self::claim) a [`Consume`]r's current tokens.
    pub fn claim_consumer(&mut self, c: &impl Consume) -> Result<(), Vec<Res>> {
        self.claim(&c.consumed())
    }

    /// Release previously-claimed tokens (the backtracker's undo). Tokens not
    /// held are ignored, so releasing a claim that partially failed is safe.
    pub fn release(&mut self, items: &[Res]) {
        for r in items {
            self.held.remove(r);
        }
    }
}

/// The union of every consumer's tokens EXCEPT the one at `skip`. The
/// "what is everyone else holding" query — used to test whether one assignment
/// can move without colliding (and the seam the backtracker rebuilds against
/// when it retries a single requirement). Mirrors `requirements.rs`'s
/// `used_signals`-excluding pattern.
pub fn used_excluding<C: Consume>(consumers: &[C], skip: usize) -> HashSet<Res> {
    let mut s = HashSet::new();
    for (i, c) in consumers.iter().enumerate() {
        if i != skip {
            s.extend(c.consumed());
        }
    }
    s
}

/// Greedily claim each consumer's *current* tokens in order. `Ok(ledger)` if they
/// all fit; `Err((index, conflict))` at the first consumer that collides. With no
/// per-consumer choice this is also complete; consumers with several candidate
/// token-sets need the selector path (a later increment) for completeness.
pub fn place_greedy<C: Consume>(consumers: &[C]) -> Result<Ledger, (usize, Vec<Res>)> {
    let mut led = Ledger::new();
    for (i, c) in consumers.iter().enumerate() {
        if let Err(conflict) = led.claim_consumer(c) {
            return Err((i, conflict));
        }
    }
    Ok(led)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::tokens::{Res, RouteKind};

    /// A test consumer: a fixed bag of tokens.
    struct Bag(Vec<Res>);
    impl Consume for Bag {
        fn consumed(&self) -> Vec<Res> {
            self.0.clone()
        }
    }

    #[test]
    fn distinct_instances_coexist_same_instance_conflicts() {
        let mut led = Ledger::new();
        assert!(led.claim(&[Res::Inst("USART", 2)]).is_ok());
        assert!(led.claim(&[Res::Inst("USART", 3)]).is_ok(), "USART3 ≠ USART2");
        // Re-claiming USART2 collides.
        assert_eq!(led.claim(&[Res::Inst("USART", 2)]), Err(vec![Res::Inst("USART", 2)]));
    }

    #[test]
    fn claim_is_atomic_on_conflict() {
        let mut led = Ledger::new();
        led.claim(&[Res::Pin(pin('A', 2))]).unwrap();
        // A claim that mixes a free and a taken token must reserve NEITHER.
        let r = led.claim(&[Res::Pin(pin('A', 3)), Res::Pin(pin('A', 2))]);
        assert_eq!(r, Err(vec![Res::Pin(pin('A', 2))]));
        assert!(!led.held().contains(&Res::Pin(pin('A', 3))), "free token must NOT be claimed on failure");
        assert_eq!(led.held().len(), 1);
    }

    #[test]
    fn internal_duplicate_within_one_claim_is_a_conflict() {
        let mut led = Ledger::new();
        // A consumer cannot claim the same channel twice.
        let dup = Res::Pool("DMA1", 0);
        assert_eq!(led.claim(&[dup, dup]), Err(vec![dup]));
        assert!(led.is_empty());
    }

    #[test]
    fn pool_capacity_is_n_exclusive_supply_tokens() {
        // A pool of capacity 8 = tokens DMA1.0..DMA1.7. Nine single-channel
        // demands (e.g. 9 USART RxTx legs each taking one channel) can't all fit:
        // the 9th finds every supply token held. This is the doc's load-bearing
        // "demand ≤ pool capacity" check, with NO special pool arithmetic.
        const CAP: u8 = 8;
        let mut led = Ledger::new();
        for want in 0..9u8 {
            // Each demand tries the first free supply channel.
            let placed = (0..CAP).any(|ch| led.claim(&[Res::Pool("DMA1", ch)]).is_ok());
            if want < CAP {
                assert!(placed, "demand {want} should fit in an 8-channel pool");
            } else {
                assert!(!placed, "9th demand must NOT fit an 8-channel pool");
            }
        }
        assert_eq!(led.held().len(), CAP as usize);
    }

    #[test]
    fn route_and_subchannel_tokens_are_independent_axes() {
        let mut led = Ledger::new();
        led.claim(&[Res::Route(RouteKind::CompToTimBreak, 1, 1, 4)]).unwrap();
        // A different break input on the same comp+tim is a different edge.
        assert!(led.claim(&[Res::Route(RouteKind::CompToTimBreak, 1, 1, 6)]).is_ok());
        // The same edge collides.
        assert!(!led.can_claim(&[Res::Route(RouteKind::CompToTimBreak, 1, 1, 4)]));
        // SubChannels don't collide with Routes even with overlapping numbers.
        assert!(led.claim(&[Res::SubChannel("ADC", 1, 4)]).is_ok());
    }

    #[test]
    fn place_greedy_reports_first_infeasible_consumer() {
        let consumers = vec![
            Bag(vec![Res::Inst("SPI", 1), Res::Pin(pin('A', 5))]),
            Bag(vec![Res::Inst("SPI", 2)]),
            Bag(vec![Res::Pin(pin('A', 5))]), // collides with consumer 0's pin
        ];
        match place_greedy(&consumers) {
            Err((i, conflict)) => {
                assert_eq!(i, 2);
                assert_eq!(conflict, vec![Res::Pin(pin('A', 5))]);
            }
            Ok(_) => panic!("expected a pin collision"),
        }
    }

    #[test]
    fn used_excluding_omits_the_skipped_consumer() {
        let consumers = vec![
            Bag(vec![Res::Inst("ADC", 1)]),
            Bag(vec![Res::Inst("ADC", 2)]),
            Bag(vec![Res::Inst("ADC", 3)]),
        ];
        let others = used_excluding(&consumers, 1);
        assert!(others.contains(&Res::Inst("ADC", 1)));
        assert!(others.contains(&Res::Inst("ADC", 3)));
        assert!(!others.contains(&Res::Inst("ADC", 2)), "the skipped consumer is excluded");
    }

    fn pin(port: char, num: u8) -> crate::mcu_pinout::PinId {
        crate::mcu_pinout::PinId { port, num }
    }
}
