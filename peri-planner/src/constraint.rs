//! Generic, chip-agnostic constraint kernel for the MCU selector.
//!
//! The heart of the "Capability Kernel" architecture (see
//! `docs/constraint-selector-design.md`): every scarce thing — a peripheral
//! instance, a DMA channel, a pin, an internal silicon route — is reduced to an
//! opaque [`Res`] token. A [`Requirement`] offers a list of candidate
//! allocations (each a set of tokens it would lock); the engine assigns at most
//! one candidate per requirement such that no two assigned candidates share a
//! token. Capacity-N (a DMA pool of N channels) is modeled as N distinct
//! exclusive tokens, so "demand <= capacity" falls out of the single-consumer
//! rule with no special capacity concept.
//!
//! The kernel is deliberately MCU-agnostic: it never interprets a `Res`, only
//! tests set membership. Family-specific candidate *generation* (G474 HRTIM,
//! C531 legs, DMA routes from the descriptor) lives in adapters that feed this
//! kernel — added in later increments. This increment is the kernel + greedy
//! assignment; backtracking for the selector path is a later increment.
//!
//! It lives in `constraint`, not `solver`, to coexist with the legacy G474
//! `solver.rs` backtracker during migration (that file is retired once the
//! generic engine subsumes it).

use std::collections::HashSet;

use crate::mcu_pinout::PinId;

/// An opaque exclusion token. The engine only does set membership over these and
/// never pattern-matches one, so adding a peripheral family or resource kind
/// needs no engine change. The "class" / route-kind discriminators are interned
/// `&'static str` (metapac's own peripheral-class strings) rather than a closed
/// enum, so the vocabulary never needs widening per family.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum Res {
    /// A whole peripheral instance, e.g. `Inst("USART", 2)`, `Inst("TIM", 1)`.
    Inst(&'static str, u8),
    /// One channel of a DMA controller pool, e.g. `Pool("DMA1", 3)`. A pool of N
    /// channels is N of these tokens (indices `0..N`); per-pool "demand <=
    /// channels" is then just single-consumer exclusivity.
    Pool(&'static str, u8),
    /// A physical pin — the cross-fabric conflict token (an ADC channel can't
    /// land on a pin a USART TX needs).
    Pin(PinId),
    /// An opaque sub-resource of an instance (ADC input channel, HRTIM compare
    /// slot, …), kept opaque so the kernel needn't know family-specific
    /// register accounting.
    Sub(&'static str, u8, u8),
    /// An internal silicon route, e.g. `Route("comp_tim_break", comp, tim, brk)`
    /// or `Route("dac_to_comp", dac, ch, comp)`.
    Route(&'static str, u8, u8, u8),
}

/// A set of locked resource tokens.
pub type ResBag = HashSet<Res>;

/// One concrete way to satisfy a requirement: the tokens it would lock, plus a
/// human-readable label for diagnostics / UI.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Candidate {
    pub label: String,
    pub tokens: Vec<Res>,
}

impl Candidate {
    pub fn new(label: impl Into<String>, tokens: Vec<Res>) -> Self {
        Self { label: label.into(), tokens }
    }

    /// Whether every token this candidate needs is free given `used`.
    pub fn fits(&self, used: &ResBag) -> bool {
        self.tokens.iter().all(|t| !used.contains(t))
    }
}

/// A demand: an id and the candidate allocations that satisfy it, in preference
/// order. No candidates means inherently unsatisfiable on this chip.
#[derive(Clone, Debug)]
pub struct Requirement {
    pub id: u32,
    pub candidates: Vec<Candidate>,
}

/// The outcome of solving a requirement set.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Solution {
    /// `(requirement id, chosen candidate index)` for each satisfied requirement.
    pub assigned: Vec<(u32, usize)>,
    /// Requirement ids with no conflict-free candidate.
    pub unmet: Vec<u32>,
}

impl Solution {
    /// Whether every requirement was satisfied.
    pub fn is_feasible(&self) -> bool {
        self.unmet.is_empty()
    }
}

/// Greedy first-fit assignment: take requirements in order; each locks its first
/// candidate that doesn't collide with tokens already locked.
///
/// Fast, and correct for the live single-design UI. NOTE: greedy never
/// backtracks, so it can report a requirement unmet even when a globally
/// consistent assignment exists (see `greedy_can_miss_a_consistent_assignment`).
/// The selector path replaces this with bounded backtracking in a later
/// increment; this is the shared kernel both build on.
pub fn assign_greedy(reqs: &[Requirement]) -> Solution {
    let mut used: ResBag = HashSet::new();
    let mut sol = Solution::default();
    for req in reqs {
        match req.candidates.iter().position(|c| c.fits(&used)) {
            Some(idx) => {
                used.extend(req.candidates[idx].tokens.iter().copied());
                sol.assigned.push((req.id, idx));
            }
            None => sol.unmet.push(req.id),
        }
    }
    sol
}

#[cfg(test)]
mod tests {
    use super::*;

    fn req(id: u32, candidates: Vec<Candidate>) -> Requirement {
        Requirement { id, candidates }
    }

    #[test]
    fn exclusive_instance_assigned_once() {
        // Two requirements both want USART1; only one can hold it.
        let reqs = vec![
            req(1, vec![Candidate::new("USART1", vec![Res::Inst("USART", 1)])]),
            req(2, vec![Candidate::new("USART1", vec![Res::Inst("USART", 1)])]),
        ];
        let s = assign_greedy(&reqs);
        assert_eq!(s.assigned, vec![(1, 0)]);
        assert_eq!(s.unmet, vec![2]);
        assert!(!s.is_feasible());
    }

    #[test]
    fn pool_capacity_is_n_exclusive_tokens() {
        // A 2-channel pool: each leg may take ch0 OR ch1. Three legs => 2 fit
        // (ch0, ch1), the 3rd is unmet — "demand <= channels" with no capacity
        // concept, just exclusivity.
        let leg = || {
            vec![
                Candidate::new("DMA1.0", vec![Res::Pool("DMA1", 0)]),
                Candidate::new("DMA1.1", vec![Res::Pool("DMA1", 1)]),
            ]
        };
        let s = assign_greedy(&[req(1, leg()), req(2, leg()), req(3, leg())]);
        assert_eq!(s.assigned, vec![(1, 0), (2, 1)]);
        assert_eq!(s.unmet, vec![3]);
    }

    #[test]
    fn one_candidate_can_lock_many_tokens() {
        // A USART needing RX+TX DMA claims the instance and TWO channels at once.
        let reqs = vec![req(
            1,
            vec![Candidate::new(
                "usart1+rxtx",
                vec![Res::Inst("USART", 1), Res::Pool("DMA1", 0), Res::Pool("DMA1", 1)],
            )],
        )];
        let s = assign_greedy(&reqs);
        assert!(s.is_feasible() && s.assigned == vec![(1, 0)]);
    }

    #[test]
    fn no_candidates_is_unmet() {
        let s = assign_greedy(&[req(7, vec![])]);
        assert_eq!(s.unmet, vec![7]);
    }

    #[test]
    fn greedy_can_miss_a_consistent_assignment() {
        // A wants pin p1 or p2; B wants only p1. A consistent assignment exists
        // (A->p2, B->p1), but greedy takes p1 for A first and leaves B unmet.
        // Documents the limitation the selector's backtracking will remove.
        let p1 = PinId { port: 'A', num: 1 };
        let p2 = PinId { port: 'A', num: 2 };
        let reqs = vec![
            req(
                1,
                vec![
                    Candidate::new("p1", vec![Res::Pin(p1)]),
                    Candidate::new("p2", vec![Res::Pin(p2)]),
                ],
            ),
            req(2, vec![Candidate::new("p1", vec![Res::Pin(p1)])]),
        ];
        let s = assign_greedy(&reqs);
        assert_eq!(s.unmet, vec![2], "greedy first-fit leaves B unmet (known limitation)");
    }
}
