//! Lower user [`Requirement`]s onto a concrete [`McuDescriptor`]: enumerate
//! candidate (instance + DMA-channel) placements and run the kernel to a
//! feasibility verdict + witness. See `docs/constraint-selector-design.md` §3.
//!
//! This is the first end-to-end use of the kernel on real chip data, and the
//! first *adapter* — the pure engine ([`super::engine`]) and tokens
//! ([`super::tokens`]) stay chip-agnostic; this module is where descriptor data
//! becomes [`Res`] tokens (`Inst` for the instance, `Pool` for each DMA channel).
//!
//! Increment 4 is GREEDY: instances are tried in order and DMA channels grabbed
//! first-fit. The error direction is one-sided and safe:
//!   - SOUND — never a false positive. An `Allocated` verdict is always a real,
//!     conflict-free allocation (the `Ledger` claims atomically), so the selector
//!     never calls an incapable chip capable.
//!   - INCOMPLETE — possible false negative. A greedy instance/channel choice can
//!     strand a later requirement that a different choice would have satisfied, so
//!     `Infeasible` can occasionally be wrong (pessimistic, never optimistic).
//! Most-constrained-first ordering + backtracking close the gap in Increment 5.

use crate::mcu::McuDescriptor;

use super::engine::Ledger;
use super::requirement::{ReqKind, Requirement};
use super::tokens::{Class, Res};

/// One satisfied requirement: the instance it took and the DMA channels it holds.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Placed {
    pub req_id: u32,
    pub class: Class,
    pub instance: u8,
    /// `(controller pool, channel)` pairs claimed for this use.
    pub dma: Vec<(&'static str, u8)>,
}

/// The verdict for a requirement set on one chip.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Every requirement placed; the witness is a concrete, conflict-free
    /// allocation.
    Allocated(Vec<Placed>),
    /// One or more requirements couldn't be placed (the rest were greedily
    /// placed); `unmet` names them by id.
    Infeasible { unmet: Vec<u32> },
}

/// Instance indices of `class` present on the descriptor, sorted —
/// e.g. `"USART"` → `[1, 2, 3]`. The numeric-suffix requirement excludes
/// look-alikes (`"DMAMUX1"` is not a `"DMA"` instance).
fn instances_of(desc: &McuDescriptor, class: Class) -> Vec<u8> {
    let mut v: Vec<u8> = desc
        .raw
        .peripherals
        .iter()
        .filter_map(|p| p.name.strip_prefix(class).and_then(|s| s.parse::<u8>().ok()))
        .collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Capacity (channel count) of a controller pool by name; 0 if unknown (e.g. the
/// C5 descriptors before a metapac refresh — their DMA pools are empty, so any
/// DMA demand is honestly reported infeasible rather than waved through).
fn pool_capacity(desc: &McuDescriptor, pool: &str) -> u8 {
    desc.dma_pools().iter().find(|p| p.name == pool).map(|p| p.channels).unwrap_or(0)
}

/// Greedily reserve `need` DMA channels for instance `(class, i)` from the union
/// of its legs' allowed pools, avoiding channels already held or already chosen
/// in `extra`. Pushes the chosen `Pool` tokens onto `extra` and the `(pool, ch)`
/// witness onto `chans`. Returns false (leaving `extra`/`chans` to be discarded
/// by the caller) if the instance can't supply `need` free channels.
fn grab_channels(
    led: &Ledger,
    desc: &McuDescriptor,
    class: Class,
    i: u8,
    need: u8,
    extra: &mut Vec<Res>,
    chans: &mut Vec<(&'static str, u8)>,
) -> bool {
    // The instance's allowed pools, de-duplicated across its DMA legs.
    let routes = desc.dma_routes(&format!("{class}{i}"));
    let mut pools: Vec<&'static str> = routes.iter().flat_map(|leg| leg.pools.iter().copied()).collect();
    pools.sort_unstable();
    pools.dedup();

    let mut taken = 0u8;
    for pool in pools {
        for ch in 0..pool_capacity(desc, pool) {
            if taken == need {
                break;
            }
            let tok = Res::Pool(pool, ch);
            if !led.held().contains(&tok) && !extra.contains(&tok) {
                extra.push(tok);
                chans.push((pool, ch));
                taken += 1;
            }
        }
        if taken == need {
            break;
        }
    }
    taken == need
}

/// Decide whether `reqs` fit on `desc`, greedily. Each requirement claims a fresh
/// instance of its class plus its DMA channels; the kernel `Ledger` enforces that
/// nothing is shared.
pub fn feasible(desc: &McuDescriptor, reqs: &[Requirement]) -> Outcome {
    let mut led = Ledger::new();
    let mut placed = Vec::new();
    let mut unmet = Vec::new();

    for req in reqs {
        let ReqKind::UsePeripheral { class, dma } = req.kind;
        let mut done = false;
        for i in instances_of(desc, class) {
            let mut claim = vec![Res::Inst(class, i)];
            let mut chans = Vec::new();
            if dma > 0 && !grab_channels(&led, desc, class, i, dma, &mut claim, &mut chans) {
                continue; // this instance can't supply the channels; try the next
            }
            if led.claim(&claim).is_ok() {
                placed.push(Placed { req_id: req.id, class, instance: i, dma: chans });
                done = true;
                break;
            }
        }
        if !done {
            unmet.push(req.id);
        }
    }

    if unmet.is_empty() {
        Outcome::Allocated(placed)
    } else {
        Outcome::Infeasible { unmet }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mcu::Package;

    fn g474() -> &'static McuDescriptor {
        Package::G474R.descriptor()
    }

    fn req(id: u32, class: Class, dma: u8) -> Requirement {
        Requirement::use_peripheral(id, class, dma)
    }

    #[test]
    fn distinct_instances_allocate_until_exhausted() {
        let d = g474();
        let n = instances_of(d, "USART").len();
        assert!(n >= 3, "G474 should expose several USART instances (got {n})");
        // Exactly `n` USART uses fit on distinct instances...
        let reqs: Vec<_> = (0..n as u32).map(|k| req(k, "USART", 0)).collect();
        match feasible(d, &reqs) {
            Outcome::Allocated(p) => {
                let used: std::collections::BTreeSet<u8> = p.iter().map(|x| x.instance).collect();
                assert_eq!(used.len(), n, "each use must take a distinct instance");
            }
            o => panic!("expected all {n} to fit, got {o:?}"),
        }
        // ...but one more than exist does not.
        let too_many: Vec<_> = (0..=n as u32).map(|k| req(k, "USART", 0)).collect();
        assert!(
            matches!(feasible(d, &too_many), Outcome::Infeasible { unmet } if unmet == vec![n as u32]),
            "the (n+1)th USART use must be the one reported unmet",
        );
    }

    #[test]
    fn dma_channels_are_allocated_distinctly_and_within_capacity() {
        let d = g474();
        // Two USART uses, each wanting RX+TX DMA (2 channels) — 4 channels total,
        // well within G474's 16 (DMA1+DMA2). All channels must be distinct.
        let reqs = vec![req(0, "USART", 2), req(1, "USART", 2)];
        match feasible(d, &reqs) {
            Outcome::Allocated(p) => {
                let chans: Vec<_> = p.iter().flat_map(|x| x.dma.iter().copied()).collect();
                let uniq: std::collections::BTreeSet<_> = chans.iter().copied().collect();
                assert_eq!(chans.len(), 4, "2 uses × 2 channels");
                assert_eq!(uniq.len(), 4, "every claimed DMA channel must be distinct");
            }
            o => panic!("expected allocation, got {o:?}"),
        }
    }

    #[test]
    fn dma_demand_exceeding_pool_capacity_is_infeasible() {
        let d = g474();
        let total = d.dma_channel_total();
        assert!(total > 0, "G474 has DMA pool data");
        // Demand more channels than exist (across as many instances as needed):
        // each instance asks for the whole pool, so the 2nd already overflows.
        let big = (total + 1) as u8;
        let reqs = vec![req(0, "USART", big.min(u8::MAX))];
        // A single use demanding > total channels can't be satisfied by any instance.
        assert!(matches!(feasible(d, &reqs), Outcome::Infeasible { .. }));
    }

    #[test]
    fn dma_demand_on_c5_without_pool_data_is_infeasible_not_waved_through() {
        // C531's compiled descriptor has no DMA pool data at the pinned metapac
        // rev (a known gap). A DMA demand must be reported infeasible — never
        // silently "allocated" — while a no-DMA use of the same class still fits.
        let c531 = Package::C531R.descriptor();
        assert_eq!(c531.dma_channel_total(), 0, "precondition: C5 DMA pools empty here");
        assert!(matches!(
            feasible(c531, &[req(0, "USART", 1)]),
            Outcome::Infeasible { .. }
        ));
        // The instance itself exists, so a no-DMA use is fine.
        if instances_of(c531, "USART").is_empty() {
            return; // no USART on this package — nothing to assert
        }
        assert!(matches!(feasible(c531, &[req(0, "USART", 0)]), Outcome::Allocated(_)));
    }
}
