//! Lower user [`Requirement`]s onto a concrete [`McuDescriptor`]: enumerate
//! candidate (instance + DMA-channel) placements and run the kernel to a
//! feasibility verdict + witness. See `docs/constraint-selector-design.md` §3.
//!
//! This is the first end-to-end use of the kernel on real chip data, and the
//! first *adapter* — the pure engine ([`super::engine`]) and tokens
//! ([`super::tokens`]) stay chip-agnostic; this module is where descriptor data
//! becomes [`Res`] tokens (`Inst` for the instance, `Pool` for each DMA channel).
//!
//! Increment 5 makes the verdict COMPLETE for the instance-contention that
//! pinned requirements create: `feasible` backtracks over instance assignment,
//! most-constrained-requirement first, bounded by problem size (single-digit reqs
//! × instances) with NO wall-clock timeout — a timeout on a deep-but-satisfiable
//! instance would itself be an unsound false negative. It stays SOUND (an
//! `Allocated` witness is always a real, conflict-free allocation).
//!
//! Increment 6 adds PIN contention: a placed instance also claims a physical pin
//! (`Res::Pin`) for each of its required signals (SERIAL → TX/RX, SPI →
//! SCK/MOSI/MISO, I2C → SCL/SDA, UCPD → CC1/CC2; ADC is analog and claims none),
//! so two peripherals can't be routed to the same pad. Instance choice stays
//! fully backtracked (complete); pins, like DMA channels, are grabbed greedily
//! per role — which catches every FORCED collision (a single-candidate pin
//! already taken) and pin exhaustion, but can be pessimistic (never optimistic)
//! when a role has several candidate pads and a different choice would have fit.
//! That one-sided error keeps the selector sound; full pin-choice backtracking is
//! a later refinement. Classes outside the modeled kinds get no pin contention.

use crate::mcu::McuDescriptor;
use crate::mcu_pinout::{pins_for, PinId, SignalId};

use super::engine::Ledger;
use super::requirement::{ReqKind, Requirement};
use super::tokens::{Class, Res};

/// One satisfied requirement: the instance, the pins, and the DMA channels it holds.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Placed {
    pub req_id: u32,
    pub class: Class,
    pub instance: u8,
    /// `(role, pin)` pairs claimed for this use's required signals.
    pub pins: Vec<(&'static str, PinId)>,
    /// `(controller pool, channel)` pairs claimed for this use.
    pub dma: Vec<(&'static str, u8)>,
}

/// The verdict for a requirement set on one chip.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Every requirement placed; the witness is a concrete, conflict-free
    /// allocation.
    Allocated(Vec<Placed>),
    /// No complete assignment exists. `unmet` is a best-effort list of culprit
    /// ids (from a greedy pass), not necessarily minimal — the verdict itself is
    /// authoritative (backtracking proved no full assignment).
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

/// Controller pools instance `(class, i)` can draw DMA from (the union of its
/// legs), each with its channel capacity. Interchangeable for the greedy channel
/// grab (see module docs).
fn allowed_pools(desc: &McuDescriptor, class: Class, i: u8) -> Vec<(&'static str, u8)> {
    let routes = desc.dma_routes(&format!("{class}{i}"));
    let names: std::collections::BTreeSet<&'static str> =
        routes.iter().flat_map(|leg| leg.pools.iter().copied()).collect();
    names.into_iter().map(|n| (n, pool_capacity(desc, n))).collect()
}

/// Free channels currently available to instance `(class, i)` across its pools.
fn free_channels(led: &Ledger, desc: &McuDescriptor, class: Class, i: u8) -> u8 {
    allowed_pools(desc, class, i)
        .iter()
        .map(|(pool, cap)| (0..*cap).filter(|&ch| !led.held().contains(&Res::Pool(pool, ch))).count() as u8)
        .sum()
}

/// The GPIO signal roles an instance of `class` must place on pins (its minimal
/// functional config). Empty for analog (ADC) and for classes outside the
/// modeled kinds — those get no pin contention.
fn required_roles(class: Class) -> Vec<&'static str> {
    match crate::select::kind_of(class) {
        Some(kind) => crate::select::required_signals(kind, &[]),
        None => Vec::new(),
    }
}

/// The descriptor's `&'static` peripheral name for `(class, i)` (e.g. "USART2"),
/// needed to key `SignalId` for pin lookup. `None` if not on the chip.
fn static_inst_name(desc: &McuDescriptor, class: Class, i: u8) -> Option<&'static str> {
    let target = format!("{class}{i}");
    desc.raw.peripherals.iter().find(|p| p.name == target).map(|p| p.name)
}

/// Whether instance `(class, i)` can host a use needing `dma` channels right now:
/// the instance is free, enough DMA channels remain, and each required role has
/// at least one free candidate pin (a necessary check for MRV ordering; `commit`
/// enforces the full per-role distinctness).
fn can_place(led: &Ledger, desc: &McuDescriptor, class: Class, i: u8, dma: u8) -> bool {
    if led.held().contains(&Res::Inst(class, i)) {
        return false;
    }
    if dma > 0 && free_channels(led, desc, class, i) < dma {
        return false;
    }
    let Some(pname) = static_inst_name(desc, class, i) else {
        return true; // not on the chip via name lookup — instance check above suffices
    };
    required_roles(class).into_iter().all(|role| {
        pins_for(desc.raw, SignalId { peripheral: pname, role })
            .iter()
            .any(|p| !led.held().contains(&Res::Pin(*p)))
    })
}

/// What a successful [`commit`] claimed.
struct Commit {
    /// Every token claimed (to release on backtrack): `Inst` + `Pin`s + `Pool`s.
    tokens: Vec<Res>,
    /// `(role, pin)` witness for the placed signals.
    pins: Vec<(&'static str, PinId)>,
    /// `(pool, channel)` witness for the placed DMA channels.
    dma: Vec<(&'static str, u8)>,
}

/// Commit instance `(class, i)` with `dma` channels into `led`: claim the `Inst`
/// token, one free `Pin` token per required role, and `dma` lowest-free `Pool`
/// tokens. Returns the [`Commit`] (whose `tokens` are released on backtrack), or
/// `None` if any required pin / channel can't be supplied.
fn commit(led: &mut Ledger, desc: &McuDescriptor, class: Class, i: u8, dma: u8) -> Option<Commit> {
    let inst = Res::Inst(class, i);
    if led.held().contains(&inst) {
        return None;
    }
    let mut tokens = vec![inst];
    // Pins: one free, distinct pad per required role.
    let mut pins = Vec::new();
    if let Some(pname) = static_inst_name(desc, class, i) {
        for role in required_roles(class) {
            let pin = pins_for(desc.raw, SignalId { peripheral: pname, role })
                .into_iter()
                .find(|p| !led.held().contains(&Res::Pin(*p)) && !tokens.contains(&Res::Pin(*p)));
            match pin {
                Some(p) => {
                    tokens.push(Res::Pin(p));
                    pins.push((role, p));
                }
                None => return None, // a required role has no free pad
            }
        }
    }
    // DMA: `dma` lowest-free channels from the instance's (interchangeable) pools.
    let mut dma_w = Vec::new();
    let mut taken = 0u8;
    'pools: for (pool, cap) in allowed_pools(desc, class, i) {
        for ch in 0..cap {
            if taken == dma {
                break 'pools;
            }
            let tok = Res::Pool(pool, ch);
            if !led.held().contains(&tok) && !tokens.contains(&tok) {
                tokens.push(tok);
                dma_w.push((pool, ch));
                taken += 1;
            }
        }
    }
    if taken < dma {
        return None;
    }
    led.claim(&tokens).ok()?;
    Some(Commit { tokens, pins, dma: dma_w })
}

/// A requirement reduced to the instances it may use on this chip.
struct Item {
    id: u32,
    class: Class,
    dma: u8,
    /// The pinned instance (if present on the chip), else every instance of the
    /// class.
    insts: Vec<u8>,
}

/// Place every item by backtracking, most-constrained (fewest currently-viable
/// instances) first. Returns true with `witness` filled, or false (and `witness`
/// restored) if no complete assignment exists. Bounded by problem size — each
/// level places exactly one more item — so it terminates without a timeout.
fn backtrack(
    desc: &McuDescriptor,
    items: &[Item],
    led: &mut Ledger,
    witness: &mut Vec<Placed>,
) -> bool {
    let placed: std::collections::BTreeSet<u32> = witness.iter().map(|p| p.req_id).collect();
    let next = items
        .iter()
        .filter(|it| !placed.contains(&it.id))
        .min_by_key(|it| it.insts.iter().filter(|&&i| can_place(led, desc, it.class, i, it.dma)).count());
    let Some(it) = next else {
        return true; // every item placed
    };
    for &i in &it.insts {
        if let Some(c) = commit(led, desc, it.class, i, it.dma) {
            witness.push(Placed {
                req_id: it.id,
                class: it.class,
                instance: i,
                pins: c.pins,
                dma: c.dma,
            });
            if backtrack(desc, items, led, witness) {
                return true;
            }
            witness.pop();
            led.release(&c.tokens);
        }
    }
    false
}

/// Best-effort culprit naming when the set is infeasible: a single greedy pass,
/// reporting the requirements it couldn't place. Not the minimal unsat core
/// (that's a later increment) — the COMPLETE verdict comes from `backtrack`.
fn greedy_unmet(desc: &McuDescriptor, items: &[Item]) -> Vec<u32> {
    let mut led = Ledger::new();
    let mut unmet = Vec::new();
    for it in items {
        if !it.insts.iter().any(|&i| commit(&mut led, desc, it.class, i, it.dma).is_some()) {
            unmet.push(it.id);
        }
    }
    unmet
}

/// Decide whether `reqs` fit on `desc`, complete over instance assignment via
/// bounded backtracking (see module docs). The witness is a real allocation.
pub fn feasible(desc: &McuDescriptor, reqs: &[Requirement]) -> Outcome {
    let items: Vec<Item> = reqs
        .iter()
        .map(|r| {
            let ReqKind::UsePeripheral { class, dma, pinned } = r.kind;
            let mut insts = instances_of(desc, class);
            if let Some(p) = pinned {
                insts.retain(|&i| i == p);
            }
            Item { id: r.id, class, dma, insts }
        })
        .collect();

    let mut led = Ledger::new();
    let mut witness = Vec::new();
    if backtrack(desc, &items, &mut led, &mut witness) {
        witness.sort_by_key(|p| p.req_id);
        Outcome::Allocated(witness)
    } else {
        Outcome::Infeasible { unmet: greedy_unmet(desc, &items) }
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
    fn c5_dma_resolves_now_that_chip_json_is_wired_in() {
        // C531's DMA pools are now sourced from the chip JSON (metapac drops C5
        // DMA; `tools/extract.rs` injects it via `tools/json_dma.rs`). LPDMA1(4) +
        // LPDMA2(4) = 8 channels, so a DMA-backed USART use ALLOCATES — where it
        // was previously infeasible-for-lack-of-data.
        let c531 = Package::C531R.descriptor();
        assert_eq!(c531.dma_channel_total(), 8, "C531 = LPDMA1(4) + LPDMA2(4)");
        if instances_of(c531, "USART").is_empty() {
            return; // no USART on this package — nothing to assert
        }
        match feasible(c531, &[req(0, "USART", 2)]) {
            Outcome::Allocated(p) => {
                assert_eq!(p[0].dma.len(), 2, "USART RX+TX claims 2 channels");
                assert!(
                    p[0].dma.iter().all(|(pool, _)| pool.starts_with("LPDMA")),
                    "C5 DMA channels come from the LPDMA controllers",
                );
            }
            o => panic!("expected C531 DMA to allocate now the data is wired in, got {o:?}"),
        }
    }

    #[test]
    fn backtracking_resolves_pinned_contention() {
        let d = g474();
        let insts = instances_of(d, "USART");
        assert!(insts.len() >= 2, "need >=2 USART instances to contend");
        let pin = insts[0];
        // Order chosen to trip a naive in-order greedy (Inc 4): the FLEXIBLE use
        // comes first and would grab `pin`, stranding the use pinned to `pin`.
        // The complete solver places the pinned (most-constrained) use first and
        // yields `pin` to it, putting the flexible one elsewhere.
        let reqs = vec![
            Requirement::use_peripheral(0, "USART", 0),
            Requirement::use_peripheral_pinned(1, "USART", 0, pin),
        ];
        match feasible(d, &reqs) {
            Outcome::Allocated(p) => {
                let pinned = p.iter().find(|x| x.req_id == 1).unwrap();
                let flex = p.iter().find(|x| x.req_id == 0).unwrap();
                assert_eq!(pinned.instance, pin, "pinned use takes its specific instance");
                assert_ne!(flex.instance, pin, "flexible use must yield `pin`");
            }
            o => panic!("backtracking should satisfy pinned contention, got {o:?}"),
        }
    }

    #[test]
    fn two_uses_pinned_to_same_instance_conflict() {
        // Both pinned to the same instance — only one can have it. Exercises the
        // backtrack-undo path (place one, dead-end on the other, release, fail).
        let d = g474();
        let pin = instances_of(d, "USART")[0];
        let reqs = vec![
            Requirement::use_peripheral_pinned(0, "USART", 0, pin),
            Requirement::use_peripheral_pinned(1, "USART", 0, pin),
        ];
        assert!(matches!(feasible(d, &reqs), Outcome::Infeasible { .. }));
    }

    #[test]
    fn pin_to_nonexistent_instance_is_infeasible_and_named() {
        let d = g474();
        let bogus = 99;
        assert!(!instances_of(d, "USART").contains(&bogus));
        assert!(matches!(
            feasible(d, &[Requirement::use_peripheral_pinned(0, "USART", 0, bogus)]),
            Outcome::Infeasible { unmet } if unmet == vec![0],
        ));
    }

    #[test]
    fn placed_uses_claim_distinct_pins_per_required_role() {
        let d = g474();
        // Two SPI uses: each places SCK/MOSI/MISO (3 pads). All 6 must be distinct
        // — the pin-contention invariant (no two peripherals on the same pad).
        let reqs = vec![req(0, "SPI", 0), req(1, "SPI", 0)];
        match feasible(d, &reqs) {
            Outcome::Allocated(p) => {
                assert!(p.iter().all(|x| x.pins.len() == 3), "SPI claims SCK/MOSI/MISO");
                let all: Vec<_> = p.iter().flat_map(|x| x.pins.iter().map(|(_, pin)| *pin)).collect();
                let uniq: std::collections::BTreeSet<_> = all.iter().copied().collect();
                assert_eq!(all.len(), 6);
                assert_eq!(uniq.len(), 6, "no two placed uses may share a pad");
            }
            o => panic!("expected SPI allocation with distinct pins, got {o:?}"),
        }
    }
}
