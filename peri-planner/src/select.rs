//! Descriptor → constraint-kernel adapter: the DMA-first slice of the MCU
//! selector. Turns high-level demands ("N peripherals of a class, each with some
//! DMA") into [`Requirement`]s whose candidates are generated from the chip's
//! real DMA routes (`McuDescriptor::dma_routes` / `dma_pools`), then solves with
//! the generic kernel. This is where the kernel meets the data.
//!
//! Scope (this increment): USART instance + DMA-channel allocation, proven on
//! G474/H523 (which carry full DMA metadata). Other classes and pin contention
//! arrive in later increments. The candidate enumeration is intentionally simple
//! (one candidate per (instance, rx-channel, tx-channel) choice) — fine for
//! greedy first-fit; a leaner representation lands with backtracking.

use std::collections::BTreeSet;

use crate::constraint::{assign_backtracking, Candidate, Requirement, Res, Solution};
use crate::mcu::McuDescriptor;

/// What DMA a peripheral instance needs.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Dma {
    None,
    Rx,
    Tx,
    RxTx,
}

impl Dma {
    /// The DMA signal names this need claims a channel for.
    fn signals(self) -> &'static [&'static str] {
        match self {
            Dma::None => &[],
            Dma::Rx => &["RX"],
            Dma::Tx => &["TX"],
            Dma::RxTx => &["RX", "TX"],
        }
    }
}

/// A high-level demand: `count` instances of `class`, each needing `dma`.
#[derive(Clone, Debug)]
pub struct Demand {
    /// metapac class prefix, e.g. "USART".
    pub class: &'static str,
    pub count: u8,
    pub dma: Dma,
}

/// The instance numbers of `class` present on the chip.
fn instances_of(d: &McuDescriptor, class: &str) -> Vec<u8> {
    match class {
        "USART" => d.comms.usart.clone(),
        "UART" => d.comms.uart.clone(),
        "SPI" => d.comms.spi.clone(),
        "I2C" => d.comms.i2c.clone(),
        "FDCAN" => d.comms.fdcan.clone(),
        _ => Vec::new(),
    }
}

/// Every DMA channel token reachable by a leg's allowed controller pools.
fn channels_for(pools: &[&'static str], d: &McuDescriptor) -> Vec<Res> {
    let supply = d.dma_pools();
    let mut out = Vec::new();
    for &p in pools {
        if let Some(def) = supply.iter().find(|s| s.name == p) {
            for idx in 0..def.channels {
                out.push(Res::Pool(p, idx));
            }
        }
    }
    out
}

/// The DMA channel tokens a `class` can use for `signal`, unioned across every
/// instance of the class. On the in-scope DMAMUX / named-controller families all
/// instances of a class share the same controller fan-out, so this is the sound
/// channel pool to draw from — and decoupling the channel choice from the
/// instance choice (rather than enumerating the instance×rx×tx cross-product)
/// keeps each requirement small so backtracking stays tractable.
fn class_channels(d: &McuDescriptor, class: &str, signal: &str) -> Vec<Res> {
    let mut pools: BTreeSet<&'static str> = BTreeSet::new();
    for n in instances_of(d, class) {
        let peri = format!("{class}{n}");
        if let Some(leg) = d
            .dma_routes(&peri)
            .iter()
            .find(|l| l.signal.eq_ignore_ascii_case(signal))
        {
            pools.extend(leg.pools.iter().copied());
        }
    }
    channels_for(&pools.into_iter().collect::<Vec<_>>(), d)
}

/// Lower `demands` into requirements against `d` and solve.
///
/// Each demand of `count` instances becomes: `count` instance-requirements
/// (candidates = every instance of the class, so the kernel picks distinct ones
/// via `Inst` exclusivity), plus `count` channel-requirements per needed DMA
/// signal (candidates = every channel in the class's pool). Channel requirements
/// are independent of instance choice — sound on the in-scope families where a
/// class's DMA fan-out is instance-uniform — which avoids the cross-product
/// blow-up and keeps every requirement's candidate list small.
///
/// Feasible iff every requirement (instance and channel) gets a non-conflicting
/// allocation.
pub fn solve(d: &McuDescriptor, demands: &[Demand]) -> Solution {
    let mut reqs: Vec<Requirement> = Vec::new();
    let mut id = 0u32;
    let mut push = |cands: Vec<Candidate>, count: u8, reqs: &mut Vec<Requirement>| {
        for _ in 0..count {
            id += 1;
            reqs.push(Requirement { id, candidates: cands.clone() });
        }
    };
    for dem in demands {
        // Instance requirements.
        let inst_cands: Vec<Candidate> = instances_of(d, dem.class)
            .into_iter()
            .map(|n| Candidate::new(format!("{}{}", dem.class, n), vec![Res::Inst(dem.class, n)]))
            .collect();
        push(inst_cands, dem.count, &mut reqs);

        // Channel requirements, one set per needed DMA signal.
        for &sig in dem.dma.signals() {
            let chan_cands: Vec<Candidate> = class_channels(d, dem.class, sig)
                .into_iter()
                .map(|c| Candidate::new(format!("{:?}", c), vec![c]))
                .collect();
            push(chan_cands, dem.count, &mut reqs);
        }
    }
    // The selector path uses backtracking: greedy first-fit can wrongly report a
    // capable part infeasible, which for a selector silently drops a good MCU.
    assign_backtracking(&reqs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mcu::Package;

    #[test]
    fn usart_rxtx_allocates_on_g474() {
        // 3 USARTs each with RX+TX DMA: G474 has 3 USART instances and 16 DMA
        // channels (6 needed), so all three get a non-conflicting allocation.
        let d = Package::G474R.descriptor();
        let s = solve(d, &[Demand { class: "USART", count: 3, dma: Dma::RxTx }]);
        assert!(s.is_feasible(), "expected feasible, unmet={:?}", s.unmet);
        assert!(!s.indeterminate);
        // 3 instance reqs + 3 RX-channel + 3 TX-channel = 9 sub-requirements.
        assert_eq!(s.assigned.len(), 9);
    }

    #[test]
    fn usart_demand_exceeding_instances_is_infeasible() {
        // G474 has only 3 USARTs — a 4th instance can't be allocated.
        let d = Package::G474R.descriptor();
        let s = solve(d, &[Demand { class: "USART", count: 4, dma: Dma::RxTx }]);
        assert!(!s.is_feasible());
        assert_eq!(s.unmet.len(), 1);
    }

    #[test]
    fn instance_only_demand_works_without_dma() {
        // C531 has >=2 USARTs; with no DMA required, the instance allocation is
        // data-driven and succeeds regardless of the DMA-metadata gap.
        let d = Package::C531R.descriptor();
        let s = solve(d, &[Demand { class: "USART", count: 2, dma: Dma::None }]);
        assert!(s.is_feasible(), "unmet={:?}", s.unmet);
    }

    #[test]
    fn rxtx_demand_infeasible_when_dma_metadata_absent() {
        // C531's DMA is empty in the compiled metapac, so a RX+TX demand finds no
        // channel candidates -> infeasible. Documents the data-currency gap
        // (resolves on a metapac refresh); the catalog already knows C531 has 8.
        let d = Package::C531R.descriptor();
        let s = solve(d, &[Demand { class: "USART", count: 1, dma: Dma::RxTx }]);
        assert!(!s.is_feasible());
    }
}
