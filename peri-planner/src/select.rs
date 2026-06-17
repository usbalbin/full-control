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

use crate::constraint::{assign_greedy, Candidate, Requirement, Res, Solution};
use crate::mcu::McuDescriptor;

/// What DMA a peripheral instance needs.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Dma {
    None,
    Rx,
    Tx,
    RxTx,
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

/// The channel options for one named signal of an instance (empty if no leg).
fn signal_channels(d: &McuDescriptor, peri: &str, signal: &str) -> Vec<Res> {
    d.dma_routes(peri)
        .iter()
        .find(|l| l.signal.eq_ignore_ascii_case(signal))
        .map(|l| channels_for(l.pools, d))
        .unwrap_or_default()
}

/// Candidate allocations for one specific instance under a DMA need. Each pairs
/// the instance token with a concrete channel choice (or pair, for RxTx).
fn instance_candidates(d: &McuDescriptor, class: &'static str, n: u8, dma: Dma) -> Vec<Candidate> {
    let inst = Res::Inst(class, n);
    let peri = format!("{class}{n}");
    match dma {
        Dma::None => vec![Candidate::new(peri, vec![inst])],
        Dma::Rx | Dma::Tx => {
            let sig = if dma == Dma::Rx { "RX" } else { "TX" };
            signal_channels(d, &peri, sig)
                .into_iter()
                .map(|ch| Candidate::new(format!("{peri}.{sig}"), vec![inst, ch]))
                .collect()
        }
        Dma::RxTx => {
            let rx = signal_channels(d, &peri, "RX");
            let tx = signal_channels(d, &peri, "TX");
            let mut cands = Vec::new();
            for &r in &rx {
                for &t in &tx {
                    if r != t {
                        cands.push(Candidate::new(
                            format!("{peri}.RX/TX"),
                            vec![inst, r, t],
                        ));
                    }
                }
            }
            cands
        }
    }
}

/// Lower `demands` into requirements against `d` and solve. Each demanded
/// instance becomes one requirement whose candidates span every free instance of
/// the class (so the kernel picks distinct instances via `Inst` exclusivity) and
/// every channel choice. Feasible iff every demanded instance got a non-
/// conflicting allocation.
pub fn solve(d: &McuDescriptor, demands: &[Demand]) -> Solution {
    let mut reqs: Vec<Requirement> = Vec::new();
    let mut id = 0u32;
    for dem in demands {
        let mut all: Vec<Candidate> = Vec::new();
        for n in instances_of(d, dem.class) {
            all.extend(instance_candidates(d, dem.class, n, dem.dma));
        }
        for _ in 0..dem.count {
            id += 1;
            reqs.push(Requirement { id, candidates: all.clone() });
        }
    }
    assign_greedy(&reqs)
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
        assert_eq!(s.assigned.len(), 3);
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
