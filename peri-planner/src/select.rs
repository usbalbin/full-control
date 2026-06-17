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
use crate::mcu_pinout::{pins_for, PinId, SignalId};

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
        "LPUART" => d.comms.lpuart.clone(),
        "SPI" => d.comms.spi.clone(),
        "I2C" => d.comms.i2c.clone(),
        "FDCAN" => d.comms.fdcan.clone(),
        _ => Vec::new(),
    }
}

/// The GPIO signals a class needs routed to pins (minimal functional config).
/// Classes not listed claim no pins (instance-only allocation).
fn required_signals(class: &str) -> &'static [&'static str] {
    match class {
        "USART" | "UART" | "LPUART" => &["TX", "RX"],
        "SPI" => &["SCK", "MOSI", "MISO"],
        "I2C" => &["SCL", "SDA"],
        _ => &[],
    }
}

/// The `&'static` metapac name of an instance, if present on the chip.
fn peri_name(d: &McuDescriptor, class: &str, n: u8) -> Option<&'static str> {
    let want = format!("{class}{n}");
    d.raw.peripherals.iter().find(|p| p.name == want).map(|p| p.name)
}

/// Every way to place `roles` on distinct pins of `peri`. Empty if any role has
/// no pin on this package (peripheral unusable here). Pin option counts are
/// small, so this cross-product stays bounded — unlike the DMA-channel case,
/// which is why pins bundle into the instance candidate while channels don't.
fn pin_combos(d: &McuDescriptor, peri: &'static str, roles: &[&'static str]) -> Vec<Vec<PinId>> {
    let mut combos: Vec<Vec<PinId>> = vec![Vec::new()];
    for &role in roles {
        let opts = pins_for(d.raw, SignalId { peripheral: peri, role });
        if opts.is_empty() {
            return Vec::new();
        }
        let mut next = Vec::new();
        for combo in &combos {
            for &p in &opts {
                if !combo.contains(&p) {
                    let mut c = combo.clone();
                    c.push(p);
                    next.push(c);
                }
            }
        }
        combos = next;
    }
    combos
}

/// Candidate allocations for one instance: one per pin placement, each claiming
/// the instance token plus a pin for every required GPIO signal. Pins are
/// instance-coupled (USART1.TX options differ from USART2.TX) — the non-symmetric
/// dimension where backtracking earns its keep — so they bundle here, while
/// fungible DMA channels stay decoupled.
fn instance_candidates(d: &McuDescriptor, class: &'static str, n: u8) -> Vec<Candidate> {
    let Some(peri) = peri_name(d, class, n) else {
        return Vec::new();
    };
    let inst = Res::Inst(class, n);
    pin_combos(d, peri, required_signals(class))
        .into_iter()
        .map(|pins| {
            let mut tokens = Vec::with_capacity(1 + pins.len());
            tokens.push(inst);
            tokens.extend(pins.into_iter().map(Res::Pin));
            Candidate::new(peri, tokens)
        })
        .collect()
}

/// The aggregate DMA-channel capacity reachable by `demands`: the channel count
/// of the union of every controller pool any demanded (class, signal) leg can
/// use. Channels are fungible, so DMA feasibility is a COUNT (demand <=
/// capacity), NOT a CSP — putting symmetric channels in the backtracker makes
/// infeasibility proofs blow up permuting equivalent channels. Exact for the
/// in-scope DMAMUX / named families (every leg of a class reaches all
/// controllers); disjoint-pool families would need per-pool matching (deferred).
fn channel_capacity(d: &McuDescriptor, demands: &[Demand]) -> usize {
    let mut pools: BTreeSet<&'static str> = BTreeSet::new();
    for dem in demands {
        for &sig in dem.dma.signals() {
            for n in instances_of(d, dem.class) {
                let Some(peri) = peri_name(d, dem.class, n) else {
                    continue;
                };
                if let Some(leg) = d
                    .dma_routes(peri)
                    .iter()
                    .find(|l| l.signal.eq_ignore_ascii_case(sig))
                {
                    pools.extend(leg.pools.iter().copied());
                }
            }
        }
    }
    pools
        .iter()
        .filter_map(|p| d.dma_pools().iter().find(|s| s.name == *p))
        .map(|s| s.channels as usize)
        .sum()
}

/// Total DMA channels demanded: one channel per instance per DMA signal.
fn channel_demand(demands: &[Demand]) -> usize {
    demands
        .iter()
        .map(|dem| dem.count as usize * dem.dma.signals().len())
        .sum()
}

/// Sentinel ids for a DMA-channel-capacity shortfall, kept above the CSP id
/// range so a channel deficit surfaces as `unmet` (infeasible) distinctly from
/// an instance/pin conflict.
const CHANNEL_UNMET_BASE: u32 = u32::MAX - 1024;

/// Lower `demands` into a feasibility check against `d`.
///
/// Instances and their GPIO pins go through the backtracking kernel — the
/// non-symmetric dimension (distinct instances, contended pins, where greedy can
/// wrongly drop a capable part). DMA channels are a separate aggregate capacity
/// count (fungible -> counting, not CSP). Feasible iff every instance+pin
/// requirement is satisfiable AND total channel demand fits reachable capacity.
pub fn solve(d: &McuDescriptor, demands: &[Demand]) -> Solution {
    let mut reqs: Vec<Requirement> = Vec::new();
    let mut id = 0u32;
    for dem in demands {
        let inst_cands: Vec<Candidate> = instances_of(d, dem.class)
            .into_iter()
            .flat_map(|n| instance_candidates(d, dem.class, n))
            .collect();
        for _ in 0..dem.count {
            id += 1;
            reqs.push(Requirement { id, candidates: inst_cands.clone() });
        }
    }
    let mut sol = assign_backtracking(&reqs);

    // DMA channel capacity is a pigeonhole count — definitive (never
    // indeterminate). A shortfall makes the design infeasible.
    let demand = channel_demand(demands);
    if demand > 0 {
        let shortfall = demand.saturating_sub(channel_capacity(d, demands));
        for k in 0..shortfall.min(1024) {
            sol.unmet.push(CHANNEL_UNMET_BASE + k as u32);
        }
    }
    sol
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
        // 3 instance+pin requirements (channels are a capacity count, not in CSP).
        assert_eq!(s.assigned.len(), 3);
    }

    #[test]
    fn mixed_classes_within_dma_budget_feasible() {
        // 3 USART + 2 SPI, each RX+TX: 10 DMA channels <= G474's 16; instances
        // exist; TX/RX and SCK/MOSI/MISO pins resolve. Exercises generalization
        // beyond USART plus pin placement.
        let d = Package::G474R.descriptor();
        let s = solve(
            d,
            &[
                Demand { class: "USART", count: 3, dma: Dma::RxTx },
                Demand { class: "SPI", count: 2, dma: Dma::RxTx },
            ],
        );
        assert!(s.is_feasible() && !s.indeterminate, "unmet={:?}", s.unmet);
    }

    #[test]
    fn dma_channel_exhaustion_across_classes_is_infeasible() {
        // 3 USART + 3 SPI + 3 I2C each RX+TX = 18 DMA channels > G474's 16. All
        // instances exist; the shared channel pool is the binding constraint.
        // Proven by the capacity count (definitive, fast — not a backtracking
        // blow-up over symmetric channels).
        let d = Package::G474R.descriptor();
        let s = solve(
            d,
            &[
                Demand { class: "USART", count: 3, dma: Dma::RxTx },
                Demand { class: "SPI", count: 3, dma: Dma::RxTx },
                Demand { class: "I2C", count: 3, dma: Dma::RxTx },
            ],
        );
        assert!(!s.is_feasible(), "18 RxTx channels should exceed 16");
        assert!(!s.indeterminate, "channel exhaustion is a definitive count, not budget-cut");
    }

    #[test]
    fn spi_pins_resolve_without_dma() {
        // SPI needs SCK/MOSI/MISO pins placed; an instance-only demand must
        // resolve all three on distinct pins.
        let d = Package::G474R.descriptor();
        let s = solve(d, &[Demand { class: "SPI", count: 2, dma: Dma::None }]);
        assert!(s.is_feasible(), "unmet={:?}", s.unmet);
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
