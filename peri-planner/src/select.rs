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

use std::collections::{BTreeMap, BTreeSet};

use crate::catalog::{self, CatalogEntry, SearchQuery};
use crate::constraint::{assign_backtracking, Candidate, Requirement, Res, Solution};
use crate::mcu::{McuDescriptor, Package};
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

// ---------- Two-tier catalog evaluation ----------

/// A two-tier verdict for a catalog part against a demand set.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Tier-2 backtracking solve on a real descriptor found a feasible
    /// allocation — the part provably satisfies the demands.
    Verified,
    /// Tier-2 proved no allocation exists — the part cannot satisfy them.
    Infeasible,
    /// Tier-1 passed but Tier-2 could not verify: no compiled descriptor for the
    /// part, the search was budget-cut, OR the descriptor is missing data the
    /// part actually has (e.g. C5 DMA absent from metapac while the catalog has
    /// it). NOT a pass and NOT a rejection — "plausible, unverified".
    BoundsOnly,
}

/// A per-class demand row for the UI: count + whether it needs RX/TX DMA.
#[derive(Clone, Debug)]
pub struct DemandInput {
    pub class: &'static str,
    pub count: u8,
    pub with_dma: bool,
}

impl DemandInput {
    pub fn new(class: &'static str) -> Self {
        Self { class, count: 0, with_dma: false }
    }
    pub fn to_demand(&self) -> Demand {
        Demand {
            class: self.class,
            count: self.count,
            dma: if self.with_dma { Dma::RxTx } else { Dma::None },
        }
    }
}

/// Total DMA channels a demand set needs — the Tier-1 capacity lower bound.
pub fn total_channel_demand(demands: &[Demand]) -> usize {
    channel_demand(demands)
}

/// Per-class instance count on a catalog entry — for the cheap, exact
/// instance-capacity check.
fn entry_class_count(e: &CatalogEntry, class: &str) -> u8 {
    match class {
        "USART" => e.usart,
        "UART" => e.uart,
        "LPUART" => e.lpuart,
        "SPI" => e.spi,
        "I2C" => e.i2c,
        "FDCAN" => e.fdcan,
        _ => 0,
    }
}

/// Total distinct GPIO pins a demand set needs — each peripheral signal needs
/// its own pin, so this is a Tier-1 pin-capacity necessary bound. A part with
/// fewer AF-capable GPIO pins than this can't host the set, whatever the AF mux
/// (precise mux contention is the Tier-2 check). Classes without modeled GPIO
/// signals (FDCAN) contribute 0 — keeping it a sound lower bound.
pub fn pin_demand(demands: &[Demand]) -> usize {
    demands
        .iter()
        .map(|dem| dem.count as usize * required_signals(dem.class).len())
        .sum()
}

/// Two-tier evaluation of the catalog against `demands`, narrowed by `base`.
///
/// Tier-1: the base query AND a derived DMA-capacity necessary bound prune the
/// ~1600 parts with cheap integer compares (sound — never drops a feasible
/// part). Tier-2: for each survivor with a bridge-reachable descriptor, run the
/// backtracking solve. Parts whose descriptor can't verify — none compiled, the
/// search budget-cut, or descriptor data missing that the catalog has — are
/// honestly labeled `BoundsOnly` rather than rejected.
pub fn evaluate(base: &SearchQuery, demands: &[Demand]) -> Vec<(&'static CatalogEntry, Verdict)> {
    let dma_demand = channel_demand(demands);
    let needs_dma = dma_demand > 0;

    // Aggregate per-class instance demand — a cheap, definitive count check that
    // also keeps the backtracker from blowing up trying to prove instance
    // exhaustion (it would permute pin combinations of the placeable instances).
    let mut class_demand: BTreeMap<&str, u8> = BTreeMap::new();
    for d in demands {
        *class_demand.entry(d.class).or_default() += d.count;
    }

    // Tier-1: fold the DMA-capacity and pin-capacity necessary bounds into the
    // query (both sound — never drop a part that could actually satisfy them).
    let mut q = base.clone();
    q.min_dma_channels = q.min_dma_channels.max(dma_demand as u16);
    q.min_gpio_pins = q.min_gpio_pins.max(pin_demand(demands) as u16);

    catalog::search(&q)
        .into_iter()
        .map(|e| {
            // Definitive instance-count check first (exact from the catalog).
            if class_demand.iter().any(|(&cls, &n)| entry_class_count(e, cls) < n) {
                return (e, Verdict::Infeasible);
            }
            let verdict = match Package::for_chip_name(&e.name) {
                None => Verdict::BoundsOnly, // no descriptor to verify against
                Some(pkg) => {
                    let d = pkg.descriptor();
                    if needs_dma && d.dma_channel_total() == 0 && e.dma_pool_total > 0 {
                        // Descriptor lacks DMA data the catalog knows the part has
                        // (metapac C5 gap) — can't verify, must not reject.
                        Verdict::BoundsOnly
                    } else {
                        let sol = solve(d, demands);
                        if sol.indeterminate {
                            Verdict::BoundsOnly
                        } else if sol.is_feasible() {
                            Verdict::Verified
                        } else {
                            Verdict::Infeasible
                        }
                    }
                }
            };
            (e, verdict)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn evaluate_labels_verified_and_bounds_only() {
        let demands = [Demand { class: "USART", count: 3, dma: Dma::RxTx }];
        let results = evaluate(&SearchQuery::default(), &demands);
        // G474RE is bridge-reachable with full DMA data -> provably Verified.
        let g4 = results.iter().find(|(e, _)| e.name == "STM32G474RE").expect("G474RE");
        assert_eq!(g4.1, Verdict::Verified);
        // Some surviving part has no compiled descriptor -> BoundsOnly, not dropped.
        assert!(results.iter().any(|(_, v)| *v == Verdict::BoundsOnly));
    }

    #[test]
    fn evaluate_c531_dma_is_bounds_only_not_infeasible() {
        // C531 is bridge-reachable and Tier-1 says it has 8 DMA channels, but its
        // descriptor's DMA is empty (metapac C5 gap). The honest verdict is
        // BoundsOnly — NOT a false Infeasible that would drop a capable part.
        let demands = [Demand { class: "USART", count: 1, dma: Dma::RxTx }];
        let results = evaluate(&SearchQuery::default(), &demands);
        let c531: Vec<_> = results.iter().filter(|(e, _)| e.name.starts_with("STM32C531R")).collect();
        assert!(!c531.is_empty(), "C531R should survive Tier-1 (8 DMA channels)");
        assert!(
            c531.iter().all(|(_, v)| *v == Verdict::BoundsOnly),
            "C531 DMA must be BoundsOnly, got {:?}",
            c531.iter().map(|(e, v)| (&e.name, v)).collect::<Vec<_>>(),
        );
    }

    #[test]
    fn pin_capacity_bound_prunes_pin_starved_parts() {
        // 4 SPI + 4 USART (no DMA) = 4*3 + 4*2 = 20 distinct pins. Small packages
        // (< 20 AF-capable GPIO) are pruned at Tier-1 even if they list the
        // peripherals — the "not enough pins to use them together" case.
        let demands = [
            Demand { class: "SPI", count: 4, dma: Dma::None },
            Demand { class: "USART", count: 4, dma: Dma::None },
        ];
        assert_eq!(pin_demand(&demands), 20);
        let results = evaluate(&SearchQuery::default(), &demands);
        assert!(
            results.iter().all(|(e, _)| e.gpio_pins >= 20),
            "every survivor must have >= 20 GPIO pins",
        );
        // The bound is real: parts below it exist in the catalog and are excluded.
        assert!(catalog::CATALOG.iter().any(|e| e.gpio_pins < 20));
    }

    #[test]
    fn evaluate_dma_capacity_bound_prunes_at_tier1() {
        // 18 RX+TX channels: G474 (16) is pruned by the Tier-1 capacity bound;
        // every survivor has >= 18 channels.
        let demands = [
            Demand { class: "USART", count: 3, dma: Dma::RxTx },
            Demand { class: "SPI", count: 3, dma: Dma::RxTx },
            Demand { class: "I2C", count: 3, dma: Dma::RxTx },
        ];
        let results = evaluate(&SearchQuery::default(), &demands);
        assert!(results.iter().all(|(e, _)| e.dma_pool_total >= 18));
        assert!(!results.iter().any(|(e, _)| e.name == "STM32G474RE"), "G474 (16ch) pruned");
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
