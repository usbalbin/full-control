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
use crate::mcu::McuDescriptor;
use crate::mcu_pinout::{pins_for, PinId, SignalId};

/// A high-level demand: `count` peripherals of a logical `kind`, optionally each
/// with DMA. Kinds: "SERIAL" (UART / USART / LPUART — USART is a superset of
/// UART, so a UART demand is met by a USART), "SPI", "I2C", "ADC".
#[derive(Clone, Debug, PartialEq)]
pub struct Demand {
    pub kind: &'static str,
    pub count: u8,
    pub with_dma: bool,
    /// Enabled optional signal groups (keys from `kind_options`), e.g.
    /// "chip-select", "dead-battery" — add their pins to the contention.
    pub options: Vec<&'static str>,
}

/// The metapac peripheral classes a logical kind can be satisfied by.
fn underlying_classes(kind: &str) -> &'static [&'static str] {
    match kind {
        "SERIAL" => &["USART", "UART", "LPUART"],
        "SPI" => &["SPI"],
        "I2C" => &["I2C"],
        "ADC" => &["ADC"],
        "UCPD" => &["UCPD"],
        _ => &[],
    }
}

/// The always-required GPIO signals for a kind (the minimal functional config).
/// ADC inputs are analog (not AF-pin-contended), so ADC claims no pins.
fn base_signals(kind: &str) -> &'static [&'static str] {
    match kind {
        "SERIAL" => &["TX", "RX"],
        "SPI" => &["SCK", "MOSI", "MISO"], // full-duplex
        "I2C" => &["SCL", "SDA"],
        "UCPD" => &["CC1", "CC2"],
        _ => &[],
    }
}

/// Optional signal groups a kind can be configured with: `(option key, the extra
/// pins it requires)`. Lets the user model real usage — SPI chip-select, serial
/// flow control, USB-PD dead-battery — so pin contention reflects what's used.
fn kind_options(kind: &str) -> &'static [(&'static str, &'static [&'static str])] {
    match kind {
        "SERIAL" => &[("flow control", &["CTS", "RTS"]), ("sync clock", &["CK"])],
        "SPI" => &[("chip-select", &["NSS"])],
        "I2C" => &[("SMBus alert", &["SMBA"])],
        "UCPD" => &[
            ("dead-battery", &["DBCC1", "DBCC2"]),
            ("fast role swap", &["FRSTX1", "FRSTX2"]),
        ],
        _ => &[],
    }
}

/// GPIO signals a kind needs given the enabled options: base + each enabled
/// option's extra signals.
fn required_signals(kind: &str, options: &[&str]) -> Vec<&'static str> {
    let mut v: Vec<&'static str> = base_signals(kind).to_vec();
    for (key, extra) in kind_options(kind) {
        if options.contains(key) {
            v.extend_from_slice(extra);
        }
    }
    v
}

/// DMA channels one instance of the kind consumes when DMA is requested:
/// RX+TX = 2 for serial/SPI/I2C/UCPD, a single stream = 1 for ADC.
fn dma_channels_per_instance(kind: &str) -> usize {
    match kind {
        "SERIAL" | "SPI" | "I2C" | "UCPD" => 2,
        "ADC" => 1,
        _ => 0,
    }
}

/// Instance numbers of a metapac peripheral class present on the chip.
fn class_instances(d: &McuDescriptor, uclass: &str) -> Vec<u8> {
    match uclass {
        "USART" => d.comms.usart.clone(),
        "UART" => d.comms.uart.clone(),
        "LPUART" => d.comms.lpuart.clone(),
        "SPI" => d.comms.spi.clone(),
        "I2C" => d.comms.i2c.clone(),
        "FDCAN" => d.comms.fdcan.clone(),
        "UCPD" => d.comms.ucpd.clone(),
        "ADC" => d.adcs.iter().map(|a| a.number).collect(),
        _ => Vec::new(),
    }
}

/// Catalog instance count for a kind — the cheap, exact Tier-1 bound. "SERIAL"
/// counts all async-serial (USART + UART + LPUART).
fn entry_kind_count(e: &CatalogEntry, kind: &str) -> u8 {
    match kind {
        "SERIAL" => e.total_uart(),
        "SPI" => e.spi,
        "I2C" => e.i2c,
        "ADC" => e.adc,
        "UCPD" => e.ucpd,
        _ => 0,
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

/// Candidate allocations for one peripheral instance: one per pin placement,
/// each claiming the instance token plus a pin for every `gpio` signal. Pins are
/// instance-coupled (USART1.TX options differ from USART2.TX) — the non-symmetric
/// dimension where backtracking earns its keep — so they bundle here, while
/// fungible DMA channels stay a capacity count.
fn instance_candidates(
    d: &McuDescriptor,
    uclass: &'static str,
    n: u8,
    gpio: &[&'static str],
) -> Vec<Candidate> {
    let Some(peri) = peri_name(d, uclass, n) else {
        return Vec::new();
    };
    let inst = Res::Inst(uclass, n);
    pin_combos(d, peri, gpio)
        .into_iter()
        .map(|pins| {
            let mut tokens = Vec::with_capacity(1 + pins.len());
            tokens.push(inst);
            tokens.extend(pins.into_iter().map(Res::Pin));
            Candidate::new(peri, tokens)
        })
        .collect()
}

/// All candidates for one requirement of `kind` with `options`: every instance
/// of every underlying class (so a SERIAL demand can take a USART, UART or
/// LPUART), each with its pin placements for the configured signals. The kernel
/// picks distinct ones via `Inst` exclusivity.
fn kind_candidates(d: &McuDescriptor, kind: &str, options: &[&str]) -> Vec<Candidate> {
    let gpio = required_signals(kind, options);
    let mut out = Vec::new();
    for &uc in underlying_classes(kind) {
        for n in class_instances(d, uc) {
            out.extend(instance_candidates(d, uc, n, &gpio));
        }
    }
    out
}

/// Aggregate DMA-channel capacity reachable by `demands`: channels in the union
/// of every controller pool any demanded instance's DMA leg can use. Channels
/// are fungible, so DMA feasibility is a COUNT (demand <= capacity), NOT a CSP —
/// symmetric channels in the backtracker make infeasibility proofs blow up.
/// Exact for the in-scope DMAMUX / named families (a class's legs reach all
/// controllers); disjoint-pool families would need per-pool matching (deferred).
fn channel_capacity(d: &McuDescriptor, demands: &[Demand]) -> usize {
    let mut pools: BTreeSet<&'static str> = BTreeSet::new();
    for dem in demands.iter().filter(|d| d.with_dma) {
        for &uc in underlying_classes(dem.kind) {
            for n in class_instances(d, uc) {
                let Some(peri) = peri_name(d, uc, n) else { continue };
                for leg in d.dma_routes(peri) {
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

/// Total DMA channels demanded across the set (per-kind channels per instance).
fn channel_demand(demands: &[Demand]) -> usize {
    demands
        .iter()
        .filter(|d| d.with_dma)
        .map(|dem| dem.count as usize * dma_channels_per_instance(dem.kind))
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
        let cands = kind_candidates(d, dem.kind, &dem.options);
        for _ in 0..dem.count {
            id += 1;
            reqs.push(Requirement { id, candidates: cands.clone() });
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

/// Caches the Part-finder evaluation so the (whole-lineup) Tier-2 solve runs only
/// when the query or demands actually change — not every egui frame. Holds
/// `Option<Verdict>` (None = plain search, no constraints active).
#[derive(Default)]
pub struct EvalCache {
    key: Option<(SearchQuery, Vec<DemandInput>)>,
    results: Vec<(&'static CatalogEntry, Option<Verdict>)>,
}

impl EvalCache {
    /// Results for `(query, demands)`, recomputing only on change. With no active
    /// demand it's a plain catalog search (no solving); otherwise it's the
    /// two-tier `evaluate`.
    pub fn results(
        &mut self,
        query: &SearchQuery,
        demands: &[DemandInput],
    ) -> &[(&'static CatalogEntry, Option<Verdict>)] {
        let changed = self.key.as_ref().map(|(q, d)| q != query || d != demands).unwrap_or(true);
        if changed {
            let active: Vec<Demand> =
                demands.iter().filter(|d| d.count > 0).map(DemandInput::to_demand).collect();
            self.results = if active.is_empty() {
                catalog::search(query).into_iter().map(|e| (e, None)).collect()
            } else {
                evaluate(query, &active).into_iter().map(|(e, v)| (e, Some(v))).collect()
            };
            self.key = Some((query.clone(), demands.to_vec()));
        }
        &self.results
    }
}

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

/// A per-kind demand row for the UI: count, DMA, and enabled optional signal
/// groups (e.g. SPI chip-select, USB-PD dead-battery).
#[derive(Clone, Debug, PartialEq)]
pub struct DemandInput {
    pub kind: &'static str,
    pub count: u8,
    pub with_dma: bool,
    pub options: Vec<&'static str>,
}

impl DemandInput {
    pub fn new(kind: &'static str) -> Self {
        Self { kind, count: 0, with_dma: false, options: Vec::new() }
    }
    /// Human label for the row.
    pub fn label(&self) -> &'static str {
        match self.kind {
            "SERIAL" => "UART / USART",
            "UCPD" => "USB-PD (UCPD)",
            other => other,
        }
    }
    /// The optional signal-group keys this kind offers (for UI toggles).
    pub fn available_options(&self) -> Vec<&'static str> {
        kind_options(self.kind).iter().map(|(k, _)| *k).collect()
    }
    /// Toggle an optional signal group on/off.
    pub fn set_option(&mut self, key: &'static str, on: bool) {
        let has = self.options.contains(&key);
        if on && !has {
            self.options.push(key);
        } else if !on && has {
            self.options.retain(|k| *k != key);
        }
    }
    pub fn has_option(&self, key: &str) -> bool {
        self.options.iter().any(|k| *k == key)
    }
    pub fn to_demand(&self) -> Demand {
        Demand {
            kind: self.kind,
            count: self.count,
            with_dma: self.with_dma,
            options: self.options.clone(),
        }
    }
}

/// Total DMA channels a demand set needs — the Tier-1 capacity lower bound.
pub fn total_channel_demand(demands: &[Demand]) -> usize {
    channel_demand(demands)
}

/// Total distinct GPIO pins a demand set needs — each peripheral signal needs
/// its own pin, so this is a Tier-1 pin-capacity necessary bound. A part with
/// fewer AF-capable GPIO pins than this can't host the set, whatever the AF mux
/// (precise mux contention is the Tier-2 check). Kinds without modeled GPIO
/// signals (ADC) contribute 0 — keeping it a sound lower bound.
pub fn pin_demand(demands: &[Demand]) -> usize {
    demands
        .iter()
        .map(|dem| dem.count as usize * required_signals(dem.kind, &dem.options).len())
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

    // Aggregate per-kind instance demand — a cheap, definitive count check that
    // also keeps the backtracker from blowing up trying to prove instance
    // exhaustion (it would permute pin combinations of the placeable instances).
    let mut kind_demand: BTreeMap<&str, u8> = BTreeMap::new();
    for d in demands {
        *kind_demand.entry(d.kind).or_default() += d.count;
    }

    // Tier-1: fold the DMA-capacity and pin-capacity necessary bounds into the
    // query (both sound — never drop a part that could actually satisfy them).
    let mut q = base.clone();
    q.min_dma_channels = q.min_dma_channels.max(dma_demand as u16);
    q.min_gpio_pins = q.min_gpio_pins.max(pin_demand(demands) as u16);

    let _ = needs_dma; // (kept for readability of the bound above)
    catalog::search(&q)
        .into_iter()
        .map(|e| {
            // Definitive instance-count check first (exact from the catalog).
            if kind_demand.iter().any(|(&k, &n)| entry_kind_count(e, k) < n) {
                return (e, Verdict::Infeasible);
            }
            // Tier-2: solve against the part's descriptor from the whole-lineup
            // asset (covers every part, and carries C5 DMA that metapac omits).
            let verdict = match crate::desc_asset::descriptor_for(&e.name) {
                None => Verdict::BoundsOnly, // asset somehow lacks this prefix
                Some(d) => {
                    let sol = solve(d, demands);
                    if sol.indeterminate {
                        Verdict::BoundsOnly
                    } else if sol.is_feasible() {
                        Verdict::Verified
                    } else {
                        Verdict::Infeasible
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
    use crate::mcu::Package;

    #[test]
    fn usart_rxtx_allocates_on_g474() {
        // 3 USARTs each with RX+TX DMA: G474 has 3 USART instances and 16 DMA
        // channels (6 needed), so all three get a non-conflicting allocation.
        let d = Package::G474R.descriptor();
        let s = solve(d, &[Demand { kind: "SERIAL", count: 3, with_dma: true, options: vec![] }]);
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
                Demand { kind: "SERIAL", count: 3, with_dma: true, options: vec![] },
                Demand { kind: "SPI", count: 2, with_dma: true, options: vec![] },
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
                Demand { kind: "SERIAL", count: 3, with_dma: true, options: vec![] },
                Demand { kind: "SPI", count: 3, with_dma: true, options: vec![] },
                Demand { kind: "I2C", count: 3, with_dma: true, options: vec![] },
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
        let s = solve(d, &[Demand { kind: "SPI", count: 2, with_dma: false, options: vec![] }]);
        assert!(s.is_feasible(), "unmet={:?}", s.unmet);
    }

    #[test]
    fn evaluate_verifies_across_the_lineup() {
        let demands = [Demand { kind: "SERIAL", count: 3, with_dma: true, options: vec![] }];
        let results = evaluate(&SearchQuery::default(), &demands);
        // G474RE has a lineup descriptor with DMA -> provably Verified.
        let g4 = results.iter().find(|(e, _)| e.name == "STM32G474RE").expect("G474RE");
        assert_eq!(g4.1, Verdict::Verified);
        // The asset covers every part, so verdicts are real (Verified/Infeasible),
        // not "no descriptor" — and many parts are genuinely Verified.
        assert!(results.iter().filter(|(_, v)| *v == Verdict::Verified).count() > 10);
    }

    #[test]
    fn evaluate_c531_dma_now_verified_via_asset() {
        // The whole-lineup asset is generated from chip JSON, which HAS C5 DMA
        // (metapac omits it). So C531 — which Tier-1 already knew has 8 LPDMA
        // channels — now Tier-2-VERIFIES instead of falling to BoundsOnly. This
        // is the C5 DMA gap closing end-to-end.
        let demands = [Demand { kind: "SERIAL", count: 1, with_dma: true, options: vec![] }];
        let results = evaluate(&SearchQuery::default(), &demands);
        let c531: Vec<_> = results.iter().filter(|(e, _)| e.name.starts_with("STM32C531R")).collect();
        assert!(!c531.is_empty(), "C531R should survive Tier-1");
        assert!(
            c531.iter().all(|(_, v)| *v == Verdict::Verified),
            "C531 should now be Verified (asset has C5 DMA), got {:?}",
            c531.iter().map(|(e, v)| (&e.name, v)).collect::<Vec<_>>(),
        );
    }

    #[test]
    fn lineup_descriptor_asset_loads() {
        // Sanity: the embedded gzipped asset inflates + parses, covering the
        // whole lineup (~870 package-letter descriptors).
        assert!(crate::desc_asset::descriptor_count() > 500);
        let d = crate::desc_asset::descriptor_for("STM32C531RC").expect("C531 in asset");
        assert!(d.dma_channel_total() >= 8, "C531 asset descriptor carries LPDMA channels");
    }

    #[test]
    fn pin_capacity_bound_prunes_pin_starved_parts() {
        // 4 SPI + 4 USART (no DMA) = 4*3 + 4*2 = 20 distinct pins. Small packages
        // (< 20 AF-capable GPIO) are pruned at Tier-1 even if they list the
        // peripherals — the "not enough pins to use them together" case.
        let demands = [
            Demand { kind: "SPI", count: 4, with_dma: false, options: vec![] },
            Demand { kind: "SERIAL", count: 4, with_dma: false, options: vec![] },
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
            Demand { kind: "SERIAL", count: 3, with_dma: true, options: vec![] },
            Demand { kind: "SPI", count: 3, with_dma: true, options: vec![] },
            Demand { kind: "I2C", count: 3, with_dma: true, options: vec![] },
        ];
        let results = evaluate(&SearchQuery::default(), &demands);
        assert!(results.iter().all(|(e, _)| e.dma_pool_total >= 18));
        assert!(!results.iter().any(|(e, _)| e.name == "STM32G474RE"), "G474 (16ch) pruned");
    }

    #[test]
    fn serial_demand_exceeding_instances_is_infeasible() {
        // G474 has 6 async-serial instances (USART1-3 + UART4/5 + LPUART1) — a
        // 7th can't be allocated.
        let d = Package::G474R.descriptor();
        let s = solve(d, &[Demand { kind: "SERIAL", count: 7, with_dma: true, options: vec![] }]);
        assert!(!s.is_feasible());
    }

    #[test]
    fn uart_demand_met_by_usart_superset() {
        // C531 has no UART instances, only USARTs — a SERIAL demand is still
        // satisfiable because USART is a superset of UART.
        let d = Package::C531R.descriptor();
        let s = solve(d, &[Demand { kind: "SERIAL", count: 2, with_dma: false, options: vec![] }]);
        assert!(s.is_feasible(), "USART should satisfy a serial demand; unmet={:?}", s.unmet);
    }

    #[test]
    fn adc_with_dma_allocates() {
        // ADC is demandable with a single-stream DMA channel. G474 has 5 ADCs +
        // DMA capacity; 2 ADCs with DMA = 2 channels <= 16.
        let d = Package::G474R.descriptor();
        let s = solve(d, &[Demand { kind: "ADC", count: 2, with_dma: true, options: vec![] }]);
        assert!(s.is_feasible() && !s.indeterminate, "unmet={:?}", s.unmet);
    }

    #[test]
    fn options_add_required_pins() {
        // SPI full-duplex = 3 pins (SCK/MOSI/MISO); + chip-select = 4 (NSS).
        let base = [Demand { kind: "SPI", count: 1, with_dma: false, options: vec![] }];
        let cs = [Demand { kind: "SPI", count: 1, with_dma: false, options: vec!["chip-select"] }];
        assert_eq!(pin_demand(&base), 3);
        assert_eq!(pin_demand(&cs), 4);
        // USB-PD: CC1/CC2 = 2 pins; + dead-battery adds DBCC1/DBCC2 = 4.
        let pd = [Demand { kind: "UCPD", count: 1, with_dma: false, options: vec!["dead-battery"] }];
        assert_eq!(pin_demand(&pd), 4);
    }

    #[test]
    fn ucpd_dead_battery_places_dbcc_pins() {
        // A USB-PD demand with dead-battery must place CC1/CC2 AND DBCC1/DBCC2.
        // Use the lineup asset descriptor (it carries the DBCC pins).
        let d = crate::desc_asset::descriptor_for("STM32G474RE").expect("G474 in asset");
        let s = solve(
            d,
            &[Demand { kind: "UCPD", count: 1, with_dma: false, options: vec!["dead-battery"] }],
        );
        assert!(s.is_feasible(), "UCPD + dead-battery should place on G474; unmet={:?}", s.unmet);
    }

    #[test]
    fn rxtx_demand_infeasible_when_dma_metadata_absent() {
        // C531's DMA is empty in the compiled metapac, so a RX+TX demand finds no
        // channel candidates -> infeasible. Documents the data-currency gap
        // (resolves on a metapac refresh); the catalog already knows C531 has 8.
        let d = Package::C531R.descriptor();
        let s = solve(d, &[Demand { kind: "SERIAL", count: 1, with_dma: true, options: vec![] }]);
        assert!(!s.is_feasible());
    }
}
