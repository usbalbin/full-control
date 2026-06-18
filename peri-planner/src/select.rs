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
        // Each OCP route needs a distinct comparator — a sound, cheap necessary
        // bound. The break-timer side (and the COMP→break edge) is the Tier-2
        // check, since the catalog has no break-capable-timer count column.
        "OCP" => e.comp,
        // Distinct complementary-PWM channels: a sound upper bound (a channel
        // also needs both pins placeable — the Tier-2 check). Counts advanced AND
        // complementary-GP timers, so it never under-counts a feasible part.
        "COMP_PWM" => e.comp_pwm_ch,
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

/// One allocated element in a feasible design — the actionable answer a
/// `Verified` part owes the user (what to wire).
#[derive(Clone, Debug)]
pub enum AssignedPeri {
    /// A comms / ADC peripheral: the chosen instance, each signal's assigned
    /// pin, and how many DMA channels it consumes.
    Instance { peri: &'static str, pins: Vec<(&'static str, PinId)>, dma_channels: u8 },
    /// A hardware over-current route: a comparator output trips a timer break
    /// input. Internal silicon routing — claims no pins and no DMA. `break_input`
    /// names the specific BRK (1) / BRK2 (2) input when a fabric edge pins it
    /// down, else `None` (the universal break-mux assumption; see `ocp_candidates`).
    Ocp { comp: &'static str, timer: &'static str, break_input: Option<u8> },
    /// A complementary PWM channel: a timer channel driving a high-side (CHx) and
    /// low-side (CHxN) output with deadtime — one synchronous-converter
    /// half-bridge. `hs`/`ls` are the assigned output pins.
    CompPwm { timer: &'static str, channel: u8, hs: PinId, ls: PinId },
}

/// A full allocation witness for a feasible design.
pub type Witness = Vec<AssignedPeri>;

/// A kernel candidate paired with the witness element it yields if the kernel
/// picks it, so a solved assignment can be turned back into a named witness.
struct CandMeta {
    candidate: Candidate,
    assigned: AssignedPeri,
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
    dma: u8,
) -> Vec<CandMeta> {
    let Some(peri) = peri_name(d, uclass, n) else {
        return Vec::new();
    };
    let inst = Res::Inst(uclass, n);
    pin_combos(d, peri, gpio)
        .into_iter()
        .map(|pins| {
            let role_pins: Vec<(&'static str, PinId)> =
                gpio.iter().copied().zip(pins.iter().copied()).collect();
            let mut tokens = Vec::with_capacity(1 + pins.len());
            tokens.push(inst);
            tokens.extend(pins.iter().map(|&p| Res::Pin(p)));
            CandMeta {
                candidate: Candidate::new(peri, tokens),
                assigned: AssignedPeri::Instance { peri, pins: role_pins, dma_channels: dma },
            }
        })
        .collect()
}

/// Candidates for one hardware-OCP route: each pairs a comparator with a
/// break-capable timer (advanced or complementary-GP — the timers with a BRK
/// input), claiming both `Inst` tokens so the kernel hands every OCP channel a
/// distinct comparator AND a distinct timer (i.e. N *independent* protected
/// converters need N comps and N break-timers). No pins, no DMA — the trip is
/// internal silicon. When the descriptor carries the COMP→break fabric we use
/// its real edges; otherwise we fall back to the universal property that every
/// advanced-timer break mux can select the on-chip comparators (the same
/// assumption the Tier-1 `require_ocp_capable` bound already encodes).
fn ocp_candidates(d: &McuDescriptor) -> Vec<CandMeta> {
    let comps: Vec<u8> = d.comps.iter().map(|c| c.number).collect();
    let break_timers: Vec<u8> =
        d.timers.iter().filter(|t| t.has_complementary).map(|t| t.number).collect();

    let mk = |comp: u8, tim: u8, brk: Option<u8>| {
        let tokens = vec![Res::Inst("COMP", comp), Res::Inst("TIM", tim)];
        let cname = peri_name(d, "COMP", comp).unwrap_or("COMP");
        let tname = peri_name(d, "TIM", tim).unwrap_or("TIM");
        CandMeta {
            candidate: Candidate::new(tname, tokens),
            assigned: AssignedPeri::Ocp { comp: cname, timer: tname, break_input: brk },
        }
    };

    match d.fabric {
        // Real fabric edges (e.g. C531 from RM0522): only existing instances.
        Some(f) if !f.comp_to_tim_break.is_empty() => f
            .comp_to_tim_break
            .iter()
            .filter(|&&(c, t, _)| comps.contains(&c) && break_timers.contains(&t))
            .map(|&(c, t, b)| mk(c, t, Some(b)))
            .collect(),
        // No fabric on this descriptor (the whole-lineup asset path): full mux.
        _ => comps
            .iter()
            .flat_map(|&c| break_timers.iter().map(move |&t| (c, t)))
            .map(|(c, t)| mk(c, t, None))
            .collect(),
    }
}

/// High- and low-side complementary timer-output roles, indexed by channel-1.
/// These are the BARE signal strings (`pins_for` keys them per timer instance);
/// CH4N exists only on advanced timers, CH1N on every complementary timer.
const CH_ROLES: [&str; 4] = ["CH1", "CH2", "CH3", "CH4"];
const CHN_ROLES: [&str; 4] = ["CH1N", "CH2N", "CH3N", "CH4N"];

/// Every wireable complementary-PWM slot on `d`: `(timer number, timer name,
/// channel)` for each complementary-capable timer channel that breaks out BOTH a
/// high-side (CHx) and a low-side (CHxN) pin on this package. Data-driven from the
/// raw pin/AF table, so it works lineup-wide through the descriptor asset. A
/// channel with a CHxN signal but no usable CHx pin (or vice-versa) is omitted —
/// you can't wire a half-bridge without both legs.
fn comp_pwm_slots(d: &McuDescriptor) -> Vec<(u8, &'static str, u8)> {
    let mut slots = Vec::new();
    for t in d.timers.iter().filter(|t| t.has_complementary) {
        let Some(peri) = peri_name(d, "TIM", t.number) else { continue };
        for ch in 0..4u8 {
            let hs = pins_for(d.raw, SignalId { peripheral: peri, role: CH_ROLES[ch as usize] });
            let ls = pins_for(d.raw, SignalId { peripheral: peri, role: CHN_ROLES[ch as usize] });
            if !hs.is_empty() && !ls.is_empty() {
                slots.push((t.number, peri, ch + 1));
            }
        }
    }
    slots
}

/// Distinct wireable complementary-PWM channels on `d` — the definitive Tier-2
/// slot-capacity bound (exact from real CHx/CHxN pins).
fn comp_pwm_slot_count(d: &McuDescriptor) -> usize {
    comp_pwm_slots(d).len()
}

/// Candidates for one complementary-PWM channel: every wireable `(timer,channel)`
/// slot, each placed on a distinct high-side (CHx) and low-side (CHxN) pin. The
/// slot token `Sub("TIMCH", timer, channel)` makes the kernel hand each demanded
/// channel a DISTINCT timer channel (so N half-bridges need N real complementary
/// outputs), while a single advanced timer can still supply several. Deliberately
/// a different `Res` variant from OCP's `Inst("TIM", n)`, so a PWM channel and an
/// OCP break on the same timer don't spuriously conflict this increment. Both
/// output pins are `Pin` tokens, so they contend with every other peripheral.
fn complementary_pwm_candidates(d: &McuDescriptor) -> Vec<CandMeta> {
    let mut out = Vec::new();
    for (tnum, peri, channel) in comp_pwm_slots(d) {
        let ch = (channel - 1) as usize;
        let hs_pins = pins_for(d.raw, SignalId { peripheral: peri, role: CH_ROLES[ch] });
        let ls_pins = pins_for(d.raw, SignalId { peripheral: peri, role: CHN_ROLES[ch] });
        let slot = Res::Sub("TIMCH", tnum, channel);
        for &hs in &hs_pins {
            for &ls in &ls_pins {
                if hs == ls {
                    continue; // a pin can't be both legs
                }
                out.push(CandMeta {
                    candidate: Candidate::new(
                        format!("{peri} CH{channel}"),
                        vec![slot, Res::Pin(hs), Res::Pin(ls)],
                    ),
                    assigned: AssignedPeri::CompPwm { timer: peri, channel, hs, ls },
                });
            }
        }
    }
    out
}

/// All candidates for one requirement of `kind` with `options`. For a peripheral
/// kind: every instance of every underlying class (so a SERIAL demand can take a
/// USART, UART or LPUART), each with its pin placements for the configured
/// signals. For "OCP": every comparator→break-timer pairing. For "COMP_PWM":
/// every wireable complementary timer channel. The kernel picks distinct ones via
/// `Inst` / `Sub` exclusivity.
fn kind_candidates(d: &McuDescriptor, kind: &str, options: &[&str], dma: u8) -> Vec<CandMeta> {
    if kind == "OCP" {
        return ocp_candidates(d);
    }
    if kind == "COMP_PWM" {
        return complementary_pwm_candidates(d);
    }
    let gpio = required_signals(kind, options);
    let mut out = Vec::new();
    for &uc in underlying_classes(kind) {
        for n in class_instances(d, uc) {
            out.extend(instance_candidates(d, uc, n, &gpio, dma));
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

/// Sentinel ids for a complementary-PWM slot shortfall (more channels demanded
/// than the chip has distinct complementary timer channels). Distinct id range.
const SLOT_UNMET_BASE: u32 = u32::MAX - 2048;

/// Lower `demands` into a feasibility check against `d`.
///
/// Instances and their GPIO pins go through the backtracking kernel — the
/// non-symmetric dimension (distinct instances, contended pins, where greedy can
/// wrongly drop a capable part). DMA channels are a separate aggregate capacity
/// count (fungible -> counting, not CSP). Feasible iff every instance+pin
/// requirement is satisfiable AND total channel demand fits reachable capacity.
pub fn solve(d: &McuDescriptor, demands: &[Demand]) -> Solution {
    run(d, demands).0
}

/// Like [`solve`], but also returns the allocation [`Witness`] (which instance,
/// pins and DMA each requirement got) when the design is feasible — what the
/// user needs to actually wire it.
pub fn allocate(d: &McuDescriptor, demands: &[Demand]) -> (Solution, Witness) {
    let (sol, metas) = run(d, demands);
    let witness = if sol.is_feasible() && !sol.indeterminate {
        sol.assigned
            .iter()
            .filter_map(|&(req_id, ci)| {
                metas.get((req_id - 1) as usize).and_then(|m| m.get(ci)).cloned()
            })
            .collect()
    } else {
        Vec::new()
    };
    (sol, witness)
}

/// Build the requirements + per-candidate witness metadata, run the backtracking
/// solve, and fold in the DMA-channel capacity check. The shared core of `solve`
/// / `allocate`. `metas[req-1][candidate]` is the witness element that candidate
/// yields, kept parallel to the kernel candidates so a solved assignment maps
/// straight back to a named witness.
fn run(d: &McuDescriptor, demands: &[Demand]) -> (Solution, Vec<Vec<AssignedPeri>>) {
    let mut reqs: Vec<Requirement> = Vec::new();
    let mut metas: Vec<Vec<AssignedPeri>> = Vec::new();
    let mut id = 0u32;
    for dem in demands {
        let dma = if dem.with_dma { dma_channels_per_instance(dem.kind) as u8 } else { 0 };
        let cm = kind_candidates(d, dem.kind, &dem.options, dma);
        let candidates: Vec<Candidate> = cm.iter().map(|m| m.candidate.clone()).collect();
        let meta: Vec<AssignedPeri> = cm.into_iter().map(|m| m.assigned).collect();
        for _ in 0..dem.count {
            id += 1;
            reqs.push(Requirement { id, candidates: candidates.clone() });
            metas.push(meta.clone());
        }
    }

    // Definitive complementary-PWM slot guard, BEFORE the CSP. Distinct wireable
    // (timer,channel) slots is a hard cap; demanding more is infeasible by
    // counting alone. Short-circuiting here both gives a clean `Infeasible` and
    // spares the backtracker from proving slot-exhaustion by permuting the many
    // per-slot pin candidates (the symmetric blow-up pattern).
    let pwm_demand: usize =
        demands.iter().filter(|dem| dem.kind == "COMP_PWM").map(|dem| dem.count as usize).sum();
    if pwm_demand > 0 {
        let cap = comp_pwm_slot_count(d);
        if pwm_demand > cap {
            let mut sol = Solution { assigned: Vec::new(), unmet: Vec::new(), indeterminate: false };
            for k in 0..(pwm_demand - cap).min(1024) {
                sol.unmet.push(SLOT_UNMET_BASE + k as u32);
            }
            return (sol, metas);
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
    (sol, metas)
}

// ---------- Two-tier catalog evaluation ----------

/// Caches the Part-finder evaluation so the (whole-lineup) Tier-2 solve runs only
/// when the query or demands actually change — not every egui frame. Holds
/// `Option<Verdict>` (None = plain search, no constraints active).
/// One Part-finder result row: the part, its verdict (`None` = plain search,
/// no constraints active), and the allocation witness for a `Verified` part.
pub type Row = (&'static CatalogEntry, Option<Verdict>, Option<Witness>);

#[derive(Default)]
pub struct EvalCache {
    key: Option<(SearchQuery, Vec<DemandInput>)>,
    results: Vec<Row>,
}

impl EvalCache {
    /// Results for `(query, demands)`, recomputing only on change. With no active
    /// demand it's a plain catalog search (no solving); otherwise it's the
    /// two-tier `evaluate` (which also carries the witness for verified parts).
    pub fn results(&mut self, query: &SearchQuery, demands: &[DemandInput]) -> &[Row] {
        let changed = self.key.as_ref().map(|(q, d)| q != query || d != demands).unwrap_or(true);
        if changed {
            let active: Vec<Demand> =
                demands.iter().filter(|d| d.count > 0).map(DemandInput::to_demand).collect();
            self.results = if active.is_empty() {
                catalog::search(query).into_iter().map(|e| (e, None, None)).collect()
            } else {
                evaluate(query, &active).into_iter().map(|(e, v, w)| (e, Some(v), w)).collect()
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
            "OCP" => "HW OCP (COMP→timer)",
            "COMP_PWM" => "Complementary PWM (CHx/CHxN)",
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
    /// Whether this kind can use DMA — OCP is an internal silicon trip (no DMA),
    /// so the UI hides its DMA toggle.
    pub fn supports_dma(&self) -> bool {
        dma_channels_per_instance(self.kind) > 0
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
pub fn evaluate(
    base: &SearchQuery,
    demands: &[Demand],
) -> Vec<(&'static CatalogEntry, Verdict, Option<Witness>)> {
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
                return (e, Verdict::Infeasible, None);
            }
            // Tier-2: solve against the part's descriptor from the whole-lineup
            // asset (covers every part, and carries C5 DMA that metapac omits).
            match crate::desc_asset::descriptor_for(&e.name) {
                None => (e, Verdict::BoundsOnly, None), // asset somehow lacks this prefix
                Some(d) => {
                    let (sol, witness) = allocate(d, demands);
                    if sol.indeterminate {
                        (e, Verdict::BoundsOnly, None)
                    } else if sol.is_feasible() {
                        // Carry the allocation so the UI can show what to wire.
                        (e, Verdict::Verified, Some(witness))
                    } else {
                        (e, Verdict::Infeasible, None)
                    }
                }
            }
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
        let g4 = results.iter().find(|(e, _, _)| e.name == "STM32G474RE").expect("G474RE");
        assert_eq!(g4.1, Verdict::Verified);
        // A Verified part carries an allocation witness (what to wire).
        assert!(g4.2.as_ref().is_some_and(|w| w.len() == 3), "Verified G474 carries a 3-peri witness");
        // The asset covers every part, so verdicts are real (Verified/Infeasible),
        // not "no descriptor" — and many parts are genuinely Verified.
        assert!(results.iter().filter(|(_, v, _)| *v == Verdict::Verified).count() > 10);
    }

    #[test]
    fn evaluate_c531_dma_now_verified_via_asset() {
        // The whole-lineup asset is generated from chip JSON, which HAS C5 DMA
        // (metapac omits it). So C531 — which Tier-1 already knew has 8 LPDMA
        // channels — now Tier-2-VERIFIES instead of falling to BoundsOnly. This
        // is the C5 DMA gap closing end-to-end.
        let demands = [Demand { kind: "SERIAL", count: 1, with_dma: true, options: vec![] }];
        let results = evaluate(&SearchQuery::default(), &demands);
        let c531: Vec<_> =
            results.iter().filter(|(e, _, _)| e.name.starts_with("STM32C531R")).collect();
        assert!(!c531.is_empty(), "C531R should survive Tier-1");
        assert!(
            c531.iter().all(|(_, v, _)| *v == Verdict::Verified),
            "C531 should now be Verified (asset has C5 DMA), got {:?}",
            c531.iter().map(|(e, v, _)| (&e.name, v)).collect::<Vec<_>>(),
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
            results.iter().all(|(e, _, _)| e.gpio_pins >= 20),
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
        assert!(results.iter().all(|(e, _, _)| e.dma_pool_total >= 18));
        assert!(!results.iter().any(|(e, _, _)| e.name == "STM32G474RE"), "G474 (16ch) pruned");
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
    fn allocate_witness_names_instances_and_pins() {
        // The actionable answer a Verified part owes: per demanded peripheral, the
        // chosen instance, each signal's pin, and the DMA channels consumed.
        // Asserted on a G474 (DMA via metapac) and a C531 (DMA via the asset).
        let demands = [Demand { kind: "SERIAL", count: 2, with_dma: true, options: vec![] }];

        let (sol, w) = allocate(Package::G474R.descriptor(), &demands);
        assert!(sol.is_feasible() && !sol.indeterminate, "G474 should verify; unmet={:?}", sol.unmet);
        assert_eq!(w.len(), 2, "two serial peripherals in the witness");
        let mut peris = Vec::new();
        let mut all_pins = Vec::new();
        for ap in &w {
            let AssignedPeri::Instance { peri, pins, dma_channels } = ap else {
                panic!("serial demand yields Instance witnesses, got {ap:?}");
            };
            // A real async-serial instance, with TX and RX placed on real pins,
            // each consuming one DMA channel (RX+TX = 2).
            assert!(
                ["USART", "UART", "LPUART"].iter().any(|c| peri.starts_with(c)),
                "named a serial instance, got {peri}",
            );
            let roles: Vec<_> = pins.iter().map(|(r, _)| *r).collect();
            assert!(roles.contains(&"TX") && roles.contains(&"RX"), "TX+RX placed, got {roles:?}");
            assert!(pins.iter().all(|(_, p)| p.name().starts_with('P')), "pins named PXn");
            assert_eq!(*dma_channels, 2, "RX+TX = 2 channels");
            peris.push(*peri);
            all_pins.extend(pins.iter().map(|(_, p)| *p));
        }
        // Distinct instances and distinct pins across the two peripherals.
        assert_ne!(peris[0], peris[1], "two distinct instances");
        let distinct: BTreeSet<_> = all_pins.iter().collect();
        assert_eq!(all_pins.len(), distinct.len(), "no pin reused across peripherals");

        // C531: DMA comes from the whole-lineup asset (metapac omits C5 DMA).
        let c531 = crate::desc_asset::descriptor_for("STM32C531RC").expect("C531 in asset");
        let (sol, w) = allocate(c531, &demands);
        assert!(sol.is_feasible() && !sol.indeterminate, "C531 should verify; unmet={:?}", sol.unmet);
        assert_eq!(w.len(), 2);
        for ap in &w {
            let AssignedPeri::Instance { peri, dma_channels, .. } = ap else { panic!("instance") };
            assert!(peri.starts_with("USART"), "C531 serial is USART-only, got {peri}");
            assert_eq!(*dma_channels, 2, "RX+TX = 2 channels each");
        }
    }

    #[test]
    fn ocp_routes_allocate_distinct_comp_and_timer() {
        // Hardware OCP: each route consumes a distinct comparator AND a distinct
        // break-capable timer. C531 (compiled fabric) routes its comparators into
        // TIM1/TIM8 break inputs per RM0522 — two independent OCP loops fit.
        let d = Package::C531R.descriptor();
        let (sol, w) = allocate(d, &[Demand { kind: "OCP", count: 2, with_dma: false, options: vec![] }]);
        assert!(sol.is_feasible() && !sol.indeterminate, "2 OCP routes on C531; unmet={:?}", sol.unmet);
        assert_eq!(w.len(), 2);
        let mut comps = Vec::new();
        let mut timers = Vec::new();
        for ap in &w {
            let AssignedPeri::Ocp { comp, timer, break_input } = ap else {
                panic!("OCP demand yields Ocp witnesses, got {ap:?}");
            };
            assert!(comp.starts_with("COMP"), "named a comparator, got {comp}");
            assert!(timer.starts_with("TIM"), "named a break timer, got {timer}");
            // C531 has fabric, so the specific BRK / BRK2 input is pinned down.
            assert!(matches!(break_input, Some(1 | 2)), "fabric pins the break input, got {break_input:?}");
            comps.push(*comp);
            timers.push(*timer);
        }
        assert_ne!(comps[0], comps[1], "two independent loops use distinct comparators");
        assert_ne!(timers[0], timers[1], "two independent loops use distinct timers");
    }

    #[test]
    fn ocp_routes_exceed_fabric_is_infeasible() {
        // C531's RM0522 fabric routes only COMP1/COMP2 into TIM1/TIM8 break
        // inputs — two distinct comparators and two distinct timers. A third
        // *independent* OCP loop can't get a distinct routed pair — infeasible.
        let d = Package::C531R.descriptor();
        let s = solve(d, &[Demand { kind: "OCP", count: 3, with_dma: false, options: vec![] }]);
        assert!(!s.is_feasible(), "3 independent OCP loops exceed C531's 2 routed comp→timer pairs");
    }

    #[test]
    fn ocp_verified_lineup_wide_via_asset_fallback() {
        // The whole-lineup asset has no fabric, so OCP uses the universal
        // break-mux fallback (any comparator → any break-timer). A single OCP
        // route still VERIFIES on G474 (the UI's asset path), with the break
        // input left unspecified (no fabric edge to pin it down).
        let d = crate::desc_asset::descriptor_for("STM32G474RE").expect("G474 in asset");
        let (sol, w) = allocate(d, &[Demand { kind: "OCP", count: 1, with_dma: false, options: vec![] }]);
        assert!(sol.is_feasible() && !sol.indeterminate, "1 OCP route on G474 asset; unmet={:?}", sol.unmet);
        assert_eq!(w.len(), 1);
        assert!(
            matches!(&w[0], AssignedPeri::Ocp { break_input: None, .. }),
            "asset path has no fabric edge, so break input is unspecified, got {:?}",
            w[0],
        );
    }

    #[test]
    fn comp_pwm_slots_are_data_driven_and_tier1_sound() {
        // The descriptor's wireable complementary channels (both CHx AND CHxN
        // pins placeable) must never EXCEED the catalog's comp_pwm_ch column —
        // otherwise the Tier-1 bound (entry_kind_count) would wrongly prune a
        // feasible part. This is the soundness invariant that makes the cheap
        // catalog filter safe.
        for (name, pkg) in [("STM32G474RE", Package::G474R), ("STM32C531RC", Package::C531R)] {
            let d = pkg.descriptor();
            let slots = comp_pwm_slot_count(d);
            let cat = catalog::CATALOG.iter().find(|e| e.name == name).expect(name);
            assert!(
                slots <= cat.comp_pwm_ch as usize,
                "{name}: descriptor {slots} wireable channels > catalog Tier-1 bound {}",
                cat.comp_pwm_ch,
            );
            // TIM1 + TIM8 alone give 8 complementary channels on these families.
            assert!(slots >= 8, "{name}: expected >=8 complementary channels, got {slots}");
        }
    }

    #[test]
    fn complementary_pwm_allocates_with_distinct_slots_and_pins() {
        // Four complementary PWM channels (one advanced timer's worth) place onto
        // four distinct (timer,channel) slots, each with a distinct HS (CHx) and
        // LS (CHxN) pin — a four-phase synchronous converter front-end.
        let d = Package::G474R.descriptor();
        let (sol, w) =
            allocate(d, &[Demand { kind: "COMP_PWM", count: 4, with_dma: false, options: vec![] }]);
        assert!(sol.is_feasible() && !sol.indeterminate, "4 compl PWM on G474; unmet={:?}", sol.unmet);
        assert_eq!(w.len(), 4);
        let mut slots = BTreeSet::new();
        let mut pins = Vec::new();
        for ap in &w {
            let AssignedPeri::CompPwm { timer, channel, hs, ls } = ap else {
                panic!("COMP_PWM demand yields CompPwm witnesses, got {ap:?}");
            };
            assert!(timer.starts_with("TIM"), "named a timer, got {timer}");
            assert!((1..=4).contains(channel), "channel 1..=4, got {channel}");
            assert_ne!(hs, ls, "high- and low-side on distinct pins");
            assert!(hs.name().starts_with('P') && ls.name().starts_with('P'), "pins named PXn");
            slots.insert((*timer, *channel));
            pins.push(*hs);
            pins.push(*ls);
        }
        assert_eq!(slots.len(), 4, "four distinct (timer,channel) slots");
        let distinct: BTreeSet<_> = pins.iter().collect();
        assert_eq!(pins.len(), distinct.len(), "no pin reused across channels");
    }

    #[test]
    fn complementary_pwm_exceeding_slots_is_infeasible() {
        // Demanding more complementary channels than the chip has distinct
        // complementary timer channels is infeasible — and DEFINITIVELY so (the
        // slot count guard short-circuits before the backtracker, so it's a clean
        // Infeasible, not a budget-cut indeterminate).
        let d = Package::G474R.descriptor();
        let cap = comp_pwm_slot_count(d);
        let s = solve(
            d,
            &[Demand { kind: "COMP_PWM", count: (cap + 1) as u8, with_dma: false, options: vec![] }],
        );
        assert!(!s.is_feasible(), "demanding {} > {cap} complementary channels is infeasible", cap + 1);
        assert!(!s.indeterminate, "slot exhaustion is a definitive count, not budget-cut");
    }

    #[test]
    fn comp_pwm_and_ocp_are_independent_on_shared_timers() {
        // A complementary-PWM channel (Res::Sub("TIMCH",..)) and an OCP route
        // (Res::Inst("TIM",..)) use distinct Res variants, so they never fight
        // over a timer — even on C531 where the PWM timers ARE the break timers.
        // 4 PWM channels + 2 OCP routes coexist.
        let d = Package::C531R.descriptor();
        let s = solve(
            d,
            &[
                Demand { kind: "COMP_PWM", count: 4, with_dma: false, options: vec![] },
                Demand { kind: "OCP", count: 2, with_dma: false, options: vec![] },
            ],
        );
        assert!(s.is_feasible() && !s.indeterminate, "PWM+OCP coexist on C531; unmet={:?}", s.unmet);
    }

    #[test]
    fn comp_pwm_has_no_dma() {
        // Complementary PWM is an output, not a DMA stream — the toggle is hidden
        // and a stray with_dma never adds channel demand.
        assert!(!DemandInput::new("COMP_PWM").supports_dma());
        let with_dma = [Demand { kind: "COMP_PWM", count: 4, with_dma: true, options: vec![] }];
        assert_eq!(channel_demand(&with_dma), 0);
    }

    #[test]
    fn evaluate_complementary_pwm_verifies_and_tier1_is_sound() {
        let demands = [Demand { kind: "COMP_PWM", count: 4, with_dma: false, options: vec![] }];
        let results = evaluate(&SearchQuery::default(), &demands);
        // G474RE (11 complementary channels) verifies for 4, with a 4-channel witness.
        let g4 = results.iter().find(|(e, _, _)| e.name == "STM32G474RE").expect("G474RE");
        assert_eq!(g4.1, Verdict::Verified);
        assert!(g4.2.as_ref().is_some_and(|w| w.len() == 4), "Verified part carries a 4-channel witness");
        // Tier-1 soundness: nothing that survived as Verified/BoundsOnly is below
        // the necessary bound, and parts below it exist and are excluded (Infeasible).
        assert!(
            results
                .iter()
                .filter(|(_, v, _)| *v != Verdict::Infeasible)
                .all(|(e, _, _)| e.comp_pwm_ch >= 4),
            "every non-infeasible survivor meets the >=4 complementary-channel bound",
        );
        assert!(catalog::CATALOG.iter().any(|e| e.comp_pwm_ch < 4), "sub-4-channel parts exist");
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
