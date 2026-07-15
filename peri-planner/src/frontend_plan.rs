//! **Multi-channel analog front-end planner** — the assignment solver behind
//! "pick N single-ended input pins that keep the most options open."
//!
//! The motivating problem (a data-logger / slow-scope front-end): you want, say,
//! 16 single-ended input pins grouped into 8 differential pairs, and you want as
//! many of them as possible to *also* be a COMP input (a hardware trigger) or an
//! OPAMP `VINP` (a programmable-gain stage), with the two ends of each pair on
//! **distinct** opamps (independent PGA) and read on **different ADCs** (so a
//! pair samples simultaneously → good common-mode rejection when subtracted).
//!
//! That is a weighted bipartite assignment against three scarce, *overlapping*
//! resources — ADC channels, the 7 comparators, the 6 opamps — where the richest
//! pins are contended. Tedious and error-prone by hand; small enough to solve
//! well here. This module is pure (no egui) and fully data-driven over
//! [`crate::mcu_pinout::af_rows`]: no per-chip tables, so it works for any family
//! (families lacking COMP/OPAMP simply score no triggers / PGA).
//!
//! Package awareness: pass the active footprint's bonded-GPIO set as `bonded` and
//! only those pins are eligible — so the same solver answers the question per
//! package with no extra code. The one internal path that is *not* gated on
//! bonding is the OPAMP output → ADC channel (`OPAINTOEN`): it is on-die and
//! exists whether or not the `VOUT` pin is bonded, so it is derived from the full
//! die (see [`opamp_out_adc`]).
//!
//! The optimizer is staged (opamps → comparators → fill), each stage an exact
//! search over a tiny instance (≤6 opamps, ≤7 comps). It is not a global optimum
//! proof, but it provably reaches the hardware ceiling on the cases we test
//! (G474: 3 PGA pairs using all 6 opamps, ≥6 triggers) — and the ceiling itself
//! is reported honestly so the user sees *why* not every channel can be rich.

use std::collections::{BTreeMap, BTreeSet};

use crate::analog_view::{adc_pos_index, comp_plus, opamp_plus};
use crate::mcu_pinout::{af_rows, PinId};
use crate::mcu_raw::RawMcuData;

/// An ADC allocation slot: `(instance, channel)`, e.g. `("ADC1", 3)`.
pub type Slot = (&'static str, u8);
/// OPAMP instance → the ADC slots its `OPAINTOEN` output can be digitized on.
pub type OpampOutAdc = BTreeMap<&'static str, Vec<Slot>>;

/// The analog capability of one physical pin, distilled from the AF table.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PinCap {
    pub pin: PinId,
    /// Direct single-ended ADC channels, sorted/deduped.
    pub adc: Vec<Slot>,
    /// COMP instances this pin can drive as a non-inverting (`INP*`) trigger.
    pub comp: Vec<&'static str>,
    /// OPAMP instances this pin can be the `VINP` of (a PGA front-end).
    pub opamp: Vec<&'static str>,
}

impl PinCap {
    /// Number of distinct ADC instances this pin's direct channels live on.
    fn adc_instance_count(&self) -> usize {
        self.adc.iter().map(|(a, _)| *a).collect::<BTreeSet<_>>().len()
    }
}

/// OPAMP output → ADC channel routing (`OPAINTOEN`), derived from the full die
/// (NOT gated on package bonding — the path is internal). Each `OPAMPx_VOUT` pin
/// co-locates with the ADC `IN<n>` role of the channel that reads the opamp
/// output, so the mapping falls straight out of the AF table with no RM tables.
pub fn opamp_out_adc(raw: &'static RawMcuData) -> OpampOutAdc {
    // adc channels present on each pin (die-wide).
    let mut adc_on_pin: BTreeMap<PinId, Vec<Slot>> = BTreeMap::new();
    let mut vout_pin: BTreeMap<&'static str, PinId> = BTreeMap::new();
    for r in af_rows(raw) {
        if r.af.is_some() {
            continue;
        }
        let p = r.signal.peripheral;
        if p.starts_with("ADC") {
            if let Some(c) = adc_pos_index(r.signal.role) {
                adc_on_pin.entry(r.pin).or_default().push((p, c));
            }
        } else if p.starts_with("OPAMP") && r.signal.role == "VOUT" {
            vout_pin.insert(p, r.pin);
        }
    }
    let mut out = BTreeMap::new();
    for (op, pin) in vout_pin {
        if let Some(chs) = adc_on_pin.get(&pin) {
            let mut v = chs.clone();
            v.sort_unstable();
            v.dedup();
            out.insert(op, v);
        }
    }
    out
}

/// Per-pin analog capabilities, restricted to `bonded` pins when given (the
/// active footprint's GPIO set). Returns only *channel-usable* pins: those with a
/// direct ADC channel, or an OPAMP `VINP` whose opamp has an `OPAINTOEN` ADC path
/// (so it can still be digitized through the PGA even without a direct channel).
pub fn capabilities(
    raw: &'static RawMcuData,
    bonded: Option<&BTreeSet<PinId>>,
) -> Vec<PinCap> {
    let opamp_out = opamp_out_adc(raw);
    let mut adc: BTreeMap<PinId, Vec<Slot>> = BTreeMap::new();
    let mut comp: BTreeMap<PinId, BTreeSet<&'static str>> = BTreeMap::new();
    let mut opamp: BTreeMap<PinId, BTreeSet<&'static str>> = BTreeMap::new();

    for r in af_rows(raw) {
        if r.af.is_some() {
            continue;
        }
        if let Some(b) = bonded
            && !b.contains(&r.pin)
        {
            continue;
        }
        let p = r.signal.peripheral;
        if p.starts_with("ADC") {
            if let Some(c) = adc_pos_index(r.signal.role) {
                adc.entry(r.pin).or_default().push((p, c));
            }
        } else if p.starts_with("COMP") && comp_plus(r.signal.role) {
            comp.entry(r.pin).or_default().insert(p);
        } else if p.starts_with("OPAMP") && opamp_plus(r.signal.role) {
            opamp.entry(r.pin).or_default().insert(p);
        }
    }

    let pins: BTreeSet<PinId> =
        adc.keys().chain(comp.keys()).chain(opamp.keys()).copied().collect();
    let mut caps = Vec::new();
    for pin in pins {
        let mut a = adc.get(&pin).cloned().unwrap_or_default();
        a.sort_unstable();
        a.dedup();
        // Opamps this pin can drive that actually have a digitizable output.
        let ops: Vec<&'static str> = opamp
            .get(&pin)
            .map(|s| s.iter().copied().filter(|o| opamp_out.contains_key(o)).collect())
            .unwrap_or_default();
        let usable = !a.is_empty() || !ops.is_empty();
        if !usable {
            continue;
        }
        caps.push(PinCap {
            pin,
            adc: a,
            comp: comp.get(&pin).map(|s| s.iter().copied().collect()).unwrap_or_default(),
            opamp: ops,
        });
    }
    caps
}

/// How a channel's sample is digitized.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdcRead {
    /// The pin is sampled directly on `(adc, ch)`.
    Direct { adc: &'static str, ch: u8 },
    /// The pin drives an opamp PGA whose output is read internally on `(adc, ch)`.
    ViaOpamp { opamp: &'static str, adc: &'static str, ch: u8 },
}

impl AdcRead {
    /// The ADC instance this read lands on (what must differ between a pair's two
    /// ends for simultaneous sampling).
    pub fn adc(&self) -> &'static str {
        match self {
            AdcRead::Direct { adc, .. } | AdcRead::ViaOpamp { adc, .. } => adc,
        }
    }
    pub fn ch(&self) -> u8 {
        match self {
            AdcRead::Direct { ch, .. } | AdcRead::ViaOpamp { ch, .. } => *ch,
        }
    }
    /// The `(instance, channel)` slot this read occupies.
    pub fn slot(&self) -> Slot {
        (self.adc(), self.ch())
    }
}

/// One planned single-ended channel.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChannelPlan {
    pub pin: PinId,
    pub read: AdcRead,
    /// Comparator watching this pin (a hardware trigger), if one was assigned.
    pub comp: Option<&'static str>,
    /// PGA opamp on this channel (present iff `read` is `ViaOpamp`).
    pub opamp: Option<&'static str>,
}

/// One planned differential pair (two single-ended ends).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PairPlan {
    pub index: usize,
    pub pos: ChannelPlan,
    pub neg: ChannelPlan,
    /// Both ends have a PGA on *distinct* opamps → independent per-end gain.
    pub full_pga: bool,
    /// The two ends read on different ADC instances → can sample simultaneously.
    pub cross_adc: bool,
}

/// The hardware ceiling for this chip/package — why not every channel can be rich.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct Ceiling {
    pub requested_channels: usize,
    /// Channel-usable analog pins bonded on this package.
    pub adc_pins: usize,
    /// Distinct comparators reachable from bonded pins.
    pub comps: usize,
    /// Distinct opamps (with a digitizable output) reachable from bonded pins.
    pub opamps: usize,
    /// Best achievable full-PGA pairs = `min(floor(opamps/2), channels/2)`.
    pub max_pga_pairs: usize,
    /// Best achievable hardware triggers = `min(comps, channels)`.
    pub max_triggers: usize,
}

/// The solved plan plus its diagnostics.
#[derive(Clone, Debug, Default)]
pub struct FrontEndPlan {
    pub pairs: Vec<PairPlan>,
    pub ceiling: Ceiling,
    /// Channels we could not place (too few bonded analog pins for the request).
    pub unplaced_channels: usize,
    pub score: i32,
    /// Human-readable ceiling / trade-off explanations for the UI.
    pub notes: Vec<String>,
}

impl FrontEndPlan {
    pub fn triggers(&self) -> usize {
        self.pairs.iter().flat_map(|p| [&p.pos, &p.neg]).filter(|c| c.comp.is_some()).count()
    }
    pub fn pga_channels(&self) -> usize {
        self.pairs.iter().flat_map(|p| [&p.pos, &p.neg]).filter(|c| c.opamp.is_some()).count()
    }
    pub fn full_pga_pairs(&self) -> usize {
        self.pairs.iter().filter(|p| p.full_pga).count()
    }
}

/// Solver weights and channel count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlanConfig {
    /// Total single-ended channels (rounded up to an even number of pairs).
    pub channels: usize,
    pub w_pga_pair: i32,
    pub w_pga_single: i32,
    pub w_trigger: i32,
    pub w_cross_adc: i32,
}

impl Default for PlanConfig {
    fn default() -> Self {
        // Priority: a full-PGA pair (both ends independent gain) is the scarcest
        // and most valuable; then per-channel triggers; a lone PGA and
        // simultaneous-sampling are smaller sweeteners.
        Self { channels: 16, w_pga_pair: 100, w_pga_single: 25, w_trigger: 30, w_cross_adc: 8 }
    }
}

// ---------------------------------------------------------------------------
// Bipartite maximum matching (Kuhn's augmenting-path). Left = a scarce resource
// (opamp / comp) index; right = pin index. Deterministic given sorted adjacency.
// ---------------------------------------------------------------------------

fn kuhn(adj: &[Vec<usize>], n_right: usize) -> Vec<Option<usize>> {
    fn aug(u: usize, adj: &[Vec<usize>], seen: &mut [bool], match_r: &mut [Option<usize>]) -> bool {
        for &v in &adj[u] {
            if !seen[v] {
                seen[v] = true;
                if match_r[v].is_none() || aug(match_r[v].unwrap(), adj, seen, match_r) {
                    match_r[v] = Some(u);
                    return true;
                }
            }
        }
        false
    }
    let mut match_r = vec![None; n_right];
    for u in 0..adj.len() {
        let mut seen = vec![false; n_right];
        aug(u, adj, &mut seen, &mut match_r);
    }
    // Invert to match_left.
    let mut match_l = vec![None; adj.len()];
    for (v, m) in match_r.iter().enumerate() {
        if let Some(u) = m {
            match_l[*u] = Some(v);
        }
    }
    match_l
}

/// Solve the front-end assignment. `bonded` restricts to a package's pins.
pub fn plan(raw: &'static RawMcuData, bonded: Option<&BTreeSet<PinId>>, cfg: PlanConfig) -> FrontEndPlan {
    let caps = capabilities(raw, bonded);
    let out_adc = opamp_out_adc(raw);
    let channels = cfg.channels.max(2) & !1; // even, ≥2
    plan_from(&caps, &out_adc, channels, cfg)
}

/// Core solver over already-extracted capabilities (the unit-test entry point).
pub fn plan_from(
    caps: &[PinCap],
    out_adc: &OpampOutAdc,
    channels: usize,
    cfg: PlanConfig,
) -> FrontEndPlan {
    // Stable pin order for determinism.
    let mut caps: Vec<PinCap> = caps.to_vec();
    caps.sort_by_key(|c| c.pin);
    let idx_of: BTreeMap<PinId, usize> = caps.iter().enumerate().map(|(i, c)| (c.pin, i)).collect();

    // ---- Ceiling (independent of the assignment chosen). ----
    let all_opamps: BTreeSet<&'static str> = caps.iter().flat_map(|c| c.opamp.iter().copied()).collect();
    let all_comps: BTreeSet<&'static str> = caps.iter().flat_map(|c| c.comp.iter().copied()).collect();
    let ceiling = Ceiling {
        requested_channels: channels,
        adc_pins: caps.len(),
        comps: all_comps.len(),
        opamps: all_opamps.len(),
        max_pga_pairs: (all_opamps.len() / 2).min(channels / 2),
        max_triggers: all_comps.len().min(channels),
    };

    // ---- Stage A: place opamps on distinct pins (max matching). ----
    let opamps: Vec<&'static str> = all_opamps.iter().copied().collect();
    let op_adj: Vec<Vec<usize>> = opamps
        .iter()
        .map(|op| {
            let mut v: Vec<usize> =
                caps.iter().enumerate().filter(|(_, c)| c.opamp.contains(op)).map(|(i, _)| i).collect();
            v.sort_unstable();
            v
        })
        .collect();
    let op_match = kuhn(&op_adj, caps.len());
    // (pin_idx, opamp) placements, capped at `channels`.
    let mut opamp_of_pin: BTreeMap<usize, &'static str> = BTreeMap::new();
    for (l, m) in op_match.iter().enumerate() {
        if let Some(pi) = m
            && opamp_of_pin.len() < channels
        {
            opamp_of_pin.insert(*pi, opamps[l]);
        }
    }

    // Pair opamp-pins into PGA pairs, maximizing cross-ADC via concrete read slots.
    let mut op_pins: Vec<usize> = opamp_of_pin.keys().copied().collect();
    op_pins.sort_by_key(|i| caps[*i].pin);
    let mut used_slot: BTreeSet<(&'static str, u8)> = BTreeSet::new();
    let mut reads: BTreeMap<usize, AdcRead> = BTreeMap::new();
    let mut pga_pairs: Vec<(usize, usize)> = Vec::new();
    // Greedy pairing: consume op_pins two at a time, choosing read instances that
    // differ (cross-ADC) when the two opamps' OPAINTOEN paths allow.
    let mut i = 0;
    while i + 1 < op_pins.len() {
        let (pa, pb) = (op_pins[i], op_pins[i + 1]);
        let (oa, ob) = (opamp_of_pin[&pa], opamp_of_pin[&pb]);
        let (ra, rb) = pick_cross_slots(out_adc.get(oa), out_adc.get(ob), &used_slot);
        if let (Some((aa, ca)), Some((ab, cb))) = (ra, rb) {
            used_slot.insert((aa, ca));
            used_slot.insert((ab, cb));
            reads.insert(pa, AdcRead::ViaOpamp { opamp: oa, adc: aa, ch: ca });
            reads.insert(pb, AdcRead::ViaOpamp { opamp: ob, adc: ab, ch: cb });
            pga_pairs.push((pa, pb));
        }
        i += 2;
    }
    // A leftover odd opamp-pin stays a single-PGA channel (paired later).
    let leftover_op: Option<usize> = (op_pins.len() % 2 == 1).then(|| *op_pins.last().unwrap());
    if let Some(pi) = leftover_op {
        let op = opamp_of_pin[&pi];
        if let Some((a, c)) = pick_free_slot(out_adc.get(op), &used_slot) {
            used_slot.insert((a, c));
            reads.insert(pi, AdcRead::ViaOpamp { opamp: op, adc: a, ch: c });
        }
    }

    // ---- Stage B: place comparators to maximize triggers, preferring pins
    // already selected for a PGA (a free co-located trigger). ----
    let selected: BTreeSet<usize> = reads.keys().copied().collect();
    let comps: Vec<&'static str> = all_comps.iter().copied().collect();
    let comp_adj: Vec<Vec<usize>> = comps
        .iter()
        .map(|cp| {
            let mut v: Vec<usize> =
                caps.iter().enumerate().filter(|(_, c)| c.comp.contains(cp)).map(|(i, _)| i).collect();
            // Already-selected pins first → Kuhn biases triggers onto them.
            v.sort_by_key(|i| (!selected.contains(i), *i));
            v
        })
        .collect();
    let comp_match = kuhn(&comp_adj, caps.len());
    let mut comp_of_pin: BTreeMap<usize, &'static str> = BTreeMap::new();
    for (l, m) in comp_match.iter().enumerate() {
        if let Some(pi) = m {
            comp_of_pin.insert(*pi, comps[l]);
        }
    }

    // ---- Stage C: assemble the selected-pin set (opamp ∪ comp), then fill with
    // plain ADC pins up to `channels`. ----
    let mut chosen: Vec<usize> = reads.keys().copied().collect();
    for &pi in comp_of_pin.keys() {
        if chosen.len() >= channels {
            break;
        }
        if let std::collections::btree_map::Entry::Vacant(e) = reads.entry(pi)
            && let Some((a, c)) = pick_free_slot(Some(&caps[pi].adc), &used_slot)
        {
            used_slot.insert((a, c));
            e.insert(AdcRead::Direct { adc: a, ch: c });
            chosen.push(pi);
        }
    }
    // Fill: remaining usable pins with a free direct ADC slot, most-flexible first
    // (fewest ADC options last so scarce single-ADC pins get placed while slots
    // are plentiful), tie-broken by pin order for determinism.
    let mut fillers: Vec<usize> = (0..caps.len())
        .filter(|i| !reads.contains_key(i) && !caps[*i].adc.is_empty())
        .collect();
    fillers.sort_by_key(|i| (caps[*i].adc_instance_count(), caps[*i].pin));
    for pi in fillers {
        if chosen.len() >= channels {
            break;
        }
        if let Some((a, c)) = pick_free_slot(Some(&caps[pi].adc), &used_slot) {
            used_slot.insert((a, c));
            reads.insert(pi, AdcRead::Direct { adc: a, ch: c });
            chosen.push(pi);
        }
    }

    // ---- Stage D: pair up the chosen pins. PGA pairs stay together; the rest are
    // greedily paired to maximize cross-ADC. ----
    let _ = &idx_of; // reserved for future locked-pin overrides
    let mut paired: BTreeSet<usize> = BTreeSet::new();
    let mut pair_idx: Vec<(usize, usize)> = Vec::new();
    for &(a, b) in &pga_pairs {
        pair_idx.push((a, b));
        paired.insert(a);
        paired.insert(b);
    }
    let mut rest: Vec<usize> = chosen.iter().copied().filter(|i| !paired.contains(i)).collect();
    rest.sort_by_key(|i| caps[*i].pin);
    // Greedy cross-ADC pairing over the remainder.
    while let Some(first) = rest.first().copied() {
        rest.remove(0);
        let fa = reads[&first].adc();
        // Prefer a partner on a different ADC instance.
        let partner_pos = rest.iter().position(|p| reads[p].adc() != fa).or(if rest.is_empty() {
            None
        } else {
            Some(0)
        });
        if let Some(p) = partner_pos {
            let second = rest.remove(p);
            pair_idx.push((first, second));
        } else {
            // Odd one out — pair with itself's slot as a lone channel (rare).
            pair_idx.push((first, first));
        }
    }

    // ---- Build PairPlans + score. ----
    let mk = |pi: usize| ChannelPlan {
        pin: caps[pi].pin,
        read: reads[&pi],
        comp: comp_of_pin.get(&pi).copied(),
        opamp: opamp_of_pin.get(&pi).copied(),
    };
    let mut pairs = Vec::new();
    let mut score = 0i32;
    for (n, &(a, b)) in pair_idx.iter().enumerate() {
        let pos = mk(a);
        let neg = mk(b);
        let full_pga = a != b
            && pos.opamp.is_some()
            && neg.opamp.is_some()
            && pos.opamp != neg.opamp;
        let cross_adc = a != b && pos.read.adc() != neg.read.adc();
        // Scoring.
        if full_pga {
            score += cfg.w_pga_pair;
        } else {
            for c in [&pos, &neg] {
                if c.opamp.is_some() {
                    score += cfg.w_pga_single;
                }
            }
        }
        for c in [&pos, &neg] {
            if c.comp.is_some() {
                score += cfg.w_trigger;
            }
        }
        if cross_adc {
            score += cfg.w_cross_adc;
        }
        pairs.push(PairPlan { index: n, pos, neg, full_pga, cross_adc });
    }

    let placed = chosen.len();
    let unplaced_channels = channels.saturating_sub(placed);
    let mut notes = Vec::new();
    notes.push(format!(
        "{} of {} channels can carry a hardware trigger (only {} comparators exist).",
        ceiling.max_triggers, channels, ceiling.comps
    ));
    notes.push(format!(
        "At most {} of {} pairs can be full-PGA (both ends on distinct opamps) — {} opamps exist.",
        ceiling.max_pga_pairs,
        channels / 2,
        ceiling.opamps
    ));
    if unplaced_channels > 0 {
        notes.push(format!(
            "Only {placed} analog-capable pins are bonded on this package — {unplaced_channels} requested channel(s) can't be placed. Pick a larger package."
        ));
    }

    FrontEndPlan { pairs, ceiling, unplaced_channels, score, notes }
}

/// Choose two ADC read slots (one from each opamp's OPAINTOEN options) that land
/// on *different* ADC instances when possible, avoiding `used`.
fn pick_cross_slots(
    a: Option<&Vec<Slot>>,
    b: Option<&Vec<Slot>>,
    used: &BTreeSet<Slot>,
) -> (Option<Slot>, Option<Slot>) {
    let (Some(a), Some(b)) = (a, b) else {
        return (pick_free_slot(a, used), pick_free_slot(b, used));
    };
    // Try to find a cross-ADC pair of free slots.
    for &(aa, ca) in a {
        if used.contains(&(aa, ca)) {
            continue;
        }
        for &(ab, cb) in b {
            if ab != aa && !used.contains(&(ab, cb)) && (ab, cb) != (aa, ca) {
                return (Some((aa, ca)), Some((ab, cb)));
            }
        }
    }
    // Fall back to any two distinct free slots.
    let sa = pick_free_slot(Some(a), used);
    let mut used2 = used.clone();
    if let Some(s) = sa {
        used2.insert(s);
    }
    (sa, pick_free_slot(Some(b), &used2))
}

/// First free `(adc, ch)` slot from `opts`, avoiding `used`.
fn pick_free_slot(opts: Option<&Vec<Slot>>, used: &BTreeSet<Slot>) -> Option<Slot> {
    opts?.iter().copied().find(|s| !used.contains(s))
}

/// The bonded-GPIO pin set of a footprint — the `bonded` argument to [`plan`].
pub fn bonded_pins(record: &crate::phys_pinout::PinoutRecord) -> BTreeSet<PinId> {
    let mut set = BTreeSet::new();
    for p in &record.pins {
        if let crate::phys_pinout::PinFunction::Gpio(pins) = p.function() {
            set.extend(pins);
        }
    }
    set
}

#[cfg(test)]
mod tests {
    use super::*;

    fn g474() -> Vec<PinCap> {
        capabilities(&crate::mcu_data::g474r::RAW, None)
    }

    #[test]
    fn opamp_out_adc_matches_known_g474_routing() {
        let m = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        // Spot-check the RM0440 Table 204 pin-bearing rows.
        assert_eq!(m.get("OPAMP1"), Some(&vec![("ADC1", 3)]));
        assert!(m.get("OPAMP3").unwrap().contains(&("ADC1", 12)));
        assert!(m.get("OPAMP3").unwrap().contains(&("ADC3", 1)));
        assert!(m.get("OPAMP6").unwrap().contains(&("ADC2", 14)));
    }

    #[test]
    fn g474_capability_buckets() {
        let caps = g474();
        let triple = caps.iter().filter(|c| !c.adc.is_empty() && !c.comp.is_empty() && !c.opamp.is_empty());
        let names: BTreeSet<String> = triple.map(|c| c.pin.name()).collect();
        // The seven triple-capable pins from the census.
        let want: BTreeSet<String> =
            ["PA1", "PA3", "PA7", "PB0", "PB11", "PB13", "PB14"].iter().map(|s| s.to_string()).collect();
        assert_eq!(names, want);
    }

    #[test]
    fn g474_ceiling_and_hits_it() {
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let p = plan_from(&caps, &out, 16, PlanConfig::default());

        // Ceiling from the silicon: 6 opamps → 3 PGA pairs; 7 comps → ≤7 triggers.
        assert_eq!(p.ceiling.opamps, 6);
        assert_eq!(p.ceiling.comps, 7);
        assert_eq!(p.ceiling.max_pga_pairs, 3);

        // The solver reaches the PGA ceiling and gets most of the triggers.
        assert_eq!(p.full_pga_pairs(), 3, "should place all 3 full-PGA pairs");
        assert_eq!(p.pga_channels(), 6, "all 6 opamps used");
        assert!(p.triggers() >= 6, "at least 6 of 7 comparators become triggers, got {}", p.triggers());
        assert_eq!(p.pairs.len(), 8);
        assert_eq!(p.unplaced_channels, 0);
    }

    #[test]
    fn every_channel_has_a_distinct_pin_and_adc_slot() {
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let p = plan_from(&caps, &out, 16, PlanConfig::default());
        let mut pins = BTreeSet::new();
        let mut slots = BTreeSet::new();
        for pair in &p.pairs {
            for c in [&pair.pos, &pair.neg] {
                assert!(pins.insert(c.pin), "pin {} reused", c.pin.name());
                assert!(slots.insert(c.read.slot()), "ADC slot {:?} reused", c.read.slot());
                // opamp flag ⇔ ViaOpamp read.
                assert_eq!(c.opamp.is_some(), matches!(c.read, AdcRead::ViaOpamp { .. }));
            }
        }
    }

    #[test]
    fn full_pga_pairs_use_distinct_opamps() {
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let p = plan_from(&caps, &out, 16, PlanConfig::default());
        for pair in p.pairs.iter().filter(|p| p.full_pga) {
            assert_ne!(pair.pos.opamp, pair.neg.opamp);
            assert!(pair.pos.opamp.is_some() && pair.neg.opamp.is_some());
        }
    }

    #[test]
    fn deterministic() {
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let a = plan_from(&caps, &out, 16, PlanConfig::default());
        let b = plan_from(&caps, &out, 16, PlanConfig::default());
        assert_eq!(a.pairs, b.pairs);
        assert_eq!(a.score, b.score);
    }

    #[test]
    fn smaller_request_places_fewer_pairs() {
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let p = plan_from(&caps, &out, 4, PlanConfig::default());
        assert_eq!(p.pairs.len(), 2);
        assert_eq!(p.ceiling.max_pga_pairs, 2, "4 channels → at most 2 pairs regardless of opamps");
    }

    #[test]
    fn h523_has_adc_channels_but_no_rich_features() {
        // Family-agnostic: H5 has ADC (INP<n>) but no COMP/OPAMP, so channels
        // place but nothing is a trigger or PGA.
        let caps = capabilities(&crate::mcu_data::h523r::RAW, None);
        assert!(!caps.is_empty(), "H5 exposes ADC channels");
        let out = opamp_out_adc(&crate::mcu_data::h523r::RAW);
        let p = plan_from(&caps, &out, 8, PlanConfig::default());
        assert_eq!(p.ceiling.opamps, 0);
        assert_eq!(p.ceiling.comps, 0);
        assert_eq!(p.full_pga_pairs(), 0);
        assert_eq!(p.triggers(), 0);
    }
}
