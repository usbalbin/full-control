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

use crate::analog_view::{adc_neg_index, adc_pos_index, comp_minus, comp_plus, opamp_plus};
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

/// One way a comparator can straddle a diff pair for **zero-cross detection**:
/// its non-inverting input (`INP`) on one pin and its external inverting input
/// (`INM`, INMSEL 110/111) on the other, so the output flips when the two ends
/// cross (V₊ = V₋). Which pin is `plus` vs `minus` is a free choice (invert in
/// firmware); we canonicalize `plus` = the INP pin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ZeroCrossOption {
    pub comp: &'static str,
    pub plus: PinId,
    pub minus: PinId,
}

/// Enumerate every comparator that can zero-cross a diff pair whose two ends are
/// both bonded and ADC-capable (they're logging channels too). Data-driven: joins
/// COMP `INP*` pins with COMP external `INM*` pins on the same instance.
pub fn zero_cross_options(
    raw: &'static RawMcuData,
    bonded: Option<&BTreeSet<PinId>>,
) -> Vec<ZeroCrossOption> {
    let mut inp: BTreeMap<&'static str, BTreeSet<PinId>> = BTreeMap::new();
    let mut inm: BTreeMap<&'static str, BTreeSet<PinId>> = BTreeMap::new();
    let mut adc: BTreeSet<PinId> = BTreeSet::new();
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
        if p.starts_with("ADC") && adc_pos_index(r.signal.role).is_some() {
            adc.insert(r.pin);
        } else if p.starts_with("COMP") {
            if comp_plus(r.signal.role) {
                inp.entry(p).or_default().insert(r.pin);
            } else if comp_minus(r.signal.role) {
                inm.entry(p).or_default().insert(r.pin);
            }
        }
    }
    let mut out = Vec::new();
    let comps: BTreeSet<&'static str> = inp.keys().chain(inm.keys()).copied().collect();
    for c in comps {
        let (Some(ps), Some(ms)) = (inp.get(c), inm.get(c)) else { continue };
        for &p in ps {
            for &m in ms {
                if p != m && adc.contains(&p) && adc.contains(&m) {
                    out.push(ZeroCrossOption { comp: c, plus: p, minus: m });
                }
            }
        }
    }
    out
}

/// A pair that can run in the ADC's **hardware differential mode** (`DIFSEL=1`):
/// the two ends are the ADC-hardwired `IN<m>` (`plus`) and `INN<m>` (`minus`)
/// pins on the SAME ADC instance, so channel `m` reads `plus − minus` in one
/// zero-skew conversion. Same wiring as two single-ended front-ends; a runtime
/// `DIFSEL` toggle chooses SE-vs-diff.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdcDiffOption {
    pub adc: &'static str,
    pub channel: u8,
    pub plus: PinId,
    pub minus: PinId,
}

/// Enumerate the ADC hardware-differential-capable pin pairs: channel `m` on an
/// ADC where BOTH `IN<m>` (positive) and `INN<m>` (negative) pins are bonded.
/// Mirrors the "Analog pairs" ADC-diff enumeration, data-driven off `INN<m>`.
pub fn adc_diff_options(
    raw: &'static RawMcuData,
    bonded: Option<&BTreeSet<PinId>>,
) -> Vec<AdcDiffOption> {
    // Positive / negative pin keyed by (ADC instance, channel).
    let mut pos: BTreeMap<(&'static str, u8), PinId> = BTreeMap::new();
    let mut neg: BTreeMap<(&'static str, u8), PinId> = BTreeMap::new();
    for r in af_rows(raw) {
        if r.af.is_some() || !r.signal.peripheral.starts_with("ADC") {
            continue;
        }
        if let Some(b) = bonded
            && !b.contains(&r.pin)
        {
            continue;
        }
        let adc = r.signal.peripheral;
        if let Some(m) = adc_pos_index(r.signal.role) {
            pos.entry((adc, m)).or_insert(r.pin);
        }
        if let Some(m) = adc_neg_index(r.signal.role) {
            neg.entry((adc, m)).or_insert(r.pin);
        }
    }
    let mut out = Vec::new();
    for (&(adc, m), &npin) in &neg {
        if let Some(&ppin) = pos.get(&(adc, m))
            && ppin != npin
        {
            out.push(AdcDiffOption { adc, channel: m, plus: ppin, minus: npin });
        }
    }
    out
}

/// Bonded analog pins by role — the pool that capability taps draw from. Unlike
/// [`PinCap`] this includes pins that are ONLY a COMP INP/INM (not ADC), since a
/// tap pin need not be a logging channel itself.
#[derive(Clone, Debug, Default)]
pub struct TapData {
    pub comp_inp: BTreeMap<&'static str, Vec<PinId>>,
    pub comp_inm: BTreeMap<&'static str, Vec<PinId>>,
    pub opamp_vinp: BTreeMap<&'static str, Vec<PinId>>,
    /// Direct ADC channels on each bonded pin (for redundant-ADC taps).
    pub adc_on_pin: BTreeMap<PinId, Vec<Slot>>,
}

/// Extract the bonded tap pool, restricted to `bonded` when given.
pub fn tap_data(raw: &'static RawMcuData, bonded: Option<&BTreeSet<PinId>>) -> TapData {
    let mut d = TapData::default();
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
                d.adc_on_pin.entry(r.pin).or_default().push((p, c));
            }
        } else if p.starts_with("COMP") {
            if comp_plus(r.signal.role) {
                d.comp_inp.entry(p).or_default().push(r.pin);
            } else if comp_minus(r.signal.role) {
                d.comp_inm.entry(p).or_default().push(r.pin);
            }
        } else if p.starts_with("OPAMP") && opamp_plus(r.signal.role) {
            d.opamp_vinp.entry(p).or_default().push(r.pin);
        }
    }
    for v in d.comp_inp.values_mut().chain(d.comp_inm.values_mut()).chain(d.opamp_vinp.values_mut()) {
        v.sort_unstable();
        v.dedup();
    }
    for v in d.adc_on_pin.values_mut() {
        v.sort_unstable();
        v.dedup();
    }
    d
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

/// A capability gained by **routing this signal to an extra bonded pin** (the
/// pins are all high-Z analog inputs, so paralleling them on the PCB is fine —
/// the cost is the extra pin). Lets a signal use capabilities no single pin has.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TapKind {
    /// Comparator threshold trigger via this INP pin (primary pin had no COMP).
    Trigger(&'static str),
    /// The INP end of a *tapped* zero-cross (a comparator straddling the pair).
    ZeroCrossInp(&'static str),
    /// The external-INM end of a tapped zero-cross.
    ZeroCrossInm(&'static str),
    /// Opamp PGA via this VINP pin — an amplified second read (dual-range).
    Pga(&'static str),
    /// A redundant second ADC read on another instance (guaranteed-simultaneous
    /// sampling / oversample / cross-check).
    RedundantAdc,
}

/// One extra pin wired to a channel's signal, and what it buys.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tap {
    pub pin: PinId,
    pub kind: TapKind,
    /// The ADC read this tap adds, for `Pga` (opamp output) and `RedundantAdc`.
    pub read: Option<AdcRead>,
}

/// One planned single-ended channel. Its signal enters at `pin` (the primary ADC
/// read) and may also be wired to `taps` for capabilities the primary pin lacks.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChannelPlan {
    pub pin: PinId,
    pub read: AdcRead,
    /// Comparator watching the primary pin (a native hardware trigger).
    pub comp: Option<&'static str>,
    /// PGA opamp on the primary pin (present iff `read` is `ViaOpamp`).
    pub opamp: Option<&'static str>,
    /// Extra pins the signal is routed to for more options.
    pub taps: Vec<Tap>,
}

impl ChannelPlan {
    /// The PGA opamp on this signal, native (primary pin) or tapped.
    pub fn pga_opamp(&self) -> Option<&'static str> {
        self.opamp.or_else(|| {
            self.taps.iter().find_map(|t| match t.kind {
                TapKind::Pga(o) => Some(o),
                _ => None,
            })
        })
    }
    pub fn has_pga(&self) -> bool {
        self.pga_opamp().is_some()
    }
    /// A per-channel threshold trigger (native COMP on the primary pin, or a
    /// tapped `Trigger`). Zero-cross ends are NOT counted here.
    pub fn has_trigger(&self) -> bool {
        self.comp.is_some() || self.taps.iter().any(|t| matches!(t.kind, TapKind::Trigger(_)))
    }
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
    /// A comparator straddles the pair (INP on `pos`, external INM on `neg`) for
    /// differential zero-cross detection. `Some(comp)` names the instance.
    pub zero_cross: Option<&'static str>,
    /// The pair sits on an ADC hardware-differential combo (`IN<m>/INN<m>` on one
    /// ADC): `Some((adc, m))` means channel `m` can read `plus − minus` in one
    /// zero-skew conversion (`DIFSEL=1`), toggleable with single-ended at runtime.
    pub adc_diff: Option<(&'static str, u8)>,
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
    /// Best achievable zero-cross pairs (comparator straddling a pair), limited by
    /// distinct comps with a both-ADC INP/INM combo and `channels/2`. Shares the
    /// comparator pool with `max_triggers`.
    pub max_zero_cross_pairs: usize,
    /// Pairs placeable on an ADC hardware-differential combo (`IN<m>/INN<m>`),
    /// limited by distinct pins (adjacent diff channels share a pin) and
    /// `channels/2`. Mutually exclusive with cross-ADC on the same pair.
    pub max_adc_diff_pairs: usize,
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
    fn channels(&self) -> impl Iterator<Item = &ChannelPlan> {
        self.pairs.iter().flat_map(|p| [&p.pos, &p.neg])
    }
    pub fn triggers(&self) -> usize {
        self.channels().filter(|c| c.has_trigger()).count()
    }
    pub fn pga_channels(&self) -> usize {
        self.channels().filter(|c| c.has_pga()).count()
    }
    pub fn full_pga_pairs(&self) -> usize {
        self.pairs.iter().filter(|p| p.full_pga).count()
    }
    pub fn zero_cross_pairs(&self) -> usize {
        self.pairs.iter().filter(|p| p.zero_cross.is_some()).count()
    }
    pub fn adc_diff_pairs(&self) -> usize {
        self.pairs.iter().filter(|p| p.adc_diff.is_some()).count()
    }
    /// Extra bonded pins spent on capability taps.
    pub fn tap_pins(&self) -> usize {
        self.channels().map(|c| c.taps.len()).sum()
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
    pub w_zero_cross: i32,
    pub w_adc_diff: i32,
    /// How many pairs to allocate a straddling comparator for zero-cross
    /// detection (0 = none). These pairs are carved out first — they consume a
    /// comparator and force the pair onto a specific INP/INM pin combo.
    pub zero_cross_target: usize,
    /// How many pairs to place on an ADC hardware-differential combo (`IN<m>/
    /// INN<m>` on one ADC) so `DIFSEL` diff mode is available. Carved first, onto
    /// specific pins; same-ADC, so mutually exclusive with cross-ADC per pair.
    pub adc_diff_target: usize,
    /// Max EXTRA bonded pins the planner may spend on capability taps (routing a
    /// signal to more pins for capabilities its primary pin lacks). 0 = off.
    pub tap_budget: usize,
}

impl Default for PlanConfig {
    fn default() -> Self {
        // Priority: a full-PGA pair (both ends independent gain) is the scarcest
        // and most valuable; then per-channel triggers; a lone PGA and
        // simultaneous-sampling are smaller sweeteners. Zero-cross is opt-in.
        Self {
            channels: 16,
            w_pga_pair: 100,
            w_pga_single: 25,
            w_trigger: 30,
            w_cross_adc: 8,
            w_zero_cross: 40,
            w_adc_diff: 35,
            zero_cross_target: 0,
            adc_diff_target: 0,
            tap_budget: 0,
        }
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
    let zc = zero_cross_options(raw, bonded);
    let diff = adc_diff_options(raw, bonded);
    let taps = tap_data(raw, bonded);
    let channels = cfg.channels.max(2) & !1; // even, ≥2
    plan_from(&caps, &out_adc, &zc, &diff, &taps, channels, cfg)
}

/// Core solver over already-extracted capabilities (the unit-test entry point).
#[allow(clippy::too_many_arguments)]
pub fn plan_from(
    caps: &[PinCap],
    out_adc: &OpampOutAdc,
    zc_options: &[ZeroCrossOption],
    diff_options: &[AdcDiffOption],
    tap_pool: &TapData,
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
    // Zero-cross options restricted to pins that survived into `caps` (bonded &
    // usable) — the extractor may have seen pins the capability filter dropped.
    let zc_opts: Vec<ZeroCrossOption> = zc_options
        .iter()
        .copied()
        .filter(|o| idx_of.contains_key(&o.plus) && idx_of.contains_key(&o.minus))
        .collect();
    let diff_opts: Vec<AdcDiffOption> = diff_options
        .iter()
        .copied()
        .filter(|o| idx_of.contains_key(&o.plus) && idx_of.contains_key(&o.minus))
        .collect();
    let ceiling = Ceiling {
        requested_channels: channels,
        adc_pins: caps.len(),
        comps: all_comps.len(),
        opamps: all_opamps.len(),
        max_pga_pairs: (all_opamps.len() / 2).min(channels / 2),
        max_triggers: all_comps.len().min(channels),
        max_zero_cross_pairs: max_zero_cross(&zc_opts, channels / 2),
        max_adc_diff_pairs: max_adc_diff(&diff_opts, channels / 2),
    };

    // Shared allocation state (the carve stages run first and seed it).
    let mut used_slot: BTreeSet<Slot> = BTreeSet::new();
    let mut reads: BTreeMap<usize, AdcRead> = BTreeMap::new();
    let mut used_comp: BTreeSet<&'static str> = BTreeSet::new();
    let mut zc_pins: BTreeSet<usize> = BTreeSet::new();
    // (pos_idx, neg_idx, comp) for each carved zero-cross pair.
    let mut zc_pairs: Vec<(usize, usize, &'static str)> = Vec::new();
    // (pos_idx, neg_idx, adc, channel) for each carved ADC-diff pair.
    let mut diff_pairs: Vec<(usize, usize, &'static str, u8)> = Vec::new();

    // ---- Stage Z: carve zero-cross pairs (a comparator straddling the pair:
    // INP on one end, external INM on the other). Runs FIRST because it forces
    // the pair onto a specific pin combo. Prefers combos that DON'T consume
    // opamp-capable pins, so PGA options aren't cannibalized. ----
    if cfg.zero_cross_target > 0 {
        let want = cfg.zero_cross_target.min(channels / 2);
        let is_op = |p: PinId| idx_of.get(&p).is_some_and(|i| !caps[*i].opamp.is_empty());
        let mut opts = zc_opts.clone();
        opts.sort_by_key(|o| {
            (is_op(o.plus) as u8 + is_op(o.minus) as u8, o.comp, o.plus, o.minus)
        });
        let mut used_pin: BTreeSet<usize> = BTreeSet::new();
        for o in &opts {
            if zc_pairs.len() >= want {
                break;
            }
            let (pa, pb) = (idx_of[&o.plus], idx_of[&o.minus]);
            if used_comp.contains(o.comp) || used_pin.contains(&pa) || used_pin.contains(&pb) {
                continue;
            }
            let (ra, rb) = pick_cross_slots(Some(&caps[pa].adc), Some(&caps[pb].adc), &used_slot);
            if let (Some(sa), Some(sb)) = (ra, rb) {
                used_slot.insert(sa);
                used_slot.insert(sb);
                reads.insert(pa, AdcRead::Direct { adc: sa.0, ch: sa.1 });
                reads.insert(pb, AdcRead::Direct { adc: sb.0, ch: sb.1 });
                used_pin.insert(pa);
                used_pin.insert(pb);
                zc_pins.insert(pa);
                zc_pins.insert(pb);
                used_comp.insert(o.comp);
                zc_pairs.push((pa, pb, o.comp));
            }
        }
    }

    // ---- Stage Xd: carve ADC hardware-differential pairs onto `IN<m>/INN<m>`
    // combos (same ADC → one zero-skew conversion, `DIFSEL` toggleable with SE).
    // Carved onto specific pins; same-ADC, so these are NOT cross-ADC. ----
    if cfg.adc_diff_target > 0 {
        let want = cfg.adc_diff_target.min(channels / 2 - zc_pairs.len());
        // Prefer combos whose pins are plain ADC (no COMP/OPAMP role), so ADC-diff
        // doesn't needlessly cannibalize trigger / PGA options.
        let rich = |p: PinId| {
            idx_of.get(&p).map_or(0u8, |i| {
                (!caps[*i].opamp.is_empty()) as u8 + (!caps[*i].comp.is_empty()) as u8
            })
        };
        let mut opts = diff_opts.clone();
        opts.sort_by_key(|o| (rich(o.plus) + rich(o.minus), o.adc, o.channel, o.plus, o.minus));
        for o in &opts {
            if diff_pairs.len() >= want {
                break;
            }
            let (pa, pb) = (idx_of[&o.plus], idx_of[&o.minus]);
            if reads.contains_key(&pa) || reads.contains_key(&pb) {
                continue;
            }
            // Both ends read on the SAME ADC: `plus` on channel m, `minus` on its
            // own single-ended channel on that instance.
            let sa = (o.adc, o.channel);
            let Some(sb) = caps[pb].adc.iter().copied().find(|(inst, _)| *inst == o.adc) else {
                continue;
            };
            if used_slot.contains(&sa) || used_slot.contains(&sb) || sa == sb {
                continue;
            }
            used_slot.insert(sa);
            used_slot.insert(sb);
            reads.insert(pa, AdcRead::Direct { adc: sa.0, ch: sa.1 });
            reads.insert(pb, AdcRead::Direct { adc: sb.0, ch: sb.1 });
            diff_pairs.push((pa, pb, o.adc, o.channel));
        }
    }

    // Pins locked by the carve stages — off-limits to opamps/comparators/fill.
    let carved: BTreeSet<usize> = zc_pins
        .iter()
        .copied()
        .chain(diff_pairs.iter().flat_map(|&(a, b, _, _)| [a, b]))
        .collect();
    // Channel budget remaining for the opamp/comp/fill stages.
    let budget_left = channels.saturating_sub(carved.len());

    // ---- Stage A: place opamps on distinct pins (max matching). ----
    let opamps: Vec<&'static str> = all_opamps.iter().copied().collect();
    let op_adj: Vec<Vec<usize>> = opamps
        .iter()
        .map(|op| {
            let mut v: Vec<usize> = caps
                .iter()
                .enumerate()
                .filter(|(i, c)| c.opamp.contains(op) && !carved.contains(i))
                .map(|(i, _)| i)
                .collect();
            v.sort_unstable();
            v
        })
        .collect();
    let op_match = kuhn(&op_adj, caps.len());
    // (pin_idx, opamp) placements, capped at the remaining channel budget.
    let mut opamp_of_pin: BTreeMap<usize, &'static str> = BTreeMap::new();
    for (l, m) in op_match.iter().enumerate() {
        if let Some(pi) = m
            && opamp_of_pin.len() < budget_left
        {
            opamp_of_pin.insert(*pi, opamps[l]);
        }
    }

    // Pair opamp-pins into PGA pairs, maximizing cross-ADC via concrete read slots.
    let mut op_pins: Vec<usize> = opamp_of_pin.keys().copied().collect();
    op_pins.sort_by_key(|i| caps[*i].pin);
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
    // already selected for a PGA (a free co-located trigger). Comparators spent on
    // a zero-cross pair, and the zero-cross pins themselves, are off the table. ----
    let selected: BTreeSet<usize> = reads.keys().copied().collect();
    let comps: Vec<&'static str> = all_comps.iter().copied().filter(|c| !used_comp.contains(c)).collect();
    let comp_adj: Vec<Vec<usize>> = comps
        .iter()
        .map(|cp| {
            let mut v: Vec<usize> = caps
                .iter()
                .enumerate()
                .filter(|(i, c)| c.comp.contains(cp) && !carved.contains(i))
                .map(|(i, _)| i)
                .collect();
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

    // ---- Stage D: pair up the chosen pins. The carved (zero-cross, ADC-diff) and
    // PGA pairs are already formed and stay together; the rest are greedily paired
    // to maximize cross-ADC.
    let mut paired: BTreeSet<usize> = BTreeSet::new();
    let mut pair_idx: Vec<(usize, usize)> = Vec::new();
    let mut zc_comp_of_pair: BTreeMap<(usize, usize), &'static str> = BTreeMap::new();
    let mut diff_of_pair: BTreeMap<(usize, usize), (&'static str, u8)> = BTreeMap::new();
    for &(a, b, comp) in &zc_pairs {
        pair_idx.push((a, b));
        zc_comp_of_pair.insert((a, b), comp);
        paired.insert(a);
        paired.insert(b);
    }
    for &(a, b, adc, ch) in &diff_pairs {
        pair_idx.push((a, b));
        diff_of_pair.insert((a, b), (adc, ch));
        paired.insert(a);
        paired.insert(b);
    }
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

    // ---- Stage T: capability taps. Route a signal to up to cfg.tap_budget spare
    // bonded pins to gain capabilities its primary pin lacks — all high-Z analog
    // inputs, so paralleling on the PCB is fine; the cost is the extra pin. ----
    let mut taps_by_idx: BTreeMap<usize, Vec<Tap>> = BTreeMap::new();
    let mut zc_tapped: BTreeMap<(usize, usize), &'static str> = BTreeMap::new();
    let mut forced_cross: BTreeSet<(usize, usize)> = BTreeSet::new();
    if cfg.tap_budget > 0 {
        let mut budget = cfg.tap_budget;
        let mut used_pins: BTreeSet<PinId> = chosen.iter().map(|&i| caps[i].pin).collect();
        // Free peripherals come from the whole fabric (the tap pool), not just
        // peripherals that happen to reach a channel pin — a comparator/opamp
        // whose pins are all non-channel taps is still usable.
        let native_comps: BTreeSet<&'static str> =
            comp_of_pin.values().copied().chain(used_comp.iter().copied()).collect();
        let mut free_comps: BTreeSet<&'static str> = tap_pool
            .comp_inp
            .keys()
            .chain(tap_pool.comp_inm.keys())
            .copied()
            .filter(|c| !native_comps.contains(c))
            .collect();
        let native_opamps: BTreeSet<&'static str> = opamp_of_pin.values().copied().collect();
        let mut free_opamps: BTreeSet<&'static str> =
            tap_pool.opamp_vinp.keys().copied().filter(|o| !native_opamps.contains(o)).collect();

        // Candidate taps, valued by the same weights; applied highest-value-first
        // (stable sort → insertion order breaks ties deterministically).
        #[derive(Clone, Copy)]
        enum Cand {
            Zc(usize, usize),
            Trig(usize),
            Pga(usize),
            Redun(usize, usize),
        }
        let mut cands: Vec<(i32, Cand)> = Vec::new();
        for &(a, b) in &pair_idx {
            if a != b && !zc_comp_of_pair.contains_key(&(a, b)) {
                cands.push((cfg.w_zero_cross, Cand::Zc(a, b)));
            }
            // Redundant cross-ADC read — but not for an ADC-diff pair, whose
            // same-ADC placement is intentional (the HW diff IS its differential).
            if a != b && reads[&a].adc() == reads[&b].adc() && !diff_of_pair.contains_key(&(a, b)) {
                cands.push((cfg.w_cross_adc, Cand::Redun(a, b)));
            }
        }
        for &pi in &chosen {
            if !comp_of_pin.contains_key(&pi) {
                cands.push((cfg.w_trigger, Cand::Trig(pi)));
            }
            if !opamp_of_pin.contains_key(&pi) {
                cands.push((cfg.w_pga_single, Cand::Pga(pi)));
            }
        }
        cands.sort_by_key(|(v, _)| std::cmp::Reverse(*v));

        // First free pin from a role list, avoiding used pins (and an excluded one).
        let take = |list: Option<&Vec<PinId>>, used: &BTreeSet<PinId>, not: Option<PinId>| {
            list.and_then(|v| v.iter().copied().find(|p| !used.contains(p) && Some(*p) != not))
        };

        for (_, c) in cands {
            match c {
                Cand::Zc(a, b) if budget >= 2 => {
                    // A comparator with a free INP and a free INM pin: prefer a
                    // truly-free one; else RECLAIM one from a native per-channel
                    // trigger (that channel loses its trigger) when zero-cross
                    // outscores a trigger — this is what lets a saturated plan give
                    // a full-PGA pair a zero-cross too.
                    let reclaim_ok = cfg.w_zero_cross > cfg.w_trigger;
                    let reclaimable: Vec<(usize, &'static str)> = if reclaim_ok {
                        comp_of_pin.iter().map(|(pi, c)| (*pi, *c)).collect()
                    } else {
                        Vec::new()
                    };
                    let pick = free_comps
                        .iter()
                        .copied()
                        .map(|c| (c, None))
                        .chain(reclaimable.iter().map(|&(pi, c)| (c, Some(pi))))
                        .find_map(|(comp, from)| {
                            let p = take(tap_pool.comp_inp.get(comp), &used_pins, None)?;
                            let m = take(tap_pool.comp_inm.get(comp), &used_pins, Some(p))?;
                            Some((comp, p, m, from))
                        });
                    if let Some((comp, p, m, from)) = pick {
                        taps_by_idx.entry(a).or_default().push(Tap {
                            pin: p,
                            kind: TapKind::ZeroCrossInp(comp),
                            read: None,
                        });
                        taps_by_idx.entry(b).or_default().push(Tap {
                            pin: m,
                            kind: TapKind::ZeroCrossInm(comp),
                            read: None,
                        });
                        used_pins.insert(p);
                        used_pins.insert(m);
                        match from {
                            Some(pi) => {
                                comp_of_pin.remove(&pi);
                            }
                            None => {
                                free_comps.remove(comp);
                            }
                        }
                        zc_tapped.insert((a, b), comp);
                        budget -= 2;
                    }
                }
                Cand::Trig(pi) if budget >= 1 => {
                    let pick = free_comps
                        .iter()
                        .copied()
                        .find_map(|comp| take(tap_pool.comp_inp.get(comp), &used_pins, None).map(|p| (comp, p)));
                    if let Some((comp, p)) = pick {
                        taps_by_idx.entry(pi).or_default().push(Tap {
                            pin: p,
                            kind: TapKind::Trigger(comp),
                            read: None,
                        });
                        used_pins.insert(p);
                        free_comps.remove(comp);
                        budget -= 1;
                    }
                }
                Cand::Pga(pi) if budget >= 1 => {
                    let pick = free_opamps.iter().copied().find_map(|op| {
                        let p = take(tap_pool.opamp_vinp.get(op), &used_pins, None)?;
                        let (adc, ch) = pick_free_slot(out_adc.get(op), &used_slot)?;
                        Some((op, p, adc, ch))
                    });
                    if let Some((op, p, adc, ch)) = pick {
                        used_slot.insert((adc, ch));
                        taps_by_idx.entry(pi).or_default().push(Tap {
                            pin: p,
                            kind: TapKind::Pga(op),
                            read: Some(AdcRead::ViaOpamp { opamp: op, adc, ch }),
                        });
                        used_pins.insert(p);
                        free_opamps.remove(op);
                        budget -= 1;
                    }
                }
                Cand::Redun(a, b) if budget >= 1 => {
                    let inst = reads[&a].adc();
                    // A free bonded pin with a free ADC slot on a DIFFERENT instance.
                    let pick = tap_pool
                        .adc_on_pin
                        .iter()
                        .filter(|(pin, _)| !used_pins.contains(pin))
                        .find_map(|(pin, slots)| {
                            slots
                                .iter()
                                .copied()
                                .find(|(ai, ci)| *ai != inst && !used_slot.contains(&(*ai, *ci)))
                                .map(|(ai, ci)| (*pin, ai, ci))
                        });
                    if let Some((pin, ai, ci)) = pick {
                        used_slot.insert((ai, ci));
                        taps_by_idx.entry(a).or_default().push(Tap {
                            pin,
                            kind: TapKind::RedundantAdc,
                            read: Some(AdcRead::Direct { adc: ai, ch: ci }),
                        });
                        used_pins.insert(pin);
                        forced_cross.insert((a, b));
                        budget -= 1;
                    }
                }
                _ => {}
            }
        }
    }

    // ---- Build PairPlans + score. ----
    let mk = |pi: usize| ChannelPlan {
        pin: caps[pi].pin,
        read: reads[&pi],
        comp: comp_of_pin.get(&pi).copied(),
        opamp: opamp_of_pin.get(&pi).copied(),
        taps: taps_by_idx.get(&pi).cloned().unwrap_or_default(),
    };
    let mut pairs = Vec::new();
    let mut score = 0i32;
    for (n, &(a, b)) in pair_idx.iter().enumerate() {
        let pos = mk(a);
        let neg = mk(b);
        let full_pga =
            a != b && pos.has_pga() && neg.has_pga() && pos.pga_opamp() != neg.pga_opamp();
        let cross_adc =
            (a != b && pos.read.adc() != neg.read.adc()) || forced_cross.contains(&(a, b));
        let zero_cross =
            zc_comp_of_pair.get(&(a, b)).copied().or_else(|| zc_tapped.get(&(a, b)).copied());
        let adc_diff = diff_of_pair.get(&(a, b)).copied();
        // Scoring.
        if zero_cross.is_some() {
            score += cfg.w_zero_cross;
        }
        if adc_diff.is_some() {
            score += cfg.w_adc_diff;
        }
        if full_pga {
            score += cfg.w_pga_pair;
        } else {
            for c in [&pos, &neg] {
                if c.has_pga() {
                    score += cfg.w_pga_single;
                }
            }
        }
        for c in [&pos, &neg] {
            if c.has_trigger() {
                score += cfg.w_trigger;
            }
        }
        if cross_adc {
            score += cfg.w_cross_adc;
        }
        pairs.push(PairPlan { index: n, pos, neg, full_pga, cross_adc, zero_cross, adc_diff });
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
    if cfg.zero_cross_target > 0 || ceiling.max_zero_cross_pairs > 0 {
        notes.push(format!(
            "Zero-cross: ≤{} pairs can have a straddling comparator (COMP+ on one end, external COMP− on the other); it spends a comparator from the same pool as triggers.",
            ceiling.max_zero_cross_pairs
        ));
    }
    if cfg.adc_diff_target > 0 || ceiling.max_adc_diff_pairs > 0 {
        notes.push(format!(
            "ADC hardware-diff: up to {} pairs sit on an IN/INN combo (same ADC → one zero-skew conversion, DIFSEL-toggleable with single-ended). Mutually exclusive with cross-ADC simultaneous sampling on the same pair.",
            ceiling.max_adc_diff_pairs
        ));
    }
    if unplaced_channels > 0 {
        notes.push(format!(
            "Only {placed} analog-capable pins are bonded on this package — {unplaced_channels} requested channel(s) can't be placed. Pick a larger package."
        ));
    }
    let taps_used: usize = pairs.iter().flat_map(|p| [&p.pos, &p.neg]).map(|c| c.taps.len()).sum();
    if cfg.tap_budget > 0 {
        notes.push(format!(
            "Capability taps: {taps_used} of {} budgeted extra pins spent (a signal wired to more pins buys a trigger / zero-cross / PGA / redundant ADC its primary pin lacks).",
            cfg.tap_budget
        ));
    }

    FrontEndPlan { pairs, ceiling, unplaced_channels, score, notes }
}

/// Achievable ADC hardware-diff pairs: a greedy vertex-disjoint matching over the
/// `(plus, minus)` pin pairs (adjacent diff channels share a pin), capped at
/// `cap`. Greedy is a valid *achievable* count — the number reported as "up to N".
fn max_adc_diff(options: &[AdcDiffOption], cap: usize) -> usize {
    let mut opts: Vec<&AdcDiffOption> = options.iter().collect();
    opts.sort_by_key(|o| (o.adc, o.channel, o.plus, o.minus));
    let mut used: BTreeSet<PinId> = BTreeSet::new();
    let mut n = 0;
    for o in opts {
        if n >= cap {
            break;
        }
        if !used.contains(&o.plus) && !used.contains(&o.minus) {
            used.insert(o.plus);
            used.insert(o.minus);
            n += 1;
        }
    }
    n
}

/// Max zero-cross pairs achievable in isolation: pick options with DISTINCT
/// comparators and DISTINCT pins, capped at `cap`. Branches per comparator (each
/// takes one of its INP/INM combos or none) — a tiny exact search (≤7 comps).
fn max_zero_cross(options: &[ZeroCrossOption], cap: usize) -> usize {
    let mut by_comp: BTreeMap<&'static str, Vec<(PinId, PinId)>> = BTreeMap::new();
    for o in options {
        by_comp.entry(o.comp).or_default().push((o.plus, o.minus));
    }
    let comps: Vec<&'static str> = by_comp.keys().copied().collect();
    fn rec(
        comps: &[&'static str],
        by_comp: &BTreeMap<&'static str, Vec<(PinId, PinId)>>,
        i: usize,
        pins: &mut BTreeSet<PinId>,
        taken: usize,
        cap: usize,
    ) -> usize {
        if taken >= cap {
            return cap;
        }
        if i == comps.len() {
            return taken;
        }
        let mut best = rec(comps, by_comp, i + 1, pins, taken, cap); // skip comp i
        for &(p, m) in &by_comp[comps[i]] {
            if !pins.contains(&p) && !pins.contains(&m) {
                pins.insert(p);
                pins.insert(m);
                best = best.max(rec(comps, by_comp, i + 1, pins, taken + 1, cap));
                pins.remove(&p);
                pins.remove(&m);
            }
        }
        best
    }
    let mut pins = BTreeSet::new();
    rec(&comps, &by_comp, 0, &mut pins, 0, cap)
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
        let p = plan_from(&caps, &out, &[], &[], &TapData::default(), 16, PlanConfig::default());

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
        let p = plan_from(&caps, &out, &[], &[], &TapData::default(), 16, PlanConfig::default());
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
        let p = plan_from(&caps, &out, &[], &[], &TapData::default(), 16, PlanConfig::default());
        for pair in p.pairs.iter().filter(|p| p.full_pga) {
            assert_ne!(pair.pos.opamp, pair.neg.opamp);
            assert!(pair.pos.opamp.is_some() && pair.neg.opamp.is_some());
        }
    }

    #[test]
    fn deterministic() {
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let a = plan_from(&caps, &out, &[], &[], &TapData::default(), 16, PlanConfig::default());
        let b = plan_from(&caps, &out, &[], &[], &TapData::default(), 16, PlanConfig::default());
        assert_eq!(a.pairs, b.pairs);
        assert_eq!(a.score, b.score);
    }

    #[test]
    fn smaller_request_places_fewer_pairs() {
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let p = plan_from(&caps, &out, &[], &[], &TapData::default(), 4, PlanConfig::default());
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
        let p = plan_from(&caps, &out, &[], &[], &TapData::default(), 8, PlanConfig::default());
        assert_eq!(p.ceiling.opamps, 0);
        assert_eq!(p.ceiling.comps, 0);
        assert_eq!(p.full_pga_pairs(), 0);
        assert_eq!(p.triggers(), 0);
    }

    #[test]
    fn zero_cross_options_g474() {
        let zc = zero_cross_options(&crate::mcu_data::g474r::RAW, None);
        let has = |c: &str, p: &str, m: &str| {
            zc.iter().any(|o| o.comp == c && o.plus.name() == p && o.minus.name() == m)
        };
        // Verified against the census: COMP1..4,6,7 have a both-ADC INP/INM combo.
        assert!(has("COMP1", "PA1", "PA0"));
        assert!(has("COMP2", "PA3", "PA2"));
        assert!(has("COMP4", "PB0", "PB2"));
        assert!(has("COMP7", "PB14", "PB12"));
        let comps: BTreeSet<&str> = zc.iter().map(|o| o.comp).collect();
        // COMP5's external INM (PB10) is NOT an ADC pin → no both-ADC zero-cross.
        assert!(!comps.contains("COMP5"), "COMP5 INM (PB10) isn't ADC-capable");
        assert_eq!(comps.len(), 6, "6 of 7 comparators can zero-cross a logging pair");
    }

    #[test]
    fn ceiling_reports_max_zero_cross() {
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let zc = zero_cross_options(&crate::mcu_data::g474r::RAW, None);
        let p = plan_from(&caps, &out, &zc, &[], &TapData::default(), 16, PlanConfig::default());
        // 6 distinct comparators, distinct-pin-feasible → 6; capped by channels/2=8.
        assert_eq!(p.ceiling.max_zero_cross_pairs, 6);
    }

    #[test]
    fn zero_cross_target_carves_pairs() {
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let zc = zero_cross_options(&crate::mcu_data::g474r::RAW, None);
        let cfg = PlanConfig { zero_cross_target: 2, ..Default::default() };
        let p = plan_from(&caps, &out, &zc, &[], &TapData::default(), 16, cfg);

        assert_eq!(p.zero_cross_pairs(), 2, "asked for 2 zero-cross pairs");
        assert_eq!(p.pairs.len(), 8);
        assert_eq!(p.unplaced_channels, 0);

        // Each zero-cross pair: names a comparator, distinct comps, its two ends
        // are a valid INP/INM combo on that comparator, plain-ADC reads, distinct
        // pins and slots. And a zero-cross end is not double-counted as a
        // per-channel trigger.
        let mut zc_comps = BTreeSet::new();
        let mut pins = BTreeSet::new();
        let mut slots = BTreeSet::new();
        for pair in &p.pairs {
            for c in [&pair.pos, &pair.neg] {
                assert!(pins.insert(c.pin), "pin {} reused", c.pin.name());
                assert!(slots.insert(c.read.slot()), "slot {:?} reused", c.read.slot());
            }
            if let Some(comp) = pair.zero_cross {
                assert!(zc_comps.insert(comp), "comparator {comp} reused for zero-cross");
                let ok = zc.iter().any(|o| {
                    o.comp == comp
                        && ((o.plus == pair.pos.pin && o.minus == pair.neg.pin)
                            || (o.plus == pair.neg.pin && o.minus == pair.pos.pin))
                });
                assert!(ok, "{comp} straddles {}/{}", pair.pos.pin.name(), pair.neg.pin.name());
                assert!(matches!(pair.pos.read, AdcRead::Direct { .. }));
                assert!(pair.pos.comp.is_none() && pair.neg.comp.is_none(), "zero-cross ≠ per-channel trigger");
            }
        }
    }

    #[test]
    fn opamp_only_pin_is_usable_via_its_output() {
        // A pin that is an OPAMP VINP but NOT a direct ADC channel is still a valid
        // logging channel — the opamp output reaches an ADC (OPAINTOEN). (No such
        // pin exists on G474, so this is a synthetic chip.)
        let caps = vec![
            PinCap { pin: PinId { port: 'A', num: 0 }, adc: vec![], comp: vec![], opamp: vec!["OPAMP1"] },
            PinCap { pin: PinId { port: 'A', num: 1 }, adc: vec![("ADC1", 2)], comp: vec![], opamp: vec![] },
        ];
        let out: OpampOutAdc = [("OPAMP1", vec![("ADC2", 5)])].into_iter().collect();
        let p = plan_from(&caps, &out, &[], &[], &TapData::default(), 2, PlanConfig::default());
        assert_eq!(p.unplaced_channels, 0, "the opamp-only pin places via its opamp");
        let pa0 = p
            .pairs
            .iter()
            .flat_map(|pr| [&pr.pos, &pr.neg])
            .find(|c| c.pin.name() == "PA0")
            .expect("PA0 is placed");
        assert!(matches!(pa0.read, AdcRead::ViaOpamp { opamp: "OPAMP1", .. }), "read via the opamp");
        assert_eq!(pa0.opamp, Some("OPAMP1"));
    }

    #[test]
    fn adc_diff_options_g474() {
        let d = adc_diff_options(&crate::mcu_data::g474r::RAW, None);
        let has = |adc: &str, ch: u8, p: &str, m: &str| {
            d.iter().any(|o| {
                o.adc == adc && o.channel == ch && o.plus.name() == p && o.minus.name() == m
            })
        };
        // Verified against the Analog-pairs view / RM0440.
        assert!(has("ADC1", 1, "PA0", "PA1"));
        assert!(has("ADC5", 1, "PA8", "PA9"));
        assert!(d.len() > 20, "many diff-capable combos: {}", d.len());
    }

    #[test]
    fn adc_diff_target_places_same_adc_pairs_without_cannibalizing() {
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let diff = adc_diff_options(&crate::mcu_data::g474r::RAW, None);
        let cfg = PlanConfig { adc_diff_target: 2, ..Default::default() };
        let p = plan_from(&caps, &out, &[], &diff, &TapData::default(), 16, cfg);

        assert_eq!(p.adc_diff_pairs(), 2);
        assert_eq!(p.ceiling.max_adc_diff_pairs, 8);
        // Prefer-plain carve keeps all PGA pairs and triggers.
        assert_eq!(p.full_pga_pairs(), 3, "plain-pin diff carve doesn't cost a PGA pair");
        for pr in p.pairs.iter().filter(|pr| pr.adc_diff.is_some()) {
            let (adc, ch) = pr.adc_diff.unwrap();
            // Both ends read on the SAME ADC (that's what HW diff needs), so the
            // pair is NOT cross-ADC — the two are mutually exclusive.
            assert_eq!(pr.pos.read.adc(), adc);
            assert_eq!(pr.neg.read.adc(), adc);
            assert!(!pr.cross_adc, "an ADC-diff pair is same-ADC, not cross-ADC");
            // The declared diff channel is the positive end's channel.
            assert_eq!(pr.pos.read.ch(), ch);
            // The two ends are a real IN/INN combo on that ADC.
            let ok = diff.iter().any(|o| {
                o.adc == adc
                    && ((o.plus == pr.pos.pin && o.minus == pr.neg.pin)
                        || (o.plus == pr.neg.pin && o.minus == pr.pos.pin))
            });
            assert!(ok, "{}/{} is an IN/INN combo on {adc}", pr.pos.pin.name(), pr.neg.pin.name());
        }
    }

    #[test]
    fn tap_data_g474_has_inm_and_vinp() {
        let td = tap_data(&crate::mcu_data::g474r::RAW, None);
        // COMP5's external INM PB10 is a non-ADC pin → present in the tap pool but
        // NOT a zero_cross_option (which requires ADC on both ends).
        assert!(td.comp_inm.get("COMP5").unwrap().iter().any(|p| p.name() == "PB10"));
        assert!(td.opamp_vinp.contains_key("OPAMP1"));
        assert!(td.comp_inp.contains_key("COMP1"));
    }

    #[test]
    fn taps_add_zero_cross_to_a_full_pga_pair() {
        // The headline: on a comparator-saturated 16ch plan, spending tap pins
        // gives a FULL-PGA pair a zero-cross too (reclaiming a comp from a trigger).
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let zc = zero_cross_options(&crate::mcu_data::g474r::RAW, None);
        let td = tap_data(&crate::mcu_data::g474r::RAW, None);
        let base = plan_from(&caps, &out, &zc, &[], &TapData::default(), 16, PlanConfig::default());
        let cfg = PlanConfig { tap_budget: 4, ..Default::default() };
        let p = plan_from(&caps, &out, &zc, &[], &td, 16, cfg);

        assert!(p.score > base.score, "taps improve the objective: {} > {}", p.score, base.score);
        let tapped_fpga = p.pairs.iter().find(|pr| pr.full_pga && pr.zero_cross.is_some());
        let pr = tapped_fpga.expect("a full-PGA pair that also zero-crosses");
        // Its zero-cross is realized by tapped comparator pins on the two ends.
        let inp = pr.pos.taps.iter().chain(pr.neg.taps.iter()).any(|t| matches!(t.kind, TapKind::ZeroCrossInp(_)));
        let inm = pr.pos.taps.iter().chain(pr.neg.taps.iter()).any(|t| matches!(t.kind, TapKind::ZeroCrossInm(_)));
        assert!(inp && inm, "zero-cross via tapped INP + INM pins");
    }

    #[test]
    fn taps_stay_within_budget_and_never_reuse_a_pin_or_slot() {
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let zc = zero_cross_options(&crate::mcu_data::g474r::RAW, None);
        let td = tap_data(&crate::mcu_data::g474r::RAW, None);
        let cfg = PlanConfig { tap_budget: 8, ..Default::default() };
        let p = plan_from(&caps, &out, &zc, &[], &td, 16, cfg);
        assert!(p.tap_pins() <= 8, "within budget");
        let mut pins = BTreeSet::new();
        let mut slots = BTreeSet::new();
        for c in p.pairs.iter().flat_map(|pr| [&pr.pos, &pr.neg]) {
            assert!(pins.insert(c.pin), "primary pin {} reused", c.pin.name());
            assert!(slots.insert(c.read.slot()), "slot {:?} reused", c.read.slot());
            for t in &c.taps {
                assert!(pins.insert(t.pin), "tap pin {} reused", t.pin.name());
                if let Some(r) = t.read {
                    assert!(slots.insert(r.slot()), "tap slot {:?} reused", r.slot());
                }
            }
        }
    }

    #[test]
    fn taps_add_pga_and_redundant_adc() {
        // Synthetic: two plain channels on the SAME ADC (not cross), a spare opamp
        // reachable only via a non-channel VINP pin, and a free pin on another ADC.
        let p = |port, num, adc: &[Slot]| PinCap {
            pin: PinId { port, num },
            adc: adc.to_vec(),
            comp: vec![],
            opamp: vec![],
        };
        let caps = vec![p('A', 0, &[("ADC1", 0)]), p('A', 1, &[("ADC1", 1)])];
        let out: OpampOutAdc = [("OPAMP1", vec![("ADC2", 5)])].into_iter().collect();
        let td = TapData {
            opamp_vinp: [("OPAMP1", vec![PinId { port: 'A', num: 2 }])].into_iter().collect(),
            adc_on_pin: [(PinId { port: 'A', num: 3 }, vec![("ADC2", 3)])].into_iter().collect(),
            ..Default::default()
        };
        let cfg = PlanConfig { channels: 2, tap_budget: 2, ..Default::default() };
        let plan = plan_from(&caps, &out, &[], &[], &td, 2, cfg);

        assert_eq!(plan.pairs.len(), 1);
        let pr = &plan.pairs[0];
        // A PGA tap on the spare opamp, and a redundant-ADC tap that makes the
        // same-ADC pair cross-ADC (simultaneous-sampleable).
        let kinds: Vec<TapKind> =
            pr.pos.taps.iter().chain(pr.neg.taps.iter()).map(|t| t.kind).collect();
        assert!(kinds.iter().any(|k| matches!(k, TapKind::Pga("OPAMP1"))), "PGA tap: {kinds:?}");
        assert!(kinds.iter().any(|k| matches!(k, TapKind::RedundantAdc)), "redundant-ADC tap: {kinds:?}");
        assert!(pr.cross_adc, "redundant tap makes the pair simultaneous-sampleable");
        assert!(plan.pga_channels() >= 1, "the PGA tap counts as a PGA channel");
    }

    #[test]
    fn zero_cross_respects_channel_cap() {
        // 4 channels, ask for 3 zero-cross pairs → capped at 2 pairs total.
        let caps = g474();
        let out = opamp_out_adc(&crate::mcu_data::g474r::RAW);
        let zc = zero_cross_options(&crate::mcu_data::g474r::RAW, None);
        let cfg = PlanConfig { channels: 4, zero_cross_target: 3, ..Default::default() };
        let p = plan_from(&caps, &out, &zc, &[], &TapData::default(), 4, cfg);
        assert_eq!(p.pairs.len(), 2);
        assert_eq!(p.zero_cross_pairs(), 2, "both pairs zero-cross, capped by channels");
    }
}
