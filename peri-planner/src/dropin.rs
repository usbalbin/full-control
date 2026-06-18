//! Drop-in / pin-compatibility matching engine. Given the user's current part
//! and its per-pin allocation, find other STM32 parts that are **board-level
//! drop-in replacements**: same physical footprint, same power/ground on the
//! same physical pins, and able to serve every wired function on the *same*
//! physical pin (no re-route — a PCB trace stays where it is).
//!
//! This is a per-physical-position *direct substitution* check, not a CSP. The
//! substrate is the physical pinout asset (`phys_pinout`); the candidate's
//! per-pin capability comes from its logical AF table (`mcu_pinout::af_rows` over
//! the descriptor asset). The engine joins them by physical position.
//!
//! ## Soundness — the verdict is a conjunction of necessary conditions
//! Every gate can only *reject*; loosening (strictness, power-mode) never flips a
//! real failure to a pass:
//! 1. **Footprint** — candidate must be the same land-pattern class + pin count
//!    (`package_class`). A QFN is never a drop-in for a QFP.
//! 2. **Coverage** — every source power/allocated position must *exist* on the
//!    candidate (a `"LQFP64"` that bonds only 52 pins is not the same
//!    footprint). A missing required position is a hard fail.
//! 3. **Power** — every source power position must be the same canonical
//!    [`Rail`] set on the candidate (rail subset), normalized so family
//!    spelling variants compare correctly.
//! 4. **Serviceability** — every allocated position must carry a
//!    capability-equivalent signal on the *candidate's own* AF table (the
//!    candidate's pin name at a position may differ from the source's).
//! 5. **Analog** signals (no AF) are matched exactly regardless of strictness —
//!    fixed silicon can't be re-muxed.
//!
//! The only honest gaps (surfaced, never hidden): stm32-data omits NRST/BOOT0 for
//! some families (so a cross-family reset/boot net can't be verified ->
//! `reset_boot_unverified`), and carries no 5V-tolerance / pad-type data (a match
//! is pin-function compatible, not pad-electrical-verified).

use std::collections::{BTreeSet, HashMap};

use crate::desc_asset::descriptor_for;
use crate::mcu::Package;
use crate::mcu_pinout::{OwnedSignal, PinId};
use crate::mcu_raw::RawMcuData;
use crate::phys_pinout::{self, package_class, PinFunction, PinoutRecord};

/// How strictly a candidate signal must match the wired one to "serve" a pin.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Strictness {
    /// Same peripheral instance + role (USART2.TX -> only USART2.TX).
    ExactInstanceRole,
    /// Same logical class + role (USART2.TX -> any SERIAL TX). The default.
    ClassRole,
    /// Same logical class, role-agnostic (USART2.TX -> any SERIAL signal).
    ClassOnly,
}

impl Strictness {
    pub const ALL: [Strictness; 3] =
        [Strictness::ExactInstanceRole, Strictness::ClassRole, Strictness::ClassOnly];
    pub fn label(self) -> &'static str {
        match self {
            Strictness::ExactInstanceRole => "Exact instance+role",
            Strictness::ClassRole => "Class + role",
            Strictness::ClassOnly => "Class only",
        }
    }
}

/// What to do when a position the source left as unused GPIO is a power pin on
/// the candidate (denser parts add VDD/VSS pins).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PowerMode {
    /// List the part, flag the position. The default.
    Warn,
    /// Disqualify the part.
    Reject,
    /// Don't-care.
    Ignore,
}

impl PowerMode {
    pub const ALL: [PowerMode; 3] = [PowerMode::Warn, PowerMode::Reject, PowerMode::Ignore];
    pub fn label(self) -> &'static str {
        match self {
            PowerMode::Warn => "Warn",
            PowerMode::Reject => "Reject",
            PowerMode::Ignore => "Ignore",
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct MatchConfig {
    pub strictness: Strictness,
    pub power_mode: PowerMode,
}

impl Default for MatchConfig {
    fn default() -> Self {
        Self { strictness: Strictness::ClassRole, power_mode: PowerMode::Warn }
    }
}

/// The tier at which a pin is served — drives ranking (`Exact` is the best
/// drop-in: identical firmware). Ordered best-first.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum MatchTier {
    Exact,
    ClassRole,
    ClassOnly,
}

/// A wired function: the user routed `(peripheral, role)` to a logical pin.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WiredFn {
    pub sig: OwnedSignal,
    pub pin: PinId,
}

/// The active design's allocation, family-agnostic. Lowered to `Vec<WiredFn>`.
pub enum DesignSource<'a> {
    G474(&'a crate::requirements::Design),
    H523(&'a crate::h523_design::H523Design),
    /// C531's model stores converter *legs*, not locked pins (placement happens
    /// in the view), so it currently contributes no wired pins — the finder then
    /// matches on footprint + power only.
    C531(&'a crate::c531_design::C531Design),
}

impl DesignSource<'_> {
    pub fn wired(&self) -> Vec<WiredFn> {
        match self {
            DesignSource::G474(d) => d
                .pin_assignments
                .iter()
                .map(|(o, p)| WiredFn { sig: o.clone(), pin: *p })
                .collect(),
            DesignSource::H523(d) => d
                .pin_locks
                .iter()
                .map(|l| WiredFn {
                    sig: OwnedSignal { peripheral: l.peripheral.clone(), role: l.role.clone() },
                    pin: l.pin(),
                })
                .collect(),
            DesignSource::C531(_) => Vec::new(),
        }
    }
}

/// The source side: the part being designed, its concrete footprint, and the
/// wired allocation.
pub struct SourceProfile {
    pub name: String,
    pub footprint: &'static PinoutRecord,
    pub wired: Vec<WiredFn>,
}

impl SourceProfile {
    /// A content fingerprint of the source for cache invalidation: changes when
    /// the part, its footprint, or any wired pin changes.
    pub fn fingerprint(&self) -> String {
        let mut parts: Vec<String> = self
            .wired
            .iter()
            .map(|w| format!("{}.{}@{}{}", w.sig.peripheral, w.sig.role, w.pin.port, w.pin.num))
            .collect();
        parts.sort();
        format!("{}|{}|{}", self.name, self.footprint.h, parts.join(","))
    }

    /// Count of positions that the match must satisfy (power + allocated GPIO).
    pub fn required_count(&self) -> u16 {
        let by_pin = self.wired_by_pin();
        self.footprint
            .pins
            .iter()
            .filter(|sp| match sp.function() {
                PinFunction::Power(_) => true,
                PinFunction::Gpio(pins) => pins.iter().any(|p| by_pin.contains_key(p)),
                _ => false,
            })
            .count() as u16
    }

    fn wired_by_pin(&self) -> HashMap<PinId, &WiredFn> {
        self.wired.iter().map(|w| (w.pin, w)).collect()
    }
}

/// Build the source profile from the active design + selected package. `None` if
/// the package's footprint isn't in the pinout asset.
pub fn build_source_profile(src: &DesignSource, package: Package) -> Option<SourceProfile> {
    let name = package.name();
    let label = package.package_label();
    // Suffix-tolerant: compiled C5 descriptors carry an ordering-code suffix
    // ("STM32C531RCT6") that the index doesn't key on ("STM32C531RC").
    let footprint = phys_pinout::footprint_for(name, label)?;
    // Keep only wired pins that physically exist as a GPIO on this footprint.
    // Defends against stale locks from another MCU: `h523_design` (the pin-lock
    // model) is shared between H523 and C5A3 and is not cleared on MCU switch.
    let gpio_pins: BTreeSet<PinId> = footprint
        .pins
        .iter()
        .filter_map(|p| match p.function() {
            PinFunction::Gpio(ps) => Some(ps),
            _ => None,
        })
        .flatten()
        .collect();
    let wired = src.wired().into_iter().filter(|w| gpio_pins.contains(&w.pin)).collect();
    Some(SourceProfile { name: name.to_string(), footprint, wired })
}

/// The outcome at one physical position (drives the per-pin diff + coloring).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PosOutcome {
    /// Source power rail(s) present on the candidate. (gate pass)
    PowerOk,
    /// Source power position is not the same rail on the candidate. (hard fail)
    PowerMismatch,
    /// Allocated pin served, by exact instance+role. (gate pass)
    ServiceableExact,
    /// Allocated pin served, by same class+role (e.g. USART2.TX -> UART4.TX).
    ServiceableClassRole,
    /// Allocated pin served, same class, different role.
    ServiceableClassOnly,
    /// Allocated pin not serviceable on the candidate. (hard fail)
    FunctionFail,
    /// Unused source GPIO, still a GPIO on the candidate. (don't-care pass)
    UnusedOk,
    /// Unused source GPIO is a power pin on the candidate (power-asymmetry).
    PowerAsym,
    /// Unused source GPIO became a dedicated/NC pin on the candidate. (info)
    UnusedRepurposed,
    /// Source dedicated pin (NRST/BOOT0/…) matches the candidate. (pass)
    DedicatedOk,
    /// Source dedicated pin is a *different* concrete function. (hard fail)
    DedicatedMismatch,
    /// Source dedicated pin absent from the candidate's data — can't verify.
    DedicatedUnverified,
    /// Nothing on either side. (don't-care)
    DontCare,
}

impl PosOutcome {
    fn is_blocker(&self, power_mode: PowerMode) -> bool {
        matches!(
            self,
            PosOutcome::PowerMismatch | PosOutcome::FunctionFail | PosOutcome::DedicatedMismatch
        ) || (*self == PosOutcome::PowerAsym && power_mode == PowerMode::Reject)
    }
    fn is_satisfied(&self) -> bool {
        matches!(
            self,
            PosOutcome::PowerOk
                | PosOutcome::ServiceableExact
                | PosOutcome::ServiceableClassRole
                | PosOutcome::ServiceableClassOnly
        )
    }
}

#[derive(Clone, Debug, Copy, PartialEq, Eq)]
pub enum SourceKind {
    Power,
    Allocated,
    UnusedGpio,
    Dedicated,
    Nc,
}

/// One physical position's verdict, with a human-readable detail line.
#[derive(Clone, Debug)]
pub struct PosResult {
    pub position: String,
    pub kind: SourceKind,
    pub outcome: PosOutcome,
    pub detail: String,
}

/// A candidate part evaluated against the source.
#[derive(Clone, Debug)]
pub struct CandidateMatch {
    pub name: String,
    pub family: String,
    pub footprint: String,
    pub compatible: bool,
    pub same_family: bool,
    /// The reset/boot net could not be verified (a family in the swap omits
    /// NRST/BOOT0 from stm32-data) — needs manual check, not a clean pass.
    pub reset_boot_unverified: bool,
    pub n_satisfied: u16,
    pub n_required: u16,
    pub warnings: u16,
    pub score: i64,
    pub positions: Vec<PosResult>,
}

#[derive(Clone)]
struct AfCap {
    peripheral: String,
    role: String,
    af: Option<u8>,
}

fn build_af_oracle(raw: &'static RawMcuData) -> HashMap<PinId, Vec<AfCap>> {
    let mut m: HashMap<PinId, Vec<AfCap>> = HashMap::new();
    for row in crate::mcu_pinout::af_rows(raw) {
        m.entry(row.pin).or_default().push(AfCap {
            peripheral: row.signal.peripheral.to_string(),
            role: row.signal.role.to_string(),
            af: row.af,
        });
    }
    m
}

fn class_of(peripheral: &str) -> &str {
    peripheral.trim_end_matches(|c: char| c.is_ascii_digit())
}

/// The logical kind (USART/UART/LPUART -> SERIAL) if modeled, else the bare
/// peripheral class (TIM, COMP, FDCAN…). Owned to avoid mixing `&'static` and
/// borrowed lifetimes when comparing two peripherals.
fn kind_or_class(peripheral: &str) -> String {
    crate::select::kind_of(peripheral)
        .map(str::to_string)
        .unwrap_or_else(|| class_of(peripheral).to_string())
}

fn is_analog_class(class: &str) -> bool {
    matches!(class, "ADC" | "COMP" | "OPAMP" | "DAC")
}

/// The tier at which a candidate signal serves a wired one, or `None`. Analog
/// signals (no AF, or an analog source class) require exact instance+role
/// regardless of `mode` — fixed silicon can't be re-muxed.
fn serve_tier(
    want: &OwnedSignal,
    cap_peri: &str,
    cap_role: &str,
    cap_af: Option<u8>,
    mode: Strictness,
) -> Option<MatchTier> {
    if want.peripheral == cap_peri && want.role == cap_role {
        return Some(MatchTier::Exact);
    }
    let analog = cap_af.is_none() || is_analog_class(class_of(&want.peripheral));
    if analog {
        return None;
    }
    let same_class = kind_or_class(&want.peripheral) == kind_or_class(cap_peri);
    match mode {
        Strictness::ExactInstanceRole => None,
        Strictness::ClassRole => {
            (same_class && want.role == cap_role).then_some(MatchTier::ClassRole)
        }
        Strictness::ClassOnly => {
            if !same_class {
                None
            } else if want.role == cap_role {
                Some(MatchTier::ClassRole)
            } else {
                Some(MatchTier::ClassOnly)
            }
        }
    }
}

/// Evaluate one candidate footprint against the source. Pure over its inputs
/// (the candidate's AF oracle is passed in), so it is fully unit-testable without
/// the descriptor asset.
fn evaluate_candidate(
    src: &SourceProfile,
    cand_name: &str,
    cand_rec: &PinoutRecord,
    cand_af: &HashMap<PinId, Vec<AfCap>>,
    cfg: MatchConfig,
) -> CandidateMatch {
    let by_pin = src.wired_by_pin();
    let mut positions: Vec<PosResult> = Vec::with_capacity(src.footprint.pins.len());

    for sp in &src.footprint.pins {
        let pos = sp.p.clone();
        let cand_fn = cand_rec.at(&sp.p).map(|p| p.function());
        let (kind, outcome, detail) = match sp.function() {
            PinFunction::Power(want_rails) => {
                // EQUALITY, not subset: a candidate that fuses extra rails onto
                // this position (e.g. VDD/VDDA where the source carried only VDD,
                // keeping VDDA on a separate net elsewhere) would bridge two
                // distinct board nets through the part — a short, not a drop-in.
                let outcome = match &cand_fn {
                    Some(PinFunction::Power(have)) if *have == want_rails => PosOutcome::PowerOk,
                    _ => PosOutcome::PowerMismatch,
                };
                let detail = format!("power {want_rails:?} → {}", describe(&cand_fn));
                (SourceKind::Power, outcome, detail)
            }
            PinFunction::Gpio(pins) => {
                let wired_here: Vec<&WiredFn> =
                    pins.iter().filter_map(|p| by_pin.get(p).copied()).collect();
                if wired_here.is_empty() {
                    // Unused source GPIO.
                    let outcome = match &cand_fn {
                        Some(PinFunction::Gpio(_)) => PosOutcome::UnusedOk,
                        Some(PinFunction::Power(_)) => match cfg.power_mode {
                            PowerMode::Ignore => PosOutcome::UnusedOk,
                            _ => PosOutcome::PowerAsym,
                        },
                        _ => PosOutcome::UnusedRepurposed,
                    };
                    let detail = format!("unused {} → {}", pins[0].name(), describe(&cand_fn));
                    (SourceKind::UnusedGpio, outcome, detail)
                } else {
                    // Allocated: EVERY wired function on this physical position
                    // must be serviceable on the candidate. (A merged ball is one
                    // net, so this is normally a single function — but if the user
                    // locked several, all are required.)
                    let cand_pins: &[PinId] = match &cand_fn {
                        Some(PinFunction::Gpio(cps)) => cps,
                        _ => &[],
                    };
                    let mut details: Vec<String> = Vec::new();
                    let mut worst: Option<MatchTier> = Some(MatchTier::Exact);
                    for w in &wired_here {
                        let mut best: Option<(MatchTier, String, String, PinId)> = None;
                        for cp in cand_pins {
                            for cap in cand_af.get(cp).into_iter().flatten() {
                                if let Some(t) = serve_tier(
                                    &w.sig,
                                    &cap.peripheral,
                                    &cap.role,
                                    cap.af,
                                    cfg.strictness,
                                ) && best.as_ref().is_none_or(|(b, ..)| t < *b)
                                {
                                    best = Some((t, cap.peripheral.clone(), cap.role.clone(), *cp));
                                }
                            }
                        }
                        match best {
                            Some((tier, cperi, crole, cpin)) => {
                                details.push(format!(
                                    "{}.{} → {cperi}.{crole} @ {}",
                                    w.sig.peripheral,
                                    w.sig.role,
                                    cpin.name()
                                ));
                                worst = worst.map(|b| b.max(tier));
                            }
                            None => {
                                worst = None;
                                details.push(format!(
                                    "{}.{} not serviceable → {}",
                                    w.sig.peripheral,
                                    w.sig.role,
                                    describe(&cand_fn)
                                ));
                            }
                        }
                    }
                    let outcome = match worst {
                        Some(MatchTier::Exact) => PosOutcome::ServiceableExact,
                        Some(MatchTier::ClassRole) => PosOutcome::ServiceableClassRole,
                        Some(MatchTier::ClassOnly) => PosOutcome::ServiceableClassOnly,
                        None => PosOutcome::FunctionFail,
                    };
                    (SourceKind::Allocated, outcome, details.join("; "))
                }
            }
            PinFunction::Dedicated(tokens) => {
                let outcome = match &cand_fn {
                    Some(PinFunction::Dedicated(ct)) => {
                        let mut a = tokens.clone();
                        let mut b = ct.clone();
                        a.sort();
                        b.sort();
                        if a == b {
                            PosOutcome::DedicatedOk
                        } else {
                            PosOutcome::DedicatedMismatch
                        }
                    }
                    Some(PinFunction::Gpio(_)) | Some(PinFunction::Power(_)) => {
                        PosOutcome::DedicatedMismatch
                    }
                    _ => PosOutcome::DedicatedUnverified,
                };
                let detail = format!("{} → {}", tokens.join("/"), describe(&cand_fn));
                (SourceKind::Dedicated, outcome, detail)
            }
            PinFunction::Nc => (SourceKind::Nc, PosOutcome::DontCare, String::new()),
        };
        positions.push(PosResult { position: pos, kind, outcome, detail });
    }

    let same_family = src.footprint.fam == cand_rec.fam;
    // Reset/boot net is only unverifiable across families when a side omits the
    // tokens; within a family the (unlabeled) reset pin position is consistent.
    let reset_boot_unverified = !same_family
        && (!src.footprint.has_reset_boot() || !cand_rec.has_reset_boot());

    // Footprint gate: a genuine drop-in has the IDENTICAL set of physical
    // positions (ignoring NC). The per-position loop above only visits SOURCE
    // positions, so it never sees a candidate position the source lacks —
    // crucially the candidate's POWER pins. Without this, a source whose data
    // omits its power pads (stm32-data drops all C5 power/ground bonds, so a C5
    // "LQFP64" carries 52 positions, 0 of them power) would vacuously accept
    // every full-bond foreign part as compatible. Comparing the non-NC position
    // SETS rejects any size/placement difference in both directions.
    let src_positions: BTreeSet<&str> = src
        .footprint
        .pins
        .iter()
        .filter(|p| !matches!(p.function(), PinFunction::Nc))
        .map(|p| p.p.as_str())
        .collect();
    let cand_positions: BTreeSet<&str> = cand_rec
        .pins
        .iter()
        .filter(|p| !matches!(p.function(), PinFunction::Nc))
        .map(|p| p.p.as_str())
        .collect();
    let footprint_match = src_positions == cand_positions;

    let n_required = positions
        .iter()
        .filter(|p| matches!(p.kind, SourceKind::Power | SourceKind::Allocated))
        .count() as u16;
    let n_satisfied = positions.iter().filter(|p| p.outcome.is_satisfied()).count() as u16;
    let warnings = positions.iter().filter(|p| p.outcome == PosOutcome::PowerAsym).count() as u16;
    let compatible =
        footprint_match && !positions.iter().any(|p| p.outcome.is_blocker(cfg.power_mode));

    let count = |o: &PosOutcome| positions.iter().filter(|p| &p.outcome == o).count() as i64;
    let score = 1000 * count(&PosOutcome::ServiceableExact)
        + 400 * count(&PosOutcome::ServiceableClassRole)
        + 100 * count(&PosOutcome::ServiceableClassOnly)
        - 50 * i64::from(warnings)
        - 30 * i64::from(reset_boot_unverified)
        + if same_family { 200 } else { 0 };

    CandidateMatch {
        name: cand_name.to_string(),
        family: cand_rec.fam.clone(),
        footprint: cand_rec.pkg.clone(),
        compatible,
        same_family,
        reset_boot_unverified,
        n_satisfied,
        n_required,
        warnings,
        score,
        positions,
    }
}

/// Short label for a candidate position's function (for detail strings).
fn describe(f: &Option<PinFunction>) -> String {
    match f {
        None => "absent".to_string(),
        Some(PinFunction::Gpio(p)) => {
            p.iter().map(|x| x.name()).collect::<Vec<_>>().join("/")
        }
        Some(PinFunction::Power(r)) => format!("{r:?}"),
        Some(PinFunction::Dedicated(t)) => t.join("/"),
        Some(PinFunction::Nc) => "NC".to_string(),
    }
}

/// Find every drop-in replacement candidate for the source, sorted compatible-
/// first then by closeness score. Candidates are package-letter-prefix parts of
/// the *same land-pattern footprint*, excluding the source's own prefix.
pub fn find_replacements(src: &SourceProfile, cfg: MatchConfig) -> Vec<CandidateMatch> {
    let src_prefix = if src.name.len() >= 10 { &src.name[..10] } else { src.name.as_str() };
    // An unparseable source footprint name cannot be land-pattern-gated, and
    // `None != None` is false, so comparing raw Options would silently admit
    // every other unparseable package. Bail instead.
    let Some(src_class) = package_class(&src.footprint.pkg) else {
        return Vec::new();
    };

    let mut seen: BTreeSet<(String, String)> = BTreeSet::new();
    let mut out: Vec<CandidateMatch> = Vec::new();

    for f in phys_pinout::index() {
        if f.name.len() < 10 {
            continue;
        }
        let prefix = &f.name[..10];
        if prefix == src_prefix {
            continue; // same part letter — not a "replacement"
        }
        if !seen.insert((prefix.to_string(), f.pkg.clone())) {
            continue; // dedup flash variants of one (prefix, footprint)
        }
        if package_class(&f.pkg) != Some(src_class) {
            continue; // different land pattern — never a board drop-in
        }
        let Some(rec) = phys_pinout::record(&f.h) else { continue };
        // The candidate's per-pin capability oracle (logical AF table). Absent
        // descriptor => empty oracle => allocated positions fail (sound), but
        // power/footprint matching still works.
        let af = descriptor_for(&f.name).map(|d| build_af_oracle(d.raw)).unwrap_or_default();
        out.push(evaluate_candidate(src, &f.name, rec, &af, cfg));
    }

    out.sort_by(|a, b| {
        b.compatible
            .cmp(&a.compatible)
            .then(b.score.cmp(&a.score))
            .then(a.name.cmp(&b.name))
    });
    out
}

// ---------- Cached query (UI binds here) ----------

/// The drop-in finder query: matching config + optional result narrowing.
#[derive(Clone, Debug, PartialEq)]
pub struct DropinQuery {
    pub strictness: Strictness,
    pub power_mode: PowerMode,
    pub family: Option<String>,
    pub same_family_only: bool,
}

impl Default for DropinQuery {
    fn default() -> Self {
        Self {
            strictness: Strictness::ClassRole,
            power_mode: PowerMode::Warn,
            family: None,
            same_family_only: false,
        }
    }
}

impl DropinQuery {
    pub fn config(&self) -> MatchConfig {
        MatchConfig { strictness: self.strictness, power_mode: self.power_mode }
    }
}

/// Memoizes the lineup-wide scan; reruns only when the source allocation or the
/// query changes (mirrors `select::EvalCache`).
#[derive(Default)]
pub struct DropinCache {
    key: Option<(String, DropinQuery)>,
    results: Vec<CandidateMatch>,
}

impl DropinCache {
    pub fn results(&mut self, src: &SourceProfile, q: &DropinQuery) -> &[CandidateMatch] {
        let fp = src.fingerprint();
        let hit = self.key.as_ref().is_some_and(|(f, qq)| *f == fp && qq == q);
        if !hit {
            let mut r = find_replacements(src, q.config());
            if let Some(fam) = &q.family {
                r.retain(|c| &c.family == fam);
            }
            if q.same_family_only {
                r.retain(|c| c.same_family);
            }
            self.results = r;
            self.key = Some((fp, q.clone()));
        }
        &self.results
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::phys_pinout::PhysPin;

    fn pin(s: &str) -> PinId {
        PinId::from_metapac(s).unwrap()
    }
    fn sig(p: &str, r: &str) -> OwnedSignal {
        OwnedSignal { peripheral: p.to_string(), role: r.to_string() }
    }
    fn ph(pos: &str, sigs: &[&str]) -> PhysPin {
        PhysPin { p: pos.to_string(), s: sigs.iter().map(|x| x.to_string()).collect() }
    }
    fn rec(fam: &str, pkg: &str, pins: Vec<PhysPin>) -> PinoutRecord {
        let n = pins.len() as u16;
        PinoutRecord {
            h: format!("{fam}:{pkg}:{n}"),
            pkg: pkg.to_string(),
            names: vec![pkg.to_string()],
            fam: fam.to_string(),
            n,
            pins,
        }
    }
    fn oracle(rows: &[(&str, &str, &str, Option<u8>)]) -> HashMap<PinId, Vec<AfCap>> {
        let mut m: HashMap<PinId, Vec<AfCap>> = HashMap::new();
        for (p, peri, role, af) in rows {
            m.entry(pin(p)).or_default().push(AfCap {
                peripheral: peri.to_string(),
                role: role.to_string(),
                af: *af,
            });
        }
        m
    }

    #[test]
    fn kind_of_is_consistent_with_underlying_classes() {
        for kind in ["SERIAL", "SPI", "I2C", "ADC", "UCPD"] {
            for class in crate::select::underlying_classes(kind) {
                let inst = format!("{class}1");
                assert_eq!(
                    crate::select::kind_of(&inst),
                    Some(kind),
                    "{inst} should map back to {kind}"
                );
            }
        }
    }

    #[test]
    fn serve_tier_truth_table() {
        let want = sig("USART2", "TX");
        // Exact wins in every mode.
        assert_eq!(
            serve_tier(&want, "USART2", "TX", Some(7), Strictness::ExactInstanceRole),
            Some(MatchTier::Exact)
        );
        // Class+role: a UART TX serves under ClassRole, not under Exact.
        assert_eq!(
            serve_tier(&want, "UART4", "TX", Some(8), Strictness::ExactInstanceRole),
            None
        );
        assert_eq!(
            serve_tier(&want, "UART4", "TX", Some(8), Strictness::ClassRole),
            Some(MatchTier::ClassRole)
        );
        // Wrong role under ClassRole fails; under ClassOnly it's ClassOnly tier.
        assert_eq!(serve_tier(&want, "UART4", "RX", Some(8), Strictness::ClassRole), None);
        assert_eq!(
            serve_tier(&want, "UART4", "RX", Some(8), Strictness::ClassOnly),
            Some(MatchTier::ClassOnly)
        );
        // Wrong class never matches.
        assert_eq!(serve_tier(&want, "SPI1", "MOSI", Some(5), Strictness::ClassOnly), None);
    }

    #[test]
    fn analog_signal_requires_exact_regardless_of_strictness() {
        let want = sig("ADC1", "IN5");
        // Different ADC instance, even role-equal, never matches under any mode
        // (analog is fixed silicon).
        for mode in Strictness::ALL {
            assert_eq!(serve_tier(&want, "ADC2", "IN5", None, mode), None);
            assert_eq!(serve_tier(&want, "ADC1", "IN5", None, mode), Some(MatchTier::Exact));
        }
        // A digital candidate cap with no AF is also treated as analog (exact only).
        let dwant = sig("USART2", "TX");
        assert_eq!(serve_tier(&dwant, "UART4", "TX", None, Strictness::ClassRole), None);
    }

    /// A G4-like LQFP4 source: pos1=VDD, pos2=PA9 wired to USART1.TX, pos3=VSS,
    /// pos4=PB0 unused. A same-family candidate that keeps power and offers a
    /// UART TX on pos2 is compatible under ClassRole.
    fn src_4pin() -> SourceProfile {
        let footprint = Box::leak(Box::new(rec(
            "STM32G4",
            "LQFP4",
            vec![ph("1", &["VDD"]), ph("2", &["PA9"]), ph("3", &["VSS"]), ph("4", &["PB0"])],
        )));
        SourceProfile {
            name: "STM32G4XXRE".to_string(),
            footprint,
            wired: vec![WiredFn { sig: sig("USART1", "TX"), pin: pin("PA9") }],
        }
    }

    #[test]
    fn compatible_when_power_holds_and_function_served() {
        let src = src_4pin();
        let cand = rec(
            "STM32G4",
            "LQFP4",
            vec![ph("1", &["VDD"]), ph("2", &["PA9"]), ph("3", &["VSS"]), ph("4", &["PB0"])],
        );
        // Candidate offers LPUART1.TX on PA9 (the wired position).
        let af = oracle(&[("PA9", "LPUART1", "TX", Some(7))]);
        let m = evaluate_candidate(&src, "STM32G4YYRE", &cand, &af, MatchConfig::default());
        assert!(m.compatible, "{:?}", m.positions);
        assert_eq!(m.n_satisfied, m.n_required); // 1 power-VDD? no: 2 power + 1 alloc = 3
        // Served as class+role (USART1.TX -> LPUART1.TX).
        assert!(m.positions.iter().any(|p| p.outcome == PosOutcome::ServiceableClassRole));
    }

    #[test]
    fn power_mismatch_is_a_hard_fail() {
        let src = src_4pin();
        // Candidate puts a GPIO where the source has VDD (pos1).
        let cand = rec(
            "STM32H5",
            "LQFP4",
            vec![ph("1", &["PC1"]), ph("2", &["PA9"]), ph("3", &["VSS"]), ph("4", &["PB0"])],
        );
        let af = oracle(&[("PA9", "USART1", "TX", Some(7)), ("PC1", "ADC1", "IN1", None)]);
        let m = evaluate_candidate(&src, "STM32H5YYRE", &cand, &af, MatchConfig::default());
        assert!(!m.compatible);
        assert!(m.positions.iter().any(|p| p.outcome == PosOutcome::PowerMismatch));
    }

    #[test]
    fn unserviceable_allocation_is_a_hard_fail() {
        let src = src_4pin();
        let cand = rec(
            "STM32G4",
            "LQFP4",
            vec![ph("1", &["VDD"]), ph("2", &["PA9"]), ph("3", &["VSS"]), ph("4", &["PB0"])],
        );
        // Candidate's PA9 only does SPI — no serial TX. Hard fail.
        let af = oracle(&[("PA9", "SPI1", "MOSI", Some(5))]);
        let m = evaluate_candidate(&src, "STM32G4ZZRE", &cand, &af, MatchConfig::default());
        assert!(!m.compatible);
        assert!(m.positions.iter().any(|p| p.outcome == PosOutcome::FunctionFail));
    }

    #[test]
    fn coverage_missing_required_position_fails() {
        let src = src_4pin();
        // Candidate omits pos3 (VSS) entirely — a partial-bond footprint.
        let cand = rec(
            "STM32G4",
            "LQFP4",
            vec![ph("1", &["VDD"]), ph("2", &["PA9"]), ph("4", &["PB0"])],
        );
        let af = oracle(&[("PA9", "USART1", "TX", Some(7))]);
        let m = evaluate_candidate(&src, "STM32G4PP RE", &cand, &af, MatchConfig::default());
        assert!(!m.compatible, "missing VSS must fail coverage");
        assert!(m.positions.iter().any(|p| p.outcome == PosOutcome::PowerMismatch));
    }

    #[test]
    fn power_asymmetry_respects_power_mode() {
        let src = src_4pin();
        // pos4 (source unused PB0) is VDD on the candidate.
        let cand = rec(
            "STM32G4",
            "LQFP4",
            vec![ph("1", &["VDD"]), ph("2", &["PA9"]), ph("3", &["VSS"]), ph("4", &["VDD"])],
        );
        let af = oracle(&[("PA9", "USART1", "TX", Some(7))]);

        let warn = evaluate_candidate(
            &src,
            "c",
            &cand,
            &af,
            MatchConfig { strictness: Strictness::ClassRole, power_mode: PowerMode::Warn },
        );
        assert!(warn.compatible);
        assert_eq!(warn.warnings, 1);

        let reject = evaluate_candidate(
            &src,
            "c",
            &cand,
            &af,
            MatchConfig { strictness: Strictness::ClassRole, power_mode: PowerMode::Reject },
        );
        assert!(!reject.compatible);

        let ignore = evaluate_candidate(
            &src,
            "c",
            &cand,
            &af,
            MatchConfig { strictness: Strictness::ClassRole, power_mode: PowerMode::Ignore },
        );
        assert!(ignore.compatible);
        assert_eq!(ignore.warnings, 0);
    }

    #[test]
    fn fused_rail_superset_is_rejected_as_short() {
        // Source keeps VDD (pos1) and VDDA (pos3) on SEPARATE board nets.
        let footprint = Box::leak(Box::new(rec(
            "STM32G4",
            "LQFP4",
            vec![ph("1", &["VDD"]), ph("2", &["PA9"]), ph("3", &["VDDA"]), ph("4", &["PB0"])],
        )));
        let src = SourceProfile { name: "STM32G4XXRE".to_string(), footprint, wired: vec![] };
        // Candidate FUSES VDD/VDDA onto pos1 and pos3 — dropping it in would short
        // the source's separate VDD and VDDA nets together. Must be rejected (the
        // power gate is rail-set EQUALITY, not subset).
        let cand = rec(
            "STM32C0",
            "LQFP4",
            vec![
                ph("1", &["VDD/VDDA"]),
                ph("2", &["PA9"]),
                ph("3", &["VDD/VDDA"]),
                ph("4", &["PB0"]),
            ],
        );
        let af = oracle(&[]);
        let m = evaluate_candidate(&src, "STM32C0", &cand, &af, MatchConfig::default());
        assert!(!m.compatible, "fused-rail candidate must not be a drop-in: {:?}", m.positions);
        assert!(m.positions.iter().any(|p| p.outcome == PosOutcome::PowerMismatch));
    }

    #[test]
    fn partial_bond_source_rejects_fuller_foreign_footprint() {
        // A C5-like source whose data omits its power pads: 2 GPIO positions, NO
        // power. The per-position loop has nothing to gate, so without the
        // footprint position-set gate every fuller foreign part would vacuously
        // pass. The candidate is a full-bond foreign LQFP with power pins the
        // source lacks — it must be rejected.
        let footprint = Box::leak(Box::new(rec(
            "STM32C5",
            "LQFP4",
            vec![ph("2", &["PA0"]), ph("3", &["PA1"])],
        )));
        let src = SourceProfile { name: "STM32C5XXR0".to_string(), footprint, wired: vec![] };
        let cand = rec(
            "STM32F1",
            "LQFP4",
            vec![ph("1", &["VDD"]), ph("2", &["PA0"]), ph("3", &["PA1"]), ph("4", &["VSS"])],
        );
        let af = oracle(&[]);
        let m = evaluate_candidate(&src, "STM32F103", &cand, &af, MatchConfig::default());
        assert!(
            !m.compatible,
            "a fuller foreign footprint must not be a drop-in for a partial-bond source"
        );
        // And a same-position-set sibling (also power-omitted) IS compatible.
        let sib = rec("STM32C5", "LQFP4", vec![ph("2", &["PA0"]), ph("3", &["PA1"])]);
        let m2 = evaluate_candidate(&src, "STM32C5YY", &sib, &af, MatchConfig::default());
        assert!(m2.compatible, "a same-footprint C5 sibling should be compatible: {:?}", m2.positions);
    }

    // ---------- Integration over the real lineup asset ----------

    #[test]
    fn real_g474_source_finds_g4_replacements_not_cross_family() {
        // Source: G474RE in LQFP64, wiring USART1.TX on PA9 (a real G4 AF).
        let footprint = phys_pinout::footprints_for("STM32G474RE")
            .iter()
            .copied()
            .find(|r| r.pkg == "LQFP64")
            .expect("G474RE LQFP64");
        // Sanity: PA9 is a real physical position on this footprint.
        assert!(footprint.pins.iter().any(|p| p.s.iter().any(|s| s == "PA9")));

        let src = SourceProfile {
            name: "STM32G474RE".to_string(),
            footprint,
            wired: vec![WiredFn { sig: sig("USART1", "TX"), pin: pin("PA9") }],
        };
        let results = find_replacements(&src, MatchConfig::default());
        assert!(!results.is_empty());

        // Every result shares the LQFP land-pattern (footprint gate).
        for c in &results {
            assert_eq!(package_class(&c.footprint), package_class("LQFP64"));
        }
        // At least one compatible G4 replacement exists...
        assert!(
            results.iter().any(|c| c.compatible && c.family == "STM32G4"),
            "expected a compatible G4 LQFP64 drop-in"
        );
        // ...and no H5 part is ever compatible (its LQFP64 power map diverges).
        assert!(
            !results.iter().any(|c| c.compatible && c.family == "STM32H5"),
            "an H5 LQFP64 must never be a compatible drop-in for a G4"
        );
        // The source's own prefix is never suggested.
        assert!(!results.iter().any(|c| c.name.starts_with("STM32G474R")));
    }

    #[test]
    fn c531_source_resolves_and_never_false_accepts_foreign_families() {
        // Regression for the critical false-accept: a C5 source's data omits all
        // power pads (C531 LQFP64 = 52 positions, 0 power), so the per-position
        // gates have nothing to check. The footprint position-set gate must still
        // reject every fuller foreign-family LQFP64. Also exercises the suffix-
        // tolerant source resolution (package.name() is "STM32C531RCT6").
        let d = crate::c531_design::C531Design::new();
        let profile = build_source_profile(&DesignSource::C531(&d), Package::C531R)
            .expect("C531R footprint must resolve despite the ordering-code suffix");
        assert_eq!(profile.footprint.pkg, "LQFP64");

        let results = find_replacements(&profile, MatchConfig::default());
        let foreign: Vec<&str> = results
            .iter()
            .filter(|c| c.compatible && c.family != "STM32C5")
            .map(|c| c.name.as_str())
            .collect();
        assert!(
            foreign.is_empty(),
            "no foreign-family part may be a compatible drop-in for a power-omitted C5 source; got {:?}",
            &foreign[..foreign.len().min(10)]
        );
    }
}
