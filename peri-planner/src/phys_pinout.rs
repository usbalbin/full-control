//! Lineup-wide **physical pinout** asset: per-footprint `position -> signal(s)`,
//! including power/ground rails — the data layer the drop-in / pin-compatibility
//! finder stands on. Generated from raw stm32-data `packages[].pins[]` by the
//! `gen_pinouts` tool and parsed on demand, exactly like the logical descriptor
//! asset (`desc_asset`).
//!
//! Why a separate asset (not the descriptor): the descriptor is *logical only*
//! (`PA0 -> USART1.TX`) and is keyed by package *letter*, collapsing parts that
//! ship in several footprints (e.g. STM32G474V = LQFP100 **and** TFBGA100). A
//! drop-in replacement check is *physical*: it compares what each datasheet pin
//! position carries (a GPIO, a power rail, a reset pin), so it needs every
//! footprint's full position map. That lives only in the raw stm32-data
//! `packages[]`, never in the descriptor.
//!
//! Design: the asset is a **dumb mirror** of stm32-data — each position stores
//! its raw signal token(s) verbatim. All the soundness-critical interpretation
//! (GPIO vs power vs dedicated, canonical rail normalization, merged-ball
//! handling) is done here at lookup time by [`PinFunction::classify`], in one
//! place, fully unit-tested. The generator only hashes + deduplicates.
//!
//! Two empirically-verified hazards this module exists to handle correctly
//! (found by sampling the data before the design was fixed):
//!   * The same package **name** is **not** the same footprint — STM32G474 /
//!     H523 / C531 all report `"LQFP64"` but their position maps diverge in 55
//!     of 64 slots. So the package string is never a match key; soundness comes
//!     from per-position gates over this data (see `dropin`).
//!   * Power-rail **spellings** vary by family (`VDD/VDDA` fused, `VDDA/VREF+`,
//!     `VSS/VSSA`). Rails are normalized to a canonical [`Rail`] set and matched
//!     as sets, never by raw string — see [`Rail::parse`].

use std::collections::{BTreeSet, HashMap};
use std::io::Read as _;
use std::sync::LazyLock;

use serde::{Deserialize, Serialize};

use crate::mcu_pinout::PinId;

/// One physical position and the raw signal token(s) bonded to it, verbatim from
/// stm32-data `packages[].pins[]`. Usually one token (`["PA0"]` or `["VDD"]`),
/// but small packages merge several GPIOs onto one ball (`["PB3","PB4","PB5"]`)
/// and some rails are fused onto one pin (`["VDD/VDDA"]`).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PhysPin {
    /// Package position string: QFP/QFN sequential (`"1".."176"`) or BGA grid
    /// (`"A1"`, `"K9"`). Compared **verbatim** as a key — never parsed or sorted
    /// for matching (`"A10" < "A2"` lexically would mis-order a BGA).
    pub p: String,
    /// Signal token(s) on this position. Length is usually 1.
    pub s: Vec<String>,
}

/// One distinct physical pinout, deduplicated across every part that shares it.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PinoutRecord {
    /// Content hash over the sorted `(position, sorted-signals)` set — the dedup
    /// key and stable identity that links an [`PartFootprint`] to its pinout.
    pub h: String,
    /// Representative datasheet package name, e.g. `"LQFP64"`. Display only —
    /// several names can share one pinout (see `names`); never a match key.
    pub pkg: String,
    /// Every datasheet package name that resolves to this exact pinout
    /// (e.g. `["LQFP64","LQFP64_GP"]`). Footprint identity is pinout identity,
    /// so these names are interchangeable physical layouts.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub names: Vec<String>,
    /// Representative family, e.g. `"STM32G4"` (display / diagnostic only).
    pub fam: String,
    /// Bonded-pin count (`== pins.len()`). A `"LQFP64"` that bonds only 52 pins
    /// (some C5 lines) has `n == 52`: it is *not* the same footprint as a full
    /// 64-pin part — the coverage gate in `dropin` relies on this.
    pub n: u16,
    pub pins: Vec<PhysPin>,
}

/// `part name -> one footprint it ships in`. One row per chip-package; a part
/// offered in several packages has several rows. The catalog-part -> physical-
/// pinout bridge (joins on the same part `name` the catalog and `descriptor_for`
/// already use).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PartFootprint {
    pub name: String,
    pub pkg: String,
    pub h: String,
}

/// The whole asset: a deduplicated pool of distinct pinouts plus a part-name
/// index into it.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct PinoutAsset {
    pub records: Vec<PinoutRecord>,
    pub index: Vec<PartFootprint>,
}

/// A canonical power/ground rail, normalized from the family-variant token
/// spellings (`VDD/VDDA` fused, `VDDA/VREF+`, …). A fused token expands to a
/// *set* of these. The set is deliberately **conservative**: a token we are not
/// certain maps to one of these rails is classified [`PinFunction::Dedicated`],
/// never silently treated as a rail. Two electrically-distinct rails must never
/// map to the same variant (that would let the power gate false-accept).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Rail {
    Vdd,
    Vss,
    Vdda,
    Vssa,
    Vbat,
    VrefP,
    VrefN,
    Vcap,
    VddUsb,
    VddIo2,
    VddSmps,
    Vlcd,
}

impl Rail {
    /// Parse a single rail token to its canonical rail. `None` for any token we
    /// do not positively recognize as a power rail — the caller then classifies
    /// the position as [`PinFunction::Dedicated`] (conservative: errs toward
    /// rejecting a match, never toward a false power-pin equivalence).
    pub fn parse(tok: &str) -> Option<Rail> {
        Some(match tok {
            "VDD" => Rail::Vdd,
            "VSS" | "GND" => Rail::Vss,
            "VDDA" => Rail::Vdda,
            "VSSA" => Rail::Vssa,
            "VBAT" => Rail::Vbat,
            "VREF+" | "VREFP" | "VREF_P" => Rail::VrefP,
            "VREF-" | "VREFN" | "VREF_N" => Rail::VrefN,
            // Multiple decoupling pins of the *same* core-LDO rail — safe to unify.
            "VCAP" | "VCAP1" | "VCAP2" | "VCAP_1" | "VCAP_2" => Rail::Vcap,
            "VDDUSB" | "VDD_USB" => Rail::VddUsb,
            "VDDIO2" => Rail::VddIo2,
            "VDDSMPS" | "VDD_SMPS" => Rail::VddSmps,
            "VLCD" => Rail::Vlcd,
            // Everything else (VDD12/VDD18 core voltages, VDD33_USB/VDD50_USB
            // voltage-distinct rails, unknown tokens) -> not a canonical rail.
            _ => return None,
        })
    }
}

/// What the data says a physical position carries, resolved from its raw signal
/// token set. Kept as distinct classes so the matcher never compares across
/// classes (a GPIO is never "served" by a power pin, etc.).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PinFunction {
    /// One or more AF-muxable GPIOs bonded to this position. A position is
    /// GPIO-serviceable only if the required logical pin is in this set.
    Gpio(Vec<PinId>),
    /// One or more power/ground rails bonded to this position (a fused pin
    /// carries several). Stored sorted+deduped.
    Power(Vec<Rail>),
    /// A named non-GPIO, non-power dedicated pin (NRST, BOOT0, PDR_ON, OSC_IN…)
    /// **or** any unrecognized / mixed token set. Matched token-for-token only.
    Dedicated(Vec<String>),
    /// No signal bonded to the position (NC).
    Nc,
}

impl PinFunction {
    /// Classify a raw signal token set into one [`PinFunction`]. The single place
    /// the data's quirks are interpreted:
    ///   * all tokens are GPIO names (`P[A-Z][0-9]+`) -> `Gpio` (handles merged
    ///     balls);
    ///   * all tokens are canonical rails (`/`-fused tokens split first) ->
    ///     `Power`;
    ///   * empty -> `Nc`;
    ///   * anything else (a dedicated pin, an unknown token, or a GPIO/power mix)
    ///     -> `Dedicated`, which only ever matches an identical token set
    ///     (conservative — cannot false-accept a GPIO or a rail).
    pub fn classify(tokens: &[String]) -> PinFunction {
        if tokens.is_empty() {
            return PinFunction::Nc;
        }
        // Try all-GPIO.
        let gpios: Option<Vec<PinId>> = tokens.iter().map(|t| PinId::from_metapac(t)).collect();
        if let Some(mut pins) = gpios {
            pins.sort();
            pins.dedup();
            return PinFunction::Gpio(pins);
        }
        // Try all-power (each token may be a `/`-fused rail group).
        let mut rails: BTreeSet<Rail> = BTreeSet::new();
        let mut all_rails = true;
        for tok in tokens {
            for part in tok.split('/') {
                match Rail::parse(part) {
                    Some(r) => {
                        rails.insert(r);
                    }
                    None => {
                        all_rails = false;
                        break;
                    }
                }
            }
            if !all_rails {
                break;
            }
        }
        if all_rails && !rails.is_empty() {
            return PinFunction::Power(rails.into_iter().collect());
        }
        // Dedicated / unknown / mixed: keep raw tokens, match only verbatim.
        PinFunction::Dedicated(tokens.to_vec())
    }
}

impl PhysPin {
    /// Classify this position's signal tokens.
    pub fn function(&self) -> PinFunction {
        PinFunction::classify(&self.s)
    }
}

impl PinoutRecord {
    /// The set of physical position strings on this footprint.
    pub fn positions(&self) -> BTreeSet<&str> {
        self.pins.iter().map(|p| p.p.as_str()).collect()
    }

    /// The [`PhysPin`] at a position string, if present (exact key match).
    pub fn at(&self, position: &str) -> Option<&PhysPin> {
        self.pins.iter().find(|p| p.p == position)
    }

    /// Whether this footprint carries any dedicated reset/boot tokens. When a
    /// footprint omits them (G4 / C5 data does), the drop-in check cannot verify
    /// the reset/boot net and must say so rather than claim a clean pass.
    pub fn has_reset_boot(&self) -> bool {
        self.pins.iter().any(|p| {
            p.s.iter()
                .any(|t| matches!(t.as_str(), "NRST" | "BOOT0" | "BOOT" | "PDR_ON"))
        })
    }
}

/// Split a datasheet package name into a land-pattern class + pin count, e.g.
/// `"LQFP64"` -> `("LQFP", 64)`, `"LQFP64_GP"` -> `("LQFP", 64)`,
/// `"UFQFPN48"` -> `("UFQFPN", 48)`, `"TFBGA100"` -> `("TFBGA", 100)`.
///
/// The marketing suffix (`_GP`, `_N`, …) is stripped — those denote pinout
/// variants of the *same* land pattern (the per-position gates catch any real
/// pin difference), so they are interchangeable footprints for candidate
/// enumeration. The package *kind* (LQFP vs UFQFPN vs TFBGA vs WLCSP) is kept
/// distinct: a QFN is never a board-level drop-in for a QFP even at equal pin
/// count. Returns `None` if the name has no leading-alpha / trailing-digit shape.
pub fn package_class(pkg: &str) -> Option<(&str, u16)> {
    let kind_end = pkg.find(|c: char| c.is_ascii_digit())?;
    let (kind, rest) = pkg.split_at(kind_end);
    if kind.is_empty() {
        return None;
    }
    let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
    let count: u16 = digits.parse().ok()?;
    Some((kind, count))
}

// ---------- Runtime loader ----------

/// The parsed asset, inflated once from the embedded gzip.
fn asset() -> &'static PinoutAsset {
    static A: LazyLock<PinoutAsset> = LazyLock::new(|| {
        let gz: &[u8] = include_bytes!("../assets/pinouts.json.gz");
        let mut json = Vec::new();
        flate2::read::GzDecoder::new(gz)
            .read_to_end(&mut json)
            .expect("inflate pinouts.json.gz");
        serde_json::from_slice(&json).expect("parse pinouts.json.gz")
    });
    &A
}

/// `hash -> record`.
fn by_hash() -> &'static HashMap<&'static str, &'static PinoutRecord> {
    static M: LazyLock<HashMap<&'static str, &'static PinoutRecord>> = LazyLock::new(|| {
        asset().records.iter().map(|r| (r.h.as_str(), r)).collect()
    });
    &M
}

/// `part name -> its footprints`.
fn by_name() -> &'static HashMap<&'static str, Vec<&'static PinoutRecord>> {
    static M: LazyLock<HashMap<&'static str, Vec<&'static PinoutRecord>>> = LazyLock::new(|| {
        let mut m: HashMap<&'static str, Vec<&'static PinoutRecord>> = HashMap::new();
        let bh = by_hash();
        for f in &asset().index {
            if let Some(rec) = bh.get(f.h.as_str()) {
                m.entry(f.name.as_str()).or_default().push(rec);
            }
        }
        m
    });
    &M
}

/// Every distinct physical footprint in the lineup.
pub fn all_records() -> &'static [PinoutRecord] {
    &asset().records
}

/// The footprint(s) the part `name` ships in. Exact name match (the index keys
/// on the full chip name, like the catalog); empty if the part is absent.
pub fn footprints_for(name: &str) -> &'static [&'static PinoutRecord] {
    static EMPTY: &[&PinoutRecord] = &[];
    by_name().get(name).map(|v| v.as_slice()).unwrap_or(EMPTY)
}

/// A record by its content hash.
pub fn record(hash: &str) -> Option<&'static PinoutRecord> {
    by_hash().get(hash).copied()
}

/// Number of distinct pinouts / index rows (test + diagnostic).
pub fn record_count() -> usize {
    asset().records.len()
}
pub fn index_count() -> usize {
    asset().index.len()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(v: &[&str]) -> Vec<String> {
        v.iter().map(|x| x.to_string()).collect()
    }

    #[test]
    fn classify_gpio_power_dedicated_nc() {
        assert_eq!(
            PinFunction::classify(&s(&["PA0"])),
            PinFunction::Gpio(vec![PinId { port: 'A', num: 0 }])
        );
        assert_eq!(PinFunction::classify(&s(&["VDD"])), PinFunction::Power(vec![Rail::Vdd]));
        assert_eq!(PinFunction::classify(&s(&["VREF+"])), PinFunction::Power(vec![Rail::VrefP]));
        assert_eq!(PinFunction::classify(&s(&["NRST"])), PinFunction::Dedicated(s(&["NRST"])));
        assert_eq!(PinFunction::classify(&[]), PinFunction::Nc);
    }

    #[test]
    fn classify_fused_rail_expands_to_set() {
        // A pin that bonds VDD and VDDA -> the rail SET, sorted+deduped.
        match PinFunction::classify(&s(&["VDD/VDDA"])) {
            PinFunction::Power(rails) => {
                assert!(rails.contains(&Rail::Vdd) && rails.contains(&Rail::Vdda));
                assert_eq!(rails.len(), 2);
            }
            other => panic!("expected Power, got {other:?}"),
        }
        match PinFunction::classify(&s(&["VDDA/VREF+"])) {
            PinFunction::Power(rails) => {
                assert!(rails.contains(&Rail::Vdda) && rails.contains(&Rail::VrefP));
            }
            other => panic!("expected Power, got {other:?}"),
        }
    }

    #[test]
    fn classify_merged_gpio_ball_keeps_all_pins() {
        // Low-pin packages bond several GPIOs to one ball — must keep the SET,
        // never just signals[0].
        match PinFunction::classify(&s(&["PB3", "PB4", "PB5", "PB6"])) {
            PinFunction::Gpio(pins) => assert_eq!(pins.len(), 4),
            other => panic!("expected Gpio set, got {other:?}"),
        }
    }

    #[test]
    fn rail_never_collapses_distinct_rails() {
        // The one property that keeps the power gate sound: distinct physical
        // rails must parse to distinct canonical rails.
        let distinct = ["VDD", "VSS", "VDDA", "VSSA", "VBAT", "VREF+", "VREF-", "VDDUSB", "VDDIO2"];
        let parsed: Vec<Rail> = distinct.iter().map(|t| Rail::parse(t).unwrap()).collect();
        let uniq: BTreeSet<Rail> = parsed.iter().copied().collect();
        assert_eq!(uniq.len(), distinct.len(), "two distinct rails collapsed to one canonical");
        // Unknown / voltage-distinct tokens are NOT rails (-> Dedicated).
        assert_eq!(Rail::parse("VDD33_USB"), None);
        assert_eq!(Rail::parse("PA0"), None);
    }

    #[test]
    fn package_class_strips_suffix_keeps_kind() {
        assert_eq!(package_class("LQFP64"), Some(("LQFP", 64)));
        assert_eq!(package_class("LQFP64_GP"), Some(("LQFP", 64)));
        assert_eq!(package_class("LQFP48_N"), Some(("LQFP", 48)));
        assert_eq!(package_class("UFQFPN48"), Some(("UFQFPN", 48)));
        assert_eq!(package_class("TFBGA100"), Some(("TFBGA", 100)));
        // A QFN and a QFP of equal pin count are DIFFERENT kinds (not drop-in).
        assert_ne!(package_class("UFQFPN64").unwrap().0, package_class("LQFP64").unwrap().0);
    }

    #[test]
    fn asset_loads_and_is_populated() {
        // The embedded asset must carry the whole lineup, not the empty
        // bootstrap placeholder.
        assert!(record_count() > 300, "only {} distinct pinouts", record_count());
        assert!(index_count() > 2000, "only {} index rows", index_count());
    }

    #[test]
    fn g474re_lqfp64_has_expected_physical_pins() {
        let recs = footprints_for("STM32G474RE");
        let lqfp64 = recs
            .iter()
            .find(|r| r.pkg == "LQFP64")
            .expect("G474RE should ship in LQFP64");
        assert_eq!(lqfp64.n, 64);
        // Position 12 is PA0, position 15 is VSS, position 1 is VBAT (datasheet).
        assert_eq!(
            lqfp64.at("12").map(|p| p.function()),
            Some(PinFunction::Gpio(vec![PinId { port: 'A', num: 0 }]))
        );
        assert_eq!(lqfp64.at("15").map(|p| p.function()), Some(PinFunction::Power(vec![Rail::Vss])));
        assert_eq!(lqfp64.at("1").map(|p| p.function()), Some(PinFunction::Power(vec![Rail::Vbat])));
    }

    #[test]
    fn same_package_name_is_not_the_same_footprint() {
        // The load-bearing soundness fact: G474 and H523 both call it "LQFP64"
        // but the physical maps diverge — so the package NAME can never be a
        // match key; per-position gates must do the work.
        let g4 = footprints_for("STM32G474RE")
            .iter()
            .find(|r| r.pkg == "LQFP64")
            .copied();
        let h5 = footprints_for("STM32H523RE")
            .iter()
            .find(|r| r.pkg == "LQFP64")
            .copied();
        if let (Some(g4), Some(h5)) = (g4, h5) {
            assert_ne!(g4.h, h5.h, "G4 and H5 LQFP64 must be distinct pinouts");
            // Concretely: position 15 is VSS on G4 but a GPIO on H5.
            let g4_15 = g4.at("15").map(|p| p.function());
            let h5_15 = h5.at("15").map(|p| p.function());
            assert_ne!(g4_15, h5_15, "pos 15 should differ between G4 and H5 LQFP64");
        }
    }
}
