//! Differential analog **pin-pair** finder — answers "what physical pin pairs
//! can do ADC+/-, COMP+/-, and OP+?" for the active chip. A power designer
//! sensing a shunt or a differential node needs to know which pins can serve as
//! the +/- terminals of a differential front-end; this view enumerates them
//! straight from the descriptor.
//!
//! Fully data-driven over [`crate::mcu_pinout::af_rows`] (the `af:None` analog
//! rows), so it works for every family with no per-chip tables:
//!   * **ADC differential** — a channel `m` is a usable pair iff BOTH a positive
//!     pin (`IN<m>` on G4, `INP<m>` on H5) AND a negative pin (`INN<m>`) are
//!     bonded on the same ADC instance. Pairing keys off the `INN<m>` role
//!     string, NOT the "INN[i] == INP[i+1] pin" heuristic (which breaks on the
//!     ADC1 PB11 shared/private-boundary quirk on G474). Where the negative
//!     pin's own single-ended index is not `m+1` (the one anomalous case), the
//!     pin pair is still correct but the *channel number* is flagged as
//!     data-derived — verify DIFSEL against RM0440 before emitting a config.
//!   * **COMP +/-** — the + (INP*) is always an external pin; the - (INM*) is
//!     usually an internal DAC/VREFINT threshold (the idiomatic over-current
//!     topology), so the default row shows an internal minus. True two-pin
//!     external INM pairs exist but are the exception — shown only under the
//!     "external -" toggle.
//!   * **OP+** — in PGA current-sense mode the inverting node is on-chip and
//!     VOUT routes internally to an ADC channel, so only the VINP+ pin is a
//!     placement constraint. External VINM pairs (inverting/external-gain modes)
//!     are shown only under the "external -" toggle.
//!
//! Read-only: the user reads pairs here and allocates them in the existing
//! peripheral / fabric / converter views.

use std::collections::HashMap;

use eframe::egui::{self, Color32};

use crate::mcu_pinout::{af_rows, signals_on, AfRow, PinId};
use crate::mcu_raw::RawMcuData;
use crate::package_view::{Action, PinLink, PinPaint};
use crate::phys_pinout::PinoutRecord;
use crate::pinout::Pin;

const INTERNAL: Color32 = Color32::from_rgb(150, 150, 160);
const PLUS_COL: Color32 = Color32::from_rgb(120, 200, 140);
const MINUS_COL: Color32 = Color32::from_rgb(210, 150, 120);
const HEDGE_COL: Color32 = Color32::from_rgb(210, 180, 80);

// Package-overlay fills.
const PLUS_FILL: Color32 = Color32::from_rgb(70, 150, 95);
const MINUS_FILL: Color32 = Color32::from_rgb(185, 110, 70);
const BOTH_FILL: Color32 = Color32::from_rgb(175, 150, 70);

/// How the Analog-pairs view is displayed.
#[derive(Default, Clone, Copy, PartialEq, Eq)]
pub enum Display {
    #[default]
    Table,
    Package,
}

fn kind_link_color(k: Kind) -> Color32 {
    match k {
        Kind::AdcDiff => Color32::from_rgb(90, 180, 220),
        Kind::Comp => Color32::from_rgb(195, 125, 205),
        Kind::OpampPlus => Color32::from_rgb(120, 195, 120),
    }
}

fn pin(p: PinId) -> Pin {
    Pin::new(p.port, p.num)
}

/// The three differential-front-end shapes the finder distinguishes.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Kind {
    AdcDiff,
    Comp,
    OpampPlus,
}

impl Kind {
    pub const ALL: [Kind; 3] = [Kind::AdcDiff, Kind::Comp, Kind::OpampPlus];
    fn label(self) -> &'static str {
        match self {
            Kind::AdcDiff => "ADC +/-",
            Kind::Comp => "COMP +/-",
            Kind::OpampPlus => "OP+",
        }
    }
    fn absent_note(self) -> &'static str {
        match self {
            Kind::AdcDiff => "No ADC differential pairs on this chip (no INN inputs in the ADC data).",
            Kind::Comp => "No comparators on this chip.",
            Kind::OpampPlus => "No op-amps on this chip.",
        }
    }
}

/// One differential-capable analog front-end: a physical (+pin, -pin) option on
/// a single peripheral instance. `minus == None` means the inverting side is
/// on-chip (OPAMP PGA) or an internal threshold (COMP default).
#[derive(Clone)]
pub struct DiffFrontEnd {
    pub kind: Kind,
    pub instance: &'static str,
    pub plus_pin: PinId,
    pub plus_role: &'static str,
    pub minus: Option<(PinId, &'static str)>,
    /// ADC differential channel `m` (`DIFSEL[m]`), when applicable.
    pub channel: Option<u8>,
    /// The channel number is a data-label artifact for this row (the negative
    /// pin's single-ended index isn't `m+1`); the pin pair is still correct.
    pub channel_hedge: bool,
    /// A true two-external-pin pair (COMP INMSEL 110/111, or OPAMP inverting /
    /// external-gain mode) — the exception, gated behind the "external -" toggle.
    pub external_minus: bool,
    pub note: String,
}

/// ADC single-ended / positive channel index. G4 exposes it as `IN<n>`, H5 as
/// `INP<n>`; try `INP` first because H5 pins carry BOTH an analog `INP1`
/// (`af:None`) and a timer `IN1` (`af:Some`) — the caller already gates on
/// `af.is_none()`, this just resolves the prefix.
pub(crate) fn adc_pos_index(role: &str) -> Option<u8> {
    let tail = role.strip_prefix("INP").or_else(|| role.strip_prefix("IN"))?;
    if tail.is_empty() || !tail.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    tail.parse().ok()
}

/// ADC differential negative channel index (`INN<m>`).
fn adc_neg_index(role: &str) -> Option<u8> {
    let tail = role.strip_prefix("INN")?;
    if tail.is_empty() || !tail.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    tail.parse().ok()
}

/// COMP non-inverting input role (`INP`, `INP0`, `INP1`, `INP2`, `INP3`, …).
/// Accepts any digit tail so both the G4 base (`INP0/INP1`) and the C5 base
/// (`INP1/INP2/INP3`) match without per-family code.
pub(crate) fn comp_plus(role: &str) -> bool {
    role == "INP" || (role.strip_prefix("INP").is_some_and(|t| !t.is_empty() && t.bytes().all(|b| b.is_ascii_digit())))
}

/// COMP inverting input role (`INM`, `INM0`, `INM1`, …) — external pins only.
pub(crate) fn comp_minus(role: &str) -> bool {
    role == "INM" || (role.strip_prefix("INM").is_some_and(|t| !t.is_empty() && t.bytes().all(|b| b.is_ascii_digit())))
}

/// OPAMP non-inverting input pins (`VINP`, `VINP0/1/2`). Skips the `_SEC`
/// secondary-mux rows (they duplicate the same pins) so each pin appears once.
pub(crate) fn opamp_plus(role: &str) -> bool {
    !role.ends_with("_SEC") && (role == "VINP" || role.starts_with("VINP"))
}

/// OPAMP inverting input pins (`VINM`, `VINM0/1`), external-gain / inverting mode.
fn opamp_minus(role: &str) -> bool {
    !role.ends_with("_SEC") && (role == "VINM" || role.starts_with("VINM"))
}

/// ADC single-ended positive channel index from an `IN<n>` role (G4/C5 style).
/// Excludes `INN<n>` (differential negative) and H5's `INP<n>`.
fn adc_in_ch(role: &str) -> Option<u8> {
    let t = role.strip_prefix("IN")?;
    if t.is_empty() || !t.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    t.parse().ok()
}

/// OPAMP output → ADC channel routing, derived purely from pin co-location: each
/// `OPAMPx_VOUT` pin also carries the ADC `IN<n>` role of the channel the ADC
/// reads the opamp output on (the internal `OPAINTOEN` path — same channel
/// number, pinless). Returns (opamp instance → a "`VOUT <pin> → ADCy_IN<n>`"
/// note fragment, VOUT pin → opamp instance for the reverse ADC-row lookup).
/// No hardcoded RM table; the category-dependent internal-ONLY extra channels
/// (which have no pin) aren't represented — the pin-bearing ones are.
fn opamp_adc_routing(
    raw: &'static RawMcuData,
) -> (HashMap<&'static str, String>, HashMap<PinId, &'static str>) {
    // ADC single-ended IN<n> channels present on each pin.
    let mut adc_on_pin: HashMap<PinId, Vec<(&'static str, u8)>> = HashMap::new();
    for r in af_rows(raw) {
        if r.af.is_none()
            && r.signal.peripheral.starts_with("ADC")
            && let Some(ch) = adc_in_ch(r.signal.role)
        {
            adc_on_pin.entry(r.pin).or_default().push((r.signal.peripheral, ch));
        }
    }
    let mut notes: HashMap<&'static str, String> = HashMap::new();
    let mut vout_opamp: HashMap<PinId, &'static str> = HashMap::new();
    for r in af_rows(raw) {
        if r.af.is_none() && r.signal.peripheral.starts_with("OPAMP") && r.signal.role == "VOUT" {
            vout_opamp.insert(r.pin, r.signal.peripheral);
            if let Some(chans) = adc_on_pin.get(&r.pin) {
                let mut cs = chans.clone();
                cs.sort_unstable();
                cs.dedup();
                let list = cs.iter().map(|(a, c)| format!("{a}_IN{c}")).collect::<Vec<_>>().join(", ");
                notes.insert(
                    r.signal.peripheral,
                    format!("VOUT {} → {list} (internal via OPAINTOEN)", r.pin.name()),
                );
            }
        }
    }
    (notes, vout_opamp)
}

/// Enumerate every differential-capable analog front-end on this chip. Family-
/// agnostic; kinds absent from the data simply yield no rows.
pub fn enumerate(raw: &'static RawMcuData) -> Vec<DiffFrontEnd> {
    // OPAMP output → ADC channel routing (VOUT pin co-locates with an ADC IN).
    let (opamp_adc_notes, vout_opamp) = opamp_adc_routing(raw);

    // Bucket analog (af:None) rows per peripheral instance.
    let mut by_inst: std::collections::BTreeMap<&'static str, Vec<AfRow>> = Default::default();
    for r in af_rows(raw) {
        if r.af.is_none() {
            by_inst.entry(r.signal.peripheral).or_default().push(r);
        }
    }

    let mut out: Vec<DiffFrontEnd> = Vec::new();
    for (inst, rows) in &by_inst {
        // ---- ADC differential: join INN<m> with IN<m>/INP<m> on the same ADC.
        // Gate on "ADC" so COMP's INP0/1/2 are never read as ADC channels.
        if inst.starts_with("ADC") {
            let mut pos: std::collections::BTreeMap<u8, (PinId, &'static str)> = Default::default();
            let mut neg: std::collections::BTreeMap<u8, (PinId, &'static str)> = Default::default();
            let mut pos_of_pin: std::collections::BTreeMap<PinId, u8> = Default::default();
            for r in rows {
                if let Some(m) = adc_pos_index(r.signal.role) {
                    pos.entry(m).or_insert((r.pin, r.signal.role));
                    pos_of_pin.entry(r.pin).or_insert(m);
                }
                if let Some(m) = adc_neg_index(r.signal.role) {
                    neg.entry(m).or_insert((r.pin, r.signal.role));
                }
            }
            for (&m, &(npin, nrole)) in &neg {
                // Require the positive pin to be bonded too, and distinct.
                let Some(&(ppin, prole)) = pos.get(&m) else { continue };
                if ppin == npin {
                    continue;
                }
                // Hedge the channel number when the negative pin's own single-
                // ended index isn't m+1 (the ADC1 shared/private boundary quirk).
                let hedge = pos_of_pin.get(&npin) != Some(&(m + 1));
                let mut note = if hedge {
                    format!(
                        "DIFSEL[{m}]=1; VINP[{m}]-VINN[{m}]. Channel # is data-derived at a shared-pin boundary — verify against RM0440."
                    )
                } else {
                    format!("DIFSEL[{m}]=1; VINP[{m}]-VINN[{m}]. Using ch{m} consumes ch{}'s pin.", m + 1)
                };
                // The +input pad is also an OPAMP output → this channel can read
                // the opamp internally (OPAINTOEN), freeing the pad.
                if let Some(op) = vout_opamp.get(&ppin) {
                    note.push_str(&format!("  ← {op} output (internal)"));
                }
                out.push(DiffFrontEnd {
                    kind: Kind::AdcDiff,
                    instance: inst,
                    plus_pin: ppin,
                    plus_role: prole,
                    minus: Some((npin, nrole)),
                    channel: Some(m),
                    channel_hedge: hedge,
                    external_minus: false,
                    note,
                });
            }
        }

        // ---- COMP: default = external + vs internal threshold; external - pins
        // (INMSEL 110/111) only under the advanced toggle.
        if inst.starts_with("COMP") {
            let mut plus: Vec<(PinId, &'static str)> = rows
                .iter()
                .filter(|r| comp_plus(r.signal.role))
                .map(|r| (r.pin, r.signal.role))
                .collect();
            plus.sort();
            plus.dedup_by_key(|(p, _)| *p);
            let minus: Vec<(PinId, &'static str)> = {
                let mut v: Vec<_> = rows
                    .iter()
                    .filter(|r| comp_minus(r.signal.role))
                    .map(|r| (r.pin, r.signal.role))
                    .collect();
                v.sort();
                v.dedup_by_key(|(p, _)| *p);
                v
            };
            for (pp, pr) in &plus {
                out.push(DiffFrontEnd {
                    kind: Kind::Comp,
                    instance: inst,
                    plus_pin: *pp,
                    plus_role: pr,
                    minus: None,
                    channel: None,
                    channel_hedge: false,
                    external_minus: false,
                    note: "- = internal DAC / VREFINT threshold (INMSEL 000-101)".into(),
                });
                for (mp, mr) in &minus {
                    if mp != pp {
                        out.push(DiffFrontEnd {
                            kind: Kind::Comp,
                            instance: inst,
                            plus_pin: *pp,
                            plus_role: pr,
                            minus: Some((*mp, mr)),
                            channel: None,
                            channel_hedge: false,
                            external_minus: true,
                            note: "true two-pin compare (INMSEL 110/111) — the exception".into(),
                        });
                    }
                }
            }
        }

        // ---- OPAMP: + only by default (PGA inverting node on-chip); external
        // VINM pairs only under the advanced toggle.
        if inst.starts_with("OPAMP") {
            let mut plus: Vec<(PinId, &'static str)> = rows
                .iter()
                .filter(|r| opamp_plus(r.signal.role))
                .map(|r| (r.pin, r.signal.role))
                .collect();
            plus.sort();
            plus.dedup_by_key(|(p, _)| *p);
            let minus: Vec<(PinId, &'static str)> = {
                let mut v: Vec<_> = rows
                    .iter()
                    .filter(|r| opamp_minus(r.signal.role))
                    .map(|r| (r.pin, r.signal.role))
                    .collect();
                v.sort();
                v.dedup_by_key(|(p, _)| *p);
                v
            };
            for (pp, pr) in &plus {
                out.push(DiffFrontEnd {
                    kind: Kind::OpampPlus,
                    instance: inst,
                    plus_pin: *pp,
                    plus_role: pr,
                    minus: None,
                    channel: None,
                    channel_hedge: false,
                    external_minus: false,
                    note: match opamp_adc_notes.get(inst) {
                        Some(route) => format!("PGA x2..64; - on-chip; {route}"),
                        None => "PGA x2..64; - on-chip; VOUT -> ADC (OPAINTOEN)".into(),
                    },
                });
                for (mp, mr) in &minus {
                    if mp != pp {
                        out.push(DiffFrontEnd {
                            kind: Kind::OpampPlus,
                            instance: inst,
                            plus_pin: *pp,
                            plus_role: pr,
                            minus: Some((*mp, mr)),
                            channel: None,
                            channel_hedge: false,
                            external_minus: true,
                            note: "external-gain / inverting mode (- is a pin)".into(),
                        });
                    }
                }
            }
        }
    }

    out.sort_by(|a, b| {
        (a.kind, a.instance, a.plus_pin, a.minus.map(|x| x.0))
            .cmp(&(b.kind, b.instance, b.plus_pin, b.minus.map(|x| x.0)))
    });
    out
}

/// Ephemeral filter/picker state for the Analog-pairs view.
#[derive(Default)]
pub struct AnalogFilter {
    pub kind: Option<Kind>,
    pub plus_pin: Option<PinId>,
    pub query: String,
    /// Show the true two-external-pin COMP/OPAMP pairs (the non-idiomatic case).
    pub advanced: bool,
    pub display: Display,
}

/// Filter check shared by the table and the package overlay (kind + advanced +
/// free-text). Deliberately excludes the `plus_pin` selection: the table filters
/// by it, but the package view uses it only for emphasis/links, not to hide pins.
fn passes_base(f: &DiffFrontEnd, filter: &AnalogFilter, q: &str) -> bool {
    if let Some(k) = filter.kind
        && f.kind != k
    {
        return false;
    }
    if !filter.advanced && f.external_minus {
        return false;
    }
    if !q.is_empty() {
        let hay = format!(
            "{} {} {} {}",
            f.instance,
            f.plus_pin.name(),
            f.plus_role,
            f.minus.map(|(p, r)| format!("{} {}", p.name(), r)).unwrap_or_default(),
        )
        .to_ascii_lowercase();
        if !hay.contains(q) {
            return false;
        }
    }
    true
}

/// Per-chip memo of the (chip-only) differential-front-end enumeration, so
/// `enumerate` — which scans the whole AF table and allocates a note String per
/// row — isn't rebuilt on every egui frame. Key = the `&'static` raw pointer
/// (one stable address per chip: compiled consts + once-leaked asset descriptors),
/// so it recomputes only on a chip switch. `enumerate` is independent of the
/// filter, so the filter is applied per-frame over the cached borrow.
#[derive(Default)]
pub struct EnumCache {
    key: Option<usize>,
    fes: Vec<DiffFrontEnd>,
}

impl EnumCache {
    pub fn frontends(&mut self, raw: &'static RawMcuData) -> &[DiffFrontEnd] {
        let k = raw as *const RawMcuData as usize;
        if self.key != Some(k) {
            self.fes = enumerate(raw);
            self.key = Some(k);
        }
        &self.fes
    }
}

pub fn show(
    ui: &mut egui::Ui,
    raw: &'static RawMcuData,
    filter: &mut AnalogFilter,
    footprint: Option<&'static PinoutRecord>,
    cache: &mut EnumCache,
) {
    let all = cache.frontends(raw);
    if footprint.is_none() {
        filter.display = Display::Table;
    }

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{} — differential analog pin pairs", raw.name)).strong());
        ui.label(egui::RichText::new("(ADC+/-, COMP+/-, OP+)").weak());
    });
    ui.label(
        egui::RichText::new(
            "Which physical pins can serve as the +/- terminals of a differential front-end. Read-only — allocate the pins in the peripheral / fabric views.",
        )
        .weak()
        .small(),
    );
    ui.separator();

    // Kind chips.
    ui.horizontal(|ui| {
        ui.label("Kind:");
        if ui.selectable_label(filter.kind.is_none(), "all").clicked() {
            filter.kind = None;
        }
        for k in Kind::ALL {
            if ui.selectable_label(filter.kind == Some(k), k.label()).clicked() {
                filter.kind = if filter.kind == Some(k) { None } else { Some(k) };
            }
        }
    });

    // + pin reciprocal picker + search + advanced toggle.
    let mut plus_pins: Vec<PinId> = all.iter().map(|f| f.plus_pin).collect();
    plus_pins.sort();
    plus_pins.dedup();
    ui.horizontal(|ui| {
        ui.label("+ pin:");
        egui::ComboBox::from_id_salt("analog_plus_pin")
            .width(90.0)
            .selected_text(filter.plus_pin.map(|p| p.name()).unwrap_or_else(|| "(any)".into()))
            .show_ui(ui, |ui| {
                if ui.selectable_label(filter.plus_pin.is_none(), "(any)").clicked() {
                    filter.plus_pin = None;
                }
                for &p in &plus_pins {
                    if ui.selectable_label(filter.plus_pin == Some(p), p.name()).clicked() {
                        filter.plus_pin = Some(p);
                    }
                }
            });

        ui.label("Search:");
        ui.add(
            egui::TextEdit::singleline(&mut filter.query)
                .desired_width(150.0)
                .hint_text("pin / peripheral"),
        );
        if !filter.query.is_empty() && ui.small_button("clear").clicked() {
            filter.query.clear();
        }
        ui.checkbox(&mut filter.advanced, "external -")
            .on_hover_text("Show true two-pin COMP/OPAMP pairs (INMSEL 110/111, opamp inverting mode). Off by default — the idiomatic minus is an internal threshold / on-chip PGA.");
    });

    // If a + pin is chosen, list the other roles that same physical pin carries —
    // one pin often feeds both a measurement and a trip.
    if let Some(pp) = filter.plus_pin {
        let mut also: Vec<String> = signals_on(raw, pp)
            .into_iter()
            .filter(|r| r.pin == pp)
            .map(|r| format!("{}.{}", r.signal.peripheral, r.signal.role))
            .collect();
        also.sort();
        also.dedup();
        ui.label(
            egui::RichText::new(format!("{} also carries: {}", pp.name(), also.join(", ")))
                .weak()
                .small(),
        );
    }
    // Table vs package drawing (package available only when a footprint resolved).
    if footprint.is_some() {
        ui.horizontal(|ui| {
            ui.label("View:");
            ui.selectable_value(&mut filter.display, Display::Table, "Table");
            ui.selectable_value(&mut filter.display, Display::Package, "Package");
        });
    }
    ui.separator();

    match (filter.display, footprint) {
        (Display::Package, Some(rec)) => show_package(ui, filter, all, rec),
        _ => show_table(ui, filter, all),
    }
}

fn show_table(ui: &mut egui::Ui, filter: &AnalogFilter, all: &[DiffFrontEnd]) {
    let q = filter.query.to_ascii_lowercase();
    let rows: Vec<&DiffFrontEnd> = all
        .iter()
        .filter(|f| passes_base(f, filter, &q))
        .filter(|f| filter.plus_pin.is_none_or(|pp| f.plus_pin == pp))
        .collect();

    // Note kinds that don't exist on this chip at all, so the view is never
    // silently blank.
    for k in Kind::ALL {
        if !all.iter().any(|f| f.kind == k) {
            ui.label(egui::RichText::new(format!("• {}", k.absent_note())).color(INTERNAL).small());
        }
    }
    ui.label(egui::RichText::new(format!("{} pairs", rows.len())).weak());

    egui::ScrollArea::vertical().show(ui, |ui| {
        egui::Grid::new("analog_pairs").striped(true).num_columns(6).show(ui, |ui| {
            for h in ["Kind", "Instance", "+ pin", "- pin", "Ch", "Note"] {
                ui.label(egui::RichText::new(h).strong());
            }
            ui.end_row();

            for f in &rows {
                ui.label(f.kind.label());
                ui.label(f.instance);
                ui.label(egui::RichText::new(format!("{} ({})", f.plus_pin.name(), f.plus_role)).color(PLUS_COL));
                match f.minus {
                    Some((p, r)) => {
                        ui.label(egui::RichText::new(format!("{} ({})", p.name(), r)).color(MINUS_COL));
                    }
                    None => {
                        let txt = match f.kind {
                            Kind::OpampPlus => "on-chip (PGA)",
                            _ => "internal threshold",
                        };
                        ui.label(egui::RichText::new(txt).color(INTERNAL).italics());
                    }
                }
                match f.channel {
                    Some(c) if f.channel_hedge => {
                        ui.label(egui::RichText::new(format!("ch{c}?")).color(HEDGE_COL))
                            .on_hover_text("Channel number is data-derived at a shared-pin boundary; the pin pair is correct. Verify DIFSEL against RM0440.");
                    }
                    Some(c) => {
                        ui.label(format!("ch{c}"));
                    }
                    None => {
                        ui.label("—");
                    }
                }
                ui.label(egui::RichText::new(&f.note).weak().small());
                ui.end_row();
            }
        });
    });
}

/// The package-drawing overlay: highlight every +/- terminal on the chip and,
/// once a + pin is picked, draw connectors to its differential partner(s).
fn show_package(
    ui: &mut egui::Ui,
    filter: &mut AnalogFilter,
    all: &[DiffFrontEnd],
    record: &PinoutRecord,
) {
    use std::collections::{BTreeMap, BTreeSet};

    #[derive(Default)]
    struct Part {
        plus: bool,
        minus: bool,
        descs: Vec<String>,
    }

    let q = filter.query.to_ascii_lowercase();
    let mut parts: BTreeMap<PinId, Part> = BTreeMap::new();
    for f in all.iter().filter(|f| passes_base(f, filter, &q)) {
        let e = parts.entry(f.plus_pin).or_default();
        e.plus = true;
        e.descs.push(format!("{} + ({})", f.instance, f.plus_role));
        if let Some((mp, mr)) = f.minus {
            let em = parts.entry(mp).or_default();
            em.minus = true;
            em.descs.push(format!("{} - ({})", f.instance, mr));
        }
    }

    // The selected + pin's partners drive the connectors + emphasis.
    let sel = filter.plus_pin;
    let mut partners: BTreeSet<PinId> = BTreeSet::new();
    let mut links: Vec<PinLink> = Vec::new();
    if let Some(s) = sel {
        for f in all.iter().filter(|f| f.plus_pin == s && passes_base(f, filter, &q)) {
            if let Some((mp, _)) = f.minus {
                partners.insert(mp);
                links.push(PinLink {
                    a: pin(s),
                    b: pin(mp),
                    color: kind_link_color(f.kind),
                    label: Some(match f.channel {
                        Some(c) => format!("{} ch{c}", f.instance),
                        None => f.instance.to_string(),
                    }),
                });
            }
        }
    }

    let mut paints: HashMap<Pin, PinPaint> = HashMap::new();
    for (pid, part) in &parts {
        let fill = if part.plus && part.minus {
            BOTH_FILL
        } else if part.plus {
            PLUS_FILL
        } else {
            MINUS_FILL
        };
        let tag = if part.plus && part.minus {
            "±"
        } else if part.plus {
            "+"
        } else {
            "-"
        };
        let emphasize = sel == Some(*pid) || partners.contains(pid);
        paints.insert(
            pin(*pid),
            PinPaint {
                fill,
                border: emphasize.then(|| egui::Stroke::new(2.0, Color32::WHITE)),
                sublabel: Some(tag.to_string()),
                tooltip: Some(part.descs.join("\n")),
                interactive: part.plus,
            },
        );
    }

    ui.label(
        egui::RichText::new(
            "Green = + terminal, orange = −, ± = both. Click a + pin (or use the picker above) to link its differential partner(s).",
        )
        .weak()
        .small(),
    );

    match crate::package_view::show_record(ui, record, &paints, &links) {
        Some(Action::Click(p)) => {
            let pid = PinId { port: p.port, num: p.num };
            // Select a clicked + terminal; clicking a non-+ pin clears the pick.
            filter.plus_pin = parts.get(&pid).filter(|pt| pt.plus).map(|_| pid);
        }
        Some(Action::ClickEmpty) => filter.plus_pin = None,
        None => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    #[test]
    fn enum_cache_recomputes_only_on_chip_switch() {
        let g474 = &crate::mcu_data::g474r::RAW;
        let h523 = &crate::mcu_data::h523r::RAW;
        let mut c = EnumCache::default();

        let g_pairs = c.frontends(g474).len();
        assert!(g_pairs > 0);
        assert_eq!(c.key, Some(g474 as *const RawMcuData as usize));

        // Same chip → cached (key unchanged), same result.
        assert_eq!(c.frontends(g474).len(), g_pairs);
        assert_eq!(c.key, Some(g474 as *const RawMcuData as usize));

        // Different chip → recompute (key + result change).
        let h_pairs = c.frontends(h523).len();
        assert_eq!(c.key, Some(h523 as *const RawMcuData as usize));
        assert_ne!(g_pairs, h_pairs, "different chips enumerate differently");
    }

    #[test]
    fn opamp_output_adc_channel_is_surfaced_both_ways() {
        let fes = enumerate(&crate::mcu_data::g474r::RAW);

        // OP+ row for OPAMP1 names the concrete ADC channel its VOUT feeds
        // (PA2 → ADC1_IN3), derived from pin co-location — no hardcoded table.
        let op1 = fes
            .iter()
            .find(|f| f.kind == Kind::OpampPlus && f.instance == "OPAMP1" && f.minus.is_none())
            .expect("OPAMP1 OP+ row");
        assert!(op1.note.contains("ADC1_IN3"), "OPAMP1 names its ADC channel: {}", op1.note);
        assert!(op1.note.contains("PA2"), "names the VOUT pad: {}", op1.note);

        // And the ADC1 ch3 diff row (its + pad PA2 == OPAMP1 VOUT) is flagged
        // opamp-fed, so ADC planning sees the internal path.
        let adc3 = fes
            .iter()
            .find(|f| f.kind == Kind::AdcDiff && f.instance == "ADC1" && f.channel == Some(3))
            .expect("ADC1 ch3 diff row");
        assert_eq!(adc3.plus_pin.name(), "PA2");
        assert!(adc3.note.contains("OPAMP1 output"), "ch3 marked opamp-fed: {}", adc3.note);
    }

    fn adc_channels(fes: &[DiffFrontEnd], adc: &str) -> BTreeSet<u8> {
        fes.iter()
            .filter(|f| f.kind == Kind::AdcDiff && f.instance == adc)
            .filter_map(|f| f.channel)
            .collect()
    }

    #[test]
    fn g474_adc_diff_channel_sets() {
        let fes = enumerate(&crate::mcu_data::g474r::RAW);
        // Verified against src/mcu_data/g474r.rs + RM0440: require BOTH IN<m> and
        // INN<m> bonded. ADC3 has orphan negatives (INN with no IN pin) => none;
        // ADC4 IN2 not bonded => only ch3,ch4; ADC5 exposes only ch1.
        assert_eq!(adc_channels(&fes, "ADC1"), (1..=11).chain([15]).collect());
        assert_eq!(adc_channels(&fes, "ADC2"), (1..=14).collect());
        assert_eq!(adc_channels(&fes, "ADC3"), BTreeSet::new());
        assert_eq!(adc_channels(&fes, "ADC4"), [3, 4].into_iter().collect());
        assert_eq!(adc_channels(&fes, "ADC5"), [1].into_iter().collect());
    }

    #[test]
    fn g474_adc_pin_pairs_and_guards() {
        let fes = enumerate(&crate::mcu_data::g474r::RAW);
        let pair = |adc: &str, ch: u8| -> Option<(String, String)> {
            fes.iter()
                .find(|f| f.kind == Kind::AdcDiff && f.instance == adc && f.channel == Some(ch))
                .and_then(|f| f.minus.map(|(m, _)| (f.plus_pin.name(), m.name())))
        };
        assert_eq!(pair("ADC1", 1), Some(("PA0".into(), "PA1".into())));
        assert_eq!(pair("ADC1", 15), Some(("PB0".into(), "PB11".into())));
        assert_eq!(pair("ADC5", 1), Some(("PA8".into(), "PA9".into())));

        // The ADC1 top channel is the sole hedged row; normal rows are not.
        let hedged: Vec<u8> = fes
            .iter()
            .filter(|f| f.kind == Kind::AdcDiff && f.instance == "ADC1" && f.channel_hedge)
            .filter_map(|f| f.channel)
            .collect();
        assert_eq!(hedged, vec![15]);

        // No emitted pair ever has the + and - on the same physical pin.
        for f in &fes {
            if let Some((m, _)) = f.minus {
                assert_ne!(f.plus_pin, m, "{} {:?} degenerate pair", f.instance, f.channel);
            }
        }
    }

    #[test]
    fn no_comp_masquerades_as_adc_channel() {
        // COMP roles INP0/INP1/INP2 must never be read as ADC channels: gate is
        // on instance.starts_with("ADC").
        for chip in [
            &crate::mcu_data::g474r::RAW,
            &crate::mcu_data::c531r::RAW,
            &crate::mcu_data::c5a3z::RAW,
        ] {
            for f in enumerate(chip) {
                if f.kind == Kind::AdcDiff {
                    assert!(f.instance.starts_with("ADC"), "{} tagged AdcDiff", f.instance);
                }
            }
        }
    }

    #[test]
    fn h523_is_adc_diff_only() {
        // H5: single-ended positive is INP<n>, negative INN<n>; no COMP/OPAMP.
        let fes = enumerate(&crate::mcu_data::h523r::RAW);
        let expect: BTreeSet<u8> = [1, 3, 4, 5, 10, 11, 12, 18].into_iter().collect();
        assert_eq!(adc_channels(&fes, "ADC1"), expect);
        assert_eq!(adc_channels(&fes, "ADC2"), expect);
        assert!(!fes.iter().any(|f| f.kind == Kind::Comp), "H5 has no COMP");
        assert!(!fes.iter().any(|f| f.kind == Kind::OpampPlus), "H5 has no OPAMP");
    }

    #[test]
    fn c5_has_comp_opamp_but_no_adc_diff() {
        let c531 = enumerate(&crate::mcu_data::c531r::RAW);
        assert!(!c531.iter().any(|f| f.kind == Kind::AdcDiff), "C531 has no ADC INN");
        assert!(c531.iter().any(|f| f.kind == Kind::Comp), "C531 has COMP");
        assert!(c531.iter().any(|f| f.kind == Kind::OpampPlus), "C531 has OPAMP1");

        let c5a3 = enumerate(&crate::mcu_data::c5a3z::RAW);
        assert!(!c5a3.iter().any(|f| f.kind == Kind::AdcDiff), "C5A3 has no ADC INN");
        assert!(c5a3.iter().any(|f| f.kind == Kind::Comp), "C5A3 has COMP1");
        assert!(!c5a3.iter().any(|f| f.kind == Kind::OpampPlus), "C5A3 has no OPAMP");
    }

    #[test]
    fn opamp_plus_pins_not_triplicated() {
        // Each OPAMP + pin appears once as a default (minus:None) row despite the
        // VINP / VINP0 / VINP_SEC role aliases.
        let fes = enumerate(&crate::mcu_data::g474r::RAW);
        let defaults: Vec<PinId> = fes
            .iter()
            .filter(|f| f.kind == Kind::OpampPlus && f.instance == "OPAMP1" && f.minus.is_none())
            .map(|f| f.plus_pin)
            .collect();
        let mut uniq = defaults.clone();
        uniq.sort();
        uniq.dedup();
        assert_eq!(defaults.len(), uniq.len(), "OPAMP1 + pins triplicated: {:?}", defaults);
    }
}
