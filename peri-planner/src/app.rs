use std::collections::BTreeMap;

use eframe::egui;

use crate::c531_design::{C531Design, ConverterLeg};
use crate::c531_view::ConverterAction;
use crate::fabric_view::{self, Selection};
use crate::g474::*;
use crate::h523_design::H523Design;
use crate::mcu::{Mcu, Package};
use crate::pinout::{self, ChipVariant};
use crate::requirements::*;
use crate::solver::{TimerSlotUsage, ALL_CR_SLOTS_HELPER};

// v4: pin_assignments re-keyed from the typed Signal enum to the owned
// (peripheral, role) form. Old v3 saves are intentionally dropped.
const STORAGE_KEY: &str = "peri_planner_design_v4";
/// The post-spine save: one RON blob holding all named projects. Replaces the
/// pre-spine per-key save (which is read once, on first run, to seed the active
/// project — see `seed_active_from_legacy_keys`).
const PROJECTS_KEY: &str = "peri_planner_projects_v1";
const HISTORY_CAP: usize = 40;

/// A point-in-time copy of whichever per-MCU design model is active — the
/// generic unit of undo/redo. Unifies the HISTORY across families without
/// unifying the model TYPES (which stay deliberately separate). The history
/// stack only ever holds the active MCU's variant (it's cleared on ANY MCU
/// switch), so restoring is always coherent with the live view.
#[derive(Clone, PartialEq)]
enum DesignSnapshot {
    G474(Design),
    H523(H523Design), // snapshot of the active H523-family (H523 or C5A3) design
    C531(C531Design),
    /// An any-STM32 (arbitrary part) generic design, tagged with its asset key.
    Asset(String, H523Design),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum ViewMode {
    Fabric,
    Hrtim,
    Comms,
    Timers,
    Waveforms,
    Package,
    Inventory,
    AfTable,
    Catalog,
    Dropin,
    Converter,
    Peripherals,
    Analog,
}

/// One named design — the savable unit ("project"). Holds the full design state:
/// the G474 `Design`, the per-MCU H523-family map (so H523/C5A3 keep their own
/// plan), the C531 converter plan, and the active chip selection. Ephemeral UI
/// (view, finder query/demands, undo stacks, picked role) stays on
/// `PeriPlannerApp` and is shared across projects.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
struct Project {
    name: String,
    design: Design,
    h523_designs: std::collections::HashMap<Mcu, H523Design>,
    c531_design: C531Design,
    mcu: Mcu,
    package: Package,
    variant: ChipVariant,
    /// The Part-finder "declare-once" spec — per project so each design keeps its
    /// own peripheral wishlist. (`DemandInput`'s `&'static` keys can't serialize
    /// directly, so it round-trips via `select::demand_serde`.)
    #[serde(with = "crate::select::demand_serde", default = "crate::select::default_demands")]
    catalog_demands: Vec<crate::select::DemandInput>,
    /// Active *arbitrary* chip: any catalog part name (not one of the 14 compiled
    /// packages). `Some` overrides `package`/`mcu` and routes the whole lineup
    /// through the generic descriptor-driven planner. `None` = a compiled chip.
    #[serde(default)]
    asset_chip: Option<String>,
    /// Per-arbitrary-part generic pin-lock design, keyed by descriptor prefix
    /// (`"STM32<line><letter>"`). The editable planning state for any STM32.
    #[serde(default)]
    asset_designs: std::collections::HashMap<String, H523Design>,
}

impl Project {
    fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            design: Design::default(),
            h523_designs: std::collections::HashMap::new(),
            c531_design: C531Design::new(),
            mcu: Mcu::G474,
            package: Package::G474R,
            variant: ChipVariant::G474R,
            catalog_demands: crate::select::default_demands(),
            asset_chip: None,
            asset_designs: std::collections::HashMap::new(),
        }
    }

    /// The descriptor-prefix key used to store an arbitrary part's design
    /// (`"STM32<line><letter>"`, first 10 chars) — several flash/temp variants of
    /// one pinout share a design, like the compiled `chip_prefix`.
    fn asset_key(name: &str) -> String {
        name.chars().take(10).collect()
    }

    /// Coarse, open family tag for the active MCU (gates fabric topology /
    /// codegen tier in the lowered plan). A string, not a closed enum, so a new
    /// line is data — see `docs/firmware-codegen-design.md` §8.
    fn family_tag(&self) -> &'static str {
        match self.mcu {
            Mcu::G474 => "G4",
            Mcu::H523 => "H5",
            Mcu::C5A3 | Mcu::C531 => "C5",
        }
    }

    /// Lower the ACTIVE family's design model into the unified
    /// [`PinPlan`](crate::pin_plan::PinPlan) — the single dispatch point that
    /// picks the right per-family lowerer. One-way / derived (the family models
    /// stay authoritative); this is the input the firmware codegen reads.
    pub(crate) fn to_pin_plan(&self) -> crate::pin_plan::PinPlan {
        // An any-STM32 chip lowers its generic design against the lineup
        // descriptor (coarse family = the line, e.g. "H7", "F4").
        if let Some(name) = &self.asset_chip
            && let Some(desc) = crate::desc_asset::descriptor_for(name)
        {
            let key = Self::asset_key(desc.name);
            let d = self.asset_designs.get(&key).cloned().unwrap_or_default();
            let target = crate::pin_plan::Target {
                package: desc.name.to_string(),
                family: desc.family.strip_prefix("STM32").unwrap_or(desc.family).to_string(),
            };
            let mut plan = d.to_pin_plan(target, desc.raw);
            crate::pin_plan::assign_dma(&mut plan, desc);
            return plan;
        }
        let target = crate::pin_plan::Target {
            package: self.package.name().to_string(),
            family: self.family_tag().to_string(),
        };
        let mut plan = match self.mcu {
            Mcu::G474 => self.design.to_pin_plan(target),
            Mcu::H523 | Mcu::C5A3 => {
                let empty = H523Design::default();
                let d = self.h523_designs.get(&self.mcu).unwrap_or(&empty);
                d.to_pin_plan(target, self.package.raw())
            }
            Mcu::C531 => self.c531_design.to_pin_plan(self.package, target),
        };
        // Generic DMA channel assignment (descriptor-driven, one place for all
        // families) after the family lowerer produced placements/routes.
        crate::pin_plan::assign_dma(&mut plan, self.package.descriptor());
        plan
    }
}

/// The whole persisted project set (one RON blob). Save-compat with the
/// pre-spine per-key save is NOT required, so this fully replaces it.
#[derive(serde::Serialize, serde::Deserialize)]
struct Persisted {
    active: Project,
    others: Vec<Project>,
}

pub struct PeriPlannerApp {
    /// The active named project — the live design state. A DIRECT field (not a
    /// `Vec` index) so the render loop's simultaneous disjoint-field borrows
    /// (e.g. `&mut active.h523_designs` alongside `&active.c531_design`) still
    /// compile, exactly as the flat fields did before the spine.
    active: Project,
    /// The other (inactive) saved projects, in a stable order; switching swaps
    /// one of these into `active`.
    others: Vec<Project>,
    view: ViewMode,
    history: Vec<DesignSnapshot>,
    redo: Vec<DesignSnapshot>,
    /// Bumped on every project/MCU switch. The non-G474 frame bracket captures it
    /// before rendering and only records an undo step if it's unchanged — so an
    /// in-frame project switch (which clears history) never lets the OLD project's
    /// snapshot land on the NEW project's stack. Ephemeral.
    nav_epoch: u64,
    /// Role "picked up" in the package view, waiting to be dropped on a
    /// candidate pin. Not persisted — ephemeral interaction state.
    picked: Option<crate::picker::PickedRole>,
    /// Filter state for the AfTable view. Ephemeral.
    af_filter: crate::af_view::AfFilter,
    /// Filter/picker state for the Analog differential-pairs view. Ephemeral.
    analog_filter: crate::analog_view::AnalogFilter,
    /// Per-chip memo of the analog front-end enumeration (avoids a per-frame
    /// full-AF-table rebuild). Ephemeral; keyed on the active chip's raw.
    analog_cache: crate::analog_view::EnumCache,
    /// The (peripheral, role) picked up for placement in the generic pin-map view.
    /// Ephemeral.
    pin_map_pick: Option<(String, String)>,
    /// Transient outcome of the last C531 Auto-assign (which legs couldn't route).
    /// Ephemeral; cleared on any other converter edit.
    c531_status: Option<String>,
    /// Part-finder query state (whole-lineup catalog search). Ephemeral.
    catalog_query: crate::catalog::SearchQuery,
    /// Memoized Part-finder evaluation (recomputed only when query/demands change).
    catalog_eval_cache: crate::select::EvalCache,
    /// Part-finder results sort (column + direction). Ephemeral.
    catalog_sort: crate::catalog_view::CatalogSort,
    /// Name of a part clicked in the Part / Drop-in finder, opened at the start
    /// of the next frame (deferred to avoid switching chips mid-render). A
    /// compiled part opens its planner; any other opens a read-only browser.
    /// Ephemeral.
    pending_open: Option<String>,
    /// A non-compiled catalog part opened read-only from a finder, rendered from
    /// the lineup descriptor asset (Inventory / Pin-AF only — no planner/fabric).
    /// When `Some`, the app shows a read-only browser instead of the planner.
    /// Ephemeral (a `&'static` into the lazily-built asset; not persisted).
    asset_part: Option<&'static crate::mcu::McuDescriptor>,
    /// Drop-in finder query (strictness + power-mode + family filter). Ephemeral.
    dropin_query: crate::dropin::DropinQuery,
    /// Memoized lineup-wide drop-in scan; reruns only on source/query change.
    dropin_cache: crate::dropin::DropinCache,
    /// Candidate whose per-pin diff is expanded in the drop-in finder. Ephemeral.
    dropin_focus: Option<String>,
    /// Set at startup when the persisted project blob existed but failed to parse:
    /// the raw bytes, so the first `save` writes them to a recovery key instead of
    /// silently letting the new blob overwrite the unparseable one. Ephemeral.
    corrupt_blob: Option<String>,
}

impl Default for PeriPlannerApp {
    fn default() -> Self {
        Self {
            active: Project::new("Untitled"),
            others: Vec::new(),
            view: ViewMode::Fabric,
            history: Vec::new(),
            redo: Vec::new(),
            nav_epoch: 0,
            picked: None,
            af_filter: Default::default(),
            analog_filter: Default::default(),
            analog_cache: Default::default(),
            pin_map_pick: None,
            c531_status: None,
            catalog_query: Default::default(),
            catalog_eval_cache: Default::default(),
            catalog_sort: Default::default(),
            pending_open: None,
            asset_part: None,
            dropin_query: Default::default(),
            dropin_cache: Default::default(),
            dropin_focus: None,
            corrupt_blob: None,
        }
    }
}

impl PeriPlannerApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut slf = Self::default();
        if let Some(storage) = cc.storage {
            // The view (ephemeral, global) restores regardless of project source.
            if let Some(v) = eframe::get_value::<ViewMode>(storage, "peri_planner_view_v1") {
                slf.view = v;
            }
            // `get_value` returns None on a PARSE error too, not just absence — so
            // distinguish via the raw string before deciding what to do.
            let blob_raw = storage.get_string(PROJECTS_KEY);
            if let Some(p) = eframe::get_value::<Persisted>(storage, PROJECTS_KEY) {
                // Multi-project blob (post-spine). The source of truth.
                slf.active = p.active;
                slf.others = p.others;
            } else if let Some(raw) = blob_raw {
                // The blob EXISTS but didn't parse (e.g. a Package/Mcu variant
                // dropped since it was saved). Do NOT reseed from the legacy keys
                // (that would load stale pre-spine data), and do NOT let the next
                // save silently clobber it: stash the raw bytes so `save` can write
                // them to a recovery key. Start with a fresh default project.
                slf.corrupt_blob = Some(raw);
            } else {
                // No project blob yet: seed the single active project from the
                // pre-spine per-key save so the user's current design survives
                // the bump (read once; from now on the project blob is written).
                Self::seed_active_from_legacy_keys(storage, &mut slf.active);
            }
            // Keep mcu / package consistent if storage drifted.
            if slf.active.package.mcu() != slf.active.mcu {
                slf.active.package = slf.active.mcu.default_package();
            }
        }
        // The view is global and (on a legacy load) restored independently of the
        // active chip — keep them coherent so a restored chip never lands on a tab
        // from another family. Idempotent for the normal (blob) load.
        slf.land_on_active_view();
        slf
    }

    /// One-time read of the pre-spine per-key save into `proj` (no project blob
    /// existed). Not a compat shim — these keys are read once and then only the
    /// project blob is ever written.
    fn seed_active_from_legacy_keys(storage: &dyn eframe::Storage, proj: &mut Project) {
        if let Some(design) = eframe::get_value::<Design>(storage, STORAGE_KEY) {
            proj.design = design;
        }
        if let Some(v) = eframe::get_value::<ChipVariant>(storage, "peri_planner_variant_v1")
            && proj.design.variant != v {
                proj.design.set_variant(v);
            }
        proj.variant = proj.design.variant;
        if let Some(m) = eframe::get_value::<Mcu>(storage, "peri_planner_mcu_v1") {
            proj.mcu = m;
        }
        if let Some(v) =
            eframe::get_value::<Vec<(Mcu, H523Design)>>(storage, "peri_planner_h523_designs_v1")
        {
            proj.h523_designs = v.into_iter().collect();
        }
        if let Some(d) = eframe::get_value::<C531Design>(storage, "peri_planner_c531_design_v1") {
            proj.c531_design = d;
        }
        if let Some(p) = eframe::get_value::<Package>(storage, "peri_planner_package_v1") {
            proj.package = p;
        } else {
            proj.package = Package::from_g474_variant(proj.variant);
        }
    }

    /// The active H523-family design (H523 / C5A3 each keep their own), or an
    /// empty borrow when none has been touched yet. Reads only — mutators use
    /// `h523_designs.entry(self.active.mcu).or_default()`.
    fn h523<'a>(&'a self, empty: &'a H523Design) -> &'a H523Design {
        self.active.h523_designs.get(&self.active.mcu).unwrap_or(empty)
    }

    /// Clear undo/redo for a navigation change (project or MCU switch) and bump
    /// `nav_epoch`. The bump is what stops the in-flight non-G474 frame bracket
    /// from recording a stale step against whatever became active mid-frame.
    fn clear_undo_for_nav(&mut self) {
        self.history.clear();
        self.redo.clear();
        self.nav_epoch = self.nav_epoch.wrapping_add(1);
        // Per-chip ephemeral UI state must not leak across a chip switch.
        self.pin_map_pick = None;
        self.c531_status = None;
    }

    /// Switch the active MCU, clearing undo/redo history. Each MCU has its own
    /// design state, so a snapshot of one MCU must never restore into another's
    /// live view — that invariant is what makes per-MCU undo coherent. No-op if
    /// already active. A package switch within an MCU keeps history (same design).
    fn set_active_mcu(&mut self, m: Mcu) {
        // Undo history holds only the ACTIVE model's snapshots, so it clears on an
        // MCU change OR when leaving any-STM32 mode (Asset snapshots must not
        // restore into a compiled model).
        let leaving_asset = self.active.asset_chip.is_some();
        if m != self.active.mcu || leaving_asset {
            self.clear_undo_for_nav();
        }
        self.active.mcu = m;
        // Selecting a compiled MCU always leaves any-STM32 mode.
        self.active.asset_chip = None;
    }

    /// Re-derive the ephemeral asset descriptor from the (persisted) active
    /// `asset_chip`. The single place `asset_part` is set — called after any
    /// change to the active project's `asset_chip` (nav, open, chip switch).
    fn sync_asset_part(&mut self) {
        self.asset_part = self
            .active
            .asset_chip
            .as_deref()
            .and_then(crate::desc_asset::descriptor_for);
    }

    /// Make an arbitrary catalog part the active (any-STM32) chip.
    fn set_active_asset(&mut self, name: String) {
        self.clear_undo_for_nav();
        self.active.asset_chip = Some(name);
        self.sync_asset_part();
        // Land on the generic planner unless already on a shared descriptor view.
        if !matches!(
            self.view,
            ViewMode::Inventory | ViewMode::AfTable | ViewMode::Analog | ViewMode::Catalog
        ) {
            self.view = ViewMode::Peripherals;
        }
    }

    /// Switch the active project to `others[i]`, swapping the current active back
    /// into `others` (so order stays stable). Undo history is per-project, so it
    /// clears — a snapshot of one project must never restore into another's.
    fn switch_project(&mut self, i: usize) {
        if i < self.others.len() {
            std::mem::swap(&mut self.active, &mut self.others[i]);
            self.clear_undo_for_nav();
            self.sync_asset_part();
            self.land_on_active_view();
        }
    }

    /// Create a fresh project and make it active (the old active joins `others`).
    fn new_project(&mut self) {
        let mut np = Project::new(format!("Untitled {}", self.others.len() + 2));
        std::mem::swap(&mut self.active, &mut np);
        self.others.push(np);
        self.clear_undo_for_nav();
        self.sync_asset_part();
        self.land_on_active_view();
    }

    /// Duplicate the active project: the copy (same design, distinct name) becomes
    /// active so you can diverge it; the original joins `others`. Undo clears (the
    /// copy is a fresh editing context).
    fn duplicate_active(&mut self) {
        let mut copy = self.active.clone();
        copy.name = format!("{} copy", self.active.name);
        std::mem::swap(&mut self.active, &mut copy);
        self.others.push(copy);
        self.clear_undo_for_nav();
        self.sync_asset_part();
        self.land_on_active_view();
    }

    /// Delete the active project, promoting the first of `others` to active.
    /// No-op when it's the only project (the Delete button is also disabled then).
    fn delete_active(&mut self) {
        if !self.others.is_empty() {
            self.active = self.others.remove(0);
            self.clear_undo_for_nav();
            self.sync_asset_part();
            self.land_on_active_view();
        }
    }

    /// Keep the (global) view coherent with the active project's chip: a
    /// family-specific tab (G474 Fabric/HRTIM…, C531 Converter, H5/C5 Peripherals)
    /// from a previous project would render blank on a different family, so fall
    /// back to that family's landing view. Shared tabs (Inventory/Pin-AF/finders)
    /// are kept so cross-project comparison isn't interrupted.
    fn land_on_active_view(&mut self) {
        let shared = matches!(
            self.view,
            ViewMode::Inventory | ViewMode::AfTable | ViewMode::Analog | ViewMode::Catalog | ViewMode::Dropin
        );
        if !shared {
            // An arbitrary (any-STM32) chip has no family planner — land on the generic one.
            self.view = if self.active.asset_chip.is_some() {
                ViewMode::Peripherals
            } else {
                match self.active.mcu {
                    Mcu::G474 => ViewMode::Fabric,
                    Mcu::C531 => ViewMode::Converter,
                    Mcu::H523 | Mcu::C5A3 => ViewMode::Peripherals,
                }
            };
        }
    }

    /// Snapshot whichever model is active.
    fn snapshot_active(&self) -> DesignSnapshot {
        // An any-STM32 chip's generic design takes precedence over the (stale,
        // placeholder) `mcu` — its editable state lives in `asset_designs`.
        if let Some(desc) = self.asset_part {
            let key = Project::asset_key(desc.name);
            let d = self.active.asset_designs.get(&key).cloned().unwrap_or_default();
            return DesignSnapshot::Asset(key, d);
        }
        match self.active.mcu {
            Mcu::G474 => DesignSnapshot::G474(self.active.design.clone()),
            Mcu::H523 | Mcu::C5A3 => {
                DesignSnapshot::H523(self.active.h523_designs.get(&self.active.mcu).cloned().unwrap_or_default())
            }
            Mcu::C531 => DesignSnapshot::C531(self.active.c531_design.clone()),
        }
    }

    /// Restore a snapshot into its matching live model. History clears on every
    /// MCU switch, so the active MCU here always matches the one the snapshot was
    /// captured under — the H523 design lands back in the correct per-MCU slot.
    fn restore_snapshot(&mut self, snap: DesignSnapshot) {
        match snap {
            DesignSnapshot::G474(d) => self.active.design = d,
            DesignSnapshot::H523(d) => {
                self.active.h523_designs.insert(self.active.mcu, d);
            }
            DesignSnapshot::C531(d) => self.active.c531_design = d,
            DesignSnapshot::Asset(key, d) => {
                self.active.asset_designs.insert(key, d);
            }
        }
    }

    /// Whether the active non-G474 model changed vs `before` (ignoring H523 note
    /// edits, which aren't their own undo step). G474 is recorded by `mutate`.
    fn non_g474_changed(&self, before: &DesignSnapshot) -> bool {
        match before {
            DesignSnapshot::H523(b) => {
                let empty = H523Design::new();
                !self.h523(&empty).eq_ignoring_notes(b)
            }
            DesignSnapshot::C531(b) => self.active.c531_design != *b,
            DesignSnapshot::Asset(key, b) => {
                let cur = self.active.asset_designs.get(key).cloned().unwrap_or_default();
                !cur.eq_ignoring_notes(b)
            }
            DesignSnapshot::G474(_) => false,
        }
    }

    /// Push a pre-edit snapshot onto the undo stack (cap-trimmed) and invalidate
    /// redo — the single place both the G474 `mutate` path and the non-G474
    /// frame-diff record history.
    fn push_history(&mut self, snap: DesignSnapshot) {
        self.history.push(snap);
        if self.history.len() > HISTORY_CAP {
            self.history.remove(0);
        }
        self.redo.clear();
    }

    /// Wrap a G474 mutation so it records a history entry iff state changed.
    fn mutate(&mut self, f: impl FnOnce(&mut Design)) {
        let before = self.active.design.clone();
        f(&mut self.active.design);
        if self.active.design != before {
            self.push_history(DesignSnapshot::G474(before));
        }
    }

    fn undo(&mut self) {
        if let Some(prev) = self.history.pop() {
            let current = self.snapshot_active();
            self.restore_snapshot(prev);
            self.redo.push(current);
        }
    }

    fn redo_op(&mut self) {
        if let Some(next) = self.redo.pop() {
            let current = self.snapshot_active();
            self.restore_snapshot(next);
            self.history.push(current);
        }
    }

    /// Logical-kind peripheral counts derived from the active design — the
    /// backward "find parts from this design" projection (lossy: drops pins,
    /// instance identity, HRTIM roles; recovers the coarse counts the Part
    /// finder consumes).
    fn design_demand_counts(&self) -> std::collections::BTreeMap<&'static str, u8> {
        use std::collections::{BTreeMap, BTreeSet};
        let mut c: BTreeMap<&'static str, u8> = BTreeMap::new();
        let mut bump = |k: &'static str, n: u8| {
            let e = c.entry(k).or_insert(0);
            *e = e.saturating_add(n);
        };
        match self.active.mcu {
            Mcu::G474 => {
                for spec in &self.active.design.requirements {
                    for k in spec.demand_kinds() {
                        bump(k, 1);
                    }
                }
            }
            Mcu::C531 => {
                for leg in &self.active.c531_design.legs {
                    if leg.complementary {
                        bump("COMP_PWM", (leg.channels_mask.count_ones() as u8).max(1));
                    }
                    if leg.ocp.is_some() {
                        bump("OCP", 1);
                    }
                    if leg.adc_sense.is_some() {
                        bump("ADC", 1);
                    }
                }
            }
            Mcu::H523 | Mcu::C5A3 => {
                // Distinct DECLARED peripheral instances per kind (the intent;
                // a declared use counts whether or not its roles are pinned yet).
                let mut by_kind: BTreeMap<&'static str, BTreeSet<&str>> = BTreeMap::new();
                let empty = H523Design::new();
                for u in &self.h523(&empty).uses {
                    if let Some(k) = crate::select::kind_of(&u.peripheral) {
                        by_kind.entry(k).or_default().insert(u.peripheral.as_str());
                    }
                }
                for (k, set) in by_kind {
                    bump(k, set.len() as u8);
                }
            }
        }
        c
    }

    /// Fill the Part-finder demands from the active design and open the finder
    /// ("find parts that fit what I've sketched").
    fn summarize_to_demands(&mut self) {
        let counts = self.design_demand_counts();
        for d in &mut self.active.catalog_demands {
            d.count = counts.get(d.kind).copied().unwrap_or(0);
        }
        self.view = ViewMode::Catalog;
    }

    /// Seed the active design from the Part-finder demands (forward "declare
    /// once" link). Idempotent: fills up to each demanded count given what the
    /// design already has. Per-family — each model holds what it cleanly can
    /// (see docs/declare-once-seed-mappings.md).
    fn seed_from_demands(&mut self) {
        let demands: Vec<crate::select::DemandInput> =
            self.active.catalog_demands.iter().filter(|d| d.count > 0).cloned().collect();
        let have = self.design_demand_counts();
        match self.active.mcu {
            Mcu::G474 => self.mutate(|d| seed_g474_into(d, &demands, &have)),
            Mcu::C531 => seed_c531_into(&mut self.active.c531_design, &demands, &have),
            Mcu::H523 | Mcu::C5A3 => {
                let raw = self.active.package.descriptor().raw;
                let d = self.active.h523_designs.entry(self.active.mcu).or_default();
                seed_h523_into(d, raw, &demands, &have);
            }
        }
    }

    /// A persistent, family-agnostic status line: active chip + design counts +
    /// a validation rollup. Health at a glance from any view — the cross-family
    /// equivalent of the G474-only capability panel.
    fn render_status_line(&self, ctx: &egui::Context) {
        use crate::requirements::Severity;
        const GREEN: egui::Color32 = egui::Color32::from_rgb(100, 200, 120);
        const YELLOW: egui::Color32 = egui::Color32::from_rgb(210, 180, 80);
        egui::TopBottomPanel::bottom("status_line").show(ctx, |ui| {
            ui.horizontal(|ui| {
                // Any-STM32 (arbitrary part): identity + generic-planner completeness.
                if let Some(desc) = self.asset_part {
                    ui.label(egui::RichText::new(desc.name).strong());
                    ui.label(desc.family);
                    ui.separator();
                    let empty = H523Design::new();
                    let d = self
                        .active
                        .asset_designs
                        .get(&Project::asset_key(desc.name))
                        .unwrap_or(&empty);
                    let problems = d.validate(desc.raw);
                    ui.label(format!("{} peripherals · {} pins locked", d.uses.len(), d.pin_locks.len()));
                    ui.separator();
                    if problems.is_empty() {
                        ui.colored_label(GREEN, "✓ complete");
                    } else {
                        ui.colored_label(YELLOW, format!("⚠ {} unplaced/unreachable", problems.len()));
                    }
                    return;
                }
                ui.label(egui::RichText::new(self.active.package.chip_prefix()).strong());
                ui.label(self.active.package.package_label());
                ui.separator();
                match self.active.mcu {
                    Mcu::G474 => {
                        let reqs = self.active.design.requirements.len();
                        let assigned =
                            self.active.design.assignments.iter().filter(|a| a.is_some()).count();
                        let pins = self.active.design.pin_assignments.len();
                        ui.label(format!("{reqs} reqs · {assigned} assigned · {pins} pins locked"));
                        ui.separator();
                        let warns = self
                            .active
                            .design
                            .warnings()
                            .into_iter()
                            .filter(|c| c.severity == Severity::Warn)
                            .count();
                        if warns == 0 {
                            ui.colored_label(GREEN, "✓ 0 warnings");
                        } else {
                            ui.colored_label(YELLOW, format!("⚠ {warns} warning(s)"));
                        }
                    }
                    Mcu::C531 => {
                        let problems = self.active.c531_design.validate(self.active.package);
                        ui.label(format!("{} legs", self.active.c531_design.legs.len()));
                        ui.separator();
                        if problems.is_empty() {
                            ui.colored_label(GREEN, "✓ realizable");
                        } else {
                            ui.colored_label(YELLOW, format!("⚠ {} problem(s)", problems.len()));
                        }
                    }
                    Mcu::H523 | Mcu::C5A3 => {
                        let empty = H523Design::new();
                        let d = self.h523(&empty);
                        let uses = d.uses.len();
                        let locks = d.pin_locks.len();
                        let problems = d.validate(self.active.package.descriptor().raw);
                        ui.label(format!("{uses} peripherals · {locks} pins locked"));
                        ui.separator();
                        if problems.is_empty() {
                            ui.colored_label(GREEN, "✓ complete");
                        } else {
                            ui.colored_label(
                                YELLOW,
                                format!("⚠ {} unplaced/unreachable", problems.len()),
                            );
                        }
                    }
                }
            });
        });
    }

    fn render_top_bar(&mut self, ctx: &egui::Context, can_undo: bool, can_redo: bool) {
        // Browsing a non-compiled catalog part: the same chrome, but the live
        // planner controls (MCU/Package/Undo/Redo/Export + planner tabs) give way
        // to a read-only badge — only the descriptor views (Inventory/Pin-AF/Part
        // finder) apply, since an asset part carries no planner/fabric.
        let browse_name: Option<&'static str> = self.asset_part.map(|d| d.name);
        let browsing = browse_name.is_some();
        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            // Named-project switcher — the savable design envelope. Hidden while
            // browsing a read-only asset part (which isn't a project).
            if !browsing {
                let mut switch_to: Option<usize> = None;
                let mut do_new = false;
                let mut do_dup = false;
                let mut do_delete = false;
                ui.horizontal(|ui| {
                    ui.label("Project:");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.active.name)
                            .desired_width(150.0)
                            .hint_text("name this design"),
                    );
                    if !self.others.is_empty() {
                        egui::ComboBox::from_id_salt("project_switch")
                            .selected_text("Switch ▾")
                            .show_ui(ui, |ui| {
                                for (i, p) in self.others.iter().enumerate() {
                                    if ui.button(&p.name).clicked() {
                                        switch_to = Some(i);
                                        ui.close();
                                    }
                                }
                            });
                    }
                    if ui.button("＋ New").on_hover_text("start a new empty design").clicked() {
                        do_new = true;
                    }
                    if ui
                        .button("⎘ Duplicate")
                        .on_hover_text("copy the active design into a new project")
                        .clicked()
                    {
                        do_dup = true;
                    }
                    if ui
                        .add_enabled(!self.others.is_empty(), egui::Button::new("🗑 Delete"))
                        .on_hover_text("delete the active project")
                        .clicked()
                    {
                        do_delete = true;
                    }
                    ui.label(
                        egui::RichText::new(format!("({} total)", self.others.len() + 1))
                            .weak()
                            .small(),
                    );
                });
                if let Some(i) = switch_to {
                    self.switch_project(i);
                }
                if do_new {
                    self.new_project();
                }
                if do_dup {
                    self.duplicate_active();
                }
                if do_delete {
                    self.delete_active();
                }
                ui.separator();
            }
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("Peripheral planner").strong());
                ui.separator();
                if let Some(name) = browse_name {
                    ui.colored_label(
                        egui::Color32::from_rgb(120, 170, 220),
                        format!("● {name}"),
                    );
                    ui.label(
                        egui::RichText::new("any STM32 · generic planner · package-letter pinout")
                            .small()
                            .weak(),
                    );
                    if ui.button("Select another…").clicked() {
                        self.view = ViewMode::Catalog;
                    }
                    if ui.button("✕ Back to G474").clicked() {
                        self.active.asset_chip = None;
                        self.clear_undo_for_nav(); // Asset history must not survive the switch.
                        // The asset-only Pin map / Peripherals view would render blank
                        // on G474 — land on a view the target chip actually has.
                        self.land_on_active_view();
                    }
                    ui.separator();
                    if ui.add_enabled(can_undo, egui::Button::new("Undo")).clicked() {
                        self.undo();
                    }
                    if ui.add_enabled(can_redo, egui::Button::new("Redo")).clicked() {
                        self.redo_op();
                    }
                    ui.separator();
                    if ui.button("Export").clicked()
                        && let Some(desc) = self.asset_part {
                            let empty = H523Design::new();
                            let d = self
                                .active
                                .asset_designs
                                .get(&Project::asset_key(desc.name))
                                .unwrap_or(&empty);
                            let text = d.export_summary(desc.name);
                            ui.ctx().copy_text(text);
                        }
                    if ui
                        .button("Gen firmware")
                        .on_hover_text("Copy a generated embassy-stm32 board scaffold for this part")
                        .clicked()
                    {
                        let plan = self.active.to_pin_plan();
                        ui.ctx().copy_text(crate::codegen::generate(&plan));
                    }
                } else {
                    ui.label("MCU:");
                    let mut pending_mcu: Option<Mcu> = None;
                    egui::ComboBox::from_id_salt("mcu")
                        .selected_text(self.active.mcu.label())
                        .show_ui(ui, |ui| {
                            for &m in Mcu::ALL {
                                let label = if m.is_implemented() {
                                    m.label().to_string()
                                } else {
                                    format!("{} (preview)", m.label())
                                };
                                if ui.selectable_label(m == self.active.mcu, label).clicked() {
                                    pending_mcu = Some(m);
                                }
                            }
                        });
                    if let Some(m) = pending_mcu {
                        // Each MCU keeps its own design state, so switching MCUs
                        // invalidates undo history (a popped snapshot must never
                        // restore into a different MCU's live view).
                        self.set_active_mcu(m);
                        // Reset package to the new MCU's default; keeps
                        // self.active.variant stale on H523 (only consumed by G474
                        // legacy code, which doesn't render on H523).
                        self.active.package = m.default_package();
                        if let Some(v) = self.active.package.to_g474_variant() {
                            self.active.variant = v;
                            self.mutate(|d| d.set_variant(v));
                        }
                        if m == Mcu::C531 {
                            // C531 has a real planner — land on it, not inventory.
                            if !matches!(
                                self.view,
                                ViewMode::Inventory | ViewMode::AfTable | ViewMode::Converter
                            ) {
                                self.view = ViewMode::Converter;
                            }
                        } else if m != Mcu::G474
                            && !matches!(
                                self.view,
                                ViewMode::Inventory | ViewMode::AfTable | ViewMode::Peripherals
                            )
                        {
                            // H523 / C5A3 land on the generic Peripherals planner.
                            self.view = ViewMode::Peripherals;
                        }
                    }
                    ui.separator();
                    ui.label("Package:");
                    let mut pending_package: Option<Package> = None;
                    egui::ComboBox::from_id_salt("package")
                        .selected_text(self.active.package.display_label())
                        .show_ui(ui, |ui| {
                            for p in self.active.mcu.packages() {
                                if ui
                                    .selectable_label(p == self.active.package, p.display_label())
                                    .clicked()
                                {
                                    pending_package = Some(p);
                                }
                            }
                        });
                    if let Some(p) = pending_package {
                        self.active.package = p;
                        if let Some(v) = p.to_g474_variant() {
                            self.active.variant = v;
                            self.mutate(|d| d.set_variant(v));
                        }
                    }
                    if ui
                        .button("Any STM32…")
                        .on_hover_text("Plan / analyse any of the ~1600 STM32 parts (Part finder → click a part)")
                        .clicked()
                    {
                        self.view = ViewMode::Catalog;
                    }
                    ui.separator();
                    if ui.add_enabled(can_undo, egui::Button::new("Undo")).clicked() {
                        self.undo();
                    }
                    if ui.add_enabled(can_redo, egui::Button::new("Redo")).clicked() {
                        self.redo_op();
                    }
                }
                ui.separator();
                ui.label("View:");
                // Planner tabs are MCU-specific and editing-only — hidden while
                // browsing a read-only asset part.
                if !browsing {
                    if self.active.mcu == Mcu::G474 {
                        ui.selectable_value(&mut self.view, ViewMode::Fabric, "Power fabric");
                        ui.selectable_value(&mut self.view, ViewMode::Hrtim, "HRTIM");
                        ui.selectable_value(&mut self.view, ViewMode::Comms, "Comms");
                        ui.selectable_value(&mut self.view, ViewMode::Timers, "Timers");
                        ui.selectable_value(&mut self.view, ViewMode::Waveforms, "Waveforms");
                        ui.selectable_value(&mut self.view, ViewMode::Package, "Package");
                    }
                    if self.active.mcu == Mcu::C531 {
                        ui.selectable_value(&mut self.view, ViewMode::Converter, "Converter");
                    }
                    if self.active.mcu == Mcu::H523 || self.active.mcu == Mcu::C5A3 {
                        ui.selectable_value(&mut self.view, ViewMode::Peripherals, "Peripherals");
                        ui.selectable_value(&mut self.view, ViewMode::Package, "Pin map");
                    }
                } else {
                    // Any arbitrary STM32 gets the generic (descriptor-driven) planner.
                    ui.selectable_value(&mut self.view, ViewMode::Peripherals, "Peripherals");
                    ui.selectable_value(&mut self.view, ViewMode::Package, "Pin map");
                }
                // Descriptor views — available for both planner chips and asset parts.
                ui.selectable_value(&mut self.view, ViewMode::Inventory, "Inventory");
                ui.selectable_value(&mut self.view, ViewMode::AfTable, "Pin / AF");
                ui.selectable_value(&mut self.view, ViewMode::Analog, "Analog pairs");
                ui.selectable_value(&mut self.view, ViewMode::Catalog, "Part finder");
                ui.selectable_value(&mut self.view, ViewMode::Dropin, "Drop-in finder");
                if !browsing {
                    ui.separator();
                    if ui.button("Export").clicked() {
                        // Family-correct: each MCU exports ITS model, not always G474.
                        let text = match self.active.mcu {
                            Mcu::G474 => self.active.design.export_summary(),
                            Mcu::C531 => self.active.c531_design.export_summary(self.active.package.name()),
                            Mcu::H523 | Mcu::C5A3 => {
                                let empty = H523Design::new();
                                self.h523(&empty).export_summary(self.active.package.name())
                            }
                        };
                        ui.ctx().copy_text(text);
                    }
                    if ui
                        .button("Gen firmware")
                        .on_hover_text(
                            "Copy a generated embassy-stm32 board scaffold (Tier-1 \
                             instances + pins) for this design",
                        )
                        .clicked()
                    {
                        let plan = self.active.to_pin_plan();
                        ui.ctx().copy_text(crate::codegen::generate(&plan));
                    }
                }
            });
            // Shared "design spec" strip: peripherals declared once (in the Part
            // finder) are summarized here and actionable from every planner view —
            // the lightweight "declare once" link between the finder and allocator.
            let active: Vec<String> = self
                .active
                .catalog_demands
                .iter()
                .filter(|d| d.count > 0)
                .map(|d| format!("{}×{}", d.kind, d.count))
                .collect();
            ui.horizontal(|ui| {
                ui.label("Spec:");
                if active.is_empty() {
                    ui.label(
                        egui::RichText::new("(none — declare peripherals in Part finder)").weak(),
                    );
                } else {
                    ui.label(active.join("   "));
                }
                // The seed/summarize buttons act on the active PLANNER design;
                // hide them while browsing a read-only asset part (which has none).
                if !browsing {
                    ui.separator();
                    if !active.is_empty()
                        && ui
                            .button("Seed design ▸")
                            .on_hover_text(
                                "Add the demanded peripherals to the current design (idempotent — \
                                 fills up to each demanded count)",
                            )
                            .clicked()
                    {
                        self.seed_from_demands();
                    }
                    if ui
                        .button("⟳ Demands from design")
                        .on_hover_text(
                            "Count this design's peripherals into the Part-finder demands and open it",
                        )
                        .clicked()
                    {
                        self.summarize_to_demands();
                    }
                }
            });
        });
    }

    fn handle_package_action(&mut self, action: Option<crate::package_view::Action>) {
        use crate::package_view::Action;
        use crate::picker;
        match action {
            Some(Action::Click(pin)) => {
                match self.picked {
                    None => {
                        // Try to identify a role from the signal on this pin.
                        let pin_signals = picker::current_pin_signals(&self.active.design, self.active.variant);
                        if let Some(sig) = pin_signals.get(&pin).copied() {
                            self.picked = picker::role_for_pin(pin, sig, self.active.variant, &self.active.design);
                        }
                    }
                    Some(role) => {
                        // Clicking the source pin cancels the pick.
                        if pin == role.current_pin() {
                            self.picked = None;
                            return;
                        }
                        if let Some(mv) = picker::resolve_move(&self.active.design, self.active.variant, role, pin) {
                            let variant = self.active.variant;
                            self.mutate(|d| picker::apply_move(d, variant, mv));
                        }
                        self.picked = None;
                    }
                }
            }
            Some(Action::ClickEmpty) => {
                self.picked = None;
            }
            None => {}
        }
    }

    /// Apply an edit from the C531 Converter view to `c531_design`. Kept
    /// outside the `Design` undo stack (like the H523-family designs) — the
    /// converter plan is a separate model.
    fn apply_converter_action(&mut self, action: Option<ConverterAction>) {
        let Some(action) = action else { return };
        // Any edit other than Auto-assign clears the transient auto-assign notice.
        if !matches!(action, ConverterAction::AutoAssign) {
            self.c531_status = None;
        }
        match action {
            ConverterAction::AddLeg => {
                // Default a new leg to the first advanced-control timer on the
                // chip (TIM1 on C531) so OCP routing is available out of the box.
                let default_tim = self
                    .active
                    .package
                    .descriptor()
                    .timers
                    .iter()
                    .filter(|t| t.kind == crate::mcu::TimerKind::Advanced)
                    .find_map(|t| TimId::ALL.iter().copied().find(|id| id.number() == t.number))
                    .unwrap_or(TimId::Tim1);
                self.active.c531_design.add_leg(ConverterLeg::pwm(default_tim));
            }
            ConverterAction::RemoveLeg(i) => self.active.c531_design.remove_leg(i),
            ConverterAction::SetLeg(i, leg) => {
                if let Some(slot) = self.active.c531_design.legs.get_mut(i) {
                    *slot = leg;
                }
            }
            ConverterAction::AutoAssign => {
                let pkg = self.active.package;
                let r = self.active.c531_design.auto_assign(pkg);
                self.c531_status = (!r.fully_solved()).then(|| {
                    let mut parts = Vec::new();
                    if !r.ocp_unassignable.is_empty() {
                        parts.push(format!("{} OCP", r.ocp_unassignable.len()));
                    }
                    if !r.sense_unassignable.is_empty() {
                        parts.push(format!("{} ADC-sense", r.sense_unassignable.len()));
                    }
                    format!(
                        "⚠ Auto-assign couldn't route {} — this chip's fabric can't satisfy them all (see problems).",
                        parts.join(" + "),
                    )
                });
            }
        }
    }
}

impl eframe::App for PeriPlannerApp {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        // Preserve an unparseable prior blob (if any) under a recovery key BEFORE
        // we overwrite the main key — so a load failure never means silent data
        // loss. Done once (the take() clears it).
        if let Some(raw) = self.corrupt_blob.take() {
            storage.set_string(&format!("{PROJECTS_KEY}_recovered"), raw);
        }
        // One blob for all projects (RON handles the enum-keyed h523 map).
        let blob = Persisted {
            active: self.active.clone(),
            others: self.others.clone(),
        };
        eframe::set_value(storage, PROJECTS_KEY, &blob);
        // The view is ephemeral/global (shared across projects), kept separate.
        eframe::set_value(storage, "peri_planner_view_v1", &self.view);
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Keep the ephemeral asset descriptor in lockstep with the (persisted)
        // active `asset_chip` — covers reload of a saved any-STM32 chip and any
        // mutation of `asset_chip` (MCU combo, Close, project switch). Cheap: a
        // single hashmap lookup.
        self.sync_asset_part();

        // Open a part clicked in a finder last frame (deferred so we never switch
        // the active chip mid-render). A compiled part becomes the fully-typed
        // active chip; any other becomes the active any-STM32 chip.
        if let Some(name) = self.pending_open.take() {
            if let Some(pkg) = Package::for_chip_name(&name) {
                // A compiled part becomes the fully-typed active chip. Opening it
                // can switch the active MCU — clear undo (each MCU has its own
                // design); set_active_mcu also leaves any-STM32 mode.
                self.set_active_mcu(pkg.mcu());
                self.active.package = pkg;
                if let Some(v) = pkg.to_g474_variant() {
                    self.active.variant = v;
                }
                self.sync_asset_part();
                self.view = ViewMode::Inventory;
            } else if crate::desc_asset::descriptor_for(&name).is_some() {
                // Any other STM32 becomes the active any-STM32 chip (generic,
                // editable planner over the lineup descriptor asset).
                self.set_active_asset(name);
            }
        }

        // Any-STM32 (arbitrary catalog part) mode: the SAME chrome as the planner,
        // but the chip is a lineup-descriptor part with the generic, editable
        // planner (Peripherals / Pin-AF locks) + analysis views (Analog + package
        // drawing / Inventory / Part finder). Pin-lock edits persist per part in
        // `asset_designs` and are undoable via the same per-frame bracket the other
        // generic families use. Ctrl+Z is handled here (this block returns early).
        if let Some(desc) = self.asset_part {
            let ctrl = ctx.input(|i| i.modifiers.command);
            if ctrl && ctx.input(|i| i.key_pressed(egui::Key::Z)) {
                if ctx.input(|i| i.modifiers.shift) { self.redo_op(); } else { self.undo(); }
            }
            if ctrl && ctx.input(|i| i.key_pressed(egui::Key::Y)) {
                self.redo_op();
            }
            let nav_before = self.nav_epoch;
            let before = self.snapshot_active();
            let can_undo = !self.history.is_empty();
            let can_redo = !self.redo.is_empty();
            self.render_top_bar(ctx, can_undo, can_redo);
            self.render_status_line(ctx);
            let view = self.view;
            let af_filter = &mut self.af_filter;
            let analog_filter = &mut self.analog_filter;
            let analog_cache = &mut self.analog_cache;
            let catalog_query = &mut self.catalog_query;
            let catalog_cache = &mut self.catalog_eval_cache;
            let catalog_sort = &mut self.catalog_sort;
            let catalog_demands = &mut self.active.catalog_demands;
            let dropin_query = &mut self.dropin_query;
            let dropin_cache = &mut self.dropin_cache;
            let dropin_focus = &mut self.dropin_focus;
            let pin_map_pick = &mut self.pin_map_pick;
            let key = Project::asset_key(desc.name);
            let design = self.active.asset_designs.entry(key.clone()).or_default();
            let mut open: Option<String> = None;
            egui::CentralPanel::default().show(ctx, |ui| match view {
                ViewMode::Peripherals => crate::peripherals_view::show(ui, desc.raw, design),
                ViewMode::Package => crate::pin_map_view::show(
                    ui,
                    desc.raw,
                    crate::phys_pinout::best_footprint(desc.name, ""),
                    design,
                    pin_map_pick,
                ),
                ViewMode::AfTable => crate::af_view::show(ui, desc.raw, af_filter, Some(design)),
                ViewMode::Analog => crate::analog_view::show(
                    ui,
                    desc.raw,
                    analog_filter,
                    crate::phys_pinout::best_footprint(desc.name, ""),
                    analog_cache,
                ),
                ViewMode::Catalog => {
                    open = crate::catalog_view::show(
                        ui, catalog_query, catalog_demands, catalog_cache, catalog_sort,
                    );
                }
                ViewMode::Dropin => {
                    open = crate::dropin_view::show_named(
                        ui,
                        &key,
                        desc.name,
                        "",
                        crate::dropin::DesignSource::H523(design),
                        dropin_query,
                        dropin_cache,
                        dropin_focus,
                    );
                }
                _ => crate::inventory_view::show(ui, desc),
            });
            self.pending_open = open;
            // Record this frame's pin-lock edits as one undo step (unless a nav
            // switched the active chip mid-frame — nav_epoch guards that).
            if self.nav_epoch == nav_before && self.non_g474_changed(&before) {
                self.push_history(before);
            }
            return;
        }

        let ctrl = ctx.input(|i| i.modifiers.command);
        let shift = ctx.input(|i| i.modifiers.shift);
        if ctrl && ctx.input(|i| i.key_pressed(egui::Key::Z)) {
            if shift { self.redo_op(); } else { self.undo(); }
        }
        if ctrl && ctx.input(|i| i.key_pressed(egui::Key::Y)) {
            self.redo_op();
        }

        let can_undo = !self.history.is_empty();
        let can_redo = !self.redo.is_empty();

        if self.active.mcu != Mcu::G474 {
            // Bracket the whole non-G474 frame to record edits (a Seed in the top
            // bar, or a view action) as ONE undo step. These models don't go
            // through `mutate`; the deferred-action views apply ≤1 edit per frame.
            let nav_before = self.nav_epoch;
            let before = self.snapshot_active();
            self.render_top_bar(ctx, can_undo, can_redo);
            self.render_status_line(ctx);
            let descriptor = self.active.package.descriptor();
            let package = self.active.package;
            let view = self.view;
            let af_filter = &mut self.af_filter;
            let analog_filter = &mut self.analog_filter;
            let analog_cache = &mut self.analog_cache;
            let c531_status = self.c531_status.as_deref();
            // Active H523-family design, created on first touch for this MCU.
            // (`render_top_bar` above may have switched MCU this frame; this binds
            // to the now-active one — the post-render diff is skipped on a switch.)
            let h523 = self.active.h523_designs.entry(self.active.mcu).or_default();
            let c531 = &self.active.c531_design;
            let catalog_query = &mut self.catalog_query;
            let catalog_demands = &mut self.active.catalog_demands;
            let catalog_cache = &mut self.catalog_eval_cache;
            let catalog_sort = &mut self.catalog_sort;
            let mcu = self.active.mcu;
            let dropin_query = &mut self.dropin_query;
            let dropin_cache = &mut self.dropin_cache;
            let dropin_focus = &mut self.dropin_focus;
            let pin_map_pick = &mut self.pin_map_pick;
            let mut jump = None;
            let mut conv_action: Option<ConverterAction> = None;
            egui::CentralPanel::default().show(ctx, |ui| {
                match view {
                    ViewMode::AfTable => {
                        crate::af_view::show(ui, descriptor.raw, af_filter, Some(h523));
                    }
                    ViewMode::Peripherals => {
                        crate::peripherals_view::show(ui, descriptor.raw, h523);
                    }
                    ViewMode::Package => {
                        crate::pin_map_view::show(
                            ui,
                            descriptor.raw,
                            crate::phys_pinout::best_footprint(package.name(), package.package_label()),
                            h523,
                            pin_map_pick,
                        );
                    }
                    ViewMode::Analog => {
                        crate::analog_view::show(
                            ui,
                            descriptor.raw,
                            analog_filter,
                            crate::phys_pinout::best_footprint(package.name(), package.package_label()),
                            analog_cache,
                        );
                    }
                    ViewMode::Catalog => {
                        jump = crate::catalog_view::show(
                            ui, catalog_query, catalog_demands, catalog_cache, catalog_sort,
                        );
                    }
                    ViewMode::Dropin => {
                        // C531 carries a converter-leg model (no locked pins);
                        // H523 / C5A3 use the generic pin-lock model.
                        let dsrc = if mcu == Mcu::C531 {
                            crate::dropin::DesignSource::C531(c531)
                        } else {
                            crate::dropin::DesignSource::H523(h523)
                        };
                        jump = crate::dropin_view::show(
                            ui, package, dsrc, dropin_query, dropin_cache, dropin_focus,
                        );
                    }
                    ViewMode::Converter if package.mcu() == Mcu::C531 => {
                        conv_action = crate::c531_view::show(ui, c531, package, c531_status);
                    }
                    _ => crate::inventory_view::show(ui, descriptor),
                }
            });
            self.pending_open = jump;
            self.apply_converter_action(conv_action);
            // Record the frame's edit — UNLESS navigation changed the active
            // project or MCU mid-frame (nav_epoch bumped, history already cleared),
            // which would otherwise push THIS project's pre-snapshot onto the OTHER
            // project's freshly-cleared stack. Also skips note-only changes.
            if self.nav_epoch == nav_before && self.non_g474_changed(&before) {
                self.push_history(before);
            }
            return;
        }

        self.active.design.normalize();

        let phases = self.active.design.phases();
        let external_eevs = self.active.design.external_eevs();
        let fault = self.active.design.first_fault();
        let drive_dac = self.active.design.first_drive_dac();
        let plan = self.active.design.master_triggered_sequencer();
        let phase_timers = self.active.design.phase_timers_and_dem();
        let phase_edges = self.active.design.phase_edges();
        let phase_dac_timers = self.active.design.phase_dac_timers();
        let non_pcm_hrtim = self.active.design.non_pcm_hrtim_uses();
        let phase_shift_links = self.active.design.phase_shift_links();
        let adc_sequencers: Vec<fabric_view::AdcSequencerView> =
            build_adc_sequencer_views(&self.active.design);
        let used_timers: Vec<HrtimId> = phase_timers.iter().map(|(t, _)| *t).collect();
        let all_used: std::collections::HashSet<Resource> = self
            .active
            .design
            .assignments
            .iter()
            .flatten()
            .flat_map(|a| a.consumed())
            .chain(self.active.design.locks.iter().copied())
            .collect();

        egui::SidePanel::right("resources")
            .resizable(true)
            .default_width(340.0)
            .show(ctx, |ui| {
                ui.heading("Resource accounting");
                ui.separator();

                ui.label(format!(
                    "Requirements: {}   Assigned: {}",
                    self.active.design.requirements.len(),
                    self.active.design.assignments.iter().flatten().count()
                ));

                ui.separator();
                ui.label("Used by kind:");
                let used = self.design_used();
                for (kind, vs) in &used {
                    ui.label(format!("  {}: {}", kind, vs.join(", ")));
                }

                ui.separator();
                ui.label("ADC sequencers (claimed):");
                let seqs: Vec<_> = self.active.design.assignments.iter().flatten().filter_map(|a| match a {
                    Assignment::AdcSequencer { adc, kind, trigger, .. } => Some((*adc, *kind, *trigger)),
                    _ => None,
                }).collect();
                if seqs.is_empty() {
                    ui.label("  (none -- add an ADC sequencer requirement)");
                } else {
                    for (adc, kind, trig) in &seqs {
                        ui.label(format!("  {:?} {:?} trig={:?}", adc, kind, trig));
                    }
                }

                ui.separator();
                ui.label("Capability checks:");
                let mut fix_to_apply: Option<Fix> = None;
                for c in self.active.design.warnings() {
                    let (color, glyph) = match c.severity {
                        Severity::Ok => (egui::Color32::from_rgb(100, 200, 120), "OK  "),
                        Severity::Info => (egui::Color32::from_rgb(120, 170, 220), "INFO"),
                        Severity::Warn => (egui::Color32::from_rgb(220, 140, 80), "WARN"),
                    };
                    ui.horizontal_wrapped(|ui| {
                        ui.colored_label(color, format!("  {} {}", glyph, c.message));
                        if let Some(fix) = &c.fix
                            && ui.small_button(fix.label()).clicked() {
                                fix_to_apply = Some(fix.clone());
                            }
                    });
                }
                if let Some(fix) = fix_to_apply {
                    self.mutate(|d| d.apply_fix(fix));
                }

                ui.separator();
                let mut pending_set_pin: Option<(pinout::Signal, pinout::Pin)> = None;
                let mut pending_clear_pin: Option<pinout::Signal> = None;
                let mut pending_clear_all_pins = false;
                ui.horizontal(|ui| {
                    ui.label("Pin map:");
                    if self.active.design.has_pins()
                        && ui.small_button("Clear pin locks").clicked()
                    {
                        pending_clear_all_pins = true;
                    }
                });
                let signals = self.active.design.used_signals();
                if signals.is_empty() {
                    ui.label("  (no pin-mappable signals yet)");
                } else {
                    let unreachable = pinout::unreachable_signals(&signals, self.active.variant);
                    for (idx, s) in signals.iter().enumerate() {
                        let cands = self.active.design.pin_candidates(*s, self.active.variant);
                        let locked = self.active.design.is_pinned(*s);
                        ui.horizontal(|ui| {
                            ui.label(format!("  {:<14} ->", s.name()));
                            let all_cands = pinout::pins_for(*s, self.active.variant);
                            if all_cands.is_empty() {
                                ui.colored_label(
                                    egui::Color32::from_rgb(220, 100, 100),
                                    "(no pin available)",
                                );
                            } else if all_cands.len() == 1 {
                                ui.label(all_cands[0].name());
                            } else {
                                let current = self.active.design.pinned(*s);
                                let label = current
                                    .map(|p| p.name())
                                    .unwrap_or_else(|| format!("{} options", cands.len()));
                                egui::ComboBox::from_id_salt(("pin", idx))
                                    .width(120.0)
                                    .selected_text(label)
                                    .show_ui(ui, |ui| {
                                        if ui
                                            .selectable_label(current.is_none(), "(any)")
                                            .clicked()
                                        {
                                            pending_clear_pin = Some(*s);
                                        }
                                        for p in &all_cands {
                                            let taken_by_other = !cands.contains(p)
                                                && Some(*p) != current;
                                            let label = if taken_by_other {
                                                format!("{} (taken)", p.name())
                                            } else {
                                                p.name()
                                            };
                                            if ui
                                                .add_enabled(
                                                    !taken_by_other,
                                                    egui::Button::selectable(
                                                        Some(*p) == current,
                                                        label,
                                                    ),
                                                )
                                                .clicked()
                                            {
                                                pending_set_pin = Some((*s, *p));
                                            }
                                        }
                                    });
                                if locked {
                                    ui.label(
                                        egui::RichText::new("(locked)").small().strong(),
                                    );
                                } else {
                                    ui.label(
                                        egui::RichText::new("[alt]").weak().small(),
                                    );
                                }
                            }
                        });
                    }
                    let conflicts =
                        pinout::conflicting_signal_pairs(&signals, self.active.variant);
                    if !conflicts.is_empty() {
                        ui.label(
                            egui::RichText::new("  Pin-contention candidates:")
                                .color(egui::Color32::from_rgb(200, 180, 80)),
                        );
                        for (a, b, p) in &conflicts {
                            ui.label(format!(
                                "    {} <-> {} share {}",
                                a.name(),
                                b.name(),
                                p.name()
                            ));
                        }
                    }
                    if !unreachable.is_empty() {
                        ui.colored_label(
                            egui::Color32::from_rgb(220, 100, 100),
                            format!(
                                "  {} signal(s) have no pin on {} -- incompatible chip.",
                                unreachable.len(),
                                self.active.variant.display_label()
                            ),
                        );
                    }
                }
                if let Some((s, p)) = pending_set_pin {
                    self.mutate(|d| d.set_pin(s, p));
                }
                if let Some(s) = pending_clear_pin {
                    self.mutate(|d| d.clear_pin(s));
                }
                if pending_clear_all_pins {
                    self.mutate(|d| d.clear_all_pins());
                }

                ui.separator();
                ui.label("HRTIM CR budget per PCM phase:");
                if phase_timers.is_empty() {
                    ui.label("  (no PCM phases)");
                } else {
                    for (t, dem) in &phase_timers {
                        let usage = cr_usage_for(*t, *dem);
                        ui.label(format!("  {:?}{}:", t, if *dem { " (+DEM)" } else { "" }));
                        for (slot, purpose) in &usage.claims {
                            ui.label(format!("    {:?}: {}", slot, purpose));
                        }
                        let free: Vec<_> = ALL_CR_SLOTS_HELPER
                            .iter()
                            .copied()
                            .filter(|s| !usage.claims.iter().any(|(u, _)| u == s))
                            .collect();
                        if free.is_empty() {
                            ui.colored_label(
                                egui::Color32::from_rgb(220, 140, 80),
                                "    (no free slots)",
                            );
                        } else {
                            ui.label(format!("    free: {:?}", free));
                        }
                    }
                }
            });

        self.render_top_bar(ctx, can_undo, can_redo);
        self.render_status_line(ctx);

        // Requirements + Add + Locks in a resizable top panel. User can
        // drag to give more or less room to the fabric view below.
        let mut to_remove: Option<usize> = None;
        let mut to_set_spec: Option<(usize, RequirementSpec)> = None;
        let mut to_set_assignment: Option<(usize, Assignment)> = None;

        egui::TopBottomPanel::top("requirements")
            .resizable(true)
            .default_height(150.0)
            .min_height(50.0)
            .max_height(500.0)
            .show(ctx, |ui| {
            // Controls first (+ Add, Locks) so they remain visible even
            // when the scroll area below is tall, then the scroll area
            // fills the remaining panel height.
            ui.horizontal_wrapped(|ui| {
                ui.label("+ Add:");
                let mut to_add: Option<RequirementSpec> = None;
                for &opt in RequirementSpec::add_palette() {
                    if ui.small_button(opt.name()).clicked() {
                        to_add = Some(opt);
                    }
                }
                // Collapse the 8 comms families into one menu to keep the
                // horizontal palette tractable.
                ui.menu_button("Comms \u{25BE}", |ui| {
                    for &opt in RequirementSpec::comms_palette() {
                        if ui.button(opt.name()).clicked() {
                            to_add = Some(opt);
                            ui.close();
                        }
                    }
                });
                if let Some(opt) = to_add {
                    self.mutate(|d| {
                        d.add(opt);
                    });
                }
            });
            ui.horizontal_wrapped(|ui| {
                ui.label("Locks:");
                if self.active.design.locks.is_empty() {
                    ui.label(
                        egui::RichText::new("(none)")
                            .weak()
                            .italics(),
                    );
                }
                let locks_snapshot: Vec<Resource> = self.active.design.locks.iter().copied().collect();
                for r in &locks_snapshot {
                    if ui.small_button(format!("{} x", resource_label(*r))).clicked() {
                        self.mutate(|d| d.clear_lock(*r));
                    }
                }
            });
            ui.separator();
            egui::ScrollArea::vertical()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    let n = self.active.design.requirements.len();
                    let mid = n.div_ceil(2);
                    let render_row = |ui: &mut egui::Ui,
                                      i: usize,
                                      design: &Design,
                                      to_remove: &mut Option<usize>,
                                      to_set_spec: &mut Option<(usize, RequirementSpec)>,
                                      to_set_assignment: &mut Option<(usize, Assignment)>| {
                        let spec = design.requirements[i];
                        let candidates = design.candidates_for(i);
                        let current = design.assignments[i].clone();
                        ui.horizontal(|ui| {
                            ui.label(format!("#{:<2}", i + 1));
                            egui::ComboBox::from_id_salt(("spec", i))
                                .width(150.0)
                                .selected_text(spec.name())
                                .show_ui(ui, |ui| {
                                    for &opt in RequirementSpec::palette() {
                                        if ui.selectable_label(opt == spec, opt.name()).clicked() {
                                            // Preserve pinned_sub_timer and
                                            // fault when switching between
                                            // HRTIM roles — otherwise a user
                                            // who pinned TimF and then picks
                                            // "Voltage-mode PWM" loses the pin.
                                            let carried = if let (
                                                RequirementSpec::UseHrtimSub { pinned_sub_timer, fault, .. },
                                                RequirementSpec::UseHrtimSub { role, outputs, .. },
                                            ) = (spec, opt) {
                                                RequirementSpec::UseHrtimSub {
                                                    pinned_sub_timer, role, outputs, fault,
                                                }
                                            } else {
                                                opt
                                            };
                                            *to_set_spec = Some((i, carried));
                                        }
                                    }
                                });
                            // Pinned-sub-timer selector for HRTIM uses. "auto" lets
                            // the solver pick the first free sub-timer; pinning
                            // reserves a specific sub-timer for this role.
                            if let RequirementSpec::UseHrtimSub { pinned_sub_timer, role, outputs, fault } = spec {
                                let label = match pinned_sub_timer {
                                    None => "auto".to_string(),
                                    Some(t) => format!("{:?}", t),
                                };
                                egui::ComboBox::from_id_salt(("hrt-t", i))
                                    .width(70.0)
                                    .selected_text(label)
                                    .show_ui(ui, |ui| {
                                        if ui.selectable_label(pinned_sub_timer.is_none(), "auto").clicked() {
                                            *to_set_spec = Some((
                                                i,
                                                RequirementSpec::UseHrtimSub {
                                                    pinned_sub_timer: None, role, outputs, fault,
                                                },
                                            ));
                                        }
                                        for &t in &[HrtimId::TimA, HrtimId::TimB, HrtimId::TimC, HrtimId::TimD, HrtimId::TimE, HrtimId::TimF] {
                                            let sel = pinned_sub_timer == Some(t);
                                            if ui.selectable_label(sel, format!("{:?}", t)).clicked() {
                                                *to_set_spec = Some((
                                                    i,
                                                    RequirementSpec::UseHrtimSub {
                                                        pinned_sub_timer: Some(t), role, outputs, fault,
                                                    },
                                                ));
                                            }
                                        }
                                    });
                            }
                            // Trigger selector for ADC sequencers — covers
                            // master compares, sub-timer compares, EEVs,
                            // and software. List comes straight from the
                            // ADC-trigger crossbar table.
                            if let RequirementSpec::AdcSequencer { adc, kind, trigger } = spec {
                                let trig_label = match trigger {
                                    TriggerSource::Software => "SW".to_string(),
                                    TriggerSource::Event(ev) => format!("{:?}", ev),
                                };
                                egui::ComboBox::from_id_salt(("trg", i))
                                    .width(100.0)
                                    .selected_text(trig_label)
                                    .show_ui(ui, |ui| {
                                        if ui.selectable_label(
                                            matches!(trigger, TriggerSource::Software),
                                            "Software",
                                        ).clicked() {
                                            *to_set_spec = Some((
                                                i,
                                                RequirementSpec::AdcSequencer {
                                                    adc, kind,
                                                    trigger: TriggerSource::Software,
                                                },
                                            ));
                                        }
                                        for (ev, _) in crate::g474::CROSSBAR_TO_ADC_TRIGGER.iter() {
                                            let ev = *ev;
                                            let selected = matches!(
                                                trigger,
                                                TriggerSource::Event(e) if e == ev
                                            );
                                            if ui.selectable_label(
                                                selected,
                                                format!("{:?}", ev),
                                            ).clicked() {
                                                *to_set_spec = Some((
                                                    i,
                                                    RequirementSpec::AdcSequencer {
                                                        adc, kind,
                                                        trigger: TriggerSource::Event(ev),
                                                    },
                                                ));
                                            }
                                        }
                                    });
                            }
                            if let RequirementSpec::AdcConversion { group, purpose, adc_pref, speed } = spec {
                                let sequencers = design.sequencer_ids();
                                let group_label = sequencers
                                    .iter()
                                    .find(|(id, _)| *id == group)
                                    .map(|(id, idx)| format!("-> #{} (id={})", idx + 1, id))
                                    .unwrap_or_else(|| "-> (no sequencer)".to_string());
                                egui::ComboBox::from_id_salt(("grp", i))
                                    .width(110.0)
                                    .selected_text(group_label)
                                    .show_ui(ui, |ui| {
                                        for (id, idx) in &sequencers {
                                            if ui.selectable_label(
                                                *id == group,
                                                format!("#{} (id={})", idx + 1, id),
                                            ).clicked() {
                                                *to_set_spec = Some((
                                                    i,
                                                    RequirementSpec::AdcConversion {
                                                        group: *id, purpose, adc_pref, speed,
                                                    },
                                                ));
                                            }
                                        }
                                        if sequencers.is_empty() {
                                            ui.label(egui::RichText::new("(add a sequencer first)").weak());
                                        }
                                    });
                                // ADC-unit preference: when parent is a dual
                                // sequencer the pool spans both ADCs, so this
                                // lets the user pin the conversion to one or
                                // the other. For single-ADC parents this is
                                // effectively a no-op but still visible.
                                let adc_label = match adc_pref {
                                    None => "any ADC".to_string(),
                                    Some(a) => format!("ADC{}", a.number()),
                                };
                                egui::ComboBox::from_id_salt(("adcpref", i))
                                    .width(80.0)
                                    .selected_text(adc_label)
                                    .show_ui(ui, |ui| {
                                        if ui.selectable_label(adc_pref.is_none(), "any ADC").clicked() {
                                            *to_set_spec = Some((
                                                i,
                                                RequirementSpec::AdcConversion {
                                                    group, purpose, adc_pref: None, speed,
                                                },
                                            ));
                                        }
                                        for &a in crate::g474::AdcInstance::ALL {
                                            let sel = adc_pref == Some(a);
                                            if ui.selectable_label(sel, format!("ADC{}", a.number())).clicked() {
                                                *to_set_spec = Some((
                                                    i,
                                                    RequirementSpec::AdcConversion {
                                                        group, purpose, adc_pref: Some(a), speed,
                                                    },
                                                ));
                                            }
                                        }
                                    });
                                // Speed preference: any / fast / slow.
                                egui::ComboBox::from_id_salt(("spd", i))
                                    .width(70.0)
                                    .selected_text(speed.short())
                                    .show_ui(ui, |ui| {
                                        for s in [SpeedPref::Any, SpeedPref::Fast, SpeedPref::Slow] {
                                            if ui.selectable_label(speed == s, s.short()).clicked() {
                                                *to_set_spec = Some((
                                                    i,
                                                    RequirementSpec::AdcConversion {
                                                        group, purpose, adc_pref, speed: s,
                                                    },
                                                ));
                                            }
                                        }
                                    });
                            }
                            if let RequirementSpec::UseOpamp { instance, external_vinp, external_vinm, external_vout } = spec {
                                egui::ComboBox::from_id_salt(("op", i))
                                    .width(90.0)
                                    .selected_text(format!("OPAMP{}", instance.number()))
                                    .show_ui(ui, |ui| {
                                        for &o in crate::g474::OpampId::ALL {
                                            if ui.selectable_label(instance == o, format!("OPAMP{}", o.number())).clicked() {
                                                *to_set_spec = Some((
                                                    i,
                                                    RequirementSpec::UseOpamp {
                                                        instance: o,
                                                        external_vinp, external_vinm, external_vout,
                                                    },
                                                ));
                                            }
                                        }
                                    });
                                let mut p = external_vinp;
                                let mut m = external_vinm;
                                let mut o = external_vout;
                                let before = (p, m, o);
                                ui.checkbox(&mut p, "VINP pin");
                                ui.checkbox(&mut m, "VINM pin");
                                ui.checkbox(&mut o, "VOUT pin");
                                if (p, m, o) != before {
                                    *to_set_spec = Some((
                                        i,
                                        RequirementSpec::UseOpamp {
                                            instance,
                                            external_vinp: p,
                                            external_vinm: m,
                                            external_vout: o,
                                        },
                                    ));
                                }
                            }
                            let sel_label = current
                                .as_ref()
                                .map(|a| a.label())
                                .unwrap_or_else(|| "(no compatible option)".to_string());
                            egui::ComboBox::from_id_salt(("asn", i))
                                .width(240.0)
                                .selected_text(sel_label)
                                .show_ui(ui, |ui| {
                                    if candidates.is_empty() {
                                        ui.label(
                                            egui::RichText::new(
                                                "(no candidate -- resource conflict)",
                                            )
                                            .weak(),
                                        );
                                    }
                                    if matches!(spec, RequirementSpec::AdcConversion { .. }) {
                                        render_adc_conversion_candidates(
                                            ui, i, &candidates, &current, to_set_assignment,
                                        );
                                    } else {
                                        for cand in &candidates {
                                            if ui
                                                .selectable_label(
                                                    current.as_ref() == Some(cand),
                                                    cand.label(),
                                                )
                                                .clicked()
                                            {
                                                *to_set_assignment = Some((i, cand.clone()));
                                            }
                                        }
                                    }
                                });
                            if ui.small_button("x").clicked() {
                                *to_remove = Some(i);
                            }
                        });
                    };
                    ui.columns(2, |cols| {
                        for i in 0..mid {
                            render_row(
                                &mut cols[0], i,
                                &self.active.design,
                                &mut to_remove, &mut to_set_spec, &mut to_set_assignment,
                            );
                        }
                        for i in mid..n {
                            render_row(
                                &mut cols[1], i,
                                &self.active.design,
                                &mut to_remove, &mut to_set_spec, &mut to_set_assignment,
                            );
                        }
                    });
                });

        });

        if let Some(i) = to_remove {
            self.mutate(|d| d.remove(i));
        }
        if let Some((i, s)) = to_set_spec {
            self.mutate(|d| d.set_spec(i, s));
        }
        if let Some((i, a)) = to_set_assignment {
            self.mutate(|d| d.set_assignment(i, a));
        }

        // Central panel: the visualization itself, using all remaining
        // vertical space.
        egui::CentralPanel::default().show(ctx, |ui| {
            match self.view {
                ViewMode::Fabric => {
                    let click = fabric_view::show(
                        ui,
                        &Selection {
                            phases: &phases,
                            external_eevs: &external_eevs,
                            phase_edges: &phase_edges,
                            used_timers: &used_timers,
                            fault,
                            drive_dac,
                            used: &all_used,
                            master_trigger_event: plan.as_ref().and_then(|a| match a {
                                Assignment::AdcSequencer { trigger: TriggerSource::Event(ev), .. } => Some(*ev),
                                _ => None,
                            }),
                            plan_label: plan.as_ref().map(|a| a.label()),
                            phase_dac_timers: &phase_dac_timers,
                            adc_sequencers: &adc_sequencers,
                            non_pcm_hrtim: &non_pcm_hrtim,
                            phase_shift_links: &phase_shift_links,
                        },
                        &self.active.design.locks,
                    );
                    if let Some(r) = click {
                        self.mutate(|d| d.toggle_lock(r));
                    }
                }
                ViewMode::Hrtim => {
                    let action = crate::hrtim_view::show(ui, &self.active.design);
                    if let Some(a) = action {
                        use crate::hrtim_view::HrtimAction;
                        match a {
                            HrtimAction::SetSpec(i, s) => self.mutate(|d| d.set_spec(i, s)),
                            HrtimAction::Remove(i) => self.mutate(|d| d.remove(i)),
                            HrtimAction::AddPhaseOn(t) => {
                                self.mutate(|d| {
                                    d.add(RequirementSpec::UseHrtimSub {
                                        pinned_sub_timer: Some(t),
                                        role: HrtimRole::PcmInternal { ext_zcd: false, dem: false },
                                        outputs: OutputMode::Ch1AndCh2,
                                        fault: None,
                                    });
                                });
                            }
                            HrtimAction::AddRoleOn(t, role) => {
                                let outputs = match role {
                                    HrtimRole::VoltageModePwm => OutputMode::Ch1Only,
                                    _ => OutputMode::Ch1AndCh2,
                                };
                                self.mutate(|d| {
                                    d.add(RequirementSpec::UseHrtimSub {
                                        pinned_sub_timer: Some(t),
                                        role, outputs, fault: None,
                                    });
                                });
                            }
                        }
                    }
                }
                ViewMode::Comms => {
                    let action = crate::comms_view::show(ui, &self.active.design, self.active.variant);
                    if let Some(a) = action {
                        use crate::comms_view::CommsAction;
                        match a {
                            CommsAction::SetSpec(i, s) => self.mutate(|d| d.set_spec(i, s)),
                            CommsAction::Remove(i) => self.mutate(|d| d.remove(i)),
                            CommsAction::SetPin(sig, p) => self.mutate(|d| d.set_pin(sig, p)),
                            CommsAction::ClearPin(sig) => self.mutate(|d| d.clear_pin(sig)),
                        }
                    }
                }
                ViewMode::Timers => {
                    let action = crate::timers_view::show(ui, &self.active.design, self.active.variant);
                    if let Some(a) = action {
                        use crate::timers_view::TimersAction;
                        match a {
                            TimersAction::SetSpec(i, s) => self.mutate(|d| d.set_spec(i, s)),
                            TimersAction::Remove(i) => self.mutate(|d| d.remove(i)),
                            TimersAction::SetPin(sig, p) => self.mutate(|d| d.set_pin(sig, p)),
                            TimersAction::ClearPin(sig) => self.mutate(|d| d.clear_pin(sig)),
                        }
                    }
                }
                ViewMode::Waveforms => {
                    crate::waveform_view::show(ui, &self.active.design.assignments);
                }
                ViewMode::Package => {
                    // Header + legend.
                    if let Some(role) = self.picked {
                        ui.horizontal(|ui| {
                            ui.label(
                                egui::RichText::new(format!("Moving: {}", role.description(&self.active.design)))
                                    .color(egui::Color32::from_rgb(220, 170, 60))
                                    .strong(),
                            );
                            ui.label(
                                egui::RichText::new(
                                    "green = direct, teal = semantic change, yellow = cascade, red = blocked",
                                ).weak(),
                            );
                        });
                    } else {
                        ui.label(
                            egui::RichText::new(
                                "Click an assigned pin to pick up its role. Candidate destinations will be colored by impact.",
                            ).weak(),
                        );
                    }
                    let paints = crate::picker::build_pin_paints(
                        &self.active.design, self.active.variant, self.picked,
                    );
                    let action = crate::package_view::show(ui, self.active.variant, &paints);
                    self.handle_package_action(action);
                }
                ViewMode::Inventory => {
                    crate::inventory_view::show(ui, self.active.package.descriptor());
                }
                ViewMode::Catalog => {
                    self.pending_open = crate::catalog_view::show(
                        ui,
                        &mut self.catalog_query,
                        &mut self.active.catalog_demands,
                        &mut self.catalog_eval_cache,
                        &mut self.catalog_sort,
                    );
                }
                ViewMode::Dropin => {
                    self.pending_open = crate::dropin_view::show(
                        ui,
                        self.active.package,
                        crate::dropin::DesignSource::G474(&self.active.design),
                        &mut self.dropin_query,
                        &mut self.dropin_cache,
                        &mut self.dropin_focus,
                    );
                }
                ViewMode::AfTable => {
                    // G474 path doesn't expose the lock UI yet — pin
                    // locking flows through the existing pin_assignments
                    // map on `Design`. Pass None.
                    crate::af_view::show(ui, self.active.package.descriptor().raw, &mut self.af_filter, None);
                }
                ViewMode::Analog => {
                    crate::analog_view::show(
                        ui,
                        self.active.package.descriptor().raw,
                        &mut self.analog_filter,
                        crate::phys_pinout::best_footprint(
                            self.active.package.name(),
                            self.active.package.package_label(),
                        ),
                        &mut self.analog_cache,
                    );
                }
                // Non-G474 views; never selectable while the G474 planner is active.
                ViewMode::Converter | ViewMode::Peripherals => {}
            }
        });
    }
}

// ---------- Forward seed: Demand -> per-family model ----------

/// True if `dem` has the option group `key` enabled.
fn dem_has(dem: &crate::select::DemandInput, key: &'static str) -> bool {
    dem.options.contains(&key)
}

/// Seed a G474 `Design` from demands: comms instances (sequential), a fault per
/// OCP, and one ADC sequencer + N conversions. COMP_PWM is NOT seeded here —
/// G474 complementary PWM is HRTIM, which the planner already seeds by default.
fn seed_g474_into(
    d: &mut Design,
    demands: &[crate::select::DemandInput],
    have: &std::collections::BTreeMap<&'static str, u8>,
) {
    use crate::g474::{I2cId, SpiId, UcpdId, UsartId};
    use crate::requirements::RequirementSpec as R;
    for dem in demands {
        let cur = have.get(dem.kind).copied().unwrap_or(0);
        let add = dem.count.saturating_sub(cur);
        match dem.kind {
            "SERIAL" => {
                for j in 0..add {
                    if let Some(&instance) = UsartId::ALL.get((cur + j) as usize) {
                        d.add(R::UseUsart {
                            instance,
                            flow_control: dem_has(dem, "flow control"),
                            synchronous: dem_has(dem, "sync clock"),
                        });
                    }
                }
            }
            "SPI" => {
                for j in 0..add {
                    if let Some(&instance) = SpiId::ALL.get((cur + j) as usize) {
                        d.add(R::UseSpi {
                            instance,
                            needs_miso: true,
                            needs_nss: dem_has(dem, "chip-select"),
                        });
                    }
                }
            }
            "I2C" => {
                for j in 0..add {
                    if let Some(&instance) = I2cId::ALL.get((cur + j) as usize) {
                        d.add(R::UseI2c { instance, needs_smba: dem_has(dem, "SMBus alert") });
                    }
                }
            }
            "UCPD" => {
                for j in 0..add {
                    if let Some(&instance) = UcpdId::ALL.get((cur + j) as usize) {
                        d.add(R::UseUcpd { instance });
                    }
                }
            }
            "OCP" => {
                for _ in 0..add {
                    d.add(R::ShortCircuitFault);
                }
            }
            "ADC" if add > 0 => {
                let seq = *R::add_palette()
                    .iter()
                    .find(|s| matches!(s, R::AdcSequencer { .. }))
                    .expect("add_palette has an AdcSequencer");
                let seq_id = d.add(seq);
                let tmpl = *R::add_palette()
                    .iter()
                    .find(|s| matches!(s, R::AdcConversion { .. }))
                    .expect("add_palette has an AdcConversion");
                for _ in 0..add {
                    let mut conv = tmpl;
                    if let R::AdcConversion { group, .. } = &mut conv {
                        *group = seq_id;
                    }
                    d.add(conv);
                }
            }
            _ => {}
        }
    }
}

/// Seed a C531 converter plan: complementary-PWM legs on the advanced timers.
/// OCP / ADC-sense routes use the converter view's fabric-aware pickers; comms
/// have no C531 model (see C1 in the seed-mapping spec).
fn seed_c531_into(
    design: &mut crate::c531_design::C531Design,
    demands: &[crate::select::DemandInput],
    have: &std::collections::BTreeMap<&'static str, u8>,
) {
    use crate::c531_design::ConverterLeg;
    use crate::g474::TimId;
    if let Some(dem) = demands.iter().find(|d| d.kind == "COMP_PWM") {
        let cur = have.get("COMP_PWM").copied().unwrap_or(0);
        let advanced = [TimId::Tim1, TimId::Tim8];
        for j in cur..dem.count {
            if let Some(&tim) = advanced.get(j as usize) {
                let mut leg = ConverterLeg::pwm(tim);
                leg.complementary = true;
                leg.dead_time = true;
                design.add_leg(leg);
            }
        }
    }
}

/// Seed H523 / C5A3 pin-locks: greedily place each demanded comms peripheral's
/// signals on the first free pin of the active package. ADC is analog (no AF
/// pin); OCP / COMP_PWM have no H523 model — both skipped.
fn seed_h523_into(
    design: &mut crate::h523_design::H523Design,
    raw: &'static crate::mcu_raw::RawMcuData,
    demands: &[crate::select::DemandInput],
    have: &std::collections::BTreeMap<&'static str, u8>,
) {
    use crate::mcu_pinout::{pins_for, PinId, SignalId};
    use std::collections::BTreeSet;
    let mut taken: BTreeSet<PinId> = design.taken_pins().keys().copied().collect();
    for dem in demands {
        let classes = crate::select::underlying_classes(dem.kind);
        if classes.is_empty() {
            continue; // ADC (analog) / OCP / COMP_PWM — no comms pins to lock
        }
        let mut instances: Vec<&'static str> = raw
            .peripherals
            .iter()
            .map(|p| p.name)
            .filter(|n| {
                classes.iter().any(|c| {
                    n.strip_prefix(*c)
                        .is_some_and(|r| !r.is_empty() && r.bytes().all(|b| b.is_ascii_digit()))
                })
            })
            .collect();
        instances.sort();
        instances.dedup();
        let signals = crate::select::required_signals(dem.kind, &dem.options);
        if signals.is_empty() {
            continue;
        }
        let cur = have.get(dem.kind).copied().unwrap_or(0);
        for j in cur..dem.count {
            let Some(&inst) = instances.get(j as usize) else { break };
            // Declare the use (shows in the Peripherals view) then greedily place
            // each role on the first free pin.
            design.add_use(inst, &signals);
            for &role in &signals {
                if design.locked_pin(inst, role).is_some() {
                    continue;
                }
                if let Some(p) =
                    pins_for(raw, SignalId { peripheral: inst, role }).into_iter().find(|p| !taken.contains(p))
                {
                    design.lock(inst, role, p);
                    taken.insert(p);
                }
            }
        }
    }
}

/// Render ADC-conversion candidates grouped by ADC unit with fast/slow
/// sub-sections, since users typically care about "which ADC" and
/// "fast vs slow" more than the specific channel number.
fn render_adc_conversion_candidates(
    ui: &mut egui::Ui,
    row: usize,
    candidates: &[Assignment],
    current: &Option<Assignment>,
    to_set_assignment: &mut Option<(usize, Assignment)>,
) {
    use crate::g474::{is_fast_adc_channel, AdcInstance};
    let mut by_adc: Vec<(AdcInstance, Vec<&Assignment>)> = Vec::new();
    for adc in AdcInstance::ALL {
        let group: Vec<&Assignment> = candidates
            .iter()
            .filter(|c| matches!(c, Assignment::AdcConversion { adc: a, .. } if a == adc))
            .collect();
        if !group.is_empty() {
            by_adc.push((*adc, group));
        }
    }
    for (adc, mut group) in by_adc {
        // Fast channels first within each ADC.
        group.sort_by_key(|c| match c {
            Assignment::AdcConversion { channel, .. } => {
                (!is_fast_adc_channel(*channel), *channel)
            }
            _ => (true, 0),
        });
        ui.label(
            egui::RichText::new(format!("ADC{}", adc.number()))
                .strong()
                .color(egui::Color32::from_gray(200)),
        );
        for cand in group {
            let Assignment::AdcConversion { channel, .. } = cand else { continue; };
            let pin = crate::pinout::pins_for(
                crate::pinout::Signal::AdcIn { adc, channel: *channel },
                crate::pinout::ChipVariant::G474R,
            )
            .first()
            .map(|p| p.name())
            .unwrap_or_else(|| "?".to_string());
            let tag = if is_fast_adc_channel(*channel) { "fast" } else { "slow" };
            let label = format!("  [{}] IN{:<2} @ {}", tag, channel, pin);
            if ui
                .selectable_label(current.as_ref() == Some(cand), label)
                .clicked()
            {
                *to_set_assignment = Some((row, cand.clone()));
            }
        }
    }
}

fn build_adc_sequencer_views(design: &Design) -> Vec<fabric_view::AdcSequencerView> {
    use fabric_view::{AdcConversionView, AdcSequencerView};
    let mut views: Vec<(u32, AdcSequencerView)> = Vec::new();
    for (idx, asn) in design.assignments.iter().enumerate() {
        let Some(asn) = asn else { continue; };
        if let Assignment::AdcSequencer { adc, kind, trigger, .. } = asn {
            let id = design.ids[idx];
            views.push((
                id,
                AdcSequencerView {
                    adc: *adc,
                    kind: *kind,
                    trigger: *trigger,
                    conversions: Vec::new(),
                },
            ));
        }
    }
    for (idx, asn) in design.assignments.iter().enumerate() {
        let Some(Assignment::AdcConversion { adc, channel, purpose }) = asn else { continue; };
        let group = match design.requirements[idx] {
            RequirementSpec::AdcConversion { group, .. } => group,
            _ => continue,
        };
        if let Some((_, view)) = views.iter_mut().find(|(id, _)| *id == group) {
            let pin = crate::pinout::pins_for(
                crate::pinout::Signal::AdcIn { adc: *adc, channel: *channel },
                crate::pinout::ChipVariant::G474R,
            )
            .first()
            .copied();
            view.conversions.push(AdcConversionView {
                adc: *adc,
                channel: *channel,
                purpose: *purpose,
                pin,
            });
        }
    }
    views.into_iter().map(|(_, v)| v).collect()
}

fn resource_label(r: Resource) -> String {
    match r {
        Resource::Dac(d) => format!("DAC {:?}", d),
        Resource::Comp(c) => format!("{:?}", c),
        Resource::Eev(e) => format!("{:?}", e),
        Resource::Flt(f) => format!("{:?}", f),
        Resource::Timer(t) => format!("{:?}", t),
        Resource::AdcSequencer(a, k) => format!("{:?}.{:?}", a, k),
        Resource::AdcInput(a, c) => format!("{:?}.IN{}", a, c),
        Resource::Pin(p) => format!("pin {}", p.name()),
        Resource::AdcTrigger(t) => format!("{:?}", t),
        Resource::TimerSlot(t, s) => format!("{:?}.{:?}", t, s),
        Resource::TimerCapture(t, c) => format!("{:?}.{:?}", t, c),
        Resource::MasterCompareSlot(n) => format!("Master.MCR{}", n),
        Resource::Opamp(o) => format!("OPAMP{}", o.number()),
        Resource::Spi(s)    => format!("SPI{}", s.number()),
        Resource::I2c(i)    => format!("I2C{}", i.number()),
        Resource::Usart(u)  => format!("USART{}", u.number()),
        Resource::Uart(u)   => format!("UART{}", u.number()),
        Resource::Lpuart(_) => "LPUART1".to_string(),
        Resource::Can(c)    => format!("FDCAN{}", c.number()),
        Resource::Usb       => "USB".to_string(),
        Resource::Ucpd(_)   => "UCPD1".to_string(),
        Resource::Tim(t)    => format!("TIM{}", t.number()),
    }
}

impl PeriPlannerApp {
    fn design_used(&self) -> Vec<(&'static str, Vec<String>)> {
        let mut by_kind = BTreeMap::<&'static str, Vec<String>>::new();
        for a in self.active.design.assignments.iter().flatten() {
            for r in a.consumed() {
                let (k, v) = match r {
                    Resource::Dac(d) => ("DAC", format!("{:?}", d)),
                    Resource::Comp(c) => ("COMP", format!("{:?}", c)),
                    Resource::Eev(e) => ("EEV", format!("{:?}", e)),
                    Resource::Flt(f) => ("FLT", format!("{:?}", f)),
                    Resource::Timer(t) => ("Timer", format!("{:?}", t)),
                    Resource::AdcSequencer(a, k) => ("ADC-seq", format!("{:?}.{:?}", a, k)),
                    Resource::AdcInput(a, c) => ("ADC-in", format!("{:?}.IN{}", a, c)),
                    Resource::Pin(p) => ("Pin", p.name()),
                    Resource::AdcTrigger(t) => ("ADC-trg", format!("{:?}", t)),
                    Resource::TimerSlot(t, s) => ("CR", format!("{:?}.{:?}", t, s)),
                    Resource::TimerCapture(t, c) => ("CPT", format!("{:?}.{:?}", t, c)),
                    Resource::MasterCompareSlot(n) => ("Master-CR", format!("MCR{}", n)),
                    Resource::Opamp(o) => ("OPAMP", format!("OPAMP{}", o.number())),
                    Resource::Spi(s)    => ("SPI", format!("SPI{}", s.number())),
                    Resource::I2c(i)    => ("I2C", format!("I2C{}", i.number())),
                    Resource::Usart(u)  => ("USART", format!("USART{}", u.number())),
                    Resource::Uart(u)   => ("UART", format!("UART{}", u.number())),
                    Resource::Lpuart(_) => ("LPUART", "LPUART1".to_string()),
                    Resource::Can(c)    => ("FDCAN", format!("FDCAN{}", c.number())),
                    Resource::Usb       => ("USB", "USB".to_string()),
                    Resource::Ucpd(_)   => ("UCPD", "UCPD1".to_string()),
                    Resource::Tim(t)    => ("TIM", format!("TIM{}", t.number())),
                };
                by_kind.entry(k).or_default().push(v);
            }
        }
        for vs in by_kind.values_mut() {
            vs.sort();
            vs.dedup();
        }
        by_kind.into_iter().collect()
    }
}

/// Per-phase CR usage summary. Inlined here because the new model carries
/// DEM per-phase while `solver::compute_cr_usage` still takes a global
/// `Intent`. Kept local so we don't rebreak the solver API.
fn cr_usage_for(timer: HrtimId, dem: bool) -> TimerSlotUsage {
    let mut claims = vec![(
        TimerCompareSlot::Cr2,
        "DAC sawtooth step (CMP2, slope comp)",
    )];
    if dem {
        claims.push((
            TimerCompareSlot::Cr4,
            "DEM auto-delayed deadtime (CMP4)",
        ));
    }
    TimerSlotUsage { timer, claims }
}

#[cfg(test)]
mod seed_tests {
    use super::*;
    use crate::requirements::RequirementSpec as R;
    use crate::select::DemandInput;
    use std::collections::BTreeMap;

    fn dem(kind: &'static str, count: u8) -> DemandInput {
        DemandInput { kind, count, with_dma: false, options: vec![] }
    }

    #[test]
    fn to_pin_plan_dispatches_per_active_family() {
        use crate::c531_design::ConverterLeg;
        use crate::g474::TimId;

        // G474: the default fabric design -> family "G4", analog routes present.
        let g4 = Project::new("g4").to_pin_plan();
        assert_eq!(g4.target.family, "G4");
        assert!(!g4.routes.is_empty(), "G474 default carries analog fabric");

        // C531: a bare PWM leg -> family "C5", a materialized TIM1 CH1, no fabric.
        let mut c5 = Project::new("c5");
        c5.mcu = Mcu::C531;
        c5.package = Package::C531R;
        c5.c531_design.add_leg(ConverterLeg::pwm(TimId::Tim1));
        let plan = c5.to_pin_plan();
        assert_eq!(plan.target.family, "C5");
        assert!(plan.placements.iter().any(|p| p.signal.peripheral == "TIM1" && p.signal.role == "CH1"));
        assert!(plan.routes.is_empty(), "a bare PWM leg has no fabric routes");

        // H523: a declared use -> family "H5"; the per-MCU map is read for the active chip.
        let mut h5 = Project::new("h5");
        h5.mcu = Mcu::H523;
        h5.package = Package::H523R;
        h5.h523_designs.entry(Mcu::H523).or_default().add_use("USART1", &["TX"]);
        let plan = h5.to_pin_plan();
        assert_eq!(plan.target.family, "H5");
        assert!(plan.routes.is_empty());
        assert!(plan.placements.iter().any(|p| p.signal.peripheral == "USART1"));
    }

    #[test]
    fn asset_chip_lowers_to_pin_plan_for_codegen() {
        // An any-STM32 part exports/codegens from its generic design (coarse
        // family derived from the descriptor line).
        let mut p = Project::new("h7");
        p.asset_chip = Some("STM32H743ZI".to_string());
        let desc = crate::desc_asset::descriptor_for("STM32H743ZI").unwrap();
        let sig = crate::mcu_pinout::af_rows(desc.raw)
            .map(|r| r.signal)
            .find(|s| s.peripheral == "USART1" && s.role == "TX")
            .expect("H743 has USART1.TX");
        let pin = *crate::mcu_pinout::pins_for(desc.raw, sig).first().unwrap();
        let d = p.asset_designs.entry(Project::asset_key(desc.name)).or_default();
        d.add_use(sig.peripheral, &[sig.role]);
        d.lock(sig.peripheral, sig.role, pin);

        let plan = p.to_pin_plan();
        assert_eq!(plan.target.family, "H7", "coarse family from the descriptor line");
        assert!(
            plan.placements.iter().any(|pl| pl.signal.peripheral == "USART1" && pl.signal.role == "TX"),
            "the locked role lowers into the plan"
        );
        // Codegen is family-generic (open family string) — must produce output.
        assert!(!crate::codegen::generate(&plan).is_empty());
    }

    #[test]
    fn asset_undo_snapshots_and_round_trips() {
        // An arbitrary (any-STM32) part's pin-lock edits are undoable/redoable via
        // the DesignSnapshot::Asset variant.
        let mut app = PeriPlannerApp::default();
        app.active.asset_chip = Some("STM32H743ZI".to_string());
        app.sync_asset_part();
        let desc = app.asset_part.expect("H743 resolves to an asset descriptor");
        let key = Project::asset_key(desc.name);

        let before = app.snapshot_active();
        assert!(matches!(before, DesignSnapshot::Asset(_, _)), "active is an asset chip");
        // Edit: lock a pin on the per-part generic design.
        app.active
            .asset_designs
            .entry(key.clone())
            .or_default()
            .lock("SPI1", "MOSI", crate::mcu_pinout::PinId { port: 'A', num: 7 });
        assert!(app.non_g474_changed(&before), "the lock edit is detected");

        // Undo restores the empty design; redo re-applies the lock.
        app.push_history(before);
        app.undo();
        assert!(app.active.asset_designs.get(&key).cloned().unwrap_or_default().pin_locks.is_empty());
        assert!(!app.redo.is_empty(), "redo available after undo");
        app.redo_op();
        assert_eq!(app.active.asset_designs.get(&key).unwrap().pin_locks.len(), 1, "redo re-applied");
    }

    #[test]
    fn g474_seed_adds_comms_idempotently() {
        let mut d = Design::default(); // starts with 4 HRTIM sub-timers
        seed_g474_into(&mut d, &[dem("SERIAL", 2), dem("SPI", 1)], &BTreeMap::new());
        let usarts = d.requirements.iter().filter(|s| matches!(s, R::UseUsart { .. })).count();
        let spis = d.requirements.iter().filter(|s| matches!(s, R::UseSpi { .. })).count();
        assert_eq!((usarts, spis), (2, 1));
        // Idempotent: with `have` reflecting the current counts, nothing is added.
        let have: BTreeMap<&'static str, u8> = [("SERIAL", 2u8), ("SPI", 1)].into_iter().collect();
        seed_g474_into(&mut d, &[dem("SERIAL", 2), dem("SPI", 1)], &have);
        assert_eq!(d.requirements.iter().filter(|s| matches!(s, R::UseUsart { .. })).count(), 2);
    }

    #[test]
    fn g474_seed_adc_adds_one_sequencer_and_n_conversions() {
        // The default design already carries an ADC sequencer + conversions, so
        // assert the DELTA with `have` reflecting the current count (as
        // design_demand_counts would compute it).
        let mut d = Design::default();
        let before_seq = d.requirements.iter().filter(|s| matches!(s, R::AdcSequencer { .. })).count();
        let before_conv =
            d.requirements.iter().filter(|s| matches!(s, R::AdcConversion { .. })).count();
        let have: BTreeMap<&'static str, u8> = [("ADC", before_conv as u8)].into_iter().collect();
        seed_g474_into(&mut d, &[dem("ADC", before_conv as u8 + 2)], &have);
        let seqs = d.requirements.iter().filter(|s| matches!(s, R::AdcSequencer { .. })).count();
        let convs = d.requirements.iter().filter(|s| matches!(s, R::AdcConversion { .. })).count();
        assert_eq!(seqs - before_seq, 1, "one new sequencer");
        assert_eq!(convs - before_conv, 2, "two new conversions");
    }

    #[test]
    fn c531_seed_adds_complementary_legs() {
        let mut c = crate::c531_design::C531Design::new();
        seed_c531_into(&mut c, &[dem("COMP_PWM", 2)], &BTreeMap::new());
        assert_eq!(c.legs.len(), 2);
        assert!(c.legs.iter().all(|l| l.complementary && l.dead_time));
    }

    #[test]
    fn h523_seed_greedily_places_serial_on_real_pins() {
        let mut h = crate::h523_design::H523Design::new();
        let raw = crate::mcu::Package::H523R.descriptor().raw;
        seed_h523_into(&mut h, raw, &[dem("SERIAL", 1)], &BTreeMap::new());
        // The use is declared (shows in the Peripherals view)...
        assert_eq!(h.uses.len(), 1);
        assert_eq!(h.uses[0].roles, vec!["TX", "RX"]);
        // ...and its TX + RX are placed on distinct real pins.
        assert_eq!(h.pin_locks.len(), 2);
        assert!(h.pin_locks.iter().any(|l| l.role == "TX"));
        assert!(h.pin_locks.iter().any(|l| l.role == "RX"));
        let pins: std::collections::BTreeSet<_> = h.pin_locks.iter().map(|l| l.pin()).collect();
        assert_eq!(pins.len(), 2, "TX and RX must land on distinct pins");
    }

    #[test]
    fn per_mcu_h523_designs_are_independent_and_undo_is_coherent() {
        let mut app = PeriPlannerApp::default(); // starts on G474
        let empty = H523Design::new();

        // Declaring on H523 must NOT appear on C5A3 (no shared field anymore).
        app.set_active_mcu(Mcu::H523);
        app.active.h523_designs.entry(Mcu::H523).or_default().add_use("USART1", &["TX"]);
        app.set_active_mcu(Mcu::C5A3);
        assert!(app.h523(&empty).uses.is_empty(), "C5A3 must not see H523's use");
        app.active.h523_designs.entry(Mcu::C5A3).or_default().add_use("LPUART1", &["TX"]);

        // Back on H523, its own use is intact (and distinct from C5A3's).
        app.set_active_mcu(Mcu::H523);
        assert_eq!(app.h523(&empty).uses.len(), 1);
        assert_eq!(app.h523(&empty).uses[0].peripheral, "USART1");
        assert_eq!(app.active.h523_designs[&Mcu::C5A3].uses[0].peripheral, "LPUART1");

        // Switching MCUs clears undo history (a snapshot of one MCU must never
        // restore into another's live view) — including H523<->C5A3.
        app.history.push(app.snapshot_active());
        app.set_active_mcu(Mcu::C5A3);
        assert!(app.history.is_empty() && app.redo.is_empty(), "MCU switch clears history");

        // Re-selecting the SAME MCU is a no-op: history is preserved (a package
        // switch within an MCU keeps the same design + its undo stack).
        app.history.push(app.snapshot_active());
        app.set_active_mcu(Mcu::C5A3);
        assert_eq!(app.history.len(), 1, "same-MCU re-select keeps history");

        // restore_snapshot lands in the ACTIVE MCU's slot (the cleared-on-switch
        // invariant guarantees active MCU == capture MCU).
        app.set_active_mcu(Mcu::H523);
        let snap = app.snapshot_active(); // H523 with just USART1
        app.active.h523_designs.get_mut(&Mcu::H523).unwrap().add_use("SPI1", &["SCK"]);
        assert_eq!(app.active.h523_designs[&Mcu::H523].uses.len(), 2);
        app.restore_snapshot(snap);
        assert_eq!(app.active.h523_designs[&Mcu::H523].uses.len(), 1, "restore reverts H523 slot");
        assert_eq!(app.active.h523_designs[&Mcu::H523].uses[0].peripheral, "USART1");
    }

    #[test]
    fn projects_are_independent_and_switch_clears_undo() {
        let mut app = PeriPlannerApp::default(); // one project "Untitled" (G474)
        assert!(app.others.is_empty());

        // Edit the active project's design state.
        app.set_active_mcu(Mcu::H523);
        app.active.h523_designs.entry(Mcu::H523).or_default().add_use("USART1", &["TX"]);

        // New project: fresh + independent; the old one is preserved in `others`.
        app.new_project();
        assert_eq!(app.others.len(), 1);
        assert_eq!(app.active.mcu, Mcu::G474, "new project starts on the default chip");
        assert!(app.active.h523_designs.is_empty(), "new project has its own empty design");

        // Building undo history on this project...
        app.history.push(app.snapshot_active());
        assert!(!app.history.is_empty());
        // ...clears when switching projects (no cross-project snapshots).
        app.switch_project(0);
        assert!(app.history.is_empty() && app.redo.is_empty(), "project switch clears undo");

        // Back on the first project with its design intact.
        assert_eq!(app.active.mcu, Mcu::H523);
        assert_eq!(app.active.h523_designs[&Mcu::H523].uses[0].peripheral, "USART1");

        // Delete promotes the other project to active; deleting the last is a no-op.
        app.delete_active();
        assert!(app.others.is_empty());
        let last = app.active.name.clone();
        app.delete_active();
        assert_eq!(app.active.name, last, "deleting the only project is a no-op");
    }

    #[test]
    fn project_blob_round_trips_through_ron() {
        // The persisted shape (RON, via eframe) must round-trip — including the
        // enum-keyed h523 map inside each Project.
        let mut active = Project::new("buck rev C");
        active.mcu = Mcu::H523;
        active.h523_designs.entry(Mcu::H523).or_default().add_use("USART2", &["TX", "RX"]);
        active.h523_designs.entry(Mcu::C5A3).or_default().add_use("LPUART1", &["TX"]);
        let blob = Persisted { active, others: vec![Project::new("scratch")] };
        let ron = ron::ser::to_string(&blob).expect("serialize");
        let back: Persisted = ron::from_str(&ron).expect("deserialize");
        assert_eq!(back.active.name, "buck rev C");
        assert_eq!(back.active.mcu, Mcu::H523);
        assert_eq!(back.active.h523_designs[&Mcu::H523].uses[0].peripheral, "USART2");
        assert_eq!(back.active.h523_designs[&Mcu::C5A3].uses[0].peripheral, "LPUART1");
        assert_eq!(back.others.len(), 1);
        assert_eq!(back.others[0].name, "scratch");
    }

    #[test]
    fn per_project_demands_are_independent_and_round_trip() {
        let mut app = PeriPlannerApp::default();
        // The active project's Part-finder spec is its own.
        app.active.catalog_demands.iter_mut().find(|d| d.kind == "SERIAL").unwrap().count = 3;
        app.new_project();
        assert!(
            app.active.catalog_demands.iter().all(|d| d.count == 0),
            "a new project starts with a fresh (zero) demand spec",
        );
        app.switch_project(0);
        assert_eq!(
            app.active.catalog_demands.iter().find(|d| d.kind == "SERIAL").unwrap().count,
            3,
            "switching back restores the original project's spec",
        );

        // The serde projection round-trips counts, the DMA flag, and options.
        let mut p = Project::new("buck");
        let serial = p.catalog_demands.iter_mut().find(|d| d.kind == "SERIAL").unwrap();
        serial.count = 2;
        serial.with_dma = true;
        serial.set_option("flow control", true);
        let blob = Persisted { active: p, others: vec![] };
        let back: Persisted =
            ron::from_str(&ron::ser::to_string(&blob).unwrap()).unwrap();
        let bs = back.active.catalog_demands.iter().find(|d| d.kind == "SERIAL").unwrap();
        assert_eq!((bs.count, bs.with_dma), (2, true));
        assert!(bs.has_option("flow control"), "option survived the &'static projection");
    }

    #[derive(Default)]
    struct MemStorage(std::collections::HashMap<String, String>);
    impl eframe::Storage for MemStorage {
        fn get_string(&self, k: &str) -> Option<String> {
            self.0.get(k).cloned()
        }
        fn set_string(&mut self, k: &str, v: String) {
            self.0.insert(k.to_string(), v);
        }
        fn flush(&mut self) {}
    }

    #[test]
    fn corrupt_project_blob_is_preserved_not_silently_lost() {
        use eframe::{App as _, Storage as _};

        let mut store = MemStorage::default();
        store.set_string(PROJECTS_KEY, "this is not valid RON".to_string());
        // The fix's basis: a present-but-unparseable blob yields None from
        // get_value yet IS visible via get_string — so "corrupt" ≠ "absent".
        assert!(eframe::get_value::<Persisted>(&store, PROJECTS_KEY).is_none());
        assert!(store.get_string(PROJECTS_KEY).is_some());

        // save() preserves the unparseable bytes under a recovery key (no silent
        // loss) and still writes the fresh blob.
        let mut app = PeriPlannerApp::default();
        app.corrupt_blob = store.get_string(PROJECTS_KEY);
        app.save(&mut store);
        assert_eq!(
            store.get_string(&format!("{PROJECTS_KEY}_recovered")).as_deref(),
            Some("this is not valid RON"),
        );
        assert!(eframe::get_value::<Persisted>(&store, PROJECTS_KEY).is_some(), "fresh blob written");
        assert!(app.corrupt_blob.is_none(), "backed up exactly once");
    }

    #[test]
    fn project_switch_mid_frame_does_not_leak_undo_across_projects() {
        // Regression: the non-G474 frame bracket captures `before` + `nav_before`,
        // then a project switch can happen mid-frame (the switcher is inside
        // render_top_bar). The post-render diff must NOT push the OLD project's
        // snapshot onto the NEW (same-MCU) project's freshly-cleared stack.
        let mut app = PeriPlannerApp::default();
        app.set_active_mcu(Mcu::H523);
        app.active.h523_designs.entry(Mcu::H523).or_default().add_use("USART1", &["TX"]);
        let mut b = Project::new("B");
        b.mcu = Mcu::H523;
        b.h523_designs.entry(Mcu::H523).or_default().add_use("SPI1", &["SCK"]);
        app.others.push(b);

        // Mirror the frame bracket around an in-frame switch.
        let nav_before = app.nav_epoch;
        let before = app.snapshot_active(); // project A's design
        app.switch_project(0); // mid-frame → clears history, bumps nav_epoch
        if app.nav_epoch == nav_before && app.non_g474_changed(&before) {
            app.push_history(before);
        }

        assert!(app.history.is_empty(), "no stale cross-project undo step after an in-frame switch");
        // B is active and intact — an undo can't restore A's design into it.
        assert_eq!(app.active.h523_designs[&Mcu::H523].uses[0].peripheral, "SPI1");
    }
}
