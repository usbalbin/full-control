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
const HISTORY_CAP: usize = 40;

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
}

pub struct PeriPlannerApp {
    design: Design,
    /// Pin-lock state for non-G474 MCUs. Lives separately from `design`
    /// because `Design` is HRTIM/COMP/OPAMP-shaped and would be all-empty
    /// fields here. Future H523 features extend this struct.
    h523_design: H523Design,
    /// C531 timer-PWM converter plan. Like `h523_design`, kept separate from
    /// the HRTIM-shaped G474 `Design` (C531 has no HRTIM).
    c531_design: C531Design,
    mcu: Mcu,
    package: Package,
    variant: ChipVariant,
    view: ViewMode,
    history: Vec<Design>,
    redo: Vec<Design>,
    /// Role "picked up" in the package view, waiting to be dropped on a
    /// candidate pin. Not persisted — ephemeral interaction state.
    picked: Option<crate::picker::PickedRole>,
    /// Filter state for the AfTable view. Ephemeral.
    af_filter: crate::af_view::AfFilter,
    /// Part-finder query state (whole-lineup catalog search). Ephemeral.
    catalog_query: crate::catalog::SearchQuery,
    /// Part-finder constraint rows (verified per-part via Tier-2 solve). Ephemeral.
    catalog_demands: Vec<crate::select::DemandInput>,
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
}

impl Default for PeriPlannerApp {
    fn default() -> Self {
        Self {
            design: Design::default(),
            h523_design: H523Design::new(),
            c531_design: C531Design::new(),
            mcu: Mcu::G474,
            package: Package::G474R,
            variant: ChipVariant::G474R,
            view: ViewMode::Fabric,
            history: Vec::new(),
            redo: Vec::new(),
            picked: None,
            af_filter: Default::default(),
            catalog_query: Default::default(),
            catalog_demands: ["SERIAL", "SPI", "I2C", "ADC", "UCPD", "OCP", "COMP_PWM"]
                .into_iter()
                .map(crate::select::DemandInput::new)
                .collect(),
            catalog_eval_cache: Default::default(),
            catalog_sort: Default::default(),
            pending_open: None,
            asset_part: None,
            dropin_query: Default::default(),
            dropin_cache: Default::default(),
            dropin_focus: None,
        }
    }
}

impl PeriPlannerApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut slf = Self::default();
        if let Some(storage) = cc.storage {
            if let Some(design) = eframe::get_value::<Design>(storage, STORAGE_KEY) {
                slf.design = design;
            }
            // Legacy: older saves persisted the variant separately. Honour
            // it only if it disagrees with the newly-loaded Design (which
            // defaults to G474R via serde when absent). Keeps an older
            // non-G474R session restorable through the bump.
            if let Some(v) = eframe::get_value::<ChipVariant>(storage, "peri_planner_variant_v1") {
                if slf.design.variant != v {
                    slf.design.set_variant(v);
                }
            }
            slf.variant = slf.design.variant;
            if let Some(v) = eframe::get_value::<ViewMode>(storage, "peri_planner_view_v1") {
                slf.view = v;
            }
            if let Some(m) = eframe::get_value::<Mcu>(storage, "peri_planner_mcu_v1") {
                slf.mcu = m;
            }
            if let Some(d) = eframe::get_value::<H523Design>(storage, "peri_planner_h523_design_v1") {
                slf.h523_design = d;
            }
            if let Some(d) = eframe::get_value::<C531Design>(storage, "peri_planner_c531_design_v1") {
                slf.c531_design = d;
            }
            if let Some(p) = eframe::get_value::<Package>(storage, "peri_planner_package_v1") {
                slf.package = p;
            } else {
                // First load after schema bump — derive package from the
                // legacy G474-only ChipVariant.
                slf.package = Package::from_g474_variant(slf.variant);
            }
            // Keep mcu / package consistent if storage drifted.
            if slf.package.mcu() != slf.mcu {
                slf.package = slf.mcu.default_package();
            }
        }
        slf
    }

    /// Wrap a mutation so it records a history entry iff state actually changed.
    fn mutate(&mut self, f: impl FnOnce(&mut Design)) {
        let before = self.design.clone();
        f(&mut self.design);
        if self.design != before {
            self.history.push(before);
            if self.history.len() > HISTORY_CAP {
                self.history.remove(0);
            }
            self.redo.clear();
        }
    }

    fn undo(&mut self) {
        if let Some(prev) = self.history.pop() {
            let current = std::mem::replace(&mut self.design, prev);
            self.redo.push(current);
        }
    }

    fn redo_op(&mut self) {
        if let Some(next) = self.redo.pop() {
            let current = std::mem::replace(&mut self.design, next);
            self.history.push(current);
        }
    }

    fn render_top_bar(&mut self, ctx: &egui::Context, can_undo: bool, can_redo: bool) {
        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("Peripheral planner").strong());
                ui.separator();
                ui.label("MCU:");
                let mut pending_mcu: Option<Mcu> = None;
                egui::ComboBox::from_id_salt("mcu")
                    .selected_text(self.mcu.label())
                    .show_ui(ui, |ui| {
                        for &m in Mcu::ALL {
                            let label = if m.is_implemented() {
                                m.label().to_string()
                            } else {
                                format!("{} (preview)", m.label())
                            };
                            if ui.selectable_label(m == self.mcu, label).clicked() {
                                pending_mcu = Some(m);
                            }
                        }
                    });
                if let Some(m) = pending_mcu {
                    self.mcu = m;
                    // Reset package to the new MCU's default; keeps
                    // self.variant stale on H523 (only consumed by G474
                    // legacy code, which doesn't render on H523).
                    self.package = m.default_package();
                    if let Some(v) = self.package.to_g474_variant() {
                        self.variant = v;
                        self.mutate(|d| d.set_variant(v));
                    }
                    if m == Mcu::C531 {
                        // C531 has a real planner — land on it, not the inventory.
                        if !matches!(
                            self.view,
                            ViewMode::Inventory | ViewMode::AfTable | ViewMode::Converter
                        ) {
                            self.view = ViewMode::Converter;
                        }
                    } else if m != Mcu::G474
                        && !matches!(self.view, ViewMode::Inventory | ViewMode::AfTable)
                    {
                        self.view = ViewMode::Inventory;
                    }
                }
                ui.separator();
                ui.label("Package:");
                let mut pending_package: Option<Package> = None;
                egui::ComboBox::from_id_salt("package")
                    .selected_text(self.package.display_label())
                    .show_ui(ui, |ui| {
                        for p in self.mcu.packages() {
                            if ui.selectable_label(p == self.package, p.display_label()).clicked() {
                                pending_package = Some(p);
                            }
                        }
                    });
                if let Some(p) = pending_package {
                    self.package = p;
                    if let Some(v) = p.to_g474_variant() {
                        self.variant = v;
                        self.mutate(|d| d.set_variant(v));
                    }
                }
                ui.separator();
                if ui.add_enabled(can_undo, egui::Button::new("Undo")).clicked() {
                    self.undo();
                }
                if ui.add_enabled(can_redo, egui::Button::new("Redo")).clicked() {
                    self.redo_op();
                }
                ui.separator();
                ui.label("View:");
                if self.mcu == Mcu::G474 {
                    ui.selectable_value(&mut self.view, ViewMode::Fabric, "Power fabric");
                    ui.selectable_value(&mut self.view, ViewMode::Hrtim, "HRTIM");
                    ui.selectable_value(&mut self.view, ViewMode::Comms, "Comms");
                    ui.selectable_value(&mut self.view, ViewMode::Timers, "Timers");
                    ui.selectable_value(&mut self.view, ViewMode::Waveforms, "Waveforms");
                    ui.selectable_value(&mut self.view, ViewMode::Package, "Package");
                }
                if self.mcu == Mcu::C531 {
                    ui.selectable_value(&mut self.view, ViewMode::Converter, "Converter");
                }
                ui.selectable_value(&mut self.view, ViewMode::Inventory, "Inventory");
                ui.selectable_value(&mut self.view, ViewMode::AfTable, "Pin / AF");
                ui.selectable_value(&mut self.view, ViewMode::Catalog, "Part finder");
                ui.selectable_value(&mut self.view, ViewMode::Dropin, "Drop-in finder");
                ui.separator();
                if ui.button("Export").clicked() {
                    let text = self.design.export_summary();
                    ui.ctx().copy_text(text);
                }
            });
        });
    }

    /// Read-only browser for a non-compiled catalog part opened from a finder.
    /// Renders Inventory / Pin-AF from the lineup descriptor asset — no planner,
    /// no fabric (asset descriptors carry neither). The Part finder stays
    /// available so the user can keep browsing; clicking another part re-targets
    /// the browser (or, for a compiled part, lands in its planner next frame).
    fn render_asset_browser(
        &mut self,
        ctx: &egui::Context,
        desc: &'static crate::mcu::McuDescriptor,
    ) {
        egui::TopBottomPanel::top("asset_browser_top").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.strong(format!("Browsing {}", desc.name));
                ui.label(
                    egui::RichText::new("read-only · package-letter pinout (flash-invariant)")
                        .small()
                        .weak(),
                );
                if ui.button("✕ Close").clicked() {
                    self.asset_part = None;
                }
                ui.separator();
                ui.label("View:");
                ui.selectable_value(&mut self.view, ViewMode::Inventory, "Inventory");
                ui.selectable_value(&mut self.view, ViewMode::AfTable, "Pin / AF");
                ui.selectable_value(&mut self.view, ViewMode::Catalog, "Part finder");
            });
        });

        let view = self.view;
        let af_filter = &mut self.af_filter;
        let catalog_query = &mut self.catalog_query;
        let catalog_demands = &mut self.catalog_demands;
        let catalog_cache = &mut self.catalog_eval_cache;
        let catalog_sort = &mut self.catalog_sort;
        let mut open: Option<String> = None;
        egui::CentralPanel::default().show(ctx, |ui| match view {
            ViewMode::AfTable => {
                crate::af_view::show(ui, desc.raw, af_filter, None);
            }
            ViewMode::Catalog => {
                open = crate::catalog_view::show(
                    ui, catalog_query, catalog_demands, catalog_cache, catalog_sort,
                );
            }
            _ => crate::inventory_view::show(ui, desc),
        });
        self.pending_open = open;
    }

    fn handle_package_action(&mut self, action: Option<crate::package_view::Action>) {
        use crate::package_view::Action;
        use crate::picker;
        match action {
            Some(Action::Click(pin)) => {
                match self.picked {
                    None => {
                        // Try to identify a role from the signal on this pin.
                        let pin_signals = picker::current_pin_signals(&self.design, self.variant);
                        if let Some(sig) = pin_signals.get(&pin).copied() {
                            self.picked = picker::role_for_pin(pin, sig, self.variant, &self.design);
                        }
                    }
                    Some(role) => {
                        // Clicking the source pin cancels the pick.
                        if pin == role.current_pin() {
                            self.picked = None;
                            return;
                        }
                        if let Some(mv) = picker::resolve_move(&self.design, self.variant, role, pin) {
                            let variant = self.variant;
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
    /// outside the `Design` undo stack (like `h523_design`) — the converter
    /// plan is a separate model.
    fn apply_converter_action(&mut self, action: Option<ConverterAction>) {
        let Some(action) = action else { return };
        match action {
            ConverterAction::AddLeg => {
                // Default a new leg to the first advanced-control timer on the
                // chip (TIM1 on C531) so OCP routing is available out of the box.
                let default_tim = self
                    .package
                    .descriptor()
                    .timers
                    .iter()
                    .filter(|t| t.kind == crate::mcu::TimerKind::Advanced)
                    .find_map(|t| TimId::ALL.iter().copied().find(|id| id.number() == t.number))
                    .unwrap_or(TimId::Tim1);
                self.c531_design.add_leg(ConverterLeg::pwm(default_tim));
            }
            ConverterAction::RemoveLeg(i) => self.c531_design.remove_leg(i),
            ConverterAction::SetLeg(i, leg) => {
                if let Some(slot) = self.c531_design.legs.get_mut(i) {
                    *slot = leg;
                }
            }
        }
    }
}

impl eframe::App for PeriPlannerApp {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, STORAGE_KEY, &self.design);
        eframe::set_value(storage, "peri_planner_variant_v1", &self.variant);
        eframe::set_value(storage, "peri_planner_view_v1", &self.view);
        eframe::set_value(storage, "peri_planner_mcu_v1", &self.mcu);
        eframe::set_value(storage, "peri_planner_package_v1", &self.package);
        eframe::set_value(storage, "peri_planner_h523_design_v1", &self.h523_design);
        eframe::set_value(storage, "peri_planner_c531_design_v1", &self.c531_design);
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Open a part clicked in a finder last frame (deferred so we never switch
        // the active chip mid-render). A compiled part becomes the active planner
        // chip; any other opens the read-only descriptor browser.
        if let Some(name) = self.pending_open.take() {
            if let Some(pkg) = Package::for_chip_name(&name) {
                self.mcu = pkg.mcu();
                self.package = pkg;
                if let Some(v) = pkg.to_g474_variant() {
                    self.variant = v;
                }
                self.asset_part = None;
                self.view = ViewMode::Inventory;
            } else if let Some(d) = crate::desc_asset::descriptor_for(&name) {
                self.asset_part = Some(d);
                self.view = ViewMode::Inventory;
            }
        }

        // Read-only browse mode for a non-compiled part: a self-contained panel,
        // bypassing both the G474 planner and the per-family thin path.
        if let Some(desc) = self.asset_part {
            self.render_asset_browser(ctx, desc);
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

        if self.mcu != Mcu::G474 {
            self.render_top_bar(ctx, can_undo, can_redo);
            let descriptor = self.package.descriptor();
            let package = self.package;
            let view = self.view;
            let af_filter = &mut self.af_filter;
            let h523 = &mut self.h523_design;
            let c531 = &self.c531_design;
            let catalog_query = &mut self.catalog_query;
            let catalog_demands = &mut self.catalog_demands;
            let catalog_cache = &mut self.catalog_eval_cache;
            let catalog_sort = &mut self.catalog_sort;
            let mcu = self.mcu;
            let dropin_query = &mut self.dropin_query;
            let dropin_cache = &mut self.dropin_cache;
            let dropin_focus = &mut self.dropin_focus;
            let mut jump = None;
            let mut conv_action: Option<ConverterAction> = None;
            egui::CentralPanel::default().show(ctx, |ui| {
                match view {
                    ViewMode::AfTable => {
                        crate::af_view::show(ui, descriptor.raw, af_filter, Some(h523));
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
                        conv_action = crate::c531_view::show(ui, c531, package);
                    }
                    _ => crate::inventory_view::show(ui, descriptor),
                }
            });
            self.pending_open = jump;
            self.apply_converter_action(conv_action);
            return;
        }

        self.design.normalize();

        let phases = self.design.phases();
        let external_eevs = self.design.external_eevs();
        let fault = self.design.first_fault();
        let drive_dac = self.design.first_drive_dac();
        let plan = self.design.master_triggered_sequencer();
        let phase_timers = self.design.phase_timers_and_dem();
        let phase_edges = self.design.phase_edges();
        let phase_dac_timers = self.design.phase_dac_timers();
        let non_pcm_hrtim = self.design.non_pcm_hrtim_uses();
        let phase_shift_links = self.design.phase_shift_links();
        let adc_sequencers: Vec<fabric_view::AdcSequencerView> =
            build_adc_sequencer_views(&self.design);
        let used_timers: Vec<HrtimId> = phase_timers.iter().map(|(t, _)| *t).collect();
        let all_used: std::collections::HashSet<Resource> = self
            .design
            .assignments
            .iter()
            .flatten()
            .flat_map(|a| a.consumed())
            .chain(self.design.locks.iter().copied())
            .collect();

        egui::SidePanel::right("resources")
            .resizable(true)
            .default_width(340.0)
            .show(ctx, |ui| {
                ui.heading("Resource accounting");
                ui.separator();

                ui.label(format!(
                    "Requirements: {}   Assigned: {}",
                    self.design.requirements.len(),
                    self.design.assignments.iter().flatten().count()
                ));

                ui.separator();
                ui.label("Used by kind:");
                let used = self.design_used();
                for (kind, vs) in &used {
                    ui.label(format!("  {}: {}", kind, vs.join(", ")));
                }

                ui.separator();
                ui.label("ADC sequencers (claimed):");
                let seqs: Vec<_> = self.design.assignments.iter().flatten().filter_map(|a| match a {
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
                for c in self.design.warnings() {
                    let (color, glyph) = match c.severity {
                        Severity::Ok => (egui::Color32::from_rgb(100, 200, 120), "OK  "),
                        Severity::Info => (egui::Color32::from_rgb(120, 170, 220), "INFO"),
                        Severity::Warn => (egui::Color32::from_rgb(220, 140, 80), "WARN"),
                    };
                    ui.horizontal_wrapped(|ui| {
                        ui.colored_label(color, format!("  {} {}", glyph, c.message));
                        if let Some(fix) = &c.fix {
                            if ui.small_button(fix.label()).clicked() {
                                fix_to_apply = Some(fix.clone());
                            }
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
                    if self.design.has_pins()
                        && ui.small_button("Clear pin locks").clicked()
                    {
                        pending_clear_all_pins = true;
                    }
                });
                let signals = self.design.used_signals();
                if signals.is_empty() {
                    ui.label("  (no pin-mappable signals yet)");
                } else {
                    let unreachable = pinout::unreachable_signals(&signals, self.variant);
                    for (idx, s) in signals.iter().enumerate() {
                        let cands = self.design.pin_candidates(*s, self.variant);
                        let locked = self.design.is_pinned(*s);
                        ui.horizontal(|ui| {
                            ui.label(format!("  {:<14} ->", s.name()));
                            let all_cands = pinout::pins_for(*s, self.variant);
                            if all_cands.is_empty() {
                                ui.colored_label(
                                    egui::Color32::from_rgb(220, 100, 100),
                                    "(no pin available)",
                                );
                            } else if all_cands.len() == 1 {
                                ui.label(all_cands[0].name());
                            } else {
                                let current = self.design.pinned(*s);
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
                        pinout::conflicting_signal_pairs(&signals, self.variant);
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
                                self.variant.display_label()
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
                if self.design.locks.is_empty() {
                    ui.label(
                        egui::RichText::new("(none)")
                            .weak()
                            .italics(),
                    );
                }
                let locks_snapshot: Vec<Resource> = self.design.locks.iter().copied().collect();
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
                    let n = self.design.requirements.len();
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
                                &self.design,
                                &mut to_remove, &mut to_set_spec, &mut to_set_assignment,
                            );
                        }
                        for i in mid..n {
                            render_row(
                                &mut cols[1], i,
                                &self.design,
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
                        &self.design.locks,
                    );
                    if let Some(r) = click {
                        self.mutate(|d| d.toggle_lock(r));
                    }
                }
                ViewMode::Hrtim => {
                    let action = crate::hrtim_view::show(ui, &self.design);
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
                    let action = crate::comms_view::show(ui, &self.design, self.variant);
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
                    let action = crate::timers_view::show(ui, &self.design, self.variant);
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
                    crate::waveform_view::show(ui, &self.design.assignments);
                }
                ViewMode::Package => {
                    // Header + legend.
                    if let Some(role) = self.picked {
                        ui.horizontal(|ui| {
                            ui.label(
                                egui::RichText::new(format!("Moving: {}", role.description(&self.design)))
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
                        &self.design, self.variant, self.picked,
                    );
                    let action = crate::package_view::show(ui, self.variant, &paints);
                    self.handle_package_action(action);
                }
                ViewMode::Inventory => {
                    crate::inventory_view::show(ui, self.package.descriptor());
                }
                ViewMode::Catalog => {
                    self.pending_open = crate::catalog_view::show(
                        ui,
                        &mut self.catalog_query,
                        &mut self.catalog_demands,
                        &mut self.catalog_eval_cache,
                        &mut self.catalog_sort,
                    );
                }
                ViewMode::Dropin => {
                    self.pending_open = crate::dropin_view::show(
                        ui,
                        self.package,
                        crate::dropin::DesignSource::G474(&self.design),
                        &mut self.dropin_query,
                        &mut self.dropin_cache,
                        &mut self.dropin_focus,
                    );
                }
                ViewMode::AfTable => {
                    // G474 path doesn't expose the lock UI yet — pin
                    // locking flows through the existing pin_assignments
                    // map on `Design`. Pass None.
                    crate::af_view::show(ui, self.package.descriptor().raw, &mut self.af_filter, None);
                }
                // C531-only view; never selectable while the G474 planner is active.
                ViewMode::Converter => {}
            }
        });
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
        for a in self.design.assignments.iter().flatten() {
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
