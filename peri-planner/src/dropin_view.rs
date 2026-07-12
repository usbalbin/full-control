//! "Drop-in finder" — given the current part and its pin allocation, list other
//! STM32 parts that are board-level pin-compatible replacements (same footprint,
//! same power pins, every wired function serviceable on the same physical pin).
//! A thin renderer over [`crate::dropin`]; all matching logic lives there.

use crate::dropin::{
    self, DesignSource, DropinCache, DropinQuery, PosOutcome, PosResult, PowerMode, SourceKind,
    Strictness,
};
use crate::mcu::Package;
use eframe::egui;

const C_OK: egui::Color32 = egui::Color32::from_rgb(100, 200, 120);
const C_CLASS: egui::Color32 = egui::Color32::from_rgb(90, 180, 190);
const C_WARN: egui::Color32 = egui::Color32::from_rgb(210, 180, 80);
const C_FAIL: egui::Color32 = egui::Color32::from_rgb(220, 100, 100);
const C_DIM: egui::Color32 = egui::Color32::from_gray(140);

fn outcome_color(o: &PosOutcome) -> egui::Color32 {
    match o {
        PosOutcome::PowerOk | PosOutcome::ServiceableExact | PosOutcome::DedicatedOk => C_OK,
        PosOutcome::ServiceableClassRole | PosOutcome::ServiceableClassOnly => C_CLASS,
        PosOutcome::PowerAsym | PosOutcome::DedicatedUnverified => C_WARN,
        PosOutcome::PowerMismatch | PosOutcome::FunctionFail | PosOutcome::DedicatedMismatch => {
            C_FAIL
        }
        _ => C_DIM,
    }
}

fn outcome_label(o: &PosOutcome) -> &'static str {
    match o {
        PosOutcome::PowerOk => "power ok",
        PosOutcome::PowerMismatch => "POWER MISMATCH",
        PosOutcome::ServiceableExact => "exact",
        PosOutcome::ServiceableClassRole => "class+role",
        PosOutcome::ServiceableClassOnly => "class",
        PosOutcome::FunctionFail => "UNSERVICEABLE",
        PosOutcome::UnusedOk => "free",
        PosOutcome::PowerAsym => "power-asym",
        PosOutcome::UnusedRepurposed => "repurposed",
        PosOutcome::DedicatedOk => "reset/boot ok",
        PosOutcome::DedicatedMismatch => "DEDICATED MISMATCH",
        PosOutcome::DedicatedUnverified => "reset/boot ?",
        PosOutcome::DontCare => "—",
    }
}

/// A row worth showing in the per-pin diff (skip the don't-care noise).
fn is_interesting(p: &PosResult) -> bool {
    !matches!(p.outcome, PosOutcome::DontCare | PosOutcome::UnusedOk)
}

fn combo<T: Copy + PartialEq>(
    ui: &mut egui::Ui,
    id: &str,
    cur: &mut T,
    opts: &[T],
    label: impl Fn(T) -> &'static str,
) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(label(*cur))
        .show_ui(ui, |ui| {
            for &o in opts {
                if ui.selectable_label(*cur == o, label(o)).clicked() {
                    *cur = o;
                }
            }
        });
}

/// Render the drop-in finder. Returns a package to jump to if the user clicked a
/// bridge-reachable candidate.
pub fn show(
    ui: &mut egui::Ui,
    package: Package,
    src: DesignSource,
    q: &mut DropinQuery,
    cache: &mut DropinCache,
    focus: &mut Option<String>,
) -> Option<String> {
    show_named(ui, package.chip_prefix(), package.name(), package.package_label(), src, q, cache, focus)
}

/// Drop-in finder for a source identified by `(prefix, name, label)` — works for
/// both a compiled `Package` (via [`show`]) and an arbitrary any-STM32 part.
#[allow(clippy::too_many_arguments)]
pub fn show_named(
    ui: &mut egui::Ui,
    prefix: &str,
    name: &str,
    label: &str,
    src: DesignSource,
    q: &mut DropinQuery,
    cache: &mut DropinCache,
    focus: &mut Option<String>,
) -> Option<String> {
    let mut open: Option<String> = None;
    ui.heading("Drop-in finder");
    ui.label(
        "Pin-compatible replacements for the current part: same footprint, same \
         power pins, and every wired function serviceable on the SAME physical pin \
         (no re-route).",
    );
    ui.label(
        egui::RichText::new(
            "Matches are pin-function compatible — NOT pad-electrical-verified \
             (stm32-data carries no 5V-tolerance / pad-type data).",
        )
        .small()
        .color(C_DIM),
    );
    ui.separator();

    let Some(profile) = dropin::build_source_profile_named(&src, name, label) else {
        ui.colored_label(
            C_WARN,
            format!(
                "No physical-pinout data for {} ({}). Cannot run the drop-in search.",
                prefix, label
            ),
        );
        return None;
    };

    // Source summary.
    let total = profile.footprint.pins.len();
    let required = profile.required_count();
    let wired = profile.wired.len();
    ui.horizontal(|ui| {
        ui.strong(format!(
            "Source: {} ({})",
            prefix,
            profile.footprint.pkg
        ));
        ui.label(format!(
            "· {total} pins · {wired} wired · {required} positions to satisfy (power + allocated)"
        ));
    });
    if wired == 0 {
        ui.label(
            egui::RichText::new(
                "No pins are allocated yet — matching on footprint + power only. \
                 Lock peripheral pins (Pin / AF view) to constrain by function.",
            )
            .small()
            .color(C_DIM),
        );
    }

    // Controls.
    ui.horizontal(|ui| {
        ui.label("Match:");
        combo(ui, "dropin_strict", &mut q.strictness, &Strictness::ALL, Strictness::label);
        ui.label("Power pins:");
        combo(ui, "dropin_power", &mut q.power_mode, &PowerMode::ALL, PowerMode::label);
        ui.separator();
        ui.label("Family:");
        let cur = q.family.clone().unwrap_or_else(|| "(any)".to_string());
        egui::ComboBox::from_id_salt("dropin_family")
            .selected_text(cur)
            .show_ui(ui, |ui| {
                if ui.selectable_label(q.family.is_none(), "(any)").clicked() {
                    q.family = None;
                }
                for fam in crate::catalog::families() {
                    if ui.selectable_label(q.family.as_deref() == Some(fam), fam).clicked() {
                        q.family = Some(fam.to_string());
                    }
                }
            });
        ui.checkbox(&mut q.same_family_only, "Same family only");
        if ui.button("Reset").clicked() {
            *q = DropinQuery::default();
        }
    });
    ui.separator();

    let results = cache.results(&profile, q);
    let compatible = results.iter().filter(|c| c.compatible).count();
    ui.label(format!(
        "{compatible} pin-compatible {} (same {} footprint) of {} candidates examined. \
         Strictness: {} · Power: {}.",
        if compatible == 1 { "part" } else { "parts" },
        profile.footprint.pkg,
        results.len(),
        q.strictness.label(),
        q.power_mode.label(),
    ));

    const CAP: usize = 300;
    egui::ScrollArea::vertical().show(ui, |ui| {
        egui::Grid::new("dropin_results")
            .striped(true)
            .num_columns(7)
            .spacing([14.0, 3.0])
            .show(ui, |ui| {
                for h in ["Part", "Family", "Package", "Match", "Warn", "Notes", "Pins"] {
                    ui.strong(h);
                }
                ui.end_row();

                for c in results.iter().filter(|c| c.compatible).take(CAP) {
                    // Every candidate is openable (compiled planner or read-only
                    // lineup-descriptor browser).
                    if ui
                        .selectable_label(false, &c.name)
                        .on_hover_text("Open in Inventory / Pin-AF")
                        .clicked()
                    {
                        open = Some(c.name.clone());
                    }
                    ui.label(&c.family);
                    ui.label(&c.footprint);
                    let frac = format!("{}/{}", c.n_satisfied, c.n_required);
                    ui.colored_label(if c.n_satisfied == c.n_required { C_OK } else { C_DIM }, frac);
                    if c.warnings > 0 {
                        ui.colored_label(C_WARN, c.warnings.to_string())
                            .on_hover_text("power-asymmetry warnings (unused source pin is power here)");
                    } else {
                        ui.label("—");
                    }
                    if c.reset_boot_unverified {
                        ui.colored_label(C_WARN, "reset/boot ?").on_hover_text(
                            "Cross-family swap: stm32-data omits NRST/BOOT0 for a family in \
                             this pair — verify the reset/boot net manually.",
                        );
                    } else {
                        ui.label(if c.same_family { "same family" } else { "—" });
                    }
                    // Diff toggle. The key is (name, footprint): one part name can
                    // yield two rows (e.g. LQFP64_GP + LQFP64_N share a name).
                    let key = format!("{}|{}", c.name, c.footprint);
                    let open = focus.as_deref() == Some(key.as_str());
                    if ui
                        .selectable_label(open, if open { "hide ▾" } else { "pins ▸" })
                        .clicked()
                    {
                        *focus = if open { None } else { Some(key.clone()) };
                    }
                    ui.end_row();

                    // Inline per-pin diff for the focused candidate.
                    if open {
                        ui.label("");
                        let diff: Vec<&PosResult> =
                            c.positions.iter().filter(|p| is_interesting(p)).collect();
                        egui::Grid::new(format!("dropin_diff_{key}"))
                            .num_columns(3)
                            .spacing([10.0, 1.0])
                            .show(ui, |ui| {
                                for p in diff {
                                    let kind = match p.kind {
                                        SourceKind::Power => "pwr",
                                        SourceKind::Allocated => "fn",
                                        SourceKind::UnusedGpio => "free",
                                        SourceKind::Dedicated => "sys",
                                        SourceKind::Nc => "nc",
                                    };
                                    ui.label(format!("pin {} [{kind}]", p.position));
                                    ui.colored_label(outcome_color(&p.outcome), outcome_label(&p.outcome));
                                    ui.label(egui::RichText::new(&p.detail).small().color(C_DIM));
                                    ui.end_row();
                                }
                            });
                        // Span the remaining columns then end the outer row.
                        for _ in 0..5 {
                            ui.label("");
                        }
                        ui.end_row();
                    }
                }
            });
        if compatible > CAP {
            ui.label(format!("… and {} more compatible — narrow by family.", compatible - CAP));
        }
    });

    open
}
