//! Generic **interactive pin-map planner** — the "deep planner" for every family
//! that uses the generic [`H523Design`] pin-lock model (H523 / C5A3 and any
//! arbitrary any-STM32 part). Click a declared peripheral role, then click a pin
//! on the real footprint to place it; conflicts are resolved by the generic
//! [`crate::mcu_pinout`] engine — no `picker.rs`, no typed `Signal`, so it works
//! for ANY MCU straight from the descriptor + the lineup pinout asset.
//!
//! This is the family-agnostic counterpart of the G474-only typed pin picker: it
//! deliberately does NOT do the G474 picker's cascade/swap (that is HRTIM-fabric
//! specific). Placement here is the simple, universal case — a role goes on a
//! free-or-own candidate pin.

use std::collections::HashMap;

use eframe::egui::{self, Color32, Stroke};

use crate::h523_design::H523Design;
use crate::mcu_pinout::{af_rows, pins_for, PinId, SignalId};
use crate::mcu_raw::RawMcuData;
use crate::package_view::{Action, PinPaint};
use crate::phys_pinout::PinoutRecord;
use crate::pinout::Pin;

const LOCKED_FILL: Color32 = Color32::from_rgb(70, 150, 95);
const CAND_FILL: Color32 = Color32::from_rgb(70, 130, 200);
const PICK_BORDER: Color32 = Color32::from_rgb(240, 240, 240);
const GREEN: Color32 = Color32::from_rgb(110, 200, 130);
const DIM: Color32 = Color32::from_gray(150);

fn as_pin(p: PinId) -> Pin {
    Pin::new(p.port, p.num)
}

/// The `&'static` SignalId for a (peripheral, role) pair, found in the descriptor
/// (the generic pin fns key on `&'static str`; a clicked role is owned Strings).
fn static_sig(raw: &'static RawMcuData, peripheral: &str, role: &str) -> Option<SignalId> {
    af_rows(raw)
        .map(|r| r.signal)
        .find(|s| s.peripheral == peripheral && s.role == role)
}

/// One deferred edit produced during rendering (egui renders the immutable
/// design, we apply after — same idiom as `peripherals_view`).
enum Edit {
    Pick(String, String),
    Unpick,
    Place(String, String, PinId),
    Unlock(String, String),
}

pub fn show(
    ui: &mut egui::Ui,
    raw: &'static RawMcuData,
    footprint: Option<&'static PinoutRecord>,
    design: &mut H523Design,
    picked: &mut Option<(String, String)>,
) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{} — pin map", raw.name)).strong());
        ui.label(
            egui::RichText::new("click a role, then a highlighted pin to place it")
                .weak()
                .small(),
        );
    });
    ui.separator();

    // Owned snapshots so the design is free to mutate after the click.
    let roles: Vec<(String, String)> = design
        .declared_signals()
        .into_iter()
        .map(|(p, r)| (p.to_string(), r.to_string()))
        .collect();
    let taken: HashMap<PinId, (String, String)> = design
        .taken_pins()
        .into_iter()
        .map(|(pin, (p, r))| (pin, (p.to_string(), r.to_string())))
        .collect();

    // Candidate pins for the picked role: reachable AND free (or already its own).
    let picked_sig = picked.as_ref().and_then(|(p, r)| static_sig(raw, p, r));
    let candidates: Vec<PinId> = picked_sig
        .map(|sig| {
            pins_for(raw, sig)
                .into_iter()
                .filter(|pin| match taken.get(pin) {
                    None => true,
                    Some((p, r)) => p == sig.peripheral && r == sig.role,
                })
                .collect()
        })
        .unwrap_or_default();
    let picked_pin = picked
        .as_ref()
        .and_then(|(p, r)| design.locked_pin(p, r));

    let mut edit: Option<Edit> = None;

    egui::SidePanel::left("pin_map_roles")
        .resizable(false)
        .exact_width(230.0)
        .show_inside(ui, |ui| {
            if roles.is_empty() {
                ui.label(
                    egui::RichText::new(
                        "No peripherals declared yet. Add them in the Peripherals tab, then place their pins here.",
                    )
                    .weak(),
                );
                return;
            }
            ui.label(egui::RichText::new("Roles").strong());
            egui::ScrollArea::vertical().show(ui, |ui| {
                for (p, r) in &roles {
                    let is_picked = picked.as_ref() == Some(&(p.clone(), r.clone()));
                    let placed = design.locked_pin(p, r);
                    ui.horizontal(|ui| {
                        let label = egui::RichText::new(format!("{p}.{r}"));
                        let label = if is_picked { label.strong().color(GREEN) } else { label };
                        if ui.selectable_label(is_picked, label).clicked() {
                            edit = Some(if is_picked {
                                Edit::Unpick
                            } else {
                                Edit::Pick(p.clone(), r.clone())
                            });
                        }
                        match placed {
                            Some(pin) => {
                                ui.label(egui::RichText::new(pin.name()).color(GREEN).small());
                                if ui.small_button("✕").on_hover_text("unplace").clicked() {
                                    edit = Some(Edit::Unlock(p.clone(), r.clone()));
                                }
                            }
                            None => {
                                ui.label(egui::RichText::new("—").color(DIM).small());
                            }
                        }
                    });
                }
            });
        });

    // Package + click handling.
    let Some(record) = footprint else {
        ui.label(
            egui::RichText::new("(no package drawing available for this footprint — use the Peripherals tab)")
                .weak(),
        );
        apply(design, picked, edit);
        return;
    };

    if let Some((pp, pr)) = picked.clone() {
        ui.label(
            egui::RichText::new(format!("Placing {pp}.{pr} — click a blue pin (or ✕/another role to cancel)"))
                .color(CAND_FILL),
        );
    }

    let mut paints: HashMap<Pin, PinPaint> = HashMap::new();
    for (pid, (p, r)) in &taken {
        paints.insert(
            as_pin(*pid),
            PinPaint {
                fill: LOCKED_FILL,
                border: (Some(*pid) == picked_pin).then(|| Stroke::new(2.0, PICK_BORDER)),
                sublabel: Some(r.clone()),
                tooltip: Some(format!("{p}.{r} (click to pick up)")),
                interactive: true,
            },
        );
    }
    for pid in &candidates {
        // A self-locked pin is already painted green; leave it.
        if taken.contains_key(pid) {
            continue;
        }
        paints.insert(
            as_pin(*pid),
            PinPaint {
                fill: CAND_FILL,
                border: None,
                sublabel: Some("•".into()),
                tooltip: picked.as_ref().map(|(p, r)| format!("place {p}.{r} here")),
                interactive: true,
            },
        );
    }

    if edit.is_none() {
        match crate::package_view::show_record(ui, record, &paints, &[]) {
            Some(Action::Click(clicked)) => {
                let pid = PinId { port: clicked.port, num: clicked.num };
                edit = Some(match (picked.clone(), taken.get(&pid)) {
                    // A candidate for the picked role → place it there.
                    (Some((p, r)), _) if candidates.contains(&pid) => Edit::Place(p, r, pid),
                    // A locked pin → pick up its role (to move/inspect).
                    (_, Some((p, r))) => Edit::Pick(p.clone(), r.clone()),
                    // Anything else with a role picked → cancel.
                    (Some(_), _) => Edit::Unpick,
                    _ => return,
                });
            }
            Some(Action::ClickEmpty) => edit = Some(Edit::Unpick),
            None => {}
        }
    } else {
        // A role-list edit already fired this frame; still draw the package.
        let _ = crate::package_view::show_record(ui, record, &paints, &[]);
    }

    apply(design, picked, edit);
}

fn apply(design: &mut H523Design, picked: &mut Option<(String, String)>, edit: Option<Edit>) {
    match edit {
        Some(Edit::Pick(p, r)) => *picked = Some((p, r)),
        Some(Edit::Unpick) => *picked = None,
        Some(Edit::Place(p, r, pin)) => {
            design.lock(&p, &r, pin);
            *picked = None;
        }
        Some(Edit::Unlock(p, r)) => {
            design.unlock(&p, &r);
            if picked.as_ref() == Some(&(p, r)) {
                *picked = None;
            }
        }
        None => {}
    }
}
