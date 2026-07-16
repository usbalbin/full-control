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
const YELLOW: Color32 = Color32::from_rgb(210, 180, 80);
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

/// Reachable pins for `(peripheral, role)` that are free — or already locked to
/// this same role. The pin-map's placement candidates (conflict-aware via the
/// generic engine + the design's current locks).
pub(crate) fn candidate_pins(
    raw: &'static RawMcuData,
    design: &H523Design,
    peripheral: &str,
    role: &str,
) -> Vec<PinId> {
    let Some(sig) = static_sig(raw, peripheral, role) else {
        return Vec::new();
    };
    pins_for(raw, sig)
        .into_iter()
        .filter(|pin| match design.occupant_of(*pin) {
            None => true,
            Some((p, r)) => p == peripheral && r == role,
        })
        .collect()
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
    let problems = design.validate(raw);
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{} — pin map", raw.name)).strong());
        ui.label(
            egui::RichText::new("click a role, then a highlighted pin to place it")
                .weak()
                .small(),
        );
        if !design.declared_signals().is_empty() {
            if problems.is_empty() {
                ui.colored_label(GREEN, "✓ all placed");
            } else {
                let detail = problems
                    .iter()
                    .map(|p| match p {
                        crate::h523_design::H523Problem::Unplaced { peripheral, role } => {
                            format!("{peripheral}.{role}: unplaced")
                        }
                        crate::h523_design::H523Problem::Unreachable { peripheral, role } => {
                            format!("{peripheral}.{role}: no pin on this chip")
                        }
                    })
                    .collect::<Vec<_>>()
                    .join("\n");
                ui.colored_label(YELLOW, format!("⚠ {} to resolve", problems.len()))
                    .on_hover_text(detail);
            }
        }
        if let Some(b) = crate::board::boards_for(raw.name).first() {
            ui.separator();
            ui.label(
                egui::RichText::new(format!("{} headers overlaid", b.name))
                    .color(YELLOW)
                    .small(),
            )
            .on_hover_text("Arduino pins are labelled; board-reserved pins (LED, button, VCP, SWD…) are outlined — hover any pin for its board function.");
        }
    });
    ui.separator();

    // Owned snapshots so the design is free to mutate after the click.
    let roles: Vec<(String, String)> = design
        .declared_signals()
        .into_iter()
        .map(|(p, r)| (p.to_string(), r.to_string()))
        .collect();
    // A pick that isn't a declared role on THIS chip (e.g. left over from another
    // chip before the pick was cleared) must never place — drop it.
    if picked.as_ref().is_some_and(|pk| !roles.contains(pk)) {
        *picked = None;
    }
    let taken: HashMap<PinId, (String, String)> = design
        .taken_pins()
        .into_iter()
        .map(|(pin, (p, r))| (pin, (p.to_string(), r.to_string())))
        .collect();

    // Candidate pins for the picked role: reachable AND free (or already its own).
    let candidates: Vec<PinId> = picked
        .as_ref()
        .map(|(p, r)| candidate_pins(raw, design, p, r))
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

    // Nucleo header overlay: label each board pin with its Arduino name and flag
    // the board-reserved ones (informational — see package_view).
    crate::package_view::overlay_board_headers(&mut paints, raw.name);

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn candidate_pins_exclude_pins_taken_by_other_roles() {
        let raw = crate::mcu::Package::H523R.raw();
        let mut d = H523Design::new();
        let cands0 = candidate_pins(raw, &d, "SPI1", "MOSI");
        assert!(!cands0.is_empty(), "SPI1.MOSI has candidate pins on H523");
        let taken = cands0[0];

        // Taken by a DIFFERENT role -> no longer a candidate.
        d.lock("USART1", "TX", taken);
        assert!(!candidate_pins(raw, &d, "SPI1", "MOSI").contains(&taken));

        // Locked to MOSI itself -> stays a candidate (its own placement).
        d.unlock("USART1", "TX");
        d.lock("SPI1", "MOSI", taken);
        assert!(candidate_pins(raw, &d, "SPI1", "MOSI").contains(&taken));
    }

    #[test]
    fn candidate_pins_empty_for_unknown_role() {
        let raw = crate::mcu::Package::H523R.raw();
        assert!(candidate_pins(raw, &H523Design::new(), "NOPE", "XYZ").is_empty());
    }
}
