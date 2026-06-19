//! Generic, descriptor-driven "Peripherals" planner for the non-G474 families
//! (H523 / C5A3) — the family-agnostic equivalent of the G474 requirements
//! panel. Declare peripheral *uses* ("USART2 for TX+RX", "TIM1 for CH1/CH1N/
//! BKIN"), place each role on a pin, and get live completeness/conflict
//! validation — all from the descriptor, with no per-family enums. Models the
//! C531 converter view's card + problems-panel idiom on the generic
//! `mcu_pinout` engine; the raw Pin/AF table remains the "browse every pin"
//! escape hatch.

use eframe::egui;
use std::collections::BTreeSet;

use crate::h523_design::{H523Design, H523Problem};
use crate::mcu_pinout::{af_rows, peripherals_with_pins, pins_for, PinId, SignalId};
use crate::mcu_raw::RawMcuData;

const GREEN: egui::Color32 = egui::Color32::from_rgb(100, 200, 120);
const YELLOW: egui::Color32 = egui::Color32::from_rgb(210, 180, 80);
const RED: egui::Color32 = egui::Color32::from_rgb(220, 100, 100);

/// One deferred mutation (egui renders the immutable design, then we apply).
enum Action {
    Add(&'static str),
    Remove(usize),
    ToggleRole { use_idx: usize, role: String, on: bool },
    Lock { peripheral: String, role: String, pin: PinId },
    Unlock { peripheral: String, role: String },
    SetNote { use_idx: usize, note: String },
}

/// Distinct roles a peripheral instance exposes on this chip (sorted).
fn available_roles(raw: &'static RawMcuData, peripheral: &str) -> Vec<&'static str> {
    let mut v: Vec<&'static str> = af_rows(raw)
        .filter(|r| r.signal.peripheral == peripheral)
        .map(|r| r.signal.role)
        .collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// The `&'static` instance name from the descriptor (so it can key `SignalId`).
fn static_name(raw: &'static RawMcuData, peripheral: &str) -> &'static str {
    raw.peripherals
        .iter()
        .find(|p| p.name == peripheral)
        .map(|p| p.name)
        .unwrap_or("")
}

/// Default roles when a peripheral is first declared: the base signals of its
/// logical kind (SERIAL->TX,RX; SPI->SCK,MOSI,MISO; …), else none (the user
/// toggles roles on the card).
fn default_roles(peripheral: &str) -> Vec<&'static str> {
    match crate::select::kind_of(peripheral) {
        Some(kind) => crate::select::required_signals(kind, &[]),
        None => Vec::new(),
    }
}

pub fn show(ui: &mut egui::Ui, raw: &'static RawMcuData, design: &mut H523Design) {
    let problems = design.validate(raw);
    let mut action: Option<Action> = None;

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{} — peripherals", raw.name)).strong());
        if !design.uses.is_empty() {
            if problems.is_empty() {
                ui.colored_label(GREEN, "✓ complete");
            } else {
                ui.colored_label(YELLOW, format!("⚠ {} to resolve", problems.len()));
            }
        }
        let declared: BTreeSet<&str> = design.uses.iter().map(|u| u.peripheral.as_str()).collect();
        ui.menu_button("+ Add peripheral", |ui| {
            egui::ScrollArea::vertical().max_height(380.0).show(ui, |ui| {
                for p in peripherals_with_pins(raw) {
                    if !declared.contains(p) && ui.button(p).clicked() {
                        action = Some(Action::Add(p));
                        ui.close();
                    }
                }
            });
        });
    });
    ui.label(
        egui::RichText::new("Declare what you're using; place each role on a pin. Validation is live.")
            .weak()
            .small(),
    );
    ui.separator();

    if design.uses.is_empty() {
        ui.label("No peripherals declared yet — use “+ Add peripheral”.");
    } else {
        egui::ScrollArea::vertical().show(ui, |ui| {
            for (i, u) in design.uses.iter().enumerate() {
                let peri = static_name(raw, &u.peripheral);
                egui::Frame::group(ui.style()).show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(&u.peripheral).strong());
                        ui.separator();
                        ui.label("note:");
                        let mut note = u.note.clone();
                        if ui
                            .add(
                                egui::TextEdit::singleline(&mut note)
                                    .desired_width(150.0)
                                    .hint_text("e.g. buck leg A"),
                            )
                            .changed()
                        {
                            action = Some(Action::SetNote { use_idx: i, note });
                        }
                        if ui.button("✕").on_hover_text("remove use + its locks").clicked() {
                            action = Some(Action::Remove(i));
                        }
                    });
                    egui::Grid::new(("roles", i)).num_columns(2).spacing([14.0, 4.0]).show(ui, |ui| {
                        for role in available_roles(raw, &u.peripheral) {
                            let declared_role = u.roles.iter().any(|r| r == role);
                            let mut on = declared_role;
                            if ui.checkbox(&mut on, role).changed() {
                                action = Some(Action::ToggleRole {
                                    use_idx: i,
                                    role: role.to_string(),
                                    on,
                                });
                            }
                            if declared_role {
                                let cur = design.locked_pin(&u.peripheral, role);
                                let txt = cur.map(|p| p.name()).unwrap_or_else(|| "— pick pin".into());
                                let color = if cur.is_some() { GREEN } else { YELLOW };
                                egui::ComboBox::from_id_salt(("pin", i, role))
                                    .selected_text(egui::RichText::new(txt).color(color))
                                    .show_ui(ui, |ui| {
                                        if ui.selectable_label(cur.is_none(), "— (none)").clicked()
                                            && cur.is_some()
                                        {
                                            action = Some(Action::Unlock {
                                                peripheral: u.peripheral.clone(),
                                                role: role.to_string(),
                                            });
                                        }
                                        for pin in pins_for(raw, SignalId { peripheral: peri, role }) {
                                            let other = design
                                                .occupant_of(pin)
                                                .filter(|o| *o != (u.peripheral.as_str(), role));
                                            let label = match other {
                                                Some((per, ro)) => {
                                                    format!("{} · used by {per} {ro}", pin.name())
                                                }
                                                None => pin.name(),
                                            };
                                            if ui
                                                .add_enabled(
                                                    other.is_none(),
                                                    egui::Button::selectable(cur == Some(pin), label),
                                                )
                                                .clicked()
                                            {
                                                action = Some(Action::Lock {
                                                    peripheral: u.peripheral.clone(),
                                                    role: role.to_string(),
                                                    pin,
                                                });
                                            }
                                        }
                                    });
                            } else {
                                ui.label("");
                            }
                            ui.end_row();
                        }
                    });
                });
            }
        });
    }

    if !problems.is_empty() {
        ui.separator();
        ui.label(egui::RichText::new("To resolve:").strong());
        for p in &problems {
            match p {
                H523Problem::Unplaced { peripheral, role } => {
                    ui.colored_label(YELLOW, format!("• {peripheral} {role} — not placed on a pin"));
                }
                H523Problem::Unreachable { peripheral, role } => {
                    ui.colored_label(RED, format!("• {peripheral} {role} — no pin on this package"));
                }
            }
        }
    }

    if let Some(a) = action {
        match a {
            Action::Add(p) => {
                let roles = default_roles(p);
                design.add_use(p, &roles);
            }
            Action::Remove(i) => design.remove_use(i),
            Action::ToggleRole { use_idx, role, on } => design.set_role(use_idx, &role, on),
            Action::Lock { peripheral, role, pin } => design.lock(&peripheral, &role, pin),
            Action::Unlock { peripheral, role } => design.unlock(&peripheral, &role),
            Action::SetNote { use_idx, note } => {
                if let Some(u) = design.uses.get_mut(use_idx) {
                    u.note = note;
                }
            }
        }
    }
}
