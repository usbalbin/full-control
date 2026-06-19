//! Pin/AF browser, descriptor-driven. Shows every (pin, peripheral, signal,
//! AF) tuple from the selected MCU's metapac-extracted data with two
//! orthogonal filters: by peripheral and by pin port. Works on any MCU.
//! When invoked with an `H523Design` it lets the user lock pins per signal.

use eframe::egui;

use crate::h523_design::H523Design;
use crate::mcu_pinout::{af_rows, peripherals_with_pins, AfRow, PinId};
use crate::mcu_raw::RawMcuData;

#[derive(Default)]
pub struct AfFilter {
    pub peripheral: Option<&'static str>,
    pub port: Option<char>,
    pub query: String,
    pub locked_only: bool,
}

pub fn show(
    ui: &mut egui::Ui,
    raw: &'static RawMcuData,
    filter: &mut AfFilter,
    design: Option<&mut H523Design>,
) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{} — pin / AF table", raw.name)).strong());
        ui.label(egui::RichText::new("(from stm32-metapac)").weak());
    });
    ui.separator();

    let peripherals = peripherals_with_pins(raw);
    let can_lock = design.is_some();
    let mut enter = false;
    let mut search_resp: Option<egui::Response> = None;
    ui.horizontal(|ui| {
        ui.label("Peripheral:");
        egui::ComboBox::from_id_salt("af_peripheral")
            .width(140.0)
            .selected_text(filter.peripheral.unwrap_or("(any)"))
            .show_ui(ui, |ui| {
                if ui.selectable_label(filter.peripheral.is_none(), "(any)").clicked() {
                    filter.peripheral = None;
                }
                for &name in &peripherals {
                    if ui.selectable_label(filter.peripheral == Some(name), name).clicked() {
                        filter.peripheral = Some(name);
                    }
                }
            });

        ui.label("Port:");
        let port_label = filter.port.map(|c| format!("P{}", c)).unwrap_or_else(|| "(any)".into());
        egui::ComboBox::from_id_salt("af_port")
            .width(70.0)
            .selected_text(port_label)
            .show_ui(ui, |ui| {
                if ui.selectable_label(filter.port.is_none(), "(any)").clicked() {
                    filter.port = None;
                }
                for c in 'A'..='I' {
                    let label = format!("P{}", c);
                    if ui.selectable_label(filter.port == Some(c), label).clicked() {
                        filter.port = Some(c);
                    }
                }
            });

        ui.label("Search:");
        let resp = ui.add(
            egui::TextEdit::singleline(&mut filter.query)
                .desired_width(170.0)
                .hint_text(if can_lock { "type, ↵ locks top" } else { "type to filter" }),
        );
        enter = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
        search_resp = Some(resp);
        if !filter.query.is_empty() && ui.small_button("clear").clicked() {
            filter.query.clear();
        }
        if can_lock {
            ui.checkbox(&mut filter.locked_only, "Locked only");
        }
    });
    ui.separator();

    let q = filter.query.to_ascii_lowercase();
    let mut rows: Vec<AfRow> = af_rows(raw)
        .filter(|r| filter.peripheral.is_none_or(|p| r.signal.peripheral == p))
        .filter(|r| filter.port.is_none_or(|c| r.pin.port == c))
        .filter(|r| {
            if q.is_empty() { return true; }
            r.signal.peripheral.to_ascii_lowercase().contains(&q)
                || r.signal.role.to_ascii_lowercase().contains(&q)
                || r.pin.name().to_ascii_lowercase().contains(&q)
        })
        .collect();

    if let Some(d) = design.as_deref() {
        if filter.locked_only {
            rows.retain(|r| {
                d.locked_pin(r.signal.peripheral, r.signal.role) == Some(r.pin)
            });
        }
        let lock_count = d.pin_locks.len();
        let entries_label = if lock_count == 0 {
            format!("{} entries", rows.len())
        } else {
            format!("{} entries · {} locked", rows.len(), lock_count)
        };
        ui.label(egui::RichText::new(entries_label).weak());
    } else {
        ui.label(egui::RichText::new(format!("{} entries", rows.len())).weak());
    }

    // Enter-to-lock: place the wired signal on the first free matching pin among
    // the currently-filtered rows. So "usart2 tx ↵" locks USART2.TX on its first
    // free pin; add a pin to the query ("usart2 tx pa2") to target a specific one.
    let enter_target: Option<(String, String, PinId)> = if enter {
        design.as_deref().and_then(|d| {
            rows.iter()
                .find(|r| {
                    d.locked_pin(r.signal.peripheral, r.signal.role) != Some(r.pin)
                        && d.occupant_of(r.pin)
                            .is_none_or(|occ| occ == (r.signal.peripheral, r.signal.role))
                })
                .map(|r| (r.signal.peripheral.to_string(), r.signal.role.to_string(), r.pin))
        })
    } else {
        None
    };

    let mut pending: Option<LockAction> = None;
    egui::ScrollArea::vertical().show(ui, |ui| {
        egui::Grid::new("af_table").striped(true).show(ui, |ui| {
            ui.label(egui::RichText::new("Pin").strong());
            ui.label(egui::RichText::new("Peripheral").strong());
            ui.label(egui::RichText::new("Signal").strong());
            ui.label(egui::RichText::new("AF").strong());
            if design.is_some() {
                ui.label(egui::RichText::new("Lock").strong());
            }
            ui.end_row();

            for r in &rows {
                let row_state = design.as_deref().map(|d| {
                    let signal_locked_to = d.locked_pin(r.signal.peripheral, r.signal.role);
                    let pin_taken_by = d.occupant_of(r.pin);
                    (signal_locked_to, pin_taken_by)
                });

                let pin_text = match row_state {
                    Some((Some(p), _)) if p == r.pin => {
                        egui::RichText::new(r.pin.name())
                            .color(egui::Color32::from_rgb(140, 220, 140)).strong()
                    }
                    _ => egui::RichText::new(r.pin.name()),
                };
                ui.label(pin_text);
                ui.label(r.signal.peripheral);
                ui.label(r.signal.role);
                ui.label(match r.af {
                    Some(n) => format!("AF{}", n), None => "—".to_string(),
                });

                if let Some((sig_locked, pin_taken_by)) = row_state {
                    let is_this_lock = sig_locked == Some(r.pin);
                    if is_this_lock {
                        if ui.small_button("✓ unlock").clicked() {
                            pending = Some(LockAction::Unlock {
                                peripheral: r.signal.peripheral.to_string(),
                                role: r.signal.role.to_string(),
                            });
                        }
                    } else {
                        let other_signal_pin = sig_locked;
                        let label = match (other_signal_pin, pin_taken_by) {
                            (Some(p), _) if p != r.pin => format!("(at {})", p.name()),
                            (_, Some((per, role))) =>
                                format!("(used by {} {})", per, role),
                            _ => "lock".to_string(),
                        };
                        let enabled = pin_taken_by.is_none() || pin_taken_by ==
                            Some((r.signal.peripheral, r.signal.role));
                        if ui.add_enabled(enabled, egui::Button::new(label).small()).clicked() {
                            pending = Some(LockAction::Lock {
                                peripheral: r.signal.peripheral.to_string(),
                                role: r.signal.role.to_string(),
                                pin: r.pin,
                            });
                        }
                    }
                }
                ui.end_row();
            }
        });
    });

    // A click takes priority; otherwise an Enter in the search box places the
    // top match.
    let mut locked_via_enter = false;
    if pending.is_none()
        && let Some((peripheral, role, pin)) = enter_target
    {
        pending = Some(LockAction::Lock { peripheral, role, pin });
        locked_via_enter = true;
    }
    if let (Some(d), Some(action)) = (design, pending) {
        match action {
            LockAction::Lock { peripheral, role, pin } => d.lock(&peripheral, &role, pin),
            LockAction::Unlock { peripheral, role } => d.unlock(&peripheral, &role),
        }
    }
    // Clear the query after a placement, and keep focus in the box on any Enter so
    // the user can chain placements without reaching for the mouse.
    if locked_via_enter {
        filter.query.clear();
    }
    if enter && can_lock && let Some(r) = &search_resp {
        r.request_focus();
    } else if can_lock
        && ui.memory(|m| m.focused().is_none())
        && let Some(r) = &search_resp
    {
        // Land-and-type: on this allocation view, keep the search box focused by
        // default (so switching here lets you type immediately, and placements
        // chain) — unless the user has focused another widget (combo, button).
        r.request_focus();
    }
}

enum LockAction {
    Lock { peripheral: String, role: String, pin: crate::mcu_pinout::PinId },
    Unlock { peripheral: String, role: String },
}
