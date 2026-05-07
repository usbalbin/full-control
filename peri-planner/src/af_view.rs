//! Pin/AF browser, descriptor-driven. Shows every (pin, peripheral, signal,
//! AF) tuple from the selected MCU's metapac-extracted data with two
//! orthogonal filters: by peripheral and by pin port. Works on any MCU.

use eframe::egui;

use crate::mcu_pinout::{af_rows, peripherals_with_pins, AfRow};
use crate::mcu_raw::RawMcuData;

#[derive(Default)]
pub struct AfFilter {
    pub peripheral: Option<&'static str>,
    pub port: Option<char>,
    pub query: String,
}

pub fn show(ui: &mut egui::Ui, raw: &'static RawMcuData, filter: &mut AfFilter) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{} — pin / AF table", raw.name)).strong());
        ui.label(egui::RichText::new("(from stm32-metapac)").weak());
    });
    ui.separator();

    let peripherals = peripherals_with_pins(raw);
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
        ui.add(egui::TextEdit::singleline(&mut filter.query).desired_width(120.0));
        if !filter.query.is_empty() && ui.small_button("clear").clicked() {
            filter.query.clear();
        }
    });
    ui.separator();

    let q = filter.query.to_ascii_lowercase();
    let rows: Vec<AfRow> = af_rows(raw)
        .filter(|r| filter.peripheral.is_none_or(|p| r.signal.peripheral == p))
        .filter(|r| filter.port.is_none_or(|c| r.pin.port == c))
        .filter(|r| {
            if q.is_empty() { return true; }
            r.signal.peripheral.to_ascii_lowercase().contains(&q)
                || r.signal.role.to_ascii_lowercase().contains(&q)
                || r.pin.name().to_ascii_lowercase().contains(&q)
        })
        .collect();

    ui.label(egui::RichText::new(format!("{} entries", rows.len())).weak());

    egui::ScrollArea::vertical().show(ui, |ui| {
        egui::Grid::new("af_table").striped(true).show(ui, |ui| {
            ui.label(egui::RichText::new("Pin").strong());
            ui.label(egui::RichText::new("Peripheral").strong());
            ui.label(egui::RichText::new("Signal").strong());
            ui.label(egui::RichText::new("AF").strong());
            ui.end_row();

            for r in &rows {
                ui.label(r.pin.name());
                ui.label(r.signal.peripheral);
                ui.label(r.signal.role);
                ui.label(match r.af {
                    Some(n) => format!("AF{}", n),
                    None => "—".to_string(),
                });
                ui.end_row();
            }
        });
    });
}
