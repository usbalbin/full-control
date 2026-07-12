//! Peripheral-inventory view, descriptor-driven. Reads `McuDescriptor`
//! and renders a grouped overview — works for any MCU whose descriptor
//! is populated. Used today as the H523 landing view; later slices can
//! promote it to a top-level tab on G474 too.

use eframe::egui;

use crate::mcu::{McuDescriptor, TimerKind};

pub fn show(ui: &mut egui::Ui, mcu: &McuDescriptor) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.heading(format!("{} — peripheral inventory", mcu.name));
        ui.separator();

        ui.columns(2, |cols| {
            render_analog(&mut cols[0], mcu);
            render_timers(&mut cols[0], mcu);
            render_comms(&mut cols[1], mcu);
            render_fabric(&mut cols[1], mcu);
        });
    });
}

fn render_analog(ui: &mut egui::Ui, mcu: &McuDescriptor) {
    ui.group(|ui| {
        ui.label(egui::RichText::new("Analog").strong());
        if mcu.adcs.is_empty() {
            ui.label("ADC: (none)");
        } else {
            let names: Vec<String> = mcu.adcs.iter().map(|a| format!("ADC{}", a.number)).collect();
            ui.label(format!("ADC: {}", names.join(", ")));
            // Fast-channel rule is uniform across instances on supported MCUs.
            if let Some(first) = mcu.adcs.first()
                && !first.fast_channels.is_empty() {
                    let chs: Vec<String> = first.fast_channels.iter().map(u8::to_string).collect();
                    ui.label(egui::RichText::new(format!("  fast channels: IN{}", chs.join(", IN"))).weak());
                }
        }
        if mcu.dacs.is_empty() {
            ui.label("DAC: (none)");
        } else {
            let names: Vec<String> = mcu.dacs.iter().map(|d| {
                let suffix = if d.fast { " (fast S&H)" } else { "" };
                format!("DAC{} ×{}{}", d.number, d.channels, suffix)
            }).collect();
            ui.label(format!("DAC: {}", names.join(", ")));
        }
        let comp_label = if mcu.comps.is_empty() {
            "COMP: (none)".to_string()
        } else {
            let names: Vec<String> = mcu.comps.iter().map(|c| format!("COMP{}", c.number)).collect();
            format!("COMP: {}", names.join(", "))
        };
        ui.label(comp_label);
        let op_label = if mcu.opamps.is_empty() {
            "OPAMP: (none)".to_string()
        } else {
            let names: Vec<String> = mcu.opamps.iter().map(|o| format!("OPAMP{}", o.number)).collect();
            format!("OPAMP: {}", names.join(", "))
        };
        ui.label(op_label);
    });
}

fn render_timers(ui: &mut egui::Ui, mcu: &McuDescriptor) {
    ui.group(|ui| {
        ui.label(egui::RichText::new("Timers").strong());
        for &kind in &[TimerKind::Advanced, TimerKind::General32, TimerKind::General16,
                       TimerKind::Basic, TimerKind::LowPower] {
            let group: Vec<&_> = mcu.timers.iter().filter(|t| t.kind == kind).collect();
            if group.is_empty() { continue; }
            let label = match kind {
                TimerKind::Advanced  => "Advanced",
                TimerKind::General32 => "GP 32-bit",
                TimerKind::General16 => "GP 16-bit",
                TimerKind::Basic     => "Basic",
                TimerKind::LowPower  => "Low-power",
            };
            let names: Vec<String> = group.iter().map(|t| match t.kind {
                TimerKind::LowPower => format!("LPTIM{}", t.number),
                _ => format!("TIM{}", t.number),
            }).collect();
            ui.label(format!("  {}: {}", label, names.join(", ")));
        }
    });
}

fn render_comms(ui: &mut egui::Ui, mcu: &McuDescriptor) {
    ui.group(|ui| {
        ui.label(egui::RichText::new("Communications").strong());
        let c = &mcu.comms;
        list(ui, "SPI",    &c.spi,    "SPI");
        list(ui, "I2C",    &c.i2c,    "I2C");
        list(ui, "I3C",    &c.i3c,    "I3C");
        list(ui, "USART",  &c.usart,  "USART");
        list(ui, "UART",   &c.uart,   "UART");
        list(ui, "LPUART", &c.lpuart, "LPUART");
        list(ui, "FDCAN",  &c.fdcan,  "FDCAN");
        list(ui, "UCPD",   &c.ucpd,   "UCPD");
        if c.has_usb { ui.label("  USB"); }
        match c.octospi {
            0 => {}
            1 => { ui.label("  OCTOSPI1"); }
            n => { ui.label(format!("  OCTOSPI ×{}", n)); }
        }
        if c.has_sdmmc { ui.label("  SDMMC1"); }
        if c.has_fmc { ui.label("  FMC"); }
        if c.has_hdmi_cec { ui.label("  HDMI-CEC"); }
    });
}

fn list(ui: &mut egui::Ui, label: &str, instances: &[u8], prefix: &str) {
    if instances.is_empty() { return; }
    let names: Vec<String> = instances.iter().map(|n| format!("{}{}", prefix, n)).collect();
    ui.label(format!("  {}: {}", label, names.join(", ")));
}

fn render_fabric(ui: &mut egui::Ui, mcu: &McuDescriptor) {
    ui.group(|ui| {
        ui.label(egui::RichText::new("Power fabric").strong());
        match mcu.hrtim {
            None => {
                ui.label("  HRTIM: (not present on this MCU)");
            }
            Some(h) => {
                ui.label(format!(
                    "  HRTIM: {} sub-timers, {} EEV, {} FLT, {} ADC triggers",
                    h.sub_timer_count, h.eev_count, h.flt_count, h.adc_trigger_count,
                ));
            }
        }
        if mcu.edges.is_empty() {
            ui.label("  Inter-peripheral routing: (none)");
        } else {
            let dac_to_comp = mcu.edges.iter()
                .filter(|e| matches!(e.kind, crate::mcu::EdgeKind::DacToComp))
                .count();
            ui.label(format!("  DAC→COMP analog routes: {}", dac_to_comp));
        }
    });
}
