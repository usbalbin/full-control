//! Comms tab: cards for SPI / I2C / USART / UART / LPUART / FDCAN / USB /
//! UCPD. Each card lists the signals this instance claims with a pin
//! dropdown per signal. Toggle checkboxes for optional signals (MISO, NSS,
//! SMBA, flow control, synchronous CK).

use eframe::egui;

use crate::g474::PeripheralKind;
use crate::pinout::{self, ChipVariant, Pin, Signal};
use crate::requirements::{Assignment, Design, RequirementSpec};

/// Editing actions returned from the view — the caller applies them
/// wrapped in `mutate()` so undo/redo records them.
pub enum CommsAction {
    SetSpec(usize, RequirementSpec),
    Remove(usize),
    SetPin(Signal, Pin),
    ClearPin(Signal),
}

pub fn show(ui: &mut egui::Ui, design: &Design, variant: ChipVariant) -> Option<CommsAction> {
    let mut action: Option<CommsAction> = None;

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("Communication peripherals").strong());
        ui.label(egui::RichText::new("— add via the top palette; configure signals here").weak());
    });
    ui.separator();

    // Group the active requirements by peripheral kind so cards stack
    // tidily by family.
    let mut groups: Vec<(PeripheralKind, Vec<usize>)> = PeripheralKind::COMMS
        .iter()
        .map(|k| (*k, Vec::new()))
        .collect();
    for (i, spec) in design.requirements.iter().enumerate() {
        if let Some(kind) = comms_kind_of(spec) {
            if let Some((_, list)) = groups.iter_mut().find(|(k, _)| *k == kind) {
                list.push(i);
            }
        }
    }

    egui::ScrollArea::vertical().show(ui, |ui| {
        for (kind, indices) in &groups {
            if indices.is_empty() { continue; }
            ui.collapsing(kind.label(), |ui| {
                ui.horizontal_wrapped(|ui| {
                    for &i in indices {
                        let card_w = 260.0;
                        ui.allocate_ui(egui::vec2(card_w, 0.0), |ui| {
                            ui.group(|ui| {
                                if let Some(a) = render_card(ui, design, variant, i) {
                                    action = Some(a);
                                }
                            });
                        });
                    }
                });
            });
        }
    });

    action
}

fn comms_kind_of(spec: &RequirementSpec) -> Option<PeripheralKind> {
    match spec {
        RequirementSpec::UseSpi { .. }    => Some(PeripheralKind::Spi),
        RequirementSpec::UseI2c { .. }    => Some(PeripheralKind::I2c),
        RequirementSpec::UseUsart { .. }  => Some(PeripheralKind::Usart),
        RequirementSpec::UseUart { .. }   => Some(PeripheralKind::Uart),
        RequirementSpec::UseLpuart { .. } => Some(PeripheralKind::Lpuart),
        RequirementSpec::UseCan { .. }    => Some(PeripheralKind::Can),
        RequirementSpec::UseUsb           => Some(PeripheralKind::Usb),
        RequirementSpec::UseUcpd { .. }   => Some(PeripheralKind::Ucpd),
        _ => None,
    }
}

fn render_card(
    ui: &mut egui::Ui,
    design: &Design,
    variant: ChipVariant,
    req_idx: usize,
) -> Option<CommsAction> {
    let mut action: Option<CommsAction> = None;
    let spec = design.requirements[req_idx];
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(spec.name()).strong());
        if ui.small_button("x").clicked() {
            action = Some(CommsAction::Remove(req_idx));
        }
    });

    // Spec-level toggles that change which signals are claimed.
    let mut new_spec: Option<RequirementSpec> = None;
    match spec {
        RequirementSpec::UseSpi { instance, needs_miso, needs_nss } => {
            let mut m = needs_miso;
            let mut n = needs_nss;
            ui.checkbox(&mut m, "MISO");
            ui.checkbox(&mut n, "NSS");
            if (m, n) != (needs_miso, needs_nss) {
                new_spec = Some(RequirementSpec::UseSpi { instance, needs_miso: m, needs_nss: n });
            }
        }
        RequirementSpec::UseI2c { instance, needs_smba } => {
            let mut s = needs_smba;
            ui.checkbox(&mut s, "SMBA");
            if s != needs_smba {
                new_spec = Some(RequirementSpec::UseI2c { instance, needs_smba: s });
            }
        }
        RequirementSpec::UseUsart { instance, flow_control, synchronous } => {
            let mut f = flow_control;
            let mut s = synchronous;
            ui.checkbox(&mut f, "CTS/RTS");
            ui.checkbox(&mut s, "CK (sync)");
            if (f, s) != (flow_control, synchronous) {
                new_spec = Some(RequirementSpec::UseUsart {
                    instance, flow_control: f, synchronous: s,
                });
            }
        }
        RequirementSpec::UseUart { instance, flow_control } => {
            let mut f = flow_control;
            ui.checkbox(&mut f, "CTS/RTS");
            if f != flow_control {
                new_spec = Some(RequirementSpec::UseUart { instance, flow_control: f });
            }
        }
        RequirementSpec::UseLpuart { instance, flow_control } => {
            let mut f = flow_control;
            ui.checkbox(&mut f, "CTS/RTS");
            if f != flow_control {
                new_spec = Some(RequirementSpec::UseLpuart { instance, flow_control: f });
            }
        }
        _ => {}
    }
    if let Some(s) = new_spec {
        action = Some(CommsAction::SetSpec(req_idx, s));
    }

    // Per-signal pin-pickers for the signals this assignment actually claims.
    let signals: Vec<Signal> = signals_of(design.assignments.get(req_idx).and_then(|a| a.as_ref()));
    if signals.is_empty() {
        ui.label(egui::RichText::new("(not assigned)").weak());
        return action;
    }
    for sig in signals {
        if let Some(a) = render_pin_row(ui, design, variant, sig) {
            action = Some(a);
        }
    }
    action
}

fn signals_of(asn: Option<&Assignment>) -> Vec<Signal> {
    let Some(a) = asn else { return Vec::new(); };
    match a {
        Assignment::Spi { instance, needs_miso, needs_nss } => {
            let mut s = vec![Signal::SpiMosi(*instance), Signal::SpiSck(*instance)];
            if *needs_miso { s.push(Signal::SpiMiso(*instance)); }
            if *needs_nss  { s.push(Signal::SpiNss(*instance)); }
            s
        }
        Assignment::I2c { instance, needs_smba } => {
            let mut s = vec![Signal::I2cSda(*instance), Signal::I2cScl(*instance)];
            if *needs_smba { s.push(Signal::I2cSmba(*instance)); }
            s
        }
        Assignment::Usart { instance, flow_control, synchronous } => {
            let mut s = vec![Signal::UsartTx(*instance), Signal::UsartRx(*instance)];
            if *flow_control {
                s.push(Signal::UsartCts(*instance));
                s.push(Signal::UsartRts(*instance));
            }
            if *synchronous { s.push(Signal::UsartCk(*instance)); }
            s
        }
        Assignment::Uart { instance, flow_control } => {
            let mut s = vec![Signal::UartTx(*instance), Signal::UartRx(*instance)];
            if *flow_control {
                s.push(Signal::UartCts(*instance));
                s.push(Signal::UartRts(*instance));
            }
            s
        }
        Assignment::Lpuart { instance, flow_control } => {
            let mut s = vec![Signal::LpuartTx(*instance), Signal::LpuartRx(*instance)];
            if *flow_control {
                s.push(Signal::LpuartCts(*instance));
                s.push(Signal::LpuartRts(*instance));
            }
            s
        }
        Assignment::Can { instance } => {
            vec![Signal::CanTx(*instance), Signal::CanRx(*instance)]
        }
        Assignment::Usb => vec![Signal::UsbDp, Signal::UsbDm],
        Assignment::Ucpd { instance } => {
            vec![Signal::UcpdCc1(*instance), Signal::UcpdCc2(*instance)]
        }
        _ => Vec::new(),
    }
}

fn render_pin_row(
    ui: &mut egui::Ui,
    design: &Design,
    variant: ChipVariant,
    signal: Signal,
) -> Option<CommsAction> {
    let mut action: Option<CommsAction> = None;
    let all_cands = pinout::pins_for(signal, variant);
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{:<14}", signal.name())).monospace());
        if all_cands.is_empty() {
            ui.colored_label(egui::Color32::from_rgb(220, 100, 100), "(no pin)");
            return;
        }
        if all_cands.len() == 1 {
            ui.label(all_cands[0].name());
            return;
        }
        let cands = design.pin_candidates(signal, variant);
        let current = design.pinned(signal);
        let label = current.map(|p| p.name()).unwrap_or_else(|| format!("{} opts", cands.len()));
        egui::ComboBox::from_id_salt(("comms-pin", signal))
            .width(110.0)
            .selected_text(label)
            .show_ui(ui, |ui| {
                if ui.selectable_label(current.is_none(), "(any)").clicked() {
                    action = Some(CommsAction::ClearPin(signal));
                }
                for p in &all_cands {
                    let taken_by_other = !cands.contains(p) && Some(*p) != current;
                    let label = if taken_by_other { format!("{} (taken)", p.name()) } else { p.name() };
                    if ui
                        .add_enabled(
                            !taken_by_other,
                            egui::Button::selectable(Some(*p) == current, label),
                        )
                        .clicked()
                    {
                        action = Some(CommsAction::SetPin(signal, *p));
                    }
                }
            });
    });
    action
}
