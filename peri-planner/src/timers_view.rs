//! Timers tab: card grid for TIM1/8/20 (advanced-control) and TIM2-5 /
//! TIM15-17 (general-purpose). Each card offers CHx / CHxN / BKIN(2) /
//! ETR checkboxes and a pin picker per active signal.
//!
//! Inter-peripheral wiring (COMP→TIM capture for triac zero-cross,
//! TIM→ADC trigger, TIM→DAC, encoder mode, slave-mode trigger) belongs
//! to a later pass — see the power fabric view for the equivalent on
//! HRTIM. Hooks for those go into TODO(wiring) markers.

use eframe::egui;

use crate::g474::{TimCh, TimId};
use crate::pinout::{self, ChipVariant, Pin, Signal};
use crate::requirements::{Assignment, Design, RequirementSpec};

pub enum TimersAction {
    SetSpec(usize, RequirementSpec),
    Remove(usize),
    SetPin(Signal, Pin),
    ClearPin(Signal),
}

pub fn show(ui: &mut egui::Ui, design: &Design, variant: ChipVariant) -> Option<TimersAction> {
    let mut action: Option<TimersAction> = None;

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("General-purpose & advanced-control timers").strong());
        ui.label(egui::RichText::new("— HRTIM lives in the Power fabric tab").weak());
    });
    ui.separator();

    let active: Vec<usize> = design
        .requirements
        .iter()
        .enumerate()
        .filter_map(|(i, s)| matches!(s, RequirementSpec::UseTim { .. }).then_some(i))
        .collect();

    if active.is_empty() {
        ui.label(egui::RichText::new(
            "No timers added. Add a TIM from the top palette (\"TIM2\" for general-purpose or \"TIM1 [±,BKIN]\" for complementary PWM with break input)."
        ).weak());
        return None;
    }

    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.horizontal_wrapped(|ui| {
            for i in active {
                ui.allocate_ui(egui::vec2(300.0, 0.0), |ui| {
                    ui.group(|ui| {
                        if let Some(a) = render_card(ui, design, variant, i) {
                            action = Some(a);
                        }
                    });
                });
            }
        });
    });

    action
}

fn render_card(
    ui: &mut egui::Ui,
    design: &Design,
    variant: ChipVariant,
    req_idx: usize,
) -> Option<TimersAction> {
    let mut action: Option<TimersAction> = None;
    let RequirementSpec::UseTim { instance, channels_mask, complementary, bkin, etr } =
        design.requirements[req_idx]
    else {
        return None;
    };

    let kind = if instance.is_advanced() { "advanced" } else { "gp" };
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new(format!("TIM{} ({})", instance.number(), kind))
                .strong(),
        );
        if ui.small_button("x").clicked() {
            action = Some(TimersAction::Remove(req_idx));
        }
    });

    // Timer-instance swap dropdown.
    egui::ComboBox::from_id_salt(("tim-inst", req_idx))
        .width(80.0)
        .selected_text(format!("TIM{}", instance.number()))
        .show_ui(ui, |ui| {
            for &t in TimId::ALL {
                if ui.selectable_label(t == instance, format!("TIM{}", t.number())).clicked() {
                    action = Some(TimersAction::SetSpec(
                        req_idx,
                        RequirementSpec::UseTim {
                            instance: t, channels_mask, complementary, bkin, etr,
                        },
                    ));
                }
            }
        });

    // Channel toggles (CH1-CH4).
    let mut mask = channels_mask;
    ui.horizontal(|ui| {
        for i in 0..4u8 {
            let mut v = mask & (1 << i) != 0;
            if ui.checkbox(&mut v, format!("CH{}", i + 1)).clicked() {
                if v { mask |= 1 << i; } else { mask &= !(1 << i); }
            }
        }
    });
    let mut c = complementary;
    let mut b = bkin;
    let mut e = etr;
    ui.horizontal(|ui| {
        ui.checkbox(&mut c, "Complementary (CHxN)");
    });
    ui.horizontal(|ui| {
        ui.checkbox(&mut b, "BKIN");
        ui.checkbox(&mut e, "ETR");
    });
    if (mask, c, b, e) != (channels_mask, complementary, bkin, etr) {
        action = Some(TimersAction::SetSpec(
            req_idx,
            RequirementSpec::UseTim {
                instance, channels_mask: mask, complementary: c, bkin: b, etr: e,
            },
        ));
    }

    // Pin pickers for active signals on this timer.
    let Some(Assignment::Tim {
        instance: ai, channels_mask: am, complementary: ac, bkin: ab, etr: ae,
    }) = design.assignments.get(req_idx).and_then(|a| a.as_ref())
    else {
        ui.label(egui::RichText::new("(not assigned)").weak());
        return action;
    };
    let mut signals: Vec<Signal> = Vec::new();
    let chs = [TimCh::Ch1, TimCh::Ch2, TimCh::Ch3, TimCh::Ch4];
    for (i, ch) in chs.iter().enumerate() {
        if am & (1 << i as u8) != 0 {
            signals.push(Signal::TimCh(*ai, *ch));
            if *ac { signals.push(Signal::TimChN(*ai, *ch)); }
        }
    }
    if *ab { signals.push(Signal::TimBkin(*ai)); }
    if *ae { signals.push(Signal::TimEtr(*ai)); }

    for sig in signals {
        if let Some(a) = render_pin_row(ui, design, variant, sig) {
            action = Some(a);
        }
    }

    // TODO(wiring): encoder-mode, COMP→TIM capture (triac zero-cross),
    // TIM→ADC trigger, slave-mode trigger. Hooks go here when we do the
    // inter-peripheral wiring pass.

    action
}

fn render_pin_row(
    ui: &mut egui::Ui,
    design: &Design,
    variant: ChipVariant,
    signal: Signal,
) -> Option<TimersAction> {
    let mut action: Option<TimersAction> = None;
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
        let cands = pinout::pin_candidates_respecting_locks(
            signal, variant, &design.pin_assignments,
        );
        let current = design.pin_assignments.get(&signal).copied();
        let label = current.map(|p| p.name()).unwrap_or_else(|| format!("{} opts", cands.len()));
        egui::ComboBox::from_id_salt(("tim-pin", signal))
            .width(110.0)
            .selected_text(label)
            .show_ui(ui, |ui| {
                if ui.selectable_label(current.is_none(), "(any)").clicked() {
                    action = Some(TimersAction::ClearPin(signal));
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
                        action = Some(TimersAction::SetPin(signal, *p));
                    }
                }
            });
    });
    action
}
