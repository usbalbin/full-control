//! Timers tab: card grid for TIM1/8/20 (advanced-control) and TIM2-5 /
//! TIM15-17 (general-purpose). Each card offers CHx / CHxN / BKIN(2) /
//! ETR checkboxes, a mode selector (PWM / Encoder / Input-capture), and
//! COMP→TIM internal-routing pickers for capture and BKIN (the triac
//! zero-cross topology uses capture←COMP). Pin pickers below the card
//! for every signal the chosen config actually claims.
//!
//! TIM→ADC and TIM→DAC routing is deliberately out of scope per user.

use eframe::egui;

use crate::g474::{CompId, TimCh, TimId};
use crate::pinout::{self, ChipVariant, Pin, Signal};
use crate::requirements::{Assignment, Design, RequirementSpec, TimMode};

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
    let RequirementSpec::UseTim {
        instance, channels_mask, complementary, bkin, etr,
        mode, capture_comp, bkin_comp,
    } = design.requirements[req_idx]
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
                            mode, capture_comp, bkin_comp,
                        },
                    ));
                }
            }
        });

    // Mode selector: PWM / Encoder / InputCapture.
    let mut new_mode = mode;
    egui::ComboBox::from_id_salt(("tim-mode", req_idx))
        .width(120.0)
        .selected_text(match mode {
            TimMode::Pwm => "PWM / compare",
            TimMode::Encoder => "Encoder (A/B)",
            TimMode::InputCapture => "Input-capture",
        })
        .show_ui(ui, |ui| {
            for (m, label) in [
                (TimMode::Pwm, "PWM / compare"),
                (TimMode::Encoder, "Encoder (A/B)"),
                (TimMode::InputCapture, "Input-capture"),
            ] {
                if ui.selectable_label(mode == m, label).clicked() {
                    new_mode = m;
                }
            }
        });

    // Channel toggles only meaningful in PWM mode — Encoder forces CH1+CH2,
    // InputCapture forces CH1.
    let mut mask = channels_mask;
    if matches!(mode, TimMode::Pwm) {
        ui.horizontal(|ui| {
            for i in 0..4u8 {
                let mut v = mask & (1 << i) != 0;
                if ui.checkbox(&mut v, format!("CH{}", i + 1)).clicked() {
                    if v { mask |= 1 << i; } else { mask &= !(1 << i); }
                }
            }
        });
    }
    let mut c = complementary;
    let mut b = bkin;
    let mut e = etr;
    if matches!(mode, TimMode::Pwm) {
        ui.horizontal(|ui| {
            ui.checkbox(&mut c, "Complementary (CHxN)");
        });
    }
    ui.horizontal(|ui| {
        ui.checkbox(&mut b, "BKIN");
        ui.checkbox(&mut e, "ETR");
    });

    // COMP→TIM internal routing: capture input (for triac zero-cross
    // and line-sync) and BKIN (for overcurrent brake). No pin is
    // claimed here — the link is internal to the MCU.
    let mut new_cap = capture_comp;
    let mut new_bkin_comp = bkin_comp;
    ui.horizontal(|ui| {
        ui.label("capture ←");
        egui::ComboBox::from_id_salt(("tim-cap", req_idx))
            .width(90.0)
            .selected_text(match capture_comp {
                Some(c) => format!("{:?}", c),
                None => "(none)".to_string(),
            })
            .show_ui(ui, |ui| {
                if ui.selectable_label(capture_comp.is_none(), "(none)").clicked() {
                    new_cap = None;
                }
                for cc in [CompId::Comp1, CompId::Comp2, CompId::Comp3, CompId::Comp4,
                           CompId::Comp5, CompId::Comp6, CompId::Comp7] {
                    if ui.selectable_label(capture_comp == Some(cc), format!("{:?}", cc)).clicked() {
                        new_cap = Some(cc);
                    }
                }
            });
    });
    ui.horizontal(|ui| {
        ui.label("BKIN ←");
        egui::ComboBox::from_id_salt(("tim-bkincomp", req_idx))
            .width(90.0)
            .selected_text(match bkin_comp {
                Some(c) => format!("{:?}", c),
                None => "(none)".to_string(),
            })
            .show_ui(ui, |ui| {
                if ui.selectable_label(bkin_comp.is_none(), "(none)").clicked() {
                    new_bkin_comp = None;
                }
                for cc in [CompId::Comp1, CompId::Comp2, CompId::Comp3, CompId::Comp4,
                           CompId::Comp5, CompId::Comp6, CompId::Comp7] {
                    if ui.selectable_label(bkin_comp == Some(cc), format!("{:?}", cc)).clicked() {
                        new_bkin_comp = Some(cc);
                    }
                }
            });
    });

    if (mask, c, b, e, new_mode, new_cap, new_bkin_comp)
        != (channels_mask, complementary, bkin, etr, mode, capture_comp, bkin_comp)
    {
        action = Some(TimersAction::SetSpec(
            req_idx,
            RequirementSpec::UseTim {
                instance, channels_mask: mask, complementary: c, bkin: b, etr: e,
                mode: new_mode, capture_comp: new_cap, bkin_comp: new_bkin_comp,
            },
        ));
    }

    // Pin pickers for active signals on this timer.
    let Some(Assignment::Tim {
        instance: ai, channels_mask: am, complementary: ac, bkin: ab, etr: ae, mode: amode, ..
    }) = design.assignments.get(req_idx).and_then(|a| a.as_ref())
    else {
        ui.label(egui::RichText::new("(not assigned)").weak());
        return action;
    };
    // Effective channel pins depend on mode — Encoder pins CH1+CH2, InputCapture CH1 only.
    let (effective_mask, effective_comp) = match amode {
        TimMode::Encoder      => (0b0011u8, false),
        TimMode::InputCapture => (0b0001u8, false),
        TimMode::Pwm          => (*am, *ac),
    };
    let mut signals: Vec<Signal> = Vec::new();
    let chs = [TimCh::Ch1, TimCh::Ch2, TimCh::Ch3, TimCh::Ch4];
    for (i, ch) in chs.iter().enumerate() {
        if effective_mask & (1 << i as u8) != 0 {
            signals.push(Signal::TimCh(*ai, *ch));
            if effective_comp { signals.push(Signal::TimChN(*ai, *ch)); }
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
