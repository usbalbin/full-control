//! STM32C531 "Converter" tab: edit a list of timer-PWM converter legs.
//!
//! This is the C531 analogue of the G474 power-fabric/timers tabs, but built on
//! the parallel `C531Design` model rather than the HRTIM-shaped `Design`. Each
//! leg is one advanced-control timer (TIM1/TIM8) driving a PWM/half-bridge stage
//! with optional hardware over-current trip (COMP -> timer break) and ADC sense.
//!
//! Every cross-peripheral dropdown is driven by the chip *fabric*
//! (`ChipFabric::comps_for_tim_break` / `dac_threshold_sources_for_comp`): only
//! comparators that can physically reach the chosen timer's break input appear,
//! and only DAC channels that can set that comparator's threshold. The validator
//! (`C531Design::validate`) is the backstop — the problems panel surfaces any
//! combination the UI can't prevent (e.g. a leftover comp after a timer swap)
//! plus cross-leg resource conflicts.

use eframe::egui;

use crate::c531_design::{
    comp_from_num, comp_num, dac_from_inst_ch, BreakInput, C531Design, ConverterLeg, Ocp, Problem,
};
use crate::g474::{AdcInstance, CompId, DacId, TimId};
use crate::mcu::Package;

pub enum ConverterAction {
    AddLeg,
    RemoveLeg(usize),
    /// Replace leg `usize` wholesale with an edited copy (mirrors the
    /// timers-tab "rebuild the spec and diff" pattern).
    SetLeg(usize, ConverterLeg),
}

/// Descriptor timer-`number` -> `TimId`. Only the ids the model understands.
fn tim_from_num(n: u8) -> Option<TimId> {
    TimId::ALL.iter().copied().find(|t| t.number() == n)
}

/// Descriptor ADC-`number` -> the `AdcInstance` enum the model stores.
fn adc_from_num(n: u8) -> Option<AdcInstance> {
    AdcInstance::ALL.iter().copied().find(|a| a.number() == n)
}

pub fn show(ui: &mut egui::Ui, design: &C531Design, package: Package) -> Option<ConverterAction> {
    let mut action: Option<ConverterAction> = None;
    let descriptor = package.descriptor();

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{} — timer-PWM converter", descriptor.name)).strong());
        ui.label(
            egui::RichText::new("— advanced-timer legs with COMP→break hardware OCP").weak(),
        );
    });
    ui.separator();

    // Problems panel first, so the user sees realizability at a glance.
    let problems = design.validate(package);
    if problems.is_empty() {
        ui.colored_label(
            egui::Color32::from_rgb(100, 200, 120),
            if design.legs.is_empty() {
                "No legs yet — add one below."
            } else {
                "OK — every leg routes on this chip and no resources collide."
            },
        );
    } else {
        ui.colored_label(
            egui::Color32::from_rgb(220, 140, 80),
            format!("{} problem(s):", problems.len()),
        );
        for p in &problems {
            ui.label(format!("  • {}", describe_problem(p)));
        }
    }
    ui.separator();

    if ui.button("+ Add leg").clicked() {
        action = Some(ConverterAction::AddLeg);
    }

    // Advanced-control timers actually present on this chip — the only timers
    // that carry complementary outputs, dead-time and break inputs.
    let advanced_tims: Vec<TimId> = descriptor
        .timers
        .iter()
        .filter(|t| t.kind == crate::mcu::TimerKind::Advanced)
        .filter_map(|t| tim_from_num(t.number))
        .collect();
    // ADC instances present, as typed ids, for the sense picker.
    let adcs: Vec<AdcInstance> = descriptor.adcs.iter().filter_map(|a| adc_from_num(a.number)).collect();

    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.horizontal_wrapped(|ui| {
            for (i, leg) in design.legs.iter().enumerate() {
                ui.allocate_ui(egui::vec2(320.0, 0.0), |ui| {
                    ui.group(|ui| {
                        if let Some(a) = render_card(ui, package, i, leg, &advanced_tims, &adcs) {
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
    package: Package,
    idx: usize,
    leg: &ConverterLeg,
    advanced_tims: &[TimId],
    adcs: &[AdcInstance],
) -> Option<ConverterAction> {
    let mut action: Option<ConverterAction> = None;
    let fabric = package.descriptor().fabric;
    // Working copy — widgets mutate this, and we diff against `leg` at the end.
    let mut new = leg.clone();

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("Leg {} — TIM{}", idx + 1, leg.tim.number())).strong());
        if ui.small_button("x").clicked() {
            action = Some(ConverterAction::RemoveLeg(idx));
        }
    });

    // Timer instance.
    egui::ComboBox::from_id_salt(("c531-tim", idx))
        .width(90.0)
        .selected_text(format!("TIM{}", new.tim.number()))
        .show_ui(ui, |ui| {
            for &t in advanced_tims {
                if ui.selectable_label(t == new.tim, format!("TIM{}", t.number())).clicked() {
                    new.tim = t;
                }
            }
        });

    // Channels + output flags.
    ui.horizontal(|ui| {
        for i in 0..4u8 {
            let mut v = new.channels_mask & (1 << i) != 0;
            if ui.checkbox(&mut v, format!("CH{}", i + 1)).clicked() {
                if v { new.channels_mask |= 1 << i; } else { new.channels_mask &= !(1 << i); }
            }
        }
    });
    ui.checkbox(&mut new.complementary, "Complementary (CHxN)");
    ui.checkbox(&mut new.dead_time, "Dead-time");
    ui.checkbox(&mut new.bkin, "External BKIN pin");

    // Hardware over-current trip: COMP -> timer break, optional DAC threshold.
    let mut ocp_on = new.ocp.is_some();
    ui.separator();
    ui.checkbox(&mut ocp_on, "Hardware OCP (COMP → break)");
    if ocp_on {
        // Comparators that can reach *either* break input on this timer.
        let reachable_comps = comps_reaching_break(fabric, new.tim);
        // Initialise a freshly-enabled OCP from the first routable comparator,
        // so the default is valid on this chip rather than an arbitrary COMP1.
        let mut ocp = new.ocp.clone().unwrap_or_else(|| Ocp {
            comp: reachable_comps.first().copied().unwrap_or(CompId::Comp1),
            break_input: 1,
            threshold_dac: None,
        });

        ui.horizontal(|ui| {
            ui.label("COMP");
            egui::ComboBox::from_id_salt(("c531-comp", idx))
                .width(90.0)
                .selected_text(format!("{:?}", ocp.comp))
                .show_ui(ui, |ui| {
                    for &c in &reachable_comps {
                        if ui.selectable_label(c == ocp.comp, format!("{:?}", c)).clicked() {
                            ocp.comp = c;
                        }
                    }
                    if reachable_comps.is_empty() {
                        ui.label(egui::RichText::new("(no comparator reaches this timer)").weak());
                    }
                });

            // Break inputs reachable for the chosen comparator (1=BRK, 2=BRK2).
            let breaks = breaks_for_comp(fabric, new.tim, ocp.comp);
            ui.label("→");
            egui::ComboBox::from_id_salt(("c531-brk", idx))
                .width(70.0)
                .selected_text(break_label(ocp.break_input))
                .show_ui(ui, |ui| {
                    let offer: &[BreakInput] = if breaks.is_empty() { &[1, 2] } else { &breaks };
                    for &b in offer {
                        if ui.selectable_label(b == ocp.break_input, break_label(b)).clicked() {
                            ocp.break_input = b;
                        }
                    }
                });
        });

        // DAC threshold for the comparator's inverting input.
        let dac_sources: Vec<DacId> = fabric
            .map(|f| f.dac_threshold_sources_for_comp(comp_num(ocp.comp)))
            .unwrap_or_default()
            .into_iter()
            .filter_map(|(inst, ch)| dac_from_inst_ch(inst, ch))
            .collect();
        ui.horizontal(|ui| {
            ui.label("threshold");
            egui::ComboBox::from_id_salt(("c531-dac", idx))
                .width(110.0)
                .selected_text(match ocp.threshold_dac {
                    Some(d) => format!("{:?}", d),
                    None => "(none)".to_string(),
                })
                .show_ui(ui, |ui| {
                    if ui.selectable_label(ocp.threshold_dac.is_none(), "(none)").clicked() {
                        ocp.threshold_dac = None;
                    }
                    for &d in &dac_sources {
                        if ui.selectable_label(ocp.threshold_dac == Some(d), format!("{:?}", d)).clicked() {
                            ocp.threshold_dac = Some(d);
                        }
                    }
                    if dac_sources.is_empty() {
                        ui.label(egui::RichText::new("(no DAC threshold for this COMP)").weak());
                    }
                });
        });

        new.ocp = Some(ocp);
    } else {
        new.ocp = None;
    }

    // ADC sense.
    ui.separator();
    let mut sense_on = new.adc_sense.is_some();
    ui.checkbox(&mut sense_on, "ADC sense");
    if sense_on {
        let (mut adc, mut ch) = new.adc_sense.unwrap_or((
            adcs.first().copied().unwrap_or(AdcInstance::Adc1),
            1,
        ));
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt(("c531-adc", idx))
                .width(70.0)
                .selected_text(format!("ADC{}", adc.number()))
                .show_ui(ui, |ui| {
                    for &a in adcs {
                        if ui.selectable_label(a == adc, format!("ADC{}", a.number())).clicked() {
                            adc = a;
                        }
                    }
                });
            ui.label("IN");
            ui.add(egui::DragValue::new(&mut ch).range(0..=23));
        });
        new.adc_sense = Some((adc, ch));
    } else {
        new.adc_sense = None;
    }

    // Pin map for the leg's claimed GPIO signals (read-only for now — pin-lock
    // persistence is a later slice).
    ui.separator();
    ui.label(egui::RichText::new("Pins:").weak());
    for (sig, pins) in leg.pin_candidates(package) {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new(format!("  {:<10}", sig.name())).monospace());
            if pins.is_empty() {
                ui.colored_label(egui::Color32::from_rgb(220, 100, 100), "(no pin)");
            } else if pins.len() == 1 {
                ui.label(pins[0].name());
            } else {
                let names: Vec<String> = pins.iter().map(|p| p.name()).collect();
                ui.label(egui::RichText::new(names.join(", ")).small());
            }
        });
    }

    if new != *leg {
        action = Some(ConverterAction::SetLeg(idx, new));
    }
    action
}

/// Comparators that can drive *either* break input of `tim` on this chip.
fn comps_reaching_break(fabric: Option<&crate::mcu::ChipFabric>, tim: TimId) -> Vec<CompId> {
    let Some(fab) = fabric else { return Vec::new() };
    let mut nums: Vec<u8> = fab.comps_for_tim_break(tim.number(), 1);
    for n in fab.comps_for_tim_break(tim.number(), 2) {
        if !nums.contains(&n) {
            nums.push(n);
        }
    }
    nums.sort_unstable();
    nums.into_iter().filter_map(comp_from_num).collect()
}

/// Which break inputs `comp` can reach on `tim` (subset of {1, 2}).
fn breaks_for_comp(fabric: Option<&crate::mcu::ChipFabric>, tim: TimId, comp: CompId) -> Vec<BreakInput> {
    let Some(fab) = fabric else { return Vec::new() };
    [1u8, 2u8]
        .into_iter()
        .filter(|&b| fab.comps_for_tim_break(tim.number(), b).contains(&comp_num(comp)))
        .collect()
}

fn break_label(b: BreakInput) -> String {
    match b {
        1 => "BRK".to_string(),
        2 => "BRK2".to_string(),
        n => format!("BRK{}", n),
    }
}

fn describe_problem(p: &Problem) -> String {
    match p {
        Problem::OcpUnroutable { leg, comp, tim, break_input } => format!(
            "Leg {}: {:?} can't drive TIM{} {} on this chip",
            leg + 1, comp, tim.number(), break_label(*break_input),
        ),
        Problem::ThresholdUnroutable { leg, dac, comp } => format!(
            "Leg {}: {:?} can't set {:?}'s threshold",
            leg + 1, dac, comp,
        ),
        Problem::TimerConflict { legs, tim } => format!(
            "Legs {} and {} both claim TIM{}",
            legs.0 + 1, legs.1 + 1, tim.number(),
        ),
        Problem::CompConflict { legs, comp } => format!(
            "Legs {} and {} both route OCP through {:?}",
            legs.0 + 1, legs.1 + 1, comp,
        ),
        Problem::DacConflict { legs, dac } => format!(
            "Legs {} and {} both use {:?} as threshold",
            legs.0 + 1, legs.1 + 1, dac,
        ),
        Problem::AdcConflict { legs, adc, channel } => format!(
            "Legs {} and {} both sense ADC{} IN{}",
            legs.0 + 1, legs.1 + 1, adc.number(), channel,
        ),
    }
}
