//! HRTIM tab: one card per sub-timer (TimA..F) + Master. Each card shows
//! which requirement currently owns the sub-timer (if any) and offers
//! quick role actions: "Add PCM phase here", "Clear", and — when a PCM
//! phase already targets this timer — DEM / threshold toggles routed
//! back to `set_spec`.
//!
//! This sits *alongside* the power fabric view, not in place of it.
//! Fabric still shows the DAC→COMP→EEV→HRTIM→FLT graph; this view lets
//! you think per-sub-timer rather than per-requirement.

use eframe::egui;

use crate::g474::HrtimId;
use crate::requirements::{Assignment, Design, RequirementSpec, ThresholdSource};

pub enum HrtimAction {
    SetSpec(usize, RequirementSpec),
    Remove(usize),
    AddPhaseOn(HrtimId),
}

const SUB_TIMERS: &[HrtimId] = &[
    HrtimId::TimA, HrtimId::TimB, HrtimId::TimC,
    HrtimId::TimD, HrtimId::TimE, HrtimId::TimF,
];

pub fn show(ui: &mut egui::Ui, design: &Design) -> Option<HrtimAction> {
    let mut action: Option<HrtimAction> = None;

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("HRTIM sub-timers").strong());
        ui.label(egui::RichText::new(
            "— per-sub-timer view of phase/fault ownership. Power fabric tab has the full graph.",
        ).weak());
    });
    ui.separator();

    egui::ScrollArea::vertical().show(ui, |ui| {
        // Master card on its own row — it doesn't carry phases, just
        // triggers ADC sequencers via MCR1..4 / MPER events.
        ui.allocate_ui(egui::vec2(680.0, 0.0), |ui| {
            ui.group(|ui| {
                render_master_card(ui, design);
            });
        });
        ui.add_space(4.0);

        // Sub-timer cards wrap 3-across at typical window widths.
        ui.horizontal_wrapped(|ui| {
            for &t in SUB_TIMERS {
                ui.allocate_ui(egui::vec2(220.0, 0.0), |ui| {
                    ui.group(|ui| {
                        if let Some(a) = render_sub_card(ui, design, t) {
                            action = Some(a);
                        }
                    });
                });
            }
        });
    });

    action
}

fn render_master_card(ui: &mut egui::Ui, design: &Design) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("Master").strong());
    });
    let triggers: Vec<String> = design
        .assignments
        .iter()
        .flatten()
        .filter_map(|a| match a {
            Assignment::AdcSequencer {
                trigger: crate::requirements::TriggerSource::Event(ev),
                adc,
                kind,
                ..
            } if is_master_event(*ev) => Some(format!("ADC{} {:?} ← {:?}", adc.number(), kind, ev)),
            _ => None,
        })
        .collect();
    if triggers.is_empty() {
        ui.label(egui::RichText::new("(no master-triggered sequencers)").weak());
    } else {
        for t in triggers {
            ui.label(format!("  • {}", t));
        }
    }
}

fn is_master_event(ev: crate::g474::CrossbarSource) -> bool {
    use crate::g474::CrossbarSource::*;
    matches!(ev, Mcr1 | Mcr2 | Mcr3 | Mcr4 | Mper)
}

fn render_sub_card(ui: &mut egui::Ui, design: &Design, timer: HrtimId) -> Option<HrtimAction> {
    let mut action: Option<HrtimAction> = None;

    // Find the requirement (if any) whose assignment currently uses this timer.
    let owner: Option<(usize, &Assignment)> = design
        .assignments
        .iter()
        .enumerate()
        .filter_map(|(i, a)| a.as_ref().map(|a| (i, a)))
        .find(|(_, a)| match a {
            Assignment::PcmPhase { timer: t, .. }
            | Assignment::PcmPhaseExternal { timer: t, .. } => *t == timer,
            _ => false,
        });

    // Header: timer name + pinned/auto hint.
    let pinned = design.requirements.iter().enumerate().find_map(|(i, s)| match s {
        RequirementSpec::PcmPhase { preferred_timer: Some(t), .. } if *t == timer => Some(i),
        _ => None,
    });
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{:?}", timer)).strong());
        if pinned.is_some() {
            ui.label(egui::RichText::new("(pinned)").small().color(egui::Color32::from_rgb(160, 200, 120)));
        }
    });

    match owner {
        Some((idx, Assignment::PcmPhase { alloc, dem, zcd_eev, .. })) => {
            let ext = if zcd_eev.is_some() { " +extZCD" } else { "" };
            let demtag = if *dem { " +DEM" } else { "" };
            ui.label(format!("{:?} → {:?} → {:?}{}{}", alloc.dac, alloc.comp, alloc.eev, demtag, ext));
            render_phase_role_toggles(ui, design, idx, &mut action);
        }
        Some((idx, Assignment::PcmPhaseExternal { dac, peak_eev, dem, zcd_eev, .. })) => {
            let src = match dac { Some(d) => format!("{:?}", d), None => "fixed-ref".to_string() };
            let demtag = if *dem { " +DEM" } else { "" };
            let ext = if zcd_eev.is_some() { " +extZCD" } else { "" };
            ui.label(format!("[ext] {} → peak {:?}{}{}", src, peak_eev, demtag, ext));
            render_phase_role_toggles(ui, design, idx, &mut action);
        }
        _ => {
            ui.label(egui::RichText::new("(free)").weak());
            if ui.small_button("Add PCM phase here").clicked() {
                action = Some(HrtimAction::AddPhaseOn(timer));
            }
        }
    }

    action
}

fn render_phase_role_toggles(
    ui: &mut egui::Ui,
    design: &Design,
    req_idx: usize,
    action: &mut Option<HrtimAction>,
) {
    let spec = design.requirements[req_idx];
    let RequirementSpec::PcmPhase { dem, threshold, preferred_timer } = spec else {
        return;
    };
    let mut new_dem = dem;
    let mut new_threshold = threshold;
    ui.horizontal(|ui| {
        ui.checkbox(&mut new_dem, "DEM");
    });
    egui::ComboBox::from_id_salt(("hrtim-thr", req_idx))
        .width(180.0)
        .selected_text(threshold_label(threshold))
        .show_ui(ui, |ui| {
            for opt in [
                ThresholdSource::Internal,
                ThresholdSource::InternalExtZcd,
                ThresholdSource::External,
            ] {
                if ui.selectable_label(threshold == opt, threshold_label(opt)).clicked() {
                    new_threshold = opt;
                }
            }
        });
    if (new_dem, new_threshold) != (dem, threshold) {
        *action = Some(HrtimAction::SetSpec(
            req_idx,
            RequirementSpec::PcmPhase {
                dem: new_dem, threshold: new_threshold, preferred_timer,
            },
        ));
    }
    ui.horizontal(|ui| {
        if ui.small_button("Unpin").clicked() {
            *action = Some(HrtimAction::SetSpec(
                req_idx,
                RequirementSpec::PcmPhase {
                    dem, threshold, preferred_timer: None,
                },
            ));
        }
        if ui.small_button("Remove").clicked() {
            *action = Some(HrtimAction::Remove(req_idx));
        }
    });
}

fn threshold_label(t: ThresholdSource) -> &'static str {
    match t {
        ThresholdSource::Internal => "internal COMP",
        ThresholdSource::InternalExtZcd => "int peak + ext ZCD",
        ThresholdSource::External => "external COMP",
    }
}
