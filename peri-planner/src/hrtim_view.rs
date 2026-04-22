//! HRTIM tab: one card per sub-timer (TimA..F) + Master. Each card shows
//! which requirement currently owns the sub-timer and offers role-specific
//! toggles plus quick Add-role actions for free sub-timers. Sits alongside
//! the power fabric view — fabric still renders the full DAC→COMP→EEV
//! graph; this view is organized per-sub-timer so you can reason about
//! "what is TimE doing?" instead of hunting through a requirement list.

use eframe::egui;

use crate::g474::HrtimId;
use crate::requirements::{
    Assignment, Design, HrtimResolved, HrtimRole, OutputMode, RequirementSpec,
};

pub enum HrtimAction {
    SetSpec(usize, RequirementSpec),
    Remove(usize),
    /// Shortcut used by the free-card "Add PCM phase here" button — adds
    /// a `UseHrtimSub` with `PcmInternal { ext_zcd: false, dem: false }`.
    AddPhaseOn(HrtimId),
    /// Generic "add this role on this sub-timer" used by the preset menu.
    AddRoleOn(HrtimId, HrtimRole),
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
            "— per-sub-timer view of role ownership. Power fabric tab has the full graph.",
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
                ui.allocate_ui(egui::vec2(240.0, 0.0), |ui| {
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

    // Find the requirement (if any) whose assignment currently uses this
    // sub-timer. Only HrtimSub claims a sub-timer in the new model.
    let owner: Option<(usize, &Assignment)> = design
        .assignments
        .iter()
        .enumerate()
        .filter_map(|(i, a)| a.as_ref().map(|a| (i, a)))
        .find(|(_, a)| matches!(a, Assignment::HrtimSub { sub_timer: t, .. } if *t == timer));

    // Pinned? (matches the pinned_sub_timer on a UseHrtimSub spec)
    let pinned = design.requirements.iter().enumerate().find_map(|(i, s)| match s {
        RequirementSpec::UseHrtimSub { pinned_sub_timer: Some(t), .. } if *t == timer => Some(i),
        _ => None,
    });
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{:?}", timer)).strong());
        if pinned.is_some() {
            ui.label(
                egui::RichText::new("(pinned)")
                    .small()
                    .color(egui::Color32::from_rgb(160, 200, 120)),
            );
        }
    });

    match owner {
        Some((idx, Assignment::HrtimSub { resolved, outputs, fault, .. })) => {
            render_resolved_summary(ui, resolved, *outputs, *fault);
            render_role_toggles(ui, design, idx, &mut action);
        }
        _ => {
            ui.label(egui::RichText::new("(free)").weak());
            // Add menu with all role presets.
            ui.menu_button("Add role \u{25BE}", |ui| {
                let presets = [
                    ("PCM phase",
                     HrtimRole::PcmInternal { ext_zcd: false, dem: false }),
                    ("PCM + DEM",
                     HrtimRole::PcmInternal { ext_zcd: false, dem: true }),
                    ("PCM + DEM (ext ZCD)",
                     HrtimRole::PcmInternal { ext_zcd: true, dem: true }),
                    ("PCM ext COMP",
                     HrtimRole::PcmExternal { with_slow_dac: true, ext_zcd: false, dem: false }),
                    ("Voltage-mode PWM",
                     HrtimRole::VoltageModePwm),
                    ("Phase-shift FB",
                     HrtimRole::PhaseShift { peer_sub_timer: None }),
                    ("Generic", HrtimRole::External),
                ];
                for (label, role) in presets {
                    if ui.button(label).clicked() {
                        action = Some(HrtimAction::AddRoleOn(timer, role));
                        ui.close();
                    }
                }
            });
            if ui.small_button("Quick: add PCM phase").clicked() {
                action = Some(HrtimAction::AddPhaseOn(timer));
            }
        }
    }

    action
}

fn render_resolved_summary(
    ui: &mut egui::Ui,
    resolved: &HrtimResolved,
    outputs: OutputMode,
    fault: Option<crate::g474::HrtimFltId>,
) {
    let out_tag = match outputs {
        OutputMode::Ch1Only => "CH1",
        OutputMode::Ch1AndCh2 => "CH1+CH2",
    };
    match resolved {
        HrtimResolved::PcmInternal { dac, comp, eev, zcd_eev, dem } => {
            let demtag = if *dem { " +DEM" } else { "" };
            let ext = match zcd_eev {
                Some(z) => format!(" +extZCD={:?}", z),
                None => String::new(),
            };
            ui.label(format!("PCM {:?}→{:?}→{:?}{}{}", dac, comp, eev, demtag, ext));
        }
        HrtimResolved::PcmExternal { dac, peak_eev, zcd_eev, dem } => {
            let src = match dac {
                Some(d) => format!("{:?}", d),
                None => "fixed-ref".to_string(),
            };
            let demtag = if *dem { " +DEM" } else { "" };
            let ext = match zcd_eev {
                Some(z) => format!(" ZCD={:?}", z),
                None => String::new(),
            };
            ui.label(format!("[ext] {} → peak {:?}{}{}", src, peak_eev, demtag, ext));
        }
        HrtimResolved::VoltageModePwm => {
            ui.label("voltage-mode PWM");
        }
        HrtimResolved::PhaseShift { peer, phase_shift_q15 } => {
            ui.label(format!("phase-shift peer={:?} φ=Q15({})", peer, phase_shift_q15));
        }
        HrtimResolved::External => {
            ui.label("generic (user-managed)");
        }
    }
    ui.label(
        egui::RichText::new(match fault {
            Some(f) => format!("outputs={}  fault={:?}", out_tag, f),
            None => format!("outputs={}", out_tag),
        })
        .small()
        .color(egui::Color32::from_gray(170)),
    );
}

fn render_role_toggles(
    ui: &mut egui::Ui,
    design: &Design,
    req_idx: usize,
    action: &mut Option<HrtimAction>,
) {
    let spec = design.requirements[req_idx];
    let RequirementSpec::UseHrtimSub { pinned_sub_timer, role, outputs, fault } = spec else {
        return;
    };

    // Role-specific toggles inline (DEM / ext ZCD only meaningful for PCM).
    let mut new_role = role;
    match role {
        HrtimRole::PcmInternal { ext_zcd, dem } => {
            let mut d = dem;
            let mut z = ext_zcd;
            ui.horizontal(|ui| {
                ui.checkbox(&mut d, "DEM");
                ui.checkbox(&mut z, "ext ZCD");
            });
            if (d, z) != (dem, ext_zcd) {
                new_role = HrtimRole::PcmInternal { ext_zcd: z, dem: d };
            }
        }
        HrtimRole::PcmExternal { with_slow_dac, ext_zcd, dem } => {
            let mut d = dem;
            let mut z = ext_zcd;
            let mut s = with_slow_dac;
            ui.horizontal(|ui| {
                ui.checkbox(&mut d, "DEM");
                ui.checkbox(&mut z, "ext ZCD");
            });
            ui.horizontal(|ui| {
                ui.checkbox(&mut s, "slow DAC threshold");
            });
            if (d, z, s) != (dem, ext_zcd, with_slow_dac) {
                new_role = HrtimRole::PcmExternal { with_slow_dac: s, ext_zcd: z, dem: d };
            }
        }
        _ => {}
    }

    // Outputs toggle (disabled when role forces both, e.g. DEM / phase-shift).
    let dem_forces_both = matches!(
        role,
        HrtimRole::PcmInternal { dem: true, .. } | HrtimRole::PcmExternal { dem: true, .. }
    );
    let phase_shift_forces_both = matches!(role, HrtimRole::PhaseShift { .. });
    let forces_both = dem_forces_both || phase_shift_forces_both;
    let mut new_outputs = outputs;
    ui.horizontal(|ui| {
        let resp_ch1 = ui.add_enabled(
            !forces_both,
            egui::RadioButton::new(outputs == OutputMode::Ch1Only, "CH1 only"),
        );
        if resp_ch1.clicked() {
            new_outputs = OutputMode::Ch1Only;
        }
        if ui.radio(outputs == OutputMode::Ch1AndCh2, "CH1+CH2").clicked() {
            new_outputs = OutputMode::Ch1AndCh2;
        }
    });

    if (new_role, new_outputs) != (role, outputs) {
        *action = Some(HrtimAction::SetSpec(
            req_idx,
            RequirementSpec::UseHrtimSub {
                pinned_sub_timer, role: new_role, outputs: new_outputs, fault,
            },
        ));
    }

    ui.horizontal(|ui| {
        if ui.small_button("Unpin").clicked() {
            *action = Some(HrtimAction::SetSpec(
                req_idx,
                RequirementSpec::UseHrtimSub {
                    pinned_sub_timer: None, role, outputs, fault,
                },
            ));
        }
        if ui.small_button("Remove").clicked() {
            *action = Some(HrtimAction::Remove(req_idx));
        }
    });
}
