//! **Multi-channel front-end planner** view — the interactive front for
//! [`crate::frontend_plan`]. Answers "pick N single-ended input pins that keep
//! the most options open" for a data-logger / slow-scope front-end, and shows the
//! *hardware ceiling* so it's clear why not every channel can be a trigger + PGA.
//!
//! Editable planning: reserve pins the rest of the system needs (click a pin to
//! exclude it) and re-solve; change the channel count; and re-weight the
//! objective (favour triggers vs. programmable-gain pairs). All package-aware —
//! only pins bonded on the active footprint are eligible, minus reservations.

use std::collections::BTreeSet;

use eframe::egui::{self, Color32, RichText};

use crate::frontend_plan::{
    bonded_pins, capabilities, plan, AdcRead, ChannelPlan, FrontEndPlan, PlanConfig,
};
use crate::mcu_pinout::PinId;
use crate::mcu_raw::RawMcuData;
use crate::phys_pinout::PinoutRecord;

const PIN_COL: Color32 = Color32::from_rgb(220, 220, 230);
const ADC_COL: Color32 = Color32::from_rgb(90, 180, 220);
const OPAMP_COL: Color32 = Color32::from_rgb(120, 200, 140);
const COMP_COL: Color32 = Color32::from_rgb(200, 140, 205);
const FLAG_COL: Color32 = Color32::from_rgb(210, 180, 80);
const DIM: Color32 = Color32::from_rgb(150, 150, 160);

/// The (memoized) balanced/triggers/pga weight presets, so the active one can be
/// highlighted and applied without storing separate weight state.
const BALANCED: (i32, i32, i32, i32) = (100, 25, 30, 8);
const FAVOR_TRIGGERS: (i32, i32, i32, i32) = (70, 15, 60, 8);
const FAVOR_PGA: (i32, i32, i32, i32) = (150, 45, 15, 8);

fn weights(cfg: &PlanConfig) -> (i32, i32, i32, i32) {
    (cfg.w_pga_pair, cfg.w_pga_single, cfg.w_trigger, cfg.w_cross_adc)
}
fn set_weights(cfg: &mut PlanConfig, w: (i32, i32, i32, i32)) {
    (cfg.w_pga_pair, cfg.w_pga_single, cfg.w_trigger, cfg.w_cross_adc) = w;
}

type MemoKey = (usize, usize, PlanConfig, BTreeSet<PinId>);

/// Ephemeral view state: the solver config, the user's pin reservations, and a
/// per-input memo of the solved plan (re-solves only when an input changes).
#[derive(Default)]
pub struct FrontEndState {
    pub cfg: PlanConfig,
    /// Pins reserved for the rest of the system — removed before solving.
    pub exclude: BTreeSet<PinId>,
    key: Option<MemoKey>,
    plan: FrontEndPlan,
}

impl FrontEndState {
    /// Re-solve iff any input (chip, footprint, config, reservations) changed.
    fn ensure(&mut self, raw: &'static RawMcuData, footprint: Option<&'static PinoutRecord>) {
        let fp = footprint.map(|r| r as *const PinoutRecord as usize).unwrap_or(0);
        let key: MemoKey =
            (raw as *const RawMcuData as usize, fp, self.cfg, self.exclude.clone());
        if self.key.as_ref() == Some(&key) {
            return;
        }
        let base: BTreeSet<PinId> = match footprint {
            Some(r) => bonded_pins(r),
            None => capabilities(raw, None).into_iter().map(|c| c.pin).collect(),
        };
        let eff: BTreeSet<PinId> = base.difference(&self.exclude).copied().collect();
        self.plan = plan(raw, Some(&eff), self.cfg);
        self.key = Some(key);
    }
}

pub fn show(
    ui: &mut egui::Ui,
    raw: &'static RawMcuData,
    footprint: Option<&'static PinoutRecord>,
    st: &mut FrontEndState,
) {
    ui.horizontal(|ui| {
        ui.label(RichText::new(format!("{} — multi-channel front-end planner", raw.name)).strong());
        ui.label(RichText::new("(single-ended logging channels)").weak());
    });
    ui.label(
        RichText::new(
            "Picks N single-ended input pins (in differential pairs) to keep the most options open: as many as possible also a COMP trigger or an OPAMP programmable-gain stage, pairs on distinct opamps and read on different ADCs (simultaneous sampling).",
        )
        .weak()
        .small(),
    );
    ui.separator();

    // ---- Controls: channel count + objective emphasis. ----
    ui.horizontal(|ui| {
        ui.label("Channels:");
        ui.add(egui::Slider::new(&mut st.cfg.channels, 2..=32).step_by(2.0));
        ui.label(RichText::new(format!("= {} pairs", st.cfg.channels / 2)).weak());
        ui.separator();
        ui.label("Priority:");
        let cur = weights(&st.cfg);
        if ui.selectable_label(cur == BALANCED, "Balanced").clicked() {
            set_weights(&mut st.cfg, BALANCED);
        }
        if ui.selectable_label(cur == FAVOR_TRIGGERS, "Favor triggers").clicked() {
            set_weights(&mut st.cfg, FAVOR_TRIGGERS);
        }
        if ui.selectable_label(cur == FAVOR_PGA, "Favor PGA").clicked() {
            set_weights(&mut st.cfg, FAVOR_PGA);
        }
    });
    // Zero-cross detection: carve N pairs with a straddling comparator (COMP+ on
    // one end, external COMP− on the other → fires on the differential crossing).
    ui.horizontal(|ui| {
        let max_zc = st.cfg.channels / 2;
        if st.cfg.zero_cross_target > max_zc {
            st.cfg.zero_cross_target = max_zc;
        }
        ui.label("Zero-cross pairs:")
            .on_hover_text("Pairs to give a comparator straddling both ends (COMP+ / external COMP−) for differential zero-cross detection. Spends a comparator per pair and forces the pair onto a specific pin combo.");
        ui.add(egui::Slider::new(&mut st.cfg.zero_cross_target, 0..=max_zc));
    });

    // Reserved-pin chips.
    if !st.exclude.is_empty() {
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("Reserved (click to free):").weak().small());
            let mut free: Option<PinId> = None;
            for p in &st.exclude {
                if ui.small_button(RichText::new(p.name()).color(DIM)).clicked() {
                    free = Some(*p);
                }
            }
            if let Some(p) = free {
                st.exclude.remove(&p);
            }
            if ui.small_button("clear all").clicked() {
                st.exclude.clear();
            }
        });
    }

    // Solve (memoized) and take a cheap copy so the borrow is released for the
    // interactive exclude-clicks below.
    st.ensure(raw, footprint);
    let p = st.plan.clone();
    let cel = &p.ceiling;

    ui.separator();
    // ---- Ceiling + achieved summary. ----
    ui.horizontal_wrapped(|ui| {
        ui.label(RichText::new("Ceiling:").strong());
        ui.label(format!(
            "{} bonded analog pins · {} comparators · {} opamps · ≤{} full-PGA pairs · ≤{} triggers",
            cel.adc_pins, cel.comps, cel.opamps, cel.max_pga_pairs, cel.max_triggers
        ));
    });
    ui.horizontal_wrapped(|ui| {
        ui.label(RichText::new("Achieved:").strong().color(OPAMP_COL));
        let xadc = p.pairs.iter().filter(|x| x.cross_adc).count();
        let zc = p.zero_cross_pairs();
        ui.label(format!(
            "{} full-PGA pairs · {} PGA channels · {} triggers · {} zero-cross pairs · {}/{} simultaneous-sample · score {}",
            p.full_pga_pairs(),
            p.pga_channels(),
            p.triggers(),
            zc,
            xadc,
            p.pairs.len(),
            p.score,
        ));
    });
    for n in &p.notes {
        ui.label(RichText::new(format!("• {n}")).color(DIM).small());
    }
    if p.unplaced_channels > 0 {
        ui.label(
            RichText::new(format!("⚠ {} channel(s) unplaced — reserve fewer pins or choose a larger package.", p.unplaced_channels))
                .color(FLAG_COL),
        );
    }
    ui.separator();
    ui.label(RichText::new("Click any pin to reserve it (removed from the plan, then re-solved).").weak().small());

    // ---- Pair table. Collect a click into a local, apply after the loop. ----
    let mut reserve: Option<PinId> = None;
    egui::ScrollArea::vertical().show(ui, |ui| {
        egui::Grid::new("frontend_pairs").striped(true).num_columns(4).show(ui, |ui| {
            ui.label(RichText::new("Pair").strong());
            ui.label(RichText::new("+ end").strong());
            ui.label(RichText::new("− end").strong());
            ui.label(RichText::new("pair").strong());
            ui.end_row();

            for pair in &p.pairs {
                ui.label(format!("{}", pair.index));
                if channel_cell(ui, &pair.pos) {
                    reserve = Some(pair.pos.pin);
                }
                if pair.pos.pin == pair.neg.pin {
                    ui.label(RichText::new("(unpaired)").color(DIM).italics());
                } else if channel_cell(ui, &pair.neg) {
                    reserve = Some(pair.neg.pin);
                }
                ui.horizontal(|ui| {
                    if pair.full_pga {
                        ui.label(RichText::new("FULL-PGA").color(OPAMP_COL).small());
                    }
                    if let Some(zc) = pair.zero_cross {
                        ui.label(RichText::new(format!("↕ 0-cross {zc}")).color(COMP_COL).small())
                            .on_hover_text("A comparator straddles the pair (COMP+ on one end, external COMP− on the other) → fires on the differential zero-crossing.");
                    }
                    if pair.cross_adc {
                        ui.label(RichText::new("⇉ sim").color(ADC_COL).small())
                            .on_hover_text("Both ends read on different ADCs → sample simultaneously (good CMRR when subtracted).");
                    }
                });
                ui.end_row();
            }
        });
    });
    if let Some(p) = reserve {
        st.exclude.insert(p);
    }
}

/// Render one channel as a row of chips; returns true if its pin was clicked
/// (to be reserved).
fn channel_cell(ui: &mut egui::Ui, c: &ChannelPlan) -> bool {
    let mut clicked = false;
    ui.horizontal(|ui| {
        if ui
            .add(egui::Button::new(RichText::new(c.pin.name()).color(PIN_COL).strong()).small().frame(false))
            .on_hover_text("Click to reserve this pin for something else and re-solve")
            .clicked()
        {
            clicked = true;
        }
        let (adc, ch) = (c.read.adc(), c.read.ch());
        let read = match c.read {
            AdcRead::Direct { .. } => format!("{adc}.{ch}"),
            AdcRead::ViaOpamp { .. } => format!("→{adc}.{ch}"),
        };
        ui.label(RichText::new(read).color(ADC_COL).small());
        if let Some(op) = c.opamp {
            ui.label(RichText::new(format!("PGA {op}")).color(OPAMP_COL).small());
        }
        if let Some(cp) = c.comp {
            ui.label(RichText::new(format!("trig {cp}")).color(COMP_COL).small());
        }
    });
    clicked
}
