//! Draw the COMP/DAC/EEV/FLT slice of the G474 fabric, colour-coded by the
//! current selection. Bezier edges, barycenter-ordered columns to reduce
//! crossings, and hover tooltips listing each node's reachable neighbours.

use eframe::egui::{
    self,
    epaint::{CubicBezierShape, PathStroke},
    Color32, FontId, Pos2, Sense, Stroke, Vec2,
};

use crate::g474::*;
use crate::requirements::{
    MonitorPurpose, PhaseEdge, Resource, ResourceBag, SequencerKind, TimerInputSlot,
    TriggerSource,
};
use crate::solver::{PhaseAllocation, ShortCircuitFault};

#[derive(Clone)]
pub struct AdcSequencerView {
    pub adc: AdcInstance,
    pub kind: SequencerKind,
    pub trigger: TriggerSource,
    pub conversions: Vec<AdcConversionView>,
}

#[derive(Clone)]
pub struct AdcConversionView {
    pub adc: AdcInstance,
    pub channel: u8,
    pub purpose: MonitorPurpose,
    pub pin: Option<crate::pinout::Pin>,
}

pub struct Selection<'a> {
    pub phases: &'a [PhaseAllocation],
    pub external_eevs: &'a [CrossbarSource],
    /// Phase-edge list: EEV → Timer routed into a specific output slot
    /// (SET/RST of CH1/CH2). Used by fabric_view to draw edges landing on
    /// the right slot of the timer block.
    pub phase_edges: &'a [PhaseEdge],
    pub used_timers: &'a [HrtimId],
    pub fault: Option<ShortCircuitFault>,
    pub drive_dac: Option<DacId>,
    /// Full resource bag (locks + assignment-consumed). Used to render
    /// CR / CPT claim state per timer.
    pub used: &'a std::collections::HashSet<crate::requirements::Resource>,
    /// Master-timer output event (if any sequencer is triggered from the
    /// master). Highlights the corresponding output slot on the master block.
    pub master_trigger_event: Option<CrossbarSource>,
    /// Optional one-liner describing the sampling-plan route. Rendered
    /// under the master block.
    pub plan_label: Option<String>,
    /// For each internal PCM phase, which HRTIM sub-timer drives its fast
    /// DAC sawtooth (each DAC3/DAC4 channel's trigger source is selectable
    /// via the DAC's STRGINSEL field — any timer can drive any channel).
    pub phase_dac_timers: &'a [(DacId, HrtimId)],
    /// Active ADC sequencers + their conversions, rendered as a row of
    /// blocks at the bottom of the fabric canvas with edges back to the
    /// trigger source (master or sub-timer).
    pub adc_sequencers: &'a [AdcSequencerView],
    /// HRTIM sub-timer uses that aren't PCM phases (voltage-mode PWM,
    /// phase-shift full-bridge, External). Rendered as annotated timer
    /// blocks in the Timer column — no DAC/COMP/EEV edges since these
    /// roles don't go through the current-mode fabric.
    pub non_pcm_hrtim: &'a [(HrtimId, String)],
    /// Phase-shift peer links between sub-timers — drawn as a separate
    /// edge style from the PCM DAC→COMP→EEV chain.
    pub phase_shift_links: &'a [(HrtimId, HrtimId)],
}

const COL_DAC: f32 = 0.07;
const COL_COMP: f32 = 0.28;
const COL_EEV: f32 = 0.50;
const COL_TIMER: f32 = 0.72;
const COL_FLT: f32 = 0.94;

const NODE_R: f32 = 8.0;
const HOVER_R: f32 = 12.0;
const TIMER_BLOCK_W: f32 = 100.0;
const TIMER_BLOCK_H: f32 = 90.0;
const MASTER_BLOCK_H: f32 = 58.0;

const C_FREE: Color32 = Color32::from_rgb(120, 120, 130);
const C_USED_PHASE: Color32 = Color32::from_rgb(60, 130, 80);
const C_USED_FAULT: Color32 = Color32::from_rgb(180, 100, 60);
const C_USED_DRIVE: Color32 = Color32::from_rgb(140, 95, 180);
const C_USED_EXT: Color32 = Color32::from_rgb(55, 140, 160);
const C_EDGE_BG: Color32 = Color32::from_gray(55);
const C_EDGE_PHASE: Color32 = Color32::from_rgb(120, 220, 140);
const C_EDGE_FAULT: Color32 = Color32::from_rgb(240, 160, 100);
const C_LABEL: Color32 = Color32::from_gray(220);

const ALL_DACS: &[DacId] = &[
    DacId::Dac1Ch1, DacId::Dac1Ch2, DacId::Dac2Ch1,
    DacId::Dac3Ch1, DacId::Dac3Ch2, DacId::Dac4Ch1, DacId::Dac4Ch2,
];
const ALL_COMPS: &[CompId] = &[
    CompId::Comp1, CompId::Comp2, CompId::Comp3, CompId::Comp4,
    CompId::Comp5, CompId::Comp6, CompId::Comp7,
];
const ALL_EEVS: &[CrossbarSource] = &[
    CrossbarSource::Eev1, CrossbarSource::Eev2, CrossbarSource::Eev3,
    CrossbarSource::Eev4, CrossbarSource::Eev5, CrossbarSource::Eev6,
    CrossbarSource::Eev7, CrossbarSource::Eev8, CrossbarSource::Eev9,
    CrossbarSource::Eev10,
];
const ALL_FLTS: &[HrtimFltId] = &[
    HrtimFltId::Flt1, HrtimFltId::Flt2, HrtimFltId::Flt3,
    HrtimFltId::Flt4, HrtimFltId::Flt5, HrtimFltId::Flt6,
];
const ALL_TIMERS_FIXED: &[HrtimId] = &[
    HrtimId::TimA, HrtimId::TimB, HrtimId::TimC,
    HrtimId::TimD, HrtimId::TimE, HrtimId::TimF,
];

struct Layout {
    dacs: Vec<DacId>,
    comps: Vec<CompId>,
    eevs: Vec<CrossbarSource>,
    timers: Vec<HrtimId>,
    flts: Vec<HrtimFltId>,
}

impl Layout {
    /// Sort each column so that phase-used items sit in phase order (phase 0
    /// at the top, phase 1 below it, …). Unused items fall after, ordered by
    /// structural barycenter relative to the *already-sorted* previous
    /// column so edges between unused nodes don't cross gratuitously either.
    ///
    /// The result is that the main DAC → COMP → EEV → Timer line for each
    /// phase runs at roughly the same row across columns — much straighter
    /// than the pure-structural barycenter the old layout used.
    fn with_selection(sel: &Selection) -> Self {
        let dacs = sort_active_first(
            ALL_DACS,
            |d| sel.phases.iter().position(|p| p.dac == *d),
            // No previous column for DACs; unused stay in declaration order.
            |d| vec![ALL_DACS.iter().position(|x| x == d).unwrap() as f32],
        );
        let comps = sort_active_first(
            ALL_COMPS,
            |c| sel.phases.iter().position(|p| p.comp == *c),
            |c| DAC_TO_COMP
                .iter()
                .filter(|(_, cs)| cs.contains(c))
                .filter_map(|(d, _)| dacs.iter().position(|x| x == d).map(|i| i as f32))
                .collect(),
        );
        let eevs = sort_active_first(
            ALL_EEVS,
            |e| sel.phases.iter().position(|p| p.eev == *e),
            |e| COMP_TO_EEV
                .iter()
                .filter(|(_, es)| es.contains(e))
                .filter_map(|(c, _)| comps.iter().position(|x| x == c).map(|i| i as f32))
                .collect(),
        );
        // Timers: sorted by the phase whose EEV lands on them (via phase_edges).
        let timers = sort_active_first(
            ALL_TIMERS_FIXED,
            |t| {
                sel.phase_edges
                    .iter()
                    .filter(|e| e.timer == *t)
                    .find_map(|e| sel.phases.iter().position(|p| p.eev == e.eev))
            },
            |t| vec![ALL_TIMERS_FIXED.iter().position(|x| x == t).unwrap() as f32],
        );
        // FLTs: fault timer (if any) goes first; unused sorted by connected
        // COMP positions.
        let flts = sort_active_first(
            ALL_FLTS,
            |f| sel.fault.and_then(|x| if x.flt == *f { Some(0) } else { None }),
            |f| COMP_TO_FLT
                .iter()
                .filter(|(_, fs)| fs.contains(f))
                .filter_map(|(c, _)| comps.iter().position(|x| x == c).map(|i| i as f32))
                .collect(),
        );
        Self { dacs, comps, eevs, timers, flts }
    }
}

/// Two-tier sort: items with an `active_index` (e.g. phase index) come first
/// in that order, then remaining items ordered by the barycenter of their
/// structural connections to the previous column.
fn sort_active_first<T: Copy>(
    items: &[T],
    active_index: impl Fn(&T) -> Option<usize>,
    structural_barycenter: impl Fn(&T) -> Vec<f32>,
) -> Vec<T> {
    let mut weighted: Vec<(T, (u32, f32))> = items
        .iter()
        .map(|t| match active_index(t) {
            Some(i) => (*t, (0, i as f32)),
            None => {
                let xs = structural_barycenter(t);
                let avg = if xs.is_empty() {
                    f32::MAX
                } else {
                    xs.iter().sum::<f32>() / xs.len() as f32
                };
                (*t, (1, avg))
            }
        })
        .collect();
    weighted.sort_by(|a, b| {
        a.1.0
            .cmp(&b.1.0)
            .then(a.1.1.partial_cmp(&b.1.1).unwrap_or(std::cmp::Ordering::Equal))
    });
    weighted.into_iter().map(|(t, _)| t).collect()
}

pub fn show(ui: &mut egui::Ui, sel: &Selection, locks: &ResourceBag) -> Option<Resource> {
    let avail = ui.available_size_before_wrap();
    let (rect, response) = ui.allocate_exact_size(
        Vec2::new(avail.x, avail.y.max(360.0)),
        Sense::click(),
    );
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, Color32::from_gray(28));

    let layout = Layout::with_selection(sel);

    let used_dacs: Vec<DacId> = sel.phases.iter().map(|p| p.dac).collect();
    let used_comps: Vec<CompId> = sel.phases.iter().map(|p| p.comp).collect();
    let used_eevs: Vec<CrossbarSource> = sel.phases.iter().map(|p| p.eev).collect();

    // Reserve right-side space for the ADC sequencer panel (if any) so
    // the column layout doesn't collide with it. ADC blocks live to the
    // right of FLT, with horizontal edges from trigger source to block.
    let adc_reserve = if sel.adc_sequencers.is_empty() { 0.0 } else { ADC_PANEL_W + 8.0 };
    let column_rect = egui::Rect::from_min_max(
        rect.min,
        Pos2::new(rect.right() - adc_reserve, rect.bottom()),
    );

    let pd = |d: DacId| pos(&column_rect, COL_DAC, layout.dacs.iter().position(|&x| x == d).unwrap(), layout.dacs.len());
    let pc = |c: CompId| pos(&column_rect, COL_COMP, layout.comps.iter().position(|&x| x == c).unwrap(), layout.comps.len());
    let pe = |e: CrossbarSource| pos(&column_rect, COL_EEV, layout.eevs.iter().position(|&x| x == e).unwrap(), layout.eevs.len());
    // Timer column: slot 0 reserved for the Master timer block, TimA..TimF fill slots 1..=6.
    let timer_slots_total = layout.timers.len() + 1;
    let pt = |t: HrtimId| pos(&column_rect, COL_TIMER, layout.timers.iter().position(|&x| x == t).unwrap() + 1, timer_slots_total);
    let master_pos = pos(&column_rect, COL_TIMER, 0, timer_slots_total);
    let pf = |f: HrtimFltId| pos(&column_rect, COL_FLT, layout.flts.iter().position(|&x| x == f).unwrap(), layout.flts.len());

    for &(dac, comps) in DAC_TO_COMP {
        for &c in comps {
            draw_edge(&painter, pd(dac), pc(c), Stroke::new(1.0, C_EDGE_BG));
        }
    }
    for &(comp, eevs) in COMP_TO_EEV {
        for &e in eevs {
            draw_edge(&painter, pc(comp), pe(e), Stroke::new(1.0, C_EDGE_BG));
        }
    }
    for &(comp, flts) in COMP_TO_FLT {
        for &f in flts {
            draw_edge(&painter, pc(comp), pf(f), Stroke::new(1.0, C_EDGE_BG));
        }
    }

    for p in sel.phases {
        draw_edge(&painter, pd(p.dac), pc(p.comp), Stroke::new(2.5, C_EDGE_PHASE));
        draw_edge(&painter, pc(p.comp), pe(p.eev), Stroke::new(2.5, C_EDGE_PHASE));
    }
    for edge in sel.phase_edges {
        let color = if sel.external_eevs.contains(&edge.eev) { C_USED_EXT } else { C_EDGE_PHASE };
        let slot_pos = timer_slot_pos(&column_rect, &layout, edge.timer, edge.slot);
        draw_edge(&painter, pe(edge.eev), slot_pos, Stroke::new(2.5, color));
    }
    if let Some(f) = sel.fault {
        draw_edge(&painter, pd(f.dac), pc(f.comp), Stroke::new(2.5, C_EDGE_FAULT));
        draw_edge(&painter, pc(f.comp), pf(f.flt), Stroke::new(2.5, C_EDGE_FAULT));
    }

    for &d in &layout.dacs {
        let p = pd(d);
        let color = if Some(d) == sel.drive_dac {
            C_USED_DRIVE
        } else if sel.fault.map(|f| f.dac) == Some(d) {
            C_USED_FAULT
        } else if used_dacs.contains(&d) {
            C_USED_PHASE
        } else {
            C_FREE
        };
        node(
            &painter, p, color,
            locks.contains(&Resource::Dac(d)),
            &format!("{:?}", d), egui::Align2::RIGHT_CENTER,
        );
        // For fast DACs used by a phase, show which timer triggers the
        // sawtooth (any TIMA..TIMF can drive any DAC3/DAC4 channel via
        // the DAC's STRGINSEL register). Placed below the node so it
        // doesn't sit on the DAC→COMP edge.
        if d.is_fast() {
            if let Some((_, timer)) = sel.phase_dac_timers.iter().find(|(dac, _)| *dac == d) {
                painter.text(
                    Pos2::new(p.x, p.y + NODE_R + 2.0),
                    egui::Align2::CENTER_TOP,
                    format!("{:?} saw", timer),
                    FontId::monospace(9.0),
                    Color32::from_rgb(200, 180, 80),
                );
            }
        }
    }
    for &c in &layout.comps {
        let p = pc(c);
        let color = if sel.fault.map(|f| f.comp) == Some(c) {
            C_USED_FAULT
        } else if used_comps.contains(&c) {
            C_USED_PHASE
        } else {
            C_FREE
        };
        node(
            &painter, p, color,
            locks.contains(&Resource::Comp(c)),
            &format!("{:?}", c), egui::Align2::CENTER_BOTTOM,
        );
    }
    for &e in &layout.eevs {
        let p = pe(e);
        let color = if sel.external_eevs.contains(&e) {
            C_USED_EXT
        } else if used_eevs.contains(&e) {
            C_USED_PHASE
        } else {
            C_FREE
        };
        node(
            &painter, p, color,
            locks.contains(&Resource::Eev(e)),
            &format!("{:?}", e), egui::Align2::CENTER_BOTTOM,
        );
    }
    // Master timer block (slot 0 of the timer column).
    draw_master_block(&painter, master_pos, sel.master_trigger_event, sel.used);
    // Master MCR claim tag (e.g. "MCR1 MCR3") below the plan label line.
    let master_tags: Vec<String> = (1u8..=4)
        .filter(|n| sel.used.contains(&Resource::MasterCompareSlot(*n)))
        .map(|n| format!("MCR{}", n))
        .collect();
    let label_y_off = MASTER_BLOCK_H / 2.0 + 8.0;
    if let Some(label) = &sel.plan_label {
        painter.text(
            Pos2::new(master_pos.x, master_pos.y + label_y_off),
            egui::Align2::CENTER_TOP,
            label,
            FontId::monospace(10.0),
            if sel.master_trigger_event.is_some() { C_EDGE_PHASE } else { C_LABEL },
        );
    }
    if !master_tags.is_empty() {
        let y = master_pos.y
            + label_y_off
            + if sel.plan_label.is_some() { 14.0 } else { 0.0 };
        painter.text(
            Pos2::new(master_pos.x, y),
            egui::Align2::CENTER_TOP,
            master_tags.join(" "),
            FontId::monospace(9.0),
            Color32::from_rgb(200, 180, 80),
        );
    }

    // ADC sequencer panel on the right of the canvas. Each block shows
    // the sequencer's ADC + kind, its trigger source, and the attached
    // conversions. An edge runs left-to-right from the trigger source
    // (master output slot or sub-timer block) to the sequencer.
    if !sel.adc_sequencers.is_empty() {
        draw_adc_sequencer_panel(&painter, &rect, &column_rect, &layout, sel);
    }

    for &t in &layout.timers {
        let center = pt(t);
        let used_by_phase = sel.used_timers.contains(&t);
        let non_pcm_role: Option<&String> = sel
            .non_pcm_hrtim
            .iter()
            .find_map(|(tt, r)| if *tt == t { Some(r) } else { None });
        let color = if used_by_phase {
            C_USED_PHASE
        } else if non_pcm_role.is_some() {
            C_USED_EXT
        } else {
            C_FREE
        };
        draw_timer_block(
            &painter,
            center,
            color,
            locks.contains(&Resource::Timer(t)),
            &format!("{:?}", t),
        );
        // Compact role badge above the block for non-PCM uses so the
        // fabric view still surfaces voltage-mode / phase-shift / external
        // even though these don't live on the DAC→COMP→EEV chain.
        if let Some(label) = non_pcm_role {
            painter.text(
                Pos2::new(center.x, center.y - TIMER_BLOCK_H / 2.0 - 4.0),
                egui::Align2::CENTER_BOTTOM,
                label,
                FontId::monospace(9.5),
                Color32::from_rgb(130, 190, 210),
            );
        }
        // SET/RST slot markers on the left edge.
        for &slot in &[
            TimerInputSlot::Set1,
            TimerInputSlot::Rst1,
            TimerInputSlot::Set2,
            TimerInputSlot::Rst2,
        ] {
            let sp = timer_slot_pos(&column_rect, &layout, t, slot);
            painter.circle_filled(sp, 3.0, Color32::from_gray(200));
            painter.text(
                sp + Vec2::new(6.0, 0.0),
                egui::Align2::LEFT_CENTER,
                slot_label(slot),
                FontId::monospace(9.0),
                Color32::from_gray(180),
            );
        }
        // CR + CPT claim tag below the block
        let mut tags: Vec<String> = Vec::new();
        for &slot in ALL_CR_SLOTS {
            if sel.used.contains(&Resource::TimerSlot(t, slot)) {
                tags.push(format!("{:?}", slot));
            }
        }
        for &cpt in ALL_CAPTURE_UNITS {
            if sel.used.contains(&Resource::TimerCapture(t, cpt)) {
                tags.push(format!("{:?}", cpt));
            }
        }
        if !tags.is_empty() {
            painter.text(
                Pos2::new(center.x, center.y + TIMER_BLOCK_H / 2.0 + 4.0),
                egui::Align2::CENTER_TOP,
                tags.join(" "),
                FontId::monospace(9.0),
                Color32::from_rgb(200, 180, 80),
            );
        }
    }

    // Phase-shift peer links: thin dashed-style line between coupled
    // sub-timer blocks. Dedup so we don't render both (A,B) and (B,A).
    let mut seen_ps: std::collections::HashSet<(HrtimId, HrtimId)> =
        std::collections::HashSet::new();
    for &(a, b) in sel.phase_shift_links {
        let (lo, hi) = if (a as u8) < (b as u8) { (a, b) } else { (b, a) };
        if !seen_ps.insert((lo, hi)) { continue; }
        let pa = pt(a);
        let pb = pt(b);
        let left_a = Pos2::new(pa.x - TIMER_BLOCK_W / 2.0, pa.y);
        let left_b = Pos2::new(pb.x - TIMER_BLOCK_W / 2.0, pb.y);
        painter.line_segment(
            [left_a, left_b],
            Stroke::new(1.5, Color32::from_rgb(130, 190, 210)),
        );
        let mid = Pos2::new(left_a.x - 10.0, (left_a.y + left_b.y) / 2.0);
        painter.text(
            mid, egui::Align2::RIGHT_CENTER,
            "φ-shift",
            FontId::monospace(9.0),
            Color32::from_rgb(130, 190, 210),
        );
    }
    for &f in &layout.flts {
        let p = pf(f);
        let color = if sel.fault.map(|x| x.flt) == Some(f) {
            C_USED_FAULT
        } else {
            C_FREE
        };
        node(
            &painter, p, color,
            locks.contains(&Resource::Flt(f)),
            &format!("{:?}", f), egui::Align2::LEFT_CENTER,
        );
    }

    let header_y = rect.top() + 14.0;
    for (col, name) in [
        (COL_DAC, "DAC"),
        (COL_COMP, "COMP"),
        (COL_EEV, "EEV"),
        (COL_TIMER, "HRTIM Timer"),
        (COL_FLT, "HRTIM FLT"),
    ] {
        painter.text(
            Pos2::new(rect.left() + col * rect.width(), header_y),
            egui::Align2::CENTER_CENTER,
            name,
            FontId::proportional(13.0),
            C_LABEL,
        );
    }

    if let Some(hover) = response.hover_pos() {
        if let Some(tip) = slot_hover_tooltip(hover, &layout, &column_rect, sel) {
            egui::Tooltip::always_open(
                ui.ctx().clone(),
                ui.layer_id(),
                egui::Id::new("fabric_slot_hover"),
                egui::PopupAnchor::Pointer,
            )
            .show(|ui| {
                ui.label(egui::RichText::new(tip.title).strong());
                for line in tip.lines {
                    ui.label(line);
                }
            });
        } else if let Some(tip) = hover_tooltip(hover, &layout, &column_rect, sel.used) {
            egui::Tooltip::always_open(
                ui.ctx().clone(),
                ui.layer_id(),
                egui::Id::new("fabric_hover"),
                egui::PopupAnchor::Pointer,
            )
            .show(|ui| {
                ui.label(egui::RichText::new(tip.title).strong());
                for line in tip.lines {
                    ui.label(line);
                }
            });
        }
    }

    if response.clicked() {
        if let Some(click_pos) = response.interact_pointer_pos() {
            return hit_test(click_pos, &layout, &column_rect);
        }
    }
    None
}

fn hit_test(cursor: Pos2, layout: &Layout, rect: &egui::Rect) -> Option<Resource> {
    for &d in &layout.dacs {
        if cursor.distance(pos(rect, COL_DAC, layout.dacs.iter().position(|&x| x == d).unwrap(), layout.dacs.len())) < HOVER_R {
            return Some(Resource::Dac(d));
        }
    }
    for &c in &layout.comps {
        if cursor.distance(pos(rect, COL_COMP, layout.comps.iter().position(|&x| x == c).unwrap(), layout.comps.len())) < HOVER_R {
            return Some(Resource::Comp(c));
        }
    }
    for &e in &layout.eevs {
        if cursor.distance(pos(rect, COL_EEV, layout.eevs.iter().position(|&x| x == e).unwrap(), layout.eevs.len())) < HOVER_R {
            return Some(Resource::Eev(e));
        }
    }
    for &t in &layout.timers {
        if cursor.distance(pos(rect, COL_TIMER, layout.timers.iter().position(|&x| x == t).unwrap() + 1, layout.timers.len() + 1)) < HOVER_R {
            return Some(Resource::Timer(t));
        }
    }
    for &f in &layout.flts {
        if cursor.distance(pos(rect, COL_FLT, layout.flts.iter().position(|&x| x == f).unwrap(), layout.flts.len())) < HOVER_R {
            return Some(Resource::Flt(f));
        }
    }
    None
}

fn pos(rect: &egui::Rect, col_frac: f32, idx: usize, total: usize) -> Pos2 {
    let x = rect.left() + col_frac * rect.width();
    let pad_top = 36.0;
    let pad_bot = 12.0;
    let usable = (rect.height() - pad_top - pad_bot).max(50.0);
    let step = usable / (total.max(1) as f32);
    let y = rect.top() + pad_top + step * (idx as f32 + 0.5);
    Pos2::new(x, y)
}

fn slot_label(slot: TimerInputSlot) -> &'static str {
    match slot {
        TimerInputSlot::Set1 => "SET1",
        TimerInputSlot::Rst1 => "RST1",
        TimerInputSlot::Set2 => "SET2",
        TimerInputSlot::Rst2 => "RST2",
    }
}

fn timer_slot_pos(
    rect: &egui::Rect,
    layout: &Layout,
    timer: HrtimId,
    slot: TimerInputSlot,
) -> Pos2 {
    let idx = layout.timers.iter().position(|&x| x == timer).unwrap();
    // Timer column reserves slot 0 for the master block, so sub-timers live
    // at indices 1..=timers.len() out of timers.len()+1 total slots.
    let center = pos(rect, COL_TIMER, idx + 1, layout.timers.len() + 1);
    // Four slots evenly spaced on the left edge of the block.
    let y_offset = match slot {
        TimerInputSlot::Set1 => -TIMER_BLOCK_H * 0.35,
        TimerInputSlot::Rst1 => -TIMER_BLOCK_H * 0.12,
        TimerInputSlot::Set2 =>  TIMER_BLOCK_H * 0.12,
        TimerInputSlot::Rst2 =>  TIMER_BLOCK_H * 0.35,
    };
    Pos2::new(center.x - TIMER_BLOCK_W / 2.0, center.y + y_offset)
}

const ADC_PANEL_W: f32 = 240.0;
const ADC_BLOCK_PAD: f32 = 6.0;

fn draw_adc_sequencer_panel(
    painter: &egui::Painter,
    rect: &egui::Rect,
    column_rect: &egui::Rect,
    layout: &Layout,
    sel: &Selection,
) {
    let n = sel.adc_sequencers.len();
    if n == 0 { return; }
    let panel_left = rect.right() - ADC_PANEL_W;
    let panel_top = rect.top() + 32.0;
    let panel_bot = rect.bottom() - 12.0;
    let panel_h = (panel_bot - panel_top).max(60.0);
    // Background tint to separate the ADC panel from the fabric.
    painter.rect_filled(
        egui::Rect::from_min_max(
            Pos2::new(panel_left - 4.0, rect.top()),
            Pos2::new(rect.right(), rect.bottom()),
        ),
        0.0,
        Color32::from_gray(22),
    );
    painter.text(
        Pos2::new(panel_left + 6.0, rect.top() + 14.0),
        egui::Align2::LEFT_CENTER,
        "ADC sequencers",
        FontId::proportional(11.0),
        C_LABEL,
    );
    // Vertically-stacked blocks down the right-side panel.
    let block_w = ADC_PANEL_W - 8.0;
    let block_h = ((panel_h - ADC_BLOCK_PAD * (n.saturating_sub(1)) as f32) / n as f32)
        .max(40.0)
        .min(180.0);
    for (i, seq) in sel.adc_sequencers.iter().enumerate() {
        let y = panel_top + i as f32 * (block_h + ADC_BLOCK_PAD);
        let block = egui::Rect::from_min_size(
            Pos2::new(panel_left + 4.0, y),
            Vec2::new(block_w, block_h),
        );
        let triggered = matches!(seq.trigger, TriggerSource::Event(_));
        let fill = if triggered { C_USED_PHASE } else { Color32::from_gray(55) };
        painter.rect_filled(block, 4.0, fill);
        painter.rect_stroke(
            block, 4.0,
            Stroke::new(1.0, Color32::from_gray(20)),
            egui::epaint::StrokeKind::Middle,
        );
        let header = format!("{:?} {:?}", seq.adc, seq.kind);
        painter.text(
            Pos2::new(block.left() + 6.0, block.top() + 4.0),
            egui::Align2::LEFT_TOP,
            header,
            FontId::proportional(11.0),
            Color32::from_gray(250),
        );
        let trig_str = match seq.trigger {
            TriggerSource::Software => "trig: SW".to_string(),
            TriggerSource::Event(ev) => format!("trig: {:?}", ev),
        };
        painter.text(
            Pos2::new(block.left() + 6.0, block.top() + 20.0),
            egui::Align2::LEFT_TOP,
            trig_str,
            FontId::monospace(9.5),
            Color32::from_gray(220),
        );
        let mut y = block.top() + 36.0;
        for c in &seq.conversions {
            let pin_str = c.pin.map(|p| p.name()).unwrap_or_else(|| "?".to_string());
            let line = format!(
                "{} -> ADC{}_IN{} @ {}",
                c.purpose.short_name(),
                c.adc.number(),
                c.channel,
                pin_str
            );
            painter.text(
                Pos2::new(block.left() + 6.0, y),
                egui::Align2::LEFT_TOP,
                line,
                FontId::monospace(9.0),
                Color32::from_gray(240),
            );
            y += 12.0;
            if y > block.bottom() - 4.0 { break; }
        }

        // Trigger edge: horizontal flow from trigger source (timer
        // column) into the left edge of the sequencer block. Right-angle
        // route: across, then down/up to align with the block.
        if let TriggerSource::Event(ev) = seq.trigger {
            let from = trigger_source_pos(column_rect, layout, ev);
            let to = Pos2::new(block.left(), block.top() + block_h / 2.0);
            let bend_x = (from.x + to.x) / 2.0;
            let stroke = Stroke::new(1.5, C_EDGE_PHASE);
            painter.line_segment([from, Pos2::new(bend_x, from.y)], stroke);
            painter.line_segment([Pos2::new(bend_x, from.y), Pos2::new(bend_x, to.y)], stroke);
            painter.line_segment([Pos2::new(bend_x, to.y), to], stroke);
        }
    }
}

/// Where a trigger event originates on the fabric canvas. Master events
/// come out of the MASTER block's output slot; sub-timer events come out
/// of the timer block.
fn trigger_source_pos(rect: &egui::Rect, layout: &Layout, ev: CrossbarSource) -> Pos2 {
    let master_idx = 0;
    let total_timer_slots = layout.timers.len() + 1;
    let master_pos = pos(rect, COL_TIMER, master_idx, total_timer_slots);
    let master_right = master_pos.x + TIMER_BLOCK_W / 2.0;
    let outputs: [(CrossbarSource, usize); 5] = [
        (CrossbarSource::Mcr1, 0),
        (CrossbarSource::Mcr2, 1),
        (CrossbarSource::Mcr3, 2),
        (CrossbarSource::Mcr4, 3),
        (CrossbarSource::Mper, 4),
    ];
    if let Some((_, i)) = outputs.iter().find(|(o, _)| *o == ev) {
        let master_rect = egui::Rect::from_center_size(
            master_pos,
            Vec2::new(TIMER_BLOCK_W, MASTER_BLOCK_H),
        );
        let y = master_rect.top() + 18.0
            + *i as f32 * (master_rect.height() - 22.0) / 4.0;
        return Pos2::new(master_right, y);
    }
    // Sub-timer compare events: pick the timer the event belongs to.
    let timer = match ev {
        CrossbarSource::TimACr2 | CrossbarSource::TimACr3 | CrossbarSource::TimACr4
        | CrossbarSource::TimACrPer | CrossbarSource::TimACrRst => Some(HrtimId::TimA),
        CrossbarSource::TimBCr2 | CrossbarSource::TimBCr3 | CrossbarSource::TimBCr4
        | CrossbarSource::TimBCrPer | CrossbarSource::TimBCrRst => Some(HrtimId::TimB),
        CrossbarSource::TimCCr2 | CrossbarSource::TimCCr3 | CrossbarSource::TimCCr4
        | CrossbarSource::TimCCrPer | CrossbarSource::TimCCrRst => Some(HrtimId::TimC),
        CrossbarSource::TimDCr2 | CrossbarSource::TimDCr3 | CrossbarSource::TimDCr4
        | CrossbarSource::TimDCrPer | CrossbarSource::TimDCrRst => Some(HrtimId::TimD),
        CrossbarSource::TimECr2 | CrossbarSource::TimECr3 | CrossbarSource::TimECr4
        | CrossbarSource::TimECrPer | CrossbarSource::TimECrRst => Some(HrtimId::TimE),
        CrossbarSource::TimFCr2 | CrossbarSource::TimFCr3 | CrossbarSource::TimFCr4
        | CrossbarSource::TimFCrPer | CrossbarSource::TimFCrRst => Some(HrtimId::TimF),
        _ => None,
    };
    if let Some(t) = timer {
        if let Some(idx) = layout.timers.iter().position(|&x| x == t) {
            let p = pos(rect, COL_TIMER, idx + 1, total_timer_slots);
            return Pos2::new(p.x + TIMER_BLOCK_W / 2.0, p.y);
        }
    }
    // Fallback: middle-right of canvas.
    Pos2::new(rect.right() - 50.0, rect.center().y)
}

fn draw_master_block(
    painter: &egui::Painter,
    center: Pos2,
    used_output: Option<CrossbarSource>,
    used: &std::collections::HashSet<crate::requirements::Resource>,
) {
    let rect = egui::Rect::from_center_size(
        center,
        Vec2::new(TIMER_BLOCK_W, MASTER_BLOCK_H),
    );
    painter.rect_filled(rect, 4.0, Color32::from_gray(55));
    painter.rect_stroke(
        rect, 4.0,
        Stroke::new(1.0, Color32::from_gray(160)),
        egui::epaint::StrokeKind::Middle,
    );
    painter.text(
        Pos2::new(rect.left() + 8.0, rect.top() + 4.0),
        egui::Align2::LEFT_TOP,
        "MASTER",
        FontId::proportional(11.0),
        Color32::from_gray(240),
    );

    // 5 output slots on the right edge. MCR1..4 slots can be claimed by
    // ADC trigger events (via `Resource::MasterCompareSlot`); tint those
    // gold. The currently-routed trigger output wins the highlight colour.
    let outputs: [(CrossbarSource, &str, Option<u8>); 5] = [
        (CrossbarSource::Mcr1, "MCR1", Some(1)),
        (CrossbarSource::Mcr2, "MCR2", Some(2)),
        (CrossbarSource::Mcr3, "MCR3", Some(3)),
        (CrossbarSource::Mcr4, "MCR4", Some(4)),
        (CrossbarSource::Mper, "MPER", None),
    ];
    for (i, (src, lbl, cr_n)) in outputs.iter().enumerate() {
        let y = rect.top() + 18.0
            + i as f32 * (rect.height() - 22.0) / (outputs.len() - 1) as f32;
        let slot = Pos2::new(rect.right(), y);
        let highlighted = Some(*src) == used_output;
        let claimed = cr_n.is_some_and(|n| {
            used.contains(&crate::requirements::Resource::MasterCompareSlot(n))
        });
        let color = if highlighted {
            C_EDGE_PHASE
        } else if claimed {
            Color32::from_rgb(200, 180, 80)
        } else {
            Color32::from_gray(180)
        };
        painter.circle_filled(slot, 3.5, color);
        painter.text(
            Pos2::new(slot.x - 6.0, slot.y),
            egui::Align2::RIGHT_CENTER,
            lbl,
            FontId::monospace(8.5),
            color,
        );
    }
}

fn draw_timer_block(
    painter: &egui::Painter,
    center: Pos2,
    fill: Color32,
    locked: bool,
    label: &str,
) {
    let rect = egui::Rect::from_center_size(center, Vec2::new(TIMER_BLOCK_W, TIMER_BLOCK_H));
    painter.rect_filled(rect, 4.0, fill);
    let border = if locked {
        Stroke::new(2.5, Color32::from_rgb(240, 220, 100))
    } else {
        Stroke::new(1.0, Color32::from_gray(20))
    };
    painter.rect_stroke(rect, 4.0, border, egui::epaint::StrokeKind::Middle);
    if locked {
        let halo = rect.expand(3.0);
        painter.rect_stroke(halo, 6.0, Stroke::new(1.0, Color32::from_rgb(240, 220, 100)), egui::epaint::StrokeKind::Middle);
    }
    painter.text(
        rect.center(),
        egui::Align2::CENTER_CENTER,
        label,
        FontId::proportional(13.0),
        Color32::from_gray(250),
    );
}

fn draw_edge(painter: &egui::Painter, from: Pos2, to: Pos2, stroke: Stroke) {
    let mid_x = (from.x + to.x) * 0.5;
    let bezier = CubicBezierShape::from_points_stroke(
        [from, Pos2::new(mid_x, from.y), Pos2::new(mid_x, to.y), to],
        false,
        Color32::TRANSPARENT,
        PathStroke::new(stroke.width, stroke.color),
    );
    painter.add(bezier);
}

fn node(
    painter: &egui::Painter,
    pos: Pos2,
    color: Color32,
    locked: bool,
    label: &str,
    label_align: egui::Align2,
) {
    painter.circle_filled(pos, NODE_R, color);
    let border = if locked {
        Stroke::new(2.5, Color32::from_rgb(240, 220, 100))
    } else {
        Stroke::new(1.0, Color32::from_gray(20))
    };
    painter.circle_stroke(pos, NODE_R, border);
    if locked {
        painter.circle_stroke(pos, NODE_R + 3.0, Stroke::new(1.0, Color32::from_rgb(240, 220, 100)));
    }
    let label_pos = match label_align {
        egui::Align2::RIGHT_CENTER => Pos2::new(pos.x - NODE_R - 4.0, pos.y),
        egui::Align2::LEFT_CENTER => Pos2::new(pos.x + NODE_R + 4.0, pos.y),
        egui::Align2::CENTER_BOTTOM => Pos2::new(pos.x, pos.y - NODE_R - 2.0),
        _ => pos,
    };
    painter.text(label_pos, label_align, label, FontId::monospace(11.0), C_LABEL);
}

struct Tooltip {
    title: String,
    lines: Vec<String>,
}

fn hover_tooltip(
    cursor: Pos2,
    layout: &Layout,
    rect: &egui::Rect,
    used: &ResourceBag,
) -> Option<Tooltip> {
    for &d in &layout.dacs {
        if cursor.distance(pos(rect, COL_DAC, layout.dacs.iter().position(|&x| x == d).unwrap(), layout.dacs.len())) < HOVER_R {
            let comps = comps_for_dac(d);
            let kind = if d.is_fast() { "fast (15 Msps)" } else { "slow (1 Msps)" };
            return Some(Tooltip {
                title: format!("{:?}  ({})", d, kind),
                lines: vec![format!("-> COMPs: {:?}", comps)],
            });
        }
    }
    for &c in &layout.comps {
        if cursor.distance(pos(rect, COL_COMP, layout.comps.iter().position(|&x| x == c).unwrap(), layout.comps.len())) < HOVER_R {
            let dacs: Vec<_> = dacs_for_comp(c).collect();
            let eevs = eevs_for_comp(c);
            let flts = flts_for_comp(c);
            return Some(Tooltip {
                title: format!("{:?}", c),
                lines: vec![
                    format!("<- DACs: {:?}", dacs),
                    format!("-> EEVs: {:?}", eevs),
                    format!("-> FLTs: {:?}", flts),
                ],
            });
        }
    }
    for &e in &layout.eevs {
        if cursor.distance(pos(rect, COL_EEV, layout.eevs.iter().position(|&x| x == e).unwrap(), layout.eevs.len())) < HOVER_R {
            let comps: Vec<_> = COMP_TO_EEV
                .iter()
                .filter(|(_, es)| es.contains(&e))
                .map(|(c, _)| *c)
                .collect();
            return Some(Tooltip {
                title: format!("{:?}", e),
                lines: vec![format!("<- COMPs: {:?}", comps)],
            });
        }
    }
    for &f in &layout.flts {
        if cursor.distance(pos(rect, COL_FLT, layout.flts.iter().position(|&x| x == f).unwrap(), layout.flts.len())) < HOVER_R {
            let comps: Vec<_> = COMP_TO_FLT
                .iter()
                .filter(|(_, fs)| fs.contains(&f))
                .map(|(c, _)| *c)
                .collect();
            return Some(Tooltip {
                title: format!("{:?}", f),
                lines: vec![format!("<- COMPs: {:?}  (placeholder data)", comps)],
            });
        }
    }
    for &t in &layout.timers {
        if cursor.distance(pos(rect, COL_TIMER, layout.timers.iter().position(|&x| x == t).unwrap() + 1, layout.timers.len() + 1)) < HOVER_R {
            return Some(timer_tooltip(t, used));
        }
    }
    None
}

/// Per-HRTIM-timer tooltip: describes the 4 user-configurable compare slots
/// (CR1..CR4) and 2 capture units (CPT1/CPT2), showing what's claimed in
/// this design versus what the hardware allows.
/// Slot tooltip: hovering a SET/RST slot on a timer block shows what (if
/// anything) is currently routed in, and a reminder of what sources are
/// eligible for that slot.
fn slot_hover_tooltip(
    cursor: Pos2,
    layout: &Layout,
    rect: &egui::Rect,
    sel: &Selection,
) -> Option<Tooltip> {
    const SLOT_HOVER_R: f32 = 8.0;
    for &t in &layout.timers {
        for &slot in &[
            TimerInputSlot::Set1,
            TimerInputSlot::Rst1,
            TimerInputSlot::Set2,
            TimerInputSlot::Rst2,
        ] {
            let p = timer_slot_pos(rect, layout, t, slot);
            if cursor.distance(p) < SLOT_HOVER_R {
                let (ch, ev) = match slot {
                    TimerInputSlot::Set1 => ("CH1", "SET"),
                    TimerInputSlot::Rst1 => ("CH1", "RST"),
                    TimerInputSlot::Set2 => ("CH2", "SET"),
                    TimerInputSlot::Rst2 => ("CH2", "RST"),
                };
                let mut lines = vec![format!("Output {} ({} event)", ch, ev)];
                let routed: Vec<String> = sel
                    .phase_edges
                    .iter()
                    .filter(|e| e.timer == t && e.slot == slot)
                    .map(|e| format!("{:?}", e.eev))
                    .collect();
                if routed.is_empty() {
                    lines.push("Routed: (nothing from the crossbar)".to_string());
                } else {
                    lines.push(format!("Routed: {}", routed.join(", ")));
                }
                lines.push(
                    "Eligible sources: own CMP1-4, PER, RST, EEV1-10, \
                     master CMP1-4/PER, other timers' events."
                        .to_string(),
                );
                return Some(Tooltip {
                    title: format!("{:?}: {} {}", t, ch, ev),
                    lines,
                });
            }
        }
    }
    None
}

fn timer_tooltip(t: HrtimId, used: &ResourceBag) -> Tooltip {
    let slot_role = |slot: TimerCompareSlot| -> &'static str {
        match slot {
            TimerCompareSlot::Cr1 => {
                "general-purpose compare / output set-reset. \
                 Often used for max-duty safety cap or period end."
            }
            TimerCompareSlot::Cr2 => {
                "DAC sawtooth step in slope-comp mode (RM0440 sec. 28.3.21). \
                 Exclusive when dual-channel DAC trigger is enabled. Also \
                 one of the two slots that supports auto-delayed mode."
            }
            TimerCompareSlot::Cr3 => {
                "general-purpose compare. Often used as timeout trigger for \
                 auto-delayed CMP2/CMP4 or for fault-input blanking."
            }
            TimerCompareSlot::Cr4 => {
                "general-purpose compare + one of the two auto-delayed \
                 slots. Used for ZVS-friendly (variable) deadtime when DEM \
                 is active."
            }
        }
    };
    let capture_role = |cpt: TimerCaptureUnit| -> &'static str {
        match cpt {
            TimerCaptureUnit::Cpt1 => {
                "capture-1 unit. Paired with auto-delayed CMP2 (when used \
                 in that mode - RM0440 sec. 28.3 'Auto-delayed mode')."
            }
            TimerCaptureUnit::Cpt2 => {
                "capture-2 unit. Paired with auto-delayed CMP4. Claimed \
                 by DEM deadtime."
            }
        }
    };
    let mut lines = Vec::new();
    for &slot in ALL_CR_SLOTS {
        let status = if used.contains(&Resource::TimerSlot(t, slot)) {
            "claimed"
        } else {
            "free"
        };
        lines.push(format!("{:?} [{}]: {}", slot, status, slot_role(slot)));
    }
    for &cpt in ALL_CAPTURE_UNITS {
        let status = if used.contains(&Resource::TimerCapture(t, cpt)) {
            "claimed"
        } else {
            "free"
        };
        lines.push(format!("{:?} [{}]: {}", cpt, status, capture_role(cpt)));
    }
    Tooltip {
        title: format!("{:?}", t),
        lines,
    }
}
