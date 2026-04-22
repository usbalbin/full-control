//! Per-phase timing-diagram view. Draws one strip per PCM phase showing
//! the counter ramp, CH1 (HS_IN) and CH2 (LS_IN) waveforms, and the SET /
//! RST events that drive them — rendered live from the design state.
//!
//! Numeric duty / DEM-off timings are schematic placeholders (assumed ~30%
//! nominal), since the planner doesn't track actual operating point. The
//! value is in showing the *shape* of how each PCM architecture plays out
//! over one switching cycle.

use eframe::egui::{
    self,
    epaint::CubicBezierShape,
    epaint::PathStroke,
    Color32, FontId, Pos2, Rect, Sense, Stroke, Vec2,
};

use crate::requirements::Assignment;

const STRIP_H: f32 = 120.0;
const STRIP_PAD: f32 = 10.0;
const LABEL_W: f32 = 180.0;

const C_BG: Color32 = Color32::from_gray(28);
const C_FG: Color32 = Color32::from_gray(220);
const C_AXIS: Color32 = Color32::from_gray(80);
const C_COUNTER: Color32 = Color32::from_rgb(120, 170, 220);
const C_CH1: Color32 = Color32::from_rgb(100, 200, 120);
const C_CH2: Color32 = Color32::from_rgb(220, 140, 80);
const C_SET: Color32 = Color32::from_rgb(130, 220, 140);
const C_RST: Color32 = Color32::from_rgb(230, 130, 110);

pub fn show(ui: &mut egui::Ui, assignments: &[Option<Assignment>]) {
    let phases: Vec<&Assignment> = assignments
        .iter()
        .flatten()
        .filter(|a| matches!(a, Assignment::PcmPhase { .. } | Assignment::PcmPhaseExternal { .. }))
        .collect();
    if phases.is_empty() {
        ui.label(egui::RichText::new("(no PCM phases in this design)").weak());
        return;
    }

    let avail = ui.available_size_before_wrap();
    let w = avail.x;
    let h = (phases.len() as f32 * (STRIP_H + STRIP_PAD)).max(STRIP_H + STRIP_PAD);
    let (rect, _) = ui.allocate_exact_size(Vec2::new(w, h), Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, C_BG);

    for (i, a) in phases.iter().enumerate() {
        let top = rect.top() + i as f32 * (STRIP_H + STRIP_PAD) + STRIP_PAD / 2.0;
        let strip = Rect::from_min_size(
            Pos2::new(rect.left() + 8.0, top),
            Vec2::new(rect.width() - 16.0, STRIP_H),
        );
        draw_phase_strip(&painter, strip, i, a);
    }
}

fn draw_phase_strip(painter: &egui::Painter, strip: Rect, idx: usize, a: &Assignment) {
    painter.rect_filled(strip, 3.0, Color32::from_gray(22));

    let title = phase_title(idx, a);
    painter.text(
        Pos2::new(strip.left() + 4.0, strip.top() + 4.0),
        egui::Align2::LEFT_TOP,
        title,
        FontId::monospace(11.0),
        C_FG,
    );

    // Waveform area = strip minus left label column and top title.
    let wave_left = strip.left() + LABEL_W;
    let wave_right = strip.right() - 12.0;
    let wave_width = wave_right - wave_left;
    let wave_top = strip.top() + 20.0;
    let wave_bot = strip.bottom() - 6.0;
    let wave_height = wave_bot - wave_top;

    // Row layout: counter (top 40%), CH1 (middle 30%), CH2 (bottom 30%).
    let counter_top = wave_top;
    let counter_bot = wave_top + wave_height * 0.40;
    let ch1_top = counter_bot + 4.0;
    let ch1_bot = ch1_top + wave_height * 0.25;
    let ch2_top = ch1_bot + 4.0;
    let ch2_bot = wave_bot;

    // Row labels
    for (y_top, y_bot, name) in [
        (counter_top, counter_bot, "COUNTER"),
        (ch1_top, ch1_bot, "CH1 (HS)"),
        (ch2_top, ch2_bot, "CH2 (LS)"),
    ] {
        painter.text(
            Pos2::new(strip.left() + 4.0, (y_top + y_bot) / 2.0),
            egui::Align2::LEFT_CENTER,
            name,
            FontId::monospace(10.0),
            C_FG,
        );
        painter.line_segment(
            [Pos2::new(wave_left, y_bot), Pos2::new(wave_right, y_bot)],
            Stroke::new(0.5, C_AXIS),
        );
    }

    // Schematic duty: 30% on-time for CH1. Deadtime is drawn wider than
    // reality (~20% of the cycle) so the auto-delay label between the two
    // edges has room without running into the CH1 falling edge or the
    // CH2 rising edge.
    let duty_end = wave_left + wave_width * 0.30;
    let ls_start = duty_end + wave_width * 0.20;
    let (dem, _has_zcd) = match a {
        Assignment::PcmPhase { dem, zcd_eev, .. } => (*dem, zcd_eev.is_some()),
        Assignment::PcmPhaseExternal { dem, zcd_eev, .. } => (*dem, zcd_eev.is_some()),
        _ => (false, false),
    };
    let ls_end = if dem {
        // DEM: LS turns off at ZCD event, ~70% through period.
        wave_left + wave_width * 0.70
    } else {
        wave_right
    };

    // Counter ramp: 0 → PER across the strip.
    let ramp_start = Pos2::new(wave_left, counter_bot);
    let ramp_end = Pos2::new(wave_right, counter_top);
    painter.line_segment([ramp_start, ramp_end], Stroke::new(2.0, C_COUNTER));
    // Tick at PER reset.
    painter.line_segment(
        [ramp_end, Pos2::new(wave_right, counter_bot)],
        Stroke::new(1.0, C_COUNTER),
    );

    // CH1: high from period start to duty_end.
    draw_digital(
        painter,
        wave_left, wave_right,
        ch1_top, ch1_bot,
        &[(wave_left, duty_end)],
        C_CH1,
    );

    // CH2: high from ls_start to ls_end.
    draw_digital(
        painter,
        wave_left, wave_right,
        ch2_top, ch2_bot,
        &[(ls_start, ls_end)],
        C_CH2,
    );

    // Event markers.
    mark_event(painter, wave_left, counter_top, counter_bot, "SET CH1 (TimRst)", C_SET);
    let rst1_label = match a {
        Assignment::PcmPhase { .. } => "RST CH1 (EEV: internal COMP peak)",
        Assignment::PcmPhaseExternal { .. } => "RST CH1 (EEV: external COMP peak)",
        _ => "RST CH1",
    };
    mark_event(painter, duty_end, counter_top, counter_bot, rst1_label, C_RST);

    // CH2 set source depends on architecture:
    //   - no DEM: complementary output via deadtime module (DTEN)
    //   - any DEM: auto-delayed CR4 (CMP4 paired with CPT2 per RM0440 §28.3)
    let set2_label = if dem {
        "SET CH2 (CMP4 auto-delayed, ZVS)"
    } else {
        "SET CH2 (deadtime module / DTEN)"
    };
    mark_event(painter, ls_start, ch2_top - 14.0, ch2_top, set2_label, C_SET);

    if dem {
        let rst2_label = match a {
            Assignment::PcmPhase { zcd_eev: Some(_), .. } => {
                "RST CH2 (EEV: external ZCD comp)"
            }
            Assignment::PcmPhase { zcd_eev: None, .. } => {
                "RST CH2 (EEV: same COMP, DMA INM-swap to sensor Vzcr)"
            }
            Assignment::PcmPhaseExternal { .. } => {
                "RST CH2 (EEV: external ZCD comp)"
            }
            _ => "RST CH2 (ZCD)",
        };
        mark_event(painter, ls_end, ch2_top - 14.0, ch2_top, rst2_label, C_RST);

        // Visualise the auto-delay capture path: CPT2 captures when HS
        // turns off (at duty_end), CMP4 fires after a programmable delay
        // -> SET CH2 (at ls_start). Thin line between the two points + label.
        // Label sits at the top of the CH1 row (empty space since CH1 is
        // low after duty_end) so it doesn't collide with the SET CH2
        // label that lives just above ch2_top.
        let auto_col = Color32::from_rgb(120, 170, 220);
        painter.line_segment(
            [Pos2::new(duty_end, ch1_bot), Pos2::new(ls_start, ch2_top)],
            Stroke::new(1.0, auto_col),
        );
        painter.text(
            Pos2::new((duty_end + ls_start) / 2.0, ch1_top + 2.0),
            egui::Align2::CENTER_TOP,
            "CPT2 -> CMP4 (delay)",
            FontId::monospace(8.5),
            auto_col,
        );
    } else {
        // No DEM: CH2 turns off at end of period (complementary).
        mark_event(painter, wave_right, ch2_top - 14.0, ch2_top, "RST CH2 (TimPer, complementary)", C_RST);
    }
}

fn draw_digital(
    painter: &egui::Painter,
    left: f32, right: f32,
    top: f32, bot: f32,
    high_intervals: &[(f32, f32)],
    color: Color32,
) {
    // Baseline
    painter.line_segment(
        [Pos2::new(left, bot), Pos2::new(right, bot)],
        Stroke::new(1.5, color),
    );
    for &(hs, he) in high_intervals {
        // Vertical up at hs, horizontal high, vertical down at he.
        painter.line_segment([Pos2::new(hs, bot), Pos2::new(hs, top)], Stroke::new(1.5, color));
        painter.line_segment([Pos2::new(hs, top), Pos2::new(he, top)], Stroke::new(1.5, color));
        painter.line_segment([Pos2::new(he, top), Pos2::new(he, bot)], Stroke::new(1.5, color));
    }
}

fn mark_event(painter: &egui::Painter, x: f32, y_top: f32, y_bot: f32, label: &str, color: Color32) {
    painter.line_segment(
        [Pos2::new(x, y_top), Pos2::new(x, y_bot)],
        Stroke::new(1.0, color),
    );
    painter.text(
        Pos2::new(x + 2.0, y_top),
        egui::Align2::LEFT_TOP,
        label,
        FontId::monospace(8.5),
        color,
    );
    // Keep bezier import alive (we may switch to curved event markers later)
    let _ = |p: Pos2| {
        CubicBezierShape::from_points_stroke(
            [p, p, p, p],
            false,
            Color32::TRANSPARENT,
            PathStroke::new(0.0, Color32::TRANSPARENT),
        )
    };
}

fn phase_title(idx: usize, a: &Assignment) -> String {
    match a {
        Assignment::PcmPhase { alloc, timer, dem, zcd_eev } => {
            let mode = match (*dem, zcd_eev.is_some()) {
                (false, _) => "internal, no DEM",
                (true, false) => "internal + DEM (INM-swap)",
                (true, true) => "hybrid (int peak, ext ZCD)",
            };
            format!(
                "Phase {}: {:?} ({})  [{:?}/{:?}/{:?}]",
                idx + 1, timer, mode, alloc.dac, alloc.comp, alloc.eev
            )
        }
        Assignment::PcmPhaseExternal { timer, dem, .. } => {
            let mode = if *dem { "external comp + DEM (ext ZCD)" } else { "external comp" };
            format!("Phase {}: {:?} ({})", idx + 1, timer, mode)
        }
        _ => format!("Phase {}: ?", idx + 1),
    }
}
