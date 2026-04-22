//! Physical-package view. Draws the chip outline with all pins laid out
//! in their datasheet positions. The module is a dumb renderer: the caller
//! supplies a per-pin `PinPaint` describing fill/border/label/tooltip, and
//! reports back which pin was clicked.
//!
//! Only LQFP64 is implemented here (G474R). Other packages can be added by
//! extending `map_for()` with that package's pin table.

use std::collections::HashMap;

use eframe::egui::{self, Color32, FontId, Pos2, Rect, Sense, Stroke, Vec2};

use crate::pinout::{ChipVariant, Pin};

/// One click event from the package view.
pub enum Action {
    /// User clicked a GPIO pin.
    Click(Pin),
    /// User clicked empty space (not on a pin).
    ClickEmpty,
}

/// How each pin should be painted this frame. The caller precomputes these
/// based on design state, the currently picked role (if any), and per-pin
/// `MoveResult` analysis.
#[derive(Clone)]
pub struct PinPaint {
    pub fill: Color32,
    pub border: Option<Stroke>,
    /// Small text just outside the pin (beside the GPIO name). Typically
    /// the signal the pin carries, or a speed tag.
    pub sublabel: Option<String>,
    /// Shown in a tooltip on hover.
    pub tooltip: Option<String>,
    /// Whether clicking this pin should fire an `Action::Click`.
    pub interactive: bool,
}

impl PinPaint {
    pub fn free() -> Self {
        Self {
            fill: C_PIN,
            border: None,
            sublabel: None,
            tooltip: None,
            interactive: true,
        }
    }
    pub fn with_tooltip(mut self, t: impl Into<String>) -> Self {
        self.tooltip = Some(t.into());
        self
    }
    pub fn with_sublabel(mut self, s: impl Into<String>) -> Self {
        self.sublabel = Some(s.into());
        self
    }
}

enum PackagePinMap {
    Quad(QuadMap),
    Bga(BgaMap),
}

struct QuadMap {
    total: usize,
    pins_per_side: usize,
    /// `map[idx]` is the GPIO pin at pin-number `idx` (1-based); index 0 is unused.
    map: &'static [Option<Pin>],
    labels: &'static [Option<&'static str>],
}

struct BgaMap {
    /// Row letters (ST skips 'I' per JEDEC).
    rows: &'static [char],
    cols: usize,
    /// Row-major: `map[row_idx * cols + col_idx]`.
    map: &'static [Option<Pin>],
    labels: &'static [Option<&'static str>],
}

macro_rules! g {
    ($port:literal, $num:literal) => { Some(Pin::new($port, $num)) };
}

// LQFP48 (G474C) pin table from datasheet DS12288 §4.2 Figure 6.
// 12 pins per side. Left 1..=12 top->bottom, bottom 13..=24 left->right,
// right 25..=36 bottom->top, top 37..=48 right->left.
#[rustfmt::skip]
const LQFP48_MAP: &[Option<Pin>] = &[
    /*  0 */ None,
    /*  1 */ None, /*  2 */ Some(Pin::new('C', 13)),
    /*  3 */ Some(Pin::new('C', 14)), /*  4 */ Some(Pin::new('C', 15)),
    /*  5 */ Some(Pin::new('F', 0)),  /*  6 */ Some(Pin::new('F', 1)),
    /*  7 */ Some(Pin::new('G', 10)), /*  8 */ Some(Pin::new('A', 0)),
    /*  9 */ Some(Pin::new('A', 1)),  /* 10 */ Some(Pin::new('A', 2)),
    /* 11 */ Some(Pin::new('A', 3)),  /* 12 */ Some(Pin::new('A', 4)),
    /* 13 */ Some(Pin::new('A', 5)),  /* 14 */ Some(Pin::new('A', 6)),
    /* 15 */ Some(Pin::new('A', 7)),  /* 16 */ Some(Pin::new('B', 0)),
    /* 17 */ Some(Pin::new('B', 1)),  /* 18 */ Some(Pin::new('B', 2)),
    /* 19 */ None, /* 20 */ None, /* 21 */ None,
    /* 22 */ Some(Pin::new('B', 10)), /* 23 */ None, /* 24 */ None,
    /* 25 */ Some(Pin::new('B', 11)), /* 26 */ Some(Pin::new('B', 12)),
    /* 27 */ Some(Pin::new('B', 13)), /* 28 */ Some(Pin::new('B', 14)),
    /* 29 */ Some(Pin::new('B', 15)), /* 30 */ Some(Pin::new('A', 8)),
    /* 31 */ Some(Pin::new('A', 9)),  /* 32 */ Some(Pin::new('A', 10)),
    /* 33 */ Some(Pin::new('A', 11)), /* 34 */ Some(Pin::new('A', 12)),
    /* 35 */ None, /* 36 */ None,
    /* 37 */ Some(Pin::new('A', 13)), /* 38 */ Some(Pin::new('A', 14)),
    /* 39 */ Some(Pin::new('A', 15)), /* 40 */ Some(Pin::new('B', 3)),
    /* 41 */ Some(Pin::new('B', 4)),  /* 42 */ Some(Pin::new('B', 5)),
    /* 43 */ Some(Pin::new('B', 6)),  /* 44 */ Some(Pin::new('B', 7)),
    /* 45 */ Some(Pin::new('B', 8)),  /* 46 */ Some(Pin::new('B', 9)),
    /* 47 */ None, /* 48 */ None,
];

#[rustfmt::skip]
const LQFP48_LABELS: &[Option<&'static str>] = &[
    None,
    Some("VBAT"), None, None, None, None, None, Some("NRST"), None, None, None, None, None,
    None, None, None, None, None, None, Some("VSSA"), Some("VREF+"), Some("VDDA"), None, Some("VSS"), Some("VDD"),
    None, None, None, None, None, None, None, None, None, None, Some("VSS"), Some("VDD"),
    None, None, None, None, None, None, None, None, None, None, Some("VSS"), Some("VDD"),
];

// LQFP64 (G474R) pin table from datasheet DS12288 §4.3 Figure 7.
#[rustfmt::skip]
const LQFP64_MAP: &[Option<Pin>] = &[
    /*  0 */ None,
    /*  1 */ None, /*  2 */ Some(Pin::new('C', 13)),
    /*  3 */ Some(Pin::new('C', 14)), /*  4 */ Some(Pin::new('C', 15)),
    /*  5 */ Some(Pin::new('F', 0)),  /*  6 */ Some(Pin::new('F', 1)),
    /*  7 */ Some(Pin::new('G', 10)), /*  8 */ Some(Pin::new('C', 0)),
    /*  9 */ Some(Pin::new('C', 1)),  /* 10 */ Some(Pin::new('C', 2)),
    /* 11 */ Some(Pin::new('C', 3)),  /* 12 */ Some(Pin::new('A', 0)),
    /* 13 */ Some(Pin::new('A', 1)),  /* 14 */ Some(Pin::new('A', 2)),
    /* 15 */ None, /* 16 */ None,
    /* 17 */ Some(Pin::new('A', 3)),  /* 18 */ Some(Pin::new('A', 4)),
    /* 19 */ Some(Pin::new('A', 5)),  /* 20 */ Some(Pin::new('A', 6)),
    /* 21 */ Some(Pin::new('A', 7)),  /* 22 */ Some(Pin::new('C', 4)),
    /* 23 */ Some(Pin::new('C', 5)),  /* 24 */ Some(Pin::new('B', 0)),
    /* 25 */ Some(Pin::new('B', 1)),  /* 26 */ Some(Pin::new('B', 2)),
    /* 27 */ None, /* 28 */ None, /* 29 */ None,
    /* 30 */ Some(Pin::new('B', 10)), /* 31 */ None, /* 32 */ None,
    /* 33 */ Some(Pin::new('B', 11)), /* 34 */ Some(Pin::new('B', 12)),
    /* 35 */ Some(Pin::new('B', 13)), /* 36 */ Some(Pin::new('B', 14)),
    /* 37 */ Some(Pin::new('B', 15)), /* 38 */ Some(Pin::new('C', 6)),
    /* 39 */ Some(Pin::new('C', 7)),  /* 40 */ Some(Pin::new('C', 8)),
    /* 41 */ Some(Pin::new('C', 9)),  /* 42 */ Some(Pin::new('A', 8)),
    /* 43 */ Some(Pin::new('A', 9)),  /* 44 */ Some(Pin::new('A', 10)),
    /* 45 */ Some(Pin::new('A', 11)), /* 46 */ Some(Pin::new('A', 12)),
    /* 47 */ None, /* 48 */ None,
    /* 49 */ Some(Pin::new('A', 13)), /* 50 */ Some(Pin::new('A', 14)),
    /* 51 */ Some(Pin::new('A', 15)), /* 52 */ Some(Pin::new('C', 10)),
    /* 53 */ Some(Pin::new('C', 11)), /* 54 */ Some(Pin::new('C', 12)),
    /* 55 */ Some(Pin::new('D', 2)),  /* 56 */ Some(Pin::new('B', 3)),
    /* 57 */ Some(Pin::new('B', 4)),  /* 58 */ Some(Pin::new('B', 5)),
    /* 59 */ Some(Pin::new('B', 6)),  /* 60 */ Some(Pin::new('B', 7)),
    /* 61 */ Some(Pin::new('B', 8)),  /* 62 */ Some(Pin::new('B', 9)),
    /* 63 */ None, /* 64 */ None,
];

#[rustfmt::skip]
const LQFP64_LABELS: &[Option<&'static str>] = &[
    None,
    Some("VBAT"), None, None, None, None, None, Some("NRST"), None, None, None, None, None, None, None, Some("VSS"), Some("VDD"),
    None, None, None, None, None, None, None, None, None, None, Some("VSSA"), Some("VREF+"), Some("VDDA"), None, Some("VSS"), Some("VDD"),
    None, None, None, None, None, None, None, None, None, None, None, None, None, None, Some("VSS"), Some("VDD"),
    None, None, None, None, None, None, None, None, None, None, None, None, None, None, Some("VSS"), Some("VDD"),
];

// LQFP100 (G474V) pin table from datasheet DS12288 §4.5 Figure 9. 25 per side.
#[rustfmt::skip]
const LQFP100_MAP: &[Option<Pin>] = &[
    /*  0 */ None,
    /*  1 */ Some(Pin::new('E', 2)),  /*  2 */ Some(Pin::new('E', 3)),
    /*  3 */ Some(Pin::new('E', 4)),  /*  4 */ Some(Pin::new('E', 5)),
    /*  5 */ Some(Pin::new('E', 6)),  /*  6 */ None,
    /*  7 */ Some(Pin::new('C', 13)), /*  8 */ Some(Pin::new('C', 14)),
    /*  9 */ Some(Pin::new('C', 15)), /* 10 */ Some(Pin::new('F', 9)),
    /* 11 */ Some(Pin::new('F', 10)), /* 12 */ Some(Pin::new('F', 0)),
    /* 13 */ Some(Pin::new('F', 1)),  /* 14 */ Some(Pin::new('G', 10)),
    /* 15 */ Some(Pin::new('C', 0)),  /* 16 */ Some(Pin::new('C', 1)),
    /* 17 */ Some(Pin::new('C', 2)),  /* 18 */ Some(Pin::new('C', 3)),
    /* 19 */ Some(Pin::new('F', 2)),  /* 20 */ Some(Pin::new('A', 0)),
    /* 21 */ Some(Pin::new('A', 1)),  /* 22 */ Some(Pin::new('A', 2)),
    /* 23 */ None, /* 24 */ None, /* 25 */ Some(Pin::new('A', 3)),
    /* 26 */ Some(Pin::new('A', 4)),  /* 27 */ Some(Pin::new('A', 5)),
    /* 28 */ Some(Pin::new('A', 6)),  /* 29 */ Some(Pin::new('A', 7)),
    /* 30 */ Some(Pin::new('C', 4)),  /* 31 */ Some(Pin::new('C', 5)),
    /* 32 */ Some(Pin::new('B', 0)),  /* 33 */ Some(Pin::new('B', 1)),
    /* 34 */ Some(Pin::new('B', 2)),  /* 35 */ None, /* 36 */ None,
    /* 37 */ None, /* 38 */ Some(Pin::new('E', 7)),
    /* 39 */ Some(Pin::new('E', 8)),  /* 40 */ Some(Pin::new('E', 9)),
    /* 41 */ Some(Pin::new('E', 10)), /* 42 */ Some(Pin::new('E', 11)),
    /* 43 */ Some(Pin::new('E', 12)), /* 44 */ Some(Pin::new('E', 13)),
    /* 45 */ Some(Pin::new('E', 14)), /* 46 */ Some(Pin::new('E', 15)),
    /* 47 */ Some(Pin::new('B', 10)), /* 48 */ None, /* 49 */ None,
    /* 50 */ Some(Pin::new('B', 11)),
    /* 51 */ Some(Pin::new('B', 12)), /* 52 */ Some(Pin::new('B', 13)),
    /* 53 */ Some(Pin::new('B', 14)), /* 54 */ Some(Pin::new('B', 15)),
    /* 55 */ Some(Pin::new('D', 8)),  /* 56 */ Some(Pin::new('D', 9)),
    /* 57 */ Some(Pin::new('D', 10)), /* 58 */ Some(Pin::new('D', 11)),
    /* 59 */ Some(Pin::new('D', 12)), /* 60 */ Some(Pin::new('D', 13)),
    /* 61 */ Some(Pin::new('D', 14)), /* 62 */ Some(Pin::new('D', 15)),
    /* 63 */ None, /* 64 */ None,
    /* 65 */ Some(Pin::new('C', 6)),  /* 66 */ Some(Pin::new('C', 7)),
    /* 67 */ Some(Pin::new('C', 8)),  /* 68 */ Some(Pin::new('C', 9)),
    /* 69 */ Some(Pin::new('A', 8)),  /* 70 */ Some(Pin::new('A', 9)),
    /* 71 */ Some(Pin::new('A', 10)), /* 72 */ Some(Pin::new('A', 11)),
    /* 73 */ Some(Pin::new('A', 12)), /* 74 */ None, /* 75 */ None,
    /* 76 */ Some(Pin::new('A', 13)), /* 77 */ Some(Pin::new('A', 14)),
    /* 78 */ Some(Pin::new('A', 15)), /* 79 */ Some(Pin::new('C', 10)),
    /* 80 */ Some(Pin::new('C', 11)), /* 81 */ Some(Pin::new('C', 12)),
    /* 82 */ Some(Pin::new('D', 0)),  /* 83 */ Some(Pin::new('D', 1)),
    /* 84 */ Some(Pin::new('D', 2)),  /* 85 */ Some(Pin::new('D', 3)),
    /* 86 */ Some(Pin::new('D', 4)),  /* 87 */ Some(Pin::new('D', 5)),
    /* 88 */ Some(Pin::new('D', 6)),  /* 89 */ Some(Pin::new('D', 7)),
    /* 90 */ Some(Pin::new('B', 3)),  /* 91 */ Some(Pin::new('B', 4)),
    /* 92 */ Some(Pin::new('B', 5)),  /* 93 */ Some(Pin::new('B', 6)),
    /* 94 */ Some(Pin::new('B', 7)),  /* 95 */ Some(Pin::new('B', 8)),
    /* 96 */ Some(Pin::new('B', 9)),  /* 97 */ Some(Pin::new('E', 0)),
    /* 98 */ Some(Pin::new('E', 1)),  /* 99 */ None, /* 100 */ None,
];

#[rustfmt::skip]
const LQFP100_LABELS: &[Option<&'static str>] = &[
    None,
    // pins 1..=25 (left)
    None, None, None, None, None, Some("VBAT"), None, None, None, None, None, None, None, Some("NRST"), None, None, None, None, None, None, None, None, Some("VSS"), Some("VDD"), None,
    // pins 26..=50 (bottom). VSSA/VREF+/VDDA at pins 35/36/37, VSS/VDD at 48/49.
    None, None, None, None, None, None, None, None, None, Some("VSSA"), Some("VREF+"), Some("VDDA"), None, None, None, None, None, None, None, None, None, None, Some("VSS"), Some("VDD"), None,
    // pins 51..=75 (right)
    None, None, None, None, None, None, None, None, None, None, None, None, Some("VSS"), Some("VDD"), None, None, None, None, None, None, None, None, None, Some("VSS"), Some("VDD"),
    // pins 76..=100 (top)
    None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, Some("VSS"), Some("VDD"),
];

// WLCSP81 (G474M) from datasheet DS12288 §4.7 Figure 11.
// Row-major, 9 rows × 9 cols. Rows: A B C D E F G H J.
#[rustfmt::skip]
const WLCSP81_MAP: &[Option<Pin>] = &[
    // A
    None, g!('A',15), g!('C',12), g!('D',1), g!('B',3), g!('B',5), g!('B',9), None, None,
    // B
    None, g!('A',13), g!('C',10), g!('D',0), g!('D',2), g!('B',6), g!('B',8), None, None,
    // C
    g!('A',12), g!('A',11), g!('A',14), g!('C',11), g!('C',8), g!('B',4), g!('B',7), g!('C',1), g!('C',14),
    // D
    g!('A',8), g!('C',9), g!('A',10), g!('A',9), g!('C',7), g!('A',4), g!('A',0), g!('G',10), g!('C',15),
    // E
    None, g!('D',11), g!('C',6), g!('B',15), g!('E',12), g!('C',4), g!('A',1), g!('C',0), g!('F',0),
    // F
    None, g!('D',10), g!('D',9), g!('E',15), g!('E',9), g!('B',0), g!('A',5), g!('C',2), g!('F',1),
    // G
    g!('D',8), g!('B',14), g!('B',12), g!('E',13), g!('E',8), g!('B',1), g!('A',6), g!('A',2), g!('C',3),
    // H
    g!('B',13), g!('B',11), g!('B',10), g!('E',11), g!('E',7), None, g!('C',5), g!('A',3), None,
    // J
    None, None, g!('E',14), g!('E',10), None, None, g!('B',2), g!('A',7), None,
];
#[rustfmt::skip]
const WLCSP81_LABELS: &[Option<&'static str>] = &[
    Some("VDD"), None, None, None, None, None, None, Some("VSS"), Some("VDD"),
    Some("VSS"), None, None, None, None, None, None, Some("PC13"), Some("VBAT"),
    None, None, None, None, None, None, None, None, None,
    None, None, None, None, None, None, None, Some("NRST"), None,
    Some("VDD"), None, None, None, None, None, None, None, None,
    Some("VSS"), None, None, None, None, None, None, None, None,
    None, None, None, None, None, None, None, None, None,
    None, None, None, None, None, Some("VSSA"), None, None, Some("VSS"),
    Some("VDD"), Some("VSS"), None, None, Some("VDDA"), Some("VREF+"), None, None, Some("VDD"),
];

// TFBGA100 (G474P) from datasheet DS12288 §4.8 Figure 12.
// Row-major, 10 rows × 10 cols. Rows: A B C D E F G H J K.
#[rustfmt::skip]
const TFBGA100_MAP: &[Option<Pin>] = &[
    // A
    g!('E',4), g!('B',9), g!('B',8), g!('B',6), g!('B',3), g!('D',6), g!('D',5), g!('D',4), g!('D',1), g!('C',12),
    // B
    g!('E',5), g!('E',3), g!('E',1), g!('B',7), g!('B',5), g!('D',7), g!('D',2), g!('D',0), g!('A',15), g!('A',14),
    // C
    g!('C',14), g!('E',6), g!('E',2), g!('E',0), g!('B',4), g!('D',3), g!('C',11), g!('C',10), g!('A',12), g!('A',11),
    // D
    g!('C',15), None, None, None, None, None, None, g!('A',13), g!('A',10), g!('A',9),
    // E
    g!('F',0), g!('F',1), g!('F',9), g!('F',10), None, None, None, g!('C',8), g!('C',9), g!('A',8),
    // F
    g!('C',2), g!('C',0), g!('G',10), g!('C',1), None, None, None, g!('D',14), g!('C',6), g!('C',7),
    // G
    g!('C',3), g!('A',1), g!('F',2), g!('A',0), g!('E',7), g!('E',12), g!('D',10), g!('D',9), g!('D',13), g!('D',15),
    // H
    g!('A',2), g!('A',4), g!('A',3), g!('B',0), g!('E',8), g!('E',9), g!('E',15), g!('B',11), g!('B',14), g!('D',11),
    // J
    g!('A',5), g!('A',6), g!('C',5), g!('B',2), None, g!('E',11), g!('E',14), g!('B',10), g!('B',13), g!('D',12),
    // K
    g!('A',7), g!('C',4), g!('B',1), None, None, g!('E',10), g!('E',13), g!('B',12), g!('B',15), g!('D',8),
];
#[rustfmt::skip]
const TFBGA100_LABELS: &[Option<&'static str>] = &[
    None, None, None, None, None, None, None, None, None, None,
    None, None, None, None, None, None, None, None, None, None,
    None, None, None, None, None, None, None, None, None, None,
    None, Some("VSS"), Some("VBAT"), Some("PC13"), Some("VDD"), Some("VSS"), Some("VDD"), None, None, None,
    None, None, None, None, Some("VSS"), Some("VSS"), Some("VSS"), None, None, None,
    None, None, Some("NRST"), None, Some("VDD"), Some("VSS"), Some("VDD"), None, None, None,
    None, None, None, None, None, None, None, None, None, None,
    None, None, None, None, None, None, None, None, None, None,
    None, None, None, None, Some("VDDA"), None, None, None, None, None,
    None, None, None, Some("VSSA"), Some("VREF+"), None, None, None, None, None,
];

// UFBGA121 (G474Q) from datasheet DS12288 §4.9 Figure 13.
// Row-major, 11 rows × 11 cols. Rows: A B C D E F G H J K L.
#[rustfmt::skip]
const UFBGA121_MAP: &[Option<Pin>] = &[
    // A
    g!('E',4), g!('E',2), None, g!('B',9), g!('B',6), g!('B',3), g!('D',4), None, g!('D',1), g!('A',15), g!('F',6),
    // B
    g!('E',5), g!('E',3), None, g!('E',0), g!('B',5), g!('D',7), g!('D',3), None, g!('D',0), g!('A',14), g!('A',13),
    // C
    None, None, g!('E',6), g!('E',1), g!('B',7), g!('B',4), g!('D',2), g!('C',11), g!('C',10), None, None,
    // D
    g!('C',14), g!('C',15), g!('F',3), g!('F',4), g!('B',8), g!('D',6), g!('C',12), g!('A',9), g!('A',10), g!('A',12), g!('A',11),
    // E
    None, None, g!('F',5), g!('F',7), g!('F',8), g!('D',5), g!('A',8), g!('C',9), g!('C',8), g!('G',4), g!('G',3),
    // F
    g!('F',0), g!('F',1), g!('F',9), g!('F',10), g!('G',10), g!('D',15), g!('G',2), g!('G',1), g!('G',0), g!('C',6), g!('C',7),
    // G
    g!('C',1), g!('C',0), g!('C',2), g!('A',0), g!('B',1), g!('F',15), g!('D',11), g!('D',12), g!('D',13), g!('D',14), None,
    // H
    g!('C',3), g!('F',2), g!('A',1), g!('C',5), g!('F',12), g!('F',14), g!('E',10), g!('B',15), g!('D',8), g!('D',9), g!('D',10),
    // J
    None, None, g!('A',2), g!('B',0), g!('F',11), g!('F',13), g!('E',9), g!('E',13), g!('B',12), g!('B',14), g!('B',13),
    // K
    g!('A',3), g!('A',5), g!('A',7), g!('B',2), None, None, g!('E',8), g!('E',12), g!('E',14), None, None,
    // L
    g!('A',4), g!('A',6), g!('C',4), None, None, None, g!('E',7), g!('E',11), g!('E',15), g!('B',10), g!('B',11),
];
#[rustfmt::skip]
const UFBGA121_LABELS: &[Option<&'static str>] = &[
    None, None, Some("VDD"), None, None, None, None, Some("VDD"), None, None, None,
    None, None, Some("VSS"), None, None, None, None, Some("VSS"), None, None, None,
    Some("PC13"), Some("VBAT"), None, None, None, None, None, None, None, Some("VSS"), Some("VDD"),
    None, None, None, None, None, None, None, None, None, None, None,
    Some("VDD"), Some("VSS"), None, None, None, None, None, None, None, None, None,
    None, None, None, None, Some("NRST"), None, None, None, None, None, None,
    None, None, None, None, None, None, None, None, None, None, Some("VDD"),
    None, None, None, None, None, None, None, None, None, None, None,
    Some("VDD"), Some("VSS"), None, None, None, None, None, None, None, None, None,
    None, None, None, None, Some("VSSA"), Some("VSS"), None, None, None, Some("VSS"), Some("VDD"),
    None, None, None, Some("VREF+"), Some("VDDA"), Some("VDD"), None, None, None, None, None,
];

fn map_for(variant: ChipVariant) -> Option<PackagePinMap> {
    match variant {
        ChipVariant::G474C => Some(PackagePinMap::Quad(QuadMap {
            total: 48, pins_per_side: 12,
            map: LQFP48_MAP, labels: LQFP48_LABELS,
        })),
        ChipVariant::G474R => Some(PackagePinMap::Quad(QuadMap {
            total: 64, pins_per_side: 16,
            map: LQFP64_MAP, labels: LQFP64_LABELS,
        })),
        ChipVariant::G474V => Some(PackagePinMap::Quad(QuadMap {
            total: 100, pins_per_side: 25,
            map: LQFP100_MAP, labels: LQFP100_LABELS,
        })),
        ChipVariant::G474M => Some(PackagePinMap::Bga(BgaMap {
            rows: &['A','B','C','D','E','F','G','H','J'], cols: 9,
            map: WLCSP81_MAP, labels: WLCSP81_LABELS,
        })),
        ChipVariant::G474P => Some(PackagePinMap::Bga(BgaMap {
            rows: &['A','B','C','D','E','F','G','H','J','K'], cols: 10,
            map: TFBGA100_MAP, labels: TFBGA100_LABELS,
        })),
        ChipVariant::G474Q => Some(PackagePinMap::Bga(BgaMap {
            rows: &['A','B','C','D','E','F','G','H','J','K','L'], cols: 11,
            map: UFBGA121_MAP, labels: UFBGA121_LABELS,
        })),
    }
}

pub const C_BG: Color32 = Color32::from_gray(24);
pub const C_PKG: Color32 = Color32::from_gray(42);
pub const C_PIN: Color32 = Color32::from_gray(80);
pub const C_PIN_POWER: Color32 = Color32::from_gray(55);
pub const C_PIN_ASSIGNED: Color32 = Color32::from_rgb(60, 130, 80);
pub const C_PIN_HELD_SOURCE: Color32 = Color32::from_rgb(220, 170, 60);
pub const C_PIN_DIRECT: Color32 = Color32::from_rgb(80, 180, 100);
pub const C_PIN_SEMANTIC: Color32 = Color32::from_rgb(80, 170, 220);
pub const C_PIN_CASCADE1: Color32 = Color32::from_rgb(220, 200, 80);
pub const C_PIN_CASCADE_MULTI: Color32 = Color32::from_rgb(220, 140, 70);
pub const C_PIN_BLOCKED: Color32 = Color32::from_rgb(130, 60, 60);
pub const C_PIN_DIMMED: Color32 = Color32::from_gray(36);
pub const C_TEXT: Color32 = Color32::from_gray(220);
pub const C_TEXT_DIM: Color32 = Color32::from_gray(130);

pub fn show(
    ui: &mut egui::Ui,
    variant: ChipVariant,
    paints: &HashMap<Pin, PinPaint>,
) -> Option<Action> {
    let Some(pkg_map) = map_for(variant) else {
        ui.label(
            egui::RichText::new(format!(
                "(package view for {} not implemented yet - only LQFP64 is supported)",
                variant.display_label()
            ))
            .weak(),
        );
        return None;
    };

    let avail = ui.available_size_before_wrap();
    let side = avail.x.min(avail.y).max(300.0) - 20.0;
    let (rect, response) = ui.allocate_exact_size(
        Vec2::new(avail.x, side + 20.0),
        Sense::click(),
    );
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, C_BG);

    let result = match pkg_map {
        PackagePinMap::Quad(q) => draw_quad(&painter, &response, rect, side, variant, &q, paints),
        PackagePinMap::Bga(b) => draw_bga(&painter, &response, rect, side, variant, &b, paints),
    };

    if let Some((_pos, tt)) = result.hover_tooltip {
        egui::Tooltip::always_open(
            ui.ctx().clone(),
            ui.layer_id(),
            egui::Id::new("pkg_pin_tooltip"),
            egui::PopupAnchor::Pointer,
        )
        .show(|ui| {
            ui.label(tt);
        });
    }

    if let Some(p) = result.hit {
        return Some(Action::Click(p));
    }
    if response.clicked() {
        return Some(Action::ClickEmpty);
    }
    None
}

struct DrawResult {
    hit: Option<Pin>,
    hover_tooltip: Option<(Pos2, String)>,
}

fn draw_quad(
    painter: &egui::Painter,
    response: &egui::Response,
    rect: Rect,
    side: f32,
    variant: ChipVariant,
    pkg_map: &QuadMap,
    paints: &HashMap<Pin, PinPaint>,
) -> DrawResult {
    let cx = rect.center().x;
    let cy = rect.top() + (side + 20.0) / 2.0;
    let pkg_size = side * 0.55;
    let pkg = Rect::from_center_size(Pos2::new(cx, cy), Vec2::new(pkg_size, pkg_size));
    painter.rect_filled(pkg, 6.0, C_PKG);
    painter.text(
        pkg.center() - Vec2::new(0.0, 10.0),
        egui::Align2::CENTER_CENTER,
        variant.part_family(),
        FontId::proportional(18.0),
        C_TEXT,
    );
    painter.text(
        pkg.center() + Vec2::new(0.0, 10.0),
        egui::Align2::CENTER_CENTER,
        variant.package(),
        FontId::monospace(11.0),
        C_TEXT_DIM,
    );
    painter.circle_filled(
        Pos2::new(pkg.left() + 10.0, pkg.top() + 10.0),
        3.0,
        Color32::from_gray(180),
    );

    let step = pkg_size / pkg_map.pins_per_side as f32;
    let pin_w = (step * 0.45).clamp(3.0, 8.0);
    let pin_l = (step * 1.0).clamp(8.0, 14.0);
    let mut hit: Option<Pin> = None;
    let mut hover_tooltip: Option<(Pos2, String)> = None;
    let hover_pos = response.hover_pos();

    for idx in 1..=pkg_map.total {
        let (pin_rect, click_rect, label_anchor, label_dir) =
            pin_geometry(idx, &pkg, pkg_map.pins_per_side, pin_w, pin_l);
        let pin_opt = pkg_map.map.get(idx).copied().flatten();
        let power_label = pkg_map.labels.get(idx).copied().flatten();

        let paint = pin_opt.and_then(|p| paints.get(&p).cloned()).unwrap_or_else(|| {
            if pin_opt.is_some() {
                PinPaint::free()
            } else {
                PinPaint {
                    fill: C_PIN_POWER,
                    border: None,
                    sublabel: power_label.map(|s| s.to_string()),
                    tooltip: None,
                    interactive: false,
                }
            }
        });

        painter.rect_filled(pin_rect, 1.0, paint.fill);
        if let Some(stroke) = paint.border {
            painter.rect_stroke(pin_rect, 1.0, stroke, egui::epaint::StrokeKind::Middle);
        }

        let small = FontId::monospace(9.0);
        painter.text(
            inside_of(pin_rect, idx, pkg_map.pins_per_side, 2.0),
            inside_align(idx, pkg_map.pins_per_side),
            idx.to_string(),
            small.clone(),
            C_TEXT_DIM,
        );

        let outside_text = match pin_opt {
            Some(p) => match &paint.sublabel {
                Some(s) => format!("{}  {}", p.name(), s),
                None => p.name(),
            },
            None => paint.sublabel.clone().unwrap_or_default(),
        };
        if !outside_text.is_empty() {
            let text_color = if matches!(paint.fill, c if c == C_PIN_DIMMED) {
                C_TEXT_DIM
            } else {
                C_TEXT
            };
            painter.text(label_anchor, label_dir, outside_text, small, text_color);
        }

        if let (Some(hp), Some(tt)) = (hover_pos, &paint.tooltip) {
            if click_rect.contains(hp) {
                hover_tooltip = Some((hp, tt.clone()));
            }
        }

        if paint.interactive && response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                if click_rect.contains(pos) {
                    if let Some(p) = pin_opt {
                        hit = Some(p);
                    }
                }
            }
        }
    }
    DrawResult { hit, hover_tooltip }
}

fn draw_bga(
    painter: &egui::Painter,
    response: &egui::Response,
    rect: Rect,
    side: f32,
    variant: ChipVariant,
    pkg_map: &BgaMap,
    paints: &HashMap<Pin, PinPaint>,
) -> DrawResult {
    let cols = pkg_map.cols;
    let rows = pkg_map.rows.len();
    // BGA grid: reserve a margin for row/column headers and labels.
    let pkg_size = (side * 0.85).min(side - 40.0);
    let cx = rect.center().x;
    let cy = rect.top() + (side + 20.0) / 2.0;
    let pkg = Rect::from_center_size(Pos2::new(cx, cy), Vec2::new(pkg_size, pkg_size));
    painter.rect_filled(pkg, 6.0, C_PKG);
    // Part label in the upper-left area, outside the ball grid.
    painter.text(
        Pos2::new(pkg.left() + 8.0, pkg.top() - 20.0),
        egui::Align2::LEFT_BOTTOM,
        format!("{}  {}", variant.part_family(), variant.package()),
        FontId::proportional(13.0),
        C_TEXT,
    );
    // Pin-1 dimple at A1 corner (top-left).
    painter.circle_filled(
        Pos2::new(pkg.left() + 10.0, pkg.top() + 10.0),
        3.0,
        Color32::from_gray(180),
    );

    // Ball geometry: leave a small inset on all sides for column/row labels.
    let inset = 24.0f32;
    let grid_rect = Rect::from_min_max(
        Pos2::new(pkg.left() + inset, pkg.top() + inset),
        Pos2::new(pkg.right() - inset, pkg.bottom() - inset),
    );
    let ball_step_x = grid_rect.width() / cols as f32;
    let ball_step_y = grid_rect.height() / rows as f32;
    let ball_r = (ball_step_x.min(ball_step_y) * 0.35).max(3.0);

    let small = FontId::monospace(9.0);
    // Column headers (1..=cols) along the top.
    for c in 0..cols {
        let x = grid_rect.left() + ball_step_x * (c as f32 + 0.5);
        painter.text(
            Pos2::new(x, grid_rect.top() - 4.0),
            egui::Align2::CENTER_BOTTOM,
            (c + 1).to_string(),
            small.clone(),
            C_TEXT_DIM,
        );
    }
    // Row headers along the left.
    for (r, letter) in pkg_map.rows.iter().enumerate() {
        let y = grid_rect.top() + ball_step_y * (r as f32 + 0.5);
        painter.text(
            Pos2::new(grid_rect.left() - 4.0, y),
            egui::Align2::RIGHT_CENTER,
            letter.to_string(),
            small.clone(),
            C_TEXT_DIM,
        );
    }

    let hover_pos = response.hover_pos();
    let mut hit: Option<Pin> = None;
    let mut hover_tooltip: Option<(Pos2, String)> = None;

    for r in 0..rows {
        for c in 0..cols {
            let idx = r * cols + c;
            let pin_opt = pkg_map.map.get(idx).copied().flatten();
            let power_label = pkg_map.labels.get(idx).copied().flatten();
            let cx = grid_rect.left() + ball_step_x * (c as f32 + 0.5);
            let cy = grid_rect.top() + ball_step_y * (r as f32 + 0.5);
            let ball_center = Pos2::new(cx, cy);

            let paint = pin_opt.and_then(|p| paints.get(&p).cloned()).unwrap_or_else(|| {
                if pin_opt.is_some() {
                    PinPaint::free()
                } else {
                    PinPaint {
                        fill: C_PIN_POWER,
                        border: None,
                        sublabel: power_label.map(|s| s.to_string()),
                        tooltip: None,
                        interactive: false,
                    }
                }
            });
            painter.circle_filled(ball_center, ball_r, paint.fill);
            if let Some(stroke) = paint.border {
                painter.circle_stroke(ball_center, ball_r, stroke);
            }

            // Small label under each ball: GPIO name if assignable;
            // the supply name for power balls; blank otherwise.
            let label_text = match (pin_opt, &paint.sublabel) {
                (Some(p), Some(s)) => format!("{} {}", p.name(), s),
                (Some(p), None) => p.name(),
                (None, Some(s)) => s.clone(),
                (None, None) => String::new(),
            };
            if !label_text.is_empty() {
                painter.text(
                    Pos2::new(cx, cy + ball_r + 1.0),
                    egui::Align2::CENTER_TOP,
                    label_text,
                    FontId::monospace(8.0),
                    if matches!(paint.fill, c if c == C_PIN_DIMMED) { C_TEXT_DIM } else { C_TEXT },
                );
            }

            let click_rect = Rect::from_center_size(
                ball_center,
                Vec2::new(ball_step_x, ball_step_y),
            );
            if let (Some(hp), Some(tt)) = (hover_pos, &paint.tooltip) {
                if click_rect.contains(hp) {
                    hover_tooltip = Some((hp, tt.clone()));
                }
            }
            if paint.interactive && response.clicked() {
                if let Some(pos) = response.interact_pointer_pos() {
                    if click_rect.contains(pos) {
                        if let Some(p) = pin_opt {
                            hit = Some(p);
                        }
                    }
                }
            }
        }
    }
    DrawResult { hit, hover_tooltip }
}

fn pin_geometry(
    n: usize,
    pkg: &Rect,
    per_side: usize,
    pin_w: f32,
    pin_l: f32,
) -> (Rect, Rect, Pos2, egui::Align2) {
    let side_idx = (n - 1) / per_side;
    let pos_in_side = (n - 1) % per_side;
    let step = pkg.width() / per_side as f32;
    let center_offset = step * (pos_in_side as f32 + 0.5);
    let label_pad = pin_l + 6.0;
    match side_idx {
        0 => {
            let cy = pkg.top() + center_offset;
            let pin = Rect::from_center_size(
                Pos2::new(pkg.left() - pin_l / 2.0, cy),
                Vec2::new(pin_l, pin_w),
            );
            let click = Rect::from_min_max(
                Pos2::new(pkg.left() - 200.0, cy - step / 2.0),
                Pos2::new(pkg.left(), cy + step / 2.0),
            );
            let anchor = Pos2::new(pkg.left() - label_pad, cy);
            (pin, click, anchor, egui::Align2::RIGHT_CENTER)
        }
        1 => {
            let cx = pkg.left() + center_offset;
            let pin = Rect::from_center_size(
                Pos2::new(cx, pkg.bottom() + pin_l / 2.0),
                Vec2::new(pin_w, pin_l),
            );
            let click = Rect::from_min_max(
                Pos2::new(cx - step / 2.0, pkg.bottom()),
                Pos2::new(cx + step / 2.0, pkg.bottom() + 200.0),
            );
            let anchor = Pos2::new(cx, pkg.bottom() + label_pad);
            (pin, click, anchor, egui::Align2::CENTER_TOP)
        }
        2 => {
            let cy = pkg.bottom() - center_offset;
            let pin = Rect::from_center_size(
                Pos2::new(pkg.right() + pin_l / 2.0, cy),
                Vec2::new(pin_l, pin_w),
            );
            let click = Rect::from_min_max(
                Pos2::new(pkg.right(), cy - step / 2.0),
                Pos2::new(pkg.right() + 200.0, cy + step / 2.0),
            );
            let anchor = Pos2::new(pkg.right() + label_pad, cy);
            (pin, click, anchor, egui::Align2::LEFT_CENTER)
        }
        _ => {
            let cx = pkg.right() - center_offset;
            let pin = Rect::from_center_size(
                Pos2::new(cx, pkg.top() - pin_l / 2.0),
                Vec2::new(pin_w, pin_l),
            );
            let click = Rect::from_min_max(
                Pos2::new(cx - step / 2.0, pkg.top() - 200.0),
                Pos2::new(cx + step / 2.0, pkg.top()),
            );
            let anchor = Pos2::new(cx, pkg.top() - label_pad);
            (pin, click, anchor, egui::Align2::CENTER_BOTTOM)
        }
    }
}

fn inside_of(pin_rect: Rect, n: usize, per_side: usize, pad: f32) -> Pos2 {
    let side_idx = (n - 1) / per_side;
    match side_idx {
        0 => Pos2::new(pin_rect.right() + pad, pin_rect.center().y),
        1 => Pos2::new(pin_rect.center().x, pin_rect.top() - pad),
        2 => Pos2::new(pin_rect.left() - pad, pin_rect.center().y),
        _ => Pos2::new(pin_rect.center().x, pin_rect.bottom() + pad),
    }
}

fn inside_align(n: usize, per_side: usize) -> egui::Align2 {
    let side_idx = (n - 1) / per_side;
    match side_idx {
        0 => egui::Align2::LEFT_CENTER,
        1 => egui::Align2::CENTER_BOTTOM,
        2 => egui::Align2::RIGHT_CENTER,
        _ => egui::Align2::CENTER_TOP,
    }
}
