//! Multi-MCU framework. Today's tool is G474-shaped throughout (typed enums
//! in `g474.rs`, AF tables in `pinout.rs`); this module adds an MCU-family
//! selector on top so that other MCUs (currently STM32H523, planned) can be
//! selected and their support filled in incrementally.
//!
//! Slice 1 (this file): `Mcu` enum + a thin descriptor; H523 shows a stub.
//! Slice 2 (later): G474 data migrates into a `McuDescriptor`-shaped table
//!  and consumers iterate the descriptor instead of the typed enums.

use crate::g474::PeripheralKind;
use crate::pinout::ChipVariant;

// ---------- MCU family selector ----------

/// Top-level MCU family. Picks which set of peripherals/pinouts/tabs the app
/// shows. Below this, G474 retains its own `ChipVariant` (pinout class).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Mcu {
    G474,
    H523,
}

impl Default for Mcu {
    fn default() -> Self { Self::G474 }
}

impl Mcu {
    pub const ALL: &'static [Mcu] = &[Mcu::G474, Mcu::H523];

    pub fn label(self) -> &'static str {
        match self {
            Self::G474 => "STM32G474",
            Self::H523 => "STM32H523",
        }
    }

    pub fn descriptor(self) -> &'static McuDescriptor {
        match self { Self::G474 => &G474, Self::H523 => &H523 }
    }

    /// True for MCUs whose tool support is implemented (peripherals, pinouts,
    /// solver). H523 is currently a stub: selectable, but planning views are
    /// not yet populated.
    pub fn is_implemented(self) -> bool {
        match self { Self::G474 => true, Self::H523 => false }
    }

    /// Default G474 pinout class. Only meaningful when `self == G474` —
    /// callers must check.
    pub fn default_g474_variant(self) -> ChipVariant {
        ChipVariant::G474R
    }
}

// ---------- Descriptor (sketch — populated progressively) ----------

/// Uniform peripheral instance ID. For now this exists alongside the typed
/// enums in `g474.rs` and is mostly used by the H523 stub. Migration into
/// G474-side code is staged in slice 2.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub struct PeripheralId {
    pub kind: PeripheralKind,
    /// 1-indexed instance. Lettered HRTIM sub-timers map A=1..F=6.
    pub instance: u8,
    /// Channel within the instance, or 0 if N/A.
    pub channel: u8,
}

/// Peripheral-to-peripheral connectivity edge (DAC→COMP today; future MCUs
/// may add more). HRTIM-internal links (COMP→EEV, COMP→FLT, crossbar→trig,
/// trig→ADC) live inside `HrtimFabric` because they target HRTIM-internal
/// slots, not other peripheral instances.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct PeripheralEdge {
    pub from: PeripheralId,
    pub to: PeripheralId,
}

/// HRTIM crossbar-fabric capability. `None` on MCUs without HRTIM (H523).
/// Members are placeholders for now — the existing G474 HRTIM tables stay
/// in `g474.rs` until slice 2 moves them in here.
pub struct HrtimFabric {
    pub eev_count: u8,
    pub flt_count: u8,
    pub adc_trigger_count: u8,
}

pub struct McuDescriptor {
    pub name: &'static str,
    /// Every peripheral instance present on this MCU. Sparse today — gets
    /// populated as consumers migrate.
    pub peripherals: &'static [PeripheralId],
    pub edges: &'static [PeripheralEdge],
    pub hrtim: Option<&'static HrtimFabric>,
}

// ---------- G474 descriptor ----------

const G474_HRTIM: HrtimFabric = HrtimFabric {
    eev_count: 10,
    flt_count: 6,
    adc_trigger_count: 10,
};

pub const G474: McuDescriptor = McuDescriptor {
    name: "STM32G474",
    // Empty for now — G474 consumers still read `g474::*` directly. Will be
    // populated as `pinout.rs`, `picker.rs`, etc. migrate to descriptor reads.
    peripherals: &[],
    edges: &[],
    hrtim: Some(&G474_HRTIM),
};

// ---------- H523 descriptor (stub) ----------

pub const H523: McuDescriptor = McuDescriptor {
    name: "STM32H523",
    peripherals: &[],
    edges: &[],
    hrtim: None,
};
