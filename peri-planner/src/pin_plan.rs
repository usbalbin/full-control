//! The unified, serde-ready **PinPlan** — the single representation that the
//! three family design models (`Design`/G474, `H523Design`/H5+C5A3,
//! `C531Design`/C5) lower INTO, and that the downstream consumers (embassy/RTIC
//! firmware codegen, KiCad drift-check) read FROM.
//!
//! Design rationale, lowering rules, and consumer contracts live in
//! `docs/firmware-codegen-design.md` §8 (produced by a judge-panel → synthesis →
//! adversarial red-team). The short version:
//!
//! * **Derived, not edited.** PinPlan is regenerated one-way from the family
//!   structs on save; it is never round-tripped, so being lossy on *intent*
//!   (requirement labels, leg flags) is fine.
//! * **Generic spine.** A `Placement` is a pure `(peripheral, role) → pin`
//!   triple over the already-regeneration-stable [`OwnedSignal`]/[`PinId`]
//!   strings, so the `placements` layer scales to **every** MCU with no new
//!   code. `target.family` is an open string, never a closed enum.
//! * **Fabric is an opt-in extra.** The analog crossbar (`routes` +
//!   `slot_claims`) is populated only for the few fabric-bearing families
//!   (G4 HRTIM, C5 OCP); it is empty for everything else.
//! * **Structural facts only.** No field can hold a behavioral value (baud,
//!   dead-time ns, fault filter, PCM threshold, phase shift) — those are
//!   surfaced by codegen as named *holes*, never stored here.

use crate::mcu_pinout::{OwnedSignal, PinId};
use serde::{Deserialize, Serialize};

pub const PIN_PLAN_FORMAT_VERSION: u32 = 1;

/// The single source of truth for one resolved design on one chip.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PinPlan {
    pub format_version: u32,
    /// Chip identity. Pins/routes are package-relative; codegen, re-validation,
    /// and the KiCad logical→pad map all need it.
    pub target: Target,
    /// LAYER 1 — every pin-bearing signal landed on a physical pin. The generic
    /// spine; sorted by `(peripheral, role)` for clean diffs.
    pub placements: Vec<Placement>,
    /// LAYER 2 — the analog fabric as an explicit edge graph. Empty for
    /// families without a crossbar.
    #[serde(default)]
    pub routes: Vec<RouteEdge>,
    /// HRTIM compare/capture register reservations (Cr2/Cr4/Cpt2,
    /// MasterCompare…). Re-derivable from the family model's `consumed()`, but
    /// stored so codegen picks the right compare register without re-deriving.
    #[serde(default)]
    pub slot_claims: Vec<SlotClaim>,
    /// Durable user-intent locks that must survive a re-lower. NOT the transient
    /// conflict set — that is always recomputed.
    #[serde(default)]
    pub locks: Vec<Lock>,
}

/// Stable chip identity (open strings, no closed enums — a new line is data).
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Target {
    /// Package/part key, e.g. `"G474RE"`, `"H523RE"`, `"C531R"`, `"C5A3Z"`.
    pub package: String,
    /// Coarse family tag, e.g. `"G4"`, `"H5"`, `"C5"`. An **open string** so a
    /// reader can pick the fabric topology / codegen tier table without parsing
    /// `package`, yet adding a new line (G0, H7, U5, …) needs no code change.
    pub family: String,
}

// ---------------------------------------------------------------------------
// LAYER 1 — Placement: one pin-bearing signal on one physical pin.
// ---------------------------------------------------------------------------
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Placement {
    /// Durable logical identity, e.g. `("USART2","TX")`, `("TIM1","CH1")`,
    /// `("HRTIM1","CHA1")`, `("ADC1","INP3")`. The KiCad `pinfunction` half AND
    /// the codegen instance+role.
    pub signal: OwnedSignal,
    /// Resolved logical pin → `"PA8"`. `None` is a deliberate, persisted state:
    /// the role is declared but not yet placed.
    pub pin: Option<PinId>,
    /// How this pin choice came to be — lets the drift-check tell a user-fixed
    /// pin from arbitrary-but-valid solver output.
    pub origin: PinOrigin,
    /// Alternate-function number, if known. `None` = analog/dedicated pin (DAC
    /// out, ADC in, COMP/OPAMP I/O). Informational: embassy infers AF from
    /// sealed per-pin traits; useful for raw-GPIO scaffolds.
    #[serde(default)]
    pub af: Option<u8>,
    /// Constructor-shaping structural facts (which pins/args exist), never
    /// behavioral values.
    #[serde(default)]
    pub role_kind: RoleKind,
    /// Resolved DMA channel for this signal, if it uses DMA. Concrete
    /// `(controller, channel)`, not a count. `None` = blocking / not resolved.
    #[serde(default)]
    pub dma: Option<DmaBinding>,
    /// IRQ lines this instance needs for `bind_interrupts!` (metapac names).
    #[serde(default)]
    pub irqs: Vec<String>,
    /// Physical package pad number for KiCad pad matching. Filled only once a
    /// `(logical pin → pad)` table exists for `target`; until then drift-check
    /// correlates on pinfunction + logical pin.
    #[serde(default)]
    pub package_pin: Option<u16>,
    /// Schematic net name for KiCad connectivity validation. Unmodeled today —
    /// reserved so the field exists when net naming lands.
    #[serde(default)]
    pub net: Option<String>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PinOrigin {
    /// User explicitly pinned it (a `Lock`, or an `H523Design` `PinLock`), or a
    /// declared-but-unplaced user intent. Stable.
    Locked,
    /// Solver/lowering picked the first valid candidate. May renumber on
    /// re-lower; drift-check treats moves as expected, not as drift.
    Solver,
    /// Only one package pin can carry this signal — forced, fully stable.
    Forced,
}

/// Constructor-shaping structural facts. Defaults to `Gpio` (a bare pin role,
/// no extra args). Behavioral params (baud, ns, freq, threshold) are HOLES.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum RoleKind {
    /// A bare pin role, no constructor-shaping args.
    #[default]
    Gpio,
    /// Comms structural flags affecting which pins/args bind.
    Comms { flow_control: bool, synchronous: bool, nss: bool, smba: bool },
    /// A timer/HRTIM PWM output. `complementary` ⇒ a CHxN partner placement
    /// exists and the dead-time GENERATOR is on (the ns is a hole). `etr` ⇒
    /// external-trigger pin present.
    PwmOut { complementary: bool, dead_time: bool, etr: bool },
    /// An ADC input landing. `sequencer_group` ties conversions to their
    /// sequencer regardless of trigger (incl. software-triggered).
    AdcInput { adc: String, channel: u8, purpose: String, sequencer_group: Option<u32> },
    /// An external board fault input PIN (TIM BRK / HRTIM FLT) — distinct from
    /// an internal COMP→break/FLT *route* (which lives in `routes`).
    FaultIn,
    /// Opamp follower/PGA external I/O (vinp/vinm/vout present).
    OpampIo,
}

/// Resolved DMA channel. `(controller, channel)` survives regeneration.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DmaBinding {
    pub controller: String,
    pub channel: u8,
}

// ---------------------------------------------------------------------------
// LAYER 2 — the analog fabric as an explicit edge graph.
//
// `EdgeKind` is a CLOSED set of silicon TOPOLOGIES (small, stable, spans the
// fabric-bearing families); endpoints (`FabricNode`) are regeneration-stable
// owned strings — the same shape the live `constraint.rs` `Res::Route(
// "dac_to_comp", …)` already keys routes by. No typed id-enum is persisted.
// ---------------------------------------------------------------------------
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct FabricNode {
    /// Node class: `"DAC"`,`"COMP"`,`"EEV"`,`"FLT"`,`"HRTIM_TIM"`,`"TIM"`,
    /// `"ADC"`,`"MASTER"`.
    pub class: String,
    /// Instance number as the fabric tables key it.
    pub instance: u8,
    /// Sub-channel for nodes that have one (e.g. `DAC3 CH1` → `Some(1)`).
    #[serde(default)]
    pub channel: Option<u8>,
    /// For timer-event crossbar sources, the raw event/slot name
    /// (`"Cr2"`,`"Mper"`,`"Mcr1"`) — preserves identity without an enum.
    #[serde(default)]
    pub event: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RouteEdge {
    pub from: FabricNode,
    pub to: FabricNode,
    pub kind: EdgeKind,
    /// Cross-ref tag grouping a fabric path / linking it to the owning leg or
    /// requirement (so flattening doesn't lose grouping). `None` = ungrouped.
    #[serde(default)]
    pub group: Option<u32>,
}

/// Every fabric edge the families can express. Adding a *family* does not widen
/// this; adding a new silicon *topology* (rare, and gated by hand-authored
/// fabric data) does. A load-time validator should reject edges illegal for
/// `target.family`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EdgeKind {
    /// A DAC channel sets a comparator threshold (slow or fast DAC).
    DacToComp,
    /// Comparator output drives an HRTIM external-event input. `role`
    /// distinguishes the peak EEV from the zero-cross-detect EEV.
    CompToEev { role: EevRole },
    /// Comparator output latches an HRTIM fault input (DAC(slow)→COMP→FLT).
    CompToFlt,
    /// An EEV routes (reset) into an HRTIM sub-timer. Slot-less: the consumed
    /// compare/capture register lives in `slot_claims`, not here.
    EevToHrtimTimer,
    /// Comparator output drives a timer input-capture (line-sync / zero-cross).
    CompToTimCapture { channel: u8 },
    /// Comparator output trips a timer break input (1=BRK, 2=BRK2). Covers both
    /// the G4 `bkin_comp` route and the C5 over-current fold.
    CompToTimBreak { break_input: u8 },
    /// An HRTIM sub-timer is phase-coupled to a peer sub-timer. The phase value
    /// itself is a behavioral hole.
    HrtimPhaseShiftPeer,
    /// A timer event starts an ADC conversion through a crossbar slot.
    /// HW-triggered sequencers only (software-triggered have no edge — the
    /// link is carried by `RoleKind::AdcInput.sequencer_group`).
    EventToAdcTrigger { trig_slot: u8, sequencer_kind: String },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EevRole {
    Peak,
    ZeroCrossDetect,
}

// ---------------------------------------------------------------------------
// Compare/capture slot reservations (derivable from the family model's
// `consumed()`, stored so codegen/conflict-check needn't re-derive).
// ---------------------------------------------------------------------------
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SlotClaim {
    /// Owning timer, e.g. `"TimA".."TimF"`, `"Master"`.
    pub timer: String,
    /// Slot, e.g. `"Cr2"`,`"Cr4"`,`"Cpt2"`,`"Mcr1"`.
    pub slot: String,
    pub purpose: SlotPurpose,
    #[serde(default)]
    pub group: Option<u32>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SlotPurpose {
    PcmStep,
    DemCompare,
    DemCapture,
    AdcTrigger,
}

// ---------------------------------------------------------------------------
// Lock — the durable user-intent subset that must survive a re-lower.
// ---------------------------------------------------------------------------
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Lock {
    /// Pin a signal to a specific pad.
    Pin { signal: OwnedSignal, pin: PinId },
    /// Pin a fabric endpoint so the solver can't renumber it on re-lower.
    Fabric { node: FabricNode },
}

impl PinPlan {
    /// A fresh, empty plan for `target`.
    pub fn empty(target: Target) -> Self {
        PinPlan {
            format_version: PIN_PLAN_FORMAT_VERSION,
            target,
            placements: Vec::new(),
            routes: Vec::new(),
            slot_claims: Vec::new(),
            locks: Vec::new(),
        }
    }

    /// Sort `placements` by `(peripheral, role)` for stable, diff-friendly
    /// output. Lowerers should call this before returning.
    pub fn sort_placements(&mut self) {
        self.placements.sort_by(|a, b| {
            (&a.signal.peripheral, &a.signal.role).cmp(&(&b.signal.peripheral, &b.signal.role))
        });
    }
}
