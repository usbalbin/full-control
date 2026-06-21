# peri-planner → Firmware (embassy/RTIC) Codegen — Design

> Status: **discussion, pre-implementation (2026-06-21).** Produced from three
> grounding workflows (peri-planner data model + persistence; KiCad extension
> surface; embassy/RTIC pin idioms; HRTIM/fault model; prior-art codegen
> philosophy). Nothing here is built. Goal: peri-planner is the single source of
> truth for a pin/peripheral plan, and *generates* the firmware wiring instead of
> the user transcribing it by hand a second time (the first time being KiCad).

## 0. The problem and the reframe

Designing a converter means doing the pin map **twice** — once in KiCad (symbol
pins → nets), once in firmware (embassy peripheral init). That feels like
duplication, but the decision is actually made *once*, upstream of both, in
peri-planner. So the integration is **one-to-many generation from peri-planner**,
not a fragile bidirectional KiCad↔firmware sync.

peri-planner's core unit is already the right lingua franca: `OwnedSignal{peripheral,
role} → PinId{port,num}` (`mcu_pinout.rs:42`, documented regeneration-stable). That
is exactly `(pin, peripheral, signal-role)` — the same triple KiCad records as
`(alternate "TIM3_CH1")` and embassy needs for a typed constructor.

**Decision: peri-planner is authoritative for the *plan*; firmware is *generated*;
KiCad is a *verification oracle* (see §1).** The firmware side is the higher-value
half — it's where the boilerplate pain actually is, and embassy's type-inferred AF
makes the generator unusually cheap.

## 1. KiCad side — decided, deferred (not a plugin)

An in-GUI KiCad **plugin is a dead end** and is dropped. Verified against KiCad
dev-docs updated 2026-04-04 (post the 2026-03-20 KiCad 10.0 release): the schematic
editor (eeschema), where pin mapping lives, exposes **zero** live-document API —
no SWIG (PCB-only, removed in v11), no IPC (officially "only implemented in the PCB
editor", schematic support unstarted/no date, blocked on unstable internals), no
Python console. Even the one editor with a plugin API (pcbnew) can't help: the
future-proof IPC `Pad` object doesn't expose `pinfunction` (only the deprecated
SWIG `GetPinFunction` does).

**The viable KiCad path (deferred behind firmware) is a file/CLI companion:**
- Read the chosen pin function out of `.kicad_pcb` (the pad grammar carries
  `pinfunction` = the schematic pin name incl. alternate, + net + pintype) or via
  `kicad-cli sch export netlist --format kicadxml` (carries `pinfunction="TIM3_CH1_42"`).
- Diff that against the plan → **drift detection** ("PA8→PA9 in KiCad no longer
  matches the plan"). Read-only, low risk, but a *safety net* rather than a
  time-saver. Every mature tool (circuit-synth/kicad-sch-api, kicad-skip, Atopile,
  SKiDL) integrates via files with their own representation as source of truth —
  same model as peri-planner-as-hub.
- "Feels integrated" without a plugin = a file watcher on `.kicad_sch`/`.kicad_pcb`
  surfacing drift live in peri-planner's own UI.

## 2. Firmware codegen — the principle

**Generate the resource/routing layer peri-planner is authoritative for, as a
never-hand-edit module; stop dead at the structural→behavioral boundary; emit the
behavioral config as a *typed contract* the user fills in a separate file.** Not a
full init — a **board-support scaffold**.

This cut is forced by two independent facts that happen to agree:

1. **peri-planner only models routing, never behavior** (grounded, §5). It stores
   *which* resource plays *which* role and *how things wire*, with no numeric/
   behavioral values.
2. **embassy deliberately has no init codegen** — it codegens only the register
   layer (metapac) and you build drivers by hand passing pins by ownership. A
   full-init generator fights the framework. **modm** proves the other half: its
   BSP generates only the structural board layer (pin aliases, signal→AF routing
   via `GpioConnector` + `static_assert`) and leaves behavior to hand config.

The boundary peri-planner *can't* cross is the same boundary embassy and modm
*already* draw. That agreement is the signal that it's the right line.

## 3. Three tiers — output shape is chosen per peripheral by embassy driver maturity

HRTIM is not a special case; it's just the peripheral that falls to the bottom
tier. The generator picks an output shape per peripheral:

- **Tier 1 — typed driver exists** (advanced TIM, ADC+injected, DAC, COMP
  `comp_v2`, OPAMP `opamp_v5`, FDCAN, UART, SPI, I2C): emit the **embassy
  constructor**, structural args bound (instance, pins, DMA), behavior passed in
  as a `Config` param. ~90% of peripherals.
- **Tier 2 — partial driver** (HRTIM plain dead-timed PWM via native `hrtim`
  converter helpers, or the `stm32-hrtim` adapter's `HrPwmBuilder`): emit the
  helper for what it covers; behavior still a hole.
- **Tier 3 — no driver** (HRTIM faults / EEV / PCM crossbar — native embassy has
  no fault/EEV/comparator types): emit a **raw-metapac register-routing scaffold**
  — the wiring peri-planner knows (`COMP2 → EEV1`, `FLT1` enabled on TimA,
  ADC-trigger slot) — with register *values* (filter, polarity, blanking) left as
  named holes.

Tier-3 is *why* "scaffold not full init" is forced rather than chosen: neither
embassy nor peri-planner holds the behavior, so a full init is impossible there.
Designing the whole generator around the scaffold model means the hardest case is
not a special case.

## 4. The hole mechanism — generate the contract, not a comment region

Avoid CubeMX's `USER CODE BEGIN/END` regions (clobbered on regen; init emitted out
of order). Split by **function signature**, not in-file markers: the generated code
defines a behavior struct *and consumes it*; the user authors the values in a file
the generator never touches.

```rust
// generated/buck.rs   —  @generated from plan <hash>, DO NOT EDIT
pub struct BuckBehavior {        // <- the CONTRACT: one field per hole peri-planner identified
    pub dead_time_ns: u16,
    pub fault: FaultCfg,         // filter / polarity / blanking
    pub pwm_freq: Hertz,
}
pub fn init_buck(p: Peripherals, cfg: &BuckBehavior) -> BuckResources {
    // ROUTING peri-planner knows (generated):
    pac::HRTIM1.eecr1().modify(|w| w.set_ee1src(EE1_COMP2));     // PcmInternal: COMP2 -> EEV1
    pac::HRTIM1.timx(A).fltr().modify(|w| w.set_flt1en(true));   // fault routed to TimA
    // BEHAVIOR you own (generated reads cfg, never invents):
    pac::HRTIM1.fltinr1().modify(|w| { w.set_flt1f(cfg.fault.filter.into());
                                       w.set_flt1p(cfg.fault.polarity.into()); });
    // ...
}
```

```rust
// app/buck_cfg.rs   —  YOURS, never regenerated
pub const BUCK: BuckBehavior = BuckBehavior {
    dead_time_ns: 80,
    fault: FaultCfg { filter: Filter::Fdiv8N8, polarity: ActiveLow, blanking: 0 },
    pwm_freq: Hertz(200_000),
};
```

**Key property: the generated `BuckBehavior` struct *is* the spec of what must be
configured.** Add a second fault input in the picker → a new field appears → the
hand-written `buck_cfg.rs` fails to compile until its value is supplied. You
physically cannot add a routed resource and forget to configure it; structural plan
and behavioral config cannot silently drift.

## 5. The drift guarantee

- Generated module is **never hand-edited**; regenerate on build (`build.rs`) or
  gate in CI with a check-mode diff (the `codegenrs` pattern).
- Emit `static_assertions`/`const` assertions tying the plan to metapac reality
  (à la modm), so a stale plan or a metapac bump that moved a pin's AF is a
  **compile error, not a silent miswire.**

## 6. Grounding facts (so this doc stands alone)

**embassy-stm32 coverage (2025-2026 git):** full typed drivers for advanced TIM
(`complementary_pwm.rs`), ADC+injected (`adc/g4.rs`), DAC, COMP (`comp_v2`), OPAMP
(`opamp_v5`), FDCAN, UART, SPI, I2C. Pin→peripheral binding is type-driven; **AF is
inferred** by sealed per-pin traits (`Channel1Pin::af_num()`), so the generator does
*not* emit AF numbers/`set_as_af`. Native `hrtim` has converter helpers but **no
fault/EEV/comparator types and no `HighResolutionControl`**; those go through the
re-exported metapac (`embassy_stm32::pac::HRTIM1.fltinr1()/.eecr1()`) or the optional
`stm32-hrtim` adapter (`HrPwmBuilder`). `bind_interrupts!` builds a zero-size proof
struct passed to async/DMA constructors.

**peri-planner HRTIM/fault model = routing only:** `UseHrtimSub(pinned_sub_timer,
role, outputs, fault) → HrtimSub(...)` (`requirements.rs:200-207, 844-849`). Six
sub-timers `TimA..TimF`; Master is only crossbar trigger sources `Mcr1..4/Mper`.
Outputs are coarse `OutputMode::{Ch1Only, Ch1AndCh2}` — no polarity/idle/push-pull/
chopper/set-reset. Roles (`PcmInternal/PcmExternal/VoltageModePwm/PhaseShift/
External`) are intent tags + booleans, no numeric params. PCM resolves to identities
(`dac, comp, eev, zcd_eev, dem`) with no threshold/slope. Faults reference one
`HrtimFltId`, no filter/polarity/blanking. EEVs are crossbar identities `Eev1..10`
+ COMP→EEV route only, no edge/filter/sampling. Compare/capture slots tracked as
scarce resources (`Cr2`, `Cr4`, `Cpt2`). **`phase_shift_q15` is hardcoded to 16384**
(a placeholder = 0.5, NOT real data). Absent entirely: burst, dead-time config,
counter mode, period/prescaler, repetition, sync, waveform timings.

**Prior art:** modm = compile-time signal→AF map + `static_assert`, structural BSP
only, behavior separate. CubeMX = structural MSP vs behavioral MX init; `USER CODE`
regions clobbered on regen (disliked). embassy = no init codegen by design,
register layer only. Safe Rust codegen = separate never-edit module + CI drift
check; `static_assertions` catch structural drift at zero runtime cost. Consensus:
**emit structural skeleton + named holes, not a complete init.**

## 7. Prereqs / open questions / next steps

1. **Don't generate placeholders as facts** — `phase_shift_q15 = 16384` is fake.
   Rule of thumb: if peri-planner hardcodes it, it's a hole, not a value.
2. **Bump the pinned metapac** (`2ab6df2` → embassy's current tag) so Tier-3 raw
   register writes match the `pac` embassy re-exports. See [[project-refresh-stm32data-for-c5]].
3. **Input is the Stage-0 unified `PinPlan`** — does not exist yet. Today the three
   family models (`Design` / `H523Design.pin_locks` / `C531Design.legs`, the last
   storing *intent* not resolved pins) must be flattened into one serializable plan
   first. The live constraint engine is `src/constraint.rs` (the old `src/kernel/`
   was deleted in `1dec619` as a redundant reimplementation); its `Res::Route(
   "dac_to_comp", …)` owned-route form is the convergence seam for `PinPlan` —
   see §8. See [[project-constraint-selector]].

**Decision (2026-06-21): nail the `PinPlan` shape first** (done — §8), since
everything downstream keys off it. The deferred alternative was to prototype the
pipeline on a **Tier-1** peripheral (UART/ADC) first, then **HRTIM Tier-3** as the
stress test; that comes after the type lands.

## 8. The unified `PinPlan` type (designed 2026-06-21)

> Produced by a judge-panel of 3 independent designs (minimalist / fabric-faithful /
> engine-convergent) → synthesis → adversarial red-team against the real code. The
> red-team found and we fixed three real losslessness gaps (see §8.4). Built
> entirely on the already-serde, regeneration-stable vocabulary `OwnedSignal{
> peripheral,role}` (`mcu_pinout.rs:46`) + `PinId{port,num}` (`mcu_pinout.rs:15`),
> with **no** typed id-enum (`DacId`/`CompId`/`CrossbarSource`) welded into the
> persisted schema. Save-compat with the three family structs is dropped (per
> [[project-save-compat-not-required]]); version starts fresh at 1.

### 8.1 Stance

Two flat layers + locks + target:

- **`placements`** — every pin-bearing signal landed on a physical pin. This is the
  *only* thing KiCad and Tier-1 constructors need, and it materializes the join
  that's split across `Design.assignments` + `Design.pin_assignments` today, and
  that C531 doesn't store at all.
- **`routes`** — the analog fabric as an explicit edge graph. `EdgeKind` is a closed
  set of silicon *topologies* (small, stable, spans all families); endpoints
  (`FabricNode`) are regeneration-stable owned strings — the same shape as the live
  `constraint.rs` `Res::Route("dac_to_comp", dac, ch, comp)` (this is the
  convergence seam: a future `Solution → PinPlan` lowering is natural, but nothing
  here depends on it, since `Res` is `&'static str` + not `Serialize`).
- **`slot_claims`** — HRTIM compare/capture register reservations (Cr2/Cr4/Cpt2,
  MasterCompare). Re-derivable from `consumed()` but stored so codegen picks the
  right compare register and conflict-check needn't re-derive. (Red-team fix — see
  §8.4.)
- Captures **structural/routing facts only**. No field can hold a numeric/behavioral
  value, so the fake `phase_shift_q15 = 16384` has nowhere to land — it re-emerges
  only as a codegen hole.

### 8.2 The type

```rust
// peri-planner/src/pin_plan.rs
use crate::mcu_pinout::{OwnedSignal, PinId};
use serde::{Deserialize, Serialize};

pub const PIN_PLAN_FORMAT_VERSION: u32 = 1;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PinPlan {
    pub format_version: u32,
    pub target: Target,                       // chip identity; folds in the package H523/C531 store separately
    pub placements: Vec<Placement>,           // LAYER 1 — sorted by (peripheral, role) for clean diffs
    #[serde(default)] pub routes: Vec<RouteEdge>,    // LAYER 2 — empty for plain H5/C5A3 comms plans
    #[serde(default)] pub slot_claims: Vec<SlotClaim>, // HRTIM compare/capture reservations (derivable)
    #[serde(default)] pub locks: Vec<Lock>,   // durable user intent, NOT the transient conflict set
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Target { pub package: String, pub family: Family }   // package = mcu::Package key, e.g. "G474RE"

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Family { G4, H5, C5 }                                  // gates which EdgeKinds are legal

// ---- LAYER 1 ----
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Placement {
    pub signal: OwnedSignal,                  // ("TIM1","CH1") — KiCad pinfunction half AND codegen instance+role
    pub pin: Option<PinId>,                   // None = declared/routed but unplaced (real persisted state)
    pub origin: PinOrigin,                    // Locked/Solver/Forced — lets drift-check tell user-fix from re-solve churn
    #[serde(default)] pub af: Option<u8>,     // None = analog/dedicated pin; informational (embassy infers AF)
    #[serde(default)] pub role_kind: RoleKind,// constructor-shaping STRUCTURE (which pins/args), never values
    #[serde(default)] pub dma: Option<DmaBinding>,  // (controller,channel) — fixes the lossy dma:u8 count
    #[serde(default)] pub irqs: Vec<String>,  // metapac IRQ names for bind_interrupts! (no producer yet)
    #[serde(default)] pub package_pin: Option<u16>, // physical pad for KiCad; needs a logical->pad table
    #[serde(default)] pub net: Option<String>,// schematic net; unmodeled today — reserved hole
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PinOrigin { Locked, Solver, Forced }

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RoleKind {
    Gpio,
    Comms { flow_control: bool, synchronous: bool, nss: bool, smba: bool },
    PwmOut { complementary: bool, dead_time: bool, etr: bool },   // dead_time = generator ON; ns is a hole
    AdcInput { adc: String, channel: u8, purpose: String, sequencer_group: Option<u32> }, // group ties convs to seq
    FaultIn,                                  // external board fault PIN (TIM BRK / HRTIM FLT) — distinct from a COMP route
    OpampIo,
}
impl Default for RoleKind { fn default() -> Self { RoleKind::Gpio } }

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DmaBinding { pub controller: String, pub channel: u8 }

// ---- LAYER 2 — fabric edge graph ----
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct FabricNode {
    pub class: String,                        // "DAC","COMP","EEV","FLT","HRTIM_TIM","TIM","ADC","MASTER"
    pub instance: u8,                         // as the fabric tables key it
    #[serde(default)] pub channel: Option<u8>,
    #[serde(default)] pub event: Option<String>, // "Cr2","Mper","Mcr1" for timer-event crossbar sources
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RouteEdge { pub from: FabricNode, pub to: FabricNode, pub kind: EdgeKind,
    #[serde(default)] pub group: Option<u32> }

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EdgeKind {
    DacToComp,                                // G4 + C5
    CompToEev { role: EevRole },              // G4 — peak vs zero-cross-detect EEV
    CompToFlt,                                // G4 — short-circuit latch (DAC(slow)->COMP->FLT)
    EevToHrtimTimer,                          // G4 — EEV reset routing into a sub-timer (SLOT-LESS; fix §8.4)
    CompToTimCapture { channel: u8 },         // G4 — COMP -> TIM input-capture (line-sync); fix §8.4
    CompToTimBreak { break_input: u8 },       // G4 bkin_comp + C5 OCP fold (1=BRK,2=BRK2); generalized; fix §8.4
    HrtimPhaseShiftPeer,                      // G4 — sub-timer<->peer phase coupling; phase value is a hole; fix §8.4
    EventToAdcTrigger { trig_slot: u8, sequencer_kind: String }, // G4 — HW-triggered seq only (software has no edge)
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EevRole { Peak, ZeroCrossDetect }

// ---- compare/capture slot reservations (derivable from consumed(), stored for codegen) ----
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SlotClaim { pub timer: String, pub slot: String, pub purpose: SlotPurpose,
    #[serde(default)] pub group: Option<u32> }   // timer "TimA".."Master", slot "Cr2"/"Cr4"/"Cpt2"/"Mcr1"

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SlotPurpose { PcmStep, DemCompare, DemCapture, AdcTrigger }

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Lock { Pin { signal: OwnedSignal, pin: PinId }, Fabric { node: FabricNode } }
```

### 8.3 Lowering (one writer per family → `PinPlan`)

- **G474** (`Design`, `requirements.rs:1513`): for each `Some(assignment)`, call the
  existing `assignment_signals()` (`requirements.rs:323`, includes ADC inputs) → pin
  via `pin_assignments` (`origin=Locked`) else single-candidate (`Forced`) else
  `Solver`. `RoleKind` from the variant. Routes walk the resolved fabric fields:
  `PcmInternal{dac,comp,eev,zcd_eev,dem}` → `DacToComp` + `CompToEev{Peak}` +
  `EevToHrtimTimer` (+ `CompToEev{ZeroCrossDetect}` if `zcd_eev`); `dem` ⇒
  `SlotClaim`s `Cr2`(PcmStep)+`Cr4`(DemCompare)+`Cpt2`(DemCapture). `ShortCircuitFault`
  → `DacToComp`+`CompToFlt`. `Tim.capture_comp`→`CompToTimCapture`,
  `bkin_comp`→`CompToTimBreak`. `PhaseShift{peer}`→`HrtimPhaseShiftPeer`.
  `AdcSequencer` HW-trigger→`EventToAdcTrigger`+`SlotClaim`(AdcTrigger); conversions
  carry `sequencer_group`. `phase_shift_q15` dropped.
- **H523/C5A3** (`H523Design`, `h523_design.rs:54`): trivial — each `PinLock` →
  `Placement{origin:Locked}`; declared `use` with no lock → `pin:None`; `routes`/
  `slot_claims` empty; package folded into `target`.
- **C531** (`C531Design`, `c531_design.rs:170`): **materializes pins** (the model
  stores none). Per `ConverterLeg`: `leg.signals()` (+ synthesize `AdcIn` from
  `adc_sense`, which `signals()` drops) → pick one pin honoring `Lock` then
  conflict-free candidate, **store port/num by value** (never by candidate index —
  data order shifts on regen). `ocp` → `CompToTimBreak`(+`DacToComp` if threshold
  DAC). `etr`/`bkin2` are unrepresentable in the leg model → documented unlowered gap.

### 8.4 Red-team verdict & fixes applied

Architecture confirmed sound (two-layer shape, owned-string vocabulary,
structural/behavioral boundary, C531 by-value materialization, honest net/pad
holes). Three real **losslessness** gaps were found in the raw synthesis and are
fixed above:

1. **(blocker) G474 COMP→TIM internal routes dropped** — `Assignment::Tim.capture_comp`
   and `.bkin_comp` (`requirements.rs:832`) had nowhere to land. Fixed: added
   `CompToTimCapture` and generalized `CompToTimBreak` to G4.
2. **(major) HRTIM phase-shift peer link dropped** — `HrtimResolved::PhaseShift{peer}`
   coupling was erased. Fixed: added `HrtimPhaseShiftPeer`.
3. **(major) `EevToHrtimTimer` mis-welded the peak EEV to slot `Cr2`** — in
   `consumed()` the EEV route and the `Cr2` compare claim are *two* resources.
   Fixed: `EevToHrtimTimer` is now slot-less (pure reset routing); compare/capture
   reservations moved to `slot_claims` (also fixes the MasterCompareSlot minor).
4. **(major) software-triggered ADC sequencer + grouping** — `AdcSequencer{trigger:
   Software}` (`requirements.rs:1339`) emits no event edge. Fixed: `sequencer_group`
   on `AdcInput` carries the conversion↔sequencer link regardless of trigger;
   software-triggered sequencers simply have no `EventToAdcTrigger` edge (documented
   convention).

### 8.5 Open questions (policy, not shape)

1. **Persisted vs derived — DECIDED 2026-06-21: DERIVED-on-save.** The three family
   structs (`Design`/`H523Design`/`C531Design`) stay authoritative for editing;
   `PinPlan` is regenerated from them (one-way `lower`) as a pure downstream output
   for codegen + KiCad. Consequences locked in: the lowerers are **additive** (no
   change to the family structs / planner), `PinPlan` is **never edited or
   round-tripped**, and being **lossy on intent** (`RequirementSpec`, `use` roles,
   leg flags) is therefore fine. (Rejected: PinPlan as the persisted/edited form —
   would force round-trippability and a planner rewrite.)
2. **Solver-pin stability:** promote a stored `Solver` pin to an implicit `Lock` on
   re-lower (so nets never silently move), or let drift-check tolerate `Solver`
   moves? Type supports both; policy unset.
3. **Logical→physical pad map** is deliberately *not* in `PinPlan`. Lives where — a
   per-`Package` table in `mcu_data`, or the KiCad companion? Until it exists,
   `package_pin` stays `None` and drift-check matches on logical `Pxy`.
4. **DMA/IRQ producer:** `DmaBinding`/`irqs` have a home but no producer; needs a
   derive step before Tier-1 async codegen works. Distinguish "blocking driver" from
   "unresolved"?
5. **Load-time validator:** `FabricNode`/`EdgeKind` can express silicon-illegal
   routes. A `validate(plan, ChipFabric)` pass (reachability via `comps_for_dac` /
   `eevs_for_comp` / `comps_for_tim_break`) is needed for hand-edited/superset plans
   — in scope for v1?
6. **`FabricNode` string conventions** (class names, event names, C5 placeholder
   `channel=1`) are an unwritten contract between materializer and codegen reader —
   want a shared format/parse module + golden round-trip test pinned to
   `crossbar_from_name`.

### 8.6 Implementation status (2026-06-21)

**Type + all three lowerers built and tested** (`src/pin_plan.rs` + a one-way
`to_pin_plan` on each family model; never edited back). `target.family` shipped as
an open `String`, not a closed enum, per the scalability requirement. 111 lib tests
green, clippy unchanged (0 net-new), native + wasm clean.

- **`H523Design::to_pin_plan`** — the *generic* lowerer (touches nothing
  family-specific): locks → placed `Placement`s, declared-but-unplaced uses →
  `pin: None`, empty fabric. Serves H5, C5A3, and any future descriptor-only family
  unchanged.
- **`C531Design::to_pin_plan`** — *materializes* pins (the model stores none),
  conflict-avoiding, stored **by value**; `ocp` → pinless `CompToTimBreak`
  (+ `DacToComp`).
- **`Design::to_pin_plan`** (G474) — reuses `assignment_signals()` for placements;
  full HRTIM/fault/phase-shift/ADC-trigger fabric → `routes` + `slot_claims`;
  `EevToHrtimTimer` slot-less (the red-team fix), compare registers in `slot_claims`.

**Deferred (documented in code):** `AdcConversion → AdcSequencer` group linking
(`sequencer_group: None` for now — needs the parallel `requirements`/`ids` walk);
G474 capture-channel / break-input specifics and C531 `ETR`/`BKIN2` (the leg model
doesn't record them); `dma`/`irqs`/`package_pin`/`net` have homes but no producer.
All behind `serde(default)`, so filling them later won't reshape the type.

**Dispatch + Tier-1 codegen built (2026-06-21):**
- `Project::to_pin_plan` — the single dispatch point; picks the per-family lowerer
  by active MCU and builds one consistent `Target` (family = open string). (G474's
  lowerer was refactored to take a `Target` too, for uniformity.)
- `src/codegen.rs` `generate(&PinPlan) -> String` — the firmware scaffold:
  - **Resource bundles** (`generate_board`): one bundle struct per peripheral
    instance (its typed singleton + role-named pins), aggregated into a `Board`
    with `Board::take(p)` destructuring embassy `Peripherals`. Instance field only
    for Tier-1 classes (USART/UART/LPUART/SPI/I2C/TIM/ADC/DAC/FDCAN/HRTIM); analog
    COMP/OPAMP bundle pins only. Unplaced signals → comment, never a bogus field.
    Pinless fabric-route endpoints that are real singletons (an OCP comparator, a
    PCM threshold DAC) get an **instance-only bundle** so the user still gets
    `p.COMP1`/`p.DAC3` for raw fabric config.
  - **Behavior contract** (`write_behavior`): the §4 holes — `Behavior` struct +
    `Default`, one field per value the plan can't know (baud / freq / bitrate /
    PWM freq + dead-time), derived from each instance's class + role kinds.
  - Wired to a **"Gen firmware"** button (copies the scaffold to clipboard).
  - Verified on the real default-G474 design (`Hrtim1{instance, cha1..chd2, flt5}`,
    `Adc1`, `Dac1` bundles; `Behavior{hrtim1_freq_hz, hrtim1_dead_time_ns}`).

### 8.7 The bundle boundary — DECIDED 2026-06-21 (we do NOT generate init)

We deliberately stop at **resource bundles** and never emit driver *constructor*
calls (`SimplePwm::new`, `Uart::new`, …). Rationale (user's call, endorsed):

- **embassy's init API churns**; chasing it is unsustainable and peri-planner has
  no embassy dependency to compile against, so emitted constructors would be
  blind guesses.
- **embassy constructors are generic over pin marker traits** (`TxPin<USART1>`,
  `Channel1Pin<TIM1>`). The user writes the call once against a bundle —
  `Uart::new(b.usart1.instance, b.usart1.rx, b.usart1.tx, …)` — and re-pinning +
  regenerating changes only the bundle field *types* (the *names* are stable), so
  the call still compiles. **The bundle is a version-stable interface; the init
  call is the user's, written once.**
- Generated code therefore touches only `peripherals::X` + `Peripherals` field
  access — the most stable surface in embassy-stm32. Codegen ≈ never breaks on an
  embassy bump.
- This dissolves the earlier "validate constructors against `hard-rt`" frontier:
  there are no constructors to validate.

**Caveats:** `bind_interrupts!` needs literal IRQ idents (a macro), so IRQs aren't
fully bundleable — the user writes that block (or we emit it separately, since IRQ
names track the instance, not the pins). DMA channels *are* bundleable (a
`DMA1_CH3` singleton is a field like a pin) once the DMA producer lands.

**Possible future niceties (not needed):** a shared trait per class so one generic
`fn setup_uart<B: UartBundle>(b: B)` handles any UART bundle; user-assigned
semantic bundle names (ties to the deferred `net`/label idea).

Related: [[project-peri-planner]], [[project-save-compat-not-required]],
[[user-role]].
