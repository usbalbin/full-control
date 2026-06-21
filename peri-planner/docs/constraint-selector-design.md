# peri-planner → Constraint-Based MCU Selector — Design

> Status: **BUILT — re-baselined 2026-06-21** (was: design, pre-implementation).
> The kernel this doc designed is shipped. The goal — user states high-level
> constraints (">=2 USART each with DMA RX+TX, >=1 ADC with DMA, hardware OCP via
> COMP→timer-break") and the tool returns the parts that satisfy them *with a
> feasible allocation* — is realized in `src/constraint.rs` (the generic kernel),
> `src/select.rs` (the descriptor→`Candidate` adapter + two-tier evaluation), and
> `src/catalog_view.rs` (the Part-finder requirements panel + 3-state verdicts).
> §1–§5 below describe the **as-built** architecture; §6 (the increment ladder) is
> annotated with verified status; §0/§8 (reconciliation + red-team) record which
> resolutions landed.
>
> **At a glance:**
> - **Inc 0–8: DONE.** Two caveats: Inc 2 deferred the per-instance capability
>   *vector* / `fabric_flags` (the count-style Tier-1 bounds shipped instead);
>   Inc 8 deferred the minimal-unsat-core (C531 keeps its own rich diagnostics).
> - **Inc 9: PARTIAL.** The analog routing IS data-driven from `G4_FABRIC`
>   (`g474.rs` LazyLocks, commit c34a574). The rest — retiring the legacy G474
>   `solver.rs` backtracker and folding `normalize()` onto the kernel — is **not
>   done and is recommended deferred** (the doc itself keeps G474 on greedy
>   `normalize`, §3; the dynamic-vs-static candidate mismatch makes the fold
>   low-ROI/high-risk against a working planner).
> - **§0 reconciliation:** all four fixes (B1 bridge, DMAMUX+NAMED scope, channel-
>   count DMA demand, dual-core dedup) landed.
> - **§8 red-team:** addressed; **A4 is partial** (DMA demand is a per-kind channel
>   *count* — 2 serial / 1 ADC — not yet full signal-set cardinality, so a literal
>   7-stream HRTIM workload would undercount); **D1/D2/D3 superseded** by the
>   "save-compat not required" decision (clean break, no migration shims).

## 0. Reconciliation / decisions on top of the workflow output

> **Status (2026-06-21): all four fixes landed.** 1) bridge built (`for_chip_name`);
> 2) DMAMUX+NAMED in scope, FIXED/dual-core deferred; 3) DMA demand is a per-kind
> channel count (partial vs full signal-set — see §8 A4); 4) dual-core dedup done in
> `gen_catalog`. The "save-compat not required" lever (end of §0) was taken — the
> persisted schema is a clean break with no migration shims.

The synthesized plan (§2–§7 below) is sound in its **architecture**, but the
red-team (§8) found four facts that are false against the current data and must
be resolved *before* the first commit. My reconciliation:

1. **The Tier-1→Tier-2 bridge is the #1 thing to fix (red-team B1).** `Package::for_chip_name`
   (mcu.rs:157) does exact full-name matching and reaches ~1 catalog part per
   family. Since Tier-2 (the backtracking solve on a real `McuDescriptor`) is the
   only *sound* stage, ~99% of the catalog would sit in "bounds-only", which the
   DMA findings (A1/A4) prove is unsound-upward (false positives). **Resolution:**
   build a robust `part-number → (family, package-letter) → descriptor` map that
   ignores flash/temp/packaging suffixes (pins & instance counts are
   flash-invariant, so reusing the one compiled RAW per package letter is sound).
   Accept that only the ~4 compiled families (G474/H523/C5A3/C531) get
   `allocation-verified`; **everything else is explicitly `bounds-only (NOT verified)`** —
   a *third* honest label state, not "plausible".

2. **Scope v1 to DMAMUX + NAMED-controller families; defer FIXED-channel and dual-core.**
   This sidesteps red-team A1 (fixed-channel DMA, ~700 parts — mostly the
   F-series you already exclude, plus L0/L1/L4), A2/A3 (dual-core DMA double-count
   + shared `Inst` identity on H7/WL/MP1). v1 covers C0/G0/**G4**/L4+/L5/U0/WB/WL
   (DMAMUX) and **C5**/**H5**/N6/U3/U5/WBA (NAMED). H7 and the FIXED families are
   shown but labeled out-of-scope-for-allocation. (FIXED fits the `Res::Pool`
   model later as a singleton allowed-set; it's deferred, not impossible.)

3. **Fix the DMA demand model: it's a channel-count / signal-set, not `{None,Rx,Tx,RxTx}` (red-team A4).**
   HRTIM exposes 7 DMA signals and TIM1/8/20 each expose 7 — the literal
   HRTIM-converter workload can need 4–7 channels from one peripheral. A 4-value
   enum silently undercounts. Demand is "this peripheral wants channels for signal
   subset S"; aggregate bound sums real demand.

4. **Dedupe DMA channels by `(controller, channel)` even in Inc 0 (red-team A2).**
   `Σ cores[].dma_channels` double-counts on dual-core parts (H745: 80 listed, 40
   real). gen_catalog already unions peripheral *names* across cores (lines 64-73);
   the DMA count must dedupe the same way — else the very first commit ships a
   wrong number.

**Your "save-file backwards-compat is not required" lifts the migration's biggest
risk.** Red-team D1/D2/D3 (typed `pinout::Pin` vs `PinId`, `SignalId` not
`Serialize`, `ReqKind::Hrtim` welding a G474 type into the persisted enum) all
revolved around preserving the serde format. We can redesign the persisted
`Design`/requirement/token schema as a clean break and drop old saves — so those
become ordinary refactors, not migration landmines. See
[[project-save-compat-not-required]].

**Cut from v1:** the `min_with_dma` per-instance predicate (red-team B4) — 100% of
async-serial instances carry DMA on every in-scope family, so it filters nothing.
The binding DMA constraint is **pool capacity**, which *is* discriminating (9
USART-RxTx > C531's 8 channels). Keep the capability vector for forward use;
don't ship the dead predicate.

**Don't over-commit to folding C531's diagnostics into the kernel (red-team C3).**
`C531Design::validate` returns *all* pairwise conflicts; a backtracker returns one
witness/certificate. Recovering N reasons needs minimal-unsat-core extraction
(re-solve per dropped requirement) — real work. Keep C531's rich diagnostic pass
as-is initially; the kernel answers feasible/infeasible.

---

## 1. Chosen architecture: Capability Kernel (Design 2 + grafts)

Lift `requirements.rs`'s claim/conflict kernel into a single chip-agnostic
`solver/` engine whose resource tokens and candidate routes come from
`McuDescriptor`/`ChipFabric`/DMA **data**, with G474's HRTIM model and C531's
converter legs as thin **enumerator adapters**. It won because the central token
vocabulary **stops growing per family**: adding a family mints zero new token
variants. (Design 1 "generalize in place" keeps typed G474 variants in the
central enum forever; Design 3 forks into two permanently-maintained kernels.)

Grafts: mandatory backtracking on the *selector* path (Design 3) bounded by
problem size **not a timeout**; the monotonic necessary-bounds contract (Design
3); the extractor seam that collapses DMA models (Design 1); two-tier filter +
honest labeling (all three).

## 2. The generic model (`src/solver/`)

### Resource tokens (`tokens.rs`) — numeric, family-agnostic
```rust
pub enum Class { Usart, Uart, Lpuart, Spi, I2c, I3c, Fdcan, Ucpd, Usb,
                 Adc, Dac, Comp, Opamp, Tim, Hrtim, Octospi }   // metapac's own class strings
pub type PoolId = &'static str;                                  // "DMA1","LPDMA2","GPDMA1"
pub enum RouteKind { DacToComp, CompToTimBreak, CompToEev, CompToFlt, CrossbarToAdc }

/// Opaque exclusion token. The engine ONLY does HashSet membership (+ a count
/// for Pool). It NEVER pattern-matches a Res.
pub enum Res {
    Inst(Class, u8),                       // whole instance: USART2, TIM1, COMP3
    SubChannel(Class, u8, u8),             // ADC ch / DAC ch / opaque HRTIM compare-slot
    Route(RouteKind, u8, u8, u8),          // ChipFabric edge, e.g. (CompToTimBreak, comp, tim, brk)
    Pin(crate::mcu_pinout::PinId),         // cross-fabric conflict token
    Pool(PoolId, u8),                      // one DMA channel; capacity-N = N exclusive tokens
}
```
HRTIM's RM0440 compare-slot/DEM/dual-ADC accounting is quarantined as opaque
`SubChannel` tokens the kernel never interprets — one unified kernel that still
handles the hardest consumer.

### Requirement (`requirement.rs`) — user intent, family-neutral
```rust
pub struct Requirement { pub id: u32, pub parent: Option<u32>, pub kind: ReqKind }
pub enum ReqKind {
    UsePeripheral { class: Class, dma: DmaDemand, pinned: Option<u8> },   // dma = channel-set (see §0.3)
    HardwareOcp   { tim: Option<u8>, break_input: u8, threshold_dac: bool },
    AdcSense      { speed: SpeedPref, pinned: Option<u8> },
    Hrtim(/* G474 adapter owns HRTIM intent */),
}
```
`parent` generalizes today's `AdcConversion.group: u32` cross-requirement ref.

### Per-assignment contract — the one thing that survives
`trait Consume { fn consumed(&self) -> Vec<Res>; }` (exactly requirements.rs:837).
Candidate generation reads **data** (mirroring `C531Design::validate`): `Inst`
from `McuDescriptor.comms/adcs/dacs/comps/timers`; `Pool` from new
`descriptor.dma_pools()` + `dma_routes(class,inst)`; `Route` from
`ChipFabric::comps_for_tim_break`/`dac_threshold_sources_for_comp`; `Pin` from
`mcu_pinout::pins_for(raw, SignalId)`.

## 3. Feasibility solver

`(descriptor, Vec<Requirement>) -> Allocated(witness) | Infeasible{unmet}`.

- **DMA (the load-bearing seam).** The **extractor** normalizes both DMA models
  into `leg → {allowed PoolId}`: Model A (dmamux, `dma=None` — G4) → all
  controllers behind that DMAMUX (`DMAMUX1 → {DMA1,DMA2}`, derived by grouping
  supply channels' `dma` under their shared `dmamux`); Model B (named `dma` —
  C5/H5) → the explicit set. A pool of N channels = N exclusive `Pool(id,0..N)`
  tokens, so "demand ≤ channels" falls out of single-consumer. Request# is a
  selector, not scarce (true for DMAMUX/NAMED; **not** for FIXED — deferred).
  A test asserts every demand `PoolId` resolves to a supply pool.
- **Pin contention.** Generalize `forced_pin_claims` from the typed
  `crate::pinout` to `mcu_pinout::pins_for`/`pin_candidates_respecting_locks`
  (keyed on `SignalId=(peripheral,role)`). Single-option folding + candidate
  pin-choice enumeration so the backtracker catches forced collisions.
- **Internal routes.** `HardwareOcp` lowers to `Route(CompToTimBreak,...) +
  Inst(Comp) [+ Route(DacToComp,...) + Inst(Dac)]`, folding C531's pairwise loop
  into the solver. `fabric == None` (H523/C5A3) ⇒ degrade to *unknown*, never
  infeasible.
- **Backtracking** (selector path): most-constrained-first, bounded by problem
  size (single-digit reqs × instances), **no wall-clock timeout** (a timeout on a
  deep-but-satisfiable instance is an unsound false-negative). Live G474 UI keeps
  greedy `normalize`.

## 4. Catalog filter at scale (two tiers)

1. **Tier-1** (all ~1600 parts, integer compares): `SearchQuery::matches` + new
   monotonic predicates lowered from the requirement set into one aggregate
   `Bounds` (`min_dma_channels`, `min_count[class]`, `require_fabric` bitset).
   Strictly necessary, loose (aggregate demand ≤ aggregate pool — **never** assume
   even per-controller spread), never drops a feasible part.
2. **Tier-2** (survivors with a reachable descriptor, via the §0.1 bridge): build
   `McuDescriptor`, lower to `Requirement`s, run the backtracking engine →
   `allocation-verified` + witness. Survivors without a descriptor →
   `bounds-only (NOT verified)`.

Per-part capability vector on `CatalogEntry` (additive): `dma_pool_total: u16`
(dedup by controller+channel), `caps: Vec<InstanceCap{class,index,has_dma}>`,
`fabric_flags: u16`. Emitted by `gen_catalog`. `catalog_view` grows a
requirements panel + a 3-state status column.

## 5. G474 coexistence / migration (clean break OK — saves can be dropped)

Lift the kernel into `solver/`, wrap G474 + C531 as adapters, then delete the
forks. G474 golden tests pin byte-identical assignments at each step. End state:
**one kernel**; G474 = richest adapter, C531 = thin adapter + diagnostics;
`g474.rs` routing fns and the legacy `solver.rs` backtracker deleted. Note the
`solver.rs` → `solver/mod.rs` rename must be sequenced (Rust won't allow both),
and `ShortCircuitFault`/`ShareBusDrive` (imported by live `requirements.rs`) move
atomically.

## 6. Increment ladder (each green/shippable) — corrected for the red-team

Status verified against the code 2026-06-21 (✅ done · 🟡 partial · ⬜ todo). The
real homes: kernel = `src/constraint.rs`; adapter + two-tier eval = `src/select.rs`;
UI = `src/catalog_view.rs`; data = `src/catalog.rs`/`tools/gen_catalog.rs`/
`src/phys_pinout.rs`/`src/mcu_raw.rs`/`src/mcu.rs`/`tools/{extract,gen_descriptors,
json_dma}.rs`.

- ✅ **Inc 0** *(0382354)* — `CatalogEntry.dma_pool_total` (deduped by
  (controller,channel) across cores in `gen_catalog`) + `SearchQuery.min_dma_channels`
  + the "Min DMA ch" widget. Test pins G474=16, H745=40-deduped.
- ✅ **Inc 0.5** *(c6f948e)* — `Package::for_chip_name` suffix-stripping bridge
  (exact → 10-char prefix) + `desc_asset::descriptor_for`. (The 3rd "BoundsOnly"
  label state actually shipped with Inc 7.)
- ✅ **Inc 1** *(a62ad46; C5 path b9b47be)* — `RawPeripheral.dma: &[RawDmaLeg{signal,
  pools}]` + `RawMcuData.dma_pools` + `McuDescriptor::dma_pools()`/`dma_routes()`;
  extractor normalizes DMAMUX/named/fixed. metapac drops C5 DMA, so C5 descriptors
  source it from chip JSON via `tools/json_dma.rs`.
- 🟡 **Inc 2** *(d6f31c7)* — Tier-1 capability filters shipped: `comp_pwm_ch`,
  `gpio_pins`, `min_tim_adv`, `require_ocp_capable`. **Deferred:** the per-instance
  capability *vector* (`Vec<InstanceCap{class,index,has_dma}>`) + `fabric_flags`
  fields on `CatalogEntry` (§4 still promises them; never added — Tier-2 covers the
  precise check, so they're low-value).
- ✅ **Inc 3** *(95771b7)* — generic kernel `src/constraint.rs`: `Res{Inst,Pool,Pin,
  Sub,Route}` (interned-str classes), `Candidate`, `Requirement`, `assign_greedy`.
  (Built as `constraint`, not `tokens`/`engine`.)
- ✅ **Inc 4** *(a76964d)* — DMA-first end-to-end in `select.rs`: `Demand` →
  candidates from `dma_routes()`/`dma_pools()` → `solve`/`allocate` + witness.
- ✅ **Inc 5** *(b835f99)* — `assign_backtracking`: most-constrained-first, bounded
  by a node budget (not wall-clock), `indeterminate` on budget exhaustion.
- ✅ **Inc 6** — generic pin contention: `select.rs` emits `Res::Pin` candidates
  from `mcu_pinout::pins_for` for the logical kinds (SERIAL/SPI/I2C/UCPD/ADC).
- ✅ **Inc 7** *(7ab100c)* — two-tier `evaluate` → `Verdict{Verified,BoundsOnly,
  Infeasible}` + `Witness`; `catalog_view` requirements panel + verdict column +
  3-state labels.
- ✅ **Inc 8** — `HardwareOcp` via fabric: `select.rs::ocp_candidates` claims
  COMP+TIM tokens; C531 folded into the lineup eval. **Deferred (by design):**
  minimal-unsat-core (C531 keeps its own rich pairwise diagnostics, per §0/C3).
- 🟡 **Inc 9** — retire forks. ✅ analog routing data-driven from `G4_FABRIC`
  (`g474.rs` LazyLocks, c34a574). ⬜ **Not done / recommended deferred:** delete
  `g474.rs` routing fns, retire the legacy `src/solver.rs` backtracker, and make
  G474 `normalize()` delegate to `constraint.rs`. `solver.rs` is still imported by
  `requirements.rs`/`app.rs`/`fabric_view.rs`, and `normalize()` is still its own
  greedy fixpoint with **dynamic** cross-reference candidate re-enumeration — which
  doesn't map onto `constraint.rs`'s **static** `Candidate` lists. The doc itself
  keeps the live G474 UI on greedy `normalize` (§3); folding it is low-ROI against
  a working planner, so it's parked. Golden test `golden_default_design_fabric_
  allocation` (fac0df5) pins the current G474 allocation if it's ever revisited.

## 7. Open decisions for the maintainer

1. **v1 family scope** — confirm DMAMUX+NAMED in, FIXED + dual-core (H7) deferred
   with out-of-scope labeling. (Recommended.)
2. **`Class` enum vs interned `&str`** — red-team argues the catalog spans 24
   families (SAI/SDMMC/LPTIM/CORDIC…) and a closed enum reintroduces a per-family
   tax; interned `&str` (like `PoolId`) never needs widening. Recommend interned
   string for `Class` given it's a *selector*.
3. **Dual-core** — design a `domain` tag into `Res::Inst`/`Pool` now (cheap) or
   strictly single-core v1? Recommend: leave the slot, scope v1 single-core.

---

## 8. Red-team findings (verified against stm32-data, 1616 parts)

> **Status (2026-06-21): addressed.** A2 (dedup) ✅, B1 (bridge) ✅, B4 (`min_with_dma`
> cut) ✅, C3 (C531 keeps its own diagnostics) ✅. **A4 partial** — DMA demand is a
> per-kind channel count (2 serial / 1 ADC), not yet full signal-set cardinality, so
> a literal 7-stream HRTIM/TIM workload would undercount. **D1/D2/D3 superseded** by
> "save-compat not required" (clean break; the typed-`Pin`/`Serialize`/module-clash
> migration concerns no longer apply).

| # | Finding | Severity | Resolution (see §0) |
|---|---|---|---|
| A1 | FIXED-channel DMA covers ~700 parts (F0/1/2/3/4/7, L0/1/4); plan wrongly calls it out of scope; "request ignored" rule unsound there | fatal | Scope v1 to DMAMUX+NAMED; FIXED deferred as singleton-pool later |
| A2 | Dual-core `Σ cores[].dma_channels` double-counts (H745: 80 vs 40 real) — live in Inc 0 | fatal | Dedup by (controller,channel) in Inc 0 |
| A3 | Dual-core shared peripherals break `Res::Inst` identity (H7 USART1 on both cores) | high | Defer dual-core; `domain` tag slot |
| A4 | HRTIM/TIM expose 7 DMA signals each; `DmaNeed{None,Rx,Tx,RxTx}` undercounts the motivating converter workload | high | DMA demand = channel count / signal-set |
| A5 | Package-vs-die caps (G474RE SPI:3 vs VE SPI:4) | med | Bridge keys on package letter; caps are per-package in catalog |
| B1 | **`for_chip_name` exact-match reaches ~1 part/family → Tier-2 unreachable for ~99% of catalog** | **#1** | Robust suffix-stripping bridge (Inc 0.5) + 3-state labels |
| B3 | Pin exhaustion is a real selector outcome, not monotonic → Tier-2 only → amplifies B1 | high | Pin feasibility is Tier-2; honest bounds-only label |
| B4 | `min_with_dma` non-discriminating (100% serial carry DMA) | low | Cut from v1; capacity is the real constraint |
| C2 | Closed `Class` enum vs 24 catalog families = relocated per-family tax | med | Open decision #2 (interned str) |
| C3 | C531 `validate` returns ALL conflicts; backtracker returns one → minimal-unsat-core needed to project diagnostics | med | Keep C531's diagnostic pass; don't force-fold |
| D1/D2/D3 | typed `pinout::Pin`≠`PinId`; `SignalId` not Serialize; `solver.rs`/`solver/` name clash; cross-module struct moves | med | **Neutralized by save-compat-not-required** → clean break |

**The single highest-priority thing to resolve before code:** the catalog-part →
descriptor **bridge** (B1). Without it the only sound stage is unreachable for
~99% of parts, and bounds-only is unsound-upward for the FIXED (A1) and
multi-stream (A4) cases — so the selector would report incapable parts as
"plausible". Resolve the bridge + the DMA-model scope (A1/A4) + dual-core dedup
(A2) on paper before Inc 0.
