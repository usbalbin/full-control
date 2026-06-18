# Declare-once: per-family seed mappings (for review)

Status: **design / for review — not implemented.** This is the "actual combine"
the part-finder ↔ allocator unification reduces to: declare peripherals once (as
finder *demands*), then **seed** the allocator on the chosen chip, and
**summarize** an allocation back into demands for re-search.

The shared spec already exists: the finder's `Vec<DemandInput>` over logical
kinds `SERIAL / SPI / I2C / ADC / UCPD / OCP / COMP_PWM`
(`select.rs::DemandInput{kind, count, with_dma, options}`). The only new code is
two lowerings per family — **forward** (Demand → model) and **backward**
(model → Demand) — plus a `[Seed ▸]` / `[⟳ re-search]` button. No engine
changes, no model merge.

The forward lowering is **lossy** (a coarse count must become concrete
instances/roles), and lossy in *different* ways per family. The decisions below
are the whole reason this is "for review" — none are mechanical.

---

## Forward seed: Demand → model

### G474 (`Design.add(RequirementSpec)`)

The richest target. The `comms_palette` / `add_palette` presets are the seed
shapes; each carries a default instance the solver then (re)assigns.

| Demand kind (+options) | Seeded `RequirementSpec` (×count) | Notes / loss |
|---|---|---|
| SERIAL (+CTS/RTS, +CK) | `UseUsart{ instance: Usart1.., flow_control: has(CTS/RTS), synchronous: has(CK) }` | **Decision G1:** SERIAL is USART∪UART∪LPUART — seed which? Recommend **USART** (the superset; user switches a row to UART/LPUART via the existing combo). |
| SPI (+NSS) | `UseSpi{ instance: Spi1.., needs_miso: true, needs_nss: has(NSS) }` | `needs_miso` defaults true (full-duplex); half-duplex is a later option. |
| I2C (+SMBA) | `UseI2c{ instance: I2c1.., needs_smba: has(SMBA) }` | — |
| UCPD | `UseUcpd{ instance: Ucpd1.. }` | — |
| ADC (+dma) | `AdcSequencer{ Adc1, DualRegular, trigger: Mcr1 }` once **+** `AdcConversion{ group, VOut, … }` ×count | **Decision G2:** the finder's "ADC" is just count+1-stream; G474's ADC is a sequencer+conversions graph. Seed one default sequencer and N conversions under it? Or N bare `UseTim`-less placeholders? Recommend **one DualRegular sequencer + N conversions** (matches how a power design actually uses ADC). Lossy: trigger/speed/dual-pref are guessed. |
| COMP_PWM | **Decision G3 (the big one)** — see below | — |
| OCP | folded into COMP_PWM's `fault`, or a standalone fault role | **Decision G4** — see below |

**Decision G3 — COMP_PWM on G474: TIM or HRTIM?** The finder's `comp_pwm_ch`
metric counts **advanced/GP-timer** CHx/CHxN pairs (TIM1/8/20/15/16/17), *not*
HRTIM. But a G474 DC/DC design's complementary PWM is almost always **HRTIM**
(the converter core; that's the whole `hrtim_palette`). So there are two honest
seeds and they disagree:
  - (a) `UseTim{ instance: Tim1, complementary: true, channels_mask, … }` —
    *matches the finder metric* the user filtered on, but is the less likely
    intent on G474.
  - (b) `UseHrtimSub{ role: PcmInternal, outputs: Ch1AndCh2, … }` ×count —
    *matches the likely intent*, but is a different resource than the finder
    verified (HRTIM isn't in `comp_pwm_ch`).
  - **Recommendation:** seed **(a) UseTim** (honest to what the finder proved),
    and surface a one-click "convert to HRTIM" since the user is in the planner
    anyway. Do **not** silently seed HRTIM — it would claim a resource the finder
    never checked.

**Decision G4 — OCP on G474.** A `COMP→timer-break` trip maps to the `fault`
field of a `UseHrtimSub` (HRTIM fault) or `bkin_comp` on a `UseTim`. If COMP_PWM
seeded as `UseTim` (G3a), seed OCP as that tim's `bkin_comp: Some(comp)`. If
standalone, seed `ShortCircuitFault`. Recommend: **attach OCP to the COMP_PWM
seed when both are demanded** (one protected leg), else `ShortCircuitFault`.

### C531 (`C531Design.legs: Vec<ConverterLeg>`)

C531 is a **converter-leg planner** — it models *only* PWM legs + their analog
routes. Non-converter kinds have nowhere to live.

| Demand kind | Seed | Notes / loss |
|---|---|---|
| COMP_PWM (×N) | `N × ConverterLeg::pwm(Tim1/Tim8…)`, `complementary: true, dead_time: true` | reuses the default-leg logic already in `apply_converter_action` (picks `TimId` advanced timers). |
| OCP (×N) | set `ocp: Some(Ocp{ comp, break_input, threshold_dac: fabric-routable })` on the seeded legs | reuses the fabric-routable comp/dac the converter view already computes. |
| ADC | `adc_sense: Some((Adc1, ch))` on a leg | — |
| **SERIAL / SPI / I2C / UCPD** | **DROPPED** (with a visible note) | **Decision C1:** C531's model has no comms/pin-lock store at all (`C531Design` = legs only). A SERIAL demand can't seed anything. Options: (i) drop + note "comms not modeled for C531 — use Pin/AF to place"; (ii) give C531 a `pin_locks` field like H523. Recommend **(i) for now**, **(ii)** as a later unification step (it also helps the drop-in finder, whose `DesignSource::C531` is currently empty). |

### H523 / C5A3 (`H523Design.pin_locks: Vec<PinLock>`)

A flat pin-lock list, no peripheral-feature model. Seed **unplaced** locks the
user then binds in Pin/AF.

| Demand kind | Seed | Notes / loss |
|---|---|---|
| SERIAL (×N) | `N ×` pairs `PinLock{ USART1.., "TX"/"RX", pin: <unplaced> }` | **Decision H1:** `PinLock` requires a concrete pin (`port,num`) — there is no "unplaced" state today. Either add an `Option<pin>` (small model change) or seed nothing and just pre-select the peripheral in the Pin/AF picker. Recommend **add `Option<PinId>` to PinLock** (lets the spec carry intent without forcing a pin; also cleans up the picker). |
| SPI / I2C / UCPD | analogous role sets (SCK/MOSI/MISO, SCL/SDA, CC1/CC2) | — |
| ADC | analog (no AF pin) — seed nothing, note "ADC channels are analog, pick in Inventory" | ADC has no GPIO to lock. |
| OCP / COMP_PWM | CHx/CHxN pin-lock placeholders; OCP has no pins → note only | H523 has no OCP/converter model; partial. |

---

## Backward summarize: model → Demand (for re-search)

Walk the active model, bucket consumed peripherals by `select::kind_of`, set
`DemandInput.count`. The bucketing already exists for the G474 resource panel
(`design_used`). Lossy by construction — it drops pins, instance identity, HRTIM
roles, converter grouping, and connections; it recovers only the coarse counts,
which is exactly what the finder consumes. Round-tripping is therefore **not**
idempotent at full fidelity (forward then backward loses the planner detail) —
acceptable and expected for a "what else in the lineup fits this shape" search.

---

## Instance-numbering decision (all families)

A coarse `SERIAL×3` must become 3 concrete instances. **Decision N1:** seed
**sequential distinct** instances (USART1, USART2, USART3) as a starting guess,
not three copies of the palette default — the solver/normalize will reassign on
G474, but H523 pin-locks and the C531 legs need distinct concrete instances up
front. Where the demanded count exceeds the chip's instances, seed up to the
max and flag the overflow (the finder already proved feasibility, so this only
bites if the user seeds onto an under-provisioned part they opened read-only).

---

## DMA option

The finder's `with_dma` is a capacity demand. G474's `Design` models DMA in the
solver (so `with_dma` informs nothing extra at seed time — the solver allocates).
H523/C531 don't track DMA in their models. **Decision D1:** `with_dma` does not
seed anything; it stays a finder-only capacity filter. (A future generic spec
could carry it; not now.)

---

## Increment plan (each green, only after the decisions above are settled)

1. **Spec carry-over on jump.** The finder already returns the clicked part; also
   stash the active `Vec<DemandInput>` so the open lands with the demands in
   hand. (No seeding yet — just plumbing.)
2. **`[Seed ▸]` per family** — three small `seed_*` functions implementing the
   tables above. Tests: `SERIAL×3` seeds 3 serial rows / locks / (C531 note).
3. **`[⟳ re-search]`** — backward summary + jump to the finder.
4. **Shared spec strip** — lift the demand grid into the top bar so it's visible
   in finder + allocator alike (pure relocation).
5. *(Optional)* the two model changes flagged: `Option<PinId>` on `PinLock`
   (H1), and `pin_locks` on `C531Design` (C1) — each independently useful.

## The decisions to sign off, in one list

- **G1** SERIAL seeds as USART (recommended) / UART / LPUART?
- **G2** ADC seeds as one sequencer + N conversions (recommended) / something leaner?
- **G3** COMP_PWM on G474 seeds as **UseTim** (recommended, honest to the metric) / **HRTIM** (likely intent)?
- **G4** OCP attaches to the COMP_PWM leg (recommended) / standalone fault?
- **C1** C531 drops non-converter demands with a note (recommended now) / gets a pin-lock store (later)?
- **H1** add `Option<PinId>` to `PinLock` so seeds can be unplaced (recommended) / pre-select in picker only?
- **N1** sequential distinct instances (recommended) / palette-default copies?
- **D1** `with_dma` stays finder-only (recommended) / carried into the spec?
