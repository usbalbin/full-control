# Using the generated firmware scaffold

peri-planner's **Gen firmware** button (top bar, next to *Export*) copies a Rust
module to the clipboard: per-peripheral **resource bundles** + a **`Behavior`**
config struct for the active design. This is the user-facing companion to
[`firmware-codegen-design.md`](firmware-codegen-design.md); read that for the
*why*, this for the *how*.

## Mental model

peri-planner generates **resources, not init.** Each peripheral becomes a bundle
of typed embassy singletons (its instance + role-named pins + any DMA channel).
**You** write the driver constructors against those bundles, in your own
embassy-stm32 version's idiom. Why this split:

- embassy's init API churns; the bundle boundary (`peripherals::X` + `Peripherals`
  field access) doesn't — so the generated code ~never breaks on an embassy bump.
- embassy constructors are generic over pin *marker traits* (`TxPin<USART1>`,
  `Channel1Pin<TIM1>`). So when you **re-pin in peri-planner and regenerate**,
  only the bundle field *types* change — the field *names* stay — and your
  constructor calls still compile **unchanged**.

So: regenerate freely; never hand-edit the generated module; keep your init code
separate.

## 1. Drop it into your firmware crate

Paste the clipboard into e.g. `src/generated.rs` and add `mod generated;`. Treat
it as a build artifact (regenerate, don't edit). The module is `#![allow(dead_code)]`
so unused bundles/fields don't warn.

## 2. Take the board, once, in `init`

```rust
let p = embassy_stm32::init(Default::default());
let b = generated::Board::take(p);   // every instance + pin, named by function
```

`Board::take` destructures embassy's `Peripherals` into the named bundles. After
this, `p` is consumed and you work with `b`.

## 3. Fill the `Behavior` holes (the values the plan can't know)

The generator emits a `Behavior` struct with one field per behavioral value
(baud, PWM frequency, dead-time …) and a `Default`. Author your values in a
**separate** file the generator never touches:

```rust
// app/buck_cfg.rs  — yours
use generated::Behavior;
pub const BUCK: Behavior = Behavior {
    tim1_freq_hz: 200_000,
    tim1_dead_time_ns: 80,
};
```

Adding a routed peripheral in peri-planner adds a `Behavior` field — so your
config fails to compile until you supply it. The design and its config can't
silently drift.

## 4. Worked example — a C531 synchronous buck

For a leg of TIM1 CH1/CH1N (complementary + dead-time), an ADC1 sense channel
with DMA, and a hardware over-current trip (COMP1 → TIM1 break, DAC1 threshold),
**Gen firmware** produces (abridged):

```rust
pub struct Tim1  { pub instance: peripherals::TIM1, pub ch1: peripherals::PA8, pub ch1n: peripherals::PA7 }
pub struct Adc1  { pub instance: peripherals::ADC1, pub in1: peripherals::PA1, pub stream_dma: peripherals::LPDMA1_CH0 }
pub struct Comp1 { pub instance: peripherals::COMP1 }   // internal fabric — no pins
pub struct Dac1  { pub instance: peripherals::DAC1 }    // internal fabric — no pins
pub struct Board { pub tim1: Tim1, pub adc1: Adc1, pub comp1: Comp1, pub dac1: Dac1 }
pub struct Behavior { pub tim1_freq_hz: u32, pub tim1_dead_time_ns: u16 }
```

You then write the init — these calls are **illustrative; match your embassy
version's exact signatures** (that's the churny part peri-planner deliberately
leaves to you):

```rust
let b   = generated::Board::take(embassy_stm32::init(Default::default()));
let cfg = buck_cfg::BUCK;

// Complementary PWM half-bridge. Pins come from the bundle; freq/dead-time from cfg.
let hs = PwmPin::new_ch1(b.tim1.ch1, OutputType::PushPull);
let ls = ComplementaryPwmPin::new_ch1(b.tim1.ch1n, OutputType::PushPull);
let mut pwm = ComplementaryPwm::new(
    b.tim1.instance, Some(hs), Some(ls), None, None, None, None, None, None,
    Hertz(cfg.tim1_freq_hz), Default::default(),
);
pwm.set_dead_time(/* ns -> ticks from cfg.tim1_dead_time_ns */);

// ADC with a DMA stream — the channel singleton is in the bundle.
let mut adc = Adc::new(b.adc1.instance);
// ... ring-buffered / async read using b.adc1.stream_dma and b.adc1.in1 ...

// Hardware OCP is internal silicon (no pins): raw register config using the
// COMP1/DAC1/TIM1 instances the bundle handed you (Tier-3 — embassy has no
// driver for the COMP->timer-break fold).
let _ = (b.comp1.instance, b.dac1.instance, &b.tim1.instance);
// pac::COMP1.csr().modify(...); pac::TIM1.bdtr().modify(|w| w.set_bke(true)); ...
```

## 5. Interrupts

DMA channels live in the bundle, but **`bind_interrupts!` needs literal IRQ
idents** (it's a macro), so IRQs aren't bundled — you write that block yourself.
IRQ names track the peripheral *instance*, not the pins, so re-pinning never
changes it:

```rust
bind_interrupts!(struct Irqs { ADC1_2 => adc::InterruptHandler<peripherals::ADC1>; });
```

## 6. RTIC integration

embassy is just the HAL; build the drivers in RTIC `#[init]` from the bundle and
store the constructed drivers (not the bundle) in `#[local]`/`#[shared]`:

```rust
#[init]
fn init(cx: init::Context) -> (Shared, Local) {
    let b = generated::Board::take(embassy_stm32::init(Default::default()));
    let pwm = /* ... build from b.tim1 + BUCK ... */;
    (Shared { /* ... */ }, Local { pwm })
}
```

## 7. The re-pin loop

1. Move a pin / add a peripheral in peri-planner.
2. **Gen firmware** → replace `generated.rs`.
3. Rebuild. Bundle field *types* changed; field *names* didn't; embassy
   constructors are generic over the pin — **your init code is unchanged.** A new
   `Behavior` field (if you added a peripheral) is the only thing that needs
   attention, and it fails to compile until filled.

## What is NOT generated (by design)

- **Driver constructors / init** — version-churny; yours to write (§4).
- **`bind_interrupts!`** — macro needs literal IRQ idents (§5).
- **Behavioral values** — baud/freq/dead-time/filters/thresholds are `Behavior`
  holes (§3); the bundles carry zero behavioral data.
- **Comms DMA** — only ADC streams get an auto-assigned channel today (a
  converter's ADC is universally DMA-driven); comms DMA is a future opt-in.
