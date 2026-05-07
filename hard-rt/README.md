# hard-rt

Two-tier priority architecture for Cortex-M: keep hard-real-time ISRs free of
soft-async crate critical sections.

## Problem

You're running RTIC + embassy (or any async stack) on a Cortex-M with priority
bits, with one ISR that's hard-real-time (a control loop, a current trip
handler, a switching-converter compensator — anything where late = bug).

Two things keep biting you:

1. **Crates leave NVIC priorities at default 0x00**, which is *above* RTIC's
   max user priority (NVIC 0x10 with `prio_bits_4`). embassy's time-driver TIM
   IRQ, DMA channel IRQs, etc. all preempt your "highest priority" RTIC task.
2. **The default `cortex-m/critical-section-single-core` impl uses PRIMASK**,
   which masks every interrupt regardless of NVIC priority. Every
   `critical_section::with(...)` call in `embassy_time`, `embassy_sync`,
   `defmt`, etc. blocks your control loop for the duration of the CS.

Both are silent failure modes — exec time looks fine, but phase relative to
the trigger drifts by microseconds.

## Solution

- Provide a custom `critical_section::Impl` that uses **BASEPRI** (priority-aware)
  instead of PRIMASK. Threshold: 0x20. Tier 1 (NVIC 0x10) is never blocked.
- Provide `demote_unmanaged_irqs()` to demote any enabled IRQ still at NVIC 0x00
  down to 0xF0, restoring RTIC's invariant that nothing user-mode is above its
  hierarchy.

Together these enable:

| tier | NVIC byte | what lives here |
|------|----------:|-----------------|
| **Tier 1** | 0x10 | the hard-real-time ISR (control loop, current trip, …) |
| **Tier 2** | 0x20 .. 0xF0 | every other ISR, every async task, every HAL crate |

## Usage

```toml
# Cargo.toml
[dependencies]
cortex-m = "0.7"  # NOTE: do NOT enable `critical-section-single-core`
hard-rt  = { path = "../hard-rt", features = ["critical-section-impl"] }
```

```rust
// In RTIC #[init], after embassy_stm32::init() / peripheral setup:
hard_rt::demote_unmanaged_irqs();
```

That's it. The custom CS impl is registered globally via the
`critical-section-impl` feature gate.

## Soundness

The custom CS deliberately does **not** mask Tier 1. The contract: Tier 1
must only access state shared with Tier 2 via atomics or lock-free queues
(SPSC, seqlock). Sharing a `static mut` / `Cell` / `RefCell` between Tier 1
and Tier 2 is a data race regardless of any CS — see the crate-level docs
in `src/lib.rs` for the full invariant.

For RTIC `#[shared]` resources between Tier 1 and Tier 2, declare them as
`&AtomicX` — the borrow checker will then enforce the invariant through type
signatures.

## Limitations

- ARMv7-M only (Cortex-M3 / M4 / M7). Cortex-M0 / M0+ has no BASEPRI.
- Hard-coded to 4 NVIC priority bits and a Tier 1/Tier 2 split at NVIC 0x20.
  If you need different bits or multiple Tier 1 priorities, copy the impl into
  your own crate and adjust the threshold.

## Background reading

- ARMv7-M Architecture Reference Manual §B3.4.6 (BASEPRI semantics).
- RTIC book, "Resources" chapter (the BASEPRI-based lock model).
- The fw/perf.md doc in the hw-half-bridge project (the empirical journey
  from "5.96 µs WCET, 4 µs phase jitter" to "1.45 µs WCET, 0.21 µs phase
  jitter" using these tools).
