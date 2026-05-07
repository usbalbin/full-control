//! # hard-rt — Two-tier priority architecture for Cortex-M
//!
//! Tools for running a hard-real-time control loop on the same Cortex-M
//! core as a soft-async stack (RTIC + embassy + CAN/USB/LCD/whatever),
//! without the soft side being able to disturb the hard side.
//!
//! Provides:
//! - [`BasepriCriticalSection`]: a `critical_section::Impl` that uses
//!   BASEPRI instead of PRIMASK, so high-priority ISRs are never blocked
//!   by `critical_section::with()` calls in lower-priority code.
//! - [`demote_unmanaged_irqs`]: a one-shot helper that demotes every
//!   enabled NVIC IRQ still at its reset-default priority (0x00) down to
//!   the lowest priority level. Required so soft-side IRQs registered by
//!   HAL crates don't silently sit *above* RTIC's max user priority.
//!
//! # The architecture
//!
//! Cortex-M with 4 NVIC priority bits gives 16 levels (NVIC byte values
//! 0x00, 0x10, ..., 0xF0; numerically lower = higher priority).
//! `hard-rt` partitions them into two tiers:
//!
//! | tier | NVIC byte | what lives here |
//! |------|----------:|-----------------|
//! | **Tier 1** | 0x10 | the hard-real-time ISR (control loop, current trip handler, …) |
//! | **Tier 2** | 0x20 .. 0xF0 | every other ISR, every async task, every HAL crate |
//!
//! With this split, `BasepriCriticalSection` raises BASEPRI to **0x20**
//! on `acquire()`. That masks Tier 2 (NVIC 0x20..=0xF0) but leaves Tier
//! 1 (NVIC 0x10) free to fire. So `critical_section::with()` calls in
//! embassy_time, embassy_sync, defmt, smoltcp, USB stacks, etc., behave
//! exactly the way those crates expect — they exclude every actor that
//! could *legitimately* race on their internal state — without
//! incidentally blocking the hard-real-time tier that has nothing to do
//! with their state.
//!
//! NVIC 0x00 is reserved for system exceptions (NMI, HardFault, …) and
//! is **also** unmasked by this CS. That matches the cortex-m reset
//! default and is consistent with the "Tier 1 is the highest user
//! priority" rule.
//!
//! # Soundness contract — read carefully
//!
//! `BasepriCriticalSection` *deliberately* does not protect Tier 1 from
//! Tier 2. That is the entire point of the architecture, and it places a
//! correctness obligation on the user code:
//!
//! > **Invariant:** Tier 1 code MUST NOT access any non-atomic memory
//! > that Tier 2 code (any soft-side actor — ISR, task, executor) also
//! > accesses, except via lock-free primitives whose atomicity is
//! > established without `critical_section`.
//!
//! Concretely:
//! - Sharing via `core::sync::atomic::Atomic*` is fine. Reads and writes
//!   are atomic by hardware; no CS needed.
//! - Sharing via `heapless::spsc::Queue` (atomic head/tail indices) is
//!   fine, when the producer/consumer split lines up with the tier
//!   boundary.
//! - Sharing a `static mut`, `Cell<T>`, `RefCell<T>`, `Mutex<RefCell<T>>`
//!   between Tier 1 and Tier 2 is **unsound**, because this CS does not
//!   exclude Tier 1 from running during the Tier 2 access. The
//!   compiler will not catch this — Tier 1 is implemented in
//!   `extern "C"` ISRs and the borrow checker has no view across
//!   ISR boundaries.
//!
//! If you only ever share `&AtomicX` (or RTIC `#[shared]` resources
//! declared with `&` — which RTIC restricts to lock-free types), the
//! invariant is maintained by construction and the borrow checker
//! enforces it through type signatures.
//!
//! If you need to send a multi-word value across the boundary, use a
//! lock-free SPSC queue, or a "seqlock" (publisher writes a generation
//! counter atomic; reader spins until generation is stable). Do not
//! reach for a CS-protected mutex.
//!
//! # Why a custom impl is necessary
//!
//! The default `cortex-m/critical-section-single-core` impl uses
//! `cpsid i` / `cpsie i` to mask interrupts via **PRIMASK**. PRIMASK
//! masks every priority level except NMI and HardFault, regardless of
//! NVIC priority configuration. Any crate that calls
//! `critical_section::with()` therefore blocks every ISR — including
//! the highest-priority one — for the duration of the CS. On a
//! firmware with a 1 kHz async ticker (embassy_time), the control loop
//! sees several µs of phase jitter every time the time-driver re-arms
//! its compare register inside a CS.
//!
//! BASEPRI is the priority-aware alternative: setting BASEPRI to a
//! non-zero value masks priorities numerically equal to or greater than
//! the value, leaving lower-numerical (= higher-priority) interrupts
//! free to preempt. ARMv7-M Architecture Reference Manual B3.4.6.
//!
//! # Recommended usage
//!
//! In the firmware crate's `Cargo.toml`, ensure no other
//! `critical_section::Impl` is provided (in particular, drop the
//! `critical-section-single-core` feature on `cortex-m`):
//!
//! ```toml
//! cortex-m = { version = "0.7" }   # NO `critical-section-single-core`
//! hard-rt  = { path = "...", features = ["critical-section-impl"] }
//! ```
//!
//! In firmware init (e.g. RTIC's `#[init]`), call
//! [`demote_unmanaged_irqs`] *after* `embassy_stm32::init()` /
//! peripheral setup so any HAL-bound IRQs at default priority are
//! demoted into Tier 2:
//!
//! ```ignore
//! #[init]
//! fn init(cx: init::Context) -> (Shared, Local) {
//!     let _p = embassy_stm32::init(config);
//!     // ... peripheral setup ...
//!     hard_rt::demote_unmanaged_irqs();
//!     // ... spawn tasks ...
//! }
//! ```
//!
//! Configure your hard-real-time RTIC task at priority 15 (with
//! `prio_bits_4`, this maps to NVIC 0x10 = Tier 1):
//!
//! ```ignore
//! #[task(binds = ADC1_2, priority = 15, ...)]
//! fn control_loop(...) { ... }
//! ```
//!
//! All other tasks at lower RTIC priorities → Tier 2.
//!
//! # Limitations
//!
//! - 4-bit-priority Cortex-M only (M3 / M4 / M7). Cortex-M0/M0+ doesn't
//!   have BASEPRI; on those parts use a simpler "single hard-RT ISR"
//!   pattern with regular PRIMASK CS.
//! - The threshold (0x20) and the choice of NVIC 0x10 for Tier 1 is
//!   hard-coded. If your part has different priority bits or you need
//!   multiple hard-RT priorities, copy the impl into your crate and
//!   adjust the threshold; the principle generalises.
//! - `hard-rt::demote_unmanaged_irqs` reads/writes raw NVIC registers
//!   and assumes ARMv7-M layout. Safe to call multiple times; idempotent
//!   for IRQs already at non-zero priority.

#![no_std]

/// `critical_section::Impl` that uses BASEPRI to mask Tier 2 priorities,
/// leaving Tier 1 (NVIC 0x10) free to fire during the critical section.
///
/// # Safety contract (caller's responsibility)
///
/// As documented in the crate-level docs: any code at NVIC priority 0x10
/// (Tier 1) MUST NOT access non-atomic state shared with code at NVIC
/// priority 0x20..=0xF0 (Tier 2). This CS does *not* mask Tier 1, so
/// the invariant cannot be enforced by the CS itself — it must be
/// upheld by construction in user code.
///
/// Sharing via atomics / lock-free queues / RTIC `&AtomicX` `#[shared]`
/// resources upholds this invariant automatically.
///
/// Violating this invariant produces a data race between Tier 1 and
/// Tier 2 even though both believe they're protected. Such races are
/// undefined behaviour. The borrow checker cannot detect this because
/// Tier 1 ISRs are linked as `extern "C"` and the compiler has no
/// view across the ISR boundary.
pub struct BasepriCriticalSection;

/// BASEPRI value to raise to on `acquire()`. Masks NVIC priorities
/// numerically `>=` 0x20 (i.e. Tier 2). NVIC 0x10 (Tier 1) and 0x00
/// (NMI / HardFault / system exceptions) remain unmasked.
///
/// ARMv7-M ARM B3.4.6: BASEPRI != 0 boosts execution priority such that
/// exceptions of priority numerically `>=` BASEPRI are masked.
const BASEPRI_THRESHOLD: u8 = 0x20;

#[cfg(feature = "critical-section-impl")]
critical_section::set_impl!(BasepriCriticalSection);

unsafe impl critical_section::Impl for BasepriCriticalSection {
    /// Save the current BASEPRI, then raise it to [`BASEPRI_THRESHOLD`]
    /// (only if doing so would actually be more restrictive than the
    /// current value — important when CSes nest, or when the caller is
    /// already running at a NVIC priority numerically less than 0x20
    /// such as a Tier 1 ISR that itself called `critical_section::with`).
    ///
    /// `dsb`/`isb` ensure the BASEPRI write is observed by the
    /// architecture before any subsequent loads/stores in the protected
    /// region begin executing.
    ///
    /// # Safety
    ///
    /// Caller must uphold the soundness contract documented on
    /// [`BasepriCriticalSection`].
    unsafe fn acquire() -> critical_section::RawRestoreState {
        let saved = cortex_m::register::basepri::read();
        // Raise only if doing so increases restriction. BASEPRI = 0
        // means "no masking" (least restrictive), so always raise from
        // 0; otherwise pick the numerically-lower (= higher-priority,
        // = more-restrictive) of saved vs THRESHOLD.
        let new = if saved == 0 || BASEPRI_THRESHOLD < saved {
            BASEPRI_THRESHOLD
        } else {
            saved
        };
        unsafe { cortex_m::register::basepri::write(new) };
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        saved
    }

    /// Restore BASEPRI to the value saved by the matching `acquire()`
    /// call. `dsb` ensures stores in the protected region commit before
    /// BASEPRI is lowered (otherwise a Tier 2 ISR firing immediately on
    /// release could observe a partially-written shared state).
    ///
    /// # Safety
    ///
    /// `saved` must be the value returned by the matching `acquire()`.
    /// `critical_section::with()` enforces this pairing for users; only
    /// custom callers of the trait need to uphold it manually.
    unsafe fn release(saved: critical_section::RawRestoreState) {
        cortex_m::asm::dsb();
        unsafe { cortex_m::register::basepri::write(saved) };
        cortex_m::asm::isb();
    }
}

/// Demote every enabled NVIC IRQ still at its reset-default priority
/// (NVIC IPR byte = 0x00, the highest possible) down to 0xF0 (the
/// lowest priority level on a Cortex-M with 4 priority bits).
///
/// # Why this is needed
///
/// RTIC with `prio_bits_4` maps "priority 15" (its max user priority)
/// to NVIC byte 0x10 — *not* 0x00. NVIC 0x00 is left unused by RTIC,
/// but it is also the **reset default** of every IRQ priority register.
/// So every HAL-bound IRQ that nothing has explicitly configured sits
/// at NVIC 0x00, **above** anything RTIC manages.
///
/// Concretely on STM32G4 + embassy + RTIC, IRQs like the embassy
/// time-driver TIM, every DMA channel, every COMP, every HRTIM
/// sub-timer get bound by `embassy_stm32::init()` without their
/// priority being set. They all default to NVIC 0x00 — silently
/// preempting the highest-priority RTIC task and breaking RTIC's
/// BASEPRI lock invariant.
///
/// This helper walks all 240 NVIC IRQ slots, checks which are enabled,
/// and demotes any still at 0x00 down to 0xF0 (a Tier 2 priority below
/// every RTIC dispatcher). Any IRQ already at non-zero priority (RTIC's
/// own dispatchers, anything you set yourself, including a Tier 1 ISR
/// at NVIC 0x10) is left alone.
///
/// # When to call
///
/// Call exactly once, *after* HAL initialisation (which enables IRQs)
/// and *after* RTIC has set its task dispatcher priorities. The
/// natural place is at the end of the RTIC `#[init]` function, just
/// before spawning tasks.
///
/// # Idempotence
///
/// Safe to call multiple times. The "only touch IRQs at exactly 0x00"
/// rule means a second call has nothing to do. (If user code later
/// re-zeroes a priority, the next call will demote it again.)
///
/// # Safety
///
/// Internally uses raw memory-mapped writes to NVIC IPR (0xE000_E400)
/// and ISER (0xE000_E100). The function is marked safe because:
/// - The addresses are fixed by the ARMv7-M architecture and writing
///   them does not violate any aliasing guarantee.
/// - Writing a higher numerical value to NVIC IPR can only **lower**
///   an interrupt's priority. Any code that depends on an IRQ being
///   at a specific priority must set that priority explicitly (so it's
///   already non-zero) — this function will then leave it alone.
/// - Reads of ISER are non-mutating.
///
/// The function does *not* touch system exception priorities (SCB
/// SHPR1/2/3) — those are at fixed offsets outside the NVIC IPR range
/// and need separate handling if you want to demote SVCall / PendSV /
/// SysTick. By default they're at 0x00 (= highest), but they typically
/// only fire at well-defined moments under software control, so
/// leaving them at 0x00 is usually safe.
pub fn demote_unmanaged_irqs() {
    /// NVIC ISER (Interrupt Set-Enable Register) base. 8 contiguous
    /// 32-bit words covering 240 IRQs.
    const NVIC_ISER: usize = 0xE000_E100;
    /// NVIC IPR (Interrupt Priority Register) base. Byte-addressable;
    /// one byte per IRQ.
    const NVIC_IPR: usize = 0xE000_E400;
    /// Number of NVIC IRQ slots on ARMv7-M. Most parts use only a
    /// fraction; unused slots are not enabled and are skipped here.
    const NVIC_IRQ_COUNT: u32 = 240;
    /// Tier 2 lowest priority (= RTIC priority 1 with `prio_bits_4`).
    /// Anything demoted here cannot preempt any RTIC task.
    const DEMOTED_PRIORITY: u8 = 0xF0;

    for irq in 0..NVIC_IRQ_COUNT {
        // SAFETY: NVIC_ISER is a fixed architectural address. The read
        // is volatile and side-effect-free. We only test the IRQ-enabled
        // bit; we do not modify ISER.
        let enabled = unsafe {
            ((NVIC_ISER + (irq as usize / 32) * 4) as *const u32).read_volatile()
                & (1 << (irq % 32))
                != 0
        };
        if !enabled {
            continue;
        }
        let ipr = (NVIC_IPR + irq as usize) as *mut u8;
        // SAFETY: NVIC_IPR + irq is a fixed architectural byte address.
        // We only modify the byte if its current value is exactly the
        // reset default (0x00). Any deliberately-set priority is
        // preserved, so no caller's invariant about an IRQ's priority
        // is violated. The write is volatile.
        if unsafe { ipr.read_volatile() } == 0x00 {
            unsafe { ipr.write_volatile(DEMOTED_PRIORITY) };
        }
    }
}
