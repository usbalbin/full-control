//! Family-agnostic resource tokens for the Capability Kernel.
//! See `docs/constraint-selector-design.md` §2.
//!
//! A token is an *opaque exclusion unit*: the engine ([`super::engine`]) only
//! ever does set membership on it (plus a capacity count that the CALLER encodes
//! as N distinct [`Res::Pool`] supply tokens). The engine MUST NOT pattern-match
//! a [`Res`] to make a decision — that invariant is what lets a new MCU family
//! be added without minting a single new token variant.

use crate::mcu_pinout::PinId;

/// Peripheral class. An interned metapac class string (`"USART"`, `"SPI"`,
/// `"TIM"`, `"COMP"`, …) rather than a closed enum: the catalog spans ~24
/// families (SAI/SDMMC/LPTIM/CORDIC/…) and a closed enum would reintroduce a
/// per-family tax every time one is added. `Class` is only ever a *selector*
/// (compared for equality, hashed) — never matched — so a string is sufficient.
/// (Design doc open-decision #2: interned string, recommended.)
pub type Class = &'static str;

/// A DMA controller pool id, e.g. `"DMA1"`, `"GPDMA1"`, `"LPDMA2"`. Interned for
/// the same reason as [`Class`].
pub type PoolId = &'static str;

/// An internal analog/timer fabric edge kind — the `ChipFabric` routes the
/// hardware-OCP and analog-sense requirements lower into.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum RouteKind {
    /// A DAC channel driving a comparator's threshold input.
    DacToComp,
    /// A comparator wired into a timer's break (BRK/BKIN) input.
    CompToTimBreak,
    /// A comparator wired into an HRTIM external-event (EEV) input.
    CompToEev,
    /// A comparator wired into an HRTIM fault (FLT) input.
    CompToFlt,
    /// The ADC input crossbar / OPAMP-follower path feeding an ADC channel.
    CrossbarToAdc,
}

/// An opaque exclusion token. Equality + hash are the *only* operations the
/// engine performs on it.
///
/// The numeric fields are deliberately untyped (`u8`s, not enums): they identify
/// *which* instance/channel/edge, and two tokens collide iff every field matches.
/// What a field *means* is the adapter's concern, never the engine's.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum Res {
    /// A whole peripheral instance: `Inst("USART", 2)` = USART2, `Inst("TIM", 1)`
    /// = TIM1, `Inst("COMP", 3)` = COMP3.
    Inst(Class, u8),
    /// A sub-channel of an instance: an ADC channel, a DAC channel, or an opaque
    /// HRTIM compare-slot — `SubChannel("ADC", 1, 5)` = ADC1 channel 5. HRTIM's
    /// RM0440 compare-slot / DEM accounting is quarantined here: the kernel never
    /// interprets the third field, it just keeps it exclusive.
    SubChannel(Class, u8, u8),
    /// A `ChipFabric` edge, e.g. `Route(CompToTimBreak, comp, tim, brk)`. Holding
    /// it reserves that specific internal connection.
    Route(RouteKind, u8, u8, u8),
    /// A physical pin — the cross-fabric conflict token (two peripherals wanting
    /// the same pin collide regardless of which fabric routed them there).
    Pin(PinId),
    /// ONE DMA channel of a controller. A pool of capacity N is modeled by the
    /// adapter minting N distinct supply tokens `Pool(id, 0)`..`Pool(id, N-1)`;
    /// each demand claims one, so "demand ≤ capacity" falls out of single
    /// ownership — the engine needs no special pool arithmetic.
    Pool(PoolId, u8),
}
