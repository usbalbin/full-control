//! Family-neutral user intent — the *input* to the selector, lowered onto a
//! concrete chip by [`super::lower`]. See `docs/constraint-selector-design.md` §2.
//!
//! Increment 4 subset: only [`ReqKind::UsePeripheral`] (the DMA-first end-to-end
//! path). `HardwareOcp` / `AdcSense` / `Hrtim` intents arrive in later increments
//! as the fabric-route and HRTIM adapters land.

use super::tokens::Class;

/// How many DMA channels a peripheral use needs (async-serial TX+RX = 2; 0 = no
/// DMA). The doc's per-signal channel demand reduced to a count — a count, unlike
/// the rejected `{None,Rx,Tx,RxTx}` enum, never undercounts a multi-stream
/// peripheral (HRTIM/TIMx can need up to 7).
pub type DmaDemand = u8;

/// A single unit of user intent.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReqKind {
    /// One instance of `class`, optionally backed by `dma` DMA channels drawn
    /// from that instance's allowed controller pools. `pinned` fixes it to a
    /// specific instance index (the user wants *USART2* specifically); `None`
    /// lets the solver choose any free instance. Pinning is what makes the
    /// selector path need backtracking — a flexible use can otherwise greedily
    /// take the instance a pinned use requires.
    UsePeripheral { class: Class, dma: DmaDemand, pinned: Option<u8> },
}

/// A user requirement with a stable id (so an infeasibility can name exactly
/// which intents went unmet).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Requirement {
    pub id: u32,
    pub kind: ReqKind,
}

impl Requirement {
    /// "Use any free `class` instance, with `dma` DMA channels (0 = none)."
    pub fn use_peripheral(id: u32, class: Class, dma: DmaDemand) -> Self {
        Self { id, kind: ReqKind::UsePeripheral { class, dma, pinned: None } }
    }

    /// "Use `class` instance `inst` specifically, with `dma` DMA channels."
    pub fn use_peripheral_pinned(id: u32, class: Class, dma: DmaDemand, inst: u8) -> Self {
        Self { id, kind: ReqKind::UsePeripheral { class, dma, pinned: Some(inst) } }
    }
}
