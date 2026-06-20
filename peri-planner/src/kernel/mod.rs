//! The Capability Kernel — a chip-agnostic claim/conflict engine over opaque
//! numeric resource tokens. See `docs/constraint-selector-design.md` §1–3 for
//! the full architecture, and `MEMORY`/`project_constraint_selector` for status.
//!
//! **Increment 3 (this module):** the token vocabulary ([`tokens`]) + the claim/
//! conflict engine ([`engine`]). Library-only — no UI, no data regen, no
//! persistence. The whole point is that the token vocabulary *stops growing per
//! family*: the family-specific models (G474 `Design`, `C531Design`, the generic
//! H523 pin-lock design) become thin [`Consume`] adapters in later increments,
//! and adding a new MCU mints zero new [`Res`] variants.
//!
//! Increment 4 ([`requirement`] + [`lower`]): the family-neutral `Requirement`
//! intent layer and a DMA-aware enumerator that lowers a requirement set onto a
//! real `McuDescriptor` (`Inst` + `Pool` tokens) and runs the engine to a
//! feasibility verdict + witness — the first end-to-end solve on chip data.
//! Greedy for now (sound for feasible answers, not yet complete).
//!
//! Not yet built (later increments): bounded backtracking for completeness
//! (Inc 5), generic pin contention via `mcu_pinout` (Inc 6), two-tier catalog
//! wiring + a requirements panel (Inc 7), `HardwareOcp` via fabric routes (Inc
//! 8), and folding G474 + C531 in as adapters with the legacy `solver.rs`
//! retired (Inc 9).

pub mod engine;
pub mod lower;
pub mod requirement;
pub mod tokens;

pub use engine::{place_greedy, used_excluding, Consume, Ledger};
pub use lower::{feasible, Outcome, Placed};
pub use requirement::{DmaDemand, ReqKind, Requirement};
pub use tokens::{Class, PoolId, Res, RouteKind};
