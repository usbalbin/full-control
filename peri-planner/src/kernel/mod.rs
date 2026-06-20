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
//! Not yet built (later increments): the family-neutral `Requirement`/intent
//! layer (Inc 4), per-instance DMA pool/route data lowered from descriptors
//! (Inc 1/4), bounded backtracking on the selector path (Inc 5), and wiring G474
//! + C531 in as adapters with the legacy `solver.rs` retired (Inc 9).

pub mod engine;
pub mod tokens;

pub use engine::{place_greedy, used_excluding, Consume, Ledger};
pub use tokens::{Class, PoolId, Res, RouteKind};
