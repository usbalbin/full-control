//! Runtime descriptor asset — per-package-letter pin/AF + DMA data for the whole
//! STM32 lineup, generated from stm32-data chip JSON by the `gen_descriptors`
//! tool and parsed on demand. This is how the constraint selector verifies
//! (Tier-2) parts far beyond the handful with compiled `mcu_data`: ~870 distinct
//! package-letter descriptors would be ~9 MB of generated Rust statics
//! (brutal to compile) — as a data asset they're parsed once at runtime instead.
//!
//! Field names are deliberately short to keep the JSON small. Stage 1 (this
//! module + the generator) just defines + produces the asset; the runtime loader
//! that turns it into `&'static RawMcuData` lands in a later stage.

use serde::{Deserialize, Serialize};

/// One peripheral instance: name, address, metapac register block, the pin/AF
/// rows, and the normalized DMA legs (signal -> allowed controller pools).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AssetPeri {
    pub n: String,
    pub a: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub b: Option<String>,
    /// (pin, signal, af) — e.g. ("PA9", "TX", Some(7)).
    pub pins: Vec<(String, String, Option<u8>)>,
    /// (signal, allowed controller pools) — e.g. ("RX", ["DMA1","DMA2"]).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub dma: Vec<(String, Vec<String>)>,
}

/// One package-letter descriptor, e.g. prefix "STM32G474R" = the LQFP64 G474
/// pinout. The catalog-part -> descriptor bridge keys on `prefix`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AssetPart {
    /// "STM32<line><letter>" (first 10 chars of the representative name).
    pub prefix: String,
    /// Representative full part name, e.g. "STM32G474RE".
    pub name: String,
    /// Family, e.g. "STM32G4".
    pub family: String,
    /// Datasheet package style, e.g. "LQFP64".
    pub package: String,
    pub peris: Vec<AssetPeri>,
    /// DMA controller pools (supply): (controller, channel count).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pools: Vec<(String, u8)>,
}
