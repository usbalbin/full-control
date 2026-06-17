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

use std::collections::HashMap;
use std::io::Read as _;
use std::sync::LazyLock;

use serde::{Deserialize, Serialize};

use crate::mcu::{build_asset_descriptor, McuDescriptor};
use crate::mcu_raw::{DmaPoolDef, RawDmaLeg, RawMcuData, RawPeripheral, RawPin};

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

// ---------- Runtime loader ----------

/// Leak an owned string to `&'static str`. The descriptor asset lives for the
/// whole session (like the catalog), so leaking once at init is the simplest way
/// to hand the `&'static`-based `RawMcuData` / `mcu_pinout` engine its data.
fn leak_str(s: String) -> &'static str {
    Box::leak(s.into_boxed_str())
}

/// Turn one parsed [`AssetPart`] into a leaked `&'static RawMcuData` the existing
/// descriptor / pin engine can consume unchanged.
fn leak_raw(p: AssetPart) -> &'static RawMcuData {
    let peripherals: Vec<RawPeripheral> = p
        .peris
        .into_iter()
        .map(|pe| {
            let pins: Vec<RawPin> = pe
                .pins
                .into_iter()
                .map(|(pin, signal, af)| RawPin { pin: leak_str(pin), signal: leak_str(signal), af })
                .collect();
            let dma: Vec<RawDmaLeg> = pe
                .dma
                .into_iter()
                .map(|(signal, pools)| {
                    let pools: Vec<&'static str> = pools.into_iter().map(leak_str).collect();
                    RawDmaLeg { signal: leak_str(signal), pools: Box::leak(pools.into_boxed_slice()) }
                })
                .collect();
            RawPeripheral {
                name: leak_str(pe.n),
                address: pe.a,
                block: pe.b.map(leak_str),
                pins: Box::leak(pins.into_boxed_slice()),
                triggers: &[],
                dma: Box::leak(dma.into_boxed_slice()),
            }
        })
        .collect();
    let dma_pools: Vec<DmaPoolDef> = p
        .pools
        .into_iter()
        .map(|(name, channels)| DmaPoolDef { name: leak_str(name), channels })
        .collect();
    Box::leak(Box::new(RawMcuData {
        name: leak_str(p.name),
        family: leak_str(p.family),
        peripherals: Box::leak(peripherals.into_boxed_slice()),
        dma_pools: Box::leak(dma_pools.into_boxed_slice()),
    }))
}

/// Per-package-letter descriptors for the WHOLE lineup, built lazily once from
/// the embedded gzipped asset (inflate -> parse -> leak). Keyed by the
/// "STM32<line><letter>" prefix. Powers the selector's Tier-2 over every part,
/// far beyond the handful with compiled `mcu_data`.
fn registry() -> &'static HashMap<&'static str, &'static McuDescriptor> {
    static REG: LazyLock<HashMap<&'static str, &'static McuDescriptor>> = LazyLock::new(|| {
        let gz: &[u8] = include_bytes!("../assets/descriptors.json.gz");
        let mut json = Vec::new();
        flate2::read::GzDecoder::new(gz)
            .read_to_end(&mut json)
            .expect("inflate descriptors.json.gz");
        let parts: Vec<AssetPart> =
            serde_json::from_slice(&json).expect("parse descriptors.json.gz");
        let mut m = HashMap::with_capacity(parts.len());
        for p in parts {
            let prefix = leak_str(p.prefix.clone());
            let raw = leak_raw(p);
            let desc: &'static McuDescriptor = Box::leak(Box::new(build_asset_descriptor(raw)));
            m.insert(prefix, desc);
        }
        m
    });
    &REG
}

/// The descriptor for the part `name`, matched by package-letter prefix (every
/// flash/temp/package variant of a letter shares the descriptor). `None` only if
/// the lineup asset somehow lacks the prefix.
pub fn descriptor_for(name: &str) -> Option<&'static McuDescriptor> {
    let reg = registry();
    if name.len() >= 10 {
        if let Some(d) = reg.get(&name[..10]) {
            return Some(d);
        }
    }
    reg.iter().find(|(pre, _)| name.starts_with(**pre)).map(|(_, d)| *d)
}

/// Number of loaded descriptors (test/diagnostic).
pub fn descriptor_count() -> usize {
    registry().len()
}
