//! Shared chip-JSON DMA extraction for the generators.
//!
//! stm32-metapac drops STM32C5 DMA (the data is in stm32-data's per-chip JSON
//! but metapac-gen doesn't propagate it), so both the compiled descriptors
//! (`extract.rs`, for the C5 families) and the lineup asset (`gen_descriptors.rs`)
//! source DMA from the chip JSON via this one module. Keeping the normalization
//! here means the DMAMUX / named-controller / fixed-channel collapse lives in
//! exactly one place.

use std::collections::{BTreeMap, BTreeSet};

use serde_json::Value;

/// DMA extracted from one chip's JSON.
pub struct ChipDma {
    /// Controller pool → its channel singleton names (e.g. `LPDMA1` →
    /// `{LPDMA1_CH0, …}`), deduped across cores. The count is `.len()`.
    pub pools: BTreeMap<String, BTreeSet<String>>,
    /// Peripheral name → (signal → allowed controller pools), DMAMUX fan-out
    /// already resolved. First occurrence of a peripheral name wins (dual-core
    /// parts list a peripheral under each core). Empty-pool signals are retained;
    /// callers filter them when emitting.
    pub legs: BTreeMap<String, BTreeMap<String, BTreeSet<String>>>,
}

/// Parse the DMA model out of one chip's stm32-data JSON.
pub fn extract(v: &Value) -> ChipDma {
    // Supply: controller channel counts + the DMAMUX → {controllers} fan-out.
    let mut pools: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    let mut mux_to_ctrls: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for core in v["cores"].as_array().into_iter().flatten() {
        for ch in core["dma_channels"].as_array().into_iter().flatten() {
            if let Some(dma) = ch["dma"].as_str() {
                if let Some(name) = ch["name"].as_str() {
                    pools.entry(dma.to_string()).or_default().insert(name.to_string());
                }
                if let Some(mux) = ch["dmamux"].as_str() {
                    mux_to_ctrls.entry(mux.to_string()).or_default().insert(dma.to_string());
                }
            }
        }
    }

    // Demand: per-peripheral signal → allowed pools (first name occurrence wins).
    let mut legs: BTreeMap<String, BTreeMap<String, BTreeSet<String>>> = BTreeMap::new();
    for core in v["cores"].as_array().into_iter().flatten() {
        for p in core["peripherals"].as_array().into_iter().flatten() {
            let Some(pn) = p["name"].as_str() else { continue };
            if legs.contains_key(pn) {
                continue; // first occurrence wins
            }
            let mut by_signal: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
            for d in p["dma_channels"].as_array().into_iter().flatten() {
                let Some(sig) = d["signal"].as_str() else { continue };
                let sp = by_signal.entry(sig.to_string()).or_default();
                if let Some(ctrl) = d["dma"].as_str() {
                    sp.insert(ctrl.to_string()); // named-controller model (C5/H5)
                } else if let Some(mux) = d["dmamux"].as_str() {
                    if let Some(ctrls) = mux_to_ctrls.get(mux) {
                        sp.extend(ctrls.iter().cloned()); // DMAMUX fan-out (G4)
                    }
                } else if let Some(chan) = d["channel"].as_str() {
                    if let Some(ctrl) = chan.split('_').next() {
                        sp.insert(ctrl.to_string()); // fixed-channel: "DMA1_CH5" → "DMA1"
                    }
                }
            }
            legs.insert(pn.to_string(), by_signal);
        }
    }

    ChipDma { pools, legs }
}

/// Find the stm32-data chip JSON for a metapac chip name by longest-prefix match
/// (`"STM32C531RCT6"` → `…/STM32C531RC.json`), tolerating the package/temp suffix
/// the descriptor name carries but the JSON file name omits. Returns the parsed
/// value, or `None` if no file matches / it won't parse.
pub fn load_for_chip(chips_dir: &str, chip_name: &str) -> Option<Value> {
    let mut best: Option<(usize, std::path::PathBuf)> = None;
    for entry in std::fs::read_dir(chips_dir).ok()?.flatten() {
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) != Some("json") {
            continue;
        }
        let Some(stem) = path.file_stem().and_then(|s| s.to_str()) else { continue };
        if chip_name.starts_with(stem) && best.as_ref().is_none_or(|(n, _)| stem.len() > *n) {
            best = Some((stem.len(), path.clone()));
        }
    }
    let (_, path) = best?;
    let text = std::fs::read_to_string(path).ok()?;
    serde_json::from_str(&text).ok()
}
