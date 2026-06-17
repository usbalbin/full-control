//! Generates `assets/descriptors.json` — per-package-letter pin/AF + DMA
//! descriptors for the whole STM32 lineup — from stm32-data's per-chip JSON.
//!
//!   cargo run --bin gen_descriptors --features gen-descriptors -- \
//!       ~/my_projects/stm32-data/build/data/chips
//!
//! Reads the generated JSON directly (no metapac build), so one run emits every
//! part. Parts are deduplicated to one entry per package letter (the prefix
//! "STM32<line><letter>"): flash variants share a pinout, so a representative is
//! enough. The DMA legs are normalized to "signal -> allowed controller pools"
//! exactly as `extract.rs` does for metapac, collapsing the DMAMUX / named /
//! fixed models so the engine never branches per family. Crucially this gets C5
//! DMA, which metapac-gen omits.

use peri_planner::desc_asset::{AssetPart, AssetPeri};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::Path;

fn normalize_part(v: &Value) -> Option<AssetPart> {
    let name = v["name"].as_str()?.to_string();
    if name.len() < 10 {
        return None;
    }
    let prefix = name[..10].to_string();
    let family = v["family"].as_str().unwrap_or("").trim_end_matches(" Series").to_string();

    // Package style + the bonded-pin set live in `packages[0]`.
    let package = v["packages"]
        .as_array()
        .and_then(|a| a.first())
        .and_then(|p| p["package"].as_str())
        .unwrap_or("")
        .to_string();

    // DMA supply: controller -> channel count, and dmamux -> {controllers}.
    let mut pool_channels: BTreeMap<String, u32> = BTreeMap::new();
    let mut mux_to_ctrls: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for core in v["cores"].as_array().into_iter().flatten() {
        for ch in core["dma_channels"].as_array().into_iter().flatten() {
            if let Some(dma) = ch["dma"].as_str() {
                *pool_channels.entry(dma.to_string()).or_default() += 1;
                if let Some(mux) = ch["dmamux"].as_str() {
                    mux_to_ctrls.entry(mux.to_string()).or_default().insert(dma.to_string());
                }
            }
        }
    }

    // Peripherals (union across cores by name; first occurrence wins).
    let mut peris: Vec<AssetPeri> = Vec::new();
    let mut seen: BTreeSet<String> = BTreeSet::new();
    for core in v["cores"].as_array().into_iter().flatten() {
        for p in core["peripherals"].as_array().into_iter().flatten() {
            let pn = match p["name"].as_str() {
                Some(s) => s.to_string(),
                None => continue,
            };
            if !seen.insert(pn.clone()) {
                continue;
            }
            let address = p["address"].as_u64().unwrap_or(0);
            let block = p["registers"]["block"].as_str().map(str::to_string);

            let mut pins: Vec<(String, String, Option<u8>)> = Vec::new();
            for pc in p["pins"].as_array().into_iter().flatten() {
                let (Some(pin), Some(sig)) = (pc["pin"].as_str(), pc["signal"].as_str()) else {
                    continue;
                };
                let af = pc["af"].as_u64().map(|n| n as u8);
                pins.push((pin.to_string(), sig.to_string(), af));
            }

            // DMA legs: signal -> set of allowed controller pools.
            let mut by_signal: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
            for d in p["dma_channels"].as_array().into_iter().flatten() {
                let sig = match d["signal"].as_str() {
                    Some(s) => s.to_string(),
                    None => continue,
                };
                let pools = by_signal.entry(sig).or_default();
                if let Some(ctrl) = d["dma"].as_str() {
                    pools.insert(ctrl.to_string());
                } else if let Some(mux) = d["dmamux"].as_str() {
                    if let Some(ctrls) = mux_to_ctrls.get(mux) {
                        pools.extend(ctrls.iter().cloned());
                    }
                } else if let Some(chan) = d["channel"].as_str() {
                    if let Some(ctrl) = chan.split('_').next() {
                        pools.insert(ctrl.to_string());
                    }
                }
            }
            let dma: Vec<(String, Vec<String>)> = by_signal
                .into_iter()
                .filter(|(_, pools)| !pools.is_empty())
                .map(|(s, pools)| (s, pools.into_iter().collect()))
                .collect();

            peris.push(AssetPeri { n: pn, a: address, b: block, pins, dma });
        }
    }

    let pools: Vec<(String, u8)> = pool_channels.into_iter().map(|(k, v)| (k, v as u8)).collect();

    Some(AssetPart { prefix, name, family, package, peris, pools })
}

/// Pick the richer of two same-prefix parts (most peripherals, then most pins) —
/// flash variants share a pinout, so a max-coverage representative is safe.
fn richness(p: &AssetPart) -> (usize, usize) {
    (p.peris.len(), p.peris.iter().map(|x| x.pins.len()).sum())
}

fn main() {
    let dir = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("usage: gen_descriptors <stm32-data chips dir>");
        std::process::exit(2);
    });

    let mut by_prefix: BTreeMap<String, AssetPart> = BTreeMap::new();
    let mut files = 0usize;
    for ent in fs::read_dir(&dir).expect("read chips dir") {
        let path = ent.unwrap().path();
        if path.extension().and_then(|e| e.to_str()) != Some("json") {
            continue;
        }
        let v: Value = serde_json::from_str(&fs::read_to_string(&path).expect("read")).expect("parse");
        files += 1;
        if let Some(part) = normalize_part(&v) {
            by_prefix
                .entry(part.prefix.clone())
                .and_modify(|cur| {
                    if richness(&part) > richness(cur) {
                        *cur = part.clone();
                    }
                })
                .or_insert(part);
        }
    }

    let parts: Vec<AssetPart> = by_prefix.into_values().collect();
    let out = "assets/descriptors.json.gz";
    if let Some(parent) = Path::new(out).parent() {
        fs::create_dir_all(parent).ok();
    }
    let json = serde_json::to_vec(&parts).unwrap();

    // The raw asset is ~12 MB (too large to embed/compile) but gzips ~28x, so we
    // commit + embed the gzipped form and inflate it once at runtime.
    use flate2::{write::GzEncoder, Compression};
    use std::io::Write as _;
    let mut enc = GzEncoder::new(Vec::new(), Compression::best());
    enc.write_all(&json).expect("gzip");
    let gz = enc.finish().expect("gzip finish");
    fs::write(out, &gz).expect("write descriptors.json.gz");

    let total_pins: usize = parts.iter().flat_map(|p| &p.peris).map(|x| x.pins.len()).sum();
    println!(
        "wrote {} ({} prefixes from {} chips, {} pin rows; {:.1} MB raw -> {:.0} KB gz)",
        out,
        parts.len(),
        files,
        total_pins,
        json.len() as f64 / 1_048_576.0,
        gz.len() as f64 / 1024.0,
    );
}
