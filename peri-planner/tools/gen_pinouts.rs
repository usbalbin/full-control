//! Generates `assets/pinouts.json.gz` — the lineup-wide **physical pinout**
//! asset (per-footprint `position -> signal(s)`, including power rails) consumed
//! by `src/phys_pinout.rs` — from stm32-data's per-chip JSON.
//!
//!   cargo run --bin gen_pinouts --features gen-pinouts -- \
//!       ~/my_projects/stm32-data/build/data/chips
//!
//! Unlike `gen_descriptors` (which keeps one logical descriptor per package
//! *letter*, collapsing footprints), this reads **every** `packages[i]` of every
//! chip and keeps the full physical position map — a part that ships in LQFP100
//! and TFBGA100 yields two footprints. Distinct pinouts are content-hashed and
//! deduplicated (the lineup has ~2600 chip-packages but only ~520 distinct
//! physical maps); an index keys each chip-package's full part name to its
//! pinout hash, giving the catalog-part -> physical-pinout bridge.
//!
//! The asset is a verbatim mirror of the source signal tokens — all
//! interpretation (GPIO vs power vs dedicated, rail normalization) is done at
//! runtime in `phys_pinout.rs`, so this generator only extracts + dedups.

use peri_planner::phys_pinout::{PartFootprint, PhysPin, PinoutAsset, PinoutRecord};
use serde_json::Value;
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

/// FNV-1a 64-bit, hex. A content hash for pinout dedup — collisions between
/// *different* canonical maps are detected and panic on (see `main`), so the
/// hash is a safe identity key without pulling in a crypto dependency.
fn fnv1a(s: &str) -> String {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in s.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{h:016x}")
}

/// Canonical string for a footprint's pin set: positions sorted, signals within
/// a position sorted, so two parts with the same physical map hash identically
/// regardless of source ordering.
fn canonical(pins: &[PhysPin]) -> String {
    let mut rows: Vec<String> = pins
        .iter()
        .map(|p| {
            let mut sig = p.s.clone();
            sig.sort();
            format!("{}={}", p.p, sig.join(","))
        })
        .collect();
    rows.sort();
    rows.join(";")
}

/// Extract every footprint of one chip JSON as `(part_name, package_name, pins)`.
fn footprints_of(v: &Value) -> Vec<(String, String, Vec<PhysPin>)> {
    let Some(name) = v["name"].as_str() else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for pkg in v["packages"].as_array().into_iter().flatten() {
        let Some(pkg_name) = pkg["package"].as_str() else {
            continue;
        };
        let mut pins: Vec<PhysPin> = Vec::new();
        for pc in pkg["pins"].as_array().into_iter().flatten() {
            let Some(pos) = pc["position"].as_str() else {
                continue;
            };
            let sigs: Vec<String> = pc["signals"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(|s| s.as_str().map(str::to_string))
                .collect();
            pins.push(PhysPin { p: pos.to_string(), s: sigs });
        }
        if !pins.is_empty() {
            out.push((name.to_string(), pkg_name.to_string(), pins));
        }
    }
    out
}

fn main() {
    let dir = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("usage: gen_pinouts <stm32-data chips dir>");
        eprintln!("  e.g. gen_pinouts ~/my_projects/stm32-data/build/data/chips");
        std::process::exit(2);
    });

    // hash -> (record, canonical) ; canonical kept to detect FNV collisions.
    let mut records: BTreeMap<String, (PinoutRecord, String)> = BTreeMap::new();
    let mut index: Vec<PartFootprint> = Vec::new();
    let mut files = 0usize;

    for ent in fs::read_dir(&dir).expect("read chips dir") {
        let path = ent.unwrap().path();
        if path.extension().and_then(|e| e.to_str()) != Some("json") {
            continue;
        }
        let v: Value =
            serde_json::from_str(&fs::read_to_string(&path).expect("read")).expect("parse");
        files += 1;
        let family = v["family"]
            .as_str()
            .unwrap_or("")
            .trim_end_matches(" Series")
            .to_string();

        for (part, pkg, pins) in footprints_of(&v) {
            let canon = canonical(&pins);
            let h = fnv1a(&canon);
            index.push(PartFootprint { name: part, pkg: pkg.clone(), h: h.clone() });
            records
                .entry(h.clone())
                .and_modify(|(rec, existing_canon)| {
                    assert_eq!(
                        *existing_canon, canon,
                        "FNV-1a hash collision between distinct pinouts (hash {h}) — \
                         widen the hash"
                    );
                    if !rec.names.contains(&pkg) {
                        rec.names.push(pkg.clone());
                    }
                })
                .or_insert_with(|| {
                    let n = pins.len() as u16;
                    (
                        PinoutRecord {
                            h: h.clone(),
                            pkg: pkg.clone(),
                            names: vec![pkg.clone()],
                            fam: family.clone(),
                            n,
                            pins,
                        },
                        canon,
                    )
                });
        }
    }

    let mut records: Vec<PinoutRecord> = records.into_values().map(|(r, _)| r).collect();
    for r in &mut records {
        r.names.sort();
        r.names.dedup();
    }
    records.sort_by(|a, b| a.h.cmp(&b.h));
    index.sort_by(|a, b| (a.name.as_str(), a.pkg.as_str()).cmp(&(b.name.as_str(), b.pkg.as_str())));

    let asset = PinoutAsset { records, index };

    let out = "assets/pinouts.json.gz";
    if let Some(parent) = Path::new(out).parent() {
        fs::create_dir_all(parent).ok();
    }
    let json = serde_json::to_vec(&asset).unwrap();

    use flate2::{write::GzEncoder, Compression};
    use std::io::Write as _;
    let mut enc = GzEncoder::new(Vec::new(), Compression::best());
    enc.write_all(&json).expect("gzip");
    let gz = enc.finish().expect("gzip finish");
    fs::write(out, &gz).expect("write pinouts.json.gz");

    println!(
        "wrote {} ({} distinct pinouts, {} chip-package index rows from {} chips; \
         {:.1} MB raw -> {:.0} KB gz)",
        out,
        asset.records.len(),
        asset.index.len(),
        files,
        json.len() as f64 / 1_048_576.0,
        gz.len() as f64 / 1024.0,
    );
}
