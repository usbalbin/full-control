//! Generates `assets/catalog.json` — the whole-lineup STM32 part catalog
//! consumed by `peri_planner::catalog` — from stm32-data's per-chip JSON.
//!
//!   cargo run --bin gen_catalog --features gen-catalog -- \
//!       ~/my_projects/stm32-data/build/data/chips
//!
//! Reads every `STM32*.json` in the given directory, distills the
//! search-relevant fields (memory + peripheral instance counts), and writes
//! a name-sorted JSON array (one entry per line). Adding/updating MCUs — a
//! new family, a fresh stm32-data release — is a re-run, not a code change.
//!
//! Note: this reads stm32-data's *generated JSON* directly; it does NOT
//! build `stm32-metapac`, so it can emit all ~1600 parts in one pass instead
//! of one metapac codegen build per chip.

use peri_planner::catalog::CatalogEntry;
use serde_json::Value;
use std::collections::BTreeSet;
use std::fs;
use std::path::Path;

/// Count peripheral instances named `<prefix><number>` (e.g. "USART3"),
/// excluding suffixed blocks like "ADC12_COMMON".
fn inst_count(names: &BTreeSet<String>, prefix: &str) -> u8 {
    names
        .iter()
        .filter(|n| {
            n.strip_prefix(prefix)
                .map(|rest| !rest.is_empty() && rest.bytes().all(|b| b.is_ascii_digit()))
                .unwrap_or(false)
        })
        .count() as u8
}

fn has_any(names: &BTreeSet<String>, want: &[&str]) -> bool {
    want.iter().any(|w| names.contains(*w))
}

/// A bare `TIM<n>` instance name (not LPTIM / HRTIM, whose outputs aren't CHxN).
fn is_plain_timer(name: &str) -> bool {
    name.strip_prefix("TIM")
        .map(|r| !r.is_empty() && r.bytes().all(|b| b.is_ascii_digit()))
        .unwrap_or(false)
}

/// A complementary-output signal `CH<n>N` (e.g. "CH1N").
fn is_chxn(sig: &str) -> bool {
    sig.strip_prefix("CH")
        .and_then(|r| r.strip_suffix('N'))
        .map(|d| !d.is_empty() && d.bytes().all(|b| b.is_ascii_digit()))
        .unwrap_or(false)
}

/// Count distinct complementary-PWM channels: `(timer instance, CHxN signal)`
/// pairs over every timer's pin/AF table, unioned across cores. Each is a
/// deadtime-capable high/low output pair (one converter half-bridge). A sound
/// upper bound on what a part can wire (the channel also needs both pins broken
/// out — the Tier-2 check); counting the signal, not the pin, keeps it package-
/// invariant and never under-counts a feasible part.
fn comp_pwm_channels(v: &Value) -> u8 {
    let mut slots: BTreeSet<(String, String)> = BTreeSet::new();
    for core in v["cores"].as_array().into_iter().flatten() {
        for p in core["peripherals"].as_array().into_iter().flatten() {
            let Some(pname) = p["name"].as_str() else { continue };
            if !is_plain_timer(pname) {
                continue;
            }
            for pc in p["pins"].as_array().into_iter().flatten() {
                if let Some(sig) = pc["signal"].as_str() {
                    if is_chxn(sig) {
                        slots.insert((pname.to_string(), sig.to_string()));
                    }
                }
            }
        }
    }
    slots.len() as u8
}

fn entry_from(v: &Value) -> Option<CatalogEntry> {
    let name = v["name"].as_str()?.to_string();
    // Normalize family: the freshly-merged C5 data declares "STM32C5 Series"
    // whereas every other family is bare ("STM32G4"). Strip the suffix so C5
    // is a first-class family alongside the rest.
    let family = v["family"]
        .as_str()
        .unwrap_or("")
        .trim_end_matches(" Series")
        .to_string();

    // `memory` is a list of alternative bank layouts for the part; each
    // layout sums to the same totals, so the first is representative.
    let (mut flash, mut ram) = (0u64, 0u64);
    if let Some(layout) = v["memory"].as_array().and_then(|a| a.first()) {
        for r in layout.as_array().into_iter().flatten() {
            let size = r["size"].as_u64().unwrap_or(0);
            match r["kind"].as_str() {
                Some("flash") => flash += size,
                Some("ram") => ram += size,
                _ => {}
            }
        }
    }

    // Union peripheral names across all cores (dual-core parts list them
    // per core); instance counting is over the deduplicated set. Also collect
    // the distinct AF-capable GPIO pins (names like "PA0") for the pin-capacity
    // bound.
    let mut names = BTreeSet::new();
    let mut gpio = BTreeSet::new();
    for core in v["cores"].as_array().into_iter().flatten() {
        for p in core["peripherals"].as_array().into_iter().flatten() {
            if let Some(n) = p["name"].as_str() {
                names.insert(n.to_string());
            }
            for pc in p["pins"].as_array().into_iter().flatten() {
                if let Some(pin) = pc["pin"].as_str() {
                    let b = pin.as_bytes();
                    // "P" + port letter + digits, e.g. "PA0".
                    if b.len() >= 3 && b[0] == b'P' && b[1].is_ascii_uppercase()
                        && b[2..].iter().all(u8::is_ascii_digit)
                    {
                        gpio.insert(pin.to_string());
                    }
                }
            }
        }
    }

    // Total physical DMA channels (supply side = core-level `dma_channels`),
    // deduplicated across cores by channel name: dual-core parts list the same
    // controllers under each core, so a naive sum double-counts (e.g. H745
    // lists 40 channels per core → 80 naive, 40 real).
    let mut dma_channels = BTreeSet::new();
    for core in v["cores"].as_array().into_iter().flatten() {
        for ch in core["dma_channels"].as_array().into_iter().flatten() {
            if let Some(n) = ch["name"].as_str() {
                dma_channels.insert(n.to_string());
            }
        }
    }

    let tim_adv = ["TIM1", "TIM8", "TIM20"]
        .iter()
        .filter(|t| names.contains(**t))
        .count() as u8;

    // Datasheet package styles this part ships in (a part can offer several
    // footprints of one letter, e.g. LQFP100 + TFBGA100).
    let mut packages = BTreeSet::new();
    for p in v["packages"].as_array().into_iter().flatten() {
        if let Some(s) = p["package"].as_str() {
            packages.insert(s.to_string());
        }
    }

    Some(CatalogEntry {
        name,
        family,
        flash_kb: (flash / 1024) as u32,
        ram_kb: (ram / 1024) as u32,
        usart: inst_count(&names, "USART"),
        uart: inst_count(&names, "UART"),
        lpuart: inst_count(&names, "LPUART"),
        spi: inst_count(&names, "SPI"),
        i2c: inst_count(&names, "I2C"),
        i3c: inst_count(&names, "I3C"),
        fdcan: inst_count(&names, "FDCAN"),
        ucpd: inst_count(&names, "UCPD"),
        adc: inst_count(&names, "ADC"),
        dac: inst_count(&names, "DAC"),
        comp: inst_count(&names, "COMP"),
        opamp: inst_count(&names, "OPAMP"),
        tim_adv,
        tim_total: inst_count(&names, "TIM"),
        comp_pwm_ch: comp_pwm_channels(v),
        has_usb: has_any(&names, &["USB", "USB_OTG_FS", "USB_OTG_HS"]),
        has_hrtim: has_any(&names, &["HRTIM", "HRTIM1"]),
        octospi: inst_count(&names, "OCTOSPI"),
        has_sdmmc: names.iter().any(|n| n.starts_with("SDMMC")),
        has_fmc: names.contains("FMC"),
        has_eth: names.iter().any(|n| n.starts_with("ETH")),
        dma_pool_total: dma_channels.len() as u16,
        gpio_pins: gpio.len() as u16,
        packages: packages.into_iter().collect(),
    })
}

fn main() {
    let dir = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("usage: gen_catalog <stm32-data chips dir>");
        eprintln!("  e.g. gen_catalog ~/my_projects/stm32-data/build/data/chips");
        std::process::exit(2);
    });

    let mut entries: Vec<CatalogEntry> = Vec::new();
    let mut skipped = 0usize;
    for ent in fs::read_dir(&dir).expect("failed to read chips directory") {
        let path = ent.unwrap().path();
        if path.extension().and_then(|e| e.to_str()) != Some("json") {
            continue;
        }
        let txt = fs::read_to_string(&path).expect("read chip json");
        let v: Value = serde_json::from_str(&txt).expect("parse chip json");
        match entry_from(&v) {
            Some(e) => entries.push(e),
            None => skipped += 1,
        }
    }
    entries.sort_by(|a, b| a.name.cmp(&b.name));

    let out = "assets/catalog.json";
    if let Some(parent) = Path::new(out).parent() {
        fs::create_dir_all(parent).ok();
    }
    // One compact entry per line: deterministic and diff-friendly.
    let mut s = String::from("[\n");
    for (i, e) in entries.iter().enumerate() {
        s.push_str(&serde_json::to_string(e).unwrap());
        s.push_str(if i + 1 < entries.len() { ",\n" } else { "\n" });
    }
    s.push_str("]\n");
    fs::write(out, s).expect("write catalog.json");

    let families: BTreeSet<&str> = entries.iter().map(|e| e.family.as_str()).collect();
    println!(
        "wrote {} ({} parts across {} families, {} skipped)",
        out,
        entries.len(),
        families.len(),
        skipped
    );
}
