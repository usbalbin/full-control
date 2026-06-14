//! Tier-3 fabric generator: recovers the analog cross-peripheral routing the
//! planner currently hand-codes (the data `stm32-metapac`/stm32-data discard)
//! from ST's CubeMX IP "modes" XML in `stm32-data-sources/cubedb`.
//!
//!   cargo run --bin gen_fabric --features gen-fabric -- \
//!       <cubedb>/mcu/IP/COMP-TSMC90_G4_Rockfish_Cube_Modes.xml \
//!       <cubedb>/mcu/IP/HRTIM-hrtim_G4_Modes.xml
//!
//! Extracts, into `src/fabric_data.rs`:
//!  * `G4_DAC_TO_COMP` — DAC channel -> comparator inverting input, from the
//!    COMP modes `InvertingInput` blocks (each DAC source gated by a
//!    `$IpNumber=<n>` condition naming the comparator instances it reaches).
//!  * `G4_COMP_TO_EEV` — comparator output -> HRTIM external event, from the
//!    HRTIM modes `HRTIM_EEV<n>SRC_COMP<c>_OUT` possible values.
//!
//! Both are validated byte-for-byte against the hand-coded tables (`G474_EDGES`
//! in `mcu.rs`, `COMP_TO_EEV` in `g474.rs`) by unit tests. Where ST's cubedb
//! has a known bug, a small documented fixup overlay restores the correct
//! edge (same pattern stm32-data uses for ST source errors).
//!
//! No external deps and no metapac build — plain string scanning over the XML.

use std::collections::BTreeSet;
use std::fs;

/// Leading run of ASCII digits at the start of `s` (possibly empty).
fn leading_digits(s: &str) -> &str {
    let end = s.find(|c: char| !c.is_ascii_digit()).unwrap_or(s.len());
    &s[..end]
}

// ---------- DAC -> COMP (from COMP modes `InvertingInput`) ----------

/// Parse `(dac_instance, dac_channel)` from a `COMP_INPUT_MINUS_DAC<d>_CH<c>`
/// value inside an `InvertingInput` block, if present.
fn parse_dac_source(chunk: &str) -> Option<(u8, u8)> {
    let after = &chunk[chunk.find("COMP_INPUT_MINUS_DAC")? + "COMP_INPUT_MINUS_DAC".len()..];
    let dac: u8 = leading_digits(after).parse().ok()?;
    let after = &after[after.find("_CH")? + "_CH".len()..];
    let ch: u8 = leading_digits(after).parse().ok()?;
    Some((dac, ch))
}

/// Comparator instance numbers named by `$IpNumber=<n>` terms (these gate
/// which comparators an inverting-input source can reach).
fn comp_instances(chunk: &str) -> Vec<u8> {
    let mut out = Vec::new();
    let mut rest = chunk;
    while let Some(i) = rest.find("$IpNumber=") {
        rest = &rest[i + "$IpNumber=".len()..];
        if let Ok(n) = leading_digits(rest).parse() {
            out.push(n);
        }
    }
    out
}

/// `(dac, channel, comp)` edges from a COMP modes XML.
fn extract_dac_to_comp(xml: &str) -> BTreeSet<(u8, u8, u8)> {
    let mut edges = BTreeSet::new();
    // Split on `<RefParameter`, then bound each chunk to its OWN `</RefParameter>`
    // — RefParameters interleave with `<RefMode>`/`<Mode>` blocks that also
    // mention `InvertingInput`/DAC, so an unbounded chunk concatenates unrelated
    // conditions.
    for raw in xml.split("<RefParameter") {
        let chunk = match raw.find("</RefParameter>") {
            Some(end) => &raw[..end],
            None => continue,
        };
        if !chunk.contains("Name=\"InvertingInput\"") {
            continue;
        }
        if let Some((dac, ch)) = parse_dac_source(chunk) {
            for comp in comp_instances(chunk) {
                edges.insert((dac, ch, comp));
            }
        }
    }
    edges
}

// ---------- COMP -> EEV (from HRTIM modes external-event sources) ----------

/// Known cubedb bugs to restore, verified against RM0440 Table 223
/// ("External events mapping"). The G4 HRTIM modes file drops EEV8<-COMP6:
/// its `HRTIM_EEV8SRC_COMP6_OUT` possible value is malformed — duplicated as
/// `HRTIM_EEV9SRC_COMP5_OUT` with `Comment="event8"`. RM0440 lists hrtim_eev8
/// sources as COMP6 and COMP3.
const COMP_TO_EEV_FIXUP_ADD: &[(u8, u8)] = &[(6, 8)]; // (comp, eev)

/// `(comp, eev)` edges from a HRTIM modes XML, parsed from
/// `HRTIM_EEV<eev>SRC_COMP<comp>_OUT` value tokens, plus documented fixups.
fn extract_comp_to_eev(xml: &str) -> BTreeSet<(u8, u8)> {
    let mut edges = BTreeSet::new();
    let mut rest = xml;
    while let Some(i) = rest.find("HRTIM_EEV") {
        rest = &rest[i + "HRTIM_EEV".len()..];
        let eev_s = leading_digits(rest);
        let after = &rest[eev_s.len()..];
        let after = match after.strip_prefix("SRC_COMP") {
            Some(a) => a,
            None => continue,
        };
        let comp_s = leading_digits(after);
        if after[comp_s.len()..].starts_with("_OUT") {
            if let (Ok(eev), Ok(comp)) = (eev_s.parse::<u8>(), comp_s.parse::<u8>()) {
                edges.insert((comp, eev));
            }
        }
    }
    for &e in COMP_TO_EEV_FIXUP_ADD {
        edges.insert(e);
    }
    edges
}

// ---------- COMP -> FLT (from RM0440 Table 228; NOT in cubedb) ----------

// The CubeMX HRTIM modes only abstract fault sources to a TYPE
// (DIGITALINPUT / INTERNAL / EEVINPUT) — the specific comparator per fault is
// absent. The authoritative mapping is RM0440 "Table 228. Fault inputs" (its
// `On-chip source: COMP` column). We parse it from the HRTIM reference-manual
// text produced by `pdftotext -layout hrtim.pdf`.

/// Fault channel number from a `Fault <n>` table row, if present.
fn fault_number(line: &str) -> Option<u8> {
    let i = line.find("Fault ")? + "Fault ".len();
    leading_digits(line[i..].trim_start()).parse().ok()
}

/// First `COMP<c>` instance number in a line, if present.
fn comp_in_line(line: &str) -> Option<u8> {
    let i = line.find("COMP")? + "COMP".len();
    leading_digits(&line[i..]).parse().ok()
}

/// `(comp, flt)` edges from RM0440 Table 228 text. Tries each "Table 228"
/// occurrence (front-matter list-of-tables vs the body) and keeps the richest.
fn extract_comp_to_flt(rm_text: &str) -> BTreeSet<(u8, u8)> {
    let mut best: BTreeSet<(u8, u8)> = BTreeSet::new();
    let mut from = 0;
    while let Some(rel) = rm_text[from..].find("Table 228") {
        let pos = from + rel;
        from = pos + "Table 228".len();
        let mut edges = BTreeSet::new();
        for line in rm_text[pos..].lines().take(30) {
            if let (Some(flt), Some(comp)) = (fault_number(line), comp_in_line(line)) {
                if (1..=6).contains(&flt) {
                    edges.insert((comp, flt));
                }
            }
        }
        if edges.len() > best.len() {
            best = edges;
        }
        if best.len() >= 6 {
            break;
        }
    }
    best
}

// ---------- HRTIM crossbar -> ADC trigger (from HRTIM modes) ----------

// ADC-trigger sources are encoded as `HRTIM_ADCTRIGGEREVENT{13|24}_<SOURCE>`:
// group 13 => ADC triggers {1,3}, group 24 => {2,4}; a source in both groups
// => {1,2,3,4}. RM0440 registers HRTIM_ADC1R..ADC4R are authoritative.

/// Map a cubedb ADC-trigger source token to the `CrossbarSource` Debug name.
fn canon_crossbar(src: &str) -> Option<String> {
    if let Some(n) = src.strip_prefix("MASTER_CMP") {
        return Some(format!("Mcr{n}"));
    }
    if src == "MASTER_PERIOD" {
        return Some("Mper".to_string());
    }
    if let Some(n) = src.strip_prefix("EVENT_") {
        return Some(format!("Eev{n}"));
    }
    if let Some(rest) = src.strip_prefix("TIMER") {
        let t = rest.chars().next()?;
        let tail = &rest[t.len_utf8()..];
        let suffix = if let Some(n) = tail.strip_prefix("_CMP") {
            format!("Cr{n}")
        } else if tail == "_PERIOD" {
            "CrPer".to_string()
        } else if tail == "_RESET" {
            "CrRst".to_string()
        } else {
            return None;
        };
        return Some(format!("Tim{t}{suffix}"));
    }
    None
}

// RM0440-verified fixups for cubedb errors. cubedb places TIMERF_CMP2 in the
// 13 group too; HRTIM_ADC1R..4R show Timer F compare 2 only in even triggers.
const CROSSBAR_FIXUP: &[(&str, &[u8])] = &[("TimFCr2", &[2, 4])];

/// `(CrossbarSource-name, [adc-trigger numbers])` from a HRTIM modes XML.
fn extract_crossbar_to_adc_trigger(xml: &str) -> Vec<(String, Vec<u8>)> {
    use std::collections::BTreeMap;
    let mut groups: BTreeMap<String, BTreeSet<u8>> = BTreeMap::new();
    let pat = "HRTIM_ADCTRIGGEREVENT";
    let mut rest = xml;
    while let Some(i) = rest.find(pat) {
        rest = &rest[i + pat.len()..];
        let grp: &[u8] = if rest.starts_with("13_") {
            &[1, 3]
        } else if rest.starts_with("24_") {
            &[2, 4]
        } else {
            continue;
        };
        let after = &rest[3..];
        let end = after
            .find(|c: char| !(c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_'))
            .unwrap_or(after.len());
        let src = &after[..end];
        if src == "NONE" || src.is_empty() {
            continue;
        }
        if let Some(name) = canon_crossbar(src) {
            groups.entry(name).or_default().extend(grp.iter().copied());
        }
    }
    for &(name, trigs) in CROSSBAR_FIXUP {
        groups.insert(name.to_string(), trigs.iter().copied().collect());
    }
    groups
        .into_iter()
        .map(|(k, v)| (k, v.into_iter().collect()))
        .collect()
}

// ---------- emit ----------

fn emit(
    dac_to_comp: &BTreeSet<(u8, u8, u8)>,
    comp_to_eev: &BTreeSet<(u8, u8)>,
    comp_to_flt: &BTreeSet<(u8, u8)>,
    crossbar: &[(String, Vec<u8>)],
) -> String {
    let mut s = String::new();
    s.push_str(
        "// GENERATED by `gen_fabric` from cubedb CubeMX IP \"modes\" XML + RM0440 — do not edit by hand.\n\n",
    );

    s.push_str(
        "/// DAC channel -> comparator inverting input, as `(dac_instance,\n\
         /// dac_channel, comp_instance)`. From CubeMX COMP `InvertingInput`\n\
         /// modes (`$IpNumber` instance gating). Family-level (G4). Validated\n\
         /// against the hand-coded `G474_EDGES`.\n",
    );
    s.push_str("pub static G4_DAC_TO_COMP: &[(u8, u8, u8)] = &[\n");
    for (d, c, n) in dac_to_comp {
        s.push_str(&format!("    ({d}, {c}, {n}),\n"));
    }
    s.push_str("];\n\n");

    s.push_str(
        "/// Comparator output -> HRTIM external event, as `(comp_instance,\n\
         /// eev_number)`. From CubeMX HRTIM `HRTIM_EEV<n>SRC_COMP<c>_OUT`\n\
         /// modes, plus documented fixups for cubedb bugs (see gen_fabric).\n\
         /// Validated against the hand-coded `COMP_TO_EEV`.\n",
    );
    s.push_str("pub static G4_COMP_TO_EEV: &[(u8, u8)] = &[\n");
    for (c, e) in comp_to_eev {
        s.push_str(&format!("    ({c}, {e}),\n"));
    }
    s.push_str("];\n\n");

    s.push_str(
        "/// Comparator output -> HRTIM fault input, as `(comp_instance,\n\
         /// flt_number)`. NOT in cubedb (which only abstracts faults to a\n\
         /// source type); parsed from RM0440 \"Table 228. Fault inputs\".\n\
         /// COMP7 has no fault input. Validated against the hand `COMP_TO_FLT`.\n",
    );
    s.push_str("pub static G4_COMP_TO_FLT: &[(u8, u8)] = &[\n");
    for (c, f) in comp_to_flt {
        s.push_str(&format!("    ({c}, {f}),\n"));
    }
    s.push_str("];\n\n");

    s.push_str(
        "/// HRTIM crossbar source -> ADC trigger, as `(CrossbarSource Debug\n\
         /// name, &[adc-trigger numbers])`. From CubeMX HRTIM\n\
         /// ADCTRIGGEREVENT{13|24} modes (13->{1,3}, 24->{2,4}), with\n\
         /// RM0440-verified fixups. Validated against `CROSSBAR_TO_ADC_TRIGGER`.\n",
    );
    s.push_str("pub static G4_CROSSBAR_TO_ADC_TRIGGER: &[(&str, &[u8])] = &[\n");
    for (name, trigs) in crossbar {
        let t = trigs.iter().map(|n| n.to_string()).collect::<Vec<_>>().join(", ");
        s.push_str(&format!("    ({name:?}, &[{t}]),\n"));
    }
    s.push_str("];\n");
    s
}

fn main() {
    let mut args = std::env::args().skip(1);
    let comp_modes = args.next().unwrap_or_else(|| {
        eprintln!("usage: gen_fabric <COMP modes xml> [HRTIM modes xml] [RM0440 hrtim text]");
        eprintln!("  RM text: `pdftotext -layout hrtim.pdf rm_hrtim.txt` (for COMP->FLT, Table 228)");
        std::process::exit(2);
    });
    let hrtim_modes = args.next();
    let rm_text = args.next();

    let dac_to_comp = extract_dac_to_comp(&fs::read_to_string(&comp_modes).expect("read COMP xml"));
    let hrtim_xml = hrtim_modes
        .as_ref()
        .map(|p| fs::read_to_string(p).expect("read HRTIM xml"));
    let comp_to_eev = match &hrtim_xml {
        Some(x) => extract_comp_to_eev(x),
        None => BTreeSet::new(),
    };
    let crossbar = match &hrtim_xml {
        Some(x) => extract_crossbar_to_adc_trigger(x),
        None => Vec::new(),
    };
    let comp_to_flt = match &rm_text {
        Some(p) => extract_comp_to_flt(&fs::read_to_string(p).expect("read RM hrtim text")),
        None => BTreeSet::new(),
    };

    fs::write(
        "src/fabric_data.rs",
        emit(&dac_to_comp, &comp_to_eev, &comp_to_flt, &crossbar),
    )
    .expect("write src/fabric_data.rs");
    println!(
        "wrote src/fabric_data.rs ({} DAC->COMP, {} COMP->EEV, {} COMP->FLT, {} crossbar->ADC edges)",
        dac_to_comp.len(),
        comp_to_eev.len(),
        comp_to_flt.len(),
        crossbar.len()
    );
}
