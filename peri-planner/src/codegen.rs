//! Firmware codegen — turns a [`PinPlan`](crate::pin_plan::PinPlan) into
//! embassy-stm32 scaffold source for the firmware crate. See
//! `docs/firmware-codegen-design.md` (the "generate a board-support *scaffold*,
//! not a full init" stance).
//!
//! This increment emits the **Tier-1 board-support layer**: a generated `Board`
//! struct that names every peripheral instance and pin the plan uses,
//! destructured from embassy's `Peripherals`. It is purely *structural* — no
//! behavioral value can appear — and is the foundation the driver constructors
//! (next increment) build on. The output is a Rust source string; peri-planner
//! itself does not depend on embassy, so codegen is string generation, not
//! compilation.

use crate::pin_plan::{DmaAssignment, FabricNode, Placement, PinPlan};

/// Peripheral classes whose embassy singleton we bind as a `Board` field.
/// Their pins are bound regardless; analog peripherals (COMP/OPAMP) get pin
/// fields but no instance field (their instances are Tier-2/3 raw access).
const TIER1_INSTANCE_CLASSES: &[&str] =
    &["USART", "UART", "LPUART", "SPI", "I2C", "TIM", "ADC", "DAC", "FDCAN", "HRTIM"];

/// The peripheral class of an instance name, i.e. the name with its trailing
/// instance digits stripped: `"USART2" -> "USART"`, `"I2C1" -> "I2C"`,
/// `"HRTIM1" -> "HRTIM"`.
fn class_of(instance: &str) -> &str {
    instance.trim_end_matches(|c: char| c.is_ascii_digit())
}

fn is_tier1_instance(instance: &str) -> bool {
    TIER1_INSTANCE_CLASSES.contains(&class_of(instance))
}

/// If a fabric-route endpoint is a standalone embassy peripheral singleton,
/// its singleton name (`COMP1`, `DAC3`, `ADC1`, `TIM8`). Used to bundle pinless
/// instances the design uses internally (an OCP comparator, a PCM threshold
/// DAC) so the user still gets `p.COMP1`/`p.DAC3` for raw fabric config.
/// `EEV`/`FLT`/`MASTER`/`HRTIM_TIM` are NOT standalone singletons — they live
/// inside `HRTIM1`, which is already bundled via its channel pins.
fn fabric_instance_singleton(node: &FabricNode) -> Option<String> {
    match node.class.as_str() {
        "COMP" | "DAC" | "ADC" | "TIM" => Some(format!("{}{}", node.class, node.instance)),
        _ => None,
    }
}

/// Sanitize a metapac name into a snake_case Rust identifier fragment.
/// Names here are alphanumeric (`USART2`, `CH1`, `IN3`), so this just
/// lowercases; non-alphanumerics (none expected) fold to `_`.
fn ident(name: &str) -> String {
    name.chars()
        .map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_lowercase() } else { '_' })
        .collect()
}

/// PascalCase an instance name into a bundle struct identifier:
/// `"USART1" -> "Usart1"`, `"HRTIM1" -> "Hrtim1"`, `"I2C1" -> "I2c1"`.
fn type_name(instance: &str) -> String {
    instance
        .chars()
        .enumerate()
        .flat_map(|(i, c)| {
            if i == 0 {
                c.to_uppercase().collect::<Vec<_>>()
            } else {
                c.to_lowercase().collect::<Vec<_>>()
            }
        })
        .collect()
}

/// Generate the board-support module — one *bundle* struct per peripheral
/// instance (its typed singleton + role-named pins), aggregated into a `Board`
/// with `Board::take(p)`. We stop at this resource-grouping boundary on purpose:
/// peri-planner does NOT emit driver init, because embassy's init API churns.
/// The user constructs each peripheral from its bundle, in their own embassy
/// version's idiom. Field NAMES are stable across pin reassignment; only the pin
/// TYPES change on regenerate, and embassy constructors are generic over the
/// pin, so the user's init is unaffected. Never hand-edited.
pub fn generate_board(plan: &PinPlan) -> String {
    use std::collections::{HashMap, HashSet};
    use std::fmt::Write as _;

    // Group placements by peripheral instance, preserving first-seen order.
    let mut order: Vec<&str> = Vec::new();
    let mut groups: HashMap<&str, Vec<&Placement>> = HashMap::new();
    for p in &plan.placements {
        let inst = p.signal.peripheral.as_str();
        if !groups.contains_key(inst) {
            order.push(inst);
        }
        groups.entry(inst).or_default().push(p);
    }

    // Pinless fabric-route endpoints that are real embassy singletons (an OCP
    // comparator, a PCM threshold DAC) — the design uses them internally and
    // the user needs the singleton for raw fabric config, even with no pin.
    let mut fabric_only: Vec<String> = Vec::new();
    for e in &plan.routes {
        for node in [&e.from, &e.to] {
            let Some(inst) = fabric_instance_singleton(node) else { continue };
            if !order.contains(&inst.as_str()) && !fabric_only.contains(&inst) {
                fabric_only.push(inst);
            }
        }
    }

    // Defensive dedup: an over-constrained PinPlan (two converter legs on one
    // timer → duplicate (peripheral, role); a pin-exhausted design that
    // double-booked a pad → one pad on two signals) would otherwise emit a
    // duplicate struct field or move a `Peripherals` singleton twice — uncompilable
    // Rust. Keep the FIRST placement per (instance, role) and per physical pin; the
    // rest are conflicts, dropped here and counted for a header warning. Both the
    // struct-field and `take()` loops read this deduped `groups`, so they agree.
    let mut seen_role: HashSet<(&str, &str)> = HashSet::new();
    let mut seen_pin: HashSet<String> = HashSet::new();
    let mut conflicts = 0u32;
    for inst in &order {
        if let Some(v) = groups.get_mut(*inst) {
            v.retain(|p| {
                let role_new = seen_role.insert((*inst, p.signal.role.as_str()));
                let pin_new = p.pin.is_none_or(|pin| seen_pin.insert(pin.name()));
                let keep = role_new && pin_new;
                if !keep {
                    conflicts += 1;
                }
                keep
            });
        }
    }

    // DMA channel fields for an instance's bundle: `{function}_dma:
    // peripherals::{CHANNEL}` (e.g. `stream_dma: peripherals::DMA1_CH3`).
    let dma_fields = |inst: &str| -> Vec<(String, &str)> {
        plan.dma_assignments
            .iter()
            .filter(|d: &&DmaAssignment| d.peripheral == inst)
            .map(|d| (format!("{}_dma", ident(&d.function)), d.channel.as_str()))
            .collect()
    };

    let mut s = String::new();
    let _ = writeln!(
        s,
        "// @generated by peri-planner from PinPlan (target {} / {}) — DO NOT EDIT.",
        plan.target.package, plan.target.family
    );
    let _ = writeln!(s, "// One bundle of typed singletons per peripheral (instance + role-named pins).");
    let _ = writeln!(s, "// peri-planner does NOT generate driver init — you construct each peripheral");
    let _ = writeln!(s, "// from its bundle, in your own embassy version's idiom. Field NAMES are stable");
    let _ = writeln!(s, "// across pin changes; only the pin TYPES change on regenerate, and embassy");
    let _ = writeln!(s, "// constructors are generic over the pin, so your init is unaffected. e.g.:");
    let _ = writeln!(s, "//   let b = Board::take(embassy_stm32::init(Default::default()));");
    let _ = writeln!(s, "//   let uart = Uart::new(b.usart1.instance, b.usart1.rx, b.usart1.tx, Irqs, dma, cfg)?;");
    if conflicts > 0 {
        let _ = writeln!(
            s,
            "// ⚠ {conflicts} signal(s) dropped — this design has pin/resource conflicts (e.g. two \
             uses competing for one pad or instance). Resolve them in the planner; the firmware \
             below covers only the conflict-free subset.",
        );
    }
    let _ = writeln!(s, "#![allow(dead_code)]");
    let _ = writeln!(s);
    let _ = writeln!(s, "use embassy_stm32::{{peripherals, Peripherals}};");
    let _ = writeln!(s);

    // One bundle struct per peripheral instance.
    for inst in &order {
        let ty = type_name(inst);
        let _ = writeln!(s, "/// Resources for {inst} — pass to your own constructor.");
        let _ = writeln!(s, "pub struct {ty} {{");
        if is_tier1_instance(inst) {
            let _ = writeln!(s, "    pub instance: peripherals::{inst},");
        }
        for p in &groups[*inst] {
            match p.pin {
                Some(pin) => {
                    let _ = writeln!(s, "    pub {}: peripherals::{},", ident(&p.signal.role), pin.name());
                }
                None => {
                    let _ = writeln!(s, "    // unplaced: {} (declared, no pin yet)", p.signal.role);
                }
            }
        }
        for (fname, chan) in dma_fields(inst) {
            let _ = writeln!(s, "    pub {fname}: peripherals::{chan},");
        }
        let _ = writeln!(s, "}}");
        let _ = writeln!(s);
    }

    // Instance-only bundles for the pinless fabric endpoints.
    for inst in &fabric_only {
        let ty = type_name(inst);
        let _ = writeln!(s, "/// Resources for {inst} — internal fabric instance (no pins on this design).");
        let _ = writeln!(s, "pub struct {ty} {{");
        let _ = writeln!(s, "    pub instance: peripherals::{inst},");
        for (fname, chan) in dma_fields(inst) {
            let _ = writeln!(s, "    pub {fname}: peripherals::{chan},");
        }
        let _ = writeln!(s, "}}");
        let _ = writeln!(s);
    }

    // Board aggregates the per-peripheral bundles.
    let _ = writeln!(s, "/// Every peripheral this design uses, grouped into per-instance bundles.");
    let _ = writeln!(s, "pub struct Board {{");
    for inst in &order {
        let _ = writeln!(s, "    pub {}: {},", ident(inst), type_name(inst));
    }
    for inst in &fabric_only {
        let _ = writeln!(s, "    pub {}: {},", ident(inst), type_name(inst));
    }
    let _ = writeln!(s, "}}");
    let _ = writeln!(s);

    // take(p): destructure Peripherals into the bundles.
    let _ = writeln!(s, "impl Board {{");
    let _ = writeln!(s, "    /// Destructure the embassy `Peripherals` into named per-peripheral bundles.");
    let _ = writeln!(s, "    pub fn take(p: Peripherals) -> Self {{");
    let _ = writeln!(s, "        Self {{");
    for inst in &order {
        let mut parts: Vec<String> = Vec::new();
        if is_tier1_instance(inst) {
            parts.push(format!("instance: p.{inst}"));
        }
        for p in &groups[*inst] {
            if let Some(pin) = p.pin {
                parts.push(format!("{}: p.{}", ident(&p.signal.role), pin.name()));
            }
        }
        for (fname, chan) in dma_fields(inst) {
            parts.push(format!("{fname}: p.{chan}"));
        }
        let _ = writeln!(s, "            {}: {} {{ {} }},", ident(inst), type_name(inst), parts.join(", "));
    }
    for inst in &fabric_only {
        let mut parts = vec![format!("instance: p.{inst}")];
        for (fname, chan) in dma_fields(inst) {
            parts.push(format!("{fname}: p.{chan}"));
        }
        let _ = writeln!(s, "            {}: {} {{ {} }},", ident(inst), type_name(inst), parts.join(", "));
    }
    let _ = writeln!(s, "        }}");
    let _ = writeln!(s, "    }}");
    let _ = writeln!(s, "}}");

    s
}

/// One behavioral parameter the pin plan cannot know — a generated `Behavior`
/// field. Adding a routed peripheral adds a hole, so the design and its config
/// can't silently drift (docs/firmware-codegen-design.md §4).
struct Hole {
    name: String,
    ty: &'static str,
    default: &'static str,
    doc: &'static str,
}

/// The behavioral holes implied by the plan's Tier-1 peripherals, in instance
/// order. Derived from the peripheral class + its placements' role kinds.
fn behavior_holes(plan: &PinPlan) -> Vec<Hole> {
    use crate::pin_plan::RoleKind;
    let mut instances: Vec<&str> = Vec::new();
    for p in &plan.placements {
        let inst = p.signal.peripheral.as_str();
        if is_tier1_instance(inst) && !instances.contains(&inst) {
            instances.push(inst);
        }
    }

    let mut holes = Vec::new();
    for inst in instances {
        let id = ident(inst);
        match class_of(inst) {
            "USART" | "UART" | "LPUART" => holes.push(Hole {
                name: format!("{id}_baud"), ty: "u32", default: "115_200", doc: "baud rate",
            }),
            "SPI" => holes.push(Hole {
                name: format!("{id}_freq_hz"), ty: "u32", default: "1_000_000", doc: "SCK frequency (Hz)",
            }),
            "I2C" => holes.push(Hole {
                name: format!("{id}_freq_hz"), ty: "u32", default: "100_000", doc: "bus frequency (Hz)",
            }),
            "FDCAN" => holes.push(Hole {
                name: format!("{id}_bitrate"), ty: "u32", default: "500_000", doc: "nominal bitrate (bps)",
            }),
            "TIM" | "HRTIM" => {
                let pwm = plan.placements.iter().any(|p| {
                    p.signal.peripheral == inst && matches!(p.role_kind, RoleKind::PwmOut { .. })
                });
                if pwm {
                    holes.push(Hole {
                        name: format!("{id}_freq_hz"), ty: "u32", default: "100_000",
                        doc: "PWM switching frequency (Hz)",
                    });
                    let dead_time = plan.placements.iter().any(|p| {
                        p.signal.peripheral == inst
                            && matches!(p.role_kind, RoleKind::PwmOut { dead_time: true, .. })
                    });
                    if dead_time {
                        holes.push(Hole {
                            name: format!("{id}_dead_time_ns"), ty: "u16", default: "0",
                            doc: "complementary dead-time (ns)",
                        });
                    }
                }
            }
            _ => {} // ADC/DAC: no required scalar in v1.
        }
    }
    holes
}

/// Append the behavioral-config contract (`Behavior` struct + `Default`) — the
/// set of holes the user fills, structurally separate from the plan.
fn write_behavior(s: &mut String, plan: &PinPlan) {
    use std::fmt::Write as _;
    let holes = behavior_holes(plan);
    let _ = writeln!(s);
    let _ = writeln!(s, "/// Behavioral configuration you must supply — one field per value the pin");
    let _ = writeln!(s, "/// plan cannot know (it carries no behavioral values by design). Adjust the");
    let _ = writeln!(s, "/// defaults to your converter. Adding a routed peripheral adds a field here,");
    let _ = writeln!(s, "/// so the design and its config can't silently drift.");
    let _ = writeln!(s, "#[derive(Clone, Copy, Debug)]");
    let _ = writeln!(s, "pub struct Behavior {{");
    for h in &holes {
        let _ = writeln!(s, "    /// {}", h.doc);
        let _ = writeln!(s, "    pub {}: {},", h.name, h.ty);
    }
    let _ = writeln!(s, "}}");
    let _ = writeln!(s);
    let _ = writeln!(s, "impl Default for Behavior {{");
    let _ = writeln!(s, "    fn default() -> Self {{");
    let _ = writeln!(s, "        Self {{");
    for h in &holes {
        let _ = writeln!(s, "            {}: {},", h.name, h.default);
    }
    let _ = writeln!(s, "        }}");
    let _ = writeln!(s, "    }}");
    let _ = writeln!(s, "}}");
}

/// Generate the full Tier-1 firmware scaffold module: the board-support layer
/// (every instance + pin) followed by the behavioral-config contract.
pub fn generate(plan: &PinPlan) -> String {
    let mut s = generate_board(plan);
    write_behavior(&mut s, plan);
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mcu_pinout::{OwnedSignal, PinId};
    use crate::pin_plan::{PinOrigin, PinPlan, Placement, RoleKind, Target};

    fn placement(peripheral: &str, role: &str, pin: Option<PinId>) -> Placement {
        Placement {
            signal: OwnedSignal { peripheral: peripheral.into(), role: role.into() },
            pin,
            origin: PinOrigin::Locked,
            af: None,
            role_kind: RoleKind::Gpio,
            irqs: Vec::new(),
            package_pin: None,
            net: None,
        }
    }

    #[test]
    fn generate_board_emits_per_peripheral_bundles() {
        let mut plan = PinPlan::empty(Target { package: "H523RE".into(), family: "H5".into() });
        plan.placements.push(placement("USART1", "TX", Some(PinId { port: 'A', num: 9 })));
        plan.placements.push(placement("USART1", "RX", None)); // declared, unplaced
        let code = generate_board(&plan);

        assert!(code.contains("from PinPlan (target H523RE / H5)"));
        // A per-peripheral bundle struct with instance + role-named pin fields.
        assert!(code.contains("pub struct Usart1 {"));
        assert!(code.contains("    pub instance: peripherals::USART1,"));
        assert!(code.contains("    pub tx: peripherals::PA9,"));
        // Board aggregates the bundle; take() builds it from `p`.
        assert!(code.contains("    pub usart1: Usart1,"));
        assert!(code.contains("usart1: Usart1 { instance: p.USART1, tx: p.PA9 },"));
        // Unplaced signal -> comment inside the bundle, never a bogus field.
        assert!(code.contains("// unplaced: RX"));
        assert!(!code.contains("pub rx: peripherals::"));
    }

    #[test]
    fn analog_peripherals_bundle_pins_but_not_instance() {
        // COMP is not a Tier-1 instance class: its pins are bundled, instance is not.
        let mut plan = PinPlan::empty(Target { package: "G474R".into(), family: "G4".into() });
        plan.placements.push(placement("COMP1", "INP", Some(PinId { port: 'A', num: 1 })));
        plan.placements.push(placement("ADC1", "IN2", Some(PinId { port: 'A', num: 2 })));
        let code = generate_board(&plan);

        assert!(code.contains("pub struct Comp1 {"));
        assert!(code.contains("    pub inp: peripherals::PA1,"), "COMP pin bundled");
        assert!(!code.contains("pub instance: peripherals::COMP1,"), "COMP instance NOT bound (Tier-2/3)");
        assert!(code.contains("pub struct Adc1 {"));
        assert!(code.contains("    pub instance: peripherals::ADC1,"), "ADC instance bound (Tier-1)");
        assert!(code.contains("    pub in2: peripherals::PA2,"));
    }

    #[test]
    fn generate_emits_behavior_contract_holes() {
        let mut plan = PinPlan::empty(Target { package: "G474R".into(), family: "G4".into() });
        plan.placements.push(placement("USART1", "TX", Some(PinId { port: 'A', num: 9 })));
        plan.placements.push(placement("SPI1", "SCK", Some(PinId { port: 'A', num: 5 })));
        // A complementary PWM channel with dead-time -> freq + dead-time holes.
        plan.placements.push(Placement {
            role_kind: RoleKind::PwmOut { complementary: true, dead_time: true, etr: false },
            ..placement("TIM1", "CH1", Some(PinId { port: 'A', num: 8 }))
        });
        let code = generate(&plan);

        // One hole per behavioral value the plan can't know.
        assert!(code.contains("pub usart1_baud: u32,"));
        assert!(code.contains("pub spi1_freq_hz: u32,"));
        assert!(code.contains("pub tim1_freq_hz: u32,"));
        assert!(code.contains("pub tim1_dead_time_ns: u16,"));
        // Defaults wired up.
        assert!(code.contains("usart1_baud: 115_200,"));
        assert!(code.contains("tim1_dead_time_ns: 0,"));
        // Behavior struct present; board layer still emitted above it.
        assert!(code.contains("pub struct Behavior {"));
        assert!(code.contains("pub struct Board {"));
        // No behavioral VALUE ever leaks into the board layer.
        assert!(!code.contains("115_200,\n    pub usart1: peripherals"));
    }

    #[test]
    fn c531_converter_bundles_pins_and_pinless_fabric_instances() {
        // A real C531 buck leg lowered end-to-end: TIM1 CH1+CH1N pins materialized,
        // ADC1 sense pin, and an OCP (COMP1->break, DAC1 threshold) that claims NO
        // pins. The pinless COMP1/DAC1 must still appear as instance bundles.
        use crate::c531_design::{C531Design, ConverterLeg, Ocp};
        use crate::g474::{AdcInstance, CompId, DacId, TimId};
        use crate::mcu::Package;

        let mut d = C531Design::new();
        d.add_leg(ConverterLeg {
            tim: TimId::Tim1, channels_mask: 0b0001, complementary: true, dead_time: true, bkin: false,
            ocp: Some(Ocp { comp: CompId::Comp1, break_input: 1, threshold_dac: Some(DacId::Dac1Ch1) }),
            adc_sense: Some((AdcInstance::Adc1, 1)),
        });
        let mut plan = d.to_pin_plan(Package::C531R, Target { package: "C531R".into(), family: "C5".into() });
        crate::pin_plan::assign_dma(&mut plan, Package::C531R.descriptor());
        let code = generate(&plan);

        // Pin-bearing bundles: TIM1 (HS/LS) + ADC1 sense, with instance fields.
        assert!(code.contains("pub struct Tim1 {"));
        assert!(code.contains("    pub instance: peripherals::TIM1,"));
        assert!(code.contains("    pub ch1: peripherals::"));
        assert!(code.contains("    pub ch1n: peripherals::"));
        assert!(code.contains("pub struct Adc1 {"));
        // The ADC stream gets a concrete, real DMA channel name in its bundle
        // (C531 LPDMA is 0-based — could not have been synthesized from a count).
        assert!(code.contains("    pub stream_dma: peripherals::LPDMA1_CH0,"));
        assert!(code.contains("stream_dma: p.LPDMA1_CH0"));

        // Pinless fabric instances the OCP uses -> instance-only bundles.
        assert!(code.contains("pub struct Comp1 {"));
        assert!(code.contains("comp1: Comp1 { instance: p.COMP1 },"));
        assert!(code.contains("pub struct Dac1 {"));
        assert!(code.contains("dac1: Dac1 { instance: p.DAC1 },"));
        // Behavior carries the PWM holes.
        assert!(code.contains("pub tim1_freq_hz: u32,"));
        assert!(code.contains("pub tim1_dead_time_ns: u16,"));
    }

    #[test]
    fn generated_board_references_only_real_singletons() {
        // NAME-VALIDITY guard (not exactly-once — that's the next test): every
        // `peripherals::IDENT` the scaffold emits must be a real metapac entity in
        // the descriptor — a peripheral instance, a GPIO pin, or a DMA channel.
        // embassy's `Peripherals` mirrors the metapac peripheral list, so "real
        // metapac name" == "compiles against embassy". Catches any future name
        // synthesis (fabric instances, DMA channels) or data change that would emit
        // a bogus singleton. (Double-move prevention lives in the lowerers' `taken`
        // sets + `assign_dma`'s `used` set, and is checked by the next test.)
        // Exercised on the richest family (G474 default: HRTIM/ADC/DAC/COMP + stream DMA).
        use crate::pin_plan::{assign_dma, Target};
        use std::collections::BTreeSet;

        let desc = crate::mcu::Package::G474R.descriptor();
        let mut plan = crate::requirements::Design::default()
            .to_pin_plan(Target { package: "G474RE".into(), family: "G4".into() });
        assign_dma(&mut plan, desc);
        let code = generate(&plan);

        // Every `peripherals::IDENT` referenced by the generated module.
        let mut idents: BTreeSet<String> = BTreeSet::new();
        for (i, _) in code.match_indices("peripherals::") {
            let rest = &code[i + "peripherals::".len()..];
            let id: String = rest.chars().take_while(|c| c.is_ascii_alphanumeric() || *c == '_').collect();
            if !id.is_empty() {
                idents.insert(id);
            }
        }
        assert!(idents.len() > 5, "expected several peripheral singletons, got {idents:?}");

        let peris: BTreeSet<&str> = desc.raw.peripherals.iter().map(|p| p.name).collect();
        let pins: BTreeSet<String> = crate::mcu_pinout::af_rows(desc.raw).map(|r| r.pin.name()).collect();
        let chans: BTreeSet<&str> = desc.dma_pools().iter().flat_map(|p| p.chans.iter().copied()).collect();
        for id in &idents {
            let ok = peris.contains(id.as_str()) || pins.contains(id) || chans.contains(id.as_str());
            assert!(ok, "generated `peripherals::{id}` is not a real descriptor peripheral/pin/DMA channel");
        }
    }

    #[test]
    fn generated_board_moves_each_singleton_exactly_once() {
        // The real soundness property: `take(p: Peripherals)` may move each
        // `Peripherals` field AT MOST ONCE — a double-move wouldn't compile. Count
        // the `p.IDENT` moves (uppercase = a metapac singleton) and assert none
        // repeats. Guards the lowerers'/assign_dma's de-dup end-to-end.
        use crate::pin_plan::{assign_dma, Target};
        use std::collections::BTreeMap;

        let desc = crate::mcu::Package::G474R.descriptor();
        let mut plan = crate::requirements::Design::default()
            .to_pin_plan(Target { package: "G474RE".into(), family: "G4".into() });
        assign_dma(&mut plan, desc);
        let code = generate(&plan);

        let mut moves: BTreeMap<String, u32> = BTreeMap::new();
        for (i, _) in code.match_indices("p.") {
            // `p` must be a standalone token (a `Peripherals` move), not the tail
            // of a longer identifier like `tmp.`.
            if code[..i].chars().last().is_some_and(|c| c.is_ascii_alphanumeric() || c == '_') {
                continue;
            }
            let id: String = code[i + 2..]
                .chars()
                .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
                .collect();
            // Peripherals fields are uppercase metapac names (USART1, PA9, DMA1_CH3).
            if id.chars().next().is_some_and(|c| c.is_ascii_uppercase()) {
                *moves.entry(id).or_default() += 1;
            }
        }
        assert!(moves.len() > 5, "expected several singleton moves, got {moves:?}");
        for (id, n) in &moves {
            assert_eq!(*n, 1, "Peripherals field p.{id} is moved {n}× (must be exactly once)");
        }
    }

    #[test]
    fn class_of_strips_instance_digits() {
        assert_eq!(class_of("USART2"), "USART");
        assert_eq!(class_of("I2C1"), "I2C");
        assert_eq!(class_of("HRTIM1"), "HRTIM");
        assert!(is_tier1_instance("LPUART1"));
        assert!(!is_tier1_instance("COMP1"));
    }

    #[test]
    fn over_constrained_design_yields_compilable_code() {
        // Two converter legs on the SAME timer (TIM1) — a conflicting design (and
        // pin-pressure case). The lowerer produces duplicate (TIM1, CH1) placements;
        // codegen MUST still emit compilable Rust: no duplicate struct field, no
        // Peripherals singleton moved twice, and a conflict warning.
        use crate::c531_design::{C531Design, ConverterLeg};
        use crate::g474::TimId;
        use crate::mcu::Package;
        use crate::pin_plan::{assign_dma, Target};

        let mut d = C531Design::new();
        d.add_leg(ConverterLeg::pwm(TimId::Tim1));
        d.add_leg(ConverterLeg::pwm(TimId::Tim1));
        let mut plan =
            d.to_pin_plan(Package::C531R, Target { package: "C531R".into(), family: "C5".into() });
        assign_dma(&mut plan, Package::C531R.descriptor());
        let code = generate(&plan);

        assert_eq!(code.matches("    pub ch1:").count(), 1, "the ch1 field must appear exactly once");

        let mut moves: std::collections::BTreeMap<String, u32> = std::collections::BTreeMap::new();
        for (i, _) in code.match_indices("p.") {
            if code[..i].chars().last().is_some_and(|c| c.is_ascii_alphanumeric() || c == '_') {
                continue;
            }
            let id: String = code[i + 2..]
                .chars()
                .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
                .collect();
            if id.chars().next().is_some_and(|c| c.is_ascii_uppercase()) {
                *moves.entry(id).or_default() += 1;
            }
        }
        assert!(moves.values().all(|&n| n == 1), "no singleton moved twice: {moves:?}");
        assert!(code.contains("signal(s) dropped"), "the conflict warning must be emitted");
    }
}
