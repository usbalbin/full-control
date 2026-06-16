// STM32C5 analog fabric. The authoritative source is ST's CubeMX2 die
// descriptor in stm32-data: `sources/cubeprogdb2/stm32c5/Descriptors/
// peripherals/D44F_peripherals.json` (C531 die 44F). C5 is the first family on
// CubeMX2, so it is absent from the classic `cubedb` "modes" XML the G4 tables
// are generated from, and metapac still carries 0 interconnect `triggers` for
// C531 — hence these tables, while transcribed here, are now backed by a
// machine-readable source and could be generated (see plan: gen_fabric_c5).
//
// IMPORTANT: D44F is a *die* descriptor shared by C531/C532/C542 — a SUPERSET of
// any one part. Routes through peripheral instances a given part does not
// populate must be dropped by intersecting with that part's actual peripheral
// set (chip JSON / metapac). The DAC->COMP table below is the canonical example.
// Guarded by golden tests in mcu.rs (`c5_support`).

/// DAC -> comparator inverting input, as `(dac_instance, dac_channel,
/// comp_instance)`. From the CubeMX2 `analogInterconnections` COMP input mux:
/// `COMP1_inst <- DAC1_inst/DACINT` and `COMP2_inst <- DAC2_inst/DACINT`.
///
/// C531 populates only DAC1 (no DAC2 — verified against the C531 chip JSON, SVD
/// and CMSIS header), so the die-level `COMP2 <- DAC2` route does NOT exist on
/// this part: **COMP2 has no internal DAC threshold on C531** (its inverting
/// input can only take the VBG scaler taps or an external pin). The C5 DACs
/// expose a single internal `DACINT` node, not a CH1/CH2 mux, so the
/// `dac_channel` field is a planner-convention placeholder (canonical `1`).
pub static C5_DAC_TO_COMP: &[(u8, u8, u8)] = &[
    (1, 1, 1), // DAC1 (single DACINT) -> COMP1 inverting input
    // (COMP2 <- DAC2 exists on die 44F but DAC2 is absent on C531 — omitted.)
];

/// Comparator output -> advanced-timer break input, as `(comp_instance,
/// tim_instance, break_input)` where `break_input` is 1 (BRK) or 2 (BRK2). This
/// is the hardware over-current path of a timer-PWM converter: a comparator trip
/// folds the PWM outputs at the silicon level, independent of firmware.
///
/// From RM0522 section 31.3.2 (TIM1/TIM8 internal signals): the break
/// comparator multiplexers route `tim_brk_cmp1 <- comp1_out`,
/// `tim_brk_cmp2 <- comp2_out` into BRK, and `tim_brk2_cmp1 <- comp1_out`,
/// `tim_brk2_cmp2 <- comp2_out` into BRK2. The table is shared by the two
/// advanced-control timers TIM1 and TIM8, so each takes COMP1 and COMP2 on both
/// break inputs.
pub static C5_COMP_TO_TIM_BREAK: &[(u8, u8, u8)] = &[
    (1, 1, 1), (2, 1, 1), // TIM1 BRK  <- COMP1, COMP2
    (1, 1, 2), (2, 1, 2), // TIM1 BRK2 <- COMP1, COMP2
    (1, 8, 1), (2, 8, 1), // TIM8 BRK  <- COMP1, COMP2
    (1, 8, 2), (2, 8, 2), // TIM8 BRK2 <- COMP1, COMP2
];
