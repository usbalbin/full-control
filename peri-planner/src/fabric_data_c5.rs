// Hand-cited from RM0522 (STM32C53x / STM32C542 reference manual) — NOT generated.
//
// The STM32C5 analog fabric is absent from the CubeMX "modes" XML used to
// generate the G4 tables (cubedb has no C5 IP modes), and metapac carries 0
// interconnect `triggers` for C531, so these tables are transcribed directly
// from the reference manual. They are guarded by golden tests in mcu.rs
// (`c5_support`) whose expectations were authored independently of this data and
// cross-checked by a separate RM0522 re-read.

/// DAC channel -> comparator inverting input, as `(dac_instance, dac_channel,
/// comp_instance)`. From RM0522 "Table 172. COMP inverting input assignment":
/// the inverting-input mux code `INMSEL = 0b0100` selects `dac1_ch1` for COMP1
/// and `dac1_ch2` for COMP2 — the threshold path for a comparator-based trip.
pub static C5_DAC_TO_COMP: &[(u8, u8, u8)] = &[
    (1, 1, 1), // dac1_ch1 -> COMP1 inverting input
    (1, 2, 2), // dac1_ch2 -> COMP2 inverting input
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
