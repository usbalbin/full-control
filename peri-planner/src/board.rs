//! **Board profiles** — dev-board overlays for the front-end planner, so you can
//! plan a *shield* against what the board actually exposes and leaves free rather
//! than the bare chip. A profile is board-level data (from the board's user
//! manual), not derivable from the MCU descriptor: which MCU pins reach which
//! header, and which pins the board already consumes (LEDs, buttons, the ST-LINK
//! virtual COM port, SWD, the clock).
//!
//! It plugs straight into the planner's existing pin machinery: selecting a board
//! sets the *usable* pin set (`bonded ∩ connector pins`) and pre-populates the
//! *reserved* set — both of which the solver already honours. Reservations are
//! overridable (freed with the same reserve chips); "soft" ones (LED, button,
//! VCP) can be used with a surfaced electrical caveat.
//!
//! First board: NUCLEO-G474RE (MB1367), from ST **UM2505** and the mbed pin map.

use std::collections::BTreeSet;

use crate::mcu_pinout::PinId;

/// How badly a board-reserved pin conflicts with reuse.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Severity {
    /// Really shouldn't be repurposed (SWD debug, the running clock).
    Block,
    /// Usable, but with an electrical caveat (an LED load, a button network, a
    /// solder-bridge to open) — surfaced as a warning.
    Warn,
}

/// What a board does with a pin by default, and the caveat for reusing it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reservation {
    pub function: &'static str,
    pub severity: Severity,
    pub caveat: &'static str,
}

/// One board pin of interest: its Arduino header label (if broken out there) and
/// what the board reserves it for (if anything). Pins with neither are ordinary
/// Morpho GPIO and don't need an entry.
#[derive(Clone, Copy, Debug)]
pub struct BoardPin {
    pub pin: PinId,
    pub arduino: Option<&'static str>,
    pub reserved: Option<Reservation>,
}

/// Which connector a shield mounts on — sets the usable pin budget.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Connector {
    /// The 2×19 ST Morpho headers (CN7/CN10) — essentially every MCU pin.
    #[default]
    Morpho,
    /// The Arduino Uno V3 headers — only the labelled A0–A5 / D0–D15 subset.
    Arduino,
}

impl Connector {
    pub const ALL: [Connector; 2] = [Connector::Morpho, Connector::Arduino];
    pub fn label(self) -> &'static str {
        match self {
            Connector::Morpho => "ST Morpho (CN7/CN10)",
            Connector::Arduino => "Arduino Uno",
        }
    }
}

/// A named dev board over a specific MCU/package.
#[derive(Clone, Copy, Debug)]
pub struct BoardProfile {
    pub name: &'static str,
    /// Descriptor-name prefix the board's chip matches (e.g. `"STM32G474R"`).
    pub chip_prefix: &'static str,
    pub pins: &'static [BoardPin],
}

impl BoardProfile {
    pub fn find(&self, pin: PinId) -> Option<&BoardPin> {
        self.pins.iter().find(|p| p.pin == pin)
    }
    /// Pins broken out on the Arduino header.
    pub fn arduino_pins(&self) -> BTreeSet<PinId> {
        self.pins.iter().filter(|p| p.arduino.is_some()).map(|p| p.pin).collect()
    }
    /// All board-reserved pins (any severity) — the reserve-by-default set.
    pub fn reserved_pins(&self) -> BTreeSet<PinId> {
        self.pins.iter().filter(|p| p.reserved.is_some()).map(|p| p.pin).collect()
    }
    /// The usable-pin set for a connector, intersected with what's actually bonded
    /// on the active footprint. Morpho brings out everything, so it's just the
    /// footprint set; Arduino restricts to the labelled subset.
    pub fn usable_pins(&self, connector: Connector, bonded: &BTreeSet<PinId>) -> BTreeSet<PinId> {
        match connector {
            Connector::Morpho => bonded.clone(),
            Connector::Arduino => bonded.intersection(&self.arduino_pins()).copied().collect(),
        }
    }
}

/// Boards whose chip matches this descriptor name (e.g. `"STM32G474RET6"`).
pub fn boards_for(chip_name: &str) -> Vec<&'static BoardProfile> {
    ALL_BOARDS.iter().filter(|b| chip_name.starts_with(b.chip_prefix)).copied().collect()
}

const fn p(port: char, num: u8, arduino: Option<&'static str>, reserved: Option<Reservation>) -> BoardPin {
    BoardPin { pin: PinId { port, num }, arduino, reserved }
}
const fn res(function: &'static str, severity: Severity, caveat: &'static str) -> Reservation {
    Reservation { function, severity, caveat }
}

/// NUCLEO-G474RE (STM32G474RET6, LQFP64, MB1367). Arduino map + reservations from
/// ST UM2505 and the mbed `TARGET_NUCLEO_G474RE` pin names. The clock is the
/// internal HSI16 by default, so PF0/PF1 stay free.
pub static NUCLEO_G474RE: BoardProfile = BoardProfile {
    name: "NUCLEO-G474RE",
    chip_prefix: "STM32G474R",
    pins: &[
        // Arduino analog header A0–A5.
        p('A', 0, Some("A0"), None),
        p('A', 1, Some("A1"), None),
        p('A', 4, Some("A2"), None),
        p('B', 0, Some("A3"), None),
        p('C', 1, Some("A4"), None),
        p('C', 0, Some("A5"), None),
        // Arduino digital header D0–D15.
        p('C', 5, Some("D0"), None),
        p('C', 4, Some("D1"), None),
        p('A', 10, Some("D2"), None),
        p('B', 3, Some("D3"), None),
        p('B', 5, Some("D4"), None),
        p('B', 4, Some("D5"), None),
        p('B', 10, Some("D6"), None),
        p('A', 8, Some("D7"), None),
        p('A', 9, Some("D8"), None),
        p('C', 7, Some("D9"), None),
        p('B', 6, Some("D10"), None),
        p('A', 7, Some("D11"), None),
        p('A', 6, Some("D12"), None),
        p(
            'A',
            5,
            Some("D13"),
            Some(res(
                "LD2 user LED / Arduino SCK",
                Severity::Warn,
                "drives the on-board green LED (~330 Ω) and is the Arduino SPI SCK — as an analog input it carries that load",
            )),
        ),
        p('B', 9, Some("D14"), None), // I2C SDA
        p('B', 8, Some("D15"), None), // I2C SCL
        // Board-reserved pins not on the Arduino header.
        p(
            'C',
            13,
            None,
            Some(res(
                "B1 user button",
                Severity::Warn,
                "tied to the blue user button (switch to GND); not a clean analog node",
            )),
        ),
        p(
            'A',
            2,
            None,
            Some(res(
                "ST-LINK VCP TX (LPUART1)",
                Severity::Warn,
                "on-board ST-LINK virtual COM port — open solder bridge SB to reuse",
            )),
        ),
        p(
            'A',
            3,
            None,
            Some(res(
                "ST-LINK VCP RX (LPUART1)",
                Severity::Warn,
                "on-board ST-LINK virtual COM port — open solder bridge SB to reuse",
            )),
        ),
        p(
            'A',
            13,
            None,
            Some(res("SWDIO (debug)", Severity::Block, "SWD debug — reusing it disables the on-board debugger")),
        ),
        p(
            'A',
            14,
            None,
            Some(res("SWCLK (debug)", Severity::Block, "SWD debug — reusing it disables the on-board debugger")),
        ),
        p(
            'C',
            14,
            None,
            Some(res(
                "LSE OSC32_IN",
                Severity::Warn,
                "32 kHz RTC crystal footprint (X2) — usable only if the crystal isn't populated",
            )),
        ),
        p(
            'C',
            15,
            None,
            Some(res(
                "LSE OSC32_OUT",
                Severity::Warn,
                "32 kHz RTC crystal footprint (X2) — usable only if the crystal isn't populated",
            )),
        ),
    ],
};

const LED_CAVEAT: &str = "drives an on-board LED / Arduino SCK — carries that load as an input";
const BTN_CAVEAT: &str = "tied to the user button (switch to GND); not a clean analog node";
const VCP_CAVEAT: &str = "on-board ST-LINK virtual COM port — open the SB to reuse";
const SWD_CAVEAT: &str = "SWD debug — reusing it disables the debugger";
const LSE_CAVEAT: &str = "32 kHz RTC crystal footprint — usable only if not populated";

/// NUCLEO-H533RE (STM32H533RET6, LQFP64, Nucleo-64, MB1814). Arduino map +
/// reserved pins from ST **UM3121** Table 16 (the NUH533RE column: A2 = PB1,
/// D0/D1 = PB15/PB14, D2 = PC8, D14/D15 = PB7/PB6 — note these differ from the
/// G474RE Nucleo-64). Reserved: LD2 = PA5, B1 = PC13, VCP = USART2 PA2/PA3, SWD =
/// PA13/PA14, LSE = PC14/PC15.
pub static NUCLEO_H533RE: BoardProfile = BoardProfile {
    name: "NUCLEO-H533RE",
    chip_prefix: "STM32H533R",
    pins: &[
        p('A', 0, Some("A0"), None),
        p('A', 1, Some("A1"), None),
        p('B', 1, Some("A2"), None),
        p('B', 0, Some("A3"), None),
        p('C', 1, Some("A4"), None),
        p('C', 0, Some("A5"), None),
        p('B', 15, Some("D0"), None),
        p('B', 14, Some("D1"), None),
        p('C', 8, Some("D2"), None),
        p('B', 3, Some("D3"), None),
        p('B', 5, Some("D4"), None),
        p('B', 4, Some("D5"), None),
        p('B', 10, Some("D6"), None),
        p('A', 8, Some("D7"), None),
        p('C', 7, Some("D8"), None),
        p('C', 6, Some("D9"), None),
        p('C', 9, Some("D10"), None),
        p('A', 7, Some("D11"), None),
        p('A', 6, Some("D12"), None),
        p('A', 5, Some("D13"), Some(res("LD2 user LED / Arduino D13", Severity::Warn, LED_CAVEAT))),
        p('B', 7, Some("D14"), None),
        p('B', 6, Some("D15"), None),
        p('C', 13, None, Some(res("B1 user button", Severity::Warn, BTN_CAVEAT))),
        p('A', 2, None, Some(res("ST-LINK VCP TX (USART2)", Severity::Warn, VCP_CAVEAT))),
        p('A', 3, None, Some(res("ST-LINK VCP RX (USART2)", Severity::Warn, VCP_CAVEAT))),
        p('A', 13, None, Some(res("SWDIO (debug)", Severity::Block, SWD_CAVEAT))),
        p('A', 14, None, Some(res("SWCLK (debug)", Severity::Block, SWD_CAVEAT))),
        p('C', 14, None, Some(res("LSE OSC32_IN", Severity::Warn, LSE_CAVEAT))),
        p('C', 15, None, Some(res("LSE OSC32_OUT", Severity::Warn, LSE_CAVEAT))),
    ],
};

/// NUCLEO-C5A3ZG (STM32C5A3ZGT6, LQFP144, Nucleo-144, MB2310). Arduino map +
/// reserved pins from ST **UM3616** (Table 12/13/14, §7.8 LEDs, §7.9 buttons,
/// §7.12 VCP). LQFP144, so the Arduino analog pins reach ports H/E (A0 = PH4,
/// A1 = PH5, A4 = PE13). LEDs: LD1 green = PA5, LD2 red = PG1, LD3 blue = PG2.
/// B1 = PC13; VCP = UART2 PA2/PA3 (muxed, exclusive with the Arduino UART).
pub static NUCLEO_C5A3ZG: BoardProfile = BoardProfile {
    name: "NUCLEO-C5A3ZG",
    chip_prefix: "STM32C5A3",
    pins: &[
        p('H', 4, Some("A0"), None),
        p('H', 5, Some("A1"), None),
        p('A', 4, Some("A2"), None),
        p('B', 0, Some("A3"), None),
        p('E', 13, Some("A4"), None),
        p('C', 0, Some("A5"), None),
        p('D', 6, Some("D0"), None),
        p('D', 5, Some("D1"), None),
        p('A', 10, Some("D2"), None),
        p('B', 3, Some("D3"), None),
        p('A', 0, Some("D4"), None),
        p('B', 4, Some("D5"), None),
        p('B', 10, Some("D6"), None),
        p('A', 8, Some("D7"), None),
        p('A', 9, Some("D8"), None),
        p('C', 6, Some("D9"), None),
        p('B', 5, Some("D10"), None),
        p('A', 7, Some("D11"), None),
        p('A', 6, Some("D12"), None),
        p('A', 5, Some("D13"), Some(res("LD1 green LED / Arduino D13", Severity::Warn, LED_CAVEAT))),
        p('B', 7, Some("D14"), None),
        p('B', 6, Some("D15"), None),
        p('G', 1, None, Some(res("LD2 red user LED", Severity::Warn, "drives the on-board red LED"))),
        p('G', 2, None, Some(res("LD3 blue user LED", Severity::Warn, "drives the on-board blue LED"))),
        p('C', 13, None, Some(res("B1 user button", Severity::Warn, BTN_CAVEAT))),
        p('A', 2, None, Some(res("ST-LINK VCP TX (UART2)", Severity::Warn, VCP_CAVEAT))),
        p('A', 3, None, Some(res("ST-LINK VCP RX (UART2)", Severity::Warn, VCP_CAVEAT))),
        p('A', 13, None, Some(res("SWDIO (debug)", Severity::Block, SWD_CAVEAT))),
        p('A', 14, None, Some(res("SWCLK (debug)", Severity::Block, SWD_CAVEAT))),
        p('C', 14, None, Some(res("LSE OSC32_IN", Severity::Warn, LSE_CAVEAT))),
        p('C', 15, None, Some(res("LSE OSC32_OUT", Severity::Warn, LSE_CAVEAT))),
    ],
};

static ALL_BOARDS: &[&BoardProfile] = &[&NUCLEO_G474RE, &NUCLEO_H533RE, &NUCLEO_C5A3ZG];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nucleo_g474re_matches_g474r_parts_only() {
        assert!(!boards_for("STM32G474RET6").is_empty());
        assert!(!boards_for("STM32G474RB").is_empty());
        assert!(boards_for("STM32G474VET6").is_empty(), "V (LQFP100) is a different board");
        assert!(boards_for("STM32H523RE").is_empty());
    }

    #[test]
    fn arduino_analog_pins_are_the_um2505_map() {
        let b = &NUCLEO_G474RE;
        let a0 = b.pins.iter().find(|p| p.arduino == Some("A0")).unwrap();
        assert_eq!(a0.pin, PinId { port: 'A', num: 0 });
        let a4 = b.pins.iter().find(|p| p.arduino == Some("A4")).unwrap();
        assert_eq!(a4.pin, PinId { port: 'C', num: 1 });
        // D13 is the LED pin AND an Arduino pin.
        let d13 = b.find(PinId { port: 'A', num: 5 }).unwrap();
        assert_eq!(d13.arduino, Some("D13"));
        assert!(d13.reserved.is_some(), "PA5 is LD2");
    }

    #[test]
    fn reserved_set_and_severities() {
        let b = &NUCLEO_G474RE;
        let r = b.reserved_pins();
        for name in ["PA2", "PA3", "PA5", "PC13", "PA13", "PA14"] {
            let pin = PinId::from_metapac(name).unwrap();
            assert!(r.contains(&pin), "{name} should be board-reserved");
        }
        // SWD is a hard block; the LED is a soft warning.
        assert_eq!(b.find(PinId { port: 'A', num: 13 }).unwrap().reserved.unwrap().severity, Severity::Block);
        assert_eq!(b.find(PinId { port: 'A', num: 5 }).unwrap().reserved.unwrap().severity, Severity::Warn);
    }

    #[test]
    fn all_boards_match_their_parts() {
        assert_eq!(boards_for("STM32H533RET6").len(), 1);
        assert_eq!(boards_for("STM32C5A3ZGT6").len(), 1);
        // H523 (compiled part) is NOT H533 — different board.
        assert!(boards_for("STM32H523RET6").is_empty());
        // Every board reserves SWD.
        for b in [&NUCLEO_G474RE, &NUCLEO_H533RE, &NUCLEO_C5A3ZG] {
            assert!(b.find(PinId { port: 'A', num: 13 }).is_some(), "{} reserves SWDIO", b.name);
        }
        // All three now have full Arduino maps (from their UMs).
        for b in [&NUCLEO_G474RE, &NUCLEO_H533RE, &NUCLEO_C5A3ZG] {
            assert_eq!(b.arduino_pins().len(), 22, "{} has A0-A5 + D0-D15", b.name);
        }
        // Board-specific analog headers verified against the UMs (they differ):
        let a2 = |b: &BoardProfile| b.pins.iter().find(|p| p.arduino == Some("A2")).unwrap().pin;
        assert_eq!(a2(&NUCLEO_G474RE), PinId { port: 'A', num: 4 }); // UM2505: A2=PA4
        assert_eq!(a2(&NUCLEO_H533RE), PinId { port: 'B', num: 1 }); // UM3121: A2=PB1
        assert_eq!(a2(&NUCLEO_C5A3ZG), PinId { port: 'A', num: 4 }); // UM3616: A2=PA4
        // C5A3ZG (LQFP144) reaches ports H/E on the Arduino header.
        let a0 = NUCLEO_C5A3ZG.pins.iter().find(|p| p.arduino == Some("A0")).unwrap().pin;
        assert_eq!(a0, PinId { port: 'H', num: 4 });
        // C5A3ZG's extra LEDs are on port G.
        assert!(NUCLEO_C5A3ZG.find(PinId { port: 'G', num: 1 }).is_some(), "LD2 red = PG1");
    }

    #[test]
    fn arduino_connector_restricts_usable_pins() {
        let b = &NUCLEO_G474RE;
        // A generous "bonded" set (pretend the whole footprint).
        let bonded: BTreeSet<PinId> = ["PA0", "PA1", "PA4", "PB1", "PC10", "PF0"]
            .iter()
            .map(|n| PinId::from_metapac(n).unwrap())
            .collect();
        let morpho = b.usable_pins(Connector::Morpho, &bonded);
        assert_eq!(morpho, bonded, "Morpho brings out everything");
        let arduino = b.usable_pins(Connector::Arduino, &bonded);
        // PA0/PA1/PA4 are Arduino pins; PB1/PC10/PF0 are Morpho-only.
        assert!(arduino.contains(&PinId::from_metapac("PA0").unwrap()));
        assert!(!arduino.contains(&PinId::from_metapac("PB1").unwrap()));
        assert!(!arduino.contains(&PinId::from_metapac("PF0").unwrap()));
    }
}
