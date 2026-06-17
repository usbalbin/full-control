//! Raw chip-metadata schema. Populated by `tools/extract.rs` from
//! `stm32-metapac`; one generated file per supported MCU lives in
//! `src/mcu_data/`. The interpretation layer (timer-kind classification,
//! fast-channel rules, HRTIM fabric, DAC→COMP edges) lives in `mcu.rs`.

pub struct RawMcuData {
    pub name: &'static str,
    pub family: &'static str,
    pub peripherals: &'static [RawPeripheral],
    /// Physical DMA channel pools (supply side), one per controller. Channel
    /// counts are deduplicated; the constraint-selector's DMA capacity bound.
    pub dma_pools: &'static [DmaPoolDef],
}

/// One DMA controller and its channel count, e.g. `("DMA1", 8)` / `("LPDMA2", 4)`.
pub struct DmaPoolDef {
    pub name: &'static str,
    pub channels: u8,
}

pub struct RawPeripheral {
    pub name: &'static str,
    pub address: u64,
    /// metapac register-block id, e.g. "TIM_ADV", "TIM_GP32", "TIM_2CH_CMP".
    /// This is the normalized, family-agnostic peripheral-variant signal —
    /// `mcu.rs` derives timer kind / complementary / counter width from it
    /// instead of hard-coding per-number tables. `None` when metapac has no
    /// register block for the instance.
    pub block: Option<&'static str>,
    pub pins: &'static [RawPin],
    /// Inter-peripheral trigger routing as `(signal, source)` pairs straight
    /// from metapac, e.g. `("ADC_EXT_TRG0", "TIM1_CC1")`. Populated for
    /// families whose triggers metapac carries (rich on G4, empty on C5 — the
    /// C5 analog/break fabric lives in `fabric_data_c5` until upstream fills
    /// these in). Empty slice when absent.
    pub triggers: &'static [RawTrigger],
    /// DMA legs (one per peripheral signal that can use DMA), each normalized to
    /// the **set of controller pools** it may draw a channel from. The extractor
    /// collapses both DMA models into this shape: DMAMUX families (the signal
    /// names only a mux) fan out to every controller behind that mux; named-
    /// controller families (C5/H5 GPDMA/LPDMA) list the controllers directly. So
    /// the engine never branches per family. Empty when the peripheral has no DMA.
    pub dma: &'static [RawDmaLeg],
}

/// A peripheral DMA request: the signal (e.g. "RX", "TX", "CH1") paired with the
/// controller pools it may use. One leg consumes one channel from any one pool.
#[derive(Copy, Clone)]
pub struct RawDmaLeg {
    pub signal: &'static str,
    pub pools: &'static [&'static str],
}

/// One inter-peripheral trigger edge: `signal` (the consuming mux input, e.g.
/// "ADC_EXT_TRG0", "DAC_CH1_TRG", "BKIN") fed by `source` (e.g. "TIM1_CC1",
/// "TIM6_TRGO", "COMP1"). Verbatim metapac strings — interpreted in `mcu.rs`.
#[derive(Copy, Clone)]
pub struct RawTrigger {
    pub signal: &'static str,
    pub source: &'static str,
}

#[derive(Copy, Clone)]
pub struct RawPin {
    /// Pin name in metapac form, e.g. "PA0".
    pub pin: &'static str,
    /// Signal name within the peripheral, e.g. "TX", "MOSI", "INP3".
    pub signal: &'static str,
    /// Alternate-function number (0..15). `None` for analog signals that
    /// aren't AF-mapped (DAC outputs, ADC inputs, COMP/OPAMP I/O).
    pub af: Option<u8>,
}
