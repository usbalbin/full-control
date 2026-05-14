//! In-process Conducted-EMC view of the input-current spectrum.
//!
//! Re-export of the `pdn-emc` crate (the same math the field-solver
//! side runs). Kept as a module here for ergonomics — the existing
//! `use crate::conducted_emc::*;` call sites continue to work.

pub use pdn_emc::{
    cispr22_class_a_qp_db_uv, cispr22_class_b_qp_db_uv, db_uv, lisn_dbuv_from_spectrum,
    lisn_impedance_cispr22, sweep_lisn_dbuv, sweep_lisn_dbuv_with_input_cap, verdict,
    CisprClass, InputCap, LISN_L_HENRY, LISN_R_OHM,
};

/// Convenience wrapper for the buck-sim-ui call sites that used
/// the local `lisn_dbuv(spec, input_cap, max_freq)` signature.
/// Same as [`pdn_emc::lisn_dbuv_from_spectrum`].
pub fn lisn_dbuv(
    spectrum: &pdn_schema::PortCurrentSpectrum,
    input_cap: Option<InputCap>,
    max_freq_hz: f64,
) -> Vec<(f64, f64)> {
    lisn_dbuv_from_spectrum(spectrum, input_cap, max_freq_hz)
}
