//! Plant impedance source abstraction for the Bode panel.
//!
//! The buck Bode plot wants a single `(omega) -> Z(jω)` to drive both
//! the plant transfer function and the |Z_out| panel. We support two
//! sources for this curve:
//!
//! - **Analytic cap-bank.** A list of `electronics_sim::CapType` (the
//!   per-component C / ESR / ESL / count fields the user dials in via
//!   sliders). The composite impedance is computed in closed form by
//!   `CapBank::impedance_at`.
//! - **Tabulated PDN.** A `PdnZSweep` loaded from a JSON file emitted
//!   by `kicad_field_solver`'s `pdn-export` flag. Frequency-domain
//!   port impedance extracted from the actual PCB, parasitic-aware,
//!   resonances and all.
//!
//! Both cases produce the same `(re, im)` shape and an "effective
//! bulk capacitance" used by the plant transfer function to keep the
//! DC gain finite. Downstream code in `bode.rs` reads through this
//! enum and doesn't otherwise care which side it came from.

use electronics_sim::cap_bank::{CapBank, CapType};
use pdn_schema::PdnZSweep;

#[derive(Debug, Clone, Copy)]
pub enum PdnSource<'a> {
    Analytic {
        caps: &'a [CapType],
        c_total: f64,
    },
    Tabulated {
        sweep: &'a PdnZSweep,
    },
}

impl<'a> PdnSource<'a> {
    pub fn analytic(caps: &'a [CapType]) -> Self {
        let c_total = CapBank::total_capacitance(caps);
        Self::Analytic { caps, c_total }
    }

    pub fn tabulated(sweep: &'a PdnZSweep) -> Self {
        Self::Tabulated { sweep }
    }

    /// Composite port impedance Z(jω) at the requested angular frequency.
    pub fn impedance_at(&self, omega: f64) -> (f64, f64) {
        match self {
            PdnSource::Analytic { caps, .. } => CapBank::impedance_at(caps, omega),
            PdnSource::Tabulated { sweep } => sweep.z_at(omega),
        }
    }

    /// Effective bulk capacitance used to normalize the plant transfer
    /// function so that `Z_out × jω × C_eff` → 1 at DC.
    pub fn c_effective(&self) -> f64 {
        match self {
            PdnSource::Analytic { c_total, .. } => *c_total,
            PdnSource::Tabulated { sweep } => sweep.c_effective_farads,
        }
    }
}
