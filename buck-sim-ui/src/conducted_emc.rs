//! In-process Conducted-EMC view of the input-current spectrum.
//!
//! Mirror of `field_solver_rs::peec::emc::conducted` — the LISN
//! model and CISPR-22 quasi-peak limit tables are pure math with no
//! field-solver dependency, so we re-implement them here so the
//! "Conducted EMC" tab can render `dB(µV)` vs frequency directly
//! from the in-memory `PortCurrentSpectrum` produced by the
//! Simulation tab's spectrum exporter. No JSON round-trip, no
//! `field_solver_cli` shell-out, no temp files.
//!
//! When `pdn-emc` is extracted as a sibling crate (deferred until
//! the plane-green branch lands) this module will be replaced by a
//! `use pdn_emc::conducted::*;` re-export.

use std::f64::consts::PI;

use rustfft::num_complex::Complex64;
use pdn_schema::PortCurrentSpectrum;

/// Standard CISPR-22 LISN: 50 µH series inductor in parallel with
/// a 50 Ω measurement load.
const LISN_L_HENRY: f64 = 50e-6;
const LISN_R_OHM: f64 = 50.0;

/// LISN port impedance Z_LISN(ω). jωL in parallel with R.
fn lisn_impedance(omega: f64) -> Complex64 {
    let jwl = Complex64::new(0.0, omega * LISN_L_HENRY);
    let r = Complex64::new(LISN_R_OHM, 0.0);
    let denom = jwl + r;
    if denom.norm_sqr() == 0.0 {
        Complex64::new(0.0, 0.0)
    } else {
        jwl * r / denom
    }
}

/// Lumped input-cap network sitting between the converter's HS-FET
/// drain (where the spectrum is FFT'd) and the LISN port. Single
/// branch `R_esr + jωL_esl + 1/(jωC)`. Disabled when `c_farads = 0`.
#[derive(Debug, Clone, Copy, Default)]
pub struct InputCap {
    pub c_farads: f64,
    pub esr_ohm: f64,
    pub esl_henry: f64,
}

impl InputCap {
    fn impedance(&self, omega: f64) -> Complex64 {
        if self.c_farads <= 0.0 || omega <= 0.0 {
            return Complex64::new(1e15, 0.0); // effectively open
        }
        let z_c = Complex64::new(0.0, -1.0 / (omega * self.c_farads));
        let z_l = Complex64::new(0.0, omega * self.esl_henry);
        Complex64::new(self.esr_ohm, 0.0) + z_c + z_l
    }
}

/// CISPR-22 conducted-emissions limit class.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CisprClass {
    A,
    B,
}

impl CisprClass {
    pub fn label(&self) -> &'static str {
        match self {
            CisprClass::A => "Class A",
            CisprClass::B => "Class B",
        }
    }
    pub fn qp_limit_db_uv(&self, freq_hz: f64) -> f64 {
        match self {
            CisprClass::A => cispr22_class_a_qp_db_uv(freq_hz),
            CisprClass::B => cispr22_class_b_qp_db_uv(freq_hz),
        }
    }
}

fn cispr22_class_b_qp_db_uv(freq_hz: f64) -> f64 {
    let f = freq_hz.max(1e3);
    if f <= 500e3 {
        let f_clamped = f.max(150e3);
        let t = (f_clamped.ln() - 150e3_f64.ln()) / (500e3_f64.ln() - 150e3_f64.ln());
        66.0 - 10.0 * t
    } else if f <= 5e6 {
        56.0
    } else {
        60.0
    }
}

fn cispr22_class_a_qp_db_uv(freq_hz: f64) -> f64 {
    if freq_hz <= 500e3 {
        79.0
    } else {
        73.0
    }
}

/// `V_meas = Z_LISN · I_LISN`, with the optional input-cap divider
/// applied first: `I_LISN = I_DUT · Z_cap / (Z_cap + Z_LISN)`.
/// Returns dB(µV) per frequency, clamping non-finite / sub-fV values
/// to a -300 dB sentinel (same convention as the field-solver side).
pub fn lisn_dbuv(
    spectrum: &PortCurrentSpectrum,
    input_cap: Option<InputCap>,
    max_freq_hz: f64,
) -> Vec<(f64, f64)> {
    spectrum
        .freqs_hz
        .iter()
        .enumerate()
        .filter_map(|(k, &f)| {
            if f <= 0.0 || f > max_freq_hz {
                return None;
            }
            let omega = 2.0 * PI * f;
            let z_lisn = lisn_impedance(omega);
            let i_dut = Complex64::new(spectrum.i_re_amp[k], spectrum.i_im_amp[k]);
            let i_lisn = match input_cap {
                Some(cap) => {
                    let z_cap = cap.impedance(omega);
                    let denom = z_cap + z_lisn;
                    if denom.norm_sqr() == 0.0 {
                        Complex64::new(0.0, 0.0)
                    } else {
                        i_dut * z_cap / denom
                    }
                }
                None => i_dut,
            };
            let v = z_lisn * i_lisn;
            let mag = v.norm();
            let db = if !mag.is_finite() || mag < 1e-15 {
                -300.0
            } else {
                20.0 * (mag / 1e-6).log10()
            };
            Some((f, db))
        })
        .collect()
}

/// Verdict for a curve against a CISPR class: returns the
/// in-band peak `(freq_hz, dB(µV), margin_db)` and a `pass` flag.
/// In-band = 150 kHz – 30 MHz (the regulated range).
pub fn verdict(
    curve: &[(f64, f64)],
    class: CisprClass,
) -> Option<(f64, f64, f64, bool)> {
    let mut worst_margin = f64::NEG_INFINITY;
    let mut worst_f = 0.0;
    let mut worst_db = f64::NEG_INFINITY;
    let mut any_in_band = false;
    for &(f, db) in curve {
        if !(150e3..=30e6).contains(&f) {
            continue;
        }
        any_in_band = true;
        let lim = class.qp_limit_db_uv(f);
        let margin = db - lim;
        if margin > worst_margin {
            worst_margin = margin;
            worst_f = f;
            worst_db = db;
        }
    }
    if !any_in_band {
        return None;
    }
    Some((worst_f, worst_db, worst_margin, worst_margin <= 0.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use pdn_schema::ControllerFlavor;

    fn pure_tone_spectrum(freq_hz: f64, amp_a: f64) -> PortCurrentSpectrum {
        PortCurrentSpectrum::new(
            "test",
            ControllerFlavor::HostF32,
            "VIN",
            "GND",
            vec![freq_hz],
            vec![amp_a],
            vec![0.0],
            amp_a,
        )
    }

    #[test]
    fn dbuv_of_100ma_at_1mhz_through_50ohm_is_about_134() {
        // 100 mA · 50 Ω (LISN at 1 MHz is ~50 Ω) = 5 V → 134 dBµV.
        let spec = pure_tone_spectrum(1e6, 0.1);
        let out = lisn_dbuv(&spec, None, 30e6);
        assert_eq!(out.len(), 1);
        assert!((out[0].1 - 134.0).abs() < 1.0, "got {} dBµV", out[0].1);
    }

    #[test]
    fn input_cap_attenuates_high_freq() {
        let cap = InputCap {
            c_farads: 100e-6,
            esr_ohm: 5e-3,
            esl_henry: 10e-9,
        };
        let spec = pure_tone_spectrum(10e6, 1.0);
        let without = lisn_dbuv(&spec, None, 30e6)[0].1;
        let with_cap = lisn_dbuv(&spec, Some(cap), 30e6)[0].1;
        let atten = without - with_cap;
        assert!(
            (30.0..=50.0).contains(&atten),
            "expected 30–50 dB attenuation, got {} dB",
            atten,
        );
    }

    #[test]
    fn class_a_above_class_b() {
        for f in [200e3, 1e6, 10e6] {
            assert!(
                CisprClass::A.qp_limit_db_uv(f) > CisprClass::B.qp_limit_db_uv(f),
                "Class A should be above Class B at {} Hz",
                f,
            );
        }
    }

    #[test]
    fn verdict_pass_below_limit() {
        // 1 µA at 1 MHz → 1µA · 50 Ω ≈ 50 µV → 34 dBµV, well below
        // Class B 56 dBµV. Margin ≈ -22 dB → PASS.
        let spec = pure_tone_spectrum(1e6, 1e-6);
        let curve = lisn_dbuv(&spec, None, 30e6);
        let v = verdict(&curve, CisprClass::B).unwrap();
        assert!(v.3, "expected PASS, margin = {} dB", v.2);
    }

    #[test]
    fn verdict_fail_above_limit() {
        // 1 A → 1 · 50 = 50 V → 154 dBµV >> Class B 56.
        let spec = pure_tone_spectrum(1e6, 1.0);
        let curve = lisn_dbuv(&spec, None, 30e6);
        let v = verdict(&curve, CisprClass::B).unwrap();
        assert!(!v.3, "expected FAIL");
        assert!(v.2 > 0.0);
    }
}
