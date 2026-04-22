//! Vienna rectifier (3-level 3-phase PFC) switching-cycle simulator.
//!
//! Each phase is a boost stage that charges either the upper or lower half of a
//! split DC bus depending on the input voltage polarity.  The bidirectional
//! switch connects the phase to the DC bus midpoint during ON time; during OFF
//! time, current flows through the upper or lower diode to the respective rail.
//!
//! The model processes all three phases per tick, sharing the split-bus
//! capacitor state.

use crate::math::{self, Func};
use crate::pfc_boost::PfcCycleResult;
use crate::{Capacitance, Current, Inductance, Resistance, Voltage};

/// Result of one Vienna rectifier switching cycle (all 3 phases).
#[derive(Debug, Clone)]
pub struct ViennaCycleResult {
    /// Per-phase results (same format as boost PFC).
    pub phases: [PfcCycleResult; 3],
    /// Upper half-bus voltage \[V\].
    pub v_top: f64,
    /// Lower half-bus voltage \[V\].
    pub v_bot: f64,
}

impl ViennaCycleResult {
    /// Total DC bus voltage.
    pub fn v_dc(&self) -> f64 {
        self.v_top + self.v_bot
    }
}

/// Vienna rectifier 3-phase PFC simulator.
///
/// Models a 3-level PFC with split DC bus.  Each phase operates as a boost
/// converter to V_dc/2 (upper or lower cap depending on voltage polarity).
pub struct ViennaRectifierSim {
    // Per-phase parameters
    pub l: f64,
    pub r_series: f64,
    pub v_diode: f64,
    pub t_sw: f64,

    // Split DC bus
    pub c_half: f64,
    pub r_esr: f64,

    // Dead time
    /// Dead time between complementary switch transitions [s]. 0 = ideal.
    /// During dead time the phase is effectively disconnected from the
    /// midpoint, reducing the effective ON time of the bidirectional switch.
    pub t_dead: f64,

    // State
    pub i_l: [f64; 3],
    pub v_top: f64,
    pub v_bot: f64,
}

impl ViennaRectifierSim {
    pub fn new(
        l: f64,
        c_half: f64,
        r_esr: f64,
        r_series: f64,
        v_diode: f64,
        f_sw: f64,
        v_dc_init: f64,
    ) -> Self {
        Self {
            l,
            c_half,
            r_esr,
            r_series,
            v_diode,
            t_sw: 1.0 / f_sw,
            t_dead: 0.0,
            i_l: [0.0; 3],
            v_top: v_dc_init / 2.0,
            v_bot: v_dc_init / 2.0,
        }
    }

    /// Set dead-time parameter (builder pattern).
    pub fn with_dead_time(mut self, t_dead: f64) -> Self {
        self.t_dead = t_dead;
        self
    }

    /// Total DC bus voltage.
    pub fn v_dc(&self) -> f64 {
        self.v_top + self.v_bot
    }

    /// Simulate one switching cycle for all 3 phases.
    ///
    /// - `v_abc`: signed phase voltages \[V\] (not rectified)
    /// - `duties`: per-phase duty cycles (0..1)
    /// - `i_load`: DC load current \[A\]
    pub fn tick(
        &mut self,
        v_abc: [f64; 3],
        duties: [f64; 3],
        i_load: f64,
    ) -> ViennaCycleResult {
        // Track charge delivered to each cap half during OFF phases.
        // Load discharge is applied once at the end.
        let mut q_charge_top = 0.0_f64;
        let mut q_charge_bot = 0.0_f64;

        let mut results = [PfcCycleResult {
            i_l_start: 0.0,
            i_l_peak: 0.0,
            i_l_end: 0.0,
            i_l_avg: 0.0,
            v_out: 0.0,
            duty: 0.0,
            dcm: false,
            t_conduct_frac: 1.0,
            t_sw_actual: self.t_sw,
        }; 3];

        for phase in 0..3 {
            let v_in = v_abc[phase];
            let v_in_abs = v_in.abs();
            let positive = v_in >= 0.0;
            let duty = duties[phase].clamp(0.0, 0.98);
            // Dead-time adjustment: reduce effective ON time.  The Vienna's
            // bidirectional switch has two transitions per cycle, each with
            // t_dead during which the phase is disconnected from the midpoint.
            let t_on = (self.t_sw * duty - self.t_dead).max(0.0);
            let t_off = self.t_sw - t_on;

            // Current cap half voltage (includes charge from previous phases)
            let v_half = if positive {
                self.v_top + q_charge_top / self.c_half
            } else {
                self.v_bot + q_charge_bot / self.c_half
            };

            let i_start = self.i_l[phase];

            // ── ON phase ──────────────────────────────────────────
            // Switch closed → inductor connected to DC bus midpoint.
            // V_L = |v_in| - R × i  (midpoint is at neutral)
            let v_on_eff = v_in_abs - self.r_series * i_start;
            let di_dt_on = v_on_eff / self.l;
            let i_peak_est = i_start + di_dt_on * t_on;
            // Midpoint R correction
            let v_on_eff =
                v_in_abs - self.r_series * (i_start + i_peak_est.max(0.0)) / 2.0;
            let i_peak = (i_start + v_on_eff / self.l * t_on).max(0.0);

            // ── OFF phase ─────────────────────────────────────────
            // Diode conducts to upper or lower rail.
            // RLC: V_source = |v_in| - V_diode drives L + C_half.
            let v_off_source = v_in_abs - self.v_diode;
            let i_off_func = math::rlc(
                Voltage(v_off_source),
                Voltage(v_half),
                Current(i_peak),
                Inductance(self.l),
                Capacitance(self.c_half),
                Resistance(self.r_series),
            );
            let i_final_raw = i_off_func.f(t_off);

            // DCM: Vienna diodes block reverse current
            let (i_final, q_in, dcm, t_conduct_frac) =
                if i_final_raw < 0.0 && i_peak >= 0.0 {
                    let t_zero = bisect_zero(|t| i_off_func.f(t), 0.0, t_off);
                    let q_in_partial = i_off_func.integral(0.0).f(t_zero);
                    let frac = if t_off > 0.0 { t_zero / t_off } else { 0.0 };
                    (0.0, q_in_partial, true, frac)
                } else {
                    let q_in_full = i_off_func.integral(0.0).f(t_off);
                    (i_final_raw.max(0.0), q_in_full, false, 1.0)
                };

            // Accumulate charge to the correct cap half
            if positive {
                q_charge_top += q_in;
            } else {
                q_charge_bot += q_in;
            }

            self.i_l[phase] = i_final;

            // Average inductor current
            let i_avg = if dcm {
                let q_on = (i_start + i_peak) / 2.0 * t_on;
                let t_zero = bisect_zero(|t| i_off_func.f(t), 0.0, t_off);
                let q_off = i_off_func.integral(0.0).f(t_zero);
                (q_on + q_off) / self.t_sw
            } else {
                let q_on = (i_start + i_peak) / 2.0 * t_on;
                let q_off = (i_peak + i_final) / 2.0 * t_off;
                (q_on + q_off) / self.t_sw
            };

            results[phase] = PfcCycleResult {
                i_l_start: i_start,
                i_l_peak: i_peak,
                i_l_end: i_final,
                i_l_avg: i_avg,
                v_out: self.v_dc(), // snapshot (updated fully at end)
                duty,
                dcm,
                t_conduct_frac,
                t_sw_actual: self.t_sw,
            };
        }

        // Apply load discharge (load current flows through both caps in series)
        let q_load = i_load * self.t_sw;
        self.v_top += (q_charge_top - q_load) / self.c_half;
        self.v_bot += (q_charge_bot - q_load) / self.c_half;

        // Update v_out in results
        let v_dc = self.v_dc();
        for r in &mut results {
            r.v_out = v_dc;
        }

        ViennaCycleResult {
            phases: results,
            v_top: self.v_top,
            v_bot: self.v_bot,
        }
    }
}

/// Bisection search for zero-crossing of f(t) in \[a, b\].
fn bisect_zero(mut f: impl FnMut(f64) -> f64, mut a: f64, mut b: f64) -> f64 {
    for _ in 0..50 {
        let mid = (a + b) * 0.5;
        if f(mid) > 0.0 {
            a = mid;
        } else {
            b = mid;
        }
    }
    (a + b) * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    /// 800V bus, 400Vac L-L (230V L-N), 65kHz, 500µH, 220µF per half.
    fn make_sim() -> ViennaRectifierSim {
        ViennaRectifierSim::new(
            500e-6,   // L
            220e-6,   // C per half
            100e-3,   // ESR
            50e-3,    // R_series
            0.7,      // V_diode
            65_000.0, // f_sw
            800.0,    // V_dc init
        )
    }

    fn grid_v(v_pk: f64, f_hz: f64, t: f64) -> [f64; 3] {
        let w = TAU * f_hz;
        let offset = TAU / 3.0;
        [
            v_pk * (w * t).sin(),
            v_pk * (w * t - offset).sin(),
            v_pk * (w * t - 2.0 * offset).sin(),
        ]
    }

    #[test]
    fn initial_bus_balanced() {
        let sim = make_sim();
        assert_eq!(sim.v_top, 400.0);
        assert_eq!(sim.v_bot, 400.0);
        assert_eq!(sim.v_dc(), 800.0);
    }

    #[test]
    fn fixed_duty_runs_without_crash() {
        let mut sim = make_sim();
        let v_pk = 325.0; // 230V RMS L-N
        let f_hz = 50.0;
        let i_load = 800.0 / 800.0; // 800W at 800V

        for i in 0..10_000 {
            let t = i as f64 * sim.t_sw;
            let v = grid_v(v_pk, f_hz, t);
            // Fixed duty: 1 - V_pk / (V_dc/2) ≈ 1 - 325/400 = 0.1875
            let duty = 1.0 - v_pk / 400.0;
            let result = sim.tick(v, [duty; 3], i_load);

            assert!(result.v_dc().is_finite(), "v_dc not finite at step {i}");
            for (p, r) in result.phases.iter().enumerate() {
                assert!(r.i_l_avg.is_finite(), "i_avg not finite, phase {p} step {i}");
                assert!(r.i_l_end >= 0.0, "negative current, phase {p} step {i}");
            }
        }
    }

    #[test]
    fn bus_voltage_settles() {
        let mut sim = make_sim();
        let v_pk = 325.0;
        let f_hz = 50.0;
        let i_load = 1.0;

        // Run for ~100 line cycles
        let n = (100.0 / f_hz / sim.t_sw) as usize;
        for i in 0..n {
            let t = i as f64 * sim.t_sw;
            let v = grid_v(v_pk, f_hz, t);
            let duty = (1.0 - v_pk / (sim.v_dc() / 2.0)).clamp(0.0, 0.95);
            sim.tick(v, [duty; 3], i_load);
        }

        // V_dc should be around 2 × V_pk / (1-D) ≈ 800V with ideal duty
        // With losses it'll be somewhat lower
        assert!(
            sim.v_dc() > 500.0 && sim.v_dc() < 1200.0,
            "v_dc = {:.1} out of reasonable range",
            sim.v_dc()
        );
    }

    #[test]
    fn bus_stays_balanced() {
        let mut sim = make_sim();
        let v_pk = 325.0;
        let f_hz = 50.0;
        let i_load = 1.0;

        let n = (50.0 / f_hz / sim.t_sw) as usize;
        for i in 0..n {
            let t = i as f64 * sim.t_sw;
            let v = grid_v(v_pk, f_hz, t);
            let duty = (1.0 - v_pk / (sim.v_dc() / 2.0)).clamp(0.0, 0.95);
            sim.tick(v, [duty; 3], i_load);
        }

        // With balanced 3-phase input and equal duties, v_top ≈ v_bot
        let actual_imbalance = (sim.v_top - sim.v_bot).abs();
        assert!(
            actual_imbalance < sim.v_dc() * 0.05,
            "bus imbalance {actual_imbalance:.1}V too large (v_top={:.1}, v_bot={:.1})",
            sim.v_top,
            sim.v_bot
        );
    }

    #[test]
    fn dcm_near_zero_crossing() {
        let mut sim = make_sim();
        // At zero crossing, v_in ≈ 0 → DCM expected
        let v = [1.0, -280.0, 280.0]; // phase A near zero
        let result = sim.tick(v, [0.05, 0.2, 0.2], 1.0);

        // Phase A should enter DCM with very low current
        assert!(result.phases[0].i_l_avg < 1.0, "phase A avg current should be small near ZC");
        assert!(result.phases[0].i_l_end >= 0.0, "current should not go negative");
    }

    #[test]
    fn phase_currents_are_nonnegative() {
        let mut sim = make_sim();
        let v_pk = 325.0;
        let f_hz = 50.0;

        for i in 0..5000 {
            let t = i as f64 * sim.t_sw;
            let v = grid_v(v_pk, f_hz, t);
            let duty = 0.2;
            let result = sim.tick(v, [duty; 3], 1.0);

            for (p, r) in result.phases.iter().enumerate() {
                assert!(
                    r.i_l_end >= 0.0,
                    "negative current at phase {p}, step {i}: {}",
                    r.i_l_end
                );
            }
        }
    }
}
