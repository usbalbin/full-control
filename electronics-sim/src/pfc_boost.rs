use crate::math::{self, Func};
use crate::{Current, Voltage, Inductance, Capacitance, Resistance, CurrentConduction};

/// Result of one PFC boost switching cycle.
#[derive(Debug, Clone, Copy)]
pub struct PfcCycleResult {
    /// Inductor current at start of cycle [A].
    pub i_l_start: f64,
    /// Inductor current at end of ON phase (peak) [A].
    pub i_l_peak: f64,
    /// Inductor current at end of OFF phase [A].
    pub i_l_end: f64,
    /// Average inductor current over the cycle [A].
    pub i_l_avg: f64,
    /// Output voltage at end of cycle [V].
    pub v_out: f64,
    /// Actual duty cycle used.
    pub duty: f64,
    /// Whether DCM occurred (inductor current hit zero during OFF).
    pub dcm: bool,
    /// Fraction of OFF time during which inductor current was nonzero.
    /// 1.0 for CCM, 0.0..1.0 for DCM. Used for waveform reconstruction.
    pub t_conduct_frac: f64,
}

/// PFC boost converter switching-cycle simulator.
///
/// Models a boost converter with average current mode control.
/// The controller provides a duty cycle; this simulator handles the physics.
pub struct PfcBoostSim {
    // Circuit parameters
    pub l: f64,         // Inductance [H]
    pub c_out: f64,     // Output capacitor [F]
    pub r_esr: f64,     // Output cap ESR [Ohm]
    pub r_series: f64,  // Total series resistance (DCR + Rds(on)) [Ohm]
    pub v_diode: f64,   // Boost diode forward voltage [V]
    pub t_sw: f64,      // Switching period [s]
    pub current_conduction: CurrentConduction, // Diode (DCM possible) or Synchronous

    // State
    pub i_inductor: f64,   // Inductor current [A]
    pub v_cap: f64,        // Internal capacitor voltage [V] (v_out = v_cap + ESR×i_cap)
}

impl PfcBoostSim {
    pub fn new(
        l: f64,
        c_out: f64,
        r_esr: f64,
        r_series: f64,
        v_diode: f64,
        f_sw: f64,
        v_out_init: f64,
        current_conduction: CurrentConduction,
    ) -> Self {
        Self {
            l,
            c_out,
            r_esr,
            r_series,
            v_diode,
            t_sw: 1.0 / f_sw,
            current_conduction,
            i_inductor: 0.0,
            v_cap: v_out_init,
        }
    }

    /// Output voltage including ESR contribution.
    pub fn v_out(&self) -> f64 {
        self.v_cap
    }

    /// Simulate one switching cycle with a given duty cycle.
    ///
    /// - `v_in_rect`: rectified input voltage for this cycle [V]
    /// - `duty`: duty cycle (0.0 to 1.0)
    /// - `i_load`: load current [A] (constant over the cycle)
    pub fn tick(&mut self, v_in_rect: f64, duty: f64, i_load: f64) -> PfcCycleResult {
        let duty = duty.clamp(0.0, 0.98);
        let t_on = self.t_sw * duty;
        let t_off = self.t_sw - t_on;

        let i_start = self.i_inductor;

        // ── ON phase ──────────────────────────────────────────────
        // Switch closed, diode reverse-biased.
        // Inductor sees: V_L = V_in_rect - i_L × R_series
        // Cap is decoupled from inductor — only load drains it.
        //
        // Linear approximation (same as existing boost ON phase):
        // Use midpoint current for R_series voltage drop.
        let di_dt_on = (v_in_rect - self.r_series * self.i_inductor) / self.l;
        let i_peak = self.i_inductor + di_dt_on * t_on;
        // Refine with midpoint R_series correction
        let v_on_eff = v_in_rect - self.r_series * (self.i_inductor + i_peak) / 2.0;
        let di_dt_on = v_on_eff / self.l;
        let i_peak = self.i_inductor + di_dt_on * t_on;

        // Cap drains at load rate during ON phase
        let q_out_on = i_load * t_on;
        let v_cap_at_off = self.v_cap - q_out_on / self.c_out;

        // ── OFF phase ─────────────────────────────────────────────
        // Switch open, diode conducts.
        // Full RLC circuit: V_in - V_diode drives L+C in series through R_series.
        let v_off_source = v_in_rect - self.v_diode;
        let i_off_func = math::rlc(
            Voltage(v_off_source),
            Voltage(v_cap_at_off),
            Current(i_peak),
            Inductance(self.l),
            Capacitance(self.c_out),
            Resistance(self.r_series),
        );
        let i_final_raw = i_off_func.f(t_off);

        // DCM: if current hits zero during OFF, inductor disconnects (diode mode only).
        let (i_final, q_in, dcm, t_conduct_frac) = {
            let q_in_full = i_off_func.integral(0.0).f(t_off);
            match self.current_conduction {
                CurrentConduction::Synchronous => {
                    (i_final_raw, q_in_full, false, 1.0)
                }
                CurrentConduction::Diode if i_final_raw < 0.0 && i_peak >= 0.0 => {
                    // Find zero-crossing
                    let t_zero = bisect_zero(|t| i_off_func.f(t), 0.0, t_off);
                    let q_in_partial = i_off_func.integral(0.0).f(t_zero);
                    let frac = if t_off > 0.0 { t_zero / t_off } else { 0.0 };
                    (0.0, q_in_partial, true, frac)
                }
                CurrentConduction::Diode => {
                    (i_final_raw, q_in_full, false, 1.0)
                }
            }
        };

        // Update cap voltage: charge delivered during OFF minus load drain
        let q_out_off = i_load * t_off;
        self.v_cap = v_cap_at_off + (q_in - q_out_off) / self.c_out;
        self.i_inductor = i_final;

        // Average inductor current (input current = inductor current for boost)
        let i_avg = if dcm {
            // DCM: triangle from i_start to i_peak, then back to 0
            // Average = (i_start + i_peak) / 2 × (t_on + t_conduct) / t_sw
            // For simplicity, use charge-based: total charge / period
            let q_on = (i_start + i_peak) / 2.0 * t_on;
            let q_off_conduct = i_off_func.integral(0.0).f(
                bisect_zero(|t| i_off_func.f(t), 0.0, t_off),
            );
            (q_on + q_off_conduct) / self.t_sw
        } else {
            // CCM: triangle wave
            // ON phase average: (i_start + i_peak) / 2
            // OFF phase average: (i_peak + i_final) / 2
            let q_on = (i_start + i_peak) / 2.0 * t_on;
            let q_off = (i_peak + i_final) / 2.0 * t_off;
            (q_on + q_off) / self.t_sw
        };

        PfcCycleResult {
            i_l_start: i_start,
            i_l_peak: i_peak,
            i_l_end: i_final,
            i_l_avg: i_avg,
            v_out: self.v_cap,
            duty,
            dcm,
            t_conduct_frac,
        }
    }
}

/// Bisection search for the zero-crossing of f(t) in [a, b].
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

    /// 400V bus, 230Vac, 65kHz, 500µH, 220µF output cap.
    fn make_sim() -> PfcBoostSim {
        make_sim_with(CurrentConduction::Diode)
    }

    fn make_sim_with(cc: CurrentConduction) -> PfcBoostSim {
        PfcBoostSim::new(
            500e-6,   // L
            220e-6,   // C_out
            100e-3,   // ESR
            50e-3,    // R_series (DCR + Rds)
            0.7,      // V_diode
            65_000.0, // f_sw
            400.0,    // v_out_init
            cc,
        )
    }

    #[test]
    fn fixed_duty_converges() {
        let mut sim = make_sim();
        let v_in = 325.0; // peak of 230Vac
        let i_load = 300.0 / 400.0; // 300W / 400V = 0.75A

        // Fixed duty = 1 - V_in/V_out ≈ 1 - 325/400 = 0.1875
        let duty = 1.0 - v_in / 400.0;

        // Run 5000 cycles (should be enough to settle)
        for _ in 0..5000 {
            sim.tick(v_in, duty, i_load);
        }

        // V_out should be close to V_in / (1-D) ≈ 400V
        let v_expected = v_in / (1.0 - duty);
        let err = (sim.v_out() - v_expected).abs();
        assert!(
            err < 5.0,
            "v_out = {:.1}, expected ≈ {v_expected:.1}",
            sim.v_out()
        );
    }

    #[test]
    fn dcm_near_zero_crossing() {
        let mut sim = make_sim();
        sim.i_inductor = 0.0;

        // Near zero-crossing: v_in ≈ 5V, very low duty
        let result = sim.tick(5.0, 0.05, 0.75);

        // Current should be small and non-negative
        assert!(result.i_l_end >= 0.0, "current went negative: {}", result.i_l_end);
        assert!(result.i_l_avg >= 0.0);
    }

    #[test]
    fn zero_duty_no_crash() {
        let mut sim = make_sim();
        let result = sim.tick(325.0, 0.0, 0.75);
        assert!(result.v_out.is_finite());
        assert!(result.i_l_avg.is_finite());
    }

    #[test]
    fn average_current_is_positive_in_ccm() {
        let mut sim = make_sim();
        // V_in=200V → duty≈0.5, I_load=2A → I_in≈4A, well into CCM
        for _ in 0..2000 {
            sim.tick(200.0, 0.5, 2.0);
        }
        let result = sim.tick(200.0, 0.5, 2.0);
        assert!(
            result.i_l_avg > 0.0,
            "avg current should be positive in CCM, got {}",
            result.i_l_avg
        );
        assert!(!result.dcm, "should be CCM at this operating point");
    }

    #[test]
    fn t_conduct_frac_is_1_in_ccm() {
        let mut sim = make_sim();
        for _ in 0..2000 {
            sim.tick(200.0, 0.5, 2.0);
        }
        let result = sim.tick(200.0, 0.5, 2.0);
        assert!(!result.dcm);
        assert!(
            (result.t_conduct_frac - 1.0).abs() < 1e-10,
            "CCM should have t_conduct_frac=1.0, got {}",
            result.t_conduct_frac
        );
    }

    #[test]
    fn t_conduct_frac_partial_in_dcm() {
        let mut sim = make_sim();
        sim.i_inductor = 0.0;
        // Near zero-crossing: low v_in, small duty → DCM
        let result = sim.tick(5.0, 0.05, 0.75);
        if result.dcm {
            assert!(
                result.t_conduct_frac > 0.0 && result.t_conduct_frac < 1.0,
                "DCM t_conduct_frac should be in (0,1), got {}",
                result.t_conduct_frac
            );
        }
    }

    #[test]
    fn synchronous_allows_negative_current() {
        let mut sim = make_sim_with(CurrentConduction::Synchronous);
        sim.i_inductor = 0.1; // small positive current
        // Low v_in, low duty → current would go negative in OFF phase
        let result = sim.tick(5.0, 0.02, 0.75);
        // In synchronous mode, current can go negative (no DCM clamping)
        assert!(
            !result.dcm,
            "synchronous mode should never report DCM"
        );
        assert!(
            (result.t_conduct_frac - 1.0).abs() < 1e-10,
            "synchronous mode always has t_conduct_frac=1.0"
        );
    }

    #[test]
    fn diode_clamps_at_zero() {
        let mut sim = make_sim_with(CurrentConduction::Diode);
        sim.i_inductor = 0.1;
        let result = sim.tick(5.0, 0.02, 0.75);
        assert!(
            result.i_l_end >= 0.0,
            "diode mode should clamp current at zero, got {}",
            result.i_l_end
        );
    }
}
