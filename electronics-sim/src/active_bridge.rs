//! 6-switch active bridge (B6) voltage source rectifier switching-cycle simulator.
//!
//! Each of the 3 phases has a half-bridge leg (top switch to V_dc+, bottom
//! switch to V_dc−).  The converter line-to-neutral voltage at each phase is
//! `v_conv_k = (duty_k − 0.5) × V_dc`.  The difference between the grid
//! voltage and the converter voltage drives current through the filter inductor.
//!
//! Unlike the Vienna rectifier, the 6-switch bridge can handle bidirectional
//! power flow and uses a single (non-split) DC bus capacitor.


/// Result of one active bridge switching cycle (all 3 phases).
#[derive(Debug, Clone)]
pub struct ActiveBridgeCycleResult {
    /// Per-phase results.
    pub phases: [ActiveBridgePhaseResult; 3],
    /// DC bus voltage after this cycle \[V\].
    pub v_dc: f64,
}

/// Per-phase result for one switching cycle.
#[derive(Debug, Clone, Copy)]
pub struct ActiveBridgePhaseResult {
    /// Phase current at start of cycle \[A\] (signed).
    pub i_start: f64,
    /// Phase current at end of cycle \[A\] (signed).
    pub i_end: f64,
    /// Average phase current over cycle \[A\] (signed).
    pub i_avg: f64,
    /// Actual duty cycle used.
    pub duty: f64,
}

/// 6-switch voltage source rectifier simulator.
///
/// Models a fully-controlled 3-phase bridge with bidirectional current flow.
/// Designed for use with a d-q synchronous frame controller.
pub struct ActiveBridgeSim {
    // Per-phase
    pub l: f64,
    pub r_series: f64,
    pub t_sw: f64,

    // DC bus
    pub c_out: f64,
    pub r_esr: f64,

    // Dead time
    /// Dead time between complementary switch transitions [s]. 0 = ideal.
    pub t_dead: f64,
    /// Body diode forward voltage [V]. Used during dead-time intervals.
    pub v_body_diode: f64,

    // State
    pub i_l: [f64; 3],
    pub v_cap: f64,
}

impl ActiveBridgeSim {
    pub fn new(
        l: f64,
        c_out: f64,
        r_esr: f64,
        r_series: f64,
        f_sw: f64,
        v_dc_init: f64,
    ) -> Self {
        Self {
            l,
            c_out,
            r_esr,
            r_series,
            t_sw: 1.0 / f_sw,
            t_dead: 0.0,
            v_body_diode: 0.0,
            i_l: [0.0; 3],
            v_cap: v_dc_init,
        }
    }

    /// Set dead-time parameters (builder pattern).
    pub fn with_dead_time(mut self, t_dead: f64, v_body_diode: f64) -> Self {
        self.t_dead = t_dead;
        self.v_body_diode = v_body_diode;
        self
    }

    /// DC bus voltage.
    pub fn v_dc(&self) -> f64 {
        self.v_cap
    }

    /// Simulate one switching cycle for all 3 phases.
    ///
    /// - `v_abc`: signed grid phase voltages \[V\]
    /// - `duties`: per-phase duty cycles (0..1).
    ///   Convention: `duty=1` → top switch ON → phase connected to V_dc+.
    ///   `duty=0` → bottom switch ON → phase connected to V_dc−.
    ///   `v_conv_k = (duty_k - 0.5) × 2 × V_dc` for centered modulation, or
    ///   equivalently `v_conv_k = duty_k × V_dc` for rail-to-rail.
    /// - `i_load`: DC load current \[A\]
    pub fn tick(
        &mut self,
        v_abc: [f64; 3],
        duties: [f64; 3],
        i_load: f64,
    ) -> ActiveBridgeCycleResult {
        // Net charge into DC bus from all phases
        let mut q_dc_total = 0.0_f64;

        let mut results = [ActiveBridgePhaseResult {
            i_start: 0.0,
            i_end: 0.0,
            i_avg: 0.0,
            duty: 0.0,
        }; 3];

        // Per-phase converter voltages (line-to-negative-rail).
        // Dead time shrinks the effective duty toward 0.5: each transition
        // has t_dead during which neither switch is on and the body diode
        // conducts.  The net effect moves the duty toward 0.5 (zero
        // converter voltage) by d_loss per edge.
        let d_loss = self.t_dead / self.t_sw;
        let clamp_dead = |d_raw: f64| -> f64 {
            let d_clamped = d_raw.clamp(0.0, 1.0);
            if d_clamped > 0.5 {
                (d_clamped - d_loss).max(0.5)
            } else {
                (d_clamped + d_loss).min(0.5)
            }
        };
        let d = [
            clamp_dead(duties[0]),
            clamp_dead(duties[1]),
            clamp_dead(duties[2]),
        ];
        // Common-mode: converter neutral = average of phase voltages
        let v_conv_cm = ((d[0] + d[1] + d[2]) / 3.0 - 0.5) * self.v_cap;

        for phase in 0..3 {
            let v_grid = v_abc[phase];
            let duty = d[phase];

            let i_start = self.i_l[phase];

            // Converter line-to-neutral voltage (common-mode rejected).
            // This enforces the 3-wire constraint (Σi = 0).
            let v_conv = (duty - 0.5) * self.v_cap - v_conv_cm;
            let v_net = v_grid - v_conv;

            // di/dt = (v_net - R×i) / L
            // Linear with midpoint correction:
            let di_dt = (v_net - self.r_series * i_start) / self.l;
            let i_end_est = i_start + di_dt * self.t_sw;
            let v_net_eff =
                v_net - self.r_series * (i_start + i_end_est) / 2.0;
            let i_end = i_start + v_net_eff / self.l * self.t_sw;

            // Average current (linear ramp)
            let i_avg = (i_start + i_end) / 2.0;

            // Power delivered to DC bus from this phase:
            // P = v_conv × i_avg, where v_conv = (duty - 0.5) × V_dc
            // Charge = P × t_sw / V_dc = (duty - 0.5) × i_avg × t_sw
            q_dc_total += (duty - 0.5) * i_avg * self.t_sw;

            self.i_l[phase] = i_end;

            results[phase] = ActiveBridgePhaseResult {
                i_start,
                i_end,
                i_avg,
                duty,
            };
        }

        // Update DC bus voltage
        let q_load = i_load * self.t_sw;
        self.v_cap += (q_dc_total - q_load) / self.c_out;

        // Body-diode conduction loss during dead time.  During each
        // dead-time interval, current freewheels through the body diode
        // of the complementary switch, dissipating v_body_diode × |i|.
        // Total per phase per cycle: v_body_diode × |i_avg| × 2 × t_dead.
        // This energy comes from the DC bus.
        if self.t_dead > 0.0 && self.v_body_diode > 0.0 {
            let mut p_dead_total = 0.0;
            for phase in 0..3 {
                p_dead_total +=
                    self.v_body_diode * results[phase].i_avg.abs() * 2.0 * self.t_dead;
            }
            // p_dead_total is energy per cycle [J]; convert to charge drained
            // from the DC bus: Q = E / V_dc.
            let q_dead = p_dead_total / self.v_cap.max(1.0);
            self.v_cap -= q_dead / self.c_out;
        }

        ActiveBridgeCycleResult {
            phases: results,
            v_dc: self.v_cap,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    /// 700V bus, 400Vac L-L, 20kHz, 1mH, 470µF.
    fn make_sim() -> ActiveBridgeSim {
        ActiveBridgeSim::new(
            1e-3,     // L per phase
            470e-6,   // C_out
            50e-3,    // ESR
            100e-3,   // R_series
            20_000.0, // f_sw
            700.0,    // V_dc init
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
    fn runs_without_crash() {
        let mut sim = make_sim();
        let v_pk = 325.0;
        let f_hz = 50.0;

        for i in 0..10_000 {
            let t = i as f64 * sim.t_sw;
            let v = grid_v(v_pk, f_hz, t);
            // Duty ≈ 0.5 + v_grid / V_dc (feedforward-like)
            let duties = [
                (0.5 + v[0] / sim.v_dc()).clamp(0.0, 1.0),
                (0.5 + v[1] / sim.v_dc()).clamp(0.0, 1.0),
                (0.5 + v[2] / sim.v_dc()).clamp(0.0, 1.0),
            ];
            let result = sim.tick(v, duties, 1.0);
            assert!(result.v_dc.is_finite(), "v_dc not finite at step {i}");
        }
    }

    #[test]
    fn bus_voltage_stays_reasonable() {
        let mut sim = make_sim();
        let v_pk = 325.0;
        let f_hz = 50.0;

        for i in 0..20_000 {
            let t = i as f64 * sim.t_sw;
            let v = grid_v(v_pk, f_hz, t);
            let duties = [
                (0.5 + v[0] / sim.v_dc()).clamp(0.0, 1.0),
                (0.5 + v[1] / sim.v_dc()).clamp(0.0, 1.0),
                (0.5 + v[2] / sim.v_dc()).clamp(0.0, 1.0),
            ];
            sim.tick(v, duties, 1.0);
        }

        assert!(
            sim.v_dc() > 300.0 && sim.v_dc() < 1500.0,
            "v_dc = {:.1} out of reasonable range",
            sim.v_dc()
        );
    }

    #[test]
    fn balanced_currents_sum_to_zero() {
        let mut sim = make_sim();
        let v_pk = 325.0;
        let f_hz = 50.0;

        // Let it settle
        for i in 0..10_000 {
            let t = i as f64 * sim.t_sw;
            let v = grid_v(v_pk, f_hz, t);
            let duties = [
                (0.5 + v[0] / sim.v_dc()).clamp(0.0, 1.0),
                (0.5 + v[1] / sim.v_dc()).clamp(0.0, 1.0),
                (0.5 + v[2] / sim.v_dc()).clamp(0.0, 1.0),
            ];
            let result = sim.tick(v, duties, 1.0);

            // Balanced 3-phase: i_a + i_b + i_c ≈ 0
            if i > 5000 {
                let i_sum = result.phases[0].i_avg
                    + result.phases[1].i_avg
                    + result.phases[2].i_avg;
                let i_max = result
                    .phases
                    .iter()
                    .map(|p| p.i_avg.abs())
                    .fold(0.0_f64, f64::max);
                if i_max > 0.1 {
                    assert!(
                        i_sum.abs() < i_max * 0.2,
                        "sum = {i_sum:.3}, max = {i_max:.3} at step {i}"
                    );
                }
            }
        }
    }

    #[test]
    fn bidirectional_current() {
        let mut sim = make_sim();
        let v_pk = 325.0;
        let f_hz = 50.0;

        let mut saw_positive = false;
        let mut saw_negative = false;

        for i in 0..10_000 {
            let t = i as f64 * sim.t_sw;
            let v = grid_v(v_pk, f_hz, t);
            let duties = [
                (0.5 + v[0] / sim.v_dc()).clamp(0.0, 1.0),
                (0.5 + v[1] / sim.v_dc()).clamp(0.0, 1.0),
                (0.5 + v[2] / sim.v_dc()).clamp(0.0, 1.0),
            ];
            let result = sim.tick(v, duties, 1.0);

            if result.phases[0].i_avg > 0.1 {
                saw_positive = true;
            }
            if result.phases[0].i_avg < -0.1 {
                saw_negative = true;
            }
        }

        assert!(saw_positive, "phase A never had positive current");
        assert!(saw_negative, "phase A never had negative current");
    }

    #[test]
    fn dead_time_runs_without_crash() {
        // Verify the simulator runs stably with dead time enabled.
        // Uses fixed duty (not feedforward) to avoid runaway bus voltage.
        let mut sim = make_sim().with_dead_time(500e-9, 1.5);
        let v_pk = 325.0;
        let f_hz = 50.0;

        for i in 0..20_000 {
            let t = i as f64 * sim.t_sw;
            let v = grid_v(v_pk, f_hz, t);
            // Fixed duty from initial voltage (no feedforward loop)
            let duties = [
                (0.5 + v[0] / 700.0).clamp(0.0, 1.0),
                (0.5 + v[1] / 700.0).clamp(0.0, 1.0),
                (0.5 + v[2] / 700.0).clamp(0.0, 1.0),
            ];
            let result = sim.tick(v, duties, 1.0);
            assert!(result.v_dc.is_finite(), "v_dc not finite at step {i}");
        }

        assert!(
            sim.v_dc() > 100.0,
            "v_dc = {:.1} should be positive",
            sim.v_dc()
        );
    }

    #[test]
    fn dead_time_changes_bus_voltage() {
        // With the same fixed duties, dead time should produce a
        // different DC bus voltage than the ideal case because the
        // effective converter voltage is reduced.
        let mut sim_ideal = make_sim();
        let mut sim_dead = make_sim().with_dead_time(1e-6, 1.5);

        let v_pk = 325.0;
        let f_hz = 50.0;

        for i in 0..20_000 {
            let t = i as f64 * sim_ideal.t_sw;
            let v = grid_v(v_pk, f_hz, t);
            // Fixed duty -- no feedforward, so dead time effect is visible.
            let duties = [
                (0.5 + v[0] / 700.0).clamp(0.0, 1.0),
                (0.5 + v[1] / 700.0).clamp(0.0, 1.0),
                (0.5 + v[2] / 700.0).clamp(0.0, 1.0),
            ];
            sim_ideal.tick(v, duties, 1.0);
            sim_dead.tick(v, duties, 1.0);
        }

        // Dead time modifies the effective duty, so the bus voltage
        // should differ from the ideal case.
        let delta = (sim_dead.v_dc() - sim_ideal.v_dc()).abs();
        assert!(
            delta > 1.0,
            "dead time should noticeably change v_dc: ideal={:.1} V, dead={:.1} V, delta={:.1} V",
            sim_ideal.v_dc(),
            sim_dead.v_dc(),
            delta
        );
    }
}
