pub mod math;

#[cfg(feature = "rerun")]
use rerun::RecordingStream;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub};
use full_control::buck_boost::Mode as Topology;

use crate::math::Line;
use math::Func;

#[cfg(feature = "rerun")]
pub fn plot(
    rec: &RecordingStream,
    sim: &CurrentModeConverter,
    t_on: Time,
    i_max: Current,
    v_in: Voltage,
    i_out: Current,
    time: &mut Time,
) {
    rec.set_timestamp_secs_since_epoch("time", (*time + t_on).0 as f64 - 10e-9);
    rec.log("sw", &rerun::Scalars::new([1.0])).unwrap();

    rec.set_timestamp_secs_since_epoch("time", (*time + t_on).0 as f64);
    rec.log("sw", &rerun::Scalars::new([0.0])).unwrap();
    rec.log("i_inductor", &rerun::Scalars::new([i_max.0 as f64]))
        .unwrap();

    *time += sim.parameters.period;
    rec.set_timestamp_secs_since_epoch("time", (time.0 - 10e-9) as f64);
    rec.log("sw", &rerun::Scalars::new([0.0])).unwrap();

    rec.set_timestamp_secs_since_epoch("time", time.0 as f64);
    rec.log("sw", &rerun::Scalars::new([1.0])).unwrap();
    rec.log(
        "i_inductor",
        &rerun::Scalars::new([sim.i_inductor.0 as f64]),
    )
    .unwrap();

    rec.log("v_in", &rerun::Scalars::new([v_in.0 as f64]))
        .unwrap();
    rec.log("v_in", &rerun::Scalars::new([v_in.0 as f64]))
        .unwrap();
    rec.log("v_out", &rerun::Scalars::new([sim.v_out.0 as f64]))
        .unwrap();
    rec.log("i_out", &rerun::Scalars::new([i_out.0 as f64]))
        .unwrap();
    rec.log(
        "d",
        &rerun::Scalars::new([(t_on.0 / sim.parameters.period.0) as f64]),
    )
    .unwrap();
}

pub type T = f64;

#[derive(Clone, Copy)]
pub struct MyThing {
    pub buck: CurrentModeConverter,
    amp_per_lsb: T,
    i_at_0lsb: Current,
}

impl MyThing {
    pub fn new(parameters: Parameters, amp_per_lsb: T, i_at_0lsb: Current) -> Self {
        Self {
            buck: CurrentModeConverter::new(parameters, Topology::Buck),
            amp_per_lsb,
            i_at_0lsb,
        }
    }

    pub fn tick(
        &mut self,
        v_in: Voltage,
        trip_current: u16,
        i_out: impl FnMut(Voltage) -> Current,
    ) -> (Time, Current) {
        // 4095 -> 20
        // 0 -> -20
        let trip_current = Current(T::from(trip_current) * self.amp_per_lsb) + self.i_at_0lsb;
        self.buck.tick(v_in, trip_current, i_out)
    }
}

#[derive(Copy, Clone)]
pub struct Parameters {
    pub period: Time,
    // This should normally be negative
    pub slope_amp_per_sec: T,

    /// Lumped series resistance (inductor DCR + conducting switch R_dson).
    /// Models conduction losses in both ON and OFF phases.
    pub r_series: Resistance,

    /// Equivalent series resistance of the output capacitor.
    /// Used by `v_sensed()` to reproduce the ESR voltage seen by the ADC.
    pub r_esr: Resistance,

    pub c_out: Capacitance,
    pub l_inductor: Inductance,

    /// Input capacitance.  Zero disables V_in ripple modelling (ideal stiff source).
    pub c_in: Capacitance,
    /// Equivalent series resistance of the input capacitor.
    /// Does not affect the charge-balance physics (second-order), but is used by
    /// `compute_v_in_ripple()` to estimate the resistive voltage spike at the
    /// converter input terminals at the switching frequency.
    pub r_esr_cin: Resistance,
    /// Thevenin source resistance seen by the input capacitor.
    /// Zero means the cap is instantly recharged to the source voltage each OFF phase.
    pub r_in: Resistance,

    /// Input cable / trace inductance.  Zero disables LC input-filter modelling
    /// (falls back to the simpler RC recharge approximation).
    pub l_in: Inductance,

    /// RC time constant of the current-sense filter (s), derived from the bandwidth
    /// passed to `new()`.  `0.0` = ideal (no filtering).
    ///
    /// A non-zero value delays the sensed current: the comparator trips slightly
    /// later than ideal, so the actual peak current is higher by ≈ `m × τ`
    /// (where `m` is the inductor current slope).
    pub tau_current_sense: Time,

    /// RC time constant of the slope-compensation DAC output filter (s), derived
    /// from the bandwidth passed to `new()`.  `0.0` = ideal.
    ///
    /// A non-zero value causes the slope compensation ramp to build up gradually
    /// at the start of each switching cycle, reducing its sub-harmonic suppression
    /// effectiveness early in the ON phase.
    pub tau_dac: Time,

    /// Comparator propagation delay (s).  `0.0` = ideal.
    ///
    /// After the (filtered) current-sense signal crosses the trip threshold the
    /// comparator takes `t_prop_delay` seconds to produce an output edge.  During
    /// this dead time the inductor current keeps ramping at `di/dt`, so the actual
    /// peak current is higher by ≈ `m × t_prop_delay` and the ON phase is extended
    /// by the same amount.
    pub t_prop_delay: Time,

    /// ZOH DAC sample period (s), derived from `f_dac_sample_hz` passed to `new()`.
    /// `0.0` = continuous ramp (ideal or LP-filtered, depending on `tau_dac`).
    ///
    /// When non-zero, the slope compensation is updated once every `t_dac_sample`
    /// seconds and held constant between updates (zero-order hold).  The effective
    /// trip threshold advances in discrete steps rather than as a smooth ramp.
    pub t_dac_sample: Time,
}

impl Parameters {
    pub fn bw_to_tau(bw: T) -> Time {
        if bw > 0.0 {
            Time(1.0 / (2.0 * std::f64::consts::PI * bw))
        } else {
            Time(0.0)
        }
    }
}

#[derive(Clone, Copy)]
pub struct CurrentModeConverter {
    pub parameters: Parameters, // TODO: Make this private
    pub v_out: Voltage,

    pub i_inductor: Current,

    pub topology: Topology,

    /// Actual voltage at the converter input terminal.
    /// Lazy-initialised to the source voltage on the first `tick()` call.
    pub v_in_cap: Voltage,

    /// Current flowing through `l_in` at the end of the previous cycle.
    pub i_in_cap: Current,

    /// Estimated peak-to-peak voltage ripple at the converter input terminals at
    /// the switching frequency, computed inside `tick()` and valid to read after it
    /// returns.  Useful as a per-cycle conducted-emissions indicator.
    pub v_in_ripple_est: Voltage,

    /// Current-sense filter state (A) — the LP-filtered inductor current at the end
    /// of the previous switching cycle, carried forward as the initial condition for
    /// the next cycle's filter response.  Maintained automatically by `tick()`.
    i_cs: Current,

    // ── Snapshot fields for waveform plotting ────────────────────────────────
    // These are written at the start of every `tick()` so that callers can
    // reconstruct the intra-cycle waveforms after the tick has overwritten state.
    /// Inductor valley current at the start of the last `tick()` call.
    pub i_inductor_prev: Current,
    /// Current-sense filter state at the start of the last `tick()` call.
    pub i_cs_prev: Current,
    /// ON-phase `di/dt` (A/s) computed during the last `tick()` call.
    pub last_di_dt_on: T,
}

impl CurrentModeConverter {
    /// Output voltage as seen by the ADC at the start of the switching cycle
    /// (valley of the inductor current ripple), including the ESR contribution.
    ///
    /// At the sample instant the cap current is `i_valley − i_load`, so:
    ///
    ///   V_sense = V_cap + R_esr × (i_valley − i_load)
    ///
    /// Pass the load current from the previous cycle's tick closure as `i_load`.
    pub fn v_sensed(&self, i_load: Current) -> Voltage {
        Voltage(self.v_out.0 + self.parameters.r_esr.0 * (self.i_inductor.0 - i_load.0))
    }

    pub fn new(parameters: Parameters, topology: Topology) -> Self {
        Self {
            parameters,
            v_out: Voltage(0.0),
            i_inductor: Current(0.0),
            topology,
            v_in_cap: Voltage(0.0),
            i_in_cap: Current(0.0),
            v_in_ripple_est: Voltage(0.0),
            i_cs: Current(0.0),

            i_inductor_prev: Current(0.0),
            i_cs_prev: Current(0.0),
            last_di_dt_on: 0.0,
        }
    }

    /// 0                     t_on       t_period
    /// |                       '           |               '                   |
    /// |                       '           |               '                   |
    ///-|-----------------------*-----------|---------------*-------------------|--- i_trip
    /// |                  *    '  *        |           *   '  *                |
    /// |             *         '     *     |       *       '     *             |
    /// |        *              '        *  |   *           '        *          |
    /// |---*-------------------'-----------*---------------'-----------*-------|--- 0
    /// *                       '           |               '              *    |
    /// |                       '           |               '                 * |
    ///
    ///
    pub fn tick(
        &mut self,
        v_in: Voltage,
        trip_current: Current,
        mut i_out: impl FnMut(Voltage) -> Current,
    ) -> (Time, Current) {
        // Lazy-initialise the input cap to the source voltage on the first call.
        if self.parameters.c_in.0 > 0.0 && self.v_in_cap.0 == 0.0 {
            self.v_in_cap = v_in;
        }
        // When C_in = 0 the source is ideal (no droop); otherwise use the tracked cap voltage.
        let v_eff = if self.parameters.c_in.0 > 0.0 {
            self.v_in_cap
        } else {
            v_in
        };

        let tau_cs = self.parameters.tau_current_sense;
        let tau_dac = self.parameters.tau_dac;

        match self.topology {
            Topology::Buck => {
                let v_ind_on = v_eff - self.v_out;

                // V = L di/dt
                // di/dt = V/L
                let di_dt_on = v_ind_on.0 / self.parameters.l_inductor.0;

                let i_on_func = math::rlc(
                    v_eff,
                    self.v_out,
                    self.i_inductor,
                    self.parameters.l_inductor,
                    self.parameters.c_out,
                    self.parameters.r_series,
                );

                let t_on_guess = Time(
                    (trip_current - self.i_inductor).0
                        / (di_dt_on - self.parameters.slope_amp_per_sec),
                );
                let t_on = if tau_cs.0 > 0.0 || tau_dac.0 > 0.0 {
                    Time(filtered_trip_time(
                        self.i_inductor,
                        di_dt_on,
                        self.i_cs,
                        tau_cs,
                        trip_current,
                        self.parameters.slope_amp_per_sec,
                        tau_dac,
                        t_on_guess,
                    ))
                } else {
                    Time(
                        i_on_func
                            .intersects_at(
                                Line {
                                    k: self.parameters.slope_amp_per_sec,
                                    m: trip_current.0,
                                },
                                t_on_guess.0,
                            )
                            .unwrap(),
                    )
                };

                // Comparator propagation delay: switch stays ON for t_prop extra after threshold.
                let t_on = t_on + self.parameters.t_prop_delay;

                let q_old = 0.0;

                // Total charge in capacitor at end of ON-phase
                let q_on;

                // current at end of the ON-phase
                let i_max;
                if t_on.0 < 0.0 {
                    i_max = self.i_inductor;
                    q_on = q_old;
                } else if t_on.0 > self.parameters.period.0 {
                    // Clamp to 100%
                    i_max = Current(i_on_func.f(self.parameters.period.0));
                    q_on = i_on_func.integral(0.0).f(self.parameters.period.0) + q_old;
                } else {
                    i_max = Current(i_on_func.f(t_on.0));
                    q_on = i_on_func.integral(0.0).f(t_on.0) + q_old;
                }

                let t_on = Time(t_on.0.clamp(0.0, self.parameters.period.0));
                let t_off = self.parameters.period - t_on;

                // EMI estimate: computed while i_inductor / i_in_cap are still start-of-cycle.
                self.v_in_ripple_est = self.compute_v_in_ripple(i_max, t_on);

                // Sample load current once at the initial output voltage.  The ON/OFF
                // voltages differ by at most a few mV of ripple, so two samples would give
                // negligibly different results while causing stateful loads (e.g. Battery) to
                // advance their internal state twice per cycle.
                let load_current = i_out(self.v_out);
                let q_out_on = load_current.0 * t_on.0;
                // Net cap voltage at start of OFF phase: inductor charge in minus load drain.
                let v_out_at_off =
                    self.v_out + Voltage((q_on - q_out_on) / self.parameters.c_out.0);

                let i_off_func = math::rlc(
                    Voltage(0.0),
                    v_out_at_off,
                    i_max,
                    self.parameters.l_inductor,
                    self.parameters.c_out,
                    self.parameters.r_series,
                );
                let i_final = Current(i_off_func.f(t_off.0));

                let q_off = i_off_func.integral(0.0).f(t_off.0);
                let q_out_off = load_current.0 * t_off.0;
                self.v_out = v_out_at_off + Voltage((q_off - q_out_off) / self.parameters.c_out.0);

                // Propagate the current-sense filter state across ON + OFF phases.
                self.i_cs = propagate_cs_filter(
                    self.i_cs,
                    self.i_inductor,
                    di_dt_on,
                    t_on,
                    i_max,
                    i_final,
                    t_off,
                    tau_cs,
                );

                self.i_inductor = i_final;

                // Buck: input draws q_on during ON; cap recovers from source during OFF.
                self.update_v_in_cap(v_in, q_on, t_on, t_off);

                (t_on, i_max)
            }

            Topology::Boost | Topology::BuckBoost => {
                // ON phase: C is decoupled (diode reverse-biased), so this is an RL circuit.
                // Exact solution is exponential; approximate as linear with effective V_in
                // computed at the midpoint of the ON-phase current swing.  Error is
                // O((R·ΔI/V_in)²) — negligible for typical R << L·f_sw.
                let v_on_eff = v_eff.0
                    - self.parameters.r_series.0 * (self.i_inductor.0 + trip_current.0) / 2.0;
                let di_dt_on = v_on_eff / self.parameters.l_inductor.0;
                let i_on_line = Line {
                    k: di_dt_on,
                    m: self.i_inductor.0,
                };

                let t_on_guess = Time(
                    (trip_current - self.i_inductor).0
                        / (di_dt_on - self.parameters.slope_amp_per_sec),
                );
                // When filters are active use the filtered intersection; otherwise two Lines
                // converge in exactly 1 Newton step so the analytic guess suffices.
                let t_on = if tau_cs.0 > 0.0 || tau_dac.0 > 0.0 {
                    Time(filtered_trip_time(
                        self.i_inductor,
                        di_dt_on,
                        self.i_cs,
                        tau_cs,
                        trip_current,
                        self.parameters.slope_amp_per_sec,
                        tau_dac,
                        t_on_guess,
                    ))
                } else {
                    Time(
                        i_on_line
                            .intersects_at(
                                Line {
                                    k: self.parameters.slope_amp_per_sec,
                                    m: trip_current.0,
                                },
                                t_on_guess.0,
                            )
                            .unwrap(),
                    )
                };

                // Comparator propagation delay: switch stays ON for t_prop extra after threshold.
                let t_on = t_on + self.parameters.t_prop_delay;

                let i_max;
                let t_on = if t_on.0 < 0.0 {
                    i_max = self.i_inductor;
                    Time(0.0)
                } else if t_on.0 > self.parameters.period.0 {
                    i_max = Current(i_on_line.f(self.parameters.period.0));
                    self.parameters.period
                } else {
                    i_max = Current(i_on_line.f(t_on.0));
                    t_on
                };
                let t_off = self.parameters.period - t_on;

                // EMI estimate: computed while i_inductor / i_in_cap are still start-of-cycle.
                self.v_in_ripple_est = self.compute_v_in_ripple(i_max, t_on);

                // During ON: cap is decoupled from the inductor but still drives the load,
                // so it droops by q_out_on / C before the OFF phase begins.
                // Sample load current once — see Buck branch for rationale.
                let load_current = i_out(self.v_out);
                let q_out_on = load_current.0 * t_on.0;
                let v_out_at_off = self.v_out - Voltage(q_out_on / self.parameters.c_out.0);

                // OFF phase: L and C coupled, starting from the drooped cap voltage.
                // Boost:     V_in drives L+C in series (energy transferred from L+source to C).
                // BuckBoost: no source, L discharges into cap.
                let v_off_source = match self.topology {
                    Topology::Boost => v_eff,
                    Topology::BuckBoost | Topology::Buck => Voltage(0.0),
                };
                let i_off_func = math::rlc(
                    v_off_source,
                    v_out_at_off,
                    i_max,
                    self.parameters.l_inductor,
                    self.parameters.c_out,
                    self.parameters.r_series,
                );
                let i_final = Current(i_off_func.f(t_off.0));

                let q_in = i_off_func.integral(0.0).f(t_off.0);
                let q_out_off = load_current.0 * t_off.0;
                self.v_out = v_out_at_off + Voltage((q_in - q_out_off) / self.parameters.c_out.0);

                // Propagate the current-sense filter state across ON + OFF phases.
                self.i_cs = propagate_cs_filter(
                    self.i_cs,
                    self.i_inductor,
                    di_dt_on,
                    t_on,
                    i_max,
                    i_final,
                    t_off,
                    tau_cs,
                );

                self.i_inductor = i_final;

                // BuckBoost: only ON phase draws from input; cap recovers during OFF.
                // Boost:     input is in-circuit during OFF too, so add q_in as extra draw;
                //            recharge time approximated as the full switching period.
                let q_input = i_on_line.integral(0.0).f(t_on.0)
                    + if matches!(self.topology, Topology::Boost) {
                        q_in
                    } else {
                        0.0
                    };
                let t_recharge = if matches!(self.topology, Topology::Boost) {
                    self.parameters.period
                } else {
                    t_off
                };
                self.update_v_in_cap(v_in, q_input, t_on, t_recharge);

                (t_on, i_max)
            }
        }
    }

    /// Estimate the peak-to-peak voltage ripple at the converter input terminals
    /// at the switching frequency.
    ///
    /// Must be called inside `tick()` **before** state is updated, so that
    /// `self.i_inductor` and `self.i_in_cap` still hold start-of-cycle values.
    ///
    /// Two components:
    /// - **Capacitive**: net charge drawn from C_in during the ON phase divided by C_in.
    /// - **ESR spike**: peak net current through C_in × r_esr_cin (the resistive
    ///   voltage that is NOT filtered by C_in and appears directly at the terminals).
    fn compute_v_in_ripple(&self, i_max: Current, t_on: Time) -> Voltage {
        if self.parameters.c_in.0 == 0.0 {
            return Voltage(0.0);
        }
        // Capacitive: net charge drawn from C_in (converter minus cable supply)
        let i_avg_on = (self.i_inductor.0 + i_max.0) / 2.0;
        let net_charge = (i_avg_on - self.i_in_cap.0).max(0.0) * t_on.0;
        let v_cap = Voltage(net_charge / self.parameters.c_in.0);
        // ESR: peak current through C_in at the end of the ON phase
        let i_peak_cap = (i_max.0 - self.i_in_cap.0).max(0.0);
        let v_esr = Voltage(self.parameters.r_esr_cin.0 * i_peak_cap);
        v_cap + v_esr
    }

    /// Droop `v_in_cap` by the net charge drawn from it this cycle, then evolve
    /// the input LC filter over `t_recharge`.
    ///
    /// During the ON phase the cable current `i_in_cap` partially compensates the
    /// droop; the net charge removed from C_in is `q_drawn − i_in_cap × t_on`.
    ///
    /// During the recovery phase (OFF or full period depending on topology):
    ///  - `l_in > 0`: full RLC dynamics via `math::rlc()` — tracks cable current.
    ///  - `l_in = 0, r_in = 0`: ideal source, cap instantly restored.
    ///  - `l_in = 0, r_in > 0`: first-order RC exponential recharge.
    fn update_v_in_cap(&mut self, v_source: Voltage, q_drawn: f64, t_on: Time, t_recharge: Time) {
        if self.parameters.c_in.0 == 0.0 {
            return;
        }
        // Cable current partially supplies C_in during the ON phase.
        let q_cable_on = self.i_in_cap.0 * t_on.0;
        self.v_in_cap = self.v_in_cap - Voltage((q_drawn - q_cable_on) / self.parameters.c_in.0);

        if self.parameters.l_in.0 > 0.0 {
            // Full LC input filter: evolve RLC from current (v_in_cap, i_in_cap).
            let resp = math::rlc(
                v_source,
                self.v_in_cap,
                self.i_in_cap,
                self.parameters.l_in,
                self.parameters.c_in,
                self.parameters.r_in,
            );
            self.v_in_cap = self.v_in_cap
                + Voltage(resp.integral(0.0).f(t_recharge.0) / self.parameters.c_in.0);
            self.i_in_cap = Current(resp.f(t_recharge.0));
        } else if self.parameters.r_in.0 == 0.0 {
            self.v_in_cap = v_source;
        } else {
            let tau = self.parameters.r_in * self.parameters.c_in.0;
            self.v_in_cap +=
                Voltage((1.0 - f64::exp(-t_recharge.0 / tau.0)) * (v_source.0 - self.v_in_cap.0));
        }
    }
}

/// Find the ON-phase trip time when current-sense and/or slope-compensation
/// filters are active (τ_cs > 0 or τ_dac > 0).
///
/// The comparator fires when the LP-filtered inductor current equals the
/// LP-filtered slope-compensation threshold:
///
///   i_cs(t)      = i₀ + m·(t − τ_cs) + err_cs·exp(−t/τ_cs)
///   threshold(t) = trip + slope_rate·(t − τ_dac + τ_dac·exp(−t/τ_dac))
///
/// where `err_cs = i_cs0 − i₀ + m·τ_cs` is the initial condition error.
///
/// Setting τ_cs = 0 recovers the ideal (unfiltered) current sense, and
/// τ_dac = 0 recovers the ideal (instantaneous) slope ramp — so the function
/// degenerates to the ordinary line-intersection formula in both cases.
///
/// Uses 15 iterations of Newton–Raphson; convergence is guaranteed for
/// practically realisable filter time constants (τ << T_sw).
fn filtered_trip_time(
    i0: Current,     // inductor current at start of ON phase (A)
    m: f64,          // inductor current slope di/dt (A/s)
    i_cs0: Current,  // current-sense filter state at start of cycle (A)
    tau_cs: Time,    // current-sense time constant (s); 0 = ideal
    trip: Current,   // trip-current reference (A)
    slope_rate: f64, // slope compensation (A/s, typically negative)
    tau_dac: Time,   // DAC output filter time constant (s); 0 = ideal
    guess: Time,     // initial guess for t_on (s)
) -> f64 {
    // If the filter state is already at or above the threshold at t=0 (e.g.
    // during soft-start overshoot where trip_current → 0 but i_valley > 0),
    // the comparator fires immediately.  Return 0 rather than letting N-R
    // step into negative time where exp(-t/τ) blows up.
    if i_cs0.0 >= trip.0 {
        return 0.0;
    }

    let err_cs = if tau_cs.0 > 0.0 {
        i_cs0.0 - i0.0 + m * tau_cs.0
    } else {
        0.0
    };
    let mut t = guess.0.max(0.0);
    for _ in 0..15 {
        // LP-filtered current sense output at time t
        let i_cs = if tau_cs.0 > 0.0 {
            i0.0 + m * (t - tau_cs.0) + err_cs * (-t / tau_cs.0).exp()
        } else {
            i0.0 + m * t
        };

        // LP-filtered slope-compensation threshold at time t
        // (ideal ramp `slope_rate·t` passed through a first-order LP filter)
        let threshold = trip.0
            + if tau_dac.0 > 0.0 {
                slope_rate * (t - tau_dac.0 + tau_dac.0 * (-t / tau_dac.0).exp())
            } else {
                slope_rate * t
            };

        let f = i_cs - threshold;

        // Derivative of F(t) = i_cs(t) − threshold(t)
        let di_cs_dt = if tau_cs.0 > 0.0 {
            m - err_cs / tau_cs.0 * (-t / tau_cs.0).exp()
        } else {
            m
        };
        let d_threshold_dt = slope_rate
            * if tau_dac.0 > 0.0 {
                1.0 - (-t / tau_dac.0).exp()
            } else {
                1.0
            };
        let f_prime = di_cs_dt - d_threshold_dt;

        if f_prime.abs() < 1e-30 {
            break;
        }
        let dt = f / f_prime;
        t -= dt;
        t = t.max(0.0); // trip can't fire before cycle start
        if dt.abs() < 1e-15 {
            break;
        }
    }
    t
}

/// Propagate the current-sense LP filter state across one full switching cycle
/// (ON phase followed by OFF phase) using a piecewise-linear approximation of
/// the inductor current trajectory.
///
/// Returns the new filter state `i_cs` to carry into the next cycle.
/// When `tau_cs == 0.0` the function is a no-op and returns `i_valley_next`
/// (ideal: the filter tracks the actual valley current instantaneously).
fn propagate_cs_filter(
    i_cs0: Current,         // filter state at start of cycle (A)
    i_valley: Current,      // inductor valley current at start of cycle (A)
    m_on: f64,              // inductor slope during ON phase (A/s, positive)
    t_on: Time,             // ON-phase duration (s)
    i_peak: Current,        // inductor peak current at end of ON phase (A)
    i_valley_next: Current, // inductor valley current at end of OFF phase (A)
    t_off: Time,            // OFF-phase duration (s)
    tau_cs: Time,           // current-sense time constant (s); 0 = ideal
) -> Current {
    if tau_cs.0 == 0.0 {
        return i_valley_next;
    }
    // ON phase: linear ramp i₀ + m_on·t with initial filter state i_cs0.
    let err_on = i_cs0 - i_valley + Current(m_on * tau_cs.0);
    let i_cs_at_ton =
        i_valley + Current(m_on * (t_on - tau_cs).0) + err_on * (-t_on / tau_cs).exp();

    // OFF phase: linear downslope from i_peak to i_valley_next.
    let m_off = if t_off.0 > 1e-15 {
        (i_valley_next - i_peak) / t_off.0
    } else {
        Current(0.0)
    };
    let err_off = i_cs_at_ton - i_peak + m_off * tau_cs.0;
    i_peak + m_off * (t_off - tau_cs).0 + err_off * (-t_off / tau_cs).exp()
}

macro_rules! impl_math {
    ($t:ident) => {
        #[derive(Copy, Clone)]
        pub struct $t(pub T);

        impl Neg for $t {
            type Output = Self;

            fn neg(self) -> Self::Output {
                Self(-self.0)
            }
        }

        impl Add for $t {
            type Output = Self;

            fn add(self, rhs: Self) -> Self::Output {
                Self(self.0 + rhs.0)
            }
        }

        impl Sub for $t {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                Self(self.0 - rhs.0)
            }
        }

        impl Mul<T> for $t {
            type Output = Self;

            fn mul(self, rhs: T) -> Self::Output {
                Self(self.0 * rhs)
            }
        }

        impl Div for $t {
            type Output = T;

            fn div(self, rhs: Self) -> Self::Output {
                self.0 / rhs.0
            }
        }

        impl Div<T> for $t {
            type Output = Self;

            fn div(self, rhs: T) -> Self::Output {
                Self(self.0 / rhs)
            }
        }

        impl AddAssign for $t {
            fn add_assign(&mut self, rhs: Self) {
                self.0 += rhs.0
            }
        }
    };
}

impl_math!(Inductance);
impl_math!(Capacitance);
impl_math!(Current);
impl_math!(Resistance);
impl_math!(Voltage);
impl_math!(Time);
