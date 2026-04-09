pub mod cap_bank;
pub mod math;
pub mod pfc_boost;

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

/// Controls whether the inductor current is allowed to go negative during the OFF phase.
///
/// In a real converter this depends on the rectifier configuration:
///
/// - With **MOSFETs driven by the MCU** (synchronous rectification) there is nothing
///   stopping reverse current: when the inductor current reaches zero it keeps
///   ramping negative, flowing back through the low-side switch into the source.
///
/// - With a **diode** (or a synchronous converter whose firmware turns off the low-side
///   switch the moment current hits zero) reverse current is blocked.  The circuit
///   enters discontinuous conduction mode (DCM): the inductor current coasts at zero
///   for the remainder of the switching period (the "dead time"), and only the
///   output capacitor — draining at the load rate — determines the output voltage
///   during that interval.
#[derive(Copy, Clone, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
pub enum CurrentConduction {
    /// Both switches are driven by the MCU throughout the full switching period.
    /// The inductor current **can go negative** — energy flows back into the source.
    /// Use this for 4-switch non-inverting buck-boost and other fully-synchronous
    /// topologies where the firmware does not implement a zero-current interlock.
    Synchronous,

    /// A diode (or dead-time-controlled synchronous switch) blocks reverse current.
    /// When the inductor current reaches zero during the OFF phase the switch opens.
    /// The remaining dead time passes with **zero inductor current**; the output cap
    /// drains at the load rate only.  This gives physically accurate DCM behaviour.
    Diode,
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

    /// Whether reverse inductor current is permitted.  See [`CurrentConduction`].
    pub current_conduction: CurrentConduction,

    /// Comparator blanking window (s).  Trip events before this time are ignored,
    /// setting a minimum on-time.  `0.0` = no blanking (ideal).
    pub t_blanking: Time,

    /// ADC sample point within the switching cycle (s from period start).
    /// `0.0` = sample at start of cycle (current behaviour).
    pub t_adc_sample_point: Time,

    /// Maximum duty cycle fraction (0.0–1.0).  `1.0` = no limit.
    pub max_duty: f64,
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

    /// Output voltage at the ADC sample point.  Updated each `tick()`.
    /// When `t_adc_sample_point` is 0, equals `v_out` at start of cycle.
    pub v_out_at_adc: Voltage,

    /// Minimum output voltage (including ESR) within the last switching cycle.
    /// Computed during `tick()` from the charge-balance trajectory + R_esr.
    pub v_out_min_cycle: Voltage,
    /// Maximum output voltage (including ESR) within the last switching cycle.
    pub v_out_max_cycle: Voltage,
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

    /// Output voltage as sensed by the ADC when using a cap bank.
    /// The cap bank's v_out already includes the ESR contributions from all
    /// cap types, so we just return it directly (no separate ESR correction).
    pub fn v_sensed_cap_bank(&self) -> Voltage {
        self.v_out
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
            v_out_at_adc: Voltage(0.0),
            v_out_min_cycle: Voltage(0.0),
            v_out_max_cycle: Voltage(0.0),
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
        let t_dac_sample = self.parameters.t_dac_sample;

        match self.topology {
            Topology::Buck => {
                let v_out_start = self.v_out;
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
                let t_on = if t_dac_sample.0 > 0.0 {
                    Time(staircase_trip_time(
                        self.i_inductor,
                        di_dt_on,
                        self.i_cs,
                        tau_cs,
                        trip_current,
                        self.parameters.slope_amp_per_sec,
                        tau_dac,
                        t_dac_sample,
                        self.parameters.period,
                    ))
                } else if tau_cs.0 > 0.0 || tau_dac.0 > 0.0 {
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

                // Blanking: comparator is blind until t_blanking
                let t_on = Time(t_on.0.max(self.parameters.t_blanking.0));

                // Comparator propagation delay: switch stays ON for t_prop extra after threshold.
                let t_on = t_on + self.parameters.t_prop_delay;

                // Max duty: HRTIM CR1 limits maximum on-time
                let t_on = Time(t_on.0.min(self.parameters.period.0 * self.parameters.max_duty));

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

                // DCM: if a diode (or dead-time control) prevents reverse current,
                // the inductor stops conducting at t_zero.  Only the charge delivered
                // up to that point enters the cap; the rest of the period is dead time
                // during which the cap drains at the load rate only.
                // q_out_off spans the full t_off regardless — the load draws current
                // throughout whether or not the inductor is conducting.
                let (i_final, q_off) = match self.parameters.current_conduction {
                    CurrentConduction::Synchronous => (i_final, q_off),
                    CurrentConduction::Diode if i_final.0 < 0.0 && i_max.0 >= 0.0 => {
                        let t_zero = bisect_zero(|t| i_off_func.f(t), 0.0, t_off.0);
                        (Current(0.0), i_off_func.integral(0.0).f(t_zero))
                    }
                    CurrentConduction::Diode => (i_final, q_off),
                };

                let q_out_off = load_current.0 * t_off.0;
                self.v_out = v_out_at_off + Voltage((q_off - q_out_off) / self.parameters.c_out.0);

                // Compute v_out at the ADC sample point within this cycle.
                let t_sample = self.parameters.t_adc_sample_point.0;
                if t_sample <= 0.0 {
                    self.v_out_at_adc = v_out_start;
                } else if t_sample <= t_on.0 {
                    let q_sample = i_on_func.integral(0.0).f(t_sample);
                    let q_load_sample = load_current.0 * t_sample;
                    self.v_out_at_adc = v_out_start
                        + Voltage((q_sample - q_load_sample) / self.parameters.c_out.0);
                } else if t_sample <= self.parameters.period.0 {
                    let t_in_off = t_sample - t_on.0;
                    let q_off_sample = i_off_func.integral(0.0).f(t_in_off);
                    let q_load_off_sample = load_current.0 * t_in_off;
                    self.v_out_at_adc = v_out_at_off
                        + Voltage((q_off_sample - q_load_off_sample) / self.parameters.c_out.0);
                } else {
                    self.v_out_at_adc = self.v_out;
                }

                // ── Compute v_out min/max including ESR contribution ─────
                // The actual output voltage at any instant is:
                //   v_actual = v_cap + R_esr × (i_L - i_load)
                // We evaluate at three key points: start of cycle (valley),
                // end of ON phase (peak current), end of cycle (next valley).
                let r_esr = self.parameters.r_esr.0;
                let i_load_val = load_current.0;
                let v_start = v_out_start.0 + r_esr * (self.i_inductor.0 - i_load_val);
                let v_peak  = v_out_at_off.0 + r_esr * (i_max.0 - i_load_val);
                let v_end   = self.v_out.0   + r_esr * (i_final.0 - i_load_val);
                self.v_out_min_cycle = Voltage(v_start.min(v_peak).min(v_end));
                self.v_out_max_cycle = Voltage(v_start.max(v_peak).max(v_end));

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
                // When staircase is active, iterate through discrete steps.
                // When LP filters are active, use the filtered intersection.
                // Otherwise two Lines converge in exactly 1 Newton step.
                let t_on = if t_dac_sample.0 > 0.0 {
                    Time(staircase_trip_time(
                        self.i_inductor,
                        di_dt_on,
                        self.i_cs,
                        tau_cs,
                        trip_current,
                        self.parameters.slope_amp_per_sec,
                        tau_dac,
                        t_dac_sample,
                        self.parameters.period,
                    ))
                } else if tau_cs.0 > 0.0 || tau_dac.0 > 0.0 {
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

                // Blanking: comparator is blind until t_blanking
                let t_on = Time(t_on.0.max(self.parameters.t_blanking.0));

                // Comparator propagation delay: switch stays ON for t_prop extra after threshold.
                let t_on = t_on + self.parameters.t_prop_delay;

                // Max duty: HRTIM CR1 limits maximum on-time
                let t_on = Time(t_on.0.min(self.parameters.period.0 * self.parameters.max_duty));

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

                // DCM: if a diode (or dead-time control) prevents reverse current,
                // the inductor stops conducting at t_zero.  Only the charge delivered
                // up to that point enters the cap; the rest of the period is dead time
                // during which the cap drains at the load rate only.
                // q_out_off spans the full t_off regardless — the load draws current
                // throughout whether or not the inductor is conducting.
                let (i_final, q_in) = {
                    let q_in_full = i_off_func.integral(0.0).f(t_off.0);
                    match self.parameters.current_conduction {
                        CurrentConduction::Synchronous => (i_final, q_in_full),
                        CurrentConduction::Diode if i_final.0 < 0.0 && i_max.0 >= 0.0 => {
                            let t_zero = bisect_zero(|t| i_off_func.f(t), 0.0, t_off.0);
                            (Current(0.0), i_off_func.integral(0.0).f(t_zero))
                        }
                        CurrentConduction::Diode => (i_final, q_in_full),
                    }
                };
                let q_out_off = load_current.0 * t_off.0;
                self.v_out = v_out_at_off + Voltage((q_in - q_out_off) / self.parameters.c_out.0);

                // Simplified: use end-of-cycle v_out (full ADC sample model is Buck-only)
                self.v_out_at_adc = self.v_out;

                // Boost/BuckBoost: v_out min/max with ESR at key points.
                // During ON the cap is decoupled from the inductor — only the
                // load drains it, so v_out drops from its start-of-cycle value.
                // During OFF the inductor dumps current into the cap.
                let r_esr = self.parameters.r_esr.0;
                let i_load_val = load_current.0;
                let v_at_on_start = v_out_at_off.0 + Voltage(q_out_on / self.parameters.c_out.0).0
                    + r_esr * (0.0 - i_load_val);
                let v_at_off_start = v_out_at_off.0 + r_esr * (i_max.0 - i_load_val);
                let v_end = self.v_out.0 + r_esr * (i_final.0 - i_load_val);
                self.v_out_min_cycle = Voltage(v_at_on_start.min(v_at_off_start).min(v_end));
                self.v_out_max_cycle = Voltage(v_at_on_start.max(v_at_off_start).max(v_end));

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

    /// Tick using a CapBank for output dynamics instead of the analytical RLC solver.
    ///
    /// This method handles the same control logic as `tick()` (slope compensation,
    /// trip detection, blanking, propagation delay, current-sense filtering, input
    /// cap) but delegates the inductor-current / output-voltage dynamics to the
    /// CapBank's RK4 state-space solver.
    ///
    /// Only supports Buck topology.
    ///
    /// # Arguments
    /// - `v_in` — external supply voltage
    /// - `trip_current` — peak current reference from the controller
    /// - `i_out` — load current closure (called once at start of cycle)
    /// - `cap_bank` — the output capacitor bank solver (state is mutated)
    ///
    /// # Returns
    /// `(t_on, i_max)` — on-time and peak inductor current, same as `tick()`.
    pub fn tick_cap_bank(
        &mut self,
        v_in: Voltage,
        trip_current: Current,
        mut i_out: impl FnMut(Voltage) -> Current,
        cap_bank: &mut cap_bank::CapBank,
    ) -> (Time, Current) {
        assert!(
            matches!(self.topology, Topology::Buck),
            "tick_cap_bank only supports Buck topology"
        );

        // ── Input voltage handling (same as tick()) ──────────────────────
        if self.parameters.c_in.0 > 0.0 && self.v_in_cap.0 == 0.0 {
            self.v_in_cap = v_in;
        }
        let v_eff = if self.parameters.c_in.0 > 0.0 {
            self.v_in_cap
        } else {
            v_in
        };

        let tau_cs = self.parameters.tau_current_sense;
        let tau_dac = self.parameters.tau_dac;
        let t_dac_sample = self.parameters.t_dac_sample;
        let r_series = self.parameters.r_series.0;
        let l = self.parameters.l_inductor.0;

        // Snapshot pre-tick state for current-sense filter and caller
        self.i_inductor_prev = self.i_inductor;
        self.i_cs_prev = self.i_cs;

        // ── Compute initial di/dt for trip time estimation ───────────────
        // The inductor ramp rate at the start of the ON phase. This uses the
        // cap bank's v_out (which equals self.v_out since they track together).
        let v_out_start = self.v_out;
        let di_dt_on = (v_eff.0 - v_out_start.0) / l;
        self.last_di_dt_on = di_dt_on;

        // ── Trip time calculation ────────────────────────────────────────
        // Uses the same analytical methods as tick(): these depend on the
        // inductor current slope (di_dt_on) and control parameters, NOT on
        // the detailed cap dynamics — so the linear ramp approximation is
        // accurate enough for trip time detection.
        let t_on_guess = Time(
            (trip_current - self.i_inductor).0
                / (di_dt_on - self.parameters.slope_amp_per_sec),
        );

        let t_on = if t_dac_sample.0 > 0.0 {
            Time(staircase_trip_time(
                self.i_inductor,
                di_dt_on,
                self.i_cs,
                tau_cs,
                trip_current,
                self.parameters.slope_amp_per_sec,
                tau_dac,
                t_dac_sample,
                self.parameters.period,
            ))
        } else if tau_cs.0 > 0.0 || tau_dac.0 > 0.0 {
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
            // Ideal trip: linear ramp intersects slope line.
            // t = (I_trip - I_valley) / (di/dt - slope_rate)
            let denom = di_dt_on - self.parameters.slope_amp_per_sec;
            if denom.abs() < 1e-30 {
                self.parameters.period
            } else {
                Time(((trip_current.0 - self.i_inductor.0) / denom).max(0.0))
            }
        };

        // Blanking and propagation delay
        let t_on = Time(t_on.0.max(self.parameters.t_blanking.0));
        let t_on = t_on + self.parameters.t_prop_delay;
        let t_on = Time(t_on.0.clamp(0.0, self.parameters.period.0 * self.parameters.max_duty));
        let t_off = self.parameters.period - t_on;

        // EMI estimate (before state update, same as tick())
        let i_max_est = Current(self.i_inductor.0 + di_dt_on * t_on.0);
        self.v_in_ripple_est = self.compute_v_in_ripple(i_max_est, t_on);

        // ── Sample load current once ─────────────────────────────────────
        let load_current = i_out(self.v_out);

        // ── Synchronize cap bank state with this phase's inductor current ──
        // In multi-phase operation each converter tracks its own i_inductor,
        // but they all share one cap bank.  Before integrating we must set
        // the cap bank's i_L to THIS phase's valley current so the RK4
        // solver integrates the correct inductor.
        cap_bank.set_i_l(self.i_inductor.0);

        // ── ON phase: integrate cap bank with RK4 ────────────────────────
        cap_bank.reset_v_out_minmax();
        cap_bank.integrate_phase(t_on.0, v_eff.0, load_current.0, r_series, l, None);
        let i_max = Current(cap_bank.i_l());

        // ── OFF phase: integrate cap bank with v_applied = 0 ─────────────
        cap_bank.integrate_phase(t_off.0, 0.0, load_current.0, r_series, l, None);
        let i_final = Current(cap_bank.i_l());

        // ── DCM: clamp to zero if diode mode and current went negative ───
        let i_final = match self.parameters.current_conduction {
            CurrentConduction::Synchronous => i_final,
            CurrentConduction::Diode if i_final.0 < 0.0 && i_max.0 >= 0.0 => {
                cap_bank.set_i_l(0.0);
                Current(0.0)
            }
            CurrentConduction::Diode => i_final,
        };

        // ── Update converter state from cap bank ─────────────────────────
        self.v_out = Voltage(cap_bank.v_out());
        self.i_inductor = i_final;
        self.v_out_min_cycle = Voltage(cap_bank.v_out_min());
        self.v_out_max_cycle = Voltage(cap_bank.v_out_max());

        // ── ADC sample point ─────────────────────────────────────────────
        // For the cap bank path, approximate v_out_at_adc using linear interpolation.
        // The exact trajectory would need sub-step cap bank queries, but for the
        // ADC sample (used for control), the start-of-cycle voltage is typical.
        let t_sample = self.parameters.t_adc_sample_point.0;
        if t_sample <= 0.0 {
            self.v_out_at_adc = v_out_start;
        } else {
            // Linearly interpolate between start and end of cycle
            let frac = (t_sample / self.parameters.period.0).clamp(0.0, 1.0);
            self.v_out_at_adc = Voltage(
                v_out_start.0 * (1.0 - frac) + self.v_out.0 * frac,
            );
        }

        // ── Current-sense filter propagation ─────────────────────────────
        self.i_cs = propagate_cs_filter(
            self.i_cs,
            self.i_inductor_prev,
            di_dt_on,
            t_on,
            i_max,
            i_final,
            t_off,
            tau_cs,
        );

        // ── Input capacitor update ───────────────────────────────────────
        // Approximate charge drawn during ON phase from the inductor current.
        let q_on = (self.i_inductor_prev.0 + i_max.0) / 2.0 * t_on.0;
        self.update_v_in_cap(v_in, q_on, t_on, t_off);

        (t_on, i_max)
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

/// Find where `f` crosses zero from positive to negative in `[a, b]`.
///
/// Assumes `f(a) ≥ 0` and `f(b) ≤ 0`.  50 iterations give sub-femtosecond
/// resolution on a nanosecond-scale switching period.
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

/// Find the ON-phase trip time when slope compensation is a ZOH staircase
/// (`t_dac_sample > 0`), optionally combined with current-sense and/or DAC
/// output LP filters.
///
/// The slope-compensation threshold advances in discrete steps:
///
///   threshold[n] = trip + slope_rate × n × t_dac_sample
///
/// Within each step the threshold is held constant (ZOH).  When `tau_dac > 0`
/// the staircase output is LP-filtered: each step triggers an exponential
/// settling from the previous filtered value toward the new staircase level.
///
/// The inductor current (or its LP-filtered version when `tau_cs > 0`) is
/// tested against the threshold within each step interval.  The first crossing
/// determines the trip time.
fn staircase_trip_time(
    i0: Current,        // inductor current at start of ON phase (A)
    m: f64,             // inductor current slope di/dt (A/s)
    i_cs0: Current,     // current-sense filter state at start of cycle (A)
    tau_cs: Time,       // current-sense time constant (s); 0 = ideal
    trip: Current,      // trip-current reference (A)
    slope_rate: f64,    // slope compensation (A/s, typically negative)
    tau_dac: Time,      // DAC output filter time constant (s); 0 = ideal
    t_dac_sample: Time, // DAC sample period (s)
    period: Time,       // switching period (s)
) -> f64 {
    // Early exit: sensed current already at/above trip at t = 0.
    let i_sensed_0 = if tau_cs.0 > 0.0 { i_cs0.0 } else { i0.0 };
    if i_sensed_0 >= trip.0 {
        return 0.0;
    }

    let t_s = t_dac_sample.0;
    let max_steps = (period.0 / t_s).ceil() as usize;
    let has_filters = tau_cs.0 > 0.0 || tau_dac.0 > 0.0;

    // Filter states carried across step boundaries.
    let mut thresh_filt = trip.0; // LP-filtered threshold (starts at trip)
    let mut i_cs_state = i_cs0.0; // CS filter state

    for n in 0..max_steps {
        let t_step_start = n as f64 * t_s;
        let t_step_end = ((n + 1) as f64 * t_s).min(period.0);
        let dt_step = t_step_end - t_step_start;

        // Ideal staircase level for this step.
        let thresh_ideal = trip.0 + slope_rate * n as f64 * t_s;

        // Inductor current (ideal) at start of this step.
        let i_at_step = i0.0 + m * t_step_start;

        // Evaluate i_sensed(t_local) − threshold(t_local) within this step.
        let eval_at = |t_local: f64| -> f64 {
            let i_sensed = if tau_cs.0 > 0.0 {
                let err = i_cs_state - i_at_step + m * tau_cs.0;
                i_at_step + m * (t_local - tau_cs.0) + err * (-t_local / tau_cs.0).exp()
            } else {
                i_at_step + m * t_local
            };
            let thresh = if tau_dac.0 > 0.0 {
                thresh_ideal + (thresh_filt - thresh_ideal) * (-t_local / tau_dac.0).exp()
            } else {
                thresh_ideal
            };
            i_sensed - thresh
        };

        let f_start = eval_at(0.0);
        if f_start >= 0.0 {
            // Already tripped at step start (threshold just dropped below current).
            return t_step_start;
        }

        let f_end = eval_at(dt_step);
        if f_end >= 0.0 {
            // Crossing within this step.
            if !has_filters {
                // Analytical: linear current crosses constant threshold.
                let t_local = (thresh_ideal - i_at_step) / m;
                return t_step_start + t_local.clamp(0.0, dt_step);
            }
            // Bisection for filtered case.
            let mut a = 0.0_f64;
            let mut b = dt_step;
            for _ in 0..50 {
                let mid = (a + b) * 0.5;
                if eval_at(mid) < 0.0 {
                    a = mid;
                } else {
                    b = mid;
                }
            }
            return t_step_start + (a + b) * 0.5;
        }

        // No crossing in this step — advance filter states to end of step.
        if tau_dac.0 > 0.0 {
            thresh_filt =
                thresh_ideal + (thresh_filt - thresh_ideal) * (-dt_step / tau_dac.0).exp();
        } else {
            thresh_filt = thresh_ideal;
        }
        if tau_cs.0 > 0.0 {
            let err = i_cs_state - i_at_step + m * tau_cs.0;
            i_cs_state =
                i_at_step + m * (dt_step - tau_cs.0) + err * (-dt_step / tau_cs.0).exp();
        } else {
            i_cs_state = i_at_step + m * dt_step;
        }
    }

    // No trip within the period — return period (100% duty).
    period.0
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

#[cfg(test)]
mod tests {
    use super::*;

    // Test that all math types compile and basic operations work
    #[test]
    fn test_voltage_operations() {
        let v1 = Voltage(12.0);
        let v2 = Voltage(5.0);
        let sum = v1 + v2;
        assert_eq!(sum.0, 17.0);
        let diff = v1 - v2;
        assert_eq!(diff.0, 7.0);
        let scaled = v1 * 2.0;
        assert_eq!(scaled.0, 24.0);
    }

    #[test]
    fn test_current_operations() {
        let c1 = Current(3.0);
        let c2 = Current(1.5);
        let sum = c1 + c2;
        assert_eq!(sum.0, 4.5);
        let diff = c1 - c2;
        assert_eq!(diff.0, 1.5);
    }

    #[test]
    fn test_resistance_operations() {
        let r1 = Resistance(10.0);
        let r2 = Resistance(5.0);
        let sum = r1 + r2;
        assert_eq!(sum.0, 15.0);
        let scaled = r1 * 2.0;
        assert_eq!(scaled.0, 20.0);
    }

    #[test]
    fn test_inductance_operations() {
        let l = Inductance(4e-6);
        let scaled = l * 2.0;
        assert_eq!(scaled.0, 8e-6);
    }

    #[test]
    fn test_capacitance_operations() {
        let c = Capacitance(100e-6);
        let scaled = c * 0.5;
        assert_eq!(scaled.0, 50e-6);
    }

    #[test]
    fn test_time_operations() {
        let t1 = Time(1e-6);
        let t2 = Time(0.5e-6);
        let sum = t1 + t2;
        assert_eq!(sum.0, 1.5e-6);
        let diff = t1 - t2;
        assert_eq!(diff.0, 0.5e-6);
    }

    #[test]
    fn test_buck_converter_basic() {
        let params = Parameters {
            period: Time(1e-6),
            slope_amp_per_sec: 0.0,
            r_series: Resistance(0.0),
            r_esr: Resistance(0.0),
            c_out: Capacitance(10e-6),
            l_inductor: Inductance(4e-6),
            c_in: Capacitance(0.0),
            r_esr_cin: Resistance(0.0),
            r_in: Resistance(0.0),
            l_in: Inductance(0.0),
            tau_current_sense: Time(0.0),
            tau_dac: Time(0.0),
            t_prop_delay: Time(0.0),
            t_dac_sample: Time(0.0),
            current_conduction: CurrentConduction::Synchronous,
            t_blanking: Time(0.0),
            t_adc_sample_point: Time(0.0),
            max_duty: 1.0,
        };
        let mut sim = CurrentModeConverter::new(params, Topology::Buck);

        // Initial state should be zeros
        assert_eq!(sim.v_out.0, 0.0);
        assert_eq!(sim.i_inductor.0, 0.0);
    }

    #[test]
    fn test_buck_converter_steady() {
        let params = Parameters {
            period: Time(1e-6),
            slope_amp_per_sec: 0.0,
            r_series: Resistance(0.1),
            r_esr: Resistance(0.0),
            c_out: Capacitance(100e-6),
            l_inductor: Inductance(4e-6),
            c_in: Capacitance(0.0),
            r_esr_cin: Resistance(0.0),
            r_in: Resistance(0.0),
            l_in: Inductance(0.0),
            tau_current_sense: Time(0.0),
            tau_dac: Time(0.0),
            t_prop_delay: Time(0.0),
            t_dac_sample: Time(0.0),
            current_conduction: CurrentConduction::Synchronous,
            t_blanking: Time(0.0),
            t_adc_sample_point: Time(0.0),
            max_duty: 1.0,
        };
        let mut sim = CurrentModeConverter::new(params, Topology::Buck);

        let v_in = Voltage(24.0);
        let trip_current = Current(5.0);

        // Run for many cycles to reach steady state
        for _ in 0..1000 {
            sim.tick(v_in, trip_current, |_| Current(2.4));
        }

        // Should have non-zero output voltage
        assert!(sim.v_out.0 > 0.0);
        assert!(sim.i_inductor.0 > 0.0);
    }

    #[test]
    fn test_parameters_bw_to_tau() {
        let tau = Parameters::bw_to_tau(1000.0);
        let expected = Time(1.0 / (2.0 * std::f64::consts::PI * 1000.0));
        assert!((tau.0 - expected.0).abs() < 1e-12);

        // Test zero bandwidth
        let tau = Parameters::bw_to_tau(0.0);
        assert_eq!(tau.0, 0.0);
    }

    #[test]
    fn test_line_intersection() {
        let line1 = math::Line { k: 1.0, m: 0.0 };
        let line2 = math::Line { k: -1.0, m: 10.0 };

        let x = line1.intersects_at(line2, 5.0).unwrap();
        assert!((x - 5.0).abs() < 1e-10);
    }

    // NOTE: test_damped_sine and test_polynomial are disabled because
    // DampedSineF and Polynomial fields were made private in a prior refactor.

    // #[test]
    // fn test_damped_sine() { ... }

    // #[test]
    // fn test_polynomial() { ... }

    #[test]
    fn test_math_line() {
        let line = math::Line { k: 2.0, m: 3.0 };
        assert!((line.f(0.0) - 3.0).abs() < 1e-10);
        assert!((line.f(1.0) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_staircase_no_filters() {
        // Linear current ramp crossing a staircase threshold with slope comp.
        // i(t) = 10 A/µs × t,  threshold drops at −5 A/µs in 200 ns steps.
        //
        // Step 0: [0, 200ns) → thresh = 5 A,  i(200ns) = 2 A  → no trip
        // Step 1: [200ns, 400ns) → thresh = 4 A,  i(400ns) = 4 A → trip at boundary
        let t = staircase_trip_time(
            Current(0.0),
            10e6,
            Current(0.0),
            Time(0.0),
            Current(5.0),
            -5e6,
            Time(0.0),
            Time(200e-9),
            Time(2e-6),
        );
        assert!(
            (t - 400e-9).abs() < 1e-12,
            "expected 400 ns, got {:.3} ns",
            t * 1e9
        );
    }

    #[test]
    fn test_staircase_converges_to_continuous() {
        // With very small step size the staircase should approach the continuous
        // line-intersection result:  t = (trip − i0) / (m − slope_rate).
        let i0 = Current(0.0);
        let m = 10e6;
        let trip = Current(5.0);
        let slope_rate = -5e6;
        let t_continuous = (trip.0 - i0.0) / (m - slope_rate); // 333.33 ns

        let t_staircase = staircase_trip_time(
            i0,
            m,
            i0,
            Time(0.0),
            trip,
            slope_rate,
            Time(0.0),
            Time(1e-9), // 1 ns steps → ~333 steps
            Time(2e-6),
        );
        assert!(
            (t_staircase - t_continuous).abs() < 1.5e-9,
            "expected ≈{:.1} ns, got {:.1} ns",
            t_continuous * 1e9,
            t_staircase * 1e9
        );
    }

    #[test]
    fn test_staircase_no_slope_comp() {
        // Without slope compensation the staircase has no effect: threshold is
        // constant, so the trip time equals the continuous case.
        let t = staircase_trip_time(
            Current(0.0),
            10e6,
            Current(0.0),
            Time(0.0),
            Current(5.0),
            0.0, // no slope comp
            Time(0.0),
            Time(200e-9),
            Time(2e-6),
        );
        let t_expected = 5.0 / 10e6; // 500 ns
        assert!(
            (t - t_expected).abs() < 1e-12,
            "expected {:.1} ns, got {:.1} ns",
            t_expected * 1e9,
            t * 1e9
        );
    }

    #[test]
    fn test_staircase_immediate_trip() {
        // Sensed current already above trip → immediate trip.
        let t = staircase_trip_time(
            Current(6.0),
            10e6,
            Current(6.0),
            Time(0.0),
            Current(5.0),
            -5e6,
            Time(0.0),
            Time(200e-9),
            Time(2e-6),
        );
        assert_eq!(t, 0.0);
    }

    #[test]
    fn test_staircase_with_cs_filter() {
        // Use parameters where the unfiltered trip lands mid-step (not on a
        // boundary) so the CS filter lag visibly pushes the crossing later.
        // m = 12 A/µs, slope = −5 A/µs, trip = 5 A, step = 200 ns.
        // Unfiltered trip at ~333 ns (mid step-1).  With tau_cs = 50 ns the
        // sensed current lags, so the trip moves later.
        let tau_cs = Time(50e-9);

        let t_no_filter = staircase_trip_time(
            Current(0.0),
            12e6,
            Current(0.0),
            Time(0.0),
            Current(5.0),
            -5e6,
            Time(0.0),
            Time(200e-9),
            Time(2e-6),
        );

        let t_filtered = staircase_trip_time(
            Current(0.0),
            12e6,
            Current(0.0),
            tau_cs,
            Current(5.0),
            -5e6,
            Time(0.0),
            Time(200e-9),
            Time(2e-6),
        );

        assert!(
            t_filtered > t_no_filter,
            "CS filter should delay trip: {:.1} ns > {:.1} ns",
            t_filtered * 1e9,
            t_no_filter * 1e9
        );
    }

    #[test]
    fn test_staircase_buck_integration() {
        // End-to-end: a Buck converter with staircase slope comp should reach
        // steady state and produce a sensible output voltage.
        let params = Parameters {
            period: Time(2e-6), // 500 kHz
            slope_amp_per_sec: -5e6,
            r_series: Resistance(0.01),
            r_esr: Resistance(0.0),
            c_out: Capacitance(47e-6),
            l_inductor: Inductance(4e-6),
            c_in: Capacitance(0.0),
            r_esr_cin: Resistance(0.0),
            r_in: Resistance(0.0),
            l_in: Inductance(0.0),
            tau_current_sense: Time(0.0),
            tau_dac: Time(0.0),
            t_prop_delay: Time(0.0),
            t_dac_sample: Time(1.0 / 15e6), // ~67 ns steps
            current_conduction: CurrentConduction::Synchronous,
            t_blanking: Time(0.0),
            t_adc_sample_point: Time(0.0),
            max_duty: 1.0,
        };
        let mut sim = CurrentModeConverter::new(params, Topology::Buck);

        for _ in 0..5000 {
            sim.tick(Voltage(24.0), Current(5.0), |_| Current(2.0));
        }

        // Should settle near some output voltage (exact value depends on
        // staircase quantisation, but must be positive and reasonable).
        assert!(sim.v_out.0 > 3.0, "v_out = {:.2} V", sim.v_out.0);
        assert!(sim.v_out.0 < 24.0, "v_out = {:.2} V", sim.v_out.0);
    }
}
