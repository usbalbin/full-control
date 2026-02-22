pub mod math;

use rerun::RecordingStream;
use std::ops::{Add, AddAssign, Neg, Sub};

use crate::math::Line;
use math::Func;

#[derive(Clone, Copy, Debug)]
pub enum Topology {
    Buck,
    Boost,
    BuckBoost,
}

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
    rec.log("i_inductor", &rerun::Scalars::new([i_max.0 as f64])).unwrap();

    *time += sim.period;
    rec.set_timestamp_secs_since_epoch("time", (time.0 - 10e-9) as f64);
    rec.log("sw", &rerun::Scalars::new([0.0])).unwrap();

    rec.set_timestamp_secs_since_epoch("time", time.0 as f64);
    rec.log("sw", &rerun::Scalars::new([1.0])).unwrap();
    rec.log("i_inductor", &rerun::Scalars::new([sim.i_inductor.0 as f64]))
        .unwrap();

    rec.log("v_in", &rerun::Scalars::new([v_in.0 as f64])).unwrap();
    rec.log("v_in", &rerun::Scalars::new([v_in.0 as f64])).unwrap();
    rec.log("v_out", &rerun::Scalars::new([sim.v_out.0 as f64]))
        .unwrap();
    rec.log("i_out", &rerun::Scalars::new([i_out.0 as f64])).unwrap();
    rec.log("d", &rerun::Scalars::new([(t_on.0 / sim.period.0) as f64]))
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
    pub fn new(
        period: Time,
        c_out: Capacitance,
        l_inductor: Inductance,
        slope_amp_per_sec: T,
        amp_per_lsb: T,
        i_at_0lsb: Current,
    ) -> Self {
        Self {
            buck: CurrentModeConverter::new(period, c_out, l_inductor, slope_amp_per_sec, Topology::Buck, Resistance(0.0), 0.0, Capacitance(0.0), 0.0, 0.0, Inductance(0.0)),
            amp_per_lsb,
            i_at_0lsb,
        }
    }

    pub fn tick(
        &mut self,
        v_in: Voltage,
        trip_current: u16,
        mut i_out: impl FnMut(Voltage) -> Current,
    ) -> (Time, Current) {
        // 4095 -> 20
        // 0 -> -20
        let trip_current = Current(T::from(trip_current) * self.amp_per_lsb) + self.i_at_0lsb;
        self.buck.tick(v_in, trip_current, i_out)
    }
}

#[derive(Clone, Copy)]
pub struct CurrentModeConverter {
    period: Time,

    c_out: Capacitance,
    pub v_out: Voltage,

    pub i_inductor: Current,
    l_inductor: Inductance,

    // This should normally be negative
    pub slope_amp_per_sec: T,

    pub topology: Topology,

    /// Lumped series resistance (inductor DCR + conducting switch R_dson).
    /// Models conduction losses in both ON and OFF phases.
    r_series: Resistance,

    /// Equivalent series resistance of the output capacitor.
    /// Used by `v_sensed()` to reproduce the ESR voltage seen by the ADC.
    r_esr: f64,

    /// Input capacitance.  Zero disables V_in ripple modelling (ideal stiff source).
    c_in: Capacitance,
    /// Equivalent series resistance of the input capacitor.
    /// Does not affect the charge-balance physics (second-order), but is used by
    /// `compute_v_in_ripple()` to estimate the resistive voltage spike at the
    /// converter input terminals at the switching frequency.
    r_esr_cin: f64,
    /// Thevenin source resistance seen by the input capacitor.
    /// Zero means the cap is instantly recharged to the source voltage each OFF phase.
    r_in: f64,
    /// Actual voltage at the converter input terminal.
    /// Lazy-initialised to the source voltage on the first `tick()` call.
    pub v_in_cap: Voltage,

    /// Input cable / trace inductance.  Zero disables LC input-filter modelling
    /// (falls back to the simpler RC recharge approximation).
    l_in: Inductance,
    /// Current flowing through `l_in` at the end of the previous cycle.
    pub i_in_cap: Current,

    /// Estimated peak-to-peak voltage ripple at the converter input terminals at
    /// the switching frequency, computed inside `tick()` and valid to read after it
    /// returns.  Useful as a per-cycle conducted-emissions indicator.
    pub v_in_ripple_est: Voltage,
}

#[derive(Copy, Clone)]
pub struct Vec2 {
    pub x: T,
    pub y: T,
}

impl Vec2 {
    fn new(x: T, y: T) -> Self {
        Self { x, y }
    }
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
        Voltage(self.v_out.0 + self.r_esr * (self.i_inductor.0 - i_load.0))
    }

    pub fn new(
        period: Time,
        c_out: Capacitance,
        l_inductor: Inductance,
        slope_amp_per_sec: T,
        topology: Topology,
        r_series: Resistance,
        r_esr: f64,
        c_in: Capacitance,
        r_esr_cin: f64,
        r_in: f64,
        l_in: Inductance,
    ) -> Self {
        Self {
            period,
            c_out,
            v_out: Voltage(0.0),
            i_inductor: Current(0.0),
            l_inductor,
            slope_amp_per_sec,
            topology,
            r_series,
            r_esr,
            c_in,
            r_in,
            v_in_cap: Voltage(0.0),
            r_esr_cin,
            l_in,
            i_in_cap: Current(0.0),
            v_in_ripple_est: Voltage(0.0),
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
        if self.c_in.0 > 0.0 && self.v_in_cap.0 == 0.0 {
            self.v_in_cap = v_in;
        }
        // When C_in = 0 the source is ideal (no droop); otherwise use the tracked cap voltage.
        let v_eff = if self.c_in.0 > 0.0 { self.v_in_cap } else { v_in };

        match self.topology {
            Topology::Buck => {
                let v_ind_on = v_eff - self.v_out;

                // V = L di/dt
                // di/dt = V/L
                let di_dt_on = v_ind_on.0 / self.l_inductor.0;

                let i_on_func = math::rlc(
                    v_eff,
                    self.v_out,
                    self.i_inductor,
                    self.l_inductor,
                    self.c_out,
                    self.r_series,
                );

                let t_on_guess =
                    Time((trip_current - self.i_inductor).0 / (di_dt_on - self.slope_amp_per_sec));
                let t_on = Time(
                    i_on_func
                        .intersects_at(
                            Line {
                                k: self.slope_amp_per_sec,
                                m: trip_current.0,
                            },
                            t_on_guess.0,
                        )
                        .unwrap(),
                );

                let q_old = 0.0;

                // Total charge in capacitor at end of ON-phase
                let q_on;

                // current at end of the ON-phase
                let i_max;
                if t_on.0 < 0.0 {
                    i_max = self.i_inductor;
                    q_on = q_old;
                } else if t_on.0 > self.period.0 {
                    // Clamp to 100%
                    i_max = Current(i_on_func.f(self.period.0));
                    q_on = i_on_func.integral(0.0).f(self.period.0) + q_old;
                } else {
                    i_max = Current(i_on_func.f(t_on.0));
                    q_on = i_on_func.integral(0.0).f(t_on.0) + q_old;
                }

                let t_on = Time(t_on.0.clamp(0.0, self.period.0));
                let t_off = self.period - t_on;

                // EMI estimate: computed while i_inductor / i_in_cap are still start-of-cycle.
                self.v_in_ripple_est = self.compute_v_in_ripple(i_max, t_on);

                // Sample load current once at the initial output voltage.  The ON/OFF
                // voltages differ by at most a few mV of ripple, so two samples would give
                // negligibly different results while causing stateful loads (e.g. Battery) to
                // advance their internal state twice per cycle.
                let load_current = i_out(self.v_out);
                let q_out_on = load_current.0 * t_on.0;
                // Net cap voltage at start of OFF phase: inductor charge in minus load drain.
                let v_out_at_off = self.v_out + Voltage((q_on - q_out_on) / self.c_out.0);

                let i_off_func = math::rlc(
                    Voltage(0.0),
                    v_out_at_off,
                    i_max,
                    self.l_inductor,
                    self.c_out,
                    self.r_series,
                );
                let i_final = Current(i_off_func.f(t_off.0));

                let q_off = i_off_func.integral(0.0).f(t_off.0);
                let q_out_off = load_current.0 * t_off.0;
                self.v_out = v_out_at_off + Voltage((q_off - q_out_off) / self.c_out.0);
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
                    - self.r_series.0 * (self.i_inductor.0 + trip_current.0) / 2.0;
                let di_dt_on = v_on_eff / self.l_inductor.0;
                let i_on_line = Line { k: di_dt_on, m: self.i_inductor.0 };

                let t_on_guess = Time(
                    (trip_current - self.i_inductor).0 / (di_dt_on - self.slope_amp_per_sec)
                );
                // intersects_at with two Lines converges in exactly 1 Newton step (guess is analytic)
                let t_on = Time(i_on_line.intersects_at(
                    Line { k: self.slope_amp_per_sec, m: trip_current.0 },
                    t_on_guess.0,
                ).unwrap());

                let i_max;
                let t_on = if t_on.0 < 0.0 {
                    i_max = self.i_inductor; Time(0.0)
                } else if t_on.0 > self.period.0 {
                    i_max = Current(i_on_line.f(self.period.0)); self.period
                } else {
                    i_max = Current(i_on_line.f(t_on.0)); t_on
                };
                let t_off = self.period - t_on;

                // EMI estimate: computed while i_inductor / i_in_cap are still start-of-cycle.
                self.v_in_ripple_est = self.compute_v_in_ripple(i_max, t_on);

                // During ON: cap is decoupled from the inductor but still drives the load,
                // so it droops by q_out_on / C before the OFF phase begins.
                // Sample load current once — see Buck branch for rationale.
                let load_current = i_out(self.v_out);
                let q_out_on = load_current.0 * t_on.0;
                let v_out_at_off = self.v_out - Voltage(q_out_on / self.c_out.0);

                // OFF phase: L and C coupled, starting from the drooped cap voltage.
                // Boost:     V_in drives L+C in series (energy transferred from L+source to C).
                // BuckBoost: no source, L discharges into cap.
                let v_off_source = match self.topology {
                    Topology::Boost => v_eff,
                    Topology::BuckBoost | Topology::Buck => Voltage(0.0),
                };
                let i_off_func = math::rlc(v_off_source, v_out_at_off, i_max,
                                            self.l_inductor, self.c_out, self.r_series);
                let i_final = Current(i_off_func.f(t_off.0));

                let q_in = i_off_func.integral(0.0).f(t_off.0);
                let q_out_off = load_current.0 * t_off.0;
                self.v_out = v_out_at_off + Voltage((q_in - q_out_off) / self.c_out.0);
                self.i_inductor = i_final;

                // BuckBoost: only ON phase draws from input; cap recovers during OFF.
                // Boost:     input is in-circuit during OFF too, so add q_in as extra draw;
                //            recharge time approximated as the full switching period.
                let q_input = i_on_line.integral(0.0).f(t_on.0)
                    + if matches!(self.topology, Topology::Boost) { q_in } else { 0.0 };
                let t_recharge = if matches!(self.topology, Topology::Boost) {
                    self.period
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
        if self.c_in.0 == 0.0 {
            return Voltage(0.0);
        }
        // Capacitive: net charge drawn from C_in (converter minus cable supply)
        let i_avg_on = (self.i_inductor.0 + i_max.0) / 2.0;
        let net_charge = (i_avg_on - self.i_in_cap.0).max(0.0) * t_on.0;
        let v_cap = net_charge / self.c_in.0;
        // ESR: peak current through C_in at the end of the ON phase
        let i_peak_cap = (i_max.0 - self.i_in_cap.0).max(0.0);
        let v_esr = self.r_esr_cin * i_peak_cap;
        Voltage(v_cap + v_esr)
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
        if self.c_in.0 == 0.0 {
            return;
        }
        // Cable current partially supplies C_in during the ON phase.
        let q_cable_on = self.i_in_cap.0 * t_on.0;
        self.v_in_cap = self.v_in_cap - Voltage((q_drawn - q_cable_on) / self.c_in.0);

        if self.l_in.0 > 0.0 {
            // Full LC input filter: evolve RLC from current (v_in_cap, i_in_cap).
            let resp = math::rlc(
                v_source, self.v_in_cap, self.i_in_cap,
                self.l_in, self.c_in, Resistance(self.r_in),
            );
            self.v_in_cap = self.v_in_cap
                + Voltage(resp.integral(0.0).f(t_recharge.0) / self.c_in.0);
            self.i_in_cap = Current(resp.f(t_recharge.0));
        } else if self.r_in == 0.0 {
            self.v_in_cap = v_source;
        } else {
            let tau = self.r_in * self.c_in.0;
            self.v_in_cap +=
                Voltage((1.0 - f64::exp(-t_recharge.0 / tau)) * (v_source.0 - self.v_in_cap.0));
        }
    }
}

fn area_under(p0: Vec2, p1: Vec2, p2: Vec2) -> T {
    0.5 * ((p1.x - p0.x) * (p0.y + p1.y) + (p2.x - p1.x) * (p1.y + p2.y))
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
