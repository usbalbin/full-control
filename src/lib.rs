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
            buck: CurrentModeConverter::new(period, c_out, l_inductor, slope_amp_per_sec, Topology::Buck, Resistance(0.0)),
            amp_per_lsb,
            i_at_0lsb,
        }
    }

    pub fn tick(
        &mut self,
        v_in: Voltage,
        trip_current: u16,
        i_out: impl FnOnce(Voltage) -> Current,
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
    pub fn new(
        period: Time,
        c_out: Capacitance,
        l_inductor: Inductance,
        slope_amp_per_sec: T,
        topology: Topology,
        r_series: Resistance,
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
        i_out: impl FnOnce(Voltage) -> Current,
    ) -> (Time, Current) {
        match self.topology {
            Topology::Buck => {
                let v_ind_on = v_in - self.v_out;

                // V = L di/dt
                // di/dt = V/L
                let di_dt_on = v_ind_on.0 / self.l_inductor.0;

                let i_on_func = math::rlc(
                    v_in,
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

                // Vout at end of the ON-phase
                let v_out_max = self.v_out + Voltage(q_on / self.c_out.0);

                let t_on = Time(t_on.0.clamp(0.0, self.period.0));
                let t_off = self.period - t_on;

                let i_off_func = math::rlc(
                    Voltage(0.0),
                    v_out_max,
                    i_max,
                    self.l_inductor,
                    self.c_out,
                    self.r_series,
                );
                let i_final = Current(i_off_func.f(t_off.0));

                // Total charge in the capacitor at the end of the period
                let q_off = i_off_func.integral(0.0).f(t_off.0);
                let q_in = q_on + q_off;
                let q_out = i_out(self.v_out).0 * self.period.0;


                self.v_out += Voltage((q_in - q_out) / self.c_out.0);
                self.i_inductor = i_final;

                (t_on, i_max)
            }

            Topology::Boost | Topology::BuckBoost => {
                // ON phase: C is decoupled (diode reverse-biased), so this is an RL circuit.
                // Exact solution is exponential; approximate as linear with effective V_in
                // computed at the midpoint of the ON-phase current swing.  Error is
                // O((R·ΔI/V_in)²) — negligible for typical R << L·f_sw.
                let v_on_eff = v_in.0
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

                // OFF phase: L and C coupled.
                // Boost:     V_in drives L+C in series (energy transferred from L+source to C).
                // BuckBoost: no source, L discharges into cap.
                let v_off_source = match self.topology {
                    Topology::Boost => v_in,
                    Topology::BuckBoost | Topology::Buck => Voltage(0.0),
                };
                let i_off_func = math::rlc(v_off_source, self.v_out, i_max,
                                            self.l_inductor, self.c_out, self.r_series);
                let i_final = Current(i_off_func.f(t_off.0));

                // Charge balance: only OFF phase inductor current charges the cap
                let q_in = i_off_func.integral(0.0).f(t_off.0);
                let q_out = i_out(self.v_out).0 * self.period.0;
                self.v_out += Voltage((q_in - q_out) / self.c_out.0);
                self.i_inductor = i_final;

                (t_on, i_max)
            }
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
