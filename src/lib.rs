use rerun::RecordingStream;
use std::ops::{Add, AddAssign, Neg, Sub};

pub fn plot(
    rec: &RecordingStream,
    sim: &BuckCurrentModeControl,
    p1: Vec2,
    v_in: Voltage,
    i_out: Current,
    time: &mut Time,
) {
    rec.set_timestamp_secs_since_epoch("time", time.0 + p1.x - 10e-9);
    rec.log("sw", &rerun::Scalars::new([1.0])).unwrap();

    rec.set_timestamp_secs_since_epoch("time", time.0 + p1.x);
    rec.log("sw", &rerun::Scalars::new([0.0])).unwrap();
    rec.log("i_inductor", &rerun::Scalars::new([p1.y])).unwrap();

    *time += sim.period;
    rec.set_timestamp_secs_since_epoch("time", time.0 - 10e-9);
    rec.log("sw", &rerun::Scalars::new([0.0])).unwrap();
    
    rec.set_timestamp_secs_since_epoch("time", time.0);
    rec.log("sw", &rerun::Scalars::new([1.0])).unwrap();
    rec.log("i_inductor", &rerun::Scalars::new([sim.i_inductor.0]))
        .unwrap();

    rec.log("v_in", &rerun::Scalars::new([v_in.0])).unwrap();
    rec.log("v_in", &rerun::Scalars::new([v_in.0])).unwrap();
    rec.log("v_out", &rerun::Scalars::new([sim.v_out.0]))
        .unwrap();
    rec.log("i_out", &rerun::Scalars::new([i_out.0])).unwrap();
    rec.log("d", &rerun::Scalars::new([p1.x / sim.period.0])).unwrap();
}

pub type T = f64;

#[derive(Clone, Copy)]
pub struct MyThing {
    pub buck: BuckCurrentModeControl,
    amp_per_lsb: T,
    i_at_0lsb: Current,
}

impl MyThing {
    pub fn new(period: Time, c_out: Capacitance, l_inductor: Inductance, slope_amp_per_sec: T, amp_per_lsb: T, i_at_0lsb: Current) -> Self {
        Self { buck: BuckCurrentModeControl::new(period, c_out, l_inductor, slope_amp_per_sec), amp_per_lsb, i_at_0lsb }
    }

    pub fn tick(&mut self, v_in: Voltage, trip_current: u16, i_out: impl FnOnce(Voltage) -> Current) -> (Vec2, Vec2) {
        // 4095 -> 20
        // 0 -> -20
        let trip_current = Current(T::from(trip_current) * self.amp_per_lsb) + self.i_at_0lsb;
        self.buck.tick(v_in, trip_current, i_out)
    }
}

#[derive(Clone, Copy)]
pub struct BuckCurrentModeControl {
    period: Time,

    c_out: Capacitance,
    pub v_out: Voltage,

    pub i_inductor: Current,
    l_inductor: Inductance,

    // This should normally be negative
    slope_amp_per_sec: T,
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

impl BuckCurrentModeControl {
    pub fn new(period: Time, c_out: Capacitance, l_inductor: Inductance, slope_amp_per_sec: T) -> Self {
        Self {
            period,
            c_out,
            v_out: Voltage(0.0),
            i_inductor: Current(0.0),
            l_inductor,
            slope_amp_per_sec,
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
    ) -> (Vec2, Vec2) {
        let v_ind_on = v_in - self.v_out;
        let v_ind_off = -self.v_out;

        // V = L di/dt
        // di/dt = V/L

        let di_dt_on = v_ind_on.0 / self.l_inductor.0;
        let di_dt_off = v_ind_off.0 / self.l_inductor.0;
        // y = kx + m
        // y - m = kx
        // x = (y - m) / k

        // i = di_dt*t + old_i = i_trip + slope * t
        // di_dt*t = i_trip + slope * t - old_i
        // di_dt*t - slope * t = i_trip - old_i
        // (di_dt - slope) * t = i_trip - old_i
        // t = (i_trip - old_i) / (di_dt - slope)

        // TODO: Slope compensation
        let t_on = Time((trip_current - self.i_inductor).0 / (di_dt_on - self.slope_amp_per_sec));

        let i_max;
        if t_on.0 < 0.0 {
            i_max = self.i_inductor;
        } else if t_on.0 > self.period.0 {
            i_max = Current(self.i_inductor.0 + di_dt_on * self.period.0);
        } else {
            i_max = Current(self.i_inductor.0 + di_dt_on * t_on.0);
        }
        let t_on = Time(t_on.0.clamp(0.0, self.period.0));

        let t_off = self.period - t_on;

        let i_final = i_max + Current(di_dt_off * t_off.0);

        let p0 = Vec2::new(0.0, self.i_inductor.0);
        let p1 = Vec2::new(t_on.0, i_max.0);
        let p2 = Vec2::new(self.period.0, i_final.0);

        let q_in = area_under(p0, p1, p2);
        let q_out = i_out(self.v_out).0 * self.period.0;

        // i = c * dv/dt;
        // i = c * dv/dt;

        self.v_out += Voltage((q_in - q_out) / self.c_out.0);
        self.i_inductor = i_final;

        (p1, p2)
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
