use electronics_sim::{
    BuckCurrentModeControl, Capacitance, Current, Inductance, MyThing, Resistance, T, Time,
    Voltage, plot,
};
use half_bridge::{
    control_2p2z::{DacSettings, ParametersBuck, TransferFunction, TwoPoleTwoZeroParams},
    types,
};
use pid::Pid;

const T_PERIOD: Time = Time(1.0e-6);
const C_OUT: Capacitance = Capacitance(47.0e-6);
const L_INDUCTOR: Inductance = Inductance(2e-6);

const MAX_LSB: f64 = 4095.0;
const MAX_CURRENT: Current = Current(22.0);
const PARAMS: ParametersBuck = ParametersBuck {
    v_in: 12.0,
    v_out: 8.0,
    c_out: C_OUT.0,
    f_sw: 1e6,
    l_inductor: L_INDUCTOR.0,
    //r_esr_inductor: 4.08e-3,   // 4.08mOhm typical
    r_esr_out_cap: 1e-3,       // todo
    current_sense_gain: 0.066, // 66mV/A
    i_load: 2.0,
    v_diode: 0.0,
    phase_margin: half_bridge::control_2p2z::PhaseMargin::Manual {
        phase_margin: 75.0f64.to_radians(),
    },
}; /*
const MAX_LSB: f64 = 1023.0;
const PARAMS: ParametersBuck = ParametersBuck {
v_in: 16.0,
v_out: 8.0,
c_out: 440e-6,
f_sw: 200e3,
l_inductor: 22e-6,
//r_esr_inductor: 4.08e-3,   // 4.08mOhm typical
r_esr_out_cap: 31e-3,      // todo
current_sense_gain: 0.48, // 66mV/A
i_load: 2.0,
v_diode: 0.6,
phase_margin: half_bridge::control_2p2z::PhaseMargin::Manual { phase_margin: 75.0f64.to_radians() }
//t_adc_sample_to_dac_out: 0.0,
};*/

// 0:    0.00V  <-20A
// 2047: 1.65V  0A
// 4095: 3.30V  >+20A

const DAC_LSB_TO_V: T = 3.3 / MAX_LSB;
const AMP_PER_LSB: T = DAC_LSB_TO_V / PARAMS.current_sense_gain;
const ASD: T = AMP_PER_LSB * MAX_LSB;
const SLOPE_AMP_PER_SEC: T = DAC_SETTINGS.dac_slope / PARAMS.current_sense_gain;
const C: T = SLOPE_AMP_PER_SEC / PARAMS.f_sw;

const TF_AND_DAC: (TransferFunction, DacSettings) = PARAMS.to_transfer_function();
const TRANSFER_FUNC: TransferFunction = TF_AND_DAC.0;
const DAC_SETTINGS: DacSettings = TF_AND_DAC.1;
const COMP_WEIGHTS: TwoPoleTwoZeroParams<f32> = TRANSFER_FUNC.to_2p2z();

fn main() {
    let rec = rerun::RecordingStreamBuilder::new("rerun_example_box3d_batch")
        .spawn()
        .unwrap();

    //i(Voltage(12.0), todo!(), todo!(), L_INDUCTOR, C_OUT, Resistance(10e-3), T_PERIOD);

    //let sim = MyThing::new(T_PERIOD, C_OUT, L_INDUCTOR, SLOPE_AMP_PER_SEC, AMP_PER_LSB, AMP_AT_0LSB);
    let sim = BuckCurrentModeControl::new(T_PERIOD, C_OUT, L_INDUCTOR, SLOPE_AMP_PER_SEC);

    let target = Voltage(5.0);

    let mut best = (0.0, 0.0);
    let mut v_out_max_min = f64::MAX;
    //for x in 0..1000 {
    let kp = 0.433;
    let ki = 0.025;

    let v_out_max = foo(kp, ki, target, sim, None);

    if v_out_max < v_out_max_min {
        best = (ki, kp);
        v_out_max_min = v_out_max;
    }
    //}
    let (ki, kp) = best;
    dbg!(kp, ki, v_out_max_min);

    //let kp = 0.999;
    //let ki = 0.025;
    foo(kp, ki, target, sim, Some(&rec));
}

fn foo(
    kp: f32,
    ki: f32,
    target: Voltage,
    mut sim: BuckCurrentModeControl,
    rec: Option<&rerun::RecordingStream>,
) -> f64 {
    let mut comp = Pid::new(target.0, 1.0);
    comp.p(kp, 1e18).i(ki, 1e9).d(0.0, 1e9);
    comp.output_limit = 0.75;

    let mut comp = COMP_WEIGHTS.to_controller();

    let loads = [3.0, 1.0, 10000.0, 1.0, 100.0];

    let mut time = Time(0.0);
    let iter = 1000;
    let mut v_out_max = 0.0f64;
    for i in 0..iter {
        let r = loads[i * loads.len() / iter];
        let v_in = if i < 500 {
            Voltage(12.0)
        } else {
            Voltage(18.0)
        };

        let mut i_out = Current(0.0);

        //let output = comp.next_control_output(sim.buck.v_out.0).output;

        //dbg!(output);
        let output = comp.update((target - sim.v_out).0 as f32);
        // Pin voltage to DAC code
        let trip_current = Current(output as T /* * MAX_CURRENT.0*/); //(output / AMP_PER_LSB as f32 - LSB_AT_ZERO_AMP as f32).clamp(LSB_AT_ZERO_AMP as f32, MAX_LSB as f32);
        //panic!("Boopi: {trip_current}");
        let (p1, _p2) = sim.tick(v_in, trip_current /*as u16*/, |v| {
            i_out = Current(v.0 / r);
            i_out
        });

        if let Some(rec) = rec {
            plot(&rec, &sim, p1, v_in, i_out, &mut time);
        }
        v_out_max = v_out_max.max(sim.v_out.0);
        //rec.log("iter", &rerun::Scalars::new([i])).unwrap();
    }

    v_out_max
}
/*
// TODO: Verify this
fn bar(v_in: Voltage, l: Inductance, c: Capacitance, r: Resistance, t: Time) {
    //let u_l = l * di_dt;
    //let u_c = Q / c;
    //let u_r = r * i;
    //
    //let q = (v_in / L) / s(s ^ 2 + (r / 2) * s + (inv_c / 2));

    // ------- The math below this line was generated with the help of ChatGPT -------

    // let k = v_in.0 / l.0;
    let a = r.0 / 2.0;
    let b = 1.0 / (2.0 * c.0);
    let ohmega = T::sqrt(b - a * a / 4.0);
    let t = t.0;

    // let q = k / b
    //     - (k / b) * T::exp(-a * t / 2.0) * T::cos(ohmega * t)
    //     - (k * a) / (2.0 * b * ohmega) * T::exp(-a * t / 2.0) * T::sin(ohmega * t)
    //     - (k * a) / (b * ohmega) * T::exp(-a * t / 2.0) * T::sin(ohmega * t);

    // i = dq_dt
    // let i = (k / b)
    //     * T::exp(-a * t / 2.0)
    //     * (-a * T::cos(ohmega * t)
    //         + (ohmega + (3.0 * a * a) / (4.0 * ohmega)) * T::sin(ohmega * t));

    // Simplified
    let i = (c.0 * v_in.0 / l.0)
        * T::exp(-r.0 * t / 4.0)
        * (-r.0 * T::cos(ohmega * t)
            + (2.0 * ohmega + (3.0 * r.0 * r.0) / (8.0 * ohmega)) * T::sin(ohmega * t));
}*/

// https://www.youtube.com/watch?v=m27OkXwBbuk
fn i(v_in: Voltage, v_cout_old: Voltage, i_old: Current, l: Inductance, c: Capacitance, r: Resistance) -> WonkyF {
    let q0 = c.0 * v_cout_old.0;

    // v_in = r * i + l * di/dt + q/c
    // d_q/d_t = i
    //
    // Differentiating w.r.t. `t` where v_in is constant:
    // r * d_i/d_t + l * d2_i / d2_t + i / c = 0
    // <=> (divide by `l`)
    // d2_i / d2_t + (r * di)/(l * dt) + i / (l * c) = 0
    //
    // Characteristic equation:
    // s^2 + r/l*s + 1 / (l*c) = 0
    //
    // s1 and s2 are the roots
    //
    // sq_root = sqrt((r/2l)^2 - 1/(l*c))
    // s1 = -r/(2*l) + sqrt((r/2l)^2 - 1/(l*c))
    // s2 = -r/(2*l) - sqrt((r/2l)^2 - 1/(l*c))
    //
    // s1 = -a + sqrt(a^2-ohmega^2) = -a + sq_root
    // s2 = -a - sqrt(a^2-ohmega^2) = -a - sq_root
    //
    let a = r.0 / (2.0 * l.0);
    let ohmega = 1.0 / T::sqrt(l.0 * c.0);
    let b = T::sqrt(a * a - ohmega * ohmega);
    //
    // i(t) = k1 * T::exp(s1*t) + k2 * T::exp(s2*t)

    match a.partial_cmp(&ohmega).unwrap() {
        std::cmp::Ordering::Less => {
            
            // Underdamped response
            // t > 0
            let s1 = -a + T::sqrt(a*a + ohmega*ohmega);
            let s2 = -a - T::sqrt(a*a + ohmega*ohmega);
            //panic!("Underdamped response: {s1} {s2}");
            //let i = k1 * T::exp(s1 * t.0) + k2 * T::exp(s2 * t.0);
            //k1 + k2 = old_i;

            // u = r*i + l * di_dt + q0; // Adderar man initial spänning av cappen här?
            let di_dt0 = (v_in.0 - r.0 * i_old.0 - q0) / l.0; // Adderar man initial spänning av cappen här?
            //r*di_dt + l * d2i_d2t + i / c;

            //-----
            
            //let di_dt = s1*k1 * T::exp(s1 * t) + s2*k2 * T::exp(s2 * t);
            //k1 + k2 = i_old;
            //k1 = i_old - k2;

            //let di_dt0 = s1*(i_old - k2) * T::exp(s1 * t) + s2*k2 * T::exp(s2 * t);
            //let di_dt0 = s1*i_old - s1*k2 + s2 * k2;
            //let di_dt0 = s1*i_old + (s2 - s1) * k2;
            //let di_dt0 - s1*i_old =  (s2 - s1) * k2;
            let k2 = (di_dt0 - s1 * i_old.0) / (s2 - s1);
            let k1 = i_old.0 - k2;

            // k1 * T::exp(s1 * t) + k2 * T::exp(s2 * t) = k*t + m;

            // this seem to be the one we have...
            //let ohmega_d = T::sqrt(ohmega * ohmega - a * a);
            //let i = T::exp(-a * t) * ((k1 + k2) * T::cos(ohmega_d * t) + j(k1 - k2) * sin(ohmega_d * t));

            WonkyF{ k1, k2, s1, s2 }
        },
        std::cmp::Ordering::Equal => {
            panic!("Critically damped")
            // Critically damped
            // t > 0
            // let i =  T::exp(-a * t) * (k1 + k2 * t);
        }
        std::cmp::Ordering::Greater => {
            panic!("Overdamped response")
            // Overdamped
            // t > 0
            // let i = k1 * T::exp(s1 * t) + k2 * T::exp(s2 * t);
        }
    }
}

pub trait Func {
    type Derivetive: Func;
    fn f(&self, x: T) -> T;
    fn prim(&self) -> Self::Derivetive;

    fn intersects_at(&self, rhs: impl Func, guess: T) -> Option<T> {
        let f = |x| x - (self.f(x) - rhs.f(x)) / (self.prim().f(x) - rhs.prim().f(x));

        let mut x = guess;
        for _ in 0..10 {
            x = f(x);
        }

        Some(x)
    }
}

struct WonkyF {
    k1: T,
    k2: T,

    s1: T,
    s2: T,
}

impl Func for WonkyF {
    type Derivetive = Self;

    fn f(&self, x: T) -> T {
        self.k1 * T::exp(self.s1 * x) + self.k2 * T::exp(self.s2 * x)
    }

    fn prim(&self) -> WonkyF {
        WonkyF{
            k1: self.s1 * self.k1,
            k2: self.s2 * self.k2,
            
            s1: self.s1,
            s2: self.s2,
        }
    }
}

struct Line {
    k: T,
    m: T
}

impl Func for Line {
    type Derivetive = Self;

    fn f(&self, x: T) -> T {
        self.k * x + self.m
    }

    fn prim(&self) -> Self {
        Self {
            k: 0.0,
            m: self.k,
        }
    }
}