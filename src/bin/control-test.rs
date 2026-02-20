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
const C_OUT: Capacitance = Capacitance(470.0e-6);
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

    let loads = [3.0, 1.0, f64::MAX, 1.0, 100.0];

    let mut time = Time(0.0);

    // Soft start: ramp the voltage reference from 0 to target over this many cycles.
    let soft_start_cycles = 200_usize;
    for i in 0..soft_start_cycles {
        let soft_target = Voltage(target.0 * (i + 1) as f64 / soft_start_cycles as f64);
        let mut i_out = Current(0.0);
        let output = comp.update((soft_target - sim.v_out).0 as f32);
        let trip_current = Current((output as T / PARAMS.current_sense_gain).clamp(0.0, MAX_CURRENT.0));
        let (t_on, i_l_max) = sim.tick(Voltage(12.0), trip_current, |v| {
            i_out = Current(v.0 / loads[0]);
            i_out
        });
        if let Some(rec) = rec {
            plot(&rec, &sim, t_on, i_l_max, Voltage(12.0), i_out, &mut time);
        }
    }

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
        // Controller output is a current-reference voltage (V); divide by current_sense_gain (V/A) to get Amps.
        let trip_current = Current((output as T / PARAMS.current_sense_gain).clamp(0.0, MAX_CURRENT.0));
        //panic!("Boopi: {trip_current}");
        let (t_on, i_l_max) = sim.tick(v_in, trip_current /*as u16*/, |v| {
            i_out = Current(v.0 / r);
            i_out
        });

        if let Some(rec) = rec {
            plot(&rec, &sim, t_on, i_l_max, v_in, i_out, &mut time);
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
