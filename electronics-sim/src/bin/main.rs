
use electronics_sim::{
    Capacitance, Current, CurrentConduction, CurrentModeConverter, Inductance, Parameters,
    Resistance, Time, Topology, Voltage,
};

fn main() {
    #[cfg(feature = "rerun")]
    let rec = rerun::RecordingStreamBuilder::new("rerun_example_box3d_batch")
        .spawn()
        .unwrap();
    let t_period = Time(1.0e-6);
    let slope_amp_per_sec = 0.0;

    let params = Parameters{
        period: t_period,
        slope_amp_per_sec,
        r_series: Resistance(0.0),
        r_esr: Resistance(0.0),
        c_out: Capacitance(10.0e-6),
        l_inductor: Inductance(2e-6),
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

    let mut time = Time(0.0);
    for i in 0..1000 {
        let r = if (250..750).contains(&i) {
            Resistance(1.0)
        } else {
            Resistance(3.0)
        };
        let v_in = if i < 500 {
            Voltage(12.0)
        } else {
            Voltage(18.0)
        };
        let trip_current = Current(2.0);
        let mut i_out = Current(0.0);
        let (t_on, i_l_max) = sim.tick(v_in, trip_current, |v| {
            i_out = Current(v.0 / r.0);
            i_out
        });

        #[cfg(feature = "rerun")]
        electronics_sim::plot(&rec, &sim, t_on, i_l_max, v_in, i_out, &mut time);
        //rec.log("iter", &rerun::Scalars::new([i])).unwrap();
    }
}
