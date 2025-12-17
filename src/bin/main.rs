use electronics_sim::{
    BuckCurrentModeControl, Capacitance, Current, Inductance, Resistance, Time, Voltage, plot,
};

fn main() {
    let rec = rerun::RecordingStreamBuilder::new("rerun_example_box3d_batch")
        .spawn()
        .unwrap();
    let t_period = Time(1.0e-6);
    let mut sim = BuckCurrentModeControl::new(t_period, Capacitance(10.0e-6), Inductance(2e-6));

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
        let (p1, _p2) = sim.tick(v_in, trip_current, |v| {
            i_out = Current(v.0 / r.0);
            i_out
        });

        plot(&rec, &sim, p1, v_in, i_out, &mut time);
        //rec.log("iter", &rerun::Scalars::new([i])).unwrap();
    }
}
