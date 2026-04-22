use full_control::control_2p2z::*;

fn show(label: &str, params: ParametersBuck) {
    let d = match params.topology {
        Topology::Buck => (params.v_out + params.v_diode) / params.v_in,
        Topology::Boost => 1.0 - params.v_in / (params.v_out - params.v_diode),
        Topology::BuckBoost => params.v_out / (params.v_in + params.v_out - params.v_diode),
    };
    let v_l_on = match params.topology {
        Topology::Buck => params.v_in - params.v_out - params.v_diode,
        Topology::Boost | Topology::BuckBoost => params.v_in,
    };
    let di_l = v_l_on * d / (params.f_sw * params.l_inductor);
    let dv_out = di_l * (params.r_esr_out_cap + 1.0 / (8.0 * params.f_sw * params.c_out));
    let (_, dac) = ParametersBuck { f_x_divisor: 1.0, ..params }.to_transfer_function();
    let vpp = dac.vpp();
    let b0_max = vpp / (dv_out * 2.0);
    println!("=== {} ===  dv_out={:.2}mV  vpp={:.3}V  b0_max={:.4}",
        label, dv_out*1000.0, vpp, b0_max);
    println!("{:<8} {:>10} {:>12} {:>12}", "divisor", "f_x[Hz]", "b0", "criterion");
    for &divisor in &[1.0f64, 2.5, 3.0, 5.0, 8.0, 10.0, 12.0, 15.0, 20.0, 30.0, 50.0, 100.0, 200.0, 300.0] {
        let p = ParametersBuck { f_x_divisor: divisor, ..params };
        let (tf, _) = p.to_transfer_function();
        let b0 = tf.to_2p2z().b0 as f64;
        let verdict = if b0 < 0.0 { "INVALID(neg)" } else if b0 > b0_max { "LIMIT-CYCLE" } else { "OK" };
        println!("{:<8.1} {:>10.0} {:>12.5} {:>12}", divisor, params.f_sw/divisor, b0, verdict);
    }
    println!("  optimal_f_x_divisor (current, bugged) = {:.1}", params.optimal_f_x_divisor(2.0));
    println!();
}

fn main() {
    let base = ParametersBuck {
        v_out: 13.5, v_diode: 0.0,
        c_out: 47e-6, f_sw: 500e3, l_inductor: 4e-6,
        r_esr_out_cap: 10e-3, current_sense_gain: 0.066,
        i_load: 13.5/6.0, topology: Topology::BuckBoost,
        phase_margin: PhaseMargin::Manual { phase_margin: 75.0f64.to_radians() },
        f_x_divisor: 1.0, cycles_per_tick: 1, v_in: 0.0,
    };
    show("Buck   (Vin=24)", ParametersBuck { v_in: 24.0, ..base });
    show("BB     (Vin=13.5)", ParametersBuck { v_in: 13.5, ..base });
    show("Boost  (Vin=8)", ParametersBuck { v_in: 8.0, ..base });
}
