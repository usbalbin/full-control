// Native entry point.  On WASM the actual entry is `start()` in lib.rs.

#[cfg(all(not(target_arch = "wasm32"), feature = "text-log"))]
fn main() {
    use buck_sim_ui::sim::{run_simulation, SimParams};
    let p = SimParams { load_kind: buck_sim_ui::sim::LoadKind::Battery, ..SimParams::default() };
    match run_simulation(&p) {
        None => eprintln!("Invalid simulation parameters"),
        Some(data) => {
            println!("t_ms,v_out,duty_pct,i_l_min,i_l_max");
            for pt in data {
                println!(
                    "{:.4},{:.6},{:.4},{:.6},{:.6}",
                    pt.t_ms, pt.v_out, pt.duty_pct, pt.i_l_min, pt.i_l_max
                );
            }
        }
    }
}

#[cfg(all(not(target_arch = "wasm32"), not(feature = "text-log")))]
fn main() {
    eframe::run_native(
        "Buck Converter Simulator",
        eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default().with_inner_size([1200.0, 700.0]),
            ..Default::default()
        },
        Box::new(|_cc| Ok(Box::new(buck_sim_ui::app::BuckSimApp::default()))),
    )
    .expect("failed to start native window");
}

#[cfg(target_arch = "wasm32")]
fn main() {} // unused on WASM; trunk invokes start() from lib.rs
