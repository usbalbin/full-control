// Native entry point. On WASM the actual entry is `start()` in lib.rs.

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result<()> {
    use peri_planner::app::PeriPlannerApp;

    eframe::run_native(
        "peri-planner",
        eframe::NativeOptions {
            viewport: eframe::egui::ViewportBuilder::default()
                .with_inner_size([1100.0, 700.0]),
            ..Default::default()
        },
        Box::new(|cc| Ok(Box::new(PeriPlannerApp::new(cc)))),
    )
}

#[cfg(target_arch = "wasm32")]
fn main() {} // unused on WASM; trunk invokes start() from lib.rs
