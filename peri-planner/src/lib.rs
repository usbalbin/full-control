pub mod af_view;
pub mod analog_view;
pub mod app;
pub mod board;
pub mod c531_design;
pub mod c531_view;
pub mod catalog;
pub mod catalog_view;
pub mod codegen;
pub mod comms_view;
pub mod constraint;
pub mod desc_asset;
pub mod dropin;
pub mod dropin_view;
pub mod fabric_data;
pub mod fabric_data_c5;
pub mod fabric_view;
pub mod frontend_plan;
pub mod frontend_view;
pub mod g474;
pub mod h523_design;
pub mod hrtim_view;
pub mod inventory_view;
pub mod mcu;
pub mod mcu_data;
pub mod mcu_pinout;
pub mod mcu_raw;
pub mod package_view;
pub mod peripherals_view;
pub mod phys_pinout;
pub mod picker;
pub mod pin_map_view;
pub mod pin_plan;
pub mod pinout;
pub mod requirements;
pub mod select;
pub mod solver;
pub mod timers_view;
pub mod waveform_view;

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(start)]
pub async fn start() {
    use app::PeriPlannerApp;
    use wasm_bindgen::JsCast as _;

    let canvas = web_sys::window()
        .unwrap()
        .document()
        .unwrap()
        .get_element_by_id("peri_planner_canvas")
        .unwrap()
        .dyn_into::<web_sys::HtmlCanvasElement>()
        .unwrap();

    eframe::WebRunner::new()
        .start(
            canvas,
            eframe::WebOptions::default(),
            Box::new(|cc| Ok(Box::new(PeriPlannerApp::new(cc)))),
        )
        .await
        .expect("failed to start eframe");
}
