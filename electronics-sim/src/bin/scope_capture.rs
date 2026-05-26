//! `scope-capture` — render a scope-style time-domain capture of one
//! buck HS-FET turn-on event, driven by board-extracted parasitics.
//!
//! Usage:
//!
//! ```bash
//! # 1. Extract parasitics from a KiCad board (run in kicad_field_solver/):
//! field_solver_cli buck-parasitics \
//!     --board PCB-7.board_geometry.json \
//!     --topology topology.json \
//!     --out-pdn-schema-buck /tmp/buck_parasitics.json
//!
//! # 2. Run the edge-transient sim:
//! cargo run --bin scope-capture --no-default-features -- \
//!     --parasitics /tmp/buck_parasitics.json \
//!     --fet bsc0902nsi \
//!     --driver si10v \
//!     --v-in 12 --v-out 3.3 --i-l 3.0 \
//!     --csv /tmp/scope_capture.csv \
//!     --json /tmp/scope_capture.json
//! ```
//!
//! Default `cargo run` enables the `rerun` feature; pass
//! `--no-default-features` to avoid the rerun dependency (this binary
//! does not use rerun directly — it writes CSV + JSON only).

use std::env;
use std::fs;
use std::path::PathBuf;
use std::process::ExitCode;

use electronics_sim::edge_transient::{
    default_loop_r, simulate_hs_turn_off, simulate_hs_turn_on, DriverModel, EdgeSample,
    EdgeWaveforms, FetModel, OperatingPoint, SimConfig,
};
use pdn_schema::BuckParasiticSet;

const HELP: &str = "\
scope-capture — edge-transient buck-converter scope-style waveform.

REQUIRED:
  --parasitics PATH       pdn_schema::BuckParasiticSet JSON (from
                          field_solver_cli buck-parasitics
                          --out-pdn-schema-buck).
  --v-in VOLTS            input bus voltage.
  --v-out VOLTS           output capacitor voltage.
  --i-l AMPS              inductor current at the turn-on moment.

FET PRESETS:
  --fet bsc0902nsi        40 V Si MOSFET (default).
  --fet epc2034c          200 V GaN HEMT.

DRIVER PRESETS:
  --driver si10v          10 V Si MOSFET driver (default).
  --driver gan5v          5 V GaN driver, V_off = 0 V.
  --driver gan5v_neg      5 V GaN driver, V_off = -2 V (anti-Cdv/dt).

OPTIONAL:
  --edge turn-on|turn-off Which switching event to capture (default: turn-on).
  --c-in FARAD            Input bypass cap value (enables V_in ripple modelling).
  --c-y-chassis FARAD     Assembly-level Y-cap SW→chassis (CM EMI path).
  --l-inductor HENRY      Buck filter inductance (default 4.7e-6).
  --loop-r OHM            Power-loop DC resistance (default 20e-3).
  --duration SECONDS      Sim window (default auto from L·C ring period).
  --dt SECONDS            Time step (default auto, ≤50 ps).
  --csv PATH              Write samples as CSV.
  --json PATH             Write summary + samples as JSON.
  -h, --help              Show this help.
";

#[derive(Default, Debug)]
struct Args {
    parasitics: Option<PathBuf>,
    v_in: Option<f64>,
    v_out: Option<f64>,
    i_l: Option<f64>,
    fet: Option<String>,
    driver: Option<String>,
    l_inductor: Option<f64>,
    loop_r: Option<f64>,
    duration: Option<f64>,
    dt: Option<f64>,
    csv: Option<PathBuf>,
    json: Option<PathBuf>,
    edge: Option<String>,
    c_in: Option<f64>,
    c_y: Option<f64>,
    c_boot: Option<f64>,
}

fn parse_args() -> Result<Args, String> {
    let mut a = Args::default();
    let mut it = env::args().skip(1);
    while let Some(k) = it.next() {
        let need = |it: &mut std::iter::Skip<env::Args>, k: &str| -> Result<String, String> {
            it.next().ok_or_else(|| format!("{} requires a value", k))
        };
        match k.as_str() {
            "-h" | "--help" => { println!("{HELP}"); std::process::exit(0); }
            "--parasitics" => a.parasitics = Some(PathBuf::from(need(&mut it, &k)?)),
            "--v-in" => a.v_in = Some(need(&mut it, &k)?.parse().map_err(|e| format!("{e}"))?),
            "--v-out" => a.v_out = Some(need(&mut it, &k)?.parse().map_err(|e| format!("{e}"))?),
            "--i-l" => a.i_l = Some(need(&mut it, &k)?.parse().map_err(|e| format!("{e}"))?),
            "--fet" => a.fet = Some(need(&mut it, &k)?),
            "--driver" => a.driver = Some(need(&mut it, &k)?),
            "--l-inductor" => a.l_inductor = Some(need(&mut it, &k)?.parse().map_err(|e| format!("{e}"))?),
            "--loop-r" => a.loop_r = Some(need(&mut it, &k)?.parse().map_err(|e| format!("{e}"))?),
            "--duration" => a.duration = Some(need(&mut it, &k)?.parse().map_err(|e| format!("{e}"))?),
            "--dt" => a.dt = Some(need(&mut it, &k)?.parse().map_err(|e| format!("{e}"))?),
            "--csv" => a.csv = Some(PathBuf::from(need(&mut it, &k)?)),
            "--json" => a.json = Some(PathBuf::from(need(&mut it, &k)?)),
            "--edge" => a.edge = Some(need(&mut it, &k)?),
            "--c-in" => a.c_in = Some(need(&mut it, &k)?.parse().map_err(|e| format!("{e}"))?),
            "--c-y-chassis" => a.c_y = Some(need(&mut it, &k)?.parse().map_err(|e| format!("{e}"))?),
            "--c-boot" => a.c_boot = Some(need(&mut it, &k)?.parse().map_err(|e| format!("{e}"))?),
            _ => return Err(format!("unknown arg: {k}")),
        }
    }
    Ok(a)
}

fn pick_fet(name: &str) -> Result<FetModel, String> {
    match name {
        "bsc0902nsi" => Ok(FetModel::bsc0902nsi()),
        "epc2034c" => Ok(FetModel::epc2034c()),
        _ => Err(format!("unknown --fet preset {:?}; try bsc0902nsi or epc2034c", name)),
    }
}

fn pick_driver(name: &str) -> Result<DriverModel, String> {
    match name {
        "si10v" => Ok(DriverModel::generic_si_10v()),
        "gan5v" => Ok(DriverModel::generic_gan_5v()),
        "gan5v_neg" => Ok(DriverModel::generic_gan_5v_neg_off()),
        _ => Err(format!("unknown --driver preset {:?}; try si10v, gan5v, or gan5v_neg", name)),
    }
}

fn write_csv(path: &PathBuf, samples: &[EdgeSample]) -> std::io::Result<()> {
    use std::io::Write;
    let mut w = std::io::BufWriter::new(std::fs::File::create(path)?);
    writeln!(w, "t_ns,v_gs_hs,v_gs_ls,v_sw,i_l,i_g_hs,i_d_hs,i_d_ls,i_diode_ls,diode_state")?;
    for s in samples {
        writeln!(
            w,
            "{:.4},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:?}",
            s.t_s * 1e9, s.v_gs_hs, s.v_gs_ls, s.v_sw, s.i_l, s.i_g_hs,
            s.i_d_hs, s.i_d_ls, s.i_diode_ls, s.diode_state,
        )?;
    }
    Ok(())
}

fn write_json(path: &PathBuf, w: &EdgeWaveforms) -> Result<(), String> {
    let j = serde_json::json!({
        "summary": {
            "n_samples": w.samples.len(),
            "t_v_th_crossed_ns": w.t_v_th_crossed.map(|t| t * 1e9),
            "t_rr_entered_ns": w.t_rr_entered.map(|t| t * 1e9),
            "t_rr_done_ns": w.t_rr_done.map(|t| t * 1e9),
            "i_rr_peak_a": w.i_rr_peak,
            "v_sw_overshoot_v": w.v_sw_overshoot,
            "f_ring_est_hz": w.f_ring_est,
        },
        "samples": w.samples,
    });
    fs::write(path, serde_json::to_string_pretty(&j).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())
}

fn run() -> Result<(), String> {
    let a = parse_args()?;
    let parasitics_path = a.parasitics.as_ref()
        .ok_or("--parasitics is required (see --help)")?;
    let v_in = a.v_in.ok_or("--v-in is required")?;
    let v_out = a.v_out.ok_or("--v-out is required")?;
    let i_l = a.i_l.ok_or("--i-l is required")?;

    let parasitics_text = fs::read_to_string(parasitics_path)
        .map_err(|e| format!("read {:?}: {}", parasitics_path, e))?;
    let parasitics = BuckParasiticSet::from_json(&parasitics_text)
        .map_err(|e| format!("parse {:?}: {}", parasitics_path, e))?;

    let fet = pick_fet(a.fet.as_deref().unwrap_or("bsc0902nsi"))?;
    let driver = pick_driver(a.driver.as_deref().unwrap_or("si10v"))?;
    let op = OperatingPoint {
        v_in, v_out, i_l_init: i_l,
        loop_r: a.loop_r.unwrap_or_else(|| default_loop_r(&parasitics)),
        l_inductor: a.l_inductor.unwrap_or(4.7e-6),
        c_in_farad: a.c_in.unwrap_or(0.0),
        c_y_chassis_farad: a.c_y.unwrap_or(0.0),
        c_boot_farad: a.c_boot.unwrap_or(0.0),
    };
    let mut cfg = SimConfig::auto(&parasitics, &fet);
    if let Some(dur) = a.duration { cfg.duration = dur; }
    if let Some(dt) = a.dt { cfg.dt = dt; }

    eprintln!(
        "# scope-capture\n\
         #  parasitics: {} → power_loop_l = {:.2} nH, sw_node_c = {:.3} pF\n\
         #  FET:        c_iss={:.0} pF, c_oss={:.0} pF, c_rss={:.0} pF, V_th={:.2} V, g_fs={:.0} S, Q_rr={:.0} nC\n\
         #  driver:     V_drive={:.1} V, R_source={:.2} Ω, R_g_ext={:.2} Ω\n\
         #  op:         V_in={:.1} V, V_out={:.1} V, I_L_init={:.2} A, loop_R={:.1} mΩ\n\
         #  sim:        dt={:.2} ps, duration={:.0} ns, expected f_ring≈{:.0} MHz",
        parasitics_path.display(),
        parasitics.power_loop_l_henry * 1e9,
        parasitics.sw_node_c_farad * 1e12,
        fet.c_iss * 1e12, fet.c_oss * 1e12, fet.c_rss * 1e12,
        fet.v_th, fet.g_fs, fet.q_rr * 1e9,
        driver.v_drive, driver.r_source, driver.r_g_ext,
        v_in, v_out, i_l, op.loop_r * 1e3,
        cfg.dt * 1e12, cfg.duration * 1e9,
        1.0 / (2.0 * std::f64::consts::PI
            * (parasitics.power_loop_l_henry
                * (parasitics.sw_node_c_farad + 2.0 * fet.c_oss)).sqrt()) / 1e6,
    );

    let edge = a.edge.as_deref().unwrap_or("turn-on");
    let w = match edge {
        "turn-on" => simulate_hs_turn_on(&parasitics, &fet, &fet, &driver, &op, &cfg),
        "turn-off" => simulate_hs_turn_off(&parasitics, &fet, &fet, &driver, &op, &cfg),
        _ => return Err(format!("--edge must be turn-on or turn-off; got {:?}", edge)),
    };

    println!();
    println!("summary:");
    println!("  n_samples              = {}", w.samples.len());
    if let Some(t) = w.t_v_th_crossed {
        println!("  V_GS crossed V_th at   = {:>8.3} ns", t * 1e9);
    }
    if let Some(t) = w.t_rr_entered {
        println!("  body diode RR entered  = {:>8.3} ns", t * 1e9);
    }
    if let Some(t) = w.t_rr_done {
        println!("  body diode RR done     = {:>8.3} ns", t * 1e9);
    }
    println!("  peak |I_RR|            = {:>8.3} A", w.i_rr_peak);
    println!("  V_SW overshoot vs V_in = {:>8.3} V", w.v_sw_overshoot);
    println!("  parasitic-LC ring est  = {:>8.3} MHz", w.f_ring_est / 1e6);
    println!("  peak |dV_SW/dt|        = {:>8.3} V/ns", w.dv_sw_dt_peak / 1e9);
    println!("  peak |dI_D/dt|         = {:>8.3} A/ns", w.di_d_dt_peak / 1e9);
    println!("  peak V_GS_LS           = {:>8.3} V  (V_th_LS = {:.2} V)", w.v_gs_ls_peak, fet.v_th);
    if w.ls_parasitic_turn_on {
        println!("  ⚠ LS-FET parasitic turn-on detected — peak I_D_LS = {:.3} A", w.i_d_ls_peak);
    } else {
        println!("  LS-FET parasitic turn-on:  none (peak V_GS_LS below V_th)");
    }
    if w.v_in_ripple_peak > 0.0 {
        println!("  V_in ripple (peak-peak)= {:>8.3} mV", w.v_in_ripple_peak * 1e3);
    }

    if let Some(path) = a.csv.as_ref() {
        write_csv(path, &w.samples).map_err(|e| format!("write CSV: {e}"))?;
        eprintln!("wrote CSV → {}", path.display());
    }
    if let Some(path) = a.json.as_ref() {
        write_json(path, &w)?;
        eprintln!("wrote JSON → {}", path.display());
    }
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => { eprintln!("scope-capture: error: {e}"); ExitCode::FAILURE }
    }
}
