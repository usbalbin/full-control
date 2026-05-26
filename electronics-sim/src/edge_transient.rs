//! Edge-transient simulator for one switching event of a half-bridge
//! buck stage. Resolves the nanosecond-scale physics that the existing
//! cycle-level [`crate::CurrentModeConverter`] simulator skips:
//!
//! - **Miller plateau** on V_GS_HS while V_DS_HS slews (C_rss feedback
//!   into the gate node holds V_GS at V_th + I_L/g_fs).
//! - **Body-diode reverse-recovery snap** of the LS-FET intrinsic
//!   diode: when the HS-FET takes over the freewheeling current, the
//!   LS diode's stored minority-carrier charge Q_rr is swept out as a
//!   negative current pulse before the diode snaps off.
//! - **Parasitic-LC ringing** between the PCB power-loop inductance
//!   `L_power` and the SW-node total capacitance (C_oss_HS + C_oss_LS
//!   + board's switch-node parasitic C to GND), damped by the loop R.
//!
//! ## Inputs
//!
//! - [`pdn_schema::BuckParasiticSet`] — board-extracted L's and the
//!   SW-node parasitic C, produced by
//!   `field_solver_cli buck-parasitics --out-pdn-schema-buck FILE`.
//! - [`FetModel`] — datasheet-driven nonlinear FET capacitances + on-
//!   resistance + transconductance + threshold + body-diode V_F and
//!   reverse-recovery charge Q_rr.
//! - [`DriverModel`] — gate driver source/sink resistance, drive rail
//!   voltage, dead-time.
//! - [`OperatingPoint`] — V_in, the inductor current at the turn-on
//!   moment, V_out (used as the "far side" boundary of the L_inductor
//!   on the µs timescale).
//!
//! ## Physics (single switching event — HS-FET turn-on)
//!
//! Just before t=0 the LS body diode is freewheeling: I_L > 0 flowing
//! from SW through the diode to GND, V_SW ≈ −V_F. At t=0 the driver
//! drives V_drive_HS from 0 → V_DRIVE.
//!
//! Phase A. V_GS_HS rises through V_th (RC charge of C_iss through
//! R_drive_HS + R_gate_HS, in series with the gate-loop inductance
//! L_gate_HS). FET still off; SW unchanged.
//!
//! Phase B. V_GS_HS at the Miller plateau V_plat = V_th + I_L / g_fs.
//! Drain current ramps from 0 to I_L. The body diode current drops
//! from I_L to 0; SW unchanged (still clamped at −V_F by the diode).
//!
//! Phase C. Body-diode reverse recovery. Once I_d hits zero the diode
//! keeps conducting in REVERSE (positive current from GND→SW direction
//! actually means SW→GND through the diode, since stored minority
//! carriers are still flushing). The HS-FET sources I_L + I_rr. We
//! integrate the reverse current; once the integral reaches Q_rr the
//! diode snaps off.
//!
//! Phase D. Parasitic LC ringing. With the diode off, C_oss_LS +
//! C_oss_HS + sw_node_c float; the inductor current keeps flowing into
//! the SW node, charging it up toward V_in. The PCB power-loop
//! inductance `L_power` resonates against C_sw_total, damped by the
//! loop DC resistance.
//!
//! Phase E. V_GS_HS rises past the Miller plateau into the FET's
//! triode region; V_DS_HS settles at I_L · R_dson_HS.
//!
//! ## Numerical scheme
//!
//! Explicit Euler with a fixed timestep `dt`. State variables:
//!
//! - `i_g_hs`  [A]   — gate-loop current
//! - `v_gs_hs` [V]   — HS-FET gate-source voltage
//! - `v_sw`    [V]   — switch node voltage
//! - `i_l`     [A]   — power-loop inductor current
//! - `q_rev`   [C]   — accumulated reverse charge in the LS body diode
//!
//! The diode state machine is encoded in [`DiodeState`]; transitions
//! latch on threshold crossings between sub-steps.
//!
//! ## Physics — now modelled (as of this revision)
//!
//! - **LS-FET body-diode re-conduction clamp** on negative-going V_SW
//!   ringing. Once the parasitic LC tries to swing V_SW below −V_F the
//!   LS body diode catches it again; this is the dominant asymmetric-
//!   damping mechanism on real boards.
//! - **C_gd / dV/dt parasitic turn-on of the LS (off) FET.** A fast
//!   rising V_SW couples through C_rss_LS into the LS gate node; if
//!   that gate voltage crosses V_th_LS the off-FET partially conducts,
//!   sourcing crow-bar current SW→GND — the #1 EMC/loss mechanism in
//!   high-dV/dt buck and GaN stages.
//! - **Source-degeneration (common-source) inductance** feedback. The
//!   FET package source inductance L_s appears in both the gate-loop
//!   return and the drain conduction path; `L_s · dI_D/dt` opposes
//!   the gate drive and slows fast switching. Schema fields
//!   `source_degen_l_henry_top/_bot` now flow into the gate-loop
//!   ODE.
//!
//! ## Physics — NOT modelled (yet)
//!
//! - **Mutual inductive coupling M_pg** between the power loop and
//!   gate loop. Real boards see 1-3 nH of mutual; di_power/dt then
//!   couples into the gate node alongside the L_s term. This needs a
//!   schema extension (`BuckParasiticSet` carries only L_self per
//!   role) plus an extractor change to emit the mutual matrix.
//! - **Body-diode soft-recovery factor S = t_b/t_a.** Real Q_rr has
//!   two phases — current falling from I_F to peak negative (t_a)
//!   and recovery from peak negative back to 0 (t_b). The current
//!   linear-decay model has S=1; most Si bodies are S=0.3-0.7 and
//!   abrupt bodies (S<<1) cause much higher dI/dt and ringing.
//! - **HS-FET turn-off** as a separate event. Today we simulate
//!   turn-on only; turn-off has its own physics (LS body diode
//!   commutating from off→on, no RR contribution) and needs a
//!   mirror simulator.
//! - **C_gd(V_DS) nonlinearity.** C_rss is treated as constant —
//!   real FETs have C_gd that drops 5-20× as V_DS rises. The Miller
//!   plateau timescale is therefore optimistic by ~5×.
//! - **Gate driver output-stage finite slew rate.** V_drive_HS is a
//!   perfect step today.
//! - **Channel-length modulation / sub-threshold slope.** I_D model
//!   is linear-gm in saturation, ohmic in triode.
//! - **Bootstrap-cap recharge dynamics.** HS gate rail is held
//!   perfectly at `driver.v_drive` for the duration of the event.
//! - **V_in node ripple** from `L_power · dI_D/dt`. The conducted-EMI
//!   path back to the LISN is opened only when V_in is allowed to
//!   ripple, requiring C_in modelling as a state node.
//! - **Assembly-level Y-capacitance** between the SW node and the
//!   chassis ground via the FET heatsink tab. Major common-mode EMI
//!   path; PCB extractor cannot see it.

use serde::{Deserialize, Serialize};

use pdn_schema::BuckParasiticSet;

/// Nonlinear FET model — datasheet-driven, half-bridge per half.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct FetModel {
    /// Input capacitance C_iss = C_gs + C_gd at V_GS=0, V_DS=test.
    /// Drives the dominant pole of the gate loop.
    pub c_iss: f64,
    /// Output capacitance C_oss = C_ds + C_gd at V_GS=0, V_DS=test.
    pub c_oss: f64,
    /// Reverse-transfer capacitance C_rss = C_gd. Sets the Miller
    /// plateau dV/dt = I_g / C_rss.
    pub c_rss: f64,
    /// Gate threshold voltage [V].
    pub v_th: f64,
    /// Forward transconductance g_fs at the rated drain current [S].
    /// Sets the Miller plateau V_plat = V_th + I_L / g_fs.
    pub g_fs: f64,
    /// Drain-source on-resistance [Ω].
    pub r_dson: f64,
    /// Body-diode forward voltage [V] (V_F at rated forward current).
    pub v_f_body: f64,
    /// Body-diode reverse-recovery charge [C].
    pub q_rr: f64,
    /// Internal gate-mesh resistance R_g_int (the chip's R_G [Ω]).
    pub r_g_int: f64,
}

impl FetModel {
    /// Silicon power FET preset based on the Infineon BSC0902NSI class
    /// (40 V, 1.4 mΩ Si MOSFET). Datasheet snapshot @ V_DS=20 V:
    /// C_iss=1100 pF, C_oss=180 pF, C_rss=10 pF, V_GS(th)=2.5 V,
    /// g_fs=130 S, R_DS(on)=1.4 mΩ, V_F=0.85 V, Q_rr=80 nC,
    /// R_G_internal=0.8 Ω.
    pub fn bsc0902nsi() -> Self {
        Self {
            c_iss: 1100e-12,
            c_oss: 180e-12,
            c_rss: 10e-12,
            v_th: 2.5,
            g_fs: 130.0,
            r_dson: 1.4e-3,
            v_f_body: 0.85,
            q_rr: 80e-9,
            r_g_int: 0.8,
        }
    }

    /// eGaN HEMT preset based on the EPC2034C class (200 V, 7 mΩ).
    /// Datasheet @ V_DS=100 V: C_iss=850 pF, C_oss=320 pF, C_rss=4 pF,
    /// V_GS(th)=1.4 V, g_fs=42 S, R_DS(on)=7 mΩ. GaN HEMTs have NO
    /// minority-carrier body diode — Q_rr=0. The "diode" is the
    /// reverse-conduction path through the same channel, with V_F set
    /// by V_GS(off) (typically 1.5-2 V). Approximated here.
    pub fn epc2034c() -> Self {
        Self {
            c_iss: 850e-12,
            c_oss: 320e-12,
            c_rss: 4e-12,
            v_th: 1.4,
            g_fs: 42.0,
            r_dson: 7e-3,
            v_f_body: 1.8,
            q_rr: 0.0,
            r_g_int: 0.4,
        }
    }
}

/// Gate driver model.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct DriverModel {
    /// Drive rail [V] (typically 5 V for GaN, 10-12 V for Si).
    pub v_drive: f64,
    /// Source (pull-up) resistance [Ω].
    pub r_source: f64,
    /// Sink (pull-down) resistance [Ω].
    pub r_sink: f64,
    /// External series gate resistor R_g_ext [Ω] (separate from the
    /// FET's intrinsic R_g_int).
    pub r_g_ext: f64,
}

impl DriverModel {
    /// Generic 5 V GaN driver preset: 1 Ω source, 0.5 Ω sink, no
    /// external R_g.
    pub fn generic_gan_5v() -> Self {
        Self { v_drive: 5.0, r_source: 1.0, r_sink: 0.5, r_g_ext: 0.0 }
    }

    /// Generic 10 V Si driver preset.
    pub fn generic_si_10v() -> Self {
        Self { v_drive: 10.0, r_source: 2.0, r_sink: 1.0, r_g_ext: 2.2 }
    }
}

/// DC operating point at the moment of the switching event.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct OperatingPoint {
    /// Input bus voltage [V].
    pub v_in: f64,
    /// Output capacitor voltage [V] (treated as constant on the
    /// nanosecond timescale of one edge).
    pub v_out: f64,
    /// Inductor current at the switching moment [A]. Positive =
    /// flowing from SW toward V_out (the normal freewheeling direction
    /// for a buck).
    pub i_l_init: f64,
    /// Lumped DC resistance of the power loop [Ω] — board copper +
    /// FET R_DS(on) at this operating point + capacitor ESR. Damps the
    /// LC ring; typically 5-50 mΩ on a real board. Defaulting from
    /// `BuckParasiticSet` is in [`SimConfig::default_loop_r`].
    pub loop_r: f64,
    /// Power-stage filter inductance [H] — the buck's *inductor*, not
    /// the parasitic loop L. Used only as the "far side" boundary of
    /// I_L on the µs timescale (di/dt = (V_SW − V_out) / L_inductor).
    pub l_inductor: f64,
}

/// Discrete LS body-diode state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DiodeState {
    /// Forward conducting (V_SW clamped to ≈ −V_F).
    Forward,
    /// Reverse-recovery: diode is conducting in the reverse direction
    /// while stored charge Q_rr is being swept out. V_SW is still
    /// near zero (carrier-level voltage is small).
    ReverseRecovery,
    /// Off — Q_rr depleted, diode is blocking. V_SW set by capacitor
    /// charge balance.
    Off,
}

/// One time-step of the recorded waveform.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct EdgeSample {
    pub t_s: f64,
    pub v_gs_hs: f64,
    pub v_gs_ls: f64,
    pub v_sw: f64,
    pub i_l: f64,
    pub i_g_hs: f64,
    pub i_d_hs: f64,
    pub i_d_ls: f64,
    pub i_diode_ls: f64,
    pub diode_state: DiodeState,
}

/// What the simulation produced.
#[derive(Debug, Clone)]
pub struct EdgeWaveforms {
    pub samples: Vec<EdgeSample>,
    /// Time at which V_GS_HS first crossed V_th [s], or `None` if it
    /// never did within the simulation window.
    pub t_v_th_crossed: Option<f64>,
    /// Time at which the LS body diode entered reverse recovery [s].
    pub t_rr_entered: Option<f64>,
    /// Time at which Q_rr was depleted and the diode snapped off [s].
    pub t_rr_done: Option<f64>,
    /// Peak negative diode current during RR [A] — the "snap"
    /// magnitude that drives EMI.
    pub i_rr_peak: f64,
    /// Peak V_SW overshoot above V_in [V] — sets switch-node stress.
    pub v_sw_overshoot: f64,
    /// Estimated ringing frequency [Hz] from L_power · C_sw_total.
    pub f_ring_est: f64,
    /// Peak V_GS_LS observed during the edge [V]. A value above
    /// `fet_ls.v_th` means the off-side FET was momentarily driven
    /// on by C_rss·dV_SW/dt — flag a shoot-through risk.
    pub v_gs_ls_peak: f64,
    /// `true` when v_gs_ls_peak exceeded `fet_ls.v_th` — i.e. the
    /// LS-FET was parasitically driven on at some point.
    pub ls_parasitic_turn_on: bool,
    /// Peak parasitic LS-FET drain current [A] (zero unless
    /// `ls_parasitic_turn_on`).
    pub i_d_ls_peak: f64,
    /// Peak `dV_SW/dt` magnitude [V/s] during the slew. Drives EMI
    /// and is the headline number for FCC/CISPR predictions.
    pub dv_sw_dt_peak: f64,
    /// Peak `dI_D_HS/dt` magnitude [A/s] during the slew. Drives the
    /// radiated H-field via the power-loop antenna area.
    pub di_d_dt_peak: f64,
}

/// Configuration knobs for the simulator.
#[derive(Debug, Clone, Copy)]
pub struct SimConfig {
    /// Time step [s]. Should be ≤ 1/(50·f_ring) and ≤ τ_gate/50.
    pub dt: f64,
    /// Total simulation window [s].
    pub duration: f64,
    /// Resampling stride for the recorded waveform (every Nth step).
    pub record_stride: usize,
}

impl SimConfig {
    /// Pick reasonable defaults from the parasitic set: dt = 1/(200·f_ring),
    /// duration = 10/f_ring (capture full ring envelope), no decimation.
    pub fn auto(parasitics: &BuckParasiticSet, fet: &FetModel) -> Self {
        let c_sw_total = parasitics.sw_node_c_farad + 2.0 * fet.c_oss;
        let f_ring = 1.0 / (2.0 * std::f64::consts::PI
            * (parasitics.power_loop_l_henry * c_sw_total).sqrt());
        let dt = (1.0 / (200.0 * f_ring)).min(50e-12); // floor at 50 ps
        let duration = (10.0 / f_ring).max(200e-9); // at least 200 ns
        Self { dt, duration, record_stride: 1 }
    }
}

/// Default loop resistance derived from the parasitic set's DC R
/// fields plus a hard floor (real boards always have a few mΩ of
/// damping). `BuckParasiticSet` doesn't currently carry per-loop DC
/// R, so we use a conservative 20 mΩ default until the extractor
/// surfaces it.
pub fn default_loop_r(_parasitics: &BuckParasiticSet) -> f64 {
    20e-3
}

/// Simulate one HS-FET turn-on event.
pub fn simulate_hs_turn_on(
    parasitics: &BuckParasiticSet,
    fet_hs: &FetModel,
    fet_ls: &FetModel,
    driver: &DriverModel,
    op: &OperatingPoint,
    cfg: &SimConfig,
) -> EdgeWaveforms {
    let l_gate_hs = parasitics.gate_loop_hs_l_henry.max(1e-12);
    let l_gate_ls = parasitics.gate_loop_ls_l_henry.max(1e-12);
    let l_power = parasitics.power_loop_l_henry.max(1e-12);
    let l_s_hs = parasitics.source_degen_l_henry_top.max(0.0);
    let l_s_ls = parasitics.source_degen_l_henry_bot.max(0.0);
    let c_sw_total = parasitics.sw_node_c_farad + fet_hs.c_oss + fet_ls.c_oss;

    let f_ring_est = 1.0 / (2.0 * std::f64::consts::PI
        * (l_power * c_sw_total).sqrt());

    // Gate-loop series R for each side.
    let r_gate_hs = driver.r_source + driver.r_g_ext + fet_hs.r_g_int;
    // The LS gate is being actively pulled LOW — driver in sink mode.
    let r_gate_ls = driver.r_sink + driver.r_g_ext + fet_ls.r_g_int;

    // ── Initial conditions ──────────────────────────────────────────
    let mut v_gs_hs = 0.0_f64;
    let mut v_gs_ls = 0.0_f64;       // LS driver pulling low
    let mut i_g_hs = 0.0_f64;
    let mut i_g_ls = 0.0_f64;
    let mut v_sw = -fet_ls.v_f_body; // diode clamping initially
    let mut i_l = op.i_l_init;
    let mut q_rev = 0.0_f64;
    let mut diode = DiodeState::Forward;

    // Track previous-step values for derivative-feedback terms.
    let mut prev_dvsw_dt = 0.0_f64;
    let mut prev_di_d_hs_dt = 0.0_f64;
    let mut prev_di_d_ls_dt = 0.0_f64;
    let mut prev_i_d_hs = 0.0_f64;
    let mut prev_i_d_ls = 0.0_f64;

    let mut t = 0.0_f64;
    let mut samples = Vec::with_capacity((cfg.duration / cfg.dt) as usize / cfg.record_stride);

    let mut t_v_th_crossed: Option<f64> = None;
    let mut t_rr_entered: Option<f64> = None;
    let mut t_rr_done: Option<f64> = None;
    let mut i_rr_peak = 0.0_f64;
    let mut v_sw_overshoot = 0.0_f64;
    let mut v_gs_ls_peak = 0.0_f64;
    let mut i_d_ls_peak = 0.0_f64;
    let mut dv_sw_dt_peak = 0.0_f64;
    let mut di_d_dt_peak = 0.0_f64;
    let mut ls_parasitic_turn_on = false;

    let mut step = 0usize;
    let n_steps = (cfg.duration / cfg.dt) as usize;

    // Helper: piecewise FET drain-current model (linear-gm in
    // saturation, ohmic in triode). I_D ≥ 0 by construction.
    let fet_i_d = |fet: &FetModel, v_gs: f64, v_ds: f64| -> f64 {
        let v_ov = (v_gs - fet.v_th).max(0.0);
        if v_ov <= 0.0 {
            0.0
        } else if v_ds > v_ov {
            fet.g_fs * v_ov
        } else {
            v_ds.max(0.0) / fet.r_dson
        }
    };

    while step < n_steps {
        // ── 1. FET drain currents ──────────────────────────────────
        let v_ds_hs = (op.v_in - v_sw).max(0.0);
        let v_ds_ls = v_sw.max(0.0);    // V_DS_LS = V_SW − GND
        let i_d_hs = fet_i_d(fet_hs, v_gs_hs, v_ds_hs);
        let i_d_ls = fet_i_d(fet_ls, v_gs_ls, v_ds_ls);

        if i_d_ls > i_d_ls_peak { i_d_ls_peak = i_d_ls; }

        // ── 2. Diode current (KCL at SW node) ──────────────────────
        let i_diode_ls = match diode {
            DiodeState::Forward => {
                // V_SW pinned at −V_F. Diode swallows whatever current
                // closes the freewheel balance (positive = SW→GND, the
                // freewheel direction).
                // KCL at SW: I_HS_in = I_L + I_LS_FET + I_diode_fwd
                // → I_diode = I_HS_in − I_L − I_LS_FET, but I_HS_in flows
                //   FROM V_in INTO SW (positive when FET on), and the
                //   diode current is from SW TO GND positive, so:
                //   I_diode = I_L + I_LS_FET − I_HS_in
                (i_l + i_d_ls - i_d_hs).max(0.0)
            }
            DiodeState::ReverseRecovery => {
                if fet_ls.q_rr > 0.0 {
                    let frac_remaining = (1.0 - q_rev / fet_ls.q_rr).max(0.0);
                    -op.i_l_init * frac_remaining
                } else {
                    0.0
                }
            }
            DiodeState::Off => 0.0,
        };

        // Track the peak (most-negative) diode current.
        if i_diode_ls < -i_rr_peak.abs() {
            i_rr_peak = i_diode_ls.abs();
        }

        // ── 3. State derivatives ───────────────────────────────────
        let v_drive_hs = driver.v_drive;
        let v_drive_ls = 0.0_f64;        // LS driver sinks (pull-down)

        // Gate loop ODE — includes the source-degeneration coupling
        // L_s · dI_D/dt term that opposes the gate drive on fast
        // di/dt edges. One-step lag on dI_D/dt to avoid an algebraic
        // loop.
        let di_g_hs_dt = (v_drive_hs - i_g_hs * r_gate_hs - v_gs_hs
                            - l_s_hs * prev_di_d_hs_dt) / l_gate_hs;
        let di_g_ls_dt = (v_drive_ls - i_g_ls * r_gate_ls - v_gs_ls
                            - l_s_ls * prev_di_d_ls_dt) / l_gate_ls;

        // V_GS dynamics — Miller feedback through C_rss.
        //   HS:  V_DS_HS = V_in − V_SW;  dV_DS_HS/dt = −dV_SW/dt.
        //   LS:  V_DS_LS = V_SW;          dV_DS_LS/dt = +dV_SW/dt.
        // Miller current INTO C_iss from C_rss is C_rss · dV_DS/dt;
        // when V_DS is FALLING (turn-on), this *robs* gate charge for
        // the on-side and *injects* charge into the off-side gate.
        let i_miller_hs = fet_hs.c_rss * (-prev_dvsw_dt);
        let i_miller_ls = fet_ls.c_rss * (prev_dvsw_dt);
        let dv_gs_hs_dt = (i_g_hs - i_miller_hs) / fet_hs.c_iss;
        let dv_gs_ls_dt = (i_g_ls + i_miller_ls) / fet_ls.c_iss;

        // SW node KCL → dV_SW/dt.
        //   Forward (V_SW pinned at −V_F): dV_SW/dt = 0.
        //   RR / Off: cap balances remaining current.
        //     KCL: I_HS_in (= i_d_hs)  =  I_L + I_LS_FET + I_diode + I_cap
        //     → I_cap = i_d_hs − i_l − i_d_ls − i_diode_ls
        //     dV_SW/dt = I_cap / C_sw_total
        let dv_sw_dt = match diode {
            DiodeState::Forward => 0.0,
            DiodeState::ReverseRecovery | DiodeState::Off => {
                (i_d_hs - i_l - i_d_ls - i_diode_ls) / c_sw_total
            }
        };

        if dv_sw_dt.abs() > dv_sw_dt_peak { dv_sw_dt_peak = dv_sw_dt.abs(); }

        // Inductor di/dt = (V_SW − V_out) / L_inductor + loop R drop.
        let di_l_dt = (v_sw - op.v_out - i_l * op.loop_r) / op.l_inductor;

        // ── 4. Euler step ──────────────────────────────────────────
        v_gs_hs += dv_gs_hs_dt * cfg.dt;
        v_gs_ls += dv_gs_ls_dt * cfg.dt;
        i_g_hs += di_g_hs_dt * cfg.dt;
        i_g_ls += di_g_ls_dt * cfg.dt;
        v_sw += dv_sw_dt * cfg.dt;
        i_l += di_l_dt * cfg.dt;

        // V_GS_LS is held ≥ 0 by the driver pull-down: don't let it
        // ring negative (the body diode of the driver clamps).
        if v_gs_ls < 0.0 { v_gs_ls = 0.0; }

        // Accumulate reverse charge (only when i_diode flows reverse).
        if i_diode_ls < 0.0 {
            q_rev += -i_diode_ls * cfg.dt;
        }

        // ── 5. Diode state transitions ─────────────────────────────
        match diode {
            DiodeState::Forward => {
                // Once forward current crosses zero, enter RR.
                if i_diode_ls <= 0.0 && q_rev == 0.0 && i_d_hs >= op.i_l_init * 0.95 {
                    diode = DiodeState::ReverseRecovery;
                    t_rr_entered = Some(t);
                }
            }
            DiodeState::ReverseRecovery => {
                if fet_ls.q_rr <= 0.0 || q_rev >= 0.99 * fet_ls.q_rr {
                    diode = DiodeState::Off;
                    t_rr_done = Some(t);
                }
            }
            DiodeState::Off => {
                // LS body-diode re-conduction clamp on negative-going
                // ring. If V_SW swings below −V_F the diode catches
                // it and we re-enter the Forward state. q_rev is
                // *not* reset — the carriers from the prior RR are
                // back, but for MVP we don't simulate a *second* RR
                // when V_SW eventually rises again.
                if v_sw < -fet_ls.v_f_body {
                    v_sw = -fet_ls.v_f_body;
                    diode = DiodeState::Forward;
                }
            }
        }

        // ── 6. Peaks + flags ──────────────────────────────────────
        if t_v_th_crossed.is_none() && v_gs_hs >= fet_hs.v_th {
            t_v_th_crossed = Some(t);
        }
        if v_sw > op.v_in + v_sw_overshoot {
            v_sw_overshoot = v_sw - op.v_in;
        }
        if v_gs_ls > v_gs_ls_peak { v_gs_ls_peak = v_gs_ls; }
        if v_gs_ls > fet_ls.v_th { ls_parasitic_turn_on = true; }
        let di_d_hs_dt = (i_d_hs - prev_i_d_hs) / cfg.dt;
        if di_d_hs_dt.abs() > di_d_dt_peak { di_d_dt_peak = di_d_hs_dt.abs(); }

        // ── 7. Recording ──────────────────────────────────────────
        if step % cfg.record_stride == 0 {
            samples.push(EdgeSample {
                t_s: t,
                v_gs_hs,
                v_gs_ls,
                v_sw,
                i_l,
                i_g_hs,
                i_d_hs,
                i_d_ls,
                i_diode_ls,
                diode_state: diode,
            });
        }

        // ── 8. Carry over for next step's derivative-feedback terms.
        prev_dvsw_dt = dv_sw_dt;
        prev_di_d_hs_dt = di_d_hs_dt;
        prev_di_d_ls_dt = (i_d_ls - prev_i_d_ls) / cfg.dt;
        prev_i_d_hs = i_d_hs;
        prev_i_d_ls = i_d_ls;
        t += cfg.dt;
        step += 1;
    }

    EdgeWaveforms {
        samples,
        t_v_th_crossed,
        t_rr_entered,
        t_rr_done,
        i_rr_peak,
        v_sw_overshoot,
        f_ring_est,
        v_gs_ls_peak,
        ls_parasitic_turn_on,
        i_d_ls_peak,
        dv_sw_dt_peak,
        di_d_dt_peak,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pdn_schema::SCHEMA_VERSION;

    fn test_parasitics() -> BuckParasiticSet {
        // PCB-7 numbers (see kicad_field_solver/docs/test_pcbs/PCB-7-...)
        BuckParasiticSet {
            schema_version: SCHEMA_VERSION,
            source: "edge_transient::tests".into(),
            board_hash: None,
            power_loop_l_henry: 13.6e-9,
            sw_node_c_farad: 0.149e-12,
            gate_loop_hs_l_henry: 4.22e-9,
            gate_loop_ls_l_henry: 19.4e-9,
            bootstrap_loop_l_henry: 3.5e-9,
            decoupling_loop_l_henry: 13.8e-9,
            sw_node_to_gnd_l_henry: 14.5e-9,
            snubber_loop_l_henry: None,
            source_degen_l_henry_top: 0.0,
            source_degen_l_henry_bot: 0.0,
        }
    }

    #[test]
    fn hs_turn_on_runs_to_completion() {
        let p = test_parasitics();
        let fet = FetModel::bsc0902nsi();
        let drv = DriverModel::generic_si_10v();
        let op = OperatingPoint {
            v_in: 12.0, v_out: 3.3, i_l_init: 3.0,
            loop_r: 20e-3, l_inductor: 4.7e-6,
        };
        let cfg = SimConfig::auto(&p, &fet);
        let w = simulate_hs_turn_on(&p, &fet, &fet, &drv, &op, &cfg);
        assert!(!w.samples.is_empty());
        assert!(w.f_ring_est > 1e6); // expect MHz-range ring with these L/C
    }

    #[test]
    fn v_gs_crosses_threshold_within_window() {
        let p = test_parasitics();
        let fet = FetModel::bsc0902nsi();
        let drv = DriverModel::generic_si_10v();
        let op = OperatingPoint {
            v_in: 12.0, v_out: 3.3, i_l_init: 3.0,
            loop_r: 20e-3, l_inductor: 4.7e-6,
        };
        let cfg = SimConfig::auto(&p, &fet);
        let w = simulate_hs_turn_on(&p, &fet, &fet, &drv, &op, &cfg);
        assert!(
            w.t_v_th_crossed.is_some(),
            "V_GS_HS never crossed V_th in {} samples",
            w.samples.len()
        );
    }

    #[test]
    fn gan_q_rr_zero_means_no_reverse_recovery_phase() {
        let p = test_parasitics();
        let fet = FetModel::epc2034c(); // Q_rr = 0
        let drv = DriverModel::generic_gan_5v();
        let op = OperatingPoint {
            v_in: 48.0, v_out: 12.0, i_l_init: 5.0,
            loop_r: 15e-3, l_inductor: 2.2e-6,
        };
        let cfg = SimConfig::auto(&p, &fet);
        let w = simulate_hs_turn_on(&p, &fet, &fet, &drv, &op, &cfg);
        // RR-entered may fire (transition from Forward→RR happens on
        // current zero-crossing) but RR-done should fire immediately
        // after, since Q_rr=0 means q_rev=0 already satisfies the
        // depletion condition.
        if let (Some(enter), Some(done)) = (w.t_rr_entered, w.t_rr_done) {
            assert!(
                done - enter <= cfg.dt * 2.0,
                "GaN RR phase should be ≤2 timesteps; got {} ns",
                (done - enter) * 1e9
            );
        }
    }
}
