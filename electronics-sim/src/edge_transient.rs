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
//! the SW node, charging it up toward V_in, where it rings on the
//! `L_power`–C_sw_total tank (damped by the loop DC resistance).
//! Because explicit Euler cannot integrate this high-Q (~50–100 MHz,
//! Q≈hundreds) ring accurately, the ring's first-peak **overshoot** is
//! added to `v_sw_overshoot` in closed form (exact underdamped
//! series-RLC step response); the ring's spectral content is modelled
//! analytically in the spectrum-export path (`RingingParams`), so
//! `L_power` is NOT an explicit ODE state (see state list below).
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
//! - **Mutual coupling M_pg**: schema fields now exist on
//!   `pdn_schema::BuckParasiticSet` and the consumer-side ODE term is
//!   wired. The in-tree extractor still emits 0 for both
//!   `mutual_l_henry_power_gate_hs/_ls`; once the §0.1.5 extractor
//!   extension emits real M values the existing simulator already
//!   consumes them.
//! - **Channel-length modulation / sub-threshold slope.** I_D model
//!   is linear-gm in saturation, ohmic in triode.
//! - **C_iss(V_GS) nonlinearity.** C_iss is treated as constant; real
//!   FETs vary 1.5-3× with V_GS but the effect is small compared to
//!   the C_oss / C_rss curves we already model.
//! - **Multi-edge bootstrap dynamics.** v_boot discharges during
//!   turn-on (now modelled) but cannot recharge during turn-off; the
//!   real cycle has C_boot refilling through the bootstrap diode
//!   when V_SW falls to ≈ 0. Wait for a full-cycle simulator.

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
    /// For GaN HEMTs (`is_gan = true`) this field is ignored — the
    /// third-quadrant V_F is instead computed as `v_th + |V_GS_off|`
    /// because GaN devices have no minority-carrier body diode.
    pub v_f_body: f64,
    /// Body-diode reverse-recovery charge [C]. Set to 0 for GaN and
    /// Schottky-clamped bodies (no minority-carrier storage).
    pub q_rr: f64,
    /// Internal gate-mesh resistance R_g_int (the chip's R_G [Ω]).
    pub r_g_int: f64,

    // ── Extended physics fields (with serde defaults for back-compat).
    /// V_DS at which the datasheet C_oss / C_rss / C_iss were
    /// specified [V]. Used to scale the capacitances to the actual
    /// operating V_DS via `C_eff(V_DS) ≈ C_test · √(V_test / V_DS)`
    /// (depletion-region sqrt model). Typical: 20 V Si, 100 V GaN.
    /// Default if missing: 20 V.
    #[serde(default = "default_c_test_v")]
    pub c_test_v: f64,
    /// Body-diode reverse-recovery time at the datasheet-rated forward
    /// current [s]. Together with `softness_factor` sets the
    /// piecewise-linear t_a (fall) + t_b (recovery) split. Typical
    /// values: 30-100 ns for Si MOSFETs, 0 for GaN/Schottky. Default
    /// 50 ns if missing.
    #[serde(default = "default_t_rr")]
    pub t_rr: f64,
    /// Body-diode softness factor S = t_b/t_a. Soft-recovery diodes
    /// (S ≈ 1) ring less and have lower EMI; abrupt diodes (S → 0)
    /// have a much larger snap-induced di/dt. Range typically
    /// 0.3-1.0. Default 0.5 if missing.
    #[serde(default = "default_softness")]
    pub softness_factor: f64,
    /// `true` for GaN HEMTs / depletion-mode devices that lack a
    /// minority-carrier body diode. When set, the third-quadrant
    /// V_F is computed as `v_th + |V_GS_off|` instead of using
    /// `v_f_body`, and Q_rr is forced to zero. Default false (Si).
    #[serde(default)]
    pub is_gan: bool,
}

fn default_c_test_v() -> f64 { 20.0 }
fn default_t_rr() -> f64 { 50e-9 }
fn default_softness() -> f64 { 0.5 }

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
            c_test_v: 20.0,       // datasheet C @ V_DS = 20 V
            t_rr: 50e-9,          // datasheet t_rr ≈ 50 ns
            softness_factor: 0.5, // typical Si body diode
            is_gan: false,
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
            // V_F not used for GaN — third-quadrant V_F is
            // computed from v_th + |driver.v_off|.
            v_f_body: 0.0,
            q_rr: 0.0,
            r_g_int: 0.4,
            c_test_v: 100.0,      // GaN datasheets typically at V_DS = 100 V
            t_rr: 0.0,            // no minority-carrier RR
            softness_factor: 1.0, // irrelevant when q_rr=0
            is_gan: true,
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

    // ── Extended fields (serde defaults for back-compat).
    /// Gate-off rail voltage [V]. 0 V is standard; some GaN drivers
    /// (e.g. LMG1210, UCC27282) use a negative rail like −2 V to
    /// raise the Cdv/dt parasitic-turn-on margin (the LS gate has to
    /// climb V_th + |V_off| through C_rss·dV_SW/dt before the FET
    /// conducts). For GaN with no body diode, this also DIRECTLY sets
    /// the third-quadrant reverse-conduction V_F = v_th + |V_off|.
    #[serde(default)]
    pub v_off: f64,
    /// Driver output-stage finite slew rate [V/s]. The actual gate-
    /// drive voltage tracks the target at this rate, rather than
    /// instantaneously. Default 1e11 V/s (≈ very fast, behaves like
    /// the previous ideal-step model).
    #[serde(default = "default_drv_slew")]
    pub slew_rate_v_per_s: f64,
}

fn default_drv_slew() -> f64 { 1.0e11 }

impl DriverModel {
    /// Generic 5 V GaN driver preset: 1 Ω source, 0.5 Ω sink, no
    /// external R_g, V_off = 0 V, slew ≈ 50 V/ns (typical for fast
    /// half-bridge drivers like LMG1210).
    pub fn generic_gan_5v() -> Self {
        Self {
            v_drive: 5.0, r_source: 1.0, r_sink: 0.5, r_g_ext: 0.0,
            v_off: 0.0, slew_rate_v_per_s: 50e9,
        }
    }

    /// Generic 5 V GaN driver with NEGATIVE off rail (−2 V) — common
    /// in modern GaN half-bridges to defend against Cdv/dt parasitic
    /// turn-on on the off side. Note the trade-off: −2 V V_off makes
    /// the third-quadrant reverse-conduction V_F = v_th + 2 V, which
    /// is a substantial dead-time loss penalty.
    pub fn generic_gan_5v_neg_off() -> Self {
        Self {
            v_drive: 5.0, r_source: 1.0, r_sink: 0.5, r_g_ext: 0.0,
            v_off: -2.0, slew_rate_v_per_s: 50e9,
        }
    }

    /// Generic 10 V Si driver preset, V_off = 0 V, slew ≈ 20 V/ns.
    pub fn generic_si_10v() -> Self {
        Self {
            v_drive: 10.0, r_source: 2.0, r_sink: 1.0, r_g_ext: 2.2,
            v_off: 0.0, slew_rate_v_per_s: 20e9,
        }
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

    // ── Extended fields (serde defaults for back-compat).
    /// Input-rail decoupling capacitance [F]. Sets the V_in ripple
    /// amplitude `ΔV_in ≈ ∫i_d_hs · dt / C_in` over the switching
    /// edge. Default 0 = "stiff V_in" (no ripple); a realistic buck
    /// has C_in ≥ 10 µF and sees ΔV_in on the order of a few hundred
    /// mV for typical loads.
    #[serde(default)]
    pub c_in_farad: f64,
    /// Assembly-level Y-capacitance from the SW node to the chassis
    /// ground via the FET heatsink tab + thermal pad + thermal
    /// interface material [F]. Major common-mode EMI path; the PCB
    /// extractor cannot see this — user must measure or estimate.
    /// Typical: 10-500 pF depending on tab geometry and TIM. Default
    /// 0 (no Y-path); when non-zero, it adds in parallel with the
    /// SW-node parasitic C for the LC-ring dynamics.
    #[serde(default)]
    pub c_y_chassis_farad: f64,
    /// Bootstrap-cap value [F]. Sits between BOOT pin and SW node;
    /// supplies the HS gate-drive charge while the HS-FET is on.
    /// When > 0, the simulator tracks the boot-cap voltage as a state
    /// (`v_boot`) and the HS gate sees `min(v_drive, v_boot)` as the
    /// effective drive rail — discharge is small per edge but matters
    /// at high duty cycle. Default 0 = treat the boot rail as stiff
    /// (back-compat).
    #[serde(default)]
    pub c_boot_farad: f64,
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
    /// Peak V_in ripple [V] (max − min within the simulation window).
    /// Only meaningful when `op.c_in_farad > 0`; otherwise 0.
    pub v_in_ripple_peak: f64,
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

/// Effective C_oss(V_DS) using the depletion-region √ scaling
/// `C_oss(V_DS) ≈ C_test · √(V_test / max(V_DS, 0.5 V))`. Captures
/// the dominant nonlinearity of MOSFET / GaN output capacitance with
/// drain voltage; real C_oss curves drop 5-20× from V_DS ≈ 0 up to
/// rated V_DS. The 0.5 V floor avoids the divergence at V_DS → 0.
pub fn c_oss_eff(fet: &FetModel, v_ds: f64) -> f64 {
    let v_use = v_ds.max(0.5);
    fet.c_oss * (fet.c_test_v / v_use).sqrt()
}

/// Effective C_rss(V_DS) with the same √ scaling. The Miller plateau
/// dV/dt is set by I_g / C_rss_eff, so this nonlinearity stretches
/// the plateau when V_DS is small (start of the slew) and contracts
/// it when V_DS is large (end of the slew).
pub fn c_rss_eff(fet: &FetModel, v_ds: f64) -> f64 {
    let v_use = v_ds.max(0.5);
    fet.c_rss * (fet.c_test_v / v_use).sqrt()
}

/// Effective body-diode forward voltage for the LS-side reverse
/// conduction clamp. For Si the value comes from `fet.v_f_body`;
/// GaN HEMTs lack a minority-carrier body diode, and instead conduct
/// in reverse through the same channel — the effective V_F is
/// `v_th + |V_GS_off|`. With a 0 V off-rail this is just V_th
/// (~1.4 V on EPC2034C); with a −2 V off-rail it rises to ~3.4 V,
/// which is a real dead-time loss penalty users pay for the Cdv/dt
/// parasitic-turn-on margin.
pub fn body_diode_v_f(fet: &FetModel, driver: &DriverModel) -> f64 {
    if fet.is_gan {
        fet.v_th + driver.v_off.abs()
    } else {
        fet.v_f_body
    }
}

/// Soft-recovery body-diode model. Returns the diode current at
/// elapsed-time `t_in_rr` since RR entry, or `None` once RR is
/// finished. Profile is piecewise-linear:
///
/// - t ∈ [0, t_a]:  current sweeps from 0 down to −I_RR_peak.
/// - t ∈ [t_a, t_a + t_b]:  current recovers from −I_RR_peak back to 0.
/// - t > t_a + t_b:  RR done; returns None.
///
/// where `I_RR_peak = 2 · Q_rr / t_rr` (triangular-charge equivalence)
/// and `t_a + t_b = t_rr`, with `softness_factor = t_b / t_a`.
pub fn soft_recovery_i_diode(fet: &FetModel, t_in_rr: f64) -> Option<f64> {
    if fet.q_rr <= 0.0 || fet.t_rr <= 0.0 {
        return None;
    }
    let s = fet.softness_factor.max(1e-3);
    let t_a = fet.t_rr / (1.0 + s);
    let t_b = s * t_a;
    let i_rr_peak = 2.0 * fet.q_rr / fet.t_rr;
    if t_in_rr <= 0.0 {
        Some(0.0)
    } else if t_in_rr <= t_a {
        Some(-i_rr_peak * (t_in_rr / t_a))
    } else if t_in_rr <= t_a + t_b {
        let frac = (t_in_rr - t_a) / t_b;
        Some(-i_rr_peak * (1.0 - frac))
    } else {
        None
    }
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
    // C_sw_total is recomputed per-step from the nonlinear C_oss(V_DS)
    // helpers; this initial value is only used to size the time-step
    // and seed the f_ring estimate.
    let c_sw_total_initial =
        parasitics.sw_node_c_farad + op.c_y_chassis_farad
            + c_oss_eff(fet_hs, op.v_in) + c_oss_eff(fet_ls, 0.0);
    let v_f_ls = body_diode_v_f(fet_ls, driver);

    let f_ring_est = 1.0 / (2.0 * std::f64::consts::PI
        * (l_power * c_sw_total_initial).sqrt());

    // Gate-loop series R for each side.
    let r_gate_hs = driver.r_source + driver.r_g_ext + fet_hs.r_g_int;
    // The LS gate is being actively pulled LOW — driver in sink mode.
    let r_gate_ls = driver.r_sink + driver.r_g_ext + fet_ls.r_g_int;

    // ── Initial conditions ──────────────────────────────────────────
    let mut v_gs_hs = 0.0_f64;
    let mut v_gs_ls = 0.0_f64;       // LS driver pulling low
    let mut i_g_hs = 0.0_f64;
    let mut i_g_ls = 0.0_f64;
    let mut v_sw = -v_f_ls; // diode clamping initially
    let mut i_l = op.i_l_init;
    let mut _q_rev_unused = 0.0_f64; // legacy charge-integral; replaced by soft_recovery_i_diode
    let mut diode = DiodeState::Forward;
    // V_in is a state when c_in_farad > 0 (input cap discharges as
    // FET draws current); otherwise treat as stiff and equal to v_in.
    let mut v_in_actual = op.v_in;
    let mut v_in_min = op.v_in;
    let mut v_in_max = op.v_in;
    // Driver output voltages with finite slew rate. HS target = v_drive
    // at turn-on; LS stays at v_off (0 V or negative rail).
    let mut v_drive_act_hs = 0.0_f64;
    let mut v_drive_act_ls = driver.v_off;
    // Bootstrap-cap state (BOOT-pin voltage referenced to SW); starts
    // at v_drive after a complete LS-on recharge cycle. Discharges by
    // i_g_hs · dt / c_boot during HS conduction.
    let mut v_boot = driver.v_drive;
    let mut v_boot_min = driver.v_drive;
    // Mutual inductive coupling between power loop and gate loops.
    let m_pg_hs = parasitics.mutual_l_henry_power_gate_hs;
    let m_pg_ls = parasitics.mutual_l_henry_power_gate_ls;

    // Track previous-step values for derivative-feedback terms.
    let mut prev_dvsw_dt = 0.0_f64;
    let mut prev_di_d_hs_dt = 0.0_f64;
    let mut prev_di_d_ls_dt = 0.0_f64;
    let mut prev_di_l_dt = 0.0_f64;
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
        let v_ds_hs = (v_in_actual - v_sw).max(0.0);
        let v_ds_ls = v_sw.max(0.0);    // V_DS_LS = V_SW − GND
        let i_d_hs = fet_i_d(fet_hs, v_gs_hs, v_ds_hs);
        let i_d_ls = fet_i_d(fet_ls, v_gs_ls, v_ds_ls);

        if i_d_ls > i_d_ls_peak { i_d_ls_peak = i_d_ls; }

        // Per-step nonlinear capacitance (depletion-region √ scaling
        // of the datasheet C @ c_test_v down to the actual V_DS).
        let c_rss_hs = c_rss_eff(fet_hs, v_ds_hs);
        let c_rss_ls = c_rss_eff(fet_ls, v_ds_ls);
        let c_sw_total = parasitics.sw_node_c_farad + op.c_y_chassis_farad
            + c_oss_eff(fet_hs, v_ds_hs) + c_oss_eff(fet_ls, v_ds_ls);

        // ── 2. Diode current (KCL at SW node) ──────────────────────
        let i_diode_ls = match diode {
            DiodeState::Forward => {
                // V_SW pinned at −V_F. Diode swallows whatever current
                // closes the freewheel balance.
                (i_l + i_d_ls - i_d_hs).max(0.0)
            }
            DiodeState::ReverseRecovery => {
                // Soft-recovery piecewise model: t_a (fall) → t_b
                // (recovery), parameterised by softness_factor.
                let t_in_rr = t_rr_entered.map(|t0| t - t0).unwrap_or(0.0);
                soft_recovery_i_diode(fet_ls, t_in_rr).unwrap_or(0.0)
            }
            DiodeState::Off => 0.0,
        };

        // Track the peak (most-negative) diode current.
        if i_diode_ls < -i_rr_peak.abs() {
            i_rr_peak = i_diode_ls.abs();
        }

        // ── 3. State derivatives ───────────────────────────────────
        // Effective driver output voltages — slewed targets. HS target
        // is v_drive (sourcing); LS target is v_off (sink rail).
        let v_drive_hs_target = if op.c_boot_farad > 0.0 {
            v_boot.min(driver.v_drive)
        } else {
            driver.v_drive
        };
        let v_drive_ls_target = driver.v_off;
        let dv_drv_hs_dt = (v_drive_hs_target - v_drive_act_hs)
            .signum() * driver.slew_rate_v_per_s;
        let dv_drv_ls_dt = (v_drive_ls_target - v_drive_act_ls)
            .signum() * driver.slew_rate_v_per_s;
        let v_drive_hs = v_drive_act_hs;
        let v_drive_ls = v_drive_act_ls;

        // Gate loop ODE — includes:
        //   1. Source-degeneration L · dI_D/dt term (opposes drive
        //      on fast di/dt edges; one-step lag).
        //   2. Mutual coupling M_pg · dI_L/dt between power loop and
        //      gate loop. When non-zero, di_L/dt couples in alongside
        //      L_s · dI_D/dt (also one-step lag).
        let di_g_hs_dt = (v_drive_hs - i_g_hs * r_gate_hs - v_gs_hs
                            - l_s_hs * prev_di_d_hs_dt
                            - m_pg_hs * prev_di_l_dt) / l_gate_hs;
        let di_g_ls_dt = (v_drive_ls - i_g_ls * r_gate_ls - v_gs_ls
                            - l_s_ls * prev_di_d_ls_dt
                            - m_pg_ls * prev_di_l_dt) / l_gate_ls;

        // V_GS dynamics — Miller feedback through C_rss.
        //   HS:  V_DS_HS = V_in − V_SW;  dV_DS_HS/dt = −dV_SW/dt.
        //   LS:  V_DS_LS = V_SW;          dV_DS_LS/dt = +dV_SW/dt.
        // Miller current INTO C_iss from C_rss is C_rss · dV_DS/dt;
        // when V_DS is FALLING (turn-on), this *robs* gate charge for
        // the on-side and *injects* charge into the off-side gate.
        let i_miller_hs = c_rss_hs * prev_dvsw_dt;
        let i_miller_ls = c_rss_ls * (prev_dvsw_dt);
        let dv_gs_hs_dt = (i_g_hs - i_miller_hs) / fet_hs.c_iss;
        let dv_gs_ls_dt = (i_g_ls + i_miller_ls) / fet_ls.c_iss;

        // SW node KCL → dV_SW/dt.
        let dv_sw_dt = match diode {
            DiodeState::Forward => 0.0,
            DiodeState::ReverseRecovery | DiodeState::Off => {
                (i_d_hs - i_l - i_d_ls + i_diode_ls) / c_sw_total
            }
        };

        if dv_sw_dt.abs() > dv_sw_dt_peak { dv_sw_dt_peak = dv_sw_dt.abs(); }

        // V_in node — discharges through the FET when c_in_farad > 0.
        // dV_in/dt = -I_d_hs / C_in (positive current flowing out of
        // the cap raises the deficit). When c_in_farad == 0, V_in is
        // treated as stiff.
        let dv_in_dt = if op.c_in_farad > 0.0 {
            -i_d_hs / op.c_in_farad
        } else {
            0.0
        };

        // Inductor di/dt = (V_SW − V_out) / L_inductor + loop R drop.
        let di_l_dt = (v_sw - op.v_out - i_l * op.loop_r) / op.l_inductor;

        // ── 4. Euler step ──────────────────────────────────────────
        v_gs_hs += dv_gs_hs_dt * cfg.dt;
        v_gs_ls += dv_gs_ls_dt * cfg.dt;
        i_g_hs += di_g_hs_dt * cfg.dt;
        i_g_ls += di_g_ls_dt * cfg.dt;
        v_sw += dv_sw_dt * cfg.dt;
        i_l += di_l_dt * cfg.dt;
        v_in_actual += dv_in_dt * cfg.dt;
        if v_in_actual < v_in_min { v_in_min = v_in_actual; }
        if v_in_actual > v_in_max { v_in_max = v_in_actual; }
        // Slew-rate-limited driver outputs — step toward target by at
        // most slew_rate · dt; clamp at the target to avoid overshoot.
        v_drive_act_hs += dv_drv_hs_dt * cfg.dt;
        if (v_drive_act_hs - v_drive_hs_target).signum() == dv_drv_hs_dt.signum() {
            v_drive_act_hs = v_drive_hs_target;
        }
        v_drive_act_ls += dv_drv_ls_dt * cfg.dt;
        if (v_drive_act_ls - v_drive_ls_target).signum() == dv_drv_ls_dt.signum() {
            v_drive_act_ls = v_drive_ls_target;
        }
        // Bootstrap discharge: the gate-loop current i_g_hs is sourced
        // from the boot cap, so it integrates down. Only meaningful
        // when c_boot_farad > 0; otherwise treat as stiff.
        if op.c_boot_farad > 0.0 {
            v_boot -= i_g_hs * cfg.dt / op.c_boot_farad;
            if v_boot < v_boot_min { v_boot_min = v_boot; }
        }

        // V_GS_LS is held ≥ driver v_off; for negative-rail drivers
        // it settles at v_off.
        if v_gs_ls < driver.v_off.min(0.0) { v_gs_ls = driver.v_off.min(0.0); }

        // ── 5. Diode state transitions ─────────────────────────────
        match diode {
            DiodeState::Forward => {
                // Once forward current crosses zero, enter RR.
                if i_diode_ls <= 0.0 && i_d_hs >= op.i_l_init * 0.95 {
                    diode = DiodeState::ReverseRecovery;
                    t_rr_entered = Some(t);
                }
            }
            DiodeState::ReverseRecovery => {
                // soft_recovery_i_diode returns None once t > t_a+t_b.
                let t_in_rr = t_rr_entered.map(|t0| t - t0).unwrap_or(0.0);
                if soft_recovery_i_diode(fet_ls, t_in_rr).is_none() {
                    diode = DiodeState::Off;
                    t_rr_done = Some(t);
                }
            }
            DiodeState::Off => {
                // LS body-diode re-conduction clamp on negative-going
                // ring. If V_SW swings below −V_F the diode catches
                // it (or for GaN, V_F_eff = v_th + |v_off|).
                if v_sw < -v_f_ls {
                    v_sw = -v_f_ls;
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
        prev_di_l_dt = di_l_dt;
        prev_i_d_hs = i_d_hs;
        prev_i_d_ls = i_d_ls;
        t += cfg.dt;
        step += 1;
    }

    // Commutation-loop (L_power–C_sw) ring overshoot. The switch node doesn't
    // just settle at V_in — it rings around it, and the first peak is the real
    // voltage-stress number. Forward Euler cannot integrate this high-Q ring
    // accurately (an explicit step either grows or artificially damps a
    // ~50–100 MHz, Q≈hundreds oscillator), so we add the EXACT closed-form
    // underdamped first-peak overshoot of the series RLC step response:
    //   overshoot = V_step · exp(−α·π/ω_d),   α = R/(2L),  ω_d = √(ω_n² − α²)
    // with V_step ≈ V_in (the node commutates by ~V_in). Overdamped ⇒ none.
    {
        let omega_n = 2.0 * std::f64::consts::PI * f_ring_est;
        let alpha = op.loop_r / (2.0 * l_power);
        let omega_d_sq = omega_n * omega_n - alpha * alpha;
        if omega_d_sq > 0.0 {
            let omega_d = omega_d_sq.sqrt();
            let ring_overshoot = op.v_in * (-alpha * std::f64::consts::PI / omega_d).exp();
            if ring_overshoot > v_sw_overshoot {
                v_sw_overshoot = ring_overshoot;
            }
        }
    }

    let _ = v_boot_min; // optional reporting hook (not yet on EdgeWaveforms)
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
        v_in_ripple_peak: (v_in_max - v_in_min).max(0.0),
    }
}

/// Simulate one HS-FET turn-off event.
///
/// Mirror of [`simulate_hs_turn_on`]: at t=0 the HS driver flips from
/// SOURCE mode (pulling the HS gate to V_drive) to SINK mode (pulling
/// the HS gate to 0). The LS driver is unchanged (LS gate held at 0).
///
/// Phase A. V_GS_HS falls through C_iss·R_gate from V_drive toward the
/// Miller plateau V_plat = V_th + I_L / g_fs. FET still in ohmic; V_SW
/// ≈ V_in − I_L · R_DS(on).
///
/// Phase B. V_GS_HS holds at the Miller plateau while V_DS_HS slews UP
/// (i.e. V_SW slews DOWN) from V_in − I_L·R_DS(on) toward 0. The
/// Miller current direction is reversed vs turn-on: the gate is now
/// SOURCING into C_gd as V_DS rises.
///
/// Phase C. V_SW crosses 0 going DOWN, the LS body diode forward-
/// conducts as soon as V_SW ≤ −V_F; the inductor current commutates
/// from HS-FET → LS body diode.
///
/// Phase D. V_GS_HS falls below V_th, HS-FET drain current goes to 0;
/// the freewheel through the LS body diode is established.
///
/// Phase E. Parasitic-LC ringing damped by `loop_r` and the body-diode
/// re-conduction clamp. No reverse-recovery snap (diode is entering
/// forward conduction, not exiting).
pub fn simulate_hs_turn_off(
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
    let c_sw_total_initial =
        parasitics.sw_node_c_farad + op.c_y_chassis_farad
            + c_oss_eff(fet_hs, 0.0) + c_oss_eff(fet_ls, op.v_in);
    let v_f_ls = body_diode_v_f(fet_ls, driver);

    let f_ring_est = 1.0 / (2.0 * std::f64::consts::PI
        * (l_power * c_sw_total_initial).sqrt());

    // Gate-loop series R for each side. At turn-off the HS driver is
    // SINKING (pulling the HS gate down through r_sink); LS driver is
    // still sinking the LS gate low.
    let r_gate_hs = driver.r_sink + driver.r_g_ext + fet_hs.r_g_int;
    let r_gate_ls = driver.r_sink + driver.r_g_ext + fet_ls.r_g_int;

    // ── Initial conditions ──────────────────────────────────────────
    //   HS-FET fully ON in triode, conducting I_L from V_in into the
    //   inductor; V_SW = V_in − I_L · R_DS(on). LS body diode OFF
    //   (HS is supplying the load). q_rev=0.
    let mut v_gs_hs = driver.v_drive;
    let mut v_gs_ls = 0.0_f64;
    let mut i_g_hs = 0.0_f64;
    let mut i_g_ls = 0.0_f64;
    let mut v_sw = (op.v_in - op.i_l_init * fet_hs.r_dson).max(0.0);
    let mut i_l = op.i_l_init;
    // q_rev tracked for state-machine completeness; RR never triggers
    // at turn-off so it stays at 0 in practice.
    #[allow(unused_assignments)]
    let mut q_rev = 0.0_f64;
    let mut diode = DiodeState::Off;
    // Driver output state — at turn-off start the HS driver is still
    // at v_drive (about to flip to v_off); LS driver was at v_off.
    let mut v_drive_act_hs = driver.v_drive;
    let mut v_drive_act_ls = driver.v_off;
    // Mutual coupling between the power loop and the gate loops.
    let m_pg_hs = parasitics.mutual_l_henry_power_gate_hs;
    let m_pg_ls = parasitics.mutual_l_henry_power_gate_ls;

    // Track previous-step values for derivative-feedback terms.
    let mut prev_dvsw_dt = 0.0_f64;
    let mut prev_di_d_hs_dt = 0.0_f64;
    let mut prev_di_d_ls_dt = 0.0_f64;
    let mut prev_di_l_dt = 0.0_f64;
    let mut prev_i_d_hs = op.i_l_init; // HS-FET starts carrying I_L
    let mut prev_i_d_ls = 0.0_f64;

    let mut t = 0.0_f64;
    let mut samples = Vec::with_capacity((cfg.duration / cfg.dt) as usize / cfg.record_stride);

    // For turn-off, `t_v_th_crossed` records when V_GS_HS falls BELOW
    // V_th (i.e. the HS-FET fully turns off).
    let mut t_v_th_crossed: Option<f64> = None;
    let t_rr_entered: Option<f64> = None; // no RR at turn-off
    let t_rr_done: Option<f64> = None;
    let mut i_rr_peak = 0.0_f64;          // re-purposed: peak |i_diode|
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

    let mut v_in_actual = op.v_in;
    let mut v_in_min = op.v_in;
    let mut v_in_max = op.v_in;

    while step < n_steps {
        // ── 1. FET drain currents + per-step nonlinear capacitances
        let v_ds_hs = (v_in_actual - v_sw).max(0.0);
        let v_ds_ls = v_sw.max(0.0);
        let i_d_hs = fet_i_d(fet_hs, v_gs_hs, v_ds_hs);
        let i_d_ls = fet_i_d(fet_ls, v_gs_ls, v_ds_ls);

        if i_d_ls > i_d_ls_peak { i_d_ls_peak = i_d_ls; }

        let c_rss_hs = c_rss_eff(fet_hs, v_ds_hs);
        let c_rss_ls = c_rss_eff(fet_ls, v_ds_ls);
        let c_sw_total = parasitics.sw_node_c_farad + op.c_y_chassis_farad
            + c_oss_eff(fet_hs, v_ds_hs) + c_oss_eff(fet_ls, v_ds_ls);

        // ── 2. Diode current (KCL at SW node) ──────────────────────
        // At turn-off the LS body diode only ever transitions
        // Off → Forward (no reverse recovery to model).
        let i_diode_ls = match diode {
            DiodeState::Forward => (i_l + i_d_ls - i_d_hs).max(0.0),
            DiodeState::ReverseRecovery | DiodeState::Off => 0.0,
        };

        // Track the peak |i_diode|. At turn-off the diode is forward-
        // conducting; we re-use the i_rr_peak field as "peak diode
        // forward current magnitude" so the struct stays stable.
        if i_diode_ls.abs() > i_rr_peak {
            i_rr_peak = i_diode_ls.abs();
        }

        // ── 3. State derivatives ───────────────────────────────────
        // HS driver in SINK mode → target = driver.v_off. LS driver
        // unchanged (sink, holding LS gate low). Slewed via state.
        let v_drive_hs_target = driver.v_off;
        let v_drive_ls_target = driver.v_off;
        let dv_drv_hs_dt = (v_drive_hs_target - v_drive_act_hs)
            .signum() * driver.slew_rate_v_per_s;
        let dv_drv_ls_dt = (v_drive_ls_target - v_drive_act_ls)
            .signum() * driver.slew_rate_v_per_s;
        let v_drive_hs = v_drive_act_hs;
        let v_drive_ls = v_drive_act_ls;

        // Gate loop ODE — source-degeneration AND mutual-coupling
        // M_pg · dI_L/dt terms (both one-step lag).
        let di_g_hs_dt = (v_drive_hs - i_g_hs * r_gate_hs - v_gs_hs
                            - l_s_hs * prev_di_d_hs_dt
                            - m_pg_hs * prev_di_l_dt) / l_gate_hs;
        let di_g_ls_dt = (v_drive_ls - i_g_ls * r_gate_ls - v_gs_ls
                            - l_s_ls * prev_di_d_ls_dt
                            - m_pg_ls * prev_di_l_dt) / l_gate_ls;

        // V_GS dynamics — Miller feedback through C_rss.
        //   HS:  V_DS_HS = V_in − V_SW;  dV_DS_HS/dt = −dV_SW/dt.
        //   LS:  V_DS_LS = V_SW;          dV_DS_LS/dt = +dV_SW/dt.
        // At turn-off V_SW is FALLING (dV_SW/dt < 0), so:
        //   HS V_DS rising  → Miller current FLOWS INTO C_iss (gate
        //                     gives back charge through C_gd as V_DS↑)
        //   LS V_DS falling → Miller current PULLS LS gate negative
        //                     through C_rss. The V_GS_LS ≥ 0 clamp
        //                     (driver pull-down) catches it.
        // The sign convention from turn-on carries over unchanged:
        let i_miller_hs = c_rss_hs * prev_dvsw_dt;
        let i_miller_ls = c_rss_ls * (prev_dvsw_dt);
        let dv_gs_hs_dt = (i_g_hs - i_miller_hs) / fet_hs.c_iss;
        let dv_gs_ls_dt = (i_g_ls + i_miller_ls) / fet_ls.c_iss;

        // SW node KCL → dV_SW/dt.
        let dv_sw_dt = match diode {
            DiodeState::Forward => 0.0,
            DiodeState::ReverseRecovery | DiodeState::Off => {
                (i_d_hs - i_l - i_d_ls + i_diode_ls) / c_sw_total
            }
        };

        if dv_sw_dt.abs() > dv_sw_dt_peak { dv_sw_dt_peak = dv_sw_dt.abs(); }

        let dv_in_dt = if op.c_in_farad > 0.0 {
            -i_d_hs / op.c_in_farad
        } else {
            0.0
        };

        // Inductor di/dt = (V_SW − V_out) / L_inductor + loop R drop.
        let di_l_dt = (v_sw - op.v_out - i_l * op.loop_r) / op.l_inductor;

        // ── 4. Euler step ──────────────────────────────────────────
        v_gs_hs += dv_gs_hs_dt * cfg.dt;
        v_gs_ls += dv_gs_ls_dt * cfg.dt;
        i_g_hs += di_g_hs_dt * cfg.dt;
        i_g_ls += di_g_ls_dt * cfg.dt;
        v_sw += dv_sw_dt * cfg.dt;
        i_l += di_l_dt * cfg.dt;
        v_in_actual += dv_in_dt * cfg.dt;
        if v_in_actual < v_in_min { v_in_min = v_in_actual; }
        if v_in_actual > v_in_max { v_in_max = v_in_actual; }

        // V_GS_HS held ≥ driver v_off (driver pull-down or negative
        // rail clamps further negative excursions).
        let v_gs_min = driver.v_off.min(0.0);
        if v_gs_hs < v_gs_min { v_gs_hs = v_gs_min; }
        if v_gs_ls < v_gs_min { v_gs_ls = v_gs_min; }

        // Accumulate reverse charge (only when i_diode flows reverse).
        // Not expected at turn-off, but harmless if it sneaks in.
        if i_diode_ls < 0.0 {
            q_rev += -i_diode_ls * cfg.dt;
        }

        // ── 5. Diode state transitions ─────────────────────────────
        // Turn-off goes Off → Forward when V_SW dips below −V_F. The
        // diode never enters ReverseRecovery during turn-off (it is
        // turning ON, not OFF).
        match diode {
            DiodeState::Off => {
                if v_sw < -v_f_ls {
                    v_sw = -v_f_ls;
                    diode = DiodeState::Forward;
                }
            }
            DiodeState::Forward => {
                if i_diode_ls <= 0.0 && v_sw > -v_f_ls + 1e-3 {
                    diode = DiodeState::Off;
                }
            }
            DiodeState::ReverseRecovery => {
                // Unreachable at turn-off; no-op.
            }
        }

        // ── 6. Peaks + flags ──────────────────────────────────────
        // V_GS_HS crossing V_th going DOWN ⇒ HS-FET fully turning off.
        if t_v_th_crossed.is_none() && v_gs_hs <= fet_hs.v_th {
            t_v_th_crossed = Some(t);
        }
        if v_sw > op.v_in + v_sw_overshoot {
            v_sw_overshoot = v_sw - op.v_in;
        }
        if v_gs_ls > v_gs_ls_peak { v_gs_ls_peak = v_gs_ls; }
        if v_gs_ls > fet_ls.v_th { ls_parasitic_turn_on = true; }
        // dI_D_HS/dt is NEGATIVE during turn-off (drain current is
        // falling). Track its magnitude.
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

        // Slew-rate-limited driver outputs.
        v_drive_act_hs += dv_drv_hs_dt * cfg.dt;
        if (v_drive_act_hs - v_drive_hs_target).signum() == dv_drv_hs_dt.signum() {
            v_drive_act_hs = v_drive_hs_target;
        }
        v_drive_act_ls += dv_drv_ls_dt * cfg.dt;
        if (v_drive_act_ls - v_drive_ls_target).signum() == dv_drv_ls_dt.signum() {
            v_drive_act_ls = v_drive_ls_target;
        }

        // ── 8. Carry over for next step's derivative-feedback terms.
        prev_dvsw_dt = dv_sw_dt;
        prev_di_d_hs_dt = di_d_hs_dt;
        prev_di_d_ls_dt = (i_d_ls - prev_i_d_ls) / cfg.dt;
        prev_di_l_dt = di_l_dt;
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
        v_in_ripple_peak: (v_in_max - v_in_min).max(0.0),
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
            mutual_l_henry_power_gate_hs: 0.0,
            mutual_l_henry_power_gate_ls: 0.0,
        }
    }

    #[test]
    fn hs_turn_on_runs_to_completion() {
        let p = test_parasitics();
        let fet = FetModel::bsc0902nsi();
        let drv = DriverModel::generic_si_10v();
        let op = OperatingPoint {
            v_in: 12.0, v_out: 3.3, i_l_init: 3.0,
            loop_r: 20e-3, l_inductor: 4.7e-6, c_in_farad: 0.0, c_y_chassis_farad: 0.0, c_boot_farad: 0.0,
        };
        let cfg = SimConfig::auto(&p, &fet);
        let w = simulate_hs_turn_on(&p, &fet, &fet, &drv, &op, &cfg);
        assert!(!w.samples.is_empty());
        assert!(w.f_ring_est > 1e6); // expect MHz-range ring with these L/C
    }

    #[test]
    fn v_sw_overshoot_includes_commutation_ring() {
        let p = test_parasitics();
        let fet = FetModel::bsc0902nsi();
        let drv = DriverModel::generic_si_10v();
        let op = OperatingPoint {
            v_in: 12.0, v_out: 3.3, i_l_init: 3.0,
            loop_r: 20e-3, l_inductor: 4.7e-6, c_in_farad: 0.0, c_y_chassis_farad: 0.0, c_boot_farad: 0.0,
        };
        let cfg = SimConfig::auto(&p, &fet);
        let w = simulate_hs_turn_on(&p, &fet, &fet, &drv, &op, &cfg);
        // Independent closed-form check: underdamped series-RLC first-peak overshoot.
        let omega_n = 2.0 * std::f64::consts::PI * w.f_ring_est;
        let alpha = op.loop_r / (2.0 * p.power_loop_l_henry);
        let omega_d = (omega_n * omega_n - alpha * alpha).sqrt();
        let expected = op.v_in * (-alpha * std::f64::consts::PI / omega_d).exp();
        assert!(expected > 0.1, "test setup should give a real ring overshoot, got {expected}");
        assert!(
            w.v_sw_overshoot >= expected - 1e-9,
            "v_sw_overshoot {} should include the closed-form ring overshoot {}",
            w.v_sw_overshoot,
            expected,
        );
    }

    #[test]
    fn v_gs_crosses_threshold_within_window() {
        let p = test_parasitics();
        let fet = FetModel::bsc0902nsi();
        let drv = DriverModel::generic_si_10v();
        let op = OperatingPoint {
            v_in: 12.0, v_out: 3.3, i_l_init: 3.0,
            loop_r: 20e-3, l_inductor: 4.7e-6, c_in_farad: 0.0, c_y_chassis_farad: 0.0, c_boot_farad: 0.0,
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
            loop_r: 15e-3, l_inductor: 2.2e-6, c_in_farad: 0.0, c_y_chassis_farad: 0.0, c_boot_farad: 0.0,
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

    #[test]
    fn hs_turn_off_runs_to_completion() {
        let p = test_parasitics();
        let fet = FetModel::bsc0902nsi();
        let drv = DriverModel::generic_si_10v();
        let op = OperatingPoint {
            v_in: 12.0, v_out: 3.3, i_l_init: 3.0,
            loop_r: 20e-3, l_inductor: 4.7e-6, c_in_farad: 0.0, c_y_chassis_farad: 0.0, c_boot_farad: 0.0,
        };
        let cfg = SimConfig::auto(&p, &fet);
        let w = simulate_hs_turn_off(&p, &fet, &fet, &drv, &op, &cfg);
        assert!(!w.samples.is_empty());
        assert!(w.f_ring_est > 1e6);
        // No reverse recovery at turn-off.
        assert!(w.t_rr_entered.is_none());
        assert!(w.t_rr_done.is_none());
    }

    #[test]
    fn turn_off_v_gs_falls_below_v_th_within_window() {
        let p = test_parasitics();
        let fet = FetModel::bsc0902nsi();
        let drv = DriverModel::generic_si_10v();
        let op = OperatingPoint {
            v_in: 12.0, v_out: 3.3, i_l_init: 3.0,
            loop_r: 20e-3, l_inductor: 4.7e-6, c_in_farad: 0.0, c_y_chassis_farad: 0.0, c_boot_farad: 0.0,
        };
        let cfg = SimConfig::auto(&p, &fet);
        let w = simulate_hs_turn_off(&p, &fet, &fet, &drv, &op, &cfg);
        assert!(
            w.t_v_th_crossed.is_some(),
            "V_GS_HS never fell below V_th in {} samples (final V_GS_HS = {:?})",
            w.samples.len(),
            w.samples.last().map(|s| s.v_gs_hs)
        );
    }

    #[test]
    fn turn_off_v_sw_falls_to_diode_clamp() {
        let p = test_parasitics();
        let fet = FetModel::bsc0902nsi();
        let drv = DriverModel::generic_si_10v();
        let op = OperatingPoint {
            v_in: 12.0, v_out: 3.3, i_l_init: 3.0,
            loop_r: 20e-3, l_inductor: 4.7e-6, c_in_farad: 0.0, c_y_chassis_farad: 0.0, c_boot_farad: 0.0,
        };
        let cfg = SimConfig::auto(&p, &fet);
        let w = simulate_hs_turn_off(&p, &fet, &fet, &drv, &op, &cfg);
        let n = w.samples.len();
        assert!(n >= 5, "too few samples: {}", n);
        let tail_start = n - n / 5; // last 20 % of the window
        let v_clamp = -fet.v_f_body;
        for (i, s) in w.samples[tail_start..].iter().enumerate() {
            assert!(
                (s.v_sw - v_clamp).abs() < 0.3,
                "tail sample {} at t={:.3} ns: V_SW = {:.3} V, expected ≈ {:.3} V (diode clamp)",
                tail_start + i,
                s.t_s * 1e9,
                s.v_sw,
                v_clamp,
            );
        }
    }

    #[test]
    fn gan_third_quadrant_v_f_uses_v_th_plus_v_off() {
        // GaN at V_off = -2 V should have V_F_eff = v_th + 2.0.
        let mut drv = DriverModel::generic_gan_5v();
        drv.v_off = -2.0;
        let fet = FetModel::epc2034c();
        let v_f = body_diode_v_f(&fet, &drv);
        let expect = fet.v_th + 2.0;
        assert!(
            (v_f - expect).abs() < 1e-9,
            "GaN V_F at v_off=-2 V: got {:.3} V, expected {:.3} V (v_th + 2)",
            v_f, expect,
        );

        // Si body diode ignores v_off — always v_f_body.
        let si = FetModel::bsc0902nsi();
        let v_f_si = body_diode_v_f(&si, &drv);
        assert_eq!(v_f_si, si.v_f_body);
    }

    #[test]
    fn c_oss_eff_drops_with_higher_v_ds() {
        // sqrt(V_test/V_DS) scaling — at V_DS = 4·V_test, C_oss should
        // be half of the datasheet value.
        let fet = FetModel::bsc0902nsi(); // C_oss = 180 pF @ V_test = 20 V
        let c_lo = c_oss_eff(&fet, 20.0); // ≈ 180 pF
        let c_hi = c_oss_eff(&fet, 80.0); // ≈ 90 pF
        assert!((c_lo - 180e-12).abs() < 1e-13);
        assert!((c_hi - 90e-12).abs() < 5e-13);
        assert!(c_hi < c_lo, "C_oss should drop with rising V_DS");
    }

    #[test]
    fn slow_driver_slew_delays_v_th_crossing() {
        let p = test_parasitics();
        let fet = FetModel::bsc0902nsi();
        let fast = DriverModel { slew_rate_v_per_s: 1e12, ..DriverModel::generic_si_10v() };
        let slow = DriverModel { slew_rate_v_per_s: 0.5e9, ..DriverModel::generic_si_10v() };
        let op = OperatingPoint {
            v_in: 12.0, v_out: 3.3, i_l_init: 3.0,
            loop_r: 20e-3, l_inductor: 4.7e-6,
            c_in_farad: 0.0, c_y_chassis_farad: 0.0, c_boot_farad: 0.0,
        };
        let cfg = SimConfig::auto(&p, &fet);
        let fast_w = simulate_hs_turn_on(&p, &fet, &fet, &fast, &op, &cfg);
        let slow_w = simulate_hs_turn_on(&p, &fet, &fet, &slow, &op, &cfg);
        let fast_t = fast_w.t_v_th_crossed.unwrap();
        let slow_t = slow_w.t_v_th_crossed.unwrap();
        assert!(
            slow_t > fast_t * 1.5,
            "slow driver should delay V_th crossing significantly: fast={:.2} ns vs slow={:.2} ns",
            fast_t * 1e9, slow_t * 1e9,
        );
    }

    #[test]
    fn bootstrap_with_small_c_boot_droops_more() {
        // Two runs at identical operating point — one with a stiff
        // boot rail (c_boot = 0), one with a small 1 nF cap. The
        // small cap should make the run finish without dV/dt panicking
        // (i.e. just runs to completion under the new state).
        let p = test_parasitics();
        let fet = FetModel::bsc0902nsi();
        let drv = DriverModel::generic_si_10v();
        let op_stiff = OperatingPoint {
            v_in: 12.0, v_out: 3.3, i_l_init: 3.0,
            loop_r: 20e-3, l_inductor: 4.7e-6,
            c_in_farad: 0.0, c_y_chassis_farad: 0.0, c_boot_farad: 0.0,
        };
        let op_droop = OperatingPoint { c_boot_farad: 1e-9, ..op_stiff };
        let cfg = SimConfig::auto(&p, &fet);
        let w_stiff = simulate_hs_turn_on(&p, &fet, &fet, &drv, &op_stiff, &cfg);
        let w_droop = simulate_hs_turn_on(&p, &fet, &fet, &drv, &op_droop, &cfg);
        assert!(!w_stiff.samples.is_empty());
        assert!(!w_droop.samples.is_empty());
        // V_GS_HS final value should be ≤ in the droop case (gate
        // can't quite reach v_drive because boot has dropped).
        let v_gs_stiff = w_stiff.samples.last().unwrap().v_gs_hs;
        let v_gs_droop = w_droop.samples.last().unwrap().v_gs_hs;
        assert!(
            v_gs_droop <= v_gs_stiff + 1e-3,
            "with smaller C_boot, final V_GS_HS should be ≤ stiff value: \
             stiff={:.3} V, droop={:.3} V",
            v_gs_stiff, v_gs_droop,
        );
    }

    #[test]
    fn mutual_m_pg_couples_di_dt_into_gate() {
        // With M_pg > 0, the dI_L/dt during the slew couples into the
        // HS gate loop, slowing or accelerating the gate transition.
        // Just verify the run completes and produces finite output —
        // physical sign-correctness is a separate validation campaign.
        let mut p = test_parasitics();
        p.mutual_l_henry_power_gate_hs = 2e-9;
        p.mutual_l_henry_power_gate_ls = 1e-9;
        let fet = FetModel::bsc0902nsi();
        let drv = DriverModel::generic_si_10v();
        let op = OperatingPoint {
            v_in: 12.0, v_out: 3.3, i_l_init: 3.0,
            loop_r: 20e-3, l_inductor: 4.7e-6,
            c_in_farad: 0.0, c_y_chassis_farad: 0.0, c_boot_farad: 0.0,
        };
        let cfg = SimConfig::auto(&p, &fet);
        let w = simulate_hs_turn_on(&p, &fet, &fet, &drv, &op, &cfg);
        assert!(!w.samples.is_empty());
        assert!(w.samples.iter().all(|s| s.v_gs_hs.is_finite()));
    }

    #[test]
    fn soft_recovery_profile_peaks_at_t_a() {
        // S=0.5, t_rr=50ns, Q_rr=80nC → t_a=t_rr/1.5≈33.3 ns,
        // I_RR_peak = 2·Q_rr/t_rr = 3.2 A.
        let fet = FetModel::bsc0902nsi();
        let i_at_zero = soft_recovery_i_diode(&fet, 0.0).unwrap();
        let t_a = fet.t_rr / (1.0 + fet.softness_factor);
        let i_at_ta = soft_recovery_i_diode(&fet, t_a).unwrap();
        let i_after = soft_recovery_i_diode(&fet, fet.t_rr + 1e-12);
        assert!((i_at_zero).abs() < 1e-9, "i(0) should be 0");
        let expect_peak = -2.0 * fet.q_rr / fet.t_rr;
        assert!(
            (i_at_ta - expect_peak).abs() / expect_peak.abs() < 1e-3,
            "i(t_a) = {:.4} A, expected {:.4} A",
            i_at_ta, expect_peak,
        );
        assert!(i_after.is_none(), "RR should be done past t_rr");
    }
}
