use electronics_sim::{
    Capacitance, Current, CurrentModeConverter, Inductance, Resistance, Time, Voltage, math::Func,
};
use full_control::{
    buck_boost::{BuckBoostTransferFunction, BuckBoostWeights, Mode},
    control_2p2z::{Parameters, PhaseMargin, Topology as ControlTopology},
};

#[cfg(feature = "rerun")]
use electronics_sim::plot;

// ── Circuit schematic ─────────────────────────────────────────────────────────
//
// 4-Switch Non-Inverting Buck-Boost Converter — Modelled Parameters
//
//        r_in ①    l_in ①
// V_src──/\/\/──UUUUUU──┬──[Q1]──r_series③──L_ind④──[Q3]──┬── V_out
//                       │    │                        │    │       │
//                     C_in② [Q2]                   [Q4] r_esr⑤ C_out⑥ Load
//                       │    │                        │    │       │       │
//                      GND  GND                      GND  GND    GND     GND
//
// Topology modes:
//   Buck:      Q1/Q2 switching; Q3 always ON,  Q4 always OFF
//   Boost:     Q3/Q4 switching; Q1 always ON,  Q2 always OFF
//   BuckBoost: Q1+Q4 switch together; Q2+Q3 complementary
//
// Parameter legend:
//   ① r_in      — cable / source resistance  }
//      l_in      — cable inductance           } model the cable from supply to converter
//   ② C_in      — input bulk / decoupling capacitor
//   ③ r_series  — lumped conduction loss: inductor DCR + 2× switch R_dson
//   ④ L_ind     — main switching inductor
//   ⑤ r_esr     — output cap ESR  (shapes v_sensed at ADC sample point)
//   ⑥ C_out     — output filter capacitor
//
// State variables tracked across cycles:
//   v_in_cap   — voltage across C_in
//   i_in_cap   — current through l_in
//   i_inductor — inductor current (valley, sampled at start of each cycle)
//   v_out      — output capacitor voltage

// ── Physical circuit constants ────────────────────────────────────────────────
const F_SW: f64 = 500e3; // 500 kHz
const T_PERIOD: Time = Time(1.0 / F_SW);
const C_OUT: Capacitance = Capacitance(47e-6);
const L_INDUCTOR: Inductance = Inductance(4e-6);
const CS_GAIN: f64 = 0.066; // 66 mV/A
const R_ESR: Resistance = Resistance(10e-3); // 10 mΩ
/// Lumped series resistance: inductor DCR (~15 mΩ) + two conducting switch R_dson (~10 mΩ each).
/// Adjust to match measured inductor / FET specs.
const R_SERIES: Resistance = Resistance(35e-3); // 35 mΩ
/// Input bulk capacitance (ceramic + electrolytic on the converter input rail).
const C_IN: Capacitance = Capacitance(10e-6); // 10 µF
/// Equivalent series resistance of the input capacitor.
/// Typical ceramic MLCC: 1–10 mΩ.  Only affects the EMI ripple estimate.
const R_ESR_CIN: f64 = 5e-3; // 5 mΩ
/// Thevenin source resistance (cable + connector + supply output impedance).
/// Set to 0.0 for an ideal stiff supply (cap still droops and recharges instantly).
const R_IN: f64 = 0.1; // 100 mΩ
/// Input cable inductance.  Typical: ~2 nH/cm, so 50 cm ≈ 100 nH, 1 m ≈ 200 nH.
/// Set to 0.0 to disable (falls back to the RC recharge approximation).
const L_IN: Inductance = Inductance(500e-9); // 500 nH ≈ 25 cm cable
const V_TARGET: Voltage = Voltage(13.5);
const V_IN_BUCK: Voltage = Voltage(24.0);
const V_IN_BOOST: Voltage = Voltage(8.0);
const R_LOAD: f64 = 6.0; // 6 Ω → 2 A at 12 V
const MAX_CURRENT: Current = Current(6.0);
/// −3 dB bandwidth of the current-sense RC filter before the comparator (Hz).
/// ACS37030LLZATR-020B3 datasheet: small-signal −3 dB bandwidth = 5 MHz (C_L = 100 pF).
const BW_CURRENT_SENSE: f64 = 5e6; // 5 MHz
/// −3 dB bandwidth of the slope-compensation DAC output filter (Hz).
/// STM32G474 fast (15 MSPS unbuffered) DAC: 10%–90% settling time = 16 ns typical.
/// Equivalent first-order τ = 16 ns / ln(9) ≈ 7.3 ns → f_−3dB ≈ 22 MHz.
const BW_DAC: f64 = 22e6; // ~22 MHz
/// Comparator propagation delay (s).
/// STM32G474 COMP: t_D = 16.7 ns typical (V_DDA ≥ 2.7 V, 50 pF load, 100 mV overdrive).
const T_COMPARATOR_DELAY: f64 = 16.7e-9; // 16.7 ns
/// Number of switching cycles between consecutive 2P2Z controller updates.
/// Set to 1 for full-rate (maximum bandwidth), or higher to simulate a slower MCU.
const CYCLES_PER_TICK: usize = 1;

const PM: PhaseMargin = PhaseMargin::Manual {
    phase_margin: 75.0f64.to_radians(),
};

// ── Compile-time 2P2Z weights per operating mode ──────────────────────────────
//
// Each set is linearised at a representative V_in for that mode.  The
// physics topology in the simulator tracks the current mode (Buck/BuckBoost/Boost)
// via sync_sim().  On a mode switch the controller gains and slope-compensation
// ramp also change, both handled gracefully by the bumpless-transfer mechanism.

// ── Crossover safety factor ────────────────────────────────────────────────────
// The crossover frequency is selected automatically for each design point via
// optimal_f_x_divisor(): the highest bandwidth where the limit-cycle criterion
// b0 × ΔV_out_ripple ≤ vpp / safety_factor is satisfied.
// Higher safety_factor → lower bandwidth → more headroom for transient excursions.
const SAFETY: f64 = 2.0;

// Buck region: design point V_in = 24 V → V_out = 13.5 V
const PARAMS: Parameters = Parameters {
    v_out: V_TARGET.0,
    c_out: C_OUT.0,
    f_sw: F_SW,
    l_inductor: L_INDUCTOR.0,
    r_esr_out_cap: R_ESR.0,
    current_sense_gain: CS_GAIN,
    i_load: V_TARGET.0 / R_LOAD,
    v_diode: 0.0,
    phase_margin: PM,
    safety_factor: SAFETY,
    cycles_per_tick: CYCLES_PER_TICK,
};

const CONTROLLER_DAC_SETTINGS: BuckBoostTransferFunction =
    BuckBoostTransferFunction::new(PARAMS, 24.0, 8.0, V_TARGET.0);
const WEIGHTS: BuckBoostWeights<f32> = CONTROLLER_DAC_SETTINGS.to_weights();

// Slope compensation in A/s for each gain-schedule region (negative = downward).
// The BuckBoost plant always has S_n = V_in/L, so each slope is computed from
// the BuckBoost DAC setting at the corresponding design-point V_in.
const SLOPE_BUCK: f64 = CONTROLLER_DAC_SETTINGS.dac_buck.dac_slope / CS_GAIN;
const SLOPE_BOOST: f64 = CONTROLLER_DAC_SETTINGS.dac_boost.dac_slope / CS_GAIN;
const SLOPE_BB: f64 = CONTROLLER_DAC_SETTINGS.dac_buck_boost.dac_slope / CS_GAIN;

// ── Simulation sync ───────────────────────────────────────────────────────────

/// Update the simulator before each tick().
///
/// Synchronise the simulator's topology and slope to match the current control mode.
///
/// The physics model in `lib.rs` correctly implements all three topologies:
///   - Buck ON:       V_L = V_in − V_out  (RLC)
///   - BuckBoost ON:  V_L = V_in          (linear, capacitor decoupled)
///   - Boost ON:      V_L = V_in          (linear, capacitor decoupled)
fn sync_sim(sim: &mut CurrentModeConverter, mode: Mode) {
    sim.topology = mode;
    let dac_settings = match mode {
        Mode::Buck => CONTROLLER_DAC_SETTINGS.dac_buck,
        Mode::Boost => CONTROLLER_DAC_SETTINGS.dac_boost,
        Mode::BuckBoost => CONTROLLER_DAC_SETTINGS.dac_buck_boost,
    };
    sim.parameters.slope_amp_per_sec = dac_settings.dac_slope / CS_GAIN;
}

// ── V_in sweep profile ────────────────────────────────────────────────────────

/// 0..25 %  : 24 V  (steady, Buck region)
/// 25..75 % : 24 V → 8 V → 24 V  (cosine, exercises BuckBoost and Boost)
/// 75..100% : 24 V  (steady, Buck region)
///
/// Cosine chosen so V_in is continuous and differentiable at the endpoints.
fn v_in_sweep(step: usize, total: usize) -> Voltage {
    let frac = step as f64 / total as f64;
    if frac < 0.25 || frac > 0.75 {
        V_IN_BUCK
    } else {
        let t = (frac - 0.25) / 0.5; // 0..1 over the active sweep portion
        // t=0 → 24V,  t=0.5 → 8V,  t=1 → 24V
        let delta = V_IN_BUCK - V_IN_BOOST;
        (V_IN_BOOST + delta * 0.5) + delta * 0.5 * f64::cos(2.0 * std::f64::consts::PI * t)
    }
}

// ── Logger ────────────────────────────────────────────────────────────────────

struct Logger {
    #[cfg(feature = "rerun")]
    rec: rerun::RecordingStream,
    prev_mode: Option<Mode>,
    /// Count cycles since the last mode change; used to print at high density
    /// immediately after transitions so oscillations are visible.
    cycles_since_change: usize,
    step: usize,
}

impl Logger {
    fn new() -> Self {
        #[cfg(feature = "text-log")]
        Self::print_header();

        Self {
            #[cfg(feature = "rerun")]
            rec: rerun::RecordingStreamBuilder::new("buck_boost_test")
                .spawn()
                .unwrap(),
            prev_mode: None,
            cycles_since_change: usize::MAX,
            step: 0,
        }
    }

    #[cfg(feature = "text-log")]
    fn print_header() {
        println!(
            "{:>9} {:>6} {:>7} {:>7} {:>9} {:>7} {:>5}",
            "t[ms]", "Vin[V]", "Vout[V]", "err[V]", "mode", "IL[A]", "d[%]",
        );
        println!("{}", "-".repeat(72));
    }

    fn log(
        &mut self,
        sim: &CurrentModeConverter,
        t_on: Time,
        i_max: Current,
        v_in: Voltage,
        i_out: Current,
        mode: Mode,
        time: &mut Time,
    ) {
        let mode_changed = self.prev_mode.map_or(true, |m| m != mode);
        if mode_changed {
            self.on_mode_change(mode, v_in.0, sim.v_out.0, time.0);
            self.cycles_since_change = 0;
        } else {
            self.cycles_since_change = self.cycles_since_change.saturating_add(1);
        }
        self.prev_mode = Some(mode);

        #[cfg(feature = "rerun")]
        {
            plot(&self.rec, sim, t_on, i_max, v_in, i_out, time);
            // Log mode as a scalar: 0=Buck, 1=BuckBoost, 2=Boost
            self.rec.set_timestamp_secs_since_epoch("time", time.0);
            let mode_val = match mode {
                Mode::Buck => -1.0_f64,
                Mode::BuckBoost => 0.0,
                Mode::Boost => 1.0,
            };
            self.rec
                .log("mode", &rerun::Scalars::new([mode_val]))
                .unwrap();
            // Input-side / EMI channels
            self.rec
                .log("v_in_cap", &rerun::Scalars::new([sim.v_in_cap.0]))
                .unwrap();
            self.rec
                .log("i_in_cap", &rerun::Scalars::new([sim.i_in_cap.0]))
                .unwrap();
            self.rec
                .log(
                    "v_in_ripple_est",
                    &rerun::Scalars::new([sim.v_in_ripple_est.0]),
                )
                .unwrap();
        }

        #[cfg(feature = "text-log")]
        {
            // plot() normally advances `time`; do it here instead.
            *time += T_PERIOD;

            // Print at high density near mode transitions (first 150 cycles),
            // then every 50 cycles elsewhere.
            let dense = self.cycles_since_change < 150;
            let should_print = dense || self.step % 50 == 0;

            if should_print {
                let d_pct = t_on.0 * F_SW * 100.0;
                let err = V_TARGET.0 - sim.v_out.0;
                println!(
                    "{:9.4} {:6.3} {:7.4} {:+7.4} {:9} {:7.4} {:5.1}",
                    time.0 * 1e3,
                    v_in.0,
                    sim.v_out.0,
                    err,
                    mode.name(),
                    sim.i_inductor.0,
                    d_pct,
                );
            }
        }

        self.step += 1;
        let _ = (i_max, i_out); // suppress unused warnings in text-log path
    }

    fn on_mode_change(&self, new_mode: Mode, v_in: f64, v_out: f64, t: f64) {
        let prev = self.prev_mode.map(|m| m.name().trim()).unwrap_or("(start)");
        // Use println! so mode changes are interleaved with the data table.
        println!(
            ">>> MODE {prev} → {} at t={:.4}ms  Vin={:.3}V  Vout={:.4}V",
            new_mode.name().trim(),
            t * 1e3,
            v_in,
            v_out,
        );
    }
}

// ── Main ──────────────────────────────────────────────────────────────────────

fn main() {
    // ── Controller design summary ──────────────────────────────────────────────
    // crossover_divisor() is const fn, evaluated at compile time inside to_transfer_function().
    // Print here for informational purposes.
    println!("Controller design (safety_factor={SAFETY}):");
    println!(
        "  Buck:      f_x = {:.0} Hz (divisor={:.1})",
        F_SW / PARAMS.crossover_divisor(ControlTopology::Buck, V_IN_BUCK.0),
        PARAMS.crossover_divisor(ControlTopology::Buck, V_IN_BUCK.0)
    );
    println!(
        "  Boost:     f_x = {:.0} Hz (divisor={:.1})",
        F_SW / PARAMS.crossover_divisor(ControlTopology::Boost, V_IN_BOOST.0),
        PARAMS.crossover_divisor(ControlTopology::Boost, V_IN_BOOST.0)
    );
    println!(
        "  BuckBoost: f_x = {:.0} Hz (divisor={:.1})",
        F_SW / PARAMS.crossover_divisor(ControlTopology::BuckBoost, V_TARGET.0),
        PARAMS.crossover_divisor(ControlTopology::BuckBoost, V_TARGET.0)
    );

    // Start the sim in BuckBoost topology; sync_sim() will update it each cycle.

    let parameters = electronics_sim::Parameters {
        period: T_PERIOD,
        slope_amp_per_sec: SLOPE_BB,
        r_series: R_SERIES,
        r_esr: R_ESR,
        c_out: C_OUT,
        l_inductor: L_INDUCTOR,
        c_in: C_IN,
        r_esr_cin: Resistance(R_ESR_CIN),
        r_in: Resistance(R_IN),
        l_in: L_IN,
        tau_current_sense: electronics_sim::Parameters::bw_to_tau(BW_CURRENT_SENSE),
        tau_dac: electronics_sim::Parameters::bw_to_tau(BW_DAC),
        t_prop_delay: Time(T_COMPARATOR_DELAY),
        t_dac_sample: Time(1.0 / 15e6),
    };
    let mut sim = CurrentModeConverter::new(parameters, Mode::BuckBoost);

    let mut ctrl = WEIGHTS.to_controller(0.0, MAX_CURRENT.0 as f32 * CS_GAIN as f32);
    let mut logger = Logger::new();
    let mut time = Time(0.0);

    // ── Soft-start ────────────────────────────────────────────────────────────
    // Ramp the voltage reference 0 → V_TARGET over 3000 cycles with V_in at
    // 24 V (well inside the Buck gain-schedule region).
    let soft_cycles = 3000_usize;
    let v_in_startup = Voltage(24.0);

    let vins = [24.0, 12.0, 8.0];

    let mut ctrl_out = (0.0f32, Mode::BuckBoost);
    let mut i_out = Current(0.0);
    for i in 0..soft_cycles {
        let soft_target = V_TARGET.0 * (i + 1) as f64 / soft_cycles as f64;
        if i % CYCLES_PER_TICK == 0 {
            ctrl_out = ctrl.update(v_in_startup.0, sim.v_sensed(i_out).0, soft_target);
        }
        let (cmd_v, mode) = ctrl_out;
        sync_sim(&mut sim, mode);

        let trip = Current((cmd_v as f64 / CS_GAIN).clamp(0.0, MAX_CURRENT.0));
        let (t_on, i_max) = sim.tick(v_in_startup, trip, |v| {
            i_out = Current(v.0 / R_LOAD);
            i_out
        });

        logger.log(&sim, t_on, i_max, v_in_startup, i_out, mode, &mut time);
    }

    // ── V_in sweep ────────────────────────────────────────────────────────────
    // Sweep V_in 24 V → 8 V → 24 V while holding V_out at 12 V.
    // A load step (R × 2 → more current) fires at the midpoint to test
    // controller recovery during the boost phase.
    let vin_sweeps = 3;
    let sweep_cycles = 8000_usize;
    let total_cycles = vin_sweeps * sweep_cycles;

    // Charge a battery
    let bat_test_cycles = 64_000;
    let mut bat = Battery::new(Voltage(11.0));
    for v_in in vins {
        let v_in = Voltage(v_in);
        let mut ctrl_out = (0.0f32, Mode::BuckBoost);
        let mut i_out = Current(0.0);
        for i in 0..bat_test_cycles {
            if i % CYCLES_PER_TICK == 0 {
                ctrl_out = ctrl.update(v_in.0, sim.v_sensed(i_out).0, V_TARGET.0);
            }
            let (cmd_v, mode) = ctrl_out;
            sync_sim(&mut sim, mode);

            let trip = Current((cmd_v as f64 / CS_GAIN).clamp(0.0, MAX_CURRENT.0));
            let (t_on, i_max) = sim.tick(v_in, trip, |v| {
                i_out = bat.tick(v, T_PERIOD);
                i_out
            });

            logger.log(&sim, t_on, i_max, v_in, i_out, mode, &mut time);
        }
    }

    let rs = [12.0, 6.0, f64::MAX];

    let mut ctrl_out = (0.0f32, Mode::BuckBoost);
    let mut i_out = Current(0.0);
    for i in 0..total_cycles {
        let v_in = v_in_sweep(i % sweep_cycles, sweep_cycles);
        let r = rs[i * rs.len() / total_cycles];

        if i % CYCLES_PER_TICK == 0 {
            ctrl_out = ctrl.update(v_in.0, sim.v_sensed(i_out).0, V_TARGET.0);
        }
        let (cmd_v, mode) = ctrl_out;
        sync_sim(&mut sim, mode);

        let trip = Current((cmd_v as f64 / CS_GAIN).clamp(0.0, MAX_CURRENT.0));
        let (t_on, i_max) = sim.tick(v_in, trip, |v| {
            i_out = Current(v.0 / r);
            i_out
        });

        logger.log(&sim, t_on, i_max, v_in, i_out, mode, &mut time);
    }

    for v_in in vins {
        let v_in = Voltage(v_in);
        let mut ctrl_out = (0.0f32, Mode::BuckBoost);
        let mut i_out = Current(0.0);
        for i in 0..sweep_cycles {
            let r = rs[i * rs.len() / sweep_cycles];

            if i % CYCLES_PER_TICK == 0 {
                ctrl_out = ctrl.update(v_in.0, sim.v_sensed(i_out).0, V_TARGET.0);
            }
            let (cmd_v, mode) = ctrl_out;
            sync_sim(&mut sim, mode);

            let trip = Current((cmd_v as f64 / CS_GAIN).clamp(0.0, MAX_CURRENT.0));
            let (t_on, i_max) = sim.tick(v_in, trip, |v| {
                i_out = Current(v.0 / r);
                i_out
            });

            logger.log(&sim, t_on, i_max, v_in, i_out, mode, &mut time);
        }
    }
}
struct Battery {
    i: Current,
    v_int: Voltage,
    r_esr: Resistance,
    l_cable: Inductance,
    c: Capacitance,
}

impl Battery {
    pub fn new(v_init: Voltage) -> Self {
        Self {
            i: Current(0.0),
            v_int: v_init,
            r_esr: Resistance(50e-3),
            l_cable: Inductance(1e-6),
            c: Capacitance(0.25),
        }
    }

    pub fn tick(&mut self, v_in: Voltage, dt: Time) -> Current {
        let q_out = 0.0;
        let f =
            electronics_sim::math::rlc(v_in, self.v_int, self.i, self.l_cable, self.c, self.r_esr);
        let q_in = f.integral(0.0).f(dt.0);
        self.v_int += Voltage((q_in - q_out) / self.c.0);
        self.i = Current(f.f(dt.0));

        self.i
    }
}
