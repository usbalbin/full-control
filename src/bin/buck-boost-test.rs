use electronics_sim::{
    Capacitance, Current, CurrentModeConverter, Inductance, Resistance, Time, Topology, Voltage,
    math::Func,
};
use half_bridge::control_2p2z::{
    DacSettings, ParametersBuck, PhaseMargin, Topology as ControlTopology, TransferFunction,
    TwoPoleTwoZero, TwoPoleTwoZeroParams,
};

#[cfg(not(feature = "text-log"))]
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
const R_ESR: f64 = 10e-3; // 10 mΩ
/// Lumped series resistance: inductor DCR (~15 mΩ) + two conducting switch R_dson (~10 mΩ each).
/// Adjust to match measured inductor / FET specs.
const R_SERIES: Resistance = Resistance(35e-3); // 35 mΩ
/// Input bulk capacitance (ceramic + electrolytic on the converter input rail).
const C_IN: Capacitance = Capacitance(10e-6); // 10 µF
/// Thevenin source resistance (cable + connector + supply output impedance).
/// Set to 0.0 for an ideal stiff supply (cap still droops and recharges instantly).
const R_IN: f64 = 0.1; // 100 mΩ
/// Input cable inductance.  Typical: ~2 nH/cm, so 50 cm ≈ 100 nH, 1 m ≈ 200 nH.
/// Set to 0.0 to disable (falls back to the RC recharge approximation).
const L_IN: Inductance = Inductance(500e-9); // 500 nH ≈ 25 cm cable
const V_TARGET: Voltage = Voltage(13.5);
const R_LOAD: f64 = 6.0; // 6 Ω → 2 A at 12 V
const MAX_CURRENT: Current = Current(6.0);
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

// Buck region: design point V_in = 24 V → V_out = 12 V
const PARAMS_BUCK: ParametersBuck = ParametersBuck {
    v_in: 24.0,
    v_out: V_TARGET.0,
    c_out: C_OUT.0,
    f_sw: F_SW,
    l_inductor: L_INDUCTOR.0,
    r_esr_out_cap: R_ESR,
    current_sense_gain: CS_GAIN,
    i_load: V_TARGET.0 / R_LOAD,
    v_diode: 0.0,
    topology: ControlTopology::BuckBoost, // gains computed for BuckBoost plant at 24 V
    phase_margin: PM,
    // Large C_out (470 µF) gives a very low plant pole (ω_p1 ≈ D'^2/(R·C) ≈ 370 rad/s
    // at 24V design point).  Without a reduced crossover frequency the b-coefficients
    // become so large (b0 ≈ 18) that steady-state output ripple drives the controller
    // into limit cycling between the saturation rails.  f_sw/100 = 5 kHz keeps
    // b0 ≈ 2–3 so the linear range covers the normal per-cycle voltage ripple.
    f_x_divisor: 20.0,
    cycles_per_tick: CYCLES_PER_TICK,
};

// Transition region: design point V_in = V_out = 12 V (unity gain, worst case)
const PARAMS_BB: ParametersBuck = ParametersBuck {
    v_in: V_TARGET.0,
    v_out: V_TARGET.0,
    c_out: C_OUT.0,
    f_sw: F_SW,
    l_inductor: L_INDUCTOR.0,
    r_esr_out_cap: R_ESR,
    current_sense_gain: CS_GAIN,
    i_load: V_TARGET.0 / R_LOAD,
    v_diode: 0.0,
    topology: ControlTopology::BuckBoost,
    phase_margin: PM,
    f_x_divisor: 50.0,
    cycles_per_tick: CYCLES_PER_TICK,
};

// Boost region: design point V_in = 8 V → V_out = 12 V
const PARAMS_BOOST: ParametersBuck = ParametersBuck {
    v_in: 8.0,
    v_out: V_TARGET.0,
    c_out: C_OUT.0,
    f_sw: F_SW,
    l_inductor: L_INDUCTOR.0,
    r_esr_out_cap: R_ESR,
    current_sense_gain: CS_GAIN,
    i_load: V_TARGET.0 / R_LOAD,
    v_diode: 0.0,
    topology: ControlTopology::BuckBoost, // still BuckBoost plant model
    phase_margin: PM,
    // At 8V→12V in BuckBoost topology (always used in sim), D'=0.4, I_L_avg=7.5A.
    // The steady-state trip ≈ 8A leaves only 2A headroom below MAX_CURRENT=10A.
    // With f_x_divisor=100: b0≈10.5, linear range = 0.132/10.5 = 12.6mV — too small
    // vs the 30-50mV per-cycle V_out ripple from the RLC dynamics.  Each ripple cycle
    // saturates the controller, causing an 8-cycle bang-bang limit cycle.
    // f_x_divisor=300 reduces b0 to ≈3.5, giving 38mV linear range which comfortably
    // contains the normal per-cycle ripple.  Bandwidth trades off (f_x = 500kHz/300 =
    // 1.67kHz), but the Boost region is only a small part of the sweep.
    f_x_divisor: 300.0,
    cycles_per_tick: CYCLES_PER_TICK,
};

const TF_BUCK: (TransferFunction, DacSettings) = PARAMS_BUCK.to_transfer_function();
const TF_BB: (TransferFunction, DacSettings) = PARAMS_BB.to_transfer_function();
const TF_BOOST: (TransferFunction, DacSettings) = PARAMS_BOOST.to_transfer_function();

const WEIGHTS_BUCK: TwoPoleTwoZeroParams<f32> = TF_BUCK.0.to_2p2z();
const WEIGHTS_BB: TwoPoleTwoZeroParams<f32> = TF_BB.0.to_2p2z();
const WEIGHTS_BOOST: TwoPoleTwoZeroParams<f32> = TF_BOOST.0.to_2p2z();

const DAC_BUCK: DacSettings = TF_BUCK.1;
const DAC_BB: DacSettings = TF_BB.1;
const DAC_BOOST: DacSettings = TF_BOOST.1;

// Slope compensation in A/s for each gain-schedule region (negative = downward).
// The BuckBoost plant always has S_n = V_in/L, so each slope is computed from
// the BuckBoost DAC setting at the corresponding design-point V_in.
const SLOPE_BUCK: f64 = DAC_BUCK.dac_slope / CS_GAIN;
const SLOPE_BB: f64 = DAC_BB.dac_slope / CS_GAIN;
const SLOPE_BOOST: f64 = DAC_BOOST.dac_slope / CS_GAIN;

// ── Mode ──────────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq)]
enum Mode {
    /// V_in significantly above V_out — gain-scheduled for high step-down.
    Buck,
    /// V_in near V_out — gains designed for unity-gain operating point.
    BuckBoost,
    /// V_in significantly below V_out — gain-scheduled for step-up.
    Boost,
}

impl Mode {
    fn name(self) -> &'static str {
        match self {
            Mode::Buck => "Buck     ",
            Mode::BuckBoost => "BuckBoost",
            Mode::Boost => "Boost    ",
        }
    }
}

/// Mode selector with hysteresis.
///
/// The BuckBoost gain region spans ±DELTA around the V_in/V_out unity-ratio
/// point.  A second boundary at ±2×DELTA provides the hysteresis: once in a
/// pure-Buck or pure-Boost gain region, the mode does not switch back until
/// the ratio crosses the inner BuckBoost boundary.
fn select_mode(v_in: f64, v_out: f64, current: Mode) -> Mode {
    if v_out < 0.5 {
        return Mode::BuckBoost; // startup guard
    }
    let ratio = v_in / v_out;
    const DELTA: f64 = 0.20;

    // Hard outer boundaries — switch regardless of current mode
    if ratio > 1.0 + 2.0 * DELTA {
        return Mode::Buck;
    }
    if ratio < 1.0 - 2.0 * DELTA {
        return Mode::Boost;
    }
    // Inner BuckBoost band — always use BuckBoost gains here
    if ratio >= 1.0 - DELTA && ratio <= 1.0 + DELTA {
        return Mode::BuckBoost;
    }
    // Soft hysteresis zone between the two boundaries — keep current mode
    current
}

// ── Controller ────────────────────────────────────────────────────────────────

struct BuckBoostController {
    buck: TwoPoleTwoZero<f32>,
    boost: TwoPoleTwoZero<f32>,
    buck_boost: TwoPoleTwoZero<f32>,
    mode: Mode,
    out_max: f32,
}

impl BuckBoostController {
    fn new(out_max: f32) -> Self {
        Self {
            buck: WEIGHTS_BUCK.to_controller(),
            boost: WEIGHTS_BOOST.to_controller(),
            buck_boost: WEIGHTS_BB.to_controller(),
            mode: Mode::BuckBoost,
            out_max,
        }
    }

    /// Returns `(output_volts, mode, slope_a_per_s, clamped)`.
    fn update(&mut self, v_in: f64, v_out: f64, target: f64) -> (f32, Mode, f64, bool) {
        let error = (target - v_out) as f32;

        let desired = select_mode(v_in, v_out, self.mode);
        if desired != self.mode {
            self.switch_to(desired, error);
        }

        let raw: f32 = match self.mode {
            Mode::Buck => self.buck.update(error),
            Mode::Boost => self.boost.update(error),
            Mode::BuckBoost => self.buck_boost.update(error),
        };

        let clamped_val = raw.clamp(0.0, self.out_max);
        let was_clamped = clamped_val != raw;

        // Clamped-feedback anti-windup: replace the stored raw output with the
        // saturated value so the embedded integrator cannot wind beyond the limits.
        if was_clamped {
            match self.mode {
                Mode::Buck => self.buck.set_last_output(clamped_val),
                Mode::Boost => self.boost.set_last_output(clamped_val),
                Mode::BuckBoost => self.buck_boost.set_last_output(clamped_val),
            }
        }

        let slope = match self.mode {
            Mode::Buck => SLOPE_BUCK,
            Mode::Boost => SLOPE_BOOST,
            Mode::BuckBoost => SLOPE_BB,
        };

        (clamped_val, self.mode, slope, was_clamped)
    }

    fn switch_to(&mut self, new_mode: Mode, current_error: f32) {
        let last_u = match self.mode {
            Mode::Buck => self.buck.last_output(),
            Mode::Boost => self.boost.last_output(),
            Mode::BuckBoost => self.buck_boost.last_output(),
        };
        match new_mode {
            Mode::Buck => self.buck.prime(last_u, current_error),
            Mode::Boost => self.boost.prime(last_u, current_error),
            Mode::BuckBoost => self.buck_boost.prime(last_u, current_error),
        }
        self.mode = new_mode;
    }
}

// ── Simulation sync ───────────────────────────────────────────────────────────

/// Update the simulator before each tick().
///
/// Synchronise the simulator's topology and slope to match the current control mode.
///
/// The physics model in `lib.rs` correctly implements all three topologies:
///   - Buck ON:       V_L = V_in − V_out  (RLC)
///   - BuckBoost ON:  V_L = V_in          (linear, capacitor decoupled)
///   - Boost ON:      V_L = V_in          (linear, capacitor decoupled)
fn sync_sim(sim: &mut CurrentModeConverter, slope: f64, mode: Mode) {
    sim.topology = match mode {
        Mode::Buck => Topology::Buck,
        Mode::BuckBoost => Topology::BuckBoost,
        Mode::Boost => Topology::Boost,
    };
    sim.slope_amp_per_sec = slope;
}

// ── V_in sweep profile ────────────────────────────────────────────────────────

/// 0..25 %  : 24 V  (steady, Buck region)
/// 25..75 % : 24 V → 8 V → 24 V  (cosine, exercises BuckBoost and Boost)
/// 75..100% : 24 V  (steady, Buck region)
///
/// Cosine chosen so V_in is continuous and differentiable at the endpoints.
fn v_in_sweep(step: usize, total: usize) -> f64 {
    let frac = step as f64 / total as f64;
    if frac < 0.25 || frac > 0.75 {
        24.0
    } else {
        let t = (frac - 0.25) / 0.5; // 0..1 over the active sweep portion
        // t=0 → 24V,  t=0.5 → 8V,  t=1 → 24V
        16.0 + 8.0 * f64::cos(2.0 * std::f64::consts::PI * t)
    }
}

// ── Logger ────────────────────────────────────────────────────────────────────

struct Logger {
    #[cfg(not(feature = "text-log"))]
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
            #[cfg(not(feature = "text-log"))]
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
            "{:>9} {:>6} {:>7} {:>7} {:>9} {:>7} {:>5} {:>7} {}",
            "t[ms]", "Vin[V]", "Vout[V]", "err[V]", "mode", "IL[A]", "d[%]", "trip[A]", "clamp"
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
        trip_a: f64,
        mode: Mode,
        was_clamped: bool,
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

        #[cfg(not(feature = "text-log"))]
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
                    "{:9.4} {:6.3} {:7.4} {:+7.4} {:9} {:7.4} {:5.1} {:7.4} {}",
                    time.0 * 1e3,
                    v_in.0,
                    sim.v_out.0,
                    err,
                    mode.name(),
                    sim.i_inductor.0,
                    d_pct,
                    trip_a,
                    if was_clamped { "CLAMP" } else { "" },
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
    // Start the sim in BuckBoost topology; sync_sim() will update it each cycle.
    let mut sim =
        CurrentModeConverter::new(T_PERIOD, C_OUT, L_INDUCTOR, SLOPE_BB, Topology::BuckBoost, R_SERIES, R_ESR, C_IN, R_IN, L_IN);

    let mut ctrl = BuckBoostController::new(MAX_CURRENT.0 as f32 * CS_GAIN as f32);
    let mut logger = Logger::new();
    let mut time = Time(0.0);

    // ── Soft-start ────────────────────────────────────────────────────────────
    // Ramp the voltage reference 0 → V_TARGET over 3000 cycles with V_in at
    // 24 V (well inside the Buck gain-schedule region).
    let soft_cycles = 3000_usize;
    let v_in_startup = Voltage(24.0);

    let vins = [24.0, 12.0, 8.0];

    let mut ctrl_out = (0.0f32, Mode::BuckBoost, SLOPE_BB, false);
    let mut i_out = Current(0.0);
    for i in 0..soft_cycles {
        let soft_target = V_TARGET.0 * (i + 1) as f64 / soft_cycles as f64;
        if i % CYCLES_PER_TICK == 0 {
            ctrl_out = ctrl.update(v_in_startup.0, sim.v_sensed(i_out).0, soft_target);
        }
        let (cmd_v, mode, slope, clamped) = ctrl_out;
        sync_sim(&mut sim, slope, mode);

        let trip = Current((cmd_v as f64 / CS_GAIN).clamp(0.0, MAX_CURRENT.0));
        let (t_on, i_max) = sim.tick(v_in_startup, trip, |v| {
            i_out = Current(v.0 / R_LOAD);
            i_out
        });

        logger.log(
            &sim,
            t_on,
            i_max,
            v_in_startup,
            i_out,
            trip.0,
            mode,
            clamped,
            &mut time,
        );
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
        let mut ctrl_out = (0.0f32, Mode::BuckBoost, SLOPE_BB, false);
        let mut i_out = Current(0.0);
        for i in 0..bat_test_cycles {
            if i % CYCLES_PER_TICK == 0 {
                ctrl_out = ctrl.update(v_in.0, sim.v_sensed(i_out).0, V_TARGET.0);
            }
            let (cmd_v, mode, slope, clamped) = ctrl_out;
            sync_sim(&mut sim, slope, mode);

            let trip = Current((cmd_v as f64 / CS_GAIN).clamp(0.0, MAX_CURRENT.0));
            let (t_on, i_max) = sim.tick(v_in, trip, |v| {
                i_out = bat.tick(v, T_PERIOD);
                i_out
            });

            logger.log(
                &sim, t_on, i_max, v_in, i_out, trip.0, mode, clamped, &mut time,
            );
        }
    }

    let rs = [12.0, 6.0, f64::MAX];

    let mut ctrl_out = (0.0f32, Mode::BuckBoost, SLOPE_BB, false);
    let mut i_out = Current(0.0);
    for i in 0..total_cycles {
        let v_in = Voltage(v_in_sweep(i % sweep_cycles, sweep_cycles));
        let r = rs[i * rs.len() / total_cycles];

        if i % CYCLES_PER_TICK == 0 {
            ctrl_out = ctrl.update(v_in.0, sim.v_sensed(i_out).0, V_TARGET.0);
        }
        let (cmd_v, mode, slope, clamped) = ctrl_out;
        sync_sim(&mut sim, slope, mode);

        let trip = Current((cmd_v as f64 / CS_GAIN).clamp(0.0, MAX_CURRENT.0));
        let (t_on, i_max) = sim.tick(v_in, trip, |v| {
            i_out = Current(v.0 / r);
            i_out
        });

        logger.log(
            &sim, t_on, i_max, v_in, i_out, trip.0, mode, clamped, &mut time,
        );
    }

    for v_in in vins {
        let v_in = Voltage(v_in);
        let mut ctrl_out = (0.0f32, Mode::BuckBoost, SLOPE_BB, false);
        let mut i_out = Current(0.0);
        for i in 0..sweep_cycles {
            let r = rs[i * rs.len() / sweep_cycles];

            if i % CYCLES_PER_TICK == 0 {
                ctrl_out = ctrl.update(v_in.0, sim.v_sensed(i_out).0, V_TARGET.0);
            }
            let (cmd_v, mode, slope, clamped) = ctrl_out;
            sync_sim(&mut sim, slope, mode);

            let trip = Current((cmd_v as f64 / CS_GAIN).clamp(0.0, MAX_CURRENT.0));
            let (t_on, i_max) = sim.tick(v_in, trip, |v| {
                i_out = Current(v.0 / r);
                i_out
            });

            logger.log(
                &sim, t_on, i_max, v_in, i_out, trip.0, mode, clamped, &mut time,
            );
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
            c: Capacitance(1e-1),
        }
    }

    pub fn tick(&mut self, v_in: Voltage, dt: Time) -> Current {
        let q_out = 0.0;
        let f = electronics_sim::math::rlc(v_in, self.v_int, self.i, self.l_cable, self.c, self.r_esr);
        let q_in = f
            .integral(0.0)
            .f(dt.0);
        self.v_int += Voltage((q_in - q_out) / self.c.0);
        self.i = Current(f.f(dt.0));

        self.i
    }
}
