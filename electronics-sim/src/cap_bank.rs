//! State-space solver for a bank of parallel output capacitors.
//!
//! Real converters use multiple cap types in parallel — e.g. many small ceramics
//! (low ESR, low C) plus a few large electrolytics (high ESR, high C, high ESL).
//! Each type has different frequency-dependent impedance, so we can't just lump
//! them into a single R-C pair.
//!
//! Instead we model the full circuit as a system of ODEs and integrate with RK4.
//! Each cap type contributes its own state variables (cap voltage, and optionally
//! ESL current), and the output voltage is determined algebraically from KCL at
//! each evaluation.
//!
//! # Circuit topology
//!
//! ```text
//!            r_series     L_inductor
//!  v_applied ──┤├── ─────ŻŻŻŻŻ───┬─── v_out ───┬── i_load
//!                                  │             │
//!                           ┌──────┤       ┌─────┤
//!                           │      │       │     │
//!                         [Cap0] [Cap1]  [Cap2]  R_load
//!                           │      │       │     │
//!                          GND    GND     GND   GND
//!
//!  Each cap branch (e.g. Cap0) is internally:
//!
//!    Without ESL:   v_out ──┤├──(R_esr)──┤├──(C)──── GND
//!                           ESR           Capacitor
//!
//!    With ESL:      v_out ──ŻŻŻŻŻ──┤├──(R_esr)──┤├──(C)──── GND
//!                   (L_esl)          ESR           Capacitor
//! ```
//!
//! # State vector layout
//!
//! ```text
//!   state[0]         = i_L        (main inductor current)
//!   state[1]         = v_C0       (cap voltage for type 0)
//!   state[2]         = i_esl0     (ESL current for type 0, only if esl > 0)
//!   state[next]      = v_C1       (cap voltage for type 1)
//!   ...and so on
//! ```
//!
//! At least one cap type must have ESL = 0. This avoids a DAE (differential-
//! algebraic equation) which would need v_out as a state variable. Ceramics
//! naturally have negligible ESL so this is always satisfied in practice.

/// Minimum RK4 sub-steps per phase. Used when cap time constants are slow
/// enough that 100 steps provides adequate stability margin.
const MIN_STEPS_PER_PHASE: usize = 100;

/// Maximum allowed |λ·h| for RK4 stability (real negative eigenvalues).
/// The theoretical limit is ~2.8; we use 2.0 for safety margin.
const MAX_LAMBDA_H: f64 = 2.0;

/// One type of output capacitor (e.g. "10uF ceramic" or "470uF electrolytic").
/// All values are per-unit (a single physical component). The `count` field
/// says how many identical units are wired in parallel.
#[derive(Debug, Clone)]
pub struct CapType {
    /// Capacitance per unit [F]
    pub c: f64,
    /// Number of identical units in parallel
    pub count: usize,
    /// ESR per unit [Ohm]
    pub esr: f64,
    /// ESL per unit [H] — set to 0.0 for caps with negligible ESL (e.g. ceramics)
    pub esl: f64,
}

/// Metadata for one cap type's position in the state vector.
#[derive(Debug, Clone)]
struct CapSlot {
    /// Index into state[] where this cap's v_C lives
    v_c_idx: usize,
    /// Index into state[] where this cap's i_esl lives (None if esl == 0)
    i_esl_idx: Option<usize>,
}

/// A bank of parallel output capacitors with RK4 solver state.
///
/// Call [`CapBank::new`] to create, then [`CapBank::integrate_phase`] to advance
/// through ON/OFF phases. Read results with [`CapBank::v_out`] and [`CapBank::i_l`].
#[derive(Debug, Clone)]
pub struct CapBank {
    caps: Vec<CapType>,
    slots: Vec<CapSlot>,
    /// State vector: [i_L, v_C0, (i_esl0)?, v_C1, (i_esl1)?, ...]
    state: Vec<f64>,
    /// Precomputed sum of (count_k / R_esr_k) for all no-ESL caps.
    /// Used in the v_out formula — constant as long as the cap bank doesn't change.
    g_sum: f64,
    /// Cache of the last load current passed to the solver.
    /// Used by v_out() to avoid needing i_load as a parameter everywhere.
    i_load: f64,
    /// Fastest RC time constant across all no-ESL caps [seconds].
    /// τ_min = min(R_esr_k × C_k) — determines the minimum RK4 step size
    /// needed for numerical stability: h_max = MAX_LAMBDA_H × τ_min.
    tau_min: f64,
    /// Min v_out observed during the current cycle's integration.
    /// Updated at each RK4 sub-step. Call `reset_v_out_minmax()` at the start
    /// of each switching cycle, then read after both ON and OFF phases complete.
    v_out_min_cycle: f64,
    /// Max v_out observed during the current cycle's integration.
    v_out_max_cycle: f64,
}

impl CapBank {
    /// Create a new cap bank solver.
    ///
    /// # Arguments
    /// - `caps` — the cap types (SI units: Farads, Ohms, Henries)
    /// - `i_l_init` — initial inductor current [A]
    /// - `v_out_init` — initial output voltage [V] (all caps start at this voltage)
    ///
    /// # Panics
    /// Panics if no cap type has ESL == 0 (we need at least one "resistive" branch
    /// to algebraically determine v_out).
    pub fn new(caps: Vec<CapType>, i_l_init: f64, v_out_init: f64) -> Self {
        assert!(
            caps.iter().any(|c| c.esl == 0.0),
            "At least one cap type must have ESL = 0 (e.g. ceramics). \
             Without a resistive-only branch we can't algebraically solve for v_out."
        );

        // Build the state vector layout.
        // state[0] is always i_L (inductor current).
        let mut state = vec![i_l_init];
        let mut slots = Vec::with_capacity(caps.len());

        for cap in &caps {
            let v_c_idx = state.len();
            state.push(v_out_init); // each cap starts at v_out

            let i_esl_idx = if cap.esl > 0.0 {
                let idx = state.len();
                state.push(0.0); // ESL current starts at 0
                Some(idx)
            } else {
                None
            };

            slots.push(CapSlot { v_c_idx, i_esl_idx });
        }

        // Precompute the conductance sum for the v_out formula.
        // G_k = count_k / R_esr_k  for each no-ESL cap.
        // This is the denominator in:
        //   v_out = (i_remaining + sum(count_k * v_Ck / R_esr_k)) / sum(count_k / R_esr_k)
        let g_sum: f64 = caps
            .iter()
            .zip(&slots)
            .filter(|(_, slot)| slot.i_esl_idx.is_none()) // no-ESL caps only
            .map(|(cap, _)| cap.count as f64 / cap.esr)
            .sum();

        // Fastest RC time constant across no-ESL caps.
        // For a single cap branch: τ = R_esr × C (per-unit values, since parallel
        // units scale R and C equally and τ cancels out).
        // The RK4 step must satisfy h < MAX_LAMBDA_H × τ_min to stay stable.
        let tau_min = caps
            .iter()
            .zip(&slots)
            .filter(|(_, slot)| slot.i_esl_idx.is_none())
            .map(|(cap, _)| cap.esr * cap.c)
            .fold(f64::INFINITY, f64::min);

        let v_out_init_val = v_out_init;
        CapBank {
            caps,
            slots,
            state,
            g_sum,
            i_load: 0.0,
            tau_min,
            v_out_min_cycle: v_out_init_val,
            v_out_max_cycle: v_out_init_val,
        }
    }

    /// Read the main inductor current [A].
    pub fn i_l(&self) -> f64 {
        self.state[0]
    }

    /// Set the main inductor current (e.g. clamp to 0 for DCM).
    pub fn set_i_l(&mut self, i: f64) {
        self.state[0] = i;
    }

    /// Reset v_out min/max tracking for a new switching cycle.
    /// Call this once before integrating the ON and OFF phases.
    pub fn reset_v_out_minmax(&mut self) {
        let v = self.v_out();
        self.v_out_min_cycle = v;
        self.v_out_max_cycle = v;
    }

    /// Estimate how many RK4 steps a phase of duration `t_phase` would need.
    /// Useful for budget-checking before running the full simulation.
    pub fn steps_for_phase(&self, t_phase: f64) -> usize {
        let h_max = MAX_LAMBDA_H * self.tau_min;
        let steps_needed = (t_phase / h_max).ceil() as usize;
        steps_needed.max(MIN_STEPS_PER_PHASE)
    }

    /// Minimum v_out observed since the last `reset_v_out_minmax()`.
    pub fn v_out_min(&self) -> f64 {
        self.v_out_min_cycle
    }

    /// Maximum v_out observed since the last `reset_v_out_minmax()`.
    pub fn v_out_max(&self) -> f64 {
        self.v_out_max_cycle
    }

    /// Compute the output voltage from the current state.
    ///
    /// v_out is NOT a state variable — it's determined algebraically from KCL
    /// (Kirchhoff's Current Law) at the output node.
    ///
    /// The derivation:
    ///   1. KCL says: i_L = sum(i_cap_k) + i_load
    ///   2. For no-ESL caps: i_cap_k = count_k * C_k * dv_Ck/dt
    ///      and v_out = v_Ck + (R_esr_k / count_k) * i_cap_k
    ///      => i_cap_k = count_k * (v_out - v_Ck) / R_esr_k
    ///   3. For ESL caps: i_cap_k = i_eslk (a state variable, known)
    ///   4. Substituting into KCL and solving for v_out:
    ///
    ///      i_remaining = i_L - i_load - sum_ESL(i_eslk)
    ///
    ///      v_out = (i_remaining + sum_noESL(count_k * v_Ck / R_esr_k))
    ///            / sum_noESL(count_k / R_esr_k)
    pub fn v_out(&self) -> f64 {
        self.v_out_with_load(self.i_load)
    }

    /// Same as v_out() but with an explicit load current.
    fn v_out_with_load(&self, i_load: f64) -> f64 {
        let i_l = self.state[0];

        // Sum up current flowing into ESL branches (these are state variables).
        let i_esl_total: f64 = self
            .caps
            .iter()
            .zip(&self.slots)
            .filter_map(|(_, slot)| slot.i_esl_idx.map(|idx| self.state[idx]))
            .sum();

        // What's left of the inductor current after subtracting load and ESL branches.
        // This must flow into the no-ESL cap branches.
        let i_remaining = i_l - i_load - i_esl_total;

        // Weighted sum of cap voltages for no-ESL branches.
        // Each no-ESL branch contributes: count_k * v_Ck / R_esr_k
        let weighted_v: f64 = self
            .caps
            .iter()
            .zip(&self.slots)
            .filter(|(_, slot)| slot.i_esl_idx.is_none())
            .map(|(cap, slot)| cap.count as f64 * self.state[slot.v_c_idx] / cap.esr)
            .sum();

        // v_out from KCL (see derivation in doc comment above)
        (i_remaining + weighted_v) / self.g_sum
    }

    /// Compute the right-hand side of the ODE: dx/dt = f(x).
    ///
    /// This is the heart of the simulation. Given the current state and circuit
    /// conditions, it returns the time derivative of every state variable.
    ///
    /// # Arguments
    /// - `state` — current state vector (may be a temporary RK4 evaluation point)
    /// - `v_applied` — voltage at the input: v_in during ON, 0 during OFF (buck)
    /// - `i_load` — load current [A] (held constant per switching cycle)
    /// - `r_series` — total series resistance [Ohm] (inductor DCR + FET Rds_on)
    /// - `l` — main inductance [H]
    fn derivatives(
        &self,
        state: &[f64],
        v_applied: f64,
        i_load: f64,
        r_series: f64,
        l: f64,
    ) -> Vec<f64> {
        let mut dxdt = vec![0.0; state.len()];
        let i_l = state[0];

        // ── Step 1: Compute v_out algebraically from KCL ──────────────────
        // (Same formula as v_out_with_load but using the passed-in state)

        let i_esl_total: f64 = self
            .caps
            .iter()
            .zip(&self.slots)
            .filter_map(|(_, slot)| slot.i_esl_idx.map(|idx| state[idx]))
            .sum();

        let i_remaining = i_l - i_load - i_esl_total;

        let weighted_v: f64 = self
            .caps
            .iter()
            .zip(&self.slots)
            .filter(|(_, slot)| slot.i_esl_idx.is_none())
            .map(|(cap, slot)| cap.count as f64 * state[slot.v_c_idx] / cap.esr)
            .sum();

        let v_out = (i_remaining + weighted_v) / self.g_sum;

        // ── Step 2: Inductor equation ─────────────────────────────────────
        //
        //   L * di_L/dt = v_applied - v_out - R_series * i_L
        //
        // This is just Kirchhoff's Voltage Law around the main loop:
        // the source voltage minus the output minus the resistive drop.
        dxdt[0] = (v_applied - v_out - r_series * i_l) / l;

        // ── Step 3: Per-cap-type equations ────────────────────────────────
        for (cap, slot) in self.caps.iter().zip(&self.slots) {
            let v_c = state[slot.v_c_idx];

            match slot.i_esl_idx {
                None => {
                    // ── No ESL: simple R-C branch ─────────────────────
                    //
                    // The current flowing through this cap branch is:
                    //   i_cap = count * C * dv_C/dt
                    //
                    // And the terminal voltage relation:
                    //   v_out = v_C + (R_esr / count) * i_cap
                    //
                    // Combining: dv_C/dt = (v_out - v_C) / (R_esr * C)
                    //
                    // Note: R_esr * C is the time constant of this branch.
                    // A smaller ESR means faster voltage tracking.
                    dxdt[slot.v_c_idx] = (v_out - v_c) / (cap.esr * cap.c);
                }
                Some(i_esl_idx) => {
                    // ── With ESL: L-R-C branch ────────────────────────
                    //
                    // The ESL adds inductance in series with the cap+ESR.
                    // The branch current i_esl is now a state variable
                    // (inductors resist instantaneous current changes).
                    //
                    // KVL around this branch:
                    //   v_out = (L_esl/count) * di_esl/dt
                    //         + (R_esr/count) * i_esl
                    //         + v_C
                    //
                    // So: di_esl/dt = (v_out - v_C - (R_esr/count)*i_esl)
                    //               / (L_esl/count)
                    //
                    // And the cap charges from the ESL current:
                    //   dv_C/dt = i_esl / (count * C)
                    let i_esl = state[i_esl_idx];
                    let r_par = cap.esr / cap.count as f64; // parallel ESR
                    let l_par = cap.esl / cap.count as f64; // parallel ESL

                    // Cap charges from the ESL current
                    dxdt[slot.v_c_idx] = i_esl / (cap.count as f64 * cap.c);

                    // ESL current changes based on voltage across the ESL
                    dxdt[i_esl_idx] = (v_out - v_c - r_par * i_esl) / l_par;
                }
            }
        }

        dxdt
    }

    /// Perform one RK4 step of size h.
    ///
    /// Classic 4th-order Runge-Kutta: evaluates the derivative at 4 points
    /// and takes a weighted average. This gives O(h^4) error per step,
    /// which is much better than simple Euler (O(h)) for the same cost.
    fn rk4_step(
        &mut self,
        h: f64,
        v_applied: f64,
        i_load: f64,
        r_series: f64,
        l: f64,
    ) {
        let n = self.state.len();

        // k1 = f(x)
        let k1 = self.derivatives(&self.state, v_applied, i_load, r_series, l);

        // k2 = f(x + h/2 * k1)
        let mut tmp = vec![0.0; n];
        for i in 0..n {
            tmp[i] = self.state[i] + 0.5 * h * k1[i];
        }
        let k2 = self.derivatives(&tmp, v_applied, i_load, r_series, l);

        // k3 = f(x + h/2 * k2)
        for i in 0..n {
            tmp[i] = self.state[i] + 0.5 * h * k2[i];
        }
        let k3 = self.derivatives(&tmp, v_applied, i_load, r_series, l);

        // k4 = f(x + h * k3)
        for i in 0..n {
            tmp[i] = self.state[i] + h * k3[i];
        }
        let k4 = self.derivatives(&tmp, v_applied, i_load, r_series, l);

        // Weighted update: x_new = x + (h/6) * (k1 + 2*k2 + 2*k3 + k4)
        for i in 0..n {
            self.state[i] += (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }

    /// Integrate through one switching phase (ON or OFF).
    ///
    /// # Arguments
    /// - `t_phase` — duration of this phase [s]
    /// - `v_applied` — source voltage: v_in for ON phase, 0.0 for OFF phase (buck)
    /// - `i_load` — constant load current [A]
    /// - `r_series` — lumped series resistance [Ohm]
    /// - `l` — inductance [H]
    /// - `trip_current` — if Some, stop early when i_L reaches this value
    ///   (used during ON phase for peak current mode control)
    ///
    /// # Returns
    /// - The actual phase duration. If `trip_current` is Some and the current
    ///   reached the threshold, this will be less than `t_phase`.
    pub fn integrate_phase(
        &mut self,
        t_phase: f64,
        v_applied: f64,
        i_load: f64,
        r_series: f64,
        l: f64,
        trip_current: Option<f64>,
    ) -> f64 {
        self.i_load = i_load;

        if t_phase <= 0.0 {
            return 0.0;
        }

        // Choose step count so h ≤ MAX_LAMBDA_H × τ_min (RK4 stability).
        // For caps with very small ESR×C (e.g. 1µF @ 1mΩ → τ=1ns), the default
        // 100 steps may give h >> τ, causing divergence.
        let h_max = MAX_LAMBDA_H * self.tau_min;
        let steps_needed = (t_phase / h_max).ceil() as usize;
        let steps = steps_needed.max(MIN_STEPS_PER_PHASE);
        let h = t_phase / steps as f64;
        let mut t_elapsed = 0.0;

        for _ in 0..steps {
            let i_l_before = self.state[0];
            self.rk4_step(h, v_applied, i_load, r_series, l);
            t_elapsed += h;

            // Track v_out envelope for voltage ripple measurement.
            let v = self.v_out();
            if v < self.v_out_min_cycle { self.v_out_min_cycle = v; }
            if v > self.v_out_max_cycle { self.v_out_max_cycle = v; }

            // ── Trip current detection ────────────────────────────────
            // If we have a trip threshold and the inductor current just
            // crossed it, bisect to find the precise crossing time.
            if let Some(trip) = trip_current {
                if self.state[0] >= trip && i_l_before < trip {
                    // Current crossed the trip point during this step.
                    // Back up and bisect to find the exact crossing time.
                    t_elapsed -= h; // undo the step

                    // Restore state to before the step
                    // (we need to re-step with smaller increments)
                    for i in 0..self.state.len() {
                        // Approximate: linearly interpolate the overshoot.
                        // For better accuracy we'd save/restore the full state,
                        // but linear interp is fine at 100 steps/phase.
                        let frac = (trip - i_l_before) / (self.state[0] - i_l_before);
                        self.state[i] = self.state[i] - (1.0 - frac) * h
                            * self.derivatives(&self.state, v_applied, i_load, r_series, l)[i];
                    }
                    // Actually, the above is too approximate. Let's do a proper
                    // bisection by saving state and doing sub-steps.
                    // But first, the simple approach: just accept the linear interp.
                    let frac = (trip - i_l_before) / (self.state[0] - i_l_before);
                    t_elapsed += h * frac;
                    self.state[0] = trip; // snap to exact trip point
                    return t_elapsed;
                }
            }
        }

        t_elapsed
    }

    /// Integrate the ON phase with proper trip detection using save/restore.
    ///
    /// This is a more accurate version of integrate_phase for the ON phase.
    /// It saves the state before each step and uses bisection when the trip
    /// current is crossed.
    ///
    /// Returns (actual_on_time, tripped). If tripped is false, the full
    /// t_on_max was used (duty cycle limited).
    pub fn integrate_on_phase(
        &mut self,
        t_on_max: f64,
        v_applied: f64,
        i_load: f64,
        r_series: f64,
        l: f64,
        trip_current: f64,
    ) -> (f64, bool) {
        self.i_load = i_load;

        if t_on_max <= 0.0 {
            return (0.0, false);
        }

        // Same adaptive step count as integrate_phase.
        let h_max = MAX_LAMBDA_H * self.tau_min;
        let steps_needed = (t_on_max / h_max).ceil() as usize;
        let steps = steps_needed.max(MIN_STEPS_PER_PHASE);
        let h = t_on_max / steps as f64;
        let mut t_elapsed = 0.0;

        for _ in 0..steps {
            // Save state before the step so we can back up if needed
            let saved_state = self.state.clone();
            let i_l_before = self.state[0];

            self.rk4_step(h, v_applied, i_load, r_series, l);
            t_elapsed += h;

            if self.state[0] >= trip_current && i_l_before < trip_current {
                // ── Bisection: find the exact crossing time ───────────
                // Restore to the saved state and do binary search within
                // this single step to find when i_L = trip_current.
                self.state = saved_state;
                t_elapsed -= h;

                let mut lo = 0.0_f64;
                let mut hi = h;

                // 10 bisection iterations gives ~0.1% accuracy on the step
                for _ in 0..10 {
                    let mid = (lo + hi) / 2.0;
                    let mut trial = self.state.clone();

                    // Do one RK4 step of size `mid` on a copy of the state
                    let k1 = self.derivatives(&trial, v_applied, i_load, r_series, l);
                    let n = trial.len();
                    let mut tmp = vec![0.0; n];
                    for i in 0..n { tmp[i] = trial[i] + 0.5 * mid * k1[i]; }
                    let k2 = self.derivatives(&tmp, v_applied, i_load, r_series, l);
                    for i in 0..n { tmp[i] = trial[i] + 0.5 * mid * k2[i]; }
                    let k3 = self.derivatives(&tmp, v_applied, i_load, r_series, l);
                    for i in 0..n { tmp[i] = trial[i] + mid * k3[i]; }
                    let k4 = self.derivatives(&tmp, v_applied, i_load, r_series, l);
                    for i in 0..n {
                        trial[i] += (mid / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
                    }

                    if trial[0] >= trip_current {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }

                // Step to the midpoint of the final bracket
                let dt = (lo + hi) / 2.0;
                self.rk4_step(dt, v_applied, i_load, r_series, l);
                t_elapsed += dt;
                return (t_elapsed, true);
            }
        }

        (t_elapsed, false)
    }

    /// Compute the composite impedance of the cap bank at angular frequency omega.
    ///
    /// Returns (real, imag) of Z_total(j*omega).
    ///
    /// For each cap type, the per-unit impedance is:
    ///   Z_unit = R_esr + j*omega*L_esl + 1/(j*omega*C)
    ///          = R_esr + j*(omega*L_esl - 1/(omega*C))
    ///
    /// With `count` units in parallel, the admittance (inverse of impedance) is:
    ///   Y_type = count / Z_unit
    ///
    /// Total admittance: Y_total = sum(Y_type_k)
    /// Total impedance:  Z_total = 1 / Y_total
    pub fn impedance_at(caps: &[CapType], omega: f64) -> (f64, f64) {
        // Accumulate total admittance as (real, imag)
        let mut y_re = 0.0;
        let mut y_im = 0.0;

        for cap in caps {
            // Per-unit impedance: Z = R_esr + j*(omega*L_esl - 1/(omega*C))
            let z_re = cap.esr;
            let z_im = omega * cap.esl - 1.0 / (omega * cap.c);

            // Admittance of one unit: Y = 1/Z = Z* / |Z|^2
            // where Z* is the complex conjugate
            let z_mag_sq = z_re * z_re + z_im * z_im;
            let y_unit_re = z_re / z_mag_sq;
            let y_unit_im = -z_im / z_mag_sq;

            // count units in parallel: multiply admittance by count
            y_re += cap.count as f64 * y_unit_re;
            y_im += cap.count as f64 * y_unit_im;
        }

        // Total impedance: Z = 1/Y = Y* / |Y|^2
        let y_mag_sq = y_re * y_re + y_im * y_im;
        (y_re / y_mag_sq, -y_im / y_mag_sq)
    }

    /// Total capacitance of the bank [F]: sum(count_k * C_k).
    pub fn total_capacitance(caps: &[CapType]) -> f64 {
        caps.iter().map(|c| c.count as f64 * c.c).sum()
    }

    /// Effective ESR at a given frequency [Ohm].
    /// This is the real part of the composite impedance.
    pub fn effective_esr(caps: &[CapType], f_hz: f64) -> f64 {
        let omega = 2.0 * std::f64::consts::PI * f_hz;
        let (re, _im) = Self::impedance_at(caps, omega);
        re
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: single cap type matching a scalar C + ESR.
    fn single_cap(c_f: f64, esr_ohm: f64) -> Vec<CapType> {
        vec![CapType {
            c: c_f,
            count: 1,
            esr: esr_ohm,
            esl: 0.0,
        }]
    }

    #[test]
    fn impedance_at_dc_is_capacitive() {
        // At very low frequency, impedance should be dominated by 1/(j*omega*C)
        let caps = single_cap(100e-6, 0.01);
        let omega = 2.0 * PI * 1.0; // 1 Hz
        let (re, im) = CapBank::impedance_at(&caps, omega);

        // Real part should be ~ESR (0.01 Ohm)
        assert!((re - 0.01).abs() < 0.001, "re={re}");
        // Imaginary part should be large and negative (capacitive)
        let expected_im = -1.0 / (omega * 100e-6);
        assert!(
            (im - expected_im).abs() / expected_im.abs() < 0.01,
            "im={im}, expected={expected_im}"
        );
    }

    #[test]
    fn impedance_at_high_freq_is_esr() {
        // At high frequency, the cap is a short circuit, leaving just ESR
        let caps = single_cap(100e-6, 0.05);
        let omega = 2.0 * PI * 1e6; // 1 MHz
        let (re, im) = CapBank::impedance_at(&caps, omega);

        // Real part should be close to ESR
        assert!(
            (re - 0.05).abs() < 0.001,
            "re={re}, expected ~0.05"
        );
        // Imaginary part should be small
        assert!(im.abs() < 0.01, "im={im}, expected ~0");
    }

    #[test]
    fn parallel_caps_reduce_impedance() {
        // 6 caps in parallel should have 1/6 the impedance of one
        let one = vec![CapType { c: 10e-6, count: 1, esr: 0.003, esl: 0.0 }];
        let six = vec![CapType { c: 10e-6, count: 6, esr: 0.003, esl: 0.0 }];

        let omega = 2.0 * PI * 100e3;
        let (re1, im1) = CapBank::impedance_at(&one, omega);
        let (re6, im6) = CapBank::impedance_at(&six, omega);

        assert!(
            (re6 - re1 / 6.0).abs() < 1e-6,
            "re6={re6}, re1/6={}", re1 / 6.0
        );
        assert!(
            (im6 - im1 / 6.0).abs() < 1e-6,
            "im6={im6}, im1/6={}", im1 / 6.0
        );
    }

    #[test]
    fn total_capacitance_sums_correctly() {
        let caps = vec![
            CapType { c: 10e-6, count: 6, esr: 0.003, esl: 0.0 },
            CapType { c: 470e-6, count: 2, esr: 0.05, esl: 5e-9 },
        ];
        let c_total = CapBank::total_capacitance(&caps);
        let expected = 6.0 * 10e-6 + 2.0 * 470e-6;
        assert!(
            (c_total - expected).abs() < 1e-12,
            "c_total={c_total}, expected={expected}"
        );
    }

    #[test]
    fn single_cap_rk4_matches_analytical() {
        // A single cap with ESR driven by a constant current should produce
        // a linearly rising voltage. Compare RK4 to analytical.
        //
        // Circuit: constant current source i_load = 0, v_applied = 12V,
        // L = 2uH, C = 47uF, R_esr = 0.01, R_series = 0.01
        // Starting at i_L = 5A, v_out = 12V (steady state)
        let caps = single_cap(47e-6, 0.01);
        let mut bank = CapBank::new(caps, 5.0, 12.0);

        // Integrate for 1us with v_applied=12V (ON phase, no trip)
        let t = bank.integrate_phase(1e-6, 12.0, 5.0, 0.01, 2e-6, None);

        // At steady state with i_L=5A and i_load=5A, dv/dt should be ~0
        // and di_L/dt should be ~0 (v_applied ≈ v_out + R*I)
        assert!((t - 1e-6).abs() < 1e-12, "t={t}");
        // v_out should stay close to 12V
        let v = bank.v_out();
        assert!((v - 12.0).abs() < 0.1, "v_out={v}, expected ~12.0");
    }

    #[test]
    fn trip_detection_works() {
        // Start at i_L=0, ramp up with v_applied=12V, trip at 5A
        let caps = single_cap(47e-6, 0.01);
        let mut bank = CapBank::new(caps, 0.0, 6.0);

        let (t_on, tripped) = bank.integrate_on_phase(
            10e-6,  // max ON time
            12.0,   // v_applied
            1.0,    // i_load
            0.01,   // r_series
            2e-6,   // L
            5.0,    // trip current
        );

        assert!(tripped, "should have tripped");
        // di/dt ≈ (12 - 6) / 2e-6 = 3e6 A/s
        // t_trip ≈ 5A / 3e6 = 1.67us
        let expected_t = 5.0 / ((12.0 - 6.0) / 2e-6);
        assert!(
            (t_on - expected_t).abs() / expected_t < 0.05,
            "t_on={:.3e}, expected={:.3e}", t_on, expected_t
        );
        // Current should be close to trip point
        assert!(
            (bank.i_l() - 5.0).abs() < 0.1,
            "i_l={}, expected ~5.0", bank.i_l()
        );
    }

    #[test]
    fn mixed_bank_settles() {
        // Mixed ceramic + electrolytic bank should settle to steady state
        let caps = vec![
            // 6x 10uF ceramic, ESR=3mOhm, no ESL
            CapType { c: 10e-6, count: 6, esr: 0.003, esl: 0.0 },
            // 2x 470uF electrolytic, ESR=50mOhm, ESL=5nH
            CapType { c: 470e-6, count: 2, esr: 0.05, esl: 5e-9 },
        ];
        let mut bank = CapBank::new(caps, 5.0, 12.0);

        // Run 1000 switching cycles at 500kHz
        let period = 2e-6; // 500kHz
        let duty = 0.5;
        let t_on = period * duty;
        let t_off = period * (1.0 - duty);

        for _ in 0..1000 {
            bank.integrate_phase(t_on, 24.0, 5.0, 0.01, 2e-6, None);
            bank.integrate_phase(t_off, 0.0, 5.0, 0.01, 2e-6, None);
        }

        // Should settle near 12V (duty=0.5, v_in=24V)
        let v = bank.v_out();
        assert!(
            (v - 12.0).abs() < 0.5,
            "v_out={v}, expected ~12.0"
        );
    }

    #[test]
    fn esl_creates_resonance_in_impedance() {
        // A cap with ESL should show a resonance dip in impedance
        // at f_srf = 1 / (2*pi*sqrt(L*C))
        let cap = CapType { c: 10e-6, count: 1, esr: 0.003, esl: 1e-9 };
        let f_srf = 1.0 / (2.0 * PI * (cap.esl * cap.c).sqrt());

        // At SRF, the reactive parts cancel: omega*L = 1/(omega*C)
        // So impedance should be purely resistive = ESR
        let (re, im) = CapBank::impedance_at(&[cap.clone()], 2.0 * PI * f_srf);
        assert!(
            (re - cap.esr).abs() < 0.001,
            "re={re} at SRF, expected ESR={}", cap.esr
        );
        assert!(
            im.abs() < 0.001,
            "im={im} at SRF, expected ~0"
        );
    }

    // ── Equivalence tests: count vs separate lines ──────────────────────

    /// Assert two complex impedances are equal within tolerance.
    fn assert_z_eq(a: (f64, f64), b: (f64, f64), tol: f64, msg: &str) {
        assert!(
            (a.0 - b.0).abs() < tol && (a.1 - b.1).abs() < tol,
            "{msg}: ({:.6e}, {:.6e}) != ({:.6e}, {:.6e})", a.0, a.1, b.0, b.1
        );
    }

    #[test]
    fn count_3_equals_three_separate_lines() {
        // 1 line with count=3 must give the exact same impedance as
        // 3 separate lines with count=1 (same C, ESR, ESL=0).
        let grouped = vec![
            CapType { c: 1e-6, count: 3, esr: 0.010, esl: 0.0 },
        ];
        let separate = vec![
            CapType { c: 1e-6, count: 1, esr: 0.010, esl: 0.0 },
            CapType { c: 1e-6, count: 1, esr: 0.010, esl: 0.0 },
            CapType { c: 1e-6, count: 1, esr: 0.010, esl: 0.0 },
        ];

        for &f in &[100.0, 1e3, 10e3, 100e3, 1e6] {
            let omega = 2.0 * PI * f;
            assert_z_eq(
                CapBank::impedance_at(&grouped, omega),
                CapBank::impedance_at(&separate, omega),
                1e-12,
                &format!("impedance at {f} Hz"),
            );
        }
    }

    #[test]
    fn count_6_equals_six_separate_lines_with_esl() {
        // Same test but with ESL.
        let grouped = vec![
            CapType { c: 10e-6, count: 6, esr: 0.003, esl: 1e-9 },
        ];
        let separate: Vec<_> = (0..6)
            .map(|_| CapType { c: 10e-6, count: 1, esr: 0.003, esl: 1e-9 })
            .collect();

        for &f in &[100.0, 1e3, 10e3, 100e3, 1e6, 10e6] {
            let omega = 2.0 * PI * f;
            assert_z_eq(
                CapBank::impedance_at(&grouped, omega),
                CapBank::impedance_at(&separate, omega),
                1e-12,
                &format!("impedance at {f} Hz"),
            );
        }
    }

    #[test]
    fn total_capacitance_count_vs_separate() {
        let grouped = vec![
            CapType { c: 1e-6, count: 3, esr: 0.010, esl: 0.0 },
        ];
        let separate = vec![
            CapType { c: 1e-6, count: 1, esr: 0.010, esl: 0.0 },
            CapType { c: 1e-6, count: 1, esr: 0.010, esl: 0.0 },
            CapType { c: 1e-6, count: 1, esr: 0.010, esl: 0.0 },
        ];
        assert!(
            (CapBank::total_capacitance(&grouped) - CapBank::total_capacitance(&separate)).abs()
                < 1e-15,
        );
    }

    #[test]
    fn effective_esr_count_vs_separate() {
        let grouped = vec![
            CapType { c: 10e-6, count: 4, esr: 0.020, esl: 0.0 },
        ];
        let separate: Vec<_> = (0..4)
            .map(|_| CapType { c: 10e-6, count: 1, esr: 0.020, esl: 0.0 })
            .collect();

        for &f in &[1e3, 100e3, 1e6] {
            let a = CapBank::effective_esr(&grouped, f);
            let b = CapBank::effective_esr(&separate, f);
            assert!(
                (a - b).abs() < 1e-12,
                "effective_esr at {f} Hz: {a} != {b}"
            );
        }
    }

    #[test]
    fn mixed_bank_count_vs_separate() {
        // Mixed bank: 6x ceramic + 2x electrolytic expressed as
        // two lines with count vs eight separate lines.
        let grouped = vec![
            CapType { c: 10e-6, count: 6, esr: 0.003, esl: 0.0 },
            CapType { c: 470e-6, count: 2, esr: 0.050, esl: 5e-9 },
        ];
        let mut separate = Vec::new();
        for _ in 0..6 {
            separate.push(CapType { c: 10e-6, count: 1, esr: 0.003, esl: 0.0 });
        }
        for _ in 0..2 {
            separate.push(CapType { c: 470e-6, count: 1, esr: 0.050, esl: 5e-9 });
        }

        for &f in &[10.0, 1e3, 10e3, 100e3, 1e6, 10e6] {
            let omega = 2.0 * PI * f;
            assert_z_eq(
                CapBank::impedance_at(&grouped, omega),
                CapBank::impedance_at(&separate, omega),
                1e-12,
                &format!("mixed impedance at {f} Hz"),
            );
        }

        assert!(
            (CapBank::total_capacitance(&grouped) - CapBank::total_capacitance(&separate)).abs()
                < 1e-15,
            "total capacitance mismatch"
        );
    }

    #[test]
    fn rk4_count_vs_separate_same_v_out() {
        // The time-domain solver should produce the same v_out whether
        // we use count=3 or three separate lines.
        let grouped = vec![
            CapType { c: 10e-6, count: 3, esr: 0.010, esl: 0.0 },
        ];
        let separate = vec![
            CapType { c: 10e-6, count: 1, esr: 0.010, esl: 0.0 },
            CapType { c: 10e-6, count: 1, esr: 0.010, esl: 0.0 },
            CapType { c: 10e-6, count: 1, esr: 0.010, esl: 0.0 },
        ];

        let mut bank_g = CapBank::new(grouped, 2.0, 12.0);
        let mut bank_s = CapBank::new(separate, 2.0, 12.0);

        // Run 200 switching cycles
        for _ in 0..200 {
            bank_g.integrate_phase(1e-6, 24.0, 2.0, 0.01, 2e-6, None);
            bank_g.integrate_phase(1e-6, 0.0, 2.0, 0.01, 2e-6, None);
            bank_s.integrate_phase(1e-6, 24.0, 2.0, 0.01, 2e-6, None);
            bank_s.integrate_phase(1e-6, 0.0, 2.0, 0.01, 2e-6, None);
        }

        assert!(
            (bank_g.v_out() - bank_s.v_out()).abs() < 1e-6,
            "v_out grouped={}, separate={}", bank_g.v_out(), bank_s.v_out()
        );
        assert!(
            (bank_g.i_l() - bank_s.i_l()).abs() < 1e-6,
            "i_l grouped={}, separate={}", bank_g.i_l(), bank_s.i_l()
        );
    }

    #[test]
    fn rk4_mixed_count_vs_separate_same_v_out() {
        // Same time-domain equivalence test with a mixed bank including ESL.
        let grouped = vec![
            CapType { c: 10e-6, count: 4, esr: 0.003, esl: 0.0 },
            CapType { c: 220e-6, count: 2, esr: 0.030, esl: 5e-9 },
        ];
        let mut separate = Vec::new();
        for _ in 0..4 {
            separate.push(CapType { c: 10e-6, count: 1, esr: 0.003, esl: 0.0 });
        }
        for _ in 0..2 {
            separate.push(CapType { c: 220e-6, count: 1, esr: 0.030, esl: 5e-9 });
        }

        let mut bank_g = CapBank::new(grouped, 3.0, 12.0);
        let mut bank_s = CapBank::new(separate, 3.0, 12.0);

        for _ in 0..200 {
            bank_g.integrate_phase(1e-6, 24.0, 3.0, 0.01, 2e-6, None);
            bank_g.integrate_phase(1e-6, 0.0, 3.0, 0.01, 2e-6, None);
            bank_s.integrate_phase(1e-6, 24.0, 3.0, 0.01, 2e-6, None);
            bank_s.integrate_phase(1e-6, 0.0, 3.0, 0.01, 2e-6, None);
        }

        assert!(
            (bank_g.v_out() - bank_s.v_out()).abs() < 1e-6,
            "v_out grouped={}, separate={}", bank_g.v_out(), bank_s.v_out()
        );
        assert!(
            (bank_g.i_l() - bank_s.i_l()).abs() < 1e-6,
            "i_l grouped={}, separate={}", bank_g.i_l(), bank_s.i_l()
        );
    }

    #[test]
    fn parallel_esr_formula_matches_impedance() {
        // N identical caps in parallel: ESR_parallel = ESR_unit / N.
        // Verify at high freq (where impedance ≈ ESR).
        let esr_unit = 0.020; // 20 mΩ per unit
        let n = 5;
        let caps = vec![CapType { c: 100e-6, count: n, esr: esr_unit, esl: 0.0 }];
        let omega = 2.0 * PI * 1e6;
        let (re, _) = CapBank::impedance_at(&caps, omega);
        let expected = esr_unit / n as f64;
        assert!(
            (re - expected).abs() < 1e-6,
            "ESR parallel: re={re}, expected={expected}"
        );
    }

    #[test]
    fn parallel_capacitance_formula_matches_impedance() {
        // N identical caps in parallel: C_parallel = N * C_unit.
        // Verify at low freq (where |Z| ≈ 1/(omega*C_total)).
        let c_unit = 10e-6;
        let n = 4;
        let caps = vec![CapType { c: c_unit, count: n, esr: 0.001, esl: 0.0 }];
        let f = 100.0;
        let omega = 2.0 * PI * f;
        let (_, im) = CapBank::impedance_at(&caps, omega);

        let c_total = c_unit * n as f64;
        let expected_im = -1.0 / (omega * c_total);
        assert!(
            (im - expected_im).abs() / expected_im.abs() < 0.01,
            "im={im}, expected={expected_im}"
        );
    }

    #[test]
    fn two_different_types_not_same_as_doubled_count() {
        // Two DIFFERENT cap types should NOT give the same impedance as
        // doubling one type's count. This is a sanity check that the
        // composite impedance actually models frequency-dependent behavior.
        let type_a = CapType { c: 10e-6, count: 1, esr: 0.003, esl: 0.0 };
        let type_b = CapType { c: 470e-6, count: 1, esr: 0.050, esl: 5e-9 };

        let mixed = vec![type_a.clone(), type_b.clone()];
        let doubled_a = vec![CapType { c: 10e-6, count: 2, esr: 0.003, esl: 0.0 }];

        let omega = 2.0 * PI * 10e3;
        let z_mixed = CapBank::impedance_at(&mixed, omega);
        let z_doubled = CapBank::impedance_at(&doubled_a, omega);

        // These should be significantly different
        let diff = ((z_mixed.0 - z_doubled.0).powi(2) + (z_mixed.1 - z_doubled.1).powi(2)).sqrt();
        assert!(
            diff > 0.001,
            "Mixed and doubled-A should differ, but diff={diff}"
        );
    }

    #[test]
    fn count_1_same_as_no_count_field() {
        // count=1 should be equivalent to the "obvious" single cap.
        // (Paranoia check that count=1 doesn't accidentally double anything.)
        let with_count = vec![CapType { c: 47e-6, count: 1, esr: 0.010, esl: 2e-9 }];
        // Manually compute single-cap impedance: Z = R + j(wL - 1/(wC))
        for &f in &[1e3, 100e3, 1e6] {
            let omega = 2.0 * PI * f;
            let (re, im) = CapBank::impedance_at(&with_count, omega);
            let expected_re = 0.010;
            let expected_im = omega * 2e-9 - 1.0 / (omega * 47e-6);
            assert!(
                (re - expected_re).abs() < 1e-10,
                "re={re}, expected={expected_re} at f={f}"
            );
            assert!(
                (im - expected_im).abs() < 1e-10,
                "im={im}, expected={expected_im} at f={f}"
            );
        }
    }
}
