# Deep code review — FINAL triaged findings

Two-pass multi-agent review of the 5 non-peri-planner crates. Every defect claim adversarially verified against source (2nd pass) or hand-checked.

- **139 real** defects/issues — 1 critical, 9 high, 31 medium, 82 low, 16 info
- **21 refuted** (verifier rejected — do not act on)
- **54 gap suggestions** (missing physics / features — enhancements, not defects)

---
## REAL — CRITICAL & HIGH

### [CRITICAL] Native sequence editor panics (index out of bounds) when deleting any non-last step
`kwr103-ui/src/main.rs:252-277` · bug · kwr · via hand-verified

In show_sequence_editor, the step list is iterated by fixed index 0..num_steps captured before the loop. The 'X' delete button calls remove_step(i) and then `return`, but that return only exits the inner `ui.horizontal(|ui| { ... })` closure, NOT the surrounding `for i in 0..num_steps` loop. After a removal the Vec length drops by one while the loop keeps iterating up to the stale num_steps-1, and the very next iteration indexes the shrunk Vec out of bounds, panicking the whole eframe app. This fires whenever the user deletes any step other than the last one.

**Fix:** Mirror the web implementation: collect `let mut to_remove = None;`, set `to_remove = Some(i)` on click, break out of / finish the loop, then call `remove_step(i)` after the loop. Alternatively iterate with `retain`/index-guard, or re-check `i < self.ui_state.sequence.len()` before indexing.

---

### [HIGH] Commutation-loop ringing (and Qrr/edges) alias: analytic waveform point-sampled with no band-limit, so f_ring above Nyquist folds to a false in-band peak
`buck-sim-ui/src/spectrum_export.rs:127-135, 276-329` · numerics · bs-dsp · via verify-pass2

PwmReconstructed synthesis point-samples an analytic waveform that contains content far above the reconstruction Nyquist (m·fsw/2). The commutation-loop ring is evaluated as a continuous damped sinusoid at the physical f_ring and sampled at fs = fsw·m with no anti-alias filter and no guard that f_ring < fs/2. For the default export path this is not a theoretical corner case: samples_per_cycle defaults to 64 (app.rs:540) and f_sw_khz defaults to 500 (sim.rs:369), so fs = 32 MHz and Nyquist = 16 MHz, while realistic loop parameters (a few nH, Coss ~370–620 pF) give f_ring in the 50–150 MHz range. The UI even prints the true f_ring to the user (app.rs:1859, e.g. '92 MHz') while the exported JSON silently places that energy at an aliased frequency. This directly corrupts the conducted-EMC spectrum the module exists to produce.

**Fix:** Guard against undersampling: in export_input_current_spectrum / PwmReconParams::build, if ringing is Some and f_ring_hz() >= 0.45·fsw·m, either (a) auto-raise samples_per_cycle to satisfy oversampling of the ring (e.g. m >= ceil(2.5·f_ring/fsw)), or (b) reject with an explicit error, or (c) omit the ring and stamp a warning in `source`. At minimum surface a UI warning when f_ring > Nyquist(m). Longer term, synthesize the ring analytically into the frequency domain (add a Lorentzian/complex pole-pair contribution at f_ring) instead of time-sampling it, which removes the aliasing entirely.

---

### [HIGH] v_droop_max KPI is dominated by the soft-start ramp (output starts at 0 V), so the droop distribution is a meaningless spike at V_out_target
`buck-sim-ui/src/monte_carlo.rs:197-205` · bug · bs-mc · via verify-pass1

extract_kpi computes droop as the minimum of v_out over the ENTIRE waveform vs target. But every simulation begins with a 1500-cycle soft-start that ramps V_out from 0 V up to target, so the global minimum is always ~0 V regardless of component tolerances. The 'worst transient dip below V_out_target' therefore equals ~V_out_target for essentially every realization, and the p5/p50/p95/worst droop percentiles all collapse to ~V_out_target with near-zero spread. The KPI never measures the load-step droop it is documented to measure (lines 20-22), which is the very quantity that varies with L/C/ESR tolerances.

**Fix:** Restrict the droop search to the post-startup / load-step region. Either skip the first SOFT_START_CYCLES + a settling margin (the sim exposes those constants), or measure droop relative to the pre-step steady-state value within each load phase, or add the phase boundaries to the returned data so extract_kpi can window on the actual step. The same startup contamination weakens v_overshoot_max (it conflates the startup-ring overshoot with the load-release overshoot).

---

### [HIGH] Compensator is re-designed for every perturbed plant, so the MC cannot detect the stability/phase-margin loss of the FIXED shipped controller
`buck-sim-ui/src/monte_carlo.rs:175-188, 242-243` · control · bs-mc · via verify-pass1

perturb_params perturbs L, C_out, ESR, DCR and both R_DS(on), then monte_carlo calls run_simulation(&p) on the perturbed params. run_simulation rebuilds the 2P2Z compensator from the perturbed plant (sim.rs:602-655 call build_ctrl_params(p) / ctrl_params_multi / tf.to_2p2z() using the perturbed p). This means each realization runs a controller RE-TUNED to its own off-nominal plant. Real production ships ONE compensator (designed at nominal) into parts whose L/C/ESR vary; the whole point of a control-loop tolerance study is to check that the fixed nominal compensator keeps adequate gain/phase margin and transient response across the plant tolerance box. By re-tuning per part, the MC makes every realization self-correcting and systematically hides the loop-gain crossover shift, phase-margin erosion, and subharmonic/period-doubling risk that a fixed compensator meeting an off-nominal plant would exhibit.

**Fix:** Design the compensator ONCE from `nominal`, freeze its weights, and provide a run path that injects the fixed weights while only the plant (L/C/ESR/DCR/R_DS(on)) varies. Then the transient/margin spread reflects a real production run. Optionally also keep the current 're-tuned' mode as a separate, clearly labeled study.

---

### [HIGH] HS-side Miller feedback has an inverted sign: positive feedback instead of the plateau-forming negative feedback
`electronics-sim/src/edge_transient.rs:672-675, 996-999` · physics · es-edge · via verify-pass1

For the high-side FET (drain = V_in fixed, source = SW), the correct gate-node equation is C_iss * dV_GS/dt = i_g - C_rss * dV_SW/dt. Carrying the algebra: i_g = C_gs*(V_g'-V_sw') + C_gd*V_g', with V_g' = V_gs' + V_sw', gives i_g = C_iss*V_gs' + C_rss*V_sw', so dV_GS/dt = (i_g - C_rss*dV_SW/dt)/C_iss. During turn-on dV_SW/dt > 0, so the Miller term SUBTRACTS and holds the gate at the plateau. The code instead ADDS it, turning the Miller effect into positive feedback that eliminates the plateau and helps the gate run away (v_gs_hs reached 80 V on a 10 V drive). The low-side line 675 uses +i_miller_ls and is correct; the two sides are inconsistent, which is the tell.

**Fix:** Change the HS update to add i_miller_hs (matching the LS convention): `let dv_gs_hs_dt = (i_g_hs + i_miller_hs) / fet_hs.c_iss;` in both simulate_hs_turn_on (line 674) and simulate_hs_turn_off (line 998). This yields (i_g_hs - C_rss*dV_SW/dt)/C_iss and restores the Miller plateau.

---

### [HIGH] Driver slew-rate limiter clamp condition is inverted (!= vs ==); rate limiting is completely defeated
`electronics-sim/src/edge_transient.rs:712-719, 1096-11` · control · es-edge · via verify-pass1

The slewed driver output is supposed to move toward its target by at most slew_rate*dt per step and clamp only when it OVERSHOOTS. Overshoot occurs when, after the step, the residual (act - target) has the SAME sign as the motion dv (both point the same way past the target). The code clamps when the signs DIFFER, which is true on every normal (non-overshooting) step, so it snaps v_drive_act to the target on the very first step, defeating slew_rate_v_per_s entirely (and after reaching target, signum(0)=+1 keeps nudging it up, producing a +/- slew*dt limit cycle around the rail).

**Fix:** Invert the comparison to clamp only on genuine overshoot: `if (v_drive_act_hs - v_drive_hs_target).signum() == dv_drv_hs_dt.signum() { v_drive_act_hs = v_drive_hs_target; }` (guard the exact-equality/zero case), in all four occurrences (turn-on 713/717, turn-off 1097/1101). Equivalently clamp when |act-target| has been crossed: `if (target-act_old) and (target-act_new) have opposite signs`.

---

### [HIGH] rem_pio2_large distill loop increments j instead of decrementing (`j += 1`) — infinite loop + out-of-bounds panic
`full-control/src/math/rem_pio2_large.rs:299-307` · bug · fc-math1 · via hand-verified

The 'distill q[] into iq[] reversingly' loop is the very first loop executed on every rem_pio2_large() call. It is supposed to walk j from jz down to 1 while i counts up from 0. The port increments j instead of decrementing it, so `while j >= 1` never terminates and i grows without bound.

**Fix:** Change line 306 to `j -= 1;` so the loop matches the reference `for(i=0,j=jz;j>0;i++,j--)`.

---

### [HIGH] rem_pio2_large prec 1|2 compress loop uses `i -= 0` — infinite loop on the exact precision (prec=1) that rem_pio2 requests
`full-control/src/math/rem_pio2_large.rs:465-475` · bug · fc-math1 · via hand-verified

In the `prec == 1 | 2` branch the first fq-summation loop decrements i by zero, so the loop counter never changes and the `if i == 0` exit is never reached. rem_pio2() always calls rem_pio2_large(..., prec = 1), so this is on the used path.

**Fix:** Change `i -= 0;` to `i -= 1;`.

---

### [HIGH] scalbn uses f64::MAX_EXP/MIN_EXP (1024/-1021) instead of Emax/Emin (1023/-1022) — factor-of-2 error, spurious Inf and underflow-to-zero
`full-control/src/math/scalbn.rs:27-86` · numerics · fc-math2 · via hand-verified

The port substitutes Rust's std constants for the exponent bounds:

    let exp_max = f64::MAX_EXP;   // = 1024
    let exp_min = f64::MIN_EXP;   // = -1021

But Rust's f64::MAX_EXP is 1024 and f64::MIN_EXP is -1021 (the "one greater" convention). The scalbn prescaling algorithm needs the actual biased-exponent-derived bounds Emax = 1023 (EXP_BIAS) and Emin = -1022 (-(EXP_BIAS-1)). Those are exactly the exponents of the prescale multipliers already computed in the same function: f_exp_max = from_parts(false, EXP_BIAS<<1, 0) = 2^1023 and f_exp_min = from_parts(false, 1, 0) = 2^-1022 (the code's own comments say '2^Emax ... (0x1p1023)' and '2^Emin ... (0x1p-1022)'). Because the prescale multiplies x by 2^1023 while decrementing n by exp_max=1024 (line 51-52), and multiplies by mul=2^-969 while adding add=-exp_min-53=968 (lines 65-73), every prescale step is off by exactly a factor of 2. The threshold comparisons (n > exp_max, n < exp_min) are also off by one, so n=1024 skips prescaling and the final from_parts(EXP_BIAS+1024=2047,0) yields +Inf even when the true result is finite.

**Fix:** Replace the std constants with the true bounds that match the prescale multipliers:
    let exp_max: i32 = EXP_BIAS as i32;          // 1023
    let exp_min: i32 = -(EXP_BIAS as i32 - 1);   // -1022
(import EXP_BIAS, already in scope via super). Verified this makes all edge cases match the reference.

---

### [HIGH] Inner current-loop design reserves zero phase for computation/ZOH/sampling delay — realized PM collapses far below the 60° target
`full-control/src/control_pfc.rs:96-128, 176-190` · control · fc-pfc · via verify-pass2

design_type2() sizes the compensator zero purely from the plant integrator (-90°) plus its own HF pole, with no term for the transport/computation/PWM-ZOH delay of a digital current loop. At the documented inner crossover of f_sw/10 this omission eats most of the phase margin.

**Fix:** Add a transport-delay phase term to design_type2 (or to the inner-loop call) equal to ω_x·τ where τ≈1.5·T_sw for the ACM loop (matching the previous-cycle sample + PWM ZOH), i.e. `phi_target = phase_margin + atan(ω_x/ω_cp1) + ω_x*tau`, and reject designs whose realized PM would be negative. Make the delay model identical to what pfc_bode.rs plots (currently 0.5·T_sw) and to control_2p2z.rs; then the achieved crossover PM will equal the requested value.

---

## REAL — MEDIUM

- **Loading/clearing a LoopExtraction JSON leaves the cached loss breakdown (Ringing loss) stale until an unrelated slider is nudged** — `buck-sim-ui/src/app.rs:1557-1568,1749-1` (bug, bs-app)
  compute_losses() takes l_commutation_loop_h and produces commutation_ringing_w (confirmed in sim.rs:1353-1356,1470). loss_breakdown is only recomputed inside the params-changed gate of show_buck_central. try_load_loop() sets self.loaded_loop = Some(...) but never recomputes loss_breakdown, and neither does the load-click handler. So after loading a PEEC LoopExtraction, the Power-Losses panel's 'Ri
  *Fix:* After a successful try_load_loop() (and after the loop 'Clear' button), recompute the cached loss breakdown from the current data, e.g. `if matches!(self.loop_load_status, Some(Ok(_))) { let l = self.loaded_loop.as_ref().map(|l| l.l_self_henry); self.loss_breakdown = self.sim_data.as_ref().ok().and_
- **3-phase PFC harmonic-limit lines are ~3x too strict: fundamental current not divided by number of AC phases** — `buck-sim-ui/src/app.rs:2419-2421,2450-2` (bug, bs-app)
  For the IEC 61000-3-2 Class A/B/D overlays the code estimates the per-phase fundamental current used to convert absolute mA (or mA/W) limits into '% of fundamental'. It uses total load power over single-phase voltage/PF, without dividing by ac_phases. In pfc_sim.rs the per-AC-phase current is p_load_w()/ac_phases divided by v_in_rms (line 274: `let p_per_ac_phase = p.p_load_w() / ac_phases`), and 
  *Fix:* Divide the total power by ac_phases before forming i_fund, e.g. `let i_fund = (p_w / self.pfc.ac_phases as f64) / (self.pfc.v_in_rms * metrics.power_factor.max(0.01));` in all three (A, B, D) branches.
- **Gain-margin detection is dead code: wrapped atan2 phase can never satisfy `loop_phase[i] <= -180`, so GM is (almost) always reported as +∞** — `buck-sim-ui/src/bode.rs:330-338` (bug, bs-bode)
  The gain-margin scan looks for a downward crossing of the -180° line, but the loop phase array is produced exclusively by `c_phase_deg = a.1.atan2(a.0).to_degrees()`, which is mathematically confined to the half-open interval (-180°, 180°]. A value strictly ≤ -180° is essentially never produced (only the measure-zero case im = -0.0, re<0 gives exactly -180). Hence the loop condition `loop_phase[i-
  *Fix:* Unwrap the phase before margin extraction (accumulate ±360° when consecutive samples jump by more than 180°) and scan the unwrapped array for the -180° crossing; equivalently detect the crossing from the sign of the imaginary part while the real part is negative (Im(T) crosses 0 with Re(T)<0). Apply
- **Same unreachable gain-margin scan in PFC find_margins() (both inner and outer loops report GM = ∞)** — `buck-sim-ui/src/pfc_bode.rs:236-244` (bug, bs-bode)
  find_margins() is shared by the inner current loop and outer voltage loop and contains the identical wrapped-phase defect described for bode.rs. `loop_phase` comes from `c_phase_deg` (atan2, range (-180,180]), so `loop_phase[i] <= -180.0` is unreachable and `gain_margin_db` is left at `f64::INFINITY`. This is especially misleading for the PFC OUTER loop: because the outer plant is modeled as a pur
  *Fix:* Unwrap phase before the -180° scan (shared helper with bode.rs). Replace the 60° PM fallback with a sentinel (e.g. NaN / Option) so 'no crossover found' is displayed honestly instead of a fabricated 60°.
- **Delay 'hold' term (cycles_per_tick-1)/f_sw is not the ZOH group delay T_ctrl/2; sampling phase lag is mis-estimated (zero for cycles_per_tick=1)** — `buck-sim-ui/src/bode.rs:50-60` (control, bs-bode)
  tau_delay adds a hold term `t_hold = (cycles_per_tick - 1)/f_sw`, while the control period is `t_ctrl = cycles_per_tick/f_sw`. The physical group delay of a zero-order hold / sample-and-hold is T_ctrl/2 = cycles_per_tick/(2·f_sw). The chosen expression equals T_ctrl/2 only when cycles_per_tick = 2; it is 0 for cycles_per_tick = 1 (the most common single-update case) and OVER-estimates for larger N
  *Fix:* Model the hold as the ZOH half-period: use t_ctrl/2 = cycles_per_tick/(2·f_sw) for the sampling delay component (or apply the full complex ZOH (1-e^{-jωT})/(jωT) instead of a lumped delay + magnitude-only sinc).
- **Interleaved multiphase modeled as one phase scaled by n_phases (all phases in-phase) — misses interleaving stagger, ripple cancellation, and the n·fsw harmonic comb** — `buck-sim-ui/src/spectrum_export.rs:270-274, 328, 51` (missing-physics, bs-dsp)
  The synthesis takes total current, divides by phase count to get a per-phase pulse, then multiplies the whole waveform (base + Qrr + ring) by n_phases. This is physically equivalent to all phases switching simultaneously and in phase. A real interleaved converter staggers phases by T_sw/n_phases, which cancels much of the fsw-rate input ripple and shifts the dominant input-current harmonics to n_p
  *Fix:* When n_phases > 1, synthesize n_phases pulse trains offset by k·T_sw/n_phases (k=0..n_phases-1) each with its own transition events, and sum them, instead of scaling one phase by n_phases. This yields correct ripple cancellation and the n·fsw harmonic comb. Alternatively, document loudly that PWM-re
- **FMAC coefficient quantization uses raw R while I/O and gain use r_eff — 2× gain error (and halved output clamp) whenever min_fmac_r()==0** — `buck-sim-ui/src/inner_ctrl.rs:95-118` (bug, bs-inner)
  In build() the coefficients and the y_min/y_max clamp bits are computed with the raw exponent `r` returned by `min_fmac_r()`, but the FMAC itself and every subsequent I/O adapter use `r_eff = r.max(1)`. When `min_fmac_r()` returns 0 (all five |coeffs| < 1.0 — e.g. a low-gain/heavily-damped code-domain compensator), the coefficients are quantized at scale 2^0 while the FMAC gain shift and the code<
  *Fix:* Clamp R once, before it is used anywhere: `let r = weights.min_fmac_r().max(1);` and drop the later `r_eff`. Then coeffs, y_min/y_max, the FMAC, r_exponent and the update/prime adapters all share the same exponent and R cancels exactly.
- **FMAC flavor omits clamped-feedback anti-windup that the F32 flavor performs — integrator winds up when the call-site clamp is tighter than the build-time FMAC limits** — `buck-sim-ui/src/inner_ctrl.rs:139-160` (control, bs-inner)
  The F32 path uses `TwoPoleTwoZero::update_clamped`, which writes the externally-clamped value back into the output history via `set_last_output(clamped)` (clamped-feedback anti-windup). The FMAC path applies the outer `.clamp(lo_code, hi_code)` only to the returned code, but the FMAC's internal y-history already stored `y_out` clamped to the build-time y_min/y_max only. `FmacIir` even exposes `set
  *Fix:* After computing the outer-clamped `out`, convert it back to q1.15 (`out<<r`, clamped to i16) and call `fmac.set_last_output(out_q15)` so the FMAC history tracks the actually-applied output, matching the F32 flavor's anti-windup.
- **Very high-gain code-domain coefficients (b0 up to 1e6 pass the guard) drive R past 14, causing a negative-shift panic/UB in FmacIir::update; the 'FmacIir::new asserts r >= 1' comment is false** — `buck-sim-ui/src/inner_ctrl.rs:108-113` (bug, bs-inner)
  The caller (sim.rs build_controller) only rejects b0 when non-finite or |b0| > 1e6, so coefficients up to ~1e6 reach build(). `min_fmac_r()` then returns r ≈ log2(coeff), e.g. r≈20 for 1e6, or r=15 already for coeff≥16384. inner_ctrl never clamps r to the valid FMAC range [1,14], and the code comment claiming 'FmacIir::new asserts r >= 1' is incorrect — `FmacIir::new` is a plain const constructor 
  *Fix:* Clamp the exponent to the hardware-valid range, e.g. `let r = weights.min_fmac_r().clamp(1, 14);`, and either add a real debug_assert in FmacIir::new or fix the misleading comment. Optionally tighten the caller's coefficient guard so unrepresentable gains are rejected rather than silently mis-scaled
- **Conducted-EMC dBµV uses peak spectral amplitude where the CISPR limit is RMS-referenced (+3.01 dB systematic error)** — `kicad_field_solver/pdn_emc/src/lib.rs:63-73, 155-185` (physics, bs-pdn)
  The LISN pipeline converts the port voltage to dBµV with db_uv(z_lisn * i_lisn), where i_lisn is derived from the PortCurrentSpectrum's i_re_amp/i_im_amp. Those are one-sided PEAK amplitudes (the producer in buck-sim-ui/src/spectrum_export.rs, lines 564-598, normalizes so 'a pure sine at amplitude A reads A'). But an EMI receiver / CISPR limit is calibrated in RMS for a CW tone. db_uv itself impli
  *Fix:* Convert the LISN voltage (or the injected current) to RMS before db_uv: e.g. in sweep_lisn_dbuv_with_input_cap use `db_uv(z_lisn * i_lisn / std::f64::consts::SQRT_2)`, and document that PortCurrentSpectrum carries peak amplitudes. Alternatively store RMS in the spectrum. Update the db_uv doc/tests t
- **3-phase efficiency metric is ~3x inflated (integrates phase-A input power only) and always saturates at 100%** — `buck-sim-ui/src/pfc_sim.rs:749-781, 866-871` (bug, bs-pfc)
  In compute_metrics, for ac_phases>=3 the per-cycle input current is taken as phase A alone (i_a = sum of phase-A interleaved stages), and p_in accumulates only v_in_rect(phase A) * i_a. So p_in_avg is the single-phase (per-phase) real power ~ P_total/3. Efficiency is then computed as p.p_load_w() (the TOTAL 3-phase load) divided by that per-phase input power, giving ~3.0, which is clamped to 100%.
  *Fix:* Multiply the 3-phase input power/energy by ac_phases before computing efficiency (or accumulate all three phases), e.g. use p_in_avg_total = p_in_avg * ac_phases in the efficiency ratio; keep PF/THD on the per-phase quantities.
- **Shared DC-bus charge balance across AC phases is broken — only the last AC phase's charge update survives** — `buck-sim-ui/src/pfc_sim.rs:367-477` (physics, bs-pfc)
  For ac_phases>1 each AC-phase group is ticked independently, each starting from the same pre-cycle v_cap and each draining only seg_i_load/ac_phases. After the per-group loop the code force-sets every sim's v_cap to the LAST group's value, discarding the charge deltas computed by groups 0..N-2. The bus therefore only integrates one phase's generation and 1/N of the load per cycle, i.e. it behaves 
  *Fix:* Accumulate charge across phases into the shared cap: either chain groups (set group i's first sim v_cap to group i-1's post-tick v_cap, like the interleave path), or sum the per-group Δq and apply once: v_cap_new = v_cap_start + (Σ_ac Δq_ac)/C, then broadcast.
- **CrCM time step collapses to ~150 ns when the voltage loop output is zero, producing ~65k spurious frozen-bus points** — `buck-sim-ui/src/pfc_sim.rs:377-402, 500-505` (numerics, bs-pfc)
  In CrCM the loop advances time by actual_t_sw_out.max(t_sw*0.01). When k=0 (e.g. during the inert soft-start above, or any interval where the outer loop saturates low), crcm_on_time returns t_on=0, so PfcBoostSim::tick_crm returns t_sw_actual=0 AND leaves v_cap unchanged (both the on-time and off-time load-drain terms are i_load*0). The guard then advances time by only t_sw*0.01 = ~0.154us at 65kH
  *Fix:* When t_on/t_sw_actual is ~0, still advance time by the nominal t_sw and still drain the load over that interval (bus should droop when idle); clamp the CrCM period to a sane [f_sw_max, f_sw_min] window instead of t_sw*0.01. Consider running the outer loop on a fixed control tick decoupled from the v
- **slope_overcomp never scales the actual simulated slope-compensation ramp (only widens the anti-windup clamp)** — `buck-sim-ui/src/sim.rs:612-622, 664-666` (control, bs-sim)
  The user-facing 'Slope over-compensation factor' (SimParams::slope_overcomp, default 1.5 = 'firmware default') is threaded ONLY into dac_settings_at() for the anti-windup clamp offset. The slope ramp actually applied by the time-domain engine (sim_params.slope_amp_per_sec) is derived from to_transfer_function(), which hard-codes overcomp=1.0. So the simulated peak-current-mode converter always run
  *Fix:* Derive the applied ramp with the user's factor: `let dac = ctrl_params.dac_settings_at(p.v_in, p.v_out_target, ControlTopology::Buck, p.slope_overcomp); let slope_amp_per_sec = dac.dac_slope / cs_gain;` and use that same overcomp'd `dac.vpp()` in the ripple/limit-cycle check (line 644) so the ramp, 
- **hs_eoss switching-node capacitance loss omits the HS FET Coss (uses only ls_fet.coss_pf)** — `buck-sim-ui/src/sim.rs:1435-1437` (physics, bs-sim)
  The hard-switching output-capacitance loss at HS turn-on charges/discharges BOTH the LS Coss (0->Vin) and the HS Coss (Vin->0) through/into the HS channel each cycle. The dissipated energy is ~0.5*(Coss_hs+Coss_ls)*Vin^2 per cycle. The code counts only Coss_ls, so for the common case of matched HS/LS FETs the Eoss term is under-reported by ~2x, and for a design with a large HS device it can be off
  *Fix:* Use the node's total switched capacitance: `let coss = 0.5*(p.hs_fet.coss_pf + p.ls_fet.coss_pf)*1e-12;` then `hs_eoss = 0.5*coss*v_in*v_in*f_sw*n_phases;` (or sum the two 0.5*C*V^2 terms). Ideally integrate charge-based Q_oss(V) if available, since MLCC/FET Coss is strongly voltage-dependent.
- **Spurious V_in*(1-cos wt) term predicts perfect ZVS at ZERO inductor current** — `buck-sim-ui/src/zvs.rs:137-165, 191-213` (physics, bs-zvs)
  The V_SW(t)=V_in - V_in*cos(wt) + I*Z0*sin(wt) expression is the step response of a *series* V_in-L-C loop, in which the source rings the cap to V_in even with zero initial current. During a real dead-time both FETs are off and the SW node is floating — the only drive is the inductor current, and the source only connects through a capacitor (HS Coss), so no such V_in step-ring exists. As coded, th
  *Fix:* Model the transition as inductor-current-driven and ring it around the correct rail (V_out for the SW-rising transition), i.e. V_SW = V_out*(1-cos wt) + I*Z0*sin(wt) with omega,Z0 from L_out and 2*Coss, or use the constant-current linear ramp V_SW = I*t/(2*Coss) with the energy check 0.5*L_out*I^2 >
- **Constant Coss ignores Coss(V) — datasheet value is spec'd at 50 V but used to swing 0->24 V; optimistically over-predicts ZVS** — `buck-sim-ui/src/zvs.rs:58-63, 84-118, 1` (missing-physics, bs-zvs)
  Coss is strongly voltage dependent (roughly ~1/sqrt(V) for a GaN HEMT); the relevant quantities for ZVS are the charge-equivalent Co(tr)=Qoss/V (for the dead-time / charge criterion) and the energy-equivalent Co(er)=2*Eoss/V^2 (for the residual loss), not the small-signal spot value. The code uses a single constant coss_pf that FetProfile documents as the value at V_DS=50 V, then uses it to charge
  *Fix:* Store/derive Qoss(V) or the energy- and time-related equivalent capacitances from the FET's Coss(V) curve; use Co(tr) for the charge/time criterion and Co(er) for the loss energy, both evaluated across 0->V_in.
- **V_SW residual not clamped by the lower body-diode rail — residual and E_hard can exceed the full-hard-switch value** — `buck-sim-ui/src/zvs.rs:204-213, 266` (bug, bs-zvs)
  vds_residual_at_turnon_v clamps only the TOP of the swing (V_SW >= V_in -> residual 0). It never clamps the bottom. For a deadtime past the ring-back (phase in (pi,2pi)) at high drive the formula yields V_SW < 0, so the returned residual V_in - V_SW exceeds V_in and E_hard = 0.5*C*V_DS^2 exceeds the physical maximum 0.5*C*V_in^2. In reality the node is clamped to roughly [-Vf, V_in+Vf] by the devi
  *Fix:* Clamp v_sw to [0, v_in] (or [-Vf, v_in]) before computing the residual: `let v_sw = v_sw.clamp(0.0, v_in_v); return (v_in_v - v_sw).max(0.0);`
- **Three of four exerciser binaries do not compile against the current control_2p2z API (stale crossover_divisor / missing crossover_hz)** — `electronics-sim/src/bin/buck-boost-test.rs:103-118, 304-315` (bug, es-bins)
  buck-boost-test.rs, control-test.rs and control-test-fmac.rs are written against an obsolete `full_control::control_2p2z::Parameters` API. They (a) omit the now-required `crossover_hz` field when building `Parameters`, and (b) call `Parameters::crossover_divisor(topology, v_in)`, a method that no longer exists. The library was refactored from auto-selecting the crossover via a `safety_factor`-deri
  *Fix:* In each binary add `crossover_hz` to the `Parameters` literal and replace the `crossover_divisor` prints. To preserve the old 'optimal auto-selected bandwidth' behaviour, set `crossover_hz: base.max_feasible_crossover_hz(v_in, topology)` per design point (buck-boost-test already effectively wants th
- **Parasitic power-loop inductance L_power is never in the state equations; the headline LC ring / overshoot / dV-dt EMI is not actually simulated (and its absence makes the SW node stiff)** — `electronics-sim/src/edge_transient.rs:516-528, 677-698` (missing-physics, es-edge)
  The module doc advertises 'Parasitic-LC ringing between the PCB power-loop inductance L_power and the SW-node total capacitance' and reports f_ring_est, v_sw_overshoot, dv_sw_dt_peak as headline EMI numbers. But l_power is used ONLY to compute f_ring_est; it never appears in any derivative. The SW-node KCL charges C_sw directly from the algebraic FET/diode currents with no series loop inductance, 
  *Fix:* Introduce an actual power-loop current state i_lp with L*di_lp/dt = v_in - v_sw - i_lp*loop_r (HS on-path) feeding the SW node, so C_sw*dV_SW/dt = i_lp - i_l - i_d_ls - i_diode_ls (and the HS FET current is the loop current, not an instantaneous algebraic value). This both realizes the L_power-C_sw 
- **Reverse-recovery diode current enters the SW-node KCL with the wrong sign** — `electronics-sim/src/edge_transient.rs:616-683` (physics, es-edge)
  i_diode_ls is defined with an into-the-SW-node convention: the Forward-state value (line 621) is (i_l + i_d_ls - i_d_hs).max(0), which is exactly the current that balances KCL as +i_diode_ls (0 = i_d_hs - i_l - i_d_ls + i_diode_ls). The consistent RR/Off KCL must therefore also use +i_diode_ls. The code uses -i_diode_ls. In ReverseRecovery i_diode_ls < 0 (reverse current, from soft_recovery_i_diod
  *Fix:* Use `(i_d_hs - i_l - i_d_ls + i_diode_ls) / c_sw_total` in the RR/Off branch (line 681) so the reverse-recovery current subtracts from the node charging current, consistent with the into-node sign convention of the Forward-state formula.
- **Buck-boost RHP-zero limit uses boost formula (missing 1/D factor)** — `full-control/src/control_2p2z.rs:429-438` (math, fc-2p2z)
  In max_feasible_crossover_hz the RHP-zero frequency used to cap the crossover is computed identically for Boost and BuckBoost as ω_RHP = D'²·R_load/L. The textbook CCM buck-boost RHP zero is ω_RHP = D'²·R_load/(D·L) — it carries an extra 1/D factor that the boost topology does not. The code therefore reports a lower RHP frequency than reality for buck-boost, making the crossover cap overly conserv
  *Fix:* Split the match arm: keep D'²R/L for Boost and use D'²R/(D·L) for BuckBoost (guard D→0). Since D≈0.5 typical, this roughly doubles the allowed buck-boost crossover.
- **Calculated phase-margin branch hardcodes 50° base and omits the voltage-loop ZOH/computation delay for cycles_per_tick=1** — `full-control/src/control_2p2z.rs:782-796` (control, fc-2p2z)
  In the PhaseMargin::Calculated branch the target margin is `50° + phase_erosion`, where phase_erosion only includes t_adc + t_processing + t_dac + t_hold, and t_hold = (cycles_per_tick − 1)/f_sw is ZERO for cycles_per_tick=1. A digital voltage loop that samples and updates a ZOH every switching cycle has an intrinsic ~T_sw/2 averaging lag (plus commonly a one-sample compute-to-apply delay) that is
  *Fix:* Add an explicit ZOH term, e.g. include `t_ctrl/2 = cycles_per_tick/(2·f_sw)` (and optionally a full compute sample T_sw) in phase_erosion so it is nonzero for N=1; and make the 50° base configurable (or derive it from the user's PhaseMargin input) instead of hardcoding.
- **FMAC emulator clamps the fed-back y-history to arbitrary [y_min,y_max]; the real STM32 FMAC cannot do this (it clips only to full-scale q1.15)** — `full-control/src/fmac.rs:22-42, 76-80` (physics, fc-bb-fmac)
  FmacIir claims to 'implement the FMAC recurrence faithfully … exactly as the hardware does', but it applies a software clamp to arbitrary code-domain limits inside the feedback path and stores the CLAMPED value in the y-history. The STM32 FMAC runs autonomously; its only saturation/clip (CLIPEN) is to full-scale q1.15 (±1.0 = [-32768,32767]), NOT to a user sub-range such as the DAC-code window. So
  *Fix:* Model the FMAC exactly: clip the fed-back y-history only to full-scale q1.15 (±32768/32767) matching CLIPEN, and expose the DAC-code clamp as a SEPARATE post-read software step (as inner_ctrl already re-applies with its own clamp) that does NOT rewrite the FMAC's internal history. If clamped-feedbac
- **FMAC gain-exponent mismatch: coefficients quantized at 2^r but FmacIir/I-O run at r_eff=r.max(1) → silent 2× output when min_fmac_r()==0** — `buck-sim-ui/src/inner_ctrl.rs:95-117` (numerics, fc-bb-fmac)
  FmacIir takes the pre-quantized coefficients AND R as independent inputs with no cross-check, and the fmac.rs contract requires coeff_bits = real_coeff/2^R. The caller quantizes with the natural exponent r = min_fmac_r() but then constructs the filter and all code↔q1.15 shifts with r_eff = r.max(1). When min_fmac_r() returns 0 (all |coeff|<1), the coefficients are scaled by 2^0=1 while the FMAC ga
  *Fix:* Quantize with the SAME exponent used at runtime: either allow r=0 in FmacIir (shift=15 is valid) and drop the .max(1) entirely, or clamp r before BOTH fmac_coeffs(r) and FmacIir::new (`let r = min_fmac_r().max(1); let coeffs = fmac_coeffs(r);`). Better: have FmacIir accept the f32 weights + R and qu
- **Current-loop anti-windup limits (+/-V_dc) are decoupled from the real [0,1] duty saturation, allowing integrator windup** — `full-control/src/dq_controller.rs:90-98` (control, fc-dq)
  Both current-axis PI controllers are constructed with saturation limits of +/-v_dc_max (the DC bus voltage), and PiController clamps both its integrator and output to those limits. But the true actuator saturation is the per-phase duty clamp to [0,1] applied AFTER inverse Park/Clarke and after the grid feedforward is added (v_cmd = -u + v_grid + decoupling). The reachable per-phase modulation volt
  *Fix:* Either (a) set the PI limits to the true available voltage margin (roughly +/-V_dc/2, or better the per-step headroom after feedforward), or (b) add back-calculation anti-windup: compute the saturated v_cmd vs unsaturated and feed (v_sat - v_unsat)*Kaw back into the integrator so the integrators sto
- **rem_pio2_large down-counting `loop`s skip index 0, dropping the dominant reduction term(s)** — `full-control/src/math/rem_pio2_large.rs:423-464` (bug, fc-math1)
  The port rewrote several reference loops of the form `for (i=jz; i>=0; i--)` (inclusive of 0) as `loop { body; i -= 1; if i == 0 { break; } }`, which processes i = jz..=1 and NEVER processes i = 0. This silently drops one array element in each such loop compared with musl/newlib.
  *Fix:* Restructure these loops to include index 0, e.g. `let mut i = jz; loop { body(i); if i == 0 { break; } i -= 1; }`, or use a signed counter with `while i >= 0`. Apply to the convert loop (424-433), the fq loop (436-449) and the prec-0 sum (455-462); leave the y[1] sum's range as 1..=jz.
- **PCMC slope compensation S_e ∝ V_in vanishes near the AC zero-crossing where duty→1, violating the subharmonic-stability floor** — `full-control/src/pfc_runtime.rs:90-113` (control, fc-pfc)
  The comparator's compensation ramp is referenced to the inductor UP-slope (V_in/L). Over a rectified line cycle V_in swings from V_pk to 0; the boost duty D=1−V_in/V_out therefore approaches 1 near the zero-crossing, which is exactly where peak-current-mode needs the MOST slope compensation to avoid period-doubling — yet the provided ramp goes to zero there.
  *Fix:* Reference the ramp to the DOWN-slope instead: `se = slope_overcomp * (v_out - v_in) / l`, or use a constant ramp ≈ oc·V_out/(2L). Because pcmc_reference_corrected's absorption factor (1+oc) assumes S_e=oc·(up-slope) so that i_peak=i_ref/(1+oc), the reference-shaping algebra (lines 58, 66) must be up
- **No numerically-extracted small-signal AC sweep (perturb-and-linearize) from the switching model** — `buck-sim-ui/src/bode.rs:bode.rs:157-210 ` (numerics, gap-numerics)
  Every loop-gain / Bode / output-impedance result in the tool is computed from hand-derived analytic transfer functions, NOT measured from the nonlinear time-domain sim. bode.rs `plant()` hardcodes the canonical PCMC form `h_dc·(1+jω/ω_esr)/((1+jω/ω_p1)·((jω/ω_n)²+jω/ω_n+1))`; pfc_bode.rs hardcodes `V_out/(jωL)` (inner) and `V_pk/(2 V_out jω C_out)` (outer). There is a full cycle-accurate nonlinear
  *Fix:* Add an AC-sweep engine: at each log-spaced ω, add a small sinusoid to the control input (i_trip / duty ref) or a series test source at the output, run the switching sim to periodic steady state, and extract the complex ratio via a single-bin DFT (Goertzel / sin-cos correlation) of stimulus and respo
- **Ethernet IP text field is dead — connection always targets hardcoded 192.168.1.100** — `kwr103-ui/src/main.rs:42-46, 62-75` (bug, kwr)
  The Ethernet radio button stores a ConnectionType::Eth whose ip is the constant literal "192.168.1.100". The user-editable IP TextEdit binds to a separate field self.ip_address, which is never copied back into self.connection_type. connect() then clones self.connection_type, so RealPowerSupply::new always dials 192.168.1.100 regardless of what the user typed. Connecting to any other IP over Ethern
  *Fix:* Before connecting, build the connection type from the live field, e.g. `let conn = if eth_selected { ConnectionType::Eth { ip: self.ip_address.clone() } } else { ConnectionType::Usb };` and pass `conn` to RealPowerSupply::new. Or make the TextEdit edit the ip inside the enum variant directly and dro
- **Sequence playback re-sends set_voltage/current/output to the real device every frame (~100 Hz command flood)** — `kwr103-ui/src/lib.rs:242-264` (bug, kwr)
  VoltageSequence::update returns Some(current) on EVERY call while a step is still active (the else branch), and the native playback loop calls apply_sequence_step on every Some — which unconditionally issues set_voltage, set_current AND set_output. Because the playback block in main.rs runs on every update() (repaint is scheduled every 10 ms and also fires on input events), the same three commands
  *Fix:* Have update() return Some(step) only on an actual step transition (new step began) and None while dwelling within a step, e.g. return an enum {Started(step), Continuing, Finished}. The caller then applies commands only on Started. Alternatively track last_applied_step_index in the app and skip apply

## REAL — LOW / INFO (titles)

- [low] PFC outer voltage-loop crossover hard-capped at f_line/3 even for 3-phase, where there is no 2·f_line bus ripple to reject — `buck-sim-ui/src/app.rs:2144-2150` (bs-app)
- [low] PFC outer voltage-loop plant omits the load pole 2/(R_load·C_out); modeled as a pure integrator — `buck-sim-ui/src/pfc_bode.rs:88-96` (bs-bode)
- [low] ZOH sinc uses signed sin(x)/x instead of |sinc|, injecting a spurious 180° phase flip and magnitude null above f_ctrl = f_sw/cycles_per_tick — `buck-sim-ui/src/bode.rs:91-96` (bs-bode)
- [low] Ringing never reaches periodic steady state: 4 warmup cycles vs a decay window of tens of cycles; export window also starts with empty ring history — `buck-sim-ui/src/spectrum_export.rs:385-404, 198-207` (bs-dsp)
- [low] Symmetric Hann window (denominator N-1) used for FFT of periodic data instead of the periodic/DFT-even Hann (denominator N) — `buck-sim-ui/src/spectrum_export.rs:568-572` (bs-dsp)
- [low] Input pre-scale is tied to coefficient exponent R (code<<R), so high-gain compensators lose input headroom and silently saturate large errors at the FMAC input — `buck-sim-ui/src/inner_ctrl.rs:148-150` (bs-inner)
- [low] FMAC bumpless-transfer prime is broken: ignores FmacIir::prime, never sets output history to `output`, and desynchronizes the cached last_output from FMAC state — `buck-sim-ui/src/inner_ctrl.rs:176-201` (bs-inner)
- [low] Failed realizations are silently excluded from the percentile statistics (survivorship bias) — the worst corners disappear from the reported distribution — `buck-sim-ui/src/monte_carlo.rs:249-263` (bs-mc)
- [low] Independent, symmetric log-normal sampling misses the temperature-correlated worst corner and cannot reach the cold-ESR 3–5x multiplier the module itself motivates — `buck-sim-ui/src/monte_carlo.rs:57-66, 166-186` (bs-mc)
- [low] Steady-state ripple and V_out average are measured only on the last load phase (default = lightest load), not the worst-case load — `buck-sim-ui/src/monte_carlo.rs:206-223` (bs-mc)
- [low] 'converged' only checks that output is non-empty, not that the loop actually settled — overstates convergence and lets non-settled runs pollute the stats — `buck-sim-ui/src/monte_carlo.rs:193-225, 244-247` (bs-mc)
- [low] Documented KPI t_settle is never computed (doc/impl mismatch) — `buck-sim-ui/src/monte_carlo.rs:18-23, 94-107` (bs-mc)
- [low] Soft-start is inert/inverted: output cap is pre-initialized to full V_out, so the reference ramp just idles the converter — `buck-sim-ui/src/pfc_sim.rs:287-296, 344-357` (bs-pfc)
- [low] Output-cap ESR never affects the simulated bus voltage — v_out_ripple_v omits the ESR ripple component — `buck-sim-ui/src/pfc_sim.rs:356, 490, 703-71` (bs-pfc)
- [low] Efficiency uses nominal load power as the numerator even during load steps — `buck-sim-ui/src/pfc_sim.rs:811-815, 866-871` (bs-pfc)
- [low] Voltage-loop prime k_init uses a wrong formula, over-commanding peak input current by ~2.2x — `buck-sim-ui/src/pfc_sim.rs:309-321` (bs-pfc)
- [low] ADC-conversion, processing and DAC-settling latencies are used only for compensator design, not as a transport delay in the time-domain loop — `buck-sim-ui/src/sim.rs:476-484, 664-686` (bs-sim)
- [low] Cap-bank RK4 budget check uses the wrong cycle count for Battery mode (8000 vs actual 11500) — `buck-sim-ui/src/sim.rs:712-718, 842-844` (bs-sim)
- [low] No guard on cycles_per_tick==0 (modulo panic) or cs_gain_mv_a==0 (Inf trip/slope) — `buck-sim-ui/src/sim.rs:763, 590-614` (bs-sim)
- [low] HRTIM DAC slope quantization is modeled as static trip-command snapping, not the intra-cycle slope staircase the engine can produce — `buck-sim-ui/src/sim.rs:616-622, 678, 99` (bs-sim)
- [low] ZVS resonant tank uses commutation-loop inductance (nH) instead of the output inductor — threshold current / period / min-load all wrong — `buck-sim-ui/src/zvs.rs:4-40, 84-118, 25` (bs-zvs)
- [low] Only one commutation transition modeled; dead-time reverse-conduction loss omitted — `buck-sim-ui/src/zvs.rs:22-49, 244-277` (bs-zvs)
- [low] optimal_deadtime_s docstring/comment do not match behavior (below-threshold return, peak-of-swing, and unimplemented epsilon) — `buck-sim-ui/src/zvs.rs:81-83, 96-102` (bs-zvs)
- [low] Dead-time distortion keyed on duty sign instead of phase-current sign (removes crossover distortion, wrong sign off unity PF) — `electronics-sim/src/active_bridge.rs:123-136` (es-3ph)
- [low] Capacitor ESR (r_esr) stored but never used in either rectifier — no bus ESR ripple or ESR loss — `electronics-sim/src/vienna.rs:45-46, 64, 73` (es-3ph)
- [low] Active bridge rejects only converter common-mode, not grid common-mode → spurious zero-sequence current under unbalanced/harmonic grid — `electronics-sim/src/active_bridge.rs:137-149` (es-3ph)
- [low] Vienna CCM average current uses trapezoidal OFF integral while charge balance uses the exact RLC integral (inconsistent reported i_avg) — `electronics-sim/src/vienna.rs:189-199` (es-3ph)
- [low] FMAC q1.15 I/O scaling sits at the exact i16-overflow cliff edge (target_q115 / y_max wrap for R>=4) — `electronics-sim/src/bin/control-test-fmac.rs:90, 155, 223` (es-bins)
- [low] control-test / control-test-fmac digitize bare v_out, omitting the ESR ripple the ADC (and the limit-cycle criterion) actually see — `electronics-sim/src/bin/control-test.rs:142, 165, 189, 2` (es-bins)
- [low] integrate_phase trip-detection branch corrupts state via in-loop mutation of self.state[0] — `electronics-sim/src/cap_bank.rs:467-491` (es-cap)
- [low] RK4 step-size / stability estimate ignores ESL-branch eigenvalues (self-resonance stiffness) — `electronics-sim/src/cap_bank.rs:157-166` (es-cap)
- [low] esr = 0 on a no-ESL cap causes NaN v_out and a near-infinite RK4 loop (unguarded user input) — `electronics-sim/src/cap_bank.rs:150-166` (es-cap)
- [low] No MLCC DC-bias capacitance derating despite being the stated cap-bank focus — `electronics-sim/src/cap_bank.rs:59-69` (es-cap)
- [low] integrate_on_phase silently omits v_out min/max envelope tracking that integrate_phase performs — `electronics-sim/src/cap_bank.rs:527-580` (es-cap)
- [low] No ripple / RMS capacitor current output (self-heating & lifetime cannot be evaluated) — `electronics-sim/src/cap_bank.rs:320-368` (es-cap)
- [low] Buck ON-phase di/dt neglects the R_series voltage drop (inconsistent with Boost and with the RLC solver) — `electronics-sim/src/lib.rs:362-381` (es-core)
- [low] Cycle v_out min/max sampled at only 3 current points — under-reports capacitance-dominated (low-ESR/MLCC) ripple — `electronics-sim/src/lib.rs:551-562` (es-core)
- [low] OFF-phase inductance evaluated at peak current instead of the OFF-phase midpoint for saturating cores — `electronics-sim/src/lib.rs:489-499` (es-core)
- [low] Boost input-cap ripple/recharge only models ON-phase draw, but a boost source conducts during OFF too — `electronics-sim/src/lib.rs:1008-1020` (es-core)
- [low] Test suite passes while the simulator produces 800 V / 20 kA garbage; assertions are too weak to catch divergence or sign errors — `electronics-sim/src/edge_transient.rs:1159-1387` (es-edge)
- [low] Newton root-finder intersects_at has no convergence/divergence guard, runs a fixed 10 iterations, never returns None, and returns a previous iterate on NaN — `electronics-sim/src/math.rs:14-37` (es-math)
- [low] Near-critical damping in rlc() is numerically ill-conditioned; the `if a == ohmega` guard only triggers on exact float equality — `electronics-sim/src/math.rs:196-213` (es-math)
- [low] Comment misdiagnoses the critical-damping singularity as 's1 = 0' in WonkyF::integral — `electronics-sim/src/math.rs:196-202` (es-math)
- [low] Output-cap ESR (r_esr) is a dead field: v_out() claims to include ESR but returns bare v_cap — `electronics-sim/src/pfc_boost.rs:37,45,72-75,82-1` (es-pfc)
- [low] OFF-phase RLC uses r_series = DCR + Rds(on) even in diode mode, where the main FET is off — `electronics-sim/src/pfc_boost.rs:107-118` (es-pfc)
- [low] tick_crm v_drop clamp produces unphysical multi-second t_off (and giant sim step) when v_cap <= v_in — `electronics-sim/src/pfc_boost.rs:200-210` (es-pfc)
- [low] AcSource is an ideal stiff sinusoid: no grid series impedance or background voltage distortion — `electronics-sim/src/ac_source.rs:8-48` (es-pfc)
- [low] prime() bumpless transfer leaves a residual step (b0+b1+b2)·e, not the documented ≈0 — `full-control/src/control_2p2z.rs:215-226` (fc-2p2z)
- [low] r_esr_out_cap = 0 (ideal cap) produces ω_esr = ∞ and silent NaN 2P2Z coefficients — `full-control/src/control_2p2z.rs:620-621` (fc-2p2z)
- [low] Boost/buck-boost limit-cycle ripple estimate underestimates the ESR component — `full-control/src/control_2p2z.rs:394-400` (fc-2p2z)
- [low] Buck-boost mode hysteresis 'keep current' can latch the wrong extreme mode across a discontinuous V_in/V_out jump — `full-control/src/buck_boost.rs:159-179` (fc-bb-fmac)
- [low] FmacIir::new performs no range check on R; update() shift math underflows/panics for R outside [1,14] — `full-control/src/fmac.rs:51-53, 73-74` (fc-bb-fmac)
- [low] FMAC output rounding is round-half-toward-+∞ and may not match the hardware, which does not round-half in its q1.15 write-back — `full-control/src/fmac.rs:72-74` (fc-bb-fmac)
- [low] Test RL plant applies 2x the commanded converter voltage (duty->voltage scaling inconsistent with controller) — `full-control/src/dq_controller.rs:277-287` (fc-dq)
- [low] Duty computation divides by v_dc with no zero/near-zero guard -> NaN/degenerate duty during precharge/fault — `full-control/src/dq_controller.rs:174-180` (fc-dq)
- [low] No computation/PWM transport delay (~1.5 Ts) modeled in the current loop or its tests — `full-control/src/dq_controller.rs:120-190` (fc-dq)
- [low] rem_pio2 passes wrong term count (`i - 1`) to rem_pio2_large — drops the two low 24-bit chunks, or usize-underflow panic — `full-control/src/math/rem_pio2.rs:182-192` (fc-math1)
- [low] const sqrt uses fixed 100 Newton iterations from a poor x/2 seed — silently wrong for large x, NaN on +∞, and wasteful — `full-control/src/math/mod.rs:36-50` (fc-math1)
- [low] scalbn uses f64::MAX_EXP/MIN_EXP (1024/-1021) where the algorithm expects unbiased EXP_MAX/EXP_MIN (1023/-1022) — off-by-one in the prescale paths — `full-control/src/math/scalbn.rs:27-73` (fc-math1)
- [low] Hand-rolled math::sqrt does not converge outside ~[2^-190, 2^180]; catastrophically wrong for very small/large args and returns NaN for +Inf — `full-control/src/math/mod.rs:36-52` (fc-math2)
- [low] max_inner_crossover_hz reports a crossover ~3× too high because it only checks the zero-phase guard, not delay/gain-margin/limit-cycle — `full-control/src/control_pfc.rs:246-266` (fc-pfc)
- [low] Reused single-phase outer-loop design mis-scales the voltage-loop gain for interleaved/3-phase (Vienna) converters — `full-control/src/control_pfc.rs:192-203` (fc-pfc)
- [low] No amplitude normalization — PLL loop bandwidth and damping scale with grid voltage — `full-control/src/pll.rs:83-95` (fc-pll)
- [low] Democratic sharing loop gain scales with active phase count ((N-1)/N); set_active silently changes loop dynamics without retuning — `full-control/src/current_sharing.rs:55-75` (fc-share)
- [low] set_active() resets ALL integrators, discarding surviving phases' learned mismatch correction (non-bumpless phase shed/add) — `full-control/src/current_sharing.rs:85-93` (fc-share)
- [low] No validation that max_trim >= 0 — negative (or misordered) max_trim makes clamp(min,max) panic with min>max — `full-control/src/current_sharing.rs:29-38` (fc-share)
- [low] Latched UVP/UVLO will false-trip at power-up: no startup blanking or arm-on-first-valid window — `full-control/src/protection.rs:95-144, 184-200` (fc-ss-prot)
- [low] SoftStart::tick adds step before clamping — non-saturating overflow for fixed-point Scalar types — `full-control/src/soft_start.rs:55-67` (fc-ss-prot)
- [low] No load-current feedforward (buck) or 1/V_rms² input feedforward (PFC voltage loop) — `buck-sim-ui/src/sim.rs:sim.rs 1078-1090` (gap-control)
- [low] Computational/transport delay is only a design-time phase penalty — no active delay compensation (predictor/Padé/Smith) — `full-control/src/control_2p2z.rs:control_2p2z.rs ` (gap-control)
- [low] No subharmonic / period-doubling stability-margin reporting despite a genuinely cycle-by-cycle inner model — `full-control/src/control_2p2z.rs:control_2p2z.rs ` (gap-control)
- [low] No state/disturbance observer (Luenberger/Kalman) for sensorless current, load-current, or ESR estimation — `full-control/ (new module):n/a (absent)` (gap-control)
- [low] Loop-gain / Bode analysis is continuous-time + e^{-jωτ}; no true discrete-time (z-domain) sampled-data model or Nyquist — `buck-sim-ui/src/bode.rs:bode.rs 82-111 (` (gap-control)
- [low] No input-filter interaction / Middlebrook (negative-resistance) stability check — `electronics-sim/src/lib.rs:lib.rs:1032-1060` (gap-numerics)
- [low] Vienna / 3-level PFC has no neutral-point (DC-bus midpoint) balancing control — `electronics-sim/src/vienna.rs:vienna.rs:37-58,` (gap-pfc)
- [low] No input-filter interaction (Middlebrook) stability check nor audiosusceptibility transfer function — `buck-sim-ui/src/bode.rs:bode.rs plant/Zo` (gap-product)
- [low] No over-temperature protection (OTP) or thermal-foldback derating in the control library — `full-control/src/protection.rs:179-208` (gap-thermal)
- [low] demote_unmanaged_irqs ignores SCB SHPR: a soft-side SysTick/PendSV left at reset-default 0x00 preempts Tier 1 and cannot be BASEPRI-masked — `hard-rt/src/lib.rs:274-316` (hrt)
- [low] No disconnect detection or error surfacing — a hung device shows 'Waiting for data...' forever — `kwr103-ui/src/main.rs:77-90, 204-216` (kwr)
- [low] Sequence timing drifts because per-frame delta is truncated to whole milliseconds — `kwr103-ui/src/web.rs:281-285` (kwr)
- [low] UI output_enabled flag desyncs from actual device/mock output state — `kwr103-ui/src/main.rs:156-171, 181-202` (kwr)
- [low] Web 'Reset' clears UI state but leaves the mock output latched on and playback running — `kwr103-ui/src/web.rs:178-181` (kwr)
- [info] Sweep extends to f_sw, past the f_sw/2 Nyquist limit where the sampled-data plant (double pole + delay approximations) is invalid — `buck-sim-ui/src/bode.rs:255-256` (bs-bode)
- [info] Short-capture guard is off-by-one relative to its error message ('need at least 16' but 15 cycles passes) — `buck-sim-ui/src/spectrum_export.rs:467-475` (bs-dsp)
- [info] Log-normal perturbation is median-preserving, not mean-preserving: realized parameters are biased high by exp(sigma^2/2), and the '±20% at 1σ' comment is inaccurate — `buck-sim-ui/src/monte_carlo.rs:161-171` (bs-mc)
- [info] Controller output cast to u16 truncates instead of saturating for large dynamic limits — `buck-sim-ui/src/sim.rs:986, 1098` (bs-sim)
- [info] threshold_drive_a doc formula 'I*sqrt(C/L) = V_in/L' is dimensionally wrong (code is correct) — `buck-sim-ui/src/zvs.rs:109-118` (bs-zvs)
- [info] Test comment mis-states T_res/4 as 1.92 ns (actual 3.85 ns) — `buck-sim-ui/src/zvs.rs:350-353` (bs-zvs)
- [info] Active bridge doc comment gives contradictory/incorrect v_conv formula — `electronics-sim/src/active_bridge.rs:96-100` (es-3ph)
- [info] FmacIir rounding shift underflows when min_fmac_r() returns 0 (low-bandwidth / decimated design) — `electronics-sim/src/bin/control-test-fmac.rs:242-243` (es-bins)
- [info] buck-boost-test simulation `time` is never advanced unless the `rerun` or `text-log` feature is on — `electronics-sim/src/bin/buck-boost-test.rs:224-256, 282-292` (es-bins)
- [info] Stale/incorrect operating-point comments in buck-boost-test (2 A @ 12 V, 'holding V_out at 12 V') — `electronics-sim/src/bin/buck-boost-test.rs:68, 376-379` (es-bins)
- [info] Unguarded division in t_on_guess can yield NaN/Inf and panic in intersects_at — `electronics-sim/src/lib.rs:392-395` (es-core)
- [info] Func::intersects_at has no convergence tolerance and swallows non-convergence/NaN as a valid result — `electronics-sim/src/math.rs:14-37` (es-core)
- [info] Newton loop recomputes self.derivative()/rhs.derivative() every iteration; for Polynomial this heap-allocates a Vec on each of ~20 calls per solve — `electronics-sim/src/math.rs:14-34` (es-math)
- [info] Polynomial Debug output is malformed: missing separators, prints the x^1 term even when its coefficient is 0 — `electronics-sim/src/math.rs:288-313` (es-math)
- [info] Boost diode reverse-recovery / hard-switching turn-on loss not represented in the power stage — `electronics-sim/src/pfc_boost.rs:107-139` (es-pfc)
- [info] fabs(-0.0) returns -0.0 (and fabs(NaN) keeps sign) — deviates from IEEE/libm |x| — `full-control/src/math/mod.rs:28-34` (fc-math2)

## GAP SUGGESTIONS (missing physics / features)

- [critical] Inductor/PFC-choke core loss (iGSE/Steinmetz) is entirely absent — magnetics loss = copper DCR only — `buck-sim-ui/src/sim.rs` (missing-physics, gap-magnetics)
- [critical] No electro-thermal self-heating: losses computed at 25 °C nominal Rds(on)/Vf/ESR, so efficiency is systematically optimistic — `buck-sim-ui/src/sim.rs` (missing-physics, gap-thermal)
- [high] Winding AC resistance (skin + proximity / Dowell) missing — conduction loss uses DC resistance against full ripple RMS — `buck-sim-ui/src/sim.rs` (missing-physics, gap-magnetics)
- [high] No electro-thermal self-heating loop — R_ds(on)/DCR/ESR/B_sat temperature dependence ignored — `buck-sim-ui/src/sim.rs` (missing-physics, gap-magnetics)
- [high] Reverse-recovery (Q_rr) and body-diode commutation loss omitted from cycle-level efficiency — Si-vs-SiC-vs-GaN decision cannot be shown — `buck-sim-ui/src/sim.rs` (missing-physics, gap-magnetics)
- [high] Nonlinear Coss(V) energy (Eoss vs Qoss) ignored — flat-Coss 0.5·C·V² and constant-Coss ZVS tank misprice HV/GaN switching — `buck-sim-ui/src/sim.rs` (improvement, gap-magnetics)
- [high] No magnetic core loss (Steinmetz/iGSE) or AC winding loss (skin/proximity) in the loss model — `buck-sim-ui/src/sim.rs` (missing-physics, gap-numerics)
- [high] No cold-start inrush / pre-charge (NTC / bypass-relay) current modeling — `buck-sim-ui/src/pfc_sim.rs` (missing-physics, gap-pfc)
- [high] AC source cannot produce distorted, unbalanced, or faulted grid conditions — `electronics-sim/src/ac_source.rs` (missing-physics, gap-pfc)
- [high] PFC efficiency model omits switching, reverse-recovery, and core losses — `buck-sim-ui/src/pfc_sim.rs` (missing-physics, gap-pfc)
- [high] No electro-thermal coupling: losses are computed isothermally and never feed back into R_ds(on)/ESR/DCR/T_j — `buck-sim-ui/src/sim.rs` (missing-physics, gap-product)
- [high] No magnetic core loss (Steinmetz/iGSE) or AC winding loss anywhere — inductor loss is DCR-copper only — `buck-sim-ui/src/sim.rs` (missing-physics, gap-product)
- [high] No general parametric-sweep or deterministic worst-case-corner engine surfaced in the workflow (Monte Carlo exists but is not wired into the app) — `buck-sim-ui/src/monte_carlo.rs` (feature, gap-product)
- [high] Golden test-vector / reference-fixture generation for the firmware codegen bridge is absent — `buck-sim-ui/src/inner_ctrl.rs` (feature, gap-product)
- [high] No thermal RC network (Foster/Cauer Zth) — cannot model transient junction temperature, load-step heating, or short-term overload ratings — `buck-sim-ui/src/sim.rs` (missing-physics, gap-thermal)
- [high] PFC loss budget omits FET switching loss and boost-diode reverse-recovery — the dominant loss in hard-switched bridged-boost, and the whole point of SiC / totem-pole — `buck-sim-ui/src/pfc_sim.rs` (missing-physics, gap-thermal)
- [high] Inductor loss is DCR-only — no magnetic core loss (Steinmetz) and no AC winding loss (skin/proximity, Rac(f)) — `buck-sim-ui/src/sim.rs` (missing-physics, gap-thermal)
- [medium] CISPR verdict checks only the quasi-peak limit; the (10-13 dB lower) average-detector limit is never evaluated — `kicad_field_solver/pdn_emc/src/lib.rs` (missing-physics, bs-pdn)
- [medium] LISN model is single-ended: no common-mode / differential-mode split, so it can miss the dominant conducted-EMC mechanism — `kicad_field_solver/pdn_emc/src/lib.rs` (missing-physics, bs-pdn)
- [medium] No negative-sequence / harmonic rejection — 2ω ripple passes straight to angle under unbalanced or distorted grid — `full-control/src/pll.rs` (missing-physics, fc-pll)
- [medium] No fault/outlier rejection — a stuck or saturated phase corrupts the mean and drags all healthy phases — `full-control/src/current_sharing.rs` (missing-physics, fc-share)
- [medium] ProtectionSet lacks OTP (over-temperature) and hiccup has no retry-count / fold-back limiter — `full-control/src/protection.rs` (missing-physics, fc-ss-prot)
- [medium] SoftStart snaps the reference on downward target changes and on pre_bias>target — not bumpless, and can command sink-current inrush on a synchronous converter — `full-control/src/soft_start.rs` (missing-physics, fc-ss-prot)
- [medium] No nonlinear/large-signal control modes: NLC/one-cycle PFC and hysteretic/constant-on-time buck are absent — `full-control/src/pfc_runtime.rs` (missing-physics, gap-control)
- [medium] No source/load impedance (Middlebrook) stability check or audiosusceptibility, despite a modeled input filter; PFC outer loop assumes an ideal inner loop — `buck-sim-ui/src/sim.rs` (feature, gap-control)
- [medium] Dead-time body-diode loss uses a constant V_F — GaN/SiC third-quadrant (reverse-conduction) loss under-modelled — `electronics-sim/src/lib.rs` (improvement, gap-magnetics)
- [medium] Saturating-inductor di/dt uses secant (apparent) inductance instead of incremental (differential) inductance — `electronics-sim/src/lib.rs` (improvement, gap-magnetics)
- [medium] No capacitor ripple-current self-heating / RMS-rating / lifetime check — `buck-sim-ui/src/sim.rs` (feature, gap-magnetics)
- [medium] No electrothermal coupling — Rds_on/ESR/core-loss are temperature-independent — `buck-sim-ui/src/sim.rs` (missing-physics, gap-numerics)
- [medium] No reactive-power / non-unity-PF command, and PF metric doesn't separate displacement vs distortion — `buck-sim-ui/src/pfc_sim.rs` (feature, gap-pfc)
- [medium] Conducted-EMC covers only differential mode; no common-mode path for bridgeless/totem-pole PFC — `buck-sim-ui/src/conducted_emc.rs` (missing-physics, gap-pfc)
- [medium] No standardized data export: time-domain waveform CSV and Bode/impedance Touchstone (.sNp) are missing — `buck-sim-ui/src/app.rs` (feature, gap-product)
- [medium] No consolidated design/margin sign-off report with pass/fail against user spec limits — `buck-sim-ui/src/app.rs` (feature, gap-product)
- [medium] Boost/Vienna diode modeled as a constant Vf — no I-V curve (Vf0 + Rd·I) and no temperature dependence, biasing both loss and the DCM boundary — `electronics-sim/src/pfc_boost.rs` (missing-physics, gap-thermal)
- [medium] No efficiency/loss map over the operating plane (Vin × Iout) and no fsw sweep to locate the conduction/switching-loss optimum — `buck-sim-ui/src/sim.rs` (feature, gap-thermal)
- [medium] No capacitor self-heating from ripple current / ESR, nor electrolytic-lifetime (Arrhenius) derating — `buck-sim-ui/src/sim.rs` (feature, gap-thermal)
- [low] Load-step R_load overlay hard-codes 3000/2000 cycle counts that silently duplicate SOFT_START+STEADY / LOAD_STEP constants — `buck-sim-ui/src/app.rs` (improvement, bs-app)
- [low] `transitions` Vec grows unbounded and is fully rescanned every sample → O(m·n_cycles^2) ring cost with no pruning — `buck-sim-ui/src/spectrum_export.rs` (improvement, bs-dsp)
- [low] No PDN target-impedance (Z_target = ΔV_ripple / ΔI_step) mask in the |Z_out| panel — `buck-sim-ui/src/bode.rs` (feature, bs-pdn)
- [low] design_summary() and to_2p2z() duplicate the full pole/zero derivation and can silently diverge — `full-control/src/control_2p2z.rs` (improvement, fc-2p2z)
- [low] Sinusoidal PWM only (no SVPWM/3rd-harmonic injection); hard per-phase [0,1] clamp distorts the voltage vector under saturation — `full-control/src/dq_controller.rs` (missing-physics, fc-dq)
- [low] Cross-coupling decoupling is off by default and uses noisy measured currents + instantaneous PLL omega — `full-control/src/dq_controller.rs` (improvement, fc-dq)
- [low] Initial phasor is π/2 away from the lock angle for the documented sin convention, causing an avoidable acquisition transient — `full-control/src/pll.rs` (improvement, fc-pll)
- [low] No lock-detection / phase-error output and no back-calculation anti-windup — `full-control/src/pll.rs` (feature, fc-pll)
- [low] No droop / load-line sharing option and no sharing-vs-voltage-loop bandwidth guard — `full-control/src/current_sharing.rs` (feature, fc-share)
- [low] SoftStart has no enable gate; Idle auto-starts on the first tick() and is behaviorally identical to Ramping — `full-control/src/soft_start.rs` (improvement, fc-ss-prot)
- [low] protection: recovery_counter is dead/redundant state; the countdown is driven entirely by the enum payload — `full-control/src/protection.rs` (improvement, fc-ss-prot)
- [low] Conducted-EMI ignores real switching-edge harmonic content and has no differential/common-mode split — `buck-sim-ui/src/conducted_emc.rs` (feature, gap-numerics)
- [low] Demotion target 0xF0 is equal to, not below, RTIC priority-1 dispatchers — doc/comment overstates the separation — `hard-rt/src/lib.rs` (improvement, hrt)
- [low] DSB/ISB on every acquire and release are heavier than needed; adds deterministic pipeline-flush latency to every Tier 2 critical section — `hard-rt/src/lib.rs` (improvement, hrt)
- [info] buck_boost::update doc-comment describes a 4-tuple return that does not exist — `full-control/src/buck_boost.rs` (improvement, fc-bb-fmac)
- [info] Inverse Clarke silently assumes zero-sequence = 0 (3-wire only); round-trip exact only for balanced inputs — `full-control/src/transforms.rs` (improvement, fc-dq)
- [info] `pow2(x) = x*x` is a squaring helper but reads as 2^x — foot-gun in controller math — `full-control/src/math/mod.rs` (improvement, fc-math1)
- [info] Inner-loop plant modeled as ideal V_out/(sL) omits the sampled-data half-switching-frequency dynamics — `full-control/src/control_pfc.rs` (missing-physics, fc-pfc)

## REFUTED (verifier rejected — ignore)

- Output-cap 'Ripple I_rms' applies a single-triangle ΔI/(2√3) to the summed multiphase inductor current, overestimating cap RMS for N>1 — `buck-sim-ui/src/app.rs:1476-1482` (bs-app) — _Refuted on both its factual evidence and its physics conclusion. (1) `worst_pp` (app.rs:1458-1460, 1470-1472) is `p.i_total_max - p.i_total_min`, where those fields come from `interleaved_envelope()` in sim.rs:894-940. T_
- Broadband spectral content (Qrr sinc, edge harmonics) normalized by coherent gain, not ENBW — levels are window/FFT-length dependent and not comparable across captures — `buck-sim-ui/src/spectrum_export.rs:564-598` (bs-dsp) — _Code confirmed at the cited lines: `coherent_gain = hann.iter().sum()/n_samples` (572), `norm = 1.0/(n_samples*coherent_gain)` (585), `scale = if k==0||k*2==n_samples {1.0} else {2.0}` (593). This is a one-sided AMPLITUD_
- Tabulated PDN Z(ω) is interpolated linearly in Re/Im between log-spaced samples, badly under-resolving anti-resonance peaks — `kicad_field_solver/pdn_schema/src/lib.rs:108-148` (bs-pdn) — _Code reading is accurate. In /home/albin/my_projects/kicad_field_solver/pdn_schema/src/lib.rs, z_at (lines 108-148) does compute t linearly in log-frequency (lines 135-143) and then interpolates Re and Im independently a_
- PDN sub-range extrapolation drops the ESR floor (returns Re(Z)=0 below the lowest sample) — `kicad_field_solver/pdn_schema/src/lib.rs:115-121, 159-175` (bs-pdn) — _Located the actual code at /home/albin/my_projects/kicad_field_solver/pdn_schema/src/lib.rs (a path-dep sibling of the repo, reachable via buck-sim-ui/Cargo.toml `pdn-schema = { path = "../../kicad_field_solver/pdn_schem_
- FmacIir::update panics if y_min > y_max (misconfigured limits) — `full-control/src/fmac.rs:77` (fc-bb-fmac) — _Read full-control/src/fmac.rs. The literal code facts are accurate: line 77 `let y_out = y_raw.clamp(self.y_min as i64, self.y_max as i64) as i16;` uses Rust's `Ord::clamp`, which does `assert!(min <= max)` and thus pani_
- Outer voltage loop provides weak 2nd-harmonic rejection: a single real pole at exactly 2·f_line, no notch — `full-control/src/control_pfc.rs:201-206` (fc-pfc) — _Read control_pfc.rs lines 96-244 plus reference params (lines 274-290). The outer voltage loop (lines 201-215) builds a Type-II compensator on a pure-integrator plant G_vi=plant_gain/(s·C_out) (lines 197,203), giving a d_
- max_freq_dev_hz does not bound the estimated frequency/angle rate — proportional path bypasses the clamp — `full-control/src/pll.rs:88-95` (fc-pll) — _Factual core is accurate: pll.rs:89-92 clamps only `self.integrator` to ±omega_max_dev; line 94 then forms `omega_correction = self.kp * v_q + self.integrator` and line 95 sets `self.omega = self.omega_nominal + omega_co_
- Small-angle sin approximation (sin_d = dθ, no −dθ³/6) biases the reported frequency slightly low — `full-control/src/pll.rs:99-103` (fc-pll) — _Read full-control/src/pll.rs lines 99-103: cos_d = 1−½dθ² (2nd order), sin_d = dθ (1st order). The claim's core math is CORRECT: the actual per-step rotation is φ = atan2(dθ, 1−½dθ²) = dθ + dθ³/6, an order-mismatch over-_
- Per-phase clamp breaks the zero-sum property, injecting a common-mode duty that perturbs the voltage loop / total output current — `full-control/src/current_sharing.rs:63-75` (fc-share) — _Read full file full-control/src/current_sharing.rs. The reviewer's math is partly right but the impact and fix are overstated/wrong. (1) The unclamped-zero-sum premise is correct: error_i = mean - currents[i] (line 66) s_
- Integrator anti-windup is bare clamping to the full output limit — no back-calculation/conditional integration, leaves no proportional headroom — `full-control/src/current_sharing.rs:68-74` (fc-share) — _Verified against full-control/src/current_sharing.rs. Evidence quote is accurate: line 70 `self.integrators[i] = self.integrators[i].clamp(neg_max, self.max_trim)` and line 74 `trims[i] = trim.clamp(neg_max, self.max_tri_
- protection: no hysteresis / no auto-recover-on-safe mode, and strict >/< means a value sitting exactly at threshold never trips — `full-control/src/protection.rs:163-176` (fc-ss-prot) — _Both factual observations are literally correct, but neither is an actionable defect. (1) FaultAction (lines 5-10) truly has only Latch and Hiccup — no hysteresis-band / AutoClear-on-safe variant, and FaultMonitor (lines_
- dq PFC current controller is pure PI — no resonant (PR) harmonic compensators, no 2ω DC-bus notch, no negative-sequence/DSOGI path — `full-control/src/dq_controller.rs:dq_controller.rs` (gap-control) — _The factual absences are real but the framing/severity are inflated and one pillar already exists elsewhere. Verified: dq_controller.rs PiController (lines 20-50) is a scalar PI (`self.integrator = self.integrator + self_
- No periodic-steady-state (shooting/Aprille-Trick) solver — steady state found by brute-force 1500+ switching cycles — `buck-sim-ui/src/sim.rs:sim.rs:255-259, ` (gap-numerics) — _The absence is factually true — grep finds no shooting/aprille/jacobian solver (the two 'Newton' hits, electronics-sim/src/lib.rs:618 and :1094, are an ideal-diode line intersection and a libm iteration, unrelated), and _
- Displayed small-signal plant is always CCM and omits the RHP zero for boost/buck-boost/PFC — `buck-sim-ui/src/bode.rs:bode.rs:157-172 ` (gap-numerics) — _The claim's literal code reading is correct but its impact is overstated and mis-scoped to the cited file.

POINT 2 (RHP zero) — refuted for the cited code. bode.rs:160-172 plant() genuinely builds only h_dc·(1+jω/ω_esr)_
- Fixed-step, stability-bounded integrators without error control; approximate switching-event detection in cap_bank — `electronics-sim/src/cap_bank.rs:cap_bank.rs:445-` (gap-numerics) — _The claim reads the LITERAL code correctly (the comments and Euler doc all exist), but its impact/severity framing is refuted on every load-bearing point.

(a) cap_bank integrate_phase trip detector. The messy block at c_
- SRF-PLL and dq control have no negative-/positive-sequence separation for unbalanced grid — `full-control/src/pll.rs:pll.rs:83-120` (gap-pfc) — _The absence is factually confirmed and the control theory is sound, but framing it as a medium-severity issue overstates it; the code is a correct standard design for its documented (balanced-grid) scope, and no unbalanc_
- Voltage loop lacks 2·f_line ripple rejection (notch / feedforward) and there is no hold-up-time analysis — `full-control/src/control_pfc.rs:control_pfc.rs:2` (gap-pfc) — _All factual assertions check out, but the framing overstates them into a "medium" issue. Facts confirmed: control_pfc.rs:206 hard-codes `outer_omega_cp1 = 2.0*PI*(2.0*self.f_line)` ("Pole at 2×f_line for 2nd harmonic rip_
- Bode/loop-gain is the design-time analytical TF, never the measured response of the actual nonlinear closed-loop sim (no injected-signal FRA) — `buck-sim-ui/src/bode.rs:bode.rs plant 16` (gap-product) — _The absence half of the claim is factually true: the Bode is a closed-form analytic small-signal loop gain, not a measured FRA of the switched sim. plant() (bode.rs:160-172), plant_pdn() (186-199) and compensator() (204-_
- Global BASEPRI critical_section::Impl breaks the crate-wide "no two CS active at once" guarantee whenever Tier 1 re-enters a critical_section primitive (e.g. defmt) — `hard-rt/src/lib.rs:172-205` (hrt) — _The mechanical description is accurate but the "high-severity bug" framing is wrong: this is the crate's intentional, explicitly-documented design, and its flagship concrete hazard (defmt) is already engineered away.

CO_
- demote_unmanaged_irqs cannot distinguish a hard-RT (or deliberately-0x00) IRQ from an unmanaged soft IRQ; a not-yet-prioritized Tier 1 ISR gets silently demoted to lowest priority — `hard-rt/src/lib.rs:306-314` (hrt) — _Read hard-rt/src/lib.rs lines 224-316. The claim's central premise — that the precondition ("every IRQ you care about must already be non-zero") is not conveyed and the safety comment "only argues writing can only lower _
- MockPowerSupply.state is Arc-wrapped and reassigned, so cloned handles silently diverge — `kwr103-ui/src/lib.rs:70-93, 122-147` (kwr) — _The code quote is accurate. lib.rs:72 declares `state: Arc<PowerSupplyState>`, and refresh() at lib.rs:145 does `self.state = Arc::new(PowerSupplyState { ... })`, which rebinds the field rather than mutating shared data._