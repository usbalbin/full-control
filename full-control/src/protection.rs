use crate::control_2p2z::Scalar;

/// What to do when a fault triggers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FaultAction {
    /// Latch off — requires explicit `clear()` to restart.
    Latch,
    /// Hiccup — wait `recovery_ticks` then auto-retry.
    Hiccup { recovery_ticks: u32 },
}

/// Current state of a fault monitor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FaultState {
    /// Normal operation.
    Ok,
    /// Fault detected, output disabled.
    Faulted,
    /// Hiccup recovery countdown.
    Recovering { ticks_remaining: u32 },
}

impl FaultState {
    /// Returns the "worst" of two states: Faulted > Recovering > Ok.
    fn worst(self, other: FaultState) -> FaultState {
        match (self, other) {
            (FaultState::Faulted, _) | (_, FaultState::Faulted) => FaultState::Faulted,
            (FaultState::Recovering { ticks_remaining: a }, FaultState::Recovering { ticks_remaining: b }) => {
                FaultState::Recovering {
                    ticks_remaining: if a > b { a } else { b },
                }
            }
            (FaultState::Recovering { ticks_remaining }, _)
            | (_, FaultState::Recovering { ticks_remaining }) => {
                FaultState::Recovering { ticks_remaining }
            }
            _ => FaultState::Ok,
        }
    }
}

/// Single fault channel — monitors one value against upper and/or lower thresholds.
///
/// Includes configurable debounce (the value must exceed the threshold for
/// `debounce_ticks` consecutive ticks before the fault triggers).
pub struct FaultMonitor<S: Scalar> {
    /// Upper threshold (fault if value > threshold). None = disabled.
    upper: Option<S>,
    /// Lower threshold (fault if value < threshold). None = disabled.
    lower: Option<S>,
    /// Number of consecutive ticks above/below threshold before fault triggers.
    debounce_ticks: u32,
    /// What to do on fault.
    action: FaultAction,

    // Internal state
    state: FaultState,
    debounce_counter: u32,
    recovery_counter: u32,
}

impl<S: Scalar> FaultMonitor<S> {
    /// Creates a monitor with no thresholds.
    pub fn new(action: FaultAction) -> Self {
        Self {
            upper: None,
            lower: None,
            debounce_ticks: 1,
            action,
            state: FaultState::Ok,
            debounce_counter: 0,
            recovery_counter: 0,
        }
    }

    /// Set the upper threshold (fault if value > threshold).
    pub fn with_upper(mut self, threshold: S) -> Self {
        self.upper = Some(threshold);
        self
    }

    /// Set the lower threshold (fault if value < threshold).
    pub fn with_lower(mut self, threshold: S) -> Self {
        self.lower = Some(threshold);
        self
    }

    /// Set the debounce count (consecutive ticks before fault triggers).
    pub fn with_debounce(mut self, ticks: u32) -> Self {
        self.debounce_ticks = ticks;
        self
    }

    /// Main per-tick method: feed the current value and advance the state machine.
    pub fn check(&mut self, value: S) -> FaultState {
        match self.state {
            FaultState::Ok => {
                let exceeded = self.threshold_exceeded(value);
                if exceeded {
                    self.debounce_counter += 1;
                    if self.debounce_counter >= self.debounce_ticks {
                        self.debounce_counter = 0;
                        self.state = FaultState::Faulted;
                        match self.action {
                            FaultAction::Hiccup { recovery_ticks } => {
                                self.recovery_counter = recovery_ticks;
                                self.state = FaultState::Faulted;
                            }
                            FaultAction::Latch => {
                                self.state = FaultState::Faulted;
                            }
                        }
                    }
                } else {
                    self.debounce_counter = 0;
                }
            }
            FaultState::Faulted => {
                match self.action {
                    FaultAction::Latch => {
                        // Stay faulted until clear() is called.
                    }
                    FaultAction::Hiccup { recovery_ticks } => {
                        self.recovery_counter = recovery_ticks;
                        self.state = FaultState::Recovering {
                            ticks_remaining: recovery_ticks,
                        };
                    }
                }
            }
            FaultState::Recovering { ticks_remaining } => {
                if ticks_remaining <= 1 {
                    self.recovery_counter = 0;
                    self.state = FaultState::Ok;
                } else {
                    self.recovery_counter = ticks_remaining - 1;
                    self.state = FaultState::Recovering {
                        ticks_remaining: ticks_remaining - 1,
                    };
                }
            }
        }
        self.state
    }

    /// Current fault state.
    pub fn state(&self) -> FaultState {
        self.state
    }

    /// Returns `true` if the monitor is in the `Ok` state.
    pub fn is_ok(&self) -> bool {
        self.state == FaultState::Ok
    }

    /// Reset to `Ok` (for latched faults).
    pub fn clear(&mut self) {
        self.state = FaultState::Ok;
        self.debounce_counter = 0;
        self.recovery_counter = 0;
    }

    /// Check if the value exceeds any configured threshold.
    fn threshold_exceeded(&self, value: S) -> bool {
        if let Some(upper) = self.upper {
            if value > upper {
                return true;
            }
        }
        if let Some(lower) = self.lower {
            if value < lower {
                return true;
            }
        }
        false
    }
}

/// Combined OCP + OVP + UVP monitor.
pub struct ProtectionSet<S: Scalar> {
    /// Over-Current Protection.
    pub ocp: FaultMonitor<S>,
    /// Over-Voltage Protection.
    pub ovp: FaultMonitor<S>,
    /// Under-Voltage Protection (input UVLO).
    pub uvp: FaultMonitor<S>,
}

impl<S: Scalar> ProtectionSet<S> {
    /// Check all monitors and return the worst fault state.
    ///
    /// - `i_out` is checked against OCP
    /// - `v_out` is checked against OVP
    /// - `v_in`  is checked against UVP
    pub fn check(&mut self, i_out: S, v_out: S, v_in: S) -> FaultState {
        let ocp_state = self.ocp.check(i_out);
        let ovp_state = self.ovp.check(v_out);
        let uvp_state = self.uvp.check(v_in);
        ocp_state.worst(ovp_state).worst(uvp_state)
    }

    /// Clear all fault monitors.
    pub fn clear_all(&mut self) {
        self.ocp.clear();
        self.ovp.clear();
        self.uvp.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Test 1: Upper threshold trips after debounce ----
    #[test]
    fn upper_threshold_trips_after_debounce() {
        let mut mon = FaultMonitor::<f32>::new(FaultAction::Latch)
            .with_upper(10.0)
            .with_debounce(3);

        // First two ticks above threshold: still Ok (debounce not reached)
        assert_eq!(mon.check(11.0), FaultState::Ok);
        assert_eq!(mon.check(11.0), FaultState::Ok);
        // Third tick: debounce reached, fault triggers
        assert_eq!(mon.check(11.0), FaultState::Faulted);
    }

    // ---- Test 2: Lower threshold trips after debounce ----
    #[test]
    fn lower_threshold_trips_after_debounce() {
        let mut mon = FaultMonitor::<f32>::new(FaultAction::Latch)
            .with_lower(5.0)
            .with_debounce(2);

        // First tick below threshold: still Ok
        assert_eq!(mon.check(4.0), FaultState::Ok);
        // Second tick: fault triggers
        assert_eq!(mon.check(3.0), FaultState::Faulted);
    }

    // ---- Test 3: Debounce resets when value returns to safe range ----
    #[test]
    fn debounce_resets_on_safe_value() {
        let mut mon = FaultMonitor::<f32>::new(FaultAction::Latch)
            .with_upper(10.0)
            .with_debounce(3);

        // Two ticks above threshold
        assert_eq!(mon.check(11.0), FaultState::Ok);
        assert_eq!(mon.check(11.0), FaultState::Ok);
        // One tick back in safe range: counter resets
        assert_eq!(mon.check(9.0), FaultState::Ok);
        // Need three consecutive ticks again
        assert_eq!(mon.check(11.0), FaultState::Ok);
        assert_eq!(mon.check(11.0), FaultState::Ok);
        assert_eq!(mon.check(11.0), FaultState::Faulted);
    }

    // ---- Test 4: Latch stays faulted until clear() ----
    #[test]
    fn latch_stays_faulted_until_clear() {
        let mut mon = FaultMonitor::<f32>::new(FaultAction::Latch)
            .with_upper(10.0)
            .with_debounce(1);

        assert_eq!(mon.check(11.0), FaultState::Faulted);
        // Stays faulted even with safe values
        assert_eq!(mon.check(5.0), FaultState::Faulted);
        assert_eq!(mon.check(5.0), FaultState::Faulted);
        // Explicit clear
        mon.clear();
        assert_eq!(mon.check(5.0), FaultState::Ok);
    }

    // ---- Test 5: Hiccup auto-recovers after recovery_ticks ----
    #[test]
    fn hiccup_auto_recovers() {
        let mut mon = FaultMonitor::<f32>::new(FaultAction::Hiccup { recovery_ticks: 3 })
            .with_upper(10.0)
            .with_debounce(1);

        // Trip the fault
        assert_eq!(mon.check(11.0), FaultState::Faulted);
        // Next tick: transitions to Recovering
        assert_eq!(
            mon.check(5.0),
            FaultState::Recovering { ticks_remaining: 3 }
        );
        assert_eq!(
            mon.check(5.0),
            FaultState::Recovering { ticks_remaining: 2 }
        );
        assert_eq!(
            mon.check(5.0),
            FaultState::Recovering { ticks_remaining: 1 }
        );
        // Recovery complete
        assert_eq!(mon.check(5.0), FaultState::Ok);
    }

    // ---- Test 6: ProtectionSet returns worst state ----
    #[test]
    fn protection_set_returns_worst_state() {
        let mut prot = ProtectionSet {
            ocp: FaultMonitor::new(FaultAction::Latch)
                .with_upper(15.0)
                .with_debounce(1),
            ovp: FaultMonitor::new(FaultAction::Hiccup { recovery_ticks: 5 })
                .with_upper(14.0)
                .with_debounce(1),
            uvp: FaultMonitor::new(FaultAction::Latch)
                .with_lower(8.0)
                .with_debounce(1),
        };

        // All safe
        assert_eq!(prot.check(10.0, 12.0, 24.0), FaultState::Ok);

        // OVP trips (hiccup), OCP and UVP are Ok => worst is Faulted
        assert_eq!(prot.check(10.0, 15.0, 24.0), FaultState::Faulted);

        // OVP transitions to Recovering, others still Ok => worst is Recovering
        let state = prot.check(10.0, 12.0, 24.0);
        assert!(matches!(state, FaultState::Recovering { .. }));

        // Now trip UVP (latch) => worst is Faulted
        assert_eq!(prot.check(10.0, 12.0, 5.0), FaultState::Faulted);

        // Clear all
        prot.clear_all();
        assert_eq!(prot.check(10.0, 12.0, 24.0), FaultState::Ok);
    }

    // ---- Test 7: No thresholds = always Ok ----
    #[test]
    fn no_thresholds_always_ok() {
        let mut mon = FaultMonitor::<f32>::new(FaultAction::Latch);

        assert_eq!(mon.check(0.0), FaultState::Ok);
        assert_eq!(mon.check(1000.0), FaultState::Ok);
        assert_eq!(mon.check(-1000.0), FaultState::Ok);
        assert!(mon.is_ok());
    }

    // ---- Test 8: Test with f32 scalar operations ----
    #[test]
    fn test_with_f32_scalar() {
        use crate::control_2p2z::Scalar;

        let threshold: f32 = Scalar::from_f32(25.0);
        let mut mon = FaultMonitor::<f32>::new(FaultAction::Hiccup { recovery_ticks: 2 })
            .with_upper(threshold)
            .with_lower(Scalar::from_f32(1.0))
            .with_debounce(2);

        // Safe value
        let safe: f32 = Scalar::from_f32(12.0);
        assert_eq!(mon.check(safe), FaultState::Ok);

        // Above upper, debounce once
        let high: f32 = Scalar::from_f32(30.0);
        assert_eq!(mon.check(high), FaultState::Ok);
        // Trip
        assert_eq!(mon.check(high), FaultState::Faulted);

        // Hiccup recovery
        assert_eq!(
            mon.check(safe),
            FaultState::Recovering { ticks_remaining: 2 }
        );
        assert_eq!(
            mon.check(safe),
            FaultState::Recovering { ticks_remaining: 1 }
        );
        assert_eq!(mon.check(safe), FaultState::Ok);

        // Now trip via lower threshold
        let low: f32 = Scalar::from_f32(0.5);
        assert_eq!(mon.check(low), FaultState::Ok); // debounce 1
        assert_eq!(mon.check(low), FaultState::Faulted); // debounce 2 => trip
    }
}
