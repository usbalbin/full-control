use crate::control_2p2z::Scalar;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SoftStartState {
    /// Waiting to start. Output = pre-bias or zero.
    Idle,
    /// Ramping from initial to target.
    Ramping,
    /// Ramp complete. Output = target.
    Done,
}

/// Firmware-reusable soft-start ramp generator.
///
/// Ramps a reference value from an initial level to a target at a configurable
/// slew rate. Supports pre-bias (starting from a non-zero output voltage when
/// the capacitor is already partially charged).
pub struct SoftStart<S: Scalar> {
    state: SoftStartState,
    output: S,
    target: S,
    /// Increment per tick (= slew_rate * dt, precomputed).
    step: S,
}

impl<S: Scalar> SoftStart<S> {
    /// Create a new soft-start ramp.
    ///
    /// - `target`: final reference value
    /// - `slew_per_tick`: how much to increase per call to `tick()` (= V/s * T_sw)
    /// - `pre_bias`: starting level (0 if cap is discharged, measured V_out if pre-charged)
    pub fn new(target: S, slew_per_tick: S, pre_bias: S) -> Self {
        if pre_bias >= target {
            Self {
                state: SoftStartState::Done,
                output: target,
                target,
                step: slew_per_tick,
            }
        } else {
            Self {
                state: SoftStartState::Idle,
                output: pre_bias,
                target,
                step: slew_per_tick,
            }
        }
    }

    /// Advance one tick. Returns the current reference value.
    pub fn tick(&mut self) -> S {
        match self.state {
            SoftStartState::Idle => {
                self.state = SoftStartState::Ramping;
                self.output = self.output + self.step;
                if self.output >= self.target {
                    self.output = self.target;
                    self.state = SoftStartState::Done;
                }
                self.output
            }
            SoftStartState::Ramping => {
                self.output = self.output + self.step;
                if self.output >= self.target {
                    self.output = self.target;
                    self.state = SoftStartState::Done;
                }
                self.output
            }
            SoftStartState::Done => self.target,
        }
    }

    /// Current state.
    pub fn state(&self) -> SoftStartState {
        self.state
    }

    /// Current output value.
    pub fn output(&self) -> S {
        self.output
    }

    /// Reset to idle with a new pre-bias.
    pub fn reset(&mut self, pre_bias: S) {
        if pre_bias >= self.target {
            self.state = SoftStartState::Done;
            self.output = self.target;
        } else {
            self.state = SoftStartState::Idle;
            self.output = pre_bias;
        }
    }

    /// Change the target (e.g., for output voltage adjustment).
    /// If already Done and new target > output, re-enters Ramping.
    /// If new target < current output, immediately snaps to target (no ramp-down).
    pub fn set_target(&mut self, target: S) {
        self.target = target;
        if target > self.output {
            if self.state == SoftStartState::Done {
                self.state = SoftStartState::Ramping;
            }
        } else {
            self.output = target;
            self.state = SoftStartState::Done;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_ramp_zero_to_one() {
        let mut ss = SoftStart::<f32>::new(1.0, 0.1, 0.0);
        assert_eq!(ss.state(), SoftStartState::Idle);
        assert_eq!(ss.output(), 0.0);

        // First tick transitions Idle -> Ramping, output = 0.1
        let v = ss.tick();
        assert!((v - 0.1).abs() < 1e-6);
        assert_eq!(ss.state(), SoftStartState::Ramping);

        // Ticks 2..9: output increments by 0.1 each
        for i in 2..=9 {
            let v = ss.tick();
            let expected = 0.1 * i as f32;
            assert!((v - expected).abs() < 1e-5, "tick {i}: got {v}, expected {expected}");
            assert_eq!(ss.state(), SoftStartState::Ramping);
        }

        // Tick 10: output reaches 1.0, clamped to target, Done
        let v = ss.tick();
        assert!((v - 1.0).abs() < 1e-6);
        assert_eq!(ss.state(), SoftStartState::Done);

        // Further ticks stay at target
        let v = ss.tick();
        assert!((v - 1.0).abs() < 1e-6);
        assert_eq!(ss.state(), SoftStartState::Done);
    }

    #[test]
    fn pre_bias_partial() {
        let mut ss = SoftStart::<f32>::new(1.0, 0.1, 0.8);
        assert_eq!(ss.state(), SoftStartState::Idle);
        assert!((ss.output() - 0.8).abs() < 1e-6);

        // Tick 1: 0.8 + 0.1 = 0.9
        let v = ss.tick();
        assert!((v - 0.9).abs() < 1e-6);
        assert_eq!(ss.state(), SoftStartState::Ramping);

        // Tick 2: 0.9 + 0.1 = 1.0 -> Done
        let v = ss.tick();
        assert!((v - 1.0).abs() < 1e-6);
        assert_eq!(ss.state(), SoftStartState::Done);
    }

    #[test]
    fn pre_bias_above_target() {
        let ss = SoftStart::<f32>::new(1.0, 0.1, 1.5);
        assert_eq!(ss.state(), SoftStartState::Done);
        assert!((ss.output() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn pre_bias_equal_to_target() {
        let ss = SoftStart::<f32>::new(1.0, 0.1, 1.0);
        assert_eq!(ss.state(), SoftStartState::Done);
        assert!((ss.output() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn set_target_higher_while_done() {
        let mut ss = SoftStart::<f32>::new(1.0, 0.5, 0.0);

        // Run to completion
        while ss.state() != SoftStartState::Done {
            ss.tick();
        }
        assert!((ss.output() - 1.0).abs() < 1e-6);

        // Raise target
        ss.set_target(2.0);
        assert_eq!(ss.state(), SoftStartState::Ramping);
        assert!((ss.output() - 1.0).abs() < 1e-6);

        // Tick: 1.0 + 0.5 = 1.5
        let v = ss.tick();
        assert!((v - 1.5).abs() < 1e-6);
        assert_eq!(ss.state(), SoftStartState::Ramping);

        // Tick: 1.5 + 0.5 = 2.0 -> Done
        let v = ss.tick();
        assert!((v - 2.0).abs() < 1e-6);
        assert_eq!(ss.state(), SoftStartState::Done);
    }

    #[test]
    fn set_target_lower_snaps() {
        let mut ss = SoftStart::<f32>::new(2.0, 0.5, 0.0);

        // Run to completion
        while ss.state() != SoftStartState::Done {
            ss.tick();
        }
        assert!((ss.output() - 2.0).abs() < 1e-6);

        // Lower target: snaps immediately
        ss.set_target(0.5);
        assert_eq!(ss.state(), SoftStartState::Done);
        assert!((ss.output() - 0.5).abs() < 1e-6);

        // Tick stays at new target
        let v = ss.tick();
        assert!((v - 0.5).abs() < 1e-6);
        assert_eq!(ss.state(), SoftStartState::Done);
    }

    #[test]
    fn reset_goes_to_idle() {
        let mut ss = SoftStart::<f32>::new(1.0, 0.1, 0.0);

        // Run to completion
        while ss.state() != SoftStartState::Done {
            ss.tick();
        }

        // Reset with new pre-bias
        ss.reset(0.3);
        assert_eq!(ss.state(), SoftStartState::Idle);
        assert!((ss.output() - 0.3).abs() < 1e-6);

        // First tick resumes ramping from 0.3
        let v = ss.tick();
        assert!((v - 0.4).abs() < 1e-6);
        assert_eq!(ss.state(), SoftStartState::Ramping);
    }

    #[test]
    fn reset_with_pre_bias_above_target() {
        let mut ss = SoftStart::<f32>::new(1.0, 0.1, 0.0);
        ss.tick(); // move to Ramping

        ss.reset(5.0);
        assert_eq!(ss.state(), SoftStartState::Done);
        assert!((ss.output() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn set_target_lower_while_ramping() {
        let mut ss = SoftStart::<f32>::new(2.0, 0.1, 0.0);

        // A few ticks into ramp
        ss.tick(); // 0.1
        ss.tick(); // 0.2
        ss.tick(); // 0.3
        assert_eq!(ss.state(), SoftStartState::Ramping);

        // Set target below current output: snap
        ss.set_target(0.1);
        assert_eq!(ss.state(), SoftStartState::Done);
        assert!((ss.output() - 0.1).abs() < 1e-6);
    }

    #[test]
    fn set_target_equal_to_output() {
        let mut ss = SoftStart::<f32>::new(1.0, 0.25, 0.0);

        // Tick to 0.5
        ss.tick(); // 0.25
        ss.tick(); // 0.5

        ss.set_target(0.5);
        assert_eq!(ss.state(), SoftStartState::Done);
        assert!((ss.output() - 0.5).abs() < 1e-6);
    }
}
