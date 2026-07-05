use crate::control_2p2z::Scalar;

/// Per-phase current balancing for multi-phase converters.
///
/// Computes duty cycle trim values that equalize phase currents.
/// The trim is added to each phase's base duty cycle.
///
/// `N`: maximum number of phases (const generic).
pub struct CurrentSharing<S: Scalar, const N: usize> {
    /// PI gain (proportional).
    kp: S,
    /// PI gain (integral).
    ki: S,
    /// Integrator state per phase.
    integrators: [S; N],
    /// Maximum absolute trim (prevents runaway).
    max_trim: S,
    /// Number of active phases (<= N).
    active: usize,
}

impl<S: Scalar, const N: usize> CurrentSharing<S, N> {
    /// Create a new current sharing controller.
    ///
    /// - `kp`: proportional gain
    /// - `ki`: integral gain
    /// - `max_trim`: maximum absolute duty cycle trim per phase
    /// - `active`: number of active phases (must be <= N)
    pub fn new(kp: S, ki: S, max_trim: S, active: usize) -> Self {
        assert!(active <= N, "active phases exceeds const generic N");
        Self {
            kp,
            ki,
            integrators: [S::ZERO; N],
            max_trim,
            active,
        }
    }

    /// Compute per-phase duty cycle trims to balance currents.
    ///
    /// `currents`: measured average current for each phase.
    ///
    /// Returns an array of duty cycle trims. Positive trim means "increase
    /// this phase's duty" (phase is carrying less than average).
    /// Inactive phases (index >= active) receive zero trim.
    pub fn update(&mut self, currents: &[S; N]) -> [S; N] {
        let mut trims = [S::ZERO; N];

        if self.active < 2 {
            // Nothing to balance with 0 or 1 phase.
            return trims;
        }

        // Compute mean current across active phases.
        let mut sum = S::ZERO;
        for i in 0..self.active {
            sum = sum + currents[i];
        }
        let n = S::from_f32(self.active as f32);
        let mean = sum / n;

        // The democratic mean-based error has an inherent (active-1)/active
        // sensitivity to each phase's own current, so the loop gain — and thus
        // bandwidth/damping — would otherwise drift with the active phase
        // count. Normalize by active/(active-1) so `kp`/`ki` map to a
        // phase-count-independent loop gain (active >= 2 here, so no /0).
        let gain_norm = n / (n - S::from_f32(1.0));

        let neg_max = -self.max_trim;

        for i in 0..self.active {
            let error = (mean - currents[i]) * gain_norm;

            // Update integrator.
            self.integrators[i] = self.integrators[i] + self.ki * error;
            self.integrators[i] = self.integrators[i].clamp(neg_max, self.max_trim);

            // PI output.
            let trim = self.kp * error + self.integrators[i];
            trims[i] = trim.clamp(neg_max, self.max_trim);
        }

        trims
    }

    /// Reset all integrator states to zero.
    pub fn reset(&mut self) {
        self.integrators = [S::ZERO; N];
    }

    /// Change the number of active phases.
    ///
    /// Only the integrators of phases whose activation status changes are
    /// zeroed (shed phases, and newly-added phases start fresh); phases that
    /// remain active KEEP their learned mismatch correction. Zeroing every
    /// integrator (the old behaviour) would bump the surviving phases at each
    /// shed/add event.
    pub fn set_active(&mut self, n: usize) {
        assert!(n <= N, "active phases exceeds const generic N");
        let (lo, hi) = (n.min(self.active), n.max(self.active));
        for i in lo..hi {
            self.integrators[i] = S::ZERO;
        }
        self.active = n;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-6;

    #[test]
    fn equal_currents_give_zero_trim() {
        let mut cs = CurrentSharing::<f32, 2>::new(0.1, 0.01, 0.5, 2);
        let trims = cs.update(&[5.0, 5.0]);
        assert!(trims[0].abs() < EPS, "trim[0] = {}", trims[0]);
        assert!(trims[1].abs() < EPS, "trim[1] = {}", trims[1]);
    }

    #[test]
    fn imbalanced_two_phases_converge() {
        let mut cs = CurrentSharing::<f32, 2>::new(0.01, 0.005, 0.5, 2);

        // Simulate a plant where each phase has a natural (mismatch) offset
        // plus a response to its accumulated duty trim.
        // i_phase[k] = baseline[k] + duty_trim[k] * plant_gain
        // The duty applied to the plant accumulates each cycle.
        let baseline = [6.0_f32, 4.0];
        let plant_gain = 2.0_f32;
        let mut duty = [0.0_f32; 2];

        for _ in 0..200 {
            let currents = [
                baseline[0] + duty[0] * plant_gain,
                baseline[1] + duty[1] * plant_gain,
            ];
            let trims = cs.update(&currents);
            // Accumulate trim into duty (integrating plant).
            duty[0] += trims[0];
            duty[1] += trims[1];
        }

        let currents = [
            baseline[0] + duty[0] * plant_gain,
            baseline[1] + duty[1] * plant_gain,
        ];
        let diff = (currents[0] - currents[1]).abs();
        assert!(
            diff < 0.1,
            "currents did not converge: [{}, {}], diff={}",
            currents[0],
            currents[1],
            diff
        );
    }

    #[test]
    fn three_phases_one_lagging_converges() {
        let mut cs = CurrentSharing::<f32, 3>::new(0.01, 0.005, 0.5, 3);

        let baseline = [5.0_f32, 5.0, 3.0];
        let plant_gain = 2.0_f32;
        let mut duty = [0.0_f32; 3];

        for _ in 0..200 {
            let currents = [
                baseline[0] + duty[0] * plant_gain,
                baseline[1] + duty[1] * plant_gain,
                baseline[2] + duty[2] * plant_gain,
            ];
            let trims = cs.update(&currents);
            for i in 0..3 {
                duty[i] += trims[i];
            }
        }

        let currents = [
            baseline[0] + duty[0] * plant_gain,
            baseline[1] + duty[1] * plant_gain,
            baseline[2] + duty[2] * plant_gain,
        ];
        let mean = (currents[0] + currents[1] + currents[2]) / 3.0;
        for i in 0..3 {
            let err = (currents[i] - mean).abs();
            assert!(
                err < 0.1,
                "phase {} did not converge: current={}, mean={}, err={}",
                i,
                currents[i],
                mean,
                err
            );
        }
    }

    #[test]
    fn max_trim_clamping() {
        let max_trim = 0.1_f32;
        let mut cs = CurrentSharing::<f32, 2>::new(10.0, 1.0, max_trim, 2);

        // Large imbalance with aggressive gains.
        let trims = cs.update(&[10.0, 0.0]);

        for i in 0..2 {
            assert!(
                trims[i].abs() <= max_trim + EPS,
                "trim[{}] = {} exceeds max_trim = {}",
                i,
                trims[i],
                max_trim
            );
        }

        // Run several more iterations to saturate integrators.
        for _ in 0..50 {
            let trims = cs.update(&[10.0, 0.0]);
            for i in 0..2 {
                assert!(
                    trims[i].abs() <= max_trim + EPS,
                    "trim[{}] = {} exceeds max_trim = {}",
                    i,
                    trims[i],
                    max_trim
                );
            }
        }
    }

    #[test]
    fn reset_zeroes_integrators() {
        let mut cs = CurrentSharing::<f32, 2>::new(0.1, 0.5, 1.0, 2);

        // Accumulate integrator state.
        for _ in 0..20 {
            cs.update(&[6.0, 4.0]);
        }

        // Verify integrators are non-zero.
        let has_state = cs.integrators.iter().any(|&v| v.abs() > EPS);
        assert!(has_state, "integrators should have accumulated state");

        cs.reset();

        for i in 0..2 {
            assert!(
                cs.integrators[i].abs() < EPS,
                "integrator[{}] not zeroed after reset: {}",
                i,
                cs.integrators[i]
            );
        }
    }

    #[test]
    fn set_active_preserves_surviving_phase_integrators() {
        let mut cs = CurrentSharing::<f32, 4>::new(0.1, 0.5, 1.0, 3);
        for _ in 0..20 {
            cs.update(&[6.0, 4.0, 5.0, 0.0]);
        }
        let saved = cs.integrators;
        assert!(saved[0].abs() > EPS && saved[1].abs() > EPS, "should have state");
        // Shed phase 2 (3 → 2): surviving phases keep state, shed phase zeroed.
        cs.set_active(2);
        assert_eq!(cs.integrators[0], saved[0], "phase 0 integrator must survive");
        assert_eq!(cs.integrators[1], saved[1], "phase 1 integrator must survive");
        assert_eq!(cs.integrators[2], 0.0, "shed phase 2 integrator must be zeroed");
    }

    #[test]
    fn inactive_phases_get_zero_trim() {
        let mut cs = CurrentSharing::<f32, 4>::new(0.1, 0.01, 0.5, 2);

        let trims = cs.update(&[6.0, 4.0, 9.0, 1.0]);

        // Only phases 0 and 1 are active.
        assert!(trims[2].abs() < EPS, "inactive trim[2] = {}", trims[2]);
        assert!(trims[3].abs() < EPS, "inactive trim[3] = {}", trims[3]);

        // Active phases should have non-zero trims (they are imbalanced).
        assert!(trims[0].abs() > EPS, "active trim[0] should be non-zero");
        assert!(trims[1].abs() > EPS, "active trim[1] should be non-zero");
    }

    #[test]
    fn works_with_f32_scalar() {
        // Explicit test that the module works with f32 Scalar impl.
        let mut cs = CurrentSharing::<f32, 2>::new(
            Scalar::from_f32(0.1),
            Scalar::from_f32(0.01),
            Scalar::from_f32(0.5),
            2,
        );

        let currents: [f32; 2] = [Scalar::from_f32(5.0), Scalar::from_f32(3.0)];
        let trims = cs.update(&currents);

        // Phase 1 is carrying less -> positive trim.
        assert!(trims[1] > f32::ZERO, "lagging phase should get positive trim");
        // Phase 0 is carrying more -> negative trim.
        assert!(trims[0] < f32::ZERO, "leading phase should get negative trim");
        // Trims should be roughly opposite (sum near zero).
        let trim_sum = trims[0] + trims[1];
        assert!(
            trim_sum.abs() < 0.01,
            "trims should approximately sum to zero, got {}",
            trim_sum
        );
    }
}
