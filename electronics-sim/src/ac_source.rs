use std::f64::consts::PI;

/// N-phase AC voltage source.
///
/// Models 1-phase or 3-phase balanced AC with configurable RMS voltage and
/// line frequency. Phase voltages are spaced evenly at `2*PI / ac_phases`.
#[derive(Debug, Clone, Copy)]
pub struct AcSource {
    /// Line-to-neutral RMS voltage [V].
    pub v_rms_ln: f64,
    /// Line frequency [Hz].
    pub f_line_hz: f64,
    /// Number of AC input phases (1 or 3). Not to be confused with
    /// interleaved switch stages per AC phase (`num_phases` elsewhere).
    pub ac_phases: usize,
}

impl AcSource {
    pub fn new(v_rms_ln: f64, f_line_hz: f64, ac_phases: usize) -> Self {
        Self { v_rms_ln, f_line_hz, ac_phases }
    }

    /// Single-phase source.
    pub fn single(v_rms: f64, f_line_hz: f64) -> Self {
        Self::new(v_rms, f_line_hz, 1)
    }

    /// Three-phase balanced source.
    pub fn three_phase(v_rms_ln: f64, f_line_hz: f64) -> Self {
        Self::new(v_rms_ln, f_line_hz, 3)
    }

    /// Peak line-to-neutral voltage.
    pub fn v_peak(&self) -> f64 {
        self.v_rms_ln * 2.0_f64.sqrt()
    }

    /// Angular frequency [rad/s].
    pub fn omega(&self) -> f64 {
        2.0 * PI * self.f_line_hz
    }

    /// Instantaneous phase voltage at time `t` (signed).
    /// `phase_idx`: 0=A, 1=B, 2=C.
    pub fn v_phase(&self, phase_idx: usize, t: f64) -> f64 {
        let angle = self.omega() * t - phase_idx as f64 * 2.0 * PI / self.ac_phases as f64;
        self.v_peak() * angle.sin()
    }

    /// Rectified (absolute) phase voltage at time `t`.
    pub fn v_phase_rect(&self, phase_idx: usize, t: f64) -> f64 {
        self.v_phase(phase_idx, t).abs()
    }

    /// All phase voltages at time `t` (signed). Always returns 3 elements;
    /// for single-phase, indices 1 and 2 are zero.
    pub fn v_abc(&self, t: f64) -> [f64; 3] {
        match self.ac_phases {
            1 => [self.v_phase(0, t), 0.0, 0.0],
            3 => [self.v_phase(0, t), self.v_phase(1, t), self.v_phase(2, t)],
            n => {
                let mut v = [0.0; 3];
                for i in 0..n.min(3) {
                    v[i] = self.v_phase(i, t);
                }
                v
            }
        }
    }

    /// Line period [s].
    pub fn t_line(&self) -> f64 {
        1.0 / self.f_line_hz
    }

    /// Half-cycle period [s].
    pub fn t_half(&self) -> f64 {
        0.5 / self.f_line_hz
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_phase_peak() {
        let ac = AcSource::single(230.0, 50.0);
        assert!((ac.v_peak() - 325.27).abs() < 0.1);
    }

    #[test]
    fn three_phase_120_deg_offset() {
        let ac = AcSource::three_phase(230.0, 50.0);
        let t = 0.0;
        let v = ac.v_abc(t);
        // At t=0: v_a = 0, v_b = sin(-120°) * Vpk, v_c = sin(-240°) * Vpk
        assert!(v[0].abs() < 0.01);
        assert!((v[1] - ac.v_peak() * (-2.0 * PI / 3.0).sin()).abs() < 0.01);
        assert!((v[2] - ac.v_peak() * (-4.0 * PI / 3.0).sin()).abs() < 0.01);
    }

    #[test]
    fn three_phase_sum_is_zero() {
        let ac = AcSource::three_phase(230.0, 50.0);
        // Balanced 3-phase: v_a + v_b + v_c = 0 at all times
        for i in 0..100 {
            let t = i as f64 * 0.001;
            let v = ac.v_abc(t);
            assert!((v[0] + v[1] + v[2]).abs() < 1e-10, "Sum not zero at t={t}");
        }
    }

    #[test]
    fn rectified_is_positive() {
        let ac = AcSource::single(230.0, 50.0);
        for i in 0..1000 {
            let t = i as f64 * 0.00001;
            assert!(ac.v_phase_rect(0, t) >= 0.0);
        }
    }
}
