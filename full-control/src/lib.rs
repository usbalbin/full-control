#![cfg_attr(not(feature = "std"), no_std)]

pub mod control_2p2z;
pub mod fmac;
pub(crate) mod math;
pub mod buck_boost;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scalar_impl_f32() {
        let x: f32 = 5.0;
        let y: f32 = Scalar::from_f32(3.0);
        assert_eq!(y, 3.0);
        let sum = x + y;
        assert_eq!(sum, 8.0);
        let clamped = y.clamp(0.0, 4.0);
        assert_eq!(clamped, 3.0);
    }

    #[test]
    fn test_two_pole_two_zero_params_f32() {
        let params = control_2p2z::TwoPoleTwoZeroParams::<f32> {
            a1: -1.5,
            a2: 0.5625,
            b0: 0.25,
            b1: 0.5,
            b2: 0.25,
        };

        let controller = params.to_controller(0.0, 4095.0);
        assert_eq!(controller.last_output(), 0.0);

        let output = controller.update(100.0);
        assert!(output >= 0.0);
        assert!(output <= 4095.0);
    }

    #[test]
    fn test_two_pole_two_zero_bumpless_transfer() {
        let params = control_2p2z::TwoPoleTwoZeroParams::<f32> {
            a1: -1.5,
            a2: 0.5625,
            b0: 0.25,
            b1: 0.5,
            b2: 0.25,
        };

        let mut controller = params.to_controller(0.0, 4095.0);
        controller.update(100.0);
        controller.update(50.0);

        let last_output = controller.last_output();

        let mut controller2 = params.to_controller(0.0, 4095.0);
        controller2.prime(last_output, 50.0);

        assert_eq!(controller2.last_output(), last_output);
    }

    #[test]
    fn test_topologies() {
        assert_eq!(control_2p2z::Topology::Buck.name(), "Buck");
        assert_eq!(control_2p2z::Topology::Boost.name(), "Boost");
        assert_eq!(control_2p2z::Topology::BuckBoost.name(), "BuckBoost");
    }

    #[test]
    fn test_mode_enum() {
        assert_eq!(buck_boost::Mode::Buck.name(), "Buck     ");
        assert_eq!(buck_boost::Mode::Boost.name(), "Boost    ");
        assert_eq!(buck_boost::Mode::BuckBoost.name(), "BuckBoost");
    }

    #[test]
    fn test_select_mode_buck() {
        let mode = buck_boost::select_mode(24.0, 13.5, buck_boost::Mode::Buck);
        assert_eq!(mode, buck_boost::Mode::Buck);
    }

    #[test]
    fn test_select_mode_boost() {
        let mode = buck_boost::select_mode(12.0, 13.5, buck_boost::Mode::BuckBoost);
        assert_eq!(mode, buck_boost::Mode::Boost);
    }

    #[test]
    fn test_select_mode_buck_boost() {
        let mode = buck_boost::select_mode(13.5, 13.5, buck_boost::Mode::BuckBoost);
        assert_eq!(mode, buck_boost::Mode::BuckBoost);
    }

    #[test]
    fn test_fmac_iir_basic() {
        // Simple low-pass filter coefficients
        let b = [16384, 16384, 0]; // b0=0.5, b1=0.5
        let a = [0, 0];
        let mut fmac = fmac::FmacIir::new(b, a, 1, -32768, 32767);

        // Input at full scale
        let output = fmac.update(32767);
        // y[0] = 0.5 * 32767 = 16383.5 -> 16384 (rounded)
        assert!(output >= 16383 && output <= 16385);
    }

    #[test]
    fn test_fmac_iir_clamping() {
        let b = [32767, 0, 0]; // b0=1.0
        let a = [0, 0];
        let mut fmac = fmac::FmacIir::new(b, a, 1, -10000, 10000);

        // Input should produce output beyond limits
        let output = fmac.update(32767);
        assert_eq!(output, 10000); // Clamped to max
    }

    #[test]
    fn test_parameters_crossover_divisor() {
        let params = control_2p2z::Parameters {
            v_out: 13.5,
            v_diode: 0.0,
            c_out: 47e-6,
            f_sw: 500e3,
            l_inductor: 4e-6,
            r_esr_out_cap: 10e-3,
            current_sense_gain: 0.066,
            i_load: 13.5 / 6.0,
            phase_margin: control_2p2z::PhaseMargin::Manual {
                phase_margin: 75.0_f64.to_radians(),
            },
            safety_factor: 2.0,
            crossover_hz: 50_000.0,
            cycles_per_tick: 1,
        };

        let divisor = params.crossover_divisor(control_2p2z::Topology::Buck, 24.0);
        assert!(divisor >= 8.0);
        assert!(divisor <= 1000.0);
    }

    #[test]
    fn test_transfer_function_to_2p2z() {
        let params = control_2p2z::Parameters {
            v_out: 13.5,
            v_diode: 0.0,
            c_out: 47e-6,
            f_sw: 500e3,
            l_inductor: 4e-6,
            r_esr_out_cap: 10e-3,
            current_sense_gain: 0.066,
            i_load: 13.5 / 6.0,
            phase_margin: control_2p2z::PhaseMargin::Manual {
                phase_margin: 75.0_f64.to_radians(),
            },
            safety_factor: 2.0,
            crossover_hz: 50_000.0,
            cycles_per_tick: 1,
        };

        let (tf, _) = params.to_transfer_function(24.0, control_2p2z::Topology::Buck);
        let coeffs = tf.to_2p2z();

        // b0 should be positive and reasonable
        assert!(coeffs.b0 > 0.0);
        assert!(coeffs.b0 < 10.0);

        // a1 + a2 should be close to 1 for a stable integrator
        assert!(coeffs.a1 + coeffs.a2 > 0.5);
        assert!(coeffs.a1 + coeffs.a2 < 1.5);
    }

    #[test]
    fn test_buck_boost_controller_mode_switching() {
        let params = control_2p2z::Parameters {
            v_out: 13.5,
            v_diode: 0.0,
            c_out: 47e-6,
            f_sw: 500e3,
            l_inductor: 4e-6,
            r_esr_out_cap: 10e-3,
            current_sense_gain: 0.066,
            i_load: 13.5 / 6.0,
            phase_margin: control_2p2z::PhaseMargin::Manual {
                phase_margin: 75.0_f64.to_radians(),
            },
            safety_factor: 2.0,
            crossover_hz: 50_000.0,
            cycles_per_tick: 1,
        };

        let tf = buck_boost::BuckBoostTransferFunction::new(
            params,
            24.0,
            12.0,
            13.5,
        );

        let controller = tf.to_weights().to_controller(0.0, 4095.0);

        // Test Buck mode
        let (output, mode) = controller.update(24.0, 13.5, 13.5);
        assert_eq!(mode, buck_boost::Mode::Buck);

        // Test Boost mode
        let (output, mode) = controller.update(12.0, 13.5, 13.5);
        assert_eq!(mode, buck_boost::Mode::Boost);
    }

    #[test]
    fn test_buck_boost_hysteresis() {
        let params = control_2p2z::Parameters {
            v_out: 13.5,
            v_diode: 0.0,
            c_out: 47e-6,
            f_sw: 500e3,
            l_inductor: 4e-6,
            r_esr_out_cap: 10e-3,
            current_sense_gain: 0.066,
            i_load: 13.5 / 6.0,
            phase_margin: control_2p2z::PhaseMargin::Manual {
                phase_margin: 75.0_f64.to_radians(),
            },
            safety_factor: 2.0,
            crossover_hz: 50_000.0,
            cycles_per_tick: 1,
        };

        let tf = buck_boost::BuckBoostTransferFunction::new(
            params,
            24.0,
            12.0,
            13.5,
        );

        let mut controller = tf.to_weights().to_controller(0.0, 4095.0);

        // Start in BuckBoost mode (unity ratio)
        let (_, mode) = controller.update(13.5, 13.5, 13.5);
        assert_eq!(mode, buck_boost::Mode::BuckBoost);

        // Switch to Boost (v_in < v_out)
        let (_, mode) = controller.update(12.0, 13.5, 13.5);
        assert_eq!(mode, buck_boost::Mode::Boost);

        // Switch back to BuckBoost (should stay in Boost until ratio is closer)
        // With hysteresis, we need to cross the inner threshold
        let (_, mode) = controller.update(13.0, 13.5, 13.5);
        assert_eq!(mode, buck_boost::Mode::BuckBoost);
    }

    #[test]
    fn test_fixed_point_scalar() {
        // Test I16F16 fixed-point type
        use fixed::types::I16F16;
        let x: I16F16 = fixed::types::I16F16::from_num(1.5_f32);
        let y: I16F16 = Scalar::from_f32(2.0_f32);
        assert_eq!(y.to_num::<f32>(), 2.0);
        let sum = x + y;
        assert!((sum.to_num::<f32>() - 3.5).abs() < 0.01);
    }

    #[test]
    fn test_dac_settings_vpp() {
        let dac = control_2p2z::DacSettings {
            dac_slope: -100000.0,
            vpp: 10.0,
        };
        assert_eq!(dac.vpp(), 10.0);
    }
}