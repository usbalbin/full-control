#![cfg_attr(not(feature = "std"), no_std)]

pub mod control_2p2z;
pub mod control_pfc;
pub mod fmac;
pub(crate) mod math;
pub mod buck_boost;
pub mod pfc_runtime;

#[cfg(test)]
mod tests {
    use super::*;
    use control_2p2z::Scalar;

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

        let mut controller = params.to_controller(0.0, 4095.0);
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
    fn test_mode_enum() {
        assert_eq!(buck_boost::Mode::Buck.name(), "Buck     ");
        assert_eq!(buck_boost::Mode::Boost.name(), "Boost    ");
        assert_eq!(buck_boost::Mode::BuckBoost.name(), "BuckBoost");
    }

    #[test]
    fn test_fmac_iir_basic() {
        // Simple low-pass filter: real b0=0.5, b1=0.5.
        // With R=1 the q1.15 bits are real_coeff / 2^R × 32768 = 0.5/2 × 32768 = 8192.
        let b = [8192, 8192, 0];
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
    fn test_max_feasible_crossover_hz() {
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

        let max_fx = params.max_feasible_crossover_hz(24.0, control_2p2z::Topology::Buck);
        // Max feasible should be between 1 Hz and f_sw/8
        assert!(max_fx >= 1.0);
        assert!(max_fx <= 500e3 / 8.0);
        // A design at the max feasible frequency should succeed
        let params_at_max = control_2p2z::Parameters { crossover_hz: max_fx, ..params };
        let (tf, _) = params_at_max.to_transfer_function(24.0, control_2p2z::Topology::Buck);
        assert!(tf.to_2p2z().is_some());
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
        let coeffs = tf.to_2p2z().unwrap();

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
            7.0,
            13.5,
        );

        let mut controller = tf.to_weights().unwrap().to_controller(0.0, 4095.0);

        // Test Buck mode (ratio 24/13.5 = 1.78 > 1.40 outer boundary)
        let (_output, mode) = controller.update(24.0, 13.5, 13.5);
        assert_eq!(mode, buck_boost::Mode::Buck);

        // Test Boost mode (ratio 7/13.5 = 0.52 < 0.60 outer boundary)
        let (_output, mode) = controller.update(7.0, 13.5, 13.5);
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
            7.0,
            13.5,
        );

        let mut controller = tf.to_weights().unwrap().to_controller(0.0, 4095.0);

        // Start in BuckBoost mode (ratio 1.0, inner band [0.80, 1.20])
        let (_, mode) = controller.update(13.5, 13.5, 13.5);
        assert_eq!(mode, buck_boost::Mode::BuckBoost);

        // Move well below unity to enter Boost (ratio 7/13.5 = 0.52 < 0.60 outer boundary)
        let (_, mode) = controller.update(7.0, 13.5, 13.5);
        assert_eq!(mode, buck_boost::Mode::Boost);

        // Move into hysteresis zone (ratio 10/13.5 = 0.74, between 0.60 and 0.80) — stays Boost
        let (_, mode) = controller.update(10.0, 13.5, 13.5);
        assert_eq!(mode, buck_boost::Mode::Boost);

        // Cross inner boundary into BuckBoost band (ratio 11.5/13.5 = 0.85 > 0.80) — switches
        let (_, mode) = controller.update(11.5, 13.5, 13.5);
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
        // Construct DacSettings via the Parameters API and verify vpp is positive.
        let params = control_2p2z::Parameters {
            v_out: 8.0,
            v_diode: 0.6,
            c_out: 440e-6,
            f_sw: 200e3,
            l_inductor: 22e-6,
            r_esr_out_cap: 31e-3,
            current_sense_gain: 0.48,
            i_load: 2.0,
            phase_margin: control_2p2z::PhaseMargin::Manual {
                phase_margin: 75.0_f64.to_radians(),
            },
            safety_factor: 2.0,
            crossover_hz: 15_000.0,
            cycles_per_tick: 1,
        };
        let (_, dac) = params.to_transfer_function(16.0, control_2p2z::Topology::Buck);
        assert!(dac.vpp() > 0.0);
    }
}