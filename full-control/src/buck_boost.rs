use crate::control_2p2z::{
    DacSettings, Parameters, Topology, TransferFunction, TwoPoleTwoZero, TwoPoleTwoZeroParams,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mode {
    /// V_in significantly above V_out — gain-scheduled for high step-down.
    Buck,
    /// V_in near V_out — gains designed for unity-gain operating point.
    BuckBoost,
    /// V_in significantly below V_out — gain-scheduled for step-up.
    Boost,
}

impl Mode {
    pub fn name(self) -> &'static str {
        match self {
            Mode::Buck => "Buck     ",
            Mode::BuckBoost => "BuckBoost",
            Mode::Boost => "Boost    ",
        }
    }
}

pub struct BuckBoostTransferFunction {
    pub buck: TransferFunction,
    pub boost: TransferFunction,
    pub buck_boost: TransferFunction,

    pub dac_buck: DacSettings,
    pub dac_boost: DacSettings,
    pub dac_buck_boost: DacSettings,
}

impl BuckBoostTransferFunction {
    pub const fn new(
        parameters: Parameters,
        vin_nom_buck: f64,
        vin_nom_boost: f64,
        vout_nom: f64,
    ) -> Self {
        let (buck, dac_buck) = parameters.to_transfer_function(vin_nom_buck, Topology::Buck);
        let (boost, dac_boost) = parameters.to_transfer_function(vin_nom_boost, Topology::Boost);
        let (buck_boost, dac_buck_boost) =
            parameters.to_transfer_function(vout_nom, Topology::BuckBoost);

        Self {
            buck,
            boost,
            buck_boost,
            dac_buck,
            dac_boost,
            dac_buck_boost,
        }
    }

    pub const fn to_weights(self) -> BuckBoostWeights<f32> {
        let BuckBoostTransferFunction {
            buck,
            boost,
            buck_boost,
            dac_buck: _,
            dac_boost: _,
            dac_buck_boost: _,
        } = self;

        BuckBoostWeights {
            buck: buck.to_2p2z(),
            boost: boost.to_2p2z(),
            buck_boost: buck_boost.to_2p2z(),
        }
    }
}

pub struct BuckBoostWeights<T> {
    pub buck: TwoPoleTwoZeroParams<T>,
    pub boost: TwoPoleTwoZeroParams<T>,
    pub buck_boost: TwoPoleTwoZeroParams<T>,
}

impl BuckBoostWeights<f32> {
    pub const fn to_controller(
        self,
        limit_min: f32,
        limit_max: f32,
    ) -> BuckBoostController {
        let BuckBoostWeights {
            buck,
            boost,
            buck_boost,
        } = self;

        BuckBoostController {
            buck: buck.to_controller(limit_min, limit_max),
            boost: boost.to_controller(limit_min, limit_max),
            buck_boost: buck_boost.to_controller(limit_min, limit_max),
            mode: Mode::BuckBoost,
        }
    }
}

pub struct BuckBoostController {
    buck: TwoPoleTwoZero<f32>,
    boost: TwoPoleTwoZero<f32>,
    buck_boost: TwoPoleTwoZero<f32>,
    mode: Mode,
}

impl BuckBoostController {
    /// Returns `(output_volts, mode, slope_a_per_s, clamped)`.
    pub fn update(&mut self, v_in: f64, v_out: f64, target: f64) -> (f32, Mode) {
        let error = (target - v_out) as f32;

        let desired = select_mode(v_in, v_out, self.mode);
        if desired != self.mode {
            self.switch_to(desired, error);
        }

        let compensator = match self.mode {
            Mode::Buck => &mut self.buck,
            Mode::Boost => &mut self.boost,
            Mode::BuckBoost => &mut self.buck_boost,
        };

        (compensator.update(error), self.mode)
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
