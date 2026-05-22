//! Hardware-specific configuration. Loaded from `configs/hardware.toml`.

use robotics_gpio::{PwmChannel, ServoCalibration};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServoConfig {
    pub channel: PwmChannel,
    pub calibration: ServoCalibration,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HardwareConfig {
    pub base: ServoConfig,
    pub shoulder: ServoConfig,
    pub elbow: ServoConfig,
    pub wrist: ServoConfig,
    pub gripper: ServoConfig,
}

impl HardwareConfig {
    /// A reasonable starting wiring: one servo per Pi PWM channel
    /// (base, shoulder on hardware PWM) plus three on software PWM
    /// pins (elbow, wrist, gripper). The Pi only has 2 hardware PWM
    /// channels, so a 5-DOF arm needs software PWM unless you add a
    /// PCA9685 servo HAT (which is the production path — wire it via
    /// I²C and swap the gpio backend).
    pub fn pi4_default() -> Self {
        Self {
            base: ServoConfig {
                channel: PwmChannel::Hardware0,
                calibration: ServoCalibration::sg90_nominal(),
            },
            shoulder: ServoConfig {
                channel: PwmChannel::Hardware1,
                calibration: ServoCalibration::sg90_nominal(),
            },
            elbow: ServoConfig {
                channel: PwmChannel::Software(17),
                calibration: ServoCalibration::sg90_nominal(),
            },
            wrist: ServoConfig {
                channel: PwmChannel::Software(27),
                calibration: ServoCalibration::sg90_nominal(),
            },
            gripper: ServoConfig {
                channel: PwmChannel::Software(22),
                calibration: ServoCalibration::sg90_nominal(),
            },
        }
    }
}
