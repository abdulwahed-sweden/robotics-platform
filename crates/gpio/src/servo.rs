//! Servo abstraction.
//!
//! A servo is a PWM channel + a calibration mapping angles to pulse
//! widths. Hobbyist hardware (SG90) has documented nominal values
//! (1ms = 0°, 2ms = 180°) but every servo I've measured is 5–10% off
//! the nominal. We bake the per-servo calibration into the config so
//! the kinematics layer can keep using clean radians.
//!
//! ## Math
//!
//! At 50 Hz the period is 20 ms. To produce a pulse of `pulse_ms`
//! milliseconds, the duty cycle is:
//!
//! ```text
//! duty = pulse_ms / 20.0
//! ```
//!
//! The calibration is `pulse_ms = pulse_at_min + (pulse_at_max -
//! pulse_at_min) * (angle - angle_min) / (angle_max - angle_min)`.

use robotics_core::{RoboticsError, Result};
use serde::{Deserialize, Serialize};

use crate::pwm::{Pwm, PwmChannel};

/// Per-servo calibration. Loaded from `configs/hardware.toml`. Keep
/// the angle range in radians to match the rest of the platform.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ServoCalibration {
    pub frequency_hz: f64,
    pub angle_min_rad: f64,
    pub angle_max_rad: f64,
    pub pulse_min_ms: f64,
    pub pulse_max_ms: f64,
}

impl ServoCalibration {
    /// Nominal SG90 values — useful starting point before you measure
    /// your specific servo with a protractor.
    pub fn sg90_nominal() -> Self {
        Self {
            frequency_hz: 50.0,
            angle_min_rad: 0.0,
            angle_max_rad: std::f64::consts::PI,
            pulse_min_ms: 1.0,
            pulse_max_ms: 2.0,
        }
    }

    /// Convert an angle (rad) to a duty fraction in [0, 1].
    pub fn angle_to_duty(&self, angle_rad: f64) -> Result<f64> {
        if angle_rad < self.angle_min_rad || angle_rad > self.angle_max_rad {
            return Err(RoboticsError::Hardware(format!(
                "servo angle {} rad out of calibrated range [{}, {}]",
                angle_rad, self.angle_min_rad, self.angle_max_rad
            )));
        }
        let span = self.angle_max_rad - self.angle_min_rad;
        let frac = (angle_rad - self.angle_min_rad) / span;
        let pulse_ms = self.pulse_min_ms + frac * (self.pulse_max_ms - self.pulse_min_ms);
        let period_ms = 1000.0 / self.frequency_hz;
        Ok(pulse_ms / period_ms)
    }
}

pub struct ServoChannel<P: Pwm> {
    pwm: P,
    channel: PwmChannel,
    calibration: ServoCalibration,
    initialized: bool,
}

impl<P: Pwm> ServoChannel<P> {
    pub fn new(pwm: P, channel: PwmChannel, calibration: ServoCalibration) -> Self {
        Self { pwm, channel, calibration, initialized: false }
    }

    pub fn set_angle(&mut self, angle_rad: f64) -> Result<()> {
        if !self.initialized {
            self.pwm.set_frequency(self.channel, self.calibration.frequency_hz)?;
            self.initialized = true;
        }
        let duty = self.calibration.angle_to_duty(angle_rad)?;
        self.pwm.set_duty(self.channel, duty)
    }

    /// Cut the pulse train. The servo goes limp — useful for e-stop
    /// and for power saving when the arm is idle.
    pub fn disable(&mut self) -> Result<()> {
        self.pwm.disable(self.channel)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sg90_midpoint_maps_to_7p5_percent_duty() {
        let cal = ServoCalibration::sg90_nominal();
        let duty = cal.angle_to_duty(std::f64::consts::FRAC_PI_2).unwrap();
        // Midpoint pulse = 1.5 ms, period = 20 ms → 7.5% duty.
        assert!((duty - 0.075).abs() < 1e-9);
    }

    #[test]
    fn out_of_range_is_rejected() {
        let cal = ServoCalibration::sg90_nominal();
        assert!(cal.angle_to_duty(-0.1).is_err());
        assert!(cal.angle_to_duty(std::f64::consts::PI + 0.1).is_err());
    }
}
