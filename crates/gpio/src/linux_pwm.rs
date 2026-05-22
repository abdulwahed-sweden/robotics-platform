//! Linux / Raspberry Pi PWM via `rppal`.
//!
//! Only compiled on `cfg(target_os = "linux")`. On the Pi this maps
//! `Hardware0` / `Hardware1` to `rppal::pwm::Channel::Pwm0` / `Pwm1`.
//! Software PWM is handled via `rppal::gpio::OutputPin::set_pwm`.
//!
//! ## Permissions
//!
//! `/sys/class/pwm` requires either root or membership in `gpio` /
//! `pwm` groups. See `docs/gpio.md` for the udev rules and `dtoverlay`
//! entries you need.

use std::collections::HashMap;
use std::time::Duration;

use robotics_core::{RoboticsError, Result};
use rppal::gpio::{Gpio, OutputPin};
use rppal::pwm::{Channel, Polarity, Pwm as RppalPwm};

use crate::pwm::{Pwm, PwmChannel};

pub struct LinuxPwm {
    hw0: Option<RppalPwm>,
    hw1: Option<RppalPwm>,
    sw: HashMap<u8, SoftwarePwm>,
    gpio: Gpio,
}

struct SoftwarePwm {
    pin: OutputPin,
    period: Duration,
}

impl LinuxPwm {
    pub fn new() -> Result<Self> {
        let gpio = Gpio::new().map_err(|e| RoboticsError::Hardware(e.to_string()))?;
        Ok(Self { hw0: None, hw1: None, sw: HashMap::new(), gpio })
    }

    fn hw_handle(&mut self, channel: PwmChannel) -> Result<&mut Option<RppalPwm>> {
        match channel {
            PwmChannel::Hardware0 => Ok(&mut self.hw0),
            PwmChannel::Hardware1 => Ok(&mut self.hw1),
            PwmChannel::Software(_) => {
                Err(RoboticsError::Hardware("not a hardware channel".into()))
            }
        }
    }
}

impl Pwm for LinuxPwm {
    fn set_frequency(&mut self, channel: PwmChannel, hz: f64) -> Result<()> {
        match channel {
            PwmChannel::Hardware0 | PwmChannel::Hardware1 => {
                let ch = if matches!(channel, PwmChannel::Hardware0) {
                    Channel::Pwm0
                } else {
                    Channel::Pwm1
                };
                let pwm = RppalPwm::with_frequency(ch, hz, 0.0, Polarity::Normal, true)
                    .map_err(|e| RoboticsError::Hardware(e.to_string()))?;
                *self.hw_handle(channel)? = Some(pwm);
                Ok(())
            }
            PwmChannel::Software(pin) => {
                let period = Duration::from_secs_f64(1.0 / hz);
                let p = self
                    .gpio
                    .get(pin)
                    .map_err(|e| RoboticsError::Hardware(e.to_string()))?
                    .into_output();
                self.sw.insert(pin, SoftwarePwm { pin: p, period });
                Ok(())
            }
        }
    }

    fn set_duty(&mut self, channel: PwmChannel, duty: f64) -> Result<()> {
        let duty = duty.clamp(0.0, 1.0);
        match channel {
            PwmChannel::Hardware0 | PwmChannel::Hardware1 => {
                if let Some(p) = self.hw_handle(channel)?.as_mut() {
                    p.set_duty_cycle(duty)
                        .map_err(|e| RoboticsError::Hardware(e.to_string()))?;
                } else {
                    return Err(RoboticsError::Hardware(
                        "channel not initialized: call set_frequency first".into(),
                    ));
                }
                Ok(())
            }
            PwmChannel::Software(pin) => {
                if let Some(s) = self.sw.get_mut(&pin) {
                    let pulse = s.period.mul_f64(duty);
                    s.pin
                        .set_pwm(s.period, pulse)
                        .map_err(|e| RoboticsError::Hardware(e.to_string()))?;
                } else {
                    return Err(RoboticsError::Hardware(
                        "software pin not initialized".into(),
                    ));
                }
                Ok(())
            }
        }
    }

    fn disable(&mut self, channel: PwmChannel) -> Result<()> {
        match channel {
            PwmChannel::Hardware0 | PwmChannel::Hardware1 => {
                if let Some(p) = self.hw_handle(channel)?.as_mut() {
                    p.disable()
                        .map_err(|e| RoboticsError::Hardware(e.to_string()))?;
                }
                Ok(())
            }
            PwmChannel::Software(pin) => {
                if let Some(s) = self.sw.get_mut(&pin) {
                    s.pin
                        .clear_pwm()
                        .map_err(|e| RoboticsError::Hardware(e.to_string()))?;
                }
                Ok(())
            }
        }
    }
}
