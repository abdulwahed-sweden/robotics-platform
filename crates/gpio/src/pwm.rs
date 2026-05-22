//! PWM trait. The unit of platform-portability for the hardware
//! backend: every servo we drive lives behind a `&mut dyn Pwm` so the
//! servo logic doesn't know whether the underlying transport is
//! `/sys/class/pwm`, an I²C HAT, or a Dynamixel U2D2.

use robotics_core::Result;
use serde::{Deserialize, Serialize};

/// Which hardware PWM channel a servo lives on. Channel numbering
/// matches the Pi's two hardware PWM channels (0 and 1). Software PWM
/// (rppal's `pwm_pin` modes) is wrapped behind the same enum and
/// dispatched in the linux backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PwmChannel {
    /// Hardware PWM channel 0 — Pi GPIO 12 or 18.
    Hardware0,
    /// Hardware PWM channel 1 — Pi GPIO 13 or 19.
    Hardware1,
    /// Software PWM on the given BCM pin number. Adequate for SG90s
    /// at 50 Hz; jittery above ~200 Hz.
    Software(u8),
}

pub trait Pwm: Send + Sync {
    /// Configure the carrier frequency. For SG90 it's 50 Hz.
    fn set_frequency(&mut self, channel: PwmChannel, hz: f64) -> Result<()>;

    /// Set the duty cycle as a fraction `[0.0, 1.0]`.
    fn set_duty(&mut self, channel: PwmChannel, duty: f64) -> Result<()>;

    /// Stop generating pulses on the channel. After this the servo
    /// is unpowered and will not hold position against gravity.
    fn disable(&mut self, channel: PwmChannel) -> Result<()>;
}
