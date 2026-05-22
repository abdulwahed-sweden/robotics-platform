//! Stub PWM. Logs every command instead of executing it.
//!
//! Used in two situations:
//!
//! 1. **Non-Linux development.** macOS and Windows lack `/sys/class/pwm`
//!    so we can't drive the Pi from them anyway. The stub lets us
//!    compile and run the hardware backend in `--dry-run` mode to
//!    sanity-check commands, timing, and limit enforcement.
//! 2. **CI.** The CI runners don't have GPIO hardware. The stub
//!    keeps integration tests for the hardware backend executable
//!    without a Pi in the loop.

use robotics_core::Result;
use tracing::info;

use crate::pwm::{Pwm, PwmChannel};

#[derive(Debug, Default)]
pub struct StubPwm;

impl StubPwm {
    pub fn new() -> Self {
        Self
    }
}

impl Pwm for StubPwm {
    fn set_frequency(&mut self, channel: PwmChannel, hz: f64) -> Result<()> {
        info!(?channel, hz, "stub pwm: set_frequency");
        Ok(())
    }

    fn set_duty(&mut self, channel: PwmChannel, duty: f64) -> Result<()> {
        info!(?channel, duty, "stub pwm: set_duty");
        Ok(())
    }

    fn disable(&mut self, channel: PwmChannel) -> Result<()> {
        info!(?channel, "stub pwm: disable");
        Ok(())
    }
}
