//! # robotics-gpio
//!
//! Thin abstraction over Raspberry Pi PWM. The hardware backend uses
//! this crate; nothing else should.
//!
//! ## What's here
//!
//! * [`Pwm`] trait — set frequency and duty cycle on a channel.
//! * [`servo::ServoChannel`] — angle → duty conversion calibrated per
//!   servo (SG90s vary 5-10% between units).
//! * Two backends:
//!   - `linux` (default on `cfg(target_os = "linux")`) — drives
//!     `/sys/class/pwm` via `rppal`.
//!   - `stub` (every other OS) — logs commands instead of executing
//!     them. Lets you `cargo run -- hardware --dry-run` on a Mac to
//!     sanity-check timing without a Pi.
//!
//! ## Why a separate crate
//!
//! Two reasons: (1) it isolates the platform-specific dependency
//! behind a clean interface, and (2) it lets us swap in
//! microcontroller transports later (CAN, I²C servo HAT, Dynamixel
//! bus) without touching the hardware backend.

pub mod pwm;
pub mod servo;

#[cfg(target_os = "linux")]
mod linux_pwm;

mod stub_pwm;

#[cfg(target_os = "linux")]
pub use linux_pwm::LinuxPwm;
pub use pwm::{Pwm, PwmChannel};
pub use servo::{ServoCalibration, ServoChannel};
pub use stub_pwm::StubPwm;
