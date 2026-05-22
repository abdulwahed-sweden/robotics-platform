//! # robotics-hardware
//!
//! Hardware backend — the real-iron counterpart of `robotics-simulation`.
//! Both implement `robotics_core::Backend`, so the planner and CLI
//! treat them interchangeably.
//!
//! ## Architecture
//!
//! ```text
//!   HardwareBackend
//!     ├── 5x ServoChannel (gpio)   ◄── radians, calibrated → duty
//!     ├── SafetyMonitor             ◄── enforces limits + e-stop
//!     └── TelemetryThread           ◄── publishes current state
//! ```
//!
//! The hardware has no native position feedback (SG90 is open-loop);
//! we model "current position" by integrating the commanded velocity
//! over time, the same way the simulator does. This is good enough
//! for the planner. If you upgrade to encoders or smart servos,
//! replace the integration with real feedback in [`backend::HardwareBackend::telemetry_state`].
//!
//! ## Safety
//!
//! The backend owns the e-stop. On `emergency_stop()` it
//! (1) clears the trajectory queue, (2) calls `disable()` on every
//! servo channel, and (3) refuses subsequent commands until
//! `reset()` is called. See `docs/safety.md`.

pub mod backend;
pub mod calibration;
pub mod config;

pub use backend::HardwareBackend;
pub use config::HardwareConfig;
