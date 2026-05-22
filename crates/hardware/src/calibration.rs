//! Calibration helpers.
//!
//! Two routines you'll need on a fresh arm:
//!
//! 1. **Center.** Drive every servo to its mechanical midpoint and
//!    leave it there. You then power off, unscrew the horn, and
//!    refit it so that midpoint corresponds to the geometric "home"
//!    posture of the arm (forearm horizontal, etc.). This makes the
//!    radian-to-pulse-width mapping match the kinematic model.
//!
//! 2. **Endstop sweep.** Slowly walk the servo from min to max in
//!    small steps; the operator records by eye the angles at which
//!    the link hits its mechanical stop. Those become the
//!    `JointLimits` in `arm.toml`.
//!
//! Both routines are runnable from the CLI: `cargo run -- calibrate
//! center` and `cargo run -- calibrate sweep --joint shoulder`.

use robotics_core::Result;
use robotics_gpio::ServoCalibration;

/// Bisection helper: given the current pulse-width range and an
/// observation ("the link is past the target" / "not yet"), return
/// the next pulse to try. Useful in the CLI calibration loop.
pub fn bisect(low_ms: f64, high_ms: f64, observed_past: bool) -> f64 {
    let mid = (low_ms + high_ms) / 2.0;
    if observed_past {
        // Overshot — search the lower half next.
        (low_ms + mid) / 2.0
    } else {
        // Undershot — search the upper half next.
        (mid + high_ms) / 2.0
    }
}

/// Sanity check before applying a calibration: pulses must be in the
/// 0.5..2.5 ms range typical for hobby servos, and min < max.
pub fn validate(cal: &ServoCalibration) -> Result<()> {
    use robotics_core::RoboticsError;
    if !(0.5..=2.5).contains(&cal.pulse_min_ms) || !(0.5..=2.5).contains(&cal.pulse_max_ms) {
        return Err(RoboticsError::Config(format!(
            "servo pulses outside 0.5..2.5 ms: {:?}",
            cal
        )));
    }
    if cal.pulse_min_ms >= cal.pulse_max_ms || cal.angle_min_rad >= cal.angle_max_rad {
        return Err(RoboticsError::Config("calibration min >= max".into()));
    }
    Ok(())
}
