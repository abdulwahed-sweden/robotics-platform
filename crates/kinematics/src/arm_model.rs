//! Geometric description of the arm.
//!
//! Loaded from `configs/arm.toml`. All lengths are in meters, all
//! angles in radians. Nothing here is hardcoded; the kinematics
//! functions take `&ArmModel` so the same code works for a desk-sized
//! SG90 arm and a meter-tall industrial unit.

use robotics_core::JointLimits;
use serde::{Deserialize, Serialize};

/// Link lengths of the 3-link planar sub-chain (the part of the arm
/// that does the reaching, after base yaw). The diagram in the crate
/// docs shows where each one lives.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ArmModel {
    /// Height of the shoulder pivot above the base mount plate (z).
    /// Often called the "base height". Adds a constant offset to the
    /// reachable workspace.
    pub base_height: f64,
    /// Length from shoulder pivot to elbow pivot.
    pub l1: f64,
    /// Length from elbow pivot to wrist pivot.
    pub l2: f64,
    /// Length from wrist pivot to the gripper tool-center-point.
    pub l3: f64,

    pub base_limits: JointLimits,
    pub shoulder_limits: JointLimits,
    pub elbow_limits: JointLimits,
    pub wrist_limits: JointLimits,
    pub gripper_limits: JointLimits,
}

impl ArmModel {
    /// A sensible default — roughly an SG90-based hobbyist arm. Useful
    /// for tests; production configs always come from `configs/arm.toml`.
    pub fn sg90_default() -> Self {
        use std::f64::consts::PI;
        let wide = JointLimits {
            min_rad: -PI,
            max_rad: PI,
            max_velocity: 3.0,
            max_acceleration: 6.0,
        };
        // Shoulder: full pitch range up and forward. Real SG90s are
        // physically limited to ~0..π but the kinematics layer takes
        // the *mathematical* limits; mechanical limits are enforced
        // again at the hardware backend.
        let shoulder = JointLimits {
            min_rad: -PI / 2.0,
            max_rad: PI,
            max_velocity: 3.0,
            max_acceleration: 6.0,
        };
        let elbow = JointLimits {
            min_rad: -PI,
            max_rad: PI,
            max_velocity: 3.0,
            max_acceleration: 6.0,
        };
        // Wrist needs ±π to allow top-down approaches across the full
        // shoulder range — the total pitch (s + e + w) must reach φ,
        // and for top-down (φ = -π/2) with a high shoulder this drives
        // the wrist near ±π.
        let wrist = JointLimits {
            min_rad: -PI,
            max_rad: PI,
            max_velocity: 3.0,
            max_acceleration: 6.0,
        };
        Self {
            base_height: 0.05,
            l1: 0.10,
            l2: 0.10,
            l3: 0.08,
            base_limits: wide,
            shoulder_limits: shoulder,
            elbow_limits: elbow,
            wrist_limits: wrist,
            gripper_limits: JointLimits {
                min_rad: 0.0,
                max_rad: 1.2,
                max_velocity: 3.0,
                max_acceleration: 6.0,
            },
        }
    }

    /// Maximum end-effector reach in the horizontal plane (with the
    /// arm fully extended). Useful as a cheap rejection test before
    /// running the full IK.
    pub fn max_reach(&self) -> f64 {
        self.l1 + self.l2 + self.l3
    }
}
