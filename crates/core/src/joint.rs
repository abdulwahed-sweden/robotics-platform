//! Joint-level types.
//!
//! A joint is the atomic actuated unit of the arm. Everything above
//! kinematics talks about *poses*; everything below talks about *joints*.
//! These types are the lingua franca between the two halves.

use serde::{Deserialize, Serialize};

use crate::error::{Result, RoboticsError};

/// Identifier for a single joint. We use an enum (rather than a string)
/// so the compiler can prove all joints are handled in match arms — a
/// common source of bugs when adding a new joint to the arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointId {
    Base,
    Shoulder,
    Elbow,
    Wrist,
    Gripper,
}

impl JointId {
    /// Iteration order is also the kinematic chain order — base first,
    /// gripper last. Several IK and FK routines rely on this.
    pub const ALL: [JointId; 5] = [
        JointId::Base,
        JointId::Shoulder,
        JointId::Elbow,
        JointId::Wrist,
        JointId::Gripper,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            JointId::Base => "base",
            JointId::Shoulder => "shoulder",
            JointId::Elbow => "elbow",
            JointId::Wrist => "wrist",
            JointId::Gripper => "gripper",
        }
    }
}

impl std::fmt::Display for JointId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Configurable per-joint mechanical and dynamic limits. Loaded from
/// `configs/arm.toml`; nothing in the platform hardcodes these.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct JointLimits {
    /// Minimum reachable angle in radians.
    pub min_rad: f64,
    /// Maximum reachable angle in radians.
    pub max_rad: f64,
    /// Maximum allowed angular velocity in rad/s. The motion planner
    /// uses this to time-scale trajectories.
    pub max_velocity: f64,
    /// Maximum allowed angular acceleration in rad/s².
    pub max_acceleration: f64,
}

impl JointLimits {
    /// Clamp an angle into the legal range, returning an error if the
    /// caller asked for something out of bounds. We return an error
    /// rather than silently clamping because in a control system,
    /// silent clamping hides bugs in the planner.
    pub fn check(&self, joint: JointId, angle_rad: f64) -> Result<()> {
        if angle_rad < self.min_rad || angle_rad > self.max_rad {
            return Err(RoboticsError::JointLimitViolation {
                joint: joint.to_string(),
                requested: angle_rad,
                min: self.min_rad,
                max: self.max_rad,
            });
        }
        Ok(())
    }
}

/// A complete snapshot of the arm's joint state. Five joints, one
/// position each, in radians. The order matches [`JointId::ALL`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct JointState {
    pub base: f64,
    pub shoulder: f64,
    pub elbow: f64,
    pub wrist: f64,
    pub gripper: f64,
}

impl JointState {
    pub fn new(base: f64, shoulder: f64, elbow: f64, wrist: f64, gripper: f64) -> Self {
        Self { base, shoulder, elbow, wrist, gripper }
    }

    pub fn get(&self, joint: JointId) -> f64 {
        match joint {
            JointId::Base => self.base,
            JointId::Shoulder => self.shoulder,
            JointId::Elbow => self.elbow,
            JointId::Wrist => self.wrist,
            JointId::Gripper => self.gripper,
        }
    }

    pub fn set(&mut self, joint: JointId, value: f64) {
        match joint {
            JointId::Base => self.base = value,
            JointId::Shoulder => self.shoulder = value,
            JointId::Elbow => self.elbow = value,
            JointId::Wrist => self.wrist = value,
            JointId::Gripper => self.gripper = value,
        }
    }

    /// Linear interpolation between two joint states. Used by the
    /// motion planner to produce intermediate setpoints. Easing is
    /// applied by the caller via the parameter `t`.
    pub fn lerp(a: Self, b: Self, t: f64) -> Self {
        let t = t.clamp(0.0, 1.0);
        Self {
            base: a.base + (b.base - a.base) * t,
            shoulder: a.shoulder + (b.shoulder - a.shoulder) * t,
            elbow: a.elbow + (b.elbow - a.elbow) * t,
            wrist: a.wrist + (b.wrist - a.wrist) * t,
            gripper: a.gripper + (b.gripper - a.gripper) * t,
        }
    }
}

/// A command issued to the backend: "drive this joint to this angle".
/// Velocity and acceleration limits are advisory; the backend may
/// further reduce them for safety.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct JointCommand {
    pub joint: JointId,
    pub target_rad: f64,
    pub max_velocity: f64,
    pub max_acceleration: f64,
}

/// Telemetry emitted by the backend at each control tick. Used by
/// higher layers to monitor execution and decide when a trajectory
/// is complete.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct JointTelemetry {
    pub joint: JointId,
    pub position_rad: f64,
    pub velocity: f64,
    pub at_target: bool,
}
