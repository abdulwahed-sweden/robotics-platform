//! High-level motion planner.
//!
//! The planner is the bridge between the *Cartesian* world (poses,
//! pick locations) and the *joint* world (trajectories, easing).
//! It does:
//!
//! 1. Take a target pose.
//! 2. Solve IK → target joint state.
//! 3. Build a time-scaled trajectory from the current state to the
//!    target, respecting per-joint velocity limits.
//! 4. Hand the trajectory off to the queue.
//!
//! The planner deliberately does *not* execute the trajectory — that
//! is the backend's job. Keeping plan and execute separate lets us
//! unit-test the planner without a backend, and lets us swap
//! simulation for hardware without touching planner code.

use robotics_core::{JointId, JointState, Result, Vec3};
use robotics_kinematics::{inverse_kinematics, ArmModel};
use tracing::{debug, instrument};

use crate::easing::Easing;
use crate::trajectory::JointTrajectory;

pub struct MotionPlanner {
    model: ArmModel,
    default_easing: Easing,
}

impl MotionPlanner {
    pub fn new(model: ArmModel) -> Self {
        Self { model, default_easing: Easing::Quintic }
    }

    pub fn with_easing(mut self, easing: Easing) -> Self {
        self.default_easing = easing;
        self
    }

    pub fn model(&self) -> &ArmModel {
        &self.model
    }

    /// Plan a move from `current` joint state to a Cartesian `target`
    /// with the given top-down/horizontal approach pitch.
    #[instrument(skip(self, current), fields(target = ?target))]
    pub fn plan_to_pose(
        &self,
        current: JointState,
        target: Vec3,
        approach_pitch: f64,
        gripper: f64,
    ) -> Result<JointTrajectory> {
        let sol = inverse_kinematics(&self.model, target, approach_pitch, gripper)?;
        debug!(?sol.joints, "IK solved");
        Ok(self.plan_to_joints(current, sol.joints))
    }

    /// Plan a move directly in joint space. Useful for "go home"
    /// motions and for tests that bypass IK.
    pub fn plan_to_joints(&self, current: JointState, target: JointState) -> JointTrajectory {
        let limits = [
            (JointId::Base, self.model.base_limits.max_velocity),
            (JointId::Shoulder, self.model.shoulder_limits.max_velocity),
            (JointId::Elbow, self.model.elbow_limits.max_velocity),
            (JointId::Wrist, self.model.wrist_limits.max_velocity),
            (JointId::Gripper, self.model.gripper_limits.max_velocity),
        ];
        JointTrajectory::time_scaled(current, target, self.default_easing, &limits)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn planner_emits_reachable_trajectory() {
        let model = ArmModel::sg90_default();
        let planner = MotionPlanner::new(model);
        let start = JointState::default();
        let traj = planner
            .plan_to_pose(
                start,
                Vec3::new(0.12, 0.0, 0.10),
                -std::f64::consts::FRAC_PI_2,
                0.5,
            )
            .unwrap();
        assert!(traj.duration.as_secs_f64() > 0.0);
    }
}
