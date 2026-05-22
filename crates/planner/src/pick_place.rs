//! Pick-and-place task.
//!
//! Drives the state machine through the canonical sequence:
//!
//! 1. Move above the pick target (pre-grasp pose, gripper open).
//! 2. Descend onto the target.
//! 3. Close the gripper.
//! 4. Lift back to the pre-grasp pose.
//! 5. Move above the place target.
//! 6. Descend.
//! 7. Open the gripper.
//! 8. Retract.
//!
//! Each step is a planned trajectory + a gripper command, executed
//! sequentially against any `Backend`. This is intentionally
//! procedural — task planners that need branching/parallel actions
//! should pull in a behavior tree library instead of growing this
//! file.

use std::time::Duration;

use robotics_core::{Backend, Result, Vec3};
use robotics_kinematics::ArmModel;
use robotics_motion::MotionPlanner;
use tokio::time::sleep;
use tracing::{info, instrument};

use crate::state::{RobotState, StateMachine, Transition};

/// Height (m) added above the pick/place points to plan a pre-grasp
/// pose. Two-stage descent avoids dragging the gripper through other
/// objects on the table.
const PREGRASP_HEIGHT: f64 = 0.05;

pub struct PickPlaceTask {
    pub pick: Vec3,
    pub place: Vec3,
    /// Approach pitch in radians. `-π/2` is straight down (the default).
    pub approach_pitch: f64,
}

impl PickPlaceTask {
    /// Top-down pick at `pick`, top-down place at `place`.
    pub fn top_down(pick: Vec3, place: Vec3) -> Self {
        Self { pick, place, approach_pitch: -std::f64::consts::FRAC_PI_2 }
    }

    /// Execute the full task. Mutates the supplied state machine so
    /// callers can observe progress externally (e.g., via a telemetry
    /// channel reading `sm.state`).
    #[instrument(skip(self, backend, sm), fields(pick = ?self.pick, place = ?self.place))]
    pub async fn execute(
        &self,
        backend: &mut dyn Backend,
        model: &ArmModel,
        sm: &mut StateMachine,
    ) -> Result<()> {
        let planner = MotionPlanner::new(*model);

        // 1. Plan + move to pre-grasp above pick.
        sm.transition(Transition::BeginPlanning)?;
        let pregrasp = above(self.pick);
        self.go_to(backend, &planner, pregrasp, self.approach_pitch, OPEN, sm).await?;

        // 2. Descend to pick.
        sm.transition(Transition::BeginPlanning)?;
        self.go_to(backend, &planner, self.pick, self.approach_pitch, OPEN, sm).await?;
        sm.transition(Transition::AtPickPose)?;

        // 3. Close gripper.
        info!("closing gripper");
        backend.gripper().close().await?;
        sleep(Duration::from_millis(500)).await; // mechanical settle
        sm.transition(Transition::Grasped)?;

        // 4. Lift.
        sm.transition(Transition::BeginPlanning)?;
        self.go_to(backend, &planner, pregrasp, self.approach_pitch, CLOSED, sm).await?;

        // 5. Move above place.
        sm.transition(Transition::BeginPlanning)?;
        let above_place = above(self.place);
        self.go_to(backend, &planner, above_place, self.approach_pitch, CLOSED, sm).await?;

        // 6. Descend.
        sm.transition(Transition::BeginPlanning)?;
        self.go_to(backend, &planner, self.place, self.approach_pitch, CLOSED, sm).await?;
        sm.transition(Transition::AtPlacePose)?;

        // 7. Release.
        info!("opening gripper");
        backend.gripper().open().await?;
        sleep(Duration::from_millis(500)).await;
        sm.transition(Transition::Released)?;

        // 8. Retract.
        sm.transition(Transition::BeginPlanning)?;
        self.go_to(backend, &planner, above_place, self.approach_pitch, OPEN, sm).await?;
        sm.transition(Transition::Complete)?;
        Ok(())
    }

    async fn go_to(
        &self,
        backend: &mut dyn Backend,
        planner: &MotionPlanner,
        target: Vec3,
        approach: f64,
        gripper: f64,
        sm: &mut StateMachine,
    ) -> Result<()> {
        let current = backend.arm().joint_state().await?;
        let traj = planner.plan_to_pose(current, target, approach, gripper)?;
        // Allow the transition to Moving (PlanReady) only if the
        // state machine is in Targeting; on the in-trajectory leg
        // (e.g. lift after grasping) we're already in Carrying so
        // skip the transition.
        if matches!(sm.state, RobotState::Targeting) {
            sm.transition(Transition::PlanReady)?;
        }
        backend.arm().apply_state(traj.end).await?;
        // Wait the trajectory duration so the backend has time to
        // integrate. A production system reads telemetry until
        // `at_target` is true on every joint.
        sleep(traj.duration).await;
        Ok(())
    }
}

fn above(p: Vec3) -> Vec3 {
    Vec3::new(p.x, p.y, p.z + PREGRASP_HEIGHT)
}

const OPEN: f64 = 1.0;
const CLOSED: f64 = 0.0;
