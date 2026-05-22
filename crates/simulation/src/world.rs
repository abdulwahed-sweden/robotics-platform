//! Simulated world state — joints, objects, gripper attachment.
//!
//! Lives behind a `Mutex` inside the backend; the tick task and the
//! command handlers both grab it. Kept deliberately small and
//! observable (`Clone` on `SimObject`, no interior mutability inside).

use robotics_core::{GripperState, JointState, Vec3};
use robotics_kinematics::{forward_kinematics, ArmModel};
use serde::{Deserialize, Serialize};

/// An object the arm can interact with. The position is updated in
/// lockstep with the end-effector when [`SimWorld::attached`] points
/// to this object, modelling the gripper's hold.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimObject {
    pub id: String,
    pub position: Vec3,
    /// How close (in meters) the end-effector must be to consider the
    /// object grabbable. Bigger for clumsy hardware; tight for sim.
    pub grasp_radius: f64,
}

#[derive(Debug)]
pub struct SimWorld {
    pub model: ArmModel,
    pub joints: JointState,
    pub commanded: JointState,
    pub gripper_state: GripperState,
    pub objects: Vec<SimObject>,
    /// Index into `objects` of the currently held object, if any.
    pub attached: Option<usize>,
}

impl SimWorld {
    pub fn new(model: ArmModel, objects: Vec<SimObject>) -> Self {
        let joints = JointState::default();
        Self {
            model,
            joints,
            commanded: joints,
            gripper_state: GripperState::Open,
            objects,
            attached: None,
        }
    }

    /// World-frame position of the end-effector right now. The sim
    /// uses this for the gripper's pickup proximity check.
    pub fn end_effector(&self) -> Vec3 {
        let iso = forward_kinematics(&self.model, &self.joints);
        Vec3::new(iso.translation.x, iso.translation.y, iso.translation.z)
    }

    /// Advance the kinematic state by `dt` seconds, moving each joint
    /// toward its commanded position at no more than its configured
    /// max velocity. Position-only — no velocity state — which is
    /// fine for a kinematic sim.
    pub fn tick(&mut self, dt: f64) {
        step(&mut self.joints.base, self.commanded.base, self.model.base_limits.max_velocity, dt);
        step(
            &mut self.joints.shoulder,
            self.commanded.shoulder,
            self.model.shoulder_limits.max_velocity,
            dt,
        );
        step(
            &mut self.joints.elbow,
            self.commanded.elbow,
            self.model.elbow_limits.max_velocity,
            dt,
        );
        step(
            &mut self.joints.wrist,
            self.commanded.wrist,
            self.model.wrist_limits.max_velocity,
            dt,
        );
        step(
            &mut self.joints.gripper,
            self.commanded.gripper,
            self.model.gripper_limits.max_velocity,
            dt,
        );

        // If we're holding something, drag it along with the gripper.
        if let Some(idx) = self.attached {
            let ee = self.end_effector();
            if let Some(obj) = self.objects.get_mut(idx) {
                obj.position = ee;
            }
        }
    }

    /// Try to attach the nearest in-range object. Returns true if
    /// something was grabbed. Idempotent if already attached.
    pub fn attempt_grasp(&mut self) -> bool {
        if self.attached.is_some() {
            self.gripper_state = GripperState::Holding;
            return true;
        }
        let ee = self.end_effector();
        let candidate = self
            .objects
            .iter()
            .enumerate()
            .map(|(i, o)| (i, distance(o.position, ee), o.grasp_radius))
            .filter(|(_, d, r)| d <= r)
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        match candidate {
            Some((i, _, _)) => {
                self.attached = Some(i);
                self.gripper_state = GripperState::Holding;
                true
            }
            None => false,
        }
    }

    pub fn release(&mut self) {
        self.attached = None;
        self.gripper_state = GripperState::Open;
    }
}

fn step(current: &mut f64, target: f64, max_vel: f64, dt: f64) {
    let delta = target - *current;
    let max_step = max_vel * dt;
    *current += delta.clamp(-max_step, max_step);
}

fn distance(a: Vec3, b: Vec3) -> f64 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joint_steps_toward_target_at_velocity_limit() {
        let model = ArmModel::sg90_default();
        let mut world = SimWorld::new(model, vec![]);
        world.commanded.base = 1.0;
        // 1 rad target, 3 rad/s limit, 0.1 s tick → 0.3 rad advance.
        world.tick(0.1);
        assert!((world.joints.base - 0.3).abs() < 1e-9);
    }

    #[test]
    fn grasp_picks_up_object_in_range() {
        let model = ArmModel::sg90_default();
        let mut world = SimWorld::new(
            model,
            vec![SimObject {
                id: "cube".into(),
                position: world_end_effector(&ArmModel::sg90_default(), JointState::default()),
                grasp_radius: 0.05,
            }],
        );
        assert!(world.attempt_grasp());
        assert!(world.attached.is_some());
    }

    fn world_end_effector(model: &ArmModel, joints: JointState) -> Vec3 {
        let iso = forward_kinematics(model, &joints);
        Vec3::new(iso.translation.x, iso.translation.y, iso.translation.z)
    }
}
