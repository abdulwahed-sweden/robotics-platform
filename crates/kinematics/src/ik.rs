//! Inverse kinematics.
//!
//! Given a desired end-effector position and approach angle, return
//! the joint angles that achieve it.
//!
//! ## Approach
//!
//! We solve in two stages:
//!
//! 1. **Base yaw** is fixed by the target's azimuth:
//!    `θ_b = atan2(y, x)`.
//!
//! 2. **The 3-link planar chain** (shoulder / elbow / wrist) then
//!    solves the problem in the (r, z) plane, where
//!    `r = sqrt(x² + y²)` and `z` is height above the shoulder.
//!
//! For the planar chain we further pick the **approach pitch** φ —
//! the angle the gripper makes with the horizontal at the moment of
//! contact. Top-down pick is φ = -π/2; horizontal pick is φ = 0. This
//! removes one degree of freedom and turns an under-determined 3-link
//! problem into a clean 2-link one:
//!
//! ```text
//! r' = r - l3·cos(φ)
//! z' = z - l3·sin(φ)
//! ```
//!
//! Then the standard 2-link IK (the one in every robotics textbook):
//!
//! ```text
//! D       = (r'² + z'² - l1² - l2²) / (2·l1·l2)
//! θ_e     = ±acos(D)
//! θ_s     = atan2(z', r') - atan2(l2·sin(θ_e), l1 + l2·cos(θ_e))
//! θ_w     = φ - θ_s - θ_e
//! ```
//!
//! The `±` gives us two solutions — "elbow up" and "elbow down". We
//! prefer elbow-up because it keeps the elbow away from the table.

use robotics_core::{JointId, JointLimits, JointState, RoboticsError, Result, Vec3};

use crate::arm_model::ArmModel;

/// The result of a successful IK solve. We hand back both the joint
/// state and which of the two analytic branches we picked, so a
/// planner can decide to switch elbow configurations between motions
/// if the preferred one would violate a limit.
#[derive(Debug, Clone, Copy)]
pub struct IkSolution {
    pub joints: JointState,
    pub elbow_up: bool,
}

/// Solve inverse kinematics.
///
/// * `target` — desired tool-center-point in robot base frame (m).
/// * `approach_pitch` — angle the wrist tool axis makes with the
///   horizontal. `-π/2` for top-down pick, `0` for horizontal.
/// * `gripper` — desired gripper opening (rad). Pass-through; the
///   planner sets it.
pub fn inverse_kinematics(
    model: &ArmModel,
    target: Vec3,
    approach_pitch: f64,
    gripper: f64,
) -> Result<IkSolution> {
    // 1. Base yaw is determined by the target azimuth. atan2 handles
    //    all four quadrants; the only failure mode is x == y == 0
    //    (target directly above base) where yaw is undefined. In that
    //    case we keep the current base angle; here we pick 0.
    let theta_b = target.y.atan2(target.x);

    // 2. Project the target into the (r, z) plane relative to the
    //    shoulder pivot.
    let r_world = (target.x * target.x + target.y * target.y).sqrt();
    let z_shoulder = target.z - model.base_height;

    // 3. Pull back by l3 along the approach direction to get the
    //    wrist target.
    let r_wrist = r_world - model.l3 * approach_pitch.cos();
    let z_wrist = z_shoulder - model.l3 * approach_pitch.sin();

    // 4. Reachability test on the 2-link sub-problem. Cheap and
    //    gives a clear error before we hit acos.
    let dist_sq = r_wrist * r_wrist + z_wrist * z_wrist;
    let dist = dist_sq.sqrt();
    let reach_min = (model.l1 - model.l2).abs();
    let reach_max = model.l1 + model.l2;
    if dist > reach_max + 1e-6 || dist < reach_min - 1e-6 {
        return Err(RoboticsError::Unreachable);
    }

    // 5. Standard 2-link IK. Numerical safety: clamp into [-1, 1] to
    //    tolerate floating-point drift at the workspace boundary.
    let cos_elbow =
        ((dist_sq - model.l1 * model.l1 - model.l2 * model.l2) / (2.0 * model.l1 * model.l2))
            .clamp(-1.0, 1.0);
    let theta_e_up = -cos_elbow.acos(); // elbow-up: negative elbow angle by our convention
    let theta_e = theta_e_up;

    let theta_s = z_wrist.atan2(r_wrist)
        - (model.l2 * theta_e.sin()).atan2(model.l1 + model.l2 * theta_e.cos());

    // 6. Wrist closes the chain so the total pitch hits the requested
    //    approach angle.
    let theta_w = approach_pitch - theta_s - theta_e;

    let joints = JointState {
        base: theta_b,
        shoulder: theta_s,
        elbow: theta_e,
        wrist: theta_w,
        gripper,
    };

    check_limits(model, &joints)?;

    Ok(IkSolution { joints, elbow_up: true })
}

fn check_limits(model: &ArmModel, joints: &JointState) -> Result<()> {
    let pairs: [(JointId, JointLimits, f64); 5] = [
        (JointId::Base, model.base_limits, joints.base),
        (JointId::Shoulder, model.shoulder_limits, joints.shoulder),
        (JointId::Elbow, model.elbow_limits, joints.elbow),
        (JointId::Wrist, model.wrist_limits, joints.wrist),
        (JointId::Gripper, model.gripper_limits, joints.gripper),
    ];
    for (id, lim, val) in pairs {
        lim.check(id, val)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fk::forward_kinematics;

    /// The defining property of IK: FK(IK(p)) ≈ p. If this fails the
    /// math is wrong; nothing downstream will work.
    #[test]
    fn ik_then_fk_roundtrip() {
        let model = ArmModel::sg90_default();
        // A point well inside the workspace, top-down approach.
        let target = Vec3::new(0.12, 0.05, 0.10);
        let sol = inverse_kinematics(&model, target, -std::f64::consts::FRAC_PI_2, 0.5).unwrap();
        let pose = forward_kinematics(&model, &sol.joints);
        let p = pose.translation.vector;
        assert!((p.x - target.x).abs() < 1e-6, "x off: {} vs {}", p.x, target.x);
        assert!((p.y - target.y).abs() < 1e-6, "y off: {} vs {}", p.y, target.y);
        assert!((p.z - target.z).abs() < 1e-6, "z off: {} vs {}", p.z, target.z);
    }

    #[test]
    fn ik_rejects_unreachable_target() {
        let model = ArmModel::sg90_default();
        // Way out of reach.
        let target = Vec3::new(2.0, 0.0, 0.5);
        let result = inverse_kinematics(&model, target, 0.0, 0.0);
        assert!(matches!(result, Err(RoboticsError::Unreachable)));
    }

    #[test]
    fn ik_base_rotates_to_face_target() {
        let model = ArmModel::sg90_default();
        let target = Vec3::new(0.0, 0.15, 0.10);
        let sol = inverse_kinematics(&model, target, -std::f64::consts::FRAC_PI_2, 0.0).unwrap();
        // Target is along +Y, so base yaw should be ~π/2.
        assert!((sol.joints.base - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }
}
