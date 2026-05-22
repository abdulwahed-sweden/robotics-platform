//! Forward kinematics.
//!
//! Given the joint angles, compute the end-effector position. This is
//! the easy direction — just walk the chain.
//!
//! ## Derivation
//!
//! Let θ_b, θ_s, θ_e, θ_w be base/shoulder/elbow/wrist angles. The
//! shoulder, elbow, and wrist live in the vertical plane containing
//! the base yaw axis, so the planar problem gives us the (r, z)
//! position of the end-effector relative to the shoulder:
//!
//! ```text
//! r = l1·cos(θ_s) + l2·cos(θ_s + θ_e) + l3·cos(θ_s + θ_e + θ_w)
//! z = l1·sin(θ_s) + l2·sin(θ_s + θ_e) + l3·sin(θ_s + θ_e + θ_w)
//! ```
//!
//! Then base yaw rotates (r, 0) into (x, y):
//!
//! ```text
//! x = r·cos(θ_b)
//! y = r·sin(θ_b)
//! z = z + base_height
//! ```
//!
//! That's it. No matrices needed — but we still wrap the result as an
//! `Isometry3` for callers who want the full pose.

use nalgebra::{Isometry3, Rotation3, Translation3, UnitQuaternion, Vector3};
use robotics_core::JointState;

use crate::arm_model::ArmModel;

/// Returns the end-effector position in robot base frame.
pub fn forward_kinematics(model: &ArmModel, joints: &JointState) -> Isometry3<f64> {
    let theta_b = joints.base;
    let theta_s = joints.shoulder;
    let theta_e = joints.elbow;
    let theta_w = joints.wrist;

    let s1 = theta_s;
    let s2 = theta_s + theta_e;
    let s3 = theta_s + theta_e + theta_w;

    // Planar position in the (radial, vertical) plane.
    let r = model.l1 * s1.cos() + model.l2 * s2.cos() + model.l3 * s3.cos();
    let z = model.l1 * s1.sin() + model.l2 * s2.sin() + model.l3 * s3.sin();

    // Rotate by base yaw into world frame.
    let x = r * theta_b.cos();
    let y = r * theta_b.sin();
    let world_z = z + model.base_height;

    // Orientation of the tool — base yaw composed with total pitch.
    // The total pitch in the planar frame is theta_s + theta_e + theta_w.
    let yaw = Rotation3::from_axis_angle(&Vector3::z_axis(), theta_b);
    let pitch = Rotation3::from_axis_angle(&Vector3::y_axis(), -s3);
    let rotation = UnitQuaternion::from_rotation_matrix(&(yaw * pitch));

    Isometry3::from_parts(Translation3::new(x, y, world_z), rotation)
}
