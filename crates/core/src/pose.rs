//! Pose types. Thin wrappers around nalgebra primitives so the rest of
//! the platform can talk about positions and orientations without
//! depending directly on nalgebra (and so the public API doesn't break
//! when nalgebra majors).

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

/// A 3D point in robot base frame, in meters. Right-handed coordinates:
/// X forward, Y left, Z up. Matches ROS REP 103.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn norm(self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn to_na(self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.z)
    }

    pub fn from_na(v: Vector3<f64>) -> Self {
        Self { x: v.x, y: v.y, z: v.z }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub const IDENTITY: Quaternion = Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    pub fn to_na(self) -> UnitQuaternion<f64> {
        UnitQuaternion::new_normalize(nalgebra::Quaternion::new(self.w, self.x, self.y, self.z))
    }
}

/// Full 6-DoF pose. Used by the motion planner and IK solver as the
/// "what does the end-effector want to be" specification.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub position: Vec3,
    pub orientation: Quaternion,
}

impl Pose {
    pub const fn at(position: Vec3) -> Self {
        Self { position, orientation: Quaternion::IDENTITY }
    }

    pub fn to_isometry(self) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(self.position.x, self.position.y, self.position.z),
            self.orientation.to_na(),
        )
    }
}
