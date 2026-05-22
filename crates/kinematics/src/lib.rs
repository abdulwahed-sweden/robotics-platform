//! # robotics-kinematics
//!
//! Forward and inverse kinematics for an anthropomorphic 5-DOF arm:
//!
//! ```text
//!     base (yaw, around Z)
//!       │
//!     shoulder (pitch, in YZ-after-base plane)
//!       │   l1
//!     elbow (pitch)
//!       │   l2
//!     wrist (pitch)
//!       │   l3   ┐
//!     gripper          └── end-effector
//! ```
//!
//! We pick an analytic (closed-form) solver rather than an iterative
//! one (Jacobian pseudo-inverse, CCD, FABRIK) because:
//!
//! 1. The geometry is simple enough that closed form exists.
//! 2. Closed-form solvers are deterministic and constant-time. A
//!    motion planner that calls IK every tick can't afford the
//!    convergence loop of an iterative solver.
//! 3. We get useful failure signals: if `acos` argument is out of
//!    range, the target is *provably* unreachable — no "did we just
//!    fail to converge?" ambiguity.
//!
//! See `docs/kinematics.md` for the full derivation.

pub mod arm_model;
pub mod fk;
pub mod ik;

pub use arm_model::ArmModel;
pub use fk::forward_kinematics;
pub use ik::{inverse_kinematics, IkSolution};
