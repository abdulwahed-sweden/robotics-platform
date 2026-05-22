//! # robotics-core
//!
//! The foundation crate of the robotics platform. Every other crate in
//! the workspace depends on this one and *only* this one for cross-cutting
//! types and traits. The rule is deliberate: it forces the platform to
//! have a single, stable contract surface between layers.
//!
//! ## Layers
//!
//! ```text
//!   ┌──────────────────────────────────────────────────────┐
//!   │ cli            high-level user entry point           │
//!   ├──────────────────────────────────────────────────────┤
//!   │ planner        task + state machine                  │
//!   ├──────────────────────────────────────────────────────┤
//!   │ motion         trajectories, easing, queues          │
//!   ├──────────────────────────────────────────────────────┤
//!   │ kinematics     FK / IK                               │
//!   ├──────────────────────────────────────────────────────┤
//!   │ core           ◄── you are here                      │
//!   ├──────────────────────────────────────────────────────┤
//!   │ simulation │ hardware  (backends implement core API) │
//!   └──────────────────────────────────────────────────────┘
//! ```
//!
//! The upper layers know nothing about whether they're driving a real
//! servo or a simulated rigid body. They speak in [`JointCommand`]s and
//! observe [`JointTelemetry`]; the backend trait is what makes the same
//! motion code run on a Raspberry Pi and inside Gazebo.

pub mod error;
pub mod joint;
pub mod pose;
pub mod telemetry;
pub mod traits;
pub mod time;

pub use error::{RoboticsError, Result};
pub use joint::{JointCommand, JointId, JointLimits, JointState, JointTelemetry};
pub use pose::{Pose, Quaternion, Vec3};
pub use telemetry::{TelemetryHub, TelemetryRx};
pub use time::{Duration, Instant};
pub use traits::{
    Backend, Gripper, GripperState, JointController, MotorController, RobotArm, Sensor,
    SensorReading,
};
