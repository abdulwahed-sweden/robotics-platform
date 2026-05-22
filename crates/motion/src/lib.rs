//! # robotics-motion
//!
//! Motion planning. Turns a *destination* (a joint state, or — via
//! IK — a pose) into a *trajectory*: a sequence of timed joint
//! setpoints that a backend can execute.
//!
//! The module is split into:
//!
//! * [`easing`] — pure-math functions mapping `t ∈ [0,1]` to a
//!   smoothed parameter. Cubic and quintic are the two that matter
//!   for robotics; linear is included for testing.
//! * [`trajectory`] — a joint-space trajectory with time scaling
//!   that respects per-joint velocity limits.
//! * [`queue`] — a FIFO of trajectories the planner emits and the
//!   backend consumes.
//! * [`planner`] — high-level `MotionPlanner` that composes IK +
//!   easing + time-scaling.

pub mod easing;
pub mod planner;
pub mod queue;
pub mod trajectory;

pub use easing::{Easing, EasingFn};
pub use planner::MotionPlanner;
pub use queue::MotionQueue;
pub use trajectory::{JointTrajectory, TrajectorySample};
