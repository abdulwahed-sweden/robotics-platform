//! # robotics-planner
//!
//! Two things live here:
//!
//! 1. The **state machine** ([`state`]) — an explicit `RobotState`
//!    enum with a transition table the rest of the system queries.
//!    No state lives in the planner's instance variables; everything
//!    is in the enum, which means an external observer can dump it
//!    to JSON and see exactly what the robot is doing.
//!
//! 2. The **task planner** ([`pick_place`]) — composes IK, motion
//!    trajectories, and gripper actions into useful behaviours like
//!    "pick up the cube at (x, y, z) and drop it at (x', y', z')".
//!    Task planning here is intentionally simple — no graph search,
//!    no PDDL — because the platform deliberately keeps the planner
//!    swappable. Plug in something heavier (MoveIt, ROS 2 Nav2,
//!    BehaviorTree.CPP) later if you need to.

pub mod pick_place;
pub mod state;

pub use pick_place::PickPlaceTask;
pub use state::{RobotState, StateMachine, Transition};
