//! # robotics-simulation
//!
//! Software backend for the platform. Implements the same `Backend`
//! trait the hardware backend does, so the CLI and planner are oblivious
//! to which is active.
//!
//! The simulator is *kinematic*, not dynamic: it integrates joint
//! positions toward their targets at the configured velocity, without
//! masses, inertias, or contact forces. That is the right level of
//! fidelity for verifying motion plans and pick-and-place logic. When
//! you need real physics, hand the joint state off to Gazebo via the
//! `gazebo` bridge.
//!
//! ## Why an in-process sim *and* a Gazebo bridge
//!
//! 1. **Fast feedback.** The in-process sim runs at any tick rate, no
//!    external process, no IPC, no rendering. CI runs it; unit tests
//!    run it. Iteration loop stays under a second.
//! 2. **Visual validation.** Gazebo gives you a 3D view, contact
//!    physics, and camera sensors — invaluable for debugging an IK
//!    solution that "should" work but doesn't visually pan out.
//!
//! Most production robotics teams end up with both. So we ship both.

pub mod backend;
pub mod gazebo;
pub mod world;

pub use backend::SimulationBackend;
pub use world::{SimObject, SimWorld};
