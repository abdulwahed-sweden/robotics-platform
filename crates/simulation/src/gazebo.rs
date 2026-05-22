//! Gazebo bridge.
//!
//! ## What this module is, and isn't
//!
//! It **is** the seam where the platform talks to Gazebo. It defines a
//! transport-agnostic [`GazeboBridge`] trait, a `NullBridge`
//! implementation that no-ops (so the rest of the platform compiles
//! without Gazebo installed), and a documented path to swap in a real
//! bridge.
//!
//! It is **not** the bridge itself. A real Gazebo connection requires
//! one of:
//!
//! * **ROS 2 + ros_gz_bridge** — publishes `JointState` on `/joint_states`
//!   and consumes it inside Gazebo via the `JointPositionController`
//!   plugin. The Rust side uses [`r2r`](https://github.com/sequenceplanner/r2r)
//!   or [`rclrs`](https://github.com/ros2-rust/ros2_rust) for ROS messaging.
//! * **gz-transport (Ignition Transport)** — the native Gazebo IPC.
//!   No Rust binding exists today; you'd FFI through C++. ROS 2 is
//!   the path of least resistance.
//! * **A custom Gazebo system plugin** — C++ shared library loaded into
//!   `gz sim`. Talks to Rust over a Unix domain socket or shared memory.
//!   Heaviest but lowest latency.
//!
//! See `docs/simulation.md` for the recipe to wire each one up.

use async_trait::async_trait;
use robotics_core::{JointState, Result};

#[async_trait]
pub trait GazeboBridge: Send + Sync {
    /// Push the current joint state to Gazebo. Called every tick.
    async fn publish_joint_state(&mut self, state: &JointState) -> Result<()>;

    /// Pull pose updates back (object positions, sensor readings) so
    /// the platform stays in sync with the simulator.
    async fn poll_world(&mut self) -> Result<()>;
}

/// Default no-op bridge so the workspace compiles without Gazebo. The
/// in-process simulator alone is enough for tests and headless runs.
pub struct NullBridge;

#[async_trait]
impl GazeboBridge for NullBridge {
    async fn publish_joint_state(&mut self, _state: &JointState) -> Result<()> {
        Ok(())
    }
    async fn poll_world(&mut self) -> Result<()> {
        Ok(())
    }
}
