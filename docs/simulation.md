# Simulation

The platform ships two simulators:

1. **In-process kinematic sim** (`SimulationBackend`). Default. No
   external dependencies. Used by tests, CI, and the
   `cargo run -- simulate` happy path.
2. **Gazebo bridge** (`gazebo::GazeboBridge`). A trait + a
   `NullBridge`. Wire it up when you need 3D rendering, contact
   physics, or camera sensors.

## In-process simulator

A kinematic — not dynamic — simulation. It tracks five joint
positions and integrates them toward their commanded values at the
configured per-joint velocity each tick. No masses, no inertias, no
contact forces.

That's the right fidelity for verifying motion plans and pick-and-
place logic. If your bug is "the IK gave wrong angles" or "the
trajectory accelerates too hard", the kinematic sim shows it. If your
bug is "the gripper slips under load", it doesn't — that's Gazebo's
job.

### Tick rate

200 Hz. Configured in `crates/simulation/src/backend.rs`. The tick
runs on a tokio interval with `MissedTickBehavior::Delay`, so if the
process gets paused (e.g., debugger), the simulator doesn't skip
forward to compensate. Determinism matters.

### Object model

`SimWorld` holds a `Vec<SimObject>` and a single `Option<usize>` of
the "currently attached" object. The gripper close handler tries
`attempt_grasp`, which finds the nearest object within its
`grasp_radius` of the current end-effector position and records the
attachment. Subsequent ticks drag the attached object along with the
gripper.

This is enough for the pick-and-place demo to work end-to-end against
the same trait surface the hardware uses.

## Gazebo bridge

`crates/simulation/src/gazebo.rs` defines `GazeboBridge` with two
methods:

```rust
async fn publish_joint_state(&mut self, state: &JointState) -> Result<()>;
async fn poll_world(&mut self) -> Result<()>;
```

The bridge is intentionally transport-agnostic. There are three
implementations you might choose:

### Option A: ROS 2 + ros\_gz\_bridge (recommended)

The most-trodden path. Gazebo runs under ROS 2, you publish
`sensor_msgs/JointState` on `/joint_states` from the Rust side
(via [`r2r`](https://github.com/sequenceplanner/r2r) or
[`rclrs`](https://github.com/ros2-rust/ros2_rust)), and the
`ign_ros2_control` plugin inside the Gazebo SDF applies it to the
URDF-defined joints.

Setup:

```bash
# install ROS 2 Humble + Gazebo Garden, then
sudo apt install ros-humble-ros-gz ros-humble-ros2-control
# in your workspace
ros2 launch robotics_platform_bringup sim.launch.py
```

You'll need a small `sim.launch.py` that:

1. Launches `gz sim -r robotics-platform/gazebo/world.sdf`
2. Spawns the URDF (`assets/robot_arm.urdf`) via
   `ros_gz_sim create -file …`
3. Starts the `joint_state_broadcaster` + `joint_trajectory_controller`
4. Launches `ros_gz_bridge` mapping ROS topics ↔ gz topics.

### Option B: Native gz-transport via FFI

Lower latency, no ROS layer. There's no Rust binding to
`gz-transport` today, so you'd need to write a small C wrapper and
FFI to it. Suitable when you have a hard latency budget and want to
keep your dependency tree clean.

### Option C: Custom Gazebo system plugin

You write a C++ class deriving from `gz::sim::System`, load it into
the world via `<plugin filename="…">`, and have it speak to the Rust
process over a Unix domain socket or shared memory. The most
flexible option, the most code.

### What ships now

`NullBridge`. It's the safe default — the in-process sim runs
without any of the above installed, the tests pass, and CI is happy.
Wire in a real bridge when you actually need Gazebo's visualization
or physics.

## SDF world

`gazebo/world.sdf` defines the demo scene: ground plane, two cubes at
the same positions the in-process sim uses. To launch:

```bash
gz sim -r robotics-platform/gazebo/world.sdf
```

You'll see the cubes; the arm is loaded separately (see Option A
above).

## Why both

The pattern I've seen work, both in industry and in robotics research
labs:

* **In-process sim runs in CI.** Every PR gets a pick-and-place
  smoke test. 200 ms wall-clock, no Docker, no GPU.
* **Gazebo runs on the developer's machine.** When you need to *see*
  what happened — when the bug is in the motion timing, not in the
  IK math — Gazebo is invaluable.
* **The real arm runs on the Pi.** Once both simulators agree, the
  transfer to hardware is mechanical.

Skipping the in-process sim means every iteration takes 30 s of
Gazebo startup. Skipping Gazebo means you can't debug visual bugs.
Pay the small cost of building both, save the larger cost of
debugging without them.
