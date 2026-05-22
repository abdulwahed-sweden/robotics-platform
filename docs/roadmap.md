# Roadmap

What's here today, what's ready to grow, what's deliberately out of
scope.

## Today (v0.1)

* Full workspace, all 10 crates compile cleanly on macOS and Linux.
* Analytic FK + IK with `FK(IK(p)) ≈ p` round-trip test.
* Motion planner with linear / cubic / quintic easing and
  velocity-bounded time scaling.
* In-process kinematic simulator at 200 Hz.
* Hardware backend (Linux/Pi only) over `rppal`, stub on other OSes.
* Explicit state machine with full transition tests.
* Pick-and-place demo runnable end-to-end against simulation.
* CLI with simulate / hardware / calibrate / move / pick / place
  subcommands.
* TOML configs for arm geometry, simulation scene, hardware wiring.
* URDF for visualization and Gazebo loading.
* SDF world with demo cubes.

## Designed for, not yet implemented

These are extension points the architecture is shaped around — most
are one or two days of work once you actually need them.

### Gazebo bridge

The seam exists (`gazebo::GazeboBridge` trait + `NullBridge`). The
production path is ROS 2 + `ros_gz_bridge` (see `docs/simulation.md`).
Other options: native `gz-transport` via FFI, or a custom C++ system
plugin.

### Vision

`ObjectDetector` trait + `VirtualWorldDetector` ship. To upgrade:

* **OpenCV blob detection** — pull in `opencv-rust`, add a
  `CameraDetector` impl, calibrate camera→base extrinsics, project
  detections onto the table plane.
* **YOLO / Detectron2** — `tract` or `ort` for ONNX inference. Same
  trait, different impl.
* **Depth sensors** (RealSense / Kinect / Orbbec) — point clouds,
  cluster segmentation, base-frame coordinates.

### Higher-fidelity hardware

The current backend is open-loop SG90. Three obvious upgrades:

* **PCA9685 over I²C** — 16 channels of hardware-clocked PWM, no
  more software-PWM jitter. New `Pwm` impl, ~200 LOC.
* **Smart servos** (Dynamixel XL-330, FeeTech STS3215) — real position
  feedback, current sensing, daisy-chain serial. Replace the gpio
  crate with a serial driver.
* **AS5048 magnetic encoders + brushless drives** — research-grade
  hardware. The trait surface stays the same.

### Telemetry & remote control

`protocols` defines the wire types. To plug in:

* **WebSocket dashboard** — `axum` + `tokio-tungstenite`, stream
  `TelemetryFrame` on one socket, accept `Command` on another.
* **MQTT** — `rumqttc` on the same types.
* **gRPC** — `tonic` with the protobufs generated from the same
  type definitions (or just send JSON if you don't need codegen).

### More tasks

`planner::pick_place` is procedural. For branching/parallel/recovery
behaviors, swap in:

* **Behavior trees** — `bonsai-bt` or a custom implementation.
* **Hierarchical FSM** — `rust-fsm` if you want to keep the explicit-
  transition philosophy.
* **PDDL planner** — beyond a single robot, when you need symbolic
  planning. `unified-planning-rs` is a research-grade option.

### Multi-arm

`Backend` is one arm. For a workcell:

* Each arm gets its own backend instance.
* A coordinator crate owns N backends and a shared scene model.
* Trajectory deconfliction (no two arms in the same volume at the
  same time) lives in the coordinator.

This is mostly orchestration code — the per-arm primitives already
exist.

### ROS 2 native

The platform doesn't depend on ROS 2 today, by choice — most
robotics projects don't need it. But the trait shapes match ROS 2
conventions (JointState is the same struct, e-stop is universal,
poses are REP-103). Wrapping the `Backend` impls in `r2r` or `rclrs`
publishers is straightforward.

## Out of scope (probably forever)

* **Full physics simulation in-process.** Rapier is fine but
  Gazebo is the right tool for the job, and we already have the
  seam.
* **Closed-source dependencies.** Everything in the workspace is
  Apache/MIT, including the future PCA9685 and Dynamixel drivers
  when they're written.
* **GPU-accelerated motion planning** (CHOMP, STOMP, etc.). The
  analytic solver covers what this hardware can express; bigger
  problems → MoveIt 2 via ROS bridge.

## What I'd build next

In order:

1. **Real Gazebo bridge** via ROS 2. Lets the demo run with
   visualization.
2. **PCA9685 driver.** Removes the software-PWM jitter that
   limits hardware-mode reliability today.
3. **OpenCV-based virtual color cube detector.** Lets pick-and-place
   work from a camera in sim and on hardware.
4. **WebSocket telemetry + a minimal web dashboard.** Live state
   visible from any browser; needed for remote operation.
5. **Behavior tree task runner.** When you outgrow the procedural
   pick-and-place.
