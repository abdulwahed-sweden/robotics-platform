# Changelog

All notable changes to this project will be documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Interactive 3D web viewer (`cargo run --example render -p robotics-cli`)
  with three modes: recorded **Playback** (play/pause + scrubber + speed),
  **Joints** (5 sliders driving the arm directly), and **IK** (X/Y/Z +
  approach pitch sliders solving in JS against the same analytic IK as
  the Rust crate).
- ASCII top-down watcher (`cargo run --example watch -p robotics-cli`)
  for headless terminals.
- `LICENSE-MIT` and `LICENSE-APACHE` files matching the workspace's
  declared dual license.
- `CONTRIBUTING.md` with setup, the crate-to-feature map, style and PR
  rules, and a safety-change escalation note.

### Changed
- Trimmed `README.md` to a one-screen quick reference.

## [0.1.0] — 2026-05-22

Initial public release.

### Added
- Cargo workspace with ten crates: `core`, `kinematics`, `motion`,
  `simulation`, `hardware`, `gpio`, `planner`, `protocols`, `vision`,
  `cli`.
- Single trait surface (`Backend`, `RobotArm`, `Gripper`, `Sensor`)
  shared by simulation and hardware backends.
- Analytic FK and IK for a 5-DOF anthropomorphic arm, with a
  `FK(IK(p)) ≈ p` round-trip test.
- Motion crate with linear / cubic / quintic easing and
  velocity-bounded time scaling.
- In-process kinematic simulator at 200 Hz with object attach/detach.
- Hardware backend driving servos via the `gpio` crate. Linux-gated
  on `rppal`; falls back to a logging stub on macOS/Windows so the
  workspace builds everywhere.
- Explicit state machine (Idle / Targeting / Moving / Picking /
  Carrying / Placing / Error / EmergencyStop) with documented
  transitions and tests.
- Procedural pick-and-place task.
- `clap`-driven CLI with `simulate`, `hardware`, `calibrate`,
  `move`, `pick`, and `place` subcommands.
- TOML configs for arm geometry, simulation scene, and hardware
  wiring.
- URDF model and Gazebo SDF world.
- `GazeboBridge` trait + `NullBridge` (real bridge requires ROS 2
  or a custom plugin; see `docs/simulation.md`).
- `ObjectDetector` trait + `VirtualWorldDetector` reading from the
  simulator.
- Documentation: architecture, kinematics math, simulation, hardware
  setup, GPIO/PWM, safety contract, roadmap.
- 17 unit tests across the workspace.

[Unreleased]: https://github.com/abdulwahed-sweden/robotics-platform/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/abdulwahed-sweden/robotics-platform/releases/tag/v0.1.0
