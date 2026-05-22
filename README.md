# robotics-platform

A Rust robotic arm. Same code runs in simulation or on real Raspberry Pi servos.

## Run

```bash
# Pick-and-place demo in the in-process simulator
cargo run -p robotics-cli -- simulate

# Move the end-effector to (x, y, z) in meters
cargo run -p robotics-cli -- move --x 0.10 --y 0.0 --z 0.10

# Open the 3D viewer in your browser
cargo run --example render -p robotics-cli
```

On a Raspberry Pi: `cargo run --release -p robotics-cli -- hardware`.

## Crates

```
core         traits + types (Backend, RobotArm, Gripper)
kinematics   analytic FK / IK
motion       trajectories + easing
planner      state machine + pick-and-place task
simulation   software backend
hardware     Pi backend (PWM via rppal)
gpio         PWM wrapper (Linux only; stub elsewhere)
cli          binary entry point
```

## Docs

- [architecture](docs/architecture.md)
- [kinematics](docs/kinematics.md) — the math
- [hardware](docs/hardware.md) — wiring + calibration
- [safety](docs/safety.md)
- [roadmap](docs/roadmap.md)

## Stack

Rust stable. `tokio`, `nalgebra`, `serde`, `clap`, `tracing`, `rppal`.

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache 2.0](LICENSE-APACHE), at your option.
