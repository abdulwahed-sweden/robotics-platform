# robotics-platform

A Rust robotic arm. Same code runs in simulation or on real Raspberry Pi servos.

[![Sponsor](https://img.shields.io/badge/Sponsor-%E2%9D%A4-db61a2?logo=githubsponsors&logoColor=white)](https://github.com/sponsors/abdulwahed-sweden)

> If this robotics work is useful to you, you can [sponsor continued open-source development](https://github.com/sponsors/abdulwahed-sweden).

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
- [roadmap](docs/roadmap.md) — near-term plan
- [ambitions](docs/ambitions.md) — long-term vision and the Rust ecosystem we'd build on

## Stack

Rust stable. `tokio`, `nalgebra`, `serde`, `clap`, `tracing`, `rppal`.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). Notable changes are tracked in
[CHANGELOG.md](CHANGELOG.md).

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache 2.0](LICENSE-APACHE), at your option.
