# robotics-platform

A foundation for a serious Rust robotics platform. The same motion
code runs against an in-process simulator and against real Raspberry
Pi-driven servos. Both share a single trait surface; everything above
that surface (kinematics, motion planning, state machine, CLI) is
backend-agnostic.

This is not a tutorial project. It is the starting point of a robotics
operating system, intentionally small, deliberately structured,
designed to grow.

## Architecture

```text
   ┌──────────────────────────────────────────────────────┐
   │ cli            high-level user entry point           │
   ├──────────────────────────────────────────────────────┤
   │ planner        task + state machine                  │
   ├──────────────────────────────────────────────────────┤
   │ motion         trajectories, easing, queues          │
   ├──────────────────────────────────────────────────────┤
   │ kinematics     FK / IK (analytic, with nalgebra)     │
   ├──────────────────────────────────────────────────────┤
   │ core           traits, errors, joint types           │
   ├──────────────────────────────────────────────────────┤
   │ simulation │ hardware  (backends implement core API) │
   ├──────────────────────────────────────────────────────┤
   │ gpio          rppal wrapper (Linux-gated, stubbed elsewhere) │
   └──────────────────────────────────────────────────────┘
```

Auxiliary crates:

* **protocols** — serde-defined wire format for future websocket /
  MQTT bridges.
* **vision** — `ObjectDetector` trait, ships a virtual-world detector,
  designed for OpenCV/ML extension.

See [docs/architecture.md](docs/architecture.md) for the rationale
behind each boundary.

## Quick start

```bash
# Run the in-process simulator with the pick-and-place demo.
cargo run --example pick_and_place -p robotics-cli

# Or via the CLI.
cargo run -p robotics-cli -- simulate

# Move to (10cm, 0cm, 10cm) above the base.
cargo run -p robotics-cli -- move --x 0.10 --y 0.0 --z 0.10
```

On a Raspberry Pi:

```bash
cargo run --release -p robotics-cli -- hardware
```

(Non-Linux machines fall back to a stub PWM that logs commands
instead of executing them, so the workspace builds and exercises
everywhere.)

## Crates

| Crate | Purpose |
|-------|---------|
| `robotics-core` | Trait surface (`Backend`, `RobotArm`, `Gripper`, …), shared types, errors. |
| `robotics-kinematics` | FK and analytic IK for the 5-DOF anthropomorphic arm. |
| `robotics-motion` | Trajectories, easing curves, velocity-bounded time scaling. |
| `robotics-simulation` | Kinematic simulator + Gazebo bridge skeleton. |
| `robotics-hardware` | Servo-driven hardware backend over the gpio crate. |
| `robotics-gpio` | Pi PWM (`rppal` on Linux, stub elsewhere). |
| `robotics-planner` | Explicit `RobotState` machine + pick-and-place task. |
| `robotics-protocols` | Serde-defined wire types. |
| `robotics-vision` | `ObjectDetector` trait + virtual-world detector. |
| `robotics-cli` | `clap`-driven CLI entry point. |

## Docs

* [architecture.md](docs/architecture.md) — boundaries, why they're
  where they are.
* [kinematics.md](docs/kinematics.md) — the FK and IK math.
* [simulation.md](docs/simulation.md) — in-process sim and Gazebo
  bridge.
* [hardware.md](docs/hardware.md) — wiring, calibration, the Pi setup.
* [gpio.md](docs/gpio.md) — PWM, duty cycles, software vs hardware PWM.
* [safety.md](docs/safety.md) — limits, e-stop, the safety contract.
* [roadmap.md](docs/roadmap.md) — what's next.

## Tech stack

Rust stable. `tokio`, `nalgebra`, `serde`, `tracing`, `anyhow` /
`thiserror`, `clap`, `rppal` (Linux only). No Python anywhere in the
control path.

## Status

v0.1. Compiles, tests pass, in-process pick-and-place demo runs.
The Gazebo bridge is structural (defines the seam, ships a `NullBridge`);
wire it to ROS 2 + ros\_gz\_bridge or to a custom system plugin per
`docs/simulation.md`.
