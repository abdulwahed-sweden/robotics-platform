# Architecture

This document explains *why* the platform is split the way it is —
where each boundary sits and what would have happened if we'd put it
somewhere else.

## The single big idea: one trait surface, two backends

The platform is organized around a single principle: the high-level
logic — IK, motion planning, the state machine, the CLI — has zero
direct knowledge of whether it is driving real hardware or a
simulation. Every higher layer talks to `robotics_core::Backend` and
to nothing more concrete. Both `SimulationBackend` and
`HardwareBackend` implement that trait.

This is the same architecture you find in industrial robot
controllers (Fanuc, KUKA, ABB) and in autonomous-vehicle stacks: a
device-agnostic "robot abstraction layer" that lets you simulate
everything, run on the real machine, and switch between them in one
config flag.

The alternative — letting the planner know whether it's talking to a
servo or a simulated rigid body — is the classic trap of "the code
that runs in simulation isn't the code that runs in production." We
explicitly do not do that.

## Layers

```text
cli           ─── humans                       │ no robot logic here
planner       ─── tasks + state machine        │ no I/O, no motion math
motion        ─── trajectories, easing         │ no IK, no actuator
kinematics    ─── FK, IK                       │ pure math, no async
core          ─── trait surface + types        │ no implementations
─── trait boundary ───────────────────────────
simulation    ─── kinematic sim (+ Gazebo)
hardware      ─── PWM-driven servos
gpio          ─── Pi I/O (rppal)
```

Above the boundary: stateless math and orchestration. Below it: I/O
and side effects. The boundary is what makes the upper half
unit-testable on a laptop with no robot in sight.

## Why these specific crates

* **core** is *only* traits, types, errors. It has no
  implementations and few dependencies (serde, nalgebra, thiserror,
  async-trait). Every other crate depends on it; it depends on
  nothing internal. This keeps the contract surface stable and
  versionable.

* **kinematics** is intentionally pure functions over `&ArmModel` and
  `&JointState`. Same input → same output, no `Future`, no async. It
  is the easiest crate in the workspace to test and the easiest to
  replace if you outgrow the analytic solver.

* **motion** depends on kinematics (it uses `inverse_kinematics`) but
  not on any backend. It produces `JointTrajectory` values and hands
  them off. The execution loop is the backend's job.

* **simulation** and **hardware** are sibling crates implementing the
  same traits. They share no code, by design — sharing would have
  let simulation-only assumptions leak into hardware. If something
  genuinely is shared (e.g. the velocity-limited integrator that
  both `SimWorld::tick` and `HardwareBackend::start` use), it lives
  inline; we pull it into `core` only when there are at least three
  consumers.

* **gpio** is small but separate so the Linux-only dependency on
  rppal lives in exactly one crate. Workspace builds on macOS via
  the stub PWM in this crate; nothing else has to know.

* **planner** is the top of the robot half of the stack. It owns the
  state machine (the only place transitions are validated) and a
  procedural pick-and-place task. Replace this crate with a
  BehaviorTree.CPP wrapper or a MoveIt bridge later; nothing below
  it has to change.

* **cli** is the only binary. It exists to (a) load configs, (b)
  pick a backend, (c) hand off to the planner. No business logic
  lives in main.

* **protocols** is currently small but pays its rent by giving
  remote bridges (websocket, MQTT, gRPC) a versioned schema to talk
  to. The serde tags are stable contracts.

* **vision** is a small trait with one implementation — but the
  point is the trait. Adding OpenCV blob detection later is a new
  impl in this crate; consumers never know.

## Why async, and why tokio

Robot I/O is fundamentally event-driven: a PWM heartbeat, a sensor
poll, a network bridge. Threads-and-locks works but `async` lets us
write the control loop, the telemetry channel, and (later) the
WebSocket dashboard in the same style. Tokio is the default Rust
async runtime; sticking to it means our code composes with everything
in the broader ecosystem (axum for telemetry, reqwest for OTA
updates, etc.).

The kinematics and motion crates are deliberately *not* async — the
math has no I/O. Keeping them sync also makes them callable from
sync contexts (a ROS 2 callback, a C FFI wrapper) without an
executor.

## Where determinism comes from

* The kinematic simulator uses a fixed tick rate (200 Hz) with
  tokio's `interval` and `MissedTickBehavior::Delay` — no
  busy-wait, no random `sleep`, no wall-clock dependence inside the
  integrator.
* Trajectories are pure functions of `(start, end, duration, easing)`
  — `sample(t)` gives the same answer every time.
* The state machine is fully explicit; transitions are rejected
  rather than silently accepted.
* The IK solver is analytic, not iterative — no convergence loop, no
  random seeds.

When the day comes that you need to record-and-replay execution for
regression testing, those four properties are what you need.

## Extension points

* **A new joint** — add a variant to `JointId` and rows in
  `JointState` and `ArmModel`. The compiler tells you every place
  you need to update.
* **A different IK** — add a function in `kinematics::ik` and have
  `MotionPlanner` switch on a config flag.
* **A new backend** — implement `Backend`, `RobotArm`, `Gripper`.
  Drop into the CLI dispatcher.
* **A new task** — add a struct in `planner`. Reuse `MotionPlanner`
  and the state machine.
* **Remote control** — depend on `protocols`, parse `Command`,
  feed it to the same planner.
